"""Boat course dynamics simulator + closed-loop test harness.

The plant is the **first-order Nomoto model**, the standard tool for autopilot
design:

    T * r_dot + r = K * delta        r = yaw rate, delta = rudder angle
    psi_dot = r                       psi = heading

On top of the bare model we add the disturbances that actually make a real
pilot hunt, so a controller that looks smooth here has a fighting chance at sea:

* **wind / current** as a steady yaw bias (tests the integral term),
* **waves** as a sinusoidal yaw disturbance (tests that we *don't* chase every
  wave - the deadband and filtering earn their keep here),
* **compass noise** as gaussian jitter on the reported heading,
* finite **sensor update rate** distinct from the control rate.

:func:`run_closed_loop` drives a :class:`~autopilot.controller.CourseKeeper`
against the model and :func:`evaluate` scores the run for the very things we're
trying to prevent: overshoot, settling time, steady-state error, and - crucially
for "hunting" and actuator wear - the number of rudder reversals.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from .controller import CourseKeeper
from .heading import heading_error, normalize_360
from .ram import RamActuator


class _Rng:
    """Tiny deterministic PRNG (stdlib ``random`` avoided for reproducible tests
    without touching global state). Gaussian via Box-Muller."""

    def __init__(self, seed: int = 1):
        self.state = seed & 0xFFFFFFFF or 1

    def _next(self) -> float:
        # xorshift32
        x = self.state
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        self.state = x & 0xFFFFFFFF
        return self.state / 0xFFFFFFFF

    def gauss(self, sigma: float) -> float:
        if sigma <= 0:
            return 0.0
        u1 = max(self._next(), 1e-9)
        u2 = self._next()
        return sigma * math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


@dataclass
class BoatModel:
    """First-order Nomoto steering model with a rudder actuator.

    The rudder angle applied to the hull comes from the *ram*, not directly from
    the controller command, so ram speed and end stops are part of the loop.
    """

    K: float = 0.15                  # yaw-rate gain (1/s)
    T: float = 3.0                   # time constant (s)
    heading: float = 0.0             # deg
    yaw_rate_dps: float = 0.0        # deg/s
    speed_knots: float = 6.0

    def step(self, rudder_deg: float, dt: float, yaw_disturbance_dps2: float = 0.0) -> None:
        """Integrate one time step (semi-implicit Euler for stability).

        ``yaw_disturbance_dps2`` is an external yaw acceleration (deg/s^2) from
        wind/waves/current, injected into the rate equation.
        """
        # r_dot = (K*delta - r)/T + disturbance
        r_dot = (self.K * rudder_deg - self.yaw_rate_dps) / self.T + yaw_disturbance_dps2
        self.yaw_rate_dps += r_dot * dt
        self.heading = normalize_360(self.heading + self.yaw_rate_dps * dt)


@dataclass
class Disturbances:
    """Environmental and sensor disturbances for a scenario."""

    steady_yaw_dps2: float = 0.0         # constant bias (wind/current), deg/s^2
    wave_amplitude_dps2: float = 0.0     # sinusoidal yaw accel amplitude
    wave_period_s: float = 6.0           # wave encounter period
    heading_noise_deg: float = 0.0       # gaussian compass jitter (std dev)
    seed: int = 1


@dataclass
class Scenario:
    """A repeatable test case."""

    name: str
    initial_heading: float = 0.0
    desired_heading: float = 0.0
    duration_s: float = 120.0
    setpoint_changes: List[Tuple[float, float]] = field(default_factory=list)  # (t, new heading)
    disturbances: Disturbances = field(default_factory=Disturbances)
    sensor_update_hz: float = 10.0

    # Pass/fail budget for this scenario. Disturbance scenarios legitimately
    # work the helm harder (a boat in waves reverses ~once per wave), so the
    # reversal budget is per-scenario rather than one global number.
    max_overshoot: float = 6.0                 # deg past target after a change
    max_steady_error: float = 3.0              # deg, abs, steady-state
    max_reversals_per_min: float = 12.0        # helm reversals >= 0.25deg
    max_ram_travel_per_min: Optional[float] = None  # deg of ram movement / min


@dataclass
class RunLog:
    """Time series recorded during a closed-loop run."""

    t: List[float] = field(default_factory=list)
    heading: List[float] = field(default_factory=list)
    desired: List[float] = field(default_factory=list)
    error: List[float] = field(default_factory=list)
    rudder_cmd: List[float] = field(default_factory=list)
    rudder_actual: List[float] = field(default_factory=list)
    yaw_rate: List[float] = field(default_factory=list)


def run_closed_loop(
    keeper: CourseKeeper,
    boat: BoatModel,
    ram: RamActuator,
    scenario: Scenario,
    control_hz: float = 10.0,
) -> RunLog:
    """Run ``keeper`` against ``boat`` for a scenario and return the log.

    The controller and sensor tick independently: the boat integrates at the
    control rate, but the controller only *sees* a fresh (noisy) heading at
    ``scenario.sensor_update_hz`` - the same sample-and-hold a real NMEA feed
    imposes.
    """
    dt = 1.0 / control_hz
    boat.heading = normalize_360(scenario.initial_heading)
    boat.speed_knots = boat.speed_knots
    rng = _Rng(scenario.disturbances.seed)
    dist = scenario.disturbances

    keeper.engage(scenario.desired_heading, current_heading=boat.heading)
    changes = sorted(scenario.setpoint_changes)

    log = RunLog()
    sensor_dt = 1.0 / max(scenario.sensor_update_hz, 1e-6)
    last_sensor_t = -1e9
    measured = boat.heading
    steps = int(round(scenario.duration_s * control_hz))

    for i in range(steps):
        t = i * dt

        # Apply any scheduled setpoint change.
        while changes and t >= changes[0][0]:
            keeper.set_heading(changes[0][1])
            changes.pop(0)

        # Sensor sample-and-hold with compass noise.
        if t - last_sensor_t >= sensor_dt:
            measured = normalize_360(boat.heading + rng.gauss(dist.heading_noise_deg))
            last_sensor_t = t

        tick = keeper.update(measured, dt, speed_knots=boat.speed_knots)

        # Environmental yaw disturbance for this instant.
        wave = 0.0
        if dist.wave_amplitude_dps2 != 0.0 and dist.wave_period_s > 0:
            wave = dist.wave_amplitude_dps2 * math.sin(2.0 * math.pi * t / dist.wave_period_s)
        disturbance = dist.steady_yaw_dps2 + wave

        boat.step(ram.rudder_angle, dt, yaw_disturbance_dps2=disturbance)

        log.t.append(t)
        log.heading.append(boat.heading)
        log.desired.append(tick.desired_heading)
        log.error.append(heading_error(tick.desired_heading, boat.heading))
        log.rudder_cmd.append(tick.rudder_command)
        log.rudder_actual.append(ram.rudder_angle)
        log.yaw_rate.append(boat.yaw_rate_dps)

    return log


def _count_reversals(series: List[float], hysteresis: float) -> int:
    """Count direction reversals of at least ``hysteresis`` magnitude.

    A peak-picking counter with a dead-zone: small back-and-forth movement
    below ``hysteresis`` is ignored, so this reflects real helm reversals (the
    thing a sailor feels as "hunting") rather than integration noise.
    """
    if len(series) < 2:
        return 0
    reversals = 0
    direction = 0          # +1 rising, -1 falling, 0 undecided
    extreme = series[0]
    for v in series[1:]:
        if direction <= 0 and v >= extreme + hysteresis:
            if direction == -1:
                reversals += 1
            direction = 1
            extreme = v
        elif direction >= 0 and v <= extreme - hysteresis:
            if direction == 1:
                reversals += 1
            direction = -1
            extreme = v
        else:  # extend the current run's extreme
            if direction >= 0 and v > extreme:
                extreme = v
            elif direction <= 0 and v < extreme:
                extreme = v
    return reversals


@dataclass
class Metrics:
    """Scored performance of a run - the numbers that define 'smooth'."""

    rms_error: float                 # steady-state RMS heading error (deg)
    max_overshoot: float             # worst overshoot past target after a change (deg)
    settling_time: Optional[float]   # time to stay within tolerance (s)
    steady_state_error: float        # mean error over the last quarter (deg)
    rudder_reversals: int            # direction changes of the ram -> hunting proxy
    rudder_travel: float             # total ram movement (deg) -> wear/energy proxy
    max_abs_error: float

    def summary(self) -> str:
        st = f"{self.settling_time:.1f}s" if self.settling_time is not None else "not settled"
        return (
            f"RMS err {self.rms_error:.2f}deg | overshoot {self.max_overshoot:.2f}deg | "
            f"settling {st} | steady err {self.steady_state_error:+.2f}deg | "
            f"reversals {self.rudder_reversals} | ram travel {self.rudder_travel:.0f}deg | "
            f"max err {self.max_abs_error:.2f}deg"
        )


def evaluate(log: RunLog, tolerance_deg: float = 2.0,
             settle_from: str = "last_change") -> Metrics:
    """Score a run for the things we're trying to prevent.

    * **overshoot** and **settling time** measure the transient after the final
      setpoint change (the classic step response),
    * **rudder_reversals** counts direction changes of the actuator - a direct
      proxy for hunting and helm chatter,
    * **rudder_travel** totals ram movement - a proxy for actuator wear and
      power draw.
    """
    n = len(log.t)
    if n == 0:
        raise ValueError("empty run log")

    # Reference the transient to the last setpoint change (or start).
    change_idx = 0
    if settle_from == "last_change":
        for i in range(1, n):
            if log.desired[i] != log.desired[i - 1]:
                change_idx = i
    t0 = log.t[change_idx]
    target = log.desired[-1]

    # Settling: last time the error left the tolerance band, after the change.
    settling_time: Optional[float] = None
    last_out = None
    for i in range(change_idx, n):
        if abs(log.error[i]) > tolerance_deg:
            last_out = log.t[i]
    if last_out is None:
        settling_time = 0.0
    elif last_out < log.t[-1]:
        settling_time = last_out - t0

    # Overshoot: how far past the target we swung, in the direction of approach.
    approach_sign = 1.0
    if change_idx > 0:
        approach_sign = math.copysign(1.0, heading_error(target, log.heading[change_idx]))
    max_overshoot = 0.0
    for i in range(change_idx, n):
        past = -approach_sign * log.error[i]  # positive once we cross the target
        if past > max_overshoot:
            max_overshoot = past

    # Steady-state stats over the final quarter of the run.
    tail_start = change_idx + (n - change_idx) * 3 // 4
    tail = log.error[tail_start:] or log.error[-1:]
    rms = math.sqrt(sum(e * e for e in tail) / len(tail))
    steady = sum(tail) / len(tail)

    # Actuator activity from the *actual* rudder trace. Reversals are counted
    # with hysteresis so sub-degree numerical dither doesn't masquerade as
    # hunting - only a genuine helm swing of at least ``reversal_hysteresis``
    # degrees in the opposite direction counts.
    travel = sum(abs(log.rudder_actual[i] - log.rudder_actual[i - 1]) for i in range(1, n))
    reversals = _count_reversals(log.rudder_actual, hysteresis=0.25)

    return Metrics(
        rms_error=rms,
        max_overshoot=max(0.0, max_overshoot),
        settling_time=settling_time,
        steady_state_error=steady,
        rudder_reversals=reversals,
        rudder_travel=travel,
        max_abs_error=max(abs(e) for e in log.error),
    )
