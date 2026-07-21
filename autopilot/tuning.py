"""Deriving well-damped PID gains instead of guessing them.

Hand-tuning a marine autopilot by twiddling knobs at sea is slow and often ends
in a boat that hunts. Because the boat's yaw behaves (to first order) like the
**Nomoto model**

    T * r_dot + r = K * delta          (r = yaw rate, delta = rudder angle)

we can *place the closed-loop poles* and get gains that are damped by
construction. With proportional + counter-rudder (PD on heading, derivative on
yaw rate) the closed loop is a standard second-order system:

    T*e'' + (1 + K*Kd)*e' + K*Kp*e = 0

Matching e'' + 2*zeta*wn*e' + wn^2*e = 0 gives closed forms:

    Kp = wn^2 * T / K
    Kd = (2*zeta*wn*T - 1) / K

You choose two intuitive numbers:

* ``zeta`` - damping ratio. **0.9-1.0 gives no overshoot and no hunting**;
  below ~0.6 the boat starts to weave. This is the "how smooth" dial.
* ``wn``  - natural frequency (rad/s), i.e. how briskly it pulls back onto
  course. Keep it well under the ram's bandwidth or you'll saturate the
  actuator; ``settling_time`` is an easier way to ask for the same thing.

The integral gain is added conservatively (it only exists to trim steady
offsets from wind/current) as a small fraction of the dominant pole, which keeps
it from eroding the phase margin the PD terms just bought us.

K and T are found from a simple dockside/sea-trial manoeuvre (a turning circle
or Bech spiral); :func:`estimate_nomoto_from_step` recovers them from a rudder
step response if you'd rather let the boat tell you.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .pid import PIDGains


@dataclass
class NomotoParams:
    """First-order Nomoto steering parameters.

    :param K: yaw-rate gain (per second) - steady yaw rate = K * rudder.
    :param T: time constant (seconds) - larger = more sluggish / more inertia.
    """

    K: float
    T: float

    def scaled_to_speed(self, speed: float, ref_speed: float) -> "NomotoParams":
        """Scale K, T from the reference speed to another boat speed.

        Rudder force ~ speed^2 while yaw rate ~ speed, so to first order
        ``K ~ speed`` and ``T ~ 1/speed``. Used for speed-scheduled gains.
        """
        if ref_speed <= 0 or speed <= 0:
            return NomotoParams(self.K, self.T)
        ratio = speed / ref_speed
        return NomotoParams(K=self.K * ratio, T=self.T / ratio)


def nomoto_pid_gains(
    params: NomotoParams,
    damping_ratio: float = 0.9,
    natural_frequency: Optional[float] = None,
    settling_time: Optional[float] = None,
    integral_fraction: float = 0.1,
) -> PIDGains:
    """Pole-placement PID gains for a first-order Nomoto boat.

    Provide *either* ``natural_frequency`` (rad/s) or ``settling_time`` (s, the
    2% settling time of the target second-order response); ``settling_time`` is
    usually the more natural thing to specify. ``damping_ratio`` of ~0.9 gives a
    fast response with negligible overshoot - the sweet spot for no hunting.

    :param integral_fraction: integral aggressiveness as a fraction of the
        dominant pole. 0 disables integral action; ~0.1 gently trims steady
        wind/current offset without hurting damping.
    """
    K, T = params.K, params.T
    if K == 0 or T <= 0:
        raise ValueError("Nomoto K must be non-zero and T positive")
    zeta = damping_ratio

    if natural_frequency is not None:
        wn = natural_frequency
    elif settling_time is not None:
        # 2% settling time of a 2nd-order system ~ 4 / (zeta * wn).
        if settling_time <= 0:
            raise ValueError("settling_time must be positive")
        wn = 4.0 / (zeta * settling_time)
    else:
        # Default: a natural frequency a comfortable margin inside the plant's
        # own time constant, so we don't demand more than the hull can give.
        wn = 1.0 / T

    kp = wn * wn * T / K
    kd = (2.0 * zeta * wn * T - 1.0) / K

    # Counter rudder can't be negative; if the plant is already over-damped for
    # this bandwidth, no counter rudder is needed.
    if kd < 0:
        kd = 0.0

    # Integral: place a slow real pole at integral_fraction * (zeta*wn).
    ki = 0.0
    if integral_fraction > 0:
        ki = integral_fraction * (zeta * wn) * kp

    # Gains are defined for "rudder such that +rudder -> +yaw rate". If the
    # boat's K is negative (rudder sense reversed), flip the proportional/
    # integral sense; the ram's mount-side sign handles the wiring separately,
    # so keep control-law sign consistent with a positive effective K.
    if K < 0:
        kp, ki = -kp, -ki
        # kd already uses |effect|; recompute with |K| to stay positive.
        kd = max(0.0, (2.0 * zeta * wn * T - 1.0) / abs(K))

    return PIDGains(kp=kp, ki=ki, kd=kd)


def estimate_nomoto_from_step(
    times: Sequence[float],
    yaw_rates: Sequence[float],
    rudder_deg: float,
) -> NomotoParams:
    """Recover K and T from a rudder-step yaw-rate response.

    Hold the boat steady, put the rudder to ``rudder_deg`` and hold it, and log
    time vs yaw rate. The response is r(t) = K*delta*(1 - exp(-t/T)):

    * **K** from the steady-state rate: K = r_final / delta.
    * **T** as the time to reach 63.2% of that steady state.

    A quick, robust estimate - good enough to seed :func:`nomoto_pid_gains`,
    after which the sim (or sea trial) confirms the damping.
    """
    if len(times) != len(yaw_rates) or len(times) < 3:
        raise ValueError("need matching, non-trivial time/rate series")
    if rudder_deg == 0:
        raise ValueError("rudder_deg must be non-zero")

    # Steady state = mean of the last ~20% of samples.
    tail = max(1, len(yaw_rates) // 5)
    r_final = sum(yaw_rates[-tail:]) / tail
    K = r_final / rudder_deg

    target = 0.632 * r_final
    T: Optional[float] = None
    t0 = times[0]
    for t, r in zip(times, yaw_rates):
        if (r_final >= 0 and r >= target) or (r_final < 0 and r <= target):
            T = t - t0
            break
    if T is None or T <= 0:
        T = (times[-1] - times[0]) / 3.0  # fallback: crude but positive
    return NomotoParams(K=K, T=T)


def gain_schedule(
    base_params: NomotoParams,
    ref_speed: float,
    speeds: Sequence[float],
    **tune_kwargs,
) -> List[Tuple[float, PIDGains]]:
    """Pre-compute (speed, gains) pairs for speed-scheduled control.

    Rudder authority falls off fast at low speed, so a single gain set either
    hunts at hull speed or is limp when drifting. The controller interpolates
    over this table.
    """
    schedule = []
    for v in speeds:
        p = base_params.scaled_to_speed(v, ref_speed)
        schedule.append((v, nomoto_pid_gains(p, **tune_kwargs)))
    return schedule
