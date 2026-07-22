"""Heading PID with the damping features a marine autopilot actually needs.

A textbook PID will make a boat *hunt* - weave back and forth across the
course, sawing the tiller. The extra machinery here is exactly what commercial
pilots (Raymarine, B&G, CPT...) add to stop that:

* **Counter rudder = derivative on measurement.** The D term acts on the boat's
  rate of turn, not on the error. As the bow swings toward the target it feeds
  in *opposite* rudder to arrest the swing, killing overshoot. Acting on
  measurement (not error) also avoids a "derivative kick" when you dial in a new
  heading.
* **Deadband.** A few degrees of "don't care" so small wander doesn't chatter
  the ram. Implemented as a *soft* deadband (subtract the band) so there's no
  discontinuity in the command at the edge of the zone.
* **Low-pass filtering** of the rate signal, so compass noise doesn't get
  amplified by the derivative term into a buzzing actuator.
* **Anti-windup.** The integral (which trims out steady wind / current / prop
  walk) stops accumulating while the rudder is hard over, so it can't wind up
  and cause a big overshoot when the boat finally comes back.
* **Output slew-rate limit.** Caps how fast the commanded rudder angle can
  change, guaranteeing smooth tiller inputs regardless of a noisy error.

All state is explicit so the controller is trivially unit-testable and can be
stepped deterministically in the simulator.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PIDGains:
    """Tuning constants. Sensible seeds come from :func:`autopilot.tuning`."""

    kp: float = 0.0          # rudder deg per deg of heading error
    ki: float = 0.0          # rudder deg per deg-second of accumulated error
    kd: float = 0.0          # counter-rudder: rudder deg per (deg/s) of turn


@dataclass
class PIDLimits:
    """Physical / comfort limits applied to the controller output."""

    output_min: float = -35.0        # max rudder to port (deg)
    output_max: float = 35.0         # max rudder to starboard (deg)
    slew_rate: float = 20.0          # max change in commanded rudder (deg/s)
    integral_limit: float = 10.0     # clamp on the integral's rudder contribution (deg)


@dataclass
class PIDConfig:
    gains: PIDGains = field(default_factory=PIDGains)
    limits: PIDLimits = field(default_factory=PIDLimits)
    deadband: float = 2.0            # heading error ignored within +/- this (deg)
    rate_filter_tau: float = 0.5     # low-pass time constant for the rate term (s)
    integral_active_band: float = float("inf")  # only integrate when |error| <= this


def _soft_deadband(error: float, band: float) -> float:
    """Shrink ``error`` toward zero by ``band`` without a jump at the edge.

    |error| <= band  -> 0
    otherwise         -> error - sign(error) * band
    """
    if band <= 0.0:
        return error
    if error > band:
        return error - band
    if error < -band:
        return error + band
    return 0.0


@dataclass
class PIDDebug:
    """Per-step breakdown, handy for tuning plots and tests."""

    p: float = 0.0
    i: float = 0.0
    d: float = 0.0
    raw: float = 0.0            # p+i+d before slew limiting
    output: float = 0.0        # after clamp + slew limit
    filtered_rate: float = 0.0
    saturated: bool = False


class PID:
    """Discrete PID heading controller.

    Call :meth:`update` once per control tick with the signed heading error and
    the boat's rate of turn. It returns the commanded rudder angle (deg,
    starboard positive).
    """

    def __init__(self, config: Optional[PIDConfig] = None):
        self.config = config or PIDConfig()
        self._integral = 0.0
        self._filtered_rate = 0.0
        self._output = 0.0
        self._have_rate = False
        self.debug = PIDDebug()

    def reset(self) -> None:
        """Clear accumulated state (e.g. when re-engaging the pilot)."""
        self._integral = 0.0
        self._filtered_rate = 0.0
        self._output = 0.0
        self._have_rate = False
        self.debug = PIDDebug()

    def _filter_rate(self, rate: float, dt: float) -> float:
        """First-order low-pass on the rate-of-turn signal."""
        tau = self.config.rate_filter_tau
        if tau <= 0.0 or not self._have_rate:
            self._filtered_rate = rate
            self._have_rate = True
            return rate
        alpha = dt / (tau + dt)
        self._filtered_rate += alpha * (rate - self._filtered_rate)
        return self._filtered_rate

    def update(self, error: float, rate_of_turn: float, dt: float) -> float:
        """Advance the controller one step.

        :param error: signed heading error, ``desired - measured`` (deg),
            already wrapped to (-180, 180] by :func:`autopilot.heading.heading_error`.
        :param rate_of_turn: boat yaw rate (deg/s), starboard positive.
        :param dt: time since last update (s).
        :returns: commanded rudder angle (deg), starboard positive.
        """
        if dt <= 0.0:
            return self._output

        g = self.config.gains
        lim = self.config.limits

        # Deadband first: within the band the boat is "on course", so we neither
        # push proportionally nor wind up the integral.
        db_error = _soft_deadband(error, self.config.deadband)

        filtered_rate = self._filter_rate(rate_of_turn, dt)

        # --- Proportional --------------------------------------------------
        p = g.kp * db_error

        # --- Derivative (counter rudder) -----------------------------------
        # Oppose the turn: if the bow is swinging to starboard (+rate) we want
        # port rudder (-), hence the minus sign. This is the damping term.
        d = -g.kd * filtered_rate

        # --- Integral with conditional anti-windup -------------------------
        # Only integrate when the boat is near course (PD during a course
        # change, PID for course keeping - standard marine autopilot practice).
        # This stops the integral winding up during a big turn and then forcing
        # a slow, overshooting crawl back onto the new heading.
        if abs(error) <= self.config.integral_active_band:
            accumulation = g.ki * db_error * dt
        else:
            accumulation = 0.0
        # Provisionally integrate, form the output, and only *keep* the new
        # integral if it didn't drive us further into saturation (clamped
        # back-calculation). This trims steady offsets without overshoot.
        provisional_integral = self._integral + accumulation
        provisional_integral = _clamp(provisional_integral,
                                      -lim.integral_limit, lim.integral_limit)
        raw = p + provisional_integral + d
        clamped = _clamp(raw, lim.output_min, lim.output_max)
        saturated = clamped != raw

        # Anti-windup: if saturated and the integral would push further into the
        # rail, don't accept the new integral value.
        if saturated and (provisional_integral * (raw - clamped) > 0):
            pass  # keep old integral
        else:
            self._integral = provisional_integral
            raw = p + self._integral + d
            clamped = _clamp(raw, lim.output_min, lim.output_max)
            saturated = clamped != raw

        # --- Output slew-rate limit ---------------------------------------
        max_step = lim.slew_rate * dt
        step = _clamp(clamped - self._output, -max_step, max_step)
        self._output += step

        self.debug = PIDDebug(
            p=p, i=self._integral, d=d, raw=raw, output=self._output,
            filtered_rate=filtered_rate, saturated=saturated,
        )
        return self._output

    @property
    def output(self) -> float:
        return self._output

    @property
    def integral(self) -> float:
        return self._integral


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x
