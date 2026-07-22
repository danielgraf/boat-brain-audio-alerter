"""CourseKeeper: the top-level "hold a true course" loop.

It owns the flow the user described:

    dial in a heading  ->  read NMEA heading  ->  compare to desired
                       ->  PID (damped) rudder command  ->  drive the ram

and adds the operational glue: engage/standby, deriving yaw rate when the
sensor doesn't supply one, speed-scheduled gains, and a low-steerage guard so
the pilot doesn't thrash the helm when the boat is barely moving.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .config import AutopilotConfig
from .heading import heading_error, normalize_180, normalize_360, yaw_rate as derive_yaw_rate
from .pid import PID
from .ram import RamActuator


@dataclass
class CourseKeeperConfig:
    """Thin wrapper so the controller can be built from an AutopilotConfig."""

    autopilot: AutopilotConfig = field(default_factory=AutopilotConfig)


@dataclass
class ControlTick:
    """Diagnostics returned from each :meth:`CourseKeeper.update` call."""

    engaged: bool
    desired_heading: float
    measured_heading: float
    error: float
    yaw_rate: float
    rudder_command: float
    rudder_achieved: float
    speed_knots: float
    reason: str = ""


class CourseKeeper:
    """Closed-loop heading-hold controller.

    Wire it to a :class:`~autopilot.ram.RamActuator` (SimRam for testing, a
    HardwareRam on the boat). Feed :meth:`update` a fresh heading each control
    tick; it returns a :class:`ControlTick` for logging/telemetry.
    """

    def __init__(self, config: AutopilotConfig, ram: RamActuator):
        self.config = config
        self.ram = ram
        self.pid = PID(config.pid)
        self._engaged = False
        self._desired: Optional[float] = None
        self._prev_heading: Optional[float] = None
        self._filtered_heading: Optional[float] = None
        self._base_gains = config.pid.gains

    # ---- engage / setpoint --------------------------------------------
    def engage(self, desired_heading: Optional[float] = None,
               current_heading: Optional[float] = None) -> None:
        """Engage the pilot.

        With no ``desired_heading`` it locks onto the current heading - the
        common "press AUTO and hold what I've got" case.
        """
        if desired_heading is None:
            desired_heading = current_heading
        if desired_heading is None:
            raise ValueError("engage needs a desired or current heading")
        self._desired = normalize_360(desired_heading)
        self._engaged = True
        self.pid.reset()
        self._prev_heading = current_heading
        self._filtered_heading = (normalize_360(current_heading)
                                  if current_heading is not None else None)

    def standby(self) -> None:
        """Disengage: stop driving the ram and let the helm go free."""
        self._engaged = False
        self.ram.stop()

    def set_heading(self, desired_heading: float) -> None:
        """Dial in a new course to hold (deg true)."""
        self._desired = normalize_360(desired_heading)

    def adjust_heading(self, delta_deg: float) -> None:
        """Nudge the target, e.g. +1 / -1 / +10 buttons on the pilot head."""
        if self._desired is None:
            raise ValueError("no heading set")
        self._desired = normalize_360(self._desired + delta_deg)

    @property
    def engaged(self) -> bool:
        return self._engaged

    @property
    def desired_heading(self) -> Optional[float]:
        return self._desired

    # ---- speed-scheduled gains ----------------------------------------
    def _apply_speed_scheduling(self, speed_knots: float) -> None:
        """Re-tune the PID for the current speed from the Nomoto model.

        Rudder authority scales ~speed^2; without this the pilot hunts at hull
        speed and goes limp when slow. We re-derive gains keeping the same
        damping ratio, so smoothness is preserved across the speed range.
        """
        if not self.config.speed_scheduling:
            self.pid.config.gains = self._base_gains
            return
        v = max(speed_knots, self.config.min_speed_knots)
        scaled = self.config.nomoto.scaled_to_speed(v, self.config.ref_speed_knots)
        try:
            # Re-derive gains for the scaled plant using the SAME feel targets
            # (damping ratio + bandwidth) as the base tuning, so smoothness is
            # preserved across the speed range instead of silently changing.
            self.pid.config.gains = self.config.gains_for(scaled)
        except ValueError:
            self.pid.config.gains = self._base_gains

    # ---- main loop -----------------------------------------------------
    def update(
        self,
        measured_heading: float,
        dt: float,
        sensor_yaw_rate: Optional[float] = None,
        speed_knots: Optional[float] = None,
    ) -> ControlTick:
        """Run one control tick.

        :param measured_heading: heading from NMEA (deg).
        :param dt: seconds since the previous tick.
        :param sensor_yaw_rate: yaw rate from a ROT sentence / gyro (deg/s).
            If ``None`` it's derived from successive headings.
        :param speed_knots: boat speed for gain scheduling / steerage guard.
        """
        raw = normalize_360(measured_heading)
        speed = speed_knots if speed_knots is not None else self.config.ref_speed_knots

        # Low-pass the compass input (wrap-aware). A raw compass is noisy, and
        # differentiating noise for the counter-rudder term is the fastest route
        # to a buzzing helm. Real fluxgate/GPS compasses are physically damped;
        # this models that so the derivative term sees a clean signal.
        tau = self.config.heading_filter_tau
        if tau > 0.0 and self._filtered_heading is not None:
            alpha = dt / (tau + dt)
            self._filtered_heading = normalize_360(
                self._filtered_heading + alpha * normalize_180(raw - self._filtered_heading))
        else:
            self._filtered_heading = raw
        measured = self._filtered_heading

        # Yaw rate: prefer a real sensor (clean), else differentiate the
        # *filtered* heading.
        if sensor_yaw_rate is not None:
            rate = sensor_yaw_rate
        elif self._prev_heading is not None:
            rate = derive_yaw_rate(self._prev_heading, measured, dt)
        else:
            rate = 0.0
        self._prev_heading = measured

        if not self._engaged or self._desired is None:
            return ControlTick(
                engaged=False, desired_heading=self._desired or 0.0,
                measured_heading=measured, error=0.0, yaw_rate=rate,
                rudder_command=self.ram.rudder_angle,
                rudder_achieved=self.ram.rudder_angle,
                speed_knots=speed, reason="standby",
            )

        error = heading_error(self._desired, measured)

        # Low-steerage guard: with no water flow over the rudder the helm has no
        # authority, so hold station rather than winding up and slamming over.
        if speed_knots is not None and speed_knots < self.config.min_speed_knots:
            achieved = self.ram.command(self.ram.rudder_angle, dt)
            return ControlTick(
                engaged=True, desired_heading=self._desired, measured_heading=measured,
                error=error, yaw_rate=rate, rudder_command=self.ram.rudder_angle,
                rudder_achieved=achieved, speed_knots=speed, reason="low-steerage",
            )

        self._apply_speed_scheduling(speed)
        rudder_cmd = self.pid.update(error, rate, dt)
        achieved = self.ram.command(rudder_cmd, dt)

        return ControlTick(
            engaged=True, desired_heading=self._desired, measured_heading=measured,
            error=error, yaw_rate=rate, rudder_command=rudder_cmd,
            rudder_achieved=achieved, speed_knots=speed, reason="hold",
        )
