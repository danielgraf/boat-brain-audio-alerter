"""Auto-helm ram: calibration and the rudder-angle -> drive mapping.

The controller thinks in **rudder angle** (deg, starboard positive). The
physical world is a linear hydraulic/electric ram shoving the tiller. This
module is the translation layer, and it owns the boat-specific *setup*:

Setup / calibration parameters
------------------------------
* **mount_side** - which side of the boat the ram is mounted. Extending the ram
  pushes the tiller one way; whether that turns the boat to port or starboard
  flips with the mounting side. Captured as ``drive_sign`` so a wrongly-wired
  install is a one-line config change, not a rebuild.
* **centre calibration** - the ram feedback reading (or stroke position) at
  which the rudder is amidships. You calibrate by driving the ram to each
  mechanical end stop, recording both, and taking the midpoint. See
  :meth:`RamCalibration.from_endpoints`.
* **equal travel each side** - a tiller pilot must have the *same* authority to
  port and starboard, otherwise it can chase a heading one way and run out of
  ram the other. ``symmetric_travel`` clamps usable stroke to the smaller of
  the two half-strokes so movement is guaranteed equal about centre, while the
  raw asymmetric limits are still respected as hard stops.

Two implementations share one interface:

* :class:`SimRam` - a software ram with finite speed and end stops, used by the
  simulator and the tests.
* :class:`HardwareRam` - a thin stub showing where real GPIO / motor-driver /
  feedback-pot calls go. It deliberately does no I/O so importing the package
  on a dev machine never touches hardware.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Callable, Optional


class MountSide(enum.Enum):
    """Which side the ram is mounted. Sets the sense of drive."""

    PORT = "port"
    STARBOARD = "starboard"


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass
class RamCalibration:
    """Geometry + wiring that maps rudder angle to ram stroke.

    ``stroke`` is in whatever unit the feedback device reports - millimetres,
    ADC counts, whatever - as long as it is linear with rudder angle. Only the
    *ratios* matter, so you never have to know the exact lever geometry.
    """

    mount_side: MountSide = MountSide.PORT
    centre: float = 0.0                  # stroke reading at rudder amidships
    port_stop: float = -1.0              # stroke reading at full port rudder
    starboard_stop: float = 1.0          # stroke reading at full starboard rudder
    max_rudder_deg: float = 35.0         # rudder angle at the end stops
    symmetric_travel: bool = True        # force equal usable travel about centre
    deadband_stroke: float = 0.0         # ignore commands within this of current (backlash)

    def __post_init__(self) -> None:
        if self.starboard_stop == self.port_stop:
            raise ValueError("port_stop and starboard_stop must differ")
        if self.max_rudder_deg <= 0:
            raise ValueError("max_rudder_deg must be positive")

    @classmethod
    def from_endpoints(
        cls,
        port_stop: float,
        starboard_stop: float,
        mount_side: MountSide = MountSide.PORT,
        max_rudder_deg: float = 35.0,
        symmetric_travel: bool = True,
    ) -> "RamCalibration":
        """Build a calibration from a dockside end-stop sweep.

        Drive the ram gently to the port stop, record the feedback; repeat to
        starboard. Centre is the midpoint, so the rudder sits amidships with
        equal stroke available each way.
        """
        centre = 0.5 * (port_stop + starboard_stop)
        return cls(
            mount_side=mount_side,
            centre=centre,
            port_stop=port_stop,
            starboard_stop=starboard_stop,
            max_rudder_deg=max_rudder_deg,
            symmetric_travel=symmetric_travel,
        )

    @property
    def drive_sign(self) -> float:
        """+1 or -1: sign relating positive (starboard) rudder to +stroke.

        With the ram on the port side, extending it (increasing stroke) pushes
        the tiller to port, which turns the rudder to starboard - so the raw
        stops already encode the sense. We normalise using the recorded stops
        and flip for a starboard mount.
        """
        base = 1.0 if self.starboard_stop >= self.port_stop else -1.0
        return base if self.mount_side is MountSide.PORT else -base

    @property
    def half_travel(self) -> float:
        """Usable stroke from centre to the (possibly symmetric-limited) stop."""
        to_port = abs(self.centre - self.port_stop)
        to_stbd = abs(self.starboard_stop - self.centre)
        return min(to_port, to_stbd) if self.symmetric_travel else max(to_port, to_stbd)

    def rudder_to_stroke(self, rudder_deg: float) -> float:
        """Convert a commanded rudder angle to a target stroke reading."""
        rudder_deg = _clamp(rudder_deg, -self.max_rudder_deg, self.max_rudder_deg)
        frac = rudder_deg / self.max_rudder_deg  # -1..1, starboard positive
        if self.symmetric_travel:
            target = self.centre + self.drive_sign * frac * self.half_travel
        else:
            span = self.starboard_stop - self.centre if frac >= 0 else self.centre - self.port_stop
            target = self.centre + frac * abs(span) * self.drive_sign
        # Never command past a physical stop, regardless of the maths above.
        lo, hi = sorted((self.port_stop, self.starboard_stop))
        return _clamp(target, lo, hi)

    def stroke_to_rudder(self, stroke: float) -> float:
        """Inverse of :meth:`rudder_to_stroke`, for reading feedback back out."""
        if self.half_travel == 0:
            return 0.0
        frac = (stroke - self.centre) / (self.drive_sign * self.half_travel)
        return _clamp(frac, -1.0, 1.0) * self.max_rudder_deg


class RamActuator:
    """Interface: accept a rudder-angle command, report the achieved angle."""

    def command(self, rudder_deg: float, dt: float) -> float:
        """Drive toward ``rudder_deg`` for ``dt`` seconds; return achieved deg."""
        raise NotImplementedError

    def stop(self) -> None:
        """Cut drive to the ram (clutch out / motor off)."""
        raise NotImplementedError

    @property
    def rudder_angle(self) -> float:
        """Best estimate of the current rudder angle (deg)."""
        raise NotImplementedError


class SimRam(RamActuator):
    """Software ram with finite speed and hard end stops.

    Models the two things that matter for control quality: the ram can only move
    so fast (``speed_stroke_per_s``), and it stops dead at the mechanical
    limits. That finite speed is *why* the controller's slew limit and damping
    matter, so the simulator would be dishonest without it.
    """

    def __init__(self, calibration: RamCalibration, speed_stroke_per_s: float = 2.0,
                 initial_stroke: Optional[float] = None):
        self.cal = calibration
        self.speed = abs(speed_stroke_per_s)
        self.stroke = calibration.centre if initial_stroke is None else initial_stroke
        self._driving = True

    def command(self, rudder_deg: float, dt: float) -> float:
        if dt <= 0.0:
            return self.rudder_angle
        target = self.cal.rudder_to_stroke(rudder_deg)
        if self.cal.deadband_stroke > 0 and abs(target - self.stroke) < self.cal.deadband_stroke:
            return self.rudder_angle
        max_step = self.speed * dt
        self.stroke += _clamp(target - self.stroke, -max_step, max_step)
        lo, hi = sorted((self.cal.port_stop, self.cal.starboard_stop))
        self.stroke = _clamp(self.stroke, lo, hi)
        self._driving = True
        return self.rudder_angle

    def stop(self) -> None:
        self._driving = False

    @property
    def rudder_angle(self) -> float:
        return self.cal.stroke_to_rudder(self.stroke)


class HardwareRam(RamActuator):
    """Stub for a real ram: motor driver + optional feedback pot.

    Wire the two callbacks to your hardware. ``drive`` receives a signed effort
    in [-1, 1] (sign already corrected for mount side); ``read_stroke`` returns
    the feedback reading, or ``None`` if the install has no rudder-reference
    feedback (in which case we dead-reckon the stroke from commanded effort).

    No GPIO is imported here on purpose - this file must be importable on any
    machine. Do the real I/O in the callbacks you pass in.
    """

    def __init__(
        self,
        calibration: RamCalibration,
        drive: Callable[[float], None],
        read_stroke: Optional[Callable[[], Optional[float]]] = None,
        speed_stroke_per_s: float = 2.0,
    ):
        self.cal = calibration
        self._drive = drive
        self._read_stroke = read_stroke
        self.speed = abs(speed_stroke_per_s)
        self._estimated_stroke = calibration.centre

    def command(self, rudder_deg: float, dt: float) -> float:
        target = self.cal.rudder_to_stroke(rudder_deg)
        current = self._current_stroke()
        error = target - current
        if self.cal.deadband_stroke > 0 and abs(error) < self.cal.deadband_stroke:
            self._drive(0.0)
            return self.cal.stroke_to_rudder(current)
        # Bang-bang with the mount-side sign folded in. A position loop can be
        # dropped in here if the feedback resolution warrants it.
        effort = 1.0 if error > 0 else -1.0
        self._drive(effort * self.cal.drive_sign)
        # Dead-reckon the stroke in case there's no feedback pot.
        self._estimated_stroke += _clamp(error, -self.speed * dt, self.speed * dt)
        return self.cal.stroke_to_rudder(self._current_stroke())

    def _current_stroke(self) -> float:
        if self._read_stroke is not None:
            reading = self._read_stroke()
            if reading is not None:
                self._estimated_stroke = reading
                return reading
        return self._estimated_stroke

    def stop(self) -> None:
        self._drive(0.0)

    @property
    def rudder_angle(self) -> float:
        return self.cal.stroke_to_rudder(self._current_stroke())
