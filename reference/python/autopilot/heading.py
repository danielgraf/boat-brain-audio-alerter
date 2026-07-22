"""Compass-heading arithmetic.

Headings are in degrees. The one thing that trips up every heading controller
is angle wrap-around: 359 deg and 1 deg are 2 deg apart, not 358. Every
comparison in the autopilot goes through :func:`heading_error` so the sign and
magnitude of the correction are always the *shortest way round*.
"""

from __future__ import annotations


def normalize_360(angle: float) -> float:
    """Wrap an angle into the compass range [0, 360)."""
    return angle % 360.0


def normalize_180(angle: float) -> float:
    """Wrap an angle into (-180, 180].

    Used for signed errors: positive means "turn starboard / clockwise",
    negative means "turn to port / anticlockwise".
    """
    a = (angle + 180.0) % 360.0 - 180.0
    # (-180 % 360) gives -180; map it to +180 so the range is (-180, 180].
    return a + 360.0 if a <= -180.0 else a


def heading_error(desired: float, measured: float) -> float:
    """Signed shortest-arc error, ``desired - measured``, in (-180, 180].

    A positive result means the boat must turn to starboard (clockwise) to
    reach the desired heading; negative means turn to port.
    """
    return normalize_180(desired - measured)


def yaw_rate(previous: float, current: float, dt: float) -> float:
    """Rate of turn (deg/s) between two heading samples, wrap-safe.

    Positive = turning to starboard. Returns 0.0 for a non-positive ``dt`` so a
    stalled or duplicated sample can never produce an infinite derivative.
    """
    if dt <= 0.0:
        return 0.0
    return normalize_180(current - previous) / dt
