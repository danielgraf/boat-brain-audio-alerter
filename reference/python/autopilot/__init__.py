"""Boat-brain autopilot package.

Core "hold a true course" heading-hold autopilot: read heading from NMEA,
compare against a dialled-in course, and drive an auto-helm ram with smooth,
well-damped tiller inputs.

The package is deliberately dependency-free (Python standard library only) so
it runs on a Raspberry Pi "boat brain" without a build toolchain. Plotting in
the test harness uses matplotlib if present and falls back to CSV otherwise.
"""

from .heading import normalize_180, normalize_360, heading_error
from .pid import PID, PIDGains
from .ram import RamCalibration, RamActuator, SimRam, MountSide
from .controller import CourseKeeper, CourseKeeperConfig
from .config import AutopilotConfig
from .tuning import nomoto_pid_gains

__all__ = [
    "normalize_180",
    "normalize_360",
    "heading_error",
    "PID",
    "PIDGains",
    "RamCalibration",
    "RamActuator",
    "SimRam",
    "MountSide",
    "CourseKeeper",
    "CourseKeeperConfig",
    "AutopilotConfig",
    "nomoto_pid_gains",
]
