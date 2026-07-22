"""Autopilot configuration with plain-JSON persistence (stdlib only).

One dataclass tree holds everything the boat needs to remember between power
cycles: PID tuning, deadband/filter settings, ram calibration (mount side,
centre, end stops), and the Nomoto parameters used for auto-tuning. It
serialises to a human-readable JSON file you can eyeball or hand-edit on the
boat, and round-trips back to typed objects.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any, Dict

from typing import Optional

from .pid import PIDConfig, PIDGains, PIDLimits
from .ram import MountSide, RamCalibration
from .tuning import NomotoParams, nomoto_pid_gains


@dataclass
class AutopilotConfig:
    """Everything persisted for the heading-hold autopilot."""

    pid: PIDConfig = field(default_factory=PIDConfig)
    ram: RamCalibration = field(default_factory=RamCalibration)
    nomoto: NomotoParams = field(default_factory=lambda: NomotoParams(K=0.15, T=3.0))

    # Control-loop behaviour.
    control_hz: float = 10.0             # control tick rate
    ram_speed_stroke_per_s: float = 2.0  # full-stroke sweeps take ~stroke/speed s
    ref_speed_knots: float = 6.0         # speed the Nomoto params were measured at
    speed_scheduling: bool = True        # scale gains with boat speed
    min_speed_knots: float = 1.0         # below this, hold rudder (no steerage)
    heading_filter_tau: float = 0.0      # low-pass on compass input (s); models fluxgate damping

    # Auto-tune targets - the "how it should feel" dials the gains derive from.
    # These are the single source of truth for both the initial tuning and the
    # per-speed re-tuning, so the gains driving the boat always match intent.
    target_damping: float = 0.9              # zeta; ~0.9 = brisk, no hunting
    target_settling_time: float = 8.0        # s; used if natural_freq is None
    target_natural_freq: Optional[float] = None  # rad/s; overrides settling_time
    integral_fraction: float = 0.1           # integral aggressiveness

    def gains_for(self, params: NomotoParams) -> PIDGains:
        """Pole-placement gains for a plant, using the stored feel targets."""
        return nomoto_pid_gains(
            params,
            damping_ratio=self.target_damping,
            natural_frequency=self.target_natural_freq,
            settling_time=None if self.target_natural_freq else self.target_settling_time,
            integral_fraction=self.integral_fraction,
        )

    def autotune(self) -> PIDGains:
        """Set ``pid.gains`` from the Nomoto params at the reference speed."""
        self.pid.gains = self.gains_for(self.nomoto)
        return self.pid.gains

    # ---- serialisation -------------------------------------------------
    def to_dict(self) -> Dict[str, Any]:
        d = {
            "pid": asdict(self.pid),
            "ram": asdict(self.ram),
            "nomoto": asdict(self.nomoto),
            "control_hz": self.control_hz,
            "ram_speed_stroke_per_s": self.ram_speed_stroke_per_s,
            "ref_speed_knots": self.ref_speed_knots,
            "speed_scheduling": self.speed_scheduling,
            "min_speed_knots": self.min_speed_knots,
            "heading_filter_tau": self.heading_filter_tau,
            "target_damping": self.target_damping,
            "target_settling_time": self.target_settling_time,
            "target_natural_freq": self.target_natural_freq,
            "integral_fraction": self.integral_fraction,
        }
        # Enum -> its value for clean JSON.
        d["ram"]["mount_side"] = self.ram.mount_side.value
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "AutopilotConfig":
        pid_d = d.get("pid", {})
        pid = PIDConfig(
            gains=PIDGains(**pid_d.get("gains", {})),
            limits=PIDLimits(**pid_d.get("limits", {})),
            deadband=pid_d.get("deadband", 2.0),
            rate_filter_tau=pid_d.get("rate_filter_tau", 0.5),
        )
        ram_d = dict(d.get("ram", {}))
        if "mount_side" in ram_d:
            ram_d["mount_side"] = MountSide(ram_d["mount_side"])
        ram = RamCalibration(**ram_d) if ram_d else RamCalibration()
        nomoto_d = d.get("nomoto", {})
        nomoto = NomotoParams(**nomoto_d) if nomoto_d else NomotoParams(K=0.15, T=3.0)
        return cls(
            pid=pid,
            ram=ram,
            nomoto=nomoto,
            control_hz=d.get("control_hz", 10.0),
            ram_speed_stroke_per_s=d.get("ram_speed_stroke_per_s", 2.0),
            ref_speed_knots=d.get("ref_speed_knots", 6.0),
            speed_scheduling=d.get("speed_scheduling", True),
            min_speed_knots=d.get("min_speed_knots", 1.0),
            heading_filter_tau=d.get("heading_filter_tau", 0.0),
            target_damping=d.get("target_damping", 0.9),
            target_settling_time=d.get("target_settling_time", 8.0),
            target_natural_freq=d.get("target_natural_freq", None),
            integral_fraction=d.get("integral_fraction", 0.1),
        )

    def save(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2, sort_keys=True)
            f.write("\n")

    @classmethod
    def load(cls, path: str) -> "AutopilotConfig":
        with open(path, encoding="utf-8") as f:
            return cls.from_dict(json.load(f))
