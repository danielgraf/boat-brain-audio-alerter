#!/usr/bin/env python3
"""Dockside ram calibration walk-through (interactive skeleton).

Run this alongside the boat to build a :class:`RamCalibration` and save it into
your autopilot config. It captures the two things setup needs:

1. **Mount side** - so the drive sense is correct.
2. **End stops** - drive the ram gently to each mechanical limit and record the
   rudder-feedback reading; centre is the midpoint, giving equal travel each way.

The actual "drive the ram" and "read feedback" calls are stubbed with
``input()`` prompts so this runs with no hardware. Replace ``read_feedback`` and
``nudge_ram`` with your GPIO/motor-driver/ADC calls (see
``examples/run_live.py`` and :class:`autopilot.ram.HardwareRam`).
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autopilot.config import AutopilotConfig
from autopilot.ram import MountSide, RamCalibration


def read_feedback() -> float:
    """Return the rudder-reference reading. Stub -> keyboard."""
    return float(input("  feedback reading now: ").strip())


def nudge_ram(direction: str) -> None:
    """Pulse the ram toward ``direction`` ('port'/'starboard'). Stub."""
    print(f"  (drive ram {direction} - replace with your motor call)")


def main() -> int:
    print("Ram calibration\n===============")
    side = ""
    while side not in ("port", "starboard"):
        side = input("Which side is the ram mounted? [port/starboard]: ").strip().lower()
    mount = MountSide(side)

    input("\nDrive the ram GENTLY to the FULL PORT stop, then press Enter...")
    nudge_ram("port")
    port_stop = read_feedback()

    input("\nNow drive GENTLY to the FULL STARBOARD stop, then press Enter...")
    nudge_ram("starboard")
    starboard_stop = read_feedback()

    max_rudder = float(input("\nRudder angle at the stops (deg, e.g. 35): ").strip() or "35")

    cal = RamCalibration.from_endpoints(
        port_stop=port_stop, starboard_stop=starboard_stop,
        mount_side=mount, max_rudder_deg=max_rudder, symmetric_travel=True)

    print("\nCalibration result")
    print(f"  centre (rudder amidships): {cal.centre:.3f}")
    print(f"  usable travel each side  : {cal.half_travel:.3f} "
          f"({'symmetric' if cal.symmetric_travel else 'asymmetric'})")
    print(f"  drive sign               : {cal.drive_sign:+.0f}")

    path = input("\nSave into config file [autopilot.json]: ").strip() or "autopilot.json"
    cfg = AutopilotConfig.load(path) if os.path.exists(path) else AutopilotConfig()
    cfg.ram = cal
    cfg.save(path)
    print(f"Saved ram calibration into {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
