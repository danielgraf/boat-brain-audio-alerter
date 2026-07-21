#!/usr/bin/env python3
"""Live heading-hold loop skeleton: NMEA in -> CourseKeeper -> ram out.

This is the on-the-boat entry point. It wires the real pieces together but
leaves the two hardware touch-points as clearly marked stubs so you can run it
from a laptop reading a recorded NMEA log before you ever energise the ram:

* **NMEA input** - here it reads lines from stdin (or a file). On the boat point
  it at your serial/UDP multiplexer instead.
* **Ram output** - here it uses a :class:`HardwareRam` whose drive callback just
  prints. Wire that callback to your motor driver, and ``read_stroke`` to your
  rudder-reference feedback (or leave it ``None`` for dead-reckoned drive).

Usage:
    python examples/run_live.py --config config.example.json --heading 90 < nmea.log
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autopilot.config import AutopilotConfig
from autopilot.controller import CourseKeeper
from autopilot.nmea import NmeaHeadingSource
from autopilot.ram import HardwareRam


def make_hardware_ram(cfg: AutopilotConfig) -> HardwareRam:
    # ---- replace these two callbacks with real hardware I/O ----
    def drive(effort: float) -> None:
        # effort in [-1, 1], sign already corrected for mount side.
        # e.g. set motor PWM + direction pin here.
        print(f"    [ram] drive={effort:+.2f}", file=sys.stderr)

    def read_stroke():
        # Return rudder-reference feedback, or None if the install has none.
        return None
    # ------------------------------------------------------------
    return HardwareRam(cfg.ram, drive=drive, read_stroke=read_stroke,
                       speed_stroke_per_s=cfg.ram_speed_stroke_per_s)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="config.example.json", help="autopilot config JSON")
    ap.add_argument("--heading", type=float, default=None,
                    help="course to hold (deg). Omit to hold the first heading seen.")
    ap.add_argument("--source", default="-", help="NMEA source file, or '-' for stdin")
    args = ap.parse_args()

    cfg = (AutopilotConfig.load(args.config)
           if os.path.exists(args.config) else AutopilotConfig())
    if not any((cfg.pid.gains.kp, cfg.pid.gains.kd)):
        cfg.autotune()

    nmea = NmeaHeadingSource(prefer_true=True)
    ram = make_hardware_ram(cfg)
    keeper = CourseKeeper(cfg, ram)

    stream = sys.stdin if args.source == "-" else open(args.source, encoding="utf-8")
    last_t = None
    engaged = False

    for line in stream:
        nmea.update(line)
        heading = nmea.heading
        if heading is None:
            continue  # wait until we have a heading fix

        now = time.monotonic()
        dt = 0.1 if last_t is None else max(now - last_t, 1e-3)
        last_t = now

        if not engaged:
            keeper.engage(args.heading, current_heading=heading)
            engaged = True
            print(f"Engaged, holding {keeper.desired_heading:.0f} deg", file=sys.stderr)

        tick = keeper.update(
            heading, dt,
            sensor_yaw_rate=nmea.rot_deg_per_sec,   # None if no ROT sentence
            speed_knots=nmea.speed_knots,
        )
        print(f"hdg={tick.measured_heading:6.1f} err={tick.error:+6.1f} "
              f"rudder_cmd={tick.rudder_command:+6.1f} "
              f"rudder={tick.rudder_achieved:+6.1f} [{tick.reason}]")

    keeper.standby()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
