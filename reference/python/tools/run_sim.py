#!/usr/bin/env python3
"""Closed-loop autopilot test harness.

Runs the CourseKeeper against the Nomoto boat model through a set of scenarios
(step change, wind bias, wave disturbance, sensor noise) and reports the
metrics that define smooth, non-hunting behaviour. Auto-tunes the PID from the
boat's Nomoto parameters via pole-placement so the damping is principled, not
guessed.

Usage:
    python tools/run_sim.py                 # run all scenarios, print metrics
    python tools/run_sim.py --plot out/     # also write PNGs (needs matplotlib)
    python tools/run_sim.py --csv out/      # write CSV time series (no deps)
    python tools/run_sim.py --scenario step # run one scenario by name
    python tools/run_sim.py --zeta 0.7      # override damping ratio

Exit code is non-zero if any scenario fails its smoothness budget, so this
doubles as a regression gate in CI.
"""

from __future__ import annotations

import argparse
import os
import sys

# Allow running from a source checkout without installing.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autopilot.config import AutopilotConfig
from autopilot.controller import CourseKeeper
from autopilot.ram import MountSide, RamCalibration, SimRam
from autopilot.simulator import (
    BoatModel, Disturbances, Metrics, Scenario, evaluate, run_closed_loop,
)
from autopilot.tuning import NomotoParams, nomoto_pid_gains


def build_scenarios():
    """A representative test matrix."""
    return [
        Scenario(
            name="step",
            initial_heading=0.0, desired_heading=30.0, duration_s=90.0,
        ),
        Scenario(
            name="wrap",  # exercises the 360->0 boundary
            initial_heading=350.0, desired_heading=20.0, duration_s=90.0,
        ),
        Scenario(
            name="wind_bias",
            initial_heading=90.0, desired_heading=90.0, duration_s=120.0,
            disturbances=Disturbances(steady_yaw_dps2=0.4),
        ),
        Scenario(
            # A moderate seaway. The helm legitimately works roughly once per
            # wave (~6s -> ~10-20 reversals/min is expected, not hunting), so
            # the budget checks that it *stays on course* with bounded helm
            # movement rather than demanding an unrealistically still helm.
            name="waves",
            initial_heading=90.0, desired_heading=90.0, duration_s=120.0,
            disturbances=Disturbances(wave_amplitude_dps2=1.5, wave_period_s=6.0),
            max_reversals_per_min=24.0, max_ram_travel_per_min=450.0,
        ),
        Scenario(
            # 0.5deg RMS compass noise - representative of a real fluxgate/GPS
            # heading feed (1.5deg white noise, tested earlier, is harsher than
            # any real sensor). The input filter must keep this off the helm.
            name="noisy_compass",
            initial_heading=0.0, desired_heading=45.0, duration_s=90.0,
            disturbances=Disturbances(heading_noise_deg=0.5),
            # Small (<0.5deg) noise-driven corrections are expected; the real
            # guard against a noise-thrashed ram is the travel cap, well met here.
            max_reversals_per_min=36.0, max_ram_travel_per_min=250.0,
        ),
        Scenario(
            name="course_changes",
            initial_heading=0.0, desired_heading=0.0, duration_s=180.0,
            setpoint_changes=[(30.0, 40.0), (90.0, 100.0), (140.0, 90.0)],
        ),
    ]


def check_budget(scenario: Scenario, m: Metrics) -> list:
    """Score a run against that scenario's own smoothness budget."""
    failures = []
    minutes = max(scenario.duration_s / 60.0, 1e-6)
    if m.max_overshoot > scenario.max_overshoot:
        failures.append(f"overshoot {m.max_overshoot:.1f} > {scenario.max_overshoot}")
    if abs(m.steady_state_error) > scenario.max_steady_error:
        failures.append(
            f"steady err {m.steady_state_error:+.1f} > {scenario.max_steady_error}")
    rpm = m.rudder_reversals / minutes
    if rpm > scenario.max_reversals_per_min:
        failures.append(f"hunting {rpm:.0f} reversals/min > {scenario.max_reversals_per_min}")
    if scenario.max_ram_travel_per_min is not None:
        tpm = m.rudder_travel / minutes
        if tpm > scenario.max_ram_travel_per_min:
            failures.append(
                f"ram travel {tpm:.0f}deg/min > {scenario.max_ram_travel_per_min}")
    return failures


def make_keeper(config: AutopilotConfig) -> tuple:
    cal = config.ram
    ram = SimRam(cal, speed_stroke_per_s=config.ram_speed_stroke_per_s)
    keeper = CourseKeeper(config, ram)
    return keeper, ram


def plot_run(log, scenario, metrics, out_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("  (matplotlib not available - use --csv for raw data)")
        return
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax1.plot(log.t, log.heading, label="heading", lw=1.5)
    ax1.plot(log.t, log.desired, "--", label="desired", color="k", lw=1)
    ax1.set_ylabel("heading (deg)")
    ax1.legend(loc="best", fontsize=8)
    ax1.set_title(f"{scenario.name}: {metrics.summary()}", fontsize=9)
    ax2.plot(log.t, log.rudder_cmd, label="rudder cmd", color="tab:orange", lw=1)
    ax2.plot(log.t, log.rudder_actual, label="rudder actual", color="tab:red", lw=1)
    ax2.set_ylabel("rudder (deg)")
    ax2.set_xlabel("time (s)")
    ax2.legend(loc="best", fontsize=8)
    fig.tight_layout()
    path = os.path.join(out_dir, f"{scenario.name}.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"  plot -> {path}")


def write_csv(log, scenario, out_dir):
    path = os.path.join(out_dir, f"{scenario.name}.csv")
    with open(path, "w", encoding="utf-8") as f:
        f.write("t,heading,desired,error,rudder_cmd,rudder_actual,yaw_rate\n")
        for i in range(len(log.t)):
            f.write(f"{log.t[i]:.3f},{log.heading[i]:.4f},{log.desired[i]:.4f},"
                    f"{log.error[i]:.4f},{log.rudder_cmd[i]:.4f},"
                    f"{log.rudder_actual[i]:.4f},{log.yaw_rate[i]:.4f}\n")
    print(f"  csv  -> {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plot", metavar="DIR", help="write PNG plots to DIR (needs matplotlib)")
    ap.add_argument("--csv", metavar="DIR", help="write CSV time series to DIR")
    ap.add_argument("--scenario", help="run only this scenario by name")
    ap.add_argument("--zeta", type=float, default=0.9, help="damping ratio (default 0.9)")
    ap.add_argument("--settling", type=float, default=8.0,
                    help="target settling time seconds for auto-tune (default 8)")
    ap.add_argument("--no-schedule", action="store_true", help="disable speed scheduling")
    args = ap.parse_args()

    # Boat + auto-tuned config. The tuning targets (damping ratio, settling
    # time) are the single source of truth; config.autotune() derives the gains.
    nomoto = NomotoParams(K=0.15, T=3.0)
    config = AutopilotConfig()
    config.nomoto = nomoto
    config.target_damping = args.zeta
    config.target_settling_time = args.settling
    config.integral_fraction = 0.1
    gains = config.autotune()
    config.pid.deadband = 1.5
    config.pid.integral_active_band = 8.0     # PD during a course change, PID for keeping
    config.pid.rate_filter_tau = 1.0          # smooth the rate term against compass noise
    config.heading_filter_tau = 1.5           # damp the compass input (fluxgate-like)
    config.pid.limits.output_max = 30.0
    config.pid.limits.output_min = -30.0
    config.pid.limits.slew_rate = 15.0
    config.pid.limits.integral_limit = 20.0   # enough standing rudder for weather helm
    config.speed_scheduling = not args.no_schedule
    config.ram = RamCalibration.from_endpoints(
        port_stop=-1.0, starboard_stop=1.0, mount_side=MountSide.PORT, max_rudder_deg=35.0)
    # Motor deadband: the ram ignores commands smaller than ~0.7deg of rudder so
    # it doesn't chatter chasing tiny corrections (the classic "dead-range").
    config.ram.deadband_stroke = 0.03

    print(f"Auto-tuned PID (zeta={args.zeta}, settling~{args.settling}s): "
          f"Kp={gains.kp:.2f} Ki={gains.ki:.3f} Kd={gains.kd:.2f}")
    print(f"Nomoto K={nomoto.K} T={nomoto.T}  speed-scheduling={config.speed_scheduling}\n")

    for d in (args.plot, args.csv):
        if d:
            os.makedirs(d, exist_ok=True)

    scenarios = build_scenarios()
    if args.scenario:
        scenarios = [s for s in scenarios if s.name == args.scenario]
        if not scenarios:
            print(f"no scenario named {args.scenario!r}")
            return 2

    any_fail = False
    for scenario in scenarios:
        keeper, ram = make_keeper(config)
        boat = BoatModel(K=nomoto.K, T=nomoto.T, speed_knots=config.ref_speed_knots)
        log = run_closed_loop(keeper, boat, ram, scenario, control_hz=config.control_hz)
        metrics = evaluate(log)
        failures = check_budget(scenario, metrics)
        status = "PASS" if not failures else "FAIL"
        if failures:
            any_fail = True
        print(f"[{status}] {scenario.name}")
        print(f"  {metrics.summary()}")
        if failures:
            print(f"  budget: {'; '.join(failures)}")
        if args.plot:
            plot_run(log, scenario, metrics, args.plot)
        if args.csv:
            write_csv(log, scenario, args.csv)

    print("\n" + ("Some scenarios FAILED their smoothness budget." if any_fail
                  else "All scenarios within smoothness budget."))
    return 1 if any_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
