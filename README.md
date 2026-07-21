# boat-brain autopilot

Core "hold a true course" heading-hold autopilot for a boat brain: read heading
from NMEA, compare against a dialled-in course, and drive an auto-helm ram with
**smooth, well-damped tiller inputs** that don't hunt or induce oscillation.

This first milestone is the **control core plus a test harness** — no hardware
required to develop or tune. The ram driver is an abstraction with a hardware
stub, so the whole loop runs against a boat simulator on a laptop, and drops
onto the real ram by filling in two callbacks.

```
   dial in heading ─▶ read NMEA heading ─▶ compare (shortest arc)
                                              │
                                     PID (damped) ─▶ rudder angle
                                              │
                                   ram (calibrated) ─▶ auto-helm
```

Pure Python standard library — no pip installs needed on the Pi. `matplotlib`
is optional, only for plots in the test harness.

---

## Why it doesn't hunt — the damping design

A naive proportional controller makes a boat *hunt*: it weaves back and forth
across the course, sawing the helm. Everything here is built to stop that, using
the same techniques as commercial pilots (Raymarine, B&G, CPT) and the ship
steering literature:

| Technique | Where | What it does |
|---|---|---|
| **Counter-rudder** (derivative on *yaw rate*, not error) | `pid.py` | Applies opposite rudder as the bow swings toward target — kills overshoot. The single most important anti-hunting term. |
| **Deadband** (soft, ~1.5°) | `pid.py` | Ignores tiny heading wander so the helm doesn't chatter. |
| **Motor deadband** (~0.7° of rudder) | `ram.py` | The ram won't move for sub-degree commands — the classic "dead-range" that stops actuator buzz. |
| **Compass input filtering** | `controller.py` | Damps a noisy heading feed so the derivative term isn't fed noise. |
| **Anti-windup** on the integral | `pid.py` | Integral stops accumulating at the rudder stops, so it can't wind up and overshoot. |
| **PD during a course change, PID for keeping** | `pid.py` | Integral is suppressed while the error is large, so a big turn doesn't wind up the integral and crawl back. |
| **Output slew-rate limit** | `pid.py` | Caps how fast the commanded rudder can change → smooth inputs. |
| **Speed-scheduled gains** | `controller.py` | Rudder authority ∝ speed²; gains rescale with boat speed so it neither hunts at hull speed nor goes limp when slow. |

### Damping is *derived*, not guessed

The boat's yaw behaves (to first order) like the **Nomoto model** `T·ṙ + r = K·δ`.
With proportional + counter-rudder that gives a standard second-order closed
loop, so `tuning.py` **places the poles** for a chosen damping ratio ζ:

```
Kp = ωₙ²·T / K
Kd = (2·ζ·ωₙ·T − 1) / K
```

You pick two intuitive numbers: **ζ** (damping — 0.9 gives brisk, no-overshoot
response) and either a **settling time** or bandwidth. The harness demonstrates
the effect on a 30° step:

| ζ | Overshoot | Settling | |
|---|---|---|---|
| 0.4 (under-damped) | 13.9° | never | hunts |
| **0.9 (default)** | **3.2°** | **14 s** | **smooth** |
| 1.2 (over-damped) | 2.7° | 58 s | sluggish |

Sources: [B&G — autopilot performance functions](https://www.bandg.com/en-nz/blog/autopilots-understanding-performance-functions/),
[CPT autopilot manual](https://www.cptautopilot.com/manual/autopilot_controls.html),
[Nomoto steering-model autopilot design](https://www.researchgate.net/publication/277497099_Ships_Steering_Autopilot_Design_by_Nomoto_Model).

---

## Quick start

```bash
# Run the closed-loop test harness across all scenarios (no dependencies):
python tools/run_sim.py

# Save plots (needs matplotlib) or raw CSV (no deps):
python tools/run_sim.py --plot out/
python tools/run_sim.py --csv out/

# Explore damping / tuning:
python tools/run_sim.py --zeta 0.7 --settling 6
python tools/run_sim.py --scenario step --no-schedule

# Run the unit tests:
python -m unittest discover -s tests
```

The harness scores each scenario for exactly the things we're preventing —
**overshoot, settling time, steady-state error, and helm reversals (the hunting
proxy)** — and exits non-zero if any scenario blows its budget, so it doubles as
a CI regression gate.

## Live on the boat

```bash
# Reads NMEA from stdin/file, drives a (stubbed) hardware ram:
python examples/run_live.py --config config.example.json --heading 90 < nmea.log
```

Two hardware touch-points to fill in (both clearly marked in
`examples/run_live.py` and `autopilot/ram.py::HardwareRam`):

1. **`drive(effort)`** — set your motor driver from an effort in `[-1, 1]` (mount-side sign already applied).
2. **`read_stroke()`** — return your rudder-reference feedback, or `None` for dead-reckoned drive.

---

## Setup & calibration

Run the guided dockside walk-through:

```bash
python examples/calibrate_ram.py
```

It captures the boat-specific setup the task called out:

- **Which side the ram is mounted** (`mount_side`) — sets the drive sense, so a
  reversed install is a one-line config change, not rewiring. Fixing it wrong
  just flips `drive_sign`.
- **Centering with equal travel each side** — you drive the ram gently to each
  mechanical end stop and record the rudder-feedback reading. Centre is the
  midpoint; with `symmetric_travel=True` the usable stroke is clamped to the
  *smaller* half so the helm has identical authority to port and starboard
  (a tiller pilot with unequal travel can chase a heading one way and run out of
  ram the other).

Calibration and tuning persist to a human-readable JSON file
(`config.example.json` is a complete, valid example) that you can hand-edit on
the boat.

---

## Tuning guide

1. **Get the boat's Nomoto K, T.** Either estimate by eye, or do a rudder-step
   sea trial and let the code recover them:
   ```python
   from autopilot.tuning import estimate_nomoto_from_step
   params = estimate_nomoto_from_step(times, yaw_rates, rudder_deg=10)
   ```
   `K` = steady yaw-rate ÷ rudder; `T` = time to reach 63% of steady yaw rate.

2. **Auto-tune** for a feel:
   ```python
   cfg.nomoto = params
   cfg.target_damping = 0.9        # 0.9 = no hunting; lower = livelier/looser
   cfg.target_settling_time = 8.0  # seconds to settle after a correction
   cfg.autotune()                  # sets Kp, Ki, Kd by pole placement
   ```

3. **Trim the comfort settings** to taste: `pid.deadband` (course-keeping
   tightness), `ram.deadband_stroke` (helm quietness), `heading_filter_tau`
   (compass smoothing), `pid.limits.slew_rate` (input smoothness).

4. **Verify in the sim** before sea trials — `tools/run_sim.py` will show you
   overshoot, settling and helm activity for your numbers.

If in doubt, more counter-rudder (higher ζ) and a wider deadband cure hunting;
too much makes the helm sluggish.

---

## Architecture

| Module | Responsibility |
|---|---|
| `autopilot/heading.py` | Wrap-safe compass math (shortest-arc error, yaw rate). |
| `autopilot/nmea.py` | Minimal NMEA 0183 parser (HDT/HDM/HDG/ROT/VHW/RMC/VTG) + heading source. |
| `autopilot/pid.py` | PID with counter-rudder, deadband, filtering, anti-windup, slew limit. |
| `autopilot/ram.py` | Ram calibration (mount side, centre, symmetric travel) + Sim/Hardware actuators. |
| `autopilot/tuning.py` | Nomoto pole-placement auto-tune + K,T estimation + gain scheduling. |
| `autopilot/controller.py` | `CourseKeeper` — the top-level hold-a-course loop. |
| `autopilot/config.py` | Config dataclasses + JSON persistence. |
| `autopilot/simulator.py` | First-order Nomoto boat model + disturbances + scoring. |
| `tools/run_sim.py` | Closed-loop test harness / regression gate. |
| `examples/` | `calibrate_ram.py`, `run_live.py` hardware-integration skeletons. |

## Status & roadmap

Done: heading-hold core, damping, calibration, auto-tune, simulator, tests.

Natural next steps: rate-of-turn limited course changes (turn at a set ROT),
adaptive "sea state" gain/deadband, wind-vane (apparent-wind) steering, cross-
track / waypoint steering, and the concrete GPIO/motor-driver + feedback wiring
behind `HardwareRam`.
