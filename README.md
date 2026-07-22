# boat-brain autopilot

Course-keeping autopilot for a boat brain, in Go, targeting a Raspberry Pi. It
reads NMEA, holds a **heading** or a **course over ground (COG)**, and drives an
auto-helm ram with smooth, well-damped tiller inputs that don't hunt. It ships
with an **interactive web simulator** so you can develop, tune and demonstrate
the whole loop with no hardware.

```
  NMEA in ─▶ heading / COG ─▶ compare (shortest arc) ─▶ PID (damped) ─▶ ram ─▶ helm
                    ▲                                        │
              COG outer loop ───── nudges target heading ────┘   (crab into current)
```

Pure standard library except `golang.org/x/sys` (for the Pi serial port).
Cross-compiles to a single static Pi binary; the web UI is embedded in it.

---

## Quick start — the simulator

```bash
go run ./cmd/simulator          # open http://localhost:8080
```

Then, in the browser:

- **Engage**, pick **Heading** or **COG track**, and dial a target with −10/−1/+1/+10.
- Add **wind, waves and current** with the sliders and watch the pilot cope.
- Drag the **Damping ζ** slider to feel the difference between crisp and hunting.

The nav view shows the boat with its **heading** (bow) and **COG** (ground track)
arrows, plus wind/current and the ground-track breadcrumb trail. The helm panel
shows the **rudder, tiller and ram** moving. The instrument panel shows heading,
COG, SOG, error, **crab angle**, rudder, ram %, and the live PID terms.

### Why COG, not just heading

Set a **3 kn current on the beam** and compare:

| Mode | Result |
|---|---|
| **Heading hold 090°** | bow holds 090°, but current sweeps the **ground track to ~116°** — you miss the mark |
| **COG track 090°** | bow **crabs to ~60°**, so the **ground track stays 090°** — you go where you aimed |

COG hold is what you actually want for making good a course through current and
leeway. See the design note below for how it's done safely.

## Run the tests

```bash
go test ./...
```

Unit tests cover the heading math, PID damping features, ram calibration,
Nomoto auto-tune, NMEA parsing, config round-trip, and **closed-loop
convergence** for both heading-hold and the COG cascade (including the
crab-into-current behaviour and the heading-drifts-under-current contrast).

## Deploy to the Raspberry Pi

```bash
# cross-compile (Pi 3/4/Zero 2 = arm64; older Pi/Zero = GOARCH=arm GOARM=6/7)
GOOS=linux GOARCH=arm64 go build -o autopilot ./cmd/autopilot

# on the Pi:
sudo ./autopilot -device /dev/ttyAMA0 -baud 4800 \
     -config /etc/boatbrain/autopilot.json \
     -stbd-pin 23 -port-pin 24 -cog 90
```

`-heading H` holds a compass heading; `-cog C` holds a ground course; neither =
hold whatever heading it sees first. `Ctrl-C`/SIGTERM releases the helm cleanly.

---

## How it steers, and why it doesn't hunt

A naive proportional controller makes a boat *hunt* — weave across the course,
sawing the helm. The anti-hunting design mirrors commercial pilots and the
ship-steering literature:

| Technique | Package | Purpose |
|---|---|---|
| **Counter-rudder** (derivative on yaw rate) | `pid` | opposite rudder as the bow swings toward target — kills overshoot |
| **Deadband** (soft) + **motor deadband** | `pid`, `ram` | ignore tiny wander / sub-degree commands so the helm doesn't chatter |
| **Compass input filtering** | `controller` | don't feed the derivative term compass noise |
| **Anti-windup** + **PD-during-turn, PID-for-keeping** | `pid` | integral can't wind up during a big turn and overshoot |
| **Output slew-rate limit** | `pid` | smooth, rate-limited tiller inputs |
| **Speed-scheduled gains** | `controller` | rudder authority ∝ speed² — no hunting at hull speed, no mush when slow |

**Damping is derived, not guessed.** The boat's yaw is modelled as a first-order
**Nomoto** system (`T·ṙ + r = K·δ`); with counter-rudder that's a standard
second-order loop, so `tuning` places the poles for a chosen damping ratio ζ:

```
Kp = ωₙ²·T / K            Kd = (2·ζ·ωₙ·T − 1) / K
```

ζ≈0.9 is the sweet spot (crisp, no overshoot). The ζ slider in the sim shows
0.4 hunt badly and 1.2 go sluggish.

### COG steering (cascade)

COG can't be steered directly — it's noisy, lags, and is meaningless at low
speed. So it's a **cascade**:

- **Inner loop** — the fast heading-hold PID (above).
- **Outer loop** — a slow controller that nudges the *target heading* until the
  actual COG matches the desired ground course. The offset it settles on is the
  **crab angle** needed to counter current and leeway. It's deliberately slow
  (~40 s) for time-scale separation from the inner loop (~8 s), integrates only
  near course (no windup during acquisition), and **falls back to heading-hold
  below a speed threshold**, where COG is unusable.

---

## Setup & calibration

The boat-specific setup lives in the ram `Calibration` (persisted in the config
JSON):

- **Mount side** (`mount_side: port|starboard`) — sets the drive sense. If the
  boat turns the wrong way, flip this one field; don't rewire.
- **Centre + equal travel** — from a dockside end-stop sweep, centre is the
  midpoint and `symmetric_travel` clamps usable stroke to the smaller half so
  the helm has identical authority to port and starboard.
- **Motor deadband** (`deadband_stroke`) — the ram ignores sub-degree commands
  so it doesn't chatter.

`config.example.json` is a complete, valid config you can copy and hand-edit.

## Layout

```
cmd/simulator      interactive web sim (embeds the UI)
cmd/autopilot      on-boat binary: NMEA serial -> controller -> GPIO ram
internal/heading   wrap-safe compass math (shortest-arc error, COG from vector)
internal/pid       damped heading PID (counter-rudder, deadband, anti-windup…)
internal/tuning    Nomoto pole-placement auto-tune + K,T estimation
internal/ram       ram calibration + SimRam + Pi GPIO driver (gpio_linux.go)
internal/nmea      NMEA 0183 parser + serial reader (serial_linux.go)
internal/controller  CourseKeeper: heading-hold + COG cascade
internal/boatsim   first-order Nomoto boat model w/ wind, waves, current, COG
internal/simserver Go SSE server + embedded canvas UI
internal/config    config structs + JSON persistence
reference/python   the earlier Python prototype (control core + sim harness)
```

## Hardware notes (Pi)

- **Ram** — two GPIO lines through an H-bridge/relay board (`-stbd-pin`,
  `-port-pin`); mount-side sign is applied in software. Rudder-reference
  feedback is optional: with none, stroke is dead-reckoned; with a pot, wire it
  through an SPI ADC (e.g. MCP3008) and supply a feedback function to `PiRam`.
- **NMEA** — 4800 baud on `/dev/ttyAMA0` (Pi UART) or a USB-serial GPS/compass
  on `/dev/ttyUSB0`. HDT/HDM/HDG (heading), RMC/VTG (COG+SOG), ROT (rate of
  turn — used directly when present, far cleaner than differentiating a compass).

## Roadmap

Rate-of-turn-limited course changes (turn at a set ROT), an adaptive "sea state"
mode, wind-vane (apparent-wind) steering, waypoint/cross-track steering, and a
gpiochar/SPI-feedback ram driver.
