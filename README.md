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

# verify ram direction + travel on the bench FIRST (drives stbd/port/centre):
sudo ./autopilot -rpwm-pin 18 -lpwm-pin 13 -en-pin 12 -test-ram

# then run it (clutch on GPIO6 via a relay/MOSFET; -clutch-pin -1 if none):
sudo ./autopilot -device /dev/ttyAMA0 -baud 4800 \
     -config /etc/boatbrain/autopilot.json \
     -rpwm-pin 18 -lpwm-pin 13 -en-pin 12 -clutch-pin 6 -cog 90
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

## Mapping to the ST4000 manual

Every knob on the Autohelm ST4000 has a direct equivalent here, so bench/sea
experience transfers straight across:

| ST4000 control | Here | Notes |
|---|---|---|
| **Rudder Gain** (Cal 1) | `pid.gains.kp` (auto-tuned) | 40° turn → crisp with 2–5° overshoot = correct; >5° = too high |
| **Rudder Damping** (Cal 13) | counter-rudder `pid.gains.kd` | "set as low as possible without hunting" — exactly what pole-placement does |
| **Automatic Trim** (standing helm) | integral `pid.gains.ki` | slow trim of weather-helm offset ("can take up to a minute") |
| **Auto seastate** (adaptive deadband) | `pid.adaptive_deadband` (+ `deadband_max`) | widens the deadband in a seaway to neglect repetitive movement |
| **Drive / operating phase** | `ram.mount_side` | reverse for a port-mounted actuator; wrong = steers hard over |
| **Off-course alarm** | `off_course_deg` / `off_course_secs` | ST4000 default 20° for 20 s; surfaced in the sim and the `autopilot` log |
| **Rudder reference** (Cal 8) | `ram.Feedback` (SPI ADC) | optional; without it, stroke is dead-reckoned |

The interactive sim shows the effective (auto-seastate) deadband live, and a
pulsing **OFF COURSE** banner when the alarm trips.

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

Target rig: **Raspberry Pi → BTS7960 (IBT-2 / HW-039) H-bridge → Raymarine /
Autohelm ST4000+ drive**. The BTS7960 replaces the course computer's internal
H-bridge; everything else follows the drive interface in the ST4000+ service
manual.

**ST4000+ drive interface → this rig** (from the service manual's connector table):

| ST4000+ pin | Original signal | Wire to |
|---|---|---|
| **MD1 / MD2** | motor drive (PWM'd H-bridge, ~1–2Ω motor, <2A run / ~6.5A stall) | **BTS7960 motor output** (the two motor leads) |
| **C+ / C–** | clutch: "+12V while engaged, else 0V" | **clutch relay/MOSFET** on `-clutch-pin` (C+), C– to 0V |
| **P9 / P10 / P11** | rudder reference 0–5V / 0V / screen | **SPI ADC** → `Feedback` (optional) |
| **NMEA in ± / SeaTalk** | comms | serial `/dev/ttyAMA0` |

**BTS7960 logic side → Pi** (header: RPWM, LPWM, R_EN, L_EN, R_IS, L_IS, VCC, GND):

| BTS7960 | Pi (BCM) | Notes |
|---|---|---|
| VCC / GND | 3V3 / GND | BTS7960 logic runs at 3.3 V; common ground |
| RPWM | GPIO18 (`-rpwm-pin`) | drives toward **starboard** rudder (→ one motor lead) |
| LPWM | GPIO13 (`-lpwm-pin`) | drives toward **port** rudder (→ other motor lead) |
| R_EN + L_EN | GPIO12 (`-en-pin`) | tie together to one pin; `<0` if hardwired to VCC |
| R_IS / L_IS | — | current sense (unused; optional over-current cut-out later) |

The module's big screw terminals take the 12 V battery feed (fuse it for the
drive — a few amps for the tiller motor). Direction is decided in software, so
if the boat turns the wrong way just flip `mount_side` — no rewiring.

- **Clutch (C+/C–)** — the ST4000+ energises the clutch for the *whole* time the
  pilot is in Auto (not just while the motor moves), and drops it on standby so
  the helm is free. The driver does exactly this. It's an inductive 12 V load, so
  drive it through a **relay or logic-level MOSFET with a flyback diode**, never
  straight off the GPIO. `-clutch-pin 6` by default; `-1` for a clutchless drive.
- **Safe power-up** — a Pi's GPIOs are inputs (hi-Z) until the program drives
  them, so fit **pull-down resistors (~10 kΩ) on RPWM, LPWM and EN** (and on the
  clutch driver's gate) to hold everything off at boot. The driver also drives
  them low before enabling, and releases the clutch on exit.
- **Verify phase first** — run `-test-ram` on the bench: it drives starboard,
  then port, then centre. Per the manual, `+rudder` must move the tiller to give
  a **turn to starboard**; if reversed, flip `mount_side` (don't rewire). A
  reversed phase is the classic "steers hard over on engage" failure.
- **Ram speed** — bang-bang (full speed) by default, which is the most reliable.
  `-pwm-hz 200` enables best-effort software PWM so the ram eases off near target
  for a softer landing — matching the original's "variable-length pulses" drive.
- **Rudder reference (0–5V)** — optional true feedback: wire P9/P10 through an
  SPI ADC (e.g. MCP3008; scale 5V→3.3V) and supply a `Feedback` func to `PiRam`.
  Without it, stroke is dead-reckoned.
- **NMEA** — 4800 baud on `/dev/ttyAMA0` (Pi UART) or a USB-serial GPS/compass
  on `/dev/ttyUSB0`. HDT/HDM/HDG (heading), RMC/VTG (COG+SOG), ROT (rate of
  turn — used directly when present, far cleaner than differentiating a compass).

## Roadmap

Rate-of-turn-limited course changes (turn at a set ROT), an adaptive "sea state"
mode, wind-vane (apparent-wind) steering, waypoint/cross-track steering, and a
gpiochar/SPI-feedback ram driver.
