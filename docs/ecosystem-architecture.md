# boat-brain ecosystem architecture

How the autopilot, the coordinator, and the panels fit into one system — and
where the planned extensions (phone/watch remote, extra steering modes) slot in.

## Components

| Repo | Lang | Role |
|---|---|---|
| **boat-brain** | Go | Central **non-real-time hub** on a Pi (Docker). N2K gateway, panel coordinator, sensor fusion, hosts/coordinates the autopilot. Early scaffold today. |
| **boat-brain-audio-alerter** (this) | Go | The **autopilot**: damped PID + Nomoto auto-tune + COG cascade, ram driver, simulator. Control core is reusable as a library. |
| **boat-brain-panel** | C++ (Pico SDK) | **Panel/knob firmware** on RP2040 / Feather-CAN. Serial slot protocol to the brain, encoder input, display renderers (smart-knob renderer is a stub). |

## Topology

```
        NMEA 2000 backbone (CAN 250k) ── marine sensors, MFDs, wind, GPS, compass
                     │  (SocketCAN, MCP2515 HAT)
        ┌────────────┴─────────────────────────────┐
        │              boat-brain  (Go, Pi)          │
        │  ┌───────────┐  ┌───────────┐  ┌────────┐  │
   ┌────┼──┤ panel link│  │ N2K gw    │  │ remote │  │
 panels │  │ (serial)  │  │ consume + │  │ BLE/WS │◄─┼── phone / watch
 (Pico) │  │           │  │ publish   │  │ adapter│  │
        │  └─────┬─────┘  └─────┬─────┘  └────────┘  │
        │        │   control contract   │            │
        │        └────────┬────────────┘            │
        │           ┌─────▼──────┐                   │
        │           │  AUTOPILOT │  (imported library)│
        │           │ CourseKeeper + modes            │
        │           └─────┬──────┘                   │
        └─────────────────┼──────────────────────────┘
                          │ rudder angle
                    ram driver (BTS7960 GPIO)  ── rod-tiller actuator
```

One rule holds everything together: **there is a single control contract**
(autopilot state + commands). Every transport — panel serial, N2K PGNs, BLE,
the web simulator — is just an *adapter* onto that one contract, and the
autopilot is the single source of truth. Two knobs, a phone, and an MFD can all
watch/command it and never disagree.

## Where the autopilot runs

**Recommendation: the control loop runs on the Pi, as a Go library imported by
`boat-brain`.** Rationale:

- Both are Go; the autopilot's `controller`/`pid`/`ram`/`tuning`/`heading`/
  `config` packages become an importable library and `boat-brain` runs the loop
  as one subsystem. (They must move out of `internal/` — Go blocks cross-module
  `internal` imports — into an importable module path.)
- A 10 Hz PID with a slew-limited output is **not hard-real-time**; it sits
  comfortably in "high-level" territory. The genuinely timing-critical work
  (encoder quadrature, display refresh) is already on the Picos.
- The one sub-loop with real timing sensitivity is **ram PWM**. Default drive is
  bang-bang (no PWM). If we want strict timing later, delegate *only* the PWM to
  a tiny MCU (or the BTS7960) while the Pi sends rudder-angle commands — the
  `ram.Actuator` interface already draws that seam.

`boat-brain`'s own doc says "timing-critical control remains on dedicated MCUs";
the 10 Hz steering law is not that. The `cmd/autopilot` binary stays in this
repo for bench/standalone use; in production the loop lives inside `boat-brain`.

## The control contract

Transport-agnostic. This is the one thing every adapter and both firmwares
build against.

### State (autopilot → everyone)

| Field | Type | Notes |
|---|---|---|
| `mode` | enum | standby, heading, cog, windvane, into-wind, tack |
| `engaged` | bool | actively driving the helm |
| `desired` | deg | setpoint in the mode's reference (heading / COG / wind angle) |
| `actual` | deg | actual value in that reference |
| `error` | deg | signed shortest-arc, +stbd |
| `heading` / `cog` / `sog` | deg / kn | always present for the compass rose |
| `rudder` | deg | commanded rudder (+stbd) |
| `steer_dir` | enum | port / centre / stbd — the "hunting" arrow (sign of rudder) |
| `lock` | enum | **locked / close / hunting** — green / yellow / red |
| `off_course` | bool | ST4000-style alarm (>limit for >delay) |

**Lock classifier** (shared Go helper, so sim + knobs colour identically):
`locked` when `|error| ≤ deadband`; `close` when `≤ ~10°`; `hunting` beyond.

### Commands (anyone → autopilot)

`engage` · `standby` · `set_mode(mode)` · `set_target(deg)` ·
`adjust_target(±deg)` · `tack(port|stbd)`. Relative `adjust` composes across
multiple controllers; absolute `set` is last-writer-wins.

### Expressed over each transport

- **Panel protocol**: two new `MessageType`s — `AutopilotState` (brain→panel)
  and `AutopilotCommand` (panel→brain) — packed structs alongside the existing
  `SlotAssignment`/`ParameterUpdate`. The panel runs the *menu* locally and
  sends **semantic** commands (not raw ticks).
- **NMEA 2000**: consume standard PGNs (127250 heading, 127251 ROT, 129026
  COG/SOG, 130306 wind, 127245 rudder); publish standard **127237** Heading/Track
  Control for MFD interop; use **proprietary boat-brain PGNs** for the command +
  rich UX telemetry (steer_dir, lock, mode) that no standard carries.
- **BLE / WebSocket**: the same state/command as JSON (WS reuses the simulator's
  shape) or a GATT service (BLE) — a thin adapter, no core changes.

## Panel autopilot screen (maps your UX spec)

Nested menu, all logic local to the panel for snappiness; identical firmware on
every knob:

- **Top level** (jog to scroll): Fuel · Oil · **Autopilot** · … Fuel/Oil are
  read-only slots (existing model). Autopilot is the new interactive screen.
- **Autopilot glance**: compass rose (exists) + **actual-course pointer** +
  **desired-course triangle** + engaged/disengaged colour + a **port/stbd
  hunting arrow** (`steer_dir`) + desired-arrow colour = `lock` (green/yellow/
  red).
- **Push → interactive**: jog moves focus between {desired-course, engage
  toggle, back}. Push on desired-course → **edit** (jog adjusts, push stores,
  revert up). Push on toggle → engaged/disengaged (clear visual state). Push on
  back → up a level.

This needs the `smart_knob_renderer` (currently a stub) implemented for the
compass screen — the one substantial new piece of panel firmware.

## Steering modes (as strategies)

Each mode is just a "reference → heading setpoint" strategy feeding the existing
inner heading PID. Adding one is a small, isolated change.

| Mode | Setpoint source |
|---|---|
| heading | desired heading (have) |
| cog | slow outer loop crabs to hold ground course (have) |
| **windvane** | hold apparent (or true) wind angle → setpoint tracks wind (needs N2K 130306) |
| **into-wind** | steer to AWA ≈ 0 and hold — for reefing/sail handling |
| **tack** | one-shot manoeuvre: turn through ~100° (ST4000 Autotack) then resume |

The controller's `Mode` becomes a small interface so windvane/into-wind/tack
slot in beside heading/cog without touching the inner loop.

## Extension: phone / watch remote

Nothing special — a **remote adapter** in `boat-brain` exposes the control
contract over **BLE GATT** (watch/phone near the helm) and/or **WebSocket over
WiFi** (phone app). Because it's the same contract the panels use, the phone is
just another controller; the autopilot arbitrates. Auth/safety (e.g. a
"remote-enable" latch so a phone can't engage the pilot unnoticed) lives in that
adapter.

## Build order

1. **Library-ify the autopilot core** — move the control packages to an
   importable path so `boat-brain` can import them; keep `cmd/*` + sim here.
2. **Lock-state classifier + steer_dir** in the Go controller (small; the sim
   uses it too).
3. **Mode strategy refactor** — generalise `Mode`; add windvane/into-wind/tack.
4. **boat-brain integration** — N2K gateway (SocketCAN) + host the control loop;
   panel serial link; the control-contract hub.
5. **Panel autopilot screen** — `AutopilotState`/`AutopilotCommand` messages +
   the real smart-knob compass renderer + the menu state machine.
6. **Remote adapter** — BLE/WS onto the same contract.

## Open decisions

- **Autopilot home**: import into `boat-brain` (recommended) vs keep a separate
  Go service talking over a socket/CAN.
- **Panels on N2K?**: keep serial-to-brain (recommended near-term) vs move panels
  onto the Feather-CAN backbone as independent N2K nodes (more decoupled, more
  work, needs N2K on the Pico via Timo Lappalainen's library).
- **PGN numbers**: pick the proprietary PGN range + a manufacturer code for the
  boat-brain command/telemetry PGNs.
