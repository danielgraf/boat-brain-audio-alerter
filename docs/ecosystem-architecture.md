# boat-brain ecosystem architecture

How the autopilot, the coordinator, and the panels fit into one system — and
where the planned extensions (phone/watch remote, extra steering modes) slot in.

## Components

| Repo | Lang | Role |
|---|---|---|
| **boat-brain** | Go | Central **non-real-time hub** on a Pi (Docker). N2K gateway, panel coordinator, sensor fusion, hosts/coordinates the autopilot. Early scaffold today. |
| **boat-brain-audio-alerter** (this) | Go | The **autopilot**: damped PID + Nomoto auto-tune + COG cascade, ram driver, simulator. Control core is reusable as a library. |
| **boat-brain-panel** | C++ (Pico SDK) | **Panel/knob firmware** on RP2040 / Feather-CAN. Serial slot protocol to the brain, encoder input, display renderers (smart-knob renderer is a stub). |
| **audio-alerter** (repurposes the old `boat-brain-audio-alerter` name/intent) | C++ (Pico SDK) | **Sound peripheral**: Pico + amp → exciter/surface-transducer speaker. A command-sink satellite of the brain — plays alert sounds on brain events (off-course, engage/disengage, N2K alarms). |

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

## Peripheral nodes & transport

The brain's satellites are all **RP2040 nodes speaking one framed protocol** over
a transport chosen behind the firmware's `ICommsLink` abstraction — so the wire
is a swap, not a redesign. Two kinds:

- **Panels / knobs** — display + input (slots, encoder). Interactive autopilot
  screen is a new message pair on top.
- **Audio alerter** — a *command sink* (no display): the brain sends
  `PlayAlert(id, priority)` and the Pico drives an exciter speaker. Triggered by
  brain events, first of which is the autopilot's `off_course` alarm plus
  engage/disengage chimes; later, N2K alarms (depth, battery, anchor drag).

Transport tiers (identical protocol across all):

| Scale | Wire | Notes |
|---|---|---|
| Bench + a couple of nodes near the Pi (helm, nav-station, alerter) | **USB CDC** (now) | each Pico = one `/dev/ttyACM*`; powers the Pico; already implemented in `SerialComms` |
| Many / distributed nodes | **RS-485 multidrop** | one differential pair, addressed by `panel_id` (already in the protocol), long robust runs; firmware unchanged |
| Phone / watch "virtual panels" | **WebSocket / BLE** | no wire; same control contract |

USB's real limits on a boat are cable length (~5 m) and connector robustness, so
it's the near-term choice for nodes near the Pi; RS-485 is the growth path.
**N2K stays the *sensor/interop* bus, not the panel bus** — panels use our own
protocol; the brain aggregates N2K and bridges the two.

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

1. ~~**Library-ify the autopilot core**~~ — done: control packages now live in
   `boat-brain/internal/autopilot/…`.
2. ~~**Lock-state classifier + steer_dir**~~ — done, in `.../autopilot/control`.
3. ~~**boat-brain integration** (control loop + contract over HTTP)~~ — done:
   supervisor runs the loop (sim-driven), `GET /autopilot/state` + `POST
   /autopilot/command`.
4. ~~**N2K gateway**~~ — done, in `boat-brain/internal/autopilot/n2k`: SocketCAN
   transport, consumes heading/ROT/COG-SOG/wind/rudder PGNs → supervisor via a
   NavSource; publishes 127237 + proprietary status/command; ISO address
   claiming; BTS7960 PiRam behind `BOAT_BRAIN_RAM=gpio`. Also carries the wind
   for windvane + the gybe guard.
5. **Wind safety + modes** (next, "back to the autopilot") — done so far: wind
   telemetry, sail-zone, gybe/tack guard (refuse-through-wind). To do: windvane /
   into-wind / auto-tack strategies + "steer the long way round" avoidance.
6. **Panel link + autopilot screen** — brain-side panel protocol (USB CDC now,
   RS-485 later) with the new `AutopilotState`/`AutopilotCommand` messages; the
   real smart-knob compass renderer + local menu state machine.
7. **Audio alerter** — `PlayAlert` peripheral + brain-side event→alert dispatch
   (first trigger: the autopilot `off_course` alarm + engage/disengage chimes).
8. **Remote adapter** — BLE/WS onto the same contract.

## Open decisions

- **Panels on N2K?**: keep serial-to-brain (recommended near-term) vs move panels
  onto the Feather-CAN backbone as independent N2K nodes (more decoupled, more
  work, needs N2K on the Pico via Timo Lappalainen's library). **Resolved:**
  panels use our own protocol over USB CDC → RS-485; N2K is the sensor bus only.
- **PGN numbers**: pick the proprietary PGN range + a manufacturer code for the
  boat-brain command/telemetry PGNs.
- **`boat-brain-audio-alerter` repo**: the autopilot has moved into `boat-brain`,
  so this repo can return to its original intent (the audio-alerter sound
  peripheral) — while keeping the simulator + web visualiser as the autopilot
  dev sandbox. Keep as sandbox for now, or start reshaping toward the alerter.
