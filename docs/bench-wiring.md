# Bench-test wiring

Goal: drive the rod-type tiller actuator from the Raspberry Pi through the
BTS7960 (IBT-2 / HW-039), on the bench, safely, before anything goes near the
boat. You need: bench PSU (12 V), Raspberry Pi + its own 5 V supply, the BTS7960
module, and the actuator motor. (The Anders TFT knob comes later — pins are
reserved for it below.)

## Two separate power domains

```
  bench PSU 12V ───────────────► BTS7960  B+/B-  (motor power)
                                    │  M+/M-  ──► actuator motor (2 leads)
                                    │
   Pi 5V USB supply ──► Raspberry Pi ── 3V3 ─► BTS7960 VCC (logic)
                                    └── GPIO ─► RPWM / LPWM / EN
                                    └── GND ──► BTS7960 GND  ◄─ also PSU 0V
                                                (ONE common ground)
```

Rules that matter:

- **Power the Pi from its own 5 V supply**, never from the 12 V PSU and never
  back-fed from the BTS7960. The 12 V only ever touches the BTS7960 power
  terminals.
- **One common ground.** Pi GND, BTS7960 logic GND, and the PSU 0 V must all be
  tied together, or the logic signals have no reference and the module behaves
  randomly.
- **BTS7960 logic VCC → Pi 3V3** (not 5 V), so the input thresholds match the
  Pi's 3.3 V GPIO.

## Connections

### BTS7960 8-pin logic header → Pi

| BTS7960 pin | Pi (BCM / physical) | Purpose |
|---|---|---|
| VCC | 3V3  (pin 1) | logic supply, 3.3 V |
| GND | GND  (pin 6) | common ground |
| RPWM | GPIO18 (pin 12) | drive toward **starboard** rudder |
| LPWM | GPIO13 (pin 33) | drive toward **port** rudder |
| R_EN | GPIO12 (pin 32) | enable — tie R_EN + L_EN together to this one pin |
| L_EN | GPIO12 (pin 32) | (tied to R_EN) |
| R_IS | — | current sense, leave unconnected |
| L_IS | — | current sense, leave unconnected |

Add **10 kΩ pull-down resistors from RPWM, LPWM and EN to GND**. At boot the
Pi's GPIOs are floating inputs; the pull-downs hold the bridge off until the
program drives the pins.

### BTS7960 screw terminals (check your board's silkscreen)

| Terminal (common labels) | Wire to |
|---|---|
| Battery **+** (B+ / VCC / +) | bench PSU **+12 V** |
| Battery **−** (B− / GND / −) | bench PSU **0 V** (and Pi GND) |
| Motor **M+** | actuator motor lead 1 (out) |
| Motor **M−** | actuator motor lead 2 (in) |

The actuator's third pin is unused — leave it disconnected. Motor polarity
(which lead is M+/M−) sets the direction; you don't need to get it "right" —
`mount_side` in the config flips it in software (see first-run below).

### Bench PSU settings

- **12 V.**
- **Start with the current limit low — ~1 A.** The tiller motor draws well
  under 2 A running; a wiring fault or a stalled/blocked rod will trip the limit
  instead of cooking something. Raise it to ~5 A once the direction test passes
  and the rod moves freely end to end.
- If your PSU can't current-limit, put a **3–5 A fuse** in the +12 V line.

## First power-up (do this in order)

1. Wire everything with **both supplies off**. Double-check the single common
   ground and that 12 V goes *only* to the BTS7960 power terminals.
2. Leave the actuator rod **free to move** its full travel (not bolted to a
   fixed tiller yet) so it can't stall.
3. Power the **Pi** first, let it boot. GPIOs + pull-downs hold the bridge off.
4. Bring up the **12 V PSU** at the low current limit.
5. Build/run the ram self-test (drives starboard → port → centre, then exits):
   ```bash
   GOOS=linux GOARCH=arm64 go build -o autopilot ./cmd/autopilot   # on a dev box
   # copy to the Pi, then on the Pi:
   sudo ./autopilot -rpwm-pin 18 -lpwm-pin 13 -en-pin 12 -test-ram
   ```
6. Watch the rod and the PSU current:
   - **First move should push the tiller to give a turn to STARBOARD.** If it
     goes the other way, set `mount_side` to the other side in the config (or
     just swap M+/M−) — don't rewire the logic.
   - Current should sit **under ~2 A** while moving and only spike briefly at
     the end stops. A sustained high current = a mechanical bind or a stall;
     cut power and check.
7. When happy, that's the drive verified. The full pilot then runs with an NMEA
   feed (below).

## Feeding NMEA on the bench (for the full loop, not the motor test)

The `-test-ram` step above needs no NMEA. To exercise the *whole* pilot on the
bench you need a heading source:

- **Easiest:** run the interactive simulator on a laptop instead —
  `go run ./cmd/simulator` — to watch the control behaviour without any boat
  sensors. The Pi/motor bench test and the simulator together cover both halves.
- **On the Pi:** feed recorded or synthetic NMEA into a serial port. A
  USB-serial adapter shows up as `/dev/ttyUSB0`; the Pi's own UART is
  `/dev/ttyAMA0` on **GPIO14 (TXD, pin 8) / GPIO15 (RXD, pin 10)**, 4800 baud.
  You can replay a log line-by-line into that port.

## Pins reserved for the Anders TFT knob (next step)

The knob will be the control interface (engage/standby, mode, dial a course).
Keep these free now so it drops in without moving the motor wiring:

| Bus | Pins (BCM / physical) | Likely use |
|---|---|---|
| SPI0 | GPIO10 MOSI (19), GPIO9 MISO (21), GPIO11 SCLK (23), GPIO8 CE0 (24) | TFT display |
| I2C1 | GPIO2 SDA (3), GPIO3 SCL (5) | encoder / touch |
| spare | GPIO23 (16), GPIO24 (18), GPIO25 (22) | encoder A/B + button, if not I2C |
| UART | GPIO14 (8), GPIO15 (10) | NMEA in — keep for the sensor feed |

The motor pins (GPIO12/13/18) deliberately avoid all of these, so the knob and
the NMEA feed can be added without touching the drive wiring.

## Quick safety checklist

- [ ] Pi on its own 5 V supply; 12 V only on the BTS7960 power terminals
- [ ] One common ground (Pi GND ↔ BTS7960 GND ↔ PSU 0 V)
- [ ] BTS7960 VCC on 3V3, not 5 V
- [ ] 10 kΩ pull-downs on RPWM / LPWM / EN
- [ ] PSU current limit ~1 A for the first run (or a 3–5 A fuse)
- [ ] Actuator free to move, not stalled
- [ ] `-test-ram` first; confirm starboard/port before trusting it to steer
