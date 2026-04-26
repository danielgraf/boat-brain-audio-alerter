# boat-brain-audio-alerter

Firmware for a **Raspberry Pi Pico 2 (RP2350)** that drives an
**Adafruit MAX98357A I2S Class-D amplifier** to play configurable audio
warning alerts — tones or WAV files — for use on a boat (low oil pressure,
high coolant temperature, alternator fault, etc.).

---

## Features

| Feature | Details |
|---|---|
| **I2S output** | PIO + DMA ping-pong, 44 100 Hz 16-bit stereo |
| **Tone generator** | Sine, square, or triangle wave at any frequency |
| **WAV playback** | 16-bit PCM mono/stereo files stored in flash |
| **One-shot alerts** | Play once, then stop |
| **Repeating alerts** | Play, wait a configurable delay, repeat until cancelled |
| **Priority** | Four levels (Low/Normal/High/Critical); higher preempts lower |
| **Up to 8 alerts** | Multiple simultaneous registrations, one plays at a time |

---

## Hardware wiring

```
Raspberry Pi Pico 2          MAX98357A breakout
──────────────────           ──────────────────
GPIO 26  ──────────────────►  DIN   (serial audio data)
GPIO 27  ──────────────────►  BCLK  (bit clock)
GPIO 28  ──────────────────►  LRC   (word-select / left-right clock)
3.3 V    ──────────────────►  VIN
GND      ──────────────────►  GND
3.3 V    ──────────────────►  SD    (SD_MODE → VIN selects left-channel output)
```

> **SD_MODE options**
> - VIN (3.3 V) → output = left channel *(default wiring above)*
> - Float        → output = right channel
> - GND          → output = (left + right) / 2

Pin numbers can be changed in [`src/config.h`](src/config.h).

---

## Project layout

```
CMakeLists.txt          CMake build file
pico_sdk_import.cmake   SDK locator (set PICO_SDK_PATH env var)
src/
  config.h              Pin definitions and audio constants
  main.cpp              Entry point + demo alert sequence
  audio/
    i2s_output.pio      PIO state-machine program (I2S bit-banging)
    i2s_output.h/.cpp   DMA + PIO I2S driver
    audio_source.h      Abstract AudioSource interface
    tone_generator.h/.cpp   Sine / square / triangle tone generator
    wav_player.h/.cpp   16-bit PCM WAV player (flash or RAM)
  alerts/
    alert.h             AlertConfig / Alert structs
    alert_manager.h/.cpp  Priority scheduler, repeat logic
```

---

## Building

### Prerequisites

- [Raspberry Pi Pico SDK v2.x](https://github.com/raspberrypi/pico-sdk)
  (SDK ≥ 1.5 also works; set `PICO_PLATFORM=rp2350` manually for older SDKs)
- CMake ≥ 3.13
- An ARM cross-compiler: `arm-none-eabi-gcc`

### Steps

```bash
# 1. Point CMake at the Pico SDK
export PICO_SDK_PATH=/path/to/pico-sdk

# 2. Configure
mkdir build && cd build
cmake .. -DPICO_BOARD=pico2

# 3. Build
make -j$(nproc)

# 4. Flash — hold BOOTSEL, connect USB, then copy the UF2
cp boat_brain_audio_alerter.uf2 /media/$USER/RPI-RP2/
```

---

## Adding a WAV file alert

1. Prepare a 16-bit, 44 100 Hz, mono or stereo WAV file.
2. Convert it to a C byte array:
   ```bash
   xxd -i my_alert.wav > src/audio/my_alert_wav.cpp
   ```
3. Declare the symbols in a header and include it in `main.cpp`.
4. Create a `WavPlayer`, call `load()`, and register it as an alert:
   ```cpp
   static WavPlayer wav;
   wav.load(my_alert_wav, my_alert_wav_len);
   int id = g_alerts.register_alert({
       .source          = &wav,
       .mode            = AlertMode::OneShot,
       .priority        = AlertPriority::High,
       .repeat_delay_ms = 0u,
   });
   g_alerts.trigger(id);
   ```

---

## Alert API quick reference

```cpp
// Register
int id = manager.register_alert({
    .source          = &my_tone,          // AudioSource*
    .mode            = AlertMode::Repeat, // OneShot or Repeat
    .priority        = AlertPriority::High,
    .repeat_delay_ms = 1000u,             // ms between repeats
});

// Control
manager.trigger(id);   // start playing
manager.cancel(id);    // stop and deactivate

// Must be called regularly (e.g. every ms in the main loop)
manager.update(elapsed_ms);
```

---

## Tone generator quick reference

```cpp
ToneGenerator beep({
    .frequency_hz = 880.0f,
    .amplitude    = 0.8f,           // 0.0 ... 1.0
    .wave_type    = WaveType::Sine, // Sine, Square, Triangle
    .duration_ms  = 500u,           // 0 = infinite
    .sample_rate  = 44100u,
});
```
