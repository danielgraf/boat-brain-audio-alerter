#pragma once

// ============================================================================
// Hardware Pin Configuration — MAX98357A I2S Class-D Amplifier
// ============================================================================
//
//  Raspberry Pi Pico 2 (RP2350) → MAX98357A wiring:
//
//   Pico 2 GPIO 26  ──►  DIN   (serial audio data)
//   Pico 2 GPIO 27  ──►  BCLK  (bit clock)
//   Pico 2 GPIO 28  ──►  LRCLK (word-select / left-right clock)
//   3.3 V           ──►  VIN
//   GND             ──►  GND
//   SD_MODE         ──►  VIN   (selects left channel output; see below)
//
//  SD_MODE pin:
//    VIN   → output = left channel
//    float → output = right channel
//    GND   → output = (left + right) / 2
//
//  Change the three GPIO constants below if you wire the board differently.

#include "pico/stdlib.h"

/// GPIO pin for I2S serial data (DIN on MAX98357A).
static constexpr uint AUDIO_DATA_PIN = 26u;

/// GPIO base pin for I2S clocks.
/// BCLK  = AUDIO_CLOCK_PIN_BASE     (GPIO 27)
/// LRCLK = AUDIO_CLOCK_PIN_BASE + 1 (GPIO 28)
static constexpr uint AUDIO_CLOCK_PIN_BASE = 27u;

// ============================================================================
// Audio Parameters
// ============================================================================

/// Output sample rate in Hz.  44 100 Hz covers all standard WAV files.
static constexpr uint32_t AUDIO_SAMPLE_RATE = 44100u;

/// PCM bit depth (only 16-bit is supported).
static constexpr uint AUDIO_BIT_DEPTH = 16u;

/// Number of stereo frames per DMA transfer buffer.
/// 512 frames ≈ 11.6 ms of audio at 44 100 Hz — small enough for
/// low latency, large enough to avoid underruns on the RP2350.
static constexpr uint32_t AUDIO_BUFFER_FRAMES = 512u;
