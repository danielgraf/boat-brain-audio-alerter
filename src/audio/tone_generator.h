#pragma once
// tone_generator.h — Configurable audio tone generator
//
// Generates sine, square, or triangle wave tones as PCM samples and exposes
// them through the AudioSource interface so they can be fed directly to the
// I2S output driver.
//
// Features:
//   • Configurable frequency, amplitude, and waveform type.
//   • Fixed duration (milliseconds) or infinite playback.
//   • Both channels carry the same mono signal (suitable for MAX98357A).

#include "audio_source.h"
#include <cstdint>

// ─── Waveform type ────────────────────────────────────────────────────────────

/// Waveform shape produced by ToneGenerator.
enum class WaveType : uint8_t {
    Sine,      ///< Smooth sinusoidal wave — least harsh, good for mild alerts.
    Square,    ///< Hard-clipped square wave — loud and attention-grabbing.
    Triangle,  ///< Triangular wave — softer harmonic content than square.
};

// ─── Tone configuration ───────────────────────────────────────────────────────

/// Parameters that fully describe a single tone burst.
struct ToneConfig {
    float    frequency_hz  = 1000.0f;  ///< Frequency in Hz (e.g. 880.0f).
    float    amplitude     = 0.8f;     ///< Peak amplitude 0.0 … 1.0.
    WaveType wave_type     = WaveType::Sine;
    /// Duration in milliseconds.  Set to 0 for an infinite (looping) tone.
    uint32_t duration_ms   = 500u;
    /// Sample rate — must match the I2S output sample rate (see config.h).
    uint32_t sample_rate   = 44100u;
};

// ─── ToneGenerator ────────────────────────────────────────────────────────────

/// Generates audio tones as 16-bit stereo PCM data.
class ToneGenerator : public AudioSource {
public:
    /// Construct with an initial configuration.
    explicit ToneGenerator(const ToneConfig& config);

    /// Replace the current configuration (implicitly calls reset()).
    void set_config(const ToneConfig& config);

    /// Read back the active configuration.
    const ToneConfig& config() const { return config_; }

    // ── AudioSource interface ──────────────────────────────────────────────
    uint32_t fill(int16_t* buffer, uint32_t frames) override;
    void     reset() override;
    bool     is_done() const override;

private:
    int16_t  next_sample();

    ToneConfig config_;
    double     phase_          = 0.0;  ///< Current waveform phase in [0, 1).
    uint32_t   samples_played_ = 0u;   ///< Samples generated so far.
    uint32_t   total_samples_  = 0u;   ///< Total samples for duration (0 = ∞).
    bool       done_           = false;
};
