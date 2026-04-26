#pragma once
// wav_player.h — WAV file playback from flash (or RAM) via the AudioSource interface
//
// Supports 16-bit PCM WAV files stored as byte arrays in flash memory.
// Mono files are automatically duplicated to both output channels.
// Stereo files are passed through unchanged.
//
// Usage:
//   extern const uint8_t alert_wav[];    // declared in a generated .cpp file
//   extern const uint32_t alert_wav_len;
//
//   WavPlayer player;
//   player.load(alert_wav, alert_wav_len);
//   // player now implements AudioSource — pass to AlertManager

#include "audio_source.h"
#include <cstdint>

// ─── WAV header ───────────────────────────────────────────────────────────────

/// Standard 44-byte PCM RIFF/WAV header (no extension chunks assumed for the
/// mandatory fields; find_data_chunk() handles files with extra metadata).
struct WavHeader {
    // RIFF descriptor
    char     riff_id[4];       ///< "RIFF"
    uint32_t riff_size;        ///< File size – 8 bytes
    char     wave_id[4];       ///< "WAVE"

    // "fmt " sub-chunk
    char     fmt_id[4];        ///< "fmt "
    uint32_t fmt_size;         ///< 16 for standard PCM
    uint16_t audio_format;     ///< 1 = Linear PCM
    uint16_t num_channels;     ///< 1 = Mono, 2 = Stereo
    uint32_t sample_rate;      ///< Samples per second
    uint32_t byte_rate;        ///< sample_rate × num_channels × bits/8
    uint16_t block_align;      ///< num_channels × bits/8
    uint16_t bits_per_sample;  ///< Must be 16

    // "data" sub-chunk — may not follow immediately; use find_data_chunk()
    char     data_id[4];       ///< "data"
    uint32_t data_size;        ///< Byte length of PCM payload
} __attribute__((packed));

// ─── WavPlayer ────────────────────────────────────────────────────────────────

/// Plays a 16-bit PCM WAV file stored as a const byte array.
///
/// Limitations:
///   • Only uncompressed (PCM, audio_format=1) 16-bit WAV is supported.
///   • Mono and stereo are both accepted; other channel counts are rejected.
///   • The file's sample rate should match AUDIO_SAMPLE_RATE; no resampling
///     is performed (mismatched rates will play at the wrong pitch/speed).
class WavPlayer : public AudioSource {
public:
    WavPlayer() = default;

    /// Load a WAV file from a pointer to its data in memory.
    ///
    /// @param data  Pointer to the raw WAV bytes (may be in flash).
    /// @param size  Total byte length of the WAV data.
    /// @return true if the header parsed successfully and the format is
    ///         supported; false otherwise.
    bool load(const uint8_t* data, uint32_t size);

    /// @return true if a valid file has been loaded.
    bool is_loaded() const { return loaded_; }

    /// Approximate duration of the loaded file in milliseconds.
    uint32_t duration_ms() const;

    // Accessor helpers (valid only after a successful load())
    uint32_t sample_rate()     const { return header_.sample_rate; }
    uint16_t num_channels()    const { return header_.num_channels; }
    uint16_t bits_per_sample() const { return header_.bits_per_sample; }

    // ── AudioSource interface ──────────────────────────────────────────────
    uint32_t fill(int16_t* buffer, uint32_t frames) override;
    void     reset() override;
    bool     is_done() const override { return done_; }

private:
    const uint8_t* wav_data_   = nullptr;
    uint32_t       wav_size_   = 0u;
    uint32_t       pcm_offset_ = 0u;  ///< Byte offset of PCM payload in wav_data_
    uint32_t       pcm_size_   = 0u;  ///< Byte length of PCM payload
    uint32_t       read_pos_   = 0u;  ///< Current read position (bytes from pcm_offset_)

    WavHeader header_  = {};
    bool      loaded_  = false;
    bool      done_    = false;
};
