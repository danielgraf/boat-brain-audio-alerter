#pragma once
// alert.h — Alert configuration and state structures
//
// An "alert" couples an AudioSource (tone or WAV) with playback behaviour
// (one-shot vs. repeating) and a priority level for preemption.

#include "audio/audio_source.h"
#include <cstdint>

// ─── Playback mode ────────────────────────────────────────────────────────────

/// Controls what happens when the audio source finishes.
enum class AlertMode : uint8_t {
    /// Play the audio once, then stop (alert deactivates automatically).
    OneShot,
    /// After playing, wait repeat_delay_ms, then replay — indefinitely until
    /// the alert is cancelled with AlertManager::cancel().
    Repeat,
};

// ─── Priority ─────────────────────────────────────────────────────────────────

/// Numeric priority for preemption: a higher-priority alert interrupts a
/// lower-priority one that is currently playing.
enum class AlertPriority : uint8_t {
    Low      = 0,
    Normal   = 1,
    High     = 2,
    Critical = 3,
};

// ─── AlertConfig ─────────────────────────────────────────────────────────────

/// Immutable configuration for an alert, supplied at registration time.
struct AlertConfig {
    /// Pointer to the AudioSource that provides the sound.
    /// The AlertManager does NOT take ownership — the caller must keep the
    /// source alive for the lifetime of the registered alert.
    AudioSource* source = nullptr;

    AlertMode     mode             = AlertMode::OneShot;
    AlertPriority priority         = AlertPriority::Normal;

    /// Time (ms) to wait between repetitions when mode == AlertMode::Repeat.
    /// Ignored for AlertMode::OneShot.
    uint32_t      repeat_delay_ms  = 1000u;
};

// ─── Alert (internal state) ───────────────────────────────────────────────────

/// Runtime state tracked by the AlertManager for each registered alert.
struct Alert {
    AlertConfig config;

    /// True while the alert has been triggered and not yet cancelled.
    bool     active              = false;

    /// True while this alert's audio source is the one currently playing.
    bool     playing             = false;

    /// Countdown (ms) until next repetition; 0 means "ready to play".
    uint32_t repeat_countdown_ms = 0u;
};
