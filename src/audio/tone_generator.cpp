// tone_generator.cpp — ToneGenerator implementation
//
// SPDX-License-Identifier: BSD-3-Clause

#include "tone_generator.h"

#include <algorithm>
#include <cmath>

// ─── Constants ───────────────────────────────────────────────────────────────

static constexpr double  TWO_PI     = 2.0 * M_PI;
static constexpr int16_t SAMPLE_MAX = 32767;

// ─── Construction / configuration ────────────────────────────────────────────

ToneGenerator::ToneGenerator(const ToneConfig& cfg) {
    set_config(cfg);
}

void ToneGenerator::set_config(const ToneConfig& cfg) {
    config_ = cfg;

    // Pre-compute total sample count (0 means run forever).
    total_samples_ = (cfg.duration_ms > 0u)
        ? static_cast<uint32_t>(
              static_cast<uint64_t>(cfg.sample_rate) * cfg.duration_ms / 1000u)
        : 0u;

    reset();
}

// ─── AudioSource interface ────────────────────────────────────────────────────

void ToneGenerator::reset() {
    phase_          = 0.0;
    samples_played_ = 0u;
    done_           = false;
}

bool ToneGenerator::is_done() const {
    return done_;
}

uint32_t ToneGenerator::fill(int16_t* buffer, uint32_t frames) {
    if (done_) return 0u;

    // If we have a finite duration, cap the request at the remaining samples.
    uint32_t to_write = frames;
    if (total_samples_ > 0u) {
        const uint32_t remaining = (samples_played_ < total_samples_)
            ? (total_samples_ - samples_played_) : 0u;
        to_write = std::min(frames, remaining);
        if (to_write == 0u) {
            done_ = true;
            return 0u;
        }
    }

    for (uint32_t f = 0u; f < to_write; ++f) {
        const int16_t s = next_sample();
        buffer[f * 2u]      = s;  // Left  channel
        buffer[f * 2u + 1u] = s;  // Right channel (same — mono alert tone)
    }

    samples_played_ += to_write;
    if (total_samples_ > 0u && samples_played_ >= total_samples_) {
        done_ = true;
    }

    return to_write;
}

// ─── Sample synthesis ────────────────────────────────────────────────────────

int16_t ToneGenerator::next_sample() {
    double value = 0.0;

    switch (config_.wave_type) {
        case WaveType::Sine:
            value = std::sin(phase_ * TWO_PI);
            break;

        case WaveType::Square:
            value = (phase_ < 0.5) ? 1.0 : -1.0;
            break;

        case WaveType::Triangle:
            // Rises from -1 to +1 over first half, falls back over second half.
            value = (phase_ < 0.5)
                ? (4.0 * phase_ - 1.0)
                : (3.0 - 4.0 * phase_);
            break;
    }

    // Advance phase, wrap into [0, 1).
    phase_ += static_cast<double>(config_.frequency_hz) /
              static_cast<double>(config_.sample_rate);
    if (phase_ >= 1.0) phase_ -= 1.0;

    return static_cast<int16_t>(
        value * static_cast<double>(config_.amplitude) * SAMPLE_MAX);
}
