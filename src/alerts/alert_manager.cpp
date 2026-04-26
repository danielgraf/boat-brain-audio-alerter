// alert_manager.cpp — AlertManager implementation
//
// SPDX-License-Identifier: BSD-3-Clause

#include "alert_manager.h"

#include <algorithm>

// ─── init ────────────────────────────────────────────────────────────────────

void AlertManager::init(I2SOutput& i2s,
                        uint data_pin,
                        uint clock_base,
                        uint32_t sample_rate) {
    i2s_ = &i2s;

    // Bind the audio fill callback so the DMA IRQ calls back into us.
    i2s.init(data_pin, clock_base, sample_rate,
             [this](int16_t* buf, uint32_t frames) -> uint32_t {
                 return fill_audio(buf, frames);
             });
}

// ─── register_alert ──────────────────────────────────────────────────────────

int AlertManager::register_alert(const AlertConfig& config) {
    if (alert_count_ >= MAX_ALERTS) return -1;

    const int id = static_cast<int>(alert_count_++);
    alerts_[id].config              = config;
    alerts_[id].active              = false;
    alerts_[id].playing             = false;
    alerts_[id].repeat_countdown_ms = 0u;
    return id;
}

// ─── trigger ─────────────────────────────────────────────────────────────────

void AlertManager::trigger(int alert_id) {
    if (alert_id < 0 || static_cast<uint>(alert_id) >= alert_count_) return;

    Alert& a = alerts_[alert_id];
    if (a.active) return;  // already running — ignore duplicate trigger

    a.active              = true;
    a.playing             = false;
    a.repeat_countdown_ms = 0u;  // play immediately

    // update(0) picks up the newly active alert and starts playing if eligible.
    update(0u);
}

// ─── cancel ──────────────────────────────────────────────────────────────────

void AlertManager::cancel(int alert_id) {
    if (alert_id < 0 || static_cast<uint>(alert_id) >= alert_count_) return;

    Alert& a    = alerts_[alert_id];
    a.active    = false;
    a.playing   = false;

    if (active_alert_id_ == alert_id) {
        stop_playing();
        // Immediately switch to the next best alert (if any).
        const int next = find_highest_priority_ready();
        if (next >= 0) start_playing(next);
    }
}

// ─── update ──────────────────────────────────────────────────────────────────

void AlertManager::update(uint32_t elapsed_ms) {
    // ── Tick repeat countdowns ───────────────────────────────────────────────
    for (uint i = 0u; i < alert_count_; ++i) {
        Alert& a = alerts_[i];
        if (a.active && !a.playing && a.repeat_countdown_ms > 0u) {
            a.repeat_countdown_ms =
                (elapsed_ms >= a.repeat_countdown_ms)
                    ? 0u
                    : (a.repeat_countdown_ms - elapsed_ms);
        }
    }

    // ── Handle the currently playing alert ──────────────────────────────────
    if (active_alert_id_ >= 0) {
        Alert& a    = alerts_[active_alert_id_];
        auto*  src  = a.config.source;

        if (src && src->is_done()) {
            if (a.config.mode == AlertMode::Repeat && a.active) {
                // Schedule the next repetition.
                a.playing             = false;
                a.repeat_countdown_ms = a.config.repeat_delay_ms;
                src->reset();
            } else {
                // One-shot finished (or alert was cancelled mid-play).
                a.active  = false;
                a.playing = false;
            }
            stop_playing();
        }
    }

    // ── Start the highest-priority ready alert ───────────────────────────────
    if (active_alert_id_ < 0) {
        const int best = find_highest_priority_ready();
        if (best >= 0) start_playing(best);
    } else {
        // Preempt a lower-priority alert if a higher-priority one is ready.
        const int best = find_highest_priority_ready();
        if (best >= 0 && best != active_alert_id_) {
            const auto cur_pri  = alerts_[active_alert_id_].config.priority;
            const auto best_pri = alerts_[best].config.priority;
            if (best_pri > cur_pri) {
                stop_playing();
                start_playing(best);
            }
        }
    }
}

// ─── fill_audio ──────────────────────────────────────────────────────────────

uint32_t AlertManager::fill_audio(int16_t* buffer, uint32_t frames) {
    if (active_alert_id_ < 0) return 0u;

    auto* src = alerts_[active_alert_id_].config.source;
    if (!src) return 0u;

    return src->fill(buffer, frames);
}

// ─── Private helpers ─────────────────────────────────────────────────────────

int AlertManager::find_highest_priority_ready() const {
    int  best     = -1;
    int  best_pri = -1;

    for (uint i = 0u; i < alert_count_; ++i) {
        const Alert& a = alerts_[i];
        // "Ready" means: active, not currently playing, and delay expired.
        if (a.active && !a.playing && a.repeat_countdown_ms == 0u) {
            const int pri = static_cast<int>(a.config.priority);
            if (pri > best_pri) {
                best_pri = pri;
                best     = static_cast<int>(i);
            }
        }
    }
    return best;
}

void AlertManager::start_playing(int id) {
    Alert& a = alerts_[id];
    a.playing = true;
    if (a.config.source) {
        a.config.source->reset();
    }
    active_alert_id_ = id;
}

void AlertManager::stop_playing() {
    active_alert_id_ = -1;
}
