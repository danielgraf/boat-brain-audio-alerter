#pragma once
// alert_manager.h — Priority-based audio alert scheduler
//
// Manages up to MAX_ALERTS simultaneously registered alerts and selects
// which one to play at any given moment.
//
// Design:
//   • Each alert has an AlertConfig (source, mode, priority, repeat delay).
//   • Alerts are triggered and cancelled independently.
//   • The highest-priority *ready* alert plays.  If a higher-priority alert
//     becomes ready while a lower-priority one is playing, it preempts it.
//   • One-shot alerts deactivate themselves when the source signals is_done().
//   • Repeat alerts reset their source and start a delay countdown; they
//     replay after repeat_delay_ms without any external intervention.
//   • update(elapsed_ms) must be called regularly from the main loop.

#include "alert.h"
#include "audio/i2s_output.h"

#include <cstdint>

/// Maximum number of alerts that can be registered.
static constexpr uint MAX_ALERTS = 8u;

/// Priority-based audio alert scheduler.
class AlertManager {
public:
    AlertManager() = default;

    /// Initialise the manager and the underlying I2S output driver.
    ///
    /// @param i2s         Reference to a constructed (but not yet init'd) I2SOutput.
    /// @param data_pin    GPIO for I2S DIN.
    /// @param clock_base  GPIO base for BCLK/LRCLK.
    /// @param sample_rate Desired sample rate in Hz (e.g. 44100).
    void init(I2SOutput& i2s, uint data_pin, uint clock_base, uint32_t sample_rate);

    /// Register a new alert.
    ///
    /// @param config  Alert configuration.  The source pointer must remain
    ///                valid for as long as the alert is registered.
    /// @return        Alert ID (0 … MAX_ALERTS-1), or -1 if the table is full.
    int register_alert(const AlertConfig& config);

    /// Trigger (activate) an alert so it begins playing.
    /// If the alert is already active this is a no-op (use cancel()+trigger()
    /// to restart a one-shot alert from the beginning).
    void trigger(int alert_id);

    /// Cancel (deactivate) an alert.  If it is currently playing, the next
    /// update() call will stop it and switch to the next ready alert.
    void cancel(int alert_id);

    /// Must be called regularly from the main loop (or a repeating timer).
    ///
    /// @param elapsed_ms  Milliseconds elapsed since the previous call.
    ///                    Used to count down repeat delays.
    void update(uint32_t elapsed_ms);

    /// Called from the I2S DMA IRQ via the AudioFillCallback.
    /// Do not call directly.
    uint32_t fill_audio(int16_t* buffer, uint32_t frames);

    /// @return the ID of the alert currently being played, or -1 if silent.
    int active_alert_id() const { return active_alert_id_; }

private:
    int  find_highest_priority_ready() const;
    void start_playing(int id);
    void stop_playing();

    Alert      alerts_[MAX_ALERTS] = {};
    uint       alert_count_        = 0u;
    int        active_alert_id_    = -1;   ///< ID of the alert currently audible
    I2SOutput* i2s_                = nullptr;
};
