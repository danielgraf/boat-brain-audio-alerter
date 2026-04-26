// main.cpp — Boat Brain Audio Alerter — entry point
//
// Demonstrates the alert system with three example alert types:
//
//   Alert 0 — Low oil pressure  (HIGH priority, repeating 880 Hz beep)
//   Alert 1 — High temperature  (NORMAL priority, repeating 660 Hz beep)
//   Alert 2 — Alternator fault  (CRITICAL priority, one-shot 1320 Hz siren)
//
// In a final installation each alert would be triggered by a GPIO input
// connected to an engine/electrical sensor.  For demonstration purposes
// the alerts are triggered in sequence from the main loop.
//
// Hardware:
//   Raspberry Pi Pico 2 (RP2350) + Adafruit MAX98357A I2S amplifier
//   GPIO 26 → DIN   (serial data)
//   GPIO 27 → BCLK  (bit clock)
//   GPIO 28 → LRCLK (word-select / left-right clock)
//
// SPDX-License-Identifier: BSD-3-Clause

#include "pico/stdlib.h"
#include "pico/time.h"

#include "config.h"
#include "audio/i2s_output.h"
#include "audio/tone_generator.h"
#include "audio/wav_player.h"
#include "alerts/alert_manager.h"

// ─── Optional: WAV file example ──────────────────────────────────────────────
//
// To use a WAV file alert, convert a 16-bit 44100 Hz mono or stereo WAV to a
// C array (e.g. with `xxd -i alert.wav > src/audio/alert_wav.cpp`) and
// uncomment the two lines below.  The array must be declared in a separate
// .cpp file or a generated header.
//
// extern const uint8_t  g_alert_wav[];
// extern const uint32_t g_alert_wav_len;

// ─── Globals ─────────────────────────────────────────────────────────────────

static I2SOutput    g_i2s;
static AlertManager g_alerts;

// Tone sources for the three demo alerts.
static ToneGenerator g_tone_oil({
    .frequency_hz = 880.0f,
    .amplitude    = 0.8f,
    .wave_type    = WaveType::Sine,
    .duration_ms  = 400u,       // 400 ms beep …
    .sample_rate  = AUDIO_SAMPLE_RATE,
});

static ToneGenerator g_tone_temp({
    .frequency_hz = 660.0f,
    .amplitude    = 0.7f,
    .wave_type    = WaveType::Triangle,
    .duration_ms  = 600u,       // 600 ms warble …
    .sample_rate  = AUDIO_SAMPLE_RATE,
});

// Siren effect: a single 300 ms burst played once.
static ToneGenerator g_tone_alternator({
    .frequency_hz = 1320.0f,
    .amplitude    = 0.9f,
    .wave_type    = WaveType::Square,
    .duration_ms  = 300u,
    .sample_rate  = AUDIO_SAMPLE_RATE,
});

// ─── Alert IDs ────────────────────────────────────────────────────────────────

static int g_alert_oil        = -1;
static int g_alert_temp       = -1;
static int g_alert_alternator = -1;

// ─── main ─────────────────────────────────────────────────────────────────────

int main() {
    stdio_init_all();

    // ── Initialise I2S and alert manager ─────────────────────────────────────
    g_alerts.init(g_i2s, AUDIO_DATA_PIN, AUDIO_CLOCK_PIN_BASE, AUDIO_SAMPLE_RATE);

    // ── Register alerts ───────────────────────────────────────────────────────

    // Low oil pressure — repeating beep, 500 ms between repetitions.
    g_alert_oil = g_alerts.register_alert({
        .source          = &g_tone_oil,
        .mode            = AlertMode::Repeat,
        .priority        = AlertPriority::High,
        .repeat_delay_ms = 500u,
    });

    // High coolant temperature — repeating warble, 1 s between repetitions.
    g_alert_temp = g_alerts.register_alert({
        .source          = &g_tone_temp,
        .mode            = AlertMode::Repeat,
        .priority        = AlertPriority::Normal,
        .repeat_delay_ms = 1000u,
    });

    // Alternator not charging — one-shot critical siren burst.
    g_alert_alternator = g_alerts.register_alert({
        .source          = &g_tone_alternator,
        .mode            = AlertMode::OneShot,
        .priority        = AlertPriority::Critical,
        .repeat_delay_ms = 0u,
    });

    // ── Example: WAV file alert (uncomment once a WAV array is available) ────
    // static WavPlayer wav_player;
    // wav_player.load(g_alert_wav, g_alert_wav_len);
    // int wav_alert = g_alerts.register_alert({
    //     .source          = &wav_player,
    //     .mode            = AlertMode::OneShot,
    //     .priority        = AlertPriority::Normal,
    //     .repeat_delay_ms = 0u,
    // });

    // ── Start audio output ────────────────────────────────────────────────────
    g_i2s.start();

    // ── Demo sequence ─────────────────────────────────────────────────────────
    // In production, alerts would be triggered by GPIO sensor inputs.
    // Here we cycle through them on a fixed schedule to demonstrate the system.

    uint32_t last_update_ms = to_ms_since_boot(get_absolute_time());
    uint32_t demo_timer_ms  = 0u;
    uint8_t  demo_step      = 0u;

    while (true) {
        const uint32_t now_ms     = to_ms_since_boot(get_absolute_time());
        const uint32_t elapsed_ms = now_ms - last_update_ms;
        last_update_ms            = now_ms;

        // ── Keep the alert scheduler ticking ─────────────────────────────────
        g_alerts.update(elapsed_ms);

        // ── Demo state machine ────────────────────────────────────────────────
        demo_timer_ms += elapsed_ms;

        switch (demo_step) {
            case 0:
                // After 1 s, trigger low oil pressure alert (repeating).
                if (demo_timer_ms >= 1000u) {
                    g_alerts.trigger(g_alert_oil);
                    demo_timer_ms = 0u;
                    demo_step     = 1u;
                }
                break;

            case 1:
                // After 3 s, add high temperature alert (lower priority).
                if (demo_timer_ms >= 3000u) {
                    g_alerts.trigger(g_alert_temp);
                    demo_timer_ms = 0u;
                    demo_step     = 2u;
                }
                break;

            case 2:
                // After 3 s, trigger critical alternator fault (preempts all).
                if (demo_timer_ms >= 3000u) {
                    g_alerts.trigger(g_alert_alternator);
                    demo_timer_ms = 0u;
                    demo_step     = 3u;
                }
                break;

            case 3:
                // After 2 s, cancel oil pressure alert; temp alert resumes.
                if (demo_timer_ms >= 2000u) {
                    g_alerts.cancel(g_alert_oil);
                    demo_timer_ms = 0u;
                    demo_step     = 4u;
                }
                break;

            case 4:
                // After 3 s, cancel remaining alerts — silence.
                if (demo_timer_ms >= 3000u) {
                    g_alerts.cancel(g_alert_temp);
                    demo_timer_ms = 0u;
                    demo_step     = 5u;
                }
                break;

            case 5:
                // Loop back to the beginning after 2 s of silence.
                if (demo_timer_ms >= 2000u) {
                    demo_timer_ms = 0u;
                    demo_step     = 0u;
                }
                break;

            default:
                demo_step = 0u;
                break;
        }

        // Yield to other tasks; the I2S DMA runs independently via IRQ.
        sleep_ms(1);
    }

    return 0;
}
