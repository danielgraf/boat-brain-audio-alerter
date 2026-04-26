#pragma once
// i2s_output.h — Low-level I2S audio output driver using PIO + DMA
//
// Drives a MAX98357A (or any standard I2S receiver) from the RP2350/RP2040
// using one PIO state machine and two DMA channels in ping-pong mode.
//
// Usage:
//   1. Construct an I2SOutput instance.
//   2. Call init() with pin numbers, sample rate, and an audio-fill callback.
//   3. Call start() to begin playback.
//   4. The callback is invoked from the DMA IRQ handler each time a buffer
//      completes; fill it with the next AUDIO_BUFFER_FRAMES stereo frames.
//   5. Call stop() to halt output.

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include <functional>

/// Callback invoked from DMA IRQ to request the next audio data.
///
/// @param buffer  Output: interleaved int16_t stereo samples [L0,R0,L1,R1,…].
/// @param frames  Number of stereo frame-pairs the driver needs.
/// @return        Number of frames actually written (zero-pad remainder).
using AudioFillCallback = std::function<uint32_t(int16_t* buffer, uint32_t frames)>;

/// Low-level stereo I2S output driver.
class I2SOutput {
public:
    I2SOutput() = default;
    ~I2SOutput();

    /// Initialise PIO, DMA channels, and interrupts.
    ///
    /// @param data_pin        GPIO for DIN  (serial data to amplifier).
    /// @param clock_pin_base  GPIO base: BCLK = base, LRCLK = base+1.
    /// @param sample_rate     Desired sample rate in Hz (e.g. 44100).
    /// @param callback        Called from IRQ to fill each outgoing buffer.
    /// @return true on success, false if resources could not be claimed.
    bool init(uint data_pin,
              uint clock_pin_base,
              uint32_t sample_rate,
              AudioFillCallback callback);

    /// Start the PIO state machine and DMA transfers.
    void start();

    /// Stop all output immediately (aborts DMA, disables SM).
    void stop();

    /// @return true if output is currently running.
    bool is_running() const { return running_; }

    /// Called from the shared DMA IRQ — do not invoke directly.
    void handle_dma_irq();

private:
    // Number of uint32_t words per DMA buffer (one word = one stereo frame).
    static constexpr uint32_t BUF_WORDS = 512u;

    void     fill_buffer(uint buf_idx);
    void     restart_dma_channel(uint buf_idx);

    PIO      pio_         = pio0;
    uint     sm_          = 0;
    uint     pio_offset_  = 0;
    uint     dma_chan_[2] = {0, 0};

    // Two ping-pong DMA buffers; each word carries one stereo frame packed as
    //   bits[31:16] = left sample, bits[15:0] = right sample.
    uint32_t buffers_[2][BUF_WORDS] = {};

    AudioFillCallback callback_;
    bool initialized_ = false;
    bool running_     = false;

    // Singleton pointer for the static IRQ forwarder.
    static I2SOutput* instance_;
    static void       static_dma_irq_handler();
};
