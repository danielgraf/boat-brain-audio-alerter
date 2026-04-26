#pragma once
// audio_source.h — Abstract interface for audio data providers
//
// Any class that wants to supply PCM samples to the I2S driver implements
// this interface.  The I2SOutput callback simply calls fill() on whatever
// AudioSource is currently active.

#include <cstdint>

/// Abstract base class for audio data sources.
class AudioSource {
public:
    virtual ~AudioSource() = default;

    /// Fill a stereo PCM buffer.
    ///
    /// @param buffer  Output: interleaved signed-16-bit stereo samples
    ///                [L0, R0, L1, R1, …].  The caller guarantees the array
    ///                is at least frames*2 elements long.
    /// @param frames  Number of stereo frame-pairs requested.
    /// @return        Number of frames actually written (0 … frames).
    ///                Returning fewer than @p frames signals that the source
    ///                has reached its end; the caller zero-pads the rest.
    virtual uint32_t fill(int16_t* buffer, uint32_t frames) = 0;

    /// Reset the source to the beginning so it can be played again.
    virtual void reset() = 0;

    /// @return true when the source has nothing more to output.
    virtual bool is_done() const = 0;
};
