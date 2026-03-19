#pragma once

#include <QByteArray>
#include <memory>

struct DenoiseState;

namespace AetherSDR {

// Client-side RNN noise suppression using Mozilla/Xiph RNNoise.
// Processes 24kHz stereo Int16 audio by upsampling to 48kHz mono,
// running RNNoise, and downsampling back to 24kHz stereo.
//
// RNNoise requires 48kHz mono float input in 480-sample (10ms) frames.

class RNNoiseFilter {
public:
    RNNoiseFilter();
    ~RNNoiseFilter();

    // Process a block of 24kHz stereo Int16 PCM.
    // Returns the processed block (same format, same size).
    QByteArray process(const QByteArray& pcm24kStereo);

    // Reset internal state (e.g., on band change).
    void reset();

private:
    DenoiseState* m_state{nullptr};
    QByteArray    m_inAccum;       // accumulate 48kHz mono float input
    QByteArray    m_outAccum;      // accumulate 24kHz stereo int16 output
    float         m_lastSample{0}; // last 24kHz mono sample for interpolation
};

} // namespace AetherSDR
