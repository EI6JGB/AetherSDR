#include "RNNoiseFilter.h"
#include "rnnoise.h"

#include <cstring>
#include <vector>

namespace AetherSDR {

// RNNoise frame size: 480 samples at 48kHz = 10ms
static constexpr int FRAME_SIZE = 480;

RNNoiseFilter::RNNoiseFilter()
    : m_state(rnnoise_create(nullptr))
{}

RNNoiseFilter::~RNNoiseFilter()
{
    if (m_state)
        rnnoise_destroy(m_state);
}

void RNNoiseFilter::reset()
{
    if (m_state)
        rnnoise_destroy(m_state);
    m_state = rnnoise_create(nullptr);
    m_inAccum.clear();
    m_outAccum.clear();
    m_lastSample = 0;
}

QByteArray RNNoiseFilter::process(const QByteArray& pcm24kStereo)
{
    if (!m_state || pcm24kStereo.isEmpty())
        return pcm24kStereo;

    const auto* src = reinterpret_cast<const int16_t*>(pcm24kStereo.constData());
    const int totalInt16 = pcm24kStereo.size() / 2;
    const int stereoFrames = totalInt16 / 2;  // L+R pairs at 24kHz

    // 1. Convert 24kHz stereo int16 → 24kHz mono float
    //    Then upsample 24k → 48k with linear interpolation
    //    RNNoise expects float in [-32768, 32768] range
    const int monoSamples48k = stereoFrames * 2;
    std::vector<float> mono48k(monoSamples48k);

    for (int i = 0; i < stereoFrames; ++i) {
        float cur = static_cast<float>(src[2 * i] + src[2 * i + 1]) * 0.5f;
        // Interpolated sample between previous and current
        mono48k[2 * i] = (m_lastSample + cur) * 0.5f;
        // Current sample
        mono48k[2 * i + 1] = cur;
        m_lastSample = cur;
    }

    // 2. Append to input accumulator and process complete 480-sample frames
    const int prevAccumSamples = m_inAccum.size() / static_cast<int>(sizeof(float));
    m_inAccum.append(reinterpret_cast<const char*>(mono48k.data()),
                     monoSamples48k * sizeof(float));

    const int totalAccumSamples = prevAccumSamples + monoSamples48k;
    const int completeFrames = totalAccumSamples / FRAME_SIZE;

    if (completeFrames > 0) {
        auto* accumData = reinterpret_cast<float*>(m_inAccum.data());
        std::vector<float> processed48k(completeFrames * FRAME_SIZE);

        for (int f = 0; f < completeFrames; ++f) {
            rnnoise_process_frame(m_state,
                                  &processed48k[f * FRAME_SIZE],
                                  &accumData[f * FRAME_SIZE]);
        }

        // Keep leftover input samples
        const int consumedSamples = completeFrames * FRAME_SIZE;
        const int leftoverSamples = totalAccumSamples - consumedSamples;
        if (leftoverSamples > 0) {
            QByteArray leftover(reinterpret_cast<const char*>(&accumData[consumedSamples]),
                                leftoverSamples * sizeof(float));
            m_inAccum = leftover;
        } else {
            m_inAccum.clear();
        }

        // 3. Downsample 48kHz mono → 24kHz stereo int16, append to output accumulator
        const int outputMonoSamples = completeFrames * FRAME_SIZE;
        const int output24kFrames = outputMonoSamples / 2;
        const int outBytes = output24kFrames * 4;  // stereo int16
        const int prevOutSize = m_outAccum.size();
        m_outAccum.resize(prevOutSize + outBytes);
        auto* dst = reinterpret_cast<int16_t*>(m_outAccum.data() + prevOutSize);

        for (int i = 0; i < output24kFrames; ++i) {
            // Average adjacent 48k samples for anti-alias decimation
            float sample = (processed48k[2 * i] + processed48k[2 * i + 1]) * 0.5f;
            int16_t s = static_cast<int16_t>(qBound(-32768.0f, sample, 32767.0f));
            dst[2 * i]     = s;  // L
            dst[2 * i + 1] = s;  // R
        }
    }

    // 4. Return exactly the same number of bytes as input
    const int needed = pcm24kStereo.size();
    if (m_outAccum.size() >= needed) {
        QByteArray result = m_outAccum.left(needed);
        m_outAccum.remove(0, needed);
        return result;
    }

    // Not enough output yet — return silence (only happens during startup)
    return QByteArray(needed, '\0');
}

} // namespace AetherSDR
