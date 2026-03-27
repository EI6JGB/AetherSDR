#include "AudioEngine.h"
#include "AppSettings.h"
#include "LogManager.h"
#include "OpusCodec.h"
#include "SpectralNR.h"
#include "RNNoiseFilter.h"
#include "NvidiaBnrFilter.h"
#include "Resampler.h"

#include <QMediaDevices>
#include <QAudioDevice>
#include <QDir>
#include <QtEndian>
#include <cmath>
#include <cstring>

namespace AetherSDR {

AudioEngine::AudioEngine(QObject* parent)
    : QObject(parent)
{
    // Restore saved audio device selections
    auto& s = AppSettings::instance();
    QByteArray savedOutId = s.value("AudioOutputDeviceId", "").toByteArray();
    QByteArray savedInId  = s.value("AudioInputDeviceId",  "").toByteArray();

    if (!savedOutId.isEmpty()) {
        for (const auto& dev : QMediaDevices::audioOutputs()) {
            if (dev.id() == savedOutId) { m_outputDevice = dev; break; }
        }
    }
    if (!savedInId.isEmpty()) {
        for (const auto& dev : QMediaDevices::audioInputs()) {
            if (dev.id() == savedInId) { m_inputDevice = dev; break; }
        }
    }
}

AudioEngine::~AudioEngine()
{
    stopRxStream();
    stopTxStream();
}

QAudioFormat AudioEngine::makeFormat() const
{
    QAudioFormat fmt;
    fmt.setSampleRate(DEFAULT_SAMPLE_RATE);
    fmt.setChannelCount(2);                        // stereo
    fmt.setSampleFormat(QAudioFormat::Int16);
    return fmt;
}

// ─── RX stream ───────────────────────────────────────────────────────────────

bool AudioEngine::startRxStream()
{
    if (m_audioSink) return true;   // already running

    QAudioFormat fmt = makeFormat();
    const QAudioDevice dev = m_outputDevice.isNull()
        ? QMediaDevices::defaultAudioOutput() : m_outputDevice;

    if (!dev.isFormatSupported(fmt)) {
        qCWarning(lcAudio) << "AudioEngine: output device does not support 24kHz stereo Int16, trying 48kHz";
        fmt.setSampleRate(48000);
        m_resampleTo48k = true;
        if (!dev.isFormatSupported(fmt)) {
            qCWarning(lcAudio) << "AudioEngine: output device does not support 48kHz stereo Int16 either";
            qCWarning(lcAudio) << "No audio device detected";
            return false;
        }
    } else {
        m_resampleTo48k = false;
    }

    m_audioSink   = new QAudioSink(dev, fmt, this);
    m_audioSink->setVolume(m_rxVolume);
    m_audioDevice = m_audioSink->start();   // push-mode

    if (!m_audioDevice) {
        qCWarning(lcAudio) << "AudioEngine: failed to open audio sink";
        delete m_audioSink;
        m_audioSink = nullptr;
        return false;
    }

    qCDebug(lcAudio) << "AudioEngine: RX stream started";
    emit rxStarted();
    return true;
}

void AudioEngine::stopRxStream()
{
    if (m_audioSink) {
        m_audioSink->stop();
        delete m_audioSink;
        m_audioSink   = nullptr;
        m_audioDevice = nullptr;
    }
    emit rxStopped();
}

void AudioEngine::setRxVolume(float v)
{
    m_rxVolume = qBound(0.0f, v, 1.0f);
    if (m_audioSink)
        m_audioSink->setVolume(m_muted ? 0.0f : m_rxVolume);
}

void AudioEngine::setMuted(bool muted)
{
    m_muted = muted;
    if (m_audioSink)
        m_audioSink->setVolume(muted ? 0.0f : m_rxVolume);
}

// Resample 24kHz stereo int16 → 48kHz stereo int16 via r8brain.
QByteArray AudioEngine::resampleStereo(const QByteArray& pcm)
{
    if (!m_rxResampler)
        m_rxResampler = std::make_unique<Resampler>(24000, 48000);
    const auto* src = reinterpret_cast<const int16_t*>(pcm.constData());
    return m_rxResampler->processStereoToStereo(src, pcm.size() / 4);
}

void AudioEngine::feedAudioData(const QByteArray& pcm)
{
    if (!m_audioSink) return;  // PC audio disabled
    // Note: m_radeMode no longer blocks feedAudioData globally.
    // The RADE slice's raw OFDM noise is muted at the slice level (audio_mute=1)
    // so it doesn't reach the speaker. Other slices' audio plays normally.

    auto writeAudio = [this](const QByteArray& data) {
        if (!m_audioDevice || !m_audioDevice->isOpen()) return;
        if (m_resampleTo48k)
            m_audioDevice->write(resampleStereo(data));
        else
            m_audioDevice->write(data);
    };

    if (m_rn2Enabled && m_rn2) {
        QByteArray processed = m_rn2->process(pcm);
        writeAudio(processed);
        emit levelChanged(computeRMS(processed));
    } else if (m_nr2Enabled && m_nr2) {
        processNr2(pcm);
        writeAudio(m_nr2Output);
        emit levelChanged(computeRMS(m_nr2Output));
    } else if (m_bnrEnabled && m_bnr && m_bnr->isConnected()) {
        processBnr(pcm);
        // processBnr writes audio and emits level internally
    } else {
        writeAudio(pcm);
        emit levelChanged(computeRMS(pcm));
    }
}

static QString wisdomDir()
{
#ifdef _WIN32
    // Windows: use %APPDATA%/AetherSDR/
    QString dir = QDir::homePath() + "/AppData/Roaming/AetherSDR/";
#else
    QString dir = QDir::homePath() + "/.config/AetherSDR/AetherSDR/";
#endif
    QDir().mkpath(dir);
    return dir;
}

bool AudioEngine::needsWisdomGeneration()
{
    QString path = wisdomDir() + "aethersdr_fftw_wisdom";
    return !QFile::exists(path);
}

void AudioEngine::generateWisdom(std::function<void(int,int,const std::string&)> progress)
{
    SpectralNR::generateWisdom(wisdomDir().toStdString(), std::move(progress));
}

void AudioEngine::setNr2Enabled(bool on)
{
    if (m_nr2Enabled == on) return;
    m_nr2Enabled = on;
    if (on) {
        // Disable RN2/BNR if on — they're mutually exclusive
        if (m_rn2Enabled) setRn2Enabled(false);
        if (m_bnrEnabled) setBnrEnabled(false);
        // Wisdom should already be generated by MainWindow::enableNr2WithWisdom().
        // This import is a fast no-op if wisdom file exists; falls back to
        // FFTW_ESTIMATE plans (slower DSP but no main-thread freeze).
        if (!needsWisdomGeneration())
            SpectralNR::generateWisdom(wisdomDir().toStdString());
        m_nr2 = std::make_unique<SpectralNR>(256, DEFAULT_SAMPLE_RATE);
        if (m_nr2->hasPlanFailed()) {
            qCWarning(lcAudio) << "AudioEngine: NR2 FFTW plan creation failed — disabling";
            m_nr2.reset();
            m_nr2Enabled = false;
            emit nr2EnabledChanged(false);
            return;
        }
    } else {
        m_nr2.reset();
    }
    qCDebug(lcAudio) << "AudioEngine: NR2" << (on ? "enabled" : "disabled");
    emit nr2EnabledChanged(on);
}

void AudioEngine::setRn2Enabled(bool on)
{
    if (m_rn2Enabled == on) return;
    m_rn2Enabled = on;
    if (on) {
        m_rn2 = std::make_unique<RNNoiseFilter>();
        if (!m_rn2->isValid()) {
            qCWarning(lcAudio) << "AudioEngine: RN2 rnnoise_create() failed — disabling";
            m_rn2.reset();
            m_rn2Enabled = false;
            emit rn2EnabledChanged(false);
            return;
        }
        // Disable NR2/BNR if on — they're mutually exclusive
        if (m_nr2Enabled) setNr2Enabled(false);
        if (m_bnrEnabled) setBnrEnabled(false);
    } else {
        m_rn2.reset();
    }
    qCDebug(lcAudio) << "AudioEngine: RN2 (RNNoise)" << (on ? "enabled" : "disabled");
    emit rn2EnabledChanged(on);
}

// ─── BNR (NVIDIA NIM GPU noise removal) ──────────────────────────────────────

void AudioEngine::setBnrEnabled(bool on)
{
    if (m_bnrEnabled == on) return;
    m_bnrEnabled = on;
    if (on) {
        // Mutual exclusion with NR2 and RN2
        if (m_nr2Enabled) setNr2Enabled(false);
        if (m_rn2Enabled) setRn2Enabled(false);

        m_bnr = std::make_unique<NvidiaBnrFilter>(this);
        connect(m_bnr.get(), &NvidiaBnrFilter::connectionChanged,
                this, &AudioEngine::bnrConnectionChanged);

        // Resamplers: 24kHz mono ↔ 48kHz mono
        m_bnrUp   = std::make_unique<Resampler>(24000, 48000);
        m_bnrDown = std::make_unique<Resampler>(48000, 24000);

        if (!m_bnr->connectToServer(m_bnrAddress)) {
            qCWarning(lcAudio) << "AudioEngine: BNR connect failed — disabling";
            m_bnr.reset();
            m_bnrUp.reset();
            m_bnrDown.reset();
            m_bnrEnabled = false;
            emit bnrEnabledChanged(false);
            return;
        }
    } else {
        if (m_bnr) m_bnr->disconnect();
        m_bnr.reset();
        m_bnrUp.reset();
        m_bnrDown.reset();
    }
    qCDebug(lcAudio) << "AudioEngine: BNR (NVIDIA NIM)" << (on ? "enabled" : "disabled");
    emit bnrEnabledChanged(on);
}

void AudioEngine::setBnrAddress(const QString& addr)
{
    m_bnrAddress = addr;
}

void AudioEngine::setBnrIntensity(float ratio)
{
    if (m_bnr) m_bnr->setIntensityRatio(ratio);
}

float AudioEngine::bnrIntensity() const
{
    return m_bnr ? m_bnr->intensityRatio() : 1.0f;
}

bool AudioEngine::bnrConnected() const
{
    return m_bnr && m_bnr->isConnected();
}

void AudioEngine::processBnr(const QByteArray& stereoPcm)
{
    // 1. 24kHz stereo int16 → 24kHz mono int16 (average L+R)
    const auto* src = reinterpret_cast<const int16_t*>(stereoPcm.constData());
    const int stereoFrames = stereoPcm.size() / 4;  // L+R pairs

    // Reuse NR2 mono buffer
    if (static_cast<int>(m_nr2Mono.size()) < stereoFrames)
        m_nr2Mono.resize(stereoFrames);
    for (int i = 0; i < stereoFrames; ++i)
        m_nr2Mono[i] = static_cast<int16_t>((src[2 * i] + src[2 * i + 1]) / 2);

    // 2. 24kHz mono → 48kHz mono (r8brain)
    QByteArray mono48k = m_bnrUp->process(m_nr2Mono.data(), stereoFrames);

    // 3. int16 → float32 for BNR container
    const auto* mono48kSrc = reinterpret_cast<const int16_t*>(mono48k.constData());
    const int mono48kSamples = mono48k.size() / 2;
    QVector<float> floatBuf(mono48kSamples);
    for (int i = 0; i < mono48kSamples; ++i)
        floatBuf[i] = mono48kSrc[i] / 32768.0f;

    // 4. Send to BNR container, get denoised data back
    QByteArray denoised = m_bnr->process(floatBuf.constData(), mono48kSamples);

    // If no denoised data yet (accumulating), pass through original
    if (denoised.isEmpty()) {
        auto writeAudio = [this](const QByteArray& data) {
            if (!m_audioDevice || !m_audioDevice->isOpen()) return;
            if (m_resampleTo48k)
                m_audioDevice->write(resampleStereo(data));
            else
                m_audioDevice->write(data);
        };
        writeAudio(stereoPcm);
        emit levelChanged(computeRMS(stereoPcm));
        return;
    }

    // 5. float32 → int16 (denoised 48kHz mono)
    const auto* denoisedFloat = reinterpret_cast<const float*>(denoised.constData());
    const int denoisedSamples = denoised.size() / sizeof(float);
    QByteArray denoisedInt16(denoisedSamples * 2, Qt::Uninitialized);
    auto* dst16 = reinterpret_cast<int16_t*>(denoisedInt16.data());
    for (int i = 0; i < denoisedSamples; ++i) {
        float s = std::clamp(denoisedFloat[i] * 32768.0f, -32768.0f, 32767.0f);
        dst16[i] = static_cast<int16_t>(s);
    }

    // 6. 48kHz mono → 24kHz mono (r8brain)
    QByteArray mono24k = m_bnrDown->process(dst16, denoisedSamples);

    // 7. Mono → stereo (duplicate both channels)
    const auto* mono24kSrc = reinterpret_cast<const int16_t*>(mono24k.constData());
    const int mono24kSamples = mono24k.size() / 2;
    QByteArray stereoOut(mono24kSamples * 4, Qt::Uninitialized);
    auto* dstStereo = reinterpret_cast<int16_t*>(stereoOut.data());
    for (int i = 0; i < mono24kSamples; ++i) {
        dstStereo[2 * i]     = mono24kSrc[i];
        dstStereo[2 * i + 1] = mono24kSrc[i];
    }

    // 8. Write to audio output
    if (m_audioDevice && m_audioDevice->isOpen()) {
        if (m_resampleTo48k)
            m_audioDevice->write(resampleStereo(stereoOut));
        else
            m_audioDevice->write(stereoOut);
    }
    emit levelChanged(computeRMS(stereoOut));
}

void AudioEngine::processNr2(const QByteArray& stereoPcm)
{
    const int totalSamples = stereoPcm.size() / 2;       // int16 count
    const int stereoFrames = totalSamples / 2;            // L+R pairs
    const auto* src = reinterpret_cast<const int16_t*>(stereoPcm.constData());

    // Resize pre-allocated buffers if needed (only re-allocates on size increase)
    if (static_cast<int>(m_nr2Mono.size()) < stereoFrames) {
        m_nr2Mono.resize(stereoFrames);
        m_nr2Processed.resize(stereoFrames);
    }

    // Stereo -> mono (average L+R)
    for (int i = 0; i < stereoFrames; ++i)
        m_nr2Mono[i] = static_cast<int16_t>((src[2 * i] + src[2 * i + 1]) / 2);

    // Process through SpectralNR
    m_nr2->process(m_nr2Mono.data(), m_nr2Processed.data(), stereoFrames);

    // Mono -> stereo (duplicate) into pre-allocated output buffer
    const int outBytes = stereoFrames * 4;  // stereoFrames x 2ch x 2bytes
    m_nr2Output.resize(outBytes);
    auto* dst = reinterpret_cast<int16_t*>(m_nr2Output.data());
    for (int i = 0; i < stereoFrames; ++i) {
        dst[2 * i]     = m_nr2Processed[i];
        dst[2 * i + 1] = m_nr2Processed[i];
    }
}

QByteArray AudioEngine::applyBoost(const QByteArray& pcm, float gain) const
{
    const int nSamples = pcm.size() / sizeof(int16_t);
    const auto* src = reinterpret_cast<const int16_t*>(pcm.constData());
    QByteArray out(pcm.size(), Qt::Uninitialized);
    auto* dst = reinterpret_cast<int16_t*>(out.data());
    for (int i = 0; i < nSamples; ++i) {
        float s = src[i] * gain;
        // Soft clamp to avoid harsh digital clipping
        if (s > 32767.0f) s = 32767.0f;
        else if (s < -32767.0f) s = -32767.0f;
        dst[i] = static_cast<int16_t>(s);
    }
    return out;
}

float AudioEngine::computeRMS(const QByteArray& pcm) const
{
    const int samples = pcm.size() / 2;  // 16-bit samples
    if (samples == 0) return 0.0f;

    const int16_t* data = reinterpret_cast<const int16_t*>(pcm.constData());
    double sum = 0.0;
    for (int i = 0; i < samples; ++i) {
        const double s = data[i] / 32768.0;
        sum += s * s;
    }
    return static_cast<float>(std::sqrt(sum / samples));
}

// ─── TX stream ────────────────────────────────────────────────────────────────

bool AudioEngine::startTxStream(const QHostAddress& radioAddress, quint16 radioPort)
{
    if (m_audioSource) return true;  // already running

    m_txAddress = radioAddress;
    m_txPort    = radioPort;
    m_txPacketCount = 0;
    m_txAccumulator.clear();

    QAudioFormat fmt = makeFormat();
    const QAudioDevice dev = m_inputDevice.isNull()
        ? QMediaDevices::defaultAudioInput() : m_inputDevice;

    if (dev.isNull()) {
        qCWarning(lcAudio) << "AudioEngine: no audio input device available";
        return false;
    }

    // Sample rate: macOS needs 48kHz for USB mics, Linux can try 24kHz first
#ifdef Q_OS_MAC
    fmt.setSampleRate(48000);
    m_txDownsampleFrom48k = true;
    if (!dev.isFormatSupported(fmt)) {
        fmt.setSampleRate(24000);
        m_txDownsampleFrom48k = false;
        if (!dev.isFormatSupported(fmt)) {
            qCWarning(lcAudio) << "AudioEngine: input device supports neither 48kHz nor 24kHz";
            return false;
        }
    }
#else
    if (!dev.isFormatSupported(fmt)) {
        qCWarning(lcAudio) << "AudioEngine: input device does not support 24kHz, trying 48kHz";
        fmt.setSampleRate(48000);
        m_txDownsampleFrom48k = true;
        if (!dev.isFormatSupported(fmt)) {
            qCWarning(lcAudio) << "AudioEngine: input device does not support 48kHz either";
            return false;
        }
    } else {
        m_txDownsampleFrom48k = false;
    }
#endif

    qCDebug(lcAudio) << "AudioEngine: input device:" << dev.description()
             << "rate:" << fmt.sampleRate() << "ch:" << fmt.channelCount();

#ifdef Q_OS_MAC
    // macOS: QAudioSource pull mode broken — use push mode with QBuffer
    m_micBuffer = new QBuffer(this);
    m_micBuffer->open(QIODevice::ReadWrite);
    m_audioSource = new QAudioSource(dev, fmt, this);
    m_audioSource->start(m_micBuffer);

    if (m_audioSource->state() == QAudio::StoppedState) {
        qCWarning(lcAudio) << "AudioEngine: failed to start audio source";
        delete m_audioSource; m_audioSource = nullptr;
        delete m_micBuffer; m_micBuffer = nullptr;
        return false;
    }

    // Poll push-mode buffer
    m_txPollTimer = new QTimer(this);
    m_txPollTimer->setInterval(5);
    connect(m_txPollTimer, &QTimer::timeout, this, &AudioEngine::onTxAudioReady);
    m_txPollTimer->start();
#else
    // Linux: pull mode works fine
    m_audioSource = new QAudioSource(dev, fmt, this);
    m_micDevice = m_audioSource->start();
    if (!m_micDevice) {
        qCWarning(lcAudio) << "AudioEngine: failed to open audio source";
        delete m_audioSource; m_audioSource = nullptr;
        return false;
    }
    connect(m_micDevice, &QIODevice::readyRead, this, &AudioEngine::onTxAudioReady);
#endif

    qCDebug(lcAudio) << "AudioEngine: TX stream started ->" << radioAddress.toString()
             << ":" << radioPort << "streamId:" << Qt::hex << m_txStreamId;
    return true;
}

void AudioEngine::stopTxStream()
{
#ifdef Q_OS_MAC
    if (m_txPollTimer) {
        m_txPollTimer->stop();
        delete m_txPollTimer;
        m_txPollTimer = nullptr;
    }
#endif
    if (m_audioSource) {
        m_audioSource->stop();
        delete m_audioSource;
        m_audioSource = nullptr;
        m_micDevice   = nullptr;
    }
#ifdef Q_OS_MAC
    if (m_micBuffer) {
        delete m_micBuffer;
        m_micBuffer = nullptr;
    }
#endif
    m_txSocket.close();
    m_txAccumulator.clear();
    m_txFloatAccumulator.clear();
}

void AudioEngine::onTxAudioReady()
{
#ifdef Q_OS_MAC
    if (!m_micBuffer || m_txStreamId == 0) return;
    qint64 avail = m_micBuffer->pos();
    if (avail <= 0) return;
    QByteArray data = m_micBuffer->data();
    m_micBuffer->buffer().clear();
    m_micBuffer->seek(0);
    if (data.isEmpty()) return;
#else
    if (!m_micDevice || m_txStreamId == 0) return;
    QByteArray data = m_micDevice->readAll();
    if (data.isEmpty()) return;
#endif

    // Downsample 48kHz → 24kHz: drop every other stereo sample pair
    if (m_txDownsampleFrom48k && data.size() >= 8) {
        const auto* src = reinterpret_cast<const qint16*>(data.constData());
        const int stereoSamples = data.size() / 4;
        QByteArray ds(stereoSamples / 2 * 4, Qt::Uninitialized);
        auto* dst = reinterpret_cast<qint16*>(ds.data());
        for (int i = 0; i < stereoSamples / 2; ++i) {
            dst[i * 2 + 0] = src[i * 4 + 0];  // L (take every other pair)
            dst[i * 2 + 1] = src[i * 4 + 1];  // R
        }
        data = ds;
    }

    // RADE mode: emit raw PCM for RADEEngine instead of sending VITA-49
    if (m_radeMode) {
        emit txRawPcmReady(data);  // data is int16 stereo 24kHz
        return;
    }

    // DAX TX mode: VirtualAudioBridge is the TX audio source.
    if (m_daxTxMode) return;

    // When not transmitting, stream mic audio on the remote_audio_tx stream
    // so the radio can monitor it for VOX detection (met_in_rx=1).
    // Don't send on the DAX TX stream during RX — it accumulates and
    // plays back when TX starts, causing mic bleed.
    if (!m_transmitting) {
        if (m_remoteTxStreamId == 0) return;
        // Send uncompressed VITA-49 mic audio for VOX monitoring
        sendVoiceTxPacket(data, m_remoteTxStreamId);
        return;
    }

    // ── Opus TX path: encode 20ms frames (480 stereo samples) ──────────
    if (m_opusTxEnabled) {
        m_opusTxAccumulator.append(data);
        constexpr int OPUS_FRAME_BYTES = 480 * 2 * sizeof(int16_t);  // 480 stereo pairs

        while (m_opusTxAccumulator.size() >= OPUS_FRAME_BYTES) {
            if (!m_opusTxCodec) {
                m_opusTxCodec = std::make_unique<OpusCodec>();
                if (!m_opusTxCodec->isValid()) {
                    qCWarning(lcAudio) << "AudioEngine: Opus TX codec init failed, falling back to uncompressed";
                    m_opusTxEnabled = false;
                    m_opusTxCodec.reset();
                    break;
                }
            }

            QByteArray frame = m_opusTxAccumulator.left(OPUS_FRAME_BYTES);
            m_opusTxAccumulator.remove(0, OPUS_FRAME_BYTES);

            QByteArray opus = m_opusTxCodec->encode(frame);
            if (opus.isEmpty()) continue;

            // Build VITA-49 Opus packet (raw bytes, no byte-swap)
            // Header: 28 bytes + opus payload + 4 byte trailer
            int words = (28 + opus.size() + 4 + 3) / 4;  // round up to 32-bit words
            QByteArray pkt(words * 4, '\0');
            auto* p = reinterpret_cast<quint32*>(pkt.data());

            // Word 0: type=3 (ExtDataWithStream), C=1, T=1, TSI=01, TSF=01, count, size
            p[0] = qToBigEndian<quint32>(
                (3u << 28) | (1u << 27) | (1u << 26) | (1u << 22) | (1u << 20)
                | ((m_txPacketCount & 0x0F) << 16) | words);
            m_txPacketCount = (m_txPacketCount + 1) & 0x0F;
            p[1] = qToBigEndian(m_txStreamId);
            p[2] = qToBigEndian<quint32>(0x00001C2D);  // OUI
            p[3] = qToBigEndian<quint32>(0x80050000 | 0x8005);  // ICC=0x8005, PCC=0x8005
            p[4] = 0; p[5] = 0; p[6] = 0;  // timestamps

            memcpy(pkt.data() + 28, opus.constData(), opus.size());
            emit txPacketReady(pkt);
        }
        return;
    }

    // ── Uncompressed TX path ─────────────────────────────────────────────
    m_txAccumulator.append(data);

    // Process complete packets (128 stereo pairs = 512 bytes of int16 PCM)
    while (m_txAccumulator.size() >= TX_PCM_BYTES_PER_PACKET) {
        const int16_t* pcm = reinterpret_cast<const int16_t*>(m_txAccumulator.constData());

        // Convert int16 stereo → float32 stereo (128 pairs = 256 floats)
        // No client-side gain — the radio handles mic level via mic_level
        // and mic_boost settings. Sending at native level prevents clipping.
        float floatBuf[TX_SAMPLES_PER_PACKET * 2];
        for (int i = 0; i < TX_SAMPLES_PER_PACKET * 2; ++i)
            floatBuf[i] = pcm[i] / 32768.0f;

        // Build VITA-49 packet and send via registered UDP socket
        QByteArray packet = buildVitaTxPacket(floatBuf, TX_SAMPLES_PER_PACKET);
        emit txPacketReady(packet);

        // Advance accumulator
        m_txAccumulator.remove(0, TX_PCM_BYTES_PER_PACKET);
    }
}

QByteArray AudioEngine::buildVitaTxPacket(const float* samples, int numStereoSamples)
{
    const int payloadBytes = numStereoSamples * 2 * 4;  // stereo × sizeof(float)
    const int packetWords = (payloadBytes / 4) + VITA_HEADER_WORDS;
    const int packetBytes = packetWords * 4;

    QByteArray packet(packetBytes, '\0');
    quint32* words = reinterpret_cast<quint32*>(packet.data());

    // ── Word 0: Header ────────────────────────────────────────────────────
    // Bits 31-28: packet type = 1 (IFDataWithStream) — DAX TX format
    // Bit  27:    C = 1 (class ID present)
    // Bit  26:    T = 0 (no trailer)
    // Bits 25-24: reserved = 0
    // Bits 23-22: TSI = 2 (Other)
    // Bits 21-20: TSF = 1 (SampleCount)
    // Bits 19-16: packet count (4-bit)
    // Bits 15-0:  packet size (in 32-bit words)
    quint32 hdr = 0;
    hdr |= (0x1u << 28);          // pkt_type = IFDataWithStream (DAX TX)
    hdr |= (1u << 27);            // C = 1
    // T = 0 (bit 26)
    hdr |= (0x2u << 22);          // TSI = Other
    hdr |= (0x1u << 20);          // TSF = SampleCount
    hdr |= ((m_txPacketCount & 0xF) << 16);
    hdr |= (packetWords & 0xFFFF);
    words[0] = qToBigEndian(hdr);

    // ── Word 1: Stream ID ─────────────────────────────────────────────────
    words[1] = qToBigEndian(m_txStreamId);

    // ── Word 2: Class ID OUI (24-bit, right-justified in 32-bit word) ────
    words[2] = qToBigEndian(FLEX_OUI);

    // ── Word 3: InformationClassCode (upper 16) | PacketClassCode (lower 16)
    words[3] = qToBigEndian(
        (static_cast<quint32>(FLEX_INFO_CLASS) << 16) | PCC_IF_NARROW);

    // ── Words 4-6: Timestamps ─────────────────────────────────────────────
    words[4] = 0;  // integer timestamp
    words[5] = 0;  // fractional timestamp high
    words[6] = 0;  // fractional timestamp low (sample count)

    // ── Payload: float32 stereo, big-endian ───────────────────────────────
    quint32* payload = words + VITA_HEADER_WORDS;
    for (int i = 0; i < numStereoSamples * 2; ++i) {
        quint32 raw;
        std::memcpy(&raw, &samples[i], 4);
        payload[i] = qToBigEndian(raw);
    }

    // Increment packet count (4-bit, mod 16)
    m_txPacketCount = (m_txPacketCount + 1) & 0xF;

    return packet;
}

void AudioEngine::sendVoiceTxPacket(const QByteArray& pcmData, quint32 streamId)
{
    // Accumulate into a separate buffer for VOX/met_in_rx audio
    m_voxAccumulator.append(pcmData);

    while (m_voxAccumulator.size() >= TX_PCM_BYTES_PER_PACKET) {
        const int16_t* pcm = reinterpret_cast<const int16_t*>(m_voxAccumulator.constData());

        float floatBuf[TX_SAMPLES_PER_PACKET * 2];
        for (int i = 0; i < TX_SAMPLES_PER_PACKET * 2; ++i)
            floatBuf[i] = pcm[i] / 32768.0f;

        // Build packet using the remote_audio_tx stream ID
        quint32 savedId = m_txStreamId;
        m_txStreamId = streamId;
        QByteArray packet = buildVitaTxPacket(floatBuf, TX_SAMPLES_PER_PACKET);
        m_txStreamId = savedId;

        emit txPacketReady(packet);
        m_voxAccumulator.remove(0, TX_PCM_BYTES_PER_PACKET);
    }
}

void AudioEngine::setOutputDevice(const QAudioDevice& dev)
{
    m_outputDevice = dev;
    qCDebug(lcAudio) << "AudioEngine: output device set to" << dev.description();

    // Persist selection
    auto& s = AppSettings::instance();
    s.setValue("AudioOutputDeviceId", dev.id());
    s.save();

    // Restart RX stream if running
    if (m_audioSink) {
        stopRxStream();
        startRxStream();
    }
}

void AudioEngine::setInputDevice(const QAudioDevice& dev)
{
    m_inputDevice = dev;
    qCDebug(lcAudio) << "AudioEngine: input device set to" << dev.description();

    // Persist selection
    auto& s = AppSettings::instance();
    s.setValue("AudioInputDeviceId", dev.id());
    s.save();

    // Restart TX stream if running
    if (m_audioSource) {
        QHostAddress addr = m_txAddress;
        quint16 port = m_txPort;
        stopTxStream();
        startTxStream(addr, port);
    }
}

// ─── RADE digital voice support ──────────────────────────────────────────────

void AudioEngine::setRadeMode(bool on)
{
    m_radeMode = on;
}

void AudioEngine::sendModemTxAudio(const QByteArray& float32pcm)
{
    if (m_txStreamId == 0) return;

    m_txFloatAccumulator.append(float32pcm);

    constexpr int FLOAT_BYTES_PER_PKT = TX_SAMPLES_PER_PACKET * 2 * sizeof(float); // 1024
    while (m_txFloatAccumulator.size() >= FLOAT_BYTES_PER_PKT) {
        auto* samples = reinterpret_cast<const float*>(m_txFloatAccumulator.constData());
        QByteArray pkt = buildVitaTxPacket(samples, TX_SAMPLES_PER_PACKET);
        emit txPacketReady(pkt);
        m_txFloatAccumulator.remove(0, FLOAT_BYTES_PER_PKT);
    }
}

void AudioEngine::feedDaxTxAudio(const QByteArray& float32pcm)
{
    if (m_txStreamId == 0) return;

    // Block DAX audio when mic is actively sending (voice mode TX).
    // This prevents dual-source jitter on the same VITA-49 stream.
    // During RX and digital TX, DAX flows freely.
    if (m_transmitting && !m_daxTxMode) return;
    m_txFloatAccumulator.append(float32pcm);
    constexpr int FLOAT_BYTES_PER_PKT = TX_SAMPLES_PER_PACKET * 2 * sizeof(float);
    while (m_txFloatAccumulator.size() >= FLOAT_BYTES_PER_PKT) {
        auto* samples = reinterpret_cast<const float*>(m_txFloatAccumulator.constData());
        QByteArray pkt = buildVitaTxPacket(samples, TX_SAMPLES_PER_PACKET);
        emit txPacketReady(pkt);
        m_txFloatAccumulator.remove(0, FLOAT_BYTES_PER_PKT);
    }
}

void AudioEngine::feedDecodedSpeech(const QByteArray& pcm)
{
    if (!m_audioSink || !m_audioDevice || !m_audioDevice->isOpen()) return;
    if (m_resampleTo48k)
        m_audioDevice->write(resampleStereo(pcm));
    else
        m_audioDevice->write(pcm);
}

} // namespace AetherSDR
