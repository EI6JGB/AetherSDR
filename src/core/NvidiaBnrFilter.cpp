#include "NvidiaBnrFilter.h"

#include <QDebug>

#ifdef HAVE_BNR
#include <cstring>
#endif

namespace AetherSDR {

NvidiaBnrFilter::NvidiaBnrFilter(QObject* parent)
    : QObject(parent)
{}

NvidiaBnrFilter::~NvidiaBnrFilter()
{
    disconnect();
}

#ifdef HAVE_BNR

bool NvidiaBnrFilter::connectToServer(const QString& address)
{
    if (m_connected.load()) disconnect();

    m_stopping.store(false);

    m_channel = grpc::CreateChannel(address.toStdString(),
                                     grpc::InsecureChannelCredentials());
    m_stub = nvidia::maxine::bnr::v1::MaxineBNR::NewStub(m_channel);

    m_context = std::make_unique<grpc::ClientContext>();
    m_stream = m_stub->EnhanceAudio(m_context.get());

    if (!m_stream) {
        qWarning() << "NvidiaBnrFilter: failed to open gRPC stream to" << address;
        emit errorOccurred("Failed to open gRPC stream");
        return false;
    }

    // Send initial config
    nvidia::maxine::bnr::v1::EnhanceAudioRequest configReq;
    auto* config = configReq.mutable_config();
    config->set_intensity_ratio(m_intensityRatio);

    if (!m_stream->Write(configReq)) {
        qWarning() << "NvidiaBnrFilter: failed to send config";
        closeStream();
        emit errorOccurred("Failed to send configuration");
        return false;
    }

    m_connected.store(true);
    m_inAccum.clear();
    {
        QMutexLocker lock(&m_outMutex);
        m_outBuf.clear();
    }

    m_readerThread = std::thread(&NvidiaBnrFilter::readerLoop, this);

    qDebug() << "NvidiaBnrFilter: connected to" << address
             << "intensity:" << m_intensityRatio;
    emit connectionChanged(true);
    return true;
}

void NvidiaBnrFilter::disconnect()
{
    if (!m_connected.load() && !m_readerThread.joinable()) return;

    m_stopping.store(true);
    m_connected.store(false);

    closeStream();

    if (m_readerThread.joinable())
        m_readerThread.join();

    m_inAccum.clear();
    {
        QMutexLocker lock(&m_outMutex);
        m_outBuf.clear();
    }

    qDebug() << "NvidiaBnrFilter: disconnected";
    emit connectionChanged(false);
}

bool NvidiaBnrFilter::isConnected() const
{
    return m_connected.load();
}

void NvidiaBnrFilter::closeStream()
{
    if (m_stream) {
        m_stream->WritesDone();
        m_stream->Finish();
        m_stream.reset();
    }
    m_context.reset();
}

void NvidiaBnrFilter::setIntensityRatio(float ratio)
{
    m_intensityRatio = std::clamp(ratio, 0.0f, 1.0f);

    if (m_connected.load() && m_stream) {
        nvidia::maxine::bnr::v1::EnhanceAudioRequest configReq;
        auto* config = configReq.mutable_config();
        config->set_intensity_ratio(m_intensityRatio);
        m_stream->Write(configReq);
    }
}

QByteArray NvidiaBnrFilter::process(const float* samples, int numSamples)
{
    if (!m_connected.load()) return {};

    m_inAccum.append(reinterpret_cast<const char*>(samples),
                     numSamples * sizeof(float));

    while (m_inAccum.size() >= kFrameBytes) {
        nvidia::maxine::bnr::v1::EnhanceAudioRequest req;
        req.set_audio_stream_data(m_inAccum.constData(), kFrameBytes);
        m_inAccum.remove(0, kFrameBytes);

        if (!m_stream->Write(req)) {
            qWarning() << "NvidiaBnrFilter: write failed, disconnecting";
            m_connected.store(false);
            QMetaObject::invokeMethod(this, [this]() {
                emit connectionChanged(false);
                emit errorOccurred("gRPC write failed");
            }, Qt::QueuedConnection);
            return {};
        }
    }

    QMutexLocker lock(&m_outMutex);
    if (m_outBuf.isEmpty()) return {};

    QByteArray result;
    result.swap(m_outBuf);
    return result;
}

void NvidiaBnrFilter::readerLoop()
{
    nvidia::maxine::bnr::v1::EnhanceAudioResponse response;

    while (!m_stopping.load() && m_stream && m_stream->Read(&response)) {
        if (response.has_audio_stream_data()) {
            const auto& data = response.audio_stream_data();
            QMutexLocker lock(&m_outMutex);
            m_outBuf.append(data.data(), data.size());

            constexpr int maxBytes = kFrameBytes * 10;  // cap at ~100ms
            if (m_outBuf.size() > maxBytes)
                m_outBuf.remove(0, m_outBuf.size() - maxBytes);
        }
    }

    if (!m_stopping.load() && m_connected.load()) {
        m_connected.store(false);
        QMetaObject::invokeMethod(this, [this]() {
            emit connectionChanged(false);
            emit errorOccurred("BNR container stream ended");
        }, Qt::QueuedConnection);
    }
}

#else // !HAVE_BNR — stub implementations

bool NvidiaBnrFilter::connectToServer(const QString&) { return false; }
void NvidiaBnrFilter::disconnect() {}
bool NvidiaBnrFilter::isConnected() const { return false; }
QByteArray NvidiaBnrFilter::process(const float*, int) { return {}; }
void NvidiaBnrFilter::setIntensityRatio(float ratio)
{
    m_intensityRatio = std::clamp(ratio, 0.0f, 1.0f);
}

#endif // HAVE_BNR

} // namespace AetherSDR
