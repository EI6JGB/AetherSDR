#pragma once

#include <QObject>
#include <QByteArray>
#include <QString>

#ifdef HAVE_BNR
#include <QMutex>
#include <memory>
#include <thread>
#include <atomic>
#include <grpcpp/grpcpp.h>
#include "bnr.grpc.pb.h"
#endif

namespace AetherSDR {

// gRPC client for NVIDIA NIM BNR (Background Noise Removal).
// Connects to a self-hosted Docker container running the Maxine BNR model.
// Audio flows as bidirectional gRPC streaming: 48kHz mono float32, 10ms chunks.
//
// When built without HAVE_BNR, all methods are no-ops.
class NvidiaBnrFilter : public QObject {
    Q_OBJECT

public:
    explicit NvidiaBnrFilter(QObject* parent = nullptr);
    ~NvidiaBnrFilter() override;

    bool connectToServer(const QString& address = "localhost:8001");
    void disconnect();
    bool isConnected() const;

    // Process 48kHz mono float32 samples. Returns denoised samples when
    // available, or empty QByteArray if accumulating or not connected.
    QByteArray process(const float* samples, int numSamples);

    void setIntensityRatio(float ratio);
    float intensityRatio() const { return m_intensityRatio; }

signals:
    void connectionChanged(bool connected);
    void errorOccurred(const QString& message);

private:
    float m_intensityRatio{1.0f};

#ifdef HAVE_BNR
    void readerLoop();
    void closeStream();

    std::shared_ptr<grpc::Channel> m_channel;
    std::unique_ptr<nvidia::maxine::bnr::v1::MaxineBNR::Stub> m_stub;
    std::unique_ptr<grpc::ClientContext> m_context;
    std::unique_ptr<grpc::ClientReaderWriter<
        nvidia::maxine::bnr::v1::EnhanceAudioRequest,
        nvidia::maxine::bnr::v1::EnhanceAudioResponse>> m_stream;

    std::thread m_readerThread;
    std::atomic<bool> m_connected{false};
    std::atomic<bool> m_stopping{false};

    QMutex m_outMutex;
    QByteArray m_outBuf;
    QByteArray m_inAccum;

    static constexpr int kSampleRate = 48000;
    static constexpr int kFrameSamples = 480;  // 10ms at 48kHz
    static constexpr int kFrameBytes = kFrameSamples * sizeof(float);
#endif
};

} // namespace AetherSDR
