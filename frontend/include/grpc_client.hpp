#pragma once

#include <cstdint>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QByteArray>
#include <QObject>
#include <QVariantMap>
#include <grpcpp/grpcpp.h>

#include "radio.grpc.pb.h"

namespace multi_radio {

using RadioControlClient = v1::RadioControlService::StubInterface;
using TelemetryClient = v1::TelemetryService::StubInterface;

class GrpcClient : public QObject {
  Q_OBJECT

 public:
  explicit GrpcClient(std::string target, std::string token, QObject* parent = nullptr);
  ~GrpcClient() override;

  bool ListReceivers(std::vector<v1::ReceiverInfo>* receivers, std::string* error);
  bool StartReceiver(uint32_t receiver_id, std::string* error);
  bool StopReceiver(uint32_t receiver_id, std::string* error);
  bool SetMode(uint32_t receiver_id, v1::RadioMode mode, std::string* error);
  bool SetModeConfig(uint32_t receiver_id, const v1::ModeConfig& config, std::string* error);
  bool GetHardwareConfig(v1::HardwareConfig* config, std::string* error);
  bool SetHardwareConfig(const v1::HardwareConfig& config, std::string* error);

  void StartStreaming();
  void StopStreaming();

 signals:
  void ReceiverEventReceived(uint32_t receiver_id, int event_kind, double tuned_frequency_hz,
                             QString message, quint64 unix_ms);
  void DecodedMessageReceived(uint32_t receiver_id, QString signal_type, double frequency_hz,
                              QString payload, QVariantMap fields, quint64 unix_ms);
  void AudioFrameReceived(uint32_t receiver_id, int sample_rate_hz, QByteArray pcm_s16le, quint64 unix_ms,
                          double tuned_frequency_hz, quint64 sequence, quint64 sample_index);
  void IqFrameReceived(uint32_t receiver_id, int sample_rate_hz, QByteArray interleaved_iq_s16le,
                       quint64 unix_ms, double tuned_frequency_hz, quint64 sequence, quint64 sample_index);
  void StreamError(QString error);

 private:
  void AddAuth(grpc::ClientContext* context) const;
  void EventsLoop();
  void MessagesLoop();
  void AudioLoop();
  void IqLoop();

  std::string token_;
  std::unique_ptr<RadioControlClient> control_client_;
  std::unique_ptr<TelemetryClient> telemetry_client_;

  std::atomic<bool> streaming_{false};
  bool audio_stream_supported_ = true;
  bool iq_stream_supported_ = true;
  std::thread events_thread_;
  std::thread messages_thread_;
  std::thread audio_thread_;
  std::thread iq_thread_;
};

}  // namespace multi_radio
