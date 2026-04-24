#pragma once

#include <cstdint>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QObject>
#include <QVariantMap>
#include <grpcpp/grpcpp.h>

#include "radio.grpc.pb.h"

namespace multi_radio {

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

  void StartStreaming();
  void StopStreaming();

 signals:
  void ReceiverEventReceived(uint32_t receiver_id, int event_kind, double tuned_frequency_hz,
                             QString message, quint64 unix_ms);
  void DecodedMessageReceived(uint32_t receiver_id, QString signal_type, double frequency_hz,
                              QString payload, QVariantMap fields, quint64 unix_ms);
  void StreamError(QString error);

 private:
  void AddAuth(grpc::ClientContext* context) const;
  void EventsLoop();
  void MessagesLoop();

  std::string token_;
  std::unique_ptr<v1::RadioControlService::Stub> control_stub_;
  std::unique_ptr<v1::TelemetryService::Stub> telemetry_stub_;

  std::atomic<bool> streaming_{false};
  std::thread events_thread_;
  std::thread messages_thread_;
};

}  // namespace multi_radio
