#include "grpc_client.hpp"

#include <chrono>

#include <grpcpp/grpcpp.h>

namespace multi_radio {

namespace {

constexpr auto kUnaryRpcTimeout = std::chrono::seconds(2);

void SetUnaryDeadline(grpc::ClientContext* context) {
  if (context == nullptr) {
    return;
  }
  context->set_deadline(std::chrono::system_clock::now() + kUnaryRpcTimeout);
}

}  // namespace

GrpcClient::GrpcClient(std::string target, std::string token, QObject* parent)
    : QObject(parent), token_(std::move(token)) {
  auto channel = grpc::CreateChannel(target, grpc::InsecureChannelCredentials());
  control_client_ = v1::RadioControlService::NewStub(channel);
  telemetry_client_ = v1::TelemetryService::NewStub(channel);
}

GrpcClient::~GrpcClient() { StopStreaming(); }

bool GrpcClient::ListReceivers(std::vector<v1::ReceiverInfo>* receivers, std::string* error) {
  grpc::ClientContext context;
  AddAuth(&context);
  SetUnaryDeadline(&context);
  v1::ListReceiversRequest request;
  v1::ListReceiversResponse response;
  grpc::Status status = control_client_->ListReceivers(&context, request, &response);
  if (!status.ok()) {
    if (error != nullptr) {
      *error = status.error_message();
    }
    return false;
  }

  if (receivers != nullptr) {
    receivers->assign(response.receivers().begin(), response.receivers().end());
  }
  return true;
}

bool GrpcClient::StartReceiver(uint32_t receiver_id, std::string* error) {
  grpc::ClientContext context;
  AddAuth(&context);
  SetUnaryDeadline(&context);

  v1::StartReceiverRequest request;
  request.set_receiver_id(receiver_id);
  v1::StartReceiverResponse response;
  grpc::Status status = control_client_->StartReceiver(&context, request, &response);
  if (!status.ok()) {
    if (error != nullptr) {
      *error = status.error_message();
    }
    return false;
  }
  if (!response.ok()) {
    if (error != nullptr) {
      *error = response.error();
    }
    return false;
  }
  return true;
}

bool GrpcClient::StopReceiver(uint32_t receiver_id, std::string* error) {
  grpc::ClientContext context;
  AddAuth(&context);
  SetUnaryDeadline(&context);

  v1::StopReceiverRequest request;
  request.set_receiver_id(receiver_id);
  v1::StopReceiverResponse response;
  grpc::Status status = control_client_->StopReceiver(&context, request, &response);
  if (!status.ok()) {
    if (error != nullptr) {
      *error = status.error_message();
    }
    return false;
  }
  if (!response.ok()) {
    if (error != nullptr) {
      *error = response.error();
    }
    return false;
  }
  return true;
}

bool GrpcClient::SetMode(uint32_t receiver_id, v1::RadioMode mode, std::string* error) {
  grpc::ClientContext context;
  AddAuth(&context);
  SetUnaryDeadline(&context);

  v1::SetModeRequest request;
  request.set_receiver_id(receiver_id);
  request.set_mode(mode);
  v1::SetModeResponse response;
  grpc::Status status = control_client_->SetMode(&context, request, &response);
  if (!status.ok()) {
    if (error != nullptr) {
      *error = status.error_message();
    }
    return false;
  }
  if (!response.ok()) {
    if (error != nullptr) {
      *error = response.error();
    }
    return false;
  }
  return true;
}

bool GrpcClient::SetModeConfig(uint32_t receiver_id, const v1::ModeConfig& config, std::string* error) {
  grpc::ClientContext context;
  AddAuth(&context);
  SetUnaryDeadline(&context);

  v1::SetModeConfigRequest request;
  request.set_receiver_id(receiver_id);
  *request.mutable_mode_config() = config;
  v1::SetModeConfigResponse response;
  grpc::Status status = control_client_->SetModeConfig(&context, request, &response);
  if (!status.ok()) {
    if (error != nullptr) {
      *error = status.error_message();
    }
    return false;
  }
  if (!response.ok()) {
    if (error != nullptr) {
      *error = response.error();
    }
    return false;
  }
  return true;
}

bool GrpcClient::GetHardwareConfig(v1::HardwareConfig* config, std::string* error) {
  grpc::ClientContext context;
  AddAuth(&context);
  SetUnaryDeadline(&context);
  v1::GetHardwareConfigRequest request;
  v1::GetHardwareConfigResponse response;
  grpc::Status status = control_client_->GetHardwareConfig(&context, request, &response);
  if (!status.ok()) { if (error) *error = status.error_message(); return false; }
  if (config) *config = response.config();
  return true;
}

bool GrpcClient::SetHardwareConfig(const v1::HardwareConfig& config, std::string* error) {
  grpc::ClientContext context;
  AddAuth(&context);
  SetUnaryDeadline(&context);
  v1::SetHardwareConfigRequest request;
  *request.mutable_config() = config;
  v1::SetHardwareConfigResponse response;
  grpc::Status status = control_client_->SetHardwareConfig(&context, request, &response);
  if (!status.ok()) { if (error) *error = status.error_message(); return false; }
  if (!response.ok()) { if (error) *error = response.error(); return false; }
  return true;
}

void GrpcClient::StartStreaming() {
  bool expected = false;
  if (!streaming_.compare_exchange_strong(expected, true)) {
    return;
  }

  audio_stream_supported_ = true;
  iq_stream_supported_ = true;
  qRegisterMetaType<QVector<RadarTargetUpdate>>("QVector<multi_radio::RadarTargetUpdate>");
  events_thread_ = std::thread([this]() { EventsLoop(); });
  messages_thread_ = std::thread([this]() { MessagesLoop(); });
  audio_thread_ = std::thread([this]() { AudioLoop(); });
  iq_thread_ = std::thread([this]() { IqLoop(); });
  radar_thread_ = std::thread([this]() { RadarSnapshotLoop(); });
}

void GrpcClient::StopStreaming() {
  bool expected = true;
  if (!streaming_.compare_exchange_strong(expected, false)) {
    return;
  }

  if (events_thread_.joinable()) {
    events_thread_.join();
  }
  if (messages_thread_.joinable()) {
    messages_thread_.join();
  }
  if (audio_thread_.joinable()) {
    audio_thread_.join();
  }
  if (iq_thread_.joinable()) {
    iq_thread_.join();
  }
  if (radar_thread_.joinable()) {
    radar_thread_.join();
  }
}

void GrpcClient::AddAuth(grpc::ClientContext* context) const {
  context->AddMetadata("authorization", "Bearer " + token_);
}

void GrpcClient::EventsLoop() {
  while (streaming_.load()) {
    grpc::ClientContext context;
    AddAuth(&context);
    v1::StreamReceiverEventsRequest request;
    request.set_include_all_receivers(true);

    auto reader = telemetry_client_->StreamReceiverEvents(&context, request);
    v1::ReceiverEvent event;
    while (streaming_.load() && reader->Read(&event)) {
      emit ReceiverEventReceived(event.receiver_id(), event.kind(), event.tuned_frequency_hz(),
                                 QString::fromStdString(event.message()), event.unix_ms());
    }

    grpc::Status status = reader->Finish();
    if (!status.ok() && streaming_.load()) {
      emit StreamError(QString::fromStdString("Event stream error: " + status.error_message()));
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

void GrpcClient::MessagesLoop() {
  while (streaming_.load()) {
    grpc::ClientContext context;
    AddAuth(&context);
    v1::StreamDecodedMessagesRequest request;
    request.set_include_all_receivers(true);

    auto reader = telemetry_client_->StreamDecodedMessages(&context, request);
    v1::DecodedMessage msg;
    while (streaming_.load() && reader->Read(&msg)) {
      QVariantMap fields;
      for (const auto& [key, value] : msg.normalized_fields()) {
        fields.insert(QString::fromStdString(key), QString::fromStdString(value));
      }
      emit DecodedMessageReceived(msg.receiver_id(),
                                  QString::fromStdString(v1::SignalType_Name(msg.signal_type())),
                                  msg.frequency_hz(), QString::fromStdString(msg.payload()), fields,
                                  msg.unix_ms());
    }

    grpc::Status status = reader->Finish();
    if (!status.ok() && streaming_.load()) {
      emit StreamError(QString::fromStdString("Decoded stream error: " + status.error_message()));
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

void GrpcClient::AudioLoop() {
  while (streaming_.load() && audio_stream_supported_) {
    grpc::ClientContext context;
    AddAuth(&context);
    v1::StreamAudioFramesRequest request;
    request.set_include_all_receivers(true);

    auto reader = telemetry_client_->StreamAudioFrames(&context, request);
    v1::AudioFrame frame;
    while (streaming_.load() && reader->Read(&frame)) {
      const std::string& pcm = frame.pcm_s16le();
      emit AudioFrameReceived(frame.receiver_id(), static_cast<int>(frame.sample_rate_hz()),
                              QByteArray(pcm.data(), static_cast<int>(pcm.size())), frame.unix_ms(),
                              frame.tuned_frequency_hz(), frame.sequence(), frame.sample_index());
    }

    grpc::Status status = reader->Finish();
    if (!status.ok() && streaming_.load()) {
      if (status.error_code() == grpc::StatusCode::UNIMPLEMENTED) {
        audio_stream_supported_ = false;
        emit StreamError("Audio stream unavailable on server; using event audio fallback.");
        break;
      }
      emit StreamError(QString::fromStdString("Audio stream error: " + status.error_message()));
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

void GrpcClient::IqLoop() {
  while (streaming_.load() && iq_stream_supported_) {
    grpc::ClientContext context;
    AddAuth(&context);
    v1::StreamIqFramesRequest request;
    request.set_include_all_receivers(true);

    auto reader = telemetry_client_->StreamIqFrames(&context, request);
    v1::IqFrame frame;
    while (streaming_.load() && reader->Read(&frame)) {
      const std::string& iq = frame.interleaved_iq_s16le();
      emit IqFrameReceived(frame.receiver_id(), static_cast<int>(frame.sample_rate_hz()),
                           QByteArray(iq.data(), static_cast<int>(iq.size())), frame.unix_ms(),
                           frame.tuned_frequency_hz(), frame.sequence(), frame.sample_index());
    }

    grpc::Status status = reader->Finish();
    if (!status.ok() && streaming_.load()) {
      if (status.error_code() == grpc::StatusCode::UNIMPLEMENTED) {
        iq_stream_supported_ = false;
        emit StreamError("IQ stream unavailable on server.");
        break;
      }
      emit StreamError(QString::fromStdString("IQ stream error: " + status.error_message()));
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

void GrpcClient::RadarSnapshotLoop() {
  while (streaming_.load()) {
    grpc::ClientContext context;
    AddAuth(&context);
    v1::StreamRadarSnapshotsRequest request;
    request.set_include_all_receivers(true);

    auto reader = telemetry_client_->StreamRadarSnapshots(&context, request);
    v1::RadarSnapshot snap;
    while (streaming_.load() && reader->Read(&snap)) {
      QVector<RadarTargetUpdate> targets;
      targets.reserve(snap.targets_size());
      for (const auto& t : snap.targets()) {
        RadarTargetUpdate u;
        u.id    = QString::fromStdString(t.id());
        u.label = QString::fromStdString(t.label());
        u.kind  = (t.kind() == "AIR") ? RadarTargetKind::kAircraft
                : (t.kind() == "SEA") ? RadarTargetKind::kVessel
                                      : RadarTargetKind::kUnknown;
        u.lat       = t.lat();
        u.lon       = t.lon();
        u.sog       = t.sog_knots();
        u.cog       = t.cog_degrees();
        u.altitude  = t.has_altitude() ? t.altitude_ft()
                                       : std::numeric_limits<double>::quiet_NaN();
        u.unix_ms   = t.last_seen_ms();
        targets.append(u);
      }
      QStringList removed;
      for (const auto& rid : snap.removed_ids())
        removed.append(QString::fromStdString(rid));

      emit RadarSnapshotReceived(targets, removed, snap.snapshot_ms());
    }

    grpc::Status status = reader->Finish();
    if (!status.ok() && streaming_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
}

}  // namespace multi_radio
