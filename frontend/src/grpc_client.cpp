#include "grpc_client.hpp"

#include <chrono>

#include <grpcpp/grpcpp.h>

namespace multi_radio {

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

void GrpcClient::StartStreaming() {
  bool expected = false;
  if (!streaming_.compare_exchange_strong(expected, true)) {
    return;
  }

  events_thread_ = std::thread([this]() { EventsLoop(); });
  messages_thread_ = std::thread([this]() { MessagesLoop(); });
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

}  // namespace multi_radio
