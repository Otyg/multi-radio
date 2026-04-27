#include <grpcpp/grpcpp.h>

#include <memory>
#include <set>
#include <string>

#include "multi_radio/auth.hpp"
#include "multi_radio/receiver_manager.hpp"
#include "multi_radio/server_app.hpp"
#include "multi_radio/types.hpp"
#include "radio.grpc.pb.h"

namespace multi_radio {
namespace {

RadioMode FromProto(v1::RadioMode mode) {
  switch (mode) {
    case v1::RADIO_MODE_FIXED:
      return RadioMode::kFixed;
    case v1::RADIO_MODE_SCAN_RANGE:
      return RadioMode::kScanRange;
    case v1::RADIO_MODE_SCAN_LIST:
      return RadioMode::kScanList;
    case v1::RADIO_MODE_AIR_MARINE_PLOT:
      return RadioMode::kAirMarinePlot;
    case v1::RADIO_MODE_UNSPECIFIED:
    default:
      return RadioMode::kFixed;
  }
}

v1::RadioMode ToProto(RadioMode mode) {
  switch (mode) {
    case RadioMode::kFixed:
      return v1::RADIO_MODE_FIXED;
    case RadioMode::kScanRange:
      return v1::RADIO_MODE_SCAN_RANGE;
    case RadioMode::kScanList:
      return v1::RADIO_MODE_SCAN_LIST;
    case RadioMode::kAirMarinePlot:
      return v1::RADIO_MODE_AIR_MARINE_PLOT;
  }
  return v1::RADIO_MODE_UNSPECIFIED;
}

v1::SignalType ToProto(SignalType type) {
  switch (type) {
    case SignalType::kAis:
      return v1::SIGNAL_TYPE_AIS;
    case SignalType::kAdsb:
      return v1::SIGNAL_TYPE_ADSB;
    case SignalType::kDsc:
      return v1::SIGNAL_TYPE_DSC;
    case SignalType::kUnknown:
    default:
      return v1::SIGNAL_TYPE_UNKNOWN;
  }
}

Modulation FromProto(v1::Modulation modulation) {
  switch (modulation) {
    case v1::MODULATION_AM:
      return Modulation::kAm;
    case v1::MODULATION_WFM:
      return Modulation::kWfm;
    case v1::MODULATION_NFM:
    case v1::MODULATION_UNSPECIFIED:
    default:
      return Modulation::kNfm;
  }
}

v1::Modulation ToProto(Modulation modulation) {
  switch (modulation) {
    case Modulation::kAm:
      return v1::MODULATION_AM;
    case Modulation::kWfm:
      return v1::MODULATION_WFM;
    case Modulation::kNfm:
    default:
      return v1::MODULATION_NFM;
  }
}

v1::EventKind ToProto(EventKind kind) {
  switch (kind) {
    case EventKind::kInfo:
      return v1::EVENT_KIND_INFO;
    case EventKind::kWarning:
      return v1::EVENT_KIND_WARNING;
    case EventKind::kError:
      return v1::EVENT_KIND_ERROR;
    case EventKind::kStateChange:
      return v1::EVENT_KIND_STATE_CHANGE;
    case EventKind::kTuneHop:
      return v1::EVENT_KIND_TUNE_HOP;
  }
  return v1::EVENT_KIND_INFO;
}

ModeConfig FromProto(const v1::ModeConfig& config) {
  ModeConfig out;
  out.fixed_frequency_hz = config.fixed_frequency_hz();
  out.range_start_hz = config.range_start_hz();
  out.range_end_hz = config.range_end_hz();
  out.range_step_hz = config.range_step_hz();
  out.frequency_list_hz.assign(config.frequency_list_hz().begin(), config.frequency_list_hz().end());
  out.scan_list_channels.clear();
  out.scan_list_channels.reserve(static_cast<size_t>(config.scan_list_channels_size()));
  for (const auto& channel : config.scan_list_channels()) {
    ModeConfig::ScanListChannel parsed;
    parsed.label = channel.label();
    parsed.frequency_hz = channel.frequency_hz();
    parsed.modulation = FromProto(channel.modulation());
    parsed.channel_bandwidth_hz = channel.channel_bandwidth_hz();
    parsed.squelch_threshold_db = channel.squelch_threshold_db();
    parsed.dwell_ms = channel.dwell_ms();
    out.scan_list_channels.push_back(std::move(parsed));
  }
  out.dwell_ms = config.dwell_ms();
  out.sample_rate_hz = config.sample_rate_hz();
  out.channel_bandwidth_hz = config.channel_bandwidth_hz();
  out.hardware_bandwidth_hz = config.hardware_bandwidth_hz();
  out.dc_blocker_enabled = config.dc_blocker_enabled();
  out.dc_blocker_cutoff_hz = config.dc_blocker_cutoff_hz();
  out.center_notch_enabled = config.center_notch_enabled();
  out.center_notch_width_hz = config.center_notch_width_hz();
  out.lo_offset_enabled = config.lo_offset_enabled();
  out.lo_offset_hz = config.lo_offset_hz();
  return out;
}

void ToProto(const ModeConfig& config, v1::ModeConfig* out) {
  out->set_fixed_frequency_hz(config.fixed_frequency_hz);
  out->set_range_start_hz(config.range_start_hz);
  out->set_range_end_hz(config.range_end_hz);
  out->set_range_step_hz(config.range_step_hz);
  out->set_dwell_ms(config.dwell_ms);
  out->set_sample_rate_hz(config.sample_rate_hz);
  out->set_channel_bandwidth_hz(config.channel_bandwidth_hz);
  out->set_hardware_bandwidth_hz(config.hardware_bandwidth_hz);
  out->set_dc_blocker_enabled(config.dc_blocker_enabled);
  out->set_dc_blocker_cutoff_hz(config.dc_blocker_cutoff_hz);
  out->set_center_notch_enabled(config.center_notch_enabled);
  out->set_center_notch_width_hz(config.center_notch_width_hz);
  out->set_lo_offset_enabled(config.lo_offset_enabled);
  out->set_lo_offset_hz(config.lo_offset_hz);
  out->clear_frequency_list_hz();
  for (double frequency : config.frequency_list_hz) {
    out->add_frequency_list_hz(frequency);
  }
  out->clear_scan_list_channels();
  for (const auto& channel : config.scan_list_channels) {
    auto* added = out->add_scan_list_channels();
    added->set_label(channel.label);
    added->set_frequency_hz(channel.frequency_hz);
    added->set_modulation(ToProto(channel.modulation));
    added->set_channel_bandwidth_hz(channel.channel_bandwidth_hz);
    added->set_squelch_threshold_db(channel.squelch_threshold_db);
    added->set_dwell_ms(channel.dwell_ms);
  }
}

void FillReceiverInfo(const ReceiverStatus& status, v1::ReceiverInfo* out) {
  out->set_receiver_id(status.receiver_id);
  out->set_serial(status.serial);
  out->set_running(status.running);
  out->set_mode(ToProto(status.mode));
  ToProto(status.mode_config, out->mutable_mode_config());
  out->set_last_error(status.last_error);
}

void FillPluginInfo(const PluginInfo& info, v1::PluginInfo* out) {
  out->set_plugin_name(info.plugin_name);
  out->set_plugin_version(info.plugin_version);
  out->set_api_version(info.api_version);
  out->set_enabled(info.enabled);
  out->set_path(info.path);
  out->clear_supported_signals();
  for (const auto signal : info.supported_signals) {
    out->add_supported_signals(ToProto(signal));
  }
}

class RadioControlServiceImpl final : public v1::RadioControlService::Service {
 public:
  RadioControlServiceImpl(ReceiverManager* receiver_manager, PluginHost* plugin_host,
                          std::string auth_token)
      : receiver_manager_(receiver_manager),
        plugin_host_(plugin_host),
        auth_token_(std::move(auth_token)) {}

  grpc::Status ListReceivers(grpc::ServerContext* context, const v1::ListReceiversRequest*,
                             v1::ListReceiversResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    for (const auto& receiver : receiver_manager_->ListReceivers()) {
      FillReceiverInfo(receiver, response->add_receivers());
    }
    return grpc::Status::OK;
  }

  grpc::Status GetReceiverStatus(grpc::ServerContext* context,
                                 const v1::GetReceiverStatusRequest* request,
                                 v1::GetReceiverStatusResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    ReceiverStatus status;
    std::string error;
    if (!receiver_manager_->GetReceiverStatus(request->receiver_id(), &status, &error)) {
      return grpc::Status(grpc::StatusCode::NOT_FOUND, error);
    }
    FillReceiverInfo(status, response->mutable_receiver());
    return grpc::Status::OK;
  }

  grpc::Status StartReceiver(grpc::ServerContext* context, const v1::StartReceiverRequest* request,
                             v1::StartReceiverResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    std::string error;
    response->set_ok(receiver_manager_->StartReceiver(request->receiver_id(), &error));
    response->set_error(error);
    return grpc::Status::OK;
  }

  grpc::Status StopReceiver(grpc::ServerContext* context, const v1::StopReceiverRequest* request,
                            v1::StopReceiverResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    std::string error;
    response->set_ok(receiver_manager_->StopReceiver(request->receiver_id(), &error));
    response->set_error(error);
    return grpc::Status::OK;
  }

  grpc::Status SetMode(grpc::ServerContext* context, const v1::SetModeRequest* request,
                       v1::SetModeResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    std::string error;
    response->set_ok(receiver_manager_->SetMode(request->receiver_id(), FromProto(request->mode()), &error));
    response->set_error(error);
    return grpc::Status::OK;
  }

  grpc::Status SetModeConfig(grpc::ServerContext* context,
                             const v1::SetModeConfigRequest* request,
                             v1::SetModeConfigResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    std::string error;
    response->set_ok(receiver_manager_->SetModeConfig(request->receiver_id(),
                                                      FromProto(request->mode_config()), &error));
    response->set_error(error);
    return grpc::Status::OK;
  }

  grpc::Status ListPlugins(grpc::ServerContext* context, const v1::ListPluginsRequest*,
                           v1::ListPluginsResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    for (const auto& plugin : plugin_host_->ListPlugins()) {
      FillPluginInfo(plugin, response->add_plugins());
    }
    return grpc::Status::OK;
  }

  grpc::Status EnablePlugin(grpc::ServerContext* context, const v1::EnablePluginRequest* request,
                            v1::EnablePluginResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    std::string error;
    response->set_ok(plugin_host_->EnablePlugin(request->plugin_name(), &error));
    response->set_error(error);
    return grpc::Status::OK;
  }

  grpc::Status DisablePlugin(grpc::ServerContext* context,
                             const v1::DisablePluginRequest* request,
                             v1::DisablePluginResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    std::string error;
    response->set_ok(plugin_host_->DisablePlugin(request->plugin_name(), &error));
    response->set_error(error);
    return grpc::Status::OK;
  }

 private:
  ReceiverManager* receiver_manager_;
  PluginHost* plugin_host_;
  std::string auth_token_;
};

class TelemetryServiceImpl final : public v1::TelemetryService::Service {
 public:
  TelemetryServiceImpl(EventBus* event_bus, std::string auth_token)
      : event_bus_(event_bus), auth_token_(std::move(auth_token)) {}

  grpc::Status StreamReceiverEvents(grpc::ServerContext* context,
                                    const v1::StreamReceiverEventsRequest* request,
                                    grpc::ServerWriter<v1::ReceiverEvent>* writer) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    size_t cursor = 0;
    while (!context->IsCancelled()) {
      auto event = event_bus_->WaitForReceiverEvent(&cursor, 1000);
      if (!event.has_value()) {
        continue;
      }
      if (!request->include_all_receivers() && event->receiver_id != request->receiver_id()) {
        continue;
      }

      v1::ReceiverEvent response;
      response.set_unix_ms(event->unix_ms);
      response.set_receiver_id(event->receiver_id);
      response.set_kind(ToProto(event->kind));
      response.set_message(event->message);
      response.set_tuned_frequency_hz(event->tuned_frequency_hz);
      if (!writer->Write(response)) {
        break;
      }
    }
    return grpc::Status::OK;
  }

  grpc::Status StreamDecodedMessages(
      grpc::ServerContext* context, const v1::StreamDecodedMessagesRequest* request,
      grpc::ServerWriter<v1::DecodedMessage>* writer) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    std::set<v1::SignalType> signal_filter;
    for (int signal : request->signal_filter()) {
      signal_filter.insert(static_cast<v1::SignalType>(signal));
    }

    size_t cursor = 0;
    while (!context->IsCancelled()) {
      auto msg = event_bus_->WaitForDecodedMessage(&cursor, 1000);
      if (!msg.has_value()) {
        continue;
      }
      if (!request->include_all_receivers() && msg->receiver_id != request->receiver_id()) {
        continue;
      }

      const auto signal_type = ToProto(msg->signal_type);
      if (!signal_filter.empty() && signal_filter.find(signal_type) == signal_filter.end()) {
        continue;
      }

      v1::DecodedMessage response;
      response.set_unix_ms(msg->unix_ms);
      response.set_receiver_id(msg->receiver_id);
      response.set_signal_type(signal_type);
      response.set_frequency_hz(msg->frequency_hz);
      response.set_payload(msg->payload);
      for (const auto& [key, value] : msg->normalized_fields) {
        (*response.mutable_normalized_fields())[key] = value;
      }

      if (!writer->Write(response)) {
        break;
      }
    }

    return grpc::Status::OK;
  }

 private:
  EventBus* event_bus_;
  std::string auth_token_;
};

}  // namespace

class ServerApp::Impl {
 public:
  explicit Impl(std::string auth_token) : auth_token_(std::move(auth_token)) {}

  bool Run(ReceiverManager* receiver_manager, PluginHost* plugin_host, EventBus* event_bus,
           const std::string& bind_address, std::string* error) {
    radio_control_service_ =
        std::make_unique<RadioControlServiceImpl>(receiver_manager, plugin_host, auth_token_);
    telemetry_service_ = std::make_unique<TelemetryServiceImpl>(event_bus, auth_token_);

    grpc::ServerBuilder builder;
    builder.AddListeningPort(bind_address, grpc::InsecureServerCredentials());
    builder.RegisterService(radio_control_service_.get());
    builder.RegisterService(telemetry_service_.get());

    server_ = builder.BuildAndStart();
    if (!server_) {
      if (error != nullptr) {
        *error = "failed to start grpc server";
      }
      return false;
    }

    server_->Wait();
    return true;
  }

  void Shutdown() {
    if (server_) {
      server_->Shutdown();
    }
  }

 private:
  std::string auth_token_;
  std::unique_ptr<RadioControlServiceImpl> radio_control_service_;
  std::unique_ptr<TelemetryServiceImpl> telemetry_service_;
  std::unique_ptr<grpc::Server> server_;
};

ServerApp::ServerApp(ServerConfig config)
    : config_(std::move(config)),
      event_bus_(std::make_shared<EventBus>()),
      logger_(std::make_shared<JsonlLogger>(config_.log_dir, "radio_events", config_.log_max_bytes,
                                            config_.log_max_files)),
      plugin_host_(std::make_shared<PluginHost>(config_.plugin_dir, config_.log_dir / "plugin_state")) {}

ServerApp::~ServerApp() { Shutdown(); }

bool ServerApp::Init(std::string* error) {
  std::string plugin_error;
  if (!plugin_host_->LoadAll(&plugin_error)) {
    if (error != nullptr) {
      *error = plugin_error;
    }
    return false;
  }

  if (!config_.enable_rtlsdr) {
    if (error != nullptr) {
      *error = "RTL-SDR backend is disabled (MR_ENABLE_RTLSDR=0). Enable it to use real radio hardware.";
    }
    return false;
  }
  if (!IsRtlSdrBackendCompiled()) {
    if (error != nullptr) {
      *error = "RTL-SDR backend is not available in this build. Install librtlsdr and rebuild.";
    }
    return false;
  }

  auto factory = CreateDefaultRadioDeviceFactory(config_.enable_rtlsdr);
  if (factory == nullptr) {
    if (error != nullptr) {
      *error = "No RTL-SDR receiver detected.";
    }
    return false;
  }
  receiver_manager_ = std::make_unique<ReceiverManager>(std::move(factory), event_bus_, plugin_host_, logger_);
  impl_ = std::make_unique<Impl>(config_.auth_token);
  return true;
}

bool ServerApp::Run(std::string* error) {
  if (!receiver_manager_ || !impl_) {
    if (error != nullptr) {
      *error = "ServerApp::Init must be called before Run";
    }
    return false;
  }
  return impl_->Run(receiver_manager_.get(), plugin_host_.get(), event_bus_.get(), config_.bind_address, error);
}

void ServerApp::Shutdown() {
  if (impl_) {
    impl_->Shutdown();
  }
}

}  // namespace multi_radio
