#include <grpcpp/grpcpp.h>

#include <chrono>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>

#include "multi_radio/auth.hpp"
#include "multi_radio/hardware_config.hpp"
#include "multi_radio/receiver_manager.hpp"
#include "multi_radio/server_app.hpp"
#include "multi_radio/types.hpp"
#include "radio.grpc.pb.h"

namespace multi_radio {
namespace {

/* ------------------------------------------------------------------ */
/* Server-side hardware config persistence                              */
/* ------------------------------------------------------------------ */

// Writes ppm_correction to hardware.conf (creates parent dir if possible).
bool SaveHardwarePpm(int ppm, std::string* error) {
  const std::string path = HardwareConfigPath();
  // Best-effort mkdir for the directory.
  const auto slash = path.rfind('/');
  if (slash != std::string::npos) {
    const std::string dir = path.substr(0, slash);
    std::string cmd = "mkdir -p " + dir;
    std::system(cmd.c_str());  // NOLINT(cert-env33-c)
  }
  std::ofstream f(path, std::ios::trunc);
  if (!f) { if (error) *error = "cannot write " + path; return false; }
  f << "ppm=" << ppm << "\n";
  return true;
}

// In-memory cache so GetHardwareConfig doesn't need to re-read the file.
struct HardwareConfigCache {
  std::mutex mu;
  int ppm = 0;
  bool loaded = false;

  int GetPpm() {
    std::lock_guard<std::mutex> lk(mu);
    if (!loaded) { ppm = LoadHardwarePpm(); loaded = true; }
    return ppm;
  }
  bool SetPpm(int v, std::string* err) {
    if (!SaveHardwarePpm(v, err)) return false;
    std::lock_guard<std::mutex> lk(mu);
    ppm = v; loaded = true;
    return true;
  }
};

HardwareConfigCache& HwCfg() {
  static HardwareConfigCache instance;
  return instance;
}

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
    case v1::MODULATION_AM:   return Modulation::kAm;
    case v1::MODULATION_WFM:  return Modulation::kWfm;
    case v1::MODULATION_FSK:  return Modulation::kFsk;
    case v1::MODULATION_GMSK: return Modulation::kGmsk;
    case v1::MODULATION_PPM:  return Modulation::kPpm;
    case v1::MODULATION_ADSB:     return Modulation::kAdsbMod;
    case v1::MODULATION_AIS_DUAL: return Modulation::kAisDual;
    case v1::MODULATION_VDES_ASM: return Modulation::kVdesAsm;
    case v1::MODULATION_NFM:
    case v1::MODULATION_UNSPECIFIED:
    default:                  return Modulation::kNfm;
  }
}

v1::Modulation ToProto(Modulation modulation) {
  switch (modulation) {
    case Modulation::kAm:   return v1::MODULATION_AM;
    case Modulation::kWfm:  return v1::MODULATION_WFM;
    case Modulation::kFsk:  return v1::MODULATION_FSK;
    case Modulation::kGmsk: return v1::MODULATION_GMSK;
    case Modulation::kPpm:     return v1::MODULATION_PPM;
    case Modulation::kAdsbMod: return v1::MODULATION_ADSB;
    case Modulation::kAisDual: return v1::MODULATION_AIS_DUAL;
    case Modulation::kVdesAsm: return v1::MODULATION_VDES_ASM;
    case Modulation::kNfm:
    default:                   return v1::MODULATION_NFM;
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
  out.fixed_modulation = (config.fixed_modulation() == v1::MODULATION_UNSPECIFIED)
                             ? Modulation::kWfm
                             : FromProto(config.fixed_modulation());
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
    parsed.use_default_squelch = channel.use_default_squelch();
    parsed.audio_gain_db = channel.audio_gain_db();
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
  out.scan_list_monitor_mode = config.scan_list_monitor_mode();
  out.scan_list_default_squelch_db = config.scan_list_default_squelch_db();
  out.audio_hpf300_enabled = config.audio_hpf300_enabled();
  out.audio_lpf3k5_enabled = config.audio_lpf3k5_enabled();
  out.audio_bpf_voice_enabled = config.audio_bpf_voice_enabled();
  out.audio_lpf4k5_enabled = config.audio_lpf4k5_enabled();
  out.scan_list_channel_locked = config.scan_list_channel_locked();
  out.scan_list_locked_channel_index = static_cast<int32_t>(config.scan_list_locked_channel_index());
  if (config.gmsk_baud_rate() > 0) out.gmsk_baud_rate = config.gmsk_baud_rate();
  if (config.gmsk_bt() > 0.0f)     out.gmsk_bt = config.gmsk_bt();
  if (config.gmsk_modulation_index() > 0.0f) out.gmsk_modulation_index = config.gmsk_modulation_index();
  out.gmsk_decoder        = config.gmsk_decoder();
  out.gmsk_postprocessor  = config.gmsk_postprocessor();
  out.gmsk_nrzi_invert    = config.gmsk_nrzi_invert();
  out.ppm_correction      = config.ppm_correction();
  if (config.ppm_bit_duration_us() > 0) out.ppm_bit_duration_us = config.ppm_bit_duration_us();
  if (config.ppm_data_rate_bps() > 0)   out.ppm_data_rate_bps   = config.ppm_data_rate_bps();
  if (config.vdes_asm_bit_rate_bps() > 0) out.vdes_asm_bit_rate_bps = config.vdes_asm_bit_rate_bps();
  if (config.vdes_asm_pll_bw() > 0.0f) out.vdes_asm_pll_bw = config.vdes_asm_pll_bw();
  if (config.vdes_asm_candidate_bits() > 0) out.vdes_asm_candidate_bits = config.vdes_asm_candidate_bits();
  out.vdes_asm_sync_errors_max = config.vdes_asm_sync_errors_max();
  if (config.adsb_agc_bandwidth() > 0.0f) out.adsb_agc_bandwidth = config.adsb_agc_bandwidth();
  if (config.adsb_agc_target_level() > 0.0f) out.adsb_agc_target_level = config.adsb_agc_target_level();
  out.rnnoise_enabled = config.rnnoise_enabled();
  out.rnnoise_strength = config.rnnoise_strength();
  return out;
}

void ToProto(const ModeConfig& config, v1::ModeConfig* out) {
  out->set_fixed_frequency_hz(config.fixed_frequency_hz);
  out->set_fixed_modulation(ToProto(config.fixed_modulation));
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
  out->set_scan_list_monitor_mode(config.scan_list_monitor_mode);
  out->set_scan_list_default_squelch_db(config.scan_list_default_squelch_db);
  out->set_audio_hpf300_enabled(config.audio_hpf300_enabled);
  out->set_audio_lpf3k5_enabled(config.audio_lpf3k5_enabled);
  out->set_audio_bpf_voice_enabled(config.audio_bpf_voice_enabled);
  out->set_audio_lpf4k5_enabled(config.audio_lpf4k5_enabled);
  out->set_scan_list_channel_locked(config.scan_list_channel_locked);
  out->set_scan_list_locked_channel_index(
      static_cast<uint32_t>(std::max(0, config.scan_list_locked_channel_index)));
  out->set_gmsk_baud_rate(config.gmsk_baud_rate);
  out->set_gmsk_bt(config.gmsk_bt);
  out->set_gmsk_modulation_index(config.gmsk_modulation_index);
  out->set_gmsk_decoder(config.gmsk_decoder);
  out->set_gmsk_postprocessor(config.gmsk_postprocessor);
  out->set_gmsk_nrzi_invert(config.gmsk_nrzi_invert);
  out->set_ppm_correction(config.ppm_correction);
  out->set_ppm_bit_duration_us(config.ppm_bit_duration_us);
  out->set_ppm_data_rate_bps(config.ppm_data_rate_bps);
  out->set_vdes_asm_bit_rate_bps(config.vdes_asm_bit_rate_bps);
  out->set_vdes_asm_pll_bw(config.vdes_asm_pll_bw);
  out->set_vdes_asm_candidate_bits(config.vdes_asm_candidate_bits);
  out->set_vdes_asm_sync_errors_max(config.vdes_asm_sync_errors_max);
  out->set_adsb_agc_bandwidth(config.adsb_agc_bandwidth);
  out->set_adsb_agc_target_level(config.adsb_agc_target_level);
  out->set_rnnoise_enabled(config.rnnoise_enabled);
  out->set_rnnoise_strength(config.rnnoise_strength);
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
    added->set_use_default_squelch(channel.use_default_squelch);
    added->set_audio_gain_db(channel.audio_gain_db);
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

void PublishRelayEvent(EventBus* event_bus, EventKind kind, const std::string& message) {
  if (event_bus == nullptr) return;
  ReceiverEvent event;
  event.unix_ms = UnixMillisNow();
  event.receiver_id = 0;
  event.kind = kind;
  event.message = message;
  event.tuned_frequency_hz = 0.0;
  event_bus->PublishReceiverEvent(event);
}

class RemoteRadioClient {
 public:
  RemoteRadioClient(std::string target, std::string token)
      : target_(std::move(target)),
        token_(std::move(token)) {
    auto channel = grpc::CreateChannel(target_, grpc::InsecureChannelCredentials());
    control_client_ = v1::RadioControlService::NewStub(channel);
    telemetry_client_ = v1::TelemetryService::NewStub(channel);
  }

  ~RemoteRadioClient() { StopIqRelay(); }

  bool ListReceivers(std::vector<ReceiverStatus>* receivers, std::string* error) {
    grpc::ClientContext context;
    AddAuth(&context);
    SetUnaryDeadline(&context);
    v1::ListReceiversRequest request;
    v1::ListReceiversResponse response;
    const grpc::Status status = control_client_->ListReceivers(&context, request, &response);
    if (!status.ok()) {
      if (error != nullptr) *error = status.error_message();
      return false;
    }
    if (receivers != nullptr) {
      receivers->clear();
      receivers->reserve(static_cast<size_t>(response.receivers_size()));
      for (const auto& info : response.receivers()) {
        ReceiverStatus out;
        out.receiver_id = info.receiver_id();
        out.serial = info.serial();
        out.running = info.running();
        out.mode = FromProto(info.mode());
        out.mode_config = FromProto(info.mode_config());
        out.last_error = info.last_error();
        receivers->push_back(std::move(out));
      }
    }
    if (error != nullptr) error->clear();
    return true;
  }

  bool GetReceiverStatus(uint32_t receiver_id, ReceiverStatus* receiver, std::string* error) {
    grpc::ClientContext context;
    AddAuth(&context);
    SetUnaryDeadline(&context);
    v1::GetReceiverStatusRequest request;
    request.set_receiver_id(receiver_id);
    v1::GetReceiverStatusResponse response;
    const grpc::Status status = control_client_->GetReceiverStatus(&context, request, &response);
    if (!status.ok()) {
      if (error != nullptr) *error = status.error_message();
      return false;
    }
    if (receiver != nullptr) {
      receiver->receiver_id = response.receiver().receiver_id();
      receiver->serial = response.receiver().serial();
      receiver->running = response.receiver().running();
      receiver->mode = FromProto(response.receiver().mode());
      receiver->mode_config = FromProto(response.receiver().mode_config());
      receiver->last_error = response.receiver().last_error();
    }
    if (error != nullptr) error->clear();
    return true;
  }

  bool StartReceiver(uint32_t receiver_id, std::string* error) {
    grpc::ClientContext context;
    AddAuth(&context);
    SetUnaryDeadline(&context);
    v1::StartReceiverRequest request;
    request.set_receiver_id(receiver_id);
    v1::StartReceiverResponse response;
    const grpc::Status status = control_client_->StartReceiver(&context, request, &response);
    return CheckBoolResponse(status, response.ok(), response.error(), error);
  }

  bool StopReceiver(uint32_t receiver_id, std::string* error) {
    grpc::ClientContext context;
    AddAuth(&context);
    SetUnaryDeadline(&context);
    v1::StopReceiverRequest request;
    request.set_receiver_id(receiver_id);
    v1::StopReceiverResponse response;
    const grpc::Status status = control_client_->StopReceiver(&context, request, &response);
    return CheckBoolResponse(status, response.ok(), response.error(), error);
  }

  bool SetMode(uint32_t receiver_id, RadioMode mode, std::string* error) {
    grpc::ClientContext context;
    AddAuth(&context);
    SetUnaryDeadline(&context);
    v1::SetModeRequest request;
    request.set_receiver_id(receiver_id);
    request.set_mode(ToProto(mode));
    v1::SetModeResponse response;
    const grpc::Status status = control_client_->SetMode(&context, request, &response);
    return CheckBoolResponse(status, response.ok(), response.error(), error);
  }

  bool SetModeConfig(uint32_t receiver_id, const ModeConfig& config, std::string* error) {
    grpc::ClientContext context;
    AddAuth(&context);
    SetUnaryDeadline(&context);
    v1::SetModeConfigRequest request;
    request.set_receiver_id(receiver_id);
    ToProto(config, request.mutable_mode_config());
    v1::SetModeConfigResponse response;
    const grpc::Status status = control_client_->SetModeConfig(&context, request, &response);
    return CheckBoolResponse(status, response.ok(), response.error(), error);
  }

  bool SetHardwareConfig(int ppm_correction, std::string* error) {
    grpc::ClientContext context;
    AddAuth(&context);
    SetUnaryDeadline(&context);
    v1::SetHardwareConfigRequest request;
    request.mutable_config()->set_ppm_correction(ppm_correction);
    v1::SetHardwareConfigResponse response;
    const grpc::Status status = control_client_->SetHardwareConfig(&context, request, &response);
    return CheckBoolResponse(status, response.ok(), response.error(), error);
  }

  bool GetHardwareConfig(int* ppm_correction, std::string* error) {
    grpc::ClientContext context;
    AddAuth(&context);
    SetUnaryDeadline(&context);
    v1::GetHardwareConfigRequest request;
    v1::GetHardwareConfigResponse response;
    const grpc::Status status = control_client_->GetHardwareConfig(&context, request, &response);
    if (!status.ok()) {
      if (error != nullptr) *error = status.error_message();
      return false;
    }
    if (ppm_correction != nullptr) {
      *ppm_correction = response.config().ppm_correction();
    }
    if (error != nullptr) error->clear();
    return true;
  }

  void StartIqRelay(ReceiverManager* receiver_manager, EventBus* event_bus) {
    if (relay_running_.exchange(true)) return;
    relay_thread_ = std::thread(&RemoteRadioClient::IqRelayLoop, this, receiver_manager, event_bus);
  }

  void StopIqRelay() {
    if (!relay_running_.exchange(false)) return;
    {
      std::lock_guard<std::mutex> lock(relay_context_mu_);
      if (relay_context_ != nullptr) {
        relay_context_->TryCancel();
      }
    }
    if (relay_thread_.joinable()) {
      relay_thread_.join();
    }
  }

 private:
  static bool CheckBoolResponse(const grpc::Status& status, bool ok, const std::string& rpc_error,
                                std::string* error) {
    if (!status.ok()) {
      if (error != nullptr) *error = status.error_message();
      return false;
    }
    if (!ok) {
      if (error != nullptr) *error = rpc_error;
      return false;
    }
    if (error != nullptr) error->clear();
    return true;
  }

  void AddAuth(grpc::ClientContext* context) const {
    if (context == nullptr || token_.empty()) return;
    context->AddMetadata("authorization", "Bearer " + token_);
  }

  void SetUnaryDeadline(grpc::ClientContext* context) const {
    if (context == nullptr) return;
    context->set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(2));
  }

  void IqRelayLoop(ReceiverManager* receiver_manager, EventBus* event_bus) {
    bool announced_connect = false;
    while (relay_running_.load()) {
      grpc::ClientContext context;
      AddAuth(&context);
      v1::StreamRawIqFramesRequest request;
      request.set_include_all_receivers(true);
      request.set_receiver_id(0);
      {
        std::lock_guard<std::mutex> lock(relay_context_mu_);
        relay_context_ = &context;
      }
      auto reader = telemetry_client_->StreamRawIqFrames(&context, request);
      if (!announced_connect) {
        PublishRelayEvent(event_bus, EventKind::kInfo,
                          "remote IQ relay connecting to " + target_);
      }

      v1::IqFrame frame;
      while (relay_running_.load() && reader->Read(&frame)) {
        if (!announced_connect) {
          PublishRelayEvent(event_bus, EventKind::kInfo,
                            "remote IQ relay connected to " + target_);
          announced_connect = true;
        }
        IqFrame local_frame;
        local_frame.unix_ms = frame.unix_ms();
        local_frame.receiver_id = frame.receiver_id();
        local_frame.sample_rate_hz = frame.sample_rate_hz();
        local_frame.tuned_frequency_hz = frame.tuned_frequency_hz();
        local_frame.sequence = frame.sequence();
        local_frame.sample_index = frame.sample_index();
        const std::string& bytes = frame.interleaved_iq_s16le();
        local_frame.interleaved_iq_s16le.resize(bytes.size() / sizeof(int16_t));
        if (!bytes.empty()) {
          std::memcpy(local_frame.interleaved_iq_s16le.data(), bytes.data(), bytes.size());
        }
        receiver_manager->SubmitIqFrame(local_frame.receiver_id, local_frame, nullptr);
      }

      const grpc::Status status = reader->Finish();
      {
        std::lock_guard<std::mutex> lock(relay_context_mu_);
        relay_context_ = nullptr;
      }
      if (!relay_running_.load()) break;

      const std::string why = status.ok() ? "stream ended" : status.error_message();
      PublishRelayEvent(event_bus, EventKind::kWarning,
                        "remote IQ relay disconnected from " + target_ + ": " + why);
      announced_connect = false;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  std::string target_;
  std::string token_;
  std::unique_ptr<v1::RadioControlService::Stub> control_client_;
  std::unique_ptr<v1::TelemetryService::Stub> telemetry_client_;
  std::atomic<bool> relay_running_{false};
  std::thread relay_thread_;
  std::mutex relay_context_mu_;
  grpc::ClientContext* relay_context_ = nullptr;
};

class RadioControlServiceImpl final : public v1::RadioControlService::Service {
 public:
  RadioControlServiceImpl(ReceiverManager* receiver_manager, PluginHost* plugin_host,
                          TrackDatabase* track_db, std::string auth_token,
                          RemoteRadioClient* remote_client)
      : receiver_manager_(receiver_manager),
        plugin_host_(plugin_host),
        track_db_(track_db),
        auth_token_(std::move(auth_token)),
        remote_client_(remote_client) {}

  grpc::Status ListReceivers(grpc::ServerContext* context, const v1::ListReceiversRequest*,
                             v1::ListReceiversResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    std::vector<ReceiverStatus> receivers;
    if (remote_client_ != nullptr) {
      std::string error;
      if (!remote_client_->ListReceivers(&receivers, &error)) {
        return grpc::Status(grpc::StatusCode::UNAVAILABLE, error);
      }
    } else {
      receivers = receiver_manager_->ListReceivers();
    }
    for (const auto& receiver : receivers) {
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
    const bool ok = (remote_client_ != nullptr)
                        ? remote_client_->GetReceiverStatus(request->receiver_id(), &status, &error)
                        : receiver_manager_->GetReceiverStatus(request->receiver_id(), &status, &error);
    if (!ok) {
      return grpc::Status(remote_client_ != nullptr ? grpc::StatusCode::UNAVAILABLE
                                                    : grpc::StatusCode::NOT_FOUND,
                          error);
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
    bool ok = receiver_manager_->StartReceiver(request->receiver_id(), &error);
    if (ok && remote_client_ != nullptr) {
      std::string remote_error;
      ok = remote_client_->StartReceiver(request->receiver_id(), &remote_error);
      if (!ok) {
        std::string rollback_error;
        receiver_manager_->StopReceiver(request->receiver_id(), &rollback_error);
        error = remote_error;
      }
    }
    response->set_ok(ok);
    response->set_error(error);
    return grpc::Status::OK;
  }

  grpc::Status StopReceiver(grpc::ServerContext* context, const v1::StopReceiverRequest* request,
                            v1::StopReceiverResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    std::string error;
    bool ok = true;
    if (remote_client_ != nullptr) {
      ok = remote_client_->StopReceiver(request->receiver_id(), &error);
    }
    std::string local_error;
    const bool local_ok = receiver_manager_->StopReceiver(request->receiver_id(), &local_error);
    if (!local_ok && ok) {
      ok = false;
      error = local_error;
    } else if (!local_ok && !ok && error.empty()) {
      error = local_error;
    }
    response->set_ok(ok);
    response->set_error(error);
    return grpc::Status::OK;
  }

  grpc::Status SetMode(grpc::ServerContext* context, const v1::SetModeRequest* request,
                       v1::SetModeResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    std::string error;
    const RadioMode mode = FromProto(request->mode());
    bool ok = receiver_manager_->SetMode(request->receiver_id(), mode, &error);
    if (ok && remote_client_ != nullptr) {
      ok = remote_client_->SetMode(request->receiver_id(), mode, &error);
    }
    response->set_ok(ok);
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
    const ModeConfig config = FromProto(request->mode_config());
    bool ok = receiver_manager_->SetModeConfig(request->receiver_id(), config, &error);
    if (ok && remote_client_ != nullptr) {
      ok = remote_client_->SetModeConfig(request->receiver_id(), config, &error);
    }
    response->set_ok(ok);
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

  grpc::Status SetHardwareConfig(grpc::ServerContext* context,
                                  const v1::SetHardwareConfigRequest* request,
                                  v1::SetHardwareConfigResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_))
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    std::string error;
    bool ok = false;
    if (remote_client_ != nullptr) {
      ok = remote_client_->SetHardwareConfig(request->config().ppm_correction(), &error);
    } else {
      ok = HwCfg().SetPpm(request->config().ppm_correction(), &error);
    }
    response->set_ok(ok);
    response->set_error(error);
    if (ok) {
      receiver_manager_->ApplyHardwarePpm(request->config().ppm_correction());
    }
    return grpc::Status::OK;
  }

  grpc::Status GetHardwareConfig(grpc::ServerContext* context,
                                  const v1::GetHardwareConfigRequest* /*request*/,
                                  v1::GetHardwareConfigResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_))
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    int ppm = 0;
    std::string error;
    const bool ok = (remote_client_ != nullptr)
                        ? remote_client_->GetHardwareConfig(&ppm, &error)
                        : (ppm = HwCfg().GetPpm(), true);
    if (!ok) {
      return grpc::Status(grpc::StatusCode::UNAVAILABLE, error);
    }
    response->mutable_config()->set_ppm_correction(ppm);
    return grpc::Status::OK;
  }

  grpc::Status ClearNameDatabase(grpc::ServerContext* context,
                                  const v1::ClearNameDatabaseRequest* /*request*/,
                                  v1::ClearNameDatabaseResponse* response) override {
    if (!auth::ValidateBearerToken(*context, auth_token_))
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    if (track_db_) track_db_->ClearEntities();
    response->set_ok(true);
    return grpc::Status::OK;
  }

 private:
  ReceiverManager* receiver_manager_;
  PluginHost*      plugin_host_;
  TrackDatabase*   track_db_;
  std::string      auth_token_;
  RemoteRadioClient* remote_client_ = nullptr;
};

class TelemetryServiceImpl final : public v1::TelemetryService::Service {
 public:
  TelemetryServiceImpl(EventBus* event_bus, TargetTracker* target_tracker, std::string auth_token)
      : event_bus_(event_bus), target_tracker_(target_tracker), auth_token_(std::move(auth_token)) {}

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

  grpc::Status StreamAudioFrames(grpc::ServerContext* context,
                                 const v1::StreamAudioFramesRequest* request,
                                 grpc::ServerWriter<v1::AudioFrame>* writer) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    // Audio should be live-only for new subscribers. Starting from the current
    // tail avoids replaying buffered historical frames at connect time.
    size_t cursor = event_bus_->AudioFrameCursorNow();
    while (!context->IsCancelled()) {
      auto frame = event_bus_->WaitForAudioFrame(&cursor, 1000);
      if (!frame.has_value()) {
        continue;
      }
      if (!request->include_all_receivers() && frame->receiver_id != request->receiver_id()) {
        continue;
      }

      v1::AudioFrame response;
      response.set_unix_ms(frame->unix_ms);
      response.set_receiver_id(frame->receiver_id);
      response.set_sample_rate_hz(frame->sample_rate_hz);
      response.set_tuned_frequency_hz(frame->tuned_frequency_hz);
      response.set_sequence(frame->sequence);
      response.set_sample_index(frame->sample_index);
      if (!frame->pcm_s16le.empty()) {
        const char* bytes = reinterpret_cast<const char*>(frame->pcm_s16le.data());
        const size_t byte_count = frame->pcm_s16le.size() * sizeof(int16_t);
        response.set_pcm_s16le(bytes, static_cast<int>(byte_count));
      }
      if (!writer->Write(response)) {
        break;
      }
    }
    return grpc::Status::OK;
  }

  grpc::Status StreamIqFrames(grpc::ServerContext* context,
                              const v1::StreamIqFramesRequest* request,
                              grpc::ServerWriter<v1::IqFrame>* writer) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    // IQ visualization stream is live-only for new subscribers.
    size_t cursor = event_bus_->IqFrameCursorNow();
    while (!context->IsCancelled()) {
      auto frame = event_bus_->WaitForIqFrame(&cursor, 1000);
      if (!frame.has_value()) {
        continue;
      }
      if (!request->include_all_receivers() && frame->receiver_id != request->receiver_id()) {
        continue;
      }

      v1::IqFrame response;
      response.set_unix_ms(frame->unix_ms);
      response.set_receiver_id(frame->receiver_id);
      response.set_sample_rate_hz(frame->sample_rate_hz);
      response.set_tuned_frequency_hz(frame->tuned_frequency_hz);
      response.set_sequence(frame->sequence);
      response.set_sample_index(frame->sample_index);
      if (!frame->interleaved_iq_s16le.empty()) {
        const char* bytes = reinterpret_cast<const char*>(frame->interleaved_iq_s16le.data());
        const size_t byte_count = frame->interleaved_iq_s16le.size() * sizeof(int16_t);
        response.set_interleaved_iq_s16le(bytes, static_cast<int>(byte_count));
      }
      if (!writer->Write(response)) {
        break;
      }
    }
    return grpc::Status::OK;
  }

  grpc::Status StreamRawIqFrames(grpc::ServerContext* context,
                                 const v1::StreamRawIqFramesRequest* request,
                                 grpc::ServerWriter<v1::IqFrame>* writer) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }

    size_t cursor = event_bus_->RawIqFrameCursorNow();
    while (!context->IsCancelled()) {
      auto frame = event_bus_->WaitForRawIqFrame(&cursor, 1000);
      if (!frame.has_value()) {
        continue;
      }
      if (!request->include_all_receivers() && frame->receiver_id != request->receiver_id()) {
        continue;
      }

      v1::IqFrame response;
      response.set_unix_ms(frame->unix_ms);
      response.set_receiver_id(frame->receiver_id);
      response.set_sample_rate_hz(frame->sample_rate_hz);
      response.set_tuned_frequency_hz(frame->tuned_frequency_hz);
      response.set_sequence(frame->sequence);
      response.set_sample_index(frame->sample_index);
      if (!frame->interleaved_iq_s16le.empty()) {
        response.set_interleaved_iq_s16le(
            frame->interleaved_iq_s16le.data(),
            static_cast<int>(frame->interleaved_iq_s16le.size() * sizeof(int16_t)));
      }
      if (!writer->Write(response)) {
        break;
      }
    }

    return grpc::Status::OK;
  }

  grpc::Status StreamRadarSnapshots(grpc::ServerContext* context,
                                    const v1::StreamRadarSnapshotsRequest* request,
                                    grpc::ServerWriter<v1::RadarSnapshot>* writer) override {
    if (!auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }
    if (!target_tracker_) {
      return grpc::Status(grpc::StatusCode::UNAVAILABLE, "target tracker not available");
    }

    while (!context->IsCancelled()) {
      // Wait 2 s between snapshots to avoid flooding the client.
      for (int i = 0; i < 20 && !context->IsCancelled(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (context->IsCancelled()) break;

      const auto snap = target_tracker_->TakeSnapshot();

      v1::RadarSnapshot response;
      response.set_snapshot_ms(snap.snapshot_ms);
      for (const auto& t : snap.targets) {
        auto* pt = response.add_targets();
        pt->set_id(t.id);
        pt->set_label(t.label);
        pt->set_kind(t.kind);
        pt->set_lat(t.lat);
        pt->set_lon(t.lon);
        pt->set_sog_knots(t.sog_knots);
        pt->set_cog_degrees(t.cog_degrees);
        pt->set_has_altitude(t.has_altitude);
        pt->set_altitude_ft(t.altitude_ft);
        pt->set_last_seen_ms(t.last_seen_ms);
      }
      for (const auto& rid : snap.removed_ids) {
        response.add_removed_ids(rid);
      }

      if (!writer->Write(response)) break;
    }
    return grpc::Status::OK;
  }

 private:
  EventBus* event_bus_;
  TargetTracker* target_tracker_;
  std::string auth_token_;
};

class PositionServiceImpl final : public v1::PositionService::Service {
 public:
  PositionServiceImpl(TargetTracker* target_tracker, std::string auth_token)
      : target_tracker_(target_tracker), auth_token_(std::move(auth_token)) {}

  grpc::Status StreamPositions(grpc::ServerContext* context,
                                const v1::StreamPositionsRequest* /*request*/,
                                grpc::ServerWriter<v1::RadarSnapshot>* writer) override {
    if (!auth_token_.empty() && !auth::ValidateBearerToken(*context, auth_token_)) {
      return grpc::Status(grpc::StatusCode::UNAUTHENTICATED, "invalid bearer token");
    }
    if (!target_tracker_) {
      return grpc::Status(grpc::StatusCode::UNAVAILABLE, "target tracker not available");
    }

    while (!context->IsCancelled()) {
      for (int i = 0; i < 20 && !context->IsCancelled(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (context->IsCancelled()) break;

      const auto snap = target_tracker_->TakeSnapshot();

      v1::RadarSnapshot response;
      response.set_snapshot_ms(snap.snapshot_ms);
      for (const auto& t : snap.targets) {
        auto* pt = response.add_targets();
        pt->set_id(t.id);
        pt->set_label(t.label);
        pt->set_kind(t.kind);
        pt->set_lat(t.lat);
        pt->set_lon(t.lon);
        pt->set_sog_knots(t.sog_knots);
        pt->set_cog_degrees(t.cog_degrees);
        pt->set_has_altitude(t.has_altitude);
        pt->set_altitude_ft(t.altitude_ft);
        pt->set_last_seen_ms(t.last_seen_ms);
      }
      for (const auto& rid : snap.removed_ids) {
        response.add_removed_ids(rid);
      }

      if (!writer->Write(response)) break;
    }
    return grpc::Status::OK;
  }

 private:
  TargetTracker* target_tracker_;
  std::string    auth_token_;
};

}  // namespace

class ServerApp::Impl {
 public:
  Impl(std::string auth_token,
       std::string position_bind_address,
       std::string position_auth_token)
      : auth_token_(std::move(auth_token)),
        position_bind_address_(std::move(position_bind_address)),
        position_auth_token_(std::move(position_auth_token)) {}

  bool PrepareRemoteRuntime(const BackendSplitConfig& split_config, std::vector<ReceiverStatus>* receivers,
                            std::string* error) {
    if (!IsRemoteBackendMode(split_config.mode)) {
      return true;
    }
    if (split_config.remote_dsp_host.empty()) {
      if (error != nullptr) {
        *error = "remote backend mode requires MR_REMOTE_DSP_HOST to point at the thin radio host";
      }
      return false;
    }
    remote_client_ = std::make_unique<RemoteRadioClient>(split_config.remote_dsp_host, auth_token_);
    if (!remote_client_->ListReceivers(receivers, error)) {
      return false;
    }
    return true;
  }

  RemoteRadioClient* remote_client() const { return remote_client_.get(); }

  bool Run(ReceiverManager* receiver_manager, PluginHost* plugin_host, EventBus* event_bus,
           TargetTracker* target_tracker, TrackDatabase* track_db,
           const std::string& bind_address, std::string* error) {
    radio_control_service_ =
        std::make_unique<RadioControlServiceImpl>(receiver_manager, plugin_host, track_db, auth_token_,
                                                  remote_client_.get());
    telemetry_service_ = std::make_unique<TelemetryServiceImpl>(event_bus, target_tracker, auth_token_);

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

    if (!position_bind_address_.empty()) {
      position_service_ = std::make_unique<PositionServiceImpl>(target_tracker, position_auth_token_);
      grpc::ServerBuilder pos_builder;
      pos_builder.AddListeningPort(position_bind_address_, grpc::InsecureServerCredentials());
      pos_builder.RegisterService(position_service_.get());
      // Keepalive: håll HTTP/2-anslutningen vid liv genom NAT/proxies.
      // Skicka PING var 30:e s; klienten måste svara inom 10 s.
      pos_builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS,                        30'000);
      pos_builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS,                     10'000);
      pos_builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS,                1);
      pos_builder.AddChannelArgument(GRPC_ARG_HTTP2_MAX_PINGS_WITHOUT_DATA,                  0);
      pos_builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS,
                                     10'000);
      position_server_ = pos_builder.BuildAndStart();
      if (position_server_) {
        std::cout << "Position-only gRPC endpoint on " << position_bind_address_
                  << (position_auth_token_.empty() ? " (no auth)" : " (auth enabled)") << "\n";
      } else {
        std::cerr << "Warning: failed to start position gRPC server on "
                  << position_bind_address_ << "\n";
      }
    }

    if (remote_client_ != nullptr) {
      remote_client_->StartIqRelay(receiver_manager, event_bus);
    }

    server_->Wait();
    return true;
  }

  void Shutdown() {
    if (remote_client_) remote_client_->StopIqRelay();
    if (position_server_) position_server_->Shutdown();
    if (server_) server_->Shutdown();
  }

 private:
  std::string auth_token_;
  std::string position_bind_address_;
  std::string position_auth_token_;
  std::unique_ptr<RadioControlServiceImpl> radio_control_service_;
  std::unique_ptr<TelemetryServiceImpl>    telemetry_service_;
  std::unique_ptr<PositionServiceImpl>     position_service_;
  std::unique_ptr<grpc::Server>            server_;
  std::unique_ptr<grpc::Server>            position_server_;
  std::unique_ptr<RemoteRadioClient>       remote_client_;
};

ServerApp::ServerApp(ServerConfig config)
    : config_(std::move(config)),
      event_bus_(std::make_shared<EventBus>()),
      logger_(std::make_shared<JsonlLogger>(config_.log_dir, "radio_events", config_.log_max_bytes,
                                            config_.log_max_files)),
      plugin_host_(std::make_shared<PluginHost>(config_.plugin_dir, config_.log_dir / "plugin_state")),
      track_db_(std::make_shared<TrackDatabase>(config_.log_dir / "track.db")),
      target_tracker_(std::make_shared<TargetTracker>(60000, 600000, track_db_)) {}

ServerApp::~ServerApp() { Shutdown(); }

bool ServerApp::Init(std::string* error) {
  impl_ = std::make_unique<Impl>(config_.auth_token,
                                 config_.position_bind_address,
                                 config_.position_auth_token);
  std::string plugin_error;
  plugin_host_->LoadAll(&plugin_error);

  std::vector<ReceiverStatus> remote_receivers;
  if (!impl_->PrepareRemoteRuntime(config_.split, &remote_receivers, error)) {
    return false;
  }

  std::vector<ReceiverDescriptor> descriptors;
  descriptors.reserve(remote_receivers.size());
  for (const auto& receiver : remote_receivers) {
    descriptors.push_back(ReceiverDescriptor{.receiver_id = receiver.receiver_id, .serial = receiver.serial});
  }

  const bool remote_mode = IsRemoteBackendMode(config_.split.mode);
  std::unique_ptr<IRadioDeviceFactory> factory = remote_mode ? nullptr : CreateRtlSdrFactory();
  receiver_manager_ = std::make_unique<ReceiverManager>(
      std::move(factory), event_bus_, plugin_host_, logger_, track_db_, target_tracker_,
      std::move(descriptors), remote_mode);

  if (remote_mode) {
    for (const auto& receiver : remote_receivers) {
      std::string sync_error;
      if (!receiver_manager_->SetMode(receiver.receiver_id, receiver.mode, &sync_error)) {
        if (error != nullptr) *error = sync_error;
        return false;
      }
      if (!receiver_manager_->SetModeConfig(receiver.receiver_id, receiver.mode_config, &sync_error)) {
        if (error != nullptr) *error = sync_error;
        return false;
      }
      if (receiver.running &&
          !receiver_manager_->StartReceiver(receiver.receiver_id, &sync_error)) {
        if (error != nullptr) *error = sync_error;
        return false;
      }
    }
  }

  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool ServerApp::Run(std::string* error) {
  if (!receiver_manager_ || !impl_) {
    if (error != nullptr) {
      *error = "ServerApp::Init must be called before Run";
    }
    return false;
  }
  return impl_->Run(receiver_manager_.get(), plugin_host_.get(), event_bus_.get(),
                    target_tracker_.get(), track_db_.get(), config_.bind_address, error);
}

void ServerApp::Shutdown() {
  if (impl_) {
    impl_->Shutdown();
  }
}

}  // namespace multi_radio
