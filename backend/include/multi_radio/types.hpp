#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace multi_radio {

enum class RadioMode {
  kFixed,
  kScanRange,
  kScanList,
  kAirMarinePlot,
};

enum class SignalType {
  kUnknown,
  kAis,
  kAdsb,
  kDsc,
};

enum class EventKind {
  kInfo,
  kWarning,
  kError,
  kStateChange,
  kTuneHop,
};

struct ModeConfig {
  double fixed_frequency_hz = 0.0;
  double range_start_hz = 0.0;
  double range_end_hz = 0.0;
  double range_step_hz = 0.0;
  std::vector<double> frequency_list_hz;
  uint32_t dwell_ms = 500;
  uint32_t sample_rate_hz = 2048000;
  uint32_t channel_bandwidth_hz = 30000;
  uint32_t hardware_bandwidth_hz = 0;
  bool ais_autotune_enabled = false;
  bool ais_baud_trim_enabled = false;
  bool dc_blocker_enabled = false;
  uint32_t dc_blocker_cutoff_hz = 30;
  bool center_notch_enabled = false;
  uint32_t center_notch_width_hz = 2000;
  bool lo_offset_enabled = false;
  int32_t lo_offset_hz = 0;
};

struct ReceiverDescriptor {
  uint32_t receiver_id = 0;
  std::string serial;
};

struct ReceiverStatus {
  uint32_t receiver_id = 0;
  std::string serial;
  bool running = false;
  RadioMode mode = RadioMode::kFixed;
  ModeConfig mode_config;
  std::string last_error;
};

struct ReceiverEvent {
  uint64_t unix_ms = 0;
  uint32_t receiver_id = 0;
  EventKind kind = EventKind::kInfo;
  std::string message;
  double tuned_frequency_hz = 0.0;
};

struct DecodedMessage {
  uint64_t unix_ms = 0;
  uint32_t receiver_id = 0;
  SignalType signal_type = SignalType::kUnknown;
  double frequency_hz = 0.0;
  std::string payload;
  std::map<std::string, std::string> normalized_fields;
};

struct IQSampleBlock {
  std::vector<int16_t> interleaved_iq;
  uint32_t sample_rate_hz = 0;
  uint32_t center_frequency_hz = 0;
  bool ais_autotune_enabled = false;
  bool ais_baud_trim_enabled = false;
};

struct PluginInfo {
  std::string plugin_name;
  std::string plugin_version;
  uint32_t api_version = 0;
  std::vector<SignalType> supported_signals;
  bool enabled = true;
  std::string path;
};

uint64_t UnixMillisNow();

std::string ToString(RadioMode mode);
std::string ToString(SignalType signal_type);
SignalType SignalTypeFromString(const std::string& value);

}  // namespace multi_radio
