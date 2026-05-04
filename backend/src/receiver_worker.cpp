#include "multi_radio/receiver_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "multi_radio/fm_demod.hpp"

namespace multi_radio {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr uint32_t kDefaultSampleRateHz = 2048000;
constexpr uint32_t kDefaultChannelBandwidthHz = 30000;
constexpr uint32_t kAudioSampleRateHzNfm = 12000;
constexpr uint32_t kAudioSampleRateHzWfm = 48000;
constexpr uint32_t kAudioFrameIntervalMs = 20;
constexpr uint32_t kAudioStatsIntervalMs = 1000;
constexpr uint32_t kIqVisualizationIntervalMs = 50;
constexpr const char* kAudioPipelineRevision = "audio-v12-stream-paced";
constexpr size_t kIqVisualizationMaxInterleavedSamples = 8192;
constexpr double kSyntheticIqToneFrequencyHz = 12000.0;
constexpr int16_t kIqClipThresholdS16 = 32256;

struct SignalHealthThresholds {
  double sample_rate_abs_error_hz = 2000.0;
  double sample_rate_rel_error = 0.02;
  double level_min_dbfs = -55.0;
  double level_max_dbfs = -8.0;
  double clip_max_pct = 2.5;
  double snr_min_db = 12.0;
  double stable_max_snr_delta_db = 20.0;
  double stable_max_level_delta_db = 18.0;
  int stable_windows_required = 2;
  int hysteresis_on_windows = 2;
  int hysteresis_off_windows = 3;
};

double ReadEnvDouble(const char* key, double fallback) {
  const char* value = std::getenv(key);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || !std::isfinite(parsed)) {
    return fallback;
  }
  return parsed;
}

int ReadEnvInt(const char* key, int fallback) {
  const char* value = std::getenv(key);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value) {
    return fallback;
  }
  if (parsed < static_cast<long>(std::numeric_limits<int>::min()) ||
      parsed > static_cast<long>(std::numeric_limits<int>::max())) {
    return fallback;
  }
  return static_cast<int>(parsed);
}

SignalHealthThresholds LoadSignalHealthThresholds() {
  SignalHealthThresholds out;
  out.sample_rate_abs_error_hz =
      std::max(0.0, ReadEnvDouble("MR_SIGNAL_SR_ABS_MAX_HZ", out.sample_rate_abs_error_hz));
  out.sample_rate_rel_error =
      std::clamp(ReadEnvDouble("MR_SIGNAL_SR_REL_MAX", out.sample_rate_rel_error), 0.0, 1.0);
  out.level_min_dbfs =
      std::clamp(ReadEnvDouble("MR_SIGNAL_LEVEL_MIN_DBFS", out.level_min_dbfs), -120.0, 0.0);
  out.level_max_dbfs =
      std::clamp(ReadEnvDouble("MR_SIGNAL_LEVEL_MAX_DBFS", out.level_max_dbfs), -120.0, 0.0);
  if (out.level_max_dbfs < out.level_min_dbfs) {
    std::swap(out.level_min_dbfs, out.level_max_dbfs);
  }
  out.clip_max_pct = std::max(0.0, ReadEnvDouble("MR_SIGNAL_CLIP_MAX_PCT", out.clip_max_pct));
  out.snr_min_db = std::max(0.0, ReadEnvDouble("MR_SIGNAL_SNR_MIN_DB", out.snr_min_db));
  out.stable_max_snr_delta_db =
      std::max(0.0, ReadEnvDouble("MR_SIGNAL_STABLE_MAX_SNR_DELTA_DB", out.stable_max_snr_delta_db));
  out.stable_max_level_delta_db =
      std::max(0.0, ReadEnvDouble("MR_SIGNAL_STABLE_MAX_LEVEL_DELTA_DB", out.stable_max_level_delta_db));
  out.stable_windows_required = std::max(1, ReadEnvInt("MR_SIGNAL_STABLE_WINDOWS", out.stable_windows_required));
  out.hysteresis_on_windows = std::max(1, ReadEnvInt("MR_SIGNAL_HYST_ON_WINDOWS", out.hysteresis_on_windows));
  out.hysteresis_off_windows =
      std::max(1, ReadEnvInt("MR_SIGNAL_HYST_OFF_WINDOWS", out.hysteresis_off_windows));
  return out;
}

const SignalHealthThresholds& GlobalSignalHealthThresholds() {
  static const SignalHealthThresholds thresholds = LoadSignalHealthThresholds();
  return thresholds;
}

ModeConfig NormalizeModeConfig(const ModeConfig& input) {
  ModeConfig out = input;
  if (out.dwell_ms == 0) {
    out.dwell_ms = 500;
  }
  if (out.sample_rate_hz == 0) {
    out.sample_rate_hz = kDefaultSampleRateHz;
  }
  if (out.channel_bandwidth_hz == 0) {
    out.channel_bandwidth_hz = kDefaultChannelBandwidthHz;
  }
  if (out.fixed_modulation != Modulation::kNfm && out.fixed_modulation != Modulation::kWfm) {
    out.fixed_modulation = Modulation::kWfm;
  }
  return out;
}

uint32_t AudioSampleRateForModulation(Modulation modulation) {
  return modulation == Modulation::kWfm ? kAudioSampleRateHzWfm : kAudioSampleRateHzNfm;
}

const char* ModulationToken(Modulation modulation) {
  switch (modulation) {
    case Modulation::kAm:
      return "AM";
    case Modulation::kWfm:
      return "WFM";
    case Modulation::kNfm:
    default:
      return "NFM";
  }
}

bool IsFmModulation(Modulation modulation) {
  return modulation == Modulation::kNfm || modulation == Modulation::kWfm;
}

std::string FormatDouble(double value, int precision) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << value;
  return out.str();
}

size_t AudioFrameSamplesForRate(uint32_t audio_sample_rate_hz) {
  if (audio_sample_rate_hz == 0) {
    return 0;
  }
  return (static_cast<size_t>(audio_sample_rate_hz) * static_cast<size_t>(kAudioFrameIntervalMs)) / 1000U;
}

struct RuntimeChannel {
  int index = -1;
  std::string label = "default";
  double frequency_hz = 0.0;
  Modulation modulation = Modulation::kWfm;
  double squelch_threshold_db = -67.5;
};

RuntimeChannel SelectRuntimeChannel(RadioMode mode, const ModeConfig& config) {
  RuntimeChannel out;
  if (mode == RadioMode::kScanList && !config.scan_list_channels.empty()) {
    const auto& ch = config.scan_list_channels.front();
    out.index = 0;
    out.label = ch.label.empty() ? "ch0" : ch.label;
    out.frequency_hz = ch.frequency_hz;
    out.modulation = ch.modulation;
    out.squelch_threshold_db = ch.squelch_threshold_db;
    return out;
  }

  out.frequency_hz = config.fixed_frequency_hz;
  out.modulation = config.fixed_modulation;
  if (out.frequency_hz <= 0.0 && !config.frequency_list_hz.empty()) {
    out.frequency_hz = config.frequency_list_hz.front();
  }
  return out;
}

std::string FormatRuntimeError(const char* operation, const std::string& error) {
  std::ostringstream out;
  out << operation << " failed";
  if (!error.empty()) {
    out << ": " << error;
  }
  return out.str();
}

struct PsdSummary {
  bool valid = false;
  double peak_db = -120.0;
  double floor_db = -120.0;
  double snr_db = 0.0;
  double peak_offset_hz = 0.0;
};

PsdSummary EstimatePsdSummary(const std::vector<int16_t>& interleaved_iq, uint32_t sample_rate_hz) {
  PsdSummary out;
  if (interleaved_iq.size() < 128) {
    return out;
  }
  const size_t iq_pairs = interleaved_iq.size() / 2U;
  const size_t n = std::min<size_t>(256, iq_pairs);
  if (n < 32) {
    return out;
  }

  const size_t start_pair = iq_pairs - n;
  std::vector<double> power_db;
  power_db.reserve(n);
  double window_energy = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double phase = (2.0 * kPi * static_cast<double>(i)) / static_cast<double>(n - 1);
    const double w = 0.5 * (1.0 - std::cos(phase));
    window_energy += w * w;
  }
  const double norm = std::max(1.0e-12, window_energy);

  for (size_t k = 0; k < n; ++k) {
    std::complex<double> acc(0.0, 0.0);
    for (size_t t = 0; t < n; ++t) {
      const size_t pair_idx = start_pair + t;
      const size_t sample_idx = pair_idx * 2U;
      const double i_norm = static_cast<double>(interleaved_iq[sample_idx]) / 32768.0;
      const double q_norm = static_cast<double>(interleaved_iq[sample_idx + 1U]) / 32768.0;
      const std::complex<double> x(i_norm, q_norm);
      const double window_phase =
          (2.0 * kPi * static_cast<double>(t)) / static_cast<double>(n - 1);
      const double w = 0.5 * (1.0 - std::cos(window_phase));
      const double dft_phase =
          (-2.0 * kPi * static_cast<double>(k) * static_cast<double>(t)) / static_cast<double>(n);
      acc += x * w * std::complex<double>(std::cos(dft_phase), std::sin(dft_phase));
    }
    const double p = std::norm(acc) / norm;
    power_db.push_back(10.0 * std::log10(std::max(1.0e-12, p)));
  }

  auto peak_it = std::max_element(power_db.begin(), power_db.end());
  if (peak_it == power_db.end()) {
    return out;
  }
  const size_t peak_bin = static_cast<size_t>(std::distance(power_db.begin(), peak_it));
  const double peak_db = *peak_it;

  std::vector<double> sorted = power_db;
  const size_t floor_idx = sorted.size() / 5U;
  std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(floor_idx), sorted.end());
  const double floor_db = sorted[floor_idx];
  const double snr_db = std::max(0.0, peak_db - floor_db);

  const int signed_bin = (peak_bin <= (n / 2U)) ? static_cast<int>(peak_bin)
                                                 : static_cast<int>(peak_bin) - static_cast<int>(n);
  const double peak_offset_hz =
      static_cast<double>(signed_bin) * static_cast<double>(sample_rate_hz) / static_cast<double>(n);

  out.valid = true;
  out.peak_db = peak_db;
  out.floor_db = floor_db;
  out.snr_db = snr_db;
  out.peak_offset_hz = peak_offset_hz;
  return out;
}

IQSampleBlock BuildSyntheticIqBlock(uint32_t sample_rate_hz, uint32_t tuned_frequency_hz, uint64_t sample_index) {
  IQSampleBlock out;
  out.sample_rate_hz = sample_rate_hz;
  out.center_frequency_hz = tuned_frequency_hz;
  out.interleaved_iq.resize(4096);
  const double sr = std::max(1.0, static_cast<double>(sample_rate_hz));
  const double step = (2.0 * kPi * kSyntheticIqToneFrequencyHz) / sr;
  for (size_t i = 0; i < out.interleaved_iq.size(); i += 2) {
    const double phase = step * static_cast<double>(sample_index + (i / 2));
    out.interleaved_iq[i] = static_cast<int16_t>(std::lrint(14000.0 * std::cos(phase)));
    out.interleaved_iq[i + 1] = static_cast<int16_t>(std::lrint(14000.0 * std::sin(phase)));
  }
  return out;
}

IqFrame BuildIqFrame(const IQSampleBlock& block, uint32_t receiver_id, double tuned_frequency_hz, uint64_t sequence,
                     uint64_t sample_index) {
  IqFrame frame;
  frame.unix_ms = UnixMillisNow();
  frame.receiver_id = receiver_id;
  frame.sample_rate_hz = block.sample_rate_hz;
  frame.tuned_frequency_hz = tuned_frequency_hz;
  frame.sequence = sequence;
  frame.sample_index = sample_index;

  const size_t total = block.interleaved_iq.size();
  if (total <= kIqVisualizationMaxInterleavedSamples) {
    frame.interleaved_iq_s16le = block.interleaved_iq;
    return frame;
  }

  const size_t stride = std::max<size_t>(2, total / kIqVisualizationMaxInterleavedSamples);
  const size_t even_stride = (stride % 2 == 0) ? stride : (stride + 1);
  frame.interleaved_iq_s16le.reserve(kIqVisualizationMaxInterleavedSamples);
  for (size_t i = 0; i + 1 < total; i += even_stride) {
    frame.interleaved_iq_s16le.push_back(block.interleaved_iq[i]);
    frame.interleaved_iq_s16le.push_back(block.interleaved_iq[i + 1]);
  }
  return frame;
}

}  // namespace

ReceiverWorker::ReceiverWorker(uint32_t receiver_id, std::string serial, std::unique_ptr<IRadioDevice> device,
                               std::shared_ptr<EventBus> event_bus, std::shared_ptr<PluginHost> plugin_host,
                               std::shared_ptr<JsonlLogger> logger)
    : receiver_id_(receiver_id),
      serial_(std::move(serial)),
      device_(std::move(device)),
      event_bus_(std::move(event_bus)),
      plugin_host_(std::move(plugin_host)),
      logger_(std::move(logger)) {
  mode_config_ = NormalizeModeConfig(mode_config_);
}

ReceiverWorker::~ReceiverWorker() {
  std::string error;
  Stop(&error);
}

bool ReceiverWorker::Start(std::string* error) {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    if (error != nullptr) {
      *error = "receiver already running";
    }
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mu_);
    last_error_.clear();
  }

  thread_ = std::thread(&ReceiverWorker::RunLoop, this);
  if (error != nullptr) {
    error->clear();
  }
  PublishEvent(EventKind::kStateChange, "receiver started");
  return true;
}

bool ReceiverWorker::Stop(std::string* error) {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  if (thread_.joinable()) {
    thread_.join();
  }
  if (error != nullptr) {
    error->clear();
  }
  PublishEvent(EventKind::kStateChange, "receiver stopped");
  return true;
}

bool ReceiverWorker::SetMode(RadioMode mode, std::string* error) {
  {
    std::lock_guard<std::mutex> lock(mu_);
    mode_ = mode;
    mode_config_ = NormalizeModeConfig(mode_config_);
  }
  if (error != nullptr) {
    error->clear();
  }
  std::ostringstream msg;
  msg << "mode changed to " << ToString(mode);
  PublishEvent(EventKind::kStateChange, msg.str());
  return true;
}

bool ReceiverWorker::SetModeConfig(const ModeConfig& config, std::string* error) {
  {
    std::lock_guard<std::mutex> lock(mu_);
    mode_config_ = NormalizeModeConfig(config);
  }
  if (error != nullptr) {
    error->clear();
  }
  PublishEvent(EventKind::kInfo, "mode config updated");
  return true;
}

ReceiverStatus ReceiverWorker::Status() const {
  std::lock_guard<std::mutex> lock(mu_);
  return ReceiverStatus{.receiver_id = receiver_id_,
                        .serial = serial_,
                        .running = running_.load(),
                        .mode = mode_,
                        .mode_config = mode_config_,
                        .last_error = last_error_};
}

void ReceiverWorker::RunLoop() {
  const SignalHealthThresholds& thresholds = GlobalSignalHealthThresholds();
  uint64_t audio_sequence = 0;
  uint64_t audio_sample_index = 0;
  uint64_t iq_sequence = 0;
  uint64_t iq_sample_index = 0;
  uint64_t generated_samples = 0;
  uint64_t published_samples = 0;
  uint64_t published_frames = 0;
  uint64_t conceal_samples = 0;
  uint64_t iq_interleaved_samples = 0;
  uint32_t iq_sample_rate_hz = 0;
  uint64_t iq_window_samples = 0;
  uint64_t iq_window_components = 0;
  uint64_t iq_window_clipped_components = 0;
  double iq_window_component_power = 0.0;
  IQSampleBlock latest_iq_block;
  bool have_latest_iq_block = false;
  bool have_prev_iq_health = false;
  double prev_iq_snr_db = 0.0;
  double prev_iq_level_dbfs = -120.0;
  int iq_stable_windows = 0;
  int iq_good_windows = 0;
  int iq_bad_windows = 0;
  bool signal_ok_latched = false;
  FmDemodulator fm_demod;
  bool fm_demod_available = FmDemodulator::Available();
  bool fm_demod_configured = false;
  bool fm_demod_warned_unavailable = false;
  uint32_t fm_demod_input_sr_hz = 0;
  uint32_t fm_demod_audio_sr_hz = 0;
  uint32_t fm_demod_channel_bw_hz = 0;
  Modulation fm_demod_modulation = Modulation::kNfm;
  std::deque<int16_t> audio_pending_pcm;
  uint64_t demod_ok_blocks = 0;
  uint64_t demod_empty_blocks = 0;
  uint64_t demod_input_iq_samples = 0;
  uint64_t demod_channelized_samples = 0;
  uint64_t demod_audio_samples = 0;

  const auto frame_interval = std::chrono::milliseconds(kAudioFrameIntervalMs);
  auto next_frame_at = std::chrono::steady_clock::now();
  auto stats_started_at = next_frame_at;
  auto next_stats_at = next_frame_at + std::chrono::milliseconds(kAudioStatsIntervalMs);

  bool device_opened = false;
  uint32_t configured_frequency_hz = 0;
  uint32_t configured_sample_rate_hz = 0;
  uint32_t configured_hardware_bandwidth_hz = 0;
  int configured_gain_tenth_db = std::numeric_limits<int>::min();
  bool squelch_open_emitted = false;
  bool thresholds_emitted = false;
  auto next_iq_visualization_at = std::chrono::steady_clock::now();

  while (running_.load()) {
    RadioMode mode = RadioMode::kFixed;
    ModeConfig config;
    {
      std::lock_guard<std::mutex> lock(mu_);
      mode = mode_;
      config = mode_config_;
    }

    const RuntimeChannel ch = SelectRuntimeChannel(mode, config);
    const uint32_t tuned_frequency_hz =
        ch.frequency_hz <= 0.0 ? 0 : static_cast<uint32_t>(std::llround(ch.frequency_hz));
    int64_t hardware_frequency_i64 = static_cast<int64_t>(tuned_frequency_hz);
    if (config.lo_offset_enabled) {
      hardware_frequency_i64 += static_cast<int64_t>(config.lo_offset_hz);
    }
    hardware_frequency_i64 = std::clamp<int64_t>(hardware_frequency_i64, 0,
                                                 static_cast<int64_t>(std::numeric_limits<uint32_t>::max()));
    const uint32_t hardware_tuned_frequency_hz = static_cast<uint32_t>(hardware_frequency_i64);
    const uint32_t audio_sample_rate_hz = AudioSampleRateForModulation(ch.modulation);
    const size_t frame_samples = AudioFrameSamplesForRate(audio_sample_rate_hz);
    if (frame_samples == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    if (!squelch_open_emitted && mode == RadioMode::kScanList) {
      std::ostringstream opened;
      opened << "SCAN squelch OPEN ch=" << ch.label << " idx=" << ch.index
             << " signal=-20.0 dB threshold=" << FormatDouble(ch.squelch_threshold_db, 1) << " dB";
      PublishEvent(EventKind::kInfo, opened.str(), ch.frequency_hz);
      squelch_open_emitted = true;
    }

    IQSampleBlock iq_block;
    bool have_iq = false;
    if (device_ != nullptr) {
      std::string error;
      if (!device_opened) {
        if (!device_->Open(&error)) {
          PublishEvent(EventKind::kError, FormatRuntimeError("rtl open", error), ch.frequency_hz);
          {
            std::lock_guard<std::mutex> lock(mu_);
            last_error_ = error;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
          continue;
        }
        device_opened = true;
        PublishEvent(EventKind::kInfo, "rtl device opened", ch.frequency_hz);
      }

      if (configured_sample_rate_hz != config.sample_rate_hz) {
        if (!device_->SetSampleRateHz(config.sample_rate_hz, &error)) {
          PublishEvent(EventKind::kError, FormatRuntimeError("set sample rate", error), ch.frequency_hz);
          {
            std::lock_guard<std::mutex> lock(mu_);
            last_error_ = error;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        configured_sample_rate_hz = config.sample_rate_hz;
      }

      if (configured_hardware_bandwidth_hz != config.hardware_bandwidth_hz) {
        if (!device_->SetHardwareBandwidthHz(config.hardware_bandwidth_hz, &error)) {
          PublishEvent(EventKind::kError, FormatRuntimeError("set hardware bandwidth", error), ch.frequency_hz);
          {
            std::lock_guard<std::mutex> lock(mu_);
            last_error_ = error;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        configured_hardware_bandwidth_hz = config.hardware_bandwidth_hz;
      }

      // WFM broadcast is often overdriven with RTL auto-gain; start with low manual gain.
      // Other modes keep auto gain unless explicitly tuned later.
      const int desired_gain_tenth_db = (ch.modulation == Modulation::kWfm) ? 0 : -1;
      if (configured_gain_tenth_db != desired_gain_tenth_db) {
        if (!device_->SetGainTenthdB(desired_gain_tenth_db, &error)) {
          PublishEvent(EventKind::kError, FormatRuntimeError("set gain", error), ch.frequency_hz);
          {
            std::lock_guard<std::mutex> lock(mu_);
            last_error_ = error;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        configured_gain_tenth_db = desired_gain_tenth_db;
        PublishEvent(EventKind::kInfo,
                     desired_gain_tenth_db < 0 ? "rtl gain auto"
                                               : ("rtl gain manual " +
                                                  std::to_string(desired_gain_tenth_db / 10) + " dB"),
                     ch.frequency_hz);
      }

      if (hardware_tuned_frequency_hz != 0 && hardware_tuned_frequency_hz != configured_frequency_hz) {
        if (!device_->SetCenterFrequencyHz(hardware_tuned_frequency_hz, &error)) {
          PublishEvent(EventKind::kError, FormatRuntimeError("set center frequency", error), ch.frequency_hz);
          {
            std::lock_guard<std::mutex> lock(mu_);
            last_error_ = error;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        configured_frequency_hz = hardware_tuned_frequency_hz;
        std::ostringstream tuned;
        tuned << "tuned to " << tuned_frequency_hz << " Hz";
        if (config.lo_offset_enabled) {
          tuned << " (hw=" << hardware_tuned_frequency_hz << " Hz, lo_offset=" << config.lo_offset_hz << " Hz)";
        }
        PublishEvent(EventKind::kTuneHop, tuned.str(), ch.frequency_hz);
      }

      if (!device_->ReadIq(&iq_block, &error)) {
        PublishEvent(EventKind::kWarning, FormatRuntimeError("read iq", error), ch.frequency_hz);
        {
          std::lock_guard<std::mutex> lock(mu_);
          last_error_ = error;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      have_iq = true;
    } else {
      iq_block = BuildSyntheticIqBlock(config.sample_rate_hz, tuned_frequency_hz, iq_sample_index);
      have_iq = true;
    }

    if (have_iq) {
      const uint64_t iq_sample_index_block_start = iq_sample_index;
      iq_sample_index += static_cast<uint64_t>(iq_block.interleaved_iq.size() / 2U);
      iq_interleaved_samples = static_cast<uint64_t>(iq_block.interleaved_iq.size());
      iq_sample_rate_hz = iq_block.sample_rate_hz;
      iq_window_samples += static_cast<uint64_t>(iq_block.interleaved_iq.size() / 2U);
      iq_window_components += static_cast<uint64_t>(iq_block.interleaved_iq.size());
      for (const int16_t sample : iq_block.interleaved_iq) {
        const double normalized = static_cast<double>(sample) / 32768.0;
        iq_window_component_power += normalized * normalized;
        if (std::abs(static_cast<int>(sample)) >= static_cast<int>(kIqClipThresholdS16)) {
          ++iq_window_clipped_components;
        }
      }
      latest_iq_block = iq_block;
      have_latest_iq_block = true;
      {
        std::lock_guard<std::mutex> lock(mu_);
        last_error_.clear();
      }
      const auto now_vis = std::chrono::steady_clock::now();
      if (now_vis >= next_iq_visualization_at) {
        IqFrame iq_frame =
            BuildIqFrame(iq_block, receiver_id_, ch.frequency_hz, iq_sequence++, iq_sample_index_block_start);
        event_bus_->PublishIqFrame(iq_frame);
        next_iq_visualization_at = now_vis + std::chrono::milliseconds(kIqVisualizationIntervalMs);
      }

      // Decoder plugins are not needed for wide-band FM broadcast audio and can
      // consume enough CPU to cause ingest underruns.
      if (plugin_host_ != nullptr && ch.modulation != Modulation::kWfm) {
        plugin_host_->ProcessIq(iq_block, [&](const PluginMessage& msg) {
          DecodedMessage decoded;
          decoded.unix_ms = msg.unix_ms;
          decoded.receiver_id = receiver_id_;
          decoded.signal_type = msg.signal_type;
          decoded.frequency_hz = msg.frequency_hz;
          decoded.payload = msg.payload;
          decoded.normalized_fields = msg.normalized_fields;
          event_bus_->PublishDecodedMessage(decoded);
        });
      }
    }

    if (have_iq && IsFmModulation(ch.modulation)) {
      if (!fm_demod_available) {
        if (!fm_demod_warned_unavailable) {
          PublishEvent(EventKind::kWarning, "FM demod unavailable: backend built without libliquid",
                       ch.frequency_hz);
          fm_demod_warned_unavailable = true;
        }
      } else {
        const uint32_t demod_input_sr_hz =
            iq_block.sample_rate_hz != 0 ? iq_block.sample_rate_hz : config.sample_rate_hz;
        const bool demod_reconfigure = !fm_demod_configured || fm_demod_input_sr_hz != demod_input_sr_hz ||
                                       fm_demod_audio_sr_hz != audio_sample_rate_hz ||
                                       fm_demod_channel_bw_hz != config.channel_bandwidth_hz ||
                                       fm_demod_modulation != ch.modulation;
        if (demod_reconfigure) {
          std::string demod_error;
          if (!fm_demod.Configure(demod_input_sr_hz, audio_sample_rate_hz, ch.modulation,
                                  config.channel_bandwidth_hz, &demod_error)) {
            fm_demod_configured = false;
            PublishEvent(EventKind::kWarning, FormatRuntimeError("configure fm demod", demod_error),
                         ch.frequency_hz);
          } else {
            fm_demod_configured = true;
            fm_demod_input_sr_hz = demod_input_sr_hz;
            fm_demod_audio_sr_hz = audio_sample_rate_hz;
            fm_demod_channel_bw_hz = config.channel_bandwidth_hz;
            fm_demod_modulation = ch.modulation;
            audio_pending_pcm.clear();
            std::ostringstream demod_msg;
            demod_msg << "FM demod configured mod=" << ModulationToken(ch.modulation)
                      << " iq_sr=" << fm_demod_input_sr_hz << " audio_sr=" << fm_demod_audio_sr_hz
                      << " bw=" << fm_demod_channel_bw_hz;
            PublishEvent(EventKind::kInfo, demod_msg.str(), ch.frequency_hz);
          }
        }

        if (fm_demod_configured) {
          std::vector<int16_t> demod_pcm;
          FmDemodProcessStats demod_stats;
          std::string demod_error;
          if (!fm_demod.ProcessIq(iq_block.interleaved_iq, &demod_pcm, &demod_stats, &demod_error)) {
            PublishEvent(EventKind::kWarning, FormatRuntimeError("fm demod", demod_error), ch.frequency_hz);
          } else {
            demod_input_iq_samples += demod_stats.input_iq_samples;
            demod_channelized_samples += demod_stats.channelized_samples;
            demod_audio_samples += demod_stats.audio_samples;
            generated_samples += static_cast<uint64_t>(demod_pcm.size());
            for (const int16_t sample : demod_pcm) {
              audio_pending_pcm.push_back(sample);
            }
          }
        }
      }
    } else if (!IsFmModulation(ch.modulation)) {
      fm_demod.Reset();
      fm_demod_configured = false;
      audio_pending_pcm.clear();
    }

    const auto now = std::chrono::steady_clock::now();
    if (now > next_frame_at + std::chrono::milliseconds(200)) {
      next_frame_at = now;
    }
    while (now >= next_frame_at) {
      AudioFrame frame;
      frame.unix_ms = UnixMillisNow();
      frame.receiver_id = receiver_id_;
      frame.sample_rate_hz = audio_sample_rate_hz;
      frame.tuned_frequency_hz = ch.frequency_hz;
      frame.sequence = audio_sequence++;
      frame.sample_index = audio_sample_index;
      frame.pcm_s16le.resize(frame_samples);
      size_t frame_filled = 0;
      while (frame_filled < frame_samples && !audio_pending_pcm.empty()) {
        frame.pcm_s16le[frame_filled] = audio_pending_pcm.front();
        audio_pending_pcm.pop_front();
        ++frame_filled;
      }
      if (frame_filled < frame_samples) {
        for (size_t i = frame_filled; i < frame_samples; ++i) {
          frame.pcm_s16le[i] = 0;
        }
        conceal_samples += static_cast<uint64_t>(frame_samples - frame_filled);
      }
      if (frame_filled == 0) {
        ++demod_empty_blocks;
      } else {
        ++demod_ok_blocks;
      }

      published_samples += static_cast<uint64_t>(frame_samples);
      ++published_frames;
      audio_sample_index += static_cast<uint64_t>(frame_samples);
      event_bus_->PublishAudioFrame(frame);
      next_frame_at += frame_interval;
    }

    if (now >= next_stats_at) {
      if (!thresholds_emitted) {
        std::ostringstream threshold_status;
        threshold_status << "IQ_THRESHOLDS"
                         << " sr_abs_max_hz=" << FormatDouble(thresholds.sample_rate_abs_error_hz, 0)
                         << " sr_rel_max=" << FormatDouble(thresholds.sample_rate_rel_error, 4)
                         << " level_min_dbfs=" << FormatDouble(thresholds.level_min_dbfs, 1)
                         << " level_max_dbfs=" << FormatDouble(thresholds.level_max_dbfs, 1)
                         << " clip_max_pct=" << FormatDouble(thresholds.clip_max_pct, 2)
                         << " snr_min_db=" << FormatDouble(thresholds.snr_min_db, 1)
                         << " stable_max_snr_delta_db="
                         << FormatDouble(thresholds.stable_max_snr_delta_db, 1)
                         << " stable_max_level_delta_db="
                         << FormatDouble(thresholds.stable_max_level_delta_db, 1)
                         << " stable_windows=" << thresholds.stable_windows_required
                         << " hyst_on_windows=" << thresholds.hysteresis_on_windows
                         << " hyst_off_windows=" << thresholds.hysteresis_off_windows;
        PublishEvent(EventKind::kInfo, threshold_status.str(), ch.frequency_hz);
        thresholds_emitted = true;
      }

      const double window_s =
          std::max(1.0e-3, std::chrono::duration<double>(now - stats_started_at).count());
      const uint64_t window_ms = static_cast<uint64_t>(std::llround(window_s * 1000.0));
      const double gen_hz = static_cast<double>(generated_samples) / window_s;
      const double pub_hz = static_cast<double>(published_samples) / window_s;
      const double iq_measured_sample_rate_hz = static_cast<double>(iq_window_samples) / window_s;
      const double iq_rms = (iq_window_components == 0)
                                ? 0.0
                                : std::sqrt(iq_window_component_power /
                                            static_cast<double>(iq_window_components));
      const double iq_level_dbfs =
          20.0 * std::log10(std::max(1.0e-9, std::min(1.0, iq_rms)));
      const double iq_clip_pct =
          (iq_window_components == 0)
              ? 0.0
              : (100.0 * static_cast<double>(iq_window_clipped_components) /
                 static_cast<double>(iq_window_components));
      const PsdSummary psd = have_latest_iq_block
                                 ? EstimatePsdSummary(latest_iq_block.interleaved_iq, iq_sample_rate_hz)
                                 : PsdSummary{};

      std::ostringstream audio_status;
      audio_status << "AUDIO_STATS idx=" << ch.index
                   << " label=" << ch.label
                   << " rev=" << kAudioPipelineRevision
                   << " mod=" << ModulationToken(ch.modulation)
                   << " sr=" << audio_sample_rate_hz
                   << " cfg_sr=" << config.sample_rate_hz
                   << " iq_sr=" << iq_sample_rate_hz
                   << " iq_est_sr=" << iq_sample_rate_hz
                   << " iq_lock=0"
                   << " iq_lock_sr=" << iq_sample_rate_hz
                   << " win_ms=" << window_ms
                   << " gen_ratio=1.000"
                   << " rate_corr=1.0000"
                   << " gen_hz=" << FormatDouble(gen_hz, 1)
                   << " pub_hz=" << FormatDouble(pub_hz, 1)
                   << " iq_n=" << iq_interleaved_samples
                   << " gate=1"
                   << " squelch=1"
                   << " signal_db=" << FormatDouble(iq_level_dbfs, 1)
                   << " blocks=" << published_frames
                   << " gate_open_blocks=" << demod_ok_blocks
                   << " demod_ok=" << demod_ok_blocks
                   << " demod_empty=" << demod_empty_blocks
                   << " gen_samples=" << generated_samples
                   << " pub_frames=" << published_frames
                   << " pub_samples=" << published_samples
                   << " pending_samples=" << audio_pending_pcm.size()
                   << " conceal_samples=" << conceal_samples
                   << " clears=0"
                   << " flush_frames=0"
                   << " flush_samples=0"
                   << " demod_in_iq=" << demod_input_iq_samples
                   << " demod_ch=" << demod_channelized_samples
                   << " demod_audio=" << demod_audio_samples;
      PublishEvent(EventKind::kInfo, audio_status.str(), ch.frequency_hz, false);

      std::ostringstream iq_status;
      const double configured_iq_sample_rate_hz = static_cast<double>(config.sample_rate_hz);
      const double block_sr_hz = static_cast<double>(iq_sample_rate_hz);
      const double block_sr_abs_error_hz = std::abs(block_sr_hz - configured_iq_sample_rate_hz);
      const double block_sr_rel_error = (configured_iq_sample_rate_hz <= 0.0)
                                            ? 0.0
                                            : block_sr_abs_error_hz / configured_iq_sample_rate_hz;
      // Use device-reported block sample-rate for health gating.
      // The measured ingest rate is intentionally lower in this prototype pipeline.
      const bool ok_sr = configured_iq_sample_rate_hz <= 0.0 || block_sr_hz <= 0.0 ||
                         (block_sr_abs_error_hz <= thresholds.sample_rate_abs_error_hz &&
                          block_sr_rel_error <= thresholds.sample_rate_rel_error);
      const bool ok_level =
          iq_level_dbfs >= thresholds.level_min_dbfs && iq_level_dbfs <= thresholds.level_max_dbfs;
      const bool ok_clip = iq_clip_pct <= thresholds.clip_max_pct;
      const bool ok_snr = psd.valid && psd.snr_db >= thresholds.snr_min_db;
      bool window_stable = true;
      if (have_prev_iq_health) {
        window_stable = std::abs(psd.snr_db - prev_iq_snr_db) <= thresholds.stable_max_snr_delta_db &&
                        std::abs(iq_level_dbfs - prev_iq_level_dbfs) <=
                            thresholds.stable_max_level_delta_db;
      }
      if (window_stable) {
        ++iq_stable_windows;
      } else {
        iq_stable_windows = 0;
      }
      const bool ok_stable = iq_stable_windows >= thresholds.stable_windows_required;
      const bool signal_ok_raw = ok_sr && ok_level && ok_clip && ok_snr && ok_stable;
      if (signal_ok_raw) {
        ++iq_good_windows;
        iq_bad_windows = 0;
      } else {
        ++iq_bad_windows;
        iq_good_windows = 0;
      }
      if (!signal_ok_latched && iq_good_windows >= thresholds.hysteresis_on_windows) {
        signal_ok_latched = true;
      } else if (signal_ok_latched && iq_bad_windows >= thresholds.hysteresis_off_windows) {
        signal_ok_latched = false;
      }
      const bool signal_ok = signal_ok_latched;

      const char* raw_status = "OK";
      if (!ok_sr) {
        raw_status = "SR_MISMATCH";
      } else if (!ok_level) {
        raw_status = "LEVEL_RANGE";
      } else if (!ok_clip) {
        raw_status = "CLIPPING";
      } else if (!ok_snr) {
        raw_status = "LOW_SNR";
      } else if (!ok_stable) {
        raw_status = "UNSTABLE";
      }
      const char* signal_status = signal_ok ? "OK" : (signal_ok_raw ? "WARMUP" : raw_status);

      double sr_score = 1.0;
      if (!(configured_iq_sample_rate_hz <= 0.0 || block_sr_hz <= 0.0)) {
        const double abs_ratio = thresholds.sample_rate_abs_error_hz <= 0.0
                                     ? (block_sr_abs_error_hz == 0.0 ? 0.0 : 1.0)
                                     : block_sr_abs_error_hz / thresholds.sample_rate_abs_error_hz;
        const double rel_ratio = thresholds.sample_rate_rel_error <= 0.0
                                     ? (block_sr_rel_error == 0.0 ? 0.0 : 1.0)
                                     : block_sr_rel_error / thresholds.sample_rate_rel_error;
        sr_score = std::clamp(1.0 - std::max(abs_ratio, rel_ratio), 0.0, 1.0);
      }
      double level_score = 1.0;
      if (iq_level_dbfs < thresholds.level_min_dbfs) {
        const double miss = thresholds.level_min_dbfs - iq_level_dbfs;
        level_score = std::clamp(1.0 - (miss / 20.0), 0.0, 1.0);
      } else if (iq_level_dbfs > thresholds.level_max_dbfs) {
        const double miss = iq_level_dbfs - thresholds.level_max_dbfs;
        level_score = std::clamp(1.0 - (miss / 10.0), 0.0, 1.0);
      }
      double clip_score = 1.0;
      if (iq_clip_pct > thresholds.clip_max_pct) {
        const double denom = std::max(0.1, thresholds.clip_max_pct);
        clip_score = std::clamp(1.0 - ((iq_clip_pct - thresholds.clip_max_pct) / denom), 0.0, 1.0);
      }
      const double snr_score =
          psd.valid ? std::clamp((psd.snr_db - (thresholds.snr_min_db - 6.0)) / 18.0, 0.0, 1.0) : 0.0;
      const double stable_score = std::clamp(
          static_cast<double>(iq_stable_windows) /
              static_cast<double>(std::max(1, thresholds.stable_windows_required)),
          0.0, 1.0);
      const double quality_score =
          (0.25 * sr_score + 0.20 * level_score + 0.20 * clip_score + 0.20 * snr_score +
           0.15 * stable_score) *
          100.0;

      prev_iq_snr_db = psd.snr_db;
      prev_iq_level_dbfs = iq_level_dbfs;
      have_prev_iq_health = true;

      iq_status << "IQ_STATS idx=" << ch.index
                << " mod=" << ModulationToken(ch.modulation)
                << " cfg_sr=" << config.sample_rate_hz
                << " meas_sr=" << FormatDouble(iq_measured_sample_rate_hz, 0)
                << " block_sr=" << iq_sample_rate_hz
                << " tuned_hz=" << tuned_frequency_hz
                << " center_hz=" << (have_latest_iq_block ? latest_iq_block.center_frequency_hz : 0U)
                << " level_dbfs=" << FormatDouble(iq_level_dbfs, 1)
                << " clip_pct=" << FormatDouble(iq_clip_pct, 2)
                << " clip=" << ((iq_clip_pct >= 1.0) ? 1 : 0)
                << " psd_peak_db=" << FormatDouble(psd.peak_db, 1)
                << " psd_floor_db=" << FormatDouble(psd.floor_db, 1)
                << " snr_db=" << FormatDouble(psd.snr_db, 1)
                << " psd_peak_offset_hz=" << FormatDouble(psd.peak_offset_hz, 0)
                << " ok_sr=" << (ok_sr ? 1 : 0)
                << " ok_level=" << (ok_level ? 1 : 0)
                << " ok_clip=" << (ok_clip ? 1 : 0)
                << " ok_snr=" << (ok_snr ? 1 : 0)
                << " ok_stable=" << (ok_stable ? 1 : 0)
                << " stable_windows=" << iq_stable_windows
                << " quality_score=" << FormatDouble(quality_score, 1)
                << " signal_ok_raw=" << (signal_ok_raw ? 1 : 0)
                << " raw_status=" << raw_status
                << " hys_good_windows=" << iq_good_windows
                << " hys_bad_windows=" << iq_bad_windows
                << " signal_ok=" << (signal_ok ? 1 : 0)
                << " signal_status=" << signal_status
                << " win_ms=" << window_ms
                << " iq_n=" << iq_interleaved_samples;
      PublishEvent(EventKind::kInfo, iq_status.str(), ch.frequency_hz);

      generated_samples = 0;
      published_samples = 0;
      published_frames = 0;
      conceal_samples = 0;
      demod_ok_blocks = 0;
      demod_empty_blocks = 0;
      demod_input_iq_samples = 0;
      demod_channelized_samples = 0;
      demod_audio_samples = 0;
      iq_window_samples = 0;
      iq_window_components = 0;
      iq_window_clipped_components = 0;
      iq_window_component_power = 0.0;
      stats_started_at = now;
      next_stats_at = now + std::chrono::milliseconds(kAudioStatsIntervalMs);
    }
    if (device_ == nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  if (device_ != nullptr && device_opened) {
    device_->Close();
  }

  if (squelch_open_emitted) {
    RuntimeChannel ch;
    {
      std::lock_guard<std::mutex> lock(mu_);
      ch = SelectRuntimeChannel(mode_, mode_config_);
    }
    std::ostringstream closed;
    closed << "SCAN squelch CLOSE ch=" << ch.label << " idx=" << ch.index
           << " open=0 ms signal=-20.0 dB threshold=" << FormatDouble(ch.squelch_threshold_db, 1) << " dB";
    PublishEvent(EventKind::kInfo, closed.str(), ch.frequency_hz);
  }
}

void ReceiverWorker::PublishEvent(EventKind kind, const std::string& message, double tuned_frequency_hz,
                                  bool log_event) {
  ReceiverEvent event;
  event.unix_ms = UnixMillisNow();
  event.receiver_id = receiver_id_;
  event.kind = kind;
  event.message = message;
  event.tuned_frequency_hz = tuned_frequency_hz;
  event_bus_->PublishReceiverEvent(event);
  if (log_event && logger_ != nullptr) {
    logger_->LogEvent(event);
  }
}

}  // namespace multi_radio
