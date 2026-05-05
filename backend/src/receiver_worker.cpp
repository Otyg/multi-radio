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

#include "multi_radio/am_demod.hpp"
#include "multi_radio/fm_demod.hpp"

#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
#include <liquid/liquid.h>
#endif

namespace multi_radio {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr uint32_t kDefaultSampleRateHz = 2048000;
constexpr uint32_t kWfmMaxRuntimeSampleRateHz = 1024000;
constexpr uint32_t kDefaultChannelBandwidthHz = 30000;
constexpr uint32_t kAudioSampleRateHzNfm = 12000;
// Keep WFM audio-rate moderate to stay real-time on typical CPUs.
// Frontend audio output can still resample to the device sample-rate.
constexpr uint32_t kAudioSampleRateHzWfm = 32000;
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
  if (out.fixed_modulation != Modulation::kNfm && out.fixed_modulation != Modulation::kWfm &&
      out.fixed_modulation != Modulation::kAm) {
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
  uint32_t dwell_ms = 0;
  uint32_t channel_bandwidth_hz = 0;
};

RuntimeChannel SelectRuntimeChannel(RadioMode mode, const ModeConfig& config, int channel_idx) {
  RuntimeChannel out;
  if (mode == RadioMode::kScanList && !config.scan_list_channels.empty()) {
    const int n = static_cast<int>(config.scan_list_channels.size());
    const int idx = std::clamp(channel_idx, 0, n - 1);
    const auto& ch = config.scan_list_channels[static_cast<size_t>(idx)];
    out.index = idx;
    out.label = ch.label.empty() ? ("ch" + std::to_string(idx)) : ch.label;
    out.frequency_hz = ch.frequency_hz;
    out.modulation = ch.modulation;
    out.squelch_threshold_db =
        ch.use_default_squelch ? config.scan_list_default_squelch_db : ch.squelch_threshold_db;
    out.dwell_ms = (ch.dwell_ms > 0) ? ch.dwell_ms : config.dwell_ms;
    out.channel_bandwidth_hz = (ch.channel_bandwidth_hz > 0) ? ch.channel_bandwidth_hz
                                                             : config.channel_bandwidth_hz;
    return out;
  }

  if (mode == RadioMode::kScanRange && !config.frequency_list_hz.empty()) {
    const int n = static_cast<int>(config.frequency_list_hz.size());
    const int idx = std::clamp(channel_idx, 0, n - 1);
    out.index = idx;
    out.frequency_hz = config.frequency_list_hz[static_cast<size_t>(idx)];
    out.label = "range-" + std::to_string(idx);
    out.modulation = config.fixed_modulation;
    out.channel_bandwidth_hz = config.channel_bandwidth_hz;
    out.dwell_ms = config.dwell_ms;
    return out;
  }

  out.frequency_hz = config.fixed_frequency_hz;
  out.modulation = config.fixed_modulation;
  out.channel_bandwidth_hz = config.channel_bandwidth_hz;
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

  // Initialize ring buffer for decoupling ingest from output timing
  // Size: enough to buffer ~2 seconds of WFM audio (32kHz * 2s = 64k samples)
  // Larger buffer provides better tolerance for timing variations
  audio_buffer_ = std::make_unique<AudioRingBuffer>(131072);

  thread_ = std::thread(&ReceiverWorker::RunLoop, this);
  ingest_thread_ = std::thread(&ReceiverWorker::IngestLoop, this);
  process_thread_ = std::thread(&ReceiverWorker::ProcessLoop, this);
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

  iq_queue_cv_.notify_all();
  if (ingest_thread_.joinable()) {
    ingest_thread_.join();
  }
  if (process_thread_.joinable()) {
    process_thread_.join();
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  audio_buffer_.reset();
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
    ModeConfig normalized = NormalizeModeConfig(config);
    // For scan range: always generate the frequency list from range params.
    // The client-supplied list may be polluted with entries from other modes.
    if (mode_ == RadioMode::kScanRange &&
        normalized.range_step_hz > 0.0 &&
        normalized.range_end_hz > normalized.range_start_hz) {
      normalized.frequency_list_hz.clear();
      for (double f = normalized.range_start_hz;
           f <= normalized.range_end_hz + normalized.range_step_hz * 0.01;
           f += normalized.range_step_hz) {
        normalized.frequency_list_hz.push_back(f);
      }
    }
    mode_config_ = std::move(normalized);
    scan_channel_idx_ = 0;
  }
  if (error != nullptr) {
    error->clear();
  }
  std::ostringstream msg;
  msg << "mode config updated freq_list=" << mode_config_.frequency_list_hz.size()
      << " range=" << FormatDouble(mode_config_.range_start_hz / 1e6, 3)
      << "-" << FormatDouble(mode_config_.range_end_hz / 1e6, 3)
      << " MHz step=" << FormatDouble(mode_config_.range_step_hz / 1e3, 1) << " kHz";
  PublishEvent(EventKind::kInfo, msg.str());
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

void ReceiverWorker::IngestLoop() {
  uint32_t configured_frequency_hz = 0;
  uint32_t configured_sample_rate_hz = 0;
  uint32_t configured_hardware_bandwidth_hz = 0;
  int configured_gain_tenth_db = std::numeric_limits<int>::min();
  bool device_opened = false;

  while (running_.load()) {
    RadioMode mode = RadioMode::kFixed;
    ModeConfig config;
    int scan_ch_idx = 0;
    {
      std::lock_guard<std::mutex> lock(mu_);
      mode = mode_;
      config = mode_config_;
      scan_ch_idx = scan_channel_idx_;
    }

    const RuntimeChannel ch = SelectRuntimeChannel(mode, config, scan_ch_idx);
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
    const uint32_t requested_sample_rate_hz = config.sample_rate_hz;
    const uint32_t effective_sample_rate_hz =
        std::min<uint32_t>(requested_sample_rate_hz, kWfmMaxRuntimeSampleRateHz);

    IQSampleBlock iq_block;
    bool have_iq = false;
    if (device_ != nullptr) {
      std::string error;
      if (!device_opened) {
        if (!device_->Open(&error)) {
          PublishEvent(EventKind::kError, FormatRuntimeError("rtl open (ingest)", error), ch.frequency_hz);
          {
            std::lock_guard<std::mutex> lock(mu_);
            last_error_ = error;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
          continue;
        }
        device_opened = true;
        PublishEvent(EventKind::kInfo, "rtl device opened (ingest thread)", ch.frequency_hz);
      }

      if (configured_sample_rate_hz != effective_sample_rate_hz) {
        if (!device_->SetSampleRateHz(effective_sample_rate_hz, &error)) {
          PublishEvent(EventKind::kError, FormatRuntimeError("set sample rate (ingest)", error), ch.frequency_hz);
          {
            std::lock_guard<std::mutex> lock(mu_);
            last_error_ = error;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        configured_sample_rate_hz = effective_sample_rate_hz;
      }

      if (configured_hardware_bandwidth_hz != config.hardware_bandwidth_hz) {
        if (!device_->SetHardwareBandwidthHz(config.hardware_bandwidth_hz, &error)) {
          PublishEvent(EventKind::kError, FormatRuntimeError("set hardware bandwidth (ingest)", error),
                       ch.frequency_hz);
          {
            std::lock_guard<std::mutex> lock(mu_);
            last_error_ = error;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        configured_hardware_bandwidth_hz = config.hardware_bandwidth_hz;
      }

      const int desired_gain_tenth_db = (ch.modulation == Modulation::kWfm) ? 0 : -1;
      if (configured_gain_tenth_db != desired_gain_tenth_db) {
        if (!device_->SetGainTenthdB(desired_gain_tenth_db, &error)) {
          PublishEvent(EventKind::kError, FormatRuntimeError("set gain (ingest)", error), ch.frequency_hz);
          {
            std::lock_guard<std::mutex> lock(mu_);
            last_error_ = error;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        configured_gain_tenth_db = desired_gain_tenth_db;
      }

      if (hardware_tuned_frequency_hz != 0 && hardware_tuned_frequency_hz != configured_frequency_hz) {
        if (!device_->SetCenterFrequencyHz(hardware_tuned_frequency_hz, &error)) {
          PublishEvent(EventKind::kError, FormatRuntimeError("set center frequency (ingest)", error),
                       ch.frequency_hz);
          {
            std::lock_guard<std::mutex> lock(mu_);
            last_error_ = error;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        configured_frequency_hz = hardware_tuned_frequency_hz;
      }

      if (!device_->ReadIq(&iq_block, &error)) {
        PublishEvent(EventKind::kWarning, FormatRuntimeError("read iq (ingest)", error), ch.frequency_hz);
        {
          std::lock_guard<std::mutex> lock(mu_);
          last_error_ = error;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      have_iq = true;
    } else {
      iq_block = BuildSyntheticIqBlock(effective_sample_rate_hz, tuned_frequency_hz, 0);
      have_iq = true;
    }

    if (have_iq) {
      IqQueueEntry entry;
      entry.block = std::move(iq_block);
      entry.effective_sample_rate_hz = effective_sample_rate_hz;
      entry.audio_sample_rate_hz = audio_sample_rate_hz;
      entry.channel_bandwidth_hz = (ch.channel_bandwidth_hz > 0) ? ch.channel_bandwidth_hz
                                                                  : config.channel_bandwidth_hz;
      entry.tuned_frequency_hz = ch.frequency_hz;
      entry.modulation = ch.modulation;
      entry.scan_channel_idx = scan_ch_idx;
      {
        std::lock_guard<std::mutex> lock(iq_queue_mu_);
        constexpr size_t kMaxQueueDepth = 4;
        if (iq_deque_.size() >= kMaxQueueDepth) {
          iq_deque_.pop_front();
        }
        iq_deque_.push_back(std::move(entry));
      }
      iq_queue_cv_.notify_one();
    }

    if (device_ == nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  if (device_ != nullptr && device_opened) {
    device_->Close();
  }
}

void ReceiverWorker::ProcessLoop() {
  FmDemodulator fm_demod;
  bool fm_demod_available = FmDemodulator::Available();
  bool fm_demod_configured = false;
  bool fm_demod_warned_unavailable = false;
  uint32_t fm_demod_input_sr_hz = 0;
  uint32_t fm_demod_audio_sr_hz = 0;
  uint32_t fm_demod_channel_bw_hz = 0;
  Modulation fm_demod_modulation = Modulation::kNfm;

  AmDemodulator am_demod;
  bool am_demod_available = AmDemodulator::Available();
  bool am_demod_configured = false;
  uint32_t am_demod_input_sr_hz = 0;
  uint32_t am_demod_audio_sr_hz = 0;
  uint32_t am_demod_channel_bw_hz = 0;
  uint32_t am_block_log_counter = 0;
  auto next_iq_visualization_at = std::chrono::steady_clock::now();
  uint64_t iq_sequence = 0;
  uint64_t iq_sample_index = 0;

  // Block arrival timing: measure the actual IQ delivery rate to compensate for
  // the ~3ms per-call OS overhead of rtlsdr_read_sync. With 128ms nominal blocks
  // a 3ms overhead gives 7.625 blocks/sec instead of 7.8125, producing only
  // 31232 audio samples/sec. By measuring the true rate and adjusting the
  // resampler's input_sr, we get the right number of output samples per block.
  std::chrono::steady_clock::time_point last_block_time{};
  bool have_last_block_time = false;
  double ema_block_period_s = 0.131;
  int block_rate_samples = 0;
  uint32_t adapted_iq_sr_hz = 0;
  bool adaptive_rate_applied = false;
  constexpr int kAdaptiveWarmupBlocks = 15;
  constexpr double kEmaAlpha = 0.08;

  // Post-demodulation audio filters (optional, applied to final PCM before ring buffer).
  // Three independent Butterworth IIR filters; recreated when audio sample rate changes.
#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
  iirfilt_rrrf audio_hpf300 = nullptr;  // standalone HPF at 300 Hz
  iirfilt_rrrf audio_lpf8k = nullptr;   // standalone LPF at 8 kHz
  iirfilt_rrrf audio_bpf_hpf = nullptr; // BPF internal HPF component at 300 Hz
  iirfilt_rrrf audio_bpf_lpf = nullptr; // BPF internal LPF component at 3 kHz
#endif
  uint32_t audio_filter_sr_hz = 0;
  std::vector<float> audio_filter_scratch;

  auto rebuild_audio_filters = [&](uint32_t sr_hz) {
#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
    if (audio_hpf300 != nullptr) { iirfilt_rrrf_destroy(audio_hpf300); audio_hpf300 = nullptr; }
    if (audio_lpf8k != nullptr) { iirfilt_rrrf_destroy(audio_lpf8k); audio_lpf8k = nullptr; }
    if (audio_bpf_hpf != nullptr) { iirfilt_rrrf_destroy(audio_bpf_hpf); audio_bpf_hpf = nullptr; }
    if (audio_bpf_lpf != nullptr) { iirfilt_rrrf_destroy(audio_bpf_lpf); audio_bpf_lpf = nullptr; }
    if (sr_hz > 0) {
      const float sr = static_cast<float>(sr_hz);
      const float fc_hpf = 300.0f / sr;
      const float fc_lpf3k = std::min(3000.0f, sr * 0.45f) / sr;
      const float fc_lpf8k = std::min(8000.0f, sr * 0.45f) / sr;
      audio_hpf300 = iirfilt_rrrf_create_prototype(
          LIQUID_IIRDES_BUTTER, LIQUID_IIRDES_HIGHPASS, LIQUID_IIRDES_SOS, 2, fc_hpf, 0.0f, 1.0f, 60.0f);
      audio_lpf8k = iirfilt_rrrf_create_prototype(
          LIQUID_IIRDES_BUTTER, LIQUID_IIRDES_LOWPASS, LIQUID_IIRDES_SOS, 2, fc_lpf8k, 0.0f, 1.0f, 60.0f);
      audio_bpf_hpf = iirfilt_rrrf_create_prototype(
          LIQUID_IIRDES_BUTTER, LIQUID_IIRDES_HIGHPASS, LIQUID_IIRDES_SOS, 2, fc_hpf, 0.0f, 1.0f, 60.0f);
      audio_bpf_lpf = iirfilt_rrrf_create_prototype(
          LIQUID_IIRDES_BUTTER, LIQUID_IIRDES_LOWPASS, LIQUID_IIRDES_SOS, 2, fc_lpf3k, 0.0f, 1.0f, 60.0f);
    }
#else
    (void)sr_hz;
#endif
    audio_filter_sr_hz = sr_hz;
  };

  auto apply_audio_filters = [&](std::vector<int16_t>* pcm) {
#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
    if (pcm == nullptr || pcm->empty()) return;
    bool do_hpf = false, do_lpf = false, do_bpf = false;
    {
      std::lock_guard<std::mutex> lock(mu_);
      do_hpf = mode_config_.audio_hpf300_enabled;
      do_lpf = mode_config_.audio_lpf8k_enabled;
      do_bpf = mode_config_.audio_bpf_voice_enabled;
    }
    if (!do_hpf && !do_lpf && !do_bpf) return;
    const size_t n = pcm->size();
    audio_filter_scratch.resize(n);
    for (size_t i = 0; i < n; ++i) {
      audio_filter_scratch[i] = static_cast<float>((*pcm)[i]) / 32768.0f;
    }
    if (do_bpf) {
      if (audio_bpf_hpf != nullptr) {
        iirfilt_rrrf_execute_block(audio_bpf_hpf, audio_filter_scratch.data(),
                                   static_cast<unsigned int>(n), audio_filter_scratch.data());
      }
      if (audio_bpf_lpf != nullptr) {
        iirfilt_rrrf_execute_block(audio_bpf_lpf, audio_filter_scratch.data(),
                                   static_cast<unsigned int>(n), audio_filter_scratch.data());
      }
    }
    if (do_hpf && audio_hpf300 != nullptr) {
      iirfilt_rrrf_execute_block(audio_hpf300, audio_filter_scratch.data(),
                                 static_cast<unsigned int>(n), audio_filter_scratch.data());
    }
    if (do_lpf && audio_lpf8k != nullptr) {
      iirfilt_rrrf_execute_block(audio_lpf8k, audio_filter_scratch.data(),
                                 static_cast<unsigned int>(n), audio_filter_scratch.data());
    }
    for (size_t i = 0; i < n; ++i) {
      const float s = std::clamp(audio_filter_scratch[i], -1.0f, 1.0f);
      (*pcm)[i] = static_cast<int16_t>(std::lrint(s * 32767.0f));
    }
#else
    (void)pcm;
#endif
  };

  int last_processed_channel_idx = -1;

  while (running_.load()) {
    IqQueueEntry entry;
    {
      std::unique_lock<std::mutex> lock(iq_queue_mu_);
      iq_queue_cv_.wait(lock, [this] { return !iq_deque_.empty() || !running_.load(); });
      if (!running_.load() && iq_deque_.empty()) {
        break;
      }
      entry = std::move(iq_deque_.front());
      iq_deque_.pop_front();
    }

    // Detect scan channel transitions: clear ring buffer and reset demods so old-channel
    // audio cannot bleed into the new channel's output. audio_channel_idx_ is the source
    // of truth for SCAN_STATUS — it only advances when the audio actually switches.
    const int new_idx = entry.scan_channel_idx;
    if (new_idx >= 0 && new_idx != last_processed_channel_idx &&
        last_processed_channel_idx >= 0) {
      if (audio_buffer_ != nullptr) {
        audio_buffer_->Clear();
      }
      fm_demod.Reset();
      fm_demod_configured = false;
      am_demod.Reset();
      am_demod_configured = false;
      rebuild_audio_filters(0);
      audio_filter_sr_hz = 0;
      {
        std::lock_guard<std::mutex> lock(mu_);
        audio_channel_idx_ = new_idx;
      }
      last_processed_channel_idx = new_idx;
      // Fall through: process this block with the freshly reset demod state.
      // The RTL-SDR PLL settles in <1 ms, so by the time the USB read finishes
      // (128 ms later) the IQ data is clean. The demod reconfigures on this block.
    }
    if (last_processed_channel_idx < 0 && new_idx >= 0) {
      last_processed_channel_idx = new_idx;
      std::lock_guard<std::mutex> lock(mu_);
      audio_channel_idx_ = new_idx;
    }

    // Measure inter-block arrival time and build adapted IQ rate estimate.
    // Freeze updates once the adapted rate has been applied to avoid reconfiguring.
    if (!adaptive_rate_applied) {
      const auto block_now = std::chrono::steady_clock::now();
      if (have_last_block_time) {
        const double period = std::chrono::duration<double>(block_now - last_block_time).count();
        if (period > 0.04 && period < 0.6) {  // sanity gate: 40ms–600ms
          ++block_rate_samples;
          if (block_rate_samples <= 3) {
            ema_block_period_s = ema_block_period_s + (period - ema_block_period_s) /
                                 static_cast<double>(block_rate_samples + 1);
          } else {
            ema_block_period_s = kEmaAlpha * period + (1.0 - kEmaAlpha) * ema_block_period_s;
          }
          if (block_rate_samples >= kAdaptiveWarmupBlocks) {
            const uint64_t block_iq_pairs = entry.block.interleaved_iq.size() / 2U;
            if (block_iq_pairs > 0 && ema_block_period_s > 0.001) {
              const double measured = static_cast<double>(block_iq_pairs) / ema_block_period_s;
              const double nominal = static_cast<double>(entry.effective_sample_rate_hz);
              // Only adopt if it deviates enough to matter (>0.5%) and is plausible (±15%)
              if (std::abs(measured - nominal) / nominal > 0.005) {
                adapted_iq_sr_hz = static_cast<uint32_t>(
                    std::llround(std::clamp(measured, nominal * 0.85, nominal * 1.15)));
              }
            }
          }
        }
      }
      last_block_time = block_now;
      have_last_block_time = true;
    }

    // IQ stats accumulation
    {
      uint64_t block_components = 0;
      uint64_t block_clipped = 0;
      double block_power = 0.0;
      for (int16_t s : entry.block.interleaved_iq) {
        const double norm = static_cast<double>(s) / 32768.0;
        block_power += norm * norm;
        ++block_components;
        if (s >= kIqClipThresholdS16 || s <= -kIqClipThresholdS16) {
          ++block_clipped;
        }
      }
      const uint64_t block_samples = entry.block.interleaved_iq.size() / 2U;
      const uint32_t block_sr_hz = entry.block.sample_rate_hz != 0 ? entry.block.sample_rate_hz
                                                                    : entry.effective_sample_rate_hz;
      {
        std::lock_guard<std::mutex> lock(mu_);
        iq_shared_.sample_rate_hz = block_sr_hz;
        iq_shared_.latest_block = entry.block;
        iq_shared_.have_latest_block = true;
        iq_shared_.window_samples += block_samples;
        iq_shared_.window_components += block_components;
        iq_shared_.window_clipped_components += block_clipped;
        iq_shared_.window_component_power += block_power;
        iq_shared_.interleaved_samples += static_cast<uint64_t>(entry.block.interleaved_iq.size());
      }

      const auto iq_now = std::chrono::steady_clock::now();
      if (iq_now >= next_iq_visualization_at) {
        IqFrame iq_frame = BuildIqFrame(entry.block, receiver_id_, entry.tuned_frequency_hz,
                                        iq_sequence++, iq_sample_index);
        event_bus_->PublishIqFrame(iq_frame);
        next_iq_visualization_at = iq_now + std::chrono::milliseconds(kIqVisualizationIntervalMs);
      }
      iq_sample_index += block_samples;
    }

    // Scan range: IQ frames published above; skip all demodulation and audio.
    {
      std::lock_guard<std::mutex> lock(mu_);
      if (mode_ == RadioMode::kScanRange) {
        continue;
      }
    }

    // Rebuild post-demod audio filters if sample rate changed
    if (entry.audio_sample_rate_hz != audio_filter_sr_hz) {
      rebuild_audio_filters(entry.audio_sample_rate_hz);
    }

    // FM demodulation
    if (IsFmModulation(entry.modulation)) {
      if (!fm_demod_available) {
        if (!fm_demod_warned_unavailable) {
          PublishEvent(EventKind::kWarning, "FM demod unavailable: backend built without libliquid",
                       entry.tuned_frequency_hz);
          fm_demod_warned_unavailable = true;
        }
      } else {
        // Use adapted IQ rate whenever measured (compensates for USB read overhead).
        // adaptive_rate_applied only freezes the EMA measurement; it does NOT gate
        // which rate is used for demod — the adapted rate is always preferred once known.
        const uint32_t nominal_iq_sr = entry.block.sample_rate_hz != 0 ? entry.block.sample_rate_hz
                                                                        : entry.effective_sample_rate_hz;
        const uint32_t demod_input_sr_hz = (adapted_iq_sr_hz != 0) ? adapted_iq_sr_hz : nominal_iq_sr;

        const bool other_params_changed =
            fm_demod_audio_sr_hz != entry.audio_sample_rate_hz ||
            fm_demod_channel_bw_hz != entry.channel_bandwidth_hz ||
            fm_demod_modulation != entry.modulation;
        const bool demod_reconfigure = !fm_demod_configured || other_params_changed ||
                                       fm_demod_input_sr_hz != demod_input_sr_hz;
        if (demod_reconfigure) {
          std::string demod_error;
          if (!fm_demod.Configure(demod_input_sr_hz, entry.audio_sample_rate_hz, entry.modulation,
                                  entry.channel_bandwidth_hz, &demod_error)) {
            fm_demod_configured = false;
            PublishEvent(EventKind::kWarning, FormatRuntimeError("configure fm demod (process)", demod_error),
                         entry.tuned_frequency_hz);
          } else {
            fm_demod_configured = true;
            fm_demod_input_sr_hz = demod_input_sr_hz;
            fm_demod_audio_sr_hz = entry.audio_sample_rate_hz;
            fm_demod_channel_bw_hz = entry.channel_bandwidth_hz;
            fm_demod_modulation = entry.modulation;
            if (adapted_iq_sr_hz != 0 && !adaptive_rate_applied) {
              adaptive_rate_applied = true;  // freeze EMA once first adapted config is applied
            }
            std::ostringstream demod_msg;
            demod_msg << "FM demod configured (process) mod=" << ModulationToken(entry.modulation)
                      << " iq_sr=" << fm_demod_input_sr_hz << " audio_sr=" << fm_demod_audio_sr_hz
                      << " bw=" << fm_demod_channel_bw_hz
                      << (adapted_iq_sr_hz != 0 ? " (adapted)" : " (nominal)");
            PublishEvent(EventKind::kInfo, demod_msg.str(), entry.tuned_frequency_hz);
          }
        }

        if (fm_demod_configured) {
          std::vector<int16_t> demod_pcm;
          FmDemodProcessStats demod_stats;
          std::string demod_error;
          if (!fm_demod.ProcessIq(entry.block.interleaved_iq, &demod_pcm, &demod_stats, &demod_error)) {
            PublishEvent(EventKind::kWarning, FormatRuntimeError("fm demod (process)", demod_error),
                         entry.tuned_frequency_hz);
          } else {
            {
              std::lock_guard<std::mutex> lock(mu_);
              iq_shared_.channel_rssi_db = demod_stats.channel_rssi_db;
            }
            apply_audio_filters(&demod_pcm);
            if (audio_buffer_ != nullptr && !demod_pcm.empty()) {
              size_t samples_written = 0;
              const size_t to_write = demod_pcm.size();
              size_t attempts = 0;
              while (samples_written < to_write && running_.load() && attempts < 10) {
                samples_written += audio_buffer_->Write(demod_pcm.data() + samples_written,
                                                        to_write - samples_written);
                if (samples_written < to_write) {
                  std::this_thread::sleep_for(std::chrono::milliseconds(1));
                  ++attempts;
                }
              }
              if (samples_written < to_write) {
                PublishEvent(EventKind::kWarning,
                             FormatRuntimeError("audio buffer overflow (process)",
                                                "dropped " + std::to_string(to_write - samples_written) +
                                                    " samples"),
                             entry.tuned_frequency_hz);
              }
            }
          }
        }
        am_demod.Reset();
        am_demod_configured = false;
      }
    } else if (entry.modulation == Modulation::kAm) {
      fm_demod.Reset();
      fm_demod_configured = false;

      if (!am_demod_available) {
        // No action; AM requires libliquid
      } else {
        const uint32_t nominal_iq_sr = entry.block.sample_rate_hz != 0
                                           ? entry.block.sample_rate_hz
                                           : entry.effective_sample_rate_hz;
        const uint32_t demod_input_sr = (adapted_iq_sr_hz != 0) ? adapted_iq_sr_hz : nominal_iq_sr;
        const bool am_reconfigure = !am_demod_configured ||
                                    am_demod_input_sr_hz != demod_input_sr ||
                                    am_demod_audio_sr_hz != entry.audio_sample_rate_hz ||
                                    am_demod_channel_bw_hz != entry.channel_bandwidth_hz;
        if (am_reconfigure) {
          std::string demod_error;
          if (!am_demod.Configure(demod_input_sr, entry.audio_sample_rate_hz,
                                  entry.channel_bandwidth_hz, &demod_error)) {
            am_demod_configured = false;
            PublishEvent(EventKind::kWarning,
                         FormatRuntimeError("configure am demod (process)", demod_error),
                         entry.tuned_frequency_hz);
          } else {
            am_demod_configured = true;
            am_demod_input_sr_hz = demod_input_sr;
            am_demod_audio_sr_hz = entry.audio_sample_rate_hz;
            am_demod_channel_bw_hz = entry.channel_bandwidth_hz;
            if (adapted_iq_sr_hz != 0 && !adaptive_rate_applied) {
              adaptive_rate_applied = true;  // freeze EMA
            }
            std::ostringstream demod_msg;
            demod_msg << "AM demod configured (process)"
                      << " iq_sr=" << am_demod_input_sr_hz
                      << " audio_sr=" << am_demod_audio_sr_hz
                      << " bw=" << am_demod_channel_bw_hz
                      << (adapted_iq_sr_hz != 0 ? " (adapted)" : " (nominal)");
            PublishEvent(EventKind::kInfo, demod_msg.str(), entry.tuned_frequency_hz);
          }
        }

        if (am_demod_configured) {
          std::vector<int16_t> demod_pcm;
          AmDemodProcessStats demod_stats;
          std::string demod_error;
          if (!am_demod.ProcessIq(entry.block.interleaved_iq, &demod_pcm, &demod_stats, &demod_error)) {
            PublishEvent(EventKind::kWarning, FormatRuntimeError("am demod (process)", demod_error),
                         entry.tuned_frequency_hz);
          } else {
            {
              std::lock_guard<std::mutex> lock(mu_);
              iq_shared_.channel_rssi_db = demod_stats.channel_rssi_db;
            }
            // Periodic diagnostic: log peak PCM amplitude every ~2 seconds (~16 blocks at 128ms/block).
            // Helps diagnose silence: if peak=0, the AM signal has no modulation (unmodulated carrier).
            if ((++am_block_log_counter % 16U) == 0U) {
              int16_t peak_pcm = 0;
              for (int16_t s : demod_pcm) {
                peak_pcm = std::max(peak_pcm, static_cast<int16_t>(std::abs(s)));
              }
              std::ostringstream am_diag;
              am_diag << "AM_DIAG samples=" << demod_pcm.size()
                      << " peak_pcm=" << peak_pcm
                      << " rssi_db=" << FormatDouble(static_cast<double>(demod_stats.channel_rssi_db), 1);
              PublishEvent(EventKind::kInfo, am_diag.str(), entry.tuned_frequency_hz, false);
            }
            apply_audio_filters(&demod_pcm);
            if (audio_buffer_ != nullptr && !demod_pcm.empty()) {
              size_t samples_written = 0;
              const size_t to_write = demod_pcm.size();
              size_t attempts = 0;
              while (samples_written < to_write && running_.load() && attempts < 10) {
                samples_written += audio_buffer_->Write(demod_pcm.data() + samples_written,
                                                        to_write - samples_written);
                if (samples_written < to_write) {
                  std::this_thread::sleep_for(std::chrono::milliseconds(1));
                  ++attempts;
                }
              }
              if (samples_written < to_write) {
                PublishEvent(EventKind::kWarning,
                             FormatRuntimeError("audio buffer overflow (process)",
                                                "dropped " + std::to_string(to_write - samples_written) +
                                                    " samples"),
                             entry.tuned_frequency_hz);
              }
            }
          }
        }
      }
    } else {
      fm_demod.Reset();
      fm_demod_configured = false;
      am_demod.Reset();
      am_demod_configured = false;
    }
  }

  rebuild_audio_filters(0);  // destroy filter objects on exit
}

void ReceiverWorker::RunLoop() {
  const SignalHealthThresholds& thresholds = GlobalSignalHealthThresholds();
  uint64_t audio_sequence = 0;
  uint64_t audio_sample_index = 0;
  uint64_t generated_samples = 0;
  uint64_t published_samples = 0;
  uint64_t published_frames = 0;
  uint64_t conceal_samples = 0;
  bool have_prev_iq_health = false;
  double prev_iq_snr_db = 0.0;
  double prev_iq_level_dbfs = -120.0;
  int iq_stable_windows = 0;
  int iq_good_windows = 0;
  int iq_bad_windows = 0;
  bool signal_ok_latched = false;

  uint64_t demod_ok_blocks = 0;
  uint64_t demod_empty_blocks = 0;

  const auto frame_interval = std::chrono::milliseconds(kAudioFrameIntervalMs);
  auto next_frame_at = std::chrono::steady_clock::now();
  auto stats_started_at = next_frame_at;
  auto next_stats_at = next_frame_at + std::chrono::milliseconds(kAudioStatsIntervalMs);

  uint32_t last_requested_sample_rate_hz = 0;
  uint32_t last_effective_sample_rate_hz = 0;
  bool thresholds_emitted = false;
  bool buffer_primed = false;

  // Scan list state machine
  enum class ScanState { kWaiting, kOpen, kTail };
  ScanState scan_state = ScanState::kWaiting;
  bool scan_audio_muted = true;   // true → zero-fill frames; only active for scanner non-monitor mode
  auto scan_dwell_started_at = std::chrono::steady_clock::now();
  auto scan_tail_started_at = std::chrono::steady_clock::now();
  auto next_scan_check_at = std::chrono::steady_clock::now();
  constexpr int64_t kScanTailMs = 500;  // hold time after signal drops before advancing

  while (running_.load()) {
    RadioMode mode = RadioMode::kFixed;
    ModeConfig config;
    int scan_ch_idx = 0;
    {
      std::lock_guard<std::mutex> lock(mu_);
      mode = mode_;
      config = mode_config_;
      scan_ch_idx = scan_channel_idx_;
    }

    // Scan range: advance scan channel on dwell expiry, then sleep — no audio processing.
    if (mode == RadioMode::kScanRange) {
      const auto now_sr = std::chrono::steady_clock::now();
      if (!config.frequency_list_hz.empty() && now_sr >= next_scan_check_at) {
        next_scan_check_at = now_sr + std::chrono::milliseconds(200);
        const int n = static_cast<int>(config.frequency_list_hz.size());
        int cur_idx;
        {
          std::lock_guard<std::mutex> lock(mu_);
          cur_idx = scan_channel_idx_;
        }
        const int64_t dwell_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     now_sr - scan_dwell_started_at).count();
        if (n > 1 && dwell_ms >= static_cast<int64_t>(config.dwell_ms)) {
          const int next_idx = (cur_idx + 1) % n;
          {
            std::lock_guard<std::mutex> lock(mu_);
            scan_channel_idx_ = next_idx;
          }
          scan_dwell_started_at = now_sr;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    const RuntimeChannel ch = SelectRuntimeChannel(mode, config, scan_ch_idx);
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
    const uint32_t requested_sample_rate_hz = config.sample_rate_hz;
    const uint32_t effective_sample_rate_hz =
        std::min<uint32_t>(requested_sample_rate_hz, kWfmMaxRuntimeSampleRateHz);
    if (effective_sample_rate_hz != requested_sample_rate_hz &&
        (last_requested_sample_rate_hz != requested_sample_rate_hz ||
         last_effective_sample_rate_hz != effective_sample_rate_hz)) {
      std::ostringstream sr_msg;
      sr_msg << "WFM sample-rate capped from " << requested_sample_rate_hz
             << " to " << effective_sample_rate_hz << " Hz for real-time stability";
      PublishEvent(EventKind::kInfo, sr_msg.str(), ch.frequency_hz);
      last_requested_sample_rate_hz = requested_sample_rate_hz;
      last_effective_sample_rate_hz = effective_sample_rate_hz;
    } else if (effective_sample_rate_hz == requested_sample_rate_hz) {
      last_requested_sample_rate_hz = requested_sample_rate_hz;
      last_effective_sample_rate_hz = effective_sample_rate_hz;
    }
    const size_t frame_samples = AudioFrameSamplesForRate(audio_sample_rate_hz);
    if (frame_samples == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    // Pre-buffering: wait until the ring buffer holds ~300ms of audio before starting output.
    // Scanner mode bypasses this — it sends muted (zero) frames while squelch is closed,
    // so there is no underrun risk and no need to accumulate a prefill.
    if (!buffer_primed && audio_buffer_ != nullptr) {
      if (mode == RadioMode::kScanList) {
        buffer_primed = true;  // scanner: muted frames prevent underruns, no prefill needed
      } else {
        const size_t minimum_prefill =
            static_cast<size_t>(static_cast<uint64_t>(audio_sample_rate_hz) * 300U / 1000U);
        if (audio_buffer_->AvailableForRead() < minimum_prefill) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        buffer_primed = true;
        PublishEvent(EventKind::kInfo, "Audio buffer primed, starting output stream", ch.frequency_hz);
      }
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

      // Read from ring buffer (populated by IngestLoop)
      // Implement adaptive dropout mitigation: if buffer is low, retry after a short sleep
      // instead of immediately padding with zeros
      size_t read_attempts = 0;
      const size_t max_read_attempts = 10;
      const size_t critical_level = frame_samples / 2;  // If buffer < 0.5 frames, it's critical
      
      while (frame_filled < frame_samples && read_attempts < max_read_attempts && audio_buffer_ != nullptr) {
        size_t available = audio_buffer_->AvailableForRead();
        size_t to_read = std::min(frame_samples - frame_filled, available);
        
        if (to_read > 0) {
          frame_filled += audio_buffer_->Read(
              frame.pcm_s16le.data() + frame_filled, 
              to_read);
        }
        
        if (frame_filled < frame_samples && available < critical_level) {
          // Buffer is critically low and we need more samples
          // Give IngestLoop a chance to fill it
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          ++read_attempts;
        } else {
          break;  // Either we got enough or buffer is empty
        }
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

      // Squelch audio gate: mute frame when scanner squelch is closed (non-monitor mode).
      // The ring buffer is still drained above to prevent overflow; only output is silenced.
      if (mode == RadioMode::kScanList && !config.scan_list_monitor_mode && scan_audio_muted) {
        std::fill(frame.pcm_s16le.begin(), frame.pcm_s16le.end(), int16_t{0});
      }

      generated_samples += frame_filled;
      published_samples += static_cast<uint64_t>(frame_samples);
      ++published_frames;
      audio_sample_index += static_cast<uint64_t>(frame_samples);
      event_bus_->PublishAudioFrame(frame);
      next_frame_at += frame_interval;
    }

    // Scan list: check squelch and advance channel every 200ms.
    // Normal mode: squelch-gated audio + channel advance only on squelch close.
    // Monitor mode: full dwell always, audio always on (for calibration).
    if (mode == RadioMode::kScanList && !config.scan_list_channels.empty() &&
        now >= next_scan_check_at) {
      next_scan_check_at = now + std::chrono::milliseconds(200);

      int cur_ingest_idx;
      float channel_rssi_db = -120.0f;
      {
        std::lock_guard<std::mutex> lock(mu_);
        cur_ingest_idx = scan_channel_idx_;
        channel_rssi_db = iq_shared_.channel_rssi_db;
      }
      const int n = static_cast<int>(config.scan_list_channels.size());
      const auto& cur_chan = config.scan_list_channels[static_cast<size_t>(cur_ingest_idx)];
      const double squelch_db = cur_chan.use_default_squelch
                                    ? config.scan_list_default_squelch_db
                                    : cur_chan.squelch_threshold_db;
      const bool signal_above = static_cast<double>(channel_rssi_db) >= squelch_db;
      const uint32_t eff_dwell_ms = (cur_chan.dwell_ms > 0) ? cur_chan.dwell_ms : config.dwell_ms;
      const int64_t dwell_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           now - scan_dwell_started_at).count();

      bool should_advance = false;

      if (config.scan_list_monitor_mode) {
        // Monitor mode: advance on dwell only, audio always unmuted.
        scan_audio_muted = false;
        if (n > 1 && dwell_elapsed_ms >= static_cast<int64_t>(eff_dwell_ms)) {
          should_advance = true;
        }
      } else {
        // Normal scanner mode: squelch gates audio and channel advance.
        switch (scan_state) {
          case ScanState::kWaiting:
            // No signal yet — wait for squelch to open or dwell to expire.
            if (signal_above) {
              scan_state = ScanState::kOpen;
              scan_audio_muted = false;
            } else if (n > 1 && dwell_elapsed_ms >= static_cast<int64_t>(eff_dwell_ms)) {
              should_advance = true;  // no signal in dwell window, skip channel
            }
            break;
          case ScanState::kOpen:
            // Signal is present — stay on channel regardless of dwell.
            if (!signal_above) {
              scan_state = ScanState::kTail;
              scan_tail_started_at = now;
              scan_audio_muted = true;
            }
            break;
          case ScanState::kTail:
            // Signal just dropped — brief hold before advancing (handles gaps in transmission).
            if (signal_above) {
              scan_state = ScanState::kOpen;  // signal came back
              scan_audio_muted = false;
            } else {
              const int64_t tail_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                  now - scan_tail_started_at).count();
              if (tail_elapsed_ms >= kScanTailMs) {
                should_advance = true;  // tail expired, move on
              }
            }
            break;
        }
      }

      if (should_advance) {
        if (n > 1) {
          const int next_idx = (cur_ingest_idx + 1) % n;
          {
            std::lock_guard<std::mutex> lock(mu_);
            scan_channel_idx_ = next_idx;
          }
        }
        scan_dwell_started_at = now;
        scan_state = ScanState::kWaiting;
        scan_audio_muted = true;
      }

      // SCAN_STATUS: report audio_channel_idx_ (the channel whose audio is in the ring buffer).
      int audio_idx;
      {
        std::lock_guard<std::mutex> lock(mu_);
        audio_idx = audio_channel_idx_;
      }
      if (audio_idx < 0 || audio_idx >= n) {
        audio_idx = cur_ingest_idx;
      }
      const auto& report_chan = config.scan_list_channels[static_cast<size_t>(audio_idx)];
      const double report_squelch = report_chan.use_default_squelch
                                        ? config.scan_list_default_squelch_db
                                        : report_chan.squelch_threshold_db;
      // State for SCAN_STATUS: in monitor mode use raw RSSI; in normal mode use scan_state.
      const bool state_open = config.scan_list_monitor_mode
                                  ? (static_cast<double>(channel_rssi_db) >= report_squelch)
                                  : (scan_state == ScanState::kOpen);

      std::ostringstream scan_ss;
      scan_ss << "SCAN_STATUS idx=" << audio_idx
              << " state=" << (state_open ? "open" : "closed")
              << " signal_db=" << FormatDouble(static_cast<double>(channel_rssi_db), 1)
              << " threshold_db=" << FormatDouble(report_squelch, 1)
              << " monitor=" << (config.scan_list_monitor_mode ? 1 : 0);
      PublishEvent(EventKind::kInfo, scan_ss.str(), report_chan.frequency_hz, false);
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

      IqSharedState iq_snap;
      {
        std::lock_guard<std::mutex> lock(mu_);
        iq_snap = iq_shared_;
        iq_shared_.window_samples = 0;
        iq_shared_.window_components = 0;
        iq_shared_.window_clipped_components = 0;
        iq_shared_.window_component_power = 0.0;
        iq_shared_.interleaved_samples = 0;
      }

      const double window_s =
          std::max(1.0e-3, std::chrono::duration<double>(now - stats_started_at).count());
      const uint64_t window_ms = static_cast<uint64_t>(std::llround(window_s * 1000.0));
      const double gen_hz = static_cast<double>(generated_samples) / window_s;
      const double pub_hz = static_cast<double>(published_samples) / window_s;
      const double iq_measured_sample_rate_hz = static_cast<double>(iq_snap.window_samples) / window_s;
      const double iq_rms = (iq_snap.window_components == 0)
                                ? 0.0
                                : std::sqrt(iq_snap.window_component_power /
                                            static_cast<double>(iq_snap.window_components));
      const double iq_level_dbfs =
          20.0 * std::log10(std::max(1.0e-9, std::min(1.0, iq_rms)));
      const double iq_clip_pct =
          (iq_snap.window_components == 0)
              ? 0.0
              : (100.0 * static_cast<double>(iq_snap.window_clipped_components) /
                 static_cast<double>(iq_snap.window_components));
      const PsdSummary psd = iq_snap.have_latest_block
                                 ? EstimatePsdSummary(iq_snap.latest_block.interleaved_iq, iq_snap.sample_rate_hz)
                                 : PsdSummary{};

      std::ostringstream audio_status;
      audio_status << "AUDIO_STATS idx=" << ch.index
                   << " label=" << ch.label
                   << " rev=" << kAudioPipelineRevision
                   << " mod=" << ModulationToken(ch.modulation)
                   << " sr=" << audio_sample_rate_hz
                   << " cfg_sr=" << requested_sample_rate_hz
                   << " run_sr=" << effective_sample_rate_hz
                   << " iq_sr=" << iq_snap.sample_rate_hz
                   << " iq_est_sr=" << iq_snap.sample_rate_hz
                   << " iq_lock=0"
                   << " iq_lock_sr=" << iq_snap.sample_rate_hz
                   << " win_ms=" << window_ms
                   << " gen_ratio=1.000"
                   << " rate_corr=1.0000"
                   << " gen_hz=" << FormatDouble(gen_hz, 1)
                   << " pub_hz=" << FormatDouble(pub_hz, 1)
                   << " iq_n=" << iq_snap.interleaved_samples
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
                   << " buffer_available=" << (audio_buffer_ ? audio_buffer_->AvailableForRead() : 0)
                   << " conceal_samples=" << conceal_samples
                   << " clears=0"
                   << " flush_frames=0"
                   << " flush_samples=0";
      PublishEvent(EventKind::kInfo, audio_status.str(), ch.frequency_hz, false);

      std::ostringstream iq_status;
      const double configured_iq_sample_rate_hz = static_cast<double>(effective_sample_rate_hz);
      const double block_sr_hz = static_cast<double>(iq_snap.sample_rate_hz);
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
                << " run_sr=" << effective_sample_rate_hz
                << " meas_sr=" << FormatDouble(iq_measured_sample_rate_hz, 0)
                << " block_sr=" << iq_snap.sample_rate_hz
                << " tuned_hz=" << tuned_frequency_hz
                << " center_hz=" << (iq_snap.have_latest_block ? iq_snap.latest_block.center_frequency_hz : 0U)
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
                << " iq_n=" << iq_snap.interleaved_samples;
      PublishEvent(EventKind::kInfo, iq_status.str(), ch.frequency_hz);

      generated_samples = 0;
      published_samples = 0;
      published_frames = 0;
      conceal_samples = 0;
      demod_ok_blocks = 0;
      demod_empty_blocks = 0;
      stats_started_at = now;
      next_stats_at = now + std::chrono::milliseconds(kAudioStatsIntervalMs);
    }
    if (device_ == nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
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
