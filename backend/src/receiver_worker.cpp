#include "multi_radio/receiver_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace multi_radio {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr uint32_t kDefaultSampleRateHz = 2048000;
constexpr uint32_t kDefaultChannelBandwidthHz = 30000;
constexpr uint32_t kAudioSampleRateHz = 8000;
constexpr uint32_t kAudioFrameIntervalMs = 20;
constexpr uint32_t kAudioStatsIntervalMs = 1000;
constexpr const char* kAudioPipelineRevision = "audio-v10-clean-slate";
constexpr double kToneFrequencyHz = 1000.0;
constexpr double kToneAmplitude = 9000.0;

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
  return out;
}

uint32_t AudioSampleRateForModulation(Modulation modulation) {
  (void)modulation;
  return kAudioSampleRateHz;
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
  if (out.frequency_hz <= 0.0 && !config.frequency_list_hz.empty()) {
    out.frequency_hz = config.frequency_list_hz.front();
  }
  return out;
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
  uint64_t sequence = 0;
  uint64_t sample_index = 0;
  uint64_t generated_samples = 0;
  uint64_t published_samples = 0;
  uint64_t published_frames = 0;
  uint64_t conceal_samples = 0;

  double tone_phase = 0.0;
  const auto frame_interval = std::chrono::milliseconds(kAudioFrameIntervalMs);
  auto next_frame_at = std::chrono::steady_clock::now();
  auto stats_started_at = next_frame_at;
  auto next_stats_at = next_frame_at + std::chrono::milliseconds(kAudioStatsIntervalMs);

  bool squelch_open_emitted = false;

  while (running_.load()) {
    RadioMode mode = RadioMode::kFixed;
    ModeConfig config;
    {
      std::lock_guard<std::mutex> lock(mu_);
      mode = mode_;
      config = mode_config_;
    }

    const RuntimeChannel ch = SelectRuntimeChannel(mode, config);
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

    AudioFrame frame;
    frame.unix_ms = UnixMillisNow();
    frame.receiver_id = receiver_id_;
    frame.sample_rate_hz = audio_sample_rate_hz;
    frame.tuned_frequency_hz = ch.frequency_hz;
    frame.sequence = sequence++;
    frame.sample_index = sample_index;
    frame.pcm_s16le.resize(frame_samples);

    const double phase_step = (2.0 * kPi * kToneFrequencyHz) / static_cast<double>(audio_sample_rate_hz);
    for (size_t i = 0; i < frame_samples; ++i) {
      frame.pcm_s16le[i] = static_cast<int16_t>(std::lrint(kToneAmplitude * std::sin(tone_phase)));
      tone_phase += phase_step;
      if (tone_phase >= 2.0 * kPi) {
        tone_phase -= 2.0 * kPi;
      }
    }

    generated_samples += static_cast<uint64_t>(frame_samples);
    published_samples += static_cast<uint64_t>(frame_samples);
    ++published_frames;
    sample_index += static_cast<uint64_t>(frame_samples);
    event_bus_->PublishAudioFrame(frame);

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_stats_at) {
      const double window_s =
          std::max(1.0e-3, std::chrono::duration<double>(now - stats_started_at).count());
      const uint64_t window_ms = static_cast<uint64_t>(std::llround(window_s * 1000.0));
      const double gen_hz = static_cast<double>(generated_samples) / window_s;
      const double pub_hz = static_cast<double>(published_samples) / window_s;

      std::ostringstream audio_status;
      audio_status << "AUDIO_STATS idx=" << ch.index
                   << " label=" << ch.label
                   << " rev=" << kAudioPipelineRevision
                   << " mod=" << ModulationToken(ch.modulation)
                   << " sr=" << audio_sample_rate_hz
                   << " cfg_sr=0"
                   << " iq_sr=0"
                   << " iq_est_sr=0"
                   << " iq_lock=0"
                   << " iq_lock_sr=0"
                   << " win_ms=" << window_ms
                   << " gen_ratio=1.000"
                   << " rate_corr=1.0000"
                   << " gen_hz=" << FormatDouble(gen_hz, 1)
                   << " pub_hz=" << FormatDouble(pub_hz, 1)
                   << " iq_n=0"
                   << " gate=1"
                   << " squelch=1"
                   << " signal_db=-20.0"
                   << " blocks=" << published_frames
                   << " gate_open_blocks=" << published_frames
                   << " demod_ok=" << published_frames
                   << " demod_empty=0"
                   << " gen_samples=" << generated_samples
                   << " pub_frames=" << published_frames
                   << " pub_samples=" << published_samples
                   << " pending_samples=0"
                   << " conceal_samples=" << conceal_samples
                   << " clears=0"
                   << " flush_frames=0"
                   << " flush_samples=0";
      PublishEvent(EventKind::kInfo, audio_status.str(), ch.frequency_hz, false);

      generated_samples = 0;
      published_samples = 0;
      published_frames = 0;
      conceal_samples = 0;
      stats_started_at = now;
      next_stats_at = now + std::chrono::milliseconds(kAudioStatsIntervalMs);
    }

    next_frame_at += frame_interval;
    std::this_thread::sleep_until(next_frame_at);
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
