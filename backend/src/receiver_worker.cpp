#include "multi_radio/receiver_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

namespace multi_radio {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kWaveformPoints = 200;
constexpr int kSpectrumBins = 256;

std::string FormatDouble(double value, int precision) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

std::string FormatSeries(const std::vector<double>& values, int precision) {
  std::ostringstream out;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << std::fixed << std::setprecision(precision) << values[i];
  }
  return out.str();
}

std::vector<double> BuildFmDiscriminator(const IQSampleBlock& iq) {
  std::vector<double> discriminator;
  if (iq.sample_rate_hz == 0 || iq.interleaved_iq.size() < 4) {
    return discriminator;
  }

  const size_t available_samples = iq.interleaved_iq.size() / 2;
  const size_t max_samples = 8192;
  const size_t sample_count = std::min(available_samples, max_samples);
  if (sample_count < 64) {
    return discriminator;
  }

  discriminator.reserve(sample_count - 1);
  double prev_i = static_cast<double>(iq.interleaved_iq[0]);
  double prev_q = static_cast<double>(iq.interleaved_iq[1]);
  for (size_t n = 1; n < sample_count; ++n) {
    const double cur_i = static_cast<double>(iq.interleaved_iq[n * 2]);
    const double cur_q = static_cast<double>(iq.interleaved_iq[n * 2 + 1]);
    const double cross = prev_i * cur_q - prev_q * cur_i;
    const double dot = prev_i * cur_i + prev_q * cur_q;
    discriminator.push_back(std::atan2(cross, dot));
    prev_i = cur_i;
    prev_q = cur_q;
  }

  if (discriminator.empty()) {
    return discriminator;
  }

  double mean = 0.0;
  for (double value : discriminator) {
    mean += value;
  }
  mean /= static_cast<double>(discriminator.size());
  for (double& value : discriminator) {
    value -= mean;
  }
  return discriminator;
}

bool BuildAudioVisualizationFrame(const IQSampleBlock& iq, std::vector<double>* waveform,
                                  std::vector<double>* spectrum, double* peak_hz,
                                  double* peak_strength) {
  if (waveform == nullptr || spectrum == nullptr || peak_hz == nullptr || peak_strength == nullptr) {
    return false;
  }

  const std::vector<double> discriminator = BuildFmDiscriminator(iq);
  if (discriminator.size() < 32) {
    return false;
  }

  waveform->assign(kWaveformPoints, 0.5);
  for (int i = 0; i < kWaveformPoints; ++i) {
    const size_t idx = static_cast<size_t>((static_cast<double>(i) * (discriminator.size() - 1)) /
                                           static_cast<double>(kWaveformPoints - 1));
    const double raw = discriminator[idx];
    const double normalized = std::clamp((raw / 1.25 + 1.0) * 0.5, 0.0, 1.0);
    (*waveform)[static_cast<size_t>(i)] = normalized;
  }

  const double sample_rate_hz = static_cast<double>(iq.sample_rate_hz);
  const double nyquist_hz = sample_rate_hz * 0.5;
  const double min_hz = 0.0;
  const double max_hz = std::min(20000.0, nyquist_hz - 1.0);
  if (max_hz <= min_hz + 1.0) {
    return false;
  }

  spectrum->assign(kSpectrumBins, 0.0);
  double max_power = 0.0;
  int peak_bin = 0;
  for (int bin = 0; bin < kSpectrumBins; ++bin) {
    const double t = (kSpectrumBins <= 1) ? 0.0 : static_cast<double>(bin) / static_cast<double>(kSpectrumBins - 1);
    const double candidate_hz = min_hz + t * (max_hz - min_hz);
    const double omega = 2.0 * kPi * candidate_hz / sample_rate_hz;
    const double coeff = 2.0 * std::cos(omega);
    double q0 = 0.0;
    double q1 = 0.0;
    double q2 = 0.0;
    for (double sample : discriminator) {
      q0 = coeff * q1 - q2 + sample;
      q2 = q1;
      q1 = q0;
    }
    const double power = q1 * q1 + q2 * q2 - coeff * q1 * q2;
    (*spectrum)[static_cast<size_t>(bin)] = std::max(0.0, power);
    if (power > max_power) {
      max_power = power;
      peak_bin = bin;
    }
  }

  if (max_power <= 0.0) {
    return false;
  }

  for (double& value : *spectrum) {
    value = std::clamp(value / max_power, 0.0, 1.0);
  }

  const double peak_t = (kSpectrumBins <= 1) ? 0.0 : static_cast<double>(peak_bin) / static_cast<double>(kSpectrumBins - 1);
  *peak_hz = min_hz + peak_t * (max_hz - min_hz);
  *peak_strength = (*spectrum)[static_cast<size_t>(peak_bin)];
  return true;
}

}  // namespace

ReceiverWorker::ReceiverWorker(uint32_t receiver_id, std::string serial,
                               std::unique_ptr<IRadioDevice> device,
                               std::shared_ptr<EventBus> event_bus,
                               std::shared_ptr<PluginHost> plugin_host,
                               std::shared_ptr<JsonlLogger> logger)
    : receiver_id_(receiver_id),
      serial_(std::move(serial)),
      device_(std::move(device)),
      event_bus_(std::move(event_bus)),
      plugin_host_(std::move(plugin_host)),
      logger_(std::move(logger)) {
  mode_config_.fixed_frequency_hz = 162025000.0;
  mode_config_.dwell_ms = 500;
  scheduler_.Configure(mode_, mode_config_);
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

  if (!device_->Open(error)) {
    running_.store(false);
    return false;
  }

  if (!device_->SetSampleRateHz(2048000, error)) {
    device_->Close();
    running_.store(false);
    return false;
  }

  thread_ = std::thread([this]() { RunLoop(); });
  PublishEvent(EventKind::kStateChange, "receiver started");
  return true;
}

bool ReceiverWorker::Stop(std::string* error) {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) {
    return true;
  }

  if (thread_.joinable()) {
    thread_.join();
  }
  device_->Close();
  PublishEvent(EventKind::kStateChange, "receiver stopped");
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool ReceiverWorker::SetMode(RadioMode mode, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  mode_ = mode;
  scheduler_.Configure(mode_, mode_config_);
  if (error != nullptr) {
    error->clear();
  }

  std::ostringstream msg;
  msg << "mode changed to " << ToString(mode_);
  PublishEvent(EventKind::kStateChange, msg.str());
  return true;
}

bool ReceiverWorker::SetModeConfig(const ModeConfig& config, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  mode_config_ = config;
  scheduler_.Configure(mode_, mode_config_);
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
  while (running_.load()) {
    std::optional<double> frequency_hz;
    uint32_t dwell_ms = 500;
    {
      std::lock_guard<std::mutex> lock(mu_);
      frequency_hz = scheduler_.NextFrequencyHz();
      dwell_ms = scheduler_.DwellMs();
    }

    if (!frequency_hz.has_value()) {
      PublishEvent(EventKind::kWarning, "no active frequencies in current mode");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    std::string error;
    const auto tune_hz = static_cast<uint32_t>(frequency_hz.value());
    if (!device_->SetCenterFrequencyHz(tune_hz, &error)) {
      {
        std::lock_guard<std::mutex> lock(mu_);
        last_error_ = error;
      }
      PublishEvent(EventKind::kError, "tune failed: " + error, frequency_hz.value());
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    PublishEvent(EventKind::kTuneHop, "tuned", frequency_hz.value());

    IQSampleBlock iq;
    if (!device_->ReadIq(&iq, &error)) {
      {
        std::lock_guard<std::mutex> lock(mu_);
        last_error_ = error;
      }
      PublishEvent(EventKind::kError, "read failed: " + error, frequency_hz.value());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    double audio_peak_hz = 0.0;
    double audio_peak_strength = 0.0;
    std::vector<double> audio_waveform;
    std::vector<double> audio_spectrum;
    const bool have_audio_frame = BuildAudioVisualizationFrame(
        iq, &audio_waveform, &audio_spectrum, &audio_peak_hz, &audio_peak_strength);
    if (have_audio_frame) {
      std::ostringstream viz_message;
      viz_message << "VIZ_FRAME peak_hz=" << FormatDouble(audio_peak_hz, 1)
                  << " peak_strength=" << FormatDouble(audio_peak_strength, 3)
                  << " waveform=" << FormatSeries(audio_waveform, 4)
                  << " spectrum=" << FormatSeries(audio_spectrum, 4);
      PublishEvent(EventKind::kInfo, viz_message.str(), frequency_hz.value(), false);
    }

    plugin_host_->ProcessIq(iq, [&](const PluginMessage& plugin_msg) {
      DecodedMessage msg;
      msg.unix_ms = plugin_msg.unix_ms == 0 ? UnixMillisNow() : plugin_msg.unix_ms;
      msg.receiver_id = receiver_id_;
      msg.signal_type = plugin_msg.signal_type;
      msg.frequency_hz = plugin_msg.frequency_hz == 0.0 ? frequency_hz.value() : plugin_msg.frequency_hz;
      msg.payload = plugin_msg.payload;
      msg.normalized_fields = plugin_msg.normalized_fields;
      if (have_audio_frame) {
        msg.normalized_fields["audio_peak_hz"] = FormatDouble(audio_peak_hz, 1);
        msg.normalized_fields["audio_peak_strength"] = FormatDouble(audio_peak_strength, 3);
      }
      event_bus_->PublishDecodedMessage(msg);
      logger_->LogDecodedMessage(msg);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(dwell_ms));
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
  if (log_event) {
    logger_->LogEvent(event);
  }
}

}  // namespace multi_radio
