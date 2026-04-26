#include "multi_radio/receiver_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

namespace multi_radio {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kWaveformPoints = 200;
constexpr int kSpectrumBins = 256;
constexpr uint32_t kVisualizationFrameIntervalMs = 50;
constexpr uint32_t kDefaultSampleRateHz = 2048000;
constexpr uint32_t kMinSampleRateHz = 225000;
constexpr uint32_t kMaxSampleRateHz = 3200000;
constexpr uint32_t kDefaultChannelBandwidthHz = 30000;
constexpr uint32_t kMinChannelBandwidthHz = 2000;
constexpr uint32_t kMaxChannelBandwidthHz = 500000;
constexpr uint32_t kMinHardwareBandwidthHz = 1000;
constexpr uint32_t kDefaultDcBlockerCutoffHz = 30;
constexpr uint32_t kMinDcBlockerCutoffHz = 1;
constexpr uint32_t kMaxDcBlockerCutoffHz = 5000;
constexpr uint32_t kDefaultCenterNotchWidthHz = 2000;
constexpr uint32_t kMinCenterNotchWidthHz = 100;
constexpr uint32_t kMaxCenterNotchWidthHz = 200000;

struct IqLowPassState {
  double i = 0.0;
  double q = 0.0;
  bool initialized = false;
};

struct IqFrequencyShiftState {
  double phase_rad = 0.0;
};

struct IqDcBlockState {
  bool initialized = false;
  double prev_in_i = 0.0;
  double prev_in_q = 0.0;
  double prev_out_i = 0.0;
  double prev_out_q = 0.0;
};

struct IqCenterNotchState {
  double i = 0.0;
  double q = 0.0;
  bool initialized = false;
};

ModeConfig NormalizeModeConfig(const ModeConfig& input) {
  ModeConfig out = input;
  if (out.dwell_ms == 0) {
    out.dwell_ms = 500;
  }
  if (out.sample_rate_hz == 0) {
    out.sample_rate_hz = kDefaultSampleRateHz;
  }
  out.sample_rate_hz = std::clamp(out.sample_rate_hz, kMinSampleRateHz, kMaxSampleRateHz);
  if (out.channel_bandwidth_hz != 0) {
    out.channel_bandwidth_hz =
        std::clamp(out.channel_bandwidth_hz, kMinChannelBandwidthHz, kMaxChannelBandwidthHz);
  }
  if (out.hardware_bandwidth_hz != 0) {
    out.hardware_bandwidth_hz =
        std::clamp(out.hardware_bandwidth_hz, kMinHardwareBandwidthHz, out.sample_rate_hz);
  }
  if (out.dc_blocker_enabled) {
    if (out.dc_blocker_cutoff_hz == 0) {
      out.dc_blocker_cutoff_hz = kDefaultDcBlockerCutoffHz;
    }
    out.dc_blocker_cutoff_hz =
        std::clamp(out.dc_blocker_cutoff_hz, kMinDcBlockerCutoffHz, kMaxDcBlockerCutoffHz);
  } else {
    out.dc_blocker_cutoff_hz = kDefaultDcBlockerCutoffHz;
  }
  if (out.center_notch_enabled) {
    if (out.center_notch_width_hz == 0) {
      out.center_notch_width_hz = kDefaultCenterNotchWidthHz;
    }
    const uint32_t max_notch =
        std::min(kMaxCenterNotchWidthHz, std::max<uint32_t>(kMinCenterNotchWidthHz, out.sample_rate_hz / 2));
    out.center_notch_width_hz =
        std::clamp(out.center_notch_width_hz, kMinCenterNotchWidthHz, max_notch);
  } else {
    out.center_notch_width_hz = kDefaultCenterNotchWidthHz;
  }
  if (!out.lo_offset_enabled) {
    out.lo_offset_hz = 0;
  } else {
    const int32_t max_offset =
        std::max<int32_t>(1000, static_cast<int32_t>(static_cast<double>(out.sample_rate_hz) * 0.45));
    out.lo_offset_hz = std::clamp(out.lo_offset_hz, -max_offset, max_offset);
  }
  return out;
}

void ApplyIqFrequencyShift(IQSampleBlock* iq, int32_t frequency_shift_hz, IqFrequencyShiftState* state) {
  if (iq == nullptr || state == nullptr || iq->sample_rate_hz == 0 || iq->interleaved_iq.size() < 4) {
    return;
  }
  if (frequency_shift_hz == 0) {
    state->phase_rad = 0.0;
    return;
  }

  const double omega = (-2.0 * kPi * static_cast<double>(frequency_shift_hz)) /
                       static_cast<double>(iq->sample_rate_hz);
  for (size_t idx = 0; idx + 1 < iq->interleaved_iq.size(); idx += 2) {
    const double in_i = static_cast<double>(iq->interleaved_iq[idx]);
    const double in_q = static_cast<double>(iq->interleaved_iq[idx + 1]);
    const double phase = state->phase_rad;
    const double cos_phase = std::cos(phase);
    const double sin_phase = std::sin(phase);
    const double out_i = in_i * cos_phase - in_q * sin_phase;
    const double out_q = in_i * sin_phase + in_q * cos_phase;
    iq->interleaved_iq[idx] = static_cast<int16_t>(std::lrint(std::clamp(out_i, -32768.0, 32767.0)));
    iq->interleaved_iq[idx + 1] = static_cast<int16_t>(std::lrint(std::clamp(out_q, -32768.0, 32767.0)));
    state->phase_rad += omega;
    if (state->phase_rad > kPi) {
      state->phase_rad -= 2.0 * kPi;
    } else if (state->phase_rad < -kPi) {
      state->phase_rad += 2.0 * kPi;
    }
  }
}

void ApplyIqDcBlocker(IQSampleBlock* iq, uint32_t cutoff_hz, IqDcBlockState* state) {
  if (iq == nullptr || state == nullptr || iq->sample_rate_hz == 0 || iq->interleaved_iq.size() < 4) {
    return;
  }
  if (cutoff_hz == 0) {
    state->initialized = false;
    return;
  }

  const double alpha = std::clamp(
      std::exp((-2.0 * kPi * static_cast<double>(cutoff_hz)) / static_cast<double>(iq->sample_rate_hz)), 0.0, 0.9999);
  for (size_t idx = 0; idx + 1 < iq->interleaved_iq.size(); idx += 2) {
    const double in_i = static_cast<double>(iq->interleaved_iq[idx]);
    const double in_q = static_cast<double>(iq->interleaved_iq[idx + 1]);
    if (!state->initialized) {
      state->prev_in_i = in_i;
      state->prev_in_q = in_q;
      state->prev_out_i = 0.0;
      state->prev_out_q = 0.0;
      state->initialized = true;
    }
    const double out_i = in_i - state->prev_in_i + alpha * state->prev_out_i;
    const double out_q = in_q - state->prev_in_q + alpha * state->prev_out_q;
    state->prev_in_i = in_i;
    state->prev_in_q = in_q;
    state->prev_out_i = out_i;
    state->prev_out_q = out_q;
    iq->interleaved_iq[idx] = static_cast<int16_t>(std::lrint(std::clamp(out_i, -32768.0, 32767.0)));
    iq->interleaved_iq[idx + 1] = static_cast<int16_t>(std::lrint(std::clamp(out_q, -32768.0, 32767.0)));
  }
}

void ApplyIqCenterNotch(IQSampleBlock* iq, uint32_t notch_width_hz, IqCenterNotchState* state) {
  if (iq == nullptr || state == nullptr || iq->sample_rate_hz == 0 || iq->interleaved_iq.size() < 4) {
    return;
  }
  if (notch_width_hz == 0) {
    state->initialized = false;
    return;
  }

  const double nyquist_hz = static_cast<double>(iq->sample_rate_hz) * 0.5;
  const double cutoff_hz =
      std::clamp(static_cast<double>(notch_width_hz) * 0.5, 1.0, std::max(1.0, nyquist_hz - 1.0));
  const double alpha = std::clamp(
      1.0 - std::exp((-2.0 * kPi * cutoff_hz) / static_cast<double>(iq->sample_rate_hz)), 0.0001, 1.0);

  for (size_t idx = 0; idx + 1 < iq->interleaved_iq.size(); idx += 2) {
    const double in_i = static_cast<double>(iq->interleaved_iq[idx]);
    const double in_q = static_cast<double>(iq->interleaved_iq[idx + 1]);
    if (!state->initialized) {
      state->i = in_i;
      state->q = in_q;
      state->initialized = true;
    } else {
      state->i += alpha * (in_i - state->i);
      state->q += alpha * (in_q - state->q);
    }
    const double out_i = std::clamp(in_i - state->i, -32768.0, 32767.0);
    const double out_q = std::clamp(in_q - state->q, -32768.0, 32767.0);
    iq->interleaved_iq[idx] = static_cast<int16_t>(std::lrint(out_i));
    iq->interleaved_iq[idx + 1] = static_cast<int16_t>(std::lrint(out_q));
  }
}

void ApplyIqChannelBandwidth(IQSampleBlock* iq, uint32_t channel_bandwidth_hz, IqLowPassState* state) {
  if (iq == nullptr || state == nullptr || iq->sample_rate_hz == 0 || iq->interleaved_iq.size() < 4) {
    return;
  }
  if (channel_bandwidth_hz == 0) {
    state->initialized = false;
    return;
  }

  const double nyquist_hz = static_cast<double>(iq->sample_rate_hz) * 0.5;
  const double cutoff_hz =
      std::clamp(static_cast<double>(channel_bandwidth_hz) * 0.5, 1.0, nyquist_hz - 1.0);
  if (cutoff_hz <= 1.0 || nyquist_hz <= 1.0) {
    return;
  }

  const double alpha = std::clamp(1.0 - std::exp((-2.0 * kPi * cutoff_hz) /
                                                  static_cast<double>(iq->sample_rate_hz)),
                                  0.0001, 1.0);
  for (size_t idx = 0; idx + 1 < iq->interleaved_iq.size(); idx += 2) {
    const double in_i = static_cast<double>(iq->interleaved_iq[idx]);
    const double in_q = static_cast<double>(iq->interleaved_iq[idx + 1]);
    if (!state->initialized) {
      state->i = in_i;
      state->q = in_q;
      state->initialized = true;
    } else {
      state->i += alpha * (in_i - state->i);
      state->q += alpha * (in_q - state->q);
    }
    const double out_i = std::clamp(state->i, -32768.0, 32767.0);
    const double out_q = std::clamp(state->q, -32768.0, 32767.0);
    iq->interleaved_iq[idx] = static_cast<int16_t>(std::lrint(out_i));
    iq->interleaved_iq[idx + 1] = static_cast<int16_t>(std::lrint(out_q));
  }
}

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

bool BuildDemodVisualizationFrame(const IQSampleBlock& iq, std::vector<double>* waveform,
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

bool BuildReceiverSpectrumFrame(const IQSampleBlock& iq, double tuned_frequency_hz,
                                uint32_t channel_bandwidth_hz, std::vector<double>* waveform,
                                std::vector<double>* spectrum, double* peak_hz, double* peak_strength,
                                double* start_hz, double* end_hz) {
  if (waveform == nullptr || spectrum == nullptr || peak_hz == nullptr || peak_strength == nullptr ||
      start_hz == nullptr || end_hz == nullptr) {
    return false;
  }
  if (iq.sample_rate_hz == 0 || iq.interleaved_iq.size() < 4) {
    return false;
  }

  const size_t available_samples = iq.interleaved_iq.size() / 2;
  const size_t sample_count = std::min(available_samples, static_cast<size_t>(4096));
  if (sample_count < 64) {
    return false;
  }

  const double sample_rate_hz = static_cast<double>(iq.sample_rate_hz);
  double half_span_hz = channel_bandwidth_hz > 0 ? static_cast<double>(channel_bandwidth_hz)
                                                  : sample_rate_hz * 0.5;
  half_span_hz = std::clamp(half_span_hz, 1000.0, std::max(1000.0, sample_rate_hz * 0.5 - 1.0));
  const double min_offset_hz = -half_span_hz;
  const double max_offset_hz = half_span_hz;
  *start_hz = tuned_frequency_hz + min_offset_hz;
  *end_hz = tuned_frequency_hz + max_offset_hz;

  spectrum->assign(kSpectrumBins, 0.0);
  double max_power = 0.0;
  int peak_bin = 0;
  for (int bin = 0; bin < kSpectrumBins; ++bin) {
    const double t = (kSpectrumBins <= 1) ? 0.0
                                          : static_cast<double>(bin) /
                                                static_cast<double>(kSpectrumBins - 1);
    const double candidate_offset_hz = min_offset_hz + t * (max_offset_hz - min_offset_hz);
    const double omega = 2.0 * kPi * candidate_offset_hz / sample_rate_hz;
    const double cos_step = std::cos(omega);
    const double sin_step = std::sin(omega);
    double osc_i = 1.0;
    double osc_q = 0.0;
    double sum_re = 0.0;
    double sum_im = 0.0;
    for (size_t n = 0; n < sample_count; ++n) {
      const double in_i = static_cast<double>(iq.interleaved_iq[n * 2]);
      const double in_q = static_cast<double>(iq.interleaved_iq[n * 2 + 1]);
      sum_re += in_i * osc_i + in_q * osc_q;
      sum_im += in_q * osc_i - in_i * osc_q;
      const double next_osc_i = osc_i * cos_step - osc_q * sin_step;
      const double next_osc_q = osc_i * sin_step + osc_q * cos_step;
      osc_i = next_osc_i;
      osc_q = next_osc_q;
    }
    const double power = sum_re * sum_re + sum_im * sum_im;
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

  const double peak_t = (kSpectrumBins <= 1) ? 0.0
                                             : static_cast<double>(peak_bin) /
                                                   static_cast<double>(kSpectrumBins - 1);
  const double peak_offset_hz = min_offset_hz + peak_t * (max_offset_hz - min_offset_hz);
  *peak_hz = tuned_frequency_hz + peak_offset_hz;
  *peak_strength = (*spectrum)[static_cast<size_t>(peak_bin)];

  waveform->assign(kWaveformPoints, 0.5);
  double max_magnitude = 0.0;
  for (size_t n = 0; n < sample_count; ++n) {
    const double in_i = static_cast<double>(iq.interleaved_iq[n * 2]);
    const double in_q = static_cast<double>(iq.interleaved_iq[n * 2 + 1]);
    max_magnitude = std::max(max_magnitude, std::sqrt(in_i * in_i + in_q * in_q));
  }
  const double normalize = max_magnitude > 1e-6 ? max_magnitude : 1.0;
  for (int i = 0; i < kWaveformPoints; ++i) {
    const size_t idx = static_cast<size_t>((static_cast<double>(i) * (sample_count - 1)) /
                                           static_cast<double>(kWaveformPoints - 1));
    const double in_i = static_cast<double>(iq.interleaved_iq[idx * 2]);
    const double in_q = static_cast<double>(iq.interleaved_iq[idx * 2 + 1]);
    const double magnitude = std::sqrt(in_i * in_i + in_q * in_q) / normalize;
    (*waveform)[static_cast<size_t>(i)] = std::clamp(magnitude, 0.0, 1.0);
  }
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
  mode_config_.sample_rate_hz = kDefaultSampleRateHz;
  mode_config_.channel_bandwidth_hz = kDefaultChannelBandwidthHz;
  mode_config_.hardware_bandwidth_hz = 0;
  mode_config_.ais_autotune_enabled = false;
  mode_config_.ais_baud_trim_enabled = false;
  mode_config_.dc_blocker_enabled = false;
  mode_config_.dc_blocker_cutoff_hz = kDefaultDcBlockerCutoffHz;
  mode_config_.center_notch_enabled = false;
  mode_config_.center_notch_width_hz = kDefaultCenterNotchWidthHz;
  mode_config_.lo_offset_enabled = false;
  mode_config_.lo_offset_hz = 0;
  mode_config_ = NormalizeModeConfig(mode_config_);
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

  uint32_t sample_rate_hz = kDefaultSampleRateHz;
  {
    std::lock_guard<std::mutex> lock(mu_);
    mode_config_ = NormalizeModeConfig(mode_config_);
    sample_rate_hz = mode_config_.sample_rate_hz;
  }

  if (!device_->SetSampleRateHz(sample_rate_hz, error)) {
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
  mode_config_ = NormalizeModeConfig(config);
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
  uint32_t applied_sample_rate_hz = 0;
  uint32_t applied_hardware_bandwidth_hz = std::numeric_limits<uint32_t>::max();
  IqLowPassState lowpass_state;
  IqFrequencyShiftState frequency_shift_state;
  IqDcBlockState dc_block_state;
  IqCenterNotchState center_notch_state;

  while (running_.load()) {
    std::optional<double> frequency_hz;
    uint32_t dwell_ms = 500;
    uint32_t desired_sample_rate_hz = kDefaultSampleRateHz;
    uint32_t channel_bandwidth_hz = kDefaultChannelBandwidthHz;
    uint32_t desired_hardware_bandwidth_hz = 0;
    bool ais_autotune_enabled = false;
    bool ais_baud_trim_enabled = false;
    bool dc_blocker_enabled = false;
    uint32_t dc_blocker_cutoff_hz = kDefaultDcBlockerCutoffHz;
    bool center_notch_enabled = false;
    uint32_t center_notch_width_hz = kDefaultCenterNotchWidthHz;
    bool lo_offset_enabled = false;
    int32_t lo_offset_hz = 0;
    {
      std::lock_guard<std::mutex> lock(mu_);
      mode_config_ = NormalizeModeConfig(mode_config_);
      frequency_hz = scheduler_.NextFrequencyHz();
      dwell_ms = scheduler_.DwellMs();
      desired_sample_rate_hz = mode_config_.sample_rate_hz;
      channel_bandwidth_hz = mode_config_.channel_bandwidth_hz;
      desired_hardware_bandwidth_hz = mode_config_.hardware_bandwidth_hz;
      ais_autotune_enabled = mode_config_.ais_autotune_enabled;
      ais_baud_trim_enabled = mode_config_.ais_baud_trim_enabled;
      dc_blocker_enabled = mode_config_.dc_blocker_enabled;
      dc_blocker_cutoff_hz = mode_config_.dc_blocker_cutoff_hz;
      center_notch_enabled = mode_config_.center_notch_enabled;
      center_notch_width_hz = mode_config_.center_notch_width_hz;
      lo_offset_enabled = mode_config_.lo_offset_enabled;
      lo_offset_hz = mode_config_.lo_offset_hz;
    }

    std::string error;
    if (desired_sample_rate_hz != applied_sample_rate_hz) {
      if (!device_->SetSampleRateHz(desired_sample_rate_hz, &error)) {
        {
          std::lock_guard<std::mutex> lock(mu_);
          last_error_ = error;
        }
        std::ostringstream msg;
        msg << "sample-rate update failed (" << desired_sample_rate_hz << " Hz): " << error;
        PublishEvent(EventKind::kError, msg.str());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }
      applied_sample_rate_hz = desired_sample_rate_hz;
      lowpass_state.initialized = false;
      dc_block_state.initialized = false;
      center_notch_state.initialized = false;
      frequency_shift_state.phase_rad = 0.0;
      std::ostringstream msg;
      msg << "sample-rate updated to " << desired_sample_rate_hz << " Hz";
      PublishEvent(EventKind::kInfo, msg.str());
    }
    if (desired_hardware_bandwidth_hz != applied_hardware_bandwidth_hz) {
      if (!device_->SetHardwareBandwidthHz(desired_hardware_bandwidth_hz, &error)) {
        {
          std::lock_guard<std::mutex> lock(mu_);
          last_error_ = error;
        }
        std::ostringstream msg;
        msg << "hardware-bandwidth update failed (" << desired_hardware_bandwidth_hz
            << " Hz): " << error;
        PublishEvent(EventKind::kError, msg.str());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }
      applied_hardware_bandwidth_hz = desired_hardware_bandwidth_hz;
      std::ostringstream msg;
      msg << "hardware-bandwidth updated to " << desired_hardware_bandwidth_hz << " Hz";
      PublishEvent(EventKind::kInfo, msg.str());
    }

    if (!frequency_hz.has_value()) {
      PublishEvent(EventKind::kWarning, "no active frequencies in current mode");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    const double logical_tuned_frequency_hz = frequency_hz.value();
    int64_t requested_tune_hz = static_cast<int64_t>(std::llround(logical_tuned_frequency_hz));
    if (lo_offset_enabled) {
      requested_tune_hz += static_cast<int64_t>(lo_offset_hz);
    }
    const bool tune_hz_in_range = requested_tune_hz > 0 &&
                                  requested_tune_hz <= static_cast<int64_t>(std::numeric_limits<uint32_t>::max());
    if (!tune_hz_in_range) {
      requested_tune_hz = static_cast<int64_t>(std::llround(logical_tuned_frequency_hz));
    }
    const int32_t effective_lo_offset_hz = (lo_offset_enabled && tune_hz_in_range) ? lo_offset_hz : 0;
    const auto tune_hz = static_cast<uint32_t>(requested_tune_hz);
    if (!device_->SetCenterFrequencyHz(tune_hz, &error)) {
      {
        std::lock_guard<std::mutex> lock(mu_);
        last_error_ = error;
      }
      PublishEvent(EventKind::kError, "tune failed: " + error, logical_tuned_frequency_hz);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    if (lo_offset_enabled && tune_hz_in_range) {
      std::ostringstream msg;
      msg << "tuned with LO offset " << lo_offset_hz << " Hz (hw=" << tune_hz
          << " Hz, target=" << static_cast<uint32_t>(std::llround(logical_tuned_frequency_hz))
          << " Hz)";
      PublishEvent(EventKind::kInfo, msg.str(), logical_tuned_frequency_hz, false);
    } else if (lo_offset_enabled && !tune_hz_in_range) {
      std::ostringstream msg;
      msg << "LO offset disabled for hop: target "
          << static_cast<uint32_t>(std::llround(logical_tuned_frequency_hz))
          << " Hz with offset " << lo_offset_hz << " Hz is out of range";
      PublishEvent(EventKind::kWarning, msg.str(), logical_tuned_frequency_hz, false);
    }

    PublishEvent(EventKind::kTuneHop, "tuned", logical_tuned_frequency_hz);
    lowpass_state.initialized = false;
    dc_block_state.initialized = false;
    center_notch_state.initialized = false;
    frequency_shift_state.phase_rad = 0.0;

    const double tuned_frequency_hz = logical_tuned_frequency_hz;
    const auto dwell_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(dwell_ms);
    auto next_viz_emit = std::chrono::steady_clock::time_point::min();

    while (running_.load() && std::chrono::steady_clock::now() < dwell_deadline) {
      IQSampleBlock iq;
      if (!device_->ReadIq(&iq, &error)) {
        {
          std::lock_guard<std::mutex> lock(mu_);
          last_error_ = error;
        }
        PublishEvent(EventKind::kError, "read failed: " + error, tuned_frequency_hz);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
      }
      if (effective_lo_offset_hz != 0) {
        ApplyIqFrequencyShift(&iq, effective_lo_offset_hz, &frequency_shift_state);
      } else {
        frequency_shift_state.phase_rad = 0.0;
      }
      if (dc_blocker_enabled) {
        ApplyIqDcBlocker(&iq, dc_blocker_cutoff_hz, &dc_block_state);
      } else {
        dc_block_state.initialized = false;
      }
      if (center_notch_enabled) {
        ApplyIqCenterNotch(&iq, center_notch_width_hz, &center_notch_state);
      } else {
        center_notch_state.initialized = false;
      }
      iq.center_frequency_hz = static_cast<uint32_t>(std::llround(logical_tuned_frequency_hz));

      double receiver_peak_hz = 0.0;
      double receiver_peak_strength = 0.0;
      double receiver_start_hz = 0.0;
      double receiver_end_hz = 0.0;
      std::vector<double> receiver_waveform;
      std::vector<double> receiver_spectrum;
      const bool have_receiver_frame =
          BuildReceiverSpectrumFrame(iq, tuned_frequency_hz, channel_bandwidth_hz, &receiver_waveform,
                                     &receiver_spectrum, &receiver_peak_hz, &receiver_peak_strength,
                                     &receiver_start_hz, &receiver_end_hz);

      iq.ais_autotune_enabled = ais_autotune_enabled;
      iq.ais_baud_trim_enabled = ais_baud_trim_enabled;
      ApplyIqChannelBandwidth(&iq, channel_bandwidth_hz, &lowpass_state);

      double demod_peak_hz = 0.0;
      double demod_peak_strength = 0.0;
      std::vector<double> demod_waveform;
      std::vector<double> demod_spectrum;
      const bool have_demod_frame = BuildDemodVisualizationFrame(
          iq, &demod_waveform, &demod_spectrum, &demod_peak_hz, &demod_peak_strength);

      plugin_host_->ProcessIq(iq, [&](const PluginMessage& plugin_msg) {
        DecodedMessage msg;
        msg.unix_ms = plugin_msg.unix_ms == 0 ? UnixMillisNow() : plugin_msg.unix_ms;
        msg.receiver_id = receiver_id_;
        msg.signal_type = plugin_msg.signal_type;
        msg.frequency_hz = plugin_msg.frequency_hz == 0.0 ? tuned_frequency_hz : plugin_msg.frequency_hz;
        msg.payload = plugin_msg.payload;
        msg.normalized_fields = plugin_msg.normalized_fields;
        if (have_demod_frame) {
          msg.normalized_fields["demod_peak_hz"] = FormatDouble(demod_peak_hz, 1);
          msg.normalized_fields["demod_peak_strength"] = FormatDouble(demod_peak_strength, 3);
        }
        event_bus_->PublishDecodedMessage(msg);
        logger_->LogDecodedMessage(msg);
      });

      const auto now = std::chrono::steady_clock::now();
      if (now >= next_viz_emit) {
        if (have_demod_frame) {
          const double demod_start_hz = 0.0;
          const double demod_end_hz =
              std::max(1.0, std::min(20000.0, static_cast<double>(iq.sample_rate_hz) * 0.5 - 1.0));
          std::ostringstream viz_message;
          viz_message << "VIZ_FRAME source=demod start_hz=" << FormatDouble(demod_start_hz, 1)
                      << " end_hz=" << FormatDouble(demod_end_hz, 1)
                      << " peak_hz=" << FormatDouble(demod_peak_hz, 1)
                      << " peak_strength=" << FormatDouble(demod_peak_strength, 3)
                      << " waveform=" << FormatSeries(demod_waveform, 4)
                      << " spectrum=" << FormatSeries(demod_spectrum, 4);
          PublishEvent(EventKind::kInfo, viz_message.str(), tuned_frequency_hz, false);
        }
        if (have_receiver_frame) {
          std::ostringstream viz_message;
          viz_message << "VIZ_FRAME source=receiver start_hz=" << FormatDouble(receiver_start_hz, 1)
                      << " end_hz=" << FormatDouble(receiver_end_hz, 1)
                      << " peak_hz=" << FormatDouble(receiver_peak_hz, 1)
                      << " peak_strength=" << FormatDouble(receiver_peak_strength, 3)
                      << " waveform=" << FormatSeries(receiver_waveform, 4)
                      << " spectrum=" << FormatSeries(receiver_spectrum, 4);
          PublishEvent(EventKind::kInfo, viz_message.str(), tuned_frequency_hz, false);
        }
        next_viz_emit = now + std::chrono::milliseconds(kVisualizationFrameIntervalMs);
      }
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
  if (log_event) {
    logger_->LogEvent(event);
  }
}

}  // namespace multi_radio
