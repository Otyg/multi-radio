#include "multi_radio/receiver_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <optional>
#include <vector>

namespace multi_radio {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kWaveformPoints = 120;
constexpr int kSpectrumBins = 128;
constexpr uint32_t kVisualizationFrameIntervalMs = 100;
constexpr bool kBackendVisualizationEnabled = false;
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
constexpr uint32_t kDefaultNfmBandwidthHz = 12500;
constexpr uint32_t kDefaultWfmBandwidthHz = 180000;
constexpr uint32_t kDefaultAmBandwidthHz = 10000;
constexpr uint32_t kAudioFrameIntervalMs = 20;
constexpr size_t kMaxPacedAudioFramesPerLoop = 8;
// Favor stable, continuous scan audio over fidelity to reduce underruns/choppy
// stitched fragments when scan processing load spikes.
constexpr uint32_t kAudioSampleRateHz = 8000;
constexpr const char* kAudioPipelineRevision = "audio-v8-8k-throughput";
constexpr uint32_t kAudioStatsIntervalMs = 1000;
constexpr uint32_t kScanStatusIntervalMs = 250;
constexpr double kSquelchCloseHysteresisDb = 2.5;
constexpr uint32_t kSquelchCloseDebounceMs = 500;

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

struct FmDiscriminatorState {
  bool initialized = false;
  double prev_i = 0.0;
  double prev_q = 0.0;
};

struct CascadedLowPassState {
  std::vector<double> stage_values;
  bool initialized = false;
};

struct DeemphasisState {
  bool initialized = false;
  double value = 0.0;
};

struct AudioDcBlockState {
  bool initialized = false;
  double prev_in = 0.0;
  double prev_out = 0.0;
};

struct AudioResamplerState {
  double next_source_pos = 0.0;
  bool has_prev_sample = false;
  double prev_sample = 0.0;
};

struct AudioDemodState {
  FmDiscriminatorState fm_discriminator;
  CascadedLowPassState lowpass;
  DeemphasisState deemphasis;
  AudioDcBlockState dc_block;
  AudioResamplerState resampler;
};

std::vector<double> BuildFmDiscriminator(const IQSampleBlock& iq, size_t max_samples = 8192,
                                         FmDiscriminatorState* state = nullptr);
bool BuildAudioPcm16(const IQSampleBlock& iq, Modulation modulation, uint32_t audio_sample_rate_hz,
                     AudioDemodState* state, std::vector<int16_t>* pcm_out);
bool BuildDemodVisualizationFrame(const IQSampleBlock& iq, Modulation modulation,
                                  std::vector<double>* waveform, std::vector<double>* spectrum,
                                  double* peak_hz, double* peak_strength);

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

size_t AudioFrameSamplesForRate(uint32_t audio_sample_rate_hz) {
  if (audio_sample_rate_hz == 0) {
    return 0;
  }
  return (static_cast<size_t>(audio_sample_rate_hz) * static_cast<size_t>(kAudioFrameIntervalMs)) /
         1000U;
}

void ApplyCascadedLowPass(std::vector<double>* samples, double sample_rate_hz, double cutoff_hz,
                          int stages, CascadedLowPassState* state = nullptr) {
  if (samples == nullptr || samples->empty() || sample_rate_hz <= 0.0 || cutoff_hz <= 0.0 || stages <= 0) {
    return;
  }
  const double nyquist_hz = sample_rate_hz * 0.5;
  const double bounded_cutoff_hz =
      std::clamp(cutoff_hz, 5.0, std::max(5.0, nyquist_hz - 1.0));
  const double alpha = std::clamp(1.0 - std::exp((-2.0 * kPi * bounded_cutoff_hz) / sample_rate_hz),
                                  1.0e-6, 1.0);

  if (state != nullptr) {
    if (state->stage_values.size() != static_cast<size_t>(stages)) {
      state->stage_values.assign(static_cast<size_t>(stages), (*samples)[0]);
      state->initialized = false;
    }
    for (int stage = 0; stage < stages; ++stage) {
      double stage_state = (!state->initialized || !std::isfinite(state->stage_values[static_cast<size_t>(stage)]))
                               ? (*samples)[0]
                               : state->stage_values[static_cast<size_t>(stage)];
      for (size_t idx = 0; idx < samples->size(); ++idx) {
        stage_state += alpha * ((*samples)[idx] - stage_state);
        (*samples)[idx] = stage_state;
      }
      state->stage_values[static_cast<size_t>(stage)] = stage_state;
    }
    state->initialized = true;
    return;
  }

  for (int stage = 0; stage < stages; ++stage) {
    double state = (*samples)[0];
    for (size_t idx = 0; idx < samples->size(); ++idx) {
      state += alpha * ((*samples)[idx] - state);
      (*samples)[idx] = state;
    }
  }
}

void ApplyFmDeemphasis(std::vector<double>* samples, double sample_rate_hz, double tau_us,
                       DeemphasisState* state = nullptr) {
  if (samples == nullptr || samples->empty() || sample_rate_hz <= 0.0 || tau_us <= 0.0) {
    return;
  }
  const double tau_s = tau_us * 1.0e-6;
  const double alpha = std::clamp(1.0 - std::exp(-1.0 / (sample_rate_hz * tau_s)), 1.0e-6, 1.0);

  if (state != nullptr) {
    if (!state->initialized || !std::isfinite(state->value)) {
      state->value = (*samples)[0];
      state->initialized = true;
    }
    for (size_t idx = 0; idx < samples->size(); ++idx) {
      state->value += alpha * ((*samples)[idx] - state->value);
      (*samples)[idx] = state->value;
    }
    return;
  }

  double deemphasis_value = (*samples)[0];
  for (size_t idx = 0; idx < samples->size(); ++idx) {
    deemphasis_value += alpha * ((*samples)[idx] - deemphasis_value);
    (*samples)[idx] = deemphasis_value;
  }
}

void ApplyAudioDcBlock(std::vector<double>* samples, double sample_rate_hz, double cutoff_hz,
                       AudioDcBlockState* state) {
  if (samples == nullptr || samples->empty() || sample_rate_hz <= 0.0 || cutoff_hz <= 0.0 ||
      state == nullptr) {
    return;
  }

  const double alpha =
      std::clamp(std::exp((-2.0 * kPi * cutoff_hz) / sample_rate_hz), 0.0, 0.99999);
  for (double& sample : *samples) {
    if (!state->initialized || !std::isfinite(state->prev_in) || !std::isfinite(state->prev_out)) {
      state->prev_in = sample;
      state->prev_out = 0.0;
      state->initialized = true;
    }
    const double out = sample - state->prev_in + alpha * state->prev_out;
    state->prev_in = sample;
    state->prev_out = out;
    sample = out;
  }
}

uint32_t DefaultBandwidthForModulation(Modulation modulation) {
  switch (modulation) {
    case Modulation::kAm:
      return kDefaultAmBandwidthHz;
    case Modulation::kWfm:
      return kDefaultWfmBandwidthHz;
    case Modulation::kNfm:
    default:
      return kDefaultNfmBandwidthHz;
  }
}

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
  out.scan_list_default_squelch_db = std::clamp(out.scan_list_default_squelch_db, -120.0, 0.0);
  for (auto& channel : out.scan_list_channels) {
    if (channel.channel_bandwidth_hz == 0) {
      channel.channel_bandwidth_hz = DefaultBandwidthForModulation(channel.modulation);
    }
    channel.channel_bandwidth_hz =
        std::clamp(channel.channel_bandwidth_hz, kMinChannelBandwidthHz, kMaxChannelBandwidthHz);
    if (channel.use_default_squelch) {
      channel.squelch_threshold_db = out.scan_list_default_squelch_db;
    } else {
      channel.squelch_threshold_db = std::clamp(channel.squelch_threshold_db, -120.0, 0.0);
    }
    if (channel.dwell_ms > 0) {
      channel.dwell_ms = std::clamp<uint32_t>(channel.dwell_ms, 50, 60000);
    }
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

std::string SanitizeToken(std::string text) {
  if (text.empty()) {
    return "unnamed";
  }
  for (char& ch : text) {
    const unsigned char uch = static_cast<unsigned char>(ch);
    if (std::isspace(uch) != 0 || ch == '=' || ch == '"' || ch == '\'' || ch == ',') {
      ch = '_';
    }
  }
  return text;
}

double EstimateSignalDbfs(const IQSampleBlock& iq) {
  if (iq.interleaved_iq.empty()) {
    return -120.0;
  }
  const size_t sample_count = iq.interleaved_iq.size() / 2;
  if (sample_count == 0) {
    return -120.0;
  }
  long double sum_power = 0.0;
  for (size_t idx = 0; idx + 1 < iq.interleaved_iq.size(); idx += 2) {
    const long double i = static_cast<long double>(iq.interleaved_iq[idx]);
    const long double q = static_cast<long double>(iq.interleaved_iq[idx + 1]);
    sum_power += i * i + q * q;
  }
  const long double mean_power = sum_power / static_cast<long double>(sample_count);
  const long double full_scale_power = 32768.0L * 32768.0L;
  const long double normalized = mean_power / full_scale_power;
  const long double bounded = std::max<long double>(normalized, 1.0e-12L);
  return 10.0 * std::log10(static_cast<double>(bounded));
}

std::vector<double> BuildAmEnvelope(const IQSampleBlock& iq) {
  std::vector<double> envelope;
  const size_t sample_count = iq.interleaved_iq.size() / 2;
  if (sample_count < 32) {
    return envelope;
  }
  envelope.reserve(sample_count);
  double mean = 0.0;
  for (size_t n = 0; n < sample_count; ++n) {
    const double i = static_cast<double>(iq.interleaved_iq[n * 2]);
    const double q = static_cast<double>(iq.interleaved_iq[n * 2 + 1]);
    const double mag = std::sqrt(i * i + q * q) / 32768.0;
    envelope.push_back(mag);
    mean += mag;
  }
  mean /= static_cast<double>(envelope.size());
  for (double& sample : envelope) {
    sample -= mean;
  }
  return envelope;
}

bool BuildAudioPcm16(const IQSampleBlock& iq, Modulation modulation, uint32_t audio_sample_rate_hz,
                     AudioDemodState* state, std::vector<int16_t>* pcm_out) {
  if (pcm_out == nullptr) {
    return false;
  }
  pcm_out->clear();
  if (iq.sample_rate_hz == 0) {
    return false;
  }
  AudioDemodState local_state;
  if (state == nullptr) {
    state = &local_state;
  }
  const IQSampleBlock* demod_iq_ptr = &iq;
  IQSampleBlock demod_iq_decimated;
  // Reduce demod CPU load by decimating already channel-filtered IQ before audio
  // demodulation. This keeps audio timing math correct (ratio uses demod sample
  // rate) but helps the worker keep real-time pace.
  if (iq.sample_rate_hz >= 768000U && iq.interleaved_iq.size() >= 2U * 128U) {
    constexpr uint32_t kDemodTargetSampleRateHz = 384000U;
    uint32_t decim = iq.sample_rate_hz / kDemodTargetSampleRateHz;
    decim = std::clamp<uint32_t>(decim, 1U, 16U);
    if (decim > 1U) {
      const size_t in_complex = iq.interleaved_iq.size() / 2U;
      const size_t out_complex = in_complex / static_cast<size_t>(decim);
      if (out_complex >= 128U) {
        demod_iq_decimated.interleaved_iq.resize(out_complex * 2U);
        for (size_t out_idx = 0; out_idx < out_complex; ++out_idx) {
          const size_t in_idx = out_idx * static_cast<size_t>(decim);
          demod_iq_decimated.interleaved_iq[out_idx * 2U] = iq.interleaved_iq[in_idx * 2U];
          demod_iq_decimated.interleaved_iq[out_idx * 2U + 1U] = iq.interleaved_iq[in_idx * 2U + 1U];
        }
        demod_iq_decimated.sample_rate_hz = iq.sample_rate_hz / decim;
        demod_iq_decimated.center_frequency_hz = iq.center_frequency_hz;
        demod_iq_ptr = &demod_iq_decimated;
      }
    }
  }
  std::vector<double> source;
  if (modulation == Modulation::kAm) {
    source = BuildAmEnvelope(*demod_iq_ptr);
  } else {
    source = BuildFmDiscriminator(*demod_iq_ptr, 0, &state->fm_discriminator);
  }
  if (source.size() < 32) {
    return false;
  }

  const double source_sample_rate_hz = static_cast<double>(demod_iq_ptr->sample_rate_hz);
  const double output_nyquist_hz =
      std::max(800.0, static_cast<double>(audio_sample_rate_hz) * 0.5 * 0.9);
  switch (modulation) {
    case Modulation::kWfm:
      // Voice-focused WFM chain with less aggressive high-cut/de-emphasis to
      // avoid overly bass-heavy, smeared audio.
      ApplyCascadedLowPass(&source, source_sample_rate_hz,
                           std::min(5200.0, output_nyquist_hz), 3, &state->lowpass);
      ApplyFmDeemphasis(&source, source_sample_rate_hz, 50.0, &state->deemphasis);
      break;
    case Modulation::kAm:
      ApplyCascadedLowPass(&source, source_sample_rate_hz,
                           std::min(4500.0, output_nyquist_hz), 3, &state->lowpass);
      break;
    case Modulation::kNfm:
    default:
      ApplyCascadedLowPass(&source, source_sample_rate_hz,
                           std::min(3400.0, output_nyquist_hz), 3, &state->lowpass);
      break;
  }
  const double dc_block_cutoff_hz =
      (modulation == Modulation::kWfm) ? 180.0 : ((modulation == Modulation::kNfm) ? 120.0 : 60.0);
  ApplyAudioDcBlock(&source, source_sample_rate_hz, dc_block_cutoff_hz, &state->dc_block);

  if (audio_sample_rate_hz == 0) {
    return false;
  }
  const double ratio = source_sample_rate_hz / static_cast<double>(audio_sample_rate_hz);
  if (ratio <= 0.0 || !std::isfinite(ratio)) {
    return false;
  }
  const double step = std::max(1.0, ratio);
  const bool have_prev_sample =
      state->resampler.has_prev_sample && std::isfinite(state->resampler.prev_sample);
  const double source_base_offset = have_prev_sample ? 1.0 : 0.0;
  const double source_virtual_size = static_cast<double>(source.size()) + source_base_offset;
  if (source_virtual_size < 2.0) {
    return false;
  }
  auto source_at = [&](size_t virtual_idx) -> double {
    if (have_prev_sample) {
      if (virtual_idx == 0) {
        return state->resampler.prev_sample;
      }
      const size_t source_idx = std::min(virtual_idx - 1, source.size() - 1);
      return source[source_idx];
    }
    const size_t source_idx = std::min(virtual_idx, source.size() - 1);
    return source[source_idx];
  };

  pcm_out->reserve(static_cast<size_t>(source_virtual_size / step) + 2U);
  double pos = std::clamp(state->resampler.next_source_pos, 0.0, step) + source_base_offset;
  while (pos < source_virtual_size) {
    const size_t idx = static_cast<size_t>(pos);
    const size_t next_idx = std::min(idx + 1, static_cast<size_t>(source_virtual_size - 1.0));
    const double frac = pos - static_cast<double>(idx);
    const double sample_a = source_at(idx);
    const double sample_b = source_at(next_idx);
    const double mixed = sample_a + (sample_b - sample_a) * frac;
    const double gain =
        (modulation == Modulation::kWfm) ? 7000.0 : ((modulation == Modulation::kAm) ? 9000.0 : 8500.0);
    // Soft-limit peaks to avoid harsh clipping artifacts from impulsive discriminator spikes.
    const double sample = 26000.0 * std::tanh((mixed * gain) / 26000.0);
    pcm_out->push_back(static_cast<int16_t>(std::lrint(sample)));
    pos += step;
  }
  state->resampler.next_source_pos = pos - source_virtual_size;
  if (!std::isfinite(state->resampler.next_source_pos) || state->resampler.next_source_pos < 0.0 ||
      state->resampler.next_source_pos >= step) {
    state->resampler.next_source_pos = 0.0;
  }
  state->resampler.prev_sample = source.back();
  state->resampler.has_prev_sample = true;
  return !pcm_out->empty();
}

std::vector<double> BuildFmDiscriminator(const IQSampleBlock& iq, size_t max_samples,
                                         FmDiscriminatorState* state) {
  std::vector<double> discriminator;
  if (iq.sample_rate_hz == 0 || iq.interleaved_iq.size() < 4) {
    return discriminator;
  }

  const size_t available_samples = iq.interleaved_iq.size() / 2;
  const size_t sample_count = (max_samples == 0) ? available_samples
                                                  : std::min(available_samples, max_samples);
  if (sample_count < 64) {
    return discriminator;
  }

  auto normalize_iq = [](double i, double q, double* out_i, double* out_q) {
    if (out_i == nullptr || out_q == nullptr) {
      return;
    }
    const double mag = std::hypot(i, q);
    if (!std::isfinite(mag) || mag <= 1.0e-6) {
      *out_i = 0.0;
      *out_q = 0.0;
      return;
    }
    *out_i = i / mag;
    *out_q = q / mag;
  };

  size_t start_idx = 1;
  double prev_i = 0.0;
  double prev_q = 0.0;
  normalize_iq(static_cast<double>(iq.interleaved_iq[0]),
               static_cast<double>(iq.interleaved_iq[1]), &prev_i, &prev_q);
  if (state != nullptr && state->initialized && std::isfinite(state->prev_i) && std::isfinite(state->prev_q)) {
    start_idx = 0;
    prev_i = state->prev_i;
    prev_q = state->prev_q;
  }
  discriminator.reserve(sample_count - start_idx);
  constexpr double kMaxPhaseStepRad = 2.2;
  for (size_t n = start_idx; n < sample_count; ++n) {
    double cur_i = 0.0;
    double cur_q = 0.0;
    normalize_iq(static_cast<double>(iq.interleaved_iq[n * 2]),
                 static_cast<double>(iq.interleaved_iq[n * 2 + 1]), &cur_i, &cur_q);
    const double cross = prev_i * cur_q - prev_q * cur_i;
    const double dot = prev_i * cur_i + prev_q * cur_q;
    const double phase_step = std::clamp(std::atan2(cross, dot), -kMaxPhaseStepRad, kMaxPhaseStepRad);
    discriminator.push_back(phase_step);
    prev_i = cur_i;
    prev_q = cur_q;
  }
  if (state != nullptr) {
    state->prev_i = prev_i;
    state->prev_q = prev_q;
    state->initialized = true;
  }

  if (discriminator.empty()) {
    return discriminator;
  }

  if (state == nullptr) {
    double mean = 0.0;
    for (double value : discriminator) {
      mean += value;
    }
    mean /= static_cast<double>(discriminator.size());
    for (double& value : discriminator) {
      value -= mean;
    }
  }
  return discriminator;
}

bool BuildDemodVisualizationFrame(const IQSampleBlock& iq, Modulation modulation,
                                  std::vector<double>* waveform, std::vector<double>* spectrum,
                                  double* peak_hz, double* peak_strength) {
  if (waveform == nullptr || spectrum == nullptr || peak_hz == nullptr || peak_strength == nullptr) {
    return false;
  }

  const uint32_t audio_sample_rate_hz = AudioSampleRateForModulation(modulation);
  if (audio_sample_rate_hz == 0) {
    return false;
  }

  std::vector<int16_t> audio_pcm;
  AudioDemodState demod_state;
  if (!BuildAudioPcm16(iq, modulation, audio_sample_rate_hz, &demod_state, &audio_pcm) ||
      audio_pcm.size() < 32) {
    return false;
  }

  waveform->assign(kWaveformPoints, 0.5);
  for (int i = 0; i < kWaveformPoints; ++i) {
    const size_t idx = static_cast<size_t>((static_cast<double>(i) * (audio_pcm.size() - 1)) /
                                           static_cast<double>(kWaveformPoints - 1));
    const double raw = static_cast<double>(audio_pcm[idx]) / 32768.0;
    const double normalized = std::clamp((raw + 1.0) * 0.5, 0.0, 1.0);
    (*waveform)[static_cast<size_t>(i)] = normalized;
  }

  const double sample_rate_hz = static_cast<double>(audio_sample_rate_hz);
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
    for (int16_t sample : audio_pcm) {
      const double sample_norm = static_cast<double>(sample) / 32768.0;
      q0 = coeff * q1 - q2 + sample_norm;
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
  const size_t sample_count = std::min(available_samples, static_cast<size_t>(2048));
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

  {
    std::lock_guard<std::mutex> lock(mu_);
    mode_config_ = NormalizeModeConfig(mode_config_);
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
  uint32_t applied_hardware_bandwidth_hz = std::numeric_limits<uint32_t>::max();
  uint32_t applied_tune_hz = std::numeric_limits<uint32_t>::max();
  IqLowPassState lowpass_state;
  IqFrequencyShiftState frequency_shift_state;
  IqDcBlockState dc_block_state;
  IqCenterNotchState center_notch_state;
  std::vector<int16_t> audio_pcm_buffer;
  AudioDemodState audio_demod_state;

  while (running_.load()) {
    std::optional<double> frequency_hz;
    RadioMode active_mode = RadioMode::kFixed;
    uint32_t dwell_ms = 500;
    uint32_t channel_bandwidth_hz = kDefaultChannelBandwidthHz;
    uint32_t desired_hardware_bandwidth_hz = 0;
    bool dc_blocker_enabled = false;
    uint32_t dc_blocker_cutoff_hz = kDefaultDcBlockerCutoffHz;
    bool center_notch_enabled = false;
    uint32_t center_notch_width_hz = kDefaultCenterNotchWidthHz;
    bool lo_offset_enabled = false;
    int32_t lo_offset_hz = 0;
    bool scan_list_monitor_mode = false;
    size_t scan_list_channel_count = 0;
    int scan_list_channel_index = -1;
    double squelch_threshold_db = -67.5;
    Modulation scan_modulation = Modulation::kNfm;
    bool has_scan_squelch = false;
    double scan_list_channel_frequency_hz = 0.0;
    std::string scan_list_channel_label = "unnamed";
    {
      std::lock_guard<std::mutex> lock(mu_);
      mode_config_ = NormalizeModeConfig(mode_config_);
      active_mode = mode_;
      const auto target = scheduler_.NextTarget();
      if (target.has_value()) {
        frequency_hz = target->frequency_hz;
        dwell_ms = target->dwell_ms;
        scan_list_channel_index = target->scan_list_channel_index;
        scan_list_channel_frequency_hz = target->frequency_hz;
      }
      channel_bandwidth_hz = mode_config_.channel_bandwidth_hz;
      desired_hardware_bandwidth_hz = mode_config_.hardware_bandwidth_hz;
      dc_blocker_enabled = mode_config_.dc_blocker_enabled;
      dc_blocker_cutoff_hz = mode_config_.dc_blocker_cutoff_hz;
      center_notch_enabled = mode_config_.center_notch_enabled;
      center_notch_width_hz = mode_config_.center_notch_width_hz;
      lo_offset_enabled = mode_config_.lo_offset_enabled;
      lo_offset_hz = mode_config_.lo_offset_hz;
      scan_list_monitor_mode = mode_config_.scan_list_monitor_mode;
      scan_list_channel_count = mode_config_.scan_list_channels.size();
      if (active_mode == RadioMode::kScanList && scan_list_channel_index >= 0 &&
          static_cast<size_t>(scan_list_channel_index) < mode_config_.scan_list_channels.size()) {
        const auto& channel = mode_config_.scan_list_channels[static_cast<size_t>(scan_list_channel_index)];
        channel_bandwidth_hz = channel.channel_bandwidth_hz;
        squelch_threshold_db = channel.squelch_threshold_db;
        scan_modulation = channel.modulation;
        has_scan_squelch = true;
        scan_list_channel_frequency_hz = channel.frequency_hz;
        if (!channel.label.empty()) {
          scan_list_channel_label = channel.label;
        } else {
          scan_list_channel_label = "idx_" + std::to_string(scan_list_channel_index);
        }
      }
    }
    const std::string scan_list_channel_label_token = SanitizeToken(scan_list_channel_label);

    std::string error;
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
      applied_tune_hz = std::numeric_limits<uint32_t>::max();
      audio_pcm_buffer.clear();
      audio_demod_state = AudioDemodState{};
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
    const bool tune_changed = (tune_hz != applied_tune_hz);
    if (tune_changed) {
      if (!device_->SetCenterFrequencyHz(tune_hz, &error)) {
        {
          std::lock_guard<std::mutex> lock(mu_);
          last_error_ = error;
        }
        applied_tune_hz = std::numeric_limits<uint32_t>::max();
        PublishEvent(EventKind::kError, "tune failed: " + error, logical_tuned_frequency_hz);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }
      applied_tune_hz = tune_hz;
      lowpass_state.initialized = false;
      dc_block_state.initialized = false;
      center_notch_state.initialized = false;
      frequency_shift_state.phase_rad = 0.0;
      audio_pcm_buffer.clear();
      audio_demod_state = AudioDemodState{};

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
    }
    if (has_scan_squelch) {
      std::ostringstream status;
      status << "SCAN_STATUS idx=" << scan_list_channel_index
             << " state=" << (scan_list_monitor_mode ? "open" : "closed")
             << " signal_db=-120.0"
             << " threshold_db=" << FormatDouble(squelch_threshold_db, 1)
             << " monitor=" << (scan_list_monitor_mode ? "1" : "0");
      PublishEvent(EventKind::kInfo, status.str(), logical_tuned_frequency_hz, false);
    }

    const double tuned_frequency_hz = logical_tuned_frequency_hz;
    const auto tune_started_at = std::chrono::steady_clock::now();
    const auto dwell_deadline = tune_started_at + std::chrono::milliseconds(dwell_ms);
    std::optional<std::chrono::steady_clock::time_point> squelch_closed_at;
    bool squelch_open = scan_list_monitor_mode && has_scan_squelch;
    bool squelch_seen_open = scan_list_monitor_mode && has_scan_squelch;
    const bool hold_on_squelch = has_scan_squelch && !scan_list_monitor_mode;
    // Latch media/decode gates after first squelch-open and keep them open until channel hop.
    bool gate_open_until_channel_hop = scan_list_monitor_mode && has_scan_squelch;
    std::optional<std::chrono::steady_clock::time_point> squelch_opened_at;
    std::optional<std::chrono::steady_clock::time_point> squelch_close_candidate_at;
    double last_signal_db = -120.0;
    auto next_viz_emit = std::chrono::steady_clock::time_point::min();
    auto next_scan_status_emit = std::chrono::steady_clock::time_point::min();
    auto next_audio_stats_emit = tune_started_at + std::chrono::milliseconds(kAudioStatsIntervalMs);
    auto audio_stats_last_emit_at = tune_started_at;
    uint64_t audio_stats_iq_blocks = 0;
    uint64_t audio_stats_gate_open_blocks = 0;
    uint64_t audio_stats_demod_ok_blocks = 0;
    uint64_t audio_stats_demod_empty_blocks = 0;
    uint64_t audio_stats_generated_samples = 0;
    uint64_t audio_stats_published_frames = 0;
    uint64_t audio_stats_published_samples = 0;
    uint64_t audio_stats_buffer_clears = 0;
    uint64_t audio_stats_flush_frames = 0;
    uint64_t audio_stats_flush_samples = 0;
    uint64_t audio_stats_conceal_samples = 0;
    uint32_t audio_gate_open_stats_windows = 0;
    uint64_t audio_stream_sequence = 0;
    uint64_t audio_stream_sample_index = 0;
    uint32_t audio_stats_last_iq_sample_rate_hz = 0;
    size_t audio_stats_last_iq_complex_samples = 0;
    uint32_t audio_stats_configured_sample_rate_hz = 0;
    double audio_stats_estimated_iq_sample_rate_hz =
        iq_rate_estimate_valid_ ? iq_rate_estimate_hz_ : 0.0;
    double audio_stats_last_gen_ratio = 1.0;
    double audio_stats_last_rate_correction = 1.0;
    std::optional<std::chrono::steady_clock::time_point> audio_iq_last_read_at;
    bool audio_pacer_primed = false;
    auto next_audio_frame_deadline = tune_started_at;
    bool audio_pacer_has_last_sample = false;
    int16_t audio_pacer_last_sample = 0;
    const bool single_scan_channel =
        (active_mode == RadioMode::kScanList && scan_list_channel_count <= 1);

    while (running_.load()) {
      const auto now = std::chrono::steady_clock::now();
      if (!hold_on_squelch && !single_scan_channel && now >= dwell_deadline) {
        break;
      }
      if (hold_on_squelch) {
        if (!single_scan_channel && !squelch_seen_open && now >= dwell_deadline) {
          break;
        }
        if (squelch_seen_open && !squelch_open && squelch_closed_at.has_value() &&
            !single_scan_channel &&
            now >= *squelch_closed_at + std::chrono::milliseconds(dwell_ms)) {
          break;
        }
      }

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
      const size_t iq_complex_samples = iq.interleaved_iq.size() / 2;
      const auto iq_read_at = std::chrono::steady_clock::now();
      if (audio_iq_last_read_at.has_value() && iq_complex_samples > 0) {
        const double dt_s =
            std::chrono::duration<double>(iq_read_at - *audio_iq_last_read_at).count();
        if (dt_s >= 1.0e-4) {
          const double inst_rate_hz =
              static_cast<double>(iq_complex_samples) / dt_s;
          if (std::isfinite(inst_rate_hz) && inst_rate_hz >= static_cast<double>(kMinSampleRateHz) &&
              inst_rate_hz <= static_cast<double>(kMaxSampleRateHz)) {
            if (!iq_rate_estimate_valid_ || iq_rate_estimate_hz_ <= 0.0) {
              iq_rate_estimate_hz_ = inst_rate_hz;
              iq_rate_estimate_valid_ = true;
            } else {
              constexpr double kIqrEmaAlpha = 0.12;
              iq_rate_estimate_hz_ =
                  (1.0 - kIqrEmaAlpha) * iq_rate_estimate_hz_ + kIqrEmaAlpha * inst_rate_hz;
            }
          }
        }
      }
      audio_iq_last_read_at = iq_read_at;
      if (!iq_rate_estimate_valid_ || iq_rate_estimate_hz_ <= 0.0) {
        iq_rate_estimate_hz_ = static_cast<double>(iq.sample_rate_hz);
        iq_rate_estimate_valid_ = true;
      }
      audio_stats_estimated_iq_sample_rate_hz = iq_rate_estimate_hz_;
      audio_stats_last_iq_sample_rate_hz = iq.sample_rate_hz;
      audio_stats_last_iq_complex_samples = iq_complex_samples;
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

      const bool emit_viz_this_tick =
          kBackendVisualizationEnabled && (now >= next_viz_emit);
      double receiver_peak_hz = 0.0;
      double receiver_peak_strength = 0.0;
      double receiver_start_hz = 0.0;
      double receiver_end_hz = 0.0;
      std::vector<double> receiver_waveform;
      std::vector<double> receiver_spectrum;
      bool have_receiver_frame = false;
      if (emit_viz_this_tick) {
        have_receiver_frame =
            BuildReceiverSpectrumFrame(iq, tuned_frequency_hz, channel_bandwidth_hz, &receiver_waveform,
                                       &receiver_spectrum, &receiver_peak_hz, &receiver_peak_strength,
                                       &receiver_start_hz, &receiver_end_hz);
      }

      ApplyIqChannelBandwidth(&iq, channel_bandwidth_hz, &lowpass_state);
      const double signal_db = EstimateSignalDbfs(iq);
      last_signal_db = signal_db;

      if (hold_on_squelch) {
        const double close_threshold_db = squelch_threshold_db - kSquelchCloseHysteresisDb;
        bool next_squelch_open = squelch_open;
        if (squelch_open) {
          if (signal_db >= close_threshold_db) {
            squelch_close_candidate_at.reset();
          } else {
            if (!squelch_close_candidate_at.has_value()) {
              squelch_close_candidate_at = now;
            }
            if (now >= *squelch_close_candidate_at +
                           std::chrono::milliseconds(kSquelchCloseDebounceMs)) {
              next_squelch_open = false;
            }
          }
        } else {
          squelch_close_candidate_at.reset();
          next_squelch_open = signal_db >= squelch_threshold_db;
        }
        if (next_squelch_open != squelch_open) {
          squelch_open = next_squelch_open;
          if (squelch_open) {
            gate_open_until_channel_hop = true;
            squelch_close_candidate_at.reset();
            squelch_seen_open = true;
            squelch_closed_at.reset();
            squelch_opened_at = std::chrono::steady_clock::now();
            std::ostringstream opened;
            opened << "SCAN_SQUELCH_OPEN idx=" << scan_list_channel_index
                   << " label=" << scan_list_channel_label_token
                   << " freq_hz=" << FormatDouble(scan_list_channel_frequency_hz, 0)
                   << " signal_db=" << FormatDouble(signal_db, 1)
                   << " threshold_db=" << FormatDouble(squelch_threshold_db, 1);
            PublishEvent(EventKind::kInfo, opened.str(), tuned_frequency_hz);
          } else {
            squelch_close_candidate_at.reset();
            squelch_closed_at = std::chrono::steady_clock::now();
            if (squelch_opened_at.has_value()) {
              const auto open_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       std::chrono::steady_clock::now() - *squelch_opened_at)
                                       .count();
              std::ostringstream closed;
              closed << "SCAN_SQUELCH_CLOSE idx=" << scan_list_channel_index
                     << " label=" << scan_list_channel_label_token
                     << " freq_hz=" << FormatDouble(scan_list_channel_frequency_hz, 0)
                     << " signal_db=" << FormatDouble(signal_db, 1)
                     << " threshold_db=" << FormatDouble(squelch_threshold_db, 1)
                     << " open_ms=" << open_ms;
              PublishEvent(EventKind::kInfo, closed.str(), tuned_frequency_hz);
            }
            squelch_opened_at.reset();
          }
          std::ostringstream status;
          status << "SCAN_STATUS idx=" << scan_list_channel_index
                 << " state=" << (squelch_open ? "open" : "closed")
                 << " signal_db=" << FormatDouble(signal_db, 1)
                 << " threshold_db=" << FormatDouble(squelch_threshold_db, 1);
          PublishEvent(EventKind::kInfo, status.str(), tuned_frequency_hz, false);
        }
      } else if (scan_list_monitor_mode && has_scan_squelch) {
        squelch_close_candidate_at.reset();
        squelch_open = true;
        squelch_seen_open = true;
        squelch_closed_at.reset();
        if (now >= next_scan_status_emit) {
          std::ostringstream status;
          status << "SCAN_STATUS idx=" << scan_list_channel_index
                 << " state=open"
                 << " signal_db=" << FormatDouble(signal_db, 1)
                 << " threshold_db=" << FormatDouble(squelch_threshold_db, 1)
                 << " monitor=1";
          PublishEvent(EventKind::kInfo, status.str(), tuned_frequency_hz, false);
          next_scan_status_emit = now + std::chrono::milliseconds(kScanStatusIntervalMs);
        }
      }
      const bool signal_gate_open = !has_scan_squelch || squelch_open || gate_open_until_channel_hop;
      const bool audio_gate_open = has_scan_squelch && (squelch_open || gate_open_until_channel_hop);
      if (has_scan_squelch) {
        ++audio_stats_iq_blocks;
        if (audio_gate_open) {
          ++audio_stats_gate_open_blocks;
        }
      }

      double demod_peak_hz = 0.0;
      double demod_peak_strength = 0.0;
      std::vector<double> demod_waveform;
      std::vector<double> demod_spectrum;
      bool have_demod_frame = false;
      if (emit_viz_this_tick) {
        have_demod_frame = BuildDemodVisualizationFrame(
            iq, scan_modulation, &demod_waveform, &demod_spectrum, &demod_peak_hz, &demod_peak_strength);
        if (has_scan_squelch && !audio_gate_open) {
          demod_waveform.assign(kWaveformPoints, 0.5);
          demod_spectrum.assign(kSpectrumBins, 0.0);
          demod_peak_hz = 0.0;
          demod_peak_strength = 0.0;
          have_demod_frame = true;
        }
      }

      if (signal_gate_open) {
        bool process_plugins_this_block = true;
        // Prioritize uninterrupted audio in scan mode while squelch is open.
        // Plugin decoding is relatively expensive and can starve audio generation.
        if (has_scan_squelch && audio_gate_open) {
          process_plugins_this_block = false;
        }
        if (process_plugins_this_block) {
          plugin_host_->ProcessIq(iq, [&](const PluginMessage& plugin_msg) {
          const auto kind_it = plugin_msg.normalized_fields.find("kind");
          if (kind_it != plugin_msg.normalized_fields.end() && kind_it->second == "candidate") {
            return;
          }
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
        }
      }

      if (emit_viz_this_tick) {
        if (have_demod_frame) {
          const double demod_start_hz = 0.0;
          const uint32_t demod_audio_sample_rate_hz = AudioSampleRateForModulation(scan_modulation);
          const double demod_nyquist_hz =
              std::max(1.0, static_cast<double>(demod_audio_sample_rate_hz) * 0.5);
          const double demod_end_hz =
              std::max(1.0, std::min(20000.0, demod_nyquist_hz - 1.0));
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

      if (audio_gate_open) {
        std::vector<int16_t> block_pcm;
        const uint32_t audio_sample_rate_hz = AudioSampleRateForModulation(scan_modulation);
        const size_t audio_frame_samples = AudioFrameSamplesForRate(audio_sample_rate_hz);
        if (audio_frame_samples == 0) {
          audio_pcm_buffer.clear();
          audio_demod_state = AudioDemodState{};
          continue;
        }
        if (BuildAudioPcm16(iq, scan_modulation, audio_sample_rate_hz, &audio_demod_state, &block_pcm)) {
          ++audio_stats_demod_ok_blocks;
          audio_stats_generated_samples += static_cast<uint64_t>(block_pcm.size());
          audio_pcm_buffer.insert(audio_pcm_buffer.end(), block_pcm.begin(), block_pcm.end());
        } else {
          ++audio_stats_demod_empty_blocks;
        }
        if (!audio_pacer_primed && !audio_pcm_buffer.empty()) {
          audio_pacer_primed = true;
          next_audio_frame_deadline = now;
        }
        if (audio_pacer_primed) {
          const auto pace_now = std::chrono::steady_clock::now();
          const auto frame_interval = std::chrono::milliseconds(kAudioFrameIntervalMs);
          const auto max_late = std::chrono::milliseconds(400);
          if (pace_now - next_audio_frame_deadline > max_late) {
            // Avoid catch-up bursts after scheduler hiccups; realign pacing to "now".
            next_audio_frame_deadline = pace_now;
          }
          size_t paced_frames_this_loop = 0;
          while (std::chrono::steady_clock::now() >= next_audio_frame_deadline &&
                 paced_frames_this_loop < kMaxPacedAudioFramesPerLoop) {
            AudioFrame frame;
            frame.unix_ms = UnixMillisNow();
            frame.receiver_id = receiver_id_;
            frame.sample_rate_hz = audio_sample_rate_hz;
            frame.tuned_frequency_hz = tuned_frequency_hz;
            frame.pcm_s16le.reserve(audio_frame_samples);

            const size_t take = std::min(audio_frame_samples, audio_pcm_buffer.size());
            if (take > 0) {
              frame.pcm_s16le.insert(frame.pcm_s16le.end(), audio_pcm_buffer.begin(),
                                     audio_pcm_buffer.begin() +
                                         static_cast<std::vector<int16_t>::difference_type>(take));
              audio_pcm_buffer.erase(
                  audio_pcm_buffer.begin(),
                  audio_pcm_buffer.begin() + static_cast<std::vector<int16_t>::difference_type>(take));
            }

            if (frame.pcm_s16le.size() < audio_frame_samples) {
              const size_t need = audio_frame_samples - frame.pcm_s16le.size();
              frame.pcm_s16le.insert(frame.pcm_s16le.end(), need, static_cast<int16_t>(0));
              audio_stats_conceal_samples += static_cast<uint64_t>(need);
            }

            if (!frame.pcm_s16le.empty()) {
              audio_pacer_last_sample = frame.pcm_s16le.back();
              audio_pacer_has_last_sample = true;
              frame.sequence = audio_stream_sequence++;
              frame.sample_index = audio_stream_sample_index;
              audio_stream_sample_index += static_cast<uint64_t>(frame.pcm_s16le.size());
              ++audio_stats_published_frames;
              audio_stats_published_samples += static_cast<uint64_t>(frame.pcm_s16le.size());
              event_bus_->PublishAudioFrame(frame);
            }

            next_audio_frame_deadline += frame_interval;
            ++paced_frames_this_loop;
          }
        }
      } else if (has_scan_squelch) {
        ++audio_stats_buffer_clears;
        audio_pcm_buffer.clear();
        audio_demod_state = AudioDemodState{};
        audio_pacer_primed = false;
        audio_pacer_has_last_sample = false;
        audio_pacer_last_sample = 0;
        next_audio_frame_deadline = now;
      }

      if (has_scan_squelch && now >= next_audio_stats_emit) {
        const uint32_t audio_sample_rate_hz = AudioSampleRateForModulation(scan_modulation);
        const double stats_window_s = std::max(
            1.0e-3, std::chrono::duration<double>(now - audio_stats_last_emit_at).count());
        const uint64_t stats_window_ms = static_cast<uint64_t>(std::llround(stats_window_s * 1000.0));
        audio_stats_last_emit_at = now;
        if (audio_gate_open) {
          ++audio_gate_open_stats_windows;
        } else {
          audio_gate_open_stats_windows = 0;
        }
        audio_stats_last_gen_ratio = 1.0;
        audio_stats_last_rate_correction = 1.0;
        if (audio_gate_open && audio_sample_rate_hz > 0 && audio_stats_generated_samples > 0) {
          const double target_generated_samples =
              static_cast<double>(audio_sample_rate_hz) * stats_window_s;
          if (target_generated_samples >= 1000.0) {
            const double generated_samples = static_cast<double>(audio_stats_generated_samples);
            const double gen_ratio =
                std::clamp(generated_samples / target_generated_samples, 0.75, 1.25);
            audio_stats_last_gen_ratio = gen_ratio;
            // Keep playback tempo stable: do not time-stretch by retuning IQ rate
            // from generated-sample ratio. Use miss_f/miss_b in frontend to detect
            // true transport underruns instead.
            audio_stats_last_rate_correction = 1.0;
          }
        }
        const double generated_hz = static_cast<double>(audio_stats_generated_samples) / stats_window_s;
        const double published_hz = static_cast<double>(audio_stats_published_samples) / stats_window_s;
        std::ostringstream audio_status;
        audio_status << "AUDIO_STATS idx=" << scan_list_channel_index
                     << " label=" << scan_list_channel_label_token
                     << " rev=" << kAudioPipelineRevision
                     << " mod=" << ModulationToken(scan_modulation)
                     << " sr=" << audio_sample_rate_hz
                     << " cfg_sr=" << audio_stats_configured_sample_rate_hz
                     << " iq_sr=" << audio_stats_last_iq_sample_rate_hz
                     << " iq_est_sr=" << FormatDouble(audio_stats_estimated_iq_sample_rate_hz, 0)
                     << " iq_lock=0"
                     << " iq_lock_sr=0"
                     << " win_ms=" << stats_window_ms
                     << " gen_ratio=" << FormatDouble(audio_stats_last_gen_ratio, 3)
                     << " rate_corr=" << FormatDouble(audio_stats_last_rate_correction, 4)
                     << " gen_hz=" << FormatDouble(generated_hz, 1)
                     << " pub_hz=" << FormatDouble(published_hz, 1)
                     << " iq_n=" << audio_stats_last_iq_complex_samples
                     << " gate=" << (audio_gate_open ? "1" : "0")
                     << " squelch=" << (squelch_open ? "1" : "0")
                     << " signal_db=" << FormatDouble(signal_db, 1)
                     << " blocks=" << audio_stats_iq_blocks
                     << " gate_open_blocks=" << audio_stats_gate_open_blocks
                     << " demod_ok=" << audio_stats_demod_ok_blocks
                     << " demod_empty=" << audio_stats_demod_empty_blocks
                     << " gen_samples=" << audio_stats_generated_samples
                     << " pub_frames=" << audio_stats_published_frames
                     << " pub_samples=" << audio_stats_published_samples
                     << " pending_samples=" << audio_pcm_buffer.size()
                     << " conceal_samples=" << audio_stats_conceal_samples
                     << " clears=" << audio_stats_buffer_clears
                     << " flush_frames=" << audio_stats_flush_frames
                     << " flush_samples=" << audio_stats_flush_samples;
        PublishEvent(EventKind::kInfo, audio_status.str(), tuned_frequency_hz, false);
        audio_stats_iq_blocks = 0;
        audio_stats_gate_open_blocks = 0;
        audio_stats_demod_ok_blocks = 0;
        audio_stats_demod_empty_blocks = 0;
        audio_stats_generated_samples = 0;
        audio_stats_published_frames = 0;
        audio_stats_published_samples = 0;
        audio_stats_conceal_samples = 0;
        audio_stats_buffer_clears = 0;
        audio_stats_flush_frames = 0;
        audio_stats_flush_samples = 0;
        next_audio_stats_emit = now + std::chrono::milliseconds(kAudioStatsIntervalMs);
      }
    }
    if (has_scan_squelch && !audio_pcm_buffer.empty()) {
      AudioFrame frame;
      frame.unix_ms = UnixMillisNow();
      frame.receiver_id = receiver_id_;
      frame.sample_rate_hz = AudioSampleRateForModulation(scan_modulation);
      frame.tuned_frequency_hz = tuned_frequency_hz;
      frame.pcm_s16le = audio_pcm_buffer;
      frame.sequence = audio_stream_sequence++;
      frame.sample_index = audio_stream_sample_index;
      audio_stream_sample_index += static_cast<uint64_t>(frame.pcm_s16le.size());
      ++audio_stats_flush_frames;
      audio_stats_flush_samples += static_cast<uint64_t>(frame.pcm_s16le.size());
      ++audio_stats_published_frames;
      audio_stats_published_samples += static_cast<uint64_t>(frame.pcm_s16le.size());
      event_bus_->PublishAudioFrame(frame);
    }
    if (has_scan_squelch) {
      audio_pcm_buffer.clear();
      audio_demod_state = AudioDemodState{};
    }
    if (hold_on_squelch && squelch_opened_at.has_value()) {
      const auto open_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - *squelch_opened_at)
                               .count();
      std::ostringstream closed;
      closed << "SCAN_SQUELCH_CLOSE idx=" << scan_list_channel_index
             << " label=" << scan_list_channel_label_token
             << " freq_hz=" << FormatDouble(scan_list_channel_frequency_hz, 0)
             << " signal_db=" << FormatDouble(last_signal_db, 1)
             << " threshold_db=" << FormatDouble(squelch_threshold_db, 1)
             << " open_ms=" << open_ms;
      PublishEvent(EventKind::kInfo, closed.str(), tuned_frequency_hz);
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
