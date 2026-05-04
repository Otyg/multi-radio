#include "multi_radio/fm_demod.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
#include <liquid/liquid.h>
#endif

namespace multi_radio {

namespace {

constexpr float kResamplerStopbandAttenuationDb = 70.0f;
constexpr float kTwoPi = 6.28318530717958647692f;

#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
using ComplexSample = liquid_float_complex;
#else
using ComplexSample = std::complex<float>;
#endif

bool IsFm(Modulation modulation) {
  return modulation == Modulation::kNfm || modulation == Modulation::kWfm;
}

uint32_t TargetChannelSampleRateHz(Modulation modulation, uint32_t channel_bandwidth_hz,
                                   uint32_t input_sample_rate_hz) {
  if (modulation == Modulation::kWfm) {
    // Keep the WFM demod channel rate moderate to avoid CPU starvation that
    // causes audible pulsing from concealment in real-time playback.
    const uint32_t requested_hz = std::max<uint32_t>(220000U, channel_bandwidth_hz + 60000U);
    const uint32_t capped_hz = std::min<uint32_t>(requested_hz, 320000U);
    return std::clamp(capped_hz, 48000U, std::max(48000U, input_sample_rate_hz));
  }

  const uint32_t requested_hz = std::max(channel_bandwidth_hz, 64000U);
  const uint32_t expanded_hz = std::max(requested_hz, channel_bandwidth_hz * 2U);
  return std::clamp(expanded_hz, 48000U, std::max(48000U, input_sample_rate_hz));
}

float FmDeviationHz(Modulation modulation) {
  return modulation == Modulation::kWfm ? 75000.0f : 5000.0f;
}

float DeemphasisTauSeconds(Modulation modulation) {
  return modulation == Modulation::kWfm ? 75.0e-6f : 300.0e-6f;
}

}  // namespace

struct FmDemodulator::Impl {
  uint32_t input_sample_rate_hz = 0;
  uint32_t channel_sample_rate_hz = 0;
  uint32_t audio_sample_rate_hz = 0;
  uint32_t channel_bandwidth_hz = 0;
  Modulation modulation = Modulation::kNfm;

  float deemphasis_alpha = 0.0f;
  float deemphasis_state = 0.0f;
  float pcm_gain = 13000.0f;

  // Pilot notch (WFM only, 19 kHz stereo pilot)
  bool use_pilot_notch = false;
  float notch_b0 = 1.0f, notch_b1 = 0.0f, notch_b2 = 1.0f;
  float notch_a1 = 0.0f, notch_a2 = 0.0f;
  float notch_x1 = 0.0f, notch_x2 = 0.0f;
  float notch_y1 = 0.0f, notch_y2 = 0.0f;

  std::vector<ComplexSample> iq_complex;
  std::vector<ComplexSample> channelized_complex;
  std::vector<float> demodulated;
  std::vector<float> deemphasized;
  std::vector<float> audio_resampled;

#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
  msresamp_crcf iq_channelizer = nullptr;
  freqdem fm_demod = nullptr;
  msresamp_rrrf audio_resampler = nullptr;
#endif

  void ResetStateOnly() {
    deemphasis_state = 0.0f;
    notch_x1 = 0.0f; notch_x2 = 0.0f;
    notch_y1 = 0.0f; notch_y2 = 0.0f;
    iq_complex.clear();
    channelized_complex.clear();
    demodulated.clear();
    deemphasized.clear();
    audio_resampled.clear();
  }

  void DestroyObjects() {
#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
    if (audio_resampler != nullptr) {
      msresamp_rrrf_destroy(audio_resampler);
      audio_resampler = nullptr;
    }
    if (fm_demod != nullptr) {
      freqdem_destroy(fm_demod);
      fm_demod = nullptr;
    }
    if (iq_channelizer != nullptr) {
      msresamp_crcf_destroy(iq_channelizer);
      iq_channelizer = nullptr;
    }
#endif
    ResetStateOnly();
    input_sample_rate_hz = 0;
    channel_sample_rate_hz = 0;
    audio_sample_rate_hz = 0;
    channel_bandwidth_hz = 0;
    modulation = Modulation::kNfm;
  }
};

FmDemodulator::FmDemodulator() : impl_(new Impl()) {}

FmDemodulator::~FmDemodulator() {
  if (impl_ != nullptr) {
    impl_->DestroyObjects();
    delete impl_;
    impl_ = nullptr;
  }
}

bool FmDemodulator::Available() {
#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
  return true;
#else
  return false;
#endif
}

void FmDemodulator::Reset() {
  if (impl_ == nullptr) {
    return;
  }
  impl_->DestroyObjects();
}

bool FmDemodulator::Configure(uint32_t input_sample_rate_hz, uint32_t audio_sample_rate_hz,
                              Modulation modulation, uint32_t channel_bandwidth_hz,
                              std::string* error) {
  if (impl_ == nullptr) {
    if (error != nullptr) {
      *error = "demodulator not initialized";
    }
    return false;
  }
  if (!IsFm(modulation)) {
    if (error != nullptr) {
      *error = "modulation is not FM";
    }
    return false;
  }
  if (input_sample_rate_hz == 0 || audio_sample_rate_hz == 0) {
    if (error != nullptr) {
      *error = "invalid sample-rate configuration";
    }
    return false;
  }
#if !(defined(MR_HAS_LIQUID) && MR_HAS_LIQUID)
  if (error != nullptr) {
    *error = "libliquid is not available in this build";
  }
  return false;
#else
  impl_->DestroyObjects();

  const uint32_t channel_sample_rate_hz =
      TargetChannelSampleRateHz(modulation, channel_bandwidth_hz, input_sample_rate_hz);
  const float channel_ratio =
      static_cast<float>(channel_sample_rate_hz) / static_cast<float>(input_sample_rate_hz);
  const float audio_ratio =
      static_cast<float>(audio_sample_rate_hz) / static_cast<float>(channel_sample_rate_hz);
  const float deviation_hz = FmDeviationHz(modulation);
  // kf = f_deviation / f_sample (normalized deviation per sample, no 2π)
  const float kf = deviation_hz /
                   static_cast<float>(std::max<uint32_t>(1U, channel_sample_rate_hz));

  impl_->iq_channelizer = msresamp_crcf_create(channel_ratio, kResamplerStopbandAttenuationDb);
  if (impl_->iq_channelizer == nullptr) {
    if (error != nullptr) {
      *error = "failed to create channel resampler";
    }
    impl_->DestroyObjects();
    return false;
  }

  impl_->fm_demod = freqdem_create(std::max(1.0e-4f, kf));
  if (impl_->fm_demod == nullptr) {
    if (error != nullptr) {
      *error = "failed to create FM demodulator";
    }
    impl_->DestroyObjects();
    return false;
  }

  impl_->audio_resampler = msresamp_rrrf_create(audio_ratio, kResamplerStopbandAttenuationDb);
  if (impl_->audio_resampler == nullptr) {
    if (error != nullptr) {
      *error = "failed to create audio resampler";
    }
    impl_->DestroyObjects();
    return false;
  }

  const float tau = DeemphasisTauSeconds(modulation);
  impl_->deemphasis_alpha =
      std::exp(-1.0f / (tau * static_cast<float>(std::max<uint32_t>(1U, channel_sample_rate_hz))));
  impl_->deemphasis_state = 0.0f;

  // 19 kHz stereo pilot notch for WFM (IIR biquad, zeros at e^(±jw0), poles at r*e^(±jw0))
  if (modulation == Modulation::kWfm) {
    const float w0 = kTwoPi * 19000.0f / static_cast<float>(std::max<uint32_t>(1U, channel_sample_rate_hz));
    const float cos_w0 = std::cos(w0);
    const float r = 0.95f;
    impl_->notch_b0 = 1.0f;
    impl_->notch_b1 = -2.0f * cos_w0;
    impl_->notch_b2 = 1.0f;
    impl_->notch_a1 = -2.0f * r * cos_w0;
    impl_->notch_a2 = r * r;
    impl_->use_pilot_notch = true;
  } else {
    impl_->use_pilot_notch = false;
  }
  impl_->notch_x1 = 0.0f; impl_->notch_x2 = 0.0f;
  impl_->notch_y1 = 0.0f; impl_->notch_y2 = 0.0f;

  impl_->modulation = modulation;
  impl_->input_sample_rate_hz = input_sample_rate_hz;
  impl_->channel_sample_rate_hz = channel_sample_rate_hz;
  impl_->audio_sample_rate_hz = audio_sample_rate_hz;
  impl_->channel_bandwidth_hz = channel_bandwidth_hz;

  if (error != nullptr) {
    error->clear();
  }
  return true;
#endif
}

bool FmDemodulator::ProcessIq(const std::vector<int16_t>& interleaved_iq, std::vector<int16_t>* pcm_out,
                              FmDemodProcessStats* stats, std::string* error) {
  if (pcm_out == nullptr) {
    if (error != nullptr) {
      *error = "pcm_out is null";
    }
    return false;
  }
  pcm_out->clear();
  if (stats != nullptr) {
    *stats = {};
  }
  if (impl_ == nullptr) {
    if (error != nullptr) {
      *error = "demodulator not initialized";
    }
    return false;
  }
#if !(defined(MR_HAS_LIQUID) && MR_HAS_LIQUID)
  (void)interleaved_iq;
  if (error != nullptr) {
    *error = "libliquid is not available in this build";
  }
  return false;
#else
  if (interleaved_iq.size() < 2U) {
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }
  if ((interleaved_iq.size() % 2U) != 0U) {
    if (error != nullptr) {
      *error = "interleaved IQ buffer must contain I/Q pairs";
    }
    return false;
  }
  if (impl_->fm_demod == nullptr || impl_->iq_channelizer == nullptr || impl_->audio_resampler == nullptr) {
    if (error != nullptr) {
      *error = "demodulator is not configured";
    }
    return false;
  }

  const unsigned int iq_pairs = static_cast<unsigned int>(interleaved_iq.size() / 2U);
  impl_->iq_complex.resize(iq_pairs);
  for (unsigned int i = 0; i < iq_pairs; ++i) {
    const float i_sample = static_cast<float>(interleaved_iq[2U * i]) / 32768.0f;
    const float q_sample = static_cast<float>(interleaved_iq[(2U * i) + 1U]) / 32768.0f;
    impl_->iq_complex[i] = liquid_float_complex{i_sample, q_sample};
  }

  const float channel_ratio = static_cast<float>(impl_->channel_sample_rate_hz) /
                              static_cast<float>(std::max<uint32_t>(1U, impl_->input_sample_rate_hz));
  const unsigned int max_channel_samples =
      static_cast<unsigned int>(2U + std::ceil((2.2f * channel_ratio) * static_cast<float>(iq_pairs)));
  impl_->channelized_complex.resize(std::max(2U, max_channel_samples));
  unsigned int channelized_count = 0;
  if (msresamp_crcf_execute(impl_->iq_channelizer, impl_->iq_complex.data(), iq_pairs,
                            impl_->channelized_complex.data(), &channelized_count) != LIQUID_OK) {
    if (error != nullptr) {
      *error = "channel resampler failed";
    }
    return false;
  }
  if (channelized_count == 0U) {
    if (stats != nullptr) {
      stats->input_iq_samples = iq_pairs;
    }
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }
  impl_->channelized_complex.resize(channelized_count);

  impl_->demodulated.resize(channelized_count);
  if (freqdem_demodulate_block(impl_->fm_demod, impl_->channelized_complex.data(), channelized_count,
                               impl_->demodulated.data()) != LIQUID_OK) {
    if (error != nullptr) {
      *error = "FM demodulation failed";
    }
    return false;
  }

  if (impl_->use_pilot_notch) {
    const float b0 = impl_->notch_b0, b1 = impl_->notch_b1, b2 = impl_->notch_b2;
    const float a1 = impl_->notch_a1, a2 = impl_->notch_a2;
    float x1 = impl_->notch_x1, x2 = impl_->notch_x2;
    float y1 = impl_->notch_y1, y2 = impl_->notch_y2;
    for (unsigned int i = 0; i < channelized_count; ++i) {
      const float x = impl_->demodulated[i];
      const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
      x2 = x1; x1 = x;
      y2 = y1; y1 = y;
      impl_->demodulated[i] = y;
    }
    impl_->notch_x1 = x1; impl_->notch_x2 = x2;
    impl_->notch_y1 = y1; impl_->notch_y2 = y2;
  }

  impl_->deemphasized.resize(channelized_count);
  const float a = impl_->deemphasis_alpha;
  float state = impl_->deemphasis_state;
  for (unsigned int i = 0; i < channelized_count; ++i) {
    state = (a * state) + ((1.0f - a) * impl_->demodulated[i]);
    impl_->deemphasized[i] = state;
  }
  impl_->deemphasis_state = state;

  const float audio_ratio = static_cast<float>(impl_->audio_sample_rate_hz) /
                            static_cast<float>(std::max<uint32_t>(1U, impl_->channel_sample_rate_hz));
  const unsigned int max_audio_samples =
      static_cast<unsigned int>(2U + std::ceil((2.2f * audio_ratio) * static_cast<float>(channelized_count)));
  impl_->audio_resampled.resize(std::max(2U, max_audio_samples));
  unsigned int audio_count = 0;
  if (msresamp_rrrf_execute(impl_->audio_resampler, impl_->deemphasized.data(), channelized_count,
                            impl_->audio_resampled.data(), &audio_count) != LIQUID_OK) {
    if (error != nullptr) {
      *error = "audio resampler failed";
    }
    return false;
  }
  impl_->audio_resampled.resize(audio_count);

  if (audio_count != 0U) {
    float peak_abs = 0.0f;
    for (const float sample : impl_->audio_resampled) {
      peak_abs = std::max(peak_abs, std::abs(sample));
    }
    if (peak_abs > 0.0f) {
      const float target_peak = 0.75f;
      const float desired_gain = target_peak / peak_abs;
      // Slow AGC to keep demod output visible but avoid pumping.
      impl_->pcm_gain = std::clamp((0.92f * impl_->pcm_gain) + (0.08f * (desired_gain * 32767.0f)),
                                   1500.0f, 22000.0f);
    }

    pcm_out->resize(audio_count);
    for (unsigned int i = 0; i < audio_count; ++i) {
      const float scaled = impl_->audio_resampled[i] * impl_->pcm_gain;
      const float clipped = std::clamp(scaled, -32767.0f, 32767.0f);
      (*pcm_out)[i] = static_cast<int16_t>(std::lrint(clipped));
    }
  }

  if (stats != nullptr) {
    stats->input_iq_samples = iq_pairs;
    stats->channelized_samples = channelized_count;
    stats->demodulated_samples = channelized_count;
    stats->audio_samples = audio_count;
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
#endif
}

}  // namespace multi_radio
