#include "multi_radio/am_demod.hpp"

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

#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
using ComplexSample = liquid_float_complex;
#else
using ComplexSample = std::complex<float>;
#endif

// Channel sample rate: 4× channel bandwidth, clamped to [32 kHz, input_sr].
// For 10 kHz AM this gives 40 kHz — shallow decimation, keeps ampmodem well-sampled.
uint32_t TargetChannelSampleRateHz(uint32_t channel_bandwidth_hz, uint32_t input_sample_rate_hz) {
  const uint32_t requested = std::max(channel_bandwidth_hz * 4U, 32000U);
  return std::clamp(requested, 32000U, std::max(32000U, input_sample_rate_hz));
}

}  // namespace

struct AmDemodulator::Impl {
  uint32_t input_sample_rate_hz = 0;
  uint32_t channel_sample_rate_hz = 0;
  uint32_t audio_sample_rate_hz = 0;
  uint32_t channel_bandwidth_hz = 0;

  float pcm_gain = 13000.0f;
  float channel_rssi_db = -120.0f;

  std::vector<ComplexSample> iq_complex;
  std::vector<ComplexSample> channelized_complex;
  std::vector<float> demodulated;
  std::vector<float> audio_resampled;

#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
  msresamp_crcf iq_channelizer = nullptr;
  msresamp_rrrf audio_resampler = nullptr;
  agc_crcf channel_agc = nullptr;
  std::vector<ComplexSample> agc_scratch;
#endif

  void ResetStateOnly() {
    iq_complex.clear();
    channelized_complex.clear();
    demodulated.clear();
    audio_resampled.clear();
    channel_rssi_db = -120.0f;
  }

  void DestroyObjects() {
#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
    if (channel_agc != nullptr) {
      agc_crcf_destroy(channel_agc);
      channel_agc = nullptr;
    }
    if (audio_resampler != nullptr) {
      msresamp_rrrf_destroy(audio_resampler);
      audio_resampler = nullptr;
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
  }
};

AmDemodulator::AmDemodulator() : impl_(new Impl()) {}

AmDemodulator::~AmDemodulator() {
  if (impl_ != nullptr) {
    impl_->DestroyObjects();
    delete impl_;
    impl_ = nullptr;
  }
}

bool AmDemodulator::Available() {
#if defined(MR_HAS_LIQUID) && MR_HAS_LIQUID
  return true;
#else
  return false;
#endif
}

void AmDemodulator::Reset() {
  if (impl_ == nullptr) {
    return;
  }
  impl_->DestroyObjects();
}

bool AmDemodulator::Configure(uint32_t input_sample_rate_hz, uint32_t audio_sample_rate_hz,
                               uint32_t channel_bandwidth_hz, std::string* error) {
  if (impl_ == nullptr) {
    if (error != nullptr) {
      *error = "demodulator not initialized";
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
      TargetChannelSampleRateHz(channel_bandwidth_hz, input_sample_rate_hz);
  const float channel_ratio =
      static_cast<float>(channel_sample_rate_hz) / static_cast<float>(input_sample_rate_hz);
  const float audio_ratio =
      static_cast<float>(audio_sample_rate_hz) / static_cast<float>(channel_sample_rate_hz);

  impl_->iq_channelizer = msresamp_crcf_create(channel_ratio, kResamplerStopbandAttenuationDb);
  if (impl_->iq_channelizer == nullptr) {
    if (error != nullptr) {
      *error = "failed to create channel resampler";
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

  impl_->channel_agc = agc_crcf_create();
  agc_crcf_set_bandwidth(impl_->channel_agc, 1e-3f);
  impl_->channel_rssi_db = -120.0f;

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

bool AmDemodulator::ProcessIq(const std::vector<int16_t>& interleaved_iq,
                               std::vector<int16_t>* pcm_out, AmDemodProcessStats* stats,
                               std::string* error) {
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
  if (impl_->iq_channelizer == nullptr || impl_->audio_resampler == nullptr) {
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
      stats->channel_rssi_db = impl_->channel_rssi_db;
    }
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }
  impl_->channelized_complex.resize(channelized_count);

  // Measure channelized signal level via AGC (non-destructive: write to scratch).
  if (impl_->channel_agc != nullptr) {
    impl_->agc_scratch.resize(channelized_count);
    agc_crcf_execute_block(impl_->channel_agc, impl_->channelized_complex.data(),
                           channelized_count, impl_->agc_scratch.data());
    impl_->channel_rssi_db = agc_crcf_get_rssi(impl_->channel_agc);
  }

  // Envelope detection: |z| for each channelized sample, then remove the DC offset
  // (the carrier amplitude A_c). This correctly demodulates standard AM regardless of
  // carrier frequency offset — unlike ampmodem's PLL-based coherent detector which
  // requires the carrier to be near DC and fails when peak_offset is large (e.g. 40 kHz).
  // The audio signal is encoded in the amplitude envelope: |A_c*(1+m*x(t))*e^(j*w_c*t)| = A_c*(1+m*x(t)).
  impl_->demodulated.resize(channelized_count);
  const auto* channelized =
      reinterpret_cast<const std::complex<float>*>(impl_->channelized_complex.data());
  float dc_acc = 0.0f;
  for (unsigned int i = 0; i < channelized_count; ++i) {
    const float mag = std::abs(channelized[i]);
    impl_->demodulated[i] = mag;
    dc_acc += mag;
  }
  // Block-mean DC removal keeps the audio centred at zero.
  const float dc = dc_acc / static_cast<float>(channelized_count);
  for (unsigned int i = 0; i < channelized_count; ++i) {
    impl_->demodulated[i] -= dc;
  }

  const float audio_ratio = static_cast<float>(impl_->audio_sample_rate_hz) /
                            static_cast<float>(std::max<uint32_t>(1U, impl_->channel_sample_rate_hz));
  const unsigned int max_audio_samples =
      static_cast<unsigned int>(2U + std::ceil((2.2f * audio_ratio) * static_cast<float>(channelized_count)));
  impl_->audio_resampled.resize(std::max(2U, max_audio_samples));
  unsigned int audio_count = 0;
  if (msresamp_rrrf_execute(impl_->audio_resampler, impl_->demodulated.data(), channelized_count,
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
    stats->audio_samples = audio_count;
    stats->channel_rssi_db = impl_->channel_rssi_db;
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
#endif
}

}  // namespace multi_radio
