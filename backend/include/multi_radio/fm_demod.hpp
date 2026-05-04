#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "multi_radio/types.hpp"

namespace multi_radio {

struct FmDemodProcessStats {
  uint32_t input_iq_samples = 0;
  uint32_t channelized_samples = 0;
  uint32_t demodulated_samples = 0;
  uint32_t audio_samples = 0;
};

class FmDemodulator {
 public:
  FmDemodulator();
  ~FmDemodulator();

  static bool Available();

  void Reset();
  bool Configure(uint32_t input_sample_rate_hz, uint32_t audio_sample_rate_hz,
                 Modulation modulation, uint32_t channel_bandwidth_hz, std::string* error);
  bool ProcessIq(const std::vector<int16_t>& interleaved_iq, std::vector<int16_t>* pcm_out,
                 FmDemodProcessStats* stats, std::string* error);

 private:
  struct Impl;
  Impl* impl_ = nullptr;
};

}  // namespace multi_radio

