#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace multi_radio {

struct AmDemodProcessStats {
  uint32_t input_iq_samples = 0;
  uint32_t channelized_samples = 0;
  uint32_t audio_samples = 0;
  float channel_rssi_db = -120.0f;  // agc_crcf RSSI on channelized IQ
};

class AmDemodulator {
 public:
  AmDemodulator();
  ~AmDemodulator();

  static bool Available();

  void Reset();
  bool Configure(uint32_t input_sample_rate_hz, uint32_t audio_sample_rate_hz,
                 uint32_t channel_bandwidth_hz, std::string* error);
  bool ProcessIq(const std::vector<int16_t>& interleaved_iq, std::vector<int16_t>* pcm_out,
                 AmDemodProcessStats* stats, std::string* error);

 private:
  struct Impl;
  Impl* impl_ = nullptr;
};

}  // namespace multi_radio
