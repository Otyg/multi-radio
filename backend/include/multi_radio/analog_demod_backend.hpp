#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "multi_radio/types.hpp"

namespace multi_radio {

struct AnalogDemodRequest {
  const IQSampleBlock* block = nullptr;
  uint32_t input_sample_rate_hz = 0;
  uint32_t audio_sample_rate_hz = 0;
  uint32_t channel_bandwidth_hz = 0;
  Modulation modulation = Modulation::kNfm;
};

struct AnalogDemodResult {
  std::vector<int16_t> pcm_s16le;
  float channel_rssi_db = -120.0f;
};

class IAnalogDemodBackend {
 public:
  virtual ~IAnalogDemodBackend() = default;

  virtual std::string Name() const = 0;
  virtual bool Supports(Modulation modulation) const = 0;
  virtual std::string UnavailableReason(Modulation modulation) const = 0;
  virtual void Reset() = 0;
  virtual bool Process(const AnalogDemodRequest& request, AnalogDemodResult* result,
                       std::string* error) = 0;
};

std::unique_ptr<IAnalogDemodBackend> CreateAnalogDemodBackend(std::string* selected_backend_name,
                                                              std::string* selection_warning);

}  // namespace multi_radio
