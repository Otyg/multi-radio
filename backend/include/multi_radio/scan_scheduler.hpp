#pragma once

#include <optional>
#include <vector>

#include "multi_radio/types.hpp"

namespace multi_radio {

class ScanScheduler {
 public:
  ScanScheduler();

  void Configure(RadioMode mode, const ModeConfig& config);

  std::optional<double> NextFrequencyHz();
  uint32_t DwellMs() const;

 private:
  std::vector<double> BuildAirMarineFrequencies() const;
  std::vector<double> BuildRangeFrequencies(const ModeConfig& config) const;

  RadioMode mode_ = RadioMode::kFixed;
  ModeConfig config_;
  std::vector<double> active_frequencies_;
  size_t cursor_ = 0;
};

}  // namespace multi_radio
