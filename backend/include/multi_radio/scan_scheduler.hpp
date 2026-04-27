#pragma once

#include <optional>
#include <vector>

#include "multi_radio/types.hpp"

namespace multi_radio {

class ScanScheduler {
 public:
  struct ScanTarget {
    double frequency_hz = 0.0;
    uint32_t dwell_ms = 500;
    int scan_list_channel_index = -1;
  };

  ScanScheduler();

  void Configure(RadioMode mode, const ModeConfig& config);

  std::optional<ScanTarget> NextTarget();
  std::optional<double> NextFrequencyHz();
  uint32_t DwellMs() const;

 private:
  std::vector<ScanTarget> BuildScanListTargets(const ModeConfig& config) const;
  std::vector<double> BuildAirMarineFrequencies() const;
  std::vector<double> BuildRangeFrequencies(const ModeConfig& config) const;

  RadioMode mode_ = RadioMode::kFixed;
  ModeConfig config_;
  std::vector<ScanTarget> active_targets_;
  size_t cursor_ = 0;
  uint32_t last_dwell_ms_ = 500;
};

}  // namespace multi_radio
