#include "multi_radio/scan_scheduler.hpp"

#include <cmath>

namespace multi_radio {

namespace {
constexpr double kAisMidHz = 162000000.0;
}  // namespace

ScanScheduler::ScanScheduler() = default;

void ScanScheduler::Configure(RadioMode mode, const ModeConfig& config) {
  mode_ = mode;
  config_ = config;
  cursor_ = 0;
  active_frequencies_.clear();

  switch (mode_) {
    case RadioMode::kFixed:
      if (config_.fixed_frequency_hz > 0.0) {
        active_frequencies_.push_back(config_.fixed_frequency_hz);
      }
      break;
    case RadioMode::kScanRange:
      active_frequencies_ = BuildRangeFrequencies(config_);
      break;
    case RadioMode::kScanList:
      active_frequencies_ = config_.frequency_list_hz;
      break;
    case RadioMode::kAirMarinePlot:
      active_frequencies_ = BuildAirMarineFrequencies();
      break;
  }
}

std::optional<double> ScanScheduler::NextFrequencyHz() {
  if (active_frequencies_.empty()) {
    return std::nullopt;
  }
  const double frequency = active_frequencies_[cursor_];
  cursor_ = (cursor_ + 1) % active_frequencies_.size();
  return frequency;
}

uint32_t ScanScheduler::DwellMs() const {
  return config_.dwell_ms == 0 ? 500 : config_.dwell_ms;
}

std::vector<double> ScanScheduler::BuildAirMarineFrequencies() const {
  return {kAisMidHz};
}

std::vector<double> ScanScheduler::BuildRangeFrequencies(const ModeConfig& config) const {
  std::vector<double> frequencies;
  if (config.range_start_hz <= 0.0 || config.range_end_hz <= 0.0 ||
      config.range_step_hz <= 0.0 || config.range_end_hz < config.range_start_hz) {
    return frequencies;
  }

  const int max_steps = static_cast<int>(
      std::floor((config.range_end_hz - config.range_start_hz) / config.range_step_hz));
  for (int step = 0; step <= max_steps; ++step) {
    frequencies.push_back(config.range_start_hz + config.range_step_hz * step);
  }

  if (frequencies.empty() || frequencies.back() < config.range_end_hz) {
    frequencies.push_back(config.range_end_hz);
  }

  return frequencies;
}

}  // namespace multi_radio
