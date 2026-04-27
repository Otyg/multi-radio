#include "multi_radio/scan_scheduler.hpp"

#include <cmath>

namespace multi_radio {

namespace {
constexpr double kAisMidHz = 162000000.0;
constexpr double kDscCh70Hz = 156525000.0;
constexpr uint32_t kAirMarineFixedDwellMs = 5000;
}  // namespace

ScanScheduler::ScanScheduler() = default;

void ScanScheduler::Configure(RadioMode mode, const ModeConfig& config) {
  mode_ = mode;
  config_ = config;
  cursor_ = 0;
  active_targets_.clear();
  last_dwell_ms_ = config_.dwell_ms == 0 ? 500 : config_.dwell_ms;

  switch (mode_) {
    case RadioMode::kFixed:
      if (config_.fixed_frequency_hz > 0.0) {
        active_targets_.push_back(ScanTarget{
            .frequency_hz = config_.fixed_frequency_hz,
            .dwell_ms = config_.dwell_ms == 0 ? 500 : config_.dwell_ms,
            .scan_list_channel_index = -1,
        });
      }
      break;
    case RadioMode::kScanRange:
      for (double frequency_hz : BuildRangeFrequencies(config_)) {
        active_targets_.push_back(ScanTarget{
            .frequency_hz = frequency_hz,
            .dwell_ms = config_.dwell_ms == 0 ? 500 : config_.dwell_ms,
            .scan_list_channel_index = -1,
        });
      }
      break;
    case RadioMode::kScanList:
      active_targets_ = BuildScanListTargets(config_);
      break;
    case RadioMode::kAirMarinePlot:
      for (double frequency_hz : BuildAirMarineFrequencies()) {
        active_targets_.push_back(ScanTarget{
            .frequency_hz = frequency_hz,
            .dwell_ms = kAirMarineFixedDwellMs,
            .scan_list_channel_index = -1,
        });
      }
      break;
  }
}

std::optional<ScanScheduler::ScanTarget> ScanScheduler::NextTarget() {
  if (active_targets_.empty()) {
    return std::nullopt;
  }
  const ScanTarget target = active_targets_[cursor_];
  cursor_ = (cursor_ + 1) % active_targets_.size();
  last_dwell_ms_ = target.dwell_ms;
  return target;
}

std::optional<double> ScanScheduler::NextFrequencyHz() {
  const auto target = NextTarget();
  if (!target.has_value()) {
    return std::nullopt;
  }
  return target->frequency_hz;
}

uint32_t ScanScheduler::DwellMs() const {
  if (mode_ == RadioMode::kAirMarinePlot) {
    return kAirMarineFixedDwellMs;
  }
  return last_dwell_ms_ == 0 ? 500 : last_dwell_ms_;
}

std::vector<double> ScanScheduler::BuildAirMarineFrequencies() const {
  return {kAisMidHz, kDscCh70Hz};
}

std::vector<ScanScheduler::ScanTarget> ScanScheduler::BuildScanListTargets(const ModeConfig& config) const {
  std::vector<ScanTarget> targets;
  const uint32_t fallback_dwell_ms = config.dwell_ms == 0 ? 500 : config.dwell_ms;
  if (!config.scan_list_channels.empty()) {
    for (size_t index = 0; index < config.scan_list_channels.size(); ++index) {
      const auto& channel = config.scan_list_channels[index];
      if (channel.frequency_hz <= 0.0) {
        continue;
      }
      targets.push_back(ScanTarget{
          .frequency_hz = channel.frequency_hz,
          .dwell_ms = channel.dwell_ms == 0 ? fallback_dwell_ms : channel.dwell_ms,
          .scan_list_channel_index = static_cast<int>(index),
      });
    }
    return targets;
  }

  for (double frequency_hz : config.frequency_list_hz) {
    if (frequency_hz <= 0.0) {
      continue;
    }
    targets.push_back(ScanTarget{
        .frequency_hz = frequency_hz,
        .dwell_ms = fallback_dwell_ms,
        .scan_list_channel_index = -1,
    });
  }
  return targets;
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
