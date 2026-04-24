#include <cassert>
#include <cmath>

#include "multi_radio/scan_scheduler.hpp"

namespace {

bool Eq(double a, double b) { return std::abs(a - b) < 0.5; }

}  // namespace

int main() {
  using namespace multi_radio;

  {
    ScanScheduler scheduler;
    ModeConfig config;
    config.fixed_frequency_hz = 123456789.0;
    config.dwell_ms = 250;
    scheduler.Configure(RadioMode::kFixed, config);

    const auto f1 = scheduler.NextFrequencyHz();
    const auto f2 = scheduler.NextFrequencyHz();
    assert(f1.has_value() && Eq(f1.value(), 123456789.0));
    assert(f2.has_value() && Eq(f2.value(), 123456789.0));
    assert(scheduler.DwellMs() == 250);
  }

  {
    ScanScheduler scheduler;
    ModeConfig config;
    config.range_start_hz = 100.0;
    config.range_end_hz = 200.0;
    config.range_step_hz = 50.0;
    scheduler.Configure(RadioMode::kScanRange, config);

    assert(Eq(scheduler.NextFrequencyHz().value(), 100.0));
    assert(Eq(scheduler.NextFrequencyHz().value(), 150.0));
    assert(Eq(scheduler.NextFrequencyHz().value(), 200.0));
    assert(Eq(scheduler.NextFrequencyHz().value(), 100.0));
  }

  {
    ScanScheduler scheduler;
    ModeConfig config;
    scheduler.Configure(RadioMode::kAirMarinePlot, config);

    assert(Eq(scheduler.NextFrequencyHz().value(), 161975000.0));
    assert(Eq(scheduler.NextFrequencyHz().value(), 162025000.0));
    assert(Eq(scheduler.NextFrequencyHz().value(), 156525000.0));
    assert(Eq(scheduler.NextFrequencyHz().value(), 1090000000.0));
    assert(Eq(scheduler.NextFrequencyHz().value(), 161975000.0));
  }

  return 0;
}
