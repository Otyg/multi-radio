#include <cassert>
#include <memory>

#include "multi_radio/event_bus.hpp"
#include "multi_radio/jsonl_logger.hpp"
#include "multi_radio/plugin_host.hpp"
#include "multi_radio/receiver_worker.hpp"

namespace {

class FakeDevice : public multi_radio::IRadioDevice {
 public:
  std::string Serial() const override { return "TEST"; }
  bool Open(std::string* /*error*/) override {
    opened_ = true;
    return true;
  }
  void Close() override { opened_ = false; }

  bool SetCenterFrequencyHz(uint32_t frequency_hz, std::string* error) override {
    if (!opened_) {
      if (error != nullptr) {
        *error = "not opened";
      }
      return false;
    }
    last_frequency_ = frequency_hz;
    return true;
  }
  bool SetSampleRateHz(uint32_t sample_rate_hz, std::string* error) override {
    if (!opened_) {
      if (error != nullptr) {
        *error = "not opened";
      }
      return false;
    }
    sample_rate_ = sample_rate_hz;
    return true;
  }
  bool SetHardwareBandwidthHz(uint32_t hardware_bandwidth_hz, std::string* error) override {
    if (!opened_) {
      if (error != nullptr) {
        *error = "not opened";
      }
      return false;
    }
    hardware_bandwidth_hz_ = hardware_bandwidth_hz;
    return true;
  }
  bool SetGainTenthdB(int /*gain_tenth_db*/, std::string* /*error*/) override { return true; }

  bool ReadIq(multi_radio::IQSampleBlock* out, std::string* error) override {
    if (!opened_) {
      if (error != nullptr) {
        *error = "not opened";
      }
      return false;
    }
    out->center_frequency_hz = last_frequency_;
    out->sample_rate_hz = sample_rate_;
    out->interleaved_iq.assign(1024, 0);
    return true;
  }

 private:
  bool opened_ = false;
  uint32_t last_frequency_ = 0;
  uint32_t sample_rate_ = 0;
  uint32_t hardware_bandwidth_hz_ = 0;
};

}  // namespace

int main() {
  using namespace multi_radio;

  auto bus = std::make_shared<EventBus>(128);
  auto logger = std::make_shared<JsonlLogger>("/tmp/multi-radio-tests", "state", 1024 * 1024, 2);
  auto plugins = std::make_shared<PluginHost>("/tmp/multi-radio-test-plugins");

  auto fake = std::make_unique<FakeDevice>();
  ReceiverWorker worker(5, "SERIAL5", std::move(fake), bus, plugins, logger);

  std::string error;
  assert(worker.Start(&error));

  ModeConfig config;
  config.range_start_hz = 1000000;
  config.range_end_hz = 1010000;
  config.range_step_hz = 5000;
  config.dwell_ms = 100;

  assert(worker.SetMode(RadioMode::kScanRange, &error));
  assert(worker.SetModeConfig(config, &error));

  const auto status = worker.Status();
  assert(status.running);
  assert(status.mode == RadioMode::kScanRange);
  assert(status.mode_config.dwell_ms == 100);

  assert(worker.Stop(&error));
  assert(!worker.Status().running);

  return 0;
}
