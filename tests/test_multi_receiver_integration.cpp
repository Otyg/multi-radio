#include <cassert>
#include <memory>

#include "multi_radio/event_bus.hpp"
#include "multi_radio/jsonl_logger.hpp"
#include "multi_radio/plugin_host.hpp"
#include "multi_radio/radio_device.hpp"
#include "multi_radio/receiver_manager.hpp"

namespace {

class FakeDevice final : public multi_radio::IRadioDevice {
 public:
  explicit FakeDevice(std::string serial) : serial_(std::move(serial)) {}

  std::string Serial() const override { return serial_; }

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
    center_frequency_hz_ = frequency_hz;
    return true;
  }

  bool SetSampleRateHz(uint32_t sample_rate_hz, std::string* error) override {
    if (!opened_) {
      if (error != nullptr) {
        *error = "not opened";
      }
      return false;
    }
    sample_rate_hz_ = sample_rate_hz;
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
  bool SetPpmCorrection(int /*ppm*/, std::string* /*error*/) override { return true; }

  bool ReadIq(multi_radio::IQSampleBlock* out, std::string* error) override {
    if (!opened_) {
      if (error != nullptr) {
        *error = "not opened";
      }
      return false;
    }
    if (out == nullptr) {
      if (error != nullptr) {
        *error = "null output";
      }
      return false;
    }
    out->center_frequency_hz = center_frequency_hz_;
    out->sample_rate_hz = sample_rate_hz_;
    out->interleaved_iq.assign(1024, 0);
    return true;
  }

 private:
  std::string serial_;
  bool opened_ = false;
  uint32_t center_frequency_hz_ = 0;
  uint32_t sample_rate_hz_ = 0;
  uint32_t hardware_bandwidth_hz_ = 0;
};

class FakeFactory final : public multi_radio::IRadioDeviceFactory {
 public:
  std::vector<multi_radio::ReceiverDescriptor> Enumerate() override {
    return {
        multi_radio::ReceiverDescriptor{.receiver_id = 0, .serial = "TEST000"},
        multi_radio::ReceiverDescriptor{.receiver_id = 1, .serial = "TEST001"},
    };
  }

  std::unique_ptr<multi_radio::IRadioDevice> Create(uint32_t receiver_id) override {
    if (receiver_id == 0) {
      return std::make_unique<FakeDevice>("TEST000");
    }
    return std::make_unique<FakeDevice>("TEST001");
  }
};

}  // namespace

int main() {
  using namespace multi_radio;

  auto bus = std::make_shared<EventBus>(1024);
  auto logger = std::make_shared<JsonlLogger>("/tmp/multi-radio-tests", "integration", 1024 * 1024, 2);
  auto plugins = std::make_shared<PluginHost>("/tmp/multi-radio-test-plugins");
  auto factory = std::make_unique<FakeFactory>();

  ReceiverManager manager(std::move(factory), bus, plugins, logger, nullptr);

  auto receivers = manager.ListReceivers();
  assert(receivers.size() >= 2);

  std::string error;
  assert(manager.SetMode(0, RadioMode::kFixed, &error));
  assert(manager.SetMode(1, RadioMode::kAirMarinePlot, &error));
  assert(manager.StartReceiver(0, &error));
  assert(manager.StartReceiver(1, &error));

  size_t cursor = 0;
  bool saw_rx0 = false;
  bool saw_rx1 = false;
  for (int i = 0; i < 20; ++i) {
    auto event = bus->WaitForReceiverEvent(&cursor, 500);
    if (!event.has_value()) {
      continue;
    }
    if (event->receiver_id == 0) {
      saw_rx0 = true;
    }
    if (event->receiver_id == 1) {
      saw_rx1 = true;
    }
    if (saw_rx0 && saw_rx1) {
      break;
    }
  }

  assert(saw_rx0 && saw_rx1);

  assert(manager.StopReceiver(0, &error));
  assert(manager.StopReceiver(1, &error));
  return 0;
}
