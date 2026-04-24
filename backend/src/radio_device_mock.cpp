#include "multi_radio/radio_device.hpp"

#include <cmath>
#include <memory>

namespace multi_radio {

namespace {

constexpr double kPi = 3.14159265358979323846;

class MockRadioDevice final : public IRadioDevice {
 public:
  explicit MockRadioDevice(std::string serial) : serial_(std::move(serial)) {}

  std::string Serial() const override { return serial_; }

  bool Open(std::string* /*error*/) override {
    opened_ = true;
    return true;
  }

  void Close() override { opened_ = false; }

  bool SetCenterFrequencyHz(uint32_t frequency_hz, std::string* error) override {
    if (!opened_) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    center_frequency_hz_ = frequency_hz;
    return true;
  }

  bool SetSampleRateHz(uint32_t sample_rate_hz, std::string* error) override {
    if (!opened_) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    sample_rate_hz_ = sample_rate_hz;
    return true;
  }

  bool SetGainTenthdB(int /*gain_tenth_db*/, std::string* error) override {
    if (!opened_) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    return true;
  }

  bool ReadIq(IQSampleBlock* out, std::string* error) override {
    if (!opened_) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    if (out == nullptr) {
      if (error != nullptr) {
        *error = "output buffer null";
      }
      return false;
    }

    constexpr size_t kSamples = 16384;
    out->interleaved_iq.resize(kSamples * 2);
    out->sample_rate_hz = sample_rate_hz_;
    out->center_frequency_hz = center_frequency_hz_;

    for (size_t i = 0; i < kSamples; ++i) {
      const double t = static_cast<double>(i + phase_) / static_cast<double>(kSamples);
      const double i_sample = std::sin(2.0 * kPi * t) * 12000.0;
      const double q_sample = std::cos(2.0 * kPi * t) * 12000.0;
      out->interleaved_iq[i * 2] = static_cast<int16_t>(i_sample);
      out->interleaved_iq[i * 2 + 1] = static_cast<int16_t>(q_sample);
    }
    phase_ += kSamples;
    return true;
  }

 private:
  std::string serial_;
  bool opened_ = false;
  uint32_t center_frequency_hz_ = 100000000;
  uint32_t sample_rate_hz_ = 2048000;
  size_t phase_ = 0;
};

class MockRadioDeviceFactory final : public IRadioDeviceFactory {
 public:
  std::vector<ReceiverDescriptor> Enumerate() override {
    return {
        ReceiverDescriptor{.receiver_id = 0, .serial = "MOCKRTL000"},
        ReceiverDescriptor{.receiver_id = 1, .serial = "MOCKRTL001"},
    };
  }

  std::unique_ptr<IRadioDevice> Create(uint32_t receiver_id) override {
    return std::make_unique<MockRadioDevice>("MOCKRTL" + std::to_string(receiver_id));
  }
};

}  // namespace

std::unique_ptr<IRadioDeviceFactory> CreateMockRadioDeviceFactory() {
  return std::make_unique<MockRadioDeviceFactory>();
}

}  // namespace multi_radio
