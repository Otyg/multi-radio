#include "multi_radio/radio_device.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#if defined(MULTI_RADIO_WITH_RTLSDR)
#include <rtl-sdr.h>
#endif

namespace multi_radio {

#if defined(MULTI_RADIO_WITH_RTLSDR)

namespace {

class RtlSdrDevice final : public IRadioDevice {
 public:
  RtlSdrDevice(uint32_t index, std::string serial) : index_(index), serial_(std::move(serial)) {}
  ~RtlSdrDevice() override { Close(); }

  std::string Serial() const override { return serial_; }

  bool Open(std::string* error) override {
    if (dev_ != nullptr) {
      return true;
    }
    if (rtlsdr_open(&dev_, index_) != 0) {
      if (error != nullptr) {
        *error = "rtlsdr_open failed";
      }
      return false;
    }
    rtlsdr_reset_buffer(dev_);
    return true;
  }

  void Close() override {
    if (dev_ != nullptr) {
      rtlsdr_close(dev_);
      dev_ = nullptr;
    }
  }

  bool SetCenterFrequencyHz(uint32_t frequency_hz, std::string* error) override {
    if (dev_ == nullptr) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    if (rtlsdr_set_center_freq(dev_, frequency_hz) != 0) {
      if (error != nullptr) {
        *error = "rtlsdr_set_center_freq failed";
      }
      return false;
    }
    center_frequency_hz_ = frequency_hz;
    return true;
  }

  bool SetSampleRateHz(uint32_t sample_rate_hz, std::string* error) override {
    if (dev_ == nullptr) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    if (rtlsdr_set_sample_rate(dev_, sample_rate_hz) != 0) {
      if (error != nullptr) {
        *error = "rtlsdr_set_sample_rate failed";
      }
      return false;
    }
    sample_rate_hz_ = sample_rate_hz;
    return true;
  }

  bool SetGainTenthdB(int gain_tenth_db, std::string* error) override {
    if (dev_ == nullptr) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    rtlsdr_set_tuner_gain_mode(dev_, 1);
    if (rtlsdr_set_tuner_gain(dev_, gain_tenth_db) != 0) {
      if (error != nullptr) {
        *error = "rtlsdr_set_tuner_gain failed";
      }
      return false;
    }
    return true;
  }

  bool ReadIq(IQSampleBlock* out, std::string* error) override {
    if (dev_ == nullptr) {
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

    constexpr int kBytes = 131072;
    std::vector<uint8_t> raw(kBytes);
    int n_read = 0;
    if (rtlsdr_read_sync(dev_, raw.data(), kBytes, &n_read) != 0 || n_read <= 0) {
      if (error != nullptr) {
        *error = "rtlsdr_read_sync failed";
      }
      return false;
    }

    out->interleaved_iq.resize(static_cast<size_t>(n_read));
    for (int i = 0; i < n_read; ++i) {
      out->interleaved_iq[static_cast<size_t>(i)] = static_cast<int16_t>((int(raw[i]) - 127) * 256);
    }
    out->sample_rate_hz = sample_rate_hz_;
    out->center_frequency_hz = center_frequency_hz_;
    return true;
  }

 private:
  uint32_t index_;
  std::string serial_;
  rtlsdr_dev_t* dev_ = nullptr;
  uint32_t center_frequency_hz_ = 100000000;
  uint32_t sample_rate_hz_ = 2048000;
};

class RtlSdrDeviceFactory final : public IRadioDeviceFactory {
 public:
  std::vector<ReceiverDescriptor> Enumerate() override {
    std::vector<ReceiverDescriptor> devices;
    const uint32_t count = rtlsdr_get_device_count();
    for (uint32_t i = 0; i < count; ++i) {
      char manufacturer[256];
      char product[256];
      char serial[256];
      rtlsdr_get_device_usb_strings(i, manufacturer, product, serial);
      devices.push_back(ReceiverDescriptor{.receiver_id = i, .serial = serial});
    }
    return devices;
  }

  std::unique_ptr<IRadioDevice> Create(uint32_t receiver_id) override {
    char manufacturer[256];
    char product[256];
    char serial[256];
    rtlsdr_get_device_usb_strings(receiver_id, manufacturer, product, serial);
    return std::make_unique<RtlSdrDevice>(receiver_id, serial);
  }
};

}  // namespace

#endif

std::unique_ptr<IRadioDeviceFactory> TryCreateRtlSdrDeviceFactory() {
#if defined(MULTI_RADIO_WITH_RTLSDR)
  auto factory = std::make_unique<RtlSdrDeviceFactory>();
  if (factory->Enumerate().empty()) {
    return nullptr;
  }
  return factory;
#else
  return nullptr;
#endif
}

bool IsRtlSdrBackendCompiled() {
#if defined(MULTI_RADIO_WITH_RTLSDR)
  return true;
#else
  return false;
#endif
}

}  // namespace multi_radio
