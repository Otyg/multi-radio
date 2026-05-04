#include "multi_radio/radio_device.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if defined(MR_HAS_RTLSDR)
#include <rtl-sdr.h>
#endif

namespace multi_radio {

#if defined(MR_HAS_RTLSDR)
namespace {

constexpr uint32_t kDefaultReadBlockBytes = 64U * 1024U;

std::string DeviceSerialOrFallback(uint32_t index) {
  char manufacturer[256] = {0};
  char product[256] = {0};
  char serial[256] = {0};
  if (rtlsdr_get_device_usb_strings(index, manufacturer, product, serial) == 0 && serial[0] != '\0') {
    return std::string(serial);
  }
  return "rtl-" + std::to_string(index);
}

class RtlSdrDevice final : public IRadioDevice {
 public:
  RtlSdrDevice(uint32_t index, std::string serial) : index_(index), serial_(std::move(serial)) {}

  ~RtlSdrDevice() override { Close(); }

  std::string Serial() const override { return serial_; }

  bool Open(std::string* error) override {
    if (device_ != nullptr) {
      if (error != nullptr) {
        error->clear();
      }
      return true;
    }

    rtlsdr_dev_t* device = nullptr;
    const int rc = rtlsdr_open(&device, index_);
    if (rc != 0 || device == nullptr) {
      if (error != nullptr) {
        *error = "rtlsdr_open failed rc=" + std::to_string(rc);
      }
      return false;
    }

    device_ = device;
    rtlsdr_reset_buffer(device_);
    rtlsdr_set_tuner_gain_mode(device_, 0);  // auto gain for first-stage integration
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  void Close() override {
    if (device_ != nullptr) {
      rtlsdr_close(device_);
      device_ = nullptr;
    }
  }

  bool SetCenterFrequencyHz(uint32_t frequency_hz, std::string* error) override {
    if (device_ == nullptr) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    const int rc = rtlsdr_set_center_freq(device_, frequency_hz);
    if (rc != 0) {
      if (error != nullptr) {
        *error = "rtlsdr_set_center_freq failed rc=" + std::to_string(rc);
      }
      return false;
    }
    center_frequency_hz_ = frequency_hz;
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool SetSampleRateHz(uint32_t sample_rate_hz, std::string* error) override {
    if (device_ == nullptr) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    const int rc = rtlsdr_set_sample_rate(device_, sample_rate_hz);
    if (rc != 0) {
      if (error != nullptr) {
        *error = "rtlsdr_set_sample_rate failed rc=" + std::to_string(rc);
      }
      return false;
    }
    sample_rate_hz_ = sample_rate_hz;
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool SetHardwareBandwidthHz(uint32_t bandwidth_hz, std::string* error) override {
    // librtlsdr does not expose generic tuner bandwidth control across all tuners.
    (void)bandwidth_hz;
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool SetGainTenthdB(int gain_tenth_db, std::string* error) override {
    if (device_ == nullptr) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    if (gain_tenth_db < 0) {
      const int rc_mode = rtlsdr_set_tuner_gain_mode(device_, 0);
      if (rc_mode != 0) {
        if (error != nullptr) {
          *error = "rtlsdr_set_tuner_gain_mode(auto) failed rc=" + std::to_string(rc_mode);
        }
        return false;
      }
      if (error != nullptr) {
        error->clear();
      }
      return true;
    }

    const int rc_mode = rtlsdr_set_tuner_gain_mode(device_, 1);
    if (rc_mode != 0) {
      if (error != nullptr) {
        *error = "rtlsdr_set_tuner_gain_mode(manual) failed rc=" + std::to_string(rc_mode);
      }
      return false;
    }
    const int rc_gain = rtlsdr_set_tuner_gain(device_, gain_tenth_db);
    if (rc_gain != 0) {
      if (error != nullptr) {
        *error = "rtlsdr_set_tuner_gain failed rc=" + std::to_string(rc_gain);
      }
      return false;
    }
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

  bool ReadIq(IQSampleBlock* out, std::string* error) override {
    if (device_ == nullptr) {
      if (error != nullptr) {
        *error = "device not opened";
      }
      return false;
    }
    if (out == nullptr) {
      if (error != nullptr) {
        *error = "output block is null";
      }
      return false;
    }

    read_buffer_.resize(kDefaultReadBlockBytes);
    int bytes_read = 0;
    const int rc = rtlsdr_read_sync(device_, read_buffer_.data(), static_cast<int>(read_buffer_.size()),
                                    &bytes_read);
    if (rc != 0 || bytes_read <= 0) {
      if (error != nullptr) {
        *error = "rtlsdr_read_sync failed rc=" + std::to_string(rc) + " bytes=" + std::to_string(bytes_read);
      }
      return false;
    }
    if ((bytes_read & 1) != 0) {
      --bytes_read;
    }

    out->interleaved_iq.resize(static_cast<size_t>(bytes_read));
    for (int i = 0; i < bytes_read; ++i) {
      // Map unsigned 8-bit IQ [0,255] to signed 16-bit centered around zero.
      out->interleaved_iq[static_cast<size_t>(i)] =
          static_cast<int16_t>((static_cast<int>(read_buffer_[static_cast<size_t>(i)]) - 128) << 8);
    }
    out->sample_rate_hz = sample_rate_hz_;
    out->center_frequency_hz = center_frequency_hz_;
    if (error != nullptr) {
      error->clear();
    }
    return true;
  }

 private:
  const uint32_t index_;
  const std::string serial_;
  rtlsdr_dev_t* device_ = nullptr;
  uint32_t sample_rate_hz_ = 0;
  uint32_t center_frequency_hz_ = 0;
  std::vector<unsigned char> read_buffer_;
};

class RtlSdrFactory final : public IRadioDeviceFactory {
 public:
  std::vector<ReceiverDescriptor> Enumerate() override {
    std::vector<ReceiverDescriptor> out;
    const uint32_t count = rtlsdr_get_device_count();
    out.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
      out.push_back(ReceiverDescriptor{.receiver_id = i, .serial = DeviceSerialOrFallback(i)});
    }
    return out;
  }

  std::unique_ptr<IRadioDevice> Create(uint32_t receiver_id) override {
    const uint32_t count = rtlsdr_get_device_count();
    if (receiver_id >= count) {
      return nullptr;
    }
    return std::make_unique<RtlSdrDevice>(receiver_id, DeviceSerialOrFallback(receiver_id));
  }
};

}  // namespace
#endif

std::unique_ptr<IRadioDeviceFactory> CreateRtlSdrFactory() {
#if defined(MR_HAS_RTLSDR)
  return std::make_unique<RtlSdrFactory>();
#else
  return nullptr;
#endif
}

}  // namespace multi_radio
