#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "multi_radio/types.hpp"

namespace multi_radio {

class IRadioDevice {
 public:
  virtual ~IRadioDevice() = default;

  virtual std::string Serial() const = 0;
  virtual bool Open(std::string* error) = 0;
  virtual void Close() = 0;

  virtual bool SetCenterFrequencyHz(uint32_t frequency_hz, std::string* error) = 0;
  virtual bool SetSampleRateHz(uint32_t sample_rate_hz, std::string* error) = 0;
  virtual bool SetGainTenthdB(int gain_tenth_db, std::string* error) = 0;
  virtual bool ReadIq(IQSampleBlock* out, std::string* error) = 0;
};

class IRadioDeviceFactory {
 public:
  virtual ~IRadioDeviceFactory() = default;

  virtual std::vector<ReceiverDescriptor> Enumerate() = 0;
  virtual std::unique_ptr<IRadioDevice> Create(uint32_t receiver_id) = 0;
};

std::unique_ptr<IRadioDeviceFactory> CreateMockRadioDeviceFactory();
std::unique_ptr<IRadioDeviceFactory> TryCreateRtlSdrDeviceFactory();
std::unique_ptr<IRadioDeviceFactory> CreateDefaultRadioDeviceFactory(bool enable_rtlsdr);

}  // namespace multi_radio
