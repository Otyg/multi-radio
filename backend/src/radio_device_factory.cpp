#include "multi_radio/radio_device.hpp"

namespace multi_radio {

std::unique_ptr<IRadioDeviceFactory> CreateDefaultRadioDeviceFactory(bool enable_rtlsdr) {
  if (!enable_rtlsdr) {
    return nullptr;
  }
  return TryCreateRtlSdrDeviceFactory();
}

}  // namespace multi_radio
