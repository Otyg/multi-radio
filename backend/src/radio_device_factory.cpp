#include "multi_radio/radio_device.hpp"

namespace multi_radio {

std::unique_ptr<IRadioDeviceFactory> CreateDefaultRadioDeviceFactory(bool enable_rtlsdr) {
  if (enable_rtlsdr) {
    auto rtl = TryCreateRtlSdrDeviceFactory();
    if (rtl != nullptr) {
      return rtl;
    }
  }
  return CreateMockRadioDeviceFactory();
}

}  // namespace multi_radio
