#include "multi_radio/types.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>

namespace multi_radio {

uint64_t UnixMillisNow() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string ToString(RadioMode mode) {
  switch (mode) {
    case RadioMode::kFixed:
      return "FIXED";
    case RadioMode::kScanRange:
      return "SCAN_RANGE";
    case RadioMode::kScanList:
      return "SCAN_LIST";
    case RadioMode::kAirMarinePlot:
      return "AIR_MARINE_PLOT";
  }
  return "UNKNOWN";
}

std::string ToString(SignalType signal_type) {
  switch (signal_type) {
    case SignalType::kAis:
      return "AIS";
    case SignalType::kAdsb:
      return "ADSB";
    case SignalType::kDsc:
      return "DSC";
    case SignalType::kUnknown:
    default:
      return "UNKNOWN";
  }
}

SignalType SignalTypeFromString(const std::string& value) {
  std::string upper = value;
  std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });

  if (upper == "AIS") {
    return SignalType::kAis;
  }
  if (upper == "ADSB" || upper == "ADS-B") {
    return SignalType::kAdsb;
  }
  if (upper == "DSC") {
    return SignalType::kDsc;
  }
  return SignalType::kUnknown;
}

std::string ToString(Modulation modulation) {
  switch (modulation) {
    case Modulation::kAm:
      return "AM";
    case Modulation::kWfm:
      return "WFM";
    case Modulation::kNfm:
    default:
      return "NFM";
  }
}

Modulation ModulationFromString(const std::string& value) {
  std::string upper = value;
  std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  if (upper == "AM") {
    return Modulation::kAm;
  }
  if (upper == "WFM") {
    return Modulation::kWfm;
  }
  return Modulation::kNfm;
}

}  // namespace multi_radio
