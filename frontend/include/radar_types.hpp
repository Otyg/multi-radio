#pragma once

#include <cstdint>
#include <limits>

#include <QString>

namespace multi_radio {

enum class RadarTargetKind {
  kUnknown = 0,
  kAircraft,
  kVessel,
  kFixed,
};

struct RadarTargetUpdate {
  QString id;
  RadarTargetKind kind = RadarTargetKind::kUnknown;
  QString label;
  double lat = 0.0;
  double lon = 0.0;
  double sog = 0.0;  // knots (if available)
  double cog = 0.0;  // degrees (if available)
  double altitude = std::numeric_limits<double>::quiet_NaN();  // feet (NaN = unknown)
  std::uint64_t unix_ms = 0;
};

struct RadarFixedObject {
  QString id;
  QString name;
  double lat = 0.0;
  double lon = 0.0;
};

}  // namespace multi_radio
