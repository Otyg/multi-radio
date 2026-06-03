#pragma once

#include <cstdint>
#include <limits>

#include <QMetaType>
#include <QString>
#include <QVector>

namespace multi_radio {

enum class RadarTargetKind {
  kUnknown = 0,
  kAircraft,
  kVessel,
  kFixed,
  kSarAircraft,
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
  QString symbol;  // optional single character drawn instead of the default circle
  double lat = 0.0;
  double lon = 0.0;
  bool is_base_station = false;
};

}  // namespace multi_radio

Q_DECLARE_METATYPE(QVector<multi_radio::RadarTargetUpdate>)
