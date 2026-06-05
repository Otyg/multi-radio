#pragma once

#include <string>
#include <vector>

#include "radar_types.hpp"

namespace multi_radio::tui {

struct RadarViewConfig {
  bool slow_only = true;
  double center_lat = 0.0;
  double center_lon = 0.0;
  bool have_fixed_center = false;
  double range_km = 10.0;
};

struct VisibleTarget {
  RadarTargetUpdate target;
  double range_km = 0.0;
  double bearing_deg = 0.0;
  double x_km = 0.0;
  double y_km = 0.0;
  bool selected = false;
};

struct RadarFrame {
  std::vector<VisibleTarget> targets;
  double center_lat = 0.0;
  double center_lon = 0.0;
  double range_km = 10.0;
  bool slow_only = true;
  bool using_auto_center = false;
};

RadarFrame BuildRadarFrame(const std::vector<RadarTargetUpdate>& targets, const RadarViewConfig& config,
                           const std::string& selected_id);

}  // namespace multi_radio::tui
