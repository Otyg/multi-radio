#include "tui_radar_renderer.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace multi_radio::tui {

namespace {

constexpr double kEarthRadiusKm = 6371.0;
constexpr double kKmPerDegLat = 111.32;
constexpr double kKmPerDegLonAtEquator = 111.32;
constexpr double kPi = 3.14159265358979323846;

double ToRadians(double degrees) {
  return degrees * (kPi / 180.0);
}

double HaversineKm(double lat1, double lon1, double lat2, double lon2) {
  const double dlat = ToRadians(lat2 - lat1);
  const double dlon = ToRadians(lon2 - lon1);
  const double alat = ToRadians(lat1);
  const double blat = ToRadians(lat2);
  const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                   std::cos(alat) * std::cos(blat) *
                       std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
  return 2.0 * kEarthRadiusKm * std::asin(std::sqrt(a));
}

double BearingDegrees(double lat1, double lon1, double lat2, double lon2) {
  const double alat = ToRadians(lat1);
  const double blat = ToRadians(lat2);
  const double dlon = ToRadians(lon2 - lon1);
  const double y = std::sin(dlon) * std::cos(blat);
  const double x = std::cos(alat) * std::sin(blat) -
                   std::sin(alat) * std::cos(blat) * std::cos(dlon);
  const double bearing = std::atan2(y, x) * (180.0 / kPi);
  return std::fmod(bearing + 360.0, 360.0);
}

std::string Quote(const std::filesystem::path& path) {
  return "'" + path.string() + "'";
}

std::string SanitizeLabel(const QString& value) {
  std::string out = value.toStdString();
  std::replace(out.begin(), out.end(), '\t', ' ');
  std::replace(out.begin(), out.end(), '\n', ' ');
  return out;
}

std::string TrimTrailing(std::string line) {
  while (!line.empty() && (line.back() == ' ' || line.back() == '\r' || line.back() == '\n')) {
    line.pop_back();
  }
  return line;
}

}  // namespace

RadarFrame BuildRadarFrame(const std::vector<RadarTargetUpdate>& targets, const RadarViewConfig& config,
                           const std::string& selected_id) {
  RadarFrame frame;
  frame.range_km = std::clamp(config.range_km, 0.5, 500.0);
  frame.slow_only = config.slow_only;

  if (config.have_fixed_center) {
    frame.center_lat = config.center_lat;
    frame.center_lon = config.center_lon;
  } else {
    double sum_lat = 0.0;
    double sum_lon = 0.0;
    int count = 0;
    for (const auto& target : targets) {
      if (config.slow_only && target.kind == RadarTargetKind::kVessel && target.sog >= 1.0) {
        continue;
      }
      if (!std::isfinite(target.lat) || !std::isfinite(target.lon)) {
        continue;
      }
      sum_lat += target.lat;
      sum_lon += target.lon;
      ++count;
    }
    if (count > 0) {
      frame.center_lat = sum_lat / static_cast<double>(count);
      frame.center_lon = sum_lon / static_cast<double>(count);
      frame.using_auto_center = true;
    }
  }

  for (const auto& target : targets) {
    if (config.slow_only && target.kind == RadarTargetKind::kVessel && target.sog >= 1.0) {
      continue;
    }
    if (!std::isfinite(target.lat) || !std::isfinite(target.lon)) {
      continue;
    }

    const double dlat = target.lat - frame.center_lat;
    const double dlon = target.lon - frame.center_lon;
    const double x_km = dlon * std::cos(ToRadians(frame.center_lat)) * kKmPerDegLonAtEquator;
    const double y_km = dlat * kKmPerDegLat;
    const double range_km = HaversineKm(frame.center_lat, frame.center_lon, target.lat, target.lon);
    const double bearing_deg = BearingDegrees(frame.center_lat, frame.center_lon, target.lat, target.lon);

    VisibleTarget visible;
    visible.target = target;
    visible.range_km = range_km;
    visible.bearing_deg = bearing_deg;
    visible.x_km = x_km;
    visible.y_km = y_km;
    visible.selected = (target.id.toStdString() == selected_id);
    frame.targets.push_back(std::move(visible));
  }

  std::sort(frame.targets.begin(), frame.targets.end(), [](const VisibleTarget& lhs, const VisibleTarget& rhs) {
    return lhs.range_km < rhs.range_km;
  });

  return frame;
}

GnuplotRadarRenderer::GnuplotRadarRenderer(std::filesystem::path work_dir) : work_dir_(std::move(work_dir)) {
  std::filesystem::create_directories(work_dir_);
}

bool GnuplotRadarRenderer::IsAvailable() const {
  const_cast<GnuplotRadarRenderer*>(this)->EnsureAvailability();
  return available_;
}

std::string GnuplotRadarRenderer::LastError() const {
  return last_error_;
}

std::vector<std::string> GnuplotRadarRenderer::Render(const RadarFrame& frame, int width, int height) {
  EnsureAvailability();
  if (!available_) {
    return {last_error_};
  }

  const auto data_path = WriteDataFile(frame);
  const auto script_path = WriteScriptFile(frame, width, height, data_path);
  return ReadGnuplotOutput(script_path);
}

void GnuplotRadarRenderer::EnsureAvailability() {
  if (checked_available_) {
    return;
  }
  checked_available_ = true;
  available_ = (std::system("command -v gnuplot >/dev/null 2>&1") == 0);
  if (!available_) {
    last_error_ = "gnuplot not found in PATH";
  }
}

std::filesystem::path GnuplotRadarRenderer::WriteDataFile(const RadarFrame& frame) {
  const auto path = work_dir_ / "radar.dat";
  std::ofstream out(path);
  out << "# x_km y_km selected label\n";
  for (const auto& target : frame.targets) {
    out << target.x_km << '\t'
        << target.y_km << '\t'
        << (target.selected ? 1 : 0) << '\t'
        << SanitizeLabel(target.target.label) << '\n';
  }
  return path;
}

std::filesystem::path GnuplotRadarRenderer::WriteScriptFile(const RadarFrame& frame, int width, int height,
                                                            const std::filesystem::path& data_path) {
  const auto path = work_dir_ / "radar.gp";
  const int plot_width = std::max(20, width);
  const int plot_height = std::max(10, height);
  const double radius = std::max(0.5, frame.range_km);
  const std::array<double, 4> ring_scales{0.25, 0.5, 0.75, 1.0};

  std::ofstream out(path);
  out << "set term dumb size " << plot_width << "," << plot_height << "\n";
  out << "set size square\n";
  out << "unset key\n";
  out << "unset xtics\n";
  out << "unset ytics\n";
  out << "unset border\n";
  out << "set xrange [" << -radius << ":" << radius << "]\n";
  out << "set yrange [" << -radius << ":" << radius << "]\n";
  for (double scale : ring_scales) {
    out << "set object circle at 0,0 size " << (radius * scale)
        << " front lw 1 fc rgb \"#000000\" fillstyle empty border rgb \"#666666\"\n";
  }
  out << "set arrow 1 from -" << radius << ",0 to " << radius << ",0 nohead lw 1 lc rgb \"#666666\"\n";
  out << "set arrow 2 from 0,-" << radius << " to 0," << radius << " nohead lw 1 lc rgb \"#666666\"\n";
  out << "set label 1 \"N\" at 0," << (radius * 0.98) << " center\n";
  out << "set label 2 \"E\" at " << (radius * 0.98) << ",0 center\n";
  out << "set label 3 \"S\" at 0," << (-radius * 0.98) << " center\n";
  out << "set label 4 \"W\" at " << (-radius * 0.98) << ",0 center\n";
  out << "plot "
      << Quote(data_path)
      << " using 1:2:(($3==1)?1/0:1) with points pt 7 ps 1.0 lc rgb \"cyan\", "
      << Quote(data_path)
      << " using 1:2:(($3==1)?1:1/0) with points pt 7 ps 1.8 lc rgb \"red\", "
      << Quote(data_path)
      << " using 1:2:4 with labels offset 1,1 tc rgb \"white\"\n";
  return path;
}

std::vector<std::string> GnuplotRadarRenderer::ReadGnuplotOutput(const std::filesystem::path& script_path) {
  std::vector<std::string> lines;
  const std::string command = "gnuplot " + Quote(script_path);
  FILE* pipe = popen(command.c_str(), "r");
  if (pipe == nullptr) {
    last_error_ = "failed to spawn gnuplot";
    return {last_error_};
  }

  std::array<char, 512> buffer{};
  while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    lines.emplace_back(TrimTrailing(buffer.data()));
  }

  const int rc = pclose(pipe);
  if (rc != 0) {
    last_error_ = "gnuplot exited with status " + std::to_string(rc);
    if (lines.empty()) {
      lines.push_back(last_error_);
    }
  }
  return lines;
}

}  // namespace multi_radio::tui
