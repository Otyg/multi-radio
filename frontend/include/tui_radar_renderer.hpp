#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "tui_radar_types.hpp"

namespace multi_radio::tui {

class GnuplotRadarRenderer {
 public:
  explicit GnuplotRadarRenderer(std::filesystem::path work_dir);

  bool IsAvailable() const;
  std::string LastError() const;

  std::vector<std::string> Render(const RadarFrame& frame, int width, int height);

 private:
  std::filesystem::path work_dir_;
  mutable bool checked_available_ = false;
  mutable bool available_ = false;
  std::string last_error_;

  void EnsureAvailability();
  std::filesystem::path WriteDataFile(const RadarFrame& frame);
  std::filesystem::path WriteScriptFile(const RadarFrame& frame, int width, int height,
                                        const std::filesystem::path& data_path);
  std::vector<std::string> ReadGnuplotOutput(const std::filesystem::path& script_path);
};

}  // namespace multi_radio::tui
