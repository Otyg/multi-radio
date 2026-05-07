#pragma once

#include <cstdlib>
#include <fstream>
#include <string>

namespace multi_radio {

inline std::string HardwareConfigPath() {
  if (const char* p = std::getenv("MR_HARDWARE_CONFIG_PATH")) return p;
  const char* home = std::getenv("HOME");
  std::string dir  = home ? std::string(home) + "/.config/multi-radio" : ".";
  return dir + "/hardware.conf";
}

// Returns the stored ppm_correction, or 0 if the file does not exist.
inline int LoadHardwarePpm() {
  std::ifstream f(HardwareConfigPath());
  std::string line;
  while (std::getline(f, line)) {
    if (line.size() > 4 && line.substr(0, 4) == "ppm=") {
      try { return std::stoi(line.substr(4)); } catch (...) {}
    }
  }
  return 0;
}

}  // namespace multi_radio
