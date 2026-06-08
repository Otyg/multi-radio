#include "multi_radio/backend_mode.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace multi_radio {
namespace {

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

}  // namespace

BackendMode ParseBackendMode(const std::string& value) {
  const std::string normalized = ToLower(value);
  if (normalized == "remote" || normalized == "thick") {
    return BackendMode::kRemote;
  }
  return BackendMode::kLocal;
}

std::string BackendModeToString(BackendMode mode) {
  return mode == BackendMode::kRemote ? "remote" : "local";
}

bool IsRemoteBackendMode(BackendMode mode) {
  return mode == BackendMode::kRemote;
}

bool IsValidIqTransport(const std::string& value) {
  const std::string normalized = ToLower(value);
  return normalized == "grpc_stream" || normalized == "udp_raw";
}

}  // namespace multi_radio
