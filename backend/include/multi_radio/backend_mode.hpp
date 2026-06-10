#pragma once

#include <string>

namespace multi_radio {

enum class BackendMode {
  kLocal,
  kRemote,
};

struct BackendSplitConfig {
  BackendMode mode = BackendMode::kLocal;
  std::string remote_dsp_host;
  std::string iq_transport = "grpc_stream";
};

BackendMode ParseBackendMode(const std::string& value);
std::string BackendModeToString(BackendMode mode);
bool IsRemoteBackendMode(BackendMode mode);
bool IsValidIqTransport(const std::string& value);
bool IsImplementedBackendMode(BackendMode mode);

}  // namespace multi_radio
