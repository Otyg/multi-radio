#include <cstdlib>
#include <iostream>
#include <unordered_map>

#include "multi_radio/backend_config.hpp"
#include "multi_radio/backend_mode.hpp"
#include "multi_radio/server_app.hpp"

#ifndef MR_BUILD_GIT_COMMIT
#define MR_BUILD_GIT_COMMIT "unknown"
#endif

namespace {

std::string GetEnvOrDefault(const char* key, const std::string& fallback) {
  const char* value = std::getenv(key);
  return value == nullptr ? fallback : value;
}

}  // namespace

int main() {
  const std::string config_path = GetEnvOrDefault("MR_BACKEND_CONFIG", "backend.ini");
  std::unordered_map<std::string, std::string> file_values;
  std::string config_error;
  if (!multi_radio::LoadBackendConfigFile(config_path, &file_values, &config_error)) {
    std::cerr << "Failed to parse backend config file '" << config_path
              << "': " << config_error << "\n";
    return 1;
  }

  auto get_value = [&](const char* key, const std::string& fallback) {
    const char* env_value = std::getenv(key);
    if (env_value != nullptr && env_value[0] != '\0') {
      return std::string(env_value);
    }
    return multi_radio::GetConfigValue(file_values, key, fallback);
  };

  multi_radio::ServerConfig config;
  config.bind_address            = get_value("MR_BIND_ADDRESS",            config.bind_address);
  config.auth_token              = get_value("MR_AUTH_TOKEN",              config.auth_token);
  config.plugin_dir              = get_value("MR_PLUGIN_DIR",              config.plugin_dir.string());
  config.log_dir                 = get_value("MR_LOG_DIR",                 config.log_dir.string());
  config.position_bind_address   = get_value("MR_POSITION_BIND_ADDRESS",   "");
  config.position_auth_token     = get_value("MR_POSITION_AUTH_TOKEN",     "");
  config.split.mode              = multi_radio::ParseBackendMode(get_value("MR_BACKEND_MODE", "local"));
  config.split.remote_dsp_host   = get_value("MR_REMOTE_DSP_HOST", "");
  config.split.iq_transport      = get_value("MR_IQ_TRANSPORT", "grpc_stream");

  if (!multi_radio::IsValidIqTransport(config.split.iq_transport)) {
    std::cerr << "Warning: unsupported IQ transport '" << config.split.iq_transport
              << "'; falling back to grpc_stream\n";
    config.split.iq_transport = "grpc_stream";
  }

  multi_radio::ServerApp app(config);

  std::cout << "Multi-Radio server build commit: " << MR_BUILD_GIT_COMMIT << "\n";
  std::cout << "Backend mode: " << multi_radio::BackendModeToString(config.split.mode)
            << ", IQ transport: " << config.split.iq_transport << "\n";
  if (multi_radio::IsRemoteBackendMode(config.split.mode) && config.split.remote_dsp_host.empty()) {
    std::cerr << "Warning: remote DSP backend requested but MR_REMOTE_DSP_HOST is empty; "
              << "falling back to local SDR path for now.\n";
  }

  std::string error;
  if (!app.Init(&error)) {
    std::cerr << "Failed to initialize server: " << error << "\n";
    return 1;
  }

  std::cout << "Starting Multi-Radio server on " << config.bind_address << "\n";
  if (!app.Run(&error)) {
    std::cerr << "Server failed: " << error << "\n";
    return 1;
  }

  return 0;
}
