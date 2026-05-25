#include <cstdlib>
#include <iostream>

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
  multi_radio::ServerConfig config;
  config.bind_address            = GetEnvOrDefault("MR_BIND_ADDRESS",            config.bind_address);
  config.auth_token              = GetEnvOrDefault("MR_AUTH_TOKEN",              config.auth_token);
  config.plugin_dir              = GetEnvOrDefault("MR_PLUGIN_DIR",              config.plugin_dir.string());
  config.log_dir                 = GetEnvOrDefault("MR_LOG_DIR",                 config.log_dir.string());
  config.position_bind_address   = GetEnvOrDefault("MR_POSITION_BIND_ADDRESS",   "");
  config.position_auth_token     = GetEnvOrDefault("MR_POSITION_AUTH_TOKEN",     "");

  multi_radio::ServerApp app(config);

  std::cout << "Multi-Radio server build commit: " << MR_BUILD_GIT_COMMIT << "\n";

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
