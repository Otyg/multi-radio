#include <cassert>
#include <filesystem>
#include <fstream>
#include <unordered_map>

#include "multi_radio/backend_config.hpp"

int main() {
  using multi_radio::GetConfigValue;
  using multi_radio::LoadBackendConfigFile;

  const std::filesystem::path path = "./backend_config_test.ini";
  {
    std::ofstream out(path);
    out << "# comment\n";
    out << "bind_address=0.0.0.0:50052\n";
    out << "auth_token=secret-token\n";
    out << "[section]\n";
    out << "log_dir=./logs-test\n";
  }

  std::unordered_map<std::string, std::string> values;
  std::string error;
  assert(LoadBackendConfigFile(path, &values, &error));
  assert(error.empty());
  assert(GetConfigValue(values, "bind_address", "") == "0.0.0.0:50052");
  assert(GetConfigValue(values, "auth_token", "") == "secret-token");
  assert(GetConfigValue(values, "log_dir", "") == "./logs-test");

  std::filesystem::remove(path);
  return 0;
}
