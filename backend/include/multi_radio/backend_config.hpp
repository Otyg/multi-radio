#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>

namespace multi_radio {

bool LoadBackendConfigFile(const std::filesystem::path& path,
                           std::unordered_map<std::string, std::string>* values,
                           std::string* error);

std::string GetConfigValue(const std::unordered_map<std::string, std::string>& values,
                           const std::string& key,
                           const std::string& fallback);

}  // namespace multi_radio
