#include "multi_radio/backend_config.hpp"

#include <cctype>
#include <fstream>
#include <sstream>

namespace multi_radio {
namespace {

std::string Trim(std::string value) {
  const auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };
  while (!value.empty() && is_space(static_cast<unsigned char>(value.front()))) {
    value.erase(value.begin());
  }
  while (!value.empty() && is_space(static_cast<unsigned char>(value.back()))) {
    value.pop_back();
  }
  return value;
}

}  // namespace

bool LoadBackendConfigFile(const std::filesystem::path& path,
                           std::unordered_map<std::string, std::string>* values,
                           std::string* error) {
  if (values == nullptr) {
    if (error != nullptr) *error = "null config map";
    return false;
  }

  values->clear();
  std::ifstream input(path);
  if (!input.is_open()) {
    return true;
  }

  std::string line;
  std::size_t line_no = 0;
  while (std::getline(input, line)) {
    ++line_no;
    std::string trimmed = Trim(line);
    if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';') {
      continue;
    }
    if (trimmed.front() == '[' && trimmed.back() == ']') {
      continue;
    }

    const std::size_t pos = trimmed.find('=');
    if (pos == std::string::npos) {
      if (error != nullptr) {
        *error = "invalid setting on line " + std::to_string(line_no) + ": " + trimmed;
      }
      return false;
    }

    const std::string key = Trim(trimmed.substr(0, pos));
    const std::string value = Trim(trimmed.substr(pos + 1));
    if (key.empty()) {
      if (error != nullptr) {
        *error = "empty key on line " + std::to_string(line_no);
      }
      return false;
    }
    (*values)[key] = value;
  }
  return true;
}

std::string GetConfigValue(const std::unordered_map<std::string, std::string>& values,
                           const std::string& key,
                           const std::string& fallback) {
  const auto it = values.find(key);
  return it == values.end() ? fallback : it->second;
}

}  // namespace multi_radio
