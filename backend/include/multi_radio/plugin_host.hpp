#pragma once

#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "multi_radio/plugin_api.h"
#include "multi_radio/types.hpp"

namespace multi_radio {

struct PluginMessage {
  SignalType signal_type = SignalType::kUnknown;
  double frequency_hz = 0.0;
  uint64_t unix_ms = 0;
  std::string payload;
  std::map<std::string, std::string> normalized_fields;
};

class PluginHost {
 public:
  using MessageCallback = std::function<void(const PluginMessage&)>;

  explicit PluginHost(std::filesystem::path plugin_dir);
  ~PluginHost();

  bool LoadAll(std::string* error);
  std::vector<PluginInfo> ListPlugins() const;

  bool EnablePlugin(const std::string& plugin_name, std::string* error);
  bool DisablePlugin(const std::string& plugin_name, std::string* error);

  void ProcessIq(const IQSampleBlock& iq, const MessageCallback& callback);

 private:
  struct LoadedPlugin {
    PluginInfo info;
    void* dl_handle = nullptr;
    const multi_radio_plugin_descriptor* descriptor = nullptr;
  };

  static void EmitFromPlugin(const char* signal_type, const char* payload, double frequency_hz,
                             uint64_t unix_ms, const char* normalized_kv_json, void* user_data);

  LoadedPlugin* FindPluginByName(const std::string& name);
  const LoadedPlugin* FindPluginByName(const std::string& name) const;

  std::filesystem::path plugin_dir_;
  mutable std::mutex mu_;
  std::vector<LoadedPlugin> plugins_;
};

}  // namespace multi_radio
