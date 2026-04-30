#include "multi_radio/plugin_host.hpp"

#include <algorithm>

namespace multi_radio {

PluginHost::PluginHost(std::filesystem::path plugin_dir, std::filesystem::path state_dir)
    : plugin_dir_(std::move(plugin_dir)), state_dir_(std::move(state_dir)) {}

PluginHost::~PluginHost() = default;

bool PluginHost::LoadAll(std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  plugins_.clear();
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

std::vector<PluginInfo> PluginHost::ListPlugins() const {
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<PluginInfo> out;
  out.reserve(plugins_.size());
  for (const auto& plugin : plugins_) {
    out.push_back(plugin.info);
  }
  return out;
}

bool PluginHost::EnablePlugin(const std::string& plugin_name, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  auto* plugin = FindPluginByName(plugin_name);
  if (plugin == nullptr) {
    if (error != nullptr) {
      *error = "Plugin not found: " + plugin_name;
    }
    return false;
  }
  plugin->info.enabled = true;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool PluginHost::DisablePlugin(const std::string& plugin_name, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  auto* plugin = FindPluginByName(plugin_name);
  if (plugin == nullptr) {
    if (error != nullptr) {
      *error = "Plugin not found: " + plugin_name;
    }
    return false;
  }
  plugin->info.enabled = false;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

void PluginHost::ProcessIq(const IQSampleBlock& iq, const MessageCallback& callback) {
  (void)iq;
  (void)callback;
}

void PluginHost::EmitFromPlugin(const char* signal_type, const char* payload, double frequency_hz,
                                uint64_t unix_ms, const char* normalized_kv_json,
                                void* user_data) {
  (void)signal_type;
  (void)payload;
  (void)frequency_hz;
  (void)unix_ms;
  (void)normalized_kv_json;
  (void)user_data;
}

PluginHost::LoadedPlugin* PluginHost::FindPluginByName(const std::string& name) {
  auto it = std::find_if(plugins_.begin(), plugins_.end(), [&](const LoadedPlugin& plugin) {
    return plugin.info.plugin_name == name;
  });
  return it == plugins_.end() ? nullptr : &(*it);
}

const PluginHost::LoadedPlugin* PluginHost::FindPluginByName(const std::string& name) const {
  auto it = std::find_if(plugins_.begin(), plugins_.end(), [&](const LoadedPlugin& plugin) {
    return plugin.info.plugin_name == name;
  });
  return it == plugins_.end() ? nullptr : &(*it);
}

}  // namespace multi_radio
