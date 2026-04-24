#include "multi_radio/plugin_host.hpp"

#include <dlfcn.h>

#include <algorithm>
#include <cstring>
#include <sstream>

namespace multi_radio {

namespace {

std::vector<SignalType> ParseSignalCsv(const std::string& csv) {
  std::vector<SignalType> out;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) {
      out.push_back(SignalTypeFromString(item));
    }
  }
  return out;
}

std::map<std::string, std::string> ParseFlatJsonMap(const std::string& json) {
  // Minimal parser for compact JSON object: {"k":"v",...}. This keeps MVP dependency-light.
  std::map<std::string, std::string> out;
  size_t pos = 0;
  while (true) {
    pos = json.find('"', pos);
    if (pos == std::string::npos) {
      break;
    }
    const size_t key_start = pos + 1;
    const size_t key_end = json.find('"', key_start);
    if (key_end == std::string::npos) {
      break;
    }
    const std::string key = json.substr(key_start, key_end - key_start);
    const size_t colon = json.find(':', key_end);
    if (colon == std::string::npos) {
      break;
    }
    const size_t value_quote = json.find('"', colon);
    if (value_quote == std::string::npos) {
      break;
    }
    const size_t value_start = value_quote + 1;
    const size_t value_end = json.find('"', value_start);
    if (value_end == std::string::npos) {
      break;
    }
    const std::string value = json.substr(value_start, value_end - value_start);
    out[key] = value;
    pos = value_end + 1;
  }
  return out;
}

struct EmitContext {
  PluginHost::MessageCallback callback;
};

}  // namespace

PluginHost::PluginHost(std::filesystem::path plugin_dir) : plugin_dir_(std::move(plugin_dir)) {}

PluginHost::~PluginHost() {
  std::lock_guard<std::mutex> lock(mu_);
  for (auto& plugin : plugins_) {
    if (plugin.descriptor != nullptr && plugin.descriptor->shutdown != nullptr) {
      plugin.descriptor->shutdown();
    }
    if (plugin.dl_handle != nullptr) {
      dlclose(plugin.dl_handle);
      plugin.dl_handle = nullptr;
    }
  }
}

bool PluginHost::LoadAll(std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  plugins_.clear();

  if (!std::filesystem::exists(plugin_dir_)) {
    if (error != nullptr) {
      *error = "Plugin directory does not exist: " + plugin_dir_.string();
    }
    return false;
  }

  for (const auto& entry : std::filesystem::directory_iterator(plugin_dir_)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".so") {
      continue;
    }

    void* handle = dlopen(entry.path().c_str(), RTLD_NOW);
    if (handle == nullptr) {
      if (error != nullptr) {
        *error = std::string("Failed to load plugin ") + entry.path().string() + ": " + dlerror();
      }
      return false;
    }

    auto* symbol = dlsym(handle, "multi_radio_get_plugin_descriptor");
    if (symbol == nullptr) {
      dlclose(handle);
      if (error != nullptr) {
        *error = "Plugin missing required symbol multi_radio_get_plugin_descriptor: " +
                 entry.path().string();
      }
      return false;
    }

    auto get_descriptor = reinterpret_cast<multi_radio_get_plugin_descriptor_fn>(symbol);
    const multi_radio_plugin_descriptor* descriptor = get_descriptor();
    if (descriptor == nullptr) {
      dlclose(handle);
      if (error != nullptr) {
        *error = "Plugin returned null descriptor: " + entry.path().string();
      }
      return false;
    }

    if (descriptor->api_version != MULTI_RADIO_PLUGIN_API_VERSION) {
      dlclose(handle);
      if (error != nullptr) {
        *error = "Plugin API mismatch in " + entry.path().string();
      }
      return false;
    }

    if (descriptor->init != nullptr) {
      const int rc = descriptor->init("{}");
      if (rc != 0) {
        dlclose(handle);
        if (error != nullptr) {
          *error = "Plugin init failed for " + entry.path().string();
        }
        return false;
      }
    }

    LoadedPlugin plugin;
    plugin.info.plugin_name = descriptor->plugin_name == nullptr ? "unknown" : descriptor->plugin_name;
    plugin.info.plugin_version =
        descriptor->plugin_version == nullptr ? "0.0.0" : descriptor->plugin_version;
    plugin.info.api_version = descriptor->api_version;
    plugin.info.supported_signals =
        ParseSignalCsv(descriptor->supported_signals_csv == nullptr ? "" : descriptor->supported_signals_csv);
    plugin.info.enabled = true;
    plugin.info.path = entry.path().string();
    plugin.dl_handle = handle;
    plugin.descriptor = descriptor;
    plugins_.push_back(std::move(plugin));
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
  return true;
}

void PluginHost::ProcessIq(const IQSampleBlock& iq, const MessageCallback& callback) {
  std::lock_guard<std::mutex> lock(mu_);

  EmitContext context{.callback = callback};
  multi_radio_iq_view view{.interleaved_iq = iq.interleaved_iq.data(),
                           .sample_count = iq.interleaved_iq.size() / 2,
                           .sample_rate_hz = iq.sample_rate_hz,
                           .center_frequency_hz = iq.center_frequency_hz};

  for (auto& plugin : plugins_) {
    if (!plugin.info.enabled || plugin.descriptor == nullptr || plugin.descriptor->process_iq == nullptr) {
      continue;
    }
    plugin.descriptor->process_iq(&view, &PluginHost::EmitFromPlugin, &context);
  }
}

void PluginHost::EmitFromPlugin(const char* signal_type, const char* payload, double frequency_hz,
                                uint64_t unix_ms, const char* normalized_kv_json,
                                void* user_data) {
  auto* ctx = static_cast<EmitContext*>(user_data);
  if (ctx == nullptr) {
    return;
  }

  PluginMessage msg;
  msg.signal_type = SignalTypeFromString(signal_type == nullptr ? "UNKNOWN" : signal_type);
  msg.payload = payload == nullptr ? "" : payload;
  msg.frequency_hz = frequency_hz;
  msg.unix_ms = unix_ms;
  msg.normalized_fields =
      ParseFlatJsonMap(normalized_kv_json == nullptr ? "{}" : normalized_kv_json);

  if (ctx->callback) {
    ctx->callback(msg);
  }
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
