#include "multi_radio/plugin_host.hpp"

#include <algorithm>
#include <cstring>
#include <sstream>

// dlopen/dlsym are POSIX; guard for non-Linux hosts if ever needed.
#if defined(__linux__) || defined(__APPLE__)
#include <dlfcn.h>
#define MR_HAVE_DLOPEN 1
#else
#define MR_HAVE_DLOPEN 0
#endif

#include "mr_plugin_api.h"

namespace multi_radio {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

// Passed to EmitFromPlugin so it can resolve back to C++ objects without
// requiring the callback to carry a PluginHost pointer.
struct EmitCallbackState {
  const PluginHost::MessageCallback* callback = nullptr;
  double frequency_hz = 0.0;
};

// Tiny JSON key=value parser: extract value for `key` from a flat object like
// {"baud_rate":"4800","deviation_hz":"2400"}.
// Returns empty string if key is absent.
std::string JsonExtract(const char* json, const char* key) {
  if (json == nullptr || key == nullptr) return {};
  const std::string haystack(json);
  const std::string needle = std::string("\"") + key + "\":\"";
  const auto pos = haystack.find(needle);
  if (pos == std::string::npos) return {};
  const auto val_start = pos + needle.size();
  const auto val_end   = haystack.find('"', val_start);
  if (val_end == std::string::npos) return {};
  return haystack.substr(val_start, val_end - val_start);
}

SignalType SignalTypeFromPluginString(const char* s) {
  if (s == nullptr) return SignalType::kUnknown;
  if (std::strcmp(s, "AIS")       == 0) return SignalType::kAis;
  if (std::strcmp(s, "ADSB")      == 0) return SignalType::kAdsb;
  if (std::strcmp(s, "DSC")       == 0) return SignalType::kDsc;
  // Plugin-specific types (FSK_DATA, GMSK_DATA, …) → kUnknown for now.
  return SignalType::kUnknown;
}

}  // namespace

// ---------------------------------------------------------------------------
// PluginHost
// ---------------------------------------------------------------------------

PluginHost::PluginHost(std::filesystem::path plugin_dir, std::filesystem::path state_dir)
    : plugin_dir_(std::move(plugin_dir)), state_dir_(std::move(state_dir)) {}

PluginHost::~PluginHost() {
  std::lock_guard<std::mutex> lock(mu_);
  for (auto& p : plugins_) {
    if (p.ctx != nullptr && p.fn_destroy != nullptr) {
      p.fn_destroy(p.ctx);
      p.ctx = nullptr;
    }
#if MR_HAVE_DLOPEN
    if (p.dl_handle != nullptr) {
      dlclose(p.dl_handle);
      p.dl_handle = nullptr;
    }
#endif
  }
  plugins_.clear();
}

bool PluginHost::LoadAll(std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);

  // Close any previously loaded plugins first.
  for (auto& p : plugins_) {
    if (p.ctx != nullptr && p.fn_destroy != nullptr) {
      p.fn_destroy(p.ctx);
      p.ctx = nullptr;
    }
#if MR_HAVE_DLOPEN
    if (p.dl_handle != nullptr) {
      dlclose(p.dl_handle);
      p.dl_handle = nullptr;
    }
#endif
  }
  plugins_.clear();

  if (error != nullptr) {
    error->clear();
  }

#if !MR_HAVE_DLOPEN
  if (error != nullptr) {
    *error = "dlopen not available on this platform";
  }
  return false;
#else
  namespace fs = std::filesystem;

  if (plugin_dir_.empty() || !fs::is_directory(plugin_dir_)) {
    // Not an error: simply no plugins to load.
    return true;
  }

  std::ostringstream errors;
  bool any_error = false;

  std::error_code ec;
  for (const auto& de : fs::directory_iterator(plugin_dir_, ec)) {
    if (ec) break;
    if (!de.is_regular_file()) continue;
    const auto& path = de.path();
    if (path.extension() != ".so") continue;

    const std::string path_str = path.string();

    void* handle = dlopen(path_str.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle == nullptr) {
      errors << path.filename().string() << ": dlopen: " << dlerror() << "; ";
      any_error = true;
      continue;
    }

    // Clear any stale error before resolving symbols.
    dlerror();

    auto resolve = [&](const char* sym) -> void* {
      void* ptr = dlsym(handle, sym);
      if (ptr == nullptr) {
        errors << path.filename().string() << ": dlsym(" << sym << "): " << dlerror() << "; ";
        any_error = true;
      }
      return ptr;
    };

    auto* fn_create     = reinterpret_cast<FnCreate>    (resolve("mr_plugin_create"));
    auto* fn_destroy    = reinterpret_cast<FnDestroy>   (resolve("mr_plugin_destroy"));
    auto* fn_get_meta   = reinterpret_cast<FnGetMeta>   (resolve("mr_plugin_get_meta"));
    auto* fn_process_iq = reinterpret_cast<FnProcessIq> (resolve("mr_plugin_process_iq"));

    if (!fn_create || !fn_destroy || !fn_get_meta || !fn_process_iq) {
      dlclose(handle);
      continue;
    }

    const MrPluginMeta* meta = fn_get_meta();
    if (meta == nullptr) {
      errors << path.filename().string() << ": mr_plugin_get_meta returned NULL; ";
      any_error = true;
      dlclose(handle);
      continue;
    }

    if (meta->api_version != MR_PLUGIN_API_VERSION) {
      errors << path.filename().string() << ": API version mismatch (plugin="
             << meta->api_version << " host=" << MR_PLUGIN_API_VERSION << "); ";
      any_error = true;
      dlclose(handle);
      continue;
    }

    MrPluginCtx* ctx = fn_create();
    if (ctx == nullptr) {
      errors << path.filename().string() << ": mr_plugin_create returned NULL; ";
      any_error = true;
      dlclose(handle);
      continue;
    }

    LoadedPlugin lp;
    lp.info.plugin_name    = meta->name    ? meta->name    : path.stem().string();
    lp.info.plugin_version = meta->version ? meta->version : "unknown";
    lp.info.api_version    = meta->api_version;
    lp.info.enabled        = true;
    lp.info.path           = path_str;
    lp.dl_handle           = handle;
    lp.fn_create     = fn_create;
    lp.fn_destroy    = fn_destroy;
    lp.fn_get_meta   = fn_get_meta;
    lp.fn_process_iq = fn_process_iq;
    // mr_plugin_set_param is optional — absence is not an error.
    lp.fn_set_param  = reinterpret_cast<FnSetParam>(dlsym(handle, "mr_plugin_set_param"));
    lp.ctx           = ctx;

    plugins_.push_back(std::move(lp));
  }

  if (error != nullptr && any_error) {
    *error = errors.str();
  }
  return !any_error || !plugins_.empty();
#endif  // MR_HAVE_DLOPEN
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
  if (iq.interleaved_iq.empty()) return;

  std::lock_guard<std::mutex> lock(mu_);

  const uint32_t num_pairs =
      static_cast<uint32_t>(iq.interleaved_iq.size() / 2U);
  const double center_freq_hz = static_cast<double>(iq.center_frequency_hz);

  // Approximate unix_ms from steady clock (good enough for plugin use).
  const uint64_t unix_ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());

  for (auto& plugin : plugins_) {
    if (!plugin.info.enabled) continue;
    if (plugin.fn_process_iq == nullptr || plugin.ctx == nullptr) continue;

    EmitCallbackState state;
    state.callback     = &callback;
    state.frequency_hz = center_freq_hz;

    plugin.fn_process_iq(
        plugin.ctx,
        iq.interleaved_iq.data(),
        num_pairs,
        iq.sample_rate_hz,
        center_freq_hz,
        unix_ms,
        &PluginHost::EmitFromPlugin,
        &state);
  }
}

// static
void PluginHost::EmitFromPlugin(const char* signal_type, const char* payload,
                                double frequency_hz, uint64_t unix_ms,
                                const char* normalized_kv_json, void* user_data) {
  if (user_data == nullptr) return;
  auto* state = static_cast<EmitCallbackState*>(user_data);
  if (state->callback == nullptr) return;

  PluginMessage msg;
  msg.signal_type   = SignalTypeFromPluginString(signal_type);
  msg.frequency_hz  = frequency_hz;
  msg.unix_ms       = unix_ms;
  msg.payload       = payload ? payload : "";

  // Parse normalized_kv_json into the map.  We handle a flat JSON object
  // {"key":"value",...} without pulling in a full JSON library.
  if (normalized_kv_json != nullptr && normalized_kv_json[0] == '{') {
    const char* p = normalized_kv_json + 1;
    while (*p != '\0' && *p != '}') {
      // skip whitespace / commas
      while (*p == ' ' || *p == '\t' || *p == ',') ++p;
      if (*p != '"') break;
      ++p;  // skip opening quote of key
      std::string key;
      while (*p && *p != '"') key += *p++;
      if (*p == '"') ++p;
      if (*p != ':') break;
      ++p;
      if (*p != '"') {
        // Skip non-string values (numbers etc.)
        while (*p && *p != ',' && *p != '}') ++p;
        continue;
      }
      ++p;  // skip opening quote of value
      std::string value;
      while (*p && *p != '"') value += *p++;
      if (*p == '"') ++p;
      if (!key.empty()) {
        msg.normalized_fields[std::move(key)] = std::move(value);
      }
    }
  }

  // Also store the raw signal_type string in fields for consumers that care.
  if (signal_type != nullptr && signal_type[0] != '\0') {
    msg.normalized_fields["signal_type"] = signal_type;
  }

  (*state->callback)(msg);
}

void PluginHost::SetParam(const std::string& key, const std::string& value) {
  std::lock_guard<std::mutex> lock(mu_);
  for (auto& p : plugins_) {
    if (p.fn_set_param && p.ctx) {
      p.fn_set_param(p.ctx, key.c_str(), value.c_str());
    }
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
