#include "multi_radio/plugin_host.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <vector>

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
  // Set when a decoder plugin should be chained (stage 1 → 2).
  void (*decoder_fn)(MrPluginCtx*, const uint8_t*, uint32_t,
                     double, uint64_t, const char*, MrEmitFn, void*) = nullptr;
  MrPluginCtx* decoder_ctx = nullptr;
  // Set when a postprocessor plugin should be chained (stage 2 → 3).
  void (*postproc_fn)(MrPluginCtx*, const uint8_t*, uint32_t,
                      double, uint64_t, const char*, MrEmitFn, void*) = nullptr;
  MrPluginCtx* postproc_ctx = nullptr;
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
  /* All AIS sub-types from ais_decoder.c → kAis */
  if (std::strncmp(s, "AIS_", 4)  == 0) return SignalType::kAis;
  if (std::strcmp(s, "ADSB")      == 0) return SignalType::kAdsb;
  if (std::strncmp(s, "ADSB_", 5) == 0) return SignalType::kAdsb;
  if (std::strcmp(s, "DSC")       == 0) return SignalType::kDsc;
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
    lp.fn_create      = fn_create;
    lp.fn_destroy     = fn_destroy;
    lp.fn_get_meta    = fn_get_meta;
    lp.fn_process_iq  = fn_process_iq;
    lp.fn_process_bits = reinterpret_cast<FnProcessBits>(
        dlsym(handle, "mr_plugin_process_bits"));
    lp.fn_set_param   = reinterpret_cast<FnSetParam>(dlsym(handle, "mr_plugin_set_param"));
    lp.role           = static_cast<int>(meta->role);
    lp.ctx            = ctx;

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

static bool ph_dbg() {
  static int v = -1;
  if (v < 0) { const char* e = getenv("MR_AIS_DEBUG"); v = (e && e[0] != '0') ? 1 : 0; }
  return v != 0;
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

  // Find the active decoder plugin (if any).
  FnProcessBits decoder_fn  = nullptr;
  MrPluginCtx*  decoder_ctx = nullptr;
  if (!active_decoder_name_.empty()) {
    for (auto& p : plugins_) {
      if (p.info.enabled && p.info.plugin_name == active_decoder_name_ &&
          p.fn_process_bits && p.ctx) {
        decoder_fn  = p.fn_process_bits;
        decoder_ctx = p.ctx;
        break;
      }
    }
  }

  // Find the active postprocessor plugin (if any).
  FnProcessBits postproc_fn  = nullptr;
  MrPluginCtx*  postproc_ctx = nullptr;
  if (!active_postprocessor_name_.empty()) {
    for (auto& p : plugins_) {
      if (p.info.enabled && p.info.plugin_name == active_postprocessor_name_ &&
          p.fn_process_bits && p.ctx) {
        postproc_fn  = p.fn_process_bits;
        postproc_ctx = p.ctx;
        break;
      }
    }
  }

  if (ph_dbg()) {
    static uint32_t call_count = 0;
    if (++call_count % 200 == 1) {  /* print on first call and every 200th */
      fprintf(stderr,
              "[plugin_host] ProcessIq #%u  demod='%s'  decoder='%s'(%s)"
              "  postproc='%s'(%s)  plugins=%zu\n",
              call_count,
              active_demodulator_name_.empty() ? "(none)" : active_demodulator_name_.c_str(),
              active_decoder_name_.empty()      ? "(none)" : active_decoder_name_.c_str(),
              decoder_fn  ? "found" : "NOT FOUND",
              active_postprocessor_name_.empty() ? "(none)" : active_postprocessor_name_.c_str(),
              postproc_fn ? "found" : "NOT FOUND",
              plugins_.size());
    }
  }

  for (auto& plugin : plugins_) {
    if (!plugin.info.enabled) continue;
    if (plugin.fn_process_iq == nullptr || plugin.ctx == nullptr) continue;
    // Decoder and postprocessor plugins are invoked via EmitFromPlugin chaining, not directly.
    if (plugin.role == static_cast<int>(MR_PLUGIN_ROLE_DECODER)) continue;
    if (plugin.role == static_cast<int>(MR_PLUGIN_ROLE_POSTPROCESSING)) continue;
    // Skip unless this plugin is the explicitly selected demodulator.
    if (plugin.info.plugin_name != active_demodulator_name_) continue;

    EmitCallbackState state;
    state.callback      = &callback;
    state.frequency_hz  = center_freq_hz;
    state.decoder_fn    = decoder_fn;
    state.decoder_ctx   = decoder_ctx;
    state.postproc_fn   = postproc_fn;
    state.postproc_ctx  = postproc_ctx;

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

  if (ph_dbg()) {
    fprintf(stderr,
            "[plugin_host] emit: sig='%s'  decoder=%s  postproc=%s"
            "  payload_len=%zu\n",
            signal_type ? signal_type : "(null)",
            state->decoder_fn  ? "set" : "null",
            state->postproc_fn ? "set" : "null",
            payload ? std::strlen(payload) : 0u);
  }

  // Stage 1 → 2: chain decoder if set and this is raw bit data from a demodulator.
  if (state->decoder_fn && state->decoder_ctx && signal_type && payload && payload[0]) {
    const std::string sig(signal_type);
    if (sig == "FSK_DATA" || sig == "GMSK_DATA") {
      // Convert hex payload → bytes, then call decoder.
      const size_t hex_len = std::strlen(payload);
      const size_t byte_count = hex_len / 2;
      if (byte_count > 0) {
        std::vector<uint8_t> raw_bytes(byte_count);
        bool hex_ok = true;
        for (size_t i = 0; i < byte_count && hex_ok; ++i) {
          unsigned val = 0;
          if (std::sscanf(payload + i * 2, "%02X", &val) != 1) hex_ok = false;
          else raw_bytes[i] = static_cast<uint8_t>(val);
        }
        if (hex_ok) {
          const uint32_t bit_count = static_cast<uint32_t>(byte_count * 8);
          // Nested state carries postproc but clears decoder to avoid infinite chain.
          EmitCallbackState nested_state;
          nested_state.callback     = state->callback;
          nested_state.frequency_hz = frequency_hz;
          nested_state.decoder_fn   = nullptr;
          nested_state.decoder_ctx  = nullptr;
          nested_state.postproc_fn  = state->postproc_fn;
          nested_state.postproc_ctx = state->postproc_ctx;
          state->decoder_fn(state->decoder_ctx,
                            raw_bytes.data(), bit_count,
                            frequency_hz, unix_ms,
                            signal_type,
                            &PluginHost::EmitFromPlugin, &nested_state);
          return;  // Decoder output replaces the raw demod output.
        }
      }
    }
  }

  // Stage 2 → 3: chain postprocessor if set and no decoder is in play.
  // Applies to any signal type when decoder_fn is null and postproc_fn is set.
  if (!state->decoder_fn && state->postproc_fn && state->postproc_ctx &&
      payload && payload[0]) {
    const size_t hex_len = std::strlen(payload);
    const size_t byte_count = hex_len / 2;
    if (byte_count > 0) {
      std::vector<uint8_t> raw_bytes(byte_count);
      bool hex_ok = true;
      for (size_t i = 0; i < byte_count && hex_ok; ++i) {
        unsigned val = 0;
        if (std::sscanf(payload + i * 2, "%02X", &val) != 1) hex_ok = false;
        else raw_bytes[i] = static_cast<uint8_t>(val);
      }
      if (hex_ok) {
        const uint32_t bit_count = static_cast<uint32_t>(byte_count * 8);
        // Final nested state: no decoder, no postproc — deliver to callback only.
        EmitCallbackState nested2_state;
        nested2_state.callback     = state->callback;
        nested2_state.frequency_hz = frequency_hz;
        nested2_state.decoder_fn   = nullptr;
        nested2_state.decoder_ctx  = nullptr;
        nested2_state.postproc_fn  = nullptr;
        nested2_state.postproc_ctx = nullptr;
        state->postproc_fn(state->postproc_ctx,
                           raw_bytes.data(), bit_count,
                           frequency_hz, unix_ms,
                           signal_type,
                           &PluginHost::EmitFromPlugin, &nested2_state);
        return;  // Postprocessor output replaces the decoder output.
      }
    }
  }

  (*state->callback)(msg);
}

void PluginHost::SetActiveDecoder(const std::string& plugin_name) {
  std::lock_guard<std::mutex> lock(mu_);
  active_decoder_name_ = plugin_name;
}

void PluginHost::SetActiveDemodulator(const std::string& plugin_name) {
  std::lock_guard<std::mutex> lock(mu_);
  active_demodulator_name_ = plugin_name;
}

void PluginHost::SetActivePostprocessor(const std::string& plugin_name) {
  std::lock_guard<std::mutex> lock(mu_);
  active_postprocessor_name_ = plugin_name;
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
