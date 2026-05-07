#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "multi_radio/types.hpp"

// Forward-declare the C API types so the header stays self-contained without
// pulling in the full C header in every translation unit.
struct MrPluginMeta;
typedef void MrPluginCtx;
typedef void (*MrEmitFn)(const char*, const char*, double, uint64_t, const char*, void*);

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

  explicit PluginHost(std::filesystem::path plugin_dir, std::filesystem::path state_dir = {});
  ~PluginHost();

  bool LoadAll(std::string* error);
  std::vector<PluginInfo> ListPlugins() const;

  bool EnablePlugin(const std::string& plugin_name, std::string* error);
  bool DisablePlugin(const std::string& plugin_name, std::string* error);

  void ProcessIq(const IQSampleBlock& iq, const MessageCallback& callback);
  // Call set_param on every loaded plugin that exports mr_plugin_set_param.
  void SetParam(const std::string& key, const std::string& value);

 private:
  // Function-pointer typedefs matching the C API signatures.
  using FnCreate      = MrPluginCtx* (*)(void);
  using FnDestroy     = void (*)(MrPluginCtx*);
  using FnGetMeta     = const MrPluginMeta* (*)(void);
  using FnProcessIq   = void (*)(MrPluginCtx*, const int16_t*, uint32_t, uint32_t,
                                  double, uint64_t, MrEmitFn, void*);
  using FnSetParam    = int  (*)(MrPluginCtx*, const char*, const char*);

  struct LoadedPlugin {
    PluginInfo   info;
    void*        dl_handle  = nullptr;  // dlopen handle
    FnCreate     fn_create  = nullptr;
    FnDestroy    fn_destroy = nullptr;
    FnGetMeta    fn_get_meta = nullptr;
    FnProcessIq  fn_process_iq = nullptr;
    FnSetParam   fn_set_param  = nullptr;  // optional
    MrPluginCtx* ctx        = nullptr;  // live instance
  };

  // Emit callback forwarded from the C plugin back into C++ land.
  // user_data must point to an EmitCallbackState (defined in plugin_host.cpp).
  static void EmitFromPlugin(const char* signal_type, const char* payload, double frequency_hz,
                             uint64_t unix_ms, const char* normalized_kv_json, void* user_data);

  LoadedPlugin* FindPluginByName(const std::string& name);
  const LoadedPlugin* FindPluginByName(const std::string& name) const;

  std::filesystem::path plugin_dir_;
  std::filesystem::path state_dir_;
  mutable std::mutex mu_;
  std::vector<LoadedPlugin> plugins_;
};

}  // namespace multi_radio
