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
  void SetParam(const std::string& key, const std::string& value);
  // Select which decoder plugin to chain after demodulators (empty = none).
  void SetActiveDecoder(const std::string& plugin_name);
  void SetActiveDemodulator(const std::string& plugin_name);
  // Select which postprocessor plugin to chain after the decoder (empty = none).
  void SetActivePostprocessor(const std::string& plugin_name);
  // Optional postprocessor used for ASM-marked decoder outputs (empty = none).
  void SetActiveAsmPostprocessor(const std::string& plugin_name);

 private:
  // Function-pointer typedefs matching the C API signatures.
  using FnCreate      = MrPluginCtx* (*)(void);
  using FnDestroy     = void (*)(MrPluginCtx*);
  using FnGetMeta     = const MrPluginMeta* (*)(void);
  using FnProcessIq   = void (*)(MrPluginCtx*, const int16_t*, uint32_t, uint32_t,
                                  double, uint64_t, MrEmitFn, void*);
  using FnProcessBits = void (*)(MrPluginCtx*, const uint8_t*, uint32_t,
                                  double, uint64_t, const char*, MrEmitFn, void*);
  using FnSetParam    = int  (*)(MrPluginCtx*, const char*, const char*);

  struct LoadedPlugin {
    PluginInfo   info;
    void*        dl_handle  = nullptr;  // dlopen handle
    FnCreate     fn_create  = nullptr;
    FnDestroy    fn_destroy = nullptr;
    FnGetMeta    fn_get_meta = nullptr;
    FnProcessIq   fn_process_iq  = nullptr;
    FnProcessBits fn_process_bits = nullptr;  // optional, decoder role
    FnSetParam    fn_set_param    = nullptr;  // optional
    MrPluginCtx*  ctx             = nullptr;
    int           role             = 0;
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
  std::string active_decoder_name_;
  std::string active_demodulator_name_;
  std::string active_postprocessor_name_;
  std::string active_asm_postprocessor_name_;
};

}  // namespace multi_radio
