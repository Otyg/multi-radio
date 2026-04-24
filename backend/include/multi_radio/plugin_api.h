#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MULTI_RADIO_PLUGIN_API_VERSION 1

typedef void (*multi_radio_emit_message_fn)(const char* signal_type, const char* payload,
                                            double frequency_hz, uint64_t unix_ms,
                                            const char* normalized_kv_json, void* user_data);

typedef struct multi_radio_iq_view {
  const int16_t* interleaved_iq;
  size_t sample_count;
  uint32_t sample_rate_hz;
  uint32_t center_frequency_hz;
} multi_radio_iq_view;

typedef struct multi_radio_plugin_descriptor {
  const char* plugin_name;
  const char* plugin_version;
  uint32_t api_version;
  const char* supported_signals_csv;

  int (*init)(const char* config_json);
  int (*process_iq)(const multi_radio_iq_view* iq_view, multi_radio_emit_message_fn emit_fn,
                    void* user_data);
  int (*flush)(multi_radio_emit_message_fn emit_fn, void* user_data);
  void (*shutdown)();
} multi_radio_plugin_descriptor;

typedef const multi_radio_plugin_descriptor* (*multi_radio_get_plugin_descriptor_fn)();

#ifdef __cplusplus
}
#endif
