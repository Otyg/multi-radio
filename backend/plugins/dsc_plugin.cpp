#include "multi_radio/plugin_api.h"

#include <stdint.h>

namespace {

int Init(const char* /*config_json*/) { return 0; }

int ProcessIq(const multi_radio_iq_view* iq_view, multi_radio_emit_message_fn emit_fn, void* user_data) {
  if (iq_view == nullptr || emit_fn == nullptr) {
    return -1;
  }
  (void)user_data;
  // Disable synthetic DSC traffic while AIS decoder tuning is in progress.
  return 0;
}

int Flush(multi_radio_emit_message_fn /*emit_fn*/, void* /*user_data*/) { return 0; }

void Shutdown() {}

const multi_radio_plugin_descriptor kDescriptor = {
    .plugin_name = "dsc_wrapper",
    .plugin_version = "0.2.0",
    .api_version = MULTI_RADIO_PLUGIN_API_VERSION,
    .supported_signals_csv = "DSC",
    .init = &Init,
    .process_iq = &ProcessIq,
    .flush = &Flush,
    .shutdown = &Shutdown,
};

}  // namespace

extern "C" const multi_radio_plugin_descriptor* multi_radio_get_plugin_descriptor() {
  return &kDescriptor;
}
