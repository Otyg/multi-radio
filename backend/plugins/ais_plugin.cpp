#include "multi_radio/plugin_api.h"

#include <stdint.h>

namespace {

int g_counter = 0;

int Init(const char* /*config_json*/) { return 0; }

int ProcessIq(const multi_radio_iq_view* iq_view, multi_radio_emit_message_fn emit_fn, void* user_data) {
  if (iq_view == nullptr || emit_fn == nullptr) {
    return -1;
  }
  ++g_counter;
  if (iq_view->center_frequency_hz < 161900000 || iq_view->center_frequency_hz > 162100000) {
    return 0;
  }
  if (g_counter % 6 == 0) {
    emit_fn("AIS", "!AIVDM,1,1,,A,15N@;P0000o?rb6E>4?wvhN0<0S,0*7D",
            static_cast<double>(iq_view->center_frequency_hz), 0,
            "{\"mmsi\":\"265547250\",\"shiptype\":\"Cargo\"}", user_data);
  }
  return 0;
}

int Flush(multi_radio_emit_message_fn /*emit_fn*/, void* /*user_data*/) { return 0; }

void Shutdown() {}

const multi_radio_plugin_descriptor kDescriptor = {
    .plugin_name = "ais_wrapper",
    .plugin_version = "0.1.0",
    .api_version = MULTI_RADIO_PLUGIN_API_VERSION,
    .supported_signals_csv = "AIS",
    .init = &Init,
    .process_iq = &ProcessIq,
    .flush = &Flush,
    .shutdown = &Shutdown,
};

}  // namespace

extern "C" const multi_radio_plugin_descriptor* multi_radio_get_plugin_descriptor() {
  return &kDescriptor;
}
