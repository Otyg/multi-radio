#include "multi_radio/plugin_api.h"

extern "C" int not_the_right_symbol() {
  return MULTI_RADIO_PLUGIN_API_VERSION;
}
