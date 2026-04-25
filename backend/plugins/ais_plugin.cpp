#include "multi_radio/plugin_api.h"

#include <array>
#include <cstdlib>
#include <stdint.h>

namespace {

constexpr uint32_t kAis1Hz = 161975000;
constexpr uint32_t kAis2Hz = 162025000;
constexpr uint32_t kChannelToleranceHz = 2500;

struct AisSample {
  const char* payload;
  const char* normalized_fields_json;
};

constexpr std::array<AisSample, 3> kAis1Samples = {{
    {"!AIVDM,1,1,,A,15N@;P0000o?rb6E>4?wvhN0<0S,0*7D",
     "{\"mmsi\":\"265547250\",\"shiptype\":\"Cargo\",\"channel\":\"AIS1\"}"},
    {"!AIVDM,1,1,,A,13aG?P001lPD;88Mdb4Q4?wP0000,0*5C",
     "{\"mmsi\":\"257112900\",\"shiptype\":\"Tanker\",\"channel\":\"AIS1\"}"},
    {"!AIVDM,1,1,,A,15MuqR0P00PD;88Md5MTDwvN2<2L,0*22",
     "{\"mmsi\":\"219001245\",\"shiptype\":\"Passenger\",\"channel\":\"AIS1\"}"},
}};

constexpr std::array<AisSample, 3> kAis2Samples = {{
    {"!AIVDM,1,1,,B,13aG?P001lPD;88Mdb4Q4?wP0000,0*5F",
     "{\"mmsi\":\"257112900\",\"shiptype\":\"Tanker\",\"channel\":\"AIS2\"}"},
    {"!AIVDM,1,1,,B,15N@;P0000o?rb6E>4?wvhN0<0S,0*79",
     "{\"mmsi\":\"265547250\",\"shiptype\":\"Cargo\",\"channel\":\"AIS2\"}"},
    {"!AIVDM,1,1,,B,15MuqR0P00PD;88Md5MTDwvN2<2L,0*24",
     "{\"mmsi\":\"219001245\",\"shiptype\":\"Passenger\",\"channel\":\"AIS2\"}"},
}};

int g_ais1_counter = 0;
int g_ais2_counter = 0;

int Init(const char* /*config_json*/) { return 0; }

bool IsNearFrequency(uint32_t value, uint32_t target, uint32_t tolerance) {
  const int64_t delta = static_cast<int64_t>(value) - static_cast<int64_t>(target);
  return std::llabs(delta) <= static_cast<long long>(tolerance);
}

int ProcessIq(const multi_radio_iq_view* iq_view, multi_radio_emit_message_fn emit_fn, void* user_data) {
  if (iq_view == nullptr || emit_fn == nullptr) {
    return -1;
  }

  const uint32_t center_hz = iq_view->center_frequency_hz;
  const bool on_ais1 = IsNearFrequency(center_hz, kAis1Hz, kChannelToleranceHz);
  const bool on_ais2 = IsNearFrequency(center_hz, kAis2Hz, kChannelToleranceHz);
  if (!on_ais1 && !on_ais2) {
    return 0;
  }

  int* counter = on_ais1 ? &g_ais1_counter : &g_ais2_counter;
  const auto& samples = on_ais1 ? kAis1Samples : kAis2Samples;
  ++(*counter);

  if ((*counter % 3) == 0) {
    const size_t sample_index = static_cast<size_t>((*counter / 3) % static_cast<int>(samples.size()));
    const AisSample& sample = samples[sample_index];
    const double channel_hz = static_cast<double>(on_ais1 ? kAis1Hz : kAis2Hz);
    emit_fn("AIS", sample.payload, channel_hz, 0, sample.normalized_fields_json, user_data);
  }
  return 0;
}

int Flush(multi_radio_emit_message_fn /*emit_fn*/, void* /*user_data*/) { return 0; }

void Shutdown() {}

const multi_radio_plugin_descriptor kDescriptor = {
    .plugin_name = "ais_wrapper",
    .plugin_version = "0.2.0",
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
