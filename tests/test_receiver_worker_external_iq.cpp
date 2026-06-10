#include <cassert>
#include <memory>

#include "multi_radio/event_bus.hpp"
#include "multi_radio/jsonl_logger.hpp"
#include "multi_radio/plugin_host.hpp"
#include "multi_radio/receiver_worker.hpp"

int main() {
  using namespace multi_radio;

  auto bus = std::make_shared<EventBus>(256);
  auto logger = std::make_shared<JsonlLogger>("/tmp/multi-radio-tests", "external_iq", 1024 * 1024, 2);
  auto plugins = std::make_shared<PluginHost>("/tmp/multi-radio-test-plugins");

  ReceiverWorker worker(7, "REMOTE7", nullptr, bus, plugins, logger, nullptr, nullptr, true);

  std::string error;
  assert(worker.Start(&error));
  assert(worker.SetMode(RadioMode::kFixed, &error));

  ModeConfig config;
  config.fixed_frequency_hz = 162000000.0;
  config.fixed_modulation = Modulation::kAisDual;
  config.sample_rate_hz = 2048000;
  config.gmsk_decoder = "nrzi_decoder";
  config.gmsk_postprocessor = "ais_decoder";
  config.gmsk_nrzi_invert = true;
  assert(worker.SetModeConfig(config, &error));

  IqFrame frame;
  frame.receiver_id = 7;
  frame.sample_rate_hz = 2048000;
  frame.tuned_frequency_hz = 162000000.0;
  frame.interleaved_iq_s16le.assign(4096, 0);
  assert(worker.SubmitIqFrame(frame, &error));

  size_t cursor = 0;
  bool saw_iq = false;
  for (int attempt = 0; attempt < 10; ++attempt) {
    auto iq = bus->WaitForIqFrame(&cursor, 250);
    if (iq.has_value() && iq->receiver_id == 7) {
      assert(iq->sample_rate_hz == frame.sample_rate_hz);
      assert(iq->tuned_frequency_hz == frame.tuned_frequency_hz);
      assert(iq->interleaved_iq_s16le.size() == frame.interleaved_iq_s16le.size());
      saw_iq = true;
      break;
    }
  }
  assert(saw_iq);

  assert(worker.Stop(&error));
  return 0;
}
