#include <cassert>
#include <memory>

#include "multi_radio/event_bus.hpp"
#include "multi_radio/jsonl_logger.hpp"
#include "multi_radio/plugin_host.hpp"
#include "multi_radio/radio_device.hpp"
#include "multi_radio/receiver_manager.hpp"

int main() {
  using namespace multi_radio;

  auto bus = std::make_shared<EventBus>(1024);
  auto logger = std::make_shared<JsonlLogger>("/tmp/multi-radio-tests", "integration", 1024 * 1024, 2);
  auto plugins = std::make_shared<PluginHost>("/tmp/multi-radio-test-plugins");
  auto factory = CreateMockRadioDeviceFactory();

  ReceiverManager manager(std::move(factory), bus, plugins, logger);

  auto receivers = manager.ListReceivers();
  assert(receivers.size() >= 2);

  std::string error;
  assert(manager.SetMode(0, RadioMode::kFixed, &error));
  assert(manager.SetMode(1, RadioMode::kAirMarinePlot, &error));
  assert(manager.StartReceiver(0, &error));
  assert(manager.StartReceiver(1, &error));

  size_t cursor = 0;
  bool saw_rx0 = false;
  bool saw_rx1 = false;
  for (int i = 0; i < 20; ++i) {
    auto event = bus->WaitForReceiverEvent(&cursor, 500);
    if (!event.has_value()) {
      continue;
    }
    if (event->receiver_id == 0) {
      saw_rx0 = true;
    }
    if (event->receiver_id == 1) {
      saw_rx1 = true;
    }
    if (saw_rx0 && saw_rx1) {
      break;
    }
  }

  assert(saw_rx0 && saw_rx1);

  assert(manager.StopReceiver(0, &error));
  assert(manager.StopReceiver(1, &error));
  return 0;
}
