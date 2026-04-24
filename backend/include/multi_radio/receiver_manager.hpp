#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "multi_radio/event_bus.hpp"
#include "multi_radio/jsonl_logger.hpp"
#include "multi_radio/plugin_host.hpp"
#include "multi_radio/radio_device.hpp"
#include "multi_radio/receiver_worker.hpp"
#include "multi_radio/types.hpp"

namespace multi_radio {

class ReceiverManager {
 public:
  ReceiverManager(std::unique_ptr<IRadioDeviceFactory> factory, std::shared_ptr<EventBus> event_bus,
                  std::shared_ptr<PluginHost> plugin_host, std::shared_ptr<JsonlLogger> logger);

  std::vector<ReceiverStatus> ListReceivers() const;
  bool GetReceiverStatus(uint32_t receiver_id, ReceiverStatus* status, std::string* error) const;

  bool StartReceiver(uint32_t receiver_id, std::string* error);
  bool StopReceiver(uint32_t receiver_id, std::string* error);

  bool SetMode(uint32_t receiver_id, RadioMode mode, std::string* error);
  bool SetModeConfig(uint32_t receiver_id, const ModeConfig& config, std::string* error);

 private:
  ReceiverWorker* FindWorker(uint32_t receiver_id);
  const ReceiverWorker* FindWorker(uint32_t receiver_id) const;

  std::shared_ptr<EventBus> event_bus_;
  std::shared_ptr<PluginHost> plugin_host_;
  std::shared_ptr<JsonlLogger> logger_;

  mutable std::mutex mu_;
  std::vector<std::unique_ptr<ReceiverWorker>> workers_;
};

}  // namespace multi_radio
