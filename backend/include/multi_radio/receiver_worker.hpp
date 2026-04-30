#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "multi_radio/event_bus.hpp"
#include "multi_radio/jsonl_logger.hpp"
#include "multi_radio/plugin_host.hpp"
#include "multi_radio/radio_device.hpp"
#include "multi_radio/types.hpp"

namespace multi_radio {

class ReceiverWorker {
 public:
  ReceiverWorker(uint32_t receiver_id, std::string serial, std::unique_ptr<IRadioDevice> device,
                 std::shared_ptr<EventBus> event_bus, std::shared_ptr<PluginHost> plugin_host,
                 std::shared_ptr<JsonlLogger> logger);
  ~ReceiverWorker();

  bool Start(std::string* error);
  bool Stop(std::string* error);

  bool SetMode(RadioMode mode, std::string* error);
  bool SetModeConfig(const ModeConfig& config, std::string* error);

  ReceiverStatus Status() const;

 private:
  void RunLoop();
  void PublishEvent(EventKind kind, const std::string& message, double tuned_frequency_hz = 0.0,
                    bool log_event = true);

  const uint32_t receiver_id_;
  const std::string serial_;
  std::unique_ptr<IRadioDevice> device_;
  std::shared_ptr<EventBus> event_bus_;
  std::shared_ptr<PluginHost> plugin_host_;
  std::shared_ptr<JsonlLogger> logger_;

  mutable std::mutex mu_;
  std::thread thread_;
  std::atomic<bool> running_{false};

  RadioMode mode_ = RadioMode::kFixed;
  ModeConfig mode_config_;
  std::string last_error_;
};

}  // namespace multi_radio
