#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include "multi_radio/event_bus.hpp"
#include "multi_radio/jsonl_logger.hpp"
#include "multi_radio/track_database.hpp"
#include "multi_radio/plugin_host.hpp"
#include "multi_radio/target_tracker.hpp"
#include "multi_radio/radio_device.hpp"
#include "multi_radio/receiver_manager.hpp"

namespace multi_radio {

struct ServerConfig {
  std::string bind_address = "0.0.0.0:50051";
  std::string auth_token = "multi-radio-dev-token";
  std::filesystem::path plugin_dir = "./backend/plugins";
  std::filesystem::path log_dir = "./logs";
  size_t log_max_bytes = 5 * 1024 * 1024;
  size_t log_max_files = 5;
};

class ServerApp {
 public:
  explicit ServerApp(ServerConfig config);
  ~ServerApp();

  bool Init(std::string* error);
  bool Run(std::string* error);
  void Shutdown();

 private:
  ServerConfig config_;
  std::shared_ptr<EventBus> event_bus_;
  std::shared_ptr<JsonlLogger> logger_;
  std::shared_ptr<TrackDatabase> track_db_;
  std::shared_ptr<TargetTracker> target_tracker_;
  std::shared_ptr<PluginHost> plugin_host_;
  std::unique_ptr<ReceiverManager> receiver_manager_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace multi_radio
