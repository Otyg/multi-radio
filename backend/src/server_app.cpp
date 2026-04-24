#include "multi_radio/server_app.hpp"

namespace multi_radio {

ServerApp::ServerApp(ServerConfig config)
    : config_(std::move(config)),
      event_bus_(std::make_shared<EventBus>()),
      logger_(std::make_shared<JsonlLogger>(config_.log_dir, "radio_events", config_.log_max_bytes,
                                            config_.log_max_files)),
      plugin_host_(std::make_shared<PluginHost>(config_.plugin_dir)) {}

ServerApp::~ServerApp() { Shutdown(); }

bool ServerApp::Init(std::string* error) {
  std::string plugin_error;
  if (!plugin_host_->LoadAll(&plugin_error)) {
    if (error != nullptr) {
      *error = plugin_error;
    }
    return false;
  }

  auto factory = CreateDefaultRadioDeviceFactory(config_.enable_rtlsdr);
  receiver_manager_ = std::make_unique<ReceiverManager>(std::move(factory), event_bus_, plugin_host_, logger_);
  impl_ = std::make_unique<Impl>(config_.auth_token);
  return true;
}

bool ServerApp::Run(std::string* error) {
  if (!receiver_manager_ || !impl_) {
    if (error != nullptr) {
      *error = "ServerApp::Init must be called before Run";
    }
    return false;
  }
  return impl_->Run(receiver_manager_.get(), plugin_host_.get(), event_bus_.get(),
                    config_.bind_address, error);
}

void ServerApp::Shutdown() {
  if (impl_) {
    impl_->Shutdown();
  }
}

}  // namespace multi_radio
