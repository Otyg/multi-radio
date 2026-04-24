#include "multi_radio/receiver_manager.hpp"

#include <algorithm>

namespace multi_radio {

ReceiverManager::ReceiverManager(std::unique_ptr<IRadioDeviceFactory> factory,
                                 std::shared_ptr<EventBus> event_bus,
                                 std::shared_ptr<PluginHost> plugin_host,
                                 std::shared_ptr<JsonlLogger> logger)
    : event_bus_(std::move(event_bus)),
      plugin_host_(std::move(plugin_host)),
      logger_(std::move(logger)) {
  const auto devices = factory->Enumerate();
  workers_.reserve(devices.size());
  for (const auto& descriptor : devices) {
    auto device = factory->Create(descriptor.receiver_id);
    workers_.push_back(std::make_unique<ReceiverWorker>(
        descriptor.receiver_id, descriptor.serial, std::move(device), event_bus_, plugin_host_, logger_));
  }
}

std::vector<ReceiverStatus> ReceiverManager::ListReceivers() const {
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<ReceiverStatus> out;
  out.reserve(workers_.size());
  for (const auto& worker : workers_) {
    out.push_back(worker->Status());
  }
  return out;
}

bool ReceiverManager::GetReceiverStatus(uint32_t receiver_id, ReceiverStatus* status,
                                        std::string* error) const {
  std::lock_guard<std::mutex> lock(mu_);
  const auto* worker = FindWorker(receiver_id);
  if (worker == nullptr) {
    if (error != nullptr) {
      *error = "receiver not found";
    }
    return false;
  }
  if (status != nullptr) {
    *status = worker->Status();
  }
  return true;
}

bool ReceiverManager::StartReceiver(uint32_t receiver_id, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  auto* worker = FindWorker(receiver_id);
  if (worker == nullptr) {
    if (error != nullptr) {
      *error = "receiver not found";
    }
    return false;
  }
  return worker->Start(error);
}

bool ReceiverManager::StopReceiver(uint32_t receiver_id, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  auto* worker = FindWorker(receiver_id);
  if (worker == nullptr) {
    if (error != nullptr) {
      *error = "receiver not found";
    }
    return false;
  }
  return worker->Stop(error);
}

bool ReceiverManager::SetMode(uint32_t receiver_id, RadioMode mode, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  auto* worker = FindWorker(receiver_id);
  if (worker == nullptr) {
    if (error != nullptr) {
      *error = "receiver not found";
    }
    return false;
  }
  return worker->SetMode(mode, error);
}

bool ReceiverManager::SetModeConfig(uint32_t receiver_id, const ModeConfig& config, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  auto* worker = FindWorker(receiver_id);
  if (worker == nullptr) {
    if (error != nullptr) {
      *error = "receiver not found";
    }
    return false;
  }
  return worker->SetModeConfig(config, error);
}

ReceiverWorker* ReceiverManager::FindWorker(uint32_t receiver_id) {
  auto it = std::find_if(workers_.begin(), workers_.end(), [&](const auto& worker) {
    return worker->Status().receiver_id == receiver_id;
  });
  return it == workers_.end() ? nullptr : it->get();
}

const ReceiverWorker* ReceiverManager::FindWorker(uint32_t receiver_id) const {
  auto it = std::find_if(workers_.begin(), workers_.end(), [&](const auto& worker) {
    return worker->Status().receiver_id == receiver_id;
  });
  return it == workers_.end() ? nullptr : it->get();
}

}  // namespace multi_radio
