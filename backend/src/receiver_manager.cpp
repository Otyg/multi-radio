#include "multi_radio/receiver_manager.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace multi_radio {

namespace {

std::vector<ReceiverDescriptor> BuildApiOnlyDescriptors(IRadioDeviceFactory* factory) {
  // Preserve receiver list shape. If hardware is available, use real descriptors.
  if (factory != nullptr) {
    auto descriptors = factory->Enumerate();
    if (!descriptors.empty()) {
      return descriptors;
    }
  }

  return std::vector<ReceiverDescriptor>{ReceiverDescriptor{.receiver_id = 0, .serial = "virtual-0"}};
}

}  // namespace

ReceiverManager::ReceiverManager(std::unique_ptr<IRadioDeviceFactory> factory,
                                 std::shared_ptr<EventBus> event_bus,
                                 std::shared_ptr<PluginHost> plugin_host,
                                 std::shared_ptr<JsonlLogger> logger,
                                 std::shared_ptr<TrackDatabase> track_db,
                                 std::shared_ptr<TargetTracker> target_tracker,
                                 std::vector<ReceiverDescriptor> descriptors,
                                 bool external_iq_input)
    : event_bus_(std::move(event_bus)),
      plugin_host_(std::move(plugin_host)),
      logger_(std::move(logger)),
      track_db_(std::move(track_db)),
      target_tracker_(std::move(target_tracker)) {
  if (descriptors.empty()) {
    descriptors = BuildApiOnlyDescriptors(factory.get());
  }
  workers_.reserve(descriptors.size());
  for (const auto& descriptor : descriptors) {
    std::unique_ptr<IRadioDevice> device = nullptr;
    if (!external_iq_input && factory != nullptr) {
      device = factory->Create(descriptor.receiver_id);
    }
    workers_.push_back(std::make_unique<ReceiverWorker>(
        descriptor.receiver_id, descriptor.serial, std::move(device),
        event_bus_, plugin_host_, logger_, track_db_, target_tracker_, external_iq_input));
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

bool ReceiverManager::SubmitIqFrame(uint32_t receiver_id, const IqFrame& frame, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  auto* worker = FindWorker(receiver_id);
  if (worker == nullptr) {
    if (error != nullptr) {
      *error = "receiver not found";
    }
    return false;
  }
  return worker->SubmitIqFrame(frame, error);
}

void ReceiverManager::ApplyHardwarePpm(int ppm) {
  std::lock_guard<std::mutex> lock(mu_);
  for (auto& worker : workers_) {
    ModeConfig cfg = worker->GetModeConfig();
    cfg.ppm_correction = ppm;
    worker->SetModeConfig(cfg, nullptr);
  }
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
