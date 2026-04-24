#include "multi_radio/receiver_worker.hpp"

#include <chrono>
#include <sstream>

namespace multi_radio {

ReceiverWorker::ReceiverWorker(uint32_t receiver_id, std::string serial,
                               std::unique_ptr<IRadioDevice> device,
                               std::shared_ptr<EventBus> event_bus,
                               std::shared_ptr<PluginHost> plugin_host,
                               std::shared_ptr<JsonlLogger> logger)
    : receiver_id_(receiver_id),
      serial_(std::move(serial)),
      device_(std::move(device)),
      event_bus_(std::move(event_bus)),
      plugin_host_(std::move(plugin_host)),
      logger_(std::move(logger)) {
  mode_config_.fixed_frequency_hz = 162025000.0;
  mode_config_.dwell_ms = 500;
  scheduler_.Configure(mode_, mode_config_);
}

ReceiverWorker::~ReceiverWorker() {
  std::string error;
  Stop(&error);
}

bool ReceiverWorker::Start(std::string* error) {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    if (error != nullptr) {
      *error = "receiver already running";
    }
    return false;
  }

  if (!device_->Open(error)) {
    running_.store(false);
    return false;
  }

  if (!device_->SetSampleRateHz(2048000, error)) {
    device_->Close();
    running_.store(false);
    return false;
  }

  thread_ = std::thread([this]() { RunLoop(); });
  PublishEvent(EventKind::kStateChange, "receiver started");
  return true;
}

bool ReceiverWorker::Stop(std::string* error) {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) {
    return true;
  }

  if (thread_.joinable()) {
    thread_.join();
  }
  device_->Close();
  PublishEvent(EventKind::kStateChange, "receiver stopped");
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool ReceiverWorker::SetMode(RadioMode mode, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  mode_ = mode;
  scheduler_.Configure(mode_, mode_config_);
  if (error != nullptr) {
    error->clear();
  }

  std::ostringstream msg;
  msg << "mode changed to " << ToString(mode_);
  PublishEvent(EventKind::kStateChange, msg.str());
  return true;
}

bool ReceiverWorker::SetModeConfig(const ModeConfig& config, std::string* error) {
  std::lock_guard<std::mutex> lock(mu_);
  mode_config_ = config;
  scheduler_.Configure(mode_, mode_config_);
  if (error != nullptr) {
    error->clear();
  }
  PublishEvent(EventKind::kInfo, "mode config updated");
  return true;
}

ReceiverStatus ReceiverWorker::Status() const {
  std::lock_guard<std::mutex> lock(mu_);
  return ReceiverStatus{.receiver_id = receiver_id_,
                        .serial = serial_,
                        .running = running_.load(),
                        .mode = mode_,
                        .mode_config = mode_config_,
                        .last_error = last_error_};
}

void ReceiverWorker::RunLoop() {
  while (running_.load()) {
    std::optional<double> frequency_hz;
    uint32_t dwell_ms = 500;
    {
      std::lock_guard<std::mutex> lock(mu_);
      frequency_hz = scheduler_.NextFrequencyHz();
      dwell_ms = scheduler_.DwellMs();
    }

    if (!frequency_hz.has_value()) {
      PublishEvent(EventKind::kWarning, "no active frequencies in current mode");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    std::string error;
    const auto tune_hz = static_cast<uint32_t>(frequency_hz.value());
    if (!device_->SetCenterFrequencyHz(tune_hz, &error)) {
      {
        std::lock_guard<std::mutex> lock(mu_);
        last_error_ = error;
      }
      PublishEvent(EventKind::kError, "tune failed: " + error, frequency_hz.value());
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    PublishEvent(EventKind::kTuneHop, "tuned", frequency_hz.value());

    IQSampleBlock iq;
    if (!device_->ReadIq(&iq, &error)) {
      {
        std::lock_guard<std::mutex> lock(mu_);
        last_error_ = error;
      }
      PublishEvent(EventKind::kError, "read failed: " + error, frequency_hz.value());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    plugin_host_->ProcessIq(iq, [&](const PluginMessage& plugin_msg) {
      DecodedMessage msg;
      msg.unix_ms = plugin_msg.unix_ms == 0 ? UnixMillisNow() : plugin_msg.unix_ms;
      msg.receiver_id = receiver_id_;
      msg.signal_type = plugin_msg.signal_type;
      msg.frequency_hz = plugin_msg.frequency_hz == 0.0 ? frequency_hz.value() : plugin_msg.frequency_hz;
      msg.payload = plugin_msg.payload;
      msg.normalized_fields = plugin_msg.normalized_fields;
      event_bus_->PublishDecodedMessage(msg);
      logger_->LogDecodedMessage(msg);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(dwell_ms));
  }
}

void ReceiverWorker::PublishEvent(EventKind kind, const std::string& message, double tuned_frequency_hz) {
  ReceiverEvent event;
  event.unix_ms = UnixMillisNow();
  event.receiver_id = receiver_id_;
  event.kind = kind;
  event.message = message;
  event.tuned_frequency_hz = tuned_frequency_hz;
  event_bus_->PublishReceiverEvent(event);
  logger_->LogEvent(event);
}

}  // namespace multi_radio
