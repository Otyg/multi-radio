#include "multi_radio/event_bus.hpp"

#include <chrono>

namespace multi_radio {

EventBus::EventBus(size_t max_messages) : max_messages_(max_messages) {}

void EventBus::PublishReceiverEvent(const ReceiverEvent& event) {
  {
    std::lock_guard<std::mutex> lock(receiver_events_mu_);
    receiver_events_.push_back(event);
    while (receiver_events_.size() > max_messages_) {
      receiver_events_.pop_front();
      ++receiver_base_index_;
    }
  }
  receiver_events_cv_.notify_all();
}

void EventBus::PublishDecodedMessage(const DecodedMessage& message) {
  {
    std::lock_guard<std::mutex> lock(decoded_messages_mu_);
    decoded_messages_.push_back(message);
    while (decoded_messages_.size() > max_messages_) {
      decoded_messages_.pop_front();
      ++decoded_base_index_;
    }
  }
  decoded_messages_cv_.notify_all();
}

std::optional<ReceiverEvent> EventBus::WaitForReceiverEvent(size_t* cursor, uint32_t timeout_ms) {
  std::unique_lock<std::mutex> lock(receiver_events_mu_);
  auto has_data = [&]() {
    if (*cursor < receiver_base_index_) {
      *cursor = receiver_base_index_;
    }
    return *cursor < receiver_base_index_ + receiver_events_.size();
  };

  if (!receiver_events_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), has_data)) {
    return std::nullopt;
  }

  const size_t offset = *cursor - receiver_base_index_;
  const ReceiverEvent value = receiver_events_.at(offset);
  ++(*cursor);
  return value;
}

std::optional<DecodedMessage> EventBus::WaitForDecodedMessage(size_t* cursor, uint32_t timeout_ms) {
  std::unique_lock<std::mutex> lock(decoded_messages_mu_);
  auto has_data = [&]() {
    if (*cursor < decoded_base_index_) {
      *cursor = decoded_base_index_;
    }
    return *cursor < decoded_base_index_ + decoded_messages_.size();
  };

  if (!decoded_messages_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), has_data)) {
    return std::nullopt;
  }

  const size_t offset = *cursor - decoded_base_index_;
  const DecodedMessage value = decoded_messages_.at(offset);
  ++(*cursor);
  return value;
}

}  // namespace multi_radio
