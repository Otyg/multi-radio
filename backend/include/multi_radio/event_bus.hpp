#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

#include "multi_radio/types.hpp"

namespace multi_radio {

class EventBus {
 public:
  explicit EventBus(size_t max_messages = 5000);

  void PublishReceiverEvent(const ReceiverEvent& event);
  void PublishDecodedMessage(const DecodedMessage& message);
  void PublishAudioFrame(const AudioFrame& frame);

  std::optional<ReceiverEvent> WaitForReceiverEvent(size_t* cursor, uint32_t timeout_ms);
  std::optional<DecodedMessage> WaitForDecodedMessage(size_t* cursor, uint32_t timeout_ms);
  std::optional<AudioFrame> WaitForAudioFrame(size_t* cursor, uint32_t timeout_ms);

 private:
  const size_t max_messages_;

  std::mutex receiver_events_mu_;
  std::condition_variable receiver_events_cv_;
  std::deque<ReceiverEvent> receiver_events_;
  size_t receiver_base_index_ = 0;

  std::mutex decoded_messages_mu_;
  std::condition_variable decoded_messages_cv_;
  std::deque<DecodedMessage> decoded_messages_;
  size_t decoded_base_index_ = 0;

  std::mutex audio_frames_mu_;
  std::condition_variable audio_frames_cv_;
  std::deque<AudioFrame> audio_frames_;
  size_t audio_base_index_ = 0;
};

}  // namespace multi_radio
