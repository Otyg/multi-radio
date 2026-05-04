#include "multi_radio/audio_ring_buffer.hpp"

#include <algorithm>

namespace multi_radio {

AudioRingBuffer::AudioRingBuffer(size_t capacity_samples)
    : capacity_(capacity_samples), buffer_(capacity_samples, 0) {}

size_t AudioRingBuffer::Write(const int16_t* samples, size_t count) {
  if (samples == nullptr || count == 0) {
    return 0;
  }

  std::lock_guard<std::mutex> lock(mu_);

  size_t written = 0;
  for (size_t i = 0; i < count; ++i) {
    if (fill_count_ >= capacity_) {
      // Buffer is full, stop writing
      break;
    }
    buffer_[write_pos_] = samples[i];
    write_pos_ = (write_pos_ + 1) % capacity_;
    ++fill_count_;
    ++written;
    ++total_written_;
  }

  peak_fill_level_ = std::max(peak_fill_level_, fill_count_);

  return written;
}

size_t AudioRingBuffer::Read(int16_t* samples, size_t count) {
  if (samples == nullptr || count == 0) {
    return 0;
  }

  std::lock_guard<std::mutex> lock(mu_);

  if (fill_count_ < count) {
    ++underrun_events_;
  }

  size_t to_read = std::min(count, fill_count_);
  for (size_t i = 0; i < to_read; ++i) {
    samples[i] = buffer_[read_pos_];
    read_pos_ = (read_pos_ + 1) % capacity_;
    ++total_read_;
  }
  fill_count_ -= to_read;

  return to_read;
}

size_t AudioRingBuffer::AvailableForRead() const {
  std::lock_guard<std::mutex> lock(mu_);
  return fill_count_;
}

size_t AudioRingBuffer::AvailableForWrite() const {
  std::lock_guard<std::mutex> lock(mu_);
  return capacity_ - fill_count_;
}

void AudioRingBuffer::Clear() {
  std::lock_guard<std::mutex> lock(mu_);
  write_pos_ = 0;
  read_pos_ = 0;
  fill_count_ = 0;
}

AudioRingBuffer::Stats AudioRingBuffer::GetStats() const {
  std::lock_guard<std::mutex> lock(mu_);
  return {total_written_, total_read_, peak_fill_level_, underrun_events_};
}

}  // namespace multi_radio
