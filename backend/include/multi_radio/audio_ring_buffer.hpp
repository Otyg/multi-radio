#pragma once

#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>

namespace multi_radio {

// Thread-safe ring buffer for audio samples (int16_t PCM).
// Designed for producer-consumer pattern: ingest thread writes, playback thread reads.
class AudioRingBuffer {
 public:
  // Create a ring buffer with the given capacity in samples.
  explicit AudioRingBuffer(size_t capacity_samples);
  ~AudioRingBuffer() = default;

  AudioRingBuffer(const AudioRingBuffer&) = delete;
  AudioRingBuffer& operator=(const AudioRingBuffer&) = delete;

  // Write samples to the buffer. Returns number of samples actually written.
  // If buffer is full, writes as many as possible without blocking.
  size_t Write(const int16_t* samples, size_t count);

  // Read samples from the buffer. Returns number of samples actually read.
  // If buffer is empty, reads 0 samples.
  size_t Read(int16_t* samples, size_t count);

  // Get number of samples available for reading (without blocking).
  size_t AvailableForRead() const;

  // Get number of samples that can be written (without blocking).
  size_t AvailableForWrite() const;

  // Get total capacity of the buffer.
  size_t Capacity() const { return capacity_; }

  // Clear all samples from the buffer.
  void Clear();

  // Get statistics for monitoring/debugging.
  struct Stats {
    size_t total_written = 0;  // cumulative samples written
    size_t total_read = 0;     // cumulative samples read
    size_t peak_fill_level = 0; // max fill level observed
    size_t underrun_events = 0; // times read requested more than available
  };

  Stats GetStats() const;

 private:
  const size_t capacity_;
  std::vector<int16_t> buffer_;
  size_t write_pos_ = 0;
  size_t read_pos_ = 0;
  size_t fill_count_ = 0;

  // Statistics
  size_t total_written_ = 0;
  size_t total_read_ = 0;
  size_t peak_fill_level_ = 0;
  size_t underrun_events_ = 0;

  mutable std::mutex mu_;
};

}  // namespace multi_radio
