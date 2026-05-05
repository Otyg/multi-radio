#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "multi_radio/audio_ring_buffer.hpp"
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
  void IngestLoop();
  void ProcessLoop();
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
  std::thread ingest_thread_;
  std::thread process_thread_;
  std::atomic<bool> running_{false};

  RadioMode mode_ = RadioMode::kFixed;
  ModeConfig mode_config_;
  std::string last_error_;

  // Ring buffer for audio samples (decouples ingest from output timing)
  std::unique_ptr<AudioRingBuffer> audio_buffer_;

  struct IqSharedState {
    uint32_t sample_rate_hz = 0;
    IQSampleBlock latest_block;
    bool have_latest_block = false;
    uint64_t window_samples = 0;
    uint64_t window_components = 0;
    uint64_t window_clipped_components = 0;
    double window_component_power = 0.0;
    uint64_t interleaved_samples = 0;
    float channel_rssi_db = -120.0f;  // channelized RSSI from agc_crcf, updated per block
  };
  IqSharedState iq_shared_;  // guarded by mu_

  int scan_channel_idx_ = 0;   // guarded by mu_; written by RunLoop, read by IngestLoop
  int audio_channel_idx_ = -1; // guarded by mu_; written by ProcessLoop when audio switches channel

  struct IqQueueEntry {
    IQSampleBlock block;
    uint32_t effective_sample_rate_hz = 0;
    uint32_t audio_sample_rate_hz = 0;
    uint32_t channel_bandwidth_hz = 0;
    double tuned_frequency_hz = 0.0;
    Modulation modulation = Modulation::kWfm;
    int scan_channel_idx = -1;  // channel index active when IngestLoop captured this block
  };
  std::deque<IqQueueEntry> iq_deque_;  // guarded by iq_queue_mu_
  std::mutex iq_queue_mu_;
  std::condition_variable iq_queue_cv_;
};

}  // namespace multi_radio
