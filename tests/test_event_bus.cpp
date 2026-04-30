#include <cassert>
#include <cstdint>

#include "multi_radio/event_bus.hpp"

int main() {
  using namespace multi_radio;

  EventBus bus(16);

  AudioFrame f1;
  f1.unix_ms = 1;
  f1.receiver_id = 1;
  f1.sample_rate_hz = 6000;
  f1.tuned_frequency_hz = 156525000.0;
  f1.pcm_s16le = {1, 2, 3, 4};
  bus.PublishAudioFrame(f1);

  AudioFrame f2 = f1;
  f2.unix_ms = 2;
  f2.pcm_s16le = {5, 6, 7, 8};
  bus.PublishAudioFrame(f2);

  size_t live_cursor = bus.AudioFrameCursorNow();
  auto historical = bus.WaitForAudioFrame(&live_cursor, 5);
  assert(!historical.has_value());

  AudioFrame f3 = f1;
  f3.unix_ms = 3;
  f3.pcm_s16le = {9, 10};
  bus.PublishAudioFrame(f3);

  auto live = bus.WaitForAudioFrame(&live_cursor, 20);
  assert(live.has_value());
  assert(live->unix_ms == 3);
  assert(live->pcm_s16le.size() == 2);
  assert(live->pcm_s16le[0] == 9);
  assert(live->pcm_s16le[1] == 10);

  IqFrame iq1;
  iq1.unix_ms = 10;
  iq1.receiver_id = 2;
  iq1.sample_rate_hz = 2048000;
  iq1.tuned_frequency_hz = 100100000.0;
  iq1.sequence = 1;
  iq1.sample_index = 0;
  iq1.interleaved_iq_s16le = {100, -100, 200, -200};
  bus.PublishIqFrame(iq1);

  size_t iq_cursor = bus.IqFrameCursorNow();
  auto iq_historical = bus.WaitForIqFrame(&iq_cursor, 5);
  assert(!iq_historical.has_value());

  IqFrame iq2 = iq1;
  iq2.unix_ms = 11;
  iq2.sequence = 2;
  iq2.sample_index = 2;
  iq2.interleaved_iq_s16le = {300, -300};
  bus.PublishIqFrame(iq2);

  auto iq_live = bus.WaitForIqFrame(&iq_cursor, 20);
  assert(iq_live.has_value());
  assert(iq_live->unix_ms == 11);
  assert(iq_live->interleaved_iq_s16le.size() == 2);
  assert(iq_live->interleaved_iq_s16le[0] == 300);
  assert(iq_live->interleaved_iq_s16le[1] == -300);

  return 0;
}
