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

  return 0;
}
