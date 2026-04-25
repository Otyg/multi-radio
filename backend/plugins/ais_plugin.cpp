#include "multi_radio/plugin_api.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kAis1Hz = 161975000;
constexpr uint32_t kAis2Hz = 162025000;
constexpr uint32_t kChannelToleranceHz = 2500;
constexpr double kSymbolRate = 9600.0;
constexpr int kSamplesPerSymbol = 5;
constexpr double kResampledRate = kSymbolRate * static_cast<double>(kSamplesPerSymbol);
constexpr size_t kMaxRecentFrames = 1024;
constexpr uint64_t kRecentFrameWindow = 4096;
constexpr uint64_t kMetricReportEveryBlocks = 1;

struct RecentFrame {
  uint64_t hash = 0;
  uint64_t seq = 0;
};

uint64_t g_sequence = 0;
std::deque<RecentFrame> g_recent_frames;
std::atomic<uint64_t> g_metric_blocks{0};
std::atomic<uint64_t> g_metric_flags{0};
std::atomic<uint64_t> g_metric_candidates{0};
std::atomic<uint64_t> g_metric_crc_ok{0};
std::atomic<uint64_t> g_metric_crc_fail{0};
std::atomic<uint64_t> g_metric_duplicates{0};
std::atomic<uint64_t> g_metric_emitted{0};

int Init(const char* /*config_json*/) {
  g_recent_frames.clear();
  g_sequence = 0;
  g_metric_blocks.store(0, std::memory_order_relaxed);
  g_metric_flags.store(0, std::memory_order_relaxed);
  g_metric_candidates.store(0, std::memory_order_relaxed);
  g_metric_crc_ok.store(0, std::memory_order_relaxed);
  g_metric_crc_fail.store(0, std::memory_order_relaxed);
  g_metric_duplicates.store(0, std::memory_order_relaxed);
  g_metric_emitted.store(0, std::memory_order_relaxed);
  return 0;
}

bool IsNearFrequency(uint32_t value, uint32_t target, uint32_t tolerance) {
  const int64_t delta = static_cast<int64_t>(value) - static_cast<int64_t>(target);
  return std::llabs(delta) <= static_cast<long long>(tolerance);
}

std::vector<double> BuildFmDiscriminator(const multi_radio_iq_view* iq_view) {
  std::vector<double> out;
  if (iq_view == nullptr || iq_view->interleaved_iq == nullptr || iq_view->sample_count < 2) {
    return out;
  }

  out.reserve(iq_view->sample_count - 1);

  double prev_i = static_cast<double>(iq_view->interleaved_iq[0]);
  double prev_q = static_cast<double>(iq_view->interleaved_iq[1]);
  for (size_t n = 1; n < iq_view->sample_count; ++n) {
    const double cur_i = static_cast<double>(iq_view->interleaved_iq[n * 2]);
    const double cur_q = static_cast<double>(iq_view->interleaved_iq[n * 2 + 1]);

    const double cross = prev_i * cur_q - prev_q * cur_i;
    const double dot = prev_i * cur_i + prev_q * cur_q;
    out.push_back(std::atan2(cross, dot));

    prev_i = cur_i;
    prev_q = cur_q;
  }

  if (out.empty()) {
    return out;
  }

  double mean = 0.0;
  for (double value : out) {
    mean += value;
  }
  mean /= static_cast<double>(out.size());
  for (double& value : out) {
    value -= mean;
  }
  return out;
}

std::vector<double> ResampleLinear(const std::vector<double>& in, double source_rate_hz,
                                   double target_rate_hz) {
  std::vector<double> out;
  if (in.size() < 2 || source_rate_hz <= 0.0 || target_rate_hz <= 0.0) {
    return out;
  }

  const double step = source_rate_hz / target_rate_hz;
  if (step <= 0.0) {
    return out;
  }

  const size_t estimated = static_cast<size_t>(std::max(0.0, static_cast<double>(in.size()) / step));
  out.reserve(estimated);

  double pos = 0.0;
  while (pos + 1.0 < static_cast<double>(in.size())) {
    const size_t left = static_cast<size_t>(pos);
    const size_t right = std::min(left + 1, in.size() - 1);
    const double frac = pos - static_cast<double>(left);
    const double sample = in[left] + (in[right] - in[left]) * frac;
    out.push_back(sample);
    pos += step;
  }
  return out;
}

int EstimateBestPhase(const std::vector<double>& samples) {
  int best_phase = 0;
  int best_score = -1;

  for (int phase = 0; phase < kSamplesPerSymbol; ++phase) {
    int score = 0;
    int prev_sign = 0;
    bool have_prev = false;
    int considered = 0;
    for (size_t i = static_cast<size_t>(phase); i < samples.size() && considered < 320;
         i += static_cast<size_t>(kSamplesPerSymbol), ++considered) {
      const int sign = samples[i] >= 0.0 ? 1 : 0;
      if (have_prev && sign != prev_sign) {
        ++score;
      }
      prev_sign = sign;
      have_prev = true;
    }
    if (score > best_score) {
      best_score = score;
      best_phase = phase;
    }
  }
  return best_phase;
}

std::vector<int> BuildSymbolStream(const std::vector<double>& samples, int phase, bool invert) {
  std::vector<int> symbols;
  if (samples.empty()) {
    return symbols;
  }

  symbols.reserve(samples.size() / static_cast<size_t>(kSamplesPerSymbol) + 1);
  for (size_t i = static_cast<size_t>(std::max(0, phase)); i < samples.size();
       i += static_cast<size_t>(kSamplesPerSymbol)) {
    int bit = samples[i] >= 0.0 ? 1 : 0;
    if (invert) {
      bit ^= 1;
    }
    symbols.push_back(bit);
  }
  return symbols;
}

std::vector<int> NrziDecode(const std::vector<int>& symbols, int initial_level) {
  std::vector<int> bits;
  bits.reserve(symbols.size());

  int prev = initial_level;
  for (int level : symbols) {
    const int bit = (level == prev) ? 1 : 0;
    bits.push_back(bit);
    prev = level;
  }
  return bits;
}

bool IsFlagAt(const std::vector<int>& bits, size_t offset) {
  if (offset + 8 > bits.size()) {
    return false;
  }
  static constexpr int kFlag[8] = {0, 1, 1, 1, 1, 1, 1, 0};
  for (size_t i = 0; i < 8; ++i) {
    if (bits[offset + i] != kFlag[i]) {
      return false;
    }
  }
  return true;
}

bool UnstuffBits(const std::vector<int>& in, std::vector<int>* out) {
  if (out == nullptr) {
    return false;
  }
  out->clear();
  out->reserve(in.size());

  int one_count = 0;
  for (size_t i = 0; i < in.size(); ++i) {
    const int bit = in[i];
    out->push_back(bit);
    if (bit == 1) {
      ++one_count;
      if (one_count == 5) {
        if (i + 1 >= in.size()) {
          return false;
        }
        if (in[i + 1] == 0) {
          ++i;
        }
        one_count = 0;
      }
    } else {
      one_count = 0;
    }
  }
  return true;
}

std::vector<uint8_t> PackBits(const std::vector<int>& bits, bool lsb_first) {
  std::vector<uint8_t> bytes;
  if (bits.empty()) {
    return bytes;
  }
  bytes.assign(bits.size() / 8, 0);
  for (size_t i = 0; i + 7 < bits.size(); i += 8) {
    uint8_t value = 0;
    for (size_t b = 0; b < 8; ++b) {
      const size_t bit_index = i + b;
      const int bit = bits[bit_index] & 0x1;
      if (lsb_first) {
        value |= static_cast<uint8_t>(bit << b);
      } else {
        value = static_cast<uint8_t>((value << 1) | bit);
      }
    }
    bytes[i / 8] = value;
  }
  return bytes;
}

uint16_t Crc16X25(const std::vector<uint8_t>& data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001U) != 0) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0x8408U);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return static_cast<uint16_t>(~crc);
}

bool ValidateFcsX25(const std::vector<uint8_t>& frame_bytes) {
  if (frame_bytes.size() < 4) {
    return false;
  }

  const size_t payload_len = frame_bytes.size() - 2;
  const uint16_t fcs_le = static_cast<uint16_t>(frame_bytes[payload_len]) |
                          static_cast<uint16_t>(frame_bytes[payload_len + 1] << 8);
  const uint16_t fcs_be = static_cast<uint16_t>(frame_bytes[payload_len] << 8) |
                          static_cast<uint16_t>(frame_bytes[payload_len + 1]);
  const uint16_t computed = Crc16X25(frame_bytes, payload_len);
  return computed == fcs_le || computed == fcs_be;
}

uint8_t ReverseBits8(uint8_t value) {
  value = static_cast<uint8_t>(((value & 0xF0U) >> 4) | ((value & 0x0FU) << 4));
  value = static_cast<uint8_t>(((value & 0xCCU) >> 2) | ((value & 0x33U) << 2));
  value = static_cast<uint8_t>(((value & 0xAAU) >> 1) | ((value & 0x55U) << 1));
  return value;
}

std::vector<uint8_t> ReverseBitsPerByte(const std::vector<uint8_t>& in) {
  std::vector<uint8_t> out;
  out.reserve(in.size());
  for (uint8_t value : in) {
    out.push_back(ReverseBits8(value));
  }
  return out;
}

uint32_t ReadBitsMsb(const std::vector<uint8_t>& bytes, size_t start_bit, size_t bit_count) {
  uint32_t value = 0;
  for (size_t i = 0; i < bit_count; ++i) {
    const size_t bit_pos = start_bit + i;
    const size_t byte_index = bit_pos / 8;
    if (byte_index >= bytes.size()) {
      break;
    }
    const size_t bit_in_byte = 7 - (bit_pos % 8);
    const uint32_t bit = static_cast<uint32_t>((bytes[byte_index] >> bit_in_byte) & 0x1U);
    value = (value << 1) | bit;
  }
  return value;
}

char AisSixBitArmoring(uint8_t value) {
  uint8_t out = static_cast<uint8_t>(value + 48U);
  if (out > 87U) {
    out = static_cast<uint8_t>(out + 8U);
  }
  return static_cast<char>(out);
}

bool BuildAivdmPayload(const std::vector<uint8_t>& message_bytes, std::string* payload, int* fill_bits) {
  if (payload == nullptr || fill_bits == nullptr) {
    return false;
  }
  payload->clear();
  *fill_bits = 0;
  if (message_bytes.empty()) {
    return false;
  }

  const size_t bit_len = message_bytes.size() * 8;
  const size_t chunk_count = (bit_len + 5U) / 6U;
  *fill_bits = static_cast<int>(chunk_count * 6U - bit_len);
  payload->reserve(chunk_count);

  for (size_t chunk = 0; chunk < chunk_count; ++chunk) {
    uint8_t six = 0;
    for (size_t b = 0; b < 6; ++b) {
      const size_t bit_pos = chunk * 6U + b;
      uint8_t bit = 0;
      if (bit_pos < bit_len) {
        const size_t byte_index = bit_pos / 8U;
        const size_t bit_in_byte = 7U - (bit_pos % 8U);
        bit = static_cast<uint8_t>((message_bytes[byte_index] >> bit_in_byte) & 0x1U);
      }
      six = static_cast<uint8_t>((six << 1U) | bit);
    }
    payload->push_back(AisSixBitArmoring(six));
  }
  return !payload->empty();
}

uint8_t NmeaChecksum(const std::string& body) {
  uint8_t checksum = 0;
  for (char c : body) {
    checksum ^= static_cast<uint8_t>(c);
  }
  return checksum;
}

uint64_t Fnv1a64(const std::string& value) {
  uint64_t hash = 1469598103934665603ULL;
  for (unsigned char c : value) {
    hash ^= static_cast<uint64_t>(c);
    hash *= 1099511628211ULL;
  }
  return hash;
}

bool IsDuplicateFrame(uint64_t hash) {
  ++g_sequence;
  while (!g_recent_frames.empty()) {
    const uint64_t age = g_sequence - g_recent_frames.front().seq;
    if (age <= kRecentFrameWindow && g_recent_frames.size() <= kMaxRecentFrames) {
      break;
    }
    g_recent_frames.pop_front();
  }

  for (const RecentFrame& entry : g_recent_frames) {
    if (entry.hash == hash) {
      g_metric_duplicates.fetch_add(1, std::memory_order_relaxed);
      return true;
    }
  }

  g_recent_frames.push_back(RecentFrame{.hash = hash, .seq = g_sequence});
  return false;
}

bool TryDecodeFrame(const std::vector<int>& stuffed_bits, std::vector<uint8_t>* message_bytes,
                    uint32_t* message_type, uint32_t* mmsi) {
  if (message_bytes == nullptr || message_type == nullptr || mmsi == nullptr) {
    return false;
  }

  std::vector<int> raw_bits;
  if (!UnstuffBits(stuffed_bits, &raw_bits)) {
    return false;
  }
  if (raw_bits.size() < 80) {
    return false;
  }
  const size_t usable = (raw_bits.size() / 8U) * 8U;
  raw_bits.resize(usable);
  if (raw_bits.size() < 80) {
    return false;
  }

  for (bool lsb_first : {true, false}) {
    const std::vector<uint8_t> candidate = PackBits(raw_bits, lsb_first);
    if (!ValidateFcsX25(candidate)) {
      continue;
    }
    if (candidate.size() <= 2) {
      continue;
    }

    std::vector<uint8_t> payload(candidate.begin(), candidate.end() - 2);
    std::vector<std::vector<uint8_t>> variants;
    variants.push_back(payload);
    variants.push_back(ReverseBitsPerByte(payload));

    for (const std::vector<uint8_t>& variant : variants) {
      const uint32_t type = ReadBitsMsb(variant, 0, 6);
      const uint32_t parsed_mmsi = ReadBitsMsb(variant, 8, 30);
      if (type == 0 || type > 27) {
        continue;
      }
      if (parsed_mmsi == 0U || parsed_mmsi > 999999999U) {
        continue;
      }
      *message_bytes = variant;
      *message_type = type;
      *mmsi = parsed_mmsi;
      return true;
    }
  }
  return false;
}

void EmitAisMessage(const std::vector<uint8_t>& message_bytes, uint32_t message_type, uint32_t mmsi,
                    bool channel_a, multi_radio_emit_message_fn emit_fn, void* user_data) {
  std::string payload;
  int fill_bits = 0;
  if (!BuildAivdmPayload(message_bytes, &payload, &fill_bits)) {
    return;
  }

  const char channel_char = channel_a ? 'A' : 'B';
  std::ostringstream body;
  body << "AIVDM,1,1,," << channel_char << "," << payload << "," << fill_bits;
  const std::string body_str = body.str();
  const uint8_t checksum = NmeaChecksum(body_str);

  char checksum_hex[3] = {'0', '0', '\0'};
  std::snprintf(checksum_hex, sizeof(checksum_hex), "%02X", checksum);

  std::string sentence = "!";
  sentence += body_str;
  sentence += "*";
  sentence += checksum_hex;

  const uint64_t dedup_hash = Fnv1a64(sentence);
  if (IsDuplicateFrame(dedup_hash)) {
    return;
  }

  const uint64_t emitted_now = g_metric_emitted.fetch_add(1, std::memory_order_relaxed) + 1;

  std::ostringstream fields_json;
  fields_json << "{\"mmsi\":\"" << mmsi << "\",\"msg_type\":\"" << message_type << "\",\"channel\":\""
              << (channel_a ? "AIS1" : "AIS2") << "\",\"decoder\":\"gmsk\""
              << ",\"metric_blocks\":\"" << g_metric_blocks.load(std::memory_order_relaxed) << "\""
              << ",\"metric_flags\":\"" << g_metric_flags.load(std::memory_order_relaxed) << "\""
              << ",\"metric_candidates\":\"" << g_metric_candidates.load(std::memory_order_relaxed) << "\""
              << ",\"metric_crc_ok\":\"" << g_metric_crc_ok.load(std::memory_order_relaxed) << "\""
              << ",\"metric_crc_fail\":\"" << g_metric_crc_fail.load(std::memory_order_relaxed) << "\""
              << ",\"metric_duplicates\":\"" << g_metric_duplicates.load(std::memory_order_relaxed) << "\""
              << ",\"metric_emitted\":\"" << emitted_now << "\"}";

  const double frequency_hz = static_cast<double>(channel_a ? kAis1Hz : kAis2Hz);
  emit_fn("AIS", sentence.c_str(), frequency_hz, 0, fields_json.str().c_str(), user_data);
}

bool DecodeAndEmitFromBits(const std::vector<int>& bits, bool channel_a, multi_radio_emit_message_fn emit_fn,
                           void* user_data) {
  if (bits.size() < 128) {
    return false;
  }

  bool emitted_any = false;
  for (size_t start = 0; start + 64 < bits.size(); ++start) {
    if (!IsFlagAt(bits, start)) {
      continue;
    }
    g_metric_flags.fetch_add(1, std::memory_order_relaxed);
    for (size_t end = start + 8; end + 8 <= bits.size(); ++end) {
      if (!IsFlagAt(bits, end)) {
        continue;
      }
      const size_t frame_len = end - (start + 8);
      if (frame_len < 80 || frame_len > 4096) {
        continue;
      }

      std::vector<int> stuffed(bits.begin() + static_cast<long>(start + 8),
                               bits.begin() + static_cast<long>(end));
      std::vector<uint8_t> message_bytes;
      uint32_t message_type = 0;
      uint32_t mmsi = 0;
      g_metric_candidates.fetch_add(1, std::memory_order_relaxed);
      if (TryDecodeFrame(stuffed, &message_bytes, &message_type, &mmsi)) {
        g_metric_crc_ok.fetch_add(1, std::memory_order_relaxed);
        EmitAisMessage(message_bytes, message_type, mmsi, channel_a, emit_fn, user_data);
        emitted_any = true;
      } else {
        g_metric_crc_fail.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
  return emitted_any;
}

int ProcessIq(const multi_radio_iq_view* iq_view, multi_radio_emit_message_fn emit_fn, void* user_data) {
  if (iq_view == nullptr || emit_fn == nullptr) {
    return -1;
  }

  const bool on_ais1 = IsNearFrequency(iq_view->center_frequency_hz, kAis1Hz, kChannelToleranceHz);
  const bool on_ais2 = IsNearFrequency(iq_view->center_frequency_hz, kAis2Hz, kChannelToleranceHz);
  if (!on_ais1 && !on_ais2) {
    return 0;
  }
  g_metric_blocks.fetch_add(1, std::memory_order_relaxed);

  if (iq_view->sample_rate_hz == 0 || iq_view->sample_count < 512) {
    return 0;
  }

  const std::vector<double> discriminator = BuildFmDiscriminator(iq_view);
  if (discriminator.size() < 512) {
    return 0;
  }

  double abs_mean = 0.0;
  for (double v : discriminator) {
    abs_mean += std::abs(v);
  }
  abs_mean /= static_cast<double>(discriminator.size());
  if (abs_mean < 0.0004) {
    return 0;
  }

  const std::vector<double> resampled =
      ResampleLinear(discriminator, static_cast<double>(iq_view->sample_rate_hz), kResampledRate);
  if (resampled.size() < 512) {
    return 0;
  }

  bool emitted_this_block = false;
  const int preferred_phase = EstimateBestPhase(resampled);
  std::vector<int> phases = {preferred_phase};
  for (int phase = 0; phase < kSamplesPerSymbol; ++phase) {
    if (phase != preferred_phase) {
      phases.push_back(phase);
    }
  }
  for (int phase : phases) {
    for (bool invert : {false, true}) {
      const std::vector<int> symbols = BuildSymbolStream(resampled, phase, invert);
      if (symbols.size() < 96) {
        continue;
      }
      for (int initial_level : {0, 1}) {
        const std::vector<int> bits = NrziDecode(symbols, initial_level);
        if (DecodeAndEmitFromBits(bits, on_ais1, emit_fn, user_data)) {
          emitted_this_block = true;
        }
      }
    }
  }

  const uint64_t blocks = g_metric_blocks.load(std::memory_order_relaxed);
  if ((blocks % kMetricReportEveryBlocks) == 0) {
    std::ostringstream metrics_json;
    metrics_json << "{\"kind\":\"metric\""
                 << ",\"channel\":\"" << (on_ais1 ? "AIS1" : "AIS2") << "\""
                 << ",\"metric_blocks\":\"" << blocks << "\""
                 << ",\"metric_flags\":\"" << g_metric_flags.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_candidates\":\"" << g_metric_candidates.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_crc_ok\":\"" << g_metric_crc_ok.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_crc_fail\":\"" << g_metric_crc_fail.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_duplicates\":\"" << g_metric_duplicates.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_emitted\":\"" << g_metric_emitted.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_abs_mean\":\"" << abs_mean << "\""
                 << ",\"metric_emitted_this_block\":\"" << (emitted_this_block ? "1" : "0") << "\"}";
    const double frequency_hz = static_cast<double>(on_ais1 ? kAis1Hz : kAis2Hz);
    emit_fn("AIS", "[AIS_METRICS]", frequency_hz, 0, metrics_json.str().c_str(), user_data);
  }

  return 0;
}

int Flush(multi_radio_emit_message_fn /*emit_fn*/, void* /*user_data*/) { return 0; }

void Shutdown() {
  g_recent_frames.clear();
  g_sequence = 0;
  g_metric_blocks.store(0, std::memory_order_relaxed);
  g_metric_flags.store(0, std::memory_order_relaxed);
  g_metric_candidates.store(0, std::memory_order_relaxed);
  g_metric_crc_ok.store(0, std::memory_order_relaxed);
  g_metric_crc_fail.store(0, std::memory_order_relaxed);
  g_metric_duplicates.store(0, std::memory_order_relaxed);
  g_metric_emitted.store(0, std::memory_order_relaxed);
}

const multi_radio_plugin_descriptor kDescriptor = {
    .plugin_name = "ais_wrapper",
    .plugin_version = "0.3.0",
    .api_version = MULTI_RADIO_PLUGIN_API_VERSION,
    .supported_signals_csv = "AIS",
    .init = &Init,
    .process_iq = &ProcessIq,
    .flush = &Flush,
    .shutdown = &Shutdown,
};

}  // namespace

extern "C" const multi_radio_plugin_descriptor* multi_radio_get_plugin_descriptor() {
  return &kDescriptor;
}
