#include "multi_radio/plugin_api.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
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
constexpr double kGmskModulationIndex = 0.5;
constexpr double kGmskBt = 0.4;
constexpr int kTimingSamplesPerSymbol = 4;
constexpr int kLegacySamplesPerSymbol = 5;
constexpr double kMinBaudRate = 9300.0;
constexpr double kMaxBaudRate = 9900.0;
constexpr size_t kMaxRecentFrames = 1024;
constexpr uint64_t kRecentFrameWindow = 4096;
constexpr uint64_t kMetricReportEveryBlocks = 1;
constexpr double kPi = 3.14159265358979323846;

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
std::atomic<uint64_t> g_metric_crc_ok_ais1{0};
std::atomic<uint64_t> g_metric_crc_ok_ais2{0};
std::atomic<uint64_t> g_metric_crc_fail_ais1{0};
std::atomic<uint64_t> g_metric_crc_fail_ais2{0};
std::atomic<uint64_t> g_metric_duplicates{0};
std::atomic<uint64_t> g_metric_emitted{0};
std::atomic<bool> g_initialized{false};

struct AutotuneProfile {
  const char* name = "";
  double afc_alpha = 0.0;
  double timing_gain_mu = 0.0;
  double timing_gain_omega = 0.0;
};

constexpr std::array<AutotuneProfile, 3> kAutotuneProfiles = {{
    {"stable", 0.0015, 0.020, 0.0006},
    {"balanced", 0.0030, 0.030, 0.0008},
    {"aggressive", 0.0060, 0.045, 0.0012},
}};
constexpr uint64_t kAfcUpdatePeriodBlocks = 10;
constexpr uint64_t kBaudUpdatePeriodBlocks = 20;
constexpr double kBaudTrimAlpha = 0.25;

constexpr auto kAutotuneWindow = std::chrono::seconds(60);
constexpr auto kAutotuneRecoveryWindow = std::chrono::seconds(300);
constexpr uint64_t kAutotuneRecoveryFailThreshold = 300;

struct AutotuneProfileScore {
  uint64_t crc_ok = 0;
  uint64_t crc_fail = 0;
  uint64_t windows = 0;
};

struct ChannelAutotuneState {
  bool initialized = false;
  bool locked = false;
  size_t active_profile = 0;
  double afc_offset_rad = 0.0;
  uint64_t afc_blocks_since_update = 0;
  double afc_mean_acc = 0.0;
  uint64_t afc_mean_count = 0;
  double baud_estimate_hz = kSymbolRate;
  uint64_t baud_blocks_since_update = 0;
  double baud_mean_acc = 0.0;
  uint64_t baud_mean_count = 0;
  std::chrono::steady_clock::time_point window_start;
  uint64_t window_crc_ok_start = 0;
  uint64_t window_crc_fail_start = 0;
  std::array<AutotuneProfileScore, kAutotuneProfiles.size()> scores{};
};

ChannelAutotuneState g_autotune_ais1;
ChannelAutotuneState g_autotune_ais2;

int Init(const char* config_json) {
  (void)config_json;
  if (!g_initialized.exchange(true, std::memory_order_relaxed)) {
    g_recent_frames.clear();
    g_sequence = 0;
    g_metric_blocks.store(0, std::memory_order_relaxed);
    g_metric_flags.store(0, std::memory_order_relaxed);
    g_metric_candidates.store(0, std::memory_order_relaxed);
    g_metric_crc_ok.store(0, std::memory_order_relaxed);
    g_metric_crc_fail.store(0, std::memory_order_relaxed);
    g_metric_crc_ok_ais1.store(0, std::memory_order_relaxed);
    g_metric_crc_ok_ais2.store(0, std::memory_order_relaxed);
    g_metric_crc_fail_ais1.store(0, std::memory_order_relaxed);
    g_metric_crc_fail_ais2.store(0, std::memory_order_relaxed);
    g_metric_duplicates.store(0, std::memory_order_relaxed);
    g_metric_emitted.store(0, std::memory_order_relaxed);
    g_autotune_ais1 = ChannelAutotuneState{};
    g_autotune_ais2 = ChannelAutotuneState{};
  }
  return 0;
}

void StartAutotuneWindow(ChannelAutotuneState* state, uint64_t crc_ok_now, uint64_t crc_fail_now) {
  if (state == nullptr) {
    return;
  }
  state->window_start = std::chrono::steady_clock::now();
  state->window_crc_ok_start = crc_ok_now;
  state->window_crc_fail_start = crc_fail_now;
}

double ProfileScore(const AutotuneProfileScore& score) {
  if (score.crc_ok == 0) {
    return -(static_cast<double>(score.crc_fail));
  }
  return static_cast<double>(score.crc_ok) / static_cast<double>(score.crc_fail + 1);
}

void AdvanceAutotune(ChannelAutotuneState* state, uint64_t crc_ok_now, uint64_t crc_fail_now) {
  if (state == nullptr) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!state->initialized) {
    state->initialized = true;
    StartAutotuneWindow(state, crc_ok_now, crc_fail_now);
    return;
  }

  const auto elapsed = now - state->window_start;
  const auto active_idx = std::min(state->active_profile, kAutotuneProfiles.size() - 1);
  const uint64_t ok_delta = crc_ok_now - state->window_crc_ok_start;
  const uint64_t fail_delta = crc_fail_now - state->window_crc_fail_start;

  if (!state->locked) {
    if (elapsed < kAutotuneWindow) {
      return;
    }
    auto& profile_score = state->scores[active_idx];
    profile_score.crc_ok += ok_delta;
    profile_score.crc_fail += fail_delta;
    ++profile_score.windows;

    if (state->active_profile + 1 < kAutotuneProfiles.size()) {
      ++state->active_profile;
      StartAutotuneWindow(state, crc_ok_now, crc_fail_now);
      return;
    }

    // Completed one full probe sweep (~3 minutes). Lock to best profile.
    size_t best_idx = 0;
    double best = ProfileScore(state->scores[0]);
    for (size_t i = 1; i < state->scores.size(); ++i) {
      const double candidate = ProfileScore(state->scores[i]);
      if (candidate > best) {
        best = candidate;
        best_idx = i;
      }
    }
    state->active_profile = best_idx;
    state->locked = true;
    StartAutotuneWindow(state, crc_ok_now, crc_fail_now);
    return;
  }

  if (elapsed < kAutotuneRecoveryWindow) {
    return;
  }

  // If locked profile stalls, restart probing from next profile.
  if (ok_delta == 0 && fail_delta >= kAutotuneRecoveryFailThreshold) {
    const size_t next_profile = (active_idx + 1U) % kAutotuneProfiles.size();
    state->locked = false;
    state->active_profile = next_profile;
    state->scores = {};
  }
  StartAutotuneWindow(state, crc_ok_now, crc_fail_now);
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

std::vector<double> BuildGaussianMatchedFilterTaps(double bt, int samples_per_symbol, int span_symbols) {
  std::vector<double> taps;
  if (bt <= 0.0 || samples_per_symbol <= 0 || span_symbols <= 0) {
    return taps;
  }
  int tap_count = span_symbols * samples_per_symbol;
  if ((tap_count % 2) == 0) {
    ++tap_count;
  }
  taps.resize(static_cast<size_t>(tap_count), 0.0);

  // Gaussian pulse-shaping/matched-filter approximation from BT product.
  const double sigma_symbols = std::sqrt(std::log(2.0)) / (2.0 * kPi * bt);
  const int center = tap_count / 2;
  double sum = 0.0;
  for (int n = 0; n < tap_count; ++n) {
    const double t_symbols = static_cast<double>(n - center) / static_cast<double>(samples_per_symbol);
    const double exponent = -0.5 * (t_symbols * t_symbols) / (sigma_symbols * sigma_symbols);
    const double value = std::exp(exponent);
    taps[static_cast<size_t>(n)] = value;
    sum += value;
  }
  if (sum > 0.0) {
    for (double& tap : taps) {
      tap /= sum;
    }
  }
  return taps;
}

std::vector<double> ApplyFirCentered(const std::vector<double>& in, const std::vector<double>& taps) {
  std::vector<double> out;
  if (in.empty() || taps.empty()) {
    return out;
  }
  out.assign(in.size(), 0.0);
  const int half = static_cast<int>(taps.size() / 2);
  for (size_t n = 0; n < in.size(); ++n) {
    double acc = 0.0;
    for (size_t k = 0; k < taps.size(); ++k) {
      const int idx = static_cast<int>(n) + static_cast<int>(k) - half;
      if (idx < 0 || idx >= static_cast<int>(in.size())) {
        continue;
      }
      acc += in[static_cast<size_t>(idx)] * taps[k];
    }
    out[n] = acc;
  }
  return out;
}

double InterpolateLinear(const std::vector<double>& samples, double pos) {
  if (samples.empty()) {
    return 0.0;
  }
  if (pos <= 0.0) {
    return samples.front();
  }
  const double max_pos = static_cast<double>(samples.size() - 1);
  if (pos >= max_pos) {
    return samples.back();
  }
  const size_t left = static_cast<size_t>(std::floor(pos));
  const size_t right = std::min(left + 1, samples.size() - 1);
  const double frac = pos - static_cast<double>(left);
  return samples[left] + (samples[right] - samples[left]) * frac;
}

struct TimingRecoveryResult {
  std::vector<double> symbol_samples;
  double avg_abs_error = 0.0;
  double avg_omega = 0.0;
  bool lock = false;
};

TimingRecoveryResult RecoverClockGardner(const std::vector<double>& samples, double nominal_sps,
                                         double gain_mu, double gain_omega) {
  TimingRecoveryResult out;
  if (samples.size() < 256 || nominal_sps < 2.0) {
    return out;
  }

  double omega = nominal_sps;
  const double omega_min = nominal_sps * 0.85;
  const double omega_max = nominal_sps * 1.15;
  double mu = 0.0;
  size_t base = static_cast<size_t>(std::ceil(nominal_sps));
  bool have_prev = false;
  double prev = 0.0;
  double error_sum = 0.0;
  size_t error_count = 0;
  double omega_sum = 0.0;
  size_t omega_count = 0;

  while (base + 4 < samples.size()) {
    const double t_now = static_cast<double>(base) + mu;
    if (t_now + 1.0 >= static_cast<double>(samples.size())) {
      break;
    }
    const double cur = InterpolateLinear(samples, t_now);
    out.symbol_samples.push_back(cur);

    if (have_prev) {
      const double half_t = t_now - 0.5 * omega;
      const double mid = InterpolateLinear(samples, half_t);
      const double err = (prev - cur) * mid;
      error_sum += std::abs(err);
      ++error_count;

      omega = std::clamp(omega + gain_omega * err, omega_min, omega_max);
      omega_sum += omega;
      ++omega_count;
      mu += omega + gain_mu * err;
    } else {
      mu += omega;
      have_prev = true;
    }
    prev = cur;

    const double advance = std::floor(mu);
    const size_t step = static_cast<size_t>(std::max(1.0, advance));
    base += step;
    mu -= advance;

    if (out.symbol_samples.size() > 12000) {
      break;
    }
  }

  if (error_count > 0) {
    out.avg_abs_error = error_sum / static_cast<double>(error_count);
  }
  if (omega_count > 0) {
    out.avg_omega = omega_sum / static_cast<double>(omega_count);
  } else {
    out.avg_omega = nominal_sps;
  }
  out.lock = out.symbol_samples.size() >= 96;
  return out;
}

std::vector<int> BuildSymbolsFromRecovered(const std::vector<double>& samples, bool invert) {
  std::vector<int> symbols;
  if (samples.empty()) {
    return {};
  }
  symbols.reserve(samples.size());
  for (double sample : samples) {
    int bit = sample >= 0.0 ? 1 : 0;
    if (invert) {
      bit ^= 1;
    }
    symbols.push_back(bit);
  }
  return symbols;
}

int EstimateBestPhaseLegacy(const std::vector<double>& samples, int samples_per_symbol) {
  int best_phase = 0;
  int best_score = -1;
  if (samples_per_symbol <= 1) {
    return 0;
  }
  for (int phase = 0; phase < samples_per_symbol; ++phase) {
    int score = 0;
    int prev_sign = 0;
    bool have_prev = false;
    int considered = 0;
    for (size_t i = static_cast<size_t>(phase); i < samples.size() && considered < 400;
         i += static_cast<size_t>(samples_per_symbol), ++considered) {
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

std::vector<int> BuildSymbolStreamLegacy(const std::vector<double>& samples, int phase, bool invert,
                                         int samples_per_symbol) {
  std::vector<int> symbols;
  if (samples.empty() || samples_per_symbol <= 1) {
    return symbols;
  }
  symbols.reserve(samples.size() / static_cast<size_t>(samples_per_symbol) + 1);
  for (size_t i = static_cast<size_t>(std::max(0, phase)); i < samples.size();
       i += static_cast<size_t>(samples_per_symbol)) {
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
        if (channel_a) {
          g_metric_crc_ok_ais1.fetch_add(1, std::memory_order_relaxed);
        } else {
          g_metric_crc_ok_ais2.fetch_add(1, std::memory_order_relaxed);
        }
        EmitAisMessage(message_bytes, message_type, mmsi, channel_a, emit_fn, user_data);
        emitted_any = true;
      } else {
        g_metric_crc_fail.fetch_add(1, std::memory_order_relaxed);
        if (channel_a) {
          g_metric_crc_fail_ais1.fetch_add(1, std::memory_order_relaxed);
        } else {
          g_metric_crc_fail_ais2.fetch_add(1, std::memory_order_relaxed);
        }
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

  const std::vector<double> discriminator_raw = BuildFmDiscriminator(iq_view);
  if (discriminator_raw.size() < 512) {
    return 0;
  }

  ChannelAutotuneState* autotune_state = on_ais1 ? &g_autotune_ais1 : &g_autotune_ais2;
  const bool autotune_enabled = (iq_view->ais_autotune_enabled != 0);
  const bool baud_tracking = (iq_view->ais_baud_trim_enabled != 0);
  const uint64_t crc_ok_channel_now =
      on_ais1 ? g_metric_crc_ok_ais1.load(std::memory_order_relaxed)
              : g_metric_crc_ok_ais2.load(std::memory_order_relaxed);
  const uint64_t crc_fail_channel_now =
      on_ais1 ? g_metric_crc_fail_ais1.load(std::memory_order_relaxed)
              : g_metric_crc_fail_ais2.load(std::memory_order_relaxed);
  if (autotune_enabled) {
    AdvanceAutotune(autotune_state, crc_ok_channel_now, crc_fail_channel_now);
  } else {
    // Explicit manual mode: no hidden profile/AFC state carries over while disabled.
    autotune_state->initialized = false;
    autotune_state->locked = false;
    autotune_state->active_profile = 0;
    autotune_state->afc_offset_rad = 0.0;
    autotune_state->afc_blocks_since_update = 0;
    autotune_state->afc_mean_acc = 0.0;
    autotune_state->afc_mean_count = 0;
    autotune_state->scores = {};
  }

  const size_t profile_idx = std::min(autotune_state->active_profile, kAutotuneProfiles.size() - 1);
  const AutotuneProfile& profile = kAutotuneProfiles[profile_idx];

  double block_mean = 0.0;
  for (double v : discriminator_raw) {
    block_mean += v;
  }
  block_mean /= static_cast<double>(discriminator_raw.size());

  const bool afc_tracking = autotune_enabled && !autotune_state->locked;
  const double alpha = afc_tracking ? std::clamp(profile.afc_alpha, 0.0, 1.0) : 0.0;
  bool afc_updated_this_block = false;
  if (afc_tracking) {
    autotune_state->afc_mean_acc += block_mean;
    ++autotune_state->afc_mean_count;
    ++autotune_state->afc_blocks_since_update;
    if (autotune_state->afc_blocks_since_update >= kAfcUpdatePeriodBlocks &&
        autotune_state->afc_mean_count > 0) {
      const double block_mean_avg =
          autotune_state->afc_mean_acc / static_cast<double>(autotune_state->afc_mean_count);
      autotune_state->afc_offset_rad =
          autotune_state->afc_offset_rad * (1.0 - alpha) + block_mean_avg * alpha;
      autotune_state->afc_blocks_since_update = 0;
      autotune_state->afc_mean_acc = 0.0;
      autotune_state->afc_mean_count = 0;
      afc_updated_this_block = true;
    }
  } else {
    autotune_state->afc_blocks_since_update = 0;
    autotune_state->afc_mean_acc = 0.0;
    autotune_state->afc_mean_count = 0;
  }

  std::vector<double> discriminator;
  discriminator.reserve(discriminator_raw.size());
  for (double v : discriminator_raw) {
    discriminator.push_back(v - autotune_state->afc_offset_rad);
  }

  autotune_state->baud_estimate_hz =
      std::clamp(autotune_state->baud_estimate_hz, kMinBaudRate, kMaxBaudRate);
  if (!baud_tracking) {
    // Explicit manual mode: keep nominal AIS symbol rate when trim is disabled.
    autotune_state->baud_estimate_hz = kSymbolRate;
    autotune_state->baud_blocks_since_update = 0;
    autotune_state->baud_mean_acc = 0.0;
    autotune_state->baud_mean_count = 0;
  }
  const double timing_resampled_rate =
      autotune_state->baud_estimate_hz * static_cast<double>(kTimingSamplesPerSymbol);
  const double legacy_resampled_rate =
      autotune_state->baud_estimate_hz * static_cast<double>(kLegacySamplesPerSymbol);

  double abs_mean = 0.0;
  for (double v : discriminator) {
    abs_mean += std::abs(v);
  }
  abs_mean /= static_cast<double>(discriminator.size());

  const std::vector<double> resampled =
      ResampleLinear(discriminator, static_cast<double>(iq_view->sample_rate_hz), timing_resampled_rate);
  const bool demod_ready = resampled.size() >= 512;

  static const std::vector<double> kGaussianTaps =
      BuildGaussianMatchedFilterTaps(kGmskBt, kTimingSamplesPerSymbol, 6);
  const std::vector<double> matched = ApplyFirCentered(resampled, kGaussianTaps);

  double power_mean = 0.0;
  for (double v : matched) {
    power_mean += v * v;
  }
  const double demod_rms =
      matched.empty() ? 0.0 : std::sqrt(power_mean / static_cast<double>(matched.size()));

  const TimingRecoveryResult timing = RecoverClockGardner(
      matched, static_cast<double>(kTimingSamplesPerSymbol), profile.timing_gain_mu,
      profile.timing_gain_omega);
  const bool timing_ready = timing.lock;
  bool baud_updated_this_block = false;
  double measured_baud_hz = 0.0;
  if (baud_tracking && timing_ready && timing.avg_omega > 1e-6) {
    measured_baud_hz = std::clamp(timing_resampled_rate / timing.avg_omega, kMinBaudRate, kMaxBaudRate);
    autotune_state->baud_mean_acc += measured_baud_hz;
    ++autotune_state->baud_mean_count;
    ++autotune_state->baud_blocks_since_update;
    if (autotune_state->baud_blocks_since_update >= kBaudUpdatePeriodBlocks &&
        autotune_state->baud_mean_count > 0) {
      const double baud_avg = autotune_state->baud_mean_acc /
                              static_cast<double>(autotune_state->baud_mean_count);
      autotune_state->baud_estimate_hz =
          std::clamp(autotune_state->baud_estimate_hz * (1.0 - kBaudTrimAlpha) +
                         baud_avg * kBaudTrimAlpha,
                     kMinBaudRate, kMaxBaudRate);
      autotune_state->baud_blocks_since_update = 0;
      autotune_state->baud_mean_acc = 0.0;
      autotune_state->baud_mean_count = 0;
      baud_updated_this_block = true;
    }
  } else if (!baud_tracking) {
    autotune_state->baud_blocks_since_update = 0;
    autotune_state->baud_mean_acc = 0.0;
    autotune_state->baud_mean_count = 0;
  }
  const std::vector<double> legacy_resampled = ResampleLinear(
      discriminator, static_cast<double>(iq_view->sample_rate_hz), legacy_resampled_rate);
  const bool legacy_ready = legacy_resampled.size() >= 512;

  bool emitted_this_block = false;
  bool decode_attempted = false;
  std::string decode_path = "none";
  std::string debug_state = demod_ready ? "TIMING_NO_LOCK" : "DEMOD_TOO_SHORT";
  if (timing_ready) {
    debug_state = "DECODE_ATTEMPT";
    decode_attempted = true;
    for (bool invert : {false, true}) {
      const std::vector<int> symbols = BuildSymbolsFromRecovered(timing.symbol_samples, invert);
      if (symbols.size() < 96) {
        continue;
      }
      for (int initial_level : {0, 1}) {
        const std::vector<int> bits = NrziDecode(symbols, initial_level);
        if (DecodeAndEmitFromBits(bits, on_ais1, emit_fn, user_data)) {
          emitted_this_block = true;
          decode_path = "timing_gardner";
        }
      }
    }
  }

  if (!emitted_this_block && legacy_ready) {
    decode_attempted = true;
    debug_state = "LEGACY_PHASE_SCAN";
    const int preferred_phase = EstimateBestPhaseLegacy(legacy_resampled, kLegacySamplesPerSymbol);
    std::vector<int> phases = {preferred_phase};
    for (int phase = 0; phase < kLegacySamplesPerSymbol; ++phase) {
      if (phase != preferred_phase) {
        phases.push_back(phase);
      }
    }
    for (int phase : phases) {
      for (bool invert : {false, true}) {
        const std::vector<int> symbols =
            BuildSymbolStreamLegacy(legacy_resampled, phase, invert, kLegacySamplesPerSymbol);
        if (symbols.size() < 96) {
          continue;
        }
        for (int initial_level : {0, 1}) {
          const std::vector<int> bits = NrziDecode(symbols, initial_level);
          if (DecodeAndEmitFromBits(bits, on_ais1, emit_fn, user_data)) {
            emitted_this_block = true;
            decode_path = "legacy_phase_scan";
          }
        }
      }
    }
  }

  if (emitted_this_block) {
    debug_state = "AIS_DECODED";
  }

  const double afc_offset_hz = autotune_state->afc_offset_rad *
                               (static_cast<double>(iq_view->sample_rate_hz) / (2.0 * kPi));
  const uint64_t channel_crc_ok =
      on_ais1 ? g_metric_crc_ok_ais1.load(std::memory_order_relaxed)
              : g_metric_crc_ok_ais2.load(std::memory_order_relaxed);
  const uint64_t channel_crc_fail =
      on_ais1 ? g_metric_crc_fail_ais1.load(std::memory_order_relaxed)
              : g_metric_crc_fail_ais2.load(std::memory_order_relaxed);

  const uint64_t blocks = g_metric_blocks.load(std::memory_order_relaxed);
  if ((blocks % kMetricReportEveryBlocks) == 0) {
    std::string decode_mode = "continuous_gmsk_gardner";
    if (autotune_enabled) {
      decode_mode += "_afc";
    }
    if (baud_tracking) {
      decode_mode += "_baudtrim";
    }
    std::ostringstream metrics_json;
    metrics_json << "{\"kind\":\"metric\""
                 << ",\"channel\":\"" << (on_ais1 ? "AIS1" : "AIS2") << "\""
                 << ",\"metric_decode_mode\":\"" << decode_mode << "\""
                 << ",\"metric_blocks\":\"" << blocks << "\""
                 << ",\"metric_flags\":\"" << g_metric_flags.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_candidates\":\"" << g_metric_candidates.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_crc_ok\":\"" << g_metric_crc_ok.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_crc_fail\":\"" << g_metric_crc_fail.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_crc_ok_channel\":\"" << channel_crc_ok << "\""
                 << ",\"metric_crc_fail_channel\":\"" << channel_crc_fail << "\""
                 << ",\"metric_duplicates\":\"" << g_metric_duplicates.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_emitted\":\"" << g_metric_emitted.load(std::memory_order_relaxed) << "\""
                 << ",\"metric_abs_mean\":\"" << abs_mean << "\""
                 << ",\"metric_demod_ready\":\"" << (demod_ready ? "1" : "0") << "\""
                 << ",\"metric_demod_resampled_samples\":\"" << resampled.size() << "\""
                 << ",\"metric_demod_rms\":\"" << demod_rms << "\""
                 << ",\"metric_afc_alpha\":\"" << alpha << "\""
                 << ",\"metric_afc_offset_rad\":\"" << autotune_state->afc_offset_rad << "\""
                 << ",\"metric_afc_offset_hz\":\"" << afc_offset_hz << "\""
                 << ",\"metric_afc_tracking\":\"" << (afc_tracking ? "1" : "0") << "\""
                 << ",\"metric_afc_update_period_blocks\":\"" << kAfcUpdatePeriodBlocks << "\""
                 << ",\"metric_afc_updated\":\"" << (afc_updated_this_block ? "1" : "0") << "\""
                 << ",\"metric_afc_hold_blocks\":\"" << autotune_state->afc_blocks_since_update << "\""
                 << ",\"metric_baud_estimate_hz\":\"" << autotune_state->baud_estimate_hz << "\""
                 << ",\"metric_baud_measured_hz\":\"" << measured_baud_hz << "\""
                 << ",\"metric_baud_tracking\":\"" << (baud_tracking ? "1" : "0") << "\""
                 << ",\"metric_baud_update_period_blocks\":\"" << kBaudUpdatePeriodBlocks << "\""
                 << ",\"metric_baud_updated\":\"" << (baud_updated_this_block ? "1" : "0") << "\""
                 << ",\"metric_baud_hold_blocks\":\"" << autotune_state->baud_blocks_since_update << "\""
                 << ",\"metric_timing_omega_avg\":\"" << timing.avg_omega << "\""
                 << ",\"metric_gmsk_bt\":\"" << kGmskBt << "\""
                 << ",\"metric_gmsk_h\":\"" << kGmskModulationIndex << "\""
                 << ",\"metric_timing_sps\":\"" << kTimingSamplesPerSymbol << "\""
                 << ",\"metric_timing_symbols\":\"" << timing.symbol_samples.size() << "\""
                 << ",\"metric_timing_avg_abs_error\":\"" << timing.avg_abs_error << "\""
                 << ",\"metric_timing_lock\":\"" << (timing_ready ? "1" : "0") << "\""
                 << ",\"metric_autotune_enabled\":\"" << (autotune_enabled ? "1" : "0") << "\""
                 << ",\"metric_autotune_profile\":\"" << profile_idx << "\""
                 << ",\"metric_autotune_profile_name\":\"" << profile.name << "\""
                 << ",\"metric_autotune_locked\":\"" << (autotune_state->locked ? "1" : "0") << "\""
                 << ",\"metric_autotune_window_s\":\"" << kAutotuneWindow.count() << "\""
                 << ",\"metric_legacy_sps\":\"" << kLegacySamplesPerSymbol << "\""
                 << ",\"metric_legacy_ready\":\"" << (legacy_ready ? "1" : "0") << "\""
                 << ",\"metric_legacy_symbols\":\""
                 << (legacy_resampled.size() / static_cast<size_t>(kLegacySamplesPerSymbol)) << "\""
                 << ",\"metric_decode_path\":\"" << decode_path << "\""
                 << ",\"metric_decode_attempted\":\"" << (decode_attempted ? "1" : "0") << "\""
                 << ",\"metric_debug_state\":\"" << debug_state << "\""
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
  g_metric_crc_ok_ais1.store(0, std::memory_order_relaxed);
  g_metric_crc_ok_ais2.store(0, std::memory_order_relaxed);
  g_metric_crc_fail_ais1.store(0, std::memory_order_relaxed);
  g_metric_crc_fail_ais2.store(0, std::memory_order_relaxed);
  g_metric_duplicates.store(0, std::memory_order_relaxed);
  g_metric_emitted.store(0, std::memory_order_relaxed);
  g_autotune_ais1 = ChannelAutotuneState{};
  g_autotune_ais2 = ChannelAutotuneState{};
  g_initialized.store(false, std::memory_order_relaxed);
}

const multi_radio_plugin_descriptor kDescriptor = {
    .plugin_name = "ais_wrapper",
    .plugin_version = "0.11.0",
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
