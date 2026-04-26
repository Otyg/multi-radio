#include "multi_radio/plugin_api.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double kDscSymbolRate = 1200.0;
constexpr double kDscDecodeSampleRate = 9600.0;
constexpr int kDscSamplesPerSymbol = 8;
constexpr double kDscMarkToneHz = 1300.0;
constexpr double kDscSpaceToneHz = 2100.0;
constexpr size_t kMinDiscriminatorSamples = 1024;
constexpr size_t kMinSymbolsForDecode = 40;
constexpr size_t kMinWordsForEmit = 16;
constexpr size_t kMaxWordsForEmit = 64;
constexpr size_t kMaxSymbolsAnalyze = 900;
constexpr double kMinConfidence = 0.25;
constexpr double kPi = 3.14159265358979323846;
constexpr size_t kMaxRecentFrames = 512;
constexpr uint64_t kRecentFrameWindow = 4096;

struct RecentFrame {
  uint64_t hash = 0;
  uint64_t seq = 0;
};

struct BitStreamCandidate {
  std::vector<int> bits;
  double avg_abs_contrast = 0.0;
  int transitions = 0;
  int bit_phase = 0;
};

enum class WordDataLayout {
  kLow7High3,
  kHigh7Low3,
};

enum class FecRule {
  kOnesCount,
  kZerosCount,
  kOnesCountInverted,
  kZerosCountInverted,
};

struct SymbolWord {
  uint8_t value = 0;
  bool fec_ok = false;
  uint8_t check_bits = 0;
  uint8_t expected_check_bits = 0;
};

struct SymbolDecodeResult {
  std::vector<SymbolWord> symbols;
  double fec_ok_ratio = 0.0;
  WordDataLayout layout = WordDataLayout::kLow7High3;
  FecRule rule = FecRule::kOnesCount;
};

struct ParsedDscFrame {
  bool parsed = false;
  bool eos_found = false;
  size_t start_index = 0;
  size_t eos_index = 0;
  uint8_t format = 0;
  std::vector<uint8_t> address;
  uint8_t category = 0;
  std::vector<uint8_t> self_id;
  uint8_t telecommand1 = 0;
  uint8_t telecommand2 = 0;
  std::vector<uint8_t> payload;
  bool ecc_present = false;
  uint8_t ecc = 0;
  bool ecc_ok = false;
  uint8_t eos = 0;
  int address_numeric_count = 0;
  int self_numeric_count = 0;
  double frame_fec_ok_ratio = 0.0;
  double score = 0.0;
};

uint64_t g_sequence = 0;
std::deque<RecentFrame> g_recent_frames;
std::atomic<uint64_t> g_metric_blocks{0};
std::atomic<uint64_t> g_metric_candidates{0};
std::atomic<uint64_t> g_metric_duplicates{0};
std::atomic<uint64_t> g_metric_emitted{0};
std::atomic<uint64_t> g_metric_frames_parsed{0};
std::atomic<uint64_t> g_metric_ecc_ok{0};
std::atomic<uint64_t> g_metric_ecc_fail{0};
std::atomic<bool> g_initialized{false};

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
    out.push_back(in[left] + (in[right] - in[left]) * frac);
    pos += step;
  }
  return out;
}

double GoertzelPower(const std::vector<double>& samples, size_t start, size_t count,
                     double tone_hz, double sample_rate_hz) {
  if (count == 0 || sample_rate_hz <= 0.0 || tone_hz <= 0.0 || start + count > samples.size()) {
    return 0.0;
  }
  const double omega = 2.0 * kPi * tone_hz / sample_rate_hz;
  const double coeff = 2.0 * std::cos(omega);
  double q0 = 0.0;
  double q1 = 0.0;
  double q2 = 0.0;
  for (size_t i = 0; i < count; ++i) {
    q0 = coeff * q1 - q2 + samples[start + i];
    q2 = q1;
    q1 = q0;
  }
  const double power = q1 * q1 + q2 * q2 - coeff * q1 * q2;
  return std::max(0.0, power);
}

BitStreamCandidate DecodeDscBits(const std::vector<double>& audio, double sample_rate_hz) {
  BitStreamCandidate best;
  if (audio.size() < static_cast<size_t>(kDscSamplesPerSymbol * 16) || sample_rate_hz <= 0.0) {
    return best;
  }

  double best_score = -1.0;
  for (int phase = 0; phase < kDscSamplesPerSymbol; ++phase) {
    std::vector<int> bits;
    bits.reserve(kMaxSymbolsAnalyze);
    double contrast_abs_sum = 0.0;
    int transitions = 0;
    int prev_bit = -1;

    for (size_t start = static_cast<size_t>(phase);
         start + static_cast<size_t>(kDscSamplesPerSymbol) <= audio.size() &&
         bits.size() < kMaxSymbolsAnalyze;
         start += static_cast<size_t>(kDscSamplesPerSymbol)) {
      const double mark_power = GoertzelPower(audio, start, static_cast<size_t>(kDscSamplesPerSymbol),
                                              kDscMarkToneHz, sample_rate_hz);
      const double space_power = GoertzelPower(audio, start, static_cast<size_t>(kDscSamplesPerSymbol),
                                               kDscSpaceToneHz, sample_rate_hz);
      const double denom = mark_power + space_power + 1e-12;
      const double contrast = (mark_power - space_power) / denom;
      const int bit = contrast >= 0.0 ? 1 : 0;
      if (prev_bit >= 0 && prev_bit != bit) {
        ++transitions;
      }
      prev_bit = bit;
      bits.push_back(bit);
      contrast_abs_sum += std::abs(contrast);
    }

    if (bits.size() < kMinSymbolsForDecode) {
      continue;
    }

    const double avg_abs_contrast = contrast_abs_sum / static_cast<double>(bits.size());
    const double transition_ratio = bits.size() > 1
                                        ? static_cast<double>(transitions) /
                                              static_cast<double>(bits.size() - 1)
                                        : 0.0;
    const double score = avg_abs_contrast * (0.70 + 0.60 * std::min(1.0, transition_ratio * 2.0));
    if (score > best_score) {
      best_score = score;
      best.bits = std::move(bits);
      best.avg_abs_contrast = avg_abs_contrast;
      best.transitions = transitions;
      best.bit_phase = phase;
    }
  }
  return best;
}

int Popcount10(uint16_t value) {
  int count = 0;
  for (int i = 0; i < 10; ++i) {
    count += ((value >> i) & 0x1U) != 0U ? 1 : 0;
  }
  return count;
}

std::vector<uint16_t> PackTenBitWords(const std::vector<int>& bits, size_t bit_offset,
                                      size_t max_words) {
  std::vector<uint16_t> words;
  if (bit_offset >= bits.size()) {
    return words;
  }
  words.reserve(max_words);
  for (size_t i = bit_offset; i + 10 <= bits.size() && words.size() < max_words; i += 10) {
    uint16_t word = 0;
    for (size_t b = 0; b < 10; ++b) {
      word = static_cast<uint16_t>((word << 1U) | static_cast<uint16_t>(bits[i + b] & 0x1));
    }
    words.push_back(word);
  }
  return words;
}

size_t SelectWordAlignment(const std::vector<int>& bits) {
  size_t best_offset = 0;
  double best_error = std::numeric_limits<double>::max();
  bool have_best = false;
  for (size_t offset = 0; offset < 10; ++offset) {
    const std::vector<uint16_t> words = PackTenBitWords(bits, offset, 24);
    if (words.size() < 12) {
      continue;
    }
    double error = 0.0;
    for (uint16_t word : words) {
      error += std::abs(static_cast<double>(Popcount10(word) - 5));
    }
    error /= static_cast<double>(words.size());
    if (!have_best || error < best_error) {
      have_best = true;
      best_error = error;
      best_offset = offset;
    }
  }
  return best_offset;
}

std::string FormatWordsHex(const std::vector<uint16_t>& words) {
  std::ostringstream out;
  for (size_t i = 0; i < words.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    char token[8] = {'\0'};
    std::snprintf(token, sizeof(token), "%03X", static_cast<unsigned int>(words[i] & 0x03FFU));
    out << token;
  }
  return out.str();
}

uint8_t Popcount7(uint8_t value) {
  uint8_t count = 0;
  for (int i = 0; i < 7; ++i) {
    count = static_cast<uint8_t>(count + (((value >> i) & 0x1U) != 0U ? 1U : 0U));
  }
  return count;
}

uint8_t ExtractDataBits(uint16_t word, WordDataLayout layout) {
  if (layout == WordDataLayout::kLow7High3) {
    return static_cast<uint8_t>(word & 0x7FU);
  }
  return static_cast<uint8_t>((word >> 3U) & 0x7FU);
}

uint8_t ExtractCheckBits(uint16_t word, WordDataLayout layout) {
  if (layout == WordDataLayout::kLow7High3) {
    return static_cast<uint8_t>((word >> 7U) & 0x07U);
  }
  return static_cast<uint8_t>(word & 0x07U);
}

uint8_t ComputeFecCheck(uint8_t data, FecRule rule) {
  const uint8_t ones = Popcount7(data);
  const uint8_t zeros = static_cast<uint8_t>(7U - ones);
  switch (rule) {
    case FecRule::kOnesCount:
      return static_cast<uint8_t>(ones & 0x07U);
    case FecRule::kZerosCount:
      return static_cast<uint8_t>(zeros & 0x07U);
    case FecRule::kOnesCountInverted:
      return static_cast<uint8_t>((ones ^ 0x07U) & 0x07U);
    case FecRule::kZerosCountInverted:
      return static_cast<uint8_t>((zeros ^ 0x07U) & 0x07U);
  }
  return 0;
}

SymbolDecodeResult DecodeSymbols(const std::vector<uint16_t>& words) {
  SymbolDecodeResult best;
  double best_score = -1.0;
  const std::array<WordDataLayout, 2> layouts = {
      WordDataLayout::kLow7High3,
      WordDataLayout::kHigh7Low3,
  };
  const std::array<FecRule, 4> rules = {
      FecRule::kOnesCount,
      FecRule::kZerosCount,
      FecRule::kOnesCountInverted,
      FecRule::kZerosCountInverted,
  };

  for (WordDataLayout layout : layouts) {
    for (FecRule rule : rules) {
      SymbolDecodeResult candidate;
      candidate.layout = layout;
      candidate.rule = rule;
      candidate.symbols.reserve(words.size());
      size_t fec_ok = 0;
      for (uint16_t word : words) {
        const uint8_t value = ExtractDataBits(word, layout);
        const uint8_t check = ExtractCheckBits(word, layout);
        const uint8_t expected = ComputeFecCheck(value, rule);
        const bool ok = (check == expected);
        if (ok) {
          ++fec_ok;
        }
        candidate.symbols.push_back(SymbolWord{.value = value,
                                               .fec_ok = ok,
                                               .check_bits = check,
                                               .expected_check_bits = expected});
      }
      candidate.fec_ok_ratio = words.empty()
                                   ? 0.0
                                   : static_cast<double>(fec_ok) / static_cast<double>(words.size());

      const double score = candidate.fec_ok_ratio;
      if (score > best_score) {
        best_score = score;
        best = std::move(candidate);
      }
    }
  }

  return best;
}

const char* LayoutName(WordDataLayout layout) {
  switch (layout) {
    case WordDataLayout::kLow7High3:
      return "low7+high3";
    case WordDataLayout::kHigh7Low3:
      return "high7+low3";
  }
  return "unknown";
}

const char* FecRuleName(FecRule rule) {
  switch (rule) {
    case FecRule::kOnesCount:
      return "ones_count";
    case FecRule::kZerosCount:
      return "zeros_count";
    case FecRule::kOnesCountInverted:
      return "ones_count_inverted";
    case FecRule::kZerosCountInverted:
      return "zeros_count_inverted";
  }
  return "unknown";
}

bool IsLikelyEosCode(uint8_t code) {
  return code == 117U || code == 122U || code == 127U;
}

const char* EosLabel(uint8_t code) {
  switch (code) {
    case 117:
      return "ARQ";
    case 122:
      return "ABQ";
    case 127:
      return "EOS";
    default:
      return "UNKNOWN";
  }
}

bool IsTwoDigitCode(uint8_t code) { return code <= 99U; }

int CountNumericCodes(const std::vector<uint8_t>& values) {
  int count = 0;
  for (uint8_t value : values) {
    if (IsTwoDigitCode(value)) {
      ++count;
    }
  }
  return count;
}

std::string EncodeTwoDigitCodes(const std::vector<uint8_t>& values) {
  std::ostringstream out;
  for (uint8_t value : values) {
    if (IsTwoDigitCode(value)) {
      char token[4] = {'\0'};
      std::snprintf(token, sizeof(token), "%02u", static_cast<unsigned int>(value));
      out << token;
    } else {
      out << "??";
    }
  }
  return out.str();
}

std::string FormatCodeList(const std::vector<uint8_t>& values) {
  std::ostringstream out;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << static_cast<unsigned int>(values[i]);
  }
  return out.str();
}

std::string FormatCodeLabel(const char* prefix, uint8_t code) {
  std::ostringstream out;
  out << prefix << "_";
  char token[8] = {'\0'};
  std::snprintf(token, sizeof(token), "%03u", static_cast<unsigned int>(code));
  out << token;
  return out.str();
}

ParsedDscFrame TryParseFrame(const std::vector<SymbolWord>& symbols, size_t start_index) {
  ParsedDscFrame frame;
  frame.start_index = start_index;
  if (start_index + 14 >= symbols.size()) {
    return frame;
  }

  frame.parsed = true;
  frame.format = symbols[start_index].value;
  frame.category = symbols[start_index + 6].value;
  frame.telecommand1 = symbols[start_index + 12].value;
  frame.telecommand2 = symbols[start_index + 13].value;

  frame.address.reserve(5);
  for (size_t i = 0; i < 5; ++i) {
    frame.address.push_back(symbols[start_index + 1 + i].value);
  }

  frame.self_id.reserve(5);
  for (size_t i = 0; i < 5; ++i) {
    frame.self_id.push_back(symbols[start_index + 7 + i].value);
  }

  size_t eos_index = symbols.size();
  for (size_t i = start_index + 14; i < symbols.size(); ++i) {
    if (IsLikelyEosCode(symbols[i].value)) {
      eos_index = i;
      break;
    }
  }

  if (eos_index < symbols.size()) {
    frame.eos_found = true;
    frame.eos_index = eos_index;
    frame.eos = symbols[eos_index].value;
  } else {
    frame.eos_found = false;
    frame.eos_index = std::min(symbols.size() - 1, start_index + static_cast<size_t>(31));
  }

  const size_t payload_start = start_index + 14;
  size_t payload_end_exclusive = frame.eos_index;
  if (frame.eos_found && frame.eos_index > payload_start) {
    frame.ecc_present = true;
    frame.ecc = symbols[frame.eos_index - 1].value;
    payload_end_exclusive = frame.eos_index - 1;
  }

  if (payload_end_exclusive > payload_start && payload_end_exclusive <= symbols.size()) {
    frame.payload.reserve(payload_end_exclusive - payload_start);
    for (size_t i = payload_start; i < payload_end_exclusive; ++i) {
      frame.payload.push_back(symbols[i].value);
    }
  }

  frame.address_numeric_count = CountNumericCodes(frame.address);
  frame.self_numeric_count = CountNumericCodes(frame.self_id);

  const size_t frame_end =
      frame.eos_found ? std::min(frame.eos_index + 1, symbols.size())
                      : std::min(symbols.size(), start_index + static_cast<size_t>(32));
  size_t fec_ok_count = 0;
  for (size_t i = start_index; i < frame_end; ++i) {
    if (symbols[i].fec_ok) {
      ++fec_ok_count;
    }
  }
  frame.frame_fec_ok_ratio = frame_end > start_index
                                 ? static_cast<double>(fec_ok_count) /
                                       static_cast<double>(frame_end - start_index)
                                 : 0.0;

  if (frame.ecc_present) {
    uint8_t ecc_value = 0;
    for (size_t i = start_index; i < payload_end_exclusive; ++i) {
      ecc_value = static_cast<uint8_t>(ecc_value ^ (symbols[i].value & 0x7FU));
    }
    frame.ecc_ok = ((ecc_value & 0x7FU) == (frame.ecc & 0x7FU));
  }

  double score = 0.0;
  score += 0.30 * frame.frame_fec_ok_ratio;
  score += 0.20 * (static_cast<double>(frame.address_numeric_count) / 5.0);
  score += 0.20 * (static_cast<double>(frame.self_numeric_count) / 5.0);
  if (frame.eos_found) {
    score += 0.15;
  }
  if (frame.ecc_present && frame.ecc_ok) {
    score += 0.10;
  }
  if (frame.payload.size() >= 2) {
    score += 0.03;
  }
  if (frame.payload.size() >= 4) {
    score += 0.02;
  }
  frame.score = std::clamp(score, 0.0, 1.0);

  return frame;
}

ParsedDscFrame ParseBestFrame(const std::vector<SymbolWord>& symbols) {
  ParsedDscFrame best;
  double best_score = -1.0;
  if (symbols.size() < 14) {
    return best;
  }

  const size_t max_start = std::min(static_cast<size_t>(20), symbols.size() - 14);
  for (size_t start = 0; start <= max_start; ++start) {
    ParsedDscFrame candidate = TryParseFrame(symbols, start);
    if (!candidate.parsed) {
      continue;
    }
    if (candidate.score > best_score) {
      best_score = candidate.score;
      best = std::move(candidate);
    }
  }

  return best;
}

std::string FrameValidityLabel(const ParsedDscFrame& frame, double symbol_fec_ratio) {
  if (!frame.parsed) {
    return "candidate";
  }
  if (frame.eos_found && frame.address_numeric_count >= 4 && frame.self_numeric_count >= 4 &&
      symbol_fec_ratio >= 0.75 && (!frame.ecc_present || frame.ecc_ok)) {
    return "valid";
  }
  if (frame.eos_found && symbol_fec_ratio >= 0.55) {
    return "partial";
  }
  return "candidate";
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

int Init(const char* /*config_json*/) {
  if (!g_initialized.exchange(true, std::memory_order_relaxed)) {
    g_sequence = 0;
    g_recent_frames.clear();
    g_metric_blocks.store(0, std::memory_order_relaxed);
    g_metric_candidates.store(0, std::memory_order_relaxed);
    g_metric_duplicates.store(0, std::memory_order_relaxed);
    g_metric_emitted.store(0, std::memory_order_relaxed);
    g_metric_frames_parsed.store(0, std::memory_order_relaxed);
    g_metric_ecc_ok.store(0, std::memory_order_relaxed);
    g_metric_ecc_fail.store(0, std::memory_order_relaxed);
  }
  return 0;
}

int ProcessIq(const multi_radio_iq_view* iq_view, multi_radio_emit_message_fn emit_fn, void* user_data) {
  if (iq_view == nullptr || emit_fn == nullptr) {
    return -1;
  }
  if (iq_view->sample_rate_hz == 0 || iq_view->sample_count < 128) {
    return 0;
  }
  const uint32_t tuned_frequency_hz = iq_view->center_frequency_hz;

  g_metric_blocks.fetch_add(1, std::memory_order_relaxed);

  const std::vector<double> discriminator = BuildFmDiscriminator(iq_view);
  if (discriminator.size() < kMinDiscriminatorSamples) {
    return 0;
  }

  const std::vector<double> audio =
      ResampleLinear(discriminator, static_cast<double>(iq_view->sample_rate_hz), kDscDecodeSampleRate);
  if (audio.size() < static_cast<size_t>(kDscSamplesPerSymbol * kMinSymbolsForDecode)) {
    return 0;
  }

  const BitStreamCandidate bits = DecodeDscBits(audio, kDscDecodeSampleRate);
  if (bits.bits.size() < kMinSymbolsForDecode) {
    return 0;
  }

  const size_t word_alignment = SelectWordAlignment(bits.bits);
  const std::vector<uint16_t> words = PackTenBitWords(bits.bits, word_alignment, kMaxWordsForEmit);
  if (words.size() < kMinWordsForEmit) {
    return 0;
  }

  const SymbolDecodeResult symbol_decode = DecodeSymbols(words);
  const ParsedDscFrame frame = ParseBestFrame(symbol_decode.symbols);

  const double base_confidence = std::clamp((bits.avg_abs_contrast - 0.08) / 0.35, 0.0, 1.0);
  const double confidence = std::clamp(0.45 * base_confidence + 0.30 * symbol_decode.fec_ok_ratio +
                                           0.25 * frame.score,
                                       0.0, 1.0);
  if (confidence < kMinConfidence) {
    return 0;
  }

  g_metric_candidates.fetch_add(1, std::memory_order_relaxed);

  const std::string words_hex = FormatWordsHex(words);
  std::ostringstream dedup_key_builder;
  dedup_key_builder << words_hex << "|phase=" << bits.bit_phase << "|align=" << word_alignment
                    << "|layout=" << LayoutName(symbol_decode.layout)
                    << "|rule=" << FecRuleName(symbol_decode.rule)
                    << "|start=" << frame.start_index
                    << "|f=" << tuned_frequency_hz;
  if (IsDuplicateFrame(Fnv1a64(dedup_key_builder.str()))) {
    return 0;
  }

  if (frame.parsed) {
    g_metric_frames_parsed.fetch_add(1, std::memory_order_relaxed);
    if (frame.ecc_present) {
      if (frame.ecc_ok) {
        g_metric_ecc_ok.fetch_add(1, std::memory_order_relaxed);
      } else {
        g_metric_ecc_fail.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  const uint64_t emitted_now = g_metric_emitted.fetch_add(1, std::memory_order_relaxed) + 1;

  const std::string validity = FrameValidityLabel(frame, symbol_decode.fec_ok_ratio);
  const std::string format_label = frame.parsed ? FormatCodeLabel("FS", frame.format) : "FS_UNKNOWN";
  const std::string category_label = frame.parsed ? FormatCodeLabel("CAT", frame.category) : "CAT_UNKNOWN";
  const std::string address_digits = frame.parsed ? EncodeTwoDigitCodes(frame.address) : std::string();
  const std::string self_digits = frame.parsed ? EncodeTwoDigitCodes(frame.self_id) : std::string();
  const std::string address_codes = frame.parsed ? FormatCodeList(frame.address) : std::string();
  const std::string self_codes = frame.parsed ? FormatCodeList(frame.self_id) : std::string();
  const std::string payload_codes = frame.parsed ? FormatCodeList(frame.payload) : std::string();

  std::ostringstream payload;
  if (frame.parsed) {
    payload << "[DSC_FRAME] f_hz=" << tuned_frequency_hz << " "
            << "type=" << format_label
            << " to=" << (address_digits.empty() ? "?" : address_digits)
            << " from=" << (self_digits.empty() ? "?" : self_digits)
            << " tc1=" << static_cast<unsigned int>(frame.telecommand1)
            << " tc2=" << static_cast<unsigned int>(frame.telecommand2)
            << " eos=" << (frame.eos_found ? EosLabel(frame.eos) : "MISSING")
            << " validity=" << validity << " conf=" << confidence;
  } else {
    payload << "[DSC_CANDIDATE] f_hz=" << tuned_frequency_hz << " words=" << words.size()
            << " conf=" << confidence;
  }

  std::ostringstream fields_json;
  fields_json << "{\"kind\":\"" << (frame.parsed ? "frame" : "candidate") << "\""
              << ",\"channel\":\"" << tuned_frequency_hz << "\""
              << ",\"rx_frequency_hz\":\"" << tuned_frequency_hz << "\""
              << ",\"decoder\":\"dsc_fsk_1200_experimental\""
              << ",\"symbol_rate\":\"" << kDscSymbolRate << "\""
              << ",\"decode_sample_rate\":\"" << kDscDecodeSampleRate << "\""
              << ",\"bit_phase\":\"" << bits.bit_phase << "\""
              << ",\"word_alignment\":\"" << word_alignment << "\""
              << ",\"symbols\":\"" << bits.bits.size() << "\""
              << ",\"transitions\":\"" << bits.transitions << "\""
              << ",\"confidence\":\"" << confidence << "\""
              << ",\"word_count\":\"" << words.size() << "\""
              << ",\"words_hex\":\"" << words_hex << "\""
              << ",\"symbol_layout\":\"" << LayoutName(symbol_decode.layout) << "\""
              << ",\"symbol_fec_rule\":\"" << FecRuleName(symbol_decode.rule) << "\""
              << ",\"symbol_fec_ok_ratio\":\"" << symbol_decode.fec_ok_ratio << "\""
              << ",\"validity\":\"" << validity << "\""
              << ",\"msg_type\":\"" << format_label << "\""
              << ",\"metric_blocks\":\"" << g_metric_blocks.load(std::memory_order_relaxed) << "\""
              << ",\"metric_candidates\":\"" << g_metric_candidates.load(std::memory_order_relaxed)
              << "\""
              << ",\"metric_duplicates\":\"" << g_metric_duplicates.load(std::memory_order_relaxed)
              << "\""
              << ",\"metric_emitted\":\"" << emitted_now << "\""
              << ",\"metric_frames_parsed\":\""
              << g_metric_frames_parsed.load(std::memory_order_relaxed) << "\""
              << ",\"metric_ecc_ok\":\"" << g_metric_ecc_ok.load(std::memory_order_relaxed) << "\""
              << ",\"metric_ecc_fail\":\"" << g_metric_ecc_fail.load(std::memory_order_relaxed)
              << "\"";

  if (frame.parsed) {
    fields_json << ",\"frame_start\":\"" << frame.start_index << "\""
                << ",\"frame_fec_ok_ratio\":\"" << frame.frame_fec_ok_ratio << "\""
                << ",\"format_code\":\"" << static_cast<unsigned int>(frame.format) << "\""
                << ",\"format_label\":\"" << format_label << "\""
                << ",\"address_codes\":\"" << address_codes << "\""
                << ",\"address_digits\":\"" << address_digits << "\""
                << ",\"category_code\":\"" << static_cast<unsigned int>(frame.category) << "\""
                << ",\"category_label\":\"" << category_label << "\""
                << ",\"self_id_codes\":\"" << self_codes << "\""
                << ",\"self_id_digits\":\"" << self_digits << "\""
                << ",\"telecommand_1\":\"" << static_cast<unsigned int>(frame.telecommand1) << "\""
                << ",\"telecommand_2\":\"" << static_cast<unsigned int>(frame.telecommand2) << "\""
                << ",\"payload_codes\":\"" << payload_codes << "\""
                << ",\"eos_found\":\"" << (frame.eos_found ? 1 : 0) << "\""
                << ",\"eos_code\":\"" << static_cast<unsigned int>(frame.eos) << "\""
                << ",\"eos_label\":\"" << (frame.eos_found ? EosLabel(frame.eos) : "MISSING") << "\""
                << ",\"ecc_present\":\"" << (frame.ecc_present ? 1 : 0) << "\""
                << ",\"ecc_code\":\"" << static_cast<unsigned int>(frame.ecc) << "\""
                << ",\"ecc_ok\":\"" << (frame.ecc_ok ? 1 : 0) << "\""
                << ",\"mmsi\":\"" << self_digits << "\"";
  }

  fields_json << "}";

  const std::string payload_text = payload.str();
  const std::string fields_text = fields_json.str();
  (void)user_data;
  emit_fn("DSC", payload_text.c_str(), static_cast<double>(tuned_frequency_hz), 0,
          fields_text.c_str(), user_data);
  return 0;
}

int Flush(multi_radio_emit_message_fn /*emit_fn*/, void* /*user_data*/) { return 0; }

void Shutdown() {
  g_recent_frames.clear();
  g_sequence = 0;
  g_metric_blocks.store(0, std::memory_order_relaxed);
  g_metric_candidates.store(0, std::memory_order_relaxed);
  g_metric_duplicates.store(0, std::memory_order_relaxed);
  g_metric_emitted.store(0, std::memory_order_relaxed);
  g_metric_frames_parsed.store(0, std::memory_order_relaxed);
  g_metric_ecc_ok.store(0, std::memory_order_relaxed);
  g_metric_ecc_fail.store(0, std::memory_order_relaxed);
  g_initialized.store(false, std::memory_order_relaxed);
}

const multi_radio_plugin_descriptor kDescriptor = {
    .plugin_name = "dsc_wrapper",
    .plugin_version = "0.4.1",
    .api_version = MULTI_RADIO_PLUGIN_API_VERSION,
    .supported_signals_csv = "DSC",
    .init = &Init,
    .process_iq = &ProcessIq,
    .flush = &Flush,
    .shutdown = &Shutdown,
};

}  // namespace

extern "C" const multi_radio_plugin_descriptor* multi_radio_get_plugin_descriptor() {
  return &kDescriptor;
}
