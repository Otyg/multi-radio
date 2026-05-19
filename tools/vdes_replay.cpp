#include "multi_radio/plugin_host.hpp"
#include "multi_radio/types.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Args {
  std::filesystem::path iq_path;
  std::filesystem::path plugin_dir = "build/plugins";
  std::string demod = "vdes_phy_demod";
  std::string decoder;
  std::string postproc;
  uint32_t sample_rate_hz = 0;
  uint32_t block_pairs = 16384;
  std::optional<uint32_t> diag_interval_blocks;
  std::optional<double> squelch_db;
  double frequency_hz = 0.0;
  bool jsonl = false;
};

void Usage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " --iq FILE.iq16 --rate HZ --freq HZ [options]\n"
      << "\n"
      << "Options:\n"
      << "  --plugin-dir DIR   plugin directory (default: build/plugins)\n"
      << "  --demod NAME       demod plugin (default: vdes_phy_demod)\n"
      << "  --decoder NAME     optional decoder plugin\n"
      << "  --postproc NAME    optional postprocessor plugin\n"
      << "  --block-pairs N    IQ pairs per replay block (default: 16384)\n"
      << "  --diag-blocks N    set diag_interval_blocks on plugins\n"
      << "  --squelch-db DB    set squelch_db on plugins\n"
      << "  --jsonl            print every plugin message as one JSON object\n";
}

bool ParseU32(const std::string& s, uint32_t* out) {
  if (!out) return false;
  char* end = nullptr;
  const unsigned long v = std::strtoul(s.c_str(), &end, 10);
  if (end == s.c_str() || *end != '\0') return false;
  *out = static_cast<uint32_t>(std::min<unsigned long>(v, UINT32_MAX));
  return true;
}

bool ParseDouble(const std::string& s, double* out) {
  if (!out) return false;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') return false;
  *out = v;
  return true;
}

bool ParseArgs(int argc, char** argv, Args* args) {
  if (!args) return false;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto need_value = [&](std::string* out) -> bool {
      if (i + 1 >= argc) return false;
      *out = argv[++i];
      return true;
    };

    std::string value;
    if (key == "--iq" && need_value(&value)) args->iq_path = value;
    else if (key == "--plugin-dir" && need_value(&value)) args->plugin_dir = value;
    else if (key == "--demod" && need_value(&value)) args->demod = value;
    else if (key == "--decoder" && need_value(&value)) args->decoder = value;
    else if (key == "--postproc" && need_value(&value)) args->postproc = value;
    else if (key == "--rate" && need_value(&value)) {
      if (!ParseU32(value, &args->sample_rate_hz)) return false;
    } else if (key == "--freq" && need_value(&value)) {
      if (!ParseDouble(value, &args->frequency_hz)) return false;
    } else if (key == "--block-pairs" && need_value(&value)) {
      if (!ParseU32(value, &args->block_pairs)) return false;
    } else if (key == "--diag-blocks" && need_value(&value)) {
      uint32_t parsed = 0;
      if (!ParseU32(value, &parsed)) return false;
      args->diag_interval_blocks = parsed;
    } else if (key == "--squelch-db" && need_value(&value)) {
      double parsed = 0.0;
      if (!ParseDouble(value, &parsed)) return false;
      args->squelch_db = parsed;
    } else if (key == "--jsonl") {
      args->jsonl = true;
    } else if (key == "--help" || key == "-h") {
      return false;
    } else {
      std::cerr << "Unknown or incomplete argument: " << key << "\n";
      return false;
    }
  }

  return !args->iq_path.empty() && args->sample_rate_hz > 0 && args->frequency_hz > 0.0 &&
         args->block_pairs > 0;
}

std::string JsonEscape(const std::string& in) {
  std::string out;
  out.reserve(in.size() + 8);
  for (char c : in) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

void PrintMessageJson(const multi_radio::PluginMessage& msg) {
  std::cout << "{\"signal_type\":\""
            << JsonEscape(msg.normalized_fields.count("signal_type")
                              ? msg.normalized_fields.at("signal_type")
                              : "UNKNOWN")
            << "\",\"frequency_hz\":" << msg.frequency_hz
            << ",\"unix_ms\":" << msg.unix_ms
            << ",\"payload\":\"" << JsonEscape(msg.payload) << "\""
            << ",\"fields\":{";
  bool first = true;
  for (const auto& [k, v] : msg.normalized_fields) {
    if (!first) std::cout << ",";
    first = false;
    std::cout << "\"" << JsonEscape(k) << "\":\"" << JsonEscape(v) << "\"";
  }
  std::cout << "}}\n";
}

uint64_t UnixMsForBlock(uint64_t start_ms, uint64_t sample_index, uint32_t sample_rate_hz) {
  return start_ms + (sample_index * 1000ull) / static_cast<uint64_t>(sample_rate_hz);
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!ParseArgs(argc, argv, &args)) {
    Usage(argv[0]);
    return 2;
  }

  std::ifstream in(args.iq_path, std::ios::binary);
  if (!in) {
    std::cerr << "Cannot open IQ file: " << args.iq_path << "\n";
    return 1;
  }

  multi_radio::PluginHost plugins(args.plugin_dir);
  std::string error;
  if (!plugins.LoadAll(&error)) {
    std::cerr << "Plugin load failed: " << error << "\n";
    return 1;
  }
  plugins.SetActiveDemodulator(args.demod);
  plugins.SetActiveDecoder(args.decoder);
  plugins.SetActivePostprocessor(args.postproc);
  plugins.SetActiveAsmPostprocessor("");
  if (args.diag_interval_blocks.has_value()) {
    plugins.SetParam("diag_interval_blocks", std::to_string(*args.diag_interval_blocks));
  }
  if (args.squelch_db.has_value()) {
    std::ostringstream s;
    s << *args.squelch_db;
    plugins.SetParam("squelch_db", s.str());
  }

  std::vector<int16_t> buf(static_cast<size_t>(args.block_pairs) * 2u);
  uint64_t blocks = 0;
  uint64_t iq_pairs = 0;
  uint64_t messages = 0;
  std::map<std::string, uint64_t> messages_by_type;
  const uint64_t start_ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());

  while (in) {
    in.read(reinterpret_cast<char*>(buf.data()),
            static_cast<std::streamsize>(buf.size() * sizeof(int16_t)));
    const std::streamsize bytes = in.gcount();
    if (bytes <= 0) break;
    const size_t components = static_cast<size_t>(bytes) / sizeof(int16_t);
    const size_t pairs = components / 2u;
    if (pairs == 0) break;

    multi_radio::IQSampleBlock block;
    block.sample_rate_hz = args.sample_rate_hz;
    block.center_frequency_hz = static_cast<uint32_t>(args.frequency_hz + 0.5);
    block.interleaved_iq.assign(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(pairs * 2u));

    plugins.ProcessIq(block, [&](const multi_radio::PluginMessage& msg) {
      messages++;
      const auto it = msg.normalized_fields.find("signal_type");
      const std::string type = it == msg.normalized_fields.end() ? "UNKNOWN" : it->second;
      messages_by_type[type]++;
      if (args.jsonl) PrintMessageJson(msg);
    });

    blocks++;
    iq_pairs += pairs;
  }

  std::cout << "Replay complete\n"
            << "  iq_file: " << args.iq_path << "\n"
            << "  plugin_dir: " << args.plugin_dir << "\n"
            << "  demod: " << args.demod << "\n"
            << "  decoder: " << (args.decoder.empty() ? "(none)" : args.decoder) << "\n"
            << "  postproc: " << (args.postproc.empty() ? "(none)" : args.postproc) << "\n"
            << "  sample_rate_hz: " << args.sample_rate_hz << "\n"
            << "  frequency_hz: " << args.frequency_hz << "\n"
            << "  blocks: " << blocks << "\n"
            << "  iq_pairs: " << iq_pairs << "\n"
            << "  duration_s: " << (static_cast<double>(iq_pairs) / args.sample_rate_hz) << "\n"
            << "  plugin_messages: " << messages << "\n";
  for (const auto& [type, count] : messages_by_type) {
    std::cout << "  " << type << ": " << count << "\n";
  }

  (void)UnixMsForBlock(start_ms, 0, args.sample_rate_hz);
  return 0;
}
