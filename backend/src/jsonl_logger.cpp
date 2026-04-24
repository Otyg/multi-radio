#include "multi_radio/jsonl_logger.hpp"

#include <sstream>

namespace multi_radio {

namespace {
std::string EscapeJson(std::string value) {
  std::string out;
  out.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}
}  // namespace

JsonlLogger::JsonlLogger(std::filesystem::path log_dir, std::string base_name, size_t max_bytes,
                         size_t max_files)
    : log_dir_(std::move(log_dir)),
      base_name_(std::move(base_name)),
      max_bytes_(max_bytes),
      max_files_(max_files) {
  std::filesystem::create_directories(log_dir_);
  current_path_ = log_dir_ / (base_name_ + ".jsonl");
}

void JsonlLogger::LogEvent(const ReceiverEvent& event) {
  std::lock_guard<std::mutex> lock(mu_);
  EnsureOpen();
  stream_ << EventToJson(event) << "\n";
  stream_.flush();
  RotateIfNeeded();
}

void JsonlLogger::LogDecodedMessage(const DecodedMessage& message) {
  std::lock_guard<std::mutex> lock(mu_);
  EnsureOpen();
  stream_ << MessageToJson(message) << "\n";
  stream_.flush();
  RotateIfNeeded();
}

void JsonlLogger::EnsureOpen() {
  if (stream_.is_open()) {
    return;
  }
  stream_.open(current_path_, std::ios::app);
}

void JsonlLogger::RotateIfNeeded() {
  if (!stream_.is_open()) {
    return;
  }

  if (static_cast<size_t>(stream_.tellp()) < max_bytes_) {
    return;
  }

  stream_.close();

  for (size_t i = max_files_; i > 0; --i) {
    const auto older = log_dir_ / (base_name_ + "." + std::to_string(i - 1) + ".jsonl");
    const auto newer = log_dir_ / (base_name_ + "." + std::to_string(i) + ".jsonl");
    if (i == max_files_) {
      std::error_code ec;
      std::filesystem::remove(newer, ec);
    }
    if (std::filesystem::exists(older)) {
      std::filesystem::rename(older, newer);
    }
  }

  if (std::filesystem::exists(current_path_)) {
    std::filesystem::rename(current_path_, log_dir_ / (base_name_ + ".0.jsonl"));
  }

  stream_.open(current_path_, std::ios::trunc);
}

std::string JsonlLogger::EventToJson(const ReceiverEvent& event) const {
  std::ostringstream ss;
  ss << "{"
     << "\"type\":\"event\"," << "\"unix_ms\":" << event.unix_ms << ","
     << "\"receiver_id\":" << event.receiver_id << ","
     << "\"kind\":\"" << EscapeJson(std::to_string(static_cast<int>(event.kind))) << "\"," 
     << "\"message\":\"" << EscapeJson(event.message) << "\"," 
     << "\"tuned_frequency_hz\":" << event.tuned_frequency_hz << "}";
  return ss.str();
}

std::string JsonlLogger::MessageToJson(const DecodedMessage& message) const {
  std::ostringstream ss;
  ss << "{"
     << "\"type\":\"decoded\"," << "\"unix_ms\":" << message.unix_ms << ","
     << "\"receiver_id\":" << message.receiver_id << ","
     << "\"signal_type\":\"" << EscapeJson(ToString(message.signal_type)) << "\"," 
     << "\"frequency_hz\":" << message.frequency_hz << ","
     << "\"payload\":\"" << EscapeJson(message.payload) << "\"," 
     << "\"normalized_fields\":{";

  bool first = true;
  for (const auto& [key, value] : message.normalized_fields) {
    if (!first) {
      ss << ",";
    }
    first = false;
    ss << "\"" << EscapeJson(key) << "\":\"" << EscapeJson(value) << "\"";
  }
  ss << "}}";
  return ss.str();
}

}  // namespace multi_radio
