#pragma once

#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

#include "multi_radio/types.hpp"

namespace multi_radio {

class JsonlLogger {
 public:
  JsonlLogger(std::filesystem::path log_dir, std::string base_name, size_t max_bytes,
              size_t max_files);

  void LogEvent(const ReceiverEvent& event);
  void LogDecodedMessage(const DecodedMessage& message);

 private:
  void EnsureOpen();
  void RotateIfNeeded();
  std::string EventToJson(const ReceiverEvent& event) const;
  std::string MessageToJson(const DecodedMessage& message) const;

  std::filesystem::path log_dir_;
  std::string base_name_;
  size_t max_bytes_;
  size_t max_files_;

  std::mutex mu_;
  std::ofstream stream_;
  std::filesystem::path current_path_;
};

}  // namespace multi_radio
