#pragma once

#include <filesystem>
#include <mutex>
#include <string>
#include <unordered_map>

namespace multi_radio {

// Thread-safe key→name store persisted as a flat JSON object.
// Keys are plain identifiers (ICAO hex, MMSI digits, etc.).
// Names are short human-readable strings (callsigns, vessel names).
class NameDatabase {
 public:
  explicit NameDatabase(std::filesystem::path path);
  ~NameDatabase();

  // Learn (or update) a key→name mapping and persist immediately.
  void Learn(const std::string& key, const std::string& name);

  // Return the stored name, or "" if unknown.
  std::string Lookup(const std::string& key) const;

  size_t Size() const;

 private:
  void Load();
  void Save() const;

  mutable std::mutex mu_;
  std::filesystem::path path_;
  std::unordered_map<std::string, std::string> db_;
};

}  // namespace multi_radio
