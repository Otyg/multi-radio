#pragma once

#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "multi_radio/types.hpp"

namespace multi_radio {

struct TrackedTarget {
  std::string id;
  std::string label;
  std::string kind;   // "AIR", "SEA", "?"
  double lat          = std::numeric_limits<double>::quiet_NaN();
  double lon          = std::numeric_limits<double>::quiet_NaN();
  double sog_knots    = 0.0;
  double cog_degrees  = 0.0;
  bool   has_altitude = false;
  double altitude_ft  = 0.0;
  uint64_t last_seen_ms = 0;
};

struct RadarSnapshot {
  std::vector<TrackedTarget> targets;
  std::vector<std::string>  removed_ids;
  uint64_t snapshot_ms = 0;
};

// Merges per-frame decoded messages into one entry per ICAO/MMSI.
// Thread-safe; call Update() from any thread, TakeSnapshot() from the gRPC thread.
class TargetTracker {
 public:
  // Separate stale windows per target type.
  // Aircraft stop transmitting immediately when they land; 60 s is safe.
  // Vessels use infrequent AIS intervals (anchored: every 3–6 min); 10 min avoids blinking.
  explicit TargetTracker(uint64_t air_stale_ms = 60000,
                         uint64_t sea_stale_ms = 600000);

  // Merge a decoded message into the tracked state.
  void Update(const DecodedMessage& msg);

  // Return full current state and the list of IDs removed since the last call.
  // Also removes stale entries.
  RadarSnapshot TakeSnapshot();

 private:
  static double ParseDouble(const std::string& s);
  static bool   HasField(const DecodedMessage& msg, const char* key);
  static double Field(const DecodedMessage& msg, const char* key);
  static std::string FieldStr(const DecodedMessage& msg, const char* key);

  mutable std::mutex mu_;
  uint64_t air_stale_ms_;
  uint64_t sea_stale_ms_;
  std::unordered_map<std::string, TrackedTarget> entries_;
  std::vector<std::string> pending_removed_;
};

}  // namespace multi_radio
