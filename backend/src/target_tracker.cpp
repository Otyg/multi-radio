#include "multi_radio/target_tracker.hpp"

#include <cmath>
#include <cstdlib>
#include <chrono>

namespace multi_radio {

namespace {

uint64_t NowMs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
}

}  // namespace

TargetTracker::TargetTracker(uint64_t air_stale_ms, uint64_t sea_stale_ms)
    : air_stale_ms_(air_stale_ms), sea_stale_ms_(sea_stale_ms) {}

double TargetTracker::ParseDouble(const std::string& s) {
  if (s.empty()) return std::numeric_limits<double>::quiet_NaN();
  char* end = nullptr;
  double v = std::strtod(s.c_str(), &end);
  return (end != s.c_str()) ? v : std::numeric_limits<double>::quiet_NaN();
}

bool TargetTracker::HasField(const DecodedMessage& msg, const char* key) {
  return msg.normalized_fields.count(key) != 0;
}

double TargetTracker::Field(const DecodedMessage& msg, const char* key) {
  auto it = msg.normalized_fields.find(key);
  return it != msg.normalized_fields.end() ? ParseDouble(it->second)
                                           : std::numeric_limits<double>::quiet_NaN();
}

std::string TargetTracker::FieldStr(const DecodedMessage& msg, const char* key) {
  auto it = msg.normalized_fields.find(key);
  return it != msg.normalized_fields.end() ? it->second : std::string{};
}

void TargetTracker::Update(const DecodedMessage& msg) {
  // Determine key and kind from signal type / fields.
  std::string key;
  std::string kind;
  if (msg.signal_type == SignalType::kAdsb) {
    key  = FieldStr(msg, "icao");
    kind = "AIR";
  } else if (msg.signal_type == SignalType::kAis) {
    key  = FieldStr(msg, "mmsi");
    kind = "SEA";
  }
  if (key.empty()) return;

  std::lock_guard<std::mutex> lock(mu_);
  auto& e = entries_[key];
  e.id   = key;
  e.kind = kind;
  e.last_seen_ms = msg.unix_ms != 0 ? msg.unix_ms : NowMs();

  // Label: prefer callsign/name; keep existing if message has none.
  const std::string cs   = FieldStr(msg, "callsign");
  const std::string name = FieldStr(msg, "name");
  const std::string call = FieldStr(msg, "call_sign");
  if (!name.empty())  e.label = name;
  else if (!cs.empty())   e.label = cs;
  else if (!call.empty()) e.label = call;
  if (e.label.empty()) e.label = key;

  // Position (only overwrite when both lat and lon are present and finite).
  const double lat = Field(msg, "lat");
  const double lon = Field(msg, "lon");
  if (std::isfinite(lat) && std::isfinite(lon)) {
    e.lat = lat;
    e.lon = lon;
  }

  // Speed / course.
  const double gs  = Field(msg, "gs");
  const double sog = Field(msg, "sog");
  if (std::isfinite(gs)  && gs  > 0.0) e.sog_knots   = gs;
  if (std::isfinite(sog) && sog > 0.0) e.sog_knots   = sog;

  const double heading = Field(msg, "heading");
  const double cog     = Field(msg, "cog");
  if (std::isfinite(heading)) e.cog_degrees = heading;
  if (std::isfinite(cog))     e.cog_degrees = cog;

  // Altitude.
  const double alt_baro = Field(msg, "alt_baro");
  const double alt_geom = Field(msg, "alt_geom");
  if (std::isfinite(alt_baro)) { e.has_altitude = true; e.altitude_ft = alt_baro; }
  else if (std::isfinite(alt_geom)) { e.has_altitude = true; e.altitude_ft = alt_geom; }
}

RadarSnapshot TargetTracker::TakeSnapshot() {
  const uint64_t now = NowMs();
  RadarSnapshot snap;
  snap.snapshot_ms = now;

  std::lock_guard<std::mutex> lock(mu_);

  // Stale removal — use kind-specific window.
  for (auto it = entries_.begin(); it != entries_.end();) {
    const auto& e = it->second;
    const uint64_t threshold = (e.kind == "SEA") ? sea_stale_ms_ : air_stale_ms_;
    const uint64_t age = (now > e.last_seen_ms) ? now - e.last_seen_ms : 0u;
    if (threshold > 0 && age > threshold) {
      pending_removed_.push_back(it->first);
      it = entries_.erase(it);
    } else {
      ++it;
    }
  }

  // Only include targets that have a valid position.
  snap.targets.reserve(entries_.size());
  for (const auto& [_, e] : entries_) {
    if (!std::isfinite(e.lat) || !std::isfinite(e.lon)) continue;
    snap.targets.push_back(e);
  }

  snap.removed_ids = std::move(pending_removed_);
  pending_removed_.clear();
  return snap;
}

}  // namespace multi_radio
