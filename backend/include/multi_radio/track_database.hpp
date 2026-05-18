#pragma once

#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

struct sqlite3;
struct sqlite3_stmt;

namespace multi_radio {

struct TrackPoint {
  uint64_t ts_ms      = 0;
  double   lat        = 0.0;
  double   lon        = 0.0;
  std::optional<double> altitude_ft;
  std::optional<double> sog_knots;
  std::optional<double> cog_degrees;
  std::string source;            // "AIS" | "ADSB"
};

struct EntityInfo {
  std::string id;
  std::string kind;              // "AIR" | "SEA"
  std::string name;
  std::string callsign;
  uint64_t    first_seen_ms = 0;
  uint64_t    last_seen_ms  = 0;
};

// SQLite-backed store for:
//   - entity identity (ICAO/MMSI → name/callsign) — replaces NameDatabase
//   - delta-compressed position tracks
//   - optional raw decoded frames
//
// Thread-safe. Uses WAL mode for concurrent reads during writes.
class TrackDatabase {
 public:
  explicit TrackDatabase(std::filesystem::path db_path);
  ~TrackDatabase();

  TrackDatabase(const TrackDatabase&)            = delete;
  TrackDatabase& operator=(const TrackDatabase&) = delete;

  // --- Entity management (replaces NameDatabase) ---

  // Learn or update a mapping. name/callsign may be empty if not yet known.
  void UpsertEntity(const std::string& id, const std::string& kind,
                    const std::string& name, const std::string& callsign,
                    uint64_t ts_ms);

  // Fast name lookup using in-memory cache. Returns "" if unknown.
  std::string LookupName(const std::string& id) const;

  // --- Track recording ---

  // Record a position. Internally delta-compressed: only writes to DB when
  // the position has changed enough or enough time has passed.
  void RecordPosition(const std::string& entity_id, uint64_t ts_ms,
                      double lat, double lon,
                      std::optional<double> altitude_ft,
                      std::optional<double> sog_knots,
                      std::optional<double> cog_degrees,
                      const std::string& source);

  // --- Raw frame storage (optional) ---
  void RecordRawFrame(uint64_t ts_ms, uint32_t receiver_id,
                      const std::string& source, double frequency_hz,
                      const std::string& payload,
                      const std::string& fields_json);

  // --- Queries ---
  std::vector<TrackPoint> QueryTracks(const std::string& entity_id,
                                      uint64_t from_ms, uint64_t to_ms) const;
  std::vector<EntityInfo> QueryEntities(const std::string& kind,
                                        uint64_t active_since_ms) const;

  // --- Maintenance ---
  // Remove tracks older than tracks_max_age_ms and raw frames older than
  // raw_max_age_ms. Pass 0 to skip that category.
  void Prune(uint64_t tracks_max_age_ms, uint64_t raw_max_age_ms);

  // Remove all learned ICAO/MMSI → name/callsign mappings.
  void ClearEntities();

 private:
  void InitSchema();
  void PrepareStatements();
  void FinalizeStatements();

  // Returns true if this position is sufficiently different from the last
  // recorded position for this entity to warrant a new row.
  bool ShouldRecord(const std::string& entity_id, uint64_t ts_ms,
                    double lat, double lon,
                    std::optional<double> sog_knots) const;

  mutable std::mutex mu_;
  sqlite3*      db_ = nullptr;

  // Prepared statements (owned, finalized in destructor).
  sqlite3_stmt* stmt_upsert_entity_  = nullptr;
  sqlite3_stmt* stmt_lookup_name_    = nullptr;
  sqlite3_stmt* stmt_insert_track_   = nullptr;
  sqlite3_stmt* stmt_insert_raw_     = nullptr;

  // In-memory name cache for fast per-message lookups.
  std::unordered_map<std::string, std::string> name_cache_;

  // Last recorded position per entity for delta compression.
  struct LastPos {
    double   lat   = 0.0;
    double   lon   = 0.0;
    double   sog   = 0.0;
    uint64_t ts_ms = 0;
  };
  std::unordered_map<std::string, LastPos> last_pos_;
};

}  // namespace multi_radio
