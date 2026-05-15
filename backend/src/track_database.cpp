#include "multi_radio/track_database.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include <sqlite3.h>

namespace multi_radio {

namespace {

// ----------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------

static void SqliteCheck(int rc, const char* ctx) {
  if (rc != SQLITE_OK && rc != SQLITE_ROW && rc != SQLITE_DONE) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "SQLite error in %s: %s (%d)", ctx,
                  sqlite3_errstr(rc), rc);
    throw std::runtime_error(buf);
  }
}

static void Exec(sqlite3* db, const char* sql) {
  char* err = nullptr;
  const int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  if (rc != SQLITE_OK) {
    std::string msg = err ? std::string(err) : std::string(sqlite3_errstr(rc));
    sqlite3_free(err);
    throw std::runtime_error("SQLite exec failed: " + msg);
  }
}

// Haversine-based distance in meters between two lat/lon points.
static double DistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kR = 6371000.0;
  const double phi1 = lat1 * M_PI / 180.0;
  const double phi2 = lat2 * M_PI / 180.0;
  const double dphi = (lat2 - lat1) * M_PI / 180.0;
  const double dlam = (lon2 - lon1) * M_PI / 180.0;
  const double a = std::sin(dphi / 2) * std::sin(dphi / 2) +
                   std::cos(phi1) * std::cos(phi2) *
                   std::sin(dlam / 2) * std::sin(dlam / 2);
  return kR * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

// Thresholds for delta compression.
constexpr double   kMinDistanceM    = 100.0;   // record if moved > 100 m
constexpr double   kMinSogChangekn  = 2.0;     // record if speed changed > 2 kn
constexpr uint64_t kMaxIntervalMs   = 60000;   // record at least every 60 s

}  // namespace

// ----------------------------------------------------------------
// Construction / destruction
// ----------------------------------------------------------------

TrackDatabase::TrackDatabase(std::filesystem::path db_path) {
  std::filesystem::create_directories(db_path.parent_path());

  const int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE |
                    SQLITE_OPEN_FULLMUTEX;
  const int rc = sqlite3_open_v2(db_path.string().c_str(), &db_, flags, nullptr);
  if (rc != SQLITE_OK || db_ == nullptr) {
    throw std::runtime_error("Failed to open TrackDatabase: " +
                             db_path.string());
  }

  Exec(db_, "PRAGMA journal_mode=WAL");
  Exec(db_, "PRAGMA synchronous=NORMAL");
  Exec(db_, "PRAGMA foreign_keys=ON");

  InitSchema();
  PrepareStatements();

  // Warm the name cache from DB.
  sqlite3_stmt* s = nullptr;
  SqliteCheck(sqlite3_prepare_v2(db_,
      "SELECT id, COALESCE(NULLIF(callsign,''), NULLIF(name,''), '') FROM entities",
      -1, &s, nullptr), "warm name cache");
  while (sqlite3_step(s) == SQLITE_ROW) {
    const char* id   = reinterpret_cast<const char*>(sqlite3_column_text(s, 0));
    const char* name = reinterpret_cast<const char*>(sqlite3_column_text(s, 1));
    if (id && name && name[0])
      name_cache_[id] = name;
  }
  sqlite3_finalize(s);
}

TrackDatabase::~TrackDatabase() {
  FinalizeStatements();
  if (db_) sqlite3_close(db_);
}

// ----------------------------------------------------------------
// Schema
// ----------------------------------------------------------------

void TrackDatabase::InitSchema() {
  Exec(db_, R"sql(
    CREATE TABLE IF NOT EXISTS entities (
      id          TEXT    PRIMARY KEY,
      kind        TEXT    NOT NULL DEFAULT '',
      name        TEXT    NOT NULL DEFAULT '',
      callsign    TEXT    NOT NULL DEFAULT '',
      first_seen  INTEGER NOT NULL DEFAULT 0,
      last_seen   INTEGER NOT NULL DEFAULT 0
    );

    CREATE TABLE IF NOT EXISTS tracks (
      id          INTEGER PRIMARY KEY AUTOINCREMENT,
      entity_id   TEXT    NOT NULL REFERENCES entities(id),
      ts          INTEGER NOT NULL,
      lat         REAL    NOT NULL,
      lon         REAL    NOT NULL,
      altitude_ft REAL,
      sog_knots   REAL,
      cog_degrees REAL,
      source      TEXT    NOT NULL DEFAULT ''
    );
    CREATE INDEX IF NOT EXISTS tracks_entity_ts ON tracks (entity_id, ts);
    CREATE INDEX IF NOT EXISTS tracks_ts        ON tracks (ts);

    CREATE TABLE IF NOT EXISTS raw_frames (
      id           INTEGER PRIMARY KEY AUTOINCREMENT,
      ts           INTEGER NOT NULL,
      receiver_id  INTEGER,
      source       TEXT    NOT NULL DEFAULT '',
      frequency_hz REAL,
      payload      TEXT,
      fields_json  TEXT
    );
    CREATE INDEX IF NOT EXISTS raw_frames_ts        ON raw_frames (ts);
    CREATE INDEX IF NOT EXISTS raw_frames_source_ts ON raw_frames (source, ts);
  )sql");
}

// ----------------------------------------------------------------
// Prepared statements
// ----------------------------------------------------------------

void TrackDatabase::PrepareStatements() {
  SqliteCheck(sqlite3_prepare_v2(db_, R"sql(
    INSERT INTO entities (id, kind, name, callsign, first_seen, last_seen)
    VALUES (?, ?, ?, ?, ?, ?)
    ON CONFLICT(id) DO UPDATE SET
      kind      = CASE WHEN excluded.kind != ''     THEN excluded.kind     ELSE kind     END,
      name      = CASE WHEN excluded.name != ''     THEN excluded.name     ELSE name     END,
      callsign  = CASE WHEN excluded.callsign != '' THEN excluded.callsign ELSE callsign END,
      last_seen = excluded.last_seen
  )sql", -1, &stmt_upsert_entity_, nullptr), "prepare upsert_entity");

  SqliteCheck(sqlite3_prepare_v2(db_,
      "SELECT COALESCE(NULLIF(callsign,''), NULLIF(name,''), '') FROM entities WHERE id=?",
      -1, &stmt_lookup_name_, nullptr), "prepare lookup_name");

  SqliteCheck(sqlite3_prepare_v2(db_, R"sql(
    INSERT INTO tracks (entity_id, ts, lat, lon, altitude_ft, sog_knots, cog_degrees, source)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
  )sql", -1, &stmt_insert_track_, nullptr), "prepare insert_track");

  SqliteCheck(sqlite3_prepare_v2(db_, R"sql(
    INSERT INTO raw_frames (ts, receiver_id, source, frequency_hz, payload, fields_json)
    VALUES (?, ?, ?, ?, ?, ?)
  )sql", -1, &stmt_insert_raw_, nullptr), "prepare insert_raw");
}

void TrackDatabase::FinalizeStatements() {
  sqlite3_finalize(stmt_upsert_entity_);  stmt_upsert_entity_ = nullptr;
  sqlite3_finalize(stmt_lookup_name_);    stmt_lookup_name_   = nullptr;
  sqlite3_finalize(stmt_insert_track_);   stmt_insert_track_  = nullptr;
  sqlite3_finalize(stmt_insert_raw_);     stmt_insert_raw_    = nullptr;
}

// ----------------------------------------------------------------
// Entity management
// ----------------------------------------------------------------

void TrackDatabase::UpsertEntity(const std::string& id, const std::string& kind,
                                  const std::string& name, const std::string& callsign,
                                  uint64_t ts_ms) {
  if (id.empty()) return;
  std::lock_guard<std::mutex> lock(mu_);

  // Update in-memory name cache.
  const std::string& label = callsign.empty() ? name : callsign;
  if (!label.empty()) name_cache_[id] = label;

  auto* s = stmt_upsert_entity_;
  sqlite3_reset(s);
  sqlite3_bind_text(s, 1, id.c_str(),       -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(s, 2, kind.c_str(),     -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(s, 3, name.c_str(),     -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(s, 4, callsign.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(s, 5, static_cast<sqlite3_int64>(ts_ms));  // first_seen (ignored on UPDATE)
  sqlite3_bind_int64(s, 6, static_cast<sqlite3_int64>(ts_ms));  // last_seen
  sqlite3_step(s);
}

std::string TrackDatabase::LookupName(const std::string& id) const {
  if (id.empty()) return {};
  std::lock_guard<std::mutex> lock(mu_);
  const auto it = name_cache_.find(id);
  return it != name_cache_.end() ? it->second : std::string{};
}

// ----------------------------------------------------------------
// Track recording
// ----------------------------------------------------------------

bool TrackDatabase::ShouldRecord(const std::string& id, uint64_t ts_ms,
                                  double lat, double lon,
                                  std::optional<double> sog_knots) const {
  const auto it = last_pos_.find(id);
  if (it == last_pos_.end()) return true;  // first position seen

  const auto& prev = it->second;
  const uint64_t age = (ts_ms > prev.ts_ms) ? ts_ms - prev.ts_ms : 0u;
  if (age >= kMaxIntervalMs) return true;

  const double dist = DistanceMeters(prev.lat, prev.lon, lat, lon);
  if (dist >= kMinDistanceM) return true;

  if (sog_knots.has_value() &&
      std::abs(*sog_knots - prev.sog) >= kMinSogChangekn) return true;

  return false;
}

void TrackDatabase::RecordPosition(const std::string& entity_id, uint64_t ts_ms,
                                    double lat, double lon,
                                    std::optional<double> altitude_ft,
                                    std::optional<double> sog_knots,
                                    std::optional<double> cog_degrees,
                                    const std::string& source) {
  if (entity_id.empty()) return;
  if (!std::isfinite(lat) || !std::isfinite(lon)) return;

  std::lock_guard<std::mutex> lock(mu_);

  if (!ShouldRecord(entity_id, ts_ms, lat, lon, sog_knots)) return;

  // Update delta state.
  auto& lp = last_pos_[entity_id];
  lp.lat   = lat;
  lp.lon   = lon;
  lp.sog   = sog_knots.value_or(lp.sog);
  lp.ts_ms = ts_ms;

  auto* s = stmt_insert_track_;
  sqlite3_reset(s);
  sqlite3_bind_text (s, 1, entity_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(s, 2, static_cast<sqlite3_int64>(ts_ms));
  sqlite3_bind_double(s, 3, lat);
  sqlite3_bind_double(s, 4, lon);
  if (altitude_ft.has_value())  sqlite3_bind_double(s, 5, *altitude_ft);
  else                          sqlite3_bind_null(s, 5);
  if (sog_knots.has_value())    sqlite3_bind_double(s, 6, *sog_knots);
  else                          sqlite3_bind_null(s, 6);
  if (cog_degrees.has_value())  sqlite3_bind_double(s, 7, *cog_degrees);
  else                          sqlite3_bind_null(s, 7);
  sqlite3_bind_text(s, 8, source.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_step(s);
}

// ----------------------------------------------------------------
// Raw frame storage
// ----------------------------------------------------------------

void TrackDatabase::RecordRawFrame(uint64_t ts_ms, uint32_t receiver_id,
                                    const std::string& source, double frequency_hz,
                                    const std::string& payload,
                                    const std::string& fields_json) {
  std::lock_guard<std::mutex> lock(mu_);
  auto* s = stmt_insert_raw_;
  sqlite3_reset(s);
  sqlite3_bind_int64 (s, 1, static_cast<sqlite3_int64>(ts_ms));
  sqlite3_bind_int   (s, 2, static_cast<int>(receiver_id));
  sqlite3_bind_text  (s, 3, source.c_str(),      -1, SQLITE_TRANSIENT);
  sqlite3_bind_double(s, 4, frequency_hz);
  sqlite3_bind_text  (s, 5, payload.c_str(),     -1, SQLITE_TRANSIENT);
  sqlite3_bind_text  (s, 6, fields_json.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_step(s);
}

// ----------------------------------------------------------------
// Queries
// ----------------------------------------------------------------

std::vector<TrackPoint> TrackDatabase::QueryTracks(const std::string& entity_id,
                                                    uint64_t from_ms,
                                                    uint64_t to_ms) const {
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<TrackPoint> result;

  sqlite3_stmt* s = nullptr;
  SqliteCheck(sqlite3_prepare_v2(db_,
      "SELECT ts, lat, lon, altitude_ft, sog_knots, cog_degrees, source "
      "FROM tracks WHERE entity_id=? AND ts>=? AND ts<=? ORDER BY ts",
      -1, &s, nullptr), "QueryTracks prepare");

  sqlite3_bind_text (s, 1, entity_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(s, 2, static_cast<sqlite3_int64>(from_ms));
  sqlite3_bind_int64(s, 3, static_cast<sqlite3_int64>(to_ms));

  while (sqlite3_step(s) == SQLITE_ROW) {
    TrackPoint p;
    p.ts_ms = static_cast<uint64_t>(sqlite3_column_int64(s, 0));
    p.lat   = sqlite3_column_double(s, 1);
    p.lon   = sqlite3_column_double(s, 2);
    if (sqlite3_column_type(s, 3) != SQLITE_NULL) p.altitude_ft = sqlite3_column_double(s, 3);
    if (sqlite3_column_type(s, 4) != SQLITE_NULL) p.sog_knots   = sqlite3_column_double(s, 4);
    if (sqlite3_column_type(s, 5) != SQLITE_NULL) p.cog_degrees = sqlite3_column_double(s, 5);
    const char* src = reinterpret_cast<const char*>(sqlite3_column_text(s, 6));
    if (src) p.source = src;
    result.push_back(std::move(p));
  }
  sqlite3_finalize(s);
  return result;
}

std::vector<EntityInfo> TrackDatabase::QueryEntities(const std::string& kind,
                                                      uint64_t active_since_ms) const {
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<EntityInfo> result;

  const std::string sql = kind.empty()
      ? "SELECT id, kind, name, callsign, first_seen, last_seen FROM entities "
        "WHERE last_seen >= ? ORDER BY last_seen DESC"
      : "SELECT id, kind, name, callsign, first_seen, last_seen FROM entities "
        "WHERE kind=? AND last_seen >= ? ORDER BY last_seen DESC";

  sqlite3_stmt* s = nullptr;
  SqliteCheck(sqlite3_prepare_v2(db_, sql.c_str(), -1, &s, nullptr),
              "QueryEntities prepare");

  if (kind.empty()) {
    sqlite3_bind_int64(s, 1, static_cast<sqlite3_int64>(active_since_ms));
  } else {
    sqlite3_bind_text (s, 1, kind.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int64(s, 2, static_cast<sqlite3_int64>(active_since_ms));
  }

  while (sqlite3_step(s) == SQLITE_ROW) {
    EntityInfo e;
    auto col_str = [&](int i) -> std::string {
      const char* p = reinterpret_cast<const char*>(sqlite3_column_text(s, i));
      return p ? p : "";
    };
    e.id           = col_str(0);
    e.kind         = col_str(1);
    e.name         = col_str(2);
    e.callsign     = col_str(3);
    e.first_seen_ms = static_cast<uint64_t>(sqlite3_column_int64(s, 4));
    e.last_seen_ms  = static_cast<uint64_t>(sqlite3_column_int64(s, 5));
    result.push_back(std::move(e));
  }
  sqlite3_finalize(s);
  return result;
}

// ----------------------------------------------------------------
// Maintenance
// ----------------------------------------------------------------

void TrackDatabase::Prune(uint64_t tracks_max_age_ms, uint64_t raw_max_age_ms) {
  const uint64_t now = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());

  std::lock_guard<std::mutex> lock(mu_);
  if (tracks_max_age_ms > 0 && now > tracks_max_age_ms) {
    sqlite3_stmt* s = nullptr;
    if (sqlite3_prepare_v2(db_, "DELETE FROM tracks WHERE ts < ?",
                            -1, &s, nullptr) == SQLITE_OK) {
      sqlite3_bind_int64(s, 1, static_cast<sqlite3_int64>(now - tracks_max_age_ms));
      sqlite3_step(s);
      sqlite3_finalize(s);
    }
  }
  if (raw_max_age_ms > 0 && now > raw_max_age_ms) {
    sqlite3_stmt* s = nullptr;
    if (sqlite3_prepare_v2(db_, "DELETE FROM raw_frames WHERE ts < ?",
                            -1, &s, nullptr) == SQLITE_OK) {
      sqlite3_bind_int64(s, 1, static_cast<sqlite3_int64>(now - raw_max_age_ms));
      sqlite3_step(s);
      sqlite3_finalize(s);
    }
  }
}

}  // namespace multi_radio
