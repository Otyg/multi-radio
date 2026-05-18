#include "coastline_cache.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>

#include <QDataStream>
#include <QDir>
#include <QFileInfo>

#include <sqlite3.h>

namespace multi_radio {

namespace {

static uint64_t NowMs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
}

}  // namespace

CoastlineCache::CoastlineCache(const QString& db_path) {
  // Ensure directory exists.
  QDir().mkpath(QFileInfo(db_path).absolutePath());

  const int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX;
  if (sqlite3_open_v2(db_path.toUtf8().constData(), &db_, flags, nullptr) != SQLITE_OK) {
    db_ = nullptr;
    return;
  }
  sqlite3_exec(db_, "PRAGMA journal_mode=WAL", nullptr, nullptr, nullptr);
  sqlite3_exec(db_, "PRAGMA synchronous=NORMAL", nullptr, nullptr, nullptr);
  sqlite3_exec(db_, R"sql(
    CREATE TABLE IF NOT EXISTS tiles (
      tile_key   TEXT    PRIMARY KEY,
      fetched_at INTEGER NOT NULL,
      points     BLOB    NOT NULL
    );
  )sql", nullptr, nullptr, nullptr);
}

CoastlineCache::~CoastlineCache() {
  if (db_) sqlite3_close(db_);
}

QString CoastlineCache::TileKey(int lat_deg, int lon_deg) {
  return QString::number(lat_deg) + ':' + QString::number(lon_deg);
}

// Encode: each polygon is a sequence of (lat, lon) float64 pairs.
// Polygons are separated by a sentinel pair (NaN, NaN).
QByteArray CoastlineCache::Encode(const QVector<QPolygonF>& polygons) {
  QByteArray out;
  QDataStream ds(&out, QIODevice::WriteOnly);
  ds.setByteOrder(QDataStream::LittleEndian);
  ds.setFloatingPointPrecision(QDataStream::DoublePrecision);
  for (const QPolygonF& poly : polygons) {
    for (const QPointF& pt : poly) {
      ds << pt.x() << pt.y();  // lat, lon stored as (x=lat, y=lon)
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    ds << nan << nan;          // polygon separator
  }
  return out;
}

QVector<QPolygonF> CoastlineCache::Decode(const QByteArray& blob) {
  QVector<QPolygonF> result;
  QDataStream ds(blob);
  ds.setByteOrder(QDataStream::LittleEndian);
  ds.setFloatingPointPrecision(QDataStream::DoublePrecision);

  QPolygonF current;
  while (!ds.atEnd()) {
    double lat, lon;
    ds >> lat >> lon;
    if (std::isnan(lat) || std::isnan(lon)) {
      if (!current.isEmpty()) {
        result.append(current);
        current.clear();
      }
    } else {
      current.append(QPointF(lat, lon));
    }
  }
  if (!current.isEmpty()) result.append(current);
  return result;
}

void CoastlineCache::ClearAll() {
  if (!db_) return;
  sqlite3_exec(db_, "DELETE FROM tiles", nullptr, nullptr, nullptr);
}

std::optional<QVector<QPolygonF>> CoastlineCache::GetTile(int lat_deg, int lon_deg) const {
  if (!db_) return std::nullopt;

  const QString key = TileKey(lat_deg, lon_deg);
  sqlite3_stmt* s = nullptr;
  if (sqlite3_prepare_v2(db_,
        "SELECT fetched_at, points FROM tiles WHERE tile_key=?",
        -1, &s, nullptr) != SQLITE_OK)
    return std::nullopt;

  sqlite3_bind_text(s, 1, key.toUtf8().constData(), -1, SQLITE_TRANSIENT);
  std::optional<QVector<QPolygonF>> result;
  if (sqlite3_step(s) == SQLITE_ROW) {
    const uint64_t fetched_at = static_cast<uint64_t>(sqlite3_column_int64(s, 0));
    if (NowMs() - fetched_at < kMaxAgeMs) {
      const void* data = sqlite3_column_blob(s, 1);
      const int   size = sqlite3_column_bytes(s, 1);
      const QByteArray blob(static_cast<const char*>(data), size);
      result = Decode(blob);
    }
  }
  sqlite3_finalize(s);
  return result;
}

void CoastlineCache::StoreTile(int lat_deg, int lon_deg, const QVector<QPolygonF>& polygons) {
  if (!db_) return;

  const QString key  = TileKey(lat_deg, lon_deg);
  const QByteArray blob = Encode(polygons);
  const uint64_t now = NowMs();

  sqlite3_stmt* s = nullptr;
  if (sqlite3_prepare_v2(db_,
        "INSERT OR REPLACE INTO tiles (tile_key, fetched_at, points) VALUES (?,?,?)",
        -1, &s, nullptr) != SQLITE_OK)
    return;

  sqlite3_bind_text  (s, 1, key.toUtf8().constData(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64 (s, 2, static_cast<sqlite3_int64>(now));
  sqlite3_bind_blob  (s, 3, blob.constData(), blob.size(), SQLITE_TRANSIENT);
  sqlite3_step(s);
  sqlite3_finalize(s);
}

}  // namespace multi_radio
