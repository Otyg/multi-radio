#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <QPolygonF>
#include <QString>
#include <QVector>

struct sqlite3;

namespace multi_radio {

// SQLite-backed tile cache for coastline geometry.
// Each tile covers a 1°×1° bounding box, keyed as "lat_floor:lon_floor".
// Tiles are considered valid for 30 days.
class CoastlineCache {
 public:
  explicit CoastlineCache(const QString& db_path);
  ~CoastlineCache();

  CoastlineCache(const CoastlineCache&)            = delete;
  CoastlineCache& operator=(const CoastlineCache&) = delete;

  // Returns polygons for the tile if cached and not expired, else nullopt.
  std::optional<QVector<QPolygonF>> GetTile(int lat_deg, int lon_deg) const;

  // Persists polygons for a tile.
  void StoreTile(int lat_deg, int lon_deg, const QVector<QPolygonF>& polygons);

  // Remove all cached tiles.
  void ClearAll();

 private:
  static QString TileKey(int lat_deg, int lon_deg);
  // Encode/decode polygon list to/from a compact binary blob.
  static QByteArray Encode(const QVector<QPolygonF>& polygons);
  static QVector<QPolygonF> Decode(const QByteArray& blob);

  sqlite3* db_ = nullptr;

  // 30-day TTL in milliseconds.
  static constexpr uint64_t kMaxAgeMs = 30ULL * 24 * 3600 * 1000;
};

}  // namespace multi_radio
