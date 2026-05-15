#pragma once

#include <memory>
#include <utility>

#include <QMap>
#include <QObject>
#include <QPolygonF>
#include <QSet>
#include <QString>
#include <QVector>

class QNetworkAccessManager;
class QNetworkReply;

namespace multi_radio {

class CoastlineCache;

// Fetches and caches Lantmäteriets coastline data.
//
// Call RequestView() to load tiles that cover a given bounding box.
// If all tiles are in the local SQLite cache they are delivered synchronously
// (via a queued signal so callers never block). Missing tiles are fetched from
// the Lantmäteriet OGC-Features API and stored in the cache before delivery.
//
// The loader merges all tiles that were requested for the current view and
// emits CoastlineReady() once every needed tile is available.
class CoastlineLoader : public QObject {
  Q_OBJECT

 public:
  explicit CoastlineLoader(const QString& cache_path, QObject* parent = nullptr);
  ~CoastlineLoader() override;

  void SetCredentials(const QString& username, const QString& password) {
    username_ = username;
    password_ = password;
  }

  // Request coastlines covering the given bbox (EPSG:4326).
  // Emits CoastlineReady() once all tiles are available.
  void RequestView(double lat_min, double lon_min,
                   double lat_max, double lon_max);

 signals:
  // Emitted when all tiles for the last RequestView() call are ready.
  // Polygons are in lat/lon coordinates (QPointF.x = lat, QPointF.y = lon).
  void CoastlineReady(QVector<QPolygonF> polygons);
  void FetchStarted();
  void FetchFailed(const QString& error);

 private slots:
  void OnReply(QNetworkReply* reply);

 private:
  using TileKey = QPair<int, int>;  // (lat_floor_deg, lon_floor_deg)

  static QVector<TileKey> TilesForBbox(double lat_min, double lon_min,
                                        double lat_max, double lon_max);
  void FetchTile(const TileKey& tile);
  void CheckDone();
  QVector<QPolygonF> ParseGeoJson(const QByteArray& data) const;

  QNetworkAccessManager*         nam_ = nullptr;
  std::unique_ptr<CoastlineCache> cache_;
  QString                        username_;
  QString                        password_;

  // State for the current request batch.
  QVector<TileKey>                        requested_;
  QMap<TileKey, QVector<QPolygonF>>       loaded_;
  QSet<TileKey>                           pending_;

  // Used to tag in-flight replies with their tile.
  QMap<QNetworkReply*, TileKey>           reply_map_;
};

}  // namespace multi_radio
