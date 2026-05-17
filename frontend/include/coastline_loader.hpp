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

// Fetches and caches Lantmäteriets LandWaterBoundary coastline data.
//
// Call RequestView() to load all 1°×1° tiles that cover the visible bbox.
// Each tile is delivered independently via TileReady() as soon as it is
// available — either from the local SQLite cache (almost instant) or after
// a network fetch.  JSON parsing runs in a background thread so the GUI
// thread is never blocked.
class CoastlineLoader : public QObject {
  Q_OBJECT

 public:
  // collection: OGC collection name, e.g. "LandWaterBoundary" or "StandingWater"
  explicit CoastlineLoader(const QString& cache_path,
                           const QString& collection = "LandWaterBoundary",
                           QObject* parent = nullptr);
  ~CoastlineLoader() override;

  void SetCredentials(const QString& username, const QString& password) {
    username_ = username;
    password_ = password;
  }

  // Request all tiles covering the given bbox (EPSG:4326, WGS84).
  // Cancels any in-flight requests that are no longer relevant.
  void RequestView(double lat_min, double lon_min,
                   double lat_max, double lon_max);

 signals:
  // Emitted once per tile as soon as its geometry is available.
  // Polygons are in lat/lon (QPointF.x = lat, QPointF.y = lon).
  void TileReady(int lat_deg, int lon_deg, QVector<QPolygonF> polygons);

  void FetchStarted();
  void FetchFailed(const QString& error);
  void FetchingUrl(const QString& url);  // emitted just before each HTTP GET

 private slots:
  void OnReply(QNetworkReply* reply);

 private:
  using TileKey = QPair<int, int>;

  static QVector<TileKey> TilesForBbox(double lat_min, double lon_min,
                                        double lat_max, double lon_max);
  void FetchTile(const TileKey& tile);
  static QVector<QPolygonF> ParseGeoJson(const QByteArray& data);

  QNetworkAccessManager*          nam_ = nullptr;
  std::unique_ptr<CoastlineCache> cache_;
  QString                         collection_;
  QString                         username_;
  QString                         password_;

  QSet<TileKey>                   pending_;
  QMap<QNetworkReply*, TileKey>   reply_map_;
};

}  // namespace multi_radio
