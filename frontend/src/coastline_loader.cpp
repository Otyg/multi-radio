#include "coastline_loader.hpp"

#include "coastline_cache.hpp"

#include <cmath>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>
#include <QUrl>
#include <QUrlQuery>
#include <QtConcurrent/QtConcurrent>

namespace multi_radio {

namespace {

constexpr const char* kBaseUrl =
    "https://api.lantmateriet.se/ogc-features/v1/hydrografi/collections/"
    "LandWaterBoundary/items";

constexpr int kLimit = 10000;

}  // namespace

CoastlineLoader::CoastlineLoader(const QString& cache_path, QObject* parent)
    : QObject(parent),
      nam_(new QNetworkAccessManager(this)),
      cache_(std::make_unique<CoastlineCache>(cache_path)) {
  connect(nam_, &QNetworkAccessManager::finished, this, &CoastlineLoader::OnReply);
}

CoastlineLoader::~CoastlineLoader() = default;

// -----------------------------------------------------------------------
// Public
// -----------------------------------------------------------------------

void CoastlineLoader::RequestView(double lat_min, double lon_min,
                                   double lat_max, double lon_max) {
  // Cancel all in-flight requests.
  for (auto it = reply_map_.begin(); it != reply_map_.end(); ++it)
    it.key()->abort();
  reply_map_.clear();
  pending_.clear();

  const auto tiles = TilesForBbox(lat_min, lon_min, lat_max, lon_max);

  bool any_fetch = false;
  for (const TileKey& tile : tiles) {
    auto cached = cache_->GetTile(tile.first, tile.second);
    if (cached.has_value()) {
      // Deliver immediately via queued invocation so the caller is never
      // re-entered synchronously.
      QTimer::singleShot(0, this, [this, tile, polys = std::move(*cached)]() mutable {
        emit TileReady(tile.first, tile.second, std::move(polys));
      });
    } else {
      pending_.insert(tile);
      FetchTile(tile);
      any_fetch = true;
    }
  }
  if (any_fetch) emit FetchStarted();
}

// -----------------------------------------------------------------------
// Tile helpers
// -----------------------------------------------------------------------

QVector<CoastlineLoader::TileKey> CoastlineLoader::TilesForBbox(
    double lat_min, double lon_min, double lat_max, double lon_max) {
  QVector<TileKey> tiles;
  for (int la = static_cast<int>(std::floor(lat_min));
       la <= static_cast<int>(std::floor(lat_max)); ++la)
    for (int lo = static_cast<int>(std::floor(lon_min));
         lo <= static_cast<int>(std::floor(lon_max)); ++lo)
      tiles.append({la, lo});
  return tiles;
}

void CoastlineLoader::FetchTile(const TileKey& tile) {
  const double lat_min = static_cast<double>(tile.first);
  const double lon_min = static_cast<double>(tile.second);
  const double lat_max = lat_min + 1.0;
  const double lon_max = lon_min + 1.0;

  QUrl url(kBaseUrl);
  QUrlQuery q;
  q.addQueryItem("bbox",
                 QString("%1,%2,%3,%4")
                     .arg(lon_min, 0, 'f', 4).arg(lat_min, 0, 'f', 4)
                     .arg(lon_max, 0, 'f', 4).arg(lat_max, 0, 'f', 4));
  q.addQueryItem("limit", QString::number(kLimit));
  q.addQueryItem("f", "json");
  url.setQuery(q);

  QNetworkRequest req(url);
  req.setHeader(QNetworkRequest::UserAgentHeader, "multi-radio/1.0");
  req.setRawHeader("Accept", "application/geo+json");
  if (!username_.isEmpty()) {
    const QByteArray credentials =
        (username_ + ':' + password_).toUtf8().toBase64();
    req.setRawHeader("Authorization", "Basic " + credentials);
  }

  QNetworkReply* reply = nam_->get(req);
  reply_map_.insert(reply, tile);
}

// -----------------------------------------------------------------------
// Network reply — parse in background thread
// -----------------------------------------------------------------------

void CoastlineLoader::OnReply(QNetworkReply* reply) {
  reply->deleteLater();
  const TileKey tile = reply_map_.take(reply);
  if (!pending_.contains(tile)) return;
  pending_.remove(tile);

  if (reply->error() != QNetworkReply::NoError) {
    emit FetchFailed(reply->errorString());
    return;
  }

  const QByteArray data = reply->readAll();

  // Parse and cache in a background thread; deliver result via queued signal.
  auto* watcher = new QFutureWatcher<QVector<QPolygonF>>(this);
  connect(watcher, &QFutureWatcher<QVector<QPolygonF>>::finished, this,
          [this, tile, watcher]() {
            QVector<QPolygonF> polys = watcher->result();
            watcher->deleteLater();
            cache_->StoreTile(tile.first, tile.second, polys);
            emit TileReady(tile.first, tile.second, std::move(polys));
          });
  watcher->setFuture(QtConcurrent::run([data]() { return ParseGeoJson(data); }));
}

// -----------------------------------------------------------------------
// GeoJSON parsing (runs in worker thread — no Qt GUI calls)
// -----------------------------------------------------------------------

QVector<QPolygonF> CoastlineLoader::ParseGeoJson(const QByteArray& data) {
  QVector<QPolygonF> result;
  const QJsonDocument doc = QJsonDocument::fromJson(data);
  if (!doc.isObject()) return result;

  const QJsonArray features = doc.object().value("features").toArray();
  result.reserve(features.size());

  auto parse_line = [&](const QJsonArray& pts) {
    QPolygonF poly;
    poly.reserve(pts.size());
    for (const QJsonValue& pt : pts) {
      const QJsonArray arr = pt.toArray();
      if (arr.size() < 2) continue;
      poly.append(QPointF(arr[1].toDouble(), arr[0].toDouble()));  // x=lat, y=lon
    }
    if (poly.size() >= 2) result.append(std::move(poly));
  };

  for (const QJsonValue& fv : features) {
    const QJsonObject geom = fv.toObject().value("geometry").toObject();
    const QString type = geom.value("type").toString();
    const QJsonValue cv = geom.value("coordinates");

    if (type == "LineString") {
      parse_line(cv.toArray());
    } else if (type == "MultiLineString") {
      for (const QJsonValue& line : cv.toArray())
        parse_line(line.toArray());
    } else if (type == "Polygon") {
      const QJsonArray rings = cv.toArray();
      if (!rings.isEmpty()) parse_line(rings[0].toArray());
    } else if (type == "MultiPolygon") {
      for (const QJsonValue& pv : cv.toArray()) {
        const QJsonArray rings = pv.toArray();
        if (!rings.isEmpty()) parse_line(rings[0].toArray());
      }
    }
  }
  return result;
}

}  // namespace multi_radio
