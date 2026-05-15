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

namespace multi_radio {

namespace {

// Lantmäteriet OGC-Features endpoint for coastline polygons.
constexpr const char* kBaseUrl =
    "https://api.lantmateriet.se/ogc-features/v1/hydrografi/collections/"
    "Havsomradesytor/items";

constexpr int kLimit = 1000;  // features per tile request

}  // namespace

CoastlineLoader::CoastlineLoader(const QString& cache_path, QObject* parent)
    : QObject(parent),
      nam_(new QNetworkAccessManager(this)),
      cache_(std::make_unique<CoastlineCache>(cache_path)) {
  connect(nam_, &QNetworkAccessManager::finished, this, &CoastlineLoader::OnReply);
}

CoastlineLoader::~CoastlineLoader() = default;

// -----------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------

void CoastlineLoader::RequestView(double lat_min, double lon_min,
                                   double lat_max, double lon_max) {
  const auto tiles = TilesForBbox(lat_min, lon_min, lat_max, lon_max);

  // Start a fresh batch.
  requested_ = tiles;
  loaded_.clear();
  pending_.clear();
  // Cancel any in-flight replies that are no longer relevant.
  for (auto it = reply_map_.begin(); it != reply_map_.end(); ++it)
    it.key()->abort();
  reply_map_.clear();

  bool any_fetch = false;
  for (const TileKey& tile : tiles) {
    auto cached = cache_->GetTile(tile.first, tile.second);
    if (cached.has_value()) {
      loaded_.insert(tile, std::move(*cached));
    } else {
      pending_.insert(tile);
      FetchTile(tile);
      any_fetch = true;
    }
  }

  if (any_fetch) emit FetchStarted();

  // If everything was cached, deliver immediately via queued invocation so the
  // caller's constructor / slot is not re-entered synchronously.
  if (pending_.isEmpty()) {
    QTimer::singleShot(0, this, [this]() { CheckDone(); });
  }
}

// -----------------------------------------------------------------------
// Tile helpers
// -----------------------------------------------------------------------

QVector<CoastlineLoader::TileKey> CoastlineLoader::TilesForBbox(
    double lat_min, double lon_min, double lat_max, double lon_max) {
  QVector<TileKey> tiles;
  const int lat0 = static_cast<int>(std::floor(lat_min));
  const int lat1 = static_cast<int>(std::floor(lat_max));
  const int lon0 = static_cast<int>(std::floor(lon_min));
  const int lon1 = static_cast<int>(std::floor(lon_max));
  for (int la = lat0; la <= lat1; ++la)
    for (int lo = lon0; lo <= lon1; ++lo)
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
  // OGC-Features bbox order: minLon,minLat,maxLon,maxLat
  q.addQueryItem("bbox",
                 QString("%1,%2,%3,%4")
                     .arg(lon_min, 0, 'f', 4)
                     .arg(lat_min, 0, 'f', 4)
                     .arg(lon_max, 0, 'f', 4)
                     .arg(lat_max, 0, 'f', 4));
  q.addQueryItem("limit",   QString::number(kLimit));
  q.addQueryItem("srsName", "EPSG:4326");
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
// Network reply handler
// -----------------------------------------------------------------------

void CoastlineLoader::OnReply(QNetworkReply* reply) {
  reply->deleteLater();
  const TileKey tile = reply_map_.take(reply);
  if (!pending_.contains(tile)) return;  // belongs to a cancelled batch

  if (reply->error() != QNetworkReply::NoError) {
    pending_.remove(tile);
    emit FetchFailed(reply->errorString());
    CheckDone();
    return;
  }

  const QByteArray data = reply->readAll();
  const QVector<QPolygonF> polygons = ParseGeoJson(data);
  cache_->StoreTile(tile.first, tile.second, polygons);
  loaded_.insert(tile, polygons);
  pending_.remove(tile);
  CheckDone();
}

// -----------------------------------------------------------------------
// Merge and emit
// -----------------------------------------------------------------------

void CoastlineLoader::CheckDone() {
  if (!pending_.isEmpty()) return;

  QVector<QPolygonF> merged;
  for (const TileKey& tile : requested_) {
    auto it = loaded_.find(tile);
    if (it != loaded_.end())
      merged.append(it.value());
  }
  emit CoastlineReady(merged);
}

// -----------------------------------------------------------------------
// GeoJSON parsing
// -----------------------------------------------------------------------

QVector<QPolygonF> CoastlineLoader::ParseGeoJson(const QByteArray& data) const {
  QVector<QPolygonF> result;
  const QJsonDocument doc = QJsonDocument::fromJson(data);
  if (!doc.isObject()) return result;

  const QJsonArray features = doc.object().value("features").toArray();
  for (const QJsonValue& fv : features) {
    const QJsonObject geom = fv.toObject().value("geometry").toObject();
    const QString type = geom.value("type").toString();
    const QJsonValue coords_val = geom.value("coordinates");

    // Handle Polygon and MultiPolygon.
    auto parse_ring = [&](const QJsonArray& ring) {
      QPolygonF poly;
      poly.reserve(ring.size());
      for (const QJsonValue& pt : ring) {
        const QJsonArray arr = pt.toArray();
        if (arr.size() < 2) continue;
        const double lon = arr[0].toDouble();
        const double lat = arr[1].toDouble();
        poly.append(QPointF(lat, lon));  // x=lat, y=lon
      }
      if (!poly.isEmpty()) result.append(poly);
    };

    if (type == "Polygon") {
      const QJsonArray rings = coords_val.toArray();
      if (!rings.isEmpty())
        parse_ring(rings[0].toArray());  // outer ring only
    } else if (type == "MultiPolygon") {
      for (const QJsonValue& poly_val : coords_val.toArray()) {
        const QJsonArray rings = poly_val.toArray();
        if (!rings.isEmpty())
          parse_ring(rings[0].toArray());
      }
    }
  }
  return result;
}

}  // namespace multi_radio
