#include "weather_worker.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QTimer>
#include <QVariant>
#include <QUrl>

namespace multi_radio {
namespace {
constexpr char kObservationUrl[] =
    "https://opendata-download-metanalys.smhi.se/api/category/mesan2g/version/3/geotype/point/lon/15.61452/lat/56.16855/data.json";
constexpr char kForecastUrl[] =
    "https://opendata-download-metfcst.smhi.se/api/category/snow1g/version/1/geotype/point/lon/15.61452/lat/56.16855/data.json";

QVariant JsonValueAtPath(const QJsonValue& value, const QStringList& path) {
  QJsonValue current = value;
  for (const QString& part : path) {
    if (!current.isObject()) {
      return {};
    }
    current = current.toObject().value(part);
  }
  return current.toVariant();
}

QJsonArray ParseSeries(const QByteArray& payload) {
  const QJsonDocument doc = QJsonDocument::fromJson(payload);
  if (!doc.isObject()) {
    return {};
  }

  const QJsonValue time_series = doc.object().value("timeSeries");
  if (time_series.isArray()) {
    return time_series.toArray();
  }

  const QJsonValue legacy = doc.object().value("timeseries");
  if (legacy.isArray()) {
    return legacy.toArray();
  }

  return {};
}

QVariantMap WeatherFieldsFromData(const QJsonValue& data_value) {
  QVariantMap fields;
  if (!data_value.isObject()) {
    return fields;
  }

  const QJsonObject data = data_value.toObject();
  for (const char* key : {"air_temperature", "wind_from_direction", "wind_speed",
                           "wind_speed_of_gust", "air_pressure_at_mean_sea_level",
                           "cloud_area_fraction", "visibility_in_air", "symbol_code"}) {
    const QJsonValue value = data.value(key);
    if (value.isDouble() || value.isString() || value.isBool() || value.isNull()) {
      if (QString::fromLatin1(key) == "symbol_code" && value.isDouble()) {
        fields.insert(QString::fromLatin1(key), WeatherWorker::MapSmhiSymbolToSvg(value.toInt()));
      } else if (QString::fromLatin1(key) == "wind_from_direction" && value.isDouble()) {
        fields.insert(QString::fromLatin1(key), WeatherWorker::MapWindDirectionToCode(value.toDouble()));
      } else {
        fields.insert(QString::fromLatin1(key), value.toVariant());
      }
    }
  }
  return fields;
}

}  // namespace

WeatherWorker::WeatherWorker(QObject* parent) : QObject(parent) {
  network_manager_ = new QNetworkAccessManager(this);
  timer_ = new QTimer(this);
  timer_->setInterval(60 * 60 * 1000);
  timer_->setSingleShot(false);
  connect(timer_, &QTimer::timeout, this, &WeatherWorker::HandleTimer);

  observation_url_ = QString::fromLatin1(kObservationUrl);
  forecast_url_ = QString::fromLatin1(kForecastUrl);
}

WeatherWorker::~WeatherWorker() {
  Stop();
}

void WeatherWorker::StartHourlyUpdates(int interval_ms) {
  if (started_) {
    return;
  }
  started_ = true;

  if (thread_ == nullptr) {
    thread_ = new QThread(this);
    moveToThread(thread_);
    timer_->moveToThread(thread_);
    network_manager_->moveToThread(thread_);
    connect(thread_, &QThread::started, this, [this, interval_ms]() {
      timer_->setInterval(interval_ms);
      timer_->start();
      FetchNow();
    });
  }

  thread_->start();
}

void WeatherWorker::Stop() {
  if (!started_) {
    return;
  }
  started_ = false;
  if (timer_ != nullptr) {
    QMetaObject::invokeMethod(timer_, "stop", Qt::QueuedConnection);
  }
  if (thread_ != nullptr) {
    thread_->quit();
    thread_->wait();
  }
}

void WeatherWorker::FetchNow() {
  if (!started_) {
    StartHourlyUpdates();
    return;
  }
  BeginFetch();
}

void WeatherWorker::HandleTimer() {
  BeginFetch();
}

void WeatherWorker::BeginFetch() {
  if (observation_in_flight_ || forecast_in_flight_) {
    has_pending_fetch_ = true;
    return;
  }

  observation_payload_.clear();
  forecast_payload_.clear();
  observation_in_flight_ = true;
  forecast_in_flight_ = true;

  QNetworkRequest observation_request{QUrl(observation_url_)};
  observation_request.setRawHeader("Accept", "application/json");
  auto* observation_reply = network_manager_->get(observation_request);
  connect(observation_reply, &QNetworkReply::finished, this, &WeatherWorker::HandleObservationReply);
  connect(observation_reply, &QNetworkReply::errorOccurred, this, [this, observation_reply](QNetworkReply::NetworkError error) {
    Q_UNUSED(error);
    if (observation_reply->error() != QNetworkReply::NoError) {
      emit errorOccurred(observation_reply->errorString());
      observation_in_flight_ = false;
      FinishIfReady();
    }
  });

  QNetworkRequest forecast_request{QUrl(forecast_url_)};
  forecast_request.setRawHeader("Accept", "application/json");
  auto* forecast_reply = network_manager_->get(forecast_request);
  connect(forecast_reply, &QNetworkReply::finished, this, &WeatherWorker::HandleForecastReply);
  connect(forecast_reply, &QNetworkReply::errorOccurred, this, [this, forecast_reply](QNetworkReply::NetworkError error) {
    Q_UNUSED(error);
    if (forecast_reply->error() != QNetworkReply::NoError) {
      emit errorOccurred(forecast_reply->errorString());
      forecast_in_flight_ = false;
      FinishIfReady();
    }
  });
}

void WeatherWorker::HandleObservationReply() {
  auto* reply = qobject_cast<QNetworkReply*>(sender());
  if (!reply) {
    observation_in_flight_ = false;
    FinishIfReady();
    return;
  }

  if (reply->error() == QNetworkReply::NoError) {
    observation_payload_ = reply->readAll();
  }
  observation_in_flight_ = false;
  reply->deleteLater();
  FinishIfReady();
}

void WeatherWorker::HandleForecastReply() {
  auto* reply = qobject_cast<QNetworkReply*>(sender());
  if (!reply) {
    forecast_in_flight_ = false;
    FinishIfReady();
    return;
  }

  if (reply->error() == QNetworkReply::NoError) {
    forecast_payload_ = reply->readAll();
  }
  forecast_in_flight_ = false;
  reply->deleteLater();
  FinishIfReady();
}

void WeatherWorker::FinishIfReady() {
  if (observation_in_flight_ || forecast_in_flight_) {
    return;
  }

  WeatherSnapshot snapshot;
  snapshot.timestamp = ExtractSeriesTimestamp(observation_payload_, 0);
  snapshot.observation = ExtractSeriesFields(observation_payload_, 0);
  snapshot.forecast_one = ExtractSeriesFields(forecast_payload_, 0);
  snapshot.forecast_two = ExtractSeriesFields(forecast_payload_, 1);

  emit weatherUpdated(snapshot);

  if (has_pending_fetch_) {
    has_pending_fetch_ = false;
    BeginFetch();
  }
}

QString WeatherWorker::MapSmhiSymbolToSvg(int symbol_code) {
  switch (symbol_code) {
    case 1: return "01d.svg";
    case 2: return "02d.svg";
    case 3:
    case 4: return "03d.svg";
    case 5:
    case 6: return "04.svg";
    case 7: return "15.svg";
    case 8: return "40d.svg";
    case 9: return "05d.svg";
    case 10: return "41d.svg";
    case 11: return "06d.svg";
    case 12: return "42d.svg";
    case 13: return "07d.svg";
    case 14: return "43d.svg";
    case 15: return "44d.svg";
    case 16: return "08d.svg";
    case 17: return "45d.svg";
    case 18: return "46.svg";
    case 19: return "09.svg";
    case 20: return "10.svg";
    case 21: return "11.svg";
    case 22: return "47.svg";
    case 23: return "12.svg";
    case 24: return "48.svg";
    case 25: return "49.svg";
    case 26: return "13.svg";
    case 27: return "50.svg";
    default: return "";
  }
}

QString WeatherWorker::MapWindDirectionToCode(double degrees) {
  if (!std::isfinite(degrees)) {
    return {};
  }

  double normalized = std::fmod(degrees, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }

  static const struct { double start; double end; const char* code; } kRanges[] = {
      {348.75, 360.0, "N"},
      {0.0, 11.25, "N"},
      {11.25, 33.75, "NNE"},
      {33.75, 56.25, "NE"},
      {56.25, 78.75, "ENE"},
      {78.75, 101.25, "E"},
      {101.25, 123.75, "ESE"},
      {123.75, 146.25, "SE"},
      {146.25, 168.75, "SSE"},
      {168.75, 191.25, "S"},
      {191.25, 213.75, "SSW"},
      {213.75, 236.25, "SW"},
      {236.25, 258.75, "WSW"},
      {258.75, 281.25, "W"},
      {281.25, 303.75, "WNW"},
      {303.75, 326.25, "NW"},
      {326.25, 348.75, "NNW"},
  };

  for (const auto& range : kRanges) {
    if (normalized >= range.start && normalized < range.end) {
      return QString::fromLatin1(range.code);
    }
  }
  return "N";
}

QVariantMap WeatherWorker::ExtractSeriesFields(const QByteArray& json, int index) {
  const QJsonArray series = ParseSeries(json);
  if (index < 0 || index >= series.size()) {
    return {};
  }

  const QJsonValue entry = series.at(index);
  if (!entry.isObject()) {
    return {};
  }

  const QJsonValue data = entry.toObject().value("data");
  return WeatherFieldsFromData(data);
}

QString WeatherWorker::ExtractSeriesTimestamp(const QByteArray& json, int index) {
  const QJsonArray series = ParseSeries(json);
  if (index < 0 || index >= series.size()) {
    return {};
  }

  const QJsonValue entry = series.at(index);
  if (!entry.isObject()) {
    return {};
  }

  return entry.toObject().value("time").toString();
}

QVariantMap WeatherWorker::ExtractRelevantFields(const QJsonValue& data_value) {
  return WeatherFieldsFromData(data_value);
}

}  // namespace multi_radio
