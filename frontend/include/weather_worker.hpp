#pragma once

#include <QObject>
#include <QByteArray>
#include <QString>
#include <QThread>
#include <QVariant>

class QNetworkAccessManager;
class QNetworkReply;
class QTimer;

namespace multi_radio {

struct WeatherSnapshot {
  QString timestamp;
  QVariantMap observation;
  QVariantMap forecast_one;
  QVariantMap forecast_two;
};

class WeatherWorker : public QObject {
  Q_OBJECT

 public:
  explicit WeatherWorker(QObject* parent = nullptr);
  ~WeatherWorker() override;

  void StartHourlyUpdates(int interval_ms = 60 * 60 * 1000);
  void Stop();
  void FetchNow();

  static QString MapSmhiSymbolToSvg(int symbol_code);
  static QString MapWindDirectionToCode(double degrees);
  static QVariantMap ExtractSeriesFields(const QByteArray& json, int index);
  static QString ExtractSeriesTimestamp(const QByteArray& json, int index);

 signals:
  void weatherUpdated(const WeatherSnapshot& snapshot);
  void errorOccurred(const QString& message);

 private slots:
  void HandleTimer();
  void HandleObservationReply();
  void HandleForecastReply();

 private:
  void BeginFetch();
  void FinishIfReady();
  static QVariantMap ExtractRelevantFields(const QJsonValue& data_value);

  QNetworkAccessManager* network_manager_ = nullptr;
  QTimer* timer_ = nullptr;
  QThread* thread_ = nullptr;
  QByteArray observation_payload_;
  QByteArray forecast_payload_;
  bool observation_in_flight_ = false;
  bool forecast_in_flight_ = false;
  bool has_pending_fetch_ = false;
  bool started_ = false;

  QString observation_url_;
  QString forecast_url_;
};

}  // namespace multi_radio
