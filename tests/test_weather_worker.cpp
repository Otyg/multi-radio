#include "weather_worker.hpp"

#include <QCoreApplication>
#include <QTimer>

#include <iostream>

int main(int argc, char** argv) {
  QCoreApplication app(argc, argv);

  multi_radio::WeatherWorker worker;
  QObject::connect(&worker, &multi_radio::WeatherWorker::weatherUpdated, [](const multi_radio::WeatherSnapshot& snapshot) {
    std::cout << "timestamp: " << snapshot.timestamp.toStdString() << '\n';
    std::cout << "observation symbol: " << snapshot.observation["symbol_code"].toString().toStdString() << '\n';
    std::cout << "observation wind: " << snapshot.observation["wind_from_direction"].toString().toStdString() << '\n';
    std::cout << "forecast_one wind: " << snapshot.forecast_one["wind_from_direction"].toString().toStdString() << '\n';
    std::cout << "forecast_two wind: " << snapshot.forecast_two["wind_from_direction"].toString().toStdString() << '\n';
    QCoreApplication::quit();
  });

  QObject::connect(&worker, &multi_radio::WeatherWorker::errorOccurred, [](const QString& message) {
    std::cerr << "weather fetch error: " << message.toStdString() << '\n';
    QCoreApplication::exit(1);
  });

  worker.StartHourlyUpdates(1000);

  QTimer::singleShot(15000, &app, &QCoreApplication::quit);
  return app.exec();
}
