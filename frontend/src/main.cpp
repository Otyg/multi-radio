#include <cstdlib>
#include <optional>
#include <string>

#include <QApplication>
#include <QSettings>

#include "main_window.hpp"

namespace {

std::string GetEnvOrDefault(const char* key, const std::string& fallback) {
  const char* value = std::getenv(key);
  return value == nullptr ? fallback : value;
}

std::optional<std::string> GetConfigPathFromArgs(int argc, char* argv[]) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] == nullptr ? "" : std::string(argv[i]);
    if (arg.rfind("--config=", 0) == 0 && arg.size() > 9) {
      return arg.substr(9);
    }
    if (arg == "--config" && (i + 1) < argc && argv[i + 1] != nullptr) {
      return std::string(argv[i + 1]);
    }
  }
  return std::nullopt;
}

std::string ReadSettingOrFallback(const QSettings& settings, const QString& key,
                                  const std::string& fallback) {
  const QString value = settings.value(key).toString().trimmed();
  if (!value.isEmpty()) {
    return value.toStdString();
  }
  return fallback;
}

}  // namespace

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  const std::string config_path =
      GetConfigPathFromArgs(argc, argv).value_or(GetEnvOrDefault("MR_CLIENT_CONFIG", "client.ini"));
  const QSettings settings(QString::fromStdString(config_path), QSettings::IniFormat);

  const std::string target = ReadSettingOrFallback(
      settings, "grpc_target", ReadSettingOrFallback(settings, "client/grpc_target",
                                                     GetEnvOrDefault("MR_GRPC_TARGET", "127.0.0.1:50051")));
  const std::string token = ReadSettingOrFallback(
      settings, "auth_token", ReadSettingOrFallback(settings, "client/auth_token",
                                                    GetEnvOrDefault("MR_AUTH_TOKEN", "multi-radio-dev-token")));

  multi_radio::MainWindow window(target, token);
  window.show();

  return app.exec();
}
