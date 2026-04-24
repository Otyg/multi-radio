#include <cstdlib>
#include <string>

#include <QApplication>

#include "main_window.hpp"

namespace {

std::string GetEnvOrDefault(const char* key, const std::string& fallback) {
  const char* value = std::getenv(key);
  return value == nullptr ? fallback : value;
}

}  // namespace

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  const std::string target = GetEnvOrDefault("MR_GRPC_TARGET", "127.0.0.1:50051");
  const std::string token = GetEnvOrDefault("MR_AUTH_TOKEN", "multi-radio-dev-token");

  multi_radio::MainWindow window(target, token);
  window.show();

  return app.exec();
}
