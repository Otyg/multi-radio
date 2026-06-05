#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <QCoreApplication>
#include <QDateTime>
#include <QSettings>

#include "grpc_client.hpp"
#include "tui_radar_renderer.hpp"

#include <curses.h>

#ifdef OK
#undef OK
#endif

namespace {

using multi_radio::GrpcClient;
using multi_radio::RadarTargetKind;
using multi_radio::RadarTargetUpdate;
using multi_radio::tui::BuildRadarFrame;
using multi_radio::tui::GnuplotRadarRenderer;
using multi_radio::tui::RadarFrame;
using multi_radio::tui::RadarViewConfig;

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

std::optional<double> GetDoubleArg(int argc, char* argv[], const std::string& key) {
  const std::string prefix = key + "=";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] == nullptr ? "" : std::string(argv[i]);
    if (arg.rfind(prefix, 0) == 0) {
      return std::stod(arg.substr(prefix.size()));
    }
    if (arg == key && (i + 1) < argc && argv[i + 1] != nullptr) {
      return std::stod(argv[i + 1]);
    }
  }
  return std::nullopt;
}

std::string ReadSettingOrFallback(const QSettings& settings, const QString& key,
                                  const std::string& fallback) {
  const QString value = settings.value(key).toString().trimmed();
  return value.isEmpty() ? fallback : value.toStdString();
}

std::optional<double> ReadOptionalDouble(const QSettings& settings, const QString& key) {
  bool ok = false;
  const double value = settings.value(key).toDouble(&ok);
  if (!ok) {
    return std::nullopt;
  }
  return value;
}

std::string KindLabel(RadarTargetKind kind) {
  switch (kind) {
    case RadarTargetKind::kAircraft:
      return "AIR";
    case RadarTargetKind::kVessel:
      return "SEA";
    case RadarTargetKind::kFixed:
      return "FIX";
    case RadarTargetKind::kSarAircraft:
      return "SAR";
    case RadarTargetKind::kUnknown:
    default:
      return "UNK";
  }
}

std::string FormatTime(std::uint64_t unix_ms) {
  if (unix_ms == 0) {
    return "";
  }
  return QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(unix_ms))
      .toLocalTime()
      .toString("HH:mm:ss")
      .toStdString();
}

class TuiApp {
 public:
  TuiApp(std::string target, std::string token, RadarViewConfig config)
      : client_(std::move(target), std::move(token)),
        config_(config),
        renderer_(std::filesystem::temp_directory_path() / "multi-radio-tui") {
    QObject::connect(&client_, &GrpcClient::RadarSnapshotReceived, &client_,
                     [this](const QVector<RadarTargetUpdate>& targets, const QStringList& removed_ids, quint64) {
                       for (const QString& id : removed_ids) {
                         targets_.erase(id.toStdString());
                       }
                       for (const auto& target : targets) {
                         targets_[target.id.toStdString()] = target;
                       }
                       if (selected_id_.empty() && !targets_.empty()) {
                         selected_id_ = targets_.begin()->first;
                       }
                     });
    QObject::connect(&client_, &GrpcClient::StreamError, &client_, [this](const QString& error) {
      stream_error_ = error.toStdString();
    });
  }

  int Run() {
    client_.StartStreaming();
    InitScreen();
    const int rc = EventLoop();
    ShutdownScreen();
    client_.StopStreaming();
    return rc;
  }

 private:
  GrpcClient client_;
  RadarViewConfig config_;
  GnuplotRadarRenderer renderer_;
  std::unordered_map<std::string, RadarTargetUpdate> targets_;
  std::string selected_id_;
  std::string stream_error_;
  std::vector<std::string> plot_lines_;

  void InitScreen() {
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(100);
    if (has_colors()) {
      start_color();
      use_default_colors();
      init_pair(1, COLOR_CYAN, -1);
      init_pair(2, COLOR_YELLOW, -1);
      init_pair(3, COLOR_RED, -1);
    }
  }

  void ShutdownScreen() {
    endwin();
  }

  std::vector<RadarTargetUpdate> SnapshotTargets() const {
    std::vector<RadarTargetUpdate> result;
    result.reserve(targets_.size());
    for (const auto& [_, target] : targets_) {
      result.push_back(target);
    }
    return result;
  }

  RadarFrame BuildFrame() {
    return BuildRadarFrame(SnapshotTargets(), config_, selected_id_);
  }

  int EventLoop() {
    using clock = std::chrono::steady_clock;
    auto last_plot = clock::now() - std::chrono::seconds(1);

    while (true) {
      QCoreApplication::processEvents();
      const auto frame = BuildFrame();

      const auto now = clock::now();
      if (now - last_plot > std::chrono::milliseconds(250)) {
        int rows = 0;
        int cols = 0;
        getmaxyx(stdscr, rows, cols);
        const int radar_width = std::max(20, cols * 2 / 3);
        const int radar_height = std::max(10, rows - 2);
        plot_lines_ = renderer_.Render(frame, radar_width, radar_height);
        last_plot = now;
      }

      Draw(frame);

      const int ch = getch();
      if (ch == ERR) {
        continue;
      }
      if (HandleInput(ch, frame) != 0) {
        return 0;
      }
    }
  }

  int HandleInput(int ch, const RadarFrame& frame) {
    switch (ch) {
      case 'q':
      case 'Q':
        return 1;
      case 'f':
      case 'F':
        config_.slow_only = !config_.slow_only;
        return 0;
      case '+':
      case '=':
        config_.range_km = std::max(0.5, config_.range_km / 1.25);
        return 0;
      case '-':
      case '_':
        config_.range_km = std::min(500.0, config_.range_km * 1.25);
        return 0;
      case KEY_UP:
      case 'k':
      case 'K':
        MoveSelection(frame, -1);
        return 0;
      case KEY_DOWN:
      case 'j':
      case 'J':
        MoveSelection(frame, 1);
        return 0;
      default:
        return 0;
    }
  }

  void MoveSelection(const RadarFrame& frame, int delta) {
    if (frame.targets.empty()) {
      selected_id_.clear();
      return;
    }

    int index = 0;
    for (int i = 0; i < static_cast<int>(frame.targets.size()); ++i) {
      if (frame.targets[static_cast<size_t>(i)].target.id.toStdString() == selected_id_) {
        index = i;
        break;
      }
    }
    index = (index + delta + static_cast<int>(frame.targets.size())) % static_cast<int>(frame.targets.size());
    selected_id_ = frame.targets[static_cast<size_t>(index)].target.id.toStdString();
  }

  void Draw(const RadarFrame& frame) {
    erase();
    int rows = 0;
    int cols = 0;
    getmaxyx(stdscr, rows, cols);

    const int status_row = 0;
    const int command_row = std::max(1, rows - 1);
    const int body_top = 1;
    const int body_height = std::max(1, rows - 2);
    const int radar_width = std::max(20, cols * 2 / 3);
    const int list_left = std::min(cols - 1, radar_width + 1);
    const int list_width = std::max(1, cols - list_left);

    DrawStatus(frame, status_row, cols);
    DrawRadarPane(body_top, radar_width, body_height);
    DrawTargetList(frame, body_top, list_left, list_width, body_height);
    DrawCommandBar(command_row, cols);
    refresh();
  }

  void DrawStatus(const RadarFrame& frame, int row, int cols) {
    mvhline(row, 0, ' ', cols);
    const std::string center_mode = frame.using_auto_center ? "AUTO" : "FIXED";
    const std::string line =
        "multi_radio_tui  targets=" + std::to_string(frame.targets.size()) +
        "  filter=" + std::string(config_.slow_only ? "<1kn" : "all") +
        "  range=" + std::to_string(static_cast<int>(std::round(frame.range_km))) + "km" +
        "  center=" + center_mode +
        (stream_error_.empty() ? "" : "  stream=" + stream_error_);
    mvaddnstr(row, 0, line.c_str(), cols - 1);
  }

  void DrawRadarPane(int top, int width, int height) {
    for (int i = 0; i < height; ++i) {
      const int row = top + i;
      if (i < static_cast<int>(plot_lines_.size())) {
        mvaddnstr(row, 0, plot_lines_[static_cast<size_t>(i)].c_str(), width);
      } else {
        mvhline(row, 0, ' ', width);
      }
    }
  }

  void DrawTargetList(const RadarFrame& frame, int top, int left, int width, int height) {
    if (width <= 1) {
      return;
    }
    mvaddnstr(top, left, "Targets", width - 1);
    for (int i = 1; i < height; ++i) {
      const int idx = i - 1;
      const int row = top + i;
      mvhline(row, left, ' ', width - 1);
      if (idx >= static_cast<int>(frame.targets.size())) {
        continue;
      }

      const auto& target = frame.targets[static_cast<size_t>(idx)];
      const std::string line =
          KindLabel(target.target.kind) + " " +
          target.target.label.toStdString() +
          "  " + std::to_string(static_cast<int>(std::round(target.range_km))) + "km" +
          "  " + std::to_string(target.target.sog).substr(0, 4) + "kn" +
          "  " + FormatTime(target.target.unix_ms);

      if (target.selected) {
        attron(A_REVERSE);
      }
      mvaddnstr(row, left, line.c_str(), width - 1);
      if (target.selected) {
        attroff(A_REVERSE);
      }
    }
  }

  void DrawCommandBar(int row, int cols) {
    mvhline(row, 0, ' ', cols);
    const std::string line = "q quit  f toggle slow filter  j/k select  +/- zoom";
    mvaddnstr(row, 0, line.c_str(), cols - 1);
  }
};

}  // namespace

int main(int argc, char* argv[]) {
  QCoreApplication app(argc, argv);
  app.setOrganizationName("multi-radio");
  app.setApplicationName("multi-radio-tui");

  const std::string config_path =
      GetConfigPathFromArgs(argc, argv).value_or(GetEnvOrDefault("MR_CLIENT_CONFIG", "client.ini"));
  const QSettings settings(QString::fromStdString(config_path), QSettings::IniFormat);

  const std::string target = ReadSettingOrFallback(
      settings, "grpc_target", ReadSettingOrFallback(settings, "client/grpc_target",
                                                     GetEnvOrDefault("MR_GRPC_TARGET", "127.0.0.1:50051")));
  const std::string token = ReadSettingOrFallback(
      settings, "auth_token", ReadSettingOrFallback(settings, "client/auth_token",
                                                    GetEnvOrDefault("MR_AUTH_TOKEN", "multi-radio-dev-token")));

  RadarViewConfig config;
  config.slow_only = settings.value("radar_view/hide_low_speed", true).toBool();
  config.range_km = std::clamp(settings.value("radar_view/range_km", 10.0).toDouble(), 0.5, 500.0);

  const auto center_lat_arg = GetDoubleArg(argc, argv, "--center-lat");
  const auto center_lon_arg = GetDoubleArg(argc, argv, "--center-lon");
  const auto center_lat_cfg = ReadOptionalDouble(settings, "radar_view/center_lat");
  const auto center_lon_cfg = ReadOptionalDouble(settings, "radar_view/center_lon");
  if (center_lat_arg && center_lon_arg) {
    config.center_lat = *center_lat_arg;
    config.center_lon = *center_lon_arg;
    config.have_fixed_center = true;
  } else if (center_lat_cfg && center_lon_cfg) {
    config.center_lat = *center_lat_cfg;
    config.center_lon = *center_lon_cfg;
    config.have_fixed_center = true;
  }

  TuiApp tui(std::move(target), std::move(token), config);
  return tui.Run();
}
