#pragma once

#include <cstdint>
#include <deque>
#include <unordered_map>

#include <QColor>
#include <QPointF>
#include <QString>
#include <QWidget>

#include "radar_types.hpp"

namespace multi_radio {

class RadarMapWidget : public QWidget {
  Q_OBJECT

 public:
  explicit RadarMapWidget(QWidget* parent = nullptr);

  void SetHome(double lat, double lon);
  void SetCenter(double lat, double lon);
  void SetRangeKm(double range_km);

  void SetShowLabels(bool enabled);
  void SetShowCoastline(bool enabled);
  void SetShowFixedNames(bool enabled);
  void SetHideLowSpeed(bool enabled);
  void SetTrailWindowSeconds(double seconds);
  void SetFixedObjects(const std::vector<RadarFixedObject>& fixed);

  void UpsertTarget(const RadarTargetUpdate& update);
  void UpdateTargetLabel(const QString& id, const QString& label);
  void ClearTargets();
  void SetSelectedTarget(const QString& id);

  QString SelectedTarget() const { return selected_target_id_; }

 signals:
  void TargetSelected(const QString& id);
  void ViewChanged(double center_lat, double center_lon, double range_km);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

 private:
  struct TrailPoint {
    double lat = 0.0;
    double lon = 0.0;
    std::uint64_t unix_ms = 0;
  };

  struct TargetState {
    RadarTargetUpdate last;
    std::deque<TrailPoint> trail;
  };

  QPointF LatLonToXY(double lat, double lon, double cx, double cy, double px_per_km) const;
  void TrimTrails(std::uint64_t now_ms);
  QString PickTargetAt(const QPointF& pos_px) const;

  double home_lat_ = 0.0;
  double home_lon_ = 0.0;
  double center_lat_ = 0.0;
  double center_lon_ = 0.0;
  double range_km_ = 10.0;

  bool show_labels_ = false;
  bool show_coastline_ = true;
  bool show_fixed_names_ = true;
  bool hide_low_speed_ = false;

  QString selected_target_id_;

  std::unordered_map<std::string, TargetState> targets_;
  std::vector<RadarFixedObject> fixed_objects_;

  // Interaction state.
  bool panning_ = false;
  QPoint last_mouse_pos_;

  // Styling.
  QColor bg_color_ = QColor("#001000");
  QColor ring_color_ = QColor("#0f4a0f");
  QColor grid_color_ = QColor("#073007");
  QColor label_color_ = QColor("#9be89b");
  QColor vessel_color_ = QColor("#7fffa0");
  QColor aircraft_color_ = QColor("#a0d8ff");
  QColor selected_color_ = QColor("#ff4d4d");

  std::uint64_t trail_window_ms_ = 120000;  // 120s
};

}  // namespace multi_radio
