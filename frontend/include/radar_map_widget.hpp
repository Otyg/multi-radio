#pragma once

#include <cstdint>
#include <deque>
#include <unordered_map>

#include <QColor>
#include <QList>
#include <QMap>
#include <QPainterPath>
#include <QPointF>
#include <QSet>
#include <QString>
#include <QTimer>
#include <QWidget>

#include "coastline_loader.hpp"
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
  void AddCoastlineLoader(CoastlineLoader* loader);  // ownership stays with caller; call once per collection
  void SetCoastlineStatus(const QString& text, bool is_error = false);
  void SetShowFixedNames(bool enabled);
  void SetHideLowSpeed(bool enabled);
  void SetTrailWindowSeconds(double seconds);
  void SetFixedObjects(const std::vector<RadarFixedObject>& fixed);

  void UpsertTarget(const RadarTargetUpdate& update);
  void UpdateTargetLabel(const QString& id, const QString& label);
  void RemoveTarget(const QString& id);
  void ClearTargets();
  void ApplySnapshot(const QVector<RadarTargetUpdate>& targets, const QStringList& removed_ids);
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

  // Coastline support.
  QList<CoastlineLoader*> coastline_loaders_;
  QTimer*                 coastline_debounce_ = nullptr;  // 500 ms one-shot

  // Per-tile storage: key = (lat_floor_deg, lon_floor_deg).
  using TileKey = QPair<int,int>;
  QMap<TileKey, QVector<QPolygonF>> tile_polygons_;  // lat/lon source, per tile
  QMap<TileKey, QPainterPath>       tile_paths_;     // projected screen paths
  QSet<TileKey>                     tiles_building_; // currently being built off-thread

  QString coastline_status_;
  bool    coastline_status_error_ = false;

  void TriggerCoastlineLoad();
  void SchedulePathBuild(TileKey key);
  void RebuildAllPaths();  // re-project all loaded tiles after view change

 private slots:
  void OnTileReady(int lat_deg, int lon_deg, QVector<QPolygonF> polygons);
};

}  // namespace multi_radio
