#include "radar_map_widget.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QWheelEvent>

namespace multi_radio {

namespace {

constexpr int kRingCount = 5;
constexpr double kKmPerDegLat = 110.574;
constexpr double kMinRangeKm = 0.2;
constexpr double kMaxRangeKm = 500.0;
constexpr double kDefaultTrailWindowSeconds = 120.0;

static std::string ToKey(const QString& id) { return id.toStdString(); }

static bool IsFinite(double v) { return std::isfinite(v); }

static QColor WithAlpha(const QColor& c, double alpha) {
  QColor out = c;
  out.setAlphaF(std::clamp(alpha, 0.0, 1.0));
  return out;
}

}  // namespace

RadarMapWidget::RadarMapWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(500, 400);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMouseTracking(true);
  SetTrailWindowSeconds(kDefaultTrailWindowSeconds);

  coastline_debounce_ = new QTimer(this);
  coastline_debounce_->setSingleShot(true);
  coastline_debounce_->setInterval(500);
  connect(coastline_debounce_, &QTimer::timeout,
          this, &RadarMapWidget::TriggerCoastlineLoad);
}

void RadarMapWidget::SetHome(double lat, double lon) {
  home_lat_ = lat;
  home_lon_ = lon;
  SetCenter(lat, lon);
}

void RadarMapWidget::SetCenter(double lat, double lon) {
  center_lat_ = lat;
  center_lon_ = lon;
  coastline_path_dirty_ = true;
  if (coastline_debounce_) coastline_debounce_->start();
  emit ViewChanged(center_lat_, center_lon_, range_km_);
  update();
}

void RadarMapWidget::SetRangeKm(double range_km) {
  range_km_ = std::clamp(range_km, kMinRangeKm, kMaxRangeKm);
  coastline_path_dirty_ = true;
  if (coastline_debounce_) coastline_debounce_->start();
  emit ViewChanged(center_lat_, center_lon_, range_km_);
  update();
}

void RadarMapWidget::SetCoastlineLoader(CoastlineLoader* loader) {
  if (coastline_loader_) {
    disconnect(coastline_loader_, nullptr, this, nullptr);
  }
  coastline_loader_ = loader;
  if (coastline_loader_) {
    connect(coastline_loader_, &CoastlineLoader::CoastlineReady,
            this, &RadarMapWidget::OnCoastlineReady);
    // Trigger an immediate load for the current view.
    if (coastline_debounce_) coastline_debounce_->start(0);
  }
}

void RadarMapWidget::TriggerCoastlineLoad() {
  if (!coastline_loader_ || !show_coastline_) return;

  // Compute bbox with a 20 % margin so tiles cover slightly beyond the view.
  const double margin = range_km_ * 0.2;
  const double dlat   = (range_km_ + margin) / kKmPerDegLat;
  const double dlon   = dlat / std::max(0.01, std::cos(center_lat_ * (M_PI / 180.0)));

  coastline_loader_->RequestView(center_lat_ - dlat, center_lon_ - dlon,
                                  center_lat_ + dlat, center_lon_ + dlon);
}

void RadarMapWidget::SetCoastlineStatus(const QString& text, bool is_error) {
  coastline_status_       = text;
  coastline_status_error_ = is_error;
  update();
}

void RadarMapWidget::OnCoastlineReady(QVector<QPolygonF> polygons) {
  coastline_polygons_ = std::move(polygons);
  coastline_path_dirty_ = true;
  coastline_status_.clear();
  update();
}

void RadarMapWidget::RebuildCoastlinePath() {
  coastline_path_ = QPainterPath{};
  if (!show_coastline_ || coastline_polygons_.isEmpty()) {
    coastline_path_dirty_ = false;
    return;
  }

  const double radius   = std::min(width(), height()) * 0.48;
  const double px_per_km = radius / std::max(0.001, range_km_);
  const double cx = width()  * 0.5;
  const double cy = height() * 0.5;

  for (const QPolygonF& poly : coastline_polygons_) {
    if (poly.isEmpty()) continue;
    bool first = true;
    for (const QPointF& pt : poly) {
      const QPointF screen = LatLonToXY(pt.x(), pt.y(), cx, cy, px_per_km);
      if (first) { coastline_path_.moveTo(screen); first = false; }
      else        coastline_path_.lineTo(screen);
    }
  }
  coastline_path_dirty_ = false;
}

void RadarMapWidget::SetShowLabels(bool enabled) {
  show_labels_ = enabled;
  update();
}

void RadarMapWidget::SetShowCoastline(bool enabled) {
  show_coastline_ = enabled;
  update();
}

void RadarMapWidget::SetShowFixedNames(bool enabled) {
  show_fixed_names_ = enabled;
  update();
}

void RadarMapWidget::SetHideLowSpeed(bool enabled) {
  hide_low_speed_ = enabled;
  update();
}

void RadarMapWidget::SetTrailWindowSeconds(double seconds) {
  if (!IsFinite(seconds)) seconds = kDefaultTrailWindowSeconds;
  seconds = std::clamp(seconds, 5.0, 3600.0);
  trail_window_ms_ = static_cast<std::uint64_t>(seconds * 1000.0);
  update();
}

void RadarMapWidget::SetFixedObjects(const std::vector<RadarFixedObject>& fixed) {
  fixed_objects_ = fixed;
  update();
}

void RadarMapWidget::ApplySnapshot(const QVector<RadarTargetUpdate>& targets,
                                   const QStringList& removed_ids) {
  for (const QString& rid : removed_ids)
    targets_.erase(rid.toStdString());

  for (const auto& upd : targets) {
    if (upd.id.isEmpty()) continue;
    if (!IsFinite(upd.lat) || !IsFinite(upd.lon)) continue;
    const std::string key = upd.id.toStdString();
    TargetState& state = targets_[key];
    state.last = upd;
    if (upd.unix_ms != 0) {
      state.trail.push_back(TrailPoint{upd.lat, upd.lon, upd.unix_ms});
      TrimTrails(upd.unix_ms);
    }
  }
  update();
}

void RadarMapWidget::RemoveTarget(const QString& id) {
  if (id.isEmpty()) return;
  targets_.erase(id.toStdString());
  update();
}

void RadarMapWidget::ClearTargets() {
  targets_.clear();
  selected_target_id_.clear();
  update();
}

void RadarMapWidget::SetSelectedTarget(const QString& id) {
  selected_target_id_ = id;
  update();
}

void RadarMapWidget::UpsertTarget(const RadarTargetUpdate& update_in) {
  if (update_in.id.isEmpty()) return;
  if (!IsFinite(update_in.lat) || !IsFinite(update_in.lon)) return;

  const std::string key = ToKey(update_in.id);
  TargetState& state = targets_[key];
  state.last = update_in;

  if (update_in.unix_ms != 0) {
    state.trail.push_back(TrailPoint{update_in.lat, update_in.lon, update_in.unix_ms});
    TrimTrails(update_in.unix_ms);
  }
  update();
}

void RadarMapWidget::UpdateTargetLabel(const QString& id, const QString& label) {
  if (id.isEmpty()) return;
  const std::string key = ToKey(id);
  auto it = targets_.find(key);
  if (it == targets_.end()) return;
  if (label.isEmpty()) return;
  it->second.last.label = label;
  update();
}

void RadarMapWidget::TrimTrails(std::uint64_t now_ms) {
  if (trail_window_ms_ == 0) return;
  const std::uint64_t cutoff = (now_ms > trail_window_ms_) ? (now_ms - trail_window_ms_) : 0;
  for (auto& [_, state] : targets_) {
    while (!state.trail.empty() && state.trail.front().unix_ms < cutoff) {
      state.trail.pop_front();
    }
  }
}

QPointF RadarMapWidget::LatLonToXY(double lat, double lon, double cx, double cy, double px_per_km) const {
  const double dlat = lat - center_lat_;
  const double dlon = lon - center_lon_;
  const double km_n = dlat * kKmPerDegLat;
  const double km_e =
      dlon * std::cos(center_lat_ * (M_PI / 180.0)) * kKmPerDegLat;  // good enough for local radar
  return QPointF(cx + (km_e * px_per_km), cy - (km_n * px_per_km));
}

QString RadarMapWidget::PickTargetAt(const QPointF& pos_px) const {
  const double w = width();
  const double h = height();
  const double cx = w * 0.5;
  const double cy = h * 0.5;
  const double radius = std::min(w, h) * 0.48;
  const double px_per_km = radius / std::max(0.001, range_km_);

  QString best;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (const auto& [_, state] : targets_) {
    const auto& t = state.last;
    const QPointF p = LatLonToXY(t.lat, t.lon, cx, cy, px_per_km);
    const double dx = p.x() - pos_px.x();
    const double dy = p.y() - pos_px.y();
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = t.id;
    }
  }
  // 12px click radius.
  if (best_d2 <= (12.0 * 12.0)) return best;
  return QString();
}

void RadarMapWidget::paintEvent(QPaintEvent* /*event*/) {
  const double w = width();
  const double h = height();
  const double cx = w * 0.5;
  const double cy = h * 0.5;
  const double radius = std::min(w, h) * 0.48;
  const double px_per_km = radius / std::max(0.001, range_km_);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), bg_color_);

  // Coastline — rebuild path if view has changed, then draw.
  if (show_coastline_ && !coastline_polygons_.isEmpty()) {
    if (coastline_path_dirty_) RebuildCoastlinePath();
    painter.save();
    painter.setClipRect(rect());
    painter.setPen(QPen(QColor("#2a7a5a"), 1));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(coastline_path_);
    painter.restore();
  }

  // Grid.
  painter.setPen(QPen(grid_color_, 1));
  painter.drawLine(QPointF(cx - radius, cy), QPointF(cx + radius, cy));
  painter.drawLine(QPointF(cx, cy - radius), QPointF(cx, cy + radius));

  painter.setPen(QPen(ring_color_, 1));
  for (int i = 1; i <= kRingCount; ++i) {
    const double r = radius * (static_cast<double>(i) / kRingCount);
    painter.drawEllipse(QPointF(cx, cy), r, r);
  }

  // Ring labels.
  painter.setPen(QPen(WithAlpha(label_color_, 0.75), 1));
  QFont ring_font = painter.font();
  ring_font.setPixelSize(11);
  painter.setFont(ring_font);
  for (int i = 1; i <= kRingCount; ++i) {
    const double km = range_km_ * (static_cast<double>(i) / kRingCount);
    const double r = radius * (static_cast<double>(i) / kRingCount);
    painter.drawText(QPointF(cx + 6.0, cy - r + 12.0), QString("%1 km").arg(km, 0, 'f', 1));
  }

  painter.setPen(QPen(label_color_, 1));
  painter.setBrush(label_color_);
  painter.drawEllipse(QPointF(cx, cy), 2.0, 2.0);

  // Fixed objects.
  if (!fixed_objects_.empty()) {
    QFont fixed_font = painter.font();
    fixed_font.setPixelSize(11);
    painter.setFont(fixed_font);
    painter.setPen(QPen(WithAlpha(label_color_, 0.9), 1));
    painter.setBrush(Qt::NoBrush);
    for (const auto& fixed : fixed_objects_) {
      if (!IsFinite(fixed.lat) || !IsFinite(fixed.lon)) continue;
      const QPointF p = LatLonToXY(fixed.lat, fixed.lon, cx, cy, px_per_km);
      painter.drawEllipse(p, 3.2, 3.2);
      if (show_fixed_names_ && !fixed.name.isEmpty()) {
        painter.drawText(QPointF(p.x() + 7.0, p.y() - 2.0), fixed.name);
      }
    }
  }

  // Trails.
  for (const auto& [_, state] : targets_) {
    const auto& t = state.last;
    if (hide_low_speed_ && t.kind == RadarTargetKind::kVessel && t.sog < 1.0) continue;
    QColor color = (t.kind == RadarTargetKind::kAircraft) ? aircraft_color_ : vessel_color_;
    if (!selected_target_id_.isEmpty() && t.id == selected_target_id_) color = selected_color_;

    // Draw recent trail points with fading alpha.
    if (!state.trail.empty()) {
      const std::uint64_t newest = state.trail.back().unix_ms;
      for (const auto& tp : state.trail) {
        const double age_s = (newest > tp.unix_ms) ? (double)(newest - tp.unix_ms) / 1000.0 : 0.0;
        const double window_s = std::max(1.0, (double)trail_window_ms_ / 1000.0);
        const double a = std::clamp(1.0 - (age_s / window_s), 0.02, 1.0);
        const QColor c = WithAlpha(color, a * 0.55);
        painter.setPen(QPen(c, 1));
        painter.setBrush(c);
        const QPointF p = LatLonToXY(tp.lat, tp.lon, cx, cy, px_per_km);
        painter.drawEllipse(p, 1.4, 1.4);
      }
    }
  }

  // Targets.
  QFont label_font = painter.font();
  label_font.setPixelSize(12);

  for (const auto& [_, state] : targets_) {
    const auto& t = state.last;
    if (hide_low_speed_ && t.kind == RadarTargetKind::kVessel && t.sog < 1.0) continue;
    const QPointF p = LatLonToXY(t.lat, t.lon, cx, cy, px_per_km);

    QColor color = (t.kind == RadarTargetKind::kAircraft) ? aircraft_color_ : vessel_color_;
    if (!selected_target_id_.isEmpty() && t.id == selected_target_id_) color = selected_color_;

    painter.setPen(QPen(color, 1));
    painter.setBrush(color);

    // Simple symbols.
    if (t.kind == RadarTargetKind::kVessel) {
      QPolygonF poly;
      poly << QPointF(p.x(), p.y() - 4) << QPointF(p.x() + 4, p.y()) << QPointF(p.x(), p.y() + 4)
           << QPointF(p.x() - 4, p.y());
      painter.drawPolygon(poly);
    } else {
      painter.drawEllipse(p, 3.2, 3.2);
    }

    if (show_labels_) {
      const QString label = !t.label.isEmpty() ? t.label : t.id;
      painter.setFont(label_font);
      painter.setPen(QPen(label_color_, 1));
      painter.drawText(QPointF(p.x() + 8.0, p.y() - 10.0), label);
    }
  }

  // Range label.
  painter.setPen(QPen(label_color_, 1));
  painter.drawText(QPointF(10.0, 18.0), QString("Range: %1 km").arg(range_km_, 0, 'f', 1));

  // Coastline status overlay (bottom-left).
  if (!coastline_status_.isEmpty()) {
    const QColor status_color = coastline_status_error_
        ? QColor("#ff6060") : QColor("#80d0ff");
    painter.setPen(QPen(status_color, 1));
    QFont sf = painter.font();
    sf.setPixelSize(11);
    painter.setFont(sf);
    painter.drawText(QPointF(10.0, h - 10.0), coastline_status_);
  }
}

void RadarMapWidget::mousePressEvent(QMouseEvent* event) {
  if (event == nullptr) return;
  if (event->button() == Qt::LeftButton) {
    const QString picked = PickTargetAt(event->position());
    if (!picked.isEmpty()) {
      selected_target_id_ = picked;
      emit TargetSelected(picked);
      update();
      return;
    }
  }
  if (event->button() == Qt::RightButton) {
    panning_ = true;
    last_mouse_pos_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    return;
  }
  QWidget::mousePressEvent(event);
}

void RadarMapWidget::mouseMoveEvent(QMouseEvent* event) {
  if (event == nullptr) return;
  if (!panning_) {
    QWidget::mouseMoveEvent(event);
    return;
  }

  const QPoint delta = event->pos() - last_mouse_pos_;
  last_mouse_pos_ = event->pos();

  const double radius = std::min(width(), height()) * 0.48;
  const double km_per_px = range_km_ / std::max(1.0, radius);

  const double km_e = delta.x() * km_per_px;
  const double km_n = -delta.y() * km_per_px;

  const double dlat = km_n / kKmPerDegLat;
  const double dlon =
      km_e / (std::cos(center_lat_ * (M_PI / 180.0)) * kKmPerDegLat + 1e-9);

  center_lat_ -= dlat;
  center_lon_ -= dlon;
  coastline_path_dirty_ = true;
  if (coastline_debounce_) coastline_debounce_->start();
  emit ViewChanged(center_lat_, center_lon_, range_km_);
  update();
}

void RadarMapWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event == nullptr) return;
  if (event->button() == Qt::RightButton && panning_) {
    panning_ = false;
    unsetCursor();
    return;
  }
  QWidget::mouseReleaseEvent(event);
}

void RadarMapWidget::wheelEvent(QWheelEvent* event) {
  if (event == nullptr) return;
  const QPoint angle = event->angleDelta();
  if (angle.y() == 0) return;

  // Zoom with a smooth exponential response.
  const double steps = static_cast<double>(angle.y()) / 120.0;
  const double factor = std::pow(0.88, steps);
  SetRangeKm(range_km_ * factor);
  event->accept();
}

}  // namespace multi_radio
