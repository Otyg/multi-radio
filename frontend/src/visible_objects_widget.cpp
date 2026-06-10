#include "visible_objects_widget.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <QDateTime>
#include <QFrame>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QVBoxLayout>

namespace multi_radio {

namespace {

static std::string ToKey(const QString& id) { return id.toStdString(); }

static QString KindIcon(RadarTargetKind kind) {
  switch (kind) {
    case RadarTargetKind::kAircraft:
    case RadarTargetKind::kSarAircraft:
      return "✈";
    case RadarTargetKind::kVessel:
      return "⛵";
    case RadarTargetKind::kFixed:
      return "📍";
    default:
      return "•";
  }
}

static QString KindLabel(RadarTargetKind kind) {
  switch (kind) {
    case RadarTargetKind::kAircraft:    return "Aircraft";
    case RadarTargetKind::kVessel:      return "Vessel";
    case RadarTargetKind::kFixed:       return "Fixed";
    case RadarTargetKind::kSarAircraft: return "SAR";
    default:                            return "Unknown";
  }
}

static double HaversineKm(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kR = 6371.0;
  constexpr double kDeg = M_PI / 180.0;
  const double dlat = (lat2 - lat1) * kDeg;
  const double dlon = (lon2 - lon1) * kDeg;
  const double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(lat1 * kDeg) * std::cos(lat2 * kDeg) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);
  return 2.0 * kR * std::asin(std::sqrt(a));
}

static double BearingDegrees(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kDeg = M_PI / 180.0;
  const double dlon = (lon2 - lon1) * kDeg;
  const double lat1r = lat1 * kDeg;
  const double lat2r = lat2 * kDeg;
  const double y = std::sin(dlon) * std::cos(lat2r);
  const double x = std::cos(lat1r) * std::sin(lat2r) -
                   std::sin(lat1r) * std::cos(lat2r) * std::cos(dlon);
  const double bearing = std::atan2(y, x) / kDeg;
  return std::fmod(bearing + 360.0, 360.0);
}

}  // namespace

VisibleObjectsWidget::VisibleObjectsWidget(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(6);

  scroll_area_ = new QScrollArea(this);
  scroll_area_->setFrameShape(QFrame::NoFrame);
  scroll_area_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll_area_->setWidgetResizable(true);

  cards_container_ = new QWidget(scroll_area_);
  cards_layout_ = new QGridLayout(cards_container_);
  cards_layout_->setContentsMargins(4, 4, 4, 4);
  cards_layout_->setHorizontalSpacing(4);
  cards_layout_->setVerticalSpacing(2);
  cards_container_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

  scroll_area_->setWidget(cards_container_);
  layout->addWidget(scroll_area_);
}

void VisibleObjectsWidget::ApplySnapshot(const QVector<RadarTargetUpdate>& targets,
                                         const QStringList& removed_ids) {
  for (const QString& rid : removed_ids)
    rows_.erase(ToKey(rid));

  for (const auto& t : targets)
    rows_[ToKey(t.id)].last = t;

  RefreshTable();
}

void VisibleObjectsWidget::SetSelectedTarget(const QString& id) {
  selected_id_ = id;
  RefreshTable();
}

void VisibleObjectsWidget::RefreshTable() {
  struct Row { RadarTargetUpdate t; double dist_km; double bearing_deg; };
  std::vector<Row> items;
  items.reserve(rows_.size());
  const bool have_center = (center_lat_ != 0.0 || center_lon_ != 0.0);
  for (const auto& [_, st] : rows_) {
    if (hide_low_speed_ && st.last.kind == RadarTargetKind::kVessel && st.last.sog < 1.0) continue;
    double dist = std::numeric_limits<double>::infinity();
    double bearing = 0.0;
    if (have_center && std::isfinite(st.last.lat) && std::isfinite(st.last.lon)) {
      dist = HaversineKm(center_lat_, center_lon_, st.last.lat, st.last.lon);
      bearing = BearingDegrees(center_lat_, center_lon_, st.last.lat, st.last.lon);
    }
    items.push_back(Row{st.last, dist, bearing});
  }
  std::sort(items.begin(), items.end(), [](const Row& a, const Row& b) {
    return a.dist_km < b.dist_km;
  });

  while (cards_layout_ != nullptr && cards_layout_->count() > 0) {
    QLayoutItem* item = cards_layout_->takeAt(0);
    if (item != nullptr) {
      if (item->widget() != nullptr) item->widget()->deleteLater();
      delete item;
    }
  }

  if (items.empty()) {
    auto* empty = new QLabel("No active objects", cards_container_);
    empty->setAlignment(Qt::AlignCenter);
    empty->setStyleSheet("color: #8FA7BE; padding: 12px;");
    cards_layout_->addWidget(empty, 0, 0);
    return;
  }

  for (int i = 0; i < static_cast<int>(items.size()); ++i) {
    const auto& row = items[static_cast<size_t>(i)];
    const auto& t = row.t;

    auto* card = new QPushButton(cards_container_);
    card->setCheckable(true);
    card->setChecked(!selected_id_.isEmpty() && t.id == selected_id_);
    card->setCursor(Qt::PointingHandCursor);
    card->setMinimumHeight(88);
    card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    card->setStyleSheet(
        "QPushButton { text-align: left; padding: 6px 8px; border-radius: 8px; "
        "border: 1px solid #2E7D32; background: #0B1018; color: #8FA7BE; }"
        "QPushButton:hover { background: #121E2E; border-color: #5CDB95; }"
        "QPushButton:checked { background: #10231A; border-color: #5CDB95; color: #DDFBE6; }");

    const QString dist_text = std::isfinite(row.dist_km)
        ? QString("%1 km @ %2°").arg(row.dist_km < 10.0
                                         ? QString::number(row.dist_km, 'f', 2)
                                         : QString::number(row.dist_km, 'f', 1))
                                .arg(static_cast<int>(std::round(row.bearing_deg)))
        : QString("Distance unknown");
    const QString sog_text = t.sog > 0.0 ? QString("SOG %1 kn").arg(t.sog, 0, 'f', 1) : QString("SOG —");
    const QString cog_text = t.sog > 0.0 ? QString("COG %1°").arg(t.cog, 0, 'f', 1) : QString("COG —");
    const QString alt_text = std::isfinite(t.altitude) ? QString("Alt %1 ft").arg(static_cast<int>(t.altitude)) : QString("Alt —");
    const QString last_text = t.unix_ms > 0
        ? QString("Last %1").arg(QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(t.unix_ms)).toLocalTime().toString("HH:mm:ss"))
        : QString("Last —");

    card->setText(QString("%1 %2 — %3\n%4  %5  %6\n%7")
                      .arg(KindIcon(t.kind))
                      .arg(t.label.trimmed().isEmpty() ? QString("<unnamed>") : t.label.trimmed())
                      .arg(dist_text)
                      .arg(sog_text)
                      .arg(cog_text)
                      .arg(alt_text)
                      .arg(last_text));

    connect(card, &QPushButton::clicked, this, [this, id = t.id]() {
      emit TargetActivated(id);
    });

    cards_layout_->addWidget(card, i, 0);
  }
}

}  // namespace multi_radio
