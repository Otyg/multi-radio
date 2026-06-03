#include "visible_objects_widget.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <QDateTime>
#include <QHeaderView>
#include <QVBoxLayout>

namespace multi_radio {

namespace {

static std::string ToKey(const QString& id) { return id.toStdString(); }

static QString KindLabel(RadarTargetKind kind) {
  switch (kind) {
    case RadarTargetKind::kAircraft:    return "AIR";
    case RadarTargetKind::kVessel:      return "SEA";
    case RadarTargetKind::kFixed:       return "FIX";
    case RadarTargetKind::kSarAircraft: return "SAR";
    default:                            return "?";
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

}  // namespace

VisibleObjectsWidget::VisibleObjectsWidget(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  table_ = new QTableWidget(0, 7, this);
  table_->setHorizontalHeaderLabels({"Kind", "Label", "Dist", "SOG", "COG", "Alt", "Last"});
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);

  layout->addWidget(table_);

  connect(table_, &QTableWidget::cellDoubleClicked, this, [this](int row, int /*col*/) {
    auto* item = table_->item(row, 1);
    if (item == nullptr) return;
    emit TargetActivated(item->data(Qt::UserRole).toString());
  });
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
  struct Row { RadarTargetUpdate t; };
  std::vector<Row> items;
  items.reserve(rows_.size());
  for (const auto& [_, st] : rows_) {
    if (hide_low_speed_ && st.last.kind == RadarTargetKind::kVessel && st.last.sog < 1.0) continue;
    items.push_back(Row{st.last});
  }
  std::sort(items.begin(), items.end(), [](const Row& a, const Row& b) {
    return a.t.unix_ms > b.t.unix_ms;
  });

  table_->setRowCount(static_cast<int>(items.size()));
  for (int i = 0; i < static_cast<int>(items.size()); ++i) {
    const auto& t = items[static_cast<size_t>(i)].t;

    table_->setItem(i, 0, new QTableWidgetItem(KindLabel(t.kind)));

    auto* label_item = new QTableWidgetItem(t.label);
    label_item->setData(Qt::UserRole, t.id);
    table_->setItem(i, 1, label_item);

    QString dist_str;
    if (std::isfinite(t.lat) && std::isfinite(t.lon) && (center_lat_ != 0.0 || center_lon_ != 0.0)) {
      const double km = HaversineKm(center_lat_, center_lon_, t.lat, t.lon);
      dist_str = km < 10.0 ? QString::number(km, 'f', 2) + " km"
                           : QString::number(km, 'f', 1) + " km";
    }
    table_->setItem(i, 2, new QTableWidgetItem(dist_str));
    table_->setItem(i, 3, new QTableWidgetItem(t.sog > 0.0 ? QString::number(t.sog, 'f', 1) : QString()));
    table_->setItem(i, 4, new QTableWidgetItem(t.sog > 0.0 ? QString::number(t.cog, 'f', 1) : QString()));
    table_->setItem(i, 5, new QTableWidgetItem(
        std::isfinite(t.altitude) ? QString::number(static_cast<int>(t.altitude)) + " ft" : QString()));

    const QString last = t.unix_ms > 0
        ? QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(t.unix_ms)).toLocalTime().toString("HH:mm:ss")
        : QString();
    table_->setItem(i, 6, new QTableWidgetItem(last));

    if (!selected_id_.isEmpty() && t.id == selected_id_)
      table_->selectRow(i);
  }
}

}  // namespace multi_radio
