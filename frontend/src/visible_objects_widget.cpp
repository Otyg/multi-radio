#include "visible_objects_widget.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <QHeaderView>
#include <QVBoxLayout>

namespace multi_radio {

namespace {

static std::string ToKey(const QString& id) { return id.toStdString(); }

static QString KindLabel(RadarTargetKind kind) {
  switch (kind) {
    case RadarTargetKind::kAircraft:
      return "AIR";
    case RadarTargetKind::kVessel:
      return "SEA";
    case RadarTargetKind::kFixed:
      return "FIX";
    case RadarTargetKind::kUnknown:
    default:
      return "?";
  }
}

}  // namespace

VisibleObjectsWidget::VisibleObjectsWidget(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  table_ = new QTableWidget(0, 8, this);
  table_->setHorizontalHeaderLabels({"Kind", "ID", "Label", "Lat", "Lon", "SOG", "COG", "Alt"});
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);

  layout->addWidget(table_);

  connect(table_, &QTableWidget::cellDoubleClicked, this, [this](int row, int /*col*/) {
    auto* item = table_->item(row, 1);
    if (item == nullptr) return;
    emit TargetActivated(item->text());
  });
}

void VisibleObjectsWidget::UpsertTarget(const RadarTargetUpdate& update) {
  if (update.id.isEmpty()) return;
  rows_[ToKey(update.id)].last = update;
  RefreshTable();
}

void VisibleObjectsWidget::UpdateTargetLabel(const QString& id, const QString& label) {
  if (id.isEmpty() || label.isEmpty()) return;
  auto it = rows_.find(ToKey(id));
  if (it == rows_.end()) return;
  it->second.last.label = label;
  RefreshTable();
}

void VisibleObjectsWidget::UpdateTargetAltitude(const QString& id, double altitude_ft) {
  if (id.isEmpty()) return;
  auto it = rows_.find(ToKey(id));
  if (it == rows_.end()) return;
  it->second.last.altitude = altitude_ft;
  RefreshTable();
}

void VisibleObjectsWidget::UpdateTargetSogCog(const QString& id, double sog_kn, double cog_deg) {
  if (id.isEmpty()) return;
  auto it = rows_.find(ToKey(id));
  if (it == rows_.end()) return;
  it->second.last.sog = sog_kn;
  it->second.last.cog = cog_deg;
  RefreshTable();
}

void VisibleObjectsWidget::RemoveStale(std::uint64_t now_ms, std::uint64_t stale_after_ms) {
  if (stale_after_ms == 0) return;
  for (auto it = rows_.begin(); it != rows_.end();) {
    const auto& last = it->second.last;
    if (last.unix_ms != 0 && now_ms > last.unix_ms && (now_ms - last.unix_ms) > stale_after_ms) {
      it = rows_.erase(it);
    } else {
      ++it;
    }
  }
  RefreshTable();
}

void VisibleObjectsWidget::SetSelectedTarget(const QString& id) {
  selected_id_ = id;
  RefreshTable();
}

void VisibleObjectsWidget::RefreshTable() {
  struct Row {
    RadarTargetUpdate t;
  };
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
    table_->setItem(i, 1, new QTableWidgetItem(t.id));
    table_->setItem(i, 2, new QTableWidgetItem(t.label));
    table_->setItem(i, 3, new QTableWidgetItem(std::isfinite(t.lat) ? QString::number(t.lat, 'f', 5) : QString()));
    table_->setItem(i, 4, new QTableWidgetItem(std::isfinite(t.lon) ? QString::number(t.lon, 'f', 5) : QString()));
    table_->setItem(i, 5, new QTableWidgetItem(QString::number(t.sog, 'f', 1)));
    table_->setItem(i, 6, new QTableWidgetItem(QString::number(t.cog, 'f', 1)));
    table_->setItem(i, 7, new QTableWidgetItem(
        std::isfinite(t.altitude) ? QString::number(static_cast<int>(t.altitude)) + " ft" : QString()));

    if (!selected_id_.isEmpty() && t.id == selected_id_) {
      table_->selectRow(i);
    }
  }
}

}  // namespace multi_radio
