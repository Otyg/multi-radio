#pragma once

#include <cstdint>
#include <unordered_map>

#include <QString>
#include <QTableWidget>
#include <QWidget>

#include "radar_types.hpp"

namespace multi_radio {

class VisibleObjectsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit VisibleObjectsWidget(QWidget* parent = nullptr);

  // Replace the full target list from a backend snapshot.
  void ApplySnapshot(const QVector<RadarTargetUpdate>& targets, const QStringList& removed_ids);
  void SetSelectedTarget(const QString& id);
  void SetHideLowSpeed(bool enabled) { hide_low_speed_ = enabled; RefreshTable(); }

 signals:
  void TargetActivated(const QString& id);

 private:
  struct RowState {
    RadarTargetUpdate last;
  };

  void RefreshTable();

  QTableWidget* table_ = nullptr;
  std::unordered_map<std::string, RowState> rows_;
  QString selected_id_;
  bool hide_low_speed_ = false;
};

}  // namespace multi_radio
