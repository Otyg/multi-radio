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

  void UpsertTarget(const RadarTargetUpdate& update);
  void UpdateTargetLabel(const QString& id, const QString& label);
  void RemoveStale(std::uint64_t now_ms, std::uint64_t stale_after_ms);
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
