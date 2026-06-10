#pragma once

#include <cstdint>
#include <unordered_map>

#include <QString>
#include <QGridLayout>
#include <QPushButton>
#include <QScrollArea>
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
  void SetCenter(double lat, double lon) { center_lat_ = lat; center_lon_ = lon; RefreshTable(); }

 signals:
  void TargetActivated(const QString& id);

 private:
  struct RowState {
    RadarTargetUpdate last;
    double prev_sog = 0.0;
    double prev_alt = std::numeric_limits<double>::quiet_NaN();
  };

  void RefreshTable();

  QScrollArea* scroll_area_ = nullptr;
  QWidget* cards_container_ = nullptr;
  QGridLayout* cards_layout_ = nullptr;
  std::unordered_map<std::string, RowState> rows_;
  QString selected_id_;
  bool hide_low_speed_ = false;
  double center_lat_ = 0.0;
  double center_lon_ = 0.0;
};

}  // namespace multi_radio
