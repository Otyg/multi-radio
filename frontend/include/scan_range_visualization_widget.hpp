#pragma once

#include <vector>

#include <QColor>
#include <QMouseEvent>
#include <QVector>
#include <QWidget>

namespace multi_radio {

class ScanRangeVisualizationWidget : public QWidget {
  Q_OBJECT

 public:
  explicit ScanRangeVisualizationWidget(QWidget* parent = nullptr);

  void Configure(double start_hz, double end_hz, double step_hz, int total_bins);
  void PushSpectrum(const std::vector<double>& spectrum, double frame_start_hz,
                    double frame_end_hz, double tuned_hz);
  void SetNoiseGate(bool enabled, double threshold);
  void SetDbCeiling(double ceiling_db);

 signals:
  void RangeSelected(double start_hz, double end_hz);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;

 private:
  static QColor HeatColor(double v);
  static QString FormatFreq(double hz);
  void DrawSpectrum(QPainter* p, const QRect& rect, const QVector<double>& spectrum);
  void DrawWaterfall(QPainter* p, const QRect& rect, const QVector<double>& current_row,
                     const QVector<QVector<double>>& rows);
  QRect SpectrumPlotRect() const;

  double scan_start_hz_ = 0.0;
  double scan_end_hz_ = 0.0;
  double scan_step_hz_ = 0.0;
  int total_bins_ = 0;

  bool noise_gate_enabled_ = false;
  double noise_gate_threshold_ = 0.0;
  double db_ceiling_ = -20.0;

  double pending_range_start_hz_ = -1.0;

  QVector<double> sweep_buffer_;
  QVector<double> latest_spectrum_;
  QVector<QVector<double>> waterfall_rows_;
  double last_tuned_hz_ = 0.0;

  static constexpr int kMaxRows = 150;
  static constexpr int kLabelW   = 38;
  static constexpr int kFreqAxisH = 14;
};

}  // namespace multi_radio
