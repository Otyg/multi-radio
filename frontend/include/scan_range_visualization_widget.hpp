#pragma once

#include <vector>

#include <QColor>
#include <QVector>
#include <QWidget>

namespace multi_radio {

class ScanRangeVisualizationWidget : public QWidget {
  Q_OBJECT

 public:
  explicit ScanRangeVisualizationWidget(QWidget* parent = nullptr);

  // Call when scan parameters change; resets all accumulated data.
  void Configure(double start_hz, double end_hz, double step_hz, int total_bins);

  // Called for each incoming IQ spectrum frame.
  void PushSpectrum(const std::vector<double>& spectrum, double frame_start_hz,
                    double frame_end_hz, double tuned_hz);
  // Suppress waterfall pixels with normalized value < threshold (0–1).
  void SetNoiseGate(bool enabled, double threshold);

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  static QColor HeatColor(double v);
  static QString FormatFreq(double hz);
  void DrawSpectrum(QPainter* p, const QRect& rect, const QVector<double>& spectrum);
  void DrawWaterfall(QPainter* p, const QRect& rect, const QVector<double>& current_row,
                     const QVector<QVector<double>>& rows);

  double scan_start_hz_ = 0.0;
  double scan_end_hz_ = 0.0;
  double scan_step_hz_ = 0.0;
  int total_bins_ = 0;

  bool noise_gate_enabled_ = false;
  double noise_gate_threshold_ = 0.0;

  QVector<double> sweep_buffer_;
  QVector<double> latest_spectrum_;
  QVector<QVector<double>> waterfall_rows_;
  double last_tuned_hz_ = 0.0;

  static constexpr int kMaxRows = 150;
};

}  // namespace multi_radio
