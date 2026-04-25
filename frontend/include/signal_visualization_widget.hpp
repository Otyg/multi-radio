#pragma once

#include <cstdint>
#include <vector>

#include <QColor>
#include <QHash>
#include <QRect>
#include <QTimer>
#include <QVector>
#include <QWidget>

class QPainter;

namespace multi_radio {

class SignalVisualizationWidget : public QWidget {
  Q_OBJECT

 public:
  explicit SignalVisualizationWidget(QWidget* parent = nullptr);

  void SetKnownReceivers(const std::vector<uint32_t>& receiver_ids);
  void SetReceiverFilter(int receiver_filter_id);
  void PushSample(uint32_t receiver_id, double frequency_hz, double intensity);

 protected:
  void paintEvent(QPaintEvent* event) override;

 private slots:
  void OnFrameTick();

 private:
  struct ReceiverState {
    QVector<double> waveform;
    QVector<double> spectrum;
    QVector<QVector<double>> spectrogram_rows;
    QVector<QVector<double>> waterfall_rows;
    double phase = 0.0;
    double last_frequency_hz = 0.0;
  };

  static void EnsureState(ReceiverState* state);
  static void PushRow(QVector<QVector<double>>* rows, const QVector<double>& row, int max_rows);

  static QColor HeatColor(double value);
  static QColor RainbowColor(double value);
  static void DrawWaveform(QPainter* painter, const QRect& area, const QVector<double>& waveform);
  static void DrawSpectrumCurve(QPainter* painter, const QRect& area, const QVector<double>& spectrum);
  static void DrawHeatmap(QPainter* painter, const QRect& area, const QVector<QVector<double>>& rows,
                          bool newest_at_top, bool rainbow_colors);

  bool RequireExplicitSelection() const;
  ReceiverState BuildDisplayState() const;

  void BlendSampleIntoState(ReceiverState* state, double frequency_hz, double intensity);
  void DecayState(ReceiverState* state, double decay_factor);

  QHash<uint32_t, ReceiverState> states_;
  std::vector<uint32_t> known_receivers_;
  int receiver_filter_id_ = -1;
  QTimer frame_timer_;
};

}  // namespace multi_radio
