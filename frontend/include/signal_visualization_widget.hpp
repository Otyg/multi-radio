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
  enum class SpectrumSource {
    kDemodulated = 0,
    kReceiverInput = 1,
  };

  explicit SignalVisualizationWidget(QWidget* parent = nullptr);

  void SetKnownReceivers(const std::vector<uint32_t>& receiver_ids);
  void SetReceiverFilter(int receiver_filter_id);
  void SetVisualizationSettings(int fft_size, double frequency_start_hz, double frequency_end_hz);
  void SetSpectrumSource(SpectrumSource source);
  void SetChannelLabel(const QString& label);
  void SetReceiverSquelchThresholdDb(uint32_t receiver_id, double threshold_db);
  void SetReceiverSignalLevelDb(uint32_t receiver_id, double signal_level_db);
  void SetReceiverIqHealth(uint32_t receiver_id, double psd_peak_db, double psd_floor_db, double snr_db,
                           double psd_peak_offset_hz, bool has_quality_score,
                           double quality_score_pct, bool has_signal_ok, bool signal_ok);
  void SetAutoNoiseReductionEnabled(bool enabled);
  void SetNoiseFloorFilterEnabled(bool enabled);
  void SetNoiseFloorDb(double noise_floor_db);
  bool AutoNoiseReductionEnabled() const;
  bool NoiseFloorFilterEnabled() const;
  double NoiseFloorDb() const;
  SpectrumSource CurrentSpectrumSource() const;
  int FftSize() const;
  double FrequencyStartHz() const;
  double FrequencyEndHz() const;
  void PushVisualizationFrame(uint32_t receiver_id, const std::vector<double>& waveform,
                              const std::vector<double>& spectrum, double peak_frequency_hz, double peak_intensity,
                              SpectrumSource source, double frame_frequency_start_hz,
                              double frame_frequency_end_hz);
  void PushSample(uint32_t receiver_id, double frequency_hz, double intensity);

 protected:
  void paintEvent(QPaintEvent* event) override;

 private slots:
  void OnFrameTick();

 private:
  struct DisplayState {
    QVector<double> waveform;
    QVector<double> spectrum;
    QVector<QVector<double>> spectrogram_rows;
    QVector<QVector<double>> waterfall_rows;
    double signal_level = 0.0;
    double signal_peak_hold = 0.0;
    double frequency_start_hz = 0.0;
    double frequency_end_hz = 20000.0;
    bool has_signal_level_db = false;
    double signal_level_db = -120.0;
    bool has_squelch_threshold_db = false;
    double squelch_threshold_db = -67.5;
    bool has_iq_health = false;
    double psd_peak_db = -120.0;
    double psd_floor_db = -120.0;
    double snr_db = 0.0;
    double psd_peak_offset_hz = 0.0;
    bool has_quality_score = false;
    double quality_score_pct = 0.0;
    bool has_signal_ok = false;
    bool signal_ok = false;
  };
  struct ReceiverState {
    QVector<double> waveform;
    QVector<double> spectrum;
    QVector<QVector<double>> spectrogram_rows;
    QVector<QVector<double>> waterfall_rows;
    double last_frequency_hz = 0.0;
    double signal_level = 0.0;
    double signal_peak_hold = 0.0;
    QVector<double> receiver_spectrum;
    QVector<QVector<double>> receiver_spectrogram_rows;
    QVector<QVector<double>> receiver_waterfall_rows;
    double receiver_frequency_start_hz = 0.0;
    double receiver_frequency_end_hz = 20000.0;
    bool receiver_frequency_range_valid = false;
    bool has_signal_level_db = false;
    double signal_level_db = -120.0;
    bool has_squelch_threshold_db = false;
    double squelch_threshold_db = -67.5;
    bool has_iq_health = false;
    double psd_peak_db = -120.0;
    double psd_floor_db = -120.0;
    double snr_db = 0.0;
    double psd_peak_offset_hz = 0.0;
    bool has_quality_score = false;
    double quality_score_pct = 0.0;
    bool has_signal_ok = false;
    bool signal_ok = false;
  };

  void EnsureState(ReceiverState* state) const;
  void ReinitializeState(ReceiverState* state) const;
  static int NormalizeFftSize(int fft_size);
  static int SpectrumBinsFromFftSize(int fft_size);
  static void PushRow(QVector<QVector<double>>* rows, const QVector<double>& row, int max_rows);

  static QColor HeatColor(double value);
  static QColor RainbowColor(double value);
  static void DrawWaveform(QPainter* painter, const QRect& area, const QVector<double>& waveform);
  static void DrawLevelMeter(QPainter* painter, const QRect& area, double level, double peak_hold,
                             bool has_signal_level_db, double signal_level_db, bool has_iq_health,
                             double psd_peak_db, double psd_floor_db, double snr_db,
                             double psd_peak_offset_hz, bool has_quality_score, double quality_score_pct,
                             bool has_signal_ok, bool signal_ok);
  static void DrawSpectrumCurve(QPainter* painter, const QRect& area, const QVector<double>& spectrum,
                                double frequency_start_hz, double frequency_end_hz,
                                bool suppress_below_mean, bool has_squelch_threshold_db,
                                double squelch_threshold_db, bool has_noise_floor_threshold_db,
                                double noise_floor_threshold_db);
  static void DrawHeatmap(QPainter* painter, const QRect& area, const QVector<QVector<double>>& rows,
                          bool newest_at_top, bool rainbow_colors, bool suppress_below_mean,
                          bool has_noise_floor_threshold_db, double noise_floor_threshold_db);

  bool RequireExplicitSelection() const;
  DisplayState BuildDisplayState() const;

  void BlendSampleIntoState(ReceiverState* state, double frequency_hz, double intensity);
  void BlendFrameIntoState(ReceiverState* state, const std::vector<double>& waveform,
                           const std::vector<double>& spectrum, double peak_frequency_hz,
                           double peak_intensity);
  void BlendReceiverSpectrumIntoState(ReceiverState* state, const std::vector<double>& spectrum,
                                      double peak_frequency_hz, double peak_intensity,
                                      double frame_frequency_start_hz, double frame_frequency_end_hz);
  void DecayState(ReceiverState* state, double decay_factor);

  QHash<uint32_t, ReceiverState> states_;
  std::vector<uint32_t> known_receivers_;
  int receiver_filter_id_ = -1;
  QString channel_label_;
  int fft_size_ = 256;
  int spectrum_bins_ = 128;
  double frequency_start_hz_ = 0.0;
  double frequency_end_hz_ = 20000.0;
  SpectrumSource spectrum_source_ = SpectrumSource::kDemodulated;
  bool auto_noise_reduction_enabled_ = false;
  bool noise_floor_filter_enabled_ = false;
  double noise_floor_db_ = -30.0;
  QTimer frame_timer_;
};

}  // namespace multi_radio
