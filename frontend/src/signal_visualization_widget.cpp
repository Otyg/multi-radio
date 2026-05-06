#include "signal_visualization_widget.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <QColor>
#include <QFontMetrics>
#include <QLinearGradient>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QRect>
#include <QString>

namespace multi_radio {

namespace {

constexpr int kWaveformPoints = 200;
constexpr int kSpectrogramRows = 34;
constexpr int kWaterfallRows = 90;
constexpr double kWaveformWindowSeconds = 10.0;

inline double Clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

QVector<double> ResampleToSize(const std::vector<double>& source, int target_size) {
  if (target_size <= 0) {
    return {};
  }
  QVector<double> out(target_size, 0.0);
  if (source.empty()) {
    return out;
  }
  if (source.size() == 1) {
    const double value = Clamp01(source.front());
    for (double& item : out) {
      item = value;
    }
    return out;
  }

  const int source_size = static_cast<int>(source.size());
  for (int i = 0; i < target_size; ++i) {
    const double t = (target_size <= 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(target_size - 1);
    const double source_pos = t * static_cast<double>(source_size - 1);
    const int left = std::clamp(static_cast<int>(std::floor(source_pos)), 0, source_size - 1);
    const int right = std::clamp(left + 1, 0, source_size - 1);
    const double frac = source_pos - static_cast<double>(left);
    const double value = source[left] + (source[right] - source[left]) * frac;
    out[i] = Clamp01(value);
  }
  return out;
}

double ComputeWaveformSamplePoint(const std::vector<double>& waveform) {
  if (waveform.empty()) {
    return 0.5;
  }
  const size_t idx = waveform.size() - 1;
  return Clamp01(waveform[idx]);
}

double ComputeWaveformActivity(const std::vector<double>& waveform) {
  if (waveform.empty()) {
    return 0.0;
  }
  double sum_abs = 0.0;
  for (const double sample : waveform) {
    sum_abs += std::abs(Clamp01(sample) - 0.5);
  }
  const double avg_abs = sum_abs / static_cast<double>(waveform.size());
  return Clamp01(avg_abs * 2.0);
}

QString FormatFrequencyLabel(double hz) {
  const double abs_hz = std::abs(hz);
  if (abs_hz >= 1e6) {
    return QString("%1M").arg(hz / 1e6, 0, 'f', 3);
  }
  if (abs_hz >= 1e3) {
    return QString("%1k").arg(hz / 1e3, 0, 'f', 1);
  }
  return QString::number(hz, 'f', 0);
}

}  // namespace

SignalVisualizationWidget::SignalVisualizationWidget(QWidget* parent) : QWidget(parent) {
  setMinimumHeight(300);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  frame_timer_.setInterval(90);
  connect(&frame_timer_, &QTimer::timeout, this, &SignalVisualizationWidget::OnFrameTick);
  frame_timer_.start();
}

void SignalVisualizationWidget::SetKnownReceivers(const std::vector<uint32_t>& receiver_ids) {
  known_receivers_ = receiver_ids;
  update();
}

void SignalVisualizationWidget::SetReceiverFilter(int receiver_filter_id) {
  receiver_filter_id_ = receiver_filter_id;
  update();
}

void SignalVisualizationWidget::SetVisualizationSettings(int fft_size, double frequency_start_hz,
                                                         double frequency_end_hz) {
  const int normalized_fft = NormalizeFftSize(fft_size);
  const int new_bins = SpectrumBinsFromFftSize(normalized_fft);
  double start_hz = frequency_start_hz;
  double end_hz = frequency_end_hz;
  if (start_hz > end_hz) {
    std::swap(start_hz, end_hz);
  }
  if (end_hz - start_hz < 1.0) {
    end_hz = start_hz + 1.0;
  }

  const bool fft_changed = normalized_fft != fft_size_ || new_bins != spectrum_bins_;
  const bool range_changed = std::abs(start_hz - frequency_start_hz_) >= 1.0 ||
                             std::abs(end_hz - frequency_end_hz_) >= 1.0;
  if (!fft_changed && !range_changed) {
    return;
  }

  fft_size_ = normalized_fft;
  spectrum_bins_ = new_bins;
  frequency_start_hz_ = start_hz;
  frequency_end_hz_ = end_hz;

  if (fft_changed) {
    for (auto it = states_.begin(); it != states_.end(); ++it) {
      ReinitializeState(&it.value());
    }
  }
  update();
}

void SignalVisualizationWidget::SetSpectrumSource(SpectrumSource source) {
  if (spectrum_source_ == source) {
    return;
  }
  spectrum_source_ = source;
  update();
}

void SignalVisualizationWidget::SetChannelLabel(const QString& label) {
  if (channel_label_ == label) {
    return;
  }
  channel_label_ = label;
  update();
}

void SignalVisualizationWidget::SetReceiverSquelchThresholdDb(uint32_t receiver_id, double threshold_db) {
  ReceiverState& state = states_[receiver_id];
  const double clamped_db = std::clamp(threshold_db, -120.0, 0.0);
  if (state.has_squelch_threshold_db && std::abs(state.squelch_threshold_db - clamped_db) < 0.05) {
    return;
  }
  state.squelch_threshold_db = clamped_db;
  state.has_squelch_threshold_db = true;
  update();
}

void SignalVisualizationWidget::SetReceiverSignalLevelDb(uint32_t receiver_id, double signal_level_db) {
  ReceiverState& state = states_[receiver_id];
  const double clamped_db = std::clamp(signal_level_db, -120.0, 0.0);
  if (state.has_signal_level_db && std::abs(state.signal_level_db - clamped_db) < 0.05) {
    return;
  }
  state.signal_level_db = clamped_db;
  state.has_signal_level_db = true;
  update();
}

void SignalVisualizationWidget::SetReceiverIqHealth(uint32_t receiver_id, double psd_peak_db,
                                                    double psd_floor_db, double snr_db,
                                                    double psd_peak_offset_hz, bool has_quality_score,
                                                    double quality_score_pct, bool has_signal_ok,
                                                    bool signal_ok) {
  ReceiverState& state = states_[receiver_id];
  state.has_iq_health = true;
  state.psd_peak_db = std::clamp(psd_peak_db, -140.0, 20.0);
  state.psd_floor_db = std::clamp(psd_floor_db, -140.0, 20.0);
  state.snr_db = std::clamp(snr_db, 0.0, 120.0);
  state.psd_peak_offset_hz = psd_peak_offset_hz;
  state.has_quality_score = has_quality_score;
  state.quality_score_pct = std::clamp(quality_score_pct, 0.0, 100.0);
  state.has_signal_ok = has_signal_ok;
  state.signal_ok = signal_ok;
  update();
}

void SignalVisualizationWidget::SetAutoNoiseReductionEnabled(bool enabled) {
  if (auto_noise_reduction_enabled_ == enabled) {
    return;
  }
  auto_noise_reduction_enabled_ = enabled;
  update();
}

void SignalVisualizationWidget::SetNoiseFloorFilterEnabled(bool enabled) {
  if (noise_floor_filter_enabled_ == enabled) {
    return;
  }
  noise_floor_filter_enabled_ = enabled;
  update();
}

void SignalVisualizationWidget::SetNoiseFloorDb(double noise_floor_db) {
  const double clamped_db = std::clamp(noise_floor_db, -120.0, 0.0);
  if (std::abs(noise_floor_db_ - clamped_db) < 0.05) {
    return;
  }
  noise_floor_db_ = clamped_db;
  update();
}

bool SignalVisualizationWidget::AutoNoiseReductionEnabled() const {
  return auto_noise_reduction_enabled_;
}

bool SignalVisualizationWidget::NoiseFloorFilterEnabled() const {
  return noise_floor_filter_enabled_;
}

double SignalVisualizationWidget::NoiseFloorDb() const {
  return noise_floor_db_;
}

SignalVisualizationWidget::SpectrumSource SignalVisualizationWidget::CurrentSpectrumSource() const {
  return spectrum_source_;
}

int SignalVisualizationWidget::FftSize() const { return fft_size_; }

double SignalVisualizationWidget::FrequencyStartHz() const { return frequency_start_hz_; }

double SignalVisualizationWidget::FrequencyEndHz() const { return frequency_end_hz_; }

void SignalVisualizationWidget::PushVisualizationFrame(uint32_t receiver_id, const std::vector<double>& waveform,
                                                       const std::vector<double>& spectrum,
                                                       double peak_frequency_hz, double peak_intensity,
                                                       SpectrumSource source, double frame_frequency_start_hz,
                                                       double frame_frequency_end_hz) {
  ReceiverState& state = states_[receiver_id];
  if (source == SpectrumSource::kReceiverInput) {
    BlendReceiverSpectrumIntoState(&state, spectrum, peak_frequency_hz, peak_intensity,
                                   frame_frequency_start_hz, frame_frequency_end_hz);
  } else {
    BlendFrameIntoState(&state, waveform, spectrum, peak_frequency_hz, peak_intensity,
                        frame_frequency_end_hz);
  }
  update();
}

void SignalVisualizationWidget::PushSample(uint32_t receiver_id, double frequency_hz, double intensity) {
  ReceiverState& state = states_[receiver_id];
  BlendSampleIntoState(&state, frequency_hz, intensity);
  update();
}

void SignalVisualizationWidget::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(14, 18, 26));

  const QRect content = rect().adjusted(10, 10, -10, -10);
  const int gap = 10;
  const int left_width = (content.width() - gap) * 55 / 100;

  const QRect waveform_rect(content.left(), content.top(), left_width, content.height());
  const QRect right_rect(content.left() + left_width + gap, content.top(),
                         content.width() - left_width - gap, content.height());
  const int right_gap = 8;
  const int right_top_height = (right_rect.height() - right_gap) / 2;
  const QRect spectrogram_rect(right_rect.left(), right_rect.top(), right_rect.width(), right_top_height);
  const QRect waterfall_rect(right_rect.left(), right_rect.top() + right_top_height + right_gap,
                             right_rect.width(), right_rect.height() - right_top_height - right_gap);

  auto draw_panel = [&painter](const QRect& panel_rect, const QString& title) {
    painter.setPen(QPen(QColor(66, 79, 102), 1));
    painter.setBrush(QColor(20, 27, 38));
    painter.drawRoundedRect(panel_rect, 6, 6);

    const QRect title_rect = panel_rect.adjusted(10, 6, -10, -panel_rect.height() + 24);
    painter.setPen(QColor(178, 192, 214));
    painter.drawText(title_rect, Qt::AlignLeft | Qt::AlignVCenter, title);
  };

  const QString source_label =
      (spectrum_source_ == SpectrumSource::kReceiverInput) ? "Receiver" : "Demod";
  draw_panel(waveform_rect, QString("Signal Waveform (last %1s)").arg(kWaveformWindowSeconds, 0, 'f', 0));
  draw_panel(spectrogram_rect, QString("Spectrogram [%1] (FFT %2)").arg(source_label).arg(fft_size_));

  const bool needs_selection = RequireExplicitSelection();
  if (needs_selection) {
    painter.setPen(QColor(212, 220, 235));
    painter.drawText(content, Qt::AlignCenter,
                     "Välj en mottagare i Message Filters för att styra visualiseringarna.");
    return;
  }

  const DisplayState display = BuildDisplayState();

  draw_panel(waterfall_rect, QString("Waterfall [%1] (%2 - %3)")
                                 .arg(source_label)
                                 .arg(FormatFrequencyLabel(display.frequency_start_hz))
                                 .arg(FormatFrequencyLabel(display.frequency_end_hz)));

  const QRect waveform_content = waveform_rect.adjusted(8, 30, -8, -8);
  const int meter_height = std::clamp(waveform_content.height() / 4, 44, 80);
  const int waveform_height = std::max(24, waveform_content.height() - meter_height - 8);
  const QRect waveform_plot_rect(waveform_content.left(), waveform_content.top(), waveform_content.width(),
                                 waveform_height);
  const QRect level_meter_rect(waveform_content.left(), waveform_plot_rect.bottom() + 8,
                               waveform_content.width(), meter_height);

  DrawWaveform(&painter, waveform_plot_rect, display.waveform);

  // Channel label overlay: shown in scanner mode when a channel is active.
  if (!channel_label_.isEmpty() && waveform_plot_rect.isValid()) {
    painter.save();
    painter.setClipRect(waveform_plot_rect);
    QFont label_font = painter.font();
    label_font.setPointSizeF(label_font.pointSizeF() * 1.5);
    label_font.setBold(true);
    painter.setFont(label_font);
    const QFontMetrics fm(label_font);
    // Measure bounding rect for multi-line text
    const QRect text_bounds = fm.boundingRect(
        waveform_plot_rect, Qt::AlignCenter | Qt::TextWordWrap, channel_label_);
    const QRect bg_rect = text_bounds.adjusted(-14, -10, 14, 10);
    painter.setBrush(QColor(8, 18, 32, 210));
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(bg_rect.intersected(waveform_plot_rect), 8, 8);
    painter.setPen(QColor(230, 240, 255));
    painter.drawText(waveform_plot_rect, Qt::AlignCenter | Qt::TextWordWrap, channel_label_);
    painter.restore();
  }

  DrawLevelMeter(&painter, level_meter_rect, display.signal_level, display.signal_peak_hold,
                 display.has_signal_level_db, display.signal_level_db, display.has_iq_health,
                 display.psd_peak_db, display.psd_floor_db, display.snr_db, display.psd_peak_offset_hz,
                 display.has_quality_score, display.quality_score_pct, display.has_signal_ok,
                 display.signal_ok,
                 display.has_squelch_threshold_db, display.squelch_threshold_db);
  const bool apply_noise_floor_filter =
      noise_floor_filter_enabled_ && spectrum_source_ == SpectrumSource::kReceiverInput;
  DrawSpectrumCurve(&painter, spectrogram_rect.adjusted(8, 30, -8, -8), display.spectrum,
                    display.frequency_start_hz, display.frequency_end_hz, false,
                    display.has_squelch_threshold_db, display.squelch_threshold_db,
                    apply_noise_floor_filter, noise_floor_db_);
  DrawHeatmap(&painter, waterfall_rect.adjusted(8, 30, -8, -8), display.waterfall_rows, true, true,
              auto_noise_reduction_enabled_, apply_noise_floor_filter, noise_floor_db_);
}

void SignalVisualizationWidget::OnFrameTick() {
  // Keep UI responsive for repaint requests, but do not synthesize decay between samples.
  update();
}

void SignalVisualizationWidget::EnsureState(ReceiverState* state) const {
  if (state == nullptr) {
    return;
  }
  if (state->waveform.size() != kWaveformPoints) {
    state->waveform = QVector<double>(kWaveformPoints, 0.5);
  }
  if (state->spectrum.size() != spectrum_bins_) {
    state->spectrum = QVector<double>(spectrum_bins_, 0.0);
    state->spectrogram_rows.clear();
    state->waterfall_rows.clear();
  }
  if (state->receiver_spectrum.size() != spectrum_bins_) {
    state->receiver_spectrum = QVector<double>(spectrum_bins_, 0.0);
    state->receiver_spectrogram_rows.clear();
    state->receiver_waterfall_rows.clear();
  }

  for (const QVector<double>& row : std::as_const(state->spectrogram_rows)) {
    if (row.size() != spectrum_bins_) {
      state->spectrogram_rows.clear();
      break;
    }
  }
  for (const QVector<double>& row : std::as_const(state->waterfall_rows)) {
    if (row.size() != spectrum_bins_) {
      state->waterfall_rows.clear();
      break;
    }
  }
  for (const QVector<double>& row : std::as_const(state->receiver_spectrogram_rows)) {
    if (row.size() != spectrum_bins_) {
      state->receiver_spectrogram_rows.clear();
      break;
    }
  }
  for (const QVector<double>& row : std::as_const(state->receiver_waterfall_rows)) {
    if (row.size() != spectrum_bins_) {
      state->receiver_waterfall_rows.clear();
      break;
    }
  }
}

void SignalVisualizationWidget::ReinitializeState(ReceiverState* state) const {
  if (state == nullptr) {
    return;
  }
  state->waveform = QVector<double>(kWaveformPoints, 0.5);
  state->spectrum = QVector<double>(spectrum_bins_, 0.0);
  state->spectrogram_rows.clear();
  state->waterfall_rows.clear();
  state->signal_level = 0.0;
  state->signal_peak_hold = 0.0;
  state->receiver_spectrum = QVector<double>(spectrum_bins_, 0.0);
  state->receiver_spectrogram_rows.clear();
  state->receiver_waterfall_rows.clear();
  state->receiver_frequency_start_hz = 0.0;
  state->receiver_frequency_end_hz = 20000.0;
  state->receiver_frequency_range_valid = false;
  state->has_signal_level_db = false;
  state->signal_level_db = -120.0;
  state->has_squelch_threshold_db = false;
  state->squelch_threshold_db = -67.5;
  state->has_iq_health = false;
  state->psd_peak_db = -120.0;
  state->psd_floor_db = -120.0;
  state->snr_db = 0.0;
  state->psd_peak_offset_hz = 0.0;
  state->has_quality_score = false;
  state->quality_score_pct = 0.0;
  state->has_signal_ok = false;
  state->signal_ok = false;
}

int SignalVisualizationWidget::NormalizeFftSize(int fft_size) {
  static constexpr std::array<int, 7> kAllowedFftSizes = {64, 128, 256, 512, 1024, 2048, 4096};
  int best = kAllowedFftSizes.front();
  int best_distance = std::abs(fft_size - best);
  for (const int candidate : kAllowedFftSizes) {
    const int distance = std::abs(fft_size - candidate);
    if (distance < best_distance) {
      best_distance = distance;
      best = candidate;
    }
  }
  return best;
}

int SignalVisualizationWidget::SpectrumBinsFromFftSize(int fft_size) {
  const int normalized_fft = NormalizeFftSize(fft_size);
  return std::max(32, normalized_fft / 2);
}

void SignalVisualizationWidget::PushRow(QVector<QVector<double>>* rows, const QVector<double>& row,
                                        int max_rows) {
  if (rows == nullptr) {
    return;
  }
  rows->push_back(row);
  while (rows->size() > max_rows) {
    rows->removeFirst();
  }
}

QColor SignalVisualizationWidget::HeatColor(double value) {
  const double v = Clamp01(value);
  const int r = static_cast<int>(20 + 235 * std::pow(v, 0.75));
  const int g = static_cast<int>(35 + 140 * std::pow(v, 1.25));
  const int b = static_cast<int>(45 + 210 * (1.0 - std::pow(v, 0.65)));
  return QColor(r, g, b);
}

QColor SignalVisualizationWidget::RainbowColor(double value) {
  const double v = Clamp01(value);
  const double hue = (1.0 - v) * 0.75;  // violet -> red
  const double sat = 1.0;
  const double val = 0.18 + std::pow(v, 0.85) * 0.82;
  return QColor::fromHsvF(hue, sat, val);
}

void SignalVisualizationWidget::DrawWaveform(QPainter* painter, const QRect& area,
                                             const QVector<double>& waveform) {
  if (painter == nullptr || waveform.isEmpty()) {
    return;
  }

  painter->save();
  painter->setClipRect(area);

  painter->fillRect(area, QColor(11, 16, 24));

  painter->setPen(QPen(QColor(48, 58, 78), 1));
  for (int i = 1; i < 5; ++i) {
    const int y = area.top() + (i * area.height() / 5);
    painter->drawLine(area.left(), y, area.right(), y);
  }
  painter->setPen(QPen(QColor(42, 74, 92), 1));
  const int center_y = area.top() + area.height() / 2;
  painter->drawLine(area.left(), center_y, area.right(), center_y);

  painter->setPen(QPen(QColor(40, 49, 66), 1));
  for (int i = 0; i <= 5; ++i) {
    const double t = static_cast<double>(i) / 5.0;
    const int x = area.left() + static_cast<int>(t * (area.width() - 1));
    painter->drawLine(x, area.top(), x, area.bottom());
  }

  QPainterPath path;
  for (int i = 0; i < waveform.size(); ++i) {
    const double t = (waveform.size() <= 1) ? 0.0 : static_cast<double>(i) / (waveform.size() - 1);
    const int x = area.left() + static_cast<int>(t * (area.width() - 1));
    const int y = area.bottom() - static_cast<int>(Clamp01(waveform[i]) * (area.height() - 1));
    if (i == 0) {
      path.moveTo(x, y);
    } else {
      path.lineTo(x, y);
    }
  }

  painter->setPen(QPen(QColor(92, 220, 168), 2));
  painter->drawPath(path);

  painter->setPen(QColor(145, 161, 186));
  painter->drawText(area.adjusted(4, 2, -4, -2), Qt::AlignTop | Qt::AlignLeft,
                    QString("%1s window").arg(kWaveformWindowSeconds, 0, 'f', 0));
  painter->restore();
}

void SignalVisualizationWidget::DrawLevelMeter(QPainter* painter, const QRect& area, double level,
                                               double peak_hold, bool has_signal_level_db,
                                               double signal_level_db, bool has_iq_health,
                                               double psd_peak_db, double psd_floor_db, double snr_db,
                                               double psd_peak_offset_hz, bool has_quality_score,
                                               double quality_score_pct, bool has_signal_ok,
                                               bool signal_ok, bool has_squelch_threshold_db,
                                               double squelch_threshold_db) {
  if (painter == nullptr || !area.isValid()) {
    return;
  }

  constexpr double kMinDb = -120.0;
  constexpr double kMaxDb = 0.0;
  constexpr double kDbRange = kMaxDb - kMinDb;

  // Convert linear amplitude [0,1] to approximate dBFS.
  auto amp_to_db = [=](double amp) -> double {
    return kMinDb + std::pow(Clamp01(amp), 0.42) * kDbRange;
  };
  // Map dBFS value to [0,1] bar position (linear in dB: −120 = 0, 0 = 1).
  auto db_to_ratio = [=](double db) -> double {
    return (std::clamp(db, kMinDb, kMaxDb) - kMinDb) / kDbRange;
  };

  const double clamped_level = Clamp01(level);
  const double clamped_peak = Clamp01(peak_hold);

  // Prefer backend channelized RSSI (matches what squelch compares); fall back to audio amplitude.
  const double level_db = has_signal_level_db ? std::clamp(signal_level_db, kMinDb, kMaxDb)
                                              : amp_to_db(clamped_level);
  const double fill_ratio = db_to_ratio(level_db);
  const double peak_ratio = db_to_ratio(amp_to_db(clamped_peak));

  painter->save();
  painter->setClipRect(area);
  painter->fillRect(area, QColor(11, 16, 24));
  painter->setPen(QPen(QColor(66, 79, 102), 1));
  painter->drawRect(area.adjusted(0, 0, -1, -1));

  const QRect bars_rect = area.adjusted(8, 6, -8, -6);
  const int sub_gap = 6;
  const int sub_height = std::max(12, (bars_rect.height() - sub_gap) / 2);
  const QRect level_bar_rect(bars_rect.left(), bars_rect.top(), bars_rect.width(), sub_height);
  const QRect quality_bar_rect(bars_rect.left(), level_bar_rect.bottom() + sub_gap, bars_rect.width(),
                               std::max(8, bars_rect.bottom() - (level_bar_rect.bottom() + sub_gap) + 1));
  painter->fillRect(level_bar_rect, QColor(24, 32, 45));
  painter->fillRect(quality_bar_rect, QColor(24, 32, 45));

  // Level bar fill: dBFS scale, −120 left → 0 right.
  const int fill_width = static_cast<int>(std::round(fill_ratio * level_bar_rect.width()));
  if (fill_width > 0) {
    QLinearGradient gradient(level_bar_rect.topLeft(), level_bar_rect.topRight());
    gradient.setColorAt(0.0, QColor(72, 168, 110));
    gradient.setColorAt(0.55, QColor(238, 194, 83));
    gradient.setColorAt(1.0, QColor(222, 98, 74));
    painter->fillRect(
        QRect(level_bar_rect.left(), level_bar_rect.top(), fill_width, level_bar_rect.height()),
        gradient);
  }

  // Tick lines every 20 dBFS (−120, −100, −80, −60, −40, −20, 0).
  painter->setPen(QPen(QColor(42, 55, 74), 1));
  for (int tick_db = static_cast<int>(kMinDb); tick_db <= static_cast<int>(kMaxDb); tick_db += 20) {
    const int tick_x = level_bar_rect.left() +
        static_cast<int>(std::round(db_to_ratio(static_cast<double>(tick_db)) *
                                    (level_bar_rect.width() - 1)));
    painter->drawLine(tick_x, level_bar_rect.top(), tick_x, level_bar_rect.bottom());
  }

  // Peak hold tick (white).
  const int peak_x = level_bar_rect.left() +
      static_cast<int>(std::round(peak_ratio * static_cast<double>(level_bar_rect.width() - 1)));
  painter->setPen(QPen(QColor(220, 230, 245), 1));
  painter->drawLine(peak_x, level_bar_rect.top(), peak_x, level_bar_rect.bottom());

  // Scale labels −120 / 0 dBFS at the bar edges (tiny font).
  if (level_bar_rect.height() >= 14) {
    QFont scale_font = painter->font();
    scale_font.setPointSizeF(std::max(5.5, painter->font().pointSizeF() * 0.72));
    painter->setFont(scale_font);
    painter->setPen(QColor(72, 88, 115));
    const QRect bar_label = level_bar_rect.adjusted(2, 0, -2, 0);
    painter->drawText(bar_label, Qt::AlignLeft | Qt::AlignBottom, "-120");
    painter->drawText(bar_label, Qt::AlignRight | Qt::AlignBottom, "0 dBFS");
  }

  // Squelch threshold marker: yellow-orange vertical line + value label.
  if (has_squelch_threshold_db) {
    const double sq_ratio = db_to_ratio(squelch_threshold_db);
    const int sq_x = level_bar_rect.left() +
        static_cast<int>(std::round(sq_ratio * (level_bar_rect.width() - 1)));
    painter->setPen(QPen(QColor(255, 196, 48), 2));
    painter->drawLine(sq_x, level_bar_rect.top(), sq_x, level_bar_rect.bottom());

    // Value label above the bar (if there is vertical room between area top and bar top).
    if (level_bar_rect.top() - area.top() >= 8) {
      QFont sq_font = painter->font();
      sq_font.setPointSizeF(std::max(5.5, sq_font.pointSizeF() * 0.72));
      sq_font.setBold(true);
      painter->setFont(sq_font);
      painter->setPen(QColor(255, 196, 48));
      const QString sq_label = QString("%1 dB").arg(squelch_threshold_db, 0, 'f', 0);
      const int label_w = 40;
      // Prefer drawing right of the line; flip left if close to right edge.
      const int label_x = (sq_x + label_w > level_bar_rect.right()) ? sq_x - label_w : sq_x + 2;
      painter->drawText(QRect(label_x, area.top(), label_w, level_bar_rect.top() - area.top()),
                        Qt::AlignLeft | Qt::AlignVCenter, sq_label);
    }
  }

  // Quality bar (IQ / signal-ok indicator).
  const double quality_ratio =
      has_quality_score ? std::clamp(quality_score_pct / 100.0, 0.0, 1.0)
                        : (has_signal_ok ? (signal_ok ? 1.0 : 0.0) : 0.0);
  const int quality_width = static_cast<int>(std::round(quality_ratio * quality_bar_rect.width()));
  if (quality_width > 0) {
    QLinearGradient quality_gradient(quality_bar_rect.topLeft(), quality_bar_rect.topRight());
    quality_gradient.setColorAt(0.0, QColor(201, 85, 74));
    quality_gradient.setColorAt(0.5, QColor(244, 190, 95));
    quality_gradient.setColorAt(1.0, QColor(92, 220, 168));
    painter->fillRect(QRect(quality_bar_rect.left(), quality_bar_rect.top(), quality_width,
                            quality_bar_rect.height()),
                      quality_gradient);
  }

  // Text labels (drawn over bars with the default font).
  painter->setFont(QFont());
  painter->setPen(QColor(178, 192, 214));
  const QRect title_row =
      QRect(area.left() + 10, area.top(), area.width() - 20, area.height() / 2 - 2);
  const QRect detail_row =
      QRect(area.left() + 10, area.top() + area.height() / 2, area.width() - 20,
            area.height() - area.height() / 2);
  painter->drawText(title_row, Qt::AlignLeft | Qt::AlignVCenter, "Signal Level");
  painter->drawText(title_row, Qt::AlignRight | Qt::AlignVCenter,
                    QString("%1 dBFS").arg(level_db, 0, 'f', 1));

  QString detail_text = "PSD: n/a";
  if (has_iq_health) {
    detail_text = QString("PSD p/f %1/%2 dB  SNR %3 dB  Peak %4 Hz")
                      .arg(psd_peak_db, 0, 'f', 1)
                      .arg(psd_floor_db, 0, 'f', 1)
                      .arg(snr_db, 0, 'f', 1)
                      .arg(psd_peak_offset_hz, 0, 'f', 0);
  }
  painter->setPen(QColor(148, 165, 190));
  painter->drawText(detail_row, Qt::AlignLeft | Qt::AlignVCenter, detail_text);

  const QString quality_text = has_quality_score
                                   ? QString("Quality %1%").arg(quality_score_pct, 0, 'f', 0)
                                   : "Quality ?";
  const QString status_text =
      has_signal_ok ? (signal_ok ? "Signal OK" : "Signal CHECK") : "Signal ?";
  const QColor status_color = has_signal_ok ? (signal_ok ? QColor(92, 220, 168) : QColor(255, 184, 93))
                                            : QColor(138, 152, 178);
  painter->setPen(status_color);
  painter->drawText(detail_row, Qt::AlignRight | Qt::AlignVCenter,
                    QString("%1  %2").arg(quality_text, status_text));
  painter->restore();
}

void SignalVisualizationWidget::DrawSpectrumCurve(QPainter* painter, const QRect& area,
                                                  const QVector<double>& spectrum, double frequency_start_hz,
                                                  double frequency_end_hz, bool suppress_below_mean,
                                                  bool has_squelch_threshold_db,
                                                  double squelch_threshold_db,
                                                  bool has_noise_floor_threshold_db,
                                                  double noise_floor_threshold_db) {
  if (painter == nullptr || spectrum.isEmpty()) {
    return;
  }

  constexpr double kMinDb = -120.0;
  constexpr double kMaxDb = 0.0;
  const double display_min_db = has_noise_floor_threshold_db
                                    ? std::clamp(noise_floor_threshold_db, kMinDb, kMaxDb - 1.0)
                                    : kMinDb;
  const double display_span_db = std::max(1.0, kMaxDb - display_min_db);

  painter->save();
  painter->setClipRect(area);
  painter->fillRect(area, QColor(11, 16, 24));

  const QRect plot = area.adjusted(46, 8, -10, -24);
  painter->setPen(QPen(QColor(70, 84, 109), 1));
  painter->drawRect(plot);

  auto db_to_y = [&plot, display_min_db, display_span_db](double db) {
    const double t = Clamp01((db - display_min_db) / display_span_db);
    return plot.bottom() - static_cast<int>(t * (plot.height() - 1));
  };
  auto amp_to_db = [](double amp) {
    return kMinDb + std::pow(Clamp01(amp), 0.42) * (kMaxDb - kMinDb);
  };
  const double clamped_noise_floor_db =
      std::clamp(noise_floor_threshold_db, kMinDb, kMaxDb);

  painter->setPen(QPen(QColor(48, 58, 78), 1));
  for (int i = 0; i <= 4; ++i) {
    const double t = static_cast<double>(i) / 4.0;
    const double tick_db = display_min_db + t * display_span_db;
    const int y = db_to_y(tick_db);
    painter->drawLine(plot.left(), y, plot.right(), y);
    painter->setPen(QColor(160, 176, 200));
    painter->drawText(area.left() + 2, y + 4, QString::number(tick_db, 'f', 1));
    painter->setPen(QPen(QColor(48, 58, 78), 1));
  }

  const double span_hz = std::max(1.0, frequency_end_hz - frequency_start_hz);
  for (int tick = 0; tick <= 4; ++tick) {
    const double t = static_cast<double>(tick) / 4.0;
    const double tick_hz = frequency_start_hz + t * span_hz;
    const int x = plot.left() + static_cast<int>(t * (plot.width() - 1));
    painter->drawLine(x, plot.top(), x, plot.bottom());
    painter->setPen(QColor(160, 176, 200));
    painter->drawText(x - 24, area.bottom() - 4, FormatFrequencyLabel(tick_hz));
    painter->setPen(QPen(QColor(48, 58, 78), 1));
  }

  painter->setPen(QColor(178, 192, 214));
  painter->drawText(area.left() + 2, plot.top() - 2, "dB");
  painter->drawText(plot.right() - 64, area.bottom() - 4, "Freq (Hz)");

  if (has_squelch_threshold_db) {
    const double clamped_db = std::clamp(squelch_threshold_db, display_min_db, kMaxDb);
    const int y = db_to_y(clamped_db);
    painter->setPen(QPen(QColor(96, 216, 255), 1, Qt::DashLine));
    painter->drawLine(plot.left(), y, plot.right(), y);

    const QString label = QString("Squelch %1 dB").arg(clamped_db, 0, 'f', 1);
    const QFontMetrics metrics(painter->font());
    const int label_width = metrics.horizontalAdvance(label) + 10;
    const int label_height = metrics.height() + 2;
    const int x = std::max(plot.left() + 2, plot.right() - label_width - 2);
    const int y_top = std::clamp(y - label_height - 2, plot.top() + 2, plot.bottom() - label_height - 2);
    const QRect label_rect(x, y_top, label_width, label_height);
    painter->fillRect(label_rect, QColor(9, 22, 34, 215));
    painter->setPen(QColor(140, 232, 255));
    painter->drawText(label_rect.adjusted(5, 0, -5, 0), Qt::AlignLeft | Qt::AlignVCenter, label);
  }

  if (has_noise_floor_threshold_db) {
    const int y = db_to_y(std::clamp(clamped_noise_floor_db, display_min_db, kMaxDb));
    painter->setPen(QPen(QColor(120, 136, 160), 1, Qt::DashLine));
    painter->drawLine(plot.left(), y, plot.right(), y);

    const QString label = QString("Noise floor %1 dB").arg(clamped_noise_floor_db, 0, 'f', 1);
    const QFontMetrics metrics(painter->font());
    const int label_width = metrics.horizontalAdvance(label) + 10;
    const int label_height = metrics.height() + 2;
    const int x = plot.left() + 2;
    const int y_top = std::clamp(y + 2, plot.top() + 2, plot.bottom() - label_height - 2);
    const QRect label_rect(x, y_top, label_width, label_height);
    painter->fillRect(label_rect, QColor(17, 22, 31, 220));
    painter->setPen(QColor(170, 184, 205));
    painter->drawText(label_rect.adjusted(5, 0, -5, 0), Qt::AlignLeft | Qt::AlignVCenter, label);
  }

  double mean_amplitude = 0.0;
  if (suppress_below_mean && !spectrum.isEmpty()) {
    for (const double value : spectrum) {
      mean_amplitude += Clamp01(value);
    }
    mean_amplitude /= static_cast<double>(spectrum.size());
  }

  QPainterPath path;
  bool segment_open = false;
  for (int i = 0; i < spectrum.size(); ++i) {
    const double amplitude = Clamp01(spectrum[i]);
    if (suppress_below_mean && amplitude < mean_amplitude) {
      segment_open = false;
      continue;
    }
    const double db = amp_to_db(amplitude);
    if (has_noise_floor_threshold_db && db <= clamped_noise_floor_db) {
      segment_open = false;
      continue;
    }
    const double t = (spectrum.size() <= 1) ? 0.0 : static_cast<double>(i) / (spectrum.size() - 1);
    const int x = plot.left() + static_cast<int>(t * (plot.width() - 1));
    const int y = db_to_y(db);
    if (!segment_open) {
      path.moveTo(x, y);
      segment_open = true;
    } else {
      path.lineTo(x, y);
    }
  }

  if (!path.isEmpty()) {
    painter->setPen(QPen(QColor(255, 176, 95), 2));
    painter->drawPath(path);
  }
  painter->restore();
}

void SignalVisualizationWidget::DrawHeatmap(QPainter* painter, const QRect& area,
                                            const QVector<QVector<double>>& rows, bool newest_at_top,
                                            bool rainbow_colors, bool suppress_below_mean,
                                            bool has_noise_floor_threshold_db,
                                            double noise_floor_threshold_db) {
  if (painter == nullptr) {
    return;
  }

  painter->save();
  painter->setClipRect(area);
  painter->fillRect(area, QColor(11, 16, 24));
  constexpr double kMinDb = -120.0;
  constexpr double kMaxDb = 0.0;
  auto amp_to_db = [](double amp) {
    return kMinDb + std::pow(Clamp01(amp), 0.42) * (kMaxDb - kMinDb);
  };
  const double clamped_noise_floor_db =
      std::clamp(noise_floor_threshold_db, kMinDb, kMaxDb);
  const double display_min_db = has_noise_floor_threshold_db
                                    ? std::clamp(clamped_noise_floor_db, kMinDb, kMaxDb - 1.0)
                                    : kMinDb;
  const double display_span_db = std::max(1.0, kMaxDb - display_min_db);

  if (!rows.isEmpty() && !rows[0].isEmpty()) {
    const int row_count = rows.size();
    const int col_count = rows[0].size();

    const double cell_w = static_cast<double>(area.width()) / static_cast<double>(col_count);
    const double cell_h = static_cast<double>(area.height()) / static_cast<double>(row_count);

    for (int row = 0; row < row_count; ++row) {
      const int src_row = newest_at_top ? (row_count - 1 - row) : row;
      double row_mean = 0.0;
      if (suppress_below_mean) {
        const QVector<double>& row_values = rows[src_row];
        for (const double value : row_values) {
          row_mean += Clamp01(value);
        }
        row_mean /= static_cast<double>(row_values.size());
      }
      for (int col = 0; col < col_count; ++col) {
        const double cell_value = Clamp01(rows[src_row][col]);
        if (suppress_below_mean && cell_value <= row_mean) {
          continue;
        }
        const double cell_db = amp_to_db(cell_value);
        if (has_noise_floor_threshold_db && cell_db <= clamped_noise_floor_db) {
          continue;
        }
        const double color_value =
            has_noise_floor_threshold_db ? Clamp01((cell_db - display_min_db) / display_span_db)
                                         : cell_value;
        const QColor color = rainbow_colors ? RainbowColor(color_value) : HeatColor(color_value);
        const int x = area.left() + static_cast<int>(col * cell_w);
        const int y = area.top() + static_cast<int>(row * cell_h);
        const int w = static_cast<int>(std::ceil(cell_w));
        const int h = static_cast<int>(std::ceil(cell_h));
        painter->fillRect(QRect(x, y, w, h), color);
      }
    }
  }

  painter->restore();
}

bool SignalVisualizationWidget::RequireExplicitSelection() const {
  return known_receivers_.size() > 1 && receiver_filter_id_ < 0;
}

SignalVisualizationWidget::DisplayState SignalVisualizationWidget::BuildDisplayState() const {
  DisplayState display;
  display.waveform = QVector<double>(kWaveformPoints, 0.5);
  display.spectrum = QVector<double>(spectrum_bins_, 0.0);
  display.signal_level = 0.0;
  display.signal_peak_hold = 0.0;
  display.frequency_start_hz = frequency_start_hz_;
  display.frequency_end_hz = frequency_end_hz_;
  display.has_signal_level_db = false;
  display.signal_level_db = -120.0;
  display.has_squelch_threshold_db = false;
  display.squelch_threshold_db = -67.5;
  display.has_iq_health = false;
  display.psd_peak_db = -120.0;
  display.psd_floor_db = -120.0;
  display.snr_db = 0.0;
  display.psd_peak_offset_hz = 0.0;
  display.has_quality_score = false;
  display.quality_score_pct = 0.0;
  display.has_signal_ok = false;
  display.signal_ok = false;

  if (states_.isEmpty()) {
    PushRow(&display.spectrogram_rows, display.spectrum, kSpectrogramRows);
    PushRow(&display.waterfall_rows, display.spectrum, kWaterfallRows);
    return display;
  }

  if (receiver_filter_id_ >= 0) {
    const uint32_t id = static_cast<uint32_t>(receiver_filter_id_);
    if (states_.contains(id)) {
      ReceiverState selected = states_.value(id);
      EnsureState(&selected);
      display.waveform = selected.waveform;
      display.signal_level = selected.signal_level;
      display.signal_peak_hold = selected.signal_peak_hold;
      if (spectrum_source_ == SpectrumSource::kReceiverInput) {
        display.spectrum = selected.receiver_spectrum;
        display.spectrogram_rows = selected.receiver_spectrogram_rows;
        display.waterfall_rows = selected.receiver_waterfall_rows;
        if (selected.receiver_frequency_range_valid) {
          display.frequency_start_hz = selected.receiver_frequency_start_hz;
          display.frequency_end_hz = selected.receiver_frequency_end_hz;
        }
      } else {
        display.spectrum = selected.spectrum;
        display.spectrogram_rows = selected.spectrogram_rows;
        display.waterfall_rows = selected.waterfall_rows;
        display.frequency_start_hz = 0.0;
        display.frequency_end_hz = selected.demod_frequency_end_hz;
      }
      display.has_signal_level_db = selected.has_signal_level_db;
      display.signal_level_db = selected.signal_level_db;
      display.has_squelch_threshold_db = selected.has_squelch_threshold_db;
      display.squelch_threshold_db = selected.squelch_threshold_db;
      display.has_iq_health = selected.has_iq_health;
      display.psd_peak_db = selected.psd_peak_db;
      display.psd_floor_db = selected.psd_floor_db;
      display.snr_db = selected.snr_db;
      display.psd_peak_offset_hz = selected.psd_peak_offset_hz;
      display.has_quality_score = selected.has_quality_score;
      display.quality_score_pct = selected.quality_score_pct;
      display.has_signal_ok = selected.has_signal_ok;
      display.signal_ok = selected.signal_ok;
      if (display.spectrogram_rows.isEmpty()) {
        PushRow(&display.spectrogram_rows, display.spectrum, kSpectrogramRows);
      }
      if (display.waterfall_rows.isEmpty()) {
        PushRow(&display.waterfall_rows, display.spectrum, kWaterfallRows);
      }
      return display;
    }
  }

  if (known_receivers_.size() == 1 && states_.contains(known_receivers_[0])) {
    ReceiverState selected = states_.value(known_receivers_[0]);
    EnsureState(&selected);
    display.waveform = selected.waveform;
    display.signal_level = selected.signal_level;
    display.signal_peak_hold = selected.signal_peak_hold;
    if (spectrum_source_ == SpectrumSource::kReceiverInput) {
      display.spectrum = selected.receiver_spectrum;
      display.spectrogram_rows = selected.receiver_spectrogram_rows;
      display.waterfall_rows = selected.receiver_waterfall_rows;
      if (selected.receiver_frequency_range_valid) {
        display.frequency_start_hz = selected.receiver_frequency_start_hz;
        display.frequency_end_hz = selected.receiver_frequency_end_hz;
      }
    } else {
      display.spectrum = selected.spectrum;
      display.spectrogram_rows = selected.spectrogram_rows;
      display.waterfall_rows = selected.waterfall_rows;
      display.frequency_start_hz = 0.0;
      display.frequency_end_hz = selected.demod_frequency_end_hz;
    }
    display.has_signal_level_db = selected.has_signal_level_db;
    display.signal_level_db = selected.signal_level_db;
    display.has_squelch_threshold_db = selected.has_squelch_threshold_db;
    display.squelch_threshold_db = selected.squelch_threshold_db;
    display.has_iq_health = selected.has_iq_health;
    display.psd_peak_db = selected.psd_peak_db;
    display.psd_floor_db = selected.psd_floor_db;
    display.snr_db = selected.snr_db;
    display.psd_peak_offset_hz = selected.psd_peak_offset_hz;
    display.has_quality_score = selected.has_quality_score;
    display.quality_score_pct = selected.quality_score_pct;
    display.has_signal_ok = selected.has_signal_ok;
    display.signal_ok = selected.signal_ok;
    if (display.spectrogram_rows.isEmpty()) {
      PushRow(&display.spectrogram_rows, display.spectrum, kSpectrogramRows);
    }
    if (display.waterfall_rows.isEmpty()) {
      PushRow(&display.waterfall_rows, display.spectrum, kWaterfallRows);
    }
    return display;
  }

  const auto state_values = states_.values();
  int used_states = 0;
  int ranged_states = 0;
  double frequency_start_sum = 0.0;
  double frequency_end_sum = 0.0;
  double signal_level_sum = 0.0;
  double signal_peak_sum = 0.0;
  double signal_level_db_sum = 0.0;
  int signal_level_db_count = 0;
  double squelch_threshold_sum = 0.0;
  int squelch_threshold_count = 0;
  double psd_peak_db_sum = 0.0;
  double psd_floor_db_sum = 0.0;
  double snr_db_sum = 0.0;
  double psd_peak_offset_sum = 0.0;
  int iq_health_count = 0;
  double quality_score_sum = 0.0;
  int quality_score_count = 0;
  int signal_ok_true_count = 0;
  int signal_ok_count = 0;
  for (const ReceiverState& state : state_values) {
    const QVector<double>& selected_spectrum =
        (spectrum_source_ == SpectrumSource::kReceiverInput) ? state.receiver_spectrum : state.spectrum;
    if (state.waveform.isEmpty() || selected_spectrum.isEmpty()) {
      continue;
    }
    ++used_states;
    for (int i = 0; i < display.waveform.size() && i < state.waveform.size(); ++i) {
      display.waveform[i] += state.waveform[i];
    }
    for (int i = 0; i < display.spectrum.size() && i < selected_spectrum.size(); ++i) {
      display.spectrum[i] += selected_spectrum[i];
    }
    signal_level_sum += state.signal_level;
    signal_peak_sum += state.signal_peak_hold;
    if (state.has_signal_level_db) {
      signal_level_db_sum += state.signal_level_db;
      ++signal_level_db_count;
    }
    if (state.has_squelch_threshold_db) {
      squelch_threshold_sum += state.squelch_threshold_db;
      ++squelch_threshold_count;
    }
    if (state.has_iq_health) {
      psd_peak_db_sum += state.psd_peak_db;
      psd_floor_db_sum += state.psd_floor_db;
      snr_db_sum += state.snr_db;
      psd_peak_offset_sum += state.psd_peak_offset_hz;
      ++iq_health_count;
    }
    if (state.has_quality_score) {
      quality_score_sum += state.quality_score_pct;
      ++quality_score_count;
    }
    if (state.has_signal_ok) {
      ++signal_ok_count;
      if (state.signal_ok) {
        ++signal_ok_true_count;
      }
    }
    if (spectrum_source_ == SpectrumSource::kReceiverInput && state.receiver_frequency_range_valid) {
      ++ranged_states;
      frequency_start_sum += state.receiver_frequency_start_hz;
      frequency_end_sum += state.receiver_frequency_end_hz;
    }
  }

  if (used_states > 0) {
    const double inv = 1.0 / static_cast<double>(used_states);
    for (double& v : display.waveform) {
      v *= inv;
    }
    for (double& v : display.spectrum) {
      v *= inv;
    }
    display.signal_level = signal_level_sum * inv;
    display.signal_peak_hold = signal_peak_sum * inv;
  }
  if (ranged_states > 0) {
    const double inv = 1.0 / static_cast<double>(ranged_states);
    display.frequency_start_hz = frequency_start_sum * inv;
    display.frequency_end_hz = frequency_end_sum * inv;
  }
  if (signal_level_db_count > 0) {
    display.has_signal_level_db = true;
    display.signal_level_db = signal_level_db_sum / static_cast<double>(signal_level_db_count);
  }
  if (squelch_threshold_count > 0) {
    display.has_squelch_threshold_db = true;
    display.squelch_threshold_db = squelch_threshold_sum / static_cast<double>(squelch_threshold_count);
  }
  if (iq_health_count > 0) {
    const double inv = 1.0 / static_cast<double>(iq_health_count);
    display.has_iq_health = true;
    display.psd_peak_db = psd_peak_db_sum * inv;
    display.psd_floor_db = psd_floor_db_sum * inv;
    display.snr_db = snr_db_sum * inv;
    display.psd_peak_offset_hz = psd_peak_offset_sum * inv;
  }
  if (quality_score_count > 0) {
    display.has_quality_score = true;
    display.quality_score_pct = quality_score_sum / static_cast<double>(quality_score_count);
  }
  if (signal_ok_count > 0) {
    display.has_signal_ok = true;
    display.signal_ok = signal_ok_true_count == signal_ok_count;
  }

  PushRow(&display.spectrogram_rows, display.spectrum, kSpectrogramRows);
  PushRow(&display.waterfall_rows, display.spectrum, kWaterfallRows);
  return display;
}

void SignalVisualizationWidget::BlendSampleIntoState(ReceiverState* state, double frequency_hz,
                                                     double intensity) {
  if (state == nullptr) {
    return;
  }
  EnsureState(state);

  const double normalized_intensity = Clamp01(intensity);
  if (frequency_hz > 0.0) {
    state->last_frequency_hz = frequency_hz;
  }

  double normalized_frequency = 0.5;
  bool frequency_in_range = false;
  const double span_hz = std::max(1.0, frequency_end_hz_ - frequency_start_hz_);
  if (state->last_frequency_hz > 0.0) {
    const double mapped_frequency = (state->last_frequency_hz - frequency_start_hz_) / span_hz;
    frequency_in_range = mapped_frequency >= 0.0 && mapped_frequency <= 1.0;
    if (frequency_in_range) {
      normalized_frequency = Clamp01(mapped_frequency);
    }
  }

  for (double& bin : state->spectrum) {
    bin = 0.0;
  }

  if (frequency_in_range) {
    const int max_bin = state->spectrum.size() - 1;
    const int peak_bin = static_cast<int>(std::round(normalized_frequency * max_bin));
    const int clamped_bin = std::clamp(peak_bin, 0, max_bin);
    state->spectrum[clamped_bin] = normalized_intensity;
  }

  state->waveform.removeFirst();
  state->waveform.push_back(normalized_intensity);
  state->signal_level = Clamp01(state->signal_level * 0.85 + normalized_intensity * 0.15);
  state->signal_peak_hold = std::max(state->signal_level, state->signal_peak_hold * 0.97);

  PushRow(&state->spectrogram_rows, state->spectrum, kSpectrogramRows);
  PushRow(&state->waterfall_rows, state->spectrum, kWaterfallRows);
}

void SignalVisualizationWidget::BlendFrameIntoState(ReceiverState* state, const std::vector<double>& waveform,
                                                    const std::vector<double>& spectrum,
                                                    double peak_frequency_hz, double peak_intensity,
                                                    double frame_frequency_end_hz) {
  if (state == nullptr) {
    return;
  }
  EnsureState(state);
  if (frame_frequency_end_hz > 1.0) {
    state->demod_frequency_end_hz = frame_frequency_end_hz;
  }

  const double waveform_point = ComputeWaveformSamplePoint(waveform);
  state->waveform.removeFirst();
  state->waveform.push_back(waveform_point);

  const double activity = ComputeWaveformActivity(waveform);
  const double level_input = std::max(activity, Clamp01(peak_intensity) * 0.35);
  state->signal_level = Clamp01(state->signal_level * 0.88 + level_input * 0.12);
  state->signal_peak_hold = std::max(state->signal_level, state->signal_peak_hold * 0.98);
  state->spectrum = ResampleToSize(spectrum, spectrum_bins_);

  if (peak_frequency_hz > 0.0) {
    state->last_frequency_hz = peak_frequency_hz;
  }

  if (!state->spectrum.isEmpty() && state->last_frequency_hz > 0.0) {
    const double span_hz = std::max(1.0, frequency_end_hz_ - frequency_start_hz_);
    const double mapped_frequency = (state->last_frequency_hz - frequency_start_hz_) / span_hz;
    if (mapped_frequency >= 0.0 && mapped_frequency <= 1.0) {
      const int max_bin = state->spectrum.size() - 1;
      const int peak_bin = std::clamp(static_cast<int>(std::round(mapped_frequency * max_bin)), 0, max_bin);
      state->spectrum[peak_bin] = std::max(state->spectrum[peak_bin], Clamp01(peak_intensity));
    }
  }

  PushRow(&state->spectrogram_rows, state->spectrum, kSpectrogramRows);
  PushRow(&state->waterfall_rows, state->spectrum, kWaterfallRows);
}

void SignalVisualizationWidget::BlendReceiverSpectrumIntoState(
    ReceiverState* state, const std::vector<double>& spectrum, double peak_frequency_hz, double peak_intensity,
    double frame_frequency_start_hz, double frame_frequency_end_hz) {
  if (state == nullptr) {
    return;
  }
  EnsureState(state);

  state->receiver_spectrum = ResampleToSize(spectrum, spectrum_bins_);
  if (frame_frequency_end_hz - frame_frequency_start_hz >= 1.0) {
    state->receiver_frequency_start_hz = frame_frequency_start_hz;
    state->receiver_frequency_end_hz = frame_frequency_end_hz;
    state->receiver_frequency_range_valid = true;
  }

  if (!state->receiver_spectrum.isEmpty() && peak_frequency_hz > 0.0 &&
      state->receiver_frequency_range_valid) {
    const double span_hz =
        std::max(1.0, state->receiver_frequency_end_hz - state->receiver_frequency_start_hz);
    const double mapped_frequency =
        (peak_frequency_hz - state->receiver_frequency_start_hz) / span_hz;
    if (mapped_frequency >= 0.0 && mapped_frequency <= 1.0) {
      const int max_bin = state->receiver_spectrum.size() - 1;
      const int peak_bin =
          std::clamp(static_cast<int>(std::round(mapped_frequency * max_bin)), 0, max_bin);
      state->receiver_spectrum[peak_bin] =
          std::max(state->receiver_spectrum[peak_bin], Clamp01(peak_intensity));
    }
  }

  PushRow(&state->receiver_spectrogram_rows, state->receiver_spectrum, kSpectrogramRows);
  PushRow(&state->receiver_waterfall_rows, state->receiver_spectrum, kWaterfallRows);
}

void SignalVisualizationWidget::DecayState(ReceiverState* state, double decay_factor) {
  Q_UNUSED(state);
  Q_UNUSED(decay_factor);
}

}  // namespace multi_radio
