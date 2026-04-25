#include "signal_visualization_widget.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <QColor>
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

int SignalVisualizationWidget::FftSize() const { return fft_size_; }

double SignalVisualizationWidget::FrequencyStartHz() const { return frequency_start_hz_; }

double SignalVisualizationWidget::FrequencyEndHz() const { return frequency_end_hz_; }

void SignalVisualizationWidget::PushVisualizationFrame(uint32_t receiver_id, const std::vector<double>& waveform,
                                                       const std::vector<double>& spectrum,
                                                       double peak_frequency_hz, double peak_intensity) {
  ReceiverState& state = states_[receiver_id];
  BlendFrameIntoState(&state, waveform, spectrum, peak_frequency_hz, peak_intensity);
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

  draw_panel(waveform_rect, "Signal Waveform");
  draw_panel(spectrogram_rect, QString("Spectrogram (FFT %1)").arg(fft_size_));
  draw_panel(waterfall_rect, QString("Waterfall (%1 - %2)")
                                 .arg(FormatFrequencyLabel(frequency_start_hz_))
                                 .arg(FormatFrequencyLabel(frequency_end_hz_)));

  const bool needs_selection = RequireExplicitSelection();
  if (needs_selection) {
    painter.setPen(QColor(212, 220, 235));
    painter.drawText(content, Qt::AlignCenter,
                     "Välj en mottagare i Message Filters för att styra visualiseringarna.");
    return;
  }

  const ReceiverState display = BuildDisplayState();

  DrawWaveform(&painter, waveform_rect.adjusted(8, 30, -8, -8), display.waveform);
  DrawSpectrumCurve(&painter, spectrogram_rect.adjusted(8, 30, -8, -8), display.spectrum,
                    frequency_start_hz_, frequency_end_hz_);
  DrawHeatmap(&painter, waterfall_rect.adjusted(8, 30, -8, -8), display.waterfall_rows, true, true);
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
    state->waveform = QVector<double>(kWaveformPoints, 0.0);
  }
  if (state->spectrum.size() != spectrum_bins_) {
    state->spectrum = QVector<double>(spectrum_bins_, 0.0);
    state->spectrogram_rows.clear();
    state->waterfall_rows.clear();
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
}

void SignalVisualizationWidget::ReinitializeState(ReceiverState* state) const {
  if (state == nullptr) {
    return;
  }
  state->waveform = QVector<double>(kWaveformPoints, 0.0);
  state->spectrum = QVector<double>(spectrum_bins_, 0.0);
  state->spectrogram_rows.clear();
  state->waterfall_rows.clear();
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
  painter->restore();
}

void SignalVisualizationWidget::DrawSpectrumCurve(QPainter* painter, const QRect& area,
                                                  const QVector<double>& spectrum, double frequency_start_hz,
                                                  double frequency_end_hz) {
  if (painter == nullptr || spectrum.isEmpty()) {
    return;
  }

  constexpr double kMinDb = -120.0;
  constexpr double kMaxDb = 0.0;

  painter->save();
  painter->setClipRect(area);
  painter->fillRect(area, QColor(11, 16, 24));

  const QRect plot = area.adjusted(46, 8, -10, -24);
  painter->setPen(QPen(QColor(70, 84, 109), 1));
  painter->drawRect(plot);

  auto db_to_y = [&plot](double db) {
    const double t = Clamp01((db + 120.0) / 120.0);
    return plot.bottom() - static_cast<int>(t * (plot.height() - 1));
  };
  auto amp_to_db = [](double amp) {
    return kMinDb + std::pow(Clamp01(amp), 0.42) * (kMaxDb - kMinDb);
  };

  const int db_ticks[] = {-120, -90, -60, -30, 0};
  painter->setPen(QPen(QColor(48, 58, 78), 1));
  for (const int tick_db : db_ticks) {
    const int y = db_to_y(static_cast<double>(tick_db));
    painter->drawLine(plot.left(), y, plot.right(), y);
    painter->setPen(QColor(160, 176, 200));
    painter->drawText(area.left() + 2, y + 4, QString::number(tick_db));
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

  QPainterPath path;
  for (int i = 0; i < spectrum.size(); ++i) {
    const double t = (spectrum.size() <= 1) ? 0.0 : static_cast<double>(i) / (spectrum.size() - 1);
    const int x = plot.left() + static_cast<int>(t * (plot.width() - 1));
    const double db = amp_to_db(spectrum[i]);
    const int y = db_to_y(db);
    if (i == 0) {
      path.moveTo(x, y);
    } else {
      path.lineTo(x, y);
    }
  }

  painter->setPen(QPen(QColor(255, 176, 95), 2));
  painter->drawPath(path);
  painter->restore();
}

void SignalVisualizationWidget::DrawHeatmap(QPainter* painter, const QRect& area,
                                            const QVector<QVector<double>>& rows, bool newest_at_top,
                                            bool rainbow_colors) {
  if (painter == nullptr) {
    return;
  }

  painter->save();
  painter->setClipRect(area);
  painter->fillRect(area, QColor(11, 16, 24));

  if (!rows.isEmpty() && !rows[0].isEmpty()) {
    const int row_count = rows.size();
    const int col_count = rows[0].size();

    const double cell_w = static_cast<double>(area.width()) / static_cast<double>(col_count);
    const double cell_h = static_cast<double>(area.height()) / static_cast<double>(row_count);

    for (int row = 0; row < row_count; ++row) {
      const int src_row = newest_at_top ? (row_count - 1 - row) : row;
      for (int col = 0; col < col_count; ++col) {
        const QColor color = rainbow_colors ? RainbowColor(rows[src_row][col]) : HeatColor(rows[src_row][col]);
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

SignalVisualizationWidget::ReceiverState SignalVisualizationWidget::BuildDisplayState() const {
  ReceiverState display;
  EnsureState(&display);

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
      return selected;
    }
  }

  if (known_receivers_.size() == 1 && states_.contains(known_receivers_[0])) {
    ReceiverState selected = states_.value(known_receivers_[0]);
    EnsureState(&selected);
    return selected;
  }

  const auto state_values = states_.values();
  int used_states = 0;
  for (const ReceiverState& state : state_values) {
    if (state.waveform.isEmpty() || state.spectrum.isEmpty()) {
      continue;
    }
    ++used_states;
    for (int i = 0; i < display.waveform.size() && i < state.waveform.size(); ++i) {
      display.waveform[i] += state.waveform[i];
    }
    for (int i = 0; i < display.spectrum.size() && i < state.spectrum.size(); ++i) {
      display.spectrum[i] += state.spectrum[i];
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

  PushRow(&state->spectrogram_rows, state->spectrum, kSpectrogramRows);
  PushRow(&state->waterfall_rows, state->spectrum, kWaterfallRows);
}

void SignalVisualizationWidget::BlendFrameIntoState(ReceiverState* state, const std::vector<double>& waveform,
                                                    const std::vector<double>& spectrum,
                                                    double peak_frequency_hz, double peak_intensity) {
  if (state == nullptr) {
    return;
  }
  EnsureState(state);

  state->waveform = ResampleToSize(waveform, kWaveformPoints);
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

void SignalVisualizationWidget::DecayState(ReceiverState* state, double decay_factor) {
  Q_UNUSED(state);
  Q_UNUSED(decay_factor);
}

}  // namespace multi_radio
