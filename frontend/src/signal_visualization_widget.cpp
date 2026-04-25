#include "signal_visualization_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QColor>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QRect>

namespace multi_radio {

namespace {

constexpr int kWaveformPoints = 200;
constexpr int kSpectrumBins = 120;
constexpr int kSpectrogramRows = 34;
constexpr int kWaterfallRows = 90;
constexpr double kSpectrumSpanHz = 20000.0;
constexpr double kTwoPi = 6.28318530717958647692;

inline double Clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
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
  draw_panel(spectrogram_rect, "Spectrogram (Freq vs dB)");
  draw_panel(waterfall_rect, "Waterfall");

  const bool needs_selection = RequireExplicitSelection();
  if (needs_selection) {
    painter.setPen(QColor(212, 220, 235));
    painter.drawText(content, Qt::AlignCenter,
                     "Välj en mottagare i Message Filters för att styra visualiseringarna.");
    return;
  }

  const ReceiverState display = BuildDisplayState();

  DrawWaveform(&painter, waveform_rect.adjusted(8, 30, -8, -8), display.waveform);
  DrawSpectrumCurve(&painter, spectrogram_rect.adjusted(8, 30, -8, -8), display.spectrum);
  DrawHeatmap(&painter, waterfall_rect.adjusted(8, 30, -8, -8), display.waterfall_rows, true, true);
}

void SignalVisualizationWidget::OnFrameTick() {
  for (auto it = states_.begin(); it != states_.end(); ++it) {
    DecayState(&it.value(), 0.94);
  }
  update();
}

void SignalVisualizationWidget::EnsureState(ReceiverState* state) {
  if (state == nullptr) {
    return;
  }
  if (state->waveform.isEmpty()) {
    state->waveform = QVector<double>(kWaveformPoints, 0.5);
  }
  if (state->spectrum.isEmpty()) {
    state->spectrum = QVector<double>(kSpectrumBins, 0.0);
  }
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
                                                  const QVector<double>& spectrum) {
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

  const int freq_ticks_khz[] = {0, 5, 10, 15, 20};
  for (const int tick_khz : freq_ticks_khz) {
    const double t = static_cast<double>(tick_khz) / 20.0;
    const int x = plot.left() + static_cast<int>(t * (plot.width() - 1));
    painter->drawLine(x, plot.top(), x, plot.bottom());
    painter->setPen(QColor(160, 176, 200));
    painter->drawText(x - 14, area.bottom() - 4, QString("%1k").arg(tick_khz));
    painter->setPen(QPen(QColor(48, 58, 78), 1));
  }

  painter->setPen(QColor(178, 192, 214));
  painter->drawText(area.left() + 2, plot.top() - 2, "dB");
  painter->drawText(plot.right() - 56, area.bottom() - 4, "Freq (kHz)");

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
      return states_.value(id);
    }
  }

  if (known_receivers_.size() == 1 && states_.contains(known_receivers_[0])) {
    return states_.value(known_receivers_[0]);
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
  if (state->last_frequency_hz > 0.0) {
    normalized_frequency = std::fmod(state->last_frequency_hz, kSpectrumSpanHz) / kSpectrumSpanHz;
  }

  for (double& bin : state->spectrum) {
    bin *= 0.9;
  }

  const double sigma = 0.045;
  for (int i = 0; i < state->spectrum.size(); ++i) {
    const double x = (state->spectrum.size() <= 1)
                         ? 0.0
                         : static_cast<double>(i) / static_cast<double>(state->spectrum.size() - 1);
    const double dist = x - normalized_frequency;
    const double gaussian = std::exp(-(dist * dist) / (2.0 * sigma * sigma));
    state->spectrum[i] = std::max(state->spectrum[i], gaussian * normalized_intensity);
  }

  const int samples_to_add = 4;
  const double amplitude = 0.15 + normalized_intensity * 0.75;
  for (int i = 0; i < samples_to_add; ++i) {
    state->phase += 0.16 + normalized_intensity * 0.12;
    const double harmonic = std::sin(state->phase * kTwoPi);
    const double overtone = std::sin((state->phase * 0.51 + normalized_frequency) * kTwoPi);
    const double sample = 0.5 + amplitude * 0.38 * harmonic + 0.12 * overtone;
    state->waveform.removeFirst();
    state->waveform.push_back(Clamp01(sample));
  }

  PushRow(&state->spectrogram_rows, state->spectrum, kSpectrogramRows);
  PushRow(&state->waterfall_rows, state->spectrum, kWaterfallRows);
}

void SignalVisualizationWidget::DecayState(ReceiverState* state, double decay_factor) {
  if (state == nullptr) {
    return;
  }
  EnsureState(state);

  for (double& bin : state->spectrum) {
    bin = Clamp01(bin * decay_factor);
  }

  double peak = 0.0;
  for (const double value : std::as_const(state->spectrum)) {
    peak = std::max(peak, value);
  }

  state->phase += 0.06;
  const double sample = 0.5 + std::sin(state->phase * kTwoPi) * (0.08 + peak * 0.25);
  state->waveform.removeFirst();
  state->waveform.push_back(Clamp01(sample));

  PushRow(&state->spectrogram_rows, state->spectrum, kSpectrogramRows);
  PushRow(&state->waterfall_rows, state->spectrum, kWaterfallRows);
}

}  // namespace multi_radio
