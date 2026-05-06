#include "scan_range_visualization_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QFontMetrics>
#include <QImage>
#include <QLinearGradient>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QRect>

namespace multi_radio {

namespace {
inline double Clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }
}  // namespace

ScanRangeVisualizationWidget::ScanRangeVisualizationWidget(QWidget* parent)
    : QWidget(parent) {
  setMinimumSize(200, 150);
}

void ScanRangeVisualizationWidget::Configure(double start_hz, double end_hz,
                                              double step_hz, int total_bins) {
  scan_start_hz_ = start_hz;
  scan_end_hz_ = end_hz;
  scan_step_hz_ = step_hz;
  total_bins_ = total_bins;
  sweep_buffer_.clear();
  latest_spectrum_.clear();
  waterfall_rows_.clear();
  last_tuned_hz_ = 0.0;
  update();
}

void ScanRangeVisualizationWidget::SetNoiseGate(bool enabled, double threshold) {
  noise_gate_enabled_ = enabled;
  noise_gate_threshold_ = threshold;
  update();
}

void ScanRangeVisualizationWidget::SetDbCeiling(double ceiling_db) {
  db_ceiling_ = std::clamp(ceiling_db, -90.0, -20.0);
  update();
}

QRect ScanRangeVisualizationWidget::SpectrumPlotRect() const {
  const QRect content = rect().adjusted(8, 8, -8, -8);
  const int spec_h = content.height() * 35 / 100;
  const QRect spec_rect(content.left(), content.top(), content.width(), spec_h);
  const QRect spec_content = spec_rect.adjusted(8, 28, -8, -8);
  return QRect(spec_content.left() + kLabelW, spec_content.top(),
               spec_content.width() - kLabelW, spec_content.height() - kFreqAxisH);
}

void ScanRangeVisualizationWidget::mousePressEvent(QMouseEvent* event) {
  if (event->button() != Qt::RightButton || scan_step_hz_ <= 0.0 ||
      scan_end_hz_ <= scan_start_hz_) {
    QWidget::mousePressEvent(event);
    return;
  }
  const QRect plot = SpectrumPlotRect();
  if (!plot.contains(event->pos())) {
    QWidget::mousePressEvent(event);
    return;
  }
  const double show_start = scan_start_hz_ - scan_step_hz_ * 0.5;
  const double show_end   = scan_end_hz_   + scan_step_hz_ * 0.5;
  const double t = static_cast<double>(event->pos().x() - plot.left()) /
                   std::max(1, plot.width() - 1);
  const double clicked_hz = show_start + t * (show_end - show_start);
  const double steps = std::round((clicked_hz - scan_start_hz_) / scan_step_hz_);
  const double snapped_hz = std::clamp(scan_start_hz_ + steps * scan_step_hz_,
                                       scan_start_hz_, scan_end_hz_);
  if (pending_range_start_hz_ < 0.0) {
    pending_range_start_hz_ = snapped_hz;
  } else {
    const double a = std::min(pending_range_start_hz_, snapped_hz);
    const double b = std::max(pending_range_start_hz_, snapped_hz);
    pending_range_start_hz_ = -1.0;
    emit RangeSelected(a, b);
  }
  update();
}

void ScanRangeVisualizationWidget::PushSpectrum(const std::vector<double>& spectrum,
                                                 double frame_start_hz,
                                                 double frame_end_hz,
                                                 double tuned_hz) {
  if (total_bins_ <= 0 || scan_step_hz_ <= 0.0 || scan_end_hz_ <= scan_start_hz_) {
    return;
  }

  const bool is_wrap = (last_tuned_hz_ > 0.0) &&
                       (tuned_hz < last_tuned_hz_ - scan_step_hz_ * 0.5);
  if (is_wrap && !sweep_buffer_.isEmpty()) {
    latest_spectrum_ = sweep_buffer_;
    waterfall_rows_.prepend(sweep_buffer_);
    if (waterfall_rows_.size() > kMaxRows) {
      waterfall_rows_.removeLast();
    }
    sweep_buffer_.fill(0.0, total_bins_);
  }
  if (sweep_buffer_.size() != total_bins_) {
    sweep_buffer_.fill(0.0, total_bins_);
  }
  last_tuned_hz_ = tuned_hz;

  const double full_start = scan_start_hz_ - scan_step_hz_ * 0.5;
  const double full_end = scan_end_hz_ + scan_step_hz_ * 0.5;
  const double full_span = full_end - full_start;
  const double frame_span = frame_end_hz - frame_start_hz;
  const int n_src = static_cast<int>(spectrum.size());
  if (full_span <= 0.0 || frame_span <= 0.0 || n_src == 0) {
    return;
  }

  for (int i = 0; i < n_src; ++i) {
    const double src_t = (n_src <= 1) ? 0.5 : (static_cast<double>(i) + 0.5) / n_src;
    const double bin_hz = frame_start_hz + src_t * frame_span;
    const double t = (bin_hz - full_start) / full_span;
    if (t < 0.0 || t >= 1.0) continue;
    const int dst = std::clamp(static_cast<int>(t * total_bins_), 0, total_bins_ - 1);
    sweep_buffer_[dst] = std::max(sweep_buffer_[dst], Clamp01(spectrum[i]));
  }

  update();
}

void ScanRangeVisualizationWidget::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(14, 18, 26));

  const QRect content = rect().adjusted(8, 8, -8, -8);
  if (content.isEmpty()) return;

  const int gap = 6;
  const int spec_h = content.height() * 35 / 100;
  const QRect spec_rect(content.left(), content.top(), content.width(), spec_h);
  const QRect wfall_rect(content.left(), content.top() + spec_h + gap, content.width(),
                         content.height() - spec_h - gap);

  const double show_start = scan_start_hz_ - scan_step_hz_ * 0.5;
  const double show_end = scan_end_hz_ + scan_step_hz_ * 0.5;

  auto draw_panel = [&](const QRect& r, const QString& title) {
    painter.setPen(QPen(QColor(66, 79, 102), 1));
    painter.setBrush(QColor(20, 27, 38));
    painter.drawRoundedRect(r, 6, 6);
    const QRect tr = r.adjusted(10, 6, -10, -(r.height() - 24));
    painter.setPen(QColor(178, 192, 214));
    painter.drawText(tr, Qt::AlignLeft | Qt::AlignVCenter, title);
  };

  draw_panel(spec_rect, QString("Spektrum [%1 – %2]")
                             .arg(FormatFreq(show_start))
                             .arg(FormatFreq(show_end)));
  draw_panel(wfall_rect, QString("Vattenfall [%1 – %2]")
                              .arg(FormatFreq(show_start))
                              .arg(FormatFreq(show_end)));

  const QRect spec_content = spec_rect.adjusted(8, 28, -8, -8);
  const QRect wfall_content = wfall_rect.adjusted(8, 28, -8, -8);

  if (total_bins_ <= 0) {
    painter.setPen(QColor(178, 192, 214));
    painter.drawText(content, Qt::AlignCenter, "Inga data ännu — starta scannen");
    return;
  }

  // Always draw the current in-progress sweep so each block appears immediately.
  if (!sweep_buffer_.isEmpty() && spec_content.isValid()) {
    DrawSpectrum(&painter, spec_content, sweep_buffer_);
  }
  if ((!sweep_buffer_.isEmpty() || !waterfall_rows_.isEmpty()) && wfall_content.isValid()) {
    DrawWaterfall(&painter, wfall_content, sweep_buffer_, waterfall_rows_);
  }
}

QColor ScanRangeVisualizationWidget::HeatColor(double v) {
  v = Clamp01(v);
  struct Stop {
    double pos;
    int r, g, b;
  };
  static constexpr Stop kStops[] = {
      {0.00, 0, 0, 0},     {0.20, 0, 0, 160},   {0.45, 0, 140, 255},
      {0.65, 0, 255, 255}, {0.82, 255, 255, 0},  {1.00, 255, 0, 0},
  };
  constexpr int kN = static_cast<int>(sizeof(kStops) / sizeof(kStops[0]));
  int lo = kN - 2;
  for (int i = 0; i < kN - 1; ++i) {
    if (kStops[i + 1].pos >= v) {
      lo = i;
      break;
    }
  }
  const auto& a = kStops[lo];
  const auto& b = kStops[lo + 1];
  const double t = (b.pos > a.pos) ? (v - a.pos) / (b.pos - a.pos) : 0.0;
  return QColor(static_cast<int>(a.r + t * (b.r - a.r)),
                static_cast<int>(a.g + t * (b.g - a.g)),
                static_cast<int>(a.b + t * (b.b - a.b)));
}

QString ScanRangeVisualizationWidget::FormatFreq(double hz) {
  const double abs_hz = std::abs(hz);
  if (abs_hz >= 1e6) return QString("%1M").arg(hz / 1e6, 0, 'f', 3);
  if (abs_hz >= 1e3) return QString("%1k").arg(hz / 1e3, 0, 'f', 1);
  return QString::number(hz, 'f', 0);
}

void ScanRangeVisualizationWidget::DrawSpectrum(QPainter* p, const QRect& rect,
                                                 const QVector<double>& spectrum) {
  if (rect.isEmpty() || spectrum.isEmpty()) return;

  // Data is normalized with kDataFloor=-90 → 0.0, kDataCeil=-20 → 1.0.
  // db_ceiling_ lets the user zoom the Y axis; values above it are clamped.
  constexpr double kDataFloor = -90.0;
  constexpr double kDataCeil  = -20.0;
  constexpr double kDataSpan  = kDataCeil - kDataFloor;  // 70 dB
  const double display_span = db_ceiling_ - kDataFloor;  // shrinks as ceiling lowers
  const double ceil_norm = display_span / kDataSpan;     // normalized ceiling in [0,1]

  const QRect plot(rect.left() + kLabelW, rect.top(),
                   rect.width() - kLabelW, rect.height() - kFreqAxisH);
  if (plot.isEmpty()) return;

  const int W = plot.width();
  const int H = plot.height();
  const int n = spectrum.size();

  // Map a normalized [0,1] data value to a y pixel, respecting the ceiling zoom.
  auto val_to_y = [&](double v) -> double {
    return plot.bottom() - Clamp01(v / ceil_norm) * (H - 1);
  };

  auto sample_at = [&](double t) -> double {
    const double pos = t * (n - 1);
    const int lo = std::clamp(static_cast<int>(pos), 0, n - 1);
    const int hi = std::clamp(lo + 1, 0, n - 1);
    return spectrum[lo] + (pos - lo) * (spectrum[hi] - spectrum[lo]);
  };

  // dBFS grid lines at every 10 dB, from floor up to the current ceiling.
  const QFontMetrics fm(p->font());
  for (int db = static_cast<int>(kDataFloor); db <= static_cast<int>(db_ceiling_); db += 10) {
    const double norm = (static_cast<double>(db) - kDataFloor) / kDataSpan;
    const int y = static_cast<int>(val_to_y(norm));
    p->setPen(QPen(QColor(50, 62, 84), 1));
    p->drawLine(plot.left(), y, plot.right(), y);
    const QString lbl = QString::number(db);
    const int lw = fm.horizontalAdvance(lbl);
    p->setPen(QColor(110, 128, 160));
    p->drawText(rect.left() + kLabelW - lw - 4, y + fm.ascent() / 2, lbl);
  }

  // Filled area under curve.
  QPainterPath fill_path;
  fill_path.moveTo(plot.left(), plot.bottom());
  for (int px = 0; px < W; ++px) {
    const double t = (W <= 1) ? 0.0 : static_cast<double>(px) / (W - 1);
    const double y = val_to_y(sample_at(t));
    if (px == 0) fill_path.lineTo(plot.left(), y);
    fill_path.lineTo(plot.left() + px, y);
  }
  fill_path.lineTo(plot.right(), plot.bottom());
  fill_path.closeSubpath();

  QLinearGradient grad(0, plot.top(), 0, plot.bottom());
  grad.setColorAt(0.0, QColor(0, 210, 190, 90));
  grad.setColorAt(1.0, QColor(0, 210, 190, 0));
  p->fillPath(fill_path, QBrush(grad));

  // Spectrum line.
  QPainterPath line_path;
  for (int px = 0; px < W; ++px) {
    const double t = (W <= 1) ? 0.0 : static_cast<double>(px) / (W - 1);
    const double y = val_to_y(sample_at(t));
    if (px == 0) line_path.moveTo(plot.left(), y);
    else line_path.lineTo(plot.left() + px, y);
  }
  p->setPen(QPen(QColor(0, 210, 190), 1.5));
  p->drawPath(line_path);

  // Pending range-start marker.
  if (pending_range_start_hz_ >= 0.0 && scan_step_hz_ > 0.0) {
    const double show_start = scan_start_hz_ - scan_step_hz_ * 0.5;
    const double show_end   = scan_end_hz_   + scan_step_hz_ * 0.5;
    const double span_show  = std::max(1.0, show_end - show_start);
    const double t = (pending_range_start_hz_ - show_start) / span_show;
    const int mx = plot.left() + static_cast<int>(t * (W - 1));
    if (mx >= plot.left() && mx <= plot.right()) {
      p->setPen(QPen(QColor(255, 220, 80), 1, Qt::DashLine));
      p->drawLine(mx, plot.top(), mx, plot.bottom());
      const QString lbl = FormatFreq(pending_range_start_hz_);
      p->setPen(QColor(255, 220, 80));
      p->drawText(mx + 3, plot.top() + fm.ascent() + 2, lbl);
    }
  }

  // Frequency axis ticks below the plot.
  p->setPen(QColor(100, 120, 155));
  const double span = scan_end_hz_ - scan_start_hz_;
  for (int i = 0; i <= 4; ++i) {
    const double frac = static_cast<double>(i) / 4.0;
    const double freq = (scan_start_hz_ - scan_step_hz_ * 0.5) + frac * (span + scan_step_hz_);
    const int px = plot.left() + static_cast<int>(frac * (W - 1));
    const QString label = FormatFreq(freq);
    const int lw = fm.horizontalAdvance(label);
    const int tx = std::clamp(px - lw / 2, plot.left(), plot.right() - lw);
    p->drawText(tx, rect.bottom(), label);
  }
}

void ScanRangeVisualizationWidget::DrawWaterfall(QPainter* p, const QRect& rect,
                                                  const QVector<double>& current_row,
                                                  const QVector<QVector<double>>& rows) {
  const bool have_current = !current_row.isEmpty();
  const int n_completed = rows.size();
  const int n_rows = n_completed + (have_current ? 1 : 0);
  if (rect.isEmpty() || n_rows == 0) return;

  const int n_bins = have_current ? current_row.size()
                                  : (n_completed > 0 ? rows[0].size() : 0);
  if (n_bins == 0) return;

  const int W = rect.width();
  const int H = rect.height();

  // Build QImage: row 0 = current in-progress sweep, rows 1..N = completed sweeps.
  const int img_w = std::min(W, 2048);
  QImage img(img_w, n_rows, QImage::Format_RGB32);

  auto draw_row = [&](int img_row, const QVector<double>& data) {
    const int n = data.size();
    if (n == 0) return;
    QRgb* line = reinterpret_cast<QRgb*>(img.scanLine(img_row));
    for (int px = 0; px < img_w; ++px) {
      const double t = (img_w <= 1) ? 0.0 : static_cast<double>(px) / (img_w - 1);
      const double pos = t * (n - 1);
      const int lo = std::clamp(static_cast<int>(pos), 0, n - 1);
      const int hi = std::clamp(lo + 1, 0, n - 1);
      const double v = data[lo] + (pos - lo) * (data[hi] - data[lo]);
      line[px] = (noise_gate_enabled_ && v < noise_gate_threshold_)
                     ? QColor(14, 18, 26).rgb()
                     : HeatColor(Clamp01(v)).rgb();
    }
  };

  if (have_current) {
    draw_row(0, current_row);
  }
  for (int i = 0; i < n_completed; ++i) {
    draw_row(have_current ? i + 1 : i, rows[i]);
  }

  p->setRenderHint(QPainter::SmoothPixmapTransform, false);
  p->drawImage(rect, img);

  // Frequency axis labels at bottom
  p->setRenderHint(QPainter::Antialiasing, true);
  p->setPen(QColor(100, 120, 155));
  const QFontMetrics fm(p->font());
  const double span = scan_end_hz_ - scan_start_hz_;
  for (int i = 0; i <= 4; ++i) {
    const double frac = static_cast<double>(i) / 4.0;
    const double freq = (scan_start_hz_ - scan_step_hz_ * 0.5) + frac * (span + scan_step_hz_);
    const int px = rect.left() + static_cast<int>(frac * (W - 1));
    const QString label = FormatFreq(freq);
    const int lw = fm.horizontalAdvance(label);
    const int tx = std::clamp(px - lw / 2, rect.left(), rect.right() - lw);
    p->drawText(tx, rect.bottom() - 2, label);
  }

  // Row count label (completed sweeps only)
  p->setPen(QColor(100, 120, 155));
  p->drawText(rect.adjusted(4, 0, -4, -14), Qt::AlignBottom | Qt::AlignRight,
              QString("%1 svep").arg(n_completed));
}

}  // namespace multi_radio
