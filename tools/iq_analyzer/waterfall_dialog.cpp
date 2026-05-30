#include "waterfall_dialog.hpp"
#include "dsp_utils.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

#include <QApplication>
#include <QFile>
#include <QFont>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QPainter>
#include <QPen>
#include <QScreen>
#include <QSizePolicy>
#include <QVBoxLayout>

namespace iq_analyzer {

// ---------------------------------------------------------------------------
// WaterfallWidget
// ---------------------------------------------------------------------------

WaterfallWidget::WaterfallWidget(QWidget* parent) : QWidget(parent) {
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMinimumSize(minimumSizeHint());
}

void WaterfallWidget::setWaterfall(const QImage& image,
                                    double freq_lo_hz, double freq_hi_hz,
                                    double t_start_s,  double t_end_s,
                                    double center_hz,  double bw_hz,
                                    double sig_t0_s,   double sig_t1_s) {
    img_       = image;
    freq_lo_hz_ = freq_lo_hz; freq_hi_hz_ = freq_hi_hz;
    t_start_s_  = t_start_s;  t_end_s_    = t_end_s;
    center_hz_  = center_hz;  bw_hz_      = bw_hz;
    sig_t0_s_   = sig_t0_s;   sig_t1_s_   = sig_t1_s;
    has_data_   = !image.isNull();
    update();
}

void WaterfallWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), Qt::black);

    const int iw = width()  - kLM - kRM;
    const int ih = height() - kTM - kBM;
    const QRect ir(kLM, kTM, iw, ih);
    if (iw <= 2 || ih <= 2) return;

    if (has_data_ && !img_.isNull())
        p.drawImage(ir, img_);

    const double fspan = freq_hi_hz_ - freq_lo_hz_;
    const double tspan = t_end_s_    - t_start_s_;

    auto freqToX = [&](double f) -> int {
        return kLM + static_cast<int>((f - freq_lo_hz_) / fspan * iw);
    };
    auto timeToY = [&](double t) -> int {
        return kTM + static_cast<int>((t - t_start_s_) / tspan * ih);
    };

    // --- Channel boundaries (white dashed) ---
    if (bw_hz_ > 0.0 && fspan > 0.0) {
        p.setPen(QPen(QColor(230, 230, 230, 170), 1, Qt::DashLine));
        const int n_lo = static_cast<int>(std::floor((freq_lo_hz_ - center_hz_) / bw_hz_)) - 1;
        const int n_hi = static_cast<int>(std::ceil( (freq_hi_hz_ - center_hz_) / bw_hz_)) + 1;
        for (int n = n_lo; n <= n_hi; ++n) {
            const double bnd = center_hz_ + (static_cast<double>(n) + 0.5) * bw_hz_;
            if (bnd < freq_lo_hz_ || bnd > freq_hi_hz_) continue;
            const int x = freqToX(bnd);
            p.drawLine(x, kTM, x, kTM + ih);
        }
    }

    // --- Signal start / end (yellow dotted) ---
    if (has_data_ && tspan > 0.0) {
        p.setPen(QPen(QColor(255, 220, 30, 220), 1, Qt::DotLine));
        for (double t : {sig_t0_s_, sig_t1_s_}) {
            if (t < t_start_s_ || t > t_end_s_) continue;
            const int y = timeToY(t);
            p.drawLine(kLM, y, kLM + iw, y);
        }
    }

    // --- Border ---
    p.setPen(QColor(70, 70, 70));
    p.drawRect(ir.adjusted(0, 0, -1, -1));

    // --- Axis labels ---
    QFont f; f.setPointSize(8); p.setFont(f);
    p.setPen(Qt::white);

    // Time labels on left (5 ticks)
    if (tspan > 0.0) {
        for (int ti = 0; ti <= 5; ++ti) {
            const double t = t_start_s_ + ti * tspan / 5.0;
            const int y = timeToY(t);
            if (y < kTM || y > kTM + ih) continue;
            p.drawText(0, y - 7, kLM - 4, 14, Qt::AlignRight | Qt::AlignVCenter,
                       QString::number(t, 'f', 2) + "s");
            p.drawLine(kLM - 3, y, kLM, y);
        }
    }

    // Freq labels on bottom (5 ticks)
    if (fspan > 0.0) {
        auto fmtF = [](double hz) -> QString {
            if (std::abs(hz) >= 1.0e6) return QString::number(hz / 1.0e6, 'f', 3) + "M";
            if (std::abs(hz) >= 1.0e3) return QString::number(hz / 1.0e3, 'f', 1) + "k";
            return QString::number(hz, 'f', 0);
        };
        for (int fi = 0; fi <= 5; ++fi) {
            const double freq = freq_lo_hz_ + fi * fspan / 5.0;
            const int x = freqToX(freq);
            p.drawText(x - 28, height() - kBM + 3, 56, kBM - 3,
                       Qt::AlignCenter, fmtF(freq));
            p.drawLine(x, kTM + ih, x, kTM + ih + 3);
        }
    }
}

// ---------------------------------------------------------------------------
// WaterfallWorker
// ---------------------------------------------------------------------------

WaterfallWorker::WaterfallWorker(QString path,
                                  double center_hz, double sample_rate_hz,
                                  double channel_bw_hz,
                                  double freq_lo_hz, double freq_hi_hz,
                                  double t_start_s,  double t_end_s,
                                  int    fft_size,
                                  QObject* parent)
    : QObject(parent),
      path_(std::move(path)),
      center_hz_(center_hz), sample_rate_hz_(sample_rate_hz), channel_bw_hz_(channel_bw_hz),
      freq_lo_hz_(freq_lo_hz), freq_hi_hz_(freq_hi_hz),
      t_start_s_(t_start_s), t_end_s_(t_end_s),
      fft_size_(fft_size) {}

void WaterfallWorker::run() {
    auto fail = [&]() {
        emit dataReady({}, 0, 0, freq_lo_hz_, freq_hi_hz_, t_start_s_, t_end_s_);
    };

    QFile file(path_);
    if (!file.open(QIODevice::ReadOnly)) { fail(); return; }

    const qint64 total_samples = file.size() / 4;
    const double total_dur = static_cast<double>(total_samples) / sample_rate_hz_;

    const double t0 = std::max(0.0, t_start_s_);
    const double t1 = std::min(total_dur, t_end_s_);
    if (t1 <= t0 + 1.0e-6) { fail(); return; }

    const qint64 samp0  = static_cast<qint64>(t0 * sample_rate_hz_);
    const qint64 samp1  = static_cast<qint64>(t1 * sample_rate_hz_);
    const qint64 n_samp = samp1 - samp0;
    if (n_samp < fft_size_) { fail(); return; }

    file.seek(samp0 * 4);
    const QByteArray raw = file.read(n_samp * 4);
    file.close();

    const size_t N    = static_cast<size_t>(fft_size_);
    const size_t half = N / 2U;
    const double bin_hz = sample_rate_hz_ / static_cast<double>(N);

    const int lo_s = static_cast<int>(
        std::floor((freq_lo_hz_ - center_hz_) / bin_hz + static_cast<double>(half)));
    const int hi_s = static_cast<int>(
        std::ceil( (freq_hi_hz_ - center_hz_) / bin_hz + static_cast<double>(half)));
    const int clo = std::max(0, lo_s);
    const int chi = std::min(static_cast<int>(N), hi_s);
    const int n_bins = chi - clo;
    if (n_bins <= 0) { fail(); return; }

    const size_t num_pairs  = static_cast<size_t>(raw.size()) / 4U;
    const size_t num_frames = num_pairs / N;
    if (num_frames == 0) { fail(); return; }

    const auto* raw_i16 = reinterpret_cast<const int16_t*>(raw.constData());

    // Compute flat power matrix [frame * n_bins + bin] in dBFS.
    QVector<float> power_flat(static_cast<int>(num_frames) * n_bins, -200.f);
    std::vector<std::complex<double>> fft_buf(N);

    for (size_t frame = 0; frame < num_frames; ++frame) {
        const size_t base = frame * N;
        for (size_t i = 0; i < N; ++i) {
            const double iv = static_cast<double>(raw_i16[(base + i) * 2])     * kNormScale;
            const double qv = static_cast<double>(raw_i16[(base + i) * 2 + 1]) * kNormScale;
            fft_buf[i] = std::complex<double>(iv, qv) * HannWindow(i, N);
        }
        FftRadix2InPlace(fft_buf);

        // Use same formula as frontend: 20·log10(|FFT[k]| / N)
        const int row_off = static_cast<int>(frame) * n_bins;
        for (int bi = 0; bi < n_bins; ++bi) {
            const int s = clo + bi;
            const int k = (s - static_cast<int>(half) + static_cast<int>(N)) % static_cast<int>(N);
            const double magnitude = std::abs(fft_buf[static_cast<size_t>(k)]) /
                                     static_cast<double>(N);
            power_flat[row_off + bi] =
                static_cast<float>(20.0 * std::log10(std::max(magnitude, 1.0e-12)));
        }

        if ((frame % 100) == 0)
            emit progress(static_cast<int>(frame * 95 / num_frames));
    }

    emit progress(100);
    emit dataReady(power_flat, n_bins, static_cast<int>(num_frames),
                   freq_lo_hz_, freq_hi_hz_, t0, t1);
}

// ---------------------------------------------------------------------------
// WaterfallDialog
// ---------------------------------------------------------------------------

namespace {
constexpr int kNeighborChannels = 2;

QString FmtFreq(double hz) {
    if (std::abs(hz) >= 1.0e6) return QString::number(hz / 1.0e6, 'f', 4) + " MHz";
    if (std::abs(hz) >= 1.0e3) return QString::number(hz / 1.0e3, 'f', 3) + " kHz";
    return QString::number(hz, 'f', 1) + " Hz";
}
}  // namespace

WaterfallDialog::WaterfallDialog(QString file_path, DetectedSignal signal,
                                  double center_hz, double sample_rate_hz,
                                  double channel_bw_hz, QWidget* parent)
    : QDialog(parent, Qt::Window),
      file_path_(std::move(file_path)),
      signal_(signal),
      center_hz_(center_hz),
      sample_rate_hz_(sample_rate_hz),
      channel_bw_hz_(channel_bw_hz) {
    const int ch_n = static_cast<int>(
        std::round((signal_.center_freq_hz - center_hz_) / channel_bw_hz_));
    setWindowTitle(QString("Vattenfall – %1  (kanal %2)")
                       .arg(FmtFreq(signal_.center_freq_hz)).arg(ch_n));
    setAttribute(Qt::WA_DeleteOnClose);

    auto* root = new QVBoxLayout(this);
    root->setSpacing(6);
    root->setContentsMargins(8, 8, 8, 8);

    // Time range row
    auto* ctrl = new QHBoxLayout();
    pre_spin_  = new QDoubleSpinBox(this);
    pre_spin_->setRange(0.0, 600.0); pre_spin_->setDecimals(2);
    pre_spin_->setValue(1.0); pre_spin_->setSuffix(" s");
    post_spin_ = new QDoubleSpinBox(this);
    post_spin_->setRange(0.0, 600.0); post_spin_->setDecimals(2);
    post_spin_->setValue(1.0); post_spin_->setSuffix(" s");
    load_btn_  = new QPushButton("Ladda", this);
    prog_bar_  = new QProgressBar(this);
    prog_bar_->setRange(0, 100); prog_bar_->setMaximumWidth(160);
    prog_bar_->setVisible(false);
    status_lbl_ = new QLabel(this);

    neighbors_spin_ = new QSpinBox(this);
    neighbors_spin_->setRange(-1, 100);
    neighbors_spin_->setValue(2);
    neighbors_spin_->setSpecialValueText("Hela bredden");
    neighbors_spin_->setToolTip("-1 = hela samplingsbredden, 0 = enbart vald kanal");

    ctrl->addWidget(new QLabel("Tid före:", this));
    ctrl->addWidget(pre_spin_);
    ctrl->addWidget(new QLabel("Tid efter:", this));
    ctrl->addWidget(post_spin_);
    ctrl->addWidget(new QLabel("Grannkanaler:", this));
    ctrl->addWidget(neighbors_spin_);
    ctrl->addWidget(load_btn_);
    ctrl->addWidget(prog_bar_);
    ctrl->addWidget(status_lbl_, 1);
    root->addLayout(ctrl);

    // FFT size + threshold row
    auto* thr_row = new QHBoxLayout();
    fft_size_combo_ = new QComboBox(this);
    for (int s : {256, 512, 1024, 2048, 4096, 8192, 16384})
        fft_size_combo_->addItem(QString::number(s), s);
    fft_size_combo_->setCurrentIndex(fft_size_combo_->findData(kDefaultFftSize));
    thr_row->addWidget(new QLabel("FFT-storlek:", this));
    thr_row->addWidget(fft_size_combo_);
    thr_row->addSpacing(16);
    // Floor ("blå under")
    floor_spin_ = new QDoubleSpinBox(this);
    floor_spin_->setRange(-200.0, 0.0);
    floor_spin_->setDecimals(1);
    floor_spin_->setSingleStep(1.0);
    floor_spin_->setSuffix(" dBFS");
    floor_spin_->setValue(signal_.noise_floor_dbfs);
    floor_spin_->setToolTip("Pixlar under denna nivå visas i svart; nollpunkt för färgskalan");
    thr_row->addWidget(new QLabel("Golv:", this));
    thr_row->addWidget(floor_spin_);
    thr_row->addStretch(1);
    root->addLayout(thr_row);

    wf_widget_ = new WaterfallWidget(this);
    root->addWidget(wf_widget_, 1);

    // Size cap
    const QScreen* screen = QApplication::primaryScreen();
    if (screen) {
        const QRect av = screen->availableGeometry();
        const int mw   = static_cast<int>(av.width()  * 0.92);
        const int mh   = static_cast<int>(av.height() * 0.92);
        resize(std::min(820, mw), std::min(640, mh));
        setMaximumSize(mw, mh);
    } else {
        resize(820, 640);
    }

    qRegisterMetaType<QVector<float>>();

    connect(load_btn_,        &QPushButton::clicked,          this, &WaterfallDialog::onLoad);
    connect(floor_spin_,     &QDoubleSpinBox::valueChanged,  this, [this](double) { buildAndShowImage(); });
    connect(fft_size_combo_, &QComboBox::currentIndexChanged,this, [this](int) { onLoad(); });
    connect(neighbors_spin_, &QSpinBox::valueChanged,        this, [this](int)  { onLoad(); });

    // Trigger initial load after the event loop starts.
    QMetaObject::invokeMethod(this, &WaterfallDialog::onLoad, Qt::QueuedConnection);
}

void WaterfallDialog::onLoad() {
    if (is_loading_) return;

    const double pre  = pre_spin_->value();
    const double post = post_spin_->value();
    const int ch_n    = static_cast<int>(
        std::round((signal_.center_freq_hz - center_hz_) / channel_bw_hz_));
    const int neighbors = neighbors_spin_->value();

    double freq_lo, freq_hi;
    if (neighbors < 0) {
        // Show the full sampling bandwidth.
        freq_lo = center_hz_ - sample_rate_hz_ / 2.0;
        freq_hi = center_hz_ + sample_rate_hz_ / 2.0;
    } else {
        freq_lo = center_hz_ + (static_cast<double>(ch_n - neighbors) - 0.5) * channel_bw_hz_;
        freq_hi = center_hz_ + (static_cast<double>(ch_n + neighbors) + 0.5) * channel_bw_hz_;
    }
    const double t_start = signal_.time_offset_s - pre;
    const double t_end   = signal_.time_offset_s + signal_.duration_s + post;

    is_loading_ = true;
    load_btn_->setEnabled(false);
    prog_bar_->setValue(0);
    prog_bar_->setVisible(true);
    status_lbl_->setText("Laddar…");

    const int fft_size = fft_size_combo_->currentData().toInt();
    auto* worker = new WaterfallWorker(file_path_, center_hz_, sample_rate_hz_,
                                        channel_bw_hz_, freq_lo, freq_hi,
                                        t_start, t_end, fft_size);
    auto* thread = new QThread();
    worker->moveToThread(thread);

    connect(thread, &QThread::started,            worker, &WaterfallWorker::run);
    connect(worker, &WaterfallWorker::progress,   this,   &WaterfallDialog::onProgress);
    connect(worker, &WaterfallWorker::dataReady,  this,   &WaterfallDialog::onDataReady);
    connect(worker, &WaterfallWorker::dataReady,  worker, &QObject::deleteLater);
    connect(worker, &WaterfallWorker::dataReady,  thread, &QThread::quit);
    connect(thread, &QThread::finished,           thread, &QObject::deleteLater);

    thread->start();
}

void WaterfallDialog::onProgress(int pct) {
    prog_bar_->setValue(pct);
}

void WaterfallDialog::onDataReady(QVector<float> power_flat, int n_bins, int n_frames,
                                   double freq_lo_hz, double freq_hi_hz,
                                   double t_start_s,  double t_end_s) {
    is_loading_ = false;
    prog_bar_->setVisible(false);
    load_btn_->setEnabled(true);

    if (power_flat.isEmpty() || n_bins <= 0 || n_frames <= 0) {
        status_lbl_->setText("Kunde inte läsa data.");
        return;
    }

    power_flat_    = std::move(power_flat);
    power_n_bins_  = n_bins;
    power_n_frames_= n_frames;
    power_freq_lo_ = freq_lo_hz;
    power_freq_hi_ = freq_hi_hz;
    power_t_start_ = t_start_s;
    power_t_end_   = t_end_s;

    status_lbl_->setText(
        QString("%1 bildramar | %2 – %3")
            .arg(n_frames)
            .arg(FmtFreq(freq_lo_hz))
            .arg(FmtFreq(freq_hi_hz)));

    buildAndShowImage();
}

void WaterfallDialog::buildAndShowImage() {
    if (power_flat_.isEmpty()) return;

    // Absolute dBFS scale: floor set by user, ceiling fixed at 0 dBFS
    // (the theoretical max for normalized IQ data). v > 1 → white via LUT.
    const float floor_db   = static_cast<float>(floor_spin_->value());
    constexpr float ceiling_db = 0.0f;
    const float span = std::max(ceiling_db - floor_db, 0.1f);

    QImage img(power_n_bins_, power_n_frames_, QImage::Format_RGB32);
    for (int frame = 0; frame < power_n_frames_; ++frame) {
        auto* line = reinterpret_cast<QRgb*>(img.scanLine(frame));
        const int row_off = frame * power_n_bins_;
        for (int bi = 0; bi < power_n_bins_; ++bi) {
            const float pw = power_flat_[row_off + bi];
            if (pw < floor_db) {
                line[bi] = qRgb(0, 0, 0);
            } else {
                // v=0 at floor (dark blue LUT[0]), v=1 at ceiling (white),
                // v>1 for overload (clamped to white by LUT).
                line[bi] = WaterfallColor((pw - floor_db) / span);
            }
        }
    }

    wf_widget_->setWaterfall(img, power_freq_lo_, power_freq_hi_,
                              power_t_start_, power_t_end_,
                              center_hz_, channel_bw_hz_,
                              signal_.time_offset_s,
                              signal_.time_offset_s + signal_.duration_s);
}

}  // namespace iq_analyzer
