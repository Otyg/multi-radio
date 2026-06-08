#include "demod_dialog.hpp"
#include "dsp_utils.hpp"
#include "plugin_chain.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numeric>
#include <vector>

#include <QApplication>
#include <QAudioDevice>
#include <QAudioFormat>
#include <QFile>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QMediaDevices>
#include <QPainter>
#include <QScreen>
#include <QSizePolicy>
#include <QVBoxLayout>

namespace iq_analyzer {

// ---------------------------------------------------------------------------
// DSP helpers (file-local)
// ---------------------------------------------------------------------------
namespace {

constexpr double kAudioRate = 48000.0;
constexpr int    kWfFftSize = 512;

// Mix IQ to baseband and apply single-pole IIR low-pass filter.
void mixAndFilter(const int16_t* raw, size_t n_pairs,
                  double f_offset, double fs, double cutoff_hz,
                  std::vector<double>& I_out, std::vector<double>& Q_out) {
    I_out.resize(n_pairs);
    Q_out.resize(n_pairs);
    const double ps = -2.0 * kPi * f_offset / fs;
    const double dt  = 1.0 / fs;
    const double rc  = (cutoff_hz > 0) ? 1.0 / (2.0 * kPi * cutoff_hz) : 0.0;
    const double alpha = (rc > 0) ? dt / (rc + dt) : 1.0;
    double yi = 0, yq = 0;
    for (size_t n = 0; n < n_pairs; ++n) {
        const double i = raw[n * 2]     * kNormScale;
        const double q = raw[n * 2 + 1] * kNormScale;
        const double ph = ps * static_cast<double>(n);
        const double c = std::cos(ph), s = std::sin(ph);
        yi = alpha * (i * c - q * s) + (1.0 - alpha) * yi;
        yq = alpha * (i * s + q * c) + (1.0 - alpha) * yq;
        I_out[n] = yi;
        Q_out[n] = yq;
    }
}

// Decimate vectors by integer factor D.
void decimateInPlace(std::vector<double>& v, int D) {
    if (D <= 1) return;
    size_t out = 0;
    for (size_t n = 0; n < v.size(); n += static_cast<size_t>(D))
        v[out++] = v[n];
    v.resize(out);
}

// FM discriminator (phase difference).
std::vector<float> demodFm(const std::vector<double>& I, const std::vector<double>& Q,
                             double fs_dec, double max_dev) {
    const size_t n = I.size();
    std::vector<float> out(n, 0.f);
    const double scale = (max_dev > 0) ? fs_dec / (2.0 * kPi * max_dev) : 1.0;
    double pi = 1.0, pq = 0.0;
    for (size_t k = 0; k < n; ++k) {
        const double ci = I[k], cq = Q[k];
        const double num = ci * pq - cq * pi;
        const double den = ci * ci  + cq * cq + 1.0e-30;
        out[k] = static_cast<float>(std::clamp(num / den * scale, -1.0, 1.0));
        pi = ci; pq = cq;
    }
    return out;
}

// AM: envelope with DC removed.
std::vector<float> demodAm(const std::vector<double>& I, const std::vector<double>& Q) {
    const size_t n = I.size();
    std::vector<float> out(n);
    double sum = 0;
    for (size_t k = 0; k < n; ++k) {
        out[k] = static_cast<float>(std::sqrt(I[k]*I[k] + Q[k]*Q[k]));
        sum += out[k];
    }
    const float mean = static_cast<float>(sum / static_cast<double>(n));
    for (float& x : out) x -= mean;
    return out;
}

// USB/LSB: I component (simplified; caller mixes to upper or lower side).
std::vector<float> demodSsb(const std::vector<double>& I) {
    std::vector<float> out(I.size());
    for (size_t k = 0; k < I.size(); ++k)
        out[k] = static_cast<float>(I[k]);
    return out;
}

// Linear resample to target size.
std::vector<float> resampleTo(const std::vector<float>& src, size_t dst_n) {
    if (src.size() == dst_n) return src;
    std::vector<float> dst(dst_n);
    const double ratio = static_cast<double>(src.size() - 1) /
                         std::max<double>(dst_n - 1, 1);
    for (size_t i = 0; i < dst_n; ++i) {
        const double pos = i * ratio;
        const size_t lo  = static_cast<size_t>(pos);
        const size_t hi  = std::min(lo + 1, src.size() - 1);
        const double t   = pos - lo;
        dst[i] = static_cast<float>(src[lo] + t * (src[hi] - src[lo]));
    }
    return dst;
}

// Compute power spectrum (dBFS per bin) from real audio samples.
// Returns flat row-major array [frame * n_bins + bin].
QVector<float> computeAudioPdb(const std::vector<float>& audio,
                                 int& out_n_bins, int& out_n_frames) {
    const size_t n_samp   = audio.size();
    const size_t n_frames = n_samp / static_cast<size_t>(kWfFftSize);
    const size_t n_bins   = static_cast<size_t>(kWfFftSize) / 2U;
    out_n_bins   = static_cast<int>(n_bins);
    out_n_frames = static_cast<int>(n_frames);
    if (n_frames == 0) return {};

    QVector<float> flat(static_cast<int>(n_frames * n_bins), -200.f);
    std::vector<std::complex<double>> buf(static_cast<size_t>(kWfFftSize));

    for (size_t fr = 0; fr < n_frames; ++fr) {
        const size_t base = fr * static_cast<size_t>(kWfFftSize);
        for (int i = 0; i < kWfFftSize; ++i) {
            const double w = HannWindow(static_cast<size_t>(i),
                                        static_cast<size_t>(kWfFftSize));
            buf[static_cast<size_t>(i)] = {audio[base + i] * w, 0.0};
        }
        FftRadix2InPlace(buf);
        const int row_off = static_cast<int>(fr) * static_cast<int>(n_bins);
        for (size_t b = 0; b < n_bins; ++b) {
            const double mag = std::abs(buf[b]) / kWfFftSize;
            flat[row_off + static_cast<int>(b)] =
                static_cast<float>(20.0 * std::log10(std::max(mag, 1.0e-12)));
        }
    }
    return flat;
}

}  // namespace

// ---------------------------------------------------------------------------
// DemodWaterfallWidget
// ---------------------------------------------------------------------------

DemodWaterfallWidget::DemodWaterfallWidget(QWidget* parent) : QWidget(parent) {
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMinimumSize(minimumSizeHint());
}

void DemodWaterfallWidget::setData(const QImage& img, double freq_hi_hz,
                                    double t_start_s, double t_end_s) {
    img_       = img;
    freq_hi_hz_ = freq_hi_hz;
    t_start_s_ = t_start_s;
    t_end_s_   = t_end_s;
    has_data_  = !img.isNull();
    update();
}

void DemodWaterfallWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), Qt::black);

    const int iw = width()  - kLM - kRM;
    const int ih = height() - kTM - kBM;
    if (iw <= 2 || ih <= 2) return;
    const QRect ir(kLM, kTM, iw, ih);

    if (has_data_ && !img_.isNull())
        p.drawImage(ir, img_);

    p.setPen(QColor(70, 70, 70));
    p.drawRect(ir.adjusted(0, 0, -1, -1));

    QFont f; f.setPointSize(8); p.setFont(f);
    p.setPen(Qt::white);

    const double tspan = t_end_s_ - t_start_s_;
    if (tspan > 0.0) {
        for (int ti = 0; ti <= 5; ++ti) {
            const double t = t_start_s_ + ti * tspan / 5.0;
            const int y = kTM + static_cast<int>(ti * ih / 5.0);
            p.drawText(0, y - 7, kLM - 3, 14, Qt::AlignRight | Qt::AlignVCenter,
                       QString::number(t, 'f', 1) + "s");
            p.drawLine(kLM - 3, y, kLM, y);
        }
    }

    if (freq_hi_hz_ > 0.0) {
        auto fmtHz = [](double hz) -> QString {
            return hz >= 1000 ? QString::number(hz / 1000.0, 'f', 1) + "k"
                              : QString::number(hz, 'f', 0);
        };
        for (int fi = 0; fi <= 5; ++fi) {
            const double freq = fi * freq_hi_hz_ / 5.0;
            const int x = kLM + static_cast<int>(fi * iw / 5.0);
            p.drawText(x - 22, height() - kBM + 3, 44, kBM - 3,
                       Qt::AlignCenter, fmtHz(freq));
            p.drawLine(x, kTM + ih, x, kTM + ih + 3);
        }
    }
}

// ---------------------------------------------------------------------------
// AudioWaveformWidget
// ---------------------------------------------------------------------------

AudioWaveformWidget::AudioWaveformWidget(QWidget* parent) : QWidget(parent) {
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    setMinimumSize(minimumSizeHint());
}

void AudioWaveformWidget::setWaveform(const QVector<float>& samples,
                                       double t_start_s, double t_end_s) {
    samples_   = samples;
    t_start_s_ = t_start_s;
    t_end_s_   = t_end_s;
    update();
}

void AudioWaveformWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor(15, 15, 15));

    const int iw = width()  - kLM - kRM;
    const int ih = height() - kTM - kBM;
    if (iw <= 2 || ih <= 2) return;

    p.setPen(QColor(70, 70, 70));
    p.drawRect(QRect(kLM, kTM, iw, ih).adjusted(0, 0, -1, -1));

    // Zero reference
    const int cx = kLM + iw / 2;
    p.setPen(QPen(QColor(70, 70, 70), 1, Qt::DotLine));
    p.drawLine(cx, kTM, cx, kTM + ih);

    if (!samples_.isEmpty() && ih > 0) {
        const int n  = samples_.size();
        const int hw = iw / 2 - 1;
        QVector<QPoint> pts;
        pts.reserve(n);
        for (int i = 0; i < n; ++i) {
            const int y = kTM + i * ih / n;
            const int x = cx + static_cast<int>(
                std::clamp(samples_[i], -1.0f, 1.0f) * hw);
            pts.append(QPoint(x, y));
        }
        p.setPen(QPen(QColor(180, 220, 180), 1));
        p.drawPolyline(pts.constData(), pts.size());
    }

    // Time axis (right side of widget)
    QFont f; f.setPointSize(7); p.setFont(f);
    p.setPen(Qt::white);
    const double tspan = t_end_s_ - t_start_s_;
    if (tspan > 0.0) {
        for (int ti = 0; ti <= 4; ++ti) {
            const double t = t_start_s_ + ti * tspan / 4.0;
            const int y = kTM + ti * ih / 4;
            p.drawText(kLM + iw + 1, y - 6, 40, 12,
                       Qt::AlignLeft | Qt::AlignVCenter,
                       QString::number(t, 'f', 1));
        }
    }

    // Freq label at bottom
    p.drawText(0, height() - kBM + 3, width(), kBM - 3,
               Qt::AlignCenter, "tid (s)");
}

// ---------------------------------------------------------------------------
// DemodWorker
// ---------------------------------------------------------------------------

DemodWorker::DemodWorker(QString path,
                          double file_center_hz, double sample_rate_hz,
                          double channel_center_hz, double channel_bw_hz,
                          double t_start_s, double t_end_s,
                          Mod mod, QObject* parent)
    : QObject(parent),
      path_(std::move(path)),
      file_center_hz_(file_center_hz), sample_rate_hz_(sample_rate_hz),
      channel_center_hz_(channel_center_hz), channel_bw_hz_(channel_bw_hz),
      t_start_s_(t_start_s), t_end_s_(t_end_s), mod_(mod) {}

void DemodWorker::run() {
    auto fail = [&]() {
        emit done({}, {}, 0, 0, t_start_s_, t_end_s_, {});
    };

    QFile file(path_);
    if (!file.open(QIODevice::ReadOnly)) { fail(); return; }

    const qint64 total_samp = file.size() / 4;
    const double total_dur  = static_cast<double>(total_samp) / sample_rate_hz_;
    const double t0 = std::max(0.0, t_start_s_);
    const double t1 = std::min(total_dur, t_end_s_);
    if (t1 <= t0 + 1.0e-6) { fail(); return; }

    const qint64 s0    = static_cast<qint64>(t0 * sample_rate_hz_);
    const qint64 s1    = static_cast<qint64>(t1 * sample_rate_hz_);
    const qint64 n_req = s1 - s0;
    file.seek(s0 * 4);
    const QByteArray raw = file.read(n_req * 4);
    file.close();

    const size_t n_pairs = static_cast<size_t>(raw.size()) / 4U;
    const auto* data = reinterpret_cast<const int16_t*>(raw.constData());

    emit progress(8);

    // Mix to baseband and low-pass filter.
    const double f_off  = channel_center_hz_ - file_center_hz_;
    const double cutoff = channel_bw_hz_ / 2.0;
    std::vector<double> sig_i, sig_q;
    mixAndFilter(data, n_pairs, f_off, sample_rate_hz_, cutoff, sig_i, sig_q);

    emit progress(35);

    // Decimate to audio rate.
    const int D = std::max(1, static_cast<int>(sample_rate_hz_ / kAudioRate));
    const double fs_dec = sample_rate_hz_ / static_cast<double>(D);
    decimateInPlace(sig_i, D);
    decimateInPlace(sig_q, D);

    emit progress(50);

    // Demodulate.
    std::vector<float> audio;
    const double max_dev = channel_bw_hz_ * 0.4;
    switch (mod_) {
        case Mod::NFM:
        case Mod::WFM: audio = demodFm(sig_i, sig_q, fs_dec, max_dev); break;
        case Mod::AM:  audio = demodAm(sig_i, sig_q); break;
        case Mod::USB: audio = demodSsb(sig_i); break;
        case Mod::LSB: {
            // Shift to lower sideband: conjugate then I.
            std::vector<double> neg_q(sig_q.size());
            for (size_t k = 0; k < sig_q.size(); ++k) neg_q[k] = -sig_q[k];
            (void)neg_q;
            audio = demodSsb(sig_i);  // simplified
            break;
        }
    }
    sig_i.clear(); sig_q.clear();

    emit progress(65);

    // Resample to exactly 48000 Hz.
    if (std::abs(fs_dec - kAudioRate) > 1.0) {
        const size_t dst_n = static_cast<size_t>(
            audio.size() * kAudioRate / fs_dec);
        audio = resampleTo(audio, dst_n);
    }

    // Normalize.
    float peak = 0.01f;
    for (float x : audio) peak = std::max(peak, std::abs(x));
    for (float& x : audio) x /= peak;

    emit progress(75);

    // Compute demodulated power spectrum (returned as flat dBFS array).
    int pdb_n_bins = 0, pdb_n_frames = 0;
    const QVector<float> pdb_flat = computeAudioPdb(audio, pdb_n_bins, pdb_n_frames);

    emit progress(88);

    // Downsample for waveform display (1000 points).
    const int   wf_pts = 1000;
    const std::vector<float> wf_std = resampleTo(audio, static_cast<size_t>(wf_pts));
    QVector<float> waveform(wf_std.begin(), wf_std.end());

    // Convert to int16 PCM.
    QByteArray pcm(static_cast<int>(audio.size() * 2), Qt::Uninitialized);
    auto* pcm16 = reinterpret_cast<int16_t*>(pcm.data());
    for (size_t k = 0; k < audio.size(); ++k)
        pcm16[k] = static_cast<int16_t>(
            std::clamp(audio[k] * 32767.f, -32768.f, 32767.f));

    emit progress(100);
    emit done(pcm, pdb_flat, pdb_n_bins, pdb_n_frames, t0, t1, waveform);
}

// ---------------------------------------------------------------------------
// DemodDialog
// ---------------------------------------------------------------------------

DemodDialog::DemodDialog(QString file_path,
                          double file_center_hz, double sample_rate_hz,
                          double channel_center_hz, double channel_bw_hz,
                          double t_start_s, double t_end_s,
                          QWidget* parent)
    : QDialog(parent, Qt::Window),
      file_path_(std::move(file_path)),
      file_center_hz_(file_center_hz), sample_rate_hz_(sample_rate_hz),
      channel_center_hz_(channel_center_hz), channel_bw_hz_(channel_bw_hz),
      t_start_s_(t_start_s), t_end_s_(t_end_s) {
    setWindowTitle(QString("Demodulering – %1 Hz")
                       .arg(channel_center_hz, 0, 'f', 0));
    setAttribute(Qt::WA_DeleteOnClose);

    auto* root = new QVBoxLayout(this);
    root->setSpacing(6);
    root->setContentsMargins(8, 8, 8, 8);

    // Control row
    auto* ctrl = new QHBoxLayout();
    mod_combo_ = new QComboBox(this);
    mod_combo_->addItem("NFM",  QVariant::fromValue(int(DemodWorker::Mod::NFM)));
    mod_combo_->addItem("WFM",  QVariant::fromValue(int(DemodWorker::Mod::WFM)));
    mod_combo_->addItem("AM",   QVariant::fromValue(int(DemodWorker::Mod::AM)));
    mod_combo_->addItem("USB",  QVariant::fromValue(int(DemodWorker::Mod::USB)));
    mod_combo_->addItem("LSB",  QVariant::fromValue(int(DemodWorker::Mod::LSB)));

    floor_spin_ = new QDoubleSpinBox(this);
    floor_spin_->setRange(-200.0, 0.0);
    floor_spin_->setDecimals(1);
    floor_spin_->setSingleStep(1.0);
    floor_spin_->setValue(-200.0);
    floor_spin_->setSuffix(" dBFS");
    floor_spin_->setToolTip("Golv för vattenfallsfärgskalan (0 dBFS = tak)");

    run_btn_  = new QPushButton("▶ Demodulera / Spela", this);
    stop_btn_ = new QPushButton("■ Stopp", this);
    stop_btn_->setEnabled(false);

    vol_slider_ = new QSlider(Qt::Horizontal, this);
    vol_slider_->setRange(0, 100);
    vol_slider_->setValue(80);
    vol_slider_->setMaximumWidth(120);
    vol_slider_->setToolTip("Volym");

    prog_bar_ = new QProgressBar(this);
    prog_bar_->setRange(0, 100);
    prog_bar_->setMaximumWidth(150);
    prog_bar_->setVisible(false);

    status_lbl_ = new QLabel(this);

    ctrl->addWidget(new QLabel("Modulering:", this));
    ctrl->addWidget(mod_combo_);
    ctrl->addWidget(new QLabel("Golv:", this));
    ctrl->addWidget(floor_spin_);
    ctrl->addWidget(run_btn_);
    ctrl->addWidget(stop_btn_);
    ctrl->addWidget(new QLabel("Volym:", this));
    ctrl->addWidget(vol_slider_);
    ctrl->addWidget(prog_bar_);
    ctrl->addWidget(status_lbl_, 1);
    root->addLayout(ctrl);

    // Visualisation area
    auto* vis = new QHBoxLayout();
    wf_widget_  = new DemodWaterfallWidget(this);
    wav_widget_ = new AudioWaveformWidget(this);
    wav_widget_->setFixedWidth(110);
    vis->addWidget(wf_widget_, 1);
    vis->addWidget(wav_widget_);
    root->addLayout(vis, 1);

    // Plugin chain section
    auto* chain_row = new QHBoxLayout();
    demod_combo_   = new QComboBox(this);
    decoder_combo_ = new QComboBox(this);
    postproc_combo_= new QComboBox(this);
    chain_run_btn_ = new QPushButton("Kör kedja", this);

    chain_row->addWidget(new QLabel("Demodulator:", this));
    chain_row->addWidget(demod_combo_);
    chain_row->addWidget(new QLabel("Avkodare:", this));
    chain_row->addWidget(decoder_combo_);
    chain_row->addWidget(new QLabel("Postprocessing:", this));
    chain_row->addWidget(postproc_combo_);
    chain_row->addWidget(chain_run_btn_);
    root->addLayout(chain_row);

    chain_output_ = new QPlainTextEdit(this);
    chain_output_->setReadOnly(true);
    chain_output_->setMaximumHeight(140);
    chain_output_->setPlaceholderText("Demodulerade meddelanden visas här…");
    QFont mono;
    mono.setFamily("Monospace");
    mono.setPointSize(8);
    chain_output_->setFont(mono);
    root->addWidget(chain_output_);

    // Size cap
    const QScreen* screen = QApplication::primaryScreen();
    if (screen) {
        const QRect av = screen->availableGeometry();
        resize(std::min(800, static_cast<int>(av.width()  * 0.92)),
               std::min(640, static_cast<int>(av.height() * 0.92)));
        setMaximumSize(static_cast<int>(av.width()  * 0.92),
                       static_cast<int>(av.height() * 0.92));
    } else {
        resize(800, 640);
    }

    connect(floor_spin_,  &QDoubleSpinBox::valueChanged,   this, [this](double) { buildAudioImage(); });
    connect(mod_combo_,   &QComboBox::currentIndexChanged, this, [this](int)    { onProcess(); });
    connect(run_btn_,     &QPushButton::clicked,            this, &DemodDialog::onProcess);
    connect(chain_run_btn_, &QPushButton::clicked,          this, &DemodDialog::onRunPluginChain);

    // Scan for plugins and populate combos.
    PluginRegistry::instance().scan(PluginRegistry::defaultPluginDir());
    populatePluginCombos();
    connect(stop_btn_,  &QPushButton::clicked, this, &DemodDialog::onStop);
    connect(vol_slider_, &QSlider::valueChanged, this, [this](int v) {
        if (audio_sink_) audio_sink_->setVolume(v / 100.0f);
    });

    // Auto-start on open.
    QMetaObject::invokeMethod(this, &DemodDialog::onProcess, Qt::QueuedConnection);
}

DemodWorker::Mod DemodDialog::currentMod() const {
    return static_cast<DemodWorker::Mod>(mod_combo_->currentData().toInt());
}

void DemodDialog::onProcess() {
    if (is_processing_) return;

    // Stop current playback
    if (audio_sink_ && audio_sink_->state() == QAudio::ActiveState)
        audio_sink_->stop();

    is_processing_ = true;
    run_btn_->setEnabled(false);
    stop_btn_->setEnabled(true);
    prog_bar_->setValue(0);
    prog_bar_->setVisible(true);
    status_lbl_->setText("Behandlar…");

    auto* worker = new DemodWorker(file_path_, file_center_hz_, sample_rate_hz_,
                                    channel_center_hz_, channel_bw_hz_,
                                    t_start_s_, t_end_s_, currentMod());
    worker_thread_ = new QThread();
    worker->moveToThread(worker_thread_);

    connect(worker_thread_, &QThread::started,      worker, &DemodWorker::run);
    connect(worker,         &DemodWorker::progress, this,   &DemodDialog::onProgress);
    connect(worker,         &DemodWorker::done,     this,   &DemodDialog::onDone);
    connect(worker,         &DemodWorker::done,     worker, &QObject::deleteLater);
    connect(worker,         &DemodWorker::done,     worker_thread_, &QThread::quit);
    connect(worker_thread_, &QThread::finished,     worker_thread_, &QObject::deleteLater);

    worker_thread_->start();
}

void DemodDialog::onStop() {
    if (audio_sink_)
        audio_sink_->stop();
    stop_btn_->setEnabled(false);
    status_lbl_->setText("Stoppad.");
}

void DemodDialog::onProgress(int pct) {
    prog_bar_->setValue(pct);
}

void DemodDialog::onDone(QByteArray pcm,
                          QVector<float> pdb_flat, int n_bins, int n_frames,
                          double t_start, double t_end, QVector<float> waveform) {
    is_processing_ = false;
    prog_bar_->setVisible(false);
    run_btn_->setEnabled(true);

    if (pcm.isEmpty()) {
        status_lbl_->setText("Kunde inte demodulera.");
        stop_btn_->setEnabled(false);
        return;
    }

    pdb_flat_    = std::move(pdb_flat);
    pdb_n_bins_  = n_bins;
    pdb_n_frames_= n_frames;
    pdb_t_start_ = t_start;
    pdb_t_end_   = t_end;
    buildAudioImage();

    wav_widget_->setWaveform(waveform, t_start, t_end);

    // Set up audio playback.
    audio_data_ = std::move(pcm);

    QAudioFormat fmt;
    fmt.setSampleRate(static_cast<int>(kAudioRate));
    fmt.setChannelCount(1);
    fmt.setSampleFormat(QAudioFormat::Int16);

    delete audio_sink_;
    delete audio_buffer_;

    const QAudioDevice dev = QMediaDevices::defaultAudioOutput();
    audio_sink_ = new QAudioSink(dev, fmt, this);
    audio_sink_->setVolume(vol_slider_->value() / 100.0f);
    connect(audio_sink_, &QAudioSink::stateChanged,
            this, &DemodDialog::onAudioStateChanged);

    audio_buffer_ = new QBuffer(this);
    audio_buffer_->setData(audio_data_);
    audio_buffer_->open(QIODevice::ReadOnly);
    audio_sink_->start(audio_buffer_);

    const double dur = static_cast<double>(audio_data_.size() / 2) / kAudioRate;
    status_lbl_->setText(QString("Spelar %1 s audio").arg(dur, 0, 'f', 1));
}

void DemodDialog::populatePluginCombos() {
    auto fill = [](QComboBox* combo, MrPluginRole role) {
        combo->clear();
        combo->addItem("Ingen", QVariant::fromValue<quintptr>(0));
        for (const PluginHandle* h : PluginRegistry::instance().byRole(role)) {
            const QString label = h->name +
                (h->description.isEmpty() ? "" : " — " + h->description);
            combo->addItem(label, QVariant::fromValue<quintptr>(
                reinterpret_cast<quintptr>(h)));
        }
    };
    fill(demod_combo_,   MR_PLUGIN_ROLE_DEMODULATOR);
    fill(decoder_combo_, MR_PLUGIN_ROLE_DECODER);
    fill(postproc_combo_,MR_PLUGIN_ROLE_POSTPROCESSING);
}

const PluginHandle* DemodDialog::selectedPlugin(QComboBox* combo) const {
    const auto ptr = combo->currentData().value<quintptr>();
    return reinterpret_cast<const PluginHandle*>(ptr);
}

void DemodDialog::onRunPluginChain() {
    const PluginHandle* demod_p   = selectedPlugin(demod_combo_);
    const PluginHandle* decoder_p = selectedPlugin(decoder_combo_);
    const PluginHandle* postproc_p= selectedPlugin(postproc_combo_);

    if (!demod_p) {
        chain_output_->appendPlainText("[Ingen demodulatorplugin vald]");
        return;
    }

    chain_output_->clear();
    chain_output_->appendPlainText(
        QString("[Kör kedja: %1 → %2 → %3]")
            .arg(demod_p->name)
            .arg(decoder_p  ? decoder_p->name  : "—")
            .arg(postproc_p ? postproc_p->name : "—"));

    // Read IQ data for the selected time range.
    QFile file(file_path_);
    if (!file.open(QIODevice::ReadOnly)) {
        chain_output_->appendPlainText("[Kunde inte öppna filen]");
        return;
    }
    const qint64 total = file.size() / 4;
    const double total_dur = static_cast<double>(total) / sample_rate_hz_;
    const double t0 = std::max(0.0, t_start_s_);
    const double t1 = std::min(total_dur, t_end_s_ > 1.0e8 ? total_dur : t_end_s_);
    if (t1 <= t0) { file.close(); return; }
    file.seek(static_cast<qint64>(t0 * sample_rate_hz_) * 4);
    const QByteArray raw = file.read(
        static_cast<qint64>((t1 - t0) * sample_rate_hz_) * 4);
    file.close();

    PluginChainRunner runner;
    runner.setDemodPlugin(demod_p);
    runner.setDecoderPlugin(decoder_p);
    runner.setPostprocPlugin(postproc_p);

    const auto messages = runner.run(
        reinterpret_cast<const int16_t*>(raw.constData()),
        static_cast<size_t>(raw.size()) / 4U,
        static_cast<uint32_t>(sample_rate_hz_),
        channel_center_hz_);

    if (messages.isEmpty()) {
        chain_output_->appendPlainText("[Inga meddelanden avkodade]");
    } else {
        for (const ChainMessage& msg : messages) {
            const QString stage = msg.stage == 0 ? "DEMOD" :
                                  msg.stage == 1 ? "AVKOD" : "POST";
            chain_output_->appendPlainText(
                QString("[%1] %2: %3 %4")
                    .arg(stage)
                    .arg(msg.signal_type)
                    .arg(msg.payload.left(120))
                    .arg(msg.kv_json.isEmpty() ? "" : msg.kv_json));
        }
    }
}

void DemodDialog::buildAudioImage() {
    if (pdb_flat_.isEmpty() || pdb_n_bins_ <= 0 || pdb_n_frames_ <= 0) return;

    // Same approach as IQ waterfall: fixed floor, ceiling = 0 dBFS, no percentiles.
    const float floor_db = static_cast<float>(floor_spin_->value());
    constexpr float ceiling_db = 0.0f;
    const float span = std::max(ceiling_db - floor_db, 0.1f);

    QImage img(pdb_n_bins_, pdb_n_frames_, QImage::Format_RGB32);
    for (int fr = 0; fr < pdb_n_frames_; ++fr) {
        auto* line = reinterpret_cast<QRgb*>(img.scanLine(fr));
        const int row_off = fr * pdb_n_bins_;
        for (int b = 0; b < pdb_n_bins_; ++b) {
            const float pw = pdb_flat_[row_off + b];
            if (pw < floor_db) {
                line[b] = qRgb(0, 0, 0);
            } else {
                line[b] = WaterfallColor((pw - floor_db) / span);
            }
        }
    }

    const double freq_hi = kAudioRate / 2.0;
    wf_widget_->setData(img, freq_hi, pdb_t_start_, pdb_t_end_);
}

void DemodDialog::onAudioStateChanged(QAudio::State state) {
    if (state == QAudio::IdleState || state == QAudio::StoppedState) {
        stop_btn_->setEnabled(false);
        if (state == QAudio::IdleState)
            status_lbl_->setText("Uppspelning klar.");
    }
}

}  // namespace iq_analyzer
