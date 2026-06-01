#include "iq_analyzer_window.hpp"
#include "dsp_utils.hpp"
#include "waterfall_dialog.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numeric>
#include <vector>

#include <QApplication>
#include <QFile>
#include <QFileDialog>
#include <QMap>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QRegularExpression>
#include <QScreen>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStatusBar>
#include <QVBoxLayout>

namespace iq_analyzer {

namespace {

constexpr double kThresholdDb   = 10.0;
constexpr int    kMinSpanFrames = 2;
constexpr int    kGapFrames     = 10;

double UnitMultiplier(const QString& unit) {
    if (unit == "kHz" || unit == "kSps") return 1.0e3;
    if (unit == "MHz" || unit == "MSps") return 1.0e6;
    return 1.0;
}

}  // namespace

// ---------------------------------------------------------------------------
// AnalysisWorker
// ---------------------------------------------------------------------------

AnalysisWorker::AnalysisWorker(QString path, double center_hz,
                               double sample_rate_hz, double channel_bw_hz,
                               int fft_size, QObject* parent)
    : QObject(parent),
      path_(std::move(path)),
      center_hz_(center_hz),
      sample_rate_hz_(sample_rate_hz),
      channel_bw_hz_(channel_bw_hz),
      fft_size_(fft_size) {}

void AnalysisWorker::run() {
    // --- Read file --------------------------------------------------------
    QFile file(path_);
    if (!file.open(QIODevice::ReadOnly)) {
        emit finished(DetectedSignalList{}, 0.0, "Kunde inte öppna filen: " + path_);
        return;
    }
    const qint64 sample_pairs = file.size() / 4;  // each IQ pair = 2 × int16 = 4 bytes
    if (sample_pairs < fft_size_) {
        emit finished(DetectedSignalList{}, 0.0, "Filen är för liten för analys.");
        return;
    }
    const QByteArray raw = file.readAll();
    file.close();

    const size_t num_pairs = static_cast<size_t>(raw.size() / 4);
    const auto*  raw_i16  = reinterpret_cast<const int16_t*>(raw.constData());

    // --- Channel grid ----------------------------------------------------
    // Channel n (signed) has centre at: cf + n * bw
    // It spans: [cf + (n - 0.5)*bw, cf + (n + 0.5)*bw]
    // FFT bin k (0-based) → fftshifted index s = (k + N/2) % N
    // s maps to frequency offset: (s - N/2) * bin_hz
    // → channel index: n = round(freq_offset / bw)
    const size_t N          = static_cast<size_t>(fft_size_);
    const size_t half       = N / 2U;
    const double bin_hz     = sample_rate_hz_ / static_cast<double>(N);
    const double bins_per_ch = channel_bw_hz_ / bin_hz;
    const size_t num_frames = num_pairs / N;

    // Signed channel range that covers the full bandwidth.
    const int max_ch_n = static_cast<int>(std::ceil(static_cast<double>(half) / bins_per_ch));
    const int ch_offset = max_ch_n;               // array_idx = ch_n + ch_offset
    const int total_ch  = 2 * max_ch_n + 1;

    // chan_power[frame][array_idx] – summed linear power per channel per frame.
    std::vector<std::vector<double>> chan_power(
        num_frames, std::vector<double>(static_cast<size_t>(total_ch), 0.0));

    std::vector<std::complex<double>> fft_buf(N);

    for (size_t frame = 0; frame < num_frames; ++frame) {
        const size_t base = frame * N;
        for (size_t i = 0; i < N; ++i) {
            const double iv = static_cast<double>(raw_i16[(base + i) * 2])     * kNormScale;
            const double qv = static_cast<double>(raw_i16[(base + i) * 2 + 1]) * kNormScale;
            fft_buf[i] = std::complex<double>(iv, qv) * HannWindow(i, N);
        }
        FftRadix2InPlace(fft_buf);

        for (size_t k = 0; k < N; ++k) {
            const size_t s     = (k + half) % N;   // fftshift
            const double mag2  = std::norm(fft_buf[k]) / static_cast<double>(N * N);
            const double foff  = (static_cast<double>(s) - static_cast<double>(half)) * bin_hz;
            const int    ch_n  = static_cast<int>(std::round(foff / channel_bw_hz_));
            const int    idx   = ch_n + ch_offset;
            if (idx >= 0 && idx < total_ch) {
                chan_power[frame][static_cast<size_t>(idx)] += mag2;
            }
        }

        if ((frame % 200) == 0) {
            emit progress(static_cast<int>((frame * 80) / num_frames));
        }
    }

    emit progress(82);

    // --- Noise floor per channel (25th percentile across frames) ---------
    const size_t nc = static_cast<size_t>(total_ch);
    std::vector<double> nf_linear(nc, 0.0);
    {
        std::vector<double> col(num_frames);
        for (size_t ai = 0; ai < nc; ++ai) {
            for (size_t f = 0; f < num_frames; ++f) col[f] = chan_power[f][ai];
            std::sort(col.begin(), col.end());
            nf_linear[ai] = std::max(col[col.size() / 4U], 1.0e-30);
        }
    }

    // Median noise floor across channels → dBFS per bin.
    double noise_floor_db = 0.0;
    {
        std::vector<double> nf_db_vec(nc);
        for (size_t ai = 0; ai < nc; ++ai) {
            nf_db_vec[ai] = 10.0 * std::log10(std::max(nf_linear[ai] / bins_per_ch, 1.0e-30));
        }
        std::sort(nf_db_vec.begin(), nf_db_vec.end());
        noise_floor_db = nf_db_vec[nf_db_vec.size() / 2U];
    }

    emit progress(90);

    // --- Detect and merge signals per channel ----------------------------
    const double frame_dur = static_cast<double>(N) / sample_rate_hz_;
    DetectedSignalList detections;

    struct Segment {
        size_t start;         // first active frame
        size_t end;           // one past last active frame (span includes any bridged gap)
        double power_sum;     // sum over active (above-threshold) frames only
        int    active_count;  // number of active frames
    };

    for (int ch_n = -max_ch_n; ch_n <= max_ch_n; ++ch_n) {
        const size_t ai = static_cast<size_t>(ch_n + ch_offset);
        const double nf = nf_linear[ai];
        const double threshold = nf * std::pow(10.0, kThresholdDb / 10.0);
        const double ch_freq_hz = center_hz_ + static_cast<double>(ch_n) * channel_bw_hz_;

        // Collect raw active segments.
        std::vector<Segment> raw_segs;
        {
            bool   active = false;
            size_t seg_start = 0;
            double psum = 0.0;
            int    cnt  = 0;
            for (size_t f = 0; f < num_frames; ++f) {
                const double p = chan_power[f][ai];
                if (p >= threshold) {
                    if (!active) { seg_start = f; active = true; psum = 0.0; cnt = 0; }
                    psum += p;
                    ++cnt;
                } else if (active) {
                    raw_segs.push_back({seg_start, f, psum, cnt});
                    active = false;
                }
            }
            if (active) raw_segs.push_back({seg_start, num_frames, psum, cnt});
        }

        // Merge segments separated by ≤ kGapFrames.
        std::vector<Segment> merged;
        for (const auto& seg : raw_segs) {
            if (!merged.empty() &&
                seg.start <= merged.back().end + static_cast<size_t>(kGapFrames)) {
                merged.back().end          = seg.end;
                merged.back().power_sum   += seg.power_sum;
                merged.back().active_count += seg.active_count;
            } else {
                merged.push_back(seg);
            }
        }

        const double ch_nf_dbfs =
            10.0 * std::log10(std::max(nf / bins_per_ch, 1.0e-30));

        for (const auto& seg : merged) {
            const int span = static_cast<int>(seg.end - seg.start);
            if (span < kMinSpanFrames) continue;
            const double mean_power    = seg.power_sum / static_cast<double>(seg.active_count);
            const double strength_dbfs = 10.0 * std::log10(std::max(mean_power / bins_per_ch, 1.0e-30));
            DetectedSignal s;
            s.center_freq_hz   = ch_freq_hz;
            s.noise_floor_dbfs = ch_nf_dbfs;
            s.time_offset_s    = static_cast<double>(seg.start) * frame_dur;
            s.strength_dbfs    = strength_dbfs;
            s.duration_s       = static_cast<double>(span) * frame_dur;
            detections.append(s);
        }
    }

    emit progress(100);

    std::sort(detections.begin(), detections.end(),
              [](const DetectedSignal& a, const DetectedSignal& b) {
                  return a.time_offset_s < b.time_offset_s;
              });

    emit finished(detections, noise_floor_db, {});
}

// ---------------------------------------------------------------------------
// IqAnalyzerWindow
// ---------------------------------------------------------------------------

IqAnalyzerWindow::IqAnalyzerWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("IQ-analysator");

    auto* central = new QWidget(this);
    setCentralWidget(central);
    auto* root = new QVBoxLayout(central);
    root->setSpacing(6);
    root->setContentsMargins(8, 8, 8, 8);

    // --- File row --------------------------------------------------------
    auto* file_group  = new QGroupBox("Fil", central);
    auto* file_layout = new QHBoxLayout(file_group);
    open_btn_      = new QPushButton("Öppna…", file_group);
    file_path_edit_ = new QLineEdit(file_group);
    file_path_edit_->setReadOnly(true);
    file_path_edit_->setPlaceholderText("Ingen fil vald");
    file_layout->addWidget(open_btn_);
    file_layout->addWidget(file_path_edit_, 1);
    root->addWidget(file_group);

    // --- Parameters ------------------------------------------------------
    auto* param_group  = new QGroupBox("Parametrar", central);
    auto* param_layout = new QFormLayout(param_group);
    param_layout->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);

    auto make_freq_row = [&](QDoubleSpinBox*& spin, QComboBox*& combo,
                              QWidget* parent, const QString& unit_default) {
        auto* row = new QWidget(parent);
        auto* hl  = new QHBoxLayout(row);
        hl->setContentsMargins(0, 0, 0, 0);
        spin = new QDoubleSpinBox(row);
        spin->setRange(0.0, 1.0e9);
        spin->setDecimals(6);
        spin->setSingleStep(1.0);
        spin->setMinimumWidth(120);
        combo = new QComboBox(row);
        if (unit_default.contains("Sps")) {
            combo->addItem("Sps");
            combo->addItem("kSps");
            combo->addItem("MSps");
        } else {
            combo->addItem("Hz");
            combo->addItem("kHz");
            combo->addItem("MHz");
        }
        combo->setCurrentText(unit_default);
        hl->addWidget(spin);
        hl->addWidget(combo);
        hl->addStretch(1);
        return row;
    };

    auto* cf_row = make_freq_row(cf_spin_, cf_unit_combo_, param_group, "MHz");
    auto* sr_row = make_freq_row(sr_spin_, sr_unit_combo_, param_group, "MSps");
    auto* bw_row = make_freq_row(bw_spin_, bw_unit_combo_, param_group, "kHz");

    fft_size_combo_ = new QComboBox(param_group);
    for (int s : {256, 512, 1024, 2048, 4096, 8192, 16384})
        fft_size_combo_->addItem(QString::number(s), s);
    fft_size_combo_->setCurrentIndex(fft_size_combo_->findData(kDefaultFftSize));

    noise_floor_label_ = new QLabel(param_group);
    noise_floor_label_->setText("–");

    param_layout->addRow("Centerfrekvens:", cf_row);
    param_layout->addRow("Samplingsfrekvens:", sr_row);
    param_layout->addRow("Kanalbredd:", bw_row);
    param_layout->addRow("FFT-storlek:", fft_size_combo_);
    param_layout->addRow("Brusgolv:", noise_floor_label_);

    // --- Top-5 panels (next to parameters) --------------------------------
    auto* top_active_group  = new QGroupBox("Mest aktiva", central);
    auto* top_active_layout = new QFormLayout(top_active_group);
    for (int i = 0; i < 5; ++i) {
        top_freq_labels_[i] = new QLabel("–", top_active_group);
        top_active_layout->addRow(QString("%1.").arg(i + 1), top_freq_labels_[i]);
    }

    auto* top_str_group  = new QGroupBox("Starkast", central);
    auto* top_str_layout = new QFormLayout(top_str_group);
    for (int i = 0; i < 5; ++i) {
        top_str_labels_[i] = new QLabel("–", top_str_group);
        top_str_layout->addRow(QString("%1.").arg(i + 1), top_str_labels_[i]);
    }

    auto* params_row = new QHBoxLayout();
    params_row->addWidget(param_group,      3);
    params_row->addWidget(top_active_group, 2);
    params_row->addWidget(top_str_group,    2);
    root->addLayout(params_row);

    // --- Analyze button + progress ---------------------------------------
    auto* action_row = new QHBoxLayout();
    analyze_btn_   = new QPushButton("Analysera", central);
    analyze_btn_->setEnabled(false);
    waterfall_btn_ = new QPushButton("Vattenfall", central);
    waterfall_btn_->setEnabled(false);
    waterfall_btn_->setToolTip("Öppna vattenfallsdialog för hela inspelningen");
    progress_bar_ = new QProgressBar(central);
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);
    progress_bar_->setVisible(false);
    status_label_ = new QLabel(central);
    action_row->addWidget(analyze_btn_);
    action_row->addWidget(waterfall_btn_);
    action_row->addWidget(progress_bar_, 1);
    action_row->addWidget(status_label_, 1);
    root->addLayout(action_row);

    // --- Strength filter -------------------------------------------------
    auto* filter_row = new QHBoxLayout();
    strength_filter_spin_ = new QDoubleSpinBox(central);
    strength_filter_spin_->setRange(-200.0, 0.0);
    strength_filter_spin_->setDecimals(1);
    strength_filter_spin_->setSingleStep(1.0);
    strength_filter_spin_->setValue(-200.0);
    strength_filter_spin_->setSuffix(" dBFS");
    strength_filter_spin_->setToolTip("Dölj signaler vars styrka är under detta värde");

    length_filter_spin_ = new QDoubleSpinBox(central);
    length_filter_spin_->setRange(0.0, 3600.0);
    length_filter_spin_->setDecimals(3);
    length_filter_spin_->setSingleStep(0.1);
    length_filter_spin_->setValue(0.0);
    length_filter_spin_->setSuffix(" s");
    length_filter_spin_->setSpecialValueText("Alla längder");
    length_filter_spin_->setToolTip("Dölj signaler kortare än detta värde");

    filter_row->addWidget(new QLabel("Dölj signaler under:", central));
    filter_row->addWidget(strength_filter_spin_);
    filter_row->addSpacing(16);
    filter_row->addWidget(new QLabel("Kortare än:", central));
    filter_row->addWidget(length_filter_spin_);
    filter_row->addStretch(1);
    root->addLayout(filter_row);

    // --- Results table ---------------------------------------------------
    results_table_ = new QTableWidget(0, 5, central);
    results_table_->setHorizontalHeaderLabels(
        {"Centerfrekvens", "Brusgolv (dBFS)", "Tidsoffset (s)", "Styrka (dBFS)", "Längd (s)"});
    results_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    results_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    results_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    results_table_->setAlternatingRowColors(true);
    results_table_->verticalHeader()->setVisible(false);
    results_table_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    root->addWidget(results_table_, 1);

    // --- Window size: cap to available screen geometry ------------------
    const QScreen* screen = QApplication::primaryScreen();
    if (screen) {
        const QRect avail = screen->availableGeometry();
        const int   max_w = static_cast<int>(avail.width()  * 0.92);
        const int   max_h = static_cast<int>(avail.height() * 0.92);
        resize(std::min(900, max_w), std::min(700, max_h));
        setMaximumSize(max_w, max_h);
    } else {
        resize(900, 700);
    }

    // --- Connections -----------------------------------------------------
    connect(open_btn_,       &QPushButton::clicked, this, &IqAnalyzerWindow::onOpenFile);
    connect(analyze_btn_,   &QPushButton::clicked, this, &IqAnalyzerWindow::onAnalyze);
    connect(waterfall_btn_, &QPushButton::clicked, this, &IqAnalyzerWindow::onOpenWaterfall);
    connect(results_table_, &QTableWidget::cellDoubleClicked,
            this, &IqAnalyzerWindow::onRowDoubleClicked);
    connect(strength_filter_spin_, &QDoubleSpinBox::valueChanged,
            this, [this](double) { populateTable(); });
    connect(length_filter_spin_,   &QDoubleSpinBox::valueChanged,
            this, [this](double) { populateTable(); });

    qRegisterMetaType<DetectedSignalList>();
}

void IqAnalyzerWindow::applyFilenameParams(const QString& path) {
    // Pattern: raw_<date>T<time>Z_<center_hz>_<sample_rate_hz>_<N>.iq16
    const QRegularExpression re(
        R"(raw_\d{8}T\d{6}Z_(\d+)_(\d+)_\d+\.iq16$)");
    const auto m = re.match(path);
    if (!m.hasMatch()) return;

    bool ok1 = false, ok2 = false;
    const double cf = m.captured(1).toDouble(&ok1);
    const double sr = m.captured(2).toDouble(&ok2);
    if (!ok1 || !ok2) return;

    // Pick appropriate unit.
    auto set_with_unit = [](QDoubleSpinBox* spin, QComboBox* combo,
                             double hz, bool is_sps) {
        const QString munit = is_sps ? "MSps" : "MHz";
        const QString kunit = is_sps ? "kSps" : "kHz";
        const QString unit  = is_sps ? "Sps"  : "Hz";
        if (hz >= 1.0e6) {
            combo->setCurrentText(munit);
            spin->setValue(hz / 1.0e6);
        } else if (hz >= 1.0e3) {
            combo->setCurrentText(kunit);
            spin->setValue(hz / 1.0e3);
        } else {
            combo->setCurrentText(unit);
            spin->setValue(hz);
        }
    };

    set_with_unit(cf_spin_, cf_unit_combo_, cf, false);
    set_with_unit(sr_spin_, sr_unit_combo_, sr, true);
}

double IqAnalyzerWindow::centerFreqHz() const {
    return cf_spin_->value() * UnitMultiplier(cf_unit_combo_->currentText());
}

double IqAnalyzerWindow::sampleRateHz() const {
    return sr_spin_->value() * UnitMultiplier(sr_unit_combo_->currentText());
}

double IqAnalyzerWindow::channelBwHz() const {
    return bw_spin_->value() * UnitMultiplier(bw_unit_combo_->currentText());
}

void IqAnalyzerWindow::setControlsEnabled(bool enabled) {
    open_btn_->setEnabled(enabled);
    const bool has_file = !file_path_edit_->text().isEmpty();
    analyze_btn_->setEnabled(enabled && has_file);
    waterfall_btn_->setEnabled(enabled && has_file);
    cf_spin_->setEnabled(enabled);
    cf_unit_combo_->setEnabled(enabled);
    sr_spin_->setEnabled(enabled);
    sr_unit_combo_->setEnabled(enabled);
    bw_spin_->setEnabled(enabled);
    bw_unit_combo_->setEnabled(enabled);
}

void IqAnalyzerWindow::onOpenFile() {
    const QString path = QFileDialog::getOpenFileName(
        this, "Öppna IQ-fil", {}, "IQ-filer (*.iq16);;Alla filer (*)");
    if (path.isEmpty()) return;
    file_path_edit_->setText(path);
    applyFilenameParams(path);
    analyze_btn_->setEnabled(true);
    waterfall_btn_->setEnabled(true);
    results_table_->setRowCount(0);
    status_label_->clear();
}

void IqAnalyzerWindow::onAnalyze() {
    const QString path = file_path_edit_->text();
    if (path.isEmpty()) return;
    const double cf = centerFreqHz();
    const double sr = sampleRateHz();
    const double bw = channelBwHz();
    if (sr <= 0.0) { QMessageBox::warning(this, "Fel", "Samplingsfrekvens måste vara > 0."); return; }
    if (bw <= 0.0) { QMessageBox::warning(this, "Fel", "Kanalbredd måste vara > 0."); return; }

    current_cf_hz_ = cf;
    current_sr_hz_ = sr;
    current_bw_hz_ = bw;

    setControlsEnabled(false);
    results_table_->setRowCount(0);
    for (auto* lbl : top_freq_labels_) lbl->setText("–");
    for (auto* lbl : top_str_labels_)  lbl->setText("–");
    status_label_->setText("Analyserar…");
    progress_bar_->setValue(0);
    progress_bar_->setVisible(true);

    const int fft_size = fft_size_combo_->currentData().toInt();
    auto* worker = new AnalysisWorker(path, cf, sr, bw, fft_size);
    worker_thread_ = new QThread(this);
    worker->moveToThread(worker_thread_);

    connect(worker_thread_, &QThread::started,  worker, &AnalysisWorker::run);
    connect(worker, &AnalysisWorker::progress,  this,   &IqAnalyzerWindow::onProgress);
    connect(worker, &AnalysisWorker::finished, this, &IqAnalyzerWindow::onFinished);
    connect(worker, &AnalysisWorker::finished,  worker, &QObject::deleteLater);
    connect(worker, &AnalysisWorker::finished,  worker_thread_, &QThread::quit);
    connect(worker_thread_, &QThread::finished, worker_thread_, &QObject::deleteLater);

    worker_thread_->start();
}

void IqAnalyzerWindow::onProgress(int percent) {
    progress_bar_->setValue(percent);
}

void IqAnalyzerWindow::onFinished(DetectedSignalList detections, double noise_floor_db,
                                   QString error) {
    progress_bar_->setVisible(false);
    setControlsEnabled(true);

    if (!error.isEmpty()) {
        status_label_->setText("Fel: " + error);
        QMessageBox::critical(this, "Analysfel", error);
        return;
    }

    current_detections_ = detections;

    noise_floor_label_->setText(QString("%1 dBFS").arg(noise_floor_db, 0, 'f', 1));

    // --- Top-5 most active frequencies (by total signal duration) ----------
    {
        QMap<double, double> freq_total_duration;
        for (const auto& s : detections)
            freq_total_duration[s.center_freq_hz] += s.duration_s;

        QList<QPair<double, double>> ranked;
        ranked.reserve(freq_total_duration.size());
        for (auto it = freq_total_duration.cbegin(); it != freq_total_duration.cend(); ++it)
            ranked.append({it.key(), it.value()});
        std::sort(ranked.begin(), ranked.end(),
                  [](const QPair<double,double>& a, const QPair<double,double>& b) {
                      return a.second > b.second;
                  });

        auto fmt_freq = [](double hz) -> QString {
            if (std::abs(hz) >= 1.0e6) return QString::number(hz / 1.0e6, 'f', 4) + " MHz";
            if (std::abs(hz) >= 1.0e3) return QString::number(hz / 1.0e3, 'f', 3) + " kHz";
            return QString::number(hz, 'f', 1) + " Hz";
        };
        for (int i = 0; i < 5; ++i) {
            if (i < ranked.size())
                top_freq_labels_[i]->setText(
                    QString("%1  (%2 s)")
                        .arg(fmt_freq(ranked[i].first))
                        .arg(ranked[i].second, 0, 'f', 1));
            else
                top_freq_labels_[i]->setText("–");
        }
    }

    // --- Top-5 strongest signals (by peak strength_dbfs) -------------------
    {
        auto fmt_freq = [](double hz) -> QString {
            if (std::abs(hz) >= 1.0e6) return QString::number(hz / 1.0e6, 'f', 4) + " MHz";
            if (std::abs(hz) >= 1.0e3) return QString::number(hz / 1.0e3, 'f', 3) + " kHz";
            return QString::number(hz, 'f', 1) + " Hz";
        };

        // Find peak strength per frequency (max over all detections on that channel).
        QMap<double, double> freq_peak_strength;
        for (const auto& s : detections) {
            auto it = freq_peak_strength.find(s.center_freq_hz);
            if (it == freq_peak_strength.end())
                freq_peak_strength.insert(s.center_freq_hz, s.strength_dbfs);
            else if (s.strength_dbfs > it.value())
                it.value() = s.strength_dbfs;
        }

        QList<QPair<double, double>> ranked;
        ranked.reserve(freq_peak_strength.size());
        for (auto it = freq_peak_strength.cbegin(); it != freq_peak_strength.cend(); ++it)
            ranked.append({it.key(), it.value()});
        std::sort(ranked.begin(), ranked.end(),
                  [](const QPair<double,double>& a, const QPair<double,double>& b) {
                      return a.second > b.second;
                  });

        for (int i = 0; i < 5; ++i) {
            if (i < ranked.size())
                top_str_labels_[i]->setText(
                    QString("%1  (%2 dBFS)")
                        .arg(fmt_freq(ranked[i].first))
                        .arg(ranked[i].second, 0, 'f', 1));
            else
                top_str_labels_[i]->setText("–");
        }
    }

    // Default filter threshold = noise floor of this analysis run.
    strength_filter_spin_->setValue(noise_floor_db);

    // populateTable() is triggered automatically by the valueChanged signal above.
    // If the value didn't change, trigger it manually.
    populateTable();
}

void IqAnalyzerWindow::populateTable() {
    const double strength_threshold = strength_filter_spin_->value();
    const double length_threshold   = length_filter_spin_->value();

    current_visible_detections_.clear();
    for (const auto& s : current_detections_) {
        if (s.strength_dbfs >= strength_threshold && s.duration_s >= length_threshold)
            current_visible_detections_.append(s);
    }

    auto fmt_freq = [](double hz) -> QString {
        if (std::abs(hz) >= 1.0e6)
            return QString::number(hz / 1.0e6, 'f', 4) + " MHz";
        if (std::abs(hz) >= 1.0e3)
            return QString::number(hz / 1.0e3, 'f', 3) + " kHz";
        return QString::number(hz, 'f', 1) + " Hz";
    };

    results_table_->setRowCount(0);
    results_table_->setRowCount(static_cast<int>(current_visible_detections_.size()));

    for (int row = 0; row < current_visible_detections_.size(); ++row) {
        const auto& s = current_visible_detections_[row];
        results_table_->setItem(row, 0, new QTableWidgetItem(fmt_freq(s.center_freq_hz)));
        results_table_->setItem(row, 1, new QTableWidgetItem(
            QString::number(s.noise_floor_dbfs, 'f', 1) + " dBFS"));
        results_table_->setItem(row, 2, new QTableWidgetItem(
            QString::number(s.time_offset_s, 'f', 3)));
        results_table_->setItem(row, 3, new QTableWidgetItem(
            QString::number(s.strength_dbfs, 'f', 1) + " dBFS"));
        results_table_->setItem(row, 4, new QTableWidgetItem(
            QString::number(s.duration_s, 'f', 3) + " s"));
        for (int col = 1; col <= 4; ++col) {
            if (auto* item = results_table_->item(row, col))
                item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        }
    }

    const int total   = current_detections_.size();
    const int visible = current_visible_detections_.size();
    if (total == 0) {
        status_label_->setText("Inga signaler hittades.");
    } else if (visible == total) {
        status_label_->setText(
            QString("%1 signal(er) hittade. Dubbelklicka för vattenfall.").arg(total));
    } else {
        status_label_->setText(
            QString("%1 av %2 signal(er) visas (filter). Dubbelklicka för vattenfall.")
                .arg(visible).arg(total));
    }
}

void IqAnalyzerWindow::onRowDoubleClicked(int row, int /*col*/) {
    if (row < 0 || row >= current_visible_detections_.size()) return;
    const DetectedSignal& sig = current_visible_detections_[row];
    auto* dlg = new WaterfallDialog(file_path_edit_->text(), sig,
                                     current_cf_hz_, current_sr_hz_, current_bw_hz_,
                                     this);
    dlg->show();
}

void IqAnalyzerWindow::onOpenWaterfall() {
    const QString path = file_path_edit_->text();
    if (path.isEmpty()) return;

    // Synthetic signal spanning the full file (duration_s <= 0 → dialog sets end = -1).
    DetectedSignal full_file{};
    full_file.center_freq_hz   = centerFreqHz();
    full_file.noise_floor_dbfs = noise_floor_label_->text().isEmpty()
        ? -120.0
        : noise_floor_label_->text().replace(" dBFS", "").toDouble();
    full_file.time_offset_s    = 0.0;
    full_file.strength_dbfs    = 0.0;
    full_file.duration_s       = -1.0;  // sentinel: whole file

    auto* dlg = new WaterfallDialog(path, full_file,
                                     centerFreqHz(), sampleRateHz(), channelBwHz(),
                                     this);
    dlg->show();
}

}  // namespace iq_analyzer
