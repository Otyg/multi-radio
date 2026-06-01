#pragma once

#include <QDialog>
#include <QDoubleSpinBox>
#include <QImage>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QThread>
#include <QVector>
#include <QWidget>

#include "iq_analyzer_window.hpp"

namespace iq_analyzer {

// Displays a time×frequency waterfall image with an oscilloscope-style
// waveform strip (I-component vs time) on the right.
class WaterfallWidget : public QWidget {
    Q_OBJECT
public:
    explicit WaterfallWidget(QWidget* parent = nullptr);

    void setWaterfall(const QImage& image,
                      double freq_lo_hz, double freq_hi_hz,
                      double t_start_s,  double t_end_s,
                      double center_hz,  double bw_hz,
                      double sig_t0_s,   double sig_t1_s);

    // One I-sample per FFT frame, range [-1, 1]. Drawn zero-centred in the strip.
    void setWaveform(const QVector<float>& waveform_i);

    QSize sizeHint() const override        { return {780, 450}; }
    QSize minimumSizeHint() const override { return {340, 180}; }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    static constexpr int kLM  = 60;
    static constexpr int kBM  = 32;
    static constexpr int kTM  = 6;
    static constexpr int kRM  = 8;
    static constexpr int kWFW = 80;  // waveform strip width (px)
    static constexpr int kWFG = 4;   // gap between waterfall and waveform strip

    QImage img_;
    double freq_lo_hz_ = 0.0, freq_hi_hz_ = 1.0;
    double t_start_s_  = 0.0, t_end_s_    = 1.0;
    double center_hz_  = 0.0, bw_hz_      = 1.0;
    double sig_t0_s_   = 0.0, sig_t1_s_   = 1.0;
    bool   has_data_   = false;

    QVector<float> waveform_i_;  // I-component, one per FFT frame
};

// Off-thread FFT worker. Emits raw per-bin power and raw I-samples.
class WaterfallWorker : public QObject {
    Q_OBJECT
public:
    explicit WaterfallWorker(QString path,
                              double center_hz, double sample_rate_hz,
                              double channel_bw_hz,
                              double freq_lo_hz, double freq_hi_hz,
                              double t_start_s,  double t_end_s,
                              int    fft_size,
                              QObject* parent = nullptr);
public slots:
    void run();

signals:
    void progress(int pct);
    // power_flat: row-major [frame * n_bins + bin], dBFS per bin.
    // waveform_i: I-component, one normalised sample per FFT frame [-1, 1].
    void dataReady(QVector<float> power_flat, int n_bins, int n_frames,
                   QVector<float> waveform_i,
                   double freq_lo_hz, double freq_hi_hz,
                   double t_start_s,  double t_end_s);

private:
    QString path_;
    double center_hz_, sample_rate_hz_, channel_bw_hz_;
    double freq_lo_hz_, freq_hi_hz_, t_start_s_, t_end_s_;
    int    fft_size_;
};

// Dialog showing a waterfall with neighbouring channels and floor/ceiling controls.
class WaterfallDialog : public QDialog {
    Q_OBJECT
public:
    explicit WaterfallDialog(QString file_path, DetectedSignal signal,
                              double center_hz, double sample_rate_hz,
                              double channel_bw_hz, QWidget* parent = nullptr);

protected:
    bool eventFilter(QObject* obj, QEvent* ev) override;

private slots:
    void onLoad();
    void onProgress(int pct);
    void onDataReady(QVector<float> power_flat, int n_bins, int n_frames,
                     QVector<float> waveform_i,
                     double freq_lo_hz, double freq_hi_hz,
                     double t_start_s,  double t_end_s);

private:
    void buildAndShowImage();
    double effectiveCenterHz()   const;
    double effectiveChannelBwHz() const;

    QString        file_path_;
    DetectedSignal signal_;
    double         sample_rate_hz_;
    // center_hz_ / channel_bw_hz_ store the constructor values for the window title;
    // effectiveCenterHz() / effectiveChannelBwHz() read from the spinboxes at runtime.
    double         center_hz_, channel_bw_hz_;
    bool           is_loading_ = false;

    QVector<float> power_flat_;
    QVector<float> waveform_i_;
    int            power_n_bins_    = 0;
    int            power_n_frames_  = 0;
    double         power_freq_lo_   = 0.0, power_freq_hi_   = 1.0;
    double         power_t_start_   = 0.0, power_t_end_     = 1.0;
    // center / bw committed when the last worker was started.
    double         pending_center_hz_ = 0.0, pending_bw_hz_  = 1.0;
    // center / bw that produced the currently displayed power data.
    double         loaded_center_hz_  = 0.0, loaded_bw_hz_   = 1.0;

    QDoubleSpinBox* start_spin_     = nullptr;
    QDoubleSpinBox* end_spin_       = nullptr;
    QSpinBox*       neighbors_spin_ = nullptr;
    QDoubleSpinBox* cf_spin_        = nullptr;
    QComboBox*      cf_unit_combo_  = nullptr;
    QDoubleSpinBox* bw_spin_        = nullptr;
    QComboBox*      bw_unit_combo_  = nullptr;
    QComboBox*      fft_size_combo_ = nullptr;
    QDoubleSpinBox* floor_spin_     = nullptr;
    QPushButton*    load_btn_       = nullptr;
    QProgressBar*   prog_bar_       = nullptr;
    QLabel*         status_lbl_     = nullptr;
    WaterfallWidget* wf_widget_     = nullptr;
};

}  // namespace iq_analyzer

Q_DECLARE_METATYPE(QVector<float>)
