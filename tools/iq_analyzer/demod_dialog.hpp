#pragma once

#include <QAudioSink>
#include <QBuffer>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QImage>
#include <QLabel>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QThread>
#include <QVector>
#include <QWidget>

#include "plugin_chain.hpp"

namespace iq_analyzer {

// Spectrum waterfall of the demodulated audio signal (time × audio frequency).
class DemodWaterfallWidget : public QWidget {
    Q_OBJECT
public:
    explicit DemodWaterfallWidget(QWidget* parent = nullptr);
    void setData(const QImage& img, double freq_hi_hz,
                 double t_start_s, double t_end_s);
    QSize sizeHint() const override        { return {500, 350}; }
    QSize minimumSizeHint() const override { return {200, 100}; }
protected:
    void paintEvent(QPaintEvent*) override;
private:
    static constexpr int kLM = 50, kBM = 28, kTM = 5, kRM = 5;
    QImage img_;
    double freq_hi_hz_ = 8000;
    double t_start_s_  = 0.0, t_end_s_ = 1.0;
    bool   has_data_   = false;
};

// Zero-centred waveform of the demodulated audio.
class AudioWaveformWidget : public QWidget {
    Q_OBJECT
public:
    explicit AudioWaveformWidget(QWidget* parent = nullptr);
    void setWaveform(const QVector<float>& samples,
                     double t_start_s, double t_end_s);
    QSize sizeHint() const override        { return {100, 350}; }
    QSize minimumSizeHint() const override { return {60,  100}; }
protected:
    void paintEvent(QPaintEvent*) override;
private:
    static constexpr int kTM = 5, kBM = 28, kLM = 4, kRM = 4;
    QVector<float> samples_;
    double t_start_s_ = 0.0, t_end_s_ = 1.0;
};

// Off-thread worker: IQ file → demodulated PCM + visualisation data.
class DemodWorker : public QObject {
    Q_OBJECT
public:
    enum class Mod { NFM, WFM, AM, USB, LSB };

    explicit DemodWorker(QString path,
                         double file_center_hz, double sample_rate_hz,
                         double channel_center_hz, double channel_bw_hz,
                         double t_start_s, double t_end_s,
                         Mod mod, QObject* parent = nullptr);
public slots:
    void run();

signals:
    void progress(int pct);
    // pdb_flat: row-major [frame * n_bins + bin], dBFS values.
    void done(QByteArray pcm_s16le,
              QVector<float> pdb_flat, int n_bins, int n_frames,
              double t_start_s, double t_end_s,
              QVector<float> waveform);

private:
    QString path_;
    double  file_center_hz_, sample_rate_hz_;
    double  channel_center_hz_, channel_bw_hz_;
    double  t_start_s_, t_end_s_;
    Mod     mod_;
};

// Demodulation and playback dialog.
class DemodDialog : public QDialog {
    Q_OBJECT
public:
    explicit DemodDialog(QString file_path,
                         double file_center_hz, double sample_rate_hz,
                         double channel_center_hz, double channel_bw_hz,
                         double t_start_s, double t_end_s,
                         QWidget* parent = nullptr);

private slots:
    void onProcess();
    void onStop();
    void onProgress(int pct);
    void onDone(QByteArray pcm,
                QVector<float> pdb_flat, int n_bins, int n_frames,
                double t_start, double t_end, QVector<float> waveform);
    void buildAudioImage();
    void onAudioStateChanged(QAudio::State state);
    void onRunPluginChain();

private:
    DemodWorker::Mod currentMod() const;
    void populatePluginCombos();
    const PluginHandle* selectedPlugin(QComboBox* combo) const;

    QString file_path_;
    double  file_center_hz_, sample_rate_hz_;
    double  channel_center_hz_, channel_bw_hz_;
    double  t_start_s_, t_end_s_;
    bool    is_processing_ = false;

    // Audio controls
    QComboBox*      mod_combo_    = nullptr;
    QDoubleSpinBox* floor_spin_   = nullptr;
    QPushButton*    run_btn_      = nullptr;
    QPushButton*    stop_btn_     = nullptr;
    QSlider*        vol_slider_   = nullptr;
    QLabel*         status_lbl_   = nullptr;
    QProgressBar*   prog_bar_     = nullptr;

    // Plugin chain controls
    QComboBox*      demod_combo_    = nullptr;  // DEMODULATOR plugins
    QComboBox*      decoder_combo_  = nullptr;  // DECODER plugins
    QComboBox*      postproc_combo_ = nullptr;  // POSTPROCESSING plugins
    QPushButton*    chain_run_btn_  = nullptr;
    QPlainTextEdit* chain_output_   = nullptr;

    DemodWaterfallWidget* wf_widget_   = nullptr;
    AudioWaveformWidget*  wav_widget_  = nullptr;

    // Stored raw power data for recolouring without re-FFT.
    QVector<float> pdb_flat_;
    int            pdb_n_bins_   = 0;
    int            pdb_n_frames_ = 0;
    double         pdb_t_start_  = 0.0, pdb_t_end_ = 1.0;

    QAudioSink* audio_sink_    = nullptr;
    QBuffer*    audio_buffer_  = nullptr;
    QByteArray  audio_data_;

    QThread*    worker_thread_ = nullptr;
};

}  // namespace iq_analyzer

// QImage and QVector<float> are auto-registered by Qt6; no Q_DECLARE_METATYPE needed.
