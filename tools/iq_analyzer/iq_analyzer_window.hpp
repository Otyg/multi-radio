#pragma once

#include <cstdint>
#include <vector>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QList>
#include <QMainWindow>
#include <QProgressBar>
#include <QPushButton>
#include <QTableWidget>
#include <QThread>

namespace iq_analyzer {

struct DetectedSignal {
    double center_freq_hz;
    double noise_floor_dbfs;  // per-channel noise floor in dBFS
    double time_offset_s;
    double strength_dbfs;     // mean signal power in dBFS (same scale as noise floor)
    double duration_s;
};

// Alias needed so moc can handle it in signal/slot declarations without '::'.
using DetectedSignalList = QList<DetectedSignal>;

// Worker that runs FFT analysis off the GUI thread.
class AnalysisWorker : public QObject {
    Q_OBJECT
public:
    explicit AnalysisWorker(QString path, double center_hz, double sample_rate_hz,
                            double channel_bw_hz, QObject* parent = nullptr);

public slots:
    void run();

signals:
    void progress(int percent);
    void finished(DetectedSignalList detections, double noise_floor_db, QString error);

private:
    QString path_;
    double center_hz_;
    double sample_rate_hz_;
    double channel_bw_hz_;
};

class IqAnalyzerWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit IqAnalyzerWindow(QWidget* parent = nullptr);

private slots:
    void onOpenFile();
    void onAnalyze();
    void onProgress(int percent);
    void onFinished(DetectedSignalList detections, double noise_floor_db, QString error);
    void onRowDoubleClicked(int row, int col);

private:
    void applyFilenameParams(const QString& path);
    double centerFreqHz() const;
    double sampleRateHz() const;
    double channelBwHz() const;
    void setControlsEnabled(bool enabled);
    void populateTable();

    QLineEdit*    file_path_edit_    = nullptr;
    QPushButton*  open_btn_          = nullptr;

    QDoubleSpinBox* cf_spin_         = nullptr;
    QComboBox*      cf_unit_combo_   = nullptr;
    QDoubleSpinBox* sr_spin_         = nullptr;
    QComboBox*      sr_unit_combo_   = nullptr;
    QDoubleSpinBox* bw_spin_         = nullptr;
    QComboBox*      bw_unit_combo_   = nullptr;
    QLabel*         noise_floor_label_ = nullptr;

    QPushButton*    analyze_btn_       = nullptr;
    QProgressBar*   progress_bar_      = nullptr;
    QDoubleSpinBox* strength_filter_spin_ = nullptr;
    QLabel*         status_label_      = nullptr;
    QTableWidget*   results_table_     = nullptr;

    QThread*      worker_thread_     = nullptr;

    // Stored after last successful analysis.
    double             current_cf_hz_         = 0.0;
    double             current_sr_hz_         = 0.0;
    double             current_bw_hz_         = 0.0;
    DetectedSignalList current_detections_;
    DetectedSignalList current_visible_detections_;  // subset currently shown in table
};

}  // namespace iq_analyzer

Q_DECLARE_METATYPE(iq_analyzer::DetectedSignal)
Q_DECLARE_METATYPE(iq_analyzer::DetectedSignalList)
