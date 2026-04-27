#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <QComboBox>
#include <QCheckBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QMainWindow>
#include <QMap>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>

#include "grpc_client.hpp"
#include "signal_visualization_widget.hpp"

class QIODevice;
#if !defined(MR_HAS_QT_MULTIMEDIA)
#define MR_HAS_QT_MULTIMEDIA 0
#endif
#if MR_HAS_QT_MULTIMEDIA
class QAudioSink;
#endif

namespace multi_radio {

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(std::string grpc_target, std::string token, QWidget* parent = nullptr);
  ~MainWindow() override;

 private slots:
  void RefreshReceivers();
  void StartSelectedReceiver();
  void StopSelectedReceiver();
  void ApplyModeAndConfig();
  void OpenVisualizationSettingsDialog();

  void OnReceiverEvent(uint32_t receiver_id, int event_kind, double tuned_frequency_hz,
                       const QString& message, quint64 unix_ms);
  void OnDecodedMessage(uint32_t receiver_id, const QString& signal_type, double frequency_hz,
                        const QString& payload, const QVariantMap& fields, quint64 unix_ms);
  void OnStreamError(const QString& error);

 private:
  struct MessageRow {
    QDateTime timestamp;
    uint32_t receiver_id = 0;
    QString signal_type;
    double frequency_hz = 0.0;
    QString payload;
    QString decoded_summary;
  };
  struct AisCrcSummaryState {
    quint64 last_log_unix_ms = 0;
    quint64 last_crc_ok = 0;
    quint64 last_crc_fail = 0;
    quint64 last_emitted = 0;
  };

  bool CurrentReceiverId(uint32_t* receiver_id) const;
  void AppendLog(const QString& line);
  void AddMessageRow(const MessageRow& row);
  bool PassesFilter(const MessageRow& row) const;
  void RefreshScanListChannelCards();
  void ConfigureScanListChannel(int index);
  void ApplyScanListStatusEvent(uint32_t receiver_id, const QString& message);
  QString ScanListChannelCardText(int index) const;
  QString ScanListChannelCardStyle(int index) const;
  bool IsSelectedReceiver(uint32_t receiver_id) const;
  void HandleAudioPcmEvent(const QString& message);
  void EnsureAudioOutputInitialized();
  void LoadScanListConfigFromSettings();
  void SaveScanListConfigToSettings() const;

  struct ScanListChannelConfig {
    QString label;
    double frequency_mhz = 0.0;
    v1::Modulation modulation = v1::MODULATION_NFM;
    int bandwidth_hz = 0;
    double squelch_threshold_db = -30.0;
    int dwell_ms = 0;
  };

  enum class ScanListChannelState {
    kIdle,
    kSquelchClosed,
    kSquelchOpen,
  };

  std::unique_ptr<GrpcClient> client_;
  std::vector<MessageRow> all_rows_;

  QComboBox* receiver_combo_ = nullptr;
  QTabWidget* mode_tabs_ = nullptr;
  int last_tab_index_ = -1;
  int last_mode_tab_index_ = 0;
  QLineEdit* fixed_frequency_edit_ = nullptr;
  QLineEdit* range_start_edit_ = nullptr;
  QLineEdit* range_end_edit_ = nullptr;
  QLineEdit* range_step_edit_ = nullptr;
  QSpinBox* dwell_ms_spin_ = nullptr;
  QSpinBox* sample_rate_spin_ = nullptr;
  QSpinBox* channel_bandwidth_spin_ = nullptr;
  QSpinBox* hardware_bandwidth_spin_ = nullptr;
  QCheckBox* dc_blocker_checkbox_ = nullptr;
  QSpinBox* dc_blocker_cutoff_spin_ = nullptr;
  QCheckBox* center_notch_checkbox_ = nullptr;
  QSpinBox* center_notch_width_spin_ = nullptr;
  QCheckBox* lo_offset_checkbox_ = nullptr;
  QSpinBox* lo_offset_spin_ = nullptr;

  QComboBox* signal_filter_combo_ = nullptr;
  QComboBox* spectrum_source_combo_ = nullptr;
  QComboBox* receiver_filter_combo_ = nullptr;
  QSpinBox* minutes_filter_spin_ = nullptr;
  QMap<QString, AisCrcSummaryState> ais_crc_summary_by_channel_;
  std::array<ScanListChannelConfig, 5> scan_list_channels_;
  std::array<QPushButton*, 5> scan_list_channel_buttons_ = {nullptr, nullptr, nullptr, nullptr, nullptr};
  int active_scan_list_channel_index_ = -1;
  ScanListChannelState active_scan_list_channel_state_ = ScanListChannelState::kIdle;

  QTableWidget* decoded_table_ = nullptr;
  QPlainTextEdit* event_log_ = nullptr;
  SignalVisualizationWidget* signal_visualization_ = nullptr;
#if MR_HAS_QT_MULTIMEDIA
  QAudioSink* audio_sink_ = nullptr;
#endif
  QIODevice* audio_output_device_ = nullptr;
  bool audio_output_disabled_ = false;
};

}  // namespace multi_radio
