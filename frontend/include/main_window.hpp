#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <QByteArray>
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
#include "scan_range_visualization_widget.hpp"
#include "signal_visualization_widget.hpp"

class QIODevice;
class QGridLayout;
class QScrollArea;
class QTimer;
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

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override;

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
  void OnAudioFrame(uint32_t receiver_id, int sample_rate_hz, const QByteArray& pcm_s16le,
                    quint64 unix_ms, double tuned_frequency_hz, quint64 sequence,
                    quint64 sample_index);
  void OnIqFrame(uint32_t receiver_id, int sample_rate_hz, const QByteArray& interleaved_iq_s16le,
                 quint64 unix_ms, double tuned_frequency_hz, quint64 sequence,
                 quint64 sample_index);
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
  void AddScanListChannel();
  void ImportScanListCsv();
  void RemoveScanListChannel(int index);
  void ApplyScanListStatusEvent(uint32_t receiver_id, const QString& message);
  void StartAutoSquelchCalibration();
  void CompleteAutoSquelchCalibration();
  QString ScanListChannelCardText(int index) const;
  QString ScanListChannelCardStyle(int index) const;
  bool IsSelectedReceiver(uint32_t receiver_id) const;
  void HandleAudioPcmFrame(int sample_rate_hz, const QByteArray& pcm);
  void HandleAudioPcmEvent(const QString& message);
  void EnsureAudioOutputInitialized(int sample_rate_hz);
  void DrainAudioOutputQueue();
  void MaybeEmitFrontendAudioStats();
  bool ApplyModeAndConfigForReceiver(uint32_t receiver_id, QString* error_text);
  void LoadScanListConfigFromSettings();
  void SaveScanListConfigToSettings() const;

  struct ScanListChannelConfig {
    QString label;
    double frequency_mhz = 0.0;
    v1::Modulation modulation = v1::MODULATION_NFM;
    int bandwidth_hz = 0;
    double squelch_threshold_db = -67.5;
    int dwell_ms = 0;
    bool use_default_squelch = true;
    double audio_gain_db = 0.0;
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
  QCheckBox* scan_list_monitor_checkbox_ = nullptr;
  QDoubleSpinBox* scan_list_default_squelch_spin_ = nullptr;
  QCheckBox* audio_hpf300_checkbox_ = nullptr;
  QCheckBox* audio_lpf8k_checkbox_ = nullptr;
  QCheckBox* audio_lpf15k_checkbox_ = nullptr;
  QCheckBox* audio_bpf_voice_checkbox_ = nullptr;
  QLineEdit* fixed_frequency_edit_ = nullptr;
  QComboBox* fixed_modulation_combo_ = nullptr;
  QLineEdit* range_start_edit_ = nullptr;
  QLineEdit* range_end_edit_ = nullptr;
  QLineEdit* range_step_edit_ = nullptr;
  QComboBox* range_fft_size_combo_ = nullptr;
  QCheckBox* range_noise_gate_checkbox_ = nullptr;
  QDoubleSpinBox* range_noise_gate_spin_ = nullptr;
  QSpinBox* dwell_ms_spin_ = nullptr;
  QSpinBox* sample_rate_spin_ = nullptr;
  QSpinBox* channel_bandwidth_spin_ = nullptr;
  bool fixed_bandwidth_manual_override_ = false;
  bool fixed_bandwidth_sync_in_progress_ = false;
  int fixed_bandwidth_last_auto_hz_ = 0;
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
  std::vector<ScanListChannelConfig> scan_list_channels_;
  std::vector<QPushButton*> scan_list_channel_buttons_;
  QGridLayout* scan_list_grid_layout_ = nullptr;
  QWidget* scan_list_grid_widget_ = nullptr;
  QScrollArea* scan_list_scroll_area_ = nullptr;
  int active_scan_list_channel_index_ = -1;
  ScanListChannelState active_scan_list_channel_state_ = ScanListChannelState::kIdle;
  bool auto_squelch_active_ = false;
  bool auto_squelch_restore_monitor_mode_ = false;
  uint32_t auto_squelch_receiver_id_ = 0;
  int auto_squelch_required_loops_ = 4;
  int auto_squelch_completed_loops_ = 0;
  int auto_squelch_last_channel_index_ = -1;
  bool auto_squelch_has_last_channel_ = false;
  double auto_squelch_signal_sum_db_ = 0.0;
  int auto_squelch_sample_count_ = 0;

  QTableWidget* decoded_table_ = nullptr;
  QPlainTextEdit* event_log_ = nullptr;
  SignalVisualizationWidget* signal_visualization_ = nullptr;
  ScanRangeVisualizationWidget* scan_range_viz_ = nullptr;
#if MR_HAS_QT_MULTIMEDIA
  QAudioSink* audio_sink_ = nullptr;
#endif
  QIODevice* audio_output_device_ = nullptr;
  int audio_output_sample_rate_hz_ = 0;
  double audio_resample_next_source_pos_ = 0.0;
  bool audio_resample_has_prev_sample_ = false;
  int16_t audio_resample_prev_sample_ = 0;
  QByteArray audio_pending_pcm_;
  QTimer* audio_drain_timer_ = nullptr;
  bool audio_prefill_complete_ = false;
  qint64 audio_prefill_started_at_ms_ = 0;
  bool audio_queue_overrun_logged_ = false;
  bool audio_output_disabled_by_env_ = false;
  bool audio_output_disabled_ = false;
  bool iq_visual_dc_suppression_enabled_ = true;
  bool iq_stream_unavailable_notified_ = false;
  bool iq_frame_seen_ = false;
  qint64 audio_backend_stats_last_seen_at_ms_ = 0;
  qint64 audio_frontend_stats_window_started_at_ms_ = 0;
  int audio_frontend_last_rx_sample_rate_hz_ = 0;
  quint64 audio_frontend_rx_frames_ = 0;
  quint64 audio_frontend_rx_bytes_ = 0;
  quint64 audio_frontend_filtered_frames_ = 0;
  quint64 audio_frontend_prefill_wait_events_ = 0;
  quint64 audio_frontend_prefill_complete_events_ = 0;
  quint64 audio_frontend_write_calls_ = 0;
  quint64 audio_frontend_written_bytes_ = 0;
  quint64 audio_frontend_write_blocked_events_ = 0;
  quint64 audio_frontend_overrun_dropped_bytes_ = 0;
  quint64 audio_frontend_gap_fill_bytes_ = 0;
  quint64 audio_frontend_missing_frame_events_ = 0;
  quint64 audio_frontend_missing_sample_bytes_ = 0;
  bool audio_stream_seq_valid_ = false;
  quint64 audio_stream_last_sequence_ = 0;
  bool audio_stream_sample_index_valid_ = false;
  quint64 audio_stream_next_sample_index_ = 0;
};

}  // namespace multi_radio
