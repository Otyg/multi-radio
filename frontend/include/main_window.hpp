#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <QComboBox>
#include <QCheckBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMap>
#include <QPlainTextEdit>
#include <QSpinBox>
#include <QTableWidget>

#include "grpc_client.hpp"
#include "signal_visualization_widget.hpp"

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

  std::unique_ptr<GrpcClient> client_;
  std::vector<MessageRow> all_rows_;

  QComboBox* receiver_combo_ = nullptr;
  QComboBox* mode_combo_ = nullptr;
  QLineEdit* fixed_frequency_edit_ = nullptr;
  QLineEdit* range_start_edit_ = nullptr;
  QLineEdit* range_end_edit_ = nullptr;
  QLineEdit* range_step_edit_ = nullptr;
  QLineEdit* list_frequencies_edit_ = nullptr;
  QSpinBox* dwell_ms_spin_ = nullptr;
  QSpinBox* sample_rate_spin_ = nullptr;
  QSpinBox* channel_bandwidth_spin_ = nullptr;
  QSpinBox* hardware_bandwidth_spin_ = nullptr;
  QCheckBox* ais_autotune_checkbox_ = nullptr;
  QCheckBox* ais_baud_trim_checkbox_ = nullptr;

  QComboBox* signal_filter_combo_ = nullptr;
  QComboBox* receiver_filter_combo_ = nullptr;
  QSpinBox* minutes_filter_spin_ = nullptr;
  QLabel* ais_autotune_indicator_ = nullptr;
  QMap<QString, AisCrcSummaryState> ais_crc_summary_by_channel_;

  QTableWidget* decoded_table_ = nullptr;
  QPlainTextEdit* event_log_ = nullptr;
  SignalVisualizationWidget* signal_visualization_ = nullptr;
};

}  // namespace multi_radio
