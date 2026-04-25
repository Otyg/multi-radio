#include "main_window.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSplitter>
#include <QVBoxLayout>
#include <QWidget>

namespace multi_radio {

namespace {

QString ToLocalTime(quint64 unix_ms) {
  return QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(unix_ms)).toLocalTime().toString("HH:mm:ss");
}

bool ParseSeries(const QString& value, std::vector<double>* out) {
  if (out == nullptr) {
    return false;
  }
  out->clear();
  if (value.isEmpty()) {
    return false;
  }
  const QStringList tokens = value.split(',', Qt::SkipEmptyParts);
  out->reserve(tokens.size());
  for (const QString& token : tokens) {
    bool ok = false;
    const double parsed = token.toDouble(&ok);
    if (!ok) {
      return false;
    }
    out->push_back(parsed);
  }
  return !out->empty();
}

bool ParseVisualizationFrameEvent(const QString& message, double* peak_hz, double* peak_strength,
                                  std::vector<double>* waveform, std::vector<double>* spectrum) {
  if (peak_hz == nullptr || peak_strength == nullptr || waveform == nullptr || spectrum == nullptr) {
    return false;
  }
  if (!message.startsWith("VIZ_FRAME ")) {
    return false;
  }

  const QStringList tokens = message.split(' ', Qt::SkipEmptyParts);
  if (tokens.size() < 5) {
    return false;
  }

  bool peak_hz_ok = false;
  bool peak_strength_ok = false;
  bool waveform_ok = false;
  bool spectrum_ok = false;
  double parsed_peak_hz = 0.0;
  double parsed_peak_strength = 0.0;
  std::vector<double> parsed_waveform;
  std::vector<double> parsed_spectrum;
  for (int i = 1; i < tokens.size(); ++i) {
    const QString token = tokens[i];
    if (token.startsWith("peak_hz=")) {
      parsed_peak_hz = token.mid(8).toDouble(&peak_hz_ok);
    } else if (token.startsWith("peak_strength=")) {
      parsed_peak_strength = token.mid(14).toDouble(&peak_strength_ok);
    } else if (token.startsWith("waveform=")) {
      waveform_ok = ParseSeries(token.mid(9), &parsed_waveform);
    } else if (token.startsWith("spectrum=")) {
      spectrum_ok = ParseSeries(token.mid(9), &parsed_spectrum);
    }
  }

  if (!peak_hz_ok || !peak_strength_ok || !waveform_ok || !spectrum_ok) {
    return false;
  }
  *peak_hz = parsed_peak_hz;
  *peak_strength = std::clamp(parsed_peak_strength, 0.0, 1.0);
  *waveform = std::move(parsed_waveform);
  *spectrum = std::move(parsed_spectrum);
  return true;
}

}  // namespace

MainWindow::MainWindow(std::string grpc_target, std::string token, QWidget* parent)
    : QMainWindow(parent), client_(std::make_unique<GrpcClient>(std::move(grpc_target), std::move(token), this)) {
  setWindowTitle("Multi-Radio Client");
  resize(1300, 780);

  auto* central = new QWidget(this);
  auto* root_layout = new QVBoxLayout(central);

  auto* top_layout = new QHBoxLayout();

  auto* control_group = new QGroupBox("Receiver Control", central);
  auto* control_layout = new QFormLayout(control_group);

  receiver_combo_ = new QComboBox(control_group);
  mode_combo_ = new QComboBox(control_group);
  mode_combo_->addItem("FIXED", QVariant::fromValue<int>(v1::RADIO_MODE_FIXED));
  mode_combo_->addItem("SCAN_RANGE", QVariant::fromValue<int>(v1::RADIO_MODE_SCAN_RANGE));
  mode_combo_->addItem("SCAN_LIST", QVariant::fromValue<int>(v1::RADIO_MODE_SCAN_LIST));
  mode_combo_->addItem("AIR_MARINE_PLOT", QVariant::fromValue<int>(v1::RADIO_MODE_AIR_MARINE_PLOT));

  fixed_frequency_edit_ = new QLineEdit("162025000", control_group);
  range_start_edit_ = new QLineEdit("156000000", control_group);
  range_end_edit_ = new QLineEdit("163000000", control_group);
  range_step_edit_ = new QLineEdit("25000", control_group);
  list_frequencies_edit_ = new QLineEdit("161975000,162025000,156525000,1090000000", control_group);

  dwell_ms_spin_ = new QSpinBox(control_group);
  dwell_ms_spin_->setRange(100, 10000);
  dwell_ms_spin_->setValue(500);

  ais_squelch_db_spin_ = new QDoubleSpinBox(control_group);
  ais_squelch_db_spin_->setDecimals(1);
  ais_squelch_db_spin_->setRange(0.0, 30.0);
  ais_squelch_db_spin_->setSingleStep(0.5);
  ais_squelch_db_spin_->setValue(6.0);
  ais_squelch_db_spin_->setSuffix(" dB");

  ais_min_signal_spin_ = new QDoubleSpinBox(control_group);
  ais_min_signal_spin_->setDecimals(4);
  ais_min_signal_spin_->setRange(0.0001, 1.0);
  ais_min_signal_spin_->setSingleStep(0.0005);
  ais_min_signal_spin_->setValue(0.0030);

  ais_hangover_spin_ = new QSpinBox(control_group);
  ais_hangover_spin_->setRange(0, 20);
  ais_hangover_spin_->setValue(2);

  auto* button_row = new QWidget(control_group);
  auto* button_layout = new QHBoxLayout(button_row);
  button_layout->setContentsMargins(0, 0, 0, 0);
  auto* refresh_button = new QPushButton("Refresh", button_row);
  auto* start_button = new QPushButton("Start", button_row);
  auto* stop_button = new QPushButton("Stop", button_row);
  auto* apply_button = new QPushButton("Apply mode/config", button_row);
  button_layout->addWidget(refresh_button);
  button_layout->addWidget(start_button);
  button_layout->addWidget(stop_button);
  button_layout->addWidget(apply_button);

  control_layout->addRow("Receiver", receiver_combo_);
  control_layout->addRow("Mode", mode_combo_);
  control_layout->addRow("Fixed Hz", fixed_frequency_edit_);
  control_layout->addRow("Range Start Hz", range_start_edit_);
  control_layout->addRow("Range End Hz", range_end_edit_);
  control_layout->addRow("Range Step Hz", range_step_edit_);
  control_layout->addRow("List Hz (comma)", list_frequencies_edit_);
  control_layout->addRow("Dwell ms", dwell_ms_spin_);
  control_layout->addRow("AIS squelch SNR", ais_squelch_db_spin_);
  control_layout->addRow("AIS min signal", ais_min_signal_spin_);
  control_layout->addRow("AIS hangover", ais_hangover_spin_);
  control_layout->addRow(button_row);

  auto* filter_group = new QGroupBox("Message Filters", central);
  auto* filter_layout = new QFormLayout(filter_group);
  signal_filter_combo_ = new QComboBox(filter_group);
  signal_filter_combo_->addItem("ALL");
  signal_filter_combo_->addItem("SIGNAL_TYPE_AIS");
  signal_filter_combo_->addItem("SIGNAL_TYPE_ADSB");
  signal_filter_combo_->addItem("SIGNAL_TYPE_DSC");

  receiver_filter_combo_ = new QComboBox(filter_group);
  receiver_filter_combo_->addItem("ALL", QVariant::fromValue(-1));

  minutes_filter_spin_ = new QSpinBox(filter_group);
  minutes_filter_spin_->setRange(1, 240);
  minutes_filter_spin_->setValue(30);
  auto* visualization_settings_button = new QPushButton("Visualization settings...", filter_group);

  filter_layout->addRow("Signal", signal_filter_combo_);
  filter_layout->addRow("Receiver", receiver_filter_combo_);
  filter_layout->addRow("Last minutes", minutes_filter_spin_);
  filter_layout->addRow(visualization_settings_button);

  top_layout->addWidget(control_group, 2);
  top_layout->addWidget(filter_group, 1);

  signal_visualization_ = new SignalVisualizationWidget(central);

  decoded_table_ = new QTableWidget(0, 5, central);
  decoded_table_->setHorizontalHeaderLabels({"Time", "Receiver", "Signal", "Frequency", "Payload"});
  decoded_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  event_log_ = new QPlainTextEdit(central);
  event_log_->setReadOnly(true);

  auto* splitter = new QSplitter(Qt::Vertical, central);
  splitter->addWidget(decoded_table_);
  splitter->addWidget(event_log_);
  splitter->setSizes({450, 250});

  root_layout->addLayout(top_layout);
  root_layout->addWidget(signal_visualization_);
  root_layout->addWidget(splitter);

  setCentralWidget(central);

  connect(refresh_button, &QPushButton::clicked, this, &MainWindow::RefreshReceivers);
  connect(start_button, &QPushButton::clicked, this, &MainWindow::StartSelectedReceiver);
  connect(stop_button, &QPushButton::clicked, this, &MainWindow::StopSelectedReceiver);
  connect(apply_button, &QPushButton::clicked, this, &MainWindow::ApplyModeAndConfig);
  connect(visualization_settings_button, &QPushButton::clicked, this,
          &MainWindow::OpenVisualizationSettingsDialog);

  connect(signal_filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
    decoded_table_->setRowCount(0);
    for (const auto& row : all_rows_) {
      AddMessageRow(row);
    }
  });
  connect(receiver_filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
    decoded_table_->setRowCount(0);
    for (const auto& row : all_rows_) {
      AddMessageRow(row);
    }
    signal_visualization_->SetReceiverFilter(receiver_filter_combo_->currentData().toInt());
  });
  connect(minutes_filter_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this]() {
    decoded_table_->setRowCount(0);
    for (const auto& row : all_rows_) {
      AddMessageRow(row);
    }
  });

  connect(client_.get(), &GrpcClient::ReceiverEventReceived, this, &MainWindow::OnReceiverEvent);
  connect(client_.get(), &GrpcClient::DecodedMessageReceived, this, &MainWindow::OnDecodedMessage);
  connect(client_.get(), &GrpcClient::StreamError, this, &MainWindow::OnStreamError);

  RefreshReceivers();
  client_->StartStreaming();
}

MainWindow::~MainWindow() { client_->StopStreaming(); }

void MainWindow::RefreshReceivers() {
  std::vector<v1::ReceiverInfo> receivers;
  std::string error;
  if (!client_->ListReceivers(&receivers, &error)) {
    QMessageBox::warning(this, "ListReceivers failed", QString::fromStdString(error));
    return;
  }

  receiver_combo_->clear();
  receiver_filter_combo_->clear();
  receiver_filter_combo_->addItem("ALL", QVariant::fromValue(-1));
  std::vector<uint32_t> receiver_ids;
  receiver_ids.reserve(receivers.size());

  for (const auto& receiver : receivers) {
    const QString label = QString("#%1 %2 (%3)")
                              .arg(receiver.receiver_id())
                              .arg(QString::fromStdString(receiver.serial()))
                              .arg(receiver.running() ? "running" : "stopped");
    receiver_ids.push_back(receiver.receiver_id());
    receiver_combo_->addItem(label, QVariant::fromValue<int>(receiver.receiver_id()));
    receiver_filter_combo_->addItem(QString("#%1").arg(receiver.receiver_id()),
                                    QVariant::fromValue<int>(receiver.receiver_id()));
  }

  signal_visualization_->SetKnownReceivers(receiver_ids);
  signal_visualization_->SetReceiverFilter(receiver_filter_combo_->currentData().toInt());

  AppendLog(QString("Refreshed %1 receivers").arg(receivers.size()));
}

void MainWindow::StartSelectedReceiver() {
  uint32_t receiver_id = 0;
  if (!CurrentReceiverId(&receiver_id)) {
    return;
  }

  std::string error;
  if (!client_->StartReceiver(receiver_id, &error)) {
    QMessageBox::warning(this, "StartReceiver failed", QString::fromStdString(error));
    return;
  }
  AppendLog(QString("Start requested for receiver %1").arg(receiver_id));
  RefreshReceivers();
}

void MainWindow::StopSelectedReceiver() {
  uint32_t receiver_id = 0;
  if (!CurrentReceiverId(&receiver_id)) {
    return;
  }

  std::string error;
  if (!client_->StopReceiver(receiver_id, &error)) {
    QMessageBox::warning(this, "StopReceiver failed", QString::fromStdString(error));
    return;
  }
  AppendLog(QString("Stop requested for receiver %1").arg(receiver_id));
  RefreshReceivers();
}

void MainWindow::ApplyModeAndConfig() {
  uint32_t receiver_id = 0;
  if (!CurrentReceiverId(&receiver_id)) {
    return;
  }

  const v1::RadioMode mode = static_cast<v1::RadioMode>(mode_combo_->currentData().toInt());

  std::string error;
  if (!client_->SetMode(receiver_id, mode, &error)) {
    QMessageBox::warning(this, "SetMode failed", QString::fromStdString(error));
    return;
  }

  v1::ModeConfig config;
  config.set_fixed_frequency_hz(fixed_frequency_edit_->text().toDouble());
  config.set_range_start_hz(range_start_edit_->text().toDouble());
  config.set_range_end_hz(range_end_edit_->text().toDouble());
  config.set_range_step_hz(range_step_edit_->text().toDouble());
  config.set_dwell_ms(static_cast<uint32_t>(dwell_ms_spin_->value()));

  const auto list_tokens = list_frequencies_edit_->text().split(',', Qt::SkipEmptyParts);
  for (const auto& token : list_tokens) {
    config.add_frequency_list_hz(token.trimmed().toDouble());
  }

  if (!client_->SetModeConfig(receiver_id, config, &error)) {
    QMessageBox::warning(this, "SetModeConfig failed", QString::fromStdString(error));
    return;
  }

  if (!client_->SetAisSquelch(ais_squelch_db_spin_->value(), ais_min_signal_spin_->value(),
                              static_cast<uint32_t>(ais_hangover_spin_->value()), &error)) {
    QMessageBox::warning(this, "SetAisSquelch failed", QString::fromStdString(error));
    return;
  }

  AppendLog(QString("Applied mode/config + AIS squelch to receiver %1 (snr=%2 dB min=%3 hang=%4)")
                .arg(receiver_id)
                .arg(ais_squelch_db_spin_->value(), 0, 'f', 1)
                .arg(ais_min_signal_spin_->value(), 0, 'f', 4)
                .arg(ais_hangover_spin_->value()));
}

void MainWindow::OnReceiverEvent(uint32_t receiver_id, int event_kind, double tuned_frequency_hz,
                                 const QString& message, quint64 unix_ms) {
  double peak_hz = 0.0;
  double peak_strength = 0.0;
  std::vector<double> waveform;
  std::vector<double> spectrum;
  if (ParseVisualizationFrameEvent(message, &peak_hz, &peak_strength, &waveform, &spectrum)) {
    signal_visualization_->PushVisualizationFrame(receiver_id, waveform, spectrum, peak_hz, peak_strength);
    return;
  }

  AppendLog(QString("[%1] RX%2 kind=%3 f=%4 %5")
                .arg(ToLocalTime(unix_ms))
                .arg(receiver_id)
                .arg(event_kind)
                .arg(tuned_frequency_hz, 0, 'f', 0)
                .arg(message));
}

void MainWindow::OnDecodedMessage(uint32_t receiver_id, const QString& signal_type, double frequency_hz,
                                  const QString& payload, const QVariantMap& fields, quint64 unix_ms) {
  MessageRow row;
  row.timestamp = QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(unix_ms)).toLocalTime();
  row.receiver_id = receiver_id;
  row.signal_type = signal_type;
  row.frequency_hz = frequency_hz;
  row.payload = payload;

  auto field_text = [&fields](const QString& key) -> QString {
    if (!fields.contains(key)) {
      return {};
    }
    return fields.value(key).toString();
  };

  const QString kind = field_text("kind");
  if (signal_type == "SIGNAL_TYPE_AIS" && kind == "metric") {
    AppendLog(QString("[%1] RX%2 AIS metrics ch=%3 blocks=%4 flags=%5 cand=%6 ok=%7 fail=%8 dup=%9 emitted=%10 abs=%11 sq_open=%12 sq_snr=%13 sq_noise=%14 sq_sig=%15 emitted_now=%16")
                  .arg(ToLocalTime(unix_ms))
                  .arg(receiver_id)
                  .arg(field_text("channel"))
                  .arg(field_text("metric_blocks"))
                  .arg(field_text("metric_flags"))
                  .arg(field_text("metric_candidates"))
                  .arg(field_text("metric_crc_ok"))
                  .arg(field_text("metric_crc_fail"))
                  .arg(field_text("metric_duplicates"))
                  .arg(field_text("metric_emitted"))
                  .arg(field_text("metric_abs_mean"))
                  .arg(field_text("metric_squelch_open"))
                  .arg(field_text("metric_squelch_snr_db"))
                  .arg(field_text("metric_squelch_noise"))
                  .arg(field_text("metric_squelch_signal"))
                  .arg(field_text("metric_emitted_this_block")));
    return;
  }

  QString summary = QString("[%1] RX%2 %3 f=%4")
                        .arg(ToLocalTime(unix_ms))
                        .arg(receiver_id)
                        .arg(signal_type)
                        .arg(frequency_hz, 0, 'f', 0);
  const QString mmsi = field_text("mmsi");
  const QString msg_type = field_text("msg_type");
  const QString channel = field_text("channel");
  if (!mmsi.isEmpty()) {
    summary += QString(" mmsi=%1").arg(mmsi);
  }
  if (!msg_type.isEmpty()) {
    summary += QString(" type=%1").arg(msg_type);
  }
  if (!channel.isEmpty()) {
    summary += QString(" ch=%1").arg(channel);
  }
  summary += QString(" payload=%1").arg(payload.left(96));
  AppendLog(summary);

  const QString metric_blocks = field_text("metric_blocks");
  if (!metric_blocks.isEmpty()) {
    AppendLog(QString("AIS metrics: blocks=%1 flags=%2 candidates=%3 crc_ok=%4 crc_fail=%5 dup=%6 emitted=%7 sq_open=%8 sq_snr=%9")
                  .arg(metric_blocks)
                  .arg(field_text("metric_flags"))
                  .arg(field_text("metric_candidates"))
                  .arg(field_text("metric_crc_ok"))
                  .arg(field_text("metric_crc_fail"))
                  .arg(field_text("metric_duplicates"))
                  .arg(field_text("metric_emitted"))
                  .arg(field_text("metric_squelch_open"))
                  .arg(field_text("metric_squelch_snr_db")));
  }

  all_rows_.push_back(row);
  AddMessageRow(row);
}

void MainWindow::OnStreamError(const QString& error) {
  AppendLog(QString("Stream error: %1").arg(error));
}

bool MainWindow::CurrentReceiverId(uint32_t* receiver_id) const {
  if (receiver_combo_->currentIndex() < 0) {
    QMessageBox::warning(const_cast<MainWindow*>(this), "No receiver", "Select a receiver first");
    return false;
  }
  if (receiver_id != nullptr) {
    *receiver_id = static_cast<uint32_t>(receiver_combo_->currentData().toInt());
  }
  return true;
}

void MainWindow::AppendLog(const QString& line) {
  event_log_->appendPlainText(line);
}

void MainWindow::OpenVisualizationSettingsDialog() {
  QDialog dialog(this);
  dialog.setWindowTitle("Visualization settings");

  auto* layout = new QFormLayout(&dialog);
  auto* fft_combo = new QComboBox(&dialog);
  const int fft_sizes[] = {64, 128, 256, 512, 1024, 2048, 4096};
  for (const int fft_size : fft_sizes) {
    fft_combo->addItem(QString::number(fft_size), QVariant::fromValue(fft_size));
  }
  const int current_fft = signal_visualization_->FftSize();
  const int fft_index = fft_combo->findData(QVariant::fromValue(current_fft));
  if (fft_index >= 0) {
    fft_combo->setCurrentIndex(fft_index);
  }

  auto* start_hz_spin = new QDoubleSpinBox(&dialog);
  start_hz_spin->setDecimals(0);
  start_hz_spin->setRange(0.0, 6000000000.0);
  start_hz_spin->setSingleStep(25000.0);
  start_hz_spin->setSuffix(" Hz");
  start_hz_spin->setValue(signal_visualization_->FrequencyStartHz());

  auto* end_hz_spin = new QDoubleSpinBox(&dialog);
  end_hz_spin->setDecimals(0);
  end_hz_spin->setRange(0.0, 6000000000.0);
  end_hz_spin->setSingleStep(25000.0);
  end_hz_spin->setSuffix(" Hz");
  end_hz_spin->setValue(signal_visualization_->FrequencyEndHz());

  layout->addRow("FFT size", fft_combo);
  layout->addRow("Frequency start", start_hz_spin);
  layout->addRow("Frequency end", end_hz_spin);

  auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  layout->addRow(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

  if (dialog.exec() != QDialog::Accepted) {
    return;
  }

  const int fft_size = fft_combo->currentData().toInt();
  const double start_hz = start_hz_spin->value();
  const double end_hz = end_hz_spin->value();
  signal_visualization_->SetVisualizationSettings(fft_size, start_hz, end_hz);
  AppendLog(QString("Updated visualization settings: FFT=%1, range=%2-%3 Hz")
                .arg(fft_size)
                .arg(start_hz, 0, 'f', 0)
                .arg(end_hz, 0, 'f', 0));
}

void MainWindow::AddMessageRow(const MessageRow& row) {
  if (!PassesFilter(row)) {
    return;
  }

  const int current = decoded_table_->rowCount();
  decoded_table_->insertRow(current);
  decoded_table_->setItem(current, 0, new QTableWidgetItem(row.timestamp.toString("HH:mm:ss")));
  decoded_table_->setItem(current, 1, new QTableWidgetItem(QString::number(row.receiver_id)));
  decoded_table_->setItem(current, 2, new QTableWidgetItem(row.signal_type));
  decoded_table_->setItem(current, 3, new QTableWidgetItem(QString::number(row.frequency_hz, 'f', 0)));
  decoded_table_->setItem(current, 4, new QTableWidgetItem(row.payload));
}

bool MainWindow::PassesFilter(const MessageRow& row) const {
  const QString signal_filter = signal_filter_combo_->currentText();
  if (signal_filter != "ALL" && row.signal_type != signal_filter) {
    return false;
  }

  const int receiver_filter = receiver_filter_combo_->currentData().toInt();
  if (receiver_filter >= 0 && static_cast<int>(row.receiver_id) != receiver_filter) {
    return false;
  }

  const int minutes = minutes_filter_spin_->value();
  const QDateTime cutoff = QDateTime::currentDateTime().addSecs(-minutes * 60);
  return row.timestamp >= cutoff;
}

}  // namespace multi_radio
