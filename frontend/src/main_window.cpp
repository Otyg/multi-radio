#include "main_window.hpp"

#include <chrono>
#include <sstream>

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

  filter_layout->addRow("Signal", signal_filter_combo_);
  filter_layout->addRow("Receiver", receiver_filter_combo_);
  filter_layout->addRow("Last minutes", minutes_filter_spin_);

  top_layout->addWidget(control_group, 2);
  top_layout->addWidget(filter_group, 1);

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
  root_layout->addWidget(splitter);

  setCentralWidget(central);

  connect(refresh_button, &QPushButton::clicked, this, &MainWindow::RefreshReceivers);
  connect(start_button, &QPushButton::clicked, this, &MainWindow::StartSelectedReceiver);
  connect(stop_button, &QPushButton::clicked, this, &MainWindow::StopSelectedReceiver);
  connect(apply_button, &QPushButton::clicked, this, &MainWindow::ApplyModeAndConfig);

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

  for (const auto& receiver : receivers) {
    const QString label = QString("#%1 %2 (%3)")
                              .arg(receiver.receiver_id())
                              .arg(QString::fromStdString(receiver.serial()))
                              .arg(receiver.running() ? "running" : "stopped");
    receiver_combo_->addItem(label, QVariant::fromValue<int>(receiver.receiver_id()));
    receiver_filter_combo_->addItem(QString("#%1").arg(receiver.receiver_id()),
                                    QVariant::fromValue<int>(receiver.receiver_id()));
  }

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

  AppendLog(QString("Applied mode/config to receiver %1").arg(receiver_id));
}

void MainWindow::OnReceiverEvent(uint32_t receiver_id, int event_kind, double tuned_frequency_hz,
                                 const QString& message, quint64 unix_ms) {
  AppendLog(QString("[%1] RX%2 kind=%3 f=%4 %5")
                .arg(ToLocalTime(unix_ms))
                .arg(receiver_id)
                .arg(event_kind)
                .arg(tuned_frequency_hz, 0, 'f', 0)
                .arg(message));
}

void MainWindow::OnDecodedMessage(uint32_t receiver_id, const QString& signal_type, double frequency_hz,
                                  const QString& payload, const QVariantMap& /*fields*/, quint64 unix_ms) {
  MessageRow row;
  row.timestamp = QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(unix_ms)).toLocalTime();
  row.receiver_id = receiver_id;
  row.signal_type = signal_type;
  row.frequency_hz = frequency_hz;
  row.payload = payload;

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
