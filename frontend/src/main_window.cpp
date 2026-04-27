#include "main_window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <sstream>

#include <QByteArray>
#include <QAbstractButton>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QEvent>
#include <QFontMetrics>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QIODevice>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSettings>
#include <QSplitter>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#if MR_HAS_QT_MULTIMEDIA
#include <QAudioDevice>
#include <QAudioFormat>
#include <QAudioSink>
#include <QMediaDevices>
#endif

namespace multi_radio {

namespace {

constexpr int kFixedModeTabIndex = 0;
constexpr int kScanRangeModeTabIndex = 1;
constexpr int kScanListModeTabIndex = 2;
constexpr int kAirMarineModeTabIndex = 3;
constexpr int kGlobalSettingsTabIndex = 4;
constexpr double kDefaultScanListSquelchDb = -30.0;

int DefaultBandwidthHzForModulation(v1::Modulation modulation) {
  switch (modulation) {
    case v1::MODULATION_AM:
      return 10000;
    case v1::MODULATION_WFM:
      return 180000;
    case v1::MODULATION_NFM:
    case v1::MODULATION_UNSPECIFIED:
    default:
      return 12500;
  }
}

QString ModulationLabel(v1::Modulation modulation) {
  switch (modulation) {
    case v1::MODULATION_AM:
      return "AM";
    case v1::MODULATION_WFM:
      return "WFM";
    case v1::MODULATION_NFM:
    case v1::MODULATION_UNSPECIFIED:
    default:
      return "NFM";
  }
}

v1::Modulation ModulationFromText(const QString& text) {
  const QString upper = text.trimmed().toUpper();
  if (upper == "AM") {
    return v1::MODULATION_AM;
  }
  if (upper == "WFM") {
    return v1::MODULATION_WFM;
  }
  return v1::MODULATION_NFM;
}

bool TryParseCsvModulation(const QString& text, v1::Modulation* out) {
  if (out == nullptr) {
    return false;
  }
  const QString upper = text.trimmed().toUpper();
  if (upper == "AM") {
    *out = v1::MODULATION_AM;
    return true;
  }
  if (upper == "WFM" || upper == "WIDEFM" || upper == "FM_BROADCAST") {
    *out = v1::MODULATION_WFM;
    return true;
  }
  if (upper == "NFM" || upper == "FM") {
    *out = v1::MODULATION_NFM;
    return true;
  }
  return false;
}

QString TokenValue(const QString& message, const QString& key) {
  const QStringList tokens = message.split(' ', Qt::SkipEmptyParts);
  const QString prefix = key + "=";
  for (const QString& token : tokens) {
    if (token.startsWith(prefix)) {
      return token.mid(prefix.size());
    }
  }
  return {};
}

v1::RadioMode ModeFromTabIndex(int tab_index) {
  switch (tab_index) {
    case kFixedModeTabIndex:
      return v1::RADIO_MODE_FIXED;
    case kScanRangeModeTabIndex:
      return v1::RADIO_MODE_SCAN_RANGE;
    case kScanListModeTabIndex:
      return v1::RADIO_MODE_SCAN_LIST;
    case kAirMarineModeTabIndex:
      return v1::RADIO_MODE_AIR_MARINE_PLOT;
    default:
      return v1::RADIO_MODE_FIXED;
  }
}

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
                                  std::vector<double>* waveform, std::vector<double>* spectrum,
                                  SignalVisualizationWidget::SpectrumSource* source,
                                  double* frame_frequency_start_hz,
                                  double* frame_frequency_end_hz) {
  if (peak_hz == nullptr || peak_strength == nullptr || waveform == nullptr || spectrum == nullptr) {
    return false;
  }
  if (source == nullptr || frame_frequency_start_hz == nullptr || frame_frequency_end_hz == nullptr) {
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
  bool frequency_start_ok = false;
  bool frequency_end_ok = false;
  double parsed_peak_hz = 0.0;
  double parsed_peak_strength = 0.0;
  double parsed_frequency_start_hz = 0.0;
  double parsed_frequency_end_hz = 20000.0;
  std::vector<double> parsed_waveform;
  std::vector<double> parsed_spectrum;
  SignalVisualizationWidget::SpectrumSource parsed_source =
      SignalVisualizationWidget::SpectrumSource::kDemodulated;
  for (int i = 1; i < tokens.size(); ++i) {
    const QString token = tokens[i];
    if (token.startsWith("peak_hz=")) {
      parsed_peak_hz = token.mid(8).toDouble(&peak_hz_ok);
    } else if (token.startsWith("peak_strength=")) {
      parsed_peak_strength = token.mid(14).toDouble(&peak_strength_ok);
    } else if (token.startsWith("source=")) {
      const QString source_value = token.mid(7).trimmed().toLower();
      if (source_value == "receiver") {
        parsed_source = SignalVisualizationWidget::SpectrumSource::kReceiverInput;
      } else {
        parsed_source = SignalVisualizationWidget::SpectrumSource::kDemodulated;
      }
    } else if (token.startsWith("start_hz=")) {
      parsed_frequency_start_hz = token.mid(9).toDouble(&frequency_start_ok);
    } else if (token.startsWith("end_hz=")) {
      parsed_frequency_end_hz = token.mid(7).toDouble(&frequency_end_ok);
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
  *source = parsed_source;
  if (frequency_start_ok && frequency_end_ok && parsed_frequency_end_hz > parsed_frequency_start_hz) {
    *frame_frequency_start_hz = parsed_frequency_start_hz;
    *frame_frequency_end_hz = parsed_frequency_end_hz;
  } else {
    *frame_frequency_start_hz = 0.0;
    *frame_frequency_end_hz = 20000.0;
  }
  return true;
}

bool EnvFlagEnabled(const char* key) {
  const char* value = std::getenv(key);
  if (value == nullptr) {
    return false;
  }
  const QString normalized = QString::fromUtf8(value).trimmed().toLower();
  return normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on";
}

bool IsWslEnvironment() {
  return std::getenv("WSL_DISTRO_NAME") != nullptr || std::getenv("WSL_INTEROP") != nullptr;
}

struct ParsedAisNmeaSentence {
  std::vector<int> bits;
  QString channel;
};

int DecodeAisSixBitValue(QChar c) {
  const int ascii = c.unicode();
  if (ascii < 48 || ascii > 119) {
    return -1;
  }
  int value = ascii - 48;
  if (value > 40) {
    value -= 8;
  }
  if (value < 0 || value > 63) {
    return -1;
  }
  return value;
}

bool ReadUnsignedBits(const std::vector<int>& bits, int start, int length, uint32_t* out) {
  if (out == nullptr || start < 0 || length <= 0 || length > 32) {
    return false;
  }
  const size_t begin = static_cast<size_t>(start);
  const size_t end = begin + static_cast<size_t>(length);
  if (end > bits.size()) {
    return false;
  }
  uint32_t value = 0;
  for (size_t idx = begin; idx < end; ++idx) {
    value = (value << 1U) | static_cast<uint32_t>(bits[idx] != 0 ? 1 : 0);
  }
  *out = value;
  return true;
}

bool ReadSignedBits(const std::vector<int>& bits, int start, int length, int32_t* out) {
  if (out == nullptr || length <= 0 || length >= 32) {
    return false;
  }
  uint32_t raw = 0;
  if (!ReadUnsignedBits(bits, start, length, &raw)) {
    return false;
  }
  const uint32_t sign_bit = 1U << static_cast<uint32_t>(length - 1);
  if ((raw & sign_bit) == 0U) {
    *out = static_cast<int32_t>(raw);
    return true;
  }
  const uint32_t mask = (1U << static_cast<uint32_t>(length)) - 1U;
  *out = static_cast<int32_t>(raw | ~mask);
  return true;
}

QString DecodeAisSixBitText(const std::vector<int>& bits, int start, int length) {
  if (start < 0 || length <= 0 || (length % 6) != 0) {
    return {};
  }
  if (static_cast<size_t>(start + length) > bits.size()) {
    return {};
  }
  static const char* kAisCharset =
      "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?";
  QString out;
  out.reserve(length / 6);
  for (int i = 0; i < length; i += 6) {
    uint32_t value = 0;
    if (!ReadUnsignedBits(bits, start + i, 6, &value)) {
      return {};
    }
    out.append(QChar::fromLatin1(kAisCharset[value]));
  }
  out.replace('@', ' ');
  return out.trimmed();
}

std::optional<double> DecodeLonDeg(const std::vector<int>& bits, int start, int length) {
  int32_t raw = 0;
  if (!ReadSignedBits(bits, start, length, &raw)) {
    return std::nullopt;
  }
  const double degrees = static_cast<double>(raw) / 600000.0;
  if (degrees < -180.0 || degrees > 180.0) {
    return std::nullopt;
  }
  return degrees;
}

std::optional<double> DecodeLatDeg(const std::vector<int>& bits, int start, int length) {
  int32_t raw = 0;
  if (!ReadSignedBits(bits, start, length, &raw)) {
    return std::nullopt;
  }
  const double degrees = static_cast<double>(raw) / 600000.0;
  if (degrees < -90.0 || degrees > 90.0) {
    return std::nullopt;
  }
  return degrees;
}

std::optional<double> DecodeTenths(const std::vector<int>& bits, int start, int length,
                                   uint32_t unavailable) {
  uint32_t raw = 0;
  if (!ReadUnsignedBits(bits, start, length, &raw) || raw == unavailable) {
    return std::nullopt;
  }
  return static_cast<double>(raw) / 10.0;
}

std::optional<int> DecodeUnsignedMaybe(const std::vector<int>& bits, int start, int length,
                                       uint32_t unavailable) {
  uint32_t raw = 0;
  if (!ReadUnsignedBits(bits, start, length, &raw) || raw == unavailable) {
    return std::nullopt;
  }
  return static_cast<int>(raw);
}

bool ParseAisNmeaSentence(const QString& sentence, ParsedAisNmeaSentence* out) {
  if (out == nullptr) {
    return false;
  }
  const QString trimmed = sentence.trimmed();
  if (trimmed.isEmpty()) {
    return false;
  }

  const int star = trimmed.indexOf('*');
  QString body = (star >= 0) ? trimmed.left(star) : trimmed;
  if (body.startsWith('!') || body.startsWith('$')) {
    body = body.mid(1);
  }
  const QStringList parts = body.split(',');
  if (parts.size() < 7) {
    return false;
  }
  if (!(parts[0].endsWith("VDM") || parts[0].endsWith("VDO"))) {
    return false;
  }
  bool fragments_ok = false;
  const int fragments = parts[1].toInt(&fragments_ok);
  bool fragment_no_ok = false;
  const int fragment_no = parts[2].toInt(&fragment_no_ok);
  if (!fragments_ok || !fragment_no_ok || fragments <= 0 || fragment_no <= 0 || fragments != 1 ||
      fragment_no != 1) {
    return false;
  }

  bool fill_ok = false;
  const int fill_bits = parts[6].toInt(&fill_ok);
  if (!fill_ok || fill_bits < 0 || fill_bits > 5) {
    return false;
  }
  const QString sixbit_payload = parts[5];
  if (sixbit_payload.isEmpty()) {
    return false;
  }

  std::vector<int> bits;
  bits.reserve(static_cast<size_t>(sixbit_payload.size()) * 6U);
  for (QChar c : sixbit_payload) {
    const int value = DecodeAisSixBitValue(c);
    if (value < 0) {
      return false;
    }
    for (int bit = 5; bit >= 0; --bit) {
      bits.push_back((value >> bit) & 1);
    }
  }
  if (fill_bits > static_cast<int>(bits.size())) {
    return false;
  }
  if (fill_bits > 0) {
    bits.resize(bits.size() - static_cast<size_t>(fill_bits));
  }

  out->bits = std::move(bits);
  out->channel = parts[4].trimmed();
  return !out->bits.empty();
}

QString NavigationStatusToString(uint32_t status) {
  switch (status) {
    case 0:
      return "under_way";
    case 1:
      return "at_anchor";
    case 2:
      return "not_under_command";
    case 3:
      return "restricted_maneuverability";
    case 4:
      return "constrained_by_draught";
    case 5:
      return "moored";
    case 6:
      return "aground";
    case 7:
      return "fishing";
    case 8:
      return "under_way_sailing";
    case 15:
      return "not_defined";
    default:
      return QString("status_%1").arg(status);
  }
}

QString BuildAisDecodedSummary(const QString& sentence, const QVariantMap& fields) {
  ParsedAisNmeaSentence parsed;
  if (!ParseAisNmeaSentence(sentence, &parsed)) {
    return {};
  }

  auto field_text = [&fields](const QString& key) -> QString {
    if (!fields.contains(key)) {
      return {};
    }
    return fields.value(key).toString();
  };

  uint32_t message_type = 0;
  bool type_ok = false;
  const QString msg_type_text = field_text("msg_type");
  if (!msg_type_text.isEmpty()) {
    message_type = msg_type_text.toUInt(&type_ok);
  }
  if (!type_ok && !ReadUnsignedBits(parsed.bits, 0, 6, &message_type)) {
    return {};
  }

  uint32_t mmsi = 0;
  bool mmsi_ok = false;
  const QString mmsi_text = field_text("mmsi");
  if (!mmsi_text.isEmpty()) {
    mmsi = mmsi_text.toUInt(&mmsi_ok);
  }
  if (!mmsi_ok) {
    ReadUnsignedBits(parsed.bits, 8, 30, &mmsi);
  }

  QStringList summary_parts;
  summary_parts << QString("type=%1").arg(message_type);
  if (mmsi > 0) {
    summary_parts << QString("mmsi=%1").arg(mmsi);
  }
  if (!parsed.channel.isEmpty()) {
    summary_parts << QString("radio_ch=%1").arg(parsed.channel);
  }

  auto append_double = [&summary_parts](const QString& key, const std::optional<double>& value,
                                        int precision) {
    if (!value.has_value()) {
      return;
    }
    summary_parts << QString("%1=%2").arg(key).arg(QString::number(*value, 'f', precision));
  };
  auto append_int = [&summary_parts](const QString& key, const std::optional<int>& value) {
    if (!value.has_value()) {
      return;
    }
    summary_parts << QString("%1=%2").arg(key).arg(*value);
  };

  if (message_type >= 1 && message_type <= 3) {
    uint32_t nav_status = 0;
    if (ReadUnsignedBits(parsed.bits, 38, 4, &nav_status)) {
      summary_parts << QString("nav=%1").arg(NavigationStatusToString(nav_status));
    }
    append_double("sog_kn", DecodeTenths(parsed.bits, 50, 10, 1023), 1);
    append_double("lon", DecodeLonDeg(parsed.bits, 61, 28), 5);
    append_double("lat", DecodeLatDeg(parsed.bits, 89, 27), 5);
    append_double("cog_deg", DecodeTenths(parsed.bits, 116, 12, 3600), 1);
    append_int("hdg_deg", DecodeUnsignedMaybe(parsed.bits, 128, 9, 511));
  } else if (message_type == 5) {
    uint32_t imo = 0;
    if (ReadUnsignedBits(parsed.bits, 40, 30, &imo) && imo > 0) {
      summary_parts << QString("imo=%1").arg(imo);
    }
    const QString call_sign = DecodeAisSixBitText(parsed.bits, 70, 42);
    if (!call_sign.isEmpty()) {
      summary_parts << QString("callsign=%1").arg(call_sign);
    }
    const QString vessel_name = DecodeAisSixBitText(parsed.bits, 112, 120);
    if (!vessel_name.isEmpty()) {
      summary_parts << QString("name=%1").arg(vessel_name);
    }
    uint32_t ship_type = 0;
    if (ReadUnsignedBits(parsed.bits, 232, 8, &ship_type) && ship_type > 0) {
      summary_parts << QString("ship_type=%1").arg(ship_type);
    }
    uint32_t to_bow = 0;
    uint32_t to_stern = 0;
    uint32_t to_port = 0;
    uint32_t to_starboard = 0;
    if (ReadUnsignedBits(parsed.bits, 240, 9, &to_bow) &&
        ReadUnsignedBits(parsed.bits, 249, 9, &to_stern) &&
        ReadUnsignedBits(parsed.bits, 258, 6, &to_port) &&
        ReadUnsignedBits(parsed.bits, 264, 6, &to_starboard)) {
      summary_parts << QString("dim_m=%1/%2/%3/%4").arg(to_bow).arg(to_stern).arg(to_port).arg(to_starboard);
    }
    const QString destination = DecodeAisSixBitText(parsed.bits, 302, 120);
    if (!destination.isEmpty()) {
      summary_parts << QString("dest=%1").arg(destination);
    }
  } else if (message_type == 18) {
    append_double("sog_kn", DecodeTenths(parsed.bits, 46, 10, 1023), 1);
    append_double("lon", DecodeLonDeg(parsed.bits, 57, 28), 5);
    append_double("lat", DecodeLatDeg(parsed.bits, 85, 27), 5);
    append_double("cog_deg", DecodeTenths(parsed.bits, 112, 12, 3600), 1);
    append_int("hdg_deg", DecodeUnsignedMaybe(parsed.bits, 124, 9, 511));
  } else if (message_type == 19) {
    append_double("sog_kn", DecodeTenths(parsed.bits, 46, 10, 1023), 1);
    append_double("lon", DecodeLonDeg(parsed.bits, 57, 28), 5);
    append_double("lat", DecodeLatDeg(parsed.bits, 85, 27), 5);
    append_double("cog_deg", DecodeTenths(parsed.bits, 112, 12, 3600), 1);
    append_int("hdg_deg", DecodeUnsignedMaybe(parsed.bits, 124, 9, 511));
    const QString vessel_name = DecodeAisSixBitText(parsed.bits, 143, 120);
    if (!vessel_name.isEmpty()) {
      summary_parts << QString("name=%1").arg(vessel_name);
    }
    uint32_t ship_type = 0;
    if (ReadUnsignedBits(parsed.bits, 263, 8, &ship_type) && ship_type > 0) {
      summary_parts << QString("ship_type=%1").arg(ship_type);
    }
  } else if (message_type == 24) {
    uint32_t part_no = 0;
    if (ReadUnsignedBits(parsed.bits, 38, 2, &part_no)) {
      summary_parts << QString("part=%1").arg(part_no);
      if (part_no == 0) {
        const QString vessel_name = DecodeAisSixBitText(parsed.bits, 40, 120);
        if (!vessel_name.isEmpty()) {
          summary_parts << QString("name=%1").arg(vessel_name);
        }
      } else if (part_no == 1) {
        uint32_t ship_type = 0;
        if (ReadUnsignedBits(parsed.bits, 40, 8, &ship_type) && ship_type > 0) {
          summary_parts << QString("ship_type=%1").arg(ship_type);
        }
        const QString vendor = DecodeAisSixBitText(parsed.bits, 48, 42);
        if (!vendor.isEmpty()) {
          summary_parts << QString("vendor=%1").arg(vendor);
        }
        const QString call_sign = DecodeAisSixBitText(parsed.bits, 90, 42);
        if (!call_sign.isEmpty()) {
          summary_parts << QString("callsign=%1").arg(call_sign);
        }
      }
    }
  }

  return summary_parts.join(" ");
}

QString BuildDscDecodedSummary(const QVariantMap& fields) {
  auto field_text = [&fields](const QString& key) -> QString {
    if (!fields.contains(key)) {
      return {};
    }
    return fields.value(key).toString();
  };

  const QString kind = field_text("kind");
  if (kind.isEmpty() || kind == "candidate") {
    return {};
  }

  QStringList summary_parts;
  const QString format = field_text("format_label");
  const QString category = field_text("category_label");
  const QString address = field_text("address_digits");
  const QString self_id = field_text("self_id_digits");
  const QString tc1 = field_text("telecommand_1");
  const QString tc2 = field_text("telecommand_2");
  const QString eos = field_text("eos_label");
  const QString validity = field_text("validity");

  if (!format.isEmpty()) {
    summary_parts << QString("fmt=%1").arg(format);
  }
  if (!category.isEmpty()) {
    summary_parts << QString("cat=%1").arg(category);
  }
  if (!address.isEmpty()) {
    summary_parts << QString("to=%1").arg(address);
  }
  if (!self_id.isEmpty()) {
    summary_parts << QString("from=%1").arg(self_id);
  }
  if (!tc1.isEmpty()) {
    summary_parts << QString("tc1=%1").arg(tc1);
  }
  if (!tc2.isEmpty()) {
    summary_parts << QString("tc2=%1").arg(tc2);
  }
  if (!eos.isEmpty()) {
    summary_parts << QString("eos=%1").arg(eos);
  }
  if (!validity.isEmpty()) {
    summary_parts << QString("validity=%1").arg(validity);
  }

  return summary_parts.join(" ");
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

  fixed_frequency_edit_ = new QLineEdit("162025000", control_group);
  range_start_edit_ = new QLineEdit("156000000", control_group);
  range_end_edit_ = new QLineEdit("163000000", control_group);
  range_step_edit_ = new QLineEdit("25000", control_group);

  dwell_ms_spin_ = new QSpinBox(control_group);
  dwell_ms_spin_->setRange(100, 10000);
  dwell_ms_spin_->setValue(500);
  sample_rate_spin_ = new QSpinBox(control_group);
  sample_rate_spin_->setRange(225000, 3200000);
  sample_rate_spin_->setSingleStep(1000);
  sample_rate_spin_->setValue(2048000);
  sample_rate_spin_->setSuffix(" Hz");
  channel_bandwidth_spin_ = new QSpinBox(control_group);
  channel_bandwidth_spin_->setRange(0, 500000);
  channel_bandwidth_spin_->setSingleStep(1000);
  channel_bandwidth_spin_->setValue(30000);
  channel_bandwidth_spin_->setSuffix(" Hz");
  channel_bandwidth_spin_->setSpecialValueText("Off");
  hardware_bandwidth_spin_ = new QSpinBox(control_group);
  hardware_bandwidth_spin_->setRange(0, 3200000);
  hardware_bandwidth_spin_->setSingleStep(1000);
  hardware_bandwidth_spin_->setValue(0);
  hardware_bandwidth_spin_->setSuffix(" Hz");
  hardware_bandwidth_spin_->setSpecialValueText("Auto");
  dc_blocker_checkbox_ = new QCheckBox("Enabled", control_group);
  dc_blocker_checkbox_->setChecked(false);
  dc_blocker_cutoff_spin_ = new QSpinBox(control_group);
  dc_blocker_cutoff_spin_->setRange(1, 5000);
  dc_blocker_cutoff_spin_->setSingleStep(10);
  dc_blocker_cutoff_spin_->setValue(30);
  dc_blocker_cutoff_spin_->setSuffix(" Hz");
  dc_blocker_cutoff_spin_->setEnabled(false);
  center_notch_checkbox_ = new QCheckBox("Enabled", control_group);
  center_notch_checkbox_->setChecked(false);
  center_notch_width_spin_ = new QSpinBox(control_group);
  center_notch_width_spin_->setRange(100, 200000);
  center_notch_width_spin_->setSingleStep(100);
  center_notch_width_spin_->setValue(2000);
  center_notch_width_spin_->setSuffix(" Hz");
  center_notch_width_spin_->setEnabled(false);
  lo_offset_checkbox_ = new QCheckBox("Enabled", control_group);
  lo_offset_checkbox_->setChecked(false);
  lo_offset_spin_ = new QSpinBox(control_group);
  lo_offset_spin_->setRange(-500000, 500000);
  lo_offset_spin_->setSingleStep(100);
  lo_offset_spin_->setValue(0);
  lo_offset_spin_->setSuffix(" Hz");
  lo_offset_spin_->setEnabled(false);

  mode_tabs_ = new QTabWidget(control_group);
  mode_tabs_->setUsesScrollButtons(false);

  auto* fixed_tab = new QWidget(mode_tabs_);
  auto* fixed_layout = new QFormLayout(fixed_tab);
  fixed_layout->addRow("Fixed Hz", fixed_frequency_edit_);
  mode_tabs_->addTab(fixed_tab, "FIXED");

  auto* range_tab = new QWidget(mode_tabs_);
  auto* range_layout = new QFormLayout(range_tab);
  range_layout->addRow("Range Start Hz", range_start_edit_);
  range_layout->addRow("Range End Hz", range_end_edit_);
  range_layout->addRow("Range Step Hz", range_step_edit_);
  mode_tabs_->addTab(range_tab, "SCAN_RANGE");

  auto* list_tab = new QWidget(mode_tabs_);
  auto* list_layout = new QVBoxLayout(list_tab);
  auto* list_caption = new QLabel(
      "Klicka pa en kanalruta for att konfigurera label, frekvens, modulation, bandbredd, squelch och dwell.",
      list_tab);
  list_caption->setWordWrap(true);
  list_layout->addWidget(list_caption);
  scan_list_monitor_checkbox_ = new QCheckBox("Monitor mode (hold scan hopping, treat all channels as open)",
                                              list_tab);
  scan_list_monitor_checkbox_->setChecked(false);
  list_layout->addWidget(scan_list_monitor_checkbox_);
  auto* default_squelch_row = new QHBoxLayout();
  auto* default_squelch_label = new QLabel("Default squelch", list_tab);
  scan_list_default_squelch_spin_ = new QDoubleSpinBox(list_tab);
  scan_list_default_squelch_spin_->setDecimals(1);
  scan_list_default_squelch_spin_->setRange(-120.0, 0.0);
  scan_list_default_squelch_spin_->setSingleStep(1.0);
  scan_list_default_squelch_spin_->setSuffix(" dB");
  scan_list_default_squelch_spin_->setValue(kDefaultScanListSquelchDb);
  default_squelch_row->addWidget(default_squelch_label);
  default_squelch_row->addWidget(scan_list_default_squelch_spin_);
  default_squelch_row->addStretch(1);
  list_layout->addLayout(default_squelch_row);

  auto* list_actions = new QHBoxLayout();
  auto* add_channel_button = new QPushButton("Add channel", list_tab);
  auto* import_csv_button = new QPushButton("Import CSV", list_tab);
  auto* clear_channels_button = new QPushButton("Clear channels", list_tab);
  list_actions->addWidget(add_channel_button);
  list_actions->addWidget(import_csv_button);
  list_actions->addWidget(clear_channels_button);
  list_actions->addStretch(1);
  list_layout->addLayout(list_actions);

  scan_list_grid_widget_ = new QWidget(list_tab);
  scan_list_grid_layout_ = new QGridLayout(scan_list_grid_widget_);
  scan_list_grid_layout_->setHorizontalSpacing(8);
  scan_list_grid_layout_->setVerticalSpacing(8);
  scan_list_scroll_area_ = new QScrollArea(list_tab);
  scan_list_scroll_area_->setWidgetResizable(true);
  scan_list_scroll_area_->setWidget(scan_list_grid_widget_);
  scan_list_scroll_area_->viewport()->installEventFilter(this);
  list_layout->addWidget(scan_list_scroll_area_);

  connect(add_channel_button, &QPushButton::clicked, this, &MainWindow::AddScanListChannel);
  connect(import_csv_button, &QPushButton::clicked, this, &MainWindow::ImportScanListCsv);
  connect(scan_list_monitor_checkbox_, &QCheckBox::toggled, this, [this](bool /*enabled*/) {
    SaveScanListConfigToSettings();
    if (receiver_combo_->currentIndex() >= 0) {
      ApplyModeAndConfig();
    }
  });
  connect(scan_list_default_squelch_spin_,
          static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
          [this](double value) {
            SaveScanListConfigToSettings();
            RefreshScanListChannelCards();
            if (receiver_combo_->currentIndex() >= 0) {
              const uint32_t receiver_id = static_cast<uint32_t>(receiver_combo_->currentData().toUInt());
              bool use_default_for_visual = true;
              if (active_scan_list_channel_index_ >= 0 &&
                  static_cast<size_t>(active_scan_list_channel_index_) < scan_list_channels_.size()) {
                use_default_for_visual =
                    scan_list_channels_[static_cast<size_t>(active_scan_list_channel_index_)]
                        .use_default_squelch;
              }
              if (use_default_for_visual) {
                signal_visualization_->SetReceiverSquelchThresholdDb(receiver_id, value);
              }
            }
          });
  connect(scan_list_default_squelch_spin_, &QDoubleSpinBox::editingFinished, this, [this]() {
    if (receiver_combo_->currentIndex() >= 0) {
      ApplyModeAndConfig();
    }
  });
  connect(clear_channels_button, &QPushButton::clicked, this, [this]() {
    if (QMessageBox::question(this, "Clear channels",
                              "Remove all scan-list channels?") != QMessageBox::Yes) {
      return;
    }
    scan_list_channels_.clear();
    active_scan_list_channel_index_ = -1;
    active_scan_list_channel_state_ = ScanListChannelState::kIdle;
    SaveScanListConfigToSettings();
    RefreshScanListChannelCards();
    if (receiver_combo_->currentIndex() >= 0) {
      ApplyModeAndConfig();
    }
  });
  mode_tabs_->addTab(list_tab, "SCAN_LIST");

  auto* air_marine_tab = new QWidget(mode_tabs_);
  auto* air_marine_layout = new QFormLayout(air_marine_tab);
  signal_filter_combo_ = new QComboBox(air_marine_tab);
  signal_filter_combo_->addItem("ALL");
  signal_filter_combo_->addItem("SIGNAL_TYPE_AIS");
  signal_filter_combo_->addItem("SIGNAL_TYPE_ADSB");
  signal_filter_combo_->addItem("SIGNAL_TYPE_DSC");
  receiver_filter_combo_ = new QComboBox(air_marine_tab);
  receiver_filter_combo_->addItem("ALL", QVariant::fromValue(-1));
  minutes_filter_spin_ = new QSpinBox(air_marine_tab);
  minutes_filter_spin_->setRange(1, 240);
  minutes_filter_spin_->setValue(30);
  air_marine_layout->addRow(new QLabel("Uses built-in AIS + DSC channels.", air_marine_tab));
  air_marine_layout->addRow(new QLabel("AIS: 162000000 Hz, DSC Ch 70: 156525000 Hz", air_marine_tab));
  air_marine_layout->addRow("AIS+DSC bandbredd", channel_bandwidth_spin_);
  air_marine_layout->addRow("Signal", signal_filter_combo_);
  air_marine_layout->addRow("Receiver", receiver_filter_combo_);
  air_marine_layout->addRow("Last minutes", minutes_filter_spin_);
  decoded_table_ = new QTableWidget(0, 6, air_marine_tab);
  decoded_table_->setHorizontalHeaderLabels({"Time", "Receiver", "Signal", "Frequency", "Payload", "Decoded"});
  decoded_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  decoded_table_->setMinimumHeight(280);
  air_marine_layout->addRow(decoded_table_);
  mode_tabs_->addTab(air_marine_tab, "AIR_MARINE_PLOT");

  auto* global_tab = new QWidget(mode_tabs_);
  auto* global_layout = new QFormLayout(global_tab);
  global_layout->addRow("Default dwell ms", dwell_ms_spin_);
  global_layout->addRow("Sample rate", sample_rate_spin_);
  global_layout->addRow("Hardware bandwidth", hardware_bandwidth_spin_);
  global_layout->addRow("DC blocker", dc_blocker_checkbox_);
  global_layout->addRow("DC cutoff", dc_blocker_cutoff_spin_);
  global_layout->addRow("Center notch", center_notch_checkbox_);
  global_layout->addRow("Notch width", center_notch_width_spin_);
  global_layout->addRow("LO offset", lo_offset_checkbox_);
  global_layout->addRow("LO offset Hz", lo_offset_spin_);
  spectrum_source_combo_ = new QComboBox(global_tab);
  spectrum_source_combo_->addItem("Demodulated", QVariant::fromValue(0));
  spectrum_source_combo_->addItem("Receiver spectrum", QVariant::fromValue(1));
  auto* visualization_settings_button = new QPushButton("Visualization settings...", global_tab);
  global_layout->addRow("Spectrum view", spectrum_source_combo_);
  global_layout->addRow(visualization_settings_button);
  mode_tabs_->addTab(global_tab, "GLOBAL");
  mode_tabs_->setCurrentIndex(kFixedModeTabIndex);
  last_tab_index_ = mode_tabs_->currentIndex();

  auto* button_row = new QWidget(control_group);
  auto* button_layout = new QHBoxLayout(button_row);
  button_layout->setContentsMargins(0, 0, 0, 0);
  auto* refresh_button = new QPushButton("Refresh", button_row);
  auto* start_button = new QPushButton("Start", button_row);
  auto* stop_button = new QPushButton("Stop", button_row);
  auto* apply_button = new QPushButton("Apply mode/radio settings", button_row);
  button_layout->addWidget(refresh_button);
  button_layout->addWidget(start_button);
  button_layout->addWidget(stop_button);
  button_layout->addWidget(apply_button);

  control_layout->addRow("Receiver", receiver_combo_);
  control_layout->addRow("Mode settings", mode_tabs_);
  control_layout->addRow(button_row);

  top_layout->addWidget(control_group, 1);

  signal_visualization_ = new SignalVisualizationWidget(central);

  event_log_ = new QPlainTextEdit(central);
  event_log_->setReadOnly(true);

  root_layout->addLayout(top_layout);
  root_layout->addWidget(signal_visualization_);
  root_layout->addWidget(event_log_);

  setCentralWidget(central);
  LoadScanListConfigFromSettings();
  RefreshScanListChannelCards();

  audio_output_disabled_ = EnvFlagEnabled("MR_DISABLE_AUDIO_OUTPUT");
  if (audio_output_disabled_) {
    AppendLog("Audio output disabled by MR_DISABLE_AUDIO_OUTPUT");
  } else if (IsWslEnvironment() && !EnvFlagEnabled("MR_ENABLE_AUDIO_OUTPUT")) {
    audio_output_disabled_ = true;
    AppendLog("Audio output auto-disabled on WSL (set MR_ENABLE_AUDIO_OUTPUT=1 to override)");
  }

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
  connect(spectrum_source_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
    const int selected = spectrum_source_combo_->currentData().toInt();
    const auto source =
        (selected == 1) ? SignalVisualizationWidget::SpectrumSource::kReceiverInput
                        : SignalVisualizationWidget::SpectrumSource::kDemodulated;
    signal_visualization_->SetSpectrumSource(source);
    AppendLog(QString("Visualization spectrum source: %1")
                  .arg(source == SignalVisualizationWidget::SpectrumSource::kReceiverInput
                           ? "receiver"
                           : "demodulated"));
  });
  connect(minutes_filter_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this]() {
    decoded_table_->setRowCount(0);
    for (const auto& row : all_rows_) {
      AddMessageRow(row);
    }
  });
  connect(dwell_ms_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this]() {
    SaveScanListConfigToSettings();
  });
  connect(receiver_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
    active_scan_list_channel_index_ = -1;
    active_scan_list_channel_state_ = ScanListChannelState::kIdle;
    if (receiver_combo_->currentIndex() >= 0 && scan_list_default_squelch_spin_ != nullptr) {
      const uint32_t receiver_id = static_cast<uint32_t>(receiver_combo_->currentData().toUInt());
      signal_visualization_->SetReceiverSquelchThresholdDb(receiver_id,
                                                           scan_list_default_squelch_spin_->value());
    }
    RefreshScanListChannelCards();
  });
  connect(mode_tabs_, &QTabWidget::currentChanged, this, [this](int index) {
    const int previous_tab_index = last_tab_index_;
    last_tab_index_ = index;
    if (index >= kFixedModeTabIndex && index <= kAirMarineModeTabIndex) {
      last_mode_tab_index_ = index;
    }
    if (previous_tab_index < 0 || previous_tab_index == index) {
      return;
    }
    if (receiver_combo_->currentIndex() < 0) {
      return;
    }

    const uint32_t receiver_id = static_cast<uint32_t>(receiver_combo_->currentData().toUInt());
    std::string error;
    if (!client_->StopReceiver(receiver_id, &error)) {
      QMessageBox::warning(this, "StopReceiver failed", QString::fromStdString(error));
      return;
    }
    AppendLog(QString("Tab switch stop requested for receiver %1").arg(receiver_id));
    RefreshReceivers();
    ApplyModeAndConfig();
  });
  connect(dc_blocker_checkbox_, &QCheckBox::toggled, this, [this](bool enabled) {
    dc_blocker_cutoff_spin_->setEnabled(enabled);
  });
  connect(center_notch_checkbox_, &QCheckBox::toggled, this, [this](bool enabled) {
    center_notch_width_spin_->setEnabled(enabled);
  });
  connect(lo_offset_checkbox_, &QCheckBox::toggled, this, [this](bool enabled) {
    lo_offset_spin_->setEnabled(enabled);
  });

  connect(client_.get(), &GrpcClient::ReceiverEventReceived, this, &MainWindow::OnReceiverEvent,
          Qt::QueuedConnection);
  connect(client_.get(), &GrpcClient::DecodedMessageReceived, this, &MainWindow::OnDecodedMessage,
          Qt::QueuedConnection);
  connect(client_.get(), &GrpcClient::StreamError, this, &MainWindow::OnStreamError,
          Qt::QueuedConnection);

  QTimer::singleShot(0, this, [this]() {
    RefreshReceivers();
    client_->StartStreaming();
  });
}

MainWindow::~MainWindow() { client_->StopStreaming(); }

bool MainWindow::eventFilter(QObject* watched, QEvent* event) {
  if (scan_list_scroll_area_ != nullptr && watched == scan_list_scroll_area_->viewport() &&
      event != nullptr && event->type() == QEvent::Resize) {
    RefreshScanListChannelCards();
  }
  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::RefreshReceivers() {
  std::vector<v1::ReceiverInfo> receivers;
  std::string error;
  if (!client_->ListReceivers(&receivers, &error)) {
    QMessageBox::warning(this, "ListReceivers failed", QString::fromStdString(error));
    return;
  }

  const uint32_t previous_receiver_id =
      (receiver_combo_->currentIndex() >= 0) ? static_cast<uint32_t>(receiver_combo_->currentData().toUInt()) : 0;
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

  if (previous_receiver_id != 0) {
    const int restore_index = receiver_combo_->findData(QVariant::fromValue<int>(previous_receiver_id));
    if (restore_index >= 0) {
      receiver_combo_->setCurrentIndex(restore_index);
    }
  }

  if (receiver_combo_->currentIndex() >= 0) {
    const uint32_t selected_id = static_cast<uint32_t>(receiver_combo_->currentData().toUInt());
    for (const auto& receiver : receivers) {
      if (receiver.receiver_id() != selected_id) {
        continue;
      }
      const double receiver_default_squelch_db =
          std::clamp(receiver.mode_config().scan_list_default_squelch_db(), -120.0, 0.0);
      if (scan_list_default_squelch_spin_ != nullptr) {
        const QSignalBlocker blocker(scan_list_default_squelch_spin_);
        scan_list_default_squelch_spin_->setValue(receiver_default_squelch_db);
      }
      const auto& channels = receiver.mode_config().scan_list_channels();
      if (!channels.empty()) {
        std::vector<ScanListChannelConfig> updated_channels;
        updated_channels.reserve(static_cast<size_t>(channels.size()));
        for (int index = 0; index < channels.size(); ++index) {
          const auto& channel = channels.Get(index);
          ScanListChannelConfig updated;
          updated.label = QString::fromStdString(channel.label());
          updated.frequency_mhz = channel.frequency_hz() > 0.0 ? channel.frequency_hz() / 1000000.0 : 0.0;
          updated.modulation = channel.modulation();
          updated.bandwidth_hz = static_cast<int>(channel.channel_bandwidth_hz());
          updated.use_default_squelch = channel.use_default_squelch();
          updated.squelch_threshold_db = channel.squelch_threshold_db();
          if (updated.use_default_squelch) {
            updated.squelch_threshold_db = receiver_default_squelch_db;
          }
          updated.dwell_ms = static_cast<int>(channel.dwell_ms());
          updated_channels.push_back(std::move(updated));
        }
        scan_list_channels_ = std::move(updated_channels);
      }
      scan_list_monitor_checkbox_->setChecked(receiver.mode_config().scan_list_monitor_mode());
      if (scan_list_default_squelch_spin_ != nullptr) {
        signal_visualization_->SetReceiverSquelchThresholdDb(selected_id,
                                                             scan_list_default_squelch_spin_->value());
      }
      break;
    }
  }
  RefreshScanListChannelCards();

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

  int mode_tab_index = mode_tabs_->currentIndex();
  if (mode_tab_index == kGlobalSettingsTabIndex) {
    mode_tab_index = last_mode_tab_index_;
  }
  const v1::RadioMode mode = ModeFromTabIndex(mode_tab_index);

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
  config.set_sample_rate_hz(static_cast<uint32_t>(sample_rate_spin_->value()));
  config.set_channel_bandwidth_hz(static_cast<uint32_t>(channel_bandwidth_spin_->value()));
  config.set_hardware_bandwidth_hz(static_cast<uint32_t>(hardware_bandwidth_spin_->value()));
  config.set_dc_blocker_enabled(dc_blocker_checkbox_->isChecked());
  config.set_dc_blocker_cutoff_hz(static_cast<uint32_t>(dc_blocker_cutoff_spin_->value()));
  config.set_center_notch_enabled(center_notch_checkbox_->isChecked());
  config.set_center_notch_width_hz(static_cast<uint32_t>(center_notch_width_spin_->value()));
  config.set_lo_offset_enabled(lo_offset_checkbox_->isChecked());
  config.set_lo_offset_hz(static_cast<int32_t>(lo_offset_spin_->value()));
  config.set_scan_list_monitor_mode(
      scan_list_monitor_checkbox_ != nullptr && scan_list_monitor_checkbox_->isChecked());
  const double default_squelch_db =
      (scan_list_default_squelch_spin_ != nullptr) ? scan_list_default_squelch_spin_->value()
                                                   : kDefaultScanListSquelchDb;
  config.set_scan_list_default_squelch_db(default_squelch_db);

  config.clear_scan_list_channels();
  config.clear_frequency_list_hz();
  for (const auto& channel : scan_list_channels_) {
    auto* out = config.add_scan_list_channels();
    out->set_label(channel.label.toStdString());
    out->set_frequency_hz(channel.frequency_mhz * 1000000.0);
    out->set_modulation(channel.modulation);
    out->set_channel_bandwidth_hz(static_cast<uint32_t>(std::max(0, channel.bandwidth_hz)));
    out->set_use_default_squelch(channel.use_default_squelch);
    out->set_squelch_threshold_db(channel.use_default_squelch ? default_squelch_db
                                                               : channel.squelch_threshold_db);
    out->set_dwell_ms(static_cast<uint32_t>(std::max(0, channel.dwell_ms)));
    if (channel.frequency_mhz > 0.0) {
      config.add_frequency_list_hz(channel.frequency_mhz * 1000000.0);
    }
  }

  if (!client_->SetModeConfig(receiver_id, config, &error)) {
    QMessageBox::warning(this, "SetModeConfig failed", QString::fromStdString(error));
    return;
  }

  AppendLog(QString("Applied mode/config to receiver %1 (sample-rate=%2 Hz, channel-bw=%3 Hz, hw-bw=%4 Hz, dc=%5@%6 Hz, notch=%7@%8 Hz, lo-offset=%9@%10 Hz)")
                .arg(receiver_id)
                .arg(sample_rate_spin_->value())
                .arg(channel_bandwidth_spin_->value())
                .arg(hardware_bandwidth_spin_->value())
                .arg(dc_blocker_checkbox_->isChecked() ? "on" : "off")
                .arg(dc_blocker_cutoff_spin_->value())
                .arg(center_notch_checkbox_->isChecked() ? "on" : "off")
                .arg(center_notch_width_spin_->value())
                .arg(lo_offset_checkbox_->isChecked() ? "on" : "off")
                .arg(lo_offset_spin_->value()));
}

void MainWindow::OnReceiverEvent(uint32_t receiver_id, int event_kind, double tuned_frequency_hz,
                                 const QString& message, quint64 unix_ms) {
  if (message.startsWith("AUDIO_PCM16 ")) {
    if (IsSelectedReceiver(receiver_id)) {
      HandleAudioPcmEvent(message);
    }
    return;
  }
  if (message.startsWith("SCAN_STATUS ")) {
    ApplyScanListStatusEvent(receiver_id, message);
    return;
  }
  if (event_kind == static_cast<int>(v1::EVENT_KIND_TUNE_HOP)) {
    return;
  }

  double peak_hz = 0.0;
  double peak_strength = 0.0;
  double frame_frequency_start_hz = 0.0;
  double frame_frequency_end_hz = 20000.0;
  std::vector<double> waveform;
  std::vector<double> spectrum;
  SignalVisualizationWidget::SpectrumSource source =
      SignalVisualizationWidget::SpectrumSource::kDemodulated;
  if (ParseVisualizationFrameEvent(message, &peak_hz, &peak_strength, &waveform, &spectrum, &source,
                                   &frame_frequency_start_hz, &frame_frequency_end_hz)) {
    signal_visualization_->PushVisualizationFrame(receiver_id, waveform, spectrum, peak_hz, peak_strength,
                                                  source, frame_frequency_start_hz,
                                                  frame_frequency_end_hz);
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
  if (signal_type == "SIGNAL_TYPE_AIS") {
    row.decoded_summary = BuildAisDecodedSummary(payload, fields);
  } else if (signal_type == "SIGNAL_TYPE_DSC") {
    row.decoded_summary = BuildDscDecodedSummary(fields);
  }

  auto field_text = [&fields](const QString& key) -> QString {
    if (!fields.contains(key)) {
      return {};
    }
    return fields.value(key).toString();
  };

  const QString kind = field_text("kind");
  if (signal_type == "SIGNAL_TYPE_AIS" && kind == "metric") {
    const QString channel = field_text("channel");
    auto parse_u64 = [&field_text](const QString& key, quint64 fallback = 0) -> quint64 {
      bool ok = false;
      const quint64 value = field_text(key).toULongLong(&ok);
      return ok ? value : fallback;
    };
    const QString channel_key = channel.isEmpty() ? QString("AIS?") : channel;
    const quint64 crc_ok_total =
        parse_u64("metric_crc_ok_channel", parse_u64("metric_crc_ok", 0));
    const quint64 crc_fail_total =
        parse_u64("metric_crc_fail_channel", parse_u64("metric_crc_fail", 0));
    const quint64 emitted_total = parse_u64("metric_emitted", 0);
    const quint64 interval_ms = 30000;

    AisCrcSummaryState& summary = ais_crc_summary_by_channel_[channel_key];
    if (summary.last_log_unix_ms == 0 || unix_ms <= summary.last_log_unix_ms ||
        (unix_ms - summary.last_log_unix_ms) >= interval_ms) {
      const quint64 ok_delta =
          (crc_ok_total >= summary.last_crc_ok) ? (crc_ok_total - summary.last_crc_ok) : crc_ok_total;
      const quint64 fail_delta = (crc_fail_total >= summary.last_crc_fail)
                                     ? (crc_fail_total - summary.last_crc_fail)
                                     : crc_fail_total;
      const quint64 emitted_delta = (emitted_total >= summary.last_emitted)
                                        ? (emitted_total - summary.last_emitted)
                                        : emitted_total;
      const double ratio = (crc_ok_total + crc_fail_total) == 0
                               ? 0.0
                               : static_cast<double>(crc_ok_total) /
                                     static_cast<double>(crc_ok_total + crc_fail_total);

      AppendLog(QString("[%1] RX%2 AIS CRC summary ch=%3 path=%4 ok=%5 (+%6) fail=%7 (+%8) emitted=%9 (+%10) ok_ratio=%11")
                    .arg(ToLocalTime(unix_ms))
                    .arg(receiver_id)
                    .arg(channel_key)
                    .arg(field_text("metric_decode_path"))
                    .arg(crc_ok_total)
                    .arg(ok_delta)
                    .arg(crc_fail_total)
                    .arg(fail_delta)
                    .arg(emitted_total)
                    .arg(emitted_delta)
                    .arg(ratio, 0, 'f', 4));

      summary.last_log_unix_ms = unix_ms;
      summary.last_crc_ok = crc_ok_total;
      summary.last_crc_fail = crc_fail_total;
      summary.last_emitted = emitted_total;
    }
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
  if (!row.decoded_summary.isEmpty()) {
    summary += QString(" decoded=%1").arg(row.decoded_summary);
  }
  summary += QString(" payload=%1").arg(payload.left(96));
  AppendLog(summary);

  const QString metric_blocks = field_text("metric_blocks");
  if (!metric_blocks.isEmpty() && signal_type == "SIGNAL_TYPE_AIS") {
    AppendLog(QString("AIS metrics: blocks=%1 flags=%2 candidates=%3 crc_ok=%4 crc_fail=%5 dup=%6 emitted=%7 demod_ready=%8")
                  .arg(metric_blocks)
                  .arg(field_text("metric_flags"))
                  .arg(field_text("metric_candidates"))
                  .arg(field_text("metric_crc_ok"))
                  .arg(field_text("metric_crc_fail"))
                  .arg(field_text("metric_duplicates"))
                  .arg(field_text("metric_emitted"))
                  .arg(field_text("metric_demod_ready")));
  } else if (!metric_blocks.isEmpty() && signal_type == "SIGNAL_TYPE_DSC") {
    AppendLog(QString("DSC metrics: blocks=%1 candidates=%2 dup=%3 emitted=%4 frames=%5 ecc_ok=%6 ecc_fail=%7")
                  .arg(metric_blocks)
                  .arg(field_text("metric_candidates"))
                  .arg(field_text("metric_duplicates"))
                  .arg(field_text("metric_emitted"))
                  .arg(field_text("metric_frames_parsed"))
                  .arg(field_text("metric_ecc_ok"))
                  .arg(field_text("metric_ecc_fail")));
  }

  all_rows_.push_back(row);
  AddMessageRow(row);
}

bool MainWindow::IsSelectedReceiver(uint32_t receiver_id) const {
  if (receiver_combo_->currentIndex() < 0) {
    return false;
  }
  return static_cast<uint32_t>(receiver_combo_->currentData().toUInt()) == receiver_id;
}

QString MainWindow::ScanListChannelCardText(int index) const {
  if (index < 0 || static_cast<size_t>(index) >= scan_list_channels_.size()) {
    return {};
  }
  const auto& channel = scan_list_channels_[static_cast<size_t>(index)];
  const QString label = channel.label.trimmed().isEmpty()
                            ? QString("Kanal %1").arg(index + 1)
                            : channel.label.trimmed();
  const bool configured = channel.frequency_mhz > 0.0;
  const QString freq_text =
      configured ? QString("%1 MHz").arg(channel.frequency_mhz, 0, 'f', 3)
                 : QString("Frekvens ej satt");
  const QString squelch_text =
      channel.use_default_squelch ? "SQ: Default"
                                  : QString("SQ: %1 dB").arg(channel.squelch_threshold_db, 0, 'f', 1);
  QString status_text = configured ? "Vilar" : "Okonfigurerad";
  if (active_scan_list_channel_index_ == index) {
    status_text = (active_scan_list_channel_state_ == ScanListChannelState::kSquelchOpen)
                      ? "Squelch oppen"
                      : "Squelch stangd";
  }
  return QString("%1\n%2\n%3\n%4").arg(label).arg(freq_text).arg(squelch_text).arg(status_text);
}

QString MainWindow::ScanListChannelCardStyle(int index) const {
  if (active_scan_list_channel_index_ == index &&
      active_scan_list_channel_state_ == ScanListChannelState::kSquelchOpen) {
    return "QPushButton { text-align: left; padding: 10px; border: 2px solid #2E7D32; background: #E8F5E9; }";
  }
  if (active_scan_list_channel_index_ == index &&
      active_scan_list_channel_state_ == ScanListChannelState::kSquelchClosed) {
    return "QPushButton { text-align: left; padding: 10px; border: 2px solid #EF6C00; background: #FFF3E0; }";
  }
  return "QPushButton { text-align: left; padding: 10px; border: 1px solid #B0BEC5; background: #FAFAFA; }";
}

void MainWindow::RefreshScanListChannelCards() {
  if (scan_list_grid_layout_ == nullptr || scan_list_grid_widget_ == nullptr) {
    return;
  }

  while (scan_list_channel_buttons_.size() > scan_list_channels_.size()) {
    QPushButton* button = scan_list_channel_buttons_.back();
    scan_list_channel_buttons_.pop_back();
    scan_list_grid_layout_->removeWidget(button);
    button->deleteLater();
  }

  while (scan_list_channel_buttons_.size() < scan_list_channels_.size()) {
    auto* channel_button = new QPushButton(scan_list_grid_widget_);
    channel_button->setMinimumHeight(84);
    channel_button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    connect(channel_button, &QPushButton::clicked, this, [this, channel_button]() {
      const auto it = std::find(scan_list_channel_buttons_.begin(), scan_list_channel_buttons_.end(),
                                channel_button);
      if (it == scan_list_channel_buttons_.end()) {
        return;
      }
      const int index = static_cast<int>(std::distance(scan_list_channel_buttons_.begin(), it));
      ConfigureScanListChannel(index);
    });
    scan_list_channel_buttons_.push_back(channel_button);
  }

  int max_text_width = 0;
  for (size_t idx = 0; idx < scan_list_channel_buttons_.size(); ++idx) {
    QPushButton* button = scan_list_channel_buttons_[idx];
    if (button == nullptr) {
      continue;
    }
    const int index = static_cast<int>(idx);
    const QString text = ScanListChannelCardText(index);
    button->setText(text);
    button->setStyleSheet(ScanListChannelCardStyle(index));
    const QFontMetrics metrics(button->font());
    const QStringList lines = text.split('\n');
    int button_text_width = 0;
    for (const QString& line : lines) {
      button_text_width = std::max(button_text_width, metrics.horizontalAdvance(line));
    }
    max_text_width = std::max(max_text_width, button_text_width);
  }

  const int button_width = std::max(220, max_text_width + 38);
  int columns = 1;
  if (scan_list_scroll_area_ != nullptr && scan_list_scroll_area_->viewport() != nullptr) {
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
    scan_list_grid_layout_->getContentsMargins(&left, &top, &right, &bottom);
    int spacing = scan_list_grid_layout_->horizontalSpacing();
    if (spacing < 0) {
      spacing = 8;
    }
    const int available_width =
        std::max(0, scan_list_scroll_area_->viewport()->width() - left - right);
    columns = std::max(1, (available_width + spacing) / (button_width + spacing));
  }

  for (size_t idx = 0; idx < scan_list_channel_buttons_.size(); ++idx) {
    QPushButton* button = scan_list_channel_buttons_[idx];
    if (button == nullptr) {
      continue;
    }
    button->setFixedWidth(button_width);
    const int index = static_cast<int>(idx);
    scan_list_grid_layout_->addWidget(button, index / columns, index % columns,
                                      Qt::AlignLeft | Qt::AlignTop);
  }
}

void MainWindow::ConfigureScanListChannel(int index) {
  if (index < 0 || static_cast<size_t>(index) >= scan_list_channels_.size()) {
    return;
  }
  ScanListChannelConfig channel = scan_list_channels_[static_cast<size_t>(index)];

  QDialog dialog(this);
  dialog.setWindowTitle(QString("Konfigurera kanal %1").arg(index + 1));
  auto* layout = new QFormLayout(&dialog);

  auto* label_edit = new QLineEdit(channel.label, &dialog);
  auto* frequency_spin = new QDoubleSpinBox(&dialog);
  frequency_spin->setDecimals(3);
  frequency_spin->setRange(0.0, 6000.0);
  frequency_spin->setSingleStep(0.025);
  frequency_spin->setSuffix(" MHz");
  frequency_spin->setValue(channel.frequency_mhz);

  auto* modulation_combo = new QComboBox(&dialog);
  modulation_combo->addItem("AM");
  modulation_combo->addItem("WFM");
  modulation_combo->addItem("NFM");
  modulation_combo->setCurrentText(ModulationLabel(channel.modulation));

  auto* bandwidth_spin = new QSpinBox(&dialog);
  bandwidth_spin->setRange(2000, 500000);
  bandwidth_spin->setSingleStep(1000);
  bandwidth_spin->setSuffix(" Hz");
  const int channel_bandwidth =
      channel.bandwidth_hz > 0 ? channel.bandwidth_hz : DefaultBandwidthHzForModulation(channel.modulation);
  bandwidth_spin->setValue(channel_bandwidth);

  auto* squelch_spin = new QDoubleSpinBox(&dialog);
  squelch_spin->setDecimals(1);
  squelch_spin->setRange(-120.0, 0.0);
  squelch_spin->setSingleStep(1.0);
  squelch_spin->setSuffix(" dB");
  squelch_spin->setValue(channel.squelch_threshold_db);
  auto* use_default_squelch_checkbox = new QCheckBox("Use default squelch", &dialog);
  use_default_squelch_checkbox->setChecked(channel.use_default_squelch);
  if (channel.use_default_squelch && scan_list_default_squelch_spin_ != nullptr) {
    squelch_spin->setValue(scan_list_default_squelch_spin_->value());
  }
  squelch_spin->setEnabled(!channel.use_default_squelch);

  auto* dwell_spin = new QSpinBox(&dialog);
  dwell_spin->setRange(0, 60000);
  dwell_spin->setSingleStep(100);
  dwell_spin->setSpecialValueText("Use default");
  dwell_spin->setSuffix(" ms");
  dwell_spin->setValue(channel.dwell_ms);

  connect(modulation_combo, &QComboBox::currentTextChanged, this, [bandwidth_spin](const QString& text) {
    bandwidth_spin->setValue(DefaultBandwidthHzForModulation(ModulationFromText(text)));
  });
  connect(use_default_squelch_checkbox, &QCheckBox::toggled, this, [this, squelch_spin](bool checked) {
    squelch_spin->setEnabled(!checked);
    if (checked && scan_list_default_squelch_spin_ != nullptr) {
      squelch_spin->setValue(scan_list_default_squelch_spin_->value());
    }
  });

  layout->addRow("Label", label_edit);
  layout->addRow("Frekvens", frequency_spin);
  layout->addRow("Modulation", modulation_combo);
  layout->addRow("Bandbredd", bandwidth_spin);
  layout->addRow(use_default_squelch_checkbox);
  layout->addRow("Squelch threshold", squelch_spin);
  layout->addRow("Kanal dwell", dwell_spin);

  auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  auto* delete_button = buttons->addButton("Delete", QDialogButtonBox::DestructiveRole);
  layout->addRow(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  connect(delete_button, &QPushButton::clicked, &dialog, [&dialog]() { dialog.done(1001); });

  const int dialog_result = dialog.exec();
  if (dialog_result == 1001) {
    RemoveScanListChannel(index);
    return;
  }
  if (dialog_result != QDialog::Accepted) {
    return;
  }

  channel.label = label_edit->text().trimmed();
  channel.frequency_mhz = frequency_spin->value();
  channel.modulation = ModulationFromText(modulation_combo->currentText());
  channel.bandwidth_hz = bandwidth_spin->value();
  channel.use_default_squelch = use_default_squelch_checkbox->isChecked();
  if (channel.use_default_squelch && scan_list_default_squelch_spin_ != nullptr) {
    channel.squelch_threshold_db = scan_list_default_squelch_spin_->value();
  } else {
    channel.squelch_threshold_db = squelch_spin->value();
  }
  channel.dwell_ms = dwell_spin->value();
  scan_list_channels_[static_cast<size_t>(index)] = channel;
  SaveScanListConfigToSettings();
  RefreshScanListChannelCards();

  if (receiver_combo_->currentIndex() >= 0) {
    ApplyModeAndConfig();
  }
}

void MainWindow::AddScanListChannel() {
  ScanListChannelConfig channel;
  channel.label = QString("Kanal %1").arg(scan_list_channels_.size() + 1);
  channel.modulation = v1::MODULATION_NFM;
  channel.bandwidth_hz = DefaultBandwidthHzForModulation(channel.modulation);
  channel.use_default_squelch = true;
  if (scan_list_default_squelch_spin_ != nullptr) {
    channel.squelch_threshold_db = scan_list_default_squelch_spin_->value();
  }
  scan_list_channels_.push_back(channel);
  SaveScanListConfigToSettings();
  RefreshScanListChannelCards();
}

void MainWindow::RemoveScanListChannel(int index) {
  if (index < 0 || static_cast<size_t>(index) >= scan_list_channels_.size()) {
    return;
  }
  scan_list_channels_.erase(scan_list_channels_.begin() + index);
  if (active_scan_list_channel_index_ == index) {
    active_scan_list_channel_index_ = -1;
    active_scan_list_channel_state_ = ScanListChannelState::kIdle;
  } else if (active_scan_list_channel_index_ > index) {
    --active_scan_list_channel_index_;
  }
  SaveScanListConfigToSettings();
  RefreshScanListChannelCards();
  if (receiver_combo_->currentIndex() >= 0) {
    ApplyModeAndConfig();
  }
}

void MainWindow::ImportScanListCsv() {
  const QString path = QFileDialog::getOpenFileName(
      this, "Import scan-list CSV", QString(), "CSV files (*.csv);;All files (*)");
  if (path.isEmpty()) {
    return;
  }

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QMessageBox::warning(this, "CSV import", QString("Could not open %1").arg(path));
    return;
  }

  QTextStream stream(&file);
  const double default_squelch_db =
      (scan_list_default_squelch_spin_ != nullptr) ? scan_list_default_squelch_spin_->value()
                                                   : kDefaultScanListSquelchDb;
  std::vector<ScanListChannelConfig> imported;
  QStringList errors;
  int line_number = 0;
  while (!stream.atEnd()) {
    const QString raw_line = stream.readLine();
    ++line_number;
    const QString line = raw_line.trimmed();
    if (line.isEmpty() || line.startsWith('#')) {
      continue;
    }

    const QStringList columns = line.split(';', Qt::KeepEmptyParts);
    if (columns.size() < 3) {
      errors.push_back(QString("Line %1: expected format MHz;modulation;label").arg(line_number));
      continue;
    }

    bool frequency_ok = false;
    const double frequency_mhz = columns[0].trimmed().toDouble(&frequency_ok);
    if (!frequency_ok || frequency_mhz <= 0.0) {
      errors.push_back(QString("Line %1: invalid frequency '%2'").arg(line_number).arg(columns[0].trimmed()));
      continue;
    }

    v1::Modulation modulation = v1::MODULATION_NFM;
    if (!TryParseCsvModulation(columns[1], &modulation)) {
      errors.push_back(QString("Line %1: invalid modulation '%2'").arg(line_number).arg(columns[1].trimmed()));
      continue;
    }

    ScanListChannelConfig channel;
    channel.frequency_mhz = frequency_mhz;
    channel.modulation = modulation;
    channel.label = columns.mid(2).join(";").trimmed();
    if (channel.label.isEmpty()) {
      channel.label = QString("CSV %1").arg(imported.size() + 1);
    }
    channel.bandwidth_hz = DefaultBandwidthHzForModulation(channel.modulation);
    channel.squelch_threshold_db = default_squelch_db;
    channel.dwell_ms = 0;
    channel.use_default_squelch = true;
    imported.push_back(std::move(channel));
  }

  if (imported.empty()) {
    const QString details = errors.isEmpty() ? "No channels found in file."
                                             : errors.mid(0, 10).join("\n");
    QMessageBox::warning(this, "CSV import", details);
    return;
  }

  QMessageBox choice(this);
  choice.setWindowTitle("CSV import");
  choice.setText(QString("Imported %1 channels from CSV.").arg(imported.size()));
  choice.setInformativeText("Do you want to replace existing channels or append to them?");
  auto* replace_button = choice.addButton("Replace", QMessageBox::AcceptRole);
  auto* append_button = choice.addButton("Append", QMessageBox::ActionRole);
  choice.addButton(QMessageBox::Cancel);
  choice.exec();
  const QAbstractButton* clicked = choice.clickedButton();
  if (clicked == nullptr || clicked == choice.button(QMessageBox::Cancel)) {
    return;
  }

  if (clicked == replace_button) {
    scan_list_channels_ = std::move(imported);
  } else if (clicked == append_button) {
    scan_list_channels_.insert(scan_list_channels_.end(), imported.begin(), imported.end());
  } else {
    return;
  }

  SaveScanListConfigToSettings();
  RefreshScanListChannelCards();
  if (receiver_combo_->currentIndex() >= 0) {
    ApplyModeAndConfig();
  }

  if (!errors.isEmpty()) {
    AppendLog(QString("CSV import warnings:\n%1").arg(errors.mid(0, 10).join("\n")));
  }
}

void MainWindow::ApplyScanListStatusEvent(uint32_t receiver_id, const QString& message) {
  bool threshold_ok = false;
  const double threshold_db = TokenValue(message, "threshold_db").toDouble(&threshold_ok);
  if (threshold_ok) {
    signal_visualization_->SetReceiverSquelchThresholdDb(receiver_id, threshold_db);
  }
  if (!IsSelectedReceiver(receiver_id)) {
    return;
  }
  bool idx_ok = false;
  const int index = TokenValue(message, "idx").toInt(&idx_ok);
  if (!idx_ok || index < 0 || static_cast<size_t>(index) >= scan_list_channels_.size()) {
    return;
  }
  const QString state = TokenValue(message, "state").trimmed().toLower();
  active_scan_list_channel_index_ = index;
  if (state == "open") {
    active_scan_list_channel_state_ = ScanListChannelState::kSquelchOpen;
  } else {
    active_scan_list_channel_state_ = ScanListChannelState::kSquelchClosed;
  }
  RefreshScanListChannelCards();
}

void MainWindow::HandleAudioPcmEvent(const QString& message) {
#if MR_HAS_QT_MULTIMEDIA
  EnsureAudioOutputInitialized();
  if (audio_output_disabled_) {
    return;
  }
  if (audio_output_device_ == nullptr) {
    return;
  }
  bool sr_ok = false;
  const int sample_rate_hz = TokenValue(message, "sr").toInt(&sr_ok);
  if (!sr_ok || sample_rate_hz <= 0) {
    return;
  }
  const QString data_b64 = TokenValue(message, "data");
  if (data_b64.isEmpty()) {
    return;
  }
  const QByteArray pcm = QByteArray::fromBase64(data_b64.toUtf8());
  if (pcm.isEmpty()) {
    return;
  }
  audio_output_device_->write(pcm);
#else
  Q_UNUSED(message);
#endif
}

void MainWindow::EnsureAudioOutputInitialized() {
#if MR_HAS_QT_MULTIMEDIA
  if (audio_output_disabled_ || audio_output_device_ != nullptr || audio_sink_ != nullptr) {
    return;
  }
  const QAudioDevice default_output = QMediaDevices::defaultAudioOutput();
  if (default_output.isNull()) {
    audio_output_disabled_ = true;
    AppendLog("Audio output unavailable (no default output device)");
    return;
  }
  QAudioFormat audio_format;
  audio_format.setSampleRate(16000);
  audio_format.setChannelCount(1);
  audio_format.setSampleFormat(QAudioFormat::Int16);
  audio_sink_ = new QAudioSink(default_output, audio_format, this);
  audio_output_device_ = audio_sink_->start();
  if (audio_output_device_ == nullptr) {
    audio_output_disabled_ = true;
    audio_sink_->deleteLater();
    audio_sink_ = nullptr;
    AppendLog("Audio output unavailable");
  }
#endif
}

void MainWindow::LoadScanListConfigFromSettings() {
  QSettings settings("multi-radio", "multi-radio-client");
  settings.beginGroup("scan_list");
  const int saved_default_dwell =
      settings.value("default_dwell_ms", dwell_ms_spin_->value()).toInt();
  if (saved_default_dwell > 0) {
    dwell_ms_spin_->setValue(saved_default_dwell);
  }
  const double default_squelch_db = std::clamp(
      settings.value("default_squelch_db", kDefaultScanListSquelchDb).toDouble(), -120.0, 0.0);
  if (scan_list_default_squelch_spin_ != nullptr) {
    const QSignalBlocker blocker(scan_list_default_squelch_spin_);
    scan_list_default_squelch_spin_->setValue(default_squelch_db);
  }
  if (scan_list_monitor_checkbox_ != nullptr) {
    const QSignalBlocker blocker(scan_list_monitor_checkbox_);
    scan_list_monitor_checkbox_->setChecked(settings.value("monitor_mode", false).toBool());
  }

  int channel_count = settings.value("count", -1).toInt();
  if (channel_count < 0) {
    int max_index = -1;
    const QStringList groups = settings.childGroups();
    for (const QString& group : groups) {
      if (!group.startsWith("channel_")) {
        continue;
      }
      bool ok = false;
      const int idx = group.mid(8).toInt(&ok);
      if (ok && idx > max_index) {
        max_index = idx;
      }
    }
    channel_count = max_index + 1;
  }
  if (channel_count <= 0) {
    channel_count = 5;
  }

  scan_list_channels_.clear();
  scan_list_channels_.reserve(static_cast<size_t>(channel_count));
  for (int index = 0; index < channel_count; ++index) {
    ScanListChannelConfig channel;
    channel.squelch_threshold_db = default_squelch_db;
    settings.beginGroup(QString("channel_%1").arg(index));
    channel.label = settings.value("label", channel.label).toString();
    channel.frequency_mhz =
        settings.value("frequency_mhz", channel.frequency_mhz).toDouble();
    channel.bandwidth_hz = settings.value("bandwidth_hz", channel.bandwidth_hz).toInt();
    const bool has_use_default_flag = settings.contains("use_default_squelch");
    const bool has_saved_squelch = settings.contains("squelch_threshold_db");
    if (has_use_default_flag) {
      channel.use_default_squelch = settings.value("use_default_squelch").toBool();
    } else if (!has_saved_squelch) {
      channel.use_default_squelch = true;
    } else {
      const double saved_squelch =
          settings.value("squelch_threshold_db", channel.squelch_threshold_db).toDouble();
      channel.use_default_squelch = std::abs(saved_squelch - default_squelch_db) < 0.05;
    }
    if (has_saved_squelch) {
      channel.squelch_threshold_db =
          settings.value("squelch_threshold_db", channel.squelch_threshold_db).toDouble();
    }
    if (channel.use_default_squelch) {
      channel.squelch_threshold_db = default_squelch_db;
    }
    channel.dwell_ms = settings.value("dwell_ms", channel.dwell_ms).toInt();

    const int modulation = settings.value("modulation", static_cast<int>(channel.modulation)).toInt();
    switch (modulation) {
      case v1::MODULATION_AM:
        channel.modulation = v1::MODULATION_AM;
        break;
      case v1::MODULATION_WFM:
        channel.modulation = v1::MODULATION_WFM;
        break;
      case v1::MODULATION_NFM:
      default:
        channel.modulation = v1::MODULATION_NFM;
        break;
    }
    scan_list_channels_.push_back(std::move(channel));
    settings.endGroup();
  }
  settings.endGroup();
}

void MainWindow::SaveScanListConfigToSettings() const {
  QSettings settings("multi-radio", "multi-radio-client");
  settings.beginGroup("scan_list");
  settings.remove("");
  settings.setValue("default_dwell_ms", dwell_ms_spin_->value());
  settings.setValue("default_squelch_db", scan_list_default_squelch_spin_ != nullptr
                                             ? scan_list_default_squelch_spin_->value()
                                             : kDefaultScanListSquelchDb);
  settings.setValue("monitor_mode",
                    scan_list_monitor_checkbox_ != nullptr && scan_list_monitor_checkbox_->isChecked());
  settings.setValue("count", static_cast<int>(scan_list_channels_.size()));
  for (size_t index = 0; index < scan_list_channels_.size(); ++index) {
    const ScanListChannelConfig& channel = scan_list_channels_[index];
    settings.beginGroup(QString("channel_%1").arg(static_cast<qulonglong>(index)));
    settings.setValue("label", channel.label);
    settings.setValue("frequency_mhz", channel.frequency_mhz);
    settings.setValue("modulation", static_cast<int>(channel.modulation));
    settings.setValue("bandwidth_hz", channel.bandwidth_hz);
    settings.setValue("use_default_squelch", channel.use_default_squelch);
    if (channel.use_default_squelch) {
      settings.remove("squelch_threshold_db");
    } else {
      settings.setValue("squelch_threshold_db", channel.squelch_threshold_db);
    }
    settings.setValue("dwell_ms", channel.dwell_ms);
    settings.endGroup();
  }
  settings.endGroup();
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

  auto* auto_noise_checkbox = new QCheckBox("Waterfall: hide bins at/below mean", &dialog);
  auto_noise_checkbox->setChecked(signal_visualization_->AutoNoiseReductionEnabled());

  layout->addRow("FFT size", fft_combo);
  layout->addRow("Frequency start", start_hz_spin);
  layout->addRow("Frequency end", end_hz_spin);
  layout->addRow("Auto noise reduction", auto_noise_checkbox);

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
  const bool auto_noise_reduction = auto_noise_checkbox->isChecked();
  signal_visualization_->SetVisualizationSettings(fft_size, start_hz, end_hz);
  signal_visualization_->SetAutoNoiseReductionEnabled(auto_noise_reduction);
  AppendLog(QString("Updated visualization settings: FFT=%1, range=%2-%3 Hz, waterfall-noise-filter=%4")
                .arg(fft_size)
                .arg(start_hz, 0, 'f', 0)
                .arg(end_hz, 0, 'f', 0)
                .arg(auto_noise_reduction ? "on" : "off"));
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
  decoded_table_->setItem(current, 5, new QTableWidgetItem(row.decoded_summary));
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
