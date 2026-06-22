#include "asm_message_widget.hpp"

#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QString>
#include <QStringList>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <memory>

// Inkludera libais-headers
#include "ais.h"

namespace multi_radio {

namespace {
QString MsgTypeName(int type) {
    switch (type) {
        case 6:  return "Addressed Binary (6)";
        case 8:  return "Binary Broadcast (8)";
        case 12: return "Addressed Safety (12)";
        case 14: return "Safety Broadcast (14)";
        default: return QString("AIS Type %1").arg(type);
    }
}
}

std::string HexToAis6BitAscii(const std::string& hex) {
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < hex.length(); i += 2) {
        std::string byteString = hex.substr(i, 2);
        uint8_t byte = (uint8_t) strtol(byteString.c_str(), nullptr, 16);
        bytes.push_back(byte);
    }

    std::string ascii6bit = "";
    size_t bitCount = 0;
    uint32_t buffer = 0;

    for (uint8_t b : bytes) {
        buffer = (buffer << 8) | b;
        bitCount += 8;
        while (bitCount >= 6) {
            uint8_t val = (buffer >> (bitCount - 6)) & 0x3F;
            bitCount -= 6;
            if (val < 40) val += 48;
            else val += 56;
            ascii6bit += (char)val;
        }
    }
    return ascii6bit;
}
QString DecodeAis6BitString(libais::AisBitset& bits, size_t start_bit, size_t num_bits) {
    QString result = "";
    size_t num_chars = num_bits / 6;
    for (size_t i = 0; i < num_chars; ++i) {
        uint32_t val = bits.ToUnsignedInt(start_bit + (i * 6), 6);
        if (val == 0) break;
        if (val < 32) result.append(QChar(val + 64));
        else result.append(QChar(val));
    }
    return result.trimmed();
}
QString GetAreaTypeText(int type) {
    switch (type) {
        case 0:  return "Inshore traffic zone";
        case 1:  return "Traffic separation scheme lane";
        case 2:  return "Traffic separation scheme roundabout";
        case 3:  return "Precautionary area";
        case 4:  return "Area to be avoided";
        case 5:  return "Two-way traffic route";
        case 6:  return "Recommended track";
        case 7:  return "Recommended traffic lane";
        case 8:  return "Deep-water route";
        case 9:  return "Fairway";
        case 10: return "Regulated area / Restricted area";
        case 11: return "Anchorage area";
        case 12: return "Asdic / Fishing area";
        case 13: return "Military practice area / Danger area";
        case 14: return "Pilot boarding area";
        default: return QString("Unknown Area Type (%1)").arg(type);
    }
}

QString GetAreaShapeText(int shape) {
    switch (shape) {
        case 0: return "Circle / Point";
        case 1: return "Rectangle / Box";
        case 2: return "Sector";
        case 3: return "Polyline";
        case 4: return "Polygon";
        case 5: return "Text notation only";
        default: return QString("Unknown Shape (%1)").arg(shape);
    }
}

QString GetTargetTypeText(int type) {
    switch (type) {
        case 0: return "Default / Unknown target";
        case 1: return "Vessel / Ship";
        case 2: return "Iceberg";
        case 3: return "Floating ice / Pack ice";
        case 4: return "AtoN (Aid to Navigation)";
        case 5: return "Oil slick / Pollution";
        case 6: return "Debris / Flotsam";
        case 7: return "Whale / Marine mammal";
        case 8: return "Life raft / Person in water";
        default: return QString("Unknown Target Type (%1)").arg(type);
    }
}

QString GetAtonTypeText(int type) {
    switch (type) {
        case 0:  return "Default / Type not specified";
        case 1:  return "Reference point";
        case 2:  return "RACON (Radar beacon)";
        case 3:  return "Fixed light / Beacon";
        case 4:  return "Light vessel / Lightship";
        case 5:  return "Light buoy";
        case 6:  return "Cardinal buoy - North";
        case 7:  return "Cardinal buoy - East";
        case 8:  return "Cardinal buoy - South";
        case 9:  return "Cardinal buoy - West";
        case 10: return "Isolated danger mark";
        case 11: return "Safe water mark";
        case 12: return "Special mark";
        case 13: return "Lateral mark - Port hand";
        case 14: return "Lateral mark - Starboard hand";
        default: return QString("Unknown AtoN Type (%1)").arg(type);
    }
}

QString GetNoticeTypeText(int type) {
    switch (type) {
        case 0:  return "Cautionary notice / General warning";
        case 1:  return "Underwater obstruction / Wreck";
        case 2:  return "Drifting hazard (e.g. logs, containers)";
        case 3:  return "Gunnery / Military exercise area active";
        case 4:  return "Cable / Pipeline laying operations";
        case 5:  return "Dredging / Underwater works active";
        case 6:  return "Diving operations active";
        case 7:  return "Bridge closed / Bridge works";
        case 8:  return "Fairway closed / Blocked";
        case 9:  return "AtoN defective / Out of position";
        case 10: return "Search and Rescue (SAR) active";
        default: return QString("Unknown Notice Type (%1)").arg(type);
    }
}

QString GetSensorSourceText(int source) {
    switch (source) {
        case 0: return "Onboard Sensor (Vessel)";
        case 1: return "Shore-based Station / Bureau";
        case 2: return "Buoy / Floating station";
        case 3: return "Fixed structure / Offshore platform";
        case 4: return "Aircraft / UAV";
        case 5: return "Satellite / Remote sensing";
        default: return QString("Unknown Sensor Source (%1)").arg(source);
    }
}

QString GetSensorStatusText(int status) {
    switch (status) {
        case 0: return "Operational / Normal data";
        case 1: return "Calibrating / Maintenance mode";
        case 2: return "Degraded performance / Warning";
        case 3: return "Error / Unreliable data";
        default: return "Unknown Status";
    }
}

QString GetRouteTypeText(int type) {
    switch (type) {
        case 0: return "Recommended / Standard Route";
        case 1: return "Alternative Route";
        case 2: return "Mandatory Traffic Route / Lane";
        case 3: return "Transit corridor";
        case 4: return "Inbound / Entry track";
        case 5: return "Outbound / Exit track";
        default: return QString("Unknown Route Type (%1)").arg(type);
    }
}

QString GetRetransmitText(int flag) {
    return (flag == 1) ? "Retransmitted (Not original)" : "Original transmission";
}

AsmMessageWidget::AsmMessageWidget(QWidget* parent) : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(4);

    auto* header = new QLabel("<b>APPLICATION & SAFETY MESSAGES</b>", this);
    header->setStyleSheet("color: #5CDB95; font-size: 11px; margin-top: 5px; margin-left: 5px;");
    root->addWidget(header);

    scroll_area_ = new QScrollArea(this);
    scroll_area_->setFrameShape(QFrame::NoFrame);
    scroll_area_->setWidgetResizable(true);
    scroll_area_->setStyleSheet("background: transparent;");

    container_ = new QWidget(scroll_area_);
    container_->setStyleSheet("background: transparent;");
    list_layout_ = new QVBoxLayout(container_);
    list_layout_->setContentsMargins(4, 4, 4, 4);
    list_layout_->setSpacing(6);
    list_layout_->addStretch(1);

    scroll_area_->setWidget(container_);
    root->addWidget(scroll_area_);
}

void AsmMessageWidget::AddMessage(int msg_type, uint32_t mmsi, const QString& label,
                                 const QString& payload, const QVariantMap& fields, quint64 unix_ms) {
    AsmMessageRecord msg{msg_type, mmsi, label, payload, fields, unix_ms};

    auto* card = new QPushButton(container_);
    card->setMinimumHeight(88);
    card->setCursor(Qt::PointingHandCursor);
    card->setStyleSheet(
        "QPushButton { text-align: left; padding: 8px; border-radius: 6px; "
        "border: 1px solid #1B5E20; background: #0D141F; color: #E0E6ED; }"
        "QPushButton:hover { background: #162231; border-color: #5CDB95; }");

    auto* v = new QVBoxLayout(card);
    v->setContentsMargins(5, 5, 5, 5);
    v->setSpacing(2);
    const QString station = label.isEmpty() ? QString::number(mmsi) : QString("%1 (%2)").arg(label).arg(mmsi);
    auto* line1 = new QLabel(QString("<b>%1</b> — %2").arg(MsgTypeName(msg_type)).arg(station), card);
    line1->setStyleSheet("color: #92E6B5; font-size: 13px;");

    auto* line2 = new QLabel(payload, card);
    line2->setWordWrap(true);
    line2->setStyleSheet("color: #B2C0D6; font-size: 12px;");

    auto* line3 = new QLabel(QDateTime::fromMSecsSinceEpoch(unix_ms).toLocalTime().toString("HH:mm:ss"), card);
    line3->setStyleSheet("color: #6B7A90; font-size: 10px;");

    v->addWidget(line1);
    v->addWidget(line2);
    v->addWidget(line3);

    connect(card, &QPushButton::clicked, this, [this, msg]() { ShowDetails(msg); });

    // Insert at top (newest first)
    list_layout_->insertWidget(0, card);

    // Prune old messages
    if (list_layout_->count() > 51) { // 50 messages + 1 stretch
        auto* old = list_layout_->takeAt(list_layout_->count() - 2);
        if (old->widget()) old->widget()->deleteLater();
        delete old;
    }
}

void AsmMessageWidget::ShowDetails(const AsmMessageRecord& org_msg) {
    QDialog dlg(this);
    dlg.setWindowTitle(QString("AIS Message Details - Type %1").arg(org_msg.msg_type));
    dlg.resize(500, 400);
    dlg.setStyleSheet("background: #0B0F16; color: #B2C0D6;");

    auto* v = new QVBoxLayout(&dlg);
    v->addWidget(new QLabel(QString("<b>From:</b> %1 (MMSI: %2)").arg(org_msg.station_label).arg(org_msg.mmsi)));
    v->addWidget(new QLabel(QString("<b>Type:</b> %1").arg(MsgTypeName(org_msg.msg_type))));
    v->addWidget(new QLabel(QString("<b>Time:</b> %1").arg(QDateTime::fromMSecsSinceEpoch(org_msg.unix_ms).toLocalTime().toString())));

    auto* text = new QTextEdit(&dlg);
    text->setReadOnly(true);
    text->setStyleSheet("background: #121E2E; border: 1px solid #2E7D32; color: #DDFBE6; font-family: monospace;");

    // Convert fields to pretty JSON
    std::string ais_ascii = HexToAis6BitAscii(org_msg.payload.toStdString());
    int pad = 0; // Standard fyllnadsbitar för råa fristående payloads
     // B. Konstruera och initiera bitsetet på rätt sätt enligt libais API
    libais::AisBitset bits;
    
    // ParseNmeaPayload är den korrekta funktionen i schwehr/libais
    libais::AIS_STATUS parse_status = bits.ParseNmeaPayload(ais_ascii.c_str(), pad);
    
    if (parse_status != libais::AIS_OK) {
        std::cerr << "Fel: Kunde inte parsa bitströmmen. Statuskod:" + parse_status;
        return;
    }
    if (bits.GetNumBits() < 6) {
        std::cerr << "Fel: Strängen innehåller för få bitar för att läsa meddelande-ID.";
        return;
    }int message_id = bits.ToUnsignedInt(0, 6);
    QJsonObject root_obj;
    
    QString msg_type_text = "Unknown AIS Message";
    if (message_id == 6) msg_type_text = "Binary Addressed Message (Type 6)";
    else if (message_id == 8) msg_type_text = "Binary Broadcast Message (Type 8)";
    else if (message_id == 12) msg_type_text = "Addressed Safety Message (Type 12)";
    else if (message_id == 14) msg_type_text = "Broadcast Safety Message (Type 14)";

    root_obj.insert("Message Type", msg_type_text);
    root_obj.insert("Repeat Indicator", (int)bits.ToUnsignedInt(6, 2));
    root_obj.insert("Source MMSI", (double)bits.ToUnsignedInt(8, 30));

    if (message_id == 6 || message_id == 8) {
        int dac = 0;
        int fi = 0;
        size_t payload_start = 0;

        if (message_id == 6) {
            root_obj.insert("Sequence Number", (int)bits.ToUnsignedInt(38, 2));
            root_obj.insert("Destination MMSI", (double)bits.ToUnsignedInt(40, 30));
            root_obj.insert("Retransmit Status", (bits.ToUnsignedInt(70, 1) == 1) ? "Retransmitted" : "Original");
            dac = bits.ToUnsignedInt(72, 10);
            fi = bits.ToUnsignedInt(82, 6);
            payload_start = 88;
        } else {
            dac = bits.ToUnsignedInt(40, 10);
            fi = bits.ToUnsignedInt(50, 6);
            payload_start = 56;
        }

        root_obj.insert("DAC", dac);
        root_obj.insert("FI", fi);

        if (dac == 1 && (fi == 11 || fi == 13 || fi == 17 || fi == 21 || 
                         fi == 22 || fi == 26 || fi == 28 || fi == 29 || fi == 31)) {
            
            QJsonObject app_fields;

            if (fi == 11) {
                int area_type = bits.ToUnsignedInt(payload_start, 4);
                int area_shape = bits.ToUnsignedInt(payload_start + 4, 3);
                app_fields.insert("Area Type", GetAreaTypeText(area_type));
                app_fields.insert("Area Shape", GetAreaShapeText(area_shape));
            }
            else if (fi == 13) {
                app_fields.insert("Persons Onboard", (int)bits.ToUnsignedInt(payload_start, 13));
            }
            else if (fi == 17) {
                int target_type = bits.ToUnsignedInt(payload_start, 4);
                app_fields.insert("Target Type", GetTargetTypeText(target_type));
                app_fields.insert("Latitude", bits.ToInt(payload_start + 4, 24) / 60000.0);
                app_fields.insert("Longitude", bits.ToInt(payload_start + 28, 25) / 60000.0);
            }
            else if (fi == 21) {
                int aton_type = bits.ToUnsignedInt(payload_start, 5);
                int status_bits = bits.ToUnsignedInt(payload_start + 5, 8);
                app_fields.insert("AtoN Type", GetAtonTypeText(aton_type));
                app_fields.insert("Light Operational Status", (status_bits & 0x80) ? "Error" : "Good");
                app_fields.insert("RACON Status", (status_bits & 0x40) ? "Error" : "Good");
            }
            else if (fi == 22) {
                int notice_type = bits.ToUnsignedInt(payload_start, 7);
                app_fields.insert("Notice Type", GetNoticeTypeText(notice_type));
                app_fields.insert("Latitude", bits.ToInt(payload_start + 7, 24) / 60000.0);
                app_fields.insert("Longitude", bits.ToInt(payload_start + 31, 25) / 60000.0);
            }
            else if (fi == 26) {
                int sensor_src = bits.ToUnsignedInt(payload_start, 4);
                int sensor_stat = bits.ToUnsignedInt(payload_start + 4, 2);
                app_fields.insert("Sensor Source", GetSensorSourceText(sensor_src));
                app_fields.insert("Sensor Status", GetSensorStatusText(sensor_stat));
            }
            else if (fi == 28) {
                int route_type = bits.ToUnsignedInt(payload_start, 5);
                app_fields.insert("Route Type", GetRouteTypeText(route_type));
                app_fields.insert("Waypoints Count", (int)bits.ToUnsignedInt(payload_start + 5, 4));
            }
            else if (fi == 29) {
                app_fields.insert("Text Segment", DecodeAis6BitString(bits, payload_start, bits.GetNumBits() - payload_start));
            }
            else if (fi == 31) {
                app_fields.insert("Latitude", bits.ToInt(payload_start, 24) / 60000.0);
                app_fields.insert("Longitude", bits.ToInt(payload_start + 24, 25) / 60000.0);
                int raw_ws = bits.ToUnsignedInt(payload_start + 49, 7);
                app_fields.insert("Wind Speed (knots)", raw_ws == 127 ? "N/A" : QJsonValue(raw_ws));
                int raw_wg = bits.ToUnsignedInt(payload_start + 56, 7);
                app_fields.insert("Wind Gusts (knots)", raw_wg == 127 ? "N/A" : QJsonValue(raw_wg));
                int raw_wd = bits.ToUnsignedInt(payload_start + 63, 9);
                app_fields.insert("Wind Direction", raw_wd == 511 ? "N/A" : QJsonValue(raw_wd));
                int raw_vis = bits.ToUnsignedInt(payload_start + 93, 7);
                app_fields.insert("Visibility (NM)", raw_vis == 127 ? "N/A" : QJsonValue(raw_vis / 10.0));
            }

            root_obj.insert("Decoded Application Data", app_fields);
        } else {
            root_obj.insert("Application Info", "Ignored: Missing selection filter criteria.");
        }
    }
    else if (message_id == 12 || message_id == 14) {
        size_t text_start_bit = (message_id == 12) ? 72 : 40;
        if (message_id == 12) {
            root_obj.insert("Sequence Number", (int)bits.ToUnsignedInt(38, 2));
            root_obj.insert("Destination MMSI", (double)bits.ToUnsignedInt(40, 30));
        }
        size_t text_bits = bits.GetNumBits() - text_start_bit;
        root_obj.insert("Safety Message Text", DecodeAis6BitString(bits, text_start_bit, text_bits));
    }

    root_obj.insert("Raw Hex Payload", org_msg.payload);
    root_obj.insert("AIS ASCII Transport", QString::fromStdString(ais_ascii));

    QJsonDocument doc(root_obj);
    text->setPlainText(doc.toJson(QJsonDocument::Indented));

    v->addWidget(text);
    auto* bb = new QDialogButtonBox(QDialogButtonBox::Close, &dlg);
    connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::accept);
    v->addWidget(bb);
    dlg.exec();
}

} // namespace multi_radio