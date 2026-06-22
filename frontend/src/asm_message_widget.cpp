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
    }
    QJsonObject obj;
    obj.insert("Message Type", (int)bits.ToUnsignedInt(0, 6));
    obj.insert("Repeat Indicator", (int)bits.ToUnsignedInt(6, 2));
    obj.insert("MMSI", (double)bits.ToUnsignedInt(8, 30));
    obj.insert("Total Bits", (int)bits.GetNumBits());

    // Loopa igenom resten av bitarna i meddelandet generellt och spara som rådata-block
    size_t total_bits = bits.GetNumBits();
    QJsonObject raw_fields;
    int field_counter = 1;
    
    // Vi läser ut resterande data i 8-bitars (byte) segment helt generellt
    for (size_t i = 38; i + 8 <= total_bits; i += 8) {
        QString key = QString("Byte_%1_bit_%2").arg(field_counter++).arg(i);
        uint32_t val = bits.ToUnsignedInt(i, 8);
        raw_fields.insert(key, (int)val);
    }
    obj.insert("Decoded Raw Bytes", raw_fields);

    // 4. Lägg till din egna metadata
    obj.insert("Hex payload", org_msg.payload);
    obj.insert("AIS ASCII Transport", QString::fromStdString(ais_ascii));
    QJsonDocument doc(obj);
    text->setPlainText(doc.toJson(QJsonDocument::Indented));

    v->addWidget(text);
    auto* bb = new QDialogButtonBox(QDialogButtonBox::Close, &dlg);
    connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::accept);
    v->addWidget(bb);
    dlg.exec();
}

} // namespace multi_radio