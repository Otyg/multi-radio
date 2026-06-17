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

QString MsgTypeIcon(int type) {
    switch (type) {
        case 6:  return "✉"; // Addressed
        case 8:  return "📡"; // Broadcast
        case 12: return "⚠"; // Addressed Safety
        case 14: return "📢"; // Safety Broadcast
        default: return "✉";
    }
}
} // namespace

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
    card->setCursor(Qt::PointingHandCursor);
    card->setMinimumHeight(88);
    card->setStyleSheet(
        "QPushButton { text-align: left; padding: 8px; border-radius: 8px; "
        "border: 1px solid #2E7D32; background: #0B1018; color: #E0E6ED; }"
        "QPushButton:hover { background: #162231; border-color: #5CDB95; }");

    auto* v = new QVBoxLayout(card);
    v->setContentsMargins(10, 6, 10, 6);
    v->setSpacing(2);

    const QString station = label.isEmpty() ? QString::number(mmsi) : QString("%1 (%2)").arg(label).arg(mmsi);
    auto* line1 = new QLabel(QString("<b>%1 %2</b>").arg(MsgTypeIcon(msg_type)).arg(MsgTypeName(msg_type)), card);
    line1->setStyleSheet("color: #5CDB95; font-size: 14px; background: transparent;");

    auto* line2 = new QLabel(station, card);
    line2->setStyleSheet("color: #DDFBE6; font-size: 13px; background: transparent;");

    auto* line3 = new QLabel(QString("Mottaget %1").arg(QDateTime::fromMSecsSinceEpoch(unix_ms).toLocalTime().toString("HH:mm:ss")), card);
    line3->setStyleSheet("color: #8FA7BE; font-size: 11px; background: transparent;");

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

void AsmMessageWidget::ShowDetails(const AsmMessageRecord& msg) {
    QDialog dlg(this);
    dlg.setWindowTitle(QString("AIS Message Details - Type %1").arg(msg.msg_type));
    dlg.resize(500, 400);
    dlg.setStyleSheet("background: #0B0F16; color: #B2C0D6;");

    auto* v = new QVBoxLayout(&dlg);
    v->addWidget(new QLabel(QString("<b>From:</b> %1 (MMSI: %2)").arg(msg.station_label).arg(msg.mmsi)));
    v->addWidget(new QLabel(QString("<b>Type:</b> %1").arg(MsgTypeName(msg.msg_type))));
    v->addWidget(new QLabel(QString("<b>Time:</b> %1").arg(QDateTime::fromMSecsSinceEpoch(msg.unix_ms).toLocalTime().toString())));

    auto* summary_label = new QLabel(QString("<b>Summary:</b> %1").arg(msg.payload), &dlg);
    summary_label->setWordWrap(true);
    v->addWidget(summary_label);

    auto* text = new QTextEdit(&dlg);
    text->setReadOnly(true);
    text->setStyleSheet("background: #121E2E; border: 1px solid #2E7D32; color: #DDFBE6; font-family: monospace;");

    QJsonObject obj;
    for (auto it = msg.fields.begin(); it != msg.fields.end(); ++it)
        obj.insert(it.key(), QJsonValue::fromVariant(it.value()));
    QJsonDocument doc(obj);

    // Samla både hex och metadata i den kopierbara textrutan
    QString content;
    if (msg.fields.contains("hex")) {
        content += "RAW HEX:\n" + msg.fields.value("hex").toString() + "\n\n";
    }
    content += "METADATA (JSON):\n" + doc.toJson(QJsonDocument::Indented);

    text->setPlainText(content);

    v->addWidget(text);
    auto* bb = new QDialogButtonBox(QDialogButtonBox::Close, &dlg);
    connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::accept);
    v->addWidget(bb);
    dlg.exec();
}

} // namespace multi_radio