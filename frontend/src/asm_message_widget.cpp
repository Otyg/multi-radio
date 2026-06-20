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

void AsmMessageWidget::ShowDetails(const AsmMessageRecord& msg) {
    QDialog dlg(this);
    dlg.setWindowTitle(QString("AIS Message Details - Type %1").arg(msg.msg_type));
    dlg.resize(500, 400);
    dlg.setStyleSheet("background: #0B0F16; color: #B2C0D6;");

    auto* v = new QVBoxLayout(&dlg);
    v->addWidget(new QLabel(QString("<b>From:</b> %1 (MMSI: %2)").arg(msg.station_label).arg(msg.mmsi)));
    v->addWidget(new QLabel(QString("<b>Type:</b> %1").arg(MsgTypeName(msg.msg_type))));
    v->addWidget(new QLabel(QString("<b>Time:</b> %1").arg(QDateTime::fromMSecsSinceEpoch(msg.unix_ms).toLocalTime().toString())));

    auto* text = new QTextEdit(&dlg);
    text->setReadOnly(true);
    text->setStyleSheet("background: #121E2E; border: 1px solid #2E7D32; color: #DDFBE6; font-family: monospace;");

    // Convert fields to pretty JSON
    QJsonObject obj;
    for (auto it = msg.fields.begin(); it != msg.fields.end(); ++it)
        obj.insert(it.key(), QJsonValue::fromVariant(it.value()));
    QJsonDocument doc(obj);
    text->setPlainText(doc.toJson(QJsonDocument::Indented));

    v->addWidget(text);
    auto* bb = new QDialogButtonBox(QDialogButtonBox::Close, &dlg);
    connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::accept);
    v->addWidget(bb);
    dlg.exec();
}

} // namespace multi_radio