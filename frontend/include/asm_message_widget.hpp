#pragma once

#include <QWidget>
#include <QVariantMap>
#include <QVBoxLayout>
#include <QScrollArea>

namespace multi_radio {

struct AsmMessageRecord {
    int msg_type;
    uint32_t mmsi;
    QString station_label;
    QString payload;
    QVariantMap fields;
    quint64 unix_ms;
};

/**
 * AsmMessageWidget displays a list of cards for AIS message types 6, 8, 12, and 14.
 * Clicking a card opens a dialog with the full decoded content.
 */
class AsmMessageWidget : public QWidget {
    Q_OBJECT
public:
    explicit AsmMessageWidget(QWidget* parent = nullptr);

    void AddMessage(int msg_type, uint32_t mmsi, const QString& label,
                    const QString& payload, const QVariantMap& fields, quint64 unix_ms);

private:
    void ShowDetails(const AsmMessageRecord& msg);

    QScrollArea* scroll_area_;
    QWidget* container_;
    QVBoxLayout* list_layout_;
};

} // namespace multi_radio