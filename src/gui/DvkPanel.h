#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QVector>

namespace AetherSDR {
class DvkModel;

class DvkPanel : public QWidget {
    Q_OBJECT
public:
    explicit DvkPanel(DvkModel* model, QWidget* parent = nullptr);
    int selectedSlot() const;

private slots:
    void onStatusChanged(int status, int id);
    void onRecordingChanged(int id);

private:
    DvkModel* m_model;
    QVector<QFrame*> m_rowFrames;
    QVector<QPushButton*> m_fkeyBtns;
    QVector<QLabel*> m_nameLabels;
    QVector<QLabel*> m_durLabels;
    QPushButton* m_recBtn;
    QPushButton* m_playBtn;
    QPushButton* m_prevBtn;
    QLabel* m_statusLabel;
    int m_selectedSlot{1};

    void selectSlot(int id);
    QString formatDuration(int ms);
};

} // namespace AetherSDR
