#pragma once

#include <QWidget>

class QLabel;
class QTextEdit;

namespace AetherSDR {

class SpectrumWidget;

// Container for a single panadapter display (FFT spectrum + waterfall).
// Adds a title bar with placeholder min/max/close buttons above the
// SpectrumWidget.  Prepares for future multi-slice stacking where each
// slice gets its own PanadapterApplet in a vertical splitter.
class PanadapterApplet : public QWidget {
    Q_OBJECT

public:
    explicit PanadapterApplet(QWidget* parent = nullptr);

    SpectrumWidget* spectrumWidget() const { return m_spectrum; }

    // Panadapter identity (e.g. "0x40000000")
    QString panId() const { return m_panId; }
    void setPanId(const QString& id) { m_panId = id; }

    // Set the slice ID (0=A, 1=B, 2=C, 3=D) shown in the title bar.
    void setSliceId(int id);

    // CW decode panel
    void setCwPanelVisible(bool visible);
    void appendCwText(const QString& text, float cost = 0.0f);
    void setCwStats(float pitchHz, float speedWpm);
    void clearCwText();

    QSize sizeHint() const override { return {800, 316}; }

signals:
    void activated(const QString& panId);

protected:
    bool eventFilter(QObject* obj, QEvent* ev) override;

private:
    QString m_panId;
    SpectrumWidget* m_spectrum{nullptr};
    QLabel*         m_titleLabel{nullptr};

    // CW decode
    QWidget*   m_cwPanel{nullptr};
    QTextEdit* m_cwText{nullptr};
    QLabel*    m_cwStatsLabel{nullptr};
};

} // namespace AetherSDR
