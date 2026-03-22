#include "PanadapterApplet.h"
#include "SpectrumWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QEvent>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>

namespace AetherSDR {

PanadapterApplet::PanadapterApplet(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    // ── Title bar (16px gradient, matching applet style) ─────────────────
    auto* titleBar = new QWidget;
    titleBar->setFixedHeight(16);
    titleBar->setStyleSheet(
        "QWidget { background: qlineargradient(x1:0,y1:0,x2:0,y2:1,"
        "stop:0 #3a4a5a, stop:0.5 #2a3a4a, stop:1 #1a2a38); "
        "border-bottom: 1px solid #0a1a28; }");

    auto* barLayout = new QHBoxLayout(titleBar);
    barLayout->setContentsMargins(6, 1, 4, 1);
    barLayout->setSpacing(2);

    m_titleLabel = new QLabel("Slice A");
    m_titleLabel->setStyleSheet("QLabel { background: transparent; color: #8aa8c0; "
                                "font-size: 10px; font-weight: bold; }");
    barLayout->addWidget(m_titleLabel);
    barLayout->addStretch();

    // Placeholder window control buttons (non-functional for now)
    const QString btnStyle = QStringLiteral(
        "QPushButton { background: transparent; color: #6a8090; "
        "border: none; font-size: 9px; padding: 0; }"
        "QPushButton:hover { color: #c8d8e8; }");

    for (const char* icon : {"_", "\u25A1", "\u00D7"}) {
        auto* btn = new QPushButton(icon);
        btn->setFixedSize(14, 14);
        btn->setStyleSheet(btnStyle);
        barLayout->addWidget(btn);
    }

    layout->addWidget(titleBar);

    // ── Spectrum widget (FFT + waterfall) ────────────────────────────────
    m_spectrum = new SpectrumWidget(this);
    m_spectrum->installEventFilter(this);  // detect clicks for pan activation
    layout->addWidget(m_spectrum, 1);

    // ── CW decode panel (hidden by default, shown in CW mode) ─────────
    m_cwPanel = new QWidget(this);
    m_cwPanel->setFixedHeight(80);
    m_cwPanel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_cwPanel->setStyleSheet("QWidget { background: #0a0a14; border-top: 1px solid #203040; }");

    auto* cwLayout = new QVBoxLayout(m_cwPanel);
    cwLayout->setContentsMargins(4, 2, 4, 2);
    cwLayout->setSpacing(1);

    // Stats bar: pitch, speed, clear button
    auto* cwBar = new QHBoxLayout;
    cwBar->setSpacing(6);
    auto* cwTitle = new QLabel("CW");
    cwTitle->setStyleSheet("QLabel { color: #00b4d8; font-size: 10px; font-weight: bold; background: transparent; }");
    cwBar->addWidget(cwTitle);

    m_cwStatsLabel = new QLabel;
    m_cwStatsLabel->setStyleSheet("QLabel { color: #6a8090; font-size: 10px; background: transparent; }");
    cwBar->addWidget(m_cwStatsLabel);
    cwBar->addStretch();

    auto* clearBtn = new QPushButton("CLR");
    clearBtn->setStyleSheet(
        "QPushButton { background: #1a2a3a; color: #8090a0; border: 1px solid #203040;"
        " border-radius: 2px; font-size: 9px; font-weight: bold;"
        " padding: 1px 6px; }"
        "QPushButton:hover { color: #c8d8e8; background: #2a3a4a; }");
    connect(clearBtn, &QPushButton::clicked, this, &PanadapterApplet::clearCwText);
    cwBar->addWidget(clearBtn);

    cwLayout->addLayout(cwBar);

    // Text area
    m_cwText = new QTextEdit;
    m_cwText->setReadOnly(true);
    m_cwText->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_cwText->setStyleSheet(
        "QTextEdit { background: #0a0a14; color: #00ff88; border: none;"
        " font-family: monospace; font-size: 13px; font-weight: bold; }"
        "QScrollBar:vertical { width: 6px; background: #0a0a14; }"
        "QScrollBar::handle:vertical { background: #304050; border-radius: 3px; }"
        "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }");
    m_cwText->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_cwText->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_cwText->setWordWrapMode(QTextOption::WrapAnywhere);
    cwLayout->addWidget(m_cwText);

    m_cwPanel->hide();
    layout->addWidget(m_cwPanel);
}

void PanadapterApplet::setSliceId(int id)
{
    const char letters[] = "ABCD";
    const char letter = (id >= 0 && id < 4) ? letters[id] : '?';
    m_titleLabel->setText(QString("Slice %1").arg(letter));
}

void PanadapterApplet::setCwPanelVisible(bool visible)
{
    m_cwPanel->setVisible(visible);
}

void PanadapterApplet::appendCwText(const QString& text, float cost)
{
    // Strip newlines — ggmorse inserts them on pitch changes, but we want
    // continuous flowing text. Replace with space to preserve word boundaries.
    QString clean = text;
    clean.replace('\n', ' ');

    // Color by confidence: lower cost = higher confidence
    //   < 0.15  green   (high confidence)
    //   < 0.35  yellow  (medium)
    //   < 0.60  orange  (meh)
    //   >= 0.60 red     (low confidence)
    QString color;
    if (cost < 0.15f)      color = "#00ff88";
    else if (cost < 0.35f) color = "#e0e040";
    else if (cost < 0.60f) color = "#ff9020";
    else                   color = "#ff4040";

    m_cwText->moveCursor(QTextCursor::End);
    m_cwText->insertHtml(QString("<span style=\"color:%1\">%2</span>")
        .arg(color, clean.toHtmlEscaped()));
    m_cwText->moveCursor(QTextCursor::End);
}

void PanadapterApplet::setCwStats(float pitchHz, float speedWpm)
{
    if (pitchHz > 0 && speedWpm > 0)
        m_cwStatsLabel->setText(QString("%1 Hz  %2 WPM").arg(pitchHz, 0, 'f', 0).arg(speedWpm, 0, 'f', 0));
}

void PanadapterApplet::clearCwText()
{
    m_cwText->clear();
}

bool PanadapterApplet::eventFilter(QObject* obj, QEvent* ev)
{
    if (ev->type() == QEvent::MouseButtonPress)
        emit activated(m_panId);
    return QWidget::eventFilter(obj, ev);
}

} // namespace AetherSDR
