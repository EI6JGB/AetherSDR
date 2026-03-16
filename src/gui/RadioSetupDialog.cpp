#include "RadioSetupDialog.h"
#include "models/RadioModel.h"

#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QComboBox>
#include <QDialogButtonBox>

namespace AetherSDR {

static const QString kGroupStyle =
    "QGroupBox { border: 1px solid #304050; border-radius: 4px; "
    "margin-top: 8px; padding-top: 12px; font-weight: bold; color: #8aa8c0; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; "
    "padding: 0 4px; }";

static const QString kLabelStyle =
    "QLabel { color: #c8d8e8; font-size: 12px; }";

static const QString kValueStyle =
    "QLabel { color: #00c8ff; font-size: 12px; font-weight: bold; }";

static const QString kEditStyle =
    "QLineEdit { background: #1a2a3a; border: 1px solid #304050; "
    "border-radius: 3px; color: #c8d8e8; font-size: 12px; padding: 2px 4px; }";

RadioSetupDialog::RadioSetupDialog(RadioModel* model, QWidget* parent)
    : QDialog(parent), m_model(model)
{
    setWindowTitle("Radio Setup");
    setMinimumSize(580, 400);
    setStyleSheet("QDialog { background: #0f0f1a; }");

    auto* layout = new QVBoxLayout(this);

    auto* tabs = new QTabWidget;
    tabs->setStyleSheet(
        "QTabWidget::pane { border: 1px solid #304050; background: #0f0f1a; }"
        "QTabBar::tab { background: #1a2a3a; color: #8aa8c0; "
        "border: 1px solid #304050; padding: 4px 12px; margin-right: 2px; }"
        "QTabBar::tab:selected { background: #0f0f1a; color: #c8d8e8; "
        "border-bottom-color: #0f0f1a; }");

    tabs->addTab(buildRadioTab(), "Radio");
    tabs->addTab(buildNetworkTab(), "Network");
    tabs->addTab(buildGpsTab(), "GPS");
    tabs->addTab(buildTxTab(), "TX");
    tabs->addTab(buildPhoneCwTab(), "Phone/CW");
    tabs->addTab(buildRxTab(), "RX");
    tabs->addTab(buildFiltersTab(), "Filters");
    tabs->addTab(buildXvtrTab(), "XVTR");

    layout->addWidget(tabs);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    buttons->setStyleSheet(
        "QPushButton { background: #1a2a3a; border: 1px solid #304050; "
        "border-radius: 3px; color: #c8d8e8; padding: 4px 16px; }"
        "QPushButton:hover { background: #203040; }");
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::close);
    layout->addWidget(buttons);
}

// ── Radio tab ─────────────────────────────────────────────────────────────────

QWidget* RadioSetupDialog::buildRadioTab()
{
    auto* page = new QWidget;
    auto* vbox = new QVBoxLayout(page);
    vbox->setSpacing(8);

    // Toggle button style: green when on, red when off
    static const QString kToggleStyle =
        "QPushButton { background: #1a2a3a; border: 1px solid #304050; "
        "border-radius: 3px; color: #c8d8e8; font-size: 11px; font-weight: bold; "
        "padding: 3px 10px; }"
        "QPushButton:checked { background: #1a5030; color: #00e060; "
        "border: 1px solid #20a040; }";

    auto makeToggle = [](const QString& text, bool checked) {
        auto* btn = new QPushButton(checked ? "Enabled" : "Disabled");
        btn->setCheckable(true);
        btn->setChecked(checked);
        btn->setStyleSheet(kToggleStyle);
        QObject::connect(btn, &QPushButton::toggled, btn, [btn](bool on) {
            btn->setText(on ? "Enabled" : "Disabled");
        });
        return btn;
    };

    // Radio Information group
    {
        auto* group = new QGroupBox("Radio Information");
        group->setStyleSheet(kGroupStyle);
        auto* grid = new QGridLayout(group);
        grid->setSpacing(6);

        grid->addWidget(new QLabel("Radio SN:"), 0, 0);
        m_serialLabel = new QLabel(m_model->chassisSerial().isEmpty()
            ? m_model->serial() : m_model->chassisSerial());
        m_serialLabel->setStyleSheet(kValueStyle);
        grid->addWidget(m_serialLabel, 0, 1);

        grid->addWidget(new QLabel("Region:"), 0, 2);
        m_regionLabel = new QLabel(m_model->region().isEmpty() ? "USA" : m_model->region());
        m_regionLabel->setStyleSheet(
            "QLabel { background: #1a2a3a; border: 1px solid #304050; "
            "border-radius: 3px; color: #00c8ff; font-size: 11px; font-weight: bold; "
            "padding: 3px 10px; }");
        m_regionLabel->setAlignment(Qt::AlignCenter);
        grid->addWidget(m_regionLabel, 0, 3);

        grid->addWidget(new QLabel("HW Version:"), 1, 0);
        m_hwVersionLabel = new QLabel("v" + m_model->version());
        m_hwVersionLabel->setStyleSheet(kValueStyle);
        grid->addWidget(m_hwVersionLabel, 1, 1);

        grid->addWidget(new QLabel("Remote On:"), 1, 2);
        m_remoteOnBtn = makeToggle("", m_model->remoteOnEnabled());
        connect(m_remoteOnBtn, &QPushButton::toggled, this, [this](bool on) {
            m_model->setRemoteOnEnabled(on);
        });
        grid->addWidget(m_remoteOnBtn, 1, 3);

        grid->addWidget(new QLabel("Options:"), 2, 0);
        m_optionsLabel = new QLabel(m_model->radioOptions().isEmpty()
            ? (m_model->hasAmplifier() ? "GPS, PGXL" : "GPS")
            : m_model->radioOptions());
        m_optionsLabel->setStyleSheet(kValueStyle);
        grid->addWidget(m_optionsLabel, 2, 1);

        grid->addWidget(new QLabel("FlexControl:"), 2, 2);
        auto* fcBtn = makeToggle("", true);
        grid->addWidget(fcBtn, 2, 3);

        grid->addWidget(new QLabel("multiFLEX:"), 3, 2);
        auto* mfBtn = makeToggle("", m_model->multiFlexEnabled());
        connect(mfBtn, &QPushButton::toggled, this, [this](bool on) {
            m_model->setMultiFlexEnabled(on);
        });
        grid->addWidget(mfBtn, 3, 3);

        for (auto* lbl : group->findChildren<QLabel*>()) {
            if (lbl->styleSheet().isEmpty())
                lbl->setStyleSheet(kLabelStyle);
        }

        vbox->addWidget(group);
    }

    // Radio Identification group
    {
        auto* group = new QGroupBox("Radio Identification");
        group->setStyleSheet(kGroupStyle);
        auto* grid = new QGridLayout(group);
        grid->setSpacing(6);

        grid->addWidget(new QLabel("Model:"), 0, 0);
        m_modelLabel = new QLabel(m_model->model());
        m_modelLabel->setStyleSheet(kValueStyle);
        grid->addWidget(m_modelLabel, 0, 1);

        grid->addWidget(new QLabel("Nickname:"), 0, 2);
        m_nicknameEdit = new QLineEdit(m_model->nickname().isEmpty()
            ? m_model->name() : m_model->nickname());
        m_nicknameEdit->setStyleSheet(kEditStyle);
        grid->addWidget(m_nicknameEdit, 0, 3);

        grid->addWidget(new QLabel("Callsign:"), 1, 0);
        m_callsignEdit = new QLineEdit(m_model->callsign());
        m_callsignEdit->setStyleSheet(kEditStyle);
        grid->addWidget(m_callsignEdit, 1, 1);

        connect(m_nicknameEdit, &QLineEdit::editingFinished, this, [this] {
            m_model->connection()->sendCommand("radio name " + m_nicknameEdit->text());
        });
        connect(m_callsignEdit, &QLineEdit::editingFinished, this, [this] {
            m_model->connection()->sendCommand("radio callsign " + m_callsignEdit->text());
        });

        for (auto* lbl : group->findChildren<QLabel*>()) {
            if (lbl->styleSheet().isEmpty())
                lbl->setStyleSheet(kLabelStyle);
        }

        vbox->addWidget(group);
    }

    vbox->addStretch(1);
    return page;
}

// ── Placeholder tabs ──────────────────────────────────────────────────────────

static QWidget* placeholderTab(const QString& name)
{
    auto* page = new QWidget;
    auto* vbox = new QVBoxLayout(page);
    auto* lbl = new QLabel(name + " settings — coming soon");
    lbl->setStyleSheet("QLabel { color: #6888a0; font-size: 14px; }");
    lbl->setAlignment(Qt::AlignCenter);
    vbox->addWidget(lbl);
    return page;
}

QWidget* RadioSetupDialog::buildNetworkTab()  { return placeholderTab("Network"); }
QWidget* RadioSetupDialog::buildGpsTab()      { return placeholderTab("GPS"); }
QWidget* RadioSetupDialog::buildTxTab()       { return placeholderTab("TX"); }
QWidget* RadioSetupDialog::buildPhoneCwTab()  { return placeholderTab("Phone/CW"); }
QWidget* RadioSetupDialog::buildRxTab()       { return placeholderTab("RX"); }
QWidget* RadioSetupDialog::buildFiltersTab()  { return placeholderTab("Filters"); }
QWidget* RadioSetupDialog::buildXvtrTab()     { return placeholderTab("XVTR"); }

} // namespace AetherSDR
