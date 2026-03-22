#include "PanadapterStack.h"
#include "PanadapterApplet.h"
#include "SpectrumWidget.h"

#include <QVBoxLayout>

namespace AetherSDR {

PanadapterStack::PanadapterStack(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    m_splitter = new QSplitter(Qt::Vertical, this);
    m_splitter->setHandleWidth(3);
    m_splitter->setChildrenCollapsible(false);
    layout->addWidget(m_splitter);
}

PanadapterApplet* PanadapterStack::addPanadapter(const QString& panId)
{
    if (m_pans.contains(panId))
        return m_pans[panId];

    auto* applet = new PanadapterApplet(m_splitter);
    applet->setPanId(panId);
    m_splitter->addWidget(applet);

    // Equal stretch for all pans
    const int idx = m_splitter->indexOf(applet);
    m_splitter->setStretchFactor(idx, 1);

    m_pans[panId] = applet;

    // First pan becomes active
    if (m_activePanId.isEmpty())
        setActivePan(panId);

    return applet;
}

void PanadapterStack::removePanadapter(const QString& panId)
{
    auto* applet = m_pans.take(panId);
    if (!applet) return;

    delete applet;

    // If active was removed, switch to first remaining
    if (m_activePanId == panId) {
        if (!m_pans.isEmpty())
            setActivePan(m_pans.firstKey());
        else
            m_activePanId.clear();
    }
}

PanadapterApplet* PanadapterStack::panadapter(const QString& panId) const
{
    return m_pans.value(panId, nullptr);
}

SpectrumWidget* PanadapterStack::spectrum(const QString& panId) const
{
    auto* applet = m_pans.value(panId, nullptr);
    return applet ? applet->spectrumWidget() : nullptr;
}

PanadapterApplet* PanadapterStack::activeApplet() const
{
    return m_pans.value(m_activePanId, nullptr);
}

SpectrumWidget* PanadapterStack::activeSpectrum() const
{
    auto* applet = activeApplet();
    return applet ? applet->spectrumWidget() : nullptr;
}

void PanadapterStack::setActivePan(const QString& panId)
{
    if (m_activePanId == panId) return;
    m_activePanId = panId;

    // Visual indicator: active pan gets cyan left border
    for (auto it = m_pans.begin(); it != m_pans.end(); ++it) {
        bool isActive = (it.key() == panId);
        it.value()->setStyleSheet(isActive
            ? "PanadapterApplet { border-left: 2px solid #00b4d8; }"
            : "PanadapterApplet { border-left: 2px solid transparent; }");
    }

    emit activePanChanged(panId);
}

} // namespace AetherSDR
