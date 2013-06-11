/* $Id: QIAdvancedSlider.cpp $ */
/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * VirtualBox Qt extensions: QIAdvancedSlider class implementation
 */

/*
 * Copyright (C) 2009-2010 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#include "QIAdvancedSlider.h"
#include "VBoxGlobal.h"

/* Qt includes */
#include <QVBoxLayout>
#include <QPainter>
#include <QStyle>
#include <QStyleOptionSlider>

/* System includes */
#include <math.h>

class CPrivateSlider: public QSlider
{
public:
    CPrivateSlider(Qt::Orientation fOrientation, QWidget *pParent = 0)
      : QSlider(fOrientation, pParent)
      , m_optColor(0x0, 0xff, 0x0, 0x3c)
      , m_wrnColor(0xff, 0x54, 0x0, 0x3c)
      , m_errColor(0xff, 0x0, 0x0, 0x3c)
      , m_minOpt(-1)
      , m_maxOpt(-1)
      , m_minWrn(-1)
      , m_maxWrn(-1)
      , m_minErr(-1)
      , m_maxErr(-1)
    {}

    int positionForValue(int val) const
    {
        QStyleOptionSlider opt;
        initStyleOption(&opt);
        opt.subControls = QStyle::SC_All;
        int available = opt.rect.width() - style()->pixelMetric(QStyle::PM_SliderLength, &opt, this);
        return QStyle::sliderPositionFromValue(opt.minimum, opt.maximum, val, available);
    }

    virtual void paintEvent(QPaintEvent *pEvent)
    {
        QPainter p(this);

        QStyleOptionSlider opt;
        initStyleOption(&opt);
        opt.subControls = QStyle::SC_All;

        int available = opt.rect.width() - style()->pixelMetric(QStyle::PM_SliderLength, &opt, this);
        QSize s = size();

        QRect ticks = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderTickmarks, this);
#ifdef Q_WS_MAC
        ticks.setRect((s.width() - available) / 2, s.height() - ticks.y(), available, ticks.height());
#else /* Q_WS_MAC */
        if (ticks.isNull() || ticks.isEmpty())
        {
            ticks = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this) | style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, this);
            ticks.setRect((s.width() - available) / 2, ticks.bottom() + 1, available, s.height() - ticks.bottom() - 1);
        }
#endif /* Q_WS_MAC */
        if ((m_minOpt != -1 &&
             m_maxOpt != -1) &&
            m_minOpt != m_maxOpt)
        {
            int posMinOpt = QStyle::sliderPositionFromValue(opt.minimum, opt.maximum, m_minOpt, available);
            int posMaxOpt = QStyle::sliderPositionFromValue(opt.minimum, opt.maximum, m_maxOpt, available);
            p.fillRect(ticks.x() + posMinOpt, ticks.y(), posMaxOpt - posMinOpt + 1, ticks.height(), m_optColor);
        }
        if ((m_minWrn != -1 &&
             m_maxWrn != -1) &&
            m_minWrn != m_maxWrn)
        {
            int posMinWrn = QStyle::sliderPositionFromValue(opt.minimum, opt.maximum, m_minWrn, available);
            int posMaxWrn = QStyle::sliderPositionFromValue(opt.minimum, opt.maximum, m_maxWrn, available);
            p.fillRect(ticks.x() + posMinWrn, ticks.y(), posMaxWrn - posMinWrn + 1, ticks.height(), m_wrnColor);
        }
        if ((m_minErr != -1 &&
             m_maxErr != -1) &&
            m_minErr != m_maxErr)
        {
            int posMinErr = QStyle::sliderPositionFromValue(opt.minimum, opt.maximum, m_minErr, available);
            int posMaxErr = QStyle::sliderPositionFromValue(opt.minimum, opt.maximum, m_maxErr, available);
            p.fillRect(ticks.x() + posMinErr, ticks.y(), posMaxErr - posMinErr + 1, ticks.height(), m_errColor);
        }
        p.end();

        QSlider::paintEvent(pEvent);
    }

    QColor m_optColor;
    QColor m_wrnColor;
    QColor m_errColor;

    int m_minOpt;
    int m_maxOpt;
    int m_minWrn;
    int m_maxWrn;
    int m_minErr;
    int m_maxErr;
};

QIAdvancedSlider::QIAdvancedSlider(QWidget *pParent /* = 0 */)
  : QWidget(pParent)
{
    init();
}

QIAdvancedSlider::QIAdvancedSlider(Qt::Orientation fOrientation, QWidget *pParent /* = 0 */)
  : QWidget(pParent)
{
    init(fOrientation);
}

int QIAdvancedSlider::value() const
{
    return m_pSlider->value();
}

void QIAdvancedSlider::setRange(int minV, int maxV)
{
    m_pSlider->setRange(minV, maxV);
}

void QIAdvancedSlider::setMaximum(int val)
{
    m_pSlider->setMaximum(val);
}

int QIAdvancedSlider::maximum() const
{
    return m_pSlider->maximum();
}

void QIAdvancedSlider::setMinimum(int val)
{
    m_pSlider->setMinimum(val);
}

int QIAdvancedSlider::minimum() const
{
    return m_pSlider->minimum();
}

void QIAdvancedSlider::setPageStep(int val)
{
    m_pSlider->setPageStep(val);
}

int QIAdvancedSlider::pageStep() const
{
    return m_pSlider->pageStep();
}

void QIAdvancedSlider::setSingleStep(int val)
{
    m_pSlider->setSingleStep(val);
}

int QIAdvancedSlider::singelStep() const
{
    return m_pSlider->singleStep();
}

void QIAdvancedSlider::setTickInterval(int val)
{
    m_pSlider->setTickInterval(val);
}

int QIAdvancedSlider::tickInterval() const
{
    return m_pSlider->tickInterval();
}

void QIAdvancedSlider::setTickPosition(QSlider::TickPosition fPos)
{
    m_pSlider->setTickPosition(fPos);
}

QSlider::TickPosition QIAdvancedSlider::tickPosition() const
{
    return m_pSlider->tickPosition();
}

Qt::Orientation QIAdvancedSlider::orientation() const
{
    return m_pSlider->orientation();
}

void QIAdvancedSlider::setSnappingEnabled(bool fOn)
{
    m_fSnappingEnabled = fOn;
}

bool QIAdvancedSlider::isSnappingEnabled() const
{
    return m_fSnappingEnabled;
}

void QIAdvancedSlider::setOptimalHint(int min, int max)
{
    m_pSlider->m_minOpt = min;
    m_pSlider->m_maxOpt = max;

    update();
}

void QIAdvancedSlider::setWarningHint(int min, int max)
{
    m_pSlider->m_minWrn = min;
    m_pSlider->m_maxWrn = max;

    update();
}

void QIAdvancedSlider::setErrorHint(int min, int max)
{
    m_pSlider->m_minErr = min;
    m_pSlider->m_maxErr = max;

    update();
}

void QIAdvancedSlider::setOrientation(Qt::Orientation fOrientation)
{
    m_pSlider->setOrientation(fOrientation);
}

void QIAdvancedSlider::setValue (int val)
{
    m_pSlider->setValue(val);
}

void QIAdvancedSlider::sltSliderMoved(int val)
{
    val = snapValue(val);
    m_pSlider->setValue(val);
    emit sliderMoved(val);
}

void QIAdvancedSlider::init(Qt::Orientation fOrientation /* = Qt::Horizontal */)
{
    m_fSnappingEnabled = false;

    QVBoxLayout *pMainLayout = new QVBoxLayout(this);
    VBoxGlobal::setLayoutMargin(pMainLayout, 0);
    m_pSlider = new CPrivateSlider(fOrientation, this);
    pMainLayout->addWidget(m_pSlider);

    connect(m_pSlider, SIGNAL(sliderMoved(int)), this, SLOT(sltSliderMoved(int)));
    connect(m_pSlider, SIGNAL(valueChanged(int)), this, SIGNAL(valueChanged(int)));
    connect(m_pSlider, SIGNAL(sliderPressed()), this, SIGNAL(sliderPressed()));
    connect(m_pSlider, SIGNAL(sliderReleased()), this, SIGNAL(sliderReleased()));
}

int QIAdvancedSlider::snapValue(int val)
{
    if (m_fSnappingEnabled &&
        val > 2)
    {
        float l2 = log((float)val)/log(2.0);
        int newVal = (int)pow((float)2, (int)qRound(l2)); /* The value to snap on */
        int pos = m_pSlider->positionForValue(val); /* Get the relative screen pos for the original value */
        int newPos = m_pSlider->positionForValue(newVal); /* Get the relative screen pos for the snap value */
        if (abs(newPos - pos) < 5) /* 10 pixel snapping range */
        {
            val = newVal;
            if (val > m_pSlider->maximum())
                val = m_pSlider->maximum();
            else if (val < m_pSlider->minimum())
                val = m_pSlider->minimum();
        }
    }
    return val;
}

