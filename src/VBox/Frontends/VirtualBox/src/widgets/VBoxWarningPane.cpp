/* $Id: VBoxWarningPane.cpp $ */
/** @file
 *
 * VBox frontends: Qt4 GUI ("VirtualBox"):
 * VBoxWarningPane class implementation
 */

/*
 * Copyright (C) 2009-2011 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

/* Global includes */
#include <QHBoxLayout>

/* Local includes */
#include "VBoxWarningPane.h"
#include "VBoxGlobal.h"

VBoxWarningPane::VBoxWarningPane(QWidget *pParent)
    : QWidget(pParent)
{
    QHBoxLayout *pLayout = new QHBoxLayout(this);
    VBoxGlobal::setLayoutMargin(pLayout, 0);
    pLayout->addWidget(&m_icon);
    pLayout->addWidget(&m_label);
}

void VBoxWarningPane::setWarningPixmap(const QPixmap &imgPixmap)
{
    m_icon.setPixmap(imgPixmap);
}

void VBoxWarningPane::setWarningText(const QString &strText)
{
    m_label.setText(strText);
}

