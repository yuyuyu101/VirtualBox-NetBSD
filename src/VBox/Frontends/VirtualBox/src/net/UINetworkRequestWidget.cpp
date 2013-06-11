/* $Id: UINetworkRequestWidget.cpp $ */
/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * UINetworkRequestWidget stuff implementation
 */

/*
 * Copyright (C) 2011-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

/* Global includes: */
#include <QTimer>
#include <QGridLayout>
#include <QProgressBar>

/* Local includes: */
#include "UINetworkRequestWidget.h"
#include "UINetworkRequest.h"
#include "UINetworkManager.h"
#include "UINetworkManagerDialog.h"
#include "UIIconPool.h"
#include "QIToolButton.h"
#include "QIRichTextLabel.h"

UINetworkRequestWidget::UINetworkRequestWidget(UINetworkManagerDialog *pParent, UINetworkRequest *pNetworkRequest)
    : QIWithRetranslateUI<UIPopupBox>(pParent)
    , m_pContentWidget(new QWidget(this))
    , m_pMainLayout(new QGridLayout(m_pContentWidget))
    , m_pProgressBar(new QProgressBar(m_pContentWidget))
    , m_pRetryButton(new QIToolButton(m_pContentWidget))
    , m_pCancelButton(new QIToolButton(m_pContentWidget))
    , m_pErrorPane(new QIRichTextLabel(m_pContentWidget))
    , m_pNetworkRequest(pNetworkRequest)
    , m_pTimer(new QTimer(this))
{
    /* Setup self: */
    setTitleIcon(UIIconPool::iconSet(":/nw_16px.png"));
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    setContentWidget(m_pContentWidget);
    setOpen(true);

    /* Prepare listeners for m_pNetworkRequest: */
    connect(m_pNetworkRequest, SIGNAL(sigProgress(qint64, qint64)), this, SLOT(sltSetProgress(qint64, qint64)));
    connect(m_pNetworkRequest, SIGNAL(sigStarted()), this, SLOT(sltSetProgressToStarted()));
    connect(m_pNetworkRequest, SIGNAL(sigFinished()), this, SLOT(sltSetProgressToFinished()));
    connect(m_pNetworkRequest, SIGNAL(sigFailed(const QString&)), this, SLOT(sltSetProgressToFailed(const QString&)));

    /* Setup timer: */
    m_pTimer->setInterval(5000);
    connect(m_pTimer, SIGNAL(timeout()), this, SLOT(sltTimeIsOut()));

    /* Setup main-layout: */
    m_pMainLayout->setContentsMargins(6, 6, 6, 6);

    /* Setup progress-bar: */
    m_pProgressBar->setRange(0, 0);
    m_pProgressBar->setMaximumHeight(16);

    /* Setup retry-button: */
    m_pRetryButton->setHidden(true);
    m_pRetryButton->removeBorder();
    m_pRetryButton->setFocusPolicy(Qt::NoFocus);
    m_pRetryButton->setIcon(UIIconPool::iconSet(":/refresh_16px.png"));
    connect(m_pRetryButton, SIGNAL(clicked(bool)), this, SIGNAL(sigRetry()));

    /* Setup cancel-button: */
    m_pCancelButton->removeBorder();
    m_pCancelButton->setFocusPolicy(Qt::NoFocus);
    m_pCancelButton->setIcon(UIIconPool::iconSet(":/delete_16px.png"));
    connect(m_pCancelButton, SIGNAL(clicked(bool)), this, SIGNAL(sigCancel()));

    /* Setup error-label: */
    m_pErrorPane->setHidden(true);
    m_pErrorPane->setWordWrapMode(QTextOption::WrapAtWordBoundaryOrAnywhere);
    /* Calculate required width: */
    int iMinimumWidth = pParent->minimumWidth();
    int iLeft, iTop, iRight, iBottom;
    /* Take into account content-widget layout margins: */
    m_pMainLayout->getContentsMargins(&iLeft, &iTop, &iRight, &iBottom);
    iMinimumWidth -= iLeft;
    iMinimumWidth -= iRight;
    /* Take into account this layout margins: */
    layout()->getContentsMargins(&iLeft, &iTop, &iRight, &iBottom);
    iMinimumWidth -= iLeft;
    iMinimumWidth -= iRight;
    /* Take into account parent layout margins: */
    QLayout *pParentLayout = qobject_cast<QMainWindow*>(parent())->centralWidget()->layout();
    pParentLayout->getContentsMargins(&iLeft, &iTop, &iRight, &iBottom);
    iMinimumWidth -= iLeft;
    iMinimumWidth -= iRight;
    /* Set minimum text width: */
    m_pErrorPane->setMinimumTextWidth(iMinimumWidth);

    /* Layout content: */
    m_pMainLayout->addWidget(m_pProgressBar, 0, 0);
    m_pMainLayout->addWidget(m_pRetryButton, 0, 1);
    m_pMainLayout->addWidget(m_pCancelButton, 0, 2);
    m_pMainLayout->addWidget(m_pErrorPane, 1, 0, 1, 3);

    /* Retranslate UI: */
    retranslateUi();
}

void UINetworkRequestWidget::sltSetProgress(qint64 iReceived, qint64 iTotal)
{
    /* Restart timer: */
    m_pTimer->start();

    /* Set current progress to passed: */
    m_pProgressBar->setRange(0, iTotal);
    m_pProgressBar->setValue(iReceived);
}

void UINetworkRequestWidget::sltSetProgressToStarted()
{
    /* Start timer: */
    m_pTimer->start();

    /* Set current progress to 'started': */
    m_pProgressBar->setRange(0, 1);
    m_pProgressBar->setValue(0);

    /* Hide 'retry' button: */
    m_pRetryButton->setHidden(true);

    /* Hide error label: */
    m_pErrorPane->setHidden(true);
    m_pErrorPane->setText(QString());
}

void UINetworkRequestWidget::sltSetProgressToFinished()
{
    /* Stop timer: */
    m_pTimer->stop();

    /* Set current progress to 'started': */
    m_pProgressBar->setRange(0, 1);
    m_pProgressBar->setValue(1);
}

void UINetworkRequestWidget::sltSetProgressToFailed(const QString &strError)
{
    /* Stop timer: */
    m_pTimer->stop();

    /* Set current progress to 'failed': */
    m_pProgressBar->setRange(0, 1);
    m_pProgressBar->setValue(1);

    /* Show 'retry' button: */
    m_pRetryButton->setHidden(false);

    /* Try to find all the links in the error-message,
     * replace them with %increment if present: */
    QString strErrorText(strError);
    QRegExp linkRegExp("[\\S]+[\\./][\\S]+");
    QStringList links;
    for (int i = 1; linkRegExp.indexIn(strErrorText) != -1; ++i)
    {
        links << linkRegExp.cap();
        strErrorText.replace(linkRegExp.cap(), QString("%%1").arg(i));
    }
    /* Return back all the links, just in bold: */
    if (!links.isEmpty())
        for (int i = 0; i < links.size(); ++i)
            strErrorText = strErrorText.arg(QString("<b>%1</b>").arg(links[i]));

    /* Show error label: */
    m_pErrorPane->setHidden(false);
    m_pErrorPane->setText(UINetworkManagerDialog::tr("Error: %1.").arg(strErrorText));
}

void UINetworkRequestWidget::sltTimeIsOut()
{
    /* Stop timer: */
    m_pTimer->stop();

    /* Set current progress to unknown: */
    m_pProgressBar->setRange(0, 0);
}

void UINetworkRequestWidget::retranslateUi()
{
    /* Get corresponding title: */
    const QString &strTitle = m_pNetworkRequest->description();

    /* Set popup title (default if missed): */
    setTitle(strTitle.isEmpty() ? UINetworkManagerDialog::tr("Network Operation") : strTitle);

    /* Translate retry button: */
    m_pRetryButton->setStatusTip(UINetworkManagerDialog::tr("Restart network operation"));

    /* Translate cancel button: */
    m_pCancelButton->setStatusTip(UINetworkManagerDialog::tr("Cancel network operation"));
}
