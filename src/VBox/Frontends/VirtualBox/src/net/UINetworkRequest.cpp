/* $Id: UINetworkRequest.cpp $ */
/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * UINetworkRequest stuff implementation
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
#include <QNetworkReply>

/* Local includes: */
#include "UINetworkRequest.h"
#include "UINetworkRequestWidget.h"
#include "UINetworkManager.h"
#include "UINetworkManagerDialog.h"
#include "UINetworkManagerIndicator.h"
#include "UINetworkCustomer.h"
#include "VBoxGlobal.h"

/* Constructor: */
UINetworkRequest::UINetworkRequest(const QNetworkRequest &request, UINetworkRequestType type, const QString &strDescription,
                                   UINetworkCustomer *pCustomer,
                                   UINetworkManager *pNetworkManager)
    : QObject(pNetworkManager)
    , m_pNetworkManagerDialog(pNetworkManager->window())
    , m_pNetworkManagerIndicator(pNetworkManager->indicator())
    , m_uuid(QUuid::createUuid())
    , m_requests(QList<QNetworkRequest>() << request)
    , m_iCurrentRequestIndex(0)
    , m_type(type)
    , m_strDescription(strDescription)
    , m_pCustomer(pCustomer)
    , m_fRunning(false)
{
    /* Initialize: */
    initialize();
}

UINetworkRequest::UINetworkRequest(const QList<QNetworkRequest> &requests, UINetworkRequestType type, const QString &strDescription,
                                   UINetworkCustomer *pCustomer,
                                   UINetworkManager *pNetworkManager)
    : QObject(pNetworkManager)
    , m_pNetworkManagerDialog(pNetworkManager->window())
    , m_pNetworkManagerIndicator(pNetworkManager->indicator())
    , m_uuid(QUuid::createUuid())
    , m_requests(requests)
    , m_iCurrentRequestIndex(0)
    , m_type(type)
    , m_strDescription(strDescription)
    , m_pCustomer(pCustomer)
    , m_fRunning(false)
{
    /* Initialize: */
    initialize();
}

/* Destructor: */
UINetworkRequest::~UINetworkRequest()
{
    /* Destroy network-reply: */
    cleanupNetworkReply();

    /* Remove network-request description from network-manager state-indicator: */
    if (m_pNetworkManagerIndicator)
        m_pNetworkManagerIndicator->removeNetworkRequest(m_uuid);

    /* Remove network-request widget from network-manager dialog: */
    m_pNetworkManagerDialog->removeNetworkRequestWidget(m_uuid);
}

/* Network-reply progress handler: */
void UINetworkRequest::sltHandleNetworkReplyProgress(qint64 iReceived, qint64 iTotal)
{
    /* Notify general network-requests listeners: */
    emit sigProgress(m_uuid, iReceived, iTotal);
    /* Notify particular network-request listeners: */
    emit sigProgress(iReceived, iTotal);
}

/* Network-reply finish handler: */
void UINetworkRequest::sltHandleNetworkReplyFinish()
{
    /* Set as non-running: */
    m_fRunning = false;

    /* Get sender network-reply: */
    QNetworkReply *pNetworkReply = static_cast<QNetworkReply*>(sender());

    /* If network-request was canceled: */
    if (pNetworkReply->error() == QNetworkReply::OperationCanceledError)
    {
        /* Notify network-manager: */
        emit sigCanceled(m_uuid);
    }
    /* If network-reply has no errors: */
    else if (pNetworkReply->error() == QNetworkReply::NoError)
    {
        /* Check if redirection required: */
        QUrl redirect = pNetworkReply->attribute(QNetworkRequest::RedirectionTargetAttribute).toUrl();
        if (redirect.isValid())
        {
            /* Cleanup current network-reply first: */
            cleanupNetworkReply();

            /* Choose redirect-source as current: */
            m_request.setUrl(redirect);

            /* Create new network-reply finally: */
            prepareNetworkReply();
        }
        else
        {
            /* Notify particular network-request listeners: */
            emit sigFinished();
            /* Notify general network-requests listeners: */
            emit sigFinished(m_uuid);
        }
    }
    /* If some error occured: */
    else
    {
        /* Check if we have other requests in set: */
        if (m_iCurrentRequestIndex < m_requests.size() - 1)
        {
            /* Cleanup current network-reply first: */
            cleanupNetworkReply();

            /* Choose next network-request as current: */
            ++m_iCurrentRequestIndex;
            m_request = m_requests[m_iCurrentRequestIndex];

            /* Create new network-reply finally: */
            prepareNetworkReply();
        }
        else
        {
            /* Notify particular network-request listeners: */
            emit sigFailed(pNetworkReply->errorString());
            /* Notify general network-requests listeners: */
            emit sigFailed(m_uuid, pNetworkReply->errorString());
        }
    }
}

/* Slot to retry network-request: */
void UINetworkRequest::sltRetry()
{
    /* Cleanup current network-reply first: */
    cleanupNetworkReply();

    /* Choose first network-request as current: */
    m_iCurrentRequestIndex = 0;
    m_request = m_requests[m_iCurrentRequestIndex];

    /* Create new network-reply finally: */
    prepareNetworkReply();
}

/* Slot to cancel network-request: */
void UINetworkRequest::sltCancel()
{
    /* Abort network-reply if present: */
    abortNetworkReply();
}

/* Initialize: */
void UINetworkRequest::initialize()
{
    /* Prepare listeners for parent(): */
    connect(parent(), SIGNAL(sigCancelNetworkRequests()), this, SLOT(sltCancel()));

    /* Create network-request widget in network-manager dialog: */
    m_pNetworkManagerDialog->addNetworkRequestWidget(this);

    /* Create network-request description in network-manager state-indicator: */
    if (m_pNetworkManagerIndicator)
        m_pNetworkManagerIndicator->addNetworkRequest(this);

    /* Choose first network-request as current: */
    m_iCurrentRequestIndex = 0;
    m_request = m_requests[m_iCurrentRequestIndex];

    /* Create network-reply: */
    prepareNetworkReply();
}

/* Prepare network-reply: */
void UINetworkRequest::prepareNetworkReply()
{
    /* Make network-request: */
    switch (m_type)
    {
        case UINetworkRequestType_HEAD:
        {
            m_pReply = gNetworkManager->head(m_request);
            break;
        }
        case UINetworkRequestType_GET:
        {
            m_pReply = gNetworkManager->get(m_request);
            break;
        }
        default:
            break;
    }

    /* Prepare listeners for m_pReply: */
    AssertMsg(m_pReply, ("Unable to make network-request!\n"));
    connect(m_pReply, SIGNAL(downloadProgress(qint64, qint64)), this, SLOT(sltHandleNetworkReplyProgress(qint64, qint64)));
    connect(m_pReply, SIGNAL(finished()), this, SLOT(sltHandleNetworkReplyFinish()));

    /* Set as running: */
    m_fRunning = true;

    /* Notify general network-requests listeners: */
    emit sigStarted(m_uuid);
    /* Notify particular network-request listeners: */
    emit sigStarted();
}

/* Cleanup network-reply: */
void UINetworkRequest::cleanupNetworkReply()
{
    /* Destroy current reply: */
    AssertMsg(m_pReply, ("Network-reply already destroyed!\n"));
    m_pReply->disconnect();
    m_pReply->deleteLater();
    m_pReply = 0;
}

/* Abort network-reply: */
void UINetworkRequest::abortNetworkReply()
{
    /* Abort network-reply if present: */
    if (m_pReply)
    {
        if (m_fRunning)
            m_pReply->abort();
        else
            emit sigCanceled(m_uuid);
    }
}

