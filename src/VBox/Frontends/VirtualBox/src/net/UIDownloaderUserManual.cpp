/* $Id: UIDownloaderUserManual.cpp $ */
/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * UIDownloaderUserManual class implementation
 */

/*
 * Copyright (C) 2006-2012 Oracle Corporation
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
#include <QDir>
#include <QFile>

/* Local includes: */
#include "UIDownloaderUserManual.h"
#include "QIFileDialog.h"
#include "VBoxGlobal.h"
#include "UIMessageCenter.h"

/* static */
UIDownloaderUserManual* UIDownloaderUserManual::m_spInstance = 0;

/* static */
UIDownloaderUserManual* UIDownloaderUserManual::create()
{
    if (!m_spInstance)
        m_spInstance = new UIDownloaderUserManual;
    return m_spInstance;
}

/* static */
UIDownloaderUserManual* UIDownloaderUserManual::current()
{
    return m_spInstance;
}

UIDownloaderUserManual::UIDownloaderUserManual()
{
    /* Prepare instance: */
    if (!m_spInstance)
        m_spInstance = this;

    /* Set description: */
    setDescription(tr("VirtualBox User Manual"));

    /* Compose User Manual filename: */
    QString strUserManualFullFileName = vboxGlobal().helpFile();
    QString strUserManualShortFileName = QFileInfo(strUserManualFullFileName).fileName();

    /* Add sources: */
    addSource(QString("http://download.virtualbox.org/virtualbox/%1/").arg(vboxGlobal().vboxVersionStringNormalized()) + strUserManualShortFileName);
    addSource(QString("http://download.virtualbox.org/virtualbox/") + strUserManualShortFileName);

    /* Set target: */
    QString strUserManualDestination = QDir(vboxGlobal().virtualBox().GetHomeFolder()).absoluteFilePath(strUserManualShortFileName);
    setTarget(strUserManualDestination);
}

UIDownloaderUserManual::~UIDownloaderUserManual()
{
    /* Cleanup instance: */
    if (m_spInstance == this)
        m_spInstance = 0;
}

bool UIDownloaderUserManual::askForDownloadingConfirmation(QNetworkReply *pReply)
{
    return msgCenter().confirmUserManualDownload(source().toString(), pReply->header(QNetworkRequest::ContentLengthHeader).toInt());
}

void UIDownloaderUserManual::handleDownloadedObject(QNetworkReply *pReply)
{
    /* Read received data into the buffer: */
    QByteArray receivedData(pReply->readAll());
    /* Serialize that buffer into the file: */
    while (true)
    {
        /* Try to open file for writing: */
        QFile file(target());
        if (file.open(QIODevice::WriteOnly))
        {
            /* Write buffer into the file: */
            file.write(receivedData);
            file.close();

            /* Warn the user about user-manual loaded and saved: */
            msgCenter().warnAboutUserManualDownloaded(source().toString(), QDir::toNativeSeparators(target()));
            /* Warn the listener about user-manual was downloaded: */
            emit sigDownloadFinished(target());
            break;
        }

        /* Warn user about user-manual was downloaded but was NOT saved: */
        msgCenter().warnAboutUserManualCantBeSaved(source().toString(), QDir::toNativeSeparators(target()));

        /* Ask the user for another location for the user-manual file: */
        QString strTarget = QIFileDialog::getExistingDirectory(QFileInfo(target()).absolutePath(),
                                                               msgCenter().networkManagerOrMainWindowShown(),
                                                               tr("Select folder to save User Manual to"), true);

        /* Check if user had really set a new target: */
        if (!strTarget.isNull())
            setTarget(QDir(strTarget).absoluteFilePath(QFileInfo(target()).fileName()));
        else
            break;
    }
}

