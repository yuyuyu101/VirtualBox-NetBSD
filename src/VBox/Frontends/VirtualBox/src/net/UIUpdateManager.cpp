/* $Id: UIUpdateManager.cpp $ */
/** @file
 *
 * VBox frontends: Qt4 GUI ("VirtualBox"):
 * UIUpdateManager class implementation
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

/* Qt includes: */
#include <QNetworkReply>
#include <QTimer>
#include <QDir>
#include <QPointer>
#include <VBox/version.h>

/* GUI includes: */
#include "UIUpdateDefs.h"
#include "UIUpdateManager.h"
#include "UINetworkManager.h"
#include "UINetworkCustomer.h"
#include "VBoxGlobal.h"
#include "UIMessageCenter.h"
#include "VBoxUtils.h"
#include "UIDownloaderExtensionPack.h"
#include "UIGlobalSettingsExtension.h"
#include "QIProcess.h"

/* COM includes: */
#include "CExtPack.h"
#include "CExtPackManager.h"

/* Other VBox includes: */
#include <iprt/path.h>
#include <iprt/system.h>

/* Forward declarations: */
class UIUpdateStep;

/* Queue for processing update-steps: */
class UIUpdateQueue : public QObject
{
    Q_OBJECT;

signals:

    /* Starting-signal of the queue: */
    void sigStartQueue();

    /* Completion-signal of the queue: */
    void sigQueueFinished();

public:

    /* Constructor: */
    UIUpdateQueue(UIUpdateManager *pParent) : QObject(pParent) {}

    /* Starts a queue: */
    void start() { emit sigStartQueue(); }

private:

    /* Helpers: */
    bool isEmpty() const { return m_pLastStep.isNull(); }
    UIUpdateStep* lastStep() const { return m_pLastStep; }
    void setLastStep(UIUpdateStep *pStep) { m_pLastStep = pStep; }

    /* Variables: */
    QPointer<UIUpdateStep> m_pLastStep;

    /* Friend classes: */
    friend class UIUpdateStep;
};

/* Interface representing update-step: */
class UIUpdateStep : public UINetworkCustomer
{
    Q_OBJECT;

signals:

    /* Completion-signal of the step: */
    void sigStepComplete();

public:

    /* Constructor: */
    UIUpdateStep(UIUpdateQueue *pQueue, bool fForceCall) : UINetworkCustomer(pQueue, fForceCall)
    {
        /* If queue has no steps yet: */
        if (pQueue->isEmpty())
        {
            /* Connect starting-signal of the queue to starting-slot of this step: */
            connect(pQueue, SIGNAL(sigStartQueue()), this, SLOT(sltStartStep()), Qt::QueuedConnection);
        }
        /* If queue has at least one step already: */
        else
        {
            /* Reconnect completion-signal of the last-step from completion-signal of the queue to starting-slot of this step: */
            disconnect(pQueue->lastStep(), SIGNAL(sigStepComplete()), pQueue, SIGNAL(sigQueueFinished()));
            connect(pQueue->lastStep(), SIGNAL(sigStepComplete()), this, SLOT(sltStartStep()), Qt::QueuedConnection);
        }

        /* Connect completion-signal of this step to the completion-signal of the queue: */
        connect(this, SIGNAL(sigStepComplete()), pQueue, SIGNAL(sigQueueFinished()), Qt::QueuedConnection);
        /* Connect completion-signal of this step to the destruction-slot of this step: */
        connect(this, SIGNAL(sigStepComplete()), this, SLOT(deleteLater()), Qt::QueuedConnection);

        /* Remember this step as the last one: */
        pQueue->setLastStep(this);
    }

protected slots:

    /* Starting-slot of the step: */
    virtual void sltStartStep() = 0;

protected:

    /* Network pregress handler dummy: */
    void processNetworkReplyProgress(qint64, qint64) {}
    /* Network reply canceled handler dummy: */
    void processNetworkReplyCanceled(QNetworkReply*) {}
    /* Network reply canceled handler dummy: */
    void processNetworkReplyFinished(QNetworkReply*) {}
};

/* Update-step to check for the new VirtualBox version: */
class UIUpdateStepVirtualBox : public UIUpdateStep
{
    Q_OBJECT;

public:

    /* Constructor: */
    UIUpdateStepVirtualBox(UIUpdateQueue *pQueue, bool fForceCall)
        : UIUpdateStep(pQueue, fForceCall)
        , m_url("http://update.virtualbox.org/query.php")
    {
    }

private slots:

    /* Startup slot: */
    void sltStartStep() { prepareNetworkRequest(); }

private:

    /* Prepare network request: */
    void prepareNetworkRequest()
    {
        /* Calculate the count of checks left: */
        int cCount = 1;
        QString strCount = vboxGlobal().virtualBox().GetExtraData(GUI_UpdateCheckCount);
        if (!strCount.isEmpty())
        {
            bool ok = false;
            int c = strCount.toLongLong(&ok);
            if (ok) cCount = c;
        }

        /* Compose query: */
        QUrl url(m_url);
        url.addQueryItem("platform", vboxGlobal().virtualBox().GetPackageType());
        /* Check if branding is active: */
        if (vboxGlobal().brandingIsActive())
        {
            /* Branding: Check whether we have a local branding file which tells us our version suffix "FOO"
                         (e.g. 3.06.54321_FOO) to identify this installation: */
            url.addQueryItem("version", QString("%1_%2_%3").arg(vboxGlobal().virtualBox().GetVersion())
                                                           .arg(vboxGlobal().virtualBox().GetRevision())
                                                           .arg(vboxGlobal().brandingGetKey("VerSuffix")));
        }
        else
        {
            /* Use hard coded version set by VBOX_VERSION_STRING: */
            url.addQueryItem("version", QString("%1_%2").arg(vboxGlobal().virtualBox().GetVersion())
                                                        .arg(vboxGlobal().virtualBox().GetRevision()));
        }
        url.addQueryItem("count", QString::number(cCount));
        url.addQueryItem("branch", VBoxUpdateData(vboxGlobal().virtualBox().GetExtraData(GUI_UpdateDate)).branchName());
        QString strUserAgent(QString("VirtualBox %1 <%2>").arg(vboxGlobal().virtualBox().GetVersion()).arg(platformInfo()));

        /* Send GET request: */
        QNetworkRequest request;
        request.setUrl(url);
        request.setRawHeader("User-Agent", strUserAgent.toAscii());
        createNetworkRequest(request, UINetworkRequestType_GET, tr("Checking for a new VirtualBox version..."));
    }

    /* Handle network reply canceled: */
    void processNetworkReplyCanceled(QNetworkReply* /* pReply */)
    {
        /* Notify about step completion: */
        emit sigStepComplete();
    }

    /* Handle network reply: */
    void processNetworkReplyFinished(QNetworkReply *pReply)
    {
        /* Deserialize incoming data: */
        QString strResponseData(pReply->readAll());

        /* Newer version of necessary package found: */
        if (strResponseData.indexOf(QRegExp("^\\d+\\.\\d+\\.\\d+ \\S+$")) == 0)
        {
            QStringList response = strResponseData.split(" ", QString::SkipEmptyParts);
            msgCenter().showUpdateSuccess(response[0], response[1]);
        }
        /* No newer version of necessary package found: */
        else
        {
            if (isItForceCall())
                msgCenter().showUpdateNotFound();
        }

        /* Save left count of checks: */
        int cCount = 1;
        QString strCount = vboxGlobal().virtualBox().GetExtraData(GUI_UpdateCheckCount);
        if (!strCount.isEmpty())
        {
            bool ok = false;
            int c = strCount.toLongLong(&ok);
            if (ok) cCount = c;
        }
        vboxGlobal().virtualBox().SetExtraData(GUI_UpdateCheckCount, QString("%1").arg((qulonglong)cCount + 1));

        /* Notify about step completion: */
        emit sigStepComplete();
    }

    /* Platform information getter: */
    static QString platformInfo()
    {
        /* Prepare platform report: */
        QString strPlatform;

#if defined (Q_OS_WIN)
        strPlatform = "win";
#elif defined (Q_OS_LINUX)
        strPlatform = "linux";
#elif defined (Q_OS_MACX)
        strPlatform = "macosx";
#elif defined (Q_OS_OS2)
        strPlatform = "os2";
#elif defined (Q_OS_FREEBSD)
        strPlatform = "freebsd";
#elif defined (Q_OS_SOLARIS)
        strPlatform = "solaris";
#else
        strPlatform = "unknown";
#endif

        /* The format is <system>.<bitness>: */
        strPlatform += QString(".%1").arg(ARCH_BITS);

        /* Add more system information: */
#if defined (Q_OS_WIN)
        OSVERSIONINFO versionInfo;
        ZeroMemory(&versionInfo, sizeof(OSVERSIONINFO));
        versionInfo.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
        GetVersionEx(&versionInfo);
        int iMajor = versionInfo.dwMajorVersion;
        int iMinor = versionInfo.dwMinorVersion;
        int iBuild = versionInfo.dwBuildNumber;
        QString strVersionInfo = QString::fromUtf16((ushort*)versionInfo.szCSDVersion);

        QString strDistributiveInfo;
        if (iMajor == 6)
            strDistributiveInfo = QString("Windows Vista %1");
        else if (iMajor == 5)
        {
            if (iMinor == 2)
                strDistributiveInfo = QString("Windows Server 2003 %1");
            else if (iMinor == 1)
                strDistributiveInfo = QString("Windows XP %1");
            else if (iMinor == 0)
                strDistributiveInfo = QString("Windows 2000 %1");
            else
                strDistributiveInfo = QString("Unknown %1");
        }
        else if (iMajor == 4)
        {
            if (iMinor == 90)
                strDistributiveInfo = QString("Windows Me %1");
            else if (iMinor == 10)
                strDistributiveInfo = QString("Windows 98 %1");
            else if (iMinor == 0)
                strDistributiveInfo = QString("Windows 95 %1");
            else
                strDistributiveInfo = QString("Unknown %1");
        }
        else strDistributiveInfo = QString("Unknown %1");
        // TODO: Windows Server 2008 == Vista? */
        strDistributiveInfo = strDistributiveInfo.arg(strVersionInfo);
        QString strVersion = QString("%1.%2").arg(iMajor).arg(iMinor);
        QString strKernel = QString("%1").arg(iBuild);
        strPlatform += QString(" [Distribution: %1 | Version: %2 | Build: %3]")
            .arg(strDistributiveInfo).arg(strVersion).arg(strKernel);
#elif defined (Q_OS_LINUX)
        /* Get script path: */
        char szAppPrivPath[RTPATH_MAX];
        int rc = RTPathAppPrivateNoArch(szAppPrivPath, sizeof(szAppPrivPath)); NOREF(rc);
        AssertRC(rc);
        /* Run script: */
        QByteArray result = QIProcess::singleShot(QString(szAppPrivPath) + "/VBoxSysInfo.sh");
        if (!result.isNull())
            strPlatform += QString(" [%1]").arg(QString(result).trimmed());
#else
        /* Use RTSystemQueryOSInfo: */
        char szTmp[256];
        QStringList components;
        int vrc = RTSystemQueryOSInfo(RTSYSOSINFO_PRODUCT, szTmp, sizeof(szTmp));
        if ((RT_SUCCESS(vrc) || vrc == VERR_BUFFER_OVERFLOW) && szTmp[0] != '\0')
            components << QString("Product: %1").arg(szTmp);
        vrc = RTSystemQueryOSInfo(RTSYSOSINFO_RELEASE, szTmp, sizeof(szTmp));
        if ((RT_SUCCESS(vrc) || vrc == VERR_BUFFER_OVERFLOW) && szTmp[0] != '\0')
            components << QString("Release: %1").arg(szTmp);
        vrc = RTSystemQueryOSInfo(RTSYSOSINFO_VERSION, szTmp, sizeof(szTmp));
        if ((RT_SUCCESS(vrc) || vrc == VERR_BUFFER_OVERFLOW) && szTmp[0] != '\0')
            components << QString("Version: %1").arg(szTmp);
        vrc = RTSystemQueryOSInfo(RTSYSOSINFO_SERVICE_PACK, szTmp, sizeof(szTmp));
        if ((RT_SUCCESS (vrc) || vrc == VERR_BUFFER_OVERFLOW) && szTmp[0] != '\0')
            components << QString("SP: %1").arg(szTmp);
        if (!components.isEmpty())
            strPlatform += QString(" [%1]").arg(components.join(" | "));
#endif

        return strPlatform;
    }

private:

    /* Variables: */
    QUrl m_url;
};

/* Update-step to check for the new VirtualBox Extension Pack version: */
class UIUpdateStepVirtualBoxExtensionPack : public UIUpdateStep
{
    Q_OBJECT;

public:

    /* Constructor: */
    UIUpdateStepVirtualBoxExtensionPack(UIUpdateQueue *pQueue, bool fForceCall)
        : UIUpdateStep(pQueue, fForceCall)
    {
    }

private slots:

    /* Startup slot: */
    void sltStartStep()
    {
        /* Return if already downloading: */
        if (UIDownloaderExtensionPack::current())
        {
            emit sigStepComplete();
            return;
        }

        /* Get extension pack: */
        CExtPack extPack = vboxGlobal().virtualBox().GetExtensionPackManager().Find(GUI_ExtPackName);
        /* Return if extension pack is NOT installed: */
        if (extPack.isNull())
        {
            emit sigStepComplete();
            return;
        }

        /* Get VirtualBox version: */
        QString strVBoxVersion(vboxGlobal().vboxVersionStringNormalized());
        QByteArray abVBoxVersion = strVBoxVersion.toUtf8();
        VBoxVersion vboxVersion(strVBoxVersion);

        /* Get extension pack version: */
        QString strExtPackVersion(extPack.GetVersion());
        QByteArray abExtPackVersion = strExtPackVersion.toUtf8();

        /* Skip the check in unstable VBox version and if the extension pack
           is equal to or newer than VBox.

           Note! Use RTStrVersionCompare for the comparison here as it takes
                 the beta/alpha/preview/whatever tags into consideration when
                 comparing versions. */
        if (   vboxVersion.z() % 2 != 0
            || RTStrVersionCompare(abExtPackVersion.constData(), abVBoxVersion.constData()) >= 0)
        {
            emit sigStepComplete();
            return;
        }

        QString strExtPackEdition(extPack.GetEdition());
        if (strExtPackEdition.contains("ENTERPRISE"))
        {
            /* Inform the user that he should update the extension pack: */
            msgCenter().requestUserDownloadExtensionPack(GUI_ExtPackName, strExtPackVersion, strVBoxVersion);
            /* Never try to download for ENTERPRISE version: */
            emit sigStepComplete();
            return;
        }

        /* Ask the user about extension pack downloading: */
        if (!msgCenter().proposeDownloadExtensionPack(GUI_ExtPackName, strExtPackVersion))
        {
            emit sigStepComplete();
            return;
        }

        /* Create and configure the Extension Pack downloader: */
        UIDownloaderExtensionPack *pDl = UIDownloaderExtensionPack::create();
        /* After downloading finished => propose to install the Extension Pack: */
        connect(pDl, SIGNAL(sigDownloadFinished(const QString&, const QString&, QString)),
                this, SLOT(sltHandleDownloadedExtensionPack(const QString&, const QString&, QString)));
        /* Also, destroyed downloader is a signal to finish the step: */
        connect(pDl, SIGNAL(destroyed(QObject*)), this, SIGNAL(sigStepComplete()));
        /* Start downloading: */
        pDl->start();
    }

    /* Finishing slot: */
    void sltHandleDownloadedExtensionPack(const QString &strSource, const QString &strTarget, QString strDigest)
    {
        /* Warn the user about extension pack was downloaded and saved, propose to install it: */
        if (msgCenter().proposeInstallExtentionPack(GUI_ExtPackName, strSource, QDir::toNativeSeparators(strTarget)))
            UIGlobalSettingsExtension::doInstallation(strTarget, strDigest, msgCenter().mainWindowShown(), NULL);
    }
};

/* UIUpdateManager stuff: */
UIUpdateManager* UIUpdateManager::m_pInstance = 0;

/* static */
void UIUpdateManager::schedule()
{
    /* Ensure instance is NOT created: */
    if (m_pInstance)
        return;

    /* Create instance: */
    new UIUpdateManager;
}

/* static */
void UIUpdateManager::shutdown()
{
    /* Ensure instance is created: */
    if (!m_pInstance)
        return;

    /* Delete instance: */
    delete m_pInstance;
}

void UIUpdateManager::sltForceCheck()
{
    /* Force call for new version check: */
    sltCheckIfUpdateIsNecessary(true /* force call */);
}

UIUpdateManager::UIUpdateManager()
    : m_pQueue(new UIUpdateQueue(this))
    , m_fIsRunning(false)
    , m_uTime(1 /* day */ * 24 /* hours */ * 60 /* minutes */ * 60 /* seconds */ * 1000 /* ms */)
{
    /* Prepare instance: */
    if (m_pInstance != this)
        m_pInstance = this;

    /* Configure queue: */
    connect(m_pQueue, SIGNAL(sigQueueFinished()), this, SLOT(sltHandleUpdateFinishing()));

#ifdef VBOX_WITH_UPDATE_REQUEST
    /* Ask updater to check for the first time: */
    if (!vboxGlobal().isVMConsoleProcess())
        QTimer::singleShot(0, this, SLOT(sltCheckIfUpdateIsNecessary()));
#endif /* VBOX_WITH_UPDATE_REQUEST */
}

UIUpdateManager::~UIUpdateManager()
{
    /* Cleanup instance: */
    if (m_pInstance == this)
        m_pInstance = 0;
}

void UIUpdateManager::sltCheckIfUpdateIsNecessary(bool fForceCall /* = false */)
{
    /* If already running: */
    if (m_fIsRunning)
    {
        /* And we have a force-call: */
        if (fForceCall)
        {
            /* Just show Network Access Manager: */
            gNetworkManager->show();
        }
        return;
    }

    /* Set as running: */
    m_fIsRunning = true;

    /* Load/decode curent update data: */
    VBoxUpdateData currentData(vboxGlobal().virtualBox().GetExtraData(GUI_UpdateDate));

    /* If update is really necessary: */
    if (fForceCall || currentData.isNeedToCheck())
    {
        /* Prepare update queue: */
        new UIUpdateStepVirtualBox(m_pQueue, fForceCall);
        new UIUpdateStepVirtualBoxExtensionPack(m_pQueue, fForceCall);
        /* Start update queue: */
        m_pQueue->start();
    }
    else
        sltHandleUpdateFinishing();
}

void UIUpdateManager::sltHandleUpdateFinishing()
{
    /* Load/decode curent update data: */
    VBoxUpdateData currentData(vboxGlobal().virtualBox().GetExtraData(GUI_UpdateDate));
    /* Encode/save new update data: */
    VBoxUpdateData newData(currentData.periodIndex(), currentData.branchIndex());
    vboxGlobal().virtualBox().SetExtraData(GUI_UpdateDate, newData.data());

#ifdef VBOX_WITH_UPDATE_REQUEST
    /* Ask updater to check for the next time: */
    QTimer::singleShot(m_uTime, this, SLOT(sltCheckIfUpdateIsNecessary()));
#endif /* VBOX_WITH_UPDATE_REQUEST */

    /* Set as not running: */
    m_fIsRunning = false;
}

#include "UIUpdateManager.moc"

