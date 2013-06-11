/* $Id: VBoxVMInformationDlg.cpp $ */
/** @file
 *
 * VBox frontends: Qt4 GUI ("VirtualBox"):
 * VBoxVMInformationDlg class implementation
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

#ifdef VBOX_WITH_PRECOMPILED_HEADERS
# include "precomp.h"
#else  /* !VBOX_WITH_PRECOMPILED_HEADERS */

/* Qt includes: */
#include <QTimer>
#include <QScrollBar>

/* GUI includes: */
#include "UIIconPool.h"
#include "UIMachineLogic.h"
#include "UIMachineView.h"
#include "UIMachineWindow.h"
#include "UISession.h"
#include "VBoxGlobal.h"
#include "VBoxVMInformationDlg.h"
#include "UIConverter.h"

/* COM includes: */
#include "COMEnums.h"
#include "CConsole.h"
#include "CSystemProperties.h"
#include "CMachineDebugger.h"
#include "CDisplay.h"
#include "CGuest.h"
#include "CStorageController.h"
#include "CMediumAttachment.h"
#include "CNetworkAdapter.h"
#include "CVRDEServerInfo.h"

#endif /* !VBOX_WITH_PRECOMPILED_HEADERS */

VBoxVMInformationDlg::InfoDlgMap VBoxVMInformationDlg::mSelfArray = InfoDlgMap();

void VBoxVMInformationDlg::createInformationDlg(UIMachineWindow *pMachineWindow)
{
    CMachine machine = pMachineWindow->machineLogic()->uisession()->session().GetMachine();
    if (mSelfArray.find (machine.GetName()) == mSelfArray.end())
    {
        /* Creating new information dialog if there is no one existing */
        VBoxVMInformationDlg *id = new VBoxVMInformationDlg(pMachineWindow, Qt::Window);
        id->centerAccording (pMachineWindow);
        // TODO_NEW_CORE: this seems not necessary, cause we set WA_DeleteOnClose.
        id->setAttribute (Qt::WA_DeleteOnClose);
        mSelfArray [machine.GetName()] = id;
    }

    VBoxVMInformationDlg *info = mSelfArray [machine.GetName()];
    info->show();
    info->raise();
    info->setWindowState (info->windowState() & ~Qt::WindowMinimized);
    info->activateWindow();
}

VBoxVMInformationDlg::VBoxVMInformationDlg (UIMachineWindow *pMachineWindow, Qt::WindowFlags aFlags)
# ifdef Q_WS_MAC
    : QIWithRetranslateUI2 <QIMainDialog> (pMachineWindow, aFlags)
# else /* Q_WS_MAC */
    : QIWithRetranslateUI2 <QIMainDialog> (0, aFlags)
# endif /* Q_WS_MAC */
    , mSession (pMachineWindow->session())
    , mIsPolished (false)
    , mStatTimer (new QTimer (this))
{
    /* Apply UI decorations */
    Ui::VBoxVMInformationDlg::setupUi (this);

#ifdef Q_WS_MAC
    /* No icon for this window on the mac, cause this would act as proxy icon which isn't necessary here. */
    setWindowIcon(QIcon());
#else
    /* Apply window icons */
    setWindowIcon(UIIconPool::iconSetFull(QSize (32, 32), QSize (16, 16),
                                          ":/session_info_32px.png", ":/session_info_16px.png"));
#endif

    /* Enable size grip without using a status bar. */
    setSizeGripEnabled (true);

    /* Setup focus-proxy for pages */
    mPage1->setFocusProxy (mDetailsText);
    mPage2->setFocusProxy (mStatisticText);

    /* Setup browsers */
    mDetailsText->viewport()->setAutoFillBackground (false);
    mStatisticText->viewport()->setAutoFillBackground (false);

    /* Setup margins */
    mDetailsText->setViewportMargins (5, 5, 5, 5);
    mStatisticText->setViewportMargins (5, 5, 5, 5);

    /* Setup handlers */
    connect (pMachineWindow->uisession(), SIGNAL (sigMediumChange(const CMediumAttachment&)), this, SLOT (updateDetails()));
    connect (pMachineWindow->uisession(), SIGNAL (sigSharedFolderChange()), this, SLOT (updateDetails()));
    /* TODO_NEW_CORE: this is ofc not really right in the mm sense. There are
     * more than one screens. */
    connect (pMachineWindow->machineView(), SIGNAL (resizeHintDone()), this, SLOT (processStatistics()));
    connect (mInfoStack, SIGNAL (currentChanged (int)), this, SLOT (onPageChanged (int)));
    connect (&vboxGlobal(), SIGNAL (mediumEnumFinished (const VBoxMediaList &)), this, SLOT (updateDetails()));
    connect (mStatTimer, SIGNAL (timeout()), this, SLOT (processStatistics()));

    /* Loading language constants */
    retranslateUi();

    /* Details page update */
    updateDetails();

    /* Statistics page update */
    processStatistics();
    mStatTimer->start (5000);

    /* Preload dialog attributes for this vm */
    QString dlgsize = mSession.GetMachine().GetExtraData(GUI_InfoDlgState);
    if (dlgsize.isEmpty())
    {
        mWidth = 400;
        mHeight = 450;
        mMax = false;
    }
    else
    {
        QStringList list = dlgsize.split (',');
        mWidth = list [0].toInt(), mHeight = list [1].toInt();
        mMax = list [2] == "max";
    }

    /* Make statistics page the default one */
    mInfoStack->setCurrentIndex (1);
}

VBoxVMInformationDlg::~VBoxVMInformationDlg()
{
    /* Save dialog attributes for this vm */
    QString dlgsize ("%1,%2,%3");
    mSession.GetMachine().SetExtraData(GUI_InfoDlgState,
                                       dlgsize.arg(mWidth).arg(mHeight).arg(isMaximized() ? "max" : "normal"));

    if (!mSession.isNull() && !mSession.GetMachine().isNull())
        mSelfArray.remove (mSession.GetMachine().GetName());
}

void VBoxVMInformationDlg::retranslateUi()
{
    /* Translate uic generated strings */
    Ui::VBoxVMInformationDlg::retranslateUi (this);

    updateDetails();

    AssertReturnVoid (!mSession.isNull());
    CMachine machine = mSession.GetMachine();
    AssertReturnVoid (!machine.isNull());

    /* Setup a dialog caption */
    setWindowTitle (tr ("%1 - Session Information").arg (machine.GetName()));

    /* Setup a tabwidget page names */
    mInfoStack->setTabText (0, tr ("&Details"));
    mInfoStack->setTabText (1, tr ("&Runtime"));

    /* Clear counter names initially */
    mNamesMap.clear();
    mUnitsMap.clear();
    mLinksMap.clear();

    /* Storage statistics */
    CSystemProperties sp = vboxGlobal().virtualBox().GetSystemProperties();
    CStorageControllerVector controllers = mSession.GetMachine().GetStorageControllers();
    int ideCount = 0, sataCount = 0, scsiCount = 0;
    foreach (const CStorageController &controller, controllers)
    {
        switch (controller.GetBus())
        {
            case KStorageBus_IDE:
            {
                for (ULONG i = 0; i < sp.GetMaxPortCountForStorageBus (KStorageBus_IDE); ++ i)
                {
                    for (ULONG j = 0; j < sp.GetMaxDevicesPerPortForStorageBus (KStorageBus_IDE); ++ j)
                    {
                        /* Names */
                        mNamesMap [QString ("/Devices/IDE%1/ATA%2/Unit%3/*DMA")
                            .arg (ideCount).arg (i).arg (j)] = tr ("DMA Transfers");
                        mNamesMap [QString ("/Devices/IDE%1/ATA%2/Unit%3/*PIO")
                            .arg (ideCount).arg (i).arg (j)] = tr ("PIO Transfers");
                        mNamesMap [QString ("/Devices/IDE%1/ATA%2/Unit%3/ReadBytes")
                            .arg (ideCount).arg (i).arg (j)] = tr ("Data Read");
                        mNamesMap [QString ("/Devices/IDE%1/ATA%2/Unit%3/WrittenBytes")
                            .arg (ideCount).arg (i).arg (j)] = tr ("Data Written");

                        /* Units */
                        mUnitsMap [QString ("/Devices/IDE%1/ATA%2/Unit%3/*DMA")
                            .arg (ideCount).arg (i).arg (j)] = "[B]";
                        mUnitsMap [QString ("/Devices/IDE%1/ATA%2/Unit%3/*PIO")
                            .arg (ideCount).arg (i).arg (j)] = "[B]";
                        mUnitsMap [QString ("/Devices/IDE%1/ATA%2/Unit%3/ReadBytes")
                            .arg (ideCount).arg (i).arg (j)] = "B";
                        mUnitsMap [QString ("/Devices/IDE%1/ATA%2/Unit%3/WrittenBytes")
                            .arg (ideCount).arg (i).arg (j)] = "B";

                        /* Belongs to */
                        mLinksMap [QString ("/Devices/IDE%1/ATA%2/Unit%3").arg (ideCount).arg (i).arg (j)] = QStringList()
                                << QString ("/Devices/IDE%1/ATA%2/Unit%3/*DMA").arg (ideCount).arg (i).arg (j)
                                << QString ("/Devices/IDE%1/ATA%2/Unit%3/*PIO").arg (ideCount).arg (i).arg (j)
                                << QString ("/Devices/IDE%1/ATA%2/Unit%3/ReadBytes").arg (ideCount).arg (i).arg (j)
                                << QString ("/Devices/IDE%1/ATA%2/Unit%3/WrittenBytes").arg (ideCount).arg (i).arg (j);
                    }
                }
                ++ ideCount;
                break;
            }
            case KStorageBus_SATA:
            {
                for (ULONG i = 0; i < sp.GetMaxPortCountForStorageBus (KStorageBus_SATA); ++ i)
                {
                    for (ULONG j = 0; j < sp.GetMaxDevicesPerPortForStorageBus (KStorageBus_SATA); ++ j)
                    {
                        /* Names */
                        mNamesMap [QString ("/Devices/SATA%1/Port%2/DMA").arg (sataCount).arg (i)]
                            = tr ("DMA Transfers");
                        mNamesMap [QString ("/Devices/SATA%1/Port%2/ReadBytes").arg (sataCount).arg (i)]
                            = tr ("Data Read");
                        mNamesMap [QString ("/Devices/SATA%1/Port%2/WrittenBytes").arg (sataCount).arg (i)]
                            = tr ("Data Written");

                        /* Units */
                        mUnitsMap [QString ("/Devices/SATA%1/Port%2/DMA").arg (sataCount).arg (i)] = "[B]";
                        mUnitsMap [QString ("/Devices/SATA%1/Port%2/ReadBytes").arg (sataCount).arg (i)] = "B";
                        mUnitsMap [QString ("/Devices/SATA%1/Port%2/WrittenBytes").arg (sataCount).arg (i)] = "B";

                        /* Belongs to */
                        mLinksMap [QString ("/Devices/SATA%1/Port%2").arg (sataCount).arg (i)] = QStringList()
                                << QString ("/Devices/SATA%1/Port%2/DMA").arg (sataCount).arg (i)
                                << QString ("/Devices/SATA%1/Port%2/ReadBytes").arg (sataCount).arg (i)
                                << QString ("/Devices/SATA%1/Port%2/WrittenBytes").arg (sataCount).arg (i);
                    }
                }
                ++ sataCount;
                break;
            }
            case KStorageBus_SCSI:
            {
                for (ULONG i = 0; i < sp.GetMaxPortCountForStorageBus (KStorageBus_SCSI); ++ i)
                {
                    for (ULONG j = 0; j < sp.GetMaxDevicesPerPortForStorageBus (KStorageBus_SCSI); ++ j)
                    {
                        /* Names */
                        mNamesMap [QString ("/Devices/SCSI%1/%2/ReadBytes").arg (scsiCount).arg (i)]
                            = tr ("Data Read");
                        mNamesMap [QString ("/Devices/SCSI%1/%2/WrittenBytes").arg (scsiCount).arg (i)]
                            = tr ("Data Written");

                        /* Units */
                        mUnitsMap [QString ("/Devices/SCSI%1/%2/ReadBytes").arg (scsiCount).arg (i)] = "B";
                        mUnitsMap [QString ("/Devices/SCSI%1/%2/WrittenBytes").arg (scsiCount).arg (i)] = "B";

                        /* Belongs to */
                        mLinksMap [QString ("/Devices/SCSI%1/%2").arg (scsiCount).arg (i)] = QStringList()
                                << QString ("/Devices/SCSI%1/%2/ReadBytes").arg (scsiCount).arg (i)
                                << QString ("/Devices/SCSI%1/%2/WrittenBytes").arg (scsiCount).arg (i);
                    }
                }
                ++ scsiCount;
                break;
            }
            default:
                break;
        }
    }

    /* Network statistics: */
    ulong count = vboxGlobal().virtualBox().GetSystemProperties().GetMaxNetworkAdapters(KChipsetType_PIIX3);
    for (ulong i = 0; i < count; ++ i)
    {
        CNetworkAdapter na = machine.GetNetworkAdapter (i);
        KNetworkAdapterType ty = na.GetAdapterType();
        const char *name;

        switch (ty)
        {
            case KNetworkAdapterType_I82540EM:
            case KNetworkAdapterType_I82543GC:
            case KNetworkAdapterType_I82545EM:
                name = "E1k";
                break;
            case KNetworkAdapterType_Virtio:
                name = "VNet";
                break;
            default:
                name = "PCNet";
                break;
        }

        /* Names */
        mNamesMap [QString ("/Devices/%1%2/TransmitBytes")
            .arg (name).arg (i)] = tr ("Data Transmitted");
        mNamesMap [QString ("/Devices/%1%2/ReceiveBytes")
            .arg (name).arg (i)] = tr ("Data Received");

        /* Units */
        mUnitsMap [QString ("/Devices/%1%2/TransmitBytes")
            .arg (name).arg (i)] = "B";
        mUnitsMap [QString ("/Devices/%1%2/ReceiveBytes")
            .arg (name).arg (i)] = "B";

        /* Belongs to */
        mLinksMap [QString ("NA%1").arg (i)] = QStringList()
            << QString ("/Devices/%1%2/TransmitBytes").arg (name).arg (i)
            << QString ("/Devices/%1%2/ReceiveBytes").arg (name).arg (i);
    }

    /* Statistics page update. */
    refreshStatistics();
}

bool VBoxVMInformationDlg::event (QEvent *aEvent)
{
    bool result = QIMainDialog::event (aEvent);
    switch (aEvent->type())
    {
        case QEvent::WindowStateChange:
        {
            if (mIsPolished)
                mMax = isMaximized();
            else if (mMax == isMaximized())
                mIsPolished = true;
            break;
        }
        default:
            break;
    }
    return result;
}

void VBoxVMInformationDlg::resizeEvent (QResizeEvent *aEvent)
{
    QIMainDialog::resizeEvent (aEvent);

    /* Store dialog size for this vm */
    if (mIsPolished && !isMaximized())
    {
        mWidth = width();
        mHeight = height();
    }
}

void VBoxVMInformationDlg::showEvent (QShowEvent *aEvent)
{
    /* One may think that QWidget::polish() is the right place to do things
     * below, but apparently, by the time when QWidget::polish() is called,
     * the widget style & layout are not fully done, at least the minimum
     * size hint is not properly calculated. Since this is sometimes necessary,
     * we provide our own "polish" implementation */
    if (!mIsPolished)
    {
        /* Load window size and state */
        resize (mWidth, mHeight);
        if (mMax)
            QTimer::singleShot (0, this, SLOT (showMaximized()));
        else
            mIsPolished = true;
    }

    QIMainDialog::showEvent (aEvent);
}


void VBoxVMInformationDlg::updateDetails()
{
    /* Details page update */
    mDetailsText->setText (vboxGlobal().detailsReport (mSession.GetMachine(), false /* aWithLinks */));
}

void VBoxVMInformationDlg::processStatistics()
{
    CMachineDebugger dbg = mSession.GetConsole().GetDebugger();
    QString info;

    /* Process selected statistics: */
    for (DataMapType::const_iterator it = mNamesMap.begin(); it != mNamesMap.end(); ++ it)
    {
        dbg.GetStats (it.key(), true, info);
        mValuesMap [it.key()] = parseStatistics (info);
    }

    /* Statistics page update */
    refreshStatistics();
}

void VBoxVMInformationDlg::onPageChanged (int aIndex)
{
    /* Focusing the browser on shown page */
    mInfoStack->widget (aIndex)->setFocus();
}

QString VBoxVMInformationDlg::parseStatistics (const QString &aText)
{
    /* Filters the statistic counters body */
    QRegExp query ("^.+<Statistics>\n(.+)\n</Statistics>.*$");
    if (query.indexIn (aText) == -1)
        return QString::null;

    QStringList wholeList = query.cap (1).split ("\n");

    ULONG64 summa = 0;
    for (QStringList::Iterator lineIt = wholeList.begin(); lineIt != wholeList.end(); ++ lineIt)
    {
        QString text = *lineIt;
        text.remove (1, 1);
        text.remove (text.length() - 2, 2);

        /* Parse incoming counter and fill the counter-element values. */
        CounterElementType counter;
        counter.type = text.section (" ", 0, 0);
        text = text.section (" ", 1);
        QStringList list = text.split ("\" ");
        for (QStringList::Iterator it = list.begin(); it != list.end(); ++ it)
        {
            QString pair = *it;
            QRegExp regExp ("^(.+)=\"([^\"]*)\"?$");
            regExp.indexIn (pair);
            counter.list.insert (regExp.cap (1), regExp.cap (2));
        }

        /* Fill the output with the necessary counter's value.
         * Currently we are using "c" field of simple counter only. */
        QString result = counter.list.contains ("c") ? counter.list ["c"] : "0";
        summa += result.toULongLong();
    }

    return QString::number (summa);
}

void VBoxVMInformationDlg::refreshStatistics()
{
    if (mSession.isNull())
        return;

    QString table = "<table width=100% cellspacing=1 cellpadding=0>%1</table>";
    QString hdrRow = "<tr><td width=22><img src='%1'></td>"
                     "<td colspan=2><nobr><b>%2</b></nobr></td></tr>";
    QString paragraph = "<tr><td colspan=3></td></tr>";
    QString result;

    CMachine m = mSession.GetMachine();

    /* Runtime Information */
    {
        CConsole console = mSession.GetConsole();

        ULONG width = 0;
        ULONG height = 0;
        ULONG bpp = 0;
        console.GetDisplay().GetScreenResolution(0, width, height, bpp);
        QString resolution = QString ("%1x%2")
            .arg (width)
            .arg (height);
        if (bpp)
            resolution += QString ("x%1").arg (bpp);

        QString clipboardMode = gpConverter->toString(m.GetClipboardMode());
        QString dragAndDropMode = gpConverter->toString(m.GetDragAndDropMode());

        CMachineDebugger debugger = console.GetDebugger();
        QString virtualization = debugger.GetHWVirtExEnabled() ?
            VBoxGlobal::tr ("Enabled", "details report (VT-x/AMD-V)") :
            VBoxGlobal::tr ("Disabled", "details report (VT-x/AMD-V)");
        QString nested = debugger.GetHWVirtExNestedPagingEnabled() ?
            VBoxGlobal::tr ("Enabled", "details report (Nested Paging)") :
            VBoxGlobal::tr ("Disabled", "details report (Nested Paging)");

        CGuest guest = console.GetGuest();
        QString addVersionStr = guest.GetAdditionsVersion();
        if (addVersionStr.isEmpty())
            addVersionStr = tr("Not Detected", "guest additions");
        else
        {
            ULONG revision = guest.GetAdditionsRevision();
            if (revision != 0)
                addVersionStr += QString(" r%1").arg(revision);
        }
        QString osType = guest.GetOSTypeId();
        if (osType.isEmpty())
            osType = tr ("Not Detected", "guest os type");
        else
            osType = vboxGlobal().vmGuestOSTypeDescription (osType);

        int vrdePort = console.GetVRDEServerInfo().GetPort();
        QString vrdeInfo = (vrdePort == 0 || vrdePort == -1)?
            tr ("Not Available", "details report (VRDE server port)") :
            QString ("%1").arg (vrdePort);

        /* Searching for longest string */
        QStringList valuesList;
        valuesList << resolution << virtualization << nested << addVersionStr << osType << vrdeInfo;
        int maxLength = 0;
        foreach (const QString &value, valuesList)
            maxLength = maxLength < fontMetrics().width (value) ?
                        fontMetrics().width (value) : maxLength;

        result += hdrRow.arg (":/state_running_16px.png").arg (tr ("Runtime Attributes"));
        result += formatValue (tr ("Screen Resolution"), resolution, maxLength);
        result += formatValue (tr ("Clipboard Mode"), clipboardMode, maxLength);
        result += formatValue (tr ("Drag'n'Drop Mode"), dragAndDropMode, maxLength);
        result += formatValue (VBoxGlobal::tr ("VT-x/AMD-V", "details report"), virtualization, maxLength);
        result += formatValue (VBoxGlobal::tr ("Nested Paging", "details report"), nested, maxLength);
        result += formatValue (tr ("Guest Additions"), addVersionStr, maxLength);
        result += formatValue (tr ("Guest OS Type"), osType, maxLength);
        result += formatValue (VBoxGlobal::tr ("Remote Desktop Server Port", "details report (VRDE Server)"), vrdeInfo, maxLength);
        result += paragraph;
    }

    /* Storage statistics */
    {
        QString storageStat;

        result += hdrRow.arg (":/attachment_16px.png").arg (tr ("Storage Statistics"));

        CStorageControllerVector controllers = mSession.GetMachine().GetStorageControllers();
        int ideCount = 0, sataCount = 0, scsiCount = 0;
        foreach (const CStorageController &controller, controllers)
        {
            QString ctrName = controller.GetName();
            KStorageBus busType = controller.GetBus();
            CMediumAttachmentVector attachments = mSession.GetMachine().GetMediumAttachmentsOfController (ctrName);
            if (!attachments.isEmpty() && busType != KStorageBus_Floppy)
            {
                QString header = "<tr><td></td><td colspan=2><nobr>%1</nobr></td></tr>";
                QString strControllerName = QApplication::translate("UIMachineSettingsStorage", "Controller: %1");
                storageStat += header.arg(strControllerName.arg(controller.GetName()));
                int scsiIndex = 0;
                foreach (const CMediumAttachment &attachment, attachments)
                {
                    LONG attPort = attachment.GetPort();
                    LONG attDevice = attachment.GetDevice();
                    switch (busType)
                    {
                        case KStorageBus_IDE:
                        {
                            storageStat += formatMedium (ctrName, attPort, attDevice,
                                                         QString ("/Devices/IDE%1/ATA%2/Unit%3").arg (ideCount).arg (attPort).arg (attDevice));
                            break;
                        }
                        case KStorageBus_SATA:
                        {
                            storageStat += formatMedium (ctrName, attPort, attDevice,
                                                         QString ("/Devices/SATA%1/Port%2").arg (sataCount).arg (attPort));
                            break;
                        }
                        case KStorageBus_SCSI:
                        {
                            storageStat += formatMedium (ctrName, attPort, attDevice,
                                                         QString ("/Devices/SCSI%1/%2").arg (scsiCount).arg (scsiIndex));
                            ++ scsiIndex;
                            break;
                        }
                        default:
                            break;
                    }
                    storageStat += paragraph;
                }
            }

            switch (busType)
            {
                case KStorageBus_IDE:
                {
                    ++ ideCount;
                    break;
                }
                case KStorageBus_SATA:
                {
                    ++ sataCount;
                    break;
                }
                case KStorageBus_SCSI:
                {
                    ++ scsiCount;
                    break;
                }
                default:
                    break;
            }
        }

        /* If there are no Hard Disks */
        if (storageStat.isNull())
        {
            storageStat = composeArticle (tr ("No Storage Devices"));
            storageStat += paragraph;
        }

        result += storageStat;
    }

    /* Network Adapters Statistics */
    {
        QString networkStat;

        result += hdrRow.arg (":/nw_16px.png").arg (tr ("Network Statistics"));

        /* Network Adapters list */
        ulong count = vboxGlobal().virtualBox().GetSystemProperties().GetMaxNetworkAdapters(KChipsetType_PIIX3);
        for (ulong slot = 0; slot < count; ++ slot)
        {
            if (m.GetNetworkAdapter (slot).GetEnabled())
            {
                networkStat += formatAdapter (slot, QString ("NA%1").arg (slot));
                networkStat += paragraph;
            }
        }

        /* If there are no Network Adapters */
        if (networkStat.isNull())
        {
            networkStat = composeArticle (tr ("No Network Adapters"));
            networkStat += paragraph;
        }

        result += networkStat;
    }

    /* Show full composed page & save/restore scroll-bar position */
    int vv = mStatisticText->verticalScrollBar()->value();
    mStatisticText->setText (table.arg (result));
    mStatisticText->verticalScrollBar()->setValue (vv);
}

/**
 *  Allows left-aligned values formatting in right column.
 *
 *  aValueName - the name of value in the left column.
 *  aValue - left-aligned value itself in the right column.
 *  aMaxSize - maximum width (in pixels) of value in right column.
 */
QString VBoxVMInformationDlg::formatValue (const QString &aValueName,
                                           const QString &aValue, int aMaxSize)
{
    QString strMargin;
    int size = aMaxSize - fontMetrics().width(aValue);
    for (int i = 0; i < size; ++i)
        strMargin += QString("<img width=1 height=1 src=:/tpixel.png>");

    QString bdyRow = "<tr>"
                     "<td></td>"
                     "<td><nobr>%1</nobr></td>"
                     "<td align=right><nobr>%2%3</nobr></td>"
                     "</tr>";

    return bdyRow.arg (aValueName).arg (aValue).arg (strMargin);
}

QString VBoxVMInformationDlg::formatMedium (const QString &aCtrName,
                                            LONG aPort, LONG aDevice,
                                            const QString &aBelongsTo)
{
    if (mSession.isNull())
        return QString::null;

    QString header = "<tr><td></td><td colspan=2><nobr>&nbsp;&nbsp;%1:</nobr></td></tr>";
    CStorageController ctr = mSession.GetMachine().GetStorageControllerByName (aCtrName);
    QString name = gpConverter->toString (StorageSlot (ctr.GetBus(), aPort, aDevice));
    return header.arg (name) + composeArticle (aBelongsTo, 2);
}

QString VBoxVMInformationDlg::formatAdapter (ULONG aSlot,
                                             const QString &aBelongsTo)
{
    if (mSession.isNull())
        return QString::null;

    QString header = "<tr><td></td><td colspan=2><nobr>%1</nobr></td></tr>";
    QString name = VBoxGlobal::tr ("Adapter %1", "details report (network)").arg (aSlot + 1);
    return header.arg (name) + composeArticle (aBelongsTo, 1);
}

QString VBoxVMInformationDlg::composeArticle (const QString &aBelongsTo, int aSpacesCount)
{
    QString body = "<tr><td></td><td width=50%><nobr>%1%2</nobr></td>"
                   "<td align=right><nobr>%3%4</nobr></td></tr>";
    QString indent;
    for (int i = 0; i < aSpacesCount; ++ i)
        indent += "&nbsp;&nbsp;";
    body = body.arg (indent);

    QString result;

    if (mLinksMap.contains (aBelongsTo))
    {
        QStringList keys = mLinksMap [aBelongsTo];
        foreach (const QString &key, keys)
        {
            QString line (body);
            if (mNamesMap.contains (key) && mValuesMap.contains (key) && mUnitsMap.contains (key))
            {
                line = line.arg (mNamesMap [key]).arg (QString ("%L1").arg (mValuesMap [key].toULongLong()));
                line = mUnitsMap [key].contains (QRegExp ("\\[\\S+\\]")) ?
                    line.arg (QString ("<img src=:/tpixel.png width=%1 height=1>")
                              .arg (QApplication::fontMetrics().width (
                              QString (" %1").arg (mUnitsMap [key]
                              .mid (1, mUnitsMap [key].length() - 2))))) :
                    line.arg (QString (" %1").arg (mUnitsMap [key]));
                result += line;
            }
        }
    }
    else
        result = body.arg (aBelongsTo).arg (QString::null).arg (QString::null);

    return result;
}
