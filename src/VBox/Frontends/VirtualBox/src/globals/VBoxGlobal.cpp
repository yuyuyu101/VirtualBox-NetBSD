/* $Id: VBoxGlobal.cpp $ */
/** @file
 * VBox Qt GUI - VBoxGlobal class implementation.
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
#include <QProgressDialog>
#include <QLibraryInfo>
#include <QFileDialog>
#include <QToolTip>
#include <QTranslator>
#include <QDesktopWidget>
#include <QDesktopServices>
#include <QMutex>
#include <QToolButton>
#include <QProcess>
#include <QThread>
#include <QPainter>
#include <QSettings>
#include <QTimer>
#include <QDir>
#include <QLocale>
#include <QNetworkProxy>

#ifdef Q_WS_WIN
# include <QEventLoop>
#endif /* Q_WS_WIN */

#ifdef Q_WS_X11
# include <QTextBrowser>
# include <QScrollBar>
# include <QX11Info>
#endif /* Q_WS_X11 */

#ifdef VBOX_GUI_WITH_PIDFILE
# include <QTextStream>
#endif /* VBOX_GUI_WITH_PIDFILE */

/* GUI includes: */
#include "VBoxGlobal.h"
#include "VBoxUtils.h"
#include "UISelectorWindow.h"
#include "UIMessageCenter.h"
#include "QIMessageBox.h"
#include "QIDialogButtonBox.h"
#include "UIIconPool.h"
#include "UIActionPoolSelector.h"
#include "UIActionPoolRuntime.h"
#include "UIExtraDataEventHandler.h"
#include "QIFileDialog.h"
#include "UINetworkManager.h"
#include "UIUpdateManager.h"
#include "UIMachine.h"
#include "UISession.h"
#include "UIConverter.h"

#ifdef Q_WS_X11
# include "UIHotKeyEditor.h"
# ifndef VBOX_OSE
#  include "VBoxLicenseViewer.h"
# endif /* VBOX_OSE */
# include "VBoxX11Helper.h"
#endif /* Q_WS_X11 */

#ifdef Q_WS_MAC
# include "VBoxUtils-darwin.h"
# include "UIMachineWindowFullscreen.h"
# include "UIMachineWindowSeamless.h"
#endif /* Q_WS_MAC */

#ifdef VBOX_WITH_VIDEOHWACCEL
# include "VBoxFBOverlay.h"
#endif /* VBOX_WITH_VIDEOHWACCEL */

#ifdef VBOX_WITH_REGISTRATION
# include "UIRegistrationWzd.h"
#endif /* VBOX_WITH_REGISTRATION */

#ifdef VBOX_GUI_WITH_SYSTRAY
#include <iprt/process.h>
#if defined(RT_OS_WINDOWS) || defined(RT_OS_OS2)
#define HOSTSUFF_EXE ".exe"
#else /* !RT_OS_WINDOWS */
#define HOSTSUFF_EXE ""
#endif /* !RT_OS_WINDOWS */
#endif /* VBOX_GUI_WITH_SYSTRAY */

/* COM includes: */
#include "CMachine.h"
#include "CSystemProperties.h"
#include "CUSBDevice.h"
#include "CUSBDeviceFilter.h"
#include "CBIOSSettings.h"
#include "CVRDEServer.h"
#include "CStorageController.h"
#include "CMediumAttachment.h"
#include "CAudioAdapter.h"
#include "CNetworkAdapter.h"
#include "CSerialPort.h"
#include "CParallelPort.h"
#include "CUSBController.h"
#include "CHostUSBDevice.h"
#include "CMediumFormat.h"
#include "CSharedFolder.h"

/* Other VBox includes: */
#include <iprt/asm.h>
#include <iprt/ctype.h>
#include <iprt/err.h>
#include <iprt/param.h>
#include <iprt/path.h>
#include <iprt/env.h>
#include <iprt/file.h>
#include <iprt/ldr.h>
#include <iprt/system.h>
#include <iprt/stream.h>

#include <VBox/vd.h>
#include <VBox/sup.h>
#include <VBox/com/Guid.h>
#include <VBox/VBoxOGLTest.h>

#ifdef Q_WS_X11
#include <iprt/mem.h>
#endif /* Q_WS_X11 */

/* External includes: */
#include <math.h>

#ifdef Q_WS_WIN
#include "shlobj.h"
#endif /* Q_WS_WIN */

#ifdef Q_WS_X11
#undef BOOL /* typedef CARD8 BOOL in Xmd.h conflicts with #define BOOL PRBool
             * in COMDefs.h. A better fix would be to isolate X11-specific
             * stuff by placing XX* helpers below to a separate source file. */
#include <X11/X.h>
#include <X11/Xmd.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/extensions/Xinerama.h>
#define BOOL PRBool
#endif /* Q_WS_X11 */

//#define VBOX_WITH_FULL_DETAILS_REPORT /* hidden for now */

//#warning "port me: check this"
/// @todo bird: Use (U)INT_PTR, (U)LONG_PTR, DWORD_PTR, or (u)intptr_t.
#if defined(Q_OS_WIN64)
typedef __int64 Q_LONG;             /* word up to 64 bit signed */
typedef unsigned __int64 Q_ULONG;   /* word up to 64 bit unsigned */
#else
typedef long Q_LONG;                /* word up to 64 bit signed */
typedef unsigned long Q_ULONG;      /* word up to 64 bit unsigned */
#endif

// VBoxMediaEnumEvent
/////////////////////////////////////////////////////////////////////////////

class VBoxMediaEnumEvent : public QEvent
{
public:

    /** Constructs a regular enum event */
    VBoxMediaEnumEvent (const UIMedium &aMedium,
                        const VBoxMediaList::iterator &aIterator)
        : QEvent ((QEvent::Type) MediaEnumEventType)
        , mMedium (aMedium), mIterator (aIterator), mLast (false)
        {}
    /** Constructs the last enum event */
    VBoxMediaEnumEvent (const VBoxMediaList::iterator &aIterator)
        : QEvent ((QEvent::Type) MediaEnumEventType)
        , mIterator (aIterator), mLast (true)
        {}

    /** Last enumerated medium (not valid when #last is true) */
    const UIMedium mMedium;
    /* Iterator which points to the corresponding item in the GUI thread: */
    const VBoxMediaList::iterator mIterator;
    /** Whether this is the last event for the given enumeration or not */
    const bool mLast;
};

// VBoxGlobal
////////////////////////////////////////////////////////////////////////////////

static bool sVBoxGlobalInited = false;
static bool sVBoxGlobalInCleanup = false;

/** @internal
 *
 *  Special routine to do VBoxGlobal cleanup when the application is being
 *  terminated. It is called before some essential Qt functionality (for
 *  instance, QThread) becomes unavailable, allowing us to use it from
 *  VBoxGlobal::cleanup() if necessary.
 */
static void vboxGlobalCleanup()
{
    Assert (!sVBoxGlobalInCleanup);
    sVBoxGlobalInCleanup = true;
    vboxGlobal().cleanup();
}

/** @internal
 *
 *  Determines the rendering mode from the argument. Sets the appropriate
 *  default rendering mode if the argument is NULL.
 */
static RenderMode vboxGetRenderMode (const char *aModeStr)
{
    RenderMode mode = InvalidRenderMode;

#if defined (Q_WS_MAC) && defined (VBOX_GUI_USE_QUARTZ2D)
    mode = Quartz2DMode;
#elif (defined (Q_WS_WIN32) || defined (Q_WS_PM) || defined (Q_WS_X11)) && defined (VBOX_GUI_USE_QIMAGE)
    mode = QImageMode;
#elif defined (Q_WS_X11) && defined (VBOX_GUI_USE_SDL)
    mode = SDLMode;
#elif defined (VBOX_GUI_USE_QIMAGE)
    mode = QImageMode;
#else
# error "Cannot determine the default render mode!"
#endif

    if (aModeStr)
    {
        if (0) ;
#if defined (VBOX_GUI_USE_QIMAGE)
        else if (::strcmp (aModeStr, "image") == 0)
            mode = QImageMode;
#endif
#if defined (VBOX_GUI_USE_SDL)
        else if (::strcmp (aModeStr, "sdl") == 0)
            mode = SDLMode;
#endif
#if defined (VBOX_GUI_USE_DDRAW)
        else if (::strcmp (aModeStr, "ddraw") == 0)
            mode = DDRAWMode;
#endif
#if defined (VBOX_GUI_USE_QUARTZ2D)
        else if (::strcmp (aModeStr, "quartz2d") == 0)
            mode = Quartz2DMode;
#endif
#if defined (VBOX_GUI_USE_QGLFB)
        else if (::strcmp (aModeStr, "qgl") == 0)
            mode = QGLMode;
#endif
//#if defined (VBOX_GUI_USE_QGL)
//        else if (::strcmp (aModeStr, "qgloverlay") == 0)
//            mode = QGLOverlayMode;
//#endif

    }

    return mode;
}

/** @class VBoxGlobal
 *
 *  The VBoxGlobal class encapsulates the global VirtualBox data.
 *
 *  There is only one instance of this class per VirtualBox application,
 *  the reference to it is returned by the static instance() method, or by
 *  the global vboxGlobal() function, that is just an inlined shortcut.
 */

VBoxGlobal::VBoxGlobal()
    : mValid (false)
    , mSelectorWnd (NULL)
    , m_pVirtualMachine(0)
    , mMainWindow (NULL)
#ifdef VBOX_WITH_REGISTRATION
    , mRegDlg (NULL)
#endif
#ifdef VBOX_GUI_WITH_SYSTRAY
    , mIsTrayMenu (false)
    , mIncreasedWindowCounter (false)
#endif
    , mMediaEnumThread (NULL)
    , mIsKWinManaged (false)
    , mDisablePatm(false)
    , mDisableCsam(false)
    , mRecompileSupervisor(false)
    , mRecompileUser(false)
    , mWarpPct(100)
    , mVerString("1.0")
    , m3DAvailable(false)
    , mSettingsPwSet(false)
{
}

//
// Public members
/////////////////////////////////////////////////////////////////////////////

/**
 *  Returns a reference to the global VirtualBox data, managed by this class.
 *
 *  The main() function of the VBox GUI must call this function soon after
 *  creating a QApplication instance but before opening any of the main windows
 *  (to let the VBoxGlobal initialization procedure use various Qt facilities),
 *  and continue execution only when the isValid() method of the returned
 *  instancereturns true, i.e. do something like:
 *
 *  @code
 *  if ( !VBoxGlobal::instance().isValid() )
 *      return 1;
 *  @endcode
 *  or
 *  @code
 *  if ( !vboxGlobal().isValid() )
 *      return 1;
 *  @endcode
 *
 *  @note Some VBoxGlobal methods can be used on a partially constructed
 *  VBoxGlobal instance, i.e. from constructors and methods called
 *  from the VBoxGlobal::init() method, which obtain the instance
 *  using this instance() call or the ::vboxGlobal() function. Currently, such
 *  methods are:
 *      #vmStateText, #vmTypeIcon, #vmTypeText, #vmTypeTextList, #vmTypeFromText.
 *
 *  @see ::vboxGlobal
 */
VBoxGlobal &VBoxGlobal::instance()
{
    static VBoxGlobal vboxGlobal_instance;

    if (!sVBoxGlobalInited)
    {
        /* check that a QApplication instance is created */
        if (qApp)
        {
            sVBoxGlobalInited = true;
            vboxGlobal_instance.init();
            /* add our cleanup handler to the list of Qt post routines */
            qAddPostRoutine (vboxGlobalCleanup);
        }
        else
            AssertMsgFailed (("Must construct a QApplication first!"));
    }
    return vboxGlobal_instance;
}

VBoxGlobal::~VBoxGlobal()
{
    qDeleteAll (mOsTypeIcons);
}

/* static */
QString VBoxGlobal::qtRTVersionString()
{
    return QString::fromLatin1 (qVersion());
}

/* static */
uint VBoxGlobal::qtRTVersion()
{
    QString rt_ver_str = VBoxGlobal::qtRTVersionString();
    return (rt_ver_str.section ('.', 0, 0).toInt() << 16) +
           (rt_ver_str.section ('.', 1, 1).toInt() << 8) +
           rt_ver_str.section ('.', 2, 2).toInt();
}

/* static */
QString VBoxGlobal::qtCTVersionString()
{
    return QString::fromLatin1 (QT_VERSION_STR);
}

/* static */
uint VBoxGlobal::qtCTVersion()
{
    QString ct_ver_str = VBoxGlobal::qtCTVersionString();
    return (ct_ver_str.section ('.', 0, 0).toInt() << 16) +
           (ct_ver_str.section ('.', 1, 1).toInt() << 8) +
           ct_ver_str.section ('.', 2, 2).toInt();
}

QString VBoxGlobal::vboxVersionString() const
{
    return mVBox.GetVersion();
}

QString VBoxGlobal::vboxVersionStringNormalized() const
{
    return mVBox.GetVersionNormalized();
}

bool VBoxGlobal::isBeta() const
{
    return mVBox.GetVersion().contains("BETA", Qt::CaseInsensitive);
}

/**
 *  Sets the new global settings and saves them to the VirtualBox server.
 */
bool VBoxGlobal::setSettings (VBoxGlobalSettings &gs)
{
    gs.save (mVBox);

    if (!mVBox.isOk())
    {
        msgCenter().cannotSaveGlobalConfig (mVBox);
        return false;
    }

    /* We don't assign gs to our gset member here, because VBoxCallback
     * will update gset as necessary when new settings are successfully
     * sent to the VirtualBox server by gs.save(). */

    return true;
}

/**
 *  Returns a reference to the main VBox VM Selector window.
 *  The reference is valid until application termination.
 *
 *  There is only one such a window per VirtualBox application.
 */
UISelectorWindow &VBoxGlobal::selectorWnd()
{
    AssertMsg (!vboxGlobal().isVMConsoleProcess(),
               ("Must NOT be a VM console process"));
    Assert (mValid);

    if (!mSelectorWnd)
    {
        /*
         *  We pass the address of mSelectorWnd to the constructor to let it be
         *  initialized right after the constructor is called. It is necessary
         *  to avoid recursion, since this method may be (and will be) called
         *  from the below constructor or from constructors/methods it calls.
         */
        UISelectorWindow *w = new UISelectorWindow (&mSelectorWnd, 0);
        Assert (w == mSelectorWnd);
        NOREF(w);
    }

    return *mSelectorWnd;
}

bool VBoxGlobal::startMachine(const QString &strMachineId)
{
    /* Some restrictions: */
    AssertMsg(mValid, ("VBoxGlobal is invalid"));
    AssertMsg(!m_pVirtualMachine, ("Machine already started"));

    /* Create VM session: */
    CSession session = vboxGlobal().openSession(strMachineId, KLockType_VM);
    if (session.isNull())
        return false;

    /* Start virtual machine: */
    UIMachine *pVirtualMachine = new UIMachine(&m_pVirtualMachine, session);
    Assert(pVirtualMachine == m_pVirtualMachine);
    Q_UNUSED(pVirtualMachine);
    return true;
}

UIMachine* VBoxGlobal::virtualMachine()
{
    return m_pVirtualMachine;
}

QWidget* VBoxGlobal::vmWindow()
{
    if (isVMConsoleProcess() && m_pVirtualMachine)
        return m_pVirtualMachine->mainWindow();
    return 0;
}

#ifdef VBOX_GUI_WITH_PIDFILE
void VBoxGlobal::createPidfile()
{
    if (!m_strPidfile.isEmpty())
    {
        qint64 pid = qApp->applicationPid();
        QFile file(m_strPidfile);
        if (file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        {
             QTextStream out(&file);
             out << pid << endl;
        }
        else
            LogRel(("Failed to create pid file %s\n", m_strPidfile.toUtf8().constData()));
    }
}

void VBoxGlobal::deletePidfile()
{
    if (   !m_strPidfile.isEmpty()
        && QFile::exists(m_strPidfile))
        QFile::remove(m_strPidfile);
}
#endif

bool VBoxGlobal::brandingIsActive (bool aForce /* = false*/)
{
    if (aForce)
        return true;

    if (mBrandingConfig.isEmpty())
    {
        mBrandingConfig = QDir(QApplication::applicationDirPath()).absolutePath();
        mBrandingConfig += "/custom/custom.ini";
    }
    return QFile::exists (mBrandingConfig);
}

/**
  * Gets a value from the custom .ini file
  */
QString VBoxGlobal::brandingGetKey (QString aKey)
{
    QSettings s(mBrandingConfig, QSettings::IniFormat);
    return s.value(QString("%1").arg(aKey)).toString();
}

#ifdef VBOX_GUI_WITH_SYSTRAY

/**
 *  Returns true if the current instance a systray menu only (started with
 *  "-systray" parameter).
 */
bool VBoxGlobal::isTrayMenu() const
{
    return mIsTrayMenu;
}

void VBoxGlobal::setTrayMenu(bool aIsTrayMenu)
{
    mIsTrayMenu = aIsTrayMenu;
}

/**
 *  Spawns a new selector window (process).
 */
void VBoxGlobal::trayIconShowSelector()
{
    /* Get the path to the executable. */
    char path[RTPATH_MAX];
    RTPathAppPrivateArch(path, RTPATH_MAX);
    size_t sz = strlen(path);
    path[sz++] = RTPATH_DELIMITER;
    path[sz] = 0;
    char *cmd = path + sz;
    sz = RTPATH_MAX - sz;

    int rc = 0;
    const char VirtualBox_exe[] = "VirtualBox" HOSTSUFF_EXE;
    Assert(sz >= sizeof(VirtualBox_exe));
    strcpy(cmd, VirtualBox_exe);
    const char * args[] = {path, 0 };
    rc = RTProcCreate(path, args, RTENV_DEFAULT, RTPROC_FLAGS_DETACHED, NULL);
    if (RT_FAILURE(rc))
        LogRel(("Systray: Failed to start new selector window! Path=%s, rc=%Rrc\n", path, rc));
}

/**
 *  Tries to install the tray icon using the current instance (singleton).
 *  Returns true if this instance is the tray icon, false if not.
 */
bool VBoxGlobal::trayIconInstall()
{
    int rc = 0;
    QString strTrayWinID = mVBox.GetExtraData(GUI_TrayIconWinID);
    if (false == strTrayWinID.isEmpty())
    {
        /* Check if current tray icon is alive by writing some bogus value. */
        mVBox.SetExtraData(GUI_TrayIconWinID, "0");
        if (mVBox.isOk())
        {
            /* Current tray icon died - clean up. */
            mVBox.SetExtraData(GUI_TrayIconWinID, NULL);
            strTrayWinID.clear();
        }
    }

    /* Is there already a tray icon or is tray icon not active? */
    if (   (mIsTrayMenu == false)
        && (vboxGlobal().settings().trayIconEnabled())
        && (QSystemTrayIcon::isSystemTrayAvailable())
        && (strTrayWinID.isEmpty()))
    {
        /* Get the path to the executable. */
        char path[RTPATH_MAX];
        RTPathAppPrivateArch(path, RTPATH_MAX);
        size_t sz = strlen(path);
        path[sz++] = RTPATH_DELIMITER;
        path[sz] = 0;
        char *cmd = path + sz;
        sz = RTPATH_MAX - sz;

        const char VirtualBox_exe[] = "VirtualBox" HOSTSUFF_EXE;
        Assert(sz >= sizeof(VirtualBox_exe));
        strcpy(cmd, VirtualBox_exe);
        const char * args[] = {path, "-systray", 0 };
        rc = RTProcCreate(path, args, RTENV_DEFAULT, RTPROC_FLAGS_DETACHED, NULL);
        if (RT_FAILURE(rc))
        {
            LogRel(("Systray: Failed to start systray window! Path=%s, rc=%Rrc\n", path, rc));
            return false;
        }
    }

    if (mIsTrayMenu)
    {
        // Use this selector for displaying the tray icon
        mVBox.SetExtraData(GUI_TrayIconWinID,
                           QString("%1").arg((qulonglong)vboxGlobal().mainWindow()->winId()));

        /* The first process which can grab this "mutex" will win ->
         * It will be the tray icon menu then. */
        if (mVBox.isOk())
        {
            emit sigTrayIconShow(true);
            return true;
        }
    }

    return false;
}

#endif

#ifdef Q_WS_X11
QList<QRect> XGetDesktopList()
{
    /* Prepare empty resulting list: */
    QList<QRect> result;

    /* Get current display: */
    Display* pDisplay = QX11Info::display();

    /* If it's a Xinerama desktop: */
    if (XineramaIsActive(pDisplay))
    {
        /* Reading Xinerama data: */
        int iScreens = 0;
        XineramaScreenInfo *pScreensData = XineramaQueryScreens(pDisplay, &iScreens);

        /* Fill resulting list: */
        for (int i = 0; i < iScreens; ++ i)
            result << QRect(pScreensData[i].x_org, pScreensData[i].y_org,
                            pScreensData[i].width, pScreensData[i].height);

        /* Free screens data: */
        XFree(pScreensData);
    }

    /* Return resulting list: */
    return result;
}

QList<Window> XGetWindowIDList()
{
    /* Get current display: */
    Display *pDisplay = QX11Info::display();

    /* Get virtual desktop window: */
    Window window = QX11Info::appRootWindow();

    /* Get 'client list' atom: */
    Atom propNameAtom = XInternAtom(pDisplay, "_NET_CLIENT_LIST", True /* only if exists */);

    /* Prepare empty resulting list: */
    QList<Window> result;

    /* If atom does not exists return empty list: */
    if (propNameAtom == None)
        return result;

    /* Get atom value: */
    Atom realAtomType = None;
    int iRealFormat = 0;
    unsigned long uItemsCount = 0;
    unsigned long uBytesAfter = 0;
    unsigned char *pData = 0;
    int rc = XGetWindowProperty(pDisplay, window, propNameAtom,
                                0, 0x7fffffff /*LONG_MAX*/, False /* delete */,
                                XA_WINDOW, &realAtomType, &iRealFormat,
                                &uItemsCount, &uBytesAfter, &pData);

    /* If get property is failed return empty list: */
    if (rc != Success)
        return result;

    /* Fill resulting list with win ids: */
    Window *pWindowData = reinterpret_cast<Window*>(pData);
    for (ulong i = 0; i < uItemsCount; ++ i)
        result << pWindowData[i];

    /* Releasing resources: */
    XFree(pData);

    /* Return resulting list: */
    return result;
}

QList<ulong> XGetStrut(Window window)
{
    /* Get current display: */
    Display *pDisplay = QX11Info::display();

    /* Get 'strut' atom: */
    Atom propNameAtom = XInternAtom(pDisplay, "_NET_WM_STRUT_PARTIAL", True /* only if exists */);

    /* Prepare empty resulting list: */
    QList<ulong> result;

    /* If atom does not exists return empty list: */
    if (propNameAtom == None)
        return result;

    /* Get atom value: */
    Atom realAtomType = None;
    int iRealFormat = 0;
    ulong uItemsCount = 0;
    ulong uBytesAfter = 0;
    unsigned char *pData = 0;
    int rc = XGetWindowProperty(pDisplay, window, propNameAtom,
                                0, LONG_MAX, False /* delete */,
                                XA_CARDINAL, &realAtomType, &iRealFormat,
                                &uItemsCount, &uBytesAfter, &pData);

    /* If get property is failed return empty list: */
    if (rc != Success)
        return result;

    /* Fill resulting list with strut shifts: */
    ulong *pStrutsData = reinterpret_cast<ulong*>(pData);
    for (ulong i = 0; i < uItemsCount; ++ i)
        result << pStrutsData[i];

    /* Releasing resources: */
    XFree(pData);

    /* Return resulting list: */
    return result;
}
#endif /* ifdef Q_WS_X11 */

const QRect VBoxGlobal::availableGeometry(int iScreen) const
{
    /* Prepare empty result: */
    QRect result;

#ifdef Q_WS_X11

    /* Get current display: */
    Display* pDisplay = QX11Info::display();

    /* Get current application desktop: */
    QDesktopWidget *pDesktopWidget = QApplication::desktop();

    /* If it's a virtual desktop: */
    if (pDesktopWidget->isVirtualDesktop())
    {
        /* If it's a Xinerama desktop: */
        if (XineramaIsActive(pDisplay))
        {
            /* Get desktops list: */
            QList<QRect> desktops = XGetDesktopList();

            /* Combine to get full virtual region: */
            QRegion virtualRegion;
            foreach (QRect desktop, desktops)
                virtualRegion += desktop;
            virtualRegion = virtualRegion.boundingRect();

            /* Remember initial virtual desktop: */
            QRect virtualDesktop = virtualRegion.boundingRect();
            //AssertMsgFailed(("LOG... Virtual desktop is: %dx%dx%dx%d\n", virtualDesktop.x(), virtualDesktop.y(),
            //                                                             virtualDesktop.width(), virtualDesktop.height()));

            /* Set available geometry to screen geometry initially: */
            result = desktops[iScreen];

            /* Feat available geometry of virtual desktop to respect all the struts: */
            QList<Window> list = XGetWindowIDList();
            for (int i = 0; i < list.size(); ++ i)
            {
                /* Get window: */
                Window wid = list[i];
                QList<ulong> struts = XGetStrut(wid);

                /* If window has strut: */
                if (struts.size())
                {
                    ulong uLeftShift = struts[0];
                    ulong uLeftFromY = struts[4];
                    ulong uLeftToY = struts[5];

                    ulong uRightShift = struts[1];
                    ulong uRightFromY = struts[6];
                    ulong uRightToY = struts[7];

                    ulong uTopShift = struts[2];
                    ulong uTopFromX = struts[8];
                    ulong uTopToX = struts[9];

                    ulong uBottomShift = struts[3];
                    ulong uBottomFromX = struts[10];
                    ulong uBottomToX = struts[11];

                    if (uLeftShift)
                    {
                        QRect sr(QPoint(0, uLeftFromY),
                                 QSize(uLeftShift, uLeftToY - uLeftFromY + 1));

                        //AssertMsgFailed(("LOG... Subtract left strut: top-left: %dx%d, size: %dx%d\n", sr.x(), sr.y(), sr.width(), sr.height()));
                        virtualRegion -= sr;
                    }

                    if (uRightShift)
                    {
                        QRect sr(QPoint(virtualDesktop.x() + virtualDesktop.width() - uRightShift, uRightFromY),
                                 QSize(virtualDesktop.x() + virtualDesktop.width(), uRightToY - uRightFromY + 1));

                        //AssertMsgFailed(("LOG... Subtract right strut: top-left: %dx%d, size: %dx%d\n", sr.x(), sr.y(), sr.width(), sr.height()));
                        virtualRegion -= sr;
                    }

                    if (uTopShift)
                    {
                        QRect sr(QPoint(uTopFromX, 0),
                                 QSize(uTopToX - uTopFromX + 1, uTopShift));

                        //AssertMsgFailed(("LOG... Subtract top strut: top-left: %dx%d, size: %dx%d\n", sr.x(), sr.y(), sr.width(), sr.height()));
                        virtualRegion -= sr;
                    }

                    if (uBottomShift)
                    {
                        QRect sr(QPoint(uBottomFromX, virtualDesktop.y() + virtualDesktop.height() - uBottomShift),
                                 QSize(uBottomToX - uBottomFromX + 1, uBottomShift));

                        //AssertMsgFailed(("LOG... Subtract bottom strut: top-left: %dx%d, size: %dx%d\n", sr.x(), sr.y(), sr.width(), sr.height()));
                        virtualRegion -= sr;
                    }
                }
            }

            /* Get final available geometry: */
            result = (virtualRegion & result).boundingRect();
        }
    }

    /* If result is still NULL: */
    if (result.isNull())
    {
        /* Use QT default functionality: */
        result = pDesktopWidget->availableGeometry(iScreen);
    }

    //AssertMsgFailed(("LOG... Final geometry: %dx%dx%dx%d\n", result.x(), result.y(), result.width(), result.height()));

#else /* ifdef Q_WS_X11 */

    result = QApplication::desktop()->availableGeometry(iScreen);

#endif /* ifndef Q_WS_X11 */

    return result;
}

/**
 *  Returns the list of few guest OS types, queried from
 *  IVirtualBox corresponding to every family id.
 */
QList <CGuestOSType> VBoxGlobal::vmGuestOSFamilyList() const
{
    QList <CGuestOSType> result;
    for (int i = 0 ; i < mFamilyIDs.size(); ++ i)
        result << mTypes [i][0];
    return result;
}

/**
 *  Returns the list of all guest OS types, queried from
 *  IVirtualBox corresponding to passed family id.
 */
QList <CGuestOSType> VBoxGlobal::vmGuestOSTypeList (const QString &aFamilyId) const
{
    AssertMsg (mFamilyIDs.contains (aFamilyId), ("Family ID incorrect: '%s'.", aFamilyId.toLatin1().constData()));
    return mFamilyIDs.contains (aFamilyId) ?
           mTypes [mFamilyIDs.indexOf (aFamilyId)] : QList <CGuestOSType>();
}

/**
 *  Returns the icon corresponding to the given guest OS type id.
 */
QPixmap VBoxGlobal::vmGuestOSTypeIcon (const QString &aTypeId) const
{
    static const QPixmap none;
    QPixmap *p = mOsTypeIcons.value (aTypeId);
    AssertMsg (p, ("Icon for type '%s' must be defined.", aTypeId.toLatin1().constData()));
    return p ? *p : none;
}

/**
 *  Returns the guest OS type object corresponding to the given type id of list
 *  containing OS types related to OS family determined by family id attribute.
 *  If the index is invalid a null object is returned.
 */
CGuestOSType VBoxGlobal::vmGuestOSType (const QString &aTypeId,
             const QString &aFamilyId /* = QString::null */) const
{
    QList <CGuestOSType> list;
    if (mFamilyIDs.contains (aFamilyId))
    {
        list = mTypes [mFamilyIDs.indexOf (aFamilyId)];
    }
    else
    {
        for (int i = 0; i < mFamilyIDs.size(); ++ i)
            list += mTypes [i];
    }
    for (int j = 0; j < list.size(); ++ j)
        if (!list [j].GetId().compare (aTypeId))
            return list [j];
    AssertMsgFailed (("Type ID incorrect: '%s'.", aTypeId.toLatin1().constData()));
    return CGuestOSType();
}

/**
 *  Returns the description corresponding to the given guest OS type id.
 */
QString VBoxGlobal::vmGuestOSTypeDescription (const QString &aTypeId) const
{
    for (int i = 0; i < mFamilyIDs.size(); ++ i)
    {
        QList <CGuestOSType> list (mTypes [i]);
        for ( int j = 0; j < list.size(); ++ j)
            if (!list [j].GetId().compare (aTypeId))
                return list [j].GetDescription();
    }
    return QString::null;
}

struct PortConfig
{
    const char *name;
    const ulong IRQ;
    const ulong IOBase;
};

static const PortConfig kComKnownPorts[] =
{
    { "COM1", 4, 0x3F8 },
    { "COM2", 3, 0x2F8 },
    { "COM3", 4, 0x3E8 },
    { "COM4", 3, 0x2E8 },
    /* must not contain an element with IRQ=0 and IOBase=0 used to cause
     * toCOMPortName() to return the "User-defined" string for these values. */
};

static const PortConfig kLptKnownPorts[] =
{
    { "LPT1", 7, 0x378 },
    { "LPT2", 5, 0x278 },
    { "LPT1", 2, 0x3BC },
    /* must not contain an element with IRQ=0 and IOBase=0 used to cause
     * toLPTPortName() to return the "User-defined" string for these values. */
};

/**
 * Similar to toString (KMediumType), but returns 'Differencing' for
 * normal hard disks that have a parent.
 */
QString VBoxGlobal::mediumTypeString(const CMedium &medium) const
{
    if (!medium.GetParent().isNull())
    {
        Assert(medium.GetType() == KMediumType_Normal);
        return mDiskTypes_Differencing;
    }
    return gpConverter->toString(medium.GetType());
}

/**
 *  Returns the list of the standard COM port names (i.e. "COMx").
 */
QStringList VBoxGlobal::COMPortNames() const
{
    QStringList list;
    for (size_t i = 0; i < RT_ELEMENTS (kComKnownPorts); ++ i)
        list << kComKnownPorts [i].name;

    return list;
}

/**
 *  Returns the list of the standard LPT port names (i.e. "LPTx").
 */
QStringList VBoxGlobal::LPTPortNames() const
{
    QStringList list;
    for (size_t i = 0; i < RT_ELEMENTS (kLptKnownPorts); ++ i)
        list << kLptKnownPorts [i].name;

    return list;
}

/**
 *  Returns the name of the standard COM port corresponding to the given
 *  parameters, or "User-defined" (which is also returned when both
 *  @a aIRQ and @a aIOBase are 0).
 */
QString VBoxGlobal::toCOMPortName (ulong aIRQ, ulong aIOBase) const
{
    for (size_t i = 0; i < RT_ELEMENTS (kComKnownPorts); ++ i)
        if (kComKnownPorts [i].IRQ == aIRQ &&
            kComKnownPorts [i].IOBase == aIOBase)
            return kComKnownPorts [i].name;

    return mUserDefinedPortName;
}

/**
 *  Returns the name of the standard LPT port corresponding to the given
 *  parameters, or "User-defined" (which is also returned when both
 *  @a aIRQ and @a aIOBase are 0).
 */
QString VBoxGlobal::toLPTPortName (ulong aIRQ, ulong aIOBase) const
{
    for (size_t i = 0; i < RT_ELEMENTS (kLptKnownPorts); ++ i)
        if (kLptKnownPorts [i].IRQ == aIRQ &&
            kLptKnownPorts [i].IOBase == aIOBase)
            return kLptKnownPorts [i].name;

    return mUserDefinedPortName;
}

/**
 *  Returns port parameters corresponding to the given standard COM name.
 *  Returns @c true on success, or @c false if the given port name is not one
 *  of the standard names (i.e. "COMx").
 */
bool VBoxGlobal::toCOMPortNumbers (const QString &aName, ulong &aIRQ,
                                   ulong &aIOBase) const
{
    for (size_t i = 0; i < RT_ELEMENTS (kComKnownPorts); ++ i)
        if (strcmp (kComKnownPorts [i].name, aName.toUtf8().data()) == 0)
        {
            aIRQ = kComKnownPorts [i].IRQ;
            aIOBase = kComKnownPorts [i].IOBase;
            return true;
        }

    return false;
}

/**
 *  Returns port parameters corresponding to the given standard LPT name.
 *  Returns @c true on success, or @c false if the given port name is not one
 *  of the standard names (i.e. "LPTx").
 */
bool VBoxGlobal::toLPTPortNumbers (const QString &aName, ulong &aIRQ,
                                   ulong &aIOBase) const
{
    for (size_t i = 0; i < RT_ELEMENTS (kLptKnownPorts); ++ i)
        if (strcmp (kLptKnownPorts [i].name, aName.toUtf8().data()) == 0)
        {
            aIRQ = kLptKnownPorts [i].IRQ;
            aIOBase = kLptKnownPorts [i].IOBase;
            return true;
        }

    return false;
}

/**
 * Searches for the given hard disk in the list of known media descriptors and
 * calls UIMedium::details() on the found descriptor.
 *
 * If the requested hard disk is not found (for example, it's a new hard disk
 * for a new VM created outside our UI), then media enumeration is requested and
 * the search is repeated. We assume that the second attempt always succeeds and
 * assert otherwise.
 *
 * @note Technically, the second attempt may fail if, for example, the new hard
 *       passed to this method disk gets removed before #startEnumeratingMedia()
 *       succeeds. This (unexpected object uninitialization) is a generic
 *       problem though and needs to be addressed using exceptions (see also the
 *       @todo in UIMedium::details()).
 */
QString VBoxGlobal::details (const CMedium &aMedium, bool aPredictDiff, bool fUseHtml /* = true */)
{
    CMedium cmedium (aMedium);
    UIMedium medium;

    if (!findMedium (cmedium, medium))
    {
        /* Medium may be new and not already in the media list, request refresh */
        startEnumeratingMedia();
        if (!findMedium (cmedium, medium))
            /* Medium might be deleted already, return null string */
            return QString();
    }

    return fUseHtml ? medium.detailsHTML (true /* aNoDiffs */, aPredictDiff) :
                      medium.details(true /* aNoDiffs */, aPredictDiff);
}

/**
 *  Returns the details of the given USB device as a single-line string.
 */
QString VBoxGlobal::details (const CUSBDevice &aDevice) const
{
    QString sDetails;
    if (aDevice.isNull())
        sDetails = tr("Unknown device", "USB device details");
    else
    {
        QString m = aDevice.GetManufacturer().trimmed();
        QString p = aDevice.GetProduct().trimmed();

        if (m.isEmpty() && p.isEmpty())
        {
            sDetails =
                tr ("Unknown device %1:%2", "USB device details")
                .arg (QString().sprintf ("%04hX", aDevice.GetVendorId()))
                .arg (QString().sprintf ("%04hX", aDevice.GetProductId()));
        }
        else
        {
            if (p.toUpper().startsWith (m.toUpper()))
                sDetails = p;
            else
                sDetails = m + " " + p;
        }
        ushort r = aDevice.GetRevision();
        if (r != 0)
            sDetails += QString().sprintf (" [%04hX]", r);
    }

    return sDetails.trimmed();
}

/**
 *  Returns the multi-line description of the given USB device.
 */
QString VBoxGlobal::toolTip (const CUSBDevice &aDevice) const
{
    QString tip =
        tr ("<nobr>Vendor ID: %1</nobr><br>"
            "<nobr>Product ID: %2</nobr><br>"
            "<nobr>Revision: %3</nobr>", "USB device tooltip")
        .arg (QString().sprintf ("%04hX", aDevice.GetVendorId()))
        .arg (QString().sprintf ("%04hX", aDevice.GetProductId()))
        .arg (QString().sprintf ("%04hX", aDevice.GetRevision()));

    QString ser = aDevice.GetSerialNumber();
    if (!ser.isEmpty())
        tip += QString (tr ("<br><nobr>Serial No. %1</nobr>", "USB device tooltip"))
                        .arg (ser);

    /* add the state field if it's a host USB device */
    CHostUSBDevice hostDev (aDevice);
    if (!hostDev.isNull())
    {
        tip += QString (tr ("<br><nobr>State: %1</nobr>", "USB device tooltip"))
                        .arg (gpConverter->toString (hostDev.GetState()));
    }

    return tip;
}

/**
 *  Returns the multi-line description of the given USB filter
 */
QString VBoxGlobal::toolTip (const CUSBDeviceFilter &aFilter) const
{
    QString tip;

    QString vendorId = aFilter.GetVendorId();
    if (!vendorId.isEmpty())
        tip += tr ("<nobr>Vendor ID: %1</nobr>", "USB filter tooltip")
                   .arg (vendorId);

    QString productId = aFilter.GetProductId();
    if (!productId.isEmpty())
        tip += tip.isEmpty() ? "":"<br/>" + tr ("<nobr>Product ID: %2</nobr>", "USB filter tooltip")
                                                .arg (productId);

    QString revision = aFilter.GetRevision();
    if (!revision.isEmpty())
        tip += tip.isEmpty() ? "":"<br/>" + tr ("<nobr>Revision: %3</nobr>", "USB filter tooltip")
                                                .arg (revision);

    QString product = aFilter.GetProduct();
    if (!product.isEmpty())
        tip += tip.isEmpty() ? "":"<br/>" + tr ("<nobr>Product: %4</nobr>", "USB filter tooltip")
                                                .arg (product);

    QString manufacturer = aFilter.GetManufacturer();
    if (!manufacturer.isEmpty())
        tip += tip.isEmpty() ? "":"<br/>" + tr ("<nobr>Manufacturer: %5</nobr>", "USB filter tooltip")
                                                .arg (manufacturer);

    QString serial = aFilter.GetSerialNumber();
    if (!serial.isEmpty())
        tip += tip.isEmpty() ? "":"<br/>" + tr ("<nobr>Serial No.: %1</nobr>", "USB filter tooltip")
                                                .arg (serial);

    QString port = aFilter.GetPort();
    if (!port.isEmpty())
        tip += tip.isEmpty() ? "":"<br/>" + tr ("<nobr>Port: %1</nobr>", "USB filter tooltip")
                                                .arg (port);

    /* add the state field if it's a host USB device */
    CHostUSBDevice hostDev (aFilter);
    if (!hostDev.isNull())
    {
        tip += tip.isEmpty() ? "":"<br/>" + tr ("<nobr>State: %1</nobr>", "USB filter tooltip")
                                                .arg (gpConverter->toString (hostDev.GetState()));
    }

    return tip;
}

/**
 * Returns a details report on a given VM represented as a HTML table.
 *
 * @param aMachine      Machine to create a report for.
 * @param aWithLinks    @c true if section titles should be hypertext links.
 */
QString VBoxGlobal::detailsReport (const CMachine &aMachine, bool aWithLinks)
{
    /* Details templates */
    static const char *sTableTpl =
        "<table border=0 cellspacing=1 cellpadding=0>%1</table>";
    static const char *sSectionHrefTpl =
        "<tr><td width=22 rowspan=%1 align=left><img src='%2'></td>"
            "<td colspan=3><b><a href='%3'><nobr>%4</nobr></a></b></td></tr>"
            "%5"
        "<tr><td colspan=3><font size=1>&nbsp;</font></td></tr>";
    static const char *sSectionBoldTpl =
        "<tr><td width=22 rowspan=%1 align=left><img src='%2'></td>"
            "<td colspan=3><!-- %3 --><b><nobr>%4</nobr></b></td></tr>"
            "%5"
        "<tr><td colspan=3><font size=1>&nbsp;</font></td></tr>";
    static const char *sSectionItemTpl1 =
        "<tr><td width=40%><nobr><i>%1</i></nobr></td><td/><td/></tr>";
    static const char *sSectionItemTpl2 =
        "<tr><td width=40%><nobr>%1:</nobr></td><td/><td>%2</td></tr>";
    static const char *sSectionItemTpl3 =
        "<tr><td width=40%><nobr>%1</nobr></td><td/><td/></tr>";

    const QString &sectionTpl = aWithLinks ? sSectionHrefTpl : sSectionBoldTpl;

    /* Compose details report */
    QString report;

    /* General */
    {
        QString item = QString (sSectionItemTpl2).arg (tr ("Name", "details report"),
                                                       aMachine.GetName())
                     + QString (sSectionItemTpl2).arg (tr ("OS Type", "details report"),
                                                       vmGuestOSTypeDescription (aMachine.GetOSTypeId()));

        report += sectionTpl
                  .arg (2 + 2) /* rows */
                  .arg (":/machine_16px.png", /* icon */
                        "#general", /* link */
                        tr ("General", "details report"), /* title */
                        item); /* items */
    }

    /* System */
    {
        /* BIOS Settings holder */
        CBIOSSettings biosSettings = aMachine.GetBIOSSettings();

        /* System details row count: */
        int iRowCount = 2; /* Memory & CPU details rows initially. */

        /* Boot order */
        QString bootOrder;
        for (ulong i = 1; i <= mVBox.GetSystemProperties().GetMaxBootPosition(); ++ i)
        {
            KDeviceType device = aMachine.GetBootOrder (i);
            if (device == KDeviceType_Null)
                continue;
            if (!bootOrder.isEmpty())
                bootOrder += ", ";
            bootOrder += gpConverter->toString (device);
        }
        if (bootOrder.isEmpty())
            bootOrder = gpConverter->toString (KDeviceType_Null);

        iRowCount += 1; /* Boot-order row. */

#ifdef VBOX_WITH_FULL_DETAILS_REPORT
        /* ACPI */
        QString acpi = biosSettings.GetACPIEnabled()
            ? tr ("Enabled", "details report (ACPI)")
            : tr ("Disabled", "details report (ACPI)");

        /* IO APIC */
        QString ioapic = biosSettings.GetIOAPICEnabled()
            ? tr ("Enabled", "details report (IO APIC)")
            : tr ("Disabled", "details report (IO APIC)");

        /* PAE/NX */
        QString pae = aMachine.GetCpuProperty(KCpuPropertyType_PAE)
            ? tr ("Enabled", "details report (PAE/NX)")
            : tr ("Disabled", "details report (PAE/NX)");

        iRowCount += 3; /* Full report rows. */
#endif /* VBOX_WITH_FULL_DETAILS_REPORT */

        /* VT-x/AMD-V */
        QString virt = aMachine.GetHWVirtExProperty(KHWVirtExPropertyType_Enabled)
            ? tr ("Enabled", "details report (VT-x/AMD-V)")
            : tr ("Disabled", "details report (VT-x/AMD-V)");

        /* Nested Paging */
        QString nested = aMachine.GetHWVirtExProperty(KHWVirtExPropertyType_NestedPaging)
            ? tr ("Enabled", "details report (Nested Paging)")
            : tr ("Disabled", "details report (Nested Paging)");

        /* VT-x/AMD-V availability: */
        bool fVTxAMDVSupported = host().GetProcessorFeature(KProcessorFeature_HWVirtEx);

        if (fVTxAMDVSupported)
            iRowCount += 2; /* VT-x/AMD-V items. */

        QString item = QString (sSectionItemTpl2).arg (tr ("Base Memory", "details report"),
                                                       tr ("<nobr>%1 MB</nobr>", "details report"))
                       .arg (aMachine.GetMemorySize())
                     + QString (sSectionItemTpl2).arg (tr ("Processor(s)", "details report"),
                                                       tr ("<nobr>%1</nobr>", "details report"))
                       .arg (aMachine.GetCPUCount())
                     + QString (sSectionItemTpl2).arg (tr ("Execution Cap", "details report"),
                                                       tr ("<nobr>%1%</nobr>", "details report"))
                       .arg (aMachine.GetCPUExecutionCap())
                     + QString (sSectionItemTpl2).arg (tr ("Boot Order", "details report"), bootOrder)
#ifdef VBOX_WITH_FULL_DETAILS_REPORT
                     + QString (sSectionItemTpl2).arg (tr ("ACPI", "details report"), acpi)
                     + QString (sSectionItemTpl2).arg (tr ("IO APIC", "details report"), ioapic)
                     + QString (sSectionItemTpl2).arg (tr ("PAE/NX", "details report"), pae)
#endif /* VBOX_WITH_FULL_DETAILS_REPORT */
                     ;

        if (fVTxAMDVSupported)
                item += QString (sSectionItemTpl2).arg (tr ("VT-x/AMD-V", "details report"), virt)
                     +  QString (sSectionItemTpl2).arg (tr ("Nested Paging", "details report"), nested);

        report += sectionTpl
                  .arg (2 + iRowCount) /* rows */
                  .arg (":/chipset_16px.png", /* icon */
                        "#system", /* link */
                        tr ("System", "details report"), /* title */
                        item); /* items */
    }

    /* Display */
    {
        /* Rows including section header and footer */
        int rows = 2;

        /* Video tab */
        QString item = QString(sSectionItemTpl2)
                       .arg(tr ("Video Memory", "details report"),
                             tr ("<nobr>%1 MB</nobr>", "details report"))
                       .arg(aMachine.GetVRAMSize());
        ++rows;

        int cGuestScreens = aMachine.GetMonitorCount();
        if (cGuestScreens > 1)
        {
            item += QString(sSectionItemTpl2)
                    .arg(tr("Screens", "details report"))
                    .arg(cGuestScreens);
            ++rows;
        }

        QString acc3d = is3DAvailable() && aMachine.GetAccelerate3DEnabled()
            ? tr ("Enabled", "details report (3D Acceleration)")
            : tr ("Disabled", "details report (3D Acceleration)");

        item += QString(sSectionItemTpl2)
                .arg(tr("3D Acceleration", "details report"), acc3d);
        ++rows;

#ifdef VBOX_WITH_VIDEOHWACCEL
        QString acc2dVideo = aMachine.GetAccelerate2DVideoEnabled()
            ? tr ("Enabled", "details report (2D Video Acceleration)")
            : tr ("Disabled", "details report (2D Video Acceleration)");

        item += QString (sSectionItemTpl2)
                .arg (tr ("2D Video Acceleration", "details report"), acc2dVideo);
        ++ rows;
#endif

        /* VRDP tab */
        CVRDEServer srv = aMachine.GetVRDEServer();
        if (!srv.isNull())
        {
            if (srv.GetEnabled())
                item += QString (sSectionItemTpl2)
                        .arg (tr ("Remote Desktop Server Port", "details report (VRDE Server)"))
                        .arg (srv.GetVRDEProperty("TCP/Ports"));
            else
                item += QString (sSectionItemTpl2)
                        .arg (tr ("Remote Desktop Server", "details report (VRDE Server)"))
                        .arg (tr ("Disabled", "details report (VRDE Server)"));
            ++ rows;
        }

        report += sectionTpl
            .arg (rows) /* rows */
            .arg (":/vrdp_16px.png", /* icon */
                  "#display", /* link */
                  tr ("Display", "details report"), /* title */
                  item); /* items */
    }

    /* Storage */
    {
        /* Rows including section header and footer */
        int rows = 2;

        QString item;

        /* Iterate over the all machine controllers: */
        CStorageControllerVector controllers = aMachine.GetStorageControllers();
        for (int i = 0; i < controllers.size(); ++i)
        {
            /* Get current controller: */
            const CStorageController &controller = controllers[i];
            /* Add controller information: */
            QString strControllerName = QApplication::translate("UIMachineSettingsStorage", "Controller: %1");
            item += QString(sSectionItemTpl3).arg(strControllerName.arg(controller.GetName()));
            ++ rows;

            /* Populate sorted map with attachments information: */
            QMap<StorageSlot,QString> attachmentsMap;
            CMediumAttachmentVector attachments = aMachine.GetMediumAttachmentsOfController(controller.GetName());
            for (int j = 0; j < attachments.size(); ++j)
            {
                /* Get current attachment: */
                const CMediumAttachment &attachment = attachments[j];
                /* Prepare current storage slot: */
                StorageSlot attachmentSlot(controller.GetBus(), attachment.GetPort(), attachment.GetDevice());
                /* Append 'device slot name' with 'device type name' for CD/DVD devices only: */
                QString strDeviceType = attachment.GetType() == KDeviceType_DVD ? tr("(CD/DVD)") : QString();
                if (!strDeviceType.isNull())
                    strDeviceType.prepend(' ');
                /* Prepare current medium object: */
                const CMedium &medium = attachment.GetMedium();
                /* Prepare information about current medium & attachment: */
                QString strAttachmentInfo = !attachment.isOk() ? QString() :
                                            QString(sSectionItemTpl2)
                                            .arg(QString("&nbsp;&nbsp;") +
                                                 gpConverter->toString(StorageSlot(controller.GetBus(),
                                                                                   attachment.GetPort(),
                                                                                   attachment.GetDevice())) + strDeviceType)
                                            .arg(details(medium, false));
                /* Insert that attachment into map: */
                if (!strAttachmentInfo.isNull())
                    attachmentsMap.insert(attachmentSlot, strAttachmentInfo);
            }

            /* Iterate over the sorted map with attachments information: */
            QMapIterator<StorageSlot,QString> it(attachmentsMap);
            while (it.hasNext())
            {
                /* Add controller information: */
                it.next();
                item += it.value();
                ++rows;
            }
        }

        if (item.isNull())
        {
            item = QString (sSectionItemTpl1)
                   .arg (tr ("Not Attached", "details report (Storage)"));
            ++ rows;
        }

        report += sectionTpl
            .arg (rows) /* rows */
            .arg (":/attachment_16px.png", /* icon */
                  "#storage", /* link */
                  tr ("Storage", "details report"), /* title */
                  item); /* items */
    }

    /* Audio */
    {
        QString item;

        CAudioAdapter audio = aMachine.GetAudioAdapter();
        int rows = audio.GetEnabled() ? 3 : 2;
        if (audio.GetEnabled())
            item = QString (sSectionItemTpl2)
                   .arg (tr ("Host Driver", "details report (audio)"),
                         gpConverter->toString (audio.GetAudioDriver())) +
                   QString (sSectionItemTpl2)
                   .arg (tr ("Controller", "details report (audio)"),
                         gpConverter->toString (audio.GetAudioController()));
        else
            item = QString (sSectionItemTpl1)
                   .arg (tr ("Disabled", "details report (audio)"));

        report += sectionTpl
            .arg (rows + 1) /* rows */
            .arg (":/sound_16px.png", /* icon */
                  "#audio", /* link */
                  tr ("Audio", "details report"), /* title */
                  item); /* items */
    }

    /* Network */
    {
        QString item;

        ulong count = mVBox.GetSystemProperties().GetMaxNetworkAdapters(KChipsetType_PIIX3);
        int rows = 2; /* including section header and footer */
        for (ulong slot = 0; slot < count; slot ++)
        {
            CNetworkAdapter adapter = aMachine.GetNetworkAdapter (slot);
            if (adapter.GetEnabled())
            {
                KNetworkAttachmentType type = adapter.GetAttachmentType();
                QString attType = gpConverter->toString (adapter.GetAdapterType())
                                  .replace (QRegExp ("\\s\\(.+\\)"), " (%1)");
                /* don't use the adapter type string for types that have
                 * an additional symbolic network/interface name field, use
                 * this name instead */
                if (type == KNetworkAttachmentType_Bridged)
                    attType = attType.arg (tr ("Bridged adapter, %1",
                        "details report (network)").arg (adapter.GetBridgedInterface()));
                else if (type == KNetworkAttachmentType_Internal)
                    attType = attType.arg (tr ("Internal network, '%1'",
                        "details report (network)").arg (adapter.GetInternalNetwork()));
                else if (type == KNetworkAttachmentType_HostOnly)
                    attType = attType.arg (tr ("Host-only adapter, '%1'",
                        "details report (network)").arg (adapter.GetHostOnlyInterface()));
                else if (type == KNetworkAttachmentType_Generic)
                    attType = attType.arg (tr ("Generic, '%1'",
                        "details report (network)").arg (adapter.GetGenericDriver()));
                else
                    attType = attType.arg (gpConverter->toString (type));

                item += QString (sSectionItemTpl2)
                        .arg (tr ("Adapter %1", "details report (network)")
                              .arg (adapter.GetSlot() + 1))
                        .arg (attType);
                ++ rows;
            }
        }
        if (item.isNull())
        {
            item = QString (sSectionItemTpl1)
                   .arg (tr ("Disabled", "details report (network)"));
            ++ rows;
        }

        report += sectionTpl
            .arg (rows) /* rows */
            .arg (":/nw_16px.png", /* icon */
                  "#network", /* link */
                  tr ("Network", "details report"), /* title */
                  item); /* items */
    }

    /* Serial Ports */
    {
        QString item;

        ulong count = mVBox.GetSystemProperties().GetSerialPortCount();
        int rows = 2; /* including section header and footer */
        for (ulong slot = 0; slot < count; slot ++)
        {
            CSerialPort port = aMachine.GetSerialPort (slot);
            if (port.GetEnabled())
            {
                KPortMode mode = port.GetHostMode();
                QString data =
                    toCOMPortName (port.GetIRQ(), port.GetIOBase()) + ", ";
                if (mode == KPortMode_HostPipe ||
                    mode == KPortMode_HostDevice ||
                    mode == KPortMode_RawFile)
                    data += QString ("%1 (<nobr>%2</nobr>)")
                            .arg (gpConverter->toString (mode))
                            .arg (QDir::toNativeSeparators (port.GetPath()));
                else
                    data += gpConverter->toString (mode);

                item += QString (sSectionItemTpl2)
                        .arg (tr ("Port %1", "details report (serial ports)")
                              .arg (port.GetSlot() + 1))
                        .arg (data);
                ++ rows;
            }
        }
        if (item.isNull())
        {
            item = QString (sSectionItemTpl1)
                   .arg (tr ("Disabled", "details report (serial ports)"));
            ++ rows;
        }

        report += sectionTpl
            .arg (rows) /* rows */
            .arg (":/serial_port_16px.png", /* icon */
                  "#serialPorts", /* link */
                  tr ("Serial Ports", "details report"), /* title */
                  item); /* items */
    }

    /* Parallel Ports */
    {
        QString item;

        ulong count = mVBox.GetSystemProperties().GetParallelPortCount();
        int rows = 2; /* including section header and footer */
        for (ulong slot = 0; slot < count; slot ++)
        {
            CParallelPort port = aMachine.GetParallelPort (slot);
            if (port.GetEnabled())
            {
                QString data =
                    toLPTPortName (port.GetIRQ(), port.GetIOBase()) +
                    QString (" (<nobr>%1</nobr>)")
                    .arg (QDir::toNativeSeparators (port.GetPath()));

                item += QString (sSectionItemTpl2)
                        .arg (tr ("Port %1", "details report (parallel ports)")
                              .arg (port.GetSlot() + 1))
                        .arg (data);
                ++ rows;
            }
        }
        if (item.isNull())
        {
            item = QString (sSectionItemTpl1)
                   .arg (tr ("Disabled", "details report (parallel ports)"));
            ++ rows;
        }

        /* Temporary disabled */
        QString dummy = sectionTpl /* report += sectionTpl */
            .arg (rows) /* rows */
            .arg (":/parallel_port_16px.png", /* icon */
                  "#parallelPorts", /* link */
                  tr ("Parallel Ports", "details report"), /* title */
                  item); /* items */
    }

    /* USB */
    {
        QString item;

        CUSBController ctl = aMachine.GetUSBController();
        if (   !ctl.isNull()
            && ctl.GetProxyAvailable())
        {
            /* the USB controller may be unavailable (i.e. in VirtualBox OSE) */

            if (ctl.GetEnabled())
            {
                CUSBDeviceFilterVector coll = ctl.GetDeviceFilters();
                uint active = 0;
                for (int i = 0; i < coll.size(); ++i)
                    if (coll[i].GetActive())
                        active ++;

                item = QString (sSectionItemTpl2)
                       .arg (tr ("Device Filters", "details report (USB)"),
                             tr ("%1 (%2 active)", "details report (USB)")
                                 .arg (coll.size()).arg (active));
            }
            else
                item = QString (sSectionItemTpl1)
                       .arg (tr ("Disabled", "details report (USB)"));

            report += sectionTpl
                .arg (2 + 1) /* rows */
                .arg (":/usb_16px.png", /* icon */
                      "#usb", /* link */
                      tr ("USB", "details report"), /* title */
                      item); /* items */
        }
    }

    /* Shared Folders */
    {
        QString item;

        ulong count = aMachine.GetSharedFolders().size();
        if (count > 0)
        {
            item = QString (sSectionItemTpl2)
                   .arg (tr ("Shared Folders", "details report (shared folders)"))
                   .arg (count);
        }
        else
            item = QString (sSectionItemTpl1)
                   .arg (tr ("None", "details report (shared folders)"));

        report += sectionTpl
            .arg (2 + 1) /* rows */
            .arg (":/shared_folder_16px.png", /* icon */
                  "#sfolders", /* link */
                  tr ("Shared Folders", "details report"), /* title */
                  item); /* items */
    }

    return QString (sTableTpl). arg (report);
}

#if defined(Q_WS_X11) && !defined(VBOX_OSE)
double VBoxGlobal::findLicenseFile (const QStringList &aFilesList, QRegExp aPattern, QString &aLicenseFile) const
{
    double maxVersionNumber = 0;
    aLicenseFile = "";
    for (int index = 0; index < aFilesList.count(); ++ index)
    {
        aPattern.indexIn (aFilesList [index]);
        QString version = aPattern.cap (1);
        if (maxVersionNumber < version.toDouble())
        {
            maxVersionNumber = version.toDouble();
            aLicenseFile = aFilesList [index];
        }
    }
    return maxVersionNumber;
}

bool VBoxGlobal::showVirtualBoxLicense()
{
    /* get the apps doc path */
    int size = 256;
    char *buffer = (char*) RTMemTmpAlloc (size);
    RTPathAppDocs (buffer, size);
    QString path (buffer);
    RTMemTmpFree (buffer);
    QDir docDir (path);
    docDir.setFilter (QDir::Files);
    docDir.setNameFilters (QStringList ("License-*.html"));

    /* Make sure that the language is in two letter code.
     * Note: if languageId() returns an empty string lang.name() will
     * return "C" which is an valid language code. */
    QLocale lang (VBoxGlobal::languageId());

    QStringList filesList = docDir.entryList();
    QString licenseFile;
    /* First try to find a localized version of the license file. */
    double versionNumber = findLicenseFile (filesList, QRegExp (QString ("License-([\\d\\.]+)-%1.html").arg (lang.name())), licenseFile);
    /* If there wasn't a localized version of the currently selected language,
     * search for the generic one. */
    if (versionNumber == 0)
        versionNumber = findLicenseFile (filesList, QRegExp ("License-([\\d\\.]+).html"), licenseFile);
    /* Check the version again. */
    if (!versionNumber)
    {
        msgCenter().cannotFindLicenseFiles (path);
        return false;
    }

    /* compose the latest license file full path */
    QString latestVersion = QString::number (versionNumber);
    QString latestFilePath = docDir.absoluteFilePath (licenseFile);

    /* check for the agreed license version */
    QStringList strList =  virtualBox().GetExtraData (GUI_LicenseKey).split(",");
    for (int i=0; i < strList.size(); ++i)
        if (strList.at(i) == latestVersion)
            return true;

    VBoxLicenseViewer licenseDialog;
    bool result = licenseDialog.showLicenseFromFile(latestFilePath) == QDialog::Accepted;
    if (result)
        virtualBox().SetExtraData (GUI_LicenseKey, (strList << latestVersion).join(","));
    return result;
}
#endif /* defined(Q_WS_X11) && !defined(VBOX_OSE) */

/**
 *  Opens a direct session for a machine with the given ID.
 *  This method does user-friendly error handling (display error messages, etc.).
 *  and returns a null CSession object in case of any error.
 *  If this method succeeds, don't forget to close the returned session when
 *  it is no more necessary.
 *
 *  @param aId          Machine ID.
 *  @param aLockType    @c KLockType_Shared to open an existing session with
 *                      the machine which is already running, @c KLockType_Write
 *                      to open a new direct session, @c KLockType_VM to open
 *                      a new session for running a VM in this process.
 */
CSession VBoxGlobal::openSession(const QString &aId, KLockType aLockType /* = KLockType_Shared */)
{
    CSession session;
    session.createInstance(CLSID_Session);
    if (session.isNull())
    {
        msgCenter().cannotOpenSession (session);
        return session;
    }

    CMachine foundMachine = CVirtualBox(mVBox).FindMachine(aId);
    if (!foundMachine.isNull())
    {
        foundMachine.LockMachine(session, aLockType);
        if (session.GetType() == KSessionType_Shared)
        {
            CMachine machine = session.GetMachine();
            /* Make sure that the language is in two letter code.
             * Note: if languageId() returns an empty string lang.name() will
             * return "C" which is an valid language code. */
            QLocale lang(VBoxGlobal::languageId());
            machine.SetGuestPropertyValue ("/VirtualBox/HostInfo/GUI/LanguageID", lang.name());
        }
    }

    if (!foundMachine.isOk())
    {
        msgCenter().cannotOpenSession(foundMachine);
        session.detach();
    }
    else if (!mVBox.isOk())
    {
        msgCenter().cannotOpenSession(mVBox, foundMachine);
        session.detach();
    }

    return session;
}

/**
 * Appends the NULL medium to the media list.
 * For using with VBoxGlobal::startEnumeratingMedia() only.
 */
static void addNullMediumToList (VBoxMediaList &aList, VBoxMediaList::iterator aWhere)
{
    UIMedium medium;
    aList.insert (aWhere, medium);
}

/**
 * Appends the given list of mediums to the media list.
 * For using with VBoxGlobal::startEnumeratingMedia() only.
 */
static void addMediumsToList (const CMediumVector &aVector,
                              VBoxMediaList &aList,
                              VBoxMediaList::iterator aWhere,
                              UIMediumType aType,
                              UIMedium *aParent = 0)
{
    VBoxMediaList::iterator first = aWhere;

    for (CMediumVector::ConstIterator it = aVector.begin(); it != aVector.end(); ++ it)
    {
        CMedium cmedium (*it);
        UIMedium medium (cmedium, aType, aParent);

        /* Search for a proper alphabetic position */
        VBoxMediaList::iterator jt = first;
        for (; jt != aWhere; ++ jt)
            if ((*jt).name().localeAwareCompare (medium.name()) > 0)
                break;

        aList.insert (jt, medium);

        /* Adjust the first item if inserted before it */
        if (jt == first)
            -- first;
    }
}

/**
 * Appends the given list of hard disks and all their children to the media list.
 * For using with VBoxGlobal::startEnumeratingMedia() only.
 */
static void addHardDisksToList (const CMediumVector &aVector,
                                VBoxMediaList &aList,
                                VBoxMediaList::iterator aWhere,
                                UIMedium *aParent = 0)
{
    VBoxMediaList::iterator first = aWhere;

    /* First pass: Add siblings sorted */
    for (CMediumVector::ConstIterator it = aVector.begin(); it != aVector.end(); ++ it)
    {
        CMedium cmedium (*it);
        UIMedium medium (cmedium, UIMediumType_HardDisk, aParent);

        /* Search for a proper alphabetic position */
        VBoxMediaList::iterator jt = first;
        for (; jt != aWhere; ++ jt)
            if ((*jt).name().localeAwareCompare (medium.name()) > 0)
                break;

        aList.insert (jt, medium);

        /* Adjust the first item if inserted before it */
        if (jt == first)
            -- first;
    }

    /* Second pass: Add children */
    for (VBoxMediaList::iterator it = first; it != aWhere;)
    {
        CMediumVector children = (*it).medium().GetChildren();
        UIMedium *parent = &(*it);

        ++ it; /* go to the next sibling before inserting children */
        addHardDisksToList (children, aList, it, parent);
    }
}

/**
 * Starts a thread that asynchronously enumerates all currently registered
 * media.
 *
 * Before the enumeration is started, the current media list (a list returned by
 * #currentMediaList()) is populated with all registered media and the
 * #mediumEnumStarted() signal is emitted. The enumeration thread then walks this
 * list, checks for media accessibility and emits #mediumEnumerated() signals of
 * each checked medium. When all media are checked, the enumeration thread is
 * stopped and the #mediumEnumFinished() signal is emitted.
 *
 * If the enumeration is already in progress, no new thread is started.
 *
 * The media list returned by #currentMediaList() is always sorted
 * alphabetically by the location attribute and comes in the following order:
 * <ol>
 *  <li>All hard disks. If a hard disk has children, these children
 *      (alphabetically sorted) immediately follow their parent and therefore
 *      appear before its next sibling hard disk.</li>
 *  <li>All CD/DVD images.</li>
 *  <li>All Floppy images.</li>
 * </ol>
 *
 * Note that #mediumEnumerated() signals are emitted in the same order as
 * described above.
 *
 * @sa #currentMediaList()
 * @sa #isMediaEnumerationStarted()
 */
void VBoxGlobal::startEnumeratingMedia()
{
    AssertReturnVoid (mValid);

    /* check if already started but not yet finished */
    if (mMediaEnumThread != NULL)
        return;

    /* ignore the request during application termination */
    if (sVBoxGlobalInCleanup)
        return;

    /* composes a list of all currently known media & their children */
    mMediaList.clear();
    addNullMediumToList (mMediaList, mMediaList.end());
    addHardDisksToList (mVBox.GetHardDisks(), mMediaList, mMediaList.end());
    addMediumsToList (mHost.GetDVDDrives(), mMediaList, mMediaList.end(), UIMediumType_DVD);
    addMediumsToList (mVBox.GetDVDImages(), mMediaList, mMediaList.end(), UIMediumType_DVD);
    addMediumsToList (mHost.GetFloppyDrives(), mMediaList, mMediaList.end(), UIMediumType_Floppy);
    addMediumsToList (mVBox.GetFloppyImages(), mMediaList, mMediaList.end(), UIMediumType_Floppy);

    /* enumeration thread class */
    class MediaEnumThread : public QThread
    {
    public:

        MediaEnumThread (VBoxMediaList &aList)
            : mVector (aList.size())
            , mSavedIt (aList.begin())
        {
            int i = 0;
            for (VBoxMediaList::const_iterator it = aList.begin();
                 it != aList.end(); ++ it)
                mVector [i ++] = *it;
        }

        virtual void run()
        {
            LogFlow (("MediaEnumThread started.\n"));
            COMBase::InitializeCOM(false);

            CVirtualBox mVBox = vboxGlobal().virtualBox();
            QObject *self = &vboxGlobal();

            /* Enumerate the list */
            for (int i = 0; i < mVector.size() && !sVBoxGlobalInCleanup; ++ i)
            {
                mVector [i].blockAndQueryState();
                QApplication::
                    postEvent (self,
                               new VBoxMediaEnumEvent (mVector [i], mSavedIt));
                ++mSavedIt;
            }

            /* Post the end-of-enumeration event */
            if (!sVBoxGlobalInCleanup)
                QApplication::postEvent (self, new VBoxMediaEnumEvent (mSavedIt));

            COMBase::CleanupCOM();
            LogFlow (("MediaEnumThread finished.\n"));
        }

    private:

        QVector <UIMedium> mVector;
        VBoxMediaList::iterator mSavedIt;
    };

    mMediaEnumThread = new MediaEnumThread (mMediaList);
    AssertReturnVoid (mMediaEnumThread);

    /* emit mediumEnumStarted() after we set mMediaEnumThread to != NULL
     * to cause isMediaEnumerationStarted() to return TRUE from slots */
    emit mediumEnumStarted();

    mMediaEnumThread->start();
}

void VBoxGlobal::reloadProxySettings()
{
    UIProxyManager proxyManager(settings().proxySettings());
    if (proxyManager.authEnabled())
    {
        proxyManager.setAuthEnabled(false);
        proxyManager.setAuthLogin(QString());
        proxyManager.setAuthPassword(QString());
        VBoxGlobalSettings globalSettings = settings();
        globalSettings.setProxySettings(proxyManager.toString());
        vboxGlobal().setSettings(globalSettings);
    }
    if (proxyManager.proxyEnabled())
    {
#if 0
        QNetworkProxy::setApplicationProxy(QNetworkProxy(QNetworkProxy::HttpProxy,
                                                         proxyManager.proxyHost(),
                                                         proxyManager.proxyPort().toInt(),
                                                         proxyManager.authEnabled() ? proxyManager.authLogin() : QString(),
                                                         proxyManager.authEnabled() ? proxyManager.authPassword() : QString()));
#else
        QNetworkProxy::setApplicationProxy(QNetworkProxy(QNetworkProxy::HttpProxy,
                                                         proxyManager.proxyHost(),
                                                         proxyManager.proxyPort().toInt()));
#endif
    }
    else
    {
        QNetworkProxy::setApplicationProxy(QNetworkProxy(QNetworkProxy::NoProxy));
    }
}

/**
 * Adds a new medium to the current media list and emits the #mediumAdded()
 * signal.
 *
 * @sa #currentMediaList()
 */
void VBoxGlobal::addMedium (const UIMedium &aMedium)
{
    /* Note that we maintain the same order here as #startEnumeratingMedia() */

    VBoxMediaList::iterator it = mMediaList.begin();

    if (aMedium.type() == UIMediumType_HardDisk)
    {
        VBoxMediaList::iterator itParent = mMediaList.end();

        for (; it != mMediaList.end(); ++ it)
        {
            /* skip null medium that come first */
            if ((*it).isNull()) continue;

            if ((*it).type() != UIMediumType_HardDisk)
                break;

            if (aMedium.parent() != NULL && itParent == mMediaList.end())
            {
                if (&*it == aMedium.parent())
                    itParent = it;
            }
            else
            {
                /* break if met a parent's sibling (will insert before it) */
                if (aMedium.parent() != NULL &&
                    (*it).parent() == (*itParent).parent())
                    break;

                /* compare to aMedium's siblings */
                if ((*it).parent() == aMedium.parent() &&
                    (*it).name().localeAwareCompare (aMedium.name()) > 0)
                    break;
            }
        }

        AssertReturnVoid (aMedium.parent() == NULL || itParent != mMediaList.end());
    }
    else
    {
        for (; it != mMediaList.end(); ++ it)
        {
            /* skip null medium that come first */
            if ((*it).isNull()) continue;

            /* skip HardDisks that come first */
            if ((*it).type() == UIMediumType_HardDisk)
                continue;

            /* skip DVD when inserting Floppy */
            if (aMedium.type() == UIMediumType_Floppy &&
                (*it).type() == UIMediumType_DVD)
                continue;

            if ((*it).name().localeAwareCompare (aMedium.name()) > 0 ||
                (aMedium.type() == UIMediumType_DVD &&
                 (*it).type() == UIMediumType_Floppy))
                break;
        }
    }

    it = mMediaList.insert (it, aMedium);

    emit mediumAdded (*it);
}

/**
 * Updates the medium in the current media list and emits the #mediumUpdated()
 * signal.
 *
 * @sa #currentMediaList()
 */
void VBoxGlobal::updateMedium (const UIMedium &aMedium)
{
    VBoxMediaList::Iterator it;
    for (it = mMediaList.begin(); it != mMediaList.end(); ++ it)
        if ((*it).id() == aMedium.id())
            break;

    AssertReturnVoid (it != mMediaList.end());

    if (&*it != &aMedium)
        *it = aMedium;

    emit mediumUpdated (*it);
}

/**
 * Removes the medium from the current media list and emits the #mediumRemoved()
 * signal.
 *
 * @sa #currentMediaList()
 */
void VBoxGlobal::removeMedium (UIMediumType aType, const QString &aId)
{
    VBoxMediaList::Iterator it;
    for (it = mMediaList.begin(); it != mMediaList.end(); ++ it)
        if ((*it).id() == aId)
            break;

    AssertReturnVoid (it != mMediaList.end());

#if DEBUG
    /* sanity: must be no children */
    {
        VBoxMediaList::Iterator jt = it;
        ++ jt;
        AssertReturnVoid (jt == mMediaList.end() || (*jt).parent() != &*it);
    }
#endif

    UIMedium *pParent = (*it).parent();

    /* remove the medium from the list to keep it in sync with the server "for
     * free" when the medium is deleted from one of our UIs */
    mMediaList.erase (it);

    emit mediumRemoved (aType, aId);

    /* also emit the parent update signal because some attributes like
     * isReadOnly() may have been changed after child removal */
    if (pParent != NULL)
    {
        pParent->refresh();
        emit mediumUpdated (*pParent);
    }
}

/**
 *  Searches for a VBoxMedum object representing the given COM medium object.
 *
 *  @return true if found and false otherwise.
 */
bool VBoxGlobal::findMedium (const CMedium &aObj, UIMedium &aMedium) const
{
    for (VBoxMediaList::ConstIterator it = mMediaList.begin(); it != mMediaList.end(); ++ it)
    {
        if (((*it).medium().isNull() && aObj.isNull()) ||
            (!(*it).medium().isNull() && !aObj.isNull() && (*it).medium().GetId() == aObj.GetId()))
        {
            aMedium = (*it);
            return true;
        }
    }
    return false;
}

/**
 *  Searches for a VBoxMedum object with the given medium id attribute.
 *
 *  @return VBoxMedum if found which is invalid otherwise.
 */
UIMedium VBoxGlobal::findMedium (const QString &aMediumId) const
{
    for (VBoxMediaList::ConstIterator it = mMediaList.begin(); it != mMediaList.end(); ++ it)
        if ((*it).id() == aMediumId)
            return *it;
    return UIMedium();
}

/* Open some external medium using file open dialog
 * and temporary cache (enumerate) it in GUI inner mediums cache: */
QString VBoxGlobal::openMediumWithFileOpenDialog(UIMediumType mediumType, QWidget *pParent,
                                                 const QString &strDefaultFolder /* = QString() */,
                                                 bool fUseLastFolder /* = false */)
{
    /* Initialize variables: */
    QList < QPair <QString, QString> > filters;
    QStringList backends;
    QStringList prefixes;
    QString strFilter;
    QString strTitle;
    QString allType;
    QString strLastFolder;
    switch (mediumType)
    {
        case UIMediumType_HardDisk:
        {
            filters = vboxGlobal().HDDBackends();
            strTitle = tr("Please choose a virtual hard drive file");
            allType = tr("All virtual hard drive files (%1)");
            strLastFolder = virtualBox().GetExtraData(GUI_RecentFolderHD);
            if (strLastFolder.isEmpty())
                strLastFolder = virtualBox().GetExtraData(GUI_RecentFolderCD);
            if (strLastFolder.isEmpty())
                strLastFolder = virtualBox().GetExtraData(GUI_RecentFolderFD);
            break;
        }
        case UIMediumType_DVD:
        {
            filters = vboxGlobal().DVDBackends();
            strTitle = tr("Please choose a virtual optical disk file");
            allType = tr("All virtual optical disk files (%1)");
            strLastFolder = virtualBox().GetExtraData(GUI_RecentFolderCD);
            if (strLastFolder.isEmpty())
                strLastFolder = virtualBox().GetExtraData(GUI_RecentFolderHD);
            if (strLastFolder.isEmpty())
                strLastFolder = virtualBox().GetExtraData(GUI_RecentFolderFD);
            break;
        }
        case UIMediumType_Floppy:
        {
            filters = vboxGlobal().FloppyBackends();
            strTitle = tr("Please choose a virtual floppy disk file");
            allType = tr("All virtual floppy disk files (%1)");
            strLastFolder = virtualBox().GetExtraData(GUI_RecentFolderFD);
            if (strLastFolder.isEmpty())
                strLastFolder = virtualBox().GetExtraData(GUI_RecentFolderCD);
            if (strLastFolder.isEmpty())
                strLastFolder = virtualBox().GetExtraData(GUI_RecentFolderHD);
            break;
        }
        default:
            break;
    }
    QString strHomeFolder = fUseLastFolder && !strLastFolder.isEmpty() ? strLastFolder :
                            strDefaultFolder.isEmpty() ? vboxGlobal().virtualBox().GetHomeFolder() : strDefaultFolder;

    /* Prepare filters and backends: */
    for (int i = 0; i < filters.count(); ++i)
    {
        /* Get iterated filter: */
        QPair <QString, QString> item = filters.at(i);
        /* Create one backend filter string: */
        backends << QString("%1 (%2)").arg(item.first).arg(item.second);
        /* Save the suffix's for the "All" entry: */
        prefixes << item.second;
    }
    if (!prefixes.isEmpty())
        backends.insert(0, allType.arg(prefixes.join(" ").trimmed()));
    backends << tr("All files (*)");
    strFilter = backends.join(";;").trimmed();

    /* Create open file dialog: */
    QStringList files = QIFileDialog::getOpenFileNames(strHomeFolder, strFilter, pParent, strTitle, 0, true, true);

    /* If dialog has some result: */
    if (!files.empty() && !files[0].isEmpty())
        return openMedium(mediumType, files[0], pParent);

    return QString();
}

QString VBoxGlobal::openMedium(UIMediumType mediumType, QString strMediumLocation, QWidget *pParent /* = 0*/)
{
    /* Convert to native separators: */
    strMediumLocation = QDir::toNativeSeparators(strMediumLocation);

    /* Initialize variables: */
    CVirtualBox vbox = vboxGlobal().virtualBox();

    /* Remember the path of the last chosen medium: */
    QString strRecentFolderKey = mediumType == UIMediumType_HardDisk ? GUI_RecentFolderHD :
                                 mediumType == UIMediumType_DVD ? GUI_RecentFolderCD :
                                 mediumType == UIMediumType_Floppy ? GUI_RecentFolderFD :
                                 QString();
    vbox.SetExtraData(strRecentFolderKey, QFileInfo(strMediumLocation).absolutePath());

    /* Update recently used list: */
    QString strRecentListKey = mediumType == UIMediumType_HardDisk ? GUI_RecentListHD :
                               mediumType == UIMediumType_DVD ? GUI_RecentListCD :
                               mediumType == UIMediumType_Floppy ? GUI_RecentListFD :
                               QString();
    QStringList recentMediumList = vbox.GetExtraData(strRecentListKey).split(';');
    if (recentMediumList.contains(strMediumLocation))
        recentMediumList.removeAll(strMediumLocation);
    recentMediumList.prepend(strMediumLocation);
    while(recentMediumList.size() > 5) recentMediumList.removeLast();
    vbox.SetExtraData(strRecentListKey, recentMediumList.join(";"));

    /* Open corresponding medium: */
    CMedium comMedium = vbox.OpenMedium(strMediumLocation, mediumTypeToGlobal(mediumType), KAccessMode_ReadWrite, false);

    if (vbox.isOk())
    {
        /* Prepare vbox medium wrapper: */
        UIMedium vboxMedium;

        /* First of all we should test if that medium already opened: */
        if (!vboxGlobal().findMedium(comMedium, vboxMedium))
        {
            /* And create new otherwise: */
            vboxMedium = UIMedium(CMedium(comMedium), mediumType, KMediumState_Created);
            vboxGlobal().addMedium(vboxMedium);
        }

        /* Return vboxMedium id: */
        return vboxMedium.id();
    }
    else
        msgCenter().cannotOpenMedium(pParent, vbox, mediumType, strMediumLocation);

    return QString();
}

#ifdef VBOX_GUI_WITH_SYSTRAY
/**
 *  Returns the number of current running Fe/Qt4 main windows.
 *
 *  @return Number of running main windows.
 */
int VBoxGlobal::mainWindowCount ()
{
    return mVBox.GetExtraData (GUI_MainWindowCount).toInt();
}
#endif

/**
 *  Native language name of the currently installed translation.
 *  Returns "English" if no translation is installed
 *  or if the translation file is invalid.
 */
QString VBoxGlobal::languageName() const
{

    return qApp->translate ("@@@", "English",
                            "Native language name");
}

/**
 *  Native language country name of the currently installed translation.
 *  Returns "--" if no translation is installed or if the translation file is
 *  invalid, or if the language is independent on the country.
 */
QString VBoxGlobal::languageCountry() const
{
    return qApp->translate ("@@@", "--",
                            "Native language country name "
                            "(empty if this language is for all countries)");
}

/**
 *  Language name of the currently installed translation, in English.
 *  Returns "English" if no translation is installed
 *  or if the translation file is invalid.
 */
QString VBoxGlobal::languageNameEnglish() const
{

    return qApp->translate ("@@@", "English",
                            "Language name, in English");
}

/**
 *  Language country name of the currently installed translation, in English.
 *  Returns "--" if no translation is installed or if the translation file is
 *  invalid, or if the language is independent on the country.
 */
QString VBoxGlobal::languageCountryEnglish() const
{
    return qApp->translate ("@@@", "--",
                            "Language country name, in English "
                            "(empty if native country name is empty)");
}

/**
 *  Comma-separated list of authors of the currently installed translation.
 *  Returns "Oracle Corporation" if no translation is installed or if the
 *  translation file is invalid, or if the translation is supplied by Oracle
 *  Corporation
 */
QString VBoxGlobal::languageTranslators() const
{
    return qApp->translate ("@@@", "Oracle Corporation",
                            "Comma-separated list of translators");
}

/**
 *  Changes the language of all global string constants according to the
 *  currently installed translations tables.
 */
void VBoxGlobal::retranslateUi()
{
    mDiskTypes_Differencing = tr ("Differencing", "DiskType");

    mUserDefinedPortName = tr ("User-defined", "serial port");

    mWarningIcon = UIIconPool::defaultIcon(UIIconPool::MessageBoxWarningIcon).pixmap (16, 16);
    Assert (!mWarningIcon.isNull());

    mErrorIcon = UIIconPool::defaultIcon(UIIconPool::MessageBoxCriticalIcon).pixmap (16, 16);
    Assert (!mErrorIcon.isNull());

    /* refresh media properties since they contain some translations too  */
    for (VBoxMediaList::iterator it = mMediaList.begin();
         it != mMediaList.end(); ++ it)
        it->refresh();

#ifdef Q_WS_X11
    /* As X11 do not have functionality for providing human readable key names,
     * we keep a table of them, which must be updated when the language is changed. */
    UIHotKey::retranslateKeyNames();
#endif /* Q_WS_X11 */
}

// public static stuff
////////////////////////////////////////////////////////////////////////////////

/* static */
bool VBoxGlobal::isDOSType (const QString &aOSTypeId)
{
    if (aOSTypeId.left (3) == "dos" ||
        aOSTypeId.left (3) == "win" ||
        aOSTypeId.left (3) == "os2")
        return true;

    return false;
}

const char *gVBoxLangSubDir = "/nls";
const char *gVBoxLangFileBase = "VirtualBox_";
const char *gVBoxLangFileExt = ".qm";
const char *gVBoxLangIDRegExp = "(([a-z]{2})(?:_([A-Z]{2}))?)|(C)";
const char *gVBoxBuiltInLangName   = "C";

class VBoxTranslator : public QTranslator
{
public:

    VBoxTranslator (QObject *aParent = 0)
        : QTranslator (aParent) {}

    bool loadFile (const QString &aFileName)
    {
        QFile file (aFileName);
        if (!file.open (QIODevice::ReadOnly))
            return false;
        mData = file.readAll();
        return load ((uchar*) mData.data(), mData.size());
    }

private:

    QByteArray mData;
};

static VBoxTranslator *sTranslator = 0;
static QString sLoadedLangId = gVBoxBuiltInLangName;

/**
 *  Returns the loaded (active) language ID.
 *  Note that it may not match with VBoxGlobalSettings::languageId() if the
 *  specified language cannot be loaded.
 *  If the built-in language is active, this method returns "C".
 *
 *  @note "C" is treated as the built-in language for simplicity -- the C
 *  locale is used in unix environments as a fallback when the requested
 *  locale is invalid. This way we don't need to process both the "built_in"
 *  language and the "C" language (which is a valid environment setting)
 *  separately.
 */
/* static */
QString VBoxGlobal::languageId()
{
    return sLoadedLangId;
}

/**
 *  Loads the language by language ID.
 *
 *  @param aLangId Language ID in in form of xx_YY. QString::null means the
 *                 system default language.
 */
/* static */
void VBoxGlobal::loadLanguage (const QString &aLangId)
{
    QString langId = aLangId.isEmpty() ?
        VBoxGlobal::systemLanguageId() : aLangId;
    QString languageFileName;
    QString selectedLangId = gVBoxBuiltInLangName;

    /* If C is selected we change it temporary to en. This makes sure any extra
     * "en" translation file will be loaded. This is necessary for loading the
     * plural forms of some of our translations. */
    bool fResetToC = false;
    if (langId == "C")
    {
        langId = "en";
        fResetToC = true;
    }

    char szNlsPath[RTPATH_MAX];
    int rc;

    rc = RTPathAppPrivateNoArch(szNlsPath, sizeof(szNlsPath));
    AssertRC (rc);

    QString nlsPath = QString(szNlsPath) + gVBoxLangSubDir;
    QDir nlsDir (nlsPath);

    Assert (!langId.isEmpty());
    if (!langId.isEmpty() && langId != gVBoxBuiltInLangName)
    {
        QRegExp regExp (gVBoxLangIDRegExp);
        int pos = regExp.indexIn (langId);
        /* the language ID should match the regexp completely */
        AssertReturnVoid (pos == 0);

        QString lang = regExp.cap (2);

        if (nlsDir.exists (gVBoxLangFileBase + langId + gVBoxLangFileExt))
        {
            languageFileName = nlsDir.absoluteFilePath (gVBoxLangFileBase + langId +
                                                        gVBoxLangFileExt);
            selectedLangId = langId;
        }
        else if (nlsDir.exists (gVBoxLangFileBase + lang + gVBoxLangFileExt))
        {
            languageFileName = nlsDir.absoluteFilePath (gVBoxLangFileBase + lang +
                                                        gVBoxLangFileExt);
            selectedLangId = lang;
        }
        else
        {
            /* Never complain when the default language is requested.  In any
             * case, if no explicit language file exists, we will simply
             * fall-back to English (built-in). */
            if (!aLangId.isNull() && langId != "en")
                msgCenter().cannotFindLanguage (langId, nlsPath);
            /* selectedLangId remains built-in here */
            AssertReturnVoid (selectedLangId == gVBoxBuiltInLangName);
        }
    }

    /* delete the old translator if there is one */
    if (sTranslator)
    {
        /* QTranslator destructor will call qApp->removeTranslator() for
         * us. It will also delete all its child translations we attach to it
         * below, so we don't have to care about them specially. */
        delete sTranslator;
    }

    /* load new language files */
    sTranslator = new VBoxTranslator (qApp);
    Assert (sTranslator);
    bool loadOk = true;
    if (sTranslator)
    {
        if (selectedLangId != gVBoxBuiltInLangName)
        {
            Assert (!languageFileName.isNull());
            loadOk = sTranslator->loadFile (languageFileName);
        }
        /* we install the translator in any case: on failure, this will
         * activate an empty translator that will give us English
         * (built-in) */
        qApp->installTranslator (sTranslator);
    }
    else
        loadOk = false;

    if (loadOk)
        sLoadedLangId = selectedLangId;
    else
    {
        msgCenter().cannotLoadLanguage (languageFileName);
        sLoadedLangId = gVBoxBuiltInLangName;
    }

    /* Try to load the corresponding Qt translation */
    if (sLoadedLangId != gVBoxBuiltInLangName)
    {
#ifdef Q_OS_UNIX
        /* We use system installations of Qt on Linux systems, so first, try
         * to load the Qt translation from the system location. */
        languageFileName = QLibraryInfo::location(QLibraryInfo::TranslationsPath) + "/qt_" +
                           sLoadedLangId + gVBoxLangFileExt;
        QTranslator *qtSysTr = new QTranslator (sTranslator);
        Assert (qtSysTr);
        if (qtSysTr && qtSysTr->load (languageFileName))
            qApp->installTranslator (qtSysTr);
        /* Note that the Qt translation supplied by Oracle is always loaded
         * afterwards to make sure it will take precedence over the system
         * translation (it may contain more decent variants of translation
         * that better correspond to VirtualBox UI). We need to load both
         * because a newer version of Qt may be installed on the user computer
         * and the Oracle version may not fully support it. We don't do it on
         * Win32 because we supply a Qt library there and therefore the
         * Oracle translation is always the best one. */
#endif
        languageFileName =  nlsDir.absoluteFilePath (QString ("qt_") +
                                                     sLoadedLangId +
                                                     gVBoxLangFileExt);
        QTranslator *qtTr = new QTranslator (sTranslator);
        Assert (qtTr);
        if (qtTr && (loadOk = qtTr->load (languageFileName)))
            qApp->installTranslator (qtTr);
        /* The below message doesn't fit 100% (because it's an additional
         * language and the main one won't be reset to built-in on failure)
         * but the load failure is so rare here that it's not worth a separate
         * message (but still, having something is better than having none) */
        if (!loadOk && !aLangId.isNull())
            msgCenter().cannotLoadLanguage (languageFileName);
    }
    if (fResetToC)
        sLoadedLangId = "C";
#ifdef Q_WS_MAC
    /* Qt doesn't translate the items in the Application menu initially.
     * Manually trigger an update. */
    ::darwinRetranslateAppMenu();
#endif /* Q_WS_MAC */
}

QString VBoxGlobal::helpFile() const
{
#if defined (Q_WS_WIN32)
    const QString name = "VirtualBox";
    const QString suffix = "chm";
#elif defined (Q_WS_MAC)
    const QString name = "UserManual";
    const QString suffix = "pdf";
#elif defined (Q_WS_X11)
# if defined VBOX_OSE
    const QString name = "UserManual";
    const QString suffix = "pdf";
# else
    const QString name = "VirtualBox";
    const QString suffix = "chm";
# endif
#endif
    /* Where are the docs located? */
    char szDocsPath[RTPATH_MAX];
    int rc = RTPathAppDocs (szDocsPath, sizeof (szDocsPath));
    AssertRC (rc);
    /* Make sure that the language is in two letter code.
     * Note: if languageId() returns an empty string lang.name() will
     * return "C" which is an valid language code. */
    QLocale lang (VBoxGlobal::languageId());

    /* Construct the path and the filename */
    QString manual = QString ("%1/%2_%3.%4").arg (szDocsPath)
                                            .arg (name)
                                            .arg (lang.name())
                                            .arg (suffix);
    /* Check if a help file with that name exists */
    QFileInfo fi (manual);
    if (fi.exists())
        return manual;

    /* Fall back to the standard */
    manual = QString ("%1/%2.%4").arg (szDocsPath)
                                 .arg (name)
                                 .arg (suffix);
    return manual;
}

/**
 *  Replacement for QToolButton::setTextLabel() that handles the shortcut
 *  letter (if it is present in the argument string) as if it were a setText()
 *  call: the shortcut letter is used to automatically assign an "Alt+<letter>"
 *  accelerator key sequence to the given tool button.
 *
 *  @note This method preserves the icon set if it was assigned before. Only
 *  the text label and the accelerator are changed.
 *
 *  @param aToolButton  Tool button to set the text label on.
 *  @param aTextLabel   Text label to set.
 */
/* static */
void VBoxGlobal::setTextLabel (QToolButton *aToolButton,
                               const QString &aTextLabel)
{
    AssertReturnVoid (aToolButton != NULL);

    /* remember the icon set as setText() will kill it */
    QIcon iset = aToolButton->icon();
    /* re-use the setText() method to detect and set the accelerator */
    aToolButton->setText (aTextLabel);
    QKeySequence accel = aToolButton->shortcut();
    aToolButton->setText (aTextLabel);
    aToolButton->setIcon (iset);
    /* set the accel last as setIconSet() would kill it */
    aToolButton->setShortcut (accel);
}

/**
 *  Performs direct and flipped search of position for \a aRectangle to make sure
 *  it is fully contained inside \a aBoundRegion region by moving & resizing
 *  \a aRectangle if necessary. Selects the minimum shifted result between direct
 *  and flipped variants.
 */
/* static */
QRect VBoxGlobal::normalizeGeometry (const QRect &aRectangle, const QRegion &aBoundRegion,
                                     bool aCanResize /* = true */)
{
    /* Direct search for normalized rectangle */
    QRect var1 (getNormalized (aRectangle, aBoundRegion, aCanResize));

    /* Flipped search for normalized rectangle */
    QRect var2 (flip (getNormalized (flip (aRectangle).boundingRect(),
                                     flip (aBoundRegion), aCanResize)).boundingRect());

    /* Calculate shift from starting position for both variants */
    double length1 = sqrt (pow ((double) (var1.x() - aRectangle.x()), (double) 2) +
                           pow ((double) (var1.y() - aRectangle.y()), (double) 2));
    double length2 = sqrt (pow ((double) (var2.x() - aRectangle.x()), (double) 2) +
                           pow ((double) (var2.y() - aRectangle.y()), (double) 2));

    /* Return minimum shifted variant */
    return length1 > length2 ? var2 : var1;
}

/**
 *  Ensures that the given rectangle \a aRectangle is fully contained within the
 *  region \a aBoundRegion by moving \a aRectangle if necessary. If \a aRectangle is
 *  larger than \a aBoundRegion, top left corner of \a aRectangle is aligned with the
 *  top left corner of maximum available rectangle and, if \a aCanResize is true,
 *  \a aRectangle is shrinked to become fully visible.
 */
/* static */
QRect VBoxGlobal::getNormalized (const QRect &aRectangle, const QRegion &aBoundRegion,
                                 bool /* aCanResize = true */)
{
    /* Storing available horizontal sub-rectangles & vertical shifts */
    int windowVertical = aRectangle.center().y();
    QVector <QRect> rectanglesVector (aBoundRegion.rects());
    QList <QRect> rectanglesList;
    QList <int> shiftsList;
    foreach (QRect currentItem, rectanglesVector)
    {
        int currentDelta = qAbs (windowVertical - currentItem.center().y());
        int shift2Top = currentItem.top() - aRectangle.top();
        int shift2Bot = currentItem.bottom() - aRectangle.bottom();

        int itemPosition = 0;
        foreach (QRect item, rectanglesList)
        {
            int delta = qAbs (windowVertical - item.center().y());
            if (delta > currentDelta) break; else ++ itemPosition;
        }
        rectanglesList.insert (itemPosition, currentItem);

        int shift2TopPos = 0;
        foreach (int shift, shiftsList)
            if (qAbs (shift) > qAbs (shift2Top)) break; else ++ shift2TopPos;
        shiftsList.insert (shift2TopPos, shift2Top);

        int shift2BotPos = 0;
        foreach (int shift, shiftsList)
            if (qAbs (shift) > qAbs (shift2Bot)) break; else ++ shift2BotPos;
        shiftsList.insert (shift2BotPos, shift2Bot);
    }

    /* Trying to find the appropriate place for window */
    QRect result;
    for (int i = -1; i < shiftsList.size(); ++ i)
    {
        /* Move to appropriate vertical */
        QRect rectangle (aRectangle);
        if (i >= 0) rectangle.translate (0, shiftsList [i]);

        /* Search horizontal shift */
        int maxShift = 0;
        foreach (QRect item, rectanglesList)
        {
            QRect trectangle (rectangle.translated (item.left() - rectangle.left(), 0));
            if (!item.intersects (trectangle))
                continue;

            if (rectangle.left() < item.left())
            {
                int shift = item.left() - rectangle.left();
                maxShift = qAbs (shift) > qAbs (maxShift) ? shift : maxShift;
            }
            else if (rectangle.right() > item.right())
            {
                int shift = item.right() - rectangle.right();
                maxShift = qAbs (shift) > qAbs (maxShift) ? shift : maxShift;
            }
        }

        /* Shift across the horizontal direction */
        rectangle.translate (maxShift, 0);

        /* Check the translated rectangle to feat the rules */
        if (aBoundRegion.united (rectangle) == aBoundRegion)
            result = rectangle;

        if (!result.isNull()) break;
    }

    if (result.isNull())
    {
        /* Resize window to feat desirable size
         * using max of available rectangles */
        QRect maxRectangle;
        quint64 maxSquare = 0;
        foreach (QRect item, rectanglesList)
        {
            quint64 square = item.width() * item.height();
            if (square > maxSquare)
            {
                maxSquare = square;
                maxRectangle = item;
            }
        }

        result = aRectangle;
        result.moveTo (maxRectangle.x(), maxRectangle.y());
        if (maxRectangle.right() < result.right())
            result.setRight (maxRectangle.right());
        if (maxRectangle.bottom() < result.bottom())
            result.setBottom (maxRectangle.bottom());
    }

    return result;
}

/**
 *  Returns the flipped (transposed) region.
 */
/* static */
QRegion VBoxGlobal::flip (const QRegion &aRegion)
{
    QRegion result;
    QVector <QRect> rectangles (aRegion.rects());
    foreach (QRect rectangle, rectangles)
        result += QRect (rectangle.y(), rectangle.x(),
                         rectangle.height(), rectangle.width());
    return result;
}

/**
 *  Aligns the center of \a aWidget with the center of \a aRelative.
 *
 *  If necessary, \a aWidget's position is adjusted to make it fully visible
 *  within the available desktop area. If \a aWidget is bigger then this area,
 *  it will also be resized unless \a aCanResize is false or there is an
 *  inappropriate minimum size limit (in which case the top left corner will be
 *  simply aligned with the top left corner of the available desktop area).
 *
 *  \a aWidget must be a top-level widget. \a aRelative may be any widget, but
 *  if it's not top-level itself, its top-level widget will be used for
 *  calculations. \a aRelative can also be NULL, in which case \a aWidget will
 *  be centered relative to the available desktop area.
 */
/* static */
void VBoxGlobal::centerWidget (QWidget *aWidget, QWidget *aRelative,
                               bool aCanResize /* = true */)
{
    AssertReturnVoid (aWidget);
    AssertReturnVoid (aWidget->isTopLevel());

    QRect deskGeo, parentGeo;
    QWidget *w = aRelative;
    if (w)
    {
        w = w->window();
        deskGeo = QApplication::desktop()->availableGeometry (w);
        parentGeo = w->frameGeometry();
        /* On X11/Gnome, geo/frameGeo.x() and y() are always 0 for top level
         * widgets with parents, what a shame. Use mapToGlobal() to workaround. */
        QPoint d = w->mapToGlobal (QPoint (0, 0));
        d.rx() -= w->geometry().x() - w->x();
        d.ry() -= w->geometry().y() - w->y();
        parentGeo.moveTopLeft (d);
    }
    else
    {
        deskGeo = QApplication::desktop()->availableGeometry();
        parentGeo = deskGeo;
    }

    /* On X11, there is no way to determine frame geometry (including WM
     * decorations) before the widget is shown for the first time. Stupidly
     * enumerate other top level widgets to find the thickest frame. The code
     * is based on the idea taken from QDialog::adjustPositionInternal(). */

    int extraw = 0, extrah = 0;

    QWidgetList list = QApplication::topLevelWidgets();
    QListIterator<QWidget*> it (list);
    while ((extraw == 0 || extrah == 0) && it.hasNext())
    {
        int framew, frameh;
        QWidget *current = it.next();
        if (!current->isVisible())
            continue;

        framew = current->frameGeometry().width() - current->width();
        frameh = current->frameGeometry().height() - current->height();

        extraw = qMax (extraw, framew);
        extrah = qMax (extrah, frameh);
    }

    /// @todo (r=dmik) not sure if we really need this
#if 0
    /* sanity check for decoration frames. With embedding, we
     * might get extraordinary values */
    if (extraw == 0 || extrah == 0 || extraw > 20 || extrah > 50)
    {
        extrah = 50;
        extraw = 20;
    }
#endif

    /* On non-X11 platforms, the following would be enough instead of the
     * above workaround: */
    // QRect geo = frameGeometry();
    QRect geo = QRect (0, 0, aWidget->width() + extraw,
                             aWidget->height() + extrah);

    geo.moveCenter (QPoint (parentGeo.x() + (parentGeo.width() - 1) / 2,
                            parentGeo.y() + (parentGeo.height() - 1) / 2));

    /* ensure the widget is within the available desktop area */
    QRect newGeo = normalizeGeometry (geo, deskGeo, aCanResize);
#ifdef Q_WS_MAC
    /* No idea why, but Qt doesn't respect if there is a unified toolbar on the
     * ::move call. So manually add the height of the toolbar before setting
     * the position. */
    if (w)
        newGeo.translate (0, ::darwinWindowToolBarHeight (aWidget));
#endif /* Q_WS_MAC */

    aWidget->move (newGeo.topLeft());

    if (aCanResize &&
        (geo.width() != newGeo.width() || geo.height() != newGeo.height()))
        aWidget->resize (newGeo.width() - extraw, newGeo.height() - extrah);
}

/**
 *  Returns the decimal separator for the current locale.
 */
/* static */
QChar VBoxGlobal::decimalSep()
{
    return QLocale::system().decimalPoint();
}

/**
 *  Returns the regexp string that defines the format of the human-readable
 *  size representation, <tt>####[.##] B|KB|MB|GB|TB|PB</tt>.
 *
 *  This regexp will capture 5 groups of text:
 *  - cap(1): integer number in case when no decimal point is present
 *            (if empty, it means that decimal point is present)
 *  - cap(2): size suffix in case when no decimal point is present (may be empty)
 *  - cap(3): integer number in case when decimal point is present (may be empty)
 *  - cap(4): fraction number (hundredth) in case when decimal point is present
 *  - cap(5): size suffix in case when decimal point is present (note that
 *            B cannot appear there)
 */
/* static */
QString VBoxGlobal::sizeRegexp()
{
    QString regexp =
        QString ("^(?:(?:(\\d+)(?:\\s?(%2|%3|%4|%5|%6|%7))?)|(?:(\\d*)%1(\\d{1,2})(?:\\s?(%3|%4|%5|%6|%7))))$")
            .arg (decimalSep())
            .arg (tr ("B", "size suffix Bytes"))
            .arg (tr ("KB", "size suffix KBytes=1024 Bytes"))
            .arg (tr ("MB", "size suffix MBytes=1024 KBytes"))
            .arg (tr ("GB", "size suffix GBytes=1024 MBytes"))
            .arg (tr ("TB", "size suffix TBytes=1024 GBytes"))
            .arg (tr ("PB", "size suffix PBytes=1024 TBytes"));
    return regexp;
}

/* static */
QString VBoxGlobal::toHumanReadableList(const QStringList &list)
{
    QString strList;
    for (int i = 0; i < list.size(); ++i)
    {
        strList += list.at(i);
        if (i < list.size() - 1)
            strList += + " ";
    }
    return strList;
}

/**
 *  Parses the given size string that should be in form of
 *  <tt>####[.##] B|KB|MB|GB|TB|PB</tt> and returns
 *  the size value in bytes. Zero is returned on error.
 */
/* static */
quint64 VBoxGlobal::parseSize (const QString &aText)
{
    QRegExp regexp (sizeRegexp());
    int pos = regexp.indexIn (aText);
    if (pos != -1)
    {
        QString intgS = regexp.cap (1);
        QString hundS;
        QString suff = regexp.cap (2);
        if (intgS.isEmpty())
        {
            intgS = regexp.cap (3);
            hundS = regexp.cap (4);
            suff = regexp.cap (5);
        }

        quint64 denom = 0;
        if (suff.isEmpty() || suff == tr ("B", "size suffix Bytes"))
            denom = 1;
        else if (suff == tr ("KB", "size suffix KBytes=1024 Bytes"))
            denom = _1K;
        else if (suff == tr ("MB", "size suffix MBytes=1024 KBytes"))
            denom = _1M;
        else if (suff == tr ("GB", "size suffix GBytes=1024 MBytes"))
            denom = _1G;
        else if (suff == tr ("TB", "size suffix TBytes=1024 GBytes"))
            denom = _1T;
        else if (suff == tr ("PB", "size suffix PBytes=1024 TBytes"))
            denom = _1P;

        quint64 intg = intgS.toULongLong();
        if (denom == 1)
            return intg;

        quint64 hund = hundS.leftJustified (2, '0').toULongLong();
        hund = hund * denom / 100;
        intg = intg * denom + hund;
        return intg;
    }
    else
        return 0;
}

/**
 * Formats the given @a aSize value in bytes to a human readable string
 * in form of <tt>####[.##] B|KB|MB|GB|TB|PB</tt>.
 *
 * The @a aMode and @a aDecimal parameters are used for rounding the resulting
 * number when converting the size value to KB, MB, etc gives a fractional part:
 * <ul>
 * <li>When \a aMode is FormatSize_Round, the result is rounded to the
 *     closest number containing \a aDecimal decimal digits.
 * </li>
 * <li>When \a aMode is FormatSize_RoundDown, the result is rounded to the
 *     largest number with \a aDecimal decimal digits that is not greater than
 *     the result. This guarantees that converting the resulting string back to
 *     the integer value in bytes will not produce a value greater that the
 *     initial size parameter.
 * </li>
 * <li>When \a aMode is FormatSize_RoundUp, the result is rounded to the
 *     smallest number with \a aDecimal decimal digits that is not less than the
 *     result. This guarantees that converting the resulting string back to the
 *     integer value in bytes will not produce a value less that the initial
 *     size parameter.
 * </li>
 * </ul>
 *
 * @param aSize     Size value in bytes.
 * @param aMode     Conversion mode.
 * @param aDecimal  Number of decimal digits in result.
 * @return          Human-readable size string.
 */
/* static */
QString VBoxGlobal::formatSize (quint64 aSize, uint aDecimal /* = 2 */,
                                FormatSize aMode /* = FormatSize_Round */)
{
    static QString Suffixes [7];
    Suffixes[0] = tr ("B", "size suffix Bytes");
    Suffixes[1] = tr ("KB", "size suffix KBytes=1024 Bytes");
    Suffixes[2] = tr ("MB", "size suffix MBytes=1024 KBytes");
    Suffixes[3] = tr ("GB", "size suffix GBytes=1024 MBytes");
    Suffixes[4] = tr ("TB", "size suffix TBytes=1024 GBytes");
    Suffixes[5] = tr ("PB", "size suffix PBytes=1024 TBytes");
    Suffixes[6] = (const char *)NULL;
    AssertCompile(6 < RT_ELEMENTS (Suffixes));

    quint64 denom = 0;
    int suffix = 0;

    if (aSize < _1K)
    {
        denom = 1;
        suffix = 0;
    }
    else if (aSize < _1M)
    {
        denom = _1K;
        suffix = 1;
    }
    else if (aSize < _1G)
    {
        denom = _1M;
        suffix = 2;
    }
    else if (aSize < _1T)
    {
        denom = _1G;
        suffix = 3;
    }
    else if (aSize < _1P)
    {
        denom = _1T;
        suffix = 4;
    }
    else
    {
        denom = _1P;
        suffix = 5;
    }

    quint64 intg = aSize / denom;
    quint64 decm = aSize % denom;
    quint64 mult = 1;
    for (uint i = 0; i < aDecimal; ++ i) mult *= 10;

    QString number;
    if (denom > 1)
    {
        if (decm)
        {
            decm *= mult;
            /* not greater */
            if (aMode == FormatSize_RoundDown)
                decm = decm / denom;
            /* not less */
            else if (aMode == FormatSize_RoundUp)
                decm = (decm + denom - 1) / denom;
            /* nearest */
            else decm = (decm + denom / 2) / denom;
        }
        /* check for the fractional part overflow due to rounding */
        if (decm == mult)
        {
            decm = 0;
            ++ intg;
            /* check if we've got 1024 XB after rounding and scale down if so */
            if (intg == 1024 && Suffixes [suffix + 1] != NULL)
            {
                intg /= 1024;
                ++ suffix;
            }
        }
        number = QString::number (intg);
        if (aDecimal) number += QString ("%1%2").arg (decimalSep())
            .arg (QString::number (decm).rightJustified (aDecimal, '0'));
    }
    else
    {
        number = QString::number (intg);
    }

    return QString ("%1 %2").arg (number).arg (Suffixes [suffix]);
}

/**
 *  Returns the required video memory in bytes for the current desktop
 *  resolution at maximum possible screen depth in bpp.
 */
/* static */
quint64 VBoxGlobal::requiredVideoMemory(const QString &strGuestOSTypeId, int cMonitors /* = 1 */)
{
    QDesktopWidget *pDW = QApplication::desktop();
    /* We create a list of the size of all available host monitors. This list
     * is sorted by value and by starting with the biggest one, we calculate
     * the memory requirements for every guest screen. This is of course not
     * correct, but as we can't predict on which host screens the user will
     * open the guest windows, this is the best assumption we can do, cause it
     * is the worst case. */
    QVector<int> screenSize(qMax(cMonitors, pDW->numScreens()), 0);
    for (int i = 0; i < pDW->numScreens(); ++i)
    {
        QRect r = pDW->screenGeometry(i);
        screenSize[i] = r.width() * r.height();
    }
    /* Now sort the vector */
    qSort(screenSize.begin(), screenSize.end(), qGreater<int>());
    /* For the case that there are more guest screens configured then host
     * screens available, replace all zeros with the greatest value in the
     * vector. */
    for (int i = 0; i < screenSize.size(); ++i)
        if (screenSize.at(i) == 0)
            screenSize.replace(i, screenSize.at(0));

    quint64 needBits = 0;
    for (int i = 0; i < cMonitors; ++i)
    {
        /* Calculate summary required memory amount in bits */
        needBits += (screenSize.at(i) * /* with x height */
                     32 + /* we will take the maximum possible bpp for now */
                     8 * _1M) + /* current cache per screen - may be changed in future */
                    8 * 4096; /* adapter info */
    }
    /* Translate value into megabytes with rounding to highest side */
    quint64 needMBytes = needBits % (8 * _1M) ? needBits / (8 * _1M) + 1 :
                         needBits / (8 * _1M) /* convert to megabytes */;

    if (strGuestOSTypeId.startsWith("Windows"))
    {
       /* Windows guests need offscreen VRAM too for graphics acceleration features: */
#ifdef VBOX_WITH_CRHGSMI
       if (isWddmCompatibleOsType(strGuestOSTypeId))
       {
           /* wddm mode, there are two surfaces for each screen: shadow & primary */
           needMBytes *= 3;
       }
       else
#endif /* VBOX_WITH_CRHGSMI */
       {
           needMBytes *= 2;
       }
    }

    return needMBytes * _1M;
}

/**
 * Puts soft hyphens after every path component in the given file name.
 *
 * @param aFileName File name (must be a full path name).
 */
/* static */
QString VBoxGlobal::locationForHTML (const QString &aFileName)
{
/// @todo (dmik) remove?
//    QString result = QDir::toNativeSeparators (fn);
//#ifdef Q_OS_LINUX
//    result.replace ('/', "/<font color=red>&shy;</font>");
//#else
//    result.replace ('\\', "\\<font color=red>&shy;</font>");
//#endif
//    return result;
    QFileInfo fi (aFileName);
    return fi.fileName();
}

/**
 *  Reformats the input string @a aStr so that:
 *  - strings in single quotes will be put inside <nobr> and marked
 *    with blue color;
 *  - UUIDs be put inside <nobr> and marked
 *    with green color;
 *  - replaces new line chars with </p><p> constructs to form paragraphs
 *    (note that <p> and </p> are not appended to the beginning and to the
 *     end of the string respectively, to allow the result be appended
 *     or prepended to the existing paragraph).
 *
 *  If @a aToolTip is true, colouring is not applied, only the <nobr> tag
 *  is added. Also, new line chars are replaced with <br> instead of <p>.
 */
/* static */
QString VBoxGlobal::highlight (const QString &aStr, bool aToolTip /* = false */)
{
    QString strFont;
    QString uuidFont;
    QString endFont;
    if (!aToolTip)
    {
        strFont = "<font color=#0000CC>";
        uuidFont = "<font color=#008000>";
        endFont = "</font>";
    }

    QString text = aStr;

    /* replace special entities, '&' -- first! */
    text.replace ('&', "&amp;");
    text.replace ('<', "&lt;");
    text.replace ('>', "&gt;");
    text.replace ('\"', "&quot;");

    /* mark strings in single quotes with color */
    QRegExp rx = QRegExp ("((?:^|\\s)[(]?)'([^']*)'(?=[:.-!);]?(?:\\s|$))");
    rx.setMinimal (true);
    text.replace (rx,
        QString ("\\1%1<nobr>'\\2'</nobr>%2").arg (strFont).arg (endFont));

    /* mark UUIDs with color */
    text.replace (QRegExp (
        "((?:^|\\s)[(]?)"
        "(\\{[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}\\})"
        "(?=[:.-!);]?(?:\\s|$))"),
        QString ("\\1%1<nobr>\\2</nobr>%2").arg (uuidFont).arg (endFont));

    /* split to paragraphs at \n chars */
    if (!aToolTip)
        text.replace ('\n', "</p><p>");
    else
        text.replace ('\n', "<br>");

    return text;
}

/* static */
QString VBoxGlobal::replaceHtmlEntities(QString strText)
{
    return strText
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('\"', "&quot;");
}

/**
 *  Reformats the input string @a aStr so that:
 *  - strings in single quotes will be put inside <nobr> and marked
 *    with bold style;
 *  - UUIDs be put inside <nobr> and marked
 *    with italic style;
 *  - replaces new line chars with </p><p> constructs to form paragraphs
 *    (note that <p> and </p> are not appended to the beginning and to the
 *     end of the string respectively, to allow the result be appended
 *     or prepended to the existing paragraph).
 */
/* static */
QString VBoxGlobal::emphasize (const QString &aStr)
{
    QString strEmphStart ("<b>");
    QString strEmphEnd ("</b>");
    QString uuidEmphStart ("<i>");
    QString uuidEmphEnd ("</i>");

    QString text = aStr;

    /* replace special entities, '&' -- first! */
    text.replace ('&', "&amp;");
    text.replace ('<', "&lt;");
    text.replace ('>', "&gt;");
    text.replace ('\"', "&quot;");

    /* mark strings in single quotes with bold style */
    QRegExp rx = QRegExp ("((?:^|\\s)[(]?)'([^']*)'(?=[:.-!);]?(?:\\s|$))");
    rx.setMinimal (true);
    text.replace (rx,
        QString ("\\1%1<nobr>'\\2'</nobr>%2").arg (strEmphStart).arg (strEmphEnd));

    /* mark UUIDs with italic style */
    text.replace (QRegExp (
        "((?:^|\\s)[(]?)"
        "(\\{[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}\\})"
        "(?=[:.-!);]?(?:\\s|$))"),
        QString ("\\1%1<nobr>\\2</nobr>%2").arg (uuidEmphStart).arg (uuidEmphEnd));

    /* split to paragraphs at \n chars */
    text.replace ('\n', "</p><p>");

    return text;
}

/**
 *  This does exactly the same as QLocale::system().name() but corrects its
 *  wrong behavior on Linux systems (LC_NUMERIC for some strange reason takes
 *  precedence over any other locale setting in the QLocale::system()
 *  implementation). This implementation first looks at LC_ALL (as defined by
 *  SUS), then looks at LC_MESSAGES which is designed to define a language for
 *  program messages in case if it differs from the language for other locale
 *  categories. Then it looks for LANG and finally falls back to
 *  QLocale::system().name().
 *
 *  The order of precedence is well defined here:
 *  http://opengroup.org/onlinepubs/007908799/xbd/envvar.html
 *
 *  @note This method will return "C" when the requested locale is invalid or
 *  when the "C" locale is set explicitly.
 */
/* static */
QString VBoxGlobal::systemLanguageId()
{
#if defined (Q_WS_MAC)
    /* QLocale return the right id only if the user select the format of the
     * language also. So we use our own implementation */
    return ::darwinSystemLanguage();
#elif defined (Q_OS_UNIX)
    const char *s = RTEnvGet ("LC_ALL");
    if (s == 0)
        s = RTEnvGet ("LC_MESSAGES");
    if (s == 0)
        s = RTEnvGet ("LANG");
    if (s != 0)
        return QLocale (s).name();
#endif
    return  QLocale::system().name();
}

#if defined (Q_WS_X11)

static char *XXGetProperty (Display *aDpy, Window aWnd,
                            Atom aPropType, const char *aPropName)
{
    Atom propNameAtom = XInternAtom (aDpy, aPropName,
                                     True /* only_if_exists */);
    if (propNameAtom == None)
        return NULL;

    Atom actTypeAtom = None;
    int actFmt = 0;
    unsigned long nItems = 0;
    unsigned long nBytesAfter = 0;
    unsigned char *propVal = NULL;
    int rc = XGetWindowProperty (aDpy, aWnd, propNameAtom,
                                 0, LONG_MAX, False /* delete */,
                                 aPropType, &actTypeAtom, &actFmt,
                                 &nItems, &nBytesAfter, &propVal);
    if (rc != Success)
        return NULL;

    return reinterpret_cast <char *> (propVal);
}

static Bool XXSendClientMessage (Display *aDpy, Window aWnd, const char *aMsg,
                                 unsigned long aData0 = 0, unsigned long aData1 = 0,
                                 unsigned long aData2 = 0, unsigned long aData3 = 0,
                                 unsigned long aData4 = 0)
{
    Atom msgAtom = XInternAtom (aDpy, aMsg, True /* only_if_exists */);
    if (msgAtom == None)
        return False;

    XEvent ev;

    ev.xclient.type = ClientMessage;
    ev.xclient.serial = 0;
    ev.xclient.send_event = True;
    ev.xclient.display = aDpy;
    ev.xclient.window = aWnd;
    ev.xclient.message_type = msgAtom;

    /* always send as 32 bit for now */
    ev.xclient.format = 32;
    ev.xclient.data.l [0] = aData0;
    ev.xclient.data.l [1] = aData1;
    ev.xclient.data.l [2] = aData2;
    ev.xclient.data.l [3] = aData3;
    ev.xclient.data.l [4] = aData4;

    return XSendEvent (aDpy, DefaultRootWindow (aDpy), False,
                       SubstructureRedirectMask, &ev) != 0;
}

#endif

/**
 * Activates the specified window. If necessary, the window will be
 * de-iconified activation.
 *
 * @note On X11, it is implied that @a aWid represents a window of the same
 * display the application was started on.
 *
 * @param aWId              Window ID to activate.
 * @param aSwitchDesktop    @c true to switch to the window's desktop before
 *                          activation.
 *
 * @return @c true on success and @c false otherwise.
 */
/* static */
bool VBoxGlobal::activateWindow (WId aWId, bool aSwitchDesktop /* = true */)
{
    bool result = true;

#if defined (Q_WS_WIN32)

    if (IsIconic (aWId))
        result &= !!ShowWindow (aWId, SW_RESTORE);
    else if (!IsWindowVisible (aWId))
        result &= !!ShowWindow (aWId, SW_SHOW);

    result &= !!SetForegroundWindow (aWId);

#elif defined (Q_WS_X11)

    Display *dpy = QX11Info::display();

    if (aSwitchDesktop)
    {
        /* try to find the desktop ID using the NetWM property */
        CARD32 *desktop = (CARD32 *) XXGetProperty (dpy, aWId, XA_CARDINAL,
                                                    "_NET_WM_DESKTOP");
        if (desktop == NULL)
            /* if the NetWM properly is not supported try to find the desktop
             * ID using the GNOME WM property */
            desktop = (CARD32 *) XXGetProperty (dpy, aWId, XA_CARDINAL,
                                                "_WIN_WORKSPACE");

        if (desktop != NULL)
        {
            Bool ok = XXSendClientMessage (dpy, DefaultRootWindow (dpy),
                                           "_NET_CURRENT_DESKTOP",
                                           *desktop);
            if (!ok)
            {
                LogWarningFunc (("Couldn't switch to desktop=%08X\n",
                                 desktop));
                result = false;
            }
            XFree (desktop);
        }
        else
        {
            LogWarningFunc (("Couldn't find a desktop ID for aWId=%08X\n",
                             aWId));
            result = false;
        }
    }

    Bool ok = XXSendClientMessage (dpy, aWId, "_NET_ACTIVE_WINDOW");
    result &= !!ok;

    XRaiseWindow (dpy, aWId);

#else

    NOREF (aWId);
    NOREF (aSwitchDesktop);
    AssertFailed();
    result = false;

#endif

    if (!result)
        LogWarningFunc (("Couldn't activate aWId=%08X\n", aWId));

    return result;
}

/**
 *  Removes the accelerator mark (the ampersand symbol) from the given string
 *  and returns the result. The string is supposed to be a menu item's text
 *  that may (or may not) contain the accelerator mark.
 *
 *  In order to support accelerators used in non-alphabet languages
 *  (e.g. Japanese) that has a form of "(&<L>)" (where <L> is a latin letter),
 *  this method first searches for this pattern and, if found, removes it as a
 *  whole. If such a pattern is not found, then the '&' character is simply
 *  removed from the string.
 *
 *  @note This function removes only the first occurrence of the accelerator
 *  mark.
 *
 *  @param aText Menu item's text to remove the accelerator mark from.
 *
 *  @return The resulting string.
 */
/* static */
QString VBoxGlobal::removeAccelMark (const QString &aText)
{
    QString result = aText;

    QRegExp accel ("\\(&[a-zA-Z]\\)");
    int pos = accel.indexIn (result);
    if (pos >= 0)
        result.remove (pos, accel.cap().length());
    else
    {
        pos = result.indexOf ('&');
        if (pos >= 0)
            result.remove (pos, 1);
    }

    return result;
}

/* static */
QString VBoxGlobal::insertKeyToActionText(const QString &strText, const QString &strKey)
{
#ifdef Q_WS_MAC
    QString pattern("%1 (Host+%2)");
#else
    QString pattern("%1 \tHost+%2");
#endif
    if (   strKey.isEmpty()
        || strKey.compare("None", Qt::CaseInsensitive) == 0)
        return strText;
    else
        return pattern.arg(strText).arg(QKeySequence(strKey).toString(QKeySequence::NativeText));
}

/* static */
QString VBoxGlobal::extractKeyFromActionText (const QString &aText)
{
    QString key;
#ifdef Q_WS_MAC
    QRegExp re (".* \\(Host\\+(.+)\\)");
#else
    QRegExp re (".* \\t\\Host\\+(.+)");
#endif
    if (re.exactMatch (aText))
        key = re.cap (1);
    return key;
}

/**
 * Joins two pixmaps horizontally with 2px space between them and returns the
 * result.
 *
 * @param aPM1 Left pixmap.
 * @param aPM2 Right pixmap.
 */
/* static */
QPixmap VBoxGlobal::joinPixmaps (const QPixmap &aPM1, const QPixmap &aPM2)
{
    if (aPM1.isNull())
        return aPM2;
    if (aPM2.isNull())
        return aPM1;

    QPixmap result (aPM1.width() + aPM2.width() + 2,
                    qMax (aPM1.height(), aPM2.height()));
    result.fill (Qt::transparent);

    QPainter painter (&result);
    painter.drawPixmap (0, 0, aPM1);
    painter.drawPixmap (aPM1.width() + 2, result.height() - aPM2.height(), aPM2);
    painter.end();

    return result;
}

/**
 *  Searches for a widget that with @a aName (if it is not NULL) which inherits
 *  @a aClassName (if it is not NULL) and among children of @a aParent. If @a
 *  aParent is NULL, all top-level widgets are searched. If @a aRecursive is
 *  true, child widgets are recursively searched as well.
 */
/* static */
QWidget *VBoxGlobal::findWidget (QWidget *aParent, const char *aName,
                                 const char *aClassName /* = NULL */,
                                 bool aRecursive /* = false */)
{
    if (aParent == NULL)
    {
        QWidgetList list = QApplication::topLevelWidgets();
        foreach(QWidget *w, list)
        {
            if ((!aName || strcmp (w->objectName().toAscii().constData(), aName) == 0) &&
                (!aClassName || strcmp (w->metaObject()->className(), aClassName) == 0))
                return w;
            if (aRecursive)
            {
                w = findWidget (w, aName, aClassName, aRecursive);
                if (w)
                    return w;
            }
        }
        return NULL;
    }

    /* Find the first children of aParent with the appropriate properties.
     * Please note that this call is recursively. */
    QList<QWidget *> list = qFindChildren<QWidget *> (aParent, aName);
    foreach(QWidget *child, list)
    {
        if (!aClassName || strcmp (child->metaObject()->className(), aClassName) == 0)
            return child;
    }
    return NULL;
}

/**
 * Figures out which medium formats are currently supported by VirtualBox for
 * the given device type.
 * Returned is a list of pairs with the form
 *   <tt>{"Backend Name", "*.suffix1 .suffix2 ..."}</tt>.
 */
/* static */
QList <QPair <QString, QString> > VBoxGlobal::MediumBackends(KDeviceType enmType)
{
    CSystemProperties systemProperties = vboxGlobal().virtualBox().GetSystemProperties();
    QVector<CMediumFormat> mediumFormats = systemProperties.GetMediumFormats();
    QList< QPair<QString, QString> > backendPropList;
    for (int i = 0; i < mediumFormats.size(); ++ i)
    {
        /* File extensions */
        QVector <QString> fileExtensions;
        QVector <KDeviceType> deviceTypes;

        mediumFormats [i].DescribeFileExtensions(fileExtensions, deviceTypes);

        QStringList f;
        for (int a = 0; a < fileExtensions.size(); ++ a)
            if (deviceTypes [a] == enmType)
                f << QString ("*.%1").arg (fileExtensions [a]);
        /* Create a pair out of the backend description and all suffix's. */
        if (!f.isEmpty())
            backendPropList << QPair<QString, QString> (mediumFormats [i].GetName(), f.join(" "));
    }
    return backendPropList;
}

/**
 * Figures out which hard disk formats are currently supported by VirtualBox.
 * Returned is a list of pairs with the form
 *   <tt>{"Backend Name", "*.suffix1 .suffix2 ..."}</tt>.
 */
/* static */
QList <QPair <QString, QString> > VBoxGlobal::HDDBackends()
{
    return MediumBackends(KDeviceType_HardDisk);
}

/**
 * Figures out which CD/DVD disk formats are currently supported by VirtualBox.
 * Returned is a list of pairs with the form
 *   <tt>{"Backend Name", "*.suffix1 .suffix2 ..."}</tt>.
 */
/* static */
QList <QPair <QString, QString> > VBoxGlobal::DVDBackends()
{
    return MediumBackends(KDeviceType_DVD);
}

/**
 * Figures out which floppy disk formats are currently supported by VirtualBox.
 * Returned is a list of pairs with the form
 *   <tt>{"Backend Name", "*.suffix1 .suffix2 ..."}</tt>.
 */
/* static */
QList <QPair <QString, QString> > VBoxGlobal::FloppyBackends()
{
    return MediumBackends(KDeviceType_Floppy);
}

/* static */
QString VBoxGlobal::documentsPath()
{
    QString path;
#if QT_VERSION < 0x040400
    path = QDir::homePath();
#else
    path = QDesktopServices::storageLocation (QDesktopServices::DocumentsLocation);
#endif

    /* Make sure the path exists */
    QDir dir (path);
    if (dir.exists())
        return QDir::cleanPath (dir.canonicalPath());
    else
    {
        dir.setPath (QDir::homePath() + "/Documents");
        if (dir.exists())
            return QDir::cleanPath (dir.canonicalPath());
        else
            return QDir::homePath();
    }
}

#ifdef VBOX_WITH_VIDEOHWACCEL
/* static */
bool VBoxGlobal::isAcceleration2DVideoAvailable()
{
    return VBoxQGLOverlay::isAcceleration2DVideoAvailable();
}

/** additional video memory required for the best 2D support performance
 *  total amount of VRAM required is thus calculated as requiredVideoMemory + required2DOffscreenVideoMemory  */
/* static */
quint64 VBoxGlobal::required2DOffscreenVideoMemory()
{
    return VBoxQGLOverlay::required2DOffscreenVideoMemory();
}

#endif

#ifdef VBOX_WITH_CRHGSMI
/* static */
quint64 VBoxGlobal::required3DWddmOffscreenVideoMemory(const QString &strGuestOSTypeId, int cMonitors /* = 1 */)
{
    cMonitors = RT_MAX(cMonitors, 1);
    quint64 cbSize = VBoxGlobal::requiredVideoMemory(strGuestOSTypeId, 1); /* why not cMonitors? */
    cbSize += 64 * _1M;
    return cbSize;
}

/* static */
bool VBoxGlobal::isWddmCompatibleOsType(const QString &strGuestOSTypeId)
{
    return    strGuestOSTypeId.startsWith("WindowsVista")
           || strGuestOSTypeId.startsWith("Windows7")
           || strGuestOSTypeId.startsWith("Windows8")
           || strGuestOSTypeId.startsWith("Windows2008")
           || strGuestOSTypeId.startsWith("Windows2012");
}
#endif /* VBOX_WITH_CRHGSMI */

#ifdef Q_WS_MAC
bool VBoxGlobal::isSheetWindowAllowed(QWidget *pParent) const
{
    /* Disallow for null parent: */
    if (!pParent)
        return false;

    /* Make sure Mac Sheet is not used for the same parent now. */
    if (sheetWindowUsed(pParent))
        return false;

    /* No sheets for fullscreen/seamless now.
     * Firstly it looks ugly and secondly in some cases it is broken. */
    if (!(qobject_cast<UIMachineWindowFullscreen*>(pParent) ||
          qobject_cast<UIMachineWindowSeamless*>(pParent)))
        return true;

    return false;
}

void VBoxGlobal::setSheetWindowUsed(QWidget *pParent, bool fUsed)
{
    /* Ignore null parent: */
    if (!pParent)
        return;

    if (fUsed)
    {
        AssertMsg(!m_sheets.contains(pParent), ("Trying to use Mac Sheet for parent which already has one!"));
        if (m_sheets.contains(pParent))
            return;
    }
    else
    {
        AssertMsg(m_sheets.contains(pParent), ("Trying to cancel use Mac Sheet for parent which has no one!"));
        if (!m_sheets.contains(pParent))
            return;
    }

    if (fUsed)
        m_sheets.insert(pParent);
    else
        m_sheets.remove(pParent);
}

bool VBoxGlobal::sheetWindowUsed(QWidget *pParent) const
{
    return m_sheets.contains(pParent);
}
#endif /* Q_WS_MAC */

/* static */
QString VBoxGlobal::fullMediumFormatName(const QString &strBaseMediumFormatName)
{
    if (strBaseMediumFormatName == "VDI")
        return tr("VDI (VirtualBox Disk Image)");
    else if (strBaseMediumFormatName == "VMDK")
        return tr("VMDK (Virtual Machine Disk)");
    else if (strBaseMediumFormatName == "VHD")
        return tr("VHD (Virtual Hard Disk)");
    else if (strBaseMediumFormatName == "Parallels")
        return tr("HDD (Parallels Hard Disk)");
    else if (strBaseMediumFormatName == "QED")
        return tr("QED (QEMU enhanced disk)");
    else if (strBaseMediumFormatName == "QCOW")
        return tr("QCOW (QEMU Copy-On-Write)");
    return strBaseMediumFormatName;
}

/* static */
bool VBoxGlobal::isApprovedByExtraData(CMachine &machine, const QString &strExtraDataKey)
{
    /* Load corresponding extra-data value: */
    QString strExtraDataValue(machine.GetExtraData(strExtraDataKey));

    /* 'false' if value was not set: */
    if (strExtraDataValue.isEmpty())
        return false;

    /* Handle particular values: */
    return    strExtraDataValue.compare("true", Qt::CaseInsensitive) == 0
           || strExtraDataValue.compare("yes", Qt::CaseInsensitive) == 0
           || strExtraDataValue.compare("on", Qt::CaseInsensitive) == 0
           || strExtraDataValue == "1";
}

/* static */
bool VBoxGlobal::shouldWeShowMachine(CMachine &machine)
{
    /* 'false' for null machines: */
    if (machine.isNull())
        return false;

    /* 'true' for inaccessible machines,
     * because we can't verify anything in that case: */
    if (!machine.GetAccessible())
        return true;

    /* 'true' if hiding is not approved by the extra-data: */
    return !isApprovedByExtraData(machine, GUI_HideFromManager);
}

/* static */
bool VBoxGlobal::shouldWeAllowMachineReconfiguration(CMachine &machine,
                                                     bool fIncludingMachineGeneralCheck /*= false*/,
                                                     bool fIncludingMachineStateCheck /*= false*/)
{
    /* Should we perform machine general check? */
    if (fIncludingMachineGeneralCheck)
    {
        /* 'false' for null machines: */
        if (machine.isNull())
            return false;

        /* 'false' for inaccessible machines,
         * because we can't configure anything in that case: */
        if (!machine.GetAccessible())
            return true;
    }

    /* Should we perform machine state check? */
    if (fIncludingMachineStateCheck)
    {
        /* 'false' for machines in [stuck] state: */
        if (machine.GetState() == KMachineState_Stuck)
            return false;
    }

    /* 'true' if reconfiguration is not restricted by the extra-data: */
    return !isApprovedByExtraData(machine, GUI_PreventReconfiguration);
}

/* static */
bool VBoxGlobal::shouldWeShowDetails(CMachine &machine,
                                     bool fIncludingMachineGeneralCheck /*= false*/)
{
    /* Should we perform machine general check? */
    if (fIncludingMachineGeneralCheck)
    {
        /* 'false' for null machines: */
        if (machine.isNull())
            return false;

        /* 'true' for inaccessible machines,
         * because we can't verify anything in that case: */
        if (!machine.GetAccessible())
            return true;
    }

    /* 'true' if hiding is not approved by the extra-data: */
    return !isApprovedByExtraData(machine, GUI_HideDetails);
}

// Public slots
////////////////////////////////////////////////////////////////////////////////

/**
 * Opens the specified URL using OS/Desktop capabilities.
 *
 * @param aURL URL to open
 *
 * @return true on success and false otherwise
 */
bool VBoxGlobal::openURL (const QString &aURL)
{
    /* Service event */
    class ServiceEvent : public QEvent
    {
        public:

            ServiceEvent (bool aResult) : QEvent (QEvent::User), mResult (aResult) {}

            bool result() const { return mResult; }

        private:

            bool mResult;
    };

    /* Service-Client object */
    class ServiceClient : public QEventLoop
    {
        public:

            ServiceClient() : mResult (false) {}

            bool result() const { return mResult; }

        private:

            bool event (QEvent *aEvent)
            {
                if (aEvent->type() == QEvent::User)
                {
                    ServiceEvent *pEvent = static_cast <ServiceEvent*> (aEvent);
                    mResult = pEvent->result();
                    pEvent->accept();
                    quit();
                    return true;
                }
                return false;
            }

            bool mResult;
    };

    /* Service-Server object */
    class ServiceServer : public QThread
    {
        public:

            ServiceServer (ServiceClient &aClient, const QString &sURL)
                : mClient (aClient), mURL (sURL) {}

        private:

            void run()
            {
                QApplication::postEvent (&mClient, new ServiceEvent (QDesktopServices::openUrl (mURL)));
            }

            ServiceClient &mClient;
            const QString &mURL;
    };

    ServiceClient client;
    ServiceServer server (client, aURL);
    server.start();
    client.exec();
    server.wait();

    bool result = client.result();

    if (!result)
        msgCenter().cannotOpenURL (aURL);

    return result;
}

/**
 * Shows the VirtualBox registration dialog.
 *
 * @note that this method is not part of UIMessageCenter (like e.g.
 *       UIMessageCenter::sltShowHelpAboutDialog()) because it is tied to
 *       VBoxCallback::OnExtraDataChange() handling performed by VBoxGlobal.
 *
 * @param aForce
 */
void VBoxGlobal::showRegistrationDialog (bool aForce)
{
    NOREF(aForce);
#ifdef VBOX_WITH_REGISTRATION
    if (!aForce && !UIRegistrationWzd::hasToBeShown())
        return;

    if (mRegDlg)
    {
        /* Show the already opened registration dialog */
        mRegDlg->setWindowState (mRegDlg->windowState() & ~Qt::WindowMinimized);
        mRegDlg->raise();
        mRegDlg->activateWindow();
    }
    else
    {
        /* Store the ID of the main window to ensure that only one
         * registration dialog is shown at a time. Due to manipulations with
         * OnExtraDataCanChange() and OnExtraDataChange() signals, this extra
         * data item acts like an inter-process mutex, so the first process
         * that attempts to set it will win, the rest will get a failure from
         * the SetExtraData() call. */
        mVBox.SetExtraData (GUI_RegistrationDlgWinID,
                            QString ("%1").arg ((qulonglong) mMainWindow->winId()));

        if (mVBox.isOk())
        {
            /* We've got the "mutex", create a new registration dialog */
            UIRegistrationWzd *dlg = new UIRegistrationWzd (&mRegDlg);
            dlg->setAttribute (Qt::WA_DeleteOnClose);
            Assert (dlg == mRegDlg);
            mRegDlg->show();
        }
    }
#endif
}

void VBoxGlobal::sltGUILanguageChange(QString strLang)
{
    loadLanguage(strLang);
}

void VBoxGlobal::sltProcessGlobalSettingChange()
{
    /* Reload proxy settings: */
    reloadProxySettings();
}

// Protected members
////////////////////////////////////////////////////////////////////////////////

bool VBoxGlobal::event (QEvent *e)
{
    switch (e->type())
    {
        case MediaEnumEventType:
        {
            VBoxMediaEnumEvent *ev = (VBoxMediaEnumEvent*) e;

            if (!ev->mLast)
            {
                if (ev->mMedium.state() == KMediumState_Inaccessible &&
                    !ev->mMedium.result().isOk())
                    msgCenter().cannotGetMediaAccessibility (ev->mMedium);
                Assert (ev->mIterator != mMediaList.end());
                *(ev->mIterator) = ev->mMedium;
                emit mediumEnumerated (*ev->mIterator);
            }
            else
            {
                /* the thread has posted the last message, wait for termination */
                mMediaEnumThread->wait();
                delete mMediaEnumThread;
                mMediaEnumThread = 0;
                emit mediumEnumFinished (mMediaList);
            }

            return true;
        }

        default:
            break;
    }

    return QObject::event (e);
}

bool VBoxGlobal::eventFilter (QObject *aObject, QEvent *aEvent)
{
    if (aEvent->type() == QEvent::LanguageChange &&
        aObject->isWidgetType() &&
        static_cast <QWidget *> (aObject)->isTopLevel())
    {
        /* Catch the language change event before any other widget gets it in
         * order to invalidate cached string resources (like the details view
         * templates) that may be used by other widgets. */
        QWidgetList list = QApplication::topLevelWidgets();
        if (list.first() == aObject)
        {
            /* call this only once per every language change (see
             * QApplication::installTranslator() for details) */
            retranslateUi();
        }
    }

    return QObject::eventFilter (aObject, aEvent);
}

#ifdef VBOX_WITH_DEBUGGER_GUI

bool VBoxGlobal::isDebuggerEnabled(CMachine &aMachine)
{
    return isDebuggerWorker(&mDbgEnabled, aMachine, GUI_DbgEnabled);
}

bool VBoxGlobal::isDebuggerAutoShowEnabled(CMachine &aMachine)
{
    return isDebuggerWorker(&mDbgAutoShow, aMachine, GUI_DbgAutoShow);
}

bool VBoxGlobal::isDebuggerAutoShowCommandLineEnabled(CMachine &aMachine)
{
    return isDebuggerWorker(&mDbgAutoShowCommandLine, aMachine, GUI_DbgAutoShow);
}

bool VBoxGlobal::isDebuggerAutoShowStatisticsEnabled(CMachine &aMachine)
{
    return isDebuggerWorker(&mDbgAutoShowStatistics, aMachine, GUI_DbgAutoShow);
}

#endif /* VBOX_WITH_DEBUGGER_GUI */

// Private members
////////////////////////////////////////////////////////////////////////////////

bool VBoxGlobal::processArgs()
{
    bool fResult = false;
    QStringList args = qApp->arguments();
    QList<QUrl> list;
    for (int i = 1; i < args.size(); ++i)
    {
        /* We break out after the first parameter, cause there could be
           parameters with arguments (e.g. --comment comment). */
        if (args.at(i).startsWith("-"))
            break;
#ifdef Q_WS_MAC
        QString strArg = ::darwinResolveAlias(args.at(i));
#else /* Q_WS_MAC */
        QString strArg = args.at(i);
#endif /* !Q_WS_MAC */
        if (   !strArg.isEmpty()
            && QFile::exists(strArg))
            list << QUrl::fromLocalFile(strArg);
    }
    if (!list.isEmpty())
    {
        for (int i = 0; i < list.size(); ++i)
        {
            const QString& strFile = list.at(i).toLocalFile();
            if (VBoxGlobal::hasAllowedExtension(strFile, VBoxFileExts))
            {
                CVirtualBox vbox = vboxGlobal().virtualBox();
                CMachine machine = vbox.FindMachine(strFile);
                if (!machine.isNull())
                {
                    fResult = true;
                    launchMachine(machine);
                    /* Remove from the arg list. */
                    list.removeAll(strFile);
                }
            }
        }
    }
    if (!list.isEmpty())
    {
        m_ArgUrlList = list;
        QTimer::singleShot(0, &vboxGlobal().selectorWnd(), SLOT(sltOpenUrls()));
    }
    return fResult;
}

void VBoxGlobal::init()
{
#ifdef DEBUG
    mVerString += " [DEBUG]";
#endif

    HRESULT rc = COMBase::InitializeCOM(true);
    if (FAILED (rc))
    {
#ifdef VBOX_WITH_XPCOM
        if (rc == NS_ERROR_FILE_ACCESS_DENIED)
        {
            char szHome[RTPATH_MAX] = "";
            com::GetVBoxUserHomeDirectory(szHome, sizeof(szHome));
            msgCenter().cannotInitUserHome(QString(szHome));
        }
        else
#endif
            msgCenter().cannotInitCOM(rc);
        return;
    }

    mVBox.createInstance (CLSID_VirtualBox);
    if (!mVBox.isOk())
    {
        msgCenter().cannotCreateVirtualBox (mVBox);
        return;
    }
    mHost = virtualBox().GetHost();
#ifdef VBOX_WITH_CROGL
    m3DAvailable = VBoxOglIs3DAccelerationSupported();
#else
    m3DAvailable = false;
#endif

    /* create default non-null global settings */
    gset = VBoxGlobalSettings (false);

    /* try to load global settings */
    gset.load (mVBox);
    if (!mVBox.isOk() || !gset)
    {
        msgCenter().cannotLoadGlobalConfig (mVBox, gset.lastError());
        return;
    }

    /* Load the customized language as early as possible to get possible error
     * messages translated */
    QString sLanguageId = gset.languageId();
    if (!sLanguageId.isNull())
        loadLanguage (sLanguageId);

    retranslateUi();

    connect(gEDataEvents, SIGNAL(sigGUILanguageChange(QString)),
            this, SLOT(sltGUILanguageChange(QString)));

#ifdef VBOX_GUI_WITH_SYSTRAY
    {
        /* Increase open Fe/Qt4 windows reference count. */
        int c = mVBox.GetExtraData (GUI_MainWindowCount).toInt() + 1;
        AssertMsgReturnVoid ((c >= 0) || (mVBox.isOk()),
            ("Something went wrong with the window reference count!"));
        mVBox.SetExtraData (GUI_MainWindowCount, QString ("%1").arg (c));
        mIncreasedWindowCounter = mVBox.isOk();
        AssertReturnVoid (mIncreasedWindowCounter);
    }
#endif

    /* Initialize guest OS Type list. */
    CGuestOSTypeVector coll = mVBox.GetGuestOSTypes();
    int osTypeCount = coll.size();
    AssertMsg (osTypeCount > 0, ("Number of OS types must not be zero"));
    if (osTypeCount > 0)
    {
        /* Here we assume the 'Other' type is always the first, so we
         * remember it and will append it to the list when finished. */
        CGuestOSType otherType = coll[0];
        QString otherFamilyId (otherType.GetFamilyId());

        /* Fill the lists with all the available OS Types except
         * the 'Other' type, which will be appended. */
        for (int i = 1; i < coll.size(); ++i)
        {
            CGuestOSType os = coll[i];
            QString familyId (os.GetFamilyId());
            if (!mFamilyIDs.contains (familyId))
            {
                mFamilyIDs << familyId;
                mTypes << QList <CGuestOSType> ();
            }
            mTypes [mFamilyIDs.indexOf (familyId)].append (os);
        }

        /* Append the 'Other' OS Type to the end of list. */
        if (!mFamilyIDs.contains (otherFamilyId))
        {
            mFamilyIDs << otherFamilyId;
            mTypes << QList <CGuestOSType> ();
        }
        mTypes [mFamilyIDs.indexOf (otherFamilyId)].append (otherType);
    }

    /* Fill in OS type icon dictionary. */
    static const char *kOSTypeIcons [][2] =
    {
        {"Other",           ":/os_other.png"},
        {"DOS",             ":/os_dos.png"},
        {"Netware",         ":/os_netware.png"},
        {"L4",              ":/os_l4.png"},
        {"Windows31",       ":/os_win31.png"},
        {"Windows95",       ":/os_win95.png"},
        {"Windows98",       ":/os_win98.png"},
        {"WindowsMe",       ":/os_winme.png"},
        {"WindowsNT4",      ":/os_winnt4.png"},
        {"Windows2000",     ":/os_win2k.png"},
        {"WindowsXP",       ":/os_winxp.png"},
        {"WindowsXP_64",    ":/os_winxp_64.png"},
        {"Windows2003",     ":/os_win2k3.png"},
        {"Windows2003_64",  ":/os_win2k3_64.png"},
        {"WindowsVista",    ":/os_winvista.png"},
        {"WindowsVista_64", ":/os_winvista_64.png"},
        {"Windows2008",     ":/os_win2k8.png"},
        {"Windows2008_64",  ":/os_win2k8_64.png"},
        {"Windows7",        ":/os_win7.png"},
        {"Windows7_64",     ":/os_win7_64.png"},
        {"Windows8",        ":/os_win8.png"},
        {"Windows8_64",     ":/os_win8_64.png"},
        {"Windows2012_64",  ":/os_win2k12_64.png"},
        {"WindowsNT",       ":/os_win_other.png"},
        {"OS2Warp3",        ":/os_os2warp3.png"},
        {"OS2Warp4",        ":/os_os2warp4.png"},
        {"OS2Warp45",       ":/os_os2warp45.png"},
        {"OS2eCS",          ":/os_os2ecs.png"},
        {"OS2",             ":/os_os2_other.png"},
        {"Linux22",         ":/os_linux22.png"},
        {"Linux24",         ":/os_linux24.png"},
        {"Linux24_64",      ":/os_linux24_64.png"},
        {"Linux26",         ":/os_linux26.png"},
        {"Linux26_64",      ":/os_linux26_64.png"},
        {"ArchLinux",       ":/os_archlinux.png"},
        {"ArchLinux_64",    ":/os_archlinux_64.png"},
        {"Debian",          ":/os_debian.png"},
        {"Debian_64",       ":/os_debian_64.png"},
        {"OpenSUSE",        ":/os_opensuse.png"},
        {"OpenSUSE_64",     ":/os_opensuse_64.png"},
        {"Fedora",          ":/os_fedora.png"},
        {"Fedora_64",       ":/os_fedora_64.png"},
        {"Gentoo",          ":/os_gentoo.png"},
        {"Gentoo_64",       ":/os_gentoo_64.png"},
        {"Mandriva",        ":/os_mandriva.png"},
        {"Mandriva_64",     ":/os_mandriva_64.png"},
        {"RedHat",          ":/os_redhat.png"},
        {"RedHat_64",       ":/os_redhat_64.png"},
        {"Turbolinux",      ":/os_turbolinux.png"},
        {"Turbolinux_64",   ":/os_turbolinux_64.png"},
        {"Ubuntu",          ":/os_ubuntu.png"},
        {"Ubuntu_64",       ":/os_ubuntu_64.png"},
        {"Xandros",         ":/os_xandros.png"},
        {"Xandros_64",      ":/os_xandros_64.png"},
        {"Oracle",          ":/os_oracle.png"},
        {"Oracle_64",       ":/os_oracle_64.png"},
        {"Linux",           ":/os_linux_other.png"},
        {"FreeBSD",         ":/os_freebsd.png"},
        {"FreeBSD_64",      ":/os_freebsd_64.png"},
        {"OpenBSD",         ":/os_openbsd.png"},
        {"OpenBSD_64",      ":/os_openbsd_64.png"},
        {"NetBSD",          ":/os_netbsd.png"},
        {"NetBSD_64",       ":/os_netbsd_64.png"},
        {"Solaris",         ":/os_solaris.png"},
        {"Solaris_64",      ":/os_solaris_64.png"},
        {"OpenSolaris",     ":/os_oraclesolaris.png"},
        {"OpenSolaris_64",  ":/os_oraclesolaris_64.png"},
        {"Solaris11_64",    ":/os_oraclesolaris_64.png"},
        {"QNX",             ":/os_qnx.png"},
        {"MacOS",           ":/os_macosx.png"},
        {"MacOS_64",        ":/os_macosx_64.png"},
        {"JRockitVE",       ":/os_jrockitve.png"},
    };
    for (uint n = 0; n < SIZEOF_ARRAY (kOSTypeIcons); ++ n)
    {
        mOsTypeIcons.insert (kOSTypeIcons [n][0],
            new QPixmap (kOSTypeIcons [n][1]));
    }

    /* online/offline snapshot icons */
    mOfflineSnapshotIcon = QPixmap (":/offline_snapshot_16px.png");
    mOnlineSnapshotIcon = QPixmap (":/online_snapshot_16px.png");

    qApp->installEventFilter (this);

    /* process command line */

    bool bForceSeamless = false;
    bool bForceFullscreen = false;

    vm_render_mode_str = RTStrDup (virtualBox()
            .GetExtraData (GUI_RenderMode).toAscii().constData());

#ifdef Q_WS_X11
    mIsKWinManaged = X11IsWindowManagerKWin();
#endif

#ifdef VBOX_WITH_DEBUGGER_GUI
# ifdef VBOX_WITH_DEBUGGER_GUI_MENU
    initDebuggerVar(&mDbgEnabled, "VBOX_GUI_DBG_ENABLED", GUI_DbgEnabled, true);
# else
    initDebuggerVar(&mDbgEnabled, "VBOX_GUI_DBG_ENABLED", GUI_DbgEnabled, false);
# endif
    initDebuggerVar(&mDbgAutoShow, "VBOX_GUI_DBG_AUTO_SHOW", GUI_DbgAutoShow, false);
    mDbgAutoShowCommandLine = mDbgAutoShowStatistics = mDbgAutoShow;
    mStartPaused = false;
#endif

    mShowStartVMErrors = true;
    bool startVM = false;
    QString vmNameOrUuid;

    int argc = qApp->argc();
    int i = 1;
    while (i < argc)
    {
        const char *arg = qApp->argv() [i];
        /* NOTE: the check here must match the corresponding check for the
         * options to start a VM in main.cpp and hardenedmain.cpp exactly,
         * otherwise there will be weird error messages. */
        if (   !::strcmp (arg, "--startvm")
            || !::strcmp (arg, "-startvm"))
        {
            if (++i < argc)
            {
                vmNameOrUuid = QString (qApp->argv() [i]);
                startVM = true;
            }
        }
#ifdef VBOX_GUI_WITH_PIDFILE
        else if (!::strcmp(arg, "-pidfile") || !::strcmp(arg, "--pidfile"))
        {
            if (++i < argc)
                m_strPidfile = QString(qApp->argv()[i]);
        }
#endif /* VBOX_GUI_WITH_PIDFILE */
        else if (!::strcmp(arg, "-seamless") || !::strcmp(arg, "--seamless"))
        {
            bForceSeamless = true;
        }
        else if (!::strcmp(arg, "-fullscreen") || !::strcmp(arg, "--fullscreen"))
        {
            bForceFullscreen = true;
        }
#ifdef VBOX_GUI_WITH_SYSTRAY
        else if (!::strcmp (arg, "-systray") || !::strcmp (arg, "--systray"))
        {
            mIsTrayMenu = true;
        }
#endif
        else if (!::strcmp (arg, "-comment") || !::strcmp (arg, "--comment"))
        {
            ++i;
        }
        else if (!::strcmp (arg, "-rmode") || !::strcmp (arg, "--rmode"))
        {
            if (++i < argc)
                vm_render_mode_str = qApp->argv() [i];
        }
        else if (!::strcmp (arg, "--settingspw"))
        {
            if (++i < argc)
            {
                RTStrCopy(mSettingsPw, sizeof(mSettingsPw), qApp->argv() [i]);
                mSettingsPwSet = true;
            }
        }
        else if (!::strcmp (arg, "--settingspwfile"))
        {
            if (++i < argc)
            {
                size_t cbFile;
                char *pszFile = qApp->argv() [i];
                bool fStdIn = !::strcmp(pszFile, "stdin");
                int vrc = VINF_SUCCESS;
                PRTSTREAM pStrm;
                if (!fStdIn)
                    vrc = RTStrmOpen(pszFile, "r", &pStrm);
                else
                    pStrm = g_pStdIn;
                if (RT_SUCCESS(vrc))
                {
                    vrc = RTStrmReadEx(pStrm, mSettingsPw, sizeof(mSettingsPw)-1, &cbFile);
                    if (RT_SUCCESS(vrc))
                    {
                        if (cbFile >= sizeof(mSettingsPw)-1)
                            continue;
                        else
                        {
                            unsigned i;
                            for (i = 0; i < cbFile && !RT_C_IS_CNTRL(mSettingsPw[i]); i++)
                                ;
                            mSettingsPw[i] = '\0';
                            mSettingsPwSet = true;
                        }
                    }
                    if (!fStdIn)
                        RTStrmClose(pStrm);
                }
            }
        }
        else if (!::strcmp (arg, "--no-startvm-errormsgbox"))
            mShowStartVMErrors = false;
        else if (!::strcmp(arg, "--disable-patm"))
            mDisablePatm = true;
        else if (!::strcmp(arg, "--disable-csam"))
            mDisableCsam = true;
        else if (!::strcmp(arg, "--recompile-supervisor"))
            mRecompileSupervisor = true;
        else if (!::strcmp(arg, "--recompile-user"))
            mRecompileUser = true;
        else if (!::strcmp(arg, "--recompile-all"))
            mDisablePatm = mDisableCsam = mRecompileSupervisor = mRecompileUser = true;
        else if (!::strcmp(arg, "--warp-pct"))
        {
            if (++i < argc)
                mWarpPct = RTStrToUInt32(qApp->argv() [i]);
        }
#ifdef VBOX_WITH_DEBUGGER_GUI
        else if (!::strcmp (arg, "-dbg") || !::strcmp (arg, "--dbg"))
            setDebuggerVar(&mDbgEnabled, true);
        else if (!::strcmp( arg, "-debug") || !::strcmp (arg, "--debug"))
        {
            setDebuggerVar(&mDbgEnabled, true);
            setDebuggerVar(&mDbgAutoShow, true);
            setDebuggerVar(&mDbgAutoShowCommandLine, true);
            setDebuggerVar(&mDbgAutoShowStatistics, true);
            mStartPaused = true;
        }
        else if (!::strcmp (arg, "--debug-command-line"))
        {
            setDebuggerVar(&mDbgEnabled, true);
            setDebuggerVar(&mDbgAutoShow, true);
            setDebuggerVar(&mDbgAutoShowCommandLine, true);
            mStartPaused = true;
        }
        else if (!::strcmp (arg, "--debug-statistics"))
        {
            setDebuggerVar(&mDbgEnabled, true);
            setDebuggerVar(&mDbgAutoShow, true);
            setDebuggerVar(&mDbgAutoShowStatistics, true);
            mStartPaused = true;
        }
        else if (!::strcmp (arg, "-no-debug") || !::strcmp (arg, "--no-debug"))
        {
            setDebuggerVar(&mDbgEnabled, false);
            setDebuggerVar(&mDbgAutoShow, false);
            setDebuggerVar(&mDbgAutoShowCommandLine, false);
            setDebuggerVar(&mDbgAutoShowStatistics, false);
        }
        /* Not quite debug options, but they're only useful with the debugger bits. */
        else if (!::strcmp (arg, "--start-paused"))
            mStartPaused = true;
        else if (!::strcmp (arg, "--start-running"))
            mStartPaused = false;
#endif
        /** @todo add an else { msgbox(syntax error); exit(1); } here, pretty please... */
        i++;
    }

    if (startVM)
    {
        QUuid uuid = QUuid(vmNameOrUuid);
        if (!uuid.isNull())
        {
            vmUuid = vmNameOrUuid;
        }
        else
        {
            CMachine m = mVBox.FindMachine (vmNameOrUuid);
            if (m.isNull())
            {
                if (showStartVMErrors())
                    msgCenter().cannotFindMachineByName (mVBox, vmNameOrUuid);
                return;
            }
            vmUuid = m.GetId();
        }
    }

    if (mSettingsPwSet)
        mVBox.SetSettingsSecret(mSettingsPw);

    if (bForceSeamless && !vmUuid.isEmpty())
    {
        mVBox.FindMachine(vmUuid).SetExtraData(GUI_Seamless, "on");
    }
    else if (bForceFullscreen && !vmUuid.isEmpty())
    {
        mVBox.FindMachine(vmUuid).SetExtraData(GUI_Fullscreen, "on");
    }

    vm_render_mode = vboxGetRenderMode (vm_render_mode_str);

#ifdef VBOX_WITH_DEBUGGER_GUI
    /* setup the debugger gui. */
    if (RTEnvExist("VBOX_GUI_NO_DEBUGGER"))
        mDbgEnabled = mDbgAutoShow =  mDbgAutoShowCommandLine = mDbgAutoShowStatistics = false;
    if (mDbgEnabled)
    {
        RTERRINFOSTATIC ErrInfo;
        RTErrInfoInitStatic(&ErrInfo);
        int vrc = SUPR3HardenedLdrLoadAppPriv("VBoxDbg", &mhVBoxDbg, RTLDRLOAD_FLAGS_LOCAL, &ErrInfo.Core);
        if (RT_FAILURE(vrc))
        {
            mhVBoxDbg = NIL_RTLDRMOD;
            mDbgAutoShow =  mDbgAutoShowCommandLine = mDbgAutoShowStatistics = false;
            LogRel(("Failed to load VBoxDbg, rc=%Rrc - %s\n", vrc, ErrInfo.Core.pszMsg));
        }
    }
#endif

    mValid = true;

    UIConverter::prepare();

    /* Cache IMedium data.
     * There could be no used mediums at all,
     * but this method should be run anyway just to enumerate null UIMedium object,
     * used by some VBox smart widgets, like VBoxMediaComboBox: */
    vboxGlobal().startEnumeratingMedia();

    /* Prepare global settings change handler: */
    connect(&settings(), SIGNAL(propertyChanged(const char*, const char*)),
            this, SLOT(sltProcessGlobalSettingChange()));
    /* Handle global settings change for the first time: */
    sltProcessGlobalSettingChange();

    /* Create action pool: */
    if (isVMConsoleProcess())
        UIActionPoolRuntime::create();
    else
        UIActionPoolSelector::create();

    /* Create network manager: */
    UINetworkManager::create();

    /* Schedule update manager: */
    UIUpdateManager::schedule();
}


/** @internal
 *
 *  This method should be never called directly. It is called automatically
 *  when the application terminates.
 */
void VBoxGlobal::cleanup()
{
    /* Shutdown update manager: */
    UIUpdateManager::shutdown();

    /* Destroy network manager: */
    UINetworkManager::destroy();

    /* Destroy action pool: */
    if (isVMConsoleProcess())
        UIActionPoolRuntime::destroy();
    else
        UIActionPoolSelector::destroy();

    /* sanity check */
    if (!sVBoxGlobalInCleanup)
    {
        AssertMsgFailed (("Should never be called directly\n"));
        return;
    }

#ifdef VBOX_GUI_WITH_SYSTRAY
    if (mIncreasedWindowCounter)
    {
        /* Decrease open Fe/Qt4 windows reference count. */
        int c = mVBox.GetExtraData (GUI_MainWindowCount).toInt() - 1;
        AssertMsg ((c >= 0) || (mVBox.isOk()),
            ("Something went wrong with the window reference count!"));
        if (c < 0)
            c = 0;   /* Clean up the mess. */
        mVBox.SetExtraData (GUI_MainWindowCount,
                            (c > 0) ? QString ("%1").arg (c) : NULL);
        AssertWrapperOk (mVBox);
        if (c == 0)
        {
            mVBox.SetExtraData (GUI_TrayIconWinID, NULL);
            AssertWrapperOk (mVBox);
        }
    }
#endif

#ifdef VBOX_GUI_WITH_PIDFILE
    deletePidfile();
#endif

    /* Destroy our event handlers */
    UIExtraDataEventHandler::destroy();

    /* Cleanup medium enumeration thread: */
    if (mMediaEnumThread)
    {
        /* sVBoxGlobalInCleanup is true here, so just wait for the thread */
        mMediaEnumThread->wait();
        delete mMediaEnumThread;
        mMediaEnumThread = 0;
    }

#ifdef VBOX_WITH_REGISTRATION
    if (mRegDlg)
        mRegDlg->close();
#endif

    if (mSelectorWnd)
        delete mSelectorWnd;
    if (m_pVirtualMachine)
        delete m_pVirtualMachine;

    UIConverter::cleanup();

    /* ensure CGuestOSType objects are no longer used */
    mFamilyIDs.clear();
    mTypes.clear();

    /* media list contains a lot of CUUnknown, release them */
    mMediaList.clear();
    /* the last steps to ensure we don't use COM any more */
    mHost.detach();
    mVBox.detach();

    /* There may be VBoxMediaEnumEvent instances still in the message
     * queue which reference COM objects. Remove them to release those objects
     * before uninitializing the COM subsystem. */
    QApplication::removePostedEvents (this);

    COMBase::CleanupCOM();

    mValid = false;
}

#ifdef VBOX_WITH_DEBUGGER_GUI

# define VBOXGLOBAL_DBG_CFG_VAR_FALSE       (0)
# define VBOXGLOBAL_DBG_CFG_VAR_TRUE        (1)
# define VBOXGLOBAL_DBG_CFG_VAR_MASK        (1)
# define VBOXGLOBAL_DBG_CFG_VAR_CMD_LINE    RT_BIT(3)
# define VBOXGLOBAL_DBG_CFG_VAR_DONE        RT_BIT(4)

/**
 * Initialize a debugger config variable.
 *
 * @param   piDbgCfgVar         The debugger config variable to init.
 * @param   pszEnvVar           The environment variable name relating to this
 *                              variable.
 * @param   pszExtraDataName    The extra data name relating to this variable.
 * @param   fDefault            The default value.
 */
void VBoxGlobal::initDebuggerVar(int *piDbgCfgVar, const char *pszEnvVar, const char *pszExtraDataName, bool fDefault)
{
    QString strEnvValue;
    char    szEnvValue[256];
    int rc = RTEnvGetEx(RTENV_DEFAULT, pszEnvVar, szEnvValue, sizeof(szEnvValue), NULL);
    if (RT_SUCCESS(rc))
    {
        strEnvValue = QString::fromUtf8(&szEnvValue[0]).toLower().trimmed();
        if (strEnvValue.isEmpty())
            strEnvValue = "yes";
    }
    else if (rc != VERR_ENV_VAR_NOT_FOUND)
        strEnvValue = "veto";

    QString     strExtraValue = mVBox.GetExtraData(pszExtraDataName).toLower().trimmed();
    if (strExtraValue.isEmpty())
        strExtraValue = QString();

    if ( strEnvValue.contains("veto") || strExtraValue.contains("veto"))
        *piDbgCfgVar = VBOXGLOBAL_DBG_CFG_VAR_DONE | VBOXGLOBAL_DBG_CFG_VAR_FALSE;
    else if (strEnvValue.isNull() && strExtraValue.isNull())
        *piDbgCfgVar = fDefault ? VBOXGLOBAL_DBG_CFG_VAR_TRUE : VBOXGLOBAL_DBG_CFG_VAR_FALSE;
    else
    {
        QString *pStr = !strEnvValue.isEmpty() ? &strEnvValue : &strExtraValue;
        if (   pStr->startsWith("y")  // yes
            || pStr->startsWith("e")  // enabled
            || pStr->startsWith("t")  // true
            || pStr->startsWith("on")
            || pStr->toLongLong() != 0)
            *piDbgCfgVar = VBOXGLOBAL_DBG_CFG_VAR_TRUE;
        else if (   pStr->startsWith("n")  // o
                 || pStr->startsWith("d")  // disable
                 || pStr->startsWith("f")  // false
                 || pStr->startsWith("off")
                 || pStr->contains("veto")
                 || pStr->toLongLong() == 0)
            *piDbgCfgVar = VBOXGLOBAL_DBG_CFG_VAR_FALSE;
        else
        {
            LogFunc(("Ignoring unknown value '%s' for '%s'\n", pStr->toAscii().constData(), pStr == &strEnvValue ? pszEnvVar : pszExtraDataName));
            *piDbgCfgVar = fDefault ? VBOXGLOBAL_DBG_CFG_VAR_TRUE : VBOXGLOBAL_DBG_CFG_VAR_FALSE;
        }
    }
}

/**
 * Set a debugger config variable according according to start up argument.
 *
 * @param   piDbgCfgVar         The debugger config variable to set.
 * @param   fState              The value from the command line.
 */
void VBoxGlobal::setDebuggerVar(int *piDbgCfgVar, bool fState)
{
    if (!(*piDbgCfgVar & VBOXGLOBAL_DBG_CFG_VAR_DONE))
        *piDbgCfgVar = (fState ? VBOXGLOBAL_DBG_CFG_VAR_TRUE : VBOXGLOBAL_DBG_CFG_VAR_FALSE)
                     | VBOXGLOBAL_DBG_CFG_VAR_CMD_LINE;
}

/**
 * Checks the state of a debugger config variable, updating it with the machine
 * settings on the first invocation.
 *
 * @returns true / false.
 * @param   piDbgCfgVar         The debugger config variable to consult.
 * @param   rMachine            Reference to the machine object.
 * @param   pszExtraDataName    The extra data name relating to this variable.
 */
bool VBoxGlobal::isDebuggerWorker(int *piDbgCfgVar, CMachine &rMachine, const char *pszExtraDataName)
{
    if (!(*piDbgCfgVar & VBOXGLOBAL_DBG_CFG_VAR_DONE) && !rMachine.isNull())
    {
        QString str = mVBox.GetExtraData(pszExtraDataName).toLower().trimmed();
        if (str.contains("veto"))
            *piDbgCfgVar = VBOXGLOBAL_DBG_CFG_VAR_DONE | VBOXGLOBAL_DBG_CFG_VAR_FALSE;
        else if (str.isEmpty() || (*piDbgCfgVar & VBOXGLOBAL_DBG_CFG_VAR_CMD_LINE))
            *piDbgCfgVar |= VBOXGLOBAL_DBG_CFG_VAR_DONE;
        else if (   str.startsWith("y")  // yes
                 || str.startsWith("e")  // enabled
                 || str.startsWith("t")  // true
                 || str.startsWith("on")
                 || str.toLongLong() != 0)
            *piDbgCfgVar = VBOXGLOBAL_DBG_CFG_VAR_DONE | VBOXGLOBAL_DBG_CFG_VAR_TRUE;
        else if (   str.startsWith("n")  // no
                 || str.startsWith("d")  // disable
                 || str.startsWith("f")  // false
                 || str.toLongLong() == 0)
            *piDbgCfgVar = VBOXGLOBAL_DBG_CFG_VAR_DONE | VBOXGLOBAL_DBG_CFG_VAR_FALSE;
        else
            *piDbgCfgVar |= VBOXGLOBAL_DBG_CFG_VAR_DONE;
    }

    return (*piDbgCfgVar & VBOXGLOBAL_DBG_CFG_VAR_MASK) == VBOXGLOBAL_DBG_CFG_VAR_TRUE;
}

#endif /* VBOX_WITH_DEBUGGER_GUI */

/** @fn vboxGlobal
 *
 *  Shortcut to the static VBoxGlobal::instance() method, for convenience.
 */

bool VBoxGlobal::switchToMachine(CMachine &machine)
{
#ifdef Q_WS_MAC
    ULONG64 id = machine.ShowConsoleWindow();
#else
    WId id = (WId) machine.ShowConsoleWindow();
#endif
    AssertWrapperOk(machine);
    if (!machine.isOk())
        return false;

    /* winId = 0 it means the console window has already done everything
     * necessary to implement the "show window" semantics. */
    if (id == 0)
        return true;

#if defined (Q_WS_WIN32) || defined (Q_WS_X11)

    return vboxGlobal().activateWindow(id, true);

#elif defined (Q_WS_MAC)
    /*
     * This is just for the case were the other process cannot steal
     * the focus from us. It will send us a PSN so we can try.
     */
    ProcessSerialNumber psn;
    psn.highLongOfPSN = id >> 32;
    psn.lowLongOfPSN = (UInt32)id;
    OSErr rc = ::SetFrontProcess(&psn);
    if (!rc)
        Log(("GUI: %#RX64 couldn't do SetFrontProcess on itself, the selector (we) had to do it...\n", id));
    else
        Log(("GUI: Failed to bring %#RX64 to front. rc=%#x\n", id, rc));
    return !rc;

#endif

    return false;

    /// @todo Below is the old method of switching to the console window
    //  based on the process ID of the console process. It should go away
    //  after the new (callback-based) method is fully tested.
#if 0

    if (!canSwitchTo())
        return false;

#if defined (Q_WS_WIN32)

    HWND hwnd = mWinId;

    /* if there are blockers (modal and modeless dialogs, etc), find the
     * topmost one */
    HWND hwndAbove = NULL;
    do
    {
        hwndAbove = GetNextWindow(hwnd, GW_HWNDPREV);
        HWND hwndOwner;
        if (hwndAbove != NULL &&
            ((hwndOwner = GetWindow(hwndAbove, GW_OWNER)) == hwnd ||
             hwndOwner  == hwndAbove))
            hwnd = hwndAbove;
        else
            break;
    }
    while (1);

    /* first, check that the primary window is visible */
    if (IsIconic(mWinId))
        ShowWindow(mWinId, SW_RESTORE);
    else if (!IsWindowVisible(mWinId))
        ShowWindow(mWinId, SW_SHOW);

#if 0
    LogFlowFunc(("mWinId=%08X hwnd=%08X\n", mWinId, hwnd));
#endif

    /* then, activate the topmost in the group */
    AllowSetForegroundWindow(m_pid);
    SetForegroundWindow(hwnd);

    return true;

#elif defined (Q_WS_X11)

    return false;

#elif defined (Q_WS_MAC)

    ProcessSerialNumber psn;
    OSStatus rc = ::GetProcessForPID(m_pid, &psn);
    if (!rc)
    {
        rc = ::SetFrontProcess(&psn);

        if (!rc)
        {
            ShowHideProcess(&psn, true);
            return true;
        }
    }
    return false;

#else

    return false;

#endif

#endif
}

bool VBoxGlobal::launchMachine(CMachine &machine, bool fHeadless /* = false */)
{
    if (machine.CanShowConsoleWindow())
        return VBoxGlobal::switchToMachine(machine);

    KMachineState state = machine.GetState(); NOREF(state);
    AssertMsg(   state == KMachineState_PoweredOff
              || state == KMachineState_Saved
              || state == KMachineState_Teleported
              || state == KMachineState_Aborted
              , ("Machine must be PoweredOff/Saved/Aborted (%d)", state));

    CVirtualBox vbox = vboxGlobal().virtualBox();
    CSession session;
    session.createInstance(CLSID_Session);
    if (session.isNull())
    {
        msgCenter().cannotOpenSession(session);
        return false;
    }

#if defined(Q_OS_WIN32)
    /* allow the started VM process to make itself the foreground window */
    AllowSetForegroundWindow(ASFW_ANY);
#endif

    QString env;
#if defined(Q_WS_X11)
    /* make sure the VM process will start on the same display as the Selector */
    const char *display = RTEnvGet("DISPLAY");
    if (display)
        env.append(QString("DISPLAY=%1\n").arg(display));
    const char *xauth = RTEnvGet("XAUTHORITY");
    if (xauth)
        env.append(QString("XAUTHORITY=%1\n").arg(xauth));
#endif
    const QString strType = fHeadless ? "headless" : "GUI/Qt";

    CProgress progress = machine.LaunchVMProcess(session, strType, env);
    if (   !vbox.isOk()
        || progress.isNull())
    {
        msgCenter().cannotOpenSession(vbox, machine);
        return false;
    }

    /* Hide the "VM spawning" progress dialog */
    /* I hope 1 minute will be enough to spawn any running VM silently, isn't it? */
    int iSpawningDuration = 60000;
    msgCenter().showModalProgressDialog(progress, machine.GetName(), "", 0, false, iSpawningDuration);
    if (progress.GetResultCode() != 0)
        msgCenter().cannotOpenSession(vbox, machine, progress);

    session.UnlockMachine();

    return true;
}
