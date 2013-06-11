/* $Id: main.cpp $ */
/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * The main() function
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
#include "precomp.h"
#ifdef Q_WS_MAC
# include "UICocoaApplication.h"
#endif /* Q_WS_MAC */
#else /* !VBOX_WITH_PRECOMPILED_HEADERS */
#include "VBoxGlobal.h"
#include "UIMessageCenter.h"
#include "UISelectorWindow.h"
#include "VBoxUtils.h"
#ifdef Q_WS_MAC
# include "UICocoaApplication.h"
#endif

#ifdef Q_WS_X11
#include <QFontDatabase>
#include <iprt/env.h>
#endif

#include <QCleanlooksStyle>
#include <QPlastiqueStyle>
#include <QMessageBox>
#include <QLocale>
#include <QTranslator>

#ifdef Q_WS_X11
# include <X11/Xlib.h>
#endif

#include <iprt/buildconfig.h>
#include <iprt/err.h>
#include <iprt/initterm.h>
#include <iprt/process.h>
#include <iprt/stream.h>
#include <VBox/err.h>
#include <VBox/version.h>
#endif /* !VBOX_WITH_PRECOMPILED_HEADERS */

#ifdef VBOX_WITH_HARDENING
# include <VBox/sup.h>
#endif

#ifdef RT_OS_LINUX
# include <unistd.h>
#endif

#include <cstdio>

/* XXX Temporarily. Don't rely on the user to hack the Makefile himself! */
QString g_QStrHintLinuxNoMemory = QApplication::tr(
  "This error means that the kernel driver was either not able to "
  "allocate enough memory or that some mapping operation failed."
  );

QString g_QStrHintLinuxNoDriver = QApplication::tr(
  "The VirtualBox Linux kernel driver (vboxdrv) is either not loaded or "
  "there is a permission problem with /dev/vboxdrv. Please reinstall the kernel "
  "module by executing<br/><br/>"
  "  <font color=blue>'/etc/init.d/vboxdrv setup'</font><br/><br/>"
  "as root. If it is available in your distribution, you should install the "
  "DKMS package first. This package keeps track of Linux kernel changes and "
  "recompiles the vboxdrv kernel module if necessary."
  );

QString g_QStrHintOtherWrongDriverVersion = QApplication::tr(
  "The VirtualBox kernel modules do not match this version of "
  "VirtualBox. The installation of VirtualBox was apparently not "
  "successful. Please try completely uninstalling and reinstalling "
  "VirtualBox."
  );

QString g_QStrHintLinuxWrongDriverVersion = QApplication::tr(
  "The VirtualBox kernel modules do not match this version of "
  "VirtualBox. The installation of VirtualBox was apparently not "
  "successful. Executing<br/><br/>"
  "  <font color=blue>'/etc/init.d/vboxdrv setup'</font><br/><br/>"
  "may correct this. Make sure that you do not mix the "
  "OSE version and the PUEL version of VirtualBox."
  );

QString g_QStrHintOtherNoDriver = QApplication::tr(
  "Make sure the kernel module has been loaded successfully."
  );

/* I hope this isn't (C), (TM) or (R) Microsoft support ;-) */
QString g_QStrHintReinstall = QApplication::tr(
  "Please try reinstalling VirtualBox."
  );

#if defined(DEBUG) && defined(Q_WS_X11) && defined(RT_OS_LINUX)

#include <signal.h>
#include <execinfo.h>

/* get REG_EIP from ucontext.h */
#ifndef __USE_GNU
#define __USE_GNU
#endif
#include <ucontext.h>
#ifdef RT_ARCH_AMD64
# define REG_PC REG_RIP
#else
# define REG_PC REG_EIP
#endif



/**
 * the signal handler that prints out a backtrace of the call stack.
 * the code is taken from http://www.linuxjournal.com/article/6391.
 */
void bt_sighandler (int sig, siginfo_t *info, void *secret) {

    void *trace[16];
    char **messages = (char **)NULL;
    int i, trace_size = 0;
    ucontext_t *uc = (ucontext_t *)secret;

    /* Do something useful with siginfo_t */
    if (sig == SIGSEGV)
        Log (("GUI: Got signal %d, faulty address is %p, from %p\n",
              sig, info->si_addr, uc->uc_mcontext.gregs[REG_PC]));
    else
        Log (("GUI: Got signal %d\n", sig));

    trace_size = backtrace (trace, 16);
    /* overwrite sigaction with caller's address */
    trace[1] = (void *) uc->uc_mcontext.gregs [REG_PC];

    messages = backtrace_symbols (trace, trace_size);
    /* skip first stack frame (points here) */
    Log (("GUI: [bt] Execution path:\n"));
    for (i = 1; i < trace_size; ++i)
        Log (("GUI: [bt] %s\n", messages[i]));

    exit (0);
}

#endif /* DEBUG && X11 && LINUX*/

#if defined(RT_OS_DARWIN)
# include <dlfcn.h>
# include <sys/mman.h>
# include <iprt/asm.h>
# include <iprt/system.h>

/** Really ugly hack to shut up a silly check in AppKit. */
static void ShutUpAppKit(void)
{
    /* Check for Snow Leopard or higher */
    char szInfo[64];
    int rc = RTSystemQueryOSInfo (RTSYSOSINFO_RELEASE, szInfo, sizeof(szInfo));
    if (   RT_SUCCESS (rc)
        && szInfo[0] == '1') /* higher than 1x.x.x */
    {
        /*
         * Find issetguid() and make it always return 0 by modifying the code.
         */
        void *addr = dlsym(RTLD_DEFAULT, "issetugid");
        int rc = mprotect((void *)((uintptr_t)addr & ~(uintptr_t)0xfff), 0x2000, PROT_WRITE|PROT_READ|PROT_EXEC);
        if (!rc)
            ASMAtomicWriteU32((volatile uint32_t *)addr, 0xccc3c031); /* xor eax, eax; ret; int3 */
    }
}
#endif /* DARWIN */

static void QtMessageOutput (QtMsgType type, const char *msg)
{
#ifndef Q_WS_X11
    NOREF(msg);
#endif
    switch (type)
    {
        case QtDebugMsg:
            Log (("Qt DEBUG: %s\n", msg));
            break;
        case QtWarningMsg:
            Log (("Qt WARNING: %s\n", msg));
#ifdef Q_WS_X11
            /* Needed for instance for the message ``cannot connect to X server'' */
            RTStrmPrintf(g_pStdErr, "Qt WARNING: %s\n", msg);
#endif
            break;
        case QtCriticalMsg:
            Log (("Qt CRITICAL: %s\n", msg));
#ifdef Q_WS_X11
            /* Needed for instance for the message ``cannot connect to X server'' */
            RTStrmPrintf(g_pStdErr, "Qt CRITICAL: %s\n", msg);
#endif
            break;
        case QtFatalMsg:
            Log (("Qt FATAL: %s\n", msg));
#ifdef Q_WS_X11
            RTStrmPrintf(g_pStdErr, "Qt FATAL: %s\n", msg);
#endif
    }
}

/**
 * Show all available command line parameters.
 */
static void showHelp()
{
    QString mode = "", dflt = "";
#ifdef VBOX_GUI_USE_SDL
    mode += "sdl";
#endif
#ifdef VBOX_GUI_USE_QIMAGE
    if (!mode.isEmpty())
        mode += "|";
    mode += "image";
#endif
#ifdef VBOX_GUI_USE_DDRAW
    if (!mode.isEmpty())
        mode += "|";
    mode += "ddraw";
#endif
#ifdef VBOX_GUI_USE_QUARTZ2D
    if (!mode.isEmpty())
        mode += "|";
    mode += "quartz2d";
#endif
#if defined (Q_WS_MAC) && defined (VBOX_GUI_USE_QUARTZ2D)
    dflt = "quartz2d";
#elif (defined (Q_WS_WIN32) || defined (Q_WS_PM) || defined(Q_WS_X11)) && defined (VBOX_GUI_USE_QIMAGE)
    dflt = "image";
#elif defined (Q_WS_X11) && defined (VBOX_GUI_USE_SDL)
    dflt = "sdl";
#else
    dflt = "image";
#endif

    RTPrintf(VBOX_PRODUCT " Manager %s\n"
            "(C) 2005-" VBOX_C_YEAR " " VBOX_VENDOR "\n"
            "All rights reserved.\n"
            "\n"
            "Usage:\n"
            "  --startvm <vmname|UUID>    start a VM by specifying its UUID or name\n"
            "  --seamless                 switch to seamless mode during startup\n"
            "  --fullscreen               switch to fullscreen mode during startup\n"
            "  --rmode %-18s select different render mode (default is %s)\n"
            "  --no-startvm-errormsgbox   do not show a message box for VM start errors\n"
# ifdef VBOX_GUI_WITH_PIDFILE
            "  --pidfile <file>           create a pidfile file when a VM is up and running\n"
# endif
# ifdef VBOX_WITH_DEBUGGER_GUI
            "  --dbg                      enable the GUI debug menu\n"
            "  --debug                    like --dbg and show debug windows at VM startup\n"
            "  --debug-command-line       like --dbg and show command line window at VM startup\n"
            "  --debug-statistics         like --dbg and show statistics window at VM startup\n"
            "  --no-debug                 disable the GUI debug menu and debug windows\n"
            "  --start-paused             start the VM in the paused state\n"
            "  --start-running            start the VM running (for overriding --debug*)\n"
            "\n"
# endif
            "Expert options:\n"
            "  --disable-patm             disable code patching (ignored by AMD-V/VT-x)\n"
            "  --disable-csam             disable code scanning (ignored by AMD-V/VT-x)\n"
            "  --recompile-supervisor     recompiled execution of supervisor code (*)\n"
            "  --recompile-user           recompiled execution of user code (*)\n"
            "  --recompile-all            recompiled execution of all code, with disabled\n"
            "                             code patching and scanning\n"
            "  --warp-pct <pct>           time warp factor, 100%% (= 1.0) = normal speed\n"
            "  (*) For AMD-V/VT-x setups the effect is --recompile-all.\n"
            "\n"
# ifdef VBOX_WITH_DEBUGGER_GUI
            "The following environment (and extra data) variables are evaluated:\n"
            "  VBOX_GUI_DBG_ENABLED (GUI/Dbg/Enabled)\n"
            "                             enable the GUI debug menu if set\n"
            "  VBOX_GUI_DBG_AUTO_SHOW (GUI/Dbg/AutoShow)\n"
            "                             show debug windows at VM startup\n"
            "  VBOX_GUI_NO_DEBUGGER       disable the GUI debug menu and debug windows\n"
# endif
            "\n",
            RTBldCfgVersion(),
            mode.toLatin1().constData(),
            dflt.toLatin1().constData());
    /** @todo Show this as a dialog on windows. */
}

extern "C" DECLEXPORT(int) TrustedMain (int argc, char **argv, char ** /*envp*/)
{
    LogFlowFuncEnter();
# if defined(RT_OS_DARWIN)
    ShutUpAppKit();
# endif

    for (int i=0; i<argc; i++)
        if (   !strcmp(argv[i], "-h")
            || !strcmp(argv[i], "-?")
            || !strcmp(argv[i], "-help")
            || !strcmp(argv[i], "--help"))
        {
            showHelp();
            return 0;
        }

#if defined(DEBUG) && defined(Q_WS_X11) && defined(RT_OS_LINUX)
    /* install our signal handler to backtrace the call stack */
    struct sigaction sa;
    sa.sa_sigaction = bt_sighandler;
    sigemptyset (&sa.sa_mask);
    sa.sa_flags = SA_RESTART | SA_SIGINFO;
    sigaction (SIGSEGV, &sa, NULL);
    sigaction (SIGBUS, &sa, NULL);
    sigaction (SIGUSR1, &sa, NULL);
#endif

#ifdef QT_MAC_USE_COCOA
    /* Instantiate our NSApplication derivative before QApplication
     * forces NSApplication to be instantiated. */
    UICocoaApplication::instance();
#endif

    qInstallMsgHandler (QtMessageOutput);

    int rc = 1; /* failure */

    /* scope the QApplication variable */
    {
#ifdef Q_WS_X11
        /* Qt has a complex algorithm for selecting the right visual which
         * doesn't always seem to work.  So we naively choose a visual - the
         * default one - ourselves and pass that to Qt.  This means that we
         * also have to open the display ourselves.
         * We check the Qt parameter list and handle Qt's -display argument
         * ourselves, since we open the display connection.  We also check the
         * to see if the user has passed Qt's -visual parameter, and if so we
         * assume that the user wants Qt to handle visual selection after all,
         * and don't supply a visual. */
        char *pszDisplay = NULL;
        bool useDefaultVisual = true;
        for (int i = 0; i < argc; ++i)
        {
            if (!::strcmp(argv[i], "-display") && (i + 1 < argc))
            /* What if it isn't?  Rely on QApplication to complain? */
            {
                pszDisplay = argv[i + 1];
                ++i;
            }
            else if (!::strcmp(argv[i], "-visual"))
                useDefaultVisual = false;
        }
        Display *pDisplay = XOpenDisplay(pszDisplay);
        if (!pDisplay)
        {
            RTPrintf(pszDisplay ? "Failed to open the X11 display \"%s\"!\n"
                                : "Failed to open the X11 display!\n",
                     pszDisplay);
            return 0;
        }
        Visual *pVisual =   useDefaultVisual
                          ? DefaultVisual(pDisplay, DefaultScreen(pDisplay))
                          : NULL;
        /* Now create the application object */
        QApplication a (pDisplay, argc, argv, (Qt::HANDLE) pVisual);
#else /* Q_WS_X11 */
        QApplication a (argc, argv);
#endif /* Q_WS_X11 */

        /* Qt4.3 version has the QProcess bug which freezing the application
         * for 30 seconds. This bug is internally used at initialization of
         * Cleanlooks style. So we have to change this style to another one.
         * See http://trolltech.com/developer/task-tracker/index_html?id=179200&method=entry
         * for details. */
        if (VBoxGlobal::qtRTVersionString().startsWith ("4.3") &&
            qobject_cast <QCleanlooksStyle*> (QApplication::style()))
            QApplication::setStyle (new QPlastiqueStyle);

#ifdef Q_OS_SOLARIS
        /* Use plastique look 'n feel for Solaris instead of the default motif (Qt 4.7.x) */
        QApplication::setStyle (new QPlastiqueStyle);
#endif

#ifdef Q_WS_X11
        /* This patch is not used for now on Solaris & OpenSolaris because
         * there is no anti-aliasing enabled by default, Qt4 to be rebuilt. */
#ifndef Q_OS_SOLARIS
        /* Cause Qt4 has the conflict with fontconfig application as a result
         * sometimes substituting some fonts with non scaleable-anti-aliased
         * bitmap font we are reseting substitutes for the current application
         * font family if it is non scaleable-anti-aliased. */
        QFontDatabase fontDataBase;

        QString currentFamily (QApplication::font().family());
        bool isCurrentScaleable = fontDataBase.isScalable (currentFamily);

        /*
        LogFlowFunc (("Font: Current family is '%s'. It is %s.\n",
            currentFamily.toLatin1().constData(),
            isCurrentScaleable ? "scalable" : "not scalable"));
        QStringList subFamilies (QFont::substitutes (currentFamily));
        foreach (QString sub, subFamilies)
        {
            bool isSubScalable = fontDataBase.isScalable (sub);
            LogFlowFunc (("Font: Substitute family is '%s'. It is %s.\n",
                sub.toLatin1().constData(),
                isSubScalable ? "scalable" : "not scalable"));
        }
        */

        QString subFamily (QFont::substitute (currentFamily));
        bool isSubScaleable = fontDataBase.isScalable (subFamily);

        if (isCurrentScaleable && !isSubScaleable)
            QFont::removeSubstitution (currentFamily);
#endif /* Q_OS_SOLARIS */
#endif

#ifdef Q_WS_WIN
        /* Drag in the sound drivers and DLLs early to get rid of the delay taking
         * place when the main menu bar (or any action from that menu bar) is
         * activated for the first time. This delay is especially annoying if it
         * happens when the VM is executing in real mode (which gives 100% CPU
         * load and slows down the load process that happens on the main GUI
         * thread to several seconds). */
        PlaySound (NULL, NULL, 0);
#endif

#ifdef Q_WS_MAC
        ::darwinDisableIconsInMenus();
#endif /* Q_WS_MAC */

#ifdef Q_WS_X11
        /* version check (major.minor are sensitive, fix number is ignored) */
        if (VBoxGlobal::qtRTVersion() < (VBoxGlobal::qtCTVersion() & 0xFFFF00))
        {
            QString msg =
                QApplication::tr ("Executable <b>%1</b> requires Qt %2.x, found Qt %3.")
                                  .arg (qAppName())
                                  .arg (VBoxGlobal::qtCTVersionString().section ('.', 0, 1))
                                  .arg (VBoxGlobal::qtRTVersionString());
            QMessageBox::critical (
                0, QApplication::tr ("Incompatible Qt Library Error"),
                msg, QMessageBox::Abort, 0);
            qFatal ("%s", msg.toAscii().constData());
        }
#endif

        /* load a translation based on the current locale */
        VBoxGlobal::loadLanguage();

        do
        {
            if (!vboxGlobal().isValid())
                break;


            if (vboxGlobal().processArgs())
                return 0;

            msgCenter().checkForMountedWrongUSB();

            VBoxGlobalSettings settings = vboxGlobal().settings();
            /* Process known keys */
            bool noSelector = settings.isFeatureActive ("noSelector");

            if (vboxGlobal().isVMConsoleProcess())
            {
#ifdef VBOX_GUI_WITH_SYSTRAY
                if (vboxGlobal().trayIconInstall())
                {
                    /* Nothing to do here yet. */
                }
#endif
                if (vboxGlobal().startMachine (vboxGlobal().managedVMUuid()))
                {
                    vboxGlobal().setMainWindow (vboxGlobal().vmWindow());
                    rc = a.exec();
                }
            }
            else if (noSelector)
            {
                msgCenter().cannotRunInSelectorMode();
            }
            else
            {
#ifdef VBOX_BLEEDING_EDGE
                msgCenter().showBEBWarning();
#else
# ifndef DEBUG
                /* Check for BETA version */
                QString vboxVersion (vboxGlobal().virtualBox().GetVersion());
                if (vboxVersion.contains ("BETA"))
                {
                    /* Allow to prevent this message */
                    QString str = vboxGlobal().virtualBox().
                        GetExtraData(GUI_PreventBetaWarning);
                    if (str != vboxVersion)
                        msgCenter().showBETAWarning();
                }
# endif
#endif

                vboxGlobal().setMainWindow (&vboxGlobal().selectorWnd());
#ifdef VBOX_GUI_WITH_SYSTRAY
                if (vboxGlobal().trayIconInstall())
                {
                    /* Nothing to do here yet. */
                }

                if (false == vboxGlobal().isTrayMenu())
                {
#endif
                    vboxGlobal().selectorWnd().show();
#ifdef VBOX_WITH_REGISTRATION_REQUEST
                    vboxGlobal().showRegistrationDialog (false /* aForce */);
#endif
#ifdef VBOX_GUI_WITH_SYSTRAY
                }

                do
                {
#endif
                    rc = a.exec();
#ifdef VBOX_GUI_WITH_SYSTRAY
                } while (vboxGlobal().isTrayMenu());
#endif
            }
        }
        while (0);
    }

    LogFlowFunc (("rc=%d\n", rc));
    LogFlowFuncLeave();

    return rc;
}

#ifndef VBOX_WITH_HARDENING

int main (int argc, char **argv, char **envp)
{
    /* Initialize VBox Runtime. Initialize the SUPLib as well only if we
     * are really about to start a VM. Don't do this if we are only starting
     * the selector window. */
    bool fInitSUPLib = false;
    for (int i = 1; i < argc; i++)
    {
        /* NOTE: the check here must match the corresponding check for the
         * options to start a VM in hardenedmain.cpp and VBoxGlobal.cpp exactly,
         * otherwise there will be weird error messages. */
        if (   !::strcmp(argv[i], "--startvm")
            || !::strcmp(argv[i], "-startvm"))
        {
            fInitSUPLib = true;
            break;
        }
    }

    int rc = RTR3InitExe(argc, &argv, fInitSUPLib ? RTR3INIT_FLAGS_SUPLIB : 0);
    if (RT_FAILURE(rc))
    {
        QApplication a (argc, &argv[0]);
#ifdef Q_OS_SOLARIS
        /* Use plastique look 'n feel for Solaris instead of the default motif (Qt 4.7.x) */
        QApplication::setStyle (new QPlastiqueStyle);
#endif
        QString msgTitle = QApplication::tr ("VirtualBox - Runtime Error");
        QString msgText = "<html>";

        switch (rc)
        {
            case VERR_VM_DRIVER_NOT_INSTALLED:
            case VERR_VM_DRIVER_LOAD_ERROR:
                msgText += QApplication::tr (
                        "<b>Cannot access the kernel driver!</b><br/><br/>");
# ifdef RT_OS_LINUX
                msgText += g_QStrHintLinuxNoDriver;
# else
                msgText += g_QStrHintOtherNoDriver;
# endif
                break;
# ifdef RT_OS_LINUX
            case VERR_NO_MEMORY:
                msgText += g_QStrHintLinuxNoMemory;
                break;
# endif
            case VERR_VM_DRIVER_NOT_ACCESSIBLE:
                msgText += QApplication::tr ("Kernel driver not accessible");
                break;
            case VERR_VM_DRIVER_VERSION_MISMATCH:
# ifdef RT_OS_LINUX
                msgText += g_QStrHintLinuxWrongDriverVersion;
# else
                msgText += g_QStrHintOtherWrongDriverVersion;
# endif
                break;
            default:
                msgText += QApplication::tr (
                        "Unknown error %2 during initialization of the Runtime"
                        ).arg (rc);
                break;
        }
        msgText += "</html>";
        QMessageBox::critical (
                               0,                      /* parent */
                               msgTitle,
                               msgText,
                               QMessageBox::Abort,     /* button0 */
                               0);                     /* button1 */
        return 1;
    }

    return TrustedMain (argc, argv, envp);
}

#else  /* VBOX_WITH_HARDENING */

/**
 * Hardened main failed, report the error without any unnecessary fuzz.
 *
 * @remarks Do not call IPRT here unless really required, it might not be
 *          initialized.
 */
extern "C" DECLEXPORT(void) TrustedError (const char *pszWhere, SUPINITOP enmWhat, int rc, const char *pszMsgFmt, va_list va)
{
# if defined(RT_OS_DARWIN)
    ShutUpAppKit();
# endif

    /*
     * Init the Qt application object. This is a bit hackish as we
     * don't have the argument vector handy.
     */
    int argc = 0;
    char *argv[2] = { NULL, NULL };
    QApplication a (argc, &argv[0]);

    /*
     * Compose and show the error message.
     */
    QString msgTitle = QApplication::tr ("VirtualBox - Error In %1").arg (pszWhere);

    char msgBuf[1024];
    vsprintf (msgBuf, pszMsgFmt, va);

    QString msgText = QApplication::tr (
            "<html><b>%1 (rc=%2)</b><br/><br/>").arg (msgBuf).arg (rc);
    switch (enmWhat)
    {
        case kSupInitOp_Driver:
# ifdef RT_OS_LINUX
            msgText += g_QStrHintLinuxNoDriver;
# else
            msgText += g_QStrHintOtherNoDriver;
# endif
            break;
# ifdef RT_OS_LINUX
        case kSupInitOp_IPRT:
            if (rc == VERR_NO_MEMORY)
                msgText += g_QStrHintLinuxNoMemory;
            else
# endif
            if (rc == VERR_VM_DRIVER_VERSION_MISMATCH)
# ifdef RT_OS_LINUX
                msgText += g_QStrHintLinuxWrongDriverVersion;
# else
                msgText += g_QStrHintOtherWrongDriverVersion;
# endif
            else
                msgText += g_QStrHintReinstall;
            break;
        case kSupInitOp_Integrity:
        case kSupInitOp_RootCheck:
            msgText += g_QStrHintReinstall;
            break;
        default:
            /* no hints here */
            break;
    }
    msgText += "</html>";

# ifdef RT_OS_LINUX
    sleep(2);
# endif
    QMessageBox::critical (
        0,                      /* parent */
        msgTitle,               /* title */
        msgText,                /* text */
        QMessageBox::Abort,     /* button0 */
        0);                     /* button1 */
    qFatal ("%s", msgText.toAscii().constData());
}

#endif /* VBOX_WITH_HARDENING */

