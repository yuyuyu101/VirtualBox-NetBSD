/** $Id: VBoxGuestR3LibDaemonize.cpp $ */
/** @file
 * VBoxGuestR3Lib - Ring-3 Support Library for VirtualBox guest additions, daemonize a process.
 */

/*
 * Copyright (C) 2007-2010 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 *
 * The contents of this file may alternatively be used under the terms
 * of the Common Development and Distribution License Version 1.0
 * (CDDL) only, as it comes in the "COPYING.CDDL" file of the
 * VirtualBox OSE distribution, in which case the provisions of the
 * CDDL are applicable instead of those of the GPL.
 *
 * You may elect to license modified versions of this file under the
 * terms and conditions of either the GPL or the CDDL or both.
 */


/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#if defined(RT_OS_DARWIN)
# error "PORTME"

#elif defined(RT_OS_OS2)
# define INCL_BASE
# define INCL_ERRORS
# include <os2.h>

# include <iprt/alloca.h>
# include <iprt/string.h>

#elif defined(RT_OS_WINDOWS)
# error "PORTME"

#else /* the unices */
# include <sys/types.h>
# include <sys/stat.h>
# include <stdio.h>
# include <fcntl.h>
# include <stdlib.h>
# include <unistd.h>
# include <signal.h>
# include <errno.h>
#endif

#include <iprt/file.h>
#include <iprt/process.h>
#include <iprt/string.h>
#include "VBGLR3Internal.h"


/**
 * Daemonize the process for running in the background.
 *
 * This is supposed to do the same job as the BSD daemon() call.
 *
 * @returns 0 on success
 *
 * @param   fNoChDir    Pass false to change working directory to root.
 * @param   fNoClose    Pass false to redirect standard file streams to /dev/null.
 *
 * @todo    Use RTProcDaemonize instead of this.
 */
VBGLR3DECL(int) VbglR3Daemonize(bool fNoChDir, bool fNoClose)
{
#if defined(RT_OS_DARWIN)
# error "PORTME"

#elif defined(RT_OS_OS2)
    PPIB pPib;
    PTIB pTib;
    DosGetInfoBlocks(&pTib, &pPib);

    /* Get the full path to the executable. */
    char szExe[CCHMAXPATH];
    APIRET rc = DosQueryModuleName(pPib->pib_hmte, sizeof(szExe), szExe);
    if (rc)
        return RTErrConvertFromOS2(rc);

    /* calc the length of the command line. */
    char *pch = pPib->pib_pchcmd;
    size_t cch0 = strlen(pch);
    pch += cch0 + 1;
    size_t cch1 = strlen(pch);
    pch += cch1 + 1;
    char *pchArgs;
    if (cch1 && *pch)
    {
        do  pch = strchr(pch, '\0') + 1;
        while (*pch);

        size_t cchTotal = pch - pPib->pib_pchcmd;
        pchArgs = (char *)alloca(cchTotal + sizeof("--daemonized\0\0"));
        memcpy(pchArgs, pPib->pib_pchcmd, cchTotal - 1);
        memcpy(pchArgs + cchTotal - 1, "--daemonized\0\0", sizeof("--daemonized\0\0"));
    }
    else
    {
        size_t cchTotal = pch - pPib->pib_pchcmd + 1;
        pchArgs = (char *)alloca(cchTotal + sizeof(" --daemonized "));
        memcpy(pchArgs, pPib->pib_pchcmd, cch0 + 1);
        pch = pchArgs + cch0 + 1;
        memcpy(pch, " --daemonized ", sizeof(" --daemonized ") - 1);
        pch += sizeof(" --daemonized ") - 1;
        if (cch1)
            memcpy(pch, pPib->pib_pchcmd + cch0 + 1, cch1 + 2);
        else
            pch[0] = pch[1] = '\0';
    }

    /* spawn a detach process  */
    char szObj[128];
    RESULTCODES ResCodes = { 0, 0 };
    szObj[0] = '\0';
    rc = DosExecPgm(szObj, sizeof(szObj), EXEC_BACKGROUND, (PCSZ)pchArgs, NULL, &ResCodes, (PCSZ)szExe);
    if (rc)
    {
        /** @todo Change this to some standard log/print error?? */
        /* VBoxServiceError("DosExecPgm failed with rc=%d and szObj='%s'\n", rc, szObj); */
        return RTErrConvertFromOS2(rc);
    }
    DosExit(EXIT_PROCESS, 0);
    return VERR_GENERAL_FAILURE;

#elif defined(RT_OS_WINDOWS)
# error "PORTME"

#else /* the unices */
    /*
     * Fork the child process in a new session and quit the parent.
     *
     * - fork once and create a new session (setsid). This will detach us
     *   from the controlling tty meaning that we won't receive the SIGHUP
     *   (or any other signal) sent to that session.
     * - The SIGHUP signal is ignored because the session/parent may throw
     *   us one before we get to the setsid.
     * - When the parent exit(0) we will become an orphan and re-parented to
     *   the init process.
     * - Because of the Linux / System V semantics of assigning the controlling
     *   tty automagically when a session leader first opens a tty, we will
     *   fork() once more on Linux to get rid of the session leadership role.
     */

    struct sigaction OldSigAct;
    struct sigaction SigAct;
    RT_ZERO(SigAct);
    SigAct.sa_handler = SIG_IGN;
    int rcSigAct = sigaction(SIGHUP, &SigAct, &OldSigAct);

    pid_t pid = fork();
    if (pid == -1)
        return RTErrConvertFromErrno(errno);
    if (pid != 0)
        exit(0);

    /*
     * The orphaned child becomes is reparented to the init process.
     * We create a new session for it (setsid), point the standard
     * file descriptors to /dev/null, and change to the root directory.
     */
    pid_t newpgid = setsid();
    int SavedErrno = errno;
    if (rcSigAct != -1)
        sigaction(SIGHUP, &OldSigAct, NULL);
    if (newpgid == -1)
        return RTErrConvertFromErrno(SavedErrno);

    if (!fNoClose)
    {
        /* Open stdin(0), stdout(1) and stderr(2) as /dev/null. */
        int fd = open("/dev/null", O_RDWR);
        if (fd == -1) /* paranoia */
        {
            close(STDIN_FILENO);
            close(STDOUT_FILENO);
            close(STDERR_FILENO);
            fd = open("/dev/null", O_RDWR);
        }
        if (fd != -1)
        {
            dup2(fd, STDIN_FILENO);
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            if (fd > 2)
                close(fd);
        }
    }

    if (!fNoChDir)
        chdir("/");

    /*
     * Change the umask - this is non-standard daemon() behavior.
     */
    umask(027);

# ifdef RT_OS_LINUX
    /*
     * And fork again to lose session leader status (non-standard daemon()
     * behaviour).
     */
    pid = fork();
    if (pid == -1)
        return RTErrConvertFromErrno(errno);
    if (pid != 0)
        exit(0);
# endif /* RT_OS_LINUX */

    return VINF_SUCCESS;
#endif
}


/**
 * Creates a PID File and returns the open file descriptor.
 *
 * On DOS based system, file sharing (deny write) is used for locking the PID
 * file.
 *
 * On Unix-y systems, an exclusive advisory lock is used for locking the PID
 * file since the file sharing support is usually missing there.
 *
 * This API will overwrite any existing PID Files without a lock on them, on the
 * assumption that they are stale files which an old process did not properly
 * clean up.
 *
 * @returns IPRT status code.
 * @param   pszPath  The path and filename to create the PID File under
 * @param   phFile   Where to store the file descriptor of the open (and locked
 *                   on Unix-y systems) PID File. On failure, or if another
 *                   process owns the PID File, this will be set to NIL_RTFILE.
 */
VBGLR3DECL(int) VbglR3PidFile(const char *pszPath, PRTFILE phFile)
{
    AssertPtrReturn(pszPath, VERR_INVALID_PARAMETER);
    AssertPtrReturn(phFile, VERR_INVALID_PARAMETER);
    *phFile = NIL_RTFILE;

    RTFILE hPidFile;
    int rc = RTFileOpen(&hPidFile, pszPath,
                        RTFILE_O_READWRITE | RTFILE_O_OPEN_CREATE | RTFILE_O_DENY_WRITE
                        | (0644 << RTFILE_O_CREATE_MODE_SHIFT));
    if (RT_SUCCESS(rc))
    {
#if !defined(RT_OS_WINDOWS) && !defined(RT_OS_OS2)
        /** @todo using size 0 for locking means lock all on Posix.
         * We should adopt this as our convention too, or something
         * similar. */
        rc = RTFileLock(hPidFile, RTFILE_LOCK_WRITE, 0, 0);
        if (RT_FAILURE(rc))
            RTFileClose(hPidFile);
        else
#endif
        {
            char szBuf[256];
            size_t cbPid = RTStrPrintf(szBuf, sizeof(szBuf), "%d\n",
                                       RTProcSelf());
            RTFileWrite(hPidFile, szBuf, cbPid, NULL);
            *phFile = hPidFile;
        }
    }
    return rc;
}


/**
 * Close and remove an open PID File.
 *
 * @param  pszPath  The path to the PID File,
 * @param  hFile    The handle for the file. NIL_RTFILE is ignored as usual.
 */
VBGLR3DECL(void) VbglR3ClosePidFile(const char *pszPath, RTFILE hFile)
{
    AssertPtrReturnVoid(pszPath);
    if (hFile != NIL_RTFILE)
    {
#if defined(RT_OS_WINDOWS) || defined(RT_OS_OS2)
        RTFileWriteAt(hFile, 0, "-1", 2, NULL);
#else
        RTFileDelete(pszPath);
#endif
        RTFileClose(hFile);
    }
}

