/* $Id: VBoxServiceControl.cpp $ */
/** @file
 * VBoxServiceControl - Host-driven Guest Control.
 */

/*
 * Copyright (C) 2012-2013 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */


/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#include <iprt/asm.h>
#include <iprt/assert.h>
#include <iprt/file.h>
#include <iprt/getopt.h>
#include <iprt/mem.h>
#include <iprt/path.h>
#include <iprt/semaphore.h>
#include <iprt/thread.h>
#include <VBox/VBoxGuestLib.h>
#include <VBox/HostServices/GuestControlSvc.h>
#include "VBoxServiceInternal.h"
#include "VBoxServiceUtils.h"

using namespace guestControl;

/*******************************************************************************
*   Global Variables                                                           *
*******************************************************************************/
/** The control interval (milliseconds). */
static uint32_t             g_uControlIntervalMS = 0;
/** The semaphore we're blocking our main control thread on. */
static RTSEMEVENTMULTI      g_hControlEvent = NIL_RTSEMEVENTMULTI;
/** The VM session ID. Changes whenever the VM is restored or reset. */
static uint64_t             g_idControlSession;
/** The guest control service client ID. */
static uint32_t             g_uControlSvcClientID = 0;
/** How many started guest processes are kept into memory for supplying
 *  information to the host. Default is 256 processes. If 0 is specified,
 *  the maximum number of processes is unlimited. */
static uint32_t             g_uControlProcsMaxKept = 256;
#ifdef DEBUG
 static bool                g_fControlDumpStdErr   = false;
 static bool                g_fControlDumpStdOut   = false;
#endif
/** List of active guest control threads (VBOXSERVICECTRLTHREAD). */
static RTLISTANCHOR         g_lstControlThreadsActive;
/** List of inactive guest control threads (VBOXSERVICECTRLTHREAD). */
static RTLISTANCHOR         g_lstControlThreadsInactive;
/** Critical section protecting g_GuestControlExecThreads. */
static RTCRITSECT           g_csControlThreads;
/** List of guest control files (VBOXSERVICECTRLFILE).
 **@todo Use a map (later). */
static RTLISTANCHOR         g_lstControlFiles;
/** The internal file count for building our internal file handles.
 *  Should be enough for now. */
static uint32_t             g_uControlFileCount = 0;


/*******************************************************************************
*   Internal Functions                                                         *
*******************************************************************************/
/** @todo Shorten "VBoxServiceControl" to "gstsvcCntl". */
static int  VBoxServiceControlHandleCmdStartProc(uint32_t u32ClientId, uint32_t uNumParms);
static int  VBoxServiceControlHandleCmdSetInput(uint32_t u32ClientId, uint32_t uNumParms, void *pvScratchBuf, size_t cbScratchBuf);
static int  VBoxServiceControlHandleCmdGetOutput(uint32_t u32ClientId, uint32_t uNumParms);
static int  VBoxServiceControlHandleFileOpen(uint32_t idClient, uint32_t cParms);
static int  VBoxServiceControlHandleFileClose(uint32_t idClient, uint32_t cParms);
static int  VBoxServiceControlHandleFileRead(uint32_t idClient, uint32_t cParms);
static int  VBoxServiceControlHandleFileWrite(uint32_t idClient, uint32_t cParms, void *pvScratchBuf, size_t cbScratchBuf);
static int  VBoxServiceControlHandleFileSeek(uint32_t idClient, uint32_t cParms);
static int  VBoxServiceControlHandleFileTell(uint32_t idClient, uint32_t cParms);
static int  VBoxServiceControlReapThreads(void);
static void VBoxServiceControlShutdown(void);
static int  vboxServiceControlSessionCloseAll(void);
static int  VBoxServiceControlStartAllowed(bool *pbAllowed);

#ifdef DEBUG
static int vboxServiceControlDump(const char *pszFileName, void *pvBuf, size_t cbBuf)
{
    AssertPtrReturn(pszFileName, VERR_INVALID_POINTER);
    AssertPtrReturn(pvBuf, VERR_INVALID_POINTER);

    if (!cbBuf)
        return VINF_SUCCESS;

    char szFile[RTPATH_MAX];

    int rc = RTPathTemp(szFile, sizeof(szFile));
    if (RT_SUCCESS(rc))
        rc = RTPathAppend(szFile, sizeof(szFile), pszFileName);

    if (RT_SUCCESS(rc))
    {
        VBoxServiceVerbose(4, "Dumping %ld bytes to \"%s\"\n", cbBuf, szFile);

        RTFILE fh;
        rc = RTFileOpen(&fh, szFile, RTFILE_O_OPEN_CREATE | RTFILE_O_WRITE | RTFILE_O_DENY_WRITE);
        if (RT_SUCCESS(rc))
        {
            rc = RTFileWrite(fh, pvBuf, cbBuf, NULL /* pcbWritten */);
            RTFileClose(fh);
        }
    }

    return rc;
}
#endif


/** @copydoc VBOXSERVICE::pfnPreInit */
static DECLCALLBACK(int) VBoxServiceControlPreInit(void)
{
#ifdef VBOX_WITH_GUEST_PROPS
    /*
     * Read the service options from the VM's guest properties.
     * Note that these options can be overridden by the command line options later.
     */
    uint32_t uGuestPropSvcClientID;
    int rc = VbglR3GuestPropConnect(&uGuestPropSvcClientID);
    if (RT_FAILURE(rc))
    {
        if (rc == VERR_HGCM_SERVICE_NOT_FOUND) /* Host service is not available. */
        {
            VBoxServiceVerbose(0, "Guest property service is not available, skipping\n");
            rc = VINF_SUCCESS;
        }
        else
            VBoxServiceError("Failed to connect to the guest property service! Error: %Rrc\n", rc);
    }
    else
    {
        rc = VBoxServiceReadPropUInt32(uGuestPropSvcClientID, "/VirtualBox/GuestAdd/VBoxService/--control-procs-max-kept",
                                       &g_uControlProcsMaxKept, 0, UINT32_MAX - 1);

        VbglR3GuestPropDisconnect(uGuestPropSvcClientID);
    }

    if (rc == VERR_NOT_FOUND) /* If a value is not found, don't be sad! */
        rc = VINF_SUCCESS;
    return rc;
#else
    /* Nothing to do here yet. */
    return VINF_SUCCESS;
#endif
}


/** @copydoc VBOXSERVICE::pfnOption */
static DECLCALLBACK(int) VBoxServiceControlOption(const char **ppszShort, int argc, char **argv, int *pi)
{
    int rc = -1;
    if (ppszShort)
        /* no short options */;
    else if (!strcmp(argv[*pi], "--control-interval"))
        rc = VBoxServiceArgUInt32(argc, argv, "", pi,
                                  &g_uControlIntervalMS, 1, UINT32_MAX - 1);
    else if (!strcmp(argv[*pi], "--control-procs-max-kept"))
        rc = VBoxServiceArgUInt32(argc, argv, "", pi,
                                  &g_uControlProcsMaxKept, 0, UINT32_MAX - 1);
#ifdef DEBUG
    else if (!strcmp(argv[*pi], "--control-dump-stderr"))
    {
        g_fControlDumpStdErr = true;
        rc = 0; /* Flag this command as parsed. */
    }
    else if (!strcmp(argv[*pi], "--control-dump-stdout"))
    {
        g_fControlDumpStdOut = true;
        rc = 0; /* Flag this command as parsed. */
    }
#endif
    return rc;
}


/** @copydoc VBOXSERVICE::pfnInit */
static DECLCALLBACK(int) VBoxServiceControlInit(void)
{
    /*
     * If not specified, find the right interval default.
     * Then create the event sem to block on.
     */
    if (!g_uControlIntervalMS)
        g_uControlIntervalMS = 1000;

    int rc = RTSemEventMultiCreate(&g_hControlEvent);
    AssertRCReturn(rc, rc);

    VbglR3GetSessionId(&g_idControlSession);
    /* The status code is ignored as this information is not available with VBox < 3.2.10. */

    rc = VbglR3GuestCtrlConnect(&g_uControlSvcClientID);
    if (RT_SUCCESS(rc))
    {
        VBoxServiceVerbose(3, "Service client ID: %#x\n", g_uControlSvcClientID);

        /* Init lists. */
        RTListInit(&g_lstControlThreadsActive);
        RTListInit(&g_lstControlThreadsInactive);
        RTListInit(&g_lstControlFiles);

        /* Init critical section for protecting the thread lists. */
        rc = RTCritSectInit(&g_csControlThreads);
        AssertRC(rc);
    }
    else
    {
        /* If the service was not found, we disable this service without
           causing VBoxService to fail. */
        if (rc == VERR_HGCM_SERVICE_NOT_FOUND) /* Host service is not available. */
        {
            VBoxServiceVerbose(0, "Guest control service is not available\n");
            rc = VERR_SERVICE_DISABLED;
        }
        else
            VBoxServiceError("Failed to connect to the guest control service! Error: %Rrc\n", rc);
        RTSemEventMultiDestroy(g_hControlEvent);
        g_hControlEvent = NIL_RTSEMEVENTMULTI;
    }
    return rc;
}


/** @copydoc VBOXSERVICE::pfnWorker */
DECLCALLBACK(int) VBoxServiceControlWorker(bool volatile *pfShutdown)
{
    /*
     * Tell the control thread that it can continue
     * spawning services.
     */
    RTThreadUserSignal(RTThreadSelf());
    Assert(g_uControlSvcClientID > 0);

    int rc = VINF_SUCCESS;

    /* Allocate a scratch buffer for commands which also send
     * payload data with them. */
    uint32_t cbScratchBuf = _64K; /** @todo Make buffer size configurable via guest properties/argv! */
    AssertReturn(RT_IS_POWER_OF_TWO(cbScratchBuf), VERR_INVALID_PARAMETER);
    uint8_t *pvScratchBuf = (uint8_t*)RTMemAlloc(cbScratchBuf);
    AssertPtrReturn(pvScratchBuf, VERR_NO_MEMORY);

    /*
     * Execution loop.
     *
     * @todo
     */
    for (;;)
    {
        VBoxServiceVerbose(3, "Waiting for host msg ...\n");
        uint32_t uMsg = 0;
        uint32_t cParms = 0;
        rc = VbglR3GuestCtrlWaitForHostMsg(g_uControlSvcClientID, &uMsg, &cParms);
        if (rc == VERR_TOO_MUCH_DATA)
        {
            VBoxServiceVerbose(4, "Message requires %ld parameters, but only 2 supplied -- retrying request (no error!)...\n", cParms);
            rc = VINF_SUCCESS; /* Try to get "real" message in next block below. */
        }
        else if (RT_FAILURE(rc))
            VBoxServiceVerbose(3, "Getting host message failed with %Rrc\n", rc); /* VERR_GEN_IO_FAILURE seems to be normal if ran into timeout. */
        if (RT_SUCCESS(rc))
        {
            VBoxServiceVerbose(3, "Msg=%u (%u parms) retrieved\n", uMsg, cParms);

            uint64_t idNewSession = g_idControlSession;
            int rc2 = VbglR3GetSessionId(&idNewSession);
            if (   RT_SUCCESS(rc2)
                && (idNewSession != g_idControlSession))
            {
                VBoxServiceVerbose(1, "The VM session ID changed\n");
                g_idControlSession = idNewSession;

                /* Close all opened guest sessions -- all context IDs, sessions etc.
                 * are now invalid. */
                rc2 = vboxServiceControlSessionCloseAll();
                AssertRC(rc2);
            }

            switch (uMsg)
            {
                case HOST_CANCEL_PENDING_WAITS:
                    VBoxServiceVerbose(1, "We were asked to quit ...\n");
                    break;

                case HOST_EXEC_CMD:
                    rc = VBoxServiceControlHandleCmdStartProc(g_uControlSvcClientID, cParms);
                    break;

                case HOST_EXEC_SET_INPUT:
                    rc = VBoxServiceControlHandleCmdSetInput(g_uControlSvcClientID, cParms,
                                                             pvScratchBuf, cbScratchBuf);
                    break;

                case HOST_EXEC_GET_OUTPUT:
                    rc = VBoxServiceControlHandleCmdGetOutput(g_uControlSvcClientID, cParms);
                    break;

               case HOST_FILE_OPEN:
                    rc = VBoxServiceControlHandleFileOpen(g_uControlSvcClientID, cParms);
                    break;

                case HOST_FILE_CLOSE:
                    rc = VBoxServiceControlHandleFileClose(g_uControlSvcClientID, cParms);
                    break;

                case HOST_FILE_READ:
                    rc = VBoxServiceControlHandleFileRead(g_uControlSvcClientID, cParms);
                    break;

                case HOST_FILE_WRITE:
                    rc = VBoxServiceControlHandleFileWrite(g_uControlSvcClientID, cParms,
                                                           pvScratchBuf, cbScratchBuf);
                    break;

                case HOST_FILE_SEEK:
                    rc = VBoxServiceControlHandleFileSeek(g_uControlSvcClientID, cParms);
                    break;

                case HOST_FILE_TELL:
                    rc = VBoxServiceControlHandleFileTell(g_uControlSvcClientID, cParms);
                    break;

                default:
                    VBoxServiceVerbose(3, "Unsupported message from host! Msg=%u\n", uMsg);
                    /* Don't terminate here; just wait for the next message. */
                    break;
            }
        }

        /* Do we need to shutdown? */
        if (   *pfShutdown
            || (RT_SUCCESS(rc) && uMsg == HOST_CANCEL_PENDING_WAITS))
        {
            break;
        }

        /* Let's sleep for a bit and let others run ... */
        RTThreadYield();
    }

    VBoxServiceVerbose(0, "Guest control service stopped\n");

    /* Delete scratch buffer. */
    if (pvScratchBuf)
        RTMemFree(pvScratchBuf);

    return rc;
}


/**
 * Handles starting processes on the guest.
 *
 * @returns IPRT status code.
 * @param   uClientID       The HGCM client session ID.
 * @param   cParms          The number of parameters the host is offering.
 */
static int VBoxServiceControlHandleCmdStartProc(uint32_t uClientID, uint32_t cParms)
{
    uint32_t uContextID = 0;

    int rc;
    bool fStartAllowed = false; /* Flag indicating whether starting a process is allowed or not. */
    if (cParms == 11)
    {
        VBOXSERVICECTRLPROCESS proc;
        RT_ZERO(proc);

        /* Initialize maximum environment block size -- needed as input
         * parameter to retrieve the stuff from the host. On output this then
         * will contain the actual block size. */
        proc.cbEnv = sizeof(proc.szEnv);

        rc = VbglR3GuestCtrlExecGetHostCmdExec(uClientID,
                                               cParms,
                                               &uContextID,
                                               /* Command */
                                               proc.szCmd,      sizeof(proc.szCmd),
                                               /* Flags */
                                               &proc.uFlags,
                                               /* Arguments */
                                               proc.szArgs,     sizeof(proc.szArgs), &proc.uNumArgs,
                                               /* Environment */
                                               proc.szEnv, &proc.cbEnv, &proc.uNumEnvVars,
                                               /* Credentials */
                                               proc.szUser,     sizeof(proc.szUser),
                                               proc.szPassword, sizeof(proc.szPassword),
                                               /* Timelimit */
                                               &proc.uTimeLimitMS);
        if (RT_SUCCESS(rc))
        {
            VBoxServiceVerbose(3, "Request to start process szCmd=%s, uFlags=0x%x, szArgs=%s, szEnv=%s, szUser=%s, szPassword=%s, uTimeout=%u\n",
                               proc.szCmd, proc.uFlags,
                               proc.uNumArgs ? proc.szArgs : "<None>",
                               proc.uNumEnvVars ? proc.szEnv : "<None>",
                               proc.szUser,
#ifdef DEBUG
                               proc.szPassword,
#else
                               "XXX", /* Never show passwords in release mode. */
#endif
                               proc.uTimeLimitMS);

            rc = VBoxServiceControlReapThreads();
            if (RT_FAILURE(rc))
                VBoxServiceError("Reaping stopped processes failed with rc=%Rrc\n", rc);
            /* Keep going. */

            rc = VBoxServiceControlStartAllowed(&fStartAllowed);
            if (RT_SUCCESS(rc))
            {
                if (fStartAllowed)
                {
                    rc = VBoxServiceControlThreadStart(uContextID, &proc);
                }
                else
                    rc = VERR_MAX_PROCS_REACHED; /* Maximum number of processes reached. */
            }
        }
    }
    else
        rc = VERR_INVALID_PARAMETER; /* Incorrect number of parameters. */

    /* In case of an error we need to notify the host to not wait forever for our response. */
    if (RT_FAILURE(rc))
    {
        VBoxServiceError("Starting process failed with rc=%Rrc\n", rc);

        /*
         * Note: The context ID can be 0 because we mabye weren't able to fetch the command
         *       from the host. The host in case has to deal with that!
         */
        int rc2 = VbglR3GuestCtrlExecReportStatus(uClientID, uContextID /* Might be 0 */, 0 /* PID, invalid */,
                                                  PROC_STS_ERROR, rc,
                                                  NULL /* pvData */, 0 /* cbData */);
        if (RT_FAILURE(rc2))
        {
            VBoxServiceError("Error sending start process status to host, rc=%Rrc\n", rc2);
            if (RT_SUCCESS(rc))
                rc = rc2;
        }
    }

    return rc;
}


/**
 * Gets output from stdout/stderr of a specified guest process.
 *
 * @return  IPRT status code.
 * @param   uPID                    PID of process to retrieve the output from.
 * @param   uHandleId               Stream ID (stdout = 0, stderr = 2) to get the output from.
 * @param   uTimeout                Timeout (in ms) to wait for output becoming available.
 * @param   pvBuf                   Pointer to a pre-allocated buffer to store the output.
 * @param   cbBuf                   Size (in bytes) of the pre-allocated buffer.
 * @param   pcbRead                 Pointer to number of bytes read.  Optional.
 */
int VBoxServiceControlExecGetOutput(uint32_t uPID, uint32_t uCID,
                                    uint32_t uHandleId, uint32_t uTimeout,
                                    void *pvBuf, uint32_t cbBuf, uint32_t *pcbRead)
{
    AssertPtrReturn(pvBuf, VERR_INVALID_POINTER);
    AssertReturn(cbBuf, VERR_INVALID_PARAMETER);
    /* pcbRead is optional. */

    int rc = VINF_SUCCESS;
    VBOXSERVICECTRLREQUESTTYPE reqType;
    switch (uHandleId)
    {
        case OUTPUT_HANDLE_ID_STDERR:
            reqType = VBOXSERVICECTRLREQUEST_STDERR_READ;
            break;

        case OUTPUT_HANDLE_ID_STDOUT:
        case OUTPUT_HANDLE_ID_STDOUT_DEPRECATED:
            reqType = VBOXSERVICECTRLREQUEST_STDOUT_READ;
            break;

        default:
            rc = VERR_INVALID_PARAMETER;
            break;
    }

    PVBOXSERVICECTRLREQUEST pRequest;
    if (RT_SUCCESS(rc))
    {
        rc = VBoxServiceControlThreadRequestAllocEx(&pRequest, reqType,
                                                    pvBuf, cbBuf, uCID);
        if (RT_SUCCESS(rc))
            rc = VBoxServiceControlThreadPerform(uPID, pRequest);

        if (RT_SUCCESS(rc))
        {
            if (pcbRead)
                *pcbRead = pRequest->cbData;
        }

        VBoxServiceControlThreadRequestFree(pRequest);
    }

    return rc;
}


/**
 * Sets the specified guest thread to a certain list.
 *
 * @return  IPRT status code.
 * @param   enmList                 List to move thread to.
 * @param   pThread                 Thread to set inactive.
 */
int VBoxServiceControlListSet(VBOXSERVICECTRLTHREADLISTTYPE enmList,
                              PVBOXSERVICECTRLTHREAD pThread)
{
    AssertReturn(enmList > VBOXSERVICECTRLTHREADLIST_UNKNOWN, VERR_INVALID_PARAMETER);
    AssertPtrReturn(pThread, VERR_INVALID_POINTER);

    int rc = RTCritSectEnter(&g_csControlThreads);
    if (RT_SUCCESS(rc))
    {
        VBoxServiceVerbose(3, "Setting thread (PID %u) to list %d\n",
                           pThread->uPID, enmList);

        PRTLISTANCHOR pAnchor = NULL;
        switch (enmList)
        {
            case VBOXSERVICECTRLTHREADLIST_STOPPED:
                pAnchor = &g_lstControlThreadsInactive;
                break;

            case VBOXSERVICECTRLTHREADLIST_RUNNING:
                pAnchor = &g_lstControlThreadsActive;
                break;

            default:
                AssertMsgFailed(("Unknown list type: %u", enmList));
                break;
        }

        if (!pAnchor)
            rc = VERR_INVALID_PARAMETER;

        if (RT_SUCCESS(rc))
        {
            if (pThread->pAnchor != NULL)
            {
                /* If thread was assigned to a list before,
                 * remove the thread from the old list first. */
                /* rc = */ RTListNodeRemove(&pThread->Node);
            }

            /* Add thread to desired list. */
            /* rc = */ RTListAppend(pAnchor, &pThread->Node);
            pThread->pAnchor = pAnchor;
        }

        int rc2 = RTCritSectLeave(&g_csControlThreads);
        if (RT_SUCCESS(rc))
            rc = rc2;
    }

    return VINF_SUCCESS;
}


/**
 * Injects input to a specified running process.
 *
 * @return  IPRT status code.
 * @param   uPID                    PID of process to set the input for.
 * @param   fPendingClose           Flag indicating whether this is the last input block sent to the process.
 * @param   pvBuf                   Pointer to a buffer containing the actual input data.
 * @param   cbBuf                   Size (in bytes) of the input buffer data.
 * @param   pcbWritten              Pointer to number of bytes written to the process.  Optional.
 */
int VBoxServiceControlSetInput(uint32_t uPID, uint32_t uCID,
                               bool fPendingClose,
                               void *pvBuf, uint32_t cbBuf,
                               uint32_t *pcbWritten)
{
    /* pvBuf is optional. */
    /* cbBuf is optional. */
    /* pcbWritten is optional. */

    PVBOXSERVICECTRLREQUEST pRequest;
    int rc = VBoxServiceControlThreadRequestAllocEx(&pRequest,
                                                      fPendingClose
                                                    ? VBOXSERVICECTRLREQUEST_STDIN_WRITE_EOF
                                                    : VBOXSERVICECTRLREQUEST_STDIN_WRITE,
                                                    pvBuf, cbBuf, uCID);
    if (RT_SUCCESS(rc))
    {
        rc = VBoxServiceControlThreadPerform(uPID, pRequest);
        if (RT_SUCCESS(rc))
        {
            if (pcbWritten)
                *pcbWritten = pRequest->cbData;
        }

        VBoxServiceControlThreadRequestFree(pRequest);
    }

    return rc;
}


/**
 * Handles input for a started process by copying the received data into its
 * stdin pipe.
 *
 * @returns IPRT status code.
 * @param   idClient                    The HGCM client session ID.
 * @param   cParms                      The number of parameters the host is
 *                                      offering.
 * @param   pvScratchBuf                The scratch buffer.
 * @param   cbScratchBuf                The scratch buffer size for retrieving the input data.
 */
static int VBoxServiceControlHandleCmdSetInput(uint32_t idClient, uint32_t cParms,
                                               void *pvScratchBuf, size_t cbScratchBuf)
{
    AssertPtrReturn(cbScratchBuf, VERR_INVALID_PARAMETER);
    AssertPtrReturn(pvScratchBuf, VERR_INVALID_POINTER);

    uint32_t uContextID;
    uint32_t uPID;
    uint32_t uFlags;
    uint32_t cbSize;

    uint32_t uStatus = INPUT_STS_UNDEFINED; /* Status sent back to the host. */
    uint32_t cbWritten = 0; /* Number of bytes written to the guest. */

    /*
     * Ask the host for the input data.
     */
    int rc = VbglR3GuestCtrlExecGetHostCmdInput(idClient, cParms,
                                                &uContextID, &uPID, &uFlags,
                                                pvScratchBuf, cbScratchBuf, &cbSize);
    if (RT_FAILURE(rc))
    {
        VBoxServiceError("[PID %u]: Failed to retrieve exec input command! Error: %Rrc\n",
                         uPID, rc);
    }
    else if (cbSize > cbScratchBuf)
    {
        VBoxServiceError("[PID %u]: Too much input received! cbSize=%u, cbScratchBuf=%u\n",
                         uPID, cbSize, cbScratchBuf);
        rc = VERR_INVALID_PARAMETER;
    }
    else
    {
        /*
         * Is this the last input block we need to deliver? Then let the pipe know ...
         */
        bool fPendingClose = false;
        if (uFlags & INPUT_FLAG_EOF)
        {
            fPendingClose = true;
            VBoxServiceVerbose(4, "[PID %u]: Got last input block of size %u ...\n",
                               uPID, cbSize);
        }

        rc = VBoxServiceControlSetInput(uPID, uContextID, fPendingClose, pvScratchBuf,
                                        cbSize, &cbWritten);
        VBoxServiceVerbose(4, "[PID %u]: Written input, CID=%u, rc=%Rrc, uFlags=0x%x, fPendingClose=%d, cbSize=%u, cbWritten=%u\n",
                           uPID, uContextID, rc, uFlags, fPendingClose, cbSize, cbWritten);
        if (RT_SUCCESS(rc))
        {
            uStatus = INPUT_STS_WRITTEN;
            uFlags = 0; /* No flags at the moment. */
        }
        else
        {
            if (rc == VERR_BAD_PIPE)
                uStatus = INPUT_STS_TERMINATED;
            else if (rc == VERR_BUFFER_OVERFLOW)
                uStatus = INPUT_STS_OVERFLOW;
        }
    }

    /*
     * If there was an error and we did not set the host status
     * yet, then do it now.
     */
    if (   RT_FAILURE(rc)
        && uStatus == INPUT_STS_UNDEFINED)
    {
        uStatus = INPUT_STS_ERROR;
        uFlags = rc;
    }
    Assert(uStatus > INPUT_STS_UNDEFINED);

    VBoxServiceVerbose(3, "[PID %u]: Input processed, CID=%u, uStatus=%u, uFlags=0x%x, cbWritten=%u\n",
                       uPID, uContextID, uStatus, uFlags, cbWritten);

    /* Note: Since the context ID is unique the request *has* to be completed here,
     *       regardless whether we got data or not! Otherwise the progress object
     *       on the host never will get completed! */
    rc = VbglR3GuestCtrlExecReportStatusIn(idClient, uContextID, uPID,
                                           uStatus, uFlags, (uint32_t)cbWritten);

    if (RT_FAILURE(rc))
        VBoxServiceError("[PID %u]: Failed to report input status! Error: %Rrc\n",
                         uPID, rc);
    return rc;
}


static PVBOXSERVICECTRLFILE VBoxControlGetFile(uint32_t uHandle)
{
    PVBOXSERVICECTRLFILE pFileCur = NULL;
    /** @todo Use a map later! */
    RTListForEach(&g_lstControlFiles, pFileCur, VBOXSERVICECTRLFILE, Node)
    {
        if (pFileCur->uHandle == uHandle)
            return pFileCur;
    }

    return NULL;
}


static int VBoxServiceControlHandleFileOpen(uint32_t idClient, uint32_t cParms)
{
    uint32_t uContextID;

    char szFile[RTPATH_MAX];
    char szOpenMode[64];
    char szDisposition[64];
    uint32_t uCreationMode;
    uint64_t uOffset;

    int rc = VbglR3GuestCtrlFileGetHostCmdOpen(idClient, cParms, &uContextID,
                                               /* File to open. */
                                               szFile, sizeof(szFile),
                                               /* Open mode. */
                                               szOpenMode, sizeof(szOpenMode),
                                               /* Disposition. */
                                               szDisposition, sizeof(szDisposition),
                                               /* Creation mode. */
                                               &uCreationMode,
                                               /* Offset. */
                                               &uOffset);
    if (RT_SUCCESS(rc))
    {
        PVBOXSERVICECTRLFILE pFile = (PVBOXSERVICECTRLFILE)RTMemAlloc(sizeof(VBOXSERVICECTRLFILE));
        if (!pFile)
            return VERR_NO_MEMORY;

        if (!RTStrPrintf(pFile->szName, sizeof(pFile->szName), "%s", szFile))
            rc = VERR_BUFFER_UNDERFLOW;

        if (RT_SUCCESS(rc))
        {
            uint64_t fFlags = RTFILE_O_OPEN_CREATE | RTFILE_O_WRITE | RTFILE_O_DENY_WRITE; /** @todo Modes! */
            rc = RTFileOpen(&pFile->hFile, pFile->szName, fFlags);
            if (   RT_SUCCESS(rc)
                && uOffset)
            {
                /* Seeking is optional. */
                int rc2 = RTFileSeek(pFile->hFile, (int64_t)uOffset, RTFILE_SEEK_BEGIN, NULL /* Current offset */);
                if (RT_FAILURE(rc2))
                    VBoxServiceVerbose(3, "[File %s]: Seeking to offset %RU64 failed; rc=%Rrc\n",
                                       pFile->szName, uOffset, rc);
            }
            else
                VBoxServiceVerbose(3, "[File %s]: Opening failed; rc=%Rrc\n",
                                   pFile->szName, rc);
        }

        uint32_t uHandle = 0;
        if (RT_SUCCESS(rc))
        {
            VBoxServiceVerbose(3, "[File %s]: Opened.\n", pFile->szName);

            uHandle = g_uControlFileCount++;
            pFile->uHandle = uHandle;
            /* rc = */ RTListAppend(&g_lstControlFiles, &pFile->Node);
        }

        if (RT_FAILURE(rc))
            RTMemFree(pFile);

        /* Report back in any case. */
        int rc2 = VbglR3GuestCtrlFileNotify(idClient, uContextID, uHandle,
                                            GUESTFILENOTIFYTYPE_OPEN, &rc, sizeof(rc));
        if (RT_FAILURE(rc2))
            VBoxServiceError("[File %s]: Failed to report open status, rc=%Rrc\n",
                             szFile, rc2);
        if (RT_SUCCESS(rc))
            rc = rc2;
    }
    return rc;
}


static int VBoxServiceControlHandleFileClose(uint32_t idClient, uint32_t cParms)
{
    uint32_t uContextID;
    uint32_t uHandle;

    int rc = VbglR3GuestCtrlFileGetHostCmdClose(idClient, cParms, &uContextID,
                                                /* File handle to close. */
                                                &uHandle);
    if (RT_SUCCESS(rc))
    {
        PVBOXSERVICECTRLFILE pFile = VBoxControlGetFile(uHandle);
        if (pFile)
        {
            rc = RTFileClose(pFile->hFile);
        }
        else
            rc = VERR_NOT_FOUND;

        /* Report back in any case. */
        int rc2 = VbglR3GuestCtrlFileNotify(idClient, uContextID, uHandle,
                                            GUESTFILENOTIFYTYPE_CLOSE, &rc, sizeof(rc));
        if (RT_FAILURE(rc2))
            VBoxServiceError("Failed to report close status, rc=%Rrc\n", rc2);
        if (RT_SUCCESS(rc))
            rc = rc2;
    }
    return rc;
}


static int VBoxServiceControlHandleFileRead(uint32_t idClient, uint32_t cParms)
{
    uint32_t uContextID;
    uint32_t uHandle;
    uint32_t cbToRead;

    int rc = VbglR3GuestCtrlFileGetHostCmdRead(idClient, cParms, &uContextID,
                                               &uHandle, &cbToRead);
    if (RT_SUCCESS(rc))
    {

    }
    return rc;
}


static int VBoxServiceControlHandleFileWrite(uint32_t idClient, uint32_t cParms,
                                             void *pvScratchBuf, size_t cbScratchBuf)
{
    AssertPtrReturn(cbScratchBuf, VERR_INVALID_PARAMETER);
    AssertPtrReturn(pvScratchBuf, VERR_INVALID_POINTER);

    uint32_t uContextID;
    uint32_t uHandle;
    uint32_t cbToWrite;

    int rc = VbglR3GuestCtrlFileGetHostCmdWrite(idClient, cParms, &uContextID,
                                                &uHandle, pvScratchBuf, cbScratchBuf,
                                                &cbToWrite);
    if (RT_SUCCESS(rc))
    {

    }
    return rc;
}


static int VBoxServiceControlHandleFileSeek(uint32_t idClient, uint32_t cParms)
{
    uint32_t uContextID;
    uint32_t uHandle;
    uint32_t uSeekMethod;
    uint64_t uOffset; /* Will be converted to int64_t. */

    int rc = VbglR3GuestCtrlFileGetHostCmdSeek(idClient, cParms, &uContextID,
                                               &uHandle, &uSeekMethod, &uOffset);
    if (RT_SUCCESS(rc))
    {

    }
    return rc;
}


static int VBoxServiceControlHandleFileTell(uint32_t idClient, uint32_t cParms)
{
    uint32_t uContextID;
    uint32_t uHandle;

    int rc = VbglR3GuestCtrlFileGetHostCmdTell(idClient, cParms, &uContextID,
                                               &uHandle);
    if (RT_SUCCESS(rc))
    {

    }
    return rc;
}


/**
 * Handles the guest control output command.
 *
 * @return  IPRT status code.
 * @param   idClient        The HGCM client session ID.
 * @param   cParms          The number of parameters the host is offering.
 */
static int VBoxServiceControlHandleCmdGetOutput(uint32_t idClient, uint32_t cParms)
{
    uint32_t uContextID;
    uint32_t uPID;
    uint32_t uHandleID;
    uint32_t uFlags;

    int rc = VbglR3GuestCtrlExecGetHostCmdOutput(idClient, cParms,
                                                 &uContextID, &uPID, &uHandleID, &uFlags);
    if (RT_SUCCESS(rc))
    {
        uint8_t *pBuf = (uint8_t*)RTMemAlloc(_64K);
        if (pBuf)
        {
            uint32_t cbRead = 0;
            rc = VBoxServiceControlExecGetOutput(uPID, uContextID, uHandleID, RT_INDEFINITE_WAIT /* Timeout */,
                                                 pBuf, _64K /* cbSize */, &cbRead);
            VBoxServiceVerbose(3, "[PID %u]: Got output, rc=%Rrc, CID=%u, cbRead=%u, uHandle=%u, uFlags=%u\n",
                               uPID, rc, uContextID, cbRead, uHandleID, uFlags);

#ifdef DEBUG
            if (   g_fControlDumpStdErr
                && uHandleID == OUTPUT_HANDLE_ID_STDERR)
            {
                char szPID[RTPATH_MAX];
                if (!RTStrPrintf(szPID, sizeof(szPID), "VBoxService_PID%u_StdOut.txt", uPID))
                    rc = VERR_BUFFER_UNDERFLOW;
                if (RT_SUCCESS(rc))
                    rc = vboxServiceControlDump(szPID, pBuf, cbRead);
            }
            else if (   g_fControlDumpStdOut
                     && (   uHandleID == OUTPUT_HANDLE_ID_STDOUT
                         || uHandleID == OUTPUT_HANDLE_ID_STDOUT_DEPRECATED))
            {
                char szPID[RTPATH_MAX];
                if (!RTStrPrintf(szPID, sizeof(szPID), "VBoxService_PID%u_StdOut.txt", uPID))
                    rc = VERR_BUFFER_UNDERFLOW;
                if (RT_SUCCESS(rc))
                    rc = vboxServiceControlDump(szPID, pBuf, cbRead);
                AssertRC(rc);
            }
#endif
            /** Note: Don't convert/touch/modify/whatever the output data here! This might be binary
             *        data which the host needs to work with -- so just pass through all data unfiltered! */

            /* Note: Since the context ID is unique the request *has* to be completed here,
             *       regardless whether we got data or not! Otherwise the progress object
             *       on the host never will get completed! */
            int rc2 = VbglR3GuestCtrlExecSendOut(idClient, uContextID, uPID, uHandleID, uFlags,
                                                 pBuf, cbRead);
            if (RT_SUCCESS(rc))
                rc = rc2;
            else if (rc == VERR_NOT_FOUND) /* It's not critical if guest process (PID) is not found. */
                rc = VINF_SUCCESS;

            RTMemFree(pBuf);
        }
        else
            rc = VERR_NO_MEMORY;
    }

    if (RT_FAILURE(rc))
        VBoxServiceError("[PID %u]: Error handling output command! Error: %Rrc\n",
                         uPID, rc);
    return rc;
}


/** @copydoc VBOXSERVICE::pfnStop */
static DECLCALLBACK(void) VBoxServiceControlStop(void)
{
    VBoxServiceVerbose(3, "Stopping ...\n");

    /** @todo Later, figure what to do if we're in RTProcWait(). It's a very
     *        annoying call since doesn't support timeouts in the posix world. */
    if (g_hControlEvent != NIL_RTSEMEVENTMULTI)
        RTSemEventMultiSignal(g_hControlEvent);

    /*
     * Ask the host service to cancel all pending requests so that we can
     * shutdown properly here.
     */
    if (g_uControlSvcClientID)
    {
        VBoxServiceVerbose(3, "Cancelling pending waits (client ID=%u) ...\n",
                           g_uControlSvcClientID);

        int rc = VbglR3GuestCtrlCancelPendingWaits(g_uControlSvcClientID);
        if (RT_FAILURE(rc))
            VBoxServiceError("Cancelling pending waits failed; rc=%Rrc\n", rc);
    }
}


/**
 * Reaps all inactive guest process threads.
 *
 * @return  IPRT status code.
 */
static int VBoxServiceControlReapThreads(void)
{
    int rc = RTCritSectEnter(&g_csControlThreads);
    if (RT_SUCCESS(rc))
    {
        PVBOXSERVICECTRLTHREAD pThread =
            RTListGetFirst(&g_lstControlThreadsInactive, VBOXSERVICECTRLTHREAD, Node);
        while (pThread)
        {
            PVBOXSERVICECTRLTHREAD pNext = RTListNodeGetNext(&pThread->Node, VBOXSERVICECTRLTHREAD, Node);
            bool fLast = RTListNodeIsLast(&g_lstControlThreadsInactive, &pThread->Node);
            int rc2 = VBoxServiceControlThreadWait(pThread, 30 * 1000 /* 30 seconds max. */,
                                                   NULL /* rc */);
            if (RT_SUCCESS(rc2))
            {
                RTListNodeRemove(&pThread->Node);

                rc2 = VBoxServiceControlThreadFree(pThread);
                if (RT_FAILURE(rc2))
                {
                    VBoxServiceError("Freeing guest process thread failed with rc=%Rrc\n", rc2);
                    if (RT_SUCCESS(rc)) /* Keep original failure. */
                        rc = rc2;
                }
            }
            else
                VBoxServiceError("Waiting on guest process thread failed with rc=%Rrc\n", rc2);
            /* Keep going. */

            if (fLast)
                break;

            pThread = pNext;
        }

        int rc2 = RTCritSectLeave(&g_csControlThreads);
        if (RT_SUCCESS(rc))
            rc = rc2;
    }

    VBoxServiceVerbose(4, "Reaping threads returned with rc=%Rrc\n", rc);
    return rc;
}


/**
 * Closes a formerly opened guest session.
 *
 * @return  IPRT status code.
 * @param   uSessionID              Guest session to close.
 */
static int vboxServiceControlSessionClose(uint32_t uSessionID)
{
    /* Note: Sessions are not implemented yet, so just shutdown all started guest processes. */

    /* Signal all threads in the active list that we want to shutdown. */
    PVBOXSERVICECTRLTHREAD pThread;
    RTListForEach(&g_lstControlThreadsActive, pThread, VBOXSERVICECTRLTHREAD, Node)
        VBoxServiceControlThreadStop(pThread);

    /* Wait for all active threads to shutdown and destroy the active thread list. */
    pThread = RTListGetFirst(&g_lstControlThreadsActive, VBOXSERVICECTRLTHREAD, Node);
    while (pThread)
    {
        PVBOXSERVICECTRLTHREAD pNext = RTListNodeGetNext(&pThread->Node, VBOXSERVICECTRLTHREAD, Node);
        bool fLast = RTListNodeIsLast(&g_lstControlThreadsActive, &pThread->Node);

        int rc2 = VBoxServiceControlThreadWait(pThread,
                                               30 * 1000 /* Wait 30 seconds max. */,
                                               NULL /* rc */);
        if (RT_FAILURE(rc2))
        {
            VBoxServiceError("Guest process thread failed to stop; rc=%Rrc\n", rc2);
            /* Keep going. */
        }

        if (fLast)
            break;

        pThread = pNext;
    }

    int rc = VBoxServiceControlReapThreads();
    if (RT_FAILURE(rc))
        VBoxServiceError("Reaping inactive threads failed with rc=%Rrc\n", rc);

    AssertMsg(RTListIsEmpty(&g_lstControlThreadsActive),
              ("Guest process active thread list still contains entries when it should not\n"));
    AssertMsg(RTListIsEmpty(&g_lstControlThreadsInactive),
              ("Guest process inactive thread list still contains entries when it should not\n"));

    return rc;
}


/**
 * Closes (shuts down) all opened guest sessions.
 *
 * @return  IPRT status code.
 */
static int vboxServiceControlSessionCloseAll(void)
{
    return vboxServiceControlSessionClose(0 /* Session ID, not used yet */);
}


/**
 * Destroys all guest process threads which are still active.
 */
static void VBoxServiceControlShutdown(void)
{
    VBoxServiceVerbose(2, "Shutting down ...\n");

    int rc2 = vboxServiceControlSessionCloseAll();
    AssertRC(rc2);

    /* Destroy critical section. */
    RTCritSectDelete(&g_csControlThreads);

    /* Close all left guest files. */
    PVBOXSERVICECTRLFILE pFile;
    pFile = RTListGetFirst(&g_lstControlFiles, VBOXSERVICECTRLFILE, Node);
    while (pFile)
    {
        PVBOXSERVICECTRLFILE pNext = RTListNodeGetNext(&pFile->Node, VBOXSERVICECTRLFILE, Node);
        bool fLast = RTListNodeIsLast(&g_lstControlFiles, &pFile->Node);

        rc2 = RTFileClose(pFile->hFile);
        if (RT_FAILURE(rc2))
        {
            VBoxServiceError("Unable to close file \"%s\"; rc=%Rrc\n",
                             pFile->szName, rc2);
            /* Keep going. */
        }

        RTListNodeRemove(&pFile->Node);

        if (fLast)
            break;

        pFile = pNext;
    }

    AssertMsg(RTListIsEmpty(&g_lstControlFiles),
              ("Guest file list still contains entries when it should not\n"));

    VBoxServiceVerbose(2, "Shutting down complete\n");
}


/** @copydoc VBOXSERVICE::pfnTerm */
static DECLCALLBACK(void) VBoxServiceControlTerm(void)
{
    VBoxServiceVerbose(3, "Terminating ...\n");

    VBoxServiceControlShutdown();

    VBoxServiceVerbose(3, "Disconnecting client ID=%u ...\n",
                       g_uControlSvcClientID);
    VbglR3GuestCtrlDisconnect(g_uControlSvcClientID);
    g_uControlSvcClientID = 0;

    if (g_hControlEvent != NIL_RTSEMEVENTMULTI)
    {
        RTSemEventMultiDestroy(g_hControlEvent);
        g_hControlEvent = NIL_RTSEMEVENTMULTI;
    }
}


/**
 * Determines whether starting a new guest process according to the
 * maximum number of concurrent guest processes defined is allowed or not.
 *
 * @return  IPRT status code.
 * @param   pbAllowed           True if starting (another) guest process
 *                              is allowed, false if not.
 */
static int VBoxServiceControlStartAllowed(bool *pbAllowed)
{
    AssertPtrReturn(pbAllowed, VERR_INVALID_POINTER);

    int rc = RTCritSectEnter(&g_csControlThreads);
    if (RT_SUCCESS(rc))
    {
        /*
         * Check if we're respecting our memory policy by checking
         * how many guest processes are started and served already.
         */
        bool fLimitReached = false;
        if (g_uControlProcsMaxKept) /* If we allow unlimited processes (=0), take a shortcut. */
        {
            uint32_t uProcsRunning = 0;
            PVBOXSERVICECTRLTHREAD pThread;
            RTListForEach(&g_lstControlThreadsActive, pThread, VBOXSERVICECTRLTHREAD, Node)
                uProcsRunning++;

            VBoxServiceVerbose(3, "Maximum served guest processes set to %u, running=%u\n",
                               g_uControlProcsMaxKept, uProcsRunning);

            int32_t iProcsLeft = (g_uControlProcsMaxKept - uProcsRunning - 1);
            if (iProcsLeft < 0)
            {
                VBoxServiceVerbose(3, "Maximum running guest processes reached (%u)\n",
                                   g_uControlProcsMaxKept);
                fLimitReached = true;
            }
        }

        *pbAllowed = !fLimitReached;

        int rc2 = RTCritSectLeave(&g_csControlThreads);
        if (RT_SUCCESS(rc))
            rc = rc2;
    }

    return rc;
}


/**
 * Finds a (formerly) started process given by its PID and locks it. Must be unlocked
 * by the caller with VBoxServiceControlThreadUnlock().
 *
 * @return  PVBOXSERVICECTRLTHREAD      Process structure if found, otherwise NULL.
 * @param   uPID                        PID to search for.
 */
PVBOXSERVICECTRLTHREAD VBoxServiceControlLockThread(uint32_t uPID)
{
    PVBOXSERVICECTRLTHREAD pThread = NULL;
    int rc = RTCritSectEnter(&g_csControlThreads);
    if (RT_SUCCESS(rc))
    {
        PVBOXSERVICECTRLTHREAD pThreadCur;
        RTListForEach(&g_lstControlThreadsActive, pThreadCur, VBOXSERVICECTRLTHREAD, Node)
        {
            if (pThreadCur->uPID == uPID)
            {
                rc = RTCritSectEnter(&pThreadCur->CritSect);
                if (RT_SUCCESS(rc))
                    pThread = pThreadCur;
                break;
            }
        }

        int rc2 = RTCritSectLeave(&g_csControlThreads);
        if (RT_SUCCESS(rc))
            rc = rc2;
    }

    return pThread;
}


/**
 * Unlocks a previously locked guest process thread.
 *
 * @param   pThread                 Thread to unlock.
 */
void VBoxServiceControlUnlockThread(const PVBOXSERVICECTRLTHREAD pThread)
{
    AssertPtr(pThread);

    int rc = RTCritSectLeave(&pThread->CritSect);
    AssertRC(rc);
}


/**
 * Assigns a valid PID to a guest control thread and also checks if there already was
 * another (stale) guest process which was using that PID before and destroys it.
 *
 * @return  IPRT status code.
 * @param   pThread        Thread to assign PID to.
 * @param   uPID           PID to assign to the specified guest control execution thread.
 */
int VBoxServiceControlAssignPID(PVBOXSERVICECTRLTHREAD pThread, uint32_t uPID)
{
    AssertPtrReturn(pThread, VERR_INVALID_POINTER);
    AssertReturn(uPID, VERR_INVALID_PARAMETER);

    int rc = RTCritSectEnter(&g_csControlThreads);
    if (RT_SUCCESS(rc))
    {
        /* Search old threads using the desired PID and shut them down completely -- it's
         * not used anymore. */
        PVBOXSERVICECTRLTHREAD pThreadCur;
        bool fTryAgain = false;
        do
        {
            RTListForEach(&g_lstControlThreadsActive, pThreadCur, VBOXSERVICECTRLTHREAD, Node)
            {
                if (pThreadCur->uPID == uPID)
                {
                    Assert(pThreadCur != pThread); /* can't happen */
                    uint32_t uTriedPID = uPID;
                    uPID += 391939;
                    VBoxServiceVerbose(2, "PID %u was used before, trying again with %u ...\n",
                                       uTriedPID, uPID);
                    fTryAgain = true;
                    break;
                }
            }
        } while (fTryAgain);

        /* Assign PID to current thread. */
        pThread->uPID = uPID;

        rc = RTCritSectLeave(&g_csControlThreads);
        AssertRC(rc);
    }

    return rc;
}


/**
 * The 'vminfo' service description.
 */
VBOXSERVICE g_Control =
{
    /* pszName. */
    "control",
    /* pszDescription. */
    "Host-driven Guest Control",
    /* pszUsage. */
#ifdef DEBUG
    "              [--control-dump-stderr] [--control-dump-stdout]\n"
#endif
    "              [--control-interval <ms>] [--control-procs-max-kept <x>]\n"
    "              [--control-procs-mem-std[in|out|err] <KB>]"
    ,
    /* pszOptions. */
#ifdef DEBUG
    "    --control-dump-stderr   Dumps all guest proccesses stderr data to the\n"
    "                            temporary directory.\n"
    "    --control-dump-stdout   Dumps all guest proccesses stdout data to the\n"
    "                            temporary directory.\n"
#endif
    "    --control-interval      Specifies the interval at which to check for\n"
    "                            new control commands. The default is 1000 ms.\n"
    "    --control-procs-max-kept\n"
    "                            Specifies how many started guest processes are\n"
    "                            kept into memory to work with. Default is 256.\n"
    ,
    /* methods */
    VBoxServiceControlPreInit,
    VBoxServiceControlOption,
    VBoxServiceControlInit,
    VBoxServiceControlWorker,
    VBoxServiceControlStop,
    VBoxServiceControlTerm
};

