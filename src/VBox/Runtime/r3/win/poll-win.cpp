/* $Id: poll-win.cpp $ */
/** @file
 * IPRT - Polling I/O Handles, Windows Implementation.
 *
 * @todo merge poll-win.cpp and poll-posix.cpp, there is lots of common code.
 */

/*
 * Copyright (C) 2010 Oracle Corporation
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
#include <Windows.h>

#include <iprt/poll.h>
#include "internal/iprt.h"

#include <iprt/asm.h>
#include <iprt/assert.h>
#include <iprt/err.h>
#include <iprt/mem.h>
#include <iprt/pipe.h>
#include <iprt/string.h>
#include <iprt/thread.h>
#include <iprt/time.h>

#include "internal/pipe.h"
#define IPRT_INTERNAL_SOCKET_POLLING_ONLY
#include "internal/socket.h"
#include "internal/magics.h"


/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/
/**
 * Handle entry in a poll set.
 */
typedef struct RTPOLLSETHNDENT
{
    /** The handle type. */
    RTHANDLETYPE    enmType;
    /** The handle ID. */
    uint32_t        id;
    /** The events we're waiting for here. */
    uint32_t        fEvents;
    /** Set if this is the final entry for this handle.
     * If the handle is entered more than once, this will be clear for all but
     * the last entry. */
    bool            fFinalEntry;
    /** The handle union. */
    RTHANDLEUNION   u;
} RTPOLLSETHNDENT;
/** Pointer to a handle entry. */
typedef RTPOLLSETHNDENT *PRTPOLLSETHNDENT;


/**
 * Poll set data, Windows.
 */
typedef struct RTPOLLSETINTERNAL
{
    /** The magic value (RTPOLLSET_MAGIC). */
    uint32_t            u32Magic;
    /** Set when someone is polling or making changes. */
    bool volatile       fBusy;

    /** The number of valid handles in the set. */
    uint32_t            cHandles;
    /** The native handles. */
    HANDLE              ahNative[MAXIMUM_WAIT_OBJECTS];
    /** Array of handles and IDs. */
    RTPOLLSETHNDENT     aHandles[MAXIMUM_WAIT_OBJECTS];
} RTPOLLSETINTERNAL;


/**
 * Common worker for RTPoll and RTPollNoResume
 */
static int rtPollNoResumeWorker(RTPOLLSETINTERNAL *pThis, RTMSINTERVAL cMillies, uint32_t *pfEvents, uint32_t *pid)
{
    int rc;

    if (RT_UNLIKELY(pThis->cHandles == 0 && cMillies == RT_INDEFINITE_WAIT))
        return VERR_DEADLOCK;

    /*
     * Check for special case, RTThreadSleep...
     */
    uint32_t const  cHandles = pThis->cHandles;
    if (cHandles == 0)
    {
        rc = RTThreadSleep(cMillies);
        if (RT_SUCCESS(rc))
            rc = VERR_TIMEOUT;
        return rc;
    }

    /*
     * Check + prepare the handles before waiting.
     */
    uint32_t        fEvents  = 0;
    bool const      fNoWait  = cMillies == 0;
    uint32_t        i;
    for (i = 0; i < cHandles; i++)
    {
        switch (pThis->aHandles[i].enmType)
        {
            case RTHANDLETYPE_PIPE:
                fEvents = rtPipePollStart(pThis->aHandles[i].u.hPipe, pThis, pThis->aHandles[i].fEvents,
                                          pThis->aHandles[i].fFinalEntry, fNoWait);
                break;

            case RTHANDLETYPE_SOCKET:
                fEvents = rtSocketPollStart(pThis->aHandles[i].u.hSocket, pThis, pThis->aHandles[i].fEvents,
                                            pThis->aHandles[i].fFinalEntry, fNoWait);
                break;

            default:
                AssertFailed();
                fEvents = UINT32_MAX;
                break;
        }
        if (fEvents)
            break;
    }
    if (   fEvents
        || fNoWait)
    {

        if (pid)
            *pid = pThis->aHandles[i].id;
        if (pfEvents)
            *pfEvents = fEvents;
        rc = !fEvents
           ? VERR_TIMEOUT
           : fEvents != UINT32_MAX
           ? VINF_SUCCESS
           : VERR_INTERNAL_ERROR_4;

        /* clean up */
        if (!fNoWait)
            while (i-- > 0)
            {
                switch (pThis->aHandles[i].enmType)
                {
                    case RTHANDLETYPE_PIPE:
                        rtPipePollDone(pThis->aHandles[i].u.hPipe, pThis->aHandles[i].fEvents,
                                       pThis->aHandles[i].fFinalEntry, false);
                        break;

                    case RTHANDLETYPE_SOCKET:
                        rtSocketPollDone(pThis->aHandles[i].u.hSocket, pThis->aHandles[i].fEvents,
                                         pThis->aHandles[i].fFinalEntry, false);
                        break;

                    default:
                        AssertFailed();
                        break;
                }
            }

        return rc;
    }

    /*
     * Wait.
     */
    DWORD dwRc = WaitForMultipleObjectsEx(cHandles, &pThis->ahNative[0],
                                          FALSE /*fWaitAll */,
                                          cMillies == RT_INDEFINITE_WAIT ? INFINITE : cMillies,
                                          TRUE /*fAlertable*/);
    if (    dwRc >= WAIT_OBJECT_0
        &&  dwRc <  WAIT_OBJECT_0 + cHandles)
        rc = VERR_INTERRUPTED;
    else if (dwRc == WAIT_TIMEOUT)
        rc = VERR_TIMEOUT;
    else if (dwRc == WAIT_IO_COMPLETION)
        rc = VERR_INTERRUPTED;
    else if (dwRc == WAIT_FAILED)
        rc = RTErrConvertFromWin32(GetLastError());
    else
    {
        AssertMsgFailed(("%u (%#x)\n", dwRc, dwRc));
        rc = VERR_INTERNAL_ERROR_5;
    }

    /*
     * Get event (if pending) and do wait cleanup.
     */
    bool fHarvestEvents = true;
    for (i = 0; i < cHandles; i++)
    {
        fEvents = 0;
        switch (pThis->aHandles[i].enmType)
        {
            case RTHANDLETYPE_PIPE:
                fEvents = rtPipePollDone(pThis->aHandles[i].u.hPipe, pThis->aHandles[i].fEvents,
                                         pThis->aHandles[i].fFinalEntry, fHarvestEvents);
                break;

            case RTHANDLETYPE_SOCKET:
                fEvents = rtSocketPollDone(pThis->aHandles[i].u.hSocket, pThis->aHandles[i].fEvents,
                                           pThis->aHandles[i].fFinalEntry, fHarvestEvents);
                break;

            default:
                AssertFailed();
                break;
        }
        if (   fEvents
            && fHarvestEvents)
        {
            Assert(fEvents != UINT32_MAX);
            fHarvestEvents = false;
            if (pfEvents)
                *pfEvents = fEvents;
            if (pid)
                *pid = pThis->aHandles[i].id;
            rc = VINF_SUCCESS;
        }
    }

    return rc;
}


RTDECL(int) RTPoll(RTPOLLSET hPollSet, RTMSINTERVAL cMillies, uint32_t *pfEvents, uint32_t *pid)
{
    RTPOLLSETINTERNAL *pThis = hPollSet;
    AssertPtrReturn(pThis, VERR_INVALID_HANDLE);
    AssertReturn(pThis->u32Magic == RTPOLLSET_MAGIC, VERR_INVALID_HANDLE);
    AssertPtrNull(pfEvents);
    AssertPtrNull(pid);

    /*
     * Set the busy flag and do the job.
     */
    AssertReturn(ASMAtomicCmpXchgBool(&pThis->fBusy, true,  false), VERR_CONCURRENT_ACCESS);

    int rc;
    if (cMillies == RT_INDEFINITE_WAIT || cMillies == 0)
    {
        do rc = rtPollNoResumeWorker(pThis, cMillies, pfEvents, pid);
        while (rc == VERR_INTERRUPTED);
    }
    else
    {
        uint64_t MsStart = RTTimeMilliTS();
        rc = rtPollNoResumeWorker(pThis, cMillies, pfEvents, pid);
        while (RT_UNLIKELY(rc == VERR_INTERRUPTED))
        {
            if (RTTimeMilliTS() - MsStart >= cMillies)
            {
                rc = VERR_TIMEOUT;
                break;
            }
            rc = rtPollNoResumeWorker(pThis, cMillies, pfEvents, pid);
        }
    }

    ASMAtomicWriteBool(&pThis->fBusy, false);

    return rc;
}


RTDECL(int) RTPollNoResume(RTPOLLSET hPollSet, RTMSINTERVAL cMillies, uint32_t *pfEvents, uint32_t *pid)
{
    RTPOLLSETINTERNAL *pThis = hPollSet;
    AssertPtrReturn(pThis, VERR_INVALID_HANDLE);
    AssertReturn(pThis->u32Magic == RTPOLLSET_MAGIC, VERR_INVALID_HANDLE);
    AssertPtrNull(pfEvents);
    AssertPtrNull(pid);

    /*
     * Set the busy flag and do the job.
     */
    AssertReturn(ASMAtomicCmpXchgBool(&pThis->fBusy, true,  false), VERR_CONCURRENT_ACCESS);

    int rc = rtPollNoResumeWorker(pThis, cMillies, pfEvents, pid);

    ASMAtomicWriteBool(&pThis->fBusy, false);

    return rc;
}


RTDECL(int) RTPollSetCreate(PRTPOLLSET phPollSet)
{
    AssertPtrReturn(phPollSet, VERR_INVALID_POINTER);
    RTPOLLSETINTERNAL *pThis = (RTPOLLSETINTERNAL *)RTMemAllocZ(sizeof(RTPOLLSETINTERNAL));
    if (!pThis)
        return VERR_NO_MEMORY;

    pThis->u32Magic             = RTPOLLSET_MAGIC;
    pThis->fBusy                = false;
    pThis->cHandles             = 0;
    for (size_t i = 0; i < RT_ELEMENTS(pThis->ahNative); i++)
        pThis->ahNative[i]     = INVALID_HANDLE_VALUE;

    *phPollSet = pThis;
    return VINF_SUCCESS;
}


RTDECL(int) RTPollSetDestroy(RTPOLLSET hPollSet)
{
    RTPOLLSETINTERNAL *pThis = hPollSet;
    if (pThis == NIL_RTPOLLSET)
        return VINF_SUCCESS;
    AssertPtrReturn(pThis, VERR_INVALID_HANDLE);
    AssertReturn(pThis->u32Magic == RTPOLLSET_MAGIC, VERR_INVALID_HANDLE);
    AssertReturn(ASMAtomicCmpXchgBool(&pThis->fBusy, true,  false), VERR_CONCURRENT_ACCESS);

    ASMAtomicWriteU32(&pThis->u32Magic, ~RTPOLLSET_MAGIC);
    RTMemFree(pThis);

    return VINF_SUCCESS;
}


RTDECL(int) RTPollSetAdd(RTPOLLSET hPollSet, PCRTHANDLE pHandle, uint32_t fEvents, uint32_t id)
{
    /*
     * Validate the input (tedious).
     */
    RTPOLLSETINTERNAL *pThis = hPollSet;
    AssertPtrReturn(pThis, VERR_INVALID_HANDLE);
    AssertReturn(pThis->u32Magic == RTPOLLSET_MAGIC, VERR_INVALID_HANDLE);
    AssertReturn(!(fEvents & ~RTPOLL_EVT_VALID_MASK), VERR_INVALID_PARAMETER);
    AssertReturn(fEvents, VERR_INVALID_PARAMETER);
    AssertReturn(id != UINT32_MAX, VERR_INVALID_PARAMETER);

    if (!pHandle)
        return VINF_SUCCESS;
    AssertPtrReturn(pHandle, VERR_INVALID_POINTER);
    AssertReturn(pHandle->enmType > RTHANDLETYPE_INVALID && pHandle->enmType < RTHANDLETYPE_END, VERR_INVALID_PARAMETER);

    /*
     * Set the busy flag and do the job.
     */
    AssertReturn(ASMAtomicCmpXchgBool(&pThis->fBusy, true,  false), VERR_CONCURRENT_ACCESS);

    int             rc       = VINF_SUCCESS;
    HANDLE          hNative  = INVALID_HANDLE_VALUE;
    RTHANDLEUNION   uh;
    uh.uInt = 0;
    switch (pHandle->enmType)
    {
        case RTHANDLETYPE_PIPE:
            uh.hPipe = pHandle->u.hPipe;
            if (uh.hPipe != NIL_RTPIPE)
                rc = rtPipePollGetHandle(uh.hPipe, fEvents, &hNative);
            break;

        case RTHANDLETYPE_SOCKET:
            uh.hSocket = pHandle->u.hSocket;
            if (uh.hSocket != NIL_RTSOCKET)
                rc = rtSocketPollGetHandle(uh.hSocket, fEvents, &hNative);
            break;

        case RTHANDLETYPE_FILE:
            AssertMsgFailed(("Files are always ready for reading/writing and thus not pollable. Use native APIs for special devices.\n"));
            rc = VERR_POLL_HANDLE_NOT_POLLABLE;
            break;

        case RTHANDLETYPE_THREAD:
            AssertMsgFailed(("Thread handles are currently not pollable\n"));
            rc = VERR_POLL_HANDLE_NOT_POLLABLE;
            break;

        default:
            AssertMsgFailed(("\n"));
            rc = VERR_POLL_HANDLE_NOT_POLLABLE;
            break;
    }
    if (   RT_SUCCESS(rc)
        && hNative != INVALID_HANDLE_VALUE)
    {
        uint32_t const i = pThis->cHandles;

        /* Check that the handle ID doesn't exist already. */
        uint32_t iPrev = UINT32_MAX;
        uint32_t j     = i;
        while (j-- > 0)
        {
            if (pThis->aHandles[j].id == id)
            {
                rc = VERR_POLL_HANDLE_ID_EXISTS;
                break;
            }
            if (   pThis->aHandles[j].enmType == pHandle->enmType
                && pThis->aHandles[j].u.uInt  == uh.uInt)
                iPrev = j;
        }

        /* Check that we won't overflow the poll set now. */
        if (    RT_SUCCESS(rc)
            &&  i + 1 > RT_ELEMENTS(pThis->ahNative))
            rc = VERR_POLL_SET_IS_FULL;
        if (RT_SUCCESS(rc))
        {
            /* Add the handles to the two parallel arrays. */
            pThis->ahNative[i]             = hNative;
            pThis->aHandles[i].enmType     = pHandle->enmType;
            pThis->aHandles[i].u           = uh;
            pThis->aHandles[i].id          = id;
            pThis->aHandles[i].fEvents     = fEvents;
            pThis->aHandles[i].fFinalEntry = true;
            pThis->cHandles = i + 1;

            if (iPrev != UINT32_MAX)
            {
                Assert(pThis->aHandles[i].fFinalEntry);
                pThis->aHandles[i].fFinalEntry = false;
            }

            rc = VINF_SUCCESS;
        }
    }

    ASMAtomicWriteBool(&pThis->fBusy, false);
    return rc;
}


RTDECL(int) RTPollSetRemove(RTPOLLSET hPollSet, uint32_t id)
{
    /*
     * Validate the input.
     */
    RTPOLLSETINTERNAL *pThis = hPollSet;
    AssertPtrReturn(pThis, VERR_INVALID_HANDLE);
    AssertReturn(pThis->u32Magic == RTPOLLSET_MAGIC, VERR_INVALID_HANDLE);
    AssertReturn(id != UINT32_MAX, VERR_INVALID_PARAMETER);

    /*
     * Set the busy flag and do the job.
     */
    AssertReturn(ASMAtomicCmpXchgBool(&pThis->fBusy, true,  false), VERR_CONCURRENT_ACCESS);

    int         rc = VERR_POLL_HANDLE_ID_NOT_FOUND;
    uint32_t    i  = pThis->cHandles;
    while (i-- > 0)
        if (pThis->aHandles[i].id == id)
        {
            /* Save some details for the duplicate searching. */
            bool            fFinalEntry = pThis->aHandles[i].fFinalEntry;
            RTHANDLETYPE    enmType     = pThis->aHandles[i].enmType;
            RTHANDLEUNION   uh          = pThis->aHandles[i].u;

            /* Remove the entry. */
            pThis->cHandles--;
            size_t const cToMove = pThis->cHandles - i;
            if (cToMove)
            {
                memmove(&pThis->aHandles[i], &pThis->aHandles[i + 1], cToMove * sizeof(pThis->aHandles[i]));
                memmove(&pThis->ahNative[i], &pThis->ahNative[i + 1], cToMove * sizeof(pThis->ahNative[i]));
            }

            /* Check for duplicate and set the fFinalEntry flag. */
            if (fFinalEntry)
                while (i-- > 0)
                    if (   pThis->aHandles[i].u.uInt  == uh.uInt
                        && pThis->aHandles[i].enmType == enmType)
                    {
                        Assert(!pThis->aHandles[i].fFinalEntry);
                        pThis->aHandles[i].fFinalEntry = true;
                        break;
                    }

            rc = VINF_SUCCESS;
            break;
        }

    ASMAtomicWriteBool(&pThis->fBusy, false);
    return rc;
}


RTDECL(int) RTPollSetQueryHandle(RTPOLLSET hPollSet, uint32_t id, PRTHANDLE pHandle)
{
    /*
     * Validate the input.
     */
    RTPOLLSETINTERNAL *pThis = hPollSet;
    AssertPtrReturn(pThis, VERR_INVALID_HANDLE);
    AssertReturn(pThis->u32Magic == RTPOLLSET_MAGIC, VERR_INVALID_HANDLE);
    AssertReturn(id != UINT32_MAX, VERR_INVALID_PARAMETER);
    AssertPtrNullReturn(pHandle, VERR_INVALID_POINTER);

    /*
     * Set the busy flag and do the job.
     */
    AssertReturn(ASMAtomicCmpXchgBool(&pThis->fBusy, true,  false), VERR_CONCURRENT_ACCESS);

    int         rc = VERR_POLL_HANDLE_ID_NOT_FOUND;
    uint32_t    i  = pThis->cHandles;
    while (i-- > 0)
        if (pThis->aHandles[i].id == id)
        {
            if (pHandle)
            {
                pHandle->enmType = pThis->aHandles[i].enmType;
                pHandle->u       = pThis->aHandles[i].u;
            }
            rc = VINF_SUCCESS;
            break;
        }

    ASMAtomicWriteBool(&pThis->fBusy, false);
    return rc;
}


RTDECL(uint32_t) RTPollSetGetCount(RTPOLLSET hPollSet)
{
    /*
     * Validate the input.
     */
    RTPOLLSETINTERNAL *pThis = hPollSet;
    AssertPtrReturn(pThis, UINT32_MAX);
    AssertReturn(pThis->u32Magic == RTPOLLSET_MAGIC, UINT32_MAX);

    /*
     * Set the busy flag and do the job.
     */
    AssertReturn(ASMAtomicCmpXchgBool(&pThis->fBusy, true,  false), UINT32_MAX);
    uint32_t cHandles = pThis->cHandles;
    ASMAtomicWriteBool(&pThis->fBusy, false);

    return cHandles;
}

RTDECL(int) RTPollSetEventsChange(RTPOLLSET hPollSet, uint32_t id, uint32_t fEvents)
{
    /*
     * Validate the input.
     */
    RTPOLLSETINTERNAL *pThis = hPollSet;
    AssertPtrReturn(pThis, VERR_INVALID_HANDLE);
    AssertReturn(pThis->u32Magic == RTPOLLSET_MAGIC, VERR_INVALID_HANDLE);
    AssertReturn(id != UINT32_MAX, VERR_INVALID_PARAMETER);
    AssertReturn(!(fEvents & ~RTPOLL_EVT_VALID_MASK), VERR_INVALID_PARAMETER);
    AssertReturn(fEvents, VERR_INVALID_PARAMETER);

    /*
     * Set the busy flag and do the job.
     */
    AssertReturn(ASMAtomicCmpXchgBool(&pThis->fBusy, true,  false), VERR_CONCURRENT_ACCESS);

    int         rc = VERR_POLL_HANDLE_ID_NOT_FOUND;
    uint32_t    i  = pThis->cHandles;
    while (i-- > 0)
        if (pThis->aHandles[i].id == id)
        {
            pThis->aHandles[i].fEvents = fEvents;
            rc = VINF_SUCCESS;
            break;
        }

    ASMAtomicWriteBool(&pThis->fBusy, false);
    return rc;
}

