/* $Id: once.cpp $ */
/** @file
 * IPRT - Execute Once.
 */

/*
 * Copyright (C) 2007-2011 Oracle Corporation
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
#include <iprt/once.h>
#include "internal/iprt.h"

#include <iprt/semaphore.h>
#include <iprt/thread.h>
#include <iprt/err.h>
#include <iprt/assert.h>
#include <iprt/asm.h>


/**
 * The state loop of the other threads.
 *
 * @returns VINF_SUCCESS when everything went smoothly. IPRT status code if we
 *          encountered trouble.
 * @param   pOnce           The execute once structure.
 * @param   phEvtM          Where to store the semaphore handle so the caller
 *                          can do the cleaning up for us.
 */
static int rtOnceOtherThread(PRTONCE pOnce, PRTSEMEVENTMULTI phEvtM)
{
    uint32_t cYields = 0;
    for (;;)
    {
        int32_t iState = ASMAtomicReadS32(&pOnce->iState);
        switch (iState)
        {
            /*
             * No semaphore, try create one.
             */
            case RTONCESTATE_BUSY_NO_SEM:
                if (ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_BUSY_CREATING_SEM, RTONCESTATE_BUSY_NO_SEM))
                {
                    int rc = RTSemEventMultiCreate(phEvtM);
                    if (RT_SUCCESS(rc))
                    {
                        ASMAtomicWriteHandle(&pOnce->hEventMulti, *phEvtM);
                        int32_t cRefs = ASMAtomicIncS32(&pOnce->cEventRefs); Assert(cRefs == 1); NOREF(cRefs);

                        if (!ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_BUSY_HAVE_SEM, RTONCESTATE_BUSY_CREATING_SEM))
                        {
                            /* Too slow. */
                            AssertReturn(ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_DONE, RTONCESTATE_DONE_CREATING_SEM)
                                         , VERR_INTERNAL_ERROR_5);

                            ASMAtomicWriteHandle(&pOnce->hEventMulti, NIL_RTSEMEVENTMULTI);
                            cRefs = ASMAtomicDecS32(&pOnce->cEventRefs); Assert(cRefs == 0);

                            RTSemEventMultiDestroy(*phEvtM);
                            *phEvtM = NIL_RTSEMEVENTMULTI;
                        }
                    }
                    else
                    {
                        AssertReturn(   ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_BUSY_SPIN, RTONCESTATE_BUSY_CREATING_SEM)
                                     || ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_DONE,      RTONCESTATE_DONE_CREATING_SEM)
                                     , VERR_INTERNAL_ERROR_4);
                        *phEvtM = NIL_RTSEMEVENTMULTI;
                    }
                }
                break;

            /*
             * This isn't nice, but it's the easy way out.
             */
            case RTONCESTATE_BUSY_CREATING_SEM:
            case RTONCESTATE_BUSY_SPIN:
                cYields++;
                if (!(++cYields % 8))
                    RTThreadSleep(1);
                else
                    RTThreadYield();
                break;

            /*
             * There is a semaphore, try wait on it.
             *
             * We continue waiting after reaching DONE_HAVE_SEM if we
             * already got the semaphore to avoid racing the first thread.
             */
            case RTONCESTATE_DONE_HAVE_SEM:
                if (*phEvtM == NIL_RTSEMEVENTMULTI)
                    return VINF_SUCCESS;
                /* fall thru */
            case RTONCESTATE_BUSY_HAVE_SEM:
            {
                /*
                 * Grab the semaphore if we haven't got it yet.
                 * We must take care not to increment the counter if it
                 * is 0.  This may happen if we're racing a state change.
                 */
                if (*phEvtM == NIL_RTSEMEVENTMULTI)
                {
                    int32_t cEventRefs = ASMAtomicUoReadS32(&pOnce->cEventRefs);
                    while (   cEventRefs > 0
                           && ASMAtomicUoReadS32(&pOnce->iState) == RTONCESTATE_BUSY_HAVE_SEM)
                    {
                        if (ASMAtomicCmpXchgExS32(&pOnce->cEventRefs, cEventRefs + 1, cEventRefs, &cEventRefs))
                            break;
                        ASMNopPause();
                    }
                    if (cEventRefs <= 0)
                        break;

                    ASMAtomicReadHandle(&pOnce->hEventMulti, phEvtM);
                    AssertReturn(*phEvtM != NIL_RTSEMEVENTMULTI, VERR_INTERNAL_ERROR_2);
                }

                /*
                 * We've got a sempahore, do the actual waiting.
                 */
                do
                    RTSemEventMultiWaitNoResume(*phEvtM, RT_INDEFINITE_WAIT);
                while (ASMAtomicReadS32(&pOnce->iState) == RTONCESTATE_BUSY_HAVE_SEM);
                break;
            }

            case RTONCESTATE_DONE_CREATING_SEM:
            case RTONCESTATE_DONE:
                return VINF_SUCCESS;

            default:
                AssertMsgFailedReturn(("%d\n", iState), VERR_INTERNAL_ERROR_3);
        }
    }
}


RTDECL(int) RTOnceSlow(PRTONCE pOnce, PFNRTONCE pfnOnce, void *pvUser1, void *pvUser2)
{
    /*
     * Validate input (strict builds only).
     */
    AssertPtr(pOnce);
    AssertPtr(pfnOnce);

    /*
     * Deal with the 'initialized' case first
     */
    int32_t iState = ASMAtomicUoReadS32(&pOnce->iState);
    if (RT_LIKELY(   iState == RTONCESTATE_DONE
                  || iState == RTONCESTATE_DONE_CREATING_SEM
                  || iState == RTONCESTATE_DONE_HAVE_SEM
                 ))
        return ASMAtomicUoReadS32(&pOnce->rc);

    AssertReturn(   iState == RTONCESTATE_UNINITIALIZED
                 || iState == RTONCESTATE_BUSY_NO_SEM
                 || iState == RTONCESTATE_BUSY_SPIN
                 || iState == RTONCESTATE_BUSY_CREATING_SEM
                 || iState == RTONCESTATE_BUSY_HAVE_SEM
                 , VERR_INTERNAL_ERROR);

    /*
     * Do we initialize it?
     */
    int32_t rcOnce;
    if (   iState == RTONCESTATE_UNINITIALIZED
        && ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_BUSY_NO_SEM, RTONCESTATE_UNINITIALIZED))
    {
        /*
         * Yes, so do the execute once stuff.
         */
        rcOnce = pfnOnce(pvUser1, pvUser2);
        ASMAtomicWriteS32(&pOnce->rc, rcOnce);

        /*
         * If there is a sempahore to signal, we're in for some extra work here.
         */
        if (   !ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_DONE,              RTONCESTATE_BUSY_NO_SEM)
            && !ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_DONE,              RTONCESTATE_BUSY_SPIN)
            && !ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_DONE_CREATING_SEM, RTONCESTATE_BUSY_CREATING_SEM)
           )
        {
            /* Grab the sempahore by switching to 'DONE_HAVE_SEM' before reaching 'DONE'. */
            AssertReturn(ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_DONE_HAVE_SEM, RTONCESTATE_BUSY_HAVE_SEM),
                         VERR_INTERNAL_ERROR_2);

            int32_t cRefs = ASMAtomicIncS32(&pOnce->cEventRefs);
            Assert(cRefs > 1); NOREF(cRefs);

            RTSEMEVENTMULTI hEvtM;
            ASMAtomicReadHandle(&pOnce->hEventMulti, &hEvtM);
            Assert(hEvtM != NIL_RTSEMEVENTMULTI);

            ASMAtomicWriteS32(&pOnce->iState, RTONCESTATE_DONE);

            /* Signal it and return. */
            RTSemEventMultiSignal(hEvtM);
        }
    }
    else
    {
        /*
         * Wait for the first thread to complete.  Delegate this to a helper
         * function to simplify cleanup and keep things a bit shorter.
         */
        RTSEMEVENTMULTI hEvtM = NIL_RTSEMEVENTMULTI;
        rcOnce = rtOnceOtherThread(pOnce, &hEvtM);
        if (hEvtM != NIL_RTSEMEVENTMULTI)
        {
            if (ASMAtomicDecS32(&pOnce->cEventRefs) == 0)
            {
                bool fRc;
                ASMAtomicCmpXchgHandle(&pOnce->hEventMulti, NIL_RTSEMEVENTMULTI, hEvtM, fRc);           Assert(fRc);
                fRc = ASMAtomicCmpXchgS32(&pOnce->iState, RTONCESTATE_DONE, RTONCESTATE_DONE_HAVE_SEM); Assert(fRc);
                RTSemEventMultiDestroy(hEvtM);
            }
        }
        if (RT_SUCCESS(rcOnce))
            rcOnce = ASMAtomicUoReadS32(&pOnce->rc);
    }

    return rcOnce;
}
RT_EXPORT_SYMBOL(RTOnceSlow);


RTDECL(void) RTOnceReset(PRTONCE pOnce)
{
    /* Cannot be done while busy! */
    AssertPtr(pOnce);
    Assert(pOnce->hEventMulti == NIL_RTSEMEVENTMULTI);
    int32_t iState = ASMAtomicUoReadS32(&pOnce->iState);
    AssertMsg(   iState == RTONCESTATE_DONE
              && iState == RTONCESTATE_UNINITIALIZED,
              ("%d\n", iState));
    NOREF(iState);

    /* Do the same as RTONCE_INITIALIZER does. */
    ASMAtomicWriteS32(&pOnce->rc, VERR_INTERNAL_ERROR);
    ASMAtomicWriteS32(&pOnce->iState, RTONCESTATE_UNINITIALIZED);
}
RT_EXPORT_SYMBOL(RTOnceReset);

