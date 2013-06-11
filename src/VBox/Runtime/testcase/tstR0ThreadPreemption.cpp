/* $Id: tstR0ThreadPreemption.cpp $ */
/** @file
 * IPRT R0 Testcase - Thread Preemption.
 */

/*
 * Copyright (C) 2009-2011 Oracle Corporation
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
#include <iprt/thread.h>

#include <iprt/asm-amd64-x86.h>
#include <iprt/err.h>
#include <iprt/time.h>
#include <iprt/string.h>
#include <VBox/sup.h>
#include "tstR0ThreadPreemption.h"



/**
 * Service request callback function.
 *
 * @returns VBox status code.
 * @param   pSession    The caller's session.
 * @param   u64Arg      64-bit integer argument.
 * @param   pReqHdr     The request header. Input / Output. Optional.
 */
DECLEXPORT(int) TSTR0ThreadPreemptionSrvReqHandler(PSUPDRVSESSION pSession, uint32_t uOperation,
                                                   uint64_t u64Arg, PSUPR0SERVICEREQHDR pReqHdr)
{
    NOREF(pSession);
    if (u64Arg)
        return VERR_INVALID_PARAMETER;
    if (!VALID_PTR(pReqHdr))
        return VERR_INVALID_PARAMETER;
    char   *pszErr = (char *)(pReqHdr + 1);
    size_t  cchErr = pReqHdr->cbReq - sizeof(*pReqHdr);
    if (cchErr < 32 || cchErr >= 0x10000)
        return VERR_INVALID_PARAMETER;
    *pszErr = '\0';

    /*
     * The big switch.
     */
    switch (uOperation)
    {
        case TSTR0THREADPREMEPTION_SANITY_OK:
            break;

        case TSTR0THREADPREMEPTION_SANITY_FAILURE:
            RTStrPrintf(pszErr, cchErr, "!42failure42%1024s", "");
            break;

        case TSTR0THREADPREMEPTION_BASIC:
        {
            if (!ASMIntAreEnabled())
                RTStrPrintf(pszErr, cchErr, "!Interrupts disabled");
            else if (!RTThreadPreemptIsEnabled(NIL_RTTHREAD))
                RTStrPrintf(pszErr, cchErr, "!RTThreadPreemptIsEnabled returns false by default");
            else
            {
                RTTHREADPREEMPTSTATE State = RTTHREADPREEMPTSTATE_INITIALIZER;
                RTThreadPreemptDisable(&State);
                if (RTThreadPreemptIsEnabled(NIL_RTTHREAD))
                    RTStrPrintf(pszErr, cchErr, "!RTThreadPreemptIsEnabled returns true after RTThreadPreemptDisable");
                else if (!ASMIntAreEnabled())
                    RTStrPrintf(pszErr, cchErr, "!Interrupts disabled");
                RTThreadPreemptRestore(&State);
            }
            break;
        }

        case TSTR0THREADPREMEPTION_IS_PENDING:
        {
            RTTHREADPREEMPTSTATE State = RTTHREADPREEMPTSTATE_INITIALIZER;
            RTThreadPreemptDisable(&State);
            if (!RTThreadPreemptIsEnabled(NIL_RTTHREAD))
            {
                if (ASMIntAreEnabled())
                {
                    uint64_t    u64StartTS    = RTTimeNanoTS();
                    uint64_t    u64StartSysTS = RTTimeSystemNanoTS();
                    uint64_t    cLoops        = 0;
                    uint64_t    cNanosSysElapsed;
                    uint64_t    cNanosElapsed;
                    bool        fPending;
                    do
                    {
                        fPending         = RTThreadPreemptIsPending(NIL_RTTHREAD);
                        cNanosElapsed    = RTTimeNanoTS()       - u64StartTS;
                        cNanosSysElapsed = RTTimeSystemNanoTS() - u64StartSysTS;
                        cLoops++;
                    } while (   !fPending
                             && cNanosElapsed    < UINT64_C(2)*1000U*1000U*1000U
                             && cNanosSysElapsed < UINT64_C(2)*1000U*1000U*1000U
                             && cLoops           < 100U*_1M);
                    if (!fPending)
                        RTStrPrintf(pszErr, cchErr, "!Preempt not pending after %'llu loops / %'llu ns / %'llu ns (sys)",
                                    cLoops, cNanosElapsed, cNanosSysElapsed);
                    else if (cLoops == 1)
                        RTStrPrintf(pszErr, cchErr, "!cLoops=1\n");
                    else
                        RTStrPrintf(pszErr, cchErr, "RTThreadPreemptIsPending returned true after %'llu loops / %'llu ns / %'llu ns (sys)",
                                    cLoops, cNanosElapsed, cNanosSysElapsed);
                }
                else
                    RTStrPrintf(pszErr, cchErr, "!Interrupts disabled");
            }
            else
                RTStrPrintf(pszErr, cchErr, "!RTThreadPreemptIsEnabled returns true after RTThreadPreemptDisable");
            RTThreadPreemptRestore(&State);
            break;
        }

        case TSTR0THREADPREMEPTION_NESTED:
        {
            bool const fDefault = RTThreadPreemptIsEnabled(NIL_RTTHREAD);
            RTTHREADPREEMPTSTATE State1 = RTTHREADPREEMPTSTATE_INITIALIZER;
            RTThreadPreemptDisable(&State1);
            if (!RTThreadPreemptIsEnabled(NIL_RTTHREAD))
            {
                RTTHREADPREEMPTSTATE State2 = RTTHREADPREEMPTSTATE_INITIALIZER;
                RTThreadPreemptDisable(&State2);
                if (!RTThreadPreemptIsEnabled(NIL_RTTHREAD))
                {
                    RTTHREADPREEMPTSTATE State3 = RTTHREADPREEMPTSTATE_INITIALIZER;
                    RTThreadPreemptDisable(&State3);
                    if (RTThreadPreemptIsEnabled(NIL_RTTHREAD))
                        RTStrPrintf(pszErr, cchErr, "!RTThreadPreemptIsEnabled returns true after 3rd RTThreadPreemptDisable");

                    RTThreadPreemptRestore(&State3);
                    if (RTThreadPreemptIsEnabled(NIL_RTTHREAD) && !*pszErr)
                        RTStrPrintf(pszErr, cchErr, "!RTThreadPreemptIsEnabled returns true after 1st RTThreadPreemptRestore");
                }
                else
                    RTStrPrintf(pszErr, cchErr, "!RTThreadPreemptIsEnabled returns true after 2nd RTThreadPreemptDisable");

                RTThreadPreemptRestore(&State2);
                if (RTThreadPreemptIsEnabled(NIL_RTTHREAD) && !*pszErr)
                    RTStrPrintf(pszErr, cchErr, "!RTThreadPreemptIsEnabled returns true after 2nd RTThreadPreemptRestore");
            }
            else
                RTStrPrintf(pszErr, cchErr, "!RTThreadPreemptIsEnabled returns true after 1st RTThreadPreemptDisable");
            RTThreadPreemptRestore(&State1);
            if (RTThreadPreemptIsEnabled(NIL_RTTHREAD) != fDefault && !*pszErr)
                RTStrPrintf(pszErr, cchErr, "!RTThreadPreemptIsEnabled returns false after 3rd RTThreadPreemptRestore");
            break;
        }

        default:
            RTStrPrintf(pszErr, cchErr, "!Unknown test #%d", uOperation);
            break;
    }

    /* The error indicator is the '!' in the message buffer. */
    return VINF_SUCCESS;
}

