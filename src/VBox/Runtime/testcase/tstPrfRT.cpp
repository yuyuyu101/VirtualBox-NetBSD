/* $Id: tstPrfRT.cpp $ */
/** @file
 * IPRT testcase - profile some of the important functions.
 */

/*
 * Copyright (C) 2006-2011 Oracle Corporation
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
#include <iprt/initterm.h>
#include <iprt/time.h>
#include <iprt/log.h>
#include <iprt/stream.h>
#include <iprt/thread.h>
#if defined(RT_ARCH_AMD64) || defined(RT_ARCH_X86)
# include <iprt/asm-amd64-x86.h>


void PrintResult(uint64_t u64Ticks, uint64_t u64MaxTicks, uint64_t u64MinTicks, unsigned cTimes, const char *pszOperation)
{
    RTPrintf("tstPrfRT: %-32s %5lld / %5lld / %5lld ticks per call (%u calls %lld ticks)\n",
             pszOperation, u64MinTicks, u64Ticks / (uint64_t)cTimes, u64MaxTicks, cTimes, u64Ticks);
}

# define ITERATE(preexpr, expr, postexpr, cIterations) \
    for (i = 0, u64TotalTS = 0, u64MinTS = ~0, u64MaxTS = 0; i < (cIterations); i++) \
    { \
        { preexpr } \
        uint64_t u64StartTS = ASMReadTSC(); \
        { expr } \
        uint64_t u64ElapsedTS = ASMReadTSC() - u64StartTS; \
        { postexpr } \
        if (u64ElapsedTS > u64MinTS * 32) \
        { \
            i--; \
            continue; \
        } \
        if (u64ElapsedTS < u64MinTS) \
            u64MinTS = u64ElapsedTS; \
        if (u64ElapsedTS > u64MaxTS) \
            u64MaxTS = u64ElapsedTS; \
        u64TotalTS += u64ElapsedTS; \
    }

#else  /* !AMD64 && !X86 */

void PrintResult(uint64_t cNs, uint64_t cNsMax, uint64_t cNsMin, unsigned cTimes, const char *pszOperation)
{
    RTPrintf("tstPrfRT: %-32s %5lld / %5lld / %5lld ns per call (%u calls %lld ns)\n",
             pszOperation, cNsMin, cNs / (uint64_t)cTimes, cNsMax, cTimes, cNs);
}

# define ITERATE(preexpr, expr, postexpr, cIterations) \
    for (i = 0, u64TotalTS = 0, u64MinTS = ~0, u64MaxTS = 0; i < (cIterations); i++) \
    { \
        { preexpr } \
        uint64_t u64StartTS = RTTimeNanoTS(); \
        { expr } \
        uint64_t u64ElapsedTS = RTTimeNanoTS() - u64StartTS; \
        { postexpr } \
        if (u64ElapsedTS > u64MinTS * 32) \
        { \
            i--; \
            continue; \
        } \
        if (u64ElapsedTS < u64MinTS) \
            u64MinTS = u64ElapsedTS; \
        if (u64ElapsedTS > u64MaxTS) \
            u64MaxTS = u64ElapsedTS; \
        u64TotalTS += u64ElapsedTS; \
    }

#endif /* !AMD64 && !X86 */


int main()
{
    uint64_t    u64TotalTS;
    uint64_t    u64MinTS;
    uint64_t    u64MaxTS;
    unsigned    i;

    RTR3InitExeNoArguments(0);
    RTPrintf("tstPrfRT: TESTING...\n");

    /*
     * RTTimeNanoTS, RTTimeProgramNanoTS, RTTimeMilliTS, and RTTimeProgramMilliTS.
     */
    ITERATE(RT_NOTHING, RTTimeNanoTS();, RT_NOTHING, 1000000);
    PrintResult(u64TotalTS, u64MaxTS, u64MinTS, i, "RTTimeNanoTS");

    ITERATE(RT_NOTHING, RTTimeProgramNanoTS();, RT_NOTHING, 1000000);
    PrintResult(u64TotalTS, u64MaxTS, u64MinTS, i, "RTTimeProgramNanoTS");

    ITERATE(RT_NOTHING, RTTimeMilliTS();, RT_NOTHING, 1000000);
    PrintResult(u64TotalTS, u64MaxTS, u64MinTS, i, "RTTimeMilliTS");

    ITERATE(RT_NOTHING, RTTimeProgramMilliTS();, RT_NOTHING, 1000000);
    PrintResult(u64TotalTS, u64MaxTS, u64MinTS, i, "RTTimeProgramMilliTS");

    /*
     * RTTimeNow
     */
    RTTIMESPEC Time;
    ITERATE(RT_NOTHING, RTTimeNow(&Time);, RT_NOTHING, 1000000);
    PrintResult(u64TotalTS, u64MaxTS, u64MinTS, i, "RTTimeNow");

    /*
     * RTLogDefaultInstance()
     */
    ITERATE(RT_NOTHING, RTLogDefaultInstance();, RT_NOTHING, 1000000);
    PrintResult(u64TotalTS, u64MaxTS, u64MinTS, i, "RTLogDefaultInstance");

    /*
     * RTThreadSelf and RTThreadNativeSelf
     */
    ITERATE(RT_NOTHING, RTThreadSelf();, RT_NOTHING, 1000000);
    PrintResult(u64TotalTS, u64MaxTS, u64MinTS, i, "RTThreadSelf");

    ITERATE(RT_NOTHING, RTThreadNativeSelf();, RT_NOTHING, 1000000);
    PrintResult(u64TotalTS, u64MaxTS, u64MinTS, i, "RTThreadNativeSelf");

    RTPrintf("tstPrtRT: DONE\n");
    return 0;
}
