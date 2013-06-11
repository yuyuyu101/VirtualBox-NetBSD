/* $Id: mp-r0drv-nt.cpp $ */
/** @file
 * IPRT - Multiprocessor, Ring-0 Driver, NT.
 */

/*
 * Copyright (C) 2008-2011 Oracle Corporation
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
#include "the-nt-kernel.h"

#include <iprt/mp.h>
#include <iprt/cpuset.h>
#include <iprt/err.h>
#include <iprt/asm.h>
#include "r0drv/mp-r0drv.h"
#include "internal-r0drv-nt.h"


/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/
typedef enum
{
    RT_NT_CPUID_SPECIFIC,
    RT_NT_CPUID_OTHERS,
    RT_NT_CPUID_ALL
} RT_NT_CPUID;


/* test a couple of assumption. */
AssertCompile(MAXIMUM_PROCESSORS <= RTCPUSET_MAX_CPUS);
AssertCompile(NIL_RTCPUID >= MAXIMUM_PROCESSORS);

/** @todo
 * We cannot do other than assume a 1:1 relationship between the
 * affinity mask and the process despite the vagueness/warnings in
 * the docs. If someone knows a better way to get this done, please
 * let bird know.
 */


RTDECL(RTCPUID) RTMpCpuId(void)
{
    /* WDK upgrade warning: PCR->Number changed from BYTE to WORD. */
    return KeGetCurrentProcessorNumber();
}


RTDECL(int) RTMpCpuIdToSetIndex(RTCPUID idCpu)
{
    return idCpu < MAXIMUM_PROCESSORS ? (int)idCpu : -1;
}


RTDECL(RTCPUID) RTMpCpuIdFromSetIndex(int iCpu)
{
    return (unsigned)iCpu < MAXIMUM_PROCESSORS ? iCpu : NIL_RTCPUID;
}


RTDECL(RTCPUID) RTMpGetMaxCpuId(void)
{
    /** @todo use KeQueryMaximumProcessorCount on vista+ */
    return MAXIMUM_PROCESSORS - 1;
}


RTDECL(bool) RTMpIsCpuOnline(RTCPUID idCpu)
{
    if (idCpu >= MAXIMUM_PROCESSORS)
        return false;

#if 0 /* this isn't safe at all IRQLs (great work guys) */
    KAFFINITY Mask = KeQueryActiveProcessors();
    return !!(Mask & RT_BIT_64(idCpu));
#else
    return RTCpuSetIsMember(&g_rtMpNtCpuSet, idCpu);
#endif
}


RTDECL(bool) RTMpIsCpuPossible(RTCPUID idCpu)
{
    /* Cannot easily distinguish between online and offline cpus. */
    /** @todo online/present cpu stuff must be corrected for proper W2K8 support
     *        (KeQueryMaximumProcessorCount). */
    return RTMpIsCpuOnline(idCpu);
}



RTDECL(PRTCPUSET) RTMpGetSet(PRTCPUSET pSet)
{
    /** @todo online/present cpu stuff must be corrected for proper W2K8 support
     *        (KeQueryMaximumProcessorCount). */
    return RTMpGetOnlineSet(pSet);
}


RTDECL(RTCPUID) RTMpGetCount(void)
{
    /** @todo online/present cpu stuff must be corrected for proper W2K8 support
     *        (KeQueryMaximumProcessorCount). */
    return RTMpGetOnlineCount();
}


RTDECL(PRTCPUSET) RTMpGetOnlineSet(PRTCPUSET pSet)
{
#if 0 /* this isn't safe at all IRQLs (great work guys) */
    KAFFINITY Mask = KeQueryActiveProcessors();
    return RTCpuSetFromU64(pSet, Mask);
#else
    *pSet = g_rtMpNtCpuSet;
    return pSet;
#endif
}


RTDECL(RTCPUID) RTMpGetOnlineCount(void)
{
    RTCPUSET Set;
    RTMpGetOnlineSet(&Set);
    return RTCpuSetCount(&Set);
}


#if 0
/* Experiment with checking the undocumented KPRCB structure
 * 'dt nt!_kprcb 0xaddress' shows the layout
 */
typedef struct
{
    LIST_ENTRY     DpcListHead;
    ULONG_PTR      DpcLock;
    volatile ULONG DpcQueueDepth;
    ULONG          DpcQueueCount;
} KDPC_DATA, *PKDPC_DATA;

RTDECL(bool) RTMpIsCpuWorkPending(void)
{
    uint8_t *pkprcb;
    PKDPC_DATA pDpcData;

    _asm {
        mov eax, fs:0x20
        mov pkprcb, eax
    }
    pDpcData = (PKDPC_DATA)(pkprcb + 0x19e0);
    if (pDpcData->DpcQueueDepth)
        return true;

    pDpcData++;
    if (pDpcData->DpcQueueDepth)
        return true;
    return false;
}
#else
RTDECL(bool) RTMpIsCpuWorkPending(void)
{
    /** @todo not implemented */
    return false;
}
#endif


/**
 * Wrapper between the native nt per-cpu callbacks and PFNRTWORKER
 *
 * @param   Dpc                 DPC object
 * @param   DeferredContext     Context argument specified by KeInitializeDpc
 * @param   SystemArgument1     Argument specified by KeInsertQueueDpc
 * @param   SystemArgument2     Argument specified by KeInsertQueueDpc
 */
static VOID rtmpNtDPCWrapper(IN PKDPC Dpc, IN PVOID DeferredContext, IN PVOID SystemArgument1, IN PVOID SystemArgument2)
{
    PRTMPARGS pArgs = (PRTMPARGS)DeferredContext;
    ASMAtomicIncU32(&pArgs->cHits);
    pArgs->pfnWorker(KeGetCurrentProcessorNumber(), pArgs->pvUser1, pArgs->pvUser2);
}


/**
 * Internal worker for the RTMpOn* APIs.
 *
 * @returns IPRT status code.
 * @param   pfnWorker       The callback.
 * @param   pvUser1         User argument 1.
 * @param   pvUser2         User argument 2.
 * @param   enmCpuid        What to do / is idCpu valid.
 * @param   idCpu           Used if enmCpuid RT_NT_CPUID_SPECIFIC, otherwise ignored.
 */
static int rtMpCall(PFNRTMPWORKER pfnWorker, void *pvUser1, void *pvUser2, RT_NT_CPUID enmCpuid, RTCPUID idCpu)
{
    PRTMPARGS pArgs;
    KDPC     *paExecCpuDpcs;

#if 0
    /* KeFlushQueuedDpcs must be run at IRQL PASSIVE_LEVEL according to MSDN, but the
     * driver verifier doesn't complain...
     */
    AssertMsg(KeGetCurrentIrql() == PASSIVE_LEVEL, ("%d != %d (PASSIVE_LEVEL)\n", KeGetCurrentIrql(), PASSIVE_LEVEL));
#endif

#ifdef IPRT_TARGET_NT4
    KAFFINITY Mask;
    /* g_pfnrtNt* are not present on NT anyway. */
    return VERR_NOT_SUPPORTED;
#else
    KAFFINITY Mask = KeQueryActiveProcessors();
#endif

    /* KeFlushQueuedDpcs is not present in Windows 2000; import it dynamically so we can just fail this call. */
    if (!g_pfnrtNtKeFlushQueuedDpcs)
        return VERR_NOT_SUPPORTED;

    pArgs = (PRTMPARGS)ExAllocatePoolWithTag(NonPagedPool, MAXIMUM_PROCESSORS*sizeof(KDPC) + sizeof(RTMPARGS), (ULONG)'RTMp');
    if (!pArgs)
        return VERR_NO_MEMORY;

    pArgs->pfnWorker = pfnWorker;
    pArgs->pvUser1   = pvUser1;
    pArgs->pvUser2   = pvUser2;
    pArgs->idCpu     = NIL_RTCPUID;
    pArgs->cHits     = 0;

    paExecCpuDpcs = (KDPC *)(pArgs + 1);

    if (enmCpuid == RT_NT_CPUID_SPECIFIC)
    {
        KeInitializeDpc(&paExecCpuDpcs[0], rtmpNtDPCWrapper, pArgs);
        KeSetImportanceDpc(&paExecCpuDpcs[0], HighImportance);
        KeSetTargetProcessorDpc(&paExecCpuDpcs[0], (int)idCpu);
    }
    else
    {
        for (unsigned i = 0; i < MAXIMUM_PROCESSORS; i++)
        {
            KeInitializeDpc(&paExecCpuDpcs[i], rtmpNtDPCWrapper, pArgs);
            KeSetImportanceDpc(&paExecCpuDpcs[i], HighImportance);
            KeSetTargetProcessorDpc(&paExecCpuDpcs[i], i);
        }
    }

    /* Raise the IRQL to DISPATCH_LEVEL so we can't be rescheduled to another cpu.
     * KeInsertQueueDpc must also be executed at IRQL >= DISPATCH_LEVEL.
     */
    KIRQL oldIrql;
    KeRaiseIrql(DISPATCH_LEVEL, &oldIrql);

    /*
     * We cannot do other than assume a 1:1 relationship between the
     * affinity mask and the process despite the warnings in the docs.
     * If someone knows a better way to get this done, please let bird know.
     */
    if (enmCpuid == RT_NT_CPUID_SPECIFIC)
    {
        BOOLEAN ret = KeInsertQueueDpc(&paExecCpuDpcs[0], 0, 0);
        Assert(ret);
    }
    else
    {
        unsigned iSelf = KeGetCurrentProcessorNumber();

        for (unsigned i = 0; i < MAXIMUM_PROCESSORS; i++)
        {
            if (    (i != iSelf)
                &&  (Mask & RT_BIT_64(i)))
            {
                BOOLEAN ret = KeInsertQueueDpc(&paExecCpuDpcs[i], 0, 0);
                Assert(ret);
            }
        }
        if (enmCpuid != RT_NT_CPUID_OTHERS)
            pfnWorker(iSelf, pvUser1, pvUser2);
    }

    KeLowerIrql(oldIrql);

    /* Flush all DPCs and wait for completion. (can take long!) */
    /** @todo Consider changing this to an active wait using some atomic inc/dec
     *  stuff (and check for the current cpu above in the specific case). */
    g_pfnrtNtKeFlushQueuedDpcs();

    ExFreePool(pArgs);
    return VINF_SUCCESS;
}


RTDECL(int) RTMpOnAll(PFNRTMPWORKER pfnWorker, void *pvUser1, void *pvUser2)
{
    return rtMpCall(pfnWorker, pvUser1, pvUser2, RT_NT_CPUID_ALL, 0);
}


RTDECL(int) RTMpOnOthers(PFNRTMPWORKER pfnWorker, void *pvUser1, void *pvUser2)
{
    return rtMpCall(pfnWorker, pvUser1, pvUser2, RT_NT_CPUID_OTHERS, 0);
}


RTDECL(int) RTMpOnSpecific(RTCPUID idCpu, PFNRTMPWORKER pfnWorker, void *pvUser1, void *pvUser2)
{
    if (!RTMpIsCpuOnline(idCpu))
        return !RTMpIsCpuPossible(idCpu)
              ? VERR_CPU_NOT_FOUND
              : VERR_CPU_OFFLINE;

    return rtMpCall(pfnWorker, pvUser1, pvUser2, RT_NT_CPUID_SPECIFIC, idCpu);
}

static KDPC aPokeDpcs[MAXIMUM_PROCESSORS] = {0};
static bool fPokeDPCsInitialized = false;

static VOID rtMpNtPokeCpuDummy(IN PKDPC Dpc, IN PVOID DeferredContext, IN PVOID SystemArgument1, IN PVOID SystemArgument2)
{
    NOREF(Dpc);
    NOREF(DeferredContext);
    NOREF(SystemArgument1);
    NOREF(SystemArgument2);
}

#ifndef IPRT_TARGET_NT4

ULONG_PTR rtMpIpiGenericCall(ULONG_PTR Argument)
{
    NOREF(Argument);
    return 0;
}


int rtMpSendIpiVista(RTCPUID idCpu)
{
    g_pfnrtKeIpiGenericCall(rtMpIpiGenericCall, 0);
////    g_pfnrtNtHalRequestIpi(1 << idCpu);
    return VINF_SUCCESS;
}


int rtMpSendIpiWin7(RTCPUID idCpu)
{
    g_pfnrtKeIpiGenericCall(rtMpIpiGenericCall, 0);
////    g_pfnrtNtHalSendSoftwareInterrupt(idCpu, DISPATCH_LEVEL);
    return VINF_SUCCESS;
}

#endif /* IPRT_TARGET_NT4 */


int rtMpSendIpiDummy(RTCPUID idCpu)
{
    return VERR_NOT_IMPLEMENTED;
}


RTDECL(int) RTMpPokeCpu(RTCPUID idCpu)
{
    if (!RTMpIsCpuOnline(idCpu))
        return !RTMpIsCpuPossible(idCpu)
              ? VERR_CPU_NOT_FOUND
              : VERR_CPU_OFFLINE;

    int rc = g_pfnrtSendIpi(idCpu);
    if (rc == VINF_SUCCESS)
        return rc;

    /* Fallback. */
    if (!fPokeDPCsInitialized)
    {
        for (unsigned i = 0; i < RT_ELEMENTS(aPokeDpcs); i++)
        {
            KeInitializeDpc(&aPokeDpcs[i], rtMpNtPokeCpuDummy, NULL);
            KeSetImportanceDpc(&aPokeDpcs[i], HighImportance);
            KeSetTargetProcessorDpc(&aPokeDpcs[i], (int)i);
        }
        fPokeDPCsInitialized = true;
    }

    /* Raise the IRQL to DISPATCH_LEVEL so we can't be rescheduled to another cpu.
     * KeInsertQueueDpc must also be executed at IRQL >= DISPATCH_LEVEL.
     */
    KIRQL oldIrql;
    KeRaiseIrql(DISPATCH_LEVEL, &oldIrql);

    KeSetImportanceDpc(&aPokeDpcs[idCpu], HighImportance);
    KeSetTargetProcessorDpc(&aPokeDpcs[idCpu], (int)idCpu);

    /* Assuming here that high importance DPCs will be delivered immediately; or at least an IPI will be sent immediately.
     * @note: not true on at least Vista & Windows 7
     */
    BOOLEAN bRet = KeInsertQueueDpc(&aPokeDpcs[idCpu], 0, 0);

    KeLowerIrql(oldIrql);
    return (bRet == TRUE) ? VINF_SUCCESS : VERR_ACCESS_DENIED /* already queued */;
}
