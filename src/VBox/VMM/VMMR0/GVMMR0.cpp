/* $Id: GVMMR0.cpp $ */
/** @file
 * GVMM - Global VM Manager.
 */

/*
 * Copyright (C) 2007-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */


/** @page pg_gvmm   GVMM - The Global VM Manager
 *
 * The Global VM Manager lives in ring-0.  Its main function at the moment is
 * to manage a list of all running VMs, keep a ring-0 only structure (GVM) for
 * each of them, and assign them unique identifiers (so GMM can track page
 * owners).  The GVMM also manage some of the host CPU resources, like the
 * periodic preemption timer.
 *
 * The GVMM will create a ring-0 object for each VM when it is registered, this
 * is both for session cleanup purposes and for having a point where it is
 * possible to implement usage polices later (in SUPR0ObjRegister).
 *
 *
 * @section  sec_gvmm_ppt       Periodic Preemption Timer (PPT)
 *
 * On system that sports a high resolution kernel timer API, we use per-cpu
 * timers to generate interrupts that preempts VT-x, AMD-V and raw-mode guest
 * execution.  The timer frequency is calculating by taking the max
 * TMCalcHostTimerFrequency for all VMs running on a CPU for the last ~160 ms
 * (RT_ELEMENTS((PGVMMHOSTCPU)0, Ppt.aHzHistory) *
 * GVMMHOSTCPU_PPT_HIST_INTERVAL_NS).
 *
 * The TMCalcHostTimerFrequency() part of the things gets its takes the max
 * TMTimerSetFrequencyHint() value and adjusts by the current catch-up percent,
 * warp drive percent and some fudge factors.  VMMR0.cpp reports the result via
 * GVMMR0SchedUpdatePeriodicPreemptionTimer() before switching to the VT-x,
 * AMD-V and raw-mode execution environments.
 */


/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#define LOG_GROUP LOG_GROUP_GVMM
#include <VBox/vmm/gvmm.h>
#include <VBox/vmm/gmm.h>
#include "GVMMR0Internal.h"
#include <VBox/vmm/gvm.h>
#include <VBox/vmm/vm.h>
#include <VBox/vmm/vmcpuset.h>
#include <VBox/vmm/vmm.h>
#include <VBox/param.h>
#include <VBox/err.h>

#include <iprt/asm.h>
#include <iprt/asm-amd64-x86.h>
#include <iprt/mem.h>
#include <iprt/semaphore.h>
#include <iprt/time.h>
#include <VBox/log.h>
#include <iprt/thread.h>
#include <iprt/process.h>
#include <iprt/param.h>
#include <iprt/string.h>
#include <iprt/assert.h>
#include <iprt/mem.h>
#include <iprt/memobj.h>
#include <iprt/mp.h>
#include <iprt/cpuset.h>
#include <iprt/spinlock.h>
#include <iprt/timer.h>

#include "dtrace/VBoxVMM.h"


/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
#if defined(RT_OS_LINUX) || defined(DOXYGEN_RUNNING)
/** Define this to enable the periodic preemption timer. */
# define GVMM_SCHED_WITH_PPT
#endif


/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/

/**
 * Global VM handle.
 */
typedef struct GVMHANDLE
{
    /** The index of the next handle in the list (free or used). (0 is nil.) */
    uint16_t volatile   iNext;
    /** Our own index / handle value. */
    uint16_t            iSelf;
    /** The process ID of the handle owner.
     * This is used for access checks. */
    RTPROCESS           ProcId;
    /** The pointer to the ring-0 only (aka global) VM structure. */
    PGVM                pGVM;
    /** The ring-0 mapping of the shared VM instance data. */
    PVM                 pVM;
    /** The virtual machine object. */
    void               *pvObj;
    /** The session this VM is associated with. */
    PSUPDRVSESSION      pSession;
    /** The ring-0 handle of the EMT0 thread.
     * This is used for ownership checks as well as looking up a VM handle by thread
     * at times like assertions. */
    RTNATIVETHREAD      hEMT0;
} GVMHANDLE;
/** Pointer to a global VM handle. */
typedef GVMHANDLE *PGVMHANDLE;

/** Number of GVM handles (including the NIL handle). */
#if HC_ARCH_BITS == 64
# define GVMM_MAX_HANDLES   8192
#else
# define GVMM_MAX_HANDLES   128
#endif

/**
 * Per host CPU GVMM data.
 */
typedef struct GVMMHOSTCPU
{
    /** Magic number (GVMMHOSTCPU_MAGIC). */
    uint32_t volatile   u32Magic;
    /** The CPU ID. */
    RTCPUID             idCpu;
    /** The CPU set index. */
    uint32_t            idxCpuSet;

#ifdef GVMM_SCHED_WITH_PPT
    /** Periodic preemption timer data. */
    struct
    {
        /** The handle to the periodic preemption timer. */
        PRTTIMER            pTimer;
        /** Spinlock protecting the data below. */
        RTSPINLOCK          hSpinlock;
        /** The smalles Hz that we need to care about. (static) */
        uint32_t            uMinHz;
        /** The number of ticks between each historization. */
        uint32_t            cTicksHistoriziationInterval;
        /** The current historization tick (counting up to
         * cTicksHistoriziationInterval and then resetting). */
        uint32_t            iTickHistorization;
        /** The current timer interval.  This is set to 0 when inactive. */
        uint32_t            cNsInterval;
        /** The current timer frequency.  This is set to 0 when inactive. */
        uint32_t            uTimerHz;
        /** The current max frequency reported by the EMTs.
         * This gets historicize and reset by the timer callback.  This is
         * read without holding the spinlock, so needs atomic updating. */
        uint32_t volatile   uDesiredHz;
        /** Whether the timer was started or not. */
        bool volatile       fStarted;
        /** Set if we're starting timer. */
        bool volatile       fStarting;
        /** The index of the next history entry (mod it). */
        uint32_t            iHzHistory;
        /** Historicized uDesiredHz values.  The array wraps around, new entries
         * are added at iHzHistory. This is updated approximately every
         * GVMMHOSTCPU_PPT_HIST_INTERVAL_NS by the timer callback. */
        uint32_t            aHzHistory[8];
        /** Statistics counter for recording the number of interval changes. */
        uint32_t            cChanges;
        /** Statistics counter for recording the number of timer starts. */
        uint32_t            cStarts;
    } Ppt;
#endif /* GVMM_SCHED_WITH_PPT */

} GVMMHOSTCPU;
/** Pointer to the per host CPU GVMM data. */
typedef GVMMHOSTCPU *PGVMMHOSTCPU;
/** The GVMMHOSTCPU::u32Magic value (Petra, Tanya & Rachel Haden). */
#define GVMMHOSTCPU_MAGIC   UINT32_C(0x19711011)
/** The interval on history entry should cover (approximately) give in
 *  nanoseconds. */
#define GVMMHOSTCPU_PPT_HIST_INTERVAL_NS    UINT32_C(20000000)


/**
 * The GVMM instance data.
 */
typedef struct GVMM
{
    /** Eyecatcher / magic. */
    uint32_t            u32Magic;
    /** The index of the head of the free handle chain. (0 is nil.) */
    uint16_t volatile   iFreeHead;
    /** The index of the head of the active handle chain. (0 is nil.) */
    uint16_t volatile   iUsedHead;
    /** The number of VMs. */
    uint16_t volatile   cVMs;
    /** Alignment padding. */
    uint16_t            u16Reserved;
    /** The number of EMTs. */
    uint32_t volatile   cEMTs;
    /** The number of EMTs that have halted in GVMMR0SchedHalt. */
    uint32_t volatile   cHaltedEMTs;
    /** Alignment padding. */
    uint32_t            u32Alignment;
    /** When the next halted or sleeping EMT will wake up.
     * This is set to 0 when it needs recalculating and to UINT64_MAX when
     * there are no halted or sleeping EMTs in the GVMM. */
    uint64_t            uNsNextEmtWakeup;
    /** The lock used to serialize VM creation, destruction and associated events that
     * isn't performance critical. Owners may acquire the list lock. */
    RTSEMFASTMUTEX      CreateDestroyLock;
    /** The lock used to serialize used list updates and accesses.
     * This indirectly includes scheduling since the scheduler will have to walk the
     * used list to examin running VMs. Owners may not acquire any other locks. */
    RTSEMFASTMUTEX      UsedLock;
    /** The handle array.
     * The size of this array defines the maximum number of currently running VMs.
     * The first entry is unused as it represents the NIL handle. */
    GVMHANDLE           aHandles[GVMM_MAX_HANDLES];

    /** @gcfgm{/GVMM/cEMTsMeansCompany, 32-bit, 0, UINT32_MAX, 1}
     * The number of EMTs that means we no longer consider ourselves alone on a
     * CPU/Core.
     */
    uint32_t            cEMTsMeansCompany;
    /** @gcfgm{/GVMM/MinSleepAlone,32-bit, 0, 100000000, 750000, ns}
     * The minimum sleep time for when we're alone, in nano seconds.
     */
    uint32_t            nsMinSleepAlone;
    /** @gcfgm{/GVMM/MinSleepCompany,32-bit,0, 100000000, 15000, ns}
     * The minimum sleep time for when we've got company, in nano seconds.
     */
    uint32_t            nsMinSleepCompany;
    /** @gcfgm{/GVMM/EarlyWakeUp1, 32-bit, 0, 100000000, 25000, ns}
     * The limit for the first round of early wakeups, given in nano seconds.
     */
    uint32_t            nsEarlyWakeUp1;
    /** @gcfgm{/GVMM/EarlyWakeUp2, 32-bit, 0, 100000000, 50000, ns}
     * The limit for the second round of early wakeups, given in nano seconds.
     */
    uint32_t            nsEarlyWakeUp2;

    /** The number of entries in the host CPU array (aHostCpus). */
    uint32_t            cHostCpus;
    /** Per host CPU data (variable length). */
    GVMMHOSTCPU         aHostCpus[1];
} GVMM;
/** Pointer to the GVMM instance data. */
typedef GVMM *PGVMM;

/** The GVMM::u32Magic value (Charlie Haden). */
#define GVMM_MAGIC      UINT32_C(0x19370806)



/*******************************************************************************
*   Global Variables                                                           *
*******************************************************************************/
/** Pointer to the GVMM instance data.
 * (Just my general dislike for global variables.) */
static PGVMM g_pGVMM = NULL;

/** Macro for obtaining and validating the g_pGVMM pointer.
 * On failure it will return from the invoking function with the specified return value.
 *
 * @param   pGVMM   The name of the pGVMM variable.
 * @param   rc      The return value on failure. Use VERR_GVMM_INSTANCE for VBox
 *                  status codes.
 */
#define GVMM_GET_VALID_INSTANCE(pGVMM, rc) \
    do { \
        (pGVMM) = g_pGVMM;\
        AssertPtrReturn((pGVMM), (rc)); \
        AssertMsgReturn((pGVMM)->u32Magic == GVMM_MAGIC, ("%p - %#x\n", (pGVMM), (pGVMM)->u32Magic), (rc)); \
    } while (0)

/** Macro for obtaining and validating the g_pGVMM pointer, void function variant.
 * On failure it will return from the invoking function.
 *
 * @param   pGVMM   The name of the pGVMM variable.
 */
#define GVMM_GET_VALID_INSTANCE_VOID(pGVMM) \
    do { \
        (pGVMM) = g_pGVMM;\
        AssertPtrReturnVoid((pGVMM)); \
        AssertMsgReturnVoid((pGVMM)->u32Magic == GVMM_MAGIC, ("%p - %#x\n", (pGVMM), (pGVMM)->u32Magic)); \
    } while (0)


/*******************************************************************************
*   Internal Functions                                                         *
*******************************************************************************/
static void gvmmR0InitPerVMData(PGVM pGVM);
static DECLCALLBACK(void) gvmmR0HandleObjDestructor(void *pvObj, void *pvGVMM, void *pvHandle);
static int gvmmR0ByVM(PVM pVM, PGVM *ppGVM, PGVMM *ppGVMM, bool fTakeUsedLock);
static int gvmmR0ByVMAndEMT(PVM pVM, VMCPUID idCpu, PGVM *ppGVM, PGVMM *ppGVMM);
#ifdef GVMM_SCHED_WITH_PPT
static DECLCALLBACK(void) gvmmR0SchedPeriodicPreemptionTimerCallback(PRTTIMER pTimer, void *pvUser, uint64_t iTick);
#endif


/**
 * Initializes the GVMM.
 *
 * This is called while owning the loader semaphore (see supdrvIOCtl_LdrLoad()).
 *
 * @returns VBox status code.
 */
GVMMR0DECL(int) GVMMR0Init(void)
{
    LogFlow(("GVMMR0Init:\n"));

    /*
     * Allocate and initialize the instance data.
     */
    uint32_t cHostCpus = RTMpGetArraySize();
    AssertMsgReturn(cHostCpus > 0 && cHostCpus < _64K, ("%d", (int)cHostCpus), VERR_GVMM_HOST_CPU_RANGE);

    PGVMM pGVMM = (PGVMM)RTMemAllocZ(RT_UOFFSETOF(GVMM, aHostCpus[cHostCpus]));
    if (!pGVMM)
        return VERR_NO_MEMORY;
    int rc = RTSemFastMutexCreate(&pGVMM->CreateDestroyLock);
    if (RT_SUCCESS(rc))
    {
        rc = RTSemFastMutexCreate(&pGVMM->UsedLock);
        if (RT_SUCCESS(rc))
        {
            pGVMM->u32Magic = GVMM_MAGIC;
            pGVMM->iUsedHead = 0;
            pGVMM->iFreeHead = 1;

            /* the nil handle */
            pGVMM->aHandles[0].iSelf = 0;
            pGVMM->aHandles[0].iNext = 0;

            /* the tail */
            unsigned i = RT_ELEMENTS(pGVMM->aHandles) - 1;
            pGVMM->aHandles[i].iSelf = i;
            pGVMM->aHandles[i].iNext = 0; /* nil */

            /* the rest */
            while (i-- > 1)
            {
                pGVMM->aHandles[i].iSelf = i;
                pGVMM->aHandles[i].iNext = i + 1;
            }

            /* The default configuration values. */
            uint32_t cNsResolution = RTSemEventMultiGetResolution();
            pGVMM->cEMTsMeansCompany     = 1;                           /** @todo should be adjusted to relative to the cpu count or something... */
            if (cNsResolution >= 5*RT_NS_100US)
            {
                pGVMM->nsMinSleepAlone   = 750000 /* ns (0.750 ms) */;  /** @todo this should be adjusted to be 75% (or something) of the scheduler granularity... */
                pGVMM->nsMinSleepCompany =  15000 /* ns (0.015 ms) */;
                pGVMM->nsEarlyWakeUp1    =  25000 /* ns (0.025 ms) */;
                pGVMM->nsEarlyWakeUp2    =  50000 /* ns (0.050 ms) */;
            }
            else if (cNsResolution > RT_NS_100US)
            {
                pGVMM->nsMinSleepAlone   = cNsResolution / 2;
                pGVMM->nsMinSleepCompany = cNsResolution / 4;
                pGVMM->nsEarlyWakeUp1    = 0;
                pGVMM->nsEarlyWakeUp2    = 0;
            }
            else
            {
                pGVMM->nsMinSleepAlone   = 2000;
                pGVMM->nsMinSleepCompany = 2000;
                pGVMM->nsEarlyWakeUp1    = 0;
                pGVMM->nsEarlyWakeUp2    = 0;
            }

            /* The host CPU data. */
            pGVMM->cHostCpus = cHostCpus;
            uint32_t    iCpu = cHostCpus;
            RTCPUSET    PossibleSet;
            RTMpGetSet(&PossibleSet);
            while (iCpu-- > 0)
            {
                pGVMM->aHostCpus[iCpu].idxCpuSet        = iCpu;
#ifdef GVMM_SCHED_WITH_PPT
                pGVMM->aHostCpus[iCpu].Ppt.pTimer       = NULL;
                pGVMM->aHostCpus[iCpu].Ppt.hSpinlock    = NIL_RTSPINLOCK;
                pGVMM->aHostCpus[iCpu].Ppt.uMinHz       = 5; /** @todo Add some API which figures this one out. (not *that* important) */
                pGVMM->aHostCpus[iCpu].Ppt.cTicksHistoriziationInterval = 1;
                //pGVMM->aHostCpus[iCpu].Ppt.iTickHistorization           = 0;
                //pGVMM->aHostCpus[iCpu].Ppt.cNsInterval  = 0;
                //pGVMM->aHostCpus[iCpu].Ppt.uTimerHz     = 0;
                //pGVMM->aHostCpus[iCpu].Ppt.uDesiredHz   = 0;
                //pGVMM->aHostCpus[iCpu].Ppt.fStarted     = false;
                //pGVMM->aHostCpus[iCpu].Ppt.fStarting    = false;
                //pGVMM->aHostCpus[iCpu].Ppt.iHzHistory   = 0;
                //pGVMM->aHostCpus[iCpu].Ppt.aHzHistory   = {0};
#endif

                if (RTCpuSetIsMember(&PossibleSet, iCpu))
                {
                    pGVMM->aHostCpus[iCpu].idCpu        = RTMpCpuIdFromSetIndex(iCpu);
                    pGVMM->aHostCpus[iCpu].u32Magic     = GVMMHOSTCPU_MAGIC;

#ifdef GVMM_SCHED_WITH_PPT
                    rc = RTTimerCreateEx(&pGVMM->aHostCpus[iCpu].Ppt.pTimer,
                                         50*1000*1000 /* whatever */,
                                         RTTIMER_FLAGS_CPU(iCpu) | RTTIMER_FLAGS_HIGH_RES,
                                         gvmmR0SchedPeriodicPreemptionTimerCallback,
                                         &pGVMM->aHostCpus[iCpu]);
                    if (RT_SUCCESS(rc))
                        rc = RTSpinlockCreate(&pGVMM->aHostCpus[iCpu].Ppt.hSpinlock, RTSPINLOCK_FLAGS_INTERRUPT_SAFE, "GVMM/CPU");
                    if (RT_FAILURE(rc))
                    {
                        while (iCpu < cHostCpus)
                        {
                            RTTimerDestroy(pGVMM->aHostCpus[iCpu].Ppt.pTimer);
                            RTSpinlockDestroy(pGVMM->aHostCpus[iCpu].Ppt.hSpinlock);
                            pGVMM->aHostCpus[iCpu].Ppt.hSpinlock = NIL_RTSPINLOCK;
                            iCpu++;
                        }
                        break;
                    }
#endif
                }
                else
                {
                    pGVMM->aHostCpus[iCpu].idCpu        = NIL_RTCPUID;
                    pGVMM->aHostCpus[iCpu].u32Magic     = 0;
                }
            }
            if (RT_SUCCESS(rc))
            {
                g_pGVMM = pGVMM;
                LogFlow(("GVMMR0Init: pGVMM=%p cHostCpus=%u\n", pGVMM, cHostCpus));
                return VINF_SUCCESS;
            }

            /* bail out. */
            RTSemFastMutexDestroy(pGVMM->UsedLock);
            pGVMM->UsedLock = NIL_RTSEMFASTMUTEX;
        }
        RTSemFastMutexDestroy(pGVMM->CreateDestroyLock);
        pGVMM->CreateDestroyLock = NIL_RTSEMFASTMUTEX;
    }

    RTMemFree(pGVMM);
    return rc;
}


/**
 * Terminates the GVM.
 *
 * This is called while owning the loader semaphore (see supdrvLdrFree()).
 * And unless something is wrong, there should be absolutely no VMs
 * registered at this point.
 */
GVMMR0DECL(void) GVMMR0Term(void)
{
    LogFlow(("GVMMR0Term:\n"));

    PGVMM pGVMM = g_pGVMM;
    g_pGVMM = NULL;
    if (RT_UNLIKELY(!VALID_PTR(pGVMM)))
    {
        SUPR0Printf("GVMMR0Term: pGVMM=%p\n", pGVMM);
        return;
    }

    /*
     * First of all, stop all active timers.
     */
    uint32_t cActiveTimers = 0;
    uint32_t iCpu = pGVMM->cHostCpus;
    while (iCpu-- > 0)
    {
        ASMAtomicWriteU32(&pGVMM->aHostCpus[iCpu].u32Magic, ~GVMMHOSTCPU_MAGIC);
#ifdef GVMM_SCHED_WITH_PPT
        if (    pGVMM->aHostCpus[iCpu].Ppt.pTimer != NULL
            &&  RT_SUCCESS(RTTimerStop(pGVMM->aHostCpus[iCpu].Ppt.pTimer)))
            cActiveTimers++;
#endif
    }
    if (cActiveTimers)
        RTThreadSleep(1); /* fudge */

    /*
     * Invalidate the and free resources.
     */
    pGVMM->u32Magic = ~GVMM_MAGIC;
    RTSemFastMutexDestroy(pGVMM->UsedLock);
    pGVMM->UsedLock = NIL_RTSEMFASTMUTEX;
    RTSemFastMutexDestroy(pGVMM->CreateDestroyLock);
    pGVMM->CreateDestroyLock = NIL_RTSEMFASTMUTEX;

    pGVMM->iFreeHead = 0;
    if (pGVMM->iUsedHead)
    {
        SUPR0Printf("GVMMR0Term: iUsedHead=%#x! (cVMs=%#x cEMTs=%#x)\n", pGVMM->iUsedHead, pGVMM->cVMs, pGVMM->cEMTs);
        pGVMM->iUsedHead = 0;
    }

#ifdef GVMM_SCHED_WITH_PPT
    iCpu = pGVMM->cHostCpus;
    while (iCpu-- > 0)
    {
        RTTimerDestroy(pGVMM->aHostCpus[iCpu].Ppt.pTimer);
        pGVMM->aHostCpus[iCpu].Ppt.pTimer = NULL;
        RTSpinlockDestroy(pGVMM->aHostCpus[iCpu].Ppt.hSpinlock);
        pGVMM->aHostCpus[iCpu].Ppt.hSpinlock = NIL_RTSPINLOCK;
    }
#endif

    RTMemFree(pGVMM);
}


/**
 * A quick hack for setting global config values.
 *
 * @returns VBox status code.
 *
 * @param   pSession    The session handle. Used for authentication.
 * @param   pszName     The variable name.
 * @param   u64Value    The new value.
 */
GVMMR0DECL(int) GVMMR0SetConfig(PSUPDRVSESSION pSession, const char *pszName, uint64_t u64Value)
{
    /*
     * Validate input.
     */
    PGVMM pGVMM;
    GVMM_GET_VALID_INSTANCE(pGVMM, VERR_GVMM_INSTANCE);
    AssertPtrReturn(pSession, VERR_INVALID_HANDLE);
    AssertPtrReturn(pszName, VERR_INVALID_POINTER);

    /*
     * String switch time!
     */
    if (strncmp(pszName, "/GVMM/", sizeof("/GVMM/") - 1))
        return VERR_CFGM_VALUE_NOT_FOUND; /* borrow status codes from CFGM... */
    int rc = VINF_SUCCESS;
    pszName += sizeof("/GVMM/") - 1;
    if (!strcmp(pszName, "cEMTsMeansCompany"))
    {
        if (u64Value <= UINT32_MAX)
            pGVMM->cEMTsMeansCompany = u64Value;
        else
            rc = VERR_OUT_OF_RANGE;
    }
    else if (!strcmp(pszName, "MinSleepAlone"))
    {
        if (u64Value <= RT_NS_100MS)
            pGVMM->nsMinSleepAlone = u64Value;
        else
            rc = VERR_OUT_OF_RANGE;
    }
    else if (!strcmp(pszName, "MinSleepCompany"))
    {
        if (u64Value <= RT_NS_100MS)
            pGVMM->nsMinSleepCompany = u64Value;
        else
            rc = VERR_OUT_OF_RANGE;
    }
    else if (!strcmp(pszName, "EarlyWakeUp1"))
    {
        if (u64Value <= RT_NS_100MS)
            pGVMM->nsEarlyWakeUp1 = u64Value;
        else
            rc = VERR_OUT_OF_RANGE;
    }
    else if (!strcmp(pszName, "EarlyWakeUp2"))
    {
        if (u64Value <= RT_NS_100MS)
            pGVMM->nsEarlyWakeUp2 = u64Value;
        else
            rc = VERR_OUT_OF_RANGE;
    }
    else
        rc = VERR_CFGM_VALUE_NOT_FOUND;
    return rc;
}


/**
 * A quick hack for getting global config values.
 *
 * @returns VBox status code.
 *
 * @param   pSession    The session handle. Used for authentication.
 * @param   pszName     The variable name.
 * @param   u64Value    The new value.
 */
GVMMR0DECL(int) GVMMR0QueryConfig(PSUPDRVSESSION pSession, const char *pszName, uint64_t *pu64Value)
{
    /*
     * Validate input.
     */
    PGVMM pGVMM;
    GVMM_GET_VALID_INSTANCE(pGVMM, VERR_GVMM_INSTANCE);
    AssertPtrReturn(pSession, VERR_INVALID_HANDLE);
    AssertPtrReturn(pszName, VERR_INVALID_POINTER);
    AssertPtrReturn(pu64Value, VERR_INVALID_POINTER);

    /*
     * String switch time!
     */
    if (strncmp(pszName, "/GVMM/", sizeof("/GVMM/") - 1))
        return VERR_CFGM_VALUE_NOT_FOUND; /* borrow status codes from CFGM... */
    int rc = VINF_SUCCESS;
    pszName += sizeof("/GVMM/") - 1;
    if (!strcmp(pszName, "cEMTsMeansCompany"))
        *pu64Value = pGVMM->cEMTsMeansCompany;
    else if (!strcmp(pszName, "MinSleepAlone"))
        *pu64Value = pGVMM->nsMinSleepAlone;
    else if (!strcmp(pszName, "MinSleepCompany"))
        *pu64Value = pGVMM->nsMinSleepCompany;
    else if (!strcmp(pszName, "EarlyWakeUp1"))
        *pu64Value = pGVMM->nsEarlyWakeUp1;
    else if (!strcmp(pszName, "EarlyWakeUp2"))
        *pu64Value = pGVMM->nsEarlyWakeUp2;
    else
        rc = VERR_CFGM_VALUE_NOT_FOUND;
    return rc;
}


/**
 * Try acquire the 'used' lock.
 *
 * @returns IPRT status code, see RTSemFastMutexRequest.
 * @param   pGVMM   The GVMM instance data.
 */
DECLINLINE(int) gvmmR0UsedLock(PGVMM pGVMM)
{
    LogFlow(("++gvmmR0UsedLock(%p)\n", pGVMM));
    int rc = RTSemFastMutexRequest(pGVMM->UsedLock);
    LogFlow(("gvmmR0UsedLock(%p)->%Rrc\n", pGVMM, rc));
    return rc;
}


/**
 * Release the 'used' lock.
 *
 * @returns IPRT status code, see RTSemFastMutexRelease.
 * @param   pGVMM   The GVMM instance data.
 */
DECLINLINE(int) gvmmR0UsedUnlock(PGVMM pGVMM)
{
    LogFlow(("--gvmmR0UsedUnlock(%p)\n", pGVMM));
    int rc = RTSemFastMutexRelease(pGVMM->UsedLock);
    AssertRC(rc);
    return rc;
}


/**
 * Try acquire the 'create & destroy' lock.
 *
 * @returns IPRT status code, see RTSemFastMutexRequest.
 * @param   pGVMM   The GVMM instance data.
 */
DECLINLINE(int) gvmmR0CreateDestroyLock(PGVMM pGVMM)
{
    LogFlow(("++gvmmR0CreateDestroyLock(%p)\n", pGVMM));
    int rc = RTSemFastMutexRequest(pGVMM->CreateDestroyLock);
    LogFlow(("gvmmR0CreateDestroyLock(%p)->%Rrc\n", pGVMM, rc));
    return rc;
}


/**
 * Release the 'create & destroy' lock.
 *
 * @returns IPRT status code, see RTSemFastMutexRequest.
 * @param   pGVMM   The GVMM instance data.
 */
DECLINLINE(int) gvmmR0CreateDestroyUnlock(PGVMM pGVMM)
{
    LogFlow(("--gvmmR0CreateDestroyUnlock(%p)\n", pGVMM));
    int rc = RTSemFastMutexRelease(pGVMM->CreateDestroyLock);
    AssertRC(rc);
    return rc;
}


/**
 * Request wrapper for the GVMMR0CreateVM API.
 *
 * @returns VBox status code.
 * @param   pReq        The request buffer.
 */
GVMMR0DECL(int) GVMMR0CreateVMReq(PGVMMCREATEVMREQ pReq)
{
    /*
     * Validate the request.
     */
    if (!VALID_PTR(pReq))
        return VERR_INVALID_POINTER;
    if (pReq->Hdr.cbReq != sizeof(*pReq))
        return VERR_INVALID_PARAMETER;
    if (!VALID_PTR(pReq->pSession))
        return VERR_INVALID_POINTER;

    /*
     * Execute it.
     */
    PVM pVM;
    pReq->pVMR0 = NULL;
    pReq->pVMR3 = NIL_RTR3PTR;
    int rc = GVMMR0CreateVM(pReq->pSession, pReq->cCpus, &pVM);
    if (RT_SUCCESS(rc))
    {
        pReq->pVMR0 = pVM;
        pReq->pVMR3 = pVM->pVMR3;
    }
    return rc;
}


/**
 * Allocates the VM structure and registers it with GVM.
 *
 * The caller will become the VM owner and there by the EMT.
 *
 * @returns VBox status code.
 * @param   pSession    The support driver session.
 * @param   cCpus       Number of virtual CPUs for the new VM.
 * @param   ppVM        Where to store the pointer to the VM structure.
 *
 * @thread  EMT.
 */
GVMMR0DECL(int) GVMMR0CreateVM(PSUPDRVSESSION pSession, uint32_t cCpus, PVM *ppVM)
{
    LogFlow(("GVMMR0CreateVM: pSession=%p\n", pSession));
    PGVMM pGVMM;
    GVMM_GET_VALID_INSTANCE(pGVMM, VERR_GVMM_INSTANCE);

    AssertPtrReturn(ppVM, VERR_INVALID_POINTER);
    *ppVM = NULL;

    if (    cCpus == 0
        ||  cCpus > VMM_MAX_CPU_COUNT)
        return VERR_INVALID_PARAMETER;

    RTNATIVETHREAD hEMT0 = RTThreadNativeSelf();
    AssertReturn(hEMT0 != NIL_RTNATIVETHREAD, VERR_GVMM_BROKEN_IPRT);
    RTPROCESS      ProcId = RTProcSelf();
    AssertReturn(ProcId != NIL_RTPROCESS, VERR_GVMM_BROKEN_IPRT);

    /*
     * The whole allocation process is protected by the lock.
     */
    int rc = gvmmR0CreateDestroyLock(pGVMM);
    AssertRCReturn(rc, rc);

    /*
     * Allocate a handle first so we don't waste resources unnecessarily.
     */
    uint16_t iHandle = pGVMM->iFreeHead;
    if (iHandle)
    {
        PGVMHANDLE pHandle = &pGVMM->aHandles[iHandle];

        /* consistency checks, a bit paranoid as always. */
        if (    !pHandle->pVM
            &&  !pHandle->pGVM
            &&  !pHandle->pvObj
            &&  pHandle->iSelf == iHandle)
        {
            pHandle->pvObj = SUPR0ObjRegister(pSession, SUPDRVOBJTYPE_VM, gvmmR0HandleObjDestructor, pGVMM, pHandle);
            if (pHandle->pvObj)
            {
                /*
                 * Move the handle from the free to used list and perform permission checks.
                 */
                rc = gvmmR0UsedLock(pGVMM);
                AssertRC(rc);

                pGVMM->iFreeHead = pHandle->iNext;
                pHandle->iNext = pGVMM->iUsedHead;
                pGVMM->iUsedHead = iHandle;
                pGVMM->cVMs++;

                pHandle->pVM      = NULL;
                pHandle->pGVM     = NULL;
                pHandle->pSession = pSession;
                pHandle->hEMT0    = NIL_RTNATIVETHREAD;
                pHandle->ProcId   = NIL_RTPROCESS;

                gvmmR0UsedUnlock(pGVMM);

                rc = SUPR0ObjVerifyAccess(pHandle->pvObj, pSession, NULL);
                if (RT_SUCCESS(rc))
                {
                    /*
                     * Allocate the global VM structure (GVM) and initialize it.
                     */
                    PGVM pGVM = (PGVM)RTMemAllocZ(RT_UOFFSETOF(GVM, aCpus[cCpus]));
                    if (pGVM)
                    {
                        pGVM->u32Magic  = GVM_MAGIC;
                        pGVM->hSelf     = iHandle;
                        pGVM->pVM       = NULL;
                        pGVM->cCpus     = cCpus;

                        gvmmR0InitPerVMData(pGVM);
                        GMMR0InitPerVMData(pGVM);

                        /*
                         * Allocate the shared VM structure and associated page array.
                         */
                        const uint32_t  cbVM   = RT_UOFFSETOF(VM, aCpus[cCpus]);
                        const uint32_t  cPages = RT_ALIGN_32(cbVM, PAGE_SIZE) >> PAGE_SHIFT;
                        rc = RTR0MemObjAllocLow(&pGVM->gvmm.s.VMMemObj, cPages << PAGE_SHIFT, false /* fExecutable */);
                        if (RT_SUCCESS(rc))
                        {
                            PVM pVM = (PVM)RTR0MemObjAddress(pGVM->gvmm.s.VMMemObj); AssertPtr(pVM);
                            memset(pVM, 0, cPages << PAGE_SHIFT);
                            pVM->enmVMState       = VMSTATE_CREATING;
                            pVM->pVMR0            = pVM;
                            pVM->pSession         = pSession;
                            pVM->hSelf            = iHandle;
                            pVM->cbSelf           = cbVM;
                            pVM->cCpus            = cCpus;
                            pVM->uCpuExecutionCap = 100; /* default is no cap. */
                            pVM->offVMCPU         = RT_UOFFSETOF(VM, aCpus);
                            AssertCompileMemberAlignment(VM, cpum, 64);
                            AssertCompileMemberAlignment(VM, tm, 64);
                            AssertCompileMemberAlignment(VM, aCpus, PAGE_SIZE);

                            rc = RTR0MemObjAllocPage(&pGVM->gvmm.s.VMPagesMemObj, cPages * sizeof(SUPPAGE), false /* fExecutable */);
                            if (RT_SUCCESS(rc))
                            {
                                PSUPPAGE paPages = (PSUPPAGE)RTR0MemObjAddress(pGVM->gvmm.s.VMPagesMemObj); AssertPtr(paPages);
                                for (uint32_t iPage = 0; iPage < cPages; iPage++)
                                {
                                    paPages[iPage].uReserved = 0;
                                    paPages[iPage].Phys = RTR0MemObjGetPagePhysAddr(pGVM->gvmm.s.VMMemObj, iPage);
                                    Assert(paPages[iPage].Phys != NIL_RTHCPHYS);
                                }

                                /*
                                 * Map them into ring-3.
                                 */
                                rc = RTR0MemObjMapUser(&pGVM->gvmm.s.VMMapObj, pGVM->gvmm.s.VMMemObj, (RTR3PTR)-1, 0,
                                                       RTMEM_PROT_READ | RTMEM_PROT_WRITE, NIL_RTR0PROCESS);
                                if (RT_SUCCESS(rc))
                                {
                                    pVM->pVMR3 = RTR0MemObjAddressR3(pGVM->gvmm.s.VMMapObj);
                                    AssertPtr((void *)pVM->pVMR3);

                                    /* Initialize all the VM pointers. */
                                    for (uint32_t i = 0; i < cCpus; i++)
                                    {
                                        pVM->aCpus[i].pVMR0           = pVM;
                                        pVM->aCpus[i].pVMR3           = pVM->pVMR3;
                                        pVM->aCpus[i].idHostCpu       = NIL_RTCPUID;
                                        pVM->aCpus[i].hNativeThreadR0 = NIL_RTNATIVETHREAD;
                                    }

                                    rc = RTR0MemObjMapUser(&pGVM->gvmm.s.VMPagesMapObj, pGVM->gvmm.s.VMPagesMemObj, (RTR3PTR)-1, 0,
                                                           RTMEM_PROT_READ | RTMEM_PROT_WRITE, NIL_RTR0PROCESS);
                                    if (RT_SUCCESS(rc))
                                    {
                                        pVM->paVMPagesR3 = RTR0MemObjAddressR3(pGVM->gvmm.s.VMPagesMapObj);
                                        AssertPtr((void *)pVM->paVMPagesR3);

                                        /* complete the handle - take the UsedLock sem just to be careful. */
                                        rc = gvmmR0UsedLock(pGVMM);
                                        AssertRC(rc);

                                        pHandle->pVM                  = pVM;
                                        pHandle->pGVM                 = pGVM;
                                        pHandle->hEMT0                = hEMT0;
                                        pHandle->ProcId               = ProcId;
                                        pGVM->pVM                     = pVM;
                                        pGVM->aCpus[0].hEMT           = hEMT0;
                                        pVM->aCpus[0].hNativeThreadR0 = hEMT0;
                                        pGVMM->cEMTs += cCpus;

                                        VBOXVMM_R0_GVMM_VM_CREATED(pGVM, pVM, ProcId, (void *)hEMT0, cCpus);

                                        gvmmR0UsedUnlock(pGVMM);
                                        gvmmR0CreateDestroyUnlock(pGVMM);

                                        *ppVM = pVM;
                                        Log(("GVMMR0CreateVM: pVM=%p pVMR3=%p pGVM=%p hGVM=%d\n", pVM, pVM->pVMR3, pGVM, iHandle));
                                        return VINF_SUCCESS;
                                    }

                                    RTR0MemObjFree(pGVM->gvmm.s.VMMapObj, false /* fFreeMappings */);
                                    pGVM->gvmm.s.VMMapObj = NIL_RTR0MEMOBJ;
                                }
                                RTR0MemObjFree(pGVM->gvmm.s.VMPagesMemObj, false /* fFreeMappings */);
                                pGVM->gvmm.s.VMPagesMemObj = NIL_RTR0MEMOBJ;
                            }
                            RTR0MemObjFree(pGVM->gvmm.s.VMMemObj, false /* fFreeMappings */);
                            pGVM->gvmm.s.VMMemObj = NIL_RTR0MEMOBJ;
                        }
                    }
                }
                /* else: The user wasn't permitted to create this VM. */

                /*
                 * The handle will be freed by gvmmR0HandleObjDestructor as we release the
                 * object reference here. A little extra mess because of non-recursive lock.
                 */
                void *pvObj = pHandle->pvObj;
                pHandle->pvObj = NULL;
                gvmmR0CreateDestroyUnlock(pGVMM);

                SUPR0ObjRelease(pvObj, pSession);

                SUPR0Printf("GVMMR0CreateVM: failed, rc=%d\n", rc);
                return rc;
            }

            rc = VERR_NO_MEMORY;
        }
        else
            rc = VERR_GVMM_IPE_1;
    }
    else
        rc = VERR_GVM_TOO_MANY_VMS;

    gvmmR0CreateDestroyUnlock(pGVMM);
    return rc;
}


/**
 * Initializes the per VM data belonging to GVMM.
 *
 * @param   pGVM        Pointer to the global VM structure.
 */
static void gvmmR0InitPerVMData(PGVM pGVM)
{
    AssertCompile(RT_SIZEOFMEMB(GVM,gvmm.s) <= RT_SIZEOFMEMB(GVM,gvmm.padding));
    AssertCompile(RT_SIZEOFMEMB(GVMCPU,gvmm.s) <= RT_SIZEOFMEMB(GVMCPU,gvmm.padding));
    pGVM->gvmm.s.VMMemObj = NIL_RTR0MEMOBJ;
    pGVM->gvmm.s.VMMapObj = NIL_RTR0MEMOBJ;
    pGVM->gvmm.s.VMPagesMemObj = NIL_RTR0MEMOBJ;
    pGVM->gvmm.s.VMPagesMapObj = NIL_RTR0MEMOBJ;
    pGVM->gvmm.s.fDoneVMMR0Init = false;
    pGVM->gvmm.s.fDoneVMMR0Term = false;

    for (VMCPUID i = 0; i < pGVM->cCpus; i++)
    {
        pGVM->aCpus[i].gvmm.s.HaltEventMulti = NIL_RTSEMEVENTMULTI;
        pGVM->aCpus[i].hEMT                  = NIL_RTNATIVETHREAD;
    }
}


/**
 * Does the VM initialization.
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 */
GVMMR0DECL(int) GVMMR0InitVM(PVM pVM)
{
    LogFlow(("GVMMR0InitVM: pVM=%p\n", pVM));

    /*
     * Validate the VM structure, state and handle.
     */
    PGVM pGVM;
    PGVMM pGVMM;
    int rc = gvmmR0ByVMAndEMT(pVM, 0 /* idCpu */, &pGVM, &pGVMM);
    if (RT_SUCCESS(rc))
    {
        if (   !pGVM->gvmm.s.fDoneVMMR0Init
            && pGVM->aCpus[0].gvmm.s.HaltEventMulti == NIL_RTSEMEVENTMULTI)
        {
            for (VMCPUID i = 0; i < pGVM->cCpus; i++)
            {
                rc = RTSemEventMultiCreate(&pGVM->aCpus[i].gvmm.s.HaltEventMulti);
                if (RT_FAILURE(rc))
                {
                    pGVM->aCpus[i].gvmm.s.HaltEventMulti = NIL_RTSEMEVENTMULTI;
                    break;
                }
            }
        }
        else
            rc = VERR_WRONG_ORDER;
    }

    LogFlow(("GVMMR0InitVM: returns %Rrc\n", rc));
    return rc;
}


/**
 * Indicates that we're done with the ring-0 initialization
 * of the VM.
 *
 * @param   pVM         Pointer to the VM.
 * @thread  EMT(0)
 */
GVMMR0DECL(void) GVMMR0DoneInitVM(PVM pVM)
{
    /* Validate the VM structure, state and handle. */
    PGVM pGVM;
    PGVMM pGVMM;
    int rc = gvmmR0ByVMAndEMT(pVM, 0 /* idCpu */, &pGVM, &pGVMM);
    AssertRCReturnVoid(rc);

    /* Set the indicator. */
    pGVM->gvmm.s.fDoneVMMR0Init = true;
}


/**
 * Indicates that we're doing the ring-0 termination of the VM.
 *
 * @returns true if termination hasn't been done already, false if it has.
 * @param   pVM         Pointer to the VM.
 * @param   pGVM        Pointer to the global VM structure. Optional.
 * @thread  EMT(0)
 */
GVMMR0DECL(bool) GVMMR0DoingTermVM(PVM pVM, PGVM pGVM)
{
    /* Validate the VM structure, state and handle. */
    AssertPtrNullReturn(pGVM, false);
    AssertReturn(!pGVM || pGVM->u32Magic == GVM_MAGIC, false);
    if (!pGVM)
    {
        PGVMM pGVMM;
        int rc = gvmmR0ByVMAndEMT(pVM, 0 /* idCpu */, &pGVM, &pGVMM);
        AssertRCReturn(rc, false);
    }

    /* Set the indicator. */
    if (pGVM->gvmm.s.fDoneVMMR0Term)
        return false;
    pGVM->gvmm.s.fDoneVMMR0Term = true;
    return true;
}


/**
 * Destroys the VM, freeing all associated resources (the ring-0 ones anyway).
 *
 * This is call from the vmR3DestroyFinalBit and from a error path in VMR3Create,
 * and the caller is not the EMT thread, unfortunately. For security reasons, it
 * would've been nice if the caller was actually the EMT thread or that we somehow
 * could've associated the calling thread with the VM up front.
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 *
 * @thread  EMT(0) if it's associated with the VM, otherwise any thread.
 */
GVMMR0DECL(int) GVMMR0DestroyVM(PVM pVM)
{
    LogFlow(("GVMMR0DestroyVM: pVM=%p\n", pVM));
    PGVMM pGVMM;
    GVMM_GET_VALID_INSTANCE(pGVMM, VERR_GVMM_INSTANCE);


    /*
     * Validate the VM structure, state and caller.
     */
    AssertPtrReturn(pVM, VERR_INVALID_POINTER);
    AssertReturn(!((uintptr_t)pVM & PAGE_OFFSET_MASK), VERR_INVALID_POINTER);
    AssertMsgReturn(pVM->enmVMState >= VMSTATE_CREATING && pVM->enmVMState <= VMSTATE_TERMINATED, ("%d\n", pVM->enmVMState), VERR_WRONG_ORDER);

    uint32_t hGVM = pVM->hSelf;
    AssertReturn(hGVM != NIL_GVM_HANDLE, VERR_INVALID_HANDLE);
    AssertReturn(hGVM < RT_ELEMENTS(pGVMM->aHandles), VERR_INVALID_HANDLE);

    PGVMHANDLE pHandle = &pGVMM->aHandles[hGVM];
    AssertReturn(pHandle->pVM == pVM, VERR_NOT_OWNER);

    RTPROCESS      ProcId = RTProcSelf();
    RTNATIVETHREAD hSelf = RTThreadNativeSelf();
    AssertReturn(   (   pHandle->hEMT0  == hSelf
                     && pHandle->ProcId == ProcId)
                 || pHandle->hEMT0 == NIL_RTNATIVETHREAD, VERR_NOT_OWNER);

    /*
     * Lookup the handle and destroy the object.
     * Since the lock isn't recursive and we'll have to leave it before dereferencing the
     * object, we take some precautions against racing callers just in case...
     */
    int rc = gvmmR0CreateDestroyLock(pGVMM);
    AssertRC(rc);

    /* be careful here because we might theoretically be racing someone else cleaning up. */
    if (    pHandle->pVM == pVM
        &&  (   (   pHandle->hEMT0  == hSelf
                 && pHandle->ProcId == ProcId)
             || pHandle->hEMT0 == NIL_RTNATIVETHREAD)
        &&  VALID_PTR(pHandle->pvObj)
        &&  VALID_PTR(pHandle->pSession)
        &&  VALID_PTR(pHandle->pGVM)
        &&  pHandle->pGVM->u32Magic == GVM_MAGIC)
    {
        void *pvObj = pHandle->pvObj;
        pHandle->pvObj = NULL;
        gvmmR0CreateDestroyUnlock(pGVMM);

        SUPR0ObjRelease(pvObj, pHandle->pSession);
    }
    else
    {
        SUPR0Printf("GVMMR0DestroyVM: pHandle=%p:{.pVM=%p, .hEMT0=%p, .ProcId=%u, .pvObj=%p} pVM=%p hSelf=%p\n",
                    pHandle, pHandle->pVM, pHandle->hEMT0, pHandle->ProcId, pHandle->pvObj, pVM, hSelf);
        gvmmR0CreateDestroyUnlock(pGVMM);
        rc = VERR_GVMM_IPE_2;
    }

    return rc;
}


/**
 * Performs VM cleanup task as part of object destruction.
 *
 * @param   pGVM        The GVM pointer.
 */
static void gvmmR0CleanupVM(PGVM pGVM)
{
    if (    pGVM->gvmm.s.fDoneVMMR0Init
        &&  !pGVM->gvmm.s.fDoneVMMR0Term)
    {
        if (    pGVM->gvmm.s.VMMemObj != NIL_RTR0MEMOBJ
            &&  RTR0MemObjAddress(pGVM->gvmm.s.VMMemObj) == pGVM->pVM)
        {
            LogFlow(("gvmmR0CleanupVM: Calling VMMR0TermVM\n"));
            VMMR0TermVM(pGVM->pVM, pGVM);
        }
        else
            AssertMsgFailed(("gvmmR0CleanupVM: VMMemObj=%p pVM=%p\n", pGVM->gvmm.s.VMMemObj, pGVM->pVM));
    }

    GMMR0CleanupVM(pGVM);
}


/**
 * Handle destructor.
 *
 * @param   pvGVMM      The GVM instance pointer.
 * @param   pvHandle    The handle pointer.
 */
static DECLCALLBACK(void) gvmmR0HandleObjDestructor(void *pvObj, void *pvGVMM, void *pvHandle)
{
    LogFlow(("gvmmR0HandleObjDestructor: %p %p %p\n", pvObj, pvGVMM, pvHandle));

    /*
     * Some quick, paranoid, input validation.
     */
    PGVMHANDLE pHandle = (PGVMHANDLE)pvHandle;
    AssertPtr(pHandle);
    PGVMM pGVMM = (PGVMM)pvGVMM;
    Assert(pGVMM == g_pGVMM);
    const uint16_t iHandle = pHandle - &pGVMM->aHandles[0];
    if (    !iHandle
        ||  iHandle >= RT_ELEMENTS(pGVMM->aHandles)
        ||  iHandle != pHandle->iSelf)
    {
        SUPR0Printf("GVM: handle %d is out of range or corrupt (iSelf=%d)!\n", iHandle, pHandle->iSelf);
        return;
    }

    int rc = gvmmR0CreateDestroyLock(pGVMM);
    AssertRC(rc);
    rc = gvmmR0UsedLock(pGVMM);
    AssertRC(rc);

    /*
     * This is a tad slow but a doubly linked list is too much hassle.
     */
    if (RT_UNLIKELY(pHandle->iNext >= RT_ELEMENTS(pGVMM->aHandles)))
    {
        SUPR0Printf("GVM: used list index %d is out of range!\n", pHandle->iNext);
        gvmmR0UsedUnlock(pGVMM);
        gvmmR0CreateDestroyUnlock(pGVMM);
        return;
    }

    if (pGVMM->iUsedHead == iHandle)
        pGVMM->iUsedHead = pHandle->iNext;
    else
    {
        uint16_t iPrev = pGVMM->iUsedHead;
        int c = RT_ELEMENTS(pGVMM->aHandles) + 2;
        while (iPrev)
        {
            if (RT_UNLIKELY(iPrev >= RT_ELEMENTS(pGVMM->aHandles)))
            {
                SUPR0Printf("GVM: used list index %d is out of range!\n", iPrev);
                gvmmR0UsedUnlock(pGVMM);
                gvmmR0CreateDestroyUnlock(pGVMM);
                return;
            }
            if (RT_UNLIKELY(c-- <= 0))
            {
                iPrev = 0;
                break;
            }

            if (pGVMM->aHandles[iPrev].iNext == iHandle)
                break;
            iPrev = pGVMM->aHandles[iPrev].iNext;
        }
        if (!iPrev)
        {
            SUPR0Printf("GVM: can't find the handle previous previous of %d!\n", pHandle->iSelf);
            gvmmR0UsedUnlock(pGVMM);
            gvmmR0CreateDestroyUnlock(pGVMM);
            return;
        }

        Assert(pGVMM->aHandles[iPrev].iNext == iHandle);
        pGVMM->aHandles[iPrev].iNext = pHandle->iNext;
    }
    pHandle->iNext = 0;
    pGVMM->cVMs--;

    /*
     * Do the global cleanup round.
     */
    PGVM pGVM = pHandle->pGVM;
    if (    VALID_PTR(pGVM)
        &&  pGVM->u32Magic == GVM_MAGIC)
    {
        pGVMM->cEMTs -= pGVM->cCpus;
        gvmmR0UsedUnlock(pGVMM);

        gvmmR0CleanupVM(pGVM);

        /*
         * Do the GVMM cleanup - must be done last.
         */
        /* The VM and VM pages mappings/allocations. */
        if (pGVM->gvmm.s.VMPagesMapObj != NIL_RTR0MEMOBJ)
        {
            rc = RTR0MemObjFree(pGVM->gvmm.s.VMPagesMapObj, false /* fFreeMappings */); AssertRC(rc);
            pGVM->gvmm.s.VMPagesMapObj = NIL_RTR0MEMOBJ;
        }

        if (pGVM->gvmm.s.VMMapObj != NIL_RTR0MEMOBJ)
        {
            rc = RTR0MemObjFree(pGVM->gvmm.s.VMMapObj, false /* fFreeMappings */); AssertRC(rc);
            pGVM->gvmm.s.VMMapObj = NIL_RTR0MEMOBJ;
        }

        if (pGVM->gvmm.s.VMPagesMemObj != NIL_RTR0MEMOBJ)
        {
            rc = RTR0MemObjFree(pGVM->gvmm.s.VMPagesMemObj, false /* fFreeMappings */); AssertRC(rc);
            pGVM->gvmm.s.VMPagesMemObj = NIL_RTR0MEMOBJ;
        }

        if (pGVM->gvmm.s.VMMemObj != NIL_RTR0MEMOBJ)
        {
            rc = RTR0MemObjFree(pGVM->gvmm.s.VMMemObj, false /* fFreeMappings */); AssertRC(rc);
            pGVM->gvmm.s.VMMemObj = NIL_RTR0MEMOBJ;
        }

        for (VMCPUID i = 0; i < pGVM->cCpus; i++)
        {
            if (pGVM->aCpus[i].gvmm.s.HaltEventMulti != NIL_RTSEMEVENTMULTI)
            {
                rc = RTSemEventMultiDestroy(pGVM->aCpus[i].gvmm.s.HaltEventMulti); AssertRC(rc);
                pGVM->aCpus[i].gvmm.s.HaltEventMulti = NIL_RTSEMEVENTMULTI;
            }
        }

        /* the GVM structure itself. */
        pGVM->u32Magic |= UINT32_C(0x80000000);
        RTMemFree(pGVM);

        /* Re-acquire the UsedLock before freeing the handle since we're updating handle fields. */
        rc = gvmmR0UsedLock(pGVMM);
        AssertRC(rc);
    }
    /* else: GVMMR0CreateVM cleanup. */

    /*
     * Free the handle.
     */
    pHandle->iNext = pGVMM->iFreeHead;
    pGVMM->iFreeHead = iHandle;
    ASMAtomicWriteNullPtr(&pHandle->pGVM);
    ASMAtomicWriteNullPtr(&pHandle->pVM);
    ASMAtomicWriteNullPtr(&pHandle->pvObj);
    ASMAtomicWriteNullPtr(&pHandle->pSession);
    ASMAtomicWriteHandle(&pHandle->hEMT0,        NIL_RTNATIVETHREAD);
    ASMAtomicWriteU32(&pHandle->ProcId,          NIL_RTPROCESS);

    gvmmR0UsedUnlock(pGVMM);
    gvmmR0CreateDestroyUnlock(pGVMM);
    LogFlow(("gvmmR0HandleObjDestructor: returns\n"));
}


/**
 * Registers the calling thread as the EMT of a Virtual CPU.
 *
 * Note that VCPU 0 is automatically registered during VM creation.
 *
 * @returns VBox status code
 * @param   pVM             Pointer to the VM.
 * @param   idCpu           VCPU id.
 */
GVMMR0DECL(int) GVMMR0RegisterVCpu(PVM pVM, VMCPUID idCpu)
{
    AssertReturn(idCpu != 0, VERR_NOT_OWNER);

    /*
     * Validate the VM structure, state and handle.
     */
    PGVM pGVM;
    PGVMM pGVMM;
    int rc = gvmmR0ByVM(pVM, &pGVM, &pGVMM, false /* fTakeUsedLock */);
    if (RT_FAILURE(rc))
        return rc;

    AssertReturn(idCpu < pGVM->cCpus, VERR_INVALID_CPU_ID);
    AssertReturn(pGVM->aCpus[idCpu].hEMT == NIL_RTNATIVETHREAD, VERR_ACCESS_DENIED);
    Assert(pGVM->cCpus == pVM->cCpus);
    Assert(pVM->aCpus[idCpu].hNativeThreadR0 == NIL_RTNATIVETHREAD);

    pVM->aCpus[idCpu].hNativeThreadR0 = pGVM->aCpus[idCpu].hEMT = RTThreadNativeSelf();

    return VINF_SUCCESS;
}


/**
 * Lookup a GVM structure by its handle.
 *
 * @returns The GVM pointer on success, NULL on failure.
 * @param   hGVM    The global VM handle. Asserts on bad handle.
 */
GVMMR0DECL(PGVM) GVMMR0ByHandle(uint32_t hGVM)
{
    PGVMM pGVMM;
    GVMM_GET_VALID_INSTANCE(pGVMM, NULL);

    /*
     * Validate.
     */
    AssertReturn(hGVM != NIL_GVM_HANDLE, NULL);
    AssertReturn(hGVM < RT_ELEMENTS(pGVMM->aHandles), NULL);

    /*
     * Look it up.
     */
    PGVMHANDLE pHandle = &pGVMM->aHandles[hGVM];
    AssertPtrReturn(pHandle->pVM, NULL);
    AssertPtrReturn(pHandle->pvObj, NULL);
    PGVM pGVM = pHandle->pGVM;
    AssertPtrReturn(pGVM, NULL);
    AssertReturn(pGVM->pVM == pHandle->pVM, NULL);

    return pHandle->pGVM;
}


/**
 * Lookup a GVM structure by the shared VM structure.
 *
 * The calling thread must be in the same process as the VM. All current lookups
 * are by threads inside the same process, so this will not be an issue.
 *
 * @returns VBox status code.
 * @param   pVM             Pointer to the VM.
 * @param   ppGVM           Where to store the GVM pointer.
 * @param   ppGVMM          Where to store the pointer to the GVMM instance data.
 * @param   fTakeUsedLock   Whether to take the used lock or not.
 *                          Be very careful if not taking the lock as it's possible that
 *                          the VM will disappear then.
 *
 * @remark  This will not assert on an invalid pVM but try return silently.
 */
static int gvmmR0ByVM(PVM pVM, PGVM *ppGVM, PGVMM *ppGVMM, bool fTakeUsedLock)
{
    RTPROCESS ProcId = RTProcSelf();
    PGVMM pGVMM;
    GVMM_GET_VALID_INSTANCE(pGVMM, VERR_GVMM_INSTANCE);

    /*
     * Validate.
     */
    if (RT_UNLIKELY(    !VALID_PTR(pVM)
                    ||  ((uintptr_t)pVM & PAGE_OFFSET_MASK)))
        return VERR_INVALID_POINTER;
    if (RT_UNLIKELY(    pVM->enmVMState < VMSTATE_CREATING
                    ||  pVM->enmVMState >= VMSTATE_TERMINATED))
        return VERR_INVALID_POINTER;

    uint16_t hGVM = pVM->hSelf;
    if (RT_UNLIKELY(    hGVM == NIL_GVM_HANDLE
                    ||  hGVM >= RT_ELEMENTS(pGVMM->aHandles)))
        return VERR_INVALID_HANDLE;

    /*
     * Look it up.
     */
    PGVMHANDLE pHandle = &pGVMM->aHandles[hGVM];
    PGVM pGVM;
    if (fTakeUsedLock)
    {
        int rc = gvmmR0UsedLock(pGVMM);
        AssertRCReturn(rc, rc);

        pGVM = pHandle->pGVM;
        if (RT_UNLIKELY(    pHandle->pVM != pVM
                        ||  pHandle->ProcId != ProcId
                        ||  !VALID_PTR(pHandle->pvObj)
                        ||  !VALID_PTR(pGVM)
                        ||  pGVM->pVM != pVM))
        {
            gvmmR0UsedUnlock(pGVMM);
            return VERR_INVALID_HANDLE;
        }
    }
    else
    {
        if (RT_UNLIKELY(pHandle->pVM != pVM))
            return VERR_INVALID_HANDLE;
        if (RT_UNLIKELY(pHandle->ProcId != ProcId))
            return VERR_INVALID_HANDLE;
        if (RT_UNLIKELY(!VALID_PTR(pHandle->pvObj)))
            return VERR_INVALID_HANDLE;

        pGVM = pHandle->pGVM;
        if (RT_UNLIKELY(!VALID_PTR(pGVM)))
            return VERR_INVALID_HANDLE;
        if (RT_UNLIKELY(pGVM->pVM != pVM))
            return VERR_INVALID_HANDLE;
    }

    *ppGVM = pGVM;
    *ppGVMM = pGVMM;
    return VINF_SUCCESS;
}


/**
 * Lookup a GVM structure by the shared VM structure.
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   ppGVM       Where to store the GVM pointer.
 *
 * @remark  This will not take the 'used'-lock because it doesn't do
 *          nesting and this function will be used from under the lock.
 */
GVMMR0DECL(int) GVMMR0ByVM(PVM pVM, PGVM *ppGVM)
{
    PGVMM pGVMM;
    return gvmmR0ByVM(pVM, ppGVM, &pGVMM, false /* fTakeUsedLock */);
}


/**
 * Lookup a GVM structure by the shared VM structure and ensuring that the
 * caller is an EMT thread.
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   idCpu       The Virtual CPU ID of the calling EMT.
 * @param   ppGVM       Where to store the GVM pointer.
 * @param   ppGVMM      Where to store the pointer to the GVMM instance data.
 * @thread  EMT
 *
 * @remark  This will assert in all failure paths.
 */
static int gvmmR0ByVMAndEMT(PVM pVM, VMCPUID idCpu, PGVM *ppGVM, PGVMM *ppGVMM)
{
    PGVMM pGVMM;
    GVMM_GET_VALID_INSTANCE(pGVMM, VERR_GVMM_INSTANCE);

    /*
     * Validate.
     */
    AssertPtrReturn(pVM, VERR_INVALID_POINTER);
    AssertReturn(!((uintptr_t)pVM & PAGE_OFFSET_MASK), VERR_INVALID_POINTER);

    uint16_t hGVM = pVM->hSelf;
    AssertReturn(hGVM != NIL_GVM_HANDLE, VERR_INVALID_HANDLE);
    AssertReturn(hGVM < RT_ELEMENTS(pGVMM->aHandles), VERR_INVALID_HANDLE);

    /*
     * Look it up.
     */
    PGVMHANDLE pHandle = &pGVMM->aHandles[hGVM];
    AssertReturn(pHandle->pVM == pVM, VERR_NOT_OWNER);
    RTPROCESS ProcId = RTProcSelf();
    AssertReturn(pHandle->ProcId == ProcId, VERR_NOT_OWNER);
    AssertPtrReturn(pHandle->pvObj, VERR_NOT_OWNER);

    PGVM pGVM = pHandle->pGVM;
    AssertPtrReturn(pGVM, VERR_NOT_OWNER);
    AssertReturn(pGVM->pVM == pVM, VERR_NOT_OWNER);
    RTNATIVETHREAD hAllegedEMT = RTThreadNativeSelf();
    AssertReturn(idCpu < pGVM->cCpus, VERR_INVALID_CPU_ID);
    AssertReturn(pGVM->aCpus[idCpu].hEMT == hAllegedEMT, VERR_NOT_OWNER);

    *ppGVM = pGVM;
    *ppGVMM = pGVMM;
    return VINF_SUCCESS;
}


/**
 * Lookup a GVM structure by the shared VM structure
 * and ensuring that the caller is the EMT thread.
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   idCpu       The Virtual CPU ID of the calling EMT.
 * @param   ppGVM       Where to store the GVM pointer.
 * @thread  EMT
 */
GVMMR0DECL(int) GVMMR0ByVMAndEMT(PVM pVM, VMCPUID idCpu, PGVM *ppGVM)
{
    AssertPtrReturn(ppGVM, VERR_INVALID_POINTER);
    PGVMM pGVMM;
    return gvmmR0ByVMAndEMT(pVM, idCpu, ppGVM, &pGVMM);
}


/**
 * Lookup a VM by its global handle.
 *
 * @returns Pointer to the VM on success, NULL on failure.
 * @param   hGVM    The global VM handle. Asserts on bad handle.
 */
GVMMR0DECL(PVM) GVMMR0GetVMByHandle(uint32_t hGVM)
{
    PGVM pGVM = GVMMR0ByHandle(hGVM);
    return pGVM ? pGVM->pVM : NULL;
}


/**
 * Looks up the VM belonging to the specified EMT thread.
 *
 * This is used by the assertion machinery in VMMR0.cpp to avoid causing
 * unnecessary kernel panics when the EMT thread hits an assertion. The
 * call may or not be an EMT thread.
 *
 * @returns Pointer to the VM on success, NULL on failure.
 * @param   hEMT    The native thread handle of the EMT.
 *                  NIL_RTNATIVETHREAD means the current thread
 */
GVMMR0DECL(PVM) GVMMR0GetVMByEMT(RTNATIVETHREAD hEMT)
{
    /*
     * No Assertions here as we're usually called in a AssertMsgN or
     * RTAssert* context.
     */
    PGVMM pGVMM = g_pGVMM;
    if (    !VALID_PTR(pGVMM)
        ||  pGVMM->u32Magic != GVMM_MAGIC)
        return NULL;

    if (hEMT == NIL_RTNATIVETHREAD)
        hEMT = RTThreadNativeSelf();
    RTPROCESS ProcId = RTProcSelf();

    /*
     * Search the handles in a linear fashion as we don't dare to take the lock (assert).
     */
    for (unsigned i = 1; i < RT_ELEMENTS(pGVMM->aHandles); i++)
    {
        if (    pGVMM->aHandles[i].iSelf == i
            &&  pGVMM->aHandles[i].ProcId == ProcId
            &&  VALID_PTR(pGVMM->aHandles[i].pvObj)
            &&  VALID_PTR(pGVMM->aHandles[i].pVM)
            &&  VALID_PTR(pGVMM->aHandles[i].pGVM))
        {
            if (pGVMM->aHandles[i].hEMT0 == hEMT)
                return pGVMM->aHandles[i].pVM;

            /* This is fearly safe with the current process per VM approach. */
            PGVM pGVM = pGVMM->aHandles[i].pGVM;
            VMCPUID const cCpus = pGVM->cCpus;
            if (    cCpus < 1
                ||  cCpus > VMM_MAX_CPU_COUNT)
                continue;
            for (VMCPUID idCpu = 1; idCpu < cCpus; idCpu++)
                if (pGVM->aCpus[idCpu].hEMT == hEMT)
                    return pGVMM->aHandles[i].pVM;
        }
    }
    return NULL;
}


/**
 * This is will wake up expired and soon-to-be expired VMs.
 *
 * @returns Number of VMs that has been woken up.
 * @param   pGVMM       Pointer to the GVMM instance data.
 * @param   u64Now      The current time.
 */
static unsigned gvmmR0SchedDoWakeUps(PGVMM pGVMM, uint64_t u64Now)
{
    /*
     * Skip this if we've got disabled because of high resolution wakeups or by
     * the user.
     */
    if (   !pGVMM->nsEarlyWakeUp1
        && !pGVMM->nsEarlyWakeUp2)
        return 0;

/** @todo Rewrite this algorithm. See performance defect XYZ. */

    /*
     * A cheap optimization to stop wasting so much time here on big setups.
     */
    const uint64_t  uNsEarlyWakeUp2 = u64Now + pGVMM->nsEarlyWakeUp2;
    if (   pGVMM->cHaltedEMTs == 0
        || uNsEarlyWakeUp2 > pGVMM->uNsNextEmtWakeup)
        return 0;

    /*
     * The first pass will wake up VMs which have actually expired
     * and look for VMs that should be woken up in the 2nd and 3rd passes.
     */
    const uint64_t  uNsEarlyWakeUp1 = u64Now + pGVMM->nsEarlyWakeUp1;
    uint64_t        u64Min          = UINT64_MAX;
    unsigned        cWoken          = 0;
    unsigned        cHalted         = 0;
    unsigned        cTodo2nd        = 0;
    unsigned        cTodo3rd        = 0;
    for (unsigned i = pGVMM->iUsedHead, cGuard = 0;
         i != NIL_GVM_HANDLE && i < RT_ELEMENTS(pGVMM->aHandles);
         i = pGVMM->aHandles[i].iNext)
    {
        PGVM pCurGVM = pGVMM->aHandles[i].pGVM;
        if (    VALID_PTR(pCurGVM)
            &&  pCurGVM->u32Magic == GVM_MAGIC)
        {
            for (VMCPUID idCpu = 0; idCpu < pCurGVM->cCpus; idCpu++)
            {
                PGVMCPU     pCurGVCpu = &pCurGVM->aCpus[idCpu];
                uint64_t    u64       = ASMAtomicUoReadU64(&pCurGVCpu->gvmm.s.u64HaltExpire);
                if (u64)
                {
                    if (u64 <= u64Now)
                    {
                        if (ASMAtomicXchgU64(&pCurGVCpu->gvmm.s.u64HaltExpire, 0))
                        {
                            int rc = RTSemEventMultiSignal(pCurGVCpu->gvmm.s.HaltEventMulti);
                            AssertRC(rc);
                            cWoken++;
                        }
                    }
                    else
                    {
                        cHalted++;
                        if (u64 <= uNsEarlyWakeUp1)
                            cTodo2nd++;
                        else if (u64 <= uNsEarlyWakeUp2)
                            cTodo3rd++;
                        else if (u64 < u64Min)
                            u64 = u64Min;
                    }
                }
            }
        }
        AssertLogRelBreak(cGuard++ < RT_ELEMENTS(pGVMM->aHandles));
    }

    if (cTodo2nd)
    {
        for (unsigned i = pGVMM->iUsedHead, cGuard = 0;
             i != NIL_GVM_HANDLE && i < RT_ELEMENTS(pGVMM->aHandles);
             i = pGVMM->aHandles[i].iNext)
        {
            PGVM pCurGVM = pGVMM->aHandles[i].pGVM;
            if (    VALID_PTR(pCurGVM)
                &&  pCurGVM->u32Magic == GVM_MAGIC)
            {
                for (VMCPUID idCpu = 0; idCpu < pCurGVM->cCpus; idCpu++)
                {
                    PGVMCPU     pCurGVCpu = &pCurGVM->aCpus[idCpu];
                    uint64_t    u64       = ASMAtomicUoReadU64(&pCurGVCpu->gvmm.s.u64HaltExpire);
                    if (   u64
                        && u64 <= uNsEarlyWakeUp1)
                    {
                        if (ASMAtomicXchgU64(&pCurGVCpu->gvmm.s.u64HaltExpire, 0))
                        {
                            int rc = RTSemEventMultiSignal(pCurGVCpu->gvmm.s.HaltEventMulti);
                            AssertRC(rc);
                            cWoken++;
                        }
                    }
                }
            }
            AssertLogRelBreak(cGuard++ < RT_ELEMENTS(pGVMM->aHandles));
        }
    }

    if (cTodo3rd)
    {
        for (unsigned i = pGVMM->iUsedHead, cGuard = 0;
             i != NIL_GVM_HANDLE && i < RT_ELEMENTS(pGVMM->aHandles);
             i = pGVMM->aHandles[i].iNext)
        {
            PGVM pCurGVM = pGVMM->aHandles[i].pGVM;
            if (    VALID_PTR(pCurGVM)
                &&  pCurGVM->u32Magic == GVM_MAGIC)
            {
                for (VMCPUID idCpu = 0; idCpu < pCurGVM->cCpus; idCpu++)
                {
                    PGVMCPU     pCurGVCpu = &pCurGVM->aCpus[idCpu];
                    uint64_t    u64       = ASMAtomicUoReadU64(&pCurGVCpu->gvmm.s.u64HaltExpire);
                    if (   u64
                        && u64 <= uNsEarlyWakeUp2)
                    {
                        if (ASMAtomicXchgU64(&pCurGVCpu->gvmm.s.u64HaltExpire, 0))
                        {
                            int rc = RTSemEventMultiSignal(pCurGVCpu->gvmm.s.HaltEventMulti);
                            AssertRC(rc);
                            cWoken++;
                        }
                    }
                }
            }
            AssertLogRelBreak(cGuard++ < RT_ELEMENTS(pGVMM->aHandles));
        }
    }

    /*
     * Set the minimum value.
     */
    pGVMM->uNsNextEmtWakeup = u64Min;

    return cWoken;
}


/**
 * Halt the EMT thread.
 *
 * @returns VINF_SUCCESS normal wakeup (timeout or kicked by other thread).
 *          VERR_INTERRUPTED if a signal was scheduled for the thread.
 * @param   pVM                 Pointer to the VM.
 * @param   idCpu               The Virtual CPU ID of the calling EMT.
 * @param   u64ExpireGipTime    The time for the sleep to expire expressed as GIP time.
 * @thread  EMT(idCpu).
 */
GVMMR0DECL(int) GVMMR0SchedHalt(PVM pVM, VMCPUID idCpu, uint64_t u64ExpireGipTime)
{
    LogFlow(("GVMMR0SchedHalt: pVM=%p\n", pVM));

    /*
     * Validate the VM structure, state and handle.
     */
    PGVM pGVM;
    PGVMM pGVMM;
    int rc = gvmmR0ByVMAndEMT(pVM, idCpu, &pGVM, &pGVMM);
    if (RT_FAILURE(rc))
        return rc;
    pGVM->gvmm.s.StatsSched.cHaltCalls++;

    PGVMCPU pCurGVCpu = &pGVM->aCpus[idCpu];
    Assert(!pCurGVCpu->gvmm.s.u64HaltExpire);

    /*
     * Take the UsedList semaphore, get the current time
     * and check if anyone needs waking up.
     * Interrupts must NOT be disabled at this point because we ask for GIP time!
     */
    rc = gvmmR0UsedLock(pGVMM);
    AssertRC(rc);

    pCurGVCpu->gvmm.s.iCpuEmt = ASMGetApicId();

    /* GIP hack: We might are frequently sleeping for short intervals where the
       difference between GIP and system time matters on systems with high resolution
       system time. So, convert the input from GIP to System time in that case. */
    Assert(ASMGetFlags() & X86_EFL_IF);
    const uint64_t u64NowSys = RTTimeSystemNanoTS();
    const uint64_t u64NowGip = RTTimeNanoTS();
    pGVM->gvmm.s.StatsSched.cHaltWakeUps += gvmmR0SchedDoWakeUps(pGVMM, u64NowGip);

    /*
     * Go to sleep if we must...
     * Cap the sleep time to 1 second to be on the safe side.
     */
    uint64_t cNsInterval = u64ExpireGipTime - u64NowGip;
    if (    u64NowGip < u64ExpireGipTime
        &&  cNsInterval >= (pGVMM->cEMTs > pGVMM->cEMTsMeansCompany
                            ? pGVMM->nsMinSleepCompany
                            : pGVMM->nsMinSleepAlone))
    {
        pGVM->gvmm.s.StatsSched.cHaltBlocking++;
        if (cNsInterval > RT_NS_1SEC)
            u64ExpireGipTime = u64NowGip + RT_NS_1SEC;
        if (u64ExpireGipTime < pGVMM->uNsNextEmtWakeup)
            pGVMM->uNsNextEmtWakeup = u64ExpireGipTime;
        ASMAtomicWriteU64(&pCurGVCpu->gvmm.s.u64HaltExpire, u64ExpireGipTime);
        ASMAtomicIncU32(&pGVMM->cHaltedEMTs);
        gvmmR0UsedUnlock(pGVMM);

        rc = RTSemEventMultiWaitEx(pCurGVCpu->gvmm.s.HaltEventMulti,
                                   RTSEMWAIT_FLAGS_ABSOLUTE | RTSEMWAIT_FLAGS_NANOSECS | RTSEMWAIT_FLAGS_INTERRUPTIBLE,
                                   u64NowGip > u64NowSys ? u64ExpireGipTime : u64NowSys + cNsInterval);

        ASMAtomicWriteU64(&pCurGVCpu->gvmm.s.u64HaltExpire, 0);
        ASMAtomicDecU32(&pGVMM->cHaltedEMTs);

        /* Reset the semaphore to try prevent a few false wake-ups. */
        if (rc == VINF_SUCCESS)
            RTSemEventMultiReset(pCurGVCpu->gvmm.s.HaltEventMulti);
        else if (rc == VERR_TIMEOUT)
        {
            pGVM->gvmm.s.StatsSched.cHaltTimeouts++;
            rc = VINF_SUCCESS;
        }
    }
    else
    {
        pGVM->gvmm.s.StatsSched.cHaltNotBlocking++;
        gvmmR0UsedUnlock(pGVMM);
        RTSemEventMultiReset(pCurGVCpu->gvmm.s.HaltEventMulti);
    }

    return rc;
}


/**
 * Worker for GVMMR0SchedWakeUp and GVMMR0SchedWakeUpAndPokeCpus that wakes up
 * the a sleeping EMT.
 *
 * @retval  VINF_SUCCESS if successfully woken up.
 * @retval  VINF_GVM_NOT_BLOCKED if the EMT wasn't blocked.
 *
 * @param   pGVM                The global (ring-0) VM structure.
 * @param   pGVCpu              The global (ring-0) VCPU structure.
 */
DECLINLINE(int) gvmmR0SchedWakeUpOne(PGVM pGVM, PGVMCPU pGVCpu)
{
    pGVM->gvmm.s.StatsSched.cWakeUpCalls++;

    /*
     * Signal the semaphore regardless of whether it's current blocked on it.
     *
     * The reason for this is that there is absolutely no way we can be 100%
     * certain that it isn't *about* go to go to sleep on it and just got
     * delayed a bit en route. So, we will always signal the semaphore when
     * the it is flagged as halted in the VMM.
     */
/** @todo we can optimize some of that by means of the pVCpu->enmState now. */
    int rc;
    if (pGVCpu->gvmm.s.u64HaltExpire)
    {
        rc = VINF_SUCCESS;
        ASMAtomicWriteU64(&pGVCpu->gvmm.s.u64HaltExpire, 0);
    }
    else
    {
        rc = VINF_GVM_NOT_BLOCKED;
        pGVM->gvmm.s.StatsSched.cWakeUpNotHalted++;
    }

    int rc2 = RTSemEventMultiSignal(pGVCpu->gvmm.s.HaltEventMulti);
    AssertRC(rc2);

    return rc;
}


/**
 * Wakes up the halted EMT thread so it can service a pending request.
 *
 * @returns VBox status code.
 * @retval  VINF_SUCCESS if successfully woken up.
 * @retval  VINF_GVM_NOT_BLOCKED if the EMT wasn't blocked.
 *
 * @param   pVM                 Pointer to the VM.
 * @param   idCpu               The Virtual CPU ID of the EMT to wake up.
 * @param   fTakeUsedLock       Take the used lock or not
 * @thread  Any but EMT.
 */
GVMMR0DECL(int) GVMMR0SchedWakeUpEx(PVM pVM, VMCPUID idCpu, bool fTakeUsedLock)
{
    /*
     * Validate input and take the UsedLock.
     */
    PGVM pGVM;
    PGVMM pGVMM;
    int rc = gvmmR0ByVM(pVM, &pGVM, &pGVMM, fTakeUsedLock);
    if (RT_SUCCESS(rc))
    {
        if (idCpu < pGVM->cCpus)
        {
            /*
             * Do the actual job.
             */
            rc = gvmmR0SchedWakeUpOne(pGVM, &pGVM->aCpus[idCpu]);

            if (fTakeUsedLock)
            {
                /*
                 * While we're here, do a round of scheduling.
                 */
                Assert(ASMGetFlags() & X86_EFL_IF);
                const uint64_t u64Now = RTTimeNanoTS(); /* (GIP time) */
                pGVM->gvmm.s.StatsSched.cWakeUpWakeUps += gvmmR0SchedDoWakeUps(pGVMM, u64Now);
            }
        }
        else
            rc = VERR_INVALID_CPU_ID;

        if (fTakeUsedLock)
        {
            int rc2 = gvmmR0UsedUnlock(pGVMM);
            AssertRC(rc2);
        }
    }

    LogFlow(("GVMMR0SchedWakeUp: returns %Rrc\n", rc));
    return rc;
}


/**
 * Wakes up the halted EMT thread so it can service a pending request.
 *
 * @returns VBox status code.
 * @retval  VINF_SUCCESS if successfully woken up.
 * @retval  VINF_GVM_NOT_BLOCKED if the EMT wasn't blocked.
 *
 * @param   pVM                 Pointer to the VM.
 * @param   idCpu               The Virtual CPU ID of the EMT to wake up.
 * @thread  Any but EMT.
 */
GVMMR0DECL(int) GVMMR0SchedWakeUp(PVM pVM, VMCPUID idCpu)
{
    return GVMMR0SchedWakeUpEx(pVM, idCpu, true /* fTakeUsedLock */);
}

/**
 * Worker common to GVMMR0SchedPoke and GVMMR0SchedWakeUpAndPokeCpus that pokes
 * the Virtual CPU if it's still busy executing guest code.
 *
 * @returns VBox status code.
 * @retval  VINF_SUCCESS if poked successfully.
 * @retval  VINF_GVM_NOT_BUSY_IN_GC if the EMT wasn't busy in GC.
 *
 * @param   pGVM                The global (ring-0) VM structure.
 * @param   pVCpu               Pointer to the VMCPU.
 */
DECLINLINE(int) gvmmR0SchedPokeOne(PGVM pGVM, PVMCPU pVCpu)
{
    pGVM->gvmm.s.StatsSched.cPokeCalls++;

    RTCPUID idHostCpu = pVCpu->idHostCpu;
    if (    idHostCpu == NIL_RTCPUID
        ||  VMCPU_GET_STATE(pVCpu) != VMCPUSTATE_STARTED_EXEC)
    {
        pGVM->gvmm.s.StatsSched.cPokeNotBusy++;
        return VINF_GVM_NOT_BUSY_IN_GC;
    }

    /* Note: this function is not implemented on Darwin and Linux (kernel < 2.6.19) */
    RTMpPokeCpu(idHostCpu);
    return VINF_SUCCESS;
}

/**
 * Pokes an EMT if it's still busy running guest code.
 *
 * @returns VBox status code.
 * @retval  VINF_SUCCESS if poked successfully.
 * @retval  VINF_GVM_NOT_BUSY_IN_GC if the EMT wasn't busy in GC.
 *
 * @param   pVM                 Pointer to the VM.
 * @param   idCpu               The ID of the virtual CPU to poke.
 * @param   fTakeUsedLock       Take the used lock or not
 */
GVMMR0DECL(int) GVMMR0SchedPokeEx(PVM pVM, VMCPUID idCpu, bool fTakeUsedLock)
{
    /*
     * Validate input and take the UsedLock.
     */
    PGVM pGVM;
    PGVMM pGVMM;
    int rc = gvmmR0ByVM(pVM, &pGVM, &pGVMM, fTakeUsedLock);
    if (RT_SUCCESS(rc))
    {
        if (idCpu < pGVM->cCpus)
            rc = gvmmR0SchedPokeOne(pGVM, &pVM->aCpus[idCpu]);
        else
            rc = VERR_INVALID_CPU_ID;

        if (fTakeUsedLock)
        {
            int rc2 = gvmmR0UsedUnlock(pGVMM);
            AssertRC(rc2);
        }
    }

    LogFlow(("GVMMR0SchedWakeUpAndPokeCpus: returns %Rrc\n", rc));
    return rc;
}


/**
 * Pokes an EMT if it's still busy running guest code.
 *
 * @returns VBox status code.
 * @retval  VINF_SUCCESS if poked successfully.
 * @retval  VINF_GVM_NOT_BUSY_IN_GC if the EMT wasn't busy in GC.
 *
 * @param   pVM                 Pointer to the VM.
 * @param   idCpu               The ID of the virtual CPU to poke.
 */
GVMMR0DECL(int) GVMMR0SchedPoke(PVM pVM, VMCPUID idCpu)
{
    return GVMMR0SchedPokeEx(pVM, idCpu, true /* fTakeUsedLock */);
}


/**
 * Wakes up a set of halted EMT threads so they can service pending request.
 *
 * @returns VBox status code, no informational stuff.
 *
 * @param   pVM                 Pointer to the VM.
 * @param   pSleepSet           The set of sleepers to wake up.
 * @param   pPokeSet            The set of CPUs to poke.
 */
GVMMR0DECL(int) GVMMR0SchedWakeUpAndPokeCpus(PVM pVM, PCVMCPUSET pSleepSet, PCVMCPUSET pPokeSet)
{
    AssertPtrReturn(pSleepSet, VERR_INVALID_POINTER);
    AssertPtrReturn(pPokeSet, VERR_INVALID_POINTER);
    RTNATIVETHREAD hSelf = RTThreadNativeSelf();

    /*
     * Validate input and take the UsedLock.
     */
    PGVM pGVM;
    PGVMM pGVMM;
    int rc = gvmmR0ByVM(pVM, &pGVM, &pGVMM, true /* fTakeUsedLock */);
    if (RT_SUCCESS(rc))
    {
        rc = VINF_SUCCESS;
        VMCPUID idCpu = pGVM->cCpus;
        while (idCpu-- > 0)
        {
            /* Don't try poke or wake up ourselves. */
            if (pGVM->aCpus[idCpu].hEMT == hSelf)
                continue;

            /* just ignore errors for now. */
            if (VMCPUSET_IS_PRESENT(pSleepSet, idCpu))
                gvmmR0SchedWakeUpOne(pGVM, &pGVM->aCpus[idCpu]);
            else if (VMCPUSET_IS_PRESENT(pPokeSet, idCpu))
                gvmmR0SchedPokeOne(pGVM, &pVM->aCpus[idCpu]);
        }

        int rc2 = gvmmR0UsedUnlock(pGVMM);
        AssertRC(rc2);
    }

    LogFlow(("GVMMR0SchedWakeUpAndPokeCpus: returns %Rrc\n", rc));
    return rc;
}


/**
 * VMMR0 request wrapper for GVMMR0SchedWakeUpAndPokeCpus.
 *
 * @returns see GVMMR0SchedWakeUpAndPokeCpus.
 * @param   pVM             Pointer to the VM.
 * @param   pReq            Pointer to the request packet.
 */
GVMMR0DECL(int) GVMMR0SchedWakeUpAndPokeCpusReq(PVM pVM, PGVMMSCHEDWAKEUPANDPOKECPUSREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(*pReq), ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    return GVMMR0SchedWakeUpAndPokeCpus(pVM, &pReq->SleepSet, &pReq->PokeSet);
}



/**
 * Poll the schedule to see if someone else should get a chance to run.
 *
 * This is a bit hackish and will not work too well if the machine is
 * under heavy load from non-VM processes.
 *
 * @returns VINF_SUCCESS if not yielded.
 *          VINF_GVM_YIELDED if an attempt to switch to a different VM task was made.
 * @param   pVM                 Pointer to the VM.
 * @param   idCpu               The Virtual CPU ID of the calling EMT.
 * @param   u64ExpireGipTime    The time for the sleep to expire expressed as GIP time.
 * @param   fYield              Whether to yield or not.
 *                              This is for when we're spinning in the halt loop.
 * @thread  EMT(idCpu).
 */
GVMMR0DECL(int) GVMMR0SchedPoll(PVM pVM, VMCPUID idCpu, bool fYield)
{
    /*
     * Validate input.
     */
    PGVM pGVM;
    PGVMM pGVMM;
    int rc = gvmmR0ByVMAndEMT(pVM, idCpu, &pGVM, &pGVMM);
    if (RT_SUCCESS(rc))
    {
        rc = gvmmR0UsedLock(pGVMM);
        AssertRC(rc);
        pGVM->gvmm.s.StatsSched.cPollCalls++;

        Assert(ASMGetFlags() & X86_EFL_IF);
        const uint64_t u64Now = RTTimeNanoTS(); /* (GIP time) */

        if (!fYield)
            pGVM->gvmm.s.StatsSched.cPollWakeUps += gvmmR0SchedDoWakeUps(pGVMM, u64Now);
        else
        {
            /** @todo implement this... */
            rc = VERR_NOT_IMPLEMENTED;
        }

        gvmmR0UsedUnlock(pGVMM);
    }

    LogFlow(("GVMMR0SchedWakeUp: returns %Rrc\n", rc));
    return rc;
}


#ifdef GVMM_SCHED_WITH_PPT
/**
 * Timer callback for the periodic preemption timer.
 *
 * @param   pTimer      The timer handle.
 * @param   pvUser      Pointer to the per cpu structure.
 * @param   iTick       The current tick.
 */
static DECLCALLBACK(void) gvmmR0SchedPeriodicPreemptionTimerCallback(PRTTIMER pTimer, void *pvUser, uint64_t iTick)
{
    PGVMMHOSTCPU pCpu = (PGVMMHOSTCPU)pvUser;
    NOREF(pTimer); NOREF(iTick);

    /*
     * Termination check
     */
    if (pCpu->u32Magic != GVMMHOSTCPU_MAGIC)
        return;

    /*
     * Do the house keeping.
     */
    RTSpinlockAcquire(pCpu->Ppt.hSpinlock);

    if (++pCpu->Ppt.iTickHistorization >= pCpu->Ppt.cTicksHistoriziationInterval)
    {
        /*
         * Historicize the max frequency.
         */
        uint32_t iHzHistory = ++pCpu->Ppt.iHzHistory % RT_ELEMENTS(pCpu->Ppt.aHzHistory);
        pCpu->Ppt.aHzHistory[iHzHistory] = pCpu->Ppt.uDesiredHz;
        pCpu->Ppt.iTickHistorization = 0;
        pCpu->Ppt.uDesiredHz         = 0;

        /*
         * Check if the current timer frequency.
         */
        uint32_t uHistMaxHz = 0;
        for (uint32_t i = 0; i < RT_ELEMENTS(pCpu->Ppt.aHzHistory); i++)
            if (pCpu->Ppt.aHzHistory[i] > uHistMaxHz)
                uHistMaxHz = pCpu->Ppt.aHzHistory[i];
        if (uHistMaxHz == pCpu->Ppt.uTimerHz)
            RTSpinlockReleaseNoInts(pCpu->Ppt.hSpinlock);
        else if (uHistMaxHz)
        {
            /*
             * Reprogram it.
             */
            pCpu->Ppt.cChanges++;
            pCpu->Ppt.iTickHistorization    = 0;
            pCpu->Ppt.uTimerHz              = uHistMaxHz;
            uint32_t const cNsInterval      = RT_NS_1SEC / uHistMaxHz;
            pCpu->Ppt.cNsInterval           = cNsInterval;
            if (cNsInterval < GVMMHOSTCPU_PPT_HIST_INTERVAL_NS)
                pCpu->Ppt.cTicksHistoriziationInterval = (  GVMMHOSTCPU_PPT_HIST_INTERVAL_NS
                                                          + GVMMHOSTCPU_PPT_HIST_INTERVAL_NS / 2 - 1)
                                                       / cNsInterval;
            else
                pCpu->Ppt.cTicksHistoriziationInterval = 1;
            RTSpinlockReleaseNoInts(pCpu->Ppt.hSpinlock);

            /*SUPR0Printf("Cpu%u: change to %u Hz / %u ns\n", pCpu->idxCpuSet, uHistMaxHz, cNsInterval);*/
            RTTimerChangeInterval(pTimer, cNsInterval);
        }
        else
        {
            /*
             * Stop it.
             */
            pCpu->Ppt.fStarted    = false;
            pCpu->Ppt.uTimerHz    = 0;
            pCpu->Ppt.cNsInterval = 0;
            RTSpinlockReleaseNoInts(pCpu->Ppt.hSpinlock);

            /*SUPR0Printf("Cpu%u: stopping (%u Hz)\n", pCpu->idxCpuSet, uHistMaxHz);*/
            RTTimerStop(pTimer);
        }
    }
    else
        RTSpinlockReleaseNoInts(pCpu->Ppt.hSpinlock);
}
#endif /* GVMM_SCHED_WITH_PPT */


/**
 * Updates the periodic preemption timer for the calling CPU.
 *
 * The caller must have disabled preemption!
 * The caller must check that the host can do high resolution timers.
 *
 * @param   pVM         Pointer to the VM.
 * @param   idHostCpu   The current host CPU id.
 * @param   uHz         The desired frequency.
 */
GVMMR0DECL(void) GVMMR0SchedUpdatePeriodicPreemptionTimer(PVM pVM, RTCPUID idHostCpu, uint32_t uHz)
{
    NOREF(pVM);
#ifdef GVMM_SCHED_WITH_PPT
    Assert(!RTThreadPreemptIsEnabled(NIL_RTTHREAD));
    Assert(RTTimerCanDoHighResolution());

    /*
     * Resolve the per CPU data.
     */
    uint32_t    iCpu  = RTMpCpuIdToSetIndex(idHostCpu);
    PGVMM       pGVMM = g_pGVMM;
    if (   !VALID_PTR(pGVMM)
        || pGVMM->u32Magic != GVMM_MAGIC)
        return;
    AssertMsgReturnVoid(iCpu < pGVMM->cHostCpus, ("iCpu=%d cHostCpus=%d\n", iCpu, pGVMM->cHostCpus));
    PGVMMHOSTCPU pCpu = &pGVMM->aHostCpus[iCpu];
    AssertMsgReturnVoid(   pCpu->u32Magic == GVMMHOSTCPU_MAGIC
                        && pCpu->idCpu    == idHostCpu,
                        ("u32Magic=%#x idCpu=% idHostCpu=%d\n", pCpu->u32Magic, pCpu->idCpu, idHostCpu));

    /*
     * Check whether we need to do anything about the timer.
     * We have to be a little bit careful since we might be race the timer
     * callback here.
     */
    if (uHz > 16384)
        uHz = 16384;  /** @todo add a query method for this! */
    if (RT_UNLIKELY(   uHz > ASMAtomicReadU32(&pCpu->Ppt.uDesiredHz)
                    && uHz >= pCpu->Ppt.uMinHz
                    && !pCpu->Ppt.fStarting /* solaris paranoia */))
    {
        RTSpinlockAcquire(pCpu->Ppt.hSpinlock);

        pCpu->Ppt.uDesiredHz = uHz;
        uint32_t cNsInterval = 0;
        if (!pCpu->Ppt.fStarted)
        {
            pCpu->Ppt.cStarts++;
            pCpu->Ppt.fStarted              = true;
            pCpu->Ppt.fStarting             = true;
            pCpu->Ppt.iTickHistorization    = 0;
            pCpu->Ppt.uTimerHz              = uHz;
            pCpu->Ppt.cNsInterval           = cNsInterval = RT_NS_1SEC / uHz;
            if (cNsInterval < GVMMHOSTCPU_PPT_HIST_INTERVAL_NS)
                pCpu->Ppt.cTicksHistoriziationInterval = (  GVMMHOSTCPU_PPT_HIST_INTERVAL_NS
                                                          + GVMMHOSTCPU_PPT_HIST_INTERVAL_NS / 2 - 1)
                                                       / cNsInterval;
            else
                pCpu->Ppt.cTicksHistoriziationInterval = 1;
        }

        RTSpinlockReleaseNoInts(pCpu->Ppt.hSpinlock);

        if (cNsInterval)
        {
            RTTimerChangeInterval(pCpu->Ppt.pTimer, cNsInterval);
            int rc = RTTimerStart(pCpu->Ppt.pTimer, cNsInterval);
            AssertRC(rc);

            RTSpinlockAcquire(pCpu->Ppt.hSpinlock);
            if (RT_FAILURE(rc))
                pCpu->Ppt.fStarted = false;
            pCpu->Ppt.fStarting = false;
            RTSpinlockReleaseNoInts(pCpu->Ppt.hSpinlock);
        }
    }
#else  /* !GVMM_SCHED_WITH_PPT */
    NOREF(idHostCpu); NOREF(uHz);
#endif /* !GVMM_SCHED_WITH_PPT */
}


/**
 * Retrieves the GVMM statistics visible to the caller.
 *
 * @returns VBox status code.
 *
 * @param   pStats      Where to put the statistics.
 * @param   pSession    The current session.
 * @param   pVM         The VM to obtain statistics for. Optional.
 */
GVMMR0DECL(int) GVMMR0QueryStatistics(PGVMMSTATS pStats, PSUPDRVSESSION pSession, PVM pVM)
{
    LogFlow(("GVMMR0QueryStatistics: pStats=%p pSession=%p pVM=%p\n", pStats, pSession, pVM));

    /*
     * Validate input.
     */
    AssertPtrReturn(pSession, VERR_INVALID_POINTER);
    AssertPtrReturn(pStats, VERR_INVALID_POINTER);
    pStats->cVMs = 0; /* (crash before taking the sem...) */

    /*
     * Take the lock and get the VM statistics.
     */
    PGVMM pGVMM;
    if (pVM)
    {
        PGVM pGVM;
        int rc = gvmmR0ByVM(pVM, &pGVM, &pGVMM, true /*fTakeUsedLock*/);
        if (RT_FAILURE(rc))
            return rc;
        pStats->SchedVM = pGVM->gvmm.s.StatsSched;
    }
    else
    {
        GVMM_GET_VALID_INSTANCE(pGVMM, VERR_GVMM_INSTANCE);
        memset(&pStats->SchedVM, 0, sizeof(pStats->SchedVM));

        int rc = gvmmR0UsedLock(pGVMM);
        AssertRCReturn(rc, rc);
    }

    /*
     * Enumerate the VMs and add the ones visible to the statistics.
     */
    pStats->cVMs = 0;
    pStats->cEMTs = 0;
    memset(&pStats->SchedSum, 0, sizeof(pStats->SchedSum));

    for (unsigned i = pGVMM->iUsedHead;
         i != NIL_GVM_HANDLE && i < RT_ELEMENTS(pGVMM->aHandles);
         i = pGVMM->aHandles[i].iNext)
    {
        PGVM pGVM = pGVMM->aHandles[i].pGVM;
        void *pvObj = pGVMM->aHandles[i].pvObj;
        if (    VALID_PTR(pvObj)
            &&  VALID_PTR(pGVM)
            &&  pGVM->u32Magic == GVM_MAGIC
            &&  RT_SUCCESS(SUPR0ObjVerifyAccess(pvObj, pSession, NULL)))
        {
            pStats->cVMs++;
            pStats->cEMTs += pGVM->cCpus;

            pStats->SchedSum.cHaltCalls        += pGVM->gvmm.s.StatsSched.cHaltCalls;
            pStats->SchedSum.cHaltBlocking     += pGVM->gvmm.s.StatsSched.cHaltBlocking;
            pStats->SchedSum.cHaltTimeouts     += pGVM->gvmm.s.StatsSched.cHaltTimeouts;
            pStats->SchedSum.cHaltNotBlocking  += pGVM->gvmm.s.StatsSched.cHaltNotBlocking;
            pStats->SchedSum.cHaltWakeUps      += pGVM->gvmm.s.StatsSched.cHaltWakeUps;

            pStats->SchedSum.cWakeUpCalls      += pGVM->gvmm.s.StatsSched.cWakeUpCalls;
            pStats->SchedSum.cWakeUpNotHalted  += pGVM->gvmm.s.StatsSched.cWakeUpNotHalted;
            pStats->SchedSum.cWakeUpWakeUps    += pGVM->gvmm.s.StatsSched.cWakeUpWakeUps;

            pStats->SchedSum.cPokeCalls        += pGVM->gvmm.s.StatsSched.cPokeCalls;
            pStats->SchedSum.cPokeNotBusy      += pGVM->gvmm.s.StatsSched.cPokeNotBusy;

            pStats->SchedSum.cPollCalls        += pGVM->gvmm.s.StatsSched.cPollCalls;
            pStats->SchedSum.cPollHalts        += pGVM->gvmm.s.StatsSched.cPollHalts;
            pStats->SchedSum.cPollWakeUps      += pGVM->gvmm.s.StatsSched.cPollWakeUps;
        }
    }

    /*
     * Copy out the per host CPU statistics.
     */
    uint32_t iDstCpu = 0;
    uint32_t cSrcCpus = pGVMM->cHostCpus;
    for (uint32_t iSrcCpu = 0; iSrcCpu < cSrcCpus; iSrcCpu++)
    {
        if (pGVMM->aHostCpus[iSrcCpu].idCpu != NIL_RTCPUID)
        {
            pStats->aHostCpus[iDstCpu].idCpu      = pGVMM->aHostCpus[iSrcCpu].idCpu;
            pStats->aHostCpus[iDstCpu].idxCpuSet  = pGVMM->aHostCpus[iSrcCpu].idxCpuSet;
#ifdef GVMM_SCHED_WITH_PPT
            pStats->aHostCpus[iDstCpu].uDesiredHz = pGVMM->aHostCpus[iSrcCpu].Ppt.uDesiredHz;
            pStats->aHostCpus[iDstCpu].uTimerHz   = pGVMM->aHostCpus[iSrcCpu].Ppt.uTimerHz;
            pStats->aHostCpus[iDstCpu].cChanges   = pGVMM->aHostCpus[iSrcCpu].Ppt.cChanges;
            pStats->aHostCpus[iDstCpu].cStarts    = pGVMM->aHostCpus[iSrcCpu].Ppt.cStarts;
#else
            pStats->aHostCpus[iDstCpu].uDesiredHz = 0;
            pStats->aHostCpus[iDstCpu].uTimerHz   = 0;
            pStats->aHostCpus[iDstCpu].cChanges   = 0;
            pStats->aHostCpus[iDstCpu].cStarts    = 0;
#endif
            iDstCpu++;
            if (iDstCpu >= RT_ELEMENTS(pStats->aHostCpus))
                break;
        }
    }
    pStats->cHostCpus = iDstCpu;

    gvmmR0UsedUnlock(pGVMM);

    return VINF_SUCCESS;
}


/**
 * VMMR0 request wrapper for GVMMR0QueryStatistics.
 *
 * @returns see GVMMR0QueryStatistics.
 * @param   pVM             Pointer to the VM. Optional.
 * @param   pReq            Pointer to the request packet.
 */
GVMMR0DECL(int) GVMMR0QueryStatisticsReq(PVM pVM, PGVMMQUERYSTATISTICSSREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(*pReq), ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    return GVMMR0QueryStatistics(&pReq->Stats, pReq->pSession, pVM);
}


/**
 * Resets the specified GVMM statistics.
 *
 * @returns VBox status code.
 *
 * @param   pStats      Which statistics to reset, that is, non-zero fields indicates which to reset.
 * @param   pSession    The current session.
 * @param   pVM         The VM to reset statistics for. Optional.
 */
GVMMR0DECL(int) GVMMR0ResetStatistics(PCGVMMSTATS pStats, PSUPDRVSESSION pSession, PVM pVM)
{
    LogFlow(("GVMMR0ResetStatistics: pStats=%p pSession=%p pVM=%p\n", pStats, pSession, pVM));

    /*
     * Validate input.
     */
    AssertPtrReturn(pSession, VERR_INVALID_POINTER);
    AssertPtrReturn(pStats, VERR_INVALID_POINTER);

    /*
     * Take the lock and get the VM statistics.
     */
    PGVMM pGVMM;
    if (pVM)
    {
        PGVM pGVM;
        int rc = gvmmR0ByVM(pVM, &pGVM, &pGVMM, true /*fTakeUsedLock*/);
        if (RT_FAILURE(rc))
            return rc;
#       define MAYBE_RESET_FIELD(field) \
            do { if (pStats->SchedVM. field ) { pGVM->gvmm.s.StatsSched. field = 0; } } while (0)
        MAYBE_RESET_FIELD(cHaltCalls);
        MAYBE_RESET_FIELD(cHaltBlocking);
        MAYBE_RESET_FIELD(cHaltTimeouts);
        MAYBE_RESET_FIELD(cHaltNotBlocking);
        MAYBE_RESET_FIELD(cHaltWakeUps);
        MAYBE_RESET_FIELD(cWakeUpCalls);
        MAYBE_RESET_FIELD(cWakeUpNotHalted);
        MAYBE_RESET_FIELD(cWakeUpWakeUps);
        MAYBE_RESET_FIELD(cPokeCalls);
        MAYBE_RESET_FIELD(cPokeNotBusy);
        MAYBE_RESET_FIELD(cPollCalls);
        MAYBE_RESET_FIELD(cPollHalts);
        MAYBE_RESET_FIELD(cPollWakeUps);
#       undef MAYBE_RESET_FIELD
    }
    else
    {
        GVMM_GET_VALID_INSTANCE(pGVMM, VERR_GVMM_INSTANCE);

        int rc = gvmmR0UsedLock(pGVMM);
        AssertRCReturn(rc, rc);
    }

    /*
     * Enumerate the VMs and add the ones visible to the statistics.
     */
    if (ASMMemIsAll8(&pStats->SchedSum, sizeof(pStats->SchedSum), 0))
    {
        for (unsigned i = pGVMM->iUsedHead;
             i != NIL_GVM_HANDLE && i < RT_ELEMENTS(pGVMM->aHandles);
             i = pGVMM->aHandles[i].iNext)
        {
            PGVM pGVM = pGVMM->aHandles[i].pGVM;
            void *pvObj = pGVMM->aHandles[i].pvObj;
            if (    VALID_PTR(pvObj)
                &&  VALID_PTR(pGVM)
                &&  pGVM->u32Magic == GVM_MAGIC
                &&  RT_SUCCESS(SUPR0ObjVerifyAccess(pvObj, pSession, NULL)))
            {
#               define MAYBE_RESET_FIELD(field) \
                    do { if (pStats->SchedSum. field ) { pGVM->gvmm.s.StatsSched. field = 0; } } while (0)
                MAYBE_RESET_FIELD(cHaltCalls);
                MAYBE_RESET_FIELD(cHaltBlocking);
                MAYBE_RESET_FIELD(cHaltTimeouts);
                MAYBE_RESET_FIELD(cHaltNotBlocking);
                MAYBE_RESET_FIELD(cHaltWakeUps);
                MAYBE_RESET_FIELD(cWakeUpCalls);
                MAYBE_RESET_FIELD(cWakeUpNotHalted);
                MAYBE_RESET_FIELD(cWakeUpWakeUps);
                MAYBE_RESET_FIELD(cPokeCalls);
                MAYBE_RESET_FIELD(cPokeNotBusy);
                MAYBE_RESET_FIELD(cPollCalls);
                MAYBE_RESET_FIELD(cPollHalts);
                MAYBE_RESET_FIELD(cPollWakeUps);
#               undef MAYBE_RESET_FIELD
            }
        }
    }

    gvmmR0UsedUnlock(pGVMM);

    return VINF_SUCCESS;
}


/**
 * VMMR0 request wrapper for GVMMR0ResetStatistics.
 *
 * @returns see GVMMR0ResetStatistics.
 * @param   pVM             Pointer to the VM. Optional.
 * @param   pReq            Pointer to the request packet.
 */
GVMMR0DECL(int) GVMMR0ResetStatisticsReq(PVM pVM, PGVMMRESETSTATISTICSSREQ pReq)
{
    /*
     * Validate input and pass it on.
     */
    AssertPtrReturn(pReq, VERR_INVALID_POINTER);
    AssertMsgReturn(pReq->Hdr.cbReq == sizeof(*pReq), ("%#x != %#x\n", pReq->Hdr.cbReq, sizeof(*pReq)), VERR_INVALID_PARAMETER);

    return GVMMR0ResetStatistics(&pReq->Stats, pReq->pSession, pVM);
}

