/* $Id: PDMNetShaper.cpp $ */
/** @file
 * PDM Network Shaper - Limit network traffic according to bandwidth group settings.
 */

/*
 * Copyright (C) 2011-2012 Oracle Corporation
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
#define LOG_GROUP LOG_GROUP_NET_SHAPER
#include "PDMInternal.h"
#include <VBox/vmm/pdm.h>
#include <VBox/vmm/mm.h>
#ifdef VBOX_WITH_REM
# include <VBox/vmm/rem.h>
#endif
#include <VBox/vmm/vm.h>
#include <VBox/vmm/uvm.h>
#include <VBox/err.h>

#include <VBox/log.h>
#include <iprt/asm.h>
#include <iprt/assert.h>
#include <iprt/thread.h>
#include <iprt/mem.h>
#include <iprt/critsect.h>
#include <iprt/tcp.h>
#include <iprt/path.h>
#include <iprt/string.h>

#include <VBox/vmm/pdmnetshaper.h>
#include <VBox/vmm/pdmnetshaperint.h>


/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/

/**
 * Network shaper data. One instance per VM.
 */
typedef struct PDMNETSHAPER
{
    /** Pointer to the VM. */
    PVM                      pVM;
    /** Critical section protecting all members below. */
    RTCRITSECT               cs;
    /** Pending TX thread. */
    PPDMTHREAD               hTxThread;
    /** Pointer to the first bandwidth group. */
    PPDMNSBWGROUP            pBwGroupsHead;
} PDMNETSHAPER;






static PPDMNSBWGROUP pdmNsBwGroupFindById(PPDMNETSHAPER pShaper, const char *pcszId)
{
    PPDMNSBWGROUP pBwGroup = NULL;

    if (RT_VALID_PTR(pcszId))
    {
        int rc = RTCritSectEnter(&pShaper->cs); AssertRC(rc);

        pBwGroup = pShaper->pBwGroupsHead;
        while (   pBwGroup
               && RTStrCmp(pBwGroup->pszName, pcszId))
            pBwGroup = pBwGroup->pNext;

        rc = RTCritSectLeave(&pShaper->cs); AssertRC(rc);
    }

    return pBwGroup;
}


static void pdmNsBwGroupLink(PPDMNSBWGROUP pBwGroup)
{
    PPDMNETSHAPER pShaper = pBwGroup->pShaper;
    int rc = RTCritSectEnter(&pShaper->cs); AssertRC(rc);

    pBwGroup->pNext = pShaper->pBwGroupsHead;
    pShaper->pBwGroupsHead = pBwGroup;

    rc = RTCritSectLeave(&pShaper->cs); AssertRC(rc);
}


#if 0
static void pdmNsBwGroupUnlink(PPDMNSBWGROUP pBwGroup)
{
    PPDMNETSHAPER pShaper = pBwGroup->pShaper;
    int rc = RTCritSectEnter(&pShaper->cs); AssertRC(rc);

    if (pBwGroup == pShaper->pBwGroupsHead)
        pShaper->pBwGroupsHead = pBwGroup->pNext;
    else
    {
        PPDMNSBWGROUP pPrev = pShaper->pBwGroupsHead;
        while (   pPrev
               && pPrev->pNext != pBwGroup)
            pPrev = pPrev->pNext;

        AssertPtr(pPrev);
        pPrev->pNext = pBwGroup->pNext;
    }

    rc = RTCritSectLeave(&pShaper->cs); AssertRC(rc);
}
#endif


static void pdmNsBwGroupSetLimit(PPDMNSBWGROUP pBwGroup, uint64_t cbTransferPerSecMax)
{
    pBwGroup->cbTransferPerSecMax = cbTransferPerSecMax;
    pBwGroup->cbBucketSize        = RT_MAX(PDM_NETSHAPER_MIN_BUCKET_SIZE,
                                           cbTransferPerSecMax * PDM_NETSHAPER_MAX_LATENCY / 1000);
    LogFlowFunc(("New rate limit is %llu bytes per second, adjusted bucket size to %d bytes\n",
                 pBwGroup->cbTransferPerSecMax, pBwGroup->cbBucketSize));
}


static int pdmNsBwGroupCreate(PPDMNETSHAPER pShaper, const char *pcszBwGroup, uint64_t cbTransferPerSecMax)
{
    LogFlowFunc(("pShaper=%#p pcszBwGroup=%#p{%s} cbTransferPerSecMax=%llu\n",
                 pShaper, pcszBwGroup, pcszBwGroup, cbTransferPerSecMax));

    AssertPtrReturn(pShaper, VERR_INVALID_POINTER);
    AssertPtrReturn(pcszBwGroup, VERR_INVALID_POINTER);
    AssertReturn(*pcszBwGroup != '\0', VERR_INVALID_PARAMETER);

    int         rc;
    PPDMNSBWGROUP pBwGroup = pdmNsBwGroupFindById(pShaper, pcszBwGroup);
    if (!pBwGroup)
    {
        rc = MMHyperAlloc(pShaper->pVM, sizeof(PDMNSBWGROUP), 64,
                          MM_TAG_PDM_NET_SHAPER, (void **)&pBwGroup);
        if (RT_SUCCESS(rc))
        {
            rc = PDMR3CritSectInit(pShaper->pVM, &pBwGroup->cs, RT_SRC_POS, "BWGRP");
            if (RT_SUCCESS(rc))
            {
                pBwGroup->pszName = RTStrDup(pcszBwGroup);
                if (pBwGroup->pszName)
                {
                    pBwGroup->pShaper               = pShaper;
                    pBwGroup->cRefs                 = 0;

                    pdmNsBwGroupSetLimit(pBwGroup, cbTransferPerSecMax);

                    pBwGroup->cbTokensLast          = pBwGroup->cbBucketSize;
                    pBwGroup->tsUpdatedLast         = RTTimeSystemNanoTS();

                    LogFlowFunc(("pcszBwGroup={%s} cbBucketSize=%u\n",
                                 pcszBwGroup, pBwGroup->cbBucketSize));
                    pdmNsBwGroupLink(pBwGroup);
                    return VINF_SUCCESS;
                }
                PDMR3CritSectDelete(&pBwGroup->cs);
            }
            MMHyperFree(pShaper->pVM, pBwGroup);
        }
        else
            rc = VERR_NO_MEMORY;
    }
    else
        rc = VERR_ALREADY_EXISTS;

    LogFlowFunc(("returns rc=%Rrc\n", rc));
    return rc;
}


static void pdmNsBwGroupTerminate(PPDMNSBWGROUP pBwGroup)
{
    Assert(pBwGroup->cRefs == 0);
    if (PDMCritSectIsInitialized(&pBwGroup->cs))
        PDMR3CritSectDelete(&pBwGroup->cs);
}


DECLINLINE(void) pdmNsBwGroupRef(PPDMNSBWGROUP pBwGroup)
{
    ASMAtomicIncU32(&pBwGroup->cRefs);
}


DECLINLINE(void) pdmNsBwGroupUnref(PPDMNSBWGROUP pBwGroup)
{
    Assert(pBwGroup->cRefs > 0);
    ASMAtomicDecU32(&pBwGroup->cRefs);
}


static void pdmNsBwGroupXmitPending(PPDMNSBWGROUP pBwGroup)
{
    /*
     * We don't need to hold the bandwidth group lock to iterate over the list
     * of filters since the filters are removed while the shaper lock is being
     * held.
     */
    AssertPtr(pBwGroup);
    AssertPtr(pBwGroup->pShaper);
    Assert(RTCritSectIsOwner(&pBwGroup->pShaper->cs));
    //int rc = RTCritSectEnter(&pBwGroup->cs); AssertRC(rc);

    /* Check if the group is disabled. */
    if (pBwGroup->cbTransferPerSecMax == 0)
        return;

    PPDMNSFILTER pFilter = pBwGroup->pFiltersHead;
    while (pFilter)
    {
        bool fChoked = ASMAtomicXchgBool(&pFilter->fChoked, false);
        Log3((LOG_FN_FMT ": pFilter=%#p fChoked=%RTbool\n", __PRETTY_FUNCTION__, pFilter, fChoked));
        if (fChoked && pFilter->pIDrvNet)
        {
            LogFlowFunc(("Calling pfnXmitPending for pFilter=%#p\n", pFilter));
            pFilter->pIDrvNet->pfnXmitPending(pFilter->pIDrvNet);
        }

        pFilter = pFilter->pNext;
    }

    //rc = RTCritSectLeave(&pBwGroup->cs); AssertRC(rc);
}


static void pdmNsFilterLink(PPDMNSFILTER pFilter)
{
    PPDMNSBWGROUP pBwGroup = pFilter->pBwGroupR3;
    int rc = PDMCritSectEnter(&pBwGroup->cs, VERR_SEM_BUSY); AssertRC(rc);

    pFilter->pNext = pBwGroup->pFiltersHead;
    pBwGroup->pFiltersHead = pFilter;

    rc = PDMCritSectLeave(&pBwGroup->cs); AssertRC(rc);
}


static void pdmNsFilterUnlink(PPDMNSFILTER pFilter)
{
    PPDMNSBWGROUP pBwGroup = pFilter->pBwGroupR3;
    /*
     * We need to make sure we hold the shaper lock since pdmNsBwGroupXmitPending()
     * does not hold the bandwidth group lock while iterating over the list
     * of group's filters.
     */
    AssertPtr(pBwGroup);
    AssertPtr(pBwGroup->pShaper);
    Assert(RTCritSectIsOwner(&pBwGroup->pShaper->cs));
    int rc = PDMCritSectEnter(&pBwGroup->cs, VERR_SEM_BUSY); AssertRC(rc);

    if (pFilter == pBwGroup->pFiltersHead)
        pBwGroup->pFiltersHead = pFilter->pNext;
    else
    {
        PPDMNSFILTER pPrev = pBwGroup->pFiltersHead;
        while (   pPrev
               && pPrev->pNext != pFilter)
            pPrev = pPrev->pNext;

        AssertPtr(pPrev);
        pPrev->pNext = pFilter->pNext;
    }

    rc = PDMCritSectLeave(&pBwGroup->cs); AssertRC(rc);
}


VMMR3DECL(int) PDMR3NsAttach(PVM pVM, PPDMDRVINS pDrvIns, const char *pcszBwGroup,
                             PPDMNSFILTER pFilter)
{
    VM_ASSERT_EMT(pVM);
    AssertPtrReturn(pFilter, VERR_INVALID_POINTER);
    AssertReturn(pFilter->pBwGroupR3 == NULL, VERR_ALREADY_EXISTS);


    PUVM pUVM = pVM->pUVM;
    PPDMNETSHAPER pShaper = pUVM->pdm.s.pNetShaper;

    PPDMNSBWGROUP pBwGroupOld = NULL;
    PPDMNSBWGROUP pBwGroupNew = NULL;

    int rc = RTCritSectEnter(&pShaper->cs); AssertRC(rc);
    if (RT_SUCCESS(rc))
    {
        if (pcszBwGroup)
        {
            pBwGroupNew = pdmNsBwGroupFindById(pShaper, pcszBwGroup);
            if (pBwGroupNew)
                pdmNsBwGroupRef(pBwGroupNew);
            else
                rc = VERR_NOT_FOUND;
        }

        if (RT_SUCCESS(rc))
        {
            pBwGroupOld = ASMAtomicXchgPtrT(&pFilter->pBwGroupR3, pBwGroupNew, PPDMNSBWGROUP);
            ASMAtomicWritePtr(&pFilter->pBwGroupR0, MMHyperR3ToR0(pVM, pBwGroupNew));
            if (pBwGroupOld)
                pdmNsBwGroupUnref(pBwGroupOld);
            pdmNsFilterLink(pFilter);
        }
        int rc2 = RTCritSectLeave(&pShaper->cs); AssertRC(rc2);
    }

    return rc;
}


VMMR3DECL(int) PDMR3NsDetach(PVM pVM, PPDMDRVINS pDrvIns, PPDMNSFILTER pFilter)
{
    VM_ASSERT_EMT(pVM);
    AssertPtrReturn(pFilter, VERR_INVALID_POINTER);
    AssertPtrReturn(pFilter->pBwGroupR3, VERR_INVALID_POINTER);

    PUVM pUVM = pVM->pUVM;
    PPDMNETSHAPER pShaper = pUVM->pdm.s.pNetShaper;

    int rc = RTCritSectEnter(&pShaper->cs); AssertRC(rc);
    if (RT_SUCCESS(rc))
    {
        pdmNsFilterUnlink(pFilter);
        PPDMNSBWGROUP pBwGroup = NULL;
        pBwGroup = ASMAtomicXchgPtrT(&pFilter->pBwGroupR3, NULL, PPDMNSBWGROUP);
        if (pBwGroup)
            pdmNsBwGroupUnref(pBwGroup);
        int rc2 = RTCritSectLeave(&pShaper->cs); AssertRC(rc2);
    }
    return rc;
}


VMMR3DECL(bool) PDMR3NsAllocateBandwidth(PPDMNSFILTER pFilter, size_t cbTransfer)
{
    return pdmNsAllocateBandwidth(pFilter, cbTransfer);
}


VMMR3DECL(int) PDMR3NsBwGroupSetLimit(PVM pVM, const char *pcszBwGroup, uint64_t cbTransferPerSecMax)
{
    PUVM pUVM = pVM->pUVM;
    PPDMNETSHAPER pShaper = pUVM->pdm.s.pNetShaper;

    int rc = RTCritSectEnter(&pShaper->cs); AssertRC(rc);
    if (RT_SUCCESS(rc))
    {
        PPDMNSBWGROUP pBwGroup = pdmNsBwGroupFindById(pShaper, pcszBwGroup);
        if (pBwGroup)
        {
            rc = PDMCritSectEnter(&pBwGroup->cs, VERR_SEM_BUSY); AssertRC(rc);
            pdmNsBwGroupSetLimit(pBwGroup, cbTransferPerSecMax);
            /* Drop extra tokens */
            if (pBwGroup->cbTokensLast > pBwGroup->cbBucketSize)
                pBwGroup->cbTokensLast = pBwGroup->cbBucketSize;
            rc = PDMCritSectLeave(&pBwGroup->cs); AssertRC(rc);
        }
        rc = RTCritSectLeave(&pShaper->cs); AssertRC(rc);
    }
    return rc;
}


/**
 * I/O thread for pending TX.
 *
 * @returns VINF_SUCCESS (ignored).
 * @param   pVM         Pointer to the VM.
 * @param   pThread     The PDM thread data.
 */
static DECLCALLBACK(int) pdmR3NsTxThread(PVM pVM, PPDMTHREAD pThread)
{
    PPDMNETSHAPER pShaper = (PPDMNETSHAPER)pThread->pvUser;
    LogFlow(("pdmR3NsTxThread: pShaper=%p\n", pShaper));
    while (pThread->enmState == PDMTHREADSTATE_RUNNING)
    {
        RTThreadSleep(PDM_NETSHAPER_MAX_LATENCY);
        /* Go over all bandwidth groups/filters calling pfnXmitPending */
        int rc = RTCritSectEnter(&pShaper->cs); AssertRC(rc);
        PPDMNSBWGROUP pBwGroup = pShaper->pBwGroupsHead;
        while (pBwGroup)
        {
            pdmNsBwGroupXmitPending(pBwGroup);
            pBwGroup = pBwGroup->pNext;
        }
        rc = RTCritSectLeave(&pShaper->cs); AssertRC(rc);
    }
    return VINF_SUCCESS;
}


/**
 * @copydoc FNPDMTHREADWAKEUPINT
 */
static DECLCALLBACK(int) pdmR3NsTxWakeUp(PVM pVM, PPDMTHREAD pThread)
{
    PPDMNETSHAPER pShaper = (PPDMNETSHAPER)pThread->pvUser;
    LogFlow(("pdmR3NsTxWakeUp: pShaper=%p\n", pShaper));
    /* Nothing to do */
    return VINF_SUCCESS;
}


/**
 * Terminate the network shaper.
 *
 * @returns VBox error code.
 * @param   pVM  Pointer to VM.
 *
 * @remarks This method destroys all bandwidth group objects.
 */
int pdmR3NetShaperTerm(PVM pVM)
{
    PUVM pUVM = pVM->pUVM;
    AssertPtrReturn(pUVM, VERR_INVALID_POINTER);
    PPDMNETSHAPER pShaper = pUVM->pdm.s.pNetShaper;
    AssertPtrReturn(pShaper, VERR_INVALID_POINTER);

    /* Destroy the bandwidth managers. */
    PPDMNSBWGROUP pBwGroup = pShaper->pBwGroupsHead;
    while (pBwGroup)
    {
        PPDMNSBWGROUP pFree = pBwGroup;
        pBwGroup = pBwGroup->pNext;
        pdmNsBwGroupTerminate(pFree);
        MMHyperFree(pVM, pFree);
    }

    RTCritSectDelete(&pShaper->cs);
    return VINF_SUCCESS;
}


/**
 * Initialize the network shaper.
 *
 * @returns VBox status code
 * @param   pVM Pointer to the VM.
 */
int pdmR3NetShaperInit(PVM pVM)
{
    LogFlowFunc((": pVM=%p\n", pVM));

    VM_ASSERT_EMT(pVM);

    PPDMNETSHAPER pNetShaper = NULL;

    int rc = MMR3HeapAllocZEx(pVM, MM_TAG_PDM_NET_SHAPER,
                              sizeof(PDMNETSHAPER),
                              (void **)&pNetShaper);
    if (RT_SUCCESS(rc))
    {
        PCFGMNODE pCfgRoot      = CFGMR3GetRoot(pVM);
        PCFGMNODE pCfgNetShaper = CFGMR3GetChild(CFGMR3GetChild(pCfgRoot, "PDM"), "NetworkShaper");

        pNetShaper->pVM = pVM;
        rc = RTCritSectInit(&pNetShaper->cs);
        if (RT_SUCCESS(rc))
        {
            /* Create all bandwidth groups. */
            PCFGMNODE pCfgBwGrp = CFGMR3GetChild(pCfgNetShaper, "BwGroups");

            if (pCfgBwGrp)
            {
                for (PCFGMNODE pCur = CFGMR3GetFirstChild(pCfgBwGrp); pCur; pCur = CFGMR3GetNextChild(pCur))
                {
                    uint64_t cbMax;
                    size_t cbName = CFGMR3GetNameLen(pCur) + 1;
                    char *pszBwGrpId = (char *)RTMemAllocZ(cbName);

                    if (!pszBwGrpId)
                    {
                        rc = VERR_NO_MEMORY;
                        break;
                    }

                    rc = CFGMR3GetName(pCur, pszBwGrpId, cbName);
                    AssertRC(rc);

                    if (RT_SUCCESS(rc))
                        rc = CFGMR3QueryU64(pCur, "Max", &cbMax);
                    if (RT_SUCCESS(rc))
                        rc = pdmNsBwGroupCreate(pNetShaper, pszBwGrpId, cbMax);

                    RTMemFree(pszBwGrpId);

                    if (RT_FAILURE(rc))
                        break;
                }
            }

            if (RT_SUCCESS(rc))
            {
                PUVM pUVM = pVM->pUVM;
                AssertMsg(!pUVM->pdm.s.pNetShaper, ("Network shaper was already initialized\n"));

                char szDesc[64];
                static unsigned s_iThread;

                RTStrPrintf(szDesc, sizeof(szDesc), "PDMNsTx-%d", ++s_iThread);
                rc = PDMR3ThreadCreate(pVM, &pNetShaper->hTxThread, pNetShaper,
                                       pdmR3NsTxThread, pdmR3NsTxWakeUp, 0,
                                       RTTHREADTYPE_IO, szDesc);
                if (RT_SUCCESS(rc))
                {
                    pUVM->pdm.s.pNetShaper = pNetShaper;
                    return VINF_SUCCESS;
                }
            }

            RTCritSectDelete(&pNetShaper->cs);
        }
        MMR3HeapFree(pNetShaper);
    }

    LogFlowFunc((": pVM=%p rc=%Rrc\n", pVM, rc));
    return rc;
}

