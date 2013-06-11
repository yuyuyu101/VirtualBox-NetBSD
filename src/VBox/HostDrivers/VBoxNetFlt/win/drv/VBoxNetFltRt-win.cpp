/* $Id: VBoxNetFltRt-win.cpp $ */
/** @file
 * VBoxNetFltRt-win.cpp - Bridged Networking Driver, Windows Specific Code.
 * NetFlt Runtime
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
#include "VBoxNetFltCmn-win.h"
#include <VBox/intnetinline.h>
#include <iprt/thread.h>

/** represents the job element of the job queue
 * see comments for VBOXNETFLT_JOB_QUEUE */
typedef struct VBOXNETFLT_JOB
{
    /** link in the job queue */
    LIST_ENTRY ListEntry;
    /** job function to be executed */
    PFNVBOXNETFLT_JOB_ROUTINE pfnRoutine;
    /** parameter to be passed to the job function */
    PVOID pContext;
    /** event that will be fired on job completion */
    KEVENT CompletionEvent;
    /** true if the job manager should use the completion even for completion indication, false-otherwise*/
    bool bUseCompletionEvent;
} VBOXNETFLT_JOB, *PVBOXNETFLT_JOB;

/**
 * represents the queue of jobs processed by the worker thread
 *
 * we use the thread to process tasks which are required to be done at passive level
 * our callbacks may be called at APC level by IntNet, there are some tasks that we can not create at APC,
 * e.g. thread creation. This is why we schedule such jobs to the worker thread working at passive level
 */
typedef struct VBOXNETFLT_JOB_QUEUE
{
    /* jobs */
    LIST_ENTRY Jobs;
    /* we are using ExInterlocked..List functions to access the jobs list */
    KSPIN_LOCK Lock;
    /** this event is used to initiate a job worker thread kill */
    KEVENT KillEvent;
    /** this event is used to notify a worker thread that jobs are added to the queue */
    KEVENT NotifyEvent;
    /** worker thread */
    PKTHREAD pThread;
} VBOXNETFLT_JOB_QUEUE, *PVBOXNETFLT_JOB_QUEUE;

typedef struct _CREATE_INSTANCE_CONTEXT
{
#ifndef VBOXNETADP
    PNDIS_STRING pOurName;
    PNDIS_STRING pBindToName;
#else
    NDIS_HANDLE hMiniportAdapter;
    NDIS_HANDLE hWrapperConfigurationContext;
#endif
    NDIS_STATUS Status;
}CREATE_INSTANCE_CONTEXT, *PCREATE_INSTANCE_CONTEXT;

/*contexts used for our jobs */
/* Attach context */
typedef struct _ATTACH_INFO
{
    PVBOXNETFLTINS pNetFltIf;
    PCREATE_INSTANCE_CONTEXT pCreateContext;
    bool fRediscovery;
    int Status;
}ATTACH_INFO, *PATTACH_INFO;

/* general worker context */
typedef struct _WORKER_INFO
{
    PVBOXNETFLTINS pNetFltIf;
    int Status;
}WORKER_INFO, *PWORKER_INFO;

/* idc initialization */
typedef struct _INIT_IDC_INFO
{
    VBOXNETFLT_JOB Job;
    bool bInitialized;
    volatile bool bStop;
    volatile int rc;
    KEVENT hCompletionEvent;
}INIT_IDC_INFO, *PINIT_IDC_INFO;


/** globals */
/** global job queue. some operations are required to be done at passive level, e.g. thread creation, adapter bind/unbind initiation,
 * while IntNet typically calls us APC_LEVEL, so we just create a system thread in our DriverEntry and enqueue the jobs to that thread */
static VBOXNETFLT_JOB_QUEUE g_VBoxJobQueue;
volatile static bool g_bVBoxIdcInitialized;
INIT_IDC_INFO g_VBoxInitIdcInfo;
/**
 * The (common) global data.
 */
static VBOXNETFLTGLOBALS g_VBoxNetFltGlobals;
/* win-specific global data */
VBOXNETFLTGLOBALS_WIN g_VBoxNetFltGlobalsWin = {0};

#define LIST_ENTRY_2_JOB(pListEntry) \
    ( (PVBOXNETFLT_JOB)((uint8_t *)(pListEntry) - RT_OFFSETOF(VBOXNETFLT_JOB, ListEntry)) )

static int vboxNetFltWinAttachToInterface(PVBOXNETFLTINS pThis, void * pContext, bool fRediscovery);
static int vboxNetFltWinConnectIt(PVBOXNETFLTINS pThis);
static int vboxNetFltWinTryFiniIdc();
static void vboxNetFltWinFiniNetFltBase();
static int vboxNetFltWinInitNetFltBase();
static int vboxNetFltWinFiniNetFlt();
static int vboxNetFltWinStartInitIdcProbing();
static int vboxNetFltWinStopInitIdcProbing();

/** makes the current thread to sleep for the given number of miliseconds */
DECLHIDDEN(void) vboxNetFltWinSleep(ULONG milis)
{
    RTThreadSleep(milis);
}

/** wait for the given device to be dereferenced */
DECLHIDDEN(void) vboxNetFltWinWaitDereference(PVBOXNETFLT_WINIF_DEVICE pState)
{
#ifdef DEBUG
    uint64_t StartNanoTS = RTTimeSystemNanoTS();
    uint64_t CurNanoTS;
#endif
    Assert(KeGetCurrentIrql() < DISPATCH_LEVEL);

    while (ASMAtomicUoReadU32((volatile uint32_t *)&pState->cReferences))
    {
        vboxNetFltWinSleep(2);
#ifdef DEBUG
        CurNanoTS = RTTimeSystemNanoTS();
        if (CurNanoTS - StartNanoTS > 20000000)
        {
            LogRel(("device not idle"));
            AssertFailed();
//            break;
        }
#endif
    }
}

/**
 * mem functions
 */
/* allocates and zeroes the nonpaged memory of a given size */
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinMemAlloc(PVOID* ppMemBuf, UINT cbLength)
{
#ifdef DEBUG_NETFLT_USE_EXALLOC
    *ppMemBuf = ExAllocatePoolWithTag(NonPagedPool, cbLength, VBOXNETFLT_MEM_TAG);
    if (*ppMemBuf)
    {
        NdisZeroMemory(*ppMemBuf, cbLength);
        return NDIS_STATUS_SUCCESS;
    }
    return NDIS_STATUS_FAILURE;
#else
    NDIS_STATUS fStatus = NdisAllocateMemoryWithTag(ppMemBuf, cbLength, VBOXNETFLT_MEM_TAG);
    if (fStatus == NDIS_STATUS_SUCCESS)
    {
        NdisZeroMemory(*ppMemBuf, cbLength);
    }
    return fStatus;
#endif
}

/* frees memory allocated with vboxNetFltWinMemAlloc */
DECLHIDDEN(void) vboxNetFltWinMemFree(PVOID pvMemBuf)
{
#ifdef DEBUG_NETFLT_USE_EXALLOC
    ExFreePool(pvMemBuf);
#else
    NdisFreeMemory(pvMemBuf, 0, 0);
#endif
}

#ifndef VBOXNETFLT_NO_PACKET_QUEUE

/* initializes packet info pool and allocates the cSize packet infos for the pool */
static NDIS_STATUS vboxNetFltWinPpAllocatePacketInfoPool(PVBOXNETFLT_PACKET_INFO_POOL pPool, UINT cSize)
{
    UINT cbBufSize = sizeof(PACKET_INFO)*cSize;
    PACKET_INFO * pPacketInfos;
    NDIS_STATUS fStatus;
    UINT i;

    Assert(cSize > 0);

    INIT_INTERLOCKED_PACKET_QUEUE(&pPool->Queue);

    fStatus = vboxNetFltWinMemAlloc((PVOID*)&pPacketInfos, cbBufSize);

    if (fStatus == NDIS_STATUS_SUCCESS)
    {
        PVBOXNETFLTPACKET_INFO pInfo;
        pPool->pBuffer = pPacketInfos;

        for (i = 0; i < cSize; i++)
        {
            pInfo = &pPacketInfos[i];
            vboxNetFltWinQuEnqueueTail(&pPool->Queue.Queue, pInfo);
            pInfo->pPool = pPool;
        }
    }
    else
    {
        AssertFailed();
    }

    return fStatus;
}

/* frees the packet info pool */
VOID vboxNetFltWinPpFreePacketInfoPool(PVBOXNETFLT_PACKET_INFO_POOL pPool)
{
    vboxNetFltWinMemFree(pPool->pBuffer);

    FINI_INTERLOCKED_PACKET_QUEUE(&pPool->Queue)
}

#endif

/**
 * copies one string to another. in case the destination string size is not enough to hold the complete source string
 * does nothing and returns NDIS_STATUS_RESOURCES .
 */
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinCopyString(PNDIS_STRING pDst, PNDIS_STRING pSrc)
{
    NDIS_STATUS Status = NDIS_STATUS_SUCCESS;

    if (pDst != pSrc)
    {
        if (pDst->MaximumLength < pSrc->Length)
        {
            AssertFailed();
            Status = NDIS_STATUS_RESOURCES;
        }
        else
        {
            pDst->Length = pSrc->Length;

            if (pDst->Buffer != pSrc->Buffer)
            {
                NdisMoveMemory(pDst->Buffer, pSrc->Buffer, pSrc->Length);
            }
        }
    }
    return Status;
}

/************************************************************************************
 * PINTNETSG pSG manipulation functions
 ************************************************************************************/

/* moves the contents of the given NDIS_BUFFER and all other buffers chained to it to the PINTNETSG
 * the PINTNETSG is expected to contain one segment whose bugger is large enough to maintain
 * the contents of the given NDIS_BUFFER and all other buffers chained to it */
static NDIS_STATUS vboxNetFltWinNdisBufferMoveToSG0(PNDIS_BUFFER pBuffer, PINTNETSG pSG)
{
    UINT cSegs = 0;
    PINTNETSEG paSeg;
    uint8_t * ptr;
    PVOID pVirtualAddress;
    UINT cbCurrentLength;
    NDIS_STATUS fStatus = NDIS_STATUS_SUCCESS;

    Assert(pSG->cSegsAlloc == 1);

    paSeg = pSG->aSegs;
    ptr = (uint8_t*)paSeg->pv;
    paSeg->cb = 0;
    paSeg->Phys = NIL_RTHCPHYS;
    pSG->cbTotal = 0;

    Assert(paSeg->pv);

    while (pBuffer)
    {
        NdisQueryBufferSafe(pBuffer, &pVirtualAddress, &cbCurrentLength, NormalPagePriority);

        if (!pVirtualAddress)
        {
            fStatus = NDIS_STATUS_FAILURE;
            break;
        }

        pSG->cbTotal += cbCurrentLength;
        paSeg->cb += cbCurrentLength;
        NdisMoveMemory(ptr, pVirtualAddress, cbCurrentLength);
        ptr += cbCurrentLength;

        NdisGetNextBuffer(pBuffer, &pBuffer);
    }

    if (fStatus == NDIS_STATUS_SUCCESS)
    {
        pSG->cSegsUsed = 1;
        Assert(pSG->cbTotal == paSeg->cb);
    }
    return fStatus;
}

/* converts the PNDIS_BUFFER to PINTNETSG by making the PINTNETSG segments to point to the memory buffers the
 * ndis buffer(s) point to (as opposed to vboxNetFltWinNdisBufferMoveToSG0 which copies the memory from ndis buffers(s) to PINTNETSG) */
static NDIS_STATUS vboxNetFltWinNdisBuffersToSG(PNDIS_BUFFER pBuffer, PINTNETSG pSG)
{
    UINT cSegs = 0;
    NDIS_STATUS Status = NDIS_STATUS_SUCCESS;
    PVOID pVirtualAddress;
    UINT cbCurrentLength;

    while (pBuffer)
    {
        NdisQueryBufferSafe(pBuffer, &pVirtualAddress, &cbCurrentLength, NormalPagePriority);

        if (!pVirtualAddress)
        {
            Status = NDIS_STATUS_FAILURE;
            break;
        }

        pSG->cbTotal += cbCurrentLength;
        pSG->aSegs[cSegs].cb = cbCurrentLength;
        pSG->aSegs[cSegs].pv = pVirtualAddress;
        pSG->aSegs[cSegs].Phys = NIL_RTHCPHYS;
        cSegs++;

        NdisGetNextBuffer(pBuffer, &pBuffer);
    }

    AssertFatal(cSegs <= pSG->cSegsAlloc);

    if (Status == NDIS_STATUS_SUCCESS)
    {
        pSG->cSegsUsed = cSegs;
    }

    return Status;
}

static void vboxNetFltWinDeleteSG(PINTNETSG pSG)
{
    vboxNetFltWinMemFree(pSG);
}

static PINTNETSG vboxNetFltWinCreateSG(uint32_t cSegs)
{
    PINTNETSG pSG;
    NTSTATUS Status = vboxNetFltWinMemAlloc((PVOID*)&pSG, RT_OFFSETOF(INTNETSG, aSegs[cSegs]));
    if (Status == STATUS_SUCCESS)
    {
        IntNetSgInitTempSegs(pSG, 0 /*cbTotal*/, cSegs, 0 /*cSegsUsed*/);
        return pSG;
    }

    return NULL;
}

/************************************************************************************
 * packet queue functions
 ************************************************************************************/
#ifndef VBOXNETFLT_NO_PACKET_QUEUE
#if !defined(VBOXNETADP)
static NDIS_STATUS vboxNetFltWinQuPostPacket(PVBOXNETFLTINS pNetFlt, PNDIS_PACKET pPacket, PINTNETSG pSG, uint32_t fFlags
# ifdef DEBUG_NETFLT_PACKETS
        , PNDIS_PACKET pTmpPacket
# endif
        )
{
    NDIS_STATUS Status;
    PNDIS_PACKET pMyPacket;
    bool bSrcHost = fFlags & PACKET_SRC_HOST;

    LogFlow(("posting packet back to driver stack..\n"));

    if (!pPacket)
    {
        /* INTNETSG was in the packet queue, create a new NdisPacket from INTNETSG*/
        pMyPacket = vboxNetFltWinNdisPacketFromSG(pNetFlt,
                pSG, /* PINTNETSG */
                pSG, /* PVOID pBufToFree */
                bSrcHost, /* bool bToWire */
                false); /* bool bCopyMemory */

        Assert(pMyPacket);

        NDIS_SET_PACKET_STATUS(pMyPacket, NDIS_STATUS_SUCCESS);

        DBG_CHECK_PACKET_AND_SG(pMyPacket, pSG);

#ifdef DEBUG_NETFLT_PACKETS
        Assert(pTmpPacket);

        DBG_CHECK_PACKET_AND_SG(pTmpPacket, pSG);

        DBG_CHECK_PACKETS(pTmpPacket, pMyPacket);
#endif

        LogFlow(("non-ndis packet info, packet created (%p)\n", pMyPacket));
    }
    else
    {
        /* NDIS_PACKET was in the packet queue */
        DBG_CHECK_PACKET_AND_SG(pPacket, pSG);

        if (!(fFlags & PACKET_MINE))
        {
            /* the packet is the one that was passed to us in send/receive callback
             * According to the DDK, we can not post it further,
             * instead we should allocate our own packet.
             * So, allocate our own packet (pMyPacket) and copy the packet info there */
            if (bSrcHost)
            {
                Status = vboxNetFltWinPrepareSendPacket(pNetFlt, pPacket, &pMyPacket/*, true*/);
                LogFlow(("packet from wire, packet created (%p)\n", pMyPacket));
            }
            else
            {
                Status = vboxNetFltWinPrepareRecvPacket(pNetFlt, pPacket, &pMyPacket, false);
                LogFlow(("packet from wire, packet created (%p)\n", pMyPacket));
            }
        }
        else
        {
            /* the packet enqueued is ours, simply assign pMyPacket and zero pPacket */
            pMyPacket = pPacket;
            pPacket = NULL;
        }
        Assert(pMyPacket);
    }

    if (pMyPacket)
    {
        /* we have successfully initialized our packet, post it to the host or to the wire */
        if (bSrcHost)
        {
#if defined(DEBUG_NETFLT_PACKETS) || !defined(VBOX_LOOPBACK_USEFLAGS)
            vboxNetFltWinLbPutSendPacket(pNetFlt, pMyPacket, false /* bFromIntNet */);
#endif
            NdisSend(&Status, pNetFlt->u.s.hBinding, pMyPacket);

            if (Status != NDIS_STATUS_PENDING)
            {
#if defined(DEBUG_NETFLT_PACKETS) || !defined(VBOX_LOOPBACK_USEFLAGS)
                /* the status is NOT pending, complete the packet */
                bool bTmp = vboxNetFltWinLbRemoveSendPacket(pNetFlt, pMyPacket);
                Assert(bTmp);
#endif
                if (pPacket)
                {
                    LogFlow(("status is not pending, completing packet (%p)\n", pPacket));

                    NdisIMCopySendCompletePerPacketInfo (pPacket, pMyPacket);

                    NdisFreePacket(pMyPacket);
                }
                else
                {
                    /* should never be here since the PINTNETSG is stored only when the underlying miniport
                     * indicates NDIS_STATUS_RESOURCES, we should never have this when processing
                     * the "from-host" packets */
                    AssertFailed();
                    LogFlow(("status is not pending, freeing myPacket (%p)\n", pMyPacket));
                    vboxNetFltWinFreeSGNdisPacket(pMyPacket, false);
                }
            }
        }
        else
        {
            NdisMIndicateReceivePacket(pNetFlt->u.s.hMiniport, &pMyPacket, 1);

            Status = NDIS_STATUS_PENDING;
            /* the packet receive completion is always indicated via MiniportReturnPacket */
        }
    }
    else
    {
        /*we failed to create our packet */
        AssertFailed();
        Status = NDIS_STATUS_FAILURE;
    }

    return Status;
}
#endif

static bool vboxNetFltWinQuProcessInfo(PVBOXNETFLTINS pNetFltIf, PPACKET_QUEUE_WORKER pWorker, PVOID pvPacket, const UINT fFlags)
#else
DECLHIDDEN(bool) vboxNetFltWinPostIntnet(PVBOXNETFLTINS pNetFltIf, PVOID pvPacket, const UINT fFlags)
#endif
{
    PNDIS_PACKET pPacket = NULL;
    PINTNETSG pSG = NULL;
    NDIS_STATUS Status;
#ifndef VBOXNETADP
    bool bSrcHost;
    bool bDropIt;
# ifndef VBOXNETFLT_NO_PACKET_QUEUE
    bool bPending;
# endif
#endif
#ifdef VBOXNETFLT_NO_PACKET_QUEUE
    bool bDeleteSG = false;
#endif
#ifdef DEBUG_NETFLT_PACKETS
    /* packet used for matching */
    PNDIS_PACKET pTmpPacket = NULL;
#endif

#ifndef VBOXNETADP
    bSrcHost = (fFlags & VBOXNETFLT_PACKET_SRC_HOST) != 0;
#endif

    /* we first need to obtain the INTNETSG to be passed to intnet */

    /* the queue may contain two "types" of packets:
     * the NDIS_PACKET and the INTNETSG.
     * I.e. on send/receive we typically enqueue the NDIS_PACKET passed to us by ndis,
     * however in case our ProtocolReceive is called or the packet's status is set to NDIS_STSTUS_RESOURCES
     * in ProtocolReceivePacket, we must return the packet immediately on ProtocolReceive*** exit
     * In this case we allocate the INTNETSG, copy the ndis packet data there and enqueue it.
     * In this case the packet info flags has the VBOXNETFLT_PACKET_SG fag set
     *
     * Besides that the NDIS_PACKET contained in the queue could be either the one passed to us in our send/receive callback
     * or the one created by us. The latter is possible in case our ProtocolReceive callback is called and we call NdisTransferData
     * in this case we need to allocate the packet the data to be transferred to.
     * If the enqueued packet is the one allocated by us the VBOXNETFLT_PACKET_MINE flag is set
     * */
    if ((fFlags & VBOXNETFLT_PACKET_SG) == 0)
    {
        /* we have NDIS_PACKET enqueued, we need to convert it to INTNETSG to be passed to intnet */
        PNDIS_BUFFER pCurrentBuffer = NULL;
        UINT cBufferCount;
        UINT uBytesCopied = 0;
        UINT cbPacketLength;

        pPacket = (PNDIS_PACKET)pvPacket;

        LogFlow(("ndis packet info, packet (%p)\n", pPacket));

        LogFlow(("preparing pSG"));
        NdisQueryPacket(pPacket, NULL, &cBufferCount, &pCurrentBuffer, &cbPacketLength);
        Assert(cBufferCount);

#ifdef VBOXNETFLT_NO_PACKET_QUEUE
        pSG = vboxNetFltWinCreateSG(cBufferCount);
#else
        /* we can not allocate the INTNETSG on stack since in this case we may get stack overflow
         * somewhere outside of our driver (3 pages of system thread stack does not seem to be enough)
         *
         * since we have a "serialized" packet processing, i.e. all packets are being processed and passed
         * to intnet by this thread, we just use one previously allocated INTNETSG which is stored in PVBOXNETFLTINS */
        pSG = pWorker->pSG;

        if (cBufferCount > pSG->cSegsAlloc)
        {
            pSG = vboxNetFltWinCreateSG(cBufferCount + 2);
            if (pSG)
            {
                vboxNetFltWinDeleteSG(pWorker->pSG);
                pWorker->pSG = pSG;
            }
            else
            {
                LogRel(("Failed to reallocate the pSG\n"));
            }
        }
#endif

        if (pSG)
        {
#ifdef VBOXNETFLT_NO_PACKET_QUEUE
            bDeleteSG = true;
#endif
            /* reinitialize */
            IntNetSgInitTempSegs(pSG, 0 /*cbTotal*/, pSG->cSegsAlloc, 0 /*cSegsUsed*/);

            /* convert the ndis buffers to INTNETSG */
            Status = vboxNetFltWinNdisBuffersToSG(pCurrentBuffer, pSG);
            if (Status != NDIS_STATUS_SUCCESS)
            {
                pSG = NULL;
            }
            else
            {
                DBG_CHECK_PACKET_AND_SG(pPacket, pSG);
            }
        }
    }
    else
    {
        /* we have the INTNETSG enqueued. (see the above comment explaining why/when this may happen)
         * just use the INTNETSG to pass it to intnet */
#ifndef VBOXNETADP
        /* the PINTNETSG is stored only when the underlying miniport
         * indicates NDIS_STATUS_RESOURCES, we should never have this when processing
         * the "from-host" packedts */
        Assert(!bSrcHost);
#endif
        pSG = (PINTNETSG)pvPacket;

        LogFlow(("not ndis packet info, pSG (%p)\n", pSG));
    }

#ifdef DEBUG_NETFLT_PACKETS
    if (!pPacket && !pTmpPacket)
    {
        /* create tmp packet that woud be used for matching */
        pTmpPacket = vboxNetFltWinNdisPacketFromSG(pNetFltIf,
                    pSG, /* PINTNETSG */
                    pSG, /* PVOID pBufToFree */
                    bSrcHost, /* bool bToWire */
                    true); /* bool bCopyMemory */

        NDIS_SET_PACKET_STATUS(pTmpPacket, NDIS_STATUS_SUCCESS);

        DBG_CHECK_PACKET_AND_SG(pTmpPacket, pSG);

        Assert(pTmpPacket);
    }
#endif
    do
    {
#ifndef VBOXNETADP
        /* the pSG was successfully initialized, post it to the netFlt*/
        bDropIt = pSG ? pNetFltIf->pSwitchPort->pfnRecv(pNetFltIf->pSwitchPort, NULL /* pvIf */, pSG,
                    bSrcHost ? INTNETTRUNKDIR_HOST : INTNETTRUNKDIR_WIRE
                            )
              : false;
#else
        if (pSG)
        {
            pNetFltIf->pSwitchPort->pfnRecv(pNetFltIf->pSwitchPort, NULL /* pvIf */, pSG, INTNETTRUNKDIR_HOST);
            STATISTIC_INCREASE(pNetFltIf->u.s.WinIf.cTxSuccess);
        }
        else
        {
            STATISTIC_INCREASE(pNetFltIf->u.s.WinIf.cTxError);
        }
#endif

#ifndef VBOXNETFLT_NO_PACKET_QUEUE

# if !defined(VBOXNETADP)
        if (!bDropIt)
        {
            Status = vboxNetFltWinQuPostPacket(pNetFltIf, pPacket, pSG, fFlags
#  ifdef DEBUG_NETFLT_PACKETS
                               , pTmpPacket
#  endif
            );

            if (Status == NDIS_STATUS_PENDING)
            {
                /* we will process packet completion in the completion routine */
                bPending = true;
                break;
            }
        }
        else
# endif
        {
            Status = NDIS_STATUS_SUCCESS;
        }

        /* drop it */
        if (pPacket)
        {
            if (!(fFlags & PACKET_MINE))
            {
# if !defined(VBOXNETADP)
                /* complete the packets */
                if (fFlags & PACKET_SRC_HOST)
                {
# endif
/*                    NDIS_SET_PACKET_STATUS(pPacket, Status); */
                    NdisMSendComplete(pNetFltIf->u.s.hMiniport, pPacket, Status);
# if !defined(VBOXNETADP)
                }
                else
                {
# endif
# ifndef VBOXNETADP
                    NdisReturnPackets(&pPacket, 1);
# endif
# if !defined(VBOXNETADP)
                }
# endif
            }
            else
            {
                Assert(!(fFlags & PACKET_SRC_HOST));
                vboxNetFltWinFreeSGNdisPacket(pPacket, true);
            }
        }
        else
        {
            Assert(pSG);
            vboxNetFltWinMemFree(pSG);
        }
# ifndef VBOXNETADP
        bPending = false;
# endif
    } while (0);

#ifdef DEBUG_NETFLT_PACKETS
    if (pTmpPacket)
    {
        vboxNetFltWinFreeSGNdisPacket(pTmpPacket, true);
    }
#endif

#ifndef VBOXNETADP
    return bPending;
#else
    return false;
#endif
#else /* #ifdef VBOXNETFLT_NO_PACKET_QUEUE */
    } while (0);

    if (bDeleteSG)
        vboxNetFltWinMemFree(pSG);

# ifndef VBOXNETADP
    return bDropIt;
# else
    return true;
# endif
#endif
}
#ifndef VBOXNETFLT_NO_PACKET_QUEUE
/*
 * thread start function for the thread which processes the packets enqueued in our send and receive callbacks called by ndis
 *
 * ndis calls us at DISPATCH_LEVEL, while IntNet is using kernel functions which require Irql<DISPATCH_LEVEL
 * this is why we can not immediately post packets to IntNet from our sen/receive callbacks
 * instead we put the incoming packets to the queue and maintain the system thread running at passive level
 * which processes the queue and posts the packets to IntNet, and further to the host or to the wire.
 */
static VOID vboxNetFltWinQuPacketQueueWorkerThreadProc(PVBOXNETFLTINS pNetFltIf)
{
    bool fResume = true;
    NTSTATUS fStatus;
    PPACKET_QUEUE_WORKER pWorker = &pNetFltIf->u.s.PacketQueueWorker;

    PVOID apEvents[] = {
        (PVOID)&pWorker->KillEvent,
        (PVOID)&pWorker->NotifyEvent
    };

    while (fResume)
    {
        uint32_t cNumProcessed;
        uint32_t cNumPostedToHostWire;

        fStatus = KeWaitForMultipleObjects(RT_ELEMENTS(apEvents), apEvents, WaitAny, Executive, KernelMode, FALSE, NULL, NULL);
        if (!NT_SUCCESS(fStatus) || fStatus == STATUS_WAIT_0)
        {
            /* "kill" event was set
             * will process queued packets and exit */
            fResume = false;
        }

        LogFlow(("processing vboxNetFltWinQuPacketQueueWorkerThreadProc\n"));

        cNumProcessed = 0;
        cNumPostedToHostWire = 0;

        do
        {
            PVBOXNETFLTPACKET_INFO pInfo;

#ifdef DEBUG_NETFLT_PACKETS
            /* packet used for matching */
            PNDIS_PACKET pTmpPacket = NULL;
#endif

            /*TODO: FIXME: !!! the better approach for performance would be to dequeue all packets at once
             * and then go through all dequeued packets
             * the same should be done for enqueue !!! */
            pInfo = vboxNetFltWinQuInterlockedDequeueHead(&pWorker->PacketQueue);

            if (!pInfo)
            {
                break;
            }

            LogFlow(("found info (0x%p)\n", pInfo));

            if (vboxNetFltWinQuProcessInfo(pNetFltIf, pWorker, pInfo->pPacket, pInfo->fFlags))
            {
                cNumPostedToHostWire++;
            }

            vboxNetFltWinPpFreePacketInfo(pInfo);

            cNumProcessed++;
        } while (TRUE);

        if (cNumProcessed)
        {
            vboxNetFltWinDecReferenceNetFlt(pNetFltIf, cNumProcessed);

            Assert(cNumProcessed >= cNumPostedToHostWire);

            if (cNumProcessed != cNumPostedToHostWire)
            {
                vboxNetFltWinDecReferenceWinIf(pNetFltIf, cNumProcessed - cNumPostedToHostWire);
            }
        }
    }

    PsTerminateSystemThread(STATUS_SUCCESS);
}
#endif
/**
 * thread start function for the job processing thread
 *
 * see comments for PVBOXNETFLT_JOB_QUEUE
 */
static VOID vboxNetFltWinJobWorkerThreadProc(PVBOXNETFLT_JOB_QUEUE pQueue)
{
    bool fResume = true;
    NTSTATUS Status;

    PVOID apEvents[] = {
        (PVOID)&pQueue->KillEvent,
        (PVOID)&pQueue->NotifyEvent,
    };

    do
    {
        Status = KeWaitForMultipleObjects(RT_ELEMENTS(apEvents), apEvents, WaitAny, Executive, KernelMode, FALSE, NULL, NULL);
        Assert(NT_SUCCESS(Status));
        if (!NT_SUCCESS(Status) || Status == STATUS_WAIT_0)
        {
            /* will process queued jobs and exit */
            Assert(Status == STATUS_WAIT_0);
            fResume = false;
        }

        do
        {
            PLIST_ENTRY pJobEntry = ExInterlockedRemoveHeadList(&pQueue->Jobs, &pQueue->Lock);
            PVBOXNETFLT_JOB pJob;

            if (!pJobEntry)
                break;

            pJob = LIST_ENTRY_2_JOB(pJobEntry);

            Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);
            pJob->pfnRoutine(pJob->pContext);
            Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);

            if (pJob->bUseCompletionEvent)
            {
                KeSetEvent(&pJob->CompletionEvent, 1, FALSE);
            }
        } while (TRUE);
    } while (fResume);

    Assert(Status == STATUS_WAIT_0);

    PsTerminateSystemThread(STATUS_SUCCESS);
}

/**
 * enqueues the job to the job queue to be processed by the job worker thread
 * see comments for PVBOXNETFLT_JOB_QUEUE
 */
static VOID vboxNetFltWinJobEnqueueJob(PVBOXNETFLT_JOB_QUEUE pQueue, PVBOXNETFLT_JOB pJob, bool bEnqueueHead)
{
    if (bEnqueueHead)
    {
        ExInterlockedInsertHeadList(&pQueue->Jobs, &pJob->ListEntry, &pQueue->Lock);
    }
    else
    {
        ExInterlockedInsertTailList(&pQueue->Jobs, &pJob->ListEntry, &pQueue->Lock);
    }

    KeSetEvent(&pQueue->NotifyEvent, 1, FALSE);
}

DECLINLINE(VOID) vboxNetFltWinJobInit(PVBOXNETFLT_JOB pJob, PFNVBOXNETFLT_JOB_ROUTINE pfnRoutine, PVOID pContext, bool bUseEvent)
{
    pJob->pfnRoutine = pfnRoutine;
    pJob->pContext = pContext;
    pJob->bUseCompletionEvent = bUseEvent;
    if (bUseEvent)
        KeInitializeEvent(&pJob->CompletionEvent, NotificationEvent, FALSE);
}

/**
 * enqueues the job to the job queue to be processed by the job worker thread and
 * blocks until the job is done
 * see comments for PVBOXNETFLT_JOB_QUEUE
 */
static VOID vboxNetFltWinJobSynchExec(PVBOXNETFLT_JOB_QUEUE pQueue, PFNVBOXNETFLT_JOB_ROUTINE pfnRoutine, PVOID pContext)
{
    VBOXNETFLT_JOB Job;

    Assert(KeGetCurrentIrql() < DISPATCH_LEVEL);

    vboxNetFltWinJobInit(&Job, pfnRoutine, pContext, true);

    vboxNetFltWinJobEnqueueJob(pQueue, &Job, false);

    KeWaitForSingleObject(&Job.CompletionEvent, Executive, KernelMode, FALSE, NULL);
}

/**
 * enqueues the job to be processed by the job worker thread at passive level and
 * blocks until the job is done
 */
DECLHIDDEN(VOID) vboxNetFltWinJobSynchExecAtPassive(PFNVBOXNETFLT_JOB_ROUTINE pfnRoutine, PVOID pContext)
{
    vboxNetFltWinJobSynchExec(&g_VBoxJobQueue, pfnRoutine, pContext);
}

/**
 * helper function used for system thread creation
 */
static NTSTATUS vboxNetFltWinQuCreateSystemThread(PKTHREAD *ppThread, PKSTART_ROUTINE pfnStartRoutine, PVOID pvStartContext)
{
    OBJECT_ATTRIBUTES ObjectAttributes;
    Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);

    InitializeObjectAttributes(&ObjectAttributes, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);

    HANDLE hThread;
    NTSTATUS Status = PsCreateSystemThread(&hThread, THREAD_ALL_ACCESS, &ObjectAttributes, NULL, NULL, (PKSTART_ROUTINE)pfnStartRoutine, pvStartContext);
    Assert(Status == STATUS_SUCCESS);
    if (Status == STATUS_SUCCESS)
    {
        Status = ObReferenceObjectByHandle(hThread, THREAD_ALL_ACCESS, NULL, KernelMode, (PVOID*)ppThread, NULL);
        Assert(Status == STATUS_SUCCESS);
        ZwClose(hThread);
        if (Status == STATUS_SUCCESS)
        {
            return STATUS_SUCCESS;
        }

        /* @todo: how would we fail in this case ?*/
    }
    return Status;
}

/**
 * initialize the job queue
 * see comments for PVBOXNETFLT_JOB_QUEUE
 */
static NTSTATUS vboxNetFltWinJobInitQueue(PVBOXNETFLT_JOB_QUEUE pQueue)
{
    NTSTATUS fStatus;

    Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);

    NdisZeroMemory(pQueue, sizeof(VBOXNETFLT_JOB_QUEUE));

    KeInitializeEvent(&pQueue->KillEvent, NotificationEvent, FALSE);

    KeInitializeEvent(&pQueue->NotifyEvent, SynchronizationEvent, FALSE);

    InitializeListHead(&pQueue->Jobs);

    fStatus = vboxNetFltWinQuCreateSystemThread(&pQueue->pThread, (PKSTART_ROUTINE)vboxNetFltWinJobWorkerThreadProc, pQueue);
    if (fStatus != STATUS_SUCCESS)
    {
        pQueue->pThread = NULL;
    }
    else
    {
        Assert(pQueue->pThread);
    }

    return fStatus;
}

/**
 * deinitialize the job queue
 * see comments for PVBOXNETFLT_JOB_QUEUE
 */
static void vboxNetFltWinJobFiniQueue(PVBOXNETFLT_JOB_QUEUE pQueue)
{
    Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);

    if (pQueue->pThread)
    {
        KeSetEvent(&pQueue->KillEvent, 0, FALSE);

        KeWaitForSingleObject(pQueue->pThread, Executive,
                            KernelMode, FALSE, NULL);
    }
}

#ifndef VBOXNETFLT_NO_PACKET_QUEUE

/**
 * initializes the packet queue
 * */
DECLHIDDEN(NTSTATUS) vboxNetFltWinQuInitPacketQueue(PVBOXNETFLTINS pInstance)
{
    NTSTATUS Status;
    PPACKET_QUEUE_WORKER pWorker = &pInstance->u.s.PacketQueueWorker;

    AssertFatal(!pWorker->pSG);

    Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);

    KeInitializeEvent(&pWorker->KillEvent, NotificationEvent, FALSE);

    KeInitializeEvent(&pWorker->NotifyEvent, SynchronizationEvent, FALSE);

    INIT_INTERLOCKED_PACKET_QUEUE(&pWorker->PacketQueue);

    do
    {
    Status = vboxNetFltWinPpAllocatePacketInfoPool(&pWorker->PacketInfoPool, VBOXNETFLT_PACKET_INFO_POOL_SIZE);

    if (Status == NDIS_STATUS_SUCCESS)
    {
        pWorker->pSG = vboxNetFltWinCreateSG(PACKET_QUEUE_SG_SEGS_ALLOC);
        if (!pWorker->pSG)
        {
            Status = STATUS_INSUFFICIENT_RESOURCES;
            break;
        }

        Status = vboxNetFltWinQuCreateSystemThread(&pWorker->pThread, (PKSTART_ROUTINE)vboxNetFltWinQuPacketQueueWorkerThreadProc, pInstance);
        if (Status != STATUS_SUCCESS)
        {
            vboxNetFltWinPpFreePacketInfoPool(&pWorker->PacketInfoPool);
            vboxNetFltWinMemFree(pWorker->pSG);
            pWorker->pSG = NULL;
            break;
        }
    }

    } while (0);

    return Status;
}

/*
 * deletes the packet queue
 */
DECLHIDDEN(void) vboxNetFltWinQuFiniPacketQueue(PVBOXNETFLTINS pInstance)
{
    PINTNETSG pSG;
    PPACKET_QUEUE_WORKER pWorker = &pInstance->u.s.PacketQueueWorker;
    Assert(KeGetCurrentIrql() < DISPATCH_LEVEL);

    /* using the pPacketQueueSG as an indicator that the packet queue is initialized */
    RTSpinlockAcquire((pInstance)->hSpinlock);
    if (pWorker->pSG)
    {
        pSG = pWorker->pSG;
        pWorker->pSG = NULL;
        RTSpinlockReleaseNoInts((pInstance)->hSpinlock);
        KeSetEvent(&pWorker->KillEvent, 0, FALSE);

        KeWaitForSingleObject(pWorker->pThread, Executive,
                            KernelMode, FALSE, NULL);

        vboxNetFltWinPpFreePacketInfoPool(&pWorker->PacketInfoPool);

        vboxNetFltWinDeleteSG(pSG);

        FINI_INTERLOCKED_PACKET_QUEUE(&pWorker->PacketQueue);
    }
    else
    {
        RTSpinlockReleaseNoInts((pInstance)->hSpinlock);
    }
}

#endif

/*
 * creates the INTNETSG containing one segment pointing to the buffer of size cbBufSize
 * the INTNETSG created should be cleaned with vboxNetFltWinMemFree
 */
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinAllocSG(UINT cbPacket, PINTNETSG *ppSG)
{
    NDIS_STATUS Status;
    PINTNETSG pSG;

    /* allocation:
     * 1. SG_PACKET - with one aSegs pointing to
     * 2. buffer of cbPacket containing the entire packet */
    AssertCompileSizeAlignment(INTNETSG, sizeof(PVOID));
    Status = vboxNetFltWinMemAlloc((PVOID*)&pSG, cbPacket + sizeof(INTNETSG));
    if (Status == NDIS_STATUS_SUCCESS)
    {
        IntNetSgInitTemp(pSG, pSG + 1, cbPacket);
        LogFlow(("pSG created (%p)\n", pSG));
        *ppSG = pSG;
    }
    return Status;
}

#ifndef VBOXNETFLT_NO_PACKET_QUEUE
/**
 * put the packet info to the queue
 */
DECLINLINE(void) vboxNetFltWinQuEnqueueInfo(PVBOXNETFLTPACKET_QUEUE_WORKER pWorker, PVBOXNETFLTPACKET_INFO pInfo)
{
    vboxNetFltWinQuInterlockedEnqueueTail(&pWorker->PacketQueue, pInfo);

    KeSetEvent(&pWorker->NotifyEvent, IO_NETWORK_INCREMENT, FALSE);
}

/**
 * puts the packet to the queue
 *
 * @return NDIST_STATUS_SUCCESS iff the packet was enqueued successfully
 * and error status otherwise.
 * NOTE: that the success status does NOT mean that the packet processing is completed, but only that it was enqueued successfully
 * the packet can be returned to the caller protocol/moniport only in case the bReleasePacket was set to true (in this case the copy of the packet was enqueued)
 * or if vboxNetFltWinQuEnqueuePacket failed, i.e. the packet was NOT enqueued
 */
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinQuEnqueuePacket(PVBOXNETFLTINS pInstance, PVOID pPacket, const UINT fPacketFlags)
{
    PVBOXNETFLT_PACKET_INFO pInfo;
    PVBOXNETFLT_PACKET_QUEUE_WORKER pWorker = &pInstance->u.s.PacketQueueWorker;
    NDIS_STATUS fStatus = NDIS_STATUS_SUCCESS;

    do
    {
        if (fPacketFlags & PACKET_COPY)
        {
            PNDIS_BUFFER pBuffer = NULL;
            UINT cBufferCount;
            UINT uBytesCopied = 0;
            UINT cbPacketLength;
            PINTNETSG pSG;

            /* the packet is Ndis packet */
            Assert(!(fPacketFlags & PACKET_SG));
            Assert(!(fPacketFlags & PACKET_MINE));

            NdisQueryPacket((PNDIS_PACKET)pPacket,
                    NULL,
                    &cBufferCount,
                    &pBuffer,
                    &cbPacketLength);


            Assert(cBufferCount);

            fStatus = vboxNetFltWinAllocSG(cbPacketLength, &pSG);
            if (fStatus != NDIS_STATUS_SUCCESS)
            {
                AssertFailed();
                break;
            }

            pInfo = vboxNetFltWinPpAllocPacketInfo(&pWorker->PacketInfoPool);

            if (!pInfo)
            {
                AssertFailed();
                /* TODO: what status to set? */
                fStatus = NDIS_STATUS_FAILURE;
                vboxNetFltWinMemFree(pSG);
                break;
            }

            Assert(pInfo->pPool);

            /* the packet we are queueing is SG, add PACKET_SG to flags */
            SET_FLAGS_TO_INFO(pInfo, fPacketFlags | PACKET_SG);
            SET_PACKET_TO_INFO(pInfo, pSG);

            fStatus = vboxNetFltWinNdisBufferMoveToSG0(pBuffer, pSG);
            if (fStatus != NDIS_STATUS_SUCCESS)
            {
                AssertFailed();
                vboxNetFltWinPpFreePacketInfo(pInfo);
                vboxNetFltWinMemFree(pSG);
                break;
            }

            DBG_CHECK_PACKET_AND_SG((PNDIS_PACKET)pPacket, pSG);
        }
        else
        {
            pInfo = vboxNetFltWinPpAllocPacketInfo(&pWorker->PacketInfoPool);

            if (!pInfo)
            {
                AssertFailed();
                /* TODO: what status to set? */
                fStatus = NDIS_STATUS_FAILURE;
                break;
            }

            Assert(pInfo->pPool);

            SET_FLAGS_TO_INFO(pInfo, fPacketFlags);
            SET_PACKET_TO_INFO(pInfo, pPacket);
        }

        vboxNetFltWinQuEnqueueInfo(pWorker, pInfo);

    } while (0);

    return fStatus;
}
#endif


/*
 * netflt
 */
#ifndef VBOXNETADP
static NDIS_STATUS vboxNetFltWinSynchNdisRequest(PVBOXNETFLTINS pNetFlt, PNDIS_REQUEST pRequest)
{
    int rc;

    Assert(KeGetCurrentIrql() < DISPATCH_LEVEL);

    /* 1. serialize */
    rc = RTSemFastMutexRequest(pNetFlt->u.s.WinIf.hSynchRequestMutex); AssertRC(rc);
    if (RT_SUCCESS(rc))
    {
        NDIS_STATUS fRequestStatus = NDIS_STATUS_SUCCESS;

        /* 2. set pNetFlt->u.s.pSynchRequest */
        Assert(!pNetFlt->u.s.WinIf.pSynchRequest);
        pNetFlt->u.s.WinIf.pSynchRequest = pRequest;

        /* 3. call NdisRequest */
        NdisRequest(&fRequestStatus, pNetFlt->u.s.WinIf.hBinding, pRequest);

        if (fRequestStatus == NDIS_STATUS_PENDING)
        {
        /* 3.1 if pending wait and assign the resulting status */
            KeWaitForSingleObject(&pNetFlt->u.s.WinIf.hSynchCompletionEvent, Executive,
                            KernelMode, FALSE, NULL);

            fRequestStatus = pNetFlt->u.s.WinIf.SynchCompletionStatus;
        }

        /* 4. clear the pNetFlt->u.s.pSynchRequest */
        pNetFlt->u.s.WinIf.pSynchRequest = NULL;

        RTSemFastMutexRelease(pNetFlt->u.s.WinIf.hSynchRequestMutex); AssertRC(rc);
        return fRequestStatus;
    }
    return NDIS_STATUS_FAILURE;
}


DECLHIDDEN(NDIS_STATUS) vboxNetFltWinGetMacAddress(PVBOXNETFLTINS pNetFlt, PRTMAC pMac)
{
    NDIS_REQUEST request;
    NDIS_STATUS status;
    request.RequestType = NdisRequestQueryInformation;
    request.DATA.QUERY_INFORMATION.InformationBuffer = pMac;
    request.DATA.QUERY_INFORMATION.InformationBufferLength = sizeof(RTMAC);
    request.DATA.QUERY_INFORMATION.Oid = OID_802_3_CURRENT_ADDRESS;
    status = vboxNetFltWinSynchNdisRequest(pNetFlt, &request);
    if (status != NDIS_STATUS_SUCCESS)
    {
        /* TODO */
        AssertFailed();
    }

    return status;

}

DECLHIDDEN(NDIS_STATUS) vboxNetFltWinQueryPhysicalMedium(PVBOXNETFLTINS pNetFlt, NDIS_PHYSICAL_MEDIUM * pMedium)
{
    NDIS_REQUEST Request;
    NDIS_STATUS Status;
    Request.RequestType = NdisRequestQueryInformation;
    Request.DATA.QUERY_INFORMATION.InformationBuffer = pMedium;
    Request.DATA.QUERY_INFORMATION.InformationBufferLength = sizeof(NDIS_PHYSICAL_MEDIUM);
    Request.DATA.QUERY_INFORMATION.Oid = OID_GEN_PHYSICAL_MEDIUM;
    Status = vboxNetFltWinSynchNdisRequest(pNetFlt, &Request);
    if (Status != NDIS_STATUS_SUCCESS)
    {
        if (Status == NDIS_STATUS_NOT_SUPPORTED || Status == NDIS_STATUS_NOT_RECOGNIZED || Status == NDIS_STATUS_INVALID_OID)
        {
            Status = NDIS_STATUS_NOT_SUPPORTED;
        }
        else
        {
            LogRel(("OID_GEN_PHYSICAL_MEDIUM failed: Status (0x%x)", Status));
            AssertFailed();
        }
    }
    return Status;
}

DECLHIDDEN(bool) vboxNetFltWinIsPromiscuous(PVBOXNETFLTINS pNetFlt)
{
    /** @todo r=bird: This is too slow and is probably returning the wrong
     *        information. What we're interested in is whether someone besides us
     *        has put the interface into promiscuous mode. */
    NDIS_REQUEST request;
    NDIS_STATUS status;
    ULONG filter;
    Assert(VBOXNETFLT_PROMISCUOUS_SUPPORTED(pNetFlt));
    request.RequestType = NdisRequestQueryInformation;
    request.DATA.QUERY_INFORMATION.InformationBuffer = &filter;
    request.DATA.QUERY_INFORMATION.InformationBufferLength = sizeof(filter);
    request.DATA.QUERY_INFORMATION.Oid = OID_GEN_CURRENT_PACKET_FILTER;
    status = vboxNetFltWinSynchNdisRequest(pNetFlt, &request);
    if (status != NDIS_STATUS_SUCCESS)
    {
        /* TODO */
        AssertFailed();
        return false;
    }
    return (filter & NDIS_PACKET_TYPE_PROMISCUOUS) != 0;
}

DECLHIDDEN(NDIS_STATUS) vboxNetFltWinSetPromiscuous(PVBOXNETFLTINS pNetFlt, bool bYes)
{
/** @todo Need to report changes to the switch via:
 *  pThis->pSwitchPort->pfnReportPromiscuousMode(pThis->pSwitchPort, fPromisc);
 */
    Assert(VBOXNETFLT_PROMISCUOUS_SUPPORTED(pNetFlt));
    if (VBOXNETFLT_PROMISCUOUS_SUPPORTED(pNetFlt))
    {
        NDIS_REQUEST Request;
        NDIS_STATUS fStatus;
        ULONG fFilter;
        ULONG fExpectedFilter;
        ULONG fOurFilter;
        Request.RequestType = NdisRequestQueryInformation;
        Request.DATA.QUERY_INFORMATION.InformationBuffer = &fFilter;
        Request.DATA.QUERY_INFORMATION.InformationBufferLength = sizeof(fFilter);
        Request.DATA.QUERY_INFORMATION.Oid = OID_GEN_CURRENT_PACKET_FILTER;
        fStatus = vboxNetFltWinSynchNdisRequest(pNetFlt, &Request);
        if (fStatus != NDIS_STATUS_SUCCESS)
        {
            /* TODO: */
            AssertFailed();
            return fStatus;
        }

        if (!pNetFlt->u.s.WinIf.StateFlags.fUpperProtSetFilterInitialized)
        {
            /* the cache was not initialized yet, initiate it with the current filter value */
            pNetFlt->u.s.WinIf.fUpperProtocolSetFilter = fFilter;
            pNetFlt->u.s.WinIf.StateFlags.fUpperProtSetFilterInitialized = TRUE;
        }


        if (bYes)
        {
            fExpectedFilter = NDIS_PACKET_TYPE_PROMISCUOUS;
            fOurFilter = NDIS_PACKET_TYPE_PROMISCUOUS;
        }
        else
        {
            fExpectedFilter = pNetFlt->u.s.WinIf.fUpperProtocolSetFilter;
            fOurFilter = 0;
        }

        if (fExpectedFilter != fFilter)
        {
            Request.RequestType = NdisRequestSetInformation;
            Request.DATA.SET_INFORMATION.InformationBuffer = &fExpectedFilter;
            Request.DATA.SET_INFORMATION.InformationBufferLength = sizeof(fExpectedFilter);
            Request.DATA.SET_INFORMATION.Oid = OID_GEN_CURRENT_PACKET_FILTER;
            fStatus = vboxNetFltWinSynchNdisRequest(pNetFlt, &Request);
            if (fStatus != NDIS_STATUS_SUCCESS)
            {
                /* TODO */
                AssertFailed();
                return fStatus;
            }
        }
        pNetFlt->u.s.WinIf.fOurSetFilter = fOurFilter;
        return fStatus;
    }
    return NDIS_STATUS_NOT_SUPPORTED;
}
#else /* if defined VBOXNETADP */

/**
 *  Generates a new unique MAC address based on our vendor ID
 */
DECLHIDDEN(void) vboxNetFltWinGenerateMACAddress(RTMAC *pMac)
{
    /* temporary use a time info */
    uint64_t NanoTS = RTTimeSystemNanoTS();
    pMac->au8[0] = (uint8_t)((VBOXNETADP_VENDOR_ID >> 16) & 0xff);
    pMac->au8[1] = (uint8_t)((VBOXNETADP_VENDOR_ID >> 8) & 0xff);
    pMac->au8[2] = (uint8_t)(VBOXNETADP_VENDOR_ID & 0xff);
    pMac->au8[3] = (uint8_t)(NanoTS & 0xff0000);
    pMac->au16[2] = (uint16_t)(NanoTS & 0xffff);
}

DECLHIDDEN(int) vboxNetFltWinMAC2NdisString(RTMAC *pMac, PNDIS_STRING pNdisString)
{
    static const char s_achDigits[17] = "0123456789abcdef";
    PWSTR pString;

    /* validate parameters */
    AssertPtrReturn(pMac, VERR_INVALID_PARAMETER);
    AssertPtrReturn(pNdisString, VERR_INVALID_PARAMETER);
    AssertReturn(pNdisString->MaximumLength >= 13*sizeof(pNdisString->Buffer[0]), VERR_INVALID_PARAMETER);

    pString = pNdisString->Buffer;

    for (int i = 0; i < 6; i++)
    {
        uint8_t u8 = pMac->au8[i];
        pString[0] = s_achDigits[(u8 >>  4) & 0xf];
        pString[1] = s_achDigits[(u8/*>>0*/)& 0xf];
        pString += 2;
    }

    pNdisString->Length = 12*sizeof(pNdisString->Buffer[0]);

    *pString = L'\0';

    return VINF_SUCCESS;
}

static int vboxNetFltWinWchar2Int(WCHAR c, uint8_t * pv)
{
    if (c >= L'A' && c <= L'F')
    {
        *pv = (c - L'A') + 10;
    }
    else if (c >= L'a' && c <= L'f')
    {
        *pv = (c - L'a') + 10;
    }
    else if (c >= L'0' && c <= L'9')
    {
        *pv = (c - L'0');
    }
    else
    {
        return VERR_INVALID_PARAMETER;
    }
    return VINF_SUCCESS;
}

DECLHIDDEN(int) vboxNetFltWinMACFromNdisString(RTMAC *pMac, PNDIS_STRING pNdisString)
{
    int i, rc;
    PWSTR pString;

    /* validate parameters */
    AssertPtrReturn(pMac, VERR_INVALID_PARAMETER);
    AssertPtrReturn(pNdisString, VERR_INVALID_PARAMETER);
    AssertReturn(pNdisString->Length >= 12*sizeof(pNdisString->Buffer[0]), VERR_INVALID_PARAMETER);

    pString = pNdisString->Buffer;

    for (i = 0; i < 6; i++)
    {
        uint8_t v1, v2;
        rc = vboxNetFltWinWchar2Int(pString[0], &v1);
        if (RT_FAILURE(rc))
        {
            break;
        }

        rc = vboxNetFltWinWchar2Int(pString[1], &v2);
        if (RT_FAILURE(rc))
        {
            break;
        }

        pMac->au8[i] = (v1 << 4) | v2;

        pString += 2;
    }

    return rc;
}

#endif
/**
 * creates a NDIS_PACKET from the PINTNETSG
 */
DECLHIDDEN(PNDIS_PACKET) vboxNetFltWinNdisPacketFromSG(PVBOXNETFLTINS pNetFlt, PINTNETSG pSG, PVOID pBufToFree, bool bToWire, bool bCopyMemory)
{
    NDIS_STATUS fStatus;
    PNDIS_PACKET pPacket;

    Assert(pSG->aSegs[0].pv);
    Assert(pSG->cbTotal >= sizeof(VBOXNETFLT_PACKET_ETHEADER_SIZE));

/** @todo Hrmpf, how can we fix this assumption?  I fear this'll cause data
 *        corruption and maybe even BSODs ... */
    AssertReturn(pSG->cSegsUsed == 1 || bCopyMemory, NULL);

#ifdef VBOXNETADP
    NdisAllocatePacket(&fStatus, &pPacket, pNetFlt->u.s.WinIf.hRecvPacketPool);
#else
    NdisAllocatePacket(&fStatus, &pPacket, bToWire ? pNetFlt->u.s.WinIf.hSendPacketPool : pNetFlt->u.s.WinIf.hRecvPacketPool);
#endif
    if (fStatus == NDIS_STATUS_SUCCESS)
    {
        PNDIS_BUFFER pBuffer;
        PVOID pvMemBuf;

        /* @todo: generally we do not always need to zero-initialize the complete OOB data here, reinitialize only when/what we need,
         * however we DO need to reset the status for the packets we indicate via NdisMIndicateReceivePacket to avoid packet loss
         * in case the status contains NDIS_STATUS_RESOURCES */
        VBOXNETFLT_OOB_INIT(pPacket);

        if (bCopyMemory)
        {
            fStatus = vboxNetFltWinMemAlloc(&pvMemBuf, pSG->cbTotal);
            Assert(fStatus == NDIS_STATUS_SUCCESS);
            if (fStatus == NDIS_STATUS_SUCCESS)
                IntNetSgRead(pSG, pvMemBuf);
        }
        else
        {
            pvMemBuf = pSG->aSegs[0].pv;
        }
        if (fStatus == NDIS_STATUS_SUCCESS)
        {
#ifdef VBOXNETADP
            NdisAllocateBuffer(&fStatus, &pBuffer,
                    pNetFlt->u.s.WinIf.hRecvBufferPool,
                    pvMemBuf,
                    pSG->cbTotal);
#else
            NdisAllocateBuffer(&fStatus, &pBuffer,
                    bToWire ? pNetFlt->u.s.WinIf.hSendBufferPool : pNetFlt->u.s.WinIf.hRecvBufferPool,
                    pvMemBuf,
                    pSG->cbTotal);
#endif

            if (fStatus == NDIS_STATUS_SUCCESS)
            {
                NdisChainBufferAtBack(pPacket, pBuffer);

                if (bToWire)
                {
                    PVBOXNETFLT_PKTRSVD_PT pSendInfo = (PVBOXNETFLT_PKTRSVD_PT)pPacket->ProtocolReserved;
                    pSendInfo->pOrigPacket = NULL;
                    pSendInfo->pBufToFree = pBufToFree;
#ifdef VBOX_LOOPBACK_USEFLAGS
                    /* set "don't loopback" flags */
                    NdisGetPacketFlags(pPacket) = g_VBoxNetFltGlobalsWin.fPacketDontLoopBack;
#else
                    NdisGetPacketFlags(pPacket) = 0;
#endif
                }
                else
                {
                    PVBOXNETFLT_PKTRSVD_MP pRecvInfo = (PVBOXNETFLT_PKTRSVD_MP)pPacket->MiniportReserved;
                    pRecvInfo->pOrigPacket = NULL;
                    pRecvInfo->pBufToFree = pBufToFree;

                    /* we must set the header size on receive */
                    NDIS_SET_PACKET_HEADER_SIZE(pPacket, VBOXNETFLT_PACKET_ETHEADER_SIZE);
                    /* NdisAllocatePacket zero-initializes the OOB data,
                     * but keeps the packet flags, clean them here */
                    NdisGetPacketFlags(pPacket) = 0;
                }
                /* TODO: set out of bound data */
            }
            else
            {
                AssertFailed();
                if (bCopyMemory)
                {
                    vboxNetFltWinMemFree(pvMemBuf);
                }
                NdisFreePacket(pPacket);
                pPacket = NULL;
            }
        }
        else
        {
            AssertFailed();
            NdisFreePacket(pPacket);
            pPacket = NULL;
        }
    }
    else
    {
        pPacket = NULL;
    }

    DBG_CHECK_PACKET_AND_SG(pPacket, pSG);

    return pPacket;
}

/*
 * frees NDIS_PACKET created with vboxNetFltWinNdisPacketFromSG
 */
DECLHIDDEN(void) vboxNetFltWinFreeSGNdisPacket(PNDIS_PACKET pPacket, bool bFreeMem)
{
    UINT cBufCount;
    PNDIS_BUFFER pFirstBuffer;
    UINT uTotalPacketLength;
    PNDIS_BUFFER pBuffer;

    NdisQueryPacket(pPacket, NULL, &cBufCount, &pFirstBuffer, &uTotalPacketLength);

    Assert(cBufCount == 1);

    do
    {
        NdisUnchainBufferAtBack(pPacket, &pBuffer);
        if (pBuffer != NULL)
        {
            PVOID pvMemBuf;
            UINT cbLength;

            NdisQueryBufferSafe(pBuffer, &pvMemBuf, &cbLength, NormalPagePriority);
            NdisFreeBuffer(pBuffer);
            if (bFreeMem)
            {
                vboxNetFltWinMemFree(pvMemBuf);
            }
        }
        else
        {
            break;
        }
    } while (true);

    NdisFreePacket(pPacket);
}

#if !defined(VBOXNETADP)
static void vboxNetFltWinAssociateMiniportProtocol(PVBOXNETFLTGLOBALS_WIN pGlobalsWin)
{
    NdisIMAssociateMiniport(pGlobalsWin->Mp.hMiniport, pGlobalsWin->Pt.hProtocol);
}
#endif

/*
 * NetFlt driver unload function
 */
DECLHIDDEN(VOID) vboxNetFltWinUnload(IN PDRIVER_OBJECT DriverObject)
{
    int rc;
    UNREFERENCED_PARAMETER(DriverObject);

    LogFlow((__FUNCTION__" ==> DO (0x%x)\n", DriverObject));

    rc = vboxNetFltWinTryFiniIdc();
    if (RT_FAILURE(rc))
    {
        /* TODO: we can not prevent driver unload here */
        AssertFailed();

        Log((__FUNCTION__": vboxNetFltWinTryFiniIdc - failed, busy.\n"));
    }

    vboxNetFltWinJobFiniQueue(&g_VBoxJobQueue);
#ifndef VBOXNETADP
    vboxNetFltWinPtDeregister(&g_VBoxNetFltGlobalsWin.Pt);
#endif

    vboxNetFltWinMpDeregister(&g_VBoxNetFltGlobalsWin.Mp);

    LogFlow((__FUNCTION__" <== DO (0x%x)\n", DriverObject));

    vboxNetFltWinFiniNetFltBase();
    /* don't use logging or any RT after de-init */
}

RT_C_DECLS_BEGIN

NTSTATUS DriverEntry(IN PDRIVER_OBJECT DriverObject, IN PUNICODE_STRING RegistryPath);

RT_C_DECLS_END

NTSTATUS DriverEntry(IN PDRIVER_OBJECT DriverObject, IN PUNICODE_STRING RegistryPath)
{
    NDIS_STATUS Status = NDIS_STATUS_SUCCESS;
    int rc;

    /* the idc registration is initiated via IOCTL since our driver
     * can be loaded when the VBoxDrv is not in case we are a Ndis IM driver */
    rc = vboxNetFltWinInitNetFltBase();
    AssertRC(rc);
    if (RT_SUCCESS(rc))
    {
        Status = vboxNetFltWinJobInitQueue(&g_VBoxJobQueue);
        Assert(Status == STATUS_SUCCESS);
        if (Status == STATUS_SUCCESS)
        {
            ULONG MjVersion;
            ULONG MnVersion;

            /* note: we do it after we initialize the Job Queue */
            vboxNetFltWinStartInitIdcProbing();

            NdisZeroMemory(&g_VBoxNetFltGlobalsWin, sizeof (g_VBoxNetFltGlobalsWin));
            KeInitializeEvent(&g_VBoxNetFltGlobalsWin.SynchEvent, SynchronizationEvent, TRUE /* signalled*/);

            PsGetVersion(&MjVersion, &MnVersion,
              NULL, /* PULONG BuildNumber OPTIONAL */
              NULL /* PUNICODE_STRING CSDVersion OPTIONAL */
              );

            g_VBoxNetFltGlobalsWin.fPacketDontLoopBack = NDIS_FLAGS_DONT_LOOPBACK;

            if (MjVersion == 5 && MnVersion == 0)
            {
                /* this is Win2k, we don't support it actually, but just in case */
                g_VBoxNetFltGlobalsWin.fPacketDontLoopBack |= NDIS_FLAGS_SKIP_LOOPBACK_W2K;
            }

            g_VBoxNetFltGlobalsWin.fPacketIsLoopedBack = NDIS_FLAGS_IS_LOOPBACK_PACKET;

            Status = vboxNetFltWinMpRegister(&g_VBoxNetFltGlobalsWin.Mp, DriverObject, RegistryPath);
            Assert(Status == STATUS_SUCCESS);
            if (Status == NDIS_STATUS_SUCCESS)
            {
#ifndef VBOXNETADP
                Status = vboxNetFltWinPtRegister(&g_VBoxNetFltGlobalsWin.Pt, DriverObject, RegistryPath);
                Assert(Status == STATUS_SUCCESS);
                if (Status == NDIS_STATUS_SUCCESS)
#endif
                {
#ifndef VBOXNETADP
                    vboxNetFltWinAssociateMiniportProtocol(&g_VBoxNetFltGlobalsWin);
#endif
                    return STATUS_SUCCESS;

//#ifndef VBOXNETADP
//                vboxNetFltWinPtDeregister(&g_VBoxNetFltGlobalsWin.Pt);
//#endif
                }
                vboxNetFltWinMpDeregister(&g_VBoxNetFltGlobalsWin.Mp);
            }
            vboxNetFltWinJobFiniQueue(&g_VBoxJobQueue);
        }
        vboxNetFltWinFiniNetFlt();
    }
    else
    {
        Status = NDIS_STATUS_FAILURE;
    }

    return Status;
}

#ifndef VBOXNETADP
/**
 * creates and initializes the packet to be sent to the underlying miniport given a packet posted to our miniport edge
 * according to DDK docs we must create our own packet rather than posting the one passed to us
 */
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinPrepareSendPacket(PVBOXNETFLTINS pNetFlt, PNDIS_PACKET pPacket, PNDIS_PACKET *ppMyPacket)
{
    NDIS_STATUS Status;

    NdisAllocatePacket(&Status, ppMyPacket, pNetFlt->u.s.WinIf.hSendPacketPool);

    if (Status == NDIS_STATUS_SUCCESS)
    {
        PVBOXNETFLT_PKTRSVD_PT pSendInfo = (PVBOXNETFLT_PKTRSVD_PT)((*ppMyPacket)->ProtocolReserved);
        pSendInfo->pOrigPacket = pPacket;
        pSendInfo->pBufToFree = NULL;
        /* the rest will be filled on send */

        vboxNetFltWinCopyPacketInfoOnSend(*ppMyPacket, pPacket);

#ifdef VBOX_LOOPBACK_USEFLAGS
        NdisGetPacketFlags(*ppMyPacket) |= g_VBoxNetFltGlobalsWin.fPacketDontLoopBack;
#endif
    }
    else
    {
        *ppMyPacket = NULL;
    }

    return Status;
}

/**
 * creates and initializes the packet to be sent to the upperlying protocol given a packet indicated to our protocol edge
 * according to DDK docs we must create our own packet rather than posting the one passed to us
 */
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinPrepareRecvPacket(PVBOXNETFLTINS pNetFlt, PNDIS_PACKET pPacket, PNDIS_PACKET *ppMyPacket, bool bDpr)
{
    NDIS_STATUS Status;

    if (bDpr)
    {
        Assert(KeGetCurrentIrql() == DISPATCH_LEVEL);
        NdisDprAllocatePacket(&Status, ppMyPacket, pNetFlt->u.s.WinIf.hRecvPacketPool);
    }
    else
    {
        NdisAllocatePacket(&Status, ppMyPacket, pNetFlt->u.s.WinIf.hRecvPacketPool);
    }

    if (Status == NDIS_STATUS_SUCCESS)
    {
        PVBOXNETFLT_PKTRSVD_MP pRecvInfo = (PVBOXNETFLT_PKTRSVD_MP)((*ppMyPacket)->MiniportReserved);
        pRecvInfo->pOrigPacket = pPacket;
        pRecvInfo->pBufToFree = NULL;

        Status = vboxNetFltWinCopyPacketInfoOnRecv(*ppMyPacket, pPacket, false);
    }
    else
    {
        *ppMyPacket = NULL;
    }
    return Status;
}
#endif
/**
 * initializes the VBOXNETFLTINS (our context structure) and binds to the given adapter
 */
#if defined(VBOXNETADP)
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinPtInitBind(PVBOXNETFLTINS *ppNetFlt, NDIS_HANDLE hMiniportAdapter, PNDIS_STRING pBindToMiniportName /* actually this is our miniport name*/, NDIS_HANDLE hWrapperConfigurationContext)
#else
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinPtInitBind(PVBOXNETFLTINS *ppNetFlt, PNDIS_STRING pOurMiniportName, PNDIS_STRING pBindToMiniportName)
#endif
{
    NDIS_STATUS Status;
    do
    {
        ANSI_STRING AnsiString;
        int rc;
        PVBOXNETFLTINS pInstance;
        USHORT cbAnsiName = pBindToMiniportName->Length;/* the length is is bytes ; *2 ;RtlUnicodeStringToAnsiSize(pBindToMiniportName)*/
        CREATE_INSTANCE_CONTEXT Context;

# ifndef VBOXNETADP
        Context.pOurName = pOurMiniportName;
        Context.pBindToName = pBindToMiniportName;
# else
        Context.hMiniportAdapter = hMiniportAdapter;
        Context.hWrapperConfigurationContext = hWrapperConfigurationContext;
# endif
        Context.Status = NDIS_STATUS_SUCCESS;

        AnsiString.Buffer = 0; /* will be allocated by RtlUnicodeStringToAnsiString */
        AnsiString.Length = 0;
        AnsiString.MaximumLength = cbAnsiName;

        Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);

        Status = RtlUnicodeStringToAnsiString(&AnsiString, pBindToMiniportName, true);

        if (Status != STATUS_SUCCESS)
        {
            break;
        }

        rc = vboxNetFltSearchCreateInstance(&g_VBoxNetFltGlobals, AnsiString.Buffer, &pInstance, &Context);
        RtlFreeAnsiString(&AnsiString);
        if (RT_FAILURE(rc))
        {
            AssertFailed();
            Status = Context.Status != NDIS_STATUS_SUCCESS ? Context.Status : NDIS_STATUS_FAILURE;
            break;
        }

        Assert(pInstance);

        if (rc == VINF_ALREADY_INITIALIZED)
        {
            /* the case when our adapter was unbound while IntNet was connected to it */
            /* the instance remains valid until IntNet disconnects from it, we simply search and re-use it*/
            rc = vboxNetFltWinAttachToInterface(pInstance, &Context, true);
            if (RT_FAILURE(rc))
            {
                AssertFailed();
                Status = Context.Status != NDIS_STATUS_SUCCESS ? Context.Status : NDIS_STATUS_FAILURE;
                /* release netflt */
                vboxNetFltRelease(pInstance, false);

                break;
            }
        }

        *ppNetFlt = pInstance;

    } while (FALSE);

    return Status;
}
/*
 * deinitializes the VBOXNETFLTWIN
 */
DECLHIDDEN(VOID) vboxNetFltWinPtFiniWinIf(PVBOXNETFLTWIN pWinIf)
{
#ifndef VBOXNETADP
    int rc;
#endif

    LogFlow(("==>"__FUNCTION__" : pWinIf 0x%p\n", pWinIf));

    Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);
#ifndef VBOXNETADP
    if (pWinIf->MpDeviceName.Buffer)
    {
        vboxNetFltWinMemFree(pWinIf->MpDeviceName.Buffer);
    }

    FINI_INTERLOCKED_SINGLE_LIST(&pWinIf->TransferDataList);
# if defined(DEBUG_NETFLT_LOOPBACK) || !defined(VBOX_LOOPBACK_USEFLAGS)
    FINI_INTERLOCKED_SINGLE_LIST(&pWinIf->SendPacketQueue);
# endif
    NdisFreeBufferPool(pWinIf->hSendBufferPool);
    NdisFreePacketPool(pWinIf->hSendPacketPool);
    rc = RTSemFastMutexDestroy(pWinIf->hSynchRequestMutex);  AssertRC(rc);
#endif

    /* NOTE: NULL is a valid handle */
    NdisFreeBufferPool(pWinIf->hRecvBufferPool);
    NdisFreePacketPool(pWinIf->hRecvPacketPool);

    LogFlow(("<=="__FUNCTION__" : pWinIf 0x%p\n", pWinIf));
}

#ifndef VBOXNETADP
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinPtInitWinIf(PVBOXNETFLTWIN pWinIf, IN PNDIS_STRING pOurDeviceName)
#else
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinPtInitWinIf(PVBOXNETFLTWIN pWinIf)
#endif
{
    NDIS_STATUS Status = NDIS_STATUS_SUCCESS;
#ifndef VBOXNETADP
    int rc;
#endif
    BOOLEAN bCallFiniOnFail = FALSE;

    LogFlow(("==>"__FUNCTION__": pWinIf 0x%p\n", pWinIf));

    Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);

    NdisZeroMemory(pWinIf, sizeof (VBOXNETFLTWIN));
    NdisAllocatePacketPoolEx(&Status, &pWinIf->hRecvPacketPool,
                               VBOXNETFLT_PACKET_POOL_SIZE_NORMAL,
                               VBOXNETFLT_PACKET_POOL_SIZE_OVERFLOW,
                               PROTOCOL_RESERVED_SIZE_IN_PACKET);
    Assert(Status == NDIS_STATUS_SUCCESS);
    if (Status == NDIS_STATUS_SUCCESS)
    {
        /* NOTE: NULL is a valid handle !!! */
        NdisAllocateBufferPool(&Status, &pWinIf->hRecvBufferPool, VBOXNETFLT_BUFFER_POOL_SIZE_RX);
        Assert(Status == NDIS_STATUS_SUCCESS);
        if (Status == NDIS_STATUS_SUCCESS)
        {
            pWinIf->MpState.PowerState = NdisDeviceStateD3;
            vboxNetFltWinSetOpState(&pWinIf->MpState, kVBoxNetDevOpState_Deinitialized);
#ifndef VBOXNETADP
            pWinIf->PtState.PowerState = NdisDeviceStateD3;
            vboxNetFltWinSetOpState(&pWinIf->PtState, kVBoxNetDevOpState_Deinitialized);

            NdisAllocateBufferPool(&Status,
                    &pWinIf->hSendBufferPool,
                    VBOXNETFLT_BUFFER_POOL_SIZE_TX);
            Assert(Status == NDIS_STATUS_SUCCESS);
            if (Status == NDIS_STATUS_SUCCESS)
            {
                INIT_INTERLOCKED_SINGLE_LIST(&pWinIf->TransferDataList);

# if defined(DEBUG_NETFLT_LOOPBACK) || !defined(VBOX_LOOPBACK_USEFLAGS)
                INIT_INTERLOCKED_SINGLE_LIST(&pWinIf->SendPacketQueue);
# endif
                NdisInitializeEvent(&pWinIf->OpenCloseEvent);

                KeInitializeEvent(&pWinIf->hSynchCompletionEvent, SynchronizationEvent, FALSE);

                NdisInitializeEvent(&pWinIf->MpInitCompleteEvent);

                NdisAllocatePacketPoolEx(&Status, &pWinIf->hSendPacketPool,
                                           VBOXNETFLT_PACKET_POOL_SIZE_NORMAL,
                                           VBOXNETFLT_PACKET_POOL_SIZE_OVERFLOW,
                                           sizeof (PVBOXNETFLT_PKTRSVD_PT));
                Assert(Status == NDIS_STATUS_SUCCESS);
                if (Status == NDIS_STATUS_SUCCESS)
                {
                    rc = RTSemFastMutexCreate(&pWinIf->hSynchRequestMutex);
                    AssertRC(rc);
                    if (RT_SUCCESS(rc))
                    {
                        Status = vboxNetFltWinMemAlloc((PVOID*)&pWinIf->MpDeviceName.Buffer, pOurDeviceName->Length);
                        Assert(Status == NDIS_STATUS_SUCCESS);
                        if (Status == NDIS_STATUS_SUCCESS)
                        {
                            pWinIf->MpDeviceName.MaximumLength = pOurDeviceName->Length;
                            pWinIf->MpDeviceName.Length = 0;
                            Status = vboxNetFltWinCopyString(&pWinIf->MpDeviceName, pOurDeviceName);
#endif
                            return NDIS_STATUS_SUCCESS;
#ifndef VBOXNETADP
                            vboxNetFltWinMemFree(pWinIf->MpDeviceName.Buffer);
                        }
                        RTSemFastMutexDestroy(pWinIf->hSynchRequestMutex);
                    }
                    else
                    {
                        Status = NDIS_STATUS_FAILURE;
                    }
                    NdisFreePacketPool(pWinIf->hSendPacketPool);
                }
                NdisFreeBufferPool(pWinIf->hSendBufferPool);
            }
#endif
            NdisFreeBufferPool(pWinIf->hRecvBufferPool);
        }
        NdisFreePacketPool(pWinIf->hRecvPacketPool);
    }

    LogFlow(("<=="__FUNCTION__": pWinIf 0x%p, Status 0x%x\n", pWinIf, Status));

    return Status;
}

/**
 * match packets
 */
#define NEXT_LIST_ENTRY(_Entry) ((_Entry)->Flink)
#define PREV_LIST_ENTRY(_Entry) ((_Entry)->Blink)
#define FIRST_LIST_ENTRY NEXT_LIST_ENTRY
#define LAST_LIST_ENTRY PREV_LIST_ENTRY

#define MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))

#ifndef VBOXNETADP

#ifdef DEBUG_misha

RTMAC g_vboxNetFltWinVerifyMACBroadcast = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
RTMAC g_vboxNetFltWinVerifyMACGuest = {0x08, 0x00, 0x27, 0x01, 0x02, 0x03};

DECLHIDDEN(PRTNETETHERHDR) vboxNetFltWinGetEthHdr(PNDIS_PACKET pPacket)
{
    UINT cBufCount1;
    PNDIS_BUFFER pBuffer1;
    UINT uTotalPacketLength1;
    RTNETETHERHDR* pEth;
    UINT cbLength1 = 0;
    UINT i = 0;

    NdisQueryPacket(pPacket, NULL, &cBufCount1, &pBuffer1, &uTotalPacketLength1);

    Assert(pBuffer1);
    Assert(uTotalPacketLength1 >= VBOXNETFLT_PACKET_ETHEADER_SIZE);
    if (uTotalPacketLength1 < VBOXNETFLT_PACKET_ETHEADER_SIZE)
        return NULL;

    NdisQueryBufferSafe(pBuffer1, &pEth, &cbLength1, NormalPagePriority);
    Assert(cbLength1 >= VBOXNETFLT_PACKET_ETHEADER_SIZE);
    if (cbLength1 < VBOXNETFLT_PACKET_ETHEADER_SIZE)
        return NULL;

    return pEth;
}

DECLHIDDEN(PRTNETETHERHDR) vboxNetFltWinGetEthHdrSG(PINTNETSG pSG)
{
    Assert(pSG->cSegsUsed);
    Assert(pSG->cSegsAlloc >= pSG->cSegsUsed);
    Assert(pSG->aSegs[0].cb >= VBOXNETFLT_PACKET_ETHEADER_SIZE);

    if (!pSG->cSegsUsed)
        return NULL;

    if (pSG->aSegs[0].cb < VBOXNETFLT_PACKET_ETHEADER_SIZE)
        return NULL;

    return (PRTNETETHERHDR)pSG->aSegs[0].pv;
}

DECLHIDDEN(bool) vboxNetFltWinCheckMACs(PNDIS_PACKET pPacket, PRTMAC pDst, PRTMAC pSrc)
{
    PRTNETETHERHDR pHdr = vboxNetFltWinGetEthHdr(pPacket);
    Assert(pHdr);

    if (!pHdr)
        return false;

    if (pDst && memcmp(pDst, &pHdr->DstMac, sizeof(RTMAC)))
        return false;

    if (pSrc && memcmp(pSrc, &pHdr->SrcMac, sizeof(RTMAC)))
        return false;

    return true;
}

DECLHIDDEN(bool) vboxNetFltWinCheckMACsSG(PINTNETSG pSG, PRTMAC pDst, PRTMAC pSrc)
{
    PRTNETETHERHDR pHdr = vboxNetFltWinGetEthHdrSG(pSG);
    Assert(pHdr);

    if (!pHdr)
        return false;

    if (pDst && memcmp(pDst, &pHdr->DstMac, sizeof(RTMAC)))
        return false;

    if (pSrc && memcmp(pSrc, &pHdr->SrcMac, sizeof(RTMAC)))
        return false;

    return true;
}
#endif

# if !defined(VBOX_LOOPBACK_USEFLAGS) || defined(DEBUG_NETFLT_PACKETS)
/*
 * answers whether the two given packets match based on the packet length and the first cbMatch bytes of the packets
 * if cbMatch < 0 matches complete packets.
 */
DECLHIDDEN(bool) vboxNetFltWinMatchPackets(PNDIS_PACKET pPacket1, PNDIS_PACKET pPacket2, const INT cbMatch)
{
    UINT cBufCount1;
    PNDIS_BUFFER pBuffer1;
    UINT uTotalPacketLength1;
    uint8_t* pMemBuf1;
    UINT cbLength1 = 0;

    UINT cBufCount2;
    PNDIS_BUFFER pBuffer2;
    UINT uTotalPacketLength2;
    uint8_t* pMemBuf2;
    UINT cbLength2 = 0;
    bool bMatch = true;

#ifdef DEBUG_NETFLT_PACKETS
    bool bCompleteMatch = false;
#endif

    NdisQueryPacket(pPacket1, NULL, &cBufCount1, &pBuffer1, &uTotalPacketLength1);
    NdisQueryPacket(pPacket2, NULL, &cBufCount2, &pBuffer2, &uTotalPacketLength2);

    Assert(pBuffer1);
    Assert(pBuffer2);

    if (uTotalPacketLength1 != uTotalPacketLength2)
    {
        bMatch = false;
    }
    else
    {
        UINT ucbLength2Match = 0;
        UINT ucbMatch;
        if (cbMatch < 0 || (UINT)cbMatch > uTotalPacketLength1)
        {
            /* NOTE: assuming uTotalPacketLength1 == uTotalPacketLength2*/
            ucbMatch = uTotalPacketLength1;
#ifdef DEBUG_NETFLT_PACKETS
            bCompleteMatch = true;
#endif
        }
        else
        {
            ucbMatch = (UINT)cbMatch;
        }

        for (;;)
        {
            if (!cbLength1)
            {
                NdisQueryBufferSafe(pBuffer1, &pMemBuf1, &cbLength1, NormalPagePriority);
                NdisGetNextBuffer(pBuffer1, &pBuffer1);
            }
            else
            {
                Assert(pMemBuf1);
                Assert(ucbLength2Match);
                pMemBuf1 += ucbLength2Match;
            }

            if (!cbLength2)
            {
                NdisQueryBufferSafe(pBuffer2, &pMemBuf2, &cbLength2, NormalPagePriority);
                NdisGetNextBuffer(pBuffer2, &pBuffer2);
            }
            else
            {
                Assert(pMemBuf2);
                Assert(ucbLength2Match);
                pMemBuf2 += ucbLength2Match;
            }

            ucbLength2Match = MIN(ucbMatch, cbLength1);
            ucbLength2Match = MIN(ucbLength2Match, cbLength2);

            if (memcmp((PVOID*)pMemBuf1, (PVOID*)pMemBuf2, ucbLength2Match))
            {
                bMatch = false;
                break;
            }

            ucbMatch -= ucbLength2Match;
            if (!ucbMatch)
                break;

            cbLength1 -= ucbLength2Match;
            cbLength2 -= ucbLength2Match;
        }
    }

#ifdef DEBUG_NETFLT_PACKETS
    if (bMatch && !bCompleteMatch)
    {
        /* check that the packets fully match */
        DBG_CHECK_PACKETS(pPacket1, pPacket2);
    }
#endif

    return bMatch;
}

/*
 * answers whether the ndis packet and PINTNETSG match based on the packet length and the first cbMatch bytes of the packet and PINTNETSG
 * if cbMatch < 0 matches complete packets.
 */
DECLHIDDEN(bool) vboxNetFltWinMatchPacketAndSG(PNDIS_PACKET pPacket, PINTNETSG pSG, const INT cbMatch)
{
    UINT cBufCount1;
    PNDIS_BUFFER pBuffer1;
    UINT uTotalPacketLength1;
    uint8_t* pMemBuf1;
    UINT cbLength1 = 0;
    UINT uTotalPacketLength2 = pSG->cbTotal;
    uint8_t* pMemBuf2;
    UINT cbLength2 = 0;
    bool bMatch = true;
    bool bCompleteMatch = false;
    UINT i = 0;

    NdisQueryPacket(pPacket, NULL, &cBufCount1, &pBuffer1, &uTotalPacketLength1);

    Assert(pBuffer1);
    Assert(pSG->cSegsUsed);
    Assert(pSG->cSegsAlloc >= pSG->cSegsUsed);

    if (uTotalPacketLength1 != uTotalPacketLength2)
    {
        AssertFailed();
        bMatch = false;
    }
    else
    {
        UINT ucbLength2Match = 0;
        UINT ucbMatch;

        if (cbMatch < 0 || (UINT)cbMatch > uTotalPacketLength1)
        {
            /* NOTE: assuming uTotalPacketLength1 == uTotalPacketLength2*/
            ucbMatch = uTotalPacketLength1;
            bCompleteMatch = true;
        }
        else
        {
            ucbMatch = (UINT)cbMatch;
        }

        for (;;)
        {
            if (!cbLength1)
            {
                NdisQueryBufferSafe(pBuffer1, &pMemBuf1, &cbLength1, NormalPagePriority);
                NdisGetNextBuffer(pBuffer1, &pBuffer1);
            }
            else
            {
                Assert(pMemBuf1);
                Assert(ucbLength2Match);
                pMemBuf1 += ucbLength2Match;
            }

            if (!cbLength2)
            {
                Assert(i < pSG->cSegsUsed);
                pMemBuf2 = (uint8_t*)pSG->aSegs[i].pv;
                cbLength2 = pSG->aSegs[i].cb;
                i++;
            }
            else
            {
                Assert(pMemBuf2);
                Assert(ucbLength2Match);
                pMemBuf2 += ucbLength2Match;
            }

            ucbLength2Match = MIN(ucbMatch, cbLength1);
            ucbLength2Match = MIN(ucbLength2Match, cbLength2);

            if (memcmp((PVOID*)pMemBuf1, (PVOID*)pMemBuf2, ucbLength2Match))
            {
                bMatch = false;
                AssertFailed();
                break;
            }

            ucbMatch -= ucbLength2Match;
            if (!ucbMatch)
                break;

            cbLength1 -= ucbLength2Match;
            cbLength2 -= ucbLength2Match;
        }
    }

    if (bMatch && !bCompleteMatch)
    {
        /* check that the packets fully match */
        DBG_CHECK_PACKET_AND_SG(pPacket, pSG);
    }
    return bMatch;
}

#  if 0
/*
 * answers whether the two PINTNETSGs match based on the packet length and the first cbMatch bytes of the PINTNETSG
 * if cbMatch < 0 matches complete packets.
 */
static bool vboxNetFltWinMatchSGs(PINTNETSG pSG1, PINTNETSG pSG2, const INT cbMatch)
{
    UINT uTotalPacketLength1 = pSG1->cbTotal;
    PVOID pMemBuf1;
    UINT cbLength1 = 0;
    UINT i1 = 0;
    UINT uTotalPacketLength2 = pSG2->cbTotal;
    PVOID pMemBuf2;
    UINT cbLength2 = 0;

    bool bMatch = true;
    bool bCompleteMatch = false;
    UINT i2 = 0;

    Assert(pSG1->cSegsUsed);
    Assert(pSG2->cSegsUsed);
    Assert(pSG1->cSegsAlloc >= pSG1->cSegsUsed);
    Assert(pSG2->cSegsAlloc >= pSG2->cSegsUsed);

    if (uTotalPacketLength1 != uTotalPacketLength2)
    {
        AssertFailed();
        bMatch = false;
    }
    else
    {
        UINT ucbMatch;
        if (cbMatch < 0 || (UINT)cbMatch > uTotalPacketLength1)
        {
            /* NOTE: assuming uTotalPacketLength1 == uTotalPacketLength2*/
            ucbMatch = uTotalPacketLength1;
            bCompleteMatch = true;
        }
        else
        {
            ucbMatch = (UINT)cbMatch;
        }

        do
        {
            UINT ucbLength2Match;
            if (!cbLength1)
            {
                Assert(i1 < pSG1->cSegsUsed);
                pMemBuf1 = pSG1->aSegs[i1].pv;
                cbLength1 = pSG1->aSegs[i1].cb;
                i1++;
            }

            if (!cbLength2)
            {
                Assert(i2 < pSG2->cSegsUsed);
                pMemBuf2 = pSG2->aSegs[i2].pv;
                cbLength2 = pSG2->aSegs[i2].cb;
                i2++;
            }

            ucbLength2Match = MIN(ucbMatch, cbLength1);
            ucbLength2Match = MIN(ucbLength2Match, cbLength2);

            if (memcmp(pMemBuf1, pMemBuf2, ucbLength2Match))
            {
                bMatch = false;
                AssertFailed();
                break;
            }
            ucbMatch -= ucbLength2Match;
            cbLength1 -= ucbLength2Match;
            cbLength2 -= ucbLength2Match;
        } while (ucbMatch);
    }

    if (bMatch && !bCompleteMatch)
    {
        /* check that the packets fully match */
        DBG_CHECK_SGS(pSG1, pSG2);
    }
    return bMatch;
}
#  endif
# endif
#endif

static void vboxNetFltWinFiniNetFltBase()
{
    do
    {
        vboxNetFltDeleteGlobals(&g_VBoxNetFltGlobals);

        /*
         * Undo the work done during start (in reverse order).
         */
        memset(&g_VBoxNetFltGlobals, 0, sizeof(g_VBoxNetFltGlobals));

        RTLogDestroy(RTLogRelSetDefaultInstance(NULL));
        RTLogDestroy(RTLogSetDefaultInstance(NULL));

        RTR0Term();
    } while (0);
}

static int vboxNetFltWinTryFiniIdc()
{
    int rc;

    vboxNetFltWinStopInitIdcProbing();

    if (g_bVBoxIdcInitialized)
    {
        rc = vboxNetFltTryDeleteIdc(&g_VBoxNetFltGlobals);
        if (RT_SUCCESS(rc))
        {
            g_bVBoxIdcInitialized = false;
        }
    }
    else
    {
        rc = VINF_SUCCESS;
    }
    return rc;

}

static int vboxNetFltWinFiniNetFlt()
{
    int rc = vboxNetFltWinTryFiniIdc();
    if (RT_SUCCESS(rc))
    {
        vboxNetFltWinFiniNetFltBase();
    }
    return rc;
}

/**
 * base netflt initialization
 */
static int vboxNetFltWinInitNetFltBase()
{
    int rc;

    do
    {
        Assert(!g_bVBoxIdcInitialized);

        rc = RTR0Init(0);
        if (!RT_SUCCESS(rc))
        {
            break;
        }

        memset(&g_VBoxNetFltGlobals, 0, sizeof(g_VBoxNetFltGlobals));
        rc = vboxNetFltInitGlobals(&g_VBoxNetFltGlobals);
        if (!RT_SUCCESS(rc))
        {
            RTR0Term();
            break;
        }
    }while (0);

    return rc;
}

/**
 * initialize IDC
 */
static int vboxNetFltWinInitIdc()
{
    int rc;

    do
    {
        if (g_bVBoxIdcInitialized)
        {
            rc = VINF_ALREADY_INITIALIZED;
            break;
        }

        /*
         * connect to the support driver.
         *
         * This will call back vboxNetFltOsOpenSupDrv (and maybe vboxNetFltOsCloseSupDrv)
         * for establishing the connect to the support driver.
         */
        rc = vboxNetFltInitIdc(&g_VBoxNetFltGlobals);
        if (!RT_SUCCESS(rc))
        {
            break;
        }

        g_bVBoxIdcInitialized = true;
    } while (0);

    return rc;
}

static VOID vboxNetFltWinInitIdcProbingWorker(PVOID pvContext)
{
    PINIT_IDC_INFO pInitIdcInfo = (PINIT_IDC_INFO)pvContext;
    int rc = vboxNetFltWinInitIdc();
    if (RT_FAILURE(rc))
    {
        bool bInterupted = ASMAtomicUoReadBool(&pInitIdcInfo->bStop);
        if (!bInterupted)
        {
            RTThreadSleep(1000); /* 1 s */
            bInterupted = ASMAtomicUoReadBool(&pInitIdcInfo->bStop);
            if (!bInterupted)
            {
                vboxNetFltWinJobEnqueueJob(&g_VBoxJobQueue, &pInitIdcInfo->Job, false);
                return;
            }
        }

        /* it's interrupted */
        rc = VERR_INTERRUPTED;
    }

    ASMAtomicUoWriteS32(&pInitIdcInfo->rc, rc);
    KeSetEvent(&pInitIdcInfo->hCompletionEvent, 0, FALSE);
}

static int vboxNetFltWinStopInitIdcProbing()
{
    if (!g_VBoxInitIdcInfo.bInitialized)
        return VERR_INVALID_STATE;

    ASMAtomicUoWriteBool(&g_VBoxInitIdcInfo.bStop, true);
    KeWaitForSingleObject(&g_VBoxInitIdcInfo.hCompletionEvent, Executive, KernelMode, FALSE, NULL);

    return g_VBoxInitIdcInfo.rc;
}

static int vboxNetFltWinStartInitIdcProbing()
{
    Assert(!g_bVBoxIdcInitialized);
    KeInitializeEvent(&g_VBoxInitIdcInfo.hCompletionEvent, NotificationEvent, FALSE);
    g_VBoxInitIdcInfo.bStop = false;
    g_VBoxInitIdcInfo.bInitialized = true;
    vboxNetFltWinJobInit(&g_VBoxInitIdcInfo.Job, vboxNetFltWinInitIdcProbingWorker, &g_VBoxInitIdcInfo, false);
    vboxNetFltWinJobEnqueueJob(&g_VBoxJobQueue, &g_VBoxInitIdcInfo.Job, false);
    return VINF_SUCCESS;
}

static int vboxNetFltWinInitNetFlt()
{
    int rc;

    do
    {
        rc = vboxNetFltWinInitNetFltBase();
        if (RT_FAILURE(rc))
        {
            AssertFailed();
            break;
        }

        /*
         * connect to the support driver.
         *
         * This will call back vboxNetFltOsOpenSupDrv (and maybe vboxNetFltOsCloseSupDrv)
         * for establishing the connect to the support driver.
         */
        rc = vboxNetFltWinInitIdc();
        if (RT_FAILURE(rc))
        {
            AssertFailed();
            vboxNetFltWinFiniNetFltBase();
            break;
        }
    } while (0);

    return rc;
}

/* detach*/
static int vboxNetFltWinDeleteInstance(PVBOXNETFLTINS pThis)
{
    LogFlow(("vboxNetFltWinDeleteInstance: pThis=0x%p \n", pThis));

    Assert(KeGetCurrentIrql() < DISPATCH_LEVEL);
    Assert(pThis);
    Assert(pThis->fDisconnectedFromHost);
    Assert(!pThis->fRediscoveryPending);
    Assert(pThis->enmTrunkState != INTNETTRUNKIFSTATE_ACTIVE);
#ifndef VBOXNETADP
    Assert(pThis->u.s.WinIf.PtState.OpState == kVBoxNetDevOpState_Deinitialized);
    Assert(!pThis->u.s.WinIf.hBinding);
#endif
    Assert(pThis->u.s.WinIf.MpState.OpState == kVBoxNetDevOpState_Deinitialized);
#ifndef VBOXNETFLT_NO_PACKET_QUEUE
    Assert(!pThis->u.s.PacketQueueWorker.pSG);
#endif

    RTSemMutexDestroy(pThis->u.s.hWinIfMutex);

    vboxNetFltWinDrvDereference();

    return VINF_SUCCESS;
}

static NDIS_STATUS vboxNetFltWinDisconnectIt(PVBOXNETFLTINS pInstance)
{
#ifndef VBOXNETFLT_NO_PACKET_QUEUE
    vboxNetFltWinQuFiniPacketQueue(pInstance);
#endif
    return NDIS_STATUS_SUCCESS;
}

/* detach*/
DECLHIDDEN(NDIS_STATUS) vboxNetFltWinDetachFromInterface(PVBOXNETFLTINS pNetFlt, bool bOnUnbind)
{
    NDIS_STATUS Status;
    int rc;
    LogFlow((__FUNCTION__": pThis=%0xp\n", pNetFlt));

    Assert(KeGetCurrentIrql() < DISPATCH_LEVEL);
    Assert(pNetFlt);

    /* paranoia to ensure the instance is not removed while we're waiting on the mutex
     * in case ndis does something unpredictable, e.g. calls our miniport halt independently
     * from protocol unbind and concurrently with it*/
    vboxNetFltRetain(pNetFlt, false);

    rc = RTSemMutexRequest(pNetFlt->u.s.hWinIfMutex, RT_INDEFINITE_WAIT);
    if (RT_SUCCESS(rc))
    {
        Assert(vboxNetFltWinGetWinIfState(pNetFlt) == kVBoxWinIfState_Connected);
        Assert(vboxNetFltWinGetOpState(&pNetFlt->u.s.WinIf.MpState) == kVBoxNetDevOpState_Initialized);
#ifndef VBOXNETADP
        Assert(vboxNetFltWinGetOpState(&pNetFlt->u.s.WinIf.PtState) == kVBoxNetDevOpState_Initialized);
#endif
        if (vboxNetFltWinGetWinIfState(pNetFlt) == kVBoxWinIfState_Connected)
        {
            vboxNetFltWinSetWinIfState(pNetFlt, kVBoxWinIfState_Disconnecting);
#ifndef VBOXNETADP
            Status = vboxNetFltWinPtDoUnbinding(pNetFlt, bOnUnbind);
#else
            Status = vboxNetFltWinMpDoDeinitialization(pNetFlt);
#endif
            Assert(Status == NDIS_STATUS_SUCCESS);

            vboxNetFltWinSetWinIfState(pNetFlt, kVBoxWinIfState_Disconnected);
            Assert(vboxNetFltWinGetOpState(&pNetFlt->u.s.WinIf.MpState) == kVBoxNetDevOpState_Deinitialized);
#ifndef VBOXNETADP
            Assert(vboxNetFltWinGetOpState(&pNetFlt->u.s.WinIf.PtState) == kVBoxNetDevOpState_Deinitialized);
#endif
            vboxNetFltWinPtFiniWinIf(&pNetFlt->u.s.WinIf);

            /* we're unbinding, make an unbind-related release */
            vboxNetFltRelease(pNetFlt, false);
        }
        else
        {
            AssertBreakpoint();
#ifndef VBOXNETADP
            pNetFlt->u.s.WinIf.OpenCloseStatus = NDIS_STATUS_FAILURE;
#endif
            if (!bOnUnbind)
            {
                vboxNetFltWinSetOpState(&pNetFlt->u.s.WinIf.MpState, kVBoxNetDevOpState_Deinitialized);
            }
            Status = NDIS_STATUS_FAILURE;
        }
        RTSemMutexRelease(pNetFlt->u.s.hWinIfMutex);
    }
    else
    {
        AssertBreakpoint();
        Status = NDIS_STATUS_FAILURE;
    }

    /* release for the retain we made before waining on the mutex */
    vboxNetFltRelease(pNetFlt, false);

    return Status;
}


/**
 * Checks if the host (not us) has put the adapter in promiscuous mode.
 *
 * @returns true if promiscuous, false if not.
 * @param   pThis               The instance.
 */
static bool vboxNetFltWinIsPromiscuous2(PVBOXNETFLTINS pThis)
{
#ifndef VBOXNETADP
    if (VBOXNETFLT_PROMISCUOUS_SUPPORTED(pThis))
    {
        bool bPromiscuous;
        if (!vboxNetFltWinReferenceWinIf(pThis))
            return false;

        bPromiscuous = (pThis->u.s.WinIf.fUpperProtocolSetFilter & NDIS_PACKET_TYPE_PROMISCUOUS) == NDIS_PACKET_TYPE_PROMISCUOUS;
            /*vboxNetFltWinIsPromiscuous(pAdapt);*/

        vboxNetFltWinDereferenceWinIf(pThis);
        return bPromiscuous;
    }
    return false;
#else
    return true;
#endif
}


/**
 * Report the MAC address, promiscuous mode setting, GSO capabilities and
 * no-preempt destinations to the internal network.
 *
 * Does nothing if we're not currently connected to an internal network.
 *
 * @param   pThis           The instance data.
 */
static void vboxNetFltWinReportStuff(PVBOXNETFLTINS pThis)
{
    /** @todo Keep these up to date, esp. the promiscuous mode bit. */
    if (pThis->pSwitchPort
        && vboxNetFltTryRetainBusyNotDisconnected(pThis))
    {
        pThis->pSwitchPort->pfnReportMacAddress(pThis->pSwitchPort, &pThis->u.s.MacAddr);
        pThis->pSwitchPort->pfnReportPromiscuousMode(pThis->pSwitchPort,
                                                     vboxNetFltWinIsPromiscuous2(pThis));
        pThis->pSwitchPort->pfnReportGsoCapabilities(pThis->pSwitchPort, 0,
                                                     INTNETTRUNKDIR_WIRE | INTNETTRUNKDIR_HOST);
        /** @todo We should be able to do pfnXmit at DISPATCH_LEVEL... */
        pThis->pSwitchPort->pfnReportNoPreemptDsts(pThis->pSwitchPort, 0 /* none */);
        vboxNetFltRelease(pThis, true /*fBusy*/);
    }
}

/**
 * Worker for vboxNetFltWinAttachToInterface.
 *
 * @param   pAttachInfo     Structure for communicating with
 *                          vboxNetFltWinAttachToInterface.
 */
static void vboxNetFltWinAttachToInterfaceWorker(PATTACH_INFO pAttachInfo)
{
    PVBOXNETFLTINS pThis = pAttachInfo->pNetFltIf;
    NDIS_STATUS Status = NDIS_STATUS_SUCCESS;
    int rc;

    Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);

    /* to ensure we're not removed while we're here */
    vboxNetFltRetain(pThis, false);

    rc = RTSemMutexRequest(pThis->u.s.hWinIfMutex, RT_INDEFINITE_WAIT);
    if (RT_SUCCESS(rc))
    {
        Assert(vboxNetFltWinGetWinIfState(pThis) == kVBoxWinIfState_Disconnected);
        Assert(vboxNetFltWinGetOpState(&pThis->u.s.WinIf.MpState) == kVBoxNetDevOpState_Deinitialized);
#ifndef VBOXNETADP
        Assert(vboxNetFltWinGetOpState(&pThis->u.s.WinIf.PtState) == kVBoxNetDevOpState_Deinitialized);
#endif
        if (vboxNetFltWinGetWinIfState(pThis) == kVBoxWinIfState_Disconnected)
        {
            if (pAttachInfo->fRediscovery)
            {
                /* rediscovery means adaptor bind is performed while intnet is already using it
                 * i.e. adaptor was unbound while being used by intnet and now being bound back again */
                Assert(((VBOXNETFTLINSSTATE)ASMAtomicUoReadU32((uint32_t volatile *)&pThis->enmState)) == kVBoxNetFltInsState_Connected);
            }
#ifndef VBOXNETADP
            Status = vboxNetFltWinPtInitWinIf(&pThis->u.s.WinIf, pAttachInfo->pCreateContext->pOurName);
#else
            Status = vboxNetFltWinPtInitWinIf(&pThis->u.s.WinIf);
#endif
            if (Status == NDIS_STATUS_SUCCESS)
            {
                vboxNetFltWinSetWinIfState(pThis, kVBoxWinIfState_Connecting);

#ifndef VBOXNETADP
                Status = vboxNetFltWinPtDoBinding(pThis, pAttachInfo->pCreateContext->pOurName, pAttachInfo->pCreateContext->pBindToName);
#else
                Status = vboxNetFltWinMpDoInitialization(pThis, pAttachInfo->pCreateContext->hMiniportAdapter, pAttachInfo->pCreateContext->hWrapperConfigurationContext);
#endif
                if (Status == NDIS_STATUS_SUCCESS)
                {
                    if (!pAttachInfo->fRediscovery)
                    {
                        vboxNetFltWinDrvReference();
                    }
#ifndef VBOXNETADP
                    if (pThis->u.s.WinIf.OpenCloseStatus == NDIS_STATUS_SUCCESS)
#endif
                    {
                        vboxNetFltWinSetWinIfState(pThis, kVBoxWinIfState_Connected);
#ifndef VBOXNETADP
                        Assert(vboxNetFltWinGetOpState(&pThis->u.s.WinIf.PtState) == kVBoxNetDevOpState_Initialized);
#endif
                        /* 4. mark as connected */
                        RTSpinlockAcquire(pThis->hSpinlock);
                        ASMAtomicUoWriteBool(&pThis->fDisconnectedFromHost, false);
                        RTSpinlockRelease(pThis->hSpinlock);

                        pAttachInfo->Status = VINF_SUCCESS;
                        pAttachInfo->pCreateContext->Status = NDIS_STATUS_SUCCESS;

                        RTSemMutexRelease(pThis->u.s.hWinIfMutex);

                        vboxNetFltRelease(pThis, false);

                        /* 5. Report MAC address, promiscuousness and GSO capabilities. */
                        vboxNetFltWinReportStuff(pThis);

                        return;
                    }
                    AssertBreakpoint();

                    if (!pAttachInfo->fRediscovery)
                    {
                        vboxNetFltWinDrvDereference();
                    }
#ifndef VBOXNETADP
                    vboxNetFltWinPtDoUnbinding(pThis, true);
#else
                    vboxNetFltWinMpDoDeinitialization(pThis);
#endif
                }
                AssertBreakpoint();
                vboxNetFltWinPtFiniWinIf(&pThis->u.s.WinIf);
            }
            AssertBreakpoint();
            vboxNetFltWinSetWinIfState(pThis, kVBoxWinIfState_Disconnected);
            Assert(vboxNetFltWinGetOpState(&pThis->u.s.WinIf.MpState) == kVBoxNetDevOpState_Deinitialized);
#ifndef VBOXNETADP
            Assert(vboxNetFltWinGetOpState(&pThis->u.s.WinIf.PtState) == kVBoxNetDevOpState_Deinitialized);
#endif
        }
        AssertBreakpoint();

        pAttachInfo->Status = VERR_GENERAL_FAILURE;
        pAttachInfo->pCreateContext->Status = Status;
        RTSemMutexRelease(pThis->u.s.hWinIfMutex);
    }
    else
    {
        AssertBreakpoint();
        pAttachInfo->Status = rc;
    }

    vboxNetFltRelease(pThis, false);

    return;
}

/**
 * Common code for vboxNetFltOsInitInstance and
 * vboxNetFltOsMaybeRediscovered.
 *
 * @returns IPRT status code.
 * @param   pThis           The instance.
 * @param   fRediscovery    True if vboxNetFltOsMaybeRediscovered is calling,
 *                          false if it's vboxNetFltOsInitInstance.
 */
static int vboxNetFltWinAttachToInterface(PVBOXNETFLTINS pThis, void * pContext, bool fRediscovery)
{
    ATTACH_INFO Info;
    Info.pNetFltIf = pThis;
    Info.fRediscovery = fRediscovery;
    Info.pCreateContext = (PCREATE_INSTANCE_CONTEXT)pContext;

    vboxNetFltWinAttachToInterfaceWorker(&Info);

    return Info.Status;
}
static NTSTATUS vboxNetFltWinPtDevDispatch(IN PDEVICE_OBJECT pDevObj, IN PIRP pIrp)
{
    PIO_STACK_LOCATION pIrpSl = IoGetCurrentIrpStackLocation(pIrp);;
    NTSTATUS Status = STATUS_SUCCESS;

    switch (pIrpSl->MajorFunction)
    {
        case IRP_MJ_DEVICE_CONTROL:
            Status = STATUS_NOT_SUPPORTED;
            break;
        case IRP_MJ_CREATE:
        case IRP_MJ_CLEANUP:
        case IRP_MJ_CLOSE:
            break;
        default:
            Assert(0);
            break;
    }

    pIrp->IoStatus.Status = Status;
    IoCompleteRequest(pIrp, IO_NO_INCREMENT);

    return Status;
}

static NDIS_STATUS vboxNetFltWinDevCreate(PVBOXNETFLTGLOBALS_WIN pGlobals)
{
    NDIS_STRING DevName, LinkName;
    PDRIVER_DISPATCH aMajorFunctions[IRP_MJ_MAXIMUM_FUNCTION+1];
    NdisInitUnicodeString(&DevName, VBOXNETFLT_NAME_DEVICE);
    NdisInitUnicodeString(&LinkName, VBOXNETFLT_NAME_LINK);

    Assert(!pGlobals->hDevice);
    Assert(!pGlobals->pDevObj);
    NdisZeroMemory(aMajorFunctions, sizeof (aMajorFunctions));
    aMajorFunctions[IRP_MJ_CREATE] = vboxNetFltWinPtDevDispatch;
    aMajorFunctions[IRP_MJ_CLEANUP] = vboxNetFltWinPtDevDispatch;
    aMajorFunctions[IRP_MJ_CLOSE] = vboxNetFltWinPtDevDispatch;
    aMajorFunctions[IRP_MJ_DEVICE_CONTROL] = vboxNetFltWinPtDevDispatch;

    NDIS_STATUS Status = NdisMRegisterDevice(pGlobals->Mp.hNdisWrapper,
              &DevName, &LinkName,
              aMajorFunctions,
              &pGlobals->pDevObj,
              &pGlobals->hDevice);
    Assert(Status == NDIS_STATUS_SUCCESS);
    return Status;
}

static NDIS_STATUS vboxNetFltWinDevDestroy(PVBOXNETFLTGLOBALS_WIN pGlobals)
{
    Assert(pGlobals->hDevice);
    Assert(pGlobals->pDevObj);
    NDIS_STATUS Status = NdisMDeregisterDevice(pGlobals->hDevice);
    Assert(Status == NDIS_STATUS_SUCCESS);
    if (Status == NDIS_STATUS_SUCCESS)
    {
        pGlobals->hDevice = NULL;
        pGlobals->pDevObj = NULL;
    }
    return Status;
}

static NDIS_STATUS vboxNetFltWinDevCreateReference(PVBOXNETFLTGLOBALS_WIN pGlobals)
{
    Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);
    NDIS_STATUS Status = KeWaitForSingleObject(&pGlobals->SynchEvent, Executive, KernelMode, FALSE, NULL);
    Assert(Status == STATUS_SUCCESS);
    if (Status == STATUS_SUCCESS)
    {
        Assert(pGlobals->cDeviceRefs >= 0);
        if (++pGlobals->cDeviceRefs == 1)
        {
            Status = vboxNetFltWinDevCreate(pGlobals);
            if (Status == NDIS_STATUS_SUCCESS)
            {
                ObReferenceObject(pGlobals->pDevObj);
            }
        }
        else
        {
            Status = NDIS_STATUS_SUCCESS;
        }
        KeSetEvent(&pGlobals->SynchEvent, 0, FALSE);
    }
    else
    {
        /* should never happen actually */
        Assert(0);
        Status = NDIS_STATUS_FAILURE;
    }
    return Status;
}

static NDIS_STATUS vboxNetFltWinDevDereference(PVBOXNETFLTGLOBALS_WIN pGlobals)
{
    Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);
    NDIS_STATUS Status = KeWaitForSingleObject(&pGlobals->SynchEvent, Executive, KernelMode, FALSE, NULL);
    Assert(Status == STATUS_SUCCESS);
    if (Status == STATUS_SUCCESS)
    {
        Assert(pGlobals->cDeviceRefs > 0);
        if (!(--pGlobals->cDeviceRefs))
        {
            ObDereferenceObject(pGlobals->pDevObj);
            Status = vboxNetFltWinDevDestroy(pGlobals);
        }
        else
        {
            Status = NDIS_STATUS_SUCCESS;
        }
        KeSetEvent(&pGlobals->SynchEvent, 0, FALSE);
    }
    else
    {
        /* should never happen actually */
        Assert(0);
        Status = NDIS_STATUS_FAILURE;
    }
    return Status;
}

/* reference the driver module to prevent driver unload */
DECLHIDDEN(void) vboxNetFltWinDrvReference()
{
    vboxNetFltWinDevCreateReference(&g_VBoxNetFltGlobalsWin);
}

/* dereference the driver module to prevent driver unload */
DECLHIDDEN(void) vboxNetFltWinDrvDereference()
{
    vboxNetFltWinDevDereference(&g_VBoxNetFltGlobalsWin);
}

/*
 *
 * The OS specific interface definition
 *
 */


bool vboxNetFltOsMaybeRediscovered(PVBOXNETFLTINS pThis)
{
    /* AttachToInterface true if disconnected */
    return !ASMAtomicUoReadBool(&pThis->fDisconnectedFromHost);
}

int vboxNetFltPortOsXmit(PVBOXNETFLTINS pThis, void *pvIfData, PINTNETSG pSG, uint32_t fDst)
{
    int rc = VINF_SUCCESS;
    uint32_t cRefs = 0;
#ifndef VBOXNETADP
    if (fDst & INTNETTRUNKDIR_WIRE)
    {
        cRefs++;
    }
    if (fDst & INTNETTRUNKDIR_HOST)
    {
        cRefs++;
    }
#else
    if (fDst & INTNETTRUNKDIR_WIRE || fDst & INTNETTRUNKDIR_HOST)
    {
        cRefs = 1;
    }
#endif

    AssertReturn(cRefs, VINF_SUCCESS);

    if (!vboxNetFltWinIncReferenceWinIf(pThis, cRefs))
    {
        return VERR_GENERAL_FAILURE;
    }
#ifndef VBOXNETADP
    if (fDst & INTNETTRUNKDIR_WIRE)
    {
        PNDIS_PACKET pPacket;

        pPacket = vboxNetFltWinNdisPacketFromSG(pThis, pSG, NULL /*pBufToFree*/,
                                                true /*fToWire*/, true /*fCopyMemory*/);

        if (pPacket)
        {
            NDIS_STATUS fStatus;

#ifndef VBOX_LOOPBACK_USEFLAGS
            /* force "don't loopback" flags to prevent loopback branch invocation in any case
             * to avoid ndis misbehave */
            NdisGetPacketFlags(pPacket) |= g_VBoxNetFltGlobalsWin.fPacketDontLoopBack;
#else
            /* this is done by default in vboxNetFltWinNdisPacketFromSG */
#endif

#if defined(DEBUG_NETFLT_PACKETS) || !defined(VBOX_LOOPBACK_USEFLAGS)
            vboxNetFltWinLbPutSendPacket(pThis, pPacket, true /* bFromIntNet */);
#endif
            NdisSend(&fStatus, pThis->u.s.WinIf.hBinding, pPacket);
            if (fStatus != NDIS_STATUS_PENDING)
            {
#if defined(DEBUG_NETFLT_PACKETS) || !defined(VBOX_LOOPBACK_USEFLAGS)
                /* the status is NOT pending, complete the packet */
                bool bTmp = vboxNetFltWinLbRemoveSendPacket(pThis, pPacket);
                Assert(bTmp);
#endif
                if (!NT_SUCCESS(fStatus))
                {
                    /* TODO: convert status to VERR_xxx */
                    rc = VERR_GENERAL_FAILURE;
                }

                vboxNetFltWinFreeSGNdisPacket(pPacket, true);
            }
            else
            {
                /* pending, dereference on packet complete */
                cRefs--;
            }
        }
        else
        {
            AssertFailed();
            rc = VERR_NO_MEMORY;
        }
    }
#endif

#ifndef VBOXNETADP
    if (fDst & INTNETTRUNKDIR_HOST)
#else
    if (cRefs)
#endif
    {
        PNDIS_PACKET pPacket = vboxNetFltWinNdisPacketFromSG(pThis, pSG, NULL /*pBufToFree*/,
                                                             false /*fToWire*/, true /*fCopyMemory*/);
        if (pPacket)
        {
            NdisMIndicateReceivePacket(pThis->u.s.WinIf.hMiniport, &pPacket, 1);
            cRefs--;
#ifdef VBOXNETADP
            STATISTIC_INCREASE(pThis->u.s.WinIf.cRxSuccess);
#endif
        }
        else
        {
            AssertFailed();
#ifdef VBOXNETADP
            STATISTIC_INCREASE(pThis->u.s.WinIf.cRxError);
#endif
            rc = VERR_NO_MEMORY;
        }
    }

    Assert(cRefs <= 2);

    if (cRefs)
    {
        vboxNetFltWinDecReferenceWinIf(pThis, cRefs);
    }

    return rc;
}

void vboxNetFltPortOsSetActive(PVBOXNETFLTINS pThis, bool fActive)
{
#ifndef VBOXNETADP
    NDIS_STATUS Status;
#endif
    /* we first wait for all pending ops to complete
     * this might include all packets queued for processing */
    for (;;)
    {
        if (fActive)
        {
            if (!pThis->u.s.cModePassThruRefs)
            {
                break;
            }
        }
        else
        {
            if (!pThis->u.s.cModeNetFltRefs)
            {
                break;
            }
        }
        vboxNetFltWinSleep(2);
    }

    if (!vboxNetFltWinReferenceWinIf(pThis))
        return;
#ifndef VBOXNETADP

    if (fActive)
    {
#ifdef DEBUG_misha
        NDIS_PHYSICAL_MEDIUM PhMedium;
        bool bPromiscSupported;

        Status = vboxNetFltWinQueryPhysicalMedium(pThis, &PhMedium);
        if (Status != NDIS_STATUS_SUCCESS)
        {

            LogRel(("vboxNetFltWinQueryPhysicalMedium failed, Status (0x%x), setting medium to NdisPhysicalMediumUnspecified\n", Status));
            Assert(Status == NDIS_STATUS_NOT_SUPPORTED);
            if (Status != NDIS_STATUS_NOT_SUPPORTED)
            {
                LogRel(("vboxNetFltWinQueryPhysicalMedium failed, Status (0x%x), setting medium to NdisPhysicalMediumUnspecified\n", Status));
            }
            PhMedium = NdisPhysicalMediumUnspecified;
        }
        else
        {
            LogRel(("(SUCCESS) vboxNetFltWinQueryPhysicalMedium SUCCESS\n"));
        }

        bPromiscSupported = (!(PhMedium == NdisPhysicalMediumWirelessWan
                        || PhMedium == NdisPhysicalMediumWirelessLan
                        || PhMedium == NdisPhysicalMediumNative802_11
                        || PhMedium == NdisPhysicalMediumBluetooth
                        /*|| PhMedium == NdisPhysicalMediumWiMax */
                        ));

        Assert(bPromiscSupported == VBOXNETFLT_PROMISCUOUS_SUPPORTED(pThis));
#endif
    }

    if (VBOXNETFLT_PROMISCUOUS_SUPPORTED(pThis))
    {
        Status = vboxNetFltWinSetPromiscuous(pThis, fActive);
        if (Status != NDIS_STATUS_SUCCESS)
        {
            LogRel(("vboxNetFltWinSetPromiscuous failed, Status (0x%x), fActive (%d)\n", Status, fActive));
            AssertFailed();
        }
    }
#else
# ifdef VBOXNETADP_REPORT_DISCONNECTED
    if (fActive)
    {
        NdisMIndicateStatus(pThis->u.s.WinIf.hMiniport,
                                 NDIS_STATUS_MEDIA_CONNECT,
                                 (PVOID)NULL,
                                 0);
    }
    else
    {
        NdisMIndicateStatus(pThis->u.s.WinIf.hMiniport,
                                 NDIS_STATUS_MEDIA_DISCONNECT,
                                 (PVOID)NULL,
                                 0);
    }
#else
    if (fActive)
    {
        /* indicate status change to make the ip settings be re-picked for dhcp */
        NdisMIndicateStatus(pThis->u.s.WinIf.hMiniport,
                                 NDIS_STATUS_MEDIA_DISCONNECT,
                                 (PVOID)NULL,
                                 0);

        NdisMIndicateStatus(pThis->u.s.WinIf.hMiniport,
                                 NDIS_STATUS_MEDIA_CONNECT,
                                 (PVOID)NULL,
                                 0);
    }
# endif
#endif
    vboxNetFltWinDereferenceWinIf(pThis);

    return;
}

int vboxNetFltOsDisconnectIt(PVBOXNETFLTINS pThis)
{
    NDIS_STATUS Status = vboxNetFltWinDisconnectIt(pThis);
    return Status == NDIS_STATUS_SUCCESS ? VINF_SUCCESS : VERR_GENERAL_FAILURE;
}

static void vboxNetFltWinConnectItWorker(PVOID pvContext)
{
    PWORKER_INFO pInfo = (PWORKER_INFO)pvContext;
#if !defined(VBOXNETADP) || !defined(VBOXNETFLT_NO_PACKET_QUEUE)
    NDIS_STATUS Status;
#endif
    PVBOXNETFLTINS pInstance = pInfo->pNetFltIf;

    Assert(KeGetCurrentIrql() == PASSIVE_LEVEL);

    /* this is not a rediscovery, initialize Mac cache */
    if (vboxNetFltWinReferenceWinIf(pInstance))
    {
#ifndef VBOXNETADP
        Status = vboxNetFltWinGetMacAddress(pInstance, &pInstance->u.s.MacAddr);
        if (Status == NDIS_STATUS_SUCCESS)
#endif
        {
#ifdef VBOXNETFLT_NO_PACKET_QUEUE
            pInfo->Status = VINF_SUCCESS;
#else
            Status = vboxNetFltWinQuInitPacketQueue(pInstance);
            if (Status == NDIS_STATUS_SUCCESS)
            {
                pInfo->Status = VINF_SUCCESS;
            }
            else
            {
                pInfo->Status = VERR_GENERAL_FAILURE;
            }
#endif
        }
#ifndef VBOXNETADP
        else
        {
            pInfo->Status = VERR_INTNET_FLT_IF_FAILED;
        }
#endif

        vboxNetFltWinDereferenceWinIf(pInstance);
    }
    else
    {
        pInfo->Status = VERR_INTNET_FLT_IF_NOT_FOUND;
    }
}

static int vboxNetFltWinConnectIt(PVBOXNETFLTINS pThis)
{
    WORKER_INFO Info;
    Info.pNetFltIf = pThis;

    vboxNetFltWinJobSynchExecAtPassive(vboxNetFltWinConnectItWorker, &Info);

    if (RT_SUCCESS(Info.Status))
        vboxNetFltWinReportStuff(pThis);

    return Info.Status;
}

int vboxNetFltOsConnectIt(PVBOXNETFLTINS pThis)
{
    return vboxNetFltWinConnectIt(pThis);
}

void vboxNetFltOsDeleteInstance(PVBOXNETFLTINS pThis)
{
    vboxNetFltWinDeleteInstance(pThis);
}

int vboxNetFltOsInitInstance(PVBOXNETFLTINS pThis, void *pvContext)
{
    int rc = RTSemMutexCreate(&pThis->u.s.hWinIfMutex);
    if (RT_SUCCESS(rc))
    {
        rc = vboxNetFltWinAttachToInterface(pThis, pvContext, false /*fRediscovery*/ );
        if (RT_SUCCESS(rc))
        {
            return rc;
        }
        RTSemMutexDestroy(pThis->u.s.hWinIfMutex);
    }
    return rc;
}

int vboxNetFltOsPreInitInstance(PVBOXNETFLTINS pThis)
{
    pThis->u.s.cModeNetFltRefs = 0;
    pThis->u.s.cModePassThruRefs = 0;
    vboxNetFltWinSetWinIfState(pThis, kVBoxWinIfState_Disconnected);
    vboxNetFltWinSetOpState(&pThis->u.s.WinIf.MpState, kVBoxNetDevOpState_Deinitialized);
#ifndef VBOXNETADP
    vboxNetFltWinSetOpState(&pThis->u.s.WinIf.PtState, kVBoxNetDevOpState_Deinitialized);
#endif
    return VINF_SUCCESS;
}

void vboxNetFltPortOsNotifyMacAddress(PVBOXNETFLTINS pThis, void *pvIfData, PCRTMAC pMac)
{
}

int vboxNetFltPortOsConnectInterface(PVBOXNETFLTINS pThis, void *pvIf, void **ppvIfData)
{
    /* Nothing to do */
    return VINF_SUCCESS;
}

int vboxNetFltPortOsDisconnectInterface(PVBOXNETFLTINS pThis, void *pvIfData)
{
    /* Nothing to do */
    return VINF_SUCCESS;
}
