/* $Id: USBProxyDevice-darwin.cpp $ */
/** @file
 * USB device proxy - the Darwin backend.
 */

/*
 * Copyright (C) 2006-2010 Oracle Corporation
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
#define LOG_GROUP LOG_GROUP_DRV_USBPROXY
#define __STDC_LIMIT_MACROS
#define __STDC_CONSTANT_MACROS

#include <mach/mach.h>
#include <Carbon/Carbon.h>
#include <IOKit/IOKitLib.h>
#include <mach/mach_error.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/IOCFPlugIn.h>

#include <VBox/log.h>
#include <VBox/err.h>
#include <VBox/vmm/pdm.h>

#include <iprt/assert.h>
#include <iprt/critsect.h>
#include <iprt/mem.h>
#include <iprt/once.h>
#include <iprt/string.h>
#include <iprt/time.h>

#include "../USBProxyDevice.h"
#include <VBox/usblib.h>


/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
/** An experiment... */
//#define USE_LOW_LATENCY_API 1

/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/
/** Forward declaration of the Darwin interface structure. */
typedef struct USBPROXYIFOSX *PUSBPROXYIFOSX;


/**
 * A low latency isochronous buffer.
 *
 * These are allocated in chunks on an interface level, see USBPROXYISOCBUFCOL.
 */
typedef struct USBPROXYISOCBUF
{
    /** Whether this buffer is in use or not. */
    bool volatile               fUsed;
    /** Pointer to the buffer. */
    void                       *pvBuf;
    /** Pointer to an array of 8 frames. */
    IOUSBLowLatencyIsocFrame   *paFrames;
} USBPROXYISOCBUF, *PUSBPROXYISOCBUF;


/**
 * Isochronous buffer collection (associated with an interface).
 *
 * These are allocated in decent sized chunks and there isn't supposed
 * to be too many of these per interface.
 */
typedef struct USBPROXYISOCBUFCOL
{
    /** Write or Read buffers? */
    USBLowLatencyBufferType     enmType;
    /** The next buffer collection on this interface. */
    struct USBPROXYISOCBUFCOL  *pNext;
    /** The buffer. */
    void                       *pvBuffer;
    /** The frame. */
    void                       *pvFrames;
    /** The buffers.
     * The number of buffers here is decided by pvFrame begin allocated in
     * PAGE_SIZE chunks. The size of IOUSBLowLatencyIsocFrame is 16 bytes
     * and we require 8 of those per buffer. PAGE_SIZE / (16 * 8) = 32.
     * @remarks  Don't allocate too many as it may temporarily halt the system if
     *           some pool is low / exhausted. (Contiguous memory woes on mach.)
     */
    USBPROXYISOCBUF             aBuffers[/*32*/ 4];
} USBPROXYISOCBUFCOL, *PUSBPROXYISOCBUFCOL;

AssertCompileSize(IOUSBLowLatencyIsocFrame, 16);

/**
 * Per-urb data for the Darwin usb proxy backend.
 *
 * This is required to track in-flight and landed URBs
 * since we take down the URBs in a different thread (perhaps).
 */
typedef struct USBPROXYURBOSX
{
    /** Pointer to the next Darwin URB. */
    struct USBPROXYURBOSX  *pNext;
    /** Pointer to the previous Darwin URB. */
    struct USBPROXYURBOSX  *pPrev;
    /** The millisecond timestamp when this URB was submitted. */
    uint64_t                u64SubmitTS;
    /** Pointer to the VUSB URB.
     * This is set to NULL if canceled. */
    PVUSBURB                pVUsbUrb;
    /** Pointer to the Darwin device. */
    struct USBPROXYDEVOSX  *pDevOsX;
    /** The transfer type. */
    VUSBXFERTYPE            enmType;
    /** Union with data depending on transfer type. */
    union
    {
        /** The control message. */
        IOUSBDevRequest     ControlMsg;
        /** The Isochronous Data. */
        struct
        {
#ifdef USE_LOW_LATENCY_API
            /** The low latency isochronous buffer. */
            PUSBPROXYISOCBUF            pBuf;
            /** Array of frames parallel to the one in VUSBURB. (Same as pBuf->paFrames.) */
            IOUSBLowLatencyIsocFrame   *aFrames;
#else
            /** Array of frames parallel to the one in VUSBURB. */
            IOUSBIsocFrame              aFrames[8];
#endif
        } Isoc;
    } u;
} USBPROXYURBOSX, *PUSBPROXYURBOSX;

/**
 * Per-pipe data for the Darwin usb proxy backend.
 */
typedef struct USBPROXYPIPEOSX
{
    /** The endpoint number. */
    uint8_t                 u8Endpoint;
    /** The IOKit pipe reference. */
    uint8_t                 u8PipeRef;
    /** The pipe Transfer type type. */
    uint8_t                 u8TransferType;
    /** The pipe direction. */
    uint8_t                 u8Direction;
    /** The endpoint interval. (interrupt) */
    uint8_t                 u8Interval;
    /** The max packet size. */
    uint16_t                u16MaxPacketSize;
    /** The next frame number (isochronous pipes only). */
    uint64_t                u64NextFrameNo;
} USBPROXYPIPEOSX, *PUSBPROXYPIPEOSX, **PPUSBPROXYPIPEOSX;

/**
 * Per-interface data for the Darwin usb proxy backend.
 */
typedef struct USBPROXYIFOSX
{
    /** Pointer to the next interface. */
    struct USBPROXYIFOSX   *pNext;
    /** The interface number. */
    uint8_t                 u8Interface;
    /** The current alternative interface setting.
     * This is used to skip unnecessary SetAltInterface calls. */
    uint8_t                 u8AltSetting;
    /** The interface class. (not really used) */
    uint8_t                 u8Class;
    /** The interface protocol. (not really used) */
    uint8_t                 u8Protocol;
    /** The number of pipes. */
    uint8_t                 cPipes;
    /** Array containing all the pipes. (Currently unsorted.) */
    USBPROXYPIPEOSX         aPipes[kUSBMaxPipes];
    /** The IOUSBDeviceInterface. */
    IOUSBInterfaceInterface245 **ppIfI;
    /** The run loop source for the async operations on the interface level. */
    CFRunLoopSourceRef      RunLoopSrcRef;
    /** List of isochronous buffer collections.
     * These are allocated on demand by the URB queuing routine and then recycled until the interface is destroyed. */
    PUSBPROXYISOCBUFCOL     pIsocBufCols;
} USBPROXYIFOSX, *PUSBPROXYIFOSX, **PPUSBPROXYIFOSX;
/** Pointer to a pointer to an darwin interface. */
typedef USBPROXYIFOSX **PPUSBPROXYIFOSX;

/**
 * Per-device Data for the Darwin usb proxy backend.
 */
typedef struct USBPROXYDEVOSX
{
    /** The USB Device IOService object. */
    io_object_t             USBDevice;
    /** The IOUSBDeviceInterface. */
    IOUSBDeviceInterface245 **ppDevI;
    /** The run loop source for the async operations on the device level
     * (i.e. the default control pipe stuff). */
    CFRunLoopSourceRef      RunLoopSrcRef;
    /** The run loop this device and its interfaces send their events to. */
    CFRunLoopRef            RunLoopRef;

    /** Pointer to the proxy device instance. */
    PUSBPROXYDEV            pProxyDev;

    /** Pointer to the first interface. */
    PUSBPROXYIFOSX          pIfHead;
    /** Pointer to the last interface. */
    PUSBPROXYIFOSX          pIfTail;

    /** Critical section protecting the lists. */
    RTCRITSECT              CritSect;
    /** The list of free Darwin URBs. Singly linked. */
    PUSBPROXYURBOSX         pFreeHead;
    /** The list of active Darwin URBs. Doubly linked.
     * Only the split head will appear in this list. */
    PUSBPROXYURBOSX         pInFlightHead;
    /** The list of landed Darwin URBs. Doubly linked.
     * Only the split head will appear in this list. */
    PUSBPROXYURBOSX         pTaxingHead;
    /** The tail of the landed Darwin URBs. */
    PUSBPROXYURBOSX         pTaxingTail;
} USBPROXYDEVOSX, *PUSBPROXYDEVOSX;


/*******************************************************************************
*   Global Variables                                                           *
*******************************************************************************/
static RTONCE       g_usbProxyDarwinOnce = RTONCE_INITIALIZER;
/** The runloop mode we use.
 * Since it's difficult to remove this, we leak it to prevent crashes.
 * @bugref{4407} */
static CFStringRef g_pRunLoopMode = NULL;
/** The IO Master Port.
 * Not worth cleaning up.  */
static mach_port_t  g_MasterPort = NULL;


/**
 * Init once callback that sets up g_MasterPort and g_pRunLoopMode.
 *
 * @returns IPRT status code.
 *
 * @param   pvUser1     NULL, ignored.
 * @param   pvUser2     NULL, ignored.
 */
static DECLCALLBACK(int32_t) usbProxyDarwinInitOnce(void *pvUser1, void *pvUser2)
{
    int rc;
    kern_return_t krc = IOMasterPort(MACH_PORT_NULL, &g_MasterPort);
    if (krc == KERN_SUCCESS)
    {
       g_pRunLoopMode = CFStringCreateWithCString(kCFAllocatorDefault, "VBoxUsbProxyMode", kCFStringEncodingUTF8);
       if (g_pRunLoopMode)
           return VINF_SUCCESS;
       rc = VERR_INTERNAL_ERROR_5;
    }
    else
        rc = RTErrConvertFromDarwin(krc);
    return rc;
}


/**
 * Allocates a Darwin URB request structure.
 *
 * @returns Pointer to an active URB request.
 * @returns NULL on failure.
 *
 * @param   pDevOsX         The darwin proxy device.
 */
static PUSBPROXYURBOSX  usbProxyDarwinUrbAlloc(PUSBPROXYDEVOSX pDevOsX)
{
    PUSBPROXYURBOSX pUrbOsX;

    RTCritSectEnter(&pDevOsX->CritSect);

    /*
     * Try remove a Darwin URB from the free list, if none there allocate a new one.
     */
    pUrbOsX = pDevOsX->pFreeHead;
    if (pUrbOsX)
        pDevOsX->pFreeHead = pUrbOsX->pNext;
    else
    {
        RTCritSectLeave(&pDevOsX->CritSect);
        pUrbOsX = (PUSBPROXYURBOSX)RTMemAlloc(sizeof(*pUrbOsX));
        if (!pUrbOsX)
            return NULL;
        RTCritSectEnter(&pDevOsX->CritSect);
    }
    pUrbOsX->pVUsbUrb = NULL;
    pUrbOsX->pDevOsX = pDevOsX;
    pUrbOsX->enmType = VUSBXFERTYPE_INVALID;

    /*
     * Link it into the active list
     */
    pUrbOsX->pPrev = NULL;
    pUrbOsX->pNext = pDevOsX->pInFlightHead;
    if (pUrbOsX->pNext)
        pUrbOsX->pNext->pPrev = pUrbOsX;
    pDevOsX->pInFlightHead = pUrbOsX;

    RTCritSectLeave(&pDevOsX->CritSect);
    return pUrbOsX;
}


#ifdef USE_LOW_LATENCY_API
/**
 * Allocates an low latency isochronous buffer.
 *
 * @returns VBox status code.
 * @param   pUrbOsX The OsX URB to allocate it for.
 * @param   pIf     The interface to allocated it from.
 */
static int usbProxyDarwinUrbAllocIsocBuf(PUSBPROXYURBOSX pUrbOsX, PUSBPROXYIFOSX pIf)
{
    USBLowLatencyBufferType enmLLType = pUrbOsX->pVUsbUrb->enmDir == VUSBDIRECTION_IN
                                      ? kUSBLowLatencyWriteBuffer : kUSBLowLatencyReadBuffer;

    /*
     * Walk the buffer collection list and look for an unused one.
     */
    pUrbOsX->u.Isoc.pBuf = NULL;
    for (PUSBPROXYISOCBUFCOL pCur = pIf->pIsocBufCols; pCur; pCur = pCur->pNext)
        if (pCur->enmType == enmLLType)
            for (unsigned i = 0; i < RT_ELEMENTS(pCur->aBuffers); i++)
                if (!pCur->aBuffers[i].fUsed)
                {
                    pCur->aBuffers[i].fUsed = true;
                    pUrbOsX->u.Isoc.pBuf = &pCur->aBuffers[i];
                    AssertPtr(pUrbOsX->u.Isoc.pBuf);
                    AssertPtr(pUrbOsX->u.Isoc.pBuf->pvBuf);
                    pUrbOsX->u.Isoc.aFrames = pCur->aBuffers[i].paFrames;
                    AssertPtr(pUrbOsX->u.Isoc.aFrames);
                    return VINF_SUCCESS;
                }

    /*
     * Didn't find an empty one, create a new buffer collection and take the first buffer.
     */
    PUSBPROXYISOCBUFCOL pNew = (PUSBPROXYISOCBUFCOL)RTMemAllocZ(sizeof(*pNew));
    AssertReturn(pNew, VERR_NO_MEMORY);

    IOReturn irc = (*pIf->ppIfI)->LowLatencyCreateBuffer(pIf->ppIfI, &pNew->pvBuffer, 8192 * RT_ELEMENTS(pNew->aBuffers), enmLLType);
    if (irc == kIOReturnSuccess != VALID_PTR(pNew->pvBuffer))
    {
        AssertPtr(pNew->pvBuffer);
        irc = kIOReturnNoMemory;
    }
    if (irc == kIOReturnSuccess)
    {
        irc = (*pIf->ppIfI)->LowLatencyCreateBuffer(pIf->ppIfI, &pNew->pvFrames, PAGE_SIZE, kUSBLowLatencyFrameListBuffer);
        if (irc == kIOReturnSuccess != VALID_PTR(pNew->pvFrames))
        {
            AssertPtr(pNew->pvFrames);
            irc = kIOReturnNoMemory;
        }
        if (irc == kIOReturnSuccess)
        {
            for (unsigned i = 0; i < RT_ELEMENTS(pNew->aBuffers); i++)
            {
                //pNew->aBuffers[i].fUsed = false;
                pNew->aBuffers[i].paFrames = &((IOUSBLowLatencyIsocFrame *)pNew->pvFrames)[i * 8];
                pNew->aBuffers[i].pvBuf = (uint8_t *)pNew->pvBuffer + i * 8192;
            }

            pNew->aBuffers[0].fUsed = true;
            pUrbOsX->u.Isoc.aFrames = pNew->aBuffers[0].paFrames;
            pUrbOsX->u.Isoc.pBuf = &pNew->aBuffers[0];

            pNew->enmType = enmLLType;
            pNew->pNext = pIf->pIsocBufCols;
            pIf->pIsocBufCols = pNew;

#if 0 /* doesn't help :-/ */
            /*
             * If this is the first time we're here, try mess with the policy?
             */
            if (!pNew->pNext)
                for (unsigned iPipe = 0; iPipe < pIf->cPipes; iPipe++)
                    if (pIf->aPipes[iPipe].u8TransferType == kUSBIsoc)
                    {
                        irc = (*pIf->ppIfI)->SetPipePolicy(pIf->ppIfI, pIf->aPipes[iPipe].u8PipeRef,
                                                           pIf->aPipes[iPipe].u16MaxPacketSize, pIf->aPipes[iPipe].u8Interval);
                        AssertMsg(irc == kIOReturnSuccess, ("%#x\n", irc));
                    }
#endif

            return VINF_SUCCESS;
        }

        /* bail out */
        (*pIf->ppIfI)->LowLatencyDestroyBuffer(pIf->ppIfI, pNew->pvBuffer);
    }
    AssertMsgFailed(("%#x\n", irc));
    RTMemFree(pNew);

    return RTErrConvertFromDarwin(irc);
}
#endif /* USE_LOW_LATENCY_API */


/**
 * Frees a Darwin URB request structure.
 *
 * @param   pDevOsX         The darwin proxy device.
 * @param   pUrbOsX         The Darwin URB to free.
 */
static void usbProxyDarwinUrbFree(PUSBPROXYDEVOSX pDevOsX, PUSBPROXYURBOSX pUrbOsX)
{
    RTCritSectEnter(&pDevOsX->CritSect);

    /*
     * Remove from the active or taxing list.
     */
    if (pUrbOsX->pNext)
        pUrbOsX->pNext->pPrev   = pUrbOsX->pPrev;
    else if (pDevOsX->pTaxingTail == pUrbOsX)
        pDevOsX->pTaxingTail    = pUrbOsX->pPrev;

    if (pUrbOsX->pPrev)
        pUrbOsX->pPrev->pNext   = pUrbOsX->pNext;
    else if (pDevOsX->pTaxingHead == pUrbOsX)
        pDevOsX->pTaxingHead    = pUrbOsX->pNext;
    else if (pDevOsX->pInFlightHead == pUrbOsX)
        pDevOsX->pInFlightHead  = pUrbOsX->pNext;
    else
        AssertFailed();

#ifdef USE_LOW_LATENCY_API
    /*
     * Free low latency stuff.
     */
    if (    pUrbOsX->enmType == VUSBXFERTYPE_ISOC
        &&  pUrbOsX->u.Isoc.pBuf)
    {
        pUrbOsX->u.Isoc.pBuf->fUsed = false;
        pUrbOsX->u.Isoc.pBuf = NULL;
    }
#endif

    /*
     * Link it into the free list.
     */
    pUrbOsX->pPrev = NULL;
    pUrbOsX->pNext = pDevOsX->pFreeHead;
    pDevOsX->pFreeHead = pUrbOsX;

    pUrbOsX->pVUsbUrb = NULL;
    pUrbOsX->pDevOsX = NULL;
    pUrbOsX->enmType = VUSBXFERTYPE_INVALID;

    RTCritSectLeave(&pDevOsX->CritSect);
}

/**
 * Translate the IOKit status code to a VUSB status.
 *
 * @returns VUSB URB status code.
 * @param   irc     IOKit status code.
 */
static VUSBSTATUS vusbProxyDarwinStatusToVUsbStatus(IOReturn irc)
{
    switch (irc)
    {
        /*   IOKit                       OHCI       VUSB  */
        case kIOReturnSuccess:          /*  0 */    return VUSBSTATUS_OK;
        case kIOUSBCRCErr:              /*  1 */    return VUSBSTATUS_CRC;
        //case kIOUSBBitstufErr:          /*  2 */    return VUSBSTATUS_;
        //case kIOUSBDataToggleErr:       /*  3 */    return VUSBSTATUS_;
        case kIOUSBPipeStalled:         /*  4 */    return VUSBSTATUS_STALL;
        case kIOReturnNotResponding:    /*  5 */    return VUSBSTATUS_DNR;
        //case kIOUSBPIDCheckErr:         /*  6 */    return VUSBSTATUS_;
        //case kIOUSBWrongPIDErr:         /*  7 */    return VUSBSTATUS_;
        case kIOReturnOverrun:          /*  8 */    return VUSBSTATUS_DATA_OVERRUN;
        case kIOReturnUnderrun:         /*  9 */    return VUSBSTATUS_DATA_UNDERRUN;
        //case kIOUSBReserved1Err:        /* 10 */    return VUSBSTATUS_;
        //case kIOUSBReserved2Err:        /* 11 */    return VUSBSTATUS_;
        //case kIOUSBBufferOverrunErr:    /* 12 */    return VUSBSTATUS_;
        //case kIOUSBBufferUnderrunErr:   /* 13 */    return VUSBSTATUS_;
        case kIOUSBNotSent1Err:         /* 14 */    return VUSBSTATUS_NOT_ACCESSED/*VUSBSTATUS_OK*/;
        case kIOUSBNotSent2Err:         /* 15 */    return VUSBSTATUS_NOT_ACCESSED/*VUSBSTATUS_OK*/;

        /* Other errors */
        case kIOUSBTransactionTimeout:              return VUSBSTATUS_DNR;
        //case kIOReturnAborted:                      return VUSBSTATUS_CRC; - see on SET_INTERFACE...

        default:
            Log(("vusbProxyDarwinStatusToVUsbStatus: irc=%#x!!\n", irc));
            return VUSBSTATUS_STALL;
    }
}


/**
 * Completion callback for an async URB transfer.
 *
 * @param   pvUrbOsX    The Darwin URB.
 * @param   irc         The status of the operation.
 * @param   Size        Possible the transfer size.
 */
static void usbProxyDarwinUrbAsyncComplete(void *pvUrbOsX, IOReturn irc, void *Size)
{
    PUSBPROXYURBOSX pUrbOsX = (PUSBPROXYURBOSX)pvUrbOsX;
    PUSBPROXYDEVOSX pDevOsX = pUrbOsX->pDevOsX;
    const uint32_t cb = (uintptr_t)Size;

    RTCritSectEnter(&pDevOsX->CritSect);

    /*
     * Do status updates.
     */
    PVUSBURB pUrb = pUrbOsX->pVUsbUrb;
    if (pUrb)
    {
        Assert(pUrb->u32Magic == VUSBURB_MAGIC);
        if (pUrb->enmType == VUSBXFERTYPE_ISOC)
        {
#ifdef USE_LOW_LATENCY_API
            /* copy the data. */
            //if (pUrb->enmDir == VUSBDIRECTION_IN)
                memcpy(pUrb->abData, pUrbOsX->u.Isoc.pBuf->pvBuf, pUrb->cbData);
#endif
            Log3(("AsyncComplete isoc - raw data (%d bytes):\n"
                  "%16.*Rhxd\n", pUrb->cbData, pUrb->cbData, pUrb->abData));
            uint32_t off = 0;
            for (unsigned i = 0; i < pUrb->cIsocPkts; i++)
            {
#ifdef USE_LOW_LATENCY_API
                Log2(("  %d{%d/%d-%x-%RX64}", i, pUrbOsX->u.Isoc.aFrames[i].frActCount, pUrb->aIsocPkts[i].cb, pUrbOsX->u.Isoc.aFrames[i].frStatus,
                      RT_MAKE_U64(pUrbOsX->u.Isoc.aFrames[i].frTimeStamp.lo, pUrbOsX->u.Isoc.aFrames[i].frTimeStamp.hi) ));
#else
                Log2(("  %d{%d/%d-%x}", i, pUrbOsX->u.Isoc.aFrames[i].frActCount, pUrb->aIsocPkts[i].cb, pUrbOsX->u.Isoc.aFrames[i].frStatus));
#endif
                pUrb->aIsocPkts[i].enmStatus = vusbProxyDarwinStatusToVUsbStatus(pUrbOsX->u.Isoc.aFrames[i].frStatus);
                pUrb->aIsocPkts[i].cb = pUrbOsX->u.Isoc.aFrames[i].frActCount;
                off += pUrbOsX->u.Isoc.aFrames[i].frActCount;
            }
            Log2(("\n"));
#if 0 /** @todo revisit this, wasn't working previously. */
            for (int i = (int)pUrb->cIsocPkts - 1; i >= 0; i--)
                Assert(   !pUrbOsX->u.Isoc.aFrames[i].frActCount
                       && !pUrbOsX->u.Isoc.aFrames[i].frReqCount
                       && !pUrbOsX->u.Isoc.aFrames[i].frStatus);
#endif
            pUrb->cbData = off; /* 'Size' seems to be pointing at an error code or something... */
            pUrb->enmStatus = VUSBSTATUS_OK; /* Don't use 'irc'. OHCI expects OK unless it's a really bad error. */
        }
        else
        {
            pUrb->cbData = cb;
            pUrb->enmStatus = vusbProxyDarwinStatusToVUsbStatus(irc);
            if (pUrb->enmType == VUSBXFERTYPE_MSG)
                pUrb->cbData += sizeof(VUSBSETUP);
        }
    }

    /*
     * Remove from the active list.
     */
    if (pUrbOsX->pNext)
        pUrbOsX->pNext->pPrev = pUrbOsX->pPrev;
    if (pUrbOsX->pPrev)
        pUrbOsX->pPrev->pNext = pUrbOsX->pNext;
    else
    {
        Assert(pDevOsX->pInFlightHead == pUrbOsX);
        pDevOsX->pInFlightHead = pUrbOsX->pNext;
    }

    /*
     * Link it into the taxing list.
     */
    pUrbOsX->pNext = NULL;
    pUrbOsX->pPrev = pDevOsX->pTaxingTail;
    if (pDevOsX->pTaxingTail)
        pDevOsX->pTaxingTail->pNext = pUrbOsX;
    else
        pDevOsX->pTaxingHead = pUrbOsX;
    pDevOsX->pTaxingTail = pUrbOsX;

    RTCritSectLeave(&pDevOsX->CritSect);

    LogFlow(("%s: usbProxyDarwinUrbAsyncComplete: cb=%d EndPt=%#x irc=%#x (%d)\n",
             pUrb->pszDesc, cb, pUrb ? pUrb->EndPt : 0xff, irc, pUrb ? pUrb->enmStatus : 0xff));
}

/**
 * Release all interfaces (current config).
 *
 * @param   pDevOsX                 The darwin proxy device.
 */
static void usbProxyDarwinReleaseAllInterfaces(PUSBPROXYDEVOSX pDevOsX)
{
    PUSBPROXYIFOSX pIf = pDevOsX->pIfHead;
    pDevOsX->pIfHead = pDevOsX->pIfTail = NULL;

    while (pIf)
    {
        PUSBPROXYIFOSX pNext = pIf->pNext;
        IOReturn irc;

        if (pIf->RunLoopSrcRef)
        {
            CFRunLoopRemoveSource(pDevOsX->RunLoopRef, pIf->RunLoopSrcRef, g_pRunLoopMode);
            CFRelease(pIf->RunLoopSrcRef);
            pIf->RunLoopSrcRef = NULL;
        }

        while (pIf->pIsocBufCols)
        {
            PUSBPROXYISOCBUFCOL pCur = pIf->pIsocBufCols;
            pIf->pIsocBufCols = pCur->pNext;
            pCur->pNext = NULL;

            irc = (*pIf->ppIfI)->LowLatencyDestroyBuffer(pIf->ppIfI, pCur->pvBuffer);
            AssertMsg(irc == kIOReturnSuccess || irc == MACH_SEND_INVALID_DEST, ("%#x\n", irc));
            pCur->pvBuffer = NULL;

            irc = (*pIf->ppIfI)->LowLatencyDestroyBuffer(pIf->ppIfI, pCur->pvFrames);
            AssertMsg(irc == kIOReturnSuccess || irc == MACH_SEND_INVALID_DEST, ("%#x\n", irc));
            pCur->pvFrames = NULL;

            RTMemFree(pCur);
        }

        irc = (*pIf->ppIfI)->USBInterfaceClose(pIf->ppIfI);
        AssertMsg(irc == kIOReturnSuccess || irc == kIOReturnNoDevice, ("%#x\n", irc));

        (*pIf->ppIfI)->Release(pIf->ppIfI);
        pIf->ppIfI = NULL;

        RTMemFree(pIf);

        pIf = pNext;
    }
}


/**
 * Get the properties all the pipes associated with an interface.
 *
 * This is used when we seize all the interface and after SET_INTERFACE.
 *
 * @returns VBox status code.
 * @param   pDevOsX                 The darwin proxy device.
 * @param   pIf                     The interface to get pipe props for.
 */
static int usbProxyDarwinGetPipeProperties(PUSBPROXYDEVOSX pDevOsX, PUSBPROXYIFOSX pIf)
{
    /*
     * Get the pipe (endpoint) count (it might have changed - even on open).
     */
    int rc = VINF_SUCCESS;
    UInt8 cPipes;
    IOReturn irc = (*pIf->ppIfI)->GetNumEndpoints(pIf->ppIfI, &cPipes);
    if (irc != kIOReturnSuccess)
    {
        pIf->cPipes = 0;
        if (irc == kIOReturnNoDevice)
            rc = VERR_VUSB_DEVICE_NOT_ATTACHED;
        else
            rc = RTErrConvertFromDarwin(irc);
        return rc;
    }
    AssertRelease(cPipes < RT_ELEMENTS(pIf->aPipes));
    pIf->cPipes = cPipes + 1;

    /*
     * Get the properties of each pipe.
     */
    for (unsigned i = 0; i < pIf->cPipes; i++)
    {
        pIf->aPipes[i].u8PipeRef = i;
        pIf->aPipes[i].u64NextFrameNo = 0;
        irc = (*pIf->ppIfI)->GetPipeProperties(pIf->ppIfI, i,
                                               &pIf->aPipes[i].u8Direction,
                                               &pIf->aPipes[i].u8Endpoint,
                                               &pIf->aPipes[i].u8TransferType,
                                               &pIf->aPipes[i].u16MaxPacketSize,
                                               &pIf->aPipes[i].u8Interval);
        if (irc != kIOReturnSuccess)
        {
            LogRel(("USB: Failed to query properties for pipe %#d / interface %#x on device '%s'. (prot=%#x class=%#x)\n",
                    i, pIf->u8Interface, pDevOsX->pProxyDev->pUsbIns->pszName, pIf->u8Protocol, pIf->u8Class));
            if (irc == kIOReturnNoDevice)
                rc = VERR_VUSB_DEVICE_NOT_ATTACHED;
            else
                rc = RTErrConvertFromDarwin(irc);
            pIf->cPipes = i;
            break;
        }
        /* reconstruct bEndpoint */
        if (pIf->aPipes[i].u8Direction == kUSBIn)
            pIf->aPipes[i].u8Endpoint |= 0x80;
        Log2(("usbProxyDarwinGetPipeProperties: #If=%d EndPt=%#x Dir=%d Type=%d PipeRef=%#x MaxPktSize=%#x Interval=%#x\n",
              pIf->u8Interface, pIf->aPipes[i].u8Endpoint, pIf->aPipes[i].u8Direction, pIf->aPipes[i].u8TransferType,
              pIf->aPipes[i].u8PipeRef, pIf->aPipes[i].u16MaxPacketSize, pIf->aPipes[i].u8Interval));
    }

    /** @todo sort or hash these for speedy lookup... */
    return VINF_SUCCESS;
}


/**
 * Seize all interfaces (current config).
 *
 * @returns VBox status code.
 * @param   pDevOsX                 The darwin proxy device.
 * @param   fMakeTheBestOfIt        If set we will not give up on error. This is for
 *                                  use during SET_CONFIGURATION and similar.
 */
static int usbProxyDarwinSeizeAllInterfaces(PUSBPROXYDEVOSX pDevOsX, bool fMakeTheBestOfIt)
{
    PUSBPROXYDEV pProxyDev = pDevOsX->pProxyDev;

    /*
     * Create a interface enumerator for all the interface (current config).
     */
    io_iterator_t Interfaces = NULL;
    IOUSBFindInterfaceRequest Req;
    Req.bInterfaceClass    = kIOUSBFindInterfaceDontCare;
    Req.bInterfaceSubClass = kIOUSBFindInterfaceDontCare;
    Req.bInterfaceProtocol = kIOUSBFindInterfaceDontCare;
    Req.bAlternateSetting  = kIOUSBFindInterfaceDontCare;
    IOReturn irc = (*pDevOsX->ppDevI)->CreateInterfaceIterator(pDevOsX->ppDevI, &Req, &Interfaces);
    int rc;
    if (irc == kIOReturnSuccess)
    {
        /*
         * Iterate the interfaces.
         */
        io_object_t Interface;
        rc = VINF_SUCCESS;
        while ((Interface = IOIteratorNext(Interfaces)))
        {
            /*
             * Create a plug-in and query the IOUSBInterfaceInterface (cute name).
             */
            IOCFPlugInInterface **ppPlugInInterface = NULL;
            kern_return_t krc;
            SInt32 Score = 0;
            krc = IOCreatePlugInInterfaceForService(Interface, kIOUSBInterfaceUserClientTypeID,
                                                    kIOCFPlugInInterfaceID, &ppPlugInInterface, &Score);
            IOObjectRelease(Interface);
            Interface = NULL;
            if (krc == KERN_SUCCESS)
            {
                IOUSBInterfaceInterface245 **ppIfI;
                HRESULT hrc = (*ppPlugInInterface)->QueryInterface(ppPlugInInterface,
                                                                   CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID245),
                                                                   (LPVOID *)&ppIfI);
                krc = IODestroyPlugInInterface(ppPlugInInterface); Assert(krc == KERN_SUCCESS);
                ppPlugInInterface = NULL;
                if (hrc == S_OK)
                {
                    /*
                     * Query some basic properties first.
                     * (This means we can print more informative messages on failure
                     * to seize the interface.)
                     */
                    UInt8 u8Interface = 0xff;
                    irc = (*ppIfI)->GetInterfaceNumber(ppIfI, &u8Interface);
                    UInt8 u8AltSetting = 0xff;
                    if (irc == kIOReturnSuccess)
                        irc = (*ppIfI)->GetAlternateSetting(ppIfI, &u8AltSetting);
                    UInt8 u8Class = 0xff;
                    if (irc == kIOReturnSuccess)
                        irc = (*ppIfI)->GetInterfaceClass(ppIfI, &u8Class);
                    UInt8 u8Protocol = 0xff;
                    if (irc == kIOReturnSuccess)
                        irc = (*ppIfI)->GetInterfaceProtocol(ppIfI, &u8Protocol);
                    UInt8 cEndpoints = 0;
                    if (irc == kIOReturnSuccess)
                        irc = (*ppIfI)->GetNumEndpoints(ppIfI, &cEndpoints);
                    if (irc == kIOReturnSuccess)
                    {
                        /*
                         * Try seize the interface.
                         */
                        irc = (*ppIfI)->USBInterfaceOpenSeize(ppIfI);
                        if (irc == kIOReturnSuccess)
                        {
                            PUSBPROXYIFOSX pIf = (PUSBPROXYIFOSX)RTMemAllocZ(sizeof(*pIf));
                            if (pIf)
                            {
                                /*
                                 * Create the per-interface entry and query the
                                 * endpoint data.
                                 */
                                /* initialize the entry */
                                pIf->u8Interface = u8Interface;
                                pIf->u8AltSetting = u8AltSetting;
                                pIf->u8Class     = u8Class;
                                pIf->u8Protocol  = u8Protocol;
                                pIf->cPipes      = cEndpoints;
                                pIf->ppIfI       = ppIfI;

                                /* query pipe/endpoint properties. */
                                rc = usbProxyDarwinGetPipeProperties(pDevOsX, pIf);
                                if (RT_SUCCESS(rc))
                                {
                                    /*
                                     * Create the async event source and add it to the
                                     * default current run loop.
                                     * (Later: Add to the worker thread run loop instead.)
                                     */
                                    irc = (*ppIfI)->CreateInterfaceAsyncEventSource(ppIfI, &pIf->RunLoopSrcRef);
                                    if (irc == kIOReturnSuccess)
                                    {
                                        CFRunLoopAddSource(pDevOsX->RunLoopRef, pIf->RunLoopSrcRef, g_pRunLoopMode);

                                        /*
                                         * Just link the interface into the list and we're good.
                                         */
                                        pIf->pNext = NULL;
                                        Log(("USB: Seized interface %#x (alt=%d prot=%#x class=%#x)\n",
                                             u8Interface, u8AltSetting, u8Protocol, u8Class));
                                        if (pDevOsX->pIfTail)
                                            pDevOsX->pIfTail = pDevOsX->pIfTail->pNext = pIf;
                                        else
                                            pDevOsX->pIfTail = pDevOsX->pIfHead = pIf;
                                        continue;
                                    }
                                    rc = RTErrConvertFromDarwin(irc);
                                }

                                /* failure cleanup. */
                                RTMemFree(pIf);
                            }
                        }
                        else if (irc == kIOReturnExclusiveAccess)
                        {
                            LogRel(("USB: Interface %#x on device '%s' is being used by another process. (prot=%#x class=%#x)\n",
                                    u8Interface, pProxyDev->pUsbIns->pszName, u8Protocol, u8Class));
                            rc = VERR_SHARING_VIOLATION;
                        }
                        else
                        {
                            LogRel(("USB: Failed to open interface %#x on device '%s'. (prot=%#x class=%#x) krc=%#x\n",
                                    u8Interface, pProxyDev->pUsbIns->pszName, u8Protocol, u8Class, irc));
                            rc = VERR_OPEN_FAILED;
                        }
                    }
                    else
                    {
                        rc = RTErrConvertFromDarwin(irc);
                        LogRel(("USB: Failed to query interface properties on device '%s', irc=%#x.\n",
                                pProxyDev->pUsbIns->pszName, irc));
                    }
                    (*ppIfI)->Release(ppIfI);
                    ppIfI = NULL;
                }
                else if (RT_SUCCESS(rc))
                    rc = RTErrConvertFromDarwinCOM(hrc);
            }
            else if (RT_SUCCESS(rc))
                rc = RTErrConvertFromDarwin(krc);
            if (!fMakeTheBestOfIt)
            {
                usbProxyDarwinReleaseAllInterfaces(pDevOsX);
                break;
            }
        } /* iterate */
        IOObjectRelease(Interfaces);
    }
    else if (irc == kIOReturnNoDevice)
        rc = VERR_VUSB_DEVICE_NOT_ATTACHED;
    else
    {
        AssertMsgFailed(("%#x\n", irc));
        rc = VERR_GENERAL_FAILURE;
    }
    return rc;
}


/**
 * Find a particular interface.
 *
 * @returns The requested interface or NULL if not found.
 * @param   pDevOsX                 The darwin proxy device.
 * @param   u8Interface             The interface number.
 */
static PUSBPROXYIFOSX usbProxyDarwinGetInterface(PUSBPROXYDEVOSX pDevOsX, uint8_t u8Interface)
{
    if (!pDevOsX->pIfHead)
        usbProxyDarwinSeizeAllInterfaces(pDevOsX, true /* make the best out of it */);

    PUSBPROXYIFOSX pIf;
    for (pIf = pDevOsX->pIfHead; pIf; pIf = pIf->pNext)
        if (pIf->u8Interface == u8Interface)
            return pIf;

/*    AssertMsgFailed(("Cannot find If#=%d\n", u8Interface)); - the 3rd quickcam interface is capture by the ****ing audio crap. */
    return NULL;
}


/**
 * Find a particular endpoint.
 *
 * @returns The requested interface or NULL if not found.
 * @param   pDevOsX                 The darwin proxy device.
 * @param   u8Endpoint              The endpoint.
 * @param   pu8PipeRef              Where to store the darwin pipe ref.
 * @param   ppPipe                  Where to store the darwin pipe pointer. (optional)
 */
static PUSBPROXYIFOSX usbProxyDarwinGetInterfaceForEndpoint(PUSBPROXYDEVOSX pDevOsX, uint8_t u8Endpoint,
                                                            uint8_t *pu8PipeRef, PPUSBPROXYPIPEOSX ppPipe)
{
    if (!pDevOsX->pIfHead)
        usbProxyDarwinSeizeAllInterfaces(pDevOsX, true /* make the best out of it */);

    PUSBPROXYIFOSX pIf;
    for (pIf = pDevOsX->pIfHead; pIf; pIf = pIf->pNext)
    {
        unsigned i = pIf->cPipes;
        while (i-- > 0)
            if (pIf->aPipes[i].u8Endpoint == u8Endpoint)
            {
                *pu8PipeRef = pIf->aPipes[i].u8PipeRef;
                if (ppPipe)
                    *ppPipe = &pIf->aPipes[i];
                return pIf;
            }
    }

    AssertMsgFailed(("Cannot find EndPt=%#x\n", u8Endpoint));
    return NULL;
}


/**
 * Gets an unsigned 32-bit integer value.
 *
 * @returns Success indicator (true/false).
 * @param   DictRef     The dictionary.
 * @param   KeyStrRef   The key name.
 * @param   pu32        Where to store the key value.
 */
static bool usbProxyDarwinDictGetU32(CFMutableDictionaryRef DictRef, CFStringRef KeyStrRef, uint32_t *pu32)
{
    CFTypeRef ValRef = CFDictionaryGetValue(DictRef, KeyStrRef);
    if (ValRef)
    {
        if (CFNumberGetValue((CFNumberRef)ValRef, kCFNumberSInt32Type, pu32))
            return true;
    }
    *pu32 = 0;
    return false;
}


/**
 * Gets an unsigned 64-bit integer value.
 *
 * @returns Success indicator (true/false).
 * @param   DictRef     The dictionary.
 * @param   KeyStrRef   The key name.
 * @param   pu64        Where to store the key value.
 */
static bool usbProxyDarwinDictGetU64(CFMutableDictionaryRef DictRef, CFStringRef KeyStrRef, uint64_t *pu64)
{
    CFTypeRef ValRef = CFDictionaryGetValue(DictRef, KeyStrRef);
    if (ValRef)
    {
        if (CFNumberGetValue((CFNumberRef)ValRef, kCFNumberSInt64Type, pu64))
            return true;
    }
    *pu64 = 0;
    return false;
}


/* -=-=-=-=-=- The exported methods -=-=-=-=-=- */


/**
 * Opens the USB Device.
 *
 * @returns VBox status code.
 * @param   pProxyDev       The device instance.
 * @param   pszAddress      The session id and/or location id of the device to open.
 *                          The format of this string is something iokit.c in Main defines, currently
 *                          it's sequences of "[l|s]=<value>" separated by ";".
 * @param   pvBackend       Backend specific pointer, unused for the Darwin backend.
 */
static int usbProxyDarwinOpen(PUSBPROXYDEV pProxyDev, const char *pszAddress, void *pvBackend)
{
    int vrc;
    LogFlow(("usbProxyDarwinOpen: pProxyDev=%p pszAddress=%s\n", pProxyDev, pszAddress));

    /*
     * Init globals once.
     */
    vrc = RTOnce(&g_usbProxyDarwinOnce, usbProxyDarwinInitOnce, NULL, NULL);
    AssertRCReturn(vrc, vrc);

    /*
     * The idea here was to create a matching directory with the sessionID
     * and locationID included, however this doesn't seem to work. So, we'll
     * use the product id and vendor id to limit the set of matching device
     * and manually match these two properties. sigh.
     * (Btw. vendor and product id must be used *together* apparently.)
     *
     * Wonder if we could use the entry path? Docs indicates says we must
     * use IOServiceGetMatchingServices and I'm not in a mood to explore
     * this subject further right now. Maybe check this later.
     */
    CFMutableDictionaryRef RefMatchingDict = IOServiceMatching(kIOUSBDeviceClassName);
    AssertReturn(RefMatchingDict, NULL);

    uint64_t u64SessionId = 0;
    uint32_t u32LocationId = 0;
    const char *psz = pszAddress;
    do
    {
        const char chValue = *psz;
        AssertReleaseReturn(psz[1] == '=', VERR_INTERNAL_ERROR);
        uint64_t u64Value;
        int rc = RTStrToUInt64Ex(psz + 2, (char **)&psz, 0, &u64Value);
        AssertReleaseRCReturn(rc, rc);
        AssertReleaseReturn(!*psz || *psz == ';', rc);
        switch (chValue)
        {
            case 'l':
                u32LocationId = (uint32_t)u64Value;
                break;
            case 's':
                u64SessionId = u64Value;
                break;
            case 'p':
            case 'v':
            {
#if 0 /* Guess what, this doesn't 'ing work either! */
                SInt32 i32 = (int16_t)u64Value;
                CFNumberRef Num = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &i32);
                AssertBreak(Num);
                CFDictionarySetValue(RefMatchingDict, chValue == 'p' ? CFSTR(kUSBProductID) : CFSTR(kUSBVendorID), Num);
                CFRelease(Num);
#endif
                break;
            }
            default:
                AssertReleaseMsgFailedReturn(("chValue=%#x\n", chValue), VERR_INTERNAL_ERROR);
        }
        if (*psz == ';')
            psz++;
    } while (*psz);

    io_iterator_t USBDevices = NULL;
    IOReturn irc = IOServiceGetMatchingServices(g_MasterPort, RefMatchingDict, &USBDevices);
    AssertMsgReturn(irc == kIOReturnSuccess, ("irc=%#x\n", irc), NULL);
    RefMatchingDict = NULL; /* the reference is consumed by IOServiceGetMatchingServices. */

    unsigned cMatches = 0;
    io_object_t USBDevice;
    while ((USBDevice = IOIteratorNext(USBDevices)))
    {
        cMatches++;
        CFMutableDictionaryRef PropsRef = 0;
        kern_return_t krc = IORegistryEntryCreateCFProperties(USBDevice, &PropsRef, kCFAllocatorDefault, kNilOptions);
        if (krc == KERN_SUCCESS)
        {
            uint64_t u64CurSessionId;
            uint32_t u32CurLocationId;
            if (    (    !u64SessionId
                     || (   usbProxyDarwinDictGetU64(PropsRef, CFSTR("sessionID"), &u64CurSessionId)
                         && u64CurSessionId == u64SessionId))
                &&  (   !u32LocationId
                     || (   usbProxyDarwinDictGetU32(PropsRef, CFSTR(kUSBDevicePropertyLocationID), &u32CurLocationId)
                         && u32CurLocationId == u32LocationId))
                )
            {
                CFRelease(PropsRef);
                break;
            }
            CFRelease(PropsRef);
        }
        IOObjectRelease(USBDevice);
    }
    IOObjectRelease(USBDevices);
    USBDevices = NULL;
    if (!USBDevice)
    {
        LogRel(("USB: Device '%s' not found (%d pid+vid matches)\n", pszAddress, cMatches));
        IOObjectRelease(USBDevices);
        return VERR_VUSB_DEVICE_NAME_NOT_FOUND;
    }

#ifdef VBOX_WITH_NEW_USB_CODE_ON_DARWIN
    /*
     * Call the USBLib init to make sure we're a valid VBoxUSB client.
     * For now we'll ignore failures here and just plunge on, it might still work...
     */
    vrc = USBLibInit();
    if (RT_FAILURE(vrc))
        LogRel(("USB: USBLibInit failed - %Rrc\n", vrc));
#endif

    /*
     * Create a plugin interface for the device and query its IOUSBDeviceInterface.
     */
    SInt32 Score = 0;
    IOCFPlugInInterface **ppPlugInInterface = NULL;
    irc = IOCreatePlugInInterfaceForService(USBDevice, kIOUSBDeviceUserClientTypeID,
                                            kIOCFPlugInInterfaceID, &ppPlugInInterface, &Score);
    if (irc == kIOReturnSuccess)
    {
        IOUSBDeviceInterface245 **ppDevI = NULL;
        HRESULT hrc = (*ppPlugInInterface)->QueryInterface(ppPlugInInterface,
                                                           CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID245),
                                                           (LPVOID *)&ppDevI);
        irc = IODestroyPlugInInterface(ppPlugInInterface); Assert(irc == kIOReturnSuccess);
        ppPlugInInterface = NULL;
        if (hrc == S_OK)
        {
            /*
             * Try open the device for exclusive access.
             * If we fail, we'll try figure out who is using the device and
             * convince them to let go of it...
             */
            irc = (*ppDevI)->USBDeviceOpenSeize(ppDevI);
            if (irc == kIOReturnExclusiveAccess)
            {
                RTThreadSleep(20);
                irc = (*ppDevI)->USBDeviceOpenSeize(ppDevI);
            }
            if (irc == kIOReturnSuccess)
            {
                /*
                 * Create a proxy device instance.
                 */
                vrc = VERR_NO_MEMORY;
                PUSBPROXYDEVOSX pDevOsX = (PUSBPROXYDEVOSX)RTMemAllocZ(sizeof(*pDevOsX));
                if (pDevOsX)
                {
                    vrc = RTCritSectInit(&pDevOsX->CritSect);
                    if (RT_SUCCESS(vrc))
                    {
                        pDevOsX->USBDevice = USBDevice;
                        pDevOsX->ppDevI = ppDevI;
                        pDevOsX->pProxyDev = pProxyDev;
                        pDevOsX->RunLoopRef = CFRunLoopGetCurrent();
                        CFRetain(pDevOsX->RunLoopRef); /* paranoia */

                        /*
                         * Try seize all the interface.
                         */
                        char *pszDummyName = pProxyDev->pUsbIns->pszName;
                        pProxyDev->pUsbIns->pszName = (char *)pszAddress;
                        vrc = usbProxyDarwinSeizeAllInterfaces(pDevOsX, false /* give up on failure */);
                        pProxyDev->pUsbIns->pszName = pszDummyName;
                        if (RT_SUCCESS(vrc))
                        {
                            /*
                             * Create the async event source and add it to the run loop.
                             */
                            irc = (*ppDevI)->CreateDeviceAsyncEventSource(ppDevI, &pDevOsX->RunLoopSrcRef);
                            if (irc == kIOReturnSuccess)
                            {
                                CFRunLoopAddSource(pDevOsX->RunLoopRef, pDevOsX->RunLoopSrcRef, g_pRunLoopMode);

                                /*
                                 * Determine the active configuration.
                                 * Can cause hangs, so drop it for now.
                                 */
                                /** @todo test Palm. */
                                //uint8_t u8Cfg;
                                //irc = (*ppDevI)->GetConfiguration(ppDevI, &u8Cfg);
                                if (irc != kIOReturnNoDevice)
                                {
                                    //pProxyDev->iActiveCfg = irc == kIOReturnSuccess ? u8Cfg : -1;
                                    pProxyDev->iActiveCfg = -1;
                                    pProxyDev->cIgnoreSetConfigs = 1;

                                    pProxyDev->Backend.pv = pDevOsX;
                                    return VINF_SUCCESS;        /* return */
                                }
                                vrc = VERR_VUSB_DEVICE_NOT_ATTACHED;

                                CFRunLoopRemoveSource(pDevOsX->RunLoopRef, pDevOsX->RunLoopSrcRef, g_pRunLoopMode);
                            }
                            else
                                vrc = RTErrConvertFromDarwin(irc);

                            usbProxyDarwinReleaseAllInterfaces(pDevOsX);
                        }
                        /* else: already bitched */

                        RTCritSectDelete(&pDevOsX->CritSect);
                    }

                    RTMemFree(pDevOsX);
                }

                irc = (*ppDevI)->USBDeviceClose(ppDevI);
                AssertMsg(irc == kIOReturnSuccess, ("%#x\n", irc));
            }
            else if (irc == kIOReturnExclusiveAccess)
            {
                LogRel(("USB: Device '%s' is being used by another process\n", pszAddress));
                vrc = VERR_SHARING_VIOLATION;
            }
            else
            {
                LogRel(("USB: Failed to open device '%s', irc=%#x.\n", pszAddress, irc));
                vrc = VERR_OPEN_FAILED;
            }
        }
        else
        {
            LogRel(("USB: Failed to create plugin interface for device '%s', hrc=%#x.\n", pszAddress, hrc));
            vrc = VERR_OPEN_FAILED;
        }

        (*ppDevI)->Release(ppDevI);
    }
    else
    {
        LogRel(("USB: Failed to open device '%s', plug-in creation failed with irc=%#x.\n", pszAddress, irc));
        vrc = RTErrConvertFromDarwin(irc);
    }

#ifdef VBOX_WITH_NEW_USB_CODE_ON_DARWIN
    USBLibTerm();
#endif
    return vrc;
}


/**
 * Closes the proxy device.
 */
static void usbProxyDarwinClose(PUSBPROXYDEV pProxyDev)
{
    LogFlow(("usbProxyDarwinClose: pProxyDev=%s\n", pProxyDev->pUsbIns->pszName));
    PUSBPROXYDEVOSX pDevOsX = (PUSBPROXYDEVOSX)pProxyDev->Backend.pv;
    Assert(pDevOsX);
    if (!pDevOsX)
        return;

    /*
     * Release interfaces we've laid claim to, then reset the device
     * and finally close it.
     */
    RTCritSectEnter(&pDevOsX->CritSect);
    /* ?? */
    RTCritSectLeave(&pDevOsX->CritSect);

    usbProxyDarwinReleaseAllInterfaces(pDevOsX);

    if (pDevOsX->RunLoopSrcRef)
    {
        CFRunLoopRemoveSource(pDevOsX->RunLoopRef, pDevOsX->RunLoopSrcRef, g_pRunLoopMode);
        CFRelease(pDevOsX->RunLoopSrcRef);
        pDevOsX->RunLoopSrcRef = NULL;
    }

    IOReturn irc = (*pDevOsX->ppDevI)->ResetDevice(pDevOsX->ppDevI);
#ifndef VBOX_WITH_NEW_USB_CODE_ON_DARWIN
    if (irc == kIOReturnSuccess)
        irc = (*pDevOsX->ppDevI)->USBDeviceReEnumerate(pDevOsX->ppDevI, 0);
#endif

    irc = (*pDevOsX->ppDevI)->USBDeviceClose(pDevOsX->ppDevI);
    if (irc != kIOReturnSuccess && irc != kIOReturnNoDevice)
    {
        LogRel(("USB: USBDeviceClose -> %#x\n", irc));
        AssertMsgFailed(("irc=%#x\n", irc));
    }

    (*pDevOsX->ppDevI)->Release(pDevOsX->ppDevI);
    pDevOsX->ppDevI = NULL;
    kern_return_t krc = IOObjectRelease(pDevOsX->USBDevice); Assert(krc == KERN_SUCCESS); NOREF(krc);
    pDevOsX->USBDevice = NULL;
    pDevOsX->pProxyDev = NULL;

    /*
     * Free all the resources.
     */
    RTCritSectDelete(&pDevOsX->CritSect);

    if (pDevOsX->RunLoopRef)
    {
        CFRelease(pDevOsX->RunLoopRef);
        pDevOsX->RunLoopRef = NULL;
    }

    PUSBPROXYURBOSX pUrbOsX;
    while ((pUrbOsX = pDevOsX->pInFlightHead) != NULL)
    {
        pDevOsX->pInFlightHead = pUrbOsX->pNext;
        //RTMemFree(pUrbOsX); - leak these for now, fix later.
    }

    while ((pUrbOsX = pDevOsX->pFreeHead) != NULL)
    {
        pDevOsX->pFreeHead = pUrbOsX->pNext;
        RTMemFree(pUrbOsX);
    }

    RTMemFree(pDevOsX);
    pProxyDev->Backend.pv = NULL;

#ifdef VBOX_WITH_NEW_USB_CODE_ON_DARWIN
    USBLibTerm();
#endif
    LogFlow(("usbProxyDarwinClose: returns\n"));
}


/**
 * Reset a device.
 *
 * @returns VBox status code.
 * @param   pDev    The device to reset.
 */
static int usbProxyDarwinReset(PUSBPROXYDEV pProxyDev, bool fResetOnLinux)
{
    PUSBPROXYDEVOSX pDevOsX = (PUSBPROXYDEVOSX)pProxyDev->Backend.pv;
    LogFlow(("usbProxyDarwinReset: pProxyDev=%s\n", pProxyDev->pUsbIns->pszName));

    IOReturn irc = (*pDevOsX->ppDevI)->ResetDevice(pDevOsX->ppDevI);
    int rc;
    if (irc == kIOReturnSuccess)
    {
        /** @todo Some docs say that some drivers will do a default config, check this out ... */
        pProxyDev->cIgnoreSetConfigs = 0;
        pProxyDev->iActiveCfg = -1;

        rc = VINF_SUCCESS;
    }
    else if (irc == kIOReturnNoDevice)
        rc = VERR_VUSB_DEVICE_NOT_ATTACHED;
    else
    {
        AssertMsgFailed(("irc=%#x\n", irc));
        rc = VERR_GENERAL_FAILURE;
    }

    LogFlow(("usbProxyDarwinReset: returns success %Rrc\n", rc));
    return rc;
}


/**
 * SET_CONFIGURATION.
 *
 * The caller makes sure that it's not called first time after open or reset
 * with the active interface.
 *
 * @returns success indicator.
 * @param   pProxyDev       The device instance data.
 * @param   iCfg            The configuration to set.
 */
static int usbProxyDarwinSetConfig(PUSBPROXYDEV pProxyDev, int iCfg)
{
    PUSBPROXYDEVOSX pDevOsX = (PUSBPROXYDEVOSX)pProxyDev->Backend.pv;
    LogFlow(("usbProxyDarwinSetConfig: pProxyDev=%s cfg=%#x\n",
             pProxyDev->pUsbIns->pszName, iCfg));

    IOReturn irc = (*pDevOsX->ppDevI)->SetConfiguration(pDevOsX->ppDevI, (uint8_t)iCfg);
    if (irc != kIOReturnSuccess)
    {
        Log(("usbProxyDarwinSetConfig: Set configuration -> %#x\n", irc));
        return false;
    }

    usbProxyDarwinReleaseAllInterfaces(pDevOsX);
    usbProxyDarwinSeizeAllInterfaces(pDevOsX, true /* make the best out of it */);
    return true;
}


/**
 * Claims an interface.
 *
 * This is a stub on Darwin since we release/claim all interfaces at
 * open/reset/setconfig time.
 *
 * @returns success indicator (always true).
 */
static int usbProxyDarwinClaimInterface(PUSBPROXYDEV pProxyDev, int iIf)
{
    return true;
}


/**
 * Releases an interface.
 *
 * This is a stub on Darwin since we release/claim all interfaces at
 * open/reset/setconfig time.
 *
 * @returns success indicator.
 */
static int usbProxyDarwinReleaseInterface(PUSBPROXYDEV pProxyDev, int iIf)
{
    return true;
}


/**
 * SET_INTERFACE.
 *
 * @returns success indicator.
 */
static int usbProxyDarwinSetInterface(PUSBPROXYDEV pProxyDev, int iIf, int iAlt)
{
    PUSBPROXYDEVOSX pDevOsX = (PUSBPROXYDEVOSX)pProxyDev->Backend.pv;
    IOReturn irc = kIOReturnSuccess;
    PUSBPROXYIFOSX pIf = usbProxyDarwinGetInterface(pDevOsX, iIf);
    LogFlow(("usbProxyDarwinSetInterface: pProxyDev=%s iIf=%#x iAlt=%#x iCurAlt=%#x\n",
             pProxyDev->pUsbIns->pszName, iIf, iAlt, pIf ? pIf->u8AltSetting : 0xbeef));
    if (pIf)
    {
        /* Avoid SetAlternateInterface when possible as it will recreate the pipes. */
        if (iAlt != pIf->u8AltSetting)
        {
            irc = (*pIf->ppIfI)->SetAlternateInterface(pIf->ppIfI, iAlt);
            if (irc == kIOReturnSuccess)
            {
                usbProxyDarwinGetPipeProperties(pDevOsX, pIf);
                return true;
            }
        }
        else
        {
            /*
             * Just send the request anyway?
             */
            IOUSBDevRequest Req;
            Req.bmRequestType = 0x01;
            Req.bRequest = 0x0b; /* SET_INTERFACE */
            Req.wIndex = iIf;
            Req.wValue = iAlt;
            Req.wLength = 0;
            Req.wLenDone = 0;
            Req.pData = NULL;
            irc = (*pDevOsX->ppDevI)->DeviceRequest(pDevOsX->ppDevI, &Req);
            Log(("usbProxyDarwinSetInterface: SET_INTERFACE(%d,%d) -> irc=%#x\n", iIf, iAlt, irc));
            return true;
        }
    }

    LogFlow(("usbProxyDarwinSetInterface: pProxyDev=%s eiIf=%#x iAlt=%#x - failure - pIf=%p irc=%#x\n",
             pProxyDev->pUsbIns->pszName, iIf, iAlt, pIf, irc));
    return false;
}


/**
 * Clears the halted endpoint 'EndPt'.
 */
static bool usbProxyDarwinClearHaltedEp(PUSBPROXYDEV pProxyDev, unsigned int EndPt)
{
    PUSBPROXYDEVOSX pDevOsX = (PUSBPROXYDEVOSX)pProxyDev->Backend.pv;
    LogFlow(("usbProxyDarwinClearHaltedEp: pProxyDev=%s EndPt=%#x\n", pProxyDev->pUsbIns->pszName, EndPt));

    /*
     * Clearing the zero control pipe doesn't make sense and isn't
     * supported by the API. Just ignore it.
     */
    if (EndPt == 0)
        return true;

    /*
     * Find the interface/pipe combination and invoke the ClearPipeStallBothEnds
     * method. (The ResetPipe and ClearPipeStall methods will not send the
     * CLEAR_FEATURE(ENDPOINT_HALT) request that this method implements.)
     */
    IOReturn irc = kIOReturnSuccess;
    uint8_t u8PipeRef;
    PUSBPROXYIFOSX pIf = usbProxyDarwinGetInterfaceForEndpoint(pDevOsX, EndPt, &u8PipeRef, NULL);
    if (pIf)
    {
        irc = (*pIf->ppIfI)->ClearPipeStallBothEnds(pIf->ppIfI, u8PipeRef);
        if (irc == kIOReturnSuccess)
            return true;
        AssertMsg(irc == kIOReturnNoDevice || irc == kIOReturnNotResponding, ("irc=#x (control pipe?)\n", irc));
    }

    LogFlow(("usbProxyDarwinClearHaltedEp: pProxyDev=%s EndPt=%#x - failure - pIf=%p irc=%#x\n",
             pProxyDev->pUsbIns->pszName, EndPt, pIf, irc));
    return false;
}


/**
 * @copydoc USBPROXYBACK::pfnUrbQueue
 */
static int usbProxyDarwinUrbQueue(PVUSBURB pUrb)
{
    PUSBPROXYDEV    pProxyDev = PDMINS_2_DATA(pUrb->pUsbIns, PUSBPROXYDEV);
    PUSBPROXYDEVOSX pDevOsX = (PUSBPROXYDEVOSX)pProxyDev->Backend.pv;
    LogFlow(("%s: usbProxyDarwinUrbQueue: pProxyDev=%s pUrb=%p EndPt=%d cbData=%d\n",
             pUrb->pszDesc, pProxyDev->pUsbIns->pszName, pUrb, pUrb->EndPt, pUrb->cbData));

    /*
     * Find the target interface / pipe.
     */
    uint8_t u8PipeRef = 0xff;
    PUSBPROXYIFOSX pIf = NULL;
    PUSBPROXYPIPEOSX pPipe = NULL;
    if (pUrb->EndPt)
    {
        const uint8_t EndPt = pUrb->EndPt | (pUrb->enmDir == VUSBDIRECTION_IN ? 0x80 : 0);
        pIf = usbProxyDarwinGetInterfaceForEndpoint(pDevOsX, EndPt, &u8PipeRef, &pPipe);
        if (!pIf)
        {
            LogFlow(("%s: usbProxyDarwinUrbQueue: pProxyDev=%s EndPt=%d cbData=%d - can't find interface / pipe!!!\n",
                     pUrb->pszDesc, pProxyDev->pUsbIns->pszName, pUrb->EndPt, pUrb->cbData));
            return false;
        }
    }
    /* else: pIf == NULL -> default control pipe.*/

    /*
     * Allocate a Darwin urb.
     */
    PUSBPROXYURBOSX pUrbOsX = usbProxyDarwinUrbAlloc(pDevOsX);
    if (!pUrbOsX)
        return false;

    pUrbOsX->u64SubmitTS = RTTimeMilliTS();
    pUrbOsX->pVUsbUrb = pUrb;
    pUrbOsX->pDevOsX = pDevOsX;
    pUrbOsX->enmType = pUrb->enmType;

    /*
     * Submit the request.
     */
    IOReturn irc = kIOReturnError;
    switch (pUrb->enmType)
    {
        case VUSBXFERTYPE_MSG:
        {
            AssertMsgBreak(pUrb->cbData >= sizeof(VUSBSETUP), ("cbData=%d\n", pUrb->cbData));
            PVUSBSETUP pSetup = (PVUSBSETUP)&pUrb->abData[0];
            pUrbOsX->u.ControlMsg.bmRequestType = pSetup->bmRequestType;
            pUrbOsX->u.ControlMsg.bRequest      = pSetup->bRequest;
            pUrbOsX->u.ControlMsg.wValue        = pSetup->wValue;
            pUrbOsX->u.ControlMsg.wIndex        = pSetup->wIndex;
            pUrbOsX->u.ControlMsg.wLength       = pSetup->wLength;
            pUrbOsX->u.ControlMsg.pData         = pSetup + 1;
            pUrbOsX->u.ControlMsg.wLenDone      = pSetup->wLength;

            if (pIf)
                irc = (*pIf->ppIfI)->ControlRequestAsync(pIf->ppIfI, u8PipeRef, &pUrbOsX->u.ControlMsg,
                                                         usbProxyDarwinUrbAsyncComplete, pUrbOsX);
            else
                irc = (*pDevOsX->ppDevI)->DeviceRequestAsync(pDevOsX->ppDevI, &pUrbOsX->u.ControlMsg,
                                                             usbProxyDarwinUrbAsyncComplete, pUrbOsX);
            break;
        }

        case VUSBXFERTYPE_BULK:
        case VUSBXFERTYPE_INTR:
        {
            AssertBreak(pIf);
            Assert(pUrb->enmDir == VUSBDIRECTION_IN || pUrb->enmDir == VUSBDIRECTION_OUT);
            if (pUrb->enmDir == VUSBDIRECTION_OUT)
                irc = (*pIf->ppIfI)->WritePipeAsync(pIf->ppIfI, u8PipeRef, pUrb->abData, pUrb->cbData,
                                                    usbProxyDarwinUrbAsyncComplete, pUrbOsX);
            else
                irc = (*pIf->ppIfI)->ReadPipeAsync(pIf->ppIfI, u8PipeRef, pUrb->abData, pUrb->cbData,
                                                   usbProxyDarwinUrbAsyncComplete, pUrbOsX);

            break;
        }

        case VUSBXFERTYPE_ISOC:
        {
            AssertBreak(pIf);
            Assert(pUrb->enmDir == VUSBDIRECTION_IN || pUrb->enmDir == VUSBDIRECTION_OUT);

#ifdef USE_LOW_LATENCY_API
            /* Allocate an isochronous buffer and copy over the data. */
            AssertBreak(pUrb->cbData <= 8192);
            int rc = usbProxyDarwinUrbAllocIsocBuf(pUrbOsX, pIf);
            AssertRCBreak(rc);
            if (pUrb->enmDir == VUSBDIRECTION_OUT)
                memcpy(pUrbOsX->u.Isoc.pBuf->pvBuf, pUrb->abData, pUrb->cbData);
            else
                memset(pUrbOsX->u.Isoc.pBuf->pvBuf, 0xfe, pUrb->cbData);
#endif

            /* Get the current frame number (+2) and make sure it doesn't
               overlap with the previous request. See WARNING in
               ApplUSBUHCI::CreateIsochTransfer for details on the +2. */
            UInt64 FrameNo;
            AbsoluteTime FrameTime;
            irc = (*pIf->ppIfI)->GetBusFrameNumber(pIf->ppIfI, &FrameNo, &FrameTime);
            AssertMsg(irc == kIOReturnSuccess, ("GetBusFrameNumber -> %#x\n", irc));
            FrameNo += 2;
            if (FrameNo <= pPipe->u64NextFrameNo)
                FrameNo = pPipe->u64NextFrameNo;

            for (unsigned j = 0; ; j++)
            {
                unsigned i;
                for (i = 0; i < pUrb->cIsocPkts; i++)
                {
                    pUrbOsX->u.Isoc.aFrames[i].frReqCount = pUrb->aIsocPkts[i].cb;
                    pUrbOsX->u.Isoc.aFrames[i].frActCount = 0;
                    pUrbOsX->u.Isoc.aFrames[i].frStatus = kIOUSBNotSent1Err;
#ifdef USE_LOW_LATENCY_API
                    pUrbOsX->u.Isoc.aFrames[i].frTimeStamp.hi = 0;
                    pUrbOsX->u.Isoc.aFrames[i].frTimeStamp.lo = 0;
#endif
                }
                for (; i < RT_ELEMENTS(pUrbOsX->u.Isoc.aFrames); i++)
                {
                    pUrbOsX->u.Isoc.aFrames[i].frReqCount = 0;
                    pUrbOsX->u.Isoc.aFrames[i].frActCount = 0;
                    pUrbOsX->u.Isoc.aFrames[i].frStatus = kIOReturnError;
#ifdef USE_LOW_LATENCY_API
                    pUrbOsX->u.Isoc.aFrames[i].frTimeStamp.hi = 0;
                    pUrbOsX->u.Isoc.aFrames[i].frTimeStamp.lo = 0;
#endif
                }

#ifdef USE_LOW_LATENCY_API
                if (pUrb->enmDir == VUSBDIRECTION_OUT)
                    irc = (*pIf->ppIfI)->LowLatencyWriteIsochPipeAsync(pIf->ppIfI, u8PipeRef,
                                                                       pUrbOsX->u.Isoc.pBuf->pvBuf, FrameNo, pUrb->cIsocPkts, 0, pUrbOsX->u.Isoc.aFrames,
                                                                       usbProxyDarwinUrbAsyncComplete, pUrbOsX);
                else
                    irc = (*pIf->ppIfI)->LowLatencyReadIsochPipeAsync(pIf->ppIfI, u8PipeRef,
                                                                      pUrbOsX->u.Isoc.pBuf->pvBuf, FrameNo, pUrb->cIsocPkts, 0, pUrbOsX->u.Isoc.aFrames,
                                                                      usbProxyDarwinUrbAsyncComplete, pUrbOsX);
#else
                if (pUrb->enmDir == VUSBDIRECTION_OUT)
                    irc = (*pIf->ppIfI)->WriteIsochPipeAsync(pIf->ppIfI, u8PipeRef,
                                                             pUrb->abData, FrameNo, pUrb->cIsocPkts, &pUrbOsX->u.Isoc.aFrames[0],
                                                             usbProxyDarwinUrbAsyncComplete, pUrbOsX);
                else
                    irc = (*pIf->ppIfI)->ReadIsochPipeAsync(pIf->ppIfI, u8PipeRef,
                                                            pUrb->abData, FrameNo, pUrb->cIsocPkts, &pUrbOsX->u.Isoc.aFrames[0],
                                                            usbProxyDarwinUrbAsyncComplete, pUrbOsX);
#endif
                if (    irc != kIOReturnIsoTooOld
                    ||  j >= 5)
                {
                    Log(("%s: usbProxyDarwinUrbQueue: isoc: u64NextFrameNo=%RX64 FrameNo=%RX64 #Frames=%d j=%d (pipe=%d)\n",
                         pUrb->pszDesc, pPipe->u64NextFrameNo, FrameNo, pUrb->cIsocPkts, j, u8PipeRef));
                    if (irc == kIOReturnSuccess)
                        pPipe->u64NextFrameNo = FrameNo + pUrb->cIsocPkts;
                    break;
                }

                /* try again... */
                irc = (*pIf->ppIfI)->GetBusFrameNumber(pIf->ppIfI, &FrameNo, &FrameTime);
                if (FrameNo <= pPipe->u64NextFrameNo)
                    FrameNo = pPipe->u64NextFrameNo;
                FrameNo += j;
            }
            break;
        }

        default:
            AssertMsgFailed(("%s: enmType=%#x\n", pUrb->pszDesc, pUrb->enmType));
            break;
    }

    /*
     * Success?
     */
    if (RT_LIKELY(irc == kIOReturnSuccess))
        return true;
    switch (irc)
    {
        case kIOUSBPipeStalled:
        {
            usbProxyDarwinUrbAsyncComplete(pUrbOsX, kIOUSBPipeStalled, 0);
            Log(("%s: usbProxyDarwinUrbQueue: pProxyDev=%s EndPt=%d cbData=%d - failed irc=%#x! (stall)\n",
                 pUrb->pszDesc, pProxyDev->pUsbIns->pszName, pUrb->EndPt, pUrb->cbData, irc));
            return true;
        }
    }

    Log(("%s: usbProxyDarwinUrbQueue: pProxyDev=%s EndPt=%d cbData=%d - failed irc=%#x!\n",
         pUrb->pszDesc, pProxyDev->pUsbIns->pszName, pUrb->EndPt, pUrb->cbData, irc));
    return false;
}


/**
 * Reap URBs in-flight on a device.
 *
 * @returns Pointer to a completed URB.
 * @returns NULL if no URB was completed.
 * @param   pProxyDev   The device.
 * @param   cMillies    Number of milliseconds to wait. Use 0 to not wait at all.
 */
static PVUSBURB usbProxyDarwinUrbReap(PUSBPROXYDEV pProxyDev, RTMSINTERVAL cMillies)
{
    PVUSBURB pUrb = NULL;
    PUSBPROXYDEVOSX pDevOsX = (PUSBPROXYDEVOSX)pProxyDev->Backend.pv;

    /*
     * If we've got any in-flight URBs, excercise the runloop.
     */
    if (pDevOsX->pInFlightHead)
        CFRunLoopRunInMode(g_pRunLoopMode, 0.0, false);

    if (    !pDevOsX->pTaxingHead
        &&  cMillies
        &&  pDevOsX->pInFlightHead)
        CFRunLoopRunInMode(g_pRunLoopMode, cMillies / 1000.0, true);

    /*
     * Any URBs pending delivery?
     */
    while (     pDevOsX->pTaxingHead
           &&   !pUrb)
    {
        RTCritSectEnter(&pDevOsX->CritSect);

        PUSBPROXYURBOSX pUrbOsX = pDevOsX->pTaxingHead;
        if (pUrbOsX)
        {
            pUrb = pUrbOsX->pVUsbUrb;
            if (pUrb)
            {
                pUrb->Dev.pvPrivate = NULL;
                usbProxyDarwinUrbFree(pDevOsX, pUrbOsX);
            }
        }
        RTCritSectLeave(&pDevOsX->CritSect);
    }

    if (pUrb)
        LogFlow(("%s: usbProxyDarwinUrbReap: pProxyDev=%s returns %p\n", pUrb->pszDesc, pProxyDev->pUsbIns->pszName, pUrb));
    return pUrb;
}


/**
 * Cancels a URB.
 *
 * The URB requires reaping, so we don't change its state.
 *
 * @remark  There isn't any way to cancel a specific async request
 *          on darwin. The interface only supports the aborting of
 *          all URBs pending on an interface / pipe pair. Provided
 *          the card does the URB cancelling before submitting new
 *          requests, we should probably be fine...
 */
static void usbProxyDarwinUrbCancel(PVUSBURB pUrb)
{
    PUSBPROXYDEV pProxyDev = PDMINS_2_DATA(pUrb->pUsbIns, PUSBPROXYDEV);
    PUSBPROXYDEVOSX pDevOsX = (PUSBPROXYDEVOSX)pProxyDev->Backend.pv;
    //PUSBPROXYURBOSX pUrbOsX = (PUSBPROXYURBOSX)pUrb->Dev.pvProxyUrb;
    LogFlow(("%s: usbProxyDarwinUrbCancel: pProxyDev=%s EndPt=%d\n",
             pUrb->pszDesc, pProxyDev->pUsbIns->pszName, pUrb->EndPt));

    /*
     * Determine the interface / endpoint ref and invoke AbortPipe.
     */
    IOReturn irc = kIOReturnSuccess;
    if (!pUrb->EndPt)
        irc = (*pDevOsX->ppDevI)->USBDeviceAbortPipeZero(pDevOsX->ppDevI);
    else
    {
        uint8_t u8PipeRef;
        const uint8_t EndPt = pUrb->EndPt | (pUrb->enmDir == VUSBDIRECTION_IN ? 0x80 : 0);
        PUSBPROXYIFOSX pIf = usbProxyDarwinGetInterfaceForEndpoint(pDevOsX, EndPt, &u8PipeRef, NULL);
        if (pIf)
            irc = (*pIf->ppIfI)->AbortPipe(pIf->ppIfI, u8PipeRef);
        else /* this may happen if a device reset, set configuration or set interface has been performed. */
            Log(("usbProxyDarwinUrbCancel: pProxyDev=%s pUrb=%p EndPt=%d - cannot find the interface / pipe!\n",
                 pProxyDev->pUsbIns->pszName, pUrb, pUrb->EndPt));
    }
    if (irc != kIOReturnSuccess)
        Log(("usbProxyDarwinUrbCancel: pProxyDev=%s pUrb=%p EndPt=%d -> %#x!\n",
             pProxyDev->pUsbIns->pszName, pUrb, pUrb->EndPt, irc));
}


/**
 * The Darwin USB Proxy Backend.
 */
extern const USBPROXYBACK g_USBProxyDeviceHost =
{
    "host",
    usbProxyDarwinOpen,
    NULL,
    usbProxyDarwinClose,
    usbProxyDarwinReset,
    usbProxyDarwinSetConfig,
    usbProxyDarwinClaimInterface,
    usbProxyDarwinReleaseInterface,
    usbProxyDarwinSetInterface,
    usbProxyDarwinClearHaltedEp,
    usbProxyDarwinUrbQueue,
    usbProxyDarwinUrbCancel,
    usbProxyDarwinUrbReap,
    0
};

