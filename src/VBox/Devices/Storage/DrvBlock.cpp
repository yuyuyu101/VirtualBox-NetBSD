/* $Id: DrvBlock.cpp $ */
/** @file
 * VBox storage devices: Generic block driver
 */

/*
 * Copyright (C) 2006-2012 Oracle Corporation
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
#define LOG_GROUP LOG_GROUP_DRV_BLOCK
#include <VBox/vmm/pdmdrv.h>
#include <iprt/assert.h>
#include <iprt/string.h>
#include <iprt/uuid.h>

#include "VBoxDD.h"


/** @def VBOX_PERIODIC_FLUSH
 * Enable support for periodically flushing the VDI to disk. This may prove
 * useful for those nasty problems with the ultra-slow host filesystems.
 * If this is enabled, it can be configured via the CFGM key
 * "VBoxInternal/Devices/piix3ide/0/LUN#<x>/Config/FlushInterval". <x>
 * must be replaced with the correct LUN number of the disk that should
 * do the periodic flushes. The value of the key is the number of bytes
 * written between flushes. A value of 0 (the default) denotes no flushes. */
#define VBOX_PERIODIC_FLUSH

/** @def VBOX_IGNORE_FLUSH
 * Enable support for ignoring VDI flush requests. This can be useful for
 * filesystems that show bad guest IDE write performance (especially with
 * Windows guests). NOTE that this does not disable the flushes caused by
 * the periodic flush cache feature above.
 * If this feature is enabled, it can be configured via the CFGM key
 * "VBoxInternal/Devices/piix3ide/0/LUN#<x>/Config/IgnoreFlush". <x>
 * must be replaced with the correct LUN number of the disk that should
 * ignore flush requests. The value of the key is a boolean. The default
 * is to ignore flushes, i.e. true. */
#define VBOX_IGNORE_FLUSH


/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/
/**
 * Block driver instance data.
 *
 * @implements  PDMIBLOCK
 * @implements  PDMIBLOCKBIOS
 * @implements  PDMIMOUNT
 * @implements  PDMIMEDIAASYNCPORT
 * @implements  PDMIBLOCKASYNC
 */
typedef struct DRVBLOCK
{
    /** Pointer driver instance. */
    PPDMDRVINS              pDrvIns;
    /** Drive type. */
    PDMBLOCKTYPE            enmType;
    /** Locked indicator. */
    bool                    fLocked;
    /** Mountable indicator. */
    bool                    fMountable;
    /** Visible to the BIOS. */
    bool                    fBiosVisible;
#ifdef VBOX_PERIODIC_FLUSH
    /** HACK: Configuration value for number of bytes written after which to flush. */
    uint32_t                cbFlushInterval;
    /** HACK: Current count for the number of bytes written since the last flush. */
    uint32_t                cbDataWritten;
#endif /* VBOX_PERIODIC_FLUSH */
#ifdef VBOX_IGNORE_FLUSH
    /** HACK: Disable flushes for this drive. */
    bool                    fIgnoreFlush;
    /** Disable async flushes for this drive. */
    bool                    fIgnoreFlushAsync;
#endif /* VBOX_IGNORE_FLUSH */
    /** Pointer to the media driver below us.
     * This is NULL if the media is not mounted. */
    PPDMIMEDIA              pDrvMedia;
    /** Pointer to the block port interface above us. */
    PPDMIBLOCKPORT          pDrvBlockPort;
    /** Pointer to the mount notify interface above us. */
    PPDMIMOUNTNOTIFY        pDrvMountNotify;
    /** Our block interface. */
    PDMIBLOCK               IBlock;
    /** Our block interface. */
    PDMIBLOCKBIOS           IBlockBios;
    /** Our mountable interface. */
    PDMIMOUNT               IMount;
    /** Our media port interface. */
    PDMIMEDIAPORT           IMediaPort;

    /** Pointer to the async media driver below us.
     * This is NULL if the media is not mounted. */
    PPDMIMEDIAASYNC         pDrvMediaAsync;
    /** Our media async port. */
    PDMIMEDIAASYNCPORT      IMediaAsyncPort;
    /** Pointer to the async block port interface above us. */
    PPDMIBLOCKASYNCPORT     pDrvBlockAsyncPort;
    /** Our async block interface. */
    PDMIBLOCKASYNC          IBlockAsync;

    /** Uuid of the drive. */
    RTUUID                  Uuid;

    /** BIOS PCHS Geometry. */
    PDMMEDIAGEOMETRY        PCHSGeometry;
    /** BIOS LCHS Geometry. */
    PDMMEDIAGEOMETRY        LCHSGeometry;
} DRVBLOCK, *PDRVBLOCK;


/* -=-=-=-=- IBlock -=-=-=-=- */

/** Makes a PDRVBLOCK out of a PPDMIBLOCK. */
#define PDMIBLOCK_2_DRVBLOCK(pInterface)        ( (PDRVBLOCK)((uintptr_t)pInterface - RT_OFFSETOF(DRVBLOCK, IBlock)) )

/** @copydoc PDMIBLOCK::pfnRead */
static DECLCALLBACK(int) drvblockRead(PPDMIBLOCK pInterface, uint64_t off, void *pvBuf, size_t cbRead)
{
    PDRVBLOCK pThis = PDMIBLOCK_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMedia)
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }

    int rc = pThis->pDrvMedia->pfnRead(pThis->pDrvMedia, off, pvBuf, cbRead);
    return rc;
}


/** @copydoc PDMIBLOCK::pfnWrite */
static DECLCALLBACK(int) drvblockWrite(PPDMIBLOCK pInterface, uint64_t off, const void *pvBuf, size_t cbWrite)
{
    PDRVBLOCK pThis = PDMIBLOCK_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMedia)
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }

    /* Set an FTM checkpoint as this operation changes the state permanently. */
    PDMDrvHlpFTSetCheckpoint(pThis->pDrvIns, FTMCHECKPOINTTYPE_STORAGE);

    int rc = pThis->pDrvMedia->pfnWrite(pThis->pDrvMedia, off, pvBuf, cbWrite);
#ifdef VBOX_PERIODIC_FLUSH
    if (pThis->cbFlushInterval)
    {
        pThis->cbDataWritten += (uint32_t)cbWrite;
        if (pThis->cbDataWritten > pThis->cbFlushInterval)
        {
            pThis->cbDataWritten = 0;
            pThis->pDrvMedia->pfnFlush(pThis->pDrvMedia);
        }
    }
#endif /* VBOX_PERIODIC_FLUSH */

    return rc;
}


/** @copydoc PDMIBLOCK::pfnFlush */
static DECLCALLBACK(int) drvblockFlush(PPDMIBLOCK pInterface)
{
    PDRVBLOCK pThis = PDMIBLOCK_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMedia)
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }

#ifdef VBOX_IGNORE_FLUSH
    if (pThis->fIgnoreFlush)
        return VINF_SUCCESS;
#endif /* VBOX_IGNORE_FLUSH */

    int rc = pThis->pDrvMedia->pfnFlush(pThis->pDrvMedia);
    if (rc == VERR_NOT_IMPLEMENTED)
        rc = VINF_SUCCESS;
    return rc;
}


/** @copydoc PDMIBLOCK::pfnMerge */
static DECLCALLBACK(int) drvblockMerge(PPDMIBLOCK pInterface,
                                       PFNSIMPLEPROGRESS pfnProgress,
                                       void *pvUser)
{
    PDRVBLOCK pThis = PDMIBLOCK_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMedia)
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }

    if (!pThis->pDrvMedia->pfnMerge)
        return VERR_NOT_SUPPORTED;

    int rc = pThis->pDrvMedia->pfnMerge(pThis->pDrvMedia, pfnProgress, pvUser);
    return rc;
}


/** @copydoc PDMIBLOCK::pfnIsReadOnly */
static DECLCALLBACK(bool) drvblockIsReadOnly(PPDMIBLOCK pInterface)
{
    PDRVBLOCK pThis = PDMIBLOCK_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMedia)
        return false;

    bool fRc = pThis->pDrvMedia->pfnIsReadOnly(pThis->pDrvMedia);
    return fRc;
}


/** @copydoc PDMIBLOCK::pfnGetSize */
static DECLCALLBACK(uint64_t) drvblockGetSize(PPDMIBLOCK pInterface)
{
    PDRVBLOCK pThis = PDMIBLOCK_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMedia)
        return 0;

    uint64_t cb = pThis->pDrvMedia->pfnGetSize(pThis->pDrvMedia);
    LogFlow(("drvblockGetSize: returns %llu\n", cb));
    return cb;
}


/** @copydoc PDMIBLOCK::pfnGetType */
static DECLCALLBACK(PDMBLOCKTYPE) drvblockGetType(PPDMIBLOCK pInterface)
{
    PDRVBLOCK pThis = PDMIBLOCK_2_DRVBLOCK(pInterface);
    LogFlow(("drvblockGetType: returns %d\n", pThis->enmType));
    return pThis->enmType;
}


/** @copydoc PDMIBLOCK::pfnGetUuid */
static DECLCALLBACK(int) drvblockGetUuid(PPDMIBLOCK pInterface, PRTUUID pUuid)
{
    PDRVBLOCK pThis = PDMIBLOCK_2_DRVBLOCK(pInterface);

    /*
     * Copy the uuid.
     */
    *pUuid = pThis->Uuid;
    return VINF_SUCCESS;
}

static DECLCALLBACK(int) drvblockDiscard(PPDMIBLOCK pInterface, PCRTRANGE paRanges, unsigned cRanges)
{
    PDRVBLOCK pThis = PDMIBLOCK_2_DRVBLOCK(pInterface);

    return pThis->pDrvMedia->pfnDiscard(pThis->pDrvMedia, paRanges, cRanges);
}

/* -=-=-=-=- IBlockAsync -=-=-=-=- */

/** Makes a PDRVBLOCK out of a PPDMIBLOCKASYNC. */
#define PDMIBLOCKASYNC_2_DRVBLOCK(pInterface)        ( (PDRVBLOCK)((uintptr_t)pInterface - RT_OFFSETOF(DRVBLOCK, IBlockAsync)) )

/** @copydoc PDMIBLOCKASYNC::pfnStartRead */
static DECLCALLBACK(int) drvblockAsyncReadStart(PPDMIBLOCKASYNC pInterface, uint64_t off, PCRTSGSEG pSeg, unsigned cSeg, size_t cbRead, void *pvUser)
{
    PDRVBLOCK pThis = PDMIBLOCKASYNC_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMediaAsync)
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }

    int rc = pThis->pDrvMediaAsync->pfnStartRead(pThis->pDrvMediaAsync, off, pSeg, cSeg, cbRead, pvUser);
    return rc;
}


/** @copydoc PDMIBLOCKASYNC::pfnStartWrite */
static DECLCALLBACK(int) drvblockAsyncWriteStart(PPDMIBLOCKASYNC pInterface, uint64_t off, PCRTSGSEG pSeg, unsigned cSeg, size_t cbWrite, void *pvUser)
{
    PDRVBLOCK pThis = PDMIBLOCKASYNC_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMediaAsync)
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }

    int rc = pThis->pDrvMediaAsync->pfnStartWrite(pThis->pDrvMediaAsync, off, pSeg, cSeg, cbWrite, pvUser);

    return rc;
}


/** @copydoc PDMIBLOCKASYNC::pfnStartFlush */
static DECLCALLBACK(int) drvblockAsyncFlushStart(PPDMIBLOCKASYNC pInterface, void *pvUser)
{
    PDRVBLOCK pThis = PDMIBLOCKASYNC_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMediaAsync)
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }

#ifdef VBOX_IGNORE_FLUSH
    if (pThis->fIgnoreFlushAsync)
        return VINF_VD_ASYNC_IO_FINISHED;
#endif /* VBOX_IGNORE_FLUSH */

    int rc = pThis->pDrvMediaAsync->pfnStartFlush(pThis->pDrvMediaAsync, pvUser);

    return rc;
}


/** @copydoc PDMIBLOCKASYNC::pfnStartDiscard */
static DECLCALLBACK(int) drvblockStartDiscard(PPDMIBLOCKASYNC pInterface, PCRTRANGE paRanges, unsigned cRanges, void *pvUser)
{
    PDRVBLOCK pThis = PDMIBLOCKASYNC_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMediaAsync)
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }

    return pThis->pDrvMediaAsync->pfnStartDiscard(pThis->pDrvMediaAsync, paRanges, cRanges, pvUser);
}

/* -=-=-=-=- IMediaAsyncPort -=-=-=-=- */

/** Makes a PDRVBLOCKASYNC out of a PPDMIMEDIAASYNCPORT. */
#define PDMIMEDIAASYNCPORT_2_DRVBLOCK(pInterface)    ( (PDRVBLOCK((uintptr_t)pInterface - RT_OFFSETOF(DRVBLOCK, IMediaAsyncPort))) )

static DECLCALLBACK(int) drvblockAsyncTransferCompleteNotify(PPDMIMEDIAASYNCPORT pInterface, void *pvUser, int rcReq)
{
    PDRVBLOCK pThis = PDMIMEDIAASYNCPORT_2_DRVBLOCK(pInterface);

    return pThis->pDrvBlockAsyncPort->pfnTransferCompleteNotify(pThis->pDrvBlockAsyncPort, pvUser, rcReq);
}

/* -=-=-=-=- IBlockBios -=-=-=-=- */

/** Makes a PDRVBLOCK out of a PPDMIBLOCKBIOS. */
#define PDMIBLOCKBIOS_2_DRVBLOCK(pInterface)    ( (PDRVBLOCK((uintptr_t)pInterface - RT_OFFSETOF(DRVBLOCK, IBlockBios))) )


/** @copydoc PDMIBLOCKBIOS::pfnGetPCHSGeometry */
static DECLCALLBACK(int) drvblockGetPCHSGeometry(PPDMIBLOCKBIOS pInterface, PPDMMEDIAGEOMETRY pPCHSGeometry)
{
    PDRVBLOCK pThis = PDMIBLOCKBIOS_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMedia)
        return VERR_PDM_MEDIA_NOT_MOUNTED;

    /*
     * Use configured/cached values if present.
     */
    if (    pThis->PCHSGeometry.cCylinders > 0
        &&  pThis->PCHSGeometry.cHeads > 0
        &&  pThis->PCHSGeometry.cSectors > 0)
    {
        *pPCHSGeometry = pThis->PCHSGeometry;
        LogFlow(("%s: returns VINF_SUCCESS {%d,%d,%d}\n", __FUNCTION__, pThis->PCHSGeometry.cCylinders, pThis->PCHSGeometry.cHeads, pThis->PCHSGeometry.cSectors));
        return VINF_SUCCESS;
    }

    /*
     * Call media.
     */
    int rc = pThis->pDrvMedia->pfnBiosGetPCHSGeometry(pThis->pDrvMedia, &pThis->PCHSGeometry);

    if (RT_SUCCESS(rc))
    {
        *pPCHSGeometry = pThis->PCHSGeometry;
        LogFlow(("%s: returns %Rrc {%d,%d,%d}\n", __FUNCTION__, rc, pThis->PCHSGeometry.cCylinders, pThis->PCHSGeometry.cHeads, pThis->PCHSGeometry.cSectors));
    }
    else if (rc == VERR_NOT_IMPLEMENTED)
    {
        rc = VERR_PDM_GEOMETRY_NOT_SET;
        LogFlow(("%s: returns %Rrc\n", __FUNCTION__, rc));
    }
    return rc;
}


/** @copydoc PDMIBLOCKBIOS::pfnSetPCHSGeometry */
static DECLCALLBACK(int) drvblockSetPCHSGeometry(PPDMIBLOCKBIOS pInterface, PCPDMMEDIAGEOMETRY pPCHSGeometry)
{
    LogFlow(("%s: cCylinders=%d cHeads=%d cSectors=%d\n", __FUNCTION__, pPCHSGeometry->cCylinders, pPCHSGeometry->cHeads, pPCHSGeometry->cSectors));
    PDRVBLOCK pThis = PDMIBLOCKBIOS_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMedia)
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }

    /*
     * Call media. Ignore the not implemented return code.
     */
    int rc = pThis->pDrvMedia->pfnBiosSetPCHSGeometry(pThis->pDrvMedia, pPCHSGeometry);

    if (    RT_SUCCESS(rc)
        ||  rc == VERR_NOT_IMPLEMENTED)
    {
        pThis->PCHSGeometry = *pPCHSGeometry;
        rc = VINF_SUCCESS;
    }
    return rc;
}


/** @copydoc PDMIBLOCKBIOS::pfnGetLCHSGeometry */
static DECLCALLBACK(int) drvblockGetLCHSGeometry(PPDMIBLOCKBIOS pInterface, PPDMMEDIAGEOMETRY pLCHSGeometry)
{
    PDRVBLOCK pThis = PDMIBLOCKBIOS_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMedia)
        return VERR_PDM_MEDIA_NOT_MOUNTED;

    /*
     * Use configured/cached values if present.
     */
    if (    pThis->LCHSGeometry.cCylinders > 0
        &&  pThis->LCHSGeometry.cHeads > 0
        &&  pThis->LCHSGeometry.cSectors > 0)
    {
        *pLCHSGeometry = pThis->LCHSGeometry;
        LogFlow(("%s: returns VINF_SUCCESS {%d,%d,%d}\n", __FUNCTION__, pThis->LCHSGeometry.cCylinders, pThis->LCHSGeometry.cHeads, pThis->LCHSGeometry.cSectors));
        return VINF_SUCCESS;
    }

    /*
     * Call media.
     */
    int rc = pThis->pDrvMedia->pfnBiosGetLCHSGeometry(pThis->pDrvMedia, &pThis->LCHSGeometry);

    if (RT_SUCCESS(rc))
    {
        *pLCHSGeometry = pThis->LCHSGeometry;
        LogFlow(("%s: returns %Rrc {%d,%d,%d}\n", __FUNCTION__, rc, pThis->LCHSGeometry.cCylinders, pThis->LCHSGeometry.cHeads, pThis->LCHSGeometry.cSectors));
    }
    else if (rc == VERR_NOT_IMPLEMENTED)
    {
        rc = VERR_PDM_GEOMETRY_NOT_SET;
        LogFlow(("%s: returns %Rrc\n", __FUNCTION__, rc));
    }
    return rc;
}


/** @copydoc PDMIBLOCKBIOS::pfnSetLCHSGeometry */
static DECLCALLBACK(int) drvblockSetLCHSGeometry(PPDMIBLOCKBIOS pInterface, PCPDMMEDIAGEOMETRY pLCHSGeometry)
{
    LogFlow(("%s: cCylinders=%d cHeads=%d cSectors=%d\n", __FUNCTION__, pLCHSGeometry->cCylinders, pLCHSGeometry->cHeads, pLCHSGeometry->cSectors));
    PDRVBLOCK pThis = PDMIBLOCKBIOS_2_DRVBLOCK(pInterface);

    /*
     * Check the state.
     */
    if (!pThis->pDrvMedia)
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }

    /*
     * Call media. Ignore the not implemented return code.
     */
    int rc = pThis->pDrvMedia->pfnBiosSetLCHSGeometry(pThis->pDrvMedia, pLCHSGeometry);

    if (    RT_SUCCESS(rc)
        ||  rc == VERR_NOT_IMPLEMENTED)
    {
        pThis->LCHSGeometry = *pLCHSGeometry;
        rc = VINF_SUCCESS;
    }
    return rc;
}


/** @copydoc PDMIBLOCKBIOS::pfnIsVisible */
static DECLCALLBACK(bool) drvblockIsVisible(PPDMIBLOCKBIOS pInterface)
{
    PDRVBLOCK pThis = PDMIBLOCKBIOS_2_DRVBLOCK(pInterface);
    LogFlow(("drvblockIsVisible: returns %d\n", pThis->fBiosVisible));
    return pThis->fBiosVisible;
}


/** @copydoc PDMIBLOCKBIOS::pfnGetType */
static DECLCALLBACK(PDMBLOCKTYPE) drvblockBiosGetType(PPDMIBLOCKBIOS pInterface)
{
    PDRVBLOCK pThis = PDMIBLOCKBIOS_2_DRVBLOCK(pInterface);
    LogFlow(("drvblockBiosGetType: returns %d\n", pThis->enmType));
    return pThis->enmType;
}



/* -=-=-=-=- IMount -=-=-=-=- */

/** Makes a PDRVBLOCK out of a PPDMIMOUNT. */
#define PDMIMOUNT_2_DRVBLOCK(pInterface)        ( (PDRVBLOCK)((uintptr_t)pInterface - RT_OFFSETOF(DRVBLOCK, IMount)) )


/** @copydoc PDMIMOUNT::pfnMount */
static DECLCALLBACK(int) drvblockMount(PPDMIMOUNT pInterface, const char *pszFilename, const char *pszCoreDriver)
{
    LogFlow(("drvblockMount: pszFilename=%p:{%s} pszCoreDriver=%p:{%s}\n", pszFilename, pszFilename, pszCoreDriver, pszCoreDriver));
    PDRVBLOCK pThis = PDMIMOUNT_2_DRVBLOCK(pInterface);

    /*
     * Validate state.
     */
    if (pThis->pDrvMedia)
    {
        AssertMsgFailed(("Already mounted\n"));
        return VERR_PDM_MEDIA_MOUNTED;
    }

    /*
     * Prepare configuration.
     */
    if (pszFilename)
    {
        int rc = PDMDrvHlpMountPrepare(pThis->pDrvIns, pszFilename, pszCoreDriver);
        if (RT_FAILURE(rc))
        {
            Log(("drvblockMount: Prepare failed for \"%s\" rc=%Rrc\n", pszFilename, rc));
            return rc;
        }
    }

    /*
     * Attach the media driver and query it's interface.
     */
    uint32_t fTachFlags = 0; /** @todo figure attachment flags for mount. */
    PPDMIBASE pBase;
    int rc = PDMDrvHlpAttach(pThis->pDrvIns, fTachFlags, &pBase);
    if (RT_FAILURE(rc))
    {
        Log(("drvblockMount: Attach failed rc=%Rrc\n", rc));
        return rc;
    }

    pThis->pDrvMedia = PDMIBASE_QUERY_INTERFACE(pBase, PDMIMEDIA);
    if (pThis->pDrvMedia)
    {
        /** @todo r=klaus missing async handling, this is just a band aid to
         * avoid using stale information */
        pThis->pDrvMediaAsync = NULL;

        /*
         * Initialize state.
         */
        pThis->fLocked = false;
        pThis->PCHSGeometry.cCylinders  = 0;
        pThis->PCHSGeometry.cHeads      = 0;
        pThis->PCHSGeometry.cSectors    = 0;
        pThis->LCHSGeometry.cCylinders  = 0;
        pThis->LCHSGeometry.cHeads      = 0;
        pThis->LCHSGeometry.cSectors    = 0;
#ifdef VBOX_PERIODIC_FLUSH
        pThis->cbDataWritten = 0;
#endif /* VBOX_PERIODIC_FLUSH */

        /*
         * Notify driver/device above us.
         */
        if (pThis->pDrvMountNotify)
            pThis->pDrvMountNotify->pfnMountNotify(pThis->pDrvMountNotify);
        Log(("drvblockMount: Success\n"));
        return VINF_SUCCESS;
    }
    else
        rc = VERR_PDM_MISSING_INTERFACE_BELOW;

    /*
     * Failed, detatch the media driver.
     */
    AssertMsgFailed(("No media interface!\n"));
    int rc2 = PDMDrvHlpDetach(pThis->pDrvIns, fTachFlags);
    AssertRC(rc2);
    pThis->pDrvMedia = NULL;
    return rc;
}


/** @copydoc PDMIMOUNT::pfnUnmount */
static DECLCALLBACK(int) drvblockUnmount(PPDMIMOUNT pInterface, bool fForce, bool fEject)
{
    PDRVBLOCK pThis = PDMIMOUNT_2_DRVBLOCK(pInterface);

    /*
     * Validate state.
     */
    if (!pThis->pDrvMedia)
    {
        Log(("drvblockUmount: Not mounted\n"));
        return VERR_PDM_MEDIA_NOT_MOUNTED;
    }
    if (pThis->fLocked && !fForce)
    {
        Log(("drvblockUmount: Locked\n"));
        return VERR_PDM_MEDIA_LOCKED;
    }

    /* Media is no longer locked even if it was previously. */
    pThis->fLocked = false;

    /*
     * Detach the media driver and query it's interface.
     */
    int rc = PDMDrvHlpDetach(pThis->pDrvIns, 0 /*fFlags*/);
    if (RT_FAILURE(rc))
    {
        Log(("drvblockUnmount: Detach failed rc=%Rrc\n", rc));
        return rc;
    }
    Assert(!pThis->pDrvMedia);

    /*
     * Notify driver/device above us.
     */
    if (pThis->pDrvMountNotify)
        pThis->pDrvMountNotify->pfnUnmountNotify(pThis->pDrvMountNotify);
    Log(("drvblockUnmount: success\n"));
    return VINF_SUCCESS;
}


/** @copydoc PDMIMOUNT::pfnIsMounted */
static DECLCALLBACK(bool) drvblockIsMounted(PPDMIMOUNT pInterface)
{
    PDRVBLOCK pThis = PDMIMOUNT_2_DRVBLOCK(pInterface);
    return pThis->pDrvMedia != NULL;
}

/** @copydoc PDMIMOUNT::pfnLock */
static DECLCALLBACK(int) drvblockLock(PPDMIMOUNT pInterface)
{
    PDRVBLOCK pThis = PDMIMOUNT_2_DRVBLOCK(pInterface);
    Log(("drvblockLock: %d -> %d\n", pThis->fLocked, true));
    pThis->fLocked = true;
    return VINF_SUCCESS;
}

/** @copydoc PDMIMOUNT::pfnUnlock */
static DECLCALLBACK(int) drvblockUnlock(PPDMIMOUNT pInterface)
{
    PDRVBLOCK pThis = PDMIMOUNT_2_DRVBLOCK(pInterface);
    Log(("drvblockUnlock: %d -> %d\n", pThis->fLocked, false));
    pThis->fLocked = false;
    return VINF_SUCCESS;
}

/** @copydoc PDMIMOUNT::pfnIsLocked */
static DECLCALLBACK(bool) drvblockIsLocked(PPDMIMOUNT pInterface)
{
    PDRVBLOCK pThis = PDMIMOUNT_2_DRVBLOCK(pInterface);
    return pThis->fLocked;
}



/* -=-=-=-=- IMediaPort -=-=-=-=- */

/** Makes a PDRVBLOCK out of a PPDMIMEDIAPORT. */
#define PDMIMEDIAPORT_2_DRVBLOCK(pInterface)    ( (PDRVBLOCK((uintptr_t)pInterface - RT_OFFSETOF(DRVBLOCK, IMediaPort))) )

/**
 * @interface_method_impl{PDMIMEDIAPORT,pfnQueryDeviceLocation}
 */
static DECLCALLBACK(int) drvblockQueryDeviceLocation(PPDMIMEDIAPORT pInterface, const char **ppcszController,
                                                     uint32_t *piInstance, uint32_t *piLUN)
{
    PDRVBLOCK pThis = PDMIMEDIAPORT_2_DRVBLOCK(pInterface);

    return pThis->pDrvBlockPort->pfnQueryDeviceLocation(pThis->pDrvBlockPort, ppcszController,
                                                        piInstance, piLUN);
}

/* -=-=-=-=- IBase -=-=-=-=- */

/**
 * @interface_method_impl{PDMIBASE,pfnQueryInterface}
 */
static DECLCALLBACK(void *)  drvblockQueryInterface(PPDMIBASE pInterface, const char *pszIID)
{
    PPDMDRVINS  pDrvIns = PDMIBASE_2_PDMDRV(pInterface);
    PDRVBLOCK   pThis = PDMINS_2_DATA(pDrvIns, PDRVBLOCK);

    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBASE, &pDrvIns->IBase);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBLOCK, &pThis->IBlock);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBLOCKBIOS, pThis->fBiosVisible ? &pThis->IBlockBios : NULL);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIMOUNT, pThis->fMountable ? &pThis->IMount : NULL);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBLOCKASYNC, pThis->pDrvMediaAsync ? &pThis->IBlockAsync : NULL);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIMEDIAASYNCPORT, pThis->pDrvBlockAsyncPort ? &pThis->IMediaAsyncPort : NULL);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIMEDIAPORT, &pThis->IMediaPort);
    return NULL;
}


/* -=-=-=-=- driver interface -=-=-=-=- */

/** @copydoc FNPDMDRVDETACH. */
static DECLCALLBACK(void)  drvblockDetach(PPDMDRVINS pDrvIns, uint32_t fFlags)
{
    PDRVBLOCK pThis = PDMINS_2_DATA(pDrvIns, PDRVBLOCK);
    pThis->pDrvMedia = NULL;
    pThis->pDrvMediaAsync = NULL;
    NOREF(fFlags);
}

/**
 * Reset notification.
 *
 * @returns VBox status.
 * @param   pDevIns     The driver instance data.
 */
static DECLCALLBACK(void)  drvblockReset(PPDMDRVINS pDrvIns)
{
    PDRVBLOCK pThis = PDMINS_2_DATA(pDrvIns, PDRVBLOCK);

    pThis->fLocked = false;
}

/**
 * Construct a block driver instance.
 *
 * @copydoc FNPDMDRVCONSTRUCT
 */
static DECLCALLBACK(int) drvblockConstruct(PPDMDRVINS pDrvIns, PCFGMNODE pCfg, uint32_t fFlags)
{
    PDRVBLOCK pThis = PDMINS_2_DATA(pDrvIns, PDRVBLOCK);
    LogFlow(("drvblockConstruct: iInstance=%d\n", pDrvIns->iInstance));
    PDMDRV_CHECK_VERSIONS_RETURN(pDrvIns);

    /*
     * Validate configuration.
     */
#if defined(VBOX_PERIODIC_FLUSH) || defined(VBOX_IGNORE_FLUSH)
    if (!CFGMR3AreValuesValid(pCfg, "Type\0Locked\0BIOSVisible\0AttachFailError\0Cylinders\0Heads\0Sectors\0Mountable\0FlushInterval\0IgnoreFlush\0IgnoreFlushAsync\0"))
#else /* !(VBOX_PERIODIC_FLUSH || VBOX_IGNORE_FLUSH) */
    if (!CFGMR3AreValuesValid(pCfg, "Type\0Locked\0BIOSVisible\0AttachFailError\0Cylinders\0Heads\0Sectors\0Mountable\0"))
#endif /* !(VBOX_PERIODIC_FLUSH || VBOX_IGNORE_FLUSH) */
        return VERR_PDM_DRVINS_UNKNOWN_CFG_VALUES;

    /*
     * Initialize most of the data members.
     */
    pThis->pDrvIns                          = pDrvIns;

    /* IBase. */
    pDrvIns->IBase.pfnQueryInterface        = drvblockQueryInterface;

    /* IBlock. */
    pThis->IBlock.pfnRead                   = drvblockRead;
    pThis->IBlock.pfnWrite                  = drvblockWrite;
    pThis->IBlock.pfnFlush                  = drvblockFlush;
    pThis->IBlock.pfnMerge                  = drvblockMerge;
    pThis->IBlock.pfnIsReadOnly             = drvblockIsReadOnly;
    pThis->IBlock.pfnGetSize                = drvblockGetSize;
    pThis->IBlock.pfnGetType                = drvblockGetType;
    pThis->IBlock.pfnGetUuid                = drvblockGetUuid;

    /* IBlockBios. */
    pThis->IBlockBios.pfnGetPCHSGeometry    = drvblockGetPCHSGeometry;
    pThis->IBlockBios.pfnSetPCHSGeometry    = drvblockSetPCHSGeometry;
    pThis->IBlockBios.pfnGetLCHSGeometry    = drvblockGetLCHSGeometry;
    pThis->IBlockBios.pfnSetLCHSGeometry    = drvblockSetLCHSGeometry;
    pThis->IBlockBios.pfnIsVisible          = drvblockIsVisible;
    pThis->IBlockBios.pfnGetType            = drvblockBiosGetType;

    /* IMount. */
    pThis->IMount.pfnMount                  = drvblockMount;
    pThis->IMount.pfnUnmount                = drvblockUnmount;
    pThis->IMount.pfnIsMounted              = drvblockIsMounted;
    pThis->IMount.pfnLock                   = drvblockLock;
    pThis->IMount.pfnUnlock                 = drvblockUnlock;
    pThis->IMount.pfnIsLocked               = drvblockIsLocked;

    /* IBlockAsync. */
    pThis->IBlockAsync.pfnStartRead         = drvblockAsyncReadStart;
    pThis->IBlockAsync.pfnStartWrite        = drvblockAsyncWriteStart;
    pThis->IBlockAsync.pfnStartFlush        = drvblockAsyncFlushStart;

    /* IMediaAsyncPort. */
    pThis->IMediaAsyncPort.pfnTransferCompleteNotify  = drvblockAsyncTransferCompleteNotify;

    /* IMediaPort */
    pThis->IMediaPort.pfnQueryDeviceLocation = drvblockQueryDeviceLocation;

    /*
     * Get the IBlockPort & IMountNotify interfaces of the above driver/device.
     */
    pThis->pDrvBlockPort = PDMIBASE_QUERY_INTERFACE(pDrvIns->pUpBase, PDMIBLOCKPORT);
    if (!pThis->pDrvBlockPort)
        return PDMDRV_SET_ERROR(pDrvIns, VERR_PDM_MISSING_INTERFACE_ABOVE,
                                N_("No block port interface above"));

    /* Try to get the optional async block port interface above. */
    pThis->pDrvBlockAsyncPort = PDMIBASE_QUERY_INTERFACE(pDrvIns->pUpBase, PDMIBLOCKASYNCPORT);
    pThis->pDrvMountNotify    = PDMIBASE_QUERY_INTERFACE(pDrvIns->pUpBase, PDMIMOUNTNOTIFY);

    /*
     * Query configuration.
     */
    /* type */
    char *psz;
    int rc = CFGMR3QueryStringAlloc(pCfg, "Type", &psz);
    if (RT_FAILURE(rc))
        return PDMDRV_SET_ERROR(pDrvIns, VERR_PDM_BLOCK_NO_TYPE, N_("Failed to obtain the type"));
    if (!strcmp(psz, "HardDisk"))
        pThis->enmType = PDMBLOCKTYPE_HARD_DISK;
    else if (!strcmp(psz, "DVD"))
        pThis->enmType = PDMBLOCKTYPE_DVD;
    else if (!strcmp(psz, "CDROM"))
        pThis->enmType = PDMBLOCKTYPE_CDROM;
    else if (!strcmp(psz, "Floppy 2.88"))
        pThis->enmType = PDMBLOCKTYPE_FLOPPY_2_88;
    else if (!strcmp(psz, "Floppy 1.44"))
        pThis->enmType = PDMBLOCKTYPE_FLOPPY_1_44;
    else if (!strcmp(psz, "Floppy 1.20"))
        pThis->enmType = PDMBLOCKTYPE_FLOPPY_1_20;
    else if (!strcmp(psz, "Floppy 720"))
        pThis->enmType = PDMBLOCKTYPE_FLOPPY_720;
    else if (!strcmp(psz, "Floppy 360"))
        pThis->enmType = PDMBLOCKTYPE_FLOPPY_360;
    else
    {
        PDMDrvHlpVMSetError(pDrvIns, VERR_PDM_BLOCK_UNKNOWN_TYPE, RT_SRC_POS,
                            N_("Unknown type \"%s\""), psz);
        MMR3HeapFree(psz);
        return VERR_PDM_BLOCK_UNKNOWN_TYPE;
    }
    Log2(("drvblockConstruct: enmType=%d\n", pThis->enmType));
    MMR3HeapFree(psz); psz = NULL;

    /* Mountable */
    rc = CFGMR3QueryBoolDef(pCfg, "Mountable", &pThis->fMountable, false);
    if (RT_FAILURE(rc))
        return PDMDRV_SET_ERROR(pDrvIns, rc, N_("Failed to query \"Mountable\" from the config"));

    /* Locked */
    rc = CFGMR3QueryBoolDef(pCfg, "Locked", &pThis->fLocked, false);
    if (RT_FAILURE(rc))
        return PDMDRV_SET_ERROR(pDrvIns, rc, N_("Failed to query \"Locked\" from the config"));

    /* BIOS visible */
    rc = CFGMR3QueryBoolDef(pCfg, "BIOSVisible", &pThis->fBiosVisible, true);
    if (RT_FAILURE(rc))
        return PDMDRV_SET_ERROR(pDrvIns, rc, N_("Failed to query \"BIOSVisible\" from the config"));

    /** @todo AttachFailError is currently completely ignored. */

    /* Cylinders */
    rc = CFGMR3QueryU32Def(pCfg, "Cylinders", &pThis->LCHSGeometry.cCylinders, 0);
    if (RT_FAILURE(rc))
        return PDMDRV_SET_ERROR(pDrvIns, rc, N_("Failed to query \"Cylinders\" from the config"));

    /* Heads */
    rc = CFGMR3QueryU32Def(pCfg, "Heads", &pThis->LCHSGeometry.cHeads, 0);
    if (RT_FAILURE(rc))
        return PDMDRV_SET_ERROR(pDrvIns, rc, N_("Failed to query \"Heads\" from the config"));

    /* Sectors */
    rc = CFGMR3QueryU32Def(pCfg, "Sectors", &pThis->LCHSGeometry.cSectors, 0);
    if (RT_FAILURE(rc))
        return PDMDRV_SET_ERROR(pDrvIns, rc, N_("Failed to query \"Sectors\" from the config"));

    /* Uuid */
    rc = CFGMR3QueryStringAlloc(pCfg, "Uuid", &psz);
    if (rc == VERR_CFGM_VALUE_NOT_FOUND)
        RTUuidClear(&pThis->Uuid);
    else if (RT_SUCCESS(rc))
    {
        rc = RTUuidFromStr(&pThis->Uuid, psz);
        if (RT_FAILURE(rc))
        {
            PDMDrvHlpVMSetError(pDrvIns, rc, RT_SRC_POS, "%s",
                                N_("Uuid from string failed on \"%s\""), psz);
            MMR3HeapFree(psz);
            return rc;
        }
        MMR3HeapFree(psz); psz = NULL;
    }
    else
        return PDMDRV_SET_ERROR(pDrvIns, rc, N_("Failed to query \"Uuid\" from the config"));

#ifdef VBOX_PERIODIC_FLUSH
    rc = CFGMR3QueryU32Def(pCfg, "FlushInterval", &pThis->cbFlushInterval, 0);
    if (RT_FAILURE(rc))
        return PDMDRV_SET_ERROR(pDrvIns, rc, N_("Failed to query \"FlushInterval\" from the config"));
#endif /* VBOX_PERIODIC_FLUSH */

#ifdef VBOX_IGNORE_FLUSH
    rc = CFGMR3QueryBoolDef(pCfg, "IgnoreFlush", &pThis->fIgnoreFlush, true);
    if (RT_FAILURE(rc))
        return PDMDRV_SET_ERROR(pDrvIns, rc, N_("Failed to query \"IgnoreFlush\" from the config"));

    if (pThis->fIgnoreFlush)
        LogRel(("DrvBlock: Flushes will be ignored\n"));
    else
        LogRel(("DrvBlock: Flushes will be passed to the disk\n"));

    rc = CFGMR3QueryBoolDef(pCfg, "IgnoreFlushAsync", &pThis->fIgnoreFlushAsync, false);
    if (RT_FAILURE(rc))
        return PDMDRV_SET_ERROR(pDrvIns, rc, N_("Failed to query \"IgnoreFlushAsync\" from the config"));

    if (pThis->fIgnoreFlushAsync)
        LogRel(("DrvBlock: Async flushes will be ignored\n"));
    else
        LogRel(("DrvBlock: Async flushes will be passed to the disk\n"));
#endif /* VBOX_IGNORE_FLUSH */

    /*
     * Try attach driver below and query it's media interface.
     */
    PPDMIBASE pBase;
    rc = PDMDrvHlpAttach(pDrvIns, fFlags, &pBase);
    if (    rc == VERR_PDM_NO_ATTACHED_DRIVER
        &&  pThis->enmType != PDMBLOCKTYPE_HARD_DISK)
        return VINF_SUCCESS;
    if (RT_FAILURE(rc))
        return PDMDrvHlpVMSetError(pDrvIns, rc, RT_SRC_POS,
                                   N_("Failed to attach driver below us! %Rrf"), rc);

    pThis->pDrvMedia = PDMIBASE_QUERY_INTERFACE(pBase, PDMIMEDIA);
    if (!pThis->pDrvMedia)
        return PDMDRV_SET_ERROR(pDrvIns, VERR_PDM_MISSING_INTERFACE_BELOW,
                                N_("No media or async media interface below"));

    /* Try to get the optional async interface. */
    pThis->pDrvMediaAsync = PDMIBASE_QUERY_INTERFACE(pBase, PDMIMEDIAASYNC);

    if (pThis->pDrvMedia->pfnDiscard)
        pThis->IBlock.pfnDiscard = drvblockDiscard;

    if (   pThis->pDrvMediaAsync
        && pThis->pDrvMediaAsync->pfnStartDiscard)
        pThis->IBlockAsync.pfnStartDiscard = drvblockStartDiscard;

    if (RTUuidIsNull(&pThis->Uuid))
    {
        if (pThis->enmType == PDMBLOCKTYPE_HARD_DISK)
            pThis->pDrvMedia->pfnGetUuid(pThis->pDrvMedia, &pThis->Uuid);
    }

    return VINF_SUCCESS;
}


/**
 * Block driver registration record.
 */
const PDMDRVREG g_DrvBlock =
{
    /* u32Version */
    PDM_DRVREG_VERSION,
    /* szName */
    "Block",
    /* szRCMod */
    "",
    /* szR0Mod */
    "",
    /* pszDescription */
    "Generic block driver.",
    /* fFlags */
    PDM_DRVREG_FLAGS_HOST_BITS_DEFAULT,
    /* fClass. */
    PDM_DRVREG_CLASS_BLOCK,
    /* cMaxInstances */
    ~0U,
    /* cbInstance */
    sizeof(DRVBLOCK),
    /* pfnConstruct */
    drvblockConstruct,
    /* pfnDestruct */
    NULL,
    /* pfnRelocate */
    NULL,
    /* pfnIOCtl */
    NULL,
    /* pfnPowerOn */
    NULL,
    /* pfnReset */
    drvblockReset,
    /* pfnSuspend */
    NULL,
    /* pfnResume */
    NULL,
    /* pfnAttach */
    NULL,
    /* pfnDetach */
    drvblockDetach,
    /* pfnPowerOff */
    NULL,
    /* pfnSoftReset */
    NULL,
    /* u32EndVersion */
    PDM_DRVREG_VERSION
};

