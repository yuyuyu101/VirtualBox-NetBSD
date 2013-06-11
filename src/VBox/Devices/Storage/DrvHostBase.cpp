/* $Id: DrvHostBase.cpp $ */
/** @file
 * DrvHostBase - Host base drive access driver.
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
 */


/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#define LOG_GROUP LOG_GROUP_DRV_HOST_BASE
#ifdef RT_OS_DARWIN
# include <mach/mach.h>
# include <Carbon/Carbon.h>
# include <IOKit/IOKitLib.h>
# include <IOKit/storage/IOStorageDeviceCharacteristics.h>
# include <IOKit/scsi/SCSITaskLib.h>
# include <IOKit/scsi/SCSICommandOperationCodes.h>
# include <IOKit/IOBSD.h>
# include <DiskArbitration/DiskArbitration.h>
# include <mach/mach_error.h>
# include <VBox/scsi.h>

#elif defined(RT_OS_L4)
  /* Nothing special requires... yeah, right. */

#elif defined(RT_OS_LINUX)
# include <sys/ioctl.h>
# include <sys/fcntl.h>
# include <errno.h>

#elif defined(RT_OS_SOLARIS)
# include <fcntl.h>
# include <errno.h>
# include <stropts.h>
# include <malloc.h>
# include <sys/dkio.h>
extern "C" char *getfullblkname(char *);

#elif defined(RT_OS_WINDOWS)
# define WIN32_NO_STATUS
# include <Windows.h>
# include <dbt.h>
# undef WIN32_NO_STATUS
# include <ntstatus.h>

/* from ntdef.h */
typedef LONG NTSTATUS;

/* from ntddk.h */
typedef struct _IO_STATUS_BLOCK {
    union {
        NTSTATUS Status;
        PVOID Pointer;
    };
    ULONG_PTR Information;
} IO_STATUS_BLOCK, *PIO_STATUS_BLOCK;


/* from ntinternals.com */
typedef enum _FS_INFORMATION_CLASS {
    FileFsVolumeInformation=1,
    FileFsLabelInformation,
    FileFsSizeInformation,
    FileFsDeviceInformation,
    FileFsAttributeInformation,
    FileFsControlInformation,
    FileFsFullSizeInformation,
    FileFsObjectIdInformation,
    FileFsMaximumInformation
} FS_INFORMATION_CLASS, *PFS_INFORMATION_CLASS;

typedef struct _FILE_FS_SIZE_INFORMATION {
    LARGE_INTEGER   TotalAllocationUnits;
    LARGE_INTEGER   AvailableAllocationUnits;
    ULONG           SectorsPerAllocationUnit;
    ULONG           BytesPerSector;
} FILE_FS_SIZE_INFORMATION, *PFILE_FS_SIZE_INFORMATION;

extern "C"
NTSTATUS __stdcall NtQueryVolumeInformationFile(
        /*IN*/ HANDLE               FileHandle,
        /*OUT*/ PIO_STATUS_BLOCK    IoStatusBlock,
        /*OUT*/ PVOID               FileSystemInformation,
        /*IN*/ ULONG                Length,
        /*IN*/ FS_INFORMATION_CLASS FileSystemInformationClass );

#elif defined(RT_OS_FREEBSD)
# include <sys/cdefs.h>
# include <sys/param.h>
# include <errno.h>
# include <stdio.h>
# include <cam/cam.h>
# include <cam/cam_ccb.h>
# include <cam/scsi/scsi_message.h>
# include <cam/scsi/scsi_pass.h>
# include <VBox/scsi.h>
# include <iprt/log.h>
#else
# error "Unsupported Platform."
#endif

#include <VBox/vmm/pdmdrv.h>
#include <iprt/assert.h>
#include <iprt/file.h>
#include <iprt/path.h>
#include <iprt/string.h>
#include <iprt/thread.h>
#include <iprt/semaphore.h>
#include <iprt/uuid.h>
#include <iprt/asm.h>
#include <iprt/critsect.h>
#include <iprt/ctype.h>

#include "DrvHostBase.h"




/* -=-=-=-=- IBlock -=-=-=-=- */

/** @copydoc PDMIBLOCK::pfnRead */
static DECLCALLBACK(int) drvHostBaseRead(PPDMIBLOCK pInterface, uint64_t off, void *pvBuf, size_t cbRead)
{
    PDRVHOSTBASE pThis = PDMIBLOCK_2_DRVHOSTBASE(pInterface);
    LogFlow(("%s-%d: drvHostBaseRead: off=%#llx pvBuf=%p cbRead=%#x (%s)\n",
             pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, off, pvBuf, cbRead, pThis->pszDevice));
    RTCritSectEnter(&pThis->CritSect);

    /*
     * Check the state.
     */
    int rc;
#ifdef RT_OS_DARWIN
    if (    pThis->fMediaPresent
        &&  pThis->ppScsiTaskDI
        &&  pThis->cbBlock)
#elif RT_OS_FREEBSD
    if (    pThis->fMediaPresent
        &&  pThis->cbBlock)
#else
    if (pThis->fMediaPresent)
#endif
    {
#if defined(RT_OS_DARWIN) || defined(RT_OS_FREEBSD)
        /*
         * Issue a READ(12) request.
         */
        do
        {
            const uint32_t  LBA       = off / pThis->cbBlock;
            AssertReturn(!(off % pThis->cbBlock), VERR_INVALID_PARAMETER);
            uint32_t        cbRead32  =   cbRead > SCSI_MAX_BUFFER_SIZE
                                        ? SCSI_MAX_BUFFER_SIZE
                                        : (uint32_t)cbRead;
            const uint32_t  cBlocks   = cbRead32 / pThis->cbBlock;
            AssertReturn(!(cbRead % pThis->cbBlock), VERR_INVALID_PARAMETER);
            uint8_t         abCmd[16] =
            {
                SCSI_READ_12, 0,
                RT_BYTE4(LBA),     RT_BYTE3(LBA),     RT_BYTE2(LBA),     RT_BYTE1(LBA),
                RT_BYTE4(cBlocks), RT_BYTE3(cBlocks), RT_BYTE2(cBlocks), RT_BYTE1(cBlocks),
                0, 0, 0, 0, 0
            };
            rc = DRVHostBaseScsiCmd(pThis, abCmd, 12, PDMBLOCKTXDIR_FROM_DEVICE, pvBuf, &cbRead32, NULL, 0, 0);

            off    += cbRead32;
            cbRead -= cbRead32;
            pvBuf   = (uint8_t *)pvBuf + cbRead32;
        } while ((cbRead > 0) && RT_SUCCESS(rc));

#else
        /*
         * Seek and read.
         */
        rc = RTFileReadAt(pThis->hFileDevice, off, pvBuf, cbRead, NULL);
        if (RT_SUCCESS(rc))
        {
            Log2(("%s-%d: drvHostBaseRead: off=%#llx cbRead=%#x\n"
                  "%16.*Rhxd\n",
                  pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, off, cbRead, cbRead, pvBuf));
        }
        else
            Log(("%s-%d: drvHostBaseRead: RTFileReadAt(%RTfile, %#llx, %p, %#x) -> %Rrc ('%s')\n",
                 pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, pThis->hFileDevice,
                 off, pvBuf, cbRead, rc, pThis->pszDevice));
#endif
    }
    else
        rc = VERR_MEDIA_NOT_PRESENT;

    RTCritSectLeave(&pThis->CritSect);
    LogFlow(("%s-%d: drvHostBaseRead: returns %Rrc\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, rc));
    return rc;
}


/** @copydoc PDMIBLOCK::pfnWrite */
static DECLCALLBACK(int) drvHostBaseWrite(PPDMIBLOCK pInterface, uint64_t off, const void *pvBuf, size_t cbWrite)
{
    PDRVHOSTBASE pThis = PDMIBLOCK_2_DRVHOSTBASE(pInterface);
    LogFlow(("%s-%d: drvHostBaseWrite: off=%#llx pvBuf=%p cbWrite=%#x (%s)\n",
             pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, off, pvBuf, cbWrite, pThis->pszDevice));
    Log2(("%s-%d: drvHostBaseWrite: off=%#llx cbWrite=%#x\n"
          "%16.*Rhxd\n",
          pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, off, cbWrite, cbWrite, pvBuf));
    RTCritSectEnter(&pThis->CritSect);

    /*
     * Check the state.
     */
    int rc;
    if (!pThis->fReadOnly)
    {
        if (pThis->fMediaPresent)
        {
#if defined(RT_OS_DARWIN) || defined(RT_OS_FREEBSD)
            /** @todo write support... */
            rc = VERR_WRITE_PROTECT;

#else
            /*
             * Seek and write.
             */
            rc = RTFileWriteAt(pThis->hFileDevice, off, pvBuf, cbWrite, NULL);
            if (RT_FAILURE(rc))
                Log(("%s-%d: drvHostBaseWrite: RTFileWriteAt(%RTfile, %#llx, %p, %#x) -> %Rrc ('%s')\n",
                     pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, pThis->hFileDevice,
                     off, pvBuf, cbWrite, rc, pThis->pszDevice));
#endif
        }
        else
            rc = VERR_MEDIA_NOT_PRESENT;
    }
    else
        rc = VERR_WRITE_PROTECT;

    RTCritSectLeave(&pThis->CritSect);
    LogFlow(("%s-%d: drvHostBaseWrite: returns %Rrc\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, rc));
    return rc;
}


/** @copydoc PDMIBLOCK::pfnFlush */
static DECLCALLBACK(int) drvHostBaseFlush(PPDMIBLOCK pInterface)
{
    int rc;
    PDRVHOSTBASE pThis = PDMIBLOCK_2_DRVHOSTBASE(pInterface);
    LogFlow(("%s-%d: drvHostBaseFlush: (%s)\n",
             pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, pThis->pszDevice));
    RTCritSectEnter(&pThis->CritSect);

    if (pThis->fMediaPresent)
    {
#if defined(RT_OS_DARWIN) || defined(RT_OS_FREEBSD)
        rc = VINF_SUCCESS;
        /** @todo scsi device buffer flush... */
#else
        rc = RTFileFlush(pThis->hFileDevice);
#endif
    }
    else
        rc = VERR_MEDIA_NOT_PRESENT;

    RTCritSectLeave(&pThis->CritSect);
    LogFlow(("%s-%d: drvHostBaseFlush: returns %Rrc\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, rc));
    return rc;
}


/** @copydoc PDMIBLOCK::pfnIsReadOnly */
static DECLCALLBACK(bool) drvHostBaseIsReadOnly(PPDMIBLOCK pInterface)
{
    PDRVHOSTBASE pThis = PDMIBLOCK_2_DRVHOSTBASE(pInterface);
    return pThis->fReadOnly;
}


/** @copydoc PDMIBLOCK::pfnGetSize */
static DECLCALLBACK(uint64_t) drvHostBaseGetSize(PPDMIBLOCK pInterface)
{
    PDRVHOSTBASE pThis = PDMIBLOCK_2_DRVHOSTBASE(pInterface);
    RTCritSectEnter(&pThis->CritSect);

    uint64_t cb = 0;
    if (pThis->fMediaPresent)
        cb = pThis->cbSize;

    RTCritSectLeave(&pThis->CritSect);
    LogFlow(("%s-%d: drvHostBaseGetSize: returns %llu\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, cb));
    return cb;
}


/** @copydoc PDMIBLOCK::pfnGetType */
static DECLCALLBACK(PDMBLOCKTYPE) drvHostBaseGetType(PPDMIBLOCK pInterface)
{
    PDRVHOSTBASE pThis = PDMIBLOCK_2_DRVHOSTBASE(pInterface);
    LogFlow(("%s-%d: drvHostBaseGetType: returns %d\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, pThis->enmType));
    return pThis->enmType;
}


/** @copydoc PDMIBLOCK::pfnGetUuid */
static DECLCALLBACK(int) drvHostBaseGetUuid(PPDMIBLOCK pInterface, PRTUUID pUuid)
{
    PDRVHOSTBASE pThis = PDMIBLOCK_2_DRVHOSTBASE(pInterface);

    *pUuid = pThis->Uuid;

    LogFlow(("%s-%d: drvHostBaseGetUuid: returns VINF_SUCCESS *pUuid=%RTuuid\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, pUuid));
    return VINF_SUCCESS;
}


/* -=-=-=-=- IBlockBios -=-=-=-=- */

/** Makes a PDRVHOSTBASE out of a PPDMIBLOCKBIOS. */
#define PDMIBLOCKBIOS_2_DRVHOSTBASE(pInterface)    ( (PDRVHOSTBASE((uintptr_t)pInterface - RT_OFFSETOF(DRVHOSTBASE, IBlockBios))) )


/** @copydoc PDMIBLOCKBIOS::pfnGetPCHSGeometry */
static DECLCALLBACK(int) drvHostBaseGetPCHSGeometry(PPDMIBLOCKBIOS pInterface, PPDMMEDIAGEOMETRY pPCHSGeometry)
{
    PDRVHOSTBASE pThis =  PDMIBLOCKBIOS_2_DRVHOSTBASE(pInterface);
    RTCritSectEnter(&pThis->CritSect);

    int rc = VINF_SUCCESS;
    if (pThis->fMediaPresent)
    {
        if (    pThis->PCHSGeometry.cCylinders > 0
            &&  pThis->PCHSGeometry.cHeads > 0
            &&  pThis->PCHSGeometry.cSectors > 0)
        {
            *pPCHSGeometry = pThis->PCHSGeometry;
        }
        else
            rc = VERR_PDM_GEOMETRY_NOT_SET;
    }
    else
        rc = VERR_PDM_MEDIA_NOT_MOUNTED;

    RTCritSectLeave(&pThis->CritSect);
    LogFlow(("%s-%d: %s: returns %Rrc CHS={%d,%d,%d}\n",
             pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, __FUNCTION__, rc, pThis->PCHSGeometry.cCylinders, pThis->PCHSGeometry.cHeads, pThis->PCHSGeometry.cSectors));
    return rc;
}


/** @copydoc PDMIBLOCKBIOS::pfnSetPCHSGeometry */
static DECLCALLBACK(int) drvHostBaseSetPCHSGeometry(PPDMIBLOCKBIOS pInterface, PCPDMMEDIAGEOMETRY pPCHSGeometry)
{
    PDRVHOSTBASE pThis = PDMIBLOCKBIOS_2_DRVHOSTBASE(pInterface);
    LogFlow(("%s-%d: %s: cCylinders=%d cHeads=%d cSectors=%d\n",
             pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, __FUNCTION__, pPCHSGeometry->cCylinders, pPCHSGeometry->cHeads, pPCHSGeometry->cSectors));
    RTCritSectEnter(&pThis->CritSect);

    int rc = VINF_SUCCESS;
    if (pThis->fMediaPresent)
    {
        pThis->PCHSGeometry = *pPCHSGeometry;
    }
    else
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        rc = VERR_PDM_MEDIA_NOT_MOUNTED;
    }

    RTCritSectLeave(&pThis->CritSect);
    return rc;
}


/** @copydoc PDMIBLOCKBIOS::pfnGetLCHSGeometry */
static DECLCALLBACK(int) drvHostBaseGetLCHSGeometry(PPDMIBLOCKBIOS pInterface, PPDMMEDIAGEOMETRY pLCHSGeometry)
{
    PDRVHOSTBASE pThis =  PDMIBLOCKBIOS_2_DRVHOSTBASE(pInterface);
    RTCritSectEnter(&pThis->CritSect);

    int rc = VINF_SUCCESS;
    if (pThis->fMediaPresent)
    {
        if (    pThis->LCHSGeometry.cCylinders > 0
            &&  pThis->LCHSGeometry.cHeads > 0
            &&  pThis->LCHSGeometry.cSectors > 0)
        {
            *pLCHSGeometry = pThis->LCHSGeometry;
        }
        else
            rc = VERR_PDM_GEOMETRY_NOT_SET;
    }
    else
        rc = VERR_PDM_MEDIA_NOT_MOUNTED;

    RTCritSectLeave(&pThis->CritSect);
    LogFlow(("%s-%d: %s: returns %Rrc CHS={%d,%d,%d}\n",
             pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, __FUNCTION__, rc, pThis->LCHSGeometry.cCylinders, pThis->LCHSGeometry.cHeads, pThis->LCHSGeometry.cSectors));
    return rc;
}


/** @copydoc PDMIBLOCKBIOS::pfnSetLCHSGeometry */
static DECLCALLBACK(int) drvHostBaseSetLCHSGeometry(PPDMIBLOCKBIOS pInterface, PCPDMMEDIAGEOMETRY pLCHSGeometry)
{
    PDRVHOSTBASE pThis = PDMIBLOCKBIOS_2_DRVHOSTBASE(pInterface);
    LogFlow(("%s-%d: %s: cCylinders=%d cHeads=%d cSectors=%d\n",
             pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, __FUNCTION__, pLCHSGeometry->cCylinders, pLCHSGeometry->cHeads, pLCHSGeometry->cSectors));
    RTCritSectEnter(&pThis->CritSect);

    int rc = VINF_SUCCESS;
    if (pThis->fMediaPresent)
    {
        pThis->LCHSGeometry = *pLCHSGeometry;
    }
    else
    {
        AssertMsgFailed(("Invalid state! Not mounted!\n"));
        rc = VERR_PDM_MEDIA_NOT_MOUNTED;
    }

    RTCritSectLeave(&pThis->CritSect);
    return rc;
}


/** @copydoc PDMIBLOCKBIOS::pfnIsVisible */
static DECLCALLBACK(bool) drvHostBaseIsVisible(PPDMIBLOCKBIOS pInterface)
{
    PDRVHOSTBASE pThis = PDMIBLOCKBIOS_2_DRVHOSTBASE(pInterface);
    return pThis->fBiosVisible;
}


/** @copydoc PDMIBLOCKBIOS::pfnGetType */
static DECLCALLBACK(PDMBLOCKTYPE) drvHostBaseBiosGetType(PPDMIBLOCKBIOS pInterface)
{
    PDRVHOSTBASE pThis = PDMIBLOCKBIOS_2_DRVHOSTBASE(pInterface);
    return pThis->enmType;
}



/* -=-=-=-=- IMount -=-=-=-=- */

/** @copydoc PDMIMOUNT::pfnMount */
static DECLCALLBACK(int) drvHostBaseMount(PPDMIMOUNT pInterface, const char *pszFilename, const char *pszCoreDriver)
{
    /* We're not mountable. */
    AssertMsgFailed(("drvHostBaseMount: This shouldn't be called!\n"));
    return VERR_PDM_MEDIA_MOUNTED;
}


/** @copydoc PDMIMOUNT::pfnUnmount */
static DECLCALLBACK(int) drvHostBaseUnmount(PPDMIMOUNT pInterface, bool fForce, bool fEject)
{
    /* While we're not mountable (see drvHostBaseMount), we're unmountable. */
    PDRVHOSTBASE pThis = PDMIMOUNT_2_DRVHOSTBASE(pInterface);
    RTCritSectEnter(&pThis->CritSect);

    /*
     * Validate state.
     */
    int rc = VINF_SUCCESS;
    if (!pThis->fLocked || fForce)
    {
        /* Unlock drive if necessary. */
        if (pThis->fLocked)
        {
            if (pThis->pfnDoLock)
                rc = pThis->pfnDoLock(pThis, false);
            if (RT_SUCCESS(rc))
                pThis->fLocked = false;
        }

        /*
         * Media is no longer present.
         */
        DRVHostBaseMediaNotPresent(pThis);
    }
    else
    {
        Log(("drvHostiBaseUnmount: Locked\n"));
        rc = VERR_PDM_MEDIA_LOCKED;
    }

    RTCritSectLeave(&pThis->CritSect);
    LogFlow(("drvHostBaseUnmount: returns %Rrc\n", rc));
    return rc;
}


/** @copydoc PDMIMOUNT::pfnIsMounted */
static DECLCALLBACK(bool) drvHostBaseIsMounted(PPDMIMOUNT pInterface)
{
    PDRVHOSTBASE pThis = PDMIMOUNT_2_DRVHOSTBASE(pInterface);
    RTCritSectEnter(&pThis->CritSect);

    bool fRc = pThis->fMediaPresent;

    RTCritSectLeave(&pThis->CritSect);
    return fRc;
}


/** @copydoc PDMIMOUNT::pfnIsLocked */
static DECLCALLBACK(int) drvHostBaseLock(PPDMIMOUNT pInterface)
{
    PDRVHOSTBASE pThis = PDMIMOUNT_2_DRVHOSTBASE(pInterface);
    RTCritSectEnter(&pThis->CritSect);

    int rc = VINF_SUCCESS;
    if (!pThis->fLocked)
    {
        if (pThis->pfnDoLock)
            rc = pThis->pfnDoLock(pThis, true);
        if (RT_SUCCESS(rc))
            pThis->fLocked = true;
    }
    else
        LogFlow(("%s-%d: drvHostBaseLock: already locked\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance));

    RTCritSectLeave(&pThis->CritSect);
    LogFlow(("%s-%d: drvHostBaseLock: returns %Rrc\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, rc));
    return rc;
}


/** @copydoc PDMIMOUNT::pfnIsLocked */
static DECLCALLBACK(int) drvHostBaseUnlock(PPDMIMOUNT pInterface)
{
    PDRVHOSTBASE pThis = PDMIMOUNT_2_DRVHOSTBASE(pInterface);
    RTCritSectEnter(&pThis->CritSect);

    int rc = VINF_SUCCESS;
    if (pThis->fLocked)
    {
        if (pThis->pfnDoLock)
            rc = pThis->pfnDoLock(pThis, false);
        if (RT_SUCCESS(rc))
            pThis->fLocked = false;
    }
    else
        LogFlow(("%s-%d: drvHostBaseUnlock: not locked\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance));

    RTCritSectLeave(&pThis->CritSect);
    LogFlow(("%s-%d: drvHostBaseUnlock: returns %Rrc\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, rc));
    return rc;
}


/** @copydoc PDMIMOUNT::pfnIsLocked */
static DECLCALLBACK(bool) drvHostBaseIsLocked(PPDMIMOUNT pInterface)
{
    PDRVHOSTBASE pThis = PDMIMOUNT_2_DRVHOSTBASE(pInterface);
    RTCritSectEnter(&pThis->CritSect);

    bool fRc = pThis->fLocked;

    RTCritSectLeave(&pThis->CritSect);
    return fRc;
}


/* -=-=-=-=- IBase -=-=-=-=- */

/**
 * @interface_method_impl{PDMIBASE,pfnQueryInterface}
 */
static DECLCALLBACK(void *)  drvHostBaseQueryInterface(PPDMIBASE pInterface, const char *pszIID)
{
    PPDMDRVINS   pDrvIns = PDMIBASE_2_PDMDRV(pInterface);
    PDRVHOSTBASE pThis   = PDMINS_2_DATA(pDrvIns, PDRVHOSTBASE);

    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBASE, &pDrvIns->IBase);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBLOCK, &pThis->IBlock);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBLOCKBIOS, pThis->fBiosVisible ? &pThis->IBlockBios : NULL);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIMOUNT, &pThis->IMount);
    return NULL;
}


/* -=-=-=-=- poller thread -=-=-=-=- */

#ifdef RT_OS_DARWIN
/** The runloop input source name for the disk arbitration events. */
# define MY_RUN_LOOP_MODE    CFSTR("drvHostBaseDA") /** @todo r=bird: Check if this will cause trouble in the same way that the one in the USB code did. */

/**
 * Gets the BSD Name (/dev/disc[0-9]+) for the service.
 *
 * This is done by recursing down the I/O registry until we hit upon an entry
 * with a BSD Name. Usually we find it two levels down. (Further down under
 * the IOCDPartitionScheme, the volume (slices) BSD Name is found. We don't
 * seem to have to go this far fortunately.)
 *
 * @return  VINF_SUCCESS if found, VERR_FILE_NOT_FOUND otherwise.
 * @param   Entry       The current I/O registry entry reference.
 * @param   pszName     Where to store the name. 128 bytes.
 * @param   cRecursions Number of recursions. This is used as an precaution
 *                      just to limit the depth and avoid blowing the stack
 *                      should we hit a bug or something.
 */
static int drvHostBaseGetBSDName(io_registry_entry_t Entry, char *pszName, unsigned cRecursions)
{
    int rc = VERR_FILE_NOT_FOUND;
    io_iterator_t Children = 0;
    kern_return_t krc = IORegistryEntryGetChildIterator(Entry, kIOServicePlane, &Children);
    if (krc == KERN_SUCCESS)
    {
        io_object_t Child;
        while (     rc == VERR_FILE_NOT_FOUND
               &&   (Child = IOIteratorNext(Children)) != 0)
        {
            CFStringRef BSDNameStrRef = (CFStringRef)IORegistryEntryCreateCFProperty(Child, CFSTR(kIOBSDNameKey), kCFAllocatorDefault, 0);
            if (BSDNameStrRef)
            {
                if (CFStringGetCString(BSDNameStrRef, pszName, 128, kCFStringEncodingUTF8))
                    rc = VINF_SUCCESS;
                else
                    AssertFailed();
                CFRelease(BSDNameStrRef);
            }
            if (rc == VERR_FILE_NOT_FOUND && cRecursions < 10)
                rc = drvHostBaseGetBSDName(Child, pszName, cRecursions + 1);
            IOObjectRelease(Child);
        }
        IOObjectRelease(Children);
    }
    return rc;
}


/**
 * Callback notifying us that the async DADiskClaim()/DADiskUnmount call has completed.
 *
 * @param   DiskRef         The disk that was attempted claimed / unmounted.
 * @param   DissenterRef    NULL on success, contains details on failure.
 * @param   pvContext       Pointer to the return code variable.
 */
static void drvHostBaseDADoneCallback(DADiskRef DiskRef, DADissenterRef DissenterRef, void *pvContext)
{
    int *prc = (int *)pvContext;
    if (!DissenterRef)
        *prc = 0;
    else
        *prc = DADissenterGetStatus(DissenterRef) ? DADissenterGetStatus(DissenterRef) : -1;
    CFRunLoopStop(CFRunLoopGetCurrent());
}


/**
 * Obtain exclusive access to the DVD device, umount it if necessary.
 *
 * @return  VBox status code.
 * @param   pThis       The driver instance.
 * @param   DVDService  The DVD service object.
 */
static int drvHostBaseObtainExclusiveAccess(PDRVHOSTBASE pThis, io_object_t DVDService)
{
    PPDMDRVINS pDrvIns = pThis->pDrvIns; NOREF(pDrvIns);

    for (unsigned iTry = 0;; iTry++)
    {
        IOReturn irc = (*pThis->ppScsiTaskDI)->ObtainExclusiveAccess(pThis->ppScsiTaskDI);
        if (irc == kIOReturnSuccess)
        {
            /*
             * This is a bit weird, but if we unmounted the DVD drive we also need to
             * unlock it afterwards or the guest won't be able to eject it later on.
             */
            if (pThis->pDADisk)
            {
                uint8_t abCmd[16] =
                {
                    SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL, 0, 0, 0, false, 0,
                    0,0,0,0,0,0,0,0,0,0
                };
                DRVHostBaseScsiCmd(pThis, abCmd, 6, PDMBLOCKTXDIR_NONE, NULL, NULL, NULL, 0, 0);
            }
            return VINF_SUCCESS;
        }
        if (irc == kIOReturnExclusiveAccess)
            return VERR_SHARING_VIOLATION;      /* already used exclusivly. */
        if (irc != kIOReturnBusy)
            return VERR_GENERAL_FAILURE;        /* not mounted */

        /*
         * Attempt to the unmount all volumes of the device.
         * It seems we can can do this all in one go without having to enumerate the
         * volumes (sessions) and deal with them one by one. This is very fortuitous
         * as the disk arbitration API is a bit cumbersome to deal with.
         */
        if (iTry > 2)
            return VERR_DRIVE_LOCKED;
        char szName[128];
        int rc = drvHostBaseGetBSDName(DVDService, &szName[0], 0);
        if (RT_SUCCESS(rc))
        {
            pThis->pDASession = DASessionCreate(kCFAllocatorDefault);
            if (pThis->pDASession)
            {
                DASessionScheduleWithRunLoop(pThis->pDASession, CFRunLoopGetCurrent(), MY_RUN_LOOP_MODE);
                pThis->pDADisk = DADiskCreateFromBSDName(kCFAllocatorDefault, pThis->pDASession, szName);
                if (pThis->pDADisk)
                {
                    /*
                     * Try claim the device.
                     */
                    Log(("%s-%d: calling DADiskClaim on '%s'.\n", pDrvIns->pReg->szName, pDrvIns->iInstance, szName));
                    int rcDA = -2;
                    DADiskClaim(pThis->pDADisk, kDADiskClaimOptionDefault, NULL, NULL, drvHostBaseDADoneCallback, &rcDA);
                    SInt32 rc32 = CFRunLoopRunInMode(MY_RUN_LOOP_MODE, 120.0, FALSE);
                    AssertMsg(rc32 == kCFRunLoopRunStopped, ("rc32=%RI32 (%RX32)\n", rc32, rc32));
                    if (    rc32 == kCFRunLoopRunStopped
                        &&  !rcDA)
                    {
                        /*
                         * Try unmount the device.
                         */
                        Log(("%s-%d: calling DADiskUnmount on '%s'.\n", pDrvIns->pReg->szName, pDrvIns->iInstance, szName));
                        rcDA = -2;
                        DADiskUnmount(pThis->pDADisk, kDADiskUnmountOptionWhole, drvHostBaseDADoneCallback, &rcDA);
                        rc32 = CFRunLoopRunInMode(MY_RUN_LOOP_MODE, 120.0, FALSE);
                        AssertMsg(rc32 == kCFRunLoopRunStopped, ("rc32=%RI32 (%RX32)\n", rc32, rc32));
                        if (    rc32 == kCFRunLoopRunStopped
                            &&  !rcDA)
                        {
                            iTry = 99;
                            DASessionUnscheduleFromRunLoop(pThis->pDASession, CFRunLoopGetCurrent(), MY_RUN_LOOP_MODE);
                            Log(("%s-%d: unmount succeed - retrying.\n", pDrvIns->pReg->szName, pDrvIns->iInstance));
                            continue;
                        }
                        Log(("%s-%d: umount => rc32=%d & rcDA=%#x\n", pDrvIns->pReg->szName, pDrvIns->iInstance, rc32, rcDA));

                        /* failed - cleanup */
                        DADiskUnclaim(pThis->pDADisk);
                    }
                    else
                        Log(("%s-%d: claim => rc32=%d & rcDA=%#x\n", pDrvIns->pReg->szName, pDrvIns->iInstance, rc32, rcDA));

                    CFRelease(pThis->pDADisk);
                    pThis->pDADisk = NULL;
                }
                else
                    Log(("%s-%d: failed to open disk '%s'!\n", pDrvIns->pReg->szName, pDrvIns->iInstance, szName));

                DASessionUnscheduleFromRunLoop(pThis->pDASession, CFRunLoopGetCurrent(), MY_RUN_LOOP_MODE);
                CFRelease(pThis->pDASession);
                pThis->pDASession = NULL;
            }
            else
                Log(("%s-%d: failed to create DA session!\n", pDrvIns->pReg->szName, pDrvIns->iInstance));
        }
        RTThreadSleep(10);
    }
}
#endif /* RT_OS_DARWIN */


#ifndef RT_OS_SOLARIS
/**
 * Wrapper for open / RTFileOpen / IOKit.
 *
 * @remark  The Darwin code must correspond exactly to the enumeration
 *          done in Main/darwin/iokit.c.
 */
static int drvHostBaseOpen(PDRVHOSTBASE pThis, PRTFILE pFileDevice, bool fReadOnly)
{
# ifdef RT_OS_DARWIN
    /* Darwin is kind of special... */
    Assert(!pFileDevice); NOREF(pFileDevice);
    Assert(!pThis->cbBlock);
    Assert(!pThis->MasterPort);
    Assert(!pThis->ppMMCDI);
    Assert(!pThis->ppScsiTaskDI);

    /*
     * Open the master port on the first invocation.
     */
    kern_return_t krc = IOMasterPort(MACH_PORT_NULL, &pThis->MasterPort);
    AssertReturn(krc == KERN_SUCCESS, VERR_GENERAL_FAILURE);

    /*
     * Create a matching dictionary for searching for DVD services in the IOKit.
     *
     * [If I understand this correctly, plain CDROMs doesn't show up as
     * IODVDServices. Too keep things simple, we will only support DVDs
     * until somebody complains about it and we get hardware to test it on.
     * (Unless I'm much mistaken, there aren't any (orignal) intel macs with
     * plain cdroms.)]
     */
    CFMutableDictionaryRef RefMatchingDict = IOServiceMatching("IODVDServices");
    AssertReturn(RefMatchingDict, NULL);

    /*
     * do the search and get a collection of keyboards.
     */
    io_iterator_t DVDServices = NULL;
    IOReturn irc = IOServiceGetMatchingServices(pThis->MasterPort, RefMatchingDict, &DVDServices);
    AssertMsgReturn(irc == kIOReturnSuccess, ("irc=%d\n", irc), NULL);
    RefMatchingDict = NULL; /* the reference is consumed by IOServiceGetMatchingServices. */

    /*
     * Enumerate the DVD drives (services).
     * (This enumeration must be identical to the one performed in DrvHostBase.cpp.)
     */
    int rc = VERR_FILE_NOT_FOUND;
    unsigned i = 0;
    io_object_t DVDService;
    while ((DVDService = IOIteratorNext(DVDServices)) != 0)
    {
        /*
         * Get the properties we use to identify the DVD drive.
         *
         * While there is a (weird 12 byte) GUID, it isn't persistent
         * across boots. So, we have to use a combination of the
         * vendor name and product name properties with an optional
         * sequence number for identification.
         */
        CFMutableDictionaryRef PropsRef = 0;
        krc = IORegistryEntryCreateCFProperties(DVDService, &PropsRef, kCFAllocatorDefault, kNilOptions);
        if (krc == KERN_SUCCESS)
        {
            /* Get the Device Characteristics dictionary. */
            CFDictionaryRef DevCharRef = (CFDictionaryRef)CFDictionaryGetValue(PropsRef, CFSTR(kIOPropertyDeviceCharacteristicsKey));
            if (DevCharRef)
            {
                /* The vendor name. */
                char szVendor[128];
                char *pszVendor = &szVendor[0];
                CFTypeRef ValueRef = CFDictionaryGetValue(DevCharRef, CFSTR(kIOPropertyVendorNameKey));
                if (    ValueRef
                    &&  CFGetTypeID(ValueRef) == CFStringGetTypeID()
                    &&  CFStringGetCString((CFStringRef)ValueRef, szVendor, sizeof(szVendor), kCFStringEncodingUTF8))
                    pszVendor = RTStrStrip(szVendor);
                else
                    *pszVendor = '\0';

                /* The product name. */
                char szProduct[128];
                char *pszProduct = &szProduct[0];
                ValueRef = CFDictionaryGetValue(DevCharRef, CFSTR(kIOPropertyProductNameKey));
                if (    ValueRef
                    &&  CFGetTypeID(ValueRef) == CFStringGetTypeID()
                    &&  CFStringGetCString((CFStringRef)ValueRef, szProduct, sizeof(szProduct), kCFStringEncodingUTF8))
                    pszProduct = RTStrStrip(szProduct);
                else
                    *pszProduct = '\0';

                /* Construct the two names and compare thwm with the one we're searching for. */
                char szName1[256 + 32];
                char szName2[256 + 32];
                if (*pszVendor || *pszProduct)
                {
                    if (*pszVendor && *pszProduct)
                    {
                        RTStrPrintf(szName1, sizeof(szName1), "%s %s", pszVendor, pszProduct);
                        RTStrPrintf(szName2, sizeof(szName2), "%s %s (#%u)", pszVendor, pszProduct, i);
                    }
                    else
                    {
                        strcpy(szName1, *pszVendor ? pszVendor : pszProduct);
                        RTStrPrintf(szName2, sizeof(szName2), "%s %s (#%u)", *pszVendor ? pszVendor : pszProduct, i);
                    }
                }
                else
                {
                    RTStrPrintf(szName1, sizeof(szName1), "(#%u)", i);
                    strcpy(szName2, szName1);
                }

                if (    !strcmp(szName1, pThis->pszDeviceOpen)
                    ||  !strcmp(szName2, pThis->pszDeviceOpen))
                {
                    /*
                     * Found it! Now, get the client interface and stuff.
                     * Note that we could also query kIOSCSITaskDeviceUserClientTypeID here if the
                     * MMC client plugin is missing. For now we assume this won't be necessary.
                     */
                    SInt32 Score = 0;
                    IOCFPlugInInterface **ppPlugInInterface = NULL;
                    krc = IOCreatePlugInInterfaceForService(DVDService, kIOMMCDeviceUserClientTypeID, kIOCFPlugInInterfaceID,
                                                            &ppPlugInInterface, &Score);
                    if (krc == KERN_SUCCESS)
                    {
                        HRESULT hrc = (*ppPlugInInterface)->QueryInterface(ppPlugInInterface,
                                                                           CFUUIDGetUUIDBytes(kIOMMCDeviceInterfaceID),
                                                                           (LPVOID *)&pThis->ppMMCDI);
                        (*ppPlugInInterface)->Release(ppPlugInInterface);
                        ppPlugInInterface = NULL;
                        if (hrc == S_OK)
                        {
                            pThis->ppScsiTaskDI = (*pThis->ppMMCDI)->GetSCSITaskDeviceInterface(pThis->ppMMCDI);
                            if (pThis->ppScsiTaskDI)
                                rc = VINF_SUCCESS;
                            else
                            {
                                LogRel(("GetSCSITaskDeviceInterface failed on '%s'\n", pThis->pszDeviceOpen));
                                rc = VERR_NOT_SUPPORTED;
                                (*pThis->ppMMCDI)->Release(pThis->ppMMCDI);
                            }
                        }
                        else
                        {
                            rc = VERR_GENERAL_FAILURE;//RTErrConvertFromDarwinCOM(krc);
                            pThis->ppMMCDI = NULL;
                        }
                    }
                    else /* Check for kIOSCSITaskDeviceUserClientTypeID? */
                        rc = VERR_GENERAL_FAILURE;//RTErrConvertFromDarwinKern(krc);

                    /* Obtain exclusive access to the device so we can send SCSI commands. */
                    if (RT_SUCCESS(rc))
                        rc = drvHostBaseObtainExclusiveAccess(pThis, DVDService);

                    /* Cleanup on failure. */
                    if (RT_FAILURE(rc))
                    {
                        if (pThis->ppScsiTaskDI)
                        {
                            (*pThis->ppScsiTaskDI)->Release(pThis->ppScsiTaskDI);
                            pThis->ppScsiTaskDI = NULL;
                        }
                        if (pThis->ppMMCDI)
                        {
                            (*pThis->ppMMCDI)->Release(pThis->ppMMCDI);
                            pThis->ppMMCDI = NULL;
                        }
                    }

                    IOObjectRelease(DVDService);
                    break;
                }
            }
            CFRelease(PropsRef);
        }
        else
            AssertMsgFailed(("krc=%#x\n", krc));

        IOObjectRelease(DVDService);
        i++;
    }

    IOObjectRelease(DVDServices);
    return rc;

#elif defined(RT_OS_FREEBSD)
    RTFILE hFileDevice;
    int rc = RTFileOpen(&hFileDevice, pThis->pszDeviceOpen, RTFILE_O_READWRITE | RTFILE_O_OPEN | RTFILE_O_DENY_NONE);
    if (RT_FAILURE(rc))
        return rc;

    /*
     * The current device handle can't passthrough SCSI commands.
     * We have to get he passthrough device path and open this.
     */
    union ccb DeviceCCB;
    memset(&DeviceCCB, 0, sizeof(DeviceCCB));

    DeviceCCB.ccb_h.func_code = XPT_GDEVLIST;
    int rcBSD = ioctl(RTFileToNative(hFileDevice), CAMGETPASSTHRU, &DeviceCCB);
    if (!rcBSD)
    {
        char *pszPassthroughDevice = NULL;
        rc = RTStrAPrintf(&pszPassthroughDevice, "/dev/%s%u",
                          DeviceCCB.cgdl.periph_name, DeviceCCB.cgdl.unit_number);
        if (rc >= 0)
        {
            RTFILE hPassthroughDevice;
            rc = RTFileOpen(&hPassthroughDevice, pszPassthroughDevice, RTFILE_O_READWRITE | RTFILE_O_OPEN | RTFILE_O_DENY_NONE);
            RTStrFree(pszPassthroughDevice);
            if (RT_SUCCESS(rc))
            {
                /* Get needed device parameters. */

                /*
                 * The device path, target id and lun id. Those are
                 * needed for the SCSI passthrough ioctl.
                 */
                memset(&DeviceCCB, 0, sizeof(DeviceCCB));
                DeviceCCB.ccb_h.func_code = XPT_GDEVLIST;

                rcBSD = ioctl(RTFileToNative(hPassthroughDevice), CAMGETPASSTHRU, &DeviceCCB);
                if (!rcBSD)
                {
                    if (DeviceCCB.cgdl.status != CAM_GDEVLIST_ERROR)
                    {
                        pThis->ScsiBus      = DeviceCCB.ccb_h.path_id;
                        pThis->ScsiTargetID = DeviceCCB.ccb_h.target_id;
                        pThis->ScsiLunID    = DeviceCCB.ccb_h.target_lun;
                        *pFileDevice = hPassthroughDevice;
                    }
                    else
                    {
                        /* The passthrough device wasn't found. */
                        rc = VERR_NOT_FOUND;
                    }
                }
                else
                    rc = RTErrConvertFromErrno(errno);

                if (RT_FAILURE(rc))
                    RTFileClose(hPassthroughDevice);
            }
        }
        else
            rc = VERR_NO_STR_MEMORY;
    }
    else
        rc = RTErrConvertFromErrno(errno);

    RTFileClose(hFileDevice);
    return rc;

#else
    uint32_t fFlags = (fReadOnly ? RTFILE_O_READ : RTFILE_O_READWRITE) | RTFILE_O_OPEN | RTFILE_O_DENY_NONE;
# ifdef RT_OS_LINUX
    fFlags |= RTFILE_O_NON_BLOCK;
# endif
    return RTFileOpen(pFileDevice, pThis->pszDeviceOpen, fFlags);
#endif
}

#else   /* RT_OS_SOLARIS */

/**
 * Solaris wrapper for RTFileOpen.
 *
 * Solaris has to deal with two filehandles, a block and a raw one. Rather than messing
 * with drvHostBaseOpen's function signature & body, having a separate one is better.
 *
 * @returns VBox status code.
 */
static int drvHostBaseOpen(PDRVHOSTBASE pThis, PRTFILE pFileBlockDevice, PRTFILE pFileRawDevice, bool fReadOnly)
{
    unsigned fFlags = (fReadOnly ? RTFILE_O_READ : RTFILE_O_READWRITE)
                    | RTFILE_O_OPEN | RTFILE_O_DENY_NONE | RTFILE_O_NON_BLOCK;
    int rc = RTFileOpen(pFileBlockDevice, pThis->pszDeviceOpen, fFlags);
    if (RT_SUCCESS(rc))
    {
        rc = RTFileOpen(pFileRawDevice, pThis->pszRawDeviceOpen, fFlags);
        if (RT_SUCCESS(rc))
            return rc;

        LogRel(("DVD: failed to open device %s rc=%Rrc\n", pThis->pszRawDeviceOpen, rc));
        RTFileClose(*pFileBlockDevice);
    }
    else
        LogRel(("DVD: failed to open device %s rc=%Rrc\n", pThis->pszDeviceOpen, rc));
    return rc;
}
#endif  /* RT_OS_SOLARIS */


/**
 * (Re)opens the device.
 *
 * This is used to open the device during construction, but it's also used to re-open
 * the device when a media is inserted. This re-open will kill off any cached data
 * that Linux for some peculiar reason thinks should survive a media change...
 *
 * @returns VBOX status code.
 * @param   pThis       Instance data.
 */
static int drvHostBaseReopen(PDRVHOSTBASE pThis)
{
#ifndef RT_OS_DARWIN /* Only *one* open for darwin. */
    LogFlow(("%s-%d: drvHostBaseReopen: '%s'\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, pThis->pszDeviceOpen));

    RTFILE hFileDevice;
#ifdef RT_OS_SOLARIS
    if (pThis->hFileRawDevice != NIL_RTFILE)
    {
        RTFileClose(pThis->hFileRawDevice);
        pThis->hFileRawDevice = NIL_RTFILE;
    }
    if (pThis->hFileDevice != NIL_RTFILE)
    {
        RTFileClose(pThis->hFileDevice);
        pThis->hFileDevice = NIL_RTFILE;
    }
    RTFILE hFileRawDevice;
    int rc = drvHostBaseOpen(pThis, &hFileDevice, &hFileRawDevice, pThis->fReadOnlyConfig);
#else
    int rc = drvHostBaseOpen(pThis, &hFileDevice, pThis->fReadOnlyConfig);
#endif
    if (RT_FAILURE(rc))
    {
        if (!pThis->fReadOnlyConfig)
        {
            LogFlow(("%s-%d: drvHostBaseReopen: '%s' - retry readonly (%Rrc)\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, pThis->pszDeviceOpen, rc));
#ifdef RT_OS_SOLARIS
            rc = drvHostBaseOpen(pThis, &hFileDevice, &hFileRawDevice, false);
#else
            rc = drvHostBaseOpen(pThis, &hFileDevice, false);
#endif
        }
        if (RT_FAILURE(rc))
        {
            LogFlow(("%s-%d: failed to open device '%s', rc=%Rrc\n",
                     pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, pThis->pszDevice, rc));
            return rc;
        }
        pThis->fReadOnly = true;
    }
    else
        pThis->fReadOnly = pThis->fReadOnlyConfig;

#ifdef RT_OS_SOLARIS
    if (pThis->hFileRawDevice != NIL_RTFILE)
        RTFileClose(pThis->hFileRawDevice);
    pThis->hFileRawDevice = hFileRawDevice;
#endif

    if (pThis->hFileDevice != NIL_RTFILE)
        RTFileClose(pThis->hFileDevice);
    pThis->hFileDevice = hFileDevice;
#endif /* !RT_OS_DARWIN */
    return VINF_SUCCESS;
}


/**
 * Queries the media size.
 *
 * @returns VBox status code.
 * @param   pThis       Pointer to the instance data.
 * @param   pcb         Where to store the media size in bytes.
 */
static int drvHostBaseGetMediaSize(PDRVHOSTBASE pThis, uint64_t *pcb)
{
#if defined(RT_OS_DARWIN) || defined(RT_OS_FREEBSD)
    /*
     * Try a READ_CAPACITY command...
     */
    struct
    {
        uint32_t cBlocks;
        uint32_t cbBlock;
    }           Buf = {0, 0};
    uint32_t    cbBuf = sizeof(Buf);
    uint8_t     abCmd[16] =
    {
        SCSI_READ_CAPACITY, 0, 0, 0, 0, 0, 0,
        0,0,0,0,0,0,0,0,0
    };
    int rc = DRVHostBaseScsiCmd(pThis, abCmd, 6, PDMBLOCKTXDIR_FROM_DEVICE, &Buf, &cbBuf, NULL, 0, 0);
    if (RT_SUCCESS(rc))
    {
        Assert(cbBuf == sizeof(Buf));
        Buf.cBlocks = RT_BE2H_U32(Buf.cBlocks);
        Buf.cbBlock = RT_BE2H_U32(Buf.cbBlock);
        //if (Buf.cbBlock > 2048) /* everyone else is doing this... check if it needed/right.*/
        //    Buf.cbBlock = 2048;
        pThis->cbBlock = Buf.cbBlock;

        *pcb = (uint64_t)Buf.cBlocks * Buf.cbBlock;
    }
    return rc;

#elif defined(RT_OS_SOLARIS)
    /*
     * Sun docs suggests using DKIOCGGEOM instead of DKIOCGMEDIAINFO, but
     * Sun themselves use DKIOCGMEDIAINFO for DVDs/CDs, and use DKIOCGGEOM
     * for secondary storage devices.
     */
    struct dk_minfo MediaInfo;
    if (ioctl(RTFileToNative(pThis->hFileRawDevice), DKIOCGMEDIAINFO, &MediaInfo) == 0)
    {
        *pcb = MediaInfo.dki_capacity * (uint64_t)MediaInfo.dki_lbsize;
        return VINF_SUCCESS;
    }
    return RTFileSeek(pThis->hFileDevice, 0, RTFILE_SEEK_END, pcb);

#elif defined(RT_OS_WINDOWS)
    /* use NT api, retry a few times if the media is being verified. */
    IO_STATUS_BLOCK             IoStatusBlock = {0};
    FILE_FS_SIZE_INFORMATION    FsSize= {0};
    NTSTATUS rcNt = NtQueryVolumeInformationFile((HANDLE)RTFileToNative(pThis->hFileDevice),  &IoStatusBlock,
                                                 &FsSize, sizeof(FsSize), FileFsSizeInformation);
    int cRetries = 5;
    while (rcNt == STATUS_VERIFY_REQUIRED && cRetries-- > 0)
    {
        RTThreadSleep(10);
        rcNt = NtQueryVolumeInformationFile((HANDLE)RTFileToNative(pThis->hFileDevice),  &IoStatusBlock,
                                            &FsSize, sizeof(FsSize), FileFsSizeInformation);
    }
    if (rcNt >= 0)
    {
        *pcb = FsSize.TotalAllocationUnits.QuadPart * FsSize.BytesPerSector;
        return VINF_SUCCESS;
    }

    /* convert nt status code to VBox status code. */
    /** @todo Make conversion function!. */
    int rc = VERR_GENERAL_FAILURE;
    switch (rcNt)
    {
        case STATUS_NO_MEDIA_IN_DEVICE:     rc = VERR_MEDIA_NOT_PRESENT; break;
        case STATUS_VERIFY_REQUIRED:        rc = VERR_TRY_AGAIN; break;
    }
    LogFlow(("drvHostBaseGetMediaSize: NtQueryVolumeInformationFile -> %#lx\n", rcNt, rc));
    return rc;
#else
    return RTFileSeek(pThis->hFileDevice, 0, RTFILE_SEEK_END, pcb);
#endif
}


#if defined(RT_OS_DARWIN) || defined(RT_OS_FREEBSD)
/**
 * Execute a SCSI command.
 *
 * @param pThis             The instance data.
 * @param pbCmd             Pointer to the SCSI command.
 * @param cbCmd             The size of the SCSI command.
 * @param enmTxDir          The transfer direction.
 * @param pvBuf             The buffer. Can be NULL if enmTxDir is PDMBLOCKTXDIR_NONE.
 * @param pcbBuf            Where to get the buffer size from and put the actual transfer size. Can be NULL.
 * @param pbSense           Where to put the sense data. Can be NULL.
 * @param cbSense           Size of the sense data buffer.
 * @param cTimeoutMillies   The timeout. 0 mean the default timeout.
 *
 * @returns VINF_SUCCESS on success (no sense code).
 * @returns VERR_UNRESOLVED_ERROR if sense code is present.
 * @returns Some other VBox status code on failures without sense code.
 *
 * @todo Fix VERR_UNRESOLVED_ERROR abuse.
 */
DECLCALLBACK(int) DRVHostBaseScsiCmd(PDRVHOSTBASE pThis, const uint8_t *pbCmd, size_t cbCmd, PDMBLOCKTXDIR enmTxDir,
                                     void *pvBuf, uint32_t *pcbBuf, uint8_t *pbSense, size_t cbSense, uint32_t cTimeoutMillies)
{
    /*
     * Minimal input validation.
     */
    Assert(enmTxDir == PDMBLOCKTXDIR_NONE || enmTxDir == PDMBLOCKTXDIR_FROM_DEVICE || enmTxDir == PDMBLOCKTXDIR_TO_DEVICE);
    Assert(!pvBuf || pcbBuf);
    Assert(pvBuf || enmTxDir == PDMBLOCKTXDIR_NONE);
    Assert(pbSense || !cbSense);
    AssertPtr(pbCmd);
    Assert(cbCmd <= 16 && cbCmd >= 1);
    const uint32_t cbBuf = pcbBuf ? *pcbBuf : 0;
    if (pcbBuf)
        *pcbBuf = 0;

# ifdef RT_OS_DARWIN
    Assert(pThis->ppScsiTaskDI);

    int rc = VERR_GENERAL_FAILURE;
    SCSITaskInterface **ppScsiTaskI = (*pThis->ppScsiTaskDI)->CreateSCSITask(pThis->ppScsiTaskDI);
    if (!ppScsiTaskI)
        return VERR_NO_MEMORY;
    do
    {
        /* Setup the scsi command. */
        SCSICommandDescriptorBlock cdb = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
        memcpy(&cdb[0], pbCmd, cbCmd);
        IOReturn irc = (*ppScsiTaskI)->SetCommandDescriptorBlock(ppScsiTaskI, cdb, cbCmd);
        AssertBreak(irc == kIOReturnSuccess);

        /* Setup the buffer. */
        if (enmTxDir == PDMBLOCKTXDIR_NONE)
            irc = (*ppScsiTaskI)->SetScatterGatherEntries(ppScsiTaskI, NULL, 0, 0, kSCSIDataTransfer_NoDataTransfer);
        else
        {
            IOVirtualRange Range = { (IOVirtualAddress)pvBuf, cbBuf };
            irc = (*ppScsiTaskI)->SetScatterGatherEntries(ppScsiTaskI, &Range, 1, cbBuf,
                                                          enmTxDir == PDMBLOCKTXDIR_FROM_DEVICE
                                                          ? kSCSIDataTransfer_FromTargetToInitiator
                                                          : kSCSIDataTransfer_FromInitiatorToTarget);
        }
        AssertBreak(irc == kIOReturnSuccess);

        /* Set the timeout. */
        irc = (*ppScsiTaskI)->SetTimeoutDuration(ppScsiTaskI, cTimeoutMillies ? cTimeoutMillies : 30000 /*ms*/);
        AssertBreak(irc == kIOReturnSuccess);

        /* Execute the command and get the response. */
        SCSI_Sense_Data SenseData = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
        SCSIServiceResponse     ServiceResponse = kSCSIServiceResponse_Request_In_Process;
        SCSITaskStatus TaskStatus = kSCSITaskStatus_GOOD;
        UInt64 cbReturned = 0;
        irc = (*ppScsiTaskI)->ExecuteTaskSync(ppScsiTaskI, &SenseData, &TaskStatus, &cbReturned);
        AssertBreak(irc == kIOReturnSuccess);
        if (pcbBuf)
            *pcbBuf = (int32_t)cbReturned;

        irc = (*ppScsiTaskI)->GetSCSIServiceResponse(ppScsiTaskI, &ServiceResponse);
        AssertBreak(irc == kIOReturnSuccess);
        AssertBreak(ServiceResponse == kSCSIServiceResponse_TASK_COMPLETE);

        if (TaskStatus == kSCSITaskStatus_GOOD)
            rc = VINF_SUCCESS;
        else if (   TaskStatus == kSCSITaskStatus_CHECK_CONDITION
                 && pbSense)
        {
            memset(pbSense, 0, cbSense); /* lazy */
            memcpy(pbSense, &SenseData, RT_MIN(sizeof(SenseData), cbSense));
            rc = VERR_UNRESOLVED_ERROR;
        }
        /** @todo convert sense codes when caller doesn't wish to do this himself. */
        /*else if (   TaskStatus == kSCSITaskStatus_CHECK_CONDITION
                 && SenseData.ADDITIONAL_SENSE_CODE == 0x3A)
            rc = VERR_MEDIA_NOT_PRESENT; */
        else
        {
            rc = enmTxDir == PDMBLOCKTXDIR_NONE
               ? VERR_DEV_IO_ERROR
               : enmTxDir == PDMBLOCKTXDIR_FROM_DEVICE
               ? VERR_READ_ERROR
               : VERR_WRITE_ERROR;
            if (pThis->cLogRelErrors++ < 10)
                LogRel(("DVD scsi error: cmd={%.*Rhxs} TaskStatus=%#x key=%#x ASC=%#x ASCQ=%#x (%Rrc)\n",
                        cbCmd, pbCmd, TaskStatus, SenseData.SENSE_KEY, SenseData.ADDITIONAL_SENSE_CODE,
                        SenseData.ADDITIONAL_SENSE_CODE_QUALIFIER, rc));
        }
    } while (0);

    (*ppScsiTaskI)->Release(ppScsiTaskI);

# elif defined(RT_OS_FREEBSD)
    int rc = VINF_SUCCESS;
    int rcBSD = 0;
    union ccb DeviceCCB;
    union ccb *pDeviceCCB = &DeviceCCB;
    u_int32_t fFlags;

    memset(pDeviceCCB, 0, sizeof(DeviceCCB));
    pDeviceCCB->ccb_h.path_id   = pThis->ScsiBus;
    pDeviceCCB->ccb_h.target_id = pThis->ScsiTargetID;
    pDeviceCCB->ccb_h.target_lun = pThis->ScsiLunID;

    /* The SCSI INQUIRY command can't be passed through directly. */
    if (pbCmd[0] == SCSI_INQUIRY)
    {
        pDeviceCCB->ccb_h.func_code = XPT_GDEV_TYPE;

        rcBSD = ioctl(RTFileToNative(pThis->hFileDevice), CAMIOCOMMAND, pDeviceCCB);
        if (!rcBSD)
        {
            uint32_t cbCopy =   cbBuf < sizeof(struct scsi_inquiry_data)
                              ? cbBuf
                              : sizeof(struct scsi_inquiry_data);;
            memcpy(pvBuf, &pDeviceCCB->cgd.inq_data, cbCopy);
            memset(pbSense, 0, cbSense);

            if (pcbBuf)
                *pcbBuf = cbCopy;
        }
        else
            rc = RTErrConvertFromErrno(errno);
    }
    else
    {
        /* Copy the CDB. */
        memcpy(&pDeviceCCB->csio.cdb_io.cdb_bytes, pbCmd, cbCmd);

        /* Set direction. */
        if (enmTxDir == PDMBLOCKTXDIR_NONE)
            fFlags = CAM_DIR_NONE;
        else if (enmTxDir == PDMBLOCKTXDIR_FROM_DEVICE)
            fFlags = CAM_DIR_IN;
        else
            fFlags = CAM_DIR_OUT;

        fFlags |= CAM_DEV_QFRZDIS;

        cam_fill_csio(&pDeviceCCB->csio, 1, NULL, fFlags, MSG_SIMPLE_Q_TAG,
                      (u_int8_t *)pvBuf, cbBuf, cbSense, cbCmd,
                      cTimeoutMillies ? cTimeoutMillies : 30000/* timeout */);

        /* Send command */
        rcBSD = ioctl(RTFileToNative(pThis->hFileDevice), CAMIOCOMMAND, pDeviceCCB);
        if (!rcBSD)
        {
            switch (pDeviceCCB->ccb_h.status & CAM_STATUS_MASK)
            {
                case CAM_REQ_CMP:
                    rc = VINF_SUCCESS;
                    break;
                case CAM_SEL_TIMEOUT:
                    rc = VERR_DEV_IO_ERROR;
                    break;
                case CAM_CMD_TIMEOUT:
                    rc = VERR_TIMEOUT;
                    break;
                default:
                    rc = VERR_DEV_IO_ERROR;
            }

            if (pcbBuf)
                *pcbBuf = cbBuf - pDeviceCCB->csio.resid;

            if (pbSense)
                memcpy(pbSense, &pDeviceCCB->csio.sense_data,
                       cbSense - pDeviceCCB->csio.sense_resid);
        }
        else
            rc = RTErrConvertFromErrno(errno);
    }
# endif

    return rc;
}
#endif


/**
 * Media present.
 * Query the size and notify the above driver / device.
 *
 * @param   pThis   The instance data.
 */
int DRVHostBaseMediaPresent(PDRVHOSTBASE pThis)
{
    /*
     * Open the drive.
     */
    int rc = drvHostBaseReopen(pThis);
    if (RT_FAILURE(rc))
        return rc;

    /*
     * Determine the size.
     */
    uint64_t cb;
    rc = pThis->pfnGetMediaSize(pThis, &cb);
    if (RT_FAILURE(rc))
    {
        LogFlow(("%s-%d: failed to figure media size of %s, rc=%Rrc\n",
                 pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, pThis->pszDevice, rc));
        return rc;
    }

    /*
     * Update the data and inform the unit.
     */
    pThis->cbSize = cb;
    pThis->fMediaPresent = true;
    if (pThis->pDrvMountNotify)
        pThis->pDrvMountNotify->pfnMountNotify(pThis->pDrvMountNotify);
    LogFlow(("%s-%d: drvHostBaseMediaPresent: cbSize=%lld (%#llx)\n",
             pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, pThis->cbSize, pThis->cbSize));
    return VINF_SUCCESS;
}


/**
 * Media no longer present.
 * @param   pThis   The instance data.
 */
void DRVHostBaseMediaNotPresent(PDRVHOSTBASE pThis)
{
    pThis->fMediaPresent = false;
    pThis->fLocked = false;
    pThis->PCHSGeometry.cCylinders = 0;
    pThis->PCHSGeometry.cHeads = 0;
    pThis->PCHSGeometry.cSectors = 0;
    pThis->LCHSGeometry.cCylinders = 0;
    pThis->LCHSGeometry.cHeads = 0;
    pThis->LCHSGeometry.cSectors = 0;
    if (pThis->pDrvMountNotify)
        pThis->pDrvMountNotify->pfnUnmountNotify(pThis->pDrvMountNotify);
}


#ifdef RT_OS_WINDOWS

/**
 * Window procedure for the invisible window used to catch the WM_DEVICECHANGE broadcasts.
 */
static LRESULT CALLBACK DeviceChangeWindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    Log2(("DeviceChangeWindowProc: hwnd=%08x uMsg=%08x\n", hwnd, uMsg));
    if (uMsg == WM_DESTROY)
    {
        PDRVHOSTBASE pThis = (PDRVHOSTBASE)GetWindowLong(hwnd, GWLP_USERDATA);
        if (pThis)
            ASMAtomicXchgSize(&pThis->hwndDeviceChange, NULL);
        PostQuitMessage(0);
    }

    if (uMsg != WM_DEVICECHANGE)
        return DefWindowProc(hwnd, uMsg, wParam, lParam);

    PDEV_BROADCAST_HDR  lpdb = (PDEV_BROADCAST_HDR)lParam;
    PDRVHOSTBASE        pThis = (PDRVHOSTBASE)GetWindowLongPtr(hwnd, GWLP_USERDATA);
    Assert(pThis);
    if (pThis == NULL)
        return 0;

    switch (wParam)
    {
        case DBT_DEVICEARRIVAL:
        case DBT_DEVICEREMOVECOMPLETE:
            // Check whether a CD or DVD was inserted into or removed from a drive.
            if (lpdb->dbch_devicetype == DBT_DEVTYP_VOLUME)
            {
                PDEV_BROADCAST_VOLUME lpdbv = (PDEV_BROADCAST_VOLUME)lpdb;
                if (    (lpdbv->dbcv_flags & DBTF_MEDIA)
                    &&  (pThis->fUnitMask & lpdbv->dbcv_unitmask))
                {
                    RTCritSectEnter(&pThis->CritSect);
                    if (wParam == DBT_DEVICEARRIVAL)
                    {
                        int cRetries = 10;
                        int rc = DRVHostBaseMediaPresent(pThis);
                        while (RT_FAILURE(rc) && cRetries-- > 0)
                        {
                            RTThreadSleep(50);
                            rc = DRVHostBaseMediaPresent(pThis);
                        }
                    }
                    else
                        DRVHostBaseMediaNotPresent(pThis);
                    RTCritSectLeave(&pThis->CritSect);
                }
            }
            break;
    }
    return TRUE;
}

#endif /* RT_OS_WINDOWS */


/**
 * This thread will periodically poll the device for media presence.
 *
 * @returns Ignored.
 * @param   ThreadSelf  Handle of this thread. Ignored.
 * @param   pvUser      Pointer to the driver instance structure.
 */
static DECLCALLBACK(int) drvHostBaseMediaThread(RTTHREAD ThreadSelf, void *pvUser)
{
    PDRVHOSTBASE pThis = (PDRVHOSTBASE)pvUser;
    LogFlow(("%s-%d: drvHostBaseMediaThread: ThreadSelf=%p pvUser=%p\n",
             pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, ThreadSelf, pvUser));
#ifdef RT_OS_WINDOWS
    static WNDCLASS s_classDeviceChange = {0};
    static ATOM     s_hAtomDeviceChange = 0;

    /*
     * Register custom window class.
     */
    if (s_hAtomDeviceChange == 0)
    {
        memset(&s_classDeviceChange, 0, sizeof(s_classDeviceChange));
        s_classDeviceChange.lpfnWndProc   = DeviceChangeWindowProc;
        s_classDeviceChange.lpszClassName = "VBOX_DeviceChangeClass";
        s_classDeviceChange.hInstance     = GetModuleHandle("VBOXDD.DLL");
        Assert(s_classDeviceChange.hInstance);
        s_hAtomDeviceChange = RegisterClassA(&s_classDeviceChange);
        Assert(s_hAtomDeviceChange);
    }

    /*
     * Create Window w/ the pThis as user data.
     */
    HWND hwnd = CreateWindow((LPCTSTR)s_hAtomDeviceChange, "", WS_POPUP, 0, 0, 0, 0, 0, 0, s_classDeviceChange.hInstance, 0);
    AssertMsg(hwnd, ("CreateWindow failed with %d\n", GetLastError()));
    SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)pThis);

    /*
     * Signal the waiting EMT thread that everything went fine.
     */
    ASMAtomicXchgSize(&pThis->hwndDeviceChange, hwnd);
    RTThreadUserSignal(ThreadSelf);
    if (!hwnd)
    {
        LogFlow(("%s-%d: drvHostBaseMediaThread: returns VERR_GENERAL_FAILURE\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance));
        return VERR_GENERAL_FAILURE;
    }
    LogFlow(("%s-%d: drvHostBaseMediaThread: Created hwndDeviceChange=%p\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance, hwnd));

    /*
     * Message pump.
     */
    MSG         Msg;
    BOOL        fRet;
    while ((fRet = GetMessage(&Msg, NULL, 0, 0)) != FALSE)
    {
        if (fRet != -1)
        {
            TranslateMessage(&Msg);
            DispatchMessage(&Msg);
        }
        //else: handle the error and possibly exit
    }
    Assert(!pThis->hwndDeviceChange);

#else /* !RT_OS_WINDOWS */
    bool        fFirst = true;
    int         cRetries = 10;
    while (!pThis->fShutdownPoller)
    {
        /*
         * Perform the polling (unless we've run out of 50ms retries).
         */
        if (    pThis->pfnPoll
            &&  cRetries-- > 0)
        {

            int rc = pThis->pfnPoll(pThis);
            if (RT_FAILURE(rc))
            {
                RTSemEventWait(pThis->EventPoller, 50);
                continue;
            }
        }

        /*
         * Signal EMT after the first go.
         */
        if (fFirst)
        {
            RTThreadUserSignal(ThreadSelf);
            fFirst = false;
        }

        /*
         * Sleep.
         */
        int rc = RTSemEventWait(pThis->EventPoller, pThis->cMilliesPoller);
        if (    RT_FAILURE(rc)
            &&  rc != VERR_TIMEOUT)
        {
            AssertMsgFailed(("rc=%Rrc\n", rc));
            pThis->ThreadPoller = NIL_RTTHREAD;
            LogFlow(("drvHostBaseMediaThread: returns %Rrc\n", rc));
            return rc;
        }
        cRetries = 10;
    }

#endif /* !RT_OS_WINDOWS */

    /* (Don't clear the thread handle here, the destructor thread is using it to wait.) */
    LogFlow(("%s-%d: drvHostBaseMediaThread: returns VINF_SUCCESS\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance));
    return VINF_SUCCESS;
}

/* -=-=-=-=- driver interface -=-=-=-=- */


/**
 * Done state load operation.
 *
 * @returns VBox load code.
 * @param   pDrvIns         Driver instance of the driver which registered the data unit.
 * @param   pSSM            SSM operation handle.
 */
static DECLCALLBACK(int) drvHostBaseLoadDone(PPDMDRVINS pDrvIns, PSSMHANDLE pSSM)
{
    PDRVHOSTBASE pThis = PDMINS_2_DATA(pDrvIns, PDRVHOSTBASE);
    LogFlow(("%s-%d: drvHostBaseMediaThread:\n", pThis->pDrvIns->pReg->szName, pThis->pDrvIns->iInstance));
    RTCritSectEnter(&pThis->CritSect);

    /*
     * Tell the device/driver above us that the media status is uncertain.
     */
    if (pThis->pDrvMountNotify)
    {
        pThis->pDrvMountNotify->pfnUnmountNotify(pThis->pDrvMountNotify);
        if (pThis->fMediaPresent)
            pThis->pDrvMountNotify->pfnMountNotify(pThis->pDrvMountNotify);
    }

    RTCritSectLeave(&pThis->CritSect);
    return VINF_SUCCESS;
}


/** @copydoc FNPDMDRVDESTRUCT */
DECLCALLBACK(void) DRVHostBaseDestruct(PPDMDRVINS pDrvIns)
{
    PDRVHOSTBASE pThis = PDMINS_2_DATA(pDrvIns, PDRVHOSTBASE);
    LogFlow(("%s-%d: drvHostBaseDestruct: iInstance=%d\n", pDrvIns->pReg->szName, pDrvIns->iInstance, pDrvIns->iInstance));

    /*
     * Terminate the thread.
     */
    if (pThis->ThreadPoller != NIL_RTTHREAD)
    {
        pThis->fShutdownPoller = true;
        int rc;
        int cTimes = 50;
        do
        {
#ifdef RT_OS_WINDOWS
            if (pThis->hwndDeviceChange)
                PostMessage(pThis->hwndDeviceChange, WM_CLOSE, 0, 0); /* default win proc will destroy the window */
#else
            RTSemEventSignal(pThis->EventPoller);
#endif
            rc = RTThreadWait(pThis->ThreadPoller, 100, NULL);
        } while (cTimes-- > 0 && rc == VERR_TIMEOUT);

        if (!rc)
            pThis->ThreadPoller = NIL_RTTHREAD;
    }

    /*
     * Unlock the drive if we've locked it or we're in passthru mode.
     */
#ifdef RT_OS_DARWIN
    if (    (   pThis->fLocked
             || pThis->IBlock.pfnSendCmd)
        &&  pThis->ppScsiTaskDI
#else /** @todo Check if the other guys can mix pfnDoLock with scsi passthru.
       * (We're currently not unlocking the device after use. See todo in DevATA.cpp.) */
    if (    pThis->fLocked
        &&  pThis->hFileDevice != NIL_RTFILE
#endif
        &&  pThis->pfnDoLock)
    {
        int rc = pThis->pfnDoLock(pThis, false);
        if (RT_SUCCESS(rc))
            pThis->fLocked = false;
    }

    /*
     * Cleanup the other resources.
     */
#ifdef RT_OS_WINDOWS
    if (pThis->hwndDeviceChange)
    {
        if (SetWindowLongPtr(pThis->hwndDeviceChange, GWLP_USERDATA, 0) == (LONG_PTR)pThis)
            PostMessage(pThis->hwndDeviceChange, WM_CLOSE, 0, 0); /* default win proc will destroy the window */
        pThis->hwndDeviceChange = NULL;
    }
#else
    if (pThis->EventPoller != NULL)
    {
        RTSemEventDestroy(pThis->EventPoller);
        pThis->EventPoller = NULL;
    }
#endif

#ifdef RT_OS_DARWIN
    /*
     * The unclaiming doesn't seem to mean much, the DVD is actually
     * remounted when we release exclusive access. I'm not quite sure
     * if I should put the unclaim first or not...
     *
     * Anyway, that it's automatically remounted very good news for us,
     * because that means we don't have to mess with that ourselves. Of
     * course there is the unlikely scenario that we've succeeded in claiming
     * and umount the DVD but somehow failed to gain exclusive scsi access...
     */
    if (pThis->ppScsiTaskDI)
    {
        LogFlow(("%s-%d: releasing exclusive scsi access!\n", pDrvIns->pReg->szName, pDrvIns->iInstance));
        (*pThis->ppScsiTaskDI)->ReleaseExclusiveAccess(pThis->ppScsiTaskDI);
        (*pThis->ppScsiTaskDI)->Release(pThis->ppScsiTaskDI);
        pThis->ppScsiTaskDI = NULL;
    }
    if (pThis->pDADisk)
    {
        LogFlow(("%s-%d: unclaiming the disk!\n", pDrvIns->pReg->szName, pDrvIns->iInstance));
        DADiskUnclaim(pThis->pDADisk);
        CFRelease(pThis->pDADisk);
        pThis->pDADisk = NULL;
    }
    if (pThis->ppMMCDI)
    {
        LogFlow(("%s-%d: releasing the MMC object!\n", pDrvIns->pReg->szName, pDrvIns->iInstance));
        (*pThis->ppMMCDI)->Release(pThis->ppMMCDI);
        pThis->ppMMCDI = NULL;
    }
    if (pThis->MasterPort)
    {
        mach_port_deallocate(mach_task_self(), pThis->MasterPort);
        pThis->MasterPort = NULL;
    }
    if (pThis->pDASession)
    {
        LogFlow(("%s-%d: releasing the DA session!\n", pDrvIns->pReg->szName, pDrvIns->iInstance));
        CFRelease(pThis->pDASession);
        pThis->pDASession = NULL;
    }
#else
    if (pThis->hFileDevice != NIL_RTFILE)
    {
        int rc = RTFileClose(pThis->hFileDevice);
        AssertRC(rc);
        pThis->hFileDevice = NIL_RTFILE;
    }
#endif

#ifdef RT_OS_SOLARIS
    if (pThis->hFileRawDevice != NIL_RTFILE)
    {
        int rc = RTFileClose(pThis->hFileRawDevice);
        AssertRC(rc);
        pThis->hFileRawDevice = NIL_RTFILE;
    }

    if (pThis->pszRawDeviceOpen)
    {
        RTStrFree(pThis->pszRawDeviceOpen);
        pThis->pszRawDeviceOpen = NULL;
    }
#endif

    if (pThis->pszDevice)
    {
        MMR3HeapFree(pThis->pszDevice);
        pThis->pszDevice = NULL;
    }

    if (pThis->pszDeviceOpen)
    {
        RTStrFree(pThis->pszDeviceOpen);
        pThis->pszDeviceOpen = NULL;
    }

    /* Forget about the notifications. */
    pThis->pDrvMountNotify = NULL;

    /* Leave the instance operational if this is just a cleanup of the state
     * after an attach error happened. So don't destroy the critsect then. */
    if (!pThis->fKeepInstance && RTCritSectIsInitialized(&pThis->CritSect))
        RTCritSectDelete(&pThis->CritSect);
    LogFlow(("%s-%d: drvHostBaseDestruct completed\n", pDrvIns->pReg->szName, pDrvIns->iInstance));
}


/**
 * Initializes the instance data (init part 1).
 *
 * The driver which derives from this base driver will override function pointers after
 * calling this method, and complete the construction by calling DRVHostBaseInitFinish().
 *
 * On failure call DRVHostBaseDestruct().
 *
 * @returns VBox status code.
 * @param   pDrvIns         Driver instance.
 * @param   pCfg            Configuration handle.
 * @param   enmType         Device type.
 */
int DRVHostBaseInitData(PPDMDRVINS pDrvIns, PCFGMNODE pCfg, PDMBLOCKTYPE enmType)
{
    PDRVHOSTBASE pThis = PDMINS_2_DATA(pDrvIns, PDRVHOSTBASE);
    LogFlow(("%s-%d: DRVHostBaseInitData: iInstance=%d\n", pDrvIns->pReg->szName, pDrvIns->iInstance, pDrvIns->iInstance));

    /*
     * Initialize most of the data members.
     */
    pThis->pDrvIns                          = pDrvIns;
    pThis->fKeepInstance                    = false;
    pThis->ThreadPoller                     = NIL_RTTHREAD;
#ifdef RT_OS_DARWIN
    pThis->MasterPort                       = NULL;
    pThis->ppMMCDI                          = NULL;
    pThis->ppScsiTaskDI                     = NULL;
    pThis->cbBlock                          = 0;
    pThis->pDADisk                          = NULL;
    pThis->pDASession                       = NULL;
#else
    pThis->hFileDevice                      = NIL_RTFILE;
#endif
#ifdef RT_OS_SOLARIS
    pThis->hFileRawDevice                   = NIL_RTFILE;
#endif
    pThis->enmType                          = enmType;
    //pThis->cErrors                          = 0;

    pThis->pfnGetMediaSize                  = drvHostBaseGetMediaSize;

    /* IBase. */
    pDrvIns->IBase.pfnQueryInterface        = drvHostBaseQueryInterface;

    /* IBlock. */
    pThis->IBlock.pfnRead                   = drvHostBaseRead;
    pThis->IBlock.pfnWrite                  = drvHostBaseWrite;
    pThis->IBlock.pfnFlush                  = drvHostBaseFlush;
    pThis->IBlock.pfnIsReadOnly             = drvHostBaseIsReadOnly;
    pThis->IBlock.pfnGetSize                = drvHostBaseGetSize;
    pThis->IBlock.pfnGetType                = drvHostBaseGetType;
    pThis->IBlock.pfnGetUuid                = drvHostBaseGetUuid;

    /* IBlockBios. */
    pThis->IBlockBios.pfnGetPCHSGeometry    = drvHostBaseGetPCHSGeometry;
    pThis->IBlockBios.pfnSetPCHSGeometry    = drvHostBaseSetPCHSGeometry;
    pThis->IBlockBios.pfnGetLCHSGeometry    = drvHostBaseGetLCHSGeometry;
    pThis->IBlockBios.pfnSetLCHSGeometry    = drvHostBaseSetLCHSGeometry;
    pThis->IBlockBios.pfnIsVisible          = drvHostBaseIsVisible;
    pThis->IBlockBios.pfnGetType            = drvHostBaseBiosGetType;

    /* IMount. */
    pThis->IMount.pfnMount                  = drvHostBaseMount;
    pThis->IMount.pfnUnmount                = drvHostBaseUnmount;
    pThis->IMount.pfnIsMounted              = drvHostBaseIsMounted;
    pThis->IMount.pfnLock                   = drvHostBaseLock;
    pThis->IMount.pfnUnlock                 = drvHostBaseUnlock;
    pThis->IMount.pfnIsLocked               = drvHostBaseIsLocked;

    /*
     * Get the IBlockPort & IMountNotify interfaces of the above driver/device.
     */
    pThis->pDrvBlockPort = PDMIBASE_QUERY_INTERFACE(pDrvIns->pUpBase, PDMIBLOCKPORT);
    if (!pThis->pDrvBlockPort)
    {
        AssertMsgFailed(("Configuration error: No block port interface above!\n"));
        return VERR_PDM_MISSING_INTERFACE_ABOVE;
    }
    pThis->pDrvMountNotify = PDMIBASE_QUERY_INTERFACE(pDrvIns->pUpBase, PDMIMOUNTNOTIFY);

    /*
     * Query configuration.
     */
    /* Device */
    int rc = CFGMR3QueryStringAlloc(pCfg, "Path", &pThis->pszDevice);
    if (RT_FAILURE(rc))
    {
        AssertMsgFailed(("Configuration error: query for \"Path\" string returned %Rra.\n", rc));
        return rc;
    }

    /* Mountable */
    uint32_t u32;
    rc = CFGMR3QueryU32(pCfg, "Interval", &u32);
    if (RT_SUCCESS(rc))
        pThis->cMilliesPoller = u32;
    else if (rc == VERR_CFGM_VALUE_NOT_FOUND)
        pThis->cMilliesPoller = 1000;
    else if (RT_FAILURE(rc))
    {
        AssertMsgFailed(("Configuration error: Query \"Mountable\" resulted in %Rrc.\n", rc));
        return rc;
    }

    /* ReadOnly */
    rc = CFGMR3QueryBool(pCfg, "ReadOnly", &pThis->fReadOnlyConfig);
    if (rc == VERR_CFGM_VALUE_NOT_FOUND)
        pThis->fReadOnlyConfig = enmType == PDMBLOCKTYPE_DVD || enmType == PDMBLOCKTYPE_CDROM ? true : false;
    else if (RT_FAILURE(rc))
    {
        AssertMsgFailed(("Configuration error: Query \"ReadOnly\" resulted in %Rrc.\n", rc));
        return rc;
    }

    /* Locked */
    rc = CFGMR3QueryBool(pCfg, "Locked", &pThis->fLocked);
    if (rc == VERR_CFGM_VALUE_NOT_FOUND)
        pThis->fLocked = false;
    else if (RT_FAILURE(rc))
    {
        AssertMsgFailed(("Configuration error: Query \"Locked\" resulted in %Rrc.\n", rc));
        return rc;
    }

    /* BIOS visible */
    rc = CFGMR3QueryBool(pCfg, "BIOSVisible", &pThis->fBiosVisible);
    if (rc == VERR_CFGM_VALUE_NOT_FOUND)
        pThis->fBiosVisible = true;
    else if (RT_FAILURE(rc))
    {
        AssertMsgFailed(("Configuration error: Query \"BIOSVisible\" resulted in %Rrc.\n", rc));
        return rc;
    }

    /* Uuid */
    char *psz;
    rc = CFGMR3QueryStringAlloc(pCfg, "Uuid", &psz);
    if (rc == VERR_CFGM_VALUE_NOT_FOUND)
        RTUuidClear(&pThis->Uuid);
    else if (RT_SUCCESS(rc))
    {
        rc = RTUuidFromStr(&pThis->Uuid, psz);
        if (RT_FAILURE(rc))
        {
            AssertMsgFailed(("Configuration error: Uuid from string failed on \"%s\", rc=%Rrc.\n", psz, rc));
            MMR3HeapFree(psz);
            return rc;
        }
        MMR3HeapFree(psz);
    }
    else
    {
        AssertMsgFailed(("Configuration error: Failed to obtain the uuid, rc=%Rrc.\n", rc));
        return rc;
    }

    /* Define whether attach failure is an error (default) or not. */
    bool fAttachFailError;
    rc = CFGMR3QueryBool(pCfg, "AttachFailError", &fAttachFailError);
    if (RT_FAILURE(rc))
        fAttachFailError = true;
    pThis->fAttachFailError = fAttachFailError;

    /* name to open & watch for */
#ifdef RT_OS_WINDOWS
    int iBit = RT_C_TO_UPPER(pThis->pszDevice[0]) - 'A';
    if (    iBit > 'Z' - 'A'
        ||  pThis->pszDevice[1] != ':'
        ||  pThis->pszDevice[2])
    {
        AssertMsgFailed(("Configuration error: Invalid drive specification: '%s'\n", pThis->pszDevice));
        return VERR_INVALID_PARAMETER;
    }
    pThis->fUnitMask = 1 << iBit;
    RTStrAPrintf(&pThis->pszDeviceOpen, "\\\\.\\%s", pThis->pszDevice);

#elif defined(RT_OS_SOLARIS)
    char *pszBlockDevName = getfullblkname(pThis->pszDevice);
    if (!pszBlockDevName)
        return VERR_NO_MEMORY;
    pThis->pszDeviceOpen = RTStrDup(pszBlockDevName);  /* for RTStrFree() */
    free(pszBlockDevName);
    pThis->pszRawDeviceOpen = RTStrDup(pThis->pszDevice);

#else
    pThis->pszDeviceOpen = RTStrDup(pThis->pszDevice);
#endif

    if (!pThis->pszDeviceOpen)
        return VERR_NO_MEMORY;

    return VINF_SUCCESS;
}


/**
 * Do the 2nd part of the init after the derived driver has overridden the defaults.
 *
 * On failure call DRVHostBaseDestruct().
 *
 * @returns VBox status code.
 * @param   pThis       Pointer to the instance data.
 */
int DRVHostBaseInitFinish(PDRVHOSTBASE pThis)
{
    int src = VINF_SUCCESS;
    PPDMDRVINS pDrvIns = pThis->pDrvIns;

    /* log config summary */
    Log(("%s-%d: pszDevice='%s' (%s) cMilliesPoller=%d fReadOnlyConfig=%d fLocked=%d fBIOSVisible=%d Uuid=%RTuuid\n",
         pDrvIns->pReg->szName, pDrvIns->iInstance, pThis->pszDevice, pThis->pszDeviceOpen, pThis->cMilliesPoller,
         pThis->fReadOnlyConfig, pThis->fLocked, pThis->fBiosVisible, &pThis->Uuid));

    /*
     * Check that there are no drivers below us.
     */
    AssertMsgReturn(PDMDrvHlpNoAttach(pDrvIns) == VERR_PDM_NO_ATTACHED_DRIVER,
                    ("Configuration error: Not possible to attach anything to this driver!\n"),
                    VERR_PDM_DRVINS_NO_ATTACH);

    /*
     * Register saved state.
     */
    int rc = PDMDrvHlpSSMRegisterLoadDone(pDrvIns, drvHostBaseLoadDone);
    if (RT_FAILURE(rc))
        return rc;

    /*
     * Verify type.
     */
#ifdef RT_OS_WINDOWS
    UINT uDriveType = GetDriveType(pThis->pszDevice);
    switch (pThis->enmType)
    {
        case PDMBLOCKTYPE_FLOPPY_360:
        case PDMBLOCKTYPE_FLOPPY_720:
        case PDMBLOCKTYPE_FLOPPY_1_20:
        case PDMBLOCKTYPE_FLOPPY_1_44:
        case PDMBLOCKTYPE_FLOPPY_2_88:
            if (uDriveType != DRIVE_REMOVABLE)
            {
                AssertMsgFailed(("Configuration error: '%s' is not a floppy (type=%d)\n",
                                 pThis->pszDevice, uDriveType));
                return VERR_INVALID_PARAMETER;
            }
            break;
        case PDMBLOCKTYPE_CDROM:
        case PDMBLOCKTYPE_DVD:
            if (uDriveType != DRIVE_CDROM)
            {
                AssertMsgFailed(("Configuration error: '%s' is not a cdrom (type=%d)\n",
                                 pThis->pszDevice, uDriveType));
                return VERR_INVALID_PARAMETER;
            }
            break;
        case PDMBLOCKTYPE_HARD_DISK:
        default:
            AssertMsgFailed(("enmType=%d\n", pThis->enmType));
            return VERR_INVALID_PARAMETER;
    }
#endif

    /*
     * Open the device.
     */
#if defined(RT_OS_DARWIN)
    rc = drvHostBaseOpen(pThis, NULL, pThis->fReadOnlyConfig);
#else
    rc = drvHostBaseReopen(pThis);
#endif
    if (RT_FAILURE(rc))
    {
        char *pszDevice = pThis->pszDevice;
#ifndef RT_OS_DARWIN
        char szPathReal[256];
        if (   RTPathExists(pszDevice)
            && RT_SUCCESS(RTPathReal(pszDevice, szPathReal, sizeof(szPathReal))))
            pszDevice = szPathReal;
        pThis->hFileDevice = NIL_RTFILE;
#endif
#ifdef RT_OS_SOLARIS
        pThis->hFileRawDevice = NIL_RTFILE;
#endif

        /*
         * Disable CD/DVD passthrough in case it was enabled. Would cause
         * weird failures later when the guest issues commands. These would
         * all fail because of the invalid file handle. So use the normal
         * virtual CD/DVD code, which deals more gracefully with unavailable
         * "media" - actually a complete drive in this case.
         */
        pThis->IBlock.pfnSendCmd = NULL;
        AssertMsgFailed(("Could not open host device %s, rc=%Rrc\n", pszDevice, rc));
        switch (rc)
        {
            case VERR_ACCESS_DENIED:
                return PDMDrvHlpVMSetError(pDrvIns, rc, RT_SRC_POS,
#ifdef RT_OS_LINUX
                        N_("Cannot open host device '%s' for %s access. Check the permissions "
                           "of that device ('/bin/ls -l %s'): Most probably you need to be member "
                           "of the device group. Make sure that you logout/login after changing "
                           "the group settings of the current user"),
#else
                        N_("Cannot open host device '%s' for %s access. Check the permissions "
                           "of that device"),
#endif
                       pszDevice, pThis->fReadOnlyConfig ? "readonly" : "read/write",
                       pszDevice);
            default:
            {
                if (pThis->fAttachFailError)
                    return rc;
                int erc = PDMDrvHlpVMSetRuntimeError(pDrvIns, 0 /*fFlags*/,
                                                     "DrvHost_MOUNTFAIL",
                                                     N_("Cannot attach to host device '%s'"), pszDevice);
                AssertRC(erc);
                src = rc;
            }
        }
    }
#ifdef RT_OS_WINDOWS
    if (RT_SUCCESS(src))
        DRVHostBaseMediaPresent(pThis);
#endif

    /*
     * Lock the drive if that's required by the configuration.
     */
    if (pThis->fLocked)
    {
        if (pThis->pfnDoLock)
            rc = pThis->pfnDoLock(pThis, true);
        if (RT_FAILURE(rc))
        {
            AssertMsgFailed(("Failed to lock the dvd drive. rc=%Rrc\n", rc));
            return rc;
        }
    }

#ifndef RT_OS_WINDOWS
    if (RT_SUCCESS(src))
    {
        /*
         * Create the event semaphore which the poller thread will wait on.
         */
        rc = RTSemEventCreate(&pThis->EventPoller);
        if (RT_FAILURE(rc))
            return rc;
    }
#endif

    /*
     * Initialize the critical section used for serializing the access to the media.
     */
    rc = RTCritSectInit(&pThis->CritSect);
    if (RT_FAILURE(rc))
        return rc;

    if (RT_SUCCESS(src))
    {
        /*
         * Start the thread which will poll for the media.
         */
        rc = RTThreadCreate(&pThis->ThreadPoller, drvHostBaseMediaThread, pThis, 0,
                            RTTHREADTYPE_INFREQUENT_POLLER, RTTHREADFLAGS_WAITABLE, "DVDMEDIA");
        if (RT_FAILURE(rc))
        {
            AssertMsgFailed(("Failed to create poller thread. rc=%Rrc\n", rc));
            return rc;
        }

        /*
         * Wait for the thread to start up (!w32:) and do one detection loop.
         */
        rc = RTThreadUserWait(pThis->ThreadPoller, 10000);
        AssertRC(rc);
#ifdef RT_OS_WINDOWS
        if (!pThis->hwndDeviceChange)
            return VERR_GENERAL_FAILURE;
#endif
    }

    if (RT_FAILURE(src))
        return src;
    return rc;
}

