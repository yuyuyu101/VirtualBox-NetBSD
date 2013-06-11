/** @file
 * VD Container API - internal interfaces.
 */

/*
 * Copyright (C) 2011 Oracle Corporation
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

#ifndef ___VBox_VD_Interfaces_Internal_h
#define ___VBox_VD_Interfaces_Internal_h

#include <iprt/sg.h>
#include <VBox/vd-ifs.h>

RT_C_DECLS_BEGIN

/**
 * Interface to get the parent state.
 *
 * Per-operation interface. Optional, present only if there is a parent, and
 * used only internally for compacting.
 */
typedef struct VDINTERFACEPARENTSTATE
{
    /**
     * Common interface header.
     */
    VDINTERFACE    Core;

    /**
     * Read data callback.
     *
     * @return  VBox status code.
     * @return  VERR_VD_NOT_OPENED if no image is opened in HDD container.
     * @param   pvUser          The opaque data passed for the operation.
     * @param   uOffset         Offset of first reading byte from start of disk.
     *                          Must be aligned to a sector boundary.
     * @param   pvBuffer        Pointer to buffer for reading data.
     * @param   cbBuffer        Number of bytes to read.
     *                          Must be aligned to a sector boundary.
     */
    DECLR3CALLBACKMEMBER(int, pfnParentRead, (void *pvUser, uint64_t uOffset, void *pvBuffer, size_t cbBuffer));

} VDINTERFACEPARENTSTATE, *PVDINTERFACEPARENTSTATE;


/**
 * Get parent state interface from interface list.
 *
 * @return Pointer to the first parent state interface in the list.
 * @param  pVDIfs    Pointer to the interface list.
 */
DECLINLINE(PVDINTERFACEPARENTSTATE) VDIfParentStateGet(PVDINTERFACE pVDIfs)
{
    PVDINTERFACE pIf = VDInterfaceGet(pVDIfs, VDINTERFACETYPE_PARENTSTATE);

    /* Check that the interface descriptor is a progress interface. */
    AssertMsgReturn(   !pIf
                    || (   (pIf->enmInterface == VDINTERFACETYPE_PARENTSTATE)
                        && (pIf->cbSize == sizeof(VDINTERFACEPARENTSTATE))),
                    ("Not a parent state interface"), NULL);

    return (PVDINTERFACEPARENTSTATE)pIf;
}

/** Forward declaration. Only visible in the VBoxHDD module. */
/** I/O context */
typedef struct VDIOCTX *PVDIOCTX;
/** Storage backend handle. */
typedef struct VDIOSTORAGE *PVDIOSTORAGE;
/** Pointer to a storage backend handle. */
typedef PVDIOSTORAGE *PPVDIOSTORAGE;

/**
 * Completion callback for meta/userdata reads or writes.
 *
 * @return  VBox status code.
 *          VINF_SUCCESS if everything was successful and the transfer can continue.
 *          VERR_VD_ASYNC_IO_IN_PROGRESS if there is another data transfer pending.
 * @param   pBackendData    The opaque backend data.
 * @param   pIoCtx          I/O context associated with this request.
 * @param   pvUser          Opaque user data passed during a read/write request.
 * @param   rcReq           Status code for the completed request.
 */
typedef DECLCALLBACK(int) FNVDXFERCOMPLETED(void *pBackendData, PVDIOCTX pIoCtx, void *pvUser, int rcReq);
/** Pointer to FNVDXFERCOMPLETED() */
typedef FNVDXFERCOMPLETED *PFNVDXFERCOMPLETED;

/** Metadata transfer handle. */
typedef struct VDMETAXFER *PVDMETAXFER;
/** Pointer to a metadata transfer handle. */
typedef PVDMETAXFER *PPVDMETAXFER;


/**
 * Internal I/O interface between the generic VD layer and the backends.
 *
 * Per-image. Always passed to backends.
 */
typedef struct VDINTERFACEIOINT
{
    /**
     * Common interface header.
     */
    VDINTERFACE    Core;

    /**
     * Open callback
     *
     * @return  VBox status code.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pszLocation     Name of the location to open.
     * @param   fOpen           Flags for opening the backend.
     *                          See RTFILE_O_* #defines, inventing another set
     *                          of open flags is not worth the mapping effort.
     * @param   ppStorage       Where to store the storage handle.
     */
    DECLR3CALLBACKMEMBER(int, pfnOpen, (void *pvUser, const char *pszLocation,
                                        uint32_t fOpen, PPVDIOSTORAGE ppStorage));

    /**
     * Close callback.
     *
     * @return  VBox status code.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pStorage        The storage handle to close.
     */
    DECLR3CALLBACKMEMBER(int, pfnClose, (void *pvUser, PVDIOSTORAGE pStorage));

    /**
     * Delete callback.
     *
     * @return  VBox status code.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pcszFilename    Name of the file to delete.
     */
    DECLR3CALLBACKMEMBER(int, pfnDelete, (void *pvUser, const char *pcszFilename));

    /**
     * Move callback.
     *
     * @return  VBox status code.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pcszSrc         The path to the source file.
     * @param   pcszDst         The path to the destination file.
     *                          This file will be created.
     * @param   fMove           A combination of the RTFILEMOVE_* flags.
     */
    DECLR3CALLBACKMEMBER(int, pfnMove, (void *pvUser, const char *pcszSrc, const char *pcszDst, unsigned fMove));

    /**
     * Returns the free space on a disk.
     *
     * @return  VBox status code.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pcszFilename    Name of a file to identify the disk.
     * @param   pcbFreeSpace    Where to store the free space of the disk.
     */
    DECLR3CALLBACKMEMBER(int, pfnGetFreeSpace, (void *pvUser, const char *pcszFilename, int64_t *pcbFreeSpace));

    /**
     * Returns the last modification timestamp of a file.
     *
     * @return  VBox status code.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pcszFilename    Name of a file to identify the disk.
     * @param   pModificationTime   Where to store the timestamp of the file.
     */
    DECLR3CALLBACKMEMBER(int, pfnGetModificationTime, (void *pvUser, const char *pcszFilename, PRTTIMESPEC pModificationTime));

    /**
     * Returns the size of the opened storage backend.
     *
     * @return  VBox status code.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pStorage        The storage handle to get the size from.
     * @param   pcbSize         Where to store the size of the storage backend.
     */
    DECLR3CALLBACKMEMBER(int, pfnGetSize, (void *pvUser, PVDIOSTORAGE pStorage,
                                           uint64_t *pcbSize));

    /**
     * Sets the size of the opened storage backend if possible.
     *
     * @return  VBox status code.
     * @retval  VERR_NOT_SUPPORTED if the backend does not support this operation.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pStorage        The storage handle.
     * @param   cbSize          The new size of the image.
     */
    DECLR3CALLBACKMEMBER(int, pfnSetSize, (void *pvUser, PVDIOSTORAGE pStorage,
                                           uint64_t cbSize));

    /**
     * Synchronous write callback.
     *
     * @return  VBox status code.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pStorage        The storage handle to use.
     * @param   uOffset         The offset to start from.
     * @param   pvBuffer        Pointer to the bits need to be written.
     * @param   cbBuffer        How many bytes to write.
     * @param   pcbWritten      Where to store how many bytes were actually written.
     *
     * @notes Do not use in code called from the async read/write entry points in the backends.
     *        This should be only used during open/close of images and for the support functions
     *        which are not called while a VM is running (pfnCompact).
     */
    DECLR3CALLBACKMEMBER(int, pfnWriteSync, (void *pvUser, PVDIOSTORAGE pStorage, uint64_t uOffset,
                                             const void *pvBuffer, size_t cbBuffer, size_t *pcbWritten));

    /**
     * Synchronous read callback.
     *
     * @return  VBox status code.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pStorage        The storage handle to use.
     * @param   uOffset         The offset to start from.
     * @param   pvBuffer        Where to store the read bits.
     * @param   cbBuffer        How many bytes to read.
     * @param   pcbRead         Where to store how many bytes were actually read.
     *
     * @notes See pfnWriteSync()
     */
    DECLR3CALLBACKMEMBER(int, pfnReadSync, (void *pvUser, PVDIOSTORAGE pStorage, uint64_t uOffset,
                                            void *pvBuffer, size_t cbBuffer, size_t *pcbRead));

    /**
     * Flush data to the storage backend.
     *
     * @return  VBox status code.
     * @param   pvUser          The opaque data passed on container creation.
     * @param   pStorage        The storage handle to flush.
     *
     * @notes See pfnWriteSync()
     */
    DECLR3CALLBACKMEMBER(int, pfnFlushSync, (void *pvUser, PVDIOSTORAGE pStorage));

    /**
     * Initiate an asynchronous read request for user data.
     *
     * @return  VBox status code.
     * @param   pvUser         The opaque user data passed on container creation.
     * @param   pStorage       The storage handle.
     * @param   uOffset        The offset to start reading from.
     * @param   pIoCtx         I/O context passed in VDAsyncRead/Write.
     * @param   cbRead         How many bytes to read.
     */
    DECLR3CALLBACKMEMBER(int, pfnReadUserAsync, (void *pvUser, PVDIOSTORAGE pStorage,
                                                 uint64_t uOffset, PVDIOCTX pIoCtx,
                                                 size_t cbRead));

    /**
     * Initiate an asynchronous write request for user data.
     *
     * @return  VBox status code.
     * @param   pvUser         The opaque user data passed on container creation.
     * @param   pStorage       The storage handle.
     * @param   uOffset        The offset to start writing to.
     * @param   pIoCtx         I/O context passed in VDAsyncRead/Write
     * @param   cbWrite        How many bytes to write.
     * @param   pfnCompleted   Completion callback.
     * @param   pvCompleteUser Opaque user data passed in the completion callback.
     */
    DECLR3CALLBACKMEMBER(int, pfnWriteUserAsync, (void *pvUser, PVDIOSTORAGE pStorage,
                                                  uint64_t uOffset, PVDIOCTX pIoCtx,
                                                  size_t cbWrite,
                                                  PFNVDXFERCOMPLETED pfnComplete,
                                                  void *pvCompleteUser));

    /**
     * Reads metadata asynchronously from storage.
     * The current I/O context will be halted.
     *
     * @returns VBox status code.
     * @param   pvUser         The opaque user data passed on container creation.
     * @param   pStorage       The storage handle.
     * @param   uOffset        Offset to start reading from.
     * @param   pvBuffer       Where to store the data.
     * @param   cbBuffer       How many bytes to read.
     * @param   pIoCtx         The I/O context which triggered the read.
     * @param   ppMetaXfer     Where to store the metadata transfer handle on success.
     * @param   pfnCompleted   Completion callback.
     * @param   pvCompleteUser Opaque user data passed in the completion callback.
     */
    DECLR3CALLBACKMEMBER(int, pfnReadMetaAsync, (void *pvUser, PVDIOSTORAGE pStorage,
                                                 uint64_t uOffset, void *pvBuffer,
                                                 size_t cbBuffer, PVDIOCTX pIoCtx,
                                                 PPVDMETAXFER ppMetaXfer,
                                                 PFNVDXFERCOMPLETED pfnComplete,
                                                 void *pvCompleteUser));

    /**
     * Writes metadata asynchronously to storage.
     *
     * @returns VBox status code.
     * @param   pvUser         The opaque user data passed on container creation.
     * @param   pStorage       The storage handle.
     * @param   uOffset        Offset to start writing to.
     * @param   pvBuffer       Written data.
     * @param   cbBuffer       How many bytes to write.
     * @param   pIoCtx         The I/O context which triggered the write.
     * @param   pfnCompleted   Completion callback.
     * @param   pvCompleteUser Opaque user data passed in the completion callback.
     */
    DECLR3CALLBACKMEMBER(int, pfnWriteMetaAsync, (void *pvUser, PVDIOSTORAGE pStorage,
                                                  uint64_t uOffset, void *pvBuffer,
                                                  size_t cbBuffer, PVDIOCTX pIoCtx,
                                                  PFNVDXFERCOMPLETED pfnComplete,
                                                  void *pvCompleteUser));

    /**
     * Releases a metadata transfer handle.
     * The free space can be used for another transfer.
     *
     * @returns nothing.
     * @param   pvUser         The opaque user data passed on container creation.
     * @param   pMetaXfer      The metadata transfer handle to release.
     */
    DECLR3CALLBACKMEMBER(void, pfnMetaXferRelease, (void *pvUser, PVDMETAXFER pMetaXfer));

    /**
     * Initiates an async flush request.
     *
     * @return  VBox status code.
     * @param   pvUser         The opaque data passed on container creation.
     * @param   pStorage       The storage handle to flush.
     * @param   pIoCtx         I/O context which triggered the flush.
     * @param   pfnCompleted   Completion callback.
     * @param   pvCompleteUser Opaque user data passed in the completion callback.
     */
    DECLR3CALLBACKMEMBER(int, pfnFlushAsync, (void *pvUser, PVDIOSTORAGE pStorage,
                                              PVDIOCTX pIoCtx,
                                              PFNVDXFERCOMPLETED pfnComplete,
                                              void *pvCompleteUser));

    /**
     * Copies a buffer into the I/O context.
     *
     * @return Number of bytes copied.
     * @param  pvUser          The opaque user data passed on container creation.
     * @param  pIoCtx          I/O context to copy the data to.
     * @param  pvBuffer        Buffer to copy.
     * @param  cbBuffer        Number of bytes to copy.
     */
    DECLR3CALLBACKMEMBER(size_t, pfnIoCtxCopyTo, (void *pvUser, PVDIOCTX pIoCtx,
                                                  void *pvBuffer, size_t cbBuffer));

    /**
     * Copies data from the I/O context into a buffer.
     *
     * @return Number of bytes copied.
     * @param  pvUser          The opaque user data passed on container creation.
     * @param  pIoCtx          I/O context to copy the data from.
     * @param  pvBuffer        Destination buffer.
     * @param  cbBuffer        Number of bytes to copy.
     */
    DECLR3CALLBACKMEMBER(size_t, pfnIoCtxCopyFrom, (void *pvUser, PVDIOCTX pIoCtx,
                                                    void *pvBuffer, size_t cbBuffer));

    /**
     * Sets the buffer of the given context to a specific byte.
     *
     * @return Number of bytes set.
     * @param  pvUser          The opaque user data passed on container creation.
     * @param  pIoCtx          I/O context to copy the data from.
     * @param  ch              The byte to set.
     * @param  cbSet           Number of bytes to set.
     */
    DECLR3CALLBACKMEMBER(size_t, pfnIoCtxSet, (void *pvUser, PVDIOCTX pIoCtx,
                                               int ch, size_t cbSet));

    /**
     * Creates a segment array from the I/O context data buffer.
     *
     * @returns Number of bytes the array describes.
     * @param  pvUser          The opaque user data passed on container creation.
     * @param  pIoCtx          I/O context to copy the data from.
     * @param  paSeg           The uninitialized segment array.
     *                         If NULL pcSeg will contain the number of segments needed
     *                         to describe the requested amount of data.
     * @param  pcSeg           The number of segments the given array has.
     *                         This will hold the actual number of entries needed upon return.
     * @param  cbData          Number of bytes the new array should describe.
     */
    DECLR3CALLBACKMEMBER(size_t, pfnIoCtxSegArrayCreate, (void *pvUser, PVDIOCTX pIoCtx,
                                                          PRTSGSEG paSeg, unsigned *pcSeg,
                                                          size_t cbData));
    /**
     * Marks the given number of bytes as completed and continues the I/O context.
     *
     * @returns nothing.
     * @param   pvUser         The opaque user data passed on container creation.
     * @param   pIoCtx         The I/O context.
     * @param   rcReq          Status code the request completed with.
     * @param   cbCompleted    Number of bytes completed.
     */
    DECLR3CALLBACKMEMBER(void, pfnIoCtxCompleted, (void *pvUser, PVDIOCTX pIoCtx,
                                                   int rcReq, size_t cbCompleted));
} VDINTERFACEIOINT, *PVDINTERFACEIOINT;

/**
 * Get internal I/O interface from interface list.
 *
 * @return Pointer to the first internal I/O interface in the list.
 * @param  pVDIfs    Pointer to the interface list.
 */
DECLINLINE(PVDINTERFACEIOINT) VDIfIoIntGet(PVDINTERFACE pVDIfs)
{
    PVDINTERFACE pIf = VDInterfaceGet(pVDIfs, VDINTERFACETYPE_IOINT);

    /* Check that the interface descriptor is a progress interface. */
    AssertMsgReturn(   !pIf
                    || (   (pIf->enmInterface == VDINTERFACETYPE_IOINT)
                        && (pIf->cbSize == sizeof(VDINTERFACEIOINT))),
                    ("Not an internal I/O interface"), NULL);

    return (PVDINTERFACEIOINT)pIf;
}

DECLINLINE(int) vdIfIoIntFileOpen(PVDINTERFACEIOINT pIfIoInt, const char *pszFilename,
                                  uint32_t fOpen, PPVDIOSTORAGE ppStorage)
{
    return pIfIoInt->pfnOpen(pIfIoInt->Core.pvUser, pszFilename, fOpen, ppStorage);
}

DECLINLINE(int) vdIfIoIntFileClose(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage)
{
    return pIfIoInt->pfnClose(pIfIoInt->Core.pvUser, pStorage);
}

DECLINLINE(int) vdIfIoIntFileDelete(PVDINTERFACEIOINT pIfIoInt, const char *pszFilename)
{
    return pIfIoInt->pfnDelete(pIfIoInt->Core.pvUser, pszFilename);
}

DECLINLINE(int) vdIfIoIntFileMove(PVDINTERFACEIOINT pIfIoInt, const char *pszSrc,
                                  const char *pszDst, unsigned fMove)
{
    return pIfIoInt->pfnMove(pIfIoInt->Core.pvUser, pszSrc, pszDst, fMove);
}

DECLINLINE(int) vdIfIoIntFileGetFreeSpace(PVDINTERFACEIOINT pIfIoInt, const char *pszFilename,
                                          int64_t *pcbFree)
{
    return pIfIoInt->pfnGetFreeSpace(pIfIoInt->Core.pvUser, pszFilename, pcbFree);
}

DECLINLINE(int) vdIfIoIntFileGetModificationTime(PVDINTERFACEIOINT pIfIoInt, const char *pcszFilename,
                                                 PRTTIMESPEC pModificationTime)
{
    return pIfIoInt->pfnGetModificationTime(pIfIoInt->Core.pvUser, pcszFilename,
                                            pModificationTime);
}

DECLINLINE(int) vdIfIoIntFileGetSize(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage,
                                     uint64_t *pcbSize)
{
    return pIfIoInt->pfnGetSize(pIfIoInt->Core.pvUser, pStorage, pcbSize);
}

DECLINLINE(int) vdIfIoIntFileSetSize(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage,
                                     uint64_t cbSize)
{
    return pIfIoInt->pfnSetSize(pIfIoInt->Core.pvUser, pStorage, cbSize);
}

DECLINLINE(int) vdIfIoIntFileWriteSync(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage,
                                       uint64_t uOffset, const void *pvBuffer, size_t cbBuffer,
                                       size_t *pcbWritten)
{
    return pIfIoInt->pfnWriteSync(pIfIoInt->Core.pvUser, pStorage, uOffset,
                                  pvBuffer, cbBuffer, pcbWritten);
}

DECLINLINE(int) vdIfIoIntFileReadSync(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage,
                                      uint64_t uOffset, void *pvBuffer, size_t cbBuffer,
                                      size_t *pcbRead)
{
    return pIfIoInt->pfnReadSync(pIfIoInt->Core.pvUser, pStorage, uOffset,
                                 pvBuffer, cbBuffer, pcbRead);
}

DECLINLINE(int) vdIfIoIntFileFlushSync(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage)
{
    return pIfIoInt->pfnFlushSync(pIfIoInt->Core.pvUser, pStorage);
}

DECLINLINE(int) vdIfIoIntFileReadUserAsync(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage,
                                           uint64_t uOffset, PVDIOCTX pIoCtx, size_t cbRead)
{
    return pIfIoInt->pfnReadUserAsync(pIfIoInt->Core.pvUser, pStorage,
                                      uOffset, pIoCtx, cbRead);
}

DECLINLINE(int) vdIfIoIntFileWriteUserAsync(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage,
                                            uint64_t uOffset, PVDIOCTX pIoCtx, size_t cbWrite,
                                            PFNVDXFERCOMPLETED pfnComplete,
                                            void *pvCompleteUser)
{
    return pIfIoInt->pfnWriteUserAsync(pIfIoInt->Core.pvUser, pStorage,
                                       uOffset, pIoCtx, cbWrite, pfnComplete,
                                       pvCompleteUser);
}

DECLINLINE(int) vdIfIoIntFileReadMetaAsync(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage,
                                           uint64_t uOffset, void *pvBuffer,
                                           size_t cbBuffer, PVDIOCTX pIoCtx,
                                           PPVDMETAXFER ppMetaXfer,
                                           PFNVDXFERCOMPLETED pfnComplete,
                                           void *pvCompleteUser)
{
    return pIfIoInt->pfnReadMetaAsync(pIfIoInt->Core.pvUser, pStorage,
                                      uOffset, pvBuffer, cbBuffer, pIoCtx,
                                      ppMetaXfer, pfnComplete, pvCompleteUser);
}

DECLINLINE(int) vdIfIoIntFileWriteMetaAsync(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage,
                                            uint64_t uOffset, void *pvBuffer,
                                            size_t cbBuffer, PVDIOCTX pIoCtx,
                                            PFNVDXFERCOMPLETED pfnComplete,
                                            void *pvCompleteUser)
{
    return pIfIoInt->pfnWriteMetaAsync(pIfIoInt->Core.pvUser, pStorage,
                                       uOffset, pvBuffer, cbBuffer, pIoCtx,
                                       pfnComplete, pvCompleteUser);
}

DECLINLINE(void) vdIfIoIntMetaXferRelease(PVDINTERFACEIOINT pIfIoInt, PVDMETAXFER pMetaXfer)
{
    pIfIoInt->pfnMetaXferRelease(pIfIoInt->Core.pvUser, pMetaXfer);
}

DECLINLINE(int) vdIfIoIntFileFlushAsync(PVDINTERFACEIOINT pIfIoInt, PVDIOSTORAGE pStorage,
                                        PVDIOCTX pIoCtx, PFNVDXFERCOMPLETED pfnComplete,
                                        void *pvCompleteUser)
{
    return pIfIoInt->pfnFlushAsync(pIfIoInt->Core.pvUser, pStorage, pIoCtx, pfnComplete,
                                   pvCompleteUser);
}

DECLINLINE(size_t) vdIfIoIntIoCtxSet(PVDINTERFACEIOINT pIfIoInt, PVDIOCTX pIoCtx,
                                     int ch, size_t cbSet)
{
    return pIfIoInt->pfnIoCtxSet(pIfIoInt->Core.pvUser, pIoCtx, ch, cbSet);
}

RT_C_DECLS_END

/** @} */

#endif
