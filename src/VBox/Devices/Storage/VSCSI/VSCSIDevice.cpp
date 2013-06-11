/* $Id: VSCSIDevice.cpp $ */
/** @file
 * Virtual SCSI driver: Device handling
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
#define LOG_GROUP LOG_GROUP_VSCSI
#include <VBox/log.h>
#include <VBox/err.h>
#include <VBox/types.h>
#include <VBox/vscsi.h>
#include <iprt/assert.h>
#include <iprt/mem.h>
#include <iprt/string.h>

#include "VSCSIInternal.h"

/**
 * Checks if a specific LUN exists fir the SCSI device
 *
 * @returns true if the LUN is present
 *          false otherwise
 * @param   pVScsiDevice    The SCSI device instance.
 * @param   iLun            The LUN to check for.
 */
DECLINLINE(bool) vscsiDeviceLunIsPresent(PVSCSIDEVICEINT pVScsiDevice, uint32_t iLun)
{
    return (   iLun < pVScsiDevice->cLunsMax
            && pVScsiDevice->papVScsiLun[iLun] != NULL);
}

/**
 * Process a request common for all device types.
 *
 * @returns Flag whether we could handle the request.
 * @param   pVScsiDevice    The virtual SCSI device instance.
 * @param   pVScsiReq       The SCSi request.
 * @param   prcReq          The final return value if the request was handled.
 */
static bool vscsiDeviceReqProcess(PVSCSIDEVICEINT pVScsiDevice, PVSCSIREQINT pVScsiReq,
                                  int *prcReq)
{
    bool fProcessed = true;

    switch (pVScsiReq->pbCDB[0])
    {
        case SCSI_INQUIRY:
        {
            if (!vscsiDeviceLunIsPresent(pVScsiDevice, pVScsiReq->iLun))
            {
                size_t cbData;
                SCSIINQUIRYDATA ScsiInquiryReply;

                memset(&ScsiInquiryReply, 0, sizeof(ScsiInquiryReply));
                ScsiInquiryReply.cbAdditional = 31;
                ScsiInquiryReply.u5PeripheralDeviceType = SCSI_INQUIRY_DATA_PERIPHERAL_DEVICE_TYPE_UNKNOWN;
                ScsiInquiryReply.u3PeripheralQualifier = SCSI_INQUIRY_DATA_PERIPHERAL_QUALIFIER_NOT_CONNECTED_NOT_SUPPORTED;
                cbData = RTSgBufCopyFromBuf(&pVScsiReq->SgBuf, (uint8_t *)&ScsiInquiryReply, sizeof(SCSIINQUIRYDATA));
                *prcReq = vscsiReqSenseOkSet(&pVScsiDevice->VScsiSense, pVScsiReq);
            }
            else
                fProcessed = false; /* Let the LUN process the request because it will provide LUN specific data */

            break;
        }
        case SCSI_REPORT_LUNS:
        {
            /*
             * If allocation length is less than 16 bytes SPC compliant devices have
             * to return an error.
             */
            if (vscsiBE2HU32(&pVScsiReq->pbCDB[6]) < 16)
                *prcReq = vscsiReqSenseErrorSet(&pVScsiDevice->VScsiSense, pVScsiReq, SCSI_SENSE_ILLEGAL_REQUEST, SCSI_ASC_INV_FIELD_IN_CMD_PACKET, 0x00);
            else
            {
                size_t cbData;
                uint8_t aReply[16]; /* We report only one LUN. */

                memset(aReply, 0, sizeof(aReply));
                vscsiH2BEU32(&aReply[0], 8); /* List length starts at position 0. */
                cbData = RTSgBufCopyFromBuf(&pVScsiReq->SgBuf, aReply, sizeof(aReply));
                if (cbData < 16)
                    *prcReq = vscsiReqSenseErrorSet(&pVScsiDevice->VScsiSense, pVScsiReq, SCSI_SENSE_ILLEGAL_REQUEST, SCSI_ASC_INV_FIELD_IN_CMD_PACKET, 0x00);
                else
                    *prcReq = vscsiReqSenseOkSet(&pVScsiDevice->VScsiSense, pVScsiReq);
            }
            break;
        }
        case SCSI_TEST_UNIT_READY:
        {
            *prcReq = vscsiReqSenseOkSet(&pVScsiDevice->VScsiSense, pVScsiReq);
            break;
        }
        case SCSI_REQUEST_SENSE:
        {
            /* Descriptor format sense data is not supported and results in an error. */
            if ((pVScsiReq->pbCDB[1] & 0x1) != 0)
                *prcReq = vscsiReqSenseErrorSet(&pVScsiDevice->VScsiSense, pVScsiReq, SCSI_SENSE_ILLEGAL_REQUEST, SCSI_ASC_INV_FIELD_IN_CMD_PACKET, 0x00);
            else
                *prcReq = vscsiReqSenseCmd(&pVScsiDevice->VScsiSense, pVScsiReq);
        }
        default:
            fProcessed = false;
    }

    return fProcessed;
}


void vscsiDeviceReqComplete(PVSCSIDEVICEINT pVScsiDevice, PVSCSIREQINT pVScsiReq,
                            int rcScsiCode, bool fRedoPossible, int rcReq)
{
    pVScsiDevice->pfnVScsiReqCompleted(pVScsiDevice, pVScsiDevice->pvVScsiDeviceUser,
                                       pVScsiReq->pvVScsiReqUser, rcScsiCode, fRedoPossible,
                                       rcReq);

    RTMemCacheFree(pVScsiDevice->hCacheReq, pVScsiReq);
}


VBOXDDU_DECL(int) VSCSIDeviceCreate(PVSCSIDEVICE phVScsiDevice,
                                    PFNVSCSIREQCOMPLETED pfnVScsiReqCompleted,
                                    void *pvVScsiDeviceUser)
{
    int rc = VINF_SUCCESS;
    PVSCSIDEVICEINT pVScsiDevice = NULL;

    AssertPtrReturn(phVScsiDevice, VERR_INVALID_POINTER);
    AssertPtrReturn(pfnVScsiReqCompleted, VERR_INVALID_POINTER);

    pVScsiDevice = (PVSCSIDEVICEINT)RTMemAllocZ(sizeof(VSCSIDEVICEINT));
    if (!pVScsiDevice)
        return VERR_NO_MEMORY;

    pVScsiDevice->pfnVScsiReqCompleted = pfnVScsiReqCompleted;
    pVScsiDevice->pvVScsiDeviceUser    = pvVScsiDeviceUser;
    pVScsiDevice->cLunsAttached        = 0;
    pVScsiDevice->cLunsMax             = 0;
    pVScsiDevice->papVScsiLun          = NULL;
    vscsiSenseInit(&pVScsiDevice->VScsiSense);

    rc = RTMemCacheCreate(&pVScsiDevice->hCacheReq, sizeof(VSCSIREQINT), 0, UINT32_MAX,
                          NULL, NULL, NULL, 0);
    if (RT_SUCCESS(rc))
    {
        *phVScsiDevice = pVScsiDevice;
        LogFlow(("%s: hVScsiDevice=%#p -> VINF_SUCCESS\n", __FUNCTION__, pVScsiDevice));
        return VINF_SUCCESS;
    }

    RTMemFree(pVScsiDevice);

    return rc;
}


VBOXDDU_DECL(int) VSCSIDeviceDestroy(VSCSIDEVICE hVScsiDevice)
{
    AssertPtrReturn(hVScsiDevice, VERR_INVALID_HANDLE);

    PVSCSIDEVICEINT pVScsiDevice = (PVSCSIDEVICEINT)hVScsiDevice;

    if (pVScsiDevice->cLunsAttached > 0)
        return VERR_VSCSI_LUN_ATTACHED_TO_DEVICE;

    if (pVScsiDevice->papVScsiLun)
        RTMemFree(pVScsiDevice->papVScsiLun);

    RTMemCacheDestroy(pVScsiDevice->hCacheReq);
    RTMemFree(pVScsiDevice);

    return VINF_SUCCESS;;
}


VBOXDDU_DECL(int) VSCSIDeviceLunAttach(VSCSIDEVICE hVScsiDevice, VSCSILUN hVScsiLun, uint32_t iLun)
{
    PVSCSIDEVICEINT pVScsiDevice = (PVSCSIDEVICEINT)hVScsiDevice;
    PVSCSILUNINT    pVScsiLun    = (PVSCSILUNINT)hVScsiLun;
    int rc = VINF_SUCCESS;

    /* Parameter checks */
    AssertPtrReturn(pVScsiDevice, VERR_INVALID_HANDLE);
    AssertPtrReturn(pVScsiLun, VERR_INVALID_HANDLE);
    AssertReturn(iLun < VSCSI_DEVICE_LUN_MAX, VERR_VSCSI_LUN_INVALID);
    AssertReturn(!pVScsiLun->pVScsiDevice, VERR_VSCSI_LUN_ATTACHED_TO_DEVICE);

    if (iLun >= pVScsiDevice->cLunsMax)
    {
        PPVSCSILUNINT papLunOld = pVScsiDevice->papVScsiLun;

        pVScsiDevice->papVScsiLun = (PPVSCSILUNINT)RTMemAllocZ((iLun + 1) * sizeof(PVSCSILUNINT));
        if (pVScsiDevice->papVScsiLun)
        {
            for (uint32_t i = 0; i < pVScsiDevice->cLunsMax; i++)
                pVScsiDevice->papVScsiLun[i] = papLunOld[i];

            if (papLunOld)
                RTMemFree(papLunOld);

            pVScsiDevice->cLunsMax = iLun + 1;
        }
        else
            rc = VERR_NO_MEMORY;
    }

    if (RT_SUCCESS(rc))
    {
        pVScsiLun->pVScsiDevice         = pVScsiDevice;
        pVScsiDevice->papVScsiLun[iLun] = pVScsiLun;
        pVScsiDevice->cLunsAttached++;
    }

    return rc;
}


VBOXDDU_DECL(int) VSCSIDeviceLunDetach(VSCSIDEVICE hVScsiDevice, uint32_t iLun,
                                       PVSCSILUN phVScsiLun)
{
    PVSCSIDEVICEINT pVScsiDevice = (PVSCSIDEVICEINT)hVScsiDevice;

    /* Parameter checks */
    AssertPtrReturn(pVScsiDevice, VERR_INVALID_HANDLE);
    AssertPtrReturn(phVScsiLun, VERR_INVALID_POINTER);
    AssertReturn(iLun < VSCSI_DEVICE_LUN_MAX, VERR_VSCSI_LUN_INVALID);
    AssertReturn(iLun < pVScsiDevice->cLunsMax, VERR_VSCSI_LUN_NOT_ATTACHED);
    AssertPtrReturn(pVScsiDevice->papVScsiLun[iLun], VERR_VSCSI_LUN_NOT_ATTACHED);

    PVSCSILUNINT pVScsiLun = pVScsiDevice->papVScsiLun[iLun];

    pVScsiLun->pVScsiDevice = NULL;
    *phVScsiLun = pVScsiLun;
    pVScsiDevice->papVScsiLun[iLun] = NULL;
    pVScsiDevice->cLunsAttached--;

    return VINF_SUCCESS;
}


VBOXDDU_DECL(int) VSCSIDeviceLunGet(VSCSIDEVICE hVScsiDevice, uint32_t iLun,
                                    PVSCSILUN phVScsiLun)
{
    PVSCSIDEVICEINT pVScsiDevice = (PVSCSIDEVICEINT)hVScsiDevice;

    /* Parameter checks */
    AssertPtrReturn(pVScsiDevice, VERR_INVALID_HANDLE);
    AssertPtrReturn(phVScsiLun, VERR_INVALID_POINTER);
    AssertReturn(iLun < VSCSI_DEVICE_LUN_MAX, VERR_VSCSI_LUN_INVALID);
    AssertReturn(iLun < pVScsiDevice->cLunsMax, VERR_VSCSI_LUN_NOT_ATTACHED);
    AssertPtrReturn(pVScsiDevice->papVScsiLun[iLun], VERR_VSCSI_LUN_NOT_ATTACHED);

    *phVScsiLun = pVScsiDevice->papVScsiLun[iLun];

    return VINF_SUCCESS;
}


VBOXDDU_DECL(int) VSCSIDeviceReqEnqueue(VSCSIDEVICE hVScsiDevice, VSCSIREQ hVScsiReq)
{
    PVSCSIDEVICEINT pVScsiDevice = (PVSCSIDEVICEINT)hVScsiDevice;
    PVSCSIREQINT    pVScsiReq    = (PVSCSIREQINT)hVScsiReq;
    int rc = VINF_SUCCESS;

    /* Parameter checks */
    AssertPtrReturn(pVScsiDevice, VERR_INVALID_HANDLE);
    AssertPtrReturn(pVScsiReq, VERR_INVALID_HANDLE);

    /* Check if this request can be handled by us */
    int rcReq;
    bool fProcessed = vscsiDeviceReqProcess(pVScsiDevice, pVScsiReq, &rcReq);
    if (!fProcessed)
    {
        /* Pass to the LUN driver */
        if (vscsiDeviceLunIsPresent(pVScsiDevice, pVScsiReq->iLun))
        {
            PVSCSILUNINT pVScsiLun = pVScsiDevice->papVScsiLun[pVScsiReq->iLun];
            rc = pVScsiLun->pVScsiLunDesc->pfnVScsiLunReqProcess(pVScsiLun, pVScsiReq);
        }
        else
        {
            /* LUN not present, report error. */
            vscsiReqSenseErrorSet(&pVScsiDevice->VScsiSense, pVScsiReq,
                                  SCSI_SENSE_ILLEGAL_REQUEST,
                                  SCSI_ASC_LOGICAL_UNIT_DOES_NOT_RESPOND_TO_SELECTION,
                                  0x00);

            vscsiDeviceReqComplete(pVScsiDevice, pVScsiReq,
                                   SCSI_STATUS_CHECK_CONDITION, false, VINF_SUCCESS);
        }
    }
    else
        vscsiDeviceReqComplete(pVScsiDevice, pVScsiReq,
                               rcReq, false, VINF_SUCCESS);

    return rc;
}


VBOXDDU_DECL(int) VSCSIDeviceReqCreate(VSCSIDEVICE hVScsiDevice, PVSCSIREQ phVScsiReq,
                                       uint32_t iLun, uint8_t *pbCDB, size_t cbCDB,
                                       size_t cbSGList, unsigned cSGListEntries,
                                       PCRTSGSEG paSGList, uint8_t *pbSense,
                                       size_t cbSense, void *pvVScsiReqUser)
{
    PVSCSIDEVICEINT pVScsiDevice = (PVSCSIDEVICEINT)hVScsiDevice;
    PVSCSIREQINT    pVScsiReq    = NULL;

    /* Parameter checks */
    AssertPtrReturn(pVScsiDevice, VERR_INVALID_HANDLE);
    AssertPtrReturn(phVScsiReq, VERR_INVALID_POINTER);
    AssertPtrReturn(pbCDB, VERR_INVALID_PARAMETER);
    AssertReturn(cbCDB > 0, VERR_INVALID_PARAMETER);

    pVScsiReq = (PVSCSIREQINT)RTMemCacheAlloc(pVScsiDevice->hCacheReq);
    if (!pVScsiReq)
        return VERR_NO_MEMORY;

    pVScsiReq->iLun           = iLun;
    pVScsiReq->pbCDB          = pbCDB;
    pVScsiReq->cbCDB          = cbCDB;
    pVScsiReq->pbSense        = pbSense;
    pVScsiReq->cbSense        = cbSense;
    pVScsiReq->pvVScsiReqUser = pvVScsiReqUser;

    if (cSGListEntries)
        RTSgBufInit(&pVScsiReq->SgBuf, paSGList, cSGListEntries);

    *phVScsiReq = pVScsiReq;

    return VINF_SUCCESS;
}

