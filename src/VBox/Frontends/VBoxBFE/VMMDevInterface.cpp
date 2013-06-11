/* $Id: VMMDevInterface.cpp $ */
/** @file
 * VBox frontends: Basic Frontend (BFE):
 * Implementation of VMMDev: driver interface to VMM device
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

#define LOG_GROUP LOG_GROUP_MAIN

#ifdef VBOXBFE_WITHOUT_COM
# include "COMDefs.h"
#else
# include <VBox/com/defs.h>
#endif
#include <VBox/vmm/pdm.h>
#include <VBox/VMMDev.h>
#include <VBox/vmm/cfgm.h>
#include <VBox/err.h>
#include <iprt/assert.h>
#include <VBox/log.h>
#include <iprt/asm.h>
#include <iprt/uuid.h>

#include "VBoxBFE.h"
#include "VMMDev.h"
#include "MouseImpl.h"
#include "DisplayImpl.h"
#include "ConsoleImpl.h"
#ifdef VBOX_WITH_HGCM
#include "HGCM.h"
#endif

#ifdef RT_OS_OS2
# define VBOXSHAREDFOLDERS_DLL "VBoxSFld"
#else
# define VBOXSHAREDFOLDERS_DLL "VBoxSharedFolders"
#endif

/**
 * VMMDev driver instance data.
 */
typedef struct DRVMAINVMMDEV
{
    /** Pointer to the VMMDev object. */
    VMMDev                      *pVMMDev;
    /** Pointer to the driver instance structure. */
    PPDMDRVINS                  pDrvIns;
    /** Pointer to the VMMDev port interface of the driver/device above us. */
    PPDMIVMMDEVPORT             pUpPort;
    /** Our VMM device connector interface. */
    PDMIVMMDEVCONNECTOR         Connector;

#ifdef VBOX_WITH_HGCM
    /** Pointer to the HGCM port interface of the driver/device above us. */
    PPDMIHGCMPORT               pHGCMPort;
    /** Our HGCM connector interface. */
    PDMIHGCMCONNECTOR           HGCMConnector;
#endif
} DRVMAINVMMDEV, *PDRVMAINVMMDEV;

/** Converts PDMIVMMDEVCONNECTOR pointer to a DRVMAINVMMDEV pointer. */
#define PDMIVMMDEVCONNECTOR_2_MAINVMMDEV(pInterface) ( (PDRVMAINVMMDEV) ((uintptr_t)pInterface - RT_OFFSETOF(DRVMAINVMMDEV, Connector)) )

#ifdef VBOX_WITH_HGCM
/** Converts PDMIHGCMCONNECTOR pointer to a DRVMAINVMMDEV pointer. */
#define PDMIHGCMCONNECTOR_2_MAINVMMDEV(pInterface) ( (PDRVMAINVMMDEV) ((uintptr_t)pInterface - RT_OFFSETOF(DRVMAINVMMDEV, HGCMConnector)) )
#endif

VMMDev::VMMDev() : mpDrv(NULL)
{
#ifdef VBOX_WITH_HGCM
    int rc = VINF_SUCCESS;
    if (fActivateHGCM())
        rc = HGCMHostInit();
    AssertRC(rc);
#endif
}

VMMDev::~VMMDev()
{
#ifdef VBOX_WITH_HGCM
    if (fActivateHGCM())
        HGCMHostShutdown ();
#endif /* VBOX_WITH_HGCM */
}


PPDMIVMMDEVPORT VMMDev::getVMMDevPort()
{
    Assert(mpDrv);
    return mpDrv->pUpPort;
}

/**
 * Update the mouse capabilities.
 * This is called when the mouse capabilities change. The new capabilities
 * are given and the connector should update its internal state.
 *
 * @param   pInterface          Pointer to this interface.
 * @param   newCapabilities     New capabilities.
 * @thread  The emulation thread.
 */
DECLCALLBACK(void) VMMDev::UpdateMouseCapabilities(PPDMIVMMDEVCONNECTOR pInterface, uint32_t fNewCaps)
{
    /*
     * Tell the console interface about the event so that it can notify its consumers.
     */

    if (gMouse)
        gMouse->onVMMDevGuestCapsChange(fNewCaps & VMMDEV_MOUSE_GUEST_MASK);
    if (gConsole)
        gConsole->resetCursor();
}


/**
 * Update the pointer shape or visibility.
 *
 * This is called when the mouse pointer shape changes or pointer is hidden/displaying.
 * The new shape is passed as a caller allocated buffer that will be freed after returning.
 *
 * @param   pInterface          Pointer to this interface.
 * @param   fVisible            Whether the pointer is visible or not.
 * @param   fAlpha              Alpha channel information is present.
 * @param   xHot                Horizontal coordinate of the pointer hot spot.
 * @param   yHot                Vertical coordinate of the pointer hot spot.
 * @param   width               Pointer width in pixels.
 * @param   height              Pointer height in pixels.
 * @param   pShape              The shape buffer. If NULL, then only pointer visibility is being changed.
 * @thread  The emulation thread.
 */
DECLCALLBACK(void) VMMDev::UpdatePointerShape(PPDMIVMMDEVCONNECTOR pInterface, bool fVisible, bool fAlpha,
                                              uint32_t xHot, uint32_t yHot,
                                              uint32_t width, uint32_t height,
                                              void *pShape)
{
    if (gConsole)
        gConsole->onMousePointerShapeChange(fVisible, fAlpha, xHot,
                                            yHot, width, height, pShape);
}

DECLCALLBACK(int) iface_VideoAccelEnable(PPDMIVMMDEVCONNECTOR pInterface, bool fEnable, VBVAMEMORY *pVbvaMemory)
{
    LogFlow(("VMMDev::VideoAccelEnable: %d, %p\n", fEnable, pVbvaMemory));
    if (gDisplay)
        gDisplay->VideoAccelEnable (fEnable, pVbvaMemory);
    return VINF_SUCCESS;
}

DECLCALLBACK(void) iface_VideoAccelFlush(PPDMIVMMDEVCONNECTOR pInterface)
{
    if (gDisplay)
        gDisplay->VideoAccelFlush ();
}

DECLCALLBACK(int) iface_SetVisibleRegion(PPDMIVMMDEVCONNECTOR pInterface, uint32_t cRect, PRTRECT pRect)
{
    /* not implemented */
    return VINF_SUCCESS;
}

DECLCALLBACK(int) iface_QueryVisibleRegion(PPDMIVMMDEVCONNECTOR pInterface, uint32_t *pcRect, PRTRECT pRect)
{
    /* not implemented */
    return VINF_SUCCESS;
}

DECLCALLBACK(int) VMMDev::VideoModeSupported(PPDMIVMMDEVCONNECTOR pInterface, uint32_t display, uint32_t width, uint32_t height,
                                             uint32_t bpp, bool *fSupported)
{
    PDRVMAINVMMDEV pDrv = PDMIVMMDEVCONNECTOR_2_MAINVMMDEV(pInterface);
    (void)pDrv;

    if (!fSupported)
        return VERR_INVALID_PARAMETER;
    *fSupported = true;
    return VINF_SUCCESS;
}

DECLCALLBACK(int) VMMDev::GetHeightReduction(PPDMIVMMDEVCONNECTOR pInterface, uint32_t *heightReduction)
{
    PDRVMAINVMMDEV pDrv = PDMIVMMDEVCONNECTOR_2_MAINVMMDEV(pInterface);
    (void)pDrv;

    if (!heightReduction)
        return VERR_INVALID_PARAMETER;
    /* XXX hard-coded */
    *heightReduction = 18;
    return VINF_SUCCESS;
}

DECLCALLBACK(int) VMMDev::QueryBalloonSize(PPDMIVMMDEVCONNECTOR pInterface, uint32_t *pu32BalloonSize)
{
    PDRVMAINVMMDEV pDrv = PDMIVMMDEVCONNECTOR_2_MAINVMMDEV(pInterface);
    (void)pDrv;

    AssertPtr(pu32BalloonSize);
    *pu32BalloonSize = 0;
    return VINF_SUCCESS;
}

#ifdef VBOX_WITH_HGCM

/* HGCM connector interface */

static DECLCALLBACK(int) iface_hgcmConnect (PPDMIHGCMCONNECTOR pInterface, PVBOXHGCMCMD pCmd, PHGCMSERVICELOCATION pServiceLocation, uint32_t *pu32ClientID)
{
    LogSunlover(("Enter\n"));

    PDRVMAINVMMDEV pDrv = PDMIHGCMCONNECTOR_2_MAINVMMDEV(pInterface);

    if (    !pServiceLocation
        || (   pServiceLocation->type != VMMDevHGCMLoc_LocalHost
            && pServiceLocation->type != VMMDevHGCMLoc_LocalHost_Existing))
    {
        return VERR_INVALID_PARAMETER;
    }

    return HGCMGuestConnect (pDrv->pHGCMPort, pCmd, pServiceLocation->u.host.achName, pu32ClientID);
}

static DECLCALLBACK(int) iface_hgcmDisconnect (PPDMIHGCMCONNECTOR pInterface, PVBOXHGCMCMD pCmd, uint32_t u32ClientID)
{
    LogSunlover(("Enter\n"));

    PDRVMAINVMMDEV pDrv = PDMIHGCMCONNECTOR_2_MAINVMMDEV(pInterface);

    return HGCMGuestDisconnect (pDrv->pHGCMPort, pCmd, u32ClientID);
}

static DECLCALLBACK(int) iface_hgcmCall (PPDMIHGCMCONNECTOR pInterface, PVBOXHGCMCMD pCmd, uint32_t u32ClientID, uint32_t u32Function,
                                         uint32_t cParms, PVBOXHGCMSVCPARM paParms)
{
    LogSunlover(("Enter\n"));

    PDRVMAINVMMDEV pDrv = PDMIHGCMCONNECTOR_2_MAINVMMDEV(pInterface);

    return HGCMGuestCall (pDrv->pHGCMPort, pCmd, u32ClientID, u32Function, cParms, paParms);
}

/**
 * Execute state save operation.
 *
 * @returns VBox status code.
 * @param   pDrvIns         Driver instance of the driver which registered the data unit.
 * @param   pSSM            SSM operation handle.
 */
static DECLCALLBACK(int) iface_hgcmSave(PPDMDRVINS pDrvIns, PSSMHANDLE pSSM)
{
    LogSunlover(("Enter\n"));
    return HGCMHostSaveState (pSSM);
}


/**
 * Execute state load operation.
 *
 * @returns VBox status code.
 * @param   pDrvIns         Driver instance of the driver which registered the data unit.
 * @param   pSSM            SSM operation handle.
 * @param   uVersion        Data layout version.
 * @param   uPass           The data pass.
 */
static DECLCALLBACK(int) iface_hgcmLoad(PPDMDRVINS pDrvIns, PSSMHANDLE pSSM, uint32_t uVersion, uint32_t uPass)
{
    LogFlowFunc(("Enter\n"));

    if (uVersion != HGCM_SSM_VERSION)
        return VERR_SSM_UNSUPPORTED_DATA_UNIT_VERSION;
    Assert(uPass == SSM_PASS_FINAL); NOREF(uPass);

    return HGCMHostLoadState (pSSM);
}

int VMMDev::hgcmLoadService (const char *pszServiceLibrary, const char *pszServiceName)
{
    return HGCMHostLoad (pszServiceLibrary, pszServiceName);
}

int VMMDev::hgcmHostCall (const char *pszServiceName, uint32_t u32Function,
                          uint32_t cParms, PVBOXHGCMSVCPARM paParms)
{
    return HGCMHostCall (pszServiceName, u32Function, cParms, paParms);
}
#endif /* HGCM */

/**
 * @interface_method_impl{PDMIBASE,pfnQueryInterface}
 */
DECLCALLBACK(void *) VMMDev::drvQueryInterface(PPDMIBASE pInterface, const char *pszIID)
{
    PPDMDRVINS pDrvIns = PDMIBASE_2_PDMDRV(pInterface);
    PDRVMAINVMMDEV pDrv = PDMINS_2_DATA(pDrvIns, PDRVMAINVMMDEV);
    if (RTUuidCompare2Strs(pszIID, PDMIBASE_IID) == 0)
        return &pDrvIns->IBase;
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIVMMDEVCONNECTOR, &pDrv->Connector);
#ifdef VBOX_WITH_HGCM
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIHGCMCONNECTOR, fActivateHGCM() ? &pDrv->HGCMConnector : NULL);
#endif
    return NULL;
}


/**
 * Destruct a VMMDev driver instance.
 *
 * @returns VBox status.
 * @param   pDrvIns     The driver instance data.
 */
DECLCALLBACK(void) VMMDev::drvDestruct(PPDMDRVINS pDrvIns)
{
    /*PDRVMAINVMMDEV pData = PDMINS_2_DATA(pDrvIns, PDRVMAINVMMDEV); - unused variables makes gcc bitch. */
}


/**
 * Construct a VMMDev driver instance.
 *
 * @copydoc FNPDMDRVCONSTRUCT
 */
DECLCALLBACK(int) VMMDev::drvConstruct(PPDMDRVINS pDrvIns, PCFGMNODE pCfg, uint32_t fFlags)
{
    PDRVMAINVMMDEV pData = PDMINS_2_DATA(pDrvIns, PDRVMAINVMMDEV);
    LogFlow(("Keyboard::drvConstruct: iInstance=%d\n", pDrvIns->iInstance));

    /*
     * Validate configuration.
     */
    if (!CFGMR3AreValuesValid(pCfg, "Object\0"))
        return VERR_PDM_DRVINS_UNKNOWN_CFG_VALUES;
    AssertMsgReturn(PDMDrvHlpNoAttach(pDrvIns) == VERR_PDM_NO_ATTACHED_DRIVER,
                    ("Configuration error: Not possible to attach anything to this driver!\n"),
                    VERR_PDM_DRVINS_NO_ATTACH);

    /*
     * IBase.
     */
    pDrvIns->IBase.pfnQueryInterface            = VMMDev::drvQueryInterface;

    //pData->Connector.pfnUpdateGuestStatus       = VMMDev::UpdateGuestStatus;
    //pData->Connector.pfnUpdateGuestInfo         = VMMDev::UpdateGuestInfo;
    //pData->Connector.pfnUpdateGuestCapabilities = VMMDev::UpdateGuestCapabilities;
    pData->Connector.pfnUpdateMouseCapabilities = VMMDev::UpdateMouseCapabilities;
    pData->Connector.pfnUpdatePointerShape      = VMMDev::UpdatePointerShape;
    pData->Connector.pfnVideoAccelEnable        = iface_VideoAccelEnable;
    pData->Connector.pfnVideoAccelFlush         = iface_VideoAccelFlush;
    pData->Connector.pfnVideoModeSupported      = VMMDev::VideoModeSupported;
    pData->Connector.pfnGetHeightReduction      = VMMDev::GetHeightReduction;
    pData->Connector.pfnSetVisibleRegion        = iface_SetVisibleRegion;
    pData->Connector.pfnQueryVisibleRegion      = iface_QueryVisibleRegion;
    pData->Connector.pfnQueryBalloonSize        = VMMDev::QueryBalloonSize;

#ifdef VBOX_WITH_HGCM
    if (fActivateHGCM())
    {
        pData->HGCMConnector.pfnConnect         = iface_hgcmConnect;
        pData->HGCMConnector.pfnDisconnect      = iface_hgcmDisconnect;
        pData->HGCMConnector.pfnCall            = iface_hgcmCall;
    }
#endif

    /*
     * Get the IVMMDevPort interface of the above driver/device.
     */
    pData->pUpPort = PDMIBASE_QUERY_INTERFACE(pDrvIns->pUpBase, PDMIVMMDEVPORT);
    AssertMsgReturn(pData->pUpPort, ("Configuration error: No VMMDev port interface above!\n"), VERR_PDM_MISSING_INTERFACE_ABOVE);

#ifdef VBOX_WITH_HGCM
    if (fActivateHGCM())
    {
        pData->pHGCMPort = PDMIBASE_QUERY_INTERFACE(pDrvIns->pUpBase, PDMIHGCMPORT);
        AssertMsgReturn(pData->pHGCMPort, ("Configuration error: No HGCM port interface above!\n"), VERR_PDM_MISSING_INTERFACE_ABOVE);
    }
#endif

    /*
     * Get the VMMDev object pointer and update the mpDrv member.
     */
    void *pv;
    int rc = CFGMR3QueryPtr(pCfg, "Object", &pv);
    if (RT_FAILURE(rc))
    {
        AssertMsgFailed(("Configuration error: No/bad \"Object\" value! rc=%Rrc\n", rc));
        return rc;
    }

    pData->pVMMDev = (VMMDev*)pv;        /** @todo Check this cast! */
    pData->pVMMDev->mpDrv = pData;

#ifdef VBOX_WITH_HGCM
    if (fActivateHGCM())
    {
        rc = pData->pVMMDev->hgcmLoadService (VBOXSHAREDFOLDERS_DLL, "VBoxSharedFolders");
        pData->pVMMDev->fSharedFolderActive = RT_SUCCESS(rc);
        if (RT_SUCCESS(rc))
            LogRel(("Shared Folders service loaded.\n"));
        else
            LogRel(("Failed to load Shared Folders service %Rrc\n", rc));


        rc = PDMDrvHlpSSMRegisterEx(pDrvIns, HGCM_SSM_VERSION, 4096 /* bad guess */,
                                    NULL, NULL, NULL,
                                    NULL, iface_hgcmSave, NULL,
                                    NULL, iface_hgcmLoad, NULL);
        if (RT_FAILURE(rc))
            return rc;

    }
#endif /* VBOX_WITH_HGCM */

    return VINF_SUCCESS;
}


/**
 * VMMDevice driver registration record.
 */
const PDMDRVREG VMMDev::DrvReg =
{
    /* u32Version */
    PDM_DRVREG_VERSION,
    /* szName */
    "HGCM",
    /* szRCMod */
    "",
    /* szR0Mod */
    "",
    /* pszDescription */
    "Main VMMDev driver (Main as in the API).",
    /* fFlags */
    PDM_DRVREG_FLAGS_HOST_BITS_DEFAULT,
    /* fClass. */
    PDM_DRVREG_CLASS_VMMDEV,
    /* cMaxInstances */
    ~0U,
    /* cbInstance */
    sizeof(DRVMAINVMMDEV),
    /* pfnConstruct */
    VMMDev::drvConstruct,
    /* pfnDestruct */
    VMMDev::drvDestruct,
    /* pfnRelocate */
    NULL,
    /* pfnIOCtl */
    NULL,
    /* pfnPowerOn */
    NULL,
    /* pfnReset */
    NULL,
    /* pfnSuspend */
    NULL,
    /* pfnResume */
    NULL,
    /* pfnAttach */
    NULL,
    /* pfnDetach */
    NULL,
    /* pfnPowerOff */
    NULL,
    /* pfnSoftReset */
    NULL,
    /* u32EndVersion */
    PDM_DRVREG_VERSION
};

