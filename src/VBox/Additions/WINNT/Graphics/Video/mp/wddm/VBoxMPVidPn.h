/* $Id: VBoxMPVidPn.h $ */

/** @file
 * VBox WDDM Miniport driver
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
 */

#ifndef ___VBoxMPVidPn_h___
#define ___VBoxMPVidPn_h___

#define VBOXVDPN_C_DISPLAY_HBLANK_SIZE 200
#define VBOXVDPN_C_DISPLAY_VBLANK_SIZE 180

NTSTATUS vboxVidPnCheckSourceModeInfo(const D3DKMDT_HVIDPN hDesiredVidPn,
        const D3DKMDT_VIDPN_SOURCE_MODE *pNewVidPnSourceModeInfo,
        BOOLEAN *pbSupported);

NTSTATUS vboxVidPnCheckSourceModeSet(const D3DKMDT_HVIDPN hDesiredVidPn,
        D3DKMDT_HVIDPNSOURCEMODESET hNewVidPnSourceModeSet, const DXGK_VIDPNSOURCEMODESET_INTERFACE *pVidPnSourceModeSetInterface,
        BOOLEAN *pbSupported);

NTSTATUS vboxVidPnCheckTargetModeInfo(const D3DKMDT_HVIDPN hDesiredVidPn,
        const D3DKMDT_VIDPN_TARGET_MODE *pNewVidPnTargetModeInfo,
        BOOLEAN *pbSupported);

NTSTATUS vboxVidPnCheckTargetModeSet(const D3DKMDT_HVIDPN hDesiredVidPn,
        D3DKMDT_HVIDPNTARGETMODESET hNewVidPnTargetModeSet, const DXGK_VIDPNTARGETMODESET_INTERFACE *pVidPnTargetModeSetInterface,
        BOOLEAN *pbSupported);

typedef enum
{
    VBOXVIDPNPATHITEM_STATE_NOT_EXISTS = 0,
    VBOXVIDPNPATHITEM_STATE_PRESENT,
    VBOXVIDPNPATHITEM_STATE_DISABLED
} VBOXVIDPNPATHITEM_STATE;

typedef struct VBOXVIDPNPATHITEM
{
    VBOXVIDPNPATHITEM_STATE enmState;
} VBOXVIDPNPATHITEM, *PVBOXVIDPNPATHITEM;

typedef struct VBOXVIDPNCOFUNCMODALITY
{
    NTSTATUS Status;
    PVBOXMP_DEVEXT pDevExt;
    const DXGK_VIDPN_INTERFACE* pVidPnInterface;
    CONST DXGKARG_ENUMVIDPNCOFUNCMODALITY* pEnumCofuncModalityArg;
    PVBOXWDDM_VIDEOMODES_INFO pInfos;
    UINT cPathInfos;
    PVBOXVIDPNPATHITEM apPathInfos;
} VBOXVIDPNCOFUNCMODALITY, *PVBOXVIDPNCOFUNCMODALITY;

typedef struct VBOXVIDPNCOMMIT
{
    NTSTATUS Status;
    PVBOXMP_DEVEXT pDevExt;
    const DXGK_VIDPN_INTERFACE* pVidPnInterface;
    CONST DXGKARG_COMMITVIDPN* pCommitVidPnArg;
} VBOXVIDPNCOMMIT, *PVBOXVIDPNCOMMIT;

/* !!!NOTE: The callback is responsible for releasing the path */
typedef DECLCALLBACK(BOOLEAN) FNVBOXVIDPNENUMPATHS(D3DKMDT_HVIDPNTOPOLOGY hVidPnTopology, const DXGK_VIDPNTOPOLOGY_INTERFACE* pVidPnTopologyInterface,
        const D3DKMDT_VIDPN_PRESENT_PATH *pNewVidPnPresentPathInfo, PVOID pContext);
typedef FNVBOXVIDPNENUMPATHS *PFNVBOXVIDPNENUMPATHS;

/* !!!NOTE: The callback is responsible for releasing the source mode info */
typedef DECLCALLBACK(BOOLEAN) FNVBOXVIDPNENUMSOURCEMODES(D3DKMDT_HVIDPNSOURCEMODESET hNewVidPnSourceModeSet, const DXGK_VIDPNSOURCEMODESET_INTERFACE *pVidPnSourceModeSetInterface,
        const D3DKMDT_VIDPN_SOURCE_MODE *pNewVidPnSourceModeInfo, PVOID pContext);
typedef FNVBOXVIDPNENUMSOURCEMODES *PFNVBOXVIDPNENUMSOURCEMODES;

/* !!!NOTE: The callback is responsible for releasing the target mode info */
typedef DECLCALLBACK(BOOLEAN) FNVBOXVIDPNENUMTARGETMODES(D3DKMDT_HVIDPNTARGETMODESET hNewVidPnTargetModeSet, const DXGK_VIDPNTARGETMODESET_INTERFACE *pVidPnTargetModeSetInterface,
        const D3DKMDT_VIDPN_TARGET_MODE *pNewVidPnTargetModeInfo, PVOID pContext);
typedef FNVBOXVIDPNENUMTARGETMODES *PFNVBOXVIDPNENUMTARGETMODES;

/* !!!NOTE: The callback is responsible for releasing the source mode info */
typedef DECLCALLBACK(BOOLEAN) FNVBOXVIDPNENUMMONITORSOURCEMODES(D3DKMDT_HMONITORSOURCEMODESET hMonitorSMS, CONST DXGK_MONITORSOURCEMODESET_INTERFACE *pMonitorSMSIf,
        CONST D3DKMDT_MONITOR_SOURCE_MODE *pMonitorSMI, PVOID pContext);
typedef FNVBOXVIDPNENUMMONITORSOURCEMODES *PFNVBOXVIDPNENUMMONITORSOURCEMODES;

typedef DECLCALLBACK(BOOLEAN) FNVBOXVIDPNENUMTARGETSFORSOURCE(PVBOXMP_DEVEXT pDevExt, D3DKMDT_HVIDPNTOPOLOGY hVidPnTopology, const DXGK_VIDPNTOPOLOGY_INTERFACE* pVidPnTopologyInterface,
        CONST D3DDDI_VIDEO_PRESENT_SOURCE_ID VidPnSourceId, D3DDDI_VIDEO_PRESENT_TARGET_ID VidPnTargetId, SIZE_T cTgtPaths, PVOID pContext);
typedef FNVBOXVIDPNENUMTARGETSFORSOURCE *PFNVBOXVIDPNENUMTARGETSFORSOURCE;

DECLCALLBACK(BOOLEAN) vboxVidPnCofuncModalityPathEnum(D3DKMDT_HVIDPNTOPOLOGY hVidPnTopology, const DXGK_VIDPNTOPOLOGY_INTERFACE* pVidPnTopologyInterface,
        const D3DKMDT_VIDPN_PRESENT_PATH *pNewVidPnPresentPathInfo, PVOID pContext);

DECLCALLBACK(BOOLEAN) vboxVidPnCofuncModalitySourceModeEnum(PVBOXMP_DEVEXT pDevExt, const D3DKMDT_HVIDPN hDesiredVidPn, const DXGK_VIDPN_INTERFACE* pVidPnInterface,
        D3DKMDT_HVIDPNSOURCEMODESET hNewVidPnSourceModeSet, const DXGK_VIDPNSOURCEMODESET_INTERFACE *pVidPnSourceModeSetInterface,
        const D3DKMDT_VIDPN_SOURCE_MODE *pNewVidPnSourceModeInfo, PVOID pContext);

DECLCALLBACK(BOOLEAN) vboxVidPnCofuncModalityTargetModeEnum(PVBOXMP_DEVEXT pDevExt, const D3DKMDT_HVIDPN hDesiredVidPn, const DXGK_VIDPN_INTERFACE* pVidPnInterface,
        D3DKMDT_HVIDPNTARGETMODESET hNewVidPnTargetModeSet, const DXGK_VIDPNTARGETMODESET_INTERFACE *pVidPnTargetModeSetInterface,
        const D3DKMDT_VIDPN_TARGET_MODE *pNewVidPnTargetModeInfo, PVOID pContext);

DECLCALLBACK(BOOLEAN) vboxVidPnCommitPathEnum(D3DKMDT_HVIDPNTOPOLOGY hVidPnTopology, const DXGK_VIDPNTOPOLOGY_INTERFACE* pVidPnTopologyInterface,
        const D3DKMDT_VIDPN_PRESENT_PATH *pNewVidPnPresentPathInfo, PVOID pContext);

NTSTATUS vboxVidPnCommitSourceModeForSrcId(PVBOXMP_DEVEXT pDevExt, const D3DKMDT_HVIDPN hDesiredVidPn, const DXGK_VIDPN_INTERFACE* pVidPnInterface, D3DDDI_VIDEO_PRESENT_SOURCE_ID srcId, struct VBOXWDDM_ALLOCATION *pAllocation);

NTSTATUS vboxVidPnEnumPaths(D3DKMDT_HVIDPNTOPOLOGY hVidPnTopology, const DXGK_VIDPNTOPOLOGY_INTERFACE* pVidPnTopologyInterface,
        PFNVBOXVIDPNENUMPATHS pfnCallback, PVOID pContext);

NTSTATUS vboxVidPnEnumSourceModes(D3DKMDT_HVIDPNSOURCEMODESET hNewVidPnSourceModeSet, const DXGK_VIDPNSOURCEMODESET_INTERFACE *pVidPnSourceModeSetInterface,
        PFNVBOXVIDPNENUMSOURCEMODES pfnCallback, PVOID pContext);

NTSTATUS vboxVidPnEnumTargetModes(D3DKMDT_HVIDPNTARGETMODESET hNewVidPnTargetModeSet, const DXGK_VIDPNTARGETMODESET_INTERFACE *pVidPnTargetModeSetInterface,
        PFNVBOXVIDPNENUMTARGETMODES pfnCallback, PVOID pContext);

NTSTATUS vboxVidPnEnumMonitorSourceModes(D3DKMDT_HMONITORSOURCEMODESET hMonitorSMS, CONST DXGK_MONITORSOURCEMODESET_INTERFACE *pMonitorSMSIf,
        PFNVBOXVIDPNENUMMONITORSOURCEMODES pfnCallback, PVOID pContext);

NTSTATUS vboxVidPnEnumTargetsForSource(PVBOXMP_DEVEXT pDevExt, D3DKMDT_HVIDPNTOPOLOGY hVidPnTopology, const DXGK_VIDPNTOPOLOGY_INTERFACE* pVidPnTopologyInterface,
        CONST D3DDDI_VIDEO_PRESENT_SOURCE_ID VidPnSourceId,
        PFNVBOXVIDPNENUMTARGETSFORSOURCE pfnCallback, PVOID pContext);

NTSTATUS vboxVidPnPopulateMonitorSourceModeInfoFromLegacy(PVBOXMP_DEVEXT pDevExt,
        D3DKMDT_MONITOR_SOURCE_MODE *pMonitorSourceMode,
        D3DKMDT_2DREGION *pResolution,
        D3DKMDT_MONITOR_CAPABILITIES_ORIGIN enmOrigin,
        BOOLEAN bPreferred);

NTSTATUS vboxVidPnCreatePopulateVidPnPathFromLegacy(PVBOXMP_DEVEXT pDevExt, D3DKMDT_HVIDPN hVidPn, const DXGK_VIDPN_INTERFACE* pVidPnInterface,
        VIDEO_MODE_INFORMATION *pModes, uint32_t cModes, int iPreferredMode,
        D3DKMDT_2DREGION *pResolutions, uint32_t cResolutions,
        const D3DDDI_VIDEO_PRESENT_SOURCE_ID VidPnSourceId, const D3DDDI_VIDEO_PRESENT_TARGET_ID VidPnTargetId);

NTSTATUS vboxVidPnCheckAddMonitorModes(PVBOXMP_DEVEXT pDevExt,
        D3DDDI_VIDEO_PRESENT_TARGET_ID targetId, D3DKMDT_MONITOR_CAPABILITIES_ORIGIN enmOrigin,
        D3DKMDT_2DREGION *pResolutions, uint32_t cResolutions, int iPreferred);

NTSTATUS vboxVidPnMatchMonitorModes(PVBOXMP_DEVEXT pDevExt, D3DDDI_VIDEO_PRESENT_TARGET_ID targetId,
        D3DKMDT_2DREGION *pResolutions, uint32_t cResolutions, BOOLEAN *pfMatch);

NTSTATUS vboxVidPnCofuncModalityForPath(PVBOXVIDPNCOFUNCMODALITY pCbContext, D3DDDI_VIDEO_PRESENT_SOURCE_ID VidPnSourceId, D3DDDI_VIDEO_PRESENT_TARGET_ID VidPnTargetId);

NTSTATUS vboxVidPnCheckTopology(D3DKMDT_HVIDPNTOPOLOGY hVidPnTopology, const DXGK_VIDPNTOPOLOGY_INTERFACE* pVidPnTopologyInterface,
                                    BOOLEAN fBreakOnDisabled, UINT cItems, PVBOXVIDPNPATHITEM paItems, BOOLEAN *pfDisabledFound);

NTSTATUS vboxVidPnPathAdd(D3DKMDT_HVIDPN hVidPn, const DXGK_VIDPN_INTERFACE* pVidPnInterface,
        const D3DDDI_VIDEO_PRESENT_SOURCE_ID VidPnSourceId, const D3DDDI_VIDEO_PRESENT_TARGET_ID VidPnTargetId);

void vboxVidPnDumpVidPn(const char * pPrefix, PVBOXMP_DEVEXT pDevExt, D3DKMDT_HVIDPN hVidPn, const DXGK_VIDPN_INTERFACE* pVidPnInterface, const char * pSuffix);
void vboxVidPnDumpCofuncModalityArg(const char *pPrefix, CONST DXGKARG_ENUMVIDPNCOFUNCMODALITY* CONST  pEnumCofuncModalityArg, const char *pSuffix);
DECLCALLBACK(BOOLEAN) vboxVidPnDumpSourceModeSetEnum(D3DKMDT_HVIDPNSOURCEMODESET hNewVidPnSourceModeSet, const DXGK_VIDPNSOURCEMODESET_INTERFACE *pVidPnSourceModeSetInterface,
        const D3DKMDT_VIDPN_SOURCE_MODE *pNewVidPnSourceModeInfo, PVOID pContext);
DECLCALLBACK(BOOLEAN) vboxVidPnDumpTargetModeSetEnum(D3DKMDT_HVIDPNTARGETMODESET hNewVidPnTargetModeSet, const DXGK_VIDPNTARGETMODESET_INTERFACE *pVidPnTargetModeSetInterface,
        const D3DKMDT_VIDPN_TARGET_MODE *pNewVidPnTargetModeInfo, PVOID pContext);

#endif /* #ifndef ___VBoxMPVidPn_h___ */
