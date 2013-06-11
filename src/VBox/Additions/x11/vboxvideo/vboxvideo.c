/* $Id: vboxvideo.c $ */
/** @file
 *
 * Linux Additions X11 graphics driver
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
 * --------------------------------------------------------------------
 *
 * This code is based on the X.Org VESA driver with the following copyrights:
 *
 * Copyright (c) 2000 by Conectiva S.A. (http://www.conectiva.com)
 * Copyright 2008 Red Hat, Inc.
 * Copyright 2012 Red Hat, Inc.
 *
 * and the following permission notice (not all original sourse files include
 * the last paragraph):
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * CONECTIVA LINUX BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Except as contained in this notice, the name of Conectiva Linux shall
 * not be used in advertising or otherwise to promote the sale, use or other
 * dealings in this Software without prior written authorization from
 * Conectiva Linux.
 *
 * Authors: Paulo César Pereira de Andrade <pcpa@conectiva.com.br>
 *          David Dawes <dawes@xfree86.org>
 *          Adam Jackson <ajax@redhat.com>
 *          Dave Airlie <airlied@redhat.com>
 */

#ifdef XORG_7X
# include <stdlib.h>
# include <string.h>
#endif

#include "xf86.h"
#include "xf86_OSproc.h"
#if GET_ABI_MAJOR(ABI_VIDEODRV_VERSION) < 6
# include "xf86Resources.h"
#endif

#ifndef PCIACCESS
/* Drivers for PCI hardware need this */
# include "xf86PciInfo.h"
/* Drivers that need to access the PCI config space directly need this */
# include "xf86Pci.h"
#endif

#include "fb.h"

#include "vboxvideo.h"
#include <iprt/asm-math.h>
#include "version-generated.h"
#include "product-generated.h"
#include <xf86.h>
#include <misc.h>

/* All drivers initialising the SW cursor need this */
#include "mipointer.h"

/* Colormap handling */
#include "micmap.h"
#include "xf86cmap.h"

/* DPMS */
/* #define DPMS_SERVER
#include "extensions/dpms.h" */

/* VGA hardware functions for setting and restoring text mode */
#include "vgaHW.h"

#ifdef VBOXVIDEO_13
/* X.org 1.3+ mode setting */
# define _HAVE_STRING_ARCH_strsep /* bits/string2.h, __strsep_1c. */
# include "xf86Crtc.h"
# include "xf86Modes.h"
# include <X11/Xatom.h>
#endif

/* Mandatory functions */

static const OptionInfoRec * VBOXAvailableOptions(int chipid, int busid);
static void VBOXIdentify(int flags);
#ifndef PCIACCESS
static Bool VBOXProbe(DriverPtr drv, int flags);
#else
static Bool VBOXPciProbe(DriverPtr drv, int entity_num,
     struct pci_device *dev, intptr_t match_data);
#endif
static Bool VBOXPreInit(ScrnInfoPtr pScrn, int flags);
static Bool VBOXScreenInit(ScreenPtr pScreen, int argc, char **argv);
static Bool VBOXEnterVT(ScrnInfoPtr pScrn);
static void VBOXLeaveVT(ScrnInfoPtr pScrn);
static Bool VBOXCloseScreen(ScreenPtr pScreen);
static Bool VBOXSaveScreen(ScreenPtr pScreen, int mode);
static Bool VBOXSwitchMode(ScrnInfoPtr pScrn, DisplayModePtr pMode);
static void VBOXAdjustFrame(ScrnInfoPtr pScrn, int x, int y);
static void VBOXFreeScreen(ScrnInfoPtr pScrn);
static void VBOXDisplayPowerManagementSet(ScrnInfoPtr pScrn, int mode,
                                          int flags);

/* locally used functions */
static Bool VBOXMapVidMem(ScrnInfoPtr pScrn);
static void VBOXUnmapVidMem(ScrnInfoPtr pScrn);
static void VBOXSaveMode(ScrnInfoPtr pScrn);
static void VBOXRestoreMode(ScrnInfoPtr pScrn);

static inline void VBOXSetRec(ScrnInfoPtr pScrn)
{
    if (!pScrn->driverPrivate)
        pScrn->driverPrivate = calloc(sizeof(VBOXRec), 1);
}

enum GenericTypes
{
    CHIP_VBOX_GENERIC
};

#ifdef PCIACCESS
static const struct pci_id_match vbox_device_match[] = {
    {
        VBOX_VENDORID, VBOX_DEVICEID, PCI_MATCH_ANY, PCI_MATCH_ANY,
        0, 0, 0
    },

    { 0, 0, 0 },
};
#endif

/* Supported chipsets */
static SymTabRec VBOXChipsets[] =
{
    {VBOX_DEVICEID, "vbox"},
    {-1,	 NULL}
};

static PciChipsets VBOXPCIchipsets[] = {
  { VBOX_DEVICEID, VBOX_DEVICEID, RES_SHARED_VGA },
  { -1,		-1,		    RES_UNDEFINED },
};

/*
 * This contains the functions needed by the server after loading the
 * driver module.  It must be supplied, and gets added the driver list by
 * the Module Setup function in the dynamic case.  In the static case a
 * reference to this is compiled in, and this requires that the name of
 * this DriverRec be an upper-case version of the driver name.
 */

#ifdef XORG_7X
_X_EXPORT
#endif
DriverRec VBOXVIDEO = {
    VBOX_VERSION,
    VBOX_DRIVER_NAME,
    VBOXIdentify,
#ifdef PCIACCESS
    NULL,
#else
    VBOXProbe,
#endif
    VBOXAvailableOptions,
    NULL,
    0,
#ifdef XORG_7X
    NULL,
#endif
#ifdef PCIACCESS
    vbox_device_match,
    VBOXPciProbe
#endif
};

/* No options for now */
static const OptionInfoRec VBOXOptions[] = {
    { -1,		NULL,		OPTV_NONE,	{0},	FALSE }
};

#ifndef XORG_7X
/*
 * List of symbols from other modules that this module references.  This
 * list is used to tell the loader that it is OK for symbols here to be
 * unresolved providing that it hasn't been told that they haven't been
 * told that they are essential via a call to xf86LoaderReqSymbols() or
 * xf86LoaderReqSymLists().  The purpose is this is to avoid warnings about
 * unresolved symbols that are not required.
 */
static const char *fbSymbols[] = {
    "fbPictureInit",
    "fbScreenInit",
    NULL
};

static const char *shadowfbSymbols[] = {
    "ShadowFBInit2",
    NULL
};

static const char *ramdacSymbols[] = {
    "xf86InitCursor",
    "xf86CreateCursorInfoRec",
    NULL
};

static const char *vgahwSymbols[] = {
    "vgaHWFreeHWRec",
    "vgaHWGetHWRec",
    "vgaHWGetIOBase",
    "vgaHWGetIndex",
    "vgaHWRestore",
    "vgaHWSave",
    "vgaHWSetStdFuncs",
    NULL
};
#endif /* !XORG_7X */

#ifdef VBOXVIDEO_13
/* X.org 1.3+ mode-setting support ******************************************/

/* For descriptions of these functions and structures, see
   hw/xfree86/modes/xf86Crtc.h and hw/xfree86/modes/xf86Modes.h in the
   X.Org source tree. */

static Bool vbox_config_resize(ScrnInfoPtr pScrn, int cw, int ch)
{
    VBOXPtr pVBox = VBOXGetRec(pScrn);
    TRACE_LOG("width=%d, height=%d\n", cw, ch);
    /* Save the size in case we need to re-set it later. */
    pVBox->FBSize.cx = cw;
    pVBox->FBSize.cy = ch;
    /* Don't fiddle with the hardware if we are switched
     * to a virtual terminal. */
    if (!pScrn->vtSema) {
        xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
                   "We do not own the active VT, exiting.\n");
        return TRUE;
    }
    return VBOXAdjustScreenPixmap(pScrn, cw, ch);
}

static const xf86CrtcConfigFuncsRec VBOXCrtcConfigFuncs = {
    vbox_config_resize
};

static void
vbox_crtc_dpms(xf86CrtcPtr crtc, int mode)
{
    VBOXPtr pVBox = VBOXGetRec(crtc->scrn);
    unsigned cDisplay = (uintptr_t)crtc->driver_private;
    TRACE_LOG("cDisplay=%u, mode=%i\n", cDisplay, mode);
    pVBox->afDisabled[cDisplay] = (mode != DPMSModeOn);
    /* Don't fiddle with the hardware if we are switched
     * to a virtual terminal. */
    if (!crtc->scrn->vtSema) {
        xf86DrvMsg(crtc->scrn->scrnIndex, X_ERROR,
                   "We do not own the active VT, exiting.\n");
        return;
    }
    if (   pVBox->aScreenLocation[cDisplay].cx
        && pVBox->aScreenLocation[cDisplay].cy)
        VBOXSetMode(crtc->scrn, cDisplay,
                    pVBox->aScreenLocation[cDisplay].cx,
                    pVBox->aScreenLocation[cDisplay].cy,
                    pVBox->aScreenLocation[cDisplay].x,
                    pVBox->aScreenLocation[cDisplay].y);
}

static Bool
vbox_crtc_lock (xf86CrtcPtr crtc)
{ (void) crtc; return FALSE; }


/* We use this function to check whether the X server owns the active virtual
 * terminal before attempting a mode switch, since the RandR extension isn't
 * very dilligent here, which can mean crashes if we are unlucky.  This is
 * not the way it the function is intended - it is meant for reporting modes
 * which the hardware can't handle.  I hope that this won't confuse any clients
 * connecting to us. */
static Bool
vbox_crtc_mode_fixup (xf86CrtcPtr crtc, DisplayModePtr mode,
                      DisplayModePtr adjusted_mode)
{ (void) crtc; (void) mode; (void) adjusted_mode; return TRUE; }

static void
vbox_crtc_stub (xf86CrtcPtr crtc)
{ (void) crtc; }

static void
vbox_crtc_mode_set (xf86CrtcPtr crtc, DisplayModePtr mode,
                    DisplayModePtr adjusted_mode, int x, int y)
{
    (void) mode;
    VBOXPtr pVBox = VBOXGetRec(crtc->scrn);
    unsigned cDisplay = (uintptr_t)crtc->driver_private;

    TRACE_LOG("name=%s, HDisplay=%d, VDisplay=%d, x=%d, y=%d\n", adjusted_mode->name,
           adjusted_mode->HDisplay, adjusted_mode->VDisplay, x, y);
    pVBox->afDisabled[cDisplay] = false;
    pVBox->aScreenLocation[cDisplay].cx = adjusted_mode->HDisplay;
    pVBox->aScreenLocation[cDisplay].cy = adjusted_mode->VDisplay;
    pVBox->aScreenLocation[cDisplay].x = x;
    pVBox->aScreenLocation[cDisplay].y = y;
    /* Don't remember any modes set while we are seamless, as they are
     * just temporary. */
    if (!vboxGuestIsSeamless(crtc->scrn))
        vboxSaveVideoMode(crtc->scrn, adjusted_mode->HDisplay,
                          adjusted_mode->VDisplay, crtc->scrn->bitsPerPixel);
    /* Don't fiddle with the hardware if we are switched
     * to a virtual terminal. */
    if (!crtc->scrn->vtSema)
    {
        xf86DrvMsg(crtc->scrn->scrnIndex, X_ERROR,
                   "We do not own the active VT, exiting.\n");
        return;
    }
    VBOXSetMode(crtc->scrn, cDisplay, adjusted_mode->HDisplay,
                adjusted_mode->VDisplay, x, y);
}

static void
vbox_crtc_gamma_set (xf86CrtcPtr crtc, CARD16 *red,
                     CARD16 *green, CARD16 *blue, int size)
{ (void) crtc; (void) red; (void) green; (void) blue; (void) size; }

static void *
vbox_crtc_shadow_allocate (xf86CrtcPtr crtc, int width, int height)
{ (void) crtc; (void) width; (void) height; return NULL; }

static const xf86CrtcFuncsRec VBOXCrtcFuncs = {
    .dpms = vbox_crtc_dpms,
    .save = NULL, /* These two are never called by the server. */
    .restore = NULL,
    .lock = vbox_crtc_lock,
    .unlock = NULL, /* This will not be invoked if lock returns FALSE. */
    .mode_fixup = vbox_crtc_mode_fixup,
    .prepare = vbox_crtc_stub,
    .mode_set = vbox_crtc_mode_set,
    .commit = vbox_crtc_stub,
    .gamma_set = vbox_crtc_gamma_set,
    .shadow_allocate = vbox_crtc_shadow_allocate,
    .shadow_create = NULL, /* These two should not be invoked if allocate
                              returns NULL. */
    .shadow_destroy = NULL,
    .set_cursor_colors = NULL, /* We are still using the old cursor API. */
    .set_cursor_position = NULL,
    .show_cursor = NULL,
    .hide_cursor = NULL,
    .load_cursor_argb = NULL,
    .destroy = vbox_crtc_stub
};

static void
vbox_output_stub (xf86OutputPtr output)
{ (void) output; }

static void
vbox_output_dpms (xf86OutputPtr output, int mode)
{ (void) output; (void) mode; }

static int
vbox_output_mode_valid (xf86OutputPtr output, DisplayModePtr mode)
{
    ScrnInfoPtr pScrn = output->scrn;
    int rc = MODE_OK;
    TRACE_LOG("HDisplay=%d, VDisplay=%d\n", mode->HDisplay, mode->VDisplay);
    /* We always like modes specified by the user in the configuration
     * file and modes requested by the host, as doing otherwise is likely to
	 * annoy people. */
    if (   !(mode->type & M_T_USERDEF)
        && !(mode->type & M_T_PREFERRED)
        && vbox_device_available(VBOXGetRec(pScrn))
        && !vboxHostLikesVideoMode(pScrn, mode->HDisplay, mode->VDisplay,
                                   pScrn->bitsPerPixel)
       )
        rc = MODE_BAD;
    TRACE_LOG("returning %s\n", MODE_OK == rc ? "MODE_OK" : "MODE_BAD");
    return rc;
}

static Bool
vbox_output_mode_fixup (xf86OutputPtr output, DisplayModePtr mode,
                        DisplayModePtr adjusted_mode)
{ (void) output; (void) mode; (void) adjusted_mode; return TRUE; }

static void
vbox_output_mode_set (xf86OutputPtr output, DisplayModePtr mode,
                        DisplayModePtr adjusted_mode)
{ (void) output; (void) mode; (void) adjusted_mode; }

/* A virtual monitor is always connected. */
static xf86OutputStatus
vbox_output_detect (xf86OutputPtr output)
{
    (void) output;
    return XF86OutputStatusConnected;
}

static DisplayModePtr
vbox_output_add_mode (VBOXPtr pVBox, DisplayModePtr *pModes,
                      const char *pszName, int x, int y,
                      Bool isPreferred, Bool isUserDef)
{
    TRACE_LOG("pszName=%s, x=%d, y=%d\n", pszName, x, y);
    DisplayModePtr pMode = xnfcalloc(1, sizeof(DisplayModeRec));

    pMode->status        = MODE_OK;
    /* We don't ask the host whether it likes user defined modes,
     * as we assume that the user really wanted that mode. */
    pMode->type          = isUserDef ? M_T_USERDEF : M_T_BUILTIN;
    if (isPreferred)
        pMode->type     |= M_T_PREFERRED;
    /* Older versions of VBox only support screen widths which are a multiple
     * of 8 */
    if (pVBox->fAnyX)
        pMode->HDisplay  = x;
    else
        pMode->HDisplay  = x & ~7;
    pMode->HSyncStart    = pMode->HDisplay + 2;
    pMode->HSyncEnd      = pMode->HDisplay + 4;
    pMode->HTotal        = pMode->HDisplay + 6;
    pMode->VDisplay      = y;
    pMode->VSyncStart    = pMode->VDisplay + 2;
    pMode->VSyncEnd      = pMode->VDisplay + 4;
    pMode->VTotal        = pMode->VDisplay + 6;
    pMode->Clock         = pMode->HTotal * pMode->VTotal * 60 / 1000; /* kHz */
    if (NULL == pszName) {
        xf86SetModeDefaultName(pMode);
    } else {
        pMode->name          = xnfstrdup(pszName);
    }
    *pModes = xf86ModesAdd(*pModes, pMode);
    return pMode;
}

static DisplayModePtr
vbox_output_get_modes (xf86OutputPtr output)
{
    unsigned i, cIndex = 0;
    DisplayModePtr pModes = NULL, pMode;
    ScrnInfoPtr pScrn = output->scrn;
    VBOXPtr pVBox = VBOXGetRec(pScrn);

    TRACE_ENTRY();
    uint32_t x, y, bpp, iScreen;
    iScreen = (uintptr_t)output->driver_private;
    vboxGetPreferredMode(pScrn, iScreen, &x, &y, &bpp);
    pMode = vbox_output_add_mode(pVBox, &pModes, NULL, x, y, TRUE, FALSE);
    VBOXEDIDSet(output, pMode);
    /* Add standard modes supported by the host */
    for ( ; ; )
    {
        cIndex = vboxNextStandardMode(pScrn, cIndex, &x, &y, NULL);
        if (cIndex == 0)
            break;
        vbox_output_add_mode(pVBox, &pModes, NULL, x, y, FALSE, FALSE);
    }

    /* Also report any modes the user may have requested in the xorg.conf
     * configuration file. */
    for (i = 0; pScrn->display->modes[i] != NULL; i++)
    {
        if (2 == sscanf(pScrn->display->modes[i], "%ux%u", &x, &y))
            vbox_output_add_mode(pVBox, &pModes, pScrn->display->modes[i], x, y,
                                 FALSE, TRUE);
    }
    TRACE_EXIT();
    return pModes;
}

#ifdef RANDR_12_INTERFACE
static Atom
vboxAtomVBoxMode(void)
{
    return MakeAtom("VBOX_MODE", sizeof("VBOX_MODE") - 1, TRUE);
}

static Atom
vboxAtomEDID(void)
{
    return MakeAtom("EDID", sizeof("EDID") - 1, TRUE);
}

/** We use this for receiving information from clients for the purpose of
 * dynamic resizing, and later possibly other things too.
 */
static Bool
vbox_output_set_property(xf86OutputPtr output, Atom property,
                         RRPropertyValuePtr value)
{
    ScrnInfoPtr pScrn = output->scrn;
    VBOXPtr pVBox = VBOXGetRec(pScrn);
    TRACE_LOG("property=%d, value->type=%d, value->format=%d, value->size=%ld\n",
              (int)property, (int)value->type, value->format, value->size);
    if (property == vboxAtomVBoxMode())
    {
        uint32_t cDisplay = (uintptr_t)output->driver_private;
        char sz[256] = { 0 };
        int w, h;

        if (   value->type != XA_STRING
            || (unsigned) value->size > (sizeof(sz) - 1))
            return FALSE;
        strncpy(sz, value->data, value->size);
        TRACE_LOG("screen=%u, property value=%s\n", cDisplay, sz);
        if (sscanf(sz, "%dx%d", &w, &h) != 2)
            return FALSE;
        pVBox->aPreferredSize[cDisplay].cx = w;
        pVBox->aPreferredSize[cDisplay].cy = h;
        return TRUE;
    }
    if (property == vboxAtomEDID())
        return TRUE;
    return FALSE;
}
#endif

static const xf86OutputFuncsRec VBOXOutputFuncs = {
    .create_resources = vbox_output_stub,
    .dpms = vbox_output_dpms,
    .save = NULL, /* These two are never called by the server. */
    .restore = NULL,
    .mode_valid = vbox_output_mode_valid,
    .mode_fixup = vbox_output_mode_fixup,
    .prepare = vbox_output_stub,
    .commit = vbox_output_stub,
    .mode_set = vbox_output_mode_set,
    .detect = vbox_output_detect,
    .get_modes = vbox_output_get_modes,
#ifdef RANDR_12_INTERFACE
     .set_property = vbox_output_set_property,
#endif
    .destroy = vbox_output_stub
};
#endif /* VBOXVIDEO_13 */

/* Module loader interface */
static MODULESETUPPROTO(vboxSetup);

static XF86ModuleVersionInfo vboxVersionRec =
{
    VBOX_DRIVER_NAME,
    VBOX_VENDOR,
    MODINFOSTRING1,
    MODINFOSTRING2,
#ifdef XORG_7X
    XORG_VERSION_CURRENT,
#else
    XF86_VERSION_CURRENT,
#endif
    1,                          /* Module major version. Xorg-specific */
    0,                          /* Module minor version. Xorg-specific */
    1,                          /* Module patchlevel. Xorg-specific */
    ABI_CLASS_VIDEODRV,	        /* This is a video driver */
    ABI_VIDEODRV_VERSION,
    MOD_CLASS_VIDEODRV,
    {0, 0, 0, 0}
};

/*
 * This data is accessed by the loader.  The name must be the module name
 * followed by "ModuleData".
 */
#ifdef XORG_7X
_X_EXPORT
#endif
XF86ModuleData vboxvideoModuleData = { &vboxVersionRec, vboxSetup, NULL };

static pointer
vboxSetup(pointer Module, pointer Options, int *ErrorMajor, int *ErrorMinor)
{
    static Bool Initialised = FALSE;

    if (!Initialised)
    {
        Initialised = TRUE;
#ifdef PCIACCESS
        xf86AddDriver(&VBOXVIDEO, Module, HaveDriverFuncs);
#else
        xf86AddDriver(&VBOXVIDEO, Module, 0);
#endif
#ifndef XORG_7X
        LoaderRefSymLists(fbSymbols,
                          shadowfbSymbols,
                          ramdacSymbols,
                          vgahwSymbols,
                          NULL);
#endif
        xf86Msg(X_CONFIG, "Load address of symbol \"VBOXVIDEO\" is %p\n",
                (void *)&VBOXVIDEO);
        return (pointer)TRUE;
    }

    if (ErrorMajor)
        *ErrorMajor = LDR_ONCEONLY;
    return (NULL);
}


static const OptionInfoRec *
VBOXAvailableOptions(int chipid, int busid)
{
    return (VBOXOptions);
}

static void
VBOXIdentify(int flags)
{
    xf86PrintChipsets(VBOX_NAME, "guest driver for VirtualBox", VBOXChipsets);
}

#ifndef XF86_SCRN_INTERFACE
# define xf86ScreenToScrn(pScreen) xf86Screens[(pScreen)->myNum]
# define xf86ScrnToScreen(pScrn) screenInfo.screens[(pScrn)->scrnIndex]
# define SCRNINDEXAPI(pfn) pfn ## Index
static Bool VBOXScreenInitIndex(int scrnIndex, ScreenPtr pScreen, int argc,
                                char **argv)
{ return VBOXScreenInit(pScreen, argc, argv); }

static Bool VBOXEnterVTIndex(int scrnIndex, int flags)
{ (void) flags; return VBOXEnterVT(xf86Screens[scrnIndex]); }

static void VBOXLeaveVTIndex(int scrnIndex, int flags)
{ (void) flags; VBOXLeaveVT(xf86Screens[scrnIndex]); }

static Bool VBOXCloseScreenIndex(int scrnIndex, ScreenPtr pScreen)
{ (void) scrnIndex; return VBOXCloseScreen(pScreen); }

static Bool VBOXSwitchModeIndex(int scrnIndex, DisplayModePtr pMode, int flags)
{ (void) flags; return VBOXSwitchMode(xf86Screens[scrnIndex], pMode); }

static void VBOXAdjustFrameIndex(int scrnIndex, int x, int y, int flags)
{ (void) flags; VBOXAdjustFrame(xf86Screens[scrnIndex], x, y); }

static void VBOXFreeScreenIndex(int scrnIndex, int flags)
{ (void) flags; VBOXFreeScreen(xf86Screens[scrnIndex]); }
# else
# define SCRNINDEXAPI(pfn) pfn
#endif /* XF86_SCRN_INTERFACE */

static void setScreenFunctions(ScrnInfoPtr pScrn, xf86ProbeProc pfnProbe)
{
    pScrn->driverVersion = VBOX_VERSION;
    pScrn->driverName    = VBOX_DRIVER_NAME;
    pScrn->name          = VBOX_NAME;
    pScrn->Probe         = pfnProbe;
    pScrn->PreInit       = VBOXPreInit;
    pScrn->ScreenInit    = SCRNINDEXAPI(VBOXScreenInit);
    pScrn->SwitchMode    = SCRNINDEXAPI(VBOXSwitchMode);
    pScrn->AdjustFrame   = SCRNINDEXAPI(VBOXAdjustFrame);
    pScrn->EnterVT       = SCRNINDEXAPI(VBOXEnterVT);
    pScrn->LeaveVT       = SCRNINDEXAPI(VBOXLeaveVT);
    pScrn->FreeScreen    = SCRNINDEXAPI(VBOXFreeScreen);
}

/*
 * One of these functions is called once, at the start of the first server
 * generation to do a minimal probe for supported hardware.
 */

#ifdef PCIACCESS
static Bool
VBOXPciProbe(DriverPtr drv, int entity_num, struct pci_device *dev,
             intptr_t match_data)
{
    ScrnInfoPtr pScrn;

    TRACE_ENTRY();
    pScrn = xf86ConfigPciEntity(NULL, 0, entity_num, VBOXPCIchipsets,
                                NULL, NULL, NULL, NULL, NULL);
    if (pScrn != NULL) {
        VBOXPtr pVBox;

        VBOXSetRec(pScrn);
        pVBox = VBOXGetRec(pScrn);
        if (!pVBox)
            return FALSE;
        setScreenFunctions(pScrn, NULL);
        pVBox->pciInfo = dev;
    }

    TRACE_LOG("returning %s\n", BOOL_STR(pScrn != NULL));
    return (pScrn != NULL);
}
#endif

#ifndef PCIACCESS
static Bool
VBOXProbe(DriverPtr drv, int flags)
{
    Bool foundScreen = FALSE;
    int numDevSections;
    GDevPtr *devSections;

    /*
     * Find the config file Device sections that match this
     * driver, and return if there are none.
     */
    if ((numDevSections = xf86MatchDevice(VBOX_NAME,
                      &devSections)) <= 0)
    return (FALSE);

    /* PCI BUS */
    if (xf86GetPciVideoInfo())
    {
        int numUsed;
        int *usedChips;
        int i;
        numUsed = xf86MatchPciInstances(VBOX_NAME, VBOX_VENDORID,
                        VBOXChipsets, VBOXPCIchipsets,
                        devSections, numDevSections,
                        drv, &usedChips);
        if (numUsed > 0)
        {
            if (flags & PROBE_DETECT)
                foundScreen = TRUE;
            else
                for (i = 0; i < numUsed; i++)
                {
                    ScrnInfoPtr pScrn = NULL;
                    /* Allocate a ScrnInfoRec  */
                    if ((pScrn = xf86ConfigPciEntity(pScrn,0,usedChips[i],
                                     VBOXPCIchipsets,NULL,
                                     NULL,NULL,NULL,NULL)))
                    {
                        setScreenFunctions(pScrn, VBOXProbe);
                        foundScreen = TRUE;
                    }
                }
            free(usedChips);
        }
    }
    free(devSections);
    return (foundScreen);
}
#endif


/*
 * QUOTE from the XFree86 DESIGN document:
 *
 * The purpose of this function is to find out all the information
 * required to determine if the configuration is usable, and to initialise
 * those parts of the ScrnInfoRec that can be set once at the beginning of
 * the first server generation.
 *
 * (...)
 *
 * This includes probing for video memory, clocks, ramdac, and all other
 * HW info that is needed. It includes determining the depth/bpp/visual
 * and related info. It includes validating and determining the set of
 * video modes that will be used (and anything that is required to
 * determine that).
 *
 * This information should be determined in the least intrusive way
 * possible. The state of the HW must remain unchanged by this function.
 * Although video memory (including MMIO) may be mapped within this
 * function, it must be unmapped before returning.
 *
 * END QUOTE
 */

static Bool
VBOXPreInit(ScrnInfoPtr pScrn, int flags)
{
    VBOXPtr pVBox;
    Gamma gzeros = {0.0, 0.0, 0.0};
    rgb rzeros = {0, 0, 0};
    unsigned DispiId;

    TRACE_ENTRY();
    /* Are we really starting the server, or is this just a dummy run? */
    if (flags & PROBE_DETECT)
        return (FALSE);

    xf86DrvMsg(pScrn->scrnIndex, X_INFO,
               "VirtualBox guest additions video driver version "
               VBOX_VERSION_STRING "\n");

    /* Get our private data from the ScrnInfoRec structure. */
    VBOXSetRec(pScrn);
    pVBox = VBOXGetRec(pScrn);
    if (!pVBox)
        return FALSE;

    /* Initialise the guest library */
    vbox_init(pScrn->scrnIndex, pVBox);

    /* Entity information seems to mean bus information. */
    pVBox->pEnt = xf86GetEntityInfo(pScrn->entityList[0]);

    /* The ramdac module is needed for the hardware cursor. */
    if (!xf86LoadSubModule(pScrn, "ramdac"))
        return FALSE;

    /* The framebuffer module. */
    if (!xf86LoadSubModule(pScrn, "fb"))
        return (FALSE);

    if (!xf86LoadSubModule(pScrn, "shadowfb"))
        return FALSE;

    if (!xf86LoadSubModule(pScrn, "vgahw"))
        return FALSE;

#ifdef VBOX_DRI
    /* Load the dri module. */
    if (!xf86LoadSubModule(pScrn, "dri"))
        return FALSE;
#endif

#ifndef PCIACCESS
    if (pVBox->pEnt->location.type != BUS_PCI)
        return FALSE;

    pVBox->pciInfo = xf86GetPciInfoForEntity(pVBox->pEnt->index);
    pVBox->pciTag = pciTag(pVBox->pciInfo->bus,
                           pVBox->pciInfo->device,
                           pVBox->pciInfo->func);
#endif

    /* Set up our ScrnInfoRec structure to describe our virtual
       capabilities to X. */

    pScrn->chipset = "vbox";
    /** @note needed during colourmap initialisation */
    pScrn->rgbBits = 8;

    /* Let's create a nice, capable virtual monitor. */
    pScrn->monitor = pScrn->confScreen->monitor;
    pScrn->monitor->DDC = NULL;
    pScrn->monitor->nHsync = 1;
    pScrn->monitor->hsync[0].lo = 1;
    pScrn->monitor->hsync[0].hi = 10000;
    pScrn->monitor->nVrefresh = 1;
    pScrn->monitor->vrefresh[0].lo = 1;
    pScrn->monitor->vrefresh[0].hi = 100;

    pScrn->progClock = TRUE;

    /* Using the PCI information caused problems with non-powers-of-two
       sized video RAM configurations */
    pVBox->cbFBMax = VBoxVideoGetVRAMSize();
    pScrn->videoRam = pVBox->cbFBMax / 1024;

    /* Check if the chip restricts horizontal resolution or not. */
    pVBox->fAnyX = VBoxVideoAnyWidthAllowed();

    /* Set up clock information that will support all modes we need. */
    pScrn->clockRanges = xnfcalloc(sizeof(ClockRange), 1);
    pScrn->clockRanges->minClock = 1000;
    pScrn->clockRanges->maxClock = 1000000000;
    pScrn->clockRanges->clockIndex = -1;
    pScrn->clockRanges->ClockMulFactor = 1;
    pScrn->clockRanges->ClockDivFactor = 1;

    /* Query the host for the preferred colour depth */
    {
        uint32_t cx = 0, cy = 0, cBits = 0;

        vboxGetPreferredMode(pScrn, 0, &cx, &cy, &cBits);
        /* We only support 16 and 24 bits depth (i.e. 16 and 32bpp) */
        if (cBits != 16)
            cBits = 24;
        if (!xf86SetDepthBpp(pScrn, cBits, 0, 0, Support32bppFb))
            return FALSE;
        vboxAddModes(pScrn, cx, cy);
    }
    if (pScrn->bitsPerPixel != 32 && pScrn->bitsPerPixel != 16)
    {
        xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
                   "The VBox additions only support 16 and 32bpp graphics modes\n");
        return FALSE;
    }
    xf86PrintDepthBpp(pScrn);

#ifdef VBOXVIDEO_13
    /* Work around a bug in the original X server modesetting code, which
     * took the first valid values set to these two as maxima over the
     * server lifetime. */
    pScrn->virtualX = 32000;
    pScrn->virtualY = 32000;
#else
    /* We don't validate with xf86ValidateModes and xf86PruneModes as we
     * already know what we like and what we don't. */

    pScrn->currentMode = pScrn->modes;

    /* Set the right virtual resolution. */
    pScrn->virtualX = pScrn->currentMode->HDisplay;
    pScrn->virtualY = pScrn->currentMode->VDisplay;

#endif /* !VBOXVIDEO_13 */

    /* Needed before we initialise DRI. */
    pVBox->cbLine = vboxLineLength(pScrn, pScrn->virtualX);
    pScrn->displayWidth = vboxDisplayPitch(pScrn, pVBox->cbLine);

    xf86PrintModes(pScrn);

    /* VGA hardware initialisation */
    if (!vgaHWGetHWRec(pScrn))
        return FALSE;
    /* Must be called before any VGA registers are saved or restored */
    vgaHWSetStdFuncs(VGAHWPTR(pScrn));
    vgaHWGetIOBase(VGAHWPTR(pScrn));

    /* Colour weight - we always call this, since we are always in
       truecolour. */
    if (!xf86SetWeight(pScrn, rzeros, rzeros))
        return (FALSE);

    /* visual init */
    if (!xf86SetDefaultVisual(pScrn, -1))
        return (FALSE);

    xf86SetGamma(pScrn, gzeros);

    /* Set the DPI.  Perhaps we should read this from the host? */
    xf86SetDpi(pScrn, 96, 96);

    if (pScrn->memPhysBase == 0) {
#ifdef PCIACCESS
        pScrn->memPhysBase = pVBox->pciInfo->regions[0].base_addr;
#else
        pScrn->memPhysBase = pVBox->pciInfo->memBase[0];
#endif
        pScrn->fbOffset = 0;
    }

    TRACE_EXIT();
    return (TRUE);
}

/**
 * Dummy function for setting the colour palette, which we actually never
 * touch.  However, the server still requires us to provide this.
 */
static void
vboxLoadPalette(ScrnInfoPtr pScrn, int numColors, int *indices,
          LOCO *colors, VisualPtr pVisual)
{
    (void)pScrn; (void) numColors; (void) indices; (void) colors;
    (void)pVisual;
}

/*
 * QUOTE from the XFree86 DESIGN document:
 *
 * This is called at the start of each server generation.
 *
 * (...)
 *
 * Decide which operations need to be placed under resource access
 * control. (...) Map any video memory or other memory regions. (...)
 * Save the video card state. (...) Initialise the initial video
 * mode.
 *
 * End QUOTE.
 */
static Bool VBOXScreenInit(ScreenPtr pScreen, int argc, char **argv)
{
    ScrnInfoPtr pScrn = xf86ScreenToScrn(pScreen);
    VBOXPtr pVBox = VBOXGetRec(pScrn);
    VisualPtr visual;
    unsigned flags;

    TRACE_ENTRY();

    if (!VBOXMapVidMem(pScrn))
        return (FALSE);

    /* save current video state */
    VBOXSaveMode(pScrn);

    /* mi layer - reset the visual list (?)*/
    miClearVisualTypes();
    if (!miSetVisualTypes(pScrn->depth, TrueColorMask,
                          pScrn->rgbBits, TrueColor))
        return (FALSE);
    if (!miSetPixmapDepths())
        return (FALSE);

#ifdef VBOX_DRI
    pVBox->useDRI = VBOXDRIScreenInit(pScrn, pScreen, pVBox);
#endif

    if (!fbScreenInit(pScreen, pVBox->base,
                      pScrn->virtualX, pScrn->virtualY,
                      pScrn->xDpi, pScrn->yDpi,
                      pScrn->displayWidth, pScrn->bitsPerPixel))
        return (FALSE);

    /* Fixup RGB ordering */
    /** @note the X server uses this even in true colour. */
    visual = pScreen->visuals + pScreen->numVisuals;
    while (--visual >= pScreen->visuals) {
        if ((visual->class | DynamicClass) == DirectColor) {
            visual->offsetRed   = pScrn->offset.red;
            visual->offsetGreen = pScrn->offset.green;
            visual->offsetBlue  = pScrn->offset.blue;
            visual->redMask     = pScrn->mask.red;
            visual->greenMask   = pScrn->mask.green;
            visual->blueMask    = pScrn->mask.blue;
        }
    }

    /* must be after RGB ordering fixed */
    fbPictureInit(pScreen, 0, 0);

    xf86SetBlackWhitePixels(pScreen);
    pScrn->vtSema = TRUE;

    if (vbox_open (pScrn, pScreen, pVBox)) {
        vboxEnableVbva(pScrn);
        vboxEnableGraphicsCap(pVBox);
    }

#ifdef VBOXVIDEO_13
    /* Initialise CRTC and output configuration for use with randr1.2. */
    xf86CrtcConfigInit(pScrn, &VBOXCrtcConfigFuncs);

    {
        uint32_t i;

        for (i = 0; i < pVBox->cScreens; ++i)
        {
            char szOutput[256];

            /* Setup our virtual CRTCs. */
            pVBox->paCrtcs[i] = xf86CrtcCreate(pScrn, &VBOXCrtcFuncs);
            pVBox->paCrtcs[i]->driver_private = (void *)(uintptr_t)i;

            /* Set up our virtual outputs. */
            snprintf(szOutput, sizeof(szOutput), "VBOX%u", i);
            pVBox->paOutputs[i] = xf86OutputCreate(pScrn, &VBOXOutputFuncs,
                                                   szOutput);

            /* We are not interested in the monitor section in the
             * configuration file. */
            xf86OutputUseScreenMonitor(pVBox->paOutputs[i], FALSE);
            pVBox->paOutputs[i]->possible_crtcs = 1 << i;
            pVBox->paOutputs[i]->possible_clones = 0;
            pVBox->paOutputs[i]->driver_private = (void *)(uintptr_t)i;
            TRACE_LOG("Created crtc (%p) and output %s (%p)\n",
                      (void *)pVBox->paCrtcs[i], szOutput,
                      (void *)pVBox->paOutputs[i]);
        }
    }

    /* Set a sane minimum and maximum mode size */
    xf86CrtcSetSizeRange(pScrn, 64, 64, 32000, 32000);

    /* Now create our initial CRTC/output configuration. */
    if (!xf86InitialConfiguration(pScrn, TRUE)) {
        xf86DrvMsg(pScrn->scrnIndex, X_ERROR, "Initial CRTC configuration failed!\n");
        return (FALSE);
    }

    /* Initialise randr 1.2 mode-setting functions and set first mode.
     * Note that the mode won't be usable until the server has resized the
     * framebuffer to something reasonable. */
    if (!xf86CrtcScreenInit(pScreen)) {
        return FALSE;
    }

    /* Create our VBOX_MODE display properties. */
    {
        uint32_t i;

        for (i = 0; i < pVBox->cScreens; ++i)
        {
            char csz[] = "0x0";
            RRChangeOutputProperty(pVBox->paOutputs[i]->randr_output,
                                   vboxAtomVBoxMode(), XA_STRING, 8,
                                   PropModeReplace, sizeof(csz), csz, TRUE,
                                   FALSE);

        }
    }

    if (!xf86SetDesiredModes(pScrn)) {
        return FALSE;
    }
#else /* !VBOXVIDEO_13 */
    /* set first video mode */
    if (!VBOXSetMode(pScrn, 0, pScrn->currentMode->HDisplay,
                     pScrn->currentMode->VDisplay, pScrn->frameX0,
                     pScrn->frameY0))
        return FALSE;
    /* Save the size in case we need to re-set it later. */
    pVBox->FBSize.cx = pScrn->currentMode->HDisplay;
    pVBox->FBSize.cy = pScrn->currentMode->VDisplay;
    pVBox->aScreenLocation[0].cx = pScrn->currentMode->HDisplay;
    pVBox->aScreenLocation[0].cy = pScrn->currentMode->VDisplay;
    pVBox->aScreenLocation[0].x = pScrn->frameX0;
    pVBox->aScreenLocation[0].y = pScrn->frameY0;
    /* And make sure that a non-current dynamic mode is at the front of the
     * list */
    vboxWriteHostModes(pScrn, pScrn->currentMode);
#endif /* !VBOXVIDEO_13 */

    /* software cursor */
    miDCInitialize(pScreen, xf86GetPointerScreenFuncs());

    /* colourmap code */
    if (!miCreateDefColormap(pScreen))
	return (FALSE);

    if(!xf86HandleColormaps(pScreen, 256, 8, vboxLoadPalette, NULL, 0))
        return (FALSE);

    pVBox->CloseScreen = pScreen->CloseScreen;
    pScreen->CloseScreen = SCRNINDEXAPI(VBOXCloseScreen);
#ifdef VBOXVIDEO_13
    pScreen->SaveScreen = xf86SaveScreen;
#else
    pScreen->SaveScreen = VBOXSaveScreen;
#endif

#ifdef VBOXVIDEO_13
    xf86DPMSInit(pScreen, xf86DPMSSet, 0);
#else
    /* We probably do want to support power management - even if we just use
       a dummy function. */
    xf86DPMSInit(pScreen, VBOXDisplayPowerManagementSet, 0);
#endif

    /* Report any unused options (only for the first generation) */
    if (serverGeneration == 1)
        xf86ShowUnusedOptions(pScrn->scrnIndex, pScrn->options);

    if (vbox_cursor_init(pScreen) != TRUE)
        xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
                   "Unable to start the VirtualBox mouse pointer integration with the host system.\n");

#ifdef VBOX_DRI
    if (pVBox->useDRI)
        pVBox->useDRI = VBOXDRIFinishScreenInit(pScreen);
#endif
    return (TRUE);
}

static Bool VBOXEnterVT(ScrnInfoPtr pScrn)
{
    VBOXPtr pVBox = VBOXGetRec(pScrn);

    TRACE_ENTRY();
    vboxClearVRAM(pScrn, 0, 0);
    if (pVBox->fHaveHGSMI)
        vboxEnableVbva(pScrn);
#ifdef VBOX_DRI
    if (pVBox->useDRI)
        DRIUnlock(xf86ScrnToScreen(pScrn));
#endif
    /* Re-assert this in case we had a change request while switched out. */
    if (pVBox->FBSize.cx && pVBox->FBSize.cy)
        VBOXAdjustScreenPixmap(pScrn, pVBox->FBSize.cx, pVBox->FBSize.cy);
#ifdef VBOXVIDEO_13
    if (!xf86SetDesiredModes(pScrn))
        return FALSE;
#else
    if (!VBOXSetMode(pScrn, 0, pScrn->currentMode->HDisplay,
                     pScrn->currentMode->VDisplay, pScrn->frameX0,
                     pScrn->frameY0))
        return FALSE;
#endif
    return TRUE;
}

static void VBOXLeaveVT(ScrnInfoPtr pScrn)
{
    VBOXPtr pVBox = VBOXGetRec(pScrn);

    TRACE_ENTRY();
    if (pVBox->fHaveHGSMI)
        vboxDisableVbva(pScrn);
    vboxClearVRAM(pScrn, 0, 0);
    VBOXRestoreMode(pScrn);
    vboxDisableGraphicsCap(pVBox);
#ifdef VBOX_DRI
    if (pVBox->useDRI)
        DRILock(xf86ScrnToScreen(pScrn), 0);
#endif
    TRACE_EXIT();
}

static Bool VBOXCloseScreen(ScreenPtr pScreen)
{
    ScrnInfoPtr pScrn = xf86ScreenToScrn(pScreen);
    VBOXPtr pVBox = VBOXGetRec(pScrn);

    if (pScrn->vtSema)
    {
        if (pVBox->fHaveHGSMI)
            vboxDisableVbva(pScrn);
        if (pScrn->vtSema)
            vboxDisableGraphicsCap(pVBox);
        vboxClearVRAM(pScrn, 0, 0);
    }
#ifdef VBOX_DRI
    if (pVBox->useDRI)
        VBOXDRICloseScreen(pScreen, pVBox);
    pVBox->useDRI = false;
#endif

    if (pScrn->vtSema) {
        VBOXRestoreMode(pScrn);
        VBOXUnmapVidMem(pScrn);
    }
    pScrn->vtSema = FALSE;

    /* Do additional bits which are separate for historical reasons */
    vbox_close(pScrn, pVBox);

    pScreen->CloseScreen = pVBox->CloseScreen;
#ifndef XF86_SCRN_INTERFACE
    return pScreen->CloseScreen(pScreen->myNum, pScreen);
#else
    return pScreen->CloseScreen(pScreen);
#endif
}

static Bool VBOXSwitchMode(ScrnInfoPtr pScrn, DisplayModePtr pMode)
{
    VBOXPtr pVBox;
    Bool rc;

    TRACE_LOG("HDisplay=%d, VDisplay=%d\n", pMode->HDisplay, pMode->VDisplay);
#ifndef VBOXVIDEO_13
    pVBox = VBOXGetRec(pScrn);
    /* Save the size in case we need to re-set it later. */
    pVBox->FBSize.cx = pMode->HDisplay;
    pVBox->FBSize.cy = pMode->VDisplay;
    pVBox->aScreenLocation[0].cx = pMode->HDisplay;
    pVBox->aScreenLocation[0].cy = pMode->VDisplay;
    pVBox->aScreenLocation[0].x = pScrn->frameX0;
    pVBox->aScreenLocation[0].y = pScrn->frameY0;
#endif
    if (!pScrn->vtSema)
    {
        xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
                   "We do not own the active VT, exiting.\n");
        return TRUE;
    }
#ifdef VBOXVIDEO_13
    rc = xf86SetSingleMode(pScrn, pMode, 0);
#else
    VBOXAdjustScreenPixmap(pScrn, pMode->HDisplay, pMode->VDisplay);
    rc = VBOXSetMode(pScrn, 0, pMode->HDisplay, pMode->VDisplay,
                     pScrn->frameX0, pScrn->frameY0);
    if (rc)
    {
        vboxWriteHostModes(pScrn, pMode);
        xf86PrintModes(pScrn);
    }
    if (rc && !vboxGuestIsSeamless(pScrn))
        vboxSaveVideoMode(pScrn, pMode->HDisplay, pMode->VDisplay,
                          pScrn->bitsPerPixel);
#endif
    TRACE_LOG("returning %s\n", rc ? "TRUE" : "FALSE");
    return rc;
}

static void VBOXAdjustFrame(ScrnInfoPtr pScrn, int x, int y)
{
    VBOXPtr pVBox = VBOXGetRec(pScrn);

    TRACE_ENTRY();
    pVBox->aScreenLocation[0].x = x;
    pVBox->aScreenLocation[0].y = y;
    /* Don't fiddle with the hardware if we are switched
     * to a virtual terminal. */
    if (!pScrn->vtSema)
    {
        xf86DrvMsg(pScrn->scrnIndex, X_ERROR,
                   "We do not own the active VT, exiting.\n");
        return;
    }
    VBOXSetMode(pScrn, 0, pVBox->aScreenLocation[0].cx,
                pVBox->aScreenLocation[0].cy, x, y);
    TRACE_EXIT();
}

static void VBOXFreeScreen(ScrnInfoPtr pScrn)
{
    /* Destroy the VGA hardware record */
    vgaHWFreeHWRec(pScrn);
    /* And our private record */
    free(pScrn->driverPrivate);
    pScrn->driverPrivate = NULL;
}

static Bool
VBOXMapVidMem(ScrnInfoPtr pScrn)
{
    VBOXPtr pVBox = VBOXGetRec(pScrn);
    Bool rc = TRUE;

    TRACE_ENTRY();
    if (!pVBox->base)
    {
#ifdef PCIACCESS
        (void) pci_device_map_range(pVBox->pciInfo,
                                    pScrn->memPhysBase,
                                    pScrn->videoRam * 1024,
                                    PCI_DEV_MAP_FLAG_WRITABLE,
                                    & pVBox->base);
#else
        pVBox->base = xf86MapPciMem(pScrn->scrnIndex,
                                    VIDMEM_FRAMEBUFFER,
                                    pVBox->pciTag, pScrn->memPhysBase,
                                    (unsigned) pScrn->videoRam * 1024);
#endif
        if (!pVBox->base)
            rc = FALSE;
    }
    TRACE_LOG("returning %s\n", rc ? "TRUE" : "FALSE");
    return rc;
}

static void
VBOXUnmapVidMem(ScrnInfoPtr pScrn)
{
    VBOXPtr pVBox = VBOXGetRec(pScrn);

    TRACE_ENTRY();
    if (pVBox->base == NULL)
        return;

#ifdef PCIACCESS
    (void) pci_device_unmap_range(pVBox->pciInfo,
                                  pVBox->base,
                                  pScrn->videoRam * 1024);
#else
    xf86UnMapVidMem(pScrn->scrnIndex, pVBox->base,
                    (unsigned) pScrn->videoRam * 1024);
#endif
    pVBox->base = NULL;
    TRACE_EXIT();
}

static Bool
VBOXSaveScreen(ScreenPtr pScreen, int mode)
{
    (void)pScreen; (void)mode;
    return TRUE;
}

void
VBOXSaveMode(ScrnInfoPtr pScrn)
{
    VBOXPtr pVBox = VBOXGetRec(pScrn);
    vgaRegPtr vgaReg;

    TRACE_ENTRY();
    vgaReg = &VGAHWPTR(pScrn)->SavedReg;
    vgaHWSave(pScrn, vgaReg, VGA_SR_ALL);
    pVBox->fSavedVBEMode = VBoxVideoGetModeRegisters(&pVBox->cSavedWidth,
                                                     &pVBox->cSavedHeight,
                                                     &pVBox->cSavedPitch,
                                                     &pVBox->cSavedBPP,
                                                     &pVBox->fSavedFlags);
}

void
VBOXRestoreMode(ScrnInfoPtr pScrn)
{
    VBOXPtr pVBox = VBOXGetRec(pScrn);
    vgaRegPtr vgaReg;

    TRACE_ENTRY();
    vgaReg = &VGAHWPTR(pScrn)->SavedReg;
    vgaHWRestore(pScrn, vgaReg, VGA_SR_ALL);
    if (pVBox->fSavedVBEMode)
        VBoxVideoSetModeRegisters(pVBox->cSavedWidth, pVBox->cSavedHeight,
                                  pVBox->cSavedPitch, pVBox->cSavedBPP,
                                  pVBox->fSavedFlags, 0, 0);
    else
        VBoxVideoDisableVBE();
}

static void
VBOXDisplayPowerManagementSet(ScrnInfoPtr pScrn, int mode,
                int flags)
{
    (void)pScrn; (void)mode; (void) flags;
}
