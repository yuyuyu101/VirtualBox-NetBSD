/* $Id: VBoxGuest-netbsd.c $ */
/** @file
 * VirtualBox Guest Additions Driver for NetBSD.
 */

/*
 * Copyright (C) 2007-2011 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

/** @todo r=bird: This must merge with SUPDrv-netbsd.c before long. The two
 * source files should only differ on prefixes and the extra bits wrt to the
 * pci device. I.e. it should be diffable so that fixes to one can easily be
 * applied to the other. */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/poll.h>
#include <sys/selinfo.h>
#include <sys/queue.h>
#include <sys/lock.h>
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/uio.h>
#include <sys/file.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#ifdef PVM
#  undef PVM
#endif
#include "VBoxGuestInternal.h"
#include <VBox/log.h>
#include <iprt/assert.h>
#include <iprt/initterm.h>
#include <iprt/process.h>
#include <iprt/mem.h>
#include <iprt/asm.h>

/** The module name. */
#define DEVICE_NAME    "vboxguest"

typedef struct VBoxGuestDeviceState
{
    /** Resource ID of the I/O port */
    int                iIOPortResId;
    /** Pointer to the I/O port resource. */
    struct resource   *pIOPortRes;
    /** Start address of the IO Port. */
    uint16_t           uIOPortBase;
    /** Resource ID of the MMIO area */
    int                iVMMDevMemResId;
    /** Pointer to the MMIO resource. */
    struct resource   *pVMMDevMemRes;
    /** Handle of the MMIO resource. */
    bus_space_handle_t VMMDevMemHandle;
    /** Size of the memory area. */
    bus_size_t         VMMDevMemSize;
    /** Mapping of the register space */
    void              *pMMIOBase;
    /** IRQ number */
    int                iIrqResId;
    /** IRQ resource handle. */
    struct resource   *pIrqRes;
    /** Pointer to the IRQ handler. */
    void              *pfnIrqHandler;
    /** VMMDev version */
    uint32_t           u32Version;
    /** Pointer to the session handle. */
    PVBOXGUESTSESSION       session;
    /** Controller features, limits and status. */
    u_int                   vboxguest_state;
} vboxguest_softc;


static MALLOC_DEFINE(M_VBOXDEV, "vboxdev_pci", "VirtualBox Guest driver PCI");

/*
 * Character device file handlers.
 */
static int VBoxGuestNetBSDOpen(dev_t device, int flags, int fmt, struct lwp *process);
static int VBoxGuestNetBSDClose(dev_t device, int flags, int fmt, struct lwp *process);
static int VBoxGuestNetBSDRead(dev_t, struct uio *, int);
static int VBoxGuestNetBSDWrite(dev_t, struct uio *, int);
static int VBoxGuestNetBSDIOCtl(dev_t device, u_long command, caddr_t data,
		      int flags, struct lwp *process);
static int VBoxGuestNetBSDPoll(dev_t, int, struct lwp *);
static void VBoxGuestNetBSDAttach(int num);

/*
 * IRQ related functions.
 */
static void VBoxGuestNetBSDRemoveIRQ(device_t pDevice, void *pvState);
static int  VBoxGuestNetBSDAddIRQ(device_t pDevice, void *pvState);
static int  VBoxGuestNetBSDISR(void *pvState);

/*
 * Device node entry points.
 */
static struct cdevsw g_VBoxGuestNetBSDChrDevSW =
{
    VBoxGuestNetBSDOpen,
    VBoxGuestNetBSDClose,
    noread,
    nowrite,
    VBoxGuestNetBSDIOCtl,
    nostop,
    notty,
    VBoxGuestNetBSDPoll,
    nommap,
    nokqfilter,
};

/** Device extention & session data association structure. */
static VBOXGUESTDEVEXT      g_DevExt;
/** List of cloned device. Managed by the kernel. */
static struct clonedevs    *g_pVBoxGuestNetBSDClones;
/** The dev_clone event handler tag. */
static eventhandler_tag     g_VBoxGuestNetBSDEHTag;
/** selinfo structure used for polling. */
static struct selinfo       g_SelInfo;
/** Reference counter */
static volatile uint32_t    cUsers;

struct cfdriver vboxguest_cd = {
        NULL, "vboxguest", DV_DULL
};

CFATTACH_DECL3_NEW(vboxguest, sizeof(vboxguest_softc),
    NULL, VBoxGuestNetBSDAttach, VBoxGuestNetBSDDetach, NULL, NULL, NULL,
    NULL);

/**
 * DEVFS event handler.
 */
static void VBoxGuestNetBSDClone(void *pvArg, struct ucred *pCred, char *pszName, int cchName, struct cdev **ppDev)
{
    int iUnit;
    int rc;

    Log(("VBoxGuestNetBSDClone: pszName=%s ppDev=%p\n", pszName, ppDev));

    /*
     * One device node per user, si_drv1 points to the session.
     * /dev/vboxguest<N> where N = {0...255}.
     */
    if (!ppDev)
        return;
    if (strcmp(pszName, "vboxguest") == 0)
        iUnit =  -1;
    else if (dev_stdclone(pszName, NULL, "vboxguest", &iUnit) != 1)
        return;
    if (iUnit >= 256)
    {
        Log(("VBoxGuestNetBSDClone: iUnit=%d >= 256 - rejected\n", iUnit));
        return;
    }

    Log(("VBoxGuestNetBSDClone: pszName=%s iUnit=%d\n", pszName, iUnit));

    rc = clone_create(&g_pVBoxGuestNetBSDClones, &g_VBoxGuestNetBSDChrDevSW, &iUnit, ppDev, 0);
    Log(("VBoxGuestNetBSDClone: clone_create -> %d; iUnit=%d\n", rc, iUnit));
    if (rc)
    {
        *ppDev = make_dev(&g_VBoxGuestNetBSDChrDevSW,
                          iUnit,
                          UID_ROOT,
                          GID_WHEEL,
                          0664,
                          "vboxguest%d", iUnit);
        if (*ppDev)
        {
            dev_ref(*ppDev);
            (*ppDev)->si_flags |= SI_CHEAPCLONE;
            Log(("VBoxGuestNetBSDClone: Created *ppDev=%p iUnit=%d si_drv1=%p si_drv2=%p\n",
                     *ppDev, iUnit, (*ppDev)->si_drv1, (*ppDev)->si_drv2));
            (*ppDev)->si_drv1 = (*ppDev)->si_drv2 = NULL;
        }
        else
            Log(("VBoxGuestNetBSDClone: make_dev iUnit=%d failed\n", iUnit));
    }
    else
        Log(("VBoxGuestNetBSDClone: Existing *ppDev=%p iUnit=%d si_drv1=%p si_drv2=%p\n",
             *ppDev, iUnit, (*ppDev)->si_drv1, (*ppDev)->si_drv2));
}

/**
 * File open handler
 *
 */
static int VBoxGuestNetBSDOpen(dev_t device, int flags, int fmt, struct lwp *process)
{
    int                 rc;
    PVBOXGUESTSESSION   pSession;
    vboxguest_softc *vboxguest;

    LogFlow((DEVICE_NAME ":VBoxGuestNetBSDOpen\n"));

    if ((vboxguest = device_lookup_private(&vboxguest_cd, minor(device))) == NULL)
        return (ENXIO);
    if ((vboxguest->vboxguest_state & VBOXGUEST_STATE_INITOK) == 0)
        return (ENXIO);
    if ((vboxguest->vboxguest_state & VBOXGUEST_STATE_OPEN) != 0)
        return (EBUSY);
    vboxguest->vboxguest_state |= VBOXGUEST_STATE_OPEN;

    /*
     * Create a new session.
     */
    rc = VBoxGuestCreateUserSession(&g_DevExt, &pSession);
    if (RT_SUCCESS(rc))
    {
        Log((DEVICE_NAME ":VBoxGuestNetBSDOpen success: g_DevExt=%p pSession=%p rc=%d pid=%d\n", &g_DevExt, pSession, rc, (int)RTProcSelf()));
        ASMAtomicIncU32(&cUsers);
        vboxguest->session = pSession;
        return 0;
    }

    LogRel((DEVICE_NAME ":VBoxGuestNetBSDOpen: failed. rc=%d\n", rc));
    return RTErrConvertToErrno(rc);
}

/**
 * File close handler
 *
 */
static int VBoxGuestNetBSDClose(dev_t device, int flags, int fmt, struct lwp *process)
{
    vboxguest_softc *vboxguest;
    vboxguest = device_lookup_private(&vboxguest_cd, minor(device));

    PVBOXGUESTSESSION pSession = (PVBOXGUESTSESSION)vboxguest->session;
    Log(("VBoxGuestNetBSDClose: device=%#x pSession=%p\n", device, pSession));

    /*
     * Close the session if it's still hanging on to the device...
     */
    if (VALID_PTR(pSession))
    {
        VBoxGuestCloseSession(&g_DevExt, pSession);
        vboxguest->vboxguest_state &= ~VBOXGUEST_STATE_OPEN;
        vboxguest->session = NULL;
        ASMAtomicDecU32(&cUsers);
    } else {
        Log(("VBoxGuestNetBSDClose: si_drv1=%p!\n", pSession));
    }
    return 0;
}

/**
 * IOCTL handler
 *
 */
static int VBoxGuestNetBSDIOCtl(dev_t device, u_long command, caddr_t data, int flags, struct lwp *process)
{
    LogFlow((DEVICE_NAME ":VBoxGuestNetBSDIOCtl\n"));

    vboxguest_softc *vboxguest;
    vboxguest = device_lookup_private(&vboxguest_cd, minor(device));
    int rc = 0;

    PVBOXGUESTSESSION pSession = vboxguest->session;
    if (!pSession) {
        LogRel((DEVICE_NAME "::IOCtl: no session data for %d\n", minor(device)));
        return EINVAL;
    }

    /*
     * Validate the request wrapper.
     */
    if (IOCPARM_LEN(command) != sizeof(VBGLBIGREQ)) {
        Log((DEVICE_NAME ": VBoxGuestFreeBSDIOCtl: bad request %lu size=%lu expected=%d\n", command, IOCPARM_LEN(command), sizeof(VBGLBIGREQ)));
        return ENOTTY;
    }

    PVBGLBIGREQ ReqWrap = (PVBGLBIGREQ)data;
    if (ReqWrap->u32Magic != VBGLBIGREQ_MAGIC)
    {
        Log((DEVICE_NAME ": VBoxGuestNetBSDIOCtl: bad magic %#x; pArg=%p Cmd=%lu.\n", ReqWrap->u32Magic, data, command));
        return EINVAL;
    }
    if (RT_UNLIKELY(   ReqWrap->cbData == 0
                    || ReqWrap->cbData > _1M*16))
    {
        printf(DEVICE_NAME ": VBoxGuestNetBSDIOCtl: bad size %#x; pArg=%p Cmd=%lu.\n", ReqWrap->cbData, data, command);
        return EINVAL;
    }

    /*
     * Read the request.
     */
    void *pvBuf = RTMemTmpAlloc(ReqWrap->cbData);
    if (RT_UNLIKELY(!pvBuf))
    {
        Log((DEVICE_NAME ":VBoxGuestNetBSDIOCtl: RTMemTmpAlloc failed to alloc %d bytes.\n", ReqWrap->cbData));
        return ENOMEM;
    }

    rc = copyin((void *)(uintptr_t)ReqWrap->pvDataR3, pvBuf, ReqWrap->cbData);
    if (RT_UNLIKELY(rc))
    {
        RTMemTmpFree(pvBuf);
        Log((DEVICE_NAME ":VBoxGuestNetBSDIOCtl: copyin failed; pvBuf=%p pArg=%p Cmd=%lu. rc=%d\n", pvBuf, data, command, rc));
        return EFAULT;
    }
    if (RT_UNLIKELY(   ReqWrap->cbData != 0
                    && !VALID_PTR(pvBuf)))
    {
        RTMemTmpFree(pvBuf);
        Log((DEVICE_NAME ":VBoxGuestNetBSDIOCtl: pvBuf invalid pointer %p\n", pvBuf));
        return EINVAL;
    }
    Log((DEVICE_NAME ":VBoxGuestNetBSDIOCtl: pSession=%p pid=%d.\n", pSession, (int)RTProcSelf()));

    /*
     * Process the IOCtl.
     */
    size_t cbDataReturned;
    rc = VBoxGuestCommonIOCtl(command, &g_DevExt, pSession, pvBuf, ReqWrap->cbData, &cbDataReturned);
    if (RT_SUCCESS(rc)) {
        rc = 0;
        if (RT_UNLIKELY(cbDataReturned > ReqWrap->cbData))
        {
            Log((DEVICE_NAME ":VBoxGuestNetBSDIOCtl: too much output data %d expected %d\n", cbDataReturned, ReqWrap->cbData));
            cbDataReturned = ReqWrap->cbData;
        }
        if (cbDataReturned > 0)
        {
            rc = copyout(pvBuf, (void *)(uintptr_t)ReqWrap->pvDataR3, cbDataReturned);
            if (RT_UNLIKELY(rc))
            {
                Log((DEVICE_NAME ":VBoxGuestNetBSDIOCtl: copyout failed; pvBuf=%p pArg=%p Cmd=%lu. rc=%d\n", pvBuf, pvData, ulCmd, rc));
                rc = EFAULT;
            }
        }
    } else {
        Log((DEVICE_NAME ":VBoxGuestNetBSDIOCtl: VBoxGuestCommonIOCtl failed. rc=%d\n", rc));
        rc = EFAULT;
    }
    RTMemTmpFree(pvBuf);
    return rc;
}

static int VBoxGuestNetBSDPoll(dev_t device, int events, struct lwp *lwp)
{
    LogFlow((DEVICE_NAME "::Poll: event=%d\n", events));

    vboxguest_softc *vboxguest;
    vboxguest = device_lookup_private(&vboxguest_cd, minor(device));
    int rc = 0;
    int events_processed;

    PVBOXGUESTSESSION pSession = vboxguest->session;
    if (RT_UNLIKELY(!VALID_PTR(pSession))) {
        Log((DEVICE_NAME "::Poll: no state data for %d\n", device));
        return (events & (POLLHUP|POLLIN|POLLRDNORM|POLLOUT|POLLWRNORM));
    }

    uint32_t u32CurSeq = ASMAtomicUoReadU32(&g_DevExt.u32MousePosChangedSeq);
    if (pSession->u32MousePosChangedSeq != u32CurSeq) {
        events_processed = events & (POLLIN | POLLRDNORM);
        pSession->u32MousePosChangedSeq = u32CurSeq;
    } else {
        events_processed = 0;

        selrecord(lwp, &g_SelInfo);
    }

    return events_processed;
}

static int VBoxGuestNetBSDDetach(device_t pDevice)
{
    vboxguest_softc *vboxguest;
    vboxguest = device_lookup_private(&vboxguest_cd, minor(device));

    if (cUsers > 0)
        return EBUSY;

    /*
     * Reverse what we did in VBoxGuestNetBSDAttach.
     */
    if (g_VBoxGuestNetBSDEHTag != NULL)
        EVENTHANDLER_DEREGISTER(dev_clone, g_VBoxGuestNetBSDEHTag);

    clone_cleanup(&g_pVBoxGuestNetBSDClones);

    VBoxGuestNetBSDRemoveIRQ(pDevice, vboxguest);

    if (vboxguest->pVMMDevMemRes)
        bus_release_resource(pDevice, SYS_RES_MEMORY, vboxguest->iVMMDevMemResId, vboxguest->pVMMDevMemRes);
    if (vboxguest->pIOPortRes)
        bus_release_resource(pDevice, SYS_RES_IOPORT, vboxguest->iIOPortResId, vboxguest->pIOPortRes);

    VBoxGuestDeleteDevExt(&g_DevExt);

    RTR0Term();

    return 0;
}

/**
 * Interrupt service routine.
 *
 * @returns Whether the interrupt was from VMMDev.
 * @param   pvState Opaque pointer to the device state.
 */
static int VBoxGuestNetBSDISR(void *pvState)
{
    LogFlow((DEVICE_NAME ":VBoxGuestNetBSDISR pvState=%p\n", pvState));

    bool fOurIRQ = VBoxGuestCommonISR(&g_DevExt);

    return fOurIRQ ? 0 : 1;
}

void VBoxGuestNativeISRMousePollEvent(PVBOXGUESTDEVEXT pDevExt)
{
    LogFlow((DEVICE_NAME "::NativeISRMousePollEvent:\n"));

    /*
     * Wake up poll waiters.
     */
    selwakeup(&g_SelInfo);
}

/**
 * Sets IRQ for VMMDev.
 *
 * @returns NetBSD error code.
 * @param   pDevice  Pointer to the device info structure.
 * @param   pvState  Pointer to the state info structure.
 */
static int VBoxGuestNetBSDAddIRQ(device_t pDevice, void *pvState)
{
    int iResId = 0;
    int rc = 0;
    vboxguest_softc *vboxguest = (vboxguest_softc *)pvState;

    vboxguest->pIrqRes = bus_alloc_resource_any(pDevice, SYS_RES_IRQ, &iResId, RF_SHAREABLE | RF_ACTIVE);

    rc = bus_setup_intr(pDevice, vboxguest->pIrqRes, INTR_TYPE_BIO | INTR_MPSAFE, NULL, (driver_intr_t *)VBoxGuestNetBSDISR, vboxguest,
                        &vboxguest->pfnIrqHandler);
    if (rc)
    {
        vboxguest->pfnIrqHandler = NULL;
        return VERR_DEV_IO_ERROR;
    }

    vboxguest->iIrqResId = iResId;

    return VINF_SUCCESS;
}

/**
 * Removes IRQ for VMMDev.
 *
 * @param   pDevice  Pointer to the device info structure.
 * @param   pvState  Opaque pointer to the state info structure.
 */
static void VBoxGuestNetBSDRemoveIRQ(device_t pDevice, void *pvState)
{
    vboxguest_softc *vboxguest = (vboxguest_softc *)pvState;

    if (vboxguest->pIrqRes)
    {
        bus_teardown_intr(pDevice, vboxguest->pIrqRes, vboxguest->pfnIrqHandler);
        bus_release_resource(pDevice, SYS_RES_IRQ, 0, vboxguest->pIrqRes);
    }
}

static int VBoxGuestNetBSDAttach(device_t self, void *aux)
{
    int rc = VINF_SUCCESS;
    int iResId = 0;
    vboxguest_softc *vboxguest;

    cUsers = 0;

    /*
     * Initialize IPRT R0 driver, which internally calls OS-specific r0 init.
     */
    rc = RTR0Init(0);
    if (RT_FAILURE(rc))
    {
        LogFunc(("RTR0Init failed.\n"));
        return ENXIO;
    }

    vboxguest = device_lookup_private(&vboxguest_cd, minor(self));

    /*
     * Allocate I/O port resource.
     */
    iResId                 = PCIR_BAR(0);
    vboxguest->pIOPortRes     = bus_alloc_resource_any(self, SYS_RES_IOPORT, &iResId, RF_ACTIVE);
    vboxguest->uIOPortBase    = rman_get_start(vboxguest->pIOPortRes);
    vboxguest->iIOPortResId   = iResId;
    if (vboxguest->uIOPortBase)
    {
        /*
         * Map the MMIO region.
         */
        iResId                   = PCIR_BAR(1);
        vboxguest->pVMMDevMemRes    = bus_alloc_resource_any(self, SYS_RES_MEMORY, &iResId, RF_ACTIVE);
        vboxguest->VMMDevMemHandle  = rman_get_bushandle(vboxguest->pVMMDevMemRes);
        vboxguest->VMMDevMemSize    = rman_get_size(vboxguest->pVMMDevMemRes);

        vboxguest->pMMIOBase       = rman_get_virtual(vboxguest->pVMMDevMemRes);
        vboxguest->iVMMDevMemResId = iResId;
        if (vboxguest->pMMIOBase)
        {
            /*
             * Call the common device extension initializer.
             */
            rc = VBoxGuestInitDevExt(&g_DevExt, vboxguest->uIOPortBase,
                                     vboxguest->pMMIOBase, vboxguest->VMMDevMemSize,
#if ARCH_BITS == 64
                                     VBOXOSTYPE_NetBSD_x64,
#else
                                     VBOXOSTYPE_NetBSD,
#endif
                                     VMMDEV_EVENT_MOUSE_POSITION_CHANGED);
            if (RT_SUCCESS(rc))
            {
                /*
                 * Add IRQ of VMMDev.
                 */
                rc = VBoxGuestNetBSDAddIRQ(self, vboxguest);
                if (RT_SUCCESS(rc))
                {
                    /*
                     * Configure device cloning.
                     */
                    clone_setup(&g_pVBoxGuestNetBSDClones);
                    g_VBoxGuestNetBSDEHTag = EVENTHANDLER_REGISTER(dev_clone, VBoxGuestNetBSDClone, 0, 1000);
                    if (g_VBoxGuestNetBSDEHTag)
                    {
                        printf(DEVICE_NAME ": loaded successfully\n");
                        return 0;
                    }

                    printf(DEVICE_NAME ": EVENTHANDLER_REGISTER(dev_clone,,,) failed\n");
                    clone_cleanup(&g_pVBoxGuestNetBSDClones);
                    VBoxGuestNetBSDRemoveIRQ(self, vboxguest);
                }
                else
                    printf((DEVICE_NAME ":VBoxGuestInitDevExt failed.\n"));
                VBoxGuestDeleteDevExt(&g_DevExt);
            }
            else
                printf((DEVICE_NAME ":VBoxGuestNetBSDAddIRQ failed.\n"));
        }
        else
            printf((DEVICE_NAME ":MMIO region setup failed.\n"));
    }
    else
        printf((DEVICE_NAME ":IOport setup failed.\n"));

    RTR0Term();
    return ENXIO;
}

/* Common code that depend on g_DevExt. */
#include "VBoxGuestIDC-unix.c.h"

MODULE(MODULE_CLASS_DRIVER, vboxguest, NULL);

static int
vboxguest_modcmd(modcmd_t cmd, void *opaque)
{
    switch (cmd) {
        case MODULE_CMD_INIT:
                return 0;
        case MODULE_CMD_FINI:
                return 0;
        default:
                return ENOTTY;
    }
}
