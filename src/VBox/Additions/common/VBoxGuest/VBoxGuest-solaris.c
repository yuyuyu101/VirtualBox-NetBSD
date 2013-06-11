/* $Id: VBoxGuest-solaris.c $ */
/** @file
 * VirtualBox Guest Additions Driver for Solaris.
 */

/*
 * Copyright (C) 2007-2012 Oracle Corporation
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


/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#include <sys/conf.h>
#include <sys/modctl.h>
#include <sys/mutex.h>
#include <sys/pci.h>
#include <sys/stat.h>
#include <sys/ddi.h>
#include <sys/ddi_intr.h>
#include <sys/sunddi.h>
#include <sys/open.h>
#include <sys/sunldi.h>
#include <sys/file.h>
#undef u /* /usr/include/sys/user.h:249:1 is where this is defined to (curproc->p_user). very cool. */

#include "VBoxGuestInternal.h"
#include <VBox/log.h>
#include <VBox/version.h>
#include <iprt/assert.h>
#include <iprt/initterm.h>
#include <iprt/process.h>
#include <iprt/mem.h>
#include <iprt/cdefs.h>
#include <iprt/asm.h>


/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
/** The module name. */
#define DEVICE_NAME              "vboxguest"
/** The module description as seen in 'modinfo'. */
#define DEVICE_DESC              "VirtualBox GstDrv"


/*******************************************************************************
*   Internal Functions                                                         *
*******************************************************************************/
static int VBoxGuestSolarisOpen(dev_t *pDev, int fFlag, int fType, cred_t *pCred);
static int VBoxGuestSolarisClose(dev_t Dev, int fFlag, int fType, cred_t *pCred);
static int VBoxGuestSolarisRead(dev_t Dev, struct uio *pUio, cred_t *pCred);
static int VBoxGuestSolarisWrite(dev_t Dev, struct uio *pUio, cred_t *pCred);
static int VBoxGuestSolarisIOCtl(dev_t Dev, int Cmd, intptr_t pArg, int Mode, cred_t *pCred, int *pVal);
static int VBoxGuestSolarisPoll(dev_t Dev, short fEvents, int fAnyYet, short *pReqEvents, struct pollhead **ppPollHead);

static int VBoxGuestSolarisGetInfo(dev_info_t *pDip, ddi_info_cmd_t enmCmd, void *pArg, void **ppResult);
static int VBoxGuestSolarisAttach(dev_info_t *pDip, ddi_attach_cmd_t enmCmd);
static int VBoxGuestSolarisDetach(dev_info_t *pDip, ddi_detach_cmd_t enmCmd);

static int VBoxGuestSolarisAddIRQ(dev_info_t *pDip);
static void VBoxGuestSolarisRemoveIRQ(dev_info_t *pDip);
static uint_t VBoxGuestSolarisISR(caddr_t Arg);


/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/
/**
 * cb_ops: for drivers that support char/block entry points
 */
static struct cb_ops g_VBoxGuestSolarisCbOps =
{
    VBoxGuestSolarisOpen,
    VBoxGuestSolarisClose,
    nodev,                  /* b strategy */
    nodev,                  /* b dump */
    nodev,                  /* b print */
    VBoxGuestSolarisRead,
    VBoxGuestSolarisWrite,
    VBoxGuestSolarisIOCtl,
    nodev,                  /* c devmap */
    nodev,                  /* c mmap */
    nodev,                  /* c segmap */
    VBoxGuestSolarisPoll,
    ddi_prop_op,            /* property ops */
    NULL,                   /* streamtab  */
    D_NEW | D_MP,           /* compat. flag */
    CB_REV                  /* revision */
};

/**
 * dev_ops: for driver device operations
 */
static struct dev_ops g_VBoxGuestSolarisDevOps =
{
    DEVO_REV,               /* driver build revision */
    0,                      /* ref count */
    VBoxGuestSolarisGetInfo,
    nulldev,                /* identify */
    nulldev,                /* probe */
    VBoxGuestSolarisAttach,
    VBoxGuestSolarisDetach,
    nodev,                  /* reset */
    &g_VBoxGuestSolarisCbOps,
    (struct bus_ops *)0,
    nodev                   /* power */
};

/**
 * modldrv: export driver specifics to the kernel
 */
static struct modldrv g_VBoxGuestSolarisModule =
{
    &mod_driverops,         /* extern from kernel */
    DEVICE_DESC " " VBOX_VERSION_STRING "r" RT_XSTR(VBOX_SVN_REV),
    &g_VBoxGuestSolarisDevOps
};

/**
 * modlinkage: export install/remove/info to the kernel
 */
static struct modlinkage g_VBoxGuestSolarisModLinkage =
{
    MODREV_1,               /* loadable module system revision */
    &g_VBoxGuestSolarisModule,
    NULL                    /* terminate array of linkage structures */
};

/**
 * State info for each open file handle.
 */
typedef struct
{
    /** Pointer to the session handle. */
    PVBOXGUESTSESSION       pSession;
    /** The process reference for posting signals */
    void                   *pvProcRef;
} vboxguest_state_t;


/*******************************************************************************
*   Global Variables                                                           *
*******************************************************************************/
/** Device handle (we support only one instance). */
static dev_info_t          *g_pDip = NULL;
/** Opaque pointer to file-descriptor states */
static void                *g_pVBoxGuestSolarisState = NULL;
/** Device extention & session data association structure. */
static VBOXGUESTDEVEXT      g_DevExt;
/** IO port handle. */
static ddi_acc_handle_t     g_PciIOHandle;
/** MMIO handle. */
static ddi_acc_handle_t     g_PciMMIOHandle;
/** IO Port. */
static uint16_t             g_uIOPortBase;
/** Address of the MMIO region.*/
static caddr_t              g_pMMIOBase;
/** Size of the MMIO region. */
static off_t                g_cbMMIO;
/** Pointer to the interrupt handle vector */
static ddi_intr_handle_t   *g_pIntr;
/** Number of actually allocated interrupt handles */
static size_t               g_cIntrAllocated;
/** The pollhead structure */
static pollhead_t           g_PollHead;
/** The IRQ Mutex */
static kmutex_t             g_IrqMtx;
/** Layered device handle for kernel keep-attached opens */
static ldi_handle_t         g_LdiHandle = NULL;
/** Ref counting for IDCOpen calls */
static uint64_t             g_cLdiOpens = 0;
/** The Mutex protecting the LDI handle in IDC opens */
static kmutex_t             g_LdiMtx;

/**
 * Kernel entry points
 */
int _init(void)
{
    /*
     * Initialize IPRT R0 driver, which internally calls OS-specific r0 init.
     */
    int rc = RTR0Init(0);
    if (RT_SUCCESS(rc))
    {
        PRTLOGGER pRelLogger;
        static const char * const s_apszGroups[] = VBOX_LOGGROUP_NAMES;
        rc = RTLogCreate(&pRelLogger, 0 /* fFlags */, "all",
                         "VBOX_RELEASE_LOG", RT_ELEMENTS(s_apszGroups), s_apszGroups,
                         RTLOGDEST_STDOUT | RTLOGDEST_DEBUGGER, NULL);
        if (RT_SUCCESS(rc))
            RTLogRelSetDefaultInstance(pRelLogger);
        else
            cmn_err(CE_NOTE, "failed to initialize driver logging rc=%d!\n", rc);

        mutex_init(&g_LdiMtx, NULL, MUTEX_DRIVER, NULL);

        /*
         * Prevent module autounloading.
         */
        modctl_t *pModCtl = mod_getctl(&g_VBoxGuestSolarisModLinkage);
        if (pModCtl)
            pModCtl->mod_loadflags |= MOD_NOAUTOUNLOAD;
        else
            LogRel((DEVICE_NAME ":failed to disable autounloading!\n"));

        rc = ddi_soft_state_init(&g_pVBoxGuestSolarisState, sizeof(vboxguest_state_t), 1);
        if (!rc)
        {
            rc = mod_install(&g_VBoxGuestSolarisModLinkage);
            if (rc)
                ddi_soft_state_fini(&g_pVBoxGuestSolarisState);
        }
    }
    else
    {
        cmn_err(CE_NOTE, "_init: RTR0Init failed. rc=%d\n", rc);
        return EINVAL;
    }

    return rc;
}


int _fini(void)
{
    LogFlow((DEVICE_NAME ":_fini\n"));
    int rc = mod_remove(&g_VBoxGuestSolarisModLinkage);
    if (!rc)
        ddi_soft_state_fini(&g_pVBoxGuestSolarisState);

    RTLogDestroy(RTLogRelSetDefaultInstance(NULL));
    RTLogDestroy(RTLogSetDefaultInstance(NULL));

    mutex_destroy(&g_LdiMtx);

    RTR0Term();
    return rc;
}


int _info(struct modinfo *pModInfo)
{
    LogFlow((DEVICE_NAME ":_info\n"));
    return mod_info(&g_VBoxGuestSolarisModLinkage, pModInfo);
}


/**
 * Attach entry point, to attach a device to the system or resume it.
 *
 * @param   pDip            The module structure instance.
 * @param   enmCmd          Attach type (ddi_attach_cmd_t)
 *
 * @return  corresponding solaris error code.
 */
static int VBoxGuestSolarisAttach(dev_info_t *pDip, ddi_attach_cmd_t enmCmd)
{
    LogFlow((DEVICE_NAME "::Attach\n"));
    switch (enmCmd)
    {
        case DDI_ATTACH:
        {
            if (g_pDip)
            {
                LogRel((DEVICE_NAME "::Attach: Only one instance supported.\n"));
                return DDI_FAILURE;
            }

            int instance = ddi_get_instance(pDip);

            /*
             * Enable resources for PCI access.
             */
            ddi_acc_handle_t PciHandle;
            int rc = pci_config_setup(pDip, &PciHandle);
            if (rc == DDI_SUCCESS)
            {
                /*
                 * Map the register address space.
                 */
                caddr_t baseAddr;
                ddi_device_acc_attr_t deviceAttr;
                deviceAttr.devacc_attr_version = DDI_DEVICE_ATTR_V0;
                deviceAttr.devacc_attr_endian_flags = DDI_NEVERSWAP_ACC;
                deviceAttr.devacc_attr_dataorder = DDI_STRICTORDER_ACC;
                deviceAttr.devacc_attr_access = DDI_DEFAULT_ACC;
                rc = ddi_regs_map_setup(pDip, 1, &baseAddr, 0, 0, &deviceAttr, &g_PciIOHandle);
                if (rc == DDI_SUCCESS)
                {
                    /*
                     * Read size of the MMIO region.
                     */
                    g_uIOPortBase = (uintptr_t)baseAddr;
                    rc = ddi_dev_regsize(pDip, 2, &g_cbMMIO);
                    if (rc == DDI_SUCCESS)
                    {
                        rc = ddi_regs_map_setup(pDip, 2, &g_pMMIOBase, 0, g_cbMMIO, &deviceAttr,
                                        &g_PciMMIOHandle);
                        if (rc == DDI_SUCCESS)
                        {
                            /*
                             * Add IRQ of VMMDev.
                             */
                            rc = VBoxGuestSolarisAddIRQ(pDip);
                            if (rc == DDI_SUCCESS)
                            {
                                /*
                                 * Call the common device extension initializer.
                                 */
                                rc = VBoxGuestInitDevExt(&g_DevExt, g_uIOPortBase, g_pMMIOBase, g_cbMMIO,
#if ARCH_BITS == 64
                                                         VBOXOSTYPE_Solaris_x64,
#else
                                                         VBOXOSTYPE_Solaris,
#endif
                                                         VMMDEV_EVENT_MOUSE_POSITION_CHANGED);
                                if (RT_SUCCESS(rc))
                                {
                                    rc = ddi_create_minor_node(pDip, DEVICE_NAME, S_IFCHR, instance, DDI_PSEUDO, 0);
                                    if (rc == DDI_SUCCESS)
                                    {
                                        g_pDip = pDip;
                                        pci_config_teardown(&PciHandle);
                                        return DDI_SUCCESS;
                                    }

                                    LogRel((DEVICE_NAME "::Attach: ddi_create_minor_node failed.\n"));
                                    VBoxGuestDeleteDevExt(&g_DevExt);
                                }
                                else
                                    LogRel((DEVICE_NAME "::Attach: VBoxGuestInitDevExt failed.\n"));
                                VBoxGuestSolarisRemoveIRQ(pDip);
                            }
                            else
                                LogRel((DEVICE_NAME "::Attach: VBoxGuestSolarisAddIRQ failed.\n"));
                            ddi_regs_map_free(&g_PciMMIOHandle);
                        }
                        else
                            LogRel((DEVICE_NAME "::Attach: ddi_regs_map_setup for MMIO region failed.\n"));
                    }
                    else
                        LogRel((DEVICE_NAME "::Attach: ddi_dev_regsize for MMIO region failed.\n"));
                    ddi_regs_map_free(&g_PciIOHandle);
                }
                else
                    LogRel((DEVICE_NAME "::Attach: ddi_regs_map_setup for IOport failed.\n"));
                pci_config_teardown(&PciHandle);
            }
            else
                LogRel((DEVICE_NAME "::Attach: pci_config_setup failed rc=%d.\n", rc));
            return DDI_FAILURE;
        }

        case DDI_RESUME:
        {
            /** @todo implement resume for guest driver. */
            return DDI_SUCCESS;
        }

        default:
            return DDI_FAILURE;
    }
}


/**
 * Detach entry point, to detach a device to the system or suspend it.
 *
 * @param   pDip            The module structure instance.
 * @param   enmCmd          Attach type (ddi_attach_cmd_t)
 *
 * @return  corresponding solaris error code.
 */
static int VBoxGuestSolarisDetach(dev_info_t *pDip, ddi_detach_cmd_t enmCmd)
{
    LogFlow((DEVICE_NAME "::Detach\n"));
    switch (enmCmd)
    {
        case DDI_DETACH:
        {
            VBoxGuestSolarisRemoveIRQ(pDip);
            ddi_regs_map_free(&g_PciIOHandle);
            ddi_regs_map_free(&g_PciMMIOHandle);
            ddi_remove_minor_node(pDip, NULL);
            VBoxGuestDeleteDevExt(&g_DevExt);
            g_pDip = NULL;
            return DDI_SUCCESS;
        }

        case DDI_SUSPEND:
        {
            /** @todo implement suspend for guest driver. */
            return DDI_SUCCESS;
        }

        default:
            return DDI_FAILURE;
    }
}


/**
 * Info entry point, called by solaris kernel for obtaining driver info.
 *
 * @param   pDip            The module structure instance (do not use).
 * @param   enmCmd          Information request type.
 * @param   pvArg           Type specific argument.
 * @param   ppvResult       Where to store the requested info.
 *
 * @return  corresponding solaris error code.
 */
static int VBoxGuestSolarisGetInfo(dev_info_t *pDip, ddi_info_cmd_t enmCmd, void *pvArg, void **ppvResult)
{
    LogFlow((DEVICE_NAME "::GetInfo\n"));

    int rc = DDI_SUCCESS;
    switch (enmCmd)
    {
        case DDI_INFO_DEVT2DEVINFO:
            *ppvResult = (void *)g_pDip;
            break;

        case DDI_INFO_DEVT2INSTANCE:
            *ppvResult = (void *)(uintptr_t)ddi_get_instance(g_pDip);
            break;

        default:
            rc = DDI_FAILURE;
            break;
    }

    NOREF(pvArg);
    return rc;
}


/**
 * User context entry points
 */
static int VBoxGuestSolarisOpen(dev_t *pDev, int fFlag, int fType, cred_t *pCred)
{
    int                 rc;
    PVBOXGUESTSESSION   pSession = NULL;

    LogFlow((DEVICE_NAME "::Open\n"));

    /*
     * Verify we are being opened as a character device.
     */
    if (fType != OTYP_CHR)
        return EINVAL;

    vboxguest_state_t *pState = NULL;
    unsigned iOpenInstance;
    for (iOpenInstance = 0; iOpenInstance < 4096; iOpenInstance++)
    {
        if (    !ddi_get_soft_state(g_pVBoxGuestSolarisState, iOpenInstance) /* faster */
            &&  ddi_soft_state_zalloc(g_pVBoxGuestSolarisState, iOpenInstance) == DDI_SUCCESS)
        {
            pState = ddi_get_soft_state(g_pVBoxGuestSolarisState, iOpenInstance);
            break;
        }
    }
    if (!pState)
    {
        Log((DEVICE_NAME "::Open: too many open instances."));
        return ENXIO;
    }

    /*
     * Create a new session.
     */
    rc = VBoxGuestCreateUserSession(&g_DevExt, &pSession);
    if (RT_SUCCESS(rc))
    {
        pState->pvProcRef = proc_ref();
        pState->pSession = pSession;
        *pDev = makedevice(getmajor(*pDev), iOpenInstance);
        Log((DEVICE_NAME "::Open: pSession=%p pState=%p pid=%d\n", pSession, pState, (int)RTProcSelf()));
        return 0;
    }

    /* Failed, clean up. */
    ddi_soft_state_free(g_pVBoxGuestSolarisState, iOpenInstance);

    LogRel((DEVICE_NAME "::Open: VBoxGuestCreateUserSession failed. rc=%d\n", rc));
    return EFAULT;
}


static int VBoxGuestSolarisClose(dev_t Dev, int flag, int fType, cred_t *pCred)
{
    LogFlow((DEVICE_NAME "::Close pid=%d\n", (int)RTProcSelf()));

    PVBOXGUESTSESSION pSession = NULL;
    vboxguest_state_t *pState = ddi_get_soft_state(g_pVBoxGuestSolarisState, getminor(Dev));
    if (!pState)
    {
        Log((DEVICE_NAME "::Close: failed to get pState.\n"));
        return EFAULT;
    }

    proc_unref(pState->pvProcRef);
    pSession = pState->pSession;
    pState->pSession = NULL;
    Log((DEVICE_NAME "::Close: pSession=%p pState=%p\n", pSession, pState));
    ddi_soft_state_free(g_pVBoxGuestSolarisState, getminor(Dev));
    if (!pSession)
    {
        Log((DEVICE_NAME "::Close: failed to get pSession.\n"));
        return EFAULT;
    }

    /*
     * Close the session.
     */
    VBoxGuestCloseSession(&g_DevExt, pSession);
    return 0;
}


static int VBoxGuestSolarisRead(dev_t Dev, struct uio *pUio, cred_t *pCred)
{
    LogFlow((DEVICE_NAME "::Read\n"));

    vboxguest_state_t *pState = ddi_get_soft_state(g_pVBoxGuestSolarisState, getminor(Dev));
    if (!pState)
    {
        Log((DEVICE_NAME "::Close: failed to get pState.\n"));
        return EFAULT;
    }

    PVBOXGUESTSESSION pSession = pState->pSession;
    uint32_t u32CurSeq = ASMAtomicUoReadU32(&g_DevExt.u32MousePosChangedSeq);
    if (pSession->u32MousePosChangedSeq != u32CurSeq)
        pSession->u32MousePosChangedSeq = u32CurSeq;

    return 0;
}


static int VBoxGuestSolarisWrite(dev_t Dev, struct uio *pUio, cred_t *pCred)
{
    LogFlow((DEVICE_NAME "::Write\n"));
    return 0;
}


/** @def IOCPARM_LEN
 * Gets the length from the ioctl number.
 * This is normally defined by sys/ioccom.h on BSD systems...
 */
#ifndef IOCPARM_LEN
# define IOCPARM_LEN(Code)                      (((Code) >> 16) & IOCPARM_MASK)
#endif


/**
 * Driver ioctl, an alternate entry point for this character driver.
 *
 * @param   Dev             Device number
 * @param   Cmd             Operation identifier
 * @param   pArg            Arguments from user to driver
 * @param   Mode            Information bitfield (read/write, address space etc.)
 * @param   pCred           User credentials
 * @param   pVal            Return value for calling process.
 *
 * @return  corresponding solaris error code.
 */
static int VBoxGuestSolarisIOCtl(dev_t Dev, int Cmd, intptr_t pArg, int Mode, cred_t *pCred, int *pVal)
{
    LogFlow((DEVICE_NAME ":VBoxGuestSolarisIOCtl\n"));

    /*
     * Get the session from the soft state item.
     */
    vboxguest_state_t *pState = ddi_get_soft_state(g_pVBoxGuestSolarisState, getminor(Dev));
    if (!pState)
    {
        LogRel((DEVICE_NAME "::IOCtl: no state data for %d\n", getminor(Dev)));
        return EINVAL;
    }

    PVBOXGUESTSESSION pSession = pState->pSession;
    if (!pSession)
    {
        LogRel((DEVICE_NAME "::IOCtl: no session data for %d\n", getminor(Dev)));
        return EINVAL;
    }

    /*
     * Read and validate the request wrapper.
     */
    VBGLBIGREQ ReqWrap;
    if (IOCPARM_LEN(Cmd) != sizeof(ReqWrap))
    {
        LogRel((DEVICE_NAME "::IOCtl: bad request %#x size=%d expected=%d\n", Cmd, IOCPARM_LEN(Cmd), sizeof(ReqWrap)));
        return ENOTTY;
    }

    int rc = ddi_copyin((void *)pArg, &ReqWrap, sizeof(ReqWrap), Mode);
    if (RT_UNLIKELY(rc))
    {
        LogRel((DEVICE_NAME "::IOCtl: ddi_copyin failed to read header pArg=%p Cmd=%d. rc=%#x.\n", pArg, Cmd, rc));
        return EINVAL;
    }

    if (ReqWrap.u32Magic != VBGLBIGREQ_MAGIC)
    {
        LogRel((DEVICE_NAME "::IOCtl: bad magic %#x; pArg=%p Cmd=%#x.\n", ReqWrap.u32Magic, pArg, Cmd));
        return EINVAL;
    }
    if (RT_UNLIKELY(ReqWrap.cbData > _1M*16))
    {
        LogRel((DEVICE_NAME "::IOCtl: bad size %#x; pArg=%p Cmd=%#x.\n", ReqWrap.cbData, pArg, Cmd));
        return EINVAL;
    }

    /*
     * Read the request payload if any; requests like VBOXGUEST_IOCTL_CANCEL_ALL_WAITEVENTS have no data payload.
     */
    void *pvBuf = NULL;
    if (RT_LIKELY(ReqWrap.cbData > 0))
    {
        pvBuf = RTMemTmpAlloc(ReqWrap.cbData);
        if (RT_UNLIKELY(!pvBuf))
        {
            LogRel((DEVICE_NAME "::IOCtl: RTMemTmpAlloc failed to alloc %d bytes.\n", ReqWrap.cbData));
            return ENOMEM;
        }

        rc = ddi_copyin((void *)(uintptr_t)ReqWrap.pvDataR3, pvBuf, ReqWrap.cbData, Mode);
        if (RT_UNLIKELY(rc))
        {
            RTMemTmpFree(pvBuf);
            LogRel((DEVICE_NAME "::IOCtl: ddi_copyin failed; pvBuf=%p pArg=%p Cmd=%d. rc=%d\n", pvBuf, pArg, Cmd, rc));
            return EFAULT;
        }
        if (RT_UNLIKELY(!VALID_PTR(pvBuf)))
        {
            RTMemTmpFree(pvBuf);
            LogRel((DEVICE_NAME "::IOCtl: pvBuf invalid pointer %p\n", pvBuf));
            return EINVAL;
        }
    }
    Log((DEVICE_NAME "::IOCtl: pSession=%p pid=%d.\n", pSession, (int)RTProcSelf()));

    /*
     * Process the IOCtl.
     */
    size_t cbDataReturned = 0;
    rc = VBoxGuestCommonIOCtl(Cmd, &g_DevExt, pSession, pvBuf, ReqWrap.cbData, &cbDataReturned);
    if (RT_SUCCESS(rc))
    {
        rc = 0;
        if (RT_UNLIKELY(cbDataReturned > ReqWrap.cbData))
        {
            LogRel((DEVICE_NAME "::IOCtl: too much output data %d expected %d\n", cbDataReturned, ReqWrap.cbData));
            cbDataReturned = ReqWrap.cbData;
        }
        if (cbDataReturned > 0)
        {
            rc = ddi_copyout(pvBuf, (void *)(uintptr_t)ReqWrap.pvDataR3, cbDataReturned, Mode);
            if (RT_UNLIKELY(rc))
            {
                LogRel((DEVICE_NAME "::IOCtl: ddi_copyout failed; pvBuf=%p pArg=%p cbDataReturned=%u Cmd=%d. rc=%d\n",
                        pvBuf, pArg, cbDataReturned, Cmd, rc));
                rc = EFAULT;
            }
        }
    }
    else
    {
        /*
         * We Log() instead of LogRel() here because VBOXGUEST_IOCTL_WAITEVENT can return VERR_TIMEOUT,
         * VBOXGUEST_IOCTL_CANCEL_ALL_EVENTS can return VERR_INTERRUPTED and possibly more in the future;
         * which are not really failures that require logging.
         */
        Log((DEVICE_NAME "::IOCtl: VBoxGuestCommonIOCtl failed. Cmd=%#x rc=%d\n", Cmd, rc));
        rc = RTErrConvertToErrno(rc);
    }
    *pVal = rc;
    if (pvBuf)
        RTMemTmpFree(pvBuf);
    return rc;
}


static int VBoxGuestSolarisPoll(dev_t Dev, short fEvents, int fAnyYet, short *pReqEvents, struct pollhead **ppPollHead)
{
    LogFlow((DEVICE_NAME "::Poll: fEvents=%d fAnyYet=%d\n", fEvents, fAnyYet));

    vboxguest_state_t *pState = ddi_get_soft_state(g_pVBoxGuestSolarisState, getminor(Dev));
    if (RT_LIKELY(pState))
    {
        PVBOXGUESTSESSION pSession  = (PVBOXGUESTSESSION)pState->pSession;
        uint32_t u32CurSeq = ASMAtomicUoReadU32(&g_DevExt.u32MousePosChangedSeq);
        if (pSession->u32MousePosChangedSeq != u32CurSeq)
        {
            *pReqEvents |= (POLLIN | POLLRDNORM);
            pSession->u32MousePosChangedSeq = u32CurSeq;
        }
        else
        {
            *pReqEvents = 0;
            if (!fAnyYet)
                *ppPollHead = &g_PollHead;
        }

        return 0;
    }
    else
    {
        Log((DEVICE_NAME "::Poll: no state data for %d\n", getminor(Dev)));
        return EINVAL;
    }
}


/**
 * Sets IRQ for VMMDev.
 *
 * @returns Solaris error code.
 * @param   pDip     Pointer to the device info structure.
 */
static int VBoxGuestSolarisAddIRQ(dev_info_t *pDip)
{
    LogFlow((DEVICE_NAME "::AddIRQ: pDip=%p\n", pDip));

    int IntrType = 0;
    int rc = ddi_intr_get_supported_types(pDip, &IntrType);
    if (rc == DDI_SUCCESS)
    {
        /* We won't need to bother about MSIs. */
        if (IntrType & DDI_INTR_TYPE_FIXED)
        {
            int IntrCount = 0;
            rc = ddi_intr_get_nintrs(pDip, IntrType, &IntrCount);
            if (   rc == DDI_SUCCESS
                && IntrCount > 0)
            {
                int IntrAvail = 0;
                rc = ddi_intr_get_navail(pDip, IntrType, &IntrAvail);
                if (   rc == DDI_SUCCESS
                    && IntrAvail > 0)
                {
                    /* Allocated kernel memory for the interrupt handles. The allocation size is stored internally. */
                    g_pIntr = RTMemAlloc(IntrCount * sizeof(ddi_intr_handle_t));
                    if (g_pIntr)
                    {
                        int IntrAllocated;
                        rc = ddi_intr_alloc(pDip, g_pIntr, IntrType, 0, IntrCount, &IntrAllocated, DDI_INTR_ALLOC_NORMAL);
                        if (   rc == DDI_SUCCESS
                            && IntrAllocated > 0)
                        {
                            g_cIntrAllocated = IntrAllocated;
                            uint_t uIntrPriority;
                            rc = ddi_intr_get_pri(g_pIntr[0], &uIntrPriority);
                            if (rc == DDI_SUCCESS)
                            {
                                /* Initialize the mutex. */
                                mutex_init(&g_IrqMtx, NULL, MUTEX_DRIVER, DDI_INTR_PRI(uIntrPriority));

                                /* Assign interrupt handler functions and enable interrupts. */
                                for (int i = 0; i < IntrAllocated; i++)
                                {
                                    rc = ddi_intr_add_handler(g_pIntr[i], (ddi_intr_handler_t *)VBoxGuestSolarisISR,
                                                            NULL /* No Private Data */, NULL);
                                    if (rc == DDI_SUCCESS)
                                        rc = ddi_intr_enable(g_pIntr[i]);
                                    if (rc != DDI_SUCCESS)
                                    {
                                        /* Changing local IntrAllocated to hold so-far allocated handles for freeing. */
                                        IntrAllocated = i;
                                        break;
                                    }
                                }
                                if (rc == DDI_SUCCESS)
                                    return rc;

                                /* Remove any assigned handlers */
                                LogRel((DEVICE_NAME ":failed to assign IRQs allocated=%d\n", IntrAllocated));
                                for (int x = 0; x < IntrAllocated; x++)
                                    ddi_intr_remove_handler(g_pIntr[x]);
                            }
                            else
                                LogRel((DEVICE_NAME "::AddIRQ: failed to get priority of interrupt. rc=%d\n", rc));

                            /* Remove allocated IRQs, too bad we can free only one handle at a time. */
                            for (int k = 0; k < g_cIntrAllocated; k++)
                                ddi_intr_free(g_pIntr[k]);
                        }
                        else
                            LogRel((DEVICE_NAME "::AddIRQ: failed to allocated IRQs. count=%d\n", IntrCount));
                        RTMemFree(g_pIntr);
                    }
                    else
                        LogRel((DEVICE_NAME "::AddIRQ: failed to allocated IRQs. count=%d\n", IntrCount));
                }
                else
                    LogRel((DEVICE_NAME "::AddIRQ: failed to get or insufficient available IRQs. rc=%d IntrAvail=%d\n", rc, IntrAvail));
            }
            else
                LogRel((DEVICE_NAME "::AddIRQ: failed to get or insufficient number of IRQs. rc=%d IntrCount=%d\n", rc, IntrCount));
        }
        else
            LogRel((DEVICE_NAME "::AddIRQ: invalid irq type. IntrType=%#x\n", IntrType));
    }
    else
        LogRel((DEVICE_NAME "::AddIRQ: failed to get supported interrupt types\n"));
    return rc;
}


/**
 * Removes IRQ for VMMDev.
 *
 * @param   pDip     Pointer to the device info structure.
 */
static void VBoxGuestSolarisRemoveIRQ(dev_info_t *pDip)
{
    LogFlow((DEVICE_NAME "::RemoveIRQ:\n"));

    for (int i = 0; i < g_cIntrAllocated; i++)
    {
        int rc = ddi_intr_disable(g_pIntr[i]);
        if (rc == DDI_SUCCESS)
        {
            rc = ddi_intr_remove_handler(g_pIntr[i]);
            if (rc == DDI_SUCCESS)
                ddi_intr_free(g_pIntr[i]);
        }
    }
    RTMemFree(g_pIntr);
    mutex_destroy(&g_IrqMtx);
}


/**
 * Interrupt Service Routine for VMMDev.
 *
 * @param   Arg     Private data (unused, will be NULL).
 * @returns DDI_INTR_CLAIMED if it's our interrupt, DDI_INTR_UNCLAIMED if it isn't.
 */
static uint_t VBoxGuestSolarisISR(caddr_t Arg)
{
    LogFlow((DEVICE_NAME "::ISR:\n"));

    mutex_enter(&g_IrqMtx);
    bool fOurIRQ = VBoxGuestCommonISR(&g_DevExt);
    mutex_exit(&g_IrqMtx);

    return fOurIRQ ? DDI_INTR_CLAIMED : DDI_INTR_UNCLAIMED;
}


void VBoxGuestNativeISRMousePollEvent(PVBOXGUESTDEVEXT pDevExt)
{
    LogFlow((DEVICE_NAME "::NativeISRMousePollEvent:\n"));

    /*
     * Wake up poll waiters.
     */
    pollwakeup(&g_PollHead, POLLIN | POLLRDNORM);
}


/* Common code that depend on g_DevExt. */
#include "VBoxGuestIDC-unix.c.h"

