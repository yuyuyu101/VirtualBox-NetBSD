/* $Id: PDMRCDevice.cpp $ */
/** @file
 * PDM - Pluggable Device and Driver Manager, RC Device parts.
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
#define LOG_GROUP LOG_GROUP_PDM_DEVICE
#include "PDMInternal.h"
#include <VBox/vmm/pdm.h>
#include <VBox/vmm/pgm.h>
#include <VBox/vmm/mm.h>
#include <VBox/vmm/vm.h>
#include <VBox/vmm/vmm.h>
#include <VBox/vmm/patm.h>

#include <VBox/log.h>
#include <VBox/err.h>
#include <iprt/asm.h>
#include <iprt/assert.h>
#include <iprt/string.h>

#include "dtrace/VBoxVMM.h"
#include "PDMInline.h"


/*******************************************************************************
*   Global Variables                                                           *
*******************************************************************************/
RT_C_DECLS_BEGIN
extern DECLEXPORT(const PDMDEVHLPRC)    g_pdmRCDevHlp;
extern DECLEXPORT(const PDMPICHLPRC)    g_pdmRCPicHlp;
extern DECLEXPORT(const PDMAPICHLPRC)   g_pdmRCApicHlp;
extern DECLEXPORT(const PDMIOAPICHLPRC) g_pdmRCIoApicHlp;
extern DECLEXPORT(const PDMPCIHLPRC)    g_pdmRCPciHlp;
extern DECLEXPORT(const PDMHPETHLPRC)   g_pdmRCHpetHlp;
extern DECLEXPORT(const PDMDRVHLPRC)    g_pdmRCDrvHlp;
/** @todo missing PDMPCIRAWHLPRC  */
RT_C_DECLS_END


/*******************************************************************************
*   Prototypes                                                                 *
*******************************************************************************/
static int pdmRCDevHlp_PhysRead(PPDMDEVINS pDevIns, RTGCPHYS GCPhys, void *pvBuf, size_t cbRead);
static int pdmRCDevHlp_PhysWrite(PPDMDEVINS pDevIns, RTGCPHYS GCPhys, const void *pvBuf, size_t cbWrite);


/*******************************************************************************
*   Internal Functions                                                         *
*******************************************************************************/
static bool pdmRCIsaSetIrq(PVM pVM, int iIrq, int iLevel, uint32_t uTagSrc);


/** @name Raw-Mode Context Device Helpers
 * @{
 */

/** @interface_method_impl{PDMDEVHLPRC,pfnPCIPhysRead} */
static DECLCALLBACK(int) pdmRCDevHlp_PCIPhysRead(PPDMDEVINS pDevIns, RTGCPHYS GCPhys, void *pvBuf, size_t cbRead)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_PCIPhysRead: caller=%p/%d: GCPhys=%RGp pvBuf=%p cbRead=%#x\n",
             pDevIns, pDevIns->iInstance, GCPhys, pvBuf, cbRead));

    PCIDevice *pPciDev = pDevIns->Internal.s.pPciDeviceRC;
    AssertPtrReturn(pPciDev, VERR_INVALID_POINTER);

    if (!PCIDevIsBusmaster(pPciDev))
    {
#ifdef DEBUG
        LogFlow(("%s: %RU16:%RU16: No bus master (anymore), skipping read %p (%z)\n", __FUNCTION__,
                 PCIDevGetVendorId(pPciDev), PCIDevGetDeviceId(pPciDev), pvBuf, cbRead));
#endif
        return VINF_PDM_PCI_PHYS_READ_BM_DISABLED;
    }

    return pdmRCDevHlp_PhysRead(pDevIns, GCPhys, pvBuf, cbRead);
}


/** @interface_method_impl{PDMDEVHLPRC,pfnPCIPhysRead} */
static DECLCALLBACK(int) pdmRCDevHlp_PCIPhysWrite(PPDMDEVINS pDevIns, RTGCPHYS GCPhys, const void *pvBuf, size_t cbWrite)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_PCIPhysWrite: caller=%p/%d: GCPhys=%RGp pvBuf=%p cbWrite=%#x\n",
             pDevIns, pDevIns->iInstance, GCPhys, pvBuf, cbWrite));

    PCIDevice *pPciDev = pDevIns->Internal.s.pPciDeviceRC;
    AssertPtrReturn(pPciDev, VERR_INVALID_POINTER);

    if (!PCIDevIsBusmaster(pPciDev))
    {
#ifdef DEBUG
        LogFlow(("%s: %RU16:%RU16: No bus master (anymore), skipping write %p (%z)\n", __FUNCTION__,
                 PCIDevGetVendorId(pPciDev), PCIDevGetDeviceId(pPciDev), pvBuf, cbWrite));
#endif
        return VINF_PDM_PCI_PHYS_WRITE_BM_DISABLED;
    }

    return pdmRCDevHlp_PhysWrite(pDevIns, GCPhys, pvBuf, cbWrite);
}


/** @interface_method_impl{PDMDEVHLPRC,pfnPCISetIrq} */
static DECLCALLBACK(void) pdmRCDevHlp_PCISetIrq(PPDMDEVINS pDevIns, int iIrq, int iLevel)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_PCISetIrq: caller=%p/%d: iIrq=%d iLevel=%d\n", pDevIns, pDevIns->iInstance, iIrq, iLevel));

    PVM         pVM     = pDevIns->Internal.s.pVMRC;
    PPCIDEVICE  pPciDev = pDevIns->Internal.s.pPciDeviceRC;
    PPDMPCIBUS  pPciBus = pDevIns->Internal.s.pPciBusRC;

    pdmLock(pVM);
    uint32_t uTagSrc;
    if (iLevel & PDM_IRQ_LEVEL_HIGH)
    {
        pDevIns->Internal.s.uLastIrqTag = uTagSrc = pdmCalcIrqTag(pVM, pDevIns->idTracing);
        if (iLevel == PDM_IRQ_LEVEL_HIGH)
            VBOXVMM_PDM_IRQ_HIGH(VMMGetCpu(pVM), RT_LOWORD(uTagSrc), RT_HIWORD(uTagSrc));
        else
            VBOXVMM_PDM_IRQ_HILO(VMMGetCpu(pVM), RT_LOWORD(uTagSrc), RT_HIWORD(uTagSrc));
    }
    else
        uTagSrc = pDevIns->Internal.s.uLastIrqTag;

    if (    pPciDev
        &&  pPciBus
        &&  pPciBus->pDevInsRC)
    {
        pPciBus->pfnSetIrqRC(pPciBus->pDevInsRC, pPciDev, iIrq, iLevel, uTagSrc);

        pdmUnlock(pVM);

        if (iLevel == PDM_IRQ_LEVEL_LOW)
            VBOXVMM_PDM_IRQ_LOW(VMMGetCpu(pVM), RT_LOWORD(uTagSrc), RT_HIWORD(uTagSrc));
    }
    else
    {
        pdmUnlock(pVM);

        /* queue for ring-3 execution. */
        PPDMDEVHLPTASK pTask = (PPDMDEVHLPTASK)PDMQueueAlloc(pVM->pdm.s.pDevHlpQueueRC);
        AssertReturnVoid(pTask);

        pTask->enmOp = PDMDEVHLPTASKOP_PCI_SET_IRQ;
        pTask->pDevInsR3 = PDMDEVINS_2_R3PTR(pDevIns);
        pTask->u.SetIRQ.iIrq = iIrq;
        pTask->u.SetIRQ.iLevel = iLevel;
        pTask->u.SetIRQ.uTagSrc = uTagSrc;

        PDMQueueInsertEx(pVM->pdm.s.pDevHlpQueueRC, &pTask->Core, 0);
    }

    LogFlow(("pdmRCDevHlp_PCISetIrq: caller=%p/%d: returns void; uTagSrc=%#x\n", pDevIns, pDevIns->iInstance, uTagSrc));
}


/** @interface_method_impl{PDMDRVHLPRC,pfnPCISetIrq} */
static DECLCALLBACK(void) pdmRCDevHlp_ISASetIrq(PPDMDEVINS pDevIns, int iIrq, int iLevel)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_ISASetIrq: caller=%p/%d: iIrq=%d iLevel=%d\n", pDevIns, pDevIns->iInstance, iIrq, iLevel));
    PVM pVM = pDevIns->Internal.s.pVMRC;

    pdmLock(pVM);
    uint32_t uTagSrc;
    if (iLevel & PDM_IRQ_LEVEL_HIGH)
    {
        pDevIns->Internal.s.uLastIrqTag = uTagSrc = pdmCalcIrqTag(pVM, pDevIns->idTracing);
        if (iLevel == PDM_IRQ_LEVEL_HIGH)
            VBOXVMM_PDM_IRQ_HIGH(VMMGetCpu(pVM), RT_LOWORD(uTagSrc), RT_HIWORD(uTagSrc));
        else
            VBOXVMM_PDM_IRQ_HILO(VMMGetCpu(pVM), RT_LOWORD(uTagSrc), RT_HIWORD(uTagSrc));
    }
    else
        uTagSrc = pDevIns->Internal.s.uLastIrqTag;

    bool fRc = pdmRCIsaSetIrq(pVM, iIrq, iLevel, uTagSrc);

    if (iLevel == PDM_IRQ_LEVEL_LOW && fRc)
        VBOXVMM_PDM_IRQ_LOW(VMMGetCpu(pVM), RT_LOWORD(uTagSrc), RT_HIWORD(uTagSrc));
    pdmUnlock(pVM);
    LogFlow(("pdmRCDevHlp_ISASetIrq: caller=%p/%d: returns void; uTagSrc=%#x\n", pDevIns, pDevIns->iInstance, uTagSrc));
}


/** @interface_method_impl{PDMDEVHLPRC,pfnPhysRead} */
static DECLCALLBACK(int) pdmRCDevHlp_PhysRead(PPDMDEVINS pDevIns, RTGCPHYS GCPhys, void *pvBuf, size_t cbRead)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_PhysRead: caller=%p/%d: GCPhys=%RGp pvBuf=%p cbRead=%#x\n",
             pDevIns, pDevIns->iInstance, GCPhys, pvBuf, cbRead));

    int rc = PGMPhysRead(pDevIns->Internal.s.pVMRC, GCPhys, pvBuf, cbRead);
    AssertRC(rc); /** @todo track down the users for this bugger. */

    Log(("pdmRCDevHlp_PhysRead: caller=%p/%d: returns %Rrc\n", pDevIns, pDevIns->iInstance, rc));
    return rc;
}


/** @interface_method_impl{PDMDEVHLPRC,pfnPhysWrite} */
static DECLCALLBACK(int) pdmRCDevHlp_PhysWrite(PPDMDEVINS pDevIns, RTGCPHYS GCPhys, const void *pvBuf, size_t cbWrite)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_PhysWrite: caller=%p/%d: GCPhys=%RGp pvBuf=%p cbWrite=%#x\n",
             pDevIns, pDevIns->iInstance, GCPhys, pvBuf, cbWrite));

    int rc = PGMPhysWrite(pDevIns->Internal.s.pVMRC, GCPhys, pvBuf, cbWrite);
    AssertRC(rc); /** @todo track down the users for this bugger. */

    Log(("pdmRCDevHlp_PhysWrite: caller=%p/%d: returns %Rrc\n", pDevIns, pDevIns->iInstance, rc));
    return rc;
}


/** @interface_method_impl{PDMDEVHLPRC,pfnA20IsEnabled} */
static DECLCALLBACK(bool) pdmRCDevHlp_A20IsEnabled(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_A20IsEnabled: caller=%p/%d:\n", pDevIns, pDevIns->iInstance));

    bool fEnabled = PGMPhysIsA20Enabled(VMMGetCpu0(pDevIns->Internal.s.pVMRC));

    Log(("pdmRCDevHlp_A20IsEnabled: caller=%p/%d: returns %RTbool\n", pDevIns, pDevIns->iInstance, fEnabled));
    return fEnabled;
}


/** @interface_method_impl{PDMDEVHLPRC,pfnVMState} */
static DECLCALLBACK(VMSTATE) pdmRCDevHlp_VMState(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);

    VMSTATE enmVMState = pDevIns->Internal.s.pVMRC->enmVMState;

    LogFlow(("pdmRCDevHlp_VMState: caller=%p/%d: returns %d\n", pDevIns, pDevIns->iInstance, enmVMState));
    return enmVMState;
}


/** @interface_method_impl{PDMDEVHLPRC,pfnVMSetError} */
static DECLCALLBACK(int) pdmRCDevHlp_VMSetError(PPDMDEVINS pDevIns, int rc, RT_SRC_POS_DECL, const char *pszFormat, ...)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    va_list args;
    va_start(args, pszFormat);
    int rc2 = VMSetErrorV(pDevIns->Internal.s.pVMRC, rc, RT_SRC_POS_ARGS, pszFormat, args); Assert(rc2 == rc); NOREF(rc2);
    va_end(args);
    return rc;
}


/** @interface_method_impl{PDMDEVHLPRC,pfnVMSetErrorV} */
static DECLCALLBACK(int) pdmRCDevHlp_VMSetErrorV(PPDMDEVINS pDevIns, int rc, RT_SRC_POS_DECL, const char *pszFormat, va_list va)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    int rc2 = VMSetErrorV(pDevIns->Internal.s.pVMRC, rc, RT_SRC_POS_ARGS, pszFormat, va); Assert(rc2 == rc); NOREF(rc2);
    return rc;
}


/** @interface_method_impl{PDMDEVHLPRC,pfnVMSetRuntimeError} */
static DECLCALLBACK(int) pdmRCDevHlp_VMSetRuntimeError(PPDMDEVINS pDevIns, uint32_t fFlags, const char *pszErrorId, const char *pszFormat, ...)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    va_list va;
    va_start(va, pszFormat);
    int rc = VMSetRuntimeErrorV(pDevIns->Internal.s.pVMRC, fFlags, pszErrorId, pszFormat, va);
    va_end(va);
    return rc;
}


/** @interface_method_impl{PDMDEVHLPRC,pfnVMSetErrorV} */
static DECLCALLBACK(int) pdmRCDevHlp_VMSetRuntimeErrorV(PPDMDEVINS pDevIns, uint32_t fFlags, const char *pszErrorId, const char *pszFormat, va_list va)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    int rc = VMSetRuntimeErrorV(pDevIns->Internal.s.pVMRC, fFlags, pszErrorId, pszFormat, va);
    return rc;
}


/** @interface_method_impl{PDMDEVHLPRC,pfnPATMSetMMIOPatchInfo} */
static DECLCALLBACK(int) pdmRCDevHlp_PATMSetMMIOPatchInfo(PPDMDEVINS pDevIns, RTGCPHYS GCPhys, RTGCPTR pCachedData)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_PATMSetMMIOPatchInfo: caller=%p/%d:\n", pDevIns, pDevIns->iInstance));

    return PATMSetMMIOPatchInfo(pDevIns->Internal.s.pVMRC, GCPhys, (RTRCPTR)(uintptr_t)pCachedData);
}


/** @interface_method_impl{PDMDEVHLPRC,pfnGetVM} */
static DECLCALLBACK(PVM)  pdmRCDevHlp_GetVM(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_GetVM: caller='%p'/%d\n", pDevIns, pDevIns->iInstance));
    return pDevIns->Internal.s.pVMRC;
}


/** @interface_method_impl{PDMDEVHLPRC,pfnGetVMCPU} */
static DECLCALLBACK(PVMCPU) pdmRCDevHlp_GetVMCPU(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_GetVMCPU: caller='%p'/%d\n", pDevIns, pDevIns->iInstance));
    return VMMGetCpu(pDevIns->Internal.s.pVMRC);
}


/** @interface_method_impl{PDMDEVHLPRC,pfnTMTimeVirtGet} */
static DECLCALLBACK(uint64_t) pdmRCDevHlp_TMTimeVirtGet(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_TMTimeVirtGet: caller='%p'/%d\n", pDevIns, pDevIns->iInstance));
    return TMVirtualGet(pDevIns->Internal.s.pVMRC);
}


/** @interface_method_impl{PDMDEVHLPRC,pfnTMTimeVirtGetFreq} */
static DECLCALLBACK(uint64_t) pdmRCDevHlp_TMTimeVirtGetFreq(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_TMTimeVirtGetFreq: caller='%p'/%d\n", pDevIns, pDevIns->iInstance));
    return TMVirtualGetFreq(pDevIns->Internal.s.pVMRC);
}


/** @interface_method_impl{PDMDEVHLPRC,pfnTMTimeVirtGetNano} */
static DECLCALLBACK(uint64_t) pdmRCDevHlp_TMTimeVirtGetNano(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCDevHlp_TMTimeVirtGetNano: caller='%p'/%d\n", pDevIns, pDevIns->iInstance));
    return TMVirtualToNano(pDevIns->Internal.s.pVMRC, TMVirtualGet(pDevIns->Internal.s.pVMRC));
}


/** @interface_method_impl{PDMDEVHLPRC,pfnDBGFTraceBuf} */
static DECLCALLBACK(RTTRACEBUF) pdmRCDevHlp_DBGFTraceBuf(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    RTTRACEBUF hTraceBuf = pDevIns->Internal.s.pVMRC->hTraceBufRC;
    LogFlow(("pdmRCDevHlp_DBGFTraceBuf: caller='%p'/%d: returns %p\n", pDevIns, pDevIns->iInstance, hTraceBuf));
    return hTraceBuf;
}


/**
 * The Raw-Mode Context Device Helper Callbacks.
 */
extern DECLEXPORT(const PDMDEVHLPRC) g_pdmRCDevHlp =
{
    PDM_DEVHLPRC_VERSION,
    pdmRCDevHlp_PCIPhysRead,
    pdmRCDevHlp_PCIPhysWrite,
    pdmRCDevHlp_PCISetIrq,
    pdmRCDevHlp_ISASetIrq,
    pdmRCDevHlp_PhysRead,
    pdmRCDevHlp_PhysWrite,
    pdmRCDevHlp_A20IsEnabled,
    pdmRCDevHlp_VMState,
    pdmRCDevHlp_VMSetError,
    pdmRCDevHlp_VMSetErrorV,
    pdmRCDevHlp_VMSetRuntimeError,
    pdmRCDevHlp_VMSetRuntimeErrorV,
    pdmRCDevHlp_PATMSetMMIOPatchInfo,
    pdmRCDevHlp_GetVM,
    pdmRCDevHlp_GetVMCPU,
    pdmRCDevHlp_TMTimeVirtGet,
    pdmRCDevHlp_TMTimeVirtGetFreq,
    pdmRCDevHlp_TMTimeVirtGetNano,
    pdmRCDevHlp_DBGFTraceBuf,
    PDM_DEVHLPRC_VERSION
};

/** @} */




/** @name PIC RC Helpers
 * @{
 */

/** @interface_method_impl{PDMPICHLPGC,pfnSetInterruptFF} */
static DECLCALLBACK(void) pdmRCPicHlp_SetInterruptFF(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    PVM pVM = pDevIns->Internal.s.pVMRC;

    if (pVM->pdm.s.Apic.pfnLocalInterruptRC)
    {
        LogFlow(("pdmRCPicHlp_SetInterruptFF: caller='%p'/%d: Setting local interrupt on LAPIC\n",
                 pDevIns, pDevIns->iInstance));
        /* Raise the LAPIC's LINT0 line instead of signaling the CPU directly. */
        pVM->pdm.s.Apic.pfnLocalInterruptRC(pVM->pdm.s.Apic.pDevInsRC, 0, 1);
        return;
    }

    PVMCPU pVCpu = &pVM->aCpus[0];  /* for PIC we always deliver to CPU 0, MP use APIC */

    LogFlow(("pdmRCPicHlp_SetInterruptFF: caller=%p/%d: VMMCPU_FF_INTERRUPT_PIC %d -> 1\n",
             pDevIns, pDevIns->iInstance, VMCPU_FF_ISSET(pVCpu, VMCPU_FF_INTERRUPT_PIC)));

    VMCPU_FF_SET(pVCpu, VMCPU_FF_INTERRUPT_PIC);
}


/** @interface_method_impl{PDMPICHLPGC,pfnClearInterruptFF} */
static DECLCALLBACK(void) pdmRCPicHlp_ClearInterruptFF(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    PVM pVM = pDevIns->Internal.s.CTX_SUFF(pVM);

    if (pVM->pdm.s.Apic.pfnLocalInterruptRC)
    {
        /* Raise the LAPIC's LINT0 line instead of signaling the CPU directly. */
        LogFlow(("pdmRCPicHlp_ClearInterruptFF: caller='%s'/%d: Clearing local interrupt on LAPIC\n",
                 pDevIns, pDevIns->iInstance));
        /* Lower the LAPIC's LINT0 line instead of signaling the CPU directly. */
        pVM->pdm.s.Apic.pfnLocalInterruptRC(pVM->pdm.s.Apic.pDevInsRC, 0, 0);
        return;
    }

    PVMCPU pVCpu = &pVM->aCpus[0];  /* for PIC we always deliver to CPU 0, MP use APIC */

    LogFlow(("pdmRCPicHlp_ClearInterruptFF: caller=%p/%d: VMCPU_FF_INTERRUPT_PIC %d -> 0\n",
             pDevIns, pDevIns->iInstance, VMCPU_FF_ISSET(pVCpu, VMCPU_FF_INTERRUPT_PIC)));

    VMCPU_FF_CLEAR(pVCpu, VMCPU_FF_INTERRUPT_PIC);
}


/** @interface_method_impl{PDMPICHLPGC,pfnLock} */
static DECLCALLBACK(int) pdmRCPicHlp_Lock(PPDMDEVINS pDevIns, int rc)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    return pdmLockEx(pDevIns->Internal.s.pVMRC, rc);
}


/** @interface_method_impl{PDMPICHLPGC,pfnUnlock} */
static DECLCALLBACK(void) pdmRCPicHlp_Unlock(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    pdmUnlock(pDevIns->Internal.s.pVMRC);
}


/**
 * The Raw-Mode Context PIC Helper Callbacks.
 */
extern DECLEXPORT(const PDMPICHLPRC) g_pdmRCPicHlp =
{
    PDM_PICHLPRC_VERSION,
    pdmRCPicHlp_SetInterruptFF,
    pdmRCPicHlp_ClearInterruptFF,
    pdmRCPicHlp_Lock,
    pdmRCPicHlp_Unlock,
    PDM_PICHLPRC_VERSION
};

/** @} */




/** @name APIC RC Helpers
 * @{
 */

/** @interface_method_impl{PDMAPICHLPRC,pfnSetInterruptFF} */
static DECLCALLBACK(void) pdmRCApicHlp_SetInterruptFF(PPDMDEVINS pDevIns, PDMAPICIRQ enmType, VMCPUID idCpu)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    PVM pVM = pDevIns->Internal.s.pVMRC;
    PVMCPU pVCpu = &pVM->aCpus[idCpu];

    AssertReturnVoid(idCpu < pVM->cCpus);

    LogFlow(("pdmRCApicHlp_SetInterruptFF: caller=%p/%d: VM_FF_INTERRUPT %d -> 1\n",
             pDevIns, pDevIns->iInstance, VMCPU_FF_ISSET(pVCpu, VMCPU_FF_INTERRUPT_APIC)));
    switch (enmType)
    {
        case PDMAPICIRQ_HARDWARE:
            VMCPU_FF_SET(pVCpu, VMCPU_FF_INTERRUPT_APIC);
            break;
        case PDMAPICIRQ_NMI:
            VMCPU_FF_SET(pVCpu, VMCPU_FF_INTERRUPT_NMI);
            break;
        case PDMAPICIRQ_SMI:
            VMCPU_FF_SET(pVCpu, VMCPU_FF_INTERRUPT_SMI);
            break;
        case PDMAPICIRQ_EXTINT:
            VMCPU_FF_SET(pVCpu, VMCPU_FF_INTERRUPT_PIC);
            break;
        default:
            AssertMsgFailed(("enmType=%d\n", enmType));
            break;
    }
}


/** @interface_method_impl{PDMAPICHLPRC,pfnClearInterruptFF} */
static DECLCALLBACK(void) pdmRCApicHlp_ClearInterruptFF(PPDMDEVINS pDevIns, PDMAPICIRQ enmType, VMCPUID idCpu)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    PVM pVM = pDevIns->Internal.s.pVMRC;
    PVMCPU pVCpu = &pVM->aCpus[idCpu];

    AssertReturnVoid(idCpu < pVM->cCpus);

    LogFlow(("pdmRCApicHlp_ClearInterruptFF: caller=%p/%d: VM_FF_INTERRUPT %d -> 0\n",
             pDevIns, pDevIns->iInstance, VMCPU_FF_ISSET(pVCpu, VMCPU_FF_INTERRUPT_APIC)));

    /* Note: NMI/SMI can't be cleared. */
    switch (enmType)
    {
        case PDMAPICIRQ_HARDWARE:
            VMCPU_FF_CLEAR(pVCpu, VMCPU_FF_INTERRUPT_APIC);
            break;
        case PDMAPICIRQ_EXTINT:
            VMCPU_FF_CLEAR(pVCpu, VMCPU_FF_INTERRUPT_PIC);
            break;
        default:
            AssertMsgFailed(("enmType=%d\n", enmType));
            break;
    }
}


/** @interface_method_impl{PDMAPICHLPRC,pfnCalcIrqTag} */
static DECLCALLBACK(uint32_t) pdmRCApicHlp_CalcIrqTag(PPDMDEVINS pDevIns, uint8_t u8Level)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    PVM pVM = pDevIns->Internal.s.pVMRC;

    pdmLock(pVM);

    uint32_t uTagSrc = pdmCalcIrqTag(pVM, pDevIns->idTracing);
    if (u8Level == PDM_IRQ_LEVEL_HIGH)
        VBOXVMM_PDM_IRQ_HIGH(VMMGetCpu(pVM), RT_LOWORD(uTagSrc), RT_HIWORD(uTagSrc));
    else
        VBOXVMM_PDM_IRQ_HILO(VMMGetCpu(pVM), RT_LOWORD(uTagSrc), RT_HIWORD(uTagSrc));


    pdmUnlock(pVM);
    LogFlow(("pdmRCApicHlp_CalcIrqTag: caller=%p/%d: returns %#x (u8Level=%d)\n",
             pDevIns, pDevIns->iInstance, uTagSrc, u8Level));
    return uTagSrc;
}


/** @interface_method_impl{PDMAPICHLPRC,pfnChangeFeature} */
static DECLCALLBACK(void) pdmRCApicHlp_ChangeFeature(PPDMDEVINS pDevIns, PDMAPICVERSION enmVersion)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    LogFlow(("pdmRCApicHlp_ChangeFeature: caller=%p/%d: version=%d\n", pDevIns, pDevIns->iInstance, (int)enmVersion));
    switch (enmVersion)
    {
        case PDMAPICVERSION_NONE:
            CPUMClearGuestCpuIdFeature(pDevIns->Internal.s.pVMRC, CPUMCPUIDFEATURE_APIC);
            CPUMClearGuestCpuIdFeature(pDevIns->Internal.s.pVMRC, CPUMCPUIDFEATURE_X2APIC);
            break;
        case PDMAPICVERSION_APIC:
            CPUMSetGuestCpuIdFeature(pDevIns->Internal.s.pVMRC, CPUMCPUIDFEATURE_APIC);
            CPUMClearGuestCpuIdFeature(pDevIns->Internal.s.pVMRC, CPUMCPUIDFEATURE_X2APIC);
            break;
        case PDMAPICVERSION_X2APIC:
            CPUMSetGuestCpuIdFeature(pDevIns->Internal.s.pVMRC, CPUMCPUIDFEATURE_X2APIC);
            CPUMSetGuestCpuIdFeature(pDevIns->Internal.s.pVMRC, CPUMCPUIDFEATURE_APIC);
            break;
        default:
            AssertMsgFailed(("Unknown APIC version: %d\n", (int)enmVersion));
    }
}


/** @interface_method_impl{PDMAPICHLPRC,pfnLock} */
static DECLCALLBACK(int) pdmRCApicHlp_Lock(PPDMDEVINS pDevIns, int rc)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    return pdmLockEx(pDevIns->Internal.s.pVMRC, rc);
}


/** @interface_method_impl{PDMAPICHLPRC,pfnUnlock} */
static DECLCALLBACK(void) pdmRCApicHlp_Unlock(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    pdmUnlock(pDevIns->Internal.s.pVMRC);
}


/** @interface_method_impl{PDMAPICHLPRC,pfnGetCpuId} */
static DECLCALLBACK(VMCPUID) pdmRCApicHlp_GetCpuId(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    return VMMGetCpuId(pDevIns->Internal.s.pVMRC);
}


/**
 * The Raw-Mode Context APIC Helper Callbacks.
 */
extern DECLEXPORT(const PDMAPICHLPRC) g_pdmRCApicHlp =
{
    PDM_APICHLPRC_VERSION,
    pdmRCApicHlp_SetInterruptFF,
    pdmRCApicHlp_ClearInterruptFF,
    pdmRCApicHlp_CalcIrqTag,
    pdmRCApicHlp_ChangeFeature,
    pdmRCApicHlp_Lock,
    pdmRCApicHlp_Unlock,
    pdmRCApicHlp_GetCpuId,
    PDM_APICHLPRC_VERSION
};

/** @} */




/** @name I/O APIC RC Helpers
 * @{
 */

/** @interface_method_impl{PDMIOAPICHLPRC,pfnApicBusDeliver} */
static DECLCALLBACK(int) pdmRCIoApicHlp_ApicBusDeliver(PPDMDEVINS pDevIns, uint8_t u8Dest, uint8_t u8DestMode, uint8_t u8DeliveryMode,
                                                       uint8_t iVector, uint8_t u8Polarity, uint8_t u8TriggerMode, uint32_t uTagSrc)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    PVM pVM = pDevIns->Internal.s.pVMRC;
    LogFlow(("pdmRCIoApicHlp_ApicBusDeliver: caller=%p/%d: u8Dest=%RX8 u8DestMode=%RX8 u8DeliveryMode=%RX8 iVector=%RX8 u8Polarity=%RX8 u8TriggerMode=%RX8 uTagSrc=%#x\n",
             pDevIns, pDevIns->iInstance, u8Dest, u8DestMode, u8DeliveryMode, iVector, u8Polarity, u8TriggerMode, uTagSrc));
    Assert(pVM->pdm.s.Apic.pDevInsRC);
    if (pVM->pdm.s.Apic.pfnBusDeliverRC)
        return pVM->pdm.s.Apic.pfnBusDeliverRC(pVM->pdm.s.Apic.pDevInsRC, u8Dest, u8DestMode, u8DeliveryMode, iVector,
                                               u8Polarity, u8TriggerMode, uTagSrc);
    return VINF_SUCCESS;
}


/** @interface_method_impl{PDMIOAPICHLPRC,pfnLock} */
static DECLCALLBACK(int) pdmRCIoApicHlp_Lock(PPDMDEVINS pDevIns, int rc)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    return pdmLockEx(pDevIns->Internal.s.pVMRC, rc);
}


/** @interface_method_impl{PDMIOAPICHLPRC,pfnUnlock} */
static DECLCALLBACK(void) pdmRCIoApicHlp_Unlock(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    pdmUnlock(pDevIns->Internal.s.pVMRC);
}


/**
 * The Raw-Mode Context I/O APIC Helper Callbacks.
 */
extern DECLEXPORT(const PDMIOAPICHLPRC) g_pdmRCIoApicHlp =
{
    PDM_IOAPICHLPRC_VERSION,
    pdmRCIoApicHlp_ApicBusDeliver,
    pdmRCIoApicHlp_Lock,
    pdmRCIoApicHlp_Unlock,
    PDM_IOAPICHLPRC_VERSION
};

/** @} */




/** @name PCI Bus RC Helpers
 * @{
 */

/** @interface_method_impl{PDMPCIHLPRC,pfnIsaSetIrq} */
static DECLCALLBACK(void) pdmRCPciHlp_IsaSetIrq(PPDMDEVINS pDevIns, int iIrq, int iLevel, uint32_t uTagSrc)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    Log4(("pdmRCPciHlp_IsaSetIrq: iIrq=%d iLevel=%d uTagSrc=%#x\n", iIrq, iLevel, uTagSrc));
    PVM pVM = pDevIns->Internal.s.pVMRC;

    pdmLock(pVM);
    pdmRCIsaSetIrq(pDevIns->Internal.s.pVMRC, iIrq, iLevel, uTagSrc);
    pdmUnlock(pVM);
}


/** @interface_method_impl{PDMPCIHLPRC,pfnIoApicSetIrq} */
static DECLCALLBACK(void) pdmRCPciHlp_IoApicSetIrq(PPDMDEVINS pDevIns, int iIrq, int iLevel, uint32_t uTagSrc)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    Log4(("pdmRCPciHlp_IoApicSetIrq: iIrq=%d iLevel=%d uTagSrc=%#x\n", iIrq, iLevel, uTagSrc));
    PVM pVM = pDevIns->Internal.s.pVMRC;

    if (pVM->pdm.s.IoApic.pDevInsRC)
    {
        pdmLock(pVM);
        pVM->pdm.s.IoApic.pfnSetIrqRC(pVM->pdm.s.IoApic.pDevInsRC, iIrq, iLevel, uTagSrc);
        pdmUnlock(pVM);
    }
    else if (pVM->pdm.s.IoApic.pDevInsR3)
    {
        /* queue for ring-3 execution. */
        PPDMDEVHLPTASK pTask = (PPDMDEVHLPTASK)PDMQueueAlloc(pVM->pdm.s.pDevHlpQueueRC);
        if (pTask)
        {
            pTask->enmOp = PDMDEVHLPTASKOP_IOAPIC_SET_IRQ;
            pTask->pDevInsR3 = NIL_RTR3PTR; /* not required */
            pTask->u.SetIRQ.iIrq = iIrq;
            pTask->u.SetIRQ.iLevel = iLevel;
            pTask->u.SetIRQ.uTagSrc = uTagSrc;

            PDMQueueInsertEx(pVM->pdm.s.pDevHlpQueueRC, &pTask->Core, 0);
        }
        else
            AssertMsgFailed(("We're out of devhlp queue items!!!\n"));
    }
}


/** @interface_method_impl{PDMPCIHLPRC,pfnIoApicSendMsi} */
static DECLCALLBACK(void) pdmRCPciHlp_IoApicSendMsi(PPDMDEVINS pDevIns, RTGCPHYS GCPhys, uint32_t uValue, uint32_t uTagSrc)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    Log4(("pdmRCPciHlp_IoApicSendMsi: GCPhys=%p uValue=%d uTagSrc=%#x\n", GCPhys, uValue, uTagSrc));
    PVM pVM = pDevIns->Internal.s.pVMRC;

    if (pVM->pdm.s.IoApic.pDevInsRC)
    {
        pdmLock(pVM);
        pVM->pdm.s.IoApic.pfnSendMsiRC(pVM->pdm.s.IoApic.pDevInsRC, GCPhys, uValue, uTagSrc);
        pdmUnlock(pVM);
    }
    else
    {
        AssertFatalMsgFailed(("Lazy bastarts!"));
    }
}


/** @interface_method_impl{PDMPCIHLPRC,pfnLock} */
static DECLCALLBACK(int) pdmRCPciHlp_Lock(PPDMDEVINS pDevIns, int rc)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    return pdmLockEx(pDevIns->Internal.s.pVMRC, rc);
}


/** @interface_method_impl{PDMPCIHLPRC,pfnUnlock} */
static DECLCALLBACK(void) pdmRCPciHlp_Unlock(PPDMDEVINS pDevIns)
{
    PDMDEV_ASSERT_DEVINS(pDevIns);
    pdmUnlock(pDevIns->Internal.s.pVMRC);
}


/**
 * The Raw-Mode Context PCI Bus Helper Callbacks.
 */
extern DECLEXPORT(const PDMPCIHLPRC) g_pdmRCPciHlp =
{
    PDM_PCIHLPRC_VERSION,
    pdmRCPciHlp_IsaSetIrq,
    pdmRCPciHlp_IoApicSetIrq,
    pdmRCPciHlp_IoApicSendMsi,
    pdmRCPciHlp_Lock,
    pdmRCPciHlp_Unlock,
    PDM_PCIHLPRC_VERSION, /* the end */
};

/** @} */




/** @name HPET RC Helpers
 * @{
 */


/**
 * The Raw-Mode Context HPET Helper Callbacks.
 */
extern DECLEXPORT(const PDMHPETHLPRC) g_pdmRCHpetHlp =
{
    PDM_HPETHLPRC_VERSION,
    PDM_HPETHLPRC_VERSION, /* the end */
};

/** @} */




/** @name Raw-Mode Context Driver Helpers
 * @{
 */

/** @interface_method_impl{PDMDRVHLPRC,pfnVMSetError} */
static DECLCALLBACK(int) pdmRCDrvHlp_VMSetError(PPDMDRVINS pDrvIns, int rc, RT_SRC_POS_DECL, const char *pszFormat, ...)
{
    PDMDRV_ASSERT_DRVINS(pDrvIns);
    va_list args;
    va_start(args, pszFormat);
    int rc2 = VMSetErrorV(pDrvIns->Internal.s.pVMRC, rc, RT_SRC_POS_ARGS, pszFormat, args); Assert(rc2 == rc); NOREF(rc2);
    va_end(args);
    return rc;
}


/** @interface_method_impl{PDMDRVHLPRC,pfnVMSetErrorV} */
static DECLCALLBACK(int) pdmRCDrvHlp_VMSetErrorV(PPDMDRVINS pDrvIns, int rc, RT_SRC_POS_DECL, const char *pszFormat, va_list va)
{
    PDMDRV_ASSERT_DRVINS(pDrvIns);
    int rc2 = VMSetErrorV(pDrvIns->Internal.s.pVMRC, rc, RT_SRC_POS_ARGS, pszFormat, va); Assert(rc2 == rc); NOREF(rc2);
    return rc;
}


/** @interface_method_impl{PDMDRVHLPRC,pfnVMSetRuntimeError} */
static DECLCALLBACK(int) pdmRCDrvHlp_VMSetRuntimeError(PPDMDRVINS pDrvIns, uint32_t fFlags, const char *pszErrorId, const char *pszFormat, ...)
{
    PDMDRV_ASSERT_DRVINS(pDrvIns);
    va_list va;
    va_start(va, pszFormat);
    int rc = VMSetRuntimeErrorV(pDrvIns->Internal.s.pVMRC, fFlags, pszErrorId, pszFormat, va);
    va_end(va);
    return rc;
}


/** @interface_method_impl{PDMDRVHLPRC,pfnVMSetErrorV} */
static DECLCALLBACK(int) pdmRCDrvHlp_VMSetRuntimeErrorV(PPDMDRVINS pDrvIns, uint32_t fFlags, const char *pszErrorId, const char *pszFormat, va_list va)
{
    PDMDRV_ASSERT_DRVINS(pDrvIns);
    int rc = VMSetRuntimeErrorV(pDrvIns->Internal.s.pVMRC, fFlags, pszErrorId, pszFormat, va);
    return rc;
}


/** @interface_method_impl{PDMDRVHLPRC,pfnAssertEMT} */
static DECLCALLBACK(bool) pdmRCDrvHlp_AssertEMT(PPDMDRVINS pDrvIns, const char *pszFile, unsigned iLine, const char *pszFunction)
{
    PDMDRV_ASSERT_DRVINS(pDrvIns);
    if (VM_IS_EMT(pDrvIns->Internal.s.pVMRC))
        return true;

    RTAssertMsg1Weak("AssertEMT", iLine, pszFile, pszFunction);
    RTAssertPanic();
    return false;
}


/** @interface_method_impl{PDMDRVHLPRC,pfnAssertOther} */
static DECLCALLBACK(bool) pdmRCDrvHlp_AssertOther(PPDMDRVINS pDrvIns, const char *pszFile, unsigned iLine, const char *pszFunction)
{
    PDMDRV_ASSERT_DRVINS(pDrvIns);
    if (!VM_IS_EMT(pDrvIns->Internal.s.pVMRC))
        return true;

    /* Note: While we don't have any other threads but EMT(0) in RC, might
       still have drive code compiled in which it shouldn't execute. */
    RTAssertMsg1Weak("AssertOther", iLine, pszFile, pszFunction);
    RTAssertPanic();
    return false;
}


/** @interface_method_impl{PDMDRVHLPRC,pfnFTSetCheckpoint} */
static DECLCALLBACK(int) pdmRCDrvHlp_FTSetCheckpoint(PPDMDRVINS pDrvIns, FTMCHECKPOINTTYPE enmType)
{
    PDMDRV_ASSERT_DRVINS(pDrvIns);
    return FTMSetCheckpoint(pDrvIns->Internal.s.pVMRC, enmType);
}


/**
 * The Raw-Mode Context Driver Helper Callbacks.
 */
extern DECLEXPORT(const PDMDRVHLPRC) g_pdmRCDrvHlp =
{
    PDM_DRVHLPRC_VERSION,
    pdmRCDrvHlp_VMSetError,
    pdmRCDrvHlp_VMSetErrorV,
    pdmRCDrvHlp_VMSetRuntimeError,
    pdmRCDrvHlp_VMSetRuntimeErrorV,
    pdmRCDrvHlp_AssertEMT,
    pdmRCDrvHlp_AssertOther,
    pdmRCDrvHlp_FTSetCheckpoint,
    PDM_DRVHLPRC_VERSION
};

/** @} */




/**
 * Sets an irq on the PIC and I/O APIC.
 *
 * @returns true if     delivered, false if postponed.
 * @param   pVM         Pointer to the VM.
 * @param   iIrq        The irq.
 * @param   iLevel      The new level.
 * @param   uTagSrc     The IRQ tag and source.
 *
 * @remarks The caller holds the PDM lock.
 */
static bool pdmRCIsaSetIrq(PVM pVM, int iIrq, int iLevel, uint32_t uTagSrc)
{
    if (RT_LIKELY(    (   pVM->pdm.s.IoApic.pDevInsRC
                       || !pVM->pdm.s.IoApic.pDevInsR3)
                  &&  (   pVM->pdm.s.Pic.pDevInsRC
                       || !pVM->pdm.s.Pic.pDevInsR3)))
    {
        if (pVM->pdm.s.Pic.pDevInsRC)
            pVM->pdm.s.Pic.pfnSetIrqRC(pVM->pdm.s.Pic.pDevInsRC, iIrq, iLevel, uTagSrc);
        if (pVM->pdm.s.IoApic.pDevInsRC)
            pVM->pdm.s.IoApic.pfnSetIrqRC(pVM->pdm.s.IoApic.pDevInsRC, iIrq, iLevel, uTagSrc);
        return true;
    }

    /* queue for ring-3 execution. */
    PPDMDEVHLPTASK pTask = (PPDMDEVHLPTASK)PDMQueueAlloc(pVM->pdm.s.pDevHlpQueueRC);
    AssertReturn(pTask, false);

    pTask->enmOp = PDMDEVHLPTASKOP_ISA_SET_IRQ;
    pTask->pDevInsR3 = NIL_RTR3PTR; /* not required */
    pTask->u.SetIRQ.iIrq = iIrq;
    pTask->u.SetIRQ.iLevel = iLevel;
    pTask->u.SetIRQ.uTagSrc = uTagSrc;

    PDMQueueInsertEx(pVM->pdm.s.pDevHlpQueueRC, &pTask->Core, 0);
    return false;
}

