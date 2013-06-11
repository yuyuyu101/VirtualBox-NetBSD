/* $Id: CPUMAllRegs.cpp $ */
/** @file
 * CPUM - CPU Monitor(/Manager) - Getters and Setters.
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
#define LOG_GROUP LOG_GROUP_CPUM
#include <VBox/vmm/cpum.h>
#include <VBox/vmm/patm.h>
#include <VBox/vmm/dbgf.h>
#include <VBox/vmm/pdm.h>
#include <VBox/vmm/pgm.h>
#include <VBox/vmm/mm.h>
#include <VBox/vmm/em.h>
#if defined(VBOX_WITH_RAW_MODE) && !defined(IN_RING0)
# include <VBox/vmm/selm.h>
#endif
#include "CPUMInternal.h"
#include <VBox/vmm/vm.h>
#include <VBox/err.h>
#include <VBox/dis.h>
#include <VBox/log.h>
#include <VBox/vmm/hwaccm.h>
#include <VBox/vmm/tm.h>
#include <iprt/assert.h>
#include <iprt/asm.h>
#include <iprt/asm-amd64-x86.h>
#ifdef IN_RING3
#include <iprt/thread.h>
#endif

/** Disable stack frame pointer generation here. */
#if defined(_MSC_VER) && !defined(DEBUG)
# pragma optimize("y", off)
#endif


/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
/**
 * Converts a CPUMCPU::Guest pointer into a VMCPU pointer.
 *
 * @returns Pointer to the Virtual CPU.
 * @param   a_pGuestCtx     Pointer to the guest context.
 */
#define CPUM_GUEST_CTX_TO_VMCPU(a_pGuestCtx) RT_FROM_MEMBER(a_pGuestCtx, VMCPU, cpum.s.Guest)

/**
 * Lazily loads the hidden parts of a selector register when using raw-mode.
 */
#if defined(VBOX_WITH_RAW_MODE) && !defined(IN_RING0)
# define CPUMSELREG_LAZY_LOAD_HIDDEN_PARTS(a_pVCpu, a_pSReg) \
    do \
    { \
        if (!CPUMSELREG_ARE_HIDDEN_PARTS_VALID(a_pVCpu, a_pSReg)) \
            cpumGuestLazyLoadHiddenSelectorReg(a_pVCpu, a_pSReg); \
    } while (0)
#else
# define CPUMSELREG_LAZY_LOAD_HIDDEN_PARTS(a_pVCpu, a_pSReg) \
    Assert(CPUMSELREG_ARE_HIDDEN_PARTS_VALID(a_pVCpu, a_pSReg));
#endif



#ifdef VBOX_WITH_RAW_MODE_NOT_R0

/**
 * Does the lazy hidden selector register loading.
 *
 * @param   pVCpu       The current Virtual CPU.
 * @param   pSReg       The selector register to lazily load hidden parts of.
 */
static void cpumGuestLazyLoadHiddenSelectorReg(PVMCPU pVCpu, PCPUMSELREG pSReg)
{
    Assert(!CPUMSELREG_ARE_HIDDEN_PARTS_VALID(pVCpu, pSReg));
    Assert(!HWACCMIsEnabled(pVCpu->CTX_SUFF(pVM)));
    Assert((uintptr_t)(pSReg - &pVCpu->cpum.s.Guest.es) < X86_SREG_COUNT);

    if (pVCpu->cpum.s.Guest.eflags.Bits.u1VM)
    {
        /* V8086 mode - Tightly controlled environment, no question about the limit or flags. */
        pSReg->Attr.u               = 0;
        pSReg->Attr.n.u4Type        = pSReg == &pVCpu->cpum.s.Guest.cs ? X86_SEL_TYPE_ER_ACC : X86_SEL_TYPE_RW_ACC;
        pSReg->Attr.n.u1DescType    = 1; /* code/data segment */
        pSReg->Attr.n.u2Dpl         = 3;
        pSReg->Attr.n.u1Present     = 1;
        pSReg->u32Limit             = 0x0000ffff;
        pSReg->u64Base              = (uint32_t)pSReg->Sel << 4;
        pSReg->ValidSel             = pSReg->Sel;
        pSReg->fFlags               = CPUMSELREG_FLAGS_VALID;
        /** @todo Check what the accessed bit should be (VT-x and AMD-V). */
    }
    else if (!(pVCpu->cpum.s.Guest.cr0 & X86_CR0_PE))
    {
        /* Real mode - leave the limit and flags alone here, at least for now. */
        pSReg->u64Base              = (uint32_t)pSReg->Sel << 4;
        pSReg->ValidSel             = pSReg->Sel;
        pSReg->fFlags               = CPUMSELREG_FLAGS_VALID;
    }
    else
    {
        /* Protected mode - get it from the selector descriptor tables. */
        if (!(pSReg->Sel & X86_SEL_MASK_OFF_RPL))
        {
            Assert(!CPUMIsGuestInLongMode(pVCpu));
            pSReg->Sel              = 0;
            pSReg->u64Base          = 0;
            pSReg->u32Limit         = 0;
            pSReg->Attr.u           = 0;
            pSReg->ValidSel         = 0;
            pSReg->fFlags           = CPUMSELREG_FLAGS_VALID;
            /** @todo see todo in iemHlpLoadNullDataSelectorProt. */
        }
        else
            SELMLoadHiddenSelectorReg(pVCpu, &pVCpu->cpum.s.Guest, pSReg);
    }
}


/**
 * Makes sure the hidden CS and SS selector registers are valid, loading them if
 * necessary.
 *
 * @param   pVCpu               The current virtual CPU.
 */
VMM_INT_DECL(void) CPUMGuestLazyLoadHiddenCsAndSs(PVMCPU pVCpu)
{
    CPUMSELREG_LAZY_LOAD_HIDDEN_PARTS(pVCpu, &pVCpu->cpum.s.Guest.cs);
    CPUMSELREG_LAZY_LOAD_HIDDEN_PARTS(pVCpu, &pVCpu->cpum.s.Guest.ss);
}


/**
 * Loads a the hidden parts of a selector register.
 *
 * @param   pVCpu               The current virtual CPU.
 */
VMM_INT_DECL(void) CPUMGuestLazyLoadHiddenSelectorReg(PVMCPU pVCpu, PCPUMSELREG pSReg)
{
    CPUMSELREG_LAZY_LOAD_HIDDEN_PARTS(pVCpu, pSReg);
}

#endif /* VBOX_WITH_RAW_MODE_NOT_R0 */


/**
 * Obsolete.
 *
 * We don't support nested hypervisor context interrupts or traps.  Life is much
 * simpler when we don't.  It's also slightly faster at times.
 *
 * @param   pVM         Handle to the virtual machine.
 */
VMMDECL(PCCPUMCTXCORE) CPUMGetHyperCtxCore(PVMCPU pVCpu)
{
    return CPUMCTX2CORE(&pVCpu->cpum.s.Hyper);
}


/**
 * Gets the pointer to the hypervisor CPU context structure of a virtual CPU.
 *
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(PCPUMCTX) CPUMGetHyperCtxPtr(PVMCPU pVCpu)
{
    return &pVCpu->cpum.s.Hyper;
}


VMMDECL(void) CPUMSetHyperGDTR(PVMCPU pVCpu, uint32_t addr, uint16_t limit)
{
    pVCpu->cpum.s.Hyper.gdtr.cbGdt = limit;
    pVCpu->cpum.s.Hyper.gdtr.pGdt  = addr;
}


VMMDECL(void) CPUMSetHyperIDTR(PVMCPU pVCpu, uint32_t addr, uint16_t limit)
{
    pVCpu->cpum.s.Hyper.idtr.cbIdt = limit;
    pVCpu->cpum.s.Hyper.idtr.pIdt  = addr;
}


VMMDECL(void) CPUMSetHyperCR3(PVMCPU pVCpu, uint32_t cr3)
{
    pVCpu->cpum.s.Hyper.cr3 = cr3;

#ifdef IN_RC
    /* Update the current CR3. */
    ASMSetCR3(cr3);
#endif
}

VMMDECL(uint32_t) CPUMGetHyperCR3(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.cr3;
}


VMMDECL(void) CPUMSetHyperCS(PVMCPU pVCpu, RTSEL SelCS)
{
    pVCpu->cpum.s.Hyper.cs.Sel = SelCS;
}


VMMDECL(void) CPUMSetHyperDS(PVMCPU pVCpu, RTSEL SelDS)
{
    pVCpu->cpum.s.Hyper.ds.Sel = SelDS;
}


VMMDECL(void) CPUMSetHyperES(PVMCPU pVCpu, RTSEL SelES)
{
    pVCpu->cpum.s.Hyper.es.Sel = SelES;
}


VMMDECL(void) CPUMSetHyperFS(PVMCPU pVCpu, RTSEL SelFS)
{
    pVCpu->cpum.s.Hyper.fs.Sel = SelFS;
}


VMMDECL(void) CPUMSetHyperGS(PVMCPU pVCpu, RTSEL SelGS)
{
    pVCpu->cpum.s.Hyper.gs.Sel = SelGS;
}


VMMDECL(void) CPUMSetHyperSS(PVMCPU pVCpu, RTSEL SelSS)
{
    pVCpu->cpum.s.Hyper.ss.Sel = SelSS;
}


VMMDECL(void) CPUMSetHyperESP(PVMCPU pVCpu, uint32_t u32ESP)
{
    pVCpu->cpum.s.Hyper.esp = u32ESP;
}


VMMDECL(void) CPUMSetHyperEDX(PVMCPU pVCpu, uint32_t u32ESP)
{
    pVCpu->cpum.s.Hyper.esp = u32ESP;
}


VMMDECL(int) CPUMSetHyperEFlags(PVMCPU pVCpu, uint32_t Efl)
{
    pVCpu->cpum.s.Hyper.eflags.u32 = Efl;
    return VINF_SUCCESS;
}


VMMDECL(void) CPUMSetHyperEIP(PVMCPU pVCpu, uint32_t u32EIP)
{
    pVCpu->cpum.s.Hyper.eip = u32EIP;
}


/**
 * Used by VMMR3RawRunGC to reinitialize the general raw-mode context registers,
 * EFLAGS and EIP prior to resuming guest execution.
 *
 * All general register not given as a parameter will be set to 0.  The EFLAGS
 * register will be set to sane values for C/C++ code execution with interrupts
 * disabled and IOPL 0.
 *
 * @param   pVCpu               The current virtual CPU.
 * @param   u32EIP              The EIP value.
 * @param   u32ESP              The ESP value.
 * @param   u32EAX              The EAX value.
 * @param   u32EDX              The EDX value.
 */
VMM_INT_DECL(void) CPUMSetHyperState(PVMCPU pVCpu, uint32_t u32EIP, uint32_t u32ESP, uint32_t u32EAX, uint32_t u32EDX)
{
    pVCpu->cpum.s.Hyper.eip      = u32EIP;
    pVCpu->cpum.s.Hyper.esp      = u32ESP;
    pVCpu->cpum.s.Hyper.eax      = u32EAX;
    pVCpu->cpum.s.Hyper.edx      = u32EDX;
    pVCpu->cpum.s.Hyper.ecx      = 0;
    pVCpu->cpum.s.Hyper.ebx      = 0;
    pVCpu->cpum.s.Hyper.ebp      = 0;
    pVCpu->cpum.s.Hyper.esi      = 0;
    pVCpu->cpum.s.Hyper.edi      = 0;
    pVCpu->cpum.s.Hyper.eflags.u = X86_EFL_1;
}


VMMDECL(void) CPUMSetHyperTR(PVMCPU pVCpu, RTSEL SelTR)
{
    pVCpu->cpum.s.Hyper.tr.Sel = SelTR;
}


VMMDECL(void) CPUMSetHyperLDTR(PVMCPU pVCpu, RTSEL SelLDTR)
{
    pVCpu->cpum.s.Hyper.ldtr.Sel = SelLDTR;
}


VMMDECL(void) CPUMSetHyperDR0(PVMCPU pVCpu, RTGCUINTREG uDr0)
{
    pVCpu->cpum.s.Hyper.dr[0] = uDr0;
    /** @todo in GC we must load it! */
}


VMMDECL(void) CPUMSetHyperDR1(PVMCPU pVCpu, RTGCUINTREG uDr1)
{
    pVCpu->cpum.s.Hyper.dr[1] = uDr1;
    /** @todo in GC we must load it! */
}


VMMDECL(void) CPUMSetHyperDR2(PVMCPU pVCpu, RTGCUINTREG uDr2)
{
    pVCpu->cpum.s.Hyper.dr[2] = uDr2;
    /** @todo in GC we must load it! */
}


VMMDECL(void) CPUMSetHyperDR3(PVMCPU pVCpu, RTGCUINTREG uDr3)
{
    pVCpu->cpum.s.Hyper.dr[3] = uDr3;
    /** @todo in GC we must load it! */
}


VMMDECL(void) CPUMSetHyperDR6(PVMCPU pVCpu, RTGCUINTREG uDr6)
{
    pVCpu->cpum.s.Hyper.dr[6] = uDr6;
    /** @todo in GC we must load it! */
}


VMMDECL(void) CPUMSetHyperDR7(PVMCPU pVCpu, RTGCUINTREG uDr7)
{
    pVCpu->cpum.s.Hyper.dr[7] = uDr7;
    /** @todo in GC we must load it! */
}


VMMDECL(RTSEL) CPUMGetHyperCS(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.cs.Sel;
}


VMMDECL(RTSEL) CPUMGetHyperDS(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.ds.Sel;
}


VMMDECL(RTSEL) CPUMGetHyperES(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.es.Sel;
}


VMMDECL(RTSEL) CPUMGetHyperFS(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.fs.Sel;
}


VMMDECL(RTSEL) CPUMGetHyperGS(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.gs.Sel;
}


VMMDECL(RTSEL) CPUMGetHyperSS(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.ss.Sel;
}


VMMDECL(uint32_t) CPUMGetHyperEAX(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.eax;
}


VMMDECL(uint32_t) CPUMGetHyperEBX(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.ebx;
}


VMMDECL(uint32_t) CPUMGetHyperECX(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.ecx;
}


VMMDECL(uint32_t) CPUMGetHyperEDX(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.edx;
}


VMMDECL(uint32_t) CPUMGetHyperESI(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.esi;
}


VMMDECL(uint32_t) CPUMGetHyperEDI(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.edi;
}


VMMDECL(uint32_t) CPUMGetHyperEBP(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.ebp;
}


VMMDECL(uint32_t) CPUMGetHyperESP(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.esp;
}


VMMDECL(uint32_t) CPUMGetHyperEFlags(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.eflags.u32;
}


VMMDECL(uint32_t) CPUMGetHyperEIP(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.eip;
}


VMMDECL(uint64_t) CPUMGetHyperRIP(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.rip;
}


VMMDECL(uint32_t) CPUMGetHyperIDTR(PVMCPU pVCpu, uint16_t *pcbLimit)
{
    if (pcbLimit)
        *pcbLimit = pVCpu->cpum.s.Hyper.idtr.cbIdt;
    return pVCpu->cpum.s.Hyper.idtr.pIdt;
}


VMMDECL(uint32_t) CPUMGetHyperGDTR(PVMCPU pVCpu, uint16_t *pcbLimit)
{
    if (pcbLimit)
        *pcbLimit = pVCpu->cpum.s.Hyper.gdtr.cbGdt;
    return pVCpu->cpum.s.Hyper.gdtr.pGdt;
}


VMMDECL(RTSEL) CPUMGetHyperLDTR(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.ldtr.Sel;
}


VMMDECL(RTGCUINTREG) CPUMGetHyperDR0(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.dr[0];
}


VMMDECL(RTGCUINTREG) CPUMGetHyperDR1(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.dr[1];
}


VMMDECL(RTGCUINTREG) CPUMGetHyperDR2(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.dr[2];
}


VMMDECL(RTGCUINTREG) CPUMGetHyperDR3(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.dr[3];
}


VMMDECL(RTGCUINTREG) CPUMGetHyperDR6(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.dr[6];
}


VMMDECL(RTGCUINTREG) CPUMGetHyperDR7(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Hyper.dr[7];
}


/**
 * Gets the pointer to the internal CPUMCTXCORE structure.
 * This is only for reading in order to save a few calls.
 *
 * @param   pVCpu       Handle to the virtual cpu.
 */
VMMDECL(PCCPUMCTXCORE) CPUMGetGuestCtxCore(PVMCPU pVCpu)
{
    return CPUMCTX2CORE(&pVCpu->cpum.s.Guest);
}


/**
 * Queries the pointer to the internal CPUMCTX structure.
 *
 * @returns The CPUMCTX pointer.
 * @param   pVCpu       Handle to the virtual cpu.
 */
VMMDECL(PCPUMCTX) CPUMQueryGuestCtxPtr(PVMCPU pVCpu)
{
    return &pVCpu->cpum.s.Guest;
}

VMMDECL(int) CPUMSetGuestGDTR(PVMCPU pVCpu, uint64_t GCPtrBase, uint16_t cbLimit)
{
#ifdef VBOX_WITH_IEM
# ifdef VBOX_WITH_RAW_MODE_NOT_R0
    if (!HWACCMIsEnabled(pVCpu->CTX_SUFF(pVM)))
        VMCPU_FF_SET(pVCpu, VMCPU_FF_SELM_SYNC_GDT);
# endif
#endif
    pVCpu->cpum.s.Guest.gdtr.cbGdt = cbLimit;
    pVCpu->cpum.s.Guest.gdtr.pGdt  = GCPtrBase;
    pVCpu->cpum.s.fChanged |= CPUM_CHANGED_GDTR;
    return VINF_SUCCESS; /* formality, consider it void. */
}

VMMDECL(int) CPUMSetGuestIDTR(PVMCPU pVCpu, uint64_t GCPtrBase, uint16_t cbLimit)
{
#ifdef VBOX_WITH_IEM
# ifdef VBOX_WITH_RAW_MODE_NOT_R0
    if (!HWACCMIsEnabled(pVCpu->CTX_SUFF(pVM)))
        VMCPU_FF_SET(pVCpu, VMCPU_FF_TRPM_SYNC_IDT);
# endif
#endif
    pVCpu->cpum.s.Guest.idtr.cbIdt = cbLimit;
    pVCpu->cpum.s.Guest.idtr.pIdt  = GCPtrBase;
    pVCpu->cpum.s.fChanged |= CPUM_CHANGED_IDTR;
    return VINF_SUCCESS; /* formality, consider it void. */
}

VMMDECL(int) CPUMSetGuestTR(PVMCPU pVCpu, uint16_t tr)
{
#ifdef VBOX_WITH_IEM
# ifdef VBOX_WITH_RAW_MODE_NOT_R0
    if (!HWACCMIsEnabled(pVCpu->CTX_SUFF(pVM)))
        VMCPU_FF_SET(pVCpu, VMCPU_FF_SELM_SYNC_TSS);
# endif
#endif
    pVCpu->cpum.s.Guest.tr.Sel  = tr;
    pVCpu->cpum.s.fChanged |= CPUM_CHANGED_TR;
    return VINF_SUCCESS; /* formality, consider it void. */
}

VMMDECL(int) CPUMSetGuestLDTR(PVMCPU pVCpu, uint16_t ldtr)
{
#ifdef VBOX_WITH_IEM
# ifdef VBOX_WITH_RAW_MODE_NOT_R0
    if (   (   ldtr != 0
            || pVCpu->cpum.s.Guest.ldtr.Sel != 0)
        && !HWACCMIsEnabled(pVCpu->CTX_SUFF(pVM)))
        VMCPU_FF_SET(pVCpu, VMCPU_FF_SELM_SYNC_LDT);
# endif
#endif
    pVCpu->cpum.s.Guest.ldtr.Sel      = ldtr;
    /* The caller will set more hidden bits if it has them. */
    pVCpu->cpum.s.Guest.ldtr.ValidSel = 0;
    pVCpu->cpum.s.Guest.ldtr.fFlags   = 0;
    pVCpu->cpum.s.fChanged  |= CPUM_CHANGED_LDTR;
    return VINF_SUCCESS; /* formality, consider it void. */
}


/**
 * Set the guest CR0.
 *
 * When called in GC, the hyper CR0 may be updated if that is
 * required. The caller only has to take special action if AM,
 * WP, PG or PE changes.
 *
 * @returns VINF_SUCCESS (consider it void).
 * @param   pVCpu   Handle to the virtual cpu.
 * @param   cr0     The new CR0 value.
 */
VMMDECL(int) CPUMSetGuestCR0(PVMCPU pVCpu, uint64_t cr0)
{
#ifdef IN_RC
    /*
     * Check if we need to change hypervisor CR0 because
     * of math stuff.
     */
    if (    (cr0                     & (X86_CR0_TS | X86_CR0_EM | X86_CR0_MP))
        !=  (pVCpu->cpum.s.Guest.cr0 & (X86_CR0_TS | X86_CR0_EM | X86_CR0_MP)))
    {
        if (!(pVCpu->cpum.s.fUseFlags & CPUM_USED_FPU))
        {
            /*
             * We haven't saved the host FPU state yet, so TS and MT are both set
             * and EM should be reflecting the guest EM (it always does this).
             */
            if ((cr0 & X86_CR0_EM) != (pVCpu->cpum.s.Guest.cr0 & X86_CR0_EM))
            {
                uint32_t HyperCR0 = ASMGetCR0();
                AssertMsg((HyperCR0 & (X86_CR0_TS | X86_CR0_MP)) == (X86_CR0_TS | X86_CR0_MP), ("%#x\n", HyperCR0));
                AssertMsg((HyperCR0 & X86_CR0_EM) == (pVCpu->cpum.s.Guest.cr0 & X86_CR0_EM), ("%#x\n", HyperCR0));
                HyperCR0 &= ~X86_CR0_EM;
                HyperCR0 |= cr0 & X86_CR0_EM;
                Log(("CPUM New HyperCR0=%#x\n", HyperCR0));
                ASMSetCR0(HyperCR0);
            }
# ifdef VBOX_STRICT
            else
            {
                uint32_t HyperCR0 = ASMGetCR0();
                AssertMsg((HyperCR0 & (X86_CR0_TS | X86_CR0_MP)) == (X86_CR0_TS | X86_CR0_MP), ("%#x\n", HyperCR0));
                AssertMsg((HyperCR0 & X86_CR0_EM) == (pVCpu->cpum.s.Guest.cr0 & X86_CR0_EM), ("%#x\n", HyperCR0));
            }
# endif
        }
        else
        {
            /*
             * Already saved the state, so we're just mirroring
             * the guest flags.
             */
            uint32_t HyperCR0 = ASMGetCR0();
            AssertMsg(     (HyperCR0                 & (X86_CR0_TS | X86_CR0_EM | X86_CR0_MP))
                      ==   (pVCpu->cpum.s.Guest.cr0  & (X86_CR0_TS | X86_CR0_EM | X86_CR0_MP)),
                      ("%#x %#x\n", HyperCR0, pVCpu->cpum.s.Guest.cr0));
            HyperCR0 &= ~(X86_CR0_TS | X86_CR0_EM | X86_CR0_MP);
            HyperCR0 |= cr0 & (X86_CR0_TS | X86_CR0_EM | X86_CR0_MP);
            Log(("CPUM New HyperCR0=%#x\n", HyperCR0));
            ASMSetCR0(HyperCR0);
        }
    }
#endif /* IN_RC */

    /*
     * Check for changes causing TLB flushes (for REM).
     * The caller is responsible for calling PGM when appropriate.
     */
    if (    (cr0                     & (X86_CR0_PG | X86_CR0_WP | X86_CR0_PE))
        !=  (pVCpu->cpum.s.Guest.cr0 & (X86_CR0_PG | X86_CR0_WP | X86_CR0_PE)))
        pVCpu->cpum.s.fChanged |= CPUM_CHANGED_GLOBAL_TLB_FLUSH;
    pVCpu->cpum.s.fChanged |= CPUM_CHANGED_CR0;

    pVCpu->cpum.s.Guest.cr0 = cr0 | X86_CR0_ET;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestCR2(PVMCPU pVCpu, uint64_t cr2)
{
    pVCpu->cpum.s.Guest.cr2 = cr2;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestCR3(PVMCPU pVCpu, uint64_t cr3)
{
    pVCpu->cpum.s.Guest.cr3 = cr3;
    pVCpu->cpum.s.fChanged |= CPUM_CHANGED_CR3;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestCR4(PVMCPU pVCpu, uint64_t cr4)
{
    if (    (cr4                     & (X86_CR4_PGE | X86_CR4_PAE | X86_CR4_PSE))
        !=  (pVCpu->cpum.s.Guest.cr4 & (X86_CR4_PGE | X86_CR4_PAE | X86_CR4_PSE)))
        pVCpu->cpum.s.fChanged |= CPUM_CHANGED_GLOBAL_TLB_FLUSH;
    pVCpu->cpum.s.fChanged |= CPUM_CHANGED_CR4;
    if (!CPUMSupportsFXSR(pVCpu->CTX_SUFF(pVM)))
        cr4 &= ~X86_CR4_OSFSXR;
    pVCpu->cpum.s.Guest.cr4 = cr4;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestEFlags(PVMCPU pVCpu, uint32_t eflags)
{
    pVCpu->cpum.s.Guest.eflags.u32 = eflags;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestEIP(PVMCPU pVCpu, uint32_t eip)
{
    pVCpu->cpum.s.Guest.eip = eip;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestEAX(PVMCPU pVCpu, uint32_t eax)
{
    pVCpu->cpum.s.Guest.eax = eax;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestEBX(PVMCPU pVCpu, uint32_t ebx)
{
    pVCpu->cpum.s.Guest.ebx = ebx;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestECX(PVMCPU pVCpu, uint32_t ecx)
{
    pVCpu->cpum.s.Guest.ecx = ecx;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestEDX(PVMCPU pVCpu, uint32_t edx)
{
    pVCpu->cpum.s.Guest.edx = edx;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestESP(PVMCPU pVCpu, uint32_t esp)
{
    pVCpu->cpum.s.Guest.esp = esp;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestEBP(PVMCPU pVCpu, uint32_t ebp)
{
    pVCpu->cpum.s.Guest.ebp = ebp;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestESI(PVMCPU pVCpu, uint32_t esi)
{
    pVCpu->cpum.s.Guest.esi = esi;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestEDI(PVMCPU pVCpu, uint32_t edi)
{
    pVCpu->cpum.s.Guest.edi = edi;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestSS(PVMCPU pVCpu, uint16_t ss)
{
    pVCpu->cpum.s.Guest.ss.Sel = ss;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestCS(PVMCPU pVCpu, uint16_t cs)
{
    pVCpu->cpum.s.Guest.cs.Sel = cs;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestDS(PVMCPU pVCpu, uint16_t ds)
{
    pVCpu->cpum.s.Guest.ds.Sel = ds;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestES(PVMCPU pVCpu, uint16_t es)
{
    pVCpu->cpum.s.Guest.es.Sel = es;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestFS(PVMCPU pVCpu, uint16_t fs)
{
    pVCpu->cpum.s.Guest.fs.Sel = fs;
    return VINF_SUCCESS;
}


VMMDECL(int) CPUMSetGuestGS(PVMCPU pVCpu, uint16_t gs)
{
    pVCpu->cpum.s.Guest.gs.Sel = gs;
    return VINF_SUCCESS;
}


VMMDECL(void) CPUMSetGuestEFER(PVMCPU pVCpu, uint64_t val)
{
    pVCpu->cpum.s.Guest.msrEFER = val;
}


/**
 * Query an MSR.
 *
 * The caller is responsible for checking privilege if the call is the result
 * of a RDMSR instruction. We'll do the rest.
 *
 * @retval  VINF_SUCCESS on success.
 * @retval  VERR_CPUM_RAISE_GP_0 on failure (invalid MSR), the caller is
 *          expected to take the appropriate actions. @a *puValue is set to 0.
 * @param   pVCpu               Pointer to the VMCPU.
 * @param   idMsr               The MSR.
 * @param   puValue             Where to return the value.
 *
 * @remarks This will always return the right values, even when we're in the
 *          recompiler.
 */
VMMDECL(int) CPUMQueryGuestMsr(PVMCPU pVCpu, uint32_t idMsr, uint64_t *puValue)
{
    /*
     * If we don't indicate MSR support in the CPUID feature bits, indicate
     * that a #GP(0) should be raised.
     */
    if (!(pVCpu->CTX_SUFF(pVM)->cpum.s.aGuestCpuIdStd[1].edx & X86_CPUID_FEATURE_EDX_MSR))
    {
        *puValue = 0;
        return VERR_CPUM_RAISE_GP_0; /** @todo isn't \#UD more correct if not supported? */
    }

    int rc = VINF_SUCCESS;
    uint8_t const u8Multiplier = 4;
    switch (idMsr)
    {
        case MSR_IA32_TSC:
            *puValue = TMCpuTickGet(pVCpu);
            break;

        case MSR_IA32_APICBASE:
            rc = PDMApicGetBase(pVCpu->CTX_SUFF(pVM), puValue);
            if (RT_SUCCESS(rc))
                rc = VINF_SUCCESS;
            else
            {
                *puValue = 0;
                rc = VERR_CPUM_RAISE_GP_0;
            }
            break;

        case MSR_IA32_CR_PAT:
            *puValue = pVCpu->cpum.s.Guest.msrPAT;
            break;

        case MSR_IA32_SYSENTER_CS:
            *puValue = pVCpu->cpum.s.Guest.SysEnter.cs;
            break;

        case MSR_IA32_SYSENTER_EIP:
            *puValue = pVCpu->cpum.s.Guest.SysEnter.eip;
            break;

        case MSR_IA32_SYSENTER_ESP:
            *puValue = pVCpu->cpum.s.Guest.SysEnter.esp;
            break;

        case MSR_IA32_MTRR_CAP:
        {
            /* This is currently a bit weird. :-) */
            uint8_t const   cVariableRangeRegs              = 0;
            bool const      fSystemManagementRangeRegisters = false;
            bool const      fFixedRangeRegisters            = false;
            bool const      fWriteCombiningType             = false;
            *puValue = cVariableRangeRegs
                     | (fFixedRangeRegisters            ? RT_BIT_64(8)  : 0)
                     | (fWriteCombiningType             ? RT_BIT_64(10) : 0)
                     | (fSystemManagementRangeRegisters ? RT_BIT_64(11) : 0);
            break;
        }

        case MSR_IA32_MTRR_DEF_TYPE:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrDefType;
            break;

        case IA32_MTRR_FIX64K_00000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix64K_00000;
            break;
        case IA32_MTRR_FIX16K_80000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix16K_80000;
            break;
        case IA32_MTRR_FIX16K_A0000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix16K_A0000;
            break;
        case IA32_MTRR_FIX4K_C0000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_C0000;
            break;
        case IA32_MTRR_FIX4K_C8000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_C8000;
            break;
        case IA32_MTRR_FIX4K_D0000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_D0000;
            break;
        case IA32_MTRR_FIX4K_D8000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_D8000;
            break;
        case IA32_MTRR_FIX4K_E0000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_E0000;
            break;
        case IA32_MTRR_FIX4K_E8000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_E8000;
            break;
        case IA32_MTRR_FIX4K_F0000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_F0000;
            break;
        case IA32_MTRR_FIX4K_F8000:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_F8000;
            break;

        case MSR_K6_EFER:
            *puValue = pVCpu->cpum.s.Guest.msrEFER;
            break;

        case MSR_K8_SF_MASK:
            *puValue = pVCpu->cpum.s.Guest.msrSFMASK;
            break;

        case MSR_K6_STAR:
            *puValue = pVCpu->cpum.s.Guest.msrSTAR;
            break;

        case MSR_K8_LSTAR:
            *puValue = pVCpu->cpum.s.Guest.msrLSTAR;
            break;

        case MSR_K8_CSTAR:
            *puValue = pVCpu->cpum.s.Guest.msrCSTAR;
            break;

        case MSR_K8_FS_BASE:
            *puValue = pVCpu->cpum.s.Guest.fs.u64Base;
            break;

        case MSR_K8_GS_BASE:
            *puValue = pVCpu->cpum.s.Guest.gs.u64Base;
            break;

        case MSR_K8_KERNEL_GS_BASE:
            *puValue = pVCpu->cpum.s.Guest.msrKERNELGSBASE;
            break;

        case MSR_K8_TSC_AUX:
            *puValue = pVCpu->cpum.s.GuestMsrs.msr.TscAux;
            break;

        case MSR_IA32_PERF_STATUS:
            /** @todo could really be not exactly correct, maybe use host's values */
            *puValue = UINT64_C(1000)                 /* TSC increment by tick */
                     | ((uint64_t)u8Multiplier << 24) /* CPU multiplier (aka bus ratio) min */
                     | ((uint64_t)u8Multiplier << 40) /* CPU multiplier (aka bus ratio) max */;
            break;

        case MSR_IA32_FSB_CLOCK_STS:
            /*
             * Encoded as:
             * 0 - 266
             * 1 - 133
             * 2 - 200
             * 3 - return 166
             * 5 - return 100
             */
            *puValue = (2 << 4);
            break;

        case MSR_IA32_PLATFORM_INFO:
            *puValue = (u8Multiplier << 8)            /* Flex ratio max */
                     | ((uint64_t)u8Multiplier << 40) /* Flex ratio min */;
            break;

        case MSR_IA32_THERM_STATUS:
            /* CPU temperature relative to TCC, to actually activate, CPUID leaf 6 EAX[0] must be set */
            *puValue = RT_BIT(31)           /* validity bit */
                     | (UINT64_C(20) << 16) /* degrees till TCC */;
            break;

        case MSR_IA32_MISC_ENABLE:
#if 0
            /* Needs to be tested more before enabling. */
            *puValue = pVCpu->cpum.s.GuestMsr.msr.miscEnable;
#else
            /* Currenty we don't allow guests to modify enable MSRs. */
            *puValue = MSR_IA32_MISC_ENABLE_FAST_STRINGS  /* by default */;

            if ((pVCpu->CTX_SUFF(pVM)->cpum.s.aGuestCpuIdStd[1].ecx & X86_CPUID_FEATURE_ECX_MONITOR) != 0)

                *puValue |= MSR_IA32_MISC_ENABLE_MONITOR /* if mwait/monitor available */;
            /** @todo: add more cpuid-controlled features this way. */
#endif
            break;

#if 0 /*def IN_RING0 */
        case MSR_IA32_PLATFORM_ID:
        case MSR_IA32_BIOS_SIGN_ID:
            if (CPUMGetCPUVendor(pVM) == CPUMCPUVENDOR_INTEL)
            {
                /* Available since the P6 family. VT-x implies that this feature is present. */
                if (idMsr == MSR_IA32_PLATFORM_ID)
                    *puValue = ASMRdMsr(MSR_IA32_PLATFORM_ID);
                else if (idMsr == MSR_IA32_BIOS_SIGN_ID)
                    *puValue = ASMRdMsr(MSR_IA32_BIOS_SIGN_ID);
                break;
            }
            /* no break */
#endif

        /*
         * Intel specifics MSRs:
         */
        case MSR_IA32_PLATFORM_ID:          /* fam/mod >= 6_01 */
        case MSR_IA32_BIOS_SIGN_ID:         /* fam/mod >= 6_01 */
        /*case MSR_IA32_BIOS_UPDT_TRIG: - write-only? */
        case MSR_IA32_MCP_CAP:              /* fam/mod >= 6_01 */
        /*case MSR_IA32_MCP_STATUS:     - indicated as not present in CAP */
        /*case MSR_IA32_MCP_CTRL:       - indicated as not present in CAP */
        case MSR_IA32_MC0_CTL:
        case MSR_IA32_MC0_STATUS:
            *puValue = 0;
            if (CPUMGetGuestCpuVendor(pVCpu->CTX_SUFF(pVM)) != CPUMCPUVENDOR_INTEL)
            {
                Log(("MSR %#x is Intel, the virtual CPU isn't an Intel one -> #GP\n", idMsr));
                rc = VERR_CPUM_RAISE_GP_0;
            }
            break;

        default:
            /*
             * Hand the X2APIC range to PDM and the APIC.
             */
            if (    idMsr >= MSR_IA32_APIC_START
                &&  idMsr <  MSR_IA32_APIC_END)
            {
                rc = PDMApicReadMSR(pVCpu->CTX_SUFF(pVM), pVCpu->idCpu, idMsr, puValue);
                if (RT_SUCCESS(rc))
                    rc = VINF_SUCCESS;
                else
                {
                    *puValue = 0;
                    rc = VERR_CPUM_RAISE_GP_0;
                }
            }
            else
            {
                *puValue = 0;
                rc = VERR_CPUM_RAISE_GP_0;
            }
            break;
    }

    return rc;
}


/**
 * Sets the MSR.
 *
 * The caller is responsible for checking privilege if the call is the result
 * of a WRMSR instruction. We'll do the rest.
 *
 * @retval  VINF_SUCCESS on success.
 * @retval  VERR_CPUM_RAISE_GP_0 on failure, the caller is expected to take the
 *          appropriate actions.
 *
 * @param   pVCpu       Pointer to the VMCPU.
 * @param   idMsr       The MSR id.
 * @param   uValue      The value to set.
 *
 * @remarks Everyone changing MSR values, including the recompiler, shall do it
 *          by calling this method. This makes sure we have current values and
 *          that we trigger all the right actions when something changes.
 */
VMMDECL(int) CPUMSetGuestMsr(PVMCPU pVCpu, uint32_t idMsr, uint64_t uValue)
{
    /*
     * If we don't indicate MSR support in the CPUID feature bits, indicate
     * that a #GP(0) should be raised.
     */
    if (!(pVCpu->CTX_SUFF(pVM)->cpum.s.aGuestCpuIdStd[1].edx & X86_CPUID_FEATURE_EDX_MSR))
        return VERR_CPUM_RAISE_GP_0; /** @todo isn't \#UD more correct if not supported? */

    int rc = VINF_SUCCESS;
    switch (idMsr)
    {
        case MSR_IA32_MISC_ENABLE:
            pVCpu->cpum.s.GuestMsrs.msr.MiscEnable = uValue;
            break;

        case MSR_IA32_TSC:
            TMCpuTickSet(pVCpu->CTX_SUFF(pVM), pVCpu, uValue);
            break;

        case MSR_IA32_APICBASE:
            rc = PDMApicSetBase(pVCpu->CTX_SUFF(pVM), uValue);
            if (rc != VINF_SUCCESS)
                rc = VERR_CPUM_RAISE_GP_0;
            break;

        case MSR_IA32_CR_PAT:
            pVCpu->cpum.s.Guest.msrPAT      = uValue;
            break;

        case MSR_IA32_SYSENTER_CS:
            pVCpu->cpum.s.Guest.SysEnter.cs = uValue & 0xffff; /* 16 bits selector */
            break;

        case MSR_IA32_SYSENTER_EIP:
            pVCpu->cpum.s.Guest.SysEnter.eip = uValue;
            break;

        case MSR_IA32_SYSENTER_ESP:
            pVCpu->cpum.s.Guest.SysEnter.esp = uValue;
            break;

        case MSR_IA32_MTRR_CAP:
            return VERR_CPUM_RAISE_GP_0;

        case MSR_IA32_MTRR_DEF_TYPE:
            if (   (uValue & UINT64_C(0xfffffffffffff300))
                || (    (uValue & 0xff) != 0
                    &&  (uValue & 0xff) != 1
                    &&  (uValue & 0xff) != 4
                    &&  (uValue & 0xff) != 5
                    &&  (uValue & 0xff) != 6) )
            {
                Log(("MSR_IA32_MTRR_DEF_TYPE: #GP(0) - writing reserved value (%#llx)\n", uValue));
                return VERR_CPUM_RAISE_GP_0;
            }
            pVCpu->cpum.s.GuestMsrs.msr.MtrrDefType = uValue;
            break;

        case IA32_MTRR_FIX64K_00000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix64K_00000 = uValue;
            break;
        case IA32_MTRR_FIX16K_80000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix16K_80000 = uValue;
            break;
        case IA32_MTRR_FIX16K_A0000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix16K_A0000 = uValue;
            break;
        case IA32_MTRR_FIX4K_C0000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_C0000 = uValue;
            break;
        case IA32_MTRR_FIX4K_C8000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_C8000 = uValue;
            break;
        case IA32_MTRR_FIX4K_D0000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_D0000 = uValue;
            break;
        case IA32_MTRR_FIX4K_D8000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_D8000 = uValue;
            break;
        case IA32_MTRR_FIX4K_E0000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_E0000 = uValue;
            break;
        case IA32_MTRR_FIX4K_E8000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_E8000 = uValue;
            break;
        case IA32_MTRR_FIX4K_F0000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_F0000 = uValue;
            break;
        case IA32_MTRR_FIX4K_F8000:
            pVCpu->cpum.s.GuestMsrs.msr.MtrrFix4K_F8000 = uValue;
            break;

        /*
         * AMD64 MSRs.
         */
        case MSR_K6_EFER:
        {
            PVM             pVM          = pVCpu->CTX_SUFF(pVM);
            uint64_t const  uOldEFER     = pVCpu->cpum.s.Guest.msrEFER;
            uint32_t const  fExtFeatures = pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001
                                         ? pVM->cpum.s.aGuestCpuIdExt[1].edx
                                         : 0;
            uint64_t        fMask        = 0;

            /* Filter out those bits the guest is allowed to change. (e.g. LMA is read-only) */
            if (fExtFeatures & X86_CPUID_EXT_FEATURE_EDX_NX)
                fMask |= MSR_K6_EFER_NXE;
            if (fExtFeatures & X86_CPUID_EXT_FEATURE_EDX_LONG_MODE)
                fMask |= MSR_K6_EFER_LME;
            if (fExtFeatures & X86_CPUID_EXT_FEATURE_EDX_SYSCALL)
                fMask |= MSR_K6_EFER_SCE;
            if (fExtFeatures & X86_CPUID_AMD_FEATURE_EDX_FFXSR)
                fMask |= MSR_K6_EFER_FFXSR;

            /* Check for illegal MSR_K6_EFER_LME transitions: not allowed to change LME if
               paging is enabled. (AMD Arch. Programmer's Manual Volume 2: Table 14-5) */
            if (    (uOldEFER & MSR_K6_EFER_LME) != (uValue & fMask & MSR_K6_EFER_LME)
                &&  (pVCpu->cpum.s.Guest.cr0 & X86_CR0_PG))
            {
                Log(("Illegal MSR_K6_EFER_LME change: paging is enabled!!\n"));
                return VERR_CPUM_RAISE_GP_0;
            }

            /* There are a few more: e.g. MSR_K6_EFER_LMSLE */
            AssertMsg(!(uValue & ~(MSR_K6_EFER_NXE | MSR_K6_EFER_LME | MSR_K6_EFER_LMA /* ignored anyway */ | MSR_K6_EFER_SCE | MSR_K6_EFER_FFXSR)),
                      ("Unexpected value %RX64\n", uValue));
            pVCpu->cpum.s.Guest.msrEFER = (uOldEFER & ~fMask) | (uValue & fMask);

            /* AMD64 Architecture Programmer's Manual: 15.15 TLB Control; flush the TLB
               if MSR_K6_EFER_NXE, MSR_K6_EFER_LME or MSR_K6_EFER_LMA are changed. */
            if (   (uOldEFER                    & (MSR_K6_EFER_NXE | MSR_K6_EFER_LME | MSR_K6_EFER_LMA))
                != (pVCpu->cpum.s.Guest.msrEFER & (MSR_K6_EFER_NXE | MSR_K6_EFER_LME | MSR_K6_EFER_LMA)))
            {
                /// @todo PGMFlushTLB(pVCpu, cr3, true /*fGlobal*/);
                HWACCMFlushTLB(pVCpu);

                /* Notify PGM about NXE changes. */
                if (   (uOldEFER                    & MSR_K6_EFER_NXE)
                    != (pVCpu->cpum.s.Guest.msrEFER & MSR_K6_EFER_NXE))
                    PGMNotifyNxeChanged(pVCpu, !(uOldEFER & MSR_K6_EFER_NXE));
            }
            break;
        }

        case MSR_K8_SF_MASK:
            pVCpu->cpum.s.Guest.msrSFMASK       = uValue;
            break;

        case MSR_K6_STAR:
            pVCpu->cpum.s.Guest.msrSTAR         = uValue;
            break;

        case MSR_K8_LSTAR:
            pVCpu->cpum.s.Guest.msrLSTAR        = uValue;
            break;

        case MSR_K8_CSTAR:
            pVCpu->cpum.s.Guest.msrCSTAR        = uValue;
            break;

        case MSR_K8_FS_BASE:
            pVCpu->cpum.s.Guest.fs.u64Base      = uValue;
            break;

        case MSR_K8_GS_BASE:
            pVCpu->cpum.s.Guest.gs.u64Base      = uValue;
            break;

        case MSR_K8_KERNEL_GS_BASE:
            pVCpu->cpum.s.Guest.msrKERNELGSBASE = uValue;
            break;

        case MSR_K8_TSC_AUX:
            pVCpu->cpum.s.GuestMsrs.msr.TscAux  = uValue;
            break;

        /*
         * Intel specifics MSRs:
         */
        /*case MSR_IA32_PLATFORM_ID: - read-only */
        case MSR_IA32_BIOS_SIGN_ID:         /* fam/mod >= 6_01 */
        case MSR_IA32_BIOS_UPDT_TRIG:       /* fam/mod >= 6_01 */
        /*case MSR_IA32_MCP_CAP:     - read-only */
        /*case MSR_IA32_MCP_STATUS:  - read-only */
        /*case MSR_IA32_MCP_CTRL:    - indicated as not present in CAP */
        /*case MSR_IA32_MC0_CTL:     - read-only? */
        /*case MSR_IA32_MC0_STATUS:  - read-only? */
            if (CPUMGetGuestCpuVendor(pVCpu->CTX_SUFF(pVM)) != CPUMCPUVENDOR_INTEL)
            {
                Log(("MSR %#x is Intel, the virtual CPU isn't an Intel one -> #GP\n", idMsr));
                return VERR_CPUM_RAISE_GP_0;
            }
            /* ignored */
            break;

        default:
            /*
             * Hand the X2APIC range to PDM and the APIC.
             */
            if (    idMsr >= MSR_IA32_APIC_START
                &&  idMsr <  MSR_IA32_APIC_END)
            {
                rc = PDMApicWriteMSR(pVCpu->CTX_SUFF(pVM), pVCpu->idCpu, idMsr, uValue);
                if (rc != VINF_SUCCESS)
                    rc = VERR_CPUM_RAISE_GP_0;
            }
            else
            {
                /* We should actually trigger a #GP here, but don't as that might cause more trouble. */
                /** @todo rc = VERR_CPUM_RAISE_GP_0 */
                Log(("CPUMSetGuestMsr: Unknown MSR %#x attempted set to %#llx\n", idMsr, uValue));
            }
            break;
    }
    return rc;
}


VMMDECL(RTGCPTR) CPUMGetGuestIDTR(PVMCPU pVCpu, uint16_t *pcbLimit)
{
    if (pcbLimit)
        *pcbLimit = pVCpu->cpum.s.Guest.idtr.cbIdt;
    return pVCpu->cpum.s.Guest.idtr.pIdt;
}


VMMDECL(RTSEL) CPUMGetGuestTR(PVMCPU pVCpu, PCPUMSELREGHID pHidden)
{
    if (pHidden)
        *pHidden = pVCpu->cpum.s.Guest.tr;
    return pVCpu->cpum.s.Guest.tr.Sel;
}


VMMDECL(RTSEL) CPUMGetGuestCS(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.cs.Sel;
}


VMMDECL(RTSEL) CPUMGetGuestDS(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.ds.Sel;
}


VMMDECL(RTSEL) CPUMGetGuestES(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.es.Sel;
}


VMMDECL(RTSEL) CPUMGetGuestFS(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.fs.Sel;
}


VMMDECL(RTSEL) CPUMGetGuestGS(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.gs.Sel;
}


VMMDECL(RTSEL) CPUMGetGuestSS(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.ss.Sel;
}


VMMDECL(RTSEL) CPUMGetGuestLDTR(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.ldtr.Sel;
}


VMMDECL(RTSEL) CPUMGetGuestLdtrEx(PVMCPU pVCpu, uint64_t *pGCPtrBase, uint32_t *pcbLimit)
{
    *pGCPtrBase = pVCpu->cpum.s.Guest.ldtr.u64Base;
    *pcbLimit   = pVCpu->cpum.s.Guest.ldtr.u32Limit;
    return pVCpu->cpum.s.Guest.ldtr.Sel;
}


VMMDECL(uint64_t) CPUMGetGuestCR0(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.cr0;
}


VMMDECL(uint64_t) CPUMGetGuestCR2(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.cr2;
}


VMMDECL(uint64_t) CPUMGetGuestCR3(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.cr3;
}


VMMDECL(uint64_t) CPUMGetGuestCR4(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.cr4;
}


VMMDECL(uint64_t) CPUMGetGuestCR8(PVMCPU pVCpu)
{
    uint64_t u64;
    int rc = CPUMGetGuestCRx(pVCpu, DISCREG_CR8, &u64);
    if (RT_FAILURE(rc))
        u64 = 0;
    return u64;
}


VMMDECL(void) CPUMGetGuestGDTR(PVMCPU pVCpu, PVBOXGDTR pGDTR)
{
    *pGDTR = pVCpu->cpum.s.Guest.gdtr;
}


VMMDECL(uint32_t) CPUMGetGuestEIP(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.eip;
}


VMMDECL(uint64_t) CPUMGetGuestRIP(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.rip;
}


VMMDECL(uint32_t) CPUMGetGuestEAX(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.eax;
}


VMMDECL(uint32_t) CPUMGetGuestEBX(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.ebx;
}


VMMDECL(uint32_t) CPUMGetGuestECX(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.ecx;
}


VMMDECL(uint32_t) CPUMGetGuestEDX(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.edx;
}


VMMDECL(uint32_t) CPUMGetGuestESI(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.esi;
}


VMMDECL(uint32_t) CPUMGetGuestEDI(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.edi;
}


VMMDECL(uint32_t) CPUMGetGuestESP(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.esp;
}


VMMDECL(uint32_t) CPUMGetGuestEBP(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.ebp;
}


VMMDECL(uint32_t) CPUMGetGuestEFlags(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.eflags.u32;
}


VMMDECL(int) CPUMGetGuestCRx(PVMCPU pVCpu, unsigned iReg, uint64_t *pValue)
{
    switch (iReg)
    {
        case DISCREG_CR0:
            *pValue = pVCpu->cpum.s.Guest.cr0;
            break;

        case DISCREG_CR2:
            *pValue = pVCpu->cpum.s.Guest.cr2;
            break;

        case DISCREG_CR3:
            *pValue = pVCpu->cpum.s.Guest.cr3;
            break;

        case DISCREG_CR4:
            *pValue = pVCpu->cpum.s.Guest.cr4;
            break;

        case DISCREG_CR8:
        {
            uint8_t u8Tpr;
            int rc = PDMApicGetTPR(pVCpu, &u8Tpr, NULL /*pfPending*/);
            if (RT_FAILURE(rc))
            {
                AssertMsg(rc == VERR_PDM_NO_APIC_INSTANCE, ("%Rrc\n", rc));
                *pValue = 0;
                return rc;
            }
            *pValue = u8Tpr >> 4; /* bits 7-4 contain the task priority that go in cr8, bits 3-0*/
            break;
        }

        default:
            return VERR_INVALID_PARAMETER;
    }
    return VINF_SUCCESS;
}


VMMDECL(uint64_t) CPUMGetGuestDR0(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.dr[0];
}


VMMDECL(uint64_t) CPUMGetGuestDR1(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.dr[1];
}


VMMDECL(uint64_t) CPUMGetGuestDR2(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.dr[2];
}


VMMDECL(uint64_t) CPUMGetGuestDR3(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.dr[3];
}


VMMDECL(uint64_t) CPUMGetGuestDR6(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.dr[6];
}


VMMDECL(uint64_t) CPUMGetGuestDR7(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.dr[7];
}


VMMDECL(int) CPUMGetGuestDRx(PVMCPU pVCpu, uint32_t iReg, uint64_t *pValue)
{
    AssertReturn(iReg <= DISDREG_DR7, VERR_INVALID_PARAMETER);
    /* DR4 is an alias for DR6, and DR5 is an alias for DR7. */
    if (iReg == 4 || iReg == 5)
        iReg += 2;
    *pValue = pVCpu->cpum.s.Guest.dr[iReg];
    return VINF_SUCCESS;
}


VMMDECL(uint64_t) CPUMGetGuestEFER(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.Guest.msrEFER;
}


/**
 * Gets a CPUID leaf.
 *
 * @param   pVCpu   Pointer to the VMCPU.
 * @param   iLeaf   The CPUID leaf to get.
 * @param   pEax    Where to store the EAX value.
 * @param   pEbx    Where to store the EBX value.
 * @param   pEcx    Where to store the ECX value.
 * @param   pEdx    Where to store the EDX value.
 */
VMMDECL(void) CPUMGetGuestCpuId(PVMCPU pVCpu, uint32_t iLeaf, uint32_t *pEax, uint32_t *pEbx, uint32_t *pEcx, uint32_t *pEdx)
{
    PVM pVM = pVCpu->CTX_SUFF(pVM);

    PCCPUMCPUID pCpuId;
    if (iLeaf < RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdStd))
        pCpuId = &pVM->cpum.s.aGuestCpuIdStd[iLeaf];
    else if (iLeaf - UINT32_C(0x80000000) < RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdExt))
        pCpuId = &pVM->cpum.s.aGuestCpuIdExt[iLeaf - UINT32_C(0x80000000)];
    else if (   iLeaf - UINT32_C(0x40000000) < RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdHyper)
             && (pVCpu->CTX_SUFF(pVM)->cpum.s.aGuestCpuIdStd[1].ecx & X86_CPUID_FEATURE_ECX_HVP))
        pCpuId = &pVM->cpum.s.aGuestCpuIdHyper[iLeaf - UINT32_C(0x40000000)];   /* Only report if HVP bit set. */
    else if (iLeaf - UINT32_C(0xc0000000) < RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdCentaur))
        pCpuId = &pVM->cpum.s.aGuestCpuIdCentaur[iLeaf - UINT32_C(0xc0000000)];
    else
        pCpuId = &pVM->cpum.s.GuestCpuIdDef;

    uint32_t cCurrentCacheIndex = *pEcx;

    *pEax = pCpuId->eax;
    *pEbx = pCpuId->ebx;
    *pEcx = pCpuId->ecx;
    *pEdx = pCpuId->edx;

    if (    iLeaf == 1)
    {
        /* Bits 31-24: Initial APIC ID */
        Assert(pVCpu->idCpu <= 255);
        *pEbx |= (pVCpu->idCpu << 24);
   }

    if (    iLeaf == 4
        &&  cCurrentCacheIndex < 3
        &&  pVM->cpum.s.enmGuestCpuVendor == CPUMCPUVENDOR_INTEL)
    {
        uint32_t type, level, sharing, linesize,
                 partitions, associativity, sets, cores;

        /* For type: 1 - data cache, 2 - i-cache, 3 - unified */
        partitions = 1;
        /* Those are only to shut up compiler, as they will always
           get overwritten, and compiler should be able to figure that out */
        sets = associativity = sharing = level = 1;
        cores = pVM->cCpus > 32 ? 32 : pVM->cCpus;
        switch (cCurrentCacheIndex)
        {
            case 0:
                type = 1;
                level = 1;
                sharing = 1;
                linesize = 64;
                associativity = 8;
                sets = 64;
                break;
            case 1:
                level = 1;
                type = 2;
                sharing = 1;
                linesize = 64;
                associativity = 8;
                sets = 64;
                break;
            default:            /* shut up gcc.*/
                AssertFailed();
            case 2:
                level = 2;
                type = 3;
                sharing = cores; /* our L2 cache is modelled as shared between all cores */
                linesize = 64;
                associativity = 24;
                sets = 4096;
                break;
        }

        *pEax |= ((cores - 1) << 26)        |
                 ((sharing - 1) << 14)      |
                 (level << 5)               |
                 1;
        *pEbx = (linesize - 1)               |
                ((partitions - 1) << 12)     |
                ((associativity - 1) << 22); /* -1 encoding */
        *pEcx = sets - 1;
    }

    Log2(("CPUMGetGuestCpuId: iLeaf=%#010x %RX32 %RX32 %RX32 %RX32\n", iLeaf, *pEax, *pEbx, *pEcx, *pEdx));
}

/**
 * Gets a number of standard CPUID leafs.
 *
 * @returns Number of leafs.
 * @param   pVM         Pointer to the VM.
 * @remark  Intended for PATM.
 */
VMMDECL(uint32_t) CPUMGetGuestCpuIdStdMax(PVM pVM)
{
    return RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdStd);
}


/**
 * Gets a number of extended CPUID leafs.
 *
 * @returns Number of leafs.
 * @param   pVM         Pointer to the VM.
 * @remark  Intended for PATM.
 */
VMMDECL(uint32_t) CPUMGetGuestCpuIdExtMax(PVM pVM)
{
    return RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdExt);
}


/**
 * Gets a number of centaur CPUID leafs.
 *
 * @returns Number of leafs.
 * @param   pVM         Pointer to the VM.
 * @remark  Intended for PATM.
 */
VMMDECL(uint32_t) CPUMGetGuestCpuIdCentaurMax(PVM pVM)
{
    return RT_ELEMENTS(pVM->cpum.s.aGuestCpuIdCentaur);
}


/**
 * Sets a CPUID feature bit.
 *
 * @param   pVM             Pointer to the VM.
 * @param   enmFeature      The feature to set.
 */
VMMDECL(void) CPUMSetGuestCpuIdFeature(PVM pVM, CPUMCPUIDFEATURE enmFeature)
{
    switch (enmFeature)
    {
        /*
         * Set the APIC bit in both feature masks.
         */
        case CPUMCPUIDFEATURE_APIC:
            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].edx |= X86_CPUID_FEATURE_EDX_APIC;
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001
                &&  pVM->cpum.s.enmGuestCpuVendor == CPUMCPUVENDOR_AMD)
                pVM->cpum.s.aGuestCpuIdExt[1].edx |= X86_CPUID_AMD_FEATURE_EDX_APIC;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled APIC\n"));
            break;

       /*
        * Set the x2APIC bit in the standard feature mask.
        */
        case CPUMCPUIDFEATURE_X2APIC:
            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].ecx |= X86_CPUID_FEATURE_ECX_X2APIC;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled x2APIC\n"));
            break;

        /*
         * Set the sysenter/sysexit bit in the standard feature mask.
         * Assumes the caller knows what it's doing! (host must support these)
         */
        case CPUMCPUIDFEATURE_SEP:
        {
            if (!(ASMCpuId_EDX(1) & X86_CPUID_FEATURE_EDX_SEP))
            {
                AssertMsgFailed(("ERROR: Can't turn on SEP when the host doesn't support it!!\n"));
                return;
            }

            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].edx |= X86_CPUID_FEATURE_EDX_SEP;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled sysenter/exit\n"));
            break;
        }

        /*
         * Set the syscall/sysret bit in the extended feature mask.
         * Assumes the caller knows what it's doing! (host must support these)
         */
        case CPUMCPUIDFEATURE_SYSCALL:
        {
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax < 0x80000001
                ||  !(ASMCpuId_EDX(0x80000001) & X86_CPUID_EXT_FEATURE_EDX_SYSCALL))
            {
#if HC_ARCH_BITS == 32
                /* X86_CPUID_EXT_FEATURE_EDX_SYSCALL not set it seems in 32 bits mode.
                 * Even when the cpu is capable of doing so in 64 bits mode.
                 */
                if (    pVM->cpum.s.aGuestCpuIdExt[0].eax < 0x80000001
                    ||  !(ASMCpuId_EDX(0x80000001) & X86_CPUID_EXT_FEATURE_EDX_LONG_MODE)
                    ||  !(ASMCpuId_EDX(1) & X86_CPUID_EXT_FEATURE_EDX_SYSCALL))
#endif
                {
                    LogRel(("WARNING: Can't turn on SYSCALL/SYSRET when the host doesn't support it!!\n"));
                    return;
                }
            }
            /* Valid for both Intel and AMD CPUs, although only in 64 bits mode for Intel. */
            pVM->cpum.s.aGuestCpuIdExt[1].edx |= X86_CPUID_EXT_FEATURE_EDX_SYSCALL;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled syscall/ret\n"));
            break;
        }

        /*
         * Set the PAE bit in both feature masks.
         * Assumes the caller knows what it's doing! (host must support these)
         */
        case CPUMCPUIDFEATURE_PAE:
        {
            if (!(ASMCpuId_EDX(1) & X86_CPUID_FEATURE_EDX_PAE))
            {
                LogRel(("WARNING: Can't turn on PAE when the host doesn't support it!!\n"));
                return;
            }

            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].edx |= X86_CPUID_FEATURE_EDX_PAE;
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001
                &&  pVM->cpum.s.enmGuestCpuVendor == CPUMCPUVENDOR_AMD)
                pVM->cpum.s.aGuestCpuIdExt[1].edx |= X86_CPUID_AMD_FEATURE_EDX_PAE;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled PAE\n"));
            break;
        }

        /*
         * Set the LONG MODE bit in the extended feature mask.
         * Assumes the caller knows what it's doing! (host must support these)
         */
        case CPUMCPUIDFEATURE_LONG_MODE:
        {
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax < 0x80000001
                ||  !(ASMCpuId_EDX(0x80000001) & X86_CPUID_EXT_FEATURE_EDX_LONG_MODE))
            {
                LogRel(("WARNING: Can't turn on LONG MODE when the host doesn't support it!!\n"));
                return;
            }

            /* Valid for both Intel and AMD. */
            pVM->cpum.s.aGuestCpuIdExt[1].edx |= X86_CPUID_EXT_FEATURE_EDX_LONG_MODE;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled LONG MODE\n"));
            break;
        }

        /*
         * Set the NX/XD bit in the extended feature mask.
         * Assumes the caller knows what it's doing! (host must support these)
         */
        case CPUMCPUIDFEATURE_NX:
        {
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax < 0x80000001
                ||  !(ASMCpuId_EDX(0x80000001) & X86_CPUID_EXT_FEATURE_EDX_NX))
            {
                LogRel(("WARNING: Can't turn on NX/XD when the host doesn't support it!!\n"));
                return;
            }

            /* Valid for both Intel and AMD. */
            pVM->cpum.s.aGuestCpuIdExt[1].edx |= X86_CPUID_EXT_FEATURE_EDX_NX;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled NX\n"));
            break;
        }

        /*
         * Set the LAHF/SAHF support in 64-bit mode.
         * Assumes the caller knows what it's doing! (host must support this)
         */
        case CPUMCPUIDFEATURE_LAHF:
        {
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax < 0x80000001
                ||  !(ASMCpuId_ECX(0x80000001) & X86_CPUID_EXT_FEATURE_ECX_LAHF_SAHF))
            {
                LogRel(("WARNING: Can't turn on LAHF/SAHF when the host doesn't support it!!\n"));
                return;
            }

            /* Valid for both Intel and AMD. */
            pVM->cpum.s.aGuestCpuIdExt[1].ecx |= X86_CPUID_EXT_FEATURE_ECX_LAHF_SAHF;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled LAHF/SAHF\n"));
            break;
        }

        case CPUMCPUIDFEATURE_PAT:
        {
            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].edx |= X86_CPUID_FEATURE_EDX_PAT;
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001
                &&  pVM->cpum.s.enmGuestCpuVendor == CPUMCPUVENDOR_AMD)
                pVM->cpum.s.aGuestCpuIdExt[1].edx |= X86_CPUID_AMD_FEATURE_EDX_PAT;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled PAT\n"));
            break;
        }

        /*
         * Set the RDTSCP support bit.
         * Assumes the caller knows what it's doing! (host must support this)
         */
        case CPUMCPUIDFEATURE_RDTSCP:
        {
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax < 0x80000001
                ||  !(ASMCpuId_EDX(0x80000001) & X86_CPUID_EXT_FEATURE_EDX_RDTSCP)
                ||  pVM->cpum.s.u8PortableCpuIdLevel > 0)
            {
                if (!pVM->cpum.s.u8PortableCpuIdLevel)
                    LogRel(("WARNING: Can't turn on RDTSCP when the host doesn't support it!!\n"));
                return;
            }

            /* Valid for both Intel and AMD. */
            pVM->cpum.s.aGuestCpuIdExt[1].edx |= X86_CPUID_EXT_FEATURE_EDX_RDTSCP;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled RDTSCP.\n"));
            break;
        }

       /*
        * Set the Hypervisor Present bit in the standard feature mask.
        */
        case CPUMCPUIDFEATURE_HVP:
            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].ecx |= X86_CPUID_FEATURE_ECX_HVP;
            LogRel(("CPUMSetGuestCpuIdFeature: Enabled Hypervisor Present bit\n"));
            break;

        default:
            AssertMsgFailed(("enmFeature=%d\n", enmFeature));
            break;
    }
    for (VMCPUID i = 0; i < pVM->cCpus; i++)
    {
        PVMCPU pVCpu = &pVM->aCpus[i];
        pVCpu->cpum.s.fChanged |= CPUM_CHANGED_CPUID;
    }
}


/**
 * Queries a CPUID feature bit.
 *
 * @returns boolean for feature presence
 * @param   pVM             Pointer to the VM.
 * @param   enmFeature      The feature to query.
 */
VMMDECL(bool) CPUMGetGuestCpuIdFeature(PVM pVM, CPUMCPUIDFEATURE enmFeature)
{
    switch (enmFeature)
    {
        case CPUMCPUIDFEATURE_PAE:
        {
            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                return !!(pVM->cpum.s.aGuestCpuIdStd[1].edx & X86_CPUID_FEATURE_EDX_PAE);
            break;
        }

        case CPUMCPUIDFEATURE_NX:
        {
            if (pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001)
                return !!(pVM->cpum.s.aGuestCpuIdExt[1].edx & X86_CPUID_EXT_FEATURE_EDX_NX);
        }

        case CPUMCPUIDFEATURE_RDTSCP:
        {
            if (pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001)
                return !!(pVM->cpum.s.aGuestCpuIdExt[1].edx & X86_CPUID_EXT_FEATURE_EDX_RDTSCP);
            break;
        }

        case CPUMCPUIDFEATURE_LONG_MODE:
        {
            if (pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001)
                return !!(pVM->cpum.s.aGuestCpuIdExt[1].edx & X86_CPUID_EXT_FEATURE_EDX_LONG_MODE);
            break;
        }

        default:
            AssertMsgFailed(("enmFeature=%d\n", enmFeature));
            break;
    }
    return false;
}


/**
 * Clears a CPUID feature bit.
 *
 * @param   pVM             Pointer to the VM.
 * @param   enmFeature      The feature to clear.
 */
VMMDECL(void) CPUMClearGuestCpuIdFeature(PVM pVM, CPUMCPUIDFEATURE enmFeature)
{
    switch (enmFeature)
    {
        /*
         * Set the APIC bit in both feature masks.
         */
        case CPUMCPUIDFEATURE_APIC:
            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].edx &= ~X86_CPUID_FEATURE_EDX_APIC;
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001
                &&  pVM->cpum.s.enmGuestCpuVendor == CPUMCPUVENDOR_AMD)
                pVM->cpum.s.aGuestCpuIdExt[1].edx &= ~X86_CPUID_AMD_FEATURE_EDX_APIC;
            Log(("CPUMClearGuestCpuIdFeature: Disabled APIC\n"));
            break;

        /*
         * Clear the x2APIC bit in the standard feature mask.
         */
        case CPUMCPUIDFEATURE_X2APIC:
            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].ecx &= ~X86_CPUID_FEATURE_ECX_X2APIC;
            LogRel(("CPUMClearGuestCpuIdFeature: Disabled x2APIC\n"));
            break;

        case CPUMCPUIDFEATURE_PAE:
        {
            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].edx &= ~X86_CPUID_FEATURE_EDX_PAE;
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001
                &&  pVM->cpum.s.enmGuestCpuVendor == CPUMCPUVENDOR_AMD)
                pVM->cpum.s.aGuestCpuIdExt[1].edx &= ~X86_CPUID_AMD_FEATURE_EDX_PAE;
            LogRel(("CPUMClearGuestCpuIdFeature: Disabled PAE!\n"));
            break;
        }

        case CPUMCPUIDFEATURE_PAT:
        {
            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].edx &= ~X86_CPUID_FEATURE_EDX_PAT;
            if (    pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001
                &&  pVM->cpum.s.enmGuestCpuVendor == CPUMCPUVENDOR_AMD)
                pVM->cpum.s.aGuestCpuIdExt[1].edx &= ~X86_CPUID_AMD_FEATURE_EDX_PAT;
            LogRel(("CPUMClearGuestCpuIdFeature: Disabled PAT!\n"));
            break;
        }

        case CPUMCPUIDFEATURE_LONG_MODE:
        {
            if (pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001)
                pVM->cpum.s.aGuestCpuIdExt[1].edx &= ~X86_CPUID_EXT_FEATURE_EDX_LONG_MODE;
            break;
        }

        case CPUMCPUIDFEATURE_LAHF:
        {
            if (pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001)
                pVM->cpum.s.aGuestCpuIdExt[1].ecx &= ~X86_CPUID_EXT_FEATURE_ECX_LAHF_SAHF;
            break;
        }

        case CPUMCPUIDFEATURE_RDTSCP:
        {
            if (pVM->cpum.s.aGuestCpuIdExt[0].eax >= 0x80000001)
                pVM->cpum.s.aGuestCpuIdExt[1].edx &= ~X86_CPUID_EXT_FEATURE_EDX_RDTSCP;
            LogRel(("CPUMClearGuestCpuIdFeature: Disabled RDTSCP!\n"));
            break;
        }

        case CPUMCPUIDFEATURE_HVP:
            if (pVM->cpum.s.aGuestCpuIdStd[0].eax >= 1)
                pVM->cpum.s.aGuestCpuIdStd[1].ecx &= ~X86_CPUID_FEATURE_ECX_HVP;
            break;

        default:
            AssertMsgFailed(("enmFeature=%d\n", enmFeature));
            break;
    }
    for (VMCPUID i = 0; i < pVM->cCpus; i++)
    {
        PVMCPU pVCpu = &pVM->aCpus[i];
        pVCpu->cpum.s.fChanged |= CPUM_CHANGED_CPUID;
    }
}


/**
 * Gets the host CPU vendor.
 *
 * @returns CPU vendor.
 * @param   pVM     Pointer to the VM.
 */
VMMDECL(CPUMCPUVENDOR) CPUMGetHostCpuVendor(PVM pVM)
{
    return pVM->cpum.s.enmHostCpuVendor;
}


/**
 * Gets the CPU vendor.
 *
 * @returns CPU vendor.
 * @param   pVM     Pointer to the VM.
 */
VMMDECL(CPUMCPUVENDOR) CPUMGetGuestCpuVendor(PVM pVM)
{
    return pVM->cpum.s.enmGuestCpuVendor;
}


VMMDECL(int) CPUMSetGuestDR0(PVMCPU pVCpu, uint64_t uDr0)
{
    pVCpu->cpum.s.Guest.dr[0] = uDr0;
    return CPUMRecalcHyperDRx(pVCpu);
}


VMMDECL(int) CPUMSetGuestDR1(PVMCPU pVCpu, uint64_t uDr1)
{
    pVCpu->cpum.s.Guest.dr[1] = uDr1;
    return CPUMRecalcHyperDRx(pVCpu);
}


VMMDECL(int) CPUMSetGuestDR2(PVMCPU pVCpu, uint64_t uDr2)
{
    pVCpu->cpum.s.Guest.dr[2] = uDr2;
    return CPUMRecalcHyperDRx(pVCpu);
}


VMMDECL(int) CPUMSetGuestDR3(PVMCPU pVCpu, uint64_t uDr3)
{
    pVCpu->cpum.s.Guest.dr[3] = uDr3;
    return CPUMRecalcHyperDRx(pVCpu);
}


VMMDECL(int) CPUMSetGuestDR6(PVMCPU pVCpu, uint64_t uDr6)
{
    pVCpu->cpum.s.Guest.dr[6] = uDr6;
    return CPUMRecalcHyperDRx(pVCpu);
}


VMMDECL(int) CPUMSetGuestDR7(PVMCPU pVCpu, uint64_t uDr7)
{
    pVCpu->cpum.s.Guest.dr[7] = uDr7;
    return CPUMRecalcHyperDRx(pVCpu);
}


VMMDECL(int) CPUMSetGuestDRx(PVMCPU pVCpu, uint32_t iReg, uint64_t Value)
{
    AssertReturn(iReg <= DISDREG_DR7, VERR_INVALID_PARAMETER);
    /* DR4 is an alias for DR6, and DR5 is an alias for DR7. */
    if (iReg == 4 || iReg == 5)
        iReg += 2;
    pVCpu->cpum.s.Guest.dr[iReg] = Value;
    return CPUMRecalcHyperDRx(pVCpu);
}


/**
 * Recalculates the hypervisor DRx register values based on
 * current guest registers and DBGF breakpoints.
 *
 * This is called whenever a guest DRx register is modified and when DBGF
 * sets a hardware breakpoint. In guest context this function will reload
 * any (hyper) DRx registers which comes out with a different value.
 *
 * @returns VINF_SUCCESS.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(int) CPUMRecalcHyperDRx(PVMCPU pVCpu)
{
    PVM pVM = pVCpu->CTX_SUFF(pVM);

    /*
     * Compare the DR7s first.
     *
     * We only care about the enabled flags. The GE and LE flags are always
     * set and we don't care if the guest doesn't set them. GD is virtualized
     * when we dispatch #DB, we never enable it.
     */
    const RTGCUINTREG uDbgfDr7 = DBGFBpGetDR7(pVM);
#ifdef CPUM_VIRTUALIZE_DRX
    const RTGCUINTREG uGstDr7  = CPUMGetGuestDR7(pVCpu);
#else
    const RTGCUINTREG uGstDr7  = 0;
#endif
    if ((uGstDr7 | uDbgfDr7) & X86_DR7_ENABLED_MASK)
    {
        /*
         * Ok, something is enabled. Recalc each of the breakpoints.
         * Straight forward code, not optimized/minimized in any way.
         */
        RTGCUINTREG uNewDr7 = X86_DR7_GE | X86_DR7_LE | X86_DR7_MB1_MASK;

        /* bp 0 */
        RTGCUINTREG uNewDr0;
        if (uDbgfDr7 & (X86_DR7_L0 | X86_DR7_G0))
        {
            uNewDr7 |= uDbgfDr7 & (X86_DR7_L0 | X86_DR7_G0 | X86_DR7_RW0_MASK | X86_DR7_LEN0_MASK);
            uNewDr0 = DBGFBpGetDR0(pVM);
        }
        else if (uGstDr7 & (X86_DR7_L0 | X86_DR7_G0))
        {
            uNewDr7 |= uGstDr7 & (X86_DR7_L0 | X86_DR7_G0 | X86_DR7_RW0_MASK | X86_DR7_LEN0_MASK);
            uNewDr0 = CPUMGetGuestDR0(pVCpu);
        }
        else
            uNewDr0 = pVCpu->cpum.s.Hyper.dr[0];

        /* bp 1 */
        RTGCUINTREG uNewDr1;
        if (uDbgfDr7 & (X86_DR7_L1 | X86_DR7_G1))
        {
            uNewDr7 |= uDbgfDr7 & (X86_DR7_L1 | X86_DR7_G1 | X86_DR7_RW1_MASK | X86_DR7_LEN1_MASK);
            uNewDr1 = DBGFBpGetDR1(pVM);
        }
        else if (uGstDr7 & (X86_DR7_L1 | X86_DR7_G1))
        {
            uNewDr7 |= uGstDr7 & (X86_DR7_L1 | X86_DR7_G1 | X86_DR7_RW1_MASK | X86_DR7_LEN1_MASK);
            uNewDr1 = CPUMGetGuestDR1(pVCpu);
        }
        else
            uNewDr1 = pVCpu->cpum.s.Hyper.dr[1];

        /* bp 2 */
        RTGCUINTREG uNewDr2;
        if (uDbgfDr7 & (X86_DR7_L2 | X86_DR7_G2))
        {
            uNewDr7 |= uDbgfDr7 & (X86_DR7_L2 | X86_DR7_G2 | X86_DR7_RW2_MASK | X86_DR7_LEN2_MASK);
            uNewDr2 = DBGFBpGetDR2(pVM);
        }
        else if (uGstDr7 & (X86_DR7_L2 | X86_DR7_G2))
        {
            uNewDr7 |= uGstDr7 & (X86_DR7_L2 | X86_DR7_G2 | X86_DR7_RW2_MASK | X86_DR7_LEN2_MASK);
            uNewDr2 = CPUMGetGuestDR2(pVCpu);
        }
        else
            uNewDr2 = pVCpu->cpum.s.Hyper.dr[2];

        /* bp 3 */
        RTGCUINTREG uNewDr3;
        if (uDbgfDr7 & (X86_DR7_L3 | X86_DR7_G3))
        {
            uNewDr7 |= uDbgfDr7 & (X86_DR7_L3 | X86_DR7_G3 | X86_DR7_RW3_MASK | X86_DR7_LEN3_MASK);
            uNewDr3 = DBGFBpGetDR3(pVM);
        }
        else if (uGstDr7 & (X86_DR7_L3 | X86_DR7_G3))
        {
            uNewDr7 |= uGstDr7 & (X86_DR7_L3 | X86_DR7_G3 | X86_DR7_RW3_MASK | X86_DR7_LEN3_MASK);
            uNewDr3 = CPUMGetGuestDR3(pVCpu);
        }
        else
            uNewDr3 = pVCpu->cpum.s.Hyper.dr[3];

        /*
         * Apply the updates.
         */
#ifdef IN_RC
        if (!(pVCpu->cpum.s.fUseFlags & CPUM_USE_DEBUG_REGS))
        {
            /** @todo save host DBx registers. */
        }
#endif
        pVCpu->cpum.s.fUseFlags |= CPUM_USE_DEBUG_REGS;
        if (uNewDr3 != pVCpu->cpum.s.Hyper.dr[3])
            CPUMSetHyperDR3(pVCpu, uNewDr3);
        if (uNewDr2 != pVCpu->cpum.s.Hyper.dr[2])
            CPUMSetHyperDR2(pVCpu, uNewDr2);
        if (uNewDr1 != pVCpu->cpum.s.Hyper.dr[1])
            CPUMSetHyperDR1(pVCpu, uNewDr1);
        if (uNewDr0 != pVCpu->cpum.s.Hyper.dr[0])
            CPUMSetHyperDR0(pVCpu, uNewDr0);
        if (uNewDr7 != pVCpu->cpum.s.Hyper.dr[7])
            CPUMSetHyperDR7(pVCpu, uNewDr7);
    }
    else
    {
#ifdef IN_RC
        if (pVCpu->cpum.s.fUseFlags & CPUM_USE_DEBUG_REGS)
        {
            /** @todo restore host DBx registers. */
        }
#endif
        pVCpu->cpum.s.fUseFlags &= ~CPUM_USE_DEBUG_REGS;
    }
    Log2(("CPUMRecalcHyperDRx: fUseFlags=%#x %RGr %RGr %RGr %RGr  %RGr %RGr\n",
          pVCpu->cpum.s.fUseFlags, pVCpu->cpum.s.Hyper.dr[0], pVCpu->cpum.s.Hyper.dr[1],
         pVCpu->cpum.s.Hyper.dr[2], pVCpu->cpum.s.Hyper.dr[3], pVCpu->cpum.s.Hyper.dr[6],
         pVCpu->cpum.s.Hyper.dr[7]));

    return VINF_SUCCESS;
}


/**
 * Tests if the guest has No-Execute Page Protection Enabled (NXE).
 *
 * @returns true if in real mode, otherwise false.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestNXEnabled(PVMCPU pVCpu)
{
    return !!(pVCpu->cpum.s.Guest.msrEFER & MSR_K6_EFER_NXE);
}


/**
 * Tests if the guest has the Page Size Extension enabled (PSE).
 *
 * @returns true if in real mode, otherwise false.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestPageSizeExtEnabled(PVMCPU pVCpu)
{
    /* PAE or AMD64 implies support for big pages regardless of CR4.PSE */
    return !!(pVCpu->cpum.s.Guest.cr4 & (X86_CR4_PSE | X86_CR4_PAE));
}


/**
 * Tests if the guest has the paging enabled (PG).
 *
 * @returns true if in real mode, otherwise false.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestPagingEnabled(PVMCPU pVCpu)
{
    return !!(pVCpu->cpum.s.Guest.cr0 & X86_CR0_PG);
}


/**
 * Tests if the guest has the paging enabled (PG).
 *
 * @returns true if in real mode, otherwise false.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestR0WriteProtEnabled(PVMCPU pVCpu)
{
    return !!(pVCpu->cpum.s.Guest.cr0 & X86_CR0_WP);
}


/**
 * Tests if the guest is running in real mode or not.
 *
 * @returns true if in real mode, otherwise false.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestInRealMode(PVMCPU pVCpu)
{
    return !(pVCpu->cpum.s.Guest.cr0 & X86_CR0_PE);
}


/**
 * Tests if the guest is running in real or virtual 8086 mode.
 *
 * @returns @c true if it is, @c false if not.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestInRealOrV86Mode(PVMCPU pVCpu)
{
    return !(pVCpu->cpum.s.Guest.cr0 & X86_CR0_PE)
        || pVCpu->cpum.s.Guest.eflags.Bits.u1VM; /** @todo verify that this cannot be set in long mode. */
}


/**
 * Tests if the guest is running in protected or not.
 *
 * @returns true if in protected mode, otherwise false.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestInProtectedMode(PVMCPU pVCpu)
{
    return !!(pVCpu->cpum.s.Guest.cr0 & X86_CR0_PE);
}


/**
 * Tests if the guest is running in paged protected or not.
 *
 * @returns true if in paged protected mode, otherwise false.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestInPagedProtectedMode(PVMCPU pVCpu)
{
    return (pVCpu->cpum.s.Guest.cr0 & (X86_CR0_PE | X86_CR0_PG)) == (X86_CR0_PE | X86_CR0_PG);
}


/**
 * Tests if the guest is running in long mode or not.
 *
 * @returns true if in long mode, otherwise false.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestInLongMode(PVMCPU pVCpu)
{
    return (pVCpu->cpum.s.Guest.msrEFER & MSR_K6_EFER_LMA) == MSR_K6_EFER_LMA;
}


/**
 * Tests if the guest is running in PAE mode or not.
 *
 * @returns true if in PAE mode, otherwise false.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestInPAEMode(PVMCPU pVCpu)
{
    return (pVCpu->cpum.s.Guest.cr4 & X86_CR4_PAE)
        && (pVCpu->cpum.s.Guest.cr0 & (X86_CR0_PE | X86_CR0_PG)) == (X86_CR0_PE | X86_CR0_PG)
        && !(pVCpu->cpum.s.Guest.msrEFER & MSR_K6_EFER_LMA);
}


/**
 * Tests if the guest is running in 64 bits mode or not.
 *
 * @returns true if in 64 bits protected mode, otherwise false.
 * @param   pVCpu       The current virtual CPU.
 */
VMMDECL(bool) CPUMIsGuestIn64BitCode(PVMCPU pVCpu)
{
    if (!CPUMIsGuestInLongMode(pVCpu))
        return false;
    CPUMSELREG_LAZY_LOAD_HIDDEN_PARTS(pVCpu, &pVCpu->cpum.s.Guest.cs);
    return pVCpu->cpum.s.Guest.cs.Attr.n.u1Long;
}


/**
 * Helper for CPUMIsGuestIn64BitCodeEx that handles lazy resolving of hidden CS
 * registers.
 *
 * @returns true if in 64 bits protected mode, otherwise false.
 * @param   pCtx        Pointer to the current guest CPU context.
 */
VMM_INT_DECL(bool) CPUMIsGuestIn64BitCodeSlow(PCPUMCTX pCtx)
{
    return CPUMIsGuestIn64BitCode(CPUM_GUEST_CTX_TO_VMCPU(pCtx));
}

#ifdef VBOX_WITH_RAW_MODE_NOT_R0
/**
 *
 * @returns @c true if we've entered raw-mode and selectors with RPL=1 are
 *          really RPL=0, @c false if we've not (RPL=1 really is RPL=1).
 * @param   pVCpu       The current virtual CPU.
 */
VMM_INT_DECL(bool) CPUMIsGuestInRawMode(PVMCPU pVCpu)
{
    return pVCpu->cpum.s.fRawEntered;
}
#endif


/**
 * Updates the EFLAGS while we're in raw-mode.
 *
 * @param   pVCpu       Pointer to the VMCPU.
 * @param   fEfl        The new EFLAGS value.
 */
VMMDECL(void) CPUMRawSetEFlags(PVMCPU pVCpu, uint32_t fEfl)
{
#ifdef VBOX_WITH_RAW_MODE_NOT_R0
    if (pVCpu->cpum.s.fRawEntered)
        PATMRawSetEFlags(pVCpu->CTX_SUFF(pVM), CPUMCTX2CORE(&pVCpu->cpum.s.Guest), fEfl);
    else
#endif
        pVCpu->cpum.s.Guest.eflags.u32 = fEfl;
}


/**
 * Gets the EFLAGS while we're in raw-mode.
 *
 * @returns The eflags.
 * @param   pVCpu       Pointer to the current virtual CPU.
 */
VMMDECL(uint32_t) CPUMRawGetEFlags(PVMCPU pVCpu)
{
#ifdef VBOX_WITH_RAW_MODE_NOT_R0
    if (pVCpu->cpum.s.fRawEntered)
        return PATMRawGetEFlags(pVCpu->CTX_SUFF(pVM), CPUMCTX2CORE(&pVCpu->cpum.s.Guest));
#endif
    return pVCpu->cpum.s.Guest.eflags.u32;
}


/**
 * Sets the specified changed flags (CPUM_CHANGED_*).
 *
 * @param   pVCpu       Pointer to the current virtual CPU.
 */
VMMDECL(void) CPUMSetChangedFlags(PVMCPU pVCpu, uint32_t fChangedFlags)
{
    pVCpu->cpum.s.fChanged |= fChangedFlags;
}


/**
 * Checks if the CPU supports the FXSAVE and FXRSTOR instruction.
 * @returns true if supported.
 * @returns false if not supported.
 * @param   pVM     Pointer to the VM.
 */
VMMDECL(bool) CPUMSupportsFXSR(PVM pVM)
{
    return pVM->cpum.s.CPUFeatures.edx.u1FXSR != 0;
}


/**
 * Checks if the host OS uses the SYSENTER / SYSEXIT instructions.
 * @returns true if used.
 * @returns false if not used.
 * @param   pVM       Pointer to the VM.
 */
VMMDECL(bool) CPUMIsHostUsingSysEnter(PVM pVM)
{
    return (pVM->cpum.s.fHostUseFlags & CPUM_USE_SYSENTER) != 0;
}


/**
 * Checks if the host OS uses the SYSCALL / SYSRET instructions.
 * @returns true if used.
 * @returns false if not used.
 * @param   pVM       Pointer to the VM.
 */
VMMDECL(bool) CPUMIsHostUsingSysCall(PVM pVM)
{
    return (pVM->cpum.s.fHostUseFlags & CPUM_USE_SYSCALL) != 0;
}

#ifndef IN_RING3

/**
 * Lazily sync in the FPU/XMM state.
 *
 * @returns VBox status code.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(int) CPUMHandleLazyFPU(PVMCPU pVCpu)
{
    return cpumHandleLazyFPUAsm(&pVCpu->cpum.s);
}

#endif /* !IN_RING3 */

/**
 * Checks if we activated the FPU/XMM state of the guest OS.
 * @returns true if we did.
 * @returns false if not.
 * @param   pVCpu   Pointer to the VMCPU.
 */
VMMDECL(bool) CPUMIsGuestFPUStateActive(PVMCPU pVCpu)
{
    return (pVCpu->cpum.s.fUseFlags & CPUM_USED_FPU) != 0;
}


/**
 * Deactivate the FPU/XMM state of the guest OS.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(void) CPUMDeactivateGuestFPUState(PVMCPU pVCpu)
{
    pVCpu->cpum.s.fUseFlags &= ~CPUM_USED_FPU;
}


/**
 * Checks if the guest debug state is active.
 *
 * @returns boolean
 * @param   pVM         Pointer to the VM.
 */
VMMDECL(bool) CPUMIsGuestDebugStateActive(PVMCPU pVCpu)
{
    return (pVCpu->cpum.s.fUseFlags & CPUM_USE_DEBUG_REGS) != 0;
}

/**
 * Checks if the hyper debug state is active.
 *
 * @returns boolean
 * @param   pVM         Pointer to the VM.
 */
VMMDECL(bool) CPUMIsHyperDebugStateActive(PVMCPU pVCpu)
{
    return (pVCpu->cpum.s.fUseFlags & CPUM_USE_DEBUG_REGS_HYPER) != 0;
}


/**
 * Mark the guest's debug state as inactive.
 *
 * @returns boolean
 * @param   pVM         Pointer to the VM.
 */
VMMDECL(void) CPUMDeactivateGuestDebugState(PVMCPU pVCpu)
{
    pVCpu->cpum.s.fUseFlags &= ~CPUM_USE_DEBUG_REGS;
}


/**
 * Mark the hypervisor's debug state as inactive.
 *
 * @returns boolean
 * @param   pVM         Pointer to the VM.
 */
VMMDECL(void) CPUMDeactivateHyperDebugState(PVMCPU pVCpu)
{
    pVCpu->cpum.s.fUseFlags &= ~CPUM_USE_DEBUG_REGS_HYPER;
}


/**
 * Get the current privilege level of the guest.
 *
 * @returns CPL
 * @param   pVCpu       Pointer to the current virtual CPU.
 */
VMMDECL(uint32_t) CPUMGetGuestCPL(PVMCPU pVCpu)
{
    /*
     * CPL can reliably be found in SS.DPL (hidden regs valid) or SS if not.
     *
     * Note! We used to check CS.DPL here, assuming it was always equal to
     * CPL even if a conforming segment was loaded.  But this truned out to
     * only apply to older AMD-V.  With VT-x we had an ACP2 regression
     * during install after a far call to ring 2 with VT-x.  Then on newer
     * AMD-V CPUs we have to move the VMCB.guest.u8CPL into cs.Attr.n.u2Dpl
     * as well as ss.Attr.n.u2Dpl to make this (and other) code work right.
     *
     * So, forget CS.DPL, always use SS.DPL.
     *
     * Note! The SS RPL is always equal to the CPL, while the CS RPL
     * isn't necessarily equal if the segment is conforming.
     * See section 4.11.1 in the AMD manual.
     */
    uint32_t uCpl;
    if (pVCpu->cpum.s.Guest.cr0 & X86_CR0_PE)
    {
        if (!pVCpu->cpum.s.Guest.eflags.Bits.u1VM)
        {
            if (CPUMSELREG_ARE_HIDDEN_PARTS_VALID(pVCpu, &pVCpu->cpum.s.Guest.ss))
                uCpl = pVCpu->cpum.s.Guest.ss.Attr.n.u2Dpl;
            else
            {
                uCpl = (pVCpu->cpum.s.Guest.ss.Sel & X86_SEL_RPL);
#ifdef VBOX_WITH_RAW_MODE_NOT_R0
# ifdef VBOX_WITH_RAW_RING1
                if (pVCpu->cpum.s.fRawEntered)
                {
                    if (   uCpl == 2
                        && EMIsRawRing1Enabled(pVCpu->CTX_SUFF(pVM)))
                        uCpl = 1;
                    else if (uCpl == 1)
                        uCpl = 0;
                }
                Assert(uCpl != 2);  /* ring 2 support not allowed anymore. */
# else
                if (uCpl == 1)
                    uCpl = 0;
# endif
#endif
            }
        }
        else
            uCpl = 3; /* V86 has CPL=3; REM doesn't set DPL=3 in V8086 mode. See @bugref{5130}. */
    }
    else
        uCpl = 0;     /* Real mode is zero; CPL set to 3 for VT-x real-mode emulation. */
    return uCpl;
}


/**
 * Gets the current guest CPU mode.
 *
 * If paging mode is what you need, check out PGMGetGuestMode().
 *
 * @returns The CPU mode.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMDECL(CPUMMODE) CPUMGetGuestMode(PVMCPU pVCpu)
{
    CPUMMODE enmMode;
    if (!(pVCpu->cpum.s.Guest.cr0 & X86_CR0_PE))
        enmMode = CPUMMODE_REAL;
    else if (!(pVCpu->cpum.s.Guest.msrEFER & MSR_K6_EFER_LMA))
        enmMode = CPUMMODE_PROTECTED;
    else
        enmMode = CPUMMODE_LONG;

    return enmMode;
}


/**
 * Figure whether the CPU is currently executing 16, 32 or 64 bit code.
 *
 * @returns 16, 32 or 64.
 * @param   pVCpu               The current virtual CPU.
 */
VMMDECL(uint32_t)       CPUMGetGuestCodeBits(PVMCPU pVCpu)
{
    if (!(pVCpu->cpum.s.Guest.cr0 & X86_CR0_PE))
        return 16;

    if (pVCpu->cpum.s.Guest.eflags.Bits.u1VM)
    {
        Assert(!(pVCpu->cpum.s.Guest.msrEFER & MSR_K6_EFER_LMA));
        return 16;
    }

    CPUMSELREG_LAZY_LOAD_HIDDEN_PARTS(pVCpu, &pVCpu->cpum.s.Guest.cs);
    if (   pVCpu->cpum.s.Guest.cs.Attr.n.u1Long
        && (pVCpu->cpum.s.Guest.msrEFER & MSR_K6_EFER_LMA))
        return 64;

    if (pVCpu->cpum.s.Guest.cs.Attr.n.u1DefBig)
        return 32;

    return 16;
}


VMMDECL(DISCPUMODE)     CPUMGetGuestDisMode(PVMCPU pVCpu)
{
    if (!(pVCpu->cpum.s.Guest.cr0 & X86_CR0_PE))
        return DISCPUMODE_16BIT;

    if (pVCpu->cpum.s.Guest.eflags.Bits.u1VM)
    {
        Assert(!(pVCpu->cpum.s.Guest.msrEFER & MSR_K6_EFER_LMA));
        return DISCPUMODE_16BIT;
    }

    CPUMSELREG_LAZY_LOAD_HIDDEN_PARTS(pVCpu, &pVCpu->cpum.s.Guest.cs);
    if (   pVCpu->cpum.s.Guest.cs.Attr.n.u1Long
        && (pVCpu->cpum.s.Guest.msrEFER & MSR_K6_EFER_LMA))
        return DISCPUMODE_64BIT;

    if (pVCpu->cpum.s.Guest.cs.Attr.n.u1DefBig)
        return DISCPUMODE_32BIT;

    return DISCPUMODE_16BIT;
}

