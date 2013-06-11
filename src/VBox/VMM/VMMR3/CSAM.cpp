/* $Id: CSAM.cpp $ */
/** @file
 * CSAM - Guest OS Code Scanning and Analysis Manager
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
#define LOG_GROUP LOG_GROUP_CSAM
#include <VBox/vmm/cpum.h>
#include <VBox/vmm/stam.h>
#include <VBox/vmm/patm.h>
#include <VBox/vmm/csam.h>
#include <VBox/vmm/cpumdis.h>
#include <VBox/vmm/pgm.h>
#include <VBox/vmm/iom.h>
#include <VBox/sup.h>
#include <VBox/vmm/mm.h>
#include <VBox/vmm/em.h>
#ifdef VBOX_WITH_REM
# include <VBox/vmm/rem.h>
#endif
#include <VBox/vmm/selm.h>
#include <VBox/vmm/trpm.h>
#include <VBox/vmm/cfgm.h>
#include <VBox/param.h>
#include <iprt/avl.h>
#include <iprt/asm.h>
#include <iprt/thread.h>
#include "CSAMInternal.h"
#include <VBox/vmm/vm.h>
#include <VBox/dbg.h>
#include <VBox/err.h>
#include <VBox/vmm/ssm.h>
#include <VBox/log.h>
#include <iprt/assert.h>
#include <iprt/string.h>
#include <VBox/dis.h>
#include <VBox/disopcode.h>
#include "internal/pgm.h"


/* Enabled by default */
#define CSAM_ENABLE

/* Enable to monitor code pages for self-modifying code. */
#define CSAM_MONITOR_CODE_PAGES
/* Enable to monitor all scanned pages
#define CSAM_MONITOR_CSAM_CODE_PAGES */
/* Enable to scan beyond ret instructions.
#define CSAM_ANALYSE_BEYOND_RET */

/*******************************************************************************
*   Internal Functions                                                         *
*******************************************************************************/
static DECLCALLBACK(int) csamr3Save(PVM pVM, PSSMHANDLE pSSM);
static DECLCALLBACK(int) csamr3Load(PVM pVM, PSSMHANDLE pSSM, uint32_t uVersion, uint32_t uPass);
static DECLCALLBACK(int) CSAMCodePageWriteHandler(PVM pVM, RTGCPTR GCPtr, void *pvPtr, void *pvBuf, size_t cbBuf, PGMACCESSTYPE enmAccessType, void *pvUser);
static DECLCALLBACK(int) CSAMCodePageInvalidate(PVM pVM, RTGCPTR GCPtr);

bool                csamIsCodeScanned(PVM pVM, RTRCPTR pInstr, PCSAMPAGE *pPage);
int                 csamR3CheckPageRecord(PVM pVM, RTRCPTR pInstr);
static PCSAMPAGE    csamCreatePageRecord(PVM pVM, RTRCPTR GCPtr, CSAMTAG enmTag, bool fCode32, bool fMonitorInvalidation = false);
static int          csamRemovePageRecord(PVM pVM, RTRCPTR GCPtr);
static int          csamReinit(PVM pVM);
static void         csamMarkCode(PVM pVM, PCSAMPAGE pPage, RTRCPTR pInstr, uint32_t opsize, bool fScanned);
static int          csamAnalyseCodeStream(PVM pVM, RCPTRTYPE(uint8_t *) pInstrGC, RCPTRTYPE(uint8_t *) pCurInstrGC, bool fCode32,
                                          PFN_CSAMR3ANALYSE pfnCSAMR3Analyse, void *pUserData, PCSAMP2GLOOKUPREC pCacheRec);

/** @todo Temporary for debugging. */
static bool fInCSAMCodePageInvalidate = false;

/*******************************************************************************
*   Global Variables                                                           *
*******************************************************************************/
#ifdef VBOX_WITH_DEBUGGER
static DECLCALLBACK(int) csamr3CmdOn(PCDBGCCMD pCmd, PDBGCCMDHLP pCmdHlp, PVM pVM, PCDBGCVAR paArgs, unsigned cArgs);
static DECLCALLBACK(int) csamr3CmdOff(PCDBGCCMD pCmd, PDBGCCMDHLP pCmdHlp, PVM pVM, PCDBGCVAR paArgs, unsigned cArgs);

/** Command descriptors. */
static const DBGCCMD    g_aCmds[] =
{
    /* pszCmd,      cArgsMin, cArgsMax, paArgDesc,  cArgDescs,  fFlags, pfnHandler     pszSyntax, ....pszDescription */
    { "csamon",     0,        0,        NULL,       0,          0,      csamr3CmdOn,   "",        "Enable CSAM code scanning."  },
    { "csamoff",    0,        0,        NULL,       0,          0,      csamr3CmdOff,  "",        "Disable CSAM code scanning." },
};
#endif

/**
 * SSM descriptor table for the CSAM structure.
 */
static const SSMFIELD g_aCsamFields[] =
{
    /** @todo there are more fields that can be ignored here. */
    SSMFIELD_ENTRY_IGNORE(      CSAM, offVM),
    SSMFIELD_ENTRY_PAD_HC64(    CSAM, Alignment0, sizeof(uint32_t)),
    SSMFIELD_ENTRY_IGN_HCPTR(   CSAM, pPageTree),
    SSMFIELD_ENTRY(             CSAM, aDangerousInstr),
    SSMFIELD_ENTRY(             CSAM, cDangerousInstr),
    SSMFIELD_ENTRY(             CSAM, iDangerousInstr),
    SSMFIELD_ENTRY_RCPTR(       CSAM, pPDBitmapGC),   /// @todo ignore this?
    SSMFIELD_ENTRY_RCPTR(       CSAM, pPDHCBitmapGC), /// @todo ignore this?
    SSMFIELD_ENTRY_IGN_HCPTR(   CSAM, pPDBitmapHC),
    SSMFIELD_ENTRY_IGN_HCPTR(   CSAM, pPDGCBitmapHC),
    SSMFIELD_ENTRY_IGN_HCPTR(   CSAM, savedstate.pSSM),
    SSMFIELD_ENTRY(             CSAM, savedstate.cPageRecords),
    SSMFIELD_ENTRY(             CSAM, savedstate.cPatchPageRecords),
    SSMFIELD_ENTRY(             CSAM, cDirtyPages),
    SSMFIELD_ENTRY_RCPTR_ARRAY( CSAM, pvDirtyBasePage),
    SSMFIELD_ENTRY_RCPTR_ARRAY( CSAM, pvDirtyFaultPage),
    SSMFIELD_ENTRY(             CSAM, cPossibleCodePages),
    SSMFIELD_ENTRY_RCPTR_ARRAY( CSAM, pvPossibleCodePage),
    SSMFIELD_ENTRY_RCPTR_ARRAY( CSAM, pvCallInstruction),
    SSMFIELD_ENTRY(             CSAM, iCallInstruction),
    SSMFIELD_ENTRY(             CSAM, fScanningStarted),
    SSMFIELD_ENTRY(             CSAM, fGatesChecked),
    SSMFIELD_ENTRY_PAD_HC(      CSAM, Alignment1, 6, 2),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrTraps),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrPages),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrPagesInv),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrRemovedPages),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrPatchPages),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrPageNPHC),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrPageNPGC),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrFlushes),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrFlushesSkipped),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrKnownPagesHC),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrKnownPagesGC),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrInstr),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrBytesRead),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrOpcodeRead),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatTime),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatTimeCheckAddr),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatTimeAddrConv),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatTimeFlushPage),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatTimeDisasm),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatFlushDirtyPages),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatCheckGates),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatCodePageModified),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatDangerousWrite),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatInstrCacheHit),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatInstrCacheMiss),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatPagePATM),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatPageCSAM),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatPageREM),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatNrUserPages),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatPageMonitor),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatPageRemoveREMFlush),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatBitmapAlloc),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatScanNextFunction),
    SSMFIELD_ENTRY_IGNORE(      CSAM, StatScanNextFunctionFailed),
    SSMFIELD_ENTRY_TERM()
};

/** Fake type to simplify g_aCsamPDBitmapArray construction. */
typedef struct
{
    uint8_t *a[CSAM_PGDIRBMP_CHUNKS];
} CSAMPDBITMAPARRAY;

/**
 * SSM descriptor table for the CSAM::pPDBitmapHC array.
 */
static SSMFIELD const g_aCsamPDBitmapArray[] =
{
    SSMFIELD_ENTRY_HCPTR_NI_ARRAY(CSAMPDBITMAPARRAY, a),
    SSMFIELD_ENTRY_TERM()
};

/**
 * SSM descriptor table for the CSAMPAGEREC structure.
 */
static const SSMFIELD g_aCsamPageRecFields[] =
{
    SSMFIELD_ENTRY_IGN_HCPTR(   CSAMPAGEREC, Core.Key),
    SSMFIELD_ENTRY_IGN_HCPTR(   CSAMPAGEREC, Core.pLeft),
    SSMFIELD_ENTRY_IGN_HCPTR(   CSAMPAGEREC, Core.pRight),
    SSMFIELD_ENTRY_IGNORE(      CSAMPAGEREC, Core.uchHeight),
    SSMFIELD_ENTRY_PAD_HC_AUTO( 3, 7),
    SSMFIELD_ENTRY_RCPTR(       CSAMPAGEREC, page.pPageGC),
    SSMFIELD_ENTRY_PAD_HC_AUTO( 0, 4),
    SSMFIELD_ENTRY_PAD_MSC32_AUTO( 4),
    SSMFIELD_ENTRY_GCPHYS(      CSAMPAGEREC, page.GCPhys),
    SSMFIELD_ENTRY(             CSAMPAGEREC, page.fFlags),
    SSMFIELD_ENTRY(             CSAMPAGEREC, page.uSize),
    SSMFIELD_ENTRY_PAD_HC_AUTO( 0, 4),
    SSMFIELD_ENTRY_HCPTR_NI(    CSAMPAGEREC, page.pBitmap),
    SSMFIELD_ENTRY(             CSAMPAGEREC, page.fCode32),
    SSMFIELD_ENTRY(             CSAMPAGEREC, page.fMonitorActive),
    SSMFIELD_ENTRY(             CSAMPAGEREC, page.fMonitorInvalidation),
    SSMFIELD_ENTRY_PAD_HC_AUTO( 1, 1),
    SSMFIELD_ENTRY(             CSAMPAGEREC, page.enmTag),
    SSMFIELD_ENTRY(             CSAMPAGEREC, page.u64Hash),
    SSMFIELD_ENTRY_TERM()
};


/**
 * Initializes the CSAM.
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 */
VMMR3DECL(int) CSAMR3Init(PVM pVM)
{
    int rc;

    LogFlow(("CSAMR3Init\n"));

    /* Allocate bitmap for the page directory. */
    rc = MMR3HyperAllocOnceNoRel(pVM, CSAM_PGDIRBMP_CHUNKS*sizeof(RTHCPTR), 0, MM_TAG_CSAM, (void **)&pVM->csam.s.pPDBitmapHC);
    AssertRCReturn(rc, rc);
    rc = MMR3HyperAllocOnceNoRel(pVM, CSAM_PGDIRBMP_CHUNKS*sizeof(RTRCPTR), 0, MM_TAG_CSAM, (void **)&pVM->csam.s.pPDGCBitmapHC);
    AssertRCReturn(rc, rc);
    pVM->csam.s.pPDBitmapGC   = MMHyperR3ToRC(pVM, pVM->csam.s.pPDGCBitmapHC);
    pVM->csam.s.pPDHCBitmapGC = MMHyperR3ToRC(pVM, pVM->csam.s.pPDBitmapHC);

    rc = csamReinit(pVM);
    AssertRCReturn(rc, rc);

    /*
     * Register save and load state notifiers.
     */
    rc = SSMR3RegisterInternal(pVM, "CSAM", 0, CSAM_SSM_VERSION, sizeof(pVM->csam.s) + PAGE_SIZE*16,
                               NULL, NULL, NULL,
                               NULL, csamr3Save, NULL,
                               NULL, csamr3Load, NULL);
    AssertRCReturn(rc, rc);

    STAM_REG(pVM, &pVM->csam.s.StatNrTraps,          STAMTYPE_COUNTER,   "/CSAM/PageTraps",           STAMUNIT_OCCURENCES,     "The number of CSAM page traps.");
    STAM_REG(pVM, &pVM->csam.s.StatDangerousWrite,   STAMTYPE_COUNTER,   "/CSAM/DangerousWrites",     STAMUNIT_OCCURENCES,     "The number of dangerous writes that cause a context switch.");

    STAM_REG(pVM, &pVM->csam.s.StatNrPageNPHC,       STAMTYPE_COUNTER,   "/CSAM/HC/PageNotPresent",   STAMUNIT_OCCURENCES,     "The number of CSAM pages marked not present.");
    STAM_REG(pVM, &pVM->csam.s.StatNrPageNPGC,       STAMTYPE_COUNTER,   "/CSAM/GC/PageNotPresent",   STAMUNIT_OCCURENCES,     "The number of CSAM pages marked not present.");
    STAM_REG(pVM, &pVM->csam.s.StatNrPages,          STAMTYPE_COUNTER,   "/CSAM/PageRec/AddedRW",     STAMUNIT_OCCURENCES,     "The number of CSAM page records (RW monitoring).");
    STAM_REG(pVM, &pVM->csam.s.StatNrPagesInv,       STAMTYPE_COUNTER,   "/CSAM/PageRec/AddedRWI",    STAMUNIT_OCCURENCES,     "The number of CSAM page records (RW & invalidation monitoring).");
    STAM_REG(pVM, &pVM->csam.s.StatNrRemovedPages,   STAMTYPE_COUNTER,   "/CSAM/PageRec/Removed",     STAMUNIT_OCCURENCES,     "The number of removed CSAM page records.");
    STAM_REG(pVM, &pVM->csam.s.StatPageRemoveREMFlush,STAMTYPE_COUNTER,  "/CSAM/PageRec/Removed/REMFlush",     STAMUNIT_OCCURENCES,     "The number of removed CSAM page records that caused a REM flush.");

    STAM_REG(pVM, &pVM->csam.s.StatNrPatchPages,     STAMTYPE_COUNTER,   "/CSAM/PageRec/Patch",       STAMUNIT_OCCURENCES,     "The number of CSAM patch page records.");
    STAM_REG(pVM, &pVM->csam.s.StatNrUserPages,      STAMTYPE_COUNTER,   "/CSAM/PageRec/Ignore/User", STAMUNIT_OCCURENCES,     "The number of CSAM user page records (ignored).");
    STAM_REG(pVM, &pVM->csam.s.StatPagePATM,         STAMTYPE_COUNTER,   "/CSAM/PageRec/Type/PATM",   STAMUNIT_OCCURENCES,     "The number of PATM page records.");
    STAM_REG(pVM, &pVM->csam.s.StatPageCSAM,         STAMTYPE_COUNTER,   "/CSAM/PageRec/Type/CSAM",   STAMUNIT_OCCURENCES,     "The number of CSAM page records.");
    STAM_REG(pVM, &pVM->csam.s.StatPageREM,          STAMTYPE_COUNTER,   "/CSAM/PageRec/Type/REM",    STAMUNIT_OCCURENCES,     "The number of REM page records.");
    STAM_REG(pVM, &pVM->csam.s.StatPageMonitor,      STAMTYPE_COUNTER,   "/CSAM/PageRec/Monitored",   STAMUNIT_OCCURENCES,     "The number of monitored pages.");

    STAM_REG(pVM, &pVM->csam.s.StatCodePageModified, STAMTYPE_COUNTER, "/CSAM/Monitor/DirtyPage",     STAMUNIT_OCCURENCES,     "The number of code page modifications.");

    STAM_REG(pVM, &pVM->csam.s.StatNrFlushes,        STAMTYPE_COUNTER, "/CSAM/PageFlushes",           STAMUNIT_OCCURENCES,     "The number of CSAM page flushes.");
    STAM_REG(pVM, &pVM->csam.s.StatNrFlushesSkipped, STAMTYPE_COUNTER, "/CSAM/PageFlushesSkipped",    STAMUNIT_OCCURENCES,     "The number of CSAM page flushes that were skipped.");
    STAM_REG(pVM, &pVM->csam.s.StatNrKnownPagesHC,   STAMTYPE_COUNTER, "/CSAM/HC/KnownPageRecords",   STAMUNIT_OCCURENCES,     "The number of known CSAM page records.");
    STAM_REG(pVM, &pVM->csam.s.StatNrKnownPagesGC,   STAMTYPE_COUNTER, "/CSAM/GC/KnownPageRecords",   STAMUNIT_OCCURENCES,     "The number of known CSAM page records.");
    STAM_REG(pVM, &pVM->csam.s.StatNrInstr,          STAMTYPE_COUNTER, "/CSAM/ScannedInstr",          STAMUNIT_OCCURENCES,     "The number of scanned instructions.");
    STAM_REG(pVM, &pVM->csam.s.StatNrBytesRead,      STAMTYPE_COUNTER, "/CSAM/BytesRead",             STAMUNIT_OCCURENCES,     "The number of bytes read for scanning.");
    STAM_REG(pVM, &pVM->csam.s.StatNrOpcodeRead,     STAMTYPE_COUNTER, "/CSAM/OpcodeBytesRead",       STAMUNIT_OCCURENCES,     "The number of opcode bytes read by the recompiler.");

    STAM_REG(pVM, &pVM->csam.s.StatBitmapAlloc,      STAMTYPE_COUNTER, "/CSAM/Alloc/PageBitmap",      STAMUNIT_OCCURENCES,     "The number of page bitmap allocations.");

    STAM_REG(pVM, &pVM->csam.s.StatInstrCacheHit,    STAMTYPE_COUNTER, "/CSAM/Cache/Hit",             STAMUNIT_OCCURENCES,     "The number of dangerous instruction cache hits.");
    STAM_REG(pVM, &pVM->csam.s.StatInstrCacheMiss,   STAMTYPE_COUNTER, "/CSAM/Cache/Miss",            STAMUNIT_OCCURENCES,     "The number of dangerous instruction cache misses.");

    STAM_REG(pVM, &pVM->csam.s.StatScanNextFunction,         STAMTYPE_COUNTER, "/CSAM/Function/Scan/Success",  STAMUNIT_OCCURENCES,     "The number of found functions beyond the ret border.");
    STAM_REG(pVM, &pVM->csam.s.StatScanNextFunctionFailed,   STAMTYPE_COUNTER, "/CSAM/Function/Scan/Failed",   STAMUNIT_OCCURENCES,     "The number of refused functions beyond the ret border.");

    STAM_REG(pVM, &pVM->csam.s.StatTime,             STAMTYPE_PROFILE, "/PROF/CSAM/Scan",             STAMUNIT_TICKS_PER_CALL, "Scanning overhead.");
    STAM_REG(pVM, &pVM->csam.s.StatTimeCheckAddr,    STAMTYPE_PROFILE, "/PROF/CSAM/CheckAddr",        STAMUNIT_TICKS_PER_CALL, "Address check overhead.");
    STAM_REG(pVM, &pVM->csam.s.StatTimeAddrConv,     STAMTYPE_PROFILE, "/PROF/CSAM/AddrConv",         STAMUNIT_TICKS_PER_CALL, "Address conversion overhead.");
    STAM_REG(pVM, &pVM->csam.s.StatTimeFlushPage,    STAMTYPE_PROFILE, "/PROF/CSAM/FlushPage",        STAMUNIT_TICKS_PER_CALL, "Page flushing overhead.");
    STAM_REG(pVM, &pVM->csam.s.StatTimeDisasm,       STAMTYPE_PROFILE, "/PROF/CSAM/Disasm",           STAMUNIT_TICKS_PER_CALL, "Disassembly overhead.");
    STAM_REG(pVM, &pVM->csam.s.StatFlushDirtyPages,  STAMTYPE_PROFILE, "/PROF/CSAM/FlushDirtyPage",   STAMUNIT_TICKS_PER_CALL, "Dirty page flushing overhead.");
    STAM_REG(pVM, &pVM->csam.s.StatCheckGates,       STAMTYPE_PROFILE, "/PROF/CSAM/CheckGates",       STAMUNIT_TICKS_PER_CALL, "CSAMR3CheckGates overhead.");

    /*
     * Check CFGM option and enable/disable CSAM.
     */
    bool fEnabled;
    rc = CFGMR3QueryBool(CFGMR3GetRoot(pVM), "CSAMEnabled", &fEnabled);
    if (RT_FAILURE(rc))
#ifdef CSAM_ENABLE
        fEnabled = true;
#else
        fEnabled = false;
#endif
    if (fEnabled)
        CSAMEnableScanning(pVM);

#ifdef VBOX_WITH_DEBUGGER
    /*
     * Debugger commands.
     */
    static bool fRegisteredCmds = false;
    if (!fRegisteredCmds)
    {
        rc = DBGCRegisterCommands(&g_aCmds[0], RT_ELEMENTS(g_aCmds));
        if (RT_SUCCESS(rc))
            fRegisteredCmds = true;
    }
#endif

    return VINF_SUCCESS;
}

/**
 * (Re)initializes CSAM
 *
 * @param   pVM     The VM.
 */
static int csamReinit(PVM pVM)
{
    /*
     * Assert alignment and sizes.
     */
    AssertRelease(!(RT_OFFSETOF(VM, csam.s) & 31));
    AssertRelease(sizeof(pVM->csam.s) <= sizeof(pVM->csam.padding));

    /*
     * Setup any fixed pointers and offsets.
     */
    pVM->csam.s.offVM = RT_OFFSETOF(VM, patm);

    pVM->csam.s.fGatesChecked    = false;
    pVM->csam.s.fScanningStarted = false;

    PVMCPU pVCpu = &pVM->aCpus[0];  /* raw mode implies 1 VPCU */
    VMCPU_FF_CLEAR(pVCpu, VMCPU_FF_CSAM_PENDING_ACTION);
    pVM->csam.s.cDirtyPages = 0;
    /* not necessary */
    memset(pVM->csam.s.pvDirtyBasePage, 0, sizeof(pVM->csam.s.pvDirtyBasePage));
    memset(pVM->csam.s.pvDirtyFaultPage, 0, sizeof(pVM->csam.s.pvDirtyFaultPage));

    memset(&pVM->csam.s.aDangerousInstr, 0, sizeof(pVM->csam.s.aDangerousInstr));
    pVM->csam.s.cDangerousInstr = 0;
    pVM->csam.s.iDangerousInstr  = 0;

    memset(pVM->csam.s.pvCallInstruction, 0, sizeof(pVM->csam.s.pvCallInstruction));
    pVM->csam.s.iCallInstruction = 0;

    /** @note never mess with the pgdir bitmap here! */
    return VINF_SUCCESS;
}

/**
 * Applies relocations to data and code managed by this
 * component. This function will be called at init and
 * whenever the VMM need to relocate itself inside the GC.
 *
 * The csam will update the addresses used by the switcher.
 *
 * @param   pVM      The VM.
 * @param   offDelta Relocation delta.
 */
VMMR3DECL(void) CSAMR3Relocate(PVM pVM, RTGCINTPTR offDelta)
{
    if (offDelta)
    {
        /* Adjust pgdir and page bitmap pointers. */
        pVM->csam.s.pPDBitmapGC   = MMHyperR3ToRC(pVM, pVM->csam.s.pPDGCBitmapHC);
        pVM->csam.s.pPDHCBitmapGC = MMHyperR3ToRC(pVM, pVM->csam.s.pPDBitmapHC);

        for(int i=0;i<CSAM_PGDIRBMP_CHUNKS;i++)
        {
            if (pVM->csam.s.pPDGCBitmapHC[i])
            {
                pVM->csam.s.pPDGCBitmapHC[i] += offDelta;
            }
        }
    }
    return;
}

/**
 * Terminates the csam.
 *
 * Termination means cleaning up and freeing all resources,
 * the VM it self is at this point powered off or suspended.
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 */
VMMR3DECL(int) CSAMR3Term(PVM pVM)
{
    int rc;

    rc = CSAMR3Reset(pVM);
    AssertRC(rc);

    /* @todo triggers assertion in MMHyperFree */
#if 0
    for(int i=0;i<CSAM_PAGEBMP_CHUNKS;i++)
    {
        if (pVM->csam.s.pPDBitmapHC[i])
            MMHyperFree(pVM, pVM->csam.s.pPDBitmapHC[i]);
    }
#endif

    return VINF_SUCCESS;
}

/**
 * CSAM reset callback.
 *
 * @returns VBox status code.
 * @param   pVM     The VM which is reset.
 */
VMMR3DECL(int) CSAMR3Reset(PVM pVM)
{
    /* Clear page bitmaps. */
    for(int i=0;i<CSAM_PGDIRBMP_CHUNKS;i++)
    {
        if (pVM->csam.s.pPDBitmapHC[i])
        {
            Assert((CSAM_PAGE_BITMAP_SIZE& 3) == 0);
            ASMMemZero32(pVM->csam.s.pPDBitmapHC[i], CSAM_PAGE_BITMAP_SIZE);
        }
    }

    /* Remove all CSAM page records. */
    while(true)
    {
        PCSAMPAGEREC pPageRec = (PCSAMPAGEREC)RTAvlPVGetBestFit(&pVM->csam.s.pPageTree, 0, true);
        if (pPageRec)
        {
            csamRemovePageRecord(pVM, pPageRec->page.pPageGC);
        }
        else
            break;
    }
    Assert(!pVM->csam.s.pPageTree);

    csamReinit(pVM);

    return VINF_SUCCESS;
}


/**
 * Callback function for RTAvlPVDoWithAll
 *
 * Counts the number of records in the tree
 *
 * @returns VBox status code.
 * @param   pNode           Current node
 * @param   pcPatches       Pointer to patch counter
 */
static DECLCALLBACK(int) CountRecord(PAVLPVNODECORE pNode, void *pcPatches)
{
    NOREF(pNode);
    *(uint32_t *)pcPatches = *(uint32_t *)pcPatches + 1;
    return VINF_SUCCESS;
}

/**
 * Callback function for RTAvlPVDoWithAll
 *
 * Saves the state of the page record
 *
 * @returns VBox status code.
 * @param   pNode           Current node
 * @param   pVM1            Pointer to the VM
 */
static DECLCALLBACK(int) SavePageState(PAVLPVNODECORE pNode, void *pVM1)
{
    PVM           pVM    = (PVM)pVM1;
    PCSAMPAGEREC  pPage  = (PCSAMPAGEREC)pNode;
    CSAMPAGEREC   page   = *pPage;
    PSSMHANDLE    pSSM   = pVM->csam.s.savedstate.pSSM;
    int           rc;

    /* Save the page record itself */
    rc = SSMR3PutMem(pSSM, &page, sizeof(page));
    AssertRCReturn(rc, rc);

    if (page.page.pBitmap)
    {
        rc = SSMR3PutMem(pSSM, page.page.pBitmap, CSAM_PAGE_BITMAP_SIZE);
        AssertRCReturn(rc, rc);
    }

    return VINF_SUCCESS;
}

/**
 * Execute state save operation.
 *
 * @returns VBox status code.
 * @param   pVM             Pointer to the VM.
 * @param   pSSM            SSM operation handle.
 */
static DECLCALLBACK(int) csamr3Save(PVM pVM, PSSMHANDLE pSSM)
{
    CSAM csamInfo = pVM->csam.s;
    int  rc;

    /*
     * Count the number of page records in the tree (feeling lazy)
     */
    csamInfo.savedstate.cPageRecords = 0;
    RTAvlPVDoWithAll(&pVM->csam.s.pPageTree, true, CountRecord, &csamInfo.savedstate.cPageRecords);

    /*
     * Save CSAM structure
     */
    pVM->csam.s.savedstate.pSSM = pSSM;
    rc = SSMR3PutMem(pSSM, &csamInfo, sizeof(csamInfo));
    AssertRCReturn(rc, rc);

    /* Save pgdir bitmap */
    rc = SSMR3PutMem(pSSM, csamInfo.pPDBitmapHC, CSAM_PGDIRBMP_CHUNKS*sizeof(RTHCPTR));
    AssertRCReturn(rc, rc);

    for (unsigned i=0;i<CSAM_PGDIRBMP_CHUNKS;i++)
    {
        if(csamInfo.pPDBitmapHC[i])
        {
            /* Save the page bitmap. */
            rc = SSMR3PutMem(pSSM, csamInfo.pPDBitmapHC[i], CSAM_PAGE_BITMAP_SIZE);
            AssertRCReturn(rc, rc);
        }
    }

    /*
     * Save page records
     */
    rc = RTAvlPVDoWithAll(&pVM->csam.s.pPageTree, true, SavePageState, pVM);
    AssertRCReturn(rc, rc);

    /** @note we don't restore aDangerousInstr; it will be recreated automatically. */
    return VINF_SUCCESS;
}

/**
 * Execute state load operation.
 *
 * @returns VBox status code.
 * @param   pVM             Pointer to the VM.
 * @param   pSSM            SSM operation handle.
 * @param   uVersion        Data layout version.
 * @param   uPass           The data pass.
 */
static DECLCALLBACK(int) csamr3Load(PVM pVM, PSSMHANDLE pSSM, uint32_t uVersion, uint32_t uPass)
{
    int  rc;
    CSAM csamInfo;

    Assert(uPass == SSM_PASS_FINAL); NOREF(uPass);
    if (uVersion != CSAM_SSM_VERSION)
    {
        AssertMsgFailed(("csamR3Load: Invalid version uVersion=%d!\n", uVersion));
        return VERR_SSM_UNSUPPORTED_DATA_UNIT_VERSION;
    }

    pVM->csam.s.savedstate.pSSM = pSSM;

    /*
     * Restore CSAM structure
     */
#if 0
    rc = SSMR3GetMem(pSSM, &csamInfo, sizeof(csamInfo));
#else
    RT_ZERO(csamInfo);
    rc = SSMR3GetStructEx(pSSM, &csamInfo, sizeof(csamInfo), SSMSTRUCT_FLAGS_MEM_BAND_AID, &g_aCsamFields[0], NULL);
#endif
    AssertRCReturn(rc, rc);

    pVM->csam.s.fGatesChecked    = csamInfo.fGatesChecked;
    pVM->csam.s.fScanningStarted = csamInfo.fScanningStarted;

     /* Restore dirty code page info. */
    pVM->csam.s.cDirtyPages = csamInfo.cDirtyPages;
    memcpy(pVM->csam.s.pvDirtyBasePage,  csamInfo.pvDirtyBasePage, sizeof(pVM->csam.s.pvDirtyBasePage));
    memcpy(pVM->csam.s.pvDirtyFaultPage, csamInfo.pvDirtyFaultPage, sizeof(pVM->csam.s.pvDirtyFaultPage));

     /* Restore possible code page  */
    pVM->csam.s.cPossibleCodePages = csamInfo.cPossibleCodePages;
    memcpy(pVM->csam.s.pvPossibleCodePage,  csamInfo.pvPossibleCodePage, sizeof(pVM->csam.s.pvPossibleCodePage));

    /* Restore pgdir bitmap (we'll change the pointers next). */
#if 0
    rc = SSMR3GetMem(pSSM, pVM->csam.s.pPDBitmapHC, CSAM_PGDIRBMP_CHUNKS*sizeof(RTHCPTR));
#else
    rc = SSMR3GetStructEx(pSSM, pVM->csam.s.pPDBitmapHC, sizeof(uint8_t *) * CSAM_PGDIRBMP_CHUNKS,
                          SSMSTRUCT_FLAGS_MEM_BAND_AID, &g_aCsamPDBitmapArray[0], NULL);
#endif
    AssertRCReturn(rc, rc);

    /*
     * Restore page bitmaps
     */
    for (unsigned i=0;i<CSAM_PGDIRBMP_CHUNKS;i++)
    {
        if(pVM->csam.s.pPDBitmapHC[i])
        {
            rc = MMHyperAlloc(pVM, CSAM_PAGE_BITMAP_SIZE, 0, MM_TAG_CSAM, (void **)&pVM->csam.s.pPDBitmapHC[i]);
            if (RT_FAILURE(rc))
            {
                Log(("MMHyperAlloc failed with %Rrc\n", rc));
                return rc;
            }
            /* Convert to GC pointer. */
            pVM->csam.s.pPDGCBitmapHC[i] = MMHyperR3ToRC(pVM, pVM->csam.s.pPDBitmapHC[i]);
            Assert(pVM->csam.s.pPDGCBitmapHC[i]);

            /* Restore the bitmap. */
            rc = SSMR3GetMem(pSSM, pVM->csam.s.pPDBitmapHC[i], CSAM_PAGE_BITMAP_SIZE);
            AssertRCReturn(rc, rc);
        }
        else
        {
            Assert(!pVM->csam.s.pPDGCBitmapHC[i]);
            pVM->csam.s.pPDGCBitmapHC[i] = 0;
        }
    }

    /*
     * Restore page records
     */
    for (uint32_t i=0;i<csamInfo.savedstate.cPageRecords + csamInfo.savedstate.cPatchPageRecords;i++)
    {
        CSAMPAGEREC  page;
        PCSAMPAGE    pPage;

#if 0
        rc = SSMR3GetMem(pSSM, &page, sizeof(page));
#else
        RT_ZERO(page);
        rc = SSMR3GetStructEx(pSSM, &page, sizeof(page), SSMSTRUCT_FLAGS_MEM_BAND_AID, &g_aCsamPageRecFields[0], NULL);
#endif
        AssertRCReturn(rc, rc);

        /*
         * Recreate the page record
         */
        pPage = csamCreatePageRecord(pVM, page.page.pPageGC, page.page.enmTag, page.page.fCode32, page.page.fMonitorInvalidation);
        AssertReturn(pPage, VERR_NO_MEMORY);

        pPage->GCPhys  = page.page.GCPhys;
        pPage->fFlags  = page.page.fFlags;
        pPage->u64Hash = page.page.u64Hash;

        if (page.page.pBitmap)
        {
            rc = SSMR3GetMem(pSSM, pPage->pBitmap, CSAM_PAGE_BITMAP_SIZE);
            AssertRCReturn(rc, rc);
        }
        else
        {
            MMR3HeapFree(pPage->pBitmap);
            pPage->pBitmap = 0;
        }
    }

    /* Note: we don't restore aDangerousInstr; it will be recreated automatically. */
    memset(&pVM->csam.s.aDangerousInstr, 0, sizeof(pVM->csam.s.aDangerousInstr));
    pVM->csam.s.cDangerousInstr = 0;
    pVM->csam.s.iDangerousInstr  = 0;
    return VINF_SUCCESS;
}

/**
 * Convert guest context address to host context pointer
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   pCacheRec   Address conversion cache record
 * @param   pGCPtr      Guest context pointer
 * @returns             Host context pointer or NULL in case of an error
 *
 */
static R3PTRTYPE(void *) CSAMGCVirtToHCVirt(PVM pVM, PCSAMP2GLOOKUPREC pCacheRec, RCPTRTYPE(uint8_t *) pGCPtr)
{
    int rc;
    R3PTRTYPE(void *) pHCPtr;
    Assert(pVM->cCpus == 1);
    PVMCPU pVCpu = VMMGetCpu0(pVM);

    STAM_PROFILE_START(&pVM->csam.s.StatTimeAddrConv, a);

    pHCPtr = PATMR3GCPtrToHCPtr(pVM, pGCPtr);
    if (pHCPtr)
        return pHCPtr;

    if (pCacheRec->pPageLocStartHC)
    {
        uint32_t offset = pGCPtr & PAGE_OFFSET_MASK;
        if (pCacheRec->pGuestLoc == (pGCPtr & PAGE_BASE_GC_MASK))
        {
            STAM_PROFILE_STOP(&pVM->csam.s.StatTimeAddrConv, a);
            return pCacheRec->pPageLocStartHC + offset;
        }
    }

    /* Release previous lock if any. */
    if (pCacheRec->Lock.pvMap)
    {
        PGMPhysReleasePageMappingLock(pVM, &pCacheRec->Lock);
        pCacheRec->Lock.pvMap = NULL;
    }

    rc = PGMPhysGCPtr2CCPtrReadOnly(pVCpu, pGCPtr, (const void **)&pHCPtr, &pCacheRec->Lock);
    if (rc != VINF_SUCCESS)
    {
////        AssertMsgRC(rc, ("MMR3PhysGCVirt2HCVirtEx failed for %RRv\n", pGCPtr));
        STAM_PROFILE_STOP(&pVM->csam.s.StatTimeAddrConv, a);
        return NULL;
    }

    pCacheRec->pPageLocStartHC = (R3PTRTYPE(uint8_t*))((RTHCUINTPTR)pHCPtr & PAGE_BASE_HC_MASK);
    pCacheRec->pGuestLoc       = pGCPtr & PAGE_BASE_GC_MASK;
    STAM_PROFILE_STOP(&pVM->csam.s.StatTimeAddrConv, a);
    return pHCPtr;
}


/** For csamR3ReadBytes. */
typedef struct CSAMDISINFO
{
    PVM             pVM;
    uint8_t const  *pbSrcInstr; /* aka pInstHC */
} CSAMDISINFO, *PCSAMDISINFO;


/**
 * @callback_method_impl{FNDISREADBYTES}
 */
static DECLCALLBACK(int) csamR3ReadBytes(PDISCPUSTATE pDis, uint8_t offInstr, uint8_t cbMinRead, uint8_t cbMaxRead)
{
    PCSAMDISINFO pDisInfo = (PCSAMDISINFO)pDis->pvUser;

    /*
     * We are not interested in patched instructions, so read the original opcode bytes.
     *
     * Note! single instruction patches (int3) are checked in CSAMR3AnalyseCallback
     *
     * Since we're decoding one instruction at the time, we don't need to be
     * concerned about any patched instructions following the first one.  We
     * could in fact probably skip this PATM call for offInstr != 0.
     */
    size_t      cbRead   = cbMaxRead;
    RTUINTPTR   uSrcAddr = pDis->uInstrAddr + offInstr;
    int rc = PATMR3ReadOrgInstr(pDisInfo->pVM, pDis->uInstrAddr + offInstr, &pDis->abInstr[offInstr], cbRead, &cbRead);
    if (RT_SUCCESS(rc))
    {
        if (cbRead >= cbMinRead)
        {
            pDis->cbCachedInstr = offInstr + (uint8_t)cbRead;
            return rc;
        }

        cbMinRead -= (uint8_t)cbRead;
        cbMaxRead -= (uint8_t)cbRead;
        offInstr  += (uint8_t)cbRead;
        uSrcAddr  += cbRead;
    }

    /*
     * The current byte isn't a patch instruction byte.
     */
    AssertPtr(pDisInfo->pbSrcInstr);
    if ((pDis->uInstrAddr >> PAGE_SHIFT) == ((uSrcAddr + cbMaxRead - 1) >> PAGE_SHIFT))
    {
        memcpy(&pDis->abInstr[offInstr], &pDisInfo->pbSrcInstr[offInstr], cbMaxRead);
        offInstr += cbMaxRead;
        rc = VINF_SUCCESS;
    }
    else if (   (pDis->uInstrAddr >> PAGE_SHIFT) == ((uSrcAddr + cbMinRead - 1) >> PAGE_SHIFT)
             || PATMIsPatchGCAddr(pDisInfo->pVM, uSrcAddr) /** @todo does CSAM actually analyze patch code, or is this just a copy&past check? */
            )
    {
        memcpy(&pDis->abInstr[offInstr], &pDisInfo->pbSrcInstr[offInstr], cbMinRead);
        offInstr += cbMinRead;
        rc = VINF_SUCCESS;
    }
    else
    {
        /* Crossed page boundrary, pbSrcInstr is no good... */
        rc = PGMPhysSimpleReadGCPtr(VMMGetCpu0(pDisInfo->pVM), &pDis->abInstr[offInstr], uSrcAddr, cbMinRead);
        offInstr += cbMinRead;
    }

    pDis->cbCachedInstr = offInstr;
    return rc;
}

DECLINLINE(int) csamR3DISInstr(PVM pVM, RTRCPTR InstrGC, uint8_t *InstrHC, DISCPUMODE enmCpuMode,
                               PDISCPUSTATE pCpu, uint32_t *pcbInstr, char *pszOutput, size_t cbOutput)
{
    CSAMDISINFO DisInfo = { pVM, InstrHC };
#ifdef DEBUG
    return DISInstrToStrEx(InstrGC, enmCpuMode, csamR3ReadBytes, &DisInfo, DISOPTYPE_ALL,
                           pCpu, pcbInstr, pszOutput, cbOutput);
#else
    /* We are interested in everything except harmless stuff */
    if (pszOutput)
        return DISInstrToStrEx(InstrGC, enmCpuMode, csamR3ReadBytes, &DisInfo,
                               ~(DISOPTYPE_INVALID | DISOPTYPE_HARMLESS | DISOPTYPE_RRM_MASK),
                               pCpu, pcbInstr, pszOutput, cbOutput);
    return DISInstrEx(InstrGC, enmCpuMode, ~(DISOPTYPE_INVALID | DISOPTYPE_HARMLESS | DISOPTYPE_RRM_MASK),
                      csamR3ReadBytes, &DisInfo, pCpu, pcbInstr);
#endif
}

/**
 * Analyses the instructions following the cli for compliance with our heuristics for cli
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   pCpu        CPU disassembly state
 * @param   pInstrGC    Guest context pointer to privileged instruction
 * @param   pCurInstrGC Guest context pointer to the current instruction
 * @param   pCacheRec   GC to HC cache record
 * @param   pUserData   User pointer (callback specific)
 *
 */
static int CSAMR3AnalyseCallback(PVM pVM, DISCPUSTATE *pCpu, RCPTRTYPE(uint8_t *) pInstrGC, RCPTRTYPE(uint8_t *) pCurInstrGC,
                                 PCSAMP2GLOOKUPREC pCacheRec, void *pUserData)
{
    PCSAMPAGE pPage = (PCSAMPAGE)pUserData;
    int rc;
    NOREF(pInstrGC);

    switch (pCpu->pCurInstr->uOpcode)
    {
    case OP_INT:
        Assert(pCpu->Param1.fUse & DISUSE_IMMEDIATE8);
        if (pCpu->Param1.uValue == 3)
        {
            //two byte int 3
            return VINF_SUCCESS;
        }
        break;

    /* removing breaks win2k guests? */
    case OP_IRET:
        if (EMIsRawRing1Enabled(pVM))
            break;
        /* no break */

    case OP_ILLUD2:
        /* This appears to be some kind of kernel panic in Linux 2.4; no point to continue. */
    case OP_RETN:
    case OP_INT3:
    case OP_INVALID:
        return VINF_SUCCESS;
    }

    // Check for exit points
    switch (pCpu->pCurInstr->uOpcode)
    {
    /* It's not a good idea to patch pushf instructions:
     * - increases the chance of conflicts (code jumping to the next instruction)
     * - better to patch the cli
     * - code that branches before the cli will likely hit an int 3
     * - in general doesn't offer any benefits as we don't allow nested patch blocks (IF is always 1)
     */
    case OP_PUSHF:
    case OP_POPF:
        break;

    case OP_CLI:
    {
        uint32_t     cbInstrs   = 0;
        uint32_t     cbCurInstr = pCpu->cbInstr;
        bool         fCode32    = pPage->fCode32;

        Assert(fCode32);

        PATMR3AddHint(pVM, pCurInstrGC, (fCode32) ? PATMFL_CODE32 : 0);

        /* Make sure the instructions that follow the cli have not been encountered before. */
        while (true)
        {
            DISCPUSTATE  cpu;

            if (cbInstrs + cbCurInstr >= SIZEOF_NEARJUMP32)
                break;

            if (csamIsCodeScanned(pVM, pCurInstrGC + cbCurInstr, &pPage) == true)
            {
                /* We've scanned the next instruction(s) already. This means we've
                   followed a branch that ended up there before -> dangerous!! */
                PATMR3DetectConflict(pVM, pCurInstrGC, pCurInstrGC + cbCurInstr);
                break;
            }
            pCurInstrGC += cbCurInstr;
            cbInstrs    += cbCurInstr;

            {   /* Force pCurInstrHC out of scope after we stop using it (page lock!) */
                uint8_t       *pCurInstrHC = 0;
                pCurInstrHC = (uint8_t *)CSAMGCVirtToHCVirt(pVM, pCacheRec, pCurInstrGC);
                if (pCurInstrHC == NULL)
                {
                    Log(("CSAMGCVirtToHCVirt failed for %RRv\n", pCurInstrGC));
                    break;
                }
                Assert(VALID_PTR(pCurInstrHC));

                rc = csamR3DISInstr(pVM, pCurInstrGC, pCurInstrHC, (fCode32) ? DISCPUMODE_32BIT : DISCPUMODE_16BIT,
                                    &cpu, &cbCurInstr, NULL, 0);
            }
            AssertRC(rc);
            if (RT_FAILURE(rc))
                break;
        }
        break;
    }

#ifdef VBOX_WITH_RAW_RING1
    case OP_MOV:
        /* mov xx, CS is a dangerous instruction as our raw ring usage leaks through. */
        if (    EMIsRawRing1Enabled(pVM)
            &&  (pCpu->Param2.fUse & DISUSE_REG_SEG) 
            &&  (pCpu->Param2.Base.idxSegReg == DISSELREG_CS))
        {
            Log(("CSAM: Patching dangerous 'mov xx, cs' instruction at %RGv with an int3\n", pCurInstrGC));
            if (PATMR3HasBeenPatched(pVM, pCurInstrGC) == false)
            {
                rc = PATMR3InstallPatch(pVM, pCurInstrGC, (pPage->fCode32) ? PATMFL_CODE32 : 0);
                if (RT_FAILURE(rc))
                {
                    Log(("PATMR3InstallPatch failed with %d\n", rc));
                    return VWRN_CONTINUE_ANALYSIS;
                }
            }
            return VWRN_CONTINUE_ANALYSIS;
        }
        break;
#endif

    case OP_PUSH:
        /** @todo broken comparison!! should be if ((pCpu->Param1.fUse & DISUSE_REG_SEG) &&  (pCpu->Param1.Base.idxSegReg == DISSELREG_SS)) */
        if (pCpu->pCurInstr->fParam1 != OP_PARM_REG_CS)
            break;

        /* no break */
#ifndef VBOX_WITH_SAFE_STR
    case OP_STR:
#endif
    case OP_LSL:
    case OP_LAR:
    case OP_SGDT:
    case OP_SLDT:
    case OP_SIDT:
    case OP_SMSW:
    case OP_VERW:
    case OP_VERR:
    case OP_CPUID:
    case OP_IRET:
#ifdef DEBUG
        switch(pCpu->pCurInstr->uOpcode)
        {
        case OP_STR:
            Log(("Privileged instruction at %RRv: str!!\n", pCurInstrGC));
            break;
        case OP_LSL:
            Log(("Privileged instruction at %RRv: lsl!!\n", pCurInstrGC));
            break;
        case OP_LAR:
            Log(("Privileged instruction at %RRv: lar!!\n", pCurInstrGC));
            break;
        case OP_SGDT:
            Log(("Privileged instruction at %RRv: sgdt!!\n", pCurInstrGC));
            break;
        case OP_SLDT:
            Log(("Privileged instruction at %RRv: sldt!!\n", pCurInstrGC));
            break;
        case OP_SIDT:
            Log(("Privileged instruction at %RRv: sidt!!\n", pCurInstrGC));
            break;
        case OP_SMSW:
            Log(("Privileged instruction at %RRv: smsw!!\n", pCurInstrGC));
            break;
        case OP_VERW:
            Log(("Privileged instruction at %RRv: verw!!\n", pCurInstrGC));
            break;
        case OP_VERR:
            Log(("Privileged instruction at %RRv: verr!!\n", pCurInstrGC));
            break;
        case OP_CPUID:
            Log(("Privileged instruction at %RRv: cpuid!!\n", pCurInstrGC));
            break;
        case OP_PUSH:
            Log(("Privileged instruction at %RRv: push cs!!\n", pCurInstrGC));
            break;
        case OP_IRET:
            Log(("Privileged instruction at %RRv: iret!!\n", pCurInstrGC));
            break;
        }
#endif

        if (PATMR3HasBeenPatched(pVM, pCurInstrGC) == false)
        {
            rc = PATMR3InstallPatch(pVM, pCurInstrGC, (pPage->fCode32) ? PATMFL_CODE32 : 0);
            if (RT_FAILURE(rc))
            {
                Log(("PATMR3InstallPatch failed with %d\n", rc));
                return VWRN_CONTINUE_ANALYSIS;
            }
        }
        if (pCpu->pCurInstr->uOpcode == OP_IRET)
            return VINF_SUCCESS;    /* Look no further in this branch. */

        return VWRN_CONTINUE_ANALYSIS;

    case OP_JMP:
    case OP_CALL:
    {
        // return or jump/call through a jump table
        if (OP_PARM_VTYPE(pCpu->pCurInstr->fParam1) != OP_PARM_J)
        {
#ifdef DEBUG
            switch(pCpu->pCurInstr->uOpcode)
            {
            case OP_JMP:
                Log(("Control Flow instruction at %RRv: jmp!!\n", pCurInstrGC));
                break;
            case OP_CALL:
                Log(("Control Flow instruction at %RRv: call!!\n", pCurInstrGC));
                break;
            }
#endif
            return VWRN_CONTINUE_ANALYSIS;
        }
        return VWRN_CONTINUE_ANALYSIS;
    }

    }

    return VWRN_CONTINUE_ANALYSIS;
}

#ifdef CSAM_ANALYSE_BEYOND_RET
/**
 * Wrapper for csamAnalyseCodeStream for call instructions.
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   pInstrGC    Guest context pointer to privileged instruction
 * @param   pCurInstrGC Guest context pointer to the current instruction
 * @param   fCode32     16 or 32 bits code
 * @param   pfnCSAMR3Analyse Callback for testing the disassembled instruction
 * @param   pUserData   User pointer (callback specific)
 *
 */
static int csamAnalyseCallCodeStream(PVM pVM, RCPTRTYPE(uint8_t *) pInstrGC, RCPTRTYPE(uint8_t *) pCurInstrGC, bool fCode32,
                                     PFN_CSAMR3ANALYSE pfnCSAMR3Analyse, void *pUserData, PCSAMP2GLOOKUPREC pCacheRec)
{
    int                 rc;
    CSAMCALLEXITREC     CallExitRec;
    PCSAMCALLEXITREC    pOldCallRec;
    PCSAMPAGE           pPage = 0;
    uint32_t            i;

    CallExitRec.cInstrAfterRet = 0;

    pOldCallRec = pCacheRec->pCallExitRec;
    pCacheRec->pCallExitRec = &CallExitRec;

    rc = csamAnalyseCodeStream(pVM, pInstrGC, pCurInstrGC, fCode32, pfnCSAMR3Analyse, pUserData, pCacheRec);

    for (i=0;i<CallExitRec.cInstrAfterRet;i++)
    {
        PCSAMPAGE pPage = 0;

        pCurInstrGC = CallExitRec.pInstrAfterRetGC[i];

        /* Check if we've previously encountered the instruction after the ret. */
        if (csamIsCodeScanned(pVM, pCurInstrGC, &pPage) == false)
        {
            DISCPUSTATE  cpu;
            uint32_t     cbInstr;
            int          rc2;
#ifdef DEBUG
            char szOutput[256];
#endif
            if (pPage == NULL)
            {
                /* New address; let's take a look at it. */
                pPage = csamCreatePageRecord(pVM, pCurInstrGC, CSAM_TAG_CSAM, fCode32);
                if (pPage == NULL)
                {
                    rc = VERR_NO_MEMORY;
                    goto done;
                }
            }

            /**
             * Some generic requirements for recognizing an adjacent function:
             * - alignment fillers that consist of:
             *   - nop
             *   - lea genregX, [genregX (+ 0)]
             * - push ebp after the filler (can extend this later); aligned at at least a 4 byte boundary
             */
            for (int j = 0; j < 16; j++)
            {
                uint8_t *pCurInstrHC = (uint8_t *)CSAMGCVirtToHCVirt(pVM, pCacheRec, pCurInstrGC);
                if (pCurInstrHC == NULL)
                {
                    Log(("CSAMGCVirtToHCVirt failed for %RRv\n", pCurInstrGC));
                    goto done;
                }
                Assert(VALID_PTR(pCurInstrHC));

                STAM_PROFILE_START(&pVM->csam.s.StatTimeDisasm, a);
#ifdef DEBUG
                rc2 = csamR3DISInstr(pVM, pCurInstrGC, pCurInstrHC, (fCode32) ? DISCPUMODE_32BIT : DISCPUMODE_16BIT,
                                     &cpu, &cbInstr, szOutput, sizeof(szOutput));
                if (RT_SUCCESS(rc2)) Log(("CSAM Call Analysis: %s", szOutput));
#else
                rc2 = csamR3DISInstr(pVM, pCurInstrGC, pCurInstrHC, (fCode32) ? DISCPUMODE_32BIT : DISCPUMODE_16BIT,
                                     &cpu, &cbInstr, NULL, 0);
#endif
                STAM_PROFILE_STOP(&pVM->csam.s.StatTimeDisasm, a);
                if (RT_FAILURE(rc2))
                {
                    Log(("Disassembly failed at %RRv with %Rrc (probably page not present) -> return to caller\n", pCurInstrGC, rc2));
                    goto done;
                }

                STAM_COUNTER_ADD(&pVM->csam.s.StatNrBytesRead, cbInstr);

                RCPTRTYPE(uint8_t *) addr = 0;
                PCSAMPAGE pJmpPage = NULL;

                if (PAGE_ADDRESS(pCurInstrGC) != PAGE_ADDRESS(pCurInstrGC + cbInstr - 1))
                {
                    if (!PGMGstIsPagePresent(pVM, pCurInstrGC + cbInstr - 1))
                    {
                        /// @todo fault in the page
                        Log(("Page for current instruction %RRv is not present!!\n", pCurInstrGC));
                        goto done;
                    }
                    //all is fine, let's continue
                    csamR3CheckPageRecord(pVM, pCurInstrGC + cbInstr - 1);
                }

                switch (cpu.pCurInstr->uOpcode)
                {
                case OP_NOP:
                case OP_INT3:
                    break;  /* acceptable */

                case OP_LEA:
                    /* Must be similar to:
                     *
                     * lea esi, [esi]
                     * lea esi, [esi+0]
                     * Any register is allowed as long as source and destination are identical.
                     */
                    if (    cpu.Param1.fUse != DISUSE_REG_GEN32
                        ||  (   cpu.Param2.flags != DISUSE_REG_GEN32
                             && (   !(cpu.Param2.flags & DISUSE_REG_GEN32)
                                 || !(cpu.Param2.flags & (DISUSE_DISPLACEMENT8|DISUSE_DISPLACEMENT16|DISUSE_DISPLACEMENT32))
                                 ||  cpu.Param2.uValue != 0
                                )
                            )
                        ||  cpu.Param1.base.reg_gen32 != cpu.Param2.base.reg_gen32
                       )
                    {
                       STAM_COUNTER_INC(&pVM->csam.s.StatScanNextFunctionFailed);
                       goto next_function;
                    }
                    break;

                case OP_PUSH:
                {
                    if (    (pCurInstrGC & 0x3) != 0
                        ||  cpu.Param1.fUse != DISUSE_REG_GEN32
                        ||  cpu.Param1.base.reg_gen32 != USE_REG_EBP
                       )
                    {
                       STAM_COUNTER_INC(&pVM->csam.s.StatScanNextFunctionFailed);
                       goto next_function;
                    }

                    if (csamIsCodeScanned(pVM, pCurInstrGC, &pPage) == false)
                    {
                        CSAMCALLEXITREC CallExitRec2;
                        CallExitRec2.cInstrAfterRet = 0;

                        pCacheRec->pCallExitRec = &CallExitRec2;

                        /* Analyse the function. */
                        Log(("Found new function at %RRv\n", pCurInstrGC));
                        STAM_COUNTER_INC(&pVM->csam.s.StatScanNextFunction);
                        csamAnalyseCallCodeStream(pVM, pInstrGC, pCurInstrGC, fCode32, pfnCSAMR3Analyse, pUserData, pCacheRec);
                    }
                    goto next_function;
                }

                case OP_SUB:
                {
                    if (    (pCurInstrGC & 0x3) != 0
                        ||  cpu.Param1.fUse != DISUSE_REG_GEN32
                        ||  cpu.Param1.base.reg_gen32 != USE_REG_ESP
                       )
                    {
                       STAM_COUNTER_INC(&pVM->csam.s.StatScanNextFunctionFailed);
                       goto next_function;
                    }

                    if (csamIsCodeScanned(pVM, pCurInstrGC, &pPage) == false)
                    {
                        CSAMCALLEXITREC CallExitRec2;
                        CallExitRec2.cInstrAfterRet = 0;

                        pCacheRec->pCallExitRec = &CallExitRec2;

                        /* Analyse the function. */
                        Log(("Found new function at %RRv\n", pCurInstrGC));
                        STAM_COUNTER_INC(&pVM->csam.s.StatScanNextFunction);
                        csamAnalyseCallCodeStream(pVM, pInstrGC, pCurInstrGC, fCode32, pfnCSAMR3Analyse, pUserData, pCacheRec);
                    }
                    goto next_function;
                }

                default:
                   STAM_COUNTER_INC(&pVM->csam.s.StatScanNextFunctionFailed);
                   goto next_function;
                }
                /* Mark it as scanned. */
                csamMarkCode(pVM, pPage, pCurInstrGC, cbInstr, true);
                pCurInstrGC += cbInstr;
            } /* for at most 16 instructions */
next_function:
            ; /* MSVC complains otherwise */
        }
    }
done:
    pCacheRec->pCallExitRec = pOldCallRec;
    return rc;
}
#else
#define csamAnalyseCallCodeStream       csamAnalyseCodeStream
#endif

/**
 * Disassembles the code stream until the callback function detects a failure or decides everything is acceptable
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   pInstrGC    Guest context pointer to privileged instruction
 * @param   pCurInstrGC Guest context pointer to the current instruction
 * @param   fCode32     16 or 32 bits code
 * @param   pfnCSAMR3Analyse Callback for testing the disassembled instruction
 * @param   pUserData   User pointer (callback specific)
 *
 */
static int csamAnalyseCodeStream(PVM pVM, RCPTRTYPE(uint8_t *) pInstrGC, RCPTRTYPE(uint8_t *) pCurInstrGC, bool fCode32,
                                   PFN_CSAMR3ANALYSE pfnCSAMR3Analyse, void *pUserData, PCSAMP2GLOOKUPREC pCacheRec)
{
    DISCPUSTATE cpu;
    PCSAMPAGE pPage = (PCSAMPAGE)pUserData;
    int rc = VWRN_CONTINUE_ANALYSIS;
    uint32_t cbInstr;
    int rc2;
    Assert(pVM->cCpus == 1);
    PVMCPU pVCpu = VMMGetCpu0(pVM);

#ifdef DEBUG
    char szOutput[256];
#endif

    LogFlow(("csamAnalyseCodeStream: code at %RRv depth=%d\n", pCurInstrGC, pCacheRec->depth));

    pVM->csam.s.fScanningStarted = true;

    pCacheRec->depth++;
    /*
     * Limit the call depth. (rather arbitrary upper limit; too low and we won't detect certain
     * cpuid instructions in Linux kernels; too high and we waste too much time scanning code)
     * (512 is necessary to detect cpuid instructions in Red Hat EL4; see defect 1355)
     * @note we are using a lot of stack here. couple of 100k when we go to the full depth (!)
     */
    if (pCacheRec->depth > 512)
    {
        LogFlow(("CSAM: maximum calldepth reached for %RRv\n", pCurInstrGC));
        pCacheRec->depth--;
        return VINF_SUCCESS;    //let's not go on forever
    }

    Assert(!PATMIsPatchGCAddr(pVM, pCurInstrGC));
    csamR3CheckPageRecord(pVM, pCurInstrGC);

    while(rc == VWRN_CONTINUE_ANALYSIS)
    {
        if (csamIsCodeScanned(pVM, pCurInstrGC, &pPage) == false)
        {
            if (pPage == NULL)
            {
                /* New address; let's take a look at it. */
                pPage = csamCreatePageRecord(pVM, pCurInstrGC, CSAM_TAG_CSAM, fCode32);
                if (pPage == NULL)
                {
                    rc = VERR_NO_MEMORY;
                    goto done;
                }
            }
        }
        else
        {
            LogFlow(("Code at %RRv has been scanned before\n", pCurInstrGC));
            rc = VINF_SUCCESS;
            goto done;
        }

        {   /* Force pCurInstrHC out of scope after we stop using it (page lock!) */
            uint8_t *pCurInstrHC = (uint8_t *)CSAMGCVirtToHCVirt(pVM, pCacheRec, pCurInstrGC);
            if (pCurInstrHC == NULL)
            {
                Log(("CSAMGCVirtToHCVirt failed for %RRv\n", pCurInstrGC));
                rc = VERR_PATCHING_REFUSED;
                goto done;
            }
            Assert(VALID_PTR(pCurInstrHC));

            STAM_PROFILE_START(&pVM->csam.s.StatTimeDisasm, a);
#ifdef DEBUG
            rc2 = csamR3DISInstr(pVM, pCurInstrGC, pCurInstrHC, fCode32 ? DISCPUMODE_32BIT : DISCPUMODE_16BIT,
                                 &cpu, &cbInstr, szOutput, sizeof(szOutput));
            if (RT_SUCCESS(rc2)) Log(("CSAM Analysis: %s", szOutput));
#else
            rc2 = csamR3DISInstr(pVM, pCurInstrGC, pCurInstrHC, fCode32 ? DISCPUMODE_32BIT : DISCPUMODE_16BIT,
                                 &cpu, &cbInstr, NULL, 0);
#endif
            STAM_PROFILE_STOP(&pVM->csam.s.StatTimeDisasm, a);
        }
        if (RT_FAILURE(rc2))
        {
            Log(("Disassembly failed at %RRv with %Rrc (probably page not present) -> return to caller\n", pCurInstrGC, rc2));
            rc = VINF_SUCCESS;
            goto done;
        }

        STAM_COUNTER_ADD(&pVM->csam.s.StatNrBytesRead, cbInstr);

        csamMarkCode(pVM, pPage, pCurInstrGC, cbInstr, true);

        RCPTRTYPE(uint8_t *) addr = 0;
        PCSAMPAGE pJmpPage = NULL;

        if (PAGE_ADDRESS(pCurInstrGC) != PAGE_ADDRESS(pCurInstrGC + cbInstr - 1))
        {
            if (!PGMGstIsPagePresent(pVCpu, pCurInstrGC + cbInstr - 1))
            {
                /// @todo fault in the page
                Log(("Page for current instruction %RRv is not present!!\n", pCurInstrGC));
                rc = VWRN_CONTINUE_ANALYSIS;
                goto next_please;
            }
            //all is fine, let's continue
            csamR3CheckPageRecord(pVM, pCurInstrGC + cbInstr - 1);
        }
        /*
         * If it's harmless, then don't bother checking it (the disasm tables had better be accurate!)
         */
        if ((cpu.pCurInstr->fOpType & ~DISOPTYPE_RRM_MASK) == DISOPTYPE_HARMLESS)
        {
            AssertMsg(pfnCSAMR3Analyse(pVM, &cpu, pInstrGC, pCurInstrGC, pCacheRec, (void *)pPage) == VWRN_CONTINUE_ANALYSIS, ("Instruction incorrectly marked harmless?!?!?\n"));
            rc = VWRN_CONTINUE_ANALYSIS;
            goto next_please;
        }

#ifdef CSAM_ANALYSE_BEYOND_RET
        /* Remember the address of the instruction following the ret in case the parent instruction was a call. */
        if (    pCacheRec->pCallExitRec
            &&  cpu.pCurInstr->uOpcode == OP_RETN
            &&  pCacheRec->pCallExitRec->cInstrAfterRet < CSAM_MAX_CALLEXIT_RET)
        {
            pCacheRec->pCallExitRec->pInstrAfterRetGC[pCacheRec->pCallExitRec->cInstrAfterRet] = pCurInstrGC + cbInstr;
            pCacheRec->pCallExitRec->cInstrAfterRet++;
        }
#endif

        rc = pfnCSAMR3Analyse(pVM, &cpu, pInstrGC, pCurInstrGC, pCacheRec, (void *)pPage);
        if (rc == VINF_SUCCESS)
            goto done;

        // For our first attempt, we'll handle only simple relative jumps and calls (immediate offset coded in instruction)
        if (    ((cpu.pCurInstr->fOpType & DISOPTYPE_CONTROLFLOW) && (OP_PARM_VTYPE(cpu.pCurInstr->fParam1) == OP_PARM_J))
            ||  (cpu.pCurInstr->uOpcode == OP_CALL && cpu.Param1.fUse == DISUSE_DISPLACEMENT32))  /* simple indirect call (call dword ptr [address]) */
        {
            /* We need to parse 'call dword ptr [address]' type of calls to catch cpuid instructions in some recent Linux distributions (e.g. OpenSuse 10.3) */
            if (    cpu.pCurInstr->uOpcode == OP_CALL
                &&  cpu.Param1.fUse == DISUSE_DISPLACEMENT32)
            {
                addr = 0;
                PGMPhysSimpleReadGCPtr(pVCpu, &addr, (RTRCUINTPTR)cpu.Param1.uDisp.i32, sizeof(addr));
            }
            else
                addr = CSAMResolveBranch(&cpu, pCurInstrGC);

            if (addr == 0)
            {
                Log(("We don't support far jumps here!! (%08X)\n", cpu.Param1.fUse));
                rc = VINF_SUCCESS;
                break;
            }
            Assert(!PATMIsPatchGCAddr(pVM, addr));

            /* If the target address lies in a patch generated jump, then special action needs to be taken. */
            PATMR3DetectConflict(pVM, pCurInstrGC, addr);

            /* Same page? */
            if (PAGE_ADDRESS(addr) != PAGE_ADDRESS(pCurInstrGC ))
            {
                if (!PGMGstIsPagePresent(pVCpu, addr))
                {
                    Log(("Page for current instruction %RRv is not present!!\n", addr));
                    rc = VWRN_CONTINUE_ANALYSIS;
                    goto next_please;
                }

                /* All is fine, let's continue. */
                csamR3CheckPageRecord(pVM, addr);
            }

            pJmpPage = NULL;
            if (csamIsCodeScanned(pVM, addr, &pJmpPage) == false)
            {
                if (pJmpPage == NULL)
                {
                    /* New branch target; let's take a look at it. */
                    pJmpPage = csamCreatePageRecord(pVM, addr, CSAM_TAG_CSAM, fCode32);
                    if (pJmpPage == NULL)
                    {
                        rc = VERR_NO_MEMORY;
                        goto done;
                    }
                    Assert(pPage);
                }
                if (cpu.pCurInstr->uOpcode == OP_CALL)
                    rc = csamAnalyseCallCodeStream(pVM, pInstrGC, addr, fCode32, pfnCSAMR3Analyse, (void *)pJmpPage, pCacheRec);
                else
                    rc = csamAnalyseCodeStream(pVM, pInstrGC, addr, fCode32, pfnCSAMR3Analyse, (void *)pJmpPage, pCacheRec);

                if (rc != VINF_SUCCESS) {
                    goto done;
                }
            }
            if (cpu.pCurInstr->uOpcode == OP_JMP)
            {//unconditional jump; return to caller
                rc = VINF_SUCCESS;
                goto done;
            }

            rc = VWRN_CONTINUE_ANALYSIS;
        } //if ((cpu.pCurInstr->fOpType & DISOPTYPE_CONTROLFLOW) && (OP_PARM_VTYPE(cpu.pCurInstr->fParam1) == OP_PARM_J))
#ifdef CSAM_SCAN_JUMP_TABLE
        else
        if (    cpu.pCurInstr->uOpcode == OP_JMP
            &&  (cpu.Param1.fUse & (DISUSE_DISPLACEMENT32|DISUSE_INDEX|DISUSE_SCALE)) == (DISUSE_DISPLACEMENT32|DISUSE_INDEX|DISUSE_SCALE)
           )
        {
            RTRCPTR  pJumpTableGC = (RTRCPTR)cpu.Param1.disp32;
            uint8_t *pJumpTableHC;
            int      rc2;

            Log(("Jump through jump table\n"));

            rc2 = PGMPhysGCPtr2CCPtrReadOnly(pVCpu, pJumpTableGC, (PRTHCPTR)&pJumpTableHC, missing page lock);
            if (rc2 == VINF_SUCCESS)
            {
                for (uint32_t i=0;i<2;i++)
                {
                    uint64_t fFlags;

                    addr = pJumpTableGC + cpu.Param1.scale * i;
                    /* Same page? */
                    if (PAGE_ADDRESS(addr) != PAGE_ADDRESS(pJumpTableGC))
                        break;

                    addr = *(RTRCPTR *)(pJumpTableHC + cpu.Param1.scale * i);

                    rc2 = PGMGstGetPage(pVCpu, addr, &fFlags, NULL);
                    if (    rc2 != VINF_SUCCESS
                        ||  (fFlags & X86_PTE_US)
                        || !(fFlags & X86_PTE_P)
                       )
                       break;

                    Log(("Jump to %RRv\n", addr));

                    pJmpPage = NULL;
                    if (csamIsCodeScanned(pVM, addr, &pJmpPage) == false)
                    {
                        if (pJmpPage == NULL)
                        {
                            /* New branch target; let's take a look at it. */
                            pJmpPage = csamCreatePageRecord(pVM, addr, CSAM_TAG_CSAM, fCode32);
                            if (pJmpPage == NULL)
                            {
                                rc = VERR_NO_MEMORY;
                                goto done;
                            }
                            Assert(pPage);
                        }
                        rc = csamAnalyseCodeStream(pVM, pInstrGC, addr, fCode32, pfnCSAMR3Analyse, (void *)pJmpPage, pCacheRec);
                        if (rc != VINF_SUCCESS) {
                            goto done;
                        }
                    }
                }
            }
        }
#endif
        if (rc != VWRN_CONTINUE_ANALYSIS) {
            break; //done!
        }
next_please:
        if (cpu.pCurInstr->uOpcode == OP_JMP)
        {
            rc = VINF_SUCCESS;
            goto done;
        }
        pCurInstrGC += cbInstr;
    }
done:
    pCacheRec->depth--;
    return rc;
}


/**
 * Calculates the 64 bits hash value for the current page
 *
 * @returns hash value
 * @param   pVM         Pointer to the VM.
 * @param   pInstr      Page address
 */
uint64_t csamR3CalcPageHash(PVM pVM, RTRCPTR pInstr)
{
    uint64_t hash   = 0;
    uint32_t val[5];
    int      rc;
    Assert(pVM->cCpus == 1);
    PVMCPU pVCpu = VMMGetCpu0(pVM);

    Assert((pInstr & PAGE_OFFSET_MASK) == 0);

    rc = PGMPhysSimpleReadGCPtr(pVCpu, &val[0], pInstr, sizeof(val[0]));
    AssertMsg(RT_SUCCESS(rc) || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT, ("rc = %Rrc\n", rc));
    if (rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT)
    {
        Log(("csamR3CalcPageHash: page %RRv not present!!\n", pInstr));
        return ~0ULL;
    }

    rc = PGMPhysSimpleReadGCPtr(pVCpu, &val[1], pInstr+1024, sizeof(val[0]));
    AssertMsg(RT_SUCCESS(rc) || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT, ("rc = %Rrc\n", rc));
    if (rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT)
    {
        Log(("csamR3CalcPageHash: page %RRv not present!!\n", pInstr));
        return ~0ULL;
    }

    rc = PGMPhysSimpleReadGCPtr(pVCpu, &val[2], pInstr+2048, sizeof(val[0]));
    AssertMsg(RT_SUCCESS(rc) || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT, ("rc = %Rrc\n", rc));
    if (rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT)
    {
        Log(("csamR3CalcPageHash: page %RRv not present!!\n", pInstr));
        return ~0ULL;
    }

    rc = PGMPhysSimpleReadGCPtr(pVCpu, &val[3], pInstr+3072, sizeof(val[0]));
    AssertMsg(RT_SUCCESS(rc) || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT, ("rc = %Rrc\n", rc));
    if (rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT)
    {
        Log(("csamR3CalcPageHash: page %RRv not present!!\n", pInstr));
        return ~0ULL;
    }

    rc = PGMPhysSimpleReadGCPtr(pVCpu, &val[4], pInstr+4092, sizeof(val[0]));
    AssertMsg(RT_SUCCESS(rc) || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT, ("rc = %Rrc\n", rc));
    if (rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT)
    {
        Log(("csamR3CalcPageHash: page %RRv not present!!\n", pInstr));
        return ~0ULL;
    }

    // don't want to get division by zero traps
    val[2] |= 1;
    val[4] |= 1;

    hash = (uint64_t)val[0] * (uint64_t)val[1] / (uint64_t)val[2] + (val[3]%val[4]);
    return (hash == ~0ULL) ? hash - 1 : hash;
}


/**
 * Notify CSAM of a page flush
 *
 * @returns VBox status code
 * @param   pVM         Pointer to the VM.
 * @param   addr        GC address of the page to flush
 * @param   fRemovePage Page removal flag
 */
static int csamFlushPage(PVM pVM, RTRCPTR addr, bool fRemovePage)
{
    PCSAMPAGEREC pPageRec;
    int rc;
    RTGCPHYS GCPhys = 0;
    uint64_t fFlags = 0;
    Assert(pVM->cCpus == 1 || !CSAMIsEnabled(pVM));

    if (!CSAMIsEnabled(pVM))
        return VINF_SUCCESS;

    PVMCPU pVCpu = VMMGetCpu0(pVM);

    STAM_PROFILE_START(&pVM->csam.s.StatTimeFlushPage, a);

    addr = addr & PAGE_BASE_GC_MASK;

    /*
     * Note: searching for the page in our tree first is more expensive (skipped flushes are two orders of magnitude more common)
     */
    if (pVM->csam.s.pPageTree == NULL)
    {
        STAM_PROFILE_STOP(&pVM->csam.s.StatTimeFlushPage, a);
        return VWRN_CSAM_PAGE_NOT_FOUND;
    }

    rc = PGMGstGetPage(pVCpu, addr, &fFlags, &GCPhys);
    /* Returned at a very early stage (no paging yet presumably). */
    if (rc == VERR_NOT_SUPPORTED)
    {
        STAM_PROFILE_STOP(&pVM->csam.s.StatTimeFlushPage, a);
        return rc;
    }

    if (RT_SUCCESS(rc))
    {
        if (    (fFlags & X86_PTE_US)
            ||  rc == VERR_PGM_PHYS_PAGE_RESERVED
           )
        {
            /* User page -> not relevant for us. */
            STAM_COUNTER_ADD(&pVM->csam.s.StatNrFlushesSkipped, 1);
            STAM_PROFILE_STOP(&pVM->csam.s.StatTimeFlushPage, a);
            return VINF_SUCCESS;
        }
    }
    else
    if (rc != VERR_PAGE_NOT_PRESENT && rc != VERR_PAGE_TABLE_NOT_PRESENT)
        AssertMsgFailed(("PGMR3GetPage %RRv failed with %Rrc\n", addr, rc));

    pPageRec = (PCSAMPAGEREC)RTAvlPVGet(&pVM->csam.s.pPageTree, (AVLPVKEY)(uintptr_t)addr);
    if (pPageRec)
    {
        if (    GCPhys == pPageRec->page.GCPhys
            &&  (fFlags & X86_PTE_P))
        {
            STAM_COUNTER_ADD(&pVM->csam.s.StatNrFlushesSkipped, 1);
            STAM_PROFILE_STOP(&pVM->csam.s.StatTimeFlushPage, a);
            return VINF_SUCCESS;
        }

        Log(("CSAMR3FlushPage: page %RRv has changed -> FLUSH (rc=%Rrc) (Phys: %RGp vs %RGp)\n", addr, rc, GCPhys, pPageRec->page.GCPhys));

        STAM_COUNTER_ADD(&pVM->csam.s.StatNrFlushes, 1);

        if (fRemovePage)
            csamRemovePageRecord(pVM, addr);
        else
        {
            CSAMMarkPage(pVM, addr, false);
            pPageRec->page.GCPhys = 0;
            pPageRec->page.fFlags = 0;
            rc = PGMGstGetPage(pVCpu, addr, &pPageRec->page.fFlags, &pPageRec->page.GCPhys);
            if (rc == VINF_SUCCESS)
                pPageRec->page.u64Hash = csamR3CalcPageHash(pVM, addr);

            if (pPageRec->page.pBitmap == NULL)
            {
                pPageRec->page.pBitmap = (uint8_t *)MMR3HeapAllocZ(pVM, MM_TAG_CSAM_PATCH, CSAM_PAGE_BITMAP_SIZE);
                Assert(pPageRec->page.pBitmap);
                if (pPageRec->page.pBitmap == NULL)
                    return VERR_NO_MEMORY;
            }
            else
                memset(pPageRec->page.pBitmap, 0, CSAM_PAGE_BITMAP_SIZE);
        }


        /*
         * Inform patch manager about the flush; no need to repeat the above check twice.
         */
        PATMR3FlushPage(pVM, addr);

        STAM_PROFILE_STOP(&pVM->csam.s.StatTimeFlushPage, a);
        return VINF_SUCCESS;
    }
    else
    {
        STAM_PROFILE_STOP(&pVM->csam.s.StatTimeFlushPage, a);
        return VWRN_CSAM_PAGE_NOT_FOUND;
    }
}

/**
 * Notify CSAM of a page flush
 *
 * @returns VBox status code
 * @param   pVM         Pointer to the VM.
 * @param   addr        GC address of the page to flush
 */
VMMR3DECL(int) CSAMR3FlushPage(PVM pVM, RTRCPTR addr)
{
    return csamFlushPage(pVM, addr, true /* remove page record */);
}

/**
 * Remove a CSAM monitored page. Use with care!
 *
 * @returns VBox status code
 * @param   pVM         Pointer to the VM.
 * @param   addr        GC address of the page to flush
 */
VMMR3DECL(int) CSAMR3RemovePage(PVM pVM, RTRCPTR addr)
{
    PCSAMPAGEREC pPageRec;
    int          rc;

    addr = addr & PAGE_BASE_GC_MASK;

    pPageRec = (PCSAMPAGEREC)RTAvlPVGet(&pVM->csam.s.pPageTree, (AVLPVKEY)(uintptr_t)addr);
    if (pPageRec)
    {
        rc = csamRemovePageRecord(pVM, addr);
        if (RT_SUCCESS(rc))
            PATMR3FlushPage(pVM, addr);
        return VINF_SUCCESS;
    }
    return VWRN_CSAM_PAGE_NOT_FOUND;
}

/**
 * Check a page record in case a page has been changed
 *
 * @returns VBox status code. (trap handled or not)
 * @param   pVM         Pointer to the VM.
 * @param   pInstrGC    GC instruction pointer
 */
int csamR3CheckPageRecord(PVM pVM, RTRCPTR pInstrGC)
{
    PCSAMPAGEREC pPageRec;
    uint64_t     u64hash;

    pInstrGC = pInstrGC & PAGE_BASE_GC_MASK;

    pPageRec = (PCSAMPAGEREC)RTAvlPVGet(&pVM->csam.s.pPageTree, (AVLPVKEY)(uintptr_t)pInstrGC);
    if (pPageRec)
    {
        u64hash = csamR3CalcPageHash(pVM, pInstrGC);
        if (u64hash != pPageRec->page.u64Hash)
            csamFlushPage(pVM, pInstrGC, false /* don't remove page record */);
    }
    else
        return VWRN_CSAM_PAGE_NOT_FOUND;

    return VINF_SUCCESS;
}

/**
 * Returns monitor description based on CSAM tag
 *
 * @return description string
 * @param   enmTag                  Owner tag
 */
const char *csamGetMonitorDescription(CSAMTAG enmTag)
{
    if (enmTag == CSAM_TAG_PATM)
        return "CSAM-PATM self-modifying code monitor handler";
    else
    if (enmTag == CSAM_TAG_REM)
        return "CSAM-REM self-modifying code monitor handler";
    Assert(enmTag == CSAM_TAG_CSAM);
    return "CSAM self-modifying code monitor handler";
}

/**
 * Adds page record to our lookup tree
 *
 * @returns CSAMPAGE ptr or NULL if failure
 * @param   pVM                     Pointer to the VM.
 * @param   GCPtr                   Page address
 * @param   enmTag                  Owner tag
 * @param   fCode32                 16 or 32 bits code
 * @param   fMonitorInvalidation    Monitor page invalidation flag
 */
static PCSAMPAGE csamCreatePageRecord(PVM pVM, RTRCPTR GCPtr, CSAMTAG enmTag, bool fCode32, bool fMonitorInvalidation)
{
    PCSAMPAGEREC pPage;
    int          rc;
    bool         ret;
    Assert(pVM->cCpus == 1);
    PVMCPU pVCpu = VMMGetCpu0(pVM);

    Log(("New page record for %RRv\n", GCPtr & PAGE_BASE_GC_MASK));

    pPage = (PCSAMPAGEREC)MMR3HeapAllocZ(pVM, MM_TAG_CSAM_PATCH, sizeof(CSAMPAGEREC));
    if (pPage == NULL)
    {
        AssertMsgFailed(("csamCreatePageRecord: Out of memory!!!!\n"));
        return NULL;
    }
    /* Round down to page boundary. */
    GCPtr = (GCPtr & PAGE_BASE_GC_MASK);
    pPage->Core.Key                  = (AVLPVKEY)(uintptr_t)GCPtr;
    pPage->page.pPageGC              = GCPtr;
    pPage->page.fCode32              = fCode32;
    pPage->page.fMonitorInvalidation = fMonitorInvalidation;
    pPage->page.enmTag               = enmTag;
    pPage->page.fMonitorActive       = false;
    pPage->page.pBitmap              = (uint8_t *)MMR3HeapAllocZ(pVM, MM_TAG_CSAM_PATCH, PAGE_SIZE/sizeof(uint8_t));
    rc = PGMGstGetPage(pVCpu, GCPtr, &pPage->page.fFlags, &pPage->page.GCPhys);
    AssertMsg(RT_SUCCESS(rc) || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT, ("rc = %Rrc\n", rc));

    pPage->page.u64Hash   = csamR3CalcPageHash(pVM, GCPtr);
    ret = RTAvlPVInsert(&pVM->csam.s.pPageTree, &pPage->Core);
    Assert(ret);

#ifdef CSAM_MONITOR_CODE_PAGES
    AssertRelease(!fInCSAMCodePageInvalidate);

    switch (enmTag)
    {
    case CSAM_TAG_PATM:
    case CSAM_TAG_REM:
#ifdef CSAM_MONITOR_CSAM_CODE_PAGES
    case CSAM_TAG_CSAM:
#endif
    {
        rc = PGMR3HandlerVirtualRegister(pVM, PGMVIRTHANDLERTYPE_WRITE, GCPtr, GCPtr + (PAGE_SIZE - 1) /* inclusive! */,
                                         (fMonitorInvalidation) ? CSAMCodePageInvalidate : 0, CSAMCodePageWriteHandler, "CSAMGCCodePageWriteHandler", 0,
                                         csamGetMonitorDescription(enmTag));
        AssertMsg(RT_SUCCESS(rc) || rc == VERR_PGM_HANDLER_VIRTUAL_CONFLICT, ("PGMR3HandlerVirtualRegisterEx %RRv failed with %Rrc\n", GCPtr, rc));
        if (RT_FAILURE(rc))
            Log(("PGMR3HandlerVirtualRegisterEx for %RRv failed with %Rrc\n", GCPtr, rc));

        /* Could fail, because it's already monitored. Don't treat that condition as fatal. */

        /* Prefetch it in case it's not there yet. */
        rc = PGMPrefetchPage(pVCpu, GCPtr);
        AssertRC(rc);

        rc = PGMShwMakePageReadonly(pVCpu, GCPtr, 0 /*fFlags*/);
        Assert(rc == VINF_SUCCESS || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT);

        pPage->page.fMonitorActive = true;
        STAM_COUNTER_INC(&pVM->csam.s.StatPageMonitor);
        break;
    }
    default:
        break; /* to shut up GCC */
    }

    Log(("csamCreatePageRecord %RRv GCPhys=%RGp\n", GCPtr, pPage->page.GCPhys));

#ifdef VBOX_WITH_STATISTICS
    switch (enmTag)
    {
    case CSAM_TAG_CSAM:
        STAM_COUNTER_INC(&pVM->csam.s.StatPageCSAM);
        break;
    case CSAM_TAG_PATM:
        STAM_COUNTER_INC(&pVM->csam.s.StatPagePATM);
        break;
    case CSAM_TAG_REM:
        STAM_COUNTER_INC(&pVM->csam.s.StatPageREM);
        break;
    default:
        break; /* to shut up GCC */
    }
#endif

#endif

    STAM_COUNTER_INC(&pVM->csam.s.StatNrPages);
    if (fMonitorInvalidation)
        STAM_COUNTER_INC(&pVM->csam.s.StatNrPagesInv);

    return &pPage->page;
}

/**
 * Monitors a code page (if not already monitored)
 *
 * @returns VBox status code
 * @param   pVM         Pointer to the VM.
 * @param   pPageAddrGC The page to monitor
 * @param   enmTag      Monitor tag
 */
VMMR3DECL(int) CSAMR3MonitorPage(PVM pVM, RTRCPTR pPageAddrGC, CSAMTAG enmTag)
{
    PCSAMPAGEREC pPageRec = NULL;
    int          rc;
    bool         fMonitorInvalidation;
    Assert(pVM->cCpus == 1);
    PVMCPU pVCpu = VMMGetCpu0(pVM);

    /* Dirty pages must be handled before calling this function!. */
    Assert(!pVM->csam.s.cDirtyPages);

    if (pVM->csam.s.fScanningStarted == false)
        return VINF_SUCCESS;    /* too early */

    pPageAddrGC &= PAGE_BASE_GC_MASK;

    Log(("CSAMR3MonitorPage %RRv %d\n", pPageAddrGC, enmTag));

    /** @todo implicit assumption */
    fMonitorInvalidation = (enmTag == CSAM_TAG_PATM);

    pPageRec = (PCSAMPAGEREC)RTAvlPVGet(&pVM->csam.s.pPageTree, (AVLPVKEY)(uintptr_t)pPageAddrGC);
    if (pPageRec == NULL)
    {
        uint64_t fFlags;

        rc = PGMGstGetPage(pVCpu, pPageAddrGC, &fFlags, NULL);
        AssertMsg(RT_SUCCESS(rc) || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT, ("rc = %Rrc\n", rc));
        if (    rc == VINF_SUCCESS
            &&  (fFlags & X86_PTE_US))
        {
            /* We don't care about user pages. */
            STAM_COUNTER_INC(&pVM->csam.s.StatNrUserPages);
            return VINF_SUCCESS;
        }

        csamCreatePageRecord(pVM, pPageAddrGC, enmTag, true /* 32 bits code */, fMonitorInvalidation);

        pPageRec = (PCSAMPAGEREC)RTAvlPVGet(&pVM->csam.s.pPageTree, (AVLPVKEY)(uintptr_t)pPageAddrGC);
        Assert(pPageRec);
    }
    /** @todo reference count */

#ifdef CSAM_MONITOR_CSAM_CODE_PAGES
    Assert(pPageRec->page.fMonitorActive);
#endif

#ifdef CSAM_MONITOR_CODE_PAGES
    if (!pPageRec->page.fMonitorActive)
    {
        Log(("CSAMR3MonitorPage: activate monitoring for %RRv\n", pPageAddrGC));

        rc = PGMR3HandlerVirtualRegister(pVM, PGMVIRTHANDLERTYPE_WRITE, pPageAddrGC, pPageAddrGC + (PAGE_SIZE - 1) /* inclusive! */,
                                         (fMonitorInvalidation) ? CSAMCodePageInvalidate : 0, CSAMCodePageWriteHandler, "CSAMGCCodePageWriteHandler", 0,
                                         csamGetMonitorDescription(enmTag));
        AssertMsg(RT_SUCCESS(rc) || rc == VERR_PGM_HANDLER_VIRTUAL_CONFLICT, ("PGMR3HandlerVirtualRegisterEx %RRv failed with %Rrc\n", pPageAddrGC, rc));
        if (RT_FAILURE(rc))
            Log(("PGMR3HandlerVirtualRegisterEx for %RRv failed with %Rrc\n", pPageAddrGC, rc));

        /* Could fail, because it's already monitored. Don't treat that condition as fatal. */

        /* Prefetch it in case it's not there yet. */
        rc = PGMPrefetchPage(pVCpu, pPageAddrGC);
        AssertRC(rc);

        rc = PGMShwMakePageReadonly(pVCpu, pPageAddrGC, 0 /*fFlags*/);
        Assert(rc == VINF_SUCCESS || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT);

        STAM_COUNTER_INC(&pVM->csam.s.StatPageMonitor);

        pPageRec->page.fMonitorActive       = true;
        pPageRec->page.fMonitorInvalidation = fMonitorInvalidation;
    }
    else
    if (   !pPageRec->page.fMonitorInvalidation
        &&  fMonitorInvalidation)
    {
        Assert(pPageRec->page.fMonitorActive);
        PGMHandlerVirtualChangeInvalidateCallback(pVM, pPageRec->page.pPageGC, CSAMCodePageInvalidate);
        pPageRec->page.fMonitorInvalidation = true;
        STAM_COUNTER_INC(&pVM->csam.s.StatNrPagesInv);

        /* Prefetch it in case it's not there yet. */
        rc = PGMPrefetchPage(pVCpu, pPageAddrGC);
        AssertRC(rc);

        /* Make sure it's readonly. Page invalidation may have modified the attributes. */
        rc = PGMShwMakePageReadonly(pVCpu, pPageAddrGC, 0 /*fFlags*/);
        Assert(rc == VINF_SUCCESS || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT);
    }

#if 0 /* def VBOX_STRICT -> very annoying) */
    if (pPageRec->page.fMonitorActive)
    {
        uint64_t fPageShw;
        RTHCPHYS GCPhys;
        rc = PGMShwGetPage(pVCpu, pPageAddrGC, &fPageShw, &GCPhys);
//        AssertMsg(     (rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT)
//                ||  !(fPageShw & X86_PTE_RW)
//                ||   (pPageRec->page.GCPhys == 0), ("Shadow page flags for %RRv (%RHp) aren't readonly (%RX64)!!\n", pPageAddrGC, GCPhys, fPageShw));
    }
#endif

    if (pPageRec->page.GCPhys == 0)
    {
        /* Prefetch it in case it's not there yet. */
        rc = PGMPrefetchPage(pVCpu, pPageAddrGC);
        AssertRC(rc);
        /* The page was changed behind our back. It won't be made read-only until the next SyncCR3, so force it here. */
        rc = PGMShwMakePageReadonly(pVCpu, pPageAddrGC, 0 /*fFlags*/);
        Assert(rc == VINF_SUCCESS || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT);
    }
#endif /* CSAM_MONITOR_CODE_PAGES */
    return VINF_SUCCESS;
}

/**
 * Unmonitors a code page
 *
 * @returns VBox status code
 * @param   pVM         Pointer to the VM.
 * @param   pPageAddrGC The page to monitor
 * @param   enmTag      Monitor tag
 */
VMMR3DECL(int) CSAMR3UnmonitorPage(PVM pVM, RTRCPTR pPageAddrGC, CSAMTAG enmTag)
{
    pPageAddrGC &= PAGE_BASE_GC_MASK;

    Log(("CSAMR3UnmonitorPage %RRv %d\n", pPageAddrGC, enmTag));

    Assert(enmTag == CSAM_TAG_REM);

#ifdef VBOX_STRICT
    PCSAMPAGEREC pPageRec;

    pPageRec = (PCSAMPAGEREC)RTAvlPVGet(&pVM->csam.s.pPageTree, (AVLPVKEY)(uintptr_t)pPageAddrGC);
    Assert(pPageRec && pPageRec->page.enmTag == enmTag);
#endif
    return CSAMR3RemovePage(pVM, pPageAddrGC);
}

/**
 * Removes a page record from our lookup tree
 *
 * @returns VBox status code
 * @param   pVM         Pointer to the VM.
 * @param   GCPtr       Page address
 */
static int csamRemovePageRecord(PVM pVM, RTRCPTR GCPtr)
{
    PCSAMPAGEREC pPageRec;
    Assert(pVM->cCpus == 1);
    PVMCPU pVCpu = VMMGetCpu0(pVM);

    Log(("csamRemovePageRecord %RRv\n", GCPtr));
    pPageRec = (PCSAMPAGEREC)RTAvlPVRemove(&pVM->csam.s.pPageTree, (AVLPVKEY)(uintptr_t)GCPtr);

    if (pPageRec)
    {
        STAM_COUNTER_INC(&pVM->csam.s.StatNrRemovedPages);

#ifdef CSAM_MONITOR_CODE_PAGES
        if (pPageRec->page.fMonitorActive)
        {
            /* @todo -> this is expensive (cr3 reload)!!!
             * if this happens often, then reuse it instead!!!
             */
            Assert(!fInCSAMCodePageInvalidate);
            STAM_COUNTER_DEC(&pVM->csam.s.StatPageMonitor);
            PGMHandlerVirtualDeregister(pVM, GCPtr);
        }
        if (pPageRec->page.enmTag == CSAM_TAG_PATM)
        {
            /* Make sure the recompiler flushes its cache as this page is no longer monitored. */
            STAM_COUNTER_INC(&pVM->csam.s.StatPageRemoveREMFlush);
            CPUMSetChangedFlags(pVCpu, CPUM_CHANGED_GLOBAL_TLB_FLUSH);
        }
#endif

#ifdef VBOX_WITH_STATISTICS
        switch (pPageRec->page.enmTag)
        {
        case CSAM_TAG_CSAM:
            STAM_COUNTER_DEC(&pVM->csam.s.StatPageCSAM);
            break;
        case CSAM_TAG_PATM:
            STAM_COUNTER_DEC(&pVM->csam.s.StatPagePATM);
            break;
        case CSAM_TAG_REM:
            STAM_COUNTER_DEC(&pVM->csam.s.StatPageREM);
            break;
        default:
            break; /* to shut up GCC */
        }
#endif

        if (pPageRec->page.pBitmap) MMR3HeapFree(pPageRec->page.pBitmap);
        MMR3HeapFree(pPageRec);
    }
    else
        AssertFailed();

    return VINF_SUCCESS;
}

/**
 * Callback for delayed writes from non-EMT threads
 *
 * @param   pVM             Pointer to the VM.
 * @param   GCPtr           The virtual address the guest is writing to. (not correct if it's an alias!)
 * @param   cbBuf           How much it's reading/writing.
 */
static DECLCALLBACK(void) CSAMDelayedWriteHandler(PVM pVM, RTRCPTR GCPtr, size_t cbBuf)
{
    int rc = PATMR3PatchWrite(pVM, GCPtr, (uint32_t)cbBuf);
    AssertRC(rc);
}

/**
 * \#PF Handler callback for virtual access handler ranges.
 *
 * Important to realize that a physical page in a range can have aliases, and
 * for ALL and WRITE handlers these will also trigger.
 *
 * @returns VINF_SUCCESS if the handler have carried out the operation.
 * @returns VINF_PGM_HANDLER_DO_DEFAULT if the caller should carry out the access operation.
 * @param   pVM             Pointer to the VM.
 * @param   GCPtr           The virtual address the guest is writing to. (not correct if it's an alias!)
 * @param   pvPtr           The HC mapping of that address.
 * @param   pvBuf           What the guest is reading/writing.
 * @param   cbBuf           How much it's reading/writing.
 * @param   enmAccessType   The access type.
 * @param   pvUser          User argument.
 */
static DECLCALLBACK(int) CSAMCodePageWriteHandler(PVM pVM, RTGCPTR GCPtr, void *pvPtr, void *pvBuf, size_t cbBuf, PGMACCESSTYPE enmAccessType, void *pvUser)
{
    int rc;

    Assert(enmAccessType == PGMACCESSTYPE_WRITE); NOREF(enmAccessType);
    Log(("CSAMCodePageWriteHandler: write to %RGv size=%zu\n", GCPtr, cbBuf));
    NOREF(pvUser);

    if (    PAGE_ADDRESS(pvPtr) == PAGE_ADDRESS((uintptr_t)pvPtr + cbBuf - 1)
         && !memcmp(pvPtr, pvBuf, cbBuf))
    {
        Log(("CSAMCodePageWriteHandler: dummy write -> ignore\n"));
        return VINF_PGM_HANDLER_DO_DEFAULT;
    }

    if (VM_IS_EMT(pVM))
        rc = PATMR3PatchWrite(pVM, GCPtr, (uint32_t)cbBuf);
    else
    {
        /* Queue the write instead otherwise we'll get concurrency issues. */
        /** @note in theory not correct to let it write the data first before disabling a patch!
         *        (if it writes the same data as the patch jump and we replace it with obsolete opcodes)
         */
        Log(("CSAMCodePageWriteHandler: delayed write!\n"));
        AssertCompileSize(RTRCPTR, 4);
        rc = VMR3ReqCallVoidNoWait(pVM, VMCPUID_ANY, (PFNRT)CSAMDelayedWriteHandler, 3, pVM, (RTRCPTR)GCPtr, cbBuf);
    }
    AssertRC(rc);

    return VINF_PGM_HANDLER_DO_DEFAULT;
}

/**
 * \#PF Handler callback for invalidation of virtual access handler ranges.
 *
 * @param   pVM             Pointer to the VM.
 * @param   GCPtr           The virtual address the guest has changed.
 */
static DECLCALLBACK(int) CSAMCodePageInvalidate(PVM pVM, RTGCPTR GCPtr)
{
    fInCSAMCodePageInvalidate = true;
    LogFlow(("CSAMCodePageInvalidate %RGv\n", GCPtr));
    /** @todo We can't remove the page (which unregisters the virtual handler) as we are called from a DoWithAll on the virtual handler tree. Argh. */
    csamFlushPage(pVM, GCPtr, false /* don't remove page! */);
    fInCSAMCodePageInvalidate = false;
    return VINF_SUCCESS;
}

/**
 * Check if the current instruction has already been checked before
 *
 * @returns VBox status code. (trap handled or not)
 * @param   pVM         Pointer to the VM.
 * @param   pInstr      Instruction pointer
 * @param   pPage       CSAM patch structure pointer
 */
bool csamIsCodeScanned(PVM pVM, RTRCPTR pInstr, PCSAMPAGE *pPage)
{
    PCSAMPAGEREC pPageRec;
    uint32_t offset;

    STAM_PROFILE_START(&pVM->csam.s.StatTimeCheckAddr, a);

    offset = pInstr & PAGE_OFFSET_MASK;
    pInstr = pInstr & PAGE_BASE_GC_MASK;

    Assert(pPage);

    if (*pPage && (*pPage)->pPageGC == pInstr)
    {
        if ((*pPage)->pBitmap == NULL || ASMBitTest((*pPage)->pBitmap, offset))
        {
            STAM_COUNTER_ADD(&pVM->csam.s.StatNrKnownPagesHC, 1);
            STAM_PROFILE_STOP(&pVM->csam.s.StatTimeCheckAddr, a);
            return true;
        }
        STAM_PROFILE_STOP(&pVM->csam.s.StatTimeCheckAddr, a);
        return false;
    }

    pPageRec = (PCSAMPAGEREC)RTAvlPVGet(&pVM->csam.s.pPageTree, (AVLPVKEY)(uintptr_t)pInstr);
    if (pPageRec)
    {
        if (pPage) *pPage= &pPageRec->page;
        if (pPageRec->page.pBitmap == NULL || ASMBitTest(pPageRec->page.pBitmap, offset))
        {
            STAM_COUNTER_ADD(&pVM->csam.s.StatNrKnownPagesHC, 1);
            STAM_PROFILE_STOP(&pVM->csam.s.StatTimeCheckAddr, a);
            return true;
        }
    }
    else
    {
        if (pPage) *pPage = NULL;
    }
    STAM_PROFILE_STOP(&pVM->csam.s.StatTimeCheckAddr, a);
    return false;
}

/**
 * Mark an instruction in a page as scanned/not scanned
 *
 * @param   pVM         Pointer to the VM.
 * @param   pPage       Patch structure pointer
 * @param   pInstr      Instruction pointer
 * @param   cbInstr      Instruction size
 * @param   fScanned    Mark as scanned or not
 */
static void csamMarkCode(PVM pVM, PCSAMPAGE pPage, RTRCPTR pInstr, uint32_t cbInstr, bool fScanned)
{
    LogFlow(("csamMarkCodeAsScanned %RRv cbInstr=%d\n", pInstr, cbInstr));
    CSAMMarkPage(pVM, pInstr, fScanned);

    /** @todo should recreate empty bitmap if !fScanned */
    if (pPage->pBitmap == NULL)
        return;

    if (fScanned)
    {
        // retn instructions can be scanned more than once
        if (ASMBitTest(pPage->pBitmap, pInstr & PAGE_OFFSET_MASK) == 0)
        {
            pPage->uSize += cbInstr;
            STAM_COUNTER_ADD(&pVM->csam.s.StatNrInstr, 1);
        }
        if (pPage->uSize >= PAGE_SIZE)
        {
            Log(("Scanned full page (%RRv) -> free bitmap\n", pInstr & PAGE_BASE_GC_MASK));
            MMR3HeapFree(pPage->pBitmap);
            pPage->pBitmap = NULL;
        }
        else
            ASMBitSet(pPage->pBitmap, pInstr & PAGE_OFFSET_MASK);
    }
    else
        ASMBitClear(pPage->pBitmap, pInstr & PAGE_OFFSET_MASK);
}

/**
 * Mark an instruction in a page as scanned/not scanned
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   pInstr      Instruction pointer
 * @param   cbInstr      Instruction size
 * @param   fScanned    Mark as scanned or not
 */
VMMR3DECL(int) CSAMR3MarkCode(PVM pVM, RTRCPTR pInstr, uint32_t cbInstr, bool fScanned)
{
    PCSAMPAGE pPage = 0;

    Assert(!fScanned);   /* other case not implemented. */
    Assert(!PATMIsPatchGCAddr(pVM, pInstr));

    if (csamIsCodeScanned(pVM, pInstr, &pPage) == false)
    {
        Assert(fScanned == true);   /* other case should not be possible */
        return VINF_SUCCESS;
    }

    Log(("CSAMR3MarkCode: %RRv size=%d fScanned=%d\n", pInstr, cbInstr, fScanned));
    csamMarkCode(pVM, pPage, pInstr, cbInstr, fScanned);
    return VINF_SUCCESS;
}


/**
 * Scan and analyse code
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   pCtxCore    CPU context
 * @param   pInstrGC    Instruction pointer
 */
VMMR3DECL(int) CSAMR3CheckCodeEx(PVM pVM, PCPUMCTXCORE pCtxCore, RTRCPTR pInstrGC)
{
    if (EMIsRawRing0Enabled(pVM) == false || PATMIsPatchGCAddr(pVM, pInstrGC) == true)
    {
        // No use
        return VINF_SUCCESS;
    }

    if (CSAMIsEnabled(pVM))
    {
        /* Assuming 32 bits code for now. */
        Assert(CPUMGetGuestCodeBits(VMMGetCpu0(pVM)) == 32);

        pInstrGC = SELMToFlat(pVM, DISSELREG_CS, pCtxCore, pInstrGC);
        return CSAMR3CheckCode(pVM, pInstrGC);
    }
    return VINF_SUCCESS;
}

/**
 * Scan and analyse code
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   pInstrGC    Instruction pointer (0:32 virtual address)
 */
VMMR3DECL(int) CSAMR3CheckCode(PVM pVM, RTRCPTR pInstrGC)
{
    int rc;
    PCSAMPAGE pPage = NULL;

    if (    EMIsRawRing0Enabled(pVM) == false
        ||  PATMIsPatchGCAddr(pVM, pInstrGC) == true)
    {
        /* Not active. */
        return VINF_SUCCESS;
    }

    if (CSAMIsEnabled(pVM))
    {
        /* Cache record for CSAMGCVirtToHCVirt */
        CSAMP2GLOOKUPREC cacheRec;
        RT_ZERO(cacheRec);

        STAM_PROFILE_START(&pVM->csam.s.StatTime, a);
        rc = csamAnalyseCallCodeStream(pVM, pInstrGC, pInstrGC, true /* 32 bits code */, CSAMR3AnalyseCallback, pPage, &cacheRec);
        STAM_PROFILE_STOP(&pVM->csam.s.StatTime, a);
        if (cacheRec.Lock.pvMap)
            PGMPhysReleasePageMappingLock(pVM, &cacheRec.Lock);

        if (rc != VINF_SUCCESS)
        {
            Log(("csamAnalyseCodeStream failed with %d\n", rc));
            return rc;
        }
    }
    return VINF_SUCCESS;
}

/**
 * Flush dirty code pages
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 */
static int csamR3FlushDirtyPages(PVM pVM)
{
    Assert(pVM->cCpus == 1);
    PVMCPU pVCpu = VMMGetCpu0(pVM);

    STAM_PROFILE_START(&pVM->csam.s.StatFlushDirtyPages, a);

    for (uint32_t i=0;i<pVM->csam.s.cDirtyPages;i++)
    {
        int          rc;
        PCSAMPAGEREC pPageRec;
        RTRCPTR      GCPtr = pVM->csam.s.pvDirtyBasePage[i];

        GCPtr = GCPtr & PAGE_BASE_GC_MASK;

#ifdef VBOX_WITH_REM
         /* Notify the recompiler that this page has been changed. */
        REMR3NotifyCodePageChanged(pVM, pVCpu, GCPtr);
#endif

        /* Enable write protection again. (use the fault address as it might be an alias) */
        rc = PGMShwMakePageReadonly(pVCpu, pVM->csam.s.pvDirtyFaultPage[i], 0 /*fFlags*/);
        Assert(rc == VINF_SUCCESS || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT);

        Log(("CSAMR3FlushDirtyPages: flush %RRv (modifypage rc=%Rrc)\n", pVM->csam.s.pvDirtyBasePage[i], rc));

        pPageRec = (PCSAMPAGEREC)RTAvlPVGet(&pVM->csam.s.pPageTree, (AVLPVKEY)(uintptr_t)GCPtr);
        if (pPageRec && pPageRec->page.enmTag == CSAM_TAG_REM)
        {
            uint64_t fFlags;

            rc = PGMGstGetPage(pVCpu, GCPtr, &fFlags, NULL);
            AssertMsg(RT_SUCCESS(rc) || rc == VERR_PAGE_NOT_PRESENT || rc == VERR_PAGE_TABLE_NOT_PRESENT, ("rc = %Rrc\n", rc));
            if (    rc == VINF_SUCCESS
                &&  (fFlags & X86_PTE_US))
            {
                /* We don't care about user pages. */
                csamRemovePageRecord(pVM, GCPtr);
                STAM_COUNTER_INC(&pVM->csam.s.StatNrUserPages);
            }
        }
    }
    pVM->csam.s.cDirtyPages = 0;
    STAM_PROFILE_STOP(&pVM->csam.s.StatFlushDirtyPages, a);
    return VINF_SUCCESS;
}

/**
 * Flush potential new code pages
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 */
static int csamR3FlushCodePages(PVM pVM)
{
    Assert(pVM->cCpus == 1);
    PVMCPU pVCpu = VMMGetCpu0(pVM);

    for (uint32_t i=0;i<pVM->csam.s.cPossibleCodePages;i++)
    {
        RTRCPTR GCPtr = pVM->csam.s.pvPossibleCodePage[i];

        GCPtr = GCPtr & PAGE_BASE_GC_MASK;

        Log(("csamR3FlushCodePages: %RRv\n", GCPtr));
        PGMShwMakePageNotPresent(pVCpu, GCPtr, 0 /*fFlags*/);
        /* Resync the page to make sure instruction fetch will fault */
        CSAMMarkPage(pVM, GCPtr, false);
    }
    pVM->csam.s.cPossibleCodePages = 0;
    return VINF_SUCCESS;
}

/**
 * Perform any pending actions
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   pVCpu       Pointer to the VMCPU.
 */
VMMR3DECL(int) CSAMR3DoPendingAction(PVM pVM, PVMCPU pVCpu)
{
    csamR3FlushDirtyPages(pVM);
    csamR3FlushCodePages(pVM);

    VMCPU_FF_CLEAR(pVCpu, VMCPU_FF_CSAM_PENDING_ACTION);
    return VINF_SUCCESS;
}

/**
 * Analyse interrupt and trap gates
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   iGate       Start gate
 * @param   cGates      Number of gates to check
 */
VMMR3DECL(int) CSAMR3CheckGates(PVM pVM, uint32_t iGate, uint32_t cGates)
{
#ifdef VBOX_WITH_RAW_MODE
    Assert(pVM->cCpus == 1);
    PVMCPU      pVCpu = VMMGetCpu0(pVM);
    uint16_t    cbIDT;
    RTRCPTR     GCPtrIDT = CPUMGetGuestIDTR(pVCpu, &cbIDT);
    uint32_t    iGateEnd;
    uint32_t    maxGates;
    VBOXIDTE    aIDT[256];
    PVBOXIDTE   pGuestIdte;
    int         rc;

    if (EMIsRawRing0Enabled(pVM) == false)
    {
        /* Enabling interrupt gates only works when raw ring 0 is enabled. */
        //AssertFailed();
        return VINF_SUCCESS;
    }

    /* We only check all gates once during a session */
    if (    !pVM->csam.s.fGatesChecked
        &&   cGates != 256)
        return VINF_SUCCESS;    /* too early */

    /* We only check all gates once during a session */
    if (    pVM->csam.s.fGatesChecked
        &&  cGates != 1)
        return VINF_SUCCESS;    /* ignored */

    Assert(cGates <= 256);
    if (!GCPtrIDT || cGates > 256)
        return VERR_INVALID_PARAMETER;

    if (cGates != 1)
    {
        pVM->csam.s.fGatesChecked = true;
        for (unsigned i=0;i<RT_ELEMENTS(pVM->csam.s.pvCallInstruction);i++)
        {
            RTRCPTR pHandler = pVM->csam.s.pvCallInstruction[i];

            if (pHandler)
            {
                PCSAMPAGE pPage = NULL;
                CSAMP2GLOOKUPREC cacheRec;                  /* Cache record for CSAMGCVirtToHCVirt. */
                RT_ZERO(cacheRec);

                Log(("CSAMCheckGates: checking previous call instruction %RRv\n", pHandler));
                STAM_PROFILE_START(&pVM->csam.s.StatTime, a);
                rc = csamAnalyseCodeStream(pVM, pHandler, pHandler, true, CSAMR3AnalyseCallback, pPage, &cacheRec);
                STAM_PROFILE_STOP(&pVM->csam.s.StatTime, a);
                if (cacheRec.Lock.pvMap)
                    PGMPhysReleasePageMappingLock(pVM, &cacheRec.Lock);

                if (rc != VINF_SUCCESS)
                {
                    Log(("CSAMCheckGates: csamAnalyseCodeStream failed with %d\n", rc));
                    continue;
                }
            }
        }
    }

    /* Determine valid upper boundary. */
    maxGates   = (cbIDT+1) / sizeof(VBOXIDTE);
    Assert(iGate < maxGates);
    if (iGate > maxGates)
        return VERR_INVALID_PARAMETER;

    if (iGate + cGates > maxGates)
        cGates = maxGates - iGate;

    GCPtrIDT   = GCPtrIDT + iGate * sizeof(VBOXIDTE);
    iGateEnd   = iGate + cGates;

    STAM_PROFILE_START(&pVM->csam.s.StatCheckGates, a);

    /*
     * Get IDT entries.
     */
    rc = PGMPhysSimpleReadGCPtr(pVCpu, aIDT, GCPtrIDT,  cGates*sizeof(VBOXIDTE));
    if (RT_FAILURE(rc))
    {
        AssertMsgRC(rc, ("Failed to read IDTE! rc=%Rrc\n", rc));
        STAM_PROFILE_STOP(&pVM->csam.s.StatCheckGates, a);
        return rc;
    }
    pGuestIdte = &aIDT[0];

    for (/*iGate*/; iGate<iGateEnd; iGate++, pGuestIdte++)
    {
        Assert(TRPMR3GetGuestTrapHandler(pVM, iGate) == TRPM_INVALID_HANDLER);

        if (    pGuestIdte->Gen.u1Present
            &&  (pGuestIdte->Gen.u5Type2 == VBOX_IDTE_TYPE2_TRAP_32 || pGuestIdte->Gen.u5Type2 == VBOX_IDTE_TYPE2_INT_32)
            &&  (pGuestIdte->Gen.u2DPL == 3 || pGuestIdte->Gen.u2DPL == 0)
           )
        {
            RTRCPTR pHandler;
            PCSAMPAGE pPage = NULL;
            DBGFSELINFO selInfo;
            CSAMP2GLOOKUPREC cacheRec;                  /* Cache record for CSAMGCVirtToHCVirt. */
            RT_ZERO(cacheRec);

            pHandler = VBOXIDTE_OFFSET(*pGuestIdte);
            pHandler = SELMToFlatBySel(pVM, pGuestIdte->Gen.u16SegSel, pHandler);

            rc = SELMR3GetSelectorInfo(pVM, pVCpu, pGuestIdte->Gen.u16SegSel, &selInfo);
            if (    RT_FAILURE(rc)
                ||  (selInfo.fFlags & (DBGFSELINFO_FLAGS_NOT_PRESENT | DBGFSELINFO_FLAGS_INVALID))
                ||  selInfo.GCPtrBase != 0
                ||  selInfo.cbLimit != ~0U
               )
            {
                /* Refuse to patch a handler whose idt cs selector isn't wide open. */
                Log(("CSAMCheckGates: check gate %d failed due to rc %Rrc GCPtrBase=%RRv limit=%x\n", iGate, rc, selInfo.GCPtrBase, selInfo.cbLimit));
                continue;
            }


            if (pGuestIdte->Gen.u5Type2 == VBOX_IDTE_TYPE2_TRAP_32)
            {
                Log(("CSAMCheckGates: check trap gate %d at %04X:%08X (flat %RRv)\n", iGate, pGuestIdte->Gen.u16SegSel, VBOXIDTE_OFFSET(*pGuestIdte), pHandler));
            }
            else
            {
                Log(("CSAMCheckGates: check interrupt gate %d at %04X:%08X (flat %RRv)\n", iGate, pGuestIdte->Gen.u16SegSel, VBOXIDTE_OFFSET(*pGuestIdte), pHandler));
            }

            STAM_PROFILE_START(&pVM->csam.s.StatTime, b);
            rc = csamAnalyseCodeStream(pVM, pHandler, pHandler, true, CSAMR3AnalyseCallback, pPage, &cacheRec);
            STAM_PROFILE_STOP(&pVM->csam.s.StatTime, b);
            if (cacheRec.Lock.pvMap)
                PGMPhysReleasePageMappingLock(pVM, &cacheRec.Lock);

            if (rc != VINF_SUCCESS)
            {
                Log(("CSAMCheckGates: csamAnalyseCodeStream failed with %d\n", rc));
                continue;
            }
            /* OpenBSD guest specific patch test. */
            if (iGate >= 0x20)
            {
                PCPUMCTX    pCtx;
                DISCPUSTATE cpu;
                RTGCUINTPTR32 aOpenBsdPushCSOffset[3] = {0x03,       /* OpenBSD 3.7 & 3.8 */
                                                       0x2B,       /* OpenBSD 4.0 installation ISO */
                                                       0x2F};      /* OpenBSD 4.0 after install */

                pCtx = CPUMQueryGuestCtxPtr(pVCpu);

                for (unsigned i=0;i<RT_ELEMENTS(aOpenBsdPushCSOffset);i++)
                {
                    rc = CPUMR3DisasmInstrCPU(pVM, pVCpu, pCtx, pHandler - aOpenBsdPushCSOffset[i], &cpu, NULL);
                    if (    rc == VINF_SUCCESS
                        &&  cpu.pCurInstr->uOpcode == OP_PUSH
                        &&  cpu.pCurInstr->fParam1 == OP_PARM_REG_CS)
                    {
                        rc = PATMR3InstallPatch(pVM, pHandler - aOpenBsdPushCSOffset[i], PATMFL_CODE32 | PATMFL_GUEST_SPECIFIC);
                        if (RT_SUCCESS(rc))
                            Log(("Installed OpenBSD interrupt handler prefix instruction (push cs) patch\n"));
                    }
                }
            }

            /* Trap gates and certain interrupt gates. */
            uint32_t fPatchFlags = PATMFL_CODE32 | PATMFL_IDTHANDLER;

            if (pGuestIdte->Gen.u5Type2 == VBOX_IDTE_TYPE2_TRAP_32)
                fPatchFlags |= PATMFL_TRAPHANDLER;
            else
                fPatchFlags |= PATMFL_INTHANDLER;

            switch (iGate) {
            case 8:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
            case 17:
                fPatchFlags |= PATMFL_TRAPHANDLER_WITH_ERRORCODE;
                break;
            default:
                /* No error code. */
                break;
            }

            Log(("Installing %s gate handler for 0x%X at %RRv\n", (pGuestIdte->Gen.u5Type2 == VBOX_IDTE_TYPE2_TRAP_32) ? "trap" : "intr", iGate, pHandler));

            rc = PATMR3InstallPatch(pVM, pHandler, fPatchFlags);
            if (    RT_SUCCESS(rc) 
                ||  rc == VERR_PATM_ALREADY_PATCHED)
            {
                Log(("Gate handler 0x%X is SAFE!\n", iGate));

                RTRCPTR pNewHandlerGC = PATMR3QueryPatchGCPtr(pVM, pHandler);
                if (pNewHandlerGC)
                {
                    rc = TRPMR3SetGuestTrapHandler(pVM, iGate, pNewHandlerGC);
                    if (RT_FAILURE(rc))
                        Log(("TRPMR3SetGuestTrapHandler %d failed with %Rrc\n", iGate, rc));
                }
            }
        }
    } /* for */
    STAM_PROFILE_STOP(&pVM->csam.s.StatCheckGates, a);
#endif /* VBOX_WITH_RAW_MODE */
    return VINF_SUCCESS;
}

/**
 * Record previous call instruction addresses
 *
 * @returns VBox status code.
 * @param   pVM         Pointer to the VM.
 * @param   GCPtrCall   Call address
 */
VMMR3DECL(int) CSAMR3RecordCallAddress(PVM pVM, RTRCPTR GCPtrCall)
{
    for (unsigned i=0;i<RT_ELEMENTS(pVM->csam.s.pvCallInstruction);i++)
    {
        if (pVM->csam.s.pvCallInstruction[i] == GCPtrCall)
            return VINF_SUCCESS;
    }

    Log(("CSAMR3RecordCallAddress %RRv\n", GCPtrCall));

    pVM->csam.s.pvCallInstruction[pVM->csam.s.iCallInstruction++] = GCPtrCall;
    if (pVM->csam.s.iCallInstruction >= RT_ELEMENTS(pVM->csam.s.pvCallInstruction))
        pVM->csam.s.iCallInstruction = 0;

    return VINF_SUCCESS;
}


/**
 * Query CSAM state (enabled/disabled)
 *
 * @returns 0 - disabled, 1 - enabled
 * @param   pVM         Pointer to the VM.
 */
VMMR3DECL(int) CSAMR3IsEnabled(PVM pVM)
{
    return pVM->fCSAMEnabled;
}

#ifdef VBOX_WITH_DEBUGGER

/**
 * The '.csamoff' command.
 *
 * @returns VBox status.
 * @param   pCmd        Pointer to the command descriptor (as registered).
 * @param   pCmdHlp     Pointer to command helper functions.
 * @param   pVM         Pointer to the current VM (if any).
 * @param   paArgs      Pointer to (readonly) array of arguments.
 * @param   cArgs       Number of arguments in the array.
 */
static DECLCALLBACK(int) csamr3CmdOff(PCDBGCCMD pCmd, PDBGCCMDHLP pCmdHlp, PVM pVM, PCDBGCVAR paArgs, unsigned cArgs)
{
    DBGC_CMDHLP_REQ_VM_RET(pCmdHlp, pCmd, pVM);
    NOREF(cArgs); NOREF(paArgs);

    int rc = CSAMDisableScanning(pVM);
    if (RT_FAILURE(rc))
        return DBGCCmdHlpFailRc(pCmdHlp, pCmd, rc, "CSAMDisableScanning");
    return DBGCCmdHlpPrintf(pCmdHlp, "CSAM Scanning disabled\n");
}

/**
 * The '.csamon' command.
 *
 * @returns VBox status.
 * @param   pCmd        Pointer to the command descriptor (as registered).
 * @param   pCmdHlp     Pointer to command helper functions.
 * @param   pVM         Pointer to the current VM (if any).
 * @param   paArgs      Pointer to (readonly) array of arguments.
 * @param   cArgs       Number of arguments in the array.
 */
static DECLCALLBACK(int) csamr3CmdOn(PCDBGCCMD pCmd, PDBGCCMDHLP pCmdHlp, PVM pVM, PCDBGCVAR paArgs, unsigned cArgs)
{
    DBGC_CMDHLP_REQ_VM_RET(pCmdHlp, pCmd, pVM);
    NOREF(cArgs); NOREF(paArgs);

    int rc = CSAMEnableScanning(pVM);
    if (RT_FAILURE(rc))
        return DBGCCmdHlpFailRc(pCmdHlp, pCmd, rc, "CSAMEnableScanning");
    return DBGCCmdHlpPrintf(pCmdHlp, "CSAM Scanning enabled\n");
}

#endif /* VBOX_WITH_DEBUGGER */
