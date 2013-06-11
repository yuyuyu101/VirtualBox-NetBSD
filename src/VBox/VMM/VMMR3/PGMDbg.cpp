/* $Id: PGMDbg.cpp $ */
/** @file
 * PGM - Page Manager and Monitor - Debugger & Debugging APIs.
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
#define LOG_GROUP LOG_GROUP_PGM
#include <VBox/vmm/pgm.h>
#include <VBox/vmm/stam.h>
#include "PGMInternal.h"
#include <VBox/vmm/vm.h>
#include "PGMInline.h"
#include <iprt/assert.h>
#include <iprt/asm.h>
#include <iprt/string.h>
#include <VBox/log.h>
#include <VBox/param.h>
#include <VBox/err.h>


/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
/** The max needle size that we will bother searching for
 * This must not be more than half a page! */
#define MAX_NEEDLE_SIZE     256


/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/
/**
 * State structure for the paging hierarchy dumpers.
 */
typedef struct PGMR3DUMPHIERARCHYSTATE
{
    /** Pointer to the VM. */
    PVM             pVM;
    /** Output helpers. */
    PCDBGFINFOHLP   pHlp;
    /** Set if PSE, PAE or long mode is enabled. */
    bool            fPse;
    /** Set if PAE or long mode is enabled. */
    bool            fPae;
    /** Set if long mode is enabled. */
    bool            fLme;
    /** Set if nested paging. */
    bool            fNp;
    /** Set if EPT. */
    bool            fEpt;
    /** Set if NXE is enabled. */
    bool            fNxe;
    /** The number or chars the address needs. */
    uint8_t         cchAddress;
    /** The last reserved bit. */
    uint8_t         uLastRsvdBit;
    /** Dump the page info as well (shadow page summary / guest physical
     *  page summary). */
    bool            fDumpPageInfo;
    /** Whether or not to print the header. */
    bool            fPrintHeader;
    /** Whether to print the CR3 value */
    bool            fPrintCr3;
    /** Padding*/
    bool            afReserved[5];
    /** The current address.  */
    uint64_t        u64Address;
    /** The last address to dump structures for. */
    uint64_t        u64FirstAddress;
    /** The last address to dump structures for. */
    uint64_t        u64LastAddress;
    /** Mask with the high reserved bits set. */
    uint64_t        u64HighReservedBits;
    /** The number of leaf entries that we've printed. */
    uint64_t        cLeaves;
} PGMR3DUMPHIERARCHYSTATE;
/** Pointer to the paging hierarchy dumper state. */
typedef PGMR3DUMPHIERARCHYSTATE *PPGMR3DUMPHIERARCHYSTATE;



/**
 * Converts a R3 pointer to a GC physical address.
 *
 * Only for the debugger.
 *
 * @returns VBox status code.
 * @retval  VINF_SUCCESS on success, *pGCPhys is set.
 * @retval  VERR_INVALID_POINTER if the pointer is not within the GC physical memory.
 *
 * @param   pVM         Pointer to the VM.
 * @param   R3Ptr       The R3 pointer to convert.
 * @param   pGCPhys     Where to store the GC physical address on success.
 */
VMMR3DECL(int) PGMR3DbgR3Ptr2GCPhys(PVM pVM, RTR3PTR R3Ptr, PRTGCPHYS pGCPhys)
{
    NOREF(pVM); NOREF(R3Ptr);
    *pGCPhys = NIL_RTGCPHYS;
    return VERR_NOT_IMPLEMENTED;
}


/**
 * Converts a R3 pointer to a HC physical address.
 *
 * Only for the debugger.
 *
 * @returns VBox status code.
 * @retval  VINF_SUCCESS on success, *pHCPhys is set.
 * @retval  VERR_PGM_PHYS_PAGE_RESERVED it it's a valid GC physical page but has no physical backing.
 * @retval  VERR_INVALID_POINTER if the pointer is not within the GC physical memory.
 *
 * @param   pVM         Pointer to the VM.
 * @param   R3Ptr       The R3 pointer to convert.
 * @param   pHCPhys     Where to store the HC physical address on success.
 */
VMMR3DECL(int) PGMR3DbgR3Ptr2HCPhys(PVM pVM, RTR3PTR R3Ptr, PRTHCPHYS pHCPhys)
{
    NOREF(pVM); NOREF(R3Ptr);
    *pHCPhys = NIL_RTHCPHYS;
    return VERR_NOT_IMPLEMENTED;
}


/**
 * Converts a HC physical address to a GC physical address.
 *
 * Only for the debugger.
 *
 * @returns VBox status code
 * @retval  VINF_SUCCESS on success, *pGCPhys is set.
 * @retval  VERR_INVALID_POINTER if the HC physical address is not within the GC physical memory.
 *
 * @param   pVM     Pointer to the VM.
 * @param   HCPhys  The HC physical address to convert.
 * @param   pGCPhys Where to store the GC physical address on success.
 */
VMMR3DECL(int) PGMR3DbgHCPhys2GCPhys(PVM pVM, RTHCPHYS HCPhys, PRTGCPHYS pGCPhys)
{
    /*
     * Validate and adjust the input a bit.
     */
    if (HCPhys == NIL_RTHCPHYS)
        return VERR_INVALID_POINTER;
    unsigned off = HCPhys & PAGE_OFFSET_MASK;
    HCPhys &= X86_PTE_PAE_PG_MASK;
    if (HCPhys == 0)
        return VERR_INVALID_POINTER;

    for (PPGMRAMRANGE pRam = pVM->pgm.s.CTX_SUFF(pRamRangesX);
         pRam;
         pRam = pRam->CTX_SUFF(pNext))
    {
        uint32_t iPage = pRam->cb >> PAGE_SHIFT;
        while (iPage-- > 0)
            if (PGM_PAGE_GET_HCPHYS(&pRam->aPages[iPage]) == HCPhys)
            {
                *pGCPhys = pRam->GCPhys + (iPage << PAGE_SHIFT) + off;
                return VINF_SUCCESS;
            }
    }
    return VERR_INVALID_POINTER;
}


/**
 * Read physical memory API for the debugger, similar to
 * PGMPhysSimpleReadGCPhys.
 *
 * @returns VBox status code.
 *
 * @param   pVM         Pointer to the VM.
 * @param   pvDst       Where to store what's read.
 * @param   GCPhysDst   Where to start reading from.
 * @param   cb          The number of bytes to attempt reading.
 * @param   fFlags      Flags, MBZ.
 * @param   pcbRead     For store the actual number of bytes read, pass NULL if
 *                      partial reads are unwanted.
 * @todo    Unused?
 */
VMMR3DECL(int) PGMR3DbgReadGCPhys(PVM pVM, void *pvDst, RTGCPHYS GCPhysSrc, size_t cb, uint32_t fFlags, size_t *pcbRead)
{
    /* validate */
    AssertReturn(!fFlags, VERR_INVALID_PARAMETER);
    AssertReturn(pVM, VERR_INVALID_PARAMETER);

    /* try simple first. */
    int rc = PGMPhysSimpleReadGCPhys(pVM, pvDst, GCPhysSrc, cb);
    if (RT_SUCCESS(rc) || !pcbRead)
        return rc;

    /* partial read that failed, chop it up in pages. */
    *pcbRead = 0;
    rc = VINF_SUCCESS;
    while (cb > 0)
    {
        size_t cbChunk = PAGE_SIZE;
        cbChunk -= GCPhysSrc & PAGE_OFFSET_MASK;
        if (cbChunk > cb)
            cbChunk = cb;

        rc = PGMPhysSimpleReadGCPhys(pVM, pvDst, GCPhysSrc, cbChunk);

        /* advance */
        if (RT_FAILURE(rc))
            break;
        *pcbRead  += cbChunk;
        cb        -= cbChunk;
        GCPhysSrc += cbChunk;
        pvDst = (uint8_t *)pvDst + cbChunk;
    }

    return *pcbRead && RT_FAILURE(rc) ? -rc : rc;
}


/**
 * Write physical memory API for the debugger, similar to
 * PGMPhysSimpleWriteGCPhys.
 *
 * @returns VBox status code.
 *
 * @param   pVM         Pointer to the VM.
 * @param   GCPhysDst   Where to start writing.
 * @param   pvSrc       What to write.
 * @param   cb          The number of bytes to attempt writing.
 * @param   fFlags      Flags, MBZ.
 * @param   pcbWritten  For store the actual number of bytes written, pass NULL
 *                      if partial writes are unwanted.
 * @todo    Unused?
 */
VMMR3DECL(int) PGMR3DbgWriteGCPhys(PVM pVM, RTGCPHYS GCPhysDst, const void *pvSrc, size_t cb, uint32_t fFlags, size_t *pcbWritten)
{
    /* validate */
    AssertReturn(!fFlags, VERR_INVALID_PARAMETER);
    AssertReturn(pVM, VERR_INVALID_PARAMETER);

    /* try simple first. */
    int rc = PGMPhysSimpleWriteGCPhys(pVM, GCPhysDst, pvSrc, cb);
    if (RT_SUCCESS(rc) || !pcbWritten)
        return rc;

    /* partial write that failed, chop it up in pages. */
    *pcbWritten = 0;
    rc = VINF_SUCCESS;
    while (cb > 0)
    {
        size_t cbChunk = PAGE_SIZE;
        cbChunk -= GCPhysDst & PAGE_OFFSET_MASK;
        if (cbChunk > cb)
            cbChunk = cb;

        rc = PGMPhysSimpleWriteGCPhys(pVM, GCPhysDst, pvSrc, cbChunk);

        /* advance */
        if (RT_FAILURE(rc))
            break;
        *pcbWritten += cbChunk;
        cb          -= cbChunk;
        GCPhysDst   += cbChunk;
        pvSrc = (uint8_t const *)pvSrc + cbChunk;
    }

    return *pcbWritten && RT_FAILURE(rc) ? -rc : rc;

}


/**
 * Read virtual memory API for the debugger, similar to PGMPhysSimpleReadGCPtr.
 *
 * @returns VBox status code.
 *
 * @param   pVM         Pointer to the VM.
 * @param   pvDst       Where to store what's read.
 * @param   GCPtrDst    Where to start reading from.
 * @param   cb          The number of bytes to attempt reading.
 * @param   fFlags      Flags, MBZ.
 * @param   pcbRead     For store the actual number of bytes read, pass NULL if
 *                      partial reads are unwanted.
 * @todo    Unused?
 */
VMMR3DECL(int) PGMR3DbgReadGCPtr(PVM pVM, void *pvDst, RTGCPTR GCPtrSrc, size_t cb, uint32_t fFlags, size_t *pcbRead)
{
    /* validate */
    AssertReturn(!fFlags, VERR_INVALID_PARAMETER);
    AssertReturn(pVM, VERR_INVALID_PARAMETER);

    /* @todo SMP support! */
    PVMCPU pVCpu = &pVM->aCpus[0];

/** @todo deal with HMA */
    /* try simple first. */
    int rc = PGMPhysSimpleReadGCPtr(pVCpu, pvDst, GCPtrSrc, cb);
    if (RT_SUCCESS(rc) || !pcbRead)
        return rc;

    /* partial read that failed, chop it up in pages. */
    *pcbRead = 0;
    rc = VINF_SUCCESS;
    while (cb > 0)
    {
        size_t cbChunk = PAGE_SIZE;
        cbChunk -= GCPtrSrc & PAGE_OFFSET_MASK;
        if (cbChunk > cb)
            cbChunk = cb;

        rc = PGMPhysSimpleReadGCPtr(pVCpu, pvDst, GCPtrSrc, cbChunk);

        /* advance */
        if (RT_FAILURE(rc))
            break;
        *pcbRead  += cbChunk;
        cb        -= cbChunk;
        GCPtrSrc  += cbChunk;
        pvDst = (uint8_t *)pvDst + cbChunk;
    }

    return *pcbRead && RT_FAILURE(rc) ? -rc : rc;

}


/**
 * Write virtual memory API for the debugger, similar to
 * PGMPhysSimpleWriteGCPtr.
 *
 * @returns VBox status code.
 *
 * @param   pVM         Pointer to the VM.
 * @param   GCPtrDst    Where to start writing.
 * @param   pvSrc       What to write.
 * @param   cb          The number of bytes to attempt writing.
 * @param   fFlags      Flags, MBZ.
 * @param   pcbWritten  For store the actual number of bytes written, pass NULL
 *                      if partial writes are unwanted.
 * @todo    Unused?
 */
VMMR3DECL(int) PGMR3DbgWriteGCPtr(PVM pVM, RTGCPTR GCPtrDst, void const *pvSrc, size_t cb, uint32_t fFlags, size_t *pcbWritten)
{
    /* validate */
    AssertReturn(!fFlags, VERR_INVALID_PARAMETER);
    AssertReturn(pVM, VERR_INVALID_PARAMETER);

    /* @todo SMP support! */
    PVMCPU pVCpu = &pVM->aCpus[0];

/** @todo deal with HMA */
    /* try simple first. */
    int rc = PGMPhysSimpleWriteGCPtr(pVCpu, GCPtrDst, pvSrc, cb);
    if (RT_SUCCESS(rc) || !pcbWritten)
        return rc;

    /* partial write that failed, chop it up in pages. */
    *pcbWritten = 0;
    rc = VINF_SUCCESS;
    while (cb > 0)
    {
        size_t cbChunk = PAGE_SIZE;
        cbChunk -= GCPtrDst & PAGE_OFFSET_MASK;
        if (cbChunk > cb)
            cbChunk = cb;

        rc = PGMPhysSimpleWriteGCPtr(pVCpu, GCPtrDst, pvSrc, cbChunk);

        /* advance */
        if (RT_FAILURE(rc))
            break;
        *pcbWritten += cbChunk;
        cb          -= cbChunk;
        GCPtrDst    += cbChunk;
        pvSrc = (uint8_t const *)pvSrc + cbChunk;
    }

    return *pcbWritten && RT_FAILURE(rc) ? -rc : rc;

}


/**
 * memchr() with alignment considerations.
 *
 * @returns Pointer to matching byte, NULL if none found.
 * @param   pb                  Where to search. Aligned.
 * @param   b                   What to search for.
 * @param   cb                  How much to search .
 * @param   uAlign              The alignment restriction of the result.
 */
static const uint8_t *pgmR3DbgAlignedMemChr(const uint8_t *pb, uint8_t b, size_t cb, uint32_t uAlign)
{
    const uint8_t *pbRet;
    if (uAlign <= 32)
    {
        pbRet = (const uint8_t *)memchr(pb, b, cb);
        if ((uintptr_t)pbRet & (uAlign - 1))
        {
            do
            {
                pbRet++;
                size_t cbLeft = cb - (pbRet - pb);
                if (!cbLeft)
                {
                    pbRet = NULL;
                    break;
                }
                pbRet = (const uint8_t *)memchr(pbRet, b, cbLeft);
            } while ((uintptr_t)pbRet & (uAlign - 1));
        }
    }
    else
    {
        pbRet = NULL;
        if (cb)
        {
            for (;;)
            {
                if (*pb == b)
                {
                    pbRet = pb;
                    break;
                }
                if (cb <= uAlign)
                    break;
                cb -= uAlign;
                pb += uAlign;
            }
        }
    }
    return pbRet;
}


/**
 * Scans a page for a byte string, keeping track of potential
 * cross page matches.
 *
 * @returns true and *poff on match.
 *          false on mismatch.
 * @param   pbPage          Pointer to the current page.
 * @param   poff            Input: The offset into the page (aligned).
 *                          Output: The page offset of the match on success.
 * @param   cb              The number of bytes to search, starting of *poff.
 * @param   uAlign          The needle alignment. This is of course less than a page.
 * @param   pabNeedle       The byte string to search for.
 * @param   cbNeedle        The length of the byte string.
 * @param   pabPrev         The buffer that keeps track of a partial match that we
 *                          bring over from the previous page. This buffer must be
 *                          at least cbNeedle - 1 big.
 * @param   pcbPrev         Input: The number of partial matching bytes from the previous page.
 *                          Output: The number of partial matching bytes from this page.
 *                          Initialize to 0 before the first call to this function.
 */
static bool pgmR3DbgScanPage(const uint8_t *pbPage, int32_t *poff, uint32_t cb, uint32_t uAlign,
                             const uint8_t *pabNeedle, size_t cbNeedle,
                             uint8_t *pabPrev, size_t *pcbPrev)
{
    /*
     * Try complete any partial match from the previous page.
     */
    if (*pcbPrev > 0)
    {
        size_t cbPrev = *pcbPrev;
        Assert(!*poff);
        Assert(cbPrev < cbNeedle);
        if (!memcmp(pbPage, pabNeedle + cbPrev, cbNeedle - cbPrev))
        {
            if (cbNeedle - cbPrev > cb)
                return false;
            *poff = -(int32_t)cbPrev;
            return true;
        }

        /* check out the remainder of the previous page. */
        const uint8_t *pb = pabPrev;
        for (;;)
        {
            if (cbPrev <= uAlign)
                break;
            cbPrev -= uAlign;
            pb = pgmR3DbgAlignedMemChr(pb + uAlign, *pabNeedle, cbPrev, uAlign);
            if (!pb)
                break;
            cbPrev = *pcbPrev - (pb - pabPrev);
            if (    !memcmp(pb + 1, &pabNeedle[1], cbPrev - 1)
                &&  !memcmp(pbPage, pabNeedle + cbPrev, cbNeedle - cbPrev))
            {
                if (cbNeedle - cbPrev > cb)
                    return false;
                *poff = -(int32_t)cbPrev;
                return true;
            }
        }

        *pcbPrev = 0;
    }

    /*
     * Match the body of the page.
     */
    const uint8_t *pb = pbPage + *poff;
    const uint8_t *pbEnd = pb + cb;
    for (;;)
    {
        pb = pgmR3DbgAlignedMemChr(pb, *pabNeedle, cb, uAlign);
        if (!pb)
            break;
        cb = pbEnd - pb;
        if (cb >= cbNeedle)
        {
            /* match? */
            if (!memcmp(pb + 1, &pabNeedle[1], cbNeedle - 1))
            {
                *poff = pb - pbPage;
                return true;
            }
        }
        else
        {
            /* partial match at the end of the page? */
            if (!memcmp(pb + 1, &pabNeedle[1], cb - 1))
            {
                /* We're copying one byte more that we really need here, but wtf. */
                memcpy(pabPrev, pb, cb);
                *pcbPrev = cb;
                return false;
            }
        }

        /* no match, skip ahead. */
        if (cb <= uAlign)
            break;
        pb += uAlign;
        cb -= uAlign;
    }

    return false;
}


/**
 * Scans guest physical memory for a byte string.
 *
 * @returns VBox status codes:
 * @retval  VINF_SUCCESS and *pGCPtrHit on success.
 * @retval  VERR_DBGF_MEM_NOT_FOUND if not found.
 * @retval  VERR_INVALID_POINTER if any of the pointer arguments are invalid.
 * @retval  VERR_INVALID_ARGUMENT if any other arguments are invalid.
 *
 * @param   pVM             Pointer to the VM.
 * @param   GCPhys          Where to start searching.
 * @param   cbRange         The number of bytes to search.
 * @param   GCPhysAlign     The alignment of the needle. Must be a power of two
 *                          and less or equal to 4GB.
 * @param   pabNeedle       The byte string to search for.
 * @param   cbNeedle        The length of the byte string. Max 256 bytes.
 * @param   pGCPhysHit      Where to store the address of the first occurrence on success.
 */
VMMR3DECL(int) PGMR3DbgScanPhysical(PVM pVM, RTGCPHYS GCPhys, RTGCPHYS cbRange, RTGCPHYS GCPhysAlign,
                                    const uint8_t *pabNeedle, size_t cbNeedle, PRTGCPHYS pGCPhysHit)
{
    /*
     * Validate and adjust the input a bit.
     */
    if (!VALID_PTR(pGCPhysHit))
        return VERR_INVALID_POINTER;
    *pGCPhysHit = NIL_RTGCPHYS;

    if (    !VALID_PTR(pabNeedle)
        ||  GCPhys == NIL_RTGCPHYS)
        return VERR_INVALID_POINTER;
    if (!cbNeedle)
        return VERR_INVALID_PARAMETER;
    if (cbNeedle > MAX_NEEDLE_SIZE)
        return VERR_INVALID_PARAMETER;

    if (!cbRange)
        return VERR_DBGF_MEM_NOT_FOUND;
    if (GCPhys + cbNeedle - 1 < GCPhys)
        return VERR_DBGF_MEM_NOT_FOUND;

    if (!GCPhysAlign)
        return VERR_INVALID_PARAMETER;
    if (GCPhysAlign > UINT32_MAX)
        return VERR_NOT_POWER_OF_TWO;
    if (GCPhysAlign & (GCPhysAlign - 1))
        return VERR_INVALID_PARAMETER;

    if (GCPhys & (GCPhysAlign - 1))
    {
        RTGCPHYS Adj = GCPhysAlign - (GCPhys & (GCPhysAlign - 1));
        if (    cbRange <= Adj
            ||  GCPhys + Adj < GCPhys)
            return VERR_DBGF_MEM_NOT_FOUND;
        GCPhys  += Adj;
        cbRange -= Adj;
    }

    const bool      fAllZero   = ASMMemIsAll8(pabNeedle, cbNeedle, 0) == NULL;
    const uint32_t  cIncPages  = GCPhysAlign <= PAGE_SIZE
                               ? 1
                               : GCPhysAlign >> PAGE_SHIFT;
    const RTGCPHYS  GCPhysLast = GCPhys + cbRange - 1 >= GCPhys
                               ? GCPhys + cbRange - 1
                               : ~(RTGCPHYS)0;

    /*
     * Search the memory - ignore MMIO and zero pages, also don't
     * bother to match across ranges.
     */
    pgmLock(pVM);
    for (PPGMRAMRANGE pRam = pVM->pgm.s.CTX_SUFF(pRamRangesX);
         pRam;
         pRam = pRam->CTX_SUFF(pNext))
    {
        /*
         * If the search range starts prior to the current ram range record,
         * adjust the search range and possibly conclude the search.
         */
        RTGCPHYS off;
        if (GCPhys < pRam->GCPhys)
        {
            if (GCPhysLast < pRam->GCPhys)
                break;
            GCPhys = pRam->GCPhys;
            off = 0;
        }
        else
            off = GCPhys - pRam->GCPhys;
        if (off < pRam->cb)
        {
            /*
             * Iterate the relevant pages.
             */
            uint8_t         abPrev[MAX_NEEDLE_SIZE];
            size_t          cbPrev   = 0;
            const uint32_t  cPages   = pRam->cb >> PAGE_SHIFT;
            uint32_t        iPage    = off >> PAGE_SHIFT;
            uint32_t        offPage  = GCPhys & PAGE_OFFSET_MASK;
            GCPhys &= ~(RTGCPHYS)PAGE_OFFSET_MASK;
            for (;; offPage = 0)
            {
                PPGMPAGE pPage = &pRam->aPages[iPage];
                if (    (   !PGM_PAGE_IS_ZERO(pPage)
                         || fAllZero)
                    &&  !PGM_PAGE_IS_BALLOONED(pPage)
                    &&  !PGM_PAGE_IS_MMIO(pPage))
                {
                    void const     *pvPage;
                    PGMPAGEMAPLOCK  Lock;
                    int rc = PGMPhysGCPhys2CCPtrReadOnly(pVM, GCPhys, &pvPage, &Lock);
                    if (RT_SUCCESS(rc))
                    {
                        int32_t     offHit = offPage;
                        bool        fRc;
                        if (GCPhysAlign < PAGE_SIZE)
                        {
                            uint32_t cbSearch = (GCPhys ^ GCPhysLast) & ~(RTGCPHYS)PAGE_OFFSET_MASK
                                              ? PAGE_SIZE                           - (uint32_t)offPage
                                              : (GCPhysLast & PAGE_OFFSET_MASK) + 1 - (uint32_t)offPage;
                            fRc = pgmR3DbgScanPage((uint8_t const *)pvPage, &offHit, cbSearch, (uint32_t)GCPhysAlign,
                                                   pabNeedle, cbNeedle, &abPrev[0], &cbPrev);
                        }
                        else
                            fRc = memcmp(pvPage, pabNeedle, cbNeedle) == 0
                               && (GCPhysLast - GCPhys) >= cbNeedle;
                        PGMPhysReleasePageMappingLock(pVM, &Lock);
                        if (fRc)
                        {
                            *pGCPhysHit = GCPhys + offHit;
                            pgmUnlock(pVM);
                            return VINF_SUCCESS;
                        }
                    }
                    else
                        cbPrev = 0; /* ignore error. */
                }
                else
                    cbPrev = 0;

                /* advance to the next page. */
                GCPhys += (RTGCPHYS)cIncPages << PAGE_SHIFT;
                if (GCPhys >= GCPhysLast) /* (may not always hit, but we're run out of ranges.) */
                {
                    pgmUnlock(pVM);
                    return VERR_DBGF_MEM_NOT_FOUND;
                }
                iPage += cIncPages;
                if (    iPage < cIncPages
                    ||  iPage >= cPages)
                    break;
            }
        }
    }
    pgmUnlock(pVM);
    return VERR_DBGF_MEM_NOT_FOUND;
}


/**
 * Scans (guest) virtual memory for a byte string.
 *
 * @returns VBox status codes:
 * @retval  VINF_SUCCESS and *pGCPtrHit on success.
 * @retval  VERR_DBGF_MEM_NOT_FOUND if not found.
 * @retval  VERR_INVALID_POINTER if any of the pointer arguments are invalid.
 * @retval  VERR_INVALID_ARGUMENT if any other arguments are invalid.
 *
 * @param   pVM             Pointer to the VM.
 * @param   pVCpu           The CPU context to search in.
 * @param   GCPtr           Where to start searching.
 * @param   GCPtrAlign      The alignment of the needle. Must be a power of two
 *                          and less or equal to 4GB.
 * @param   cbRange         The number of bytes to search. Max 256 bytes.
 * @param   pabNeedle       The byte string to search for.
 * @param   cbNeedle        The length of the byte string.
 * @param   pGCPtrHit       Where to store the address of the first occurrence on success.
 */
VMMR3DECL(int) PGMR3DbgScanVirtual(PVM pVM, PVMCPU pVCpu, RTGCPTR GCPtr, RTGCPTR cbRange, RTGCPTR GCPtrAlign,
                                   const uint8_t *pabNeedle, size_t cbNeedle, PRTGCUINTPTR pGCPtrHit)
{
    VMCPU_ASSERT_EMT(pVCpu);

    /*
     * Validate and adjust the input a bit.
     */
    if (!VALID_PTR(pGCPtrHit))
        return VERR_INVALID_POINTER;
    *pGCPtrHit = 0;

    if (!VALID_PTR(pabNeedle))
        return VERR_INVALID_POINTER;
    if (!cbNeedle)
        return VERR_INVALID_PARAMETER;
    if (cbNeedle > MAX_NEEDLE_SIZE)
        return VERR_INVALID_PARAMETER;

    if (!cbRange)
        return VERR_DBGF_MEM_NOT_FOUND;
    if (GCPtr + cbNeedle - 1 < GCPtr)
        return VERR_DBGF_MEM_NOT_FOUND;

    if (!GCPtrAlign)
        return VERR_INVALID_PARAMETER;
    if (GCPtrAlign > UINT32_MAX)
        return VERR_NOT_POWER_OF_TWO;
    if (GCPtrAlign & (GCPtrAlign - 1))
        return VERR_INVALID_PARAMETER;

    if (GCPtr & (GCPtrAlign - 1))
    {
        RTGCPTR Adj = GCPtrAlign - (GCPtr & (GCPtrAlign - 1));
        if (    cbRange <= Adj
            ||  GCPtr + Adj < GCPtr)
            return VERR_DBGF_MEM_NOT_FOUND;
        GCPtr   += Adj;
        cbRange -= Adj;
    }

    /*
     * Search the memory - ignore MMIO, zero and not-present pages.
     */
    const bool      fAllZero  = ASMMemIsAll8(pabNeedle, cbNeedle, 0) == NULL;
    PGMMODE         enmMode   = PGMGetGuestMode(pVCpu);
    RTGCPTR         GCPtrMask = PGMMODE_IS_LONG_MODE(enmMode) ? UINT64_MAX : UINT32_MAX;
    uint8_t         abPrev[MAX_NEEDLE_SIZE];
    size_t          cbPrev    = 0;
    const uint32_t  cIncPages = GCPtrAlign <= PAGE_SIZE
                              ? 1
                              : GCPtrAlign >> PAGE_SHIFT;
    const RTGCPTR   GCPtrLast = GCPtr + cbRange - 1 >= GCPtr
                              ? (GCPtr + cbRange - 1) & GCPtrMask
                              : GCPtrMask;
    RTGCPTR         cPages    = (((GCPtrLast - GCPtr) + (GCPtr & PAGE_OFFSET_MASK)) >> PAGE_SHIFT) + 1;
    uint32_t        offPage   = GCPtr & PAGE_OFFSET_MASK;
    GCPtr &= ~(RTGCPTR)PAGE_OFFSET_MASK;
    for (;; offPage = 0)
    {
        RTGCPHYS GCPhys;
        int rc = PGMPhysGCPtr2GCPhys(pVCpu, GCPtr, &GCPhys);
        if (RT_SUCCESS(rc))
        {
            PPGMPAGE pPage = pgmPhysGetPage(pVM, GCPhys);
            if (    pPage
                &&  (   !PGM_PAGE_IS_ZERO(pPage)
                     || fAllZero)
                &&  !PGM_PAGE_IS_BALLOONED(pPage)
                &&  !PGM_PAGE_IS_MMIO(pPage))
            {
                void const *pvPage;
                PGMPAGEMAPLOCK Lock;
                rc = PGMPhysGCPhys2CCPtrReadOnly(pVM, GCPhys, &pvPage, &Lock);
                if (RT_SUCCESS(rc))
                {
                    int32_t offHit = offPage;
                    bool    fRc;
                    if (GCPtrAlign < PAGE_SIZE)
                    {
                        uint32_t cbSearch = cPages > 0
                                          ? PAGE_SIZE                          - (uint32_t)offPage
                                          : (GCPtrLast & PAGE_OFFSET_MASK) + 1 - (uint32_t)offPage;
                        fRc = pgmR3DbgScanPage((uint8_t const *)pvPage, &offHit, cbSearch, (uint32_t)GCPtrAlign,
                                               pabNeedle, cbNeedle, &abPrev[0], &cbPrev);
                    }
                    else
                        fRc = memcmp(pvPage, pabNeedle, cbNeedle) == 0
                           && (GCPtrLast - GCPtr) >= cbNeedle;
                    PGMPhysReleasePageMappingLock(pVM, &Lock);
                    if (fRc)
                    {
                        *pGCPtrHit = GCPtr + offHit;
                        return VINF_SUCCESS;
                    }
                }
                else
                    cbPrev = 0; /* ignore error. */
            }
            else
                cbPrev = 0;
        }
        else
            cbPrev = 0; /* ignore error. */

        /* advance to the next page. */
        if (cPages <= cIncPages)
            break;
        cPages -= cIncPages;
        GCPtr += (RTGCPTR)cIncPages << PAGE_SHIFT;
    }
    return VERR_DBGF_MEM_NOT_FOUND;
}


/**
 * Initializes the dumper state.
 *
 * @param   pState          The state to initialize.
 * @param   pVM             Pointer to the VM.
 * @param   fFlags          The flags.
 * @param   u64FirstAddr    The first address.
 * @param   u64LastAddr     The last address.
 * @param   pHlp            The output helpers.
 */
static void pgmR3DumpHierarchyInitState(PPGMR3DUMPHIERARCHYSTATE pState, PVM pVM, uint32_t fFlags,
                                        uint64_t u64FirstAddr, uint64_t u64LastAddr, PCDBGFINFOHLP pHlp)
{
    pState->pVM                 = pVM;
    pState->pHlp                = pHlp ? pHlp : DBGFR3InfoLogHlp();
    pState->fPse                = !!(fFlags & (DBGFPGDMP_FLAGS_PSE | DBGFPGDMP_FLAGS_PAE | DBGFPGDMP_FLAGS_LME));
    pState->fPae                = !!(fFlags & (DBGFPGDMP_FLAGS_PAE | DBGFPGDMP_FLAGS_LME));
    pState->fLme                = !!(fFlags & DBGFPGDMP_FLAGS_LME);
    pState->fNp                 = !!(fFlags & DBGFPGDMP_FLAGS_NP);
    pState->fEpt                = !!(fFlags & DBGFPGDMP_FLAGS_EPT);
    pState->fNxe                = !!(fFlags & DBGFPGDMP_FLAGS_NXE);
    pState->cchAddress          = pState->fLme ? 16 : 8;
    pState->uLastRsvdBit        = pState->fNxe ? 62 : 63;
    pState->fDumpPageInfo       = !!(fFlags & DBGFPGDMP_FLAGS_PAGE_INFO);
    pState->fPrintHeader        = !!(fFlags & DBGFPGDMP_FLAGS_HEADER);
    pState->fPrintCr3           = !!(fFlags & DBGFPGDMP_FLAGS_PRINT_CR3);
    pState->afReserved[0]       = false;
    pState->afReserved[1]       = false;
    pState->afReserved[2]       = false;
    pState->afReserved[3]       = false;
    pState->afReserved[4]       = false;
    pState->u64Address          = u64FirstAddr;
    pState->u64FirstAddress     = u64FirstAddr;
    pState->u64LastAddress      = u64LastAddr;
    pState->u64HighReservedBits = pState->uLastRsvdBit == 62 ? UINT64_C(0x7ff) << 52 : UINT64_C(0xfff) << 52;
    pState->cLeaves             = 0;
}


/**
 * The simple way out, too tired to think of a more elegant solution.
 *
 * @returns The base address of this page table/directory/whatever.
 * @param   pState              The state where we get the current address.
 * @param   cShift              The shift count for the table entries.
 * @param   cEntries            The number of table entries.
 * @param   piFirst             Where to return the table index of the first
 *                              entry to dump.
 * @param   piLast              Where to return the table index of the last
 *                              entry.
 */
static uint64_t pgmR3DumpHierarchyCalcRange(PPGMR3DUMPHIERARCHYSTATE pState, uint32_t cShift, uint32_t cEntries,
                                            uint32_t *piFirst, uint32_t *piLast)
{
    const uint64_t iBase  = (pState->u64Address     >> cShift) & ~(uint64_t)(cEntries - 1);
    const uint64_t iFirst = pState->u64FirstAddress >> cShift;
    const uint64_t iLast  = pState->u64LastAddress  >> cShift;

    if (   iBase                >= iFirst
        && iBase + cEntries - 1 <= iLast)
    {
        /* full range. */
        *piFirst = 0;
        *piLast  = cEntries - 1;
    }
    else if (   iBase + cEntries - 1 < iFirst
             || iBase                > iLast)
    {
        /* no match */
        *piFirst = cEntries;
        *piLast  = 0;
    }
    else
    {
        /* partial overlap */
        *piFirst = iBase <= iFirst
                 ? iFirst - iBase
                 : 0;
        *piLast  = iBase + cEntries - 1 <= iLast
                 ? cEntries - 1
                 : iLast - iBase;
    }

    return iBase << cShift;
}


/**
 * Maps/finds the shadow page.
 *
 * @returns VBox status code.
 * @param   pState              The dumper state.
 * @param   HCPhys              The physical address of the shadow page.
 * @param   pszDesc             The description.
 * @param   fIsMapping          Set if it's a mapping.
 * @param   ppv                 Where to return the pointer.
 */
static int pgmR3DumpHierarchyShwMapPage(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS HCPhys, const char *pszDesc,
                                        bool fIsMapping, void const **ppv)
{
    void *pvPage;
    if (!fIsMapping)
    {
        int rc = MMPagePhys2PageTry(pState->pVM, HCPhys, &pvPage);
        if (RT_FAILURE(rc))
        {
            pState->pHlp->pfnPrintf(pState->pHlp, "%0*llx error! %s at HCPhys=%RHp was not found in the page pool!\n",
                                    pState->cchAddress, pState->u64Address, pszDesc, HCPhys);
            return rc;
        }
    }
    else
    {
        pvPage = NULL;
        for (PPGMMAPPING pMap = pState->pVM->pgm.s.pMappingsR3; pMap; pMap = pMap->pNextR3)
        {
            uint64_t off = pState->u64Address - pMap->GCPtr;
            if (off < pMap->cb)
            {
                const int iPDE = (uint32_t)(off >> X86_PD_SHIFT);
                const int iSub = (int)((off >> X86_PD_PAE_SHIFT) & 1); /* MSC is a pain sometimes */
                if ((iSub ? pMap->aPTs[iPDE].HCPhysPaePT1 : pMap->aPTs[iPDE].HCPhysPaePT0) != HCPhys)
                    pState->pHlp->pfnPrintf(pState->pHlp,
                                            "%0*llx error! Mapping error! PT %d has HCPhysPT=%RHp not %RHp is in the PD.\n",
                                            pState->cchAddress, pState->u64Address, iPDE,
                                            iSub ? pMap->aPTs[iPDE].HCPhysPaePT1 : pMap->aPTs[iPDE].HCPhysPaePT0, HCPhys);
                pvPage = &pMap->aPTs[iPDE].paPaePTsR3[iSub];
                break;
            }
        }
        if (!pvPage)
        {
            pState->pHlp->pfnPrintf(pState->pHlp, "%0*llx error! PT mapping %s at HCPhys=%RHp was not found in the page pool!\n",
                                    pState->cchAddress, pState->u64Address, HCPhys);
            return VERR_INVALID_PARAMETER;
        }
    }
    *ppv = pvPage;
    return VINF_SUCCESS;
}


/**
 * Dumps the a shadow page summary or smth.
 *
 * @param   pState              The dumper state.
 * @param   HCPhys              The page address.
 */
static void pgmR3DumpHierarchyShwTablePageInfo(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS HCPhys)
{
    pgmLock(pState->pVM);
    char            szPage[80];
    PPGMPOOLPAGE    pPage = pgmPoolQueryPageForDbg(pState->pVM->pgm.s.CTX_SUFF(pPool), HCPhys);
    if (pPage)
        RTStrPrintf(szPage, sizeof(szPage), " idx=0i%u", pPage->idx);
    else
    {
        /* probably a mapping */
        strcpy(szPage, " not found");
        for (PPGMMAPPING pMap = pState->pVM->pgm.s.pMappingsR3; pMap; pMap = pMap->pNextR3)
        {
            uint64_t off = pState->u64Address - pMap->GCPtr;
            if (off < pMap->cb)
            {
                const int iPDE = (uint32_t)(off >> X86_PD_SHIFT);
                if (pMap->aPTs[iPDE].HCPhysPT == HCPhys)
                    RTStrPrintf(szPage, sizeof(szPage), " #%u: %s", iPDE, pMap->pszDesc);
                else if (pMap->aPTs[iPDE].HCPhysPaePT0 == HCPhys)
                    RTStrPrintf(szPage, sizeof(szPage), " #%u/0: %s", iPDE, pMap->pszDesc);
                else if (pMap->aPTs[iPDE].HCPhysPaePT1 == HCPhys)
                    RTStrPrintf(szPage, sizeof(szPage), " #%u/1: %s", iPDE, pMap->pszDesc);
                else
                    continue;
                break;
            }
        }
    }
    pgmUnlock(pState->pVM);
    pState->pHlp->pfnPrintf(pState->pHlp, "%s", szPage);
}


/**
 * Figures out which guest page this is and dumps a summary.
 *
 * @param   pState              The dumper state.
 * @param   HCPhys              The page address.
 * @param   cbPage              The page size.
 */
static void pgmR3DumpHierarchyShwGuestPageInfo(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS HCPhys, uint32_t cbPage)
{
    char        szPage[80];
    RTGCPHYS    GCPhys;
    int rc = PGMR3DbgHCPhys2GCPhys(pState->pVM, HCPhys, &GCPhys);
    if (RT_SUCCESS(rc))
    {
        pgmLock(pState->pVM);
        PCPGMPAGE pPage = pgmPhysGetPage(pState->pVM, GCPhys);
        if (pPage)
            RTStrPrintf(szPage, sizeof(szPage), "%R[pgmpage]", pPage);
        else
            strcpy(szPage, "not found");
        pgmUnlock(pState->pVM);
        pState->pHlp->pfnPrintf(pState->pHlp, " -> %RGp %s", GCPhys, szPage);
    }
    else
    {
        /* check the heap */
        uint32_t cbAlloc;
        rc = MMR3HyperQueryInfoFromHCPhys(pState->pVM, HCPhys, szPage, sizeof(szPage), &cbAlloc);
        if (RT_SUCCESS(rc))
            pState->pHlp->pfnPrintf(pState->pHlp, " %s %#x bytes", szPage, cbAlloc);
        else
            pState->pHlp->pfnPrintf(pState->pHlp, " not found");
    }
    NOREF(cbPage);
}


/**
 * Dumps a PAE shadow page table.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pState              The dumper state.
 * @param   HCPhys              The page table address.
 * @param   fIsMapping          Whether it is a mapping.
 */
static int pgmR3DumpHierarchyShwPaePT(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS HCPhys, bool fIsMapping)
{
    PCPGMSHWPTPAE pPT;
    int rc = pgmR3DumpHierarchyShwMapPage(pState, HCPhys, "Page table", fIsMapping, (void const **)&pPT);
    if (RT_FAILURE(rc))
        return rc;

    uint32_t iFirst, iLast;
    uint64_t u64BaseAddress = pgmR3DumpHierarchyCalcRange(pState, X86_PT_PAE_SHIFT, X86_PG_PAE_ENTRIES, &iFirst, &iLast);
    for (uint32_t i = iFirst; i <= iLast; i++)
        if (PGMSHWPTEPAE_GET_U(pPT->a[i]) & X86_PTE_P)
        {
            pState->u64Address = u64BaseAddress + ((uint64_t)i << X86_PT_PAE_SHIFT);
            if (PGMSHWPTEPAE_IS_P(pPT->a[i]))
            {
                X86PTEPAE Pte;
                Pte.u = PGMSHWPTEPAE_GET_U(pPT->a[i]);
                pState->pHlp->pfnPrintf(pState->pHlp,
                                        pState->fLme  /*P R  S  A  D  G  WT CD AT NX 4M a p ?  */
                                        ? "%016llx 3    | P %c %c %c %c %c %s %s %s %s 4K %c%c%c  %016llx"
                                        :  "%08llx 2   |  P %c %c %c %c %c %s %s %s %s 4K %c%c%c  %016llx",
                                        pState->u64Address,
                                        Pte.n.u1Write       ? 'W'  : 'R',
                                        Pte.n.u1User        ? 'U'  : 'S',
                                        Pte.n.u1Accessed    ? 'A'  : '-',
                                        Pte.n.u1Dirty       ? 'D'  : '-',
                                        Pte.n.u1Global      ? 'G'  : '-',
                                        Pte.n.u1WriteThru   ? "WT" : "--",
                                        Pte.n.u1CacheDisable? "CD" : "--",
                                        Pte.n.u1PAT         ? "AT" : "--",
                                        Pte.n.u1NoExecute   ? "NX" : "--",
                                        Pte.u & PGM_PTFLAGS_TRACK_DIRTY   ? 'd' : '-',
                                        Pte.u & RT_BIT(10)                ? '1' : '0',
                                        Pte.u & PGM_PTFLAGS_CSAM_VALIDATED? 'v' : '-',
                                        Pte.u & X86_PTE_PAE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyShwGuestPageInfo(pState, Pte.u & X86_PTE_PAE_PG_MASK, _4K);
                if ((Pte.u >> 52) & 0x7ff)
                    pState->pHlp->pfnPrintf(pState->pHlp, " 62:52=%03llx%s", (Pte.u >> 52) & 0x7ff, pState->fLme ? "" : "!");
                pState->pHlp->pfnPrintf(pState->pHlp, "\n");
            }
            else if (   (PGMSHWPTEPAE_GET_U(pPT->a[i]) & (pState->pVM->pgm.s.HCPhysInvMmioPg | X86_PTE_PAE_MBZ_MASK_NO_NX))
                     ==                                  (pState->pVM->pgm.s.HCPhysInvMmioPg | X86_PTE_PAE_MBZ_MASK_NO_NX))
                pState->pHlp->pfnPrintf(pState->pHlp,
                                        pState->fLme
                                        ? "%016llx 3    | invalid / MMIO optimization\n"
                                        :  "%08llx 2   |  invalid / MMIO optimization\n",
                                        pState->u64Address);
            else
                pState->pHlp->pfnPrintf(pState->pHlp,
                                        pState->fLme
                                        ? "%016llx 3    | invalid: %RX64\n"
                                        :  "%08llx 2   |  invalid: %RX64\n",
                                        pState->u64Address, PGMSHWPTEPAE_GET_U(pPT->a[i]));
            pState->cLeaves++;
        }
    return VINF_SUCCESS;
}


/**
 * Dumps a PAE shadow page directory table.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pState      The dumper state.
 * @param   HCPhys      The physical address of the page directory table.
 * @param   cMaxDepth   The maximum depth.
 */
static int  pgmR3DumpHierarchyShwPaePD(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS HCPhys, unsigned cMaxDepth)
{
    PCX86PDPAE pPD;
    int rc = pgmR3DumpHierarchyShwMapPage(pState, HCPhys, "Page directory", false, (void const **)&pPD);
    if (RT_FAILURE(rc))
        return rc;

    Assert(cMaxDepth > 0);
    cMaxDepth--;

    uint32_t iFirst, iLast;
    uint64_t u64BaseAddress = pgmR3DumpHierarchyCalcRange(pState, X86_PD_PAE_SHIFT, X86_PG_PAE_ENTRIES, &iFirst, &iLast);
    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PDEPAE Pde = pPD->a[i];
        if (Pde.n.u1Present)
        {
            pState->u64Address = u64BaseAddress + ((uint64_t)i << X86_PD_PAE_SHIFT);
            if (Pde.b.u1Size)
            {
                pState->pHlp->pfnPrintf(pState->pHlp,
                                        pState->fLme    /*P R  S  A  D  G  WT CD AT NX 2M a  p ?  phys*/
                                        ? "%016llx 2   |  P %c %c %c %c %c %s %s %s %s 2M %c%c%c  %016llx"
                                        :  "%08llx 1  |   P %c %c %c %c %c %s %s %s %s 2M %c%c%c  %016llx",
                                        pState->u64Address,
                                        Pde.b.u1Write       ? 'W'  : 'R',
                                        Pde.b.u1User        ? 'U'  : 'S',
                                        Pde.b.u1Accessed    ? 'A'  : '-',
                                        Pde.b.u1Dirty       ? 'D'  : '-',
                                        Pde.b.u1Global      ? 'G'  : '-',
                                        Pde.b.u1WriteThru   ? "WT" : "--",
                                        Pde.b.u1CacheDisable? "CD" : "--",
                                        Pde.b.u1PAT         ? "AT" : "--",
                                        Pde.b.u1NoExecute   ? "NX" : "--",
                                        Pde.u & RT_BIT_64(9)            ? '1' : '0',
                                        Pde.u & PGM_PDFLAGS_MAPPING     ? 'm' : '-',
                                        Pde.u & PGM_PDFLAGS_TRACK_DIRTY ? 'd' : '-',
                                        Pde.u & X86_PDE2M_PAE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyShwGuestPageInfo(pState, Pde.u & X86_PDE2M_PAE_PG_MASK, _2M);
                if ((Pde.u >> 52) & 0x7ff)
                    pState->pHlp->pfnPrintf(pState->pHlp, " 62:52=%03llx%s", (Pde.u >> 52) & 0x7ff, pState->fLme ? "" : "!");
                if ((Pde.u >> 13) & 0xff)
                    pState->pHlp->pfnPrintf(pState->pHlp, " 20:13=%02llx%s", (Pde.u >> 13) & 0x0ff, pState->fLme ? "" : "!");
                pState->pHlp->pfnPrintf(pState->pHlp, "\n");

                pState->cLeaves++;
            }
            else
            {
                pState->pHlp->pfnPrintf(pState->pHlp,
                                        pState->fLme    /*P R  S  A  D  G  WT CD AT NX 4M a  p ?  phys */
                                        ? "%016llx 2   |  P %c %c %c %c %c %s %s .. %s .. %c%c%c  %016llx"
                                        :  "%08llx 1  |   P %c %c %c %c %c %s %s .. %s .. %c%c%c  %016llx",
                                        pState->u64Address,
                                        Pde.n.u1Write       ? 'W'  : 'R',
                                        Pde.n.u1User        ? 'U'  : 'S',
                                        Pde.n.u1Accessed    ? 'A'  : '-',
                                        Pde.n.u1Reserved0   ? '?'  : '.', /* ignored */
                                        Pde.n.u1Reserved1   ? '?'  : '.', /* ignored */
                                        Pde.n.u1WriteThru   ? "WT" : "--",
                                        Pde.n.u1CacheDisable? "CD" : "--",
                                        Pde.n.u1NoExecute   ? "NX" : "--",
                                        Pde.u & RT_BIT_64(9)            ? '1' : '0',
                                        Pde.u & PGM_PDFLAGS_MAPPING     ? 'm' : '-',
                                        Pde.u & PGM_PDFLAGS_TRACK_DIRTY ? 'd' : '-',
                                        Pde.u & X86_PDE_PAE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyShwTablePageInfo(pState, Pde.u & X86_PDE_PAE_PG_MASK);
                if ((Pde.u >> 52) & 0x7ff)
                    pState->pHlp->pfnPrintf(pState->pHlp, " 62:52=%03llx!", (Pde.u >> 52) & 0x7ff);
                pState->pHlp->pfnPrintf(pState->pHlp, "\n");

                if (cMaxDepth)
                {
                    int rc2 = pgmR3DumpHierarchyShwPaePT(pState, Pde.u & X86_PDE_PAE_PG_MASK, !!(Pde.u & PGM_PDFLAGS_MAPPING));
                    if (rc2 < rc && RT_SUCCESS(rc))
                        rc = rc2;
                }
                else
                    pState->cLeaves++;
            }
        }
    }
    return rc;
}


/**
 * Dumps a PAE shadow page directory pointer table.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pState      The dumper state.
 * @param   HCPhys      The physical address of the page directory pointer table.
 * @param   cMaxDepth   The maximum depth.
 */
static int  pgmR3DumpHierarchyShwPaePDPT(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS HCPhys, unsigned cMaxDepth)
{
    /* Fend of addresses that are out of range in PAE mode - simplifies the code below. */
    if (!pState->fLme && pState->u64Address >= _4G)
        return VINF_SUCCESS;

    PCX86PDPT pPDPT;
    int rc = pgmR3DumpHierarchyShwMapPage(pState, HCPhys, "Page directory pointer table", false, (void const **)&pPDPT);
    if (RT_FAILURE(rc))
        return rc;

    Assert(cMaxDepth > 0);
    cMaxDepth--;

    uint32_t iFirst, iLast;
    uint64_t u64BaseAddress = pgmR3DumpHierarchyCalcRange(pState, X86_PDPT_SHIFT,
                                                          pState->fLme ? X86_PG_AMD64_PDPE_ENTRIES : X86_PG_PAE_PDPE_ENTRIES,
                                                          &iFirst, &iLast);
    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PDPE Pdpe = pPDPT->a[i];
        if (Pdpe.n.u1Present)
        {
            pState->u64Address = u64BaseAddress + ((uint64_t)i << X86_PDPT_SHIFT);
            if (pState->fLme)
            {
                pState->pHlp->pfnPrintf(pState->pHlp, /*P R  S  A  D  G  WT CD AT NX .. a p ?  */
                                        "%016llx 1  |   P %c %c %c %c %c %s %s %s %s .. %c%c%c  %016llx",
                                        pState->u64Address,
                                        Pdpe.lm.u1Write       ? 'W'  : 'R',
                                        Pdpe.lm.u1User        ? 'U'  : 'S',
                                        Pdpe.lm.u1Accessed    ? 'A'  : '-',
                                        Pdpe.lm.u3Reserved & 1? '?'  : '.', /* ignored */
                                        Pdpe.lm.u3Reserved & 4? '!'  : '.', /* mbz */
                                        Pdpe.lm.u1WriteThru   ? "WT" : "--",
                                        Pdpe.lm.u1CacheDisable? "CD" : "--",
                                        Pdpe.lm.u3Reserved & 2? "!"  : "..",/* mbz */
                                        Pdpe.lm.u1NoExecute   ? "NX" : "--",
                                        Pdpe.u & RT_BIT(9)                ? '1' : '0',
                                        Pdpe.u & PGM_PLXFLAGS_PERMANENT   ? 'p' : '-',
                                        Pdpe.u & RT_BIT(11)               ? '1' : '0',
                                        Pdpe.u & X86_PDPE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyShwTablePageInfo(pState, Pdpe.u & X86_PDPE_PG_MASK);
                if ((Pdpe.u >> 52) & 0x7ff)
                    pState->pHlp->pfnPrintf(pState->pHlp, " 62:52=%03llx", (Pdpe.u >> 52) & 0x7ff);
            }
            else
            {
                pState->pHlp->pfnPrintf(pState->pHlp,/*P R  S  A  D  G  WT CD AT NX .. a p ?  */
                                        "%08llx 0 |    P %c %c %c %c %c %s %s %s %s .. %c%c%c  %016llx",
                                        pState->u64Address,
                                        Pdpe.n.u2Reserved & 1? '!'  : '.', /* mbz */
                                        Pdpe.n.u2Reserved & 2? '!'  : '.', /* mbz */
                                        Pdpe.n.u4Reserved & 1? '!'  : '.', /* mbz */
                                        Pdpe.n.u4Reserved & 2? '!'  : '.', /* mbz */
                                        Pdpe.n.u4Reserved & 8? '!'  : '.', /* mbz */
                                        Pdpe.n.u1WriteThru   ? "WT" : "--",
                                        Pdpe.n.u1CacheDisable? "CD" : "--",
                                        Pdpe.n.u4Reserved & 2? "!"  : "..",/* mbz */
                                        Pdpe.lm.u1NoExecute  ? "!!"  : "..",/* mbz */
                                        Pdpe.u & RT_BIT(9)                ? '1' : '0',
                                        Pdpe.u & PGM_PLXFLAGS_PERMANENT   ? 'p' : '-',
                                        Pdpe.u & RT_BIT(11)               ? '1' : '0',
                                        Pdpe.u & X86_PDPE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyShwTablePageInfo(pState, Pdpe.u & X86_PDPE_PG_MASK);
                if ((Pdpe.u >> 52) & 0xfff)
                    pState->pHlp->pfnPrintf(pState->pHlp, " 63:52=%03llx!", (Pdpe.u >> 52) & 0xfff);
            }
            pState->pHlp->pfnPrintf(pState->pHlp, "\n");

            if (cMaxDepth)
            {
                int rc2 = pgmR3DumpHierarchyShwPaePD(pState, Pdpe.u & X86_PDPE_PG_MASK, cMaxDepth);
                if (rc2 < rc && RT_SUCCESS(rc))
                    rc = rc2;
            }
            else
                pState->cLeaves++;
        }
    }
    return rc;
}


/**
 * Dumps a 32-bit shadow page table.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pVM         Pointer to the VM.
 * @param   HCPhys      The physical address of the table.
 * @param   cMaxDepth   The maximum depth.
 */
static int pgmR3DumpHierarchyShwPaePML4(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS HCPhys, unsigned cMaxDepth)
{
    PCX86PML4 pPML4;
    int rc = pgmR3DumpHierarchyShwMapPage(pState, HCPhys, "Page map level 4", false, (void const **)&pPML4);
    if (RT_FAILURE(rc))
        return rc;

    Assert(cMaxDepth);
    cMaxDepth--;

    /*
     * This is a bit tricky as we're working on unsigned addresses while the
     * AMD64 spec uses signed tricks.
     */
    uint32_t iFirst = (pState->u64FirstAddress >> X86_PML4_SHIFT) & X86_PML4_MASK;
    uint32_t iLast  = (pState->u64LastAddress  >> X86_PML4_SHIFT) & X86_PML4_MASK;
    if (   pState->u64LastAddress  <= UINT64_C(0x00007fffffffffff)
        || pState->u64FirstAddress >= UINT64_C(0xffff800000000000))
    { /* Simple, nothing to adjust */ }
    else if (pState->u64FirstAddress <= UINT64_C(0x00007fffffffffff))
        iLast = X86_PG_AMD64_ENTRIES / 2 - 1;
    else if (pState->u64LastAddress  >= UINT64_C(0xffff800000000000))
        iFirst = X86_PG_AMD64_ENTRIES / 2;
    else
        iFirst = X86_PG_AMD64_ENTRIES; /* neither address is canonical */

    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PML4E Pml4e = pPML4->a[i];
        if (Pml4e.n.u1Present)
        {
            pState->u64Address = ((uint64_t)i << X86_PML4_SHIFT)
                               | (i >= RT_ELEMENTS(pPML4->a) / 2 ? UINT64_C(0xffff000000000000) : 0);
            pState->pHlp->pfnPrintf(pState->pHlp, /*P R  S  A  D  G  WT CD AT NX 4M a p ?  */
                                    "%016llx 0 |    P %c %c %c %c %c %s %s %s %s .. %c%c%c  %016llx",
                                    pState->u64Address,
                                    Pml4e.n.u1Write       ? 'W'  : 'R',
                                    Pml4e.n.u1User        ? 'U'  : 'S',
                                    Pml4e.n.u1Accessed    ? 'A'  : '-',
                                    Pml4e.n.u3Reserved & 1? '?'  : '.', /* ignored */
                                    Pml4e.n.u3Reserved & 4? '!'  : '.', /* mbz */
                                    Pml4e.n.u1WriteThru   ? "WT" : "--",
                                    Pml4e.n.u1CacheDisable? "CD" : "--",
                                    Pml4e.n.u3Reserved & 2? "!"  : "..",/* mbz */
                                    Pml4e.n.u1NoExecute   ? "NX" : "--",
                                    Pml4e.u & RT_BIT(9)                ? '1' : '0',
                                    Pml4e.u & PGM_PLXFLAGS_PERMANENT   ? 'p' : '-',
                                    Pml4e.u & RT_BIT(11)               ? '1' : '0',
                                    Pml4e.u & X86_PML4E_PG_MASK);
            if (pState->fDumpPageInfo)
                pgmR3DumpHierarchyShwTablePageInfo(pState, Pml4e.u & X86_PML4E_PG_MASK);
            if ((Pml4e.u >> 52) & 0x7ff)
                pState->pHlp->pfnPrintf(pState->pHlp, " 62:52=%03llx!", (Pml4e.u >> 52) & 0x7ff);
            pState->pHlp->pfnPrintf(pState->pHlp, "\n");

            if (cMaxDepth)
            {
                int rc2 = pgmR3DumpHierarchyShwPaePDPT(pState, Pml4e.u & X86_PML4E_PG_MASK, cMaxDepth);
                if (rc2 < rc && RT_SUCCESS(rc))
                    rc = rc2;
            }
            else
                pState->cLeaves++;
        }
    }
    return rc;
}


/**
 * Dumps a 32-bit shadow page table.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pVM         Pointer to the VM.
 * @param   pPT         Pointer to the page table.
 * @param   fMapping    Set if it's a guest mapping.
 */
static int pgmR3DumpHierarchyShw32BitPT(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS HCPhys, bool fMapping)
{
    PCX86PT pPT;
    int rc = pgmR3DumpHierarchyShwMapPage(pState, HCPhys, "Page table", fMapping, (void const **)&pPT);
    if (RT_FAILURE(rc))
        return rc;

    uint32_t iFirst, iLast;
    uint64_t u64BaseAddress = pgmR3DumpHierarchyCalcRange(pState, X86_PT_SHIFT, X86_PG_ENTRIES, &iFirst, &iLast);
    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PTE Pte = pPT->a[i];
        if (Pte.n.u1Present)
        {
            pState->u64Address = u64BaseAddress + (i << X86_PT_SHIFT);
            pState->pHlp->pfnPrintf(pState->pHlp,/*P R  S  A  D  G  WT CD AT NX 4M a m d  */
                                    "%08llx 1  |   P %c %c %c %c %c %s %s %s .. 4K %c%c%c  %08x",
                                    pState->u64Address,
                                    Pte.n.u1Write       ? 'W'  : 'R',
                                    Pte.n.u1User        ? 'U'  : 'S',
                                    Pte.n.u1Accessed    ? 'A'  : '-',
                                    Pte.n.u1Dirty       ? 'D'  : '-',
                                    Pte.n.u1Global      ? 'G'  : '-',
                                    Pte.n.u1WriteThru   ? "WT" : "--",
                                    Pte.n.u1CacheDisable? "CD" : "--",
                                    Pte.n.u1PAT         ? "AT" : "--",
                                    Pte.u & PGM_PTFLAGS_TRACK_DIRTY     ? 'd' : '-',
                                    Pte.u & RT_BIT(10)                  ? '1' : '0',
                                    Pte.u & PGM_PTFLAGS_CSAM_VALIDATED  ? 'v' : '-',
                                    Pte.u & X86_PDE_PG_MASK);
            if (pState->fDumpPageInfo)
                pgmR3DumpHierarchyShwGuestPageInfo(pState, Pte.u & X86_PDE_PG_MASK, _4K);
            pState->pHlp->pfnPrintf(pState->pHlp, "\n");
        }
    }
    return VINF_SUCCESS;
}


/**
 * Dumps a 32-bit shadow page directory and page tables.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pState      The dumper state.
 * @param   HCPhys      The physical address of the table.
 * @param   cMaxDepth   The maximum depth.
 */
static int pgmR3DumpHierarchyShw32BitPD(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS HCPhys, unsigned cMaxDepth)
{
    if (pState->u64Address >= _4G)
        return VINF_SUCCESS;

    PCX86PD pPD;
    int rc = pgmR3DumpHierarchyShwMapPage(pState, HCPhys, "Page directory", false, (void const **)&pPD);
    if (RT_FAILURE(rc))
        return rc;

    Assert(cMaxDepth > 0);
    cMaxDepth--;

    uint32_t iFirst, iLast;
    pgmR3DumpHierarchyCalcRange(pState, X86_PD_SHIFT, X86_PG_ENTRIES, &iFirst, &iLast);
    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PDE Pde = pPD->a[i];
        if (Pde.n.u1Present)
        {
            pState->u64Address = (uint32_t)i << X86_PD_SHIFT;
            if (Pde.b.u1Size && pState->fPse)
            {
                uint64_t u64Phys = ((uint64_t)(Pde.u & X86_PDE4M_PG_HIGH_MASK) << X86_PDE4M_PG_HIGH_SHIFT)
                                 | (Pde.u & X86_PDE4M_PG_MASK);
                pState->pHlp->pfnPrintf(pState->pHlp,/*P R  S  A  D  G  WT CD AT NX 4M a m d   phys */
                                        "%08llx 0 |    P %c %c %c %c %c %s %s %s .. 4M %c%c%c  %08llx",
                                        pState->u64Address,
                                        Pde.b.u1Write       ? 'W'  : 'R',
                                        Pde.b.u1User        ? 'U'  : 'S',
                                        Pde.b.u1Accessed    ? 'A'  : '-',
                                        Pde.b.u1Dirty       ? 'D'  : '-',
                                        Pde.b.u1Global      ? 'G'  : '-',
                                        Pde.b.u1WriteThru   ? "WT" : "--",
                                        Pde.b.u1CacheDisable? "CD" : "--",
                                        Pde.b.u1PAT         ? "AT" : "--",
                                        Pde.u & RT_BIT_32(9)            ? '1' : '0',
                                        Pde.u & PGM_PDFLAGS_MAPPING     ? 'm' : '-',
                                        Pde.u & PGM_PDFLAGS_TRACK_DIRTY ? 'd' : '-',
                                        u64Phys);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyShwGuestPageInfo(pState, u64Phys, _4M);
                pState->pHlp->pfnPrintf(pState->pHlp, "\n");
                pState->cLeaves++;
            }
            else
            {
                pState->pHlp->pfnPrintf(pState->pHlp,/*P R  S  A  D  G  WT CD AT NX 4M a m d   phys */
                                        "%08llx 0 |    P %c %c %c %c %c %s %s .. .. 4K %c%c%c  %08x",
                                        pState->u64Address,
                                        Pde.n.u1Write       ? 'W'  : 'R',
                                        Pde.n.u1User        ? 'U'  : 'S',
                                        Pde.n.u1Accessed    ? 'A'  : '-',
                                        Pde.n.u1Reserved0   ? '?'  : '.', /* ignored */
                                        Pde.n.u1Reserved1   ? '?'  : '.', /* ignored */
                                        Pde.n.u1WriteThru   ? "WT" : "--",
                                        Pde.n.u1CacheDisable? "CD" : "--",
                                        Pde.u & RT_BIT_32(9)            ? '1' : '0',
                                        Pde.u & PGM_PDFLAGS_MAPPING     ? 'm' : '-',
                                        Pde.u & PGM_PDFLAGS_TRACK_DIRTY ? 'd' : '-',
                                        Pde.u & X86_PDE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyShwTablePageInfo(pState, Pde.u & X86_PDE_PG_MASK);
                pState->pHlp->pfnPrintf(pState->pHlp, "\n");

                if (cMaxDepth)
                {
                    int rc2 = pgmR3DumpHierarchyShw32BitPT(pState, Pde.u & X86_PDE_PG_MASK, !!(Pde.u & PGM_PDFLAGS_MAPPING));
                    if (rc2 < rc && RT_SUCCESS(rc))
                        rc = rc2;
                }
                else
                    pState->cLeaves++;
            }
        }
    }

    return rc;
}


/**
 * Internal worker that initiates the actual dump.
 *
 * @returns VBox status code.
 * @param   pState              The dumper state.
 * @param   cr3                 The CR3 value.
 * @param   cMaxDepth           The max depth.
 */
static int pgmR3DumpHierarchyShwDoIt(PPGMR3DUMPHIERARCHYSTATE pState, uint64_t cr3, unsigned cMaxDepth)
{
    int             rc;
    unsigned const  cch     = pState->cchAddress;
    uint64_t const  cr3Mask = pState->fEpt ? X86_CR3_AMD64_PAGE_MASK
                            : pState->fLme ? X86_CR3_AMD64_PAGE_MASK
                            : pState->fPae ? X86_CR3_PAE_PAGE_MASK
                            :                X86_CR3_PAGE_MASK;
    if (pState->fPrintCr3)
    {
        const char * const  pszMode = pState->fEpt ? "Extended Page Tables"
                                    : pState->fLme ? "Long Mode"
                                    : pState->fPae ? "PAE Mode"
                                    : pState->fPse ? "32-bit w/ PSE"
                                    :                "32-bit";
        pState->pHlp->pfnPrintf(pState->pHlp, "cr3=%0*llx", cch, cr3);
        if (pState->fDumpPageInfo)
            pgmR3DumpHierarchyShwTablePageInfo(pState, cr3 & X86_CR3_AMD64_PAGE_MASK);
        pState->pHlp->pfnPrintf(pState->pHlp, " %s%s%s\n",
                                pszMode,
                                pState->fNp  ? " + Nested Paging" : "",
                                pState->fNxe ? " + NX" : "");
    }


    if (pState->fEpt)
    {
        if (pState->fPrintHeader)
            pState->pHlp->pfnPrintf(pState->pHlp,
                                    "%-*s        R - Readable\n"
                                    "%-*s        | W - Writeable\n"
                                    "%-*s        | | X - Executable\n"
                                    "%-*s        | | | EMT - EPT memory type\n"
                                    "%-*s        | | | |   PAT - Ignored PAT?\n"
                                    "%-*s        | | | |   |    AVL1 - 4 available bits\n"
                                    "%-*s        | | | |   |    |   AVL2 - 12 available bits\n"
                                    "%-*s Level  | | | |   |    |    |     page  \n"
                                  /* xxxx n **** R W X EMT PAT AVL1 AVL2   xxxxxxxxxxxxx
                                                 R W X  7   0   f   fff    0123456701234567 */
                                    ,
                                    cch, "", cch, "", cch, "", cch, "", cch, "", cch, "", cch, "", cch, "Address");

        pState->pHlp->pfnPrintf(pState->pHlp, "EPT dumping is not yet implemented, sorry.\n");
        /** @todo implemented EPT dumping. */
        rc = VERR_NOT_IMPLEMENTED;
    }
    else
    {
        if (pState->fPrintHeader)
            pState->pHlp->pfnPrintf(pState->pHlp,
                                    "%-*s        P - Present\n"
                                    "%-*s        | R/W - Read (0) / Write (1)\n"
                                    "%-*s        | | U/S - User (1) / Supervisor (0)\n"
                                    "%-*s        | | | A - Accessed\n"
                                    "%-*s        | | | | D - Dirty\n"
                                    "%-*s        | | | | | G - Global\n"
                                    "%-*s        | | | | | | WT - Write thru\n"
                                    "%-*s        | | | | | | |  CD - Cache disable\n"
                                    "%-*s        | | | | | | |  |  AT - Attribute table (PAT)\n"
                                    "%-*s        | | | | | | |  |  |  NX - No execute (K8)\n"
                                    "%-*s        | | | | | | |  |  |  |  4K/4M/2M - Page size.\n"
                                    "%-*s        | | | | | | |  |  |  |  |  AVL - a=allocated; m=mapping; d=track dirty;\n"
                                    "%-*s        | | | | | | |  |  |  |  |  |     p=permanent; v=validated;\n"
                                    "%-*s Level  | | | | | | |  |  |  |  |  |    Page\n"
                                  /* xxxx n **** P R S A D G WT CD AT NX 4M AVL xxxxxxxxxxxxx
                                                 - W U - - - -- -- -- -- -- 010 */
                                    ,
                                    cch, "", cch, "", cch, "", cch, "", cch, "", cch, "", cch, "",
                                    cch, "", cch, "", cch, "", cch, "", cch, "", cch, "", cch, "Address");
        if (pState->fLme)
            rc = pgmR3DumpHierarchyShwPaePML4(pState, cr3 & cr3Mask, cMaxDepth);
        else if (pState->fPae)
            rc = pgmR3DumpHierarchyShwPaePDPT(pState, cr3 & cr3Mask, cMaxDepth);
        else
            rc = pgmR3DumpHierarchyShw32BitPD(pState, cr3 & cr3Mask, cMaxDepth);
    }

    if (!pState->cLeaves)
        pState->pHlp->pfnPrintf(pState->pHlp, "not present\n");
    return rc;
}


/**
 * dbgfR3PagingDumpEx worker.
 *
 * @returns VBox status code.
 * @param   pVM             Pointer to the VM.
 * @param   cr3             The CR3 register value.
 * @param   fFlags          The flags, DBGFPGDMP_FLAGS_XXX.
 * @param   u64FirstAddr    The start address.
 * @param   u64LastAddr     The address to stop after.
 * @param   cMaxDepth       The max depth.
 * @param   pHlp            The output callbacks.  Defaults to log if NULL.
 *
 * @internal
 */
VMMR3_INT_DECL(int) PGMR3DumpHierarchyShw(PVM pVM, uint64_t cr3, uint32_t fFlags, uint64_t u64FirstAddr, uint64_t u64LastAddr,
                                          uint32_t cMaxDepth, PCDBGFINFOHLP pHlp)
{
    /* Minimal validation as we're only supposed to service DBGF. */
    AssertReturn(~(fFlags & ~DBGFPGDMP_FLAGS_VALID_MASK), VERR_INVALID_PARAMETER);
    AssertReturn(!(fFlags & (DBGFPGDMP_FLAGS_CURRENT_MODE | DBGFPGDMP_FLAGS_CURRENT_CR3)), VERR_INVALID_PARAMETER);
    AssertReturn(fFlags & DBGFPGDMP_FLAGS_SHADOW, VERR_INVALID_PARAMETER);

    PGMR3DUMPHIERARCHYSTATE State;
    pgmR3DumpHierarchyInitState(&State, pVM, fFlags, u64FirstAddr, u64LastAddr, pHlp);
    return pgmR3DumpHierarchyShwDoIt(&State, cr3, cMaxDepth);
}


/**
 * Dumps a page table hierarchy use only physical addresses and cr4/lm flags.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pVM         Pointer to the VM.
 * @param   cr3         The root of the hierarchy.
 * @param   cr4         The cr4, only PAE and PSE is currently used.
 * @param   fLongMode   Set if long mode, false if not long mode.
 * @param   cMaxDepth   Number of levels to dump.
 * @param   pHlp        Pointer to the output functions.
 *
 * @deprecated Use DBGFR3PagingDumpEx.
 */
VMMR3DECL(int) PGMR3DumpHierarchyHC(PVM pVM, uint64_t cr3, uint64_t cr4, bool fLongMode, unsigned cMaxDepth, PCDBGFINFOHLP pHlp)
{
    if (!cMaxDepth)
        return VINF_SUCCESS;

    PVMCPU pVCpu = VMMGetCpu(pVM);
    if (!pVCpu)
        pVCpu = &pVM->aCpus[0];

    uint32_t fFlags = DBGFPGDMP_FLAGS_HEADER | DBGFPGDMP_FLAGS_PRINT_CR3 | DBGFPGDMP_FLAGS_PAGE_INFO | DBGFPGDMP_FLAGS_SHADOW;
    fFlags |= cr4 & (X86_CR4_PAE | X86_CR4_PSE);
    if (fLongMode)
        fFlags |= DBGFPGDMP_FLAGS_LME;

    return DBGFR3PagingDumpEx(pVM, pVCpu->idCpu, fFlags, cr3, 0, fLongMode ? UINT64_MAX : UINT32_MAX, cMaxDepth, pHlp);
}


/**
 * Maps the guest page.
 *
 * @returns VBox status code.
 * @param   pState              The dumper state.
 * @param   GCPhys              The physical address of the guest page.
 * @param   pszDesc             The description.
 * @param   ppv                 Where to return the pointer.
 * @param   pLock               Where to return the mapping lock.  Hand this to
 *                              PGMPhysReleasePageMappingLock when done.
 */
static int pgmR3DumpHierarchyGstMapPage(PPGMR3DUMPHIERARCHYSTATE pState, RTGCPHYS GCPhys, const char *pszDesc,
                                        void const **ppv, PPGMPAGEMAPLOCK pLock)
{
    int rc = PGMPhysGCPhys2CCPtrReadOnly(pState->pVM, GCPhys, ppv, pLock);
    if (RT_FAILURE(rc))
    {
        pState->pHlp->pfnPrintf(pState->pHlp, "%0*llx error! Failed to map %s at GCPhys=%RGp: %Rrc!\n",
                                pState->cchAddress, pState->u64Address, pszDesc, GCPhys, rc);
        return rc;
    }
    return VINF_SUCCESS;
}


/**
 * Figures out which guest page this is and dumps a summary.
 *
 * @param   pState              The dumper state.
 * @param   GCPhys              The page address.
 * @param   cbPage              The page size.
 */
static void pgmR3DumpHierarchyGstPageInfo(PPGMR3DUMPHIERARCHYSTATE pState, RTGCPHYS GCPhys, uint32_t cbPage)
{
    char szPage[80];
    pgmLock(pState->pVM);
    PCPGMPAGE pPage = pgmPhysGetPage(pState->pVM, GCPhys);
    if (pPage)
        RTStrPrintf(szPage, sizeof(szPage), " %R[pgmpage]", pPage);
    else
        strcpy(szPage, " not found");
    pgmUnlock(pState->pVM);
    pState->pHlp->pfnPrintf(pState->pHlp, "%s", szPage);
    NOREF(cbPage);
}


/**
 * Checks the entry for reserved bits.
 *
 * @param   pState              The dumper state.
 * @param   u64Entry            The entry to check.
 */
static void pgmR3DumpHierarchyGstCheckReservedHighBits(PPGMR3DUMPHIERARCHYSTATE pState, uint64_t u64Entry)
{
    uint32_t uRsvd = (u64Entry & pState->u64HighReservedBits) >> 52;
    if (uRsvd)
        pState->pHlp->pfnPrintf(pState->pHlp, " %u:52=%03x%s",
                                pState->uLastRsvdBit, uRsvd, pState->fLme ? "" : "!");
    /** @todo check the valid physical bits as well. */
}


/**
 * Dumps a PAE shadow page table.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pState              The dumper state.
 * @param   GCPhys              The page table address.
 */
static int pgmR3DumpHierarchyGstPaePT(PPGMR3DUMPHIERARCHYSTATE pState, RTGCPHYS GCPhys)
{
    PCX86PTPAE      pPT;
    PGMPAGEMAPLOCK  Lock;
    int rc = pgmR3DumpHierarchyGstMapPage(pState, GCPhys, "Page table", (void const **)&pPT, &Lock);
    if (RT_FAILURE(rc))
        return rc;

    uint32_t iFirst, iLast;
    uint64_t u64BaseAddress = pgmR3DumpHierarchyCalcRange(pState, X86_PT_PAE_SHIFT, X86_PG_PAE_ENTRIES, &iFirst, &iLast);
    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PTEPAE Pte = pPT->a[i];
        if (Pte.n.u1Present)
        {
            pState->u64Address = u64BaseAddress + ((uint64_t)i << X86_PT_PAE_SHIFT);
            pState->pHlp->pfnPrintf(pState->pHlp,
                                    pState->fLme  /*P R  S  A  D  G  WT CD AT NX 4M a p ?  */
                                    ? "%016llx 3    | P %c %c %c %c %c %s %s %s %s 4K %c%c%c  %016llx"
                                    :  "%08llx 2   |  P %c %c %c %c %c %s %s %s %s 4K %c%c%c  %016llx",
                                    pState->u64Address,
                                    Pte.n.u1Write       ? 'W'  : 'R',
                                    Pte.n.u1User        ? 'U'  : 'S',
                                    Pte.n.u1Accessed    ? 'A'  : '-',
                                    Pte.n.u1Dirty       ? 'D'  : '-',
                                    Pte.n.u1Global      ? 'G'  : '-',
                                    Pte.n.u1WriteThru   ? "WT" : "--",
                                    Pte.n.u1CacheDisable? "CD" : "--",
                                    Pte.n.u1PAT         ? "AT" : "--",
                                    Pte.n.u1NoExecute   ? "NX" : "--",
                                    Pte.u & RT_BIT(9)   ? '1' : '0',
                                    Pte.u & RT_BIT(10)  ? '1' : '0',
                                    Pte.u & RT_BIT(11)  ? '1' : '0',
                                    Pte.u & X86_PTE_PAE_PG_MASK);
            if (pState->fDumpPageInfo)
                pgmR3DumpHierarchyGstPageInfo(pState, Pte.u & X86_PTE_PAE_PG_MASK, _4K);
            pgmR3DumpHierarchyGstCheckReservedHighBits(pState, Pte.u);
            pState->pHlp->pfnPrintf(pState->pHlp, "\n");
            pState->cLeaves++;
        }
    }

    PGMPhysReleasePageMappingLock(pState->pVM, &Lock);
    return VINF_SUCCESS;
}


/**
 * Dumps a PAE shadow page directory table.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pState      The dumper state.
 * @param   GCPhys      The physical address of the table.
 * @param   cMaxDepth   The maximum depth.
 */
static int  pgmR3DumpHierarchyGstPaePD(PPGMR3DUMPHIERARCHYSTATE pState, RTGCPHYS GCPhys, unsigned cMaxDepth)
{
    PCX86PDPAE      pPD;
    PGMPAGEMAPLOCK  Lock;
    int rc = pgmR3DumpHierarchyGstMapPage(pState, GCPhys, "Page directory", (void const **)&pPD, &Lock);
    if (RT_FAILURE(rc))
        return rc;

    Assert(cMaxDepth > 0);
    cMaxDepth--;

    uint32_t iFirst, iLast;
    uint64_t u64BaseAddress = pgmR3DumpHierarchyCalcRange(pState, X86_PD_PAE_SHIFT, X86_PG_PAE_ENTRIES, &iFirst, &iLast);
    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PDEPAE Pde = pPD->a[i];
        if (Pde.n.u1Present)
        {
            pState->u64Address = u64BaseAddress + ((uint64_t)i << X86_PD_PAE_SHIFT);
            if (Pde.b.u1Size)
            {
                pState->pHlp->pfnPrintf(pState->pHlp,
                                        pState->fLme    /*P R  S  A  D  G  WT CD AT NX 2M a  p ?  phys*/
                                        ? "%016llx 2   |  P %c %c %c %c %c %s %s %s %s 2M %c%c%c  %016llx"
                                        :  "%08llx 1  |   P %c %c %c %c %c %s %s %s %s 2M %c%c%c  %016llx",
                                        pState->u64Address,
                                        Pde.b.u1Write         ? 'W'  : 'R',
                                        Pde.b.u1User          ? 'U'  : 'S',
                                        Pde.b.u1Accessed      ? 'A'  : '-',
                                        Pde.b.u1Dirty         ? 'D'  : '-',
                                        Pde.b.u1Global        ? 'G'  : '-',
                                        Pde.b.u1WriteThru     ? "WT" : "--",
                                        Pde.b.u1CacheDisable  ? "CD" : "--",
                                        Pde.b.u1PAT           ? "AT" : "--",
                                        Pde.b.u1NoExecute     ? "NX" : "--",
                                        Pde.u & RT_BIT_64(9)  ? '1' : '0',
                                        Pde.u & RT_BIT_64(10) ? '1' : '0',
                                        Pde.u & RT_BIT_64(11) ? '1' : '0',
                                        Pde.u & X86_PDE2M_PAE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyGstPageInfo(pState, Pde.u & X86_PDE2M_PAE_PG_MASK, _2M);
                pgmR3DumpHierarchyGstCheckReservedHighBits(pState, Pde.u);
                if ((Pde.u >> 13) & 0xff)
                    pState->pHlp->pfnPrintf(pState->pHlp, " 20:13=%02llx%s", (Pde.u >> 13) & 0x0ff, pState->fLme ? "" : "!");
                pState->pHlp->pfnPrintf(pState->pHlp, "\n");

                pState->cLeaves++;
            }
            else
            {
                pState->pHlp->pfnPrintf(pState->pHlp,
                                        pState->fLme    /*P R  S  A  D  G  WT CD AT NX 4M a  p ?  phys */
                                        ? "%016llx 2   |  P %c %c %c %c %c %s %s .. %s .. %c%c%c  %016llx"
                                        :  "%08llx 1  |   P %c %c %c %c %c %s %s .. %s .. %c%c%c  %016llx",
                                        pState->u64Address,
                                        Pde.n.u1Write         ? 'W'  : 'R',
                                        Pde.n.u1User          ? 'U'  : 'S',
                                        Pde.n.u1Accessed      ? 'A'  : '-',
                                        Pde.n.u1Reserved0     ? '?'  : '.', /* ignored */
                                        Pde.n.u1Reserved1     ? '?'  : '.', /* ignored */
                                        Pde.n.u1WriteThru     ? "WT" : "--",
                                        Pde.n.u1CacheDisable  ? "CD" : "--",
                                        Pde.n.u1NoExecute     ? "NX" : "--",
                                        Pde.u & RT_BIT_64(9)  ? '1' : '0',
                                        Pde.u & RT_BIT_64(10) ? '1' : '0',
                                        Pde.u & RT_BIT_64(11) ? '1' : '0',
                                        Pde.u & X86_PDE_PAE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyGstPageInfo(pState, Pde.u & X86_PDE_PAE_PG_MASK, _4K);
                pgmR3DumpHierarchyGstCheckReservedHighBits(pState, Pde.u);
                pState->pHlp->pfnPrintf(pState->pHlp, "\n");

                if (cMaxDepth)
                {
                    int rc2 = pgmR3DumpHierarchyGstPaePT(pState, Pde.u & X86_PDE_PAE_PG_MASK);
                    if (rc2 < rc && RT_SUCCESS(rc))
                        rc = rc2;
                }
                else
                    pState->cLeaves++;
            }
        }
    }

    PGMPhysReleasePageMappingLock(pState->pVM, &Lock);
    return rc;
}


/**
 * Dumps a PAE shadow page directory pointer table.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pState      The dumper state.
 * @param   GCPhys      The physical address of the table.
 * @param   cMaxDepth   The maximum depth.
 */
static int  pgmR3DumpHierarchyGstPaePDPT(PPGMR3DUMPHIERARCHYSTATE pState, RTGCPHYS GCPhys, unsigned cMaxDepth)
{
    /* Fend of addresses that are out of range in PAE mode - simplifies the code below. */
    if (!pState->fLme && pState->u64Address >= _4G)
        return VINF_SUCCESS;

    PCX86PDPT       pPDPT;
    PGMPAGEMAPLOCK  Lock;
    int rc = pgmR3DumpHierarchyGstMapPage(pState, GCPhys, "Page directory pointer table", (void const **)&pPDPT, &Lock);
    if (RT_FAILURE(rc))
        return rc;

    Assert(cMaxDepth > 0);
    cMaxDepth--;

    uint32_t iFirst, iLast;
    uint64_t u64BaseAddress = pgmR3DumpHierarchyCalcRange(pState, X86_PDPT_SHIFT,
                                                          pState->fLme ? X86_PG_AMD64_PDPE_ENTRIES : X86_PG_PAE_PDPE_ENTRIES,
                                                          &iFirst, &iLast);
    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PDPE Pdpe = pPDPT->a[i];
        if (Pdpe.n.u1Present)
        {
            pState->u64Address = u64BaseAddress + ((uint64_t)i << X86_PDPT_SHIFT);
            if (pState->fLme)
            {
                /** @todo Do 1G pages.  */
                pState->pHlp->pfnPrintf(pState->pHlp, /*P R  S  A  D  G  WT CD AT NX .. a p ?  */
                                        "%016llx 1  |   P %c %c %c %c %c %s %s %s %s .. %c%c%c  %016llx",
                                        pState->u64Address,
                                        Pdpe.lm.u1Write        ? 'W'  : 'R',
                                        Pdpe.lm.u1User         ? 'U'  : 'S',
                                        Pdpe.lm.u1Accessed     ? 'A'  : '-',
                                        Pdpe.lm.u3Reserved & 1 ? '?'  : '.', /* ignored */
                                        Pdpe.lm.u3Reserved & 4 ? '!'  : '.', /* mbz */
                                        Pdpe.lm.u1WriteThru    ? "WT" : "--",
                                        Pdpe.lm.u1CacheDisable ? "CD" : "--",
                                        Pdpe.lm.u3Reserved & 2 ? "!"  : "..",/* mbz */
                                        Pdpe.lm.u1NoExecute    ? "NX" : "--",
                                        Pdpe.u & RT_BIT_64(9)  ? '1' : '0',
                                        Pdpe.u & RT_BIT_64(10) ? '1' : '0',
                                        Pdpe.u & RT_BIT_64(11) ? '1' : '0',
                                        Pdpe.u & X86_PDPE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyGstPageInfo(pState, Pdpe.u & X86_PDPE_PG_MASK, _4K);
                pgmR3DumpHierarchyGstCheckReservedHighBits(pState, Pdpe.u);
            }
            else
            {
                pState->pHlp->pfnPrintf(pState->pHlp,/*P R  S  A  D  G  WT CD AT NX .. a p ?  */
                                        "%08llx 0 |    P %c %c %c %c %c %s %s %s %s .. %c%c%c  %016llx",
                                        pState->u64Address,
                                        Pdpe.n.u2Reserved & 1  ? '!'  : '.',  /* mbz */
                                        Pdpe.n.u2Reserved & 2  ? '!'  : '.',  /* mbz */
                                        Pdpe.n.u4Reserved & 1  ? '!'  : '.',  /* mbz */
                                        Pdpe.n.u4Reserved & 2  ? '!'  : '.',  /* mbz */
                                        Pdpe.n.u4Reserved & 8  ? '!'  : '.',  /* mbz */
                                        Pdpe.n.u1WriteThru     ? "WT" : "--",
                                        Pdpe.n.u1CacheDisable  ? "CD" : "--",
                                        Pdpe.n.u4Reserved & 2  ? "!"  : "..", /* mbz */
                                        Pdpe.lm.u1NoExecute    ? "!!"  : "..",/* mbz */
                                        Pdpe.u & RT_BIT_64(9)  ? '1' : '0',
                                        Pdpe.u & RT_BIT_64(10) ? '1' : '0',
                                        Pdpe.u & RT_BIT_64(11) ? '1' : '0',
                                        Pdpe.u & X86_PDPE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyGstPageInfo(pState, Pdpe.u & X86_PDPE_PG_MASK, _4K);
                pgmR3DumpHierarchyGstCheckReservedHighBits(pState, Pdpe.u);
            }
            pState->pHlp->pfnPrintf(pState->pHlp, "\n");

            if (cMaxDepth)
            {
                int rc2 = pgmR3DumpHierarchyGstPaePD(pState, Pdpe.u & X86_PDPE_PG_MASK, cMaxDepth);
                if (rc2 < rc && RT_SUCCESS(rc))
                    rc = rc2;
            }
            else
                pState->cLeaves++;
        }
    }

    PGMPhysReleasePageMappingLock(pState->pVM, &Lock);
    return rc;
}


/**
 * Dumps a 32-bit shadow page table.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pVM         Pointer to the VM.
 * @param   GCPhys      The physical address of the table.
 * @param   cMaxDepth   The maximum depth.
 */
static int pgmR3DumpHierarchyGstPaePML4(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS GCPhys, unsigned cMaxDepth)
{
    PCX86PML4       pPML4;
    PGMPAGEMAPLOCK  Lock;
    int rc = pgmR3DumpHierarchyGstMapPage(pState, GCPhys, "Page map level 4", (void const **)&pPML4, &Lock);
    if (RT_FAILURE(rc))
        return rc;

    Assert(cMaxDepth);
    cMaxDepth--;

    /*
     * This is a bit tricky as we're working on unsigned addresses while the
     * AMD64 spec uses signed tricks.
     */
    uint32_t iFirst = (pState->u64FirstAddress >> X86_PML4_SHIFT) & X86_PML4_MASK;
    uint32_t iLast  = (pState->u64LastAddress  >> X86_PML4_SHIFT) & X86_PML4_MASK;
    if (   pState->u64LastAddress  <= UINT64_C(0x00007fffffffffff)
        || pState->u64FirstAddress >= UINT64_C(0xffff800000000000))
    { /* Simple, nothing to adjust */ }
    else if (pState->u64FirstAddress <= UINT64_C(0x00007fffffffffff))
        iLast = X86_PG_AMD64_ENTRIES / 2 - 1;
    else if (pState->u64LastAddress  >= UINT64_C(0xffff800000000000))
        iFirst = X86_PG_AMD64_ENTRIES / 2;
    else
        iFirst = X86_PG_AMD64_ENTRIES; /* neither address is canonical */

    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PML4E Pml4e = pPML4->a[i];
        if (Pml4e.n.u1Present)
        {
            pState->u64Address = ((uint64_t)i << X86_PML4_SHIFT)
                               | (i >= RT_ELEMENTS(pPML4->a) / 2 ? UINT64_C(0xffff000000000000) : 0);
            pState->pHlp->pfnPrintf(pState->pHlp, /*P R  S  A  D  G  WT CD AT NX 4M a p ?  */
                                    "%016llx 0 |    P %c %c %c %c %c %s %s %s %s .. %c%c%c  %016llx",
                                    pState->u64Address,
                                    Pml4e.n.u1Write         ? 'W'  : 'R',
                                    Pml4e.n.u1User          ? 'U'  : 'S',
                                    Pml4e.n.u1Accessed      ? 'A'  : '-',
                                    Pml4e.n.u3Reserved & 1  ? '?'  : '.', /* ignored */
                                    Pml4e.n.u3Reserved & 4  ? '!'  : '.', /* mbz */
                                    Pml4e.n.u1WriteThru     ? "WT" : "--",
                                    Pml4e.n.u1CacheDisable  ? "CD" : "--",
                                    Pml4e.n.u3Reserved & 2  ? "!"  : "..",/* mbz */
                                    Pml4e.n.u1NoExecute     ? "NX" : "--",
                                    Pml4e.u & RT_BIT_64(9)  ? '1' : '0',
                                    Pml4e.u & RT_BIT_64(10) ? '1' : '0',
                                    Pml4e.u & RT_BIT_64(11) ? '1' : '0',
                                    Pml4e.u & X86_PML4E_PG_MASK);
            if (pState->fDumpPageInfo)
                pgmR3DumpHierarchyGstPageInfo(pState, Pml4e.u & X86_PML4E_PG_MASK, _4K);
            pgmR3DumpHierarchyGstCheckReservedHighBits(pState, Pml4e.u);
            pState->pHlp->pfnPrintf(pState->pHlp, "\n");

            if (cMaxDepth)
            {
                int rc2 = pgmR3DumpHierarchyGstPaePDPT(pState, Pml4e.u & X86_PML4E_PG_MASK, cMaxDepth);
                if (rc2 < rc && RT_SUCCESS(rc))
                    rc = rc2;
            }
            else
                pState->cLeaves++;
        }
    }

    PGMPhysReleasePageMappingLock(pState->pVM, &Lock);
    return rc;
}


/**
 * Dumps a 32-bit shadow page table.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pState      The dumper state.
 * @param   GCPhys      The physical address of the table.
 */
static int pgmR3DumpHierarchyGst32BitPT(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS GCPhys)
{
    PCX86PT         pPT;
    PGMPAGEMAPLOCK  Lock;
    int rc = pgmR3DumpHierarchyGstMapPage(pState, GCPhys, "Page table", (void const **)&pPT, &Lock);
    if (RT_FAILURE(rc))
        return rc;

    uint32_t iFirst, iLast;
    uint64_t u64BaseAddress = pgmR3DumpHierarchyCalcRange(pState, X86_PT_SHIFT, X86_PG_ENTRIES, &iFirst, &iLast);
    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PTE Pte = pPT->a[i];
        if (Pte.n.u1Present)
        {
            pState->u64Address = u64BaseAddress + (i << X86_PT_SHIFT);
            pState->pHlp->pfnPrintf(pState->pHlp,/*P R  S  A  D  G  WT CD AT NX 4M a m d  */
                                    "%08llx 1  |   P %c %c %c %c %c %s %s %s .. 4K %c%c%c  %08x",
                                    pState->u64Address,
                                    Pte.n.u1Write         ? 'W'  : 'R',
                                    Pte.n.u1User          ? 'U'  : 'S',
                                    Pte.n.u1Accessed      ? 'A'  : '-',
                                    Pte.n.u1Dirty         ? 'D'  : '-',
                                    Pte.n.u1Global        ? 'G'  : '-',
                                    Pte.n.u1WriteThru     ? "WT" : "--",
                                    Pte.n.u1CacheDisable  ? "CD" : "--",
                                    Pte.n.u1PAT           ? "AT" : "--",
                                    Pte.u & RT_BIT_32(9)  ? '1' : '0',
                                    Pte.u & RT_BIT_32(10) ? '1' : '0',
                                    Pte.u & RT_BIT_32(11) ? '1' : '0',
                                    Pte.u & X86_PDE_PG_MASK);
            if (pState->fDumpPageInfo)
                pgmR3DumpHierarchyGstPageInfo(pState, Pte.u & X86_PDE_PG_MASK, _4K);
            pState->pHlp->pfnPrintf(pState->pHlp, "\n");
        }
    }

    PGMPhysReleasePageMappingLock(pState->pVM, &Lock);
    return VINF_SUCCESS;
}


/**
 * Dumps a 32-bit shadow page directory and page tables.
 *
 * @returns VBox status code (VINF_SUCCESS).
 * @param   pState      The dumper state.
 * @param   GCPhys      The physical address of the table.
 * @param   cMaxDepth   The maximum depth.
 */
static int pgmR3DumpHierarchyGst32BitPD(PPGMR3DUMPHIERARCHYSTATE pState, RTHCPHYS GCPhys, unsigned cMaxDepth)
{
    if (pState->u64Address >= _4G)
        return VINF_SUCCESS;

    PCX86PD         pPD;
    PGMPAGEMAPLOCK  Lock;
    int rc = pgmR3DumpHierarchyGstMapPage(pState, GCPhys, "Page directory", (void const **)&pPD, &Lock);
    if (RT_FAILURE(rc))
        return rc;

    Assert(cMaxDepth > 0);
    cMaxDepth--;

    uint32_t iFirst, iLast;
    pgmR3DumpHierarchyCalcRange(pState, X86_PD_SHIFT, X86_PG_ENTRIES, &iFirst, &iLast);
    for (uint32_t i = iFirst; i <= iLast; i++)
    {
        X86PDE Pde = pPD->a[i];
        if (Pde.n.u1Present)
        {
            pState->u64Address = (uint32_t)i << X86_PD_SHIFT;
            if (Pde.b.u1Size && pState->fPse)
            {
                uint64_t u64Phys = ((uint64_t)(Pde.u & X86_PDE4M_PG_HIGH_MASK) << X86_PDE4M_PG_HIGH_SHIFT)
                                 | (Pde.u & X86_PDE4M_PG_MASK);
                pState->pHlp->pfnPrintf(pState->pHlp,/*P R  S  A  D  G  WT CD AT NX 4M a m d   phys */
                                        "%08llx 0 |    P %c %c %c %c %c %s %s %s .. 4M %c%c%c  %08llx",
                                        pState->u64Address,
                                        Pde.b.u1Write         ? 'W'  : 'R',
                                        Pde.b.u1User          ? 'U'  : 'S',
                                        Pde.b.u1Accessed      ? 'A'  : '-',
                                        Pde.b.u1Dirty         ? 'D'  : '-',
                                        Pde.b.u1Global        ? 'G'  : '-',
                                        Pde.b.u1WriteThru     ? "WT" : "--",
                                        Pde.b.u1CacheDisable  ? "CD" : "--",
                                        Pde.b.u1PAT           ? "AT" : "--",
                                        Pde.u & RT_BIT_32(9)  ? '1' : '0',
                                        Pde.u & RT_BIT_32(10) ? '1' : '0',
                                        Pde.u & RT_BIT_32(11) ? '1' : '0',
                                        u64Phys);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyGstPageInfo(pState, u64Phys, _4M);
                pState->pHlp->pfnPrintf(pState->pHlp, "\n");
                pState->cLeaves++;
            }
            else
            {
                pState->pHlp->pfnPrintf(pState->pHlp,/*P R  S  A  D  G  WT CD AT NX 4M a m d   phys */
                                        "%08llx 0 |    P %c %c %c %c %c %s %s .. .. .. %c%c%c  %08x",
                                        pState->u64Address,
                                        Pde.n.u1Write         ? 'W'  : 'R',
                                        Pde.n.u1User          ? 'U'  : 'S',
                                        Pde.n.u1Accessed      ? 'A'  : '-',
                                        Pde.n.u1Reserved0     ? '?'  : '.', /* ignored */
                                        Pde.n.u1Reserved1     ? '?'  : '.', /* ignored */
                                        Pde.n.u1WriteThru     ? "WT" : "--",
                                        Pde.n.u1CacheDisable  ? "CD" : "--",
                                        Pde.u & RT_BIT_32(9)  ? '1' : '0',
                                        Pde.u & RT_BIT_32(10) ? '1' : '0',
                                        Pde.u & RT_BIT_32(11) ? '1' : '0',
                                        Pde.u & X86_PDE_PG_MASK);
                if (pState->fDumpPageInfo)
                    pgmR3DumpHierarchyGstPageInfo(pState, Pde.u & X86_PDE_PG_MASK, _4K);
                pState->pHlp->pfnPrintf(pState->pHlp, "\n");

                if (cMaxDepth)
                {
                    int rc2 = pgmR3DumpHierarchyGst32BitPT(pState, Pde.u & X86_PDE_PG_MASK);
                    if (rc2 < rc && RT_SUCCESS(rc))
                        rc = rc2;
                }
                else
                    pState->cLeaves++;
            }
        }
    }

    PGMPhysReleasePageMappingLock(pState->pVM, &Lock);
    return rc;
}


/**
 * Internal worker that initiates the actual dump.
 *
 * @returns VBox status code.
 * @param   pState              The dumper state.
 * @param   cr3                 The CR3 value.
 * @param   cMaxDepth           The max depth.
 */
static int pgmR3DumpHierarchyGstDoIt(PPGMR3DUMPHIERARCHYSTATE pState, uint64_t cr3, unsigned cMaxDepth)
{
    int             rc;
    unsigned const  cch     = pState->cchAddress;
    uint64_t const  cr3Mask = pState->fEpt ? X86_CR3_AMD64_PAGE_MASK
                            : pState->fLme ? X86_CR3_AMD64_PAGE_MASK
                            : pState->fPae ? X86_CR3_PAE_PAGE_MASK
                            :                X86_CR3_PAGE_MASK;
    if (pState->fPrintCr3)
    {
        const char * const  pszMode = pState->fEpt ? "Extended Page Tables"
                                    : pState->fLme ? "Long Mode"
                                    : pState->fPae ? "PAE Mode"
                                    : pState->fPse ? "32-bit w/ PSE"
                                    :                "32-bit";
        pState->pHlp->pfnPrintf(pState->pHlp, "cr3=%0*llx", cch, cr3);
        if (pState->fDumpPageInfo)
            pgmR3DumpHierarchyGstPageInfo(pState, cr3 & X86_CR3_AMD64_PAGE_MASK, _4K);
        pState->pHlp->pfnPrintf(pState->pHlp, " %s%s%s\n",
                                pszMode,
                                pState->fNp  ? " + Nested Paging" : "",
                                pState->fNxe ? " + NX" : "");
    }


    if (pState->fEpt)
    {
        if (pState->fPrintHeader)
            pState->pHlp->pfnPrintf(pState->pHlp,
                                    "%-*s        R - Readable\n"
                                    "%-*s        | W - Writeable\n"
                                    "%-*s        | | X - Executable\n"
                                    "%-*s        | | | EMT - EPT memory type\n"
                                    "%-*s        | | | |   PAT - Ignored PAT?\n"
                                    "%-*s        | | | |   |    AVL1 - 4 available bits\n"
                                    "%-*s        | | | |   |    |   AVL2 - 12 available bits\n"
                                    "%-*s Level  | | | |   |    |    |     page  \n"
                                  /* xxxx n **** R W X EMT PAT AVL1 AVL2   xxxxxxxxxxxxx
                                                 R W X  7   0   f   fff    0123456701234567 */
                                    ,
                                    cch, "", cch, "", cch, "", cch, "", cch, "", cch, "", cch, "", cch, "Address");

        pState->pHlp->pfnPrintf(pState->pHlp, "EPT dumping is not yet implemented, sorry.\n");
        /** @todo implemented EPT dumping. */
        rc = VERR_NOT_IMPLEMENTED;
    }
    else
    {
        if (pState->fPrintHeader)
            pState->pHlp->pfnPrintf(pState->pHlp,
                                    "%-*s        P - Present\n"
                                    "%-*s        | R/W - Read (0) / Write (1)\n"
                                    "%-*s        | | U/S - User (1) / Supervisor (0)\n"
                                    "%-*s        | | | A - Accessed\n"
                                    "%-*s        | | | | D - Dirty\n"
                                    "%-*s        | | | | | G - Global\n"
                                    "%-*s        | | | | | | WT - Write thru\n"
                                    "%-*s        | | | | | | |  CD - Cache disable\n"
                                    "%-*s        | | | | | | |  |  AT - Attribute table (PAT)\n"
                                    "%-*s        | | | | | | |  |  |  NX - No execute (K8)\n"
                                    "%-*s        | | | | | | |  |  |  |  4K/4M/2M - Page size.\n"
                                    "%-*s        | | | | | | |  |  |  |  |  AVL - 3 available bits.\n"
                                    "%-*s Level  | | | | | | |  |  |  |  |  |    Page\n"
                                  /* xxxx n **** P R S A D G WT CD AT NX 4M AVL xxxxxxxxxxxxx
                                                 - W U - - - -- -- -- -- -- 010 */
                                    ,
                                    cch, "", cch, "", cch, "", cch, "", cch, "", cch, "", cch, "",
                                    cch, "", cch, "", cch, "", cch, "", cch, "", cch, "", cch, "Address");
        if (pState->fLme)
            rc = pgmR3DumpHierarchyGstPaePML4(pState, cr3 & cr3Mask, cMaxDepth);
        else if (pState->fPae)
            rc = pgmR3DumpHierarchyGstPaePDPT(pState, cr3 & cr3Mask, cMaxDepth);
        else
            rc = pgmR3DumpHierarchyGst32BitPD(pState, cr3 & cr3Mask, cMaxDepth);
    }

    if (!pState->cLeaves)
        pState->pHlp->pfnPrintf(pState->pHlp, "not present\n");
    return rc;
}


/**
 * dbgfR3PagingDumpEx worker.
 *
 * @returns VBox status code.
 * @param   pVM             Pointer to the VM.
 * @param   cr3             The CR3 register value.
 * @param   fFlags          The flags, DBGFPGDMP_FLAGS_XXX.
 * @param   FirstAddr       The start address.
 * @param   LastAddr        The address to stop after.
 * @param   cMaxDepth       The max depth.
 * @param   pHlp            The output callbacks.  Defaults to log if NULL.
 *
 * @internal
 */
VMMR3_INT_DECL(int) PGMR3DumpHierarchyGst(PVM pVM, uint64_t cr3, uint32_t fFlags, RTGCPTR FirstAddr, RTGCPTR LastAddr,
                                          uint32_t cMaxDepth, PCDBGFINFOHLP pHlp)
{
    /* Minimal validation as we're only supposed to service DBGF. */
    AssertReturn(~(fFlags & ~DBGFPGDMP_FLAGS_VALID_MASK), VERR_INVALID_PARAMETER);
    AssertReturn(!(fFlags & (DBGFPGDMP_FLAGS_CURRENT_MODE | DBGFPGDMP_FLAGS_CURRENT_CR3)), VERR_INVALID_PARAMETER);
    AssertReturn(fFlags & DBGFPGDMP_FLAGS_GUEST, VERR_INVALID_PARAMETER);

    PGMR3DUMPHIERARCHYSTATE State;
    pgmR3DumpHierarchyInitState(&State, pVM, fFlags, FirstAddr, LastAddr, pHlp);
    return pgmR3DumpHierarchyGstDoIt(&State, cr3, cMaxDepth);
}

