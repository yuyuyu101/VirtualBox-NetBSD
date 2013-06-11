/* $Id: DBGFOS.cpp $ */
/** @file
 * DBGF - Debugger Facility, Guest OS Diggers.
 */

/*
 * Copyright (C) 2008-2012 Oracle Corporation
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
#define LOG_GROUP LOG_GROUP_DBGF
#include <VBox/vmm/dbgf.h>
#include <VBox/vmm/mm.h>
#include "DBGFInternal.h"
#include <VBox/vmm/vm.h>
#include <VBox/err.h>
#include <VBox/log.h>

#include <iprt/assert.h>
#include <iprt/thread.h>
#include <iprt/param.h>


/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
#define DBGF_OS_READ_LOCK(pVM)      do { } while (0)
#define DBGF_OS_READ_UNLOCK(pVM)    do { } while (0)

#define DBGF_OS_WRITE_LOCK(pVM)     do { } while (0)
#define DBGF_OS_WRITE_UNLOCK(pVM)   do { } while (0)


/**
 * Internal cleanup routine called by DBGFR3Term().
 *
 * @param   pVM     Pointer to the VM.
 */
void dbgfR3OSTerm(PVM pVM)
{
    /*
     * Terminate the current one.
     */
    if (pVM->dbgf.s.pCurOS)
    {
        pVM->dbgf.s.pCurOS->pReg->pfnTerm(pVM, pVM->dbgf.s.pCurOS->abData);
        pVM->dbgf.s.pCurOS = NULL;
    }

    /*
     * Destroy all the instances.
     */
    while (pVM->dbgf.s.pOSHead)
    {
        PDBGFOS pOS = pVM->dbgf.s.pOSHead;
        pVM->dbgf.s.pOSHead = pOS->pNext;
        if (pOS->pReg->pfnDestruct)
            pOS->pReg->pfnDestruct(pVM, pOS->abData);
        MMR3HeapFree(pOS);
    }
}


/**
 * EMT worker function for DBGFR3OSRegister.
 *
 * @returns VBox status code.
 * @param   pVM     Pointer to the VM.
 * @param   pReg    The registration structure.
 */
static DECLCALLBACK(int) dbgfR3OSRegister(PVM pVM, PDBGFOSREG pReg)
{
    /* more validations. */
    DBGF_OS_READ_LOCK(pVM);
    PDBGFOS pOS;
    for (pOS = pVM->dbgf.s.pOSHead; pOS; pOS = pOS->pNext)
        if (!strcmp(pOS->pReg->szName, pReg->szName))
        {
            DBGF_OS_READ_UNLOCK(pVM);
            Log(("dbgfR3OSRegister: %s -> VERR_ALREADY_LOADED\n", pReg->szName));
            return VERR_ALREADY_LOADED;
        }

    /*
     * Allocate a new structure, call the constructor and link it into the list.
     */
    pOS = (PDBGFOS)MMR3HeapAllocZ(pVM, MM_TAG_DBGF_OS, RT_OFFSETOF(DBGFOS, abData[pReg->cbData]));
    AssertReturn(pOS, VERR_NO_MEMORY);
    pOS->pReg = pReg;

    int rc = pOS->pReg->pfnConstruct(pVM, pOS->abData);
    if (RT_SUCCESS(rc))
    {
        DBGF_OS_WRITE_LOCK(pVM);
        pOS->pNext = pVM->dbgf.s.pOSHead;
        pVM->dbgf.s.pOSHead = pOS;
        DBGF_OS_WRITE_UNLOCK(pVM);
    }
    else
    {
        if (pOS->pReg->pfnDestruct)
            pOS->pReg->pfnDestruct(pVM, pOS->abData);
        MMR3HeapFree(pOS);
    }

    return VINF_SUCCESS;
}


/**
 * Registers a guest OS digger.
 *
 * This will instantiate an instance of the digger and add it
 * to the list for us in the next call to DBGFR3OSDetect().
 *
 * @returns VBox status code.
 * @param   pVM     Pointer to the VM.
 * @param   pReg    The registration structure.
 * @thread  Any.
 */
VMMR3DECL(int) DBGFR3OSRegister(PVM pVM, PCDBGFOSREG pReg)
{
    /*
     * Validate intput.
     */
    VM_ASSERT_VALID_EXT_RETURN(pVM, VERR_INVALID_VM_HANDLE);

    AssertPtrReturn(pReg, VERR_INVALID_POINTER);
    AssertReturn(pReg->u32Magic == DBGFOSREG_MAGIC, VERR_INVALID_MAGIC);
    AssertReturn(pReg->u32EndMagic == DBGFOSREG_MAGIC, VERR_INVALID_MAGIC);
    AssertReturn(!pReg->fFlags, VERR_INVALID_PARAMETER);
    AssertReturn(pReg->cbData < _2G, VERR_INVALID_PARAMETER);
    AssertReturn(pReg->szName[0], VERR_INVALID_NAME);
    AssertReturn(RTStrEnd(&pReg->szName[0], sizeof(pReg->szName)), VERR_INVALID_NAME);
    AssertPtrReturn(pReg->pfnConstruct, VERR_INVALID_POINTER);
    AssertPtrNullReturn(pReg->pfnDestruct, VERR_INVALID_POINTER);
    AssertPtrReturn(pReg->pfnProbe, VERR_INVALID_POINTER);
    AssertPtrReturn(pReg->pfnInit, VERR_INVALID_POINTER);
    AssertPtrReturn(pReg->pfnRefresh, VERR_INVALID_POINTER);
    AssertPtrReturn(pReg->pfnTerm, VERR_INVALID_POINTER);
    AssertPtrReturn(pReg->pfnQueryVersion, VERR_INVALID_POINTER);
    AssertPtrReturn(pReg->pfnQueryInterface, VERR_INVALID_POINTER);

    /*
     * Pass it on to EMT(0).
     */
    return VMR3ReqPriorityCallWait(pVM, 0 /*idDstCpu*/, (PFNRT)dbgfR3OSRegister, 2, pVM, pReg);
}


/**
 * EMT worker function for DBGFR3OSDeregister.
 *
 * @returns VBox status code.
 * @param   pVM     Pointer to the VM.
 * @param   pReg    The registration structure.
 */
static DECLCALLBACK(int) dbgfR3OSDeregister(PVM pVM, PDBGFOSREG pReg)
{
    /*
     * Unlink it.
     */
    bool    fWasCurOS = false;
    PDBGFOS pOSPrev   = NULL;
    PDBGFOS pOS;
    DBGF_OS_WRITE_LOCK(pVM);
    for (pOS = pVM->dbgf.s.pOSHead; pOS; pOSPrev = pOS, pOS = pOS->pNext)
        if (pOS->pReg == pReg)
        {
            if (pOSPrev)
                pOSPrev->pNext = pOS->pNext;
            else
                pVM->dbgf.s.pOSHead = pOS->pNext;
            if (pVM->dbgf.s.pCurOS == pOS)
            {
                pVM->dbgf.s.pCurOS = NULL;
                fWasCurOS = true;
            }
            break;
        }
    DBGF_OS_WRITE_UNLOCK(pVM);
    if (!pOS)
    {
        Log(("DBGFR3OSDeregister: %s -> VERR_NOT_FOUND\n", pReg->szName));
        return VERR_NOT_FOUND;
    }

    /*
     * Terminate it if it was the current OS, then invoke the
     * destructor and clean up.
     */
    if (fWasCurOS)
        pOS->pReg->pfnTerm(pVM, pOS->abData);
    if (pOS->pReg->pfnDestruct)
        pOS->pReg->pfnDestruct(pVM, pOS->abData);
    MMR3HeapFree(pOS);

    return VINF_SUCCESS;
}


/**
 * Deregisters a guest OS digger previously registered by DBGFR3OSRegister.
 *
 * @returns VBox status code.
 *
 * @param   pVM     Pointer to the VM.
 * @param   pReg    The registration structure.
 * @thread  Any.
 */
VMMR3DECL(int)  DBGFR3OSDeregister(PVM pVM, PCDBGFOSREG pReg)
{
    /*
     * Validate input.
     */
    VM_ASSERT_VALID_EXT_RETURN(pVM, VERR_INVALID_VM_HANDLE);
    AssertPtrReturn(pReg, VERR_INVALID_POINTER);
    AssertReturn(pReg->u32Magic == DBGFOSREG_MAGIC, VERR_INVALID_MAGIC);
    AssertReturn(pReg->u32EndMagic == DBGFOSREG_MAGIC, VERR_INVALID_MAGIC);
    AssertReturn(RTStrEnd(&pReg->szName[0], sizeof(pReg->szName)), VERR_INVALID_NAME);

    DBGF_OS_READ_LOCK(pVM);
    PDBGFOS pOS;
    for (pOS = pVM->dbgf.s.pOSHead; pOS; pOS = pOS->pNext)
        if (pOS->pReg == pReg)
            break;
    DBGF_OS_READ_LOCK(pVM);

    if (!pOS)
    {
        Log(("DBGFR3OSDeregister: %s -> VERR_NOT_FOUND\n", pReg->szName));
        return VERR_NOT_FOUND;
    }

    /*
     * Pass it on to EMT(0).
     */
    return VMR3ReqPriorityCallWait(pVM, 0 /*idDstCpu*/, (PFNRT)dbgfR3OSDeregister, 2, pVM, pReg);
}


/**
 * EMT worker function for DBGFR3OSDetect.
 *
 * @returns VBox status code.
 * @retval  VINF_SUCCESS if successfully detected.
 * @retval  VINF_DBGF_OS_NOT_DETCTED if we cannot figure it out.
 *
 * @param   pVM         Pointer to the VM.
 * @param   pszName     Where to store the OS name. Empty string if not detected.
 * @param   cchName     Size of the buffer.
 */
static DECLCALLBACK(int) dbgfR3OSDetect(PVM pVM, char *pszName, size_t cchName)
{
    /*
     * Cycle thru the detection routines.
     */
    PDBGFOS const pOldOS = pVM->dbgf.s.pCurOS;
    pVM->dbgf.s.pCurOS = NULL;

    for (PDBGFOS pNewOS = pVM->dbgf.s.pOSHead; pNewOS; pNewOS = pNewOS->pNext)
        if (pNewOS->pReg->pfnProbe(pVM, pNewOS->abData))
        {
            int rc;
            pVM->dbgf.s.pCurOS = pNewOS;
            if (pOldOS == pNewOS)
                rc = pNewOS->pReg->pfnRefresh(pVM, pNewOS->abData);
            else
            {
                if (pOldOS)
                    pOldOS->pReg->pfnTerm(pVM, pNewOS->abData);
                rc = pNewOS->pReg->pfnInit(pVM, pNewOS->abData);
            }
            if (pszName && cchName)
                strncat(pszName, pNewOS->pReg->szName, cchName);
            return rc;
        }

    /* not found */
    if (pOldOS)
        pOldOS->pReg->pfnTerm(pVM, pOldOS->abData);
    return VINF_DBGF_OS_NOT_DETCTED;
}


/**
 * Detects the guest OS and try dig out symbols and useful stuff.
 *
 * When called the 2nd time, symbols will be updated that if the OS
 * is the same.
 *
 * @returns VBox status code.
 * @retval  VINF_SUCCESS if successfully detected.
 * @retval  VINF_DBGF_OS_NOT_DETCTED if we cannot figure it out.
 *
 * @param   pVM         Pointer to the VM.
 * @param   pszName     Where to store the OS name. Empty string if not detected.
 * @param   cchName     Size of the buffer.
 * @thread  Any.
 */
VMMR3DECL(int) DBGFR3OSDetect(PVM pVM, char *pszName, size_t cchName)
{
    AssertPtrNullReturn(pszName, VERR_INVALID_POINTER);
    if (pszName && cchName)
        *pszName = '\0';

    /*
     * Pass it on to EMT(0).
     */
    return VMR3ReqPriorityCallWait(pVM, 0 /*idDstCpu*/, (PFNRT)dbgfR3OSDetect, 3, pVM, pszName, cchName);
}


/**
 * EMT worker function for DBGFR3OSQueryNameAndVersion
 *
 * @returns VBox status code.
 * @param   pVM             Pointer to the VM.
 * @param   pszName         Where to store the OS name. Optional.
 * @param   cchName         The size of the name buffer.
 * @param   pszVersion      Where to store the version string. Optional.
 * @param   cchVersion      The size of the version buffer.
 */
static DECLCALLBACK(int) dbgfR3OSQueryNameAndVersion(PVM pVM, char *pszName, size_t cchName, char *pszVersion, size_t cchVersion)
{
    /*
     * Any known OS?
     */
    if (pVM->dbgf.s.pCurOS)
    {
        int rc = VINF_SUCCESS;
        if (pszName && cchName)
        {
            size_t cch = strlen(pVM->dbgf.s.pCurOS->pReg->szName);
            if (cchName > cch)
                memcpy(pszName, pVM->dbgf.s.pCurOS->pReg->szName, cch + 1);
            else
            {
                memcpy(pszName, pVM->dbgf.s.pCurOS->pReg->szName, cchName - 1);
                pszName[cchName - 1] = '\0';
                rc = VINF_BUFFER_OVERFLOW;
            }
        }

        if (pszVersion && cchVersion)
        {
            int rc2 = pVM->dbgf.s.pCurOS->pReg->pfnQueryVersion(pVM, pVM->dbgf.s.pCurOS->abData, pszVersion, cchVersion);
            if (RT_FAILURE(rc2) || rc == VINF_SUCCESS)
                rc = rc2;
        }
        return rc;
    }

    return VERR_DBGF_OS_NOT_DETCTED;
}


/**
 * Queries the name and/or version string for the guest OS.
 *
 * It goes without saying that this querying is done using the current
 * guest OS digger and not additions or user configuration.
 *
 * @returns VBox status code.
 * @param   pVM             Pointer to the VM.
 * @param   pszName         Where to store the OS name. Optional.
 * @param   cchName         The size of the name buffer.
 * @param   pszVersion      Where to store the version string. Optional.
 * @param   cchVersion      The size of the version buffer.
 * @thread  Any.
 */
VMMR3DECL(int) DBGFR3OSQueryNameAndVersion(PVM pVM, char *pszName, size_t cchName, char *pszVersion, size_t cchVersion)
{
    AssertPtrNullReturn(pszName, VERR_INVALID_POINTER);
    AssertPtrNullReturn(pszVersion, VERR_INVALID_POINTER);

    /*
     * Initialize the output up front.
     */
    if (pszName && cchName)
        *pszName = '\0';
    if (pszVersion && cchVersion)
        *pszVersion = '\0';

    /*
     * Pass it on to EMT(0).
     */
    return VMR3ReqPriorityCallWait(pVM, 0 /*idDstCpu*/,
                                   (PFNRT)dbgfR3OSQueryNameAndVersion, 5, pVM, pszName, cchName, pszVersion, cchVersion);
}


/**
 * EMT worker for DBGFR3OSQueryInterface.
 *
 * @param   pVM         Pointer to the VM.
 * @param   enmIf       The interface identifier.
 * @param   ppvIf       Where to store the interface pointer on success.
 */
static DECLCALLBACK(void) dbgfR3OSQueryInterface(PVM pVM, DBGFOSINTERFACE enmIf, void **ppvIf)
{
    if (pVM->dbgf.s.pCurOS)
    {
        *ppvIf = pVM->dbgf.s.pCurOS->pReg->pfnQueryInterface(pVM, pVM->dbgf.s.pCurOS->abData, enmIf);
        if (*ppvIf)
        {
            /** @todo Create EMT wrapper for the returned interface once we've defined one...
             * Just keep a list of wrapper together with the OS instance. */
        }
    }
    else
        *ppvIf = NULL;
}


/**
 * Query an optional digger interface.
 *
 * @returns Pointer to the digger interface on success, NULL if the interfaces isn't
 *          available or no active guest OS digger.
 * @param   pVM         Pointer to the VM.
 * @param   enmIf       The interface identifier.
 * @thread  Any.
 */
VMMR3DECL(void *) DBGFR3OSQueryInterface(PVM pVM, DBGFOSINTERFACE enmIf)
{
    AssertMsgReturn(enmIf > DBGFOSINTERFACE_INVALID && enmIf < DBGFOSINTERFACE_END, ("%d\n", enmIf), NULL);

    /*
     * Pass it on to an EMT.
     */
    void *pvIf = NULL;
    VMR3ReqPriorityCallVoidWait(pVM, VMCPUID_ANY, (PFNRT)dbgfR3OSQueryInterface, 3, pVM, enmIf, &pvIf);
    return pvIf;
}

