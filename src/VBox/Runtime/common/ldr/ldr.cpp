/* $Id: ldr.cpp $ */
/** @file
 * IPRT - Binary Image Loader.
 */

/*
 * Copyright (C) 2006-2011 Oracle Corporation
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
#define LOG_GROUP RTLOGGROUP_LDR
#include <iprt/ldr.h>
#include "internal/iprt.h"

#include <iprt/alloc.h>
#include <iprt/string.h>
#include <iprt/assert.h>
#include <iprt/err.h>
#include <iprt/log.h>
#include "internal/ldr.h"


/**
 * Checks if a library is loadable or not.
 *
 * This may attempt load and unload the library.
 *
 * @returns true/false accordingly.
 * @param   pszFilename     Image filename.
 */
RTDECL(bool) RTLdrIsLoadable(const char *pszFilename)
{
    /*
     * Try to load the library.
     */
    RTLDRMOD hLib;
    int rc = RTLdrLoad(pszFilename, &hLib);
    if (RT_SUCCESS(rc))
    {
        RTLdrClose(hLib);
        return true;
    }
    return false;
}
RT_EXPORT_SYMBOL(RTLdrIsLoadable);


/**
 * Gets the address of a named exported symbol.
 *
 * @returns iprt status code.
 * @param   hLdrMod         The loader module handle.
 * @param   pszSymbol       Symbol name.
 * @param   ppvValue        Where to store the symbol value. Note that this is restricted to the
 *                          pointer size used on the host!
 */
RTDECL(int) RTLdrGetSymbol(RTLDRMOD hLdrMod, const char *pszSymbol, void **ppvValue)
{
    LogFlow(("RTLdrGetSymbol: hLdrMod=%RTldrm pszSymbol=%p:{%s} ppvValue=%p\n",
             hLdrMod, pszSymbol, pszSymbol, ppvValue));
    /*
     * Validate input.
     */
    AssertMsgReturn(rtldrIsValid(hLdrMod), ("hLdrMod=%p\n", hLdrMod), VERR_INVALID_HANDLE);
    AssertMsgReturn(pszSymbol, ("pszSymbol=%p\n", pszSymbol), VERR_INVALID_PARAMETER);
    AssertMsgReturn(VALID_PTR(ppvValue), ("ppvValue=%p\n", ppvValue), VERR_INVALID_PARAMETER);
    PRTLDRMODINTERNAL pMod = (PRTLDRMODINTERNAL)hLdrMod;
    //AssertMsgReturn(pMod->eState == LDR_STATE_OPENED, ("eState=%d\n", pMod->eState), VERR_WRONG_ORDER);

    /*
     * Do it.
     */
    int rc;
    if (pMod->pOps->pfnGetSymbol)
        rc = pMod->pOps->pfnGetSymbol(pMod, pszSymbol, ppvValue);
    else
    {
        RTUINTPTR Value = 0;
        rc = pMod->pOps->pfnGetSymbolEx(pMod, NULL, 0, pszSymbol, &Value);
        if (RT_SUCCESS(rc))
        {
            *ppvValue = (void *)(uintptr_t)Value;
            if ((uintptr_t)*ppvValue != Value)
                rc = VERR_BUFFER_OVERFLOW;
        }
    }
    LogFlow(("RTLdrGetSymbol: return %Rrc *ppvValue=%p\n", rc, *ppvValue));
    return rc;
}
RT_EXPORT_SYMBOL(RTLdrGetSymbol);


/**
 * Closes a loader module handle.
 *
 * The handle can be obtained using any of the RTLdrLoad(), RTLdrOpen()
 * and RTLdrOpenBits() functions.
 *
 * @returns iprt status code.
 * @param   hLdrMod         The loader module handle.
 */
RTDECL(int) RTLdrClose(RTLDRMOD hLdrMod)
{
    LogFlow(("RTLdrClose: hLdrMod=%RTldrm\n", hLdrMod));

    /*
     * Validate input.
     */
    AssertMsgReturn(rtldrIsValid(hLdrMod), ("hLdrMod=%p\n", hLdrMod), VERR_INVALID_HANDLE);
    PRTLDRMODINTERNAL pMod = (PRTLDRMODINTERNAL)hLdrMod;
    //AssertMsgReturn(pMod->eState == LDR_STATE_OPENED, ("eState=%d\n", pMod->eState), VERR_WRONG_ORDER);

    /*
     * Do it.
     */
    int rc = pMod->pOps->pfnClose(pMod);
    AssertRC(rc);
    pMod->eState = LDR_STATE_INVALID;
    pMod->u32Magic++;
    RTMemFree(pMod);

    LogFlow(("RTLdrClose: returns VINF_SUCCESS\n"));
    return VINF_SUCCESS;
}
RT_EXPORT_SYMBOL(RTLdrClose);

