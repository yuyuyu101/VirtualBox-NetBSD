/* $Id: ldrNative-win.cpp $ */
/** @file
 * IPRT - Binary Image Loader, Win32 native.
 */

/*
 * Copyright (C) 2006-2010 Oracle Corporation
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
#include <Windows.h>

#include <iprt/ldr.h>
#include <iprt/assert.h>
#include <iprt/path.h>
#include <iprt/err.h>
#include <iprt/alloca.h>
#include <iprt/string.h>
#include "internal/ldr.h"


int rtldrNativeLoad(const char *pszFilename, uintptr_t *phHandle, uint32_t fFlags, PRTERRINFO pErrInfo)
{
    Assert(sizeof(*phHandle) >= sizeof(HMODULE));
    AssertReturn(fFlags == 0, VERR_INVALID_PARAMETER);

    /*
     * Do we need to add an extension?
     */
    if (!RTPathHaveExt(pszFilename))
    {
        size_t cch = strlen(pszFilename);
        char *psz = (char *)alloca(cch + sizeof(".DLL"));
        if (!psz)
            return RTErrInfoSet(pErrInfo, VERR_NO_MEMORY, "alloca failed");
        memcpy(psz, pszFilename, cch);
        memcpy(psz + cch, ".DLL", sizeof(".DLL"));
        pszFilename = psz;
    }

    /*
     * Attempt load.
     */
    HMODULE hmod = LoadLibrary(pszFilename);
    if (hmod)
    {
        *phHandle = (uintptr_t)hmod;
        return VINF_SUCCESS;
    }

    /*
     * Try figure why it failed to load.
     */
    DWORD dwErr = GetLastError();
    int   rc    = RTErrConvertFromWin32(dwErr);
    return RTErrInfoSetF(pErrInfo, rc, "GetLastError=%u", dwErr);
}


DECLCALLBACK(int) rtldrNativeGetSymbol(PRTLDRMODINTERNAL pMod, const char *pszSymbol, void **ppvValue)
{
    PRTLDRMODNATIVE pModNative = (PRTLDRMODNATIVE)pMod;
    FARPROC pfn = GetProcAddress((HMODULE)pModNative->hNative, pszSymbol);
    if (pfn)
    {
        *ppvValue = (void *)pfn;
        return VINF_SUCCESS;
    }
    *ppvValue = NULL;
    return RTErrConvertFromWin32(GetLastError());
}


DECLCALLBACK(int) rtldrNativeClose(PRTLDRMODINTERNAL pMod)
{
    PRTLDRMODNATIVE pModNative = (PRTLDRMODNATIVE)pMod;
    if (FreeLibrary((HMODULE)pModNative->hNative))
    {
        pModNative->hNative = (uintptr_t)INVALID_HANDLE_VALUE;
        return VINF_SUCCESS;
    }
    return RTErrConvertFromWin32(GetLastError());
}

