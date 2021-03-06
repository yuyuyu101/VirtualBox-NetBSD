/* $Id: ldrNative.cpp $ */
/** @file
 * IPRT - Binary Image Loader, Native interface.
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
#include <iprt/assert.h>
#include <iprt/log.h>
#include <iprt/param.h>
#include <iprt/path.h>
#include <iprt/string.h>
#include <iprt/err.h>
#include "internal/ldr.h"


/** @copydoc RTLDROPS::pfnEnumSymbols */
static DECLCALLBACK(int) rtldrNativeEnumSymbols(PRTLDRMODINTERNAL pMod, unsigned fFlags, const void *pvBits,
                                                RTUINTPTR BaseAddress, PFNRTLDRENUMSYMS pfnCallback, void *pvUser)
{
    NOREF(pMod); NOREF(fFlags); NOREF(pvBits); NOREF(BaseAddress); NOREF(pfnCallback); NOREF(pvUser);
    return VERR_NOT_SUPPORTED;
}


/** @copydoc RTLDROPS::pfnDone */
static DECLCALLBACK(int) rtldrNativeDone(PRTLDRMODINTERNAL pMod)
{
    NOREF(pMod);
    return VINF_SUCCESS;
}


/**
 * Operations for a native module.
 */
static const RTLDROPS s_rtldrNativeOps =
{
    "native",
    rtldrNativeClose,
    rtldrNativeGetSymbol,
    rtldrNativeDone,
    rtldrNativeEnumSymbols,
    /* ext: */
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    42
};



/**
 * Loads a dynamic load library (/shared object) image file using native
 * OS facilities.
 *
 * The filename will be appended the default DLL/SO extension of
 * the platform if it have been omitted. This means that it's not
 * possible to load DLLs/SOs with no extension using this interface,
 * but that's not a bad tradeoff.
 *
 * If no path is specified in the filename, the OS will usually search it's library
 * path to find the image file.
 *
 * @returns iprt status code.
 * @param   pszFilename Image filename.
 * @param   phLdrMod    Where to store the handle to the loaded module.
 */
RTDECL(int) RTLdrLoad(const char *pszFilename, PRTLDRMOD phLdrMod)
{
    return RTLdrLoadEx(pszFilename, phLdrMod, RTLDRLOAD_FLAGS_LOCAL, NULL);
}
RT_EXPORT_SYMBOL(RTLdrLoad);


RTDECL(int) RTLdrLoadEx(const char *pszFilename, PRTLDRMOD phLdrMod, uint32_t fFlags, PRTERRINFO pErrInfo)
{
    LogFlow(("RTLdrLoadEx: pszFilename=%p:{%s} phLdrMod=%p fFlags=%#x pErrInfo=%p\n", pszFilename, pszFilename, phLdrMod, fFlags, pErrInfo));

    /*
     * Validate and massage the input.
     */
    RTErrInfoClear(pErrInfo);
    AssertPtrReturn(pszFilename, VERR_INVALID_POINTER);
    AssertPtrReturn(phLdrMod, VERR_INVALID_POINTER);
    AssertReturn(!(fFlags & ~RTLDRLOAD_FLAGS_VALID_MASK), VERR_INVALID_PARAMETER);

    /*
     * Allocate and initialize module structure.
     */
    int rc = VERR_NO_MEMORY;
    PRTLDRMODNATIVE pMod = (PRTLDRMODNATIVE)RTMemAlloc(sizeof(*pMod));
    if (pMod)
    {
        pMod->Core.u32Magic = RTLDRMOD_MAGIC;
        pMod->Core.eState   = LDR_STATE_LOADED;
        pMod->Core.pOps     = &s_rtldrNativeOps;
        pMod->hNative       = ~(uintptr_t)0;

        /*
         * Attempt to open the module.
         */
        rc = rtldrNativeLoad(pszFilename, &pMod->hNative, fFlags, pErrInfo);
        if (RT_SUCCESS(rc))
        {
            *phLdrMod = &pMod->Core;
            LogFlow(("RTLdrLoad: returns %Rrc *phLdrMod=%RTldrm\n", rc, *phLdrMod));
            return rc;
        }

        RTMemFree(pMod);
    }
    else
        RTErrInfoSetF(pErrInfo, rc, "Failed to allocate %zu bytes for the module handle", sizeof(*pMod));
    *phLdrMod = NIL_RTLDRMOD;
    LogFlow(("RTLdrLoad: returns %Rrc\n", rc));
    return rc;
}
RT_EXPORT_SYMBOL(RTLdrLoadEx);


/**
 * Loads a dynamic load library (/shared object) image file residing in the
 * RTPathAppPrivateArch() directory.
 *
 * Suffix is not required.
 *
 * @returns iprt status code.
 * @param   pszFilename Image filename. No path.
 * @param   phLdrMod    Where to store the handle to the loaded module.
 */
RTDECL(int) RTLdrLoadAppPriv(const char *pszFilename, PRTLDRMOD phLdrMod)
{
    LogFlow(("RTLdrLoadAppPriv: pszFilename=%p:{%s} phLdrMod=%p\n", pszFilename, pszFilename, phLdrMod));

    /*
     * Validate input.
     */
    AssertPtrReturn(phLdrMod, VERR_INVALID_PARAMETER);
    *phLdrMod = NIL_RTLDRMOD;
    AssertPtrReturn(pszFilename, VERR_INVALID_PARAMETER);
    AssertMsgReturn(!RTPathHavePath(pszFilename), ("%s\n", pszFilename), VERR_INVALID_PARAMETER);

    /*
     * Check the filename.
     */
    size_t cchFilename = strlen(pszFilename);
    AssertMsgReturn(cchFilename < (RTPATH_MAX / 4) * 3, ("%zu\n", cchFilename), VERR_INVALID_PARAMETER);

    const char *pszExt = "";
    size_t cchExt = 0;
    if (!RTPathHaveExt(pszFilename))
    {
        pszExt = RTLdrGetSuff();
        cchExt = strlen(pszExt);
    }

    /*
     * Construct the private arch path and check if the file exists.
     */
    char szPath[RTPATH_MAX];
    int rc = RTPathAppPrivateArch(szPath, sizeof(szPath) - 1 - cchExt - cchFilename);
    AssertRCReturn(rc, rc);

    char *psz = strchr(szPath, '\0');
    *psz++ = RTPATH_SLASH;
    memcpy(psz, pszFilename, cchFilename);
    psz += cchFilename;
    memcpy(psz, pszExt, cchExt + 1);

    if (!RTPathExists(szPath))
    {
        LogRel(("RTLdrLoadAppPriv: \"%s\" not found\n", szPath));
        return VERR_FILE_NOT_FOUND;
    }

    /*
     * Pass it on to RTLdrLoad.
     */
    rc = RTLdrLoad(szPath, phLdrMod);

    LogFlow(("RTLdrLoadAppPriv: returns %Rrc\n", rc));
    return rc;
}
RT_EXPORT_SYMBOL(RTLdrLoadAppPriv);


/**
 * Gets the default file suffix for DLL/SO/DYLIB/whatever.
 *
 * @returns The stuff (readonly).
 */
RTDECL(const char *) RTLdrGetSuff(void)
{
#if defined(RT_OS_OS2) || defined(RT_OS_WINDOWS)
    static const char s_szSuff[] = ".DLL";
#elif defined(RT_OS_L4)
    static const char s_szSuff[] = ".s.so";
#elif defined(RT_OS_DARWIN)
    static const char s_szSuff[] = ".dylib";
#else
    static const char s_szSuff[] = ".so";
#endif

    return s_szSuff;
}
RT_EXPORT_SYMBOL(RTLdrGetSuff);

