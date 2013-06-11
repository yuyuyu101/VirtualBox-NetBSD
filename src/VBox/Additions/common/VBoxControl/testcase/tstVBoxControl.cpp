/* $Id: tstVBoxControl.cpp $ */
/** @file
 * VBoxControl - Guest Additions Command Line Management Interface, test case
 */

/*
 * Copyright (C) 2007-2010 Oracle Corporation
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
#include <iprt/cpp/autores.h>
#include <iprt/initterm.h>
#include <iprt/mem.h>
#include <iprt/path.h>
#include <iprt/stream.h>
#include <iprt/string.h>
#include <VBox/log.h>
#include <VBox/version.h>
#include <VBox/VBoxGuestLib.h>
#ifdef VBOX_WITH_GUEST_PROPS
# include <VBox/HostServices/GuestPropertySvc.h>
#endif

VBGLR3DECL(int)     VbglR3Init(void)
{
    RTPrintf("Initialising guest library...\n");
    return VINF_SUCCESS;
}

VBGLR3DECL(int)     VbglR3GuestPropConnect(uint32_t *pu32ClientId)
{
    AssertPtrReturn(pu32ClientId, VERR_INVALID_POINTER);
    RTPrintf("Connect to guest property service...\n");
    *pu32ClientId = 1;
    return VINF_SUCCESS;
}

VBGLR3DECL(int)     VbglR3GuestPropDisconnect(uint32_t u32ClientId)
{
    RTPrintf("Disconnect client %d from guest property service...\n", u32ClientId);
    return VINF_SUCCESS;
}

VBGLR3DECL(int)     VbglR3GuestPropWrite(uint32_t u32ClientId,
                                         const char *pszName,
                                         const char *pszValue,
                                         const char *pszFlags)
{
    RTPrintf("Called SET_PROP, client %d, name %s, value %s, flags %s...\n",
             u32ClientId, pszName, pszValue, pszFlags);
    return VINF_SUCCESS;
}

VBGLR3DECL(int)     VbglR3GuestPropWriteValue(uint32_t u32ClientId,
                                              const char *pszName,
                                              const char *pszValue)
{
    RTPrintf("Called SET_PROP_VALUE, client %d, name %s, value %s...\n",
             u32ClientId, pszName, pszValue);
    return VINF_SUCCESS;
}

#ifdef VBOX_WITH_GUEST_PROPS
VBGLR3DECL(int)     VbglR3GuestPropRead(uint32_t u32ClientId,
                                        const char *pszName,
                                        void *pvBuf,
                                        uint32_t cbBuf,
                                        char **ppszValue,
                                        uint64_t *pu64Timestamp,
                                        char **ppszFlags,
                                        uint32_t *pcbBufActual)
{
    RTPrintf("Called GET_PROP, client %d, name %s...\n",
             u32ClientId, pszName);
    static char szValue[] = "Value";
    static char szFlags[] = "TRANSIENT";
    if (VALID_PTR(ppszValue))
        *ppszValue = szValue;
    if (VALID_PTR(pu64Timestamp))
        *pu64Timestamp = 12345;
    if (VALID_PTR(ppszFlags))
        *ppszFlags = szFlags;
    if (VALID_PTR(pcbBufActual))
        *pcbBufActual = 256;
    return VINF_SUCCESS;
}

struct VBGLR3GUESTPROPENUM
{
    uint32_t u32;
};

VBGLR3DECL(int)     VbglR3GuestPropEnum(uint32_t u32ClientId,
                                        char const * const *ppaszPatterns,
                                        uint32_t cPatterns,
                                        PVBGLR3GUESTPROPENUM *ppHandle,
                                        char const **ppszName,
                                        char const **ppszValue,
                                        uint64_t *pu64Timestamp,
                                        char const **ppszFlags)
{
    RTPrintf("Called ENUM_PROPS, client %d...\n", u32ClientId);
    AssertPtrReturn(ppHandle, VERR_INVALID_POINTER);
    static VBGLR3GUESTPROPENUM Handle = { 0 };
    static char szName[] = "Name";
    static char szValue[] = "Value";
    static char szFlags[] = "TRANSIENT";
    *ppHandle = &Handle;
    if (VALID_PTR(ppszName))
        *ppszName = szName;
    if (VALID_PTR(ppszValue))
        *ppszValue = szValue;
    if (VALID_PTR(pu64Timestamp))
        *pu64Timestamp = 12345;
    if (VALID_PTR(ppszFlags))
        *ppszFlags = szFlags;
    return VINF_SUCCESS;
}

VBGLR3DECL(int)     VbglR3GuestPropEnumNext(PVBGLR3GUESTPROPENUM pHandle,
                                            char const **ppszName,
                                            char const **ppszValue,
                                            uint64_t *pu64Timestamp,
                                            char const **ppszFlags)
{
    RTPrintf("Called enumerate next...\n");
    AssertReturn(VALID_PTR(ppszName) || VALID_PTR(ppszValue) || VALID_PTR(ppszFlags),
                 VERR_INVALID_POINTER);
    if (VALID_PTR(ppszName))
        *ppszName = NULL;
    if (VALID_PTR(ppszValue))
        *ppszValue = NULL;
    if (VALID_PTR(pu64Timestamp))
        *pu64Timestamp = 0;
    if (VALID_PTR(ppszFlags))
        *ppszFlags = NULL;
    return VINF_SUCCESS;
}

VBGLR3DECL(void)    VbglR3GuestPropEnumFree(PVBGLR3GUESTPROPENUM pHandle)
{
    RTPrintf("Called enumerate free...\n");
}

VBGLR3DECL(int)     VbglR3GuestPropWait(uint32_t u32ClientId,
                                        const char *pszPatterns,
                                        void *pvBuf,
                                        uint32_t cbBuf,
                                        uint64_t u64Timestamp,
                                        uint32_t u32Timeout,
                                        char ** ppszName,
                                        char **ppszValue,
                                        uint64_t *pu64Timestamp,
                                        char **ppszFlags,
                                        uint32_t *pcbBufActual)
{
    if (u32Timeout == RT_INDEFINITE_WAIT)
        RTPrintf("Called GET_NOTIFICATION, client %d, patterns %s, timestamp %llu,\n"
                 "    timeout RT_INDEFINITE_WAIT...\n",
                 u32ClientId, pszPatterns, u64Timestamp);
    else
        RTPrintf("Called GET_NOTIFICATION, client %d, patterns %s, timestamp %llu,\n"
                 "    timeout %u...\n",
                 u32ClientId, pszPatterns, u64Timestamp, u32Timeout);
    static char szName[] = "Name";
    static char szValue[] = "Value";
    static char szFlags[] = "TRANSIENT";
    if (VALID_PTR(ppszName))
        *ppszName = szName;
    if (VALID_PTR(ppszValue))
        *ppszValue = szValue;
    if (VALID_PTR(pu64Timestamp))
        *pu64Timestamp = 12345;
    if (VALID_PTR(ppszFlags))
        *ppszFlags = szFlags;
    if (VALID_PTR(pcbBufActual))
        *pcbBufActual = 256;
    return VINF_SUCCESS;
}

#endif
