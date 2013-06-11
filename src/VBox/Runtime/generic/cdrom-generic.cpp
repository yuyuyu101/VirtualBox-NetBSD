/* $Id: cdrom-generic.cpp $ */
/** @file
 * IPRT - CD/DVD/BD-ROM Drive, Generic.
 */

/*
 * Copyright (C) 2012 Oracle Corporation
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
#define RTCRITSECT_WITHOUT_REMAPPING
#include <iprt/cdrom.h>
#include "internal/iprt.h"

#include <iprt/assert.h>
#include <iprt/err.h>


RTDECL(int) RTCdromOpen(const char *psz, uint32_t fFlags, PRTCDROM phCdrom)
{
    return VERR_NOT_IMPLEMENTED;
}


RTDECL(uint32_t) RTCdromRetain(RTCDROM hCdrom)
{
    AssertFailedReturn(UINT32_MAX);
}


RTDECL(uint32_t)    RTCdromRelease(RTCDROM hCdrom)
{
    AssertFailedReturn(UINT32_MAX);
}


RTDECL(int) RTCdromQueryMountPoint(RTCDROM hCdrom, char *pszMountPoint, size_t cbMountPoint)
{
    AssertFailedReturn(VERR_NOT_IMPLEMENTED);
}


RTDECL(int)         RTCdromUnmount(RTCDROM hCdrom)
{
    AssertFailedReturn(VERR_NOT_IMPLEMENTED);
}


RTDECL(int)         RTCdromEject(RTCDROM hCdrom, bool fForce)
{
    AssertFailedReturn(VERR_NOT_IMPLEMENTED);
}


RTDECL(int)         RTCdromLock(RTCDROM hCdrom)
{
    AssertFailedReturn(VERR_NOT_IMPLEMENTED);
}


RTDECL(int)         RTCdromUnlock(RTCDROM hCdrom)
{
    AssertFailedReturn(VERR_NOT_IMPLEMENTED);
}


RTDECL(unsigned)    RTCdromCount(void)
{
    return 0;
}

RTDECL(int)         RTCdromOrdinalToName(unsigned iCdrom, char *pszName, size_t cbName)
{
    if (cbName)
        *pszName = '\0';
    return VERR_OUT_OF_RANGE;
}


RTDECL(int)         RTCdromOpenByOrdinal(unsigned iCdrom, uint32_t fFlags, PRTCDROM phCdrom)
{
    return VERR_OUT_OF_RANGE;
}

