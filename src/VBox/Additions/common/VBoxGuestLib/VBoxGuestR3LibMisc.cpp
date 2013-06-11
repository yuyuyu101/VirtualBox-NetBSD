/* $Id: VBoxGuestR3LibMisc.cpp $ */
/** @file
 * VBoxGuestR3Lib - Ring-3 Support Library for VirtualBox guest additions, Misc.
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
#include <VBox/log.h>
#include "VBGLR3Internal.h"


/**
 * Change the IRQ filter mask.
 *
 * @returns IPRT status code.
 * @param   fOr     The OR mask.
 * @param   fNo     The NOT mask.
 */
VBGLR3DECL(int) VbglR3CtlFilterMask(uint32_t fOr, uint32_t fNot)
{
    VBoxGuestFilterMaskInfo Info;
    Info.u32OrMask = fOr;
    Info.u32NotMask = fNot;
    return vbglR3DoIOCtl(VBOXGUEST_IOCTL_CTL_FILTER_MASK, &Info, sizeof(Info));
}


/**
 * Report a change in the capabilities that we support to the host.
 *
 * @returns IPRT status code.
 * @param   fOr     Capabilities which have been added.
 * @param   fNot    Capabilities which have been removed.
 *
 * @todo    Move to a different file.
 */
VBGLR3DECL(int) VbglR3SetGuestCaps(uint32_t fOr, uint32_t fNot)
{
    VMMDevReqGuestCapabilities2 Req;

    vmmdevInitRequest(&Req.header, VMMDevReq_SetGuestCapabilities);
    Req.u32OrMask = fOr;
    Req.u32NotMask = fNot;
    int rc = vbglR3GRPerform(&Req.header);
#if defined(DEBUG)
    if (RT_SUCCESS(rc))
        LogRel(("Successfully changed guest capabilities: or mask 0x%x, not mask 0x%x.\n", fOr, fNot));
    else
        LogRel(("Failed to change guest capabilities: or mask 0x%x, not mask 0x%x.  rc=%Rrc.\n", fOr, fNot, rc));
#endif
    return rc;
}


/**
 * Query the session ID of this VM.
 *
 * The session id is an unique identifier that gets changed for each VM start,
 * reset or restore.  Useful for detection a VM restore.
 *
 * @returns IPRT status code.
 * @param   pu64IdSession       Session id (out).  This is NOT changed on
 *                              failure, so the caller can depend on this to
 *                              deal with backward compatibility (see
 *                              VBoxServiceVMInfoWorker() for an example.)
 */
VBGLR3DECL(int) VbglR3GetSessionId(uint64_t *pu64IdSession)
{
    VMMDevReqSessionId Req;

    vmmdevInitRequest(&Req.header, VMMDevReq_GetSessionId);
    Req.idSession = 0;
    int rc = vbglR3GRPerform(&Req.header);
    if (RT_SUCCESS(rc))
        *pu64IdSession = Req.idSession;

    return rc;
}
