/* $Id: em.h $ */
/** @file
 * EM - Internal VMM header file.
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
 */

#ifndef ___EM_include_internal_h
#define ___EM_include_internal_h

#include <VBox/vmm/em.h>

VMMR3DECL(int)  EMR3NotifyResume(PVM pVM);
VMMR3DECL(int)  EMR3NotifySuspend(PVM pVM);

VMMR3DECL(bool) EMR3IsExecutionAllowed(PVM pVM, PVMCPU pVCpu);

#endif
