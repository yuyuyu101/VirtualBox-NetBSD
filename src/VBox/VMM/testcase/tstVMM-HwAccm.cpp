/* $Id: tstVMM-HwAccm.cpp $ */
/** @file
 * VMM Testcase.
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
 */


/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#include <VBox/vmm/vm.h>
#include <VBox/vmm/vmm.h>
#include <VBox/vmm/cpum.h>
#include <VBox/err.h>
#include <VBox/log.h>
#include <iprt/assert.h>
#include <iprt/initterm.h>
#include <iprt/semaphore.h>
#include <iprt/stream.h>


/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
#define TESTCASE    "tstVMM-HwAccm"

VMMR3DECL(int) VMMDoHwAccmTest(PVM pVM);


static DECLCALLBACK(int) CFGMConstructor(PVM pVM, void *pvUser)
{
    NOREF(pvUser);

    /*
     * Get root node first.
     * This is the only node in the tree.
     */
    PCFGMNODE pRoot = CFGMR3GetRoot(pVM);
    int rc = CFGMR3InsertInteger(pRoot, "RamSize", 32*1024*1024);
    AssertRC(rc);

    /* rc = CFGMR3InsertInteger(pRoot, "EnableNestedPaging", false);
    AssertRC(rc); */

    PCFGMNODE pHWVirtExt;
    rc = CFGMR3InsertNode(pRoot, "HWVirtExt", &pHWVirtExt);
    AssertRC(rc);
    rc = CFGMR3InsertInteger(pHWVirtExt, "Enabled", 1);
    AssertRC(rc);

    return VINF_SUCCESS;
}

int main(int argc, char **argv)
{
    int     rcRet = 0;                  /* error count. */

    RTR3InitExe(argc, &argv, RTR3INIT_FLAGS_SUPLIB);

    /*
     * Doesn't work and I'm sick of rebooting the machine to try figure out
     * what the heck is going wrong. (Linux sucks at this)
     */
    RTPrintf(TESTCASE ": This testcase hits a bunch of breakpoint assertions which\n"
             TESTCASE ": causes kernel panics on linux regardless of what\n"
             TESTCASE ": RTAssertDoBreakpoint returns. Only checked AMD-V on linux.\n");
    /** @todo Make tstVMM-HwAccm to cause kernel panics. */
    return 1;

    /*
     * Create empty VM.
     */
    RTPrintf(TESTCASE ": Initializing...\n");
    PVM pVM;
    int rc = VMR3Create(1, NULL, NULL, NULL, CFGMConstructor, NULL, &pVM);
    if (RT_SUCCESS(rc))
    {
        /*
         * Do testing.
         */
        RTPrintf(TESTCASE ": Testing...\n");
        rc = VMR3ReqCallWait(pVM, VMCPUID_ANY, (PFNRT)VMMDoHwAccmTest, 1, pVM);
        AssertRC(rc);

        STAMR3Dump(pVM, "*");

        /*
         * Cleanup.
         */
        rc = VMR3Destroy(pVM);
        if (RT_FAILURE(rc))
        {
            RTPrintf(TESTCASE ": error: failed to destroy vm! rc=%d\n", rc);
            rcRet++;
        }
    }
    else
    {
        RTPrintf(TESTCASE ": fatal error: failed to create vm! rc=%d\n", rc);
        rcRet++;
    }

    return rcRet;
}
