/* $Id: tstCFGM.cpp $ */
/** @file
 * Testcase for CFGM.
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
#include <VBox/sup.h>
#include <VBox/vmm/cfgm.h>
#include <VBox/vmm/mm.h>
#include <VBox/vmm/vm.h>
#include <VBox/vmm/uvm.h>

#include <VBox/err.h>
#include <VBox/param.h>
#include <iprt/initterm.h>
#include <iprt/stream.h>
#include <iprt/string.h>


int main()
{
    /*
     * Init runtime.
     */
    RTR3InitExeNoArguments(RTR3INIT_FLAGS_SUPLIB);

    /*
     * Create empty VM structure and init SSM.
     */
    PVM         pVM;
    int rc = SUPR3Init(NULL);
    if (RT_SUCCESS(rc))
        rc = SUPR3PageAlloc(RT_ALIGN_Z(sizeof(*pVM), PAGE_SIZE) >> PAGE_SHIFT, (void **)&pVM);
    if (RT_FAILURE(rc))
    {
        RTPrintf("Fatal error: SUP Failure! rc=%Rrc\n", rc);
        return 1;
    }

    static UVM s_UVM;
    PUVM pUVM = &s_UVM;
    pUVM->pVM = pVM;
    pVM->pUVM = pUVM;

    rc = STAMR3InitUVM(pUVM);
    if (RT_FAILURE(rc))
    {
        RTPrintf("FAILURE: STAMR3Init failed. rc=%Rrc\n", rc);
        return 1;
    }

    rc = MMR3InitUVM(pUVM);
    if (RT_FAILURE(rc))
    {
        RTPrintf("FAILURE: STAMR3Init failed. rc=%Rrc\n", rc);
        return 1;
    }

    rc = CFGMR3Init(pVM, NULL, NULL);
    if (RT_FAILURE(rc))
    {
        RTPrintf("FAILURE: CFGMR3Init failed. rc=%Rrc\n", rc);
        return 1;
    }

    if (!CFGMR3GetRoot(pVM))
    {
        RTPrintf("FAILURE: CFGMR3GetRoot failed\n");
        return 1;
    }

    /* integer */
    uint64_t u64;
    rc = CFGMR3QueryU64(CFGMR3GetRoot(pVM), "RamSize", &u64);
    if (RT_FAILURE(rc))
    {
        RTPrintf("FAILURE: CFGMR3QueryU64(,\"RamSize\",) failed. rc=%Rrc\n", rc);
        return 1;
    }

    size_t cb;
    rc = CFGMR3QuerySize(CFGMR3GetRoot(pVM), "RamSize", &cb);
    if (RT_FAILURE(rc))
    {
        RTPrintf("FAILURE: CFGMR3QuerySize(,\"RamSize\",) failed. rc=%Rrc\n", rc);
        return 1;
    }
    if (cb != sizeof(uint64_t))
    {
        RTPrintf("FAILURE: Incorrect valuesize %d for \"RamSize\" value.\n", cb);
        return 1;
    }

    /* string */
    char *pszName = NULL;
    rc = CFGMR3QueryStringAlloc(CFGMR3GetRoot(pVM), "Name", &pszName);
    if (RT_FAILURE(rc))
    {
        RTPrintf("FAILURE: CFGMR3QueryStringAlloc(,\"Name\" failed. rc=%Rrc\n", rc);
        return 1;
    }

    rc = CFGMR3QuerySize(CFGMR3GetRoot(pVM), "Name", &cb);
    if (RT_FAILURE(rc))
    {
        RTPrintf("FAILURE: CFGMR3QuerySize(,\"RamSize\",) failed. rc=%Rrc\n", rc);
        return 1;
    }
    if (cb != strlen(pszName) + 1)
    {
        RTPrintf("FAILURE: Incorrect valuesize %d for \"Name\" value '%s'.\n", cb, pszName);
        return 1;
    }
    MMR3HeapFree(pszName);


    /* test multilevel node creation */
    PCFGMNODE pChild = NULL;
    rc = CFGMR3InsertNode(CFGMR3GetRoot(pVM), "First/Second/Third//Final", &pChild);
    if (RT_FAILURE(rc))
    {
        RTPrintf("FAILURE: CFGMR3InsertNode(,\"First/Second/Third//Final\" failed. rc=%Rrc\n", rc);
        return 1;
    }
    rc = CFGMR3InsertInteger(pChild, "BoolValue", 1);
    if (RT_FAILURE(rc))
    {
        RTPrintf("FAILURE: CFGMR3InsertInteger(,\"BoolValue\", 1) failed. rc=%Rrc\n", rc);
        return 1;
    }
    PCFGMNODE pNode = CFGMR3GetChild(CFGMR3GetRoot(pVM), "First/Second/Third/Final");
    if (pNode != pChild)
    {
        RTPrintf("FAILURE: CFGMR3GetChild(,\"First/Second/Third/Final/BoolValue\") failed. pNode=%p expected %p\n", pNode, pChild);
        return 1;
    }
    bool f = false;
    rc = CFGMR3QueryBool(pNode, "BoolValue", &f);
    if (RT_FAILURE(rc) || !f)
    {
        RTPrintf("FAILURE: CFGMR3QueryBool(,\"BoolValue\",) failed. rc=%Rrc f=%d\n", rc, f);
        return 1;
    }


    /* done */
    rc = CFGMR3Term(pVM);
    if (RT_FAILURE(rc))
    {
        RTPrintf("FAILURE: CFGMR3QueryU64(,\"RamSize\" failed. rc=%Rrc\n", rc);
        return 1;
    }

    RTPrintf("tstCFGM: SUCCESS\n");
    return rc;
}
