/* $Id: tstGuestControlSvc.cpp $ */
/** @file
 *
 * Testcase for the guest control service.
 */

/*
 * Copyright (C) 2011 Oracle Corporation
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
#include <VBox/HostServices/GuestControlSvc.h>
#include <iprt/initterm.h>
#include <iprt/stream.h>
#include <iprt/test.h>

#include "../gctrl.h"

/*******************************************************************************
*   Global Variables                                                           *
*******************************************************************************/
static RTTEST g_hTest = NIL_RTTEST;

using namespace guestControl;

extern "C" DECLCALLBACK(DECLEXPORT(int)) VBoxHGCMSvcLoad(VBOXHGCMSVCFNTABLE *pTable);

/** Simple call handle structure for the guest call completion callback */
struct VBOXHGCMCALLHANDLE_TYPEDEF
{
    /** Where to store the result code. */
    int32_t rc;
};

/** Call completion callback for guest calls. */
static void callComplete(VBOXHGCMCALLHANDLE callHandle, int32_t rc)
{
    callHandle->rc = rc;
}

/**
 * Initialise the HGCM service table as much as we need to start the
 * service.
 *
 * @return IPRT status.
 * @param  pTable the table to initialise
 */
int initTable(VBOXHGCMSVCFNTABLE *pTable, VBOXHGCMSVCHELPERS *pHelpers)
{
    pTable->cbSize = sizeof (VBOXHGCMSVCFNTABLE);
    pTable->u32Version = VBOX_HGCM_SVC_VERSION;
    pHelpers->pfnCallComplete = callComplete;
    pTable->pHelpers = pHelpers;

    return VINF_SUCCESS;
}

typedef struct CMDHOST
{
    /** The HGCM command to execute. */
    int cmd;
    /** Number of parameters. */
    int num_parms;
    /** The actual parameters. */
    const PVBOXHGCMSVCPARM parms;
    /** Flag indicating whether we need a connected client for this command. */
    bool fNeedsClient;
    /** The desired return value from the host. */
    int rc;
} CMDHOST, *PCMDHOST;

typedef struct CMDCLIENT
{
    /** The client's ID. */
    int client_id;
    /** The HGCM command to execute. */
    int cmd;
    /** Number of parameters. */
    int num_parms;
    /** The actual parameters. */
    const PVBOXHGCMSVCPARM parms;
    /** The desired return value from the host. */
    int rc;
} CMDCLIENT, *PCMDCLIENT;

/**
 * Tests the HOST_EXEC_CMD function.
 * @returns iprt status value to indicate whether the test went as expected.
 * @note    prints its own diagnostic information to stdout.
 */
static int testHostCmd(const VBOXHGCMSVCFNTABLE *pTable, const PCMDHOST pCmd, uint32_t uNumTests)
{
    int rc = VINF_SUCCESS;
    if (!VALID_PTR(pTable->pfnHostCall))
    {
        RTTestPrintf(g_hTest, RTTESTLVL_FAILURE, "Invalid pfnHostCall() pointer\n");
        rc = VERR_INVALID_POINTER;
    }
    if (RT_SUCCESS(rc))
    {
        for (unsigned i = 0; (i < uNumTests) && RT_SUCCESS(rc); i++)
        {
            RTTestPrintf(g_hTest, RTTESTLVL_INFO, "Testing #%u (cmd: %d, num_parms: %d, parms: 0x%p\n",
                         i, pCmd[i].cmd, pCmd[i].num_parms, pCmd[i].parms);

            if (pCmd[i].fNeedsClient)
            {
                int client_rc = pTable->pfnConnect(pTable->pvService, 1000 /* Client ID */, NULL /* pvClient */);
                if (RT_FAILURE(client_rc))
                    rc = client_rc;
            }

            if (RT_SUCCESS(rc))
            {
                int host_rc = pTable->pfnHostCall(pTable->pvService,
                                                  pCmd[i].cmd,
                                                  pCmd[i].num_parms,
                                                  pCmd[i].parms);
                if (host_rc != pCmd[i].rc)
                {
                    RTTestPrintf(g_hTest, RTTESTLVL_FAILURE, "Host call test #%u returned with rc=%Rrc instead of rc=%Rrc\n",
                                 i, host_rc, pCmd[i].rc);
                    rc = host_rc;
                    if (RT_SUCCESS(rc))
                        rc = VERR_INVALID_PARAMETER;
                }

                if (pCmd[i].fNeedsClient)
                {
                    int client_rc = pTable->pfnDisconnect(pTable->pvService, 1000 /* Client ID */, NULL /* pvClient */);
                    if (RT_SUCCESS(rc))
                        rc = client_rc;
                }
            }
        }
    }
    return rc;
}

static int testHost(const VBOXHGCMSVCFNTABLE *pTable)
{
    RTTestSub(g_hTest, "Testing host commands ...");

    static VBOXHGCMSVCPARM s_aParms[1];
    s_aParms[0].setUInt32(1000 /* Context ID */);

    static CMDHOST s_aCmdHostAll[] =
    {
        /** No client connected, invalid command. */
        { 1024 /* Not existing command */, 0, 0, false, VERR_NOT_SUPPORTED },
        { -1 /* Invalid command */, 0, 0, false, VERR_NOT_SUPPORTED },
        { HOST_CANCEL_PENDING_WAITS, 1024, 0, false, VERR_NOT_FOUND },
        { HOST_CANCEL_PENDING_WAITS, 0, &s_aParms[0], false, VERR_NOT_FOUND },

        /** No client connected, valid command. */
        { HOST_CANCEL_PENDING_WAITS, 0, 0, false, VERR_NOT_FOUND },

        /** Client connected. */
        { 1024 /* Not existing command */, 0, 0, true, VERR_NOT_SUPPORTED },
        { -1 /* Invalid command */, 0, 0, true, VERR_NOT_SUPPORTED },

        /** Client connected, valid parameters given. */
        { HOST_CANCEL_PENDING_WAITS, 0, 0, true, VINF_SUCCESS },
        { HOST_CANCEL_PENDING_WAITS, 1024, &s_aParms[0], true, VINF_SUCCESS },
        { HOST_CANCEL_PENDING_WAITS, 0, &s_aParms[0], true, VINF_SUCCESS},

        /** Client connected, invalid parameters given. */
        { HOST_CANCEL_PENDING_WAITS, 1024, 0, true, VERR_INVALID_POINTER },
        { HOST_CANCEL_PENDING_WAITS, 1, 0, true, VERR_INVALID_POINTER },
        { HOST_CANCEL_PENDING_WAITS, -1, 0, true, VERR_INVALID_POINTER },

        /** Client connected, parameters given. */
        { HOST_CANCEL_PENDING_WAITS, 1, &s_aParms[0], true, VINF_SUCCESS },
        { HOST_EXEC_CMD, 1, &s_aParms[0], true, VINF_SUCCESS },
        { HOST_EXEC_SET_INPUT, 1, &s_aParms[0], true, VINF_SUCCESS },
        { HOST_EXEC_GET_OUTPUT, 1, &s_aParms[0], true, VINF_SUCCESS }
    };

    int rc = testHostCmd(pTable, &s_aCmdHostAll[0], RT_ELEMENTS(s_aCmdHostAll));
    RTTestSubDone(g_hTest);
    return rc;
}

static int testClient(const VBOXHGCMSVCFNTABLE *pTable)
{
    RTTestSub(g_hTest, "Testing client commands ...");

    int rc = pTable->pfnConnect(pTable->pvService, 1 /* Client ID */, NULL /* pvClient */);
    if (RT_SUCCESS(rc))
    {
        VBOXHGCMCALLHANDLE_TYPEDEF callHandle = { VINF_SUCCESS };

        /* No commands from host yet. */
        static VBOXHGCMSVCPARM s_aParmsGuest[8];
        pTable->pfnCall(pTable->pvService, &callHandle, 1 /* Client ID */, NULL /* pvClient */,
                        GUEST_GET_HOST_MSG, 2, &s_aParmsGuest[0]);
        RTTEST_CHECK_RC_RET(g_hTest, callHandle.rc, VINF_SUCCESS, callHandle.rc);

        /* Host: Add a dummy command. */
        static VBOXHGCMSVCPARM s_aParmsHost[8];
        s_aParmsHost[0].setUInt32(1000 /* Context ID */);
        s_aParmsHost[1].setString("foo.bar");

        rc = pTable->pfnHostCall(pTable->pvService, HOST_EXEC_CMD, 2, &s_aParmsHost[0]);
        RTTEST_CHECK_RC_RET(g_hTest, rc, VINF_SUCCESS, rc);

        /* Client: Get host command with a invalid parameter count specified. */
        pTable->pfnCall(pTable->pvService, &callHandle, 1 /* Client ID */, NULL /* pvClient */,
                        GUEST_GET_HOST_MSG, 1024, &s_aParmsGuest[0]);
        RTTEST_CHECK_RC_RET(g_hTest, callHandle.rc, VERR_INVALID_PARAMETER, callHandle.rc);
        pTable->pfnCall(pTable->pvService, &callHandle, 1 /* Client ID */, NULL /* pvClient */,
                        GUEST_GET_HOST_MSG, -1, &s_aParmsGuest[0]);
        RTTEST_CHECK_RC_RET(g_hTest, callHandle.rc, VERR_INVALID_PARAMETER, callHandle.rc);
        pTable->pfnCall(pTable->pvService, &callHandle, 1 /* Client ID */, NULL /* pvClient */,
                        GUEST_GET_HOST_MSG, -1, NULL);
        RTTEST_CHECK_RC_RET(g_hTest, callHandle.rc, VERR_INVALID_PARAMETER, callHandle.rc);
        pTable->pfnCall(pTable->pvService, &callHandle, 1 /* Client ID */, NULL /* pvClient */,
                        GUEST_GET_HOST_MSG, 16, NULL);
        RTTEST_CHECK_RC_RET(g_hTest, callHandle.rc, VERR_INVALID_PARAMETER, callHandle.rc);

        /* Client: Get host command with a too small HGCM array. */
        pTable->pfnCall(pTable->pvService, &callHandle, 1 /* Client ID */, NULL /* pvClient */,
                        GUEST_GET_HOST_MSG, 0, &s_aParmsGuest[0]);
        RTTEST_CHECK_RC_RET(g_hTest, callHandle.rc, VERR_TOO_MUCH_DATA, callHandle.rc);
        pTable->pfnCall(pTable->pvService, &callHandle, 1 /* Client ID */, NULL /* pvClient */,
                        GUEST_GET_HOST_MSG, 1, &s_aParmsGuest[0]);
        RTTEST_CHECK_RC_RET(g_hTest, callHandle.rc, VERR_TOO_MUCH_DATA, callHandle.rc);

        /* Client: Get host command without an allocated buffer. */
        pTable->pfnCall(pTable->pvService, &callHandle, 1 /* Client ID */, NULL /* pvClient */,
                        GUEST_GET_HOST_MSG, 2, &s_aParmsGuest[0]);
        RTTEST_CHECK_RC_RET(g_hTest, callHandle.rc, VERR_BUFFER_OVERFLOW, callHandle.rc);

        /* Client: Get host command, this time with a valid buffer. */
        char szBuf[16];
        s_aParmsGuest[1].setPointer(szBuf, sizeof(szBuf));
        pTable->pfnCall(pTable->pvService, &callHandle, 1 /* Client ID */, NULL /* pvClient */,
                        GUEST_GET_HOST_MSG, 2, &s_aParmsGuest[0]);
        RTTEST_CHECK_RC_RET(g_hTest, callHandle.rc, VINF_SUCCESS, callHandle.rc);

        /* Client: Now make sure there's no host message left anymore. */
        pTable->pfnCall(pTable->pvService, &callHandle, 1 /* Client ID */, NULL /* pvClient */,
                        GUEST_GET_HOST_MSG, 2, &s_aParmsGuest[0]);
        RTTEST_CHECK_RC_RET(g_hTest, callHandle.rc, VINF_SUCCESS, callHandle.rc);

        /* Client: Disconnect again. */
        int rc2 = pTable->pfnDisconnect(pTable->pvService, 1000 /* Client ID */, NULL /* pvClient */);
        if (RT_SUCCESS(rc))
            rc = rc2;
    }

    RTTestSubDone(g_hTest);
    return rc;
}

/*
 * Set environment variable "IPRT_TEST_MAX_LEVEL=all" to get more debug output!
 */
int main(int argc, char **argv)
{
    RTEXITCODE rcExit  = RTTestInitAndCreate("tstGuestControlSvc", &g_hTest);
    if (rcExit != RTEXITCODE_SUCCESS)
        return rcExit;
    RTTestBanner(g_hTest);

    /* Some host info. */
    RTTestIPrintf(RTTESTLVL_ALWAYS, "sizeof(void*)=%d\n", sizeof(void*));

    /* Do the tests. */
    VBOXHGCMSVCFNTABLE svcTable;
    VBOXHGCMSVCHELPERS svcHelpers;
    RTTEST_CHECK_RC_RET(g_hTest, initTable(&svcTable, &svcHelpers), VINF_SUCCESS, 1);

    do
    {
        RTTESTI_CHECK_RC_BREAK(VBoxHGCMSvcLoad(&svcTable), VINF_SUCCESS);

        RTTESTI_CHECK_RC_BREAK(testHost(&svcTable), VINF_SUCCESS);

        RTTESTI_CHECK_RC_BREAK(svcTable.pfnUnload(svcTable.pvService), VINF_SUCCESS);

        RTTESTI_CHECK_RC_BREAK(VBoxHGCMSvcLoad(&svcTable), VINF_SUCCESS);

        RTTESTI_CHECK_RC_BREAK(testClient(&svcTable), VINF_SUCCESS);

        RTTESTI_CHECK_RC_BREAK(svcTable.pfnUnload(svcTable.pvService), VINF_SUCCESS);

    } while (0);

    return RTTestSummaryAndDestroy(g_hTest);
}

