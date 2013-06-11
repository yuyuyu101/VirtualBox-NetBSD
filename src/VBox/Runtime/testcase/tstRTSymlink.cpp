/* $Id: tstRTSymlink.cpp $ */
/** @file
 * IPRT Testcase - Symbolic Links.
 */

/*
 * Copyright (C) 2010-2011 Oracle Corporation
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
#include <iprt/symlink.h>

#include <iprt/test.h>
#include <iprt/dir.h>
#include <iprt/err.h>
#include <iprt/file.h>
#include <iprt/param.h>
#include <iprt/path.h>
#include <iprt/process.h>
#include <iprt/string.h>
#include <iprt/initterm.h>


static void test1Worker(RTTEST hTest, const char *pszBaseDir,
                        const char *pszTarget, RTSYMLINKTYPE enmType, bool fDangling)
{
    char    szPath1[RTPATH_MAX];
    char    szPath2[RTPATH_MAX];
    size_t  cchTarget = strlen(pszTarget);

    /* Create it.*/
    RTTESTI_CHECK_RC_OK_RETV(RTPathJoin(szPath1, sizeof(szPath1), pszBaseDir, "tstRTSymlink-link-1"));
    RTSymlinkDelete(szPath1, 0); /* clean up previous run */
    RTTESTI_CHECK_RC_RETV(RTSymlinkCreate(szPath1, pszTarget, RTSYMLINKTYPE_FILE, 0), VINF_SUCCESS);

    /* Check the predicate functions. */
    RTTESTI_CHECK(RTSymlinkExists(szPath1));
    RTTESTI_CHECK(RTSymlinkIsDangling(szPath1) == fDangling);

    /* Read it. */
    memset(szPath2, 0xff, sizeof(szPath2));
    szPath2[sizeof(szPath2) - 1] = '\0';
    RTTESTI_CHECK_RC(RTSymlinkRead(szPath1, szPath2, sizeof(szPath2), 0), VINF_SUCCESS);
    RTTESTI_CHECK_MSG(strcmp(szPath2, pszTarget) == 0, ("got=\"%s\" expected=\"%s\"", szPath2, pszTarget));

    memset(szPath2, 0xff, sizeof(szPath2));
    szPath2[sizeof(szPath2) - 1] = '\0';
    RTTESTI_CHECK_RC(RTSymlinkRead(szPath1, szPath2, cchTarget + 1, 0), VINF_SUCCESS);
    RTTESTI_CHECK_MSG(strcmp(szPath2, pszTarget) == 0, ("got=\"%s\" expected=\"%s\"", szPath2, pszTarget));

    memset(szPath2, 0xff, sizeof(szPath2));
    szPath2[sizeof(szPath2) - 1] = '\0';
    RTTESTI_CHECK_RC(RTSymlinkRead(szPath1, szPath2, cchTarget, 0), VERR_BUFFER_OVERFLOW);
    RTTESTI_CHECK_MSG(   strncmp(szPath2, pszTarget, cchTarget - 1) == 0
                      && szPath2[cchTarget - 1] == '\0',
                      ("got=\"%s\" expected=\"%.*s\"", szPath2, cchTarget - 1, pszTarget));

    /* Other APIs that have to handle symlinks carefully. */
    int rc;
    RTFSOBJINFO ObjInfo;
    RTTESTI_CHECK_RC(rc = RTPathQueryInfo(szPath1, &ObjInfo, RTFSOBJATTRADD_NOTHING), VINF_SUCCESS);
    if (RT_SUCCESS(rc))
        RTTESTI_CHECK(RTFS_IS_SYMLINK(ObjInfo.Attr.fMode));
    RTTESTI_CHECK_RC(rc = RTPathQueryInfoEx(szPath1, &ObjInfo, RTFSOBJATTRADD_NOTHING, RTPATH_F_ON_LINK), VINF_SUCCESS);
    if (RT_SUCCESS(rc))
        RTTESTI_CHECK(RTFS_IS_SYMLINK(ObjInfo.Attr.fMode));

    if (!fDangling)
    {
        RTTESTI_CHECK_RC(rc = RTPathQueryInfoEx(szPath1, &ObjInfo, RTFSOBJATTRADD_NOTHING, RTPATH_F_FOLLOW_LINK), VINF_SUCCESS);
        if (RT_SUCCESS(rc))
            RTTESTI_CHECK(!RTFS_IS_SYMLINK(ObjInfo.Attr.fMode));
        else
            RT_ZERO(ObjInfo);

        if (enmType == RTSYMLINKTYPE_DIR)
        {
            RTTESTI_CHECK(RTDirExists(szPath1));
            RTTESTI_CHECK(RTFS_IS_DIRECTORY(ObjInfo.Attr.fMode));
        }
        else if (enmType == RTSYMLINKTYPE_FILE)
        {
            RTTESTI_CHECK(RTFileExists(szPath1));
            RTTESTI_CHECK(RTFS_IS_FILE(ObjInfo.Attr.fMode));
        }

        /** @todo Check more APIs */
    }

    /* Finally, the removal of the symlink. */
    RTTESTI_CHECK_RC(RTSymlinkDelete(szPath1, 0), VINF_SUCCESS);
    RTTESTI_CHECK_RC(RTSymlinkDelete(szPath1, 0), VERR_FILE_NOT_FOUND);
}


static void test1(RTTEST hTest, const char *pszBaseDir)
{
    char szPath1[RTPATH_MAX];
    char szPath2[RTPATH_MAX];

    /*
     * Making some assumptions about how we are executed from to start with...
     */
    RTTestISub("Negative RTSymlinkRead, RTSymlinkExists and RTSymlinkIsDangling");
    char szExecDir[RTPATH_MAX];
    RTTESTI_CHECK_RC_OK_RETV(RTPathExecDir(szExecDir, sizeof(szExecDir)));
    size_t cchExecDir = strlen(szExecDir);
    RTTESTI_CHECK(RTDirExists(szExecDir));

    char szExecFile[RTPATH_MAX];
    RTTESTI_CHECK_RETV(RTProcGetExecutablePath(szExecFile, sizeof(szExecFile)) != NULL);
    size_t cchExecFile = strlen(szExecFile);
    RTTESTI_CHECK(RTFileExists(szExecFile));

    RTTESTI_CHECK(!RTSymlinkExists(szExecFile));
    RTTESTI_CHECK(!RTSymlinkExists(szExecDir));
    RTTESTI_CHECK(!RTSymlinkIsDangling(szExecFile));
    RTTESTI_CHECK(!RTSymlinkIsDangling(szExecDir));
    RTTESTI_CHECK(!RTSymlinkExists("/"));
    RTTESTI_CHECK(!RTSymlinkIsDangling("/"));
    RTTESTI_CHECK(!RTSymlinkExists("/some/non-existing/directory/name/iprt"));
    RTTESTI_CHECK(!RTSymlinkExists("/some/non-existing/directory/name/iprt/"));
    RTTESTI_CHECK(!RTSymlinkIsDangling("/some/non-existing/directory/name/iprt"));
    RTTESTI_CHECK(!RTSymlinkIsDangling("/some/non-existing/directory/name/iprt/"));

    RTTESTI_CHECK_RC(RTSymlinkRead(szExecFile, szPath1, sizeof(szPath1), 0), VERR_NOT_SYMLINK);
    RTTESTI_CHECK_RC(RTSymlinkRead(szExecDir,  szPath1, sizeof(szPath1), 0), VERR_NOT_SYMLINK);

    /*
     * Do some symlinking.  ASSUME they are supported on the test file system.
     */
    RTTestISub("Basics");
    RTTESTI_CHECK_RETV(RTDirExists(pszBaseDir));
    test1Worker(hTest, pszBaseDir, szExecFile, RTSYMLINKTYPE_FILE,    false /*fDangling*/);
    test1Worker(hTest, pszBaseDir, szExecDir,  RTSYMLINKTYPE_DIR,     false /*fDangling*/);
    test1Worker(hTest, pszBaseDir, szExecFile, RTSYMLINKTYPE_UNKNOWN, false /*fDangling*/);
    test1Worker(hTest, pszBaseDir, szExecDir,  RTSYMLINKTYPE_UNKNOWN, false /*fDangling*/);

    /*
     * Create a few dangling links.
     */
    RTTestISub("Dangling links");
    test1Worker(hTest, pszBaseDir, "../dangle/dangle",  RTSYMLINKTYPE_FILE,     true /*fDangling*/);
    test1Worker(hTest, pszBaseDir, "../dangle/dangle",  RTSYMLINKTYPE_DIR,      true /*fDangling*/);
    test1Worker(hTest, pszBaseDir, "../dangle/dangle",  RTSYMLINKTYPE_UNKNOWN,  true /*fDangling*/);
    test1Worker(hTest, pszBaseDir, "../dangle/dangle/", RTSYMLINKTYPE_UNKNOWN,  true /*fDangling*/);
}


int main(int argc, char **argv)
{
    RTTEST hTest;
    int rc = RTTestInitAndCreate("tstRTSymlink", &hTest);
    if (rc)
        return rc;
    RTTestBanner(hTest);

    test1(hTest, ".");

    return RTTestSummaryAndDestroy(hTest);
}

