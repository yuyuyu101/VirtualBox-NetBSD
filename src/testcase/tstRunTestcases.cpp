/* $Id: tstRunTestcases.cpp $ */
/** @file
 * tstRunTestcases - Driver program for running VBox testcase (tst* testcase/tst*).
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
#include <iprt/initterm.h>
#include <iprt/dir.h>
#include <iprt/param.h>
#include <iprt/path.h>
#include <iprt/process.h>
#include <iprt/string.h>
#include <iprt/stream.h>
#include <iprt/thread.h>
#include <iprt/err.h>
#include <iprt/env.h>


/*******************************************************************************
*   Global Variables                                                           *
*******************************************************************************/
/** The number of passed testcases. */
static unsigned     g_cPasses = 0;
/** The number of failed testcases. */
static unsigned     g_cFailures = 0;
/** The number of skipped testcases. */
static unsigned     g_cSkipped = 0;
/** The exclude list.
 * @note Stripped extensions! */
static const char  *g_apszExclude[] =
{
#if 1 // slow stuff
    "testcase/tstFile",
    "testcase/tstRTAvl",
    "testcase/tstSemMutex",
    "testcase/tstVD",
#endif
    "testcase/tstFileLock",
    "testcase/tstCritSect",
    "testcase/tstCritSectW32",
    "testcase/tstDeadlock",
    "testcase/tstDisasm-2",
    "testcase/tstFileAppendWin-1",
    "testcase/tstDir",              /* useless, requires parameters */
    "testcase/tstDir-2",            /* useless, requires parameters */
    "testcase/tstGlobalConfig",
    "testcase/tstHostHardwareLinux", /* must be killed with CTRL-C */
    "testcase/tstLdr-2",
    "testcase/tstLdr-3",
    "testcase/tstLdr",
    "testcase/tstLdrLoad",
    "testcase/tstLdrObj",
    "testcase/tstLdrObjR0",
    "testcase/tstMove",
#if !defined(RT_OS_LINUX)
    "testcase/tstRTR0Timer",
#endif
    "testcase/tstRunTestcases",
    "testcase/tstRTS3",             /* requires parameters <access key>, <secret key> */
    "testcase/tstSDL",
    "testcase/tstTime-3",
    "testcase/tstSeamlessX11",
    "testcase/tstVBoxControl",
    "testcase/tstVDCopy",           /* requires parameters <hdd1>, <hdd2> */
    "./tstRunTestcases",
    "./tstAnimate",
    "./tstAPI",
    "./tstCollector",               /* takes forever */
    "./tstHeadless",
    "./tstHeadless2",
    "./tstMicro",
    "./tstMicroGC",
    "./tstVBoxDbg",
    "./tstVMM-2",
    "./tstTestServMgr",
    "./tstPDMAsyncCompletion",      /* requires parameters <source>, <dest> */
    "./tstXptDump",
    "./tstnsIFileEnumerator",
    "./tstSimpleTypeLib",
    "./tstTestAtoms",
    "./tstXptLink",
    "./tstXPCOMCGlue",              /* user interaction required */
    "./tstTestCallTemplates",
#if 1 // later
    "testcase/tstIntNetR0",
    "./tstVMM",
    "./tstVMReq",
    "./tstVMREQ",
#endif
    /* final entry*/
    ""
};


/**
 * Checks if a testcase is include or should be skipped.
 *
 * @param pszTestcase   The testcase (filename).
 *
 * @return  true if the testcase is included.
 *          false if the testcase should be skipped.
 */
static bool IsTestcaseIncluded(const char *pszTestcase)
{
    /* exclude special modules based on extension. */
    const char *pszExt = RTPathExt(pszTestcase);
    if (   !RTStrICmp(pszExt, ".r0")
        || !RTStrICmp(pszExt, ".gc")
        || !RTStrICmp(pszExt, ".sys")
        || !RTStrICmp(pszExt, ".ko")
        || !RTStrICmp(pszExt, ".o")
        || !RTStrICmp(pszExt, ".obj")
        || !RTStrICmp(pszExt, ".lib")
        || !RTStrICmp(pszExt, ".a")
        || !RTStrICmp(pszExt, ".so")
        || !RTStrICmp(pszExt, ".dll")
        || !RTStrICmp(pszExt, ".dylib")
        || !RTStrICmp(pszExt, ".tmp")
        || !RTStrICmp(pszExt, ".log")
       )
        return false;

    /* check by name */
    char *pszDup = RTStrDup(pszTestcase);
    if (pszDup)
    {
        RTPathStripExt(pszDup);
        for (unsigned i = 0; i < RT_ELEMENTS(g_apszExclude); i++)
        {
            if (!strcmp(g_apszExclude[i], pszDup))
            {
                RTStrFree(pszDup);
                return false;
            }
        }
        RTStrFree(pszDup);
        return true;
    }

    RTPrintf("tstRunTestcases: Out of memory!\n");
    return false;
}


/**
 * Process the testcases found in the filter.
 *
 * @param   pszFilter   The filter (winnt) to pass to RTDirOpenFiltered for
 *                      selecting the testcases.
 * @param   pszDir      The directory we're processing.
 */
static void Process(const char *pszFilter, const char *pszDir)
{
    /*
     * Open and enumerate the directory.
     */
    PRTDIR pDir;
    int rc = RTDirOpenFiltered(&pDir, pszFilter, RTDIRFILTER_WINNT, 0);
    if (RT_SUCCESS(rc))
    {
        for (;;)
        {
            RTDIRENTRY DirEntry;
            rc = RTDirRead(pDir, &DirEntry, NULL);
            if (RT_FAILURE(rc))
            {
                if (rc == VERR_NO_MORE_FILES)
                    rc = VINF_SUCCESS;
                else
                    RTPrintf("tstRunTestcases: reading '%s' -> %Rrc\n", pszFilter, rc);
                break;
            }

            /*
             * Construct the testcase name.
             */
            char *pszTestcase;
            RTStrAPrintf(&pszTestcase, "%s/%s", pszDir, DirEntry.szName);
            if (!pszTestcase)
            {
                RTPrintf("tstRunTestcases: out of memory!\n");
                rc = VERR_NO_MEMORY;
                break;
            }
            if (IsTestcaseIncluded(pszTestcase))
            {
                /*
                 * Execute the testcase.
                 */
                RTPrintf("*** %s: Executing...\n", pszTestcase);  RTStrmFlush(g_pStdOut);
                const char *papszArgs[2];
                papszArgs[0] = pszTestcase;
                papszArgs[1] = NULL;
                RTPROCESS Process;
                rc = RTProcCreate(pszTestcase, papszArgs, RTENV_DEFAULT, 0, &Process);
                if (RT_SUCCESS(rc))
                {
                    /*
                     * Wait for the process and collect it's return code.
                     * If it takes too long, we'll terminate it and continue.
                     */
                    RTTIMESPEC Start;
                    RTTimeNow(&Start);
                    RTPROCSTATUS ProcStatus;
                    for (;;)
                    {
                        rc = RTProcWait(Process, RTPROCWAIT_FLAGS_NOBLOCK, &ProcStatus);
                        if (rc != VERR_PROCESS_RUNNING)
                            break;
                        RTTIMESPEC Now;
                        if (RTTimeSpecGetMilli(RTTimeSpecSub(RTTimeNow(&Now), &Start)) > 120*1000 /* 1 min */)
                        {
                            RTPrintf("*** %s: FAILED - timed out. killing it.\n", pszTestcase);
                            RTProcTerminate(Process);
                            RTThreadSleep(100);
                            RTProcWait(Process, RTPROCWAIT_FLAGS_NOBLOCK, &ProcStatus);
                            g_cFailures++;
                            break;
                        }
                        RTThreadSleep(100);
                    }

                    /*
                     * Examin the exit status.
                     */
                    if (RT_SUCCESS(rc))
                    {
                        if (    ProcStatus.enmReason == RTPROCEXITREASON_NORMAL
                            &&  ProcStatus.iStatus == 0)
                        {
                            RTPrintf("*** %s: PASSED\n", pszTestcase);
                            g_cPasses++;
                        }
                        else
                        {
                            RTPrintf("*** %s: FAILED\n", pszTestcase);
                            g_cFailures++;
                        }
                    }
                    else if (rc != VERR_PROCESS_RUNNING)
                    {
                        RTPrintf("tstRunTestcases: %s: RTProcWait failed -> %Rrc\n", pszTestcase, rc);
                        g_cFailures++;
                    }
                }
                else
                {
                    RTPrintf("tstRunTestcases: %s: failed to start -> %Rrc\n", pszTestcase, rc);
                    g_cFailures++;
                }

            }
            else
            {
                RTPrintf("tstRunTestcases: %s: SKIPPED\n", pszTestcase);
                g_cSkipped++;
            }
            RTStrFree(pszTestcase);
        } /* enumeration loop */

        RTDirClose(pDir);
    }
    else
        RTPrintf("tstRunTestcases: opening '%s' -> %Rrc\n", pszDir, rc);
}



int main(int argc, char **argv)
{
    RTR3InitExe(argc, &argv, 0);

    if (argc == 1)
    {
        char szPath[RTPATH_MAX];
        int rc = RTPathExecDir(szPath, sizeof(szPath) - sizeof("/.."));
        if (RT_FAILURE(rc))
        {
            RTPrintf("fatal error: RTPathExecDir -> %Rrc\n", rc);
            return 1;
        }
        rc = RTPathSetCurrent(strcat(szPath, "/.."));
        if (RT_FAILURE(rc))
        {
            RTPrintf("fatal error: RTPathSetCurrent -> %Rrc\n", rc);
            return 1;
        }

        Process("testcase/tst*", "testcase");
        Process("tst*", ".");
    }
    else
    {
        char szDir[RTPATH_MAX];
        for (int i = 1; i < argc; i++)
        {
            if (argv[i][0] == '-')
            {
                switch (argv[i][1])
                {
                    /* case '':... */

                    default:
                        RTPrintf("syntax error: Option '%s' is not recognized\n", argv[i]);
                        return 1;
                }
            }
            else
            {
                size_t cch = strlen(argv[i]);
                if (cch >= sizeof(szDir))
                {
                    RTPrintf("syntax error: '%s' is too long!\n", argv[i]);
                    return 1;
                }
                memcpy(szDir, argv[i], cch + 1);
                char *pszFilename = RTPathFilename(szDir);
                if (!pszFilename)
                {
                    RTPrintf("syntax error: '%s' does not include a file name or file name mask!\n", argv[i]);
                    return 1;
                }
                RTPathStripFilename(szDir);
                Process(argv[i], szDir);
            }
        }
    }

    RTPrintf("\n"
             "********************\n"
             "***  PASSED: %u\n"
             "***  FAILED: %u\n"
             "*** SKIPPED: %u\n"
             "***   TOTAL: %u\n",
             g_cPasses,
             g_cFailures,
             g_cSkipped,
             g_cPasses + g_cFailures + g_cSkipped);
    return !!g_cFailures;
}

