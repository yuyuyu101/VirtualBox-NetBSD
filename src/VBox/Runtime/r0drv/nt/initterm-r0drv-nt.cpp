/* $Id: initterm-r0drv-nt.cpp $ */
/** @file
 * IPRT - Initialization & Termination, R0 Driver, NT.
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
#include "the-nt-kernel.h"
#include <iprt/asm-amd64-x86.h>
#include <iprt/assert.h>
#include <iprt/err.h>
#include <iprt/mp.h>
#include <iprt/string.h>
#include "internal/initterm.h"
#include "internal-r0drv-nt.h"


/*******************************************************************************
*   Global Variables                                                           *
*******************************************************************************/
/** The Nt CPU set.
 * KeQueryActiveProcssors() cannot be called at all IRQLs and therefore we'll
 * have to cache it. Fortunately, Nt doesn't really support taking CPUs offline
 * or online. It's first with W2K8 that support for CPU hotplugging was added.
 * Once we start caring about this, we'll simply let the native MP event callback
 * and update this variable as CPUs comes online. (The code is done already.)
 */
RTCPUSET                    g_rtMpNtCpuSet;

/** ExSetTimerResolution, introduced in W2K. */
PFNMYEXSETTIMERRESOLUTION   g_pfnrtNtExSetTimerResolution;
/** KeFlushQueuedDpcs, introduced in XP. */
PFNMYKEFLUSHQUEUEDDPCS      g_pfnrtNtKeFlushQueuedDpcs;
/** HalRequestIpi, introduced in ??. */
PFNHALREQUESTIPI            g_pfnrtNtHalRequestIpi;
/** HalSendSoftwareInterrupt */
PFNHALSENDSOFTWAREINTERRUPT g_pfnrtNtHalSendSoftwareInterrupt;
/** SendIpi handler based on Windows version */
PFNRTSENDIPI                g_pfnrtSendIpi;
/** KeIpiGenericCall - Windows Server 2003+ only */
PFNRTKEIPIGENERICCALL       g_pfnrtKeIpiGenericCall;

/** Offset of the _KPRCB::QuantumEnd field. 0 if not found. */
uint32_t                    g_offrtNtPbQuantumEnd;
/** Size of the _KPRCB::QuantumEnd field. 0 if not found. */
uint32_t                    g_cbrtNtPbQuantumEnd;
/** Offset of the _KPRCB::DpcQueueDepth field. 0 if not found. */
uint32_t                    g_offrtNtPbDpcQueueDepth;



DECLHIDDEN(int) rtR0InitNative(void)
{
    /*
     * Init the Nt cpu set.
     */
#ifdef IPRT_TARGET_NT4
    KAFFINITY ActiveProcessors = (UINT64_C(1) << KeNumberProcessors) - UINT64_C(1);
#else
    KAFFINITY ActiveProcessors = KeQueryActiveProcessors();
#endif
    RTCpuSetEmpty(&g_rtMpNtCpuSet);
    RTCpuSetFromU64(&g_rtMpNtCpuSet, ActiveProcessors);
/** @todo Port to W2K8 with > 64 cpus/threads. */

#ifdef IPRT_TARGET_NT4
    g_pfnrtNtExSetTimerResolution = NULL;
    g_pfnrtNtKeFlushQueuedDpcs = NULL;
    g_pfnrtNtHalRequestIpi = NULL;
    g_pfnrtNtHalSendSoftwareInterrupt = NULL;
    g_pfnrtKeIpiGenericCall = NULL;
#else
    /*
     * Initialize the function pointers.
     */
    UNICODE_STRING RoutineName;
    RtlInitUnicodeString(&RoutineName, L"ExSetTimerResolution");
    g_pfnrtNtExSetTimerResolution = (PFNMYEXSETTIMERRESOLUTION)MmGetSystemRoutineAddress(&RoutineName);

    RtlInitUnicodeString(&RoutineName, L"KeFlushQueuedDpcs");
    g_pfnrtNtKeFlushQueuedDpcs = (PFNMYKEFLUSHQUEUEDDPCS)MmGetSystemRoutineAddress(&RoutineName);

    RtlInitUnicodeString(&RoutineName, L"HalRequestIpi");
    g_pfnrtNtHalRequestIpi = (PFNHALREQUESTIPI)MmGetSystemRoutineAddress(&RoutineName);

    RtlInitUnicodeString(&RoutineName, L"HalSendSoftwareInterrupt");
    g_pfnrtNtHalSendSoftwareInterrupt = (PFNHALSENDSOFTWAREINTERRUPT)MmGetSystemRoutineAddress(&RoutineName);

    RtlInitUnicodeString(&RoutineName, L"KeIpiGenericCall");
    g_pfnrtKeIpiGenericCall = (PFNRTKEIPIGENERICCALL)MmGetSystemRoutineAddress(&RoutineName);
#endif

    /*
     * Get some info that might come in handy below.
     */
    ULONG MajorVersion = 0;
    ULONG MinorVersion = 0;
    ULONG BuildNumber  = 0;
    BOOLEAN fChecked = PsGetVersion(&MajorVersion, &MinorVersion, &BuildNumber, NULL);

    g_pfnrtSendIpi = rtMpSendIpiDummy;
#ifndef IPRT_TARGET_NT4
    if (    g_pfnrtNtHalRequestIpi
        &&  MajorVersion == 6
        &&  MinorVersion == 0)
    {
        /* Vista or Windows Server 2008 */
        g_pfnrtSendIpi = rtMpSendIpiVista;
    }
    else
    if (    g_pfnrtNtHalSendSoftwareInterrupt
        &&  MajorVersion == 6
        &&  MinorVersion == 1)
    {
        /* Windows 7 or Windows Server 2008 R2 */
        g_pfnrtSendIpi = rtMpSendIpiWin7;
    }
    /* Windows XP should send always send an IPI -> VERIFY */
#endif
    KIRQL OldIrql;
    KeRaiseIrql(DISPATCH_LEVEL, &OldIrql); /* make sure we stay on the same cpu */

    union
    {
        uint32_t auRegs[4];
        char szVendor[4*3+1];
    } u;
    ASMCpuId(0, &u.auRegs[3], &u.auRegs[0], &u.auRegs[2], &u.auRegs[1]);
    u.szVendor[4*3] = '\0';

    /*
     * HACK ALERT (and d�j� vu warning)!
     *
     * Try find _KPRCB::QuantumEnd and _KPRCB::[DpcData.]DpcQueueDepth.
     * For purpose of verification we use the VendorString member (12+1 chars).
     *
     * The offsets was initially derived by poking around with windbg
     * (dt _KPRCB, !prcb ++, and such like). Systematic harvesting is now done
     * by means of dia2dump, grep and the symbol packs. Typically:
     *      dia2dump -type _KDPC_DATA -type _KPRCB EXE\ntkrnlmp.pdb | grep -wE "QuantumEnd|DpcData|DpcQueueDepth|VendorString"
     */
    /** @todo array w/ data + script for extracting a row. (save space + readability; table will be short.) */
    __try
    {
#if defined(RT_ARCH_X86)
        PKPCR    pPcr   = (PKPCR)__readfsdword(RT_OFFSETOF(KPCR,SelfPcr));
        uint8_t *pbPrcb = (uint8_t *)pPcr->Prcb;

        if (    BuildNumber == 2600                             /* XP SP2 */
            &&  !memcmp(&pbPrcb[0x900], &u.szVendor[0], 4*3))
        {
            g_offrtNtPbQuantumEnd    = 0x88c;
            g_cbrtNtPbQuantumEnd     = 4;
            g_offrtNtPbDpcQueueDepth = 0x870;
        }
        /* WindowsVista.6002.090410-1830.x86fre.Symbols.exe
           WindowsVista.6002.090410-1830.x86chk.Symbols.exe
           WindowsVista.6002.090130-1715.x86fre.Symbols.exe
           WindowsVista.6002.090130-1715.x86chk.Symbols.exe */
        else if (   BuildNumber == 6002
                 && !memcmp(&pbPrcb[0x1c2c], &u.szVendor[0], 4*3))
        {
            g_offrtNtPbQuantumEnd    = 0x1a41;
            g_cbrtNtPbQuantumEnd     = 1;
            g_offrtNtPbDpcQueueDepth = 0x19e0 + 0xc;
        }
        else if (   BuildNumber == 3790                         /* Server 2003 SP2 */
                 && !memcmp(&pbPrcb[0xb60], &u.szVendor[0], 4*3))
        {
            g_offrtNtPbQuantumEnd    = 0x981;
            g_cbrtNtPbQuantumEnd     = 1;
            g_offrtNtPbDpcQueueDepth = 0x920 + 0xc;
        }

        /** @todo more */
        //pbQuantumEnd = (uint8_t volatile *)pPcr->Prcb + 0x1a41;

#elif defined(RT_ARCH_AMD64)
        PKPCR    pPcr   = (PKPCR)__readgsqword(RT_OFFSETOF(KPCR,Self));
        uint8_t *pbPrcb = (uint8_t *)pPcr->CurrentPrcb;

        if (    BuildNumber == 3790                             /* XP64 / W2K3-AMD64 SP1 */
            &&  !memcmp(&pbPrcb[0x22b4], &u.szVendor[0], 4*3))
        {
            g_offrtNtPbQuantumEnd    = 0x1f75;
            g_cbrtNtPbQuantumEnd     = 1;
            g_offrtNtPbDpcQueueDepth = 0x1f00 + 0x18;
        }
        else if (   BuildNumber == 6000                         /* Vista/AMD64 */
                 && !memcmp(&pbPrcb[0x38bc], &u.szVendor[0], 4*3))
        {
            g_offrtNtPbQuantumEnd    = 0x3375;
            g_cbrtNtPbQuantumEnd     = 1;
            g_offrtNtPbDpcQueueDepth = 0x3300 + 0x18;
        }
        /* WindowsVista.6002.090410-1830.amd64fre.Symbols
           WindowsVista.6002.090130-1715.amd64fre.Symbols
           WindowsVista.6002.090410-1830.amd64chk.Symbols */
        else if (   BuildNumber == 6002
                 && !memcmp(&pbPrcb[0x399c], &u.szVendor[0], 4*3))
        {
            g_offrtNtPbQuantumEnd    = 0x3475;
            g_cbrtNtPbQuantumEnd     = 1;
            g_offrtNtPbDpcQueueDepth = 0x3400 + 0x18;
        }
        /* Windows7.7600.16539.amd64fre.win7_gdr.100226-1909 */
        else if (    BuildNumber == 7600
                 && !memcmp(&pbPrcb[0x4bb8], &u.szVendor[0], 4*3))
        {
            g_offrtNtPbQuantumEnd    = 0x21d9;
            g_cbrtNtPbQuantumEnd     = 1;
            g_offrtNtPbDpcQueueDepth = 0x2180 + 0x18;
        }

#else
# error "port me"
#endif
    }
    __except(EXCEPTION_EXECUTE_HANDLER) /** @todo this handler doesn't seem to work... Because of Irql? */
    {
        g_offrtNtPbQuantumEnd    = 0;
        g_cbrtNtPbQuantumEnd     = 0;
        g_offrtNtPbDpcQueueDepth = 0;
    }

    KeLowerIrql(OldIrql);

#ifndef IN_GUEST /** @todo fix above for all Nt versions. */
    if (!g_offrtNtPbQuantumEnd && !g_offrtNtPbDpcQueueDepth)
        DbgPrint("IPRT: Neither _KPRCB::QuantumEnd nor _KPRCB::DpcQueueDepth was not found! Kernel %u.%u %u %s\n",
                 MajorVersion, MinorVersion, BuildNumber, fChecked ? "checked" : "free");
# ifdef DEBUG
    else
        DbgPrint("IPRT: _KPRCB:{.QuantumEnd=%x/%d, .DpcQueueDepth=%x/%d} Kernel %ul.%ul %ul %s\n",
                 g_offrtNtPbQuantumEnd, g_cbrtNtPbQuantumEnd, g_offrtNtPbDpcQueueDepth,
                 MajorVersion, MinorVersion, BuildNumber, fChecked ? "checked" : "free");
# endif
#endif

    return VINF_SUCCESS;
}


DECLHIDDEN(void) rtR0TermNative(void)
{
}

