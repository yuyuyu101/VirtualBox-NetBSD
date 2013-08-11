/* $Id: sleepqueue-r0drv-netbsd.h $ */
/** @file
 * IPRT - NetBSD Ring-0 Driver Helpers for Abstracting Sleep Queues,
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


#ifndef ___r0drv_netbsd_sleepqueue_r0drv_netbsd_h
#define ___r0drv_netbsd_sleepqueue_r0drv_netbsd_h

#include "the-netbsd-kernel.h"

#include <iprt/asm-math.h>
#include <iprt/err.h>
#include <iprt/time.h>

static syncobj_t vbox_syncobj = {
        SOBJ_SLEEPQ_FIFO,
        sleepq_unsleep,
        sleepq_changepri,
        sleepq_lendpri,
        syncobj_noowner,
};

/**
 * Kernel mode Linux wait state structure.
 */
typedef struct RTR0SEMBSDSLEEP
{
    /** The absolute timeout given as nano seconds since the start of the
     *  monotonic clock. */
    uint64_t        uNsAbsTimeout;
    /** The timeout in ticks. Updated after waiting. */
    int             iTimeout;
    /** Set if it's an indefinite wait. */
    bool            fIndefinite;
    /** Set if we've already timed out.
     * Set by rtR0SemBsdWaitDoIt and read by rtR0SemBsdWaitHasTimedOut. */
    bool            fTimedOut;
    /** Flag whether the wait was interrupted. */
    bool            fInterrupted;
    /** flag whether the wait is interruptible or not. */
    bool            fInterruptible;
    /** Opaque wait channel id. */
    kmutex_t        *sc_lock;
    sleepq_t        sleepq;
} RTR0SEMBSDSLEEP;
/** Pointer to a NetBSD wait state. */
typedef RTR0SEMBSDSLEEP *PRTR0SEMBSDSLEEP;


/**
 * Updates the timeout of the NetBSD wait.
 *
 * @returns RTSEMWAIT_FLAGS_INDEFINITE if the timeout value is too big.
 *          0 otherwise
 * @param   pWait               The wait structure.
 * @param   uTimeout            The relative timeout in nanoseconds.
 */
DECLINLINE(uint32_t) rtR0SemBsdWaitUpdateTimeout(PRTR0SEMBSDSLEEP pWait, uint64_t uTimeout)
{
#if 0
    struct timeval tv;

    tv.tv_sec = uTimeout / UINT64_C(1000000000);
    tv.tv_usec = (uTimeout % UINT64_C(1000000000)) / UINT64_C(1000);

    pWait->iTimeout = tvtohz(&tv);
#else
    uint64_t cTicks = ASMMultU64ByU32DivByU32(uTimeout, hz, UINT32_C(1000000000));
    if (cTicks >= INT_MAX)
        return RTSEMWAIT_FLAGS_INDEFINITE;
    else
        pWait->iTimeout     = (int)cTicks;
#endif

    return 0;
}

/**
 * Initializes a wait.
 *
 * The caller MUST check the wait condition BEFORE calling this function or the
 * timeout logic will be flawed.
 *
 * @returns VINF_SUCCESS or VERR_TIMEOUT.
 * @param   pWait               The wait structure.
 * @param   fFlags              The wait flags.
 * @param   uTimeout            The timeout.
 * @param   pvWaitChan          The opaque wait channel.
 */
DECLINLINE(int) rtR0SemBsdWaitInit(PRTR0SEMBSDSLEEP pWait, uint32_t fFlags, uint64_t uTimeout,
                                   void *pvWaitChan)
{
    pWait->iTimeout = 0;
    pWait->uNsAbsTimeout = 0; /* shut up gcc */

    /*
     * Process the flags and timeout.
     */
    if (!(fFlags & RTSEMWAIT_FLAGS_INDEFINITE))
    {
/** @todo optimize: millisecs -> nanosecs -> millisec -> jiffies */
        if (fFlags & RTSEMWAIT_FLAGS_MILLISECS)
            uTimeout = uTimeout < UINT64_MAX / UINT32_C(1000000) * UINT32_C(1000000)
                     ? uTimeout * UINT32_C(1000000)
                     : UINT64_MAX;
        if (uTimeout == UINT64_MAX)
            fFlags |= RTSEMWAIT_FLAGS_INDEFINITE;
        else
        {
            uint64_t u64Now;
            if (fFlags & RTSEMWAIT_FLAGS_RELATIVE)
            {
                if (uTimeout == 0)
                    return VERR_TIMEOUT;

                u64Now = RTTimeSystemNanoTS();
                if (u64Now + uTimeout < u64Now) /* overflow */
                    fFlags |= RTSEMWAIT_FLAGS_INDEFINITE;
                else
                    pWait->uNsAbsTimeout = u64Now + uTimeout;
            }
            else
            {
                u64Now = RTTimeSystemNanoTS();
                if (u64Now >= uTimeout)
                    return VERR_TIMEOUT;

                pWait->uNsAbsTimeout = uTimeout;
                uTimeout -= u64Now; /* Get a relative value. */
            }
        }
    }

    if (!(fFlags & RTSEMWAIT_FLAGS_INDEFINITE))
    {
        pWait->fIndefinite      = false;
        fFlags |= rtR0SemBsdWaitUpdateTimeout(pWait, uTimeout);
    }

    if (fFlags & RTSEMWAIT_FLAGS_INDEFINITE)
    {
        pWait->fIndefinite      = true;
        pWait->iTimeout         = INT_MAX;
        pWait->uNsAbsTimeout    = UINT64_MAX;
    }

    pWait->fTimedOut   = false;

    /*
     * Initialize the wait queue related bits.
     */
    pWait->fInterruptible = fFlags & RTSEMWAIT_FLAGS_INTERRUPTIBLE
                            ? true : false;
    pWait->fInterrupted   = false;
    pWait->sc_lock = mutex_obj_alloc(MUTEX_DEFAULT, IPL_SCHED);
    sleepq_init(&pWait->sleepq);

    return VINF_SUCCESS;
}

/**
 * Prepares the next wait.
 *
 * This must be called before rtR0SemBsdWaitDoIt, and the caller should check
 * the exit conditions inbetween the two calls.
 *
 * @param   pWait               The wait structure.
 */
DECLINLINE(void) rtR0SemBsdWaitPrepare(PRTR0SEMBSDSLEEP pWait)
{
    /* Lock the queues. */
    sleepq_enter(&pWait->sleepq, curlwp, pWait->sc_lock);
}

/**
 * Do the actual wait.
 *
 * @param   pWait               The wait structure.
 */
DECLINLINE(void) rtR0SemBsdWaitDoIt(PRTR0SEMBSDSLEEP pWait)
{

    sleepq_enqueue(&pWait->sleepq, pWait, "VBoxIS", &vbox_syncobj);

    sleepq_block(pWait->iTimeout, true);
}


/**
 * Checks if a NetBSD wait was interrupted.
 *
 * @returns true / false
 * @param   pWait               The wait structure.
 * @remarks This shall be called before the first rtR0SemLnxWaitDoIt().
 */
DECLINLINE(bool) rtR0SemBsdWaitWasInterrupted(PRTR0SEMBSDSLEEP pWait)
{
    return pWait->fInterrupted;
}


/**
 * Checks if a NetBSD wait has timed out.
 *
 * @returns true / false
 * @param   pWait               The wait structure.
 */
DECLINLINE(bool) rtR0SemBsdWaitHasTimedOut(PRTR0SEMBSDSLEEP pWait)
{
    return pWait->fTimedOut;
}


/**
 * Deletes a NetBSD wait.
 *
 * @param   pWait               The wait structure.
 */
DECLINLINE(void) rtR0SemBsdWaitDelete(PRTR0SEMBSDSLEEP pWait)
{
}


/**
 * Signals the wait channel.
 *
 * @param  pvWaitChan           The opaque wait channel handle.
 */
DECLINLINE(void) rtR0SemBsdSignal(void *pvWaitChan)
{
}

/**
 * Wakes up all waiters on the wait channel.
 *
 * @param  pvWaitChan           The opaque wait channel handle.
 */
DECLINLINE(void) rtR0SemBsdBroadcast(void *pvWaitChan)
{
}

/**
 * Gets the max resolution of the timeout machinery.
 *
 * @returns Resolution specified in nanoseconds.
 */
DECLINLINE(uint32_t) rtR0SemBsdWaitGetResolution(void)
{
    return 1000000000 / hz; /* ns */
}

#endif

