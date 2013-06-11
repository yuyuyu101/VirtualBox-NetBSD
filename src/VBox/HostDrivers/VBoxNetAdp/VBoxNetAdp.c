/* $Id: VBoxNetAdp.c $ */
/** @file
 * VBoxNetAdp - Virtual Network Adapter Driver (Host), Common Code.
 */

/*
 * Copyright (C) 2008-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

/** @page pg_netadp     VBoxNetAdp - Network Adapter
 *
 * This is a kernel module that creates a virtual interface that can be attached
 * to an internal network.
 *
 * In the big picture we're one of the three trunk interface on the internal
 * network, the one named "TAP Interface": @image html Networking_Overview.gif
 *
 */

/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#define LOG_GROUP LOG_GROUP_NET_ADP_DRV
#include "VBoxNetAdpInternal.h"

#include <VBox/log.h>
#include <VBox/err.h>
#include <iprt/string.h>

#ifdef VBOXANETADP_DO_NOT_USE_NETFLT
#error "this code is broken"

#include <VBox/sup.h>
#include <iprt/assert.h>
#include <iprt/spinlock.h>
#include <iprt/uuid.h>
#include <VBox/version.h>

/** r=bird: why is this here in the agnostic code? */
#ifdef RT_OS_DARWIN
# include <net/ethernet.h>
# include <net/if_ether.h>
# include <net/if_types.h>
# include <sys/socket.h>
# include <net/if.h>
# include <net/if_dl.h>
# include <sys/errno.h>
# include <sys/param.h>
#endif


/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
#define IFPORT_2_VBOXNETADP(pIfPort) \
    ( (PVBOXNETADP)((uint8_t *)pIfPort - RT_OFFSETOF(VBOXNETADP, MyPort)) )


AssertCompileMemberSize(VBOXNETADP, enmState, sizeof(uint32_t));

/**
 * Gets the enmState member atomically.
 *
 * Used for all reads.
 *
 * @returns The enmState value.
 * @param   pThis           The instance.
 */
DECLINLINE(VBOXNETADPSTATE) vboxNetAdpGetState(PVBOXNETADP pThis)
{
    return (VBOXNETADPSTATE)ASMAtomicUoReadU32((uint32_t volatile *)&pThis->enmState);
}


/**
 * Sets the enmState member atomically.
 *
 * Used for all updates.
 *
 * @param   pThis           The instance.
 * @param   enmNewState     The new value.
 */
DECLINLINE(void) vboxNetAdpSetState(PVBOXNETADP pThis, VBOXNETADPSTATE enmNewState)
{
    Log(("vboxNetAdpSetState: pThis=%p, state change: %d -> %d.\n", pThis, vboxNetAdpGetState(pThis), enmNewState));
    ASMAtomicWriteU32((uint32_t volatile *)&pThis->enmState, enmNewState);
}


/**
 * Sets the enmState member atomically after first acquiring the spinlock.
 *
 * Used for all updates.
 *
 * @param   pThis           The instance.
 * @param   enmNewState     The new value.
 */
DECLINLINE(void) vboxNetAdpSetStateWithLock(PVBOXNETADP pThis, VBOXNETADPSTATE enmNewState)
{
    Log(("vboxNetAdpSetStateWithLock: pThis=%p, state=%d.\n", pThis, enmNewState));
    RTSpinlockAcquire(pThis->hSpinlock);
    vboxNetAdpSetState(pThis, enmNewState);
    RTSpinlockReleaseNoInts(pThis->hSpinlock);
}


/**
 * Gets the enmState member with locking.
 *
 * Used for all reads.
 *
 * @returns The enmState value.
 * @param   pThis           The instance.
 */
DECLINLINE(VBOXNETADPSTATE) vboxNetAdpGetStateWithLock(PVBOXNETADP pThis)
{
    VBOXNETADPSTATE enmState;
    RTSpinlockAcquire(pThis->hSpinlock);
    enmState = vboxNetAdpGetState(pThis);
    RTSpinlockReleaseNoInts(pThis->hSpinlock);
    Log(("vboxNetAdpGetStateWithLock: pThis=%p, state=%d.\n", pThis, enmState));
    return enmState;
}


/**
 * Checks and sets the enmState member atomically.
 *
 * Used for all updates.
 *
 * @returns true if the state has been changed.
 * @param   pThis           The instance.
 * @param   enmNewState     The new value.
 */
DECLINLINE(bool) vboxNetAdpCheckAndSetState(PVBOXNETADP pThis, VBOXNETADPSTATE enmOldState, VBOXNETADPSTATE enmNewState)
{
    VBOXNETADPSTATE enmActualState;
    bool fRc = true; /* be optimistic */

    RTSpinlockAcquire(pThis->hSpinlock);
    enmActualState = vboxNetAdpGetState(pThis); /** @todo r=bird: ASMAtomicCmpXchgU32()*/
    if (enmActualState == enmOldState)
        vboxNetAdpSetState(pThis, enmNewState);
    else
        fRc = false;
    RTSpinlockReleaseNoInts(pThis->hSpinlock);

    if (fRc)
        Log(("vboxNetAdpCheckAndSetState: pThis=%p, state changed: %d -> %d.\n", pThis, enmOldState, enmNewState));
    else
        Log(("vboxNetAdpCheckAndSetState: pThis=%p, no state change: %d != %d (expected).\n", pThis, enmActualState, enmOldState));
    return fRc;
}


/**
 * Finds a instance by its name, the caller does the locking.
 *
 * @returns Pointer to the instance by the given name. NULL if not found.
 * @param   pGlobals        The globals.
 * @param   pszName         The name of the instance.
 */
static PVBOXNETADP vboxNetAdpFind(PVBOXNETADPGLOBALS pGlobals, const char *pszName)
{
    unsigned i;

    for (i = 0; i < RT_ELEMENTS(pGlobals->aAdapters); i++)
    {
        PVBOXNETADP pThis = &pGlobals->aAdapters[i];
        RTSpinlockAcquire(pThis->hSpinlock);
        if (    vboxNetAdpGetState(pThis)
            &&  !strcmp(pThis->szName, pszName))
        {
            RTSpinlockReleaseNoInts(pThis->hSpinlock);
            return pThis;
        }
        RTSpinlockReleaseNoInts(pThis->hSpinlock);
    }
    return NULL;
}


/**
 * Releases a reference to the specified instance.
 *
 * @param   pThis           The instance.
 * @param   fBusy           Whether the busy counter should be decremented too.
 */
DECLHIDDEN(void) vboxNetAdpRelease(PVBOXNETADP pThis)
{
    uint32_t cRefs;

    /*
     * Paranoid Android.
     */
    AssertPtr(pThis);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);
    Assert(pThis->MyPort.u32VersionEnd == INTNETTRUNKIFPORT_VERSION);
    Assert(vboxNetAdpGetState(pThis) > kVBoxNetAdpState_Invalid);
    AssertPtr(pThis->pGlobals);
    Assert(pThis->hEventIdle != NIL_RTSEMEVENT);
    Assert(pThis->hSpinlock != NIL_RTSPINLOCK);
    Assert(pThis->szName[0]);

    /*
     * The object reference counting.
     */
    cRefs = ASMAtomicDecU32(&pThis->cRefs);
    Assert(cRefs < UINT32_MAX / 2);
}


/**
 * Decrements the busy counter and does idle wakeup.
 *
 * @param   pThis           The instance.
 */
DECLHIDDEN(void) vboxNetAdpIdle(PVBOXNETADP pThis)
{
    uint32_t cBusy;

    /*
     * Paranoid Android.
     */
    AssertPtr(pThis);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);
    Assert(pThis->MyPort.u32VersionEnd == INTNETTRUNKIFPORT_VERSION);
    Assert(vboxNetAdpGetState(pThis) >= kVBoxNetAdpState_Connected);
    AssertPtr(pThis->pGlobals);
    Assert(pThis->hEventIdle != NIL_RTSEMEVENT);

    cBusy = ASMAtomicDecU32(&pThis->cBusy);
    if (!cBusy)
    {
        int rc = RTSemEventSignal(pThis->hEventIdle);
        AssertRC(rc);
    }
    else
        Assert(cBusy < UINT32_MAX / 2);
}


/**
 * Retains a reference to the specified instance.
 *
 * @param   pThis           The instance.
 */
DECLHIDDEN(void) vboxNetAdpRetain(PVBOXNETADP pThis)
{
    uint32_t cRefs;

    /*
     * Paranoid Android.
     */
    AssertPtr(pThis);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);
    Assert(pThis->MyPort.u32VersionEnd == INTNETTRUNKIFPORT_VERSION);
    Assert(vboxNetAdpGetState(pThis) > kVBoxNetAdpState_Invalid);
    AssertPtr(pThis->pGlobals);
    Assert(pThis->hEventIdle != NIL_RTSEMEVENT);
    Assert(pThis->hSpinlock != NIL_RTSPINLOCK);
    Assert(pThis->szName[0]);

    /*
     * Retain the object.
     */
    cRefs = ASMAtomicIncU32(&pThis->cRefs);
    Assert(cRefs > 1 && cRefs < UINT32_MAX / 2);

    NOREF(cRefs);
}


/**
 * Increments busy counter.
 *
 * @param   pThis           The instance.
 */
DECLHIDDEN(void) vboxNetAdpBusy(PVBOXNETADP pThis)
{
    uint32_t cBusy;

    /*
     * Are we vigilant enough?
     */
    AssertPtr(pThis);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);
    Assert(pThis->MyPort.u32VersionEnd == INTNETTRUNKIFPORT_VERSION);
    Assert(vboxNetAdpGetState(pThis) >= kVBoxNetAdpState_Connected);
    AssertPtr(pThis->pGlobals);
    Assert(pThis->hEventIdle != NIL_RTSEMEVENT);
    cBusy = ASMAtomicIncU32(&pThis->cBusy);
    Assert(cBusy > 0 && cBusy < UINT32_MAX / 2);

    NOREF(cBusy);
}


/**
 * Generate a suitable MAC address.
 *
 * @param   pThis       The instance.
 * @param   pMac        Where to return the MAC address.
 */
DECLHIDDEN(void) vboxNetAdpComposeMACAddress(PVBOXNETADP pThis, PRTMAC pMac)
{
#if 0 /* Use a locally administered version of the OUI we use for the guest NICs. */
    pMac->au8[0] = 0x08 | 2;
    pMac->au8[1] = 0x00;
    pMac->au8[2] = 0x27;
#else /* this is what \0vb comes down to. It seems to be unassigned atm. */
    pMac->au8[0] = 0;
    pMac->au8[1] = 0x76;
    pMac->au8[2] = 0x62;
#endif

    pMac->au8[3] = 0; /* pThis->uUnit >> 16; */
    pMac->au8[4] = 0; /* pThis->uUnit >> 8; */
    pMac->au8[5] = pThis->uUnit;
}


/**
 * Checks if receive is possible and increases busy and ref counters if so.
 *
 * @param   pThis           The instance.
 */
DECLHIDDEN(bool) vboxNetAdpPrepareToReceive(PVBOXNETADP pThis)
{
    bool fCanReceive  = false;
    /*
     * Input validation.
     */
    AssertPtr(pThis);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);
    RTSpinlockAcquire(pThis->hSpinlock);
    if (vboxNetAdpGetState(pThis) == kVBoxNetAdpState_Active)
    {
        fCanReceive = true;
        vboxNetAdpRetain(pThis);
        vboxNetAdpBusy(pThis);
    }
    RTSpinlockReleaseNoInts(pThis->hSpinlock);
    Log(("vboxNetAdpPrepareToReceive: fCanReceive=%d.\n", fCanReceive));

    return fCanReceive;
}


/**
 * Forwards scatter/gather list to internal network and decreases busy and ref counters.
 *
 * @param   pThis           The instance.
 */
DECLHIDDEN(void) vboxNetAdpReceive(PVBOXNETADP pThis, PINTNETSG pSG)
{
    /*
     * Input validation.
     */
    AssertPtr(pThis);
    AssertPtr(pSG);
    AssertPtr(pThis->pSwitchPort);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);
    Log(("vboxNetAdpReceive: forwarding packet to internal net...\n"));
    pThis->pSwitchPort->pfnRecv(pThis->pSwitchPort, pSG, INTNETTRUNKDIR_HOST);
    vboxNetAdpIdle(pThis);
    vboxNetAdpRelease(pThis);
}


/**
 * Decreases busy and ref counters.
 *
 * @param   pThis           The instance.
 */
DECLHIDDEN(void) vboxNetAdpCancelReceive(PVBOXNETADP pThis)
{
    Log(("vboxNetAdpCancelReceive: cancelled.\n"));
    vboxNetAdpIdle(pThis);
    vboxNetAdpRelease(pThis);
}


/**
 * @copydoc INTNETTRUNKIFPORT::pfnXmit
 */
static DECLCALLBACK(int) vboxNetAdpPortXmit(PINTNETTRUNKIFPORT pIfPort, PINTNETSG pSG, uint32_t fDst)
{
    PVBOXNETADP pThis = IFPORT_2_VBOXNETADP(pIfPort);
    int rc = VINF_SUCCESS;

    /*
     * Input validation.
     */
    AssertPtr(pThis);
    AssertPtr(pSG);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);

    Log(("vboxNetAdpPortXmit: outgoing packet (len=%d)\n", pSG->cbTotal));

    /*
     * Do a retain/busy, invoke the OS specific code.
     */
    RTSpinlockAcquire(pThis->hSpinlock);
    if (vboxNetAdpGetState(pThis) != kVBoxNetAdpState_Active)
    {
        RTSpinlockReleaseNoInts(pThis->hSpinlock);
        Log(("vboxNetAdpReceive: Dropping incoming packet for inactive interface %s.\n",
             pThis->szName));
        return VERR_INVALID_STATE;
    }
    vboxNetAdpRetain(pThis);
    vboxNetAdpBusy(pThis);
    RTSpinlockReleaseNoInts(pThis->hSpinlock);

    rc = vboxNetAdpPortOsXmit(pThis, pSG, fDst);
    vboxNetAdpIdle(pThis);
    vboxNetAdpRelease(pThis);

    return rc;
}


/**
 * @copydoc INTNETTRUNKIFPORT::pfnGetMacAddress
 */
static DECLCALLBACK(void) vboxNetAdpPortGetMacAddress(PINTNETTRUNKIFPORT pIfPort, PRTMAC pMac)
{
    PVBOXNETADP pThis = IFPORT_2_VBOXNETADP(pIfPort);

    /*
     * Input validation.
     */
    AssertPtr(pThis);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);
    Assert(vboxNetAdpGetStateWithLock(pThis) == kVBoxNetAdpState_Active);

    /*
     * Forward the question to the OS specific code.
     */
    vboxNetAdpPortOsGetMacAddress(pThis, pMac);
}


/**
 * @copydoc INTNETTRUNKIFPORT::pfnWaitForIdle
 */
static DECLCALLBACK(int) vboxNetAdpPortWaitForIdle(PINTNETTRUNKIFPORT pIfPort, uint32_t cMillies)
{
    int rc;
    PVBOXNETADP pThis = IFPORT_2_VBOXNETADP(pIfPort);

    /*
     * Input validation.
     */
    AssertPtr(pThis);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);
    AssertReturn(vboxNetAdpGetStateWithLock(pThis) >= kVBoxNetAdpState_Connected, VERR_INVALID_STATE);

    /*
     * Go to sleep on the semaphore after checking the busy count.
     */
    vboxNetAdpRetain(pThis);

    rc = VINF_SUCCESS;
    while (pThis->cBusy && RT_SUCCESS(rc))
        rc = RTSemEventWait(pThis->hEventIdle, cMillies); /** @todo make interruptible? */

    vboxNetAdpRelease(pThis);

    return rc;
}


/**
 * @copydoc INTNETTRUNKIFPORT::pfnSetActive
 */
static DECLCALLBACK(bool) vboxNetAdpPortSetActive(PINTNETTRUNKIFPORT pIfPort, bool fActive)
{
    bool fPreviouslyActive;
    PVBOXNETADP pThis = IFPORT_2_VBOXNETADP(pIfPort);

    /*
     * Input validation.
     */
    AssertPtr(pThis);
    AssertPtr(pThis->pGlobals);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);

    Log(("vboxNetAdpPortSetActive: pThis=%p, fActive=%d, state before: %d.\n", pThis, fActive, vboxNetAdpGetState(pThis)));
    RTSpinlockAcquire(pThis->hSpinlock);

    fPreviouslyActive = vboxNetAdpGetState(pThis) == kVBoxNetAdpState_Active;
    if (fPreviouslyActive != fActive)
    {
        switch (vboxNetAdpGetState(pThis))
        {
            case kVBoxNetAdpState_Connected:
                vboxNetAdpSetState(pThis, kVBoxNetAdpState_Active);
                break;
            case kVBoxNetAdpState_Active:
                vboxNetAdpSetState(pThis, kVBoxNetAdpState_Connected);
                break;
            default:
                break;
        }
    }

    RTSpinlockReleaseNoInts(pThis->hSpinlock);
    Log(("vboxNetAdpPortSetActive: state after: %RTbool.\n", vboxNetAdpGetState(pThis)));
    return fPreviouslyActive;
}


/**
 * @copydoc INTNETTRUNKIFPORT::pfnDisconnectAndRelease
 */
static DECLCALLBACK(void) vboxNetAdpPortDisconnectAndRelease(PINTNETTRUNKIFPORT pIfPort)
{
    PVBOXNETADP pThis = IFPORT_2_VBOXNETADP(pIfPort);

    /*
     * Serious paranoia.
     */
    AssertPtr(pThis);
    Assert(pThis->MyPort.u32Version == INTNETTRUNKIFPORT_VERSION);
    Assert(pThis->MyPort.u32VersionEnd == INTNETTRUNKIFPORT_VERSION);
    AssertPtr(pThis->pGlobals);
    Assert(pThis->hEventIdle != NIL_RTSEMEVENT);
    Assert(pThis->hSpinlock != NIL_RTSPINLOCK);


    /*
     * Disconnect and release it.
     */
    RTSpinlockAcquire(pThis->hSpinlock);
    //Assert(vboxNetAdpGetState(pThis) == kVBoxNetAdpState_Connected);
    Assert(!pThis->cBusy);
    vboxNetAdpSetState(pThis, kVBoxNetAdpState_Transitional);
    RTSpinlockReleaseNoInts(pThis->hSpinlock);

    vboxNetAdpOsDisconnectIt(pThis);
    pThis->pSwitchPort = NULL;

    RTSpinlockAcquire(pThis->hSpinlock);
    vboxNetAdpSetState(pThis, kVBoxNetAdpState_Available);
    RTSpinlockReleaseNoInts(pThis->hSpinlock);

    vboxNetAdpRelease(pThis);
}


/**
 * @copydoc INTNETTRUNKIFPORT::pfnRelease
 */
static DECLCALLBACK(void) vboxNetAdpPortRelease(PINTNETTRUNKIFPORT pIfPort)
{
    PVBOXNETADP pThis = IFPORT_2_VBOXNETADP(pIfPort);
    vboxNetAdpRelease(pThis);
}


/**
 * @copydoc INTNETTRUNKIFPORT::pfnRetain
 */
static DECLCALLBACK(void) vboxNetAdpPortRetain(PINTNETTRUNKIFPORT pIfPort)
{
    PVBOXNETADP pThis = IFPORT_2_VBOXNETADP(pIfPort);
    vboxNetAdpRetain(pThis);
}


int vboxNetAdpCreate(PINTNETTRUNKFACTORY pIfFactory, PVBOXNETADP *ppNew)
{
    PVBOXNETADPGLOBALS pGlobals = (PVBOXNETADPGLOBALS)((uint8_t *)pIfFactory - RT_OFFSETOF(VBOXNETADPGLOBALS, TrunkFactory));
    unsigned i;
    int rc;

    for (i = 0; i < RT_ELEMENTS(pGlobals->aAdapters); i++)
    {
        PVBOXNETADP pThis = &pGlobals->aAdapters[i];

        if (vboxNetAdpCheckAndSetState(pThis, kVBoxNetAdpState_Invalid, kVBoxNetAdpState_Transitional))
        {
            /* Found an empty slot -- use it. */
            uint32_t cRefs = ASMAtomicIncU32(&pThis->cRefs);
            Assert(cRefs == 1);
            RTMAC Mac;
            vboxNetAdpComposeMACAddress(pThis, &Mac);
            rc = vboxNetAdpOsCreate(pThis, &Mac);
            *ppNew = pThis;

            RTSpinlockAcquire(pThis->hSpinlock);
            vboxNetAdpSetState(pThis, kVBoxNetAdpState_Available);
            RTSpinlockReleaseNoInts(pThis->hSpinlock);
            return rc;
        }
    }

    /* All slots in adapter array are busy. */
    return VERR_OUT_OF_RESOURCES;
}

int vboxNetAdpDestroy(PVBOXNETADP pThis)
{
    int rc = VINF_SUCCESS;

    RTSpinlockAcquire(pThis->hSpinlock);
    if (vboxNetAdpGetState(pThis) != kVBoxNetAdpState_Available || pThis->cBusy)
    {
        RTSpinlockReleaseNoInts(pThis->hSpinlock);
        return VERR_INTNET_FLT_IF_BUSY;
    }
    vboxNetAdpSetState(pThis, kVBoxNetAdpState_Transitional);
    RTSpinlockReleaseNoInts(pThis->hSpinlock);
    vboxNetAdpRelease(pThis);

    vboxNetAdpOsDestroy(pThis);

    RTSpinlockAcquire(pThis->hSpinlock);
    vboxNetAdpSetState(pThis, kVBoxNetAdpState_Invalid);
    RTSpinlockReleaseNoInts(pThis->hSpinlock);

    return rc;
}

/**
 * Connects the instance to the specified switch port.
 *
 * Called while owning the lock. We're ASSUMING that the internal
 * networking code is already owning an recursive mutex, so, there
 * will be no deadlocks when vboxNetAdpOsConnectIt calls back into
 * it for setting preferences.
 *
 * @returns VBox status code.
 * @param   pThis               The instance.
 * @param   pSwitchPort         The port on the internal network 'switch'.
 * @param   ppIfPort            Where to return our port interface.
 */
static int vboxNetAdpConnectIt(PVBOXNETADP pThis, PINTNETTRUNKSWPORT pSwitchPort, PINTNETTRUNKIFPORT *ppIfPort)
{
    int rc;

    /*
     * Validate state.
     */
    Assert(!pThis->cBusy);
    Assert(vboxNetAdpGetStateWithLock(pThis) == kVBoxNetAdpState_Transitional);

    /*
     * Do the job.
     * Note that we're calling the os stuff while owning the semaphore here.
     */
    pThis->pSwitchPort = pSwitchPort;
    rc = vboxNetAdpOsConnectIt(pThis);
    if (RT_SUCCESS(rc))
    {
        *ppIfPort = &pThis->MyPort;
    }
    else
        pThis->pSwitchPort = NULL;

    return rc;
}


/**
 * @copydoc INTNETTRUNKFACTORY::pfnCreateAndConnect
 */
static DECLCALLBACK(int) vboxNetAdpFactoryCreateAndConnect(PINTNETTRUNKFACTORY pIfFactory, const char *pszName,
                                                           PINTNETTRUNKSWPORT pSwitchPort, uint32_t fFlags,
                                                           PINTNETTRUNKIFPORT *ppIfPort)
{
    PVBOXNETADPGLOBALS pGlobals = (PVBOXNETADPGLOBALS)((uint8_t *)pIfFactory - RT_OFFSETOF(VBOXNETADPGLOBALS, TrunkFactory));
    PVBOXNETADP pThis;
    int rc;

    LogFlow(("vboxNetAdpFactoryCreateAndConnect: pszName=%p:{%s} fFlags=%#x\n", pszName, pszName, fFlags));
    Assert(pGlobals->cFactoryRefs > 0);
    AssertMsgReturn(!fFlags,
                    ("%#x\n", fFlags), VERR_INVALID_PARAMETER);

    /*
     * Find instance, check if busy, connect if not.
     */
    pThis = vboxNetAdpFind(pGlobals, pszName);
    if (pThis)
    {
        if (vboxNetAdpCheckAndSetState(pThis, kVBoxNetAdpState_Available, kVBoxNetAdpState_Transitional))
        {
            vboxNetAdpRetain(pThis);
            rc = vboxNetAdpConnectIt(pThis, pSwitchPort, ppIfPort);
            vboxNetAdpSetStateWithLock(pThis, RT_SUCCESS(rc) ? kVBoxNetAdpState_Connected : kVBoxNetAdpState_Available);
        }
        else
            rc = VERR_INTNET_FLT_IF_BUSY;
    }
    else
        rc = VERR_INTNET_FLT_IF_NOT_FOUND;

    return rc;
}


/**
 * @copydoc INTNETTRUNKFACTORY::pfnRelease
 */
static DECLCALLBACK(void) vboxNetAdpFactoryRelease(PINTNETTRUNKFACTORY pIfFactory)
{
    PVBOXNETADPGLOBALS pGlobals = (PVBOXNETADPGLOBALS)((uint8_t *)pIfFactory - RT_OFFSETOF(VBOXNETADPGLOBALS, TrunkFactory));

    int32_t cRefs = ASMAtomicDecS32(&pGlobals->cFactoryRefs);
    Assert(cRefs >= 0); NOREF(cRefs);
    LogFlow(("vboxNetAdpFactoryRelease: cRefs=%d (new)\n", cRefs));
}


/**
 * Implements the SUPDRV component factor interface query method.
 *
 * @returns Pointer to an interface. NULL if not supported.
 *
 * @param   pSupDrvFactory      Pointer to the component factory registration structure.
 * @param   pSession            The session - unused.
 * @param   pszInterfaceUuid    The factory interface id.
 */
static DECLCALLBACK(void *) vboxNetAdpQueryFactoryInterface(PCSUPDRVFACTORY pSupDrvFactory, PSUPDRVSESSION pSession, const char *pszInterfaceUuid)
{
    PVBOXNETADPGLOBALS pGlobals = (PVBOXNETADPGLOBALS)((uint8_t *)pSupDrvFactory - RT_OFFSETOF(VBOXNETADPGLOBALS, SupDrvFactory));

    /*
     * Convert the UUID strings and compare them.
     */
    RTUUID UuidReq;
    int rc = RTUuidFromStr(&UuidReq, pszInterfaceUuid);
    if (RT_SUCCESS(rc))
    {
        if (!RTUuidCompareStr(&UuidReq, INTNETTRUNKFACTORY_UUID_STR))
        {
            ASMAtomicIncS32(&pGlobals->cFactoryRefs);
            return &pGlobals->TrunkFactory;
        }
#ifdef LOG_ENABLED
        else
            Log(("VBoxNetAdp: unknown factory interface query (%s)\n", pszInterfaceUuid));
#endif
    }
    else
        Log(("VBoxNetAdp: rc=%Rrc, uuid=%s\n", rc, pszInterfaceUuid));

    return NULL;
}


/**
 * Checks whether the VBoxNetAdp wossname can be unloaded.
 *
 * This will return false if someone is currently using the module.
 *
 * @returns true if it's relatively safe to unload it, otherwise false.
 * @param   pGlobals        Pointer to the globals.
 */
DECLHIDDEN(bool) vboxNetAdpCanUnload(PVBOXNETADPGLOBALS pGlobals)
{
    bool fRc = true; /* Assume it can be unloaded. */
    unsigned i;

    for (i = 0; i < RT_ELEMENTS(pGlobals->aAdapters); i++)
    {
        PVBOXNETADP pThis = &pGlobals->aAdapters[i];
        if (vboxNetAdpGetStateWithLock(&pGlobals->aAdapters[i]) >= kVBoxNetAdpState_Connected)
        {
            fRc = false;
            break; /* We already know the answer. */
        }
    }
    return fRc && ASMAtomicUoReadS32((int32_t volatile *)&pGlobals->cFactoryRefs) <= 0;
}

/**
 * tries to deinitialize Idc
 * we separate the globals settings "base" which is actually
 * "general" globals settings except for Idc, and idc.
 * This is needed for windows filter driver, which gets loaded prior to VBoxDrv,
 * thus it's not possible to make idc initialization from the driver startup routine for it,
 * though the "base is still needed for the driver to functions".
 * @param pGlobals
 * @return VINF_SUCCESS on success, VERR_WRONG_ORDER if we're busy.
 */
DECLHIDDEN(int) vboxNetAdpTryDeleteIdc(PVBOXNETADPGLOBALS pGlobals)
{
    int rc;

    Assert(pGlobals->hFastMtx != NIL_RTSEMFASTMUTEX);

    /*
     * Check before trying to deregister the factory.
     */
    if (!vboxNetAdpCanUnload(pGlobals))
        return VERR_WRONG_ORDER;

    /*
     * Disconnect from SUPDRV and check that nobody raced us,
     * reconnect if that should happen.
     */
    rc = SUPR0IdcComponentDeregisterFactory(&pGlobals->SupDrvIDC, &pGlobals->SupDrvFactory);
    AssertRC(rc);
    if (!vboxNetAdpCanUnload(pGlobals))
    {
        rc = SUPR0IdcComponentRegisterFactory(&pGlobals->SupDrvIDC, &pGlobals->SupDrvFactory);
        AssertRC(rc);
        return VERR_WRONG_ORDER;
    }

    SUPR0IdcClose(&pGlobals->SupDrvIDC);

    return rc;
}

static int vboxNetAdpSlotCreate(PVBOXNETADPGLOBALS pGlobals, unsigned uUnit, PVBOXNETADP pNew)
{
    int rc;

    pNew->MyPort.u32Version             = INTNETTRUNKIFPORT_VERSION;
    pNew->MyPort.pfnRetain              = vboxNetAdpPortRetain;
    pNew->MyPort.pfnRelease             = vboxNetAdpPortRelease;
    pNew->MyPort.pfnDisconnectAndRelease= vboxNetAdpPortDisconnectAndRelease;
    pNew->MyPort.pfnSetState            = vboxNetAdpPortSetState;
    pNew->MyPort.pfnWaitForIdle         = vboxNetAdpPortWaitForIdle;
    pNew->MyPort.pfnXmit                = vboxNetAdpPortXmit;
    pNew->MyPort.u32VersionEnd          = INTNETTRUNKIFPORT_VERSION;
    pNew->pSwitchPort                   = NULL;
    pNew->pGlobals                      = pGlobals;
    pNew->hSpinlock                     = NIL_RTSPINLOCK;
    pNew->enmState                      = kVBoxNetAdpState_Invalid;
    pNew->cRefs                         = 0;
    pNew->cBusy                         = 0;
    pNew->hEventIdle                    = NIL_RTSEMEVENT;

    rc = RTSpinlockCreate(&pNew->hSpinlock, RTSPINLOCK_FLAGS_INTERRUPT_SAFE, "VBoxNetAdptSlotCreate");
    if (RT_SUCCESS(rc))
    {
        rc = RTSemEventCreate(&pNew->hEventIdle);
        if (RT_SUCCESS(rc))
        {
            rc = vboxNetAdpOsInit(pNew);
            if (RT_SUCCESS(rc))
            {
                return rc;
            }
            RTSemEventDestroy(pNew->hEventIdle);
            pNew->hEventIdle = NIL_RTSEMEVENT;
        }
        RTSpinlockDestroy(pNew->hSpinlock);
        pNew->hSpinlock = NIL_RTSPINLOCK;
    }
    return rc;
}

static void vboxNetAdpSlotDestroy(PVBOXNETADP pThis)
{
    Assert(pThis->cRefs == 0);
    Assert(pThis->cBusy == 0);
    Assert(vboxNetAdpGetState(pThis) == kVBoxNetAdpState_Invalid);
    if (pThis->hEventIdle != NIL_RTSEMEVENT)
    {
        RTSemEventDestroy(pThis->hEventIdle);
        pThis->hEventIdle = NIL_RTSEMEVENT;
    }
    if (pThis->hSpinlock != NIL_RTSPINLOCK)
    {
        RTSpinlockDestroy(pThis->hSpinlock);
        pThis->hSpinlock = NIL_RTSPINLOCK;
    }
}

/**
 * performs "base" globals deinitialization
 * we separate the globals settings "base" which is actually
 * "general" globals settings except for Idc, and idc.
 * This is needed for windows filter driver, which gets loaded prior to VBoxDrv,
 * thus it's not possible to make idc initialization from the driver startup routine for it,
 * though the "base is still needed for the driver to functions".
 * @param pGlobals
 * @return none
 */
DECLHIDDEN(void) vboxNetAdpDeleteGlobalsBase(PVBOXNETADPGLOBALS pGlobals)
{
    int i;
    /*
     * Release resources.
     */
    for (i = 0; i < (int)RT_ELEMENTS(pGlobals->aAdapters); i++)
        if (RT_SUCCESS(vboxNetAdpDestroy(&pGlobals->aAdapters[i])))
            vboxNetAdpSlotDestroy(&pGlobals->aAdapters[i]);

    RTSemFastMutexDestroy(pGlobals->hFastMtx);
    pGlobals->hFastMtx = NIL_RTSEMFASTMUTEX;

#ifdef VBOXNETADP_STATIC_CONFIG
    RTSemEventDestroy(pGlobals->hTimerEvent);
    pGlobals->hTimerEvent = NIL_RTSEMEVENT;
#endif

}


/**
 * Called by the native part when the OS wants the driver to unload.
 *
 * @returns VINF_SUCCESS on success, VERR_WRONG_ORDER if we're busy.
 *
 * @param   pGlobals        Pointer to the globals.
 */
DECLHIDDEN(int) vboxNetAdpTryDeleteGlobals(PVBOXNETADPGLOBALS pGlobals)
{
    int rc = vboxNetAdpTryDeleteIdc(pGlobals);
    if (RT_SUCCESS(rc))
    {
        vboxNetAdpDeleteGlobalsBase(pGlobals);
    }
    return rc;
}


/**
 * performs the "base" globals initialization
 * we separate the globals initialization to globals "base" initialization which is actually
 * "general" globals initialization except for Idc not being initialized, and idc initialization.
 * This is needed for windows filter driver, which gets loaded prior to VBoxDrv,
 * thus it's not possible to make idc initialization from the driver startup routine for it.
 *
 * @returns VBox status code.
 * @param   pGlobals    Pointer to the globals. */
DECLHIDDEN(int) vboxNetAdpInitGlobalsBase(PVBOXNETADPGLOBALS pGlobals)
{
    /*
     * Initialize the common portions of the structure.
     */
    int i;
    int rc = RTSemFastMutexCreate(&pGlobals->hFastMtx);
    if (RT_SUCCESS(rc))
    {
        memset(pGlobals->aAdapters, 0, sizeof(pGlobals->aAdapters));
        for (i = 0; i < (int)RT_ELEMENTS(pGlobals->aAdapters); i++)
        {
            rc = vboxNetAdpSlotCreate(pGlobals, i, &pGlobals->aAdapters[i]);
            if (RT_FAILURE(rc))
            {
                /* Clean up. */
                while (--i >= 0)
                    vboxNetAdpSlotDestroy(&pGlobals->aAdapters[i]);
                Log(("vboxNetAdpInitGlobalsBase: Failed to create fast mutex (rc=%Rrc).\n", rc));
                RTSemFastMutexDestroy(pGlobals->hFastMtx);
                return rc;
            }
        }
        pGlobals->TrunkFactory.pfnRelease = vboxNetAdpFactoryRelease;
        pGlobals->TrunkFactory.pfnCreateAndConnect = vboxNetAdpFactoryCreateAndConnect;

        strcpy(pGlobals->SupDrvFactory.szName, "VBoxNetAdp");
        pGlobals->SupDrvFactory.pfnQueryFactoryInterface = vboxNetAdpQueryFactoryInterface;
    }

    return rc;
}

/**
 * performs the Idc initialization
 * we separate the globals initialization to globals "base" initialization which is actually
 * "general" globals initialization except for Idc not being initialized, and idc initialization.
 * This is needed for windows filter driver, which gets loaded prior to VBoxDrv,
 * thus it's not possible to make idc initialization from the driver startup routine for it.
 *
 * @returns VBox status code.
 * @param   pGlobals    Pointer to the globals. */
DECLHIDDEN(int) vboxNetAdpInitIdc(PVBOXNETADPGLOBALS pGlobals)
{
    int rc;
    /*
     * Establish a connection to SUPDRV and register our component factory.
     */
    rc = SUPR0IdcOpen(&pGlobals->SupDrvIDC, 0 /* iReqVersion = default */, 0 /* iMinVersion = default */, NULL, NULL, NULL);
    if (RT_SUCCESS(rc))
    {
        rc = SUPR0IdcComponentRegisterFactory(&pGlobals->SupDrvIDC, &pGlobals->SupDrvFactory);
        if (RT_SUCCESS(rc))
        {
#if 1 /** @todo REMOVE ME! */
            PVBOXNETADP pTmp;
            rc = vboxNetAdpCreate(&pGlobals->TrunkFactory, &pTmp);
            if (RT_FAILURE(rc))
                Log(("Failed to create vboxnet0, rc=%Rrc.\n", rc));
#endif
            Log(("VBoxNetAdp: pSession=%p\n", SUPR0IdcGetSession(&pGlobals->SupDrvIDC)));
            return rc;
        }

        /* bail out. */
        LogRel(("VBoxNetAdp: Failed to register component factory, rc=%Rrc\n", rc));
        SUPR0IdcClose(&pGlobals->SupDrvIDC);
    }

    return rc;
}

/**
 * Called by the native driver/kext module initialization routine.
 *
 * It will initialize the common parts of the globals, assuming the caller
 * has already taken care of the OS specific bits.
 *
 * @returns VBox status code.
 * @param   pGlobals    Pointer to the globals.
 */
DECLHIDDEN(int) vboxNetAdpInitGlobals(PVBOXNETADPGLOBALS pGlobals)
{
    /*
     * Initialize the common portions of the structure.
     */
    int rc = vboxNetAdpInitGlobalsBase(pGlobals);
    if (RT_SUCCESS(rc))
    {
        rc = vboxNetAdpInitIdc(pGlobals);
        if (RT_SUCCESS(rc))
        {
            return rc;
        }

        /* bail out. */
        vboxNetAdpDeleteGlobalsBase(pGlobals);
    }

    return rc;
}

#else /* !VBOXANETADP_DO_NOT_USE_NETFLT */


VBOXNETADP g_aAdapters[VBOXNETADP_MAX_INSTANCES];
static uint8_t g_aUnits[VBOXNETADP_MAX_UNITS/8];


DECLINLINE(int) vboxNetAdpGetUnitByName(const char *pcszName)
{
    uint32_t iUnit = RTStrToUInt32(pcszName + sizeof(VBOXNETADP_NAME) - 1);
    bool fOld;

    if (iUnit >= VBOXNETADP_MAX_UNITS)
        return -1;

    fOld = ASMAtomicBitTestAndSet(g_aUnits, iUnit);
    return fOld ? -1 : (int)iUnit;
}

DECLINLINE(int) vboxNetAdpGetNextAvailableUnit(void)
{
    bool fOld;
    int iUnit;
    /* There is absolutely no chance that all units are taken */
    do {
        iUnit = ASMBitFirstClear(g_aUnits, VBOXNETADP_MAX_UNITS);
        if (iUnit < 0)
            break;
        fOld = ASMAtomicBitTestAndSet(g_aUnits, iUnit);
    } while (fOld);

    return iUnit;
}

DECLINLINE(void) vboxNetAdpReleaseUnit(int iUnit)
{
    bool fSet = ASMAtomicBitTestAndClear(g_aUnits, iUnit);
    NOREF(fSet);
    Assert(fSet);
}

/**
 * Generate a suitable MAC address.
 *
 * @param   pThis       The instance.
 * @param   pMac        Where to return the MAC address.
 */
DECLHIDDEN(void) vboxNetAdpComposeMACAddress(PVBOXNETADP pThis, PRTMAC pMac)
{
    /* Use a locally administered version of the OUI we use for the guest NICs. */
    pMac->au8[0] = 0x08 | 2;
    pMac->au8[1] = 0x00;
    pMac->au8[2] = 0x27;

    pMac->au8[3] = 0; /* pThis->iUnit >> 16; */
    pMac->au8[4] = 0; /* pThis->iUnit >> 8; */
    pMac->au8[5] = pThis->iUnit;
}

int vboxNetAdpCreate(PVBOXNETADP *ppNew, const char *pcszName)
{
    int rc;
    unsigned i;
    for (i = 0; i < RT_ELEMENTS(g_aAdapters); i++)
    {
        PVBOXNETADP pThis = &g_aAdapters[i];

        if (ASMAtomicCmpXchgU32((uint32_t volatile *)&pThis->enmState, kVBoxNetAdpState_Transitional, kVBoxNetAdpState_Invalid))
        {
            RTMAC Mac;
            /* Found an empty slot -- use it. */
            Log(("vboxNetAdpCreate: found empty slot: %d\n", i));
            if (pcszName)
            {
                Log(("vboxNetAdpCreate: using name: %s\n", pcszName));
                pThis->iUnit = vboxNetAdpGetUnitByName(pcszName);
                strncpy(pThis->szName, pcszName, sizeof(pThis->szName));
                pThis->szName[sizeof(pThis->szName) - 1] = '\0';
            }
            else
            {
                pThis->iUnit = vboxNetAdpGetNextAvailableUnit();
                pThis->szName[0] = '\0';
            }
            if (pThis->iUnit < 0)
                rc = VERR_INVALID_PARAMETER;
            else
            {
                vboxNetAdpComposeMACAddress(pThis, &Mac);
                rc = vboxNetAdpOsCreate(pThis, &Mac);
                Log(("vboxNetAdpCreate: pThis=%p pThis->iUnit=%d, pThis->szName=%s\n",
                     pThis, pThis->iUnit, pThis->szName));
            }
            if (RT_SUCCESS(rc))
            {
                *ppNew = pThis;
                ASMAtomicWriteU32((uint32_t volatile *)&pThis->enmState, kVBoxNetAdpState_Active);
                Log2(("VBoxNetAdpCreate: Created %s\n", g_aAdapters[i].szName));
            }
            else
            {
                ASMAtomicWriteU32((uint32_t volatile *)&pThis->enmState, kVBoxNetAdpState_Invalid);
                Log(("vboxNetAdpCreate: vboxNetAdpOsCreate failed with '%Rrc'.\n", rc));
            }
            for (i = 0; i < RT_ELEMENTS(g_aAdapters); i++)
                Log2(("VBoxNetAdpCreate: Scanning entry: state=%d unit=%d name=%s\n",
                      g_aAdapters[i].enmState, g_aAdapters[i].iUnit, g_aAdapters[i].szName));
            return rc;
        }
    }
    Log(("vboxNetAdpCreate: no empty slots!\n"));

    /* All slots in adapter array are busy. */
    return VERR_OUT_OF_RESOURCES;
}

int vboxNetAdpDestroy(PVBOXNETADP pThis)
{
    int rc = VINF_SUCCESS;

    if (!ASMAtomicCmpXchgU32((uint32_t volatile *)&pThis->enmState, kVBoxNetAdpState_Transitional, kVBoxNetAdpState_Active))
        return VERR_INTNET_FLT_IF_BUSY;

    Assert(pThis->iUnit >= 0 && pThis->iUnit < VBOXNETADP_MAX_UNITS);
    vboxNetAdpOsDestroy(pThis);
    vboxNetAdpReleaseUnit(pThis->iUnit);
    pThis->iUnit = -1;
    pThis->szName[0] = '\0';

    ASMAtomicWriteU32((uint32_t volatile *)&pThis->enmState, kVBoxNetAdpState_Invalid);

    return rc;
}

int  vboxNetAdpInit(void)
{
    unsigned i;
    /*
     * Init common members and call OS-specific init.
     */
    memset(g_aUnits, 0, sizeof(g_aUnits));
    memset(g_aAdapters, 0, sizeof(g_aAdapters));
    LogFlow(("vboxnetadp: max host-only interfaces supported: %d (%d bytes)\n",
             VBOXNETADP_MAX_INSTANCES, sizeof(g_aAdapters)));
    for (i = 0; i < RT_ELEMENTS(g_aAdapters); i++)
    {
        g_aAdapters[i].enmState = kVBoxNetAdpState_Invalid;
        g_aAdapters[i].iUnit    = -1;
        vboxNetAdpOsInit(&g_aAdapters[i]);
    }

    return VINF_SUCCESS;
}

/**
 * Finds an adapter by its name.
 *
 * @returns Pointer to the instance by the given name. NULL if not found.
 * @param   pGlobals        The globals.
 * @param   pszName         The name of the instance.
 */
PVBOXNETADP vboxNetAdpFindByName(const char *pszName)
{
    unsigned i;

    for (i = 0; i < RT_ELEMENTS(g_aAdapters); i++)
    {
        PVBOXNETADP pThis = &g_aAdapters[i];
        Log2(("VBoxNetAdp: Scanning entry: state=%d name=%s\n", pThis->enmState, pThis->szName));
        if (   strcmp(pThis->szName, pszName) == 0
            && ASMAtomicReadU32((uint32_t volatile *)&pThis->enmState) == kVBoxNetAdpState_Active)
            return pThis;
    }
    return NULL;
}

void vboxNetAdpShutdown(void)
{
    unsigned i;

    /* Remove virtual adapters */
    for (i = 0; i < RT_ELEMENTS(g_aAdapters); i++)
        vboxNetAdpDestroy(&g_aAdapters[i]);
}
#endif /* !VBOXANETADP_DO_NOT_USE_NETFLT */
