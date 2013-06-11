/* $Id: HostUSBDeviceImpl.cpp $ */
/** @file
 * VirtualBox IHostUSBDevice COM interface implementation.
 */

/*
 * Copyright (C) 2005-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#include <iprt/types.h> /* for UINT64_C */

#include "HostUSBDeviceImpl.h"
#include "MachineImpl.h"
#include "HostImpl.h"
#include "VirtualBoxErrorInfoImpl.h"
#include "USBProxyService.h"

#include "AutoCaller.h"
#include "Logging.h"

#include <VBox/err.h>
#include <iprt/cpp/utils.h>

// constructor / destructor
/////////////////////////////////////////////////////////////////////////////

DEFINE_EMPTY_CTOR_DTOR(HostUSBDevice)

HRESULT HostUSBDevice::FinalConstruct()
{
    mUSBProxyService = NULL;
    mUsb = NULL;

    return BaseFinalConstruct();
}

void HostUSBDevice::FinalRelease()
{
    uninit();
    BaseFinalRelease();
}

// public initializer/uninitializer for internal purposes only
/////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the USB device object.
 *
 * @returns COM result indicator
 * @param   aUsb                Pointer to the usb device structure for which the object is to be a wrapper.
 *                              This structure is now fully owned by the HostUSBDevice object and will be
 *                              freed when it is destructed.
 * @param   aUSBProxyService    Pointer to the USB Proxy Service object.
 */
HRESULT HostUSBDevice::init(PUSBDEVICE aUsb, USBProxyService *aUSBProxyService)
{
    ComAssertRet(aUsb, E_INVALIDARG);

    /* Enclose the state transition NotReady->InInit->Ready */
    AutoInitSpan autoInitSpan(this);
    AssertReturn(autoInitSpan.isOk(), E_FAIL);

    /*
     * We need a unique ID for this VBoxSVC session.
     * The UUID isn't stored anywhere.
     */
    unconst(mId).create();

    /*
     * Set the initial device state.
     */
    AssertMsgReturn(   aUsb->enmState >= USBDEVICESTATE_UNSUPPORTED
                    && aUsb->enmState <  USBDEVICESTATE_USED_BY_GUEST, /* used-by-guest is not a legal initial state. */
                    ("%d\n", aUsb->enmState), E_FAIL);
    mUniState = (HostUSBDeviceState)aUsb->enmState;
    mUniSubState = kHostUSBDeviceSubState_Default;
    mPendingUniState = kHostUSBDeviceState_Invalid;
    mPrevUniState = mUniState;
    mIsPhysicallyDetached = false;

    /* Other data members */
    mUSBProxyService = aUSBProxyService;
    mUsb = aUsb;

    /* Set the name. */
    mNameObj = getName();
    mName = mNameObj.c_str();

    /* Confirm the successful initialization */
    autoInitSpan.setSucceeded();

    return S_OK;
}

/**
 *  Uninitializes the instance and sets the ready flag to FALSE.
 *  Called either from FinalRelease() or by the parent when it gets destroyed.
 */
void HostUSBDevice::uninit()
{
    /* Enclose the state transition Ready->InUninit->NotReady */
    AutoUninitSpan autoUninitSpan(this);
    if (autoUninitSpan.uninitDone())
        return;

    if (mUsb != NULL)
    {
        USBProxyService::freeDevice(mUsb);
        mUsb = NULL;
    }

    mUSBProxyService = NULL;
    mUniState = kHostUSBDeviceState_Invalid;
}

// IUSBDevice properties
/////////////////////////////////////////////////////////////////////////////

STDMETHODIMP HostUSBDevice::COMGETTER(Id)(BSTR *aId)
{
    CheckComArgOutPointerValid(aId);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    /* mId is constant during life time, no need to lock */
    mId.toUtf16().cloneTo(aId);

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(VendorId)(USHORT *aVendorId)
{
    CheckComArgOutPointerValid(aVendorId);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    *aVendorId = mUsb->idVendor;

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(ProductId)(USHORT *aProductId)
{
    CheckComArgOutPointerValid(aProductId);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    *aProductId = mUsb->idProduct;

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(Revision)(USHORT *aRevision)
{
    CheckComArgOutPointerValid(aRevision);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    *aRevision = mUsb->bcdDevice;

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(Manufacturer)(BSTR *aManufacturer)
{
    CheckComArgOutPointerValid(aManufacturer);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    Bstr(mUsb->pszManufacturer).cloneTo(aManufacturer);

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(Product)(BSTR *aProduct)
{
    CheckComArgOutPointerValid(aProduct);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    Bstr(mUsb->pszProduct).cloneTo(aProduct);

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(SerialNumber)(BSTR *aSerialNumber)
{
    CheckComArgOutPointerValid(aSerialNumber);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    Bstr(mUsb->pszSerialNumber).cloneTo(aSerialNumber);

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(Address)(BSTR *aAddress)
{
    CheckComArgOutPointerValid(aAddress);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    Bstr(mUsb->pszAddress).cloneTo(aAddress);

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(Port)(USHORT *aPort)
{
    CheckComArgOutPointerValid(aPort);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

#if !defined(RT_OS_WINDOWS) /// @todo check up the bPort value on Windows before enabling this.
    *aPort = mUsb->bPort;
#else
    *aPort = 0;
#endif

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(Version)(USHORT *aVersion)
{
    CheckComArgOutPointerValid(aVersion);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    *aVersion = mUsb->bcdUSB >> 8;

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(PortVersion)(USHORT *aPortVersion)
{
    CheckComArgOutPointerValid(aPortVersion);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    /* Port version is 2 (EHCI) if and only if the device runs at high speed;
     * if speed is unknown, fall back to the old and inaccurate method.
     */
    if (mUsb->enmSpeed == USBDEVICESPEED_UNKNOWN)
        *aPortVersion = mUsb->bcdUSB >> 8;
    else
        *aPortVersion = (mUsb->enmSpeed == USBDEVICESPEED_HIGH) ? 2 : 1;

    return S_OK;
}

STDMETHODIMP HostUSBDevice::COMGETTER(Remote)(BOOL *aRemote)
{
    CheckComArgOutPointerValid(aRemote);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    *aRemote = FALSE;

    return S_OK;
}

// IHostUSBDevice properties
/////////////////////////////////////////////////////////////////////////////

STDMETHODIMP HostUSBDevice::COMGETTER(State)(USBDeviceState_T *aState)
{
    CheckComArgOutPointerValid(aState);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    *aState = canonicalState();

    return S_OK;
}


// public methods only for internal purposes
////////////////////////////////////////////////////////////////////////////////

/**
 * @note Locks this object for reading.
 */
Utf8Str HostUSBDevice::getName()
{
    Utf8Str name;

    AutoCaller autoCaller(this);
    AssertComRCReturn(autoCaller.rc(), name);

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    bool haveManufacturer = mUsb->pszManufacturer && *mUsb->pszManufacturer;
    bool haveProduct = mUsb->pszProduct && *mUsb->pszProduct;
    if (haveManufacturer && haveProduct)
        name = Utf8StrFmt("%s %s", mUsb->pszManufacturer, mUsb->pszProduct);
    else if (haveManufacturer)
        name = Utf8StrFmt("%s", mUsb->pszManufacturer);
    else if (haveProduct)
        name = Utf8StrFmt("%s", mUsb->pszProduct);
    else
        name = "<unknown>";

    return name;
}

/**
 * Requests the USB proxy service capture the device (from the host)
 * and attach it to a VM.
 *
 * As a convenience, this method will operate like attachToVM() if the device
 * is already held by the proxy. Note that it will then perform IPC to the VM
 * process, which means it will temporarily release all locks. (Is this a good idea?)
 *
 * @param   aMachine    Machine this device should be attach to.
 * @param   aSetError   Whether to set full error message or not to bother.
 * @param   aMaskedIfs  The interfaces to hide from the guest.
 *
 * @returns Status indicating whether it was successfully captured and/or attached.
 * @retval  S_OK on success.
 * @retval  E_UNEXPECTED if the device state doesn't permit for any attaching.
 * @retval  E_* as appropriate.
 */
HRESULT HostUSBDevice::requestCaptureForVM(SessionMachine *aMachine, bool aSetError, ULONG aMaskedIfs /* = 0*/)
{
    /*
     * Validate preconditions and input.
     */
    AssertReturn(aMachine, E_INVALIDARG);
    AssertReturn(!isWriteLockOnCurrentThread(), E_FAIL);
    AssertReturn(!aMachine->isWriteLockOnCurrentThread(), E_FAIL);

    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);
    LogFlowThisFunc(("{%s} aMachine=%p aMaskedIfs=%#x\n", mName, aMachine, aMaskedIfs));

    if (aSetError)
    {
        if (mUniState == kHostUSBDeviceState_Unsupported)
            return setError(E_INVALIDARG,
                            tr("USB device '%s' with UUID {%RTuuid} cannot be accessed by guest computers"),
                            mName, mId.raw());
        if (mUniState == kHostUSBDeviceState_UsedByHost)
            return setError(E_INVALIDARG,
                            tr("USB device '%s' with UUID {%RTuuid} is being exclusively used by the host computer"),
                            mName, mId.raw());
        if (mUniState == kHostUSBDeviceState_UsedByVM)
        {
            /* Machine::name() requires a read lock */
            alock.release();
            AutoReadLock machLock(mMachine COMMA_LOCKVAL_SRC_POS);
            return setError(E_INVALIDARG,
                            tr("USB device '%s' with UUID {%RTuuid} is already captured by the virtual machine '%s'"),
                            mName, mId.raw(), mMachine->getName().c_str());
        }
        if (mUniState >= kHostUSBDeviceState_FirstTransitional)
            return setError(E_INVALIDARG,
                            tr("USB device '%s' with UUID {%RTuuid} is busy with a previous request. Please try again later"),
                             mName, mId.raw());
        if (    mUniState != kHostUSBDeviceState_Unused
            &&  mUniState != kHostUSBDeviceState_HeldByProxy
            &&  mUniState != kHostUSBDeviceState_Capturable)
            return setError(E_INVALIDARG,
                            tr("USB device '%s' with UUID {%RTuuid} is not in the right state for capturing (%s)"),
                            mName, mId.raw(), getStateName());
    }

    AssertReturn(   mUniState == kHostUSBDeviceState_HeldByProxy
                 || mUniState == kHostUSBDeviceState_Unused
                 || mUniState == kHostUSBDeviceState_Capturable,
                 E_UNEXPECTED);
    Assert(mMachine.isNull());

    /*
     * If it's already held by the proxy, we'll simply call
     * attachToVM synchronously.
     */
    if (mUniState == kHostUSBDeviceState_HeldByProxy)
    {
        alock.release();
        HRESULT hrc = attachToVM(aMachine, aMaskedIfs);
        return SUCCEEDED(hrc);
    }

    /*
     * Need to capture the device before it can be used.
     *
     * The device will be attached to the VM by the USB proxy service thread
     * when the request succeeds (i.e. asynchronously).
     */
    LogFlowThisFunc(("{%s} capturing the device.\n", mName));
#if (defined(RT_OS_DARWIN) && defined(VBOX_WITH_NEW_USB_CODE_ON_DARWIN)) /* PORTME */ \
 || defined(RT_OS_WINDOWS) || defined(RT_OS_SOLARIS)
    setState(kHostUSBDeviceState_Capturing, kHostUSBDeviceState_UsedByVM, kHostUSBDeviceSubState_AwaitingDetach);
#else
    setState(kHostUSBDeviceState_Capturing, kHostUSBDeviceState_UsedByVM);
#endif
    mMachine = aMachine;
    mMaskedIfs = aMaskedIfs;
    alock.release();
    int rc = mUSBProxyService->captureDevice(this);
    if (RT_FAILURE(rc))
    {
        alock.acquire();
        failTransition();
        mMachine.setNull();
        if (rc == VERR_SHARING_VIOLATION)
            return setError(E_FAIL,
                            tr("USB device '%s' with UUID {%RTuuid} is in use by someone else"),
                            mName, mId.raw());
        return E_FAIL;
    }

    return S_OK;
}

/**
 * Attempts to attach the USB device to a VM.
 *
 * The device must be in the HeldByProxy state or about to exit the
 * Capturing state.
 *
 * This method will make an IPC to the VM process and do the actual
 * attaching. While in the IPC locks will be abandond.
 *
 * @returns Status indicating whether it was successfully attached or not.
 * @retval  S_OK on success.
 * @retval  E_UNEXPECTED if the device state doesn't permit for any attaching.
 * @retval  E_* as appropriate.
 *
 * @param   aMachine        Machine this device should be attach to.
 * @param   aMaskedIfs      The interfaces to hide from the guest.
 */
HRESULT HostUSBDevice::attachToVM(SessionMachine *aMachine, ULONG aMaskedIfs /* = 0*/)
{
    AssertReturn(!isWriteLockOnCurrentThread(), E_FAIL);
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);
    /*
     * Validate and update the state.
     */
    AssertReturn(   mUniState == kHostUSBDeviceState_Capturing
                 || mUniState == kHostUSBDeviceState_HeldByProxy
                 || mUniState == kHostUSBDeviceState_AttachingToVM,
                 E_UNEXPECTED);
    setState(kHostUSBDeviceState_AttachingToVM, kHostUSBDeviceState_UsedByVM);

    /*
     * The VM process will query the object, so grab a reference to ourselves and release the locks.
     */
    ComPtr<IUSBDevice> d = this;

    /*
     * Call the VM process (IPC) and request it to attach the device.
     *
     * There are many reasons for this to fail, so, as a consequence we don't
     * assert the return code as it will crash the daemon and annoy the heck
     * out of people.
     */
    LogFlowThisFunc(("{%s} Calling machine->onUSBDeviceAttach()...\n", mName));
    alock.release();
    HRESULT hrc = aMachine->onUSBDeviceAttach(d, NULL, aMaskedIfs);
    LogFlowThisFunc(("{%s} Done machine->onUSBDeviceAttach()=%08X\n", mName, hrc));

    /*
     * As we re-acquire the lock, we'll have to check if the device was
     * physically detached while we were busy.
     */
    alock.acquire();

    if (SUCCEEDED(hrc))
    {
        mMachine = aMachine;
        if (!mIsPhysicallyDetached)
            setState(kHostUSBDeviceState_UsedByVM);
        else
        {
            alock.release();
            detachFromVM(kHostUSBDeviceState_PhysDetached);
            hrc = E_UNEXPECTED;
        }
    }
    else
    {
        mMachine.setNull();
        if (!mIsPhysicallyDetached)
        {
            setState(kHostUSBDeviceState_HeldByProxy);
            if (hrc == E_UNEXPECTED)
                hrc = E_FAIL; /* No confusion. */
        }
        else
        {
            alock.release();
            onPhysicalDetachedInternal();
            hrc = E_UNEXPECTED;
        }
    }
    return hrc;
}


/**
 * Detaches the device from the VM.
 *
 * This is used for a special scenario in attachToVM() and from
 * onPhysicalDetachedInternal().
 *
 * @param   aFinalState     The final state (PhysDetached).
 */
void HostUSBDevice::detachFromVM(HostUSBDeviceState aFinalState)
{
    NOREF(aFinalState);

    /*
     * Assert preconditions.
     */
    Assert(aFinalState == kHostUSBDeviceState_PhysDetached);
    AssertReturnVoid(!isWriteLockOnCurrentThread());
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);
    Assert(   mUniState == kHostUSBDeviceState_AttachingToVM
           || mUniState == kHostUSBDeviceState_UsedByVM);
    Assert(!mMachine.isNull());

    /*
     * Change the state and abandon the locks. The VM may query
     * data and we don't want to deadlock - the state protects us,
     * so, it's not a bit issue here.
     */
    setState(kHostUSBDeviceState_PhysDetachingFromVM, kHostUSBDeviceState_PhysDetached);

    /*
     * Call the VM process (IPC) and request it to detach the device.
     *
     * There are many reasons for this to fail, so, as a consequence we don't
     * assert the return code as it will crash the daemon and annoy the heck
     * out of people.
     */
    alock.release();
    LogFlowThisFunc(("{%s} Calling machine->onUSBDeviceDetach()...\n", mName));
    HRESULT hrc = mMachine->onUSBDeviceDetach(mId.toUtf16().raw(), NULL);
    LogFlowThisFunc(("{%s} Done machine->onUSBDeviceDetach()=%Rhrc\n", mName, hrc));
    NOREF(hrc);

    /*
     * Re-acquire the locks and complete the transition.
     */
    alock.acquire();
    advanceTransition();
}

/**
 * Called when the VM process to inform us about the device being
 * detached from it.
 *
 * This is NOT called when we detach the device via onUSBDeviceDetach.
 *
 *
 * @param[in]   aMachine    The machine making the request.
 *                          This must be the machine this device is currently attached to.
 * @param[in]   aDone       When set to false, the VM just informs us that it's about
 *                          to detach this device but hasn't done it just yet.
 *                          When set to true, the VM informs us that it has completed
 *                          the detaching of this device.
 * @param[out]  aRunFilters Whether to run filters.
 * @param[in]   aAbnormal   Set if we're cleaning up after a crashed VM.
 *
 * @returns S_OK on success, and E_UNEXPECTED if the device isn't in the right state.
 *
 * @note    Must be called from under the object write lock.
 */
HRESULT HostUSBDevice::onDetachFromVM(SessionMachine *aMachine, bool aDone, bool *aRunFilters, bool aAbnormal /*= true*/)
{
    LogFlowThisFunc(("{%s} state=%s aDone=%RTbool aAbnormal=%RTbool\n", mName, getStateName(), aDone, aAbnormal));

    /*
     * Validate preconditions.
     */
    AssertPtrReturn(aRunFilters, E_INVALIDARG);
    AssertReturn(isWriteLockOnCurrentThread(), E_FAIL);
    if (!aDone)
    {
        if (mUniState != kHostUSBDeviceState_UsedByVM)
            return setError(E_INVALIDARG,
                            tr("USB device '%s' with UUID {%RTuuid} is busy (state '%s'). Please try again later"),
                            mName, mId.raw(), getStateName());
    }
    else
        AssertMsgReturn(    mUniState == kHostUSBDeviceState_DetachingFromVM /** @todo capturing for VM ends up here on termination. */
                        ||  (mUniState == kHostUSBDeviceState_UsedByVM && aAbnormal),
                        ("{%s} %s\n", mName, getStateName()), E_UNEXPECTED);
    AssertMsgReturn((mMachine == aMachine), ("%p != %p\n", (void *)mMachine, aMachine), E_FAIL);

    /*
     * Change the state.
     */
    if (!aDone)
    {
        *aRunFilters = startTransition(kHostUSBDeviceState_DetachingFromVM, kHostUSBDeviceState_HeldByProxy);
        /* PORTME: This might require host specific changes if you re-enumerate the device. */
    }
    else if (aAbnormal && mUniState == kHostUSBDeviceState_UsedByVM)
    {
        /* Fast forward thru the DetachingFromVM state and on to HeldByProxy. */
        /** @todo need to update the state machine to handle crashed VMs. */
        startTransition(kHostUSBDeviceState_DetachingFromVM, kHostUSBDeviceState_HeldByProxy);
        *aRunFilters = advanceTransition();
        mMachine.setNull();
        /* PORTME: ditto / trouble if you depend on the VM process to do anything. */
    }
    else
    {
        /* normal completion. */
        Assert(mUniSubState == kHostUSBDeviceSubState_Default); /* PORTME: ditto */
        *aRunFilters = advanceTransition();
        mMachine.setNull();
    }

    return S_OK;
}

/**
 * Requests the USB proxy service to release the device back to the host.
 *
 * This method will ignore (not assert) calls for devices that already
 * belong to the host because it simplifies the usage a bit.
 *
 * @returns COM status code.
 * @retval  S_OK on success.
 * @retval  E_UNEXPECTED on bad state.
 * @retval  E_* as appropriate.
 *
 * @note Must be called without holding the object lock.
 */
HRESULT HostUSBDevice::requestReleaseToHost()
{
    /*
     * Validate preconditions.
     */
    AssertReturn(!isWriteLockOnCurrentThread(), E_FAIL);
    Assert(mMachine.isNull());

    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);
    LogFlowThisFunc(("{%s}\n", mName));
    if (    mUniState == kHostUSBDeviceState_Unused
        ||  mUniState == kHostUSBDeviceState_Capturable)
        return S_OK;
    AssertMsgReturn(mUniState == kHostUSBDeviceState_HeldByProxy, ("{%s} %s\n", mName, getStateName()), E_UNEXPECTED);

    /*
     * Try release it.
     */
#if (defined(RT_OS_DARWIN) && defined(VBOX_WITH_NEW_USB_CODE_ON_DARWIN)) /* PORTME */ \
 || defined(RT_OS_WINDOWS)
    startTransition(kHostUSBDeviceState_ReleasingToHost, kHostUSBDeviceState_Unused, kHostUSBDeviceSubState_AwaitingDetach);
#else
    startTransition(kHostUSBDeviceState_ReleasingToHost, kHostUSBDeviceState_Unused);
#endif
    alock.release();
    int rc = mUSBProxyService->releaseDevice(this);
    if (RT_FAILURE(rc))
    {
        alock.acquire();
        failTransition();
        return E_FAIL;
    }
    return S_OK;
}

/**
 * Requests the USB proxy service to capture and hold the device.
 *
 * The device must be owned by the host at the time of the call. But for
 * the callers convenience, calling this method on a device that is already
 * being held will success without any assertions.
 *
 * @returns COM status code.
 * @retval  S_OK on success.
 * @retval  E_UNEXPECTED on bad state.
 * @retval  E_* as appropriate.
 *
 * @note Must be called without holding the object lock.
 */
HRESULT HostUSBDevice::requestHold()
{
    /*
     * Validate preconditions.
     */
    AssertReturn(!isWriteLockOnCurrentThread(), E_FAIL);
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);
    LogFlowThisFunc(("{%s}\n", mName));
    AssertMsgReturn(   mUniState == kHostUSBDeviceState_Unused
                    || mUniState == kHostUSBDeviceState_Capturable
                    || mUniState == kHostUSBDeviceState_HeldByProxy,
                    ("{%s} %s\n", mName, getStateName()),
                    E_UNEXPECTED);

    Assert(mMachine.isNull());
    mMachine.setNull();

    if (mUniState == kHostUSBDeviceState_HeldByProxy)
        return S_OK;

    /*
     * Do the job.
     */
#if (defined(RT_OS_DARWIN) && defined(VBOX_WITH_NEW_USB_CODE_ON_DARWIN)) /* PORTME */ \
 || defined(RT_OS_WINDOWS)
    startTransition(kHostUSBDeviceState_Capturing, kHostUSBDeviceState_HeldByProxy, kHostUSBDeviceSubState_AwaitingDetach);
#else
    startTransition(kHostUSBDeviceState_Capturing, kHostUSBDeviceState_HeldByProxy);
#endif
    alock.release();
    int rc = mUSBProxyService->captureDevice(this);
    if (RT_FAILURE(rc))
    {
        alock.acquire();
        failTransition();
        return E_FAIL;
    }
    return S_OK;
}


/**
 * Check a detach detected by the USB Proxy Service to see if
 * it's a real one or just a logical following a re-enumeration.
 *
 * This will work the internal sub state of the device and do time
 * outs, so it does more than just querying data!
 *
 * @returns true if it was actually detached, false if it's just a re-enumeration.
 */
bool HostUSBDevice::wasActuallyDetached()
{
    /*
     * This only applies to the detach and re-attach states.
     */
    switch (mUniState)
    {
        case kHostUSBDeviceState_Capturing:
        case kHostUSBDeviceState_ReleasingToHost:
        case kHostUSBDeviceState_AttachingToVM:
        case kHostUSBDeviceState_DetachingFromVM:
            switch (mUniSubState)
            {
                /*
                 * If we're awaiting a detach, the this has now occurred
                 * and the state should be advanced.
                 */
                case kHostUSBDeviceSubState_AwaitingDetach:
                    advanceTransition();
                    return false; /* not physically detached. */

                /*
                 * Check for timeouts.
                 */
                case kHostUSBDeviceSubState_AwaitingReAttach:
                {
#ifndef RT_OS_WINDOWS /* check the implementation details here. */
                    uint64_t elapsedNanoseconds = RTTimeNanoTS() - mLastStateChangeTS;
                    if (elapsedNanoseconds > UINT64_C(60000000000)) /* 60 seconds */
                    {
                        LogRel(("USB: Async operation timed out for device %s (state: %s)\n", mName, getStateName()));
                        failTransition();
                    }
#endif
                    return false; /* not physically detached. */
                }

                /* not applicable.*/
                case kHostUSBDeviceSubState_Default:
                    break;
            }
            break;

        /* not applicable. */
        case kHostUSBDeviceState_Unsupported:
        case kHostUSBDeviceState_UsedByHost:
        case kHostUSBDeviceState_Capturable:
        case kHostUSBDeviceState_Unused:
        case kHostUSBDeviceState_HeldByProxy:
        case kHostUSBDeviceState_UsedByVM:
        case kHostUSBDeviceState_PhysDetachingFromVM:
        case kHostUSBDeviceState_PhysDetached:
            break;

        default:
            AssertLogRelMsgFailed(("this=%p %s\n", this, getStateName()));
            break;
    }

    /* It was detached. */
    return true;
}

/**
 * Notification from the USB Proxy that the device was physically detached.
 *
 * If a transition is pending, mIsPhysicallyDetached will be set and
 * handled when the transition advances forward.
 *
 * Otherwise the device will be detached from any VM currently using it - this
 * involves IPC and will temporarily abandon locks - and all the device data
 * reset.
 */
void HostUSBDevice::onPhysicalDetached()
{
    AssertReturnVoid(!isWriteLockOnCurrentThread());
    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);
    LogFlowThisFunc(("{%s}\n", mName));

    mIsPhysicallyDetached = true;
    if (mUniState < kHostUSBDeviceState_FirstTransitional)
    {
        alock.release();
        onPhysicalDetachedInternal();
    }
}


/**
 * Do the physical detach work for a device in a stable state or
 * at a transition state change.
 *
 * See onPhysicalDetach() for details.
 */
void HostUSBDevice::onPhysicalDetachedInternal()
{
    AssertReturnVoid(!isWriteLockOnCurrentThread());
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);
    LogFlowThisFunc(("{%s}\n", mName));
    Assert(mIsPhysicallyDetached);

    /*
     * Do we need to detach it from the VM first?
     */
    if (    !mMachine.isNull()
        &&  (   mUniState == kHostUSBDeviceState_UsedByVM
             || mUniState == kHostUSBDeviceState_AttachingToVM))
    {
        alock.release();
        detachFromVM(kHostUSBDeviceState_PhysDetached);
        alock.acquire();
    }
    else
        AssertMsg(mMachine.isNull(), ("%s\n", getStateName()));

    /*
     * Reset the data and enter the final state.
     */
    mMachine.setNull();
    setState(kHostUSBDeviceState_PhysDetached);
}


/**
 *  Returns true if this device matches the given filter data.
 *
 *  @note It is assumed, that the filter data owner is appropriately
 *        locked before calling this method.
 *
 *  @note
 *      This method MUST correlate with
 *      USBController::hasMatchingFilter (IUSBDevice *)
 *      in the sense of the device matching logic.
 *
 *  @note Locks this object for reading.
 */
bool HostUSBDevice::isMatch(const USBDeviceFilter::Data &aData)
{
    AutoCaller autoCaller(this);
    AssertComRCReturn(autoCaller.rc(), false);

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    if (!aData.mActive)
        return false;

    if (!aData.mRemote.isMatch(FALSE))
        return false;

    if (!USBFilterMatchDevice(&aData.mUSBFilter, mUsb))
        return false;

    /* Don't match busy devices with a 100% wildcard filter - this will
       later become a filter prop (ring-3 only). */
    if (    mUsb->enmState == USBDEVICESTATE_USED_BY_HOST_CAPTURABLE
        &&  !USBFilterHasAnySubstatialCriteria(&aData.mUSBFilter))
        return false;

    LogFlowThisFunc(("returns true\n"));
    return true;
}

/**
 * Compares this device with a USBDEVICE and decides if the match or which comes first.
 *
 * This will take into account device re-attaching and omit the bits
 * that may change during a device re-enumeration.
 *
 * @param   aDev2   Device 2.
 *
 * @returns < 0 if this should come before aDev2.
 * @returns   0 if this and aDev2 are equal.
 * @returns > 0 if this should come after aDev2.
 *
 * @note Must be called from under the object write lock.
 */
int HostUSBDevice::compare(PCUSBDEVICE aDev2)
{
    AssertReturn(isWriteLockOnCurrentThread(), -1);
    //Log3(("%Rfn: %p {%s}\n", __PRETTY_FUNCTION__, this, mName));
    return compare(mUsb, aDev2,
                      mUniSubState == kHostUSBDeviceSubState_AwaitingDetach /* (In case we don't get the detach notice.) */
                   || mUniSubState == kHostUSBDeviceSubState_AwaitingReAttach);
}

/**
 * Compares two USBDEVICE structures and decides if the match or which comes first.
 *
 * @param   aDev1               Device 1.
 * @param   aDev2               Device 2.
 * @param   aIsAwaitingReAttach Whether to omit bits that will change in a device
 *                              re-enumeration (true) or not (false).
 *
 * @returns < 0 if aDev1 should come before aDev2.
 * @returns   0 if aDev1 and aDev2 are equal.
 * @returns > 0 if aDev1 should come after aDev2.
 */
/*static*/
int HostUSBDevice::compare(PCUSBDEVICE aDev1, PCUSBDEVICE aDev2, bool aIsAwaitingReAttach /*= false */)
{
    /*
     * Things that stays the same everywhere.
     *
     * The more uniquely these properties identifies a device the less the chance
     * that we mix similar devices during re-enumeration. Bus+port would help
     * provide ~99.8% accuracy if the host can provide those attributes.
     */
    int iDiff = aDev1->idVendor - aDev2->idVendor;
    if (iDiff)
        return iDiff;

    iDiff = aDev1->idProduct - aDev2->idProduct;
    if (iDiff)
        return iDiff;

    iDiff = aDev1->bcdDevice - aDev2->bcdDevice;
    if (iDiff)
    {
        //Log3(("compare: bcdDevice: %#x != %#x\n", aDev1->bcdDevice, aDev2->bcdDevice));
        return iDiff;
    }

#ifdef RT_OS_WINDOWS /* the string query may fail on windows during replugging, ignore serial mismatch if this is the case. */
    if (    aDev1->u64SerialHash != aDev2->u64SerialHash
        &&  (   !aIsAwaitingReAttach
             || (aDev2->pszSerialNumber && *aDev2->pszSerialNumber)
             || (aDev2->pszManufacturer && *aDev2->pszManufacturer)
             || (aDev2->pszProduct      && *aDev2->pszProduct))
       )
#else
    if (aDev1->u64SerialHash != aDev2->u64SerialHash)
#endif
    {
        //Log3(("compare: u64SerialHash: %#llx != %#llx\n", aDev1->u64SerialHash, aDev2->u64SerialHash));
        return aDev1->u64SerialHash < aDev2->u64SerialHash ? -1 : 1;
    }

    /* The hub/bus + port should help a lot in a re-attach situation. */
#ifdef RT_OS_WINDOWS
    iDiff = strcmp(aDev1->pszHubName, aDev2->pszHubName);
    if (iDiff)
    {
        //Log3(("compare: HubName: %s != %s\n", aDev1->pszHubName, aDev2->pszHubName));
        return iDiff;
    }
#else
    iDiff = aDev1->bBus - aDev2->bBus;
    if (iDiff)
    {
        //Log3(("compare: bBus: %#x != %#x\n", aDev1->bBus, aDev2->bBus));
        return iDiff;
    }
#endif

    iDiff = aDev1->bPort - aDev2->bPort;    /* shouldn't change anywhere and help pinpoint it very accurately. */
    if (iDiff)
    {
        //Log3(("compare: bPort: %#x != %#x\n", aDev1->bPort, aDev2->bPort));
        return iDiff;
    }

    /*
     * Things that usually doesn't stay the same when re-enumerating
     * a device. The fewer things in the category the better chance
     * that we avoid messing up when more than one device of the same
     * kind is attached.
     */
    if (aIsAwaitingReAttach)
    {
        //Log3(("aDev1=%p == aDev2=%p\n", aDev1, aDev2));
        return 0;
    }
    /* device number always changes. */
    return strcmp(aDev1->pszAddress, aDev2->pszAddress);
}

/**
 *  Updates the state of the device.
 *
 *  If this method returns @c true, Host::onUSBDeviceStateChanged() will be
 *  called to process the state change (complete the state change request,
 *  inform the VM process etc.).
 *
 *  If this method returns @c false, it is assumed that the given state change
 *  is "minor": it doesn't require any further action other than update the
 *  mState field with the actual state value.
 *
 *  Regardless of the return value, this method always takes ownership of the
 *  new USBDEVICE structure passed in and updates the pNext and pPrev fiends in
 *  it using the values of the old structure.
 *
 * @param[in]   aDev            The current device state as seen by the proxy backend.
 * @param[out]  aRunFilters     Whether the state change should be accompanied by
 *                              running filters on the device.
 * @param[out]  aIgnoreMachine  Machine to ignore when running filters.
 *
 * @returns Whether the Host object should be bothered with this state change.
 *
 * @todo    Just do everything here, that is, call filter runners and everything that
 *          works by state change. Using 3 return codes/parameters is just plain ugly.
 */
bool HostUSBDevice::updateState(PCUSBDEVICE aDev, bool *aRunFilters, SessionMachine **aIgnoreMachine)
{
    *aRunFilters = false;
    *aIgnoreMachine = NULL;

    /*
     * Locking.
     */
    AssertReturn(!isWriteLockOnCurrentThread(), false);
    AutoCaller autoCaller(this);
    AssertComRCReturn(autoCaller.rc(), false);
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    /*
     * Replace the existing structure by the new one.
     */
    const USBDEVICESTATE enmOldState = mUsb->enmState; NOREF(enmOldState);
    if (mUsb != aDev)
    {
#ifdef RT_OS_WINDOWS
        /* we used this logic of string comparison in HostUSBDevice::compare
         * now we need to preserve strings from the old device if the new device has zero strings
         * this ensures the device is correctly matched later on
         * otherwise we may end up with a phantom misconfigured device instance */
        if ((mUniSubState == kHostUSBDeviceSubState_AwaitingDetach /* (In case we don't get the detach notice.) */
                   || mUniSubState == kHostUSBDeviceSubState_AwaitingReAttach)
               && (!aDev->pszSerialNumber || !*aDev->pszSerialNumber)
               && (!aDev->pszManufacturer || !*aDev->pszManufacturer)
               && (!aDev->pszProduct      || !*aDev->pszProduct))
        {
            aDev->u64SerialHash = mUsb->u64SerialHash;

            if (mUsb->pszSerialNumber && *mUsb->pszSerialNumber)
            {
                if (aDev->pszSerialNumber)
                    RTStrFree((char *)aDev->pszSerialNumber);

                /* since we're going to free old device later on,
                 * we can just assign the string from it to the new device
                 * and zero up the string filed for the old device */
                aDev->pszSerialNumber = mUsb->pszSerialNumber;
                mUsb->pszSerialNumber = NULL;
            }

            if (mUsb->pszManufacturer && *mUsb->pszManufacturer)
            {
                if (aDev->pszManufacturer)
                    RTStrFree((char *)aDev->pszManufacturer);

                /* since we're going to free old device later on,
                 * we can just assign the string from it to the new device
                 * and zero up the string filed for the old device */
                aDev->pszManufacturer = mUsb->pszManufacturer;
                mUsb->pszManufacturer = NULL;
            }

            if (mUsb->pszProduct && *mUsb->pszProduct)
            {
                if (aDev->pszProduct)
                    RTStrFree((char *)aDev->pszProduct);

                /* since we're going to free old device later on,
                 * we can just assign the string from it to the new device
                 * and zero up the string filed for the old device */
                aDev->pszProduct = mUsb->pszProduct;
                mUsb->pszProduct = NULL;
            }
        }
#endif
        aDev->pNext = mUsb->pNext;
        aDev->pPrev = mUsb->pPrev;
        USBProxyService::freeDevice(mUsb);
        mUsb = aDev;
    }

/** @def HOSTUSBDEVICE_FUZZY_STATE
 * Defined on hosts where we have a driver that keeps proper device states.
 */
# if defined(RT_OS_LINUX) || defined(DOXYGEN_RUNNING)
#  define HOSTUSBDEVICE_FUZZY_STATE 1
# else
#  undef  HOSTUSBDEVICE_FUZZY_STATE
# endif
    /*
     * For some hosts we'll have to be pretty careful here because
     * they don't always have a clue what is going on. This is
     * particularly true on linux and solaris, while windows and
     * darwin generally knows a bit more.
     */
    bool fIsImportant = false;
    if (enmOldState != mUsb->enmState)
    {
        LogFlowThisFunc(("%p {%s} %s\n", this, mName, getStateName()));
        switch (mUsb->enmState)
        {
            /*
             * Little fuzziness here, except where we fake capture.
             */
            case USBDEVICESTATE_USED_BY_HOST:
                switch (mUniState)
                {
                    /* Host drivers installed, that's fine. */
                    case kHostUSBDeviceState_Capturable:
                    case kHostUSBDeviceState_Unused:
                        LogThisFunc(("{%s} %s -> %s\n", mName, getStateName(), stateName(kHostUSBDeviceState_UsedByHost)));
                        *aRunFilters = setState(kHostUSBDeviceState_UsedByHost);
                        break;
                    case kHostUSBDeviceState_UsedByHost:
                        break;

                    /* Can only mean that we've failed capturing it. */
                    case kHostUSBDeviceState_Capturing:
                        LogThisFunc(("{%s} capture failed!\n", mName));
                        mUSBProxyService->captureDeviceCompleted(this, false /* aSuccess */);
                        *aRunFilters = failTransition();
                        mMachine.setNull();
                        break;

                    /* Guess we've successfully released it. */
                    case kHostUSBDeviceState_ReleasingToHost:
                        LogThisFunc(("{%s} %s -> %s\n", mName, getStateName(), stateName(kHostUSBDeviceState_UsedByHost)));
                        mUSBProxyService->releaseDeviceCompleted(this, true /* aSuccess */);
                        *aRunFilters = setState(kHostUSBDeviceState_UsedByHost);
                        break;

                    /* These are IPC states and should be left alone. */
                    case kHostUSBDeviceState_AttachingToVM:
                    case kHostUSBDeviceState_DetachingFromVM:
                    case kHostUSBDeviceState_PhysDetachingFromVM:
                        LogThisFunc(("{%s} %s - changed to USED_BY_HOST...\n", mName, getStateName()));
                        break;

#ifdef HOSTUSBDEVICE_FUZZY_STATE
                    /* Fake: We can't prevent anyone from grabbing it. */
                    case kHostUSBDeviceState_HeldByProxy:
                        LogThisFunc(("{%s} %s -> %s!\n", mName, getStateName(), stateName(kHostUSBDeviceState_UsedByHost)));
                        *aRunFilters = setState(kHostUSBDeviceState_UsedByHost);
                        break;
                    //case kHostUSBDeviceState_UsedByVM:
                    //    /** @todo needs to be detached from the VM. */
                    //    break;
#endif
                    /* Not supposed to happen... */
#ifndef HOSTUSBDEVICE_FUZZY_STATE
                    case kHostUSBDeviceState_HeldByProxy:
#endif
                    case kHostUSBDeviceState_UsedByVM:
                    case kHostUSBDeviceState_PhysDetached:
                    case kHostUSBDeviceState_Unsupported:
                    default:
                        AssertMsgFailed(("{%s} %s\n", mName, getStateName()));
                        break;
                }
                break;

            /*
             * It changed to capturable. Fuzzy hosts might easily
             * confuse UsedByVM with this one.
             */
            case USBDEVICESTATE_USED_BY_HOST_CAPTURABLE:
                switch (mUniState)
                {
                    /* No change. */
#ifdef HOSTUSBDEVICE_FUZZY_STATE
                    case kHostUSBDeviceState_HeldByProxy:
                    case kHostUSBDeviceState_UsedByVM:
#endif
                    case kHostUSBDeviceState_Capturable:
                        break;

                    /* Changed! */
                    case kHostUSBDeviceState_UsedByHost:
                        fIsImportant = true;
                    case kHostUSBDeviceState_Unused:
                        LogThisFunc(("{%s} %s -> %s\n", mName, getStateName(), stateName(kHostUSBDeviceState_Capturable)));
                        *aRunFilters = setState(kHostUSBDeviceState_Capturable);
                        break;

                    /* Can only mean that we've failed capturing it. */
                    case kHostUSBDeviceState_Capturing:
                        LogThisFunc(("{%s} capture failed!\n", mName));
                        mUSBProxyService->captureDeviceCompleted(this, false /* aSuccess */);
                        *aRunFilters = failTransition();
                        mMachine.setNull();
                        break;

                    /* Guess we've successfully released it. */
                    case kHostUSBDeviceState_ReleasingToHost:
                        LogThisFunc(("{%s} %s -> %s\n", mName, getStateName(), stateName(kHostUSBDeviceState_Capturable)));
                        mUSBProxyService->releaseDeviceCompleted(this, true /* aSuccess */);
                        *aRunFilters = setState(kHostUSBDeviceState_Capturable);
                        break;

                    /* These are IPC states and should be left alone. */
                    case kHostUSBDeviceState_AttachingToVM:
                    case kHostUSBDeviceState_DetachingFromVM:
                    case kHostUSBDeviceState_PhysDetachingFromVM:
                        LogThisFunc(("{%s} %s - changed to USED_BY_HOST_CAPTURABLE...\n", mName, getStateName()));
                        break;

                    /* Not supposed to happen*/
#ifndef HOSTUSBDEVICE_FUZZY_STATE
                    case kHostUSBDeviceState_HeldByProxy:
                    case kHostUSBDeviceState_UsedByVM:
#endif
                    case kHostUSBDeviceState_Unsupported:
                    case kHostUSBDeviceState_PhysDetached:
                    default:
                        AssertMsgFailed(("{%s} %s\n", mName, getStateName()));
                        break;
                }
                break;


            /*
             * It changed to capturable. Fuzzy hosts might easily
             * confuse UsedByVM and HeldByProxy with this one.
             */
            case USBDEVICESTATE_UNUSED:
                switch (mUniState)
                {
                    /* No change. */
#ifdef HOSTUSBDEVICE_FUZZY_STATE
                    case kHostUSBDeviceState_HeldByProxy:
                    case kHostUSBDeviceState_UsedByVM:
#endif
                    case kHostUSBDeviceState_Unused:
                        break;

                    /* Changed! */
                    case kHostUSBDeviceState_UsedByHost:
                    case kHostUSBDeviceState_Capturable:
                        fIsImportant = true;
                        LogThisFunc(("{%s} %s -> %s\n", mName, getStateName(), stateName(kHostUSBDeviceState_Unused)));
                        *aRunFilters = setState(kHostUSBDeviceState_Unused);
                        break;

                    /* Can mean that we've failed capturing it, but on windows it is the detach signal. */
                    case kHostUSBDeviceState_Capturing:
#if defined(RT_OS_WINDOWS)
                        if (mUniSubState == kHostUSBDeviceSubState_AwaitingDetach)
                        {
                            LogThisFunc(("{%s} capture advancing thru UNUSED...\n", mName));
                            *aRunFilters = advanceTransition();
                        }
                        else
#endif
                        {
                            LogThisFunc(("{%s} capture failed!\n", mName));
                            mUSBProxyService->captureDeviceCompleted(this, false /* aSuccess */);
                            *aRunFilters = failTransition();
                            mMachine.setNull();
                        }
                        break;

                    /* Guess we've successfully released it. */
                    case kHostUSBDeviceState_ReleasingToHost:
                        LogThisFunc(("{%s} %s -> %s\n", mName, getStateName(), stateName(kHostUSBDeviceState_Unused)));
                        mUSBProxyService->releaseDeviceCompleted(this, true /* aSuccess */);
                        *aRunFilters = setState(kHostUSBDeviceState_Unused);
                        break;

                    /* These are IPC states and should be left alone. */
                    case kHostUSBDeviceState_AttachingToVM:
                    case kHostUSBDeviceState_DetachingFromVM:
                    case kHostUSBDeviceState_PhysDetachingFromVM:
                        LogThisFunc(("{%s} %s - changed to UNUSED...\n", mName, getStateName()));
                        break;

                    /* Not supposed to happen*/
#ifndef HOSTUSBDEVICE_FUZZY_STATE
                    case kHostUSBDeviceState_HeldByProxy:
                    case kHostUSBDeviceState_UsedByVM:
#endif
                    case kHostUSBDeviceState_Unsupported:
                    case kHostUSBDeviceState_PhysDetached:
                    default:
                        AssertMsgFailed(("{%s} %s\n", mName, getStateName()));
                        break;
                }
                break;

            /*
             * This is pretty straight forward, except that everyone
             * might sometimes confuse this and the UsedByVM state.
             */
            case USBDEVICESTATE_HELD_BY_PROXY:
                switch (mUniState)
                {
                    /* No change. */
                    case kHostUSBDeviceState_HeldByProxy:
                        break;
                    case kHostUSBDeviceState_UsedByVM:
                        LogThisFunc(("{%s} %s - changed to HELD_BY_PROXY...\n", mName, getStateName()));
                        break;

                    /* Guess we've successfully captured it. */
                    case kHostUSBDeviceState_Capturing:
                        LogThisFunc(("{%s} capture succeeded!\n", mName));
                        mUSBProxyService->captureDeviceCompleted(this, true /* aSuccess */);
                        *aRunFilters = advanceTransition(true /* fast forward thru re-attach */);

                        /* Take action if we're supposed to attach it to a VM. */
                        if (mUniState == kHostUSBDeviceState_AttachingToVM)
                        {
                            alock.release();
                            attachToVM(mMachine, mMaskedIfs);
                            alock.acquire();
                        }
                        break;

                    /* Can only mean that we've failed capturing it. */
                    case kHostUSBDeviceState_ReleasingToHost:
                        LogThisFunc(("{%s} %s failed!\n", mName, getStateName()));
                        mUSBProxyService->releaseDeviceCompleted(this, false /* aSuccess */);
                        *aRunFilters = setState(kHostUSBDeviceState_HeldByProxy);
                        break;

                    /* These are IPC states and should be left alone. */
                    case kHostUSBDeviceState_AttachingToVM:
                    case kHostUSBDeviceState_DetachingFromVM:
                    case kHostUSBDeviceState_PhysDetachingFromVM:
                        LogThisFunc(("{%s} %s - changed to HELD_BY_PROXY...\n", mName, getStateName()));
                        break;

                    /* Not supposed to happen. */
                    case kHostUSBDeviceState_Unsupported:
                    case kHostUSBDeviceState_UsedByHost:
                    case kHostUSBDeviceState_Capturable:
                    case kHostUSBDeviceState_Unused:
                    case kHostUSBDeviceState_PhysDetached:
                    default:
                        AssertMsgFailed(("{%s} %s\n", mName, getStateName()));
                        break;
                }
                break;

            /*
             * This is very straight forward and only Darwin implements it.
             */
            case USBDEVICESTATE_USED_BY_GUEST:
                switch (mUniState)
                {
                    /* No change. */
                    case kHostUSBDeviceState_HeldByProxy:
                        LogThisFunc(("{%s} %s - changed to USED_BY_GUEST...\n", mName, getStateName()));
                        break;
                    case kHostUSBDeviceState_UsedByVM:
                        break;

                    /* These are IPC states and should be left alone. */
                    case kHostUSBDeviceState_AttachingToVM:
                    case kHostUSBDeviceState_DetachingFromVM:
                    case kHostUSBDeviceState_PhysDetachingFromVM:
                        LogThisFunc(("{%s} %s - changed to USED_BY_GUEST...\n", mName, getStateName()));
                        break;

                    /* Not supposed to happen. */
                    case kHostUSBDeviceState_Unsupported:
                    case kHostUSBDeviceState_Capturable:
                    case kHostUSBDeviceState_Unused:
                    case kHostUSBDeviceState_UsedByHost:
                    case kHostUSBDeviceState_PhysDetached:
                    case kHostUSBDeviceState_ReleasingToHost:
                    case kHostUSBDeviceState_Capturing:
                    default:
                        AssertMsgFailed(("{%s} %s\n", mName, getStateName()));
                        break;
                }
                break;

            /*
             * This is not supposed to happen and indicates a bug in the backend!
             */
            case USBDEVICESTATE_UNSUPPORTED:
                AssertMsgFailed(("enmOldState=%d {%s} %s\n", enmOldState, mName, getStateName()));
                break;
            default:
                AssertMsgFailed(("enmState=%d {%s} %s\n", mUsb->enmState, mName, getStateName()));
                break;
        }
    }
    else if (   mUniSubState == kHostUSBDeviceSubState_AwaitingDetach
             && hasAsyncOperationTimedOut())
    {
        LogRel(("USB: timeout in %s for {%RTuuid} / {%s}\n",
                getStateName(), mId.raw(), mName));
        *aRunFilters = failTransition();
        fIsImportant = true;
    }
    else
    {
        LogFlowThisFunc(("%p {%s} %s - no change %d\n", this, mName, getStateName(), enmOldState));
        /** @todo might have to handle some stuff here too if we cannot make the release/capture handling deal with that above ... */
    }

    return fIsImportant;
}


/**
 * Updates the state of the device, checking for cases which we fake.
 *
 * See HostUSBDevice::updateState() for details.
 *
 * @param[in]   aDev            See HostUSBDevice::updateState().
 * @param[out]  aRunFilters     See HostUSBDevice::updateState()
 * @param[out]  aIgnoreMachine  See HostUSBDevice::updateState()
 *
 * @returns See HostUSBDevice::updateState()
 */
bool HostUSBDevice::updateStateFake(PCUSBDEVICE aDev, bool *aRunFilters, SessionMachine **aIgnoreMachine)
{
    Assert(!isWriteLockOnCurrentThread());
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);
    const HostUSBDeviceState enmState = mUniState;
    switch (enmState)
    {
        case kHostUSBDeviceState_Capturing:
        case kHostUSBDeviceState_ReleasingToHost:
        {
            *aIgnoreMachine = mUniState == kHostUSBDeviceState_ReleasingToHost ? mMachine : NULL;
            *aRunFilters = advanceTransition();
            LogThisFunc(("{%s} %s\n", mName, getStateName()));

            if (mUsb != aDev)
            {
                aDev->pNext = mUsb->pNext;
                aDev->pPrev = mUsb->pPrev;
                USBProxyService::freeDevice(mUsb);
                mUsb = aDev;
            }

            /* call the completion method */
            if (enmState == kHostUSBDeviceState_Capturing)
                mUSBProxyService->captureDeviceCompleted(this, true /* aSuccess */);
            else
                mUSBProxyService->releaseDeviceCompleted(this, true /* aSuccess */);

            /* Take action if we're supposed to attach it to a VM. */
            if (mUniState == kHostUSBDeviceState_AttachingToVM)
            {
                alock.release();
                attachToVM(mMachine, mMaskedIfs);
            }
            return true;
        }

        default:
            alock.release();
            return updateState(aDev, aRunFilters, aIgnoreMachine);
    }
}


/**
 * Checks if there is a pending asynchronous operation and whether
 * it has timed out or not.
 *
 * @returns true on timeout, false if not.
 *
 * @note    Caller must have read or write locked the object before calling.
 */
bool HostUSBDevice::hasAsyncOperationTimedOut() const
{
    switch (mUniSubState)
    {
#ifndef RT_OS_WINDOWS /* no timeouts on windows yet since I don't have all the details here... */
        case kHostUSBDeviceSubState_AwaitingDetach:
        case kHostUSBDeviceSubState_AwaitingReAttach:
        {
            uint64_t elapsedNanoseconds = RTTimeNanoTS() - mLastStateChangeTS;
            return elapsedNanoseconds > UINT64_C(60000000000); /* 60 seconds */  /* PORTME */
        }
#endif
        default:
            return false;
    }
}


/**
 * Translate the state into
 *
 * @returns
 * @param   aState
 * @param   aSubState
 * @param   aPendingState
 */
/*static*/ const char *HostUSBDevice::stateName(HostUSBDeviceState aState,
                                                HostUSBDeviceState aPendingState /*= kHostUSBDeviceState_Invalid*/,
                                                HostUSBDeviceSubState aSubState /*= kHostUSBDeviceSubState_Default*/)
{
    switch (aState)
    {
        case kHostUSBDeviceState_Unsupported:
            AssertReturn(aPendingState == kHostUSBDeviceState_Invalid, "Unsupported{bad}");
            AssertReturn(aSubState == kHostUSBDeviceSubState_Default, "Unsupported[bad]");
            return "Unsupported";

        case kHostUSBDeviceState_UsedByHost:
            AssertReturn(aPendingState == kHostUSBDeviceState_Invalid, "UsedByHost{bad}");
            AssertReturn(aSubState == kHostUSBDeviceSubState_Default, "UsedByHost[bad]");
            return "UsedByHost";

        case kHostUSBDeviceState_Capturable:
            AssertReturn(aPendingState == kHostUSBDeviceState_Invalid, "Capturable{bad}");
            AssertReturn(aSubState == kHostUSBDeviceSubState_Default, "Capturable[bad]");
            return "Capturable";

        case kHostUSBDeviceState_Unused:
            AssertReturn(aPendingState == kHostUSBDeviceState_Invalid, "Unused{bad}");
            AssertReturn(aSubState == kHostUSBDeviceSubState_Default, "Unused[bad]");
            return "Unused";

        case kHostUSBDeviceState_HeldByProxy:
            AssertReturn(aPendingState == kHostUSBDeviceState_Invalid, "HeldByProxy{bad}");
            AssertReturn(aSubState == kHostUSBDeviceSubState_Default, "HeldByProxy[bad]");
            return "HeldByProxy";

        case kHostUSBDeviceState_UsedByVM:
            AssertReturn(aPendingState == kHostUSBDeviceState_Invalid, "UsedByVM{bad}");
            AssertReturn(aSubState == kHostUSBDeviceSubState_Default, "UsedByVM[bad]");
            return "UsedByVM";

        case kHostUSBDeviceState_PhysDetached:
            AssertReturn(aPendingState == kHostUSBDeviceState_Invalid, "PhysDetached{bad}");
            AssertReturn(aSubState == kHostUSBDeviceSubState_Default, "PhysDetached[bad]");
            return "PhysDetached";

        case kHostUSBDeviceState_Capturing:
            switch (aPendingState)
            {
                case kHostUSBDeviceState_UsedByVM:
                    switch (aSubState)
                    {
                        case kHostUSBDeviceSubState_Default:
                            return "CapturingForVM";
                        case kHostUSBDeviceSubState_AwaitingDetach:
                            return "CapturingForVM[Detach]";
                        case kHostUSBDeviceSubState_AwaitingReAttach:
                            return "CapturingForVM[Attach]";
                        default:
                            AssertFailedReturn("CapturingForVM[bad]");
                    }
                    break;

                case kHostUSBDeviceState_HeldByProxy:
                    switch (aSubState)
                    {
                        case kHostUSBDeviceSubState_Default:
                            return "CapturingForProxy";
                        case kHostUSBDeviceSubState_AwaitingDetach:
                            return "CapturingForProxy[Detach]";
                        case kHostUSBDeviceSubState_AwaitingReAttach:
                            return "CapturingForProxy[Attach]";
                        default:
                            AssertFailedReturn("CapturingForProxy[bad]");
                    }
                    break;

                default:
                    AssertFailedReturn("Capturing{bad}");
            }
            break;

        case kHostUSBDeviceState_ReleasingToHost:
            switch (aPendingState)
            {
                case kHostUSBDeviceState_Unused:
                    switch (aSubState)
                    {
                        case kHostUSBDeviceSubState_Default:
                            return "ReleasingToHost";
                        case kHostUSBDeviceSubState_AwaitingDetach:
                            return "ReleasingToHost[Detach]";
                        case kHostUSBDeviceSubState_AwaitingReAttach:
                            return "ReleasingToHost[Attach]";
                        default:
                            AssertFailedReturn("ReleasingToHost[bad]");
                    }
                    break;
                default:
                    AssertFailedReturn("ReleasingToHost{bad}");
            }
            break;

        case kHostUSBDeviceState_DetachingFromVM:
            switch (aPendingState)
            {
                case kHostUSBDeviceState_HeldByProxy:
                    switch (aSubState)
                    {
                        case kHostUSBDeviceSubState_Default:
                            return "DetatchingFromVM>Proxy";
                        case kHostUSBDeviceSubState_AwaitingDetach:
                            return "DetatchingFromVM>Proxy[Detach]";
                        case kHostUSBDeviceSubState_AwaitingReAttach:
                            return "DetatchingFromVM>Proxy[Attach]";
                        default:
                            AssertFailedReturn("DetatchingFromVM>Proxy[bad]");
                    }
                    break;

                case kHostUSBDeviceState_Unused:
                    switch (aSubState)
                    {
                        case kHostUSBDeviceSubState_Default:
                            return "DetachingFromVM>Host";
                        case kHostUSBDeviceSubState_AwaitingDetach:
                            return "DetachingFromVM>Host[Detach]";
                        case kHostUSBDeviceSubState_AwaitingReAttach:
                            return "DetachingFromVM>Host[Attach]";
                        default:
                            AssertFailedReturn("DetachingFromVM>Host[bad]");
                    }
                    break;

                default:
                    AssertFailedReturn("DetachingFromVM{bad}");
            }
            break;

        case kHostUSBDeviceState_AttachingToVM:
            switch (aPendingState)
            {
                case kHostUSBDeviceState_UsedByVM:
                    switch (aSubState)
                    {
                        case kHostUSBDeviceSubState_Default:
                            return "AttachingToVM";
                        case kHostUSBDeviceSubState_AwaitingDetach:
                            return "AttachingToVM[Detach]";
                        case kHostUSBDeviceSubState_AwaitingReAttach:
                            return "AttachingToVM[Attach]";
                        default:
                            AssertFailedReturn("AttachingToVM[bad]");
                    }
                    break;

                default:
                    AssertFailedReturn("AttachingToVM{bad}");
            }
            break;


        case kHostUSBDeviceState_PhysDetachingFromVM:
            switch (aPendingState)
            {
                case kHostUSBDeviceState_PhysDetached:
                    switch (aSubState)
                    {
                        case kHostUSBDeviceSubState_Default:
                            return "PhysDetachingFromVM";
                        default:
                            AssertFailedReturn("AttachingToVM[bad]");
                    }
                    break;

                default:
                    AssertFailedReturn("AttachingToVM{bad}");
            }
            break;

        default:
            AssertFailedReturn("BadState");

    }

    AssertFailedReturn("shouldn't get here");
}

/**
 * Set the device state.
 *
 * This method will verify that the state transition is a legal one
 * according to the statemachine. It will also take care of the
 * associated house keeping and determine if filters needs to be applied.
 *
 * @param   aNewState           The new state.
 * @param   aNewPendingState    The final state of a transition when applicable.
 * @param   aNewSubState        The new sub-state when applicable.
 *
 * @returns true if filters should be applied to the device, false if not.
 *
 * @note    The caller must own the write lock for this object.
 */
bool HostUSBDevice::setState(HostUSBDeviceState aNewState, HostUSBDeviceState aNewPendingState /*= kHostUSBDeviceState_Invalid*/,
                             HostUSBDeviceSubState aNewSubState /*= kHostUSBDeviceSubState_Default*/)
{
    Assert(isWriteLockOnCurrentThread());
    Assert(    aNewSubState == kHostUSBDeviceSubState_Default
           ||  aNewSubState == kHostUSBDeviceSubState_AwaitingDetach
           ||  aNewSubState == kHostUSBDeviceSubState_AwaitingReAttach);

    /*
     * If the state is unchanged, then don't bother going
     * thru the validation and setting. This saves a bit of code.
     */
    if (   aNewState == mUniState
        && aNewPendingState == mPendingUniState
        && aNewSubState == mUniSubState)
        return false;

    /*
     * Welcome to the switch orgies!
     * You're welcome to check out the ones in startTransition(),
     * advanceTransition(), failTransition() and getStateName() too. Enjoy!
     */

    bool fFilters = false;
    HostUSBDeviceState NewPrevState = mUniState;
    switch (mUniState)
    {
        /*
         * Not much can be done with a device in this state.
         */
        case kHostUSBDeviceState_Unsupported:
            switch (aNewState)
            {
                case kHostUSBDeviceState_PhysDetached:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;
                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;

        /*
         * Only the host OS (or the user) can make changes
         * that'll make a device get out of this state.
         */
        case kHostUSBDeviceState_UsedByHost:
            switch (aNewState)
            {
                case kHostUSBDeviceState_Capturable:
                case kHostUSBDeviceState_Unused:
                    fFilters = true;
                case kHostUSBDeviceState_PhysDetached:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;
                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;

        /*
         * Now it gets interesting.
         */
        case kHostUSBDeviceState_Capturable:
            switch (aNewState)
            {
                /* Host changes. */
                case kHostUSBDeviceState_Unused:
                    fFilters = true; /* Wildcard only... */
                case kHostUSBDeviceState_UsedByHost:
                case kHostUSBDeviceState_PhysDetached:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;

                /* VBox actions */
                case kHostUSBDeviceState_Capturing:
                    switch (aNewPendingState)
                    {
                        case kHostUSBDeviceState_HeldByProxy:
                        case kHostUSBDeviceState_UsedByVM:
                            break;
                        default:
                            AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                         stateName(aNewState, aNewPendingState, aNewSubState)), false);
                    }
                    break;
                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;

        case kHostUSBDeviceState_Unused:
            switch (aNewState)
            {
                /* Host changes. */
                case kHostUSBDeviceState_PhysDetached:
                case kHostUSBDeviceState_UsedByHost:
                case kHostUSBDeviceState_Capturable:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;

                /* VBox actions */
                case kHostUSBDeviceState_Capturing:
                    switch (aNewPendingState)
                    {
                        case kHostUSBDeviceState_HeldByProxy:
                        case kHostUSBDeviceState_UsedByVM:
                            break;
                        default:
                            AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                         stateName(aNewState, aNewPendingState, aNewSubState)), false);
                    }
                    break;
                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;

        /*
         * VBox owns this device now, what's next...
         */
        case kHostUSBDeviceState_HeldByProxy:
            switch (aNewState)
            {
                /* Host changes. */
                case kHostUSBDeviceState_PhysDetached:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;

                /* VBox actions */
                case kHostUSBDeviceState_AttachingToVM:
                    switch (aNewPendingState)
                    {
                        case kHostUSBDeviceState_UsedByVM:
                            break;
                        default:
                            AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                         stateName(aNewState, aNewPendingState, aNewSubState)), false);
                    }
                    break;
                case kHostUSBDeviceState_ReleasingToHost:
                    switch (aNewPendingState)
                    {
                        case kHostUSBDeviceState_Unused: /* Only this! */
                            break;
                        default:
                            AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                         stateName(aNewState, aNewPendingState, aNewSubState)), false);
                    }
                    break;
                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;


        case kHostUSBDeviceState_UsedByVM:
            switch (aNewState)
            {
                /* Host changes. */
                case kHostUSBDeviceState_PhysDetachingFromVM:
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    Assert(aNewPendingState == kHostUSBDeviceState_PhysDetached);
                    break;

                /* VBox actions */
                case kHostUSBDeviceState_DetachingFromVM:
                    switch (aNewPendingState)
                    {
                        case kHostUSBDeviceState_HeldByProxy:
                        case kHostUSBDeviceState_Unused: /* Only this! */
                            break;
                        default:
                            AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                         stateName(aNewState, aNewPendingState, aNewSubState)), false);
                    }
                    break;
                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;

        /*
         * The final state.
         */
        case kHostUSBDeviceState_PhysDetached:
            switch (mUniState)
            {
                case kHostUSBDeviceState_Unsupported:
                case kHostUSBDeviceState_UsedByHost:
                case kHostUSBDeviceState_Capturable:
                case kHostUSBDeviceState_Unused:
                case kHostUSBDeviceState_HeldByProxy:
                case kHostUSBDeviceState_PhysDetachingFromVM:
                case kHostUSBDeviceState_DetachingFromVM: // ??
                case kHostUSBDeviceState_Capturing:
                case kHostUSBDeviceState_ReleasingToHost:
                    break;

                case kHostUSBDeviceState_AttachingToVM: // ??
                case kHostUSBDeviceState_UsedByVM:
                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;


        /*
         * The transitional states.
         */
        case kHostUSBDeviceState_Capturing:
            NewPrevState = mPrevUniState;
            switch (aNewState)
            {
                /* Sub state advance. */
                case kHostUSBDeviceState_Capturing:
                    switch (aNewSubState)
                    {
                        case kHostUSBDeviceSubState_AwaitingReAttach:
                            Assert(mUniSubState == kHostUSBDeviceSubState_AwaitingDetach);
                            Assert(aNewPendingState == mPendingUniState);
                            break;
                        default:
                            AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);
                    }
                    break;

                /* Host/User/Failure. */
                case kHostUSBDeviceState_PhysDetached:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;
                case kHostUSBDeviceState_UsedByHost:
                case kHostUSBDeviceState_Capturable:
                case kHostUSBDeviceState_Unused:
                    Assert(aNewState == mPrevUniState);
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;

                /* VBox */
                case kHostUSBDeviceState_HeldByProxy:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    Assert(   mPendingUniState == kHostUSBDeviceState_HeldByProxy
                           || mPendingUniState == kHostUSBDeviceState_UsedByVM /* <- failure */ );
                    break;
                case kHostUSBDeviceState_AttachingToVM:
                    Assert(aNewPendingState == kHostUSBDeviceState_UsedByVM);
                    NewPrevState = kHostUSBDeviceState_HeldByProxy;
                    break;

                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;

        case kHostUSBDeviceState_ReleasingToHost:
            Assert(mPrevUniState == kHostUSBDeviceState_HeldByProxy);
            NewPrevState = mPrevUniState;
            switch (aNewState)
            {
                /* Sub state advance. */
                case kHostUSBDeviceState_ReleasingToHost:
                    switch (aNewSubState)
                    {
                        case kHostUSBDeviceSubState_AwaitingReAttach:
                            Assert(mUniSubState == kHostUSBDeviceSubState_AwaitingDetach);
                            Assert(aNewPendingState == mPendingUniState);
                            break;
                        default:
                            AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);
                    }
                    break;

                /* Host/Failure. */
                case kHostUSBDeviceState_PhysDetached:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;
                case kHostUSBDeviceState_HeldByProxy:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    Assert(mPendingUniState == kHostUSBDeviceState_Unused);
                    break;

                /* Success */
                case kHostUSBDeviceState_UsedByHost:
                case kHostUSBDeviceState_Capturable:
                case kHostUSBDeviceState_Unused:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    Assert(mPendingUniState == kHostUSBDeviceState_Unused);
                    break;

                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;

        case kHostUSBDeviceState_AttachingToVM:
            Assert(mPrevUniState == kHostUSBDeviceState_HeldByProxy);
            NewPrevState = mPrevUniState;
            switch (aNewState)
            {
                /* Host/Failure. */
                case kHostUSBDeviceState_PhysDetachingFromVM:
                    Assert(aNewPendingState == kHostUSBDeviceState_PhysDetached);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;
                case kHostUSBDeviceState_HeldByProxy:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    Assert(mPendingUniState == kHostUSBDeviceState_Unused);
                    break;

                /* Success */
                case kHostUSBDeviceState_UsedByVM:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    Assert(mPendingUniState == kHostUSBDeviceState_UsedByVM);
                    break;

                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;

        case kHostUSBDeviceState_DetachingFromVM:
            Assert(mPrevUniState == kHostUSBDeviceState_UsedByVM);
            NewPrevState = mPrevUniState;
            switch (aNewState)
            {
                /* Host/Failure. */
                case kHostUSBDeviceState_PhysDetached: //??
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;
                case kHostUSBDeviceState_PhysDetachingFromVM:
                    Assert(aNewPendingState == kHostUSBDeviceState_PhysDetached);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;

                /* Success */
                case kHostUSBDeviceState_HeldByProxy:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    Assert(mPendingUniState == kHostUSBDeviceState_HeldByProxy);
                    fFilters = true;
                    break;

                case kHostUSBDeviceState_ReleasingToHost:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    Assert(mPendingUniState == kHostUSBDeviceState_Unused);
                    NewPrevState = kHostUSBDeviceState_HeldByProxy;
                    break;

                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;

        case kHostUSBDeviceState_PhysDetachingFromVM:
            Assert(   mPrevUniState == kHostUSBDeviceState_DetachingFromVM
                   || mPrevUniState == kHostUSBDeviceState_AttachingToVM
                   || mPrevUniState == kHostUSBDeviceState_UsedByVM);
            NewPrevState = mPrevUniState; /* preserving it is more useful. */
            switch (aNewState)
            {
                case kHostUSBDeviceState_PhysDetached:
                    Assert(aNewPendingState == kHostUSBDeviceState_Invalid);
                    Assert(aNewSubState == kHostUSBDeviceSubState_Default);
                    break;
                default:
                    AssertLogRelMsgFailedReturn(("this=%p %s -X-> %s\n", this, getStateName(),
                                                 stateName(aNewState, aNewPendingState, aNewSubState)), false);
            }
            break;

        default:
            AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);
    }

    /*
     * Make the state change.
     */
    if (NewPrevState != mPrevUniState)
        LogFlowThisFunc(("%s -> %s (prev: %s -> %s) [%s]\n",
                         getStateName(), stateName(aNewState, aNewPendingState, aNewSubState),
                         stateName(mPrevUniState), stateName(NewPrevState), mName));
    else
        LogFlowThisFunc(("%s -> %s (prev: %s) [%s]\n",
                         getStateName(), stateName(aNewState, aNewPendingState, aNewSubState), stateName(NewPrevState), mName));
    mPrevUniState = NewPrevState;
    mUniState = aNewState;
    mUniSubState = aNewSubState;
    mPendingUniState = aNewPendingState;
    mLastStateChangeTS = RTTimeNanoTS();

    return fFilters;
}


/**
 * A convenience for entering a transitional state.

 * @param   aNewState       The new state (transitional).
 * @param   aFinalSubState  The final state of the transition (non-transitional).
 * @param   aNewSubState    The new sub-state when applicable.
 *
 * @returns Always false because filters are never applied for the start of a transition.
 *
 * @note    The caller must own the write lock for this object.
 */
bool HostUSBDevice::startTransition(HostUSBDeviceState aNewState, HostUSBDeviceState aFinalState,
                                    HostUSBDeviceSubState aNewSubState /*= kHostUSBDeviceSubState_Default*/)
{
    AssertReturn(isWriteLockOnCurrentThread(), false);
    /*
     * A quick prevalidation thing. Not really necessary since setState
     * verifies this too, but it's very easy here.
     */
    switch (mUniState)
    {
        case kHostUSBDeviceState_Unsupported:
        case kHostUSBDeviceState_UsedByHost:
        case kHostUSBDeviceState_Capturable:
        case kHostUSBDeviceState_Unused:
        case kHostUSBDeviceState_HeldByProxy:
        case kHostUSBDeviceState_UsedByVM:
            break;

        case kHostUSBDeviceState_DetachingFromVM:
        case kHostUSBDeviceState_Capturing:
        case kHostUSBDeviceState_ReleasingToHost:
        case kHostUSBDeviceState_AttachingToVM:
        case kHostUSBDeviceState_PhysDetachingFromVM:
            AssertMsgFailedReturn(("this=%p %s is a transitional state.\n", this, getStateName()), false);

        case kHostUSBDeviceState_PhysDetached:
        default:
            AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);
    }

    return setState(aNewState, aFinalState, aNewSubState);
}


/**
 * A convenience for advancing a transitional state forward.
 *
 * @param   aSkipReAttach   Fast forwards thru the re-attach substate if
 *                          applicable.
 *
 * @returns true if filters should be applied to the device, false if not.
 *
 * @note    The caller must own the write lock for this object.
 */
bool HostUSBDevice::advanceTransition(bool aSkipReAttach /* = false */)
{
    AssertReturn(isWriteLockOnCurrentThread(), false);
    HostUSBDeviceState enmPending = mPendingUniState;
    HostUSBDeviceSubState enmSub = mUniSubState;
    HostUSBDeviceState enmState = mUniState;
    switch (enmState)
    {
        case kHostUSBDeviceState_Capturing:
            switch (enmSub)
            {
                case kHostUSBDeviceSubState_AwaitingDetach:
                    enmSub = kHostUSBDeviceSubState_AwaitingReAttach;
                    break;
                case kHostUSBDeviceSubState_AwaitingReAttach:
                    enmSub = kHostUSBDeviceSubState_Default;
                    /* fall thru */
                case kHostUSBDeviceSubState_Default:
                    switch (enmPending)
                    {
                        case kHostUSBDeviceState_UsedByVM:
                            enmState = kHostUSBDeviceState_AttachingToVM;
                            break;
                        case kHostUSBDeviceState_HeldByProxy:
                            enmState = enmPending;
                            enmPending = kHostUSBDeviceState_Invalid;
                            break;
                        default:
                            AssertMsgFailedReturn(("this=%p invalid pending state %d: %s\n", this, enmPending, getStateName()), false);
                    }
                    break;
                default:
                    AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);
            }
            break;

        case kHostUSBDeviceState_ReleasingToHost:
            switch (enmSub)
            {
                case kHostUSBDeviceSubState_AwaitingDetach:
                    enmSub = kHostUSBDeviceSubState_AwaitingReAttach;
                    break;
                case kHostUSBDeviceSubState_AwaitingReAttach:
                    enmSub = kHostUSBDeviceSubState_Default;
                    /* fall thru */
                case kHostUSBDeviceSubState_Default:
                    switch (enmPending)
                    {
                        /* Use Unused here since it implies that filters has been applied
                           and will make sure they aren't applied if the final state really
                           is Capturable. */
                        case kHostUSBDeviceState_Unused:
                            enmState = enmPending;
                            enmPending = kHostUSBDeviceState_Invalid;
                            break;
                        default:
                            AssertMsgFailedReturn(("this=%p invalid pending state %d: %s\n", this, enmPending, getStateName()), false);
                    }
                    break;
                default:
                    AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);
            }
            break;

        case kHostUSBDeviceState_AttachingToVM:
            switch (enmSub)
            {
                case kHostUSBDeviceSubState_AwaitingDetach:
                    enmSub = kHostUSBDeviceSubState_AwaitingReAttach;
                    break;
                case kHostUSBDeviceSubState_AwaitingReAttach:
                    enmSub = kHostUSBDeviceSubState_Default;
                    /* fall thru */
                case kHostUSBDeviceSubState_Default:
                    switch (enmPending)
                    {
                        case kHostUSBDeviceState_UsedByVM:
                            enmState = enmPending;
                            enmPending = kHostUSBDeviceState_Invalid;
                            break;
                        default:
                            AssertMsgFailedReturn(("this=%p invalid pending state %d: %s\n", this, enmPending, getStateName()), false);
                    }
                    break;
                default:
                    AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);
            }
            break;

        case kHostUSBDeviceState_DetachingFromVM:
            switch (enmSub)
            {
                case kHostUSBDeviceSubState_AwaitingDetach:
                    enmSub = kHostUSBDeviceSubState_AwaitingReAttach;
                    break;
                case kHostUSBDeviceSubState_AwaitingReAttach:
                    enmSub = kHostUSBDeviceSubState_Default;
                    /* fall thru */
                case kHostUSBDeviceSubState_Default:
                    switch (enmPending)
                    {
                        case kHostUSBDeviceState_HeldByProxy:
                            enmState = enmPending;
                            enmPending = kHostUSBDeviceState_Invalid;
                            break;
                        case kHostUSBDeviceState_Unused:
                            enmState = kHostUSBDeviceState_ReleasingToHost;
                            break;
                        default:
                            AssertMsgFailedReturn(("this=%p invalid pending state %d: %s\n", this, enmPending, getStateName()), false);
                    }
                    break;
                default:
                    AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);
            }
            break;

        case kHostUSBDeviceState_PhysDetachingFromVM:
            switch (enmSub)
            {
                case kHostUSBDeviceSubState_Default:
                    switch (enmPending)
                    {
                        case kHostUSBDeviceState_PhysDetached:
                            enmState = enmPending;
                            enmPending = kHostUSBDeviceState_Invalid;
                            break;
                        default:
                            AssertMsgFailedReturn(("this=%p invalid pending state %d: %s\n", this, enmPending, getStateName()), false);
                    }
                    break;
                default:
                    AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);
            }
            break;

        case kHostUSBDeviceState_Unsupported:
        case kHostUSBDeviceState_UsedByHost:
        case kHostUSBDeviceState_Capturable:
        case kHostUSBDeviceState_Unused:
        case kHostUSBDeviceState_HeldByProxy:
        case kHostUSBDeviceState_UsedByVM:
            AssertMsgFailedReturn(("this=%p %s is not transitional\n", this, getStateName()), false);
        case kHostUSBDeviceState_PhysDetached:
        default:
            AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, enmState), false);

    }

    bool fRc = setState(enmState, enmPending, enmSub);
    if (aSkipReAttach && mUniSubState == kHostUSBDeviceSubState_AwaitingReAttach)
        fRc |= advanceTransition(false /* don't fast forward re-attach */);
    return fRc;
}

/**
 * A convenience for failing a transitional state.
 *
 * @return true if filters should be applied to the device, false if not.
 *
 * @note    The caller must own the write lock for this object.
 */
bool HostUSBDevice::failTransition()
{
    AssertReturn(isWriteLockOnCurrentThread(), false);
    HostUSBDeviceSubState enmSub = mUniSubState;
    HostUSBDeviceState enmState = mUniState;
    switch (enmState)
    {
        /*
         * There are just two cases, either we got back to the
         * previous state (assumes Capture+Attach-To-VM updates it)
         * or we assume the device has been unplugged (physically).
         */
        case kHostUSBDeviceState_DetachingFromVM:
        case kHostUSBDeviceState_Capturing:
        case kHostUSBDeviceState_ReleasingToHost:
        case kHostUSBDeviceState_AttachingToVM:
            switch (enmSub)
            {
                case kHostUSBDeviceSubState_AwaitingDetach:
                    enmSub = kHostUSBDeviceSubState_Default;
                    /* fall thru */
                case kHostUSBDeviceSubState_Default:
                    enmState = mPrevUniState;
                    break;
                case kHostUSBDeviceSubState_AwaitingReAttach:
                    enmSub = kHostUSBDeviceSubState_Default;
                    enmState = kHostUSBDeviceState_PhysDetached;
                    break;
                default:
                    AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);
            }
            break;

        case kHostUSBDeviceState_PhysDetachingFromVM:
            AssertMsgFailedReturn(("this=%p %s shall not fail\n", this, getStateName()), false);

        case kHostUSBDeviceState_Unsupported:
        case kHostUSBDeviceState_UsedByHost:
        case kHostUSBDeviceState_Capturable:
        case kHostUSBDeviceState_Unused:
        case kHostUSBDeviceState_HeldByProxy:
        case kHostUSBDeviceState_UsedByVM:
            AssertMsgFailedReturn(("this=%p %s is not transitional\n", this, getStateName()), false);
        case kHostUSBDeviceState_PhysDetached:
        default:
            AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), false);

    }

    return setState(enmState, kHostUSBDeviceState_Invalid, enmSub);
}


/**
 * Determines the canonical state of the device.
 *
 * @returns canonical state.
 *
 * @note    The caller must own the read (or write) lock for this object.
 */
USBDeviceState_T HostUSBDevice::canonicalState() const
{
    switch (mUniState)
    {
        /*
         * Straight forward.
         */
        case kHostUSBDeviceState_Unsupported:
            return USBDeviceState_NotSupported;

        case kHostUSBDeviceState_UsedByHost:
            return USBDeviceState_Unavailable;

        case kHostUSBDeviceState_Capturable:
            return USBDeviceState_Busy;

        case kHostUSBDeviceState_Unused:
            return USBDeviceState_Available;

        case kHostUSBDeviceState_HeldByProxy:
            return USBDeviceState_Held;

        case kHostUSBDeviceState_UsedByVM:
            return USBDeviceState_Captured;

        /*
         * Pretend we've reached the final state.
         */
        case kHostUSBDeviceState_Capturing:
            Assert(   mPendingUniState == kHostUSBDeviceState_UsedByVM
                   || mPendingUniState == kHostUSBDeviceState_HeldByProxy);
            return mPendingUniState == kHostUSBDeviceState_UsedByVM
                ? (USBDeviceState_T)USBDeviceState_Captured
                : (USBDeviceState_T)USBDeviceState_Held;
            /* The cast ^^^^ is because xidl is using different enums for
               each of the values. *Very* nice idea... :-) */

        case kHostUSBDeviceState_AttachingToVM:
            return USBDeviceState_Captured;

        /*
         * Return the previous state.
         */
        case kHostUSBDeviceState_ReleasingToHost:
            Assert(   mPrevUniState == kHostUSBDeviceState_UsedByVM
                   || mPrevUniState == kHostUSBDeviceState_HeldByProxy);
            return mPrevUniState == kHostUSBDeviceState_UsedByVM
                ? (USBDeviceState_T)USBDeviceState_Captured
                : (USBDeviceState_T)USBDeviceState_Held;
            /* The cast ^^^^ is because xidl is using different enums for
               each of the values. *Very* nice idea... :-) */

        case kHostUSBDeviceState_DetachingFromVM:
            return USBDeviceState_Captured;
        case kHostUSBDeviceState_PhysDetachingFromVM:
            return USBDeviceState_Captured;

        case kHostUSBDeviceState_PhysDetached:
        default:
            AssertReleaseMsgFailedReturn(("this=%p mUniState=%d\n", this, mUniState), USBDeviceState_NotSupported);
    }
    /* won't ever get here. */
}
/* vi: set tabstop=4 shiftwidth=4 expandtab: */
