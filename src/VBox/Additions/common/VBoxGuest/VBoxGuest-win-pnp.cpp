/** @file
 *
 * VBoxGuest-win-pnp - Windows Plug'n'Play specifics.
 *
 * Copyright (C) 2010-2011 Oracle Corporation
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
#include "VBoxGuest-win.h"
#include "VBoxGuestInternal.h"
#include <VBox/err.h>
#include <VBox/log.h>
#include <VBox/version.h>
#include <VBox/VBoxGuestLib.h>


/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
extern winVersion_t g_winVersion;

RT_C_DECLS_BEGIN
static NTSTATUS vboxguestwinSendIrpSynchronously(PDEVICE_OBJECT pDevObj, PIRP pIrp, BOOLEAN fStrict);
static NTSTATUS vboxguestwinPnPIrpComplete(PDEVICE_OBJECT pDevObj, PIRP pIrp, PKEVENT pEvent);
static VOID     vboxguestwinShowDeviceResources(PCM_PARTIAL_RESOURCE_LIST pResourceList);
RT_C_DECLS_END

#ifdef ALLOC_PRAGMA
#pragma alloc_text (PAGE, vboxguestwinPnP)
#pragma alloc_text (PAGE, vboxguestwinPower)
#pragma alloc_text (PAGE, vboxguestwinSendIrpSynchronously)
#pragma alloc_text (PAGE, vboxguestwinShowDeviceResources)
#endif

/* Reenable logging, this was #undef'ed on iprt/log.h for RING0. */
#define LOG_ENABLED


/*******************************************************************************
*   Internal Functions                                                         *
*******************************************************************************/

/**
 * Irp completion routine for PnP Irps we send.
 *
 * @param   pDevObj   Device object.
 * @param   pIrp      Request packet.
 * @param   event     Semaphore.
 * @return   NT status code
 */
static NTSTATUS vboxguestwinPnpIrpComplete(PDEVICE_OBJECT pDevObj, PIRP pIrp, PKEVENT pEvent)
{
    KeSetEvent(pEvent, 0, FALSE);
    return STATUS_MORE_PROCESSING_REQUIRED;
}


/**
 * Helper to send a PnP IRP and wait until it's done.
 *
 * @param    pDevObj    Device object.
 * @param    pIrp       Request packet.
 * @param    fStrict    When set, returns an error if the IRP gives an error.
 * @return   NT status code
 */
static NTSTATUS vboxguestwinSendIrpSynchronously(PDEVICE_OBJECT pDevObj, PIRP pIrp, BOOLEAN fStrict)
{
    KEVENT event;

    KeInitializeEvent(&event, SynchronizationEvent, FALSE);

    IoCopyCurrentIrpStackLocationToNext(pIrp);
    IoSetCompletionRoutine(pIrp, (PIO_COMPLETION_ROUTINE)vboxguestwinPnpIrpComplete,
                           &event, TRUE, TRUE, TRUE);

    NTSTATUS rc = IoCallDriver(pDevObj, pIrp);

    if (rc == STATUS_PENDING)
    {
        KeWaitForSingleObject(&event, Executive, KernelMode, FALSE, NULL);
        rc = pIrp->IoStatus.Status;
    }

    if (!fStrict
        && (rc == STATUS_NOT_SUPPORTED || rc == STATUS_INVALID_DEVICE_REQUEST))
    {
        rc = STATUS_SUCCESS;
    }

    Log(("VBoxGuest::vboxguestwinSendIrpSynchronously: Returning 0x%x\n", rc));
    return rc;
}


/**
 * PnP Request handler.
 *
 * @param  pDevObj    Device object.
 * @param  pIrp       Request packet.
 */
NTSTATUS vboxguestwinPnP(PDEVICE_OBJECT pDevObj, PIRP pIrp)
{
    PVBOXGUESTDEVEXT   pDevExt = (PVBOXGUESTDEVEXT)pDevObj->DeviceExtension;
    PIO_STACK_LOCATION pStack  = IoGetCurrentIrpStackLocation(pIrp);

#ifdef LOG_ENABLED
    static char* aszFnctName[] =
    {
        "IRP_MN_START_DEVICE",
        "IRP_MN_QUERY_REMOVE_DEVICE",
        "IRP_MN_REMOVE_DEVICE",
        "IRP_MN_CANCEL_REMOVE_DEVICE",
        "IRP_MN_STOP_DEVICE",
        "IRP_MN_QUERY_STOP_DEVICE",
        "IRP_MN_CANCEL_STOP_DEVICE",
        "IRP_MN_QUERY_DEVICE_RELATIONS",
        "IRP_MN_QUERY_INTERFACE",
        "IRP_MN_QUERY_CAPABILITIES",
        "IRP_MN_QUERY_RESOURCES",
        "IRP_MN_QUERY_RESOURCE_REQUIREMENTS",
        "IRP_MN_QUERY_DEVICE_TEXT",
        "IRP_MN_FILTER_RESOURCE_REQUIREMENTS",
        "",
        "IRP_MN_READ_CONFIG",
        "IRP_MN_WRITE_CONFIG",
        "IRP_MN_EJECT",
        "IRP_MN_SET_LOCK",
        "IRP_MN_QUERY_ID",
        "IRP_MN_QUERY_PNP_DEVICE_STATE",
        "IRP_MN_QUERY_BUS_INFORMATION",
        "IRP_MN_DEVICE_USAGE_NOTIFICATION",
        "IRP_MN_SURPRISE_REMOVAL",
    };
    Log(("VBoxGuest::vboxguestwinGuestPnp: MinorFunction: %s\n",
           pStack->MinorFunction < (sizeof(aszFnctName) / sizeof(aszFnctName[0]))
         ? aszFnctName[pStack->MinorFunction]
         : "Unknown"));
#endif

    NTSTATUS rc = STATUS_SUCCESS;
    switch (pStack->MinorFunction)
    {
        case IRP_MN_START_DEVICE:
        {
            Log(("VBoxGuest::vboxguestwinVBoxGuestPnP: START_DEVICE\n"));

            /* This must be handled first by the lower driver. */
            rc = vboxguestwinSendIrpSynchronously(pDevExt->win.s.pNextLowerDriver, pIrp, TRUE);

            if (   NT_SUCCESS(rc)
                && NT_SUCCESS(pIrp->IoStatus.Status))
            {
                Log(("VBoxGuest::vboxguestwinVBoxGuestPnP: START_DEVICE: pStack->Parameters.StartDevice.AllocatedResources = %p\n",
                     pStack->Parameters.StartDevice.AllocatedResources));

                if (!pStack->Parameters.StartDevice.AllocatedResources)
                {
                    Log(("VBoxGuest::vboxguestwinVBoxGuestPnP: START_DEVICE: No resources, pDevExt = %p, nextLowerDriver = %p!\n",
                         pDevExt, pDevExt ? pDevExt->win.s.pNextLowerDriver : NULL));
                    rc = STATUS_UNSUCCESSFUL;
                }
                else
                {
                    rc = vboxguestwinInit(pDevObj, pIrp);
                }
            }

            if (NT_ERROR(rc))
            {
                Log(("VBoxGuest::vboxguestwinGuestPnp: START_DEVICE: Error: rc = 0x%x\n", rc));

                /* Need to unmap memory in case of errors ... */
                vboxguestwinUnmapVMMDevMemory(pDevExt);
            }
            break;
        }

        case IRP_MN_CANCEL_REMOVE_DEVICE:
        {
            Log(("VBoxGuest::vboxguestwinVBoxGuestPnP: CANCEL_REMOVE_DEVICE\n"));

            /* This must be handled first by the lower driver. */
            rc = vboxguestwinSendIrpSynchronously(pDevExt->win.s.pNextLowerDriver, pIrp, TRUE);

            if (NT_SUCCESS(rc) && pDevExt->win.s.devState == PENDINGREMOVE)
            {
                /* Return to the state prior to receiving the IRP_MN_QUERY_REMOVE_DEVICE request. */
                pDevExt->win.s.devState = pDevExt->win.s.prevDevState;
            }

            /* Complete the IRP. */
            break;
        }

        case IRP_MN_SURPRISE_REMOVAL:
        {
            Log(("VBoxGuest::vboxguestwinVBoxGuestPnP: IRP_MN_SURPRISE_REMOVAL\n"));

            VBOXGUEST_UPDATE_DEVSTATE(pDevExt, SURPRISEREMOVED);

            /* Do nothing here actually. Cleanup is done in IRP_MN_REMOVE_DEVICE.
             * This request is not expected for VBoxGuest.
             */
            LogRel(("VBoxGuest: unexpected device removal\n"));

            /* Pass to the lower driver. */
            pIrp->IoStatus.Status = STATUS_SUCCESS;

            IoSkipCurrentIrpStackLocation(pIrp);

            rc = IoCallDriver(pDevExt->win.s.pNextLowerDriver, pIrp);

            /* Do not complete the IRP. */
            return rc;
        }

        case IRP_MN_QUERY_REMOVE_DEVICE:
        {
            Log(("VBoxGuest::vboxguestwinVBoxGuestPnP: QUERY_REMOVE_DEVICE\n"));

#ifdef VBOX_REBOOT_ON_UNINSTALL
            Log(("VBoxGuest::vboxguestwinGuestPnp: QUERY_REMOVE_DEVICE: Device cannot be removed without a reboot.\n"));
            rc = STATUS_UNSUCCESSFUL;
#endif /* VBOX_REBOOT_ON_UNINSTALL */

            if (NT_SUCCESS(rc))
            {
                VBOXGUEST_UPDATE_DEVSTATE(pDevExt, PENDINGREMOVE);

                /* This IRP passed down to lower driver. */
                pIrp->IoStatus.Status = STATUS_SUCCESS;

                IoSkipCurrentIrpStackLocation(pIrp);

                rc = IoCallDriver(pDevExt->win.s.pNextLowerDriver, pIrp);
                Log(("VBoxGuest::vboxguestwinGuestPnp: QUERY_REMOVE_DEVICE: Next lower driver replied rc = 0x%x\n", rc));

                /* we must not do anything the IRP after doing IoSkip & CallDriver
                 * since the driver below us will complete (or already have completed) the IRP.
                 * I.e. just return the status we got from IoCallDriver */
                return rc;
            }

            /* Complete the IRP on failure. */
            break;
        }

        case IRP_MN_REMOVE_DEVICE:
        {
            Log(("VBoxGuest::vboxguestwinVBoxGuestPnP: REMOVE_DEVICE\n"));

            VBOXGUEST_UPDATE_DEVSTATE(pDevExt, REMOVED);

            /* Free hardware resources. */
            /* @todo this should actually free I/O ports, interrupts, etc. */
            rc = vboxguestwinCleanup(pDevObj);
            Log(("VBoxGuest::vboxguestwinGuestPnp: REMOVE_DEVICE: vboxguestwinCleanup rc = 0x%08X\n", rc));

            /*
             * We need to send the remove down the stack before we detach,
             * but we don't need to wait for the completion of this operation
             * (and to register a completion routine).
             */
            pIrp->IoStatus.Status = STATUS_SUCCESS;

            IoSkipCurrentIrpStackLocation(pIrp);

            rc = IoCallDriver(pDevExt->win.s.pNextLowerDriver, pIrp);
            Log(("VBoxGuest::vboxguestwinGuestPnp: REMOVE_DEVICE: Next lower driver replied rc = 0x%x\n", rc));

            IoDetachDevice(pDevExt->win.s.pNextLowerDriver);

            Log(("VBoxGuest::vboxguestwinGuestPnp: REMOVE_DEVICE: Removing device ...\n"));

            /* Destroy device extension and clean up everything else. */
            VBoxGuestDeleteDevExt(pDevExt);

            /* Remove DOS device + symbolic link. */
            UNICODE_STRING win32Name;
            RtlInitUnicodeString(&win32Name, VBOXGUEST_DEVICE_NAME_DOS);
            IoDeleteSymbolicLink(&win32Name);

            Log(("VBoxGuest::vboxguestwinGuestPnp: REMOVE_DEVICE: Deleting device ...\n"));

            /* Last action: Delete our device! pDevObj is *not* failed
             * anymore after this call! */
            IoDeleteDevice(pDevObj);

            Log(("VBoxGuest::vboxguestwinGuestPnp: REMOVE_DEVICE: Device removed!\n"));

            /* Propagating rc from IoCallDriver. */
            return rc; /* Make sure that we don't do anything below here anymore! */
        }

        case IRP_MN_CANCEL_STOP_DEVICE:
        {
            Log(("VBoxGuest::vboxguestwinVBoxGuestPnP: CANCEL_STOP_DEVICE\n"));

            /* This must be handled first by the lower driver. */
            rc = vboxguestwinSendIrpSynchronously(pDevExt->win.s.pNextLowerDriver, pIrp, TRUE);

            if (NT_SUCCESS(rc) && pDevExt->win.s.devState == PENDINGSTOP)
            {
                /* Return to the state prior to receiving the IRP_MN_QUERY_STOP_DEVICE request. */
                pDevExt->win.s.devState = pDevExt->win.s.prevDevState;
            }

            /* Complete the IRP. */
            break;
        }

        case IRP_MN_QUERY_STOP_DEVICE:
        {
            Log(("VBoxGuest::vboxguestwinVBoxGuestPnP: QUERY_STOP_DEVICE\n"));

#ifdef VBOX_REBOOT_ON_UNINSTALL
            Log(("VBoxGuest::vboxguestwinGuestPnp: QUERY_STOP_DEVICE: Device cannot be stopped without a reboot!\n"));
            pIrp->IoStatus.Status = STATUS_UNSUCCESSFUL;
#endif /* VBOX_REBOOT_ON_UNINSTALL */

            if (NT_SUCCESS(rc))
            {
                VBOXGUEST_UPDATE_DEVSTATE(pDevExt, PENDINGSTOP);

                /* This IRP passed down to lower driver. */
                pIrp->IoStatus.Status = STATUS_SUCCESS;

                IoSkipCurrentIrpStackLocation(pIrp);

                rc = IoCallDriver(pDevExt->win.s.pNextLowerDriver, pIrp);
                Log(("VBoxGuest::vboxguestwinGuestPnp: QUERY_STOP_DEVICE: Next lower driver replied rc = 0x%x\n", rc));

                /* we must not do anything with the IRP after doing IoSkip & CallDriver
                 * since the driver below us will complete (or already have completed) the IRP.
                 * I.e. just return the status we got from IoCallDriver */
                return rc;
            }

            /* Complete the IRP on failure. */
            break;
        }

        case IRP_MN_STOP_DEVICE:
        {
            Log(("VBoxGuest::vboxguestwinVBoxGuestPnP: STOP_DEVICE\n"));

            VBOXGUEST_UPDATE_DEVSTATE(pDevExt, STOPPED);

            /* Free hardware resources. */
            /* @todo this should actually free I/O ports, interrupts, etc. */
            rc = vboxguestwinCleanup(pDevObj);
            Log(("VBoxGuest::vboxguestwinGuestPnp: STOP_DEVICE: cleaning up, rc = 0x%x\n", rc));

            /* Pass to the lower driver. */
            pIrp->IoStatus.Status = STATUS_SUCCESS;

            IoSkipCurrentIrpStackLocation(pIrp);

            rc = IoCallDriver(pDevExt->win.s.pNextLowerDriver, pIrp);
            Log(("VBoxGuest::vboxguestwinGuestPnp: STOP_DEVICE: Next lower driver replied rc = 0x%x\n", rc));

            return rc;
        }

        default:
        {
            IoSkipCurrentIrpStackLocation(pIrp);
            rc = IoCallDriver(pDevExt->win.s.pNextLowerDriver, pIrp);
            return rc;
        }
    }

    pIrp->IoStatus.Status = rc;
    IoCompleteRequest(pIrp, IO_NO_INCREMENT);

    Log(("VBoxGuest::vboxguestwinGuestPnp: Returning with rc = 0x%x\n", rc));
    return rc;
}


/**
 * Handle the power completion event.
 *
 * @returns NT status code.
 * @param pDevObj   Targetted device object.
 * @param pIrp      IO request packet.
 * @param pContext  Context value passed to IoSetCompletionRoutine in VBoxGuestPower.
 */
NTSTATUS vboxguestwinPowerComplete(IN PDEVICE_OBJECT pDevObj,
                                   IN PIRP pIrp, IN PVOID pContext)
{
    PIO_STACK_LOCATION pIrpSp;
    PVBOXGUESTDEVEXT pDevExt = (PVBOXGUESTDEVEXT)pContext;

    ASSERT(pDevExt);
    ASSERT(pDevExt->signature == DEVICE_EXTENSION_SIGNATURE);

    pIrpSp = IoGetCurrentIrpStackLocation(pIrp);
    if (pIrpSp)
    {
        ASSERT(pIrpSp->MajorFunction == IRP_MJ_POWER);
        if (NT_SUCCESS(pIrp->IoStatus.Status))
        {
            switch (pIrpSp->MinorFunction)
            {
                case IRP_MN_SET_POWER:

                    switch (pIrpSp->Parameters.Power.Type)
                    {
                        case DevicePowerState:
                            switch (pIrpSp->Parameters.Power.State.DeviceState)
                            {
                                case PowerDeviceD0:
                                    break;
                            }
                            break;
                    }
                    break;
            }
        }
    }

    return STATUS_SUCCESS;
}


/**
 * Handle the Power requests.
 *
 * @returns   NT status code
 * @param     pDevObj   device object
 * @param     pIrp      IRP
 */
NTSTATUS vboxguestwinPower(PDEVICE_OBJECT pDevObj, PIRP pIrp)
{
    PIO_STACK_LOCATION pStack = IoGetCurrentIrpStackLocation(pIrp);
    PVBOXGUESTDEVEXT   pDevExt = (PVBOXGUESTDEVEXT)pDevObj->DeviceExtension;
    POWER_STATE_TYPE   powerType;
    POWER_STATE        powerState;
    POWER_ACTION       powerAction;

    Log(("VBoxGuest::vboxguestwinGuestPower\n"));

    powerType   = pStack->Parameters.Power.Type;
    powerAction = pStack->Parameters.Power.ShutdownType;
    powerState  = pStack->Parameters.Power.State;

    switch (pStack->MinorFunction)
    {
        case IRP_MN_SET_POWER:
        {
            Log(("VBoxGuest::vboxguestwinGuestPower: IRP_MN_SET_POWER, type= %d\n", powerType));
            switch (powerType)
            {
                case SystemPowerState:
                {
                    Log(("VBoxGuest::vboxguestwinGuestPower: SystemPowerState, action = %d, state = %d\n", powerAction, powerState));

                    switch (powerAction)
                    {
                        case PowerActionSleep:

                            /* System now is in a working state. */
                            if (powerState.SystemState == PowerSystemWorking)
                            {
                                if (   pDevExt
                                    && pDevExt->win.s.LastSystemPowerAction == PowerActionHibernate)
                                {
                                    Log(("VBoxGuest::vboxguestwinGuestPower: Returning from hibernation!\n"));
                                    int rc = VBoxGuestReinitDevExtAfterHibernation(pDevExt, vboxguestwinVersionToOSType(g_winVersion));
                                    if (RT_FAILURE(rc))
                                        Log(("VBoxGuest::vboxguestwinGuestPower: Cannot re-init VMMDev chain, rc = %d!\n", rc));
                                }
                            }
                            break;

                        case PowerActionShutdownReset:
                        {
                            Log(("VBoxGuest::vboxguestwinGuestPower: Power action reset!\n"));

                            /* Tell the VMM that we no longer support mouse pointer integration. */
                            VMMDevReqMouseStatus *pReq = NULL;
                            int vrc = VbglGRAlloc((VMMDevRequestHeader **)&pReq, sizeof (VMMDevReqMouseStatus), VMMDevReq_SetMouseStatus);
                            if (RT_SUCCESS(vrc))
                            {
                                pReq->mouseFeatures = 0;
                                pReq->pointerXPos = 0;
                                pReq->pointerYPos = 0;

                                vrc = VbglGRPerform(&pReq->header);
                                if (RT_FAILURE(vrc))
                                {
                                    Log(("VBoxGuest::PowerStateRequest: error communicating new power status to VMMDev. "
                                             "vrc = %Rrc\n", vrc));
                                }

                                VbglGRFree(&pReq->header);
                            }

                            /* Don't do any cleanup here; there might be still coming in some IOCtls after we got this
                             * power action and would assert/crash when we already cleaned up all the stuff! */
                            break;
                        }

                        case PowerActionShutdown:
                        case PowerActionShutdownOff:
                        {
                            Log(("VBoxGuest::vboxguestwinGuestPower: Power action shutdown!\n"));
                            if (powerState.SystemState >= PowerSystemShutdown)
                            {
                                Log(("VBoxGuest::vboxguestwinGuestPower: Telling the VMMDev to close the VM ...\n"));

                                VMMDevPowerStateRequest *pReq = pDevExt->win.s.pPowerStateRequest;
                                int vrc = VERR_NOT_IMPLEMENTED;
                                if (pReq)
                                {
                                    pReq->header.requestType = VMMDevReq_SetPowerStatus;
                                    pReq->powerState = VMMDevPowerState_PowerOff;

                                    vrc = VbglGRPerform(&pReq->header);
                                }
                                if (RT_FAILURE(vrc))
                                {
                                    Log(("VBoxGuest::PowerStateRequest: Error communicating new power status to VMMDev. "
                                             "vrc = %Rrc\n", vrc));
                                }

                                /* No need to do cleanup here; at this point we should've been
                                 * turned off by VMMDev already! */
                            }
                            break;
                        }

                        case PowerActionHibernate:

                            Log(("VBoxGuest::vboxguestwinGuestPower: Power action hibernate!\n"));
                            break;
                    }

                    /*
                     * Save the current system power action for later use.
                     * This becomes handy when we return from hibernation for example.
                     */
                    if (pDevExt)
                        pDevExt->win.s.LastSystemPowerAction = powerAction;

                    break;
                }
                default:
                    break;
            }
            break;
        }
        default:
            break;
    }

    /*
     *  Whether we are completing or relaying this power IRP,
     *  we must call PoStartNextPowerIrp.
     */
    PoStartNextPowerIrp(pIrp);

    /*
     *  Send the IRP down the driver stack,
     *  using PoCallDriver (not IoCallDriver, as for non-power irps).
     */
    IoCopyCurrentIrpStackLocationToNext(pIrp);
    IoSetCompletionRoutine(pIrp,
                           vboxguestwinPowerComplete,
                           (PVOID)pDevExt,
                           TRUE,
                           TRUE,
                           TRUE);
    return PoCallDriver(pDevExt->win.s.pNextLowerDriver, pIrp);
}
