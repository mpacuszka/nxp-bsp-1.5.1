/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2022 NXP
   Licensed under the MIT License.

Abstract:
    This module implements the power framework (PoFx) in PWM driver to manage the power 
    state of the component that represents device.

*/

#include <ntddk.h>
#include <wdf.h>

#include "imxi2cinternal.h"
#include "imxi2cpofx.h"

NTSTATUS
PowerManagementSetup(
    _Inout_ WDFDEVICE device
    )
/*++
Routine Description:

    PowerManagementSetup is called by the KMDF from AddDevice to setup S0IdleSettings
    and power management component states. 

Arguments:

    device - the WDF device

Return Value:

    An NTSTATUS value representing success or failure of the function.

--*/
{
    NTSTATUS status;    
    DEVICE_CONTEXT*  deviceContextPtr = NULL;
    
    PAGED_CODE();

    deviceContextPtr = GetDeviceContext(device);

    //
    // Our initial state is active
    //
    deviceContextPtr->IsActive = TRUE;

    status = AssignS0IdleSettings(device);
    if (!NT_SUCCESS(status)) {
        goto exit;
    }

    //
    // If you need to talk to hardware to figure out what F-states are 
    // applicable this can be done in EvtSelfManagedIoInit 
    // (but no later than that). EvtSelfManagedIoInit gets invoked after
    // EvtPrepareHardware so you'd have chance to initialize your hardware.
    //
    status = AssignPowerFrameworkSettings(device);
    if (!NT_SUCCESS(status)) {
        goto exit;
    }

    // WakeUp related settings
    {
        WDF_DEVICE_POWER_POLICY_WAKE_SETTINGS wakeSettings;
        WDF_DEVICE_POWER_POLICY_WAKE_SETTINGS_INIT(&wakeSettings);
        wakeSettings.Enabled = WdfTrue;
        status = WdfDeviceAssignSxWakeSettings(device, &wakeSettings);
        if (!NT_SUCCESS(status)) {
            //
            // We are probably enumerated on a bus that doesn't support Sx-wake.
            // Let us not fail the device add just because we aren't able to support
            // wait-wake. I will let the user of this sample decide how important it's
            // to support wait-wake for their hardware and return appropriate status.
            //
            /*TraceEvents(TRACE_LEVEL_WARNING, TRACE_DRIVER,
                "Failed in WdfDeviceAssignSxWakeSettings() for WDFDEVICE %p",
                device);*/
            status = STATUS_SUCCESS;
        }

    }

exit:
    return status;
}

void
PowerManagementSetupWakeUpCallbacks(
    _In_ PWDFDEVICE_INIT DeviceInit
)
/*++
Routine Description:

    Registers callbacks related to device WakeUp.

Arguments:

    DeviceInit - information about the PDO that we are loading on
--*/
{
    WDF_POWER_POLICY_EVENT_CALLBACKS powerPolicyCallbacks;
    // Register power policy event callbacks so that we would know when to
    // arm/disarm the hardware to handle wait-wake and when the wake event
    // is triggered by the hardware.
    //
    WDF_POWER_POLICY_EVENT_CALLBACKS_INIT(&powerPolicyCallbacks);

    //
    // This group of three callbacks allows this sample driver to manage
    // arming the device for wake from the S0 or Sx state.  We don't really
    // differentiate between S0 and Sx state..
    //
    powerPolicyCallbacks.EvtDeviceArmWakeFromS0 = SingleCompEvtDeviceArmWakeFromS0;
    powerPolicyCallbacks.EvtDeviceDisarmWakeFromS0 = SingleCompEvtDeviceDisarmWakeFromS0;
    powerPolicyCallbacks.EvtDeviceWakeFromS0Triggered = SingleCompEvtDeviceWakeFromS0Triggered;
    powerPolicyCallbacks.EvtDeviceArmWakeFromSx = SingleCompEvtDeviceArmWakeFromSx;
    powerPolicyCallbacks.EvtDeviceDisarmWakeFromSx = SingleCompEvtDeviceDisarmWakeFromSx;
    powerPolicyCallbacks.EvtDeviceWakeFromSxTriggered = SingleCompEvtDeviceWakeFromSxTriggered;

    //
    // Register the power policy callbacks.
    //
    WdfDeviceInitSetPowerPolicyEventCallbacks(DeviceInit, &powerPolicyCallbacks);
}

NTSTATUS
SingleCompWdmEvtDeviceWdmPostPoFxRegisterDevice(
    _In_ WDFDEVICE Device,
    _In_ POHANDLE PoHandle
    )
/*++
Routine Description:

    KMDF calls this routine after it has registered with the Power Framework
    and supplies the registration handle that driver can use directly.

Arguments:

    Device - Handle to the framework device object

    PoHandle - Handle of registration with Power Framework.

Return Value:

    An NTSTATUS value representing success or failure of the function.
    
--*/
{
    DEVICE_CONTEXT* deviceContextPtr = NULL;

    PAGED_CODE();
    
    //
    // Get the device context
    //
    deviceContextPtr = GetDeviceContext(Device);

    //
    // Save the POHANDLE
    //
    deviceContextPtr->PoHandle = PoHandle;

    //
    // Set latency and residency hints so that the power framework chooses lower
    // powered F-states when we are idle.
    // The values used here are for illustration purposes only. The driver 
    // should use values that are appropriate for its device.
    //
    PoFxSetComponentLatency(
        PoHandle,
        0, // Component
        (WDF_ABS_TIMEOUT_IN_MS(DEEPEST_FSTATE_LATENCY_IN_MS) + 1)
        );
    PoFxSetComponentResidency(
        PoHandle,
        0, // Component
        (WDF_ABS_TIMEOUT_IN_SEC(DEEPEST_FSTATE_RESIDENCY_IN_SEC) + 1)
        );

    return STATUS_SUCCESS;
}

VOID
SingleCompWdmEvtDeviceWdmPrePoFxUnregisterDevice(
    _In_ WDFDEVICE Device,
    _In_ POHANDLE PoHandle
    )
/*++
Routine Description:

    KMDF calls this routine when it is about to unregister with the Power
    Framework. After returning from this routine driver must not use the
    supplied registration handle anymore.

Arguments:

    Device - Handle to the framework device object

    PoHandle - Handle of registration with Power Framework.

Return Value:

    An NTSTATUS value representing success or failure of the function.
    
--*/
{
    DEVICE_CONTEXT* deviceContextPtr = NULL;

    PAGED_CODE();
    
    UNREFERENCED_PARAMETER(PoHandle);

    //
    // Get the device context
    //
    deviceContextPtr = GetDeviceContext(Device);

    //
    // Reset the POHANDLE
    //
    deviceContextPtr->PoHandle = NULL;

    return;
}

_IRQL_requires_same_
_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS 
AssignS0IdleSettings(
    _In_ WDFDEVICE Device
    )
/*++
Routine Description:

    Helper function to assign S0 idle settings for the device

Arguments:

    Device - Handle to the framework device object

Return Value:

    An NTSTATUS value representing success or failure of the function.
    
--*/
{
    NTSTATUS status;
    WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS powerPolicy;

    PAGED_CODE();

    WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS_INIT(&powerPolicy, 
                                               IdleCannotWakeFromS0);
    powerPolicy.IdleTimeoutType = SystemManagedIdleTimeout;
    powerPolicy.IdleTimeout = IDLE_TIMEOUT_MONITOR_ON;

    status = WdfDeviceAssignS0IdleSettings(Device, &powerPolicy);
    if (FALSE == NT_SUCCESS(status)) {
        /*Trace(TRACE_LEVEL_ERROR,
              "%!FUNC! - WdfDeviceAssignS0IdleSettings failed with %!status!.", 
              status);*/
    }
    return status;
}

_IRQL_requires_same_
_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS 
AssignPowerFrameworkSettings(
    _In_ WDFDEVICE Device
    )
/*++
Routine Description:

    Helper function to assign Power Framework related settings for the device

Arguments:

    Device - Handle to the framework device object

Return Value:

    An NTSTATUS value representing success or failure of the function.
    
--*/
{
    NTSTATUS status;
    WDF_POWER_FRAMEWORK_SETTINGS poFxSettings;
    PO_FX_COMPONENT component;
    PO_FX_COMPONENT_IDLE_STATE idleStates[FSTATE_COUNT];
    
    //
    // Note that we initialize the 'idleStates' array below based on the
    // assumption that MAX_FSTATE_COUNT is 4.
    // If we increase the value of MAX_FSTATE_COUNT, we need to initialize those
    // additional F-states below. If we decrease the value of MAX_FSTATE_COUNT,
    // we need to remove the corresponding initializations below.
    //
    C_ASSERT(FSTATE_COUNT == 2);
    
    PAGED_CODE();
    
    //
    // Initialization
    //
    RtlZeroMemory(&component, sizeof(component));
    RtlZeroMemory(idleStates, sizeof(idleStates));

    //
    // The transition latency and residency requirement values used here are for
    // illustration purposes only. The driver should use values that are 
    // appropriate for its device.
    //

    //
    // F0
    //
    idleStates[0].TransitionLatency = 0;    
    idleStates[0].ResidencyRequirement = 0;    
    idleStates[0].NominalPower = 0;    

    //
    // F1
    //
    idleStates[1].TransitionLatency = 
        WDF_ABS_TIMEOUT_IN_MS(DEEPEST_FSTATE_LATENCY_IN_MS); 
    idleStates[1].ResidencyRequirement = 
        WDF_ABS_TIMEOUT_IN_SEC(DEEPEST_FSTATE_RESIDENCY_IN_SEC);
    idleStates[1].NominalPower = 0;

    //
    // Component 0 (the only component)
    //
    component.IdleStateCount = FSTATE_COUNT;
    component.IdleStates = idleStates;

    WDF_POWER_FRAMEWORK_SETTINGS_INIT(&poFxSettings);

    poFxSettings.EvtDeviceWdmPostPoFxRegisterDevice = 
                        SingleCompWdmEvtDeviceWdmPostPoFxRegisterDevice;
    poFxSettings.EvtDeviceWdmPrePoFxUnregisterDevice =
                        SingleCompWdmEvtDeviceWdmPrePoFxUnregisterDevice;

    poFxSettings.Component = &component;
    poFxSettings.ComponentActiveConditionCallback = 
                        SingleCompWdmActiveConditionCallback;
    poFxSettings.ComponentIdleConditionCallback = 
                        SingleCompWdmIdleConditionCallback;
    poFxSettings.ComponentIdleStateCallback = 
                        SingleCompWdmIdleStateCallback;
    poFxSettings.PoFxDeviceContext = (PVOID) Device;
    
    status = WdfDeviceWdmAssignPowerFrameworkSettings(Device, &poFxSettings);
    if (FALSE == NT_SUCCESS(status)) {
        /*Trace(TRACE_LEVEL_ERROR,
              "%!FUNC! - WdfDeviceWdmAssignPowerFrameworkSettings failed with "
              "%!status!.", 
              status);*/
    }
    return status;
}

_IRQL_requires_same_
_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
F0Entry(
    _In_ WDFDEVICE /*Device*/
    )
/*++
Routine Description:

    Helper function invoked when component (representing the whole device) is
    requested to enter F0.

Arguments:

    Device - Handle to the framework device object

Return Value:

    BOOLEAN indicating whether F0 transition has completed.
    
--*/
{
    //
    // Change the F-state of the component
    // This includes hardware-specific operations such as:
    //   * enabling DMA capabilities associated with the component
    //   * enabling interrupts associated with the component 
    //   * restoring component state
    //
    //
    //HwSimFStateChange(Device, 0);
    return TRUE;
}

_IRQL_requires_same_
_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
F0Exit(
    _In_ WDFDEVICE /*Device*/,
    _In_ ULONG /*State*/
    )
/*++
Routine Description:

    Helper function invoked when component (representing the whole device) is
    requested to exit F0.

Arguments:

    Device - Handle to the framework device object
    State - The new F-state to be entered

Return Value:

    BOOLEAN indicating whether Fx transition has completed.
    
--*/
{
    //
    // Change the F-state of the component
    // This includes hardware-specific operations such as:
    //   * disabling DMA capabilities associated with the component
    //   * disabling interrupts associated with the component 
    //   * saving component state
    //
    //HwSimFStateChange(Device, State);
    return TRUE;
}

VOID
SingleCompWdmIdleStateCallback(
    _In_ PVOID Context,
    _In_ ULONG Component,
    _In_ ULONG State
    )
/*++
Routine Description:

    This callback is invoked by Power Framework to notify driver about any
    F-state transition.

Arguments:

    Context - Context supplied to Power Framework. KMDF supplies WDFDEVICE as
              the context while registering with Power Framework. Hence Context
              contains the KMDF device object.
              
    Component - Component for which F state transition is requested. Since we
                have only one component this value is always 0.
                
Return Value:

    BOOLEAN indicating whether Fx transition has completed.
    
--*/
{
    WDFDEVICE device = NULL;
    DEVICE_CONTEXT* deviceContextPtr = NULL;
    BOOLEAN transitionComplete = TRUE;

    //
    // We have only component 0
    //
    if (0 != Component) {
        /*Trace(TRACE_LEVEL_ERROR,"%!FUNC! - Unexpected component %d",Component);*/
        ASSERT(FALSE);
    }

    //
    // Get the device
    //
    device = (WDFDEVICE) Context;
    
    //
    // Get the device context
    //
    deviceContextPtr = GetDeviceContext(device);

    //
    // Note the new F-state
    //
    switch (State) {
        case 0: {
            transitionComplete = F0Entry(device);
        }
        break;

        //
        // PEP may make us go to any of the F-states directly, hence we execute
        // F0Exit code for all of the Fx states.
        //
        // Transition to any Fx state happens from F0 (and not another
        // Fx state)
        //
        default: {
            ASSERT(State < FSTATE_COUNT);

            transitionComplete = F0Exit(device, State);
        }
        break;        
    }

    if (transitionComplete) {
        PoFxCompleteIdleState(deviceContextPtr->PoHandle, 0 /* Component */);
    }
}

VOID
SingleCompWdmActiveConditionCallback(
    _In_ PVOID Context,
    _In_ ULONG Component
    )
/*++
Routine Description:

    This callback is invoked by Power Framework to notify driver that one of its
    components has become active.

Arguments:

    Context - Context that we supplied when calling 
              WdfDeviceWdmAssignPowerFrameworkSettings.
              
    Component - Component that have become active. Since we have only one 
                component this value is always 0.
    
Return Value:

    None
    
--*/
{
    WDFDEVICE  device;
    DEVICE_CONTEXT* deviceContextPtr = NULL;

    //
    // We have only component 0
    //
    if (0 != Component) {
        /*Trace(TRACE_LEVEL_ERROR,"%!FUNC! - Unexpected component %d",Component);*/
        ASSERT(FALSE);
    }

    //
    // Get the device
    //
    device = (WDFDEVICE) Context;
    
    //
    // Get the device context
    //
    deviceContextPtr = GetDeviceContext(device);

    //
    // Mark ourselves as active
    //
    deviceContextPtr->IsActive = TRUE;

    return;
}
    
VOID
SingleCompWdmIdleConditionCallback(
   _In_ PVOID Context,
   _In_ ULONG Component
   )
/*++
Routine Description:
    This callback is invoked by Power Framework to notify driver that one of its
    components has become idle.

Arguments:

    Context - Context that we supplied when calling 
              WdfDeviceWdmAssignPowerFrameworkSettings.
              
    Component - Component that have become idle. Since we have only one 
                component this value is always 0.
    
Return Value:

    None
    
--*/
{
    WDFDEVICE  device;
    DEVICE_CONTEXT* deviceContextPtr = NULL;

    //
    // We have only component 0
    //
    if (0 != Component) {
        /*Trace(TRACE_LEVEL_ERROR,"%!FUNC! - Unexpected component %d",Component);*/
        ASSERT(FALSE);
    }

    //
    // Get the device
    //
    device = (WDFDEVICE) Context;
    
    //
    // Get the device context
    //
    deviceContextPtr = GetDeviceContext(device);

    PoFxCompleteIdleCondition(deviceContextPtr->PoHandle, 0);

    return;
}

/* 
WAKE-UP RELATED CALLBACKS 
*/

NTSTATUS
SingleCompEvtDeviceArmWakeFromS0(
    IN WDFDEVICE Device
)
/*++

Routine Description:

    EvtDeviceArmWakeFromS0 is called when the Framework arms the device for
    wake from S0.  If there is any device-specific initialization
    that needs to be done to arm internal wake signals, or to route internal
    interrupt signals to the wake logic, it should be done here.  The device
    will be moved out of the D0 state soon after this callback is invoked.

    This function is pageable and it will run at PASSIVE_LEVEL.

Arguments:

    Device - Handle to a Framework device object.

Return Value:

    NTSTATUS - Failure will result in the device remaining in the D0 state.

--*/
{
    UNREFERENCED_PARAMETER(Device);

    PAGED_CODE();

    KdPrint(("--> SingleCompEvtDeviceArmWakeFromS0\n"));

    KdPrint(("<-- SingleCompEvtDeviceArmWakeFromS0\n"));

    return STATUS_SUCCESS;
}

NTSTATUS
SingleCompEvtDeviceArmWakeFromSx(
    IN WDFDEVICE Device
)
/*++

Routine Description:

    EvtDeviceArmWakeFromSx is called when the Framework arms the device for
    wake from Sx.  If there is any device-specific initialization
    that needs to be done to arm internal wake signals, or to route internal
    interrupt signals to the wake logic, it should be done here.  The device
    will be moved out of the D0 state soon after this callback is invoked.

    This function is pageable and it will run at PASSIVE_LEVEL.

Arguments:

    Device - Handle to a Framework device object.

Return Value:

    NTSTATUS - Failure will result in the device remaining in the D0 state.

--*/
{
    UNREFERENCED_PARAMETER(Device);

    PAGED_CODE();

    KdPrint(("--> SingleCompEvtDeviceArmWakeFromSx\n"));

    KdPrint(("<-- SingleCompEvtDeviceArmWakeFromSx\n"));

    return STATUS_SUCCESS;
}

VOID
SingleCompEvtDeviceDisarmWakeFromS0(
    IN WDFDEVICE Device
)
/*++

Routine Description:

    EvtDeviceDisarmWakeFromS0 reverses anything done in EvtDeviceArmWakeFromS0.

    This function is not marked pageable because this function is in the
    device power up path. When a function is marked pagable and the code
    section is paged out, it will generate a page fault which could impact
    the fast resume behavior because the client driver will have to wait
    until the system drivers can service this page fault.

Arguments:

    Device - Handle to a Framework device object.

Return Value:

    VOID.

--*/
{
    UNREFERENCED_PARAMETER(Device);

    KdPrint(("--> SingleCompEvtDeviceDisarmWakeFromS0\n"));

    KdPrint(("<-- SingleCompEvtDeviceDisarmWakeFromS0\n"));

    return;
}

VOID
SingleCompEvtDeviceDisarmWakeFromSx(
    IN WDFDEVICE Device
)
/*++

Routine Description:

    EvtDeviceDisarmWakeFromSx reverses anything done in EvtDeviceArmWakeFromSx.

    This function will run at PASSIVE_LEVEL.

    This function is not marked pageable because this function is in the
    device power up path. When a function is marked pagable and the code
    section is paged out, it will generate a page fault which could impact
    the fast resume behavior because the client driver will have to wait
    until the system drivers can service this page fault.

Arguments:

    Device - Handle to a Framework device object.

Return Value:

    VOID.

--*/
{
    UNREFERENCED_PARAMETER(Device);

    KdPrint(("--> SingleCompEvtDeviceDisarmWakeFromSx\n"));

    KdPrint(("<-- SingleCompEvtDeviceDisarmWakeFromSx\n"));

    return;
}

VOID
SingleCompEvtDeviceWakeFromS0Triggered(
    IN WDFDEVICE Device
)
/*++

Routine Description:

    EvtDeviceWakeFromS0Triggered will be called whenever the device triggers its
    wake signal after being armed for wake.

    This function is pageable and runs at PASSIVE_LEVEL.

Arguments:

    Device - Handle to a Framework device object.

Return Value:

    VOID

--*/
{
    UNREFERENCED_PARAMETER(Device);

    PAGED_CODE();

    KdPrint(("--> SingleCompEvtDeviceWakeFromS0Triggered\n"));


    KdPrint(("<-- SingleCompEvtDeviceWakeFromS0Triggered\n"));

}

VOID
SingleCompEvtDeviceWakeFromSxTriggered(
    IN WDFDEVICE Device
)
/*++

Routine Description:

    EvtDeviceWakeFromSxTriggered will be called whenever the device triggers its
    wake signal after being armed for wake.

    This function runs at PASSIVE_LEVEL.

    This function is not marked pageable because this function is in the
    device power up path. When a function is marked pagable and the code
    section is paged out, it will generate a page fault which could impact
    the fast resume behavior because the client driver will have to wait
    until the system drivers can service this page fault.

Arguments:

    Device - Handle to a Framework device object.

Return Value:

    VOID

--*/
{
    UNREFERENCED_PARAMETER(Device);

    KdPrint(("--> SingleCompEvtDeviceWakeFromSxTriggered\n"));

    KdPrint(("<-- SingleCompEvtDeviceWakeFromSxTriggered\n"));

}
