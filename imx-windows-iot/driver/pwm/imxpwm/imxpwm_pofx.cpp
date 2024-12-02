/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2022 NXP
   Licensed under the MIT License.

Abstract:
    This module implements the power framework (PoFx) in PWM driver to manage the power 
    state of the component that represents device.

*/

#include "precomp.h"

#include <ntddk.h>
#include <wdf.h>

#include "imxpwm_pofx.h"
#include "imxpwmhw.hpp"
#include "imxpwm.hpp"

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
    IMXPWM_DEVICE_CONTEXT*  deviceContextPtr = NULL;
    
    PAGED_CODE();

    deviceContextPtr = ImxPwmGetDeviceContext(device);

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

exit:
    return status;
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
    IMXPWM_DEVICE_CONTEXT* deviceContextPtr = NULL;

    PAGED_CODE();
    
    //
    // Get the device context
    //
    deviceContextPtr = ImxPwmGetDeviceContext(Device);

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
    IMXPWM_DEVICE_CONTEXT* deviceContextPtr = NULL;

    PAGED_CODE();
    
    UNREFERENCED_PARAMETER(PoHandle);

    //
    // Get the device context
    //
    deviceContextPtr = ImxPwmGetDeviceContext(Device);

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
    IMXPWM_DEVICE_CONTEXT* deviceContextPtr = NULL;
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
    deviceContextPtr = ImxPwmGetDeviceContext(device);

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
    IMXPWM_DEVICE_CONTEXT* deviceContextPtr = NULL;

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
    deviceContextPtr = ImxPwmGetDeviceContext(device);

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
    IMXPWM_DEVICE_CONTEXT* deviceContextPtr = NULL;

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
    deviceContextPtr = ImxPwmGetDeviceContext(device);

    PoFxCompleteIdleCondition(deviceContextPtr->PoHandle, 0);

    return;
}

