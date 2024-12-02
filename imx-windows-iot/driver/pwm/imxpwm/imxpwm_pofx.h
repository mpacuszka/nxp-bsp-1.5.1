/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2022 NXP
   Licensed under the MIT License.

Abstract:
    This module implements the power framework (PoFx) in PWM driver to manage the power
    state of the component that represents device.

*/

#pragma once

#define FSTATE_COUNT    2
#define DEEPEST_FSTATE_LATENCY_IN_MS    800
#define DEEPEST_FSTATE_RESIDENCY_IN_SEC 12

// The PWM device will do to D3/F1 (low power) state after 30 sec of idle
#define PWM_IDLE_TIMEOUT_IN_MS        30000

//
// Driver's KMDF callbacks
//

NTSTATUS
PowerManagementSetup(
    _Inout_ WDFDEVICE device
);

EVT_WDFDEVICE_WDM_POST_PO_FX_REGISTER_DEVICE    SingleCompWdmEvtDeviceWdmPostPoFxRegisterDevice;
EVT_WDFDEVICE_WDM_PRE_PO_FX_UNREGISTER_DEVICE   SingleCompWdmEvtDeviceWdmPrePoFxUnregisterDevice;

//
// Driver's power framework callbacks
//
PO_FX_COMPONENT_ACTIVE_CONDITION_CALLBACK SingleCompWdmActiveConditionCallback;
PO_FX_COMPONENT_IDLE_CONDITION_CALLBACK SingleCompWdmIdleConditionCallback;
PO_FX_COMPONENT_IDLE_STATE_CALLBACK SingleCompWdmIdleStateCallback;

//
// Helper functions
// 

_IRQL_requires_same_
_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS 
AssignS0IdleSettings(
    _In_ WDFDEVICE Device
    );

_IRQL_requires_same_
_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS 
AssignPowerFrameworkSettings(
    _In_ WDFDEVICE Device
    );

_IRQL_requires_same_
_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
F0Entry(
    _In_ WDFDEVICE Device
    );

_IRQL_requires_same_
_IRQL_requires_max_(DISPATCH_LEVEL)
BOOLEAN
F0Exit(
    _In_ WDFDEVICE Device,
    _In_ ULONG State
    );

//
// Define the tracing flags.
//
#ifndef WPP_CONTROL_GUIDS
#define WPP_CONTROL_GUIDS                                                   \
    WPP_DEFINE_CONTROL_GUID(                                                \
        MyDriverTraceControl, (f9eb5c3a,c292,4c69,8b52,ccf043c25ab0),       \
                                                                            \
        WPP_DEFINE_BIT(MYDRIVER_ALL_INFO)                                   \
        )
#endif

#define WPP_FLAG_LEVEL_LOGGER(flag, level)                                  \
    WPP_LEVEL_LOGGER(flag)

#define WPP_FLAG_LEVEL_ENABLED(flag, level)                                 \
    (WPP_LEVEL_ENABLED(flag) &&                                             \
     WPP_CONTROL(WPP_BIT_ ## flag).Level >= level)

//
// This comment block is scanned by the trace preprocessor to define our
// Trace function.
//
// begin_wpp config
// FUNC Trace{FLAG=MYDRIVER_ALL_INFO}(LEVEL, MSG, ...);
// end_wpp
//    

