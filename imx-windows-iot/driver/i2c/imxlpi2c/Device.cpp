/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2022-2023 NXP

   Licensed under the MIT License.

Module Name:

    device.c - Device handling events for example driver.

Abstract:

   This file contains the device entry points and callbacks.

Environment:

    Kernel-mode Driver Framework

*/

extern "C" {
#include <svc/scfw.h>
#include <svc/ipc.h>
#include <svc/pm/pm_api.h>
}
#include "imxlpi2cinternal.h"
#include "imxlpi2cdevice.h"
#include "imxlpi2ccontroller.h"
#include <devpkey.h>
#include "device.tmh"
#include <acpiioct.h>
#include "acpiutil.hpp"

const sc_rsrc_t I2cQxpResources[] =
{
    SC_R_I2C_0,          // I2C0
    SC_R_I2C_1,          // I2C1
    SC_R_I2C_2,          // I2C2
    SC_R_I2C_3,          // I2C3
    SC_R_I2C_4,          // I2C4
    SC_R_CSI_0_I2C_0,    // I2C0_MIPI_CSI0
    SC_R_PI_0_I2C_0,     // I2C0_PARALLEL
    SC_R_CSI_1_I2C_0,    // I2C0_MIPI_CSI1
    SC_R_MIPI_0_I2C_0,   // I2C0_MIPI_LVDS0
    SC_R_MIPI_1_I2C_0,   // I2C0_MIPI_LVDS1
    SC_R_LAST,           // I2C1_LVDS0 - not on QXP
    SC_R_LAST,           // I2C0_LVDS1 - not on QXP
    SC_R_LAST,           // I2C1_LVDS1 - not on QXP
    SC_R_LAST,           // I2C0_MIPI_DSI1 - not on QXP
    SC_R_LAST,           // I2C0_HDMI - not on QXP
    SC_R_M4_0_I2C,       // CM40_I2C
    SC_R_LAST            // CM41_I2C - not on QXP
};


// WDF and SPB DDI callbacks.

/*++

  Routine Description:

    This routine gets clocks of I2C peripheral.

  Arguments:

    MuIpcHandle - a handle to the SCFW Message Unit
    DeviceCtxPtr - a pointer to the PBC device context
    Frequency - a pointer to Module clock frquency 

  Return Value:

    Status

--*/
static NTSTATUS GetI2cFrequencyIpc(sc_ipc_t MuIpcHandle, PDEVICE_CONTEXT DeviceCtxPtr, ULONG* Frequency)
{
    NTSTATUS status = STATUS_SUCCESS;
    sc_err_t sc_err = SC_ERR_NONE;
    sc_pm_clock_rate_t i2cRate = 0;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++GetI2CFrequencyIpc()");

    NT_ASSERT(DeviceCtxPtr != NULL);
    NT_ASSERT((DeviceCtxPtr->ControllerID < I2CUNKNOWN) && (DeviceCtxPtr->ControllerID >= 0));

    sc_err = sc_pm_get_clock_rate(MuIpcHandle, I2cQxpResources[DeviceCtxPtr->ControllerID],
        SC_PM_CLK_PER, &i2cRate);
    if (sc_err != SC_ERR_NONE) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_CTRLR,
            "GetI2CFrequency() Error %Xh unable to get I2C rate",
            status);
        status = STATUS_UNSUCCESSFUL;
        goto DoneGetI2CFrequencyIpc;
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "GetI2cConfigValues() I2C (index: %u, SCU resource: %u) rate value from SCFW %luHz", 
        DeviceCtxPtr->ControllerID, I2cQxpResources[DeviceCtxPtr->ControllerID], (ULONG)i2cRate);

    *Frequency = (ULONG)i2cRate;

DoneGetI2CFrequencyIpc:
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--GetI2CFrequencyIpc()=%Xh", status);

    return status;
}

/*++

  Routine Description:

    If Frequency parameter is nono-zero then this routine sets frequency of I2C peripheral and
    gets actually set frequency. If the Frequency iz zero then this routine just gets
    currently set frequency.

  Arguments:

    DeviceCtxPtr - a pointer to the PBC device context
    Frequency - a pointer to Module clock frquency

  Return Value:

    Status

--*/
static NTSTATUS SetI2cFrequency(PDEVICE_CONTEXT DeviceCtxPtr, ULONG* Frequency)
{
    NTSTATUS status = STATUS_SUCCESS;
    sc_ipc_struct_t MuIpcHandle = { 0 };

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "++GetI2cFrequency()");

    status = sc_ipc_open(&MuIpcHandle, &DeviceCtxPtr->scfw_ipc_id);
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
            "GetI2cFrequency() Error %Xh to open MU channel to SCU",
            status);

        goto DoneGetI2cFrequency;
    }

    if (*Frequency != 0) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
            "GetI2cFrequency() Error setting I2C mdule clock is not supported");

        status = STATUS_UNSUCCESSFUL;
        goto DoneGetI2cFrequency;
    }
    else {
        status = GetI2cFrequencyIpc(&MuIpcHandle, DeviceCtxPtr, Frequency);

        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
                "GetI2cFrequency() Error %Xh unable to get I2C module clock",
                status);

            goto DoneGetI2cFrequency;
        }
    }

DoneGetI2cFrequency:
    sc_ipc_close(&MuIpcHandle);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "--GetI2cFrequency()=%Xh", status);

    return status;
}

/*++

  Routine Description:

    This routine maps the hardware resources to the SPB
    controller register structure.

  Arguments:

    WdfDevice - a handle to the framework device object
    FxResourcesRaw - list of translated hardware resources that
        the PnP manager has assigned to the device
    FxResourcesTranslated - list of raw hardware resources that
        the PnP manager has assigned to the device

  Return Value:

    Status

--*/_Use_decl_annotations_
NTSTATUS OnPrepareHardware(
    WDFDEVICE WdfDevice,
    WDFCMRESLIST FxResourcesRaw,
    WDFCMRESLIST FxResourcesTranslated
    )
{
    ULONG resourceCount = 0;
    PDEVICE_CONTEXT deviceCtxPtr = NULL;
    NTSTATUS status = STATUS_SUCCESS;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR res;
    const CM_PARTIAL_RESOURCE_DESCRIPTOR* scfwResourceIpcPtr = NULL;
    UNREFERENCED_PARAMETER(FxResourcesRaw);
    int iCountMemoryRes = 1;  // must have one memory resource
    int iCountInterruptRes = 1; 

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnPrepareHardware()");

    deviceCtxPtr = GetDeviceContext(WdfDevice);
    NT_ASSERT(deviceCtxPtr != NULL);

    // Get the register base for the I2C controller.

    resourceCount = WdfCmResourceListGetCount(FxResourcesTranslated);
    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "OnPrepareHardware() - %lu resources found.", resourceCount);

    for (ULONG i = 0; i < resourceCount; i++) {

        res = WdfCmResourceListGetDescriptor(FxResourcesTranslated, i);
        if (res == NULL) {
            continue;
        }

        switch (res->Type) {

        case CmResourceTypeMemory:
            TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "ACPI resource %lu - memory.", i);

            deviceCtxPtr->RegistersPtr = (IMXI2C_REGISTERS*)MmMapIoSpaceEx(res->u.Memory.Start,
                            res->u.Memory.Length, PAGE_NOCACHE | PAGE_READWRITE);

            deviceCtxPtr->RegistersCb = res->u.Memory.Length;

            // determine controller ID from memory address

            switch (res->u.Memory.Start.QuadPart) {

            case IMX8_I2C0_PA:
                deviceCtxPtr->ControllerID = I2C0;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C0");
                break;

            case IMX8_I2C1_PA:
            case IMX9_I2C1_PA:
                deviceCtxPtr->ControllerID = I2C1;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C1");
                break;

            case IMX8_I2C2_PA:
            case IMX9_I2C2_PA:
                deviceCtxPtr->ControllerID = I2C2;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C2");
                break;

            case IMX8_I2C3_PA:
            case IMX9_I2C3_PA:
                deviceCtxPtr->ControllerID = I2C3;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C3");
                break;

            case IMX8_I2C4_PA:
            case IMX9_I2C4_PA:
                deviceCtxPtr->ControllerID = I2C4;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C4");
                break;

            case IMX8_I2C0_MIPI_CSI0_PA:
            case IMX9_I2C5_PA:
                deviceCtxPtr->ControllerID = I2C5;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C0_MIPI_CSI0");
                break;

            case IMX8_I2C0_PARALLEL_PA:
            case IMX9_I2C6_PA:
                deviceCtxPtr->ControllerID = I2C6;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C0_PARALLEL");
                break;

            case IMX8_I2C0_MIPI_CSI1_PA:
            case IMX9_I2C7_PA:
                deviceCtxPtr->ControllerID = I2C7;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C0_MIPI_CSI1");
                break;

            case IMX8_I2C0_MIPI_LVDS0_OR_I2C0_MIPI_DSI0_PA:
            case IMX9_I2C8_PA:
                deviceCtxPtr->ControllerID = I2C8;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C0_MIPI_LVDS0 or I2C0_MIPI_DSI0");
                break;

            case IMX8_I2C0_MIPI_LVDS1_OR_I2C0_LVDS0_PA:
                deviceCtxPtr->ControllerID = I2C9;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C0_MIPI_LVDS1 or I2C0_LVDS0");
                break;

            case IMX8_I2C1_LVDS0_PA:
                deviceCtxPtr->ControllerID = I2C10;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C1_LVDS0");
                break;

            case IMX8_I2C0_LVDS1_PA:
                deviceCtxPtr->ControllerID = I2C11;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C0_LVDS1");
                break;

            case IMX8_I2C1_LVDS1_PA:
                deviceCtxPtr->ControllerID = I2C12;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C1_LVDS1");
                break;

            case IMX8_I2C0_MIPI_DSI1_PA:
                deviceCtxPtr->ControllerID = I2C13;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C0_MIPI_DSI1");
                break;

            case IMX8_I2C0_HDMI_PA:
                deviceCtxPtr->ControllerID = I2C14;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller I2C0_HDMI");
                break;

            case IMX8_CM40_I2C_PA:
                deviceCtxPtr->ControllerID = I2C15;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller CM40_I2C");
                break;

            case IMX8_CM41_I2C_PA:
                deviceCtxPtr->ControllerID = I2C16;
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, " Controller CM41_I2C");
                break;

            default:
                deviceCtxPtr->ControllerID = I2CUNKNOWN;
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, " Controller I2C ??");
                break;
            }


            if (deviceCtxPtr->RegistersPtr == NULL) {

                status = STATUS_INSUFFICIENT_RESOURCES;

                TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
                    "Error mapping controller registers (PA:%I64x, length:%d) "
                    "for WDFDEVICE %p - Err=%Xh",
                    res->u.Memory.Start.QuadPart,
                    res->u.Memory.Length,
                    deviceCtxPtr->WdfDevice,
                    status);

                NT_ASSERT(deviceCtxPtr->RegistersPtr != NULL);
            } 
            else {

                // Save the physical address to help identify
                // the underlying controller for debugging.

                deviceCtxPtr->RegistersPhysicalAddress = res->u.Memory.Start;

                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
                    "I2C controller @ phys addr %I64xh virt addr @ %ph for WDFDEVICE %ph",
                    deviceCtxPtr->RegistersPhysicalAddress.QuadPart,
                    deviceCtxPtr->RegistersPtr,
                    deviceCtxPtr->WdfDevice);
                iCountMemoryRes -= 1;

            }
            break;

        case CmResourceTypeInterrupt:
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "ACPI resource %lu - interrupt.", i);
            iCountInterruptRes-=1;
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "Interrupt vect=%Xh disp=%Xh affin=%IXh",
                res->u.Interrupt.Vector,
                res->ShareDisposition,
                res->u.Interrupt.Affinity);
            break;

        case CmResourceTypeConnection:
            if ((res->u.Connection.Type != CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)) {
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
                    "Received unexpected connection resource type");

                return STATUS_DEVICE_CONFIGURATION_ERROR;
            }
            if (scfwResourceIpcPtr == NULL) {
                scfwResourceIpcPtr = res;
                deviceCtxPtr->scfw_ipc_id.ipc_id.LowPart = scfwResourceIpcPtr->u.Connection.IdLowPart;
                deviceCtxPtr->scfw_ipc_id.ipc_id.HighPart = scfwResourceIpcPtr->u.Connection.IdHighPart;
                deviceCtxPtr->ScfwAvailable = TRUE;
            }
            else {
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
                    "Received unexpected connection resource count");

                return STATUS_DEVICE_CONFIGURATION_ERROR;
            }
            break;

        default:
            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "ACPI resource %lu - other", i);

            // unknown resource will result in an error condition.

            break;
        }
    } // end FOR loop for resources

    if (((iCountInterruptRes == 0) || (iCountInterruptRes == 1)) &&
        (iCountMemoryRes == 0) && ((scfwResourceIpcPtr != NULL) || (deviceCtxPtr->ModuleClock_Hz != 0U))) {

        status=STATUS_SUCCESS;
    } 
    else {

        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "OnPrepareHardware(ERR) Did not get required ACPI resource(s).");
        status = STATUS_INSUFFICIENT_RESOURCES;
    }

    if (iCountInterruptRes == 1) {
        deviceCtxPtr->UseInterruptMode = FALSE;
    }
    else {
        deviceCtxPtr->UseInterruptMode = TRUE;
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnPrepareHardware()=%Xh",status);

    return status;
}

/*++

  Routine Description:

    This routine unmaps the SPB controller register structure.

  Arguments:

    WdfDevice - a handle to the framework device object
    FxResourcesRaw - list of translated hardware resources that
        the PnP manager has assigned to the device
    FxResourcesTranslated - list of raw hardware resources that
        the PnP manager has assigned to the device

  Return Value:

    Status

--*/
_Use_decl_annotations_
NTSTATUS OnReleaseHardware(
    WDFDEVICE WdfDevice,
    WDFCMRESLIST FxResourcesTranslated
    )
{

    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(WdfDevice);
    NT_ASSERT(deviceCtxPtr != NULL);

    NTSTATUS status = STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(FxResourcesTranslated);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnReleaseHardware()");

    if (deviceCtxPtr->RegistersPtr != NULL) {

        MmUnmapIoSpace(deviceCtxPtr->RegistersPtr, deviceCtxPtr->RegistersCb);

        deviceCtxPtr->RegistersPtr = NULL;
        deviceCtxPtr->RegistersCb = 0;
    };

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnReleaseHardware()=%Xh",status);

    return status;
}

/*++

  Routine Description:

    This routine allocates objects needed by the driver
    and initializes the controller hardware.

  Arguments:

    WdfDevice - a handle to the framework device object
    FxPreviousState - previous power state

  Return Value:

    Status

--*/
_Use_decl_annotations_
NTSTATUS OnD0Entry(
    WDFDEVICE Device,
    WDF_POWER_DEVICE_STATE FxPreviousState
    )
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnD0Entry");
    NTSTATUS status = STATUS_SUCCESS;
    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(Device);
    ULONG ModuleClock = 0;
    NT_ASSERT(deviceCtxPtr != NULL);

    UNREFERENCED_PARAMETER(FxPreviousState);

    // Initialize iMX i2c controller.

    deviceCtxPtr->CurrentTargetPtr = NULL;

    deviceCtxPtr = GetDeviceContext(Device);
    NT_ASSERT(deviceCtxPtr != NULL);

    // Get currently set I2C module clock (0 in ModuleClock variable)
    ModuleClock = deviceCtxPtr->ModuleClock_Hz;
    if (deviceCtxPtr->ScfwAvailable) {
        status = SetI2cFrequency(deviceCtxPtr, &ModuleClock);

        if (!NT_SUCCESS(status)) {

            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "OnD0Entry() - error returned by InitI2CPwrClocks");
            goto DoneOnD0Entry;
        }

        deviceCtxPtr->ModuleClock_Hz = ModuleClock;
        deviceCtxPtr->PeripheralAccessClock_Hz = ModuleClock;
    }

    ControllerInitialize(deviceCtxPtr);

DoneOnD0Entry:
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnD0Entry()=%Xh", status);

    return status;
}

/*++

  Routine Description:

    This routine destroys objects needed by the driver
    and uninitializes the controller hardware.

  Arguments:

    WdfDevice - a handle to the framework device object
    FxPreviousState - previous power state

  Return Value:

    Status

--*/
_Use_decl_annotations_
NTSTATUS OnD0Exit(
    WDFDEVICE WdfDevice,
    WDF_POWER_DEVICE_STATE FxPreviousState
    )
{
    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(WdfDevice);
    NTSTATUS status = STATUS_SUCCESS;
    UNREFERENCED_PARAMETER(FxPreviousState);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnD0Exit");

    NT_ASSERT(deviceCtxPtr != NULL);

    // Uninitialize controller.

    ControllerUninitialize(deviceCtxPtr);

    deviceCtxPtr->CurrentTargetPtr = NULL;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnD0Exit()=%Xh",status);

    return status;
}

/*++

  Routine Description:

    Initializes and starts the device's self-managed I/O operations.

  Arguments:

    WdfDevice - a handle to the framework device object

  Return Value:

    None

--*/
_Use_decl_annotations_
NTSTATUS OnSelfManagedIoInit(WDFDEVICE WdfDevice)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnSelfManagedIoInit()");

    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(WdfDevice);
    NTSTATUS status;

    // Register for monitor power setting callback. This will be
    // used to dynamically set the idle timeout delay according
    // to the monitor power state.

    NT_ASSERT(deviceCtxPtr->MonitorPowerSettingHandlePtr == NULL);

    status = PoRegisterPowerSettingCallback(WdfDeviceWdmGetDeviceObject(deviceCtxPtr->WdfDevice),
        &GUID_MONITOR_POWER_ON,
        OnMonitorPowerSettingCallback,
        (PVOID)deviceCtxPtr->WdfDevice,
        &deviceCtxPtr->MonitorPowerSettingHandlePtr);

    if (!NT_SUCCESS(status)) {

        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "Failed to register monitor power setting callback - Err=%Xh",
            status);
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnSelfManagedIoInit()=success");

    return status;
}

/*++

  Routine Description:

    Cleanup for the device's self-managed I/O operations.

  Arguments:

    WdfDevice - a handle to the framework device object

  Return Value:

    None

--*/
_Use_decl_annotations_
VOID OnSelfManagedIoCleanup(WDFDEVICE WdfDevice)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnSelfManagedIoCleanup");

    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(WdfDevice);

    // Unregister for monitor power setting callback.

    if (deviceCtxPtr->MonitorPowerSettingHandlePtr != NULL) {

        PoUnregisterPowerSettingCallback(deviceCtxPtr->MonitorPowerSettingHandlePtr);
        deviceCtxPtr->MonitorPowerSettingHandlePtr = NULL;
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnSelfManagedIoCleanup()");
    return;
}

/*++

  Routine Description:

    This routine updates the idle timeout delay according
    to the current monitor power setting.

  Arguments:

    SettingGuid - the setting GUID
    Value - pointer to the new value of the power setting that changed
    ValueLength - value of type ULONG that specifies the size, in bytes,
                  of the new power setting value
    Context - the WDFDEVICE pointer context

  Return Value:

    Status

--*/
_Use_decl_annotations_
NTSTATUS OnMonitorPowerSettingCallback(
    LPCGUID SettingGuid,
    PVOID Value,
    ULONG ValueLength,
    PVOID Context
    )
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnMonitorPowerSettingCallback()");

    UNREFERENCED_PARAMETER(ValueLength);

    WDFDEVICE device;
    WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS idleSettings;
    BOOLEAN isMonitorOff;
    NTSTATUS status = STATUS_SUCCESS;

    if (Context == NULL) {

        status = STATUS_INVALID_PARAMETER;

        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
            "%!FUNC! parameter Context is NULL - Err=%Xh",
            status);

        goto exit;
    }

    device = (WDFDEVICE)Context;

    // We only expect GUID_MONITOR_POWER_ON notifications
    // in this callback, but let's check just to be sure.

    if (IsEqualGUID(*SettingGuid, GUID_MONITOR_POWER_ON)) {
        NT_ASSERT(Value != NULL);
        NT_ASSERT(ValueLength == sizeof(ULONG));

        // Determine power setting.

        isMonitorOff = ((*(PULONG)Value) == MONITOR_POWER_OFF);

        // Update the idle timeout delay.

        WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS_INIT(&idleSettings,
            IdleCannotWakeFromS0);

        idleSettings.IdleTimeoutType = SystemManagedIdleTimeoutWithHint;

        if (isMonitorOff) {

            idleSettings.IdleTimeout = IDLE_TIMEOUT_MONITOR_OFF;
        }
        else {
            idleSettings.IdleTimeout = IDLE_TIMEOUT_MONITOR_ON;

        }

        // do not assign idle power settings for a driver which is not power managed

#ifdef I2C_IS_PEP_MANAGED
        status = WdfDeviceAssignS0IdleSettings(Device, &idleSettings);

        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
                "Failed to assign S0 idle settings - Err=%Xh",
                status);
         }
#endif
    }

exit:

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnMonitorPowerSettingCallback()=%Xh", status);

    return status;
}

/*++

  Routine Description:

    This routine is invoked whenever a peripheral driver opens
    a target.  It retrieves target-specific settings from the
    Resource Hub and saves them in the target's context.
    This callback is made at passive level.

  Arguments:

    SpbController - a handle to the framework device object representing an SPB controller

    SpbTarget - handle to the SPBTARGET target to open. The target is a peripheral device or port that is attached to the bus.

  Return Value:

    returns STATUS_SUCCESS if the driver successfully opens the connection to the target.
    Otherwise, the function returns an appropriate NTSTATUS error code.

--*/
_Use_decl_annotations_
NTSTATUS OnTargetConnect(
    WDFDEVICE SpbController,
    SPBTARGET SpbTarget
    )
{
    NT_ASSERT(KeGetCurrentIrql() <= PASSIVE_LEVEL);
    NTSTATUS status = STATUS_SUCCESS;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnTargetConnect()");

    PDEVICE_CONTEXT deviceCtxPtr  = GetDeviceContext(SpbController);
    PPBC_TARGET targetPtr  = GetTargetContext(SpbTarget);

    NT_ASSERT(deviceCtxPtr != NULL);
    NT_ASSERT(targetPtr != NULL);

    // Get target connection parameters.

    SPB_CONNECTION_PARAMETERS params;
    SPB_CONNECTION_PARAMETERS_INIT(&params);

    SpbTargetGetConnectionParameters(SpbTarget, &params);

    // Retrieve target settings.

    status = PbcTargetGetSettings(deviceCtxPtr,
                                  params.ConnectionParameters,
                                  &targetPtr->Settings);

    // Initialize target context.

    if (NT_SUCCESS(status)) {

        targetPtr->SpbTarget = SpbTarget;
        targetPtr->CurrentRequestPtr = NULL;

        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
            "Connected to SPBTARGET %p at address 0x%lx from WDFDEVICE %p",
            targetPtr->SpbTarget,
            targetPtr->Settings.Address,
            deviceCtxPtr->WdfDevice);
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnTargetConnect()=%Xh",status);

    return status;
}


/*++

  Routine Description:

    This routine sets up a read from the target device using
    the supplied buffers.  The request is only completed
    if there is an error configuring the transfer.

  Arguments:

    SpbController - a handle to the framework device object
        representing an SPB controller
    SpbTarget - a handle to the SPBTARGET object
    SpbRequest - a handle to the SPBREQUEST object
    Length - the number of bytes to read from the target

  Return Value:

    None.  The request is completed asynchronously.

--*/
_Use_decl_annotations_
VOID OnRead(
    WDFDEVICE SpbController,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest,
    size_t Length)
{
    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_READ, "++OnRead()");

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_READ,
        "Received read request %p of length %Iu for SPBTARGET %p "
        "(WDFDEVICE %p)",
        SpbRequest,
        Length,
        SpbTarget,
        SpbController);

    PbcRequestConfigureForNonSequence(SpbController,
        SpbTarget,
        SpbRequest,
        Length);

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_READ, "--OnRead()");
    return;
}

/*++
  Routine Description:

    This callback function writes data to the specified target device from the buffers that are supplied with the write request.
    EvtSpbControllerIoWrite callback is synchronous. That is, the callback function initiates the requested write operation and
    completes the write request.

  Arguments:

    SpbController - a handle to the framework device object representing an SPB controller
    SpbTarget - a handle to the SPBTARGET object for this I/O request.
                The target is a peripheral device or port that is attached to the bus.
                The SPB framework extension (SpbCx) previously assigned this handle to the target in the EvtSpbTargetConnect callback
                that opened the connection to the target
    SpbRequest - a handle to I/O SPBREQUEST object. Must complete this request either by performing the requested operation or
                by returning an error status
    Length - the number of bytes to write to the target

  Return Value:

    None.  EvtSpbControllerIoWrite callback does not return a status value.
    Instead, the SPB controller driver indicates the status of the write operation in the completion status for the I/O request

--*/
_Use_decl_annotations_
VOID OnWrite(
    WDFDEVICE SpbController,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest,
    size_t Length
    )
{
    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_WRITE, "++OnWrite()");

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_WRITE,
        "Received write request %p of length %Iu for SPBTARGET %p "
        "(WDFDEVICE %p)",
        SpbRequest,
        Length,
        SpbTarget,
        SpbController);

    PbcRequestConfigureForNonSequence(SpbController,
        SpbTarget,
        SpbRequest,
        Length);

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_WRITE, "--OnWrite()");
    return;
}

/*++

  Routine Description:

    This routine sets up a sequence of reads and writes.  It
    validates parameters as necessary.  The request is only
    completed if there is an error configuring the transfer.

  Arguments:

    SpbController - a handle to the framework device object
        representing an SPB controller
    SpbTarget - a handle to the SPBTARGET object
    SpbRequest - a handle to the SPBREQUEST object
    TransferCount - number of individual transfers in the sequence

  Return Value:

    None.  The request is completed asynchronously.

--*/
_Use_decl_annotations_
VOID OnSequence(
    WDFDEVICE SpbController,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest,
    ULONG TransferCount
    )
{
    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "++OnSequence()");

    PDEVICE_CONTEXT deviceCtxPtr  = GetDeviceContext(SpbController);
    PPBC_TARGET  targetPtr  = GetTargetContext(SpbTarget);
    PPBC_REQUEST requestPtr = GetRequestContext(SpbRequest);

    NT_ASSERT(deviceCtxPtr != NULL);
    NT_ASSERT(targetPtr != NULL);
    NT_ASSERT(requestPtr != NULL);

    NTSTATUS status = STATUS_SUCCESS;

    // Get request parameters.

    SPB_REQUEST_PARAMETERS params;
    SPB_REQUEST_PARAMETERS_INIT(&params);
    SpbRequestGetParameters(SpbRequest, &params);

    NT_ASSERT(params.Position == SpbRequestSequencePositionSingle);
    NT_ASSERT(params.Type == SpbRequestTypeSequence);

    // Initialize request context.

    requestPtr->SpbRequest = SpbRequest;
    requestPtr->Type = params.Type;
    requestPtr->TotalInformation = 0;
    requestPtr->TransferCount = TransferCount;
    requestPtr->TransferIndex = 0;
    requestPtr->bIoComplete = FALSE;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER,
        "Received sequence request %p with transfer count %d for SPBTARGET %p "
        "(WDFDEVICE %p)",
        requestPtr->SpbRequest,
        requestPtr->TransferCount,
        SpbTarget,
        SpbController);

    // Validate the request before beginning the transfer.

    status = PbcRequestValidate(requestPtr);

    if (!NT_SUCCESS(status)) {

        goto exit;
    }

    // Configure the request. Starts with TransferIndex=0
    // TransferIndex will grow as transfers succeed

    status = PbcRequestConfigureForIndex(requestPtr, 0);

    if (!NT_SUCCESS(status)) {

        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
            "Error configuring request context for SPBREQUEST %p "
            "(SPBTARGET %p) - %!STATUS!",
            requestPtr->SpbRequest,
            SpbTarget,
            status);

        goto exit;
    }

    // Acquire the device lock.

    WdfSpinLockAcquire(deviceCtxPtr->Lock);

    // Mark request cancellable (if cancellation supported).

    status = WdfRequestMarkCancelableEx(requestPtr->SpbRequest, OnCancel);

    if (!NT_SUCCESS(status)) {

        // WdfRequestMarkCancelableEx should only fail if the request
        // has already been cancelled. If it does fail the request
        // must be completed with the corresponding status.

        NT_ASSERTMSG("WdfRequestMarkCancelableEx should only fail"
            " if the request has already been cancelled",
            status == STATUS_CANCELLED);

        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER,
            "Failed to mark SPBREQUEST %p cancellable - %!STATUS!",
            requestPtr->SpbRequest,
            status);

        WdfSpinLockRelease(deviceCtxPtr->Lock);
        goto exit;
    }

    // Update device and target contexts.

    NT_ASSERT(deviceCtxPtr->CurrentTargetPtr == NULL);
    NT_ASSERT(targetPtr->CurrentRequestPtr == NULL);

    deviceCtxPtr->CurrentTargetPtr = targetPtr;
    targetPtr->CurrentRequestPtr = requestPtr;

    // Configure controller and kick-off read or write.

    PbcRequestDoTransfer(deviceCtxPtr, requestPtr);

    WdfSpinLockRelease(deviceCtxPtr->Lock);

exit:

    if (!NT_SUCCESS(status)) {

        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
            "Error configuring sequence, completing "
            "SPBREQUEST %p synchronously - Err=%Xh",
            requestPtr->SpbRequest,
            status);

        SpbRequestComplete(SpbRequest, status);
    }

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "--OnSequence()");
    return;
}

/*++

  Routine Description:

    This routine preprocesses custom IO requests before the framework
    places them in an IO queue. For requests using the SPB transfer list
    format, it calls SpbRequestCaptureIoOtherTransferList to capture the
    client's buffers.

  Arguments:

    SpbController - a handle to the framework device object
        representing an SPB controller
    SpbRequest - a handle to the SPBREQUEST object

  Return Value:

    None.  The request is either completed or enqueued asynchronously.

--*/
_Use_decl_annotations_
VOID OnOtherInCallerContext(
    WDFDEVICE SpbController,
    WDFREQUEST Request
    )
{
    NTSTATUS status;
    WDF_REQUEST_PARAMETERS fxParams;
    WDF_REQUEST_PARAMETERS_INIT(&fxParams);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnOtherInCallerContext()");

    // Check for custom IOCTLs that this driver handles. If
    // unrecognized mark as STATUS_NOT_SUPPORTED and complete.


    WdfRequestGetParameters(Request, &fxParams);

    if ((fxParams.Type != WdfRequestTypeDeviceControl) &&
        (fxParams.Type != WdfRequestTypeDeviceControlInternal)) {

        status = STATUS_NOT_SUPPORTED;
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "FxRequest %p is of unsupported request type - Err=%Xh",
            Request, status);
        goto exit;
    }

    // Verify the driver supports this DeviceIoContol code,
    // otherwise mark as STATUS_NOT_SUPPORTED and complete.
    //

    // For custom IOCTLs that use the SPB transfer list format
    // (i.e. sequence formatting), call SpbRequestCaptureIoOtherTransferList
    // so that the driver can leverage other SPB DDIs for this request.

    status = SpbRequestCaptureIoOtherTransferList((SPBREQUEST)Request);

    if (!NT_SUCCESS(status)) {

        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "Failed to capture transfer list for custom SpbRequest %p"
            " - Err=%Xh",
            Request, status);
        goto exit;
    }

    // Preprocessing has succeeded, enqueue the request.

    status = WdfDeviceEnqueueRequest(SpbController, Request);

    if (!NT_SUCCESS(status)) {

        goto exit;
    }

exit:

    if (!NT_SUCCESS(status)) {

        WdfRequestComplete(Request, status);
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnOtherInCallerContext()");
    return;
}

/*++

  Routine Description:

    This routine processes custom IO requests that are not natively
    supported by the SPB framework extension. For requests using the
    SPB transfer list format, SpbRequestCaptureIoOtherTransferList
    must have been called in the driver's OnOtherInCallerContext routine.

  Arguments:

    SpbController - a handle to the framework device object
        representing an SPB controller
    SpbTarget - a handle to the SPBTARGET object
    SpbRequest - a handle to the SPBREQUEST object
    OutputBufferLength - the request's output buffer length
    InputBufferLength - the requests input buffer length
    IoControlCode - the device IO control code

  Return Value:

    None.  The request is completed asynchronously.

--*/
_Use_decl_annotations_
VOID OnOther(
    WDFDEVICE SpbController,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest,
    size_t OutputBufferLength,
    size_t InputBufferLength,
    ULONG IoControlCode
    )
{
    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "++OnOther()");

    NTSTATUS status = STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(SpbController);
    UNREFERENCED_PARAMETER(SpbTarget);
    UNREFERENCED_PARAMETER(SpbRequest);
    UNREFERENCED_PARAMETER(OutputBufferLength);
    UNREFERENCED_PARAMETER(InputBufferLength);
    UNREFERENCED_PARAMETER(IoControlCode);

    //
    // Option: the driver may take the following steps
    //
    //    1. Verify this specific DeviceIoContol code is supported,
    //       otherwise mark as STATUS_NOT_SUPPORTED and complete.
    //
    //    2. If this IOCTL uses SPB_TRANSFER_LIST and the driver has
    //       called SpbRequestCaptureIoOtherTransferList previously,
    //       validate the request format. The driver can make use of
    //       SpbRequestGetTransferParameters to retrieve each transfer
    //       descriptor.
    //
    //       If this IOCTL uses some proprietary buffer formating
    //       instead of SPB_TRANSFER_LIST, validate appropriately.
    //
    //    3. Setup the device, target, and request contexts as necessary,
    //       and program the hardware for the transfer.
    //

    SpbRequestComplete(SpbRequest, status);

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "--OnOther()");
    return;
}


/*++

  Routine Description:

    This routine cancels an outstanding request. It
    must synchronize with other driver callbacks.

  Arguments:

    wdfRequest - a handle to the WDFREQUEST object

  Return Value:

    None.  The request is completed with status.

--*/
_Use_decl_annotations_
VOID OnCancel(WDFREQUEST Request)
{
    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "++OnCancel()");

    SPBREQUEST spbRequest = (SPBREQUEST)Request;
    PDEVICE_CONTEXT deviceCtxPtr;
    PPBC_TARGET targetPtr;
    PPBC_REQUEST requestPtr;
    BOOLEAN transferCompleted = FALSE;

    // Get the contexts.

    deviceCtxPtr = GetDeviceContext(SpbRequestGetController(spbRequest));
    targetPtr = GetTargetContext(SpbRequestGetTarget(spbRequest));
    requestPtr = GetRequestContext(spbRequest);

    NT_ASSERT(deviceCtxPtr  != NULL);
    NT_ASSERT(targetPtr  != NULL);
    NT_ASSERT(requestPtr != NULL);

    // Acquire the device lock.

    WdfSpinLockAcquire(deviceCtxPtr->Lock);

    // Make sure the current target and request
    // are valid.

    if (targetPtr != deviceCtxPtr->CurrentTargetPtr) {

        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DRIVER,
            "Cancel callback without a valid current target for WDFDEVICE %p, "
            "this should only occur if SPBREQUEST %p was already completed",
            deviceCtxPtr->WdfDevice,
            spbRequest);

        goto exit;
    }

    if (requestPtr != targetPtr->CurrentRequestPtr) {

        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DRIVER,
            "Cancel callback without a valid current request for SPBTARGET %p, "
            "this should only occur if SPBREQUEST %p was already completed",
            targetPtr->SpbTarget,
            spbRequest);

        goto exit;
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER,
        "Cancel callback with outstanding SPBREQUEST %p, "
        "stop IO and complete it",
        spbRequest);

    // Stop delay timer.

    if(WdfTimerStop(deviceCtxPtr->DelayTimer, FALSE)) {

        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER,
            "Delay timer previously schedule, now stopped");
    }

    //
    // Disable interrupts and clear saved stat for DPC. 
    // Must synchronize with ISR.
    //

    NT_ASSERT(deviceCtxPtr->InterruptObject != NULL);

    if (deviceCtxPtr->UseInterruptMode) {
        WdfInterruptAcquireLock(deviceCtxPtr->InterruptObject);
    }

#if (defined(DBG) || defined(DEBUG))
    ControllerDumpRegisters(deviceCtxPtr);
#endif

    ControllerDisableInterrupts(deviceCtxPtr);
    deviceCtxPtr->InterruptStatus = 0;
    PbcDeviceSetInterruptMask(deviceCtxPtr, 0);

    ControllerCancel(deviceCtxPtr);

    if (deviceCtxPtr->UseInterruptMode) {
        WdfInterruptReleaseLock(deviceCtxPtr->InterruptObject);
    }

#if (defined(DBG) || defined(DEBUG))
    ControllerDumpRegisters(deviceCtxPtr);
#endif

    // Mark request as cancelled and complete.

    requestPtr->Status = STATUS_CANCELLED;

    ControllerCompleteTransfer(deviceCtxPtr, requestPtr, TRUE);
    NT_ASSERT(requestPtr->bIoComplete == TRUE);
    transferCompleted = TRUE;

exit:

    // Release the device lock.

    WdfSpinLockRelease(deviceCtxPtr->Lock);

    // Complete the request. There shouldn't be more IO.
    // This must be done outside of the locked code.

    if (transferCompleted) {

        PbcRequestComplete(requestPtr);
    }

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "--OnCancel()");
    return;
}

/*++

  Routine Description:

    This routine is invoked whenever the driver's delay
    timer expires. It kicks off the transfer for the request.

  Arguments:

    Timer - a handle to a framework timer object

  Return Value:

    None.

--*/
_Use_decl_annotations_
VOID OnDelayTimerExpired(WDFTIMER Timer)
{
    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "++OnDelayTimerExpired()");

    WDFDEVICE fxDevice;
    PDEVICE_CONTEXT deviceCtxPtr;
    PPBC_TARGET targetPtr = NULL;
    PPBC_REQUEST requestPtr = NULL;
    BOOLEAN completeRequest = FALSE;

    fxDevice = (WDFDEVICE) WdfTimerGetParentObject(Timer);
    deviceCtxPtr = GetDeviceContext(fxDevice);

    NT_ASSERT(deviceCtxPtr != NULL);

    // Acquire the device lock.

    WdfSpinLockAcquire(deviceCtxPtr->Lock);

    // Make sure the target and request are still valid.

    targetPtr = deviceCtxPtr->CurrentTargetPtr;

    if (targetPtr == NULL) {

        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
            "Delay timer expired without a valid current target for WDFDEVICE %p, "
            "this should only occur if the request was already completed",
            deviceCtxPtr->WdfDevice);

        goto exit;
    }

    requestPtr = targetPtr->CurrentRequestPtr;

    if (requestPtr == NULL) {

        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
            "Delay timer expired without a valid current request for SPBTARGET %p, "
            "this should only occur if the request was already cancelled",
            targetPtr->SpbTarget);

        goto exit;
    }

    NT_ASSERT(requestPtr->SpbRequest != NULL);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
        "Delay timer expired, ready to configure transfer for WDFDEVICE %p",
        deviceCtxPtr->WdfDevice);

    ControllerConfigureForTransfer(deviceCtxPtr, requestPtr);

    if (requestPtr->bIoComplete) {

        completeRequest = TRUE;
    }

exit:

    // Release the device lock.

    WdfSpinLockRelease(deviceCtxPtr->Lock);

    if (completeRequest) {

        PbcRequestComplete(requestPtr);
    }

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "--OnDelayTimerExpired()");
    return;
}

/*++

  Routine Description:

    This routine gets i2c controller configuration parametrs from registry

  Arguments:

    Driver - a pointer to the WDFDRIVER driver object
    DeviceCtxPtr - a pointer to the PBC device context

  Return Value:

    status of operation.

--*/
_Use_decl_annotations_
NTSTATUS GetI2cConfigValues(
    IMX_I2C_CONFIG_DATA* DriverI2cConfigPtr,
    WDFDEVICE Device)
{
    NTSTATUS status = STATUS_SUCCESS;
    DECLARE_UNICODE_STRING_SIZE(valueName, PARAMATER_NAME_LEN);
    ACPI_EVAL_OUTPUT_BUFFER *dsdBufferPtr = nullptr;
    DEVICE_OBJECT *pdoPtr = WdfDeviceWdmGetPhysicalDevice(Device);
    const ACPI_METHOD_ARGUMENT *devicePropertiesPkgPtr;

    UNREFERENCED_PARAMETER(Device);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "++GetI2cConfigValues()");

    if (NULL == DriverI2cConfigPtr) {

        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "GetI2cConfigValues() - error NULL pointer");
        goto DoneGetParams;
    }

    DriverI2cConfigPtr->PeripheralClock_Hz = 0;
    DriverI2cConfigPtr->ModuleClock_Hz = 0;

    DriverI2cConfigPtr->SdaGlitchFilter_ns = 0;
    DriverI2cConfigPtr->SclGlitchFilter_ns = 0;
    DriverI2cConfigPtr->SdaRisetime_ns = 0;
    DriverI2cConfigPtr->SclRisetime_ns = 0;
    DriverI2cConfigPtr->BusIdle_ns = 0;
    DriverI2cConfigPtr->PinLowTimeout_ns = 0;

    DriverI2cConfigPtr->DelayBeforeStart_us = 0;

    status = AcpiQueryDsd(pdoPtr, &dsdBufferPtr);

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, 
            "AcpiQueryDsd failed with error %!STATUS!", status);
        goto DoneGetParams;
    }

    status = AcpiParseDsdAsDeviceProperties(dsdBufferPtr, &devicePropertiesPkgPtr);

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, 
            "AcpiParseSdsAsDeviceProperties failed with error %!STATUS!", status);
        goto DoneGetParams;
    }

    status = AcpiDevicePropertiesQueryIntegerValue(devicePropertiesPkgPtr,
        "ModuleClockFrequencyHz", &DriverI2cConfigPtr->ModuleClock_Hz);

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, 
            "Failed to query ACPI ModuleClockFrequencyHz value (status = %!STATUS!). ", status);
    }


DoneGetParams:
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "--GetI2cConfigValues()=%Xh", status);

    if (dsdBufferPtr != nullptr) {
        ExFreePool(dsdBufferPtr);
    }
    return status;
}

/////////////////////////////////////////////////
//
// Interrupt handling functions.
//
/////////////////////////////////////////////////

BOOLEAN
OnInterruptIsr(
    _In_  WDFINTERRUPT Interrupt,
    _In_  ULONG        MessageID
)
/*++

  Routine Description:

    This routine responds to interrupts generated by the
    controller. If one is recognized, it queues a DPC for
    processing. The interrupt is acknowledged and subsequent
    interrupts are temporarily disabled.

  Arguments:

    Interrupt - a handle to a framework interrupt object
    MessageID - message number identifying the device's
        hardware interrupt message (if using MSI)

  Return Value:

    TRUE if interrupt recognized.

--*/
{
    BOOLEAN interruptRecognized = FALSE;
    ULONG stat;
    PDEVICE_CONTEXT DeviceCtxPtr = GetDeviceContext(
        WdfInterruptGetDevice(Interrupt));

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnInterruptIsr()");
    UNREFERENCED_PARAMETER(MessageID);

    NT_ASSERT(DeviceCtxPtr != NULL);

    //
    // Queue a DPC if the device's interrupt
    // is enabled and active.
    //

    stat = ControllerGetInterruptStatus(
        DeviceCtxPtr,
        PbcDeviceGetInterruptMask(DeviceCtxPtr));

    if (stat > 0) {
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
            "Interrupt with status 0x%lx for WDFDEVICE %p",
            stat,
            DeviceCtxPtr->WdfDevice);

        //
        // Save the interrupt status and disable all other
        // interrupts for now.  They will be re-enabled
        // in OnInterruptDpc.  Queue the DPC.
        //

        interruptRecognized = TRUE;

        DeviceCtxPtr->InterruptStatus |= (stat);
        ControllerDisableInterrupts(DeviceCtxPtr);

        if (!WdfInterruptQueueDpcForIsr(Interrupt)) {
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
                "Interrupt with status 0x%lx occurred with "
                "DPC already queued for WDFDEVICE %p",
                stat,
                DeviceCtxPtr->WdfDevice);
        }
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnInterruptIsr()");

    return interruptRecognized;
}

VOID
OnInterruptDpc(
    _In_  WDFINTERRUPT Interrupt,
    _In_  WDFOBJECT    WdfDevice
)
/*++

  Routine Description:

    This routine processes interrupts from the controller.
    When finished it reenables interrupts as appropriate.

  Arguments:

    Interrupt - a handle to a framework interrupt object
    WdfDevice - a handle to the framework device object

  Return Value:

    None.

--*/
{
    PDEVICE_CONTEXT DeviceCtxPtr;
    PPBC_TARGET pTarget;
    PPBC_REQUEST pRequest = NULL;
    ULONG stat;
    BOOLEAN bInterruptsProcessed = FALSE;
    BOOLEAN completeRequest = FALSE;

    UNREFERENCED_PARAMETER(Interrupt);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnInterruptDpc()");

    DeviceCtxPtr = GetDeviceContext(WdfDevice);
    NT_ASSERT(DeviceCtxPtr != NULL);

    //
    // Acquire the device lock.
    //

    WdfSpinLockAcquire(DeviceCtxPtr->Lock);

    //
    // Make sure the target and request are
    // still valid.
    //

    pTarget = DeviceCtxPtr->CurrentTargetPtr;

    if (pTarget == NULL) {
        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
            "DPC scheduled without a valid current target for WDFDEVICE %p, "
            "this should only occur if the request was already cancelled",
            DeviceCtxPtr->WdfDevice);

        goto exit;
    }

    pRequest = pTarget->CurrentRequestPtr;

    if (pRequest == NULL) {
        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
            "DPC scheduled without a valid current request for SPBTARGET %p, "
            "this should only occur if the request was already cancelled",
            pTarget->SpbTarget);

        goto exit;
    }

    NT_ASSERT(pRequest->SpbRequest != NULL);

    //
    // Synchronize shared data buffers with ISR.
    // Copy interrupt status and clear shared buffer.
    // If there is a current target and request,
    // a DPC should never occur with interrupt status 0.
    //

    WdfInterruptAcquireLock(Interrupt);

    stat = DeviceCtxPtr->InterruptStatus;
    DeviceCtxPtr->InterruptStatus = 0;

    WdfInterruptReleaseLock(Interrupt);

    if (stat == 0) {
        goto exit;
    }

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE,
        "DPC for interrupt with status 0x%lx for WDFDEVICE %p",
        stat,
        DeviceCtxPtr->WdfDevice);

    //
    // Acknowledge and process interrupts.
    //

    ControllerAcknowledgeInterrupts(DeviceCtxPtr, stat);

    ControllerProcessInterrupts(DeviceCtxPtr, pRequest, stat);
    bInterruptsProcessed = TRUE;
    if (pRequest->bIoComplete) {
        completeRequest = TRUE;
    }

    //
    // Re-enable interrupts if necessary. Synchronize with ISR.
    //

    WdfInterruptAcquireLock(Interrupt);

    ULONG mask = PbcDeviceGetInterruptMask(DeviceCtxPtr);

    if (mask > 0) {
        TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE,
            "Re-enable interrupts with mask 0x%lx for WDFDEVICE %p",
            mask,
            DeviceCtxPtr->WdfDevice);

        ControllerEnableInterrupts(DeviceCtxPtr, mask);
    }

    WdfInterruptReleaseLock(Interrupt);

exit:

    //
    // Release the device lock.
    //

    WdfSpinLockRelease(DeviceCtxPtr->Lock);

    //
    // Complete the request if necessary.
    // This must be done outside of the locked code.
    //

    if (bInterruptsProcessed) {
        if (completeRequest) {
            PbcRequestComplete(pRequest);
        }
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnInterruptDpc()");
}
