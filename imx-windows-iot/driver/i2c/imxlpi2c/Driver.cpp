/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2022 NXP
   Licensed under the MIT License.

Module Name:

    driver.c

Abstract:

    This file contains the driver entry points and callbacks.

Environment:

    Kernel-mode Driver Framework

*/

#include "imxlpi2cinternal.h"
#include "imxlpi2cdriver.h"
#include "imxlpi2cdevice.h"
#include "imxlpi2ccontroller.h"
#include "ntstrsafe.h"

#include "driver.tmh"

/*++

Routine Description:
    DriverEntry initializes the driver and is the first routine called by the
    system after the driver is loaded. DriverEntry specifies the other entry
    points in the function driver, such as EvtDevice and DriverUnload.

Parameters Description:

    DriverObject - represents the instance of the function driver that is loaded
    into memory. DriverEntry must initialize members of DriverObject before it
    returns to the caller. DriverObject is allocated by the system before the
    driver is loaded, and it is released by the system after the system unloads
    the function driver from memory.

    RegistryPath - represents the driver specific path in the Registry.
    The function driver can use the path to store driver related data between
    reboots. The path does not store hardware instance specific data.

Return Value:

    STATUS_SUCCESS if successful,
    STATUS_UNSUCCESSFUL otherwise.

--*/
_Use_decl_annotations_
NTSTATUS DriverEntry(
    PDRIVER_OBJECT  DriverObject,
    PUNICODE_STRING RegistryPath
    )
{
    WDF_DRIVER_CONFIG driverConfig;
    WDF_OBJECT_ATTRIBUTES driverAttributes;
    WDFDRIVER fxDriver;
    NTSTATUS status;

    // Initialize WPP Tracing

    WPP_INIT_TRACING( DriverObject, RegistryPath );

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "++DriverEntry() iMX I2C drv");

    WDF_DRIVER_CONFIG_INIT(&driverConfig, OnDeviceAdd);
    driverConfig.DriverPoolTag = IMX_I2C_POOL_TAG;

    WDF_OBJECT_ATTRIBUTES_INIT(&driverAttributes);
    driverAttributes.EvtCleanupCallback = OnDriverCleanup;

    status = WdfDriverCreate(DriverObject,
        RegistryPath,
        &driverAttributes,
        &driverConfig,
        &fxDriver);

    if (!NT_SUCCESS(status)) {

        TraceEvents(TRACE_LEVEL_ERROR,  TRACE_DRIVER,
                    "Error creating WDF driver object - %Xh",
                    status);

        goto DriverEntryEnd;
    };

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DRIVER,
                "Created WDFDRIVER %p",
                fxDriver);

DriverEntryEnd:
    if (!NT_SUCCESS(status)) {
        WPP_CLEANUP(DriverObject);
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "--DriverEntry()=%Xh", status);
    return status;
}

/*++

Routine Description:
    Driver clean routine.

Parameters Description:

    DriverObject - represents the instance of the function driver that was loaded
    into memory.

Return Value:
    None

--*/
_Use_decl_annotations_
VOID OnDriverCleanup(WDFOBJECT Object)
{
    UNREFERENCED_PARAMETER(Object);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "+-OnDriverCleanup()");
    WPP_CLEANUP(Object);
    return;
}


/*++

  Routine Description:

    This routine creates the device object for an SPB
    controller and the device's child objects.

  Arguments:

    FxDriver - the WDF driver object handle
    FxDeviceInit - information about the PDO that we are loading on

  Return Value:

    Status

--*/
_Use_decl_annotations_
NTSTATUS OnDeviceAdd(
    WDFDRIVER FxDriver,
    PWDFDEVICE_INIT FxDeviceInit
)
{
    DEVICE_CONTEXT* pDeviceCtx = NULL;
    NTSTATUS status;
    WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
    IMX_I2C_CONFIG_DATA I2cConfigDataSt = { 0x00 };

    UNREFERENCED_PARAMETER(FxDriver);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "++OnDeviceAdd()");

    // Configure DeviceInit structure
    status = SpbDeviceInitConfig(FxDeviceInit);
    if (!NT_SUCCESS(status)) {

        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
            "Failed SpbDeviceInitConfig() for WDFDEVICE_INIT %p - Err=%Xh",
            FxDeviceInit,
            status);
        goto OnDeviceAddErr;
    }

    // Setup PNP/Power callbacks.
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);

    pnpCallbacks.EvtDevicePrepareHardware = OnPrepareHardware;
    pnpCallbacks.EvtDeviceReleaseHardware = OnReleaseHardware;
    pnpCallbacks.EvtDeviceD0Entry = OnD0Entry;
    pnpCallbacks.EvtDeviceD0Exit = OnD0Exit;
    pnpCallbacks.EvtDeviceSelfManagedIoInit = OnSelfManagedIoInit;
    pnpCallbacks.EvtDeviceSelfManagedIoCleanup = OnSelfManagedIoCleanup;

    WdfDeviceInitSetPnpPowerEventCallbacks(FxDeviceInit, &pnpCallbacks);

    // Note: The SPB class extension sets a default
    //       security descriptor to allow access to
    //       user-mode drivers. This can be overridden
    //       by calling WdfDeviceInitAssignSDDLString()
    //       with the desired setting. This must be done
    //       after calling SpbDeviceInitConfig() but
    //       before WdfDeviceCreate().

    // Create the device.
    {
        WDF_OBJECT_ATTRIBUTES deviceAttributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&deviceAttributes, DEVICE_CONTEXT);
        WDFDEVICE wdfDevice;

        status = WdfDeviceCreate(&FxDeviceInit,
            &deviceAttributes,
            &wdfDevice);

        if (!NT_SUCCESS(status)) {

            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
                "Failed to create WDFDEVICE from WDFDEVICE_INIT %p - Error=%Xh",
                FxDeviceInit,
                status);

            goto OnDeviceAddErr;
        }

        pDeviceCtx = GetDeviceContext(wdfDevice);
        NT_ASSERT(pDeviceCtx != NULL);

        pDeviceCtx->WdfDevice = wdfDevice;

    }


    // obtain i2c device configuration parameters from registry
    if (STATUS_SUCCESS != GetI2cConfigValues(&I2cConfigDataSt, pDeviceCtx->WdfDevice)) {

        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DRIVER,
            "Failed in GetI2cConfigValues() for WDFDEVICE %p",
            pDeviceCtx->WdfDevice);
    }

    // Ensure device is disable-able
    {
        WDF_DEVICE_STATE deviceState;
        WDF_DEVICE_STATE_INIT(&deviceState);

        deviceState.NotDisableable = WdfFalse;
        WdfDeviceSetDeviceState(pDeviceCtx->WdfDevice, &deviceState);
    }

    // Bind a SPB controller object to the device.
    {
        SPB_CONTROLLER_CONFIG spbConfig;
        SPB_CONTROLLER_CONFIG_INIT(&spbConfig);

        // Register for target connect callback.  The driver
        // does not need to respond to target disconnect.

        spbConfig.EvtSpbTargetConnect = OnTargetConnect;

        // Register for SPB IO callbacks.
        // note: an I2C controller driver that operates in subordinate mode
        // should set this member to WdfIoQueueDispatchParallel

        spbConfig.ControllerDispatchType = WdfIoQueueDispatchSequential;

        // iMX i2c controller queues should be power managed

        spbConfig.PowerManaged = WdfTrue;

        // these three callback functions are required.

        spbConfig.EvtSpbIoWrite = OnWrite;
        spbConfig.EvtSpbIoRead = OnRead;
        spbConfig.EvtSpbIoSequence = OnSequence;

        // EvtSpbTargetConnect is optional and can be NULL.
        // EvtSpbTargetDisconnect is optional and can be NULL.

        // lock-unlock callbacks are optional. However, if SPB controller driver
        // that implements an EvtSpbControllerLock function,
        // it must also implement an EvtSpbControllerUnlock function.

        spbConfig.EvtSpbControllerLock = OnControllerLock;
        spbConfig.EvtSpbControllerUnlock = OnControllerUnlock;

        status = SpbDeviceInitialize(pDeviceCtx->WdfDevice, &spbConfig);

        if (!NT_SUCCESS(status)) {

            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
                "Failed SpbDeviceInitialize() for WDFDEVICE %p - Error=%Xh",
                pDeviceCtx->WdfDevice,
                status);

            goto OnDeviceAddErr;
        }

        // Register for IO other callbacks.

        SpbControllerSetIoOtherCallback(pDeviceCtx->WdfDevice,
            OnOther,
            OnOtherInCallerContext);
    }

    // Set target object attributes.

    {
        WDF_OBJECT_ATTRIBUTES targetAttributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&targetAttributes, PBC_TARGET);

        SpbControllerSetTargetAttributes(pDeviceCtx->WdfDevice, &targetAttributes);
    }

    // Set request object attributes.
    {
        WDF_OBJECT_ATTRIBUTES requestAttributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&requestAttributes, PBC_REQUEST);

        // NOTE: When registering for EvtCleanupCallback or
        // EvtDestroyCallback be aware that IO requests arriving in the class
        // extension, but not presented to the driver (due to
        // cancellation), will still have their cleanup and destroy
        // callbacks invoked.

        SpbControllerSetRequestAttributes(pDeviceCtx->WdfDevice, &requestAttributes);
    }

    // Create an interrupt object, interrupt spinlock,
    // and register callbacks.
    {
        // Create the interrupt spinlock.
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = pDeviceCtx->WdfDevice;

        WDFSPINLOCK interruptLock;

        status = WdfSpinLockCreate(
            &attributes,
            &interruptLock);

        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
                "Failed to create interrupt spinlock for WDFDEVICE %p - %!STATUS!",
                pDeviceCtx->WdfDevice,
                status);

            goto OnDeviceAddErr;
        }

        // Create the interrupt object.
        WDF_INTERRUPT_CONFIG interruptConfig;

        WDF_INTERRUPT_CONFIG_INIT(
            &interruptConfig,
            OnInterruptIsr,
            OnInterruptDpc);

        interruptConfig.SpinLock = interruptLock;

        status = WdfInterruptCreate(
            pDeviceCtx->WdfDevice,
            &interruptConfig,
            WDF_NO_OBJECT_ATTRIBUTES,
            &pDeviceCtx->InterruptObject);

        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
                "Failed to create interrupt object for WDFDEVICE %p - %!STATUS!",
                pDeviceCtx->WdfDevice,
                status);

            goto OnDeviceAddErr;
        }
    }
    
    // Create an work item
    {
        WDF_OBJECT_ATTRIBUTES workitemAttributes;
        WDF_WORKITEM_CONFIG  workitemConfig;

        WDF_OBJECT_ATTRIBUTES_INIT(&workitemAttributes);
        workitemAttributes.ParentObject = pDeviceCtx->WdfDevice;
        
        WDF_WORKITEM_CONFIG_INIT(
            &workitemConfig,
            ControllerPollingMain
        );

        status = WdfWorkItemCreate(
            &workitemConfig,
            &workitemAttributes,
            &pDeviceCtx->WorkItem
        );
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
                "Failed to create WorkItem for WDFDEVICE %p - %!STATUS!",
                pDeviceCtx->WdfDevice,
                status);

            goto OnDeviceAddErr;
        }

    }

    // Create the delay timer to stall between transfers.
    {
        WDF_TIMER_CONFIG wdfTimerConfig;
        WDF_OBJECT_ATTRIBUTES timerAttributes;

        WDF_TIMER_CONFIG_INIT(&wdfTimerConfig, OnDelayTimerExpired);
        WDF_OBJECT_ATTRIBUTES_INIT(&timerAttributes);
        timerAttributes.ParentObject = pDeviceCtx->WdfDevice;

        status = WdfTimerCreate(&wdfTimerConfig,
                                &timerAttributes,
                                &(pDeviceCtx->DelayTimer));

        if (!NT_SUCCESS(status)) {

            TraceEvents(TRACE_LEVEL_ERROR, TRACE_CTRLR,
                        "Failed to create delay timer for WDFDEVICE %p - Err=%Xh",
                        pDeviceCtx->WdfDevice,
                        status);

            goto OnDeviceAddErr;
        }
    }

    // Create the spin lock to synchronize access
    // to the controller driver.
    WDF_OBJECT_ATTRIBUTES attributes;
    WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
    attributes.ParentObject = pDeviceCtx->WdfDevice;

    status = WdfSpinLockCreate(&attributes, &pDeviceCtx->Lock);
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
                    "Failed to create device spinlock for WDFDEVICE %p - Err=%Xh",
                    pDeviceCtx->WdfDevice,
                    status);

        goto OnDeviceAddErr;
    }

    // disable idle settings for i2c on iMX if not under PEP control

#ifdef I2C_IS_PEP_MANAGED
    // Configure idle settings to use system
    // managed idle timeout.
    {
        WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS idleSettings;
        WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS_INIT(&idleSettings,
            IdleCannotWakeFromS0);

        // Explicitly set initial idle timeout delay.

        idleSettings.IdleTimeoutType = SystemManagedIdleTimeoutWithHint;
        idleSettings.IdleTimeout = IDLE_TIMEOUT_MONITOR_ON;

        status = WdfDeviceAssignS0IdleSettings(pDeviceCtx->WdfDevice,
            &idleSettings);

        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER,
                "Failed to initalize S0 idle settings for WDFDEVICE %p- Err=%Xh",
                pDeviceCtx->WdfDevice,
                status);

            goto OnDeviceAddErr;
        }
    }
#endif

    pDeviceCtx->ModuleClock_Hz = I2cConfigDataSt.ModuleClock_Hz;
    pDeviceCtx->PeripheralAccessClock_Hz = I2cConfigDataSt.PeripheralClock_Hz;
    pDeviceCtx->SdaGlitchFilter_ns = I2cConfigDataSt.SdaGlitchFilter_ns;
    pDeviceCtx->SclGlitchFilter_ns = I2cConfigDataSt.SclGlitchFilter_ns;
    pDeviceCtx->SdaRisetime_ns = I2cConfigDataSt.SdaRisetime_ns;
    pDeviceCtx->SclRisetime_ns = I2cConfigDataSt.SclRisetime_ns;
    pDeviceCtx->BusIdle_ns = I2cConfigDataSt.BusIdle_ns;
    pDeviceCtx->PinLowTimeout_ns = I2cConfigDataSt.PinLowTimeout_ns;

OnDeviceAddErr:

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "--OnDeviceAdd()=%Xh", status);
    return status;
}