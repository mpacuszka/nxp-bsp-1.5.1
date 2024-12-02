// Copyright (C) Microsoft Corporation, All Rights Reserved.
// Copyright 2023 NXP
// 
// Abstract:
//
//  This module contains the implementation of WDF callback functions
//  for IMU driver.
//
// Environment:
//
//  Windows User-Mode Driver Framework (WUDF)

#include "Device.h"
#include "Driver.h"

#include <new.h>

#include "Device.tmh"

// Get the HW resource from the ACPI, then configure and store the IoTarget
NTSTATUS IMUDevice::ConfigureIoTarget(
    _In_ WDFCMRESLIST ResourcesRaw,         // Supplies a handle to a collection of framework resource
                                            // objects. This collection identifies the raw (bus-relative) hardware
                                            // resources that have been assigned to the device.
    _In_ WDFCMRESLIST ResourcesTranslated)  // Supplies a handle to a collection of framework
                                            // resource objects. This collection identifies the translated
                                            // (system-physical) hardware resources that have been assigned to the
                                            // device. The resources appear from the CPU's point of view.
{
    NTSTATUS Status = STATUS_SUCCESS;
    ULONG I2CConnectionResourceCount = 0;
    LARGE_INTEGER I2CConnId = {};

    DECLARE_UNICODE_STRING_SIZE(deviceName, RESOURCE_HUB_PATH_SIZE);

    SENSOR_FunctionEnter();

    // Get hardware resource from ACPI and set up IO target
    ULONG ResourceCount = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < ResourceCount; i++)
    {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR DescriptorRaw = WdfCmResourceListGetDescriptor(ResourcesRaw, i);
        PCM_PARTIAL_RESOURCE_DESCRIPTOR Descriptor = WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        switch (Descriptor->Type)
        {
            // Check we have I2C bus assigned in ACPI
        case CmResourceTypeConnection:
            TraceInformation("IMU %!FUNC! I2C resource found.");
            if (Descriptor->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL &&
                Descriptor->u.Connection.Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)
            {
                I2CConnId.LowPart = Descriptor->u.Connection.IdLowPart;
                I2CConnId.HighPart = Descriptor->u.Connection.IdHighPart;
                I2CConnectionResourceCount++;
            }
            break;

            // Check we have an interrupt assigned in ACPI and create interrupt
        case CmResourceTypeInterrupt:
            TraceInformation("IMU %!FUNC! GPIO interrupt resource found.");
            WDF_INTERRUPT_CONFIG InterruptConfig;
            WDF_INTERRUPT_CONFIG_INIT(&InterruptConfig, OnInterruptIsr, NULL);
            InterruptConfig.InterruptRaw = DescriptorRaw;
            InterruptConfig.InterruptTranslated = Descriptor;
            // Configure an interrupt work item which runs at IRQL = PASSIVE_LEVEL
            // Note: to configure to run at IRQL = DISPATCH_LEVEL, set up an InterruptDpc instead of an InterruptWorkItem
            InterruptConfig.EvtInterruptWorkItem = OnInterruptWorkItem;
            InterruptConfig.PassiveHandling = true;
            Status = WdfInterruptCreate(m_Device, &InterruptConfig, WDF_NO_OBJECT_ATTRIBUTES, &m_Interrupt);
            if (!NT_SUCCESS(Status))
            {
                TraceError("IMU %!FUNC! WdfInterruptCreate failed %!STATUS!", Status);
            }
            break;

        default:
            break;
        }
    }

    if (NT_SUCCESS(Status) && I2CConnectionResourceCount != 1)
    {
        Status = STATUS_UNSUCCESSFUL;
        TraceError("IMU %!FUNC! Did not find I2C resource! %!STATUS!", Status);
    }

    // Set up I2C I/O target. Issued with I2C R/W transfers
    if (NT_SUCCESS(Status))
    {
        m_I2CIoTarget = NULL;
        Status = WdfIoTargetCreate(m_Device, WDF_NO_OBJECT_ATTRIBUTES, &m_I2CIoTarget);

        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! WdfIoTargetCreate failed! %!STATUS!", Status);
        }
    }

    // Setup Target string (\\\\.\\RESOURCE_HUB\\<ConnID from ResHub>
    if (NT_SUCCESS(Status))
    {
        Status = StringCbPrintfW(deviceName.Buffer, RESOURCE_HUB_PATH_SIZE, L"%s\\%0*I64x", RESOURCE_HUB_DEVICE_NAME, static_cast<unsigned int>(sizeof(LARGE_INTEGER) * 2), I2CConnId.QuadPart);
        deviceName.Length = _countof(deviceName_buffer);

        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! RESOURCE_HUB_CREATE_PATH_FROM_ID failed!");
        }
    }

    // Connect to I2C target
    if (NT_SUCCESS(Status))
    {
        WDF_IO_TARGET_OPEN_PARAMS OpenParams;
        WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME(&OpenParams, &deviceName, FILE_ALL_ACCESS);

        Status = WdfIoTargetOpen(m_I2CIoTarget, &OpenParams);
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! WdfIoTargetOpen failed! %!STATUS!", Status);
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

inline size_t GetDeviceSizeAtIndex(
    _In_ ULONG Index)
{
    size_t result = 0;
    switch (static_cast<Device>(Index))
    {
        case Device_LinearAccelerometer:    result = sizeof(LinearAccelerometerDevice); break;
        case Device_Gyr:                    result = sizeof(GyrDevice); break;
        default: break; // invalid
    }
    return result;
}

void AllocateDeviceAtIndex(
    _In_ ULONG Index,
    _Inout_ PIMUDevice* ppDevice
    )
{
    switch (static_cast<Device>(Index))
    {
        case Device_LinearAccelerometer:    *ppDevice = new(*ppDevice) LinearAccelerometerDevice; break;
        case Device_Gyr:                    *ppDevice = new(*ppDevice) GyrDevice; break;

        default: break; // invalid (let driver fail)
    }
}

//------------------------------------------------------------------------------
//
// Function: OnDeviceAdd
//
// This routine is the AddDevice entry point for the  IMU client
// driver. This routine is called by the framework in response to AddDevice
// call from the PnP manager. It will create and initialize the device object
// to represent a new instance of the sensor client.
//
// Arguments:
//      Driver: IN: Supplies a handle to the driver object created in DriverEntry
//      DeviceInit: IN: Supplies a pointer to a framework-allocated WDFDEVICE_INIT structure
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
OnDeviceAdd(
    _In_ WDFDRIVER /*Driver*/,
    _Inout_ PWDFDEVICE_INIT pDeviceInit
    )
{
    WDF_PNPPOWER_EVENT_CALLBACKS Callbacks;
    WDFDEVICE Device;
    WDF_OBJECT_ATTRIBUTES FdoAttributes;
    ULONG Flag = 0;
    SENSOR_CONTROLLER_CONFIG SensorConfig;
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    WDF_OBJECT_ATTRIBUTES_INIT(&FdoAttributes);

    //
    // Initialize FDO attributes and set up file object with sensor extension
    //
    Status = SensorsCxDeviceInitConfig(pDeviceInit, &FdoAttributes, Flag);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! SensorsCxDeviceInitConfig failed %!STATUS!", Status);
        goto Exit;
    }

    //
    // Register the PnP callbacks with the framework.
    //
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&Callbacks);
    Callbacks.EvtDevicePrepareHardware = OnPrepareHardware;
    Callbacks.EvtDeviceReleaseHardware = OnReleaseHardware;
    Callbacks.EvtDeviceD0Entry = OnD0Entry;
    Callbacks.EvtDeviceD0Exit = OnD0Exit;

    WdfDeviceInitSetPnpPowerEventCallbacks(pDeviceInit, &Callbacks);

    //
    // Call the framework to create the device
    //
    Status = WdfDeviceCreate(&pDeviceInit, &FdoAttributes, &Device);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! WdfDeviceCreate failed %!STATUS!", Status);
        goto Exit;
    }

    //
    // Register CLX callback function pointers
    //
    SENSOR_CONTROLLER_CONFIG_INIT(&SensorConfig);
    SensorConfig.DriverIsPowerPolicyOwner = WdfUseDefault;

    SensorConfig.EvtSensorStart                     = IMUDevice::OnStart;
    SensorConfig.EvtSensorStop                      = IMUDevice::OnStop;
    SensorConfig.EvtSensorGetSupportedDataFields    = IMUDevice::OnGetSupportedDataFields;
    SensorConfig.EvtSensorGetDataInterval           = IMUDevice::OnGetDataInterval;
    SensorConfig.EvtSensorSetDataInterval           = IMUDevice::OnSetDataInterval;
    SensorConfig.EvtSensorGetDataFieldProperties    = IMUDevice::OnGetDataFieldProperties;
    SensorConfig.EvtSensorGetDataThresholds         = IMUDevice::OnGetDataThresholds;
    SensorConfig.EvtSensorSetDataThresholds         = IMUDevice::OnSetDataThresholds;
    SensorConfig.EvtSensorGetProperties             = IMUDevice::OnGetProperties;
    SensorConfig.EvtSensorDeviceIoControl           = IMUDevice::OnIoControl;
    SensorConfig.EvtSensorStartHistory              = IMUDevice::OnStartHistory;
    SensorConfig.EvtSensorStopHistory               = IMUDevice::OnStopHistory;
    SensorConfig.EvtSensorClearHistory              = IMUDevice::OnClearHistory;
    SensorConfig.EvtSensorStartHistoryRetrieval     = IMUDevice::OnStartHistoryRetrieval;
    SensorConfig.EvtSensorCancelHistoryRetrieval    = IMUDevice::OnCancelHistoryRetrieval;
    SensorConfig.EvtSensorEnableWake                = IMUDevice::OnEnableWake;
    SensorConfig.EvtSensorDisableWake               = IMUDevice::OnDisableWake;
    SensorConfig.EvtSensorSetBatchLatency           = IMUDevice::OnSetBatchLatency;

    //
    // Set up power capabilities and IO queues
    //
    Status = SensorsCxDeviceInitialize(Device, &SensorConfig);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! SensorDeviceInitialize failed %!STATUS!", Status);
        goto Exit;
    }

    // Ensure device is disable-able
    // By default, devices enumerated by ACPI are not disable-able
    // Our accelerometer is enumerated by the ACPI so we must
    // explicitly make it disable-able.
    if (NT_SUCCESS(Status))
    {
        WDF_DEVICE_STATE DeviceState;
        WDF_DEVICE_STATE_INIT(&DeviceState);
        DeviceState.NotDisableable = WdfFalse;
        WdfDeviceSetDeviceState(Device, &DeviceState);
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}


//------------------------------------------------------------------------------
//
// Function: OnPrepareHardware
//
// This routine is called by the framework when the PnP manager sends an
// IRP_MN_START_DEVICE request to the driver stack. This routine is
// responsible for performing operations that are necessary to make the
// driver's device operational (for e.g. mapping the hardware resources
// into memory).
//
// Argument:
//      Device: IN: Supplies a handle to the framework device object
//      ResourcesRaw: IN: Supplies a handle to a collection of framework resource
//          objects. This collection identifies the raw (bus-relative) hardware
//          resources that have been assigned to the device.
//      ResourcesTranslated: IN: Supplies a handle to a collection of framework
//          resource objects. This collection identifies the translated
//          (system-physical) hardware resources that have been assigned to the
//          device. The resources appear from the CPU's point of view.
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
OnPrepareHardware(
    _In_ WDFDEVICE Device,
    _In_ WDFCMRESLIST ResourcesRaw,
    _In_ WDFCMRESLIST ResourcesTranslated
    )
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    PIMUDevice pDevice = nullptr;

    for (ULONG Count = 0; Count < SensorInstanceCount; Count++)
    {  

        WDF_OBJECT_ATTRIBUTES SensorAttr;
        SENSOR_CONFIG SensorConfig;
        SENSOROBJECT SensorInstance;

        // Create WDFOBJECT for the sensor
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&SensorAttr, IMUDevice);
        SensorAttr.ContextSizeOverride = GetDeviceSizeAtIndex(Count);

        // Register sensor instance with clx
        Status = SensorsCxSensorCreate(Device, &SensorAttr, &SensorInstance);
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! SensorsCxSensorCreate failed %!STATUS!", Status);
            goto Exit;
        }

        pDevice = GetContextFromSensorInstance(SensorInstance);
        if (nullptr == pDevice)
        {
            Status = STATUS_INSUFFICIENT_RESOURCES;
            TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
            goto Exit;
        }

        AllocateDeviceAtIndex(Count, &pDevice);

        // Fill out properties
        Status = pDevice->Initialize(Device, SensorInstance);
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! Initialize device object failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_CONFIG_INIT(&SensorConfig);
        SensorConfig.pEnumerationList = pDevice->m_pEnumerationProperties;
        Status = SensorsCxSensorInitialize(SensorInstance, &SensorConfig);
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! SensorsCxSensorInitialize failed %!STATUS!", Status);
            goto Exit;
        }       
    }

    //
    // Create Lock
    //
    Status = WdfWaitLockCreate(WDF_NO_OBJECT_ATTRIBUTES, &pDevice->m_I2CWaitLock);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! LAC WdfWaitLockCreate failed %!STATUS!", Status);
        goto Exit;
    }

    // ACPI and IoTarget configuration
    Status = IMUDevice::ConfigureIoTarget(ResourcesRaw, ResourcesTranslated);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! Failed to configure IoTarget %!STATUS!", Status);
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
//
// Function: OnReleaseHardware
//
// This routine is called by the framework when the PnP manager is revoking
// ownership of our resources. This may be in response to either
// IRP_MN_STOP_DEVICE or IRP_MN_REMOVE_DEVICE. This routine is responsible for
// performing cleanup of resources allocated in PrepareHardware callback.
// This callback is invoked before passing  the request down to the lower driver.
// This routine will also be invoked by the framework if the prepare hardware
// callback returns a failure.
//
// Argument:
//      Device: IN: Supplies a handle to the framework device object
//      ResourcesTranslated: IN: Supplies a handle to a collection of framework
//          resource objects. This collection identifies the translated
//          (system-physical) hardware resources that have been assigned to the
//          device. The resources appear from the CPU's point of view.
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
OnReleaseHardware(
    _In_ WDFDEVICE Device,
    _In_ WDFCMRESLIST /*ResourcesTranslated*/
    )
{
    ULONG Count = SensorInstanceCount;
    PIMUDevice pDevice = nullptr;
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    Status = SensorsCxDeviceGetSensorList(Device, SensorInstancesBuffer, &Count);
    if (!NT_SUCCESS(Status))
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! SensorsCxDeviceGetSensorList failed %!STATUS!", Status);
        goto Exit;
    }

    for (Count = 0; Count < SensorInstanceCount; Count++)
    {
        pDevice = GetContextFromSensorInstance(SensorInstancesBuffer[Count]);
        if (nullptr == pDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
            goto Exit;
        }

        // Delete sensor instance
        if (NULL != pDevice->m_SensorInstance)
        {
            WdfObjectDelete(pDevice->m_SensorInstance);
            // The pDevice context created using WdfMemoryCreate and parented to m_SensorInstance is automatically
            // destroyed when m_SensorInstance is deleted. pDevice is therefore no longer accessible beyond the above call to WdfObjectDelete.
            // We can therefore not set the m_SensorInstance member back to NULL. We instead set pDevice to nullptr.
            pDevice = nullptr;
        }
    }

    // Delete lock
    if (NULL != pDevice->m_I2CWaitLock)
    {
        WdfObjectDelete(pDevice->m_I2CWaitLock);
        pDevice->m_I2CWaitLock = NULL;
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
//
// Function: OnD0Entry
//
// This routine is invoked by the framework to program the device to goto
// D0, which is the working state. The framework invokes callback every
// time the hardware needs to be (re-)initialized.  This includes after
// IRP_MN_START_DEVICE, IRP_MN_CANCEL_STOP_DEVICE, IRP_MN_CANCEL_REMOVE_DEVICE,
// and IRP_MN_SET_POWER-D0.
//
// Argument:
//      Device: IN: Supplies a handle to the framework device object
//      PreviousState: IN: WDF_POWER_DEVICE_STATE-typed enumerator that identifies
//          the device power state that the device was in before this transition to D0
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
OnD0Entry(
    _In_ WDFDEVICE Device,
    _In_ WDF_POWER_DEVICE_STATE /*PreviousState*/
    )
{
    SENSOR_FunctionEnter();

    ULONG Count = SensorInstanceCount;
    PIMUDevice pDevice = nullptr;
    NTSTATUS Status = STATUS_SUCCESS;

    // Get the sensor instance
    
    Status = SensorsCxDeviceGetSensorList(Device, SensorInstancesBuffer, &Count);
    if (!NT_SUCCESS(Status) || (0 == SensorInstanceCount))
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! SensorsCxDeviceGetSensorList failed %!STATUS!", Status);
        goto Exit;
    }

    //
    // Powering on sensor
    //
    pDevice = GetContextFromSensorInstance(SensorInstancesBuffer[Device_LinearAccelerometer]);
    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
        goto Exit;
    }

    Status = IMUDevice::PowerOn();

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
//
// Function: OnD0Exit
//
// This routine is invoked by the framework to program the device to go into
// a certain Dx state. The framework invokes callback every the the device is
// leaving the D0 state, which happens when the device is stopped, when it is
// removed, and when it is powered off.
//
// Argument:
//      Device: IN: Supplies a handle to the framework device object
//      TargetState: IN: Supplies the device power state which the device will be put
//          in once the callback is complete
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
OnD0Exit(
    _In_ WDFDEVICE Device,
    _In_ WDF_POWER_DEVICE_STATE /*TargetState*/
    )
{  
    NTSTATUS Status = STATUS_SUCCESS;
    ULONG Count = SensorInstanceCount;
    //PIMUDevice pDevice = nullptr;

    SENSOR_FunctionEnter();

    //
    // Get sensor instances
    //
    Status = SensorsCxDeviceGetSensorList(Device, SensorInstancesBuffer, &Count);
    if (!NT_SUCCESS(Status))
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! SensorsCxDeviceGetSensorList failed %!STATUS!", Status);
        goto Exit;
    }

    Status = IMUDevice::PowerOff();

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}

// Write the default device configuration to the device
NTSTATUS IMUDevice::PowerOn()
{
    ULONG Count = SensorInstanceCount;
    PIMUDevice pDevice = nullptr;
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    WdfWaitLockAcquire(m_I2CWaitLock, NULL);

    for (DWORD i = 0; i < ARRAYSIZE(g_ConfigurationSettings); i++)
    {
        REGISTER_SETTING setting = g_ConfigurationSettings[i];
        Status = I2CSensorWriteRegister(m_I2CIoTarget, setting.Register, &setting.Value, sizeof(setting.Value));
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! I2CSensorWriteRegister from 0x%02x failed! %!STATUS!", setting.Register, Status);
            WdfWaitLockRelease(m_I2CWaitLock);
            return Status;
        }
    }
    WdfWaitLockRelease(m_I2CWaitLock);

    for (Count = 0; Count < SensorInstanceCount; Count++)
    {
        pDevice = GetContextFromSensorInstance(SensorInstancesBuffer[Count]);
        if (nullptr == pDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
            goto Exit;
        }

        InitPropVariantFromUInt32(SensorState_Idle, &(pDevice->m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Value));
    }
    
    m_PoweredOn = true;

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}

NTSTATUS IMUDevice::PowerOff()
{
    SENSOR_FunctionEnter();
    NTSTATUS Status = STATUS_SUCCESS;
    REGISTER_SETTING RegisterSetting = { 0, 0 };

    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    RegisterSetting = { LSM6DSOX_CTRL1_XL, LSM6DSOX_CTRL1_XL_ODR_POWER_DOWN };
    Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));

    RegisterSetting = { LSM6DSOX_CTRL2_G,  LSM6DSOX_CTRL2_G_ODR_POWER_DOWN };
    Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));

    //Read data from datareg - empty the register to prevent firing data ready interrupt 
    BYTE DataBuffer[LSM6DSOX_DATA_REPORT_SIZE_BYTES]; // Burst-read mode 2x amount of data
    Status = I2CSensorReadRegister(m_I2CIoTarget, LSM6DSOX_OUTX_L_A, &DataBuffer[0], sizeof(DataBuffer));
    //end temp

    WdfWaitLockRelease(m_I2CWaitLock);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! Failed to put device into power-down mode %!STATUS!", Status);
        goto Exit;
    }
    m_PoweredOn = false;

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}
