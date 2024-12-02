// Copyright (C) Microsoft Corporation, All Rights Reserved.
// Copyright 2023 NXP
// 
// Abstract:
//
//  This module contains the implementation of driver callback function
//  from clx to clients.
//
// Environment:
//
//  Windows User-Mode Driver Framework (WUDF)

#include "Device.h"

#include "Clients.tmh"

static const UINT SYSTEM_TICK_COUNT_1MS = 1; // 1ms

WDFDEVICE            IMUDevice::m_Device           = nullptr;
WDFIOTARGET          IMUDevice::m_I2CIoTarget      = nullptr;
WDFWAITLOCK          IMUDevice::m_I2CWaitLock      = nullptr;
WDFINTERRUPT         IMUDevice::m_Interrupt        = nullptr;

//
// Sensor Operation
//
BOOLEAN              IMUDevice::m_PoweredOn        = false;
ULONG                IMUDevice::m_MinimumInterval  = SENSOR_MIN_REPORT_INTERVAL;
BOOLEAN              IMUDevice::m_WakeEnabled      = false;

//------------------------------------------------------------------------------
// Function: IMUDevice::OnStart
//
// Called by Sensor CLX to begin continously sampling the sensor.
//
// Arguments:
//      SensorInstance: IN: sensor device object
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
IMUDevice::OnStart(
    _In_ SENSOROBJECT SensorInstance
    )
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (pDevice == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! Sensor(%p) parameter is invalid. Failed %!STATUS!", static_cast<PVOID>(&SensorInstance), Status);
        goto Exit;
    }

    if (m_PoweredOn == FALSE)
    {
        Status = STATUS_DEVICE_NOT_READY;
        TraceError("IMU %!FUNC! Sensor is not powered on! %!STATUS!", Status);
        goto Exit;
    }

    Status = pDevice->StartSensor();
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! StartSensor failed! %!STATUS!", Status);
        goto Exit;
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: IMUDevice::OnStop
//
// Called by Sensor CLX to stop continously sampling the sensor.
//
// Arguments:
//      SensorInstance: IN: sensor device object
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
IMUDevice::OnStop(
    _In_ SENSOROBJECT SensorInstance
    )
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (pDevice == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! Sensor(%p) parameter is invalid. Failed %!STATUS!", static_cast<PVOID>(&SensorInstance), Status);
        goto Exit;
    }

    Status = pDevice->StopSensor();
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! StopSensor failed! %!STATUS!", Status);
        goto Exit;
    }

 Exit:
    SENSOR_FunctionExit(Status);

    return Status;
}

// Services a hardware interrupt.
BOOLEAN IMUDevice::OnInterruptIsr(
    _In_ WDFINTERRUPT Interrupt,        // Handle to a framework interrupt object
    _In_ ULONG /*MessageID*/)           // If the device is using message-signaled interrupts (MSIs),
                                        // this parameter is the message number that identifies the
                                        // device's hardware interrupt message. Otherwise, this value is 0.
{
    NTSTATUS Status = STATUS_SUCCESS;
    BOOLEAN InterruptRecognized = FALSE;
    PIMUDevice pDevice = nullptr;

    SENSOR_FunctionEnter();

    // Get the sensor instance
    ULONG Count = SensorInstanceCount;
    Status = SensorsCxDeviceGetSensorList(WdfInterruptGetDevice(Interrupt), SensorInstancesBuffer, &Count);
    if (!NT_SUCCESS(Status) || 0 == Count || NULL == SensorInstancesBuffer)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! SensorsCxDeviceGetSensorList failed %!STATUS!", Status);
        goto Exit;
    }

    // Read the interrupt source
    BYTE IntSrcBuffer = 0;
    BYTE FStatus2 = 0;

    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    Status = I2CSensorReadRegister(m_I2CIoTarget, LSM6DSOX_STATUS_REG, &IntSrcBuffer, sizeof(IntSrcBuffer));
    Status = I2CSensorReadRegister(m_I2CIoTarget, LSM6DSOX_FIFO_STATUS2, &FStatus2, sizeof(FStatus2));
    WdfWaitLockRelease(m_I2CWaitLock);
    
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", LSM6DSOX_STATUS_REG, Status);
        goto Exit;
    }
    if ((IntSrcBuffer & (LSM6DSOX_STATUS_REG_XLDA_MASK | LSM6DSOX_STATUS_REG_GDA_MASK )) == 0 && (FStatus2 & LSM6DSOX_FIFO_STATUS2_FIFO_WTM_IA) == 0)
    {
        TraceError("%!FUNC! Interrupt source not recognized");
        goto Exit;        
    }

    if (IntSrcBuffer & LSM6DSOX_STATUS_REG_XLDA_MASK || FStatus2 & LSM6DSOX_FIFO_STATUS2_FIFO_WTM_IA)
    {        
        pDevice = GetContextFromSensorInstance(SensorInstancesBuffer[Device_LinearAccelerometer]);
        if (nullptr == pDevice)
        { 
            Status = STATUS_INVALID_PARAMETER;
            TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
            goto Exit;
        }
        if ((IntSrcBuffer & LSM6DSOX_STATUS_REG_XLDA_MASK && !pDevice->m_fifo_enabled) || (FStatus2 & LSM6DSOX_FIFO_STATUS2_FIFO_WTM_IA && pDevice->m_fifo_enabled))
        {
            pDevice->m_DataReady = true;
            InterruptRecognized = TRUE;
            BOOLEAN WorkItemQueued = WdfInterruptQueueWorkItemForIsr(Interrupt);
            TraceVerbose("%!FUNC! Work item %s queued for interrupt", WorkItemQueued ? "" : " already");
        }        
    }

    if ( IntSrcBuffer & LSM6DSOX_STATUS_REG_GDA_MASK )
    {
        pDevice = GetContextFromSensorInstance(SensorInstancesBuffer[Device_Gyr]);
        if (nullptr == pDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
            goto Exit;
        }
        pDevice->m_DataReady = true;
        InterruptRecognized = TRUE;
        BOOLEAN WorkItemQueued = WdfInterruptQueueWorkItemForIsr(Interrupt);
        TraceVerbose("%!FUNC! Work item %s queued for interrupt", WorkItemQueued ? "" : " already");
    }

Exit:

    SENSOR_FunctionExit(Status);
    return InterruptRecognized;
}

// Processes interrupt information that the driver's EvtInterruptIsr callback function has stored.
VOID IMUDevice::OnInterruptWorkItem(
    _In_ WDFINTERRUPT Interrupt,            // Handle to a framework object
    _In_ WDFOBJECT /*AssociatedObject*/)    // A handle to the framework device object that 
                                            // the driver passed to WdfInterruptCreate.
{
    PIMUDevice pDevice = nullptr;
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Get the sensor instance
    ULONG Count = SensorInstanceCount;
    Status = SensorsCxDeviceGetSensorList(WdfInterruptGetDevice(Interrupt), SensorInstancesBuffer, &Count);
    if (!NT_SUCCESS(Status) || 0 == Count || NULL == SensorInstancesBuffer)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! SensorsCxDeviceGetSensorList failed %!STATUS!", Status);
        goto Exit;
    }

    // Read the device data
    WdfInterruptAcquireLock(Interrupt);
    for (Count = 0; Count < SensorInstanceCount; Count++)
    {
        pDevice = GetContextFromSensorInstance(SensorInstancesBuffer[Count]);
        if (nullptr == pDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
            goto Exit;
        }

        if (pDevice->m_DataReady)
        {
            Status = pDevice->GetData();
            if (!NT_SUCCESS(Status) && STATUS_DATA_NOT_ACCEPTED != Status)
            {
                TraceError("IMU %!FUNC! GetData failed %!STATUS!", Status);
                goto Exit;
            }
        }
        pDevice->m_DataReady = false;
    }

Exit:

    WdfInterruptReleaseLock(Interrupt);
    SENSOR_FunctionExit(Status);
}


//------------------------------------------------------------------------------
// Function: IMUDevice::OnGetSupportedDataFields
//
// Called by Sensor CLX to get supported data fields. The typical usage is to call
// this function once with buffer pointer as NULL to acquire the required size
// for the buffer, allocate buffer, then call the function again to retrieve
// sensor information.
//
// Arguments:
//      SensorInstance: IN: sensor device object
//      pFields: INOUT_OPT: pointer to a list of supported properties
//      pSize: OUT: number of bytes for the list of supported properties
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
IMUDevice::OnGetSupportedDataFields(
    _In_ SENSOROBJECT SensorInstance,
    _Inout_opt_ PSENSOR_PROPERTY_LIST pFields,
    _Out_ PULONG pSize
    )
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice || nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! Invalid parameters! %!STATUS!", Status);
        goto Exit;
    }

    if (nullptr == pFields)
    {
        // Just return size
        *pSize = pDevice->m_pSupportedDataFields->AllocatedSizeInBytes;
    }
    else
    {
        if (pFields->AllocatedSizeInBytes < pDevice->m_pSupportedDataFields->AllocatedSizeInBytes)
        {
            Status = STATUS_INSUFFICIENT_RESOURCES;
            TraceError("IMU %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            goto Exit;
        }

        // Fill out data
        Status = PropertiesListCopy (pFields, pDevice->m_pSupportedDataFields);
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! PropertiesListCopy failed %!STATUS!", Status);
            goto Exit;
        }

        *pSize = pDevice->m_pSupportedDataFields->AllocatedSizeInBytes;
    }

Exit:
    if (!NT_SUCCESS(Status))
    {
        *pSize = 0;
    }
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: IMUDevice::OnGetProperties
//
// Called by Sensor CLX to get sensor properties. The typical usage is to call
// this function once with buffer pointer as NULL to acquire the required size
// for the buffer, allocate buffer, then call the function again to retrieve
// sensor information.
//
// Arguments:
//      SensorInstance: IN: sensor device object
//      pProperties: INOUT_OPT: pointer to a list of sensor properties
//      pSize: OUT: number of bytes for the list of sensor properties
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
IMUDevice::OnGetProperties(
    _In_ SENSOROBJECT SensorInstance,
    _Inout_opt_ PSENSOR_COLLECTION_LIST pProperties,
    _Out_ PULONG pSize
    )
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice || nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! Invalid parameters! %!STATUS!", Status);
        goto Exit;
    }

    if (nullptr == pProperties)
    {
        // Just return size
        *pSize = CollectionsListGetMarshalledSize(pDevice->m_pSensorProperties);
    }
    else
    {
        if (pProperties->AllocatedSizeInBytes <
            CollectionsListGetMarshalledSize(pDevice->m_pSensorProperties))
        {
            Status = STATUS_INSUFFICIENT_RESOURCES;
            TraceError("IMU %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            goto Exit;
        }

        // Fill out all data
        Status = CollectionsListCopyAndMarshall(pProperties, pDevice->m_pSensorProperties);
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
            goto Exit;
        }

        *pSize = CollectionsListGetMarshalledSize(pDevice->m_pSensorProperties);
    }

Exit:
    if (!NT_SUCCESS(Status))
    {
        *pSize = 0;
    }
    SENSOR_FunctionExit(Status);
    return Status;
}


//------------------------------------------------------------------------------
// Function: IMUDevice::OnGetDataFieldProperties
//
// Called by Sensor CLX to get data field properties. The typical usage is to call
// this function once with buffer pointer as NULL to acquire the required size
// for the buffer, allocate buffer, then call the function again to retrieve
// sensor information.
//
// Arguments:
//      SensorInstance: IN: sensor device object
//      DataField: IN: pointer to the propertykey of requested property
//      pProperties: INOUT_OPT: pointer to a list of sensor properties
//      pSize: OUT: number of bytes for the list of sensor properties
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
IMUDevice::OnGetDataFieldProperties(
    _In_ SENSOROBJECT SensorInstance,
    _In_ const PROPERTYKEY *DataField,
    _Inout_opt_ PSENSOR_COLLECTION_LIST pProperties,
    _Out_ PULONG pSize
)
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice || nullptr == pSize || nullptr == DataField)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! Invalid parameters! %!STATUS!", Status);
        goto Exit;
    }

    if (IsKeyPresentInPropertyList(pDevice->m_pSupportedDataFields, DataField) != FALSE)
    {
        if (nullptr == pProperties)
        {
            // Just return size
            *pSize = CollectionsListGetMarshalledSize(pDevice->m_pDataFieldProperties);
        }
        else
        {
            if (pProperties->AllocatedSizeInBytes <
                CollectionsListGetMarshalledSize(pDevice->m_pDataFieldProperties))
            {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                TraceError("IMU %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
                goto Exit;
            }

            // Fill out all data
            Status = CollectionsListCopyAndMarshall (pProperties, pDevice->m_pDataFieldProperties);
            if (!NT_SUCCESS(Status))
            {
                TraceError("IMU %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
                goto Exit;
            }

            *pSize = CollectionsListGetMarshalledSize(pDevice->m_pDataFieldProperties);
        }
    }
    else
    {
        Status = STATUS_NOT_SUPPORTED;
        TraceError("IMU %!FUNC! Sensor does NOT have properties for this data field. Failed %!STATUS!", Status);
        goto Exit;
    }

Exit:
    if (!NT_SUCCESS(Status))
    {
        *pSize = 0;
    }
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: IMUDevice::OnGetDataInterval
//
// Called by Sensor CLX to get sampling rate of the sensor.
//
// Arguments:
//      SensorInstance: IN: sensor device object
//      DataRateMs: OUT: sampling rate in ms
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
IMUDevice::OnGetDataInterval(
    _In_ SENSOROBJECT SensorInstance,
    _Out_ PULONG DataRateMs
    )
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (pDevice == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! Invalid parameter!");
        goto Exit;
    }

    if (DataRateMs == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! DataRateMs(%p) parameter is invalid. Failed %!STATUS!", static_cast<PVOID>(DataRateMs), Status);
        goto Exit;
    }

    *DataRateMs = pDevice->m_DataRate.DataRateInterval;
    TraceInformation("%!FUNC! giving data rate %lu", *DataRateMs);

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: IMUDevice::OnSetDataInterval
//
// Called by Sensor CLX to set sampling rate of the sensor.
//
// Arguments:
//      SensorInstance: IN: sensor device object
//      DataRateMs: IN: sampling rate in ms
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
IMUDevice::OnSetDataInterval(
    _In_ SENSOROBJECT SensorInstance,
    _In_ ULONG DataRateMs
    )
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (pDevice == nullptr || DataRateMs < SENSOR_MIN_REPORT_INTERVAL)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! Invalid parameter!");
        goto Exit;
    }

    Status = pDevice->UpdateDataInterval(DataRateMs);

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: IMUDevice::OnGetDataThresholds
//
// Called by Sensor CLX to get data thresholds. The typical usage is to call
// this function once with buffer pointer as NULL to acquire the required size
// for the buffer, allocate buffer, then call the function again to retrieve
// sensor information.
//
// Arguments:
//      SensorInstance: IN: sensor device object
//      pThresholds: INOUT_OPT: pointer to a list of sensor thresholds
//      pSize: OUT: number of bytes for the list of sensor thresholds
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
IMUDevice::OnGetDataThresholds(
    _In_ SENSOROBJECT SensorInstance,
    _Inout_opt_ PSENSOR_COLLECTION_LIST pThresholds,
    _Out_ PULONG pSize
    )
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice || nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! Invalid parameters!");
        goto Exit;
    }

    if (nullptr == pThresholds)
    {
        // Just return size
        *pSize = CollectionsListGetMarshalledSize(pDevice->m_pThresholds);
    }
    else
    {
        if (pThresholds->AllocatedSizeInBytes <
            CollectionsListGetMarshalledSize(pDevice->m_pThresholds))
        {
            Status = STATUS_INSUFFICIENT_RESOURCES;
            TraceError("IMU %!FUNC! Buffer is too small!");
            goto Exit;
        }

        // Fill out all data
        Status = CollectionsListCopyAndMarshall (pThresholds, pDevice->m_pThresholds);
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
            goto Exit;
        }

        *pSize = CollectionsListGetMarshalledSize(pDevice->m_pThresholds);
    }

Exit:
    if (!NT_SUCCESS(Status))
    {
        *pSize = 0;
    }
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: IMUDevice::OnSetDataThresholds
//
// Called by Sensor CLX to set data thresholds.
//
// Arguments:
//      SensorInstance: IN: sensor device object
//      pThresholds: IN: pointer to a list of sensor thresholds
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
IMUDevice::OnSetDataThresholds(
    _In_ SENSOROBJECT SensorInstance,
    _In_ PSENSOR_COLLECTION_LIST pThresholds
    )
{
    ULONG Element;
    BOOLEAN IsLocked = FALSE;
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (pDevice == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! Invalid parameter!");
        goto Exit;
    }

    Lock(m_I2CWaitLock);
    IsLocked = TRUE;

    for (Element = 0; Element < pThresholds->Count; Element++)
    {
        Status = PropKeyFindKeySetPropVariant(pDevice->m_pThresholds,
                                              &(pThresholds->List[Element].Key),
                                              TRUE,
                                              &(pThresholds->List[Element].Value));
        if (!NT_SUCCESS(Status))
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("IMU %!FUNC! Sensor does NOT have threshold for this data field. Failed %!STATUS!", Status);
            goto Exit;
        }
    }

    // Update cached threshholds
    Status = pDevice->UpdateCachedThreshold();
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! UpdateCachedThreshold failed! %!STATUS!", Status);
        goto Exit;
    }

Exit:
    if (IsLocked)
    {
        Unlock(m_I2CWaitLock);
        IsLocked = FALSE;
    }
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: IMUDevice::OnIoControl
//
// Called by Sensor CLX to handle IOCTLs that clx does not support
//
// Arguments:
//      SensorInstance: IN: Sensor object
//      Request: IN: WDF request object
//      OutputBufferLength: IN: number of bytes to retrieve from output buffer
//      InputBufferLength: IN: number of bytes to retrieve from input buffer
//      IoControlCode: IN: IOCTL control code
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS IMUDevice::OnIoControl(
    _In_ SENSOROBJECT /*SensorInstance*/, // Sensor object
    _In_ WDFREQUEST /*Request*/,          // WDF request object
    _In_ size_t /*OutputBufferLength*/,   // number of bytes to retrieve from output buffer
    _In_ size_t /*InputBufferLength*/,    // number of bytes to retrieve from input buffer
    _In_ ULONG /*IoControlCode*/          // IOCTL control code
    )
{
    NTSTATUS Status = STATUS_NOT_SUPPORTED;

    SENSOR_FunctionEnter();

    SENSOR_FunctionExit(Status);
    return Status;
}



// Called by Sensor CLX to begin keeping history
NTSTATUS IMUDevice::OnStartHistory(_In_ SENSOROBJECT SensorInstance)
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else
    {
        Status = pDevice->StartHistory();
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to stop keeping history.
NTSTATUS IMUDevice::OnStopHistory(_In_ SENSOROBJECT SensorInstance)
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else
    {
        Status = pDevice->StopHistory();
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to enable wake
NTSTATUS IMUDevice::OnEnableWake(_In_ SENSOROBJECT SensorInstance)
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else
    {
        m_WakeEnabled = TRUE;
        Status = EnableWake();
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to disable wake
NTSTATUS IMUDevice::OnDisableWake(_In_ SENSOROBJECT SensorInstance)
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else
    {
        pDevice->m_WakeEnabled = FALSE;
        Status = pDevice->DisableWake();
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to clear all history stored in the sensor.
NTSTATUS IMUDevice::OnClearHistory(_In_ SENSOROBJECT SensorInstance)
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else
    {
        Status = pDevice->ClearHistory();
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to start retrieving history.
NTSTATUS IMUDevice::OnStartHistoryRetrieval(
    _In_ SENSOROBJECT SensorInstance,
    _Inout_updates_bytes_(HistorySizeInBytes) PSENSOR_COLLECTION_LIST pHistoryBuffer,
    _In_ ULONG HistorySizeInBytes)
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else if (sizeof(SENSOR_COLLECTION_LIST) > HistorySizeInBytes)
    {
        Status = STATUS_BUFFER_TOO_SMALL;
        TraceError("IMU %!FUNC! HistorySizeInBytes is too small %!STATUS!", Status);
    }
    else
    {
        Status = pDevice->StartHistoryRetrieval(pHistoryBuffer, HistorySizeInBytes);
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to cancel history retrieval.
NTSTATUS IMUDevice::OnCancelHistoryRetrieval(_In_ SENSOROBJECT SensorInstance, _Out_ PULONG pBytesWritten)
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else
    {
        Status = pDevice->CancelHistoryRetrieval(pBytesWritten);
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

NTSTATUS IMUDevice::OnSetBatchLatency(_In_ SENSOROBJECT SensorInstance, _In_ ULONG batchLatencyMs)
{
    PIMUDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (pDevice == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("IMU %!FUNC! Invalid parameter!");
        goto Exit;
    }

    Status = pDevice->UpdateBatchLatency(batchLatencyMs);

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}