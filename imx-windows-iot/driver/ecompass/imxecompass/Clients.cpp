// Copyright (C) Microsoft Corporation, All Rights Reserved.
// Copyright 2022-2023 NXP
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

WDFDEVICE            eCompassDevice::m_Device           = nullptr;
WDFTIMER             eCompassDevice::m_Timer            = nullptr;
WDFIOTARGET          eCompassDevice::m_I2CIoTarget      = nullptr;
WDFWAITLOCK          eCompassDevice::m_I2CWaitLock      = nullptr;
WDFINTERRUPT         eCompassDevice::m_Interrupt        = nullptr;

//
// Sensor Operation
//
BOOLEAN              eCompassDevice::m_PoweredOn        = false;
DATA_RATE            eCompassDevice::m_DataRate         = GetDataRateFromReportInterval(DEFAULT_SENSOR_REPORT_INTERVAL, STANDBY_MODE);
ULONG                eCompassDevice::m_MinimumInterval  = SENSOR_MIN_REPORT_INTERVAL;
BOOLEAN              eCompassDevice::m_WakeEnabled      = false;
SensorMode           eCompassDevice::m_SensorMode       = STANDBY_MODE;

//
//FIFO
//

BOOLEAN             eCompassDevice::m_fifo_enabled = false;
ULONG               eCompassDevice::m_batch_latency = DEFAULT_BATCH_LATENCY;
ULONG               eCompassDevice::m_fifo_size = DEFAULT_BATCH_SAMPLE_COUNT;
UINT32              eCompassDevice::m_accelerometer_max_fifo_samples = SENSOR_FIFORESERVEDSIZE_SAMPLES_ACC;
UINT32              eCompassDevice::m_magnetometer_max_fifo_samples = SENSOR_FIFORESERVEDSIZE_SAMPLES_MAG;

// Return the rate that is just less than the desired report interval
DATA_RATE eCompassDevice::GetDataRateFromReportInterval(_In_ ULONG ReportInterval, _In_ ULONG SensorMode)
{
    DATA_RATE dataRate = SENSOR_SUPPORTED_DATA_RATES_SINGLE[0];
    const DATA_RATE* dataRates = SENSOR_SUPPORTED_DATA_RATES_SINGLE;

    if (SensorMode == HYBRID_MODE)
    {
        dataRates = SENSOR_SUPPORTED_DATA_RATES_HYBRID;
    }

    for (ULONG i = 0; i < SENSOR_SUPPORTED_DATA_RATES_COUNT; i++)
    {
        dataRate = dataRates[i];
        if (dataRate.DataRateInterval <= ReportInterval)
        {
            break;
        }
    }

    return dataRate;
}

//------------------------------------------------------------------------------
// Function: eCompassDevice::OnTimerExpire
//
// This callback is called when interval wait time has expired and driver is ready
// to collect new sample. The callback reads current value, compare value to threshold,
// pushes it up to CLX framework, and schedule next wake up time.
//
// Arguments:
//      Timer: IN: WDF timer object
//
// Return Value:
//      None
//------------------------------------------------------------------------------
VOID
eCompassDevice::OnTimerExpire(
    _In_ WDFTIMER Timer
    )
{
    PeCompassDevice pDevice = nullptr;
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    pDevice = GetContextFromSensorInstance(WdfTimerGetParentObject(Timer));
    if (pDevice == nullptr)
    {
        Status = STATUS_INSUFFICIENT_RESOURCES;
        TraceError("eCompass %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
        goto Exit;
    }

    // Get data and push to clx
    Lock(m_I2CWaitLock);
    Status = pDevice->GetData();
    if (!NT_SUCCESS(Status) && Status != STATUS_DATA_NOT_ACCEPTED)
    {
        TraceError("eCompass %!FUNC! GetData Failed %!STATUS!", Status);
    }
    Unlock(m_I2CWaitLock);

    // Schedule next wake up time
    if (m_MinimumInterval <= m_DataRate.DataRateInterval &&
        FALSE != m_PoweredOn &&
        FALSE != pDevice->m_Started)
    {
        LONGLONG WaitTime = 0;  // in unit of 100ns

        if (pDevice->m_StartTime == 0)
        {
            // in case we fail to get sensor start time, use static wait time
            WaitTime = WDF_REL_TIMEOUT_IN_MS(m_DataRate.DataRateInterval);
        }
        else
        {
            ULONG CurrentTimeMs = 0;

            // dynamically calculate wait time to avoid jitter
            Status = GetPerformanceTime (&CurrentTimeMs);
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! GetPerformanceTime %!STATUS!", Status);
                WaitTime = WDF_REL_TIMEOUT_IN_MS(m_DataRate.DataRateInterval);
            }
            else
            {
                pDevice->m_SampleCount++;
                if (CurrentTimeMs > (pDevice->m_StartTime + (m_DataRate.DataRateInterval * (pDevice->m_SampleCount + 1))))
                {
                    // If we skipped two or more beats, reschedule the timer with a zero due time to catch up on missing samples
                    WaitTime = 0;
                }
                else
                {
                    // Else, just compute the remaining time
                    WaitTime = (pDevice->m_StartTime +
                        (m_DataRate.DataRateInterval * (pDevice->m_SampleCount + 1))) - CurrentTimeMs;
                }

                WaitTime = WDF_REL_TIMEOUT_IN_MS(WaitTime);
            }
        }

        WdfTimerStart(m_Timer, WaitTime);
    }

Exit:

    SENSOR_FunctionExit(Status);
}



//------------------------------------------------------------------------------
// Function: eCompassDevice::OnStart
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
eCompassDevice::OnStart(
    _In_ SENSOROBJECT SensorInstance
    )
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (pDevice == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! Sensor(%p) parameter is invalid. Failed %!STATUS!", static_cast<PVOID>(&SensorInstance), Status);
        goto Exit;
    }

    if (m_PoweredOn == FALSE)
    {
        Status = STATUS_DEVICE_NOT_READY;
        TraceError("eCompass %!FUNC! Sensor is not powered on! %!STATUS!", Status);
        goto Exit;
    }

    Status = pDevice->StartSensor();
    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! StartSensor failed! %!STATUS!", Status);
        goto Exit;
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: eCompassDevice::OnStop
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
eCompassDevice::OnStop(
    _In_ SENSOROBJECT SensorInstance
    )
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (pDevice == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! Sensor(%p) parameter is invalid. Failed %!STATUS!", static_cast<PVOID>(&SensorInstance), Status);
        goto Exit;
    }

    Status = pDevice->StopSensor();
    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! StopSensor failed! %!STATUS!", Status);
        goto Exit;
    }

 Exit:
    SENSOR_FunctionExit(Status);

    return Status;
}

// Services a hardware interrupt.
BOOLEAN eCompassDevice::OnInterruptIsr(
    _In_ WDFINTERRUPT Interrupt,        // Handle to a framework interrupt object
    _In_ ULONG /*MessageID*/)           // If the device is using message-signaled interrupts (MSIs),
                                        // this parameter is the message number that identifies the
                                        // device's hardware interrupt message. Otherwise, this value is 0.
{
    NTSTATUS Status = STATUS_SUCCESS;
    BOOLEAN InterruptRecognized = FALSE;

    SENSOR_FunctionEnter();

    // Read the interrupt source and status register to clear interrupt
    BYTE IntSrcBuffer = 0;
    BYTE StatusBuffer = 0;

    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    Status = I2CSensorReadRegister(m_I2CIoTarget, FXOS8700_INT_SOURCE, &IntSrcBuffer, sizeof(IntSrcBuffer));
    Status = I2CSensorReadRegister(m_I2CIoTarget, 0x00, &StatusBuffer, sizeof(StatusBuffer));
    
    WdfWaitLockRelease(m_I2CWaitLock);

    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", FXOS8700_INT_SOURCE, Status);
        goto Exit;
    }
    else if ((IntSrcBuffer & (FXOS8700_INT_SOURCE_SRC_DRDY_MASK | FXOS8700_INT_SOURCE_SRC_FIFO_MASK)) == 0)
    {
        TraceError("%!FUNC! Interrupt source not recognized");
        goto Exit;
    }
    else
    {
        InterruptRecognized = TRUE;
        BOOLEAN WorkItemQueued = WdfInterruptQueueWorkItemForIsr(Interrupt);
        TraceVerbose("%!FUNC! Work item %s queued for interrupt", WorkItemQueued ? "" : " already");
    }

Exit:

    SENSOR_FunctionExit(Status);
    return InterruptRecognized;
}

// Processes interrupt information that the driver's EvtInterruptIsr callback function has stored.
VOID eCompassDevice::OnInterruptWorkItem(
    _In_ WDFINTERRUPT Interrupt,            // Handle to a framework object
    _In_ WDFOBJECT /*AssociatedObject*/)    // A handle to the framework device object that 
                                            // the driver passed to WdfInterruptCreate.
{
    PeCompassDevice pDevice = nullptr;
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Get the sensor instance
    ULONG Count = SensorInstanceCount;
    Status = SensorsCxDeviceGetSensorList(WdfInterruptGetDevice(Interrupt), SensorInstancesBuffer, &Count);
    if (!NT_SUCCESS(Status) || 0 == Count || NULL == SensorInstancesBuffer)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! SensorsCxDeviceGetSensorList failed %!STATUS!", Status);
        goto Exit;
    }

    // Read the device data
    WdfInterruptAcquireLock(Interrupt);
    if ((m_SensorMode == ACCELEROMETER_ONLY_MODE) || (m_SensorMode == HYBRID_MODE))
    {
        pDevice = GetContextFromSensorInstance(SensorInstancesBuffer[Device_LinearAccelerometer]);
        if (nullptr == pDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("eCompass %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
            goto Exit;
        }

        Status = pDevice->GetData();
        if (!NT_SUCCESS(Status) && STATUS_DATA_NOT_ACCEPTED != Status)
        {
            TraceError("eCompass %!FUNC! GetData failed %!STATUS!", Status);
            goto Exit;
        }
    }

    if ((m_SensorMode == MAGNETOMETER_ONLY_MODE) || (m_SensorMode == HYBRID_MODE))
    {
        pDevice = GetContextFromSensorInstance(SensorInstancesBuffer[Device_Mag]);
        if (nullptr == pDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("eCompass %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
            goto Exit;
        }

        Status = pDevice->GetData();
        if (!NT_SUCCESS(Status) && STATUS_DATA_NOT_ACCEPTED != Status)
        {
            TraceError("eCompass %!FUNC! GetData failed %!STATUS!", Status);
            goto Exit;
        }
    }

Exit:

    WdfInterruptReleaseLock(Interrupt);
    SENSOR_FunctionExit(Status);
}


//------------------------------------------------------------------------------
// Function: eCompassDevice::OnGetSupportedDataFields
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
eCompassDevice::OnGetSupportedDataFields(
    _In_ SENSOROBJECT SensorInstance,
    _Inout_opt_ PSENSOR_PROPERTY_LIST pFields,
    _Out_ PULONG pSize
    )
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice || nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! Invalid parameters! %!STATUS!", Status);
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
            TraceError("eCompass %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            goto Exit;
        }

        // Fill out data
        Status = PropertiesListCopy (pFields, pDevice->m_pSupportedDataFields);
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! PropertiesListCopy failed %!STATUS!", Status);
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
// Function: eCompassDevice::OnGetProperties
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
eCompassDevice::OnGetProperties(
    _In_ SENSOROBJECT SensorInstance,
    _Inout_opt_ PSENSOR_COLLECTION_LIST pProperties,
    _Out_ PULONG pSize
    )
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice || nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! Invalid parameters! %!STATUS!", Status);
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
            TraceError("eCompass %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            goto Exit;
        }

        // Fill out all data
        Status = CollectionsListCopyAndMarshall(pProperties, pDevice->m_pSensorProperties);
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
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
// Function: eCompassDevice::OnGetDataFieldProperties
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
eCompassDevice::OnGetDataFieldProperties(
    _In_ SENSOROBJECT SensorInstance,
    _In_ const PROPERTYKEY *DataField,
    _Inout_opt_ PSENSOR_COLLECTION_LIST pProperties,
    _Out_ PULONG pSize
)
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice || nullptr == pSize || nullptr == DataField)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! Invalid parameters! %!STATUS!", Status);
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
                TraceError("eCompass %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
                goto Exit;
            }

            // Fill out all data
            Status = CollectionsListCopyAndMarshall (pProperties, pDevice->m_pDataFieldProperties);
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
                goto Exit;
            }

            *pSize = CollectionsListGetMarshalledSize(pDevice->m_pDataFieldProperties);
        }
    }
    else
    {
        Status = STATUS_NOT_SUPPORTED;
        TraceError("eCompass %!FUNC! Sensor does NOT have properties for this data field. Failed %!STATUS!", Status);
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
// Function: eCompassDevice::OnGetDataInterval
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
eCompassDevice::OnGetDataInterval(
    _In_ SENSOROBJECT SensorInstance,
    _Out_ PULONG DataRateMs
    )
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (pDevice == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! Invalid parameter!");
        goto Exit;
    }

    if (DataRateMs == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! DataRateMs(%p) parameter is invalid. Failed %!STATUS!", static_cast<PVOID>(DataRateMs), Status);
        goto Exit;
    }

    *DataRateMs = m_DataRate.DataRateInterval;
    TraceInformation("%!FUNC! giving data rate %lu", *DataRateMs);

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: eCompassDevice::OnSetDataInterval
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
eCompassDevice::OnSetDataInterval(
    _In_ SENSOROBJECT SensorInstance,
    _In_ ULONG DataRateMs
    )
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;
    REGISTER_SETTING RegisterSetting = { 0, 0 };

    SENSOR_FunctionEnter();

    if (pDevice == nullptr || DataRateMs == 0)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! Invalid parameter!");
        goto Exit;
    }

    if (pDevice->m_Started)
    {
        pDevice->m_Started = false;

        WdfWaitLockAcquire(m_I2CWaitLock, NULL);

        // Set STANDBY mode
        RegisterSetting = { FXOS8700_CTRL_REG1, 0 };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            WdfWaitLockRelease(m_I2CWaitLock);
            TraceError("eCompass %!FUNC! Failed to disable interrupts. %!STATUS!", Status);
            goto Exit;
        }
        m_DataRate = GetDataRateFromReportInterval(DataRateMs, m_SensorMode);

        //In case FIFO mode is enabled, try to adjust FIFO buffer size to keep same batch latency
        if (m_fifo_enabled)
        {
            FXOS8700_F_SETUP_t F_setupReg = { 0 };
            uint8_t desiredSampleCount = (uint8_t)(m_batch_latency / m_DataRate.DataRateInterval);
            uint8_t sampleCountToSet = min(desiredSampleCount, (uint8_t)m_accelerometer_max_fifo_samples);
            F_setupReg.b.f_mode = FXOS8700_F_SETUP_F_MODE_FIFO_STOP_OVF_VAL;
            F_setupReg.b.f_wmrk = sampleCountToSet;
            RegisterSetting = { FXOS8700_F_SETUP, F_setupReg.w };
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            WdfWaitLockRelease(m_I2CWaitLock);
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                goto Exit;
            }

            m_batch_latency = sampleCountToSet * m_DataRate.DataRateInterval;
            m_fifo_size = sampleCountToSet;
        }
        else
        {
            m_fifo_size = DEFAULT_BATCH_SAMPLE_COUNT;
            m_batch_latency = DEFAULT_BATCH_LATENCY;
        }


        // Update data rate in HW
        pDevice->m_Started = true;
        pDevice->m_FirstSample = true;
        // Set data rate in HW and set ACTIVE MODE
        RegisterSetting = { FXOS8700_CTRL_REG1, m_DataRate.RateCode };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        WdfWaitLockRelease(m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }
    }
    else {
        m_DataRate = GetDataRateFromReportInterval(DataRateMs, m_SensorMode);
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: eCompassDevice::OnGetDataThresholds
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
eCompassDevice::OnGetDataThresholds(
    _In_ SENSOROBJECT SensorInstance,
    _Inout_opt_ PSENSOR_COLLECTION_LIST pThresholds,
    _Out_ PULONG pSize
    )
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice || nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! Invalid parameters!");
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
            TraceError("eCompass %!FUNC! Buffer is too small!");
            goto Exit;
        }

        // Fill out all data
        Status = CollectionsListCopyAndMarshall (pThresholds, pDevice->m_pThresholds);
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
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
// Function: eCompassDevice::OnSetDataThresholds
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
eCompassDevice::OnSetDataThresholds(
    _In_ SENSOROBJECT SensorInstance,
    _In_ PSENSOR_COLLECTION_LIST pThresholds
    )
{
    ULONG Element;
    BOOLEAN IsLocked = FALSE;
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (pDevice == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! Invalid parameter!");
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
            TraceError("eCompass %!FUNC! Sensor does NOT have threshold for this data field. Failed %!STATUS!", Status);
            goto Exit;
        }
    }

    // Update cached threshholds
    Status = pDevice->UpdateCachedThreshold();
    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! UpdateCachedThreshold failed! %!STATUS!", Status);
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
// Function: eCompassDevice::OnIoControl
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
NTSTATUS eCompassDevice::OnIoControl(
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
NTSTATUS eCompassDevice::OnStartHistory(_In_ SENSOROBJECT SensorInstance)
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else
    {
        Status = pDevice->StartHistory();
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to stop keeping history.
NTSTATUS eCompassDevice::OnStopHistory(_In_ SENSOROBJECT SensorInstance)
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else
    {
        Status = pDevice->StopHistory();
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to enable wake
NTSTATUS eCompassDevice::OnEnableWake(_In_ SENSOROBJECT SensorInstance)
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
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
NTSTATUS eCompassDevice::OnDisableWake(_In_ SENSOROBJECT SensorInstance)
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
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
NTSTATUS eCompassDevice::OnClearHistory(_In_ SENSOROBJECT SensorInstance)
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else
    {
        Status = pDevice->ClearHistory();
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to start retrieving history.
NTSTATUS eCompassDevice::OnStartHistoryRetrieval(
    _In_ SENSOROBJECT SensorInstance,
    _Inout_updates_bytes_(HistorySizeInBytes) PSENSOR_COLLECTION_LIST pHistoryBuffer,
    _In_ ULONG HistorySizeInBytes)
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else if (sizeof(SENSOR_COLLECTION_LIST) > HistorySizeInBytes)
    {
        Status = STATUS_BUFFER_TOO_SMALL;
        TraceError("eCompass %!FUNC! HistorySizeInBytes is too small %!STATUS!", Status);
    }
    else
    {
        Status = pDevice->StartHistoryRetrieval(pHistoryBuffer, HistorySizeInBytes);
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to cancel history retrieval.
NTSTATUS eCompassDevice::OnCancelHistoryRetrieval(_In_ SENSOROBJECT SensorInstance, _Out_ PULONG pBytesWritten)
{
    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! GetContextFromSensorInstance failed %!STATUS!", Status);
    }
    else
    {
        Status = pDevice->CancelHistoryRetrieval(pBytesWritten);
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

NTSTATUS eCompassDevice::OnSetBatchLatency(_In_ SENSOROBJECT SensorInstance, _In_ ULONG batchLatencyMs)
{ 
    SENSOR_FunctionEnter();

    PeCompassDevice pDevice = GetContextFromSensorInstance(SensorInstance);
    NTSTATUS Status = STATUS_SUCCESS;

    UINT32 maxFifo;
    REGISTER_SETTING RegisterSetting = { 0, 0 };
    FXOS8700_F_SETUP_t F_setupReg = { 0 };

    if (pDevice == nullptr)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("eCompass %!FUNC! Invalid parameter!");
        goto Exit;
    }

    maxFifo = m_accelerometer_max_fifo_samples;
    if (pDevice->m_Started)
    {
        if (batchLatencyMs == 0) // Disabling FIFO mode
        {
            WdfWaitLockAcquire(m_I2CWaitLock, NULL);

            //Set F_SETUP register to 0 to disable FIFO mode
            F_setupReg.b.f_mode = FXOS8700_F_SETUP_F_MODE_FIFO_DISABLE_VAL;
            F_setupReg.b.f_wmrk = 0;
            RegisterSetting = { FXOS8700_F_SETUP, F_setupReg.w };
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            WdfWaitLockRelease(m_I2CWaitLock);
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                goto Exit;
            }

            m_batch_latency = 0;
            m_fifo_size = DEFAULT_BATCH_SAMPLE_COUNT;
            m_fifo_enabled = false;
        }
        else { // Enabling FIFO mode
            ULONG desiredSampleCount = batchLatencyMs / m_DataRate.DataRateInterval;
            ULONG sampleCountToSet = min(desiredSampleCount, maxFifo);

            WdfWaitLockAcquire(m_I2CWaitLock, NULL);

            //Set watermark to sampleCountToSet
            F_setupReg.b.f_mode = FXOS8700_F_SETUP_F_MODE_FIFO_STOP_OVF_VAL;
            F_setupReg.b.f_wmrk = sampleCountToSet;
            RegisterSetting = { FXOS8700_F_SETUP, F_setupReg.w };
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            WdfWaitLockRelease(m_I2CWaitLock);
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                goto Exit;
            }

            m_batch_latency = sampleCountToSet * m_DataRate.DataRateInterval;
            m_fifo_size = sampleCountToSet;
            m_fifo_enabled = true;
        }
    }
    else {
        if (batchLatencyMs == 0)
        {
            m_fifo_enabled = false;
            m_batch_latency = 0;
        }
        else {
            m_fifo_enabled = true;
            m_batch_latency = batchLatencyMs;
        }
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}