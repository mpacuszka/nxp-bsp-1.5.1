// Copyright (C) Microsoft Corporation, All Rights Reserved.
// Copyright 2022-2023 NXP
// 
// Abstract:
//
//  This module contains the implementation of sensor specific functions.
//
// Environment:
//
//  Windows User-Mode Driver Framework (UMDF)

#include "Device.h"

#include "LinearAccelerometerClient.tmh"

#define SENSORV2_POOL_TAG_LINEAR_ACCELEROMETER            '2CaL'

#define LinearAccelerometerDevice_Default_Threshold       ( 0.1f)
#define LinearAccelerometerDevice_Minimum                 (-4.0f)    // in g
#define LinearAccelerometerDevice_Maximum                 ( 4.0f)     // in g
#define LinearAccelerometerDevice_Precision               ( 16384.0f) // 16384 = 2^14, 14 bit data
#define LinearAccelerometerDevice_Range      \
            (LinearAccelerometerDevice_Maximum - LinearAccelerometerDevice_Minimum)
#define LinearAccelerometerDevice_Resolution      \
            (LinearAccelerometerDevice_Range / LinearAccelerometerDevice_Precision)


// Linear Accelerometer Unique ID
// {2BAAA1A7-6795-42A0-B830-82526CFD28D1}
DEFINE_GUID(GUID_LinearAccelerometerDevice_UniqueID,
    0x2baaa1a7, 0x6795, 0x42a0, 0xb8, 0x30, 0x82, 0x52, 0x6c, 0xfd, 0x28, 0xd1);

// Sensor data
typedef enum
{
    LINEAR_ACCELEROMETER_DATA_X = 0,
    LINEAR_ACCELEROMETER_DATA_Y,
    LINEAR_ACCELEROMETER_DATA_Z,
    LINEAR_ACCELEROMETER_DATA_TIMESTAMP,
    LINEAR_ACCELEROMETER_DATA_COUNT
} LINEAR_ACCELEROMETER_DATA_INDEX;

//------------------------------------------------------------------------------
// Function: Initialize
//
// This routine initializes the sensor to its default properties
//
// Arguments:
//       Device: IN: WDFDEVICE object
//       SensorInstance: IN: SENSOROBJECT for each sensor instance
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
LinearAccelerometerDevice::Initialize(
    _In_ WDFDEVICE Device,
    _In_ SENSOROBJECT SensorInstance
    )
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    //
    // Store device and instance
    //
    m_Device = Device;
    m_SensorInstance = SensorInstance;
    m_Started = FALSE;
    
    //
    // Sensor Enumeration Properties
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;
        ULONG Size = SENSOR_COLLECTION_LIST_SIZE(SENSOR_ENUMERATION_PROPERTIES_COUNT);

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_LINEAR_ACCELEROMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pEnumerationProperties);
        if (!NT_SUCCESS(Status) || m_pEnumerationProperties == nullptr)
        {
            TraceError("eCompass %!FUNC! LAC WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pEnumerationProperties, Size);
        m_pEnumerationProperties->Count = SENSOR_ENUMERATION_PROPERTIES_COUNT;

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_TYPE].Key = DEVPKEY_Sensor_Type;
        InitPropVariantFromCLSID(GUID_SensorType_LinearAccelerometer,
                                 &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_TYPE].Value));

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MANUFACTURER].Key = DEVPKEY_Sensor_Manufacturer;
        InitPropVariantFromString(SENSOR_MANUFACTURER,
                                  &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MANUFACTURER].Value));

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MODEL].Key = DEVPKEY_Sensor_Model;
        InitPropVariantFromString(SENSOR_MODEL,
                                  &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MODEL].Value));

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CONNECTION_TYPE].Key = DEVPKEY_Sensor_ConnectionType;
        // The DEVPKEY_Sensor_ConnectionType values match the SensorConnectionType enumeration
        InitPropVariantFromUInt32(static_cast<ULONG>(SensorConnectionType::Integrated),
                                 &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CONNECTION_TYPE].Value));

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_PERSISTENT_UNIQUE_ID].Key = DEVPKEY_Sensor_PersistentUniqueId;
        InitPropVariantFromCLSID(GUID_LinearAccelerometerDevice_UniqueID,
                                 &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_PERSISTENT_UNIQUE_ID].Value));

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CATEGORY].Key = DEVPKEY_Sensor_Category;
        InitPropVariantFromCLSID(GUID_SensorCategory_Motion,
                                 &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CATEGORY].Value));

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_ISPRIMARY].Key = DEVPKEY_Sensor_IsPrimary;
        InitPropVariantFromBoolean(TRUE,
                                 &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_ISPRIMARY].Value));
    }

    //
    // Supported Data-Fields
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;
        ULONG Size = SENSOR_PROPERTY_LIST_SIZE(LINEAR_ACCELEROMETER_DATA_COUNT);

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_LINEAR_ACCELEROMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pSupportedDataFields);
        if (!NT_SUCCESS(Status) || m_pSupportedDataFields == nullptr)
        {
            TraceError("eCompass %!FUNC! LAC WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_PROPERTY_LIST_INIT(m_pSupportedDataFields, Size);
        m_pSupportedDataFields->Count = LINEAR_ACCELEROMETER_DATA_COUNT;

        m_pSupportedDataFields->List[LINEAR_ACCELEROMETER_DATA_TIMESTAMP] = PKEY_SensorData_Timestamp;
        m_pSupportedDataFields->List[LINEAR_ACCELEROMETER_DATA_X] = PKEY_SensorData_AccelerationX_Gs;
        m_pSupportedDataFields->List[LINEAR_ACCELEROMETER_DATA_Y] = PKEY_SensorData_AccelerationY_Gs;
        m_pSupportedDataFields->List[LINEAR_ACCELEROMETER_DATA_Z] = PKEY_SensorData_AccelerationZ_Gs;    }

    //
    // Data
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;
        ULONG Size = SENSOR_COLLECTION_LIST_SIZE(LINEAR_ACCELEROMETER_DATA_COUNT);
        FILETIME Time = {0};

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_LINEAR_ACCELEROMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pSensorData);
        if (!NT_SUCCESS(Status) || m_pSensorData == nullptr)
        {
            TraceError("eCompass %!FUNC! LAC WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

    SENSOR_COLLECTION_LIST_INIT(m_pSensorData, Size);
    m_pSensorData->Count = LINEAR_ACCELEROMETER_DATA_COUNT;

    m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_TIMESTAMP].Key = PKEY_SensorData_Timestamp;
    GetSystemTimePreciseAsFileTime(&Time);
    InitPropVariantFromFileTime(&Time, &(m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_TIMESTAMP].Value));

    m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_X].Key = PKEY_SensorData_AccelerationX_Gs;
    InitPropVariantFromFloat(0.0, &(m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_X].Value));

    m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_Y].Key = PKEY_SensorData_AccelerationY_Gs;
    InitPropVariantFromFloat(0.0, &(m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_Y].Value));

    m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_Z].Key = PKEY_SensorData_AccelerationZ_Gs;
    InitPropVariantFromFloat(0.0, &(m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_Z].Value));

    m_CachedData.Axis.X = 0.0f;
    m_CachedData.Axis.Y = 0.0f;
    m_CachedData.Axis.Z = -1.0f;

    m_LastSample.Axis.X  = 0.0f;
    m_LastSample.Axis.Y  = 0.0f;
    m_LastSample.Axis.Z  = 0.0f;
    }

    //
    // Sensor Properties
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;
        ULONG Size = SENSOR_COLLECTION_LIST_SIZE(SENSOR_PROPERTY_COUNT);

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_LINEAR_ACCELEROMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pSensorProperties);
        if (!NT_SUCCESS(Status) || m_pSensorProperties == nullptr)
        {
            TraceError("LAC %!FUNC! WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pSensorProperties, Size);
        m_pSensorProperties->Count = SENSOR_PROPERTY_COUNT;

        m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Key = PKEY_Sensor_State;
        InitPropVariantFromUInt32(SensorState_Initializing,
                                  &(m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_MIN_DATA_INTERVAL].Key = PKEY_Sensor_MinimumDataInterval_Ms;
        InitPropVariantFromUInt32(SENSOR_MIN_REPORT_INTERVAL,
                                  &(m_pSensorProperties->List[SENSOR_PROPERTY_MIN_DATA_INTERVAL].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE].Key = PKEY_Sensor_MaximumDataFieldSize_Bytes;
        InitPropVariantFromUInt32(CollectionsListGetMarshalledSize(m_pSensorData),
                                  &(m_pSensorProperties->List[SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_TYPE].Key = PKEY_Sensor_Type;
        InitPropVariantFromCLSID(GUID_SensorType_LinearAccelerometer,
                                     &(m_pSensorProperties->List[SENSOR_PROPERTY_TYPE].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_FIFORESERVEDSIZE_SAMPLES].Key = PKEY_Sensor_FifoReservedSize_Samples;
        InitPropVariantFromUInt32(SENSOR_FIFORESERVEDSIZE_SAMPLES_ACC,
            &(m_pSensorProperties->List[SENSOR_PROPERTY_FIFORESERVEDSIZE_SAMPLES].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_FIFO_MAXSIZE_SAMPLES].Key = PKEY_Sensor_FifoMaxSize_Samples;
        InitPropVariantFromUInt32(SENSOR_FIFO_MAXSIZE_SAMPLES,
            &(m_pSensorProperties->List[SENSOR_PROPERTY_FIFO_MAXSIZE_SAMPLES].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_WAKE_CAPABLE].Key = PKEY_Sensor_WakeCapable;
        InitPropVariantFromBoolean(SENSOR_WAKE_CAPABLE,
            &(m_pSensorProperties->List[SENSOR_PROPERTY_WAKE_CAPABLE].Value));
    }

    //
    // Data field properties
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;
        ULONG Size = SENSOR_COLLECTION_LIST_SIZE(SENSOR_DATA_FIELD_PROPERTY_COUNT);

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_LINEAR_ACCELEROMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pDataFieldProperties);
        if (!NT_SUCCESS(Status) || m_pDataFieldProperties == nullptr)
        {
            TraceError("eCompass %!FUNC! LAC WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pDataFieldProperties, Size);
        m_pDataFieldProperties->Count = SENSOR_DATA_FIELD_PROPERTY_COUNT;

        m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RESOLUTION].Key = PKEY_SensorDataField_Resolution;
        InitPropVariantFromFloat(LinearAccelerometerDevice_Resolution,
                                 &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RESOLUTION].Value));

        m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MIN].Key = PKEY_SensorDataField_RangeMinimum;
        InitPropVariantFromFloat(LinearAccelerometerDevice_Minimum,
                                 &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MIN].Value));

        m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MAX].Key = PKEY_SensorDataField_RangeMaximum;
        InitPropVariantFromFloat(LinearAccelerometerDevice_Maximum,
                                 &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MAX].Value));
    }

    //
    // Set default threshold
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;

        ULONG Size = SENSOR_COLLECTION_LIST_SIZE(LINEAR_ACCELEROMETER_DATA_COUNT - 1);    //  Timestamp does not have thresholds

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_LINEAR_ACCELEROMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pThresholds);
        if (!NT_SUCCESS(Status) || m_pThresholds == nullptr)
        {
            TraceError("eCompass %!FUNC! LAC WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pThresholds, Size);
        m_pThresholds->Count = LINEAR_ACCELEROMETER_DATA_COUNT - 1;

        m_pThresholds->List[LINEAR_ACCELEROMETER_DATA_X].Key = PKEY_SensorData_AccelerationX_Gs;
        InitPropVariantFromFloat(LinearAccelerometerDevice_Default_Threshold,
                                 &(m_pThresholds->List[LINEAR_ACCELEROMETER_DATA_X].Value));

        m_pThresholds->List[LINEAR_ACCELEROMETER_DATA_Y].Key = PKEY_SensorData_AccelerationY_Gs;
        InitPropVariantFromFloat(LinearAccelerometerDevice_Default_Threshold,
                                 &(m_pThresholds->List[LINEAR_ACCELEROMETER_DATA_Y].Value));

        m_pThresholds->List[LINEAR_ACCELEROMETER_DATA_Z].Key = PKEY_SensorData_AccelerationZ_Gs;
        InitPropVariantFromFloat(LinearAccelerometerDevice_Default_Threshold,
                                 &(m_pThresholds->List[LINEAR_ACCELEROMETER_DATA_Z].Value));

        m_CachedThresholds.Axis.X = LinearAccelerometerDevice_Default_Threshold;
        m_CachedThresholds.Axis.Y = LinearAccelerometerDevice_Default_Threshold;
        m_CachedThresholds.Axis.Z = LinearAccelerometerDevice_Default_Threshold;

        m_FirstSample = TRUE;
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}



//------------------------------------------------------------------------------
// Function: GetData
//
// This routine is called by worker thread to read sensor samples, compare threshold
// and push it back to CLX. It simulates hardware thresholding by only generating data
// when the change of data is greater than threshold.
//
// Arguments:
//       None
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
LinearAccelerometerDevice::GetData(
    )
{
    BOOLEAN DataReady = FALSE;
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    FILETIME Timestamp = { 0 };
    ULARGE_INTEGER ulTimeStamp;
    const DWORD sampleInterval = 10000 * m_DataRate.DataRateInterval; //Time between samples in FIFO stack in nanoseconds
    GetSystemTimePreciseAsFileTime(&Timestamp);

    //Since first sample in FIFO buffer is the oldest one, we calculate its correct timestamp by subtracting time interval between samples multiplied by size of FIFO buffer from current timestamp
    ulTimeStamp.LowPart = Timestamp.dwLowDateTime;
    ulTimeStamp.HighPart = Timestamp.dwHighDateTime;
    ulTimeStamp.QuadPart = ulTimeStamp.QuadPart - (ULONGLONG)sampleInterval * (m_fifo_size - 1); 
    Timestamp.dwLowDateTime = ulTimeStamp.LowPart;
    Timestamp.dwHighDateTime = ulTimeStamp.HighPart;

    BYTE DataBuffer[FXOS8700_DATA_REPORT_SIZE_BYTES]; // Burst-read mode 2x amount of data

    //Read the device data - if FIFO is enabled, read whole FIFO buffer
    for (ULONG i = 0; i < m_fifo_size; i++) {

        // Read the device data
        WdfWaitLockAcquire(m_I2CWaitLock, NULL);
        Status = I2CSensorReadRegister(m_I2CIoTarget, FXOS8700_OUT_X_MSB, &DataBuffer[0], sizeof(DataBuffer));
        WdfWaitLockRelease(m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", FXOS8700_OUT_X_MSB, Status);
            goto Exit;
        }
        else
        {
            // Perform data conversion
            SHORT xRaw = static_cast<SHORT>(((DataBuffer[0] << 8) | DataBuffer[1])) >> 2;
            SHORT yRaw = static_cast<SHORT>(((DataBuffer[2] << 8) | DataBuffer[3])) >> 2;
            SHORT zRaw = static_cast<SHORT>(((DataBuffer[4] << 8) | DataBuffer[5])) >> 2;

            const float ScaleFactor = LinearAccelerometerDevice_Resolution;
            VEC3D Sample = {};
            Sample.X = static_cast<float>(xRaw * ScaleFactor);
            Sample.Y = static_cast<float>(yRaw * ScaleFactor);
            Sample.Z = static_cast<float>(zRaw * ScaleFactor);

            // Set data ready if this is the first sample or we have exceeded the thresholds
            if (m_FirstSample)
            {
                m_FirstSample = false;
                DataReady = true;
            }
            else if ((fabsf(Sample.X - m_LastSample.Axis.X) >= m_CachedThresholds.Axis.X) ||
                     (fabsf(Sample.Y - m_LastSample.Axis.Y) >= m_CachedThresholds.Axis.Y) ||
                     (fabsf(Sample.Z - m_LastSample.Axis.Z) >= m_CachedThresholds.Axis.Z))
            {
                DataReady = true;
            }

            if (DataReady)
            {
                // Update values for SW thresholding and send data to class extension
                m_LastSample.Axis.X = Sample.X;
                m_LastSample.Axis.Y = Sample.Y;
                m_LastSample.Axis.Z = Sample.Z;

                // Save the data in the context
                InitPropVariantFromFloat(Sample.X, &(m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_X].Value));
                InitPropVariantFromFloat(Sample.Y, &(m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_Y].Value));
                InitPropVariantFromFloat(Sample.Z, &(m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_Z].Value));

                
                InitPropVariantFromFileTime(&Timestamp, &(m_pSensorData->List[LINEAR_ACCELEROMETER_DATA_TIMESTAMP].Value));

                SensorsCxSensorDataReady(m_SensorInstance, m_pSensorData);
            }
            else
            {
                TraceInformation("eCompass %!FUNC! Data from sample no. %u did NOT meet the threshold", i+1);
            }

        }

        //Calculate correct timestamp by adding data interval to timestamp of previous sample
        ulTimeStamp.LowPart = Timestamp.dwLowDateTime;
        ulTimeStamp.HighPart = Timestamp.dwHighDateTime;
        ulTimeStamp.QuadPart = ulTimeStamp.QuadPart + sampleInterval;
        Timestamp.dwLowDateTime = ulTimeStamp.LowPart;
        Timestamp.dwHighDateTime = ulTimeStamp.HighPart;
    }
    Exit:
        SENSOR_FunctionExit(Status);
        return Status;
}

//------------------------------------------------------------------------------
// Function: UpdateCachedThreshold
//
// This routine updates the cached threshold
//
// Arguments:
//       None
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
LinearAccelerometerDevice::UpdateCachedThreshold(
    )
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_AccelerationX_Gs,
                                    &m_CachedThresholds.Axis.X);
    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! LAC PropKeyFindKeyGetFloat for X failed! %!STATUS!", Status);
        goto Exit;
    }

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_AccelerationY_Gs,
                                    &m_CachedThresholds.Axis.Y);
    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! LAC PropKeyFindKeyGetFloat for Y failed! %!STATUS!", Status);
        goto Exit;
    }

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_AccelerationZ_Gs,
                                    &m_CachedThresholds.Axis.Z);
    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! LAC PropKeyFindKeyGetFloat for Z failed! %!STATUS!", Status);
        goto Exit;
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}

//------------------------------------------------------------------------------
// Function: StartSensor
//
// This routine updates the cached threshold
//
// Arguments:
//       None
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
LinearAccelerometerDevice::StartSensor(
)
{
    NTSTATUS Status = STATUS_SUCCESS;
    REGISTER_SETTING RegisterSetting = { 0, 0 };
    BYTE IntSrcBuffer = 0;

    SENSOR_FunctionEnter();

    TraceInformation("eCompass %!FUNC! Sensor Mode [%d] !", m_SensorMode);

    switch (m_SensorMode)
    {
    case STANDBY_MODE:

        // Set eCompass to accelerometer only mode
        RegisterSetting = { FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_HMS_ACCEL_ONLY };
        WdfWaitLockAcquire(m_I2CWaitLock, NULL);
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            WdfWaitLockRelease(m_I2CWaitLock);
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        // Set eCompass to active mode
        m_DataRate = GetDataRateFromReportInterval(m_DataRate.DataRateInterval, ACCELEROMETER_ONLY_MODE);

        if (m_fifo_enabled)
        {
            FXOS8700_F_SETUP_t F_setupReg = { 0 };
            ULONG desiredSampleCount = m_batch_latency / m_DataRate.DataRateInterval;
            ULONG sampleCountToSet = min(desiredSampleCount, m_accelerometer_max_fifo_samples);
            F_setupReg.b.f_mode = FXOS8700_F_SETUP_F_MODE_FIFO_STOP_OVF_VAL;
            F_setupReg.b.f_wmrk = sampleCountToSet;
            RegisterSetting = { FXOS8700_F_SETUP, F_setupReg.w };
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
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

        RegisterSetting = { FXOS8700_CTRL_REG1, m_DataRate.RateCode };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        WdfWaitLockRelease(m_I2CWaitLock);

        m_SensorMode = ACCELEROMETER_ONLY_MODE;
        m_Started = true;

        break;
    case MAGNETOMETER_ONLY_MODE:

        // Set eCompass to standby
        RegisterSetting = { FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_ACTIVE_STANDBY_MODE };
        WdfWaitLockAcquire(m_I2CWaitLock, NULL);
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            WdfWaitLockRelease(m_I2CWaitLock);
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        do
        {
            // Read the System mode if sensor is in the standby
            Status = I2CSensorReadRegister(m_I2CIoTarget, FXOS8700_SYSMOD, &IntSrcBuffer, sizeof(IntSrcBuffer));

            if (!NT_SUCCESS(Status))
            {
                WdfWaitLockRelease(m_I2CWaitLock);
                TraceError("eCompass %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", FXOS8700_SYSMOD, Status);
                goto Exit;
            }
        } while (IntSrcBuffer & FXOS8700_SYSMOD_SYSMOD_STANDBY);

        m_SensorMode = STANDBY_MODE;
        m_Started = false;

        m_DataRate = GetDataRateFromReportInterval(m_DataRate.DataRateInterval, HYBRID_MODE);
        if (m_fifo_enabled)
        {
            FXOS8700_F_SETUP_t F_setupReg = { 0 };
            ULONG desiredSampleCount = m_batch_latency / m_DataRate.DataRateInterval;
            ULONG sampleCountToSet = min(desiredSampleCount, m_accelerometer_max_fifo_samples);
            F_setupReg.b.f_mode = FXOS8700_F_SETUP_F_MODE_FIFO_STOP_OVF_VAL;
            F_setupReg.b.f_wmrk = sampleCountToSet;
            RegisterSetting = { FXOS8700_F_SETUP, F_setupReg.w };
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
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

        // Set eCompass to hybrid mode
        RegisterSetting = { FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_HMS_HYBRID_MODE | FXOS8700_M_CTRL_REG1_M_ACAL_EN };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            WdfWaitLockRelease(m_I2CWaitLock);
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        // Set eCompass to active mode
        RegisterSetting = { FXOS8700_CTRL_REG1, m_DataRate.RateCode };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
        
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        WdfWaitLockRelease(m_I2CWaitLock);
        m_SensorMode = HYBRID_MODE;
        m_Started = true;

        break;
    default:

        TraceError("eCompass %!FUNC! Incorrect Sensor Mode [%d] !", m_SensorMode);
        goto Exit;

        break;
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}

//------------------------------------------------------------------------------
// Function: StopSensor
//
// This routine updates the cached threshold
//
// Arguments:
//       None
//
// Return Value:
//      NTSTATUS code
//------------------------------------------------------------------------------
NTSTATUS
LinearAccelerometerDevice::StopSensor(
)
{
    NTSTATUS Status = STATUS_SUCCESS;
    REGISTER_SETTING RegisterSetting = { 0, 0 };
    BYTE IntSrcBuffer = 0;

    SENSOR_FunctionEnter();

    TraceInformation("eCompass %!FUNC! Sensor Mode [%d] !", m_SensorMode);

    switch (m_SensorMode)
    {
    case ACCELEROMETER_ONLY_MODE:

        // Set eCompass to standby mode
        RegisterSetting = { FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_ACTIVE_STANDBY_MODE };
        WdfWaitLockAcquire(m_I2CWaitLock, NULL);
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        WdfWaitLockRelease(m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        m_SensorMode = STANDBY_MODE;
        m_Started = false;

        break;
    case HYBRID_MODE:

        // Set eCompass to standby
        RegisterSetting = { FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_ACTIVE_STANDBY_MODE };
        WdfWaitLockAcquire(m_I2CWaitLock, NULL);
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            WdfWaitLockRelease(m_I2CWaitLock);
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        do
        {
            // Read the System mode if sensor is in the standby
            Status = I2CSensorReadRegister(m_I2CIoTarget, FXOS8700_SYSMOD, &IntSrcBuffer, sizeof(IntSrcBuffer));

            if (!NT_SUCCESS(Status))
            {
                WdfWaitLockRelease(m_I2CWaitLock);
                TraceError("eCompass %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", FXOS8700_SYSMOD, Status);
                goto Exit;
            }
        } while (IntSrcBuffer & FXOS8700_SYSMOD_SYSMOD_STANDBY);

        m_SensorMode = STANDBY_MODE;
        m_Started = false;

        // Set eCompass to magnetometer only mode with automatic calibration
        RegisterSetting = { FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_HMS_MAG_ONLY | FXOS8700_M_CTRL_REG1_M_ACAL_EN };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            WdfWaitLockRelease(m_I2CWaitLock);
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        // Set eCompass to active mode
        m_DataRate = GetDataRateFromReportInterval(m_DataRate.DataRateInterval, MAGNETOMETER_ONLY_MODE);
        RegisterSetting = { FXOS8700_CTRL_REG1, m_DataRate.RateCode };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        WdfWaitLockRelease(m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        m_SensorMode = MAGNETOMETER_ONLY_MODE;

        break;
    default:

        // Set eCompass to standby
        RegisterSetting = { FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_ACTIVE_STANDBY_MODE };
        WdfWaitLockAcquire(m_I2CWaitLock, NULL);
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        WdfWaitLockRelease(m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }
        m_SensorMode = STANDBY_MODE;
        m_Started = false;

        TraceError("eCompass %!FUNC! Incorrect Sensor Mode [%d] !", m_SensorMode);

        break;
    }

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}