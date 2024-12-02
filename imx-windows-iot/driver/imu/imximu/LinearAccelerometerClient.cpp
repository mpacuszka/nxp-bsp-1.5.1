// Copyright (C) Microsoft Corporation, All Rights Reserved.
// Copyright 2023 NXP
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

#define SENSORV2_POOL_TAG_LINEAR_ACCELEROMETER      '2CaL'

#define LinearAccelerometerDevice_Default_Threshold ( 0.1f)
#define LinearAccelerometerDevice_Minimum           (-4.0f)     // in g
#define LinearAccelerometerDevice_Maximum           ( 4.0f)     // in g
#define LinearAccelerometerDevice_Precision         ( 65536.0f) // 65536 = 2^16, 16 bit data
#define LinearAccelerometerDevice_Range             (LinearAccelerometerDevice_Maximum - LinearAccelerometerDevice_Minimum)
#define LinearAccelerometerDevice_Resolution        (LinearAccelerometerDevice_Range / LinearAccelerometerDevice_Precision)
#define LinearAccelerometerDevice_DefaultDataRate   (5)

const DATA_RATE ACC_SUPPORTED_DATA_RATES[] = { { 625, LSM6DSOX_CTRL1_XL_ODR_1P6_HZ  | LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G },
                                               {  80, LSM6DSOX_CTRL1_XL_ODR_12P5_HZ | LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G },
                                               {  40, LSM6DSOX_CTRL1_XL_ODR_26_HZ   | LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G },
                                               {  20, LSM6DSOX_CTRL1_XL_ODR_52_HZ   | LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G },
                                               {  10, LSM6DSOX_CTRL1_XL_ODR_104_HZ  | LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G },
                                               {   5, LSM6DSOX_CTRL1_XL_ODR_208_HZ  | LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G },
                                               {   3, LSM6DSOX_CTRL1_XL_ODR_416_HZ  | LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G },
                                               {   2, LSM6DSOX_CTRL1_XL_ODR_833_HZ  | LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G },
                                               {   1, LSM6DSOX_CTRL1_XL_ODR_1P6_KHZ | LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G } };

const ULONG ACC_SUPPORTED_DATA_RATES_COUNT = ARRAYSIZE(ACC_SUPPORTED_DATA_RATES);

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

// Return the rate that is just less than the desired report interval
DATA_RATE LinearAccelerometerDevice::GetDataRateFromReportInterval(_In_ ULONG ReportInterval)
{
    DATA_RATE dataRate = ACC_SUPPORTED_DATA_RATES[0];

    for (ULONG i = 0; i < ACC_SUPPORTED_DATA_RATES_COUNT; i++)
    {
        dataRate = ACC_SUPPORTED_DATA_RATES[i];
        if (dataRate.DataRateInterval <= ReportInterval)
        {
            break;
        }
    }

    return dataRate;
}

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
            TraceError("IMU %!FUNC! LAC WdfMemoryCreate failed %!STATUS!", Status);
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
            TraceError("IMU %!FUNC! LAC WdfMemoryCreate failed %!STATUS!", Status);
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
            TraceError("IMU %!FUNC! LAC WdfMemoryCreate failed %!STATUS!", Status);
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

        m_DataRate = GetDataRateFromReportInterval(LinearAccelerometerDevice_DefaultDataRate);

        m_fifo_enabled = false;
        m_batch_latency = DEFAULT_BATCH_LATENCY;
        m_fifo_size = DEFAULT_BATCH_SAMPLE_COUNT;
        m_max_fifo_samples = SENSOR_FIFORESERVEDSIZE_SAMPLES_ACC;

        SENSOR_COLLECTION_LIST_INIT(m_pSensorProperties, Size);
        m_pSensorProperties->Count = SENSOR_PROPERTY_COUNT;

        m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Key = PKEY_Sensor_State;
        InitPropVariantFromUInt32(SensorState_Initializing,
                                  &(m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_MIN_DATA_INTERVAL].Key = PKEY_Sensor_MinimumDataInterval_Ms;
        InitPropVariantFromUInt32(SENSOR_MIN_REPORT_INTERVAL,
                                  &(m_pSensorProperties->List[SENSOR_PROPERTY_MIN_DATA_INTERVAL].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_DEFAULT_DATA_INTERVAL].Key = PKEY_SensorHistory_Interval_Ms;
        InitPropVariantFromUInt32(LinearAccelerometerDevice_DefaultDataRate,
            &(m_pSensorProperties->List[SENSOR_PROPERTY_DEFAULT_DATA_INTERVAL].Value));
        

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
            TraceError("IMU %!FUNC! LAC WdfMemoryCreate failed %!STATUS!", Status);
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
            TraceError("IMU %!FUNC! LAC WdfMemoryCreate failed %!STATUS!", Status);
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
// This routine is called by worker thread to read a single sample, compare threshold
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

    // Read the device data
    BYTE DataBuffer[LSM6DSOX_DATA_REPORT_SIZE_BYTES]; // Burst-read mode 2x amount of data

    FILETIME Timestamp = { 0 };
    ULARGE_INTEGER ulTimestamp;
    const DWORD sampleInterval = 10000 * m_DataRate.DataRateInterval; //Time between samples in FIFO stack in nanoseconds
    GetSystemTimePreciseAsFileTime(&Timestamp);

    //Since first sample in FIFO buffer is the oldest one, we calculate its correct timestamp by subtracting time interval between samples multiplied by size of FIFO buffer from current timestamp
    ulTimestamp.LowPart = Timestamp.dwLowDateTime;
    ulTimestamp.HighPart = Timestamp.dwHighDateTime;
    ulTimestamp.QuadPart = ulTimestamp.QuadPart - (ULONGLONG)sampleInterval * (m_fifo_size - 1);
    Timestamp.dwLowDateTime = ulTimestamp.LowPart;
    Timestamp.dwHighDateTime = ulTimestamp.HighPart;

    UCHAR regAddress = m_fifo_enabled ? LSM6DSOX_FIFO_DATA_OUT_X_L : LSM6DSOX_OUTX_L_A;

    for (ULONG i = 0; i < m_fifo_size; i++)
    {
        WdfWaitLockAcquire(m_I2CWaitLock, NULL);
        Status = I2CSensorReadRegister(m_I2CIoTarget, regAddress, &DataBuffer[0], sizeof(DataBuffer));
        WdfWaitLockRelease(m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", regAddress, Status);
        }
        else
        {
            // Perform data conversion
            SHORT xRaw = static_cast<SHORT>(((DataBuffer[1] << 8) | DataBuffer[0]));
            SHORT yRaw = static_cast<SHORT>(((DataBuffer[3] << 8) | DataBuffer[2]));
            SHORT zRaw = static_cast<SHORT>(((DataBuffer[5] << 8) | DataBuffer[4]));

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
                TraceInformation("IMU %!FUNC! Data did NOT meet the threshold");
            }

            //Calculate correct timestamp by adding data interval to timestamp of previous sample
            ulTimestamp.LowPart = Timestamp.dwLowDateTime;
            ulTimestamp.HighPart = Timestamp.dwHighDateTime;
            ulTimestamp.QuadPart = ulTimestamp.QuadPart + sampleInterval;
            Timestamp.dwLowDateTime = ulTimestamp.LowPart;
            Timestamp.dwHighDateTime = ulTimestamp.HighPart;
        }
    }

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
        TraceError("IMU %!FUNC! LAC PropKeyFindKeyGetFloat for X failed! %!STATUS!", Status);
        goto Exit;
    }

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_AccelerationY_Gs,
                                    &m_CachedThresholds.Axis.Y);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! LAC PropKeyFindKeyGetFloat for Y failed! %!STATUS!", Status);
        goto Exit;
    }

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_AccelerationZ_Gs,
                                    &m_CachedThresholds.Axis.Z);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! LAC PropKeyFindKeyGetFloat for Z failed! %!STATUS!", Status);
        goto Exit;
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
LinearAccelerometerDevice::UpdateDataInterval(
    _In_ ULONG DataRateMs
    )
{
    SENSOR_FunctionEnter();

    NTSTATUS Status = STATUS_SUCCESS;
    REGISTER_SETTING RegisterSetting = { 0, 0 };

    m_FirstSample = true;
    m_DataRate = GetDataRateFromReportInterval(DataRateMs);

    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    if (m_DataRate.DataRateInterval <= SENSOR_HIGH_PERFORMANCE_THRESHOLD_INTERVAL)
    {
        // Set High Performance mode
        RegisterSetting = { LSM6DSOX_CTRL6_C, LSM6DSOX_CTRL6_C_HM_MODE_ENABLED };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value)); 
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            WdfWaitLockRelease(m_I2CWaitLock);
            goto Exit;
        }
    }

    if (m_Started)
    {
        // Set data rate in HW and set ACTIVE MODE
        RegisterSetting = { LSM6DSOX_CTRL1_XL, m_DataRate.RateCode };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            WdfWaitLockRelease(m_I2CWaitLock);
            goto Exit;
        }

        if (m_fifo_enabled)
        {
            TraceInformation("SENSOR START FIFO");
            ULONG desiredSampleCount = m_batch_latency / m_DataRate.DataRateInterval;
            ULONG sampleCountToSet = min(desiredSampleCount, m_max_fifo_samples);
            sampleCountToSet = sampleCountToSet;

            //Set watermark threshold in registers LSM6DSOX_FIFO_CTRL1 and LSM6DSOX_FIFO_CTRL2

            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL1;
            RegisterSetting.Value = (uint8_t)sampleCountToSet;//Watermark[7:0]
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL2;
            RegisterSetting.Value = (uint8_t)(((sampleCountToSet >> 8) & 1));//Watermark[8]
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            //Set fifo poll rate
            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL3;
            RegisterSetting.Value = m_DataRate.RateCode >> LSM6DSOX_FIFO_CTRL3_XL_DATARATE_SHIFT;
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL4;
            RegisterSetting.Value = LSM6DSOX_FIFO_CTRL4_FIFO_MODE_ENABLE;
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            m_fifo_size = sampleCountToSet;
        }

    }

    if (m_DataRate.DataRateInterval > SENSOR_HIGH_PERFORMANCE_THRESHOLD_INTERVAL)
    {
        // Set Normal mode to save power
        RegisterSetting = { LSM6DSOX_CTRL6_C, LSM6DSOX_CTRL6_C_HM_MODE_DISABLED };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
        }
    } 
    WdfWaitLockRelease(m_I2CWaitLock);

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}

NTSTATUS
LinearAccelerometerDevice::UpdateBatchLatency(
    _In_ ULONG batchLatencyMs
    )
{
    SENSOR_FunctionEnter();

    NTSTATUS Status = STATUS_SUCCESS;
    REGISTER_SETTING RegisterSetting = { 0,0 };

    m_FirstSample = true;
    
    if (m_Started)
    {
        if (batchLatencyMs == 0)
        {
            WdfWaitLockAcquire(m_I2CWaitLock, NULL);

            RegisterSetting.Register = LSM6DSOX_INT1_CTRL;
            RegisterSetting.Value = LSM6DSOX_INT1_CTRL_DRDY_G_ENABLE | LSM6DSOX_INT1_CTRL_DRDY_XL_ENABLE;
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL1;
            RegisterSetting.Value = 0;
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL2;
            RegisterSetting.Value = 0;
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL3;
            RegisterSetting.Value = LSM6DSOX_FIFO_CTRL3_BDR_XL_NOT_BATCHED | LSM6DSOX_FIFO_CTRL3_BDR_GY_NOT_BATCHED;
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL4;
            RegisterSetting.Value = LSM6DSOX_FIFO_CTRL4_FIFO_MODE_DISABLE;
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }


            m_fifo_enabled = false;
            m_batch_latency = 0;
            m_fifo_size = DEFAULT_BATCH_SAMPLE_COUNT;
        }
        else {
            ULONG desiredSampleCount = m_batch_latency / m_DataRate.DataRateInterval;
            ULONG sampleCountToSet = min(desiredSampleCount, m_max_fifo_samples);
            sampleCountToSet = sampleCountToSet;

            WdfWaitLockAcquire(m_I2CWaitLock, NULL);
            //Disable data ready interrupts and enable FIFO Interrupts
            RegisterSetting.Register = LSM6DSOX_INT1_CTRL;
            RegisterSetting.Value = LSM6DSOX_INT1_CTRL_DRDY_G_ENABLE | LSM6DSOX_INT1_CTRL_FIFO_TH_MASK;
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            //Set watermark threshold in registers LSM6DSOX_FIFO_CTRL1 and LSM6DSOX_FIFO_CTRL2

            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL1;
            RegisterSetting.Value = (uint8_t)sampleCountToSet;//Watermark[7:0]
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL2;
            RegisterSetting.Value = (uint8_t)(((sampleCountToSet >> 8) & 1));//Watermark[8]
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            //Set fifo poll rate
            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL3;
            RegisterSetting.Value = m_DataRate.RateCode >> LSM6DSOX_FIFO_CTRL3_XL_DATARATE_SHIFT;
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            RegisterSetting.Register = LSM6DSOX_FIFO_CTRL4;
            RegisterSetting.Value = LSM6DSOX_FIFO_CTRL4_FIFO_MODE_ENABLE;
            Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                WdfWaitLockRelease(m_I2CWaitLock);
                goto Exit;
            }

            WdfWaitLockRelease(m_I2CWaitLock);

            m_fifo_size = sampleCountToSet;
            m_batch_latency = batchLatencyMs;
            m_fifo_enabled = true;
        }
    } 
    else {
        if (batchLatencyMs == 0)
        {
            m_fifo_enabled = false;
            m_batch_latency = 0;
            m_fifo_size = DEFAULT_BATCH_SAMPLE_COUNT;
            
        }
        else {
            m_fifo_enabled = true;
            m_batch_latency = batchLatencyMs;
            ULONG desiredSampleCount = batchLatencyMs / m_DataRate.DataRateInterval;
            m_fifo_size = min(desiredSampleCount, m_max_fifo_samples);
        }
    }
    goto Exit;
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

    
    SENSOR_FunctionEnter();

    // Set IMU to active mode
    RegisterSetting = { LSM6DSOX_CTRL1_XL, m_DataRate.RateCode };
    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
        goto Exit;
    }


    if (m_fifo_enabled)
    {
        ULONG desiredSampleCount = m_batch_latency / m_DataRate.DataRateInterval;
        ULONG sampleCountToSet = min(desiredSampleCount, m_max_fifo_samples);
        sampleCountToSet = sampleCountToSet;

        //Disable data ready interrupts and enable FIFO Interrupts
        RegisterSetting.Register = LSM6DSOX_INT1_CTRL; 
        RegisterSetting.Value = LSM6DSOX_INT1_CTRL_DRDY_G_ENABLE | LSM6DSOX_INT1_CTRL_FIFO_TH_MASK;
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            WdfWaitLockRelease(m_I2CWaitLock);
            goto Exit;
        }

        //Set watermark threshold in registers LSM6DSOX_FIFO_CTRL1 and LSM6DSOX_FIFO_CTRL2

        RegisterSetting.Register = LSM6DSOX_FIFO_CTRL1;
        RegisterSetting.Value = (uint8_t)sampleCountToSet;//Watermark[7:0]
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            WdfWaitLockRelease(m_I2CWaitLock);
            goto Exit;
        }

        RegisterSetting.Register = LSM6DSOX_FIFO_CTRL2; 
        RegisterSetting.Value = (uint8_t)(((sampleCountToSet >> 8) & 1));//Watermark[8]
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            WdfWaitLockRelease(m_I2CWaitLock);
            goto Exit;
        }

        //Set fifo poll rate
        RegisterSetting.Register = LSM6DSOX_FIFO_CTRL3;
        RegisterSetting.Value = m_DataRate.RateCode >> LSM6DSOX_FIFO_CTRL3_XL_DATARATE_SHIFT;
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            WdfWaitLockRelease(m_I2CWaitLock);
            goto Exit;
        }

        //Enable fifo
        RegisterSetting.Register = LSM6DSOX_FIFO_CTRL4;
        RegisterSetting.Value = LSM6DSOX_FIFO_CTRL4_FIFO_MODE_ENABLE;
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            WdfWaitLockRelease(m_I2CWaitLock);
            goto Exit;
        }


        m_fifo_size = sampleCountToSet;
    }
    WdfWaitLockRelease(m_I2CWaitLock);
    m_Started = true;

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

    SENSOR_FunctionEnter();
    WdfWaitLockAcquire(m_I2CWaitLock, NULL);

    // Set IMU to power-down mode
    RegisterSetting = { LSM6DSOX_CTRL1_XL, LSM6DSOX_CTRL1_XL_ODR_POWER_DOWN };
    Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
        WdfWaitLockRelease(m_I2CWaitLock);
        goto Exit;
    }

    WdfWaitLockRelease(m_I2CWaitLock);
    m_Started = false;

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}