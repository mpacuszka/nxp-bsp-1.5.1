// Copyright (C) Microsoft Corporation, All Rights Reserved.
// Copyright 2023 NXP
// 
// Abstract:
//
//  This module contains the implementation of sensor specific functions.
//
// Environment:
//
//  Windows User-Mode Driver Framework (WUDF)

#include "Device.h"

#include "GyrClient.tmh"

#define SENSORV2_POOL_TAG_GYROSCOPE '2oyG'


#define Gyr_Initial_Threshold       ( 0.5f)

#define GyrDevice_Minimum           (-2000.0f)
#define GyrDevice_Maximum           ( 2000.0f)
#define GyrDevice_Precision         ( 65536.0f) // 65536 = 2^16, 16 bit data
#define GyrDevice_Range             (GyrDevice_Maximum - GyrDevice_Minimum)
#define GyrDevice_Resolution        (GyrDevice_Range / GyrDevice_Precision)
#define GyrDevice_DefaultDataRate   (10)

const DATA_RATE GYR_SUPPORTED_DATA_RATES[] = { {  80, LSM6DSOX_CTRL2_G_ODR_12P5_HZ | LSM6DSOX_CTRL2_G_FS_MODE_2000_DPS },
                                               {  40, LSM6DSOX_CTRL2_G_ODR_26_HZ   | LSM6DSOX_CTRL2_G_FS_MODE_2000_DPS },
                                               {  20, LSM6DSOX_CTRL2_G_ODR_52_HZ   | LSM6DSOX_CTRL2_G_FS_MODE_2000_DPS },
                                               {  10, LSM6DSOX_CTRL2_G_ODR_104_HZ  | LSM6DSOX_CTRL2_G_FS_MODE_2000_DPS },
                                               {   5, LSM6DSOX_CTRL2_G_ODR_208_HZ  | LSM6DSOX_CTRL2_G_FS_MODE_2000_DPS },
                                               {   3, LSM6DSOX_CTRL2_G_ODR_416_HZ  | LSM6DSOX_CTRL2_G_FS_MODE_2000_DPS },
                                               {   2, LSM6DSOX_CTRL2_G_ODR_833_HZ  | LSM6DSOX_CTRL2_G_FS_MODE_2000_DPS },
                                               {   1, LSM6DSOX_CTRL2_G_ODR_1P6_KHZ | LSM6DSOX_CTRL2_G_FS_MODE_2000_DPS } };

const ULONG GYR_SUPPORTED_DATA_RATES_COUNT = ARRAYSIZE(GYR_SUPPORTED_DATA_RATES);

//  Gyroscope Unique ID
// {61A61B96-1E4C-47C6-8697-654680101446}
DEFINE_GUID(GUID_GyrDevice_UniqueID,
    0x61a61b96, 0x1e4c, 0x47c6, 0x86, 0x97, 0x65, 0x46, 0x80, 0x10, 0x14, 0x46);

// Sensor data
typedef enum
{
    GYR_DATA_TIMESTAMP = 0,
    GYR_DATA_X,
    GYR_DATA_Y,
    GYR_DATA_Z,
    GYR_DATA_COUNT
} GYR_DATA_INDEX;

// Sensor thresholds
typedef enum
{
    GYR_THRESHOLD_X = 0,
    GYR_THRESHOLD_Y,
    GYR_THRESHOLD_Z,
    GYR_THRESHOLD_COUNT
} GYR_THRESHOLD_INDEX;

// Return the rate that is just less than the desired report interval
DATA_RATE GyrDevice::GetDataRateFromReportInterval(_In_ ULONG ReportInterval)
{
    DATA_RATE dataRate = GYR_SUPPORTED_DATA_RATES[0];

    for (ULONG i = 0; i < GYR_SUPPORTED_DATA_RATES_COUNT; i++)
    {
        dataRate = GYR_SUPPORTED_DATA_RATES[i];
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
GyrDevice::Initialize(
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
    // Create Lock
    //
    Status = WdfWaitLockCreate(WDF_NO_OBJECT_ATTRIBUTES, &m_I2CWaitLock);
    if (!NT_SUCCESS(Status))
    {
        TraceError("COMBO %!FUNC! GYR WdfWaitLockCreate failed %!STATUS!", Status);
        goto Exit;
    }

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
                                 SENSORV2_POOL_TAG_GYROSCOPE,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pEnumerationProperties);
        if (!NT_SUCCESS(Status) || m_pEnumerationProperties == nullptr)
        {
            TraceError("COMBO %!FUNC! GYR WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pEnumerationProperties, Size);
        m_pEnumerationProperties->Count = SENSOR_ENUMERATION_PROPERTIES_COUNT;

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_TYPE].Key = DEVPKEY_Sensor_Type;
        InitPropVariantFromCLSID(GUID_SensorType_Gyrometer3D,
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
        InitPropVariantFromCLSID(GUID_GyrDevice_UniqueID,
            &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_PERSISTENT_UNIQUE_ID].Value));

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CATEGORY].Key = DEVPKEY_Sensor_Category;
        InitPropVariantFromCLSID(GUID_SensorCategory_Motion,
            &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CATEGORY].Value));

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_ISPRIMARY].Key = DEVPKEY_Sensor_IsPrimary;
        InitPropVariantFromBoolean(FALSE,
                                 &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_ISPRIMARY].Value));
    }

    //
    // Supported Data-Fields
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;
        ULONG Size = SENSOR_PROPERTY_LIST_SIZE(GYR_DATA_COUNT);

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_GYROSCOPE,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pSupportedDataFields);
        if (!NT_SUCCESS(Status) || m_pSupportedDataFields == nullptr)
        {
            TraceError("COMBO %!FUNC! GYR WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_PROPERTY_LIST_INIT(m_pSupportedDataFields, Size);
        m_pSupportedDataFields->Count = GYR_DATA_COUNT;

        m_pSupportedDataFields->List[GYR_DATA_TIMESTAMP] = PKEY_SensorData_Timestamp;
        m_pSupportedDataFields->List[GYR_DATA_X] = PKEY_SensorData_AngularVelocityX_DegreesPerSecond;
        m_pSupportedDataFields->List[GYR_DATA_Y] = PKEY_SensorData_AngularVelocityY_DegreesPerSecond;
        m_pSupportedDataFields->List[GYR_DATA_Z] = PKEY_SensorData_AngularVelocityZ_DegreesPerSecond;
    }

    //
    // Data
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;
        ULONG Size = SENSOR_COLLECTION_LIST_SIZE(GYR_DATA_COUNT);
        FILETIME Time = {0};

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_GYROSCOPE,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pSensorData);
        if (!NT_SUCCESS(Status) || m_pSensorData == nullptr)
        {
            TraceError("COMBO %!FUNC! GYR WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pSensorData, Size);
        m_pSensorData->Count = GYR_DATA_COUNT;

        m_pSensorData->List[GYR_DATA_TIMESTAMP].Key = PKEY_SensorData_Timestamp;
        GetSystemTimePreciseAsFileTime(&Time);
        InitPropVariantFromFileTime(&Time, &(m_pSensorData->List[GYR_DATA_TIMESTAMP].Value));

        m_pSensorData->List[GYR_DATA_X].Key = PKEY_SensorData_AngularVelocityX_DegreesPerSecond;
        InitPropVariantFromFloat(0.0f, &(m_pSensorData->List[GYR_DATA_X].Value));

        m_pSensorData->List[GYR_DATA_Y].Key = PKEY_SensorData_AngularVelocityY_DegreesPerSecond;
        InitPropVariantFromFloat(0.0f, &(m_pSensorData->List[GYR_DATA_Y].Value));

        m_pSensorData->List[GYR_DATA_Z].Key = PKEY_SensorData_AngularVelocityZ_DegreesPerSecond;
        InitPropVariantFromFloat(0.0f, &(m_pSensorData->List[GYR_DATA_Z].Value));

        m_CachedData.X = 0.0f;
        m_CachedData.Y = 0.0f;
        m_CachedData.Z = 0.0f;

        m_LastSample.X = 0.0f;
        m_LastSample.Y = 0.0f;
        m_LastSample.Z = 0.0f;
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
                                 SENSORV2_POOL_TAG_GYROSCOPE,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pSensorProperties);
        if (!NT_SUCCESS(Status) || m_pSensorProperties == nullptr)
        {
            TraceError("COMBO %!FUNC! GYR WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        m_DataRate = GetDataRateFromReportInterval(GyrDevice_DefaultDataRate);

        m_fifo_enabled = false;
        m_batch_latency = DEFAULT_BATCH_LATENCY;
        m_fifo_size = DEFAULT_BATCH_SAMPLE_COUNT;
        m_max_fifo_samples = SENSOR_FIFORESERVEDSIZE_SAMPLES_GYR;

        SENSOR_COLLECTION_LIST_INIT(m_pSensorProperties, Size);
        m_pSensorProperties->Count = SENSOR_PROPERTY_COUNT;

        m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Key = PKEY_Sensor_State;
        InitPropVariantFromUInt32(SensorState_Initializing,
                                  &(m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_MIN_DATA_INTERVAL].Key = PKEY_Sensor_MinimumDataInterval_Ms;
        InitPropVariantFromUInt32(SENSOR_MIN_REPORT_INTERVAL,
                                  &(m_pSensorProperties->List[SENSOR_PROPERTY_MIN_DATA_INTERVAL].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_DEFAULT_DATA_INTERVAL].Key = PKEY_SensorHistory_Interval_Ms;
        InitPropVariantFromUInt32(GyrDevice_DefaultDataRate,
            &(m_pSensorProperties->List[SENSOR_PROPERTY_DEFAULT_DATA_INTERVAL].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE].Key = PKEY_Sensor_MaximumDataFieldSize_Bytes;
        InitPropVariantFromUInt32(CollectionsListGetMarshalledSize(m_pSensorData),
                                  &(m_pSensorProperties->List[SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_TYPE].Key = PKEY_Sensor_Type;
        InitPropVariantFromCLSID(GUID_SensorType_Gyrometer3D,
                                     &(m_pSensorProperties->List[SENSOR_PROPERTY_TYPE].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_FIFORESERVEDSIZE_SAMPLES].Key = PKEY_Sensor_FifoReservedSize_Samples;
        InitPropVariantFromUInt32(SENSOR_FIFORESERVEDSIZE_SAMPLES_GYR,
            &(m_pSensorProperties->List[SENSOR_PROPERTY_FIFORESERVEDSIZE_SAMPLES].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_FIFO_MAXSIZE_SAMPLES].Key = PKEY_Sensor_FifoMaxSize_Samples;
        InitPropVariantFromUInt32(SENSOR_FIFO_MAXSIZE_SAMPLES,
            &(m_pSensorProperties->List[SENSOR_PROPERTY_FIFO_MAXSIZE_SAMPLES].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_WAKE_CAPABLE].Key = PKEY_Sensor_WakeCapable;
        InitPropVariantFromBoolean(SENSOR_WAKE_CAPABLE,
            &(m_pSensorProperties->List[SENSOR_PROPERTY_WAKE_CAPABLE].Value));
    }

    //
    // Data filed properties
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
                                 SENSORV2_POOL_TAG_GYROSCOPE,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pDataFieldProperties);
        if (!NT_SUCCESS(Status) || m_pDataFieldProperties == nullptr)
        {
            TraceError("COMBO %!FUNC! GYR WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pDataFieldProperties, Size);
        m_pDataFieldProperties->Count = SENSOR_DATA_FIELD_PROPERTY_COUNT;

        m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RESOLUTION].Key = PKEY_SensorDataField_Resolution;
        InitPropVariantFromFloat(GyrDevice_Resolution,
            &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RESOLUTION].Value));

        m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MIN].Key = PKEY_SensorDataField_RangeMinimum;
        InitPropVariantFromFloat(GyrDevice_Minimum,
            &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MIN].Value));

        m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MAX].Key = PKEY_SensorDataField_RangeMaximum;
        InitPropVariantFromFloat(GyrDevice_Maximum,
            &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MAX].Value));
    }

    //
    // Set default threshold
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;
        ULONG Size =  SENSOR_COLLECTION_LIST_SIZE(GYR_THRESHOLD_COUNT);    //  Timestamp and shake do not have thresholds

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_GYROSCOPE,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pThresholds);
        if (!NT_SUCCESS(Status) || m_pThresholds == nullptr)
        {
            TraceError("COMBO %!FUNC! GYR WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pThresholds, Size);
        m_pThresholds->Count = GYR_THRESHOLD_COUNT;

        m_pThresholds->List[GYR_THRESHOLD_X].Key = PKEY_SensorData_AngularVelocityX_DegreesPerSecond;
        InitPropVariantFromFloat(Gyr_Initial_Threshold,
                                    &(m_pThresholds->List[GYR_THRESHOLD_X].Value));

        m_pThresholds->List[GYR_THRESHOLD_Y].Key = PKEY_SensorData_AngularVelocityY_DegreesPerSecond;
        InitPropVariantFromFloat(Gyr_Initial_Threshold,
                                    &(m_pThresholds->List[GYR_THRESHOLD_Y].Value));

        m_pThresholds->List[GYR_THRESHOLD_Z].Key = PKEY_SensorData_AngularVelocityZ_DegreesPerSecond;
        InitPropVariantFromFloat(Gyr_Initial_Threshold,
                                    &(m_pThresholds->List[GYR_THRESHOLD_Z].Value));

        m_CachedThresholds.X = Gyr_Initial_Threshold;
        m_CachedThresholds.Y = Gyr_Initial_Threshold;
        m_CachedThresholds.Z = Gyr_Initial_Threshold;

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
GyrDevice::GetData(
    )
{
    BOOLEAN DataReady = FALSE;
    FILETIME TimeStamp = {0};
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Read the device data
    BYTE DataBuffer[LSM6DSOX_DATA_REPORT_SIZE_BYTES]; // Burst-read mode 2x amount of data
    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    Status = I2CSensorReadRegister(m_I2CIoTarget, LSM6DSOX_OUTX_L_G, &DataBuffer[0], sizeof(DataBuffer));
    WdfWaitLockRelease(m_I2CWaitLock);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", LSM6DSOX_OUTX_L_A, Status);
    }
    else
    {
        // Perform data conversion
        SHORT xRaw = static_cast<SHORT>(((DataBuffer[1] << 8) | DataBuffer[0]));
        SHORT yRaw = static_cast<SHORT>(((DataBuffer[3] << 8) | DataBuffer[2]));
        SHORT zRaw = static_cast<SHORT>(((DataBuffer[5] << 8) | DataBuffer[4]));

        const float ScaleFactor = GyrDevice_Resolution;
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
        else if ((fabsf(Sample.X - m_LastSample.X) >= m_CachedThresholds.X) ||
                 (fabsf(Sample.Y - m_LastSample.Y) >= m_CachedThresholds.Y) ||
                 (fabsf(Sample.Z - m_LastSample.Z) >= m_CachedThresholds.Z))
        {
            DataReady = true;
        }

        if (DataReady)
        {
            // Update values for SW thresholding and send data to class extension
            m_LastSample.X = Sample.X;
            m_LastSample.Y = Sample.Y;
            m_LastSample.Z = Sample.Z;

            // Save the data in the context
            InitPropVariantFromFloat(Sample.X, &(m_pSensorData->List[GYR_DATA_X].Value));
            InitPropVariantFromFloat(Sample.Y, &(m_pSensorData->List[GYR_DATA_Y].Value));
            InitPropVariantFromFloat(Sample.Z, &(m_pSensorData->List[GYR_DATA_Z].Value));

            FILETIME Timestamp = {};
            GetSystemTimePreciseAsFileTime(&Timestamp);
            InitPropVariantFromFileTime(&Timestamp, &(m_pSensorData->List[GYR_DATA_TIMESTAMP].Value));

            SensorsCxSensorDataReady(m_SensorInstance, m_pSensorData);
        }
        else
        {
            TraceInformation("IMU %!FUNC! Data did NOT meet the threshold");
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
GyrDevice::UpdateCachedThreshold(
    )
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_AngularVelocityX_DegreesPerSecond,
                                    &m_CachedThresholds.X);
    if (!NT_SUCCESS(Status))
    {
        TraceError("COMBO %!FUNC! GYR PropKeyFindKeyGetFloat for X failed! %!STATUS!", Status);
        goto Exit;
    }

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_AngularVelocityY_DegreesPerSecond,
                                    &m_CachedThresholds.Y);
    if (!NT_SUCCESS(Status))
    {
        TraceError("COMBO %!FUNC! GYR PropKeyFindKeyGetFloat for Y failed! %!STATUS!", Status);
        goto Exit;
    }

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_AngularVelocityZ_DegreesPerSecond,
                                    &m_CachedThresholds.Z);
    if (!NT_SUCCESS(Status))
    {
        TraceError("COMBO %!FUNC! GYR PropKeyFindKeyGetFloat for Z failed! %!STATUS!", Status);
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
GyrDevice::UpdateDataInterval(
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
        RegisterSetting = { LSM6DSOX_CTRL7_G, LSM6DSOX_CTRL7_G_HM_MODE_ENABLED };
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
        RegisterSetting = { LSM6DSOX_CTRL2_G, m_DataRate.RateCode };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            TraceError("IMU %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            WdfWaitLockRelease(m_I2CWaitLock);
            goto Exit;
        }
    }

    if (m_DataRate.DataRateInterval > SENSOR_HIGH_PERFORMANCE_THRESHOLD_INTERVAL)
    {
        // Set Normal mode to save power
        RegisterSetting = { LSM6DSOX_CTRL7_G, LSM6DSOX_CTRL7_G_HM_MODE_DISABLED };
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
_GyrDevice::UpdateBatchLatency(
    _In_ ULONG BatchLatencyMs
)
{
    UNREFERENCED_PARAMETER(BatchLatencyMs);
    SENSOR_FunctionEnter();

    NTSTATUS Status = STATUS_SUCCESS;

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
GyrDevice::StartSensor(
)
{
    NTSTATUS Status = STATUS_SUCCESS;
    REGISTER_SETTING RegisterSetting = { 0, 0 };

    SENSOR_FunctionEnter();

    // Set IMU to active mode
    RegisterSetting = { LSM6DSOX_CTRL2_G, m_DataRate.RateCode };
    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
    WdfWaitLockRelease(m_I2CWaitLock);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
        goto Exit;
    }
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
GyrDevice::StopSensor(
)
{
    NTSTATUS Status = STATUS_SUCCESS;
    REGISTER_SETTING RegisterSetting = { 0, 0 };

    SENSOR_FunctionEnter();

    // Set IMU to power-down mode
    RegisterSetting = { LSM6DSOX_CTRL2_G, LSM6DSOX_CTRL2_G_ODR_POWER_DOWN };
    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
    WdfWaitLockRelease(m_I2CWaitLock);
    if (!NT_SUCCESS(Status))
    {
        TraceError("IMU %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
        goto Exit;
    }
    m_Started = false;

Exit:
    SENSOR_FunctionExit(Status);
    return Status;
}