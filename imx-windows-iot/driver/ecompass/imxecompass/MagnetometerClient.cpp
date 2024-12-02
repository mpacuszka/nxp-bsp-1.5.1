// Copyright (C) Microsoft Corporation, All Rights Reserved.
// Copyright 2022-2023 NXP
// Abstract:
//
//  This module contains the implementation of sensor specific functions.
//
// Environment:
//
//  Windows User-Mode Driver Framework (WUDF)

#include "Device.h"
#include "Fxos8700.h"

#include "MagnetometerClient.tmh"

#define SENSORV2_POOL_TAG_MAGNETOMETER       '2ngM'

#define Mag_Initial_Threshold_Microteslas ( 1.0f)     // 5uT

#define MagDevice_Minimum_Microteslas     (-1200.0f)
#define MagDevice_Maximum_Microteslas     ( 1200.0f)
#define MagDevice_Precision               (65536.0f) // 65536 = 2^16, 16 bit data
#define MagDevice_Range_Microteslas      \
            (MagDevice_Maximum_Microteslas - MagDevice_Minimum_Microteslas)
#define MagDevice_Resolution_Microteslas \
            (MagDevice_Range_Microteslas / MagDevice_Precision)

// Magnetometer Sensor Unique ID
// {0746712D-DFB1-42A7-8AAE-6089D2423CDE}
DEFINE_GUID(GUID_MagDevice_UniqueID,
    0x746712d, 0xdfb1, 0x42a7, 0x8a, 0xae, 0x60, 0x89, 0xd2, 0x42, 0x3c, 0xde);

// Sensor data
typedef enum
{
    MAG_DATA_TIMESTAMP = 0,
    MAG_DATA_X,
    MAG_DATA_Y,
    MAG_DATA_Z,
    MAG_DATA_ACCURACY,
    MAG_DATA_COUNT
} MAG_DATA_INDEX;

// Sensor thresholds
typedef enum
{
    MAG_THRESHOLD_X = 0,
    MAG_THRESHOLD_Y,
    MAG_THRESHOLD_Z,
    MAG_THRESHOLD_COUNT
} MAG_THRESHOLD_INDEX;


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
MagDevice::Initialize(
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
                                 SENSORV2_POOL_TAG_MAGNETOMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pEnumerationProperties);
        if (!NT_SUCCESS(Status) || m_pEnumerationProperties == nullptr)
        {
            TraceError("eCompass %!FUNC! MAG WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pEnumerationProperties, Size);
        m_pEnumerationProperties->Count = SENSOR_ENUMERATION_PROPERTIES_COUNT;

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_TYPE].Key = DEVPKEY_Sensor_Type;
        InitPropVariantFromCLSID(GUID_SensorType_Magnetometer3D,
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

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CATEGORY].Key = DEVPKEY_Sensor_Category;
        InitPropVariantFromCLSID(GUID_SensorCategory_Orientation,
                                 &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CATEGORY].Value));

        m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_PERSISTENT_UNIQUE_ID].Key = DEVPKEY_Sensor_PersistentUniqueId;
        InitPropVariantFromCLSID(GUID_MagDevice_UniqueID,
                                 &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_PERSISTENT_UNIQUE_ID].Value));

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
        ULONG Size = SENSOR_PROPERTY_LIST_SIZE(MAG_DATA_COUNT);

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_MAGNETOMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pSupportedDataFields);
        if (!NT_SUCCESS(Status) || m_pSupportedDataFields == nullptr)
        {
            TraceError("eCompass %!FUNC! MAG WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_PROPERTY_LIST_INIT(m_pSupportedDataFields, Size);
        m_pSupportedDataFields->Count = MAG_DATA_COUNT;

        m_pSupportedDataFields->List[MAG_DATA_TIMESTAMP] = PKEY_SensorData_Timestamp;
        m_pSupportedDataFields->List[MAG_DATA_X]         = PKEY_SensorData_MagneticFieldStrengthX_Microteslas;
        m_pSupportedDataFields->List[MAG_DATA_Y]         = PKEY_SensorData_MagneticFieldStrengthY_Microteslas;
        m_pSupportedDataFields->List[MAG_DATA_Z]         = PKEY_SensorData_MagneticFieldStrengthZ_Microteslas;
        m_pSupportedDataFields->List[MAG_DATA_ACCURACY]  = PKEY_SensorData_MagnetometerAccuracy;
    }

    //
    // Data
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;
        ULONG Size = SENSOR_COLLECTION_LIST_SIZE(MAG_DATA_COUNT);
        FILETIME Time = {0};

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_MAGNETOMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pSensorData);
        if (!NT_SUCCESS(Status) || m_pSensorData == nullptr)
        {
            TraceError("eCompass %!FUNC! MAG WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pSensorData, Size);
        m_pSensorData->Count = MAG_DATA_COUNT;

        m_pSensorData->List[MAG_DATA_TIMESTAMP].Key = PKEY_SensorData_Timestamp;
        GetSystemTimePreciseAsFileTime(&Time);
        InitPropVariantFromFileTime(&Time, &(m_pSensorData->List[MAG_DATA_TIMESTAMP].Value));

        m_pSensorData->List[MAG_DATA_X].Key = PKEY_SensorData_MagneticFieldStrengthX_Microteslas;
        InitPropVariantFromFloat(0.0f, &(m_pSensorData->List[MAG_DATA_X].Value));

        m_pSensorData->List[MAG_DATA_Y].Key = PKEY_SensorData_MagneticFieldStrengthY_Microteslas;
        InitPropVariantFromFloat(0.0f, &(m_pSensorData->List[MAG_DATA_Y].Value));

        m_pSensorData->List[MAG_DATA_Z].Key = PKEY_SensorData_MagneticFieldStrengthZ_Microteslas;
        InitPropVariantFromFloat(0.0f, &(m_pSensorData->List[MAG_DATA_Z].Value));

        m_pSensorData->List[MAG_DATA_ACCURACY].Key  = PKEY_SensorData_MagnetometerAccuracy;
        InitPropVariantFromUInt32(MagnetometerAccuracy_Unknown, &(m_pSensorData->List[MAG_DATA_ACCURACY].Value));

        m_CachedData.Axis.X   = 0.0f;
        m_CachedData.Axis.Y   = 1.0f;
        m_CachedData.Axis.Z   = 0.0f;
        m_CachedData.Accuracy = MagnetometerAccuracy_High;

        m_LastSample.Axis.X   = 0.0f;
        m_LastSample.Axis.Y   = 0.0f;
        m_LastSample.Axis.Z   = 0.0f;
        m_LastSample.Accuracy = MagnetometerAccuracy_Unknown;
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
                                 SENSORV2_POOL_TAG_MAGNETOMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pSensorProperties);
        if (!NT_SUCCESS(Status) || m_pSensorProperties == nullptr)
        {
            TraceError("eCompass %!FUNC! MAG WdfMemoryCreate failed %!STATUS!", Status);
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
        InitPropVariantFromCLSID(GUID_SensorType_Magnetometer3D,
                                 &(m_pSensorProperties->List[SENSOR_PROPERTY_TYPE].Value));

        m_pSensorProperties->List[SENSOR_PROPERTY_FIFORESERVEDSIZE_SAMPLES].Key = PKEY_Sensor_FifoReservedSize_Samples;
        InitPropVariantFromUInt32(SENSOR_FIFORESERVEDSIZE_SAMPLES_MAG,
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
                                 SENSORV2_POOL_TAG_MAGNETOMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pDataFieldProperties);
        if (!NT_SUCCESS(Status) || m_pDataFieldProperties == nullptr)
        {
            TraceError("eCompass %!FUNC! MAG WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pDataFieldProperties, Size);
        m_pDataFieldProperties->Count = SENSOR_DATA_FIELD_PROPERTY_COUNT;

        m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RESOLUTION].Key = PKEY_SensorDataField_Resolution;
        InitPropVariantFromFloat(MagDevice_Resolution_Microteslas,
                                 &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RESOLUTION].Value));

        m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MIN].Key = PKEY_SensorDataField_RangeMinimum;
        InitPropVariantFromFloat(MagDevice_Minimum_Microteslas,
                                 &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MIN].Value));

        m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MAX].Key = PKEY_SensorDataField_RangeMaximum;
        InitPropVariantFromFloat(MagDevice_Maximum_Microteslas,
                                 &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MAX].Value));
    }

    //
    // Set default threshold
    //
    {
        WDF_OBJECT_ATTRIBUTES MemoryAttributes;
        WDFMEMORY MemoryHandle = NULL;
        ULONG Size =  SENSOR_COLLECTION_LIST_SIZE(MAG_THRESHOLD_COUNT);    //  Timestamp and shake do not have thresholds

        MemoryHandle = NULL;
        WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
        MemoryAttributes.ParentObject = SensorInstance;
        Status = WdfMemoryCreate(&MemoryAttributes,
                                 PagedPool,
                                 SENSORV2_POOL_TAG_MAGNETOMETER,
                                 Size,
                                 &MemoryHandle,
                                 (PVOID*)&m_pThresholds);
        if (!NT_SUCCESS(Status) || m_pThresholds == nullptr)
        {
            TraceError("eCompass %!FUNC! MAG WdfMemoryCreate failed %!STATUS!", Status);
            goto Exit;
        }

        SENSOR_COLLECTION_LIST_INIT(m_pThresholds, Size);
        m_pThresholds->Count = MAG_THRESHOLD_COUNT;

        m_pThresholds->List[MAG_THRESHOLD_X].Key = PKEY_SensorData_MagneticFieldStrengthX_Microteslas;
        InitPropVariantFromFloat(Mag_Initial_Threshold_Microteslas,
                                    &(m_pThresholds->List[MAG_THRESHOLD_X].Value));

        m_pThresholds->List[MAG_THRESHOLD_Y].Key = PKEY_SensorData_MagneticFieldStrengthY_Microteslas;
        InitPropVariantFromFloat(Mag_Initial_Threshold_Microteslas,
                                    &(m_pThresholds->List[MAG_THRESHOLD_Y].Value));

        m_pThresholds->List[MAG_THRESHOLD_Z].Key = PKEY_SensorData_MagneticFieldStrengthZ_Microteslas;
        InitPropVariantFromFloat(Mag_Initial_Threshold_Microteslas,
                                    &(m_pThresholds->List[MAG_THRESHOLD_Z].Value));

        m_CachedThresholds.X = Mag_Initial_Threshold_Microteslas;
        m_CachedThresholds.Y = Mag_Initial_Threshold_Microteslas;
        m_CachedThresholds.Z = Mag_Initial_Threshold_Microteslas;

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
MagDevice::GetData(
    )
{
    BOOLEAN DataReady = FALSE;
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Read the device data
    BYTE DataBuffer[FXOS8700_DATA_REPORT_SIZE_BYTES];
    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    Status = I2CSensorReadRegister(m_I2CIoTarget, FXOS8700_M_OUT_X_MSB, &DataBuffer[0], sizeof(DataBuffer));
    WdfWaitLockRelease(m_I2CWaitLock);
    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", FXOS8700_M_OUT_X_MSB, Status);
    }
    else
    {
        // Perform data conversion
        SHORT xRaw = static_cast<SHORT>((DataBuffer[0] << 8) | DataBuffer[1]);
        SHORT yRaw = static_cast<SHORT>((DataBuffer[2] << 8) | DataBuffer[3]);
        SHORT zRaw = static_cast<SHORT>((DataBuffer[4] << 8) | DataBuffer[5]);

        const float ScaleFactor = MagDevice_Resolution_Microteslas;
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
        else if ((fabsf(Sample.X - m_LastSample.Axis.X) >= m_CachedThresholds.X) ||
                 (fabsf(Sample.Y - m_LastSample.Axis.Y) >= m_CachedThresholds.Y) ||
                 (fabsf(Sample.Z - m_LastSample.Axis.Z) >= m_CachedThresholds.Z))
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
            InitPropVariantFromFloat(Sample.X, &(m_pSensorData->List[MAG_DATA_X].Value));
            InitPropVariantFromFloat(Sample.Y, &(m_pSensorData->List[MAG_DATA_Y].Value));
            InitPropVariantFromFloat(Sample.Z, &(m_pSensorData->List[MAG_DATA_Z].Value));

            FILETIME Timestamp = {};
            GetSystemTimePreciseAsFileTime(&Timestamp);
            InitPropVariantFromFileTime(&Timestamp, &(m_pSensorData->List[MAG_DATA_TIMESTAMP].Value));

            SensorsCxSensorDataReady(m_SensorInstance, m_pSensorData);
        }
        else
        {
            TraceInformation("eCompass %!FUNC! Data did NOT meet the threshold");
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
MagDevice::UpdateCachedThreshold(
    )
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_MagneticFieldStrengthX_Microteslas,
                                    &m_CachedThresholds.X);
    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! MAG PropKeyFindKeyGetFloat for X failed! %!STATUS!", Status);
        goto Exit;
    }

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_MagneticFieldStrengthY_Microteslas,
                                    &m_CachedThresholds.Y);
    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! MAG PropKeyFindKeyGetFloat for Y failed! %!STATUS!", Status);
        goto Exit;
    }

    Status = PropKeyFindKeyGetFloat(m_pThresholds,
                                    &PKEY_SensorData_MagneticFieldStrengthZ_Microteslas,
                                    &m_CachedThresholds.Z);
    if (!NT_SUCCESS(Status))
    {
        TraceError("eCompass %!FUNC! MAG PropKeyFindKeyGetFloat for Z failed! %!STATUS!", Status);
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
MagDevice::StartSensor(
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

        // Set eCompass to magnetometer only mode with automatic calibration
        RegisterSetting = { FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_HMS_MAG_ONLY | FXOS8700_M_CTRL_REG1_M_ACAL_EN };
        WdfWaitLockAcquire(m_I2CWaitLock, NULL);
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
        m_Started = true;

        break;
    case ACCELEROMETER_ONLY_MODE:

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

        // Set eCompass to hybrid mode with automatic calibration
        RegisterSetting = { FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_HMS_HYBRID_MODE | FXOS8700_M_CTRL_REG1_M_ACAL_EN };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            WdfWaitLockRelease(m_I2CWaitLock);
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        // Set eCompass to active mode
        m_DataRate = GetDataRateFromReportInterval(m_DataRate.DataRateInterval, HYBRID_MODE);
        RegisterSetting = { FXOS8700_CTRL_REG1, m_DataRate.RateCode };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        WdfWaitLockRelease(m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        } 

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
MagDevice::StopSensor(
)
{
    NTSTATUS Status = STATUS_SUCCESS;
    REGISTER_SETTING RegisterSetting = { 0, 0 };
    BYTE IntSrcBuffer = 0;

    SENSOR_FunctionEnter();  

    TraceInformation("eCompass %!FUNC! Sensor Mode [%d] !", m_SensorMode);

    switch (m_SensorMode)
    {
    case MAGNETOMETER_ONLY_MODE:

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

        // Set eCompass to accelerometer only mode
        RegisterSetting = { FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_HMS_ACCEL_ONLY };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status))
        {
            WdfWaitLockRelease(m_I2CWaitLock);
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

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

        // Set eCompass to active mode
        m_DataRate = GetDataRateFromReportInterval(m_DataRate.DataRateInterval, ACCELEROMETER_ONLY_MODE);
        RegisterSetting = { FXOS8700_CTRL_REG1, m_DataRate.RateCode };
        Status = I2CSensorWriteRegister(m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        WdfWaitLockRelease(m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("eCompass %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            goto Exit;
        }

        m_SensorMode = ACCELEROMETER_ONLY_MODE;

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