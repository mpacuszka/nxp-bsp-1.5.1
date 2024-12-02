//Copyright (C) Microsoft Corporation, All Rights Reserved.
//Copyright 2022 NXP
// 
//Abstract:
//
//    This module contains the implementation of driver callback functions
//    from clx to FXAS210002 gyroscope.
//
//Environment:
//
//   Windows User-Mode Driver Framework (UMDF)

#include "Device.h"
#include "fxas21002.h"
#include <timeapi.h>

#include "Client.tmh"

// Gyroscope FXAS21002 Unique ID
// {63B5DCA0-2ADF-46BC-A8B2-B2A0847171AD}
DEFINE_GUID(GUID_FXAS21002 ,
    0x63b5dca0, 0x2adf, 0x46bc, 0xa8, 0xb2, 0xb2, 0xa0, 0x84, 0x71, 0x71, 0xad);

// Helper function for initializing FXAS21002. Returns status.
inline NTSTATUS InitSensorCollection(
    _In_ ULONG CollectionListCount,
    _Outptr_ PSENSOR_COLLECTION_LIST *CollectionList,
    _In_ SENSOROBJECT SensorInstance)                   // SENSOROBJECT for sensor instance
{
    WDF_OBJECT_ATTRIBUTES MemoryAttributes;
    WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
    MemoryAttributes.ParentObject = SensorInstance;

    WDFMEMORY MemoryHandle = NULL;
    ULONG MemorySize = SENSOR_COLLECTION_LIST_SIZE(CollectionListCount);
    NTSTATUS Status = WdfMemoryCreate(&MemoryAttributes,
                                      PagedPool,
                                      SENSORV2_POOL_TAG_FXAS_GYROSCOPE,
                                      MemorySize,
                                      &MemoryHandle,
                                      reinterpret_cast<PVOID*>(CollectionList));
    if (!NT_SUCCESS(Status) || nullptr == *CollectionList)
    {
        Status = STATUS_UNSUCCESSFUL;
        TraceError("GYRO %!FUNC! WdfMemoryCreate failed %!STATUS!", Status);
        return Status;
    }

    SENSOR_COLLECTION_LIST_INIT(*CollectionList, MemorySize);
    (*CollectionList)->Count = CollectionListCount;
    return Status;
}

// Helper function for initializing FXAS21002GyroDevice. Returns status.
inline NTSTATUS InitSensorProperty(
    _In_ ULONG PropertyListCount,
    _Outptr_ PSENSOR_PROPERTY_LIST *PropertyList,
    _In_ SENSOROBJECT SensorInstance)               // SENSOROBJECT for sensor instance
{
    WDF_OBJECT_ATTRIBUTES MemoryAttributes;
    WDF_OBJECT_ATTRIBUTES_INIT(&MemoryAttributes);
    MemoryAttributes.ParentObject = SensorInstance;

    WDFMEMORY MemoryHandle = NULL;
    ULONG MemorySize = SENSOR_PROPERTY_LIST_SIZE(PropertyListCount);
    NTSTATUS Status = WdfMemoryCreate(&MemoryAttributes,
                                      PagedPool,
                                      SENSORV2_POOL_TAG_FXAS_GYROSCOPE,
                                      MemorySize,
                                      &MemoryHandle,
                                      reinterpret_cast<PVOID*>(PropertyList));
    if (!NT_SUCCESS(Status) || nullptr == *PropertyList)
    {
        Status = STATUS_UNSUCCESSFUL;
        TraceError("GYRO %!FUNC! WdfMemoryCreate failed %!STATUS!", Status);
        return Status;
    }

    SENSOR_PROPERTY_LIST_INIT(*PropertyList, MemorySize);
    (*PropertyList)->Count = PropertyListCount;
    return Status;
}

// This routine initializes the sensor to its default properties
NTSTATUS FXAS21002GyroDevice::Initialize(
    _In_ WDFDEVICE Device,            // WDFDEVICE object
    _In_ SENSOROBJECT SensorInstance) // SENSOROBJECT for each sensor instance
{
    SENSOR_FunctionEnter();

    // Store device and instance
    m_Device = Device;
    m_SensorInstance = SensorInstance;
    m_Started = false;

    // Create Lock
    NTSTATUS Status = WdfWaitLockCreate(WDF_NO_OBJECT_ATTRIBUTES, &(m_I2CWaitLock));
    if (!NT_SUCCESS(Status))
    {
        TraceError("GYRO %!FUNC! WdfWaitLockCreate failed %!STATUS!", Status);
    }

    // Sensor Enumeration Properties
    if (NT_SUCCESS(Status))
    {
        Status = InitSensorCollection(SENSOR_ENUMERATION_PROPERTIES_COUNT, &m_pEnumerationProperties, SensorInstance);
        if (NT_SUCCESS(Status))
        {
            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_TYPE].Key = DEVPKEY_Sensor_Type;
            InitPropVariantFromCLSID(GUID_SensorType_Gyrometer3D,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_TYPE].Value));
        
            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MANUFACTURER].Key = DEVPKEY_Sensor_Manufacturer;
            InitPropVariantFromString(SENSOR_GYROSCOPE_MANUFACTURER,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MANUFACTURER].Value));
        
            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MODEL].Key = DEVPKEY_Sensor_Model;
            InitPropVariantFromString(SENSOR_FXAS21002_MODEL,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MODEL].Value));
            
            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CONNECTION_TYPE].Key = DEVPKEY_Sensor_ConnectionType;
            // The DEVPKEY_Sensor_ConnectionType values match the SensorConnectionType enumeration
            InitPropVariantFromUInt32(0, // 0: INTEGRATED, 1: ATTACHED, 2: EXTERNAL
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CONNECTION_TYPE].Value));

            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_PERSISTENT_UNIQUE_ID].Key = DEVPKEY_Sensor_PersistentUniqueId;
            InitPropVariantFromCLSID(GUID_FXAS21002,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_PERSISTENT_UNIQUE_ID].Value));

            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CATEGORY].Key = DEVPKEY_Sensor_Category;
            InitPropVariantFromCLSID(GUID_SensorCategory_Motion,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CATEGORY].Value));

 
            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_ISPRIMARY].Key = DEVPKEY_Sensor_IsPrimary;
            InitPropVariantFromBoolean(TRUE,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_ISPRIMARY].Value)); // This value should be set to TRUE in order for simple device orientation
                                                                                                 // to pick up GYRO sample readings from this sensor. If this value is set to FALSE
                                                                                                 // simple device orientation may ignore this sensor.
        }
    }

    // Supported Data-Fields
    if (NT_SUCCESS(Status))
    {
        Status = InitSensorProperty(SENSOR_DATA_COUNT, &m_pSupportedDataFields, SensorInstance);
        if (NT_SUCCESS(Status))
        {
            m_pSupportedDataFields->List[GYR_DATA_TIMESTAMP] = PKEY_SensorData_Timestamp;
            m_pSupportedDataFields->List[GYR_DATA_X] = PKEY_SensorData_AngularVelocityX_DegreesPerSecond;
            m_pSupportedDataFields->List[GYR_DATA_Y] = PKEY_SensorData_AngularVelocityY_DegreesPerSecond;
            m_pSupportedDataFields->List[GYR_DATA_Z] = PKEY_SensorData_AngularVelocityZ_DegreesPerSecond;
        }
    }

    // Sensor Data
    if (NT_SUCCESS(Status))
    {
        Status = InitSensorCollection(SENSOR_DATA_COUNT, &m_pSensorData, SensorInstance);
        if (NT_SUCCESS(Status))
        {
            FILETIME time;
            m_pSensorData->List[GYR_DATA_TIMESTAMP].Key = PKEY_SensorData_Timestamp;
            GetSystemTimePreciseAsFileTime(&time);
            InitPropVariantFromFileTime(&time, &(m_pSensorData->List[GYR_DATA_TIMESTAMP].Value));
        
            m_pSensorData->List[GYR_DATA_X].Key = PKEY_SensorData_AngularVelocityX_DegreesPerSecond;
            InitPropVariantFromFloat(0, &(m_pSensorData->List[GYR_DATA_X].Value));
        
            m_pSensorData->List[GYR_DATA_Y].Key = PKEY_SensorData_AngularVelocityY_DegreesPerSecond;
            InitPropVariantFromFloat(0, &(m_pSensorData->List[GYR_DATA_Y].Value));
        
            m_pSensorData->List[GYR_DATA_Z].Key = PKEY_SensorData_AngularVelocityZ_DegreesPerSecond;
            InitPropVariantFromFloat(0, &(m_pSensorData->List[GYR_DATA_Z].Value));
        }
    }

    // Sensor Properties
    if (NT_SUCCESS(Status))
    {
        Status = InitSensorCollection(SENSOR_PROPERTIES_COUNT, &m_pSensorProperties, SensorInstance);
        if (NT_SUCCESS(Status))
        {
            m_DataRate = _GetDataRateFromReportInterval(DEFAULT_GYROSCOPE_REPORT_INTERVAL);

            m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Key = PKEY_Sensor_State;
            InitPropVariantFromUInt32(SensorState_Initializing, &(m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Value));
        
            m_pSensorProperties->List[SENSOR_PROPERTY_MIN_DATA_INTERVAL].Key = PKEY_Sensor_MinimumDataInterval_Ms;
            InitPropVariantFromUInt32(GYROSCOPE_MIN_REPORT_INTERVAL, &(m_pSensorProperties->List[SENSOR_PROPERTY_MIN_DATA_INTERVAL].Value));
        
            m_pSensorProperties->List[SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE].Key = PKEY_Sensor_MaximumDataFieldSize_Bytes;
            InitPropVariantFromUInt32(CollectionsListGetMarshalledSize(m_pSensorData), &(m_pSensorProperties->List[SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE].Value));
        
            m_pSensorProperties->List[SENSOR_PROPERTY_TYPE].Key = PKEY_Sensor_Type;
            InitPropVariantFromCLSID(GUID_SensorType_Gyrometer3D, &(m_pSensorProperties->List[SENSOR_PROPERTY_TYPE].Value));
        }
    }

    // Data field properties
    if (NT_SUCCESS(Status))
    {
        Status = InitSensorCollection(SENSOR_DATA_FIELD_PROPERTIES_COUNT, &m_pDataFieldProperties, SensorInstance);
        if (NT_SUCCESS(Status))
        {
            m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RESOLUTION].Key = PKEY_SensorDataField_Resolution;
            InitPropVariantFromFloat(static_cast<float>(GYROSCOPE_CHANGE_SENSITIVITY_RESOLUTION), &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RESOLUTION].Value));
        
            m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MIN].Key = PKEY_SensorDataField_RangeMinimum;
            InitPropVariantFromFloat(static_cast<float>(GYROSCOPE_MIN_CHANGE_SENSITIVITY), &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MIN].Value));
        
            m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MAX].Key = PKEY_SensorDataField_RangeMaximum;
            InitPropVariantFromFloat(static_cast<float>(GYROSCOPE_MAX_CHANGE_SENSITIVITY), &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_RANGE_MAX].Value));
        }
    }

    // Set default threshold
    if (NT_SUCCESS(Status))
    {
        // note: COUNT-1 as timestamp does not have thresholds
        Status = InitSensorCollection(SENSOR_DATA_COUNT-1, &m_pThresholds, SensorInstance);
        if NT_SUCCESS(Status)
        {
            m_CachedThresholds.X = static_cast<float>(GYROSCOPE_DEFAULT_AXIS_THRESHOLD);
            m_CachedThresholds.Y = static_cast<float>(GYROSCOPE_DEFAULT_AXIS_THRESHOLD);
            m_CachedThresholds.Z = static_cast<float>(GYROSCOPE_DEFAULT_AXIS_THRESHOLD);
            
            m_pThresholds->List[GYR_DATA_X].Key = PKEY_SensorData_AngularVelocityX_DegreesPerSecond;
            InitPropVariantFromFloat(static_cast<float>(GYROSCOPE_DEFAULT_AXIS_THRESHOLD), &(m_pThresholds->List[GYR_DATA_X].Value));
        
            m_pThresholds->List[GYR_DATA_Y].Key = PKEY_SensorData_AngularVelocityY_DegreesPerSecond;
            InitPropVariantFromFloat(static_cast<float>(GYROSCOPE_DEFAULT_AXIS_THRESHOLD), &(m_pThresholds->List[GYR_DATA_Y].Value));
        
            m_pThresholds->List[GYR_DATA_Z].Key = PKEY_SensorData_AngularVelocityZ_DegreesPerSecond;
            InitPropVariantFromFloat(static_cast<float>(GYROSCOPE_DEFAULT_AXIS_THRESHOLD), &(m_pThresholds->List[GYR_DATA_Z].Value));
        }
    }

    // Reset the FirstSample flag
    if (NT_SUCCESS(Status))
    {
        m_FirstSample = true;
    }
    // Trace to this function in case of failure
    else
    {
        TraceError("GYRO %!FUNC! failed %!STATUS!", Status);
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

VOID FXAS21002GyroDevice::DeInit()
{
    // Delete lock
    if (NULL != m_I2CWaitLock)
    {
        WdfObjectDelete(m_I2CWaitLock);
        m_I2CWaitLock = NULL;
    }

    // Delete sensor instance
    if (NULL != m_SensorInstance)
    {
        WdfObjectDelete(m_SensorInstance);
    }
}

// This routine reads a single sample, compares threshold and pushes sample
// to sensor class extension. This routine is protected by the caller.
NTSTATUS FXAS21002GyroDevice::GetData() 
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Read the device data
    BYTE DataBuffer[FXAS21002_DATA_SIZE_BYTES];
    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    Status = I2CSensorReadRegister(m_I2CIoTarget, FXAS21002_OUT_X_MSB, &DataBuffer[0], sizeof(DataBuffer));
    WdfWaitLockRelease(m_I2CWaitLock);
    if (!NT_SUCCESS(Status))
    {
        TraceError("GYRO %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", FXAS21002_OUT_X_MSB, Status);
    }
    else
    {
        bool DataReady = false;

        // Perform data conversion
        SHORT xRaw = static_cast<SHORT>((DataBuffer[0] << 8) | DataBuffer[1]);
        SHORT yRaw = static_cast<SHORT>((DataBuffer[2] << 8) | DataBuffer[3]);
        SHORT zRaw = static_cast<SHORT>((DataBuffer[4] << 8) | DataBuffer[5]);
    
        const float ScaleFactor = 0.0625F;
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
            TraceInformation("GYRO %!FUNC! Data did NOT meet the threshold");
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Services a hardware interrupt.
BOOLEAN FXAS21002GyroDevice::OnInterruptIsr(
    _In_ WDFINTERRUPT Interrupt,        // Handle to a framework interrupt object
    _In_ ULONG /*MessageID*/)           // If the device is using message-signaled interrupts (MSIs),
                                        // this parameter is the message number that identifies the
                                        // device's hardware interrupt message. Otherwise, this value is 0.
{
    BOOLEAN InterruptRecognized = FALSE;
    PFXAS21002GyroDevice pGyroDevice = nullptr;

    SENSOR_FunctionEnter();

    // Get the sensor instance
    ULONG SensorInstanceCount = 1;
    SENSOROBJECT SensorInstance = NULL;
    NTSTATUS Status = SensorsCxDeviceGetSensorList(WdfInterruptGetDevice(Interrupt), &SensorInstance, &SensorInstanceCount);
    if (!NT_SUCCESS(Status) || 0 == SensorInstanceCount || NULL == SensorInstance)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! SensorsCxDeviceGetSensorList failed %!STATUS!", Status);
    }

    // Get the device context
    else // if (NT_SUCCESS(Status))
    {
        pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
        if (nullptr == pGyroDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("GYRO %!FUNC! GetFXAS21002GyroContextFromSensorInstance failed %!STATUS!", Status);
        }
    }

    // Read the interrupt source
    if (NT_SUCCESS(Status))
    {
        BYTE IntSrcBuffer = 0;
        WdfWaitLockAcquire(pGyroDevice->m_I2CWaitLock, NULL);
        Status = I2CSensorReadRegister(pGyroDevice->m_I2CIoTarget, FXAS21002_INT_SRC_FLAG, &IntSrcBuffer, sizeof(IntSrcBuffer));
        WdfWaitLockRelease(pGyroDevice->m_I2CWaitLock);
    
        if (!NT_SUCCESS(Status)) 
        {
            TraceError("GYRO %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", FXAS21002_INT_SRC_FLAG, Status);
        }
        else if ((IntSrcBuffer & FXAS21002_INT_SRC_FLAG_SRC_DRDY_MASK) == 0)
        {      
            TraceError("%!FUNC! Interrupt source not recognized");
        }
        else
        {
            InterruptRecognized = TRUE;
            BOOLEAN WorkItemQueued = WdfInterruptQueueWorkItemForIsr(Interrupt);
            TraceVerbose("%!FUNC! Work item %s queued for interrupt", WorkItemQueued ? "" : " already");
        }
    }

    SENSOR_FunctionExit(Status);
    return InterruptRecognized;
}

// Processes interrupt information that the driver's EvtInterruptIsr callback function has stored.
VOID FXAS21002GyroDevice::OnInterruptWorkItem(
    _In_ WDFINTERRUPT Interrupt,            // Handle to a framework object
    _In_ WDFOBJECT /*AssociatedObject*/)    // A handle to the framework device object that 
                                            // the driver passed to WdfInterruptCreate.
{
    PFXAS21002GyroDevice pGyroDevice = nullptr;

    SENSOR_FunctionEnter();

    // Get the sensor instance
    ULONG SensorInstanceCount = 1;
    SENSOROBJECT SensorInstance = NULL;
    NTSTATUS Status = SensorsCxDeviceGetSensorList(WdfInterruptGetDevice(Interrupt), &SensorInstance, &SensorInstanceCount);
    if (!NT_SUCCESS(Status) || 0 == SensorInstanceCount || NULL == SensorInstance)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! SensorsCxDeviceGetSensorList failed %!STATUS!", Status);
    }

    // Get the device context
    else //if (NT_SUCCESS(Status))
    {
        pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
        if (nullptr == pGyroDevice) 
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("GYRO %!FUNC! GetFXAS21002GyroContextFromSensorInstance failed %!STATUS!", Status);
        }
    }

    // Read the device data
    if (NT_SUCCESS(Status))
    {
        WdfInterruptAcquireLock(Interrupt);
        Status = pGyroDevice->GetData();
        WdfInterruptReleaseLock(Interrupt);
        if (!NT_SUCCESS(Status) && STATUS_DATA_NOT_ACCEPTED != Status) 
        {
            TraceError("GYRO %!FUNC! GetData failed %!STATUS!", Status);
        }
    }

    SENSOR_FunctionExit(Status);
}

// Called by Sensor CLX to begin continously sampling the sensor.
NTSTATUS FXAS21002GyroDevice::OnStart(
    _In_ SENSOROBJECT SensorInstance)    // Sensor device object
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Get the device context
    PFXAS21002GyroDevice pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
    if (nullptr == pGyroDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! Sensor(0x%p) parameter is invalid %!STATUS!", SensorInstance, Status);
    }
    else if (!pGyroDevice->m_PoweredOn)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! Sensor is not powered on! %!STATUS!", Status);
    }
    else
    {
        WdfWaitLockAcquire(pGyroDevice->m_I2CWaitLock, NULL);
    
        // Set gyroscope to active mode
        BYTE RegisterValue = (pGyroDevice->m_DataRate.RateCode << FXAS21002_CTRL_REG1_DR_SHIFT) | FXAS21002_CTRL_REG1_MODE_ACTIVE;
        REGISTER_SETTING RegisterSetting = { FXAS21002_CTRL_REG1, RegisterValue};
        Status = I2CSensorWriteRegister(pGyroDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        WdfWaitLockRelease(pGyroDevice->m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("GYRO %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
        }


        if (NT_SUCCESS(Status))
        {
            pGyroDevice->m_FirstSample = true;
            pGyroDevice->m_Started = true;
            
            InitPropVariantFromUInt32(SensorState_Active, &(pGyroDevice->m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Value));
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to stop continously sampling the sensor.
NTSTATUS FXAS21002GyroDevice::OnStop(
    _In_ SENSOROBJECT SensorInstance)   // Sensor device object
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Get the device context
    PFXAS21002GyroDevice pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
    if (nullptr == pGyroDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! Sensor(0x%p) parameter is invalid %!STATUS!", SensorInstance, Status);
    }
    else
    {
        pGyroDevice->m_Started = false;
        WdfWaitLockAcquire(pGyroDevice->m_I2CWaitLock, NULL);
    
        // Set gyroscope to standby
        REGISTER_SETTING RegisterSetting = { FXAS21002_CTRL_REG1, 0 };
        Status = I2CSensorWriteRegister(pGyroDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        if (!NT_SUCCESS(Status)) 
        {
            TraceError("GYRO %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
        }

        //Clear stale interrupts
        RegisterSetting = { FXAS21002_INT_SRC_FLAG, 0 };
        Status = I2CSensorWriteRegister(pGyroDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        WdfWaitLockRelease(pGyroDevice->m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("GYRO %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
        }
        else
        {
            InitPropVariantFromUInt32(SensorState_Idle, &(pGyroDevice->m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Value));
        }
        
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to get supported data fields. The typical usage is to call
// this function once with buffer pointer as NULL to acquire the required size 
// for the buffer, allocate buffer, then call the function again to retrieve 
// sensor information.
NTSTATUS FXAS21002GyroDevice::OnGetSupportedDataFields(
    _In_ SENSOROBJECT SensorInstance,          // Sensor device object
    _Inout_opt_ PSENSOR_PROPERTY_LIST pFields, // Pointer to a list of supported properties
    _Out_ PULONG pSize)                        // Number of bytes for the list of supported properties
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! pSize: Invalid parameter! %!STATUS!", Status);
    }
    else
    {
        *pSize = 0;

        // Get the device context
        PFXAS21002GyroDevice pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
        if (nullptr == pGyroDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("GYRO %!FUNC! Invalid parameters! %!STATUS!", Status);
        }
        else if (nullptr == pFields)
        {
            // Just return size
            *pSize = pGyroDevice->m_pSupportedDataFields->AllocatedSizeInBytes;
        }
        else 
        {
            if (pFields->AllocatedSizeInBytes < pGyroDevice->m_pSupportedDataFields->AllocatedSizeInBytes)
            {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                TraceError("GYRO %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            }
            else
            {
                // Fill out data
                Status = PropertiesListCopy(pFields, pGyroDevice->m_pSupportedDataFields);
                if (!NT_SUCCESS(Status))
                {
                    TraceError("GYRO %!FUNC! PropertiesListCopy failed %!STATUS!", Status);
                }
                else
                {
                    *pSize = pGyroDevice->m_pSupportedDataFields->AllocatedSizeInBytes;
                }
            }
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to get sensor properties. The typical usage is to call
// this function once with buffer pointer as NULL to acquire the required size 
// for the buffer, allocate buffer, then call the function again to retrieve 
// sensor information.
NTSTATUS FXAS21002GyroDevice::OnGetProperties(
    _In_ SENSOROBJECT SensorInstance,                // Sensor device object
    _Inout_opt_ PSENSOR_COLLECTION_LIST pProperties, // Pointer to a list of sensor properties
    _Out_ PULONG pSize)                              // Number of bytes for the list of sensor properties
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! pSize: Invalid parameter! %!STATUS!", Status);
    }
    else
    {
        *pSize = 0;

        // Get the device context
        PFXAS21002GyroDevice pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
        if (nullptr == pGyroDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("GYRO %!FUNC! Invalid parameters! %!STATUS!", Status);
        }
        else if (nullptr == pProperties)
        {
            // Just return size
            *pSize = CollectionsListGetMarshalledSize(pGyroDevice->m_pSensorProperties);
        }
        else
        {
            if (pProperties->AllocatedSizeInBytes < CollectionsListGetMarshalledSize(pGyroDevice->m_pSensorProperties))
            {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                TraceError("GYRO %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            }
            else
            {
                // Fill out all data
                Status = CollectionsListCopyAndMarshall(pProperties, pGyroDevice->m_pSensorProperties);
                if (!NT_SUCCESS(Status))
                {
                    TraceError("GYRO %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
                }
                else
                {
                    *pSize = CollectionsListGetMarshalledSize(pGyroDevice->m_pSensorProperties);
                }
            }
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to get data field properties. The typical usage is to call
// this function once with buffer pointer as NULL to acquire the required size 
// for the buffer, allocate buffer, then call the function again to retrieve 
// sensor information.
NTSTATUS FXAS21002GyroDevice::OnGetDataFieldProperties(
    _In_ SENSOROBJECT SensorInstance,                   // Sensor device object
    _In_ const PROPERTYKEY *pDataField,                 // Pointer to the propertykey of requested property
    _Inout_opt_ PSENSOR_COLLECTION_LIST pProperties,    // Pointer to a list of sensor properties
    _Out_ PULONG pSize)                                 // Number of bytes for the list of sensor properties
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! pSize: Invalid parameter! %!STATUS!", Status);
    }
    else
    {
        *pSize = 0;

        // Get the device context
        PFXAS21002GyroDevice pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
        if (nullptr == pGyroDevice || nullptr == pDataField)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("GYRO %!FUNC! Invalid parameters! %!STATUS!", Status);
        }
        else if (!IsEqualPropertyKey(*pDataField, pGyroDevice->m_pSupportedDataFields->List[GYR_DATA_X]) &&
                 !IsEqualPropertyKey(*pDataField, pGyroDevice->m_pSupportedDataFields->List[GYR_DATA_Y]) &&
                 !IsEqualPropertyKey(*pDataField, pGyroDevice->m_pSupportedDataFields->List[GYR_DATA_Z]))
        {
            Status = STATUS_NOT_SUPPORTED;
            TraceError("GYRO %!FUNC! ADXL345 does NOT have properties for this data field. Failed %!STATUS!", Status);
        }
        else if (nullptr == pProperties)
        {
            // Just return size
            *pSize = CollectionsListGetMarshalledSize(pGyroDevice->m_pDataFieldProperties);
        }
        else
        {
            if (pProperties->AllocatedSizeInBytes < CollectionsListGetMarshalledSize(pGyroDevice->m_pDataFieldProperties))
            {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                TraceError("GYRO %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            }
            else
            {
                // Fill out all data
                Status = CollectionsListCopyAndMarshall(pProperties, pGyroDevice->m_pDataFieldProperties);
                if (!NT_SUCCESS(Status))
                {
                    TraceError("GYRO %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
                }
                else
                {
                    *pSize = CollectionsListGetMarshalledSize(pGyroDevice->m_pDataFieldProperties);
                }
            }
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to get sampling rate of the sensor.
NTSTATUS FXAS21002GyroDevice::OnGetDataInterval(
    _In_ SENSOROBJECT SensorInstance,   // Sensor device object
    _Out_ PULONG pDataRateMs)           // Sampling rate in milliseconds
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    PFXAS21002GyroDevice pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
    if (nullptr == pGyroDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! Invalid parameters! %!STATUS!", Status);
    }
    else if (nullptr == pDataRateMs) 
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! Invalid parameters! %!STATUS!", Status);
    }
    else
    {
        *pDataRateMs = pGyroDevice->m_DataRate.DataRateInterval;
        TraceInformation("%!FUNC! giving data rate %lu", *pDataRateMs);
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to set sampling rate of the sensor.
NTSTATUS FXAS21002GyroDevice::OnSetDataInterval(
    _In_ SENSOROBJECT SensorInstance, // Sensor device object
    _In_ ULONG DataRateMs)            // Sampling rate in milliseconds
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Get the device context
    PFXAS21002GyroDevice pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
    if (nullptr == pGyroDevice || DataRateMs < GYROSCOPE_MIN_REPORT_INTERVAL)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! Invalid parameters! %!STATUS!", Status);
    }
    else
    {
        
        if (pGyroDevice->m_Started) 
        {
            pGyroDevice->m_Started = false;
        
            WdfWaitLockAcquire(pGyroDevice->m_I2CWaitLock, NULL);
        
            // Set to standby mode
            REGISTER_SETTING RegisterSetting = { FXAS21002_CTRL_REG1, 0 };
            Status = I2CSensorWriteRegister(pGyroDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                WdfWaitLockRelease(pGyroDevice->m_I2CWaitLock);
                TraceError("GYRO %!FUNC! Failed to disable interrupts. %!STATUS!", Status);
            }

            // Get new data rate, pass it to the gyroscope
            else // (if NT_SUCCESS(Status))
            {
                pGyroDevice->m_DataRate = _GetDataRateFromReportInterval(DataRateMs);
                TraceInformation("GYRO %!FUNC! New data rate: RateCode = %lu, Rate = %lu", pGyroDevice->m_DataRate.RateCode, pGyroDevice->m_DataRate.DataRateInterval);
                BYTE RegisterValue = (pGyroDevice->m_DataRate.RateCode << FXAS21002_CTRL_REG1_DR_SHIFT);
                RegisterSetting = { FXAS21002_CTRL_REG1, RegisterValue };
                Status = I2CSensorWriteRegister(pGyroDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
                if (!NT_SUCCESS(Status))
                {
                    TraceError("GYRO %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                }

                //Set gyroscoppe to active mode - include new data rate
                else
                {
                    RegisterValue = (pGyroDevice->m_DataRate.RateCode << FXAS21002_CTRL_REG1_DR_SHIFT) | FXAS21002_CTRL_REG1_MODE_ACTIVE;
                    RegisterSetting = { FXAS21002_CTRL_REG1, RegisterValue };
                    Status = I2CSensorWriteRegister(pGyroDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
                    if (!NT_SUCCESS(Status))
                    {
                        WdfWaitLockRelease(pGyroDevice->m_I2CWaitLock);
                        TraceError("GYRO %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                    }
                }
                WdfWaitLockRelease(pGyroDevice->m_I2CWaitLock);
            }

            pGyroDevice->m_Started = true;
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to get data thresholds. The typical usage is to call
// this function once with buffer pointer as NULL to acquire the required size
// for the buffer, allocate buffer, then call the function again to retrieve
// sensor information.
NTSTATUS FXAS21002GyroDevice::OnGetDataThresholds(
    _In_ SENSOROBJECT SensorInstance,                   // Sensor Device Object
    _Inout_opt_ PSENSOR_COLLECTION_LIST pThresholds,    // Pointer to a list of sensor thresholds
    _Out_ PULONG pSize)                                 // Number of bytes for the list of sensor thresholds
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! pSize: Invalid parameter! %!STATUS!", Status);
    }
    else
    {
        *pSize = 0;
    
        PFXAS21002GyroDevice pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
        if (nullptr == pGyroDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("GYRO %!FUNC! Invalid parameters! %!STATUS!", Status);
        }
        else if (nullptr == pThresholds)
        {
            // Just return size
            *pSize = CollectionsListGetMarshalledSize(pGyroDevice->m_pThresholds);
        }
        else
        {
            if (pThresholds->AllocatedSizeInBytes < CollectionsListGetMarshalledSize(pGyroDevice->m_pThresholds))
            {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                TraceError("GYRO %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            }
            else
            {
                // Fill out all data
                Status = CollectionsListCopyAndMarshall(pThresholds, pGyroDevice->m_pThresholds);
                if (!NT_SUCCESS(Status))
                {
                    TraceError("GYRO %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
                }
                else
                {
                    *pSize = CollectionsListGetMarshalledSize(pGyroDevice->m_pThresholds);
                }
            }
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to set data thresholds.
NTSTATUS FXAS21002GyroDevice::OnSetDataThresholds(
    _In_ SENSOROBJECT SensorInstance,           // Sensor Device Object
    _In_ PSENSOR_COLLECTION_LIST pThresholds)   // Pointer to a list of sensor thresholds
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    PFXAS21002GyroDevice pGyroDevice = GetFXAS21002GyroContextFromSensorInstance(SensorInstance);
    if (nullptr == pGyroDevice) 
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("GYRO %!FUNC! Sensor(0x%p) parameter is invalid %!STATUS!", SensorInstance, Status);
    }

    else // if (NT_SUCCESS(Status))
    {
        for (ULONG i = 0; i < pThresholds->Count; i++)
        {
            Status = PropKeyFindKeySetPropVariant(pGyroDevice->m_pThresholds, &(pThresholds->List[i].Key), true, &(pThresholds->List[i].Value));
            if (!NT_SUCCESS(Status))
            {
                Status = STATUS_INVALID_PARAMETER;
                TraceError("GYRO %!FUNC! FXAS21002 does NOT have threshold for this data field. Failed %!STATUS!", Status);
                break;
            }
        }
    }

    if (NT_SUCCESS(Status))
    {
        Status = PropKeyFindKeyGetFloat(pGyroDevice->m_pThresholds, &PKEY_SensorData_AngularVelocityX_DegreesPerSecond, &(pGyroDevice->m_CachedThresholds.X));
        if (!NT_SUCCESS(Status))
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("GYRO %!FUNC! PropKeyFindKeyGetFloat for X failed! %!STATUS!", Status);
        }
    }

    if (NT_SUCCESS(Status))
    {
        Status = PropKeyFindKeyGetFloat(pGyroDevice->m_pThresholds, &PKEY_SensorData_AngularVelocityY_DegreesPerSecond, &(pGyroDevice->m_CachedThresholds.Y));
        if (!NT_SUCCESS(Status))
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("GYRO %!FUNC! PropKeyFindKeyGetFloat for Y failed! %!STATUS!", Status);
        }
    }

    if (NT_SUCCESS(Status))
    {
        Status = PropKeyFindKeyGetFloat(pGyroDevice->m_pThresholds, &PKEY_SensorData_AngularVelocityZ_DegreesPerSecond, &(pGyroDevice->m_CachedThresholds.Z));
        if (!NT_SUCCESS(Status))
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("GYRO %!FUNC! PropKeyFindKeyGetFloat for Z failed! %!STATUS!", Status);
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to handle IOCTLs that clx does not support
NTSTATUS FXAS21002GyroDevice::OnIoControl(
    _In_ SENSOROBJECT /*SensorInstance*/, // WDF queue object
    _In_ WDFREQUEST /*Request*/,          // WDF request object
    _In_ size_t /*OutputBufferLength*/,   // number of bytes to retrieve from output buffer
    _In_ size_t /*InputBufferLength*/,    // number of bytes to retrieve from input buffer
    _In_ ULONG /*IoControlCode*/)         // IOCTL control code
{
    NTSTATUS Status = STATUS_NOT_SUPPORTED;

    SENSOR_FunctionEnter();

    SENSOR_FunctionExit(Status);
    return Status;
}
