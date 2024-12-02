//Copyright (C) Microsoft Corporation, All Rights Reserved.
//Copyright 2022 NXP
// 
//Abstract:
//
//    This module contains the implementation of driver callback functions
//    from clx to ISL29023 light sensor.
//
//Environment:
//
//   Windows User-Mode Driver Framework (UMDF)


#include "Device.h"
#include "Isl29023.h"

#include <timeapi.h>

#include "Client.tmh"


// Analog Isl29023 Unique ID
// {698A8599-00A4-4B8C-B425-D4A38FC816F7}
DEFINE_GUID(GUID_AlsDevice_UniqueID,
    0x698a8599, 0xa4, 0x4b8c, 0xb4, 0x25, 0xd4, 0xa3, 0x8f, 0xc8, 0x16, 0xf7);

// Helper function for initializing AlsDevice. Returns status.
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
                                      SENSORV2_POOL_TAG_LIGHTSENSOR,
                                      MemorySize,
                                      &MemoryHandle,
                                      reinterpret_cast<PVOID*>(CollectionList));
    if (!NT_SUCCESS(Status) || nullptr == *CollectionList)
    {
        Status = STATUS_UNSUCCESSFUL;
        TraceError("ALS %!FUNC! WdfMemoryCreate failed %!STATUS!", Status);
        return Status;
    }

    SENSOR_COLLECTION_LIST_INIT(*CollectionList, MemorySize);
    (*CollectionList)->Count = CollectionListCount;
    return Status;
}

// Helper function for initializing AlsDevice. Returns status.
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
                                      SENSORV2_POOL_TAG_LIGHTSENSOR,
                                      MemorySize,
                                      &MemoryHandle,
                                      reinterpret_cast<PVOID*>(PropertyList));
    if (!NT_SUCCESS(Status) || nullptr == *PropertyList)
    {
        Status = STATUS_UNSUCCESSFUL;
        TraceError("ALS %!FUNC! WdfMemoryCreate failed %!STATUS!", Status);
        return Status;
    }

    SENSOR_PROPERTY_LIST_INIT(*PropertyList, MemorySize);
    (*PropertyList)->Count = PropertyListCount;
    return Status;
}

// This routine initializes the sensor to its default properties
NTSTATUS AlsDevice::Initialize(
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
        TraceError("ALS %!FUNC! WdfWaitLockCreate failed %!STATUS!", Status);
    }

    // Create timer object for polling sensor samples
    WDF_OBJECT_ATTRIBUTES TimerAttributes;
    WDF_TIMER_CONFIG TimerConfig;
    WDF_TIMER_CONFIG_INIT(&TimerConfig, AlsDevice::OnTimerExpire);
    WDF_OBJECT_ATTRIBUTES_INIT(&TimerAttributes);
    TimerAttributes.ParentObject = SensorInstance;
    TimerAttributes.ExecutionLevel = WdfExecutionLevelPassive;

    Status = WdfTimerCreate(&TimerConfig, &TimerAttributes, &m_Timer);
    if (!NT_SUCCESS(Status))
    {
        TraceError("ALS %!FUNC! WdfTimerCreate failed %!STATUS!", Status);
    }

    // Sensor Enumeration Properties
    if (NT_SUCCESS(Status))
    {
        Status = InitSensorCollection(SENSOR_ENUMERATION_PROPERTIES_COUNT, &m_pEnumerationProperties, SensorInstance);
        if (NT_SUCCESS(Status))
        {
            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_TYPE].Key = DEVPKEY_Sensor_Type;
            InitPropVariantFromCLSID(GUID_SensorType_AmbientLight,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_TYPE].Value));
        
            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MANUFACTURER].Key = DEVPKEY_Sensor_Manufacturer;
            InitPropVariantFromString(SENSOR_MANUFACTURER,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MANUFACTURER].Value));
        
            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MODEL].Key = DEVPKEY_Sensor_Model;
            InitPropVariantFromString(SENSOR_MODEL,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_MODEL].Value));
            
            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CONNECTION_TYPE].Key = DEVPKEY_Sensor_ConnectionType;
            // The DEVPKEY_Sensor_ConnectionType values match the SensorConnectionType enumeration
            InitPropVariantFromUInt32(0, // 0: INTEGRATED, 1: ATTACHED, 2: EXTERNAL
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CONNECTION_TYPE].Value));

            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_PERSISTENT_UNIQUE_ID].Key = DEVPKEY_Sensor_PersistentUniqueId;
            InitPropVariantFromCLSID(GUID_AlsDevice_UniqueID,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_PERSISTENT_UNIQUE_ID].Value));

            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CATEGORY].Key = DEVPKEY_Sensor_Category;
            InitPropVariantFromCLSID(GUID_SensorCategory_Light,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_CATEGORY].Value));

            m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_ISPRIMARY].Key = DEVPKEY_Sensor_IsPrimary;
            InitPropVariantFromBoolean(TRUE,
                &(m_pEnumerationProperties->List[SENSOR_ENUMERATION_PROPERTY_ISPRIMARY].Value)); // This value should be set to TRUE in order for simple device orientation
                                                                                                 // to pick up ALS sample readings from this sensor. If this value is set to FALSE
                                                                                                 // simple device orientation may ignore this sensor.
        }
    }

    // Supported Data-Fields
    if (NT_SUCCESS(Status))
    {
        Status = InitSensorProperty(SENSOR_DATA_COUNT, &m_pSupportedDataFields, SensorInstance);
        if (NT_SUCCESS(Status))
        {
            m_pSupportedDataFields->List[SENSOR_DATA_TIMESTAMP] = PKEY_SensorData_Timestamp;
            m_pSupportedDataFields->List[SENSOR_DATA_LIGHTLEVEL_LUX] = PKEY_SensorData_LightLevel_Lux;
        }
    }

    // Sensor Data
    if (NT_SUCCESS(Status))
    {
        Status = InitSensorCollection(SENSOR_DATA_COUNT, &m_pSensorData, SensorInstance);
        if (NT_SUCCESS(Status))
        {
            FILETIME time;
            m_pSensorData->List[SENSOR_DATA_TIMESTAMP].Key = PKEY_SensorData_Timestamp;
            GetSystemTimePreciseAsFileTime(&time);
            InitPropVariantFromFileTime(&time, &(m_pSensorData->List[SENSOR_DATA_TIMESTAMP].Value));
        
            m_pSensorData->List[SENSOR_DATA_LIGHTLEVEL_LUX].Key = PKEY_SensorData_LightLevel_Lux;
            InitPropVariantFromFloat(0.0f, &(m_pSensorData->List[SENSOR_DATA_LIGHTLEVEL_LUX].Value));
        }
    }

    // Sensor Properties
    if (NT_SUCCESS(Status))
    {
        Status = InitSensorCollection(SENSOR_PROPERTIES_COUNT, &m_pSensorProperties, SensorInstance);
        if (NT_SUCCESS(Status))
        {
            m_DataRate = _GetDataRateFromReportInterval(DEFAULT_SENSOR_REPORT_INTERVAL);
    
            m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Key = PKEY_Sensor_State;
            InitPropVariantFromUInt32(SensorState_Initializing, &(m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Value));
        
            m_pSensorProperties->List[SENSOR_PROPERTY_MIN_DATA_INTERVAL].Key = PKEY_Sensor_MinimumDataInterval_Ms;
            InitPropVariantFromUInt32(SENSOR_MIN_REPORT_INTERVAL, &(m_pSensorProperties->List[SENSOR_PROPERTY_MIN_DATA_INTERVAL].Value));
        
            m_pSensorProperties->List[SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE].Key = PKEY_Sensor_MaximumDataFieldSize_Bytes;
            InitPropVariantFromUInt32(CollectionsListGetMarshalledSize(m_pSensorData), &(m_pSensorProperties->List[SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE].Value));
        
            m_pSensorProperties->List[SENSOR_PROPERTY_TYPE].Key = PKEY_Sensor_Type;
            InitPropVariantFromCLSID(GUID_SensorType_AmbientLight, &(m_pSensorProperties->List[SENSOR_PROPERTY_TYPE].Value));
        }
    }

    // Data field properties
    if (NT_SUCCESS(Status))
    {
        Status = InitSensorCollection(SENSOR_DATA_FIELD_PROPERTIES_COUNT, &m_pDataFieldProperties, SensorInstance);
        if (NT_SUCCESS(Status))
        {
            m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_ALS_RESOLUTION].Key = PKEY_SensorDataField_Resolution;
            InitPropVariantFromFloat(static_cast<float>(SENSOR_CHANGE_SENSITIVITY_RESOLUTION), &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_ALS_RESOLUTION].Value));
        
            m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_ALS_RANGE_MIN].Key = PKEY_SensorDataField_RangeMinimum;
            InitPropVariantFromFloat(static_cast<float>(SENSOR_MIN_LUX), &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_ALS_RANGE_MIN].Value));
        
            m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_ALS_RANGE_MAX].Key = PKEY_SensorDataField_RangeMaximum;
            InitPropVariantFromFloat(static_cast<float>(SENSOR_MAX_LUX), &(m_pDataFieldProperties->List[SENSOR_DATA_FIELD_PROPERTY_ALS_RANGE_MAX].Value));
        }
    }

    // Set default threshold
    if (NT_SUCCESS(Status))
    {
        // note: COUNT-1 as timestamp does not have thresholds
        Status = InitSensorCollection(SENSOR_DATA_COUNT-1, &m_pThresholds, SensorInstance);
        if NT_SUCCESS(Status)
        {
            m_CachedThresholds.Lux = static_cast<float>(SENSOR_DEFAULT_AXIS_THRESHOLD);

            m_pThresholds->List[SENSOR_DATA_LIGHTLEVEL_LUX].Key = PKEY_SensorData_LightLevel_Lux;
            InitPropVariantFromFloat(static_cast<float>(SENSOR_DEFAULT_AXIS_THRESHOLD), &(m_pThresholds->List[SENSOR_DATA_LIGHTLEVEL_LUX].Value));
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
        TraceError("ALS %!FUNC! failed %!STATUS!", Status);
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

VOID AlsDevice::DeInit()
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
NTSTATUS AlsDevice::GetData() 
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Read the device data
    BYTE DataBuffer[ISL29023_DATA_REPORT_SIZE_BYTES];
    WdfWaitLockAcquire(m_I2CWaitLock, NULL);
    Status = I2CSensorReadRegister(m_I2CIoTarget, ISL29023_DATA_LSB, &DataBuffer[0], sizeof(DataBuffer));
    WdfWaitLockRelease(m_I2CWaitLock);

    if (!NT_SUCCESS(Status))
    {
        TraceError("ALS %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", ISL29023_DATA_LSB, Status);
    }
    else
    {
        bool DataReady = false;

        // Perform data conversion
        USHORT Raw = static_cast<USHORT>((DataBuffer[1] << 8) | DataBuffer[0] );

        // Accuracy correction to abtain zero lux value.
        if (Raw > 0) Raw--;

        const float ScaleFactor = m_DataRate.DataResolution;
        AlsSample Sample = {};
        Sample.Lux = static_cast<float>(Raw * ScaleFactor);
 
        // Set data ready if this is the first sample or we have exceeded the thresholds
        if (m_FirstSample)
        {
            m_FirstSample = false;
            DataReady = true;
        }
        else if (fabsf(Sample.Lux - m_LastSample.Lux) >= m_CachedThresholds.Lux) 
        {
            DataReady = true;
        }
    
        if (DataReady) 
        {
            // Update values for SW thresholding and send data to class extension
            m_LastSample.Lux = Sample.Lux;
    
            // Save the data in the context
            InitPropVariantFromFloat(Sample.Lux, &(m_pSensorData->List[SENSOR_DATA_LIGHTLEVEL_LUX].Value));
    
            FILETIME Timestamp = {};
            GetSystemTimePreciseAsFileTime(&Timestamp);
            InitPropVariantFromFileTime(&Timestamp, &(m_pSensorData->List[SENSOR_DATA_TIMESTAMP].Value));
    
            SensorsCxSensorDataReady(m_SensorInstance, m_pSensorData);
        }
        else 
        {
            TraceInformation("ALS %!FUNC! Data did NOT meet the threshold");
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Services a hardware interrupt.
BOOLEAN AlsDevice::OnInterruptIsr(
    _In_ WDFINTERRUPT Interrupt,        // Handle to a framework interrupt object
    _In_ ULONG /*MessageID*/)           // If the device is using message-signaled interrupts (MSIs),
                                        // this parameter is the message number that identifies the
                                        // device's hardware interrupt message. Otherwise, this value is 0.
{
    BOOLEAN InterruptRecognized = FALSE;
    PAlsDevice pDevice = nullptr;

    SENSOR_FunctionEnter();

    // Get the sensor instance
    ULONG SensorInstanceCount = 1;
    SENSOROBJECT SensorInstance = NULL;
    NTSTATUS Status = SensorsCxDeviceGetSensorList(WdfInterruptGetDevice(Interrupt), &SensorInstance, &SensorInstanceCount);
    if (!NT_SUCCESS(Status) || 0 == SensorInstanceCount || NULL == SensorInstance)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! SensorsCxDeviceGetSensorList failed %!STATUS!", Status);
    }

    // Get the device context
    else // if (NT_SUCCESS(Status))
    {
        pDevice = GetAlsContextFromSensorInstance(SensorInstance);
        if (nullptr == pDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("ALS %!FUNC! GetAlsContextFromSensorInstance failed %!STATUS!", Status);
        }
    }

    // Read the interrupt source
    if (NT_SUCCESS(Status))
    {
        BYTE IntSrcBuffer = 0;
        WdfWaitLockAcquire(pDevice->m_I2CWaitLock, NULL);
        Status = I2CSensorReadRegister(pDevice->m_I2CIoTarget, ISL29023_COMMAND_I, &IntSrcBuffer, sizeof(IntSrcBuffer));
        WdfWaitLockRelease(pDevice->m_I2CWaitLock);
        TraceError("ALS %!FUNC! I2CSensorReadRegister  0x%02x", IntSrcBuffer);
    
        if (!NT_SUCCESS(Status)) 
        {
            TraceError("ALS %!FUNC! I2CSensorReadRegister from 0x%02x failed! %!STATUS!", ISL29023_COMMAND_I, Status);
        }
        else if ((IntSrcBuffer & ISL29023_COMMAND_I_INT_FLAG_MASK) == 0)
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
VOID AlsDevice::OnInterruptWorkItem(
    _In_ WDFINTERRUPT Interrupt,            // Handle to a framework object
    _In_ WDFOBJECT /*AssociatedObject*/)    // A handle to the framework device object that 
                                            // the driver passed to WdfInterruptCreate.
{
    PAlsDevice pDevice = nullptr;

    SENSOR_FunctionEnter();

    // Get the sensor instance
    ULONG SensorInstanceCount = 1;
    SENSOROBJECT SensorInstance = NULL;
    NTSTATUS Status = SensorsCxDeviceGetSensorList(WdfInterruptGetDevice(Interrupt), &SensorInstance, &SensorInstanceCount);
    if (!NT_SUCCESS(Status) || 0 == SensorInstanceCount || NULL == SensorInstance)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! SensorsCxDeviceGetSensorList failed %!STATUS!", Status);
    }

    // Get the device context
    else //if (NT_SUCCESS(Status))
    {
        pDevice = GetAlsContextFromSensorInstance(SensorInstance);
        if (nullptr == pDevice) 
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("ALS %!FUNC! GetAlsContextFromSensorInstance failed %!STATUS!", Status);
        }
    }

    // Read the device data
    if (NT_SUCCESS(Status))
    {
        WdfInterruptAcquireLock(Interrupt);
        Status = pDevice->GetData();       
        WdfInterruptReleaseLock(Interrupt);
        if (!NT_SUCCESS(Status) && STATUS_DATA_NOT_ACCEPTED != Status) 
        {
            TraceError("ALS %!FUNC! GetData failed %!STATUS!", Status);
        }
    }

    SENSOR_FunctionExit(Status);
}

// Called by Sensor CLX to begin continously sampling the sensor.
NTSTATUS AlsDevice::OnStart(
    _In_ SENSOROBJECT SensorInstance)    // Sensor device object
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Get the device context
    PAlsDevice pDevice = GetAlsContextFromSensorInstance(SensorInstance);
    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! Sensor(0x%p) parameter is invalid %!STATUS!", SensorInstance, Status);
    }
    else if (!pDevice->m_PoweredOn)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! Sensor is not powered on! %!STATUS!", Status);
    }
    else
    {
        WdfWaitLockAcquire(pDevice->m_I2CWaitLock, NULL);

        // Set DataRate by changing resolution
        pDevice->m_DataRate = _GetDataRateFromReportInterval(DEFAULT_SENSOR_REPORT_INTERVAL);
        REGISTER_SETTING RegisterSetting = { ISL29023_COMMAND_II, pDevice->m_DataRate.RateCode };
        Status = I2CSensorWriteRegister(pDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));   
        if (!NT_SUCCESS(Status))
        {
            WdfWaitLockRelease(pDevice->m_I2CWaitLock);
            TraceError("ALS %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
        }
        // Set continuous Measuring mode
        else // if (NT_SUCCESS(Status))
        {
            RegisterSetting = { ISL29023_COMMAND_I , ISL29023_COMMAND_I_MODE_ALS_CONTINUOUS };
            Status = I2CSensorWriteRegister(pDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            WdfWaitLockRelease(pDevice->m_I2CWaitLock);
            if (!NT_SUCCESS(Status))
            {  
                TraceError("ALS %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
            }
        }

        pDevice->m_FirstSample = true;
        pDevice->m_Started = true;
            
        InitPropVariantFromUInt32(SensorState_Active, &(pDevice->m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Value));
        
        // Start the sample polling timer.
        // Note1: the WDF timer is only as precise as the system resolution allows it to be.
        // In the case of the CO2 sensor, the reporting interval is 200 milliseconds. The default 
        // system resolution (15.6 milliseconds) is therefore fine enough to guarantee an accurate sample 
        // reporting interval. Some sensors using a lower reporting interval may want to reduce the system 
        // time resolution by calling into timeBeginPeriod() before starting the polling timer.
        //
        // Important consideration: calling into timeBeginPeriod() should be used with care as it has 
        // an adverse on the system performance and power consumption.
        //
        // Note2: The polling timer is configured to allow for the first sample to be reported immediately.
        // Some hardware may want to delay the first sample report a little to account for hardware start time.

        WdfTimerStart(pDevice->m_Timer, 0 );
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to stop continously sampling the sensor.
NTSTATUS AlsDevice::OnStop(
    _In_ SENSOROBJECT SensorInstance)   // Sensor device object
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Get the device context
    PAlsDevice pDevice = GetAlsContextFromSensorInstance(SensorInstance);
    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! Sensor(0x%p) parameter is invalid %!STATUS!", SensorInstance, Status);
    }
    else
    {
        pDevice->m_Started = false;

        WdfTimerStop(pDevice->m_Timer, TRUE);
    
        // Set Light sensor to Power Down mode
        REGISTER_SETTING RegisterSetting = { ISL29023_COMMAND_I , ISL29023_COMMAND_I_MODE_POWER_DOWN };
        WdfWaitLockAcquire(pDevice->m_I2CWaitLock, NULL);
        Status = I2CSensorWriteRegister(pDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
        WdfWaitLockRelease(pDevice->m_I2CWaitLock);
        if (!NT_SUCCESS(Status))
        {
            TraceError("ALS %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
        }
        else
        {
            InitPropVariantFromUInt32(SensorState_Idle, &(pDevice->m_pSensorProperties->List[SENSOR_PROPERTY_STATE].Value));
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to get supported data fields. The typical usage is to call
// this function once with buffer pointer as NULL to acquire the required size 
// for the buffer, allocate buffer, then call the function again to retrieve 
// sensor information.
NTSTATUS AlsDevice::OnGetSupportedDataFields(
    _In_ SENSOROBJECT SensorInstance,          // Sensor device object
    _Inout_opt_ PSENSOR_PROPERTY_LIST pFields, // Pointer to a list of supported properties
    _Out_ PULONG pSize)                        // Number of bytes for the list of supported properties
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! pSize: Invalid parameter! %!STATUS!", Status);
    }
    else
    {
        *pSize = 0;

        // Get the device context
        PAlsDevice pDevice = GetAlsContextFromSensorInstance(SensorInstance);
        if (nullptr == pDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("ALS %!FUNC! Invalid parameters! %!STATUS!", Status);
        }
        else if (nullptr == pFields)
        {
            // Just return size
            *pSize = pDevice->m_pSupportedDataFields->AllocatedSizeInBytes;
        }
        else 
        {
            if (pFields->AllocatedSizeInBytes < pDevice->m_pSupportedDataFields->AllocatedSizeInBytes)
            {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                TraceError("ALS %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            }
            else
            {
                // Fill out data
                Status = PropertiesListCopy(pFields, pDevice->m_pSupportedDataFields);
                if (!NT_SUCCESS(Status))
                {
                    TraceError("ALS %!FUNC! PropertiesListCopy failed %!STATUS!", Status);
                }
                else
                {
                    *pSize = pDevice->m_pSupportedDataFields->AllocatedSizeInBytes;
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
NTSTATUS AlsDevice::OnGetProperties(
    _In_ SENSOROBJECT SensorInstance,                // Sensor device object
    _Inout_opt_ PSENSOR_COLLECTION_LIST pProperties, // Pointer to a list of sensor properties
    _Out_ PULONG pSize)                              // Number of bytes for the list of sensor properties
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! pSize: Invalid parameter! %!STATUS!", Status);
    }
    else
    {
        *pSize = 0;

        // Get the device context
        PAlsDevice pDevice = GetAlsContextFromSensorInstance(SensorInstance);
        if (nullptr == pDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("ALS %!FUNC! Invalid parameters! %!STATUS!", Status);
        }
        else if (nullptr == pProperties)
        {
            // Just return size
            *pSize = CollectionsListGetMarshalledSize(pDevice->m_pSensorProperties);
        }
        else
        {
            if (pProperties->AllocatedSizeInBytes < CollectionsListGetMarshalledSize(pDevice->m_pSensorProperties))
            {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                TraceError("ALS %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            }
            else
            {
                // Fill out all data
                Status = CollectionsListCopyAndMarshall(pProperties, pDevice->m_pSensorProperties);
                if (!NT_SUCCESS(Status))
                {
                    TraceError("ALS %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
                }
                else
                {
                    *pSize = CollectionsListGetMarshalledSize(pDevice->m_pSensorProperties);
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
NTSTATUS AlsDevice::OnGetDataFieldProperties(
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
        TraceError("ALS %!FUNC! pSize: Invalid parameter! %!STATUS!", Status);
    }
    else
    {
        *pSize = 0;

        // Get the device context
        PAlsDevice pDevice = GetAlsContextFromSensorInstance(SensorInstance);
        if (nullptr == pDevice || nullptr == pDataField)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("ALS %!FUNC! Invalid parameters! %!STATUS!", Status);
        }
        else if (!IsEqualPropertyKey(*pDataField, pDevice->m_pSupportedDataFields->List[SENSOR_DATA_LIGHTLEVEL_LUX]))
        {
            Status = STATUS_NOT_SUPPORTED;
            TraceError("ALS %!FUNC! ISL29023 does NOT have properties for this data field. Failed %!STATUS!", Status);
        }
        else if (nullptr == pProperties)
        {
            // Just return size
            *pSize = CollectionsListGetMarshalledSize(pDevice->m_pDataFieldProperties);
        }
        else
        {
            if (pProperties->AllocatedSizeInBytes < CollectionsListGetMarshalledSize(pDevice->m_pDataFieldProperties))
            {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                TraceError("ALS %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            }
            else
            {
                // Fill out all data
                Status = CollectionsListCopyAndMarshall(pProperties, pDevice->m_pDataFieldProperties);
                if (!NT_SUCCESS(Status))
                {
                    TraceError("ALS %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
                }
                else
                {
                    *pSize = CollectionsListGetMarshalledSize(pDevice->m_pDataFieldProperties);
                }
            }
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to get sampling rate of the sensor.
NTSTATUS AlsDevice::OnGetDataInterval(
    _In_ SENSOROBJECT SensorInstance,   // Sensor device object
    _Out_ PULONG pDataRateMs)           // Sampling rate in milliseconds
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    PAlsDevice pDevice = GetAlsContextFromSensorInstance(SensorInstance);
    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! Invalid parameters! %!STATUS!", Status);
    }
    else if (nullptr == pDataRateMs) 
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! Invalid parameters! %!STATUS!", Status);
    }
    else
    {
        *pDataRateMs = pDevice->m_DataRate.DataRateInterval;
        TraceInformation("%!FUNC! giving data rate %lu", *pDataRateMs);
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to set sampling rate of the sensor.
NTSTATUS AlsDevice::OnSetDataInterval(
    _In_ SENSOROBJECT SensorInstance, // Sensor device object
    _In_ ULONG DataRateMs)            // Sampling rate in milliseconds
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    // Get the device context
    PAlsDevice pDevice = GetAlsContextFromSensorInstance(SensorInstance);
    if (nullptr == pDevice)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! Invalid parameters! %!STATUS!", Status);
    }
    else
    {
        if (pDevice->m_Started) 
        {
            pDevice->m_Started = false;
            WdfTimerStop(pDevice->m_Timer, TRUE);

            WdfWaitLockAcquire(pDevice->m_I2CWaitLock, NULL);
        
            // Set STANDBY mode
            REGISTER_SETTING RegisterSetting = { ISL29023_COMMAND_I , ISL29023_COMMAND_I_MODE_POWER_DOWN };
            Status = I2CSensorWriteRegister(pDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
            if (!NT_SUCCESS(Status))
            {
                WdfWaitLockRelease(pDevice->m_I2CWaitLock);
                TraceError("ALS %!FUNC! Failed to disable interrupts. %!STATUS!", Status);
            }
            // Update data rate in HW
            else // (if NT_SUCCESS(Status))
            {
                // Set data rate in HW
                pDevice->m_DataRate = _GetDataRateFromReportInterval(DataRateMs);
                RegisterSetting = { ISL29023_COMMAND_II, pDevice->m_DataRate.RateCode };
                Status = I2CSensorWriteRegister(pDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
                if (!NT_SUCCESS(Status))
                {
                    WdfWaitLockRelease(pDevice->m_I2CWaitLock);
                    TraceError("ALS %!FUNC! I2CSensorWriteRegister to 0x%02x failed! %!STATUS!", RegisterSetting.Register, Status);
                }
            }

            // Set to measuring mode
            if (NT_SUCCESS(Status))
            {
                pDevice->m_Started = true;
                pDevice->m_FirstSample = true;

                RegisterSetting = { ISL29023_COMMAND_I , ISL29023_COMMAND_I_MODE_ALS_CONTINUOUS };
                Status = I2CSensorWriteRegister(pDevice->m_I2CIoTarget, RegisterSetting.Register, &RegisterSetting.Value, sizeof(RegisterSetting.Value));
                WdfWaitLockRelease(pDevice->m_I2CWaitLock);
                if (!NT_SUCCESS(Status))
                {     
                    TraceError("ALS %!FUNC! Failed to disable interrupts. %!STATUS!", Status);
                }

                WdfTimerStart(pDevice->m_Timer, 0 );
            }
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to get data thresholds. The typical usage is to call
// this function once with buffer pointer as NULL to acquire the required size
// for the buffer, allocate buffer, then call the function again to retrieve
// sensor information.
NTSTATUS AlsDevice::OnGetDataThresholds(
    _In_ SENSOROBJECT SensorInstance,                   // Sensor Device Object
    _Inout_opt_ PSENSOR_COLLECTION_LIST pThresholds,    // Pointer to a list of sensor thresholds
    _Out_ PULONG pSize)                                 // Number of bytes for the list of sensor thresholds
{

    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    if (nullptr == pSize)
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! pSize: Invalid parameter! %!STATUS!", Status);
    }
    else
    {
        *pSize = 0;
    
        PAlsDevice pDevice = GetAlsContextFromSensorInstance(SensorInstance);
        if (nullptr == pDevice)
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("ALS %!FUNC! Invalid parameters! %!STATUS!", Status);
        }
        else if (nullptr == pThresholds)
        {
            // Just return size
            *pSize = CollectionsListGetMarshalledSize(pDevice->m_pThresholds);
        }
        else
        {
            if (pThresholds->AllocatedSizeInBytes < CollectionsListGetMarshalledSize(pDevice->m_pThresholds))
            {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                TraceError("ALS %!FUNC! Buffer is too small. Failed %!STATUS!", Status);
            }
            else
            {
                // Fill out all data
                Status = CollectionsListCopyAndMarshall(pThresholds, pDevice->m_pThresholds);
                if (!NT_SUCCESS(Status))
                {
                    TraceError("ALS %!FUNC! CollectionsListCopyAndMarshall failed %!STATUS!", Status);
                }
                else
                {
                    *pSize = CollectionsListGetMarshalledSize(pDevice->m_pThresholds);
                }
            }
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to set data thresholds.
NTSTATUS AlsDevice::OnSetDataThresholds(
    _In_ SENSOROBJECT SensorInstance,           // Sensor Device Object
    _In_ PSENSOR_COLLECTION_LIST pThresholds)   // Pointer to a list of sensor thresholds
{
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    PAlsDevice pDevice = GetAlsContextFromSensorInstance(SensorInstance);
    if (nullptr == pDevice) 
    {
        Status = STATUS_INVALID_PARAMETER;
        TraceError("ALS %!FUNC! Sensor(0x%p) parameter is invalid %!STATUS!", SensorInstance, Status);
    }

    else // if (NT_SUCCESS(Status))
    {
        for (ULONG i = 0; i < pThresholds->Count; i++)
        {
            Status = PropKeyFindKeySetPropVariant(pDevice->m_pThresholds, &(pThresholds->List[i].Key), true, &(pThresholds->List[i].Value));
            if (!NT_SUCCESS(Status))
            {
                Status = STATUS_INVALID_PARAMETER;
                TraceError("ALS %!FUNC! Isl29023 does NOT have threshold for this data field. Failed %!STATUS!", Status);
                break;
            }
        }
    }

    if (NT_SUCCESS(Status))
    {
        Status = PropKeyFindKeyGetFloat(pDevice->m_pThresholds, &PKEY_SensorData_LightLevel_Lux, &(pDevice->m_CachedThresholds.Lux));
        if (!NT_SUCCESS(Status))
        {
            Status = STATUS_INVALID_PARAMETER;
            TraceError("ALS %!FUNC! PropKeyFindKeyGetFloat for X failed! %!STATUS!", Status);
        }
    }

    SENSOR_FunctionExit(Status);
    return Status;
}

// Called by Sensor CLX to handle IOCTLs that clx does not support
NTSTATUS AlsDevice::OnIoControl(
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

// This callback is called when interval wait time has expired and driver is ready
// to collect new sample. The callback reads current value,
// pushes it up to CLX framework, and schedule next wake up time.
VOID
AlsDevice::OnTimerExpire(
    _In_ WDFTIMER Timer // WDF timer object
)
{
    PAlsDevice pDevice = nullptr;
    NTSTATUS Status = STATUS_SUCCESS;

    SENSOR_FunctionEnter();

    pDevice = GetAlsContextFromSensorInstance(WdfTimerGetParentObject(Timer));
    if (nullptr == pDevice)
    {
        Status = STATUS_INSUFFICIENT_RESOURCES;
        TraceError("ALS %!FUNC! GetAlsContextFromSensorInstance failed %!STATUS!", Status);
    }

    if (NT_SUCCESS(Status))
    {
        // Get data and push to clx
        Status = pDevice->GetData();
        if (!NT_SUCCESS(Status) && Status != STATUS_DATA_NOT_ACCEPTED)
        {
            TraceError("ALS %!FUNC! GetCstmData Failed %!STATUS!", Status);
        }

        // Schedule next wake up time
        if (FALSE != pDevice->m_PoweredOn &&
            FALSE != pDevice->m_Started)
        {
            LONGLONG WaitTimeHundredNanoseconds = 0;  // in unit of 100ns

            if (0 == pDevice->m_StartTime)
            {
                // in case we fail to get sensor start time, use static wait time
                WaitTimeHundredNanoseconds = WDF_REL_TIMEOUT_IN_MS(pDevice->m_DataRate.DataRateInterval);
            }
            else
            {
                ULONG CurrentTimeMs = 0;

                // dynamically calculate wait time to avoid jitter
                Status = GetPerformanceTime(&CurrentTimeMs);
                if (!NT_SUCCESS(Status))
                {
                    TraceError("PED %!FUNC! GetPerformanceTime %!STATUS!", Status);
                    WaitTimeHundredNanoseconds = WDF_REL_TIMEOUT_IN_MS(pDevice->m_DataRate.DataRateInterval);
                }
                else
                {
                    pDevice->m_SampleCount++;
                    if (CurrentTimeMs > (pDevice->m_StartTime + (pDevice->m_DataRate.DataRateInterval * (pDevice->m_SampleCount + 1))))
                    {
                        // If we skipped two or more beats, reschedule the timer with a zero due time to catch up on missing samples
                        WaitTimeHundredNanoseconds = 0;
                    }
                    else
                    {
                        WaitTimeHundredNanoseconds = (pDevice->m_StartTime +
                            (pDevice->m_DataRate.DataRateInterval * (pDevice->m_SampleCount + 1))) - CurrentTimeMs;
                    }
                    WaitTimeHundredNanoseconds = WDF_REL_TIMEOUT_IN_MS(WaitTimeHundredNanoseconds);
                }
            }
            WdfTimerStart(pDevice->m_Timer, WaitTimeHundredNanoseconds);
        }
    }

    SENSOR_FunctionExit(Status);
}