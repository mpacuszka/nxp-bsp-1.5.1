// Copyright (C) Microsoft Corporation, All Rights Reserved
// Copyright 2023 NXP
// 
// Abstract:
//
//  This module contains the type definitions for the client
//  driver's device callback class.
//
// Environment:
//
//  Windows User-Mode Driver Framework (WUDF)

#pragma once

#include <windows.h>
#include <wdf.h>
#include <cmath>
#include <timeapi.h>
#include <reshub.h>
#include <strsafe.h>

#include <SensorsDef.h>
#include <SensorsCx.h>
#include <sensorsutils.h>
#include <SensorsDriversUtils.h>

#include "LSM6DSOX.h"
#include "SensorsTrace.h"


/*++
Routine Description:
    Critical section lock/unlock to protect shared context
Return Value:
    None
--*/
#define Lock(lock)      { WdfWaitLockAcquire(lock, NULL); }
#define Unlock(lock)    { WdfWaitLockRelease(lock); }

#define SENSORV2_POOL_TAG_IMU           '2bmC'

//
// Sensor Enumeration Properties
//
typedef enum
{
    SENSOR_ENUMERATION_PROPERTY_TYPE = 0,
    SENSOR_ENUMERATION_PROPERTY_MANUFACTURER,
    SENSOR_ENUMERATION_PROPERTY_MODEL,
    SENSOR_ENUMERATION_PROPERTY_CONNECTION_TYPE,
    SENSOR_ENUMERATION_PROPERTY_PERSISTENT_UNIQUE_ID,
    SENSOR_ENUMERATION_PROPERTY_CATEGORY,
    SENSOR_ENUMERATION_PROPERTY_ISPRIMARY,
    SENSOR_ENUMERATION_PROPERTIES_COUNT
} SENSOR_ENUMERATION_PROPERTIES_INDEX;

enum class SensorConnectionType : ULONG
{
    Integrated = 0,
    Attached = 1,
    External = 2
};

//
// Data-field Properties
//
typedef enum
{
    SENSOR_DATA_FIELD_PROPERTY_RESOLUTION = 0,
    SENSOR_DATA_FIELD_PROPERTY_RANGE_MIN,
    SENSOR_DATA_FIELD_PROPERTY_RANGE_MAX,
    SENSOR_DATA_FIELD_PROPERTY_COUNT
} SENSOR_DATA_FIELD_PROPERTY_INDEX;

//
// Sensor Common Properties
//
typedef enum
{
    SENSOR_PROPERTY_STATE = 0,
    SENSOR_PROPERTY_MIN_DATA_INTERVAL,
    SENSOR_PROPERTY_DEFAULT_DATA_INTERVAL,
    SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE,
    SENSOR_PROPERTY_TYPE,
    SENSOR_PROPERTY_FIFORESERVEDSIZE_SAMPLES,
    SENSOR_PROPERTY_FIFO_MAXSIZE_SAMPLES,
    SENSOR_PROPERTY_WAKE_CAPABLE,
    SENSOR_PROPERTY_COUNT
} SENSOR_PROPERTIES_INDEX;

//---------------------------------------
// Declare and map devices below
//---------------------------------------
enum Device
{
    Device_LinearAccelerometer = 0,
    Device_Gyr,
    // Keep this last
    Device_Count
};

static const ULONG SensorInstanceCount = Device_Count;
static SENSOROBJECT SensorInstancesBuffer[SensorInstanceCount];    // Global buffer to avoid allocate and free

typedef struct _REGISTER_SETTING
{
    BYTE Register;
    BYTE Value;
} REGISTER_SETTING, * PREGISTER_SETTING;

// Array of settings that describe the initial device configuration.
const REGISTER_SETTING g_ConfigurationSettings[] =
{
    // Power-dowm mode, 4G , 2000 DPS
    { LSM6DSOX_CTRL1_XL , LSM6DSOX_CTRL1_XL_ODR_POWER_DOWN | LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G  },
    { LSM6DSOX_CTRL2_G , LSM6DSOX_CTRL2_G_ODR_POWER_DOWN | LSM6DSOX_CTRL2_G_FS_MODE_2000_DPS },

    // Interrupt Active level low, Push/Pull, Auto address increase disabled
    { LSM6DSOX_CTRL3_C , LSM6DSOX_CTRL3_C_BDU_UNTIL_READ | LSM6DSOX_CTRL3_C_H_LACTIVE_ACTIVE_HIGH | LSM6DSOX_CTRL3_C_PP_OD_PUSH_PULL | LSM6DSOX_CTRL3_C_IF_INC_ENABLE },

    // Interrupt Active level low, Push/Pull, Auto address increase disabled
    { LSM6DSOX_CTRL4_C , LSM6DSOX_CTRL4_C_DRDY_MASK_ENABLE },

    // Rounding disabled (Circular burst-mode)
    { LSM6DSOX_CTRL5_C , LSM6DSOX_CTRL5_C_ROUNDING_DISABLED },

    // Disable accelerometer High Performance mode
    { LSM6DSOX_CTRL6_C , LSM6DSOX_CTRL6_C_HM_MODE_DISABLED },

    // Disable gyroscope High Performance mode
    { LSM6DSOX_CTRL7_G , LSM6DSOX_CTRL7_G_HM_MODE_DISABLED },

    // I3C comunication disabled
    { LSM6DSOX_CTRL9_XL , LSM6DSOX_CTRL9_XL_I3C_DISABLE },

    

    // Data ready interrupt enabled on INT1 pin
    { LSM6DSOX_INT1_CTRL , LSM6DSOX_INT1_CTRL_DRDY_XL_ENABLE | LSM6DSOX_INT1_CTRL_DRDY_G_ENABLE },

    // FIFO
    { LSM6DSOX_COUNTER_BDR_REG1 , 0 },
    { LSM6DSOX_COUNTER_BDR_REG2 , 0 },
    { LSM6DSOX_FIFO_CTRL1 , 0 },
    { LSM6DSOX_FIFO_CTRL2 , 0 },
    { LSM6DSOX_FIFO_CTRL3 , LSM6DSOX_FIFO_CTRL3_BDR_XL_NOT_BATCHED | LSM6DSOX_FIFO_CTRL3_BDR_GY_NOT_BATCHED },
    { LSM6DSOX_FIFO_CTRL4 , LSM6DSOX_FIFO_CTRL4_FIFO_MODE_DISABLE },

   
};

//
// Base ---------------------------------------------------------------------
//
typedef class _IMUDevice
{
public:
    //
    // WDF
    //
    static WDFDEVICE            m_Device;
    static WDFIOTARGET          m_I2CIoTarget;
    static WDFWAITLOCK          m_I2CWaitLock;
    static WDFINTERRUPT         m_Interrupt;

    //
    // Sensor Operation
    //
    static BOOLEAN              m_PoweredOn;  
    static ULONG                m_MinimumInterval;
    static BOOLEAN              m_WakeEnabled;

    BOOLEAN                     m_Started{};
    BOOLEAN                     m_FirstSample{};
    ULONG                       m_StartTime{};
    ULONGLONG                   m_SampleCount{};
    DATA_RATE                   m_DataRate;
    BOOLEAN                     m_DataReady{};
    
    //
    //FIFO
    //
    BOOLEAN                     m_fifo_enabled;
    ULONG                       m_batch_latency;
    ULONG                       m_fifo_size;
    UINT32                      m_max_fifo_samples;


    SENSOROBJECT                m_SensorInstance{};

    //
    // Sensor Specific Properties
    //
    PSENSOR_PROPERTY_LIST       m_pSupportedDataFields{};
    PSENSOR_COLLECTION_LIST     m_pEnumerationProperties{};
    PSENSOR_COLLECTION_LIST     m_pSensorProperties{};
    PSENSOR_COLLECTION_LIST     m_pSensorData{};
    PSENSOR_COLLECTION_LIST     m_pDataFieldProperties{};
    PSENSOR_COLLECTION_LIST     m_pThresholds{};

public:

    //
// CLX Callbacks
//
    static EVT_SENSOR_DRIVER_START_SENSOR               OnStart;
    static EVT_SENSOR_DRIVER_STOP_SENSOR                OnStop;
    static EVT_SENSOR_DRIVER_GET_SUPPORTED_DATA_FIELDS  OnGetSupportedDataFields;
    static EVT_SENSOR_DRIVER_GET_PROPERTIES             OnGetProperties;
    static EVT_SENSOR_DRIVER_GET_DATA_FIELD_PROPERTIES  OnGetDataFieldProperties;
    static EVT_SENSOR_DRIVER_GET_DATA_INTERVAL          OnGetDataInterval;
    static EVT_SENSOR_DRIVER_SET_DATA_INTERVAL          OnSetDataInterval;
    static EVT_SENSOR_DRIVER_GET_DATA_THRESHOLDS        OnGetDataThresholds;
    static EVT_SENSOR_DRIVER_SET_DATA_THRESHOLDS        OnSetDataThresholds;
    static EVT_SENSOR_DRIVER_DEVICE_IO_CONTROL          OnIoControl;
    static EVT_SENSOR_DRIVER_START_SENSOR_HISTORY       OnStartHistory;
    static EVT_SENSOR_DRIVER_STOP_SENSOR_HISTORY        OnStopHistory;
    static EVT_SENSOR_DRIVER_CLEAR_SENSOR_HISTORY       OnClearHistory;
    static EVT_SENSOR_DRIVER_START_HISTORY_RETRIEVAL    OnStartHistoryRetrieval;
    static EVT_SENSOR_DRIVER_CANCEL_HISTORY_RETRIEVAL   OnCancelHistoryRetrieval;
    static EVT_SENSOR_DRIVER_ENABLE_WAKE                OnEnableWake;
    static EVT_SENSOR_DRIVER_DISABLE_WAKE               OnDisableWake;
    static EVT_SENSOR_DRIVER_SET_BATCH_LATENCY          OnSetBatchLatency;

    // Interrupt callbacks
    static EVT_WDF_INTERRUPT_ISR                        OnInterruptIsr;
    static EVT_WDF_INTERRUPT_WORKITEM                   OnInterruptWorkItem;

public:

    static NTSTATUS             ConfigureIoTarget(_In_ WDFCMRESLIST ResourceList, _In_ WDFCMRESLIST ResourceListTranslated);
    //
    // Sensor specific functions
    //
    virtual DATA_RATE           GetDataRateFromReportInterval(_In_ ULONG ReportInterval)       = NULL;
    virtual NTSTATUS            Initialize(_In_ WDFDEVICE Device, _In_ SENSOROBJECT SensorObj) = NULL;
    virtual NTSTATUS            GetData()                                                      = NULL;

    virtual NTSTATUS            UpdateCachedThreshold()                                        = NULL;
    virtual NTSTATUS            UpdateDataInterval(_In_ ULONG DataRateMs)                      = NULL;
    virtual NTSTATUS            UpdateBatchLatency(_In_ ULONG BatchLatencyMs)                  = NULL;
    virtual NTSTATUS            StartSensor()                                                  = NULL;
    virtual NTSTATUS            StopSensor()                                                   = NULL;
    static  NTSTATUS            EnableWake() { return STATUS_NOT_SUPPORTED; }
    static  NTSTATUS            DisableWake() { return STATUS_NOT_SUPPORTED; }

    //
    // History functions - none of the sensors in this driver actually support history yet, this is for testing purpose now.
    //
    virtual NTSTATUS            StartHistory() { return STATUS_NOT_SUPPORTED; }
    virtual NTSTATUS            StopHistory() { return STATUS_NOT_SUPPORTED; }
    virtual NTSTATUS            ClearHistory() { return STATUS_NOT_SUPPORTED; }
    virtual NTSTATUS            StartHistoryRetrieval(_Inout_ PSENSOR_COLLECTION_LIST /*pHistoryBuffer*/, _In_ ULONG /*HistorySizeInBytes*/) { return STATUS_NOT_SUPPORTED; }
    virtual NTSTATUS            CancelHistoryRetrieval(_Out_ PULONG /*pBytesWritten*/) { return STATUS_NOT_SUPPORTED; }

    // Helper function for OnD0Entry which sets up device to default configuration
    static NTSTATUS             PowerOn();
    static NTSTATUS             PowerOff();

} IMUDevice, *PIMUDevice;

// Set up accessor function to retrieve device context
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(IMUDevice, GetContextFromSensorInstance);

//
// Linear Accelerometer --------------------------------------------------------------
//
typedef class _LinearAccelerometerDevice : public _IMUDevice
{
private:

    typedef struct _LinearAccelerometerSample
    {
        VEC3D   Axis;
        BOOL    Shake;
    } LinearAccelerometerSample, *PLinearAccelerometerSample;

    LinearAccelerometerSample                       m_CachedThresholds{};
    LinearAccelerometerSample                       m_CachedData{};
    LinearAccelerometerSample                       m_LastSample{};

public:

    DATA_RATE                   GetDataRateFromReportInterval(_In_ ULONG ReportInterval);

    NTSTATUS                    Initialize(_In_ WDFDEVICE Device, _In_ SENSOROBJECT SensorObj);
    NTSTATUS                    GetData();
    NTSTATUS                    UpdateCachedThreshold();
    NTSTATUS                    UpdateDataInterval(_In_ ULONG DataRateMs);
    NTSTATUS                    UpdateBatchLatency(_In_ ULONG BatchLatencyMs);
    NTSTATUS                    StartSensor();
    NTSTATUS                    StopSensor();

} LinearAccelerometerDevice, *PLinearAccelerometerDevice;


//
// Gyroscope ------------------------------------------------------------------
//
typedef class _GyrDevice : public _IMUDevice
{
private:

    VEC3D                       m_CachedThresholds;
    VEC3D                       m_CachedData;
    VEC3D                       m_LastSample;

public:

    DATA_RATE                   GetDataRateFromReportInterval(_In_ ULONG ReportInterval);

    NTSTATUS                    Initialize(_In_ WDFDEVICE Device, _In_ SENSOROBJECT SensorObj);
    NTSTATUS                    GetData();
    NTSTATUS                    UpdateCachedThreshold();
    NTSTATUS                    UpdateDataInterval(_In_ ULONG DataRateMs);
    NTSTATUS                    UpdateBatchLatency(_In_ ULONG BatchLatencyMs);
    NTSTATUS                    StartSensor();
    NTSTATUS                    StopSensor();

} GyrDevice, *PGyrDevice;