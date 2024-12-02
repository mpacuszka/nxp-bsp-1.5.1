// Copyright (C) Microsoft Corporation, All Rights Reserved
// Copyright 2022-2023 NXP
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

#include "fxos8700.h"
#include "SensorsTrace.h"


/*++
Routine Description:
    Critical section lock/unlock to protect shared context
Return Value:
    None
--*/
#define Lock(lock)      { WdfWaitLockAcquire(lock, NULL); }
#define Unlock(lock)    { WdfWaitLockRelease(lock); }

#define SENSORV2_POOL_TAG_ECOMPASS           '2bmC'

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
    SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE,
    SENSOR_PROPERTY_TYPE,
    SENSOR_PROPERTY_FIFORESERVEDSIZE_SAMPLES,
    SENSOR_PROPERTY_FIFO_MAXSIZE_SAMPLES,
    SENSOR_PROPERTY_WAKE_CAPABLE,
    SENSOR_PROPERTY_COUNT
} SENSOR_PROPERTIES_INDEX;

enum SensorMode : ULONG
{
    STANDBY_MODE             = 0,
    ACCELEROMETER_ONLY_MODE,
    MAGNETOMETER_ONLY_MODE,
    HYBRID_MODE
};

//---------------------------------------
// Declare and map devices below
//---------------------------------------
enum Device
{
    Device_LinearAccelerometer = 0,
    Device_Mag,
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
    // Standby mode (Active), 14-bit resolution (f_read), Normal mode (Inoise), 1.5625 Hz (ODR), 50 Hz(aslp_rate).
    { FXOS8700_CTRL_REG1 , FXOS8700_CTRL_REG1_ACTIVE_STANDBY_MODE },
    // +-4g, No High-pass filter
    { FXOS8700_XYZ_DATA_CFG , FXOS8700_XYZ_DATA_CFG_FS_4G_0P488 },
    // No FIFO
    { FXOS8700_F_SETUP , FXOS8700_F_SETUP_F_MODE_FIFO_DISABLE },
    // Interrupt active low
    { FXOS8700_CTRL_REG3 , FXOS8700_CTRL_REG3_IPOL_ACTIVE_LOW | FXOS8700_CTRL_REG3_PP_OD_OPEN_DRAIN },
    // Data ready and FIFO interrupt routing to INT1 pin
    { FXOS8700_CTRL_REG5 , FXOS8700_CTRL_REG5_INT_CFG_DRDY_INT1 | FXOS8700_CTRL_REG5_INT_CFG_FIFO_INT1 },
    // Data ready and FIFO interrupts enable
    { FXOS8700_CTRL_REG4 , FXOS8700_CTRL_REG4_INT_EN_DRDY_EN | FXOS8700_CTRL_REG4_INT_EN_FIFO_EN },
    // Hybrid mode
    {FXOS8700_M_CTRL_REG1 , FXOS8700_M_CTRL_REG1_M_ACAL_EN | FXOS8700_M_CTRL_REG1_M_HMS_HYBRID_MODE },
};

//
// Base ---------------------------------------------------------------------
//
typedef class _eCompassDevice
{
public:
    //
    // WDF
    //
    static WDFDEVICE            m_Device;
    static WDFTIMER             m_Timer;
    static WDFIOTARGET          m_I2CIoTarget;
    static WDFWAITLOCK          m_I2CWaitLock;
    static WDFINTERRUPT         m_Interrupt;

    //
    // Sensor Operation
    //
    static BOOLEAN              m_PoweredOn;  
    static DATA_RATE            m_DataRate;
    static ULONG                m_MinimumInterval;
    static BOOLEAN              m_WakeEnabled;
    static SensorMode           m_SensorMode;

    BOOLEAN                     m_Started{};
    BOOLEAN                     m_FirstSample{};
    ULONG                       m_StartTime{};
    ULONGLONG                   m_SampleCount{};
    
    SENSOROBJECT                m_SensorInstance{};

    //
    //FIFO
    //
    static BOOLEAN              m_fifo_enabled;
    static ULONG                m_batch_latency;
    static ULONG                m_fifo_size;
    static UINT32               m_accelerometer_max_fifo_samples;
    static UINT32               m_magnetometer_max_fifo_samples;

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

    // WDF callbacks
    static EVT_WDF_TIMER                                OnTimerExpire;

    // Interrupt callbacks
    static EVT_WDF_INTERRUPT_ISR                        OnInterruptIsr;
    static EVT_WDF_INTERRUPT_WORKITEM                   OnInterruptWorkItem;

public:

    static DATA_RATE            GetDataRateFromReportInterval(_In_ ULONG ReportInterval, _In_ ULONG SensorMode);
    static NTSTATUS             ConfigureIoTarget(_In_ WDFCMRESLIST ResourceList, _In_ WDFCMRESLIST ResourceListTranslated);
    //
    // Sensor specific functions
    //
    virtual NTSTATUS            Initialize(_In_ WDFDEVICE Device, _In_ SENSOROBJECT SensorObj) = NULL;
    virtual NTSTATUS            GetData()                                                      = NULL;

    virtual NTSTATUS            UpdateCachedThreshold()                                        = NULL;
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

} eCompassDevice, *PeCompassDevice;

// Set up accessor function to retrieve device context
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(eCompassDevice, GetContextFromSensorInstance);

//
// Linear Accelerometer --------------------------------------------------------------
//
typedef class _LinearAccelerometerDevice : public _eCompassDevice
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

    NTSTATUS                    Initialize(_In_ WDFDEVICE Device, _In_ SENSOROBJECT SensorObj);
    NTSTATUS                    GetData();
    NTSTATUS                    UpdateCachedThreshold();
    NTSTATUS                    StartSensor();
    NTSTATUS                    StopSensor();

} LinearAccelerometerDevice, *PLinearAccelerometerDevice;

//
// Magnetometer ---------------------------------------------------------------
//
typedef struct _MagData
{
    VEC3D Axis;
    ULONG Accuracy;
} MagData, * PMagData;

typedef class _MagDevice : public _eCompassDevice
{
private:

    VEC3D                       m_CachedThresholds{};
    MagData                     m_CachedData{};
    MagData                     m_LastSample{};

public:

    NTSTATUS                    Initialize(_In_ WDFDEVICE Device, _In_ SENSOROBJECT SensorObj);
    NTSTATUS                    GetData();
    NTSTATUS                    UpdateCachedThreshold();
    NTSTATUS                    StartSensor();
    NTSTATUS                    StopSensor();

} MagDevice, * PMagDevice;