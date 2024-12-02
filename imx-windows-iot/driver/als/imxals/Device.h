//Copyright (C) Microsoft Corporation, All Rights Reserved
//Copyright 2022 NXP
//
//Abstract:
//
//    This module contains the type definitions for the client
//    driver's device callback class.
//
//Environment:
//
//    Windows User-Mode Driver Framework (UMDF)

#pragma once

#include <windows.h>
#include <wdf.h>
#include <reshub.h>
#include <strsafe.h>

#include <SensorsDef.h>
#include <SensorsCx.h>
#include <sensorsutils.h>
#include <SensorsDriversUtils.h>

#include "Isl29023.h"
#include "SensorsTrace.h"

#define SENSORV2_POOL_TAG_LIGHTSENSOR '2LmA'

// Sensor Common Properties
typedef enum
{
    SENSOR_PROPERTY_STATE = 0,
    SENSOR_PROPERTY_MIN_DATA_INTERVAL,
    SENSOR_PROPERTY_MAX_DATA_FIELD_SIZE,
    SENSOR_PROPERTY_TYPE,
    SENSOR_PROPERTIES_COUNT
} SENSOR_PROPERTIES_INDEX;

// Sensor Enumeration Properties
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

// Data
typedef enum
{
    SENSOR_DATA_LIGHTLEVEL_LUX = 0,
    SENSOR_DATA_TIMESTAMP,
    SENSOR_DATA_COUNT
} SENSOR_DATA_INDEX;

// Data-field Properties
typedef enum
{
    SENSOR_DATA_FIELD_PROPERTY_ALS_RESOLUTION = 0,
    SENSOR_DATA_FIELD_PROPERTY_ALS_RANGE_MIN,
    SENSOR_DATA_FIELD_PROPERTY_ALS_RANGE_MAX,
    SENSOR_DATA_FIELD_PROPERTIES_COUNT
} SENSOR_DATA_FIELD_PROPERTIES_INDEX;

typedef struct _REGISTER_SETTING
{
    BYTE Register;
    BYTE Value;
} REGISTER_SETTING, * PREGISTER_SETTING;

// Return the rate that is just less than the desired report interval
inline DATA_RATE _GetDataRateFromReportInterval(_In_ ULONG ReportInterval)
{
    DATA_RATE dataRate = SENSOR_SUPPORTED_DATA_RATES[0];

    for (ULONG i = 0; i < SENSOR_SUPPORTED_DATA_RATES_COUNT; i++)
    {
        dataRate = SENSOR_SUPPORTED_DATA_RATES[i];
        if (dataRate.DataRateInterval <= ReportInterval)
        {
            break;
        }
    }

    return dataRate;
}

// Array of settings that describe the initial device configuration.
const REGISTER_SETTING g_ConfigurationSettings[] =
{
    // Power-Down mode
    { ISL29023_COMMAND_I , ISL29023_COMMAND_I_MODE_POWER_DOWN },
    // RANGE 64000 LUX, Resolution 16bit = 65536
    { ISL29023_COMMAND_II , ISL29023_COMMAND_II_RESOLUTION_16BIT | ISL29023_COMMAND_II_RANGE_64000LUX },
    // Interrupt Lower threshold to min default value
    { ISL29023_INT_LT_LSB , 0x00 },
    { ISL29023_INT_LT_MSB , 0x00 },
    // Interrupt Higher threshold to max default value
    { ISL29023_INT_HT_LSB , 0xFF },
    { ISL29023_INT_HT_MSB , 0xFF },
    // Test value set to 0
    { ISL29023_TEST , 0x00 },
};

typedef struct _AlsSample
{
    float Lux;
} AlsSample, * PAlsSample;

typedef class _AlsDevice
{
private:
    // WDF
    WDFDEVICE                   m_Device;
    WDFIOTARGET                 m_I2CIoTarget;
    WDFWAITLOCK                 m_I2CWaitLock;
    WDFINTERRUPT                m_Interrupt;
    WDFTIMER                    m_Timer;

    // Sensor Operation
    bool                        m_PoweredOn;
    bool                        m_Started;
    DATA_RATE                   m_DataRate;

    bool                        m_FirstSample;
    ULONG                       m_StartTime;
    ULONGLONG                   m_SampleCount;
    AlsSample                   m_CachedThresholds;
    AlsSample                   m_LastSample;

    SENSOROBJECT                m_SensorInstance;

    // Sensor Specific Properties
    PSENSOR_PROPERTY_LIST       m_pSupportedDataFields;
    PSENSOR_COLLECTION_LIST     m_pEnumerationProperties;
    PSENSOR_COLLECTION_LIST     m_pSensorProperties;
    PSENSOR_COLLECTION_LIST     m_pSensorData;
    PSENSOR_COLLECTION_LIST     m_pDataFieldProperties;
    PSENSOR_COLLECTION_LIST     m_pThresholds;

public:
    // WDF callbacks
    static EVT_WDF_DRIVER_DEVICE_ADD                OnDeviceAdd;
    static EVT_WDF_DEVICE_PREPARE_HARDWARE          OnPrepareHardware;
    static EVT_WDF_DEVICE_RELEASE_HARDWARE          OnReleaseHardware;
    static EVT_WDF_DEVICE_D0_ENTRY                  OnD0Entry;
    static EVT_WDF_DEVICE_D0_EXIT                   OnD0Exit;
    static EVT_WDF_TIMER                            OnTimerExpire;

    // CLX callbacks
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

    // Interrupt callbacks
    static EVT_WDF_INTERRUPT_ISR       OnInterruptIsr;
    static EVT_WDF_INTERRUPT_WORKITEM  OnInterruptWorkItem;

private:
    NTSTATUS                    GetData();
    NTSTATUS                    RegisterCheck(_In_ SENSOROBJECT SensorInstance, _In_ REGISTER_SETTING RegisterSetting);

    // Helper function for OnPrepareHardware to initialize sensor to default properties
    NTSTATUS                    Initialize(_In_ WDFDEVICE Device, _In_ SENSOROBJECT SensorInstance);
    VOID                        DeInit();

    // Helper function for OnPrepareHardware to get resources from ACPI and configure the I/O target
    NTSTATUS                    ConfigureIoTarget(_In_ WDFCMRESLIST ResourceList,
                                                  _In_ WDFCMRESLIST ResourceListTranslated);

    // Helper function for OnD0Entry which sets up device to default configuration
    NTSTATUS                    PowerOn();
    NTSTATUS                    PowerOff();

} AlsDevice, *PAlsDevice;

// Set up accessor function to retrieve device context
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(AlsDevice, GetAlsContextFromSensorInstance);