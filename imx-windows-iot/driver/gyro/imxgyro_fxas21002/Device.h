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
#include "fxas21002.h"
#include "SensorsTrace.h"

#define SENSORV2_POOL_TAG_FXAS_GYROSCOPE 'fxas'

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

// Data-field Properties
typedef enum
{
    GYR_DATA_X = 0,
    GYR_DATA_Y,
    GYR_DATA_Z,
    GYR_DATA_TIMESTAMP,
    SENSOR_DATA_COUNT
} SENSOR_DATA_INDEX;

typedef enum
{
    SENSOR_DATA_FIELD_PROPERTY_RESOLUTION = 0,
    SENSOR_DATA_FIELD_PROPERTY_RANGE_MIN,
    SENSOR_DATA_FIELD_PROPERTY_RANGE_MAX,
    SENSOR_DATA_FIELD_PROPERTIES_COUNT
} SENSOR_DATA_FIELD_PROPERTIES_INDEX;

typedef struct _REGISTER_SETTING
{
    BYTE Register;
    BYTE Value;
} REGISTER_SETTING, *PREGISTER_SETTING;

// Return the rate that is just less than the desired report interval
inline DATA_RATE _GetDataRateFromReportInterval(_In_ ULONG ReportInterval)
{
    DATA_RATE dataRate = GYROSCOPE_SUPPORTED_DATA_RATES[0];

    for (ULONG i = 0; i < GYROSCOPE_SUPPORTED_DATA_RATES_COUNT; i++)
    {
        dataRate = GYROSCOPE_SUPPORTED_DATA_RATES[i];
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
    //Standby mode
    {FXAS21002_CTRL_REG1, 0},
    //disable FIFO
    {FXAS21002_F_SETUP, 0},
    // LPF cutoff frequency=256Hz, disable high-pass filter, range +-2000dps
    {FXAS21002_CTRL_REG1, 0},
    //Disable rate treshold interrupt generation
    {FXAS21002_RT_CFG, 0},
    //Enable data ready interrupt, active low interrupt logic polarity, INT1 and INT2 pin output driver configuration open drain, data ready interrupt routed to INT1 pin
    {FXAS21002_CTRL_REG2, FXAS21002_CTRL_REG2_INT_CFG_DRDY_MASK | FXAS21002_CTRL_REG2_INT_EN_DRDY_MASK | FXAS21002_CTRL_REG2_PP_OD_MASK},
    //Disable full-scale range expansion
    {FXAS21002_CTRL_REG3, 0}

};

typedef class _FXAS21002GyroDevice
{
private:
    // WDF
    WDFDEVICE                   m_Device;
    WDFIOTARGET                 m_I2CIoTarget;
    WDFWAITLOCK                 m_I2CWaitLock;
    WDFINTERRUPT                m_Interrupt;

    // Sensor Operation
    bool                        m_PoweredOn;
    bool                        m_Started;

    DATA_RATE                   m_DataRate;

    bool                        m_FirstSample;
    VEC3D                       m_CachedThresholds;
    VEC3D                       m_LastSample;

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

    // Helper function for OnPrepareHardware to initialize sensor to default properties
    NTSTATUS                    Initialize(_In_ WDFDEVICE Device, _In_ SENSOROBJECT SensorInstance);
    VOID                        DeInit();

    // Helper function for OnPrepareHardware to get resources from ACPI and configure the I/O target
    NTSTATUS                    ConfigureIoTarget(_In_ WDFCMRESLIST ResourceList,
                                                  _In_ WDFCMRESLIST ResourceListTranslated);

    // Helper function for OnD0Entry which sets up device to default configuration
    NTSTATUS                    PowerOn();
    NTSTATUS                    PowerOff();

} FXAS21002GyroDevice, *PFXAS21002GyroDevice;

// Set up accessor function to retrieve device context
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(FXAS21002GyroDevice, GetFXAS21002GyroContextFromSensorInstance);