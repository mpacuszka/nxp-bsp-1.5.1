// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//    LPSPIdriver.h
//
// Abstract:
//
//    This module contains the IMX LPSPI controller driver entry functions.
//    This controller driver uses the SPB WDF class extension (SpbCx).
//
// Environment:
//
//    kernel-mode only
//

#ifndef _LPSPI_DRIVER_H_
#define _LPSPI_DRIVER_H_

WDF_EXTERN_C_START

//
// LPSPI_DRIVER_EXTENSION.
//  Contains all The IMXLPSPI driver configuration parameters.
//  It is associated with the WDFDRIVER object.
//
typedef struct _LPSPI_DRIVER_EXTENSION
{
    //
    // Driver log handle
    //
    RECORDER_LOG DriverLogHandle;

    //
    // Driver configuration parameters if they reside in registry
    //  The configuration parameters reside at:
    //  HKLM\System\CurrentControlSet\Services\imxecspi\Parameters
    //

    //
    // Reference clock.
    //
    ULONG ReferenceClockHz;

    //
    // Max connection speed.
    // Optional, if not given ReferenceClockHz is used.
    //
    ULONG MaxConnectionSpeedHz;

    //
    // Driver flags
    //
    ULONG Flags;

} LPSPI_DRIVER_EXTENSION;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(LPSPI_DRIVER_EXTENSION, LPSPIDriverGetExtension);


//
// LPSPI driver WDF Driver event handlers
//
DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_UNLOAD LPSPIEvtDriverUnload;
EVT_WDF_DRIVER_DEVICE_ADD LPSPIEvtDeviceAdd;


LPSPI_DRIVER_EXTENSION*
LPSPIDriverGetDriverExtension ();

//
// Driver trace log handle
//
#define DRIVER_LOG_HANDLE LPSPIDriverGetDriverExtension()->DriverLogHandle

WDF_EXTERN_C_END

#endif // !_LPSPI_DRIVER_H_
