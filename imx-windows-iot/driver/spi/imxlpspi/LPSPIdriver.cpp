// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//    LPSPIdriver.cpp
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
#include "precomp.h"
#pragma hdrstop

#define _LPSPI_DRIVER_CPP_

// Logging header files
#include "LPSPItrace.h"
#include "LPSPIdriver.tmh"

// Common driver header files
#include "LPSPIcommon.h"

// Module specific header files
#include "LPSPIhw.h"
#include "LPSPIspb.h"
#include "LPSPIdriver.h"
#include "LPSPIdevice.h"


#ifdef ALLOC_PRAGMA
    #pragma alloc_text(INIT, DriverEntry)
    #pragma alloc_text(PAGE, LPSPIEvtDriverUnload)
    #pragma alloc_text(PAGE, LPSPIEvtDeviceAdd)

#endif


//
// Routine Description:
//
//  Installable driver initialization entry point.
//  This entry point is called directly by the I/O system.
//
// Arguments:
//
//  DriverObjectPtr - pointer to the driver object
//
//  RegistryPathPtr - pointer to a Unicode string representing the path,
//      to driver-specific key in the registry.
//
// Return Value:
//
//    STATUS_SUCCESS, or appropriate error code
//
_Use_decl_annotations_
NTSTATUS
DriverEntry (
    DRIVER_OBJECT* DriverObjectPtr,
    UNICODE_STRING* RegistryPathPtr
    )
{
    NTSTATUS status;

    PAGED_CODE();

    //
    // Initialize tracing
    //
    RECORDER_LOG defLogHandle = NULL;
    {
        WPP_INIT_TRACING(DriverObjectPtr, RegistryPathPtr);

        RECORDER_CONFIGURE_PARAMS recorderConfigureParams;
        RECORDER_CONFIGURE_PARAMS_INIT(&recorderConfigureParams);

        WppRecorderConfigure(&recorderConfigureParams);
        #if DBG
            WPP_RECORDER_LEVEL_FILTER(LPSPI_TRACING_VERBOSE) = TRUE;
        #endif // DBG
        
        defLogHandle = WppRecorderLogGetDefault();

    } // Tracing

    //
    // Create the LPSPI WDF driver object
    //
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, LPSPI_DRIVER_EXTENSION);

        WDF_DRIVER_CONFIG wdfDriverConfig;
        WDF_DRIVER_CONFIG_INIT(&wdfDriverConfig, LPSPIEvtDeviceAdd);
        wdfDriverConfig.EvtDriverUnload = LPSPIEvtDriverUnload;

        //
        // Specify a pool tag for allocations WDF makes on our behalf
        //
        wdfDriverConfig.DriverPoolTag = ULONG(LPSPI_ALLOC_TAG::LPSPI_ALLOC_TAG_WDF);

        status = WdfDriverCreate(
            DriverObjectPtr,
            RegistryPathPtr,
            &attributes,
            &wdfDriverConfig,
            WDF_NO_HANDLE
            );
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                defLogHandle,
                "WdfDriverCreate failed, status = %!STATUS!", 
                status
                );
            goto done;
        }
        LPSPIDriverGetDriverExtension()->DriverLogHandle = defLogHandle;

    } // WDF driver 

done:

    //
    // Cleanup...
    //
    if (!NT_SUCCESS(status)) {

        WPP_CLEANUP(DriverObjectPtr);
    }

    return status;
}


//
// Routine Description:
//
//  LPSPIEvtDriverUnload is called by the framework just before the driver 
//  is unloaded. We use it to call WPP_CLEANUP.
//
// Arguments:
//
//  WdfDriver - Handle to a framework driver object created in DriverEntry
//
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIEvtDriverUnload (
    WDFDRIVER WdfDriver
    )
{
    PAGED_CODE();

    WPP_CLEANUP(WdfDriverWdmGetDriverObject(WdfDriver));
}


//
// Routine Description:
//
//  LPSPIEvtDeviceAdd is called by the framework in response to AddDevice
//  call from the PnP manager.
//  The function creates and initializes a new LPSPI WDF device, and registers it
//  with the SpbCx framework.
//
// Arguments:
//
//  WdfDriver - The LPSPI WDF driver object.
//
//  DeviceInitPtr - Pointer to a framework-allocated WDFDEVICE_INIT structure.
//
// Return Value:
//
//   Device creation and SpbCx registration status.
//
_Use_decl_annotations_
NTSTATUS
LPSPIEvtDeviceAdd (
    WDFDRIVER /*WdfDriver*/,
    PWDFDEVICE_INIT DeviceInitPtr
    )
{
    PAGED_CODE();

    NTSTATUS status;
    LPSPI_CONFIG_DATA SpiConfigDataSt = { 0x00 };

    //
    // Configure DeviceInit structure
    //
    status = SpbDeviceInitConfig(DeviceInitPtr);
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            DRIVER_LOG_HANDLE,
            "SpbDeviceInitConfig() failed. "
            "DeviceInitPtr = %p, status = %!STATUS!",
            DeviceInitPtr,
            status
            );
        return status;
    }

    //
    // Setup PNP/Power callbacks.
    //
    {
        WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);

        pnpCallbacks.EvtDevicePrepareHardware = 
            LPSPIEvtDevicePrepareHardware;
        pnpCallbacks.EvtDeviceReleaseHardware =
            LPSPIEvtDeviceReleaseHardware;

        WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInitPtr, &pnpCallbacks);

    } // // Setup PNP/Power callbacks

    //
    // Create the device.
    //
    WDFDEVICE wdfDevice;
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(
            &attributes, 
            LPSPI_DEVICE_EXTENSION
            );

        status = WdfDeviceCreate(&DeviceInitPtr, &attributes, &wdfDevice);
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                DRIVER_LOG_HANDLE,
                "Failed to create WDFDEVICE. "
                "DeviceInitPtr = %p, status = %!STATUS!",
                DeviceInitPtr,
                status
                );
            return status;
        }

        //
        // For DMA...
        //
        WdfDeviceSetAlignmentRequirement(wdfDevice, FILE_LONG_ALIGNMENT);

    } // Create the device.

    // Obtain SPI device configuration parameters from ACPI
    status = LPSPIGetConfigValues(&SpiConfigDataSt, wdfDevice);
    if (!NT_SUCCESS(status)) {
            LPSPI_LOG_ERROR(
            DRIVER_LOG_HANDLE,
            "LPSPIGetConfigValues failed,"
            "WDFDEVICE = %p, status = %!STATUS!",
            wdfDevice,
            status
        );
        return status;
    }


    //
    // Register LPSPI with SpbCx as an SPB controller.
    //
    {
        SPB_CONTROLLER_CONFIG spbControllerConfig;
        SPB_CONTROLLER_CONFIG_INIT(&spbControllerConfig);

        spbControllerConfig.ControllerDispatchType = WdfIoQueueDispatchSequential;

        spbControllerConfig.EvtSpbTargetConnect = LPSPIEvtSpbTargetConnect;
        spbControllerConfig.EvtSpbTargetDisconnect = LPSPIEvtSpbTargetDisconnect;
        spbControllerConfig.EvtSpbControllerLock = LPSPIEvtSpbControllerLock;
        spbControllerConfig.EvtSpbControllerUnlock = LPSPIEvtSpbControllerUnlock;
        spbControllerConfig.EvtSpbIoRead = LPSPIEvtSpbIoRead;
        spbControllerConfig.EvtSpbIoWrite = LPSPIEvtSpbIoWrite;
        spbControllerConfig.EvtSpbIoSequence = LPSPIEvtSpbIoSequence;

        status = SpbDeviceInitialize(wdfDevice, &spbControllerConfig);
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                DRIVER_LOG_HANDLE,
                "SpbDeviceInitialize failed, "
                "wdfDevice = %p, status = %!STATUS!",
                wdfDevice,
                status
                );
            return status;
        }

        //
        // Register for other to support custom IOctl(s), required 
        // for handling FULL_DUPLEX requests.
        //
        SpbControllerSetIoOtherCallback(
            wdfDevice,
            LPSPIEvtSpbIoOther,
            LPSPIEvtIoInCallerContext
            );

    } // Register LPSPI with SpbCx as an SPB controller.

    //
    // Set target object attributes.
    // Create our SPB target context
    //
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(
            &attributes,
            LPSPI_TARGET_CONTEXT
            );

        SpbControllerSetTargetAttributes(wdfDevice, &attributes);

    } // Create our SPB target context

    //
    // Create an interrupt object
    //
    WDFINTERRUPT wdfInterrupt;
    {
        WDF_INTERRUPT_CONFIG interruptConfig;
        WDF_INTERRUPT_CONFIG_INIT(
            &interruptConfig,
            LPSPIEvtInterruptIsr,
            LPSPIEvtInterruptDpc
            );

        status = WdfInterruptCreate(
            wdfDevice,
            &interruptConfig,
            WDF_NO_OBJECT_ATTRIBUTES,
            &wdfInterrupt
            );
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                DRIVER_LOG_HANDLE,
                "Failed to create interrupt object. "
                "wdfDevice = %p, status = %!STATUS!",
                wdfDevice,
                status
                );
            return status;
        }

    } // Create an interrupt object

    //
    // Initialize the device extension
    //
    {
        LPSPI_DEVICE_EXTENSION* devExtPtr = LPSPIDeviceGetExtension(wdfDevice);
        devExtPtr->ReferenceClockHz = SpiConfigDataSt.ReferenceClockHz;
        devExtPtr->MaxConnectionSpeedHz = SpiConfigDataSt.MaxConnectionSpeedHz;
        devExtPtr->SampleOnDelayedSckEdge = SpiConfigDataSt.SampleOnDelayedSckEdge != 0;
        devExtPtr->WdfDevice = wdfDevice;
        devExtPtr->WdfSpiInterrupt = wdfInterrupt;
        KeInitializeSpinLock(&devExtPtr->DeviceLock);

    } // Initialize the device extension

    return STATUS_SUCCESS;
}

//
// Routine Description:
//
//  LPSPIDriverGetDriverExtension return the driver extension.
//
// Arguments:
//
// Return Value:
//
//  The driver extension address.
//
LPSPI_DRIVER_EXTENSION*
LPSPIDriverGetDriverExtension ()
{
    static LPSPI_DRIVER_EXTENSION* drvExtPtr = nullptr;

    if (drvExtPtr == nullptr) {

        drvExtPtr = LPSPIDriverGetExtension(WdfGetDriver());
    }
    LPSPI_ASSERT(
        WppRecorderLogGetDefault(),
        drvExtPtr != nullptr
        );

    return drvExtPtr;
}


// 
// LPSPIdriver private methods.
// ----------------------------
//

#undef _LPSPI_DRIVER_CPP_