// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//    LPSPIdevice.h
//
// Abstract:
//
//    This module contains all the enums, types, and functions related to
//    an IMX LPSPI controller device.
//    This controller driver uses the SPB WDF class extension (SpbCx).
//
// Environment:
//
//    kernel-mode only
//

#ifndef _LPSPI_DEVICE_H_
#define _LPSPI_DEVICE_H_

WDF_EXTERN_C_START

#define PARAMATER_NAME_LEN 80

//
// LPSPI_CS_GPIO_PIN.
//  A CS GPIO pin descriptor.
//
typedef struct _LPSPI_CS_GPIO_PIN
{
    //
    // The CS GPIO pin connection ID
    //
    LARGE_INTEGER GpioConnectionId;

    //
    // The CS GPIO pin device handle
    //
    WDFIOTARGET WdfIoTargetGpio;

    //
    // The CS GPIO data, and memory objects
    //
    ULONG GpioData;
    WDFMEMORY WdfMemoryGpio;

    //
    // The CS GPIO requests
    //
    WDFREQUEST WdfRequestGpioAssert;
    WDFREQUEST WdfRequestGpioNegate;

    //
    // Used for synchronous settings of GPIO pin.
    //
    KEVENT SyncCallEvent;

    //
    // Open count
    //
    ULONG OpenCount;

} LPSPI_CS_GPIO_PIN;

typedef struct _LPSPI_CONFIG_DATA {
    ULONG ReferenceClockHz;
    ULONG MaxConnectionSpeedHz;
    ULONG SampleOnDelayedSckEdge;
} LPSPI_CONFIG_DATA;

typedef struct _LPSPI_HW_VERSION {
    UINT8 Minor;
    UINT8 Major;
} LPSPI_HW_VERSION;

//
// LPSPI_DEVICE_EXTENSION.
//  Contains all The LPSPI device runtime parameters.
//  It is associated with the WDFDEVICE object.
//
typedef struct _LPSPI_DEVICE_EXTENSION
{
    //
    // The WDFDEVICE associated with this instance of the
    // controller driver.
    //
    WDFDEVICE WdfDevice;

    //
    // Device lock
    //
    KSPIN_LOCK DeviceLock;

    //
    // The LOG handle for this device
    //
    RECORDER_LOG IfrLogHandle;

    //
    // LPSPI mapped resources
    //

    //
    // LPSPI register file
    //
    volatile LPSPI_REGISTERS* LPSPIRegsPtr;

    //
    // LPSPI interrupt object
    //
    WDFINTERRUPT WdfSpiInterrupt;

    //
    //  Runtime...
    //

    //
    // If controller is locked
    //
    BOOLEAN IsControllerLocked;

    //
    // Lock/unlock request
    //
    SPBREQUEST LockUnlockRequest;

    //
    // The current request in progress
    //
    LPSPI_SPB_REQUEST CurrentRequest;

    //
    // The CS GPIO pin descriptors
    //
    LPSPI_CS_GPIO_PIN CsGpioPins[LPSPI_CHANNEL::COUNT];

    //
    // Device configuration parameters from ACPI
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
    // Sample On Delayed Sck Edge
    //
    BOOLEAN SampleOnDelayedSckEdge;

    //
    // LPSPI configuration
    // 

    //
    // LPSPI HW version
    //
    LPSPI_HW_VERSION ControllerHwVersion;

    //
    // Tx and Rx Fifo size
    //
    size_t TxFifoSize;
    size_t RxFifoSize;

    //
    // Tx and Rx max Fifo watermark
    //
    size_t TxFifoWatermarkMax;
    size_t RxFifoWatermarkMax;

    //
    // PRESCALE bits higher limitation
    //
    ULONG PrescaleMax;

} LPSPI_DEVICE_EXTENSION;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(LPSPI_DEVICE_EXTENSION, LPSPIDeviceGetExtension);


//
// LPSPIdevice WDF device event handlers
//
EVT_WDF_DEVICE_PREPARE_HARDWARE LPSPIEvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE LPSPIEvtDeviceReleaseHardware;
EVT_WDF_INTERRUPT_ISR LPSPIEvtInterruptIsr;
EVT_WDF_INTERRUPT_DPC LPSPIEvtInterruptDpc;
EVT_WDF_REQUEST_CANCEL LPSPIEvtRequestCancel;

_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS LPSPIGetConfigValues(
    _In_ LPSPI_CONFIG_DATA* DriverSpiConfigPtr,
    _In_ WDFDEVICE Device);

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
LPSPIDeviceEnableRequestCancellation (
    _In_ LPSPI_SPB_REQUEST* RequestPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
LPSPIDeviceDisableRequestCancellation (
    _In_ LPSPI_SPB_REQUEST* RequestPtr
);

_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS
LPSPIDeviceOpenGpioTarget (
    _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr
    );

_IRQL_requires_max_(PASSIVE_LEVEL)
VOID
LPSPIDeviceCloseGpioTarget (
    _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr
    );

_When_(IsWait == 0, _IRQL_requires_max_(DISPATCH_LEVEL))
_When_(IsWait == 1, _IRQL_requires_max_(APC_LEVEL))
_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
LPSPIDeviceAssertCS (
    _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr,
    _In_ BOOLEAN IsWait
    );

_When_(IsWait == 0, _IRQL_requires_max_(DISPATCH_LEVEL))
_When_(IsWait == 1, _IRQL_requires_max_(APC_LEVEL))
NTSTATUS
LPSPIDeviceNegateCS (
    _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr,
    _In_ BOOLEAN IsWait
    );

//
// Routine Description:
//
//  LPSPIDeviceGetGpio get the CS GPIO descriptor
//  based on the target device selection.
//
// Arguments:
//
//  TrgCtxPtr - The target context
//
// Return Value:
//
//  The CS GPIO device descriptor address.
//
__forceinline
LPSPI_CS_GPIO_PIN*
LPSPIDeviceGetCsGpio (
    _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;

    return &devExtPtr->CsGpioPins[TrgCtxPtr->Settings.DeviceSelection];
}

//
// Routine Description:
//
//  LPSPIDeviceGetReferenceClock returns the reference clock in Hz.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
// Return Value:
//
//  Reference clock in Hz.
//
__forceinline
ULONG
LPSPIDeviceGetReferenceClock(
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr
)
{
    return DevExtPtr->ReferenceClockHz;
}

//
// Routine Description:
//
//  LPSPIDevicerGetMaxSpeed returns the maximum connection speed in Hz.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
// Return Value:
//
//  The maximum connection speed in Hz.
//
__forceinline
ULONG
LPSPIDeviceGetMaxSpeed(
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr
)
{
    ULONG maxSpeed = DevExtPtr->MaxConnectionSpeedHz;
    ULONG refClock = LPSPIDeviceGetReferenceClock(DevExtPtr);
    if ((maxSpeed == 0) || (maxSpeed > (refClock / 2))) {

        return refClock / 2;
    }
    return maxSpeed;
}

//
// Routine Description:
//
//  LPSPIDeviceIsSampleOnDelayedSck returns TRUE loopback is enabled.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
// Return Value:
//
//  TRUE if  loopback is enabled, otherwise FALSE.
//
__forceinline
BOOLEAN
LPSPIDeviceIsSampleOnDelayedSckEdge(
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr
)
{
    return DevExtPtr->SampleOnDelayedSckEdge;
}


//
// LPSPIdevice private methods
//
#ifdef _LPSPI_DEVICE_CPP_

    _IRQL_requires_max_(PASSIVE_LEVEL)
    static VOID
    LPSPIpDeviceLogInit (
        _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
        _In_ ULONG DeviceLogId
        );

    _IRQL_requires_max_(PASSIVE_LEVEL)
    static VOID
    LPSPIpDeviceLogDeinit (
        _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr
        );

    _IRQL_requires_max_(DISPATCH_LEVEL)
    static NTSTATUS
    LPSPIpDeviceSetGpioPin (
        _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr,
        _In_ ULONG Value
        );

    static EVT_WDF_REQUEST_COMPLETION_ROUTINE LPSPIpGpioAssertCompletionRoutine;
    static EVT_WDF_REQUEST_COMPLETION_ROUTINE LPSPIpGpioNegateCompletionRoutine;

#endif // _LPSPI_DEVICE_CPP_

WDF_EXTERN_C_END

#endif // !_LPSPI_DEVICE_H_