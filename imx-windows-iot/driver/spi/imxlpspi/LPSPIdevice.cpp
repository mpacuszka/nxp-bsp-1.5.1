// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//    LPSPIdevice.cpp
//
// Abstract:
//
//    This module contains the IMX LPSPI controller's PnP device functions.
//    This controller driver uses the SPB WDF class extension (SpbCx).
//
// Environment:
//
//    kernel-mode only
//
#include "precomp.h"
#pragma hdrstop

#define _LPSPI_DEVICE_CPP_

// Logging header files
#include "LPSPItrace.h"
#include "LPSPIdevice.tmh"

// Common driver header files
#include "LPSPIcommon.h"

// Module specific header files
#include "LPSPIhw.h"
#include "LPSPIspb.h"
#include "LPSPIdriver.h"
#include "LPSPIdevice.h"
#include <acpiioct.h>
#include "acpiutil.hpp"


#ifdef ALLOC_PRAGMA
    #pragma alloc_text(PAGE, LPSPIEvtDevicePrepareHardware)
    #pragma alloc_text(PAGE, LPSPIEvtDeviceReleaseHardware)
#endif

//
// Routine Description:
//
//  LPSPIEvtDevicePrepareHardware is called by the framework when a LPSPI
//  device is coming online, after being it's resources have been negotiated and
//  translated.
//  The routine reads and map the device resources and initializes the device.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  ResourcesRaw - Hardware resource list.
//
//  ResourcesTranslated - Hardware translated resource list.
//
// Return Value:
//
//  Resources mapping status and controller initialization completion code.
//
_Use_decl_annotations_
NTSTATUS
LPSPIEvtDevicePrepareHardware (
    WDFDEVICE WdfDevice,
    WDFCMRESLIST /*ResourcesRaw*/,
    WDFCMRESLIST ResourcesTranslated
    )
{
    PAGED_CODE();

    LPSPI_DEVICE_EXTENSION* devExtPtr = LPSPIDeviceGetExtension(WdfDevice);
    ULONG numResourses = WdfCmResourceListGetCount(ResourcesTranslated);
    ULONG numIntResourcesFound = 0;
    ULONG numMemResourcesFound = 0;
    ULONG numConnectionResourcesFound = 0;
    const CM_PARTIAL_RESOURCE_DESCRIPTOR* memResourceDescPtr = nullptr;
    ULONG traceLogId = 0;

    for (ULONG resInx = 0; resInx < numResourses; ++resInx) {

        const CM_PARTIAL_RESOURCE_DESCRIPTOR* resDescPtr =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, resInx);

        switch (resDescPtr->Type) {
        case CmResourceTypeMemory:
            ++numMemResourcesFound;
            if (numMemResourcesFound > 1) {

                LPSPI_LOG_ERROR(
                    DRIVER_LOG_HANDLE,
                    "Unexpected additional memory resource!"
                    );
                return STATUS_DEVICE_CONFIGURATION_ERROR;
            }

            if (resDescPtr->u.Memory.Length ==
                LPSPI_REGISTERS_ADDRESS_SPACE_SIZE) {

                memResourceDescPtr = resDescPtr;
                //
                // Use the lower part of the register base address
                // as trace log ID.
                //
                traceLogId = resDescPtr->u.Memory.Start.LowPart;

            } else {

                LPSPI_LOG_ERROR(
                    DRIVER_LOG_HANDLE,
                    "Invalid LPSPI register file span (%lu), expected %lu!",
                    resDescPtr->u.Memory.Length,
                    LPSPI_REGISTERS_ADDRESS_SPACE_SIZE
                    );
                return STATUS_DEVICE_CONFIGURATION_ERROR;
            }
            break;

        case CmResourceTypeInterrupt:
            ++numIntResourcesFound;
            if (numIntResourcesFound > 1) {

                LPSPI_LOG_ERROR(
                    DRIVER_LOG_HANDLE,
                    "Unexpected additional interrupt resource %lu, expected 1!",
                    numIntResourcesFound
                    );
                return STATUS_DEVICE_CONFIGURATION_ERROR;
            }
            break;

        case CmResourceTypeConnection:
            if ((resDescPtr->u.Connection.Class ==
                 CM_RESOURCE_CONNECTION_CLASS_FUNCTION_CONFIG) &&
                (resDescPtr->u.Connection.Type ==
                 CM_RESOURCE_CONNECTION_TYPE_FUNCTION_CONFIG)) {
                //
                // ignore FunctionConfig resources
                //
                continue;
            }

            if (numConnectionResourcesFound >= LPSPI_CHANNEL::COUNT) {

                LPSPI_LOG_ERROR(
                    DRIVER_LOG_HANDLE,
                    "Unexpected additional connection resource %lu, "
                    "expected %lu!",
                    numConnectionResourcesFound,
                    LPSPI_CHANNEL::COUNT
                    );
                return STATUS_DEVICE_CONFIGURATION_ERROR;
            }

            if ((resDescPtr->u.Connection.Class !=
                 CM_RESOURCE_CONNECTION_CLASS_GPIO) ||
                (resDescPtr->u.Connection.Type !=
                 CM_RESOURCE_CONNECTION_TYPE_GPIO_IO)) {

                LPSPI_LOG_ERROR(
                    DRIVER_LOG_HANDLE,
                    "Unexpected connection class/type (%lu/%lu), "
                    "only GPIO class/type (%lu/%lu) is expected!",
                    resDescPtr->u.Connection.Class,
                    resDescPtr->u.Connection.Type,
                    CM_RESOURCE_CONNECTION_CLASS_GPIO,
                    CM_RESOURCE_CONNECTION_TYPE_GPIO_IO
                    );
                return STATUS_DEVICE_CONFIGURATION_ERROR;
            }

            //
            // New CS GPIO pin entry
            //
            {
                LPSPI_CS_GPIO_PIN* csGpioPinPtr = 
                    &devExtPtr->CsGpioPins[numConnectionResourcesFound];

                csGpioPinPtr->GpioConnectionId.LowPart =
                    resDescPtr->u.Connection.IdLowPart;
                csGpioPinPtr->GpioConnectionId.HighPart =
                    resDescPtr->u.Connection.IdHighPart;

                KeInitializeEvent(
                    &csGpioPinPtr->SyncCallEvent,
                    NotificationEvent,
                    FALSE
                    );

                ++numConnectionResourcesFound;

            } // New CS GPIO pin entry
            break;

        default:
            LPSPI_ASSERT(DRIVER_LOG_HANDLE, FALSE);
            break;

        } // switch

    } // for (resource list)

    //
    // Make sure we got everything we need...
    //
    if (numMemResourcesFound != 1) {

        LPSPI_LOG_ERROR(DRIVER_LOG_HANDLE, "Invalid or no memory resource!");
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }

    if (numIntResourcesFound != 1) {

        LPSPI_LOG_ERROR(DRIVER_LOG_HANDLE, "Invalid or not interrupt resource!");
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }

    //
    // Map device registers into virtual memory...
    // The registers address space is 4 PAGES, but registers
    // span just 0x44 bytes, so we map the minimum number of pages.
    //
    {
        PHYSICAL_ADDRESS ecspiRegistersPhysAddress = memResourceDescPtr->u.Memory.Start;
        SIZE_T registersMappedSize = ROUND_TO_PAGES(sizeof(LPSPI_REGISTERS));

        devExtPtr->LPSPIRegsPtr = static_cast<LPSPI_REGISTERS*>(MmMapIoSpaceEx(
            ecspiRegistersPhysAddress,
            registersMappedSize,
            PAGE_READWRITE | PAGE_NOCACHE
            ));
        if (devExtPtr->LPSPIRegsPtr == nullptr) {

            LPSPI_LOG_ERROR(
                DRIVER_LOG_HANDLE,
                "Failed to map LPSPI regs, span %Iu bytes!",
                registersMappedSize
                );
            return STATUS_INSUFFICIENT_RESOURCES;
        }
    }

    NTSTATUS status = LPSPIHwInitController(devExtPtr);
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            DRIVER_LOG_HANDLE,
            "LPSPIHwInitController failed. "
            "wdfDevice = %p, status = %!STATUS!",
            WdfDevice,
            status
            );
        return status;
    }

    LPSPIpDeviceLogInit(devExtPtr, traceLogId);

    LPSPI_LOG_INFORMATION(
        devExtPtr->IfrLogHandle,
        "LPSPI regs mapped at %p",
        PVOID(devExtPtr->LPSPIRegsPtr)
        );

    return STATUS_SUCCESS;
}


//
// Routine Description:
//
//  LPSPIEvtDeviceReleaseHardware is called by the framework when a LPSPI
//  device is offline and not accessible anymore.
//  The routine just releases device resources.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  ResourcesTranslated - Hardware translated resource list.
//
// Return Value:
//
//  STATUS_SUCCESS
//
_Use_decl_annotations_
NTSTATUS
LPSPIEvtDeviceReleaseHardware (
    WDFDEVICE WdfDevice,
    WDFCMRESLIST ResourcesTranslated
    )
{
    PAGED_CODE();

    LPSPI_DEVICE_EXTENSION* devExtPtr = LPSPIDeviceGetExtension(WdfDevice);

    UNREFERENCED_PARAMETER(ResourcesTranslated);

    if (devExtPtr->LPSPIRegsPtr != nullptr) {

        MmUnmapIoSpace(
            PVOID(devExtPtr->LPSPIRegsPtr),
            ROUND_TO_PAGES(sizeof(LPSPI_REGISTERS))
            );
    }
    devExtPtr->LPSPIRegsPtr = nullptr;

    for (ULONG spiCh = 0; spiCh < LPSPI_CHANNEL::COUNT; ++spiCh) {

        LPSPI_CS_GPIO_PIN* csGpioPinPtr = &devExtPtr->CsGpioPins[spiCh];

        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle,
            csGpioPinPtr->WdfIoTargetGpio == NULL
            );
        csGpioPinPtr->GpioConnectionId.QuadPart = 0;
    }

    //
    // Disconnecting from interrupt will automatically be done
    // by the framework....
    //

    LPSPIpDeviceLogDeinit(devExtPtr);

    return STATUS_SUCCESS;
}


//
// Routine Description:
//
//  LPSPIEvtIoInCallerContext is called by the framework to pre-process
//  requests before they are put in a WDF IO queue.
//  It is used for custom IO control requests and specifically for FULL_DUPLEX
//  transfers.
//
// Arguments:
//
//  WdfDevice - The SPI device object
//
//  WdfRequest - The request.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIEvtIoInCallerContext (
    WDFDEVICE WdfDevice,
    WDFREQUEST WdfRequest
    )
{
    const LPSPI_DEVICE_EXTENSION* devExtPtr = LPSPIDeviceGetExtension(WdfDevice);

    WDF_REQUEST_PARAMETERS wdfRequestParams;
    WDF_REQUEST_PARAMETERS_INIT(&wdfRequestParams);
    WdfRequestGetParameters(WdfRequest, &wdfRequestParams);

    //
    // Filter out unrecognized IO control requests.
    //
    switch (wdfRequestParams.Type) {
    case WdfRequestTypeDeviceControl:
    case WdfRequestTypeDeviceControlInternal:
        break;

    default:
        WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
        return;

    } // switch

    switch (wdfRequestParams.Parameters.DeviceIoControl.IoControlCode) {
    case IOCTL_SPB_FULL_DUPLEX:
        break;

    default:
        WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
        return;
    }

    NTSTATUS status = SpbRequestCaptureIoOtherTransferList(
        static_cast<SPBREQUEST>(WdfRequest)
        );
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "SpbRequestCaptureIoOtherTransferList failed. "
            "wdfDevice = %p, status = %!STATUS!",
            WdfDevice,
            status
            );
        WdfRequestComplete(WdfRequest, status);
        return;
    }

    status = WdfDeviceEnqueueRequest(WdfDevice, WdfRequest);
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "WdfDeviceEnqueueRequest failed. "
            "wdfDevice = %p, status = %!STATUS!",
            WdfDevice,
            status
            );
        WdfRequestComplete(WdfRequest, status);
        return;
    }
}


//
// Routine Description:
//
//  LPSPIEvtRequestCancel is called by the framework to cancel
//  A submitted request.
//  The routine aborts all associated transfers and complete the request.
//
// Arguments:
//
//  WdfRequest - The canceled request.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIEvtRequestCancel (
    WDFREQUEST WdfRequest
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = LPSPIDeviceGetExtension(
        WdfFileObjectGetDevice(WdfRequestGetFileObject(WdfRequest))
        );

    KLOCK_QUEUE_HANDLE lockHandle;
    KeAcquireInStackQueuedSpinLock(&devExtPtr->DeviceLock, &lockHandle);

    LPSPI_SPB_REQUEST* requestPtr = &devExtPtr->CurrentRequest;
    SPBREQUEST spbRequest = requestPtr->SpbRequest;
    BOOLEAN isCompleteRequest = FALSE;

    LPSPI_LOG_WARNING(
        devExtPtr->IfrLogHandle,
        "LPSPIEvtRequestCancel called for request = 0x%p",
        spbRequest
        );

    LPSPISpbAbortAllTransfers(requestPtr);

    if (spbRequest != NULL) {

        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle,
            spbRequest == static_cast<SPBREQUEST>(WdfRequest)
            );

        requestPtr->State = LPSPI_REQUEST_STATE::CANCELED;

        if (devExtPtr->IsControllerLocked) {
            //
            // Device is locked, no need to negate CS, 
            // complete the request locally.
            //
            requestPtr->SpbRequest = NULL;
            isCompleteRequest = TRUE;

        } else {

            NTSTATUS status = LPSPIDeviceNegateCS(requestPtr->SpbTargetPtr, FALSE);
            if (status != STATUS_PENDING) {
                //
                // Negate CS failed, or GPIO for CS is not used,
                // complete the request locally.
                //
                requestPtr->SpbRequest = NULL;
                isCompleteRequest = TRUE;
            }
        }
    }

    KeReleaseInStackQueuedSpinLock(&lockHandle);

    //
    // We complete the request outside the device lock protected 
    // zone to avoid getting a new request while the device lock
    // is held.
    //
    if (isCompleteRequest) {

        requestPtr->Type = LPSPI_REQUEST_TYPE::INVALID;
        requestPtr->State = LPSPI_REQUEST_STATE::INACTIVE;

        WdfRequestSetInformation(spbRequest, 0);
        SpbRequestComplete(spbRequest, STATUS_CANCELLED);
    }
}


//
// Routine Description:
//
//  LPSPIEvtInterruptIsr is the LPSPI ISR.
//  It continues driving the active transfer(s), and schedules DPC
//  when transfer is done, to complete the SPB request, or to prepare more
//  transfers during a SEQUENCE request processing.
//
// Arguments:
//
//  WdfInterrupt - The SPI interrupt object
//
//  MessageID - Message ID (not used)
//
// Return Value:
//
//  TRUE: interrupt originated from SPI controller, otherwise FALSE.
//
_Use_decl_annotations_
BOOLEAN
LPSPIEvtInterruptIsr (
    WDFINTERRUPT WdfInterrupt,
    ULONG /*MessageID*/
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = 
        LPSPIDeviceGetExtension(WdfInterruptGetDevice(WdfInterrupt));
    LPSPI_SPB_REQUEST* requestPtr = &devExtPtr->CurrentRequest;

    //
    // Read and ACK current interrupts
    //
    LPSPI_SR statReg;
    {
        volatile LPSPI_REGISTERS* lpspiRegsPtr = devExtPtr->LPSPIRegsPtr;

        statReg.AsUlong = READ_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->SR);
        if ((statReg.AsUlong & 
             LPSPI_INTERRUPT_GROUP::LPSPI_ALL_INTERRUPTS) == 0) {
            //
            // Not our interrupt
            //
            return FALSE;
        }
        WRITE_REGISTER_NOFENCE_ULONG(
            &lpspiRegsPtr->SR,
            (statReg.AsUlong & LPSPI_INTERRUPT_GROUP::LPSPI_ACKABLE_INTERRUPTS)
            );

    } // Read and ACK current interrupts

    LPSPI_SPB_TRANSFER* transfer1Ptr;
    LPSPI_SPB_TRANSFER* transfer2Ptr;
    LPSPISpbGetActiveTransfers(requestPtr, &transfer1Ptr, &transfer2Ptr);

    LPSPI_ASSERT(devExtPtr->IfrLogHandle, transfer1Ptr != nullptr);

    switch (requestPtr->Type) {
    case LPSPI_REQUEST_TYPE::READ:
    { // READ

        if (LPSPIHwReadRxFIFO(devExtPtr, transfer1Ptr)) {

            LPSPI_ASSERT(
                devExtPtr->IfrLogHandle,
                LPSPISpbIsAllDataTransferred(transfer1Ptr)
                );

            LPSPIHwDisableTransferInterrupts(devExtPtr, transfer1Ptr);

            //
            // READ transfer is done.
            // Schedule a DPC to complete the request
            //
            break;
        }
        return TRUE;

    } // READ

    case LPSPI_REQUEST_TYPE::WRITE:
    { // WRITE

        if (LPSPISpbIsTransferDone(transfer1Ptr)) {
            //
            // Make sure all TX data was sent
            //
            if (statReg.TCF == 0) {

                LPSPIHwDisableTransferInterrupts(devExtPtr, transfer1Ptr);

                return TRUE;
            }
            (void)LPSPIHwInterruptControl(
                devExtPtr,
                LPSPI_INTERRUPT_TYPE::ALL,   // Disable all (Tx + Transfer Complete) interrupt
                0x0    // No interrupt to enable
            );

            //
            // WRITE transfer is done.
            // Schedule a DPC to complete the request
            //
            break;
        }

        (void)LPSPIHwWriteTxFIFO(devExtPtr, transfer1Ptr, FALSE);
        return TRUE;

    } // WRITE

    case LPSPI_REQUEST_TYPE::SEQUENCE:
    { // SEQUENCE

        if (LPSPISpbIsWriteTransfer(transfer1Ptr)) {
            //
            // A WRITE transfer
            // 
            if (LPSPISpbIsTransferDone(transfer1Ptr)) {
                //
                // Make sure all TX data was sent
                //
                if (statReg.TCF == 0) {

                    LPSPIHwDisableTransferInterrupts(devExtPtr, transfer1Ptr);

                    return TRUE;
                }

                (void)LPSPIHwInterruptControl(
                    devExtPtr,
                    LPSPI_INTERRUPT_TYPE::ALL,   // Disable all (Tx + Transfer Complete) interrupt
                    0x0    // No interrupt to enable
                );

                //
                // Request is done or more transfers need to be prepared.
                // Schedule a DPC to complete the request, or prepare more
                // transfers.
                //
                LPSPISpbCompleteSequenceTransfer(transfer1Ptr);
                break;
            }

            //
            // Write transfer not done, keep sending data...
            //
            (void)LPSPIHwWriteTxFIFO(devExtPtr, transfer1Ptr, FALSE);
            return TRUE;

        } else {
            //
            // A READ transfer
            // 
            if (LPSPIHwReadRxFIFO(devExtPtr, transfer1Ptr)) {

                LPSPIHwDisableTransferInterrupts(devExtPtr, transfer1Ptr);

                //
                // Request is done or more transfers need to be prepared.
                // Schedule a DPC to complete the request, or prepare more
                // transfers.
                //
                LPSPISpbCompleteSequenceTransfer(transfer1Ptr);
                break;
            }

            return TRUE;
        }

    } // SEQUENCE

    case LPSPI_REQUEST_TYPE::FULL_DUPLEX:
    { // FULL_DUPLEX

        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle, 
            transfer2Ptr != nullptr
            );
        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle, 
            LPSPISpbIsWriteTransfer(transfer1Ptr)
            );
        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle, 
            !LPSPISpbIsWriteTransfer(transfer2Ptr)
            );

        if (!LPSPISpbIsTransferDone(transfer1Ptr)) {

            (void)LPSPIHwWriteTxFIFO(devExtPtr, transfer1Ptr, FALSE);
        }

        if (LPSPIHwReadRxFIFO(devExtPtr, transfer2Ptr)) {

            LPSPI_ASSERT(
                devExtPtr->IfrLogHandle,
                LPSPISpbIsTransferDone(transfer1Ptr)
                );

            LPSPIHwDisableTransferInterrupts(devExtPtr, transfer2Ptr);

            //
            // Both transfers are done.
            // Schedule a DPC to complete the request.
            //
            break;
        }
        return TRUE;

    } // FULL_DUPLEX

    default:
        LPSPI_ASSERT(devExtPtr->IfrLogHandle, FALSE);
        return TRUE;

    } // switch

    //
    // Continue in DPC...
    //
    WdfInterruptQueueDpcForIsr(WdfInterrupt);
    return TRUE;
}


//
// Routine Description:
//
//  LPSPIEvtInterruptDpc is the LPSPI DPC.
//  It is scheduled by LPSPIEvtInterruptDpc to complete requests or to
//  prepare and possible start more transfers during a SEQUENCE request
//  processing.
//
// Arguments:
//
//  WdfInterrupt - The SPI interrupt object
//
//  AssociatedWdfObject - 
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIEvtInterruptDpc (
    WDFINTERRUPT WdfInterrupt,
    WDFOBJECT /*AssociatedWdfObject*/
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr =
        LPSPIDeviceGetExtension(WdfInterruptGetDevice(WdfInterrupt));
    LPSPI_SPB_REQUEST* requestPtr = &devExtPtr->CurrentRequest;

    //
    // Make sure request has not been already canceled, and 
    // prevent request from going away while request processing continues.
    //
    NTSTATUS status = LPSPIDeviceDisableRequestCancellation(requestPtr);
    if (!NT_SUCCESS(status)) {
        //
        // More than one instance of the DPC may have been scheduled 
        // during a multi-transfer sequence request, so the request 
        // cancellation may have already been disabled. 
        // This is a benign case, in which we can just leave...
        //
        if (status != STATUS_NO_WORK_DONE) {

            LPSPI_LOG_WARNING(
                devExtPtr->IfrLogHandle,
                "SPB request already canceled or completed!"
                );
        }
        return;
    }

    switch (requestPtr->Type) {
    case LPSPI_REQUEST_TYPE::READ:
    case LPSPI_REQUEST_TYPE::WRITE:
    case LPSPI_REQUEST_TYPE::FULL_DUPLEX:
    {
        LPSPI_SPB_TRANSFER* transfer1Ptr;
        LPSPI_SPB_TRANSFER* transfer2Ptr;
        LPSPISpbGetActiveTransfers(requestPtr, &transfer1Ptr, &transfer2Ptr);

        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle,
            LPSPISpbIsAllDataTransferred(transfer1Ptr)
            );
        if (transfer2Ptr != nullptr) {

            LPSPI_ASSERT(
                devExtPtr->IfrLogHandle,
                LPSPISpbIsAllDataTransferred(transfer2Ptr)
                );
        }

        break;

    } // READ/WRITE/FULL_DUPLEX

    case LPSPI_REQUEST_TYPE::SEQUENCE:
    { // SEQUENCE
        
        if(LPSPISpbIsRequestDone(requestPtr)) {
            //
            // Continue to request completion
            //
            break;
        }

        status = LPSPISpbPrepareNextTransfer(requestPtr);
        if (!NT_SUCCESS(status) && (status != STATUS_NO_MORE_FILES)) {
            
            LPSPI_ASSERT(devExtPtr->IfrLogHandle, NT_SUCCESS(status));

            //
            // Continue to request completion
            //
            break;
        }

        //
        // Before we continue handling remaining transfers, make sure
        // request has not been canceled, and setup the cancellation routine.
        //
        status = LPSPIDeviceEnableRequestCancellation(requestPtr);
        if (!NT_SUCCESS(status)) {

            return;
        }

        //
        // If request is idle, start the next transfer...
        //
        if (InterlockedExchange(&requestPtr->IsIdle, 0) != 0) {

            status = LPSPISpbStartTransferSafe(requestPtr);
            LPSPI_ASSERT(
                devExtPtr->IfrLogHandle,
                status == STATUS_SUCCESS
                );
        }

        //
        // Continue handling remaining transfers
        //
        return;

    } // SEQUENCE

    default:
        LPSPI_ASSERT(devExtPtr->IfrLogHandle, FALSE);
        return;

    } // switch

    //
    // Request completion
    //

    if (!devExtPtr->IsControllerLocked) {

        status = LPSPIDeviceNegateCS(requestPtr->SpbTargetPtr, FALSE);
        if (status == STATUS_PENDING) {

            return;
        }
    }

    //
    // Complete the request, we get here when one of the following is TRUE:
    // - Not using GPIO for CS.
    // - CS negation failed.
    // - Controller is locked so we do not negate CS.
    //
    LPSPISpbCompleteTransferRequest(
        requestPtr,
        status,
        requestPtr->TotalBytesTransferred
        );
}


//
// Routine Description:
//
//  LPSPIDeviceEnableRequestCancellation is called to set the current request 
//  cancellation routine. 
//  If the request is already canceled, the routine completes the 
//  request if needed.
//
// Arguments:
//
//  RequestPtr - The request object.
//
// Return Value:
//
//  NTSTATUS
//
_Use_decl_annotations_
NTSTATUS
LPSPIDeviceEnableRequestCancellation (
    LPSPI_SPB_REQUEST* RequestPtr
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = RequestPtr->SpbTargetPtr->DevExtPtr;

    KLOCK_QUEUE_HANDLE lockHandle;
    KeAcquireInStackQueuedSpinLock(&devExtPtr->DeviceLock, &lockHandle);

    SPBREQUEST spbRequest = RequestPtr->SpbRequest;
    LPSPI_ASSERT(devExtPtr->IfrLogHandle, spbRequest != NULL);

    NTSTATUS status = WdfRequestMarkCancelableEx(spbRequest, LPSPIEvtRequestCancel);
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "WdfRequestMarkCancelableEx failed. "
            "request %p, status = %!STATUS!",
            spbRequest,
            status
            );

        LPSPISpbAbortAllTransfers(RequestPtr);
        LPSPISpbCompleteTransferRequest(RequestPtr, status, 0);

    } else {

        RequestPtr->State = LPSPI_REQUEST_STATE::CANCELABLE;
    }

    KeReleaseInStackQueuedSpinLock(&lockHandle);

    return status;
}


//
// Routine Description:
//
//  LPSPIDeviceDisableRequestCancellation is called to remove the current 
//  request cancellation routine, so it cannot be canceled.
//  If the request is already canceled, the routine completes the 
//  request if needed.
//
// Arguments:
//
//  RequestPtr - The request object.
//
// Return Value:
//
//  STATUS_NO_WORK_DONE is the request cancellation has 
//  already been disabled, which is benign or NTSTATUS.
//
_Use_decl_annotations_
NTSTATUS
LPSPIDeviceDisableRequestCancellation (
    LPSPI_SPB_REQUEST* RequestPtr
    )
{
    NTSTATUS status;
    LPSPI_DEVICE_EXTENSION* devExtPtr = RequestPtr->SpbTargetPtr->DevExtPtr;

    KLOCK_QUEUE_HANDLE lockHandle;
    KeAcquireInStackQueuedSpinLock(&devExtPtr->DeviceLock, &lockHandle);

    SPBREQUEST spbRequest = RequestPtr->SpbRequest;
    if (spbRequest == NULL) {

        LPSPI_LOG_INFORMATION(
            devExtPtr->IfrLogHandle,
            "SPB request already canceled!"
            );
        status = STATUS_NO_WORK_DONE;
        goto done;
    }

    if (RequestPtr->State != LPSPI_REQUEST_STATE::CANCELABLE) {

        status = STATUS_NO_WORK_DONE;
        goto done;
    }

    status = WdfRequestUnmarkCancelable(spbRequest);
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "WdfRequestUnmarkCancelable failed. "
            "Request = 0x%p, status = %!STATUS!",
            spbRequest,
            status
            );

        LPSPISpbAbortAllTransfers(RequestPtr);

        if (status != STATUS_CANCELLED) {

            LPSPISpbCompleteTransferRequest(RequestPtr, status, 0);
        }
        goto done;
    }

    RequestPtr->State = LPSPI_REQUEST_STATE::NOT_CANCELABLE;
    status = STATUS_SUCCESS;

done:

    KeReleaseInStackQueuedSpinLock(&lockHandle);

    return status;
}


//
// Routine Description:
//
//  LPSPIDeviceOpenGpioTarget is called to create and open the
//  GPIO target device.
//  Since LPSPI burst length is limited to 512 bytes, we need to extend it
//  on the fly for long transfers.
//  To avoid CS negation between bursts, we control the CS with a GPIO pin.
//
// Arguments:
//
//  TrgCtxPtr - The target context
//
// Return Value:
//
//  NTSTATUS
//
_Use_decl_annotations_
NTSTATUS
LPSPIDeviceOpenGpioTarget (
    LPSPI_TARGET_CONTEXT* TrgCtxPtr
    )
{
    NTSTATUS status;
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;
    LPSPI_CS_GPIO_PIN* csGpioPinPtr = LPSPIDeviceGetCsGpio(TrgCtxPtr);

    WDFIOTARGET wdfIoTargetGpio = NULL;
    WDFREQUEST wdfRequestGpioAssert = NULL;
    WDFREQUEST wdfRequestGpioNegate = NULL;
    WDFMEMORY wdfMemoryGpio = NULL;

    if (csGpioPinPtr->GpioConnectionId.QuadPart == 0) {
        //
        // We do not mandate using a GPIO pin for CS.
        //
        status = STATUS_SUCCESS;
        goto done;
    }

    if (csGpioPinPtr->WdfIoTargetGpio != NULL) {

        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle,
            csGpioPinPtr->OpenCount != 0
            );
        status = STATUS_SUCCESS;
        goto done;
    }

    //
    // Create and open the IO target
    //
    {
        status = WdfIoTargetCreate(
            devExtPtr->WdfDevice,
            WDF_NO_OBJECT_ATTRIBUTES,
            &wdfIoTargetGpio
            );
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "WdfIoTargetCreate failed, "
                "(status = %!STATUS!)",
                status
                );
            goto done;
        }

        DECLARE_UNICODE_STRING_SIZE(gpioDeviceName, RESOURCE_HUB_PATH_CHARS);
        status = RESOURCE_HUB_CREATE_PATH_FROM_ID(
            &gpioDeviceName,
            csGpioPinPtr->GpioConnectionId.LowPart,
            csGpioPinPtr->GpioConnectionId.HighPart
            );
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "RESOURCE_HUB_CREATE_PATH_FROM_ID failed for GPIO connection, "
                "(status = %!STATUS!)",
                status
                );
            goto done;
        }

        WDF_IO_TARGET_OPEN_PARAMS wdfIoTargetOpenParams;
        WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME(
            &wdfIoTargetOpenParams,
            &gpioDeviceName,
            FILE_GENERIC_WRITE
            );
        wdfIoTargetOpenParams.ShareAccess = FILE_SHARE_WRITE;

        status = WdfIoTargetOpen(
            wdfIoTargetGpio,
            &wdfIoTargetOpenParams
            );
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "WdfIoTargetOpen failed, "
                "(status = %!STATUS!)",
                status
                );
            goto done;
        }

    } // Create and open the IO target

    //
    // Create the GPIO requests and associated resources
    //
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = wdfIoTargetGpio;

        status = WdfRequestCreate(
            &attributes,
            wdfIoTargetGpio,
            &wdfRequestGpioAssert
            );
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "WdfRequestCreate failed, for GPIO assert request"
                "(status = %!STATUS!)",
                status
                );
            goto done;
        }

        status = WdfRequestCreate(
            &attributes,
            wdfIoTargetGpio,
            &wdfRequestGpioNegate
            );
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "WdfRequestCreate failed, for GPIO negate request"
                "(status = %!STATUS!)",
                status
            );
            goto done;
        }

        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = wdfIoTargetGpio;

        status = WdfMemoryCreatePreallocated(
            &attributes,
            &csGpioPinPtr->GpioData,
            sizeof(UCHAR),
            &wdfMemoryGpio
            );
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "WdfMemoryCreatePreallocated failed, for GPIO request data"
                "(status = %!STATUS!)",
                status
                );
            goto done;
        }

    } // Create the GPIO request and associated resources

    csGpioPinPtr->WdfIoTargetGpio = wdfIoTargetGpio;
    csGpioPinPtr->WdfRequestGpioAssert = wdfRequestGpioAssert;
    csGpioPinPtr->WdfRequestGpioNegate = wdfRequestGpioNegate;
    csGpioPinPtr->WdfMemoryGpio = wdfMemoryGpio;
    status = STATUS_SUCCESS;

done:

    if (NT_SUCCESS(status)) {

        csGpioPinPtr->OpenCount += 1;

    } else {

        if (wdfIoTargetGpio != NULL) {

            WdfObjectDelete(wdfIoTargetGpio);
        }
    }

    return status;
}


//
// Routine Description:
//
//  LPSPIDeviceCloseGpioTarget closes the GPIO target device, 
//  and releases associated resources.
//  Since LPSPI burst length is limited to 512 bytes, we need to extend it
//  on the fly for long transfers.
//  To avoid CS negation between bursts, we control the CS with a GPIO pin.
//
// Arguments:
//
//  TrgCtxPtr - The target context
//
// Return Value:
//
//  NTSTATUS
//
_Use_decl_annotations_
VOID
LPSPIDeviceCloseGpioTarget (
    LPSPI_TARGET_CONTEXT* TrgCtxPtr
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;
    LPSPI_CS_GPIO_PIN* csGpioPinPtr = LPSPIDeviceGetCsGpio(TrgCtxPtr);

    LPSPI_ASSERT(
        devExtPtr->IfrLogHandle,
        csGpioPinPtr->OpenCount != 0
        );

    csGpioPinPtr->OpenCount -= 1;

    if (csGpioPinPtr->OpenCount == 0) {

        if (csGpioPinPtr->WdfIoTargetGpio != NULL) {

            WdfObjectDelete(csGpioPinPtr->WdfIoTargetGpio);
        }
        csGpioPinPtr->WdfIoTargetGpio = NULL;
        csGpioPinPtr->WdfRequestGpioAssert = NULL;
        csGpioPinPtr->WdfRequestGpioNegate = NULL;
        csGpioPinPtr->WdfMemoryGpio = NULL;
    }
}


//
// Routine Description:
//
//  LPSPIDeviceAssertCS asserts the CS GPIO pin.
//  If no GPIO pin resource is available, it starts the transfer directly.
//
// Arguments:
//
//  TrgCtxPtr - The target context
//
//  IsWait - If to wait for CS negation completion
//
// Return Value:
//
//  NTSTATUS
//
_Use_decl_annotations_
NTSTATUS
LPSPIDeviceAssertCS (
    LPSPI_TARGET_CONTEXT* TrgCtxPtr,
    BOOLEAN IsWait
    )
{
    LPSPI_CS_GPIO_PIN* csGpioPinPtr = LPSPIDeviceGetCsGpio(TrgCtxPtr);
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;
    LPSPI_SPB_REQUEST* requestPtr = &devExtPtr->CurrentRequest;

    //
    // Select (reset) the target (there is no other way to reset RX/TX FIFO).
    // All interrupts are disabled.
    //
    LPSPIHwSelectTarget(TrgCtxPtr);

    if (csGpioPinPtr->GpioConnectionId.QuadPart == 0) {
        //
        // Not using GPIO for CS
        //
        SPBREQUEST lockRequest = devExtPtr->LockUnlockRequest;
        if (lockRequest != NULL) {

            LPSPI_ASSERT(
                devExtPtr->IfrLogHandle,
                requestPtr->SpbRequest == NULL
                );
            devExtPtr->LockUnlockRequest = NULL;
            SpbRequestComplete(lockRequest, STATUS_SUCCESS);
            return STATUS_SUCCESS;
        }

        NTSTATUS status = LPSPIDeviceEnableRequestCancellation(requestPtr);
        if (!NT_SUCCESS(status)) {

            return status;
        }

        return LPSPISpbStartTransferSafe(requestPtr);
    }

    KeResetEvent(&csGpioPinPtr->SyncCallEvent);

    NTSTATUS status = LPSPIpDeviceSetGpioPin(
        TrgCtxPtr,
        LPSPISpbGetCsActiveValue(TrgCtxPtr)
        );
    if (!NT_SUCCESS(status) && (status != STATUS_PENDING)) {

        return status;
    }

    if (!IsWait) {

        return STATUS_PENDING;
    }

    status = KeWaitForSingleObject(
        &csGpioPinPtr->SyncCallEvent,
        Executive,
        KernelMode,
        FALSE,
        nullptr
        );
    LPSPI_ASSERT(
        TrgCtxPtr->DevExtPtr->IfrLogHandle,
        status == STATUS_WAIT_0
        );

    return STATUS_SUCCESS;
}


//
// Routine Description:
//
//  LPSPIDeviceNegateCS negates the CS GPIO pin.
//  If no GPIO pin resource is available, it starts the transfer directly.
//
// Arguments:
//
//  TrgCtxPtr - The target context
//
//  IsWait - If to wait for CS negation completion
//
// Return Value:
//
//  NTSTATUS
//
_Use_decl_annotations_
NTSTATUS
LPSPIDeviceNegateCS (
    LPSPI_TARGET_CONTEXT* TrgCtxPtr,
    BOOLEAN IsWait
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;
    LPSPI_CS_GPIO_PIN* csGpioPinPtr = LPSPIDeviceGetCsGpio(TrgCtxPtr);

    if (csGpioPinPtr->GpioConnectionId.QuadPart == 0) {
        //
        // Not using GPIO for CS
        //
        SPBREQUEST lockRequest = devExtPtr->LockUnlockRequest;
        if (lockRequest != NULL) {

            LPSPI_ASSERT(
                devExtPtr->IfrLogHandle,
                devExtPtr->CurrentRequest.SpbRequest == NULL
                );
            devExtPtr->LockUnlockRequest = NULL;
            SpbRequestComplete(lockRequest, STATUS_SUCCESS);
            return STATUS_SUCCESS;
        }

        return STATUS_SUCCESS;
    }

    KeResetEvent(&csGpioPinPtr->SyncCallEvent);

    NTSTATUS status = LPSPIpDeviceSetGpioPin(
        TrgCtxPtr,
        LPSPISpbGetCsNonActiveValue(TrgCtxPtr)
        );
    if (!NT_SUCCESS(status) && (status != STATUS_PENDING)) {

        return status;
    }

    if (!IsWait) {

        return STATUS_PENDING;
    }

    status = KeWaitForSingleObject(
        &csGpioPinPtr->SyncCallEvent,
        Executive,
        KernelMode,
        FALSE,
        nullptr
        );
    LPSPI_ASSERT(
        TrgCtxPtr->DevExtPtr->IfrLogHandle,
        status == STATUS_WAIT_0
        );

    return STATUS_SUCCESS;
}


//
// LPSPIdevice private methods
//


//
// Routine Description:
//
//  LPSPIpDeviceLogInit is called to create a per device log.
//  To differentiate among the LPSPI devices, the routine creates
//  a separate trace log for the device and uses the given device unique 
//  DeviceLogId the identifier.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
//  DeviceLogId - The unique device trace ID
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIpDeviceLogInit (
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    ULONG DeviceLogId
    )
{
    RECORDER_LOG_CREATE_PARAMS recorderLogCreateParams;
    RECORDER_LOG_CREATE_PARAMS_INIT(&recorderLogCreateParams, nullptr);

    recorderLogCreateParams.TotalBufferSize = LPSPI_RECORDER_TOTAL_BUFFER_SIZE;
    recorderLogCreateParams.ErrorPartitionSize = LPSPI_RECORDER_ERROR_PARTITION_SIZE;

    RtlStringCchPrintfA(
        recorderLogCreateParams.LogIdentifier,
        sizeof(recorderLogCreateParams.LogIdentifier),
        LPSPI_RECORDER_LOG_ID,
        DeviceLogId
        );

    NTSTATUS status = WppRecorderLogCreate(
        &recorderLogCreateParams,
        &DevExtPtr->IfrLogHandle
        );
    if (!NT_SUCCESS(status)) {

        NT_ASSERTMSG(
            "Unable to create trace log recorder!"
            "Default log will be used instead", 
            FALSE
            );
        DevExtPtr->IfrLogHandle = WppRecorderLogGetDefault();
    }
}


//
// Routine Description:
//
//  LPSPIpDeviceLogDeinit is called to delete the device trace log.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIpDeviceLogDeinit (
    LPSPI_DEVICE_EXTENSION* DevExtPtr
    )
{
    if (DevExtPtr->IfrLogHandle != NULL) {

        WppRecorderLogDelete(DevExtPtr->IfrLogHandle);
    }
    DevExtPtr->IfrLogHandle = NULL;
}


//
// Routine Description:
//
//  LPSPIpDeviceSetGpioPin is called to change the state of the
//  CS GPIO pin.
//
// Arguments:
//
//  TrgCtxPtr - The target context.
//
//  AssertNegateValue - The new GPIO pin value,
//      should be LPSPISpbGetCsActiveValue(TrgCtxPtr) or 
//      LPSPISpbGetCsNonActiveValue(TrgCtxPtr).
//
// Return Value:
//
//  NT_STATUS
//
_Use_decl_annotations_
NTSTATUS
LPSPIpDeviceSetGpioPin (
    LPSPI_TARGET_CONTEXT* TrgCtxPtr,
    ULONG AssertNegateValue
    )
{
    NTSTATUS status;
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;

    LPSPI_ASSERT(
        devExtPtr->IfrLogHandle,
        (AssertNegateValue == LPSPISpbGetCsActiveValue(TrgCtxPtr)) ||
            (AssertNegateValue == LPSPISpbGetCsNonActiveValue(TrgCtxPtr))
        );

    //
    // Prepare the GPIO request
    //
    LPSPI_CS_GPIO_PIN* csGpioPinPtr = LPSPIDeviceGetCsGpio(TrgCtxPtr);
    LPSPI_SPB_REQUEST* requestPtr = &devExtPtr->CurrentRequest;
    WDFIOTARGET wdfIoTargetGpio = csGpioPinPtr->WdfIoTargetGpio;
    WDFMEMORY wdfMemoryGpio = csGpioPinPtr->WdfMemoryGpio;
    BOOLEAN isAssert = AssertNegateValue == LPSPISpbGetCsActiveValue(TrgCtxPtr);

    WDFREQUEST wdfRequestGpio = isAssert ? 
        csGpioPinPtr->WdfRequestGpioAssert : 
        csGpioPinPtr->WdfRequestGpioNegate;
    {
        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle,
            (wdfIoTargetGpio != NULL) &&
            (wdfRequestGpio != NULL) &&
            (wdfMemoryGpio != NULL)
            );

        WDF_REQUEST_REUSE_PARAMS wdfRequestReuseParams;
        WDF_REQUEST_REUSE_PARAMS_INIT(
            &wdfRequestReuseParams,
            WDF_REQUEST_REUSE_NO_FLAGS,
            STATUS_SUCCESS
            );

        status = WdfRequestReuse(wdfRequestGpio, &wdfRequestReuseParams);
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "WdfRequestReuse failed for Read Request, "
                "(status = %!STATUS!)",
                status
                );
            return status;
        }

        csGpioPinPtr->GpioData = AssertNegateValue;

        status = WdfIoTargetFormatRequestForIoctl(
            wdfIoTargetGpio,
            wdfRequestGpio,
            IOCTL_GPIO_WRITE_PINS,
            wdfMemoryGpio,
            nullptr,
            NULL,
            nullptr
            );
        if (!NT_SUCCESS(status)) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "WdfIoTargetFormatRequestForIoctl failed, for GPIO request"
                "(status = %!STATUS!)",
                status
                );
            return status;
        }

        WdfRequestSetCompletionRoutine(
            wdfRequestGpio,
            isAssert ? 
                LPSPIpGpioAssertCompletionRoutine : 
                LPSPIpGpioNegateCompletionRoutine,
            requestPtr
            );

    } // Prepare the GPIO request

    BOOLEAN isRequestSent = WdfRequestSend(
        wdfRequestGpio,
        wdfIoTargetGpio,
        WDF_NO_SEND_OPTIONS
        );
    if (!isRequestSent) {

        status = WdfRequestGetStatus(wdfRequestGpio);
        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "WdfRequestSend failed for 'CS Assert' GPIO Request, "
            "(status = %!STATUS!)",
            status
            );
        return status;
    }

    return STATUS_PENDING;
}


//
// Routine Description:
//
//  This is completion routine of the CS GPIO pin assertion.
//  If request was successful the routine starts the transfer.
//
// Arguments:
//
//  WdfRequest - The IOCTL_GPIO_WRITE_PINS request.
//
//  WdfIoTarget - The GPIO driver WDF IO Target
//
//  WdfRequestCompletopnParamsPtr - Completion parameters
//
//  WdfContext - The LPSPI_SPB_REQUEST.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIpGpioAssertCompletionRoutine (
    WDFREQUEST /*WdfRequest*/,
    WDFIOTARGET /*WdfIoTarget*/,
    PWDF_REQUEST_COMPLETION_PARAMS WdfRequestCompletopnParamsPtr,
    WDFCONTEXT WdfContext
    )
{
    LPSPI_SPB_REQUEST* requestPtr =
        static_cast<LPSPI_SPB_REQUEST*>(WdfContext);
    LPSPI_TARGET_CONTEXT* trgCtxPtr = requestPtr->SpbTargetPtr;
    LPSPI_DEVICE_EXTENSION* devExtPtr = trgCtxPtr->DevExtPtr;
    LPSPI_CS_GPIO_PIN* csGpioPinPtr = LPSPIDeviceGetCsGpio(trgCtxPtr);

    KeSetEvent(&csGpioPinPtr->SyncCallEvent, 0, FALSE);

    //
    // During lock controller request...
    //
    {
        SPBREQUEST lockRequest = devExtPtr->LockUnlockRequest;
        if (lockRequest != NULL) {

            LPSPI_ASSERT(
                devExtPtr->IfrLogHandle,
                requestPtr->SpbRequest == NULL
                );
            devExtPtr->LockUnlockRequest = NULL;
            SpbRequestComplete(lockRequest, STATUS_SUCCESS);
            return;
        }

    } // During lock controller request...

    //
    // Normal transfer
    //
    {
        ULONG_PTR information = 0;
        NTSTATUS status = WdfRequestCompletopnParamsPtr->IoStatus.Status;
        if (!NT_SUCCESS(status)) {

            LPSPISpbCompleteTransferRequest(requestPtr, status, information);
            return;
        }

        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle,
            csGpioPinPtr->GpioData ==
            LPSPISpbGetCsActiveValue(requestPtr->SpbTargetPtr)
            );

        status = LPSPIDeviceEnableRequestCancellation(requestPtr);
        if (!NT_SUCCESS(status)) {

            return;
        }

        //
        // Now that CS is asserted we can start the transfer...
        //
        status = LPSPISpbStartTransferSafe(requestPtr);
        if (!NT_SUCCESS(status)) {

            LPSPISpbCompleteTransferRequest(requestPtr, status, information);
            return;
        }

        //
        // Transfer has started successfully!
        //

    } // Normal transfer
}


//
// Routine Description:
//
//  This is completion routine of the CS GPIO pin negation.
//  The routine complete the SPB transfer requests.
//
// Arguments:
//
//  WdfRequest - The IOCTL_GPIO_WRITE_PINS request.
//
//  WdfIoTarget - The GPIO driver WDF IO Target
//
//  WdfRequestCompletopnParamsPtr - Completion parameters
//
//  WdfContext - The LPSPI_SPB_REQUEST.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIpGpioNegateCompletionRoutine (
    WDFREQUEST /*WdfRequest*/,
    WDFIOTARGET /*WdfIoTarget*/,
    PWDF_REQUEST_COMPLETION_PARAMS WdfRequestCompletopnParamsPtr,
    WDFCONTEXT WdfContext
    )
{
    LPSPI_SPB_REQUEST* requestPtr =
        static_cast<LPSPI_SPB_REQUEST*>(WdfContext);
    LPSPI_TARGET_CONTEXT* trgCtxPtr = requestPtr->SpbTargetPtr;
    LPSPI_DEVICE_EXTENSION* devExtPtr = trgCtxPtr->DevExtPtr;
    LPSPI_CS_GPIO_PIN* csGpioPinPtr = LPSPIDeviceGetCsGpio(trgCtxPtr);

    KeSetEvent(&csGpioPinPtr->SyncCallEvent, 0, FALSE);

    //
    // During controller unlock request...
    //
    {
        SPBREQUEST unlockRequest = devExtPtr->LockUnlockRequest;
        if (unlockRequest != NULL) {

            LPSPI_ASSERT(
                devExtPtr->IfrLogHandle,
                requestPtr->SpbRequest == NULL
            );
            devExtPtr->LockUnlockRequest = NULL;
            SpbRequestComplete(unlockRequest, STATUS_SUCCESS);
            return;
        }

    } // During controller unlock request...

    //
    // Normal transfer...
    //
    {
        ULONG_PTR information = 0;
        NTSTATUS status = WdfRequestCompletopnParamsPtr->IoStatus.Status;

        //
        // We do not need to sync access to requestPtr->SpbRequest with
        // cancel routine, since LPSPIDeviceNegateCS was either called from 
        // the DPC, where cancel routine has been removed, or from 
        // the cancel routine itself.
        //
        SPBREQUEST spbRequest = requestPtr->SpbRequest;
        if (spbRequest != NULL) {

            LPSPI_ASSERT(
                devExtPtr->IfrLogHandle,
                csGpioPinPtr->GpioData ==
                    LPSPISpbGetCsNonActiveValue(requestPtr->SpbTargetPtr)
                );

            if (requestPtr->State == LPSPI_REQUEST_STATE::CANCELED) {

                status = STATUS_CANCELLED;

            } else {

                information = requestPtr->TotalBytesTransferred;
            }

            LPSPISpbCompleteTransferRequest(requestPtr, status, information);
        }

    } // Normal transfer...
}

/*++

  Routine Description:

    This routine gets SPI controller configuration parametrs from registry

  Arguments:

    Driver - a pointer to the WDFDRIVER driver object
    Device - a pointer to the device context

  Return Value:

    status of operation.

--*/
_Use_decl_annotations_
NTSTATUS LPSPIGetConfigValues(
    LPSPI_CONFIG_DATA* DriverSpiConfigPtr,
    WDFDEVICE Device)
{
    NTSTATUS status = STATUS_SUCCESS;
    DECLARE_UNICODE_STRING_SIZE(valueName, PARAMATER_NAME_LEN);
    ACPI_EVAL_OUTPUT_BUFFER* dsdBufferPtr = nullptr;
    DEVICE_OBJECT* pdoPtr = WdfDeviceWdmGetPhysicalDevice(Device);
    const ACPI_METHOD_ARGUMENT* devicePropertiesPkgPtr;

    UNREFERENCED_PARAMETER(Device);

    if (NULL == DriverSpiConfigPtr) {
        LPSPI_LOG_ERROR(
            DRIVER_LOG_HANDLE,
            "LPSPIGetConfigValues() - error NULL pointer, "
            "wdfDevice = %p",
            Device
        );
        goto DoneGetParams;
    }

    DriverSpiConfigPtr->ReferenceClockHz = 0;
    DriverSpiConfigPtr->MaxConnectionSpeedHz = 0;
    DriverSpiConfigPtr->SampleOnDelayedSckEdge = FALSE;

    status = AcpiQueryDsd(pdoPtr, &dsdBufferPtr);

    if (!NT_SUCCESS(status)) {
        LPSPI_LOG_ERROR(
            DRIVER_LOG_HANDLE,
            "LPSPIGetConfigValues() - AcpiQueryDsd failed, "
            "wdfDevice = %p, status = %!STATUS!",
            Device,
            status
        );
        goto DoneGetParams;
    }

    status = AcpiParseDsdAsDeviceProperties(dsdBufferPtr, &devicePropertiesPkgPtr);

    if (!NT_SUCCESS(status)) {
        LPSPI_LOG_ERROR(
            DRIVER_LOG_HANDLE,
            "LPSPIGetConfigValues() - AcpiParseSdsAsDeviceProperties failed, "
            "wdfDevice = %p, status = %!STATUS!",
            Device,
            status
        );
        goto DoneGetParams;
    }

    status = AcpiDevicePropertiesQueryIntegerValue(devicePropertiesPkgPtr,
        "ReferenceClockHz", &DriverSpiConfigPtr->ReferenceClockHz);

    if (!NT_SUCCESS(status)) {
        LPSPI_LOG_ERROR(
            DRIVER_LOG_HANDLE,
            "LPSPIGetConfigValues() - Failed ReferenceClockHz query to ACPI, "
            "wdfDevice = %p, status = %!STATUS!",
            Device,
            status
        );
        goto DoneGetParams;
    }

    status = AcpiDevicePropertiesQueryIntegerValue(devicePropertiesPkgPtr,
        "MaxConnectionSpeedHz", &DriverSpiConfigPtr->MaxConnectionSpeedHz);

    if (!NT_SUCCESS(status)) {
        LPSPI_LOG_INFORMATION(
            DRIVER_LOG_HANDLE,
            "LPSPIGetConfigValues() - Failed MaxConnectionSpeedHz query to ACPI, "
            "wdfDevice = %p, status = %!STATUS!",
            Device,
            status
        );
    }

    status = AcpiDevicePropertiesQueryIntegerValue(devicePropertiesPkgPtr,
        "SampleOnDelayedSckEdge", &DriverSpiConfigPtr->SampleOnDelayedSckEdge);

    if (!NT_SUCCESS(status)) {
        LPSPI_LOG_INFORMATION(
            DRIVER_LOG_HANDLE,
            "LPSPIGetConfigValues() - Failed SampleOnDelayedSckEdge query to ACPI, "
            "wdfDevice = %p, status = %!STATUS!",
            Device,
            status
        );
    }

DoneGetParams:
    if (dsdBufferPtr != nullptr) {
        ExFreePool(dsdBufferPtr);
    }

    return status;
}

#undef _LPSPI_DEVICE_CPP_