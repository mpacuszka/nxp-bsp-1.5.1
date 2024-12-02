// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//    LPSPIspb.cpp
//
// Abstract:
//
//    This module contains the implementation of the IMX LPSPI controller driver
//    SpbCx callback functions.
//    This controller driver uses the SPB WDF class extension (SpbCx).
//
// Environment:
//
//    kernel-mode only
//
#include "precomp.h"
#pragma hdrstop

#define _LPSPI_SPB_CPP_

// Logging header files
#include "LPSPItrace.h"
#include "LPSPIspb.tmh"

// Common driver header files
#include "LPSPIcommon.h"

// Module specific header files
#include "LPSPIhw.h"
#include "LPSPIspb.h"
#include "LPSPIdevice.h"


#ifdef ALLOC_PRAGMA
    #pragma alloc_text(PAGE, LPSPIEvtSpbTargetConnect)
    #pragma alloc_text(PAGE, LPSPIEvtSpbTargetDisconnect)
#endif


//
// Routine Description:
//
//  LPSPIEvtSpbTargetConnect is called by the framework when a peripheral 
//  driver opens a target. 
//  The routine retrieves and validates target settings. 
//  If settings are valid, target context is updated
//
// Arguments:
//
//  Controller - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  SpbTarget - The SPB target object
//
// Return Value:
//
//  NTSTATUS
//
_Use_decl_annotations_
NTSTATUS
LPSPIEvtSpbTargetConnect (
    WDFDEVICE Controller,
    SPBTARGET SpbTarget
    )
{
    PAGED_CODE();

    LPSPI_DEVICE_EXTENSION* devExtPtr = LPSPIDeviceGetExtension(Controller);

    //
    // Get target connection parameters.
    //
    const PNP_SERIAL_BUS_DESCRIPTOR* serialBusDescriptor;
    const PNP_SPI_SERIAL_BUS_DESCRIPTOR* spiSerialBusDescriptorPtr;
    {
        SPB_CONNECTION_PARAMETERS spbConnectionParams;
        SPB_CONNECTION_PARAMETERS_INIT(&spbConnectionParams);
        SpbTargetGetConnectionParameters(SpbTarget, &spbConnectionParams);

        RH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER* connectionPropPtr =
            static_cast<RH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER*>(
                spbConnectionParams.ConnectionParameters
                );
        if (connectionPropPtr->PropertiesLength < sizeof(PNP_SPI_SERIAL_BUS_DESCRIPTOR)) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "Invalid connection parameters "
                "for PNP_SPI_SERIAL_BUS_DESCRIPTOR! "
                "Size %lu, required size %lu",
                connectionPropPtr->PropertiesLength,
                sizeof(PNP_SPI_SERIAL_BUS_DESCRIPTOR)
                );
            return STATUS_INVALID_PARAMETER;
        }

        spiSerialBusDescriptorPtr =
            reinterpret_cast<const PNP_SPI_SERIAL_BUS_DESCRIPTOR*>(
                &connectionPropPtr->ConnectionProperties
                );
        serialBusDescriptor = &spiSerialBusDescriptorPtr->SerialBusDescriptor;
        if (serialBusDescriptor->SerialBusType != PNP_SERIAL_BUS_TYPE_SPI) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "Invalid connection parameters, not a SPI descriptor!"
                "Type 0x%lX, required type 0x%lX",
                serialBusDescriptor->SerialBusType,
                PNP_SERIAL_BUS_TYPE_SPI
                );
            return STATUS_INVALID_PARAMETER;
        }

    } // Get target connection parameters.

    if ((serialBusDescriptor->GeneralFlags & PNP_SERIAL_GENERAL_FLAGS_SLV_BIT)
        != 0) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "Slave mode not supported."
            );
        return STATUS_NOT_SUPPORTED;
    }

    if ((serialBusDescriptor->TypeSpecificFlags & PNP_SPI_WIREMODE_BIT) != 0) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "3-wire mode not supported."
            );
        return STATUS_NOT_SUPPORTED;
    }

    if (!LPSPIHwIsSupportedDataBitLength(
            spiSerialBusDescriptorPtr->DataBitLength)
            ) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "Data bit length of %d is not supported! "
            "Driver supports 8/16/32 data bit length.",
            spiSerialBusDescriptorPtr->DataBitLength
            );
        return STATUS_NOT_SUPPORTED;
    }

    //
    // Validate connection speed
    //
    {
        ULONG minClockHz, maxClockHz;
        LPSPIHwGetClockRange(devExtPtr, &minClockHz, &maxClockHz);

        if ((spiSerialBusDescriptorPtr->ConnectionSpeed > maxClockHz) ||
            (spiSerialBusDescriptorPtr->ConnectionSpeed < minClockHz)) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "Connection speed %luHz is not supported. "
                "Supported range: %luHz..%luHz",
                spiSerialBusDescriptorPtr->ConnectionSpeed,
                minClockHz,
                maxClockHz
                );
            return STATUS_NOT_SUPPORTED;
        }

    } // Validate connection speed

    if (spiSerialBusDescriptorPtr->DeviceSelection >= LPSPI_CHANNEL::COUNT) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "Device selection %d, is not supported. "
            "Supported device selection: %d..%d",
            spiSerialBusDescriptorPtr->DeviceSelection,
            LPSPI_CHANNEL::CS0,
            LPSPI_CHANNEL::CS1
            );
        return STATUS_NOT_SUPPORTED;
    }

    //
    // Initialize TARGET context.
    //
    LPSPI_TARGET_CONTEXT* trgCtxPtr = LPSPITargetGetContext(SpbTarget);
    {
        LPSPI_TARGET_SETTINGS* trgSettingsPtr = &trgCtxPtr->Settings;

        trgCtxPtr->SpbTarget = SpbTarget;
        trgCtxPtr->DevExtPtr = devExtPtr;

        trgSettingsPtr->ConnectionSpeed =
            spiSerialBusDescriptorPtr->ConnectionSpeed;
        trgSettingsPtr->Polarity = spiSerialBusDescriptorPtr->Polarity;
        trgSettingsPtr->Phase = spiSerialBusDescriptorPtr->Phase;
        trgSettingsPtr->DeviceSelection = 
            spiSerialBusDescriptorPtr->DeviceSelection;
        trgSettingsPtr->DataBitLength = 
            spiSerialBusDescriptorPtr->DataBitLength;

        trgSettingsPtr->BufferStride = 
            LPSPISpbGetBufferStride(trgSettingsPtr->DataBitLength);
        
        if ((serialBusDescriptor->TypeSpecificFlags &
             PNP_SPI_FLAGS::PNP_SPI_DEVICEPOLARITY_BIT) != 0) {

            trgSettingsPtr->CsActiveValue = 1;

        } else {

            trgSettingsPtr->CsActiveValue = 0;
        }

        LPSPI_LOG_INFORMATION(
            devExtPtr->IfrLogHandle,
            "New target settings: Target %p, "
            "Channel %lu, flags (gen/type) 0x%lX/0x%lX, speed %luHz, "
            "Data length %lu bits, polarity %lu, phase %lu",
            trgCtxPtr,
            trgSettingsPtr->DeviceSelection,
            serialBusDescriptor->GeneralFlags,
            serialBusDescriptor->TypeSpecificFlags,
            trgSettingsPtr->ConnectionSpeed,
            trgSettingsPtr->DataBitLength,
            trgSettingsPtr->Polarity,
            trgSettingsPtr->Phase
            );

    } // Initialize TARGET context.

    NTSTATUS status = LPSPIDeviceOpenGpioTarget(trgCtxPtr);
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "LPSPIDeviceOpenGpioTarget failed, "
            "wdfDevice = %p, status = %!STATUS!",
            Controller,
            status
        );
        return status;
    }

    //
    // Calculate the HW setup.
    //
    status = LPSPIHwSetTargetConfiguration(trgCtxPtr);
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "LPSPIHwSetTargetConfiguration failed, "
            "wdfDevice = %p, status = %!STATUS!",
            Controller,
            status
            );
        LPSPIDeviceCloseGpioTarget(trgCtxPtr);
        return status;
    }

    //
    // Associate the new target with the 
    // request object.
    //
    devExtPtr->CurrentRequest.SpbTargetPtr = trgCtxPtr;

    status = LPSPIDeviceNegateCS(trgCtxPtr, TRUE);
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "LPSPIDeviceNegateCS failed, "
            "wdfDevice = %p, status = %!STATUS!",
            Controller,
            status
            );
        LPSPIDeviceCloseGpioTarget(trgCtxPtr);
        return status;
    }

    return STATUS_SUCCESS;
}


//
// Routine Description:
//
//  LPSPIEvtSpbTargetDisconnect is called by the framework when a peripheral 
//  driver closes a target. 
//  The routine closes the GPIO target, if it was opened. 
//
// Arguments:
//
//  Controller - The WdfDevice object the represent the LPSPI instance of
//      the LPSPI controller.
//
//  SpbTarget - The SPB target object
//
// Return Value:
//
//  NTSTATUS
//
VOID
LPSPIEvtSpbTargetDisconnect (
    WDFDEVICE /*Controller*/,
    SPBTARGET SpbTarget
    )
{
    PAGED_CODE();

    LPSPI_TARGET_CONTEXT* trgCtxPtr = LPSPITargetGetContext(SpbTarget);

    LPSPIDeviceCloseGpioTarget(trgCtxPtr);
}


//
// Routine Description:
//
//  LPSPIEvtSpbControllerLock is called by the framework to lock bus access
//  to a specific target.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  SpbTarget - The SPB target object
//
//  SpbRequestLock - The Lock SPB request
//
// Return Value:
//
//  Through SpbRequest completion.
//
_Use_decl_annotations_
VOID
LPSPIEvtSpbControllerLock (
    WDFDEVICE /*WdfDevice*/,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest
    )
{
    LPSPI_TARGET_CONTEXT* trgCtxPtr = LPSPITargetGetContext(SpbTarget);
    LPSPI_DEVICE_EXTENSION* devExtPtr = trgCtxPtr->DevExtPtr;

    LPSPI_ASSERT(
        devExtPtr->IfrLogHandle,
        !devExtPtr->IsControllerLocked &&
            (devExtPtr->LockUnlockRequest == NULL)
        );

    devExtPtr->LockUnlockRequest = SpbRequest;
    devExtPtr->IsControllerLocked = TRUE;

    NTSTATUS status = LPSPIDeviceAssertCS(trgCtxPtr, FALSE);
    LPSPI_ASSERT(
        devExtPtr->IfrLogHandle,
        NT_SUCCESS(status)
        );
}


//
// Routine Description:
//
//  LPSPIEvtSpbControllerLock is called by the framework to lock bus access
//  to a specific target.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  SpbTarget - The SPB target object
//
//  SpbRequestLock - The Unlock SPB request
//
// Return Value:
//
//  Through SpbRequest completion.
//
_Use_decl_annotations_
VOID
LPSPIEvtSpbControllerUnlock (
    WDFDEVICE /*WdfDevice*/,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest
    )
{
    LPSPI_TARGET_CONTEXT* trgCtxPtr = LPSPITargetGetContext(SpbTarget);
    LPSPI_DEVICE_EXTENSION* devExtPtr = trgCtxPtr->DevExtPtr;

    LPSPIHwUnselectTarget(trgCtxPtr);

    devExtPtr->LockUnlockRequest = SpbRequest;
    devExtPtr->IsControllerLocked = FALSE;

    NTSTATUS status = LPSPIDeviceNegateCS(trgCtxPtr, FALSE);
    LPSPI_ASSERT(
        devExtPtr->IfrLogHandle,
        NT_SUCCESS(status)
        );
}


//
// Routine Description:
//
//  LPSPIEvtSpbIoRead is called by the framework to perform a read
//  transfer from a target.
//  The routine retrieves the request parameters, prepares the HW and
//  enables the RX interrupts.
//  Handling of the read request continues in ISR/DPC.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  SpbTarget - The SPB target object
//
//  SpbRequest - The SPB READ request.
//
//  Length - Number of bytes to read.
//
// Return Value:
//
//  Through SpbRequest completion.
//
_Use_decl_annotations_
VOID
LPSPIEvtSpbIoRead (
    WDFDEVICE /*WdfDevice*/,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest,
    size_t Length
    )
{
    LPSPI_TARGET_CONTEXT* trgCtxPtr = LPSPITargetGetContext(SpbTarget);
    const LPSPI_DEVICE_EXTENSION* devExtPtr = trgCtxPtr->DevExtPtr;

    LPSPI_LOG_INFORMATION(
        devExtPtr->IfrLogHandle,
        "READ request: target %p, request %p, length %Iu",
        trgCtxPtr,
        SpbRequest,
        Length
        );

    LPSPIpSpbInitiateIo(
        trgCtxPtr,
        SpbRequest,
        LPSPI_REQUEST_TYPE::READ
        );
}


//
// Routine Description:
//
//  LPSPIEvtSpbIoWrite is called by the framework to perform a write
//  transfer to a target.
//  The routine retrieves the request parameters, prepares the HW, 
//  write the first bytes to TX FIFO, and enables the TX interrupts.
//  Handling of the write request continues in ISR/DPC.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  SpbTarget - The SPB target object
//
//  SpbRequestLock - The SPB WRITE request.
//
//  Length - Number of bytes to write.
//
// Return Value:
//
//  Through SpbRequest completion.
//
_Use_decl_annotations_
VOID
LPSPIEvtSpbIoWrite (
    WDFDEVICE /*WdfDevice*/,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest,
    size_t Length
    )
{
    LPSPI_TARGET_CONTEXT* trgCtxPtr = LPSPITargetGetContext(SpbTarget);
    const LPSPI_DEVICE_EXTENSION* devExtPtr = trgCtxPtr->DevExtPtr;

    LPSPI_LOG_INFORMATION(
        devExtPtr->IfrLogHandle,
        "WRITE request: target %p, request %p, length %Iu",
        trgCtxPtr,
        SpbRequest,
        Length
        );

    LPSPIpSpbInitiateIo(
        trgCtxPtr, 
        SpbRequest, 
        LPSPI_REQUEST_TYPE::WRITE
        );
}


//
// Routine Description:
//
//  LPSPIEvtSpbIoSequence is called by the framework to perform a sequence of
//  transfers to/from a target.
//  The routine retrieves the request parameters, prepares the HW and
//  enables the RX interrupts.
//  Handling of the read/write transfers continues in ISR/DPC.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  SpbTarget - The SPB target object
//
//  SpbRequest - The SPB SEQUENCE request.
//
//  TransferCount - Number of transfers.
//
// Return Value:
//
//  Through SpbRequest completion.
//
_Use_decl_annotations_
VOID
LPSPIEvtSpbIoSequence (
    WDFDEVICE /*WdfDevice*/,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest,
    ULONG TransferCount
    )
{
    LPSPI_TARGET_CONTEXT* trgCtxPtr = LPSPITargetGetContext(SpbTarget);
    const LPSPI_DEVICE_EXTENSION* devExtPtr = trgCtxPtr->DevExtPtr;

    LPSPI_LOG_INFORMATION(
        devExtPtr->IfrLogHandle,
        "Sequence request: target %p, request %p, transfers %lu",
        trgCtxPtr,
        SpbRequest,
        TransferCount
        );

    LPSPIpSpbInitiateIo(
        trgCtxPtr,
        SpbRequest,
        LPSPI_REQUEST_TYPE::SEQUENCE
        );
}


//
// Routine Description:
//
//  LPSPIEvtSpbIoOther is called by the framework to handle custom
//  IO control requests. 
//  SPB uses this path for handling IOCTL_SPB_FULL_DUPLEX.
//  The routine retrieves the request parameters, prepares the HW and
//  enables interrupts.
//  Handling of the read request continues in ISR/DPC.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  SpbTarget - The SPB target object
//
//  SpbRequest - The SPB SEQUENCE request.
//
//  OutputBufferLength - Output buffer length.
//
//  InputBufferLength - Input buffer length.
//
//  IoControlCode - IO control code
//
// Return Value:
//
//  Through SpbRequest completion.
//
_Use_decl_annotations_
VOID 
LPSPIEvtSpbIoOther (
    WDFDEVICE /*WdfDevice*/,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest,
    size_t /*OutputBufferLength*/,
    size_t /*InputBufferLength*/,
    ULONG IoControlCode
    )
{
    LPSPI_TARGET_CONTEXT* trgCtxPtr = LPSPITargetGetContext(SpbTarget);
    const LPSPI_DEVICE_EXTENSION* devExtPtr = trgCtxPtr->DevExtPtr;

    LPSPI_ASSERT(
        devExtPtr->IfrLogHandle,
        IoControlCode == IOCTL_SPB_FULL_DUPLEX
        );

    LPSPI_LOG_INFORMATION(
        devExtPtr->IfrLogHandle,
        "FULL_DUPLEX request: target %p, request %p",
        trgCtxPtr,
        SpbRequest
        );

    LPSPIpSpbInitiateIo(
        trgCtxPtr,
        SpbRequest,
        LPSPI_REQUEST_TYPE::FULL_DUPLEX
        );
}


//
// Routine Description:
//
//  LPSPISpbPrepareNextTransfer is called by either LPSPIpSpbPrepareRequest or
//  by LPSPIEvtInterruptDpc to prepare the next transfers(s).
//  If the request has multiple transfers, the routine tries to prepare as many
//  as it can. 
//
// Arguments:
//
//  RequestPtr - The SPB request context.
//
// Return Value:
//
//  NTSTATUS: STATUS_SUCCESS, or STATUS_NO_MORE_FILES if no
//      transfers were prepared.
//
_Use_decl_annotations_
NTSTATUS
LPSPISpbPrepareNextTransfer (
    LPSPI_SPB_REQUEST* RequestPtr
    )
{
    const LPSPI_DEVICE_EXTENSION* devExtPtr = 
        RequestPtr->SpbTargetPtr->DevExtPtr;
    const LPSPI_TARGET_SETTINGS* trgSettingsPtr = 
        &RequestPtr->SpbTargetPtr->Settings;

    //
    // Calculate how many transfers we can prepare
    //
    ULONG maxTransferToPrepare = MAX_PREPARED_TRANSFERS_COUNT - 
        LPSPISpbGetReadyTransferCount(RequestPtr);
    ULONG transfersToPrepare = 
        RequestPtr->TransferCount - RequestPtr->NextTransferIndex;
    transfersToPrepare = min(transfersToPrepare, maxTransferToPrepare);

    //
    // Prepare the next 'transfersToPrepare' transfers
    //
    ULONG preparedTransfers = 0;
    ULONG transferIn = RequestPtr->TransferIn;
    for (ULONG xferIndex = 0;
         xferIndex < transfersToPrepare;
         ++xferIndex) {
        
        LPSPI_SPB_TRANSFER* reqXferPtr =  &RequestPtr->Transfers[transferIn];

        //
        // Initialize transfer parameters
        //
        RtlZeroMemory(reqXferPtr, sizeof(*reqXferPtr));

        PMDL baseMdlPtr;
        SPB_TRANSFER_DESCRIPTOR_INIT(&reqXferPtr->SpbTransferDescriptor);
        SpbRequestGetTransferParameters(
            RequestPtr->SpbRequest,
            RequestPtr->NextTransferIndex + xferIndex,
            &reqXferPtr->SpbTransferDescriptor,
            &baseMdlPtr
            );

        if ((reqXferPtr->SpbTransferDescriptor.TransferLength %
            trgSettingsPtr->BufferStride) != 0) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "Invalid transfer length (%Iu). "
                "Transfer length should we a integer product of %d.",
                reqXferPtr->SpbTransferDescriptor.TransferLength,
                trgSettingsPtr->BufferStride
                );
            return STATUS_INVALID_PARAMETER;
        }

        if (reqXferPtr->SpbTransferDescriptor.DelayInUs != 0) {
            //
            // The LPSPI supports delay between bursts.
            // Since we use a single burst, delay is not supported.
            //
            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "Transfer delay is not supported!"
                );
            return STATUS_NOT_SUPPORTED;
        }

        reqXferPtr->CurrentMdlPtr = baseMdlPtr;
        reqXferPtr->BufferStride = trgSettingsPtr->BufferStride;
        reqXferPtr->AssociatedRequestPtr = RequestPtr;
        reqXferPtr->BurstLength = min(
            reqXferPtr->SpbTransferDescriptor.TransferLength,
            LPSPI_MAX_BURST_LENGTH_BYTES
            );
        reqXferPtr->BytesLeftInBurst = reqXferPtr->BurstLength;
		reqXferPtr->BurstWords = LPSPISpbWordsLeftInBurst(reqXferPtr);
        reqXferPtr->NumberOfBursts = (reqXferPtr->SpbTransferDescriptor.TransferLength +
            LPSPI_MAX_BURST_LENGTH_BYTES - 1) / LPSPI_MAX_BURST_LENGTH_BYTES;
        reqXferPtr->CurrentBurst = 0;
        LPSPI_LOG_INFORMATION(
            devExtPtr->IfrLogHandle,
            "Preparing transfer %p: target %p, request %p ,"
            "type %!REQUESTTYPE!, direction %s, "
            "length %Iu, delay %lu uSec",
            reqXferPtr,
            RequestPtr->SpbTargetPtr,
            RequestPtr,
            RequestPtr->Type,
            DIR2STR(reqXferPtr->SpbTransferDescriptor.Direction),
            reqXferPtr->SpbTransferDescriptor.TransferLength,
            reqXferPtr->SpbTransferDescriptor.DelayInUs
            );

        //
        // Map MDL(s)
        //
        for (PMDL mdlPtr = baseMdlPtr;
             mdlPtr != nullptr;
             mdlPtr = mdlPtr->Next) {

            ULONG pagePriority = NormalPagePriority | MdlMappingNoExecute;
            if (LPSPISpbIsWriteTransfer(reqXferPtr)) {

                pagePriority |= MdlMappingNoWrite;
            }

            VOID const* mdlVaPtr = MmGetSystemAddressForMdlSafe(
                mdlPtr, 
                pagePriority
                );
            if (mdlVaPtr == nullptr) {

                LPSPI_LOG_ERROR(
                    devExtPtr->IfrLogHandle,
                    "MmGetSystemAddressForMdlSafe failed. "
                    "request %p, MDL %p",
                    RequestPtr->SpbRequest,
                    mdlPtr
                    );
                return STATUS_INSUFFICIENT_RESOURCES;
            }

        } // More MDLs

        transferIn = (transferIn + 1) % MAX_PREPARED_TRANSFERS_COUNT;
        ++preparedTransfers;

        InterlockedIncrement(&RequestPtr->ReadyTransferCount);

    } // More transfers to initialize

    RequestPtr->TransferIn = transferIn;
    RequestPtr->NextTransferIndex += preparedTransfers;

    return preparedTransfers != 0 ? STATUS_SUCCESS : STATUS_NO_MORE_FILES;
}


//
// Routine Description:
//
//  LPSPISpbStartNextTransfer is called to start the next IO transfer.
//  The routine prepares the HW and starts the transfer.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  RequestPtr - The SPB request context.
//
// Return Value:
//
//  NTSTATUS: STATUS_SUCCESS, or STATUS_NO_MORE_FILES if there are no
//      more prepared transfers.
//
_Use_decl_annotations_
NTSTATUS
LPSPISpbStartNextTransfer (
    LPSPI_SPB_REQUEST* RequestPtr
    )
{
    if (LPSPISpbGetReadyTransferCount(RequestPtr) == 0) {

        return STATUS_NO_MORE_FILES;
    }

    LPSPI_TARGET_CONTEXT* spbTargetPtr = RequestPtr->SpbTargetPtr;
    LPSPI_DEVICE_EXTENSION* devExtPtr = spbTargetPtr->DevExtPtr;

    LPSPI_SPB_TRANSFER* activeXfer1Ptr;
    LPSPI_SPB_TRANSFER* activeXfer2Ptr;
    LPSPISpbGetActiveTransfers(RequestPtr, &activeXfer1Ptr, &activeXfer2Ptr);

    LPSPI_ASSERT(
        devExtPtr->IfrLogHandle,
        activeXfer1Ptr->CurrentMdlPtr != nullptr
        );

    LPSPI_LOG_TRACE(
        devExtPtr->IfrLogHandle,
        "Starting transfer %p (1/%d): target %p, request %p, "
        "type %!REQUESTTYPE!, direction %s, "
        "length %Iu, delay %lu uSec",
        activeXfer1Ptr,
        activeXfer2Ptr == nullptr ? 1 : 2,
        spbTargetPtr,
        RequestPtr,
        RequestPtr->Type,
        DIR2STR(activeXfer1Ptr->SpbTransferDescriptor.Direction),
        activeXfer1Ptr->SpbTransferDescriptor.TransferLength,
        activeXfer1Ptr->SpbTransferDescriptor.DelayInUs
        );
    if (activeXfer2Ptr != nullptr) {

        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle,
            activeXfer2Ptr->CurrentMdlPtr != nullptr
            );

        LPSPI_LOG_TRACE(
            devExtPtr->IfrLogHandle,
            "Starting transfer %p (1/2): target %p, request %p, "
            "type %!REQUESTTYPE!, direction %s, "
            "length %Iu, delay %lu uSec",
            activeXfer2Ptr,
            spbTargetPtr,
            RequestPtr,
            RequestPtr->Type,
            DIR2STR(activeXfer2Ptr->SpbTransferDescriptor.Direction),
            activeXfer2Ptr->SpbTransferDescriptor.TransferLength,
            activeXfer2Ptr->SpbTransferDescriptor.DelayInUs
            );
    }

    LPSPIHwClearFIFOs(devExtPtr);

    activeXfer1Ptr->IsStartBurst = TRUE;

    if (LPSPISpbIsWriteTransfer(activeXfer1Ptr)) {
        //
        // On a write transfer, write the first chunk.
        //
        (void)LPSPIHwWriteTxFIFO(devExtPtr, activeXfer1Ptr, TRUE /* Initial setup */ );

    } else {
        //
        // On a read transfer, write 0s to clock in RX data.
        //
        LPSPIHwWriteZerosTxFIFO(devExtPtr, activeXfer1Ptr);

    }

    ULONG interruptMaskForTransfer = 0;

    LPSPIHwConfigureTransfer(
        devExtPtr,
        activeXfer1Ptr,
        TRUE, // Initial setup
        &interruptMaskForTransfer
    );

    if (activeXfer2Ptr != nullptr) {

        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle,
            LPSPISpbIsWriteTransfer(activeXfer1Ptr)
        );
        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle,
            !LPSPISpbIsWriteTransfer(activeXfer2Ptr)
        );
        LPSPIHwConfigureTransfer(
            devExtPtr,
            activeXfer2Ptr,
            FALSE, // Add to current setup
            &interruptMaskForTransfer
        );
    }

    //
    // Allow ISR to take over
    //
    (void)LPSPIHwInterruptControl(
        devExtPtr,
        0x0,   // Disable mask
        interruptMaskForTransfer   // Enable mask
    );

#if (defined(DBG) || defined(DEBUG))
    LPSPIHwDumpRegisters(devExtPtr);
#endif

    return STATUS_SUCCESS;
}


//
// Routine Description:
//
//  LPSPISpbStartTransferSafe starts the next transfer 
//  synchronized with the cancel routine.
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
LPSPISpbStartTransferSafe (
    LPSPI_SPB_REQUEST* RequestPtr
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = RequestPtr->SpbTargetPtr->DevExtPtr;

    KLOCK_QUEUE_HANDLE lockHandle;
    KeAcquireInStackQueuedSpinLock(&devExtPtr->DeviceLock, &lockHandle);

    NTSTATUS status = LPSPISpbStartNextTransfer(RequestPtr);

    KeReleaseInStackQueuedSpinLock(&lockHandle);

    return status;
}


//
// Routine Description:
//
//  LPSPISpbGetActiveTransfers returns the active transfers of a give request.
//  Only FULL_DUPLEX request has 2 active requests, all other request types have 1.
//
// Arguments:
//
//  RequestPtr - The Request context.
//
//  Transfer1PPtr - The 1st active transfer.
//
//  Transfer2PPtr - The 2nd active transfer.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPISpbGetActiveTransfers(
    LPSPI_SPB_REQUEST* RequestPtr,
    LPSPI_SPB_TRANSFER** Transfer1PPtr,
    LPSPI_SPB_TRANSFER** Transfer2PPtr
    )
{
    *Transfer1PPtr = &RequestPtr->Transfers[RequestPtr->TransferOut];
    *Transfer2PPtr = nullptr;

    switch (RequestPtr->Type) {
    case LPSPI_REQUEST_TYPE::READ:
    case LPSPI_REQUEST_TYPE::WRITE:
    case LPSPI_REQUEST_TYPE::SEQUENCE:
        break;

    case LPSPI_REQUEST_TYPE::FULL_DUPLEX:
    {
        ULONG nextXferIndex = (RequestPtr->TransferOut + 1) %
            MAX_PREPARED_TRANSFERS_COUNT;
        *Transfer2PPtr = &RequestPtr->Transfers[nextXferIndex];
        break;

    } // LPSPI_REQUEST_TYPE::FULL_DUPLEX

    default:
        NT_ASSERT(FALSE);
    } // switch (...)
}


//
// Routine Description:
//
//  LPSPISpbCompleteTransfer is called when a SEQUENCE transfer is complete.
//  The routines attempts to start the next transfer if available.
//
// Arguments:
//
//  TransferPtr - The completed transfer context.
//      the LPSPI controller.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPISpbCompleteSequenceTransfer (
    LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    LPSPI_SPB_REQUEST* requestPtr = TransferPtr->AssociatedRequestPtr;
    LPSPI_DEVICE_EXTENSION* devExtPtr = requestPtr->SpbTargetPtr->DevExtPtr;

    LPSPI_ASSERT(
        devExtPtr->IfrLogHandle,
        requestPtr->Type == LPSPI_REQUEST_TYPE::SEQUENCE
        );
    LPSPI_ASSERT(
        devExtPtr->IfrLogHandle,
        TransferPtr == &requestPtr->Transfers[requestPtr->TransferOut]
        );

    LPSPI_LOG_TRACE(
        devExtPtr->IfrLogHandle,
        "Sequence transfer %p completed, %d left. Target %p, request %p, "
        "type %!REQUESTTYPE!, direction %s, length %Iu",
        TransferPtr,
        requestPtr->TransfersLeft,
        requestPtr->SpbTargetPtr,
        requestPtr,
        requestPtr->Type,
        DIR2STR(TransferPtr->SpbTransferDescriptor.Direction),
        TransferPtr->SpbTransferDescriptor.TransferLength
        );

    InterlockedDecrement(
        reinterpret_cast<volatile LONG*>(&requestPtr->ReadyTransferCount)
        );
    requestPtr->TransfersLeft -= 1;

    requestPtr->TransferOut =
        (requestPtr->TransferOut + 1) % MAX_PREPARED_TRANSFERS_COUNT;

    //
    // If request has more transfers, start the next one...
    //
    if (requestPtr->TransfersLeft != 0) {

        if (LPSPISpbStartNextTransfer(requestPtr) == STATUS_NO_MORE_FILES) {
            //
            // Mark that transfer is idle due to lack of prepared transfers, since
            // transfers can only be prepared at IRQL <= DISPATCH_LEVEL.
            // When a request is marked as 'idle', DPC knows it needs 
            // to start the next transfer after preparing it.
            //
            InterlockedExchange(&requestPtr->IsIdle, 1);
        }
    }
}

//
// Routine Description:
//
//  LPSPIDeviceCompleteTrasnferRequest is called to complete the
//  current transfer request.
//
// Arguments:
//
//  RequestPtr - The request to complete
//
//  Status - Request status
//
//  Information - Request information (number of bytes transferred).
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPISpbCompleteTransferRequest (
    LPSPI_SPB_REQUEST* RequestPtr,
    NTSTATUS Status,
    ULONG_PTR Information
    )
{
    SPBREQUEST spbRequest = static_cast<SPBREQUEST>(
        InterlockedExchangePointer(
            reinterpret_cast<PVOID volatile *>(&RequestPtr->SpbRequest),
            nullptr
        ));

    if (spbRequest != NULL) {

        RequestPtr->Type = LPSPI_REQUEST_TYPE::INVALID;
        RequestPtr->State = LPSPI_REQUEST_STATE::INACTIVE;

        WdfRequestSetInformation(spbRequest, Information);
        SpbRequestComplete(spbRequest, Status);
    }
}


//
// Routine Description:
//
//  LPSPISpbAbortAllTransfers is called to abort all active transfers.
//  The routine resets the block.
//
// Arguments:
//
//  RequestPtr - The request who's transfers should be aborted.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPISpbAbortAllTransfers (
    _In_ LPSPI_SPB_REQUEST* RequestPtr
    )
{
    LPSPI_TARGET_CONTEXT* trgCtxPtr = RequestPtr->SpbTargetPtr;
    LPSPI_DEVICE_EXTENSION* devExtPtr = trgCtxPtr->DevExtPtr;

    WdfInterruptAcquireLock(devExtPtr->WdfSpiInterrupt);

    LPSPIHwUnselectTarget(trgCtxPtr);
    
    RequestPtr->ReadyTransferCount = 0;

    WdfInterruptReleaseLock(devExtPtr->WdfSpiInterrupt);
}


// 
// LPSPIspb private methods.
// ------------------------
//


//
// Routine Description:
//
//  LPSPIpSpbInitiateIo is called by SPB IO callback function to prepare 
//  a SPB request for IO, and initiate the transfer(s).
//  If the process fails the routine completes the request.
//
// Arguments:
//
//  TrgCtxPtr - The SPB target context.
//
//  SpbRequest - The SPB request.
//
//  RequestType - The request type
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIpSpbInitiateIo (
    LPSPI_TARGET_CONTEXT* TrgCtxPtr,
    SPBREQUEST SpbRequest,
    LPSPI_REQUEST_TYPE RequestType
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;

    NTSTATUS status = LPSPIpSpbPrepareRequest(TrgCtxPtr, SpbRequest, RequestType);
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "LPSPIpSpbPrepareRequest failed. "
            "request %p, status = %!STATUS!",
            SpbRequest,
            status
            );
        LPSPISpbCompleteTransferRequest(&devExtPtr->CurrentRequest, status, 0);
        return;
    }

    if (devExtPtr->IsControllerLocked) {
        //
        // Controller is locked, CS is already asserted,
        // just start the transfer...
        //
        status = LPSPIDeviceEnableRequestCancellation(&devExtPtr->CurrentRequest);
        if (!NT_SUCCESS(status)) {

            return;
        }

        status = LPSPISpbStartTransferSafe(&devExtPtr->CurrentRequest);
        LPSPI_ASSERT(
            devExtPtr->IfrLogHandle,
            NT_SUCCESS(status)
            );
        return;
    }

    status = LPSPIDeviceAssertCS(TrgCtxPtr, FALSE);
    if (!NT_SUCCESS(status)) {

        LPSPI_LOG_ERROR(
            devExtPtr->IfrLogHandle,
            "LPSPISpbStartNextTransfer failed. "
            "request %p, status = %!STATUS!",
            SpbRequest,
            status
            );
        LPSPISpbCompleteTransferRequest(&devExtPtr->CurrentRequest, status, 0);
        return;
    }
}


//
// Routine Description:
//
//  LPSPIpSpbPrepareRequest is called by LPSPIpSpbInitiateIo to prepare 
//  a SPB request for IO.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
//  TrgCtxPtr - The SPB target context.
//
//  SpbRequest - The SPB request.
//
//  RequestType - The request type
//
// Return Value:
//
//  NTSTATUS
//
_Use_decl_annotations_
NTSTATUS
LPSPIpSpbPrepareRequest (
    LPSPI_TARGET_CONTEXT* TrgCtxPtr,
    SPBREQUEST SpbRequest,
    LPSPI_REQUEST_TYPE RequestType
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;
    LPSPI_SPB_REQUEST* requestPtr = &devExtPtr->CurrentRequest;
    NTSTATUS status;

    //
    // Initialize request parameters
    //
    {
        KLOCK_QUEUE_HANDLE lockHandle;
        KeAcquireInStackQueuedSpinLock(&devExtPtr->DeviceLock, &lockHandle);

        RtlZeroMemory(requestPtr, sizeof(LPSPI_SPB_REQUEST));
        requestPtr->SpbRequest = SpbRequest;
        requestPtr->SpbTargetPtr = TrgCtxPtr;
        requestPtr->Type = RequestType;
        requestPtr->State = LPSPI_REQUEST_STATE::INACTIVE;

        SPB_REQUEST_PARAMETERS spbRequestParameters;
        SPB_REQUEST_PARAMETERS_INIT(&spbRequestParameters);
        SpbRequestGetParameters(SpbRequest, &spbRequestParameters);

        requestPtr->TransferCount = spbRequestParameters.SequenceTransferCount;
        requestPtr->TransfersLeft = requestPtr->TransferCount;
        if (requestPtr->TransferCount == 0) {

            LPSPI_LOG_ERROR(
                devExtPtr->IfrLogHandle,
                "Invalid transfer count: 0"
                );
            KeReleaseInStackQueuedSpinLock(&lockHandle);
            return STATUS_INVALID_PARAMETER;
        }

        switch (RequestType) {
        case LPSPI_REQUEST_TYPE::READ:
        case LPSPI_REQUEST_TYPE::WRITE:
        case LPSPI_REQUEST_TYPE::SEQUENCE:
            break;

        case LPSPI_REQUEST_TYPE::FULL_DUPLEX:
            if (requestPtr->TransferCount != 2) {

                LPSPI_LOG_ERROR(
                    devExtPtr->IfrLogHandle,
                    "Invalid transfer count (%lu) for a FULL_DUPLEX request!"
                    " 2 transfers are required.",
                    requestPtr->TransferCount
                    );
                KeReleaseInStackQueuedSpinLock(&lockHandle);
                return STATUS_INVALID_PARAMETER;
            }
            break;

        default:
            LPSPI_ASSERT(devExtPtr->IfrLogHandle, FALSE);
            KeReleaseInStackQueuedSpinLock(&lockHandle);
            return STATUS_INVALID_PARAMETER;
        } // switch (...)

        KeReleaseInStackQueuedSpinLock(&lockHandle);

    } // Get request parameters

    status = LPSPISpbPrepareNextTransfer(requestPtr);
    if (!NT_SUCCESS(status)) {

        return status;
    }

    return STATUS_SUCCESS;
}

#undef _LPSPI_SPB_CPP_