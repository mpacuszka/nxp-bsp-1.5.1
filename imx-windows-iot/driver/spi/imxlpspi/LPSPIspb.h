// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//    LPSPIspb.h
//
// Abstract:
//
//    This module contains all the enums, types, and functions related to
//    an IMX LPSPI controller driver as a SPB controller driver.
//    This controller driver uses the SPB WDF class extension (SpbCx).
//
// Environment:
//
//    kernel-mode only
//

#ifndef _LPSPI_SPB_H_
#define _LPSPI_SPB_H_

WDF_EXTERN_C_START


//
// SPI Connection Descriptor, defined in ACPI 5.0 spec table 6-192
//
#include <pshpack1.h>
typedef struct _PNP_SPI_SERIAL_BUS_DESCRIPTOR {
    PNP_SERIAL_BUS_DESCRIPTOR SerialBusDescriptor;
    // 
    // SPI extension
    //
    ULONG ConnectionSpeed;
    UCHAR DataBitLength;
    UCHAR Phase;
    UCHAR Polarity;
    USHORT DeviceSelection;
    //
    // Vendor specific extension:
    // - followed by optional Vendor Data
    // - followed by resource name string
    //
} PNP_SPI_SERIAL_BUS_DESCRIPTOR;
#include <poppack.h> // pshpack1.h


//
// See section 6.4.3.8.2 of the ACPI 5.0 specification
//
enum PNP_SERIAL_BUS_TYPE {
    PNP_SERIAL_BUS_TYPE_I2C = 0x1,
    PNP_SERIAL_BUS_TYPE_SPI = 0x2,
    PNP_SERIAL_BUS_TYPE_UART = 0x3,
};


//
// Serial general configuration flags
//
enum PNP_SERIAL_GENERAL_FLAGS : ULONG {
    PNP_SERIAL_GENERAL_FLAGS_SLV_BIT = 0x1, // 0 = ControllerInitiated, 1 = DeviceInitiated
};


//
// SPI configuration flags
//
enum PNP_SPI_FLAGS : ULONG {
    PNP_SPI_WIREMODE_BIT = 0x1,		// 0 = FourWireMode, 1 = ThreeWireMode
    PNP_SPI_DEVICEPOLARITY_BIT = 0x2,   // 0 = ActiveLow, 1 = ActiveHigh
};


//
// LPSPI_TARGET_SETTINGS
//  Current target settings.
//
typedef struct _LPSPI_TARGET_SETTINGS
{
    //
    // From PNP_SPI_SERIAL_BUS_DESCRIPTOR
    //
    ULONG ConnectionSpeed;
    ULONG DataBitLength;
    ULONG Phase;
    ULONG Polarity;
    USHORT DeviceSelection;

    //
    // Private settings
    //
    ULONG BufferStride;
    ULONG CsActiveValue;

    //
    // Reflection of the required settings in HW.
    //
    LPSPI_CR CtrlReg;
    LPSPI_CFGR1 Config1Reg;
    LPSPI_CCR ClockConfigReg;
    LPSPI_TCR TransmitCmdReg;

} LPSPI_TARGET_SETTINGS;


//
// LPSPI_TARGET_CONTEXT.
//  Contains all The LPSPI SPB target runtime parameters.
//  It is associated with the SPBTARGET object.
//
typedef struct _LPSPI_TARGET_CONTEXT
{
    //
    // Handle to the SPB target.
    //
    SPBTARGET SpbTarget;

    //
    // The device extension
    //
    LPSPI_DEVICE_EXTENSION* DevExtPtr;

    //
    // Target settings.
    //
    LPSPI_TARGET_SETTINGS Settings;

} LPSPI_TARGET_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(LPSPI_TARGET_CONTEXT, LPSPITargetGetContext);


//
// LPSPI_SPB_TRANSFER.
//  Contains all the information about a single input/output 
//  SPB request transfer;
//
typedef struct _LPSPI_SPB_TRANSFER {

    //
    // The associated SPB transfer descriptor
    //
    SPB_TRANSFER_DESCRIPTOR SpbTransferDescriptor;

    //
    // The request this transfer is associated with
    //
    LPSPI_SPB_REQUEST* AssociatedRequestPtr;

    //
    // Number of bytes transferred
    //
    size_t BytesTransferred;

    //
    // Which burst transfered
    //
    size_t NumberOfBursts;

    //
    // Which burst transfered
    //
    ULONG CurrentBurst;

    //
    // Number of bytes left in burst
    //
    size_t BytesLeftInBurst;

    //
    // Burst length (bytes)
    //
    size_t BurstLength;

    //
    // Burst length (ULONGs)
    //
    size_t BurstWords;

    //
    // If we need to start the burst
    //
    BOOLEAN IsStartBurst;

    //
    // Current MDL address
    //
    PMDL CurrentMdlPtr;

    //
    // Current MDL offset
    //
    size_t CurrentMdlOffset;

    //
    // Buffer stride in bytes
    //
    ULONG BufferStride;

} LPSPI_SPB_TRANSFER;


//
// Max number of pre-prepared transfers
//
enum : ULONG {
    MAX_PREPARED_TRANSFERS_COUNT = 4
};


//
// LPSPI_REQUEST_TYPE.
//
enum LPSPI_REQUEST_TYPE : ULONG {
    INVALID,
    WRITE,
    READ,
    SEQUENCE,
    FULL_DUPLEX
};


//
// LPSPI_REQUEST_TYPE. 
//
enum LPSPI_REQUEST_STATE : ULONG {
    INACTIVE,
    CANCELABLE,
    NOT_CANCELABLE,
    CANCELED
};


//
// LPSPI_REQUEST.
//  Contains all the parameters and run time variables need to
//  manage an SPB request.
//
typedef struct _LPSPI_SPB_REQUEST {

    //
    // The associated SPB request
    //
    SPBREQUEST SpbRequest;

    //
    // The associated target 
    //
    LPSPI_TARGET_CONTEXT* SpbTargetPtr;

    //
    // The request type
    //
    LPSPI_REQUEST_TYPE Type;

    //
    // Current request state
    //
    LPSPI_REQUEST_STATE State;

    //
    // Total request transfer count
    //
    ULONG TransferCount;

    //
    // Index of next transfer to prepare
    //
    ULONG NextTransferIndex;

    //
    // Number of transfer left to complete
    //
    ULONG TransfersLeft;

    //
    // Total bytes transferred
    //
    ULONG_PTR TotalBytesTransferred;

    //
    // If sequence request is idle and next
    // transfer needs to be started manually from DPC.
    //
    LONG IsIdle;

    //
    // Transfers file
    //
    LONG ReadyTransferCount;
    ULONG TransferIn;  // Use when preparing a transfer
    ULONG TransferOut; // Used when running the transfer 
    LPSPI_SPB_TRANSFER Transfers[MAX_PREPARED_TRANSFERS_COUNT];

} LPSPI_SPB_REQUEST;

//
// LPSPIcontroller SpbCX device event handlers
//
EVT_SPB_TARGET_CONNECT LPSPIEvtSpbTargetConnect;
EVT_SPB_TARGET_DISCONNECT LPSPIEvtSpbTargetDisconnect;
EVT_SPB_CONTROLLER_LOCK LPSPIEvtSpbControllerLock;
EVT_SPB_CONTROLLER_UNLOCK LPSPIEvtSpbControllerUnlock;
EVT_SPB_CONTROLLER_READ LPSPIEvtSpbIoRead;
EVT_SPB_CONTROLLER_WRITE LPSPIEvtSpbIoWrite;
EVT_SPB_CONTROLLER_SEQUENCE LPSPIEvtSpbIoSequence;
EVT_SPB_CONTROLLER_OTHER LPSPIEvtSpbIoOther;
EVT_WDF_IO_IN_CALLER_CONTEXT LPSPIEvtIoInCallerContext;


//
// Direction to string, to avoid the long
// SpbTransferDirection???Device
//
#define DIR2STR(_dir_)\
    (_dir_) == SpbTransferDirectionFromDevice ? "IN" : "OUT"


//
// Routine Description:
//
//  LPSPISpbGetBufferStride gets the buffer stride base on
//  the data bit length.
//
// Arguments:
//
//  DataBitLength - Data bit length.
//
// Return Value:
//
//  Buffer stride (bytes)
//
__forceinline
ULONG
LPSPISpbGetBufferStride (
    _In_ ULONG DataBitLength
    )
{
    NT_ASSERT(DataBitLength <= LPSPI_MAX_DATA_BIT_LENGTH);

    ULONG bytes = (DataBitLength + 7) / 8;

    NT_ASSERT(bytes <= sizeof(ULONG));

    switch (bytes) {
    case 3: 
        return 4;
    default:
        return bytes;
    }
}

//
// Routine Description:
//
//  LPSPISpbIsBurstStart returns TRUE is next data starts the burst.
//
// Arguments:
//
//  TransferPtr - The transfer descriptor
//
// Return Value:
//
//  TRUE if is next data starts the burst, otherwise FALSE.
//
__forceinline
BOOLEAN
LPSPISpbIsBurstStart (
    _In_ const LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    return TransferPtr->BytesLeftInBurst == TransferPtr->BurstLength;
}

//
// Routine Description:
//
//  LPSPISpbIsWriteTransfer returns TRUE if the transfer direction is 'to device'
//
// Arguments:
//
//  TransferPtr - The transfer context.
//
// Return Value:
//
//  TRUE if the transfer direction is 'to device', otherwise FALSE.
//
__forceinline
BOOLEAN
LPSPISpbIsWriteTransfer (
    _In_ const LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    return TransferPtr->SpbTransferDescriptor.Direction ==
        SpbTransferDirectionToDevice;
}

//
// Routine Description:
//
//  LPSPISpbIsTransferDone returns TRUE if all bursts are done.
//
// Arguments:
//
//  TransferPtr - The transfer descriptor
//
// Return Value:
//
//  TRUE if all bursts are done, otherwise FALSE.
//
__forceinline
BOOLEAN
LPSPISpbIsTransferDone (
    _In_ const LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    NT_ASSERT(
        TransferPtr->BytesTransferred <=
        TransferPtr->SpbTransferDescriptor.TransferLength
        );
    BOOLEAN isTransferDone = TransferPtr->BytesTransferred ==
        TransferPtr->SpbTransferDescriptor.TransferLength;

    //
    // For read transfers, transfer is done after all data has been
    // received.
    // For write transfers, transfer is done after all data bytes have been
    // sent and last burst has been started. 
    //
    if (LPSPISpbIsWriteTransfer(TransferPtr)) {

        isTransferDone &= !TransferPtr->IsStartBurst;
    }
    return isTransferDone;
}

//
// Routine Description:
//
//  LPSPISpbIsAllDataTransferred returns TRUE is all bytes have
//  been transferred.
//
// Arguments:
//
//  TransferPtr - The transfer descriptor
//
// Return Value:
//
//  TRUE if all transfer bytes have been transferred to/from SPI device, otherwise FALSE.
//
__forceinline
BOOLEAN
LPSPISpbIsAllDataTransferred (
    _In_ const LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    NT_ASSERT(
        TransferPtr->BytesTransferred <=
        TransferPtr->SpbTransferDescriptor.TransferLength
        );
    return TransferPtr->BytesTransferred ==
        TransferPtr->SpbTransferDescriptor.TransferLength;
}

//
// Routine Description:
//
//  LPSPISpbBytesLeftToTransfer returns the number of bytes
//  left to transfer.
//
// Arguments:
//
//  TransferPtr - The transfer descriptor
//
// Return Value:
//
//  The number of bytes left to transfer.
//
__forceinline
size_t
LPSPISpbBytesLeftToTransfer (
    _In_ const LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    NT_ASSERT(
        TransferPtr->BytesTransferred <=
        TransferPtr->SpbTransferDescriptor.TransferLength
        );
    return TransferPtr->SpbTransferDescriptor.TransferLength -
        TransferPtr->BytesTransferred;
}

//
// Routine Description:
//
//  LPSPISpbWordsLeftToTransfer returns the number of words
//  left to transfer.
//
// Arguments:
//
//  TransferPtr - The transfer descriptor
//
// Return Value:
//
//  The number of words left to transfer.
//
__forceinline
size_t
LPSPISpbWordsLeftToTransfer (
    _In_ const LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    return (LPSPISpbBytesLeftToTransfer(TransferPtr) + sizeof(ULONG) - 1) 
        / sizeof(ULONG);
}

//
// Routine Description:
//
//  LPSPISpbWordsLeftInBurst returns the number of words
//  left to transfer in the current burst
//
// Arguments:
//
//  TransferPtr - The transfer descriptor
//
// Return Value:
//
//  The number of words in burst.
//
__forceinline
size_t
LPSPISpbWordsLeftInBurst (
    _In_ const LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    return (TransferPtr->BytesLeftInBurst + sizeof(ULONG) - 1) / sizeof(ULONG);
}

//
// Routine Description:
//
//  LPSPISpbGetReadyTransferCount the number transfers that are ready to
//  go (prepared).
//
// Arguments:
//
//  TransferPtr - The transfer context.
//
// Return Value:
//
//  Number of ready transfers
//
__forceinline
ULONG
LPSPISpbGetReadyTransferCount (
    _In_ const LPSPI_SPB_REQUEST* RequestPtr
    )
{
    ULONG readyTranferCount = ReadULongAcquire(
        reinterpret_cast<ULONG const*>(&RequestPtr->ReadyTransferCount)
        );

    NT_ASSERT(readyTranferCount <= MAX_PREPARED_TRANSFERS_COUNT);

    return readyTranferCount;
}

//
// Routine Description:
//
//  LPSPISpbIsRequestDone returns TRUE if the SEQUENCE request is done.
//
// Arguments:
//
//  RequestPtr - The request context.
//
// Return Value:
//
//  TRUE if request is done, otherwise FALSE.
//
__forceinline
BOOLEAN
LPSPISpbIsRequestDone (
    _In_ const LPSPI_SPB_REQUEST* RequestPtr
    )
{
    NT_ASSERT(RequestPtr->Type == LPSPI_REQUEST_TYPE::SEQUENCE);

    return RequestPtr->TransfersLeft == 0;
}

//
// Routine Description:
//
//  LPSPISpbGetCsActiveValue returns the GPIO pin value for
//  active CS. 
//
// Arguments:
//
//  TargetContextPtr - The target context.
//
// Return Value:
//
//  The GPIO pin value for active CS.
//
__forceinline
ULONG
LPSPISpbGetCsActiveValue (
    _In_ const LPSPI_TARGET_CONTEXT* TargetContextPtr
    )
{
    return TargetContextPtr->Settings.CsActiveValue;
}

//
// Routine Description:
//
//  LPSPISpbGetCsNonActiveValue returns the GPIO pin value for
//  non active CS. 
//
// Arguments:
//
//  TargetContextPtr - The target context.
//
// Return Value:
//
//  The GPIO pin value for non active CS.
//
__forceinline
ULONG
LPSPISpbGetCsNonActiveValue(
    _In_ const LPSPI_TARGET_CONTEXT* TargetContextPtr
    )
{
    NT_ASSERT(TargetContextPtr->Settings.CsActiveValue <= 1);

    return 1 - TargetContextPtr->Settings.CsActiveValue;
}

VOID
LPSPISpbGetActiveTransfers(
    _In_ LPSPI_SPB_REQUEST* RequestPtr,
    _Out_ LPSPI_SPB_TRANSFER** Transfer1PPtr,
    _Out_ LPSPI_SPB_TRANSFER** Transfer2PPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
LPSPISpbPrepareNextTransfer (
    _In_ LPSPI_SPB_REQUEST* RequestPtr
    );

NTSTATUS
LPSPISpbStartNextTransfer (
    _In_ LPSPI_SPB_REQUEST* RequestPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
LPSPISpbStartTransferSafe (
    _In_ LPSPI_SPB_REQUEST* RequestPtr
    );

VOID
LPSPISpbCompleteSequenceTransfer (
    _In_ LPSPI_SPB_TRANSFER* TransferPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
VOID
LPSPISpbAbortAllTransfers(
    _In_ LPSPI_SPB_REQUEST* RequestPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
VOID
LPSPISpbCompleteTransferRequest(
    _In_ LPSPI_SPB_REQUEST* RequestPtr,
    _In_ NTSTATUS Status,
    _In_ ULONG_PTR Information
    );

//
// LPSPIdevice private methods
//
#ifdef _LPSPI_SPB_CPP_

    _IRQL_requires_max_(DISPATCH_LEVEL)
    static VOID
    LPSPIpSpbInitiateIo (
        _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr,
        _In_ SPBREQUEST SpbRequest,
        _In_ LPSPI_REQUEST_TYPE RequestType
        );

    _IRQL_requires_max_(DISPATCH_LEVEL)
    static NTSTATUS
    LPSPIpSpbPrepareRequest (
        _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr,
        _In_ SPBREQUEST SpbRequest,
        _In_ LPSPI_REQUEST_TYPE RequestType
        );

#endif // _LPSPI_SPB_CPP_

WDF_EXTERN_C_END

#endif // !_LPSPI_SPB_H_