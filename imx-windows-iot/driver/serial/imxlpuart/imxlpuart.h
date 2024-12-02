// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022 NXP
// Licensed under the MIT License.
//
//
// Module Name:
//
//   imxlpuart.h
//
// Abstract:
//
//   IMX8 LPUART Driver Declarations
//
#ifndef _IMX_LPUART_H_
#define _IMX_LPUART_H_

#define DEBUG_PRINT KdPrint

//
// Macros to be used for proper PAGED/NON-PAGED code placement
//

#define IMX_LPUART_NONPAGED_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    //__pragma(code_seg(.text))

#define IMX_LPUART_NONPAGED_SEGMENT_END \
    __pragma(code_seg(pop))

#define IMX_LPUART_PAGED_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    __pragma(code_seg("PAGE"))

#define IMX_LPUART_PAGED_SEGMENT_END \
    __pragma(code_seg(pop))

#define IMX_LPUART_INIT_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    __pragma(code_seg("INIT"))

#define IMX_LPUART_INIT_SEGMENT_END \
    __pragma(code_seg(pop))

#define IMX_LPUART_ASSERT_MAX_IRQL(Irql) NT_ASSERT(KeGetCurrentIrql() <= (Irql))

enum : ULONG { IMX_LPUART_POOL_TAG = 'LPUM' };

enum : ULONG { IMX_LPUART_STATE_ACTIVE = 0x100 };
enum IMX_LPUART_STATE : ULONG {
    STOPPED = 0x00,
    IDLE = 0x01,
    CANCELLED = 0x02,
    WAITING_FOR_INTERRUPT = (IMX_LPUART_STATE_ACTIVE | 0x03),
    WAITING_FOR_DPC = (IMX_LPUART_STATE_ACTIVE | 0x04),
    STOPPING = (IMX_LPUART_STATE_ACTIVE | 0x05),
    PAUSED = (IMX_LPUART_STATE_ACTIVE | 0x06),
    COMPLETE = 0x0A,
};


enum : ULONG { IMX_LPUART_RX_DMA_MIN_BUFFER_SIZE = 4096UL };

//
// Placement new and delete operators
//

_Notnull_
void *__cdecl operator new ( size_t, _In_ void *Ptr ) throw ();

void __cdecl operator delete ( void*, void*) throw ();

_Notnull_
void *__cdecl operator new[] ( size_t, _In_ void *Ptr ) throw ();

void __cdecl operator delete[] ( void*, void* ) throw ();

//
// Single-reader, single-writer circular buffer.
//
struct IMX_LPUART_RING_BUFFER {
    ULONG HeadIndex;                    // points to next available slot
    ULONG TailIndex;                    // points to least recently filled slot
    ULONG Size;                         // number of slots in buffer
    _Field_size_(Size) UCHAR *BufferPtr;

    FORCEINLINE IMX_LPUART_RING_BUFFER () :
        HeadIndex(0),
        TailIndex(0),
        Size(0),
        BufferPtr(nullptr)
    {}

    FORCEINLINE void SetBuffer (_In_reads_(Size) UCHAR *InBufferPtr, ULONG InBufferSize)
    {
        this->HeadIndex = 0;
        this->TailIndex = 0;
        this->Size = InBufferSize;
        this->BufferPtr = InBufferPtr;
    }

    FORCEINLINE bool IsEmpty () const
    {
        return this->HeadIndex == this->TailIndex;
    }

    FORCEINLINE ULONG Count () const
    {
        return Count(this->HeadIndex, this->TailIndex, this->Size);
    }

    FORCEINLINE static ULONG Count (ULONG HeadIndex, ULONG TailIndex, ULONG Size)
    {
        if (HeadIndex >= TailIndex)
        {
            return HeadIndex - TailIndex;
        }
        else
        {
            return Size + HeadIndex - TailIndex;
        }
    }

    FORCEINLINE ULONG Capacity () const
    {
        return this->Size - 1;
    }

    //
    // Empties the buffer and returns the number of bytes that were discarded.
    // Caller is responsible to synchronize.
    //
    FORCEINLINE ULONG Reset ()
    {
        const ULONG count = this->Count();
        this->HeadIndex = 0;
        this->TailIndex = 0;
        return count;
    }

    ULONG
    EnqueueBytes (
        _In_reads_(InputBufferSize) const UCHAR *InputBufferPtr,
        ULONG InputBufferSize
        );

    ULONG
    DequeueBytes (
        _Out_writes_to_(OutputBufferSize, return) UCHAR *OutputBufferPtr,
        ULONG OutputBufferSize
        );
};

struct IMX_LPUART_WDFKEY {
    WDFKEY Handle;

    FORCEINLINE IMX_LPUART_WDFKEY (WDFKEY _Handle = WDF_NO_HANDLE) : Handle(_Handle) {}

    _IRQL_requires_max_(DISPATCH_LEVEL)
    FORCEINLINE ~IMX_LPUART_WDFKEY ()
    {
        if (this->Handle == WDF_NO_HANDLE) return;
        WdfObjectDelete(this->Handle);
        this->Handle = WDF_NO_HANDLE;
    }
};

struct IMX_LPUART_INTERRUPT_CONTEXT;

//
// Default xon/xoff characters.
//
#define SERIAL_DEF_XON  0x11
#define SERIAL_DEF_XOFF 0x13

struct IMX_LPUART_DEVICE_CONTEXT {
    IMX_LPUART_REGISTERS *RegistersPtr;
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr;

    ULONG CurrentBaudRate;
    ULONG CurrentFlowReplace;

    BYTE *RxBufferStorage;
    BYTE *TxBufferStorage;
    WDFDEVICE WdfDevice;
    WDFINTERRUPT WdfInterrupt;

    //
    // Has iMXLPUARTEvtSerCx2ApplyConfig() been called at least once?
    //
    bool Initialized;

    //
    // Were RTS and CTS specified in the SerialLinesEnabled field of the
    // connection descriptor? Output CTS flow control (SERIAL_CTS_HANDSHAKE)
    // is only allowed if RTS and CTS were specified in connection
    // descriptor
    //
    bool RtsCtsLinesEnabled;

    //
    // Select UART as data terminal equipment (DTE mode)
    // or as data communication equipment (DCE mode)
    //
    // bool DTEModeSelected;

    bool Loop;

    struct {
        ULONG RxIntermediateBufferSize;
        ULONG RxDmaIntermediateBufferSize;
        ULONG TxIntermediateBufferSize;
        ULONG RxFifoThresholdUs;
        ULONG TxFifoThresholdUs;
        ULONG TxDpcThresholdUs;
        ULONG RtsTriggerLevel;
        ULONG ModuleClockFrequency;
        ULONG RxDmaMinTransactionLength;
        ULONG TxDmaMinTransactionLength;
    } Parameters;
};

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(
    IMX_LPUART_DEVICE_CONTEXT,
    iMXLPUARTGetDeviceContext);

struct IMX_LPUART_RX_DMA_TRANSACTION_CONTEXT;
struct IMX_LPUART_TX_DMA_TRANSACTION_CONTEXT;

struct IMX_LPUART_INTERRUPT_CONTEXT {
    IMX_LPUART_REGISTERS *RegistersPtr;

    ULONG Counter;
    ULONG64 TSC;

    ULONG EnabledInterruptsMask;
    bool TxXoff;

    IMX_LPUART_STATE RxState;
    IMX_LPUART_STATE RxDmaState;
    IMX_LPUART_STATE TxState;
    IMX_LPUART_STATE TxDrainState;
    IMX_LPUART_STATE TxPurgeState;
    IMX_LPUART_STATE TxDmaState;
    IMX_LPUART_STATE TxDmaDrainState;

    IMX_LPUART_RING_BUFFER RxBuffer;
    IMX_LPUART_RING_BUFFER TxBuffer;

    ULONG BAUDCopy;
    ULONG STATCopy;
    ULONG CTRLCopy;
    ULONG DATACopy;
    ULONG MATCHCopy;
    ULONG MODIRCopy;
    ULONG FIFOCopy;
    ULONG WATERCopy;

    //
    // Stores errors supplied to SERIAL_STATUS::Errors member
    //
    ULONG CommStatusErrors;

    //
    // Stores current wait mask (IOCTL_SERIAL_SET_WAIT_MASK)
    //
    ULONG WaitMask;

    //
    // Stores pending wait events
    //
    ULONG WaitEvents;

    //
    // Number of bytes purged from the TX intermediate FIFO
    //
    ULONG PurgedByteCount;

    //
    // If intermediate TX buffer falls below this threshold, call
    // SerCx2PioTransmitReady() to request more bytes.
    //
    ULONG TxDpcThreshold;

    WDFINTERRUPT WdfInterrupt;
    SERCX2PIORECEIVE SerCx2PioReceive;
    SERCX2PIOTRANSMIT SerCx2PioTransmit;

    //
    // DMA related objects
    //
    IMX_LPUART_RX_DMA_TRANSACTION_CONTEXT *RxDmaTransactionContextPtr;
    IMX_LPUART_TX_DMA_TRANSACTION_CONTEXT *TxDmaTransactionContextPtr;
    bool IsRxDmaStarted;

    // //
    // // Handflow 1:1 copy
    // //
    // ULONG CurrentFlowReplace;

    //
    // Xon threshold to send XON character
    //
    //ULONG XonThreshold;

    //
    // Xoff threshold to send XOFF character
    //
    //ULONG XoffThreshold;

    struct {
        struct {
            ULONG Size;
            BOOLEAN Enable;
        } FIFO;
    } RX, TX;
};

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(
    IMX_LPUART_INTERRUPT_CONTEXT,
    iMXLPUARTGetInterruptContext);

struct IMX_LPUART_RX_CONTEXT {
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr;
};

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(IMX_LPUART_RX_CONTEXT, iMXLPUARTGetRxContext);

struct IMX_LPUART_TX_CONTEXT {
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr;
};

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(IMX_LPUART_TX_CONTEXT, iMXLPUARTGetTxContext);

struct IMX_LPUART_DMA_TRANSACTION_CONTEXT {
    //
    // Transaction associated objects
    //
    WDFDEVICE WdfDevice;
    WDFDMAENABLER WdfDmaEnabler;
    PDMA_ADAPTER DmaAdapterPtr;
    WDFDMATRANSACTION WdfDmaTransaction;

    //
    // Configuration
    //
    ULONG DmaRequestLine;

    //
    // Container
    //
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr;

    //
    // Runtime
    //
    WDFSPINLOCK Lock;
    bool IsDeferredCancellation;
    WDFREQUEST WdfRequest;
    size_t TransferLength;
    size_t BytesTransferred;
};

struct IMX_LPUART_RX_DMA_TRANSACTION_CONTEXT
    : public IMX_LPUART_DMA_TRANSACTION_CONTEXT {
    //
    // SerCx2 associated handles
    //
    SERCX2CUSTOMRECEIVE SerCx2CustomRx;
    SERCX2CUSTOMRECEIVETRANSACTION SerCx2CustomRxTransaction;

    //
    // DMA RX buffer information
    //
    UCHAR *DmaBufferPtr;
    size_t DmaBufferSize;
    PMDL DmaBufferMdlPtr;
    size_t DmaBufferReadPos;
    size_t DmaBufferPendingBytes;
    size_t LastDmaCounter;

    //
    // Caller request buffer parameters
    //
    PMDL BufferMdlPtr;
    size_t BufferMdlOffset;

    //
    // Progress information
    //
    WDFTIMER WdfProgressTimer;
    ULONG CurrentFrameTimeUsec;
    ULONG CurrentBaudRate;
    LONG UnreportedBytes;
};

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(
    IMX_LPUART_RX_DMA_TRANSACTION_CONTEXT,
    iMXLPUARTGetRxDmaTransactionContext);

struct IMX_LPUART_RX_DMA_TIMER_CONTEXT {
    IMX_LPUART_RX_DMA_TRANSACTION_CONTEXT *RxDmaTransactionPtr;
};

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(
    IMX_LPUART_RX_DMA_TIMER_CONTEXT,
    iMXLPUARTGetRxDmaTimerContext);

struct IMX_LPUART_TX_DMA_TRANSACTION_CONTEXT
    : public IMX_LPUART_DMA_TRANSACTION_CONTEXT {
    //
    // SerCx2 associated handles
    //
    SERCX2CUSTOMTRANSMIT SerCx2CustomTx;
    SERCX2CUSTOMTRANSMITTRANSACTION SerCx2CustomTxTransaction;

    //
    // Runtime
    //
    bool IsDeferredCancellation;
};

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(
    IMX_LPUART_TX_DMA_TRANSACTION_CONTEXT,
    iMXLPUARTGetTxDmaTransactionContext);

//
// NONPAGED
//

EVT_WDF_INTERRUPT_ISR iMXLPUARTEvtInterruptISR;
EVT_WDF_INTERRUPT_DPC iMXLPUARTEvtInterruptDpc;

EVT_SERCX2_FILEOPEN iMXLPUARTEvtSerCx2FileOpen;
EVT_SERCX2_FILECLOSE iMXLPUARTEvtSerCx2FileClose;
EVT_SERCX2_SET_WAIT_MASK iMXLPUARTEvtSerCx2SetWaitMask;
EVT_SERCX2_CONTROL iMXLPUARTEvtSerCx2Control;

EVT_SERCX2_PIO_RECEIVE_READ_BUFFER iMXLPUARTEvtSerCx2PioReceiveReadBuffer;
EVT_SERCX2_PIO_RECEIVE_ENABLE_READY_NOTIFICATION iMXLPUARTEvtSerCx2PioReceiveEnableReadyNotification;
EVT_SERCX2_PIO_RECEIVE_CANCEL_READY_NOTIFICATION iMXLPUARTEvtSerCx2PioReceiveCancelReadyNotification;

EVT_SERCX2_PIO_TRANSMIT_WRITE_BUFFER iMXLPUARTEvtSerCx2PioTransmitWriteBuffer;
EVT_SERCX2_PIO_TRANSMIT_ENABLE_READY_NOTIFICATION iMXLPUARTEvtSerCx2PioTransmitEnableReadyNotification;
EVT_SERCX2_PIO_TRANSMIT_CANCEL_READY_NOTIFICATION iMXLPUARTEvtSerCx2PioTransmitCancelReadyNotification;
EVT_SERCX2_PIO_TRANSMIT_DRAIN_FIFO iMXLPUARTEvtSerCx2PioTransmitDrainFifo;
EVT_SERCX2_PIO_TRANSMIT_CANCEL_DRAIN_FIFO iMXLPUARTEvtSerCx2PioTransmitCancelDrainFifo;
EVT_SERCX2_PIO_TRANSMIT_PURGE_FIFO iMXLPUARTEvtSerCx2PioTransmitPurgeFifo;

EVT_SERCX2_CUSTOM_RECEIVE_TRANSACTION_START iMXLPUARTEvtSerCx2CustomReceiveTransactionStart;
EVT_SERCX2_CUSTOM_RECEIVE_TRANSACTION_CLEANUP iMXLPUARTEvtSerCx2CustomReceiveTransactionCleanup;
EVT_SERCX2_CUSTOM_RECEIVE_TRANSACTION_QUERY_PROGRESS iMXLPUARTEvtSerCx2CustomReceiveTransactionQueryProgress;
EVT_WDF_REQUEST_CANCEL iMXLPUARTEvtCustomReceiveTransactionRequestCancel;

EVT_WDF_DMA_TRANSACTION_CONFIGURE_DMA_CHANNEL iMXLPUARTEvtWdfRxDmaTransactionConfigureDmaChannel;
EVT_WDF_DMA_TRANSACTION_DMA_TRANSFER_COMPLETE iMXLPUARTEvtWdfRxDmaTransactionTransferComplete;
EVT_WDF_PROGRAM_DMA iMXLPUARTEvtWdfProgramRxDma;
EVT_WDF_TIMER iMXLPUARTEvtTimerRxDmaProgress;

EVT_SERCX2_CUSTOM_TRANSMIT_TRANSACTION_INITIALIZE iMXLPUARTEvtSerCx2CustomTransmitTransactionInitialize;
EVT_SERCX2_CUSTOM_TRANSMIT_TRANSACTION_START iMXLPUARTEvtSerCx2CustomTransmitTransactionStart;
EVT_SERCX2_CUSTOM_TRANSMIT_TRANSACTION_CLEANUP iMXLPUARTEvtSerCx2CustomTransmitTransactionCleanup;
EVT_WDF_REQUEST_CANCEL iMXLPUARTEvtCustomTransmitTransactionRequestCancel;

EVT_WDF_DMA_TRANSACTION_CONFIGURE_DMA_CHANNEL iMXLPUARTEvtWdfTxDmaTransactionConfigureDmaChannel;
EVT_WDF_DMA_TRANSACTION_DMA_TRANSFER_COMPLETE iMXLPUARTEvtWdfTxDmaTransactionTransferComplete;
EVT_WDF_PROGRAM_DMA iMXLPUARTEvtWdfProgramTxDma;

EVT_SERCX2_PURGE_FIFOS iMXLPUARTEvtSerCx2PurgeFifos;
EVT_SERCX2_APPLY_CONFIG iMXLPUARTEvtSerCx2ApplyConfig;

NTSTATUS
iMXLPUARTCheckValidStateForConfigIoctl (
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr
    );

NTSTATUS
iMXLPUARTResetHardwareAndPreserveShadowedRegisters (
    const IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
    );

NTSTATUS
iMXLPUARTResetHardwareAndClearShadowedRegisters (
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
    );

void
iMXLPUARTRestoreHardwareFromShadowedRegisters (
    const IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
    );

void
iMXLPUARTComputeFifoThresholds (
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    ULONG BaudRate,
    _In_range_(9, 11) ULONG BitsPerFrame,
    _Out_range_(2, IMX_LPUART_FIFO_COUNT) ULONG *TxFifoThresholdPtr,
    _Out_range_(1, IMX_LPUART_FIFO_COUNT - 1) ULONG *RxFifoThresholdPtr
    );

ULONG
iMXLPUARTComputeCharactersPerDuration (
    ULONG DurationUs,
    ULONG BaudRate,
    ULONG BitsPerFrame
    );

NTSTATUS
iMXLPUARTComputeDividers (
    ULONG LPUARTModuleClock,
    ULONG DesiredBaudRate,
    _Out_ ULONG *OSRPtr,
    _Out_ ULONG *SBRPtr
    );

_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS
iMXLPUARTConfigureUart (
    WDFDEVICE WdfDevice,
    ULONG BaudRate,
    const SERIAL_LINE_CONTROL *LineControl,
    const SERIAL_HANDFLOW *Handflow
    // , bool RtsCtsEnabled
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
iMXLPUARTSetBaudRate (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    ULONG BaudRate
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
iMXLPUARTSetLineControl (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    const SERIAL_LINE_CONTROL *LineControlPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
iMXLPUARTSetHandflow (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    const SERIAL_HANDFLOW *LineControlPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTSetBreakOnOff (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    bool BreakOn
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTSetWaitMask (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    ULONG WaitMask
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
iMXLPUARTUpdateDmaSettings (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
iMXLPUARTStartRxDma (
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
iMXLPUARTStopRxDma (
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
FORCEINLINE ULONG
iMXLPUARTDmaReadCounter (
    PDMA_ADAPTER DmaAdapterPtr
    )
{
    return DmaAdapterPtr->DmaOperations->ReadDmaCounter(DmaAdapterPtr);
}

FORCEINLINE bool
iMXLPUARTIsRxDmaActive (
    const IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
    )
{
    const ULONG rxDmaState = InterruptContextPtr->RxDmaState;
    return (rxDmaState & IMX_LPUART_STATE_ACTIVE) != 0;
}

FORCEINLINE bool
iMXLPUARTIsTxDmaActive (
    const IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
    )
{
    const ULONG txDmaState = InterruptContextPtr->TxDmaState |
        InterruptContextPtr->TxDmaDrainState;

    return (txDmaState & IMX_LPUART_STATE_ACTIVE) != 0;
}

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
iMXLPUARTAcquireDmaRequestLineOwnership (
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
iMXLPUARTReleaseDmaRequestLineOwnership (
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
ULONG
iMXLPUARTRxDmaGetBytesTransferred (
    IMX_LPUART_RX_DMA_TRANSACTION_CONTEXT *RxDmaTransactionContextPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
VOID
iMXLPUARTRxDmaStartProgressTimer (
    IMX_LPUART_RX_DMA_TRANSACTION_CONTEXT *RxDmaTransactionContextPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
bool
iMXLPUARTRxDmaCopyToUserBuffer (
    IMX_LPUART_RX_DMA_TRANSACTION_CONTEXT *RxDmaTransactionContextPtr,
    size_t NewBytesTransferred
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
ULONG
iMXLPUARTPioDequeueDmaBytes (
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr,
    _Out_writes_(Length) PUCHAR Buffer,
    ULONG Length
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
VOID
iMXLPUARTCompleteCustomRxTransactionRequest (
    IMX_LPUART_RX_DMA_TRANSACTION_CONTEXT *RxDmaTransactionContextPtr,
    NTSTATUS RequestStatus
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
VOID
iMXLPUARTTxDmaDrainFifo (
    IMX_LPUART_TX_DMA_TRANSACTION_CONTEXT *TxDmaTransactionContextPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
VOID
iMXLPUARTCompleteCustomTxTransactionRequest (
    IMX_LPUART_TX_DMA_TRANSACTION_CONTEXT *TxDmaTransactionContextPtr,
    NTSTATUS RequestStatus
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS
iMXLPUARTGetErrorInformation (
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
VOID
iMXLPUARTNotifyEventsDuringDma (
    IMX_LPUART_DMA_TRANSACTION_CONTEXT *DmaTransactionContextPtr,
    const ULONG WaitEvents,
    const ULONG CommStatusInfo
    );

//
// IOCTL Handlers
//

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlSetBaudRate (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlGetBaudRate (
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlSetLineControl (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlGetLineControl (
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlSetHandflow (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlGetHandflow (
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlGetProperties (
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlGetCommStatus (
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlGetModemStatus (
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlSetClrRts (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest,
    bool RtsState
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlGetDtrRts (
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlSetBreakOnOff (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest,
    bool BreakOn
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlGetModemControl (
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
    );

_IRQL_requires_max_(DISPATCH_LEVEL)
void
iMXLPUARTIoctlSetClrDtr (
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest,
    bool SetDtr
);

//
// ACPI - Device Properties
//

_IRQL_requires_max_(APC_LEVEL)
NTSTATUS iMXLPUARTReadDeviceProperties(
    WDFDEVICE WdfDevice,
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr
    );

//
// PAGED
//

EVT_WDF_DEVICE_PREPARE_HARDWARE iMXLPUARTEvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE iMXLPUARTEvtDeviceReleaseHardware;
EVT_WDF_DRIVER_DEVICE_ADD iMXLPUARTEvtDeviceAdd;
EVT_WDF_DRIVER_UNLOAD iMXLPUARTEvtDriverUnload;

_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS
iMXLPUARTCreateDeviceInterface (
    WDFDEVICE WdfDevice,
    LARGE_INTEGER ConnectionId
    );

_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS
iMXLPUARTParseConnectionDescriptor (
    const RH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER *RhBufferPtr,
    _Out_ ULONG *BaudRatePtr,
    _Out_ SERIAL_LINE_CONTROL *LineControlPtr,
    _Out_ SERIAL_HANDFLOW *HandflowPtr,
    _Out_ bool *RtsCtsEnabledPtr
    );

_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS
iMXLPUARTInitializeDma (
    WDFDEVICE WdfDevice,
    const CM_PARTIAL_RESOURCE_DESCRIPTOR *RegistersResourcePtr,
    const CM_PARTIAL_RESOURCE_DESCRIPTOR *RxDmaResourcePtr,
    const CM_PARTIAL_RESOURCE_DESCRIPTOR *TxDmaResourcePtr
    );

_IRQL_requires_max_(APC_LEVEL)
VOID
iMXLPUARTDeinitializeDma (
    WDFDEVICE WdfDevice
    );

extern "C" DRIVER_INITIALIZE DriverEntry;

#endif // _IMX_LPUART_H_
