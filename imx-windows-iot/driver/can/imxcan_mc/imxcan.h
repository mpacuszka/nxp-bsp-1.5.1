/*
 * Copyright 2022 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once
#pragma warning(push)
#pragma warning(disable:4201) /* nameless struct/union */


#include "imxcanhw.h"

#define IMXCAN_NONPAGED_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    //__pragma(code_seg(.text))

#define IMXCAN_NONPAGED_SEGMENT_END \
    __pragma(code_seg(pop))

#define IMXCAN_PAGED_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    __pragma(code_seg("PAGE"))

#define IMXCAN_PAGED_SEGMENT_END \
    __pragma(code_seg(pop))

#define IMXCAN_INIT_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    __pragma(code_seg("INIT"))

#define IMXCAN_INIT_SEGMENT_END \
    __pragma(code_seg(pop))

/*  Pool tag that the framework will assign to all of the driver's pool allocations. */
#define IMXCAN_POOL_TAG 'WCXM'
/* The maximum number of entries in the circular buffer that is used to transfer data from the InterruptIsr routine to the InterruptDpc. */
#define CB_CAPACITY (8U)

#pragma data_seg("NONPAGE")

/* FlexCAN Message Buffer transfer. */
typedef struct _flexcan_mb_transfer
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    flexcan_fd_frame_t *framefd;    /* The FD buffer of CAN Message to be transfer. */
#endif
    flexcan_frame_t    *frame;      /* The buffer of CAN Message to be transfer. */
    UINT8               mbIdx;      /* The index of Message buffer used to transfer Message. */
} flexcan_mb_transfer_t;

/* FlexCAN handle structure. */
typedef struct _flexcan_handle_t
{
    void *userData;                                                   /* FlexCAN callback function parameter. */
    flexcan_frame_t *volatile       mbFrameBuf[CAN_WORD1_COUNT];      /* The buffer for received CAN data from Message Buffers. */
    flexcan_fd_frame_t *volatile    mbFDFrameBuf[CAN_WORD1_COUNT];    /* The buffer for received CAN FD data from Message Buffers. */
    flexcan_frame_t *volatile       rxFifoFrameBuf;                   /* The buffer for received CAN data from Legacy Rx FIFO. */
    flexcan_fd_frame_t *volatile    rxFifoFDFrameBuf;                 /* The buffer for received CAN FD data from Ehanced Rx FIFO. */
    UINT32                          frameNum;                         /* The number of CAN messages remaining to be received. */
    UINT32                          transferTotalNum;                 /* Total CAN Message number need to be received from Ehanced Rx FIFO. */
    volatile UINT8                  mbState[CAN_WORD1_COUNT];         /* Message Buffer transfer state. */
    volatile UINT8                  rxFifoState;                      /* Rx FIFO transfer state. */
    volatile UINT32                 timestamp[CAN_WORD1_COUNT];       /* Mailbox transfer timestamp. */
    flexcan_mb_transfer_t           TransferBuf[CAN_WORD1_COUNT];
    flexcan_frame_t                 TransferFrame[CAN_WORD1_COUNT];
    flexcan_fd_frame_t              TransferFDFrame[CAN_WORD1_COUNT];

    flexcan_frame_t *volatile       copymbFrame;                      /* Temporary copy of received frame */
    flexcan_fd_frame_t *volatile    copymbFDFrame;                    /* Temporary copy of received FD frame */
} flexcan_handle_t;

typedef struct _IMXCAN_PIN_STATE {
    BOOLEAN IsStarted;
    BOOLEAN IsOpenForWrite;
    /* A lock to protect IsOpenForWrite during file create/close callbacks */
    WDFWAITLOCK Lock;
} IMXCAN_PIN_STATE;

typedef struct _DEV_CONTEXT {
    IMXCAN_REGISTERS    *RegistersPtr;
    WDFDEVICE           WdfDevice;
    WDFINTERRUPT        WdfInterrupt;
    WDFREQUEST          CurrentRequest;
    WDFSTRING           DeviceInterfaceSymlinkName;
    UNICODE_STRING      DeviceInterfaceSymlinkNameWsz;
    /* HW State */
    flexcan_config_t    CANconfig;
    flexcan_handle_t    CANhandle;
    WDFQUEUE            NotificationQueue;

    /* Controller and Pin State */
    BOOLEAN             IsControllerOpenForWrite;
    /* A lock to protect the controller IsOpenForWrite during file create/close callbacks */
    WDFWAITLOCK         ControllerLock;
    IMXCAN_PIN_STATE    Pin;
    /* Controller Info */
    CAN_CONTROLLER_INFO ControllerInfo;
} DEV_CONTEXT, *PDEV_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEV_CONTEXT,ImxCanGetDeviceContext);

typedef struct _IMXCAN_FILE_OBJECT_CONTEXT {
    BOOLEAN IsOpenForWrite;
    BOOLEAN IsPinInterface;
} IMXCAN_FILE_OBJECT_CONTEXT;

typedef struct _circular_buffer
{
    void *buffer;     /* data buffer */
    void *buffer_end; /* end of data buffer */
    size_t capacity;  /* maximum number of items in the buffer */
    size_t count;     /* number of items in the buffer */
    size_t sz;        /* size of each item in the buffer */
    void *head;       /* pointer to head */
    void *tail;       /* pointer to tail */
} circular_buffer;

typedef struct _INT_CONTEXT {
    UINT64 InterruptStatus;
    circular_buffer FrameQueue;
    WDFMEMORY memhandle;
} INT_CONTEXT, *PINT_CONTEXT; /* struct IMXCAN_INTERRUPT_CONTEXT */

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(INT_CONTEXT, ImxCanGetInterruptContext);
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(IMXCAN_FILE_OBJECT_CONTEXT, ImxCanGetFileObjectContext);

_IRQL_requires_same_ NTSTATUS ImxCanResetPinDefaults(PDEV_CONTEXT DeviceContextPtr);
_IRQL_requires_same_ NTSTATUS ImxCanResetControllerDefaults(PDEV_CONTEXT DeviceContextPtr);
_IRQL_requires_same_ _IRQL_requires_max_(PASSIVE_LEVEL) NTSTATUS ImxCanCreateDeviceInterface(_In_ PDEV_CONTEXT DeviceContextPtr);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerGetInfo(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerGetConfig(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetClkSrc(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetBitRate(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetLoopBack(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetSelfWakeup(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetListenOnlyMode(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetDoze(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerImprovedTiming(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerFDImprovedTiming(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#endif
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerInit(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerFDInit(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#endif
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerDeinit(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetRxMbConfig(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetFDRxMbConfig(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetFDTxMbConfig(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#endif
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlHelperFLEXCAN_ID_STD(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlHelperFLEXCAN_RX_MB_STD_MASK(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetTxMbConfig(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerTransferCreateHandle(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerReceiveNonBlocking(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSendNonBlocking(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerReceiveBlocking(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSendBlocking(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerFDSendBlocking(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerFDReceiveBlocking(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerFDSendNonBlocking(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerFDReceiveNonBlocking(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerFDAbortSend(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerFDAbortReceive(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#endif
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerNotification(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerEnterFreezeMode(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerExitFreezeMode(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetTimingConfig(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetFDTimingConfig(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#endif
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetRxMbGlobalMask(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetRxFifoGlobalMask(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetRxIndividualMask(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerSetRxFifoConfig(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerGetStatusFlags(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerClearStatusFlags(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerGetBusErrCount(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerGetMbStatusFlags(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerClearMbStatusFlags(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerGetMemoryErrorReportStatus(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#endif
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerEnableInterrupts(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerDisableInterrupts(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerEnableMbInterrupts(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerDisableMbInterrupts(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA) && FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA)
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerEnableRxFifoDMA(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerGetRxFifoHeadAddr(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#endif
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerEnable(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerWriteTxMb(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerReadRxMb(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerWriteFDTxMb(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerReadFDRxMb(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
#endif
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerReadRxFifo(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerReceiveFifoBlocking(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerReceiveFifoNonBlocking(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerGetTimeStamp(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerAbortSend(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerAbortReceive(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);
_IRQL_requires_max_(DISPATCH_LEVEL) VOID ImxCanIoctlControllerAbortReceiveFifo(_In_ const PDEV_CONTEXT DeviceContextPtr, _In_ WDFREQUEST WdfRequest);

EVT_WDF_INTERRUPT_ISR ImxCanEvtInterruptIsr;
EVT_WDF_INTERRUPT_DPC ImxCanEvtInterruptDpc;
EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL ImxCanEvtIoDeviceControl;
EVT_WDF_DEVICE_FILE_CREATE ImxCanEvtDeviceFileCreate;
EVT_WDF_FILE_CLOSE ImxCanEvtFileClose;

#if DBG
VOID FLEXCAN_DumpRegisters(volatile IMXCAN_REGISTERS *base);
#endif
VOID FLEXCAN_Reset(volatile IMXCAN_REGISTERS *base);

/* Definitions */

/* Computes the number of elements in an array. */
#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/* FlexCAN frame length helper macro. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
#define DLC_LENGTH_DECODE(dlc) (((dlc) <= 8U) ? (dlc) : (((dlc) <= 12U) ? (((dlc)-6U) * 4U) : (((dlc)-11U) * 16U)))
#endif

/* FlexCAN Frame ID helper macro. */
#define FLEXCAN_ID_STD(id) \
    (((UINT32)(((UINT32)(id)) << CAN_ID_STD_SHIFT)) & CAN_ID_STD_MASK) /* Standard Frame ID helper macro. */
#define FLEXCAN_ID_EXT(id)                                \
    (((UINT32)(((UINT32)(id)) << CAN_ID_EXT_SHIFT)) & \
     (CAN_ID_EXT_MASK | CAN_ID_STD_MASK)) /* Extend Frame ID helper macro. */

/* FlexCAN Rx Message Buffer Mask helper macro. */
#define FLEXCAN_RX_MB_STD_MASK(id, rtr, ide)                                   \
    (((UINT32)((UINT32)(rtr) << 31) | (UINT32)((UINT32)(ide) << 30)) | \
     FLEXCAN_ID_STD(id)) /* Standard Rx Message Buffer Mask helper macro. */
#define FLEXCAN_RX_MB_EXT_MASK(id, rtr, ide)                                   \
    (((UINT32)((UINT32)(rtr) << 31) | (UINT32)((UINT32)(ide) << 30)) | \
     FLEXCAN_ID_EXT(id)) /* Extend Rx Message Buffer Mask helper macro. */

/* FlexCAN Legacy Rx FIFO Mask helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_A(id, rtr, ide)                          \
    (((UINT32)((UINT32)(rtr) << 31) | (UINT32)((UINT32)(ide) << 30)) | \
     (FLEXCAN_ID_STD(id) << 1)) /* Standard Rx FIFO Mask helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_HIGH(id, rtr, ide)                     \
    (((UINT32)((UINT32)(rtr) << 31) | (UINT32)((UINT32)(ide) << 30)) | \
     (((UINT32)(id)&0x7FF) << 19)) /* Standard Rx FIFO Mask helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_LOW(id, rtr, ide)                      \
    (((UINT32)((UINT32)(rtr) << 15) | (UINT32)((UINT32)(ide) << 14)) | \
     (((UINT32)(id)&0x7FF) << 3)) /* Standard Rx FIFO Mask helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_HIGH(id) \
    (((UINT32)(id)&0x7F8) << 21) /* Standard Rx FIFO Mask helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_HIGH(id) \
    (((UINT32)(id)&0x7F8) << 13) /* Standard Rx FIFO Mask helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_LOW(id) \
    (((UINT32)(id)&0x7F8) << 5) /* Standard Rx FIFO Mask helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_LOW(id) \
    (((UINT32)(id)&0x7F8) >> 3) /* Standard Rx FIFO Mask helper macro Type C lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_A(id, rtr, ide)                          \
    (((UINT32)((UINT32)(rtr) << 31) | (UINT32)((UINT32)(ide) << 30)) | \
     (FLEXCAN_ID_EXT(id) << 1)) /* Extend Rx FIFO Mask helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(id, rtr, ide)                        \
    (                                                                             \
        ((UINT32)((UINT32)(rtr) << 31) | (UINT32)((UINT32)(ide) << 30)) | \
        ((FLEXCAN_ID_EXT(id) & 0x1FFF8000)                                        \
         << 1)) /* Extend Rx FIFO Mask helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(id, rtr, ide)                      \
    (((UINT32)((UINT32)(rtr) << 15) | (UINT32)((UINT32)(ide) << 14)) | \
     ((FLEXCAN_ID_EXT(id) & 0x1FFF8000) >>                                     \
      15)) /* Extend Rx FIFO Mask helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_HIGH(id) \
    ((FLEXCAN_ID_EXT(id) & 0x1FE00000) << 3) /* Extend Rx FIFO Mask helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_HIGH(id) \
    ((FLEXCAN_ID_EXT(id) & 0x1FE00000) >>            \
     5) /* Extend Rx FIFO Mask helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_LOW(id) \
    ((FLEXCAN_ID_EXT(id) & 0x1FE00000) >>           \
     13) /* Extend Rx FIFO Mask helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_LOW(id) \
    ((FLEXCAN_ID_EXT(id) & 0x1FE00000) >> 21) /* Extend Rx FIFO Mask helper macro Type C lower part helper macro. */

/* FlexCAN Rx FIFO Filter helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(id, rtr, ide) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_A(id, rtr, ide) /* Standard Rx FIFO Filter helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_B_HIGH(id, rtr, ide) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_HIGH(id, rtr, ide) /* Standard Rx FIFO Filter helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_B_LOW(id, rtr, ide) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_LOW(id, rtr, ide) /* Standard Rx FIFO Filter helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_HIGH(id) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_HIGH(id) /* Standard Rx FIFO Filter helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_MID_HIGH(id) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_HIGH(id) /* Standard Rx FIFO Filter helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_MID_LOW(id) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_LOW(id) /* Standard Rx FIFO Filter helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_LOW(id) \
    FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_LOW(id) /* Standard Rx FIFO Filter helper macro Type C lower part helper macro.  */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_A(id, rtr, ide) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_A(id, rtr, ide) /* Extend Rx FIFO Filter helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_B_HIGH(id, rtr, ide) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(id, rtr, ide) /* Extend Rx FIFO Filter helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_B_LOW(id, rtr, ide) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(id, rtr, ide) /* Extend Rx FIFO Filter helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_HIGH(id) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_HIGH(id) /* Extend Rx FIFO Filter helper macro Type C upper part helper macro.           */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_MID_HIGH(id) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_HIGH(id) /* Extend Rx FIFO Filter helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_MID_LOW(id) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_LOW(id) /* Extend Rx FIFO Filter helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_LOW(id) \
    FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_LOW(id) /* Extend Rx FIFO Filter helper macro Type C lower part helper macro. */

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/* FlexCAN Enhanced Rx FIFO Filter and Mask helper macro. */
#define ENHANCED_RX_FIFO_FSCH(x) (((UINT32)(((UINT32)(x)) << 30)) & 0xC0000000U)
#define RTR_STD_HIGH(x)          (((UINT32)(((UINT32)(x)) << 27)) & 0x08000000U)
#define RTR_STD_LOW(x)           (((UINT32)(((UINT32)(x)) << 11)) & 0x00000800U)
#define RTR_EXT(x)               (((UINT32)(((UINT32)(x)) << 29)) & 0x40000000U)
#define ID_STD_LOW(id)           (((UINT32)id) & 0x7FFU)
#define ID_STD_HIGH(id)          (((UINT32)(((UINT32)(id)) << 16)) & 0x07FF0000U)
#define ID_EXT(id)               (((UINT32)id) & 0x1FFFFFFFU)

/* Standard ID filter element with filter + mask scheme. */
#define FLEXCAN_ENHANCED_RX_FIFO_STD_MASK_AND_FILTER(id, rtr, id_mask, rtr_mask) \
    (ENHANCED_RX_FIFO_FSCH(0x0) | RTR_STD_HIGH(rtr) | ID_STD_HIGH(id) | RTR_STD_LOW(rtr_mask) | ID_STD_LOW(id_mask))
/* Standard ID filter element with filter range. */
#define FLEXCAN_ENHANCED_RX_FIFO_STD_FILTER_WITH_RANGE(id_upper, rtr, id_lower, rtr_mask)             \
    (ENHANCED_RX_FIFO_FSCH(0x1) | RTR_STD_HIGH(rtr) | ID_STD_HIGH(id_upper) | RTR_STD_LOW(rtr_mask) | \
     ID_STD_LOW(id_lower))
/* Standard ID filter element with two filters without masks. */
#define FLEXCAN_ENHANCED_RX_FIFO_STD_TWO_FILTERS(id1, rtr1, id2, rtr2) \
    (ENHANCED_RX_FIFO_FSCH(0x2) | RTR_STD_HIGH(rtr1) | ID_STD_HIGH(id1) | RTR_STD_LOW(rtr2) | ID_STD_LOW(id2))
/* Extended ID filter element with  filter + mask scheme low word. */
#define FLEXCAN_ENHANCED_RX_FIFO_EXT_MASK_AND_FILTER_LOW(id, rtr) \
    (ENHANCED_RX_FIFO_FSCH(0x0) | RTR_EXT(rtr) | ID_EXT(id))
/* Extended ID filter element with  filter + mask scheme high word. */
#define FLEXCAN_ENHANCED_RX_FIFO_EXT_MASK_AND_FILTER_HIGH(id_mask, rtr_mask) \
    (ENHANCED_RX_FIFO_FSCH(0x0) | RTR_EXT(rtr_mask) | ID_EXT(id_mask))
/* Extended ID filter element with range scheme low word. */
#define FLEXCAN_ENHANCED_RX_FIFO_EXT_FILTER_WITH_RANGE_LOW(id_upper, rtr) \
    (ENHANCED_RX_FIFO_FSCH(0x1) | RTR_EXT(rtr) | ID_EXT(id_upper))
/* Extended ID filter element with range scheme high word. */
#define FLEXCAN_ENHANCED_RX_FIFO_EXT_FILTER_WITH_RANGE_HIGH(id_lower, rtr_mask) \
    (ENHANCED_RX_FIFO_FSCH(0x1) | RTR_EXT(rtr_mask) | ID_EXT(id_lower))
/* Extended ID filter element with two filters without masks low word. */
#define FLEXCAN_ENHANCED_RX_FIFO_EXT_TWO_FILTERS_LOW(id2, rtr2) \
    (ENHANCED_RX_FIFO_FSCH(0x2) | RTR_EXT(rtr2) | ID_EXT(id2))
/* Extended ID filter element with two filters without masks high word. */
#define FLEXCAN_ENHANCED_RX_FIFO_EXT_TWO_FILTERS_HIGH(id1, rtr1) \
    (ENHANCED_RX_FIFO_FSCH(0x2) | RTR_EXT(rtr1) | ID_EXT(id1))
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
/* FlexCAN Pretended Networking ID Mask helper macro. */
#define FLEXCAN_PN_STD_MASK(id, rtr)                            \
    ((UINT32)((UINT32)(rtr) << CAN_FLT_ID1_FLT_RTR_SHIFT) | \
     FLEXCAN_ID_STD(id)) /* Standard Rx Message Buffer Mask helper macro. */
#define FLEXCAN_PN_EXT_MASK(id, rtr)                                                                 \
    ((UINT32)CAN_FLT_ID1_FLT_IDE_MASK | (UINT32)((UINT32)(rtr) << CAN_FLT_ID1_FLT_RTR_SHIFT) | \
     FLEXCAN_ID_EXT(id)) /* Extend Rx Message Buffer Mask helper macro. */
#endif

/* FlexCAN interrupt/status flag helper macro. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
#define FLEXCAN_PN_INT_MASK(x)      (((UINT64)(((UINT64)(x)) << 32)) & 0x3000000000000U)
#define FLEXCAN_PN_INT_UNMASK(x)    (((UINT32)(((UINT64)(x)) >> 32)) & 0x00030000U)
#define FLEXCAN_PN_STATUS_MASK(x)   (((UINT64)(((UINT64)(x)) << 16)) & 0x300000000U)
#define FLEXCAN_PN_STATUS_UNMASK(x) (((UINT32)(((UINT64)(x)) >> 16)) & 0x00030000U)
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
#define FLEXCAN_EFIFO_INT_MASK(x)      (((UINT64)(((UINT64)(x)) << 32)) & 0xF000000000000000U)
#define FLEXCAN_EFIFO_INT_UNMASK(x)    (((UINT32)(((UINT64)(x)) >> 32)) & 0xF0000000U)
#define FLEXCAN_EFIFO_STATUS_MASK(x)   (((UINT64)(((UINT64)(x)) << 32)) & 0xF003000000000000U)
#define FLEXCAN_EFIFO_STATUS_UNMASK(x) (((UINT32)(((UINT64)(x)) >> 32)) & 0xF0030000U)
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
#define FLEXCAN_MECR_INT_MASK(x)      (((UINT64)(((UINT64)(x)) << 16)) & 0xD00000000U)
#define FLEXCAN_MECR_INT_UNMASK(x)    (((UINT32)(((UINT64)(x)) >> 16)) & 0x000D0000U)
#define FLEXCAN_MECR_STATUS_MASK(x)   (((UINT64)(((UINT64)(x)) << 34)) & 0x34003400000000U)
#define FLEXCAN_MECR_STATUS_UNMASK(x) (((UINT32)(((UINT64)(x)) >> 34)) & 0x000D000DU)
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
#define FLEXCAN_ERROR_AND_STATUS_INIT_FLAG                                                                            \
    ((UINT32)kFLEXCAN_ErrorOverrunFlag | (UINT32)kFLEXCAN_FDErrorIntFlag | (UINT32)kFLEXCAN_BusoffDoneIntFlag | \
     (UINT32)kFLEXCAN_TxWarningIntFlag | (UINT32)kFLEXCAN_RxWarningIntFlag | (UINT32)kFLEXCAN_BusOffIntFlag |   \
     (UINT32)kFLEXCAN_ErrorIntFlag | FLEXCAN_MEMORY_ERROR_INIT_FLAG)
#else
#define FLEXCAN_ERROR_AND_STATUS_INIT_FLAG                                                                          \
    ((UINT32)kFLEXCAN_TxWarningIntFlag | (UINT32)kFLEXCAN_RxWarningIntFlag | (UINT32)kFLEXCAN_BusOffIntFlag | \
     (UINT32)kFLEXCAN_ErrorIntFlag | FLEXCAN_MEMORY_ERROR_INIT_FLAG)
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
#define FLEXCAN_WAKE_UP_FLAG \
    ((UINT32)kFLEXCAN_WakeUpIntFlag | (UINT64)kFLEXCAN_PNMatchIntFlag | (UINT64)kFLEXCAN_PNTimeoutIntFlag)
#else
#define FLEXCAN_WAKE_UP_FLAG ((UINT32)kFLEXCAN_WakeUpIntFlag)
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
#define FLEXCAN_MEMORY_ERROR_INIT_FLAG ((UINT64)kFLEXCAN_AllMemoryErrorFlag)
#else
#define FLEXCAN_MEMORY_ERROR_INIT_FLAG (0U)
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
#define FLEXCAN_MEMORY_ENHANCED_RX_FIFO_INIT_FLAG                                             \
    ((UINT64)kFLEXCAN_ERxFifoUnderflowIntFlag | (UINT64)kFLEXCAN_ERxFifoOverflowIntFlag | \
     (UINT64)kFLEXCAN_ERxFifoWatermarkIntFlag | (UINT64)kFLEXCAN_ERxFifoDataAvlIntFlag)
#define FLEXCAN_MEMORY_ENHANCED_RX_FIFO_INIT_MASK \
    (CAN_ERFIER_ERFUFWIE_MASK | CAN_ERFIER_ERFOVFIE_MASK | CAN_ERFIER_ERFWMIIE_MASK | CAN_ERFIER_ERFDAIE_MASK)
#endif

/* FlexCAN Enhanced Rx FIFO base address helper macro. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
#define E_RX_FIFO(base) ((UINT32)(base) + 0x2000U)
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)

/*
 * FlexCAN CAN FD frame supporting data length (available DLC values).
 *
 * For Tx, when the Data size corresponding to DLC value stored in the MB selected for transmission is larger than the
 * MB Payload size, FlexCAN adds the necessary number of bytes with constant 0xCC pattern to complete the expected DLC.
 * For Rx, when the Data size corresponding to DLC value received from the CAN bus is larger than the MB Payload size,
 * the high order bytes that do not fit the Payload size will lose.
 */
enum _flexcan_fd_frame_length
{
    kFLEXCAN_0BperFrame = 0x0U, /* Frame contains 0 valid data bytes. */
    kFLEXCAN_1BperFrame,        /* Frame contains 1 valid data bytes. */
    kFLEXCAN_2BperFrame,        /* Frame contains 2 valid data bytes. */
    kFLEXCAN_3BperFrame,        /* Frame contains 3 valid data bytes. */
    kFLEXCAN_4BperFrame,        /* Frame contains 4 valid data bytes. */
    kFLEXCAN_5BperFrame,        /* Frame contains 5 valid data bytes. */
    kFLEXCAN_6BperFrame,        /* Frame contains 6 valid data bytes. */
    kFLEXCAN_7BperFrame,        /* Frame contains 7 valid data bytes. */
    kFLEXCAN_8BperFrame,        /* Frame contains 8 valid data bytes. */
    kFLEXCAN_12BperFrame,       /* Frame contains 12 valid data bytes. */
    kFLEXCAN_16BperFrame,       /* Frame contains 16 valid data bytes. */
    kFLEXCAN_20BperFrame,       /* Frame contains 20 valid data bytes. */
    kFLEXCAN_24Bperrame,        /* Frame contains 24 valid data bytes. */
    kFLEXCAN_32BperFrame,       /* Frame contains 32 valid data bytes. */
    kFLEXCAN_48BperFrame,       /* Frame contains 48 valid data bytes. */
    kFLEXCAN_64BperFrame,       /* Frame contains 64 valid data bytes. */
};
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/* FlexCAN Enhanced Rx Fifo DMA transfer per read length enumerations. */
typedef enum _flexcan_efifo_dma_per_read_length
{
    kFLEXCAN_1WordPerRead = 0x0U, /* Transfer 1 32-bit words (CS).*/
    kFLEXCAN_2WordPerRead,        /* Transfer 2 32-bit words (CS + ID).*/
    kFLEXCAN_3WordPerRead,        /* Transfer 3 32-bit words (CS + ID + 1~4 bytes data).*/
    kFLEXCAN_4WordPerRead,        /* Transfer 4 32-bit words (CS + ID + 5~8 bytes data).*/
    kFLEXCAN_5WordPerRead,        /* Transfer 5 32-bit words (CS + ID + 9~12 bytes data).*/
    kFLEXCAN_6WordPerRead,        /* Transfer 6 32-bit words (CS + ID + 13~16 bytes data).*/
    kFLEXCAN_7WordPerRead,        /* Transfer 7 32-bit words (CS + ID + 17~20 bytes data).*/
    kFLEXCAN_8WordPerRead,        /* Transfer 8 32-bit words (CS + ID + 21~24 bytes data).*/
    kFLEXCAN_9WordPerRead,        /* Transfer 9 32-bit words (CS + ID + 25~28 bytes data).*/
    kFLEXCAN_10WordPerRead,       /* Transfer 10 32-bit words (CS + ID + 29~32 bytes data).*/
    kFLEXCAN_11WordPerRead,       /* Transfer 11 32-bit words (CS + ID + 33~36 bytes data).*/
    kFLEXCAN_12WordPerRead,       /* Transfer 12 32-bit words (CS + ID + 37~40 bytes data).*/
    kFLEXCAN_13WordPerRead,       /* Transfer 13 32-bit words (CS + ID + 41~44 bytes data).*/
    kFLEXCAN_14WordPerRead,       /* Transfer 14 32-bit words (CS + ID + 45~48 bytes data).*/
    kFLEXCAN_15WordPerRead,       /* Transfer 15 32-bit words (CS + ID + 49~52 bytes data).*/
    kFLEXCAN_16WordPerRead,       /* Transfer 16 32-bit words (CS + ID + 53~56 bytes data).*/
    kFLEXCAN_17WordPerRead,       /* Transfer 17 32-bit words (CS + ID + 57~60 bytes data).*/
    kFLEXCAN_18WordPerRead,       /* Transfer 18 32-bit words (CS + ID + 61~64 bytes data).*/
    kFLEXCAN_19WordPerRead        /* Transfer 19 32-bit words (CS + ID + 64 bytes data + ID HIT).*/
} flexcan_efifo_dma_per_read_length_t;
#endif

/*
 * FlexCAN interrupt enable enumerations.
 *
 * This provides constants for the FlexCAN interrupt enable enumerations for use in the FlexCAN functions.
 * Note: FlexCAN Message Buffers and Legacy Rx FIFO interrupts not included in.
 */
enum _flexcan_interrupt_enable
{
    kFLEXCAN_BusOffInterruptEnable      = CAN_CTRL1_BOFFMSK_MASK,       /* Bus Off interrupt, use bit 15. */
    kFLEXCAN_ErrorInterruptEnable       = CAN_CTRL1_ERRMSK_MASK,        /* CAN Error interrupt, use bit 14. */
    kFLEXCAN_TxWarningInterruptEnable   = CAN_CTRL1_TWRNMSK_MASK,       /* Tx Warning interrupt, use bit 11. */
    kFLEXCAN_RxWarningInterruptEnable   = CAN_CTRL1_RWRNMSK_MASK,       /* Rx Warning interrupt, use bit 10. */
    kFLEXCAN_WakeUpInterruptEnable      = CAN_MCR_WAKMSK_MASK,          /* Self Wake Up interrupt, use bit 22. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    kFLEXCAN_FDErrorInterruptEnable     = CAN_CTRL2_ERRMSK_FAST_MASK,   /* CAN FD Error interrupt, use bit 31. */
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    /* PN Match Wake Up interrupt, use high word bit 17. */
    kFLEXCAN_PNMatchWakeUpInterruptEnable    = FLEXCAN_PN_INT_MASK(CAN_CTRL1_PN_WTOF_MSK_MASK),
    /* PN Timeout Wake Up interrupt, use high word bit 16. */
    kFLEXCAN_PNTimeoutWakeUpInterruptEnable  = FLEXCAN_PN_INT_MASK(CAN_CTRL1_PN_WUMF_MSK_MASK),
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
    /* Enhanced Rx FIFO Underflow interrupt, use high word bit 31. */
    kFLEXCAN_ERxFifoUnderflowInterruptEnable = FLEXCAN_EFIFO_INT_MASK(CAN_ERFIER_ERFUFWIE_MASK),
    /* Enhanced Rx FIFO Overflow interrupt, use high word bit 30. */
    kFLEXCAN_ERxFifoOverflowInterruptEnable  = FLEXCAN_EFIFO_INT_MASK(CAN_ERFIER_ERFOVFIE_MASK),
    /* Enhanced Rx FIFO Watermark interrupt, use high word bit 29. */
    kFLEXCAN_ERxFifoWatermarkInterruptEnable = FLEXCAN_EFIFO_INT_MASK(CAN_ERFIER_ERFWMIIE_MASK),
    /* Enhanced Rx FIFO Data Avilable interrupt, use high word bit 28. */
    kFLEXCAN_ERxFifoDataAvlInterruptEnable   = FLEXCAN_EFIFO_INT_MASK(CAN_ERFIER_ERFDAIE_MASK),
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
#if defined(COMPILER_COMPATIBILITY)
    /* Host Access With Non-Correctable Errors interrupt, use high word bit 0. */
    kFLEXCAN_HostAccessNCErrorInterruptEnable    = FLEXCAN_MECR_INT_MASK(CAN_MECR_HANCEI_MSK_MASK),
    /* FlexCAN Access With Non-Correctable Errors interrupt, use high word bit 2. */
    kFLEXCAN_FlexCanAccessNCErrorInterruptEnable = FLEXCAN_MECR_INT_MASK(CAN_MECR_FANCEI_MSK_MASK),
    /* Host or FlexCAN Access With Correctable Errors interrupt, use high word bit 3. */
    kFLEXCAN_HostOrFlexCanCErrorInterruptEnable  = FLEXCAN_MECR_INT_MASK(CAN_MECR_CEI_MSK_MASK),
#else
    #define kFLEXCAN_HostAccessNCErrorInterruptEnable FLEXCAN_MECR_INT_MASK(CAN_MECR_HANCEI_MSK_MASK)
    #define kFLEXCAN_FlexCanAccessNCErrorInterruptEnable FLEXCAN_MECR_INT_MASK(CAN_MECR_FANCEI_MSK_MASK)
    #define kFLEXCAN_HostOrFlexCanCErrorInterruptEnable FLEXCAN_MECR_INT_MASK(CAN_MECR_CEI_MSK_MASK)
    #endif
#endif
};

/*
 * FlexCAN status flags.
 *
 * This provides constants for the FlexCAN status flags for use in the FlexCAN functions.
 * Note: The CPU read action clears the bits corresponding to the FlEXCAN_ErrorFlag macro, therefore user need to
 * read status flags and distinguish which error is occur using _flexcan_error_flags enumerations.
 */
enum _flexcan_flags
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    kFLEXCAN_ErrorOverrunFlag       = CAN_ESR1_ERROVR_MASK,      /* Error Overrun Status. */
    kFLEXCAN_FDErrorIntFlag         = CAN_ESR1_ERRINT_FAST_MASK, /* CAN FD Error Interrupt Flag. */
    kFLEXCAN_BusoffDoneIntFlag      = CAN_ESR1_BOFFDONEINT_MASK, /* Bus Off process completed Interrupt Flag. */
#endif
    kFLEXCAN_SynchFlag              = CAN_ESR1_SYNCH_MASK,   /* CAN Synchronization Status. */
    kFLEXCAN_TxWarningIntFlag       = CAN_ESR1_TWRNINT_MASK, /* Tx Warning Interrupt Flag. */
    kFLEXCAN_RxWarningIntFlag       = CAN_ESR1_RWRNINT_MASK, /* Rx Warning Interrupt Flag. */
    kFLEXCAN_IdleFlag               = CAN_ESR1_IDLE_MASK,    /* FlexCAN In IDLE Status. */
    kFLEXCAN_FaultConfinementFlag   = CAN_ESR1_FLTCONF_MASK, /* FlexCAN Fault Confinement State. */
    kFLEXCAN_TransmittingFlag       = CAN_ESR1_TX_MASK,      /* FlexCAN In Transmission Status. */
    kFLEXCAN_ReceivingFlag          = CAN_ESR1_RX_MASK,      /* FlexCAN In Reception Status. */
    kFLEXCAN_BusOffIntFlag          = CAN_ESR1_BOFFINT_MASK, /* Bus Off Interrupt Flag. */
    kFLEXCAN_ErrorIntFlag           = CAN_ESR1_ERRINT_MASK,  /* CAN Error Interrupt Flag. */
    kFLEXCAN_WakeUpIntFlag          = CAN_ESR1_WAKINT_MASK,  /* Self Wake-Up Interrupt Flag. */
    kFLEXCAN_ErrorFlag =
    (UINT32)(/* All FlexCAN Read Clear Error Status. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
        CAN_ESR1_STFERR_FAST_MASK | CAN_ESR1_FRMERR_FAST_MASK | CAN_ESR1_CRCERR_FAST_MASK |
        CAN_ESR1_BIT0ERR_FAST_MASK | CAN_ESR1_BIT1ERR_FAST_MASK | CAN_ESR1_ERROVR_MASK |
#endif
        CAN_ESR1_TXWRN_MASK | CAN_ESR1_RXWRN_MASK | CAN_ESR1_BIT1ERR_MASK | CAN_ESR1_BIT0ERR_MASK |
        CAN_ESR1_ACKERR_MASK | CAN_ESR1_CRCERR_MASK | CAN_ESR1_FRMERR_MASK | CAN_ESR1_STFERR_MASK),
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    kFLEXCAN_PNMatchIntFlag = FLEXCAN_PN_STATUS_MASK(CAN_WU_MTC_WUMF_MASK), /* PN Matching Event Interrupt Flag. */
    kFLEXCAN_PNTimeoutIntFlag = FLEXCAN_PN_STATUS_MASK(CAN_WU_MTC_WTOF_MASK), /* PN Timeout Event Interrupt Flag. */
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
    kFLEXCAN_ERxFifoUnderflowIntFlag =
    FLEXCAN_EFIFO_STATUS_MASK(CAN_ERFSR_ERFUFW_MASK), /* Enhanced Rx FIFO underflow Interrupt Flag. */
    kFLEXCAN_ERxFifoOverflowIntFlag =
    FLEXCAN_EFIFO_STATUS_MASK(CAN_ERFSR_ERFOVF_MASK), /* Enhanced Rx FIFO overflow Interrupt Flag. */
    kFLEXCAN_ERxFifoWatermarkIntFlag =
    FLEXCAN_EFIFO_STATUS_MASK(CAN_ERFSR_ERFWMI_MASK), /* Enhanced Rx FIFO watermark Interrupt Flag. */
    kFLEXCAN_ERxFifoDataAvlIntFlag =
    FLEXCAN_EFIFO_STATUS_MASK(CAN_ERFSR_ERFDA_MASK), /* Enhanced Rx FIFO data available Interrupt Flag. */
    kFLEXCAN_ERxFifoEmptyFlag = FLEXCAN_EFIFO_STATUS_MASK(CAN_ERFSR_ERFE_MASK), /* Enhanced Rx FIFO empty status. */
    kFLEXCAN_ERxFifoFullFlag = FLEXCAN_EFIFO_STATUS_MASK(CAN_ERFSR_ERFF_MASK), /* Enhanced Rx FIFO full status. */
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
#if defined(COMPILER_COMPATIBILITY)
    /* Host Access With Non-Correctable Error Interrupt Flag. */
    kFLEXCAN_HostAccessNonCorrectableErrorIntFlag = FLEXCAN_MECR_INT_MASK(CAN_ERRSR_HANCEIF_MASK),
    /* FlexCAN Access With Non-Correctable Error Interrupt Flag. */
    kFLEXCAN_FlexCanAccessNonCorrectableErrorIntFlag = FLEXCAN_MECR_INT_MASK(CAN_ERRSR_FANCEIF_MASK),
    /* Correctable Error Interrupt Flag. */
    kFLEXCAN_CorrectableErrorIntFlag = FLEXCAN_MECR_INT_MASK(CAN_ERRSR_CEIF_MASK),
    /* Host Access With Non-Correctable Error Interrupt Overrun Flag. */
    kFLEXCAN_HostAccessNonCorrectableErrorOverrunFlag = FLEXCAN_MECR_INT_MASK(CAN_ERRSR_HANCEIOF_MASK),
    /* FlexCAN Access With Non-Correctable Error Interrupt Overrun Flag. */
    kFLEXCAN_FlexCanAccessNonCorrectableErrorOverrunFlag = FLEXCAN_MECR_INT_MASK(CAN_ERRSR_FANCEIOF_MASK),
    /* Correctable Error Interrupt Overrun Flag. */
    kFLEXCAN_CorrectableErrorOverrunFlag = FLEXCAN_MECR_INT_MASK(CAN_ERRSR_CEIOF_MASK),
    /* All Memory Error Flags. */
    kFLEXCAN_AllMemoryErrorFlag =
    (kFLEXCAN_HostAccessNonCorrectableErrorIntFlag | kFLEXCAN_FlexCanAccessNonCorrectableErrorIntFlag |
        kFLEXCAN_CorrectableErrorIntFlag | kFLEXCAN_HostAccessNonCorrectableErrorOverrunFlag |
        kFLEXCAN_FlexCanAccessNonCorrectableErrorOverrunFlag | kFLEXCAN_CorrectableErrorOverrunFlag)
#else
    #define kFLEXCAN_HostAccessNonCorrectableErrorIntFlag FLEXCAN_MECR_INT_MASK(CAN_ERRSR_HANCEIF_MASK)
    #define kFLEXCAN_FlexCanAccessNonCorrectableErrorIntFlag FLEXCAN_MECR_INT_MASK(CAN_ERRSR_FANCEIF_MASK)
    #define kFLEXCAN_CorrectableErrorIntFlag FLEXCAN_MECR_INT_MASK(CAN_ERRSR_CEIF_MASK)
    #define kFLEXCAN_HostAccessNonCorrectableErrorOverrunFlag FLEXCAN_MECR_INT_MASK(CAN_ERRSR_HANCEIOF_MASK)
    #define kFLEXCAN_FlexCanAccessNonCorrectableErrorOverrunFlag FLEXCAN_MECR_INT_MASK(CAN_ERRSR_FANCEIOF_MASK)
    #define kFLEXCAN_CorrectableErrorOverrunFlag FLEXCAN_MECR_INT_MASK(CAN_ERRSR_CEIOF_MASK)
    #define kFLEXCAN_AllMemoryErrorFlag (kFLEXCAN_HostAccessNonCorrectableErrorIntFlag | kFLEXCAN_FlexCanAccessNonCorrectableErrorIntFlag | kFLEXCAN_CorrectableErrorIntFlag | kFLEXCAN_HostAccessNonCorrectableErrorOverrunFlag | kFLEXCAN_FlexCanAccessNonCorrectableErrorOverrunFlag | kFLEXCAN_CorrectableErrorOverrunFlag)
    #endif
#endif
};

/*
 * FlexCAN error status flags.
 *
 * The FlexCAN Error Status enumerations is used to report current error of the FlexCAN bus.
 * This enumerations should be used with KFLEXCAN_ErrorFlag in @ref _flexcan_flags enumerations
 * to ditermine which error is generated.
 */
enum _flexcan_error_flags
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    kFLEXCAN_FDStuffingError    = CAN_ESR1_STFERR_FAST_MASK,        /* Stuffing Error. */
    kFLEXCAN_FDFormError        = CAN_ESR1_FRMERR_FAST_MASK,        /* Form Error. */
    kFLEXCAN_FDCrcError         = CAN_ESR1_CRCERR_FAST_MASK,        /* Cyclic Redundancy Check Error. */
    kFLEXCAN_FDBit0Error        = CAN_ESR1_BIT0ERR_FAST_MASK,       /* Unable to send dominant bit. */
    kFLEXCAN_FDBit1Error        = (int)CAN_ESR1_BIT1ERR_FAST_MASK,  /* Unable to send recessive bit. */
#endif
    kFLEXCAN_TxErrorWarningFlag = CAN_ESR1_TXWRN_MASK,              /* Tx Error Warning Status. */
    kFLEXCAN_RxErrorWarningFlag = CAN_ESR1_RXWRN_MASK,              /* Rx Error Warning Status. */
    kFLEXCAN_StuffingError      = CAN_ESR1_STFERR_MASK,             /* Stuffing Error. */
    kFLEXCAN_FormError          = CAN_ESR1_FRMERR_MASK,             /* Form Error. */
    kFLEXCAN_CrcError           = CAN_ESR1_CRCERR_MASK,             /* Cyclic Redundancy Check Error. */
    kFLEXCAN_AckError           = CAN_ESR1_ACKERR_MASK,             /* Received no ACK on transmission. */
    kFLEXCAN_Bit0Error          = CAN_ESR1_BIT0ERR_MASK,            /* Unable to send dominant bit. */
    kFLEXCAN_Bit1Error          = CAN_ESR1_BIT1ERR_MASK,            /* Unable to send recessive bit. */
};

/*
 * FlexCAN Legacy Rx FIFO status flags.
 *
 * The FlexCAN Legacy Rx FIFO Status enumerations are used to determine the status of the
 * Rx FIFO. Because Rx FIFO occupy the MB0 ~ MB7 (Rx Fifo filter also occupies
 * more Message Buffer space), Rx FIFO status flags are mapped to the corresponding
 * Message Buffer status flags.
 */
enum
{
    kFLEXCAN_RxFifoOverflowFlag = CAN_IFLAG1_BUF7I_MASK, /* Rx FIFO overflow flag. */
    kFLEXCAN_RxFifoWarningFlag = CAN_IFLAG1_BUF6I_MASK, /* Rx FIFO almost full flag. */
    kFLEXCAN_RxFifoFrameAvlFlag = CAN_IFLAG1_BUF5I_MASK, /* Frames available in Rx FIFO flag. */
};

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/* FlexCAN Enhanced Rx FIFO Standard ID filter element structure. */
typedef struct _flexcan_enhanced_rx_fifo_std_id_filter
{
    UINT32 filterType : 2; /* FlexCAN internal Free-Running Counter Time Stamp. */
    UINT32 : 2;
               UINT32 rtr1 : 1;  /* CAN FD frame data length code (DLC), range see @ref _flexcan_fd_frame_length, When the
                                        length <= 8, it equal to the data length, otherwise the number of valid frame data is
                                        not equal to the length value.  user can
                                        use DLC_LENGTH_DECODE(length) macro to get the number of valid data bytes. */
               UINT32 std1 : 11; /* CAN Frame Type(DATA or REMOTE). */
               UINT32 : 4;
                          UINT32 rtr2 : 1;  /* CAN Frame Identifier(STD or EXT format). */
                          UINT32 std2 : 11; /* Substitute Remote request. */
} flexcan_enhanced_rx_fifo_std_id_filter_t;

/* FlexCAN Enhanced Rx FIFO Extended ID filter element structure. */
typedef struct _flexcan_enhanced_rx_fifo_ext_id_filter
{
    UINT32 filterType : 2; /* FlexCAN internal Free-Running Counter Time Stamp. */
    UINT32 rtr1 : 1;       /* CAN FD frame data length code (DLC), range see @ref _flexcan_fd_frame_length, When the
                                  length <= 8, it equal to the data length, otherwise the number of valid frame data is
                                  not equal to the length value.  user can
                                  use DLC_LENGTH_DECODE(length) macro to get the number of valid data bytes. */
    UINT32 std1 : 29;      /* CAN Frame Type(DATA or REMOTE). */
    UINT32 : 2;
               UINT32 rtr2 : 1;  /* CAN Frame Identifier(STD or EXT format). */
               UINT32 std2 : 29; /* Substitute Remote request. */
} flexcan_enhanced_rx_fifo_ext_id_filter_t;
/* FlexCAN Enhanced Rx FIFO configuration structure. */
typedef struct _flexcan_enhanced_rx_fifo_config
{
    UINT32 *idFilterTable; /* Pointer to the FlexCAN Enhanced Rx FIFO identifier filter table, each table member
                             occupies 32 bit word, table size should be equal to idFilterNum. There are two types of
                           Enhanced Rx FIFO filter elements that can be stored in table : extended-ID filter element
                           (1 word, occupie 1 table members) and standard-ID filter element (2 words, occupies 2 table
                           members), the extended-ID filter element needs to be placed in front of the table. */
    UINT8 idFilterPairNum; /* (idFilterPairNum + 1) is the Enhanced Rx FIFO identifier filter element pair numbers,
                                each pair of filter elements occupies 2 words and can consist of one extended ID filter
                                element or two standard ID filter elements. */
    UINT8 extendIdFilterNum; /* The number of extended ID filter element items in the FlexCAN enhanced Rx FIFO
                                  identifier filter table, each extended-ID filter element occupies 2 words,
                                  extendIdFilterNum need less than or equal to (idFilterPairNum + 1). */
    UINT8 fifoWatermark; /* (fifoWatermark + 1) is the minimum number of CAN messages stored in the Enhanced RX FIFO
                              which can trigger FIFO watermark interrupt or a DMA request. */
    flexcan_efifo_dma_per_read_length_t dmaPerReadLength; /* Define the length of each read of the Enhanced RX FIFO
                                                             element by the DAM, see @ref _flexcan_fd_frame_length. */
    flexcan_rx_fifo_priority_t priority;                  /* The FlexCAN Enhanced Rx FIFO receive priority. */
} flexcan_enhanced_rx_fifo_config_t;
#endif

/* FlexCAN Rx FIFO transfer. */
typedef struct _flexcan_fifo_transfer
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
    flexcan_fd_frame_t *framefd; /* The buffer of CAN Message to be received from Enhanced Rx FIFO. */
    size_t frameNum;             /* Number of CAN Message need to be received from Ehanced Rx FIFO. */
#endif
    flexcan_frame_t *frame; /* The buffer of CAN Message to be received from Rx FIFO. */
} flexcan_fifo_transfer_t;

/******************************************************************************
 * API
 *****************************************************************************/

   /*
   * Enter FlexCAN Freeze Mode.
   *
   * This function makes the FlexCAN work under Freeze Mode.
   *
   * param base FlexCAN peripheral base address.
   */
void FLEXCAN_EnterFreezeMode(volatile IMXCAN_REGISTERS *base);

/*
 * Exit FlexCAN Freeze Mode.
 *
 * This function makes the FlexCAN leave Freeze Mode.
 *
 * param base FlexCAN peripheral base address.
 */
void FLEXCAN_ExitFreezeMode(volatile IMXCAN_REGISTERS *base);

/*
 * Get the FlexCAN instance from peripheral base address.
 *
 * param base FlexCAN peripheral base address.
 * @return FlexCAN instance.
 
UINT32 FLEXCAN_GetInstance(IMXCAN_REGISTERS *base);
*/

/*
 * Calculates the improved timing values by specific bit Rates for classical CAN.
 *
 * This function use to calculates the Classical CAN timing values according to the given bit rate. The Calculated
 * timing values will be set in CTRL1/CBT/ENCBT register. The calculation is based on the recommendation of the CiA 301
 * v4.2.0 and previous version document.
 *
 * param base FlexCAN peripheral base address.
 * param bitRate  The classical CAN speed in bps defined by user, should be less than or equal to 1Mbps.
 * param sourceClock_Hz The Source clock frequency in Hz.
 * param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 *
 * @return TRUE if timing configuration found, FALSE if failed to find configuration.
 */
BOOLEAN FLEXCAN_CalculateImprovedTimingValues(volatile IMXCAN_REGISTERS *base,
    UINT32 bitRate,
    UINT32 sourceClock_Hz,
    flexcan_timing_config_t *pTimingConfig);

/*
 * Initializes a FlexCAN instance.
 *
 * This function initializes the FlexCAN module with user-defined settings.
 * This example shows how to set up the flexcan_config_t parameters and how
 * to call the FLEXCAN_Init function by passing in these parameters.
 *  @code
 *   flexcan_config_t flexcanConfig;
 *   flexcanConfig.clkSrc               = kFLEXCAN_ClkSrc0;
 *   flexcanConfig.bitRate              = 1000000U;
 *   flexcanConfig.maxMbNum             = 16;
 *   flexcanConfig.enableLoopBack       = false;
 *   flexcanConfig.enableSelfWakeup     = false;
 *   flexcanConfig.enableIndividMask    = false;
 *   flexcanConfig.enableDoze           = false;
 *   flexcanConfig.disableSelfReception = false;
 *   flexcanConfig.enableListenOnlyMode = false;
 *   flexcanConfig.timingConfig         = timingConfig;
 *   FLEXCAN_Init(CAN0, &flexcanConfig, 40000000UL);
 *   @endcode
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the user-defined configuration structure.
 * param sourceClock_Hz FlexCAN Protocol Engine clock source frequency in Hz.
 */
void FLEXCAN_Init(volatile IMXCAN_REGISTERS *base, const flexcan_config_t *pConfig, UINT32 sourceClock_Hz);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Calculates the improved timing values by specific bit rates for CANFD.
 *
 * This function use to calculates the CANFD timing values according to the given nominal phase bit rate and data phase
 * bit rate. The Calculated timing values will be set in CBT/ENCBT and FDCBT/EDCBT registers. The calculation is based
 * on the recommendation of the CiA 1301 v1.0.0 document.
 *
 * param base FlexCAN peripheral base address.
 * param bitRate  The CANFD bus control speed in bps defined by user.
 * param bitRateFD  The CAN FD data phase speed in bps defined by user. Equal to bitRate means disable bit rate
 * switching.
 * param sourceClock_Hz The Source clock frequency in Hz.
 * param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 *
 * @return TRUE if timing configuration found, FALSE if failed to find configuration
 */
BOOLEAN FLEXCAN_FDCalculateImprovedTimingValues(volatile IMXCAN_REGISTERS *base,
    UINT32 bitRate,
    UINT32 bitRateFD,
    UINT32 sourceClock_Hz,
    flexcan_timing_config_t *pTimingConfig);
/*
 * Initializes a FlexCAN instance.
 *
 * This function initializes the FlexCAN module with user-defined settings.
 * This example shows how to set up the flexcan_config_t parameters and how
 * to call the FLEXCAN_FDInit function by passing in these parameters.
 *  @code
 *   flexcan_config_t flexcanConfig;
 *   flexcanConfig.clkSrc               = kFLEXCAN_ClkSrc0;
 *   flexcanConfig.bitRate              = 1000000U;
 *   flexcanConfig.bitRateFD            = 2000000U;
 *   flexcanConfig.maxMbNum             = 16;
 *   flexcanConfig.enableLoopBack       = false;
 *   flexcanConfig.enableSelfWakeup     = false;
 *   flexcanConfig.enableIndividMask    = false;
 *   flexcanConfig.disableSelfReception = false;
 *   flexcanConfig.enableListenOnlyMode = false;
 *   flexcanConfig.enableDoze           = false;
 *   flexcanConfig.timingConfig         = timingConfig;
 *   FLEXCAN_FDInit(CAN0, &flexcanConfig, 80000000UL, kFLEXCAN_16BperMB, true);
 *   @endcode
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the user-defined configuration structure.
 * param sourceClock_Hz FlexCAN Protocol Engine clock source frequency in Hz.
 * param dataSize FlexCAN Message Buffer payload size. The actual transmitted or received CAN FD frame data size needs
 *                to be less than or equal to this value.
 * param brs True if bit rate switch is enabled in FD mode.
 */
void FLEXCAN_FDInit(volatile IMXCAN_REGISTERS *base, const flexcan_config_t *pConfig, UINT32 sourceClock_Hz, flexcan_mb_size_t dataSize, BOOLEAN brs);
#endif

/*
 * De-initializes a FlexCAN instance.
 *
 * This function disables the FlexCAN module clock and sets all register values
 * to the reset value.
 *
 * param base FlexCAN peripheral base address.
 */
void FLEXCAN_Deinit(volatile IMXCAN_REGISTERS *base);

/*
 * Gets the default configuration structure.
 *
 * This function initializes the FlexCAN configuration structure to default values. The default
 * values are as follows.
 *   flexcanConfig->clkSrc                               = kFLEXCAN_ClkSrc0;
 *   flexcanConfig->bitRate                              = 1000000U;
 *   flexcanConfig->bitRateFD                            = 2000000U;
 *   flexcanConfig->maxMbNum                             = 16;
 *   flexcanConfig->enableLoopBack                       = false;
 *   flexcanConfig->enableSelfWakeup                     = false;
 *   flexcanConfig->enableIndividMask                    = false;
 *   flexcanConfig->disableSelfReception                 = false;
 *   flexcanConfig->enableListenOnlyMode                 = false;
 *   flexcanConfig->enableDoze                           = false;
 *   flexcanConfig->enableMemoryErrorControl             = true;
 *   flexcanConfig->enableNonCorrectableErrorEnterFreeze = true;
 *   flexcanConfig.timingConfig                          = timingConfig;
 *
 * param pConfig Pointer to the FlexCAN configuration structure.
 */
void FLEXCAN_GetDefaultConfig(flexcan_config_t *pConfig);

 /*
  * Sets the FlexCAN protocol timing characteristic.
  *
  * This function gives user settings to classical CAN or CANFD nominal phase timing characteristic.
  * The function is for an experienced user. For less experienced users, call the FLEXCAN_GetDefaultConfig()
  * and get the default timing characteristicsthe, then call FLEXCAN_Init() and fill the
  * bit rate field.
  *
  * Note: Calling FLEXCAN_SetTimingConfig() overrides the bit rate set in FLEXCAN_Init().
  *
  * param base FlexCAN peripheral base address.
  * param pConfig Pointer to the timing configuration structure.
  */
void FLEXCAN_SetTimingConfig(volatile IMXCAN_REGISTERS *base, const flexcan_timing_config_t *pConfig);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Sets the FlexCAN CANFD data phase timing characteristic.
 *
 * This function gives user settings to CANFD data phase timing characteristic.
 * The function is for an experienced user. For less experienced users, call the FLEXCAN_GetDefaultConfig()
 * and get the default timing characteristicsthe, then call FLEXCAN_FDInit() and fill the
 * data phase bit rate field.
 *
 * Note: Calling FLEXCAN_SetFDTimingConfig() overrides the bit rate set in FLEXCAN_FDInit().
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the timing configuration structure.
 */
void FLEXCAN_SetFDTimingConfig(volatile IMXCAN_REGISTERS *base, const flexcan_timing_config_t *pConfig);
#endif

/*
 * Sets the FlexCAN receive message buffer global mask.
 *
 * This function sets the global mask for the FlexCAN message buffer in a matching process.
 * The configuration is only effective when the Rx individual mask is disabled in the FLEXCAN_Init().
 *
 * param base FlexCAN peripheral base address.
 * param mask Rx Message Buffer Global Mask value.
 */
void FLEXCAN_SetRxMbGlobalMask(volatile IMXCAN_REGISTERS *base, UINT32 mask);

/*
 * Sets the FlexCAN receive FIFO global mask.
 *
 * This function sets the global mask for FlexCAN FIFO in a matching process.
 *
 * param base FlexCAN peripheral base address.
 * param mask Rx Fifo Global Mask value.
 */
void FLEXCAN_SetRxFifoGlobalMask(volatile IMXCAN_REGISTERS *base, UINT32 mask);

/*
 * Sets the FlexCAN receive individual mask.
 *
 * This function sets the individual mask for the FlexCAN matching process.
 * The configuration is only effective when the Rx individual mask is enabled in the FLEXCAN_Init().
 * If the Rx FIFO is disabled, the individual mask is applied to the corresponding Message Buffer.
 * If the Rx FIFO is enabled, the individual mask for Rx FIFO occupied Message Buffer is applied to
 * the Rx Filter with the same index. Note that only the first 32
 * individual masks can be used as the Rx FIFO filter mask.
 *
 * param base FlexCAN peripheral base address.
 * param maskIdx The Index of individual Mask.
 * param mask Rx Individual Mask value.
 */
void FLEXCAN_SetRxIndividualMask(volatile IMXCAN_REGISTERS *base, UINT8 maskIdx, UINT32 mask);

/*
 * Configures a FlexCAN transmit message buffer.
 *
 * This function aborts the previous transmission, cleans the Message Buffer, and
 * configures it as a Transmit Message Buffer.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param enable Enable/disable Tx Message Buffer.
 *               - true: Enable Tx Message Buffer.
 *               - false: Disable Tx Message Buffer.
 */
void FLEXCAN_SetTxMbConfig(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, BOOLEAN enable);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Configures a FlexCAN transmit message buffer.
 *
 * This function aborts the previous transmission, cleans the Message Buffer, and
 * configures it as a Transmit Message Buffer.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param enable Enable/disable Tx Message Buffer.
 *               - true: Enable Tx Message Buffer.
 *               - false: Disable Tx Message Buffer.
 */
void FLEXCAN_SetFDTxMbConfig(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, BOOLEAN enable);
#endif

/*
 * Configures a FlexCAN Receive Message Buffer.
 *
 * This function cleans a FlexCAN build-in Message Buffer and configures it
 * as a Receive Message Buffer.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param pRxMbConfig Pointer to the FlexCAN Message Buffer configuration structure.
 * param enable Enable/disable Rx Message Buffer.
 *               - true: Enable Rx Message Buffer.
 *               - false: Disable Rx Message Buffer.
 */
void FLEXCAN_SetRxMbConfig(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, BOOLEAN enable);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Configures a FlexCAN Receive Message Buffer.
 *
 * This function cleans a FlexCAN build-in Message Buffer and configures it
 * as a Receive Message Buffer.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param pRxMbConfig Pointer to the FlexCAN Message Buffer configuration structure.
 * param enable Enable/disable Rx Message Buffer.
 *               - true: Enable Rx Message Buffer.
 *               - false: Disable Rx Message Buffer.
 */
void FLEXCAN_SetFDRxMbConfig(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, BOOLEAN enable);
#endif

/*
 * Configures the FlexCAN Legacy Rx FIFO.
 *
 * This function configures the FlexCAN Rx FIFO with given configuration.
 * Note: Legacy Rx FIFO only can receive classic CAN message.
 *
 * param base FlexCAN peripheral base address.
 * param pRxFifoConfig Pointer to the FlexCAN Legacy Rx FIFO configuration structure. Can be NULL when enable parameter
 *                      is false.
 * param enable Enable/disable Legacy Rx FIFO.
 *               - true: Enable Legacy Rx FIFO.
 *               - false: Disable Legacy Rx FIFO.
 */
void FLEXCAN_SetRxFifoConfig(volatile IMXCAN_REGISTERS *base, const flexcan_rx_fifo_config_t *pRxFifoConfig, BOOLEAN enable);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Configures the FlexCAN Enhanced Rx FIFO.
 *
 * This function configures the Enhanced Rx FIFO with given configuration.
 * Note: Enhanced Rx FIFO support receive classic CAN or CAN FD messages, Legacy Rx FIFO and Enhanced Rx FIFO
 *       cannot be enabled at the same time.
 *
 * param base    FlexCAN peripheral base address.
 * param pConfig Pointer to the FlexCAN Enhanced Rx FIFO configuration structure. Can be NULL when enable parameter
 *                is false.
 * param enable  Enable/disable Enhanced Rx FIFO.
 *                - true: Enable Enhanced Rx FIFO.
 *                - false: Disable Enhanced Rx FIFO.
 */
void FLEXCAN_SetEnhancedRxFifoConfig(volatile IMXCAN_REGISTERS *base, const flexcan_enhanced_rx_fifo_config_t *pConfig, BOOLEAN enable);
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
/*
 * Configures the FlexCAN Pretended Networking mode.
 *
 * This function configures the FlexCAN Pretended Networking mode with given configuration.
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the FlexCAN Rx FIFO configuration structure.
 */
void FLEXCAN_SetPNConfig(volatile IMXCAN_REGISTERS *base, const flexcan_pn_config_t *pConfig);
#endif

 /*
  * Gets the FlexCAN module interrupt flags.
  *
  * This function gets all FlexCAN status flags. The flags are returned as the logical
  * OR value of the enumerators @ref _flexcan_flags. To check the specific status,
  * compare the return value with enumerators in @ref _flexcan_flags.
  *
  * param base FlexCAN peripheral base address.
  * @return FlexCAN status flags which are ORed by the enumerators in the _flexcan_flags.
  */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) ||                   \
    (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
inline UINT64 FLEXCAN_GetStatusFlags(volatile IMXCAN_REGISTERS *base)
{
    UINT64 tempflag = (UINT64)base->ESR1;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    /* Get PN Wake Up status. */
    tempflag |= FLEXCAN_PN_STATUS_MASK(base->WU_MTC);
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
    /* Get Enhanced Rx FIFO status. */
    tempflag |= FLEXCAN_EFIFO_STATUS_MASK(base->ERFSR);
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    /* Get Memory Error status. */
    tempflag |= FLEXCAN_MECR_STATUS_MASK(base->ERRSR);
#endif
    return tempflag;
}
#else
static inline UINT32 FLEXCAN_GetStatusFlags(volatile IMXCAN_REGISTERS *base)
{
    return base->ESR1;
}
#endif

/*
 * Clears status flags with the provided mask.
 *
 * This function clears the FlexCAN status flags with a provided mask. An automatically cleared flag
 * can't be cleared by this function.
 *
 * param base FlexCAN peripheral base address.
 * param mask The status flags to be cleared, it is logical OR value of @ref _flexcan_flags.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) ||                   \
    (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
static inline void FLEXCAN_ClearStatusFlags(volatile IMXCAN_REGISTERS *base, UINT64 mask)
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    /* Clear PN Wake Up status. */
    base->WU_MTC = FLEXCAN_PN_STATUS_UNMASK(mask);
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
    /* Clear Enhanced Rx FIFO status. */
    base->ERFSR |= FLEXCAN_EFIFO_STATUS_UNMASK(mask);
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    /* Clear Memory Error status. */
    base->ERRSR = FLEXCAN_MECR_STATUS_UNMASK(mask);
#endif
    base->ESR1 = (UINT32)(mask & 0xFFFFFFFFU);
}
#else
static inline void FLEXCAN_ClearStatusFlags(volatile IMXCAN_REGISTERS *base, UINT32 mask)
{
    /* Write 1 to clear status flag. */
    base->ESR1 = mask;
}
#endif

/*
 * Gets the FlexCAN Bus Error Counter value.
 *
 * This function gets the FlexCAN Bus Error Counter value for both Tx and
 * Rx direction. These values may be needed in the upper layer error handling.
 *
 * param base FlexCAN peripheral base address.
 * param txErrBuf Buffer to store Tx Error Counter value.
 * param rxErrBuf Buffer to store Rx Error Counter value.
 */
static inline void FLEXCAN_GetBusErrCount(volatile IMXCAN_REGISTERS *base, UINT8 *txErrBuf, UINT8 *rxErrBuf)
{
    if (NULL != txErrBuf)
    {
        *txErrBuf = (UINT8)((base->ECR & CAN_ECR_TXERRCNT_MASK) >> CAN_ECR_TXERRCNT_SHIFT);
    }

    if (NULL != rxErrBuf)
    {
        *rxErrBuf = (UINT8)((base->ECR & CAN_ECR_RXERRCNT_MASK) >> CAN_ECR_RXERRCNT_SHIFT);
    }
}

/*
 * Gets the FlexCAN Message Buffer interrupt flags.
 *
 * This function gets the interrupt flags of a given Message Buffers.
 *
 * param base FlexCAN peripheral base address.
 * param mask The ORed FlexCAN Message Buffer mask.
 * @return The status of given Message Buffers.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
static inline UINT64 FLEXCAN_GetMbStatusFlags(volatile IMXCAN_REGISTERS *base, UINT64 mask)
#else
static inline UINT32 FLEXCAN_GetMbStatusFlags(volatile IMXCAN_REGISTERS *base, UINT32 mask)
#endif
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 tempflag = (UINT64)base->IFLAG1;
    return (tempflag | (((UINT64)base->IFLAG2) << 32)) & mask;
#else
    return (base->IFLAG1 & mask);
#endif
}

/*
 * Clears the FlexCAN Message Buffer interrupt flags.
 *
 * This function clears the interrupt flags of a given Message Buffers.
 *
 * param base FlexCAN peripheral base address.
 * param mask The ORed FlexCAN Message Buffer mask.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
static inline void FLEXCAN_ClearMbStatusFlags(volatile IMXCAN_REGISTERS *base, UINT64 mask)
#else
static inline void FLEXCAN_ClearMbStatusFlags(volatile IMXCAN_REGISTERS *base, UINT32 mask)
#endif
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    base->IFLAG1 = (UINT32)(mask & 0xFFFFFFFFU);
    base->IFLAG2 = (UINT32)(mask >> 32);
#else
    base->IFLAG1 = mask;
#endif
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
/*
 * Gets the FlexCAN Memory Error Report registers status.
 *
 * This function gets the FlexCAN Memory Error Report registers status.
 *
 * param base FlexCAN peripheral base address.
 * param errorStatus Pointer to FlexCAN Memory Error Report registers status structure.
 */
void FLEXCAN_GetMemoryErrorReportStatus(volatile IMXCAN_REGISTERS *base, flexcan_memory_error_report_status_t *errorStatus);
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
/*
 * Gets the FlexCAN Number of Matches when in Pretended Networking.
 *
 * This function gets the number of times a given message has matched the predefined filtering criteria for ID and/or PL
 * before a wakeup event.
 *
 * param base FlexCAN peripheral base address.
 * @return The number of received wake up msessages.
 */
static inline UINT8 FLEXCAN_GetPNMatchCount(volatile IMXCAN_REGISTERS *base)
{
    return (UINT8)((base->WU_MTC & CAN_WU_MTC_MCOUNTER_MASK) >> CAN_WU_MTC_MCOUNTER_SHIFT);
}
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Gets the number of FlexCAN Enhanced Rx FIFO available frames.
 *
 * This function gets the number of CAN messages stored in the Enhanced Rx FIFO.
 *
 * param base FlexCAN peripheral base address.
 * @return The number of available CAN messages stored in the Enhanced Rx FIFO.
 */
static inline UINT32 FLEXCAN_GetEnhancedFifoDataCount(volatile IMXCAN_REGISTERS *base)
{
    return (base->ERFSR & CAN_ERFSR_ERFEL_MASK);
}
#endif

#pragma warning(push)
#pragma warning(disable:4127) /* conditional expression is constant */
 /*
  * Enables FlexCAN interrupts according to the provided mask.
  *
  * This function enables the FlexCAN interrupts according to the provided mask. The mask
  * is a logical OR of enumeration members, see @ref _flexcan_interrupt_enable.
  *
  * param base FlexCAN peripheral base address.
  * param mask The interrupts to enable. Logical OR of @ref _flexcan_interrupt_enable.
  */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) ||                   \
    (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
static inline void FLEXCAN_EnableInterrupts(volatile IMXCAN_REGISTERS *base, UINT64 mask)
#else
static inline void FLEXCAN_EnableInterrupts(volatile IMXCAN_REGISTERS *base, UINT32 mask)
#endif
{
    /* Solve Self Wake Up interrupt. */
    base->MCR |= (UINT32)(mask & (UINT32)kFLEXCAN_WakeUpInterruptEnable);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base))
    {
        /* Solve CAN FD frames data phase error interrupt. */
        base->CTRL2 |= (UINT32)(mask & (UINT32)kFLEXCAN_FDErrorInterruptEnable);
    }
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    /* Solve PN Wake Up interrupt. */
    base->CTRL1_PN |= FLEXCAN_PN_INT_UNMASK(mask);
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
    /* Solve Enhanced Rx FIFO interrupt. */
    base->ERFIER |= FLEXCAN_EFIFO_INT_UNMASK(mask);
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    /* Solve Memory Error interrupt. */
    base->MECR |= FLEXCAN_MECR_INT_UNMASK(mask);
#endif

    /* Solve interrupt enable bits in CTRL1 register. */
    base->CTRL1 |=
        (UINT32)(mask & ((UINT32)kFLEXCAN_BusOffInterruptEnable | (UINT32)kFLEXCAN_ErrorInterruptEnable |
        (UINT32)kFLEXCAN_RxWarningInterruptEnable | (UINT32)kFLEXCAN_TxWarningInterruptEnable |
            (UINT32)kFLEXCAN_WakeUpInterruptEnable));
}

/*
 * Disables FlexCAN interrupts according to the provided mask.
 *
 * This function disables the FlexCAN interrupts according to the provided mask. The mask
 * is a logical OR of enumeration members, see @ref _flexcan_interrupt_enable.
 *
 * param base FlexCAN peripheral base address.
 * param mask The interrupts to disable. Logical OR of @ref _flexcan_interrupt_enable.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
static inline void FLEXCAN_DisableInterrupts(volatile IMXCAN_REGISTERS *base, UINT64 mask)
#else
static inline void FLEXCAN_DisableInterrupts(volatile IMXCAN_REGISTERS *base, UINT32 mask)
#endif
{
    /* Solve Wake Up Interrupt. */
    base->MCR &= ~(UINT32)(mask & (UINT32)kFLEXCAN_WakeUpInterruptEnable);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base))
    {
        /* Solve CAN FD frames data phase error interrupt. */
        base->CTRL2 &= ~(UINT32)(mask & (UINT32)kFLEXCAN_FDErrorInterruptEnable);
    }
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    /* Solve PN Wake Up Interrupt. */
    base->CTRL1_PN &= ~FLEXCAN_PN_STATUS_UNMASK(mask);
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
    /* Solve Enhanced Rx FIFO interrupt. */
    base->ERFIER &= ~FLEXCAN_EFIFO_INT_UNMASK(mask);
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    /* Solve Memory Error Interrupt. */
    base->MECR &= ~FLEXCAN_MECR_STATUS_UNMASK(mask);
#endif

    /* Solve interrupt enable bits in CTRL1 register. */
    base->CTRL1 &=
        ~(UINT32)(mask & ((UINT32)kFLEXCAN_BusOffInterruptEnable | (UINT32)kFLEXCAN_ErrorInterruptEnable |
        (UINT32)kFLEXCAN_RxWarningInterruptEnable | (UINT32)kFLEXCAN_TxWarningInterruptEnable |
            (UINT32)kFLEXCAN_WakeUpInterruptEnable));
}
#pragma warning(pop)

/*
 * Enables FlexCAN Message Buffer interrupts.
 *
 * This function enables the interrupts of given Message Buffers.
 *
 * param base FlexCAN peripheral base address.
 * param mask The ORed FlexCAN Message Buffer mask.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
static inline void FLEXCAN_EnableMbInterrupts(volatile IMXCAN_REGISTERS *base, UINT64 mask)
#else
static inline void FLEXCAN_EnableMbInterrupts(volatile IMXCAN_REGISTERS *base, UINT32 mask)
#endif
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    base->IMASK1 |= (UINT32)(mask & 0xFFFFFFFFU);
    base->IMASK2 |= (UINT32)(mask >> 32);
#else
    base->IMASK1 |= mask;
#endif
}

/*
 * Disables FlexCAN Message Buffer interrupts.
 *
 * This function disables the interrupts of given Message Buffers.
 *
 * param base FlexCAN peripheral base address.
 * param mask The ORed FlexCAN Message Buffer mask.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
static inline void FLEXCAN_DisableMbInterrupts(volatile IMXCAN_REGISTERS *base, UINT64 mask)
#else
static inline void FLEXCAN_DisableMbInterrupts(volatile IMXCAN_REGISTERS *base, UINT32 mask)
#endif
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    base->IMASK1 &= ~((UINT32)(mask & 0xFFFFFFFFU));
    base->IMASK2 &= ~((UINT32)(mask >> 32));
#else
    base->IMASK1 &= ~mask;
#endif
}


#if (defined(FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA) && FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA)
/*
 * Enables or disables the FlexCAN Legacy/Enhanced Rx FIFO DMA request.
 *
 * This function enables or disables the DMA feature of FlexCAN build-in Rx FIFO.
 *
 * param base FlexCAN peripheral base address.
 * param enable TRUE to enable, FALSE to disable.
 */
void FLEXCAN_EnableRxFifoDMA(volatile IMXCAN_REGISTERS *base, BOOLEAN enable);

/*
 * Gets the Rx FIFO Head address.
 *
 * This function returns the FlexCAN Rx FIFO Head address, which is mainly used for the DMA/eDMA use case.
 *
 * param base FlexCAN peripheral base address.
 * return FlexCAN Rx FIFO Head address.
 */
static inline PUINT32 FLEXCAN_GetRxFifoHeadAddr(volatile IMXCAN_REGISTERS *base)
{
    return (PUINT32) &(base->MB[0].CS);
}
#endif /* FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA */

 /*
  * Enables or disables the FlexCAN module operation.
  *
  * This function enables or disables the FlexCAN module.
  *
  * param base FlexCAN base pointer.
  * param enable true to enable, false to disable.
  */
static inline void FLEXCAN_Enable(volatile IMXCAN_REGISTERS *base, BOOLEAN enable)
{
    if (enable)
    {
        base->MCR &= ~CAN_MCR_MDIS_MASK;

        /* Wait FlexCAN exit from low-power mode. */
        while (0U != (base->MCR & CAN_MCR_LPMACK_MASK))
        {
        }
    }
    else
    {
        base->MCR |= CAN_MCR_MDIS_MASK;

        /* Wait FlexCAN enter low-power mode. */
        while (0U == (base->MCR & CAN_MCR_LPMACK_MASK))
        {
        }
    }
}

/*
 * Writes a FlexCAN Message to the Transmit Message Buffer.
 *
 * This function writes a CAN Message to the specified Transmit Message Buffer
 * and changes the Message Buffer state to start CAN Message transmit. After
 * that the function returns immediately.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pTxFrame Pointer to CAN message frame to be sent.
 * @retval kStatus_Success - Write Tx Message Buffer Successfully.
 * @retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_WriteTxMb(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, const flexcan_frame_t *pTxFrame);

/*
 * Reads a FlexCAN Message from Receive Message Buffer.
 *
 * This function reads a CAN message from a specified Receive Message Buffer.
 * The function fills a receive CAN message frame structure with
 * just received data and activates the Message Buffer again.
 * The function returns immediately.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * @retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * @retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_ReadRxMb(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_frame_t *pRxFrame);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Writes a FlexCAN FD Message to the Transmit Message Buffer.
 *
 * This function writes a CAN FD Message to the specified Transmit Message Buffer
 * and changes the Message Buffer state to start CAN FD Message transmit. After
 * that the function returns immediately.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN FD Message Buffer index.
 * param pTxFrame Pointer to CAN FD message frame to be sent.
 * @retval kStatus_Success - Write Tx Message Buffer Successfully.
 * @retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_WriteFDTxMb(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, const flexcan_fd_frame_t *pTxFrame);

/*
 * Reads a FlexCAN FD Message from Receive Message Buffer.
 *
 * This function reads a CAN FD message from a specified Receive Message Buffer.
 * The function fills a receive CAN FD message frame structure with
 * just received data and activates the Message Buffer again.
 * The function returns immediately.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN FD Message Buffer index.
 * param pRxFrame Pointer to CAN FD message frame structure for reception.
 * @retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * @retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * @retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_ReadFDRxMb(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_fd_frame_t *pRxFrame);
#endif

/*
 * Reads a FlexCAN Message from Legacy Rx FIFO.
 *
 * This function reads a CAN message from the FlexCAN Legacy Rx FIFO.
 *
 * param base FlexCAN peripheral base address.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success - Read Message from Rx FIFO successfully.
 * @retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_ReadRxFifo(volatile IMXCAN_REGISTERS *base, flexcan_frame_t *pRxFrame);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Reads a FlexCAN Message from Enhanced Rx FIFO.
 *
 * This function reads a CAN or CAN FD message from the FlexCAN Enhanced Rx FIFO.
 *
 * param  base FlexCAN peripheral base address.
 * param  pRxFrame Pointer to CAN FD message frame structure for reception.
 * @retval kStatus_Success - Read Message from Rx FIFO successfully.
 * @retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_ReadEnhancedRxFifo(volatile IMXCAN_REGISTERS *base, flexcan_fd_frame_t *pRxFrame);
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
/*
 * Reads a FlexCAN Message from Wake Up MB.
 *
 * This function reads a CAN message from the FlexCAN Wake up Message Buffers. There are four Wake up Message Buffers
 * (WMBs) used to store incoming messages in Pretended Networking mode. The WMB index indicates the arrival order. The
 * last message is stored in WMB3.
 *
 * param base FlexCAN peripheral base address.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * param mbIdx The FlexCAN Wake up Message Buffer index. Range in 0x0 ~ 0x3.
 * @retval kStatus_Success - Read Message from Wake up Message Buffer successfully.
 * @retval kStatus_Fail    - Wake up Message Buffer has no valid content.
 */
status_t FLEXCAN_ReadPNWakeUpMB(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_frame_t *pRxFrame);
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
 /*
  * Performs a polling send transaction on the CAN bus.
  *
  * Note:  A transfer handle does not need to be created before calling this API.
  *
  * param base FlexCAN peripheral base pointer.
  * param mbIdx The FlexCAN FD Message Buffer index.
  * param pTxFrame Pointer to CAN FD message frame to be sent.
  * @retval kStatus_Success - Write Tx Message Buffer Successfully.
  * @retval kStatus_Fail    - Tx Message Buffer is currently in use.
  */
status_t FLEXCAN_TransferFDSendBlocking(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_fd_frame_t *pTxFrame);

/*
 * Performs a polling receive transaction on the CAN bus.
 *
 * note  A transfer handle does not need to be created before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param mbIdx The FlexCAN FD Message Buffer index.
 * param pRxFrame Pointer to CAN FD message frame structure for reception.
 * retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_TransferFDReceiveBlocking(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_fd_frame_t *pRxFrame);

/*
 * Sends a message using IRQ.
 *
 * This function sends a message using IRQ. This is a non-blocking function, which returns
 * right away. When messages have been sent out, the send callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pMbXfer FlexCAN FD Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * @retval kStatus_Success        Start Tx Message Buffer sending process successfully.
 * @retval kStatus_Fail           Write Tx Message Buffer failed.
 * @retval kStatus_FLEXCAN_TxBusy Tx Message Buffer is in use.
 */
status_t FLEXCAN_TransferFDSendNonBlocking(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);

/*
 * Receives a message using IRQ.
 *
 * This function receives a message using IRQ. This is non-blocking function, which returns
 * right away. When the message has been received, the receive callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pMbXfer FlexCAN FD Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * @retval kStatus_Success        - Start Rx Message Buffer receiving process successfully.
 * @retval kStatus_FLEXCAN_RxBusy - Rx Message Buffer is in use.
 */
status_t FLEXCAN_TransferFDReceiveNonBlocking(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);

/*
 * Aborts the interrupt driven message send process.
 *
 * This function aborts the interrupt driven message send process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN FD Message Buffer index.
 */
void FLEXCAN_TransferFDAbortSend(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT8 mbIdx);

/*
 * Aborts the interrupt driven message receive process.
 *
 * This function aborts the interrupt driven message receive process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN FD Message Buffer index.
 */
void FLEXCAN_TransferFDAbortReceive(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT8 mbIdx);
#endif

/*
 * Performs a polling send transaction on the CAN bus.
 *
 * Note:  A transfer handle does not need to be created  before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pTxFrame Pointer to CAN message frame to be sent.
 * @retval kStatus_Success - Write Tx Message Buffer Successfully.
 * @retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_TransferSendBlocking(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_frame_t *pTxFrame);

/*
 * Performs a polling receive transaction on the CAN bus.
 *
 * Note:  A transfer handle does not need to be created  before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * @retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * @retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_TransferReceiveBlocking(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_frame_t *pRxFrame);

/*
 * Performs a polling receive transaction from Legacy Rx FIFO on the CAN bus.
 *
 * Note:  A transfer handle does not need to be created before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success - Read Message from Rx FIFO successfully.
 * @retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_TransferReceiveFifoBlocking(volatile IMXCAN_REGISTERS *base, flexcan_frame_t *pRxFrame);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Performs a polling receive transaction from Enhanced Rx FIFO on the CAN bus.
 *
 * Note:  A transfer handle does not need to be created before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param pRxFrame Pointer to CAN FD message frame structure for reception.
 * @retval kStatus_Success - Read Message from Rx FIFO successfully.
 * @retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_TransferReceiveEnhancedFifoBlocking(volatile IMXCAN_REGISTERS *base, flexcan_fd_frame_t *pRxFrame);
#endif

/*
 * Initializes the FlexCAN handle.
 *
 * This function initializes the FlexCAN handle, which can be used for other FlexCAN
 * transactional APIs. Usually, for a specified FlexCAN instance,
 * call this API once to get the initialized handle.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param callback The callback function.
 * param userData The parameter of the callback function.
 */
void FLEXCAN_TransferCreateHandle(volatile IMXCAN_REGISTERS *base,
    flexcan_handle_t *handle,
    /*flexcan_transfer_callback_t callback, */
    void *userData);

/*
 * Sends a message using IRQ.
 *
 * This function sends a message using IRQ. This is a non-blocking function, which returns
 * right away. When messages have been sent out, the send callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pMbXfer FlexCAN Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * @retval kStatus_Success        Start Tx Message Buffer sending process successfully.
 * @retval kStatus_Fail           Write Tx Message Buffer failed.
 * @retval kStatus_FLEXCAN_TxBusy Tx Message Buffer is in use.
 */
status_t FLEXCAN_TransferSendNonBlocking(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);

/*
 * Receives a message using IRQ.
 *
 * This function receives a message using IRQ. This is non-blocking function, which returns
 * right away. When the message has been received, the receive callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pMbXfer FlexCAN Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * @retval kStatus_Success        - Start Rx Message Buffer receiving process successfully.
 * @retval kStatus_FLEXCAN_RxBusy - Rx Message Buffer is in use.
 */
status_t FLEXCAN_TransferReceiveNonBlocking(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);

/*
 * Receives a message from Rx FIFO using IRQ.
 *
 * This function receives a message using IRQ. This is a non-blocking function, which returns
 * right away. When all messages have been received, the receive callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pFifoXfer FlexCAN Rx FIFO transfer structure. See the @ref flexcan_fifo_transfer_t.
 * @retval kStatus_Success            - Start Rx FIFO receiving process successfully.
 * @retval kStatus_FLEXCAN_RxFifoBusy - Rx FIFO is currently in use.
 */
status_t FLEXCAN_TransferReceiveFifoNonBlocking(volatile IMXCAN_REGISTERS *base,
    flexcan_handle_t *handle,
    flexcan_fifo_transfer_t *pFifoXfer);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Receives a message from Enhanced Rx FIFO using IRQ.
 *
 * This function receives a message using IRQ. This is a non-blocking function, which returns
 * right away. When all messages have been received, the receive callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pFifoXfer FlexCAN Rx FIFO transfer structure. See the ref flexcan_fifo_transfer_t.@
 * @retval kStatus_Success            - Start Rx FIFO receiving process successfully.
 * @retval kStatus_FLEXCAN_RxFifoBusy - Rx FIFO is currently in use.
 */
status_t FLEXCAN_TransferReceiveEnhancedFifoNonBlocking(volatile IMXCAN_REGISTERS *base,
    flexcan_handle_t *handle,
    flexcan_fifo_transfer_t *pFifoXfer);

/*
 * Gets the Enhanced Rx Fifo transfer status during a interrupt non-blocking receive.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param count Number of CAN messages receive so far by the non-blocking transaction.
 * @retval kStatus_InvalidArgument count is Invalid.
 * @retval kStatus_Success Successfully return the count.
 */

status_t FLEXCAN_TransferGetReceiveEnhancedFifoCount(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, size_t *count);
#endif

/*
 * Gets the detail index of Mailbox's Timestamp by handle.
 *
 * Then function can only be used when calling non-blocking Data transfer (TX/RX) API,
 * After TX/RX data transfer done (User can get the status by handler's callback function),
 * we can get the detail index of Mailbox's timestamp by handle,
 * Detail non-blocking data transfer API (TX/RX) contain.
 *   -FLEXCAN_TransferSendNonBlocking
 *   -FLEXCAN_TransferFDSendNonBlocking
 *   -FLEXCAN_TransferReceiveNonBlocking
 *   -FLEXCAN_TransferFDReceiveNonBlocking
 *   -FLEXCAN_TransferReceiveFifoNonBlocking
 *
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 * @retval the index of mailbox 's timestamp stored in the handle.
 *
 */
UINT32 FLEXCAN_GetTimeStamp(flexcan_handle_t *handle, UINT8 mbIdx);

/*
 * Aborts the interrupt driven message send process.
 *
 * This function aborts the interrupt driven message send process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 */
void FLEXCAN_TransferAbortSend(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT8 mbIdx);

/*
 * Aborts the interrupt driven message receive process.
 *
 * This function aborts the interrupt driven message receive process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 */
void FLEXCAN_TransferAbortReceive(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT8 mbIdx);

/*
 * Aborts the interrupt driven message receive from Rx FIFO process.
 *
 * This function aborts the interrupt driven message receive from Rx FIFO process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 */
void FLEXCAN_TransferAbortReceiveFifo(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Aborts the interrupt driven message receive from Enhanced Rx FIFO process.
 *
 * This function aborts the interrupt driven message receive from Enhanced Rx FIFO process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 */
void FLEXCAN_TransferAbortReceiveEnhancedFifo(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle);
#endif

/*
 * Check unhandle interrupt events
 *
 * param base FlexCAN peripheral base address.
 * return TRUE if unhandled interrupt action exist, FALSE if no unhandlered interrupt action exist.
 */
_IRQL_requires_same_ BOOLEAN FLEXCAN_CheckUnhandleInterruptEvents(volatile IMXCAN_REGISTERS *base);

/*
 * Sub Handler Data Trasfered Events
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pResult Pointer to the Handle result.
 *
 * return the status after handle each data transfered event.
 */
_IRQL_requires_same_ status_t FLEXCAN_SubHandlerForDataTransfered(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT32 *pResult);


#pragma warning(pop)