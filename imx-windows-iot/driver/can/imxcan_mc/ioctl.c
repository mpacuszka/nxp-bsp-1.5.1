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
#include "precomp.h"
#pragma hdrstop

#include "imxcanhw.h"
#include "imxcan.h"

/* This module contains CAN IOCTLs implementation. */

IMXCAN_NONPAGED_SEGMENT_BEGIN;

VOID ImxCanEvtIoDeviceControl(
    WDFQUEUE    WdfQueue,
    WDFREQUEST  WdfRequest,
    size_t      OutputBufferLength,
    size_t      InputBufferLength,
    ULONG       IoControlCode
)
{
    UNREFERENCED_PARAMETER(OutputBufferLength);
    UNREFERENCED_PARAMETER(InputBufferLength);

    WDFDEVICE wdfDevice = WdfIoQueueGetDevice(WdfQueue);
    WDFFILEOBJECT wdfFileObject = WdfRequestGetFileObject(WdfRequest);
    PDEV_CONTEXT deviceContextPtr = ImxCanGetDeviceContext(wdfDevice);
    IMXCAN_FILE_OBJECT_CONTEXT *fileObjectContextPtr = ImxCanGetFileObjectContext(wdfFileObject);

    if (fileObjectContextPtr->IsPinInterface) {
        /* Is Pin Interface */
        switch (IoControlCode) {
        case IOCTL_CAN_CONTROLLER_GET_INFO:
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Controller IOCTL directed to a pin. (IoControlCode = 0x%x)\n", IoControlCode));
            WdfRequestComplete(WdfRequest, STATUS_INVALID_DEVICE_REQUEST);
            break;
        default:
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "IOCTL not supported. (IoControlCode = 0x%x)\n", IoControlCode));
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
            return;
        }
    }
    else {
        /* Is Controller Interface */
        switch (IoControlCode) {
        case IOCTL_CAN_CONTROLLER_GET_INFO:
            ImxCanIoctlControllerGetInfo(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_GET_CONFIG:
            ImxCanIoctlControllerGetConfig(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_CLKSRC:
            ImxCanIoctlControllerSetClkSrc(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_BITRATE:
            ImxCanIoctlControllerSetBitRate(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_LOOPBACK:
            ImxCanIoctlControllerSetLoopBack(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_SELFWAKEUP:
            ImxCanIoctlControllerSetSelfWakeup(deviceContextPtr, WdfRequest);
            break;
        case CAN_IOCTL_ID_CONTROLLER_SET_LISTEN_ONLY_MODE:
            ImxCanIoctlControllerSetListenOnlyMode(deviceContextPtr, WdfRequest);
            break;
        case CAN_IOCTL_ID_CONTROLLER_SET_DOZE:
            ImxCanIoctlControllerSetDoze(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_IMPROVED_TIMING:
            ImxCanIoctlControllerImprovedTiming(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_FD_IMPROVED_TIMING:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerFDImprovedTiming(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_INIT:
            ImxCanIoctlControllerInit(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_FD_INIT:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerFDInit(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_DEINIT:
            ImxCanIoctlControllerDeinit(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_RXMB_CONFIG:
            ImxCanIoctlControllerSetRxMbConfig(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_FD_RXMB_CONFIG:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerSetFDRxMbConfig(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_SET_TXMB_CONFIG:
            ImxCanIoctlControllerSetTxMbConfig(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_FD_TXMB_CONFIG:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerSetFDTxMbConfig(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_CREATE_HANDLE:
            ImxCanIoctlControllerTransferCreateHandle(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_RECEIVE_NON_BLOCKING:
            ImxCanIoctlControllerReceiveNonBlocking(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_FD_RECEIVE_NON_BLOCKING:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerFDReceiveNonBlocking(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_SEND_NON_BLOCKING:
            ImxCanIoctlControllerSendNonBlocking(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_FD_SEND_NON_BLOCKING:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerFDSendNonBlocking(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_RECEIVE_BLOCKING:
            ImxCanIoctlControllerReceiveBlocking(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SEND_BLOCKING:
            ImxCanIoctlControllerSendBlocking(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_FD_SEND_BLOCKING:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerFDSendBlocking(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_FD_RECEIVE_BLOCKING:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerFDReceiveBlocking(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_ENTER_FREEZEMODE:
            ImxCanIoctlControllerEnterFreezeMode(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_EXIT_FREEZEMODE:
            ImxCanIoctlControllerExitFreezeMode(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_TIMING_CONFIG:
            ImxCanIoctlControllerSetTimingConfig(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_FDTIMING_CONFIG:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerSetFDTimingConfig(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_SET_RXMB_GLOBAL_MASK:
            ImxCanIoctlControllerSetRxMbGlobalMask(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_RXFIFO_GLOBAL_MASK:
            ImxCanIoctlControllerSetRxFifoGlobalMask(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_RX_INDIVIDUAL_MASK:
            ImxCanIoctlControllerSetRxIndividualMask(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_SET_RXFIFO_CONFIG:
            ImxCanIoctlControllerSetRxFifoConfig(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_GET_STATUS_FLAGS:
            ImxCanIoctlControllerGetStatusFlags(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_CLEAR_STATUS_FLAGS:
            ImxCanIoctlControllerClearStatusFlags(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_GET_BUSERR_COUNT:
            ImxCanIoctlControllerGetBusErrCount(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_GET_MBSTATUS_FLAGS:
            ImxCanIoctlControllerGetMbStatusFlags(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_CLEAR_MBSTATUS_FLAGS:
            ImxCanIoctlControllerClearMbStatusFlags(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_GET_MEMORY_ERROR_STATUS:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
            ImxCanIoctlControllerGetMemoryErrorReportStatus(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_ENABLE_INTERRUPTS:
            ImxCanIoctlControllerEnableInterrupts(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_DISABLE_INTERRUPTS:
            ImxCanIoctlControllerDisableInterrupts(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_ENABLE_MB_INTERRUPTS:
            ImxCanIoctlControllerEnableMbInterrupts(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_DISABLE_MB_INTERRUPTS:
            ImxCanIoctlControllerDisableMbInterrupts(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_ENABLE_RXFIFO_DMA:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA) && FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA)
            ImxCanIoctlControllerEnableRxFifoDMA(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_GET_RXFIFO_ADDR:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA) && FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA)
            ImxCanIoctlControllerGetRxFifoHeadAddr(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_ENABLE:
            ImxCanIoctlControllerEnable(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_WRITE_TXMB:
            ImxCanIoctlControllerWriteTxMb(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_READ_RXMB:
            ImxCanIoctlControllerReadRxMb(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_WRITE_FD_TXMB:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerWriteFDTxMb(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
        case IOCTL_CAN_CONTROLLER_READ_FD_RXMB:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerReadFDRxMb(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
        case IOCTL_CAN_CONTROLLER_READ_RXFIFO:
            ImxCanIoctlControllerReadRxFifo(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_FD_ABORT_SEND:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerFDAbortSend(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_FD_ABORT_RECEIVE:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            ImxCanIoctlControllerFDAbortReceive(deviceContextPtr, WdfRequest);
#else
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
#endif
            break;
        case IOCTL_CAN_CONTROLLER_RECEIVE_FIFO_BLOCKING:
            ImxCanIoctlControllerReceiveFifoBlocking(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_RECEIVE_FIFO_NON_BLOCKING:
            ImxCanIoctlControllerReceiveFifoNonBlocking(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_GET_TIMESTAMP:
            ImxCanIoctlControllerGetTimeStamp(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_ABORT_SEND:
            ImxCanIoctlControllerAbortSend(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_ABORT_RECEIVE:
            ImxCanIoctlControllerAbortReceive(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_CONTROLLER_ABORT_RECEIVE_FIFO:
            ImxCanIoctlControllerAbortReceiveFifo(deviceContextPtr, WdfRequest);
            break;
        /* Invert notification */
        case IOCTL_CAN_INVERT_NOTIFICATION:
            ImxCanIoctlControllerNotification(deviceContextPtr, WdfRequest);
            break;
        /* Helper functions */
        case IOCTL_CAN_HELPER_FLEXCAN_ID_STD:
            ImxCanIoctlHelperFLEXCAN_ID_STD(deviceContextPtr, WdfRequest);
            break;
        case IOCTL_CAN_HELPER_FLEXCAN_RX_MB_STD_MASK:
            ImxCanIoctlHelperFLEXCAN_RX_MB_STD_MASK(deviceContextPtr, WdfRequest);
            break;
        default:
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "IOCTL not supported. (IoControlCode = 0x%x)\n", IoControlCode));
            WdfRequestComplete(WdfRequest, STATUS_NOT_SUPPORTED);
            return;
        }
    }
}


/* IOCTL_CAN_CONTROLLER_GET_INFO */
_Use_decl_annotations_
VOID ImxCanIoctlControllerGetInfo(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    CAN_CONTROLLER_GET_INFO_OUTPUT *outputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    RtlCopyMemory(outputBufferPtr, &DeviceContextPtr->ControllerInfo, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}


/* IOCTL_CAN_CONTROLLER_GET_CONFIG */
/* This function initializes the FlexCAN configuration structure to default values */
_Use_decl_annotations_
VOID ImxCanIoctlControllerGetConfig(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    CAN_CONTROLLER_GET_CONFIG_OUTPUT* outputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_GetDefaultConfig(&DeviceContextPtr->CANconfig);
    DeviceContextPtr->ControllerInfo.IsInitialized = FALSE;
    DeviceContextPtr->ControllerInfo.Sequence = 0;
    DeviceContextPtr->ControllerInfo.RequestCnt = 0;
    RtlCopyMemory(outputBufferPtr, &DeviceContextPtr->CANconfig, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_SET_CLKSRC */
/* Clock source for FlexCAN Protocol Engine */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetClkSrc(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    CAN_CONTROLLER_SET_CLKSRC_INTPUT *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    switch (*inputBufferPtr) {
    case kFLEXCAN_ClkSrcOsc:
    case kFLEXCAN_ClkSrcPeri:
        break;
    default:
        WdfRequestComplete(WdfRequest, STATUS_INVALID_PARAMETER);
        return;
    }
    if (DeviceContextPtr->ControllerInfo.IsInitialized == TRUE) {
        WdfRequestComplete(WdfRequest, STATUS_INVALID_DEVICE_STATE);
        return;
    }
    DeviceContextPtr->CANconfig.clkSrc = *inputBufferPtr;
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}
/* IOCTL_CAN_CONTROLLER_SET_BITRATE */
/* Frequency for FlexCAN bit rate in Hz */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetBitRate(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    CAN_CONTROLLER_SET_BITRATE_INTPUT *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    if (inputBufferPtr == 0) {
        WdfRequestComplete(WdfRequest, STATUS_INVALID_PARAMETER);
        return;
    }
    if (DeviceContextPtr->ControllerInfo.IsInitialized == TRUE) {
        WdfRequestComplete(WdfRequest, STATUS_INVALID_DEVICE_STATE);
        return;
    }
    DeviceContextPtr->CANconfig.bitRate = *inputBufferPtr;
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_SET_LOOPBACK */
/* Enable or Disable Loop Back Self Test Mode */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetLoopBack(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    CAN_CONTROLLER_BOOLEAN_INPUT *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    DeviceContextPtr->CANconfig.enableLoopBack = *inputBufferPtr;
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_SET_SELFWAKEUP */
/* Enable or Disable Self Wakeup Mode. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetSelfWakeup(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    CAN_CONTROLLER_BOOLEAN_INPUT *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    DeviceContextPtr->CANconfig.enableSelfWakeup = *inputBufferPtr;
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_SET_LISTEN_ONLY_MODE */
/* Enable or Disable Listen Only Mode */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetListenOnlyMode(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    CAN_CONTROLLER_BOOLEAN_INPUT *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    DeviceContextPtr->CANconfig.enableListenOnlyMode = *inputBufferPtr;
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_SET_DOZE */
/* Enable or Disable Doze Mode */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetDoze(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    CAN_CONTROLLER_BOOLEAN_INPUT *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    DeviceContextPtr->CANconfig.enableDoze = *inputBufferPtr;
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_IMPROVED_TIMING */
/* Calculates the improved timing values by specific bit Rates for classical CAN. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerImprovedTiming(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SOURCE_CLOCK_INPUT *inputBufferPtr;
    CAN_CONTROLLER_IMPROVED_TIMING_OUTPUT *outputBufferPtr;
    CAN_CONTROLLER_IMPROVED_TIMING_OUTPUT localBufferPtr;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    if (DeviceContextPtr->ControllerInfo.IsInitialized == TRUE) {
        WdfRequestComplete(WdfRequest, STATUS_INVALID_DEVICE_STATE);
        return;
    }
    localBufferPtr.status = FLEXCAN_CalculateImprovedTimingValues(registersPtr, DeviceContextPtr->CANconfig.bitRate, *inputBufferPtr, &DeviceContextPtr->CANconfig.timingConfig);
    if (localBufferPtr.status) {
    }
    else {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FLEXCAN_CalculateImprovedTimingValues(..) failed.\n"));
    }
    localBufferPtr.tcfg = DeviceContextPtr->CANconfig.timingConfig;
    RtlCopyMemory(outputBufferPtr, &localBufferPtr, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/* IOCTL_CAN_CONTROLLER_FD_IMPROVED_TIMING */
/* Calculates the improved timing values by specific bit rates for CAN FD */
_Use_decl_annotations_
VOID ImxCanIoctlControllerFDImprovedTiming(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SOURCE_CLOCK_INPUT *inputBufferPtr;
    CAN_CONTROLLER_FD_IMPROVED_TIMING_OUTPUT *outputBufferPtr;
    CAN_CONTROLLER_FD_IMPROVED_TIMING_OUTPUT localBufferPtr;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    if (DeviceContextPtr->ControllerInfo.IsInitialized == TRUE) {
        WdfRequestComplete(WdfRequest, STATUS_INVALID_DEVICE_STATE);
        return;
    }
    localBufferPtr.status = FLEXCAN_FDCalculateImprovedTimingValues(registersPtr, DeviceContextPtr->CANconfig.bitRate, DeviceContextPtr->CANconfig.bitRateFD, *inputBufferPtr, &DeviceContextPtr->CANconfig.timingConfig);
    if (localBufferPtr.status) {
    }
    else {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FLEXCAN_FDCalculateImprovedTimingValues(..) failed.\n"));
    }
    localBufferPtr.tcfg = DeviceContextPtr->CANconfig.timingConfig;
    RtlCopyMemory(outputBufferPtr, &localBufferPtr, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}
#endif

/* IOCTL_CAN_CONTROLLER_INIT */
/* Initializes a FlexCAN instance */
_Use_decl_annotations_
VOID ImxCanIoctlControllerInit(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT32 *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    if (DeviceContextPtr->ControllerInfo.IsInitialized == TRUE) {
        WdfRequestComplete(WdfRequest, STATUS_INVALID_DEVICE_STATE);
        return;
    }
    FLEXCAN_Init(registersPtr,&DeviceContextPtr->CANconfig, *inputBufferPtr);
    DeviceContextPtr->ControllerInfo.IsInitialized = TRUE;
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/* IOCTL_CAN_CONTROLLER_FD_INIT */
/* Initializes a FlexCAN instance */
_Use_decl_annotations_
VOID ImxCanIoctlControllerFDInit(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SET_FDINIT_INTPUT *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    if (DeviceContextPtr->ControllerInfo.IsInitialized == TRUE) {
        WdfRequestComplete(WdfRequest, STATUS_INVALID_DEVICE_STATE);
        return;
    }
    FLEXCAN_FDInit(registersPtr, &DeviceContextPtr->CANconfig, inputBufferPtr->frequency, inputBufferPtr->mbsize, inputBufferPtr->brs);
    DeviceContextPtr->ControllerInfo.IsInitialized = TRUE;
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}
#endif

/* IOCTL_CAN_CONTROLLER_DEINIT */
/* De-initializes a FlexCAN instance */
_Use_decl_annotations_
VOID ImxCanIoctlControllerDeinit(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;

    FLEXCAN_Deinit(registersPtr);
    DeviceContextPtr->ControllerInfo.IsInitialized = FALSE;
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_SET_RXMB_CONFIG */
/* Configures a FlexCAN Receive Message Buffer */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetRxMbConfig(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SET_RXMB_CONFIG *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_SetRxMbConfig(registersPtr, inputBufferPtr->mbidx, &inputBufferPtr->mbConfig, inputBufferPtr->enable);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/* IOCTL_CAN_CONTROLLER_SET_FD_RXMB_CONFIG */
/* Configures a FlexCAN Receive Message Buffer */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetFDRxMbConfig(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SET_RXMB_CONFIG *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_SetFDRxMbConfig(registersPtr, inputBufferPtr->mbidx, &inputBufferPtr->mbConfig, inputBufferPtr->enable);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}
#endif

/* IOCTL_CAN_CONTROLLER_SET_TXMB_CONFIG */
/* Configures a FlexCAN transmit message buffer */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetTxMbConfig(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SET_TXMB_CONFIG *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_SetTxMbConfig(registersPtr, inputBufferPtr->mbidx, inputBufferPtr->enable);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/* IOCTL_CAN_CONTROLLER_SET_FD_TXMB_CONFIG */
/* Configures a FlexCAN transmit message buffer. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetFDTxMbConfig(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SET_TXMB_CONFIG *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_SetFDTxMbConfig(registersPtr, inputBufferPtr->mbidx, inputBufferPtr->enable);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}
#endif

/* IOCTL_CAN_CONTROLLER_CREATE_HANDLE */
/* Initializes the FlexCAN handle */
_Use_decl_annotations_
VOID ImxCanIoctlControllerTransferCreateHandle(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    PINT_CONTEXT interruptContextPtr = ImxCanGetInterruptContext(DeviceContextPtr->WdfInterrupt);

    /* Interrupts are not allowed yet, lock is not required (?) */
    WdfInterruptAcquireLock(DeviceContextPtr->WdfInterrupt);
    /* Initialize circular buffer */
    interruptContextPtr->FrameQueue.count = 0;
    interruptContextPtr->FrameQueue.head = interruptContextPtr->FrameQueue.buffer;
    interruptContextPtr->FrameQueue.tail = interruptContextPtr->FrameQueue.buffer;
    WdfInterruptReleaseLock(DeviceContextPtr->WdfInterrupt);
    RtlZeroMemory(&DeviceContextPtr->CANhandle, sizeof(DeviceContextPtr->CANhandle));
    FLEXCAN_TransferCreateHandle(registersPtr, &DeviceContextPtr->CANhandle, NULL);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_RECEIVE_NON_BLOCKING */
/* Receives a message using IRQ */
_Use_decl_annotations_
VOID ImxCanIoctlControllerReceiveNonBlocking(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_NB_RECEIVE_INPUT *inputBufferPtr;
    status_t *outputBufferPtr;
    status_t status;
    UINT8 mbIdx;
    //flexcan_frame_t frame;
    //flexcan_fd_frame_t framefd;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    mbIdx = inputBufferPtr->mbIdx;
    //frame = inputBufferPtr->frame;
    //framefd = inputBufferPtr->framefd;
    DeviceContextPtr->CANhandle.TransferBuf[mbIdx].mbIdx = mbIdx;
    RtlZeroMemory(&DeviceContextPtr->CANhandle.TransferFrame[mbIdx], sizeof(DeviceContextPtr->CANhandle.TransferFrame[mbIdx]));
    DeviceContextPtr->CANhandle.TransferBuf[mbIdx].frame = &DeviceContextPtr->CANhandle.TransferFrame[mbIdx];
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    RtlZeroMemory(&DeviceContextPtr->CANhandle.TransferFDFrame[mbIdx], sizeof(DeviceContextPtr->CANhandle.TransferFDFrame[mbIdx]));
    DeviceContextPtr->CANhandle.TransferBuf[mbIdx].framefd = &DeviceContextPtr->CANhandle.TransferFDFrame[mbIdx];
#endif
    /* Use TransferBuf[mbIdx] for receive */
    status = FLEXCAN_TransferReceiveNonBlocking(registersPtr, &DeviceContextPtr->CANhandle, &DeviceContextPtr->CANhandle.TransferBuf[mbIdx]);
    RtlCopyMemory(outputBufferPtr, &status, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/* IOCTL_CAN_CONTROLLER_FD_RECEIVE_NON_BLOCKING */
/* Receives a message using IRQ */
_Use_decl_annotations_
VOID ImxCanIoctlControllerFDReceiveNonBlocking(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_NB_RECEIVE_INPUT *inputBufferPtr;
    status_t *outputBufferPtr;
    status_t status;
    UINT8 mbIdx;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    mbIdx = inputBufferPtr->mbIdx;
    DeviceContextPtr->CANhandle.TransferBuf[mbIdx].mbIdx = mbIdx;
    RtlZeroMemory(&DeviceContextPtr->CANhandle.TransferFrame[mbIdx], sizeof(DeviceContextPtr->CANhandle.TransferFrame[mbIdx]));
    DeviceContextPtr->CANhandle.TransferBuf[mbIdx].frame = &DeviceContextPtr->CANhandle.TransferFrame[mbIdx];
    RtlZeroMemory(&DeviceContextPtr->CANhandle.TransferFDFrame[mbIdx], sizeof(DeviceContextPtr->CANhandle.TransferFDFrame[mbIdx]));
    DeviceContextPtr->CANhandle.TransferBuf[mbIdx].framefd = &DeviceContextPtr->CANhandle.TransferFDFrame[mbIdx];
    /* Use TransferBuf[mbIdx] for receive */
    status = FLEXCAN_TransferFDReceiveNonBlocking(registersPtr, &DeviceContextPtr->CANhandle, &DeviceContextPtr->CANhandle.TransferBuf[mbIdx]);
    RtlCopyMemory(outputBufferPtr, &status, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}
#endif

/* IOCTL_CAN_CONTROLLER_SEND_NON_BLOCKING */
/* Sends a message using IRQ */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSendNonBlocking(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_NB_SEND_INPUT *inputBufferPtr;
    flexcan_mb_transfer_t pTransfer;
    status_t *outputBufferPtr;
    status_t status;

    DeviceContextPtr->CurrentRequest = WdfRequest;
    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        DeviceContextPtr->CurrentRequest = NULL;
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        DeviceContextPtr->CurrentRequest = NULL;
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    pTransfer.framefd = &inputBufferPtr->framefd;
#endif
    pTransfer.frame = &inputBufferPtr->frame;
    pTransfer.mbIdx = inputBufferPtr->mbIdx;
    status = FLEXCAN_TransferSendNonBlocking(registersPtr, &DeviceContextPtr->CANhandle, &pTransfer);
    RtlCopyMemory(outputBufferPtr, &status, sizeof(*outputBufferPtr));
    DeviceContextPtr->CurrentRequest = NULL;
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/* IOCTL_CAN_CONTROLLER_FD_SEND_NON_BLOCKING */
/* Sends a message using IRQ. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerFDSendNonBlocking(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_NB_SEND_INPUT *inputBufferPtr;
    flexcan_mb_transfer_t pTransfer;
    status_t *outputBufferPtr;
    status_t status;

    DeviceContextPtr->CurrentRequest = WdfRequest;
    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        DeviceContextPtr->CurrentRequest = NULL;
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        DeviceContextPtr->CurrentRequest = NULL;
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    pTransfer.framefd = &inputBufferPtr->framefd;
    pTransfer.frame = &inputBufferPtr->frame;
    pTransfer.mbIdx = inputBufferPtr->mbIdx;
    status = FLEXCAN_TransferFDSendNonBlocking(registersPtr, &DeviceContextPtr->CANhandle, &pTransfer);
    RtlCopyMemory(outputBufferPtr, &status, sizeof(*outputBufferPtr));
    DeviceContextPtr->CurrentRequest = NULL;
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}
#endif

/* IOCTL_CAN_CONTROLLER_RECEIVE_BLOCKING */
/* Performs a polling receive transaction on the CAN bus */
_Use_decl_annotations_
VOID ImxCanIoctlControllerReceiveBlocking(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT8                          *inputBufferPtr;
    CAN_CONTROLLER_RECEIVE_OUTPUT  *outputBufferPtr;
    CAN_CONTROLLER_RECEIVE_OUTPUT  localBufferPtr;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    localBufferPtr.status = FLEXCAN_TransferReceiveBlocking(registersPtr, *inputBufferPtr, &localBufferPtr.RxFrame);
    RtlCopyMemory(outputBufferPtr, &localBufferPtr, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_SEND_BLOCKING */
/* Performs a polling send transaction on the CAN bus */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSendBlocking(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SEND_INPUT *inputBufferPtr;
    status_t *outputBufferPtr;
    status_t status;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    status = FLEXCAN_TransferSendBlocking(registersPtr, inputBufferPtr->mbidx, &inputBufferPtr->TxFrame);
    RtlCopyMemory(outputBufferPtr, &status, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/* IOCTL_CAN_CONTROLLER_FD_SEND_BLOCKING */
/* Performs a polling send transaction on the CAN bus */
_Use_decl_annotations_
VOID ImxCanIoctlControllerFDSendBlocking(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_FD_SEND_INPUT *inputBufferPtr;
    status_t *outputBufferPtr;
    status_t status;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    status = FLEXCAN_TransferFDSendBlocking(registersPtr, inputBufferPtr->mbidx, &inputBufferPtr->TxFrame);
    RtlCopyMemory(outputBufferPtr, &status, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_FD_RECEIVE_BLOCKING */
/* Performs a polling receive transaction on the CAN bus */
_Use_decl_annotations_
VOID ImxCanIoctlControllerFDReceiveBlocking(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT8                          *inputBufferPtr;
    CAN_CONTROLLER_FD_RECEIVE_OUTPUT  *outputBufferPtr;
    CAN_CONTROLLER_FD_RECEIVE_OUTPUT  localBufferPtr;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    localBufferPtr.status = FLEXCAN_TransferFDReceiveBlocking(registersPtr, *inputBufferPtr, &localBufferPtr.RxFrame);
    RtlCopyMemory(outputBufferPtr, &localBufferPtr, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}
#endif

/* IOCTL_CAN_INVERT_NOTIFICATION */
/* Request is forwarded to the NotificationQueue. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerNotification(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    data_callback   *outputBufferPtr;

    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    NTSTATUS status = WdfRequestForwardToIoQueue(WdfRequest, DeviceContextPtr->NotificationQueue);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestForwardToIoQueue(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    /* Request is pending */
    // KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestForwardToIoQueue(..) (status = %!STATUS!)\n", status));
    DeviceContextPtr->ControllerInfo.RequestCnt++;
}

/* IOCTL_CAN_CONTROLLER_ENTER_FREEZEMODE */
/* Enter FlexCAN Freeze Mode */
_Use_decl_annotations_
VOID ImxCanIoctlControllerEnterFreezeMode(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;

    FLEXCAN_EnterFreezeMode(registersPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_EXIT_FREEZEMODE */
/* Exit FlexCAN Freeze Mode */
_Use_decl_annotations_
VOID ImxCanIoctlControllerExitFreezeMode(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;

    FLEXCAN_ExitFreezeMode(registersPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_SET_TIMING_CONFIG */
/* Sets the FlexCAN classical protocol timing characteristic */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetTimingConfig(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_TIMING_INPUT *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_SetTimingConfig(registersPtr, inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/* IOCTL_CAN_CONTROLLER_SET_FDTIMING_CONFIG */
/* Sets the FlexCAN FD data phase timing characteristic */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetFDTimingConfig(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_TIMING_INPUT *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_SetFDTimingConfig(registersPtr, inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}
#endif

/* IOCTL_CAN_CONTROLLER_SET_RXMB_GLOBAL_MASK */
/* Sets the FlexCAN receive message buffer global mask */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetRxMbGlobalMask(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT32 *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_SetRxMbGlobalMask(registersPtr, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_SET_RXFIFO_GLOBAL_MASK */
/* Sets the FlexCAN receive FIFO global mask */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetRxFifoGlobalMask(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT32 *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_SetRxFifoGlobalMask(registersPtr, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_SET_RX_INDIVIDUAL_MASK */
/* Sets the FlexCAN receive individual mask. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetRxIndividualMask(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_INDIVIDUAL_MASK_INPUT *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_SetRxIndividualMask(registersPtr, inputBufferPtr->maskIdx, inputBufferPtr->mask);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_SET_RXFIFO_CONFIG */
/* Configures the FlexCAN Legacy Rx FIFO. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerSetRxFifoConfig(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SET_RXFIFO_CONFIG *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_SetRxFifoConfig(registersPtr, &inputBufferPtr->RxFifoConfig, inputBufferPtr->enable);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_GET_STATUS_FLAGS */
/* Gets the FlexCAN module interrupt flags. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerGetStatusFlags(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) ||                   \
    (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    UINT64 *outputBufferPtr;
    UINT64 localBuffer;
#else
    UINT32 *outputBufferPtr;
    UINT32 localBuffer;
#endif

    NTSTATUS status = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    localBuffer = FLEXCAN_GetStatusFlags(registersPtr);
    RtlCopyMemory(outputBufferPtr, &localBuffer, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_CLEAR_STATUS_FLAGS */
/* Clears status flags with the provided mask */
_Use_decl_annotations_
VOID ImxCanIoctlControllerClearStatusFlags(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) ||                   \
    (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    UINT64 *inputBufferPtr;
#else
    UINT32 *inputBufferPtr;
#endif

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_ClearStatusFlags(registersPtr, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_GET_BUSERR_COUNT */
/* Gets the FlexCAN Bus Error Counter value */
_Use_decl_annotations_
VOID ImxCanIoctlControllerGetBusErrCount(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_BUSERR_OUTPUT  *outputBufferPtr;
    CAN_CONTROLLER_BUSERR_OUTPUT  localBuffer;

    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    FLEXCAN_GetBusErrCount(registersPtr, &localBuffer.txErrBuf, &localBuffer.rxErrBuf);
    RtlCopyMemory(outputBufferPtr, &localBuffer, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_GET_MBSTATUS_FLAGS */
/* Gets the FlexCAN Message Buffer interrupt flags. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerGetMbStatusFlags(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 *inputBufferPtr;
    UINT64 *outputBufferPtr;
    UINT64 localBuffer;
#else
    UINT32 *inputBufferPtr;
    UINT32 *outputBufferPtr;
    UINT32 localBuffer;
#endif

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }

    localBuffer = FLEXCAN_GetMbStatusFlags(registersPtr, *inputBufferPtr);
    RtlCopyMemory(outputBufferPtr, &localBuffer, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_CLEAR_MBSTATUS_FLAGS */
/* Clears the FlexCAN Message Buffer interrupt flags. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerClearMbStatusFlags(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 *inputBufferPtr;
#else
    UINT32 *inputBufferPtr;
#endif

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    FLEXCAN_ClearMbStatusFlags(registersPtr, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
/* IOCTL_CAN_CONTROLLER_GET_MEMORY_ERROR_STATUS */
/* Gets the FlexCAN Memory Error Report registers status. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerGetMemoryErrorReportStatus(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    flexcan_memory_error_report_status_t *outputBufferPtr;
    /*flexcan_memory_error_report_status_t localBuffer; */

    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    /*FLEXCAN_GetMemoryErrorReportStatus(registersPtr, &localBuffer); */
    /*RtlCopyMemory(outputBufferPtr, &localBuffer, sizeof(*outputBufferPtr)); */
    FLEXCAN_GetMemoryErrorReportStatus(registersPtr, outputBufferPtr);
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}
#endif

/* IOCTL_CAN_CONTROLLER_ENABLE_INTERRUPTS */
/* Enables FlexCAN interrupts according to the provided mask */
_Use_decl_annotations_
VOID ImxCanIoctlControllerEnableInterrupts(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) ||                   \
    (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    UINT64 *inputBufferPtr;
#else
    UINT32 *inputBufferPtr;
#endif

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_EnableInterrupts(registersPtr, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_DISABLE_INTERRUPTS */
/* Disables FlexCAN interrupts according to the provided mask. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerDisableInterrupts(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    UINT64 *inputBufferPtr;
#else
    UINT32 *inputBufferPtr;
#endif

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_DisableInterrupts(registersPtr, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_ENABLE_MB_INTERRUPTS */
/* Enables FlexCAN Message Buffer interrupts. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerEnableMbInterrupts(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 *inputBufferPtr;
#else
    UINT32 *inputBufferPtr;
#endif

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_EnableMbInterrupts(registersPtr, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_DISABLE_MB_INTERRUPTS */
/* Disables FlexCAN Message Buffer interrupts. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerDisableMbInterrupts(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 *inputBufferPtr;
#else
    UINT32 *inputBufferPtr;
#endif

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_DisableMbInterrupts(registersPtr, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA) && FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA)
/* IOCTL_CAN_CONTROLLER_ENABLE_RXFIFO_DMA */
/* Enables or disables the FlexCAN Legacy/Enhanced Rx FIFO DMA request. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerEnableRxFifoDMA(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    BOOLEAN *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_EnableRxFifoDMA(registersPtr, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_GET_RXFIFO_ADDR */
/* Gets the Rx FIFO Head address. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerGetRxFifoHeadAddr(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT32 *outputBufferPtr;
    PUINT32 localBuffer;

    NTSTATUS status = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    localBuffer = FLEXCAN_GetRxFifoHeadAddr(registersPtr);
    RtlCopyMemory(outputBufferPtr, &localBuffer, sizeof(*outputBufferPtr));
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}
#endif

/* IOCTL_CAN_CONTROLLER_ENABLE */
/* Enables or disables the FlexCAN module operation */
_Use_decl_annotations_
VOID ImxCanIoctlControllerEnable(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    BOOLEAN *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_Enable(registersPtr, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_WRITE_TXMB */
/* Writes a FlexCAN Message to the Transmit Message Buffer. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerWriteTxMb(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SEND_INPUT *inputBufferPtr;
    status_t *outputBufferPtr;
    status_t status;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    status = FLEXCAN_WriteTxMb(registersPtr, inputBufferPtr->mbidx, &inputBufferPtr->TxFrame);
    RtlCopyMemory(outputBufferPtr, &status, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_READ_FD_RXMB */
/* Reads a FlexCAN Message from Receive Message Buffer. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerReadRxMb(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT8                          *inputBufferPtr;
    CAN_CONTROLLER_RECEIVE_OUTPUT  *outputBufferPtr;
    CAN_CONTROLLER_RECEIVE_OUTPUT  localBuffer;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    localBuffer.status = FLEXCAN_ReadRxMb(registersPtr, *inputBufferPtr, &localBuffer.RxFrame);
    RtlCopyMemory(outputBufferPtr, &localBuffer, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/* IOCTL_CAN_CONTROLLER_WRITE_FD_TXMB */
/* Writes a FlexCAN FD Message to the Transmit Message Buffer._Use_decl_annotations_ */
_Use_decl_annotations_
VOID ImxCanIoctlControllerWriteFDTxMb(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_SEND_FD_INPUT *inputBufferPtr;
    status_t *outputBufferPtr;
    status_t status;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    status = FLEXCAN_WriteFDTxMb(registersPtr, inputBufferPtr->mbidx, &inputBufferPtr->TxFrame);
    RtlCopyMemory(outputBufferPtr, &status, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_READ_FD_RXMB */
/* Reads a FlexCAN FD Message from Receive Message Buffer. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerReadFDRxMb(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT8                          *inputBufferPtr;
    CAN_CONTROLLER_FD_RECEIVE_OUTPUT  *outputBufferPtr;
    CAN_CONTROLLER_FD_RECEIVE_OUTPUT  localBuffer;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    localBuffer.status = FLEXCAN_ReadFDRxMb(registersPtr, *inputBufferPtr, &localBuffer.RxFrame);
    RtlCopyMemory(outputBufferPtr, &localBuffer, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}
#endif

/* IOCTL_CAN_CONTROLLER_READ_RXFIFO */
/* Reads a FlexCAN Message from Legacy Rx FIFO. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerReadRxFifo(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_RECEIVE_OUTPUT  *outputBufferPtr;
    CAN_CONTROLLER_RECEIVE_OUTPUT  localBuffer;

    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    localBuffer.status = FLEXCAN_ReadRxFifo(registersPtr, &localBuffer.RxFrame);
    RtlCopyMemory(outputBufferPtr, &localBuffer, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/* IOCTL_CAN_CONTROLLER_FD_ABORT_SEND */
/* Aborts the interrupt driven message send process */
_Use_decl_annotations_
VOID ImxCanIoctlControllerFDAbortSend(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT8 *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_TransferFDAbortSend(registersPtr, &DeviceContextPtr->CANhandle, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_FD_ABORT_RECEIVE */
/* Aborts the interrupt driven message receive process */
_Use_decl_annotations_
VOID ImxCanIoctlControllerFDAbortReceive(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT8 *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_TransferFDAbortReceive(registersPtr, &DeviceContextPtr->CANhandle, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}
#endif

/* IOCTL_CAN_CONTROLLER_RECEIVE_FIFO_BLOCKING */
/* Performs a polling receive transaction from Legacy Rx FIFO on the CAN bus. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerReceiveFifoBlocking(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_RECEIVE_OUTPUT  *outputBufferPtr;
    CAN_CONTROLLER_RECEIVE_OUTPUT  localBufferPtr;

    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    localBufferPtr.status = FLEXCAN_TransferReceiveFifoBlocking(registersPtr, &localBufferPtr.RxFrame);
    RtlCopyMemory(outputBufferPtr, &localBufferPtr, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_RECEIVE_FIFO_NON_BLOCKING */
/* Receives a message from Legacy Rx FIFO using IRQ. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerReceiveFifoNonBlocking(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    CAN_CONTROLLER_RECEIVE_FIFO_NB_INPUT *inputBufferPtr;
    flexcan_fifo_transfer_t Transfer;
    status_t *outputBufferPtr;
    status_t status;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    Transfer.frame = &inputBufferPtr->frame;
    status = FLEXCAN_TransferReceiveFifoNonBlocking(registersPtr, &DeviceContextPtr->CANhandle, &Transfer);
    RtlCopyMemory(outputBufferPtr, &status, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_GET_TIMESTAMP */
/* Gets the detail index of Mailbox's Timestamp by handle. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerGetTimeStamp(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    UINT8 *inputBufferPtr;
    UINT32 *outputBufferPtr;
    UINT32 localBuffer;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    localBuffer = FLEXCAN_GetTimeStamp(&DeviceContextPtr->CANhandle, *inputBufferPtr);
    RtlCopyMemory(outputBufferPtr, &localBuffer, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_CONTROLLER_ABORT_SEND */
/* Aborts the interrupt driven message send process. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerAbortSend(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT8 *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_TransferAbortSend(registersPtr, &DeviceContextPtr->CANhandle, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_ABORT_RECEIVE */
/* Aborts the interrupt driven message receive process. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerAbortReceive(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    UINT8 *inputBufferPtr;

    NTSTATUS status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", status));
        WdfRequestComplete(WdfRequest, status);
        return;
    }
    FLEXCAN_TransferAbortReceive(registersPtr, &DeviceContextPtr->CANhandle, *inputBufferPtr);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/* IOCTL_CAN_CONTROLLER_ABORT_RECEIVE_FIFO */
/* Aborts the interrupt driven message receive from Legacy Rx FIFO process. */
_Use_decl_annotations_
VOID ImxCanIoctlControllerAbortReceiveFifo(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;

    FLEXCAN_TransferAbortReceiveFifo(registersPtr, &DeviceContextPtr->CANhandle);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}



#pragma warning(push)
#pragma warning(disable:4100) /* unreferenced formal parameter */
/* IOCTL_CAN_HELPER_FLEXCAN_ID_STD */
/* FlexCAN Frame ID helper macro */
_Use_decl_annotations_
VOID ImxCanIoctlHelperFLEXCAN_ID_STD(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    UINT32 *inputBufferPtr;
    UINT32 *outputBufferPtr;
    UINT32 tmp;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    tmp = *inputBufferPtr;
    tmp = FLEXCAN_ID_STD(tmp);
    RtlCopyMemory(outputBufferPtr, &tmp, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

/* IOCTL_CAN_HELPER_FLEXCAN_RX_MB_STD_MASK */
/* FlexCAN Rx Message Buffer Mask helper macro */
_Use_decl_annotations_
VOID ImxCanIoctlHelperFLEXCAN_RX_MB_STD_MASK(const PDEV_CONTEXT DeviceContextPtr, WDFREQUEST WdfRequest)
{
    CAN_HELPER_FLEXCAN_RX_MB_STD_MASK_INPUT *inputBufferPtr;
    UINT32 *outputBufferPtr;
    UINT32 tmp;

    NTSTATUS istatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), (PVOID*)(&inputBufferPtr), NULL);
    if (!NT_SUCCESS(istatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveInputBuffer(..) failed. (status = %!STATUS!)\n", istatus));
        WdfRequestComplete(WdfRequest, istatus);
        return;
    }
    NTSTATUS ostatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), (PVOID*)(&outputBufferPtr), NULL);
    if (!NT_SUCCESS(ostatus)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", ostatus));
        WdfRequestComplete(WdfRequest, ostatus);
        return;
    }
    tmp = FLEXCAN_RX_MB_STD_MASK(inputBufferPtr->id, inputBufferPtr->rtr, inputBufferPtr->ide);
    RtlCopyMemory(outputBufferPtr, &tmp, sizeof(*outputBufferPtr));
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}
#pragma warning(pop)

IMXCAN_NONPAGED_SEGMENT_END;