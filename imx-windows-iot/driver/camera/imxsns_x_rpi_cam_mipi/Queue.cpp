/*
 * Copyright 2023 NXP
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

#include "driver.h"
#include "queue.tmh"

PAGED_SEGMENT_BEGIN

NTSTATUS SNS0_ctx::QueueInitialize()
/*!
 * Create default queue.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{

    NTSTATUS status;
    WDF_IO_QUEUE_CONFIG queueConfig;

    PAGED_CODE();

    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchSequential);
    queueConfig.EvtIoDeviceControl = SNS0_ctx::EvtDeviceControl;
    queueConfig.EvtIoStop = SNS0_ctx::EvtIoStop;

    WDF_OBJECT_ATTRIBUTES QueueObjectAttributes;
    WDF_OBJECT_ATTRIBUTES_INIT(&QueueObjectAttributes);
    QueueObjectAttributes.ExecutionLevel = WdfExecutionLevelPassive;
    status = WdfIoQueueCreate(m_WdfDevice, &queueConfig, &QueueObjectAttributes, &m_Queue);

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_QUEUE, "WdfIoQueueCreate failed %!STATUS!", status);
        return status;
    }

    return status;
}

VOID SNS0_ctx::EvtDeviceControl(_In_ WDFQUEUE Queue, _In_ WDFREQUEST Request, _In_ size_t OutputBufferLength, _In_ size_t InputBufferLength, _In_ ULONG IoControlCode)
/*!
 * EvtDeviceControl callback.
 *
 * @param Queue handle to a WDF queue object.
 * @param WdfRequest handle to a WDF request object.
 * @param OutputBufferLength size of request output buffer.
 * @param OutputBufferLength size of request input buffer.
 * @param IoControlCode control code of the IRP.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    WDFDEVICE wdfDevice = WdfIoQueueGetDevice(Queue);
    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(wdfDevice);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_QUEUE, "%!FUNC! Queue 0x%p, Request 0x%p OutputBufferLength %d InputBufferLength %d IoControlCode %d",
                Queue, Request, (int) OutputBufferLength, (int) InputBufferLength, IoControlCode);

    switch (IoControlCode)
    {
    case IOCTL_SNS_CONFIGURE:
        TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "IOCTL_SNS_CONFIGURE");
        status = deviceCtxPtr->ConfigureReq(Request);
        break;
    case IOCTL_SNS_START:
        TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "IOCTL_SNS_START");
        status = deviceCtxPtr->StartReq(Request);
        break;
    case IOCTL_SNS_STOP:
        TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "IOCTL_SNS_STOP");
        status = deviceCtxPtr->StopReq(Request);
        break;
    default:
        TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "Invalid device request 0x%x", IoControlCode);
        status = STATUS_INVALID_DEVICE_REQUEST;
        break;
    }
    WdfRequestComplete(Request, status);

    return;
}

VOID SNS0_ctx::EvtIoStop(_In_ WDFQUEUE Queue, _In_ WDFREQUEST Request, _In_ ULONG ActionFlags)
/*!
 * Calls WdfMipi_ctx::EvtWdfRequestCancel to cancel the request.
 *
 * @param Queue handle to a WDF queue object.
 * @param WdfRequest handle to a WDF request object.
 * @param ActionFlags requested action on the request object.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_QUEUE, "%!FUNC! Queue 0x%p, Request 0x%p ActionFlags %d", Queue, Request, ActionFlags);

    // No action required as all implemented requests are completed within request handler.

    return;
}

PAGED_SEGMENT_END
