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

#include "Trace.h"
#if (MALONE_TRACE == MALONE_TRACE_WPP)
    #include "ioctl.tmh"
#endif
#include "Device.h"
#include "vpu_dec/vdec.h"

#ifdef ALLOC_PRAGMA
    #pragma alloc_text (PAGE, IoInit)
#endif

/*****************************************************************************************************************************
* devctl api calls
*****************************************************************************************************************************/
static NTSTATUS io_devctl_dec_create(MALONE_VPU_DEVICE_CONTEXT *DeviceContextPtr, WDFREQUEST Request)
{
    NTSTATUS Status = STATUS_SUCCESS;
    vdec_init_t *dec = NULL;
    size_t isize = 0;
    vdec_mem_desc_t *mem;
    size_t osize = 0;
    int vdec_err = VDEC_EOK;
    WDFFILEOBJECT file = NULL; /* file object - for memlist alloc entry */
    DBG_IOCTL_METHOD_BEG();

    Status = WdfRequestRetrieveInputBuffer(Request, sizeof(vdec_init_t), (PVOID *)&dec, &isize);
    if (!NT_SUCCESS(Status)) {
        DBG_PRINT_ERROR_WITH_STATUS(Status, "Unable to retrieve ioctl input buffer");
        DBG_IOCTL_METHOD_END();
        return Status;
    }

    Status = WdfRequestRetrieveOutputBuffer(Request, sizeof(vdec_mem_desc_t), (PVOID *)&mem, &osize);
    if (!NT_SUCCESS(Status)) {
        DBG_PRINT_ERROR_WITH_STATUS(Status, "Unable to retrieve ioctl output buffer");
        DBG_IOCTL_METHOD_END();
        return Status;
    }

    file = WdfRequestGetFileObject(Request);

    if ((vdec_err = vdec_open(DeviceContextPtr, file, dec, mem)) != VDEC_EOK) {
        DBG_PRINT_ERROR_WITH_STATUS(vdec_err, "vdec_open failed!");
        Status = STATUS_UNSUCCESSFUL;
    }

    DBG_IOCTL_METHOD_END();
    return Status;
}

static NTSTATUS io_devctl_dec_destroy(MALONE_VPU_DEVICE_CONTEXT *DeviceContextPtr, WDFREQUEST Request)
{
    NTSTATUS Status = STATUS_SUCCESS;
    int vdec_err = VDEC_EOK;
    WDFFILEOBJECT file = NULL;                                   /* file object - for memlist alloc entry */
    int stream_id = -1;
    DBG_IOCTL_METHOD_BEG();

    file = WdfRequestGetFileObject(Request);
    stream_id = GetStreamIDFromFile(DeviceContextPtr, file);
    if (stream_id >= 0) {
        do {
            DBG_EVENTS_PRINT_INFO("MFT->KM decoder_deinit() ctx[%d]", stream_id);
            if ((vdec_err = vdec_stop(DeviceContextPtr, stream_id, VPU_BLOCK)) != VDEC_EOK) {
                DBG_PRINT_ERROR_WITH_STATUS(vdec_err, "vdec_open failed!");
                Status = STATUS_UNSUCCESSFUL;
                break;
            }
            if ((vdec_err = vdec_close(DeviceContextPtr, stream_id)) != VDEC_EOK) {
                DBG_PRINT_ERROR_WITH_STATUS(vdec_err, "vdec_open failed!");
                Status = STATUS_UNSUCCESSFUL;
                break;
            }
        } while (FALSE);
    } else {
        Status = STATUS_INSUFFICIENT_RESOURCES;
    }
    DBG_IOCTL_METHOD_END();
    return Status;
}

static NTSTATUS io_devctl_dec_status(MALONE_VPU_DEVICE_CONTEXT *DeviceContextPtr, WDFREQUEST Request)
{
    NTSTATUS Status = STATUS_SUCCESS;
    int vdec_err = VDEC_EOK;
    vdec_status_t *status;
    size_t osize = 0;
    WDFFILEOBJECT file;    /* file object - for memlist alloc entry */
    int stream_id = -1;
    DBG_IOCTL_METHOD_BEG();

    file = WdfRequestGetFileObject(Request);
    Status = WdfRequestRetrieveOutputBuffer(Request, sizeof(vdec_status_t), (PVOID *)&status, &osize);
    if (!NT_SUCCESS(Status)) {
        DBG_PRINT_ERROR_WITH_STATUS(Status, "Unable to retrieve ioctl output buffer");
        DBG_IOCTL_METHOD_END();
        return Status;
    }

    stream_id = GetStreamIDFromFile(DeviceContextPtr, file);
    if (stream_id >= 0) {
        if ((vdec_err = vdec_status(DeviceContextPtr, stream_id, status)) != VDEC_EOK) {
            DBG_PRINT_ERROR_WITH_STATUS(vdec_err, "vdec_decode failed!");
            Status = STATUS_UNSUCCESSFUL;
        }
    } else {
        Status = STATUS_INSUFFICIENT_RESOURCES;
    }
    DBG_IOCTL_METHOD_END();
    return Status;
}

static NTSTATUS io_devctl_dec_decode(MALONE_VPU_DEVICE_CONTEXT *DeviceContextPtr, WDFREQUEST Request)
{
    NTSTATUS Status = STATUS_SUCCESS;
    vdec_decode_t *dec; /*BIG WARN shared for input and output! CHECK IF POSSIBLE! */
    size_t isize = 0;
    size_t osize = 0;
    int vdec_err = VDEC_EOK;
    int stream_id = -1;
    WDFFILEOBJECT file;                                  /* file object - for memlist alloc entry */
    DBG_IOCTL_METHOD_BEG();

    Status = WdfRequestRetrieveInputBuffer(Request, sizeof(vdec_decode_t), (PVOID *)&dec, &isize);
    if (!NT_SUCCESS(Status)) {
        DBG_PRINT_ERROR_WITH_STATUS(Status, "Unable to retrieve ioctl input buffer");
        DBG_IOCTL_METHOD_END();
        return Status;
    }

    Status = WdfRequestRetrieveOutputBuffer(Request, sizeof(vdec_decode_t), (PVOID *)&dec, &osize);
    if (!NT_SUCCESS(Status)) {
        DBG_PRINT_ERROR_WITH_STATUS(Status, "Unable to retrieve ioctl output buffer");
        DBG_IOCTL_METHOD_END();
        return Status;
    }

    file = WdfRequestGetFileObject(Request);
    stream_id = GetStreamIDFromFile(DeviceContextPtr, file);
    if (stream_id >= 0) {
        if ((vdec_err = vdec_decode(DeviceContextPtr, stream_id, dec)) != VDEC_EOK) {
            DBG_PRINT_ERROR_WITH_STATUS(vdec_err, "vdec_decode failed!");
            Status = STATUS_SUCCESS;
        }
    } else {
        Status = STATUS_INSUFFICIENT_RESOURCES;
    }
    DBG_IOCTL_METHOD_END();
    return Status;
}

static NTSTATUS io_devctl_dec_getoutput(MALONE_VPU_DEVICE_CONTEXT *DeviceContextPtr, WDFREQUEST Request)
{
    NTSTATUS Status = STATUS_SUCCESS;
    fbo_t *fbo = NULL; /*BIG WARN shared for input and output! CHECK IF POSSIBLE! */
    size_t isize = 0;
    size_t osize = 0;
    int vdec_err = VDEC_EOK;
    int stream_id = -1;
    WDFFILEOBJECT file;                                  /* file object - for memlist alloc entry */
    DBG_IOCTL_METHOD_BEG();

    Status = WdfRequestRetrieveInputBuffer(Request, sizeof(fbo_t), (PVOID *)&fbo, &isize);
    if (!NT_SUCCESS(Status)) {
        DBG_PRINT_ERROR_WITH_STATUS(Status, "Unable to retrieve ioctl input buffer");
        DBG_IOCTL_METHOD_END();
        return Status;
    }

    Status = WdfRequestRetrieveOutputBuffer(Request, sizeof(fbo_t), (PVOID *)&fbo, &osize);
    if (!NT_SUCCESS(Status)) {
        DBG_PRINT_ERROR_WITH_STATUS(Status, "Unable to retrieve ioctl output buffer");
        DBG_IOCTL_METHOD_END();
        return Status;
    }

    file = WdfRequestGetFileObject(Request);
    stream_id = GetStreamIDFromFile(DeviceContextPtr, file);
    if (stream_id >= 0) {
        if ((vdec_err = vdec_get_output(DeviceContextPtr, stream_id, fbo)) != VDEC_EOK) {
            DBG_PRINT_ERROR_WITH_STATUS(vdec_err, "vdec_get_output failed!");
            Status = STATUS_UNSUCCESSFUL;
        }
    } else {
        Status = STATUS_INSUFFICIENT_RESOURCES;
    }
    DBG_IOCTL_METHOD_END();
    return Status;
}


static NTSTATUS io_devctl_dec_flush(MALONE_VPU_DEVICE_CONTEXT *DeviceContextPtr, WDFREQUEST Request)
{
    NTSTATUS Status = STATUS_SUCCESS;
    vdec_flush_t *flush = NULL; /*BIG WARN shared for input and output! CHECK IF POSSIBLE! */
    size_t isize = 0;
    int vdec_err = VDEC_EOK;
    int stream_id = -1;
    WDFFILEOBJECT file;                                  /* file object - for memlist alloc entry */
    DBG_IOCTL_METHOD_BEG();

    Status = WdfRequestRetrieveInputBuffer(Request, sizeof(vdec_flush_t), (PVOID *)&flush, &isize);
    if (!NT_SUCCESS(Status)) {
        DBG_PRINT_ERROR_WITH_STATUS(Status, "Unable to retrieve ioctl input buffer");
        DBG_IOCTL_METHOD_END();
        return Status;
    }

    file = WdfRequestGetFileObject(Request);
    stream_id = GetStreamIDFromFile(DeviceContextPtr, file);
    if (stream_id >= 0) {
        if ((vdec_err = vdec_flush(DeviceContextPtr, stream_id, flush)) != VDEC_EOK) {
            DBG_PRINT_ERROR_WITH_STATUS(vdec_err, "vdec_flush failed!");
            Status = STATUS_UNSUCCESSFUL;
        }
    } else {
        Status = STATUS_INSUFFICIENT_RESOURCES;
    }
    DBG_IOCTL_METHOD_END();
    return Status;
}


static NTSTATUS io_devctl_dec_clear(MALONE_VPU_DEVICE_CONTEXT *DeviceContextPtr, WDFREQUEST Request)
{
    NTSTATUS Status = STATUS_SUCCESS;
    int *clr_idx; /*BIG WARN shared for input and output! CHECK IF POSSIBLE! */
    size_t isize = 0;
    int vdec_err = VDEC_EOK;
    int stream_id = -1;
    WDFFILEOBJECT file = { 0 };                                  /* file object - for memlist alloc entry */
    DBG_IOCTL_METHOD_BEG();

    Status = WdfRequestRetrieveInputBuffer(Request, sizeof(int), (PVOID *)&clr_idx, &isize);
    if (!NT_SUCCESS(Status)) {
        DBG_PRINT_ERROR_WITH_STATUS(Status, "Unable to retrieve ioctl input buffer");
        DBG_IOCTL_METHOD_END();
        return Status;
    }

    file = WdfRequestGetFileObject(Request);
    stream_id = GetStreamIDFromFile(DeviceContextPtr, file);
    if (stream_id >= 0) {
        if ((vdec_err = vdec_clear_output(DeviceContextPtr, stream_id, *clr_idx)) != VDEC_EOK) {
            DBG_PRINT_ERROR_WITH_STATUS(vdec_err, "vdec_clear_output failed!");
            Status = STATUS_UNSUCCESSFUL;
        }
    } else {
        Status = STATUS_INSUFFICIENT_RESOURCES;
    }
    DBG_IOCTL_METHOD_END();
    return Status;
}


VOID OnIoDeviceControl(
    _In_ WDFQUEUE Queue,
    _In_ WDFREQUEST Request,
    _In_ size_t OutputBufferLength,
    _In_ size_t InputBufferLength,
    _In_ ULONG IoControlCode
)
/*++
   Routine Description:
    This event is invoked when the framework receives IRP_MJ_DEVICE_CONTROL request.
   Arguments:
    Queue -  Handle to the framework queue object that is associated with the
             I/O request.
    Request - Handle to a framework request object.
    OutputBufferLength - Size of the output buffer in bytes
    InputBufferLength - Size of the input buffer in bytes
    IoControlCode - I/O control code.
   Return Value:
    VOID
--*/
{

    DBG_IOCTL_METHOD_BEG_WITH_PARAMS(
        "%!FUNC! Queue 0x%p, Request 0x%p OutputBufferLength %d InputBufferLength %d IoControlCode %d",
        Queue,
        Request,
        (int)OutputBufferLength,
        (int)InputBufferLength,
        IoControlCode);

    WDFDEVICE device;
    MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr;

    /*PCHAR Buffer; */
    /*size_t BufferSize; */
    ULONG BytesWritten = 0;
    NTSTATUS Status = STATUS_SUCCESS;

    PAGED_CODE();
    device = WdfIoQueueGetDevice(Queue);
    deviceContextPtr = DeviceGetContext(device);

    switch (IoControlCode) {
        case VPU_IOC_INIT:
            DBG_IOCTL_PRINT_INFO("decoder_init()");
            if (InputBufferLength != sizeof(vdec_init_t) || OutputBufferLength != sizeof(vdec_mem_desc_t)) {
                DBG_PRINT_ERROR("STATUS_INVALID_PARAMETER");
                Status = STATUS_INVALID_PARAMETER;
                goto end;
            }
            Status = io_devctl_dec_create(deviceContextPtr, Request);
            BytesWritten = sizeof(vdec_mem_desc_t);

            break;
        case VPU_IOC_DEINIT:
            DBG_IOCTL_PRINT_INFO("decoder_deinit()");
            Status = io_devctl_dec_destroy(deviceContextPtr, Request);
            break;
        case VPU_IOC_STATUS:
            DBG_IOCTL_PRINT_INFO("decoder_status()");
            Status = io_devctl_dec_status(deviceContextPtr, Request);
            BytesWritten = sizeof(vdec_status_t);

            break;
        case VPU_IOC_DECODE:
            DBG_IOCTL_PRINT_INFO("decoder_decode()");
            Status = io_devctl_dec_decode(deviceContextPtr, Request);
            BytesWritten = sizeof(vdec_decode_t);
            break;

#ifdef NOT_SUPPORTED
        case VPU_IOC_REG_FRAME_BUFFERS:
            DBG_IOCTL_PRINT_INFO("API-call:VPU_IOC_REG_FRAME_BUFFERS");
            /*status = io_devctl_reg_frame_buffers(ctp, msg, ocb); */
            break;
#endif
        case VPU_IOC_FLUSH:
            DBG_IOCTL_PRINT_INFO("decoder_flush()");
            Status = io_devctl_dec_flush(deviceContextPtr, Request);
            break;
        case VPU_IOC_CLEAR:
            DBG_IOCTL_PRINT_INFO("decoder_clear_output()");
            Status = io_devctl_dec_clear(deviceContextPtr, Request);
            break;
        case VPU_IOC_GETOUTPUT:
            DBG_IOCTL_PRINT_INFO("decoder_get_output()");
            Status = io_devctl_dec_getoutput(deviceContextPtr, Request);
            BytesWritten = sizeof(fbo_t);
            break;
        default:
            DBG_EVENTS_PRINT_INFO("API-call:unknown command");
            Status = STATUS_INVALID_DEVICE_REQUEST;
    }

end:
    WdfRequestCompleteWithInformation(Request, Status, BytesWritten);
    return;

}

VOID
OnIoStop(
    _In_ WDFQUEUE Queue,
    _In_ WDFREQUEST Request,
    _In_ ULONG ActionFlags
)
/*++

   Routine Description:

    This event is invoked for a power-managed queue before the device leaves the working state (D0).

   Arguments:

    Queue -  Handle to the framework queue object that is associated with the
             I/O request.

    Request - Handle to a framework request object.

    ActionFlags - A bitwise OR of one or more WDF_REQUEST_STOP_ACTION_FLAGS-typed flags
                  that identify the reason that the callback function is being called
                  and whether the request is cancelable.

   Return Value:

    VOID

   --*/
{
    UNREFERENCED_PARAMETER(Queue);
    UNREFERENCED_PARAMETER(Request);
    UNREFERENCED_PARAMETER(ActionFlags);

    DBG_IOCTL_METHOD_BEG_WITH_PARAMS("%!FUNC! Queue 0x%p, Request 0x%p ActionFlags %d",
                                     Queue, Request, ActionFlags);

    //
    // In most cases, the EvtIoStop callback function completes, cancels, or postpones
    // further processing of the I/O request.
    //
    // Typically, the driver uses the following rules:
    //
    // - If the driver owns the I/O request, it calls WdfRequestUnmarkCancelable
    //   (if the request is cancelable) and either calls WdfRequestStopAcknowledge
    //   with a Requeue value of TRUE, or it calls WdfRequestComplete with a
    //   completion status value of STATUS_SUCCESS or STATUS_CANCELLED.
    //
    //   Before it can call these methods safely, the driver must make sure that
    //   its implementation of EvtIoStop has exclusive access to the request.
    //
    //   In order to do that, the driver must synchronize access to the request
    //   to prevent other threads from manipulating the request concurrently.
    //   The synchronization method you choose will depend on your driver's design.
    //
    //   For example, if the request is held in a shared context, the EvtIoStop callback
    //   might acquire an internal driver lock, take the request from the shared context,
    //   and then release the lock. At this point, the EvtIoStop callback owns the request
    //   and can safely complete or requeue the request.
    //
    // - If the driver has forwarded the I/O request to an I/O target, it either calls
    //   WdfRequestCancelSentRequest to attempt to cancel the request, or it postpones
    //   further processing of the request and calls WdfRequestStopAcknowledge with
    //   a Requeue value of FALSE.
    //
    // A driver might choose to take no action in EvtIoStop for requests that are
    // guaranteed to complete in a small amount of time.
    //
    // In this case, the framework waits until the specified request is complete
    // before moving the device (or system) to a lower power state or removing the device.
    // Potentially, this inaction can prevent a system from entering its hibernation state
    // or another low system power state. In extreme cases, it can cause the system
    // to crash with bugcheck code 9F.
    //
    DBG_IOCTL_METHOD_END();
    return;
}


NTSTATUS
IoInit(_In_ WDFDEVICE Device)
/*++
   Routine Description:
     The I/O dispatch callbacks for the frameworks device object
     are configured in this function.
     A single default I/O Queue is configured for parallel request
     processing, and a driver context memory allocation is created
     to hold our structure QUEUE_CONTEXT.
   Arguments:
    Device - Handle to a framework device object.
   Return Value:
    VOID
--*/
{
    WDFQUEUE queue = { 0 };
    NTSTATUS status = { 0 };
    WDF_IO_QUEUE_CONFIG queueConfig = { 0 };

    PAGED_CODE();
    DBG_DEV_METHOD_BEG();

    /* Configure a default queue so that requests that are not */
    /* configure-fowarded using WdfDeviceConfigureRequestDispatching to goto */
    /* other queues get dispatched here. */
    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(
        &queueConfig,
        WdfIoQueueDispatchParallel
    );

    queueConfig.EvtIoDeviceControl = OnIoDeviceControl;
    queueConfig.EvtIoStop = OnIoStop;

    status = WdfIoQueueCreate(
                 Device,
                 &queueConfig,
                 WDF_NO_OBJECT_ATTRIBUTES,
                 &queue
             );

    if (!NT_SUCCESS(status)) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "WdfIoQueueCreate failed!");
        DBG_DEV_METHOD_END();
        return status;
    }

    DBG_DEV_METHOD_END_WITH_STATUS(status);
    return status;
}
