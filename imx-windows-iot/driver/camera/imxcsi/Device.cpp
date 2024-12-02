
/*
 * Copyright 2022-2023 NXP
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

#include "imxcsi.h"
#include "public.h"
#include <Ntstrsafe.h>
#include <Ntintsafe.h>
#include "device.tmh"

extern "C" {
    INIT_SEGMENT_BEGIN

    NTSTATUS DriverEntry(_In_ PDRIVER_OBJECT  DriverObject, _In_ PUNICODE_STRING RegistryPath)
    /*!
     * DriverEntry initializes the driver.
     *
     * @param DriverObject handle to a WDF Driver object.
     * @param RegistryPath driver specific path in the Registry.
     *
     * @returns STATUS_SUCCESS or error code.
     */
    {
        WDF_DRIVER_CONFIG config;
        NTSTATUS status;
        WDF_OBJECT_ATTRIBUTES attributes;

        // Initialize WPP Tracing
        WPP_INIT_TRACING(DriverObject, RegistryPath);
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "%!FUNC! Entry");

        // Register a cleanup callback.
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.EvtCleanupCallback = WdfCsi_ctx::EvtDriverContextCleanup;
        attributes.SynchronizationScope = WdfSynchronizationScopeDevice;
        attributes.ExecutionLevel = WdfExecutionLevelDispatch;
        WDF_DRIVER_CONFIG_INIT(&config, DEVICE_CONTEXT::EvtDeviceAdd);
        config.DriverPoolTag = 'IMXM';

        status = WdfDriverCreate(DriverObject, RegistryPath, &attributes, &config, WDF_NO_HANDLE);

        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "WdfDriverCreate failed %!STATUS!", status);
            WPP_CLEANUP(DriverObject);
            return status;
        }

        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "%!FUNC! Exit");

        return status;
    }

    INIT_SEGMENT_END
}

PAGED_SEGMENT_BEGIN

VOID WdfCsi_ctx::EvtDriverContextCleanup(_In_ WDFOBJECT DriverObject)
/*!
 * Free all the resources allocated in DriverEntry.
 *
 * @param DriverObject handle to a WDF Driver object.
 */
{
    UNREFERENCED_PARAMETER(DriverObject);

    PAGED_CODE();
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "%!FUNC! Entry");

    // Stop WPP Tracing
    WPP_CLEANUP(WdfDriverWdmGetDriverObject((WDFDRIVER)DriverObject));
}

NTSTATUS WdfCsi_ctx::EvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit)
/*!
 * Set pnp callbacks, Fileobject and WdfRequest context and creqte default queue.
 *
 * @param DriverObject handle to a WDF Driver object.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    WDF_OBJECT_ATTRIBUTES deviceAttributes;
    WDFDEVICE wdfDevice;
    NTSTATUS status = STATUS_SUCCESS;

    PAGED_CODE();
    UNREFERENCED_PARAMETER(Driver);
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&deviceAttributes, DEVICE_CONTEXT);

    { // PNP and Power
        WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);

        pnpCallbacks.EvtDevicePrepareHardware = WdfCsi_ctx::EvtPrepareHw;
        pnpCallbacks.EvtDeviceReleaseHardware = WdfCsi_ctx::EvtReleaseHw;
        pnpCallbacks.EvtDeviceD0Entry = WdfCsi_ctx::EvtD0Entry;
        pnpCallbacks.EvtDeviceD0Exit = WdfCsi_ctx::EvtD0Exit;

        WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);
        WdfDeviceInitSetIoType(DeviceInit, WdfDeviceIoDirect);
    }

    { // File object
        WDF_FILEOBJECT_CONFIG wdfFileObjectConfig;
        WDF_FILEOBJECT_CONFIG_INIT(
            &wdfFileObjectConfig,
            WdfCsi_ctx::EvtDeviceFileCreate,
            WdfCsi_ctx::EvtDeviceFileClose,
            WdfCsi_ctx::EvtDeviceFileCleanup);

        WdfDeviceInitSetFileObjectConfig(DeviceInit, &wdfFileObjectConfig, WDF_NO_OBJECT_ATTRIBUTES);
        WdfDeviceInitSetExclusive(DeviceInit, TRUE);
    }

    { // Request context
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, REQUEST_CONTEXT);
        WdfDeviceInitSetRequestAttributes(DeviceInit, &attributes);
    }

    status = WdfDeviceCreate(&DeviceInit, &deviceAttributes, &wdfDevice);

    if (NT_SUCCESS(status)) {
        PDEVICE_CONTEXT deviceContextPtr = new (DeviceGetContext(wdfDevice)) DEVICE_CONTEXT(wdfDevice);

        status = DEVICE_INTERRUPT_CONTEXT::registerInterruptHandler(deviceContextPtr);
        if (NT_SUCCESS(status)) {
            status = deviceContextPtr->RegisterQueue();
        }
    }
    return status;
}

PAGED_SEGMENT_END

WdfCsi_ctx::WdfCsi_ctx(WDFDEVICE &WdfDevice)
    : m_WdfDevice(WdfDevice),
      m_Csi1Reg(WdfDevice),
      m_RxLevel(3),
      m_DsdRes(WdfDeviceWdmGetPhysicalDevice(WdfDevice)),
      m_CsiTwo8bitSensorMode(false)
/*!
 * Initialize device context.
 *
 * @param WdfDevice handle to a WDF device object.
 */
{

    PAGED_CODE();
}

NTSTATUS WdfCsi_ctx::Get_DsdAcpiResources()
/*!
 * Load device resources and device specific data.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;

    status = m_DsdRes.LoadDsd();
    if (NT_SUCCESS(status)) {
        const AcpiDsdRes_t::_DSDVAL_GET_DESCRIPTOR ParamTable[] = {
            {
                "CoreClockFrequencyHz",
                &coreClockFrequencyHz,
                (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
            },
            {
                "Csi1RegResId",
                &Csi1RegResId,
                (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
            },
            {
                "CpuId",
                &m_CpuId,
                (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
            },
            {
                "DeviceEndpoint0",
                (PUINT32) &(m_DeviceEndpoint[0]),
                sizeof(m_DeviceEndpoint),
                ACPI_METHOD_ARGUMENT_STRING,
            },
        };

        status = m_DsdRes.GetDsdResources(ParamTable, sizeof(ParamTable) / sizeof(ParamTable[0]));
        if (!NT_SUCCESS(status)) {
            _DbgKdPrint(("WdfCsi_ctx::DsdRes.GetDsdResources Fail\r\n"));
        }
        if (NT_SUCCESS(status)) {
            NTSTATUS tmpStatus = m_DsdRes.GetInteger("Two8bitSensorMode", &(m_CsiTwo8bitSensorMode));
            if (!NT_SUCCESS(tmpStatus)) {
                m_CsiTwo8bitSensorMode = false;
            }
        }
    }
    return status;
}

NTSTATUS WdfCsi_ctx::EvtPrepareHw(_In_ WDFDEVICE WdfDevice, _In_ WDFCMRESLIST ResourcesRaw, _In_ WDFCMRESLIST ResourcesTranslated)
/*!
 * WdfCsi_ctx::EvtPrepareHw bridges to non-static method WdfCsi_ctx::PrepareHw.
 *
 * @param WdfDevice handle to a WDF device object.
 * @param ResourcesRaw list of assigned resources.
 * @param ResourcesTranslated list of translated assigned resources.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    PDEVICE_CONTEXT deviceContextPtr = DeviceGetContext(WdfDevice);

    if (deviceContextPtr == NULL) {
        return STATUS_NOINTERFACE;
    }
    else {
        return deviceContextPtr->PrepareHw(ResourcesRaw, ResourcesTranslated);
    }
}

NTSTATUS WdfCsi_ctx::PrepareHw(WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
/*!
 * Opens assigned resources.
 *
 * @param ResourcesRaw list of assigned resources.
 * @param ResourcesTranslated list of translated assigned resources.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(ResourcesRaw);
    PAGED_CODE();

    status = Get_CrsAcpiResources(ResourcesTranslated);
    if (NT_SUCCESS(status)) {
        mem_res *memRes = NULL;

        status = Get_DsdAcpiResources();
        if (NT_SUCCESS(status)) {
            if (NULL != (memRes = m_MeResList.at(Csi1RegResId))) {
                status = m_Csi1Reg.IoSpaceMap(*memRes);
            }
            else {
                status = STATUS_INSUFFICIENT_RESOURCES; // Memory is required, report STATUS_INSUFFICIENT_RESOURCES if none.
            }
            if (NT_SUCCESS(status)) {
                {
                    m_State = S_STOPPED;

                    for (unsigned i = 0; ((i < 3) && NT_SUCCESS(status)); ++i) {
                        status = AllocFb(m_DiscardBuff[i]);
                    }
                    if (NT_SUCCESS(status)) {
                        m_MipiCsiRes.m_CsiRegistersPtr = m_Csi1Reg.Reg();
                        if (NT_SUCCESS(status)) {
                            m_MipiCsiRes.m_CpuId = m_CpuId;
                            m_CsiRegistersPtr = m_MipiCsiRes.m_CsiRegistersPtr;
                        }
                    }
                }
            }
        }
        if (NT_SUCCESS(status)) {
            ANSI_STRING deviceEndpointAnsiName;

            _Analysis_assume_nullterminated_(m_DeviceEndpoint);
            status = RtlInitAnsiStringEx(&deviceEndpointAnsiName, (PCSZ)(m_DeviceEndpoint));
            if (NT_SUCCESS(status)) {
                RtlInitEmptyUnicodeString(&m_DeviceEndpointUnicodeName, m_DeviceEndpointUnicodeNameBuff, sizeof(m_DeviceEndpointUnicodeNameBuff));
                status = RtlAnsiStringToUnicodeString(&m_DeviceEndpointUnicodeName, &deviceEndpointAnsiName, FALSE);
            }
            if (NT_SUCCESS(status)) {
                _DbgKdPrint(("WdfDeviceCreateSymbolicLink (%ws).\r\n", m_DeviceEndpointUnicodeName.Buffer));
                status = WdfDeviceCreateSymbolicLink(m_WdfDevice, &m_DeviceEndpointUnicodeName);
            }
#if (DBG)
            if (NT_SUCCESS(status)) {
                status = WdfDeviceCreateDeviceInterface(m_WdfDevice, &GUID_DEVINTERFACE_IMXCSI, NULL);
                if (!NT_SUCCESS(status)) {
                    _DbgKdPrint(("WdfCsi_ctx::PrepareHw: Failed to create interface 0x%x.\r\n", status));
                }
            }
#endif
        }
    }
    return status;
}

NTSTATUS WdfCsi_ctx::EvtReleaseHw(_In_ WDFDEVICE WdfDevice, _In_ WDFCMRESLIST ResourcesTranslated)
/*!
 * Unmaps memory resources and closes IoTargets.
 *
 * @param WdfDevice handle to a WDF device object.
 * @param ResourcesTranslated list of translated assigned resources.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(WdfDevice);
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    PAGED_CODE();

    PDEVICE_CONTEXT deviceContextPtr = DeviceGetContext(WdfDevice);
    ASSERT(deviceContextPtr != NULL);
    regBase alocs[] = {
        deviceContextPtr->m_Csi1Reg,
    };

    deviceContextPtr->FreeFb(deviceContextPtr->m_DiscardBuff[0]);
    deviceContextPtr->FreeFb(deviceContextPtr->m_DiscardBuff[1]);
    deviceContextPtr->FreeFb(deviceContextPtr->m_DiscardBuff[2]);

    for (auto a : alocs) {
        a.IoSpaceUnmap();
    }
    {
        auto &resList = deviceContextPtr->m_MeResList;
        auto *a = resList.Pop();

        while (a != NULL) {
            ExFreePoolWithTag(a, resList.DriverPoolTag);
            a = resList.Pop();
        }
    }
    {
        auto &resList = deviceContextPtr->m_GpioResList;
        auto *a = resList.Pop();

        while (a != NULL) {
            ExFreePoolWithTag(a, resList.DriverPoolTag);
            a = resList.Pop();
        }
    }
    {
        auto &resList = deviceContextPtr->m_I2cResList;
        auto *a = resList.Pop();

        while (a != NULL) {
            ExFreePoolWithTag(a, resList.DriverPoolTag);
            a = resList.Pop();
        }
    }
    {
        auto &resList = deviceContextPtr->m_IntResList;
        auto *a = resList.Pop();

        while (a != NULL) {
            ExFreePoolWithTag(a, resList.DriverPoolTag);
            a = resList.Pop();
        }
    }
    deviceContextPtr->m_DsdRes.Cleanup();

    return STATUS_SUCCESS;
}

NTSTATUS WdfCsi_ctx::EvtD0Entry(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE PreviousState)
/*!
 * Handles EvtD0Entry.
 *
 * @param WdfDevice handle to a WDF device object.
 * @param PreviousState previous power state.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(WdfDevice);
    UNREFERENCED_PARAMETER(PreviousState);
    PAGED_CODE();

    return STATUS_SUCCESS;
}

NTSTATUS WdfCsi_ctx::EvtD0Exit(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE FxPreviousState)
/*!
 * Stop device on EvtD0Exit.
 *
 * @param WdfDevice handle to a WDF device object.
 * @param PreviousState previous power state.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(WdfDevice);
    UNREFERENCED_PARAMETER(FxPreviousState);
    PAGED_CODE();

    return STATUS_SUCCESS;
}

NTSTATUS WdfCsi_ctx::RegisterQueue()
/*!
 * Create default queue.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    WDFQUEUE queue;
    NTSTATUS status;
    WDF_IO_QUEUE_CONFIG wdfQueueConfig;

    PAGED_CODE();

    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&wdfQueueConfig, WdfIoQueueDispatchSequential);

    WDF_OBJECT_ATTRIBUTES wdfQueueAttributes;
    WDF_OBJECT_ATTRIBUTES_INIT(&wdfQueueAttributes);

    wdfQueueAttributes.ExecutionLevel = WdfExecutionLevelDispatch; // All parts of driver must execute at same level for Automatic synchronization to work.
    wdfQueueConfig.PowerManaged = WdfTrue;

    wdfQueueConfig.EvtIoDeviceControl = WdfCsi_ctx::EvtDeviceControl;
    wdfQueueConfig.EvtIoStop = WdfCsi_ctx::EvtIoStop;

    status = WdfIoQueueCreate(m_WdfDevice, &wdfQueueConfig, &wdfQueueAttributes, &queue);
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_QUEUE, "WdfIoQueueCreate failed %!STATUS!", status);
    }

    return status;
}

void WdfCsi_ctx::EvtDeviceControl(_In_ WDFQUEUE Queue, _In_ WDFREQUEST WdfRequest, _In_ size_t OutputBufferLength, _In_ size_t InputBufferLength, _In_ ULONG IoControlCode)
/*!
 * EvtDeviceControl calls handlers for known control requests.
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
    ASSERT(WdfRequest);
    UNREFERENCED_PARAMETER(OutputBufferLength);
    UNREFERENCED_PARAMETER(InputBufferLength);
    NTSTATUS status = STATUS_AUDIT_FAILED;
    auto ctxPtr = DeviceGetContext(WdfIoQueueGetDevice(Queue));
    auto requestCtxPtr = GetRequestContext(WdfRequest);
    ASSERT(ctxPtr != NULL);
    ASSERT(requestCtxPtr != NULL);

    _DbgFrameKdPrint(("WdfCsi_ctx::EvtDeviceControl(0x%x, 0x%x, 0x%x, %u).\r\n", Queue, WdfRequest, IoControlCode, (unsigned)KeGetCurrentIrql()));
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_QUEUE, "%!FUNC! Queue 0x%p, Request 0x%p OutputBufferLength %d InputBufferLength %d IoControlCode %d",
        Queue, WdfRequest, (int)OutputBufferLength, (int)InputBufferLength, IoControlCode);

    requestCtxPtr->m_WdfRequest = WdfRequest;
    requestCtxPtr->m_CtlCode = IoControlCode;

    if (WdfRequestIsCanceled(WdfRequest)) {
        WdfRequestComplete(WdfRequest, STATUS_CANCELLED);
    }
    else {
        switch (IoControlCode) {
        case IOCTL_CSI_DRIVER_INIT:
            {
                ctxPtr->ReinitializeRequest(requestCtxPtr);
                // ReinitializeRequest completes request on it's own. Don't call WdfRequestComplete().
            }
            break;
        case IOCTL_CSI_DRIVER_STOP:
            {
                ctxPtr->TerminateIo();
                status = STATUS_SUCCESS;
                // TerminateIo() doesn't complete request on itself.
                WdfRequestComplete(WdfRequest, status);
            }
            break;
        case IOCTL_CSI_DRIVER_GET_FRAME:
            {
                ctxPtr->EvtFrameRequest(requestCtxPtr);
                // EvtFrameRequest completes request on it's own. Don't call WdfRequestComplete().
            }
            break;
        default:
            {
                _DbgKdPrint(("WdfCsi_ctx::EvtDeviceControl: STATUS_INVALID_DEVICE_REQUEST\r\n"));
                status = STATUS_INVALID_DEVICE_REQUEST;
                WdfRequestComplete(WdfRequest, status);
            }
            break;
        }
    }
}

void WdfCsi_ctx::EvtDeviceFileCreate(WDFDEVICE WdfDevice, WDFREQUEST WdfRequest, WDFFILEOBJECT WdfFileObject)
/*!
 * Opens channel assigned to device file when it is opened and assigns it to newly allocated file context.
 *
 * @param WdfDevice handle to a WDF device object.
 * @param WdfRequest handle to a WDF request object.
 * @param WdfFileObject handle to a WDF request object.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(WdfDevice);
    PAGED_CODE();

    NTSTATUS status = STATUS_SUCCESS;
    PDEVICE_CONTEXT devCtxPtr = DeviceGetContext(WdfFileObjectGetDevice(WdfFileObject));

    NT_ASSERT(devCtxPtr != NULL);
    status = devCtxPtr->AcquireBuffers();
    WdfRequestComplete(WdfRequest, status);
}

void WdfCsi_ctx::EvtDeviceFileCleanup(WDFFILEOBJECT WdfFileObject)
/*!
 * Set device as free and ensures IO is stopped.
 *
 * @param WdfFileObject handle to a WDF request object.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    PAGED_CODE();

    PDEVICE_CONTEXT devCtxPtr = DeviceGetContext(WdfFileObjectGetDevice(WdfFileObject));

    NT_ASSERT(devCtxPtr != NULL);
    devCtxPtr->Close();
}

NTSTATUS WdfCsi_ctx::Close()
/*!
 * Nonstatic handler for EvtDeviceFileCleanup.
 *
 * @returns STATUS_SUCCESS.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    CsiResetAndStop();
    ReleaseBuffers();
    return status;
}

void WdfCsi_ctx::EvtDeviceFileClose(WDFFILEOBJECT WdfFileObject)
/*!
 * Dummy file close event handler.
 * Could be used to verify that device references are being handled properly by upper layer when logging is enabled.
 *
 * @param WdfFileObject handle to a WDF request object.
 *
 * @returns STATUS_SUCCESS.
 */
{
    PAGED_CODE();

    PDEVICE_FILE_CONTEXT fileObjectCtxPtr = DeviceGetFileContext(WdfFileObject);

    UNREFERENCED_PARAMETER(fileObjectCtxPtr);
}

void WdfCsi_ctx::EvtIoStop(_In_ WDFQUEUE Queue, _In_ WDFREQUEST WdfRequest, _In_ ULONG ActionFlags)
/*!
 * Calls WdfCsi_ctx::EvtWdfRequestCancel to cancel the request.
 *
 * @param Queue handle to a WDF queue object.
 * @param WdfRequest handle to a WDF request object.
 * @param ActionFlags requested action on the request object.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(Queue);
    UNREFERENCED_PARAMETER(ActionFlags);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_QUEUE, "%!FUNC! Queue 0x%p, Request 0x%p ActionFlags %d", Queue, WdfRequest, ActionFlags);

    EvtWdfRequestCancel(WdfRequest);
}

void WdfCsi_ctx::EvtWdfRequestCancel(WDFREQUEST WdfRequest)
/*!
 * Completes request as canceled.
 *
 * @param WdfRequest WDF request object to be canceled. Request should be the same as m_ActiveRequestCtxPtr.
 */
{
    DEVICE_CONTEXT *devCtxPtr = DeviceGetContext(WdfIoQueueGetDevice(WdfRequestGetIoQueue(WdfRequest)));

    devCtxPtr->TerminateIo();
    devCtxPtr->m_ActiveRequestCtxPtr = NULL;
    WdfRequestComplete(WdfRequest, STATUS_CANCELLED);
}

void WdfCsi_ctx::TerminateIo()
/*!
 * Stop the running CSI.
 */
{
    // Queue is synchronous with DPC and never races.
    // Actually only single request is in flight because of Serialized dispatch.
    KIRQL irql = KeGetCurrentIrql();

    if (irql <= DISPATCH_LEVEL) {
        WdfInterruptAcquireLock(m_IsrCtx->m_WdfInterrupt);
    }
    m_State = S_STOPPED;
    CsiStart(false);
    if (m_FinishedBuffPtr != NULL) {
        m_FinishedBuffPtr->state = m_FinishedBuffPtr->FREE;
        m_FinishedBuffPtr = NULL;
    }
    if (irql <= DISPATCH_LEVEL) {
        WdfInterruptReleaseLock(m_IsrCtx->m_WdfInterrupt);
    }
}

void WdfCsi_ctx::EvtFrameRequest(PREQUEST_CONTEXT RequestCtxPtr)
/*!
 * Handles get frame IRP.
 * EvtFrameRequest completes request on it's own. Don't use RequestCtxPtr after call.
 * Function either finishes IRP with ready image or setups request for DPC completion. Might also complete with error code if driver is in incorrect state.
 *
 * @param RequestCtxPtr pointer to request context.
 */
{
    NTSTATUS status = STATUS_SUCCESS;
    auto state = m_State;

    _DbgFrameKdPrint(("WdfCsi_ctx::EvtFrameRequest(s=0x%x)\r\n", state));
    ASSERT(RequestCtxPtr != NULL);
    ASSERT(RequestCtxPtr->m_WdfRequest != NULL);

    if (WdfRequestIsCanceled(RequestCtxPtr->m_WdfRequest)) {
        WdfRequestComplete(RequestCtxPtr->m_WdfRequest, STATUS_CANCELLED);
    }
    else {
        WdfRequestMarkCancelable(RequestCtxPtr->m_WdfRequest, EvtWdfRequestCancel);
        _DbgFrameKdPrint(("MarkCancelable %p\r\n", RequestCtxPtr->m_WdfRequest));
        {
            switch (state) {
            case S_FRAME_REQUESTED: // Finish IoRequest. Could modify state to discarding if EvtStop didn't occur. Other IoRequests are serialized by queue.
                _DbgFrameKdPrint(("WdfCsi_ctx::EvtFrameRequest called in S_FRAME_REQUESTED state. (only single allowed rn)\r\n", state));
                if (m_ActiveRequestCtxPtr != NULL) {
                    break;
                }
                // Otherwise debug assert and continue as if state == S_DISCARDING.
                ASSERT(m_ActiveRequestCtxPtr != NULL);

            case S_DISCARDING:
                {
                    const FrameInfo_t *infoPtr = NULL;

                    status = WdfRequestRetrieveInputBuffer(RequestCtxPtr->m_WdfRequest, sizeof(FrameInfo_t), &((PVOID)infoPtr), NULL);
                    if (NT_SUCCESS(state)) {
                        RtlCopyMemory((PVOID)&(RequestCtxPtr->m_FrameInfo), infoPtr, sizeof(FrameInfo_t));
                        if (!NT_SUCCESS(status)) {
                            _DbgFrameKdPrint(("EvtFrameRequest::WdfRequestRetrieveInputBuffer(s=0x%x)\r\n", state));
                        }
                        else {
                            auto *fInfoPtr = &RequestCtxPtr->m_FrameInfo;
                            status = WdfRequestRetrieveOutputBuffer(RequestCtxPtr->m_WdfRequest, 1280 * 720 * 2, &(PVOID)fInfoPtr->m_Fb, &fInfoPtr->m_FbSize);
                            if (!NT_SUCCESS(status)) {
                                _DbgFrameKdPrint(("EvtFrameRequest::WdfRequestRetrieveOutputBuffer(s=0x%x)\r\n", state));
                            }
                            else {
                                _DbgFrameKdPrint(("FrameInfo_t\r\n\t(Virtual=0x%x)\r\n\t(Fb=0x%p)\r\n\t(ByteCount=0x%x)\r\n\t(FbSize=0x%x)\r\n\t(StrideValid=0x%x)\r\n\t\r\n",
                                    fInfoPtr->Virtual,
                                    fInfoPtr->Fb,
                                    fInfoPtr->ByteCount,
                                    fInfoPtr->FbSize,
                                    fInfoPtr->StrideValid
                                    ));
                                fInfoPtr->m_RequestCtxPtr = RequestCtxPtr;

                                WdfInterruptAcquireLock(m_IsrCtx->m_WdfInterrupt);
                                {
                                    // m_State, m_FinishedBuffPtr and m_ActiveRequestCtxPtr has to be handled atomically. Because these variables change ISR behavior.

                                    state = S_FRAME_REQUESTED; // Stop ISR from reusing m_FinishedBuffPtr.
                                    m_State = state; // Routine was originally designed to get state, do processing and write new state, but it has been optimized to finish IRP here.
                                    if (m_FinishedBuffPtr != NULL) { // Check if finished image is available.
                                        m_ActiveRequestCtxPtr = NULL; // Disable DPC
                                    }
                                    else {
                                        m_ActiveRequestCtxPtr = RequestCtxPtr; // Enable DPC
                                        RequestCtxPtr = NULL;
                                    }
                                }
                                WdfInterruptReleaseLock(m_IsrCtx->m_WdfInterrupt);

                                if (RequestCtxPtr) {
                                    ASSERT(m_ActiveRequestCtxPtr == NULL);
                                    FinishGetFrameRequest(RequestCtxPtr); // This must not be IRQL
                                    state = m_State;
                                    RequestCtxPtr = NULL;
                                }
                            }
                        }
                    }
                }
                break;
            case S_STOPPED:
                status = STATUS_DEVICE_NOT_READY;
                break;
            default:
                status = STATUS_DEVICE_NOT_READY;
                m_State = S_STOP_REQUESTED;
                break;
            }
        }
        if (m_ActiveRequestCtxPtr == NULL && !NT_SUCCESS(status)) {
            WdfRequestUnmarkCancelable(RequestCtxPtr->m_WdfRequest);
            _DbgFrameKdPrint(("UnmarkCancelable %p\r\n", RequestCtxPtr->m_WdfRequest));
            WdfRequestComplete(RequestCtxPtr->m_WdfRequest, status);
        }
        _DbgFrameKdPrint(("-freq %x , 0x%p.\r\n", m_State, m_ActiveRequestCtxPtr));
    }
}

void WdfCsi_ctx::ReinitializeRequest(PREQUEST_CONTEXT RequestCtxPtr)
/*!
 * Reinitialize the CSI peripheral to given configuration.
 * ReinitializeRequest completes request on it's own. Don't use RequestCtxPtr after call.
 *
 * @param RequestCtxPtr context of request containing video stream properties (resolution, frame rate, color space ..).
 */
{
    NTSTATUS status;
    ASSERT(RequestCtxPtr != NULL);
    ASSERT(RequestCtxPtr->m_WdfRequest != NULL);

    if (WdfRequestIsCanceled(RequestCtxPtr->m_WdfRequest)) {
        status = STATUS_CANCELLED;
    }
    else {
        camera_config_t *configPtr = NULL;

        status = WdfRequestRetrieveInputBuffer(RequestCtxPtr->m_WdfRequest, sizeof(camera_config_t), &(PVOID)configPtr, NULL);
        if (NT_SUCCESS(status)) {
            WdfInterruptAcquireLock(m_IsrCtx->m_WdfInterrupt);

            status = CsiInit(*configPtr);
            if (NT_SUCCESS(status)) {
                CsiStart(true); // Enables IRQ
                m_State = S_DISCARDING;
                WdfInterruptReleaseLock(m_IsrCtx->m_WdfInterrupt);
            }
            else {
                CsiStart(false);
                WdfInterruptReleaseLock(m_IsrCtx->m_WdfInterrupt);
            }
        }
    }
    WdfRequestComplete(RequestCtxPtr->m_WdfRequest, status);
}

NTSTATUS WdfCsi_ctx::AllocFb(DiscardBuffInfo_t &buffInfo)
/*!
 * Allocated buffer for common buffer DMA.
 *
 * @param buffInfo Storage for allocated buffer information.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    _DbgKdPrint(("WdfCsi_ctx::AllocFb+\r\n"));

    NTSTATUS status = STATUS_SUCCESS;
    static const PHYSICAL_ADDRESS zero = { 0,0 };
    static const PHYSICAL_ADDRESS high = { 0xffffffff, 0 }; // this is required by MmAllocatePagesForMdlEx

    buffInfo.mdlPtr = MmAllocatePagesForMdlEx(zero, high, zero, m_BufferRequiredSize, MmCached, MM_ALLOCATE_FULLY_REQUIRED | MM_ALLOCATE_REQUIRE_CONTIGUOUS_CHUNKS);
    if (buffInfo.mdlPtr == NULL) {
        _DbgKdPrint(("WdfCsi_ctx::AllocFb Out of memory.\r\n"));
        status = STATUS_INSUFFICIENT_RESOURCES;
    }

    UNREFERENCED_PARAMETER(buffInfo);
    return status;
}

NTSTATUS WdfCsi_ctx::AcquireBuffers()
/*!
 * Prepare all allocated buffers for common buffer mode DMA address.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    for (unsigned i = 0; ((i < 3) && NT_SUCCESS(status)); ++i) {
        status = MapFb(m_DiscardBuff[i]);
    }
    return status;
}

NTSTATUS WdfCsi_ctx::MapFb(DiscardBuffInfo_t &buffInfo)
/*!
 * Get allocated buffer for common buffer mode DMA address.
 *
 * @param buffInfo Storage for allocated buffer information.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_INSUFFICIENT_RESOURCES;
    static const PHYSICAL_ADDRESS zero = { 0,0 };           // this is required by MmAllocatePagesForMdlEx
    static const PHYSICAL_ADDRESS high = { 0xffffffff, 0 }; // this is required by MmAllocatePagesForMdlEx

    buffInfo.virt = MmMapLockedPagesSpecifyCache(buffInfo.mdlPtr, KernelMode, MmCached, NULL, FALSE, HighPagePriority);
    if (buffInfo.virt != NULL) {
        buffInfo.phys = MmGetPhysicalAddress(buffInfo.virt);
        if (buffInfo.phys.QuadPart != NULL) {
            status = STATUS_SUCCESS;
        }
    }
    return status;
}

NTSTATUS WdfCsi_ctx::ReleaseBuffers()
/*!
 * Prepare all allocated buffers for common buffer mode DMA address.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    for (unsigned i = 0; ((i < COMMON_FRAME_BUFFER_NUM) && NT_SUCCESS(status)); ++i) {
        status = UnmapFb(m_DiscardBuff[i]);
    }
    return status;
}

NTSTATUS WdfCsi_ctx::UnmapFb(DiscardBuffInfo_t& buffInfo)
/*!
 * Get allocated buffer for common buffer mode DMA address.
 *
 * @param buffInfo Storage for allocated buffer information.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    if (buffInfo.mdlPtr != NULL) {
        if (buffInfo.virt != NULL) {
            MmUnmapLockedPages(buffInfo.virt, buffInfo.mdlPtr);
            buffInfo.virt = NULL;
        }
    }
    return STATUS_SUCCESS;
}

void WdfCsi_ctx::FreeFb(DiscardBuffInfo_t &buffInfo)
/*!
 * Free allocated buffer for common buffer mode DMA.
 *
 * @param buffInfo Storage with allocated buffer information.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    _DbgKdPrint(("WdfCsi_ctx::FreeFb+\r\n"));
    if (buffInfo.mdlPtr != NULL) {
        if (buffInfo.virt != NULL) {
            _DbgKdPrint(("MmUnmapLockedPages+\r\n"));
            MmUnmapLockedPages(buffInfo.virt, buffInfo.mdlPtr);
            buffInfo.virt = NULL;
        }
        _DbgKdPrint(("MmFreePagesFromMdl+\r\n"));
        MmFreePagesFromMdl(buffInfo.mdlPtr);
        ExFreePool(buffInfo.mdlPtr);
        buffInfo.mdlPtr = NULL;
    }
}

// IRQ Handling ----------------

NTSTATUS CsiIsrCtx_t::registerInterruptHandler(PDEVICE_CONTEXT CtxPtr)
/*!
 * Create an interrupt object with Isr and Dpc callbacks.
 *
 * @param CtxPtr interrupt context.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{ // Create an interrupt object
    NTSTATUS status;
    WDF_OBJECT_ATTRIBUTES attributes;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, DEVICE_INTERRUPT_CONTEXT);
    WDF_INTERRUPT_CONFIG interruptConfig;

    interruptConfig.AutomaticSerialization = TRUE;
    WDF_INTERRUPT_CONFIG_INIT(&interruptConfig, EvtInterruptIsr, EvtInterruptDpc);
    interruptConfig.EvtInterruptEnable = EvtWdfInterruptEnable;
    interruptConfig.EvtInterruptDisable = EvtWdfInterruptDisable;
    {
        WDFINTERRUPT wdfInterrupt;

        status = WdfInterruptCreate(CtxPtr->m_WdfDevice, &interruptConfig, &attributes, &wdfInterrupt);
        if (NT_SUCCESS(status)) {
            PDEVICE_INTERRUPT_CONTEXT interruptContextPtr = new (DeviceGetInterruptContext(wdfInterrupt)) DEVICE_INTERRUPT_CONTEXT(CtxPtr, wdfInterrupt);

            CtxPtr->m_IsrCtx = interruptContextPtr;
        }
    }
    return status;
}

void WdfCsi_ctx::FinishGetFrameRequest(PREQUEST_CONTEXT RequestCtxPtr)
/*!
 * Completes get frame IRP.
 * Function finishes IRP with ready image or cancels it.
 * Asserts when there's no image in m_FinishedBuffPtr or request is not IOCTL_CSI_DRIVER_GET_FRAME.
 * Function also dispatches next IO request as side effect of CompleteRequest() call.
 *
 * @param RequestCtxPtr exclusive pointer to request context, caller must remove request from Device Context prior this call.
 */
{
    ASSERT(m_ActiveRequestCtxPtr == NULL);

    auto WdfRequest = RequestCtxPtr->m_WdfRequest;
    ASSERT(WdfRequest != NULL);
    ASSERT(RequestCtxPtr->m_CtlCode == IOCTL_CSI_DRIVER_GET_FRAME);

    auto *finishedBuffPtr = m_FinishedBuffPtr; // Keep a copy, m_FinishedBuffPtr will be freed later to enable ISR.
    ASSERT(finishedBuffPtr != NULL); // Function must be called only when there is m_FinishedBuffPtr
    ASSERT(finishedBuffPtr->state == finishedBuffPtr->DONE);

    // Make sure to do not modify Device context after WdfRequestComplete as Request Dispatch gets invoked there.
    if ((finishedBuffPtr != NULL) && (m_CsiRegistersPtr != NULL) && (finishedBuffPtr->state == finishedBuffPtr->DONE)) {
        NTSTATUS status = WdfRequestUnmarkCancelable(WdfRequest);
        auto *fInfoPtr = &RequestCtxPtr->m_FrameInfo;
        size_t imSize = min(m_BufferRequiredSize, m_FrameLenBytes);

        ++m_CompleteFrameCnt;
        switch (status) {
        case STATUS_SUCCESS:
            {
                if (fInfoPtr->m_Fb != NULL) {
                    if ((m_CsiRegistersPtr->DMASA_FB1 == (UINT32)finishedBuffPtr->phys.QuadPart)
                        || (m_CsiRegistersPtr->DMASA_FB2 == (UINT32)finishedBuffPtr->phys.QuadPart))
                    {
                        _DbgFrameKdPrint(("Error: Frame with artifacts - finishedBuffPtr is in use 0x%p.\r\n", finishedBuffPtr->phys.QuadPart));
                        imSize = 0;
                        status = STATUS_IO_TIMEOUT;
                    }
                    else {
                        status = STATUS_SUCCESS;
                    }
                    RtlCopyMemory(fInfoPtr->m_Fb, finishedBuffPtr->virt, imSize);


                    WdfRequestCompleteWithInformation(WdfRequest, status, imSize); // Dispatch new request, sets state to IOCTL_CSI_DRIVER_GET_FRAME
                }
                else {
                    status = STATUS_INVALID_PARAMETER;
                    WdfRequestComplete(WdfRequest, STATUS_INVALID_PARAMETER); // Dispatch new request, sets state to IOCTL_CSI_DRIVER_GET_FRAME
                }
            }
            break;

        default: // Could be STATUS_CANCELLED(IRP is ownced by Cancel() routine), STATUS_INVALID_DEVICE_REQUEST or STATUS_INVALID_PARAMETER. Lets change state to discarding.
            break;
        }

#if (DBG)
        RtlZeroMemory(finishedBuffPtr->virt, imSize); // Debug partial image.
#endif
        finishedBuffPtr->state = finishedBuffPtr->FREE; // Mark buff as free. ISR still waits for m_FinishedBuffPtr to become NULL.
        m_FinishedBuffPtr = NULL; // ISR Will fill add newly finished buffer and give CSI next free buffer.
        m_State = WdfCsi_ctx::S_DISCARDING; // Switch ISR to reuse Finished buffs

    }
}

BOOLEAN CsiIsrCtx_t::EvtInterruptIsr(_In_ WDFINTERRUPT WdfInterrupt, _In_ ULONG MessageID)
/*!
 * EvtInterruptIsr callback handles CSI errors and schedules DPC for finished frames.
 *
 * @param WdfInterrupt handle to WDF interrupt object.
 * @param MessageID device is not using MSI, always zero.
 *
 * @returns TRUE if interrupt has been serviced.
 */
{
    UNREFERENCED_PARAMETER(MessageID);
    BOOLEAN scheduleDpc = false;
    BOOLEAN servicedIrq = false;
    WdfCsi_ctx *devCtxPtr = DeviceGetContext(WdfInterruptGetDevice(WdfInterrupt));

    ASSERT(devCtxPtr);
    auto registersPtr = devCtxPtr->m_CsiRegistersPtr;
    ASSERT(registersPtr);

    UINT32 csiSr = registersPtr->SR;
    BOOLEAN csiErrorIrqFired = (0 < (csiSr & devCtxPtr->m_CurrSrErrorMask)); // SR_INT_MASK
    BOOLEAN csiFrameIrqFired = (0 < (csiSr & SR_SOF_INT_BIT)); // SR_INT_MASK

    if (csiErrorIrqFired | csiFrameIrqFired) {
        servicedIrq = 1;
        _DbgFrameKdPrint(("Isr %x , 0x%p.\r\n", devCtxPtr->m_State, devCtxPtr->m_ActiveRequestCtxPtr));
        if ((devCtxPtr->m_State != WdfCsi_ctx::S_DISCARDING)
            && (devCtxPtr->m_State != WdfCsi_ctx::S_FRAME_REQUESTED))
        { // Peripheral is supposed to be stopped.
            devCtxPtr->CsiStart(false);
            devCtxPtr->m_State = WdfCsi_ctx::S_STOPPED;
            // Interrupts disabled and DMA stopped.
        }
        else {
            if (devCtxPtr->DpcCsiResetDmaIfError(csiSr)) {
                // Enabled(S_DISCARDING or S_FRAME_REQUESTED) and errors, just re-enabling
                devCtxPtr->CsiStart(true);
                devCtxPtr->EnableErrorInterruptsNofence(true, true);
                devCtxPtr->EnableFrameInterruptsNofence(true, true);
                // IRQ enabled. Don't let both F1 and F2 done flags set.
            }
            else {
                // Enabled and no errors
                if (csiFrameIrqFired
                    && (((devCtxPtr->m_State == WdfCsi_ctx::S_FRAME_REQUESTED) && (devCtxPtr->m_FinishedBuffPtr == NULL))
                       || (devCtxPtr->m_State == WdfCsi_ctx::S_DISCARDING))
                   )
                {
                    UINT8 frameId = devCtxPtr->CsiPopSrFrameId(devCtxPtr->m_CsiRegistersPtr, csiSr);
                    // auto requestCtxPtr = devCtxPtr->m_ActiveRequestCtxPtr;
                    WdfCsi_ctx::DiscardBuffInfo_t *freeBuffPtr = NULL;
                    WdfCsi_ctx::DiscardBuffInfo_t *finishedBuffPtr = NULL;

                    if ((devCtxPtr->m_State == WdfCsi_ctx::S_DISCARDING) && (devCtxPtr->m_FinishedBuffPtr != NULL)) {
                        devCtxPtr->m_FinishedBuffPtr->state = devCtxPtr->m_FinishedBuffPtr->FREE;
                        devCtxPtr->m_FinishedBuffPtr = NULL;
                    }

                    if (frameId > 0) {
                        UINT64 donePhys = ((frameId == 1) ? devCtxPtr->m_CsiRegistersPtr->DMASA_FB1 : devCtxPtr->m_CsiRegistersPtr->DMASA_FB2);
                        // Search buffer lists for free and finished buffer.
                        for (int i = 0; i < 3; ++i) {
                            auto &buff = devCtxPtr->m_DiscardBuff[i];

                            if ((buff.state == buff.FREE) && (freeBuffPtr == NULL)) {
                                freeBuffPtr = &buff;
                            }
                            if (((UINT64)buff.phys.QuadPart == donePhys)) {
                                ASSERT(buff.state == buff.WORKING);
                                finishedBuffPtr = &buff;
                            }
                        }

                        if ((freeBuffPtr != NULL) && (finishedBuffPtr != NULL)) {
                            if (frameId == 1) {
                                devCtxPtr->m_CsiRegistersPtr->DMASA_FB1 = (UINT32)freeBuffPtr->phys.QuadPart;
                            }
                            else {
                                devCtxPtr->m_CsiRegistersPtr->DMASA_FB2 = (UINT32)freeBuffPtr->phys.QuadPart;
                            }
                            freeBuffPtr->state = finishedBuffPtr->WORKING;
                            finishedBuffPtr->state = finishedBuffPtr->DONE;
                            devCtxPtr->m_FinishedBuffPtr = finishedBuffPtr;
                            // The S_FRAME_REQUESTED could be set to disable reuse of finished buff although request will be finished on dispatch.
                            if ((devCtxPtr->m_State == WdfCsi_ctx::S_FRAME_REQUESTED)
                                && (devCtxPtr->m_ActiveRequestCtxPtr != NULL)) {
                                scheduleDpc = true;
                            }
                        }
                        else {
                            if (finishedBuffPtr != NULL) {
#if (DBG)
                                size_t imSize = min(devCtxPtr->m_BufferRequiredSize, 1280 * 720 * 2);
                                RtlZeroMemory(finishedBuffPtr->virt, imSize); // Debug partial image.
#endif
                            }
                        }
                    }
                }

                // Just clear flags. // In non S_FRAME_REQUESTED don't care which frame arrived.
                devCtxPtr->EnableErrorInterruptsNofence(true, true);
                devCtxPtr->EnableFrameInterruptsNofence(true, true);
                // IRQ enabled. Don't let both F1 and F2 done flags set.
            }
            // Interrupts enabled and DMA running.
        }
    }
    if (scheduleDpc) {
        _DbgFrameKdPrint(("ScheduleDpc\r\n"));
        WdfInterruptQueueDpcForIsr(WdfInterrupt);
    }
    // TODO Consider barrier forIRQ disable write.
    return servicedIrq;
}

void CsiIsrCtx_t::EvtInterruptDpc(_In_ WDFINTERRUPT WdfInterrupt, _In_ WDFOBJECT AssociatedObject)
/*!
 * EvtInterruptDpc callback finishes get frame IRP.
 * Device AutomaticSerialization is at DPC level so DPC and Dispatch doesn't race except the WdfRequestComplete() call.
 *
 * @param WdfInterrupt handle to WDF interrupt object.
 * @param AssociatedObject handle to associated WDF object.
 */
{
    UNREFERENCED_PARAMETER(AssociatedObject);

    WdfCsi_ctx *devCtxPtr = DeviceGetContext(WdfInterruptGetDevice(WdfInterrupt));
    ASSERT(devCtxPtr);
    ASSERT(devCtxPtr->m_ActiveRequestCtxPtr != NULL);
    ASSERT(devCtxPtr->m_State == WdfCsi_ctx::S_FRAME_REQUESTED);

    auto *requestCtxPtr = devCtxPtr->m_ActiveRequestCtxPtr;

    devCtxPtr->m_ActiveRequestCtxPtr = NULL;  // Disable DPC
    devCtxPtr->FinishGetFrameRequest(requestCtxPtr);
}

CsiIsrCtx_t::CsiIsrCtx_t(PDEVICE_CONTEXT CtxPtr, WDFINTERRUPT WdfInterrupt)
    : m_CtxPtr(CtxPtr), m_WdfInterrupt(WdfInterrupt)
/*!
 * CsiIsrCtx_t constructor initializes context to defaults.
 *
 * @param ctxPtr pointer to device context.
 * @param wdfInterrupt handle to WDF interrupt object.
 */
{}

NTSTATUS CsiIsrCtx_t::EvtWdfInterruptEnable(WDFINTERRUPT Interrupt, WDFDEVICE AssociatedDevice)
/*!
 * EvtWdfInterruptEnable callback.
 *
 * @param Interrupt handle to WDF interrupt object.
 * @param AssociatedDevice handle to WDF device object.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(Interrupt);
    UNREFERENCED_PARAMETER(AssociatedDevice);

    return STATUS_SUCCESS;
}

NTSTATUS CsiIsrCtx_t::EvtWdfInterruptDisable(WDFINTERRUPT Interrupt, WDFDEVICE AssociatedDevice)
/*!
 * EvtWdfInterruptDisable callback.
 *
 * @param Interrupt handle to WDF interrupt object.
 * @param AssociatedDevice handle to WDF device object.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(Interrupt);

    auto ctxPtr = DeviceGetContext(AssociatedDevice);

    ASSERT(ctxPtr != NULL);
    ctxPtr->TerminateIo();
    return STATUS_SUCCESS;
}
