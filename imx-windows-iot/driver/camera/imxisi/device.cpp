
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

#include "imxisi.h"
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
        attributes.EvtCleanupCallback = WdfIsi_ctx::EvtDriverContextCleanup;
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

VOID WdfIsi_ctx::EvtDriverContextCleanup(_In_ WDFOBJECT DriverObject)
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

NTSTATUS WdfIsi_ctx::EvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit)
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

        pnpCallbacks.EvtDevicePrepareHardware = WdfIsi_ctx::EvtPrepareHw;
        pnpCallbacks.EvtDeviceReleaseHardware = WdfIsi_ctx::EvtReleaseHw;
        pnpCallbacks.EvtDeviceD0Entry = WdfIsi_ctx::EvtD0Entry;
        pnpCallbacks.EvtDeviceD0Exit = WdfIsi_ctx::EvtD0Exit;

        WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);
        WdfDeviceInitSetIoType(DeviceInit, WdfDeviceIoDirect);
    }

    { // File object
        WDF_FILEOBJECT_CONFIG wdfFileObjectConfig;
        WDF_FILEOBJECT_CONFIG_INIT(
            &wdfFileObjectConfig,
            WdfIsi_ctx::EvtDeviceFileCreate,
            WdfIsi_ctx::EvtDeviceFileClose,
            WdfIsi_ctx::EvtDeviceFileCleanup);

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

WdfIsi_ctx::WdfIsi_ctx(WDFDEVICE &WdfDevice)
    : m_WdfDevice(WdfDevice),
    m_Isi1Reg(WdfDevice),
    m_IsiFbMemReserve(WdfDevice, PAGE_READWRITE),
    m_MaxMipiCsiSrc(1),
    m_DsdRes(WdfDeviceWdmGetPhysicalDevice(WdfDevice)),
    m_NumPlanes(1),
    m_QuirkInvertFrameId(0)
/*!
 * Initialize device context.
 *
 * @param WdfDevice handle to a WDF device object.
 */
{
    PAGED_CODE();
}

NTSTATUS WdfIsi_ctx::Get_DsdAcpiResources()
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
                "Isi1RegResId",
                &m_Isi1RegResId,
                (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
            },
            {
                "MipiCsiSrc",
                &m_MipiCsiSrc,
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
            _DbgKdPrint(("WdfIsi_ctx::DsdRes.GetDsdResources Fail\r\n"));
        }
        if (NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_CpuId = 0x%x", m_CpuId);
            switch (m_CpuId) {
            case IMX_CPU_MX8MP:
            case IMX_CPU_MX8QXP:
                m_QuirkInvertFrameId = 1;
                break;
            default:
                m_QuirkInvertFrameId = 0;
                break;
            }

            switch (m_CpuId) {
            case IMX_CPU_MX8QXP:
                m_MaxMipiCsiSrc = 5;
                break;
            case IMX_CPU_MX93:
                m_MaxMipiCsiSrc = 2;
                break;
            default:
                m_MaxMipiCsiSrc = 1;
                break;
            }
        }
        if (NT_SUCCESS(status)) {
            NTSTATUS tmpStatus = m_DsdRes.GetInteger("IsiFbMemReserveResId", &(m_IsiFbMemReserveResId));
            if (!NT_SUCCESS(tmpStatus)) {
                m_IsiHasFbMemReserveRes = false;
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Failed to query ACPI IsiFbMemReserveResId value, Framebuffers will be allocated later. (status = %!STATUS!)", status);
            }
            else {
                m_IsiHasFbMemReserveRes = true;
            }
        }
    }
    return status;
}

NTSTATUS WdfIsi_ctx::EvtPrepareHw(_In_ WDFDEVICE WdfDevice, _In_ WDFCMRESLIST ResourcesRaw, _In_ WDFCMRESLIST ResourcesTranslated)
/*!
 * WdfIsi_ctx::EvtPrepareHw bridges to non-static method WdfIsi_ctx::PrepareHw.
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

NTSTATUS WdfIsi_ctx::PrepareHw(WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
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
            if (NULL != (memRes = m_MeResList.at(m_Isi1RegResId))) {
                status = m_Isi1Reg.IoSpaceMap(*memRes);
            }
            else {
                status = STATUS_INSUFFICIENT_RESOURCES; // Memory is required, report STATUS_INSUFFICIENT_RESOURCES if none.
            }
        }
        if (NT_SUCCESS(status)) {
            if (m_IsiHasFbMemReserveRes) {
                if (NULL != (memRes = m_MeResList.at(m_IsiFbMemReserveResId))) {
                    m_IsiHasFbMemReserveRes = 0;
                }
                else {
                    status = STATUS_INSUFFICIENT_RESOURCES; // Memory is required, report STATUS_INSUFFICIENT_RESOURCES if none.
                }
            }
        }
        if (NT_SUCCESS(status)) {
            m_State = S_STOPPED;
            for (unsigned i = 0; ((i < COMMON_FRAME_BUFFER_NUM) && NT_SUCCESS(status)); ++i) {
                status = AllocFb(m_DiscardBuff[i]);
            }
            if (NT_SUCCESS(status)) {
                m_IsiRes.m_IsiRegistersPtr = m_Isi1Reg.Reg();
                if (NT_SUCCESS(status)) {
                    m_IsiRegistersPtr = m_IsiRes.m_IsiRegistersPtr;
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
                    _DbgKdPrint(("WdfIsi_ctx::PrepareHw: Failed to create interface 0x%x.\r\n", status));
                }
            }
#endif
        }
    }
    return status;
}

NTSTATUS WdfIsi_ctx::EvtReleaseHw(_In_ WDFDEVICE WdfDevice, _In_ WDFCMRESLIST ResourcesTranslated)
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

    deviceContextPtr->FreeFb(deviceContextPtr->m_DiscardBuff[0]);
    deviceContextPtr->FreeFb(deviceContextPtr->m_DiscardBuff[1]);
    deviceContextPtr->FreeFb(deviceContextPtr->m_DiscardBuff[2]);

    regBase alocs[] = {
        deviceContextPtr->m_Isi1Reg,
    };

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

NTSTATUS WdfIsi_ctx::EvtD0Entry(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE PreviousState)
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

NTSTATUS WdfIsi_ctx::EvtD0Exit(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE FxPreviousState)
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

NTSTATUS WdfIsi_ctx::RegisterQueue()
/*!
 * Create default queue.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    WDFQUEUE queue;
    NTSTATUS status;
    WDF_IO_QUEUE_CONFIG wdfQueueConfig;
    WDF_OBJECT_ATTRIBUTES wdfQueueAttributes;

    PAGED_CODE();

    WDF_OBJECT_ATTRIBUTES_INIT(&wdfQueueAttributes);
    wdfQueueAttributes.ExecutionLevel = WdfExecutionLevelDispatch; // All parts of driver must execute at same level for Automatic synchronization to work.

    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&wdfQueueConfig, WdfIoQueueDispatchSequential);
    wdfQueueConfig.PowerManaged = WdfTrue;
    wdfQueueConfig.EvtIoDeviceControl = WdfIsi_ctx::EvtDeviceControl;
    wdfQueueConfig.EvtIoStop = WdfIsi_ctx::EvtIoStop;

    status = WdfIoQueueCreate(m_WdfDevice, &wdfQueueConfig, &wdfQueueAttributes, &queue);

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_QUEUE, "WdfIoQueueCreate failed %!STATUS!", status);
    }

    return status;
}

void WdfIsi_ctx::EvtDeviceControl(_In_ WDFQUEUE Queue, _In_ WDFREQUEST WdfRequest, _In_ size_t OutputBufferLength, _In_ size_t InputBufferLength, _In_ ULONG IoControlCode)
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

    _DbgFrameKdPrint(("WdfIsi_ctx::EvtDeviceControl(0x%x, 0x%x, 0x%x, %u).\r\n", Queue, WdfRequest, IoControlCode, (unsigned)KeGetCurrentIrql()));
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_QUEUE,
        "%!FUNC! Queue 0x%p, Request 0x%p OutputBufferLength %d InputBufferLength %d IoControlCode %d",
        Queue, WdfRequest, (int)OutputBufferLength, (int)InputBufferLength, IoControlCode);

    requestCtxPtr->m_WdfRequest = WdfRequest;
    requestCtxPtr->m_CtlCode = IoControlCode;
    if (WdfRequestIsCanceled(WdfRequest)) {
        WdfRequestComplete(WdfRequest, STATUS_CANCELLED);
    }
    else {
        switch (IoControlCode) {
        case IOCTL_ISI_DRIVER_INIT:
            {
                _DbgKdPrint(("IOCTL_ISI_DRIVER_INIT\r\n"));
                ctxPtr->ReinitializeRequest(requestCtxPtr);
                // ReinitializeRequest completes request on it's own. Don't call WdfRequestComplete().
            }
            break;
        case IOCTL_ISI_DRIVER_STOP:
            {
                _DbgKdPrint(("IOCTL_ISI_DRIVER_STOP\r\n"));
                ctxPtr->TerminateIo();
                status = STATUS_SUCCESS;
                // TerminateIo() doesn't complete request on itself.
                WdfRequestComplete(WdfRequest, status);
            }
            break;
        case IOCTL_ISI_DRIVER_GET_FRAME:
            {
                _DbgFrameKdPrint(("IOCTL_ISI_DRIVER_GET_FRAME\r\n"));
                ctxPtr->EvtFrameRequest(requestCtxPtr);
                // EvtFrameRequest completes request on it's own. Don't call WdfRequestComplete().
            }
            break;
        case IOCTL_ISI_REQUIRED_FMT:
        {
            _DbgKdPrint(("IOCTL_ISI_REQUIRED_FMT\r\n"));
            ctxPtr->EvtInputFormatRequest(requestCtxPtr);
            // EvtFrameRequest completes request on it's own. Don't call WdfRequestComplete().
        }
        break;
        default:
            {
                _DbgKdPrint(("WdfIsi_ctx::EvtDeviceControl: STATUS_INVALID_DEVICE_REQUEST\r\n"));
                status = STATUS_INVALID_DEVICE_REQUEST;
                WdfRequestComplete(WdfRequest, status);
            }
            break;
        }
    }
}

void WdfIsi_ctx::EvtDeviceFileCreate(WDFDEVICE WdfDevice, WDFREQUEST WdfRequest, WDFFILEOBJECT WdfFileObject)
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

void WdfIsi_ctx::EvtDeviceFileCleanup(WDFFILEOBJECT WdfFileObject)
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

NTSTATUS WdfIsi_ctx::Close()
/*!
 * Nonstatic handler for EvtDeviceFileCleanup.
 *
 * @returns STATUS_SUCCESS.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    IsiResetAndStop();
    ReleaseBuffers();
    return status;
}

void WdfIsi_ctx::EvtDeviceFileClose(WDFFILEOBJECT WdfFileObject)
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
    UNREFERENCED_PARAMETER(WdfFileObject);
    // If file context is needed use PDEVICE_FILE_CONTEXT fileObjectCtxPtr = DeviceGetFileContext(WdfFileObject);.
}

void WdfIsi_ctx::EvtIoStop(_In_ WDFQUEUE Queue, _In_ WDFREQUEST WdfRequest, _In_ ULONG ActionFlags)
/*!
 * Calls WdfIsi_ctx::EvtWdfRequestCancel to cancel the request.
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

void WdfIsi_ctx::EvtWdfRequestCancel(WDFREQUEST WdfRequest)
/*!
 * Completes request as canceled.
 *
 * @param WdfRequest WDF request object to be canceled. Request should be the same as m_ActiveRequestCtxPtr.
 */
{
    PDEVICE_CONTEXT devCtxPtr = DeviceGetContext(WdfIoQueueGetDevice(WdfRequestGetIoQueue(WdfRequest)));
#if DBG
    PREQUEST_CONTEXT requestCtxPtr = GetRequestContext(WdfRequest);
    auto* activeRequestCtxPtr = InterlockedExchangePointer(&((void*)devCtxPtr->m_ActiveRequestCtxPtr), NULL);

    ASSERT(requestCtxPtr == activeRequestCtxPtr); // This must not occur. WdfQueue serializes requests.
#else
    devCtxPtr->m_ActiveRequestCtxPtr = NULL;
#endif
    devCtxPtr->TerminateIo();
    WdfRequestComplete(WdfRequest, STATUS_CANCELLED);
}

void WdfIsi_ctx::TerminateIo()
/*!
 * Stop the running CSI.
 */
{
    // Queue is synchronous with DPC and never races.
    // Actually only single request is in flight because of Serialized dispatch.
    KIRQL irql = KeGetCurrentIrql();
    bool locked = false;

    if (irql <= DISPATCH_LEVEL) {
        WdfInterruptAcquireLock(m_IsrCtx->m_WdfInterrupt);
        locked = true;
    }
    m_State = S_STOPPED;
    IsiStart(false);
    if (m_FinishedBuffPtr != NULL) {
        m_FinishedBuffPtr->state = m_FinishedBuffPtr->FREE;
        m_FinishedBuffPtr = NULL;
    }
    if (locked) {
        WdfInterruptReleaseLock(m_IsrCtx->m_WdfInterrupt);
    }
}

void WdfIsi_ctx::EvtFrameRequest(PREQUEST_CONTEXT RequestCtxPtr)
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

    _DbgFrameKdPrint(("WdfIsi_ctx::EvtFrameRequest(s=0x%x)\r\n", state));
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
            case S_FRAME_REQUESTED:
                _DbgFrameKdPrint(("WdfIsi_ctx::EvtFrameRequest called in S_FRAME_REQUESTED state. (only single allowed rn)\r\n", state));
            case S_DISCARDING:
                {
                    const FrameInfo_t *infoPtr = NULL;

                    status = WdfRequestRetrieveInputBuffer(RequestCtxPtr->m_WdfRequest, sizeof(FrameInfo_t), &((PVOID)infoPtr), NULL);
                    if (NT_SUCCESS(status)) {
                        RtlCopyMemory((PVOID)&(RequestCtxPtr->m_FrameInfo), infoPtr, sizeof(FrameInfo_t));
                        if (!NT_SUCCESS(status)) {
                            _DbgFrameKdPrint(("EvtFrameRequest::WdfRequestRetrieveInputBuffer(s=0x%x)\r\n", status));
                        }
                        else {
                            auto *fInfoPtr = &RequestCtxPtr->m_FrameInfo;
                            size_t imSize = min(m_BufferRequiredSize, m_FrameLenBytes);

                            status = WdfRequestRetrieveOutputBuffer(RequestCtxPtr->m_WdfRequest, imSize, &(PVOID)fInfoPtr->m_Fb, &fInfoPtr->m_FbSize);
                            _DbgFrameKdPrint(("Io Request m_FrameLenBytes 0x%x, m_FbSize 0x%x.\r\n", m_FrameLenBytes, fInfoPtr->m_FbSize));
                            ASSERT(m_FrameLenBytes <= fInfoPtr->m_FbSize);
                            if (!NT_SUCCESS(status)) {
                                _DbgFrameKdPrint(("EvtFrameRequest::WdfRequestRetrieveOutputBuffer(s=0x%x)\r\n", status));
                            }
                            else {
                                _DbgFrameKdPrint(("FrameInfo_t\r\n\t(Virtual=0x%x)\r\n\t(Fb=0x%p)\r\n\t(ByteCount=0x%x)\r\n\t(FbSize=0x%x)\r\n\t(StrideValid=0x%x)\r\n\t\r\n",
                                    fInfoPtr->m_Virtual,
                                    fInfoPtr->m_Fb,
                                    fInfoPtr->m_ByteCount,
                                    fInfoPtr->m_FbSize,
                                    fInfoPtr->m_StrideValid
                                    ));

                                fInfoPtr->m_RequestCtxPtr = RequestCtxPtr;

                                ASSERT(m_ActiveRequestCtxPtr == NULL); // Using serialized dispatch.
                                auto* finishedBuffPtr = (WdfIsi_ctx::DiscardBuffInfo_t*)InterlockedExchangePointer(&(void*)m_FinishedBuffPtr, NULL);

                                if (finishedBuffPtr != NULL) {
                                    _DbgFrameKdPrint(("FinishGetFrameRequest fast dispatch\r\n"));
                                    FinishGetFrameRequest(finishedBuffPtr, RequestCtxPtr); // After call finishedBuffPtr == Free, RequestCtxPtr == NULL.
                                }
                                else {
                                    m_State = S_FRAME_REQUESTED; // Just sidenote.
                                    m_ActiveRequestCtxPtr = RequestCtxPtr;
                                }
                                RequestCtxPtr = NULL;
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
        if (RequestCtxPtr) { // Actually m_ActiveRequestCtxPtr == NULL and !NT_SUCCESS(status)
            WdfRequestUnmarkCancelable(RequestCtxPtr->m_WdfRequest);
            _DbgFrameKdPrint(("UnmarkCancelable %p\r\n", RequestCtxPtr->m_WdfRequest));
            WdfRequestComplete(RequestCtxPtr->m_WdfRequest, status);
        }
        _DbgFrameKdPrint(("-freqi %x , 0x%p.\r\n", m_State, m_ActiveRequestCtxPtr));
    }
}

void WdfIsi_ctx::ReinitializeRequest(PREQUEST_CONTEXT RequestCtxPtr)
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
    ASSERT(m_MipiCsiSrc <= m_MaxMipiCsiSrc); // Depends on platform

    if (WdfRequestIsCanceled(RequestCtxPtr->m_WdfRequest)) {
        status = STATUS_CANCELLED;
    }
    else {
        camera_config_t *configPtr = NULL;

        status = WdfRequestRetrieveInputBuffer(RequestCtxPtr->m_WdfRequest, sizeof(camera_config_t), &(PVOID)configPtr, NULL);
        if (NT_SUCCESS(status)) {
            WdfInterruptAcquireLock(m_IsrCtx->m_WdfInterrupt);

            status = IsiInit(*configPtr);
            if (NT_SUCCESS(status)) {
                IsiStart(true); // Enables IRQ
                m_State = S_DISCARDING;
                WdfInterruptReleaseLock(m_IsrCtx->m_WdfInterrupt);
            }
            else {
                IsiStart(false);
                WdfInterruptReleaseLock(m_IsrCtx->m_WdfInterrupt);
            }
        }
    }
    WdfRequestComplete(RequestCtxPtr->m_WdfRequest, status);
}

void WdfIsi_ctx::EvtInputFormatRequest(PREQUEST_CONTEXT RequestCtxPtr)
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
        video_pixel_format_t *inputPixelFmtPtr = NULL;
        video_pixel_format_t *outputPixelFmtPtr = NULL;

        status = WdfRequestRetrieveInputBuffer(RequestCtxPtr->m_WdfRequest, sizeof(*outputPixelFmtPtr), &((PVOID)outputPixelFmtPtr), NULL);
        if (!NT_SUCCESS(status)) {
            _DbgKdPrint(("Failed EvtInputFormatRequest::WdfRequestRetrieveInputBuffer(s=0x%x)\r\n", status));
        }
        else {
            status = WdfRequestRetrieveOutputBuffer(RequestCtxPtr->m_WdfRequest, sizeof(*inputPixelFmtPtr), &((PVOID)inputPixelFmtPtr), NULL);
            if (!NT_SUCCESS(status)) {
                _DbgKdPrint(("Failed EvtInputFormatRequest::WdfRequestRetrieveOutputBuffer(s=0x%x)\r\n", status));
            }
            else {
                switch (*outputPixelFmtPtr) {
                case kVIDEO_PixelFormatUYVY:
                // case kVIDEO_PixelFormatNV21:
                    *inputPixelFmtPtr = kVIDEO_PixelFormatYUYV;
                    WdfRequestSetInformation(RequestCtxPtr->m_WdfRequest, sizeof(*inputPixelFmtPtr));
                    break;
                case kVIDEO_PixelFormatYUYV:
                case kVIDEO_PixelFormatNV12:
                    *inputPixelFmtPtr = kVIDEO_PixelFormatUYVY;
                    WdfRequestSetInformation(RequestCtxPtr->m_WdfRequest, sizeof(*inputPixelFmtPtr));
                    break;
                default:
                    status = STATUS_INVALID_PARAMETER;
                    break;
                }
            }
        }
    }
    WdfRequestComplete(RequestCtxPtr->m_WdfRequest, status);
}

NTSTATUS WdfIsi_ctx::AllocFb(DiscardBuffInfo_t &buffInfo)
/*!
 * Allocated buffer for common buffer DMA.
 *
 * @param buffInfo Storage for allocated buffer information.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;
    static const PHYSICAL_ADDRESS zero = { 0,0 };
    static const PHYSICAL_ADDRESS high = { 0xffffffff, 0 }; // this is required by MmAllocatePagesForMdlEx

    buffInfo.mdlPtr = MmAllocatePagesForMdlEx(zero, high, zero, m_BufferRequiredSize, MmCached, MM_ALLOCATE_FULLY_REQUIRED | MM_ALLOCATE_REQUIRE_CONTIGUOUS_CHUNKS);
    if (buffInfo.mdlPtr == NULL) {
        _DbgKdPrint(("WdfIsi Out of contignous memory.\r\n"));
        status = STATUS_INSUFFICIENT_RESOURCES;
    }

    UNREFERENCED_PARAMETER(buffInfo);
    return status;
}

NTSTATUS WdfIsi_ctx::AcquireBuffers()
/*!
 * Prepare all allocated buffers for common buffer mode DMA address.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    for (unsigned i = 0; ((i < COMMON_FRAME_BUFFER_NUM) && NT_SUCCESS(status)); ++i) {
        status = MapFb(m_DiscardBuff[i]);
    }
    return status;
}

NTSTATUS WdfIsi_ctx::MapFb(DiscardBuffInfo_t &buffInfo)
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
        buffInfo.physY = MmGetPhysicalAddress(buffInfo.virt);
        if (buffInfo.physY.QuadPart != NULL) {
            status = STATUS_SUCCESS;
        }
    }
    return status;
}

NTSTATUS WdfIsi_ctx::ReleaseBuffers()
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

NTSTATUS WdfIsi_ctx::UnmapFb(DiscardBuffInfo_t& buffInfo)
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

void WdfIsi_ctx::FreeFb(DiscardBuffInfo_t &buffInfo)
/*!
 * Free allocated buffer for common buffer mode DMA.
 *
 * @param buffInfo Storage with allocated buffer information.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    if (buffInfo.mdlPtr != NULL) {
        if (buffInfo.virt != NULL) {
            MmUnmapLockedPages(buffInfo.virt, buffInfo.mdlPtr);
            buffInfo.virt = NULL;
        }
        MmFreePagesFromMdl(buffInfo.mdlPtr);
        ExFreePool(buffInfo.mdlPtr);
        buffInfo.mdlPtr = NULL;
    }
}

NTSTATUS WdfIsi_ctx::ReSplitFrameBuffers(const camera_config_t &Config)
{
    NTSTATUS status = STATUS_SUCCESS;
    const UINT32 width = FSL_VIDEO_EXTRACT_WIDTH((UINT32)Config.resolution);
    const UINT32 height = FSL_VIDEO_EXTRACT_HEIGHT((UINT32)Config.resolution);
    const UINT32 planeSize = width * height;

    for (unsigned i = 0; ((i < COMMON_FRAME_BUFFER_NUM) && NT_SUCCESS(status)); ++i) {
        LONGLONG phys = m_DiscardBuff[i].physY.QuadPart;
        UINT32 frameSize = 0;

        frameSize += planeSize * m_PlaneBytesPerPixel;

        if (m_NumPlanes > 1) {
            // phys += frameSize;
            m_DiscardBuff[i].physU.QuadPart = phys + frameSize;
            frameSize += planeSize / 2;
        }
        else {
            m_DiscardBuff[i].physU.QuadPart = 0;
        }

        if (m_NumPlanes > 2) {
            // phys += planeSize / 2;
            frameSize += planeSize / 2;
            m_DiscardBuff[i].physV.QuadPart = phys + frameSize;
        }
        else {
            m_DiscardBuff[i].physV.QuadPart = 0;
        }
        m_FrameLenBytes = frameSize;
    }
    return status;
}

// IRQ Handling ----------------

NTSTATUS IsiIsrCtx_t::registerInterruptHandler(PDEVICE_CONTEXT CtxPtr)
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
    WDF_INTERRUPT_CONFIG interruptConfig;
    _DbgKdPrint(("IsiIsrCtx_t::registerInterruptHandler.\r\n"));

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, DEVICE_INTERRUPT_CONTEXT);
    WDF_INTERRUPT_CONFIG_INIT(&interruptConfig, EvtInterruptIsr, EvtInterruptDpc);
    interruptConfig.AutomaticSerialization = TRUE;
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

void WdfIsi_ctx::FinishGetFrameRequest(WdfIsi_ctx::DiscardBuffInfo_t *FinishedBuffPtr, PREQUEST_CONTEXT RequestCtxPtr)
/*!
 * Completes get frame IRP.
 * Function finishes IRP with ready image or cancels it.
 * Asserts when there's no image in m_FinishedBuffPtr or request is not IOCTL_CSI_DRIVER_GET_FRAME.
 * Function also dispatches next IO request as side effect of CompleteRequest() call.
 *
 * @param RequestCtxPtr pointer to request context.
 */
{
    size_t imSize = min(m_BufferRequiredSize, m_FrameLenBytes);

    if (RequestCtxPtr != NULL) {
        auto WdfRequest = RequestCtxPtr->m_WdfRequest;
        if (WdfRequest == NULL) {
            m_State = WdfIsi_ctx::S_DISCARDING;
        }
        else {
            ASSERT(RequestCtxPtr->m_CtlCode == IOCTL_ISI_DRIVER_GET_FRAME);
            NTSTATUS status = WdfRequestUnmarkCancelable(WdfRequest);
            auto* fInfoPtr = &RequestCtxPtr->m_FrameInfo;
                
            _DbgFrameKdPrint(("FinishGetFrameRequest m_FrameLenBytes 0x%x.\r\n", m_FrameLenBytes));
            ++m_CompleteFrameCnt;
            switch (status) {
            case STATUS_SUCCESS:
            {
                if ((fInfoPtr->m_Fb != NULL) && (FinishedBuffPtr != NULL)) {
                    if ((m_IsiRegistersPtr->OUT_BUF1_ADDR_Y == (UINT32)FinishedBuffPtr->physY.QuadPart)
                        || (m_IsiRegistersPtr->OUT_BUF2_ADDR_Y == (UINT32)FinishedBuffPtr->physY.QuadPart))
                    {
                        DbgBreakPoint();
                        _DbgFrameKdPrint(("Error: Frame with artifacts - finishedBuffPtr is in use 0x%p.\r\n", finishedBuffPtr->physY.QuadPart));
                        imSize = 0;
                        status = STATUS_IO_TIMEOUT;
                    }
                    else {
                        status = STATUS_SUCCESS;
                    }
                    RtlCopyMemory(fInfoPtr->m_Fb, FinishedBuffPtr->virt, imSize);
                    WdfRequestCompleteWithInformation(WdfRequest, status, imSize); // Dispatch new request, sets state to IOCTL_CSI_DRIVER_GET_FRAME
                }
                else {
                    status = STATUS_SUCCESS;
                    m_State = WdfIsi_ctx::S_DISCARDING; // Switch ISR to reuse Finished buffs
                    WdfRequestComplete(WdfRequest, status); // Dispatch new request, sets state to IOCTL_CSI_DRIVER_GET_FRAME
                }
            }
            break;

            default: // Could be STATUS_CANCELLED (owned by cancel routine), STATUS_INVALID_DEVICE_REQUEST or STATUS_INVALID_PARAMETER. Lets change state to discarding.
                break;
            }
        }
    }

    m_State = WdfIsi_ctx::S_DISCARDING;

    if (FinishedBuffPtr != NULL) {
#if (DBG)
        RtlZeroMemory(FinishedBuffPtr->virt, imSize); // Debug - see partial image.
#endif
        FinishedBuffPtr->state = FinishedBuffPtr->FREE;
    }
}

BOOLEAN IsiIsrCtx_t::EvtInterruptIsr(_In_ WDFINTERRUPT WdfInterrupt, _In_ ULONG MessageID)
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
    _DbgFrameKdPrint(("Isr+\n"));
    BOOLEAN scheduleDpc = false;
    BOOLEAN servicedIrq = false;
    WdfIsi_ctx *devCtxPtr = DeviceGetContext(WdfInterruptGetDevice(WdfInterrupt));

    ASSERT(devCtxPtr);
    auto registersPtr = devCtxPtr->m_IsiRegistersPtr;
    ASSERT(registersPtr);

    UINT32 isiSts = registersPtr->STS;
    BOOLEAN csiErrorIrqFired = (0 < (isiSts & devCtxPtr->m_CurrSrErrorMask)); // err = STS & CHNL_STS_ERR(ver)
    BOOLEAN csiFrameIrqFired = (0 < (isiSts & STS_FRM_STRD_BIT)); // Frame done

    registersPtr->STS = isiSts; // Clear STS register.
#if DBG
    if (isiSts & (STS_LATE_VSYNC_ERR_BIT | STS_EARLY_VSYNC_ERR_BIT | STS_OFLW_Y_BUF_BIT | STS_PANIC_Y_BUF_BIT | STS_OFLW_U_BUF_BIT | 
        STS_PANIC_U_BUF_BIT | STS_OFLW_V_BUF_BIT | STS_PANIC_V_BUF_BIT | STS_AXI_RD_ERR_BIT | STS_AXI_WR_ERR_Y_BIT | STS_AXI_WR_ERR_U_BIT | STS_AXI_WR_ERR_V_BIT)
        ) {
        _DbgKdPrint(("ISI STS_ERROR 0x%x.\r\n", isiSts));
    }
#endif

    if ((devCtxPtr->m_State != WdfIsi_ctx::S_DISCARDING) && (devCtxPtr->m_State != WdfIsi_ctx::S_FRAME_REQUESTED)) {
        // Peripheral is supposed to be stopped; this case probably never happens. 
        devCtxPtr->IsiStart(false);
    }
    else {
        UINT8 frameId = 0;

        if (csiErrorIrqFired) {
            servicedIrq = true;
            if ((isiSts & STS_EARLY_VSYNC_ERR_BIT)) {
                _DbgKdPrint(("STS_EARLY_VSYNC_ERR_BIT.\r\n")); // Log the errors only as peripheral doesn't need any error handling.
            }
            if ((isiSts & STS_LATE_VSYNC_ERR_BIT)) {
                _DbgKdPrint(("STS_LATE_VSYNC_ERR_BIT.\r\n")); // Log the errors only as peripheral doesn't need any error handling.
            }
            // If peripheral has been reset set csiFrameIrqFired = 0.
        }
        // Enabled and no errors
        if (csiFrameIrqFired) {
            servicedIrq = true;
            frameId = ((isiSts & STS_BUF1_ACTIVE_BIT) > 0) + (((isiSts & STS_BUF2_ACTIVE_BIT) > 0) << 1); // STS_BUF1_ACTIVE_BIT == 1, STS_BUF2_ACTIVE_BIT == 2
            if (frameId > 0) {
                frameId -= devCtxPtr->m_QuirkInvertFrameId;

                UINT64 donePhys = ((frameId == 1) ? devCtxPtr->m_IsiRegistersPtr->OUT_BUF1_ADDR_Y : devCtxPtr->m_IsiRegistersPtr->OUT_BUF2_ADDR_Y);
                WdfIsi_ctx::DiscardBuffInfo_t *finishedBuffPtr = NULL, *freeBuffPtr = NULL;

#if COMMON_FRAME_BUFFER_NUM < 4
                {
                    WdfIsi_ctx::DiscardBuffInfo_t * prevFinishedBuffPtr = (WdfIsi_ctx::DiscardBuffInfo_t*)InterlockedExchangePointerAcquire(&(void*)devCtxPtr->m_FinishedBuffPtr, NULL);

                    if (prevFinishedBuffPtr != NULL) {
                        prevFinishedBuffPtr->state = prevFinishedBuffPtr->FREE;
                    }
                }
#endif

                for (int i = 0; i < COMMON_FRAME_BUFFER_NUM; ++i) { // Search buffer lists for free and finished buffer.
                    auto &buff = devCtxPtr->m_DiscardBuff[i];

                    if ((buff.state == buff.FREE) && (freeBuffPtr == NULL)) {
                        freeBuffPtr = &buff; // Found free buffer.
                    }
                    if (((UINT64)buff.physY.QuadPart == donePhys)) {
                        ASSERT(buff.state == buff.WORKING); // Found finished buffer.
                        finishedBuffPtr = &buff;
                    }
                }
                ASSERT(finishedBuffPtr != NULL); // It's always one in our list.
                if (freeBuffPtr != NULL) { // If there's empty buffer, put that buffer to use.
                    freeBuffPtr->state = freeBuffPtr->WORKING;
                    if (frameId == 1) {
                        devCtxPtr->m_IsiRegistersPtr->OUT_BUF1_ADDR_Y = (UINT32)freeBuffPtr->physY.QuadPart;
                        devCtxPtr->m_IsiRegistersPtr->OUT_BUF1_ADDR_U = (UINT32)freeBuffPtr->physU.QuadPart;
                        devCtxPtr->m_IsiRegistersPtr->OUT_BUF_CTRL ^= OUT_BUF_CTRL_LOAD_BUF1_ADDR_BIT;
                    }
                    else {
                        devCtxPtr->m_IsiRegistersPtr->OUT_BUF2_ADDR_Y = (UINT32)freeBuffPtr->physY.QuadPart;
                        devCtxPtr->m_IsiRegistersPtr->OUT_BUF2_ADDR_U = (UINT32)freeBuffPtr->physU.QuadPart;
                        devCtxPtr->m_IsiRegistersPtr->OUT_BUF_CTRL ^= OUT_BUF_CTRL_LOAD_BUF2_ADDR_BIT;
                    }
                        
                    finishedBuffPtr->state = finishedBuffPtr->DONE;
                    {
                        auto* prevFinishedBuffer = (WdfIsi_ctx::DiscardBuffInfo_t*)InterlockedExchangePointer(&(void*)devCtxPtr->m_FinishedBuffPtr, finishedBuffPtr);

                        if (prevFinishedBuffer != NULL) {
                            prevFinishedBuffer->state = devCtxPtr->m_FinishedBuffPtr->FREE;
                            ++devCtxPtr->m_DiscardedFrameCnt;
                        }
                    }
                    scheduleDpc = true;
                }
            }
        }
        if (scheduleDpc) {
            WdfInterruptQueueDpcForIsr(WdfInterrupt);
        }
    }
    // Consider barrier after IRQ disable.
    return servicedIrq;
}

void IsiIsrCtx_t::EvtInterruptDpc(_In_ WDFINTERRUPT WdfInterrupt, _In_ WDFOBJECT AssociatedObject)
/*!
 * EvtInterruptDpc callback finishes get frame IRP.
 * Device AutomaticSerialization is at DPC level so DPC and Dispatch doesn't race except the WdfRequestComplete() call.
 *
 * @param WdfInterrupt handle to WDF interrupt object.
 * @param AssociatedObject handle to associated WDF object.
 */
{
    WdfIsi_ctx *devCtxPtr = DeviceGetContext(WdfInterruptGetDevice(WdfInterrupt));

    UNREFERENCED_PARAMETER(AssociatedObject);
    ASSERT(devCtxPtr);

    auto *requestCtxPtr = (PREQUEST_CONTEXT) InterlockedExchangePointer(&(void*)devCtxPtr->m_ActiveRequestCtxPtr, NULL);
    auto *finishedBuffPtr = (WdfIsi_ctx::DiscardBuffInfo_t*) InterlockedExchangePointer(&(void*)devCtxPtr->m_FinishedBuffPtr, NULL);

    devCtxPtr->FinishGetFrameRequest(finishedBuffPtr, requestCtxPtr);
}

IsiIsrCtx_t::IsiIsrCtx_t(PDEVICE_CONTEXT CtxPtr, WDFINTERRUPT WdfInterrupt)
    : m_CtxPtr(CtxPtr), m_WdfInterrupt(WdfInterrupt)
/*!
 * IsiIsrCtx_t constructor initializes context to defaults.
 *
 * @param ctxPtr pointer to device context.
 * @param wdfInterrupt handle to WDF interrupt object.
 */
{
}

NTSTATUS IsiIsrCtx_t::EvtWdfInterruptEnable(WDFINTERRUPT Interrupt, WDFDEVICE AssociatedDevice)
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
    _DbgKdPrint(("IsiIsrCtx_t::EvtWdfInterruptEnable+\n"));
    return STATUS_SUCCESS;
}

NTSTATUS IsiIsrCtx_t::EvtWdfInterruptDisable(WDFINTERRUPT Interrupt, WDFDEVICE AssociatedDevice)
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
    _DbgKdPrint(("IsiIsrCtx_t::EvtWdfInterruptDisable+\n"));
    auto ctxPtr = DeviceGetContext(AssociatedDevice);

    ASSERT(ctxPtr != NULL);
    ctxPtr->TerminateIo();
    return STATUS_SUCCESS;
}
