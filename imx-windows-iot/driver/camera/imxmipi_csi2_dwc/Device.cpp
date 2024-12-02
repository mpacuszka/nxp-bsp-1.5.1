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

#include "imxmipi_csi2_dwc.h"
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
        NTSTATUS status;
        WDF_DRIVER_CONFIG config;
        WDF_OBJECT_ATTRIBUTES attributes;
        // Initialize WPP Tracing
        WPP_INIT_TRACING(DriverObject, RegistryPath);

        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "%!FUNC! Entry");

        //
        // Register a cleanup callback so that we can call WPP_CLEANUP when
        // the framework driver object is deleted during driver unload.
        //
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.EvtCleanupCallback = WdfMipi_ctx::EvtDriverContextCleanup;
        attributes.SynchronizationScope = WdfSynchronizationScopeDevice;
        attributes.ExecutionLevel = WdfExecutionLevelPassive; // Allow synchronous requests to ACPI.
        WDF_DRIVER_CONFIG_INIT(&config, DEVICE_CONTEXT::EvtDeviceAdd);
        config.DriverPoolTag = 'ICXM';

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

VOID WdfMipi_ctx::EvtDriverContextCleanup(_In_ WDFOBJECT DriverObject)
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

NTSTATUS WdfMipi_ctx::EvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit)
/*!
 * Set pnp callbacks, Fileobject and WdfRequest context and creqte default queue.
 *
 * @param Driver DriverObject handle to a WDF Driver object.
 * @param DeviceInit DeviceInit structure.
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

    {
        WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;

        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);
        pnpCallbacks.EvtDevicePrepareHardware = WdfMipi_ctx::EvtPrepareHw;
        pnpCallbacks.EvtDeviceReleaseHardware = WdfMipi_ctx::EvtReleaseHw;
        pnpCallbacks.EvtDeviceD0Entry = WdfMipi_ctx::EvtD0Entry;
        pnpCallbacks.EvtDeviceD0Exit = WdfMipi_ctx::EvtD0Exit;

        WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);
        WdfDeviceInitSetIoType(DeviceInit, WdfDeviceIoDirect);
    }

    {
        WDF_FILEOBJECT_CONFIG wdfFileObjectConfig;

        WDF_FILEOBJECT_CONFIG_INIT(
            &wdfFileObjectConfig,
            WdfMipi_ctx::EvtDeviceFileCreate,
            WdfMipi_ctx::EvtDeviceFileClose, // Stop CSI on file close or cleanup? FIXME
            WdfMipi_ctx::EvtDeviceFileCleanup); // Stop CSI on file cleanup

        WdfDeviceInitSetFileObjectConfig(DeviceInit, &wdfFileObjectConfig, WDF_NO_OBJECT_ATTRIBUTES);
        WdfDeviceInitSetExclusive(DeviceInit, TRUE);
    }

    {
        WDF_OBJECT_ATTRIBUTES attributes;

        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, REQUEST_CONTEXT);
        WdfDeviceInitSetRequestAttributes(DeviceInit, &attributes);
    }

    status = WdfDeviceCreate(&DeviceInit, &deviceAttributes, &wdfDevice);

    if (NT_SUCCESS(status)) {
        PDEVICE_CONTEXT deviceContextPtr = new (DeviceGetContext(wdfDevice)) DEVICE_CONTEXT(wdfDevice);

        if (NT_SUCCESS(status)) {
            status = deviceContextPtr->RegisterQueue();
        }
    }

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "EvtDeviceAdd failed %!STATUS!", status);
    }
    return status;
}

// PAGED_SEGMENT_END

WdfMipi_ctx::WdfMipi_ctx(WDFDEVICE &WdfDevice)
/*!
 * CsiIsrCtx_t constructor initializes context to defaults.
 *
 * @param WdfDevice handle to WDF device object.
 */
    : m_WdfDevice(WdfDevice),
      m_Mipi1Reg(WdfDevice),
      m_IsOpen(false),
      m_DsdRes(WdfDeviceWdmGetPhysicalDevice(WdfDevice)),
      m_MipiCsiRes(this)
{
    PAGED_CODE();
}

NTSTATUS WdfMipi_ctx::Get_DsdAcpiResources()
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
                "Mipi1RegResId",
                &Mipi1RegResId,
                (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
            },
            {
                "CpuId",
                &m_CpuId,
                (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
            },
            {
                "DeviceEndpoint0",
                (PUINT32) & (m_DeviceEndpoint[0]),
                sizeof(m_DeviceEndpoint),
                ACPI_METHOD_ARGUMENT_STRING,
            },
        };
        status = m_DsdRes.GetDsdResources(ParamTable, sizeof(ParamTable) / sizeof(ParamTable[0]));
        if (NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_CpuId = 0x%x", m_CpuId);
        }
    }
    else {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "DsdRes.LoadDsd failed. (status = %!status!)", status);
    }
    return status;
}

NTSTATUS WdfMipi_ctx::AcpiArgumentToUint32(PACPI_METHOD_ARGUMENT ArgumetPtr, UINT32 &Val)
{
    NTSTATUS status = STATUS_SUCCESS;

    if (ArgumetPtr->Type != ACPI_METHOD_ARGUMENT_INTEGER) {
        status = STATUS_ACPI_INVALID_ARGTYPE;
    }
    else {
        switch (ArgumetPtr->DataLength)
        {
            case sizeof(UINT8) :
                Val = *(reinterpret_cast<UINT8*>(ArgumetPtr->Data));
                break;
            case sizeof(UINT16) :
                Val = *(reinterpret_cast<UINT16*>(ArgumetPtr->Data));
                break;
            case sizeof(UINT32) :
                Val = *(reinterpret_cast<UINT32*>(ArgumetPtr->Data));
                break;
            default:
                NT_ASSERTMSG("Invalid _DSM query return size", FALSE);
                status = STATUS_ACPI_INVALID_DATA;
        }
    }
    return status;
}

NTSTATUS WdfMipi_ctx::AcpiReadInt(ULONG MethodNameUlong, UINT32 Offset, UINT32 *Val, UINT32 &Num)
/*!
 * Call Acpi metod that returns UINT32 argument.
 *
 * @param MethodNameUlong Four letters stored in UINT32. The string written backwards in code because of endianity.
 * @param Val             UINT32 reference to store returned argument of called Acpi method.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    PACPI_EVAL_OUTPUT_BUFFER buffPtr;
    ACPI_EVAL_INPUT_BUFFER_SIMPLE_INTEGER_V1 inputBuffer;
    UINT32 cnt = 0;

    RtlZeroMemory(&inputBuffer, sizeof(inputBuffer));
    inputBuffer.Signature = ACPI_EVAL_INPUT_BUFFER_SIMPLE_INTEGER_SIGNATURE_V1;
    inputBuffer.MethodNameAsUlong = MethodNameUlong; //'RPGR'; // Has to be spelled backwards because of endianity
    inputBuffer.IntegerArgument = Offset;

    status = m_DsdRes.EvalMethodSync((ACPI_EVAL_INPUT_BUFFER *)(&inputBuffer), sizeof(inputBuffer), &buffPtr);
    if (NT_SUCCESS(status)) {
        PACPI_METHOD_ARGUMENT methodArgumentsBegin = ACPI_EVAL_OUTPUT_BUFFER_ARGUMENTS_BEGIN(buffPtr);
        PACPI_METHOD_ARGUMENT methodArgumentsEnd = ACPI_EVAL_OUTPUT_BUFFER_ARGUMENTS_END(buffPtr);

        if (Num > 0) {
            FOR_EACH_ACPI_METHOD_ARGUMENT(argument, methodArgumentsBegin, methodArgumentsEnd) {
                status = AcpiArgumentToUint32(argument, *Val);
                if (!NT_SUCCESS(status)) {
                    break;
                }
                ++cnt;
                if (--Num == 0) {
                    break;
                }
                ++Val;
            }
        }
        Num = cnt;
        ExFreePool(buffPtr);
    }
    return status;
}

NTSTATUS Resources_t::AcpiRgpr(UINT32 Offset, UINT32 &Val)
/*!
 * Get Value from RGPR (Read GPR) Acpi metod.
 *
 * @param Val UINT32 reference to store returned argument of called Acpi method.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UINT32 acpiRetVal[2];
    UINT32 Len = 2;
    NTSTATUS status = m_DeviceCtxPtr->AcpiReadInt('RPGR', Offset, acpiRetVal, Len);

    if (NT_SUCCESS(status)) {
        status = acpiRetVal[0];
        if (Len == 2) {
            Val = acpiRetVal[1];
        }
    }
    return status;
}

NTSTATUS WdfMipi_ctx::AcpiWriteInt(ULONG MethodNameUlong, UINT32 Offset, UINT32 Val)
/*!
 * Call Acpi metod with UINT32 argument.
 *
 * @param MethodNameUlong Four letters stored in UINT32. The string written backwards in code because of endianity.
 * @param Val             UINT32 argument of called Acpi method.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    PACPI_EVAL_OUTPUT_BUFFER buffPtr;
    auto inputBufferSize = sizeof(ACPI_EVAL_INPUT_BUFFER_COMPLEX) + sizeof(ACPI_METHOD_ARGUMENT_V1);
    PACPI_EVAL_INPUT_BUFFER_COMPLEX inputBufferPtr = (PACPI_EVAL_INPUT_BUFFER_COMPLEX)ExAllocatePoolWithTag (NonPagedPoolNx, inputBufferSize, ACPI_TAG_IOCTL_INTERMEDIATE_BUFFER);

    RtlZeroMemory(inputBufferPtr, inputBufferSize);
    inputBufferPtr->Signature = ACPI_EVAL_INPUT_BUFFER_COMPLEX_SIGNATURE; // ACPI_METHOD_ARGUMENT_INTEGER
    inputBufferPtr->MethodNameAsUlong = MethodNameUlong; //'RPGR'; // Has to be spelled backwards because of endianity
    inputBufferPtr->ArgumentCount = 2;
    inputBufferPtr->Size = (ULONG)inputBufferSize;
    ACPI_METHOD_SET_ARGUMENT_INTEGER((&inputBufferPtr->Argument[0]), Offset);
    ACPI_METHOD_SET_ARGUMENT_INTEGER((&inputBufferPtr->Argument[1]), Val);


    status = m_DsdRes.EvalMethodSync((ACPI_EVAL_INPUT_BUFFER*)inputBufferPtr, inputBufferSize, &buffPtr);
    if (inputBufferPtr != NULL) {
        ExFreePool(inputBufferPtr);
    }
    if (buffPtr != NULL) {
        ExFreePool(buffPtr);
    }
    return status;
}

NTSTATUS Resources_t::AcpiWgpr(UINT32 Offset, UINT32 Val)
/*!
 * Pass Value to WGPR (Write GPR) Acpi metod.
 *
 * @param Val value to be set in GPR for MIPI.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    return m_DeviceCtxPtr->AcpiWriteInt('RPGW', Offset, Val);
}

NTSTATUS WdfMipi_ctx::EvtPrepareHw(_In_ WDFDEVICE WdfDevice, _In_ WDFCMRESLIST ResourcesRaw, _In_ WDFCMRESLIST ResourcesTranslated)
/*!
 * WdfMipi_ctx::EvtPrepareHw bridges to non-static method WdfMipi_ctx::PrepareHw.
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

NTSTATUS WdfMipi_ctx::PrepareHw(WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
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
            if (NT_SUCCESS(status) && (NULL != (memRes = m_MeResList.at(Mipi1RegResId)))) {
                status = m_Mipi1Reg.IoSpaceMap(*memRes);
            }
            if (NT_SUCCESS(status) && memRes == NULL) {
                status = STATUS_INSUFFICIENT_RESOURCES;
            }
            if (NT_SUCCESS(status)) {
                m_MipiCsiRes.m_CpuId = m_CpuId;
                m_MipiCsiRes.m_MipiRegistersPtr = m_Mipi1Reg.Reg();
                status = m_Mipi.PrepareHw(m_MipiCsiRes);
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
                status = WdfDeviceCreateSymbolicLink(m_WdfDevice, &m_DeviceEndpointUnicodeName);
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "Create WdfDeviceCreateSymbolicLink (%ws): %!STATUS!", m_DeviceEndpointUnicodeName.Buffer, status);
            }
#if (DBG)
            if (NT_SUCCESS(status)) {
                status = WdfDeviceCreateDeviceInterface(m_WdfDevice, &GUID_DEVINTERFACE_IMXMIPI, NULL);
            }
#endif
        }
    }
    return status;
}

NTSTATUS WdfMipi_ctx::EvtReleaseHw(_In_ WDFDEVICE WdfDevice, _In_ WDFCMRESLIST ResourcesTranslated)
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
        deviceContextPtr->m_Mipi1Reg,
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

NTSTATUS WdfMipi_ctx::EvtD0Entry(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE PreviousState)
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

NTSTATUS WdfMipi_ctx::EvtD0Exit(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE FxPreviousState)
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

    PDEVICE_CONTEXT deviceContextPtr = DeviceGetContext(WdfDevice);

    ASSERT(deviceContextPtr != NULL);
    deviceContextPtr->m_Mipi.Deinit();
    return STATUS_SUCCESS;
}

NTSTATUS WdfMipi_ctx::RegisterQueue()
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
    wdfQueueAttributes.ExecutionLevel = WdfExecutionLevelPassive; // Allow Synchronous send IRP. Also all parts of driver must execute at same level for Automatic synchronization to work.

    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&wdfQueueConfig, WdfIoQueueDispatchSequential);
    wdfQueueConfig.PowerManaged = WdfTrue;
    wdfQueueConfig.EvtIoDeviceControl = WdfMipi_ctx::EvtDeviceControl;
    wdfQueueConfig.EvtIoStop = WdfMipi_ctx::EvtIoStop;

    status = WdfIoQueueCreate(m_WdfDevice, &wdfQueueConfig, &wdfQueueAttributes, &queue);

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_QUEUE, "WdfIoQueueCreate failed %!status!", status);
        return status;
    }

    return status;
}

void WdfMipi_ctx::EvtDeviceControl(_In_ WDFQUEUE Queue, _In_ WDFREQUEST WdfRequest, _In_ size_t OutputBufferLength, _In_ size_t InputBufferLength, _In_ ULONG IoControlCode)
/*!
 * EvtDeviceControl handles known control requests.
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

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_QUEUE,
        "%!FUNC! Queue 0x%p, Request 0x%p OutputBufferLength %d InputBufferLength %d IoControlCode %d",
        Queue, WdfRequest, (int)OutputBufferLength, (int)InputBufferLength, IoControlCode);

    requestCtxPtr->m_WdfRequest = WdfRequest;
    if (WdfRequestIsCanceled(WdfRequest)) {
        WdfRequestComplete(WdfRequest, STATUS_CANCELLED);
    }
    else {
        switch (IoControlCode)
        {
        case IOCTL_MIPI_CONFIGURE:
            {
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "IOCTL_MIPI_CONFIGURE");
                ctxPtr->ConfigureRequest(requestCtxPtr); // ConfigureRequest calls WdfRequestComplete on it's own. Don't call WdfRequestComplete().
            }
            return;
        case IOCTL_MIPI_STOP:
            {
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "IOCTL_MIPI_STOP");
                ctxPtr->m_Mipi.Deinit();
                status = STATUS_SUCCESS;
                WdfRequestComplete(WdfRequest, status);
            }
            break;
        case IOCTL_MIPI_REQUIRED_FMT:
        {
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "IOCTL_MIPI_REQUIRED_FMT");
            ctxPtr->EvtInputFormatRequest(requestCtxPtr); // EvtFrameRequest completes request on it's own. Don't call WdfRequestComplete().
            break;
        }
        default:
            {
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "STATUS_INVALID_DEVICE_REQUEST");
                status = STATUS_INVALID_DEVICE_REQUEST;
                WdfRequestComplete(WdfRequest, status);
            }
            break;
        }
    }
}

void WdfMipi_ctx::EvtDeviceFileCreate(WDFDEVICE WdfDevice, WDFREQUEST WdfRequest, WDFFILEOBJECT WdfFileObject)
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
    UNREFERENCED_PARAMETER(WdfFileObject);
    PAGED_CODE();

    NTSTATUS status = STATUS_SUCCESS;

    WdfRequestComplete(WdfRequest, status);
}

void WdfMipi_ctx::EvtDeviceFileCleanup(WDFFILEOBJECT WdfFileObject)
/*!
 * Frees channel assigned to device file on file clean up event.
 *
 * @param WdfFileObject handle to a WDF request object.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    PAGED_CODE();

    PDEVICE_CONTEXT devCtxPtr = DeviceGetContext(WdfFileObjectGetDevice(WdfFileObject)); // fileObjectCtxPtr->m_DevCtxPtr;
    NT_ASSERT(devCtxPtr != NULL);

    devCtxPtr->Close();
}

NTSTATUS WdfMipi_ctx::Close()
/*!
 * Nonstatic handler for EvtDeviceFileCleanup.
 *
 * @returns STATUS_SUCCESS.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    status = m_Mipi.Deinit();
    m_IsOpen = false;

    return status;
}

void WdfMipi_ctx::EvtDeviceFileClose(WDFFILEOBJECT WdfFileObject)
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

void WdfMipi_ctx::EvtIoStop(_In_ WDFQUEUE Queue, _In_ WDFREQUEST WdfRequest, _In_ ULONG ActionFlags)
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
    UNREFERENCED_PARAMETER(Queue);
    UNREFERENCED_PARAMETER(ActionFlags);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_QUEUE, "%!FUNC! Queue 0x%p, Request 0x%p ActionFlags %d", Queue, WdfRequest, ActionFlags);

    EvtWdfRequestCancel(WdfRequest);

    return;
}

void WdfMipi_ctx::EvtWdfRequestCancel(WDFREQUEST WdfRequest)
/*!
 * Completes request as canceled.
 *
 * @param WdfRequest WDF request object to be canceled. Request must be the same as m_ActiveRequestCtxPtr.
 */
{
    // Bevare this method easily gets called after file cleanup which might free some resources. I have witnessed this in when userspace app failed, closed file and request got canceled later on just after FileCleanup and before FileClose.
    auto devCtxPtr = DeviceGetContext(WdfIoQueueGetDevice(WdfRequestGetIoQueue(WdfRequest)));

    devCtxPtr->m_Mipi.Stop();
    devCtxPtr->m_ActiveRequestCtxPtr = NULL;
    WdfRequestComplete(WdfRequest, STATUS_CANCELLED);
}

void WdfMipi_ctx::EvtInputFormatRequest(PREQUEST_CONTEXT RequestCtxPtr)
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
        video_pixel_format_t* inputPixelFmtPtr = NULL;
        video_pixel_format_t* outputPixelFmtPtr = NULL;

        status = WdfRequestRetrieveInputBuffer(RequestCtxPtr->m_WdfRequest, sizeof(*outputPixelFmtPtr), &((PVOID)outputPixelFmtPtr), NULL);
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "WdfRequestRetrieveInputBuffer failed %!STATUS!", status);
        }
        else {
            status = WdfRequestRetrieveOutputBuffer(RequestCtxPtr->m_WdfRequest, sizeof(*inputPixelFmtPtr), &((PVOID)inputPixelFmtPtr), NULL);
            if (!NT_SUCCESS(status)) {
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "WdfRequestRetrieveOutputBuffer failed %!STATUS!", status);
            }
            else {
                switch (*outputPixelFmtPtr)
                {
                case kVIDEO_PixelFormatYUYV:
                    *inputPixelFmtPtr = kVIDEO_PixelFormatYUYV;
                    break;
                case kVIDEO_PixelFormatUYVY:
                    *inputPixelFmtPtr = kVIDEO_PixelFormatUYVY;
                    break;
                default:
                    status = STATUS_INVALID_PARAMETER;
                    break;
                }
                if (NT_SUCCESS(status)) {
                    WdfRequestSetInformation(RequestCtxPtr->m_WdfRequest, sizeof(*inputPixelFmtPtr));
                }
            }
        }
    }
    WdfRequestComplete(RequestCtxPtr->m_WdfRequest, status);
}

void WdfMipi_ctx::ConfigureRequest(PREQUEST_CONTEXT RequestCtxPtr)
/*!
 * Reinitialize the CSI and MIPI peripherals to given configuration.
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

        status = WdfRequestRetrieveInputBuffer(RequestCtxPtr->m_WdfRequest, sizeof(camera_config_t), &((PVOID)configPtr), NULL);
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "WdfRequestRetrieveInputBuffer failed %!STATUS!", status);
        }
        else {
            status = m_Mipi.Init(*configPtr);
            if (NT_SUCCESS(status)) {
                status = m_Mipi.Start(*configPtr);
            }
            else {
                m_Mipi.Stop();
            }
        }  
    }
    WdfRequestComplete(RequestCtxPtr->m_WdfRequest, status);
}
