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
#include "device.tmh"
#include <acpiioct.h>
#include "acpiutil.hpp"

PAGED_SEGMENT_BEGIN
// The driver runs fully on passive level. IRP calls to I2C driver are synchronous.

NTSTATUS SNS0_ctx::GetRequestBuff(WDFREQUEST a_Request, REQUEST_BUFF_INFO &a_BuffInfo, bool a_GetWriteBuff)
/*!
 * Get request buffer.
 *
 * @param a_Request WDF request.
 * @param a_BuffInfo WDF.
 * @param a_GetWriteBuff.
 *
 * @returns STATUS_SUCCESS, otherwise error code.
 */
{
    NTSTATUS status;

    if (a_GetWriteBuff) {
        status = WdfRequestRetrieveOutputBuffer(a_Request, 1, &a_BuffInfo.m_pBuff, &a_BuffInfo.m_Len);
    }
    else {
        status = WdfRequestRetrieveInputBuffer(a_Request, 1, &a_BuffInfo.m_pBuff, &a_BuffInfo.m_Len);
    }
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "WdfRequestRetrieveBuffer failed %!STATUS!", status);
    }
    return status;
}

NTSTATUS SNS0_ctx::ConfigureReq(WDFREQUEST a_WdfRequest)
/*!
 * Configure video parameters (resolution, format,...).
 *
 * @param a_Request WDF request.
 *
 * @returns STATUS_SUCCESS, otherwise error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;
    camera_config_t *configPtr = NULL;

    status = WdfRequestRetrieveInputBuffer(a_WdfRequest, sizeof(camera_config_t), &((PVOID)configPtr), NULL);
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Failed to WdfRequestRetrieveInputBuffer %!STATUS!", status);
    }
    else {
        status = m_Camera.Configure(configPtr);
    }
    return status;
}

NTSTATUS SNS0_ctx::StartReq(WDFREQUEST a_WdfRequest)
/*!
 * Start the video stream.
 *
 * @param a_Request WDF request.
 *
 * @returns STATUS_SUCCESS, otherwise error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    a_WdfRequest;

    return status;
}

NTSTATUS SNS0_ctx::StopReq(WDFREQUEST a_Request)
/*!
 * Stop the video stream.
 *
 * @param a_Request WDF request.
 *
 * @returns STATUS_SUCCESS, otherwise error code.
 */
{
    NTSTATUS status;

    m_QueueCurrentRequest = a_Request;
    status = m_Camera.Stop();
    return status;
}

NTSTATUS SNS0_ctx::Get_DsdAcpiResources()
/*!
 * Get ACPI resources.
 *
 * @returns STATUS_SUCCESS, otherwise error code.
 */
{
    NTSTATUS status;

    status = m_DsdRes.LoadDsd();
    if (NT_SUCCESS(status)) {
        const AcpiDsdRes_t::_DSDVAL_GET_DESCRIPTOR ParamTable[] = {
            {
                "CameraClockFrequencyHz",
                &cameraClockFrequencyHz, // We want to reference final destination.
                (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
            },
            {
                "DeviceEndpoint0",
                (PUINT32)&(m_DeviceEndpoint[0]),
                sizeof(m_DeviceEndpoint),
                ACPI_METHOD_ARGUMENT_STRING,
            },
            {
                "I2c1ResIdAdp5585",
                &I2c1ResIdAdp5585,
                (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
            },
            {
                "I2c2ResIdAdp5585",
                &I2c2ResIdAdp5585,
                (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
            },
            {
                "I2c1ResIdAp1302",
                &I2c1ResIdAp1302,
                (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
            },
        };

        status = m_DsdRes.GetDsdResources(ParamTable, sizeof(ParamTable) / sizeof(ParamTable[0]));
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Failed to DsdRes.GetDsdResources %!STATUS!", status);
        }
    }
    else {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Failed to DsdRes.LoadDsd %!STATUS!", status);
    }
    return status;
}

NTSTATUS SNS0_ctx::EvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit)
/*!
 * Set pnp callbacks, Fileobject and WdfRequest context and creqte default queue.
 *
 * @param DriverObject handle to a WDF Driver object.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(Driver);

    NTSTATUS status;
    WDF_OBJECT_ATTRIBUTES deviceAttributes;
    PDEVICE_CONTEXT pDeviceContext;
    WDFDEVICE device;
    WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;

    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);
    pnpCallbacks.EvtDevicePrepareHardware = EvtPrepareHw; // OnPrepareHardware;
    pnpCallbacks.EvtDeviceReleaseHardware = EvtReleaseHw;
    pnpCallbacks.EvtDeviceD0Entry = EvtD0Entry;
    pnpCallbacks.EvtDeviceD0Exit = EvtD0Exit;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&deviceAttributes, DEVICE_CONTEXT);
    deviceAttributes.EvtCleanupCallback = SNS0_ctx::EvtDeviceObjCtxCleanup;
    status = WdfDeviceCreate(&DeviceInit, &deviceAttributes, &device);
    if (NT_SUCCESS(status)) {
        pDeviceContext = GetDeviceContext(device); // Since now the deviceContext is valid.
        ((SNS0_ctx *)pDeviceContext)->SNS0_ctx::SNS0_ctx(device);
        status = pDeviceContext->QueueInitialize(); // Initialize the I/O Package and any Queues
    }
    return status;
}

void SNS0_ctx::EvtDeviceObjCtxCleanup(_In_ WDFOBJECT DeviceObject)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "EvtDeviceObjCtxCleanup()");
    ((SNS0_ctx *)GetDeviceContext(DeviceObject))->SNS0_ctx::~SNS0_ctx();
}

SNS0_ctx::SNS0_ctx(WDFDEVICE &WdfDevice) : m_WdfDevice(WdfDevice), ctx_acpi_csr_stub(), CamWdf_Res(WdfDevice), m_Camera(*this), m_DsdRes(WdfDeviceWdmGetPhysicalDevice(WdfDevice)) /* CamWdf_Res is parent* / {};

SNS0_ctx::~SNS0_ctx()
/*!
 * SNS0_ctx constructor initializes context to defaults.
 */
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "~SNS0_ctx");
};

NTSTATUS SNS0_ctx::CreateDeviceInterface()
/*!
 * Create default queue and device interface.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    WDF_OBJECT_ATTRIBUTES attributes;

    WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
    attributes.ParentObject = m_WdfDevice;
    status = WdfStringCreate(NULL, &attributes, &m_DeviceInterfaceSymlinkName);
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "WdfStringCreate failed %!STATUS!", status);
    }
    else {
        if (NT_SUCCESS(status)) {
            ANSI_STRING deviceEndpointAnsiName;

            _Analysis_assume_nullterminated_(m_DeviceEndpoint);
            status = RtlInitAnsiStringEx(&deviceEndpointAnsiName, (PCSZ)(m_DeviceEndpoint));
            if (NT_SUCCESS(status)) {
                RtlInitEmptyUnicodeString(&m_DeviceEndpointUnicodeName, m_DeviceEndpointUnicodeNameBuff, sizeof(m_DeviceEndpointUnicodeNameBuff));
                status = RtlAnsiStringToUnicodeString(&m_DeviceEndpointUnicodeName, &deviceEndpointAnsiName, FALSE);
            }
            if (NT_SUCCESS(status)) {
                TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "Creating symbolic link %ws", m_DeviceEndpointUnicodeName.Buffer);
                status = WdfDeviceCreateSymbolicLink(m_WdfDevice, &m_DeviceEndpointUnicodeName);
                TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "WdfDeviceCreateSymbolicLink() Status: %!STATUS!", status);
            }
#if (DBG)
            if (NT_SUCCESS(status)) {
                status = WdfDeviceCreateDeviceInterface(m_WdfDevice, &GUID_DEVINTERFACE_SNS0_DRIVER, NULL);
                if (!NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Failed to create interface %!STATUS!", status);
                }
                if (NT_SUCCESS(status)) {
                    status = WdfDeviceRetrieveDeviceInterfaceString(m_WdfDevice, &GUID_DEVINTERFACE_SNS0_DRIVER, NULL, m_DeviceInterfaceSymlinkName);
                    if (NT_SUCCESS(status)) { // Log the symlink name
                        UNICODE_STRING StringName;

                        WdfStringGetUnicodeString(m_DeviceInterfaceSymlinkName, &StringName);
                        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "Created Link %ws.", StringName.Buffer);
                    }
                }
            }
#endif
        }
    }
    return status;
}

NTSTATUS SNS0_ctx::EvtPrepareHw(WDFDEVICE WdfDevice, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
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
    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(WdfDevice);

    return deviceCtxPtr->PrepareHw(ResourcesRaw, ResourcesTranslated);
}

NTSTATUS SNS0_ctx::EvtReleaseHw(WDFDEVICE WdfDevice, WDFCMRESLIST ResourcesTranslated)
/*!
 * WdfCsi_ctx::EvtReleaseHw bridges to non-static method WdfCsi_ctx::ReleaseHw.
 *
 * @param WdfDevice handle to a WDF device object.
 * @param ResourcesTranslated list of translated assigned resources.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(WdfDevice);

    return deviceCtxPtr->ReleaseHw(ResourcesTranslated);
}

NTSTATUS SNS0_ctx::EvtD0Entry(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE FxPreviousState)
/*!
 * WdfCsi_ctx::EvtD0Entry bridges to non-static method WdfCsi_ctx::D0Entry.
 *
 * @param WdfDevice handle to a WDF device object.
 * @param PreviousState previous power state.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(WdfDevice);

    return deviceCtxPtr->D0Entry(FxPreviousState);
}

NTSTATUS SNS0_ctx::EvtD0Exit(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE FxPreviousState)
/*!
 * WdfCsi_ctx::EvtD0Exit bridges to non-static method WdfCsi_ctx::D0Exit.
 *
 * @param WdfDevice handle to a WDF device object.
 * @param PreviousState previous power state.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(WdfDevice);

    return deviceCtxPtr->D0Exit(FxPreviousState);
}

NTSTATUS SNS0_ctx::PrepareHw(WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
/*!
 * Opens assigned resources.
 *
 * @param ResourcesRaw list of assigned resources.
 * @param ResourcesTranslated list of translated assigned resources.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(ResourcesRaw);
    NTSTATUS status;

    status = Get_CrsAcpiResources(ResourcesTranslated);
    if (NT_SUCCESS(status)) {
        status = Get_DsdAcpiResources();
        if (NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2c1ResIdAdp5585.IoTargetInit.");
            status = m_I2c1ResIdAdp5585.IoTargetInit(m_I2cResList.at(I2c1ResIdAdp5585));
        }
        if (NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2c1ResIdAp1302.IoTargetInit.");
            status = m_I2c1ResIdAp1302.IoTargetInit(m_I2cResList.at(I2c1ResIdAp1302));
        }
        if (NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2c2ResIdAdp5585.IoTargetInit.");
            status = m_I2c2ResIdAdp5585.IoTargetInit(m_I2cResList.at(I2c2ResIdAdp5585));
        }
    }
    if (NT_SUCCESS(status)) {
        status = CreateDeviceInterface();
    }
    return status;
}

NTSTATUS SNS0_ctx::ReleaseHw(WDFCMRESLIST FxResourcesTranslated)
/*!
 * Unmaps memory resources and closes IoTargets.
 *
 * @param ResourcesTranslated list of translated assigned resources.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(FxResourcesTranslated);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnReleaseHardware()");
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2c1ResIdAdp5585.IoTargetClose.");
    m_I2c1ResIdAdp5585.IoTargetClose();
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2c1ResIdAp1302.IoTargetClose.");
    m_I2c1ResIdAp1302.IoTargetClose();
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2c2ResIdAdp5585.IoTargetClose.");
    m_I2c2ResIdAdp5585.IoTargetClose();
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnReleaseHardware()=%Xh", status);

    {
        auto *a = m_MeResList.Pop();
        while (a != NULL) {
            ExFreePoolWithTag(a, m_MeResList.DriverPoolTag);
            a = m_MeResList.Pop();
        }
    }
    {
        auto *a = m_I2cResList.Pop();
        while (a != NULL) {
            ExFreePoolWithTag(a, m_MeResList.DriverPoolTag);
            a = m_I2cResList.Pop();
        }
    }
    {
        auto *a = m_IntResList.Pop();
        while (a != NULL) {
            ExFreePoolWithTag(a, m_MeResList.DriverPoolTag);
            a = m_IntResList.Pop();
        }
    }
    m_DsdRes.Cleanup();
    return status;
}

NTSTATUS SNS0_ctx::D0Entry(WDF_POWER_DEVICE_STATE FxPreviousState)
/*!
 * Handles EvtD0Entry.
 *
 * @param PreviousState previous power state.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(FxPreviousState);

    NTSTATUS status = m_Camera.Init();
    return status;
}

NTSTATUS SNS0_ctx::D0Exit(WDF_POWER_DEVICE_STATE FxPreviousState)
/*!
 * Stop device on EvtD0Exit.
 *
 * @param WdfDevice handle to a WDF device object.
 * @param PreviousState previous power state.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(FxPreviousState);

    return m_Camera.Deinit();
}

PAGED_SEGMENT_END
