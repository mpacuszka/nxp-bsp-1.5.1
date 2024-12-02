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

#include "driver.h"
#include "device.tmh"
#include <acpiioct.h>
#include "acpiutil.hpp"

PAGED_SEGMENT_BEGIN
// The driver runs fully on passive level. IRP calls to I2C driver are synchronous.

NTSTATUS SNS0_ctx::GetRequestBuff(WDFREQUEST a_Request, REQUEST_BUFF_INFO &a_BuffInfo, bool a_GetWriteBuff) {
    NTSTATUS status;

    if (a_GetWriteBuff) {
        status = WdfRequestRetrieveOutputBuffer(a_Request, 1, &a_BuffInfo.m_pBuff, &a_BuffInfo.m_Len);
    }
    else {
        status = WdfRequestRetrieveInputBuffer(a_Request, 1, &a_BuffInfo.m_pBuff, &a_BuffInfo.m_Len);
    }
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "WdfRequestRetrieveInputBuffer failed %!STATUS!", status);
    }
    return status;
}

NTSTATUS SNS0_ctx::ConfigureReq(WDFREQUEST a_WdfRequest)
{
    NTSTATUS status = STATUS_SUCCESS;

    camera_config_t *configPtr = NULL;

    status = WdfRequestRetrieveInputBuffer(a_WdfRequest, sizeof(camera_config_t), &((PVOID)configPtr), NULL);

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "WdfRequestRetrieveInputBuffer failed %!STATUS!", status);
    }
    else {
        m_Camera.LinkProbe();
        if (!m_Camera.GetCameraInitStatus()) {
            if ((status = m_Camera.Max_Init(configPtr)) == STATUS_SUCCESS) {
                if ((status = m_Camera.Ov10635_Init(configPtr)) != STATUS_SUCCESS) {
                    TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "Initialization of the OV10635 failed. Status: %!STATUS!", status);
                    return status;
                }
            } else {
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "Initialization of the MAX9286-MAX9271 chain failed. Status: %!STATUS!", status);
                return status;
            }
        } else {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "Skipping camera chain initialization as it was already initilized");
        }
    }
    if (NT_SUCCESS(status)) {
        PUINT32 mipiFreq;

        status = WdfRequestRetrieveOutputBuffer(a_WdfRequest, sizeof(UINT32), &((PVOID)mipiFreq), NULL);
        if (NT_SUCCESS(status)) {
            *mipiFreq = 160000000;
            WdfRequestSetInformation(a_WdfRequest, sizeof(UINT32));
        }
    }
    return status;
}

NTSTATUS SNS0_ctx::StartReq(WDFREQUEST a_WdfRequest)
{
    NTSTATUS status = STATUS_SUCCESS;

    a_WdfRequest;

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "StartReq: failed: %!STATUS!", status);
    }
    else {
        /* Start MIPI CSI-2 output */
        m_Camera.Max_VideoEnable();
    }
    return status;
}

NTSTATUS SNS0_ctx::StopReq(WDFREQUEST a_Request)
{
    NTSTATUS status;

    m_QueueCurrentRequest = a_Request;
    status = m_Camera.Stop();
    return status;
}

NTSTATUS SNS0_ctx::Get_DsdAcpiResources()
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
            "I2cResIdMax9286",
            &I2cResIdMax9286,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdMax9271",
            &I2cResIdMax9271,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdMax9271_0",
            &I2cResIdMax9271_0,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdMax9271_1",
            &I2cResIdMax9271_1,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdMax9271_2",
            &I2cResIdMax9271_2,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdMax9271_3",
            &I2cResIdMax9271_3,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdMax9271_Broadcast",
            &I2cResIdMax9271_Broadcast,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdOv10635",
            &I2cResIdOv10635,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdOv10635_0",
            &I2cResIdOv10635_0,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdOv10635_1",
            &I2cResIdOv10635_1,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdOv10635_2",
            &I2cResIdOv10635_2,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        {
            "I2cResIdOv10635_3",
            &I2cResIdOv10635_3,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        };
        status = m_DsdRes.GetDsdResources(ParamTable, sizeof(ParamTable) / sizeof(ParamTable[0]));
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "SNS0_ctx::DsdRes.GetDsdResources failed: %!STATUS!", status);
        }
        if (NT_SUCCESS(status)) {
            NTSTATUS tmpStatus = m_DsdRes.GetInteger("GpioRstResId", &GpioRstResId);

            if (!NT_SUCCESS(tmpStatus)) {
                m_HasRstGpio = false;
                TraceEvents(TRACE_LEVEL_WARNING, TRACE_DRIVER, "Failed to query GpioRstResId: %!STATUS!", tmpStatus);
            }
            else {
                m_HasRstGpio = true;
            }
        }
    }
    else {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "SNS0_ctx::DsdRes.LoadDsd failed: %!STATUS!", status);
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
    UNREFERENCED_PARAMETER(DeviceObject);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "EvtDeviceObjCtxCleanup()");
}

SNS0_ctx::SNS0_ctx(WDFDEVICE &WdfDevice) : m_WdfDevice(WdfDevice), io::ctx_acpi_csr_stub(), CamWdf_Res(WdfDevice), m_Camera(*this), m_DsdRes(WdfDeviceWdmGetPhysicalDevice(WdfDevice)) /* CamWdf_Res is parent */ {};


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
                status = WdfDeviceCreateSymbolicLink(m_WdfDevice, &m_DeviceEndpointUnicodeName);
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "Create WdfDeviceCreateSymbolicLink (%ws): %!STATUS!", m_DeviceEndpointUnicodeName.Buffer, status);
            }
#if (DBG)
            if (NT_SUCCESS(status)) {
                status = WdfDeviceCreateDeviceInterface(m_WdfDevice, &GUID_DEVINTERFACE_SNS0_DRIVER, NULL);
                if (!NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Failed to create interface %!STATUS!", status);
                }
                if (NT_SUCCESS(status)) {
                status = WdfDeviceRetrieveDeviceInterfaceString(m_WdfDevice, &GUID_DEVINTERFACE_SNS0_DRIVER, NULL, m_DeviceInterfaceSymlinkName);
                if (!NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "WdfDeviceRetrieveDeviceInterfaceString failed %!STATUS!", status);
                }
                else { // Log the symlink name
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
            if (m_HasRstGpio) {
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_RstGpio.IoTargetInit.");
                status = m_RstGpio.IoTargetInit(m_GpioResList.at(GpioRstResId));
            }
            {
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cMax9286.IoTargetInit.");
                    status = m_I2cMax9286.IoTargetInit(m_I2cResList.at(I2cResIdMax9286));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cMax9271.IoTargetInit.");
                    status = m_I2cMax9271.IoTargetInit(m_I2cResList.at(I2cResIdMax9271));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cMax9271_0.IoTargetInit.");
                    status = m_I2cMax9271_0.IoTargetInit(m_I2cResList.at(I2cResIdMax9271_0));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cMax9271_1.IoTargetInit.");
                    status = m_I2cMax9271_1.IoTargetInit(m_I2cResList.at(I2cResIdMax9271_1));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cMax9271_2.IoTargetInit.");
                    status = m_I2cMax9271_2.IoTargetInit(m_I2cResList.at(I2cResIdMax9271_2));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cMax9271_3.IoTargetInit.");
                    status = m_I2cMax9271_3.IoTargetInit(m_I2cResList.at(I2cResIdMax9271_3));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cMax9271_Broadcast.IoTargetInit.");
                    status = m_I2cMax9271_Broadcast.IoTargetInit(m_I2cResList.at(I2cResIdMax9271_Broadcast));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cOv10635.IoTargetInit.");
                    status = m_I2cOv10635.IoTargetInit(m_I2cResList.at(I2cResIdOv10635));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cOv10635_0.IoTargetInit.");
                    status = m_I2cOv10635_0.IoTargetInit(m_I2cResList.at(I2cResIdOv10635_0));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cOv10635_1.IoTargetInit.");
                    status = m_I2cOv10635_1.IoTargetInit(m_I2cResList.at(I2cResIdOv10635_1));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cOv10635_2.IoTargetInit.");
                    status = m_I2cOv10635_2.IoTargetInit(m_I2cResList.at(I2cResIdOv10635_2));
                }
                if (NT_SUCCESS(status)) {
                    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cOv10635_3.IoTargetInit.");
                    status = m_I2cOv10635_3.IoTargetInit(m_I2cResList.at(I2cResIdOv10635_3));
                }

            }
            if (NT_SUCCESS(status)) {
                status = CreateDeviceInterface();
            }
        }
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

    if (m_HasRstGpio) {
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_RstGpio.IoTargetClose.");
        m_RstGpio.IoTargetClose();
    }
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "m_I2cMax9286.IoTargetClose.");
    m_I2cMax9286.IoTargetClose();

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnReleaseHardware()=%Xh", status);

    {
        auto *a = m_MeResList.Pop();
        while (a != NULL) {
            ExFreePoolWithTag(a, m_MeResList.DriverPoolTag);
            a = m_MeResList.Pop();
        }
    }
    {
        auto *a = m_GpioResList.Pop();
        while (a != NULL) {
            ExFreePoolWithTag(a, m_MeResList.DriverPoolTag);
            a = m_GpioResList.Pop();
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
 * @param WdfDevice handle to a WDF device object.
 * @param PreviousState previous power state.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(FxPreviousState);

    /* Initialize MAX9286 deserializer and MAX9271 serializer as first step */
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
