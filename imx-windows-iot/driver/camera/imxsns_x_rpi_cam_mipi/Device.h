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

#pragma once

#include "dsdtutil.hpp"
#include "OperatorNew.hpp"
#include "WdfIoTargets.hpp"
#include "x_rpi_cam_mipi/imx_x_rpi_cam_mipi.h"

#define IMX_X_RPI_CAM_MIPI_0_POOL_TAG 'AP13'

struct REQUEST_BUFF_INFO
{
    void *m_pBuff;
    size_t m_Len;
};
typedef REQUEST_BUFF_INFO *PREQUEST_BUFF_INFO;

struct SNS0_ctx: io::ctx_acpi_csr_stub, CamWdf_Res
{
    const WDFDEVICE m_WdfDevice;
    WDFSTRING m_DeviceInterfaceSymlinkName;
    WDFQUEUE m_Queue;
    WDFREQUEST m_QueueCurrentRequest;

    XRpiCamMipi_t m_Camera;

    /* ACPI */
    AcpiDsdRes_t m_DsdRes;
    NTSTATUS Get_DsdAcpiResources();

    /* PNP static callbacks */
    static NTSTATUS EvtPrepareHw(_In_ WDFDEVICE Device, _In_ WDFCMRESLIST ResourcesRaw, _In_ WDFCMRESLIST ResourcesTranslated);
    static NTSTATUS EvtReleaseHw(_In_ WDFDEVICE WdfDevice, _In_ WDFCMRESLIST FxResourcesTranslated);
    static NTSTATUS EvtD0Entry(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE FxPreviousState);
    static NTSTATUS EvtD0Exit(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE FxPreviousState);

    /* PNP worker methods */
    NTSTATUS PrepareHw(_In_ WDFCMRESLIST ResourcesRaw, _In_ WDFCMRESLIST ResourcesTranslated);
    NTSTATUS ReleaseHw(WDFCMRESLIST FxResourcesTranslated);
    NTSTATUS D0Entry(WDF_POWER_DEVICE_STATE FxPreviousState);
    NTSTATUS D0Exit(WDF_POWER_DEVICE_STATE FxPreviousState);

    /* IRP utils */
    NTSTATUS GetRequestBuff(WDFREQUEST a_Request, REQUEST_BUFF_INFO &a_BuffInfo, bool a_WriteBuff = false);
    /* IRP */
    NTSTATUS CreateDeviceInterface();
    NTSTATUS QueueInitialize();
    static VOID EvtDeviceControl(_In_ WDFQUEUE Queue, _In_ WDFREQUEST Request, _In_ size_t OutputBufferLength, _In_ size_t InputBufferLength, _In_ ULONG IoControlCode);
    static VOID EvtIoStop(_In_ WDFQUEUE Queue, _In_ WDFREQUEST Request, _In_ ULONG ActionFlags);
    /* IRP MJ */
    NTSTATUS ConfigureReq(WDFREQUEST a_Request);
    NTSTATUS StartReq(WDFREQUEST a_Request);
    NTSTATUS StopReq(WDFREQUEST a_Request);

public:
    SNS0_ctx(WDFDEVICE &device);
    static NTSTATUS EvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit);
    static EVT_WDF_DEVICE_CONTEXT_CLEANUP EvtDeviceObjCtxCleanup;
    static EVT_WDF_OBJECT_CONTEXT_CLEANUP EvtDriverObjCtxCleanup;
};

typedef SNS0_ctx DEVICE_CONTEXT, *PDEVICE_CONTEXT;
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, GetDeviceContext);
