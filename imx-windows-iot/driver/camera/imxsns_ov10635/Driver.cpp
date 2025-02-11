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
#include "driver.tmh"

#ifdef ALLOC_PRAGMA
#pragma code_seg("INIT")
#endif // ALLOC_PRAGMA

NTSTATUS
DriverEntry(
    _In_ PDRIVER_OBJECT  DriverObject,
    _In_ PUNICODE_STRING RegistryPath
    )
/*!
 * DriverEntry initializes the driver.
 *
 * @param DriverObject handle to a WDF Driver object.
 * @param RegistryPath driver specific path in the Registry.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS Status;
    WDF_OBJECT_ATTRIBUTES attributes;

    //
    // Initialize WPP Tracing
    WPP_INIT_TRACING(DriverObject, RegistryPath);
    DbgPrint("DriverEntry  %wZ\r\n", RegistryPath);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "%!FUNC! Entry");

    //
    // Register a cleanup callback so that we can call WPP_CLEANUP when
    // the framework driver object is deleted during driver unload.
    //
    WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
    attributes.EvtCleanupCallback = SNS0_ctx::EvtDriverObjCtxCleanup;

    {
        WDF_DRIVER_CONFIG wdfDriverConfig;
        WDF_DRIVER_CONFIG_INIT(&wdfDriverConfig, SNS0_ctx::EvtDeviceAdd);
        wdfDriverConfig.DriverPoolTag = IMX_OV10635_POOL_TAG;

        Status = WdfDriverCreate(DriverObject, RegistryPath, &attributes, &wdfDriverConfig, WDF_NO_HANDLE);
        if (!NT_SUCCESS(Status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "WdfDriverCreate failed %!STATUS!", Status);
            WPP_CLEANUP(DriverObject);
            return Status;
        }
    }
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "%!FUNC! Exit");

    return Status;
}

#ifdef ALLOC_PRAGMA
#pragma code_seg("PAGE")
#endif // ALLOC_PRAGMA

VOID SNS0_ctx::EvtDriverObjCtxCleanup(_In_ WDFOBJECT WdfDriver) /* Free all the resources allocated in DriverEntry. --*/
{
    PAGED_CODE();

    WPP_CLEANUP(WdfDriver); //  driverObjectPtr);
}
