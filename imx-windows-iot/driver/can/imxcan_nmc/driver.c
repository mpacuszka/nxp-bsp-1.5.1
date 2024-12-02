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

DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD imxcanEvtDeviceAdd;
EVT_WDF_DRIVER_UNLOAD ImxCanEvtWdfDriverUnload;
EVT_WDF_DEVICE_PREPARE_HARDWARE ImxCanEvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE ImxCanEvtDeviceReleaseHardware;
EVT_WDF_DEVICE_D0_ENTRY ImxCanEvtDeviceD0Entry;

IMXCAN_PAGED_SEGMENT_BEGIN;

#pragma warning(push)
#pragma warning(disable:4189) /* local variable is initialized but not referenced */
VOID ImxCanEvtDriverUnload(WDFDRIVER WdfDriver)
{
    PAGED_CODE();

    DRIVER_OBJECT* driverObjectPtr = WdfDriverWdmGetDriverObject(WdfDriver);
    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "ImxCanEvtDriverUnload\n"));
}
#pragma warning(pop)

IMXCAN_PAGED_SEGMENT_END;
IMXCAN_INIT_SEGMENT_BEGIN

NTSTATUS DriverEntry(
    _In_ PDRIVER_OBJECT     DriverObject,
    _In_ PUNICODE_STRING    RegistryPath
)
{
    PAGED_CODE();

    /* NTSTATUS variable to record success or failure */
    NTSTATUS status = STATUS_SUCCESS;

    /* Allocate the driver configuration object */
    WDF_DRIVER_CONFIG wdfDriverConfig;

    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "imxcan: DriverEntry\n"));

    /* Initialize the driver configuration object to register the entry point for the EvtDeviceAdd callback, imxcanEvtDeviceAdd */
    WDF_DRIVER_CONFIG_INIT(&wdfDriverConfig, imxcanEvtDeviceAdd);
    wdfDriverConfig.DriverPoolTag = IMXCAN_POOL_TAG;
    wdfDriverConfig.EvtDriverUnload = ImxCanEvtDriverUnload;

    /* Create the driver object */
    WDFDRIVER wdfDriver;
    status = WdfDriverCreate(
        DriverObject,
        RegistryPath,
        WDF_NO_OBJECT_ATTRIBUTES,
        &wdfDriverConfig,
        &wdfDriver
    );
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Failed to create WDF driver object.\n"));
    }
    return status;
}

IMXCAN_INIT_SEGMENT_END;
IMXCAN_PAGED_SEGMENT_BEGIN;

NTSTATUS imxcanEvtDeviceAdd(
    _In_    WDFDRIVER       Driver,
    _Inout_ PWDFDEVICE_INIT DeviceInitPtr
)
{
    UNREFERENCED_PARAMETER(Driver);
    PAGED_CODE();

    NTSTATUS                status;
    WDFDEVICE               wdfDevice;
    PDEV_CONTEXT            deviceContextPtr;

    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FlexCAN: imxcanEvtDeviceAdd\n"));

    /* Set PNP and Power callbacks */
    {
        WDF_PNPPOWER_EVENT_CALLBACKS wdfPnpPowerEventCallbacks;
        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&wdfPnpPowerEventCallbacks);
        wdfPnpPowerEventCallbacks.EvtDevicePrepareHardware = ImxCanEvtDevicePrepareHardware;
        wdfPnpPowerEventCallbacks.EvtDeviceReleaseHardware = ImxCanEvtDeviceReleaseHardware;
        wdfPnpPowerEventCallbacks.EvtDeviceD0Entry = ImxCanEvtDeviceD0Entry;
        WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInitPtr, &wdfPnpPowerEventCallbacks);

    }

    /* Configure file create/close callbacks */
    {
        WDF_FILEOBJECT_CONFIG wdfFileObjectConfig;
        WDF_FILEOBJECT_CONFIG_INIT(&wdfFileObjectConfig, ImxCanEvtDeviceFileCreate, ImxCanEvtFileClose, WDF_NO_EVENT_CALLBACK);
        WdfDeviceInitSetFileObjectConfig(DeviceInitPtr, &wdfFileObjectConfig, WDF_NO_OBJECT_ATTRIBUTES);
    }

    /* Create and initialize the WDF device */
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, DEV_CONTEXT);

        /* Create the device object */
        status = WdfDeviceCreate(&DeviceInitPtr, &attributes, &wdfDevice);
        if (!NT_SUCCESS(status)) {
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfDeviceCreate(...) failed.\n"));
            return status;
        }
        deviceContextPtr = ImxCanGetDeviceContext(wdfDevice);
        deviceContextPtr->WdfDevice = wdfDevice;
    }

    /* Create an interrupt object with an associated context. */
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, INT_CONTEXT);
        WDF_INTERRUPT_CONFIG interruptConfig;
        WDF_INTERRUPT_CONFIG_INIT(&interruptConfig, ImxCanEvtInterruptIsr, ImxCanEvtInterruptDpc);

        WDFINTERRUPT wdfInterrupt;
        status = WdfInterruptCreate(wdfDevice, &interruptConfig, &attributes, &wdfInterrupt);
        if (!NT_SUCCESS(status)) {
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Failed to create interrupt object.\n"));
            return status;
        }
        deviceContextPtr->WdfInterrupt = wdfInterrupt;
    }

    /* Create controller and pin locks */
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);

        attributes.ParentObject = wdfDevice;
        status = WdfWaitLockCreate(&attributes, &deviceContextPtr->ControllerLock);
        if (!NT_SUCCESS(status)) {
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfWaitLockCreate(...ControllerLock) failed. (status = %!STATUS!)\n", status));
            return status;
        }
        status = WdfWaitLockCreate(&attributes, &deviceContextPtr->Pin.Lock);
        if (!NT_SUCCESS(status)) {
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfWaitLockCreate(...Pin.Lock) failed. (status = %!STATUS!)\n", status));
            return status;
        }
    }

    /* Set controller info and defaults */
    deviceContextPtr->ControllerInfo.PinCount = 1;
    deviceContextPtr->ControllerInfo.Size = sizeof(CAN_CONTROLLER_INFO);
    deviceContextPtr->ControllerInfo.Sequence = 0;
    deviceContextPtr->ControllerInfo.RequestCnt = 0;

    /* Set circular buffer */
    {
        size_t buffersize, capacity, sz;
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = deviceContextPtr->WdfInterrupt;
        WDFMEMORY memhandle;
        PINT_CONTEXT interruptContextPtr = ImxCanGetInterruptContext(deviceContextPtr->WdfInterrupt);

        interruptContextPtr->InterruptStatus = 0;
        capacity = CB_CAPACITY;
        sz = sizeof(data_callback);
        buffersize = capacity * sz;
        status = WdfMemoryCreate(
            &attributes,
            NonPagedPoolNx,
            IMXCAN_POOL_TAG,
            buffersize,
            &memhandle,
            &interruptContextPtr->FrameQueue.buffer); /* buffer pointer */
        if (!NT_SUCCESS(status)) {
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfMemoryCreate failed 0x%x\n", status));
            return status;
        }
        RtlZeroMemory(interruptContextPtr->FrameQueue.buffer, buffersize);
        interruptContextPtr->memhandle = memhandle;
        interruptContextPtr->FrameQueue.buffer_end = (char *)interruptContextPtr->FrameQueue.buffer + buffersize;
        interruptContextPtr->FrameQueue.capacity = capacity;
        interruptContextPtr->FrameQueue.count = 0;
        interruptContextPtr->FrameQueue.sz = sz;
        interruptContextPtr->FrameQueue.head = interruptContextPtr->FrameQueue.buffer;
        interruptContextPtr->FrameQueue.tail = interruptContextPtr->FrameQueue.buffer;
    }

    /* Set PNP capabilities */
    {
        WDF_DEVICE_PNP_CAPABILITIES pnpCaps;
        WDF_DEVICE_PNP_CAPABILITIES_INIT(&pnpCaps);

        pnpCaps.Removable = WdfFalse;
        pnpCaps.SurpriseRemovalOK = WdfFalse;
        WdfDeviceSetPnpCapabilities(wdfDevice, &pnpCaps);
    }

     /* Make the device disable-able */
     {
         WDF_DEVICE_STATE wdfDeviceState;
         WDF_DEVICE_STATE_INIT(&wdfDeviceState);

         wdfDeviceState.NotDisableable = WdfFalse;
         WdfDeviceSetDeviceState(wdfDevice, &wdfDeviceState);
     }

     /* Create default sequential dispatch queue */
     {
         WDF_IO_QUEUE_CONFIG wdfQueueConfig;
         WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(
             &wdfQueueConfig,
             WdfIoQueueDispatchSequential);
         wdfQueueConfig.EvtIoDeviceControl = ImxCanEvtIoDeviceControl;

         WDF_OBJECT_ATTRIBUTES wdfQueueAttributes;
         WDF_OBJECT_ATTRIBUTES_INIT(&wdfQueueAttributes);
         wdfQueueAttributes.ExecutionLevel = WdfExecutionLevelPassive;

         WDFQUEUE wdfQueue;
         status = WdfIoQueueCreate(
             wdfDevice,
             &wdfQueueConfig,
             &wdfQueueAttributes,
             &wdfQueue);

         if (!NT_SUCCESS(status)) {
             KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfIoQueueCreate(..) failed. (status=%!STATUS!)\n", status));
             return status;
         }
     }
     /* Queue with manual dispatching to hold requests */
     {
         WDF_IO_QUEUE_CONFIG holdQueueConfig;
         WDF_IO_QUEUE_CONFIG_INIT(
             &holdQueueConfig,
             WdfIoQueueDispatchManual);
         holdQueueConfig.PowerManaged = WdfFalse;

         status = WdfIoQueueCreate(
             wdfDevice,
             &holdQueueConfig,
             WDF_NO_OBJECT_ATTRIBUTES,
             &deviceContextPtr->NotificationQueue);
     }

    /* Publish controller device interface */
     status = ImxCanCreateDeviceInterface(deviceContextPtr);
     if (!NT_SUCCESS(status)) {
         return status;
     }

    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FlexCAN: imxcanEvtDeviceAdd - return Success\n"));
    return status;
}

NTSTATUS ImxCanEvtDevicePrepareHardware(
    _In_    WDFDEVICE       WdfDevice,
    _In_    WDFCMRESLIST    ResourcesRaw,
    _In_    WDFCMRESLIST    ResourcesTranslated
)
{
    UNREFERENCED_PARAMETER(ResourcesRaw);
    PAGED_CODE();

    const CM_PARTIAL_RESOURCE_DESCRIPTOR* memResourcePtr = NULL;
    ULONG interruptResourceCount = 0;

    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FlexCAN: imxcanEvtDevicePrepareHardware\n"));

    /* Look for single memory and interrupt resource. */
    const ULONG resourceCount = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < resourceCount; ++i) {
        const CM_PARTIAL_RESOURCE_DESCRIPTOR* resourcePtr = WdfCmResourceListGetDescriptor(ResourcesTranslated, i);

        switch (resourcePtr->Type) {
            case CmResourceTypeMemory:
                if (memResourcePtr != NULL) {
                    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Received unexpected memory resource. (resourcePtr = 0x%p)\n", resourcePtr));
                    return STATUS_DEVICE_CONFIGURATION_ERROR;
                }
                memResourcePtr = resourcePtr;
                break;
            case CmResourceTypeInterrupt:
                if (interruptResourceCount > 0) {
                    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Received unexpected interrupt resource. (interruptResourceCount = %lu, resourcePtr = 0x%p)\n", interruptResourceCount, resourcePtr));
                    return STATUS_DEVICE_CONFIGURATION_ERROR;
                }
                ++interruptResourceCount;
                break;
            default:
                KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Unknown resource\n"));
                break;
        }
    }

    if ((memResourcePtr == NULL) || (interruptResourceCount < 1)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Did not receive required memory resource and interrupt resource. (memResourcePtr = 0x%p, interruptResourceCount = %lu)\n", memResourcePtr, interruptResourceCount));
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }
    if (memResourcePtr->u.Memory.Length < sizeof(IMXCAN_REGISTERS)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Memory resource is too small. memResourcePtr->u.Memory.Length = %lu, sizeof(IMXCAN_REGISTERS) = %lu\n", memResourcePtr->u.Memory.Length, sizeof(IMXCAN_REGISTERS)));
        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }

    PDEV_CONTEXT deviceContextPtr = ImxCanGetDeviceContext(WdfDevice);

    NT_ASSERT(memResourcePtr->Type == CmResourceTypeMemory);
    deviceContextPtr->RegistersPtr = (IMXCAN_REGISTERS *)(MmMapIoSpaceEx(memResourcePtr->u.Memory.Start, sizeof(IMXCAN_REGISTERS), PAGE_READWRITE | PAGE_NOCACHE));
    
    if (deviceContextPtr->RegistersPtr == NULL) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "MmMapIoSpaceEx(...) failed. memResourcePtr->u.Memory.Start = 0x%llx, sizeof(IMXCAN_REGISTERS) = %lu\n", memResourcePtr->u.Memory.Start.QuadPart, sizeof(IMXCAN_REGISTERS)));
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    else {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "MmMapIoSpaceEx(...) returns  memResourcePtr->u.Memory.Start = 0x%llx, sizeof(IMXCAN_REGISTERS) = %lu\n", memResourcePtr->u.Memory.Start.QuadPart, sizeof(IMXCAN_REGISTERS)));
    }
 

    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FlexCAN: imxcanEvtDevicePrepareHardware - return Success\n"));
    return STATUS_SUCCESS;
}

NTSTATUS ImxCanEvtDeviceReleaseHardware(
    _In_    WDFDEVICE       WdfDevice,
    _In_    WDFCMRESLIST    ResourcesTranslated
)
{
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    PAGED_CODE();

    PDEV_CONTEXT deviceContextPtr = ImxCanGetDeviceContext(WdfDevice);

    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FlexCAN: imxcanEvtDeviceReleaseHardware\n"));

    if (deviceContextPtr->RegistersPtr != NULL) {
        MmUnmapIoSpace(deviceContextPtr->RegistersPtr, sizeof(IMXCAN_REGISTERS));

        deviceContextPtr->RegistersPtr = NULL;
    }
    return STATUS_SUCCESS;
}

#pragma warning(push)
#pragma warning(disable:4189) /* local variable is initialized but not referenced */
NTSTATUS ImxCanSoftReset(PDEV_CONTEXT DeviceContextPtr)
{
    PAGED_CODE();

    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FlexCAN: ImxCanSoftReset\n"));
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
#if DBG
    FLEXCAN_DumpRegisters(registersPtr);
    //FLEXCAN_Deinit(registersPtr);
    //FLEXCAN_DumpRegisters(registersPtr);
#endif
    return STATUS_SUCCESS;
}
#pragma warning(pop)

NTSTATUS ImxCanEvtDeviceD0Entry(
    _In_    WDFDEVICE               WdfDevice,
    _In_    WDF_POWER_DEVICE_STATE  PreviousState
)
{
    UNREFERENCED_PARAMETER(PreviousState);
    PAGED_CODE();

    PDEV_CONTEXT deviceContextPtr = ImxCanGetDeviceContext(WdfDevice);
    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FlexCAN: ImxCanEvtDeviceD0Entry\n"));

    NTSTATUS status = ImxCanSoftReset(deviceContextPtr);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "ImxCanSoftReset(...) failed. (status = %!STATUS!)\n",status));
        return status;
    }
    
    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FlexCAN: ImxCanEvtDeviceD0Entry - return Success\n"));
    return STATUS_SUCCESS;
}


NTSTATUS
ImxCanCreateDeviceInterface(
    PDEV_CONTEXT DeviceContextPtr
)
{
    PAGED_CODE();

    NTSTATUS status = WdfDeviceCreateDeviceInterface(DeviceContextPtr->WdfDevice, &GUID_DEVINTERFACE_CAN_CONTROLLER, NULL);
    if (!NT_SUCCESS(status)) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfDeviceCreateDeviceInterface(...) failed. (status = %!STATUS!)\n", status));
        return status;
    }

    /* Retrieve device interface symbolic link string */
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = DeviceContextPtr->WdfDevice;
        status = WdfStringCreate(NULL, &attributes, &DeviceContextPtr->DeviceInterfaceSymlinkName);
        if (!NT_SUCCESS(status)) {
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfStringCreate(...) failed. (status = %!STATUS!)\n", status));
            return status;
        }
        status = WdfDeviceRetrieveDeviceInterfaceString(DeviceContextPtr->WdfDevice, &GUID_DEVINTERFACE_CAN_CONTROLLER, NULL, DeviceContextPtr->DeviceInterfaceSymlinkName);
        if (!NT_SUCCESS(status)) {
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfDeviceRetrieveDeviceInterfaceString(...) failed. (status = %!STATUS!, GUID_DEVINTERFACE_CAN_CONTROLLER = %!GUID!)\n", status, &GUID_DEVINTERFACE_CAN_CONTROLLER));
            return status;
        }
        WdfStringGetUnicodeString(DeviceContextPtr->DeviceInterfaceSymlinkName, &DeviceContextPtr->DeviceInterfaceSymlinkNameWsz);
    }

    return STATUS_SUCCESS;
}

IMXCAN_PAGED_SEGMENT_END;