/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"

#include "imx8mglobal.tmh"

NTSTATUS
GcKmAdapter::AddAdapter(
    IN_CONST_PDEVICE_OBJECT     PhysicalDeviceObject,
    OUT_PPVOID                  MiniportDeviceContext)
{
    NTSTATUS    status;
    WCHAR   deviceID[512];
    ULONG   dataLen;

    status = IoGetDeviceProperty(PhysicalDeviceObject, DevicePropertyHardwareID, sizeof(deviceID), deviceID, &dataLen);
    if (!NT_SUCCESS(status))
    {
        GC_LOG_ERROR(
            "Failed to get DevicePropertyHardwareID from PDO. (status=%!STATUS!)",
            status);

        return status;
    }

    Imx8mDodAdapter    *pDodAdapter = new Imx8mDodAdapter(PhysicalDeviceObject, deviceID);

    if (!pDodAdapter)
    {
        GC_LOG_LOW_MEMORY("Failed to allocate GcKmAdapter.");

        return STATUS_NO_MEMORY;
    }

    *MiniportDeviceContext = pDodAdapter;

    return STATUS_SUCCESS;
}

