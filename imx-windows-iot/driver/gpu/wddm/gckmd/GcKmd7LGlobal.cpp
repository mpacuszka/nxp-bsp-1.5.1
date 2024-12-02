/****************************************************************************
* Copyright (c) Microsoft Corporation.
*
*    Licensed under the MIT License.
*    Licensed under the GPL License.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmd7LGlobal.tmh"

#include "GcHw.h"
#include "GcKmdAdapter.h"
#include "GcKmd7LAdapter.h"
#include "GcKmdDevice.h"
#include "GcKmdProcess.h"
#include "GcKmdAllocation.h"
#include "GcKmdResource.h"

#include "GcKmdEmuAdapter.h"
#include "GcKmdGlobal.h"

#include "GcKmdUtil.h"

NTSTATUS
GcKmAdapter::AddAdapter(
    IN_CONST_PDEVICE_OBJECT     PhysicalDeviceObject,
    OUT_PPVOID                  MiniportDeviceContext)
{
    NTSTATUS status;
    WCHAR deviceID[512];
    ULONG dataLen;

    status = IoGetDeviceProperty(PhysicalDeviceObject, DevicePropertyHardwareID, sizeof(deviceID), deviceID, &dataLen);
    if (!NT_SUCCESS(status))
    {
        GC_LOG_ERROR(
            "Failed to get DevicePropertyHardwareID from PDO. (status=%!STATUS!)",
            status);

        return status;
    }

    GcKmAdapter* pGcKmAdapter = NULL;

    //
    // Create subclass of adapter based on device ID
    //

#ifdef _ARM64_
    if (0 == wcscmp(deviceID, GC_7000_LITE_MQ))
    {
        pGcKmAdapter = new GcKm7LMQAdapter(PhysicalDeviceObject, deviceID);
    }
    else if (0 == wcscmp(deviceID, GC_7000_LITE_8X))
    {
        pGcKmAdapter = new GcKm7LMQAdapter(PhysicalDeviceObject, deviceID);
    }
    else if (0 == wcscmp(deviceID, GC_7000_LITE_MP))
    {
        GcKmdGlobal::s_bMapDummyPage = true;

        pGcKmAdapter = new GcKm7LMPAdapter(PhysicalDeviceObject, deviceID);
    }
    else if (0 == wcscmp(deviceID, GC_7000_LITE_MN))
    {
        GcKmdGlobal::s_bMapDummyPage = true;

        pGcKmAdapter = new GcKm7LMPAdapter(PhysicalDeviceObject, deviceID);
    }
#else
    {
        //
        // Device ID for the emulated device is in the format of EmuGC7LDevice_NNNN
        //

        C_ASSERT(15 == (sizeof(EMU_GC_7000_LITE) / sizeof(WCHAR)));
        WCHAR   deviceName[15];
        UINT    deviceSeqNum;

        if ((2 == swscanf_s(deviceID, L"%14s%4u", deviceName, 15, &deviceSeqNum)) &&
            (0 == wcscmp(deviceName, EMU_GC_7000_LITE)))
        {
            pGcKmAdapter = new GcKmEmuAdapter(PhysicalDeviceObject, deviceID);
        }
        else
        {
            GC_LOG_ERROR("Device with (deviceID=%S) is not support.", deviceID);

            return STATUS_NOT_SUPPORTED;
        }
    }
#endif

    if (!pGcKmAdapter)
    {
        GC_LOG_LOW_MEMORY("Failed to allocate GcKmAdapter.");

        return STATUS_NO_MEMORY;
    }

    *MiniportDeviceContext = pGcKmAdapter;

    return STATUS_SUCCESS;
}


