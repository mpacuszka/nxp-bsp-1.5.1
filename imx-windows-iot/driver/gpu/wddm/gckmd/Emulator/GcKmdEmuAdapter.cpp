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

#pragma once

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmdEmuAdapter.tmh"

#include "GcKmdEmuAdapter.h"
#include "GcKmd7LUtil.h"
#include "GcKmdContext.h"
#include "GcKmdGlobal.h"
#include "GcKmdUtil.h"


void* GcKmEmuAdapter::operator new(size_t size)
{
    return ExAllocatePoolWithTag(NonPagedPoolNx, size, 'DAL7'); // Tag for 7L Adapter
}

void GcKmEmuAdapter::operator delete(void* ptr)
{
    ExFreePool(ptr);
}

GcKmEmuAdapter::GcKmEmuAdapter(
    IN_CONST_PDEVICE_OBJECT PhysicalDeviceObject,
    WCHAR  *pwszDeviceId) :
    GcKmAdapter(PhysicalDeviceObject, pwszDeviceId, true),
    m_Mmu(this),
    m_3DNode(&m_Mmu)
{
    // do nothing
}

NTSTATUS
GcKmEmuAdapter::Start(
    IN_PDXGK_START_INFO     DxgkStartInfo,
    IN_PDXGKRNL_INTERFACE   DxgkInterface,
    OUT_PULONG              NumberOfVideoPresentSources,
    OUT_PULONG              NumberOfChildren)
{
    NTSTATUS status;

    do
    {
        status = GcKmAdapter::Start(DxgkStartInfo, DxgkInterface, NumberOfVideoPresentSources, NumberOfChildren);
        if (!NT_SUCCESS(status))
        {
            break;
        }

        status = m_3DNode.Start(DxgkInterface);
        if (!NT_SUCCESS(status))
        {
            break;
        }
    } while (FALSE);

    if (!NT_SUCCESS(status))
    {
        Stop();

        return status;
    }

    RegisterMmu(&m_Mmu);

    RegisterNode(GC_EMU_ENGINE_ORDINAL_3D, &m_3DNode);

    return status;
}

NTSTATUS GcKmEmuAdapter::Stop()
{
    // m_3DNode.Stop();

    return GcKmAdapter::Stop();
}

NTSTATUS
GcKmEmuAdapter::QueryAdapterInfo(
    IN_CONST_PDXGKARG_QUERYADAPTERINFO  pQueryAdapterInfo)
{
    GC_LOG_TRACE(
        "QueryAdapterInfo was called. (Type=%d)",
        pQueryAdapterInfo->Type);

    switch (pQueryAdapterInfo->Type)
    {
    case DXGKQAITYPE_DISPLAY_DRIVERCAPS_EXTENSION:
    case DXGKQAITYPE_DISPLAYID_DESCRIPTOR:
        return STATUS_NOT_SUPPORTED;

    case DXGKQAITYPE_DRIVERCAPS:
    {
        NTSTATUS Status = GcKmAdapter::QueryAdapterInfo(pQueryAdapterInfo);

        // Set these to make emulator a high-performance discrete hybrid GPU
        if (NT_SUCCESS(Status))
        {
            DWORD FeatureEnabled = 0;
            bool EnableHybridAdapter = false;
            if (ReadDwordRegistryValue(&m_DeviceInfo.DeviceRegistryPath, L"D3D9Hybrid", &FeatureEnabled))
            {
                EnableHybridAdapter = FeatureEnabled == 1;
            }

            if (EnableHybridAdapter)
            {
                DXGK_DRIVERCAPS* pDriverCaps = (DXGK_DRIVERCAPS*)pQueryAdapterInfo->pOutputData;

                pDriverCaps->HybridDiscrete = TRUE;
                pDriverCaps->MiscCaps.Detachable = 1;
                pDriverCaps->MemoryManagementCaps.CrossAdapterResource = 1;
            }
        }

        return Status;
    }

    default:
        return GcKmAdapter::QueryAdapterInfo(pQueryAdapterInfo);
    }
}

NTSTATUS
GcKmEmuAdapter::CollectDbgInfo(
    IN_CONST_PDXGKARG_COLLECTDBGINFO    pCollectDbgInfo)
{
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmEmuAdapter::ResetFromTimeout()
{
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmEmuAdapter::RestartFromTimeout()
{
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmEmuAdapter::Escape(
    IN_CONST_PDXGKARG_ESCAPE    pEscape)
{
    return STATUS_INVALID_PARAMETER;
}

NTSTATUS GcKmEmuAdapter::SetPowerState(
    IN_ULONG                DeviceUid,
    IN_DEVICE_POWER_STATE   DevicePowerState,
    IN_POWER_ACTION         ActionType)
{
    return STATUS_SUCCESS;
}

NTSTATUS GcKmEmuAdapter::GetNodeMetaData(
    UINT                            NodeOrdinal,
    OUT_PDXGKARG_GETNODEMETADATA    pGetNodeMetadata)
{
    NT_ASSERT(NodeOrdinal == 0);

    RtlZeroMemory(pGetNodeMetadata, sizeof(*pGetNodeMetadata));

    pGetNodeMetadata->EngineType = DXGK_ENGINE_TYPE_3D;

    RtlStringCbPrintfW(pGetNodeMetadata->FriendlyName,
        sizeof(pGetNodeMetadata->FriendlyName),
        L"3DNode%02X",
        NodeOrdinal);

    pGetNodeMetadata->GpuMmuSupported = 1;

    return STATUS_SUCCESS;
}


