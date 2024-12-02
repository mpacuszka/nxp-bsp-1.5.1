/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmdDevice.tmh"

#include "GcKmdAdapter.h"
#include "GcKmdDevice.h"
#include "GcKmdContext.h"
#include "GcKmdProcess.h"

#include "GcKmdUtil.h"

NTSTATUS __stdcall
GcKmDevice::Initialize(
    GcKmAdapter*            pAdapter,
    DXGKARG_CREATEDEVICE*   pCreateDevice)
{
    m_hRTDevice = pCreateDevice->hDevice;
    m_pGcKmAdapter = pAdapter;
    m_Flags = pCreateDevice->Flags;

    m_pGcKmProcess = (GcKmProcess*)pCreateDevice->hKmdProcess;

    return STATUS_SUCCESS;
}

NTSTATUS __stdcall
GcKmDevice::DdiCloseAllocation(
    IN_CONST_HANDLE                     hDevice,
    IN_CONST_PDXGKARG_CLOSEALLOCATION   pCloseAllocation)
{
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hDevice=%lx\n",
        __FUNCTION__, hDevice);

    for (UINT i = 0; i < pCloseAllocation->NumAllocations; i++)
    {
        GcKmDeviceAllocation* pGcKmDeviceAllocation = (GcKmDeviceAllocation*)pCloseAllocation->pOpenHandleList[i];

        ExFreePoolWithTag(pGcKmDeviceAllocation, GC_POOL_TAG_DEVICE_ALLOC);
    }

    return STATUS_SUCCESS;
}

GC_PAGED_SEGMENT_BEGIN; //===================================================

_Use_decl_annotations_
NTSTATUS
GcKmDevice::OpenAllocation(const DXGKARG_OPENALLOCATION* pOpenAllocation)
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    if (pOpenAllocation->NumAllocations != 1)
    {
        return STATUS_INVALID_PARAMETER;
    }

    GcKmDeviceAllocation* pGcKmDeviceAllocation;

    pGcKmDeviceAllocation = new (NonPagedPoolNx, GC_POOL_TAG_DEVICE_ALLOC)
                                GcKmDeviceAllocation();
    if (!pGcKmDeviceAllocation)
    {
        GC_LOG_LOW_MEMORY(
            "Failed to allocate memory for GcKmDeviceAllocation structure. (sizeof(GcKmDeviceAllocation)=%d)",
            sizeof(GcKmDeviceAllocation));

        return STATUS_NO_MEMORY;
    }

    const DXGKRNL_INTERFACE    *pDxgkInterface = m_pGcKmAdapter->GetDxgkInterface();
    DXGK_OPENALLOCATIONINFO    *pOpenAllocInfo = pOpenAllocation->pOpenAllocation;
    
    GcKmAllocation             *pGcKmAllocation;

    DXGKARGCB_GETHANDLEDATA     GetHandleData;
    DXGKARG_RELEASE_HANDLE      hReleaseHandle;

    GetHandleData.hObject = pOpenAllocInfo->hAllocation;
    GetHandleData.Type = DXGK_HANDLE_ALLOCATION;
    GetHandleData.Flags.DeviceSpecific = 0;

    pGcKmAllocation = static_cast<GcKmAllocation*>(
        pDxgkInterface->DxgkCbAcquireHandleData(&GetHandleData, &hReleaseHandle));

    DXGKARGCB_RELEASEHANDLEDATA ReleaseHandleData;

    ReleaseHandleData.ReleaseHandle = hReleaseHandle;
    ReleaseHandleData.Type = DXGK_HANDLE_ALLOCATION;

    pDxgkInterface->DxgkCbReleaseHandleData(ReleaseHandleData);

    //
    // Set up per device allocation object
    //
    pGcKmDeviceAllocation->m_hKMAllocation = pOpenAllocInfo->hAllocation;
    pGcKmDeviceAllocation->m_pGcKmAllocation = pGcKmAllocation;

    // Return the per device allocation object
    pOpenAllocInfo->hDeviceSpecificAllocation = pGcKmDeviceAllocation;

    GC_LOG_TRACE(
        "Successfully opened allocation. (Flags.Create=%d, Flags.ReadOnly=%d)",
        pOpenAllocation->Flags.Create,
        pOpenAllocation->Flags.ReadOnly);

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmDevice::CreateContext(
    DXGKARG_CREATECONTEXT*  pCreateContext)
{
    GcKmContext*    pContext;
    NTSTATUS        Status;

    pContext = new (NonPagedPoolNx, 'TCCG') GcKmContext();
    if (!pContext)
    {
        GC_LOG_LOW_MEMORY("Failed to allocate GcKmContext.");
        return STATUS_NO_MEMORY;
    }

    Status = pContext->Initialize(this, pCreateContext);
    if (NT_SUCCESS(Status))
    {
        pCreateContext->hContext = pContext;
    }
    else
    {
        delete pContext;
    }

    return Status;
}

GC_PAGED_SEGMENT_END; //=====================================================

