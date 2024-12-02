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
#include "GcKmd7LEmuMmu.tmh"

#include "GcKmd7LEmuMmu.h"
#include "GcKmdEmuAdapter.h"
#include "GcKmdProcess.h"
#include "GcKmdAllocation.h"

#include "GcKmdUtil.h"


NTSTATUS
GcKm7LEmuMmu::CreateProcess(
    DXGKARG_CREATEPROCESS  *pCreateProcess)
{
    GcKmProcess    *pGcKmProcess;

    pGcKmProcess = new (NonPagedPoolNx, GC_POOL_TAG_PROCESS) GcKmProcess(m_pAdapter, pCreateProcess);
    if (NULL == pGcKmProcess)
    {
        return STATUS_NO_MEMORY;
    }

    pCreateProcess->hKmdProcess = pGcKmProcess;

    return STATUS_SUCCESS;
}

NTSTATUS
GcKm7LEmuMmu::BuildPagingBuffer(
    DXGKARG_BUILDPAGINGBUFFER  *pBuildPagingBuffer)
{
    NTSTATUS    Status = STATUS_SUCCESS;

    switch (pBuildPagingBuffer->Operation)
    {
    case DXGK_OPERATION_FLUSH_TLB:
        break;

    default:
        Status = GcKm7LMmuCommon::BuildPagingBuffer(pBuildPagingBuffer, 0, 0LL);
        break;
    }

    return Status;
}


ULONGLONG
GcKm7LEmuMmu::GetPhysicalAddress(
    const D3DGPU_PHYSICAL_ADDRESS  *pGpuPhysicalAddress)
{
    NT_ASSERT(pGpuPhysicalAddress->SegmentId == m_ImplicitSysMemSegmentId);

    return pGpuPhysicalAddress->SegmentOffset;
}

void
GcKm7LEmuMmu::UpdateHwPageTable(
    GcKmContext    *pContext)
{
}

NTSTATUS
GcKm7LEmuMmu::SwitchGpuVaSpace(
    const D3DGPU_PHYSICAL_ADDRESS  *pRootPageTable)
{
    return STATUS_SUCCESS;
}


