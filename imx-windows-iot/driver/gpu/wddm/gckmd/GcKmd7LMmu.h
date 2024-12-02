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

#pragma once

#include "GcKmd7L.h"
#include "GcKmdAdapter.h"
#include "GcKmdAllocation.h"

#include "Gc7LMmu.h"
#include "GcKmd7LMmuCommon.h"

class GcKm7LAdapter;

class GcKm7LMmu: public GcKm7LMmuCommon
{
public:

    GcKm7LMmu(
        GcKm7LAdapter  *pAdapter)
    {
        m_pAdapter = pAdapter;

        m_VideoMemorySize = 0;
        m_pVideoMemory = nullptr;
        m_VideoMemoryPhysicalAddress.QuadPart = 0;

        m_ImplicitSysMemSegmentId = LOCAL_VIDMEM_SEGMENT_ID + 1;

        m_bEnableShadow1MSTlb = true;
    }

    virtual NTSTATUS
    Start();

    virtual void
    Stop();

    virtual NTSTATUS
    QueryAdapterInfo(
        CONST DXGKARG_QUERYADAPTERINFO *pQueryAdapterInfo);

    virtual NTSTATUS
    CreateProcess(
        DXGKARG_CREATEPROCESS* pCreateProcess);

    virtual void
    SetAllocationSegmentInfo(
        GcKmAllocation         *pGcKmAllocation,
        DXGK_ALLOCATIONINFO    *pRtAllocInfo);

    virtual NTSTATUS
    BuildPagingBuffer(
        DXGKARG_BUILDPAGINGBUFFER  *pArgs);

    virtual ULONGLONG
    GetPhysicalAddress(
        const D3DGPU_PHYSICAL_ADDRESS  *pGpuPhysicalAddress);

    virtual void
    UpdateHwPageTable(
        GcKmContext    *pContext);

    virtual NTSTATUS
    SwitchGpuVaSpace(
        const D3DGPU_PHYSICAL_ADDRESS  *pRootPageTable);

    UINT GetLocalVidMemSegmentBase()
    {
        return m_VideoMemoryPhysicalAddress.LowPart;
    }

    virtual PBYTE
    GetLocalVidMemCpuVA(
        ULONGLONG   PageTablePhysicalAddress)
    {
        ULONGLONG LocalVidMemPhysicalStart = m_VideoMemoryPhysicalAddress.QuadPart;
        ULONGLONG LocalVidMemPhysicalEnd = LocalVidMemPhysicalStart + m_VideoMemorySize;

        if ((PageTablePhysicalAddress < LocalVidMemPhysicalStart) ||
            (PageTablePhysicalAddress >= LocalVidMemPhysicalEnd))
        {
            return nullptr;
        }

        return ((PBYTE)m_pVideoMemory) + (PageTablePhysicalAddress - LocalVidMemPhysicalStart);
    }

protected:

    GcKm7LAdapter  *m_pAdapter;

    size_t              m_VideoMemorySize;
    void               *m_pVideoMemory;
    PHYSICAL_ADDRESS    m_VideoMemoryPhysicalAddress;

    UINT                m_1stGlobalMTlbEntries;
    UINT                m_NumGlobalMTlbEntries;

    BOOLEAN             m_bEnableShadow1MSTlb;

    void               *m_pSafePages = NULL;
    PHYSICAL_ADDRESS    m_DummyPagePhysicalAddress;
};

const UINT  Gc7LPageSizes[] =
{
          4*1024,
         64*1024,
       1024*1024,
    16*1024*1024
};


struct GcKmGdiAllocation : public GcKmAllocation
{
    BYTE*                   m_pCpuAddress;
    D3DGPU_VIRTUAL_ADDRESS  m_GpuVa;
};


