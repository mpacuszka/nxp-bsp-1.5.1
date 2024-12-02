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

#include "GcKmd7L.h"

#include "GcKmd7LAdapter.h"
#include "GcKmd7LProcess.h"

#include "GcKmdGlobal.h"


extern "C" {

#include "gc_hal_kernel_driver.h"

extern gckGALDEVICE galDevice;

PCSTR PsGetProcessImageFileName(PEPROCESS);
}

NTSTATUS
GcKm7LProcess::Initialize(
    UINT    GlobalGpuVaAlignment,
    UINT    GlobalGpuVaBaseAddress,
    UINT    GlobalGpuVaSizeInByte)
{
    if (GcKmdGlobal::s_bShadowGpuVa)
    {
        return InitGlobalGpuVaShadow();
    }

    //
    // Exclude flat mapping range from UMD GPU VA allocation
    //

    NTSTATUS    Status = STATUS_SUCCESS;
    DXGKRNL_INTERFACE  *pDxgkInterface;

    pDxgkInterface = m_pGcKmAdapter->GetDxgkInterface();

    DXGKARGCB_RESERVEGPUVIRTUALADDRESSRANGE ReserveGpuVaRange = { 0 };

    ReserveGpuVaRange.hDxgkProcess  = m_hDxgkProcess;
    ReserveGpuVaRange.AllowUserModeMapping = 0;             // Block UMD usage

    ReserveGpuVaRange.Alignment = GlobalGpuVaAlignment;
    ReserveGpuVaRange.BaseAddress = GlobalGpuVaBaseAddress;
    ReserveGpuVaRange.SizeInBytes = GlobalGpuVaSizeInByte;

    if (ReserveGpuVaRange.SizeInBytes > 0)
    {
        Status = pDxgkInterface->DxgkCbReserveGpuVirtualAddressRange(
                                    pDxgkInterface->DeviceHandle,
                                    &ReserveGpuVaRange);
        if (!NT_SUCCESS(Status))
        {
            return Status;
        }

        NT_ASSERT(ReserveGpuVaRange.BaseAddress == ReserveGpuVaRange.StartVirtualAddress);
    }


    Status = AllocatePrivateSTlb();
    if(!NT_SUCCESS(Status))
    {
        return Status;
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKm7LProcess::InitGlobalGpuVaShadow()
{
    NTSTATUS    Status = STATUS_SUCCESS;

    if (m_Flags.SystemProcess)
    {
        return Status;
    }

    gctSIZE_T   gcPerProcessVaInPages = GC_7L_GPU_VA_SIZE_IN_4K_PAGES;
    PCSTR   ProcessName;

    ProcessName = PsGetProcessImageFileName(PsGetCurrentProcess());
    if (0 == strcmp(ProcessName, "dwm.exe"))
    {
        gcPerProcessVaInPages *= 2;
    }

    //
    // Allocate the global GPU address for the process
    //

    gceSTATUS   GcStatus;

    GcStatus = gckMMU_AllocatePagesEx(
                    galDevice->kernels[gcvCORE_MAJOR]->mmu,
                    gcPerProcessVaInPages, // Size in 4K pages
                    gcvVIDMEM_TYPE_GENERIC,
                    gcvPAGE_TYPE_4K,
                    FALSE,
                    &m_pPageTable, // Ponter to STLB page table
                    &m_GlobalGpuVa);
    if (GcStatus != gcvSTATUS_OK)
    {
        return STATUS_UNSUCCESSFUL;
    }

    //
    // Exclude the ranges from which UMD shouldn't allocate GPU VA
    //

    DXGKRNL_INTERFACE  *pDxgkInterface;

    pDxgkInterface = m_pGcKmAdapter->GetDxgkInterface();

    DXGKARGCB_RESERVEGPUVIRTUALADDRESSRANGE ReserveGpuVaRange = { 0 };

    ReserveGpuVaRange.hDxgkProcess  = m_hDxgkProcess;
    ReserveGpuVaRange.Alignment     = PAGE_SIZE;
    ReserveGpuVaRange.AllowUserModeMapping = 0;             // Block UMD usage

    //
    // The 1st MTLB entry is made invalid via GpuVaFirstValidAddress in INF
    // So the BaseAddress for lower exclusion starts from the 2nd MTLB entry
    //
    ReserveGpuVaRange.BaseAddress   = (1 << GC_7L_STLB_INDEX_BITS)*PAGE_SIZE;
    ReserveGpuVaRange.SizeInBytes   = m_GlobalGpuVa - ReserveGpuVaRange.BaseAddress;

    if (ReserveGpuVaRange.SizeInBytes > 0)
    {
        Status = pDxgkInterface->DxgkCbReserveGpuVirtualAddressRange(
                                    pDxgkInterface->DeviceHandle,
                                    &ReserveGpuVaRange);
        if (!NT_SUCCESS(Status))
        {
            return Status;
        }

        NT_ASSERT(ReserveGpuVaRange.BaseAddress == ReserveGpuVaRange.StartVirtualAddress);
    }

    //
    // The higher exclusion goes up to 4 GB
    //
    ReserveGpuVaRange.BaseAddress   = ((UINT64)m_GlobalGpuVa) + gcPerProcessVaInPages*PAGE_SIZE;
    ReserveGpuVaRange.SizeInBytes   = 1024ULL*1024ULL*1024ULL*4LL - ReserveGpuVaRange.BaseAddress;

    // Returned by Graphic Kernel Runtime
    ReserveGpuVaRange.StartVirtualAddress = 0;

    Status = pDxgkInterface->DxgkCbReserveGpuVirtualAddressRange(
                                pDxgkInterface->DeviceHandle,
                                &ReserveGpuVaRange);
    NT_ASSERT(((Status != STATUS_SUCCESS) ||
               (ReserveGpuVaRange.BaseAddress == ReserveGpuVaRange.StartVirtualAddress)));

    return Status;
}

NTSTATUS
GcKm7LProcess::AllocatePrivateSTlb()
{
    PHYSICAL_ADDRESS LowAddress = { 0 }, SkipBytes = { 0 };
    PHYSICAL_ADDRESS HighAddress = { 0 };
    HighAddress.QuadPart = 0xFFFFFFFF;

    m_pPrivateSTlbMdl = MmAllocatePagesForMdlEx(LowAddress,
                                                HighAddress,
                                                SkipBytes,
                                                m_PrivateSTlbSize,
                                                MmNonCached,
                                                MM_ALLOCATE_FULLY_REQUIRED);
    if (!m_pPrivateSTlbMdl)
    {
        return STATUS_NO_MEMORY;
    }

    m_pPrivateSTlbPfn = MmGetMdlPfnArray(m_pPrivateSTlbMdl);

    m_pPrivateSTlb = (Gc7LSTlbEntry1M*)MmMapLockedPagesSpecifyCache(m_pPrivateSTlbMdl,
                                                                    KernelMode,
                                                                    MmNonCached,
                                                                    NULL,
                                                                    FALSE,
                                                                    HighPagePriority);
    if (!m_pPrivateSTlb)
    {
        return STATUS_NO_MEMORY;
    }

    return STATUS_SUCCESS;
}

GcKm7LProcess::~GcKm7LProcess()
{
    if (m_GlobalGpuVa)
    {
        gckMMU_FreePages(
            galDevice->kernels[gcvCORE_MAJOR]->mmu,
            FALSE,
            gcvPAGE_TYPE_4K,
            m_GlobalGpuVa,
            m_pPageTable,
            GC_7L_GPU_VA_SIZE_IN_4K_PAGES);
    }

    if (m_pPrivateSTlbMdl)
    {
        if (m_pPrivateSTlb)
        {
            MmUnmapLockedPages(m_pPrivateSTlb, m_pPrivateSTlbMdl);
        }

        MmFreePagesFromMdl(m_pPrivateSTlbMdl);
        ExFreePool(m_pPrivateSTlbMdl);
    }
}


