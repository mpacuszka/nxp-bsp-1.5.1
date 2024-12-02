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
#include "GcKmd7LMmu.tmh"

#include "GcKmd7LMmu.h"
#include "GcKmd7LProcess.h"
#include "GcKmdAllocation.h"
#include "GcKmd7LNode.h"
#include "GcKmd7LContext.h"

#include "GcKmdGlobal.h"
#include "GcKmdUtil.h"


extern "C" {

#include "gc_hal_kernel_driver.h"

extern gckGALDEVICE galDevice;

}


NTSTATUS
GcKm7LMmu::Start()
{
    NTSTATUS    Status;

    Status = GcKm7LMmuCommon::Start();
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    PHYSICAL_ADDRESS LowestAcceptableAddress;
    LowestAcceptableAddress.QuadPart = 0;

    //
    // Set to 4GB for allocating memory for DCSS, STLB and VPU
    //

    PHYSICAL_ADDRESS HighestAcceptableAddress;
    HighestAcceptableAddress.QuadPart = 4ULL*1024ULL*1024ULL*1024ULL - 1ULL;

    //
    // BoundaryAddressMultiple is used to ensure Local VidMem segment starts at
    // 16MB aligned address, so that large allocations can have physical address
    // that is aligned to 1MB or 16MB
    //

    PHYSICAL_ADDRESS BoundaryAddressMultiple;
    BoundaryAddressMultiple.QuadPart = GC_7L_LOCAL_VIDMEM_SEGMENT_SIZE;

    m_VideoMemorySize = GC_7L_LOCAL_VIDMEM_SEGMENT_SIZE;
    C_ASSERT(GC_7L_LOCAL_VIDMEM_SEGMENT_SIZE <= GC_7L_LOCAL_VIDMEM_SEGMENT_SIZE_MAX);

    if (gcPlatform->videoMemoryBase.QuadPart != 0 && gcPlatform->videoMemorySize > 0)
    {
        m_pVideoMemory = MmMapIoSpace(gcPlatform->videoMemoryBase, gcPlatform->videoMemorySize, MmWriteCombined);
        m_VideoMemorySize = gcPlatform->videoMemorySize;
        m_VideoMemoryPhysicalAddress = gcPlatform->videoMemoryBase;
        gcmkPRINT("Galcore Info: VideoMemoryBase=0x%llx VideoMemoryBaseSize=0x%zx\n", gcPlatform->videoMemoryBase.QuadPart, gcPlatform->videoMemorySize);
    }
    else
    {
        while (m_pVideoMemory == NULL)
        {
            m_pVideoMemory = MmAllocateContiguousMemorySpecifyCache(
                m_VideoMemorySize,
                LowestAcceptableAddress,
                HighestAcceptableAddress,
                BoundaryAddressMultiple,
                MmWriteCombined);
            if (m_pVideoMemory != NULL)
            {
                break;
            }

            m_VideoMemorySize >>= 1;
            if (m_VideoMemorySize < GC_7L_LOCAL_VIDMEM_SEGMENT_SIZE_MIN)
            {
                break;
            }

            BoundaryAddressMultiple.QuadPart >>= 1;
        }

        if (m_pVideoMemory)
        {
            m_VideoMemoryPhysicalAddress = MmGetPhysicalAddress(m_pVideoMemory);
        }
        else
        {
            return STATUS_NO_MEMORY;
        }
    }

    NT_ASSERT(1 == galDevice->kernels[0]->mmu->gpuPhysicalRangeCount);
    NT_ASSERT(2 == galDevice->kernels[0]->mmu->gpuAddressRangeCount);

    UINT64  MtlbEntryCoverage = 1 << (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS);
    UINT64  StartGlobalGpuVa, EndGlobalGpuVa;

    StartGlobalGpuVa = galDevice->kernels[0]->mmu->gpuAddressRanges[1].start;
    EndGlobalGpuVa = StartGlobalGpuVa + galDevice->kernels[0]->mmu->gpuAddressRanges[1].size;

    EndGlobalGpuVa += (MtlbEntryCoverage - 1);
    EndGlobalGpuVa &= (~(MtlbEntryCoverage - 1));

    m_1stGlobalMTlbEntries = (UINT)(StartGlobalGpuVa/MtlbEntryCoverage);
    m_NumGlobalMTlbEntries = ((UINT)(EndGlobalGpuVa/MtlbEntryCoverage)) - m_1stGlobalMTlbEntries;

    if (GcKmdGlobal::s_bMapDummyPage)
    {
        BoundaryAddressMultiple.QuadPart = 128*1024;

        m_pSafePages = MmAllocateContiguousMemorySpecifyCache(
                        128*1024,
                        LowestAcceptableAddress,
                        HighestAcceptableAddress,
                        BoundaryAddressMultiple,
                        MmWriteCombined);

        if (m_pSafePages)
        {
            m_DummyPagePhysicalAddress = MmGetPhysicalAddress(m_pSafePages);

            m_DummySTlbEntry.Value = 0;
            m_DummySTlbEntry.Present = 1;
            m_DummySTlbEntry.Writable = 1;
            m_DummySTlbEntry.PageIndex = m_DummyPagePhysicalAddress.QuadPart >> 12;

            UINT   *pSTlbEntry = (UINT*)(((BYTE*)m_pSafePages) + 64*1024);
            for (UINT i = 0; i < (16*1024/sizeof(UINT)); i++)
            {
                *pSTlbEntry++ = m_DummySTlbEntry.Value;
            }

            m_DummyMTlbEntry.Value = 0;
            m_DummyMTlbEntry.Present = 1;
            m_DummyMTlbEntry.PageTableIndex = (m_DummyPagePhysicalAddress.QuadPart + 64*1024) >> 12;
        }
    }

    return STATUS_SUCCESS;
}

void
GcKm7LMmu::Stop()
{
    if (m_pVideoMemory != NULL)
    {
        if (gcPlatform->videoMemoryBase.QuadPart != 0 && gcPlatform->videoMemorySize > 0)
        {
            MmUnmapIoSpace(m_pVideoMemory, m_VideoMemorySize);
        }
        else
        {
            MmFreeContiguousMemory(m_pVideoMemory);
        }

        m_pVideoMemory = NULL;
        m_VideoMemorySize = 0;
        m_VideoMemoryPhysicalAddress.QuadPart = 0;
    }

    if (m_pSafePages)
    {
        MmFreeContiguousMemory(m_pSafePages);
    }

    GcKm7LMmuCommon::Stop();
}

NTSTATUS
GcKm7LMmu::QueryAdapterInfo(
    CONST DXGKARG_QUERYADAPTERINFO* pQueryAdapterInfo)
{
    NTSTATUS    Status;

    Status = GcKm7LMmuCommon::QueryAdapterInfo(pQueryAdapterInfo);
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    switch (pQueryAdapterInfo->Type)
    {
    case DXGKQAITYPE_GPUMMUCAPS:
    {
        DXGK_GPUMMUCAPS* pGpuMmuCaps = (DXGK_GPUMMUCAPS*)pQueryAdapterInfo->pOutputData;

        pGpuMmuCaps->SysMem64KBPageSupported = 1;
        pGpuMmuCaps->LeafPageTableSizeFor64KPagesInBytes = GC_7L_STLB_SIZE/16;

        //
        // Paging process' GPU VA space is restricted to 1/4th of Local VidMem Segment's size,
        // So it need to be larger than 4 times of 16MB for large page to operate properly
        //

        if (m_VideoMemorySize > (4*GC_7L_LARGE_PAGE_16MB_SIZE))
        {
            pGpuMmuCaps->LargePageSupported = 1;
        }

#if 0
        //
        // Take advantage of ExplicitPageTableInvalidation to manage internal STlb
        //

        pGpuMmuCaps->ExplicitPageTableInvalidation = 1;
#endif
    }
    break;

    case DXGKQAITYPE_PAGETABLELEVELDESC:
    {
        //
        // Put both MTLB and STLB into local video memory segment to avoid dynamic mapping/unmapping
        //

        DXGK_PAGE_TABLE_LEVEL_DESC* pPageTableLevelDesc = (DXGK_PAGE_TABLE_LEVEL_DESC*)pQueryAdapterInfo->pOutputData;

        pPageTableLevelDesc->PageTableSegmentId = pPageTableLevelDesc->PagingProcessPageTableSegmentId = LOCAL_VIDMEM_SEGMENT_ID;
    }
    break;

    case DXGKQAITYPE_QUERYSEGMENT4:
    {
        DXGK_QUERYSEGMENTOUT4* pSegmentInfo = (DXGK_QUERYSEGMENTOUT4*)pQueryAdapterInfo->pOutputData;

        if (!pSegmentInfo[0].pSegmentDescriptor)
        {
            pSegmentInfo->NbSegment += 1;
        }
        else
        {
            DXGK_SEGMENTDESCRIPTOR4* pLocalVidMemSegmentDesc = (DXGK_SEGMENTDESCRIPTOR4*)(pSegmentInfo->pSegmentDescriptor +
                                                                                          pSegmentInfo->SegmentDescriptorStride*(pSegmentInfo->NbSegment - 1));

            memset(pLocalVidMemSegmentDesc, 0, sizeof(*pLocalVidMemSegmentDesc));

            pLocalVidMemSegmentDesc->BaseAddress.QuadPart = 0LL;    // Gpu base physical address
            pLocalVidMemSegmentDesc->Flags.CpuVisible = true;
#if CACHE_COHERENT_PLATFORM
            pLocalVidMemSegmentDesc->Flags.CacheCoherent = true;
#endif
            pLocalVidMemSegmentDesc->Flags.DirectFlip = true;
            pLocalVidMemSegmentDesc->Flags.Use64KBPages = true;
            pLocalVidMemSegmentDesc->CpuTranslatedAddress = m_VideoMemoryPhysicalAddress;   // Cpu base physical address
            pLocalVidMemSegmentDesc->Size = m_VideoMemorySize;
        }
    }
    break;
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKm7LMmu::CreateProcess(
    DXGKARG_CREATEPROCESS  *pCreateProcess)
{
    GcKm7LProcess  *pGcKm7LProcess;
    NTSTATUS        Status;

    pGcKm7LProcess = new (NonPagedPoolNx, GC_POOL_TAG_PROCESS) GcKm7LProcess(m_pAdapter, pCreateProcess);
    if (NULL == pGcKm7LProcess)
    {
        return STATUS_NO_MEMORY;
    }

    Status = pGcKm7LProcess->Initialize(
                                1 << (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS),
                                m_1stGlobalMTlbEntries << (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS),
                                m_NumGlobalMTlbEntries << (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS));
    if (!NT_SUCCESS(Status))
    {
        delete pGcKm7LProcess;
        return Status;
    }

    pCreateProcess->hKmdProcess = pGcKm7LProcess;

    return STATUS_SUCCESS;
}

void
GcKm7LMmu::SetAllocationSegmentInfo(
    GcKmAllocation* pGcKmAllocation,
    DXGK_ALLOCATIONINFO* pRtAllocInfo)
{
    if (GcKmdGlobal::s_bLimitAllocBelow4gbPa)
    {
        // override preference to put everything in local segment
        pGcKmAllocation->m_memorySegmentId = LOCAL_VIDMEM_SEGMENT_ID;
    }

    if (pGcKmAllocation->m_memorySegmentId == APERTURE_SEGMENT_ID)
    {
        GcKm7LMmuCommon::SetAllocationSegmentInfo(pGcKmAllocation, pRtAllocInfo);

        if (GcKmdGlobal::s_bLocalVidMemGeneralUse &&
            (!(pGcKmAllocation->m_isShared || pGcKmAllocation->m_isCpuVisible)))
        {
            pRtAllocInfo->PreferredSegment.SegmentId0 = LOCAL_VIDMEM_SEGMENT_ID;
            pRtAllocInfo->PreferredSegment.Direction0 = 0;

            pRtAllocInfo->PreferredSegment.SegmentId1 = m_ImplicitSysMemSegmentId;
            pRtAllocInfo->PreferredSegment.Direction1 = 0;

            pRtAllocInfo->SupportedReadSegmentSet |= 1 << (LOCAL_VIDMEM_SEGMENT_ID - 1);
            pRtAllocInfo->SupportedWriteSegmentSet |= 1 << (LOCAL_VIDMEM_SEGMENT_ID - 1);
        }
    }
    else
    {
        pRtAllocInfo->PreferredSegment.Value = 0;
        pRtAllocInfo->PreferredSegment.SegmentId0 = LOCAL_VIDMEM_SEGMENT_ID;
        pRtAllocInfo->PreferredSegment.Direction0 = 0;

        pRtAllocInfo->SupportedReadSegmentSet = 1 << (LOCAL_VIDMEM_SEGMENT_ID - 1);
        pRtAllocInfo->SupportedWriteSegmentSet = 1 << (LOCAL_VIDMEM_SEGMENT_ID - 1);
    }

    for (UINT i = 1; i < ARRAYSIZE(Gc7LPageSizes); i++)
    {
        if ((pRtAllocInfo->Size & (Gc7LPageSizes[i] - 1)) == 0)
        {
            pRtAllocInfo->Alignment = Gc7LPageSizes[i];
        }
        else
        {
            break;
        }
    }
}

#define GC_7L_ENABLE_FILL_VIRTUAL       1
#define GC_7L_ENABLE_TRANSFER_VIRTUAL   1

NTSTATUS
GcKm7LMmu::BuildPagingBuffer(
    DXGKARG_BUILDPAGINGBUFFER  *pBuildPagingBuffer)
{
    NTSTATUS    Status = STATUS_SUCCESS;

    if (pBuildPagingBuffer->DmaBufferWriteOffset == 0)
    {
        // New buffer - zero out the private data
        memset(pBuildPagingBuffer->pDmaBufferPrivateData, 0, pBuildPagingBuffer->DmaBufferPrivateDataSize);
    }

    switch (pBuildPagingBuffer->Operation)
    {
    case DXGK_OPERATION_UPDATE_PAGE_TABLE:
    {
        DXGK_BUILDPAGINGBUFFER_UPDATEPAGETABLE* pUpdatePageTable;

        pUpdatePageTable = &pBuildPagingBuffer->UpdatePageTable;

        DXGK_PTE* pRtPte = pUpdatePageTable->pPageTableEntries;

        GcKm7LMmuCommon::BuildPagingBuffer(pBuildPagingBuffer, LOCAL_VIDMEM_SEGMENT_ID, m_VideoMemoryPhysicalAddress.QuadPart);

        GcKm7LProcess  *pGcKm7LProcess = static_cast<GcKm7LProcess *>(pUpdatePageTable->hProcess);

        UINT    RtPteInc = pUpdatePageTable->Flags.Repeat ? 0 : 1;

        if ((TRUE == GcKmdGlobal::s_bShadowGpuVa) &&
            (0 == pUpdatePageTable->PageTableLevel) &&
            (!pGcKm7LProcess->m_Flags.SystemProcess))
        {
            // Set up STlb

            NT_ASSERT(pUpdatePageTable->FirstPteVirtualAddress);

            ULONGLONG       CpuPageAddress;
            gctPHYS_ADDR_T  GpuPageAddress;
            gctUINT32      *pGlobalSTlbEntry;

            pGlobalSTlbEntry = ((gctUINT32 *)pGcKm7LProcess->m_pPageTable) +
                                ((pUpdatePageTable->FirstPteVirtualAddress - pGcKm7LProcess->m_GlobalGpuVa)/PAGE_SIZE);

            Gc7LSTlbEntry   UnmappedSTlbEntry;

            UnmappedSTlbEntry.Value = 0;
            UnmappedSTlbEntry.EnableException = 1;

            for (UINT i = 0; i < pUpdatePageTable->NumPageTableEntries; i++)
            {
                //
                // Set up STlb for global GPU VA/aperture space
                //

                if (pRtPte->Valid)
                {
                    CpuPageAddress = pRtPte->PageAddress << 12;
                    if (pRtPte->Segment == LOCAL_VIDMEM_SEGMENT_ID)
                    {
                        CpuPageAddress += m_VideoMemoryPhysicalAddress.QuadPart;
                    }

                    gckOS_CPUPhysicalToGPUPhysical(
                        galDevice->os,
                        CpuPageAddress,
                        &GpuPageAddress);

                    gckMMU_SetPage(
                        galDevice->kernels[gcvCORE_MAJOR]->mmu,
                        GpuPageAddress,
                        gcvPAGE_TYPE_4K,
                        pRtPte->ReadOnly ? 0 : 1,
                        pGlobalSTlbEntry);
                }
                else
                {
                    *pGlobalSTlbEntry = UnmappedSTlbEntry.Value;
                }

                pRtPte += RtPteInc;

                pGlobalSTlbEntry++;
            }
        }

        GcKm7LProcess* pProcess = (GcKm7LProcess*)pUpdatePageTable->hProcess;

        if (pUpdatePageTable->Flags.InitialUpdate &&
            (1 == pUpdatePageTable->PageTableLevel))
        {
            NT_ASSERT(pUpdatePageTable->NumPageTableEntries == (1 << GC_7L_MTLB_INDEX_BITS));

            // Copy gloabl MTlb entries into per process MTlb

            memcpy(
                ((Gc7LMTlbEntry4K *)pUpdatePageTable->PageTableAddress.CpuVirtual) + m_1stGlobalMTlbEntries,
                ((Gc7LMTlbEntry4K *)galDevice->kernels[0]->mmu->mtlbLogical) + m_1stGlobalMTlbEntries,
                sizeof(Gc7LMTlbEntry4K)*m_NumGlobalMTlbEntries);

            Gc7LMTlbEntry4K    *pGlobalMTlb = (Gc7LMTlbEntry4K *)galDevice->kernels[0]->mmu->mtlbLogical;
            for (UINT i = 0; i < m_NumGlobalMTlbEntries; i++)
            {
                pProcess->UpdateSTlbPageSize(
                            m_1stGlobalMTlbEntries + i,
                            pGlobalMTlb[m_1stGlobalMTlbEntries + i].STlbPageSize);
            }
        }

        if (pUpdatePageTable->PageTableLevel)
        {
            pProcess->UpdateMTlbEntries(
                        (Gc7LMTlbEntry4K *)pUpdatePageTable->PageTableAddress.CpuVirtual,
                        pUpdatePageTable->StartIndex,
                        pUpdatePageTable->NumPageTableEntries,
                        m_DummyMTlbEntry.Value);

            DXGK_PTE   *pRtPde = pUpdatePageTable->pPageTableEntries;
            for (UINT i = 0; i < pUpdatePageTable->NumPageTableEntries; i++, pRtPde++)
            {
                if (pRtPde->LargePage)
                {
                    pProcess->UpdateSTlbPageSize(
                                pUpdatePageTable->StartIndex + i,
                                STlbPageSize16M,
                                pRtPde->ReadOnly);

                    ULONGLONG           GpuAddress1M = (pRtPde->PageTableAddress << 12) + m_VideoMemoryPhysicalAddress.QuadPart;
                    Gc7LSTlbEntry1M    *pInternalSTlb;

                    UINT MTlbIndex              = pUpdatePageTable->StartIndex + i;
                    pInternalSTlb               = pProcess->GetPrivateSTlb(MTlbIndex);
                    ULONG64 STlbPhysicalAddress = pProcess->GetPrivateSTlbPhysAddr(pInternalSTlb);

                    for (UINT j = 0; j < 16; j++, pInternalSTlb++)
                    {
                        pInternalSTlb->Value = 1; // Initialize the STlb entry and set Present to 1

                        pInternalSTlb->PageIndex    = (GpuAddress1M >> 20);
                        pInternalSTlb->PageIndexExt = (GpuAddress1M >> 32);

                        if (pRtPde->ReadOnly)
                        {
                            pInternalSTlb->WriteException = 1;
                        }
                        else
                        {
                            pInternalSTlb->Writable = 1;
                        }

                        GpuAddress1M += (1024*1024);
                    }

                    Gc7LMTlbEntry1M* pHwMtlbEntry = (Gc7LMTlbEntry1M* )pUpdatePageTable->PageTableAddress.CpuVirtual;
                    pHwMtlbEntry += MTlbIndex;

                    pHwMtlbEntry->STlbPageSize   = STlbPageSize16M;
                    pHwMtlbEntry->PageTableIndex = STlbPhysicalAddress >> 6;

                }
            }
        }
        else
        {
            pProcess->UpdateSTlbPageSize(
                        (UINT)(pUpdatePageTable->FirstPteVirtualAddress >> (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS)),
                        pUpdatePageTable->Flags.Use64KBPages ? STlbPageSize64K : STlbPageSize4K);

            if (m_bEnableShadow1MSTlb &&
                (pUpdatePageTable->FirstPteVirtualAddress >= GC_7L_1MB_PAGE_GPU_VA_START) &&
                (pUpdatePageTable->FirstPteVirtualAddress < GC_7L_1MB_PAGE_GPU_VA_END) &&
                pUpdatePageTable->Flags.Use64KBPages)
            {
                D3DGPU_VIRTUAL_ADDRESS  AffectedAddressStart = pUpdatePageTable->FirstPteVirtualAddress & ~(1024*1024 - 1);
                D3DGPU_VIRTUAL_ADDRESS  RtGpuVaEnd = pUpdatePageTable->FirstPteVirtualAddress + pUpdatePageTable->NumPageTableEntries*64*1024;
                D3DGPU_VIRTUAL_ADDRESS  AffectedAddressEnd = (RtGpuVaEnd + 1024*1024 - 1) & ~(1024*1024 - 1);

                UINT    NumPageTableEntries = (UINT)((AffectedAddressEnd - AffectedAddressStart)/(64*1024));

                UINT                MTlbIndex = (UINT)(AffectedAddressStart >> (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS));
                Gc7LSTlbEntry1M    *pPrivateSTlb = pProcess->GetPrivateSTlb(MTlbIndex);

                DXGK_PTE    RtPTEs[16*1024*1024/(64*1024)];
                D3DGPU_VIRTUAL_ADDRESS  GpuVa = AffectedAddressStart;

                pRtPte = pUpdatePageTable->pPageTableEntries;
                RtPteInc = pUpdatePageTable->Flags.Repeat ? 0 : 1;

                for (UINT i = 0; i < NumPageTableEntries; i++, GpuVa += 64*1024)
                {
                    if ((GpuVa < pUpdatePageTable->FirstPteVirtualAddress) ||
                        (GpuVa >= RtGpuVaEnd))
                    {
                        Gc7LSTlbEntry  *pSTlbEntry = (Gc7LSTlbEntry*)pUpdatePageTable->PageTableAddress.CpuVirtual;

                        Gc7LSTlbEntry   CurSTlbEntry = pSTlbEntry[(GpuVa - MTlbIndex*16*1024*1024)/(64*1024)];

                        memset(&RtPTEs[i], 0, sizeof(RtPTEs[i]));

                        if (CurSTlbEntry.Present)
                        {
                            if (CurSTlbEntry.WriteException)
                            {
                                RtPTEs[i].ReadOnly = 1;
                            }

                            ULONGLONG   PagePhysicalAddress = (((ULONGLONG)CurSTlbEntry.PageIndexExt) << 32) | (((ULONGLONG)CurSTlbEntry.PageIndex) << 12);

                            if ((PagePhysicalAddress >= (ULONGLONG)m_VideoMemoryPhysicalAddress.QuadPart) &&
                                (PagePhysicalAddress < (ULONGLONG)m_VideoMemoryPhysicalAddress.QuadPart + m_VideoMemorySize))
                            {
                                RtPTEs[i].Segment = LOCAL_VIDMEM_SEGMENT_ID;
                                RtPTEs[i].PageAddress = (PagePhysicalAddress - m_VideoMemoryPhysicalAddress.QuadPart) >> 12;

                                RtPTEs[i].Valid = 1;
                            }
                            else
                            {
                                if (PagePhysicalAddress != (ULONGLONG)m_DummyPagePhysicalAddress.QuadPart)
                                {
                                    RtPTEs[i].PageAddress = PagePhysicalAddress >> 12;

                                    RtPTEs[i].Valid = 1;
                                }
                            }
                        }
                    }
                    else
                    {
                        RtPTEs[i] = *pRtPte;
                        pRtPte += RtPteInc;
                    }
                }

                UINT    STlbOffset = pUpdatePageTable->StartIndex >> 4;
                UINT    STlbOffsetEnd = (pUpdatePageTable->StartIndex + pUpdatePageTable->NumPageTableEntries + 15) >> 4;

                pRtPte = RtPTEs;
                RtPteInc = 1;
                DXGK_PTE   *pStart64KRtPte = RtPTEs;

                for (; STlbOffset < STlbOffsetEnd; STlbOffset++, pStart64KRtPte += (16*RtPteInc))
                {
                    Gc7LSTlbEntry1M    *pCur1MSTlbEntry = pPrivateSTlb + STlbOffset;

                    pCur1MSTlbEntry->Value = 0;
                    pCur1MSTlbEntry->Present = pStart64KRtPte->Valid;

                    //
                    // HW requires physical page address for 1MB STlb to be 1M aligned.
                    // So in term of pages 0x400*0x400/0x1000 = 0x100 pages aligned
                    //

                    if (pStart64KRtPte->Valid &&
                        (pStart64KRtPte->PageAddress & (0x100 - 1)))
                    {
                        continue;
                    }

                    UINT        i = 0;
                    DXGK_PTE   *pNextRtPte;

                    for (i = 0, pRtPte = pStart64KRtPte; i < 15; i++, pRtPte += RtPteInc)
                    {
                        pNextRtPte = pRtPte + RtPteInc;

                        if ((pNextRtPte->Valid != pRtPte->Valid) ||
                            (pNextRtPte->ReadOnly != pRtPte->ReadOnly))
                        {
                            break;
                        }

                        if (pRtPte->Valid &&
                            (pNextRtPte->PageAddress != (pRtPte->PageAddress + ((64*1024) >> 12))))
                        {
                            break;
                        }
                    }

                    //
                    // Build an internal 1MB STlb if all 16 64KB PTEs have the
                    // consecutive physical addresses and the same access bits
                    //

                    if (i == 15)
                    {
                        if (pCur1MSTlbEntry->Present)
                        {
                            if (pStart64KRtPte->ReadOnly)
                            {
                                pCur1MSTlbEntry->WriteException = 1;
                            }
                            else
                            {
                                pCur1MSTlbEntry->Writable = 1;
                            }

                            ULONGLONG   GpuPageAddress = pStart64KRtPte->PageAddress << 12;

                            if (pRtPte->Segment == LOCAL_VIDMEM_SEGMENT_ID)
                            {
                                GpuPageAddress += m_VideoMemoryPhysicalAddress.QuadPart;
                            }

                            pCur1MSTlbEntry->PageIndexExt = (GpuPageAddress >> 32);
                            pCur1MSTlbEntry->PageIndex    = (GpuPageAddress >> 20);
                        }
                        else
                        {
                            pCur1MSTlbEntry->EnableException = 1;
                        }
                    }
                    else
                    {
                        //
                        // Mark the 1MB range as not contiguous
                        //

                        pCur1MSTlbEntry->Present = 1;
                    }
                }

                //
                // If the newly updated STlbs can be upgraded to 1MB entry
                //

                UINT    i;

                //
                // Check if all 16MB of the MTlb can be covered by 1MB STlb
                //

                for (i = 0; i < 16; i++)
                {
                    if (pPrivateSTlb[i].Present &&
                        (pPrivateSTlb[i].PageIndex == 0))
                    {
                        break;
                    }
                }

                if (i == 16)
                {
                    pProcess->UpdateSTlbPageSize(
                                MTlbIndex,
                                STlbPageSize1M);
                }
                else
                {
                    //
                    // Request UpdateHwPageTable() to switch back to 64KB page table
                    //

                    pProcess->UpdateSTlbPageSize(
                                MTlbIndex,
                                STlbPageSize64K);
                }
            }
        }
    }
    break;

    case DXGK_OPERATION_FLUSH_TLB:
    {
        //
        // gckMMU_Flush sets the pageTableDirty, and the actual flush
        // happens on the SubmitCommandBuffer path via _CheckFlushMm()
        //

        gckMMU_Flush(
            galDevice->kernels[gcvCORE_MAJOR]->mmu,
            gcvVIDMEM_TYPE_GENERIC);
    }
    break;

    case DXGK_OPERATION_FILL:
        break;

    case DXGK_OPERATION_DISCARD_CONTENT:
        break;

    case DXGK_OPERATION_VIRTUAL_FILL:
#if GC_7L_ENABLE_FILL_VIRTUAL
    {
        GcKm7LContext* pGcKm7LContext = (GcKm7LContext*)pBuildPagingBuffer->hSystemContext;

        Status = pGcKm7LContext->FillBuffer(pBuildPagingBuffer);
    }
#elif GC_7L_USE_DUMMY_FILL
    {
        GcDmaBufInfo   *pDmaBufInfo;

        pDmaBufInfo = (GcDmaBufInfo *)pBuildPagingBuffer->pDmaBufferPrivateData;

        if (pBuildPagingBuffer->DmaSize < 0x40)
        {
            return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
        }

        memset(pBuildPagingBuffer->pDmaBuffer, 0, 0x40);

        pBuildPagingBuffer->pDmaBuffer = ((BYTE *)pBuildPagingBuffer->pDmaBuffer) + 0x40;

        pDmaBufInfo->m_DmaBufState.m_bPaging = 1;
    }
#endif
    break;

    case DXGK_OPERATION_VIRTUAL_TRANSFER:
#if GC_7L_ENABLE_TRANSFER_VIRTUAL
    {
        GcKm7LContext*  pGcKm7LContext = (GcKm7LContext*)pBuildPagingBuffer->hSystemContext;

        Status = pGcKm7LContext->TransferBuffer(pBuildPagingBuffer);
    }
#endif
    break;

    case DXGK_OPERATION_NOTIFY_RESIDENCY:
    {
        GcKmAllocation *pAllocation = (GcKmAllocation *)pBuildPagingBuffer->NotifyResidency.hAllocation;

        pAllocation->m_gpuPhysicalAddress = pBuildPagingBuffer->NotifyResidency.PhysicalAddress;
    }
    break;

    case DXGK_OPERATION_INIT_CONTEXT_RESOURCE:
    {
        GcKmGdiAllocation*  pGdiAllocation = (GcKmGdiAllocation*)pBuildPagingBuffer->InitContextResource.hAllocation;

        pGdiAllocation->m_pCpuAddress = (BYTE*)pBuildPagingBuffer->InitContextResource.Destination.VirtualAddress;
        pGdiAllocation->m_GpuVa = pBuildPagingBuffer->InitContextResource.Destination.GpuVirtualAddress;
    }
    break;

    default:
        NT_ASSERT(FALSE);
        break;
    }

    return Status;
}

ULONGLONG
GcKm7LMmu::GetPhysicalAddress(
    const D3DGPU_PHYSICAL_ADDRESS  *pGpuPhysicalAddress)
{
    if (pGpuPhysicalAddress->SegmentId == LOCAL_VIDMEM_SEGMENT_ID)
    {
        return m_VideoMemoryPhysicalAddress.QuadPart + pGpuPhysicalAddress->SegmentOffset;
    }
    else
    {
        return pGpuPhysicalAddress->SegmentOffset;
    }
}

void
GcKm7LMmu::UpdateHwPageTable(
    GcKmContext    *pContext)
{
    GcKm7LProcess      *pProcess = (GcKm7LProcess *)pContext->GetProcess();
    Gc7LMTlbEntry4K    *pCachedMTlbEntry = nullptr;
    Gc7LSTlbEntry1M    *pPrivateSTlb     = nullptr;

    if (pProcess->Get1MBRangeUpdated(&pCachedMTlbEntry, &pPrivateSTlb))
    {
        D3DGPU_PHYSICAL_ADDRESS     RootPageTable;
        Gc7LMTlbEntry4K            *pHwMTlbEntry;

        pContext->GetRootPageTable(&RootPageTable);
        pHwMTlbEntry = (Gc7LMTlbEntry4K *)(((BYTE *)m_pVideoMemory) + RootPageTable.SegmentOffset);

        UINT MTlbIndex = (GC_7L_1MB_PAGE_GPU_VA_START >> (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS));

        pCachedMTlbEntry += MTlbIndex;
        pHwMTlbEntry     += MTlbIndex;
        pPrivateSTlb     += (MTlbIndex * 16);

        for (UINT i = 0;
            i < ((GC_7L_1MB_PAGE_GPU_VA_END - GC_7L_1MB_PAGE_GPU_VA_START) >> (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS));
            i++, pCachedMTlbEntry++, pHwMTlbEntry++)
        {
            //
            // In case global GPU VA range collides with internal GPU VA range for 1MB STlb
            //

            UINT    MTlbIndex = i + (GC_7L_1MB_PAGE_GPU_VA_START >> (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS));
            if ((MTlbIndex >= m_1stGlobalMTlbEntries) &&
                (MTlbIndex < (m_1stGlobalMTlbEntries + m_NumGlobalMTlbEntries)))
            {
                continue;
            }

            if (pCachedMTlbEntry->Zeroed)
            {
                if (STlbPageSize1M == pCachedMTlbEntry->STlbPageSize)
                {
                    Gc7LMTlbEntry1M    *pHwMTlbEntry1M = (Gc7LMTlbEntry1M *)pHwMTlbEntry;

                    ULONG64 PhysicalAddress = pProcess->GetPrivateSTlbPhysAddr(pPrivateSTlb + i*16);

                    pHwMTlbEntry1M->PageTableIndex = PhysicalAddress >> 6;
                    pHwMTlbEntry1M->STlbPageSize = STlbPageSize1M;
                    pHwMTlbEntry1M->Zeroed = 0;
                }
                else
                {
                    pHwMTlbEntry->STlbPageSize = pCachedMTlbEntry->STlbPageSize;
                    pHwMTlbEntry->PageTableIndex = pCachedMTlbEntry->PageTableIndex;
                    pHwMTlbEntry->Zeroed = 0;
                }

                pCachedMTlbEntry->Zeroed = false;
            }
        }

        pProcess->Clear1MBRangeUpdated();
    }
}

NTSTATUS
GcKm7LMmu::SwitchGpuVaSpace(
    const D3DGPU_PHYSICAL_ADDRESS  *pRootPageTable)
{
    if (GcKmdGlobal::s_bShadowGpuVa && !GcKmdGlobal::s_bSwitchGpuVa)
    {
        return STATUS_SUCCESS;
    }

    NT_ASSERT(pRootPageTable->SegmentId == LOCAL_VIDMEM_SEGMENT_ID);
    BYTE   *pPerProcessMTlb = ((BYTE*)m_pVideoMemory) + pRootPageTable->SegmentOffset;

    BYTE   *pGpuHwMTlb = (BYTE *)galDevice->kernels[0]->mmu->mtlbLogical;
    UINT    NumMTlbEntries = (1 << GC_7L_MTLB_INDEX_BITS);

    memcpy(pGpuHwMTlb, pPerProcessMTlb, NumMTlbEntries*sizeof(Gc7LMTlbEntry4K));

    gckMMU_Flush(
        galDevice->kernels[gcvCORE_MAJOR]->mmu,
        gcvVIDMEM_TYPE_GENERIC);

    return STATUS_SUCCESS;
}


