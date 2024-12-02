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

#include "GcKmd.h"
#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmd7LMmuCommon.tmh"

#include "GcKmd7LMmuCommon.h"
#include "GcKmd7LProcess.h"
#include "GcKmdAllocation.h"
#include "GcKmd7LNode.h"

#include "GcKmdUtil.h"

NTSTATUS
GcKm7LMmuCommon::Start()
{
    return STATUS_SUCCESS;
}

void
GcKm7LMmuCommon::Stop()
{
}

NTSTATUS
GcKm7LMmuCommon::QueryAdapterInfo(
    CONST DXGKARG_QUERYADAPTERINFO *pQueryAdapterInfo)
{
    switch (pQueryAdapterInfo->Type)
    {
    case DXGKQAITYPE_QUERYSEGMENT4:
    {
        if (pQueryAdapterInfo->OutputDataSize < sizeof(DXGK_QUERYSEGMENTOUT4))
        {
            GC_ASSERTION(
                "Output buffer is too small. (pQueryAdapterInfo->OutputDataSize=%d, sizeof(DXGK_QUERYSEGMENTOUT4)=%d)",
                pQueryAdapterInfo->OutputDataSize,
                sizeof(DXGK_QUERYSEGMENTOUT4));
            return STATUS_BUFFER_TOO_SMALL;
        }

        DXGK_QUERYSEGMENTOUT4* pSegmentInfo = (DXGK_QUERYSEGMENTOUT4*)pQueryAdapterInfo->pOutputData;

        if (!pSegmentInfo[0].pSegmentDescriptor)
        {
            pSegmentInfo->NbSegment = 1;
        }
        else
        {
            //
            // Private data size should be the maximum of UMD and KMD and the same size must
            // be reported in DxgkDdiCreateContext for paging engine
            //
            pSegmentInfo->PagingBufferPrivateDataSize = sizeof(GcDmaBufInfo);

            pSegmentInfo->PagingBufferSegmentId = APERTURE_SEGMENT_ID;  // Create paging buffer in aperture segment
            pSegmentInfo->PagingBufferSize = GC_PAGING_BUFFER_SIZE;

            //
            // Setup aperture segment, which is only used for allocation with AccessedPhysically
            //
            DXGK_SEGMENTDESCRIPTOR4* pApertureSegmentDesc = (DXGK_SEGMENTDESCRIPTOR4*)(pSegmentInfo->pSegmentDescriptor);

            memset(pApertureSegmentDesc, 0, sizeof(*pApertureSegmentDesc));

            pApertureSegmentDesc->Flags.Aperture = true;
            pApertureSegmentDesc->Flags.CpuVisible = true;
#if CACHE_COHERENT_PLATFORM
            pApertureSegmentDesc->Flags.CacheCoherent = true;
#endif
            pApertureSegmentDesc->Flags.PitchAlignment = true;
            pApertureSegmentDesc->Flags.PreservedDuringStandby = true;
            pApertureSegmentDesc->Flags.PreservedDuringHibernate = true;

            pApertureSegmentDesc->BaseAddress.QuadPart = GC_7L_APERTURE_SEGMENT_BASE_ADDRESS;   // Gpu base physical address
            pApertureSegmentDesc->CpuTranslatedAddress.QuadPart = 0xFFFFFFFE00000000;           // Cpu base physical address
            pApertureSegmentDesc->Size = GC_7L_APERTURE_SEGMENT_SIZE;
        }
    }
    break;

    case DXGKQAITYPE_GPUMMUCAPS:
    {
        DXGK_GPUMMUCAPS* pGpuMmuCaps = (DXGK_GPUMMUCAPS*)pQueryAdapterInfo->pOutputData;

        memset(pGpuMmuCaps, 0, sizeof(DXGK_GPUMMUCAPS));

        pGpuMmuCaps->ZeroInPteSupported = 1;
#if CACHE_COHERENT_PLATFORM
        pGpuMmuCaps->CacheCoherentMemorySupported = 1;
#endif
        pGpuMmuCaps->PageTableUpdateRequireAddressSpaceIdle = 1;

        pGpuMmuCaps->PageTableUpdateMode = DXGK_PAGETABLEUPDATE_CPU_VIRTUAL;

        pGpuMmuCaps->VirtualAddressBitCount = GC_7L_GPU_VA_BIT_COUNT;
        pGpuMmuCaps->PageTableLevelCount = GC_7L_PAGE_TABLE_LEVEL_COUNT;
    }
    break;

    case DXGKQAITYPE_PAGETABLELEVELDESC:
    {
        static DXGK_PAGE_TABLE_LEVEL_DESC s_Gc7LPageTableLevelDesc[2] =
        {
            {
                GC_7L_STLB_INDEX_BITS, // PageTableIndexBitCount
                m_ImplicitSysMemSegmentId, // PageTableSegmentId
                m_ImplicitSysMemSegmentId, // PagingProcessPageTableSegmentId
                GC_7L_STLB_SIZE, // PageTableSizeInBytes
                0, // PageTableAlignmentInBytes
            },
            {
                GC_7L_MTLB_INDEX_BITS, // PageTableIndexBitCount
                m_ImplicitSysMemSegmentId, // PageTableSegmentId
                m_ImplicitSysMemSegmentId, // PagingProcessPageTableSegmentId
                GC_7L_MTLB_SIZE, // PageTableSizeInBytes
                0, // PageTableAlignmentInBytes
            }
        };

        UINT PageTableLevel = *((UINT*)pQueryAdapterInfo->pInputData);
        DXGK_PAGE_TABLE_LEVEL_DESC* pPageTableLevelDesc = (DXGK_PAGE_TABLE_LEVEL_DESC*)pQueryAdapterInfo->pOutputData;

        *pPageTableLevelDesc = s_Gc7LPageTableLevelDesc[PageTableLevel];
    }
    break;

    default:
    {
        return STATUS_NOT_SUPPORTED;
    }
    break;
    }

    return STATUS_SUCCESS;
}

void
GcKm7LMmuCommon::SetAllocationSegmentInfo(
    GcKmAllocation         *pGcKmAllocation,
    DXGK_ALLOCATIONINFO    *pRtAllocInfo)
{
    NT_ASSERT(pGcKmAllocation->m_memorySegmentId == APERTURE_SEGMENT_ID);

    //
    // If the allocation is backed by system memory, then the driver should
    // specify the implicit system memory segment in the allocation info.
    //

    pRtAllocInfo->PreferredSegment.Value = 0;
    pRtAllocInfo->PreferredSegment.SegmentId0 = m_ImplicitSysMemSegmentId;
    pRtAllocInfo->PreferredSegment.Direction0 = 0;

    pRtAllocInfo->SupportedReadSegmentSet = 1 << (m_ImplicitSysMemSegmentId - 1);
    pRtAllocInfo->SupportedWriteSegmentSet = 1 << (m_ImplicitSysMemSegmentId - 1);
}

SIZE_T
GcKm7LMmuCommon::GetRootPageTableSize(
    INOUT_PDXGKARG_GETROOTPAGETABLESIZE pGetRootTableSize)
{
    //
    // GC 7000 Lite's MTLB is in system memory, so using fixed size to avoid churning
    //

    pGetRootTableSize->NumberOfPte = (1 << GC_7L_MTLB_INDEX_BITS);

    return GC_7L_MTLB_SIZE;
}

NTSTATUS
GcKm7LMmuCommon::BuildPagingBuffer(
    DXGKARG_BUILDPAGINGBUFFER  *pBuildPagingBuffer,
    UINT        LocalVidMemSegmentId,
    ULONGLONG   SegmentStartAddress)
{
    NTSTATUS    Status = STATUS_SUCCESS;

    switch (pBuildPagingBuffer->Operation)
    {
    case DXGK_OPERATION_UPDATE_PAGE_TABLE:
    {
        DXGK_BUILDPAGINGBUFFER_UPDATEPAGETABLE *pUpdatePageTable;

        pUpdatePageTable = &pBuildPagingBuffer->UpdatePageTable;

        DXGK_PTE   *pRtPte = pUpdatePageTable->pPageTableEntries;
        UINT        RtPteInc = pUpdatePageTable->Flags.Repeat ? 0 : 1;

        if (pUpdatePageTable->PageTableLevel)
        {
            // Set up MTlb

            Gc7LMTlbEntry4K    *pGc7LMTlbEntry;

            pGc7LMTlbEntry = (Gc7LMTlbEntry4K *)pUpdatePageTable->PageTableAddress.CpuVirtual;
            pGc7LMTlbEntry += pUpdatePageTable->StartIndex;

            for (UINT i = 0; i < pUpdatePageTable->NumPageTableEntries; i++)
            {
                pGc7LMTlbEntry->Value = 0;

                if (pRtPte->Valid)
                {
                    pGc7LMTlbEntry->Present = 1;

                    ULONGLONG   GpuPageTableAddress = pRtPte->PageTableAddress << 12;

                    if (pRtPte->Segment == LocalVidMemSegmentId)
                    {
                        GpuPageTableAddress += SegmentStartAddress;
                    }

#ifndef _GC7L_EMULATOR_
                    NT_ASSERT(GpuPageTableAddress < 4ULL*1024ULL*1024ULL*1024ULL);
#endif

                    pGc7LMTlbEntry->PageTableIndex = GpuPageTableAddress >> 12;
                }
                else
                {
                    pGc7LMTlbEntry->Value = m_DummyMTlbEntry.Value;
                }

                pGc7LMTlbEntry++;
                pRtPte += RtPteInc;
            }
        }
        else
        {
            // Set up STlb

            Gc7LSTlbEntry  *pGc7LSTlbEntry;

            pGc7LSTlbEntry = (Gc7LSTlbEntry *)pUpdatePageTable->PageTableAddress.CpuVirtual;
            pGc7LSTlbEntry += pUpdatePageTable->StartIndex;

            for (UINT i = 0; i < pUpdatePageTable->NumPageTableEntries; i++)
            {
                pGc7LSTlbEntry->Value = 0;

                if (pRtPte->Valid)
                {
                    pGc7LSTlbEntry->Present = 1;

                    if (pRtPte->ReadOnly)
                    {
                        pGc7LSTlbEntry->WriteException = 1;
                    }
                    else
                    {
                        pGc7LSTlbEntry->Writable = 1;
                    }

                    ULONGLONG   GpuPageAddress = pRtPte->PageAddress << 12;

                    if (pRtPte->Segment == LocalVidMemSegmentId)
                    {
                        GpuPageAddress += SegmentStartAddress;
                    }

                    pGc7LSTlbEntry->PageIndexExt = (GpuPageAddress >> 32);
                    pGc7LSTlbEntry->PageIndex    = (GpuPageAddress >> 12);
                }
                else
                {
                    pGc7LSTlbEntry->Value = m_DummySTlbEntry.Value;
                }

                pGc7LSTlbEntry++;
                pRtPte += RtPteInc;
            }
        }
    }
    break;

    default:
        NT_ASSERT(FALSE);
        break;
    }

    return STATUS_SUCCESS;
}


