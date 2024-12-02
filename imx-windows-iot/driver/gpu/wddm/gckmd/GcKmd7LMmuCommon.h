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

const UINT GC_PAGING_BUFFER_SIZE    = 3*PAGE_SIZE;


const UINT  GC_7L_GPU_VA_BIT_COUNT = 32;
const UINT  GC_7L_PAGE_TABLE_LEVEL_COUNT = 2;

const UINT  GC_7L_MTLB_INDEX_BITS = 8;
const UINT  GC_7L_STLB_INDEX_BITS = 12;

//
// GC 7000 Lite page table entry
//

typedef enum _Gc7LSTlbPageSize : unsigned int
{
    STlbPageSize4K  = 0,
    STlbPageSize64K = 1,
    STlbPageSize1M  = 2,
    STlbPageSize16M = 3
} Gc7LSTlbPageSize;

//
// Physcial address for STLB pages has to be lower than 4 GB
//
// Each MTLB entry covers a 16 MB range
//
// When covered using 4 KB page, the MTLB entry's STLB page table is 16 KB with 4 KB alignment
//
// When covered using 1 MB page, the MTLB entry's STLB page table is 64 bytes (16 entries)
// The alignment is only 64 bytes, thus the PagePageIndex1M of 26 below
//

typedef struct _Gc7LMTlbEntry1M
{
    union
    {
        struct
        {
            UINT                Present         : 1;
            UINT                EnableException : 1;
            Gc7LSTlbPageSize    STlbPageSize    : 2;
            UINT                Zeroed          : 2;
            UINT                PageTableIndex  : 26;
        };

        UINT32      Value;
    };
} Gc7LMTlbEntry1M;

typedef struct _Gc7LMTlbEntry4K
{
    union
    {
        struct
        {
            UINT                Present         : 1;
            UINT                EnableException : 1;
            Gc7LSTlbPageSize    STlbPageSize    : 2;
            UINT                Zeroed          : 8;
            UINT                PageTableIndex  : 20;
        };

        UINT32      Value;
    };
} Gc7LMTlbEntry4K;

#define GC_7L_LARGE_PAGE_1MB_SIZE    (1*1024*1024)
#define GC_7L_LARGE_PAGE_16MB_SIZE  (16*1024*1024)

typedef struct _Gc7LSTlbEntry
{
    union
    {
        struct
        {
            UINT    Present         : 1;
            UINT    EnableException : 1;
            UINT    Writable        : 1;
            UINT    WriteException  : 1;
            UINT    PageIndexExt    : 8;    // 39 : 32 bits of the physical address
            UINT    PageIndex       : 20;   // Physical address of data page has 40 bits
        };

        UINT32      Value;
    };
} Gc7LSTlbEntry;

//
// MTlb entry for 1MB large pages points to a STlb of 16 Gc7LSTlbEntry1M
//

typedef struct _Gc7LSTlbEntry1M
{
    union
    {
        struct
        {
            UINT    Present         : 1;
            UINT    EnableException : 1;
            UINT    Writable        : 1;
            UINT    WriteException  : 1;
            UINT    PageIndexExt    : 8;    // 39 : 32 bits of the physical address
            UINT    Unused          : 8;
            UINT    PageIndex       : 12;   // 1MB aligned physical address of 40 bits
        };

        UINT32      Value;
    };
} Gc7LSTlbEntry1M;

//
// MTlb entry for 16MB large page also points to a STlb of 16 Gc7LSTlbEntry1M,
// but they map a single chunk of 16MB aligned physical memory. So each entry
// maps 1MB consecutively.
//

typedef struct _Gc7LSTlbEntry16M1st1M
{
    union
    {
        struct
        {
            UINT    Present         : 1;
            UINT    EnableException : 1;
            UINT    Writable        : 1;
            UINT    WriteException  : 1;
            UINT    PageIndexExt    : 8;    // 39 : 32 bits of the physical address
            UINT    Unused          : 12;
            UINT    PageIndex       : 8;    // 16MB aligned physical address of 40 bits
        };

        UINT32      Value;
    };
} Gc7LSTlbEntry16M1st1M;

//
// GC 7000 Lite's page table has 2 levels
//
// Top level : MTLB uses 8 bits, so 256 entries
// 2nd level : STLB uses 12 bits for 4KB pages, so 4096 entries
//             STLB uses 8 bits for 64KB pages, so 256 entries
//             STLB uses 4 bits for 1 MB pages, so 16 entries
//
// The page table entry is 4 bytes
//
// Each STLB covers 16MB, so the total VA size is 4GB.
//

const UINT GC_7L_MTLB_SIZE = (1 << GC_7L_MTLB_INDEX_BITS)*sizeof(Gc7LMTlbEntry4K);
const UINT GC_7L_STLB_SIZE = (1 << GC_7L_STLB_INDEX_BITS)*sizeof(Gc7LSTlbEntry);

class GcKm7LAdapter;

class GcKm7LMmuCommon: public GcKmMmu
{
public:

    GcKm7LMmuCommon()
    {
        m_DummySTlbEntry.Value = 0;
        m_DummySTlbEntry.EnableException = 1;

        m_DummyMTlbEntry.Value = 0;
        m_DummyMTlbEntry.STlbPageSize = STlbPageSize4K;
    }

    virtual NTSTATUS
    Start();

    virtual void
    Stop();

    virtual NTSTATUS
    QueryAdapterInfo(
        CONST DXGKARG_QUERYADAPTERINFO *pQueryAdapterInfo);

    virtual void
    SetAllocationSegmentInfo(
        GcKmAllocation         *pGcKmAllocation,
        DXGK_ALLOCATIONINFO    *pRtAllocInfo);

    virtual SIZE_T
    GetRootPageTableSize(
        INOUT_PDXGKARG_GETROOTPAGETABLESIZE pArgs);

    virtual NTSTATUS
    BuildPagingBuffer(
        DXGKARG_BUILDPAGINGBUFFER  *pArgs,
        UINT        LocalVidMemSegmentId,
        ULONGLONG   SegmentStartAddress);

    virtual PBYTE
    GetLocalVidMemCpuVA(
        ULONGLONG   PageTablePhysicalAddress)
    {
        return nullptr;
    }

protected:

    UINT    m_ImplicitSysMemSegmentId;

    Gc7LSTlbEntry   m_DummySTlbEntry;
    Gc7LMTlbEntry4K m_DummyMTlbEntry;
};

