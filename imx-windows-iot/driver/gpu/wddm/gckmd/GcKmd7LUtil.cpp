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

#include "GcKmd7LUtil.h"

#include "GcKmdGlobal.h"
#include <ntimage.h>
#include "gcdispif.h"


GcKm7LPageTableWalker::GcKm7LPageTableWalker(
    ULONGLONG   RootPhysicalAddress)
{
    MmInitializeMdl(&m_MdlMTlb.m_Mdl, NULL, PAGE_SIZE);
    MmInitializeMdl(&m_MdlSTlb.m_Mdl, NULL, GC_7L_STLB_SIZE);

    m_MdlMTlb.m_PageNumber = RootPhysicalAddress >> 12;

    for (UINT i = 0; i < (GC_7L_STLB_SIZE/PAGE_SIZE); i++)
    {
        m_MdlSTlb.m_PageNumbers[i] = 0;
    }

    m_pMTlb = NULL;

    m_pMappedSTlb = NULL;
}

GcKm7LPageTableWalker::~GcKm7LPageTableWalker()
{
    if (m_pMappedSTlb)
    {
        MmUnmapLockedPages(
            m_pMappedSTlb,
            &m_MdlSTlb.m_Mdl);
    }

    if (m_pMTlb)
    {
        MmUnmapLockedPages(
            m_pMTlb,
            &m_MdlMTlb.m_Mdl);
    }
}

NTSTATUS
GcKm7LPageTableWalker::GpuVaRangeToCpuPhysicalPages(
    UINT        StartGpuVa,
    UINT        NumPages,
    PFN_NUMBER *pPhysicalPageNumber)
{
    NTSTATUS    Status = STATUS_SUCCESS;

    if (NULL == m_pMTlb)
    {
        m_pMTlb = (Gc7LMTlbEntry4K *)MmMapLockedPagesSpecifyCache(
                                        &m_MdlMTlb.m_Mdl,
                                        KernelMode,
                                        MmWriteCombined,
                                        NULL,
                                        FALSE,
                                        NormalPagePriority | MdlMappingNoExecute);
        if (NULL == m_pMTlb)
        {
            return STATUS_NO_MEMORY;
        }
    }

    UINT    STlbCoverage = 1 << (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS);
    UINT    StartSTlbGpuVa = StartGpuVa & (~(STlbCoverage - 1));
    UINT    EndStlbGpuVa = ((StartGpuVa + NumPages*PAGE_SIZE) + (STlbCoverage - 1)) & (~(STlbCoverage - 1));
    UINT    STlbSpans = (EndStlbGpuVa - StartSTlbGpuVa)/STlbCoverage;

    UINT    CurGpuVa = StartGpuVa;

    for (UINT i = 0; i < STlbSpans; i++)
    {
        UINT    MTlbIndex = CurGpuVa/STlbCoverage;

        Gc7LMTlbEntry4K MTlbEntry = m_pMTlb[MTlbIndex];

        if (!MTlbEntry.Present)
        {
            return STATUS_NOT_MAPPED_DATA;
        }

        NT_ASSERT(MTlbEntry.STlbPageSize == STlbPageSize4K);

        if (m_MdlSTlb.m_PageNumbers[0] != MTlbEntry.PageTableIndex)
        {
            if (m_pMappedSTlb)
            {
                MmUnmapLockedPages(
                    m_pMappedSTlb,
                    &m_MdlSTlb.m_Mdl);
            }

            for (UINT j = 0; j < (GC_7L_STLB_SIZE/PAGE_SIZE); j++)
            {
                m_MdlSTlb.m_PageNumbers[j] = MTlbEntry.PageTableIndex + j;
            }

            m_pMappedSTlb = (Gc7LSTlbEntry *)MmMapLockedPagesSpecifyCache(
                                                &m_MdlSTlb.m_Mdl,
                                                KernelMode,
                                                MmWriteCombined,
                                                NULL,
                                                FALSE,
                                                NormalPagePriority | MdlMappingNoExecute);
        }

        if (NULL == m_pMappedSTlb)
        {
            Status = STATUS_NO_MEMORY;
            break;
        }

        UINT    StartSTlbIndex = (CurGpuVa & (STlbCoverage - 1))/PAGE_SIZE;
        UINT    EndSTlbIndex = 1 << GC_7L_STLB_INDEX_BITS;
        UINT    NumPagesInSTlb = EndSTlbIndex - StartSTlbIndex;

        if (NumPagesInSTlb > NumPages)
        {
            NumPagesInSTlb = NumPages;
        }

        for (UINT j = 0; j < NumPagesInSTlb; j++)
        {
            Gc7LSTlbEntry STlbEntry = m_pMappedSTlb[StartSTlbIndex + j];

            if (!STlbEntry.Present)
            {
                return STATUS_NOT_MAPPED_DATA;
            }

            *pPhysicalPageNumber++ = (STlbEntry.PageIndexExt << 20) + STlbEntry.PageIndex;
        }

        CurGpuVa += (NumPagesInSTlb*PAGE_SIZE);
        NumPages -= NumPagesInSTlb;
    }

    return Status;
}


NTSTATUS
GcKm7LPageTableWalker2::GpuVaRangeToCpuPhysicalPages(
    UINT        StartGpuVa,
    UINT        NumPages,
    PFN_NUMBER *pPhysicalPageNumber)
{
    NTSTATUS    Status = STATUS_SUCCESS;

    Gc7LMTlbEntry4K    *pMTlb = (Gc7LMTlbEntry4K *)m_pMmu->GetLocalVidMemCpuVA(m_RootPhysicalAddress);
    if (!pMTlb)
    {
        return STATUS_NO_MEMORY;
    }

    UINT    STlbCoverage = 1 << (GC_7L_GPU_VA_BIT_COUNT - GC_7L_MTLB_INDEX_BITS);
    UINT    StartSTlbGpuVa = StartGpuVa & (~(STlbCoverage - 1));
    UINT    EndSTlbGpuVa = ((StartGpuVa + NumPages*PAGE_SIZE) + (STlbCoverage - 1)) & (~(STlbCoverage - 1));
    UINT    STlbSpans = (EndSTlbGpuVa - StartSTlbGpuVa)/STlbCoverage;

    UINT    CurGpuVa = StartGpuVa;

    for (UINT i = 0; i < STlbSpans; i++)
    {
        UINT    MTlbIndex = CurGpuVa/STlbCoverage;

        Gc7LMTlbEntry4K    *pMTlbEntry = pMTlb + MTlbIndex;

        if (!pMTlbEntry->Present)
        {
            return STATUS_NOT_MAPPED_DATA;
        }

        Gc7LSTlbEntry  *pSTlb;

        if (pMTlbEntry->STlbPageSize < STlbPageSize1M)
        {
            pSTlb = (Gc7LSTlbEntry*)m_pMmu->GetLocalVidMemCpuVA(pMTlbEntry->PageTableIndex << 12);
        }
        else
        {
            Gc7LMTlbEntry1M    *pMTlbEntry1M = (Gc7LMTlbEntry1M *)pMTlbEntry;

            pSTlb = (Gc7LSTlbEntry*)m_pMmu->GetLocalVidMemCpuVA(pMTlbEntry1M->PageTableIndex << 6);
        }

        if (!pSTlb)
        {
            return STATUS_NO_MEMORY;
        }

        UINT    StartPageIndex = (CurGpuVa & (STlbCoverage - 1)) >> PAGE_SHIFT;
        UINT    EndPageIndex = STlbCoverage >> PAGE_SHIFT;
        UINT    NumPagesInSTlb = EndPageIndex - StartPageIndex;

        if (NumPagesInSTlb > NumPages)
        {
            NumPagesInSTlb = NumPages;
            EndPageIndex = StartPageIndex + NumPagesInSTlb;
        }

        const UINT  ENTRY_TO_PAGE_SHIFT = 4*pMTlbEntry->STlbPageSize;
        UINT    StartEntryIndex = StartPageIndex >> ENTRY_TO_PAGE_SHIFT;
        UINT    EndEntryIndex = (EndPageIndex + (1 << ENTRY_TO_PAGE_SHIFT) - 1) >> ENTRY_TO_PAGE_SHIFT;
        UINT    NumSTlbEntries = EndEntryIndex - StartEntryIndex;

        for (UINT j = 0; j < NumSTlbEntries; j++)
        {
            Gc7LSTlbEntry  *pSTlbEntry = pSTlb + StartEntryIndex + j;

            if (!pSTlbEntry->Present)
            {
                return STATUS_NOT_MAPPED_DATA;
            }

            UINT    CurPhysicalPageNumber = (pSTlbEntry->PageIndexExt << 20) + pSTlbEntry->PageIndex;
            UINT    StartOffset = StartPageIndex % (1 << ENTRY_TO_PAGE_SHIFT);
            UINT    NumPagesToCopy = (1 << ENTRY_TO_PAGE_SHIFT) - StartOffset;

            if (NumPagesToCopy > NumPages)
            {
                NumPagesToCopy = NumPages;
            }

            for (UINT k = 0; k < NumPagesToCopy; k++)
            {
                *pPhysicalPageNumber++ = CurPhysicalPageNumber + StartOffset + k;
            }

            StartPageIndex += NumPagesToCopy;
            NumPages -= NumPagesToCopy;
        }

        CurGpuVa += (NumPagesInSTlb*PAGE_SIZE);
    }

    return Status;
}


GcKm7LGpuMemMapper::GcKm7LGpuMemMapper(
    GcKm7LPageTableWalkerIf    *pPageTableWalker)
{
    m_pPageTableWalker = pPageTableWalker;

    m_pAlignedCpuVa = NULL;
}

GcKm7LGpuMemMapper::~GcKm7LGpuMemMapper()
{
    if (m_pAlignedCpuVa)
    {
        MmUnmapLockedPages(
            m_pAlignedCpuVa,
            &m_Mdl64Pages.m_Mdl);
    }
}

//
// If the same allocation is also mapped in user mode VA space for the app,
// both mappings must use the same cache properties to avoid cache corruption
//

PBYTE
GcKm7LGpuMemMapper::MapToCpuVa(
    UINT    GpuVa,
    UINT    GpuVaSizeInBytes)
{
    UINT    AlignedStart = GpuVa & (~(PAGE_SIZE - 1));
    UINT    AlignedEnd = (GpuVa + GpuVaSizeInBytes + (PAGE_SIZE - 1)) & (~(PAGE_SIZE - 1));
    UINT    NumPages = (AlignedEnd - AlignedStart)/PAGE_SIZE;

    if (NumPages > (sizeof(m_Mdl64Pages.m_PageNumbers)/sizeof(PFN_NUMBER)))
    {
        return NULL;
    }

    if (m_pAlignedCpuVa)
    {
        MmUnmapLockedPages(
            m_pAlignedCpuVa,
            &m_Mdl64Pages.m_Mdl);
    }

    MmInitializeMdl(&m_Mdl64Pages.m_Mdl, NULL, PAGE_SIZE*NumPages);
    //
    // GPU memory pages are locked down by GPU kernel runtime
    //
    m_Mdl64Pages.m_Mdl.MdlFlags = MDL_PARTIAL | MDL_PAGES_LOCKED | MDL_IO_SPACE;

    NTSTATUS    Status;

    Status = m_pPageTableWalker->GpuVaRangeToCpuPhysicalPages(
                                    AlignedStart,
                                    NumPages,
                                    m_Mdl64Pages.m_PageNumbers);
    if (!NT_SUCCESS(Status))
    {
        return NULL;
    }

    m_pAlignedCpuVa = (PBYTE)MmMapLockedPagesSpecifyCache(
                                &m_Mdl64Pages.m_Mdl,
                                KernelMode,
                                MmWriteCombined,
                                NULL,
                                FALSE,
                                NormalPagePriority | MdlMappingNoExecute);
    if (NULL == m_pAlignedCpuVa)
    {
        return NULL;
    }

    return m_pAlignedCpuVa + (GpuVa & (PAGE_SIZE - 1));
}

UINT
AlignAllocationSize(
    UINT64 *pAllocationSize)
{
    UINT    largestPageAligned = 0;
    UINT64  allocationSize = *pAllocationSize;

    for (UINT i = 1; i < ARRAYSIZE(Gc7LPageSizes); i++)
    {
        UINT    sizePageAligned;

        sizePageAligned = (allocationSize + (Gc7LPageSizes[i] - 1)) & (~(((UINT64)Gc7LPageSizes[i]) - 1));
        if (((sizePageAligned - allocationSize)*16) <= sizePageAligned)
        {
            allocationSize = sizePageAligned;
            largestPageAligned = i;
        }
        else
        {
            break;
        }
    }

    *pAllocationSize = allocationSize;

    return largestPageAligned;
}


extern "C"
{

typedef NTSTATUS (__stdcall *PFN_SetSystemInformation)(ULONG, PVOID, ULONG);

//
// Doc:
//    https://www.geoffchappell.com/studies/windows/km/ntoskrnl/api/ex/sysinfo/gdi_driver.htm
//    https://www.geoffchappell.com/studies/windows/km/ntoskrnl/api/ex/sysinfo/set.htm?tx=75
//

typedef struct _SYSTEM_GDI_DRIVER_INFORMATION
{
        UNICODE_STRING          DriverName;
        PVOID                   ImageAddress;
        PVOID                   SectionPointer;
        PVOID                   EntryPoint;
        IMAGE_EXPORT_DIRECTORY* ExportSectionPointer;
        ULONG                   ImageLength;
} SYSTEM_GDI_DRIVER_INFORMATION;

SYSTEM_GDI_DRIVER_INFORMATION   g_dispDllInfo = { 0 };

PFN_DisplayStartController  g_pfnDisplayStartController = nullptr;
PFN_DisplayStopController   g_pfnDisplayStopController = nullptr;


PVOID
GetExportFunction(
    SYSTEM_GDI_DRIVER_INFORMATION  *pDriverInfo,
    PCSTR                           functionName)
{
    PULONG  nameTableBase;
    USHORT  ordinalNumber;
    PUSHORT nameOrdinalTableBase;
    ULONG   numberOfNames;
    PULONG  addressTableBase;
    ULONG   i;

    if (pDriverInfo->ExportSectionPointer)
    {
        nameTableBase = (PULONG)((ULONG_PTR)pDriverInfo->ImageAddress +
                                 (ULONG)pDriverInfo->ExportSectionPointer->AddressOfNames);
        nameOrdinalTableBase = (PUSHORT)((ULONG_PTR)pDriverInfo->ImageAddress +
                                         (ULONG)pDriverInfo->ExportSectionPointer->AddressOfNameOrdinals);
        numberOfNames = pDriverInfo->ExportSectionPointer->NumberOfNames;
        addressTableBase = (PULONG)((ULONG_PTR)pDriverInfo->ImageAddress +
                                    (ULONG_PTR)pDriverInfo->ExportSectionPointer->AddressOfFunctions);
        for (i=0; i < numberOfNames; i++)
        {
            if (!strncmp(
                    functionName,
                    (PCHAR)(nameTableBase[i] + (ULONG_PTR)pDriverInfo->ImageAddress),
                    strlen(functionName)))
            {
                ordinalNumber = nameOrdinalTableBase[i];
                return ((PVOID)((ULONG_PTR)pDriverInfo->ImageAddress +
                                addressTableBase[ordinalNumber]));
            }
        }
    }

    return NULL;
}


VOID
GcWddmUnloadDispDll()
{
    if (g_dispDllInfo.SectionPointer)
    {
        UNICODE_STRING  usSetSystemInfo;
        PFN_SetSystemInformation pfnSetSystemInfo;

        RtlInitUnicodeString(&usSetSystemInfo, L"ZwSetSystemInformation");
        pfnSetSystemInfo = (PFN_SetSystemInformation)MmGetSystemRoutineAddress(&usSetSystemInfo);

        pfnSetSystemInfo(
            0x1B,
            &g_dispDllInfo.SectionPointer,
            sizeof(g_dispDllInfo.SectionPointer));

        RtlZeroMemory(&g_dispDllInfo, sizeof(g_dispDllInfo));
        g_pfnDisplayStartController = nullptr;
        g_pfnDisplayStopController = nullptr;
    }
}


NTSTATUS
GcWddmCheckDriverMode()
{
    NTSTATUS    ntStatus = STATUS_SUCCESS;
    GcKmdDriverMode driverMode = GcKmdGlobal::s_DriverMode;

    if (driverMode != RenderOnly)
    {
        UNICODE_STRING  usSetSystemInfo;
        PFN_SetSystemInformation    pfnSetSystemInfo;

        //
        // Check if dispdll.dll can be loaded
        //

        RtlInitUnicodeString(&usSetSystemInfo, L"ZwSetSystemInformation");

        pfnSetSystemInfo = (PFN_SetSystemInformation)MmGetSystemRoutineAddress(&usSetSystemInfo);

        RtlInitUnicodeString(&g_dispDllInfo.DriverName, L"\\SystemRoot\\System32\\drivers\\dispctrl.dll");

        ntStatus = pfnSetSystemInfo(
                    0x36,
                    &g_dispDllInfo,
                    sizeof(g_dispDllInfo));
        if (NT_SUCCESS(ntStatus))
        {
            //
            // Check if dispdll.dll exports Display[Start|Stop]Controller
            //

            g_pfnDisplayStartController = (PFN_DisplayStartController)GetExportFunction(
                &g_dispDllInfo,
                "DisplayStartController");
            g_pfnDisplayStopController = (PFN_DisplayStopController)GetExportFunction(
                &g_dispDllInfo,
                "DisplayStopController");

            if ((nullptr == g_pfnDisplayStartController) ||
                (nullptr == g_pfnDisplayStopController))
            {
                ntStatus = STATUS_INVALID_ADDRESS;

                GcWddmUnloadDispDll();
            }
        }

        if (!NT_SUCCESS(ntStatus))
        {
            if (FullDriver == driverMode)
            {
                //
                // Reduce Full Driver mode to Render Only
                //

                driverMode = RenderOnly;
            }
            else
            {
                //
                // Display Only mode can't continue without dispdll.dll
                //

                return ntStatus;
            }
        }
    }

    DWORD   isNotPostDevice = (RenderOnly == driverMode) ? 1 : 0;

    ntStatus = RtlWriteRegistryValue(
                RTL_REGISTRY_CONTROL,
                L"GraphicsDrivers",
                L"DisableAutoAcpiPostDeivce",
                REG_DWORD,
                &isNotPostDevice,
                sizeof(DWORD));

    GcKmdGlobal::s_DriverMode = driverMode;

    return ntStatus;
}

}

