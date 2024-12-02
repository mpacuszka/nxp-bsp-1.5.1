/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include "AllocationExchangeData.h"
#include "GcKmdAllocation.h"

//
// Macros to be used for proper PAGED/NON-PAGED code placement
//

#define GC_NONPAGED_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    //__pragma(code_seg(.text))

#define GC_NONPAGED_SEGMENT_END \
    __pragma(code_seg(pop))

#define GC_PAGED_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    __pragma(code_seg("PAGE"))

#define GC_PAGED_SEGMENT_END \
    __pragma(code_seg(pop))

//
// We have some non-paged functions that supposed to be called on low IRQL.
// The following macro defines unified assert to be put at the beginning of
// such functions.
//
// NOTE: We can't use standard PAGED_CODE macro as it requires function to be
// placed in paged segment during compilation.
//
#define GC_ASSERT_MAX_IRQL(Irql) NT_ASSERT(KeGetCurrentIrql() <= (Irql))
#define GC_ASSERT_LOW_IRQL() GC_ASSERT_MAX_IRQL(APC_LEVEL)

//
// KMD new and delete operator
//

void* __cdecl operator new (
    size_t      size,
    POOL_TYPE   poolType,
    UINT        tag
    ) throw ();

void __cdecl operator delete (
    void   *ptr,
    size_t  size
    ) throw ();

DXGI_FORMAT
TranslateD3dDdiFormat(
    D3DDDIFORMAT    Format);

D3DDDIFORMAT
TranslateDxgiFormat(
    DXGI_FORMAT DxgiFormat);

UINT
AlignAllocationSize(
    UINT64 *pAllocationSize);

template<typename TSurfaceType> 
void
FillAllocationInfo(
    AllocationExchangeData         *pAllocationInfo,
    TSurfaceType                   *pSurfaceInfo,
    GcSurfaceRequirement           *pSurfaceRequirement,
    BOOL                            bShared = false,
    D3DDDI_VIDEO_PRESENT_SOURCE_ID  VidPnSourceId = 0,
    D3DDDI_RATIONAL                 RefreshRate = { 0, 0 })
{
    pAllocationInfo->m_resourceDimension = D3D10DDIRESOURCE_TEXTURE2D;
    
    pAllocationInfo->m_mip0Info.TexelWidth = pSurfaceInfo->Width;
    pAllocationInfo->m_mip0Info.TexelHeight = pSurfaceInfo->Height;
    pAllocationInfo->m_mip0Info.TexelDepth = 1;
    pAllocationInfo->m_mip0Info.PhysicalWidth = pSurfaceInfo->Width;
    pAllocationInfo->m_mip0Info.PhysicalHeight = pSurfaceInfo->Height;
    pAllocationInfo->m_mip0Info.PhysicalDepth = 1;

    pAllocationInfo->m_mapFlags = 0;

    pAllocationInfo->m_format = TranslateD3dDdiFormat(pSurfaceInfo->Format);

    pAllocationInfo->m_sampleDesc.Count = 1;
    pAllocationInfo->m_sampleDesc.Quality = 0;
    pAllocationInfo->m_mipLevels = 1;
    pAllocationInfo->m_arraySize = 1;

    pAllocationInfo->m_textureLayout = D3DWDDM2_0DDI_TL_ROW_MAJOR;

    if (pAllocationInfo->m_isPrimary)
    {
        pAllocationInfo->m_primaryDesc.Flags = 0;
        pAllocationInfo->m_primaryDesc.VidPnSourceId = VidPnSourceId;
        pAllocationInfo->m_primaryDesc.ModeDesc.Width = pSurfaceInfo->Width;
        pAllocationInfo->m_primaryDesc.ModeDesc.Height = pSurfaceInfo->Height;
        pAllocationInfo->m_primaryDesc.ModeDesc.Format = TranslateD3dDdiFormat(pSurfaceInfo->Format);
        pAllocationInfo->m_primaryDesc.ModeDesc.RefreshRate.Numerator = RefreshRate.Numerator;
        pAllocationInfo->m_primaryDesc.ModeDesc.RefreshRate.Denominator = RefreshRate.Denominator;
        pAllocationInfo->m_primaryDesc.ModeDesc.ScanlineOrdering = DXGI_DDI_MODE_SCANLINE_ORDER_UNSPECIFIED;
        pAllocationInfo->m_primaryDesc.ModeDesc.Rotation = DXGI_DDI_MODE_ROTATION_UNSPECIFIED;
        pAllocationInfo->m_primaryDesc.ModeDesc.Scaling = DXGI_DDI_MODE_SCALING_UNSPECIFIED;
        pAllocationInfo->m_primaryDesc.DriverFlags = 0;

        pAllocationInfo->m_memorySegmentId = LOCAL_VIDMEM_SEGMENT_ID;
        pAllocationInfo->m_usage = D3D10_DDI_USAGE_DEFAULT;

        pAllocationInfo->m_bindFlags = D3D10_DDI_BIND_RENDER_TARGET | D3D10_DDI_BIND_PRESENT;

        NT_ASSERT(bShared);
    }
    else
    {
        pAllocationInfo->m_memorySegmentId = APERTURE_SEGMENT_ID;
        pAllocationInfo->m_usage = D3D10_DDI_USAGE_DYNAMIC;

        pAllocationInfo->m_bindFlags = D3D10_DDI_BIND_SHADER_RESOURCE;

        pAllocationInfo->m_isCpuVisible = true;
    }

    if (bShared)
    {
        pAllocationInfo->m_miscFlags = D3D10_DDI_RESOURCE_MISC_SHARED;
        pAllocationInfo->m_isShared = true;
    }
    else
    {
        pAllocationInfo->m_miscFlags = 0;
        pAllocationInfo->m_isShared = false;
    }

    NT_ASSERT((pSurfaceInfo->Format == D3DDDIFMT_A8R8G8B8) ||
              (pSurfaceInfo->Format == D3DDDIFMT_A8));

    pAllocationInfo->m_hwTileMode = 0;
    
    UINT    AlignedWidth = pSurfaceInfo->Width;
    UINT    AlignedHeight = pSurfaceInfo->Height;

    AlignedWidth += (pSurfaceRequirement->AlignmentWidth - 1);
    AlignedWidth &= ~(pSurfaceRequirement->AlignmentWidth - 1);

    AlignedHeight += (pSurfaceRequirement->AlignmentHeight - 1);
    AlignedHeight &= ~(pSurfaceRequirement->AlignmentHeight - 1);

    pAllocationInfo->m_hwPitch = AlignedWidth*((pSurfaceInfo->Format == D3DDDIFMT_A8) ? 1 : 4);
    pAllocationInfo->m_hwPitch += (sizeof(DWORD) - 1);
    pAllocationInfo->m_hwPitch &= ~(sizeof(DWORD) - 1);

    pAllocationInfo->m_hwWidthPixels = pSurfaceInfo->Width;
    pAllocationInfo->m_hwHeightPixels = pSurfaceInfo->Height;

    pAllocationInfo->m_hwSizeBytes = pAllocationInfo->m_hwPitch*AlignedHeight;

    AlignAllocationSize(&pAllocationInfo->m_hwSizeBytes);
}

struct GC_STAGINGSURFACEDATA : public D3DKMDT_STAGINGSURFACEDATA
{
    D3DDDIFORMAT    Format;

    GC_STAGINGSURFACEDATA(
        D3DKMDT_STAGINGSURFACEDATA* pStandardStagingSurfData)
    {
        Width  = pStandardStagingSurfData->Width;
        Height = pStandardStagingSurfData->Height;
        Format = D3DDDIFMT_A8R8G8B8;
    }
};

class CMutexTracker
{
public:

    CMutexTracker(
        KMUTEX* pMutex)
    {
        m_pMutex = pMutex;
        KeWaitForSingleObject(m_pMutex, Executive, KernelMode, false, NULL);
    }

    ~CMutexTracker()
    {
        KeReleaseMutex(m_pMutex, false);
    }

private:

    KMUTEX* m_pMutex;
};

bool
ReadDwordRegistryValue(
    UNICODE_STRING* pRegistryPath,
    WCHAR* pRegKeyName,
    DWORD* pData);

void
ReadDwordRegistryValue2(
    HANDLE* phRegKey,
    WCHAR*  pRegValueName,
    DWORD*  pData);

