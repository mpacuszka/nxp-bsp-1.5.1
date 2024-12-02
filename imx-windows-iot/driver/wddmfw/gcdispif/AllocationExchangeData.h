/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#pragma warning(disable : 4201)

#include <wingdi.h>
#include <d3d10umddi.h>

typedef enum _HwMemFormat
{
    X565d,
    X8888,
    X565,
    X32,
    X16,
    X8,
    D24S8       // For depth stencil
} HwMemFormat;

//
// Memory segment ID values
//

#define APERTURE_SEGMENT_ID                 1
#define LOCAL_VIDMEM_SEGMENT_ID             2

// Allocate data exchanged between UMD and KMD
struct AllocationExchangeData
{
    // Input from UMD CreateResource DDI
    D3D10DDIRESOURCE_TYPE           m_resourceDimension;

    D3D10DDI_MIPINFO                m_mip0Info;
    UINT                            m_usage;        // D3D10_DDI_RESOURCE_USAGE
    UINT                            m_bindFlags;    // D3D10_DDI_RESOURCE_BIND_FLAG
    UINT                            m_mapFlags;     // D3D10_DDI_CPU_ACCESS
    UINT                            m_miscFlags;    // D3D10_DDI_RESOURCE_MISC_FLAG
    DXGI_FORMAT                     m_format;
    DXGI_SAMPLE_DESC                m_sampleDesc;
    UINT                            m_mipLevels;
    UINT                            m_arraySize;
    D3DWDDM2_0DDI_TEXTURE_LAYOUT    m_textureLayout;

    BOOLEAN                         m_isPrimary;
    DXGI_DDI_PRIMARY_DESC           m_primaryDesc;
#ifndef SCAFFOLD_OFF
    BOOLEAN                         m_isGdiPrimary;
    BOOLEAN                         m_isDwmPrimary;
#endif

    UINT                            m_hwTileMode;
    UINT                            m_hwPitch;
    UINT                            m_hwWidthPixels;
    UINT                            m_hwHeightPixels;

    UINT                            m_memorySegmentId;
    BOOLEAN                         m_isCpuVisible;
    BOOLEAN                         m_isShared;
    BOOLEAN                         m_isGdiSurface;
    UINT64                          m_hwSizeBytes;

    UINT                            m_linearOffset;
};

struct AllocationGroupExchangeData
{
    int     m_dummy;
};
