/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include "AllocationExchangeData.h"

struct GcKmAllocation : public AllocationExchangeData
{
    D3DGPU_PHYSICAL_ADDRESS m_gpuPhysicalAddress;
    D3DGPU_VIRTUAL_ADDRESS  m_texDescGpuVa;
};

struct GcKmDeviceAllocation
{
    D3DKMT_HANDLE   m_hKMAllocation;
    GcKmAllocation *m_pGcKmAllocation;
};


struct GcSurfaceRequirement
{
    UINT    AlignmentWidth;
    UINT    AlignmentHeight;
};

