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

#include "GcKmd7LMmu.h"
#include <math.h>

struct GcKmMdl1Page
{
    MDL         m_Mdl;
    PFN_NUMBER  m_PageNumber;
};

struct GcKmMdl4Pages
{
    MDL         m_Mdl;
    PFN_NUMBER  m_PageNumbers[4];
};

struct GcKmMdl64Pages
{
    MDL         m_Mdl;
    PFN_NUMBER  m_PageNumbers[64];
};

class GcKm7LPageTableWalkerIf
{
public:

    virtual NTSTATUS
    GpuVaRangeToCpuPhysicalPages(
        UINT        StartGpuVa,
        UINT        NumPages,
        PFN_NUMBER *pPhysicalPageNumber) = NULL;
};

class GcKm7LPageTableWalker : public GcKm7LPageTableWalkerIf
{
public:

    GcKm7LPageTableWalker(
        ULONGLONG   RootPhysicalAddress);

    ~GcKm7LPageTableWalker();

    virtual NTSTATUS
    GpuVaRangeToCpuPhysicalPages(
        UINT        StartGpuVa,
        UINT        NumPages,
        PFN_NUMBER *pPhysicalPageNumber);

private:

    GcKmMdl1Page        m_MdlMTlb;
    GcKmMdl4Pages       m_MdlSTlb;

    Gc7LMTlbEntry4K    *m_pMTlb;

    Gc7LSTlbEntry      *m_pMappedSTlb;
};

class GcKm7LPageTableWalker2 : public GcKm7LPageTableWalkerIf
{
public:

    GcKm7LPageTableWalker2(
        ULONGLONG   RootPhysicalAddress,
        GcKmMmu    *pMmu)
    {
        m_RootPhysicalAddress = RootPhysicalAddress;
        m_pMmu = pMmu;
    }

    ~GcKm7LPageTableWalker2()
    {
    }

    virtual NTSTATUS
    GpuVaRangeToCpuPhysicalPages(
        UINT        StartGpuVa,
        UINT        NumPages,
        PFN_NUMBER *pPhysicalPageNumber);

private:

    ULONGLONG   m_RootPhysicalAddress;
    GcKmMmu    *m_pMmu;
};

class GcKm7LGpuMemMapper
{
public:

    GcKm7LGpuMemMapper(
        GcKm7LPageTableWalkerIf    *pPageTableWalker);

    ~GcKm7LGpuMemMapper();

    PBYTE
    MapToCpuVa(
        UINT    GpuVa,
        UINT    GpuVaSizeInBytes);

private:

    GcKm7LPageTableWalkerIf    *m_pPageTableWalker;

    GcKmMdl64Pages          m_Mdl64Pages;
    PBYTE                   m_pAlignedCpuVa;
};


typedef unsigned int uint32_t;

__inline uint32_t
__vk_UtilLog2inXdot8(
    uint32_t val)
{
    uint32_t ret = 0;
    uint32_t scale = (1 << 8);

    if (val <= 1)
    {
        return 0;
    }

    if (!(val & 0xFF))
    {
        val >>= 8;
        ret += 8*scale;
    }
    if (!(val & 0xF))
    {
        val >>= 4;
        ret += 4*scale;
    }
    if (!(val & 0x3))
    {
        val >>= 2;
        ret += 2*scale;
    }
    if (!(val & 0x1))
    {
        val >>= 1;
        ret += scale;
    }

    if (val > 1)
    {
        ret += (uint32_t)(logf((float)val)/logf(2.0f)*scale);
    }

    return ret;
}

