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
#include "GcKmdAllocation.h"

#include "Gc7LMmu.h"
#include "GcKmd7LMmuCommon.h"

class GcKmEmuAdapter;

class GcKm7LEmuMmu: public GcKm7LMmuCommon
{
public:

    GcKm7LEmuMmu(
        GcKmEmuAdapter *pAdapter)
    {
        m_pAdapter = pAdapter;

        m_ImplicitSysMemSegmentId = APERTURE_SEGMENT_ID + 1;
    }

    virtual NTSTATUS
    CreateProcess(
        DXGKARG_CREATEPROCESS* pCreateProcess);

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

protected:

    GcKmEmuAdapter *m_pAdapter;
};



