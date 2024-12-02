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
#include "GcKmdAdapter.h"
#include "GcKmdContext.h"

class GcKmEmu3DEngine : public GcKmEngine
{
public:

    GcKmEmu3DEngine();

    ~GcKmEmu3DEngine();

    virtual NTSTATUS Start();

    virtual void Stop();

    virtual NTSTATUS SubmitCommandBuffer(
        D3DGPU_VIRTUAL_ADDRESS  DmaBufGpuVa,
        UINT                    DmaBufSize,
        GcKmContext            *pContext,
        ULONGLONG               RootPhysicalAddress,
        BOOLEAN                 bForceContextSwitch);

    virtual NTSTATUS CpuTransfer(
        ULONGLONG               RootPhysicalAddress,
        DXGK_BUILDPAGINGBUFFER_TRANSFERVIRTUAL* pTransferVirtual);
};


