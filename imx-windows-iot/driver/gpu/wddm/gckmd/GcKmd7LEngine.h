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

typedef unsigned int    uint32_t;
typedef int             int32_t;

#include "gc_hal_metadata.h"

extern "C" {

#include "gc_hal_kernel_driver.h"

extern gckGALDEVICE galDevice;

gceSTATUS
_Commit(
    IN gckDEVICE Device,
    IN gceHARDWARE_TYPE HwType,
    IN gceENGINE Engine,
    IN gctPID ProcessId,
    IN OUT gcsHAL_COMMIT* Commit);

}


class GcKm7L3DEngine : public GcKmEngine
{
public:

    GcKm7L3DEngine(
        bool   *pbResetRequested,
        GcKmMmu    *pMmu);

    ~GcKm7L3DEngine();

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

private:

    gckCONTEXT      m_GlobalContexts[2];
    gctUINT32       m_GlobalContextHandles[2];

    UINT32          m_nGlobalContextForSubmission;
    GcKmContext    *m_pLastContextForSubmission;

    gctSIGNAL       m_pDmaBufCompletionSignal;

    volatile bool  *m_pbResetRequested;

    GcKmMmu        *m_pMmu;
};

class GcKmCopyEngine : public GcKmEngine
{
public:

    GcKmCopyEngine();

    ~GcKmCopyEngine();

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


