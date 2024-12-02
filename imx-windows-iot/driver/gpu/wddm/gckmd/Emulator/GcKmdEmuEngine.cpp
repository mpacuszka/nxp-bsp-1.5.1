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

#include "GcKmdLogging.h"
#include "GcKmdEmuEngine.tmh"

#include "GcKmdEmuEngine.h"

#include "GcKmdUtil.h"
#include "GcKmd7LUtil.h"

GcKmEmu3DEngine::GcKmEmu3DEngine()
{
}

GcKmEmu3DEngine::~GcKmEmu3DEngine()
{
}

NTSTATUS
GcKmEmu3DEngine::Start()
{
    return STATUS_SUCCESS;
}

void
GcKmEmu3DEngine::Stop()
{
}

NTSTATUS
GcKmEmu3DEngine::SubmitCommandBuffer(
    D3DGPU_VIRTUAL_ADDRESS  DmaBufGpuVa,
    UINT                    DmaBufSize,
    GcKmContext            *pContext,
    ULONGLONG               RootPhysicalAddress,
    BOOLEAN                 bForceContextSwitch)
{
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmEmu3DEngine::CpuTransfer(
    ULONGLONG               RootPhysicalAddress,
    DXGK_BUILDPAGINGBUFFER_TRANSFERVIRTUAL* pTransferVirtual)
{
    return STATUS_SUCCESS;
}


