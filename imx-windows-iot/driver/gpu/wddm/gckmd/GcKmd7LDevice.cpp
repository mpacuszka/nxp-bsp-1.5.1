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
#include "GcKmd7LDevice.tmh"

#include "GcKmd7LDevice.h"
#include "GcKmd7LContext.h"
#include "GcKmdUtil.h"

GC_PAGED_SEGMENT_BEGIN; //===================================================

NTSTATUS
GcKm7LMQDevice::CreateContext(
    DXGKARG_CREATECONTEXT*  pCreateContext)
{
    GcKm7LContext*  pContext;
    NTSTATUS        Status;

    pContext = new (NonPagedPoolNx, 'TCCG') GcKm7LMQContext();
    if (!pContext)
    {
        GC_LOG_LOW_MEMORY("Failed to allocate GcKm7LContext.");
        return STATUS_NO_MEMORY;
    }

    Status = pContext->Initialize(this, pCreateContext);
    if (NT_SUCCESS(Status))
    {
        pCreateContext->hContext = pContext;
    }
    else
    {
        delete pContext;
    }

    return Status;
}


NTSTATUS
GcKm7LMPDevice::CreateContext(
    DXGKARG_CREATECONTEXT*  pCreateContext)
{
    GcKm7LContext*  pContext;
    NTSTATUS        Status;

    pContext = new (NonPagedPoolNx, 'TCCG') GcKm7LMPContext();
    if (!pContext)
    {
        GC_LOG_LOW_MEMORY("Failed to allocate GcKm7LContext.");
        return STATUS_NO_MEMORY;
    }

    Status = pContext->Initialize(this, pCreateContext);
    if (NT_SUCCESS(Status))
    {
        pCreateContext->hContext = pContext;
    }
    else
    {
        delete pContext;
    }

    return Status;
}

GC_PAGED_SEGMENT_END; //=====================================================


