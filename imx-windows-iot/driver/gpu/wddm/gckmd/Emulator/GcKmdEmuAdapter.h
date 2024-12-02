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
#include "GcHw.h"

#include "GcKmd7LEmuMmu.h"
#include "GcKmdAdapter.h"
#include "GcKmdEmuNode.h"

typedef enum _GC_EMU_ENGINE_ORDINAL
{
    GC_EMU_ENGINE_ORDINAL_3D = 0,
    GC_EMU_ENGINE_COUNT
} GC_EMU_ENGINE_ORDINAL;

#define EMU_GC_7000_LITE    L"EmuGC7LDevice_"

class GcKmEmuAdapter : public GcKmAdapter
{
public:

    GcKmEmuAdapter(
        IN_CONST_PDEVICE_OBJECT PhysicalDeviceObject,
        WCHAR  *pwszDeviceId);

    virtual ~GcKmEmuAdapter()
    {
        // do nothing
    }

    void* operator new(size_t  size);
    void operator delete(void* ptr);

protected:

    virtual NTSTATUS Start(
        IN_PDXGK_START_INFO     DxgkStartInfo,
        IN_PDXGKRNL_INTERFACE   DxgkInterface,
        OUT_PULONG              NumberOfVideoPresentSources,
        OUT_PULONG              NumberOfChildren);

    virtual NTSTATUS Stop();

    virtual UINT GetNumNodes()
    {
        return GC_EMU_ENGINE_COUNT;
    }

    virtual UINT GetPagingNode()
    {
        return GC_EMU_ENGINE_ORDINAL_3D;
    }

    virtual NTSTATUS QueryAdapterInfo(
        IN_CONST_PDXGKARG_QUERYADAPTERINFO  pQueryAdapterInfo);

    virtual NTSTATUS GetNodeMetaData(
        UINT                            NodeOrdinal,
        OUT_PDXGKARG_GETNODEMETADATA    pGetNodeMetadata);

    virtual NTSTATUS CollectDbgInfo(
        IN_CONST_PDXGKARG_COLLECTDBGINFO    pCollectDbgInfo);

    virtual NTSTATUS ResetFromTimeout();

    virtual NTSTATUS RestartFromTimeout();

    virtual NTSTATUS Escape(
        IN_CONST_PDXGKARG_ESCAPE    pEscape);

    virtual NTSTATUS SetPowerState(
        IN_ULONG                DeviceUid,
        IN_DEVICE_POWER_STATE   DevicePowerState,
        IN_POWER_ACTION         ActionType);

protected:

    GcKm7LEmuMmu    m_Mmu;
    GcKmEmu3DNode   m_3DNode;
};

