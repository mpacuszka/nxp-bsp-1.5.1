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

#include "GcKmdAdapter.h"
#include "GcKmd7LMmu.h"
#include "GcKmd7LNode.h"
#include "GcKmd7LDevice.h"

#include "Gc7LCmdGdiOp.h"

typedef struct _GpuMemoryBlock
{
    UINT    m_GpuVa = 0;
    BYTE*   m_pMem = gcvNULL;
    UINT    m_Size = 0;
} GpuMemoryHeap, GpuMemoryChunk;

class GcKm7LAdapter : public GcKmAdapter
{
public:

    friend class GcKmAdapter;

    GcKm7LAdapter(
        IN_CONST_PDEVICE_OBJECT PhysicalDeviceObject,
        WCHAR  *pwszDeviceId) :
        GcKmAdapter(PhysicalDeviceObject, pwszDeviceId, false),
        m_Mmu(this),
        m_3DNode(&m_Mmu),
        m_CopyNode(nullptr)
    {
        // do nothing
    }

    virtual ~GcKm7LAdapter()
    {
        // do nothing
    }

private:

    void* operator new(size_t  size);
    void operator delete(void* ptr);

protected:

    virtual NTSTATUS Start(
        IN_PDXGK_START_INFO     DxgkStartInfo,
        IN_PDXGKRNL_INTERFACE   DxgkInterface,
        OUT_PULONG              NumberOfVideoPresentSources,
        OUT_PULONG              NumberOfChildren);

    virtual NTSTATUS Stop();

    virtual BOOLEAN InterruptRoutine(
        IN_ULONG        MessageNumber);

    virtual UINT GetNumNodes()
    {
        return GC_7L_ENGINE_COUNT;
    }

    virtual UINT GetPagingNode()
    {
        return GC_7L_ENGINE_ORDINAL_3D;
    }

    virtual NTSTATUS QueryAdapterInfo(
        IN_CONST_PDXGKARG_QUERYADAPTERINFO  pQueryAdapterInfo);

    virtual NTSTATUS CreateDevice(
        INOUT_PDXGKARG_CREATEDEVICE pCreateDevice) = NULL;

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

    virtual NTSTATUS AllocateTextureDescriptor(
        GcKmAllocation* pAllocation);

    virtual void ReleaseTextureDescriptor(
        GcKmAllocation* pAllocation);

    virtual GC_7L_GDI_OP* GetGdiOps() = NULL;

    virtual UINT GetNumGdiOps() = NULL;

    virtual BOOLEAN UseGdiShaderForPaging() = NULL;

    virtual NTSTATUS IsGdiSurfaceSupported(
        INOUT_PDXGKARG_GETSTANDARDALLOCATIONDRIVERDATA  pGetStandardAllocationDriverData);

public:

    HANDLE GetGdiUniformBufferHeap(
        DXGK_CREATECONTEXTFLAGS ContextFlags,
        HANDLE                  hRtDevice,
        HANDLE                  hRtContext,
        GpuMemoryHeap*          pHeap,
        UINT*                   pMaxPendingDmaBuffers);

    void UpdateTextureDescriptor(
        GcKmAllocation*         pAllocation,
        D3DGPU_VIRTUAL_ADDRESS  TexGpuVa);

    static const UINT   s_TexDescSize = 0x100;
    static const UINT   s_GdiTexDescHeapSize = 0x200*0x400;    // 512 KB

    static const UINT   s_TileSize = 0x40;
    static const UINT   s_TexSizeAlignment = 0x100;

protected:

    GcKm7LMmu       m_Mmu;
    GcKm7L3DNode    m_3DNode;
    GcKmCopyNode    m_CopyNode;
    GcKmDisplay    *m_pDisplay = nullptr;

    gckVIDMEM_NODE  m_GdiVidMem = gcvNULL;
    gctUINT32       m_GdiVidMemGpuVa = 0;
    gctPOINTER      m_pGdiVidMem = gcvNULL;

    GpuMemoryHeap   m_GdiUboHeap;
    GpuMemoryHeap   m_GdiTexDescHeap;
    KMUTEX          m_GdiTexDescBitmapMutex;
    RTL_BITMAP      m_GdiTexDescBitmap;
    ULONG           m_GdiTexDescBitmapBuf[s_GdiTexDescHeapSize/s_TexDescSize/0x20];
    ULONG           m_GdiLastTexDescIndex = 0;

    GpuMemoryHeap   m_SysUboHeap;
};


class GcKm7LMQAdapter : public GcKm7LAdapter
{
public:

    virtual
    ~GcKm7LMQAdapter()
    {
    }

    GcKm7LMQAdapter(
        IN_CONST_PDEVICE_OBJECT PhysicalDeviceObject,
        WCHAR* pwszDeviceId) :
        GcKm7LAdapter(PhysicalDeviceObject, pwszDeviceId)
    {
    }

protected:

    virtual NTSTATUS CreateDevice(
        INOUT_PDXGKARG_CREATEDEVICE pCreateDevice);

    virtual GC_7L_GDI_OP* GetGdiOps();

    virtual UINT GetNumGdiOps();

    virtual BOOLEAN UseGdiShaderForPaging()
    {
        return FALSE;
    }
};

class GcKm7LMPAdapter : public GcKm7LAdapter
{
public:

    virtual
    ~GcKm7LMPAdapter()
    {
    }

    GcKm7LMPAdapter(
        IN_CONST_PDEVICE_OBJECT PhysicalDeviceObject,
        WCHAR* pwszDeviceId) :
        GcKm7LAdapter(PhysicalDeviceObject, pwszDeviceId)
    {
    }

    virtual VOID GetSurfaceRequirement(
        GcSurfaceRequirement* pSurfaceRequirement)
    {
        pSurfaceRequirement->AlignmentWidth = 16;
        pSurfaceRequirement->AlignmentHeight = 4;
    }

protected:

    virtual NTSTATUS CreateDevice(
        INOUT_PDXGKARG_CREATEDEVICE pCreateDevice);

    virtual GC_7L_GDI_OP* GetGdiOps();

    virtual UINT GetNumGdiOps();

    virtual BOOLEAN UseGdiShaderForPaging()
    {
        return TRUE;
    }
};


