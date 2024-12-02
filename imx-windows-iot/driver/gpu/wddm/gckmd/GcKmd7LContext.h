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

#include "GcKmdContext.h"
#include "Gc7LCmdGdiOp.h"
#include "Gc7LGdiOpParams.h"
#include "GcKmd7LAdapter.h"


const UINT  GC_7L_COMMAND_BUFFER_HEADER_SIZE    = 0x20;
const UINT  GC_7L_COMMAND_BUFFER_FOOTER_SIZE    = 0x20;

class GcKm7LContext : public GcKmContext
{
public:

    virtual
    ~GcKm7LContext();

    NTSTATUS
    Initialize(
        GcKmDevice*             pDevice,
        DXGKARG_CREATECONTEXT*  pCreateContext);

    // For paging support
    virtual NTSTATUS
    FillBuffer(
        DXGKARG_BUILDPAGINGBUFFER*  pBuildPagingBuffer) = NULL;

    virtual NTSTATUS
    TransferBuffer(
        DXGKARG_BUILDPAGINGBUFFER* pBuildPagingBuffer) = NULL;

    D3DGPU_VIRTUAL_ADDRESS
    GetShadowDmaBufferGpuVa()
    {
        return m_ShadowDmaBuffer.m_GdiVidMemGpuVa;
    }

    void*
    GetShadowDmaBufferCpuVa()
    {
        return m_ShadowDmaBuffer.m_pGdiVidMem;
    }

protected:

    virtual UINT
    GetCommandBufferHeaderSize()
    {
        return GC_7L_COMMAND_BUFFER_HEADER_SIZE;
    }

    virtual UINT
    GetCommandBufferFooterSize()
    {
        return GC_7L_COMMAND_BUFFER_FOOTER_SIZE;
    }

    virtual NTSTATUS
    CopyImage(
        GcKmAllocation*         pSrcAllocation,
        D3DGPU_VIRTUAL_ADDRESS  SrcAllocationVirtualAddress,
        GcKmAllocation*         pDstAllocation,
        D3DGPU_VIRTUAL_ADDRESS  DstAllocationVirtualAddress,
        const RECT*             pSrcRect,
        const RECT*             pDstRect,
        UINT                    SubRectCnt,
        const RECT*             pDstSubRects,
        void**                  ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
        UINT                    DmaBufferSize,
        BYTE*                   pDmaBufferPrivateData,
        UINT                    DmaBufferPrivateDataSize,
        UINT*                   pMultipassOffset) = NULL;

    virtual NTSTATUS
    GdiBitBlt(
        DXGK_GDIARG_BITBLT*     pCmdBitBlt,
        DXGK_ALLOCATIONLIST*    pAllocationList,
        UINT                    AllocationListSize,
        BYTE**                  ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
        UINT                    DmaBufferSize,
        BYTE*                   pDmaBufferPrivateData,
        UINT                    DmaBufferPrivateDataSize,
        UINT*                   pMultipassOffset);

    virtual NTSTATUS
    GdiColorFill(
        DXGK_GDIARG_COLORFILL*  pCmdColorFill,
        DXGK_ALLOCATIONLIST*    pAllocationList,
        UINT                    AllocationListSize,
        BYTE**                  ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
        UINT                    DmaBufferSize,
        BYTE*                   pDmaBufferPrivateData,
        UINT                    DmaBufferPrivateDataSize,
        UINT*                   pMultipassOffset);

    virtual NTSTATUS
    GdiAlphaBlend(
        DXGK_GDIARG_ALPHABLEND* pCmdAlphaBlt,
        DXGK_ALLOCATIONLIST*    pAllocationList,
        UINT                    AllocationListSize,
        BYTE**                  ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
        UINT                    DmaBufferSize,
        BYTE*                   pDmaBufferPrivateData,
        UINT                    DmaBufferPrivateDataSize,
        UINT*                   pMultipassOffset);

    virtual NTSTATUS
    GdiStretchBlt(
        DXGK_GDIARG_STRETCHBLT* pCmdStretchBlt,
        DXGK_ALLOCATIONLIST*    pAllocationList,
        UINT                    AllocationListSize,
        BYTE**                  ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
        UINT                    DmaBufferSize,
        BYTE*                   pDmaBufferPrivateData,
        UINT                    DmaBufferPrivateDataSize,
        UINT*                   pMultipassOffset);

    virtual NTSTATUS
    GdiTransparentBlt(
        DXGK_GDIARG_TRANSPARENTBLT* pCmdTransparentBlt,
        DXGK_ALLOCATIONLIST*        pAllocationList,
        UINT                        AllocationListSize,
        BYTE**                      ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS*     pDmaBufferGpuVirtualAddress,
        UINT                        DmaBufferSize,
        BYTE*                       pDmaBufferPrivateData,
        UINT                        DmaBufferPrivateDataSize,
        UINT*                       pMultipassOffset);

    virtual NTSTATUS
    GdiClearTypeBlend(
        DXGK_GDIARG_CLEARTYPEBLEND* pCmdClearTypeBlend,
        DXGK_ALLOCATIONLIST*        pAllocationList,
        UINT                        AllocationListSize,
        BYTE**                      ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS*     pDmaBufferGpuVirtualAddress,
        UINT                        DmaBufferSize,
        BYTE*                       pDmaBufferPrivateData,
        UINT                        DmaBufferPrivateDataSize,
        UINT*                       pMultipassOffset);

    virtual GC_7L_GDI_OP*
    GetGdiOp(
        DXGK_RENDERKM_OPERATION     GdiOp,
        UINT                        GdiOpRop) = NULL;

    NTSTATUS
    GdiOpBuildCommandBuffer(
        GC_7L_GDI_OP*               pGdiOp,
        GcKmAllocation*             pDstAllocation,
        D3DGPU_VIRTUAL_ADDRESS      DstAllocationGpuVa,
        GcKmAllocation*             pSrcAllocation,
        D3DGPU_VIRTUAL_ADDRESS      SrcAllocationGpuVa,
        UINT                        SrcPitchOverride,
        GcKmAllocation*             pGammaAllocation,
        D3DGPU_VIRTUAL_ADDRESS      GammaAllocationGpuVa,
        RECT*                       pDstRect,
        RECT*                       pSrcRect,
        UINT                        NumSubRects,
        RECT*                       pDstSubRects,
        UINT                        Color,
        UINT                        Gamma,
        UINT                        Color2,
        BYTE**                      ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS*     pDmaBufferGpuVirtualAddress,
        UINT                        DmaBufferSize,
        BYTE*                       pDmaBufferPrivateData,
        UINT                        DmaBufferPrivateDataSize,
        UINT*                       pMultipassOffset);

    void
    GdiOpPatchStorageImage(
        UINT*                       pDmaBuffer,
        GcKmAllocation*             pAllocation,
        D3DGPU_VIRTUAL_ADDRESS      AllocationGpuVa,
        UINT                        PitchOverride);

    GdiOpParams*
    GdiAllocateUniformBuffer(
        GcDmaBufInfo*   pDmaBufInfo,
        UINT*           pUboGpuVa);

    NTSTATUS
    AllocateShadowBuffer(
        UINT BufferSize);

public:

    //
    // Uniform Buffer requires 0x100 alignment
    //

    static const UINT   s_GdiUniformBufferSize = (sizeof(GdiOpParams) + (0x100 - 1)) & (~(0x100 - 1));

    static const UINT   s_GdiUboHeapSize = 256*1024;    // 256 KB

protected:

    GpuMemoryHeap   m_GdiUboHeap = {};

    UINT            m_MaxPendingDmaBuffers;
    UINT            m_CurGdiUboChunk = 0;       // 0 to m_MaxPendingDmaBuffers - 1
    GpuMemoryChunk  m_GdiUboChunk;

    struct
    {
        gckVIDMEM_NODE  m_GdiVidMem;
        gctUINT32       m_GdiVidMemGpuVa;
        gctPOINTER      m_pGdiVidMem;
    } m_ShadowDmaBuffer;

};

class GcKm7LMQContext : public GcKm7LContext
{
public:

    virtual
    ~GcKm7LMQContext()
    {
    }

    // For paging support
    virtual NTSTATUS
    FillBuffer(
        DXGKARG_BUILDPAGINGBUFFER*  pBuildPagingBuffer);

    virtual NTSTATUS
    TransferBuffer(
        DXGKARG_BUILDPAGINGBUFFER* pBuildPagingBuffer);

protected:

    NTSTATUS
    FillImage(
        GcKmAllocation*         pSrcAllocation,
        D3DGPU_VIRTUAL_ADDRESS  DstAllocationVirtualAddress,
        const RECT*             pDstRect,
        UINT                    SubRectCnt,
        const RECT*             pDstSubRects,
        UINT                    ClearColor,
        void**                  ppDmaBuffer,
        UINT                    DmaBufferSize,
        UINT*                   pMultipassOffset);

    virtual NTSTATUS
    CopyImage(
        GcKmAllocation*         pSrcAllocation,
        D3DGPU_VIRTUAL_ADDRESS  SrcAllocationVirtualAddress,
        GcKmAllocation*         pDstAllocation,
        D3DGPU_VIRTUAL_ADDRESS  DstAllocationVirtualAddress,
        const RECT*             pSrcRect,
        const RECT*             pDstRect,
        UINT                    SubRectCnt,
        const RECT*             pDstSubRects,
        void**                  ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
        UINT                    DmaBufferSize,
        BYTE*                   pDmaBufferPrivateData,
        UINT                    DmaBufferPrivateDataSize,
        UINT*                   pMultipassOffset);

    virtual NTSTATUS
    GdiBitBlt(
        DXGK_GDIARG_BITBLT*     pCmdBitBlt,
        DXGK_ALLOCATIONLIST*    pAllocationList,
        UINT                    AllocationListSize,
        BYTE**                  ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
        UINT                    DmaBufferSize,
        BYTE*                   pDmaBufferPrivateData,
        UINT                    DmaBufferPrivateDataSize,
        UINT*                   pMultipassOffset);

    virtual NTSTATUS
    GdiColorFill(
        DXGK_GDIARG_COLORFILL*  pCmdColorFill,
        DXGK_ALLOCATIONLIST*    pAllocationList,
        UINT                    AllocationListSize,
        BYTE**                  ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
        UINT                    DmaBufferSize,
        BYTE*                   pDmaBufferPrivateData,
        UINT                    DmaBufferPrivateDataSize,
        UINT*                   pMultipassOffset);

    virtual GC_7L_GDI_OP*
    GetGdiOp(
        DXGK_RENDERKM_OPERATION GdiOp,
        UINT                    GdiOpRop);
};

class GcKm7LMPContext : public GcKm7LContext
{
public:

    virtual
    ~GcKm7LMPContext();

    // For paging support
    virtual NTSTATUS
    FillBuffer(
        DXGKARG_BUILDPAGINGBUFFER*  pBuildPagingBuffer);

    virtual NTSTATUS
    TransferBuffer(
        DXGKARG_BUILDPAGINGBUFFER* pBuildPagingBuffer);

protected:

    virtual NTSTATUS
    CopyImage(
        GcKmAllocation*         pSrcAllocation,
        D3DGPU_VIRTUAL_ADDRESS  SrcAllocationVirtualAddress,
        GcKmAllocation*         pDstAllocation,
        D3DGPU_VIRTUAL_ADDRESS  DstAllocationVirtualAddress,
        const RECT*             pSrcRect,
        const RECT*             pDstRect,
        UINT                    SubRectCnt,
        const RECT*             pDstSubRects,
        void**                  ppDmaBuffer,
        D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
        UINT                    DmaBufferSize,
        BYTE*                   pDmaBufferPrivateData,
        UINT                    DmaBufferPrivateDataSize,
        UINT*                   pMultipassOffset);

    virtual GC_7L_GDI_OP*
    GetGdiOp(
        DXGK_RENDERKM_OPERATION GdiOp,
        UINT                    GdiOpRop);

private:

    HANDLE  m_hGdiAllocation = nullptr;
};


