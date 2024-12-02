/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include "GcKmd.h"
#include "GcKmdDevice.h"
#include "GcKmdAllocation.h"

class GcKmContext
{
public:

    GcKmContext();
    virtual ~GcKmContext();

    __forceinline static GcKmContext* Cast(IN_CONST_HANDLE hContext)
    {
        return static_cast<GcKmContext*>(hContext);
    }

    NTSTATUS
    Initialize(
        GcKmDevice*             pDevice,
        DXGKARG_CREATECONTEXT*  pCreateContext);

    GcKmProcess*
    GetProcess()
    {
        return m_pDevice->GetProcess();
    }

    NTSTATUS
    Present(
        INOUT_PDXGKARG_PRESENT  pPresent);

    NTSTATUS
    RenderGdi(
        INOUT_PDXGKARG_RENDERGDI    pRenderGdi);

protected:

    GcKmDevice                 *m_pDevice;
    UINT                        m_Node;
    HANDLE                      m_hRTContext;

    DXGK_CREATECONTEXTFLAGS     m_Flags;

    D3DGPU_PHYSICAL_ADDRESS     m_RootPageTableAddress;
    UINT                        m_NumRootPageTableEntries;

    KEVENT                     *m_pUmdPresentEvent;
    KEVENT                     *m_pKmdPresentEvent;

    virtual UINT
    GetCommandBufferHeaderSize()
    {
        return 0;
    }

    virtual UINT
    GetCommandBufferFooterSize()
    {
        return 0;
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

public:

    VOID
    SetRootPageTable(
        IN_CONST_PDXGKARG_SETROOTPAGETABLE  pSetPageTable);

    VOID
    GetRootPageTable(
        D3DGPU_PHYSICAL_ADDRESS    *pRootPageTable)
    {
        *pRootPageTable = m_RootPageTableAddress;
    }
    
};

