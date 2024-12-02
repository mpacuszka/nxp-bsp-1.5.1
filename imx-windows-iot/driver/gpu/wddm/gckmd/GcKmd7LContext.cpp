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

#include "GcKmdGlobal.h"
#include "GcKmdLogging.h"
#include "GcKmd7LContext.tmh"

#include "GcKmd7LContext.h"
#include "GcKmdAllocation.h"

#include "Gc7LMmu.h"

#include "GcKmd7LUtil.h"

#include "MQ\Gc7LCmdCopyImg.h"
#include "MQ\Gc7LCmdFillImg.h"
#include "Gc7LCmdGdiOp.h"
#include "Gc7LGdiOpParams.h"

#include "MQ\Gc7LCmdCopyBuf.h"
#include "MQ\Gc7LCmdFillBuf.h"

#include "gc_hal.h"
#include "gc_hal_kernel.h"
#include "gc_hal_kernel_hardware.h"


extern unsigned int GC_7L_GDI_OP_STARTS[];


static BOOLEAN  g_bGdiSingleCommandType = FALSE;


NTSTATUS
GcKm7LContext::Initialize(
    GcKmDevice*             pDevice,
    DXGKARG_CREATECONTEXT*  pCreateContext)
{
    NTSTATUS    Status;

    Status = GcKmContext::Initialize(pDevice, pCreateContext);
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    if (pCreateContext->Flags.GdiContext || pCreateContext->Flags.SystemContext)
    {
        ((GcKm7LAdapter*)m_pDevice->m_pGcKmAdapter)->GetGdiUniformBufferHeap(
                                                        pCreateContext->Flags,
                                                        NULL,
                                                        NULL,
                                                        &m_GdiUboHeap,
                                                        &m_MaxPendingDmaBuffers);
    }

    m_ShadowDmaBuffer = { 0 };
    if (GcKmdGlobal::s_bLimitAllocBelow4gbPa &&
       (pCreateContext->Flags.SystemContext || pCreateContext->Flags.GdiContext))
    {
        Status = AllocateShadowBuffer(pCreateContext->ContextInfo.DmaBufferSize);
        if (!NT_SUCCESS(Status))
        {
            return Status;
        }
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKm7LContext::AllocateShadowBuffer(
    UINT BufferSize)
{
    gcePOOL     pool      = gcvPOOL_DEFAULT;
    gctSIZE_T   allocSize = BufferSize;
    gctUINT32   allocFlag = 0;
    gceSTATUS   GcStatus;

    GcStatus = gckKERNEL_AllocateVideoMemory(galDevice->kernels[gcvCORE_MAJOR],
                                             PAGE_SIZE,
                                             gcvVIDMEM_TYPE_COMMAND,
                                             0,
                                             &allocSize,
                                             &pool,
                                             &m_ShadowDmaBuffer.m_GdiVidMem);
    if (GcStatus != gcvSTATUS_OK)
    {
        return STATUS_UNSUCCESSFUL;
    }

    // Lock for GPU access
    GcStatus = gckVIDMEM_NODE_Lock(galDevice->kernels[gcvCORE_MAJOR],
                                   m_ShadowDmaBuffer.m_GdiVidMem,
                                   &m_ShadowDmaBuffer.m_GdiVidMemGpuVa);
    if (GcStatus != gcvSTATUS_OK)
    {
        return STATUS_UNSUCCESSFUL;
    }

    // Lock for kernel mode CPU access
    GcStatus = gckVIDMEM_NODE_LockCPU(galDevice->kernels[gcvCORE_MAJOR],
                                      m_ShadowDmaBuffer.m_GdiVidMem,
                                      gcvFALSE,
                                      gcvFALSE,
                                      &m_ShadowDmaBuffer.m_pGdiVidMem);
    if (GcStatus != gcvSTATUS_OK)
    {
        return STATUS_UNSUCCESSFUL;
    }

    return STATUS_SUCCESS;
}

GcKm7LContext::~GcKm7LContext()
{
    if (m_ShadowDmaBuffer.m_pGdiVidMem)
    {
        gckVIDMEM_NODE_UnlockCPU(
            galDevice->kernels[gcvCORE_MAJOR],
            m_ShadowDmaBuffer.m_GdiVidMem,
            0,
            gcvFALSE,
            gcvFALSE);
    }

    if (m_ShadowDmaBuffer.m_GdiVidMemGpuVa)
    {
        gckVIDMEM_NODE_Unlock(
            galDevice->kernels[gcvCORE_MAJOR],
            m_ShadowDmaBuffer.m_GdiVidMem,
            0,
            gcvNULL);
    }

    if (m_ShadowDmaBuffer.m_GdiVidMem)
    {
        gckVIDMEM_NODE_Dereference(
            galDevice->kernels[gcvCORE_MAJOR],
            m_ShadowDmaBuffer.m_GdiVidMem);
    }

    m_ShadowDmaBuffer.m_pGdiVidMem = gcvNULL;
    m_ShadowDmaBuffer.m_GdiVidMemGpuVa = 0;
    m_ShadowDmaBuffer.m_GdiVidMem = gcvNULL;

}

NTSTATUS
GcKm7LContext::GdiBitBlt(
    DXGK_GDIARG_BITBLT*     pCmdBitBlt,
    DXGK_ALLOCATIONLIST*    pAllocationList,
    UINT                    AllocationListSize,
    BYTE**                  ppDmaBuffer,
    D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
    UINT                    DmaBufferSize,
    BYTE*                   pDmaBufferPrivateData,
    UINT                    DmaBufferPrivateDataSize,
    UINT*                   pMultipassOffset)
{
    DXGK_ALLOCATIONLIST*    pSrcAllocInfo = pAllocationList + pCmdBitBlt->SrcAllocationIndex;
    DXGK_ALLOCATIONLIST*    pDstAllocInfo = pAllocationList + pCmdBitBlt->DstAllocationIndex;
    GcKmDeviceAllocation*   pSrcDeviceAlloc = (GcKmDeviceAllocation*)pSrcAllocInfo->hDeviceSpecificAllocation;
    GcKmDeviceAllocation*   pDstDeviceAlloc = (GcKmDeviceAllocation*)pDstAllocInfo->hDeviceSpecificAllocation;

    NT_ASSERT(pCmdBitBlt->Rop != DXGK_GDIROP_ROP3);

    NT_ASSERT((0 == pCmdBitBlt->SrcPitch) || (pCmdBitBlt->SrcPitch == pSrcDeviceAlloc->m_pGcKmAllocation->m_hwPitch));
    NT_ASSERT((0 == pCmdBitBlt->DstPitch) || (pCmdBitBlt->DstPitch == pDstDeviceAlloc->m_pGcKmAllocation->m_hwPitch));

    GcDmaBufInfo*   pDmaBufInfo = (GcDmaBufInfo*)pDmaBufferPrivateData;

    NT_ASSERT(pDmaBufInfo->m_DmaBufState.m_bGdi);
    // Forced context switch is required even when src is accessed as Storage Image
    pDmaBufInfo->m_DmaBufState.m_bGdiSamplerUsed = 1;

    WORD    BitBltRop = pCmdBitBlt->Rop;

    if (pSrcDeviceAlloc->m_pGcKmAllocation->m_format == DXGI_FORMAT_A8_UNORM)
    {
        NT_ASSERT(DXGK_GDIROP_SRCCOPY == pCmdBitBlt->Rop);
        BitBltRop = DXGK_GDIROP_ROP3 + 1;
    }

    return GdiOpBuildCommandBuffer(
            GetGdiOp(DXGK_GDIOP_BITBLT, BitBltRop),
            pDstDeviceAlloc->m_pGcKmAllocation,
            pDstAllocInfo->VirtualAddress,
            pSrcDeviceAlloc->m_pGcKmAllocation,
            pSrcAllocInfo->VirtualAddress,
            pCmdBitBlt->SrcPitch,
            NULL,
            0,
            &pCmdBitBlt->DstRect,
            &pCmdBitBlt->SrcRect,
            pCmdBitBlt->NumSubRects,
            pCmdBitBlt->pSubRects,
            0,
            0,
            0,
            ppDmaBuffer,
            pDmaBufferGpuVirtualAddress,
            DmaBufferSize,
            pDmaBufferPrivateData,
            DmaBufferPrivateDataSize,
            pMultipassOffset);
}

NTSTATUS
GcKm7LContext::GdiColorFill(
    DXGK_GDIARG_COLORFILL*  pCmdColorFill,
    DXGK_ALLOCATIONLIST*    pAllocationList,
    UINT                    AllocationListSize,
    BYTE**                  ppDmaBuffer,
    D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
    UINT                    DmaBufferSize,
    BYTE*                   pDmaBufferPrivateData,
    UINT                    DmaBufferPrivateDataSize,
    UINT*                   pMultipassOffset)
{
    DXGK_ALLOCATIONLIST*    pDstAllocInfo = pAllocationList + pCmdColorFill->DstAllocationIndex;
    GcKmDeviceAllocation*   pDstDeviceAlloc = (GcKmDeviceAllocation*)pDstAllocInfo->hDeviceSpecificAllocation;

    NT_ASSERT(pCmdColorFill->Rop != DXGK_GDIROPCF_ROP3);

    UINT    ActualRop = pCmdColorFill->Rop;
    UINT    ActualColor = pCmdColorFill->Color;

    switch (pCmdColorFill->Rop)
    {
    case DXGK_GDIROPCF_PDXN:
        ActualRop = DXGK_GDIROPCF_PATINVERT;
        ActualColor = ~pCmdColorFill->Color;
        break;
    case DXGK_GDIROPCF_DSTINVERT:
        ActualRop = DXGK_GDIROPCF_PATINVERT;
        ActualColor = 0xFFFFFFFF;
        break;
    }

    GcDmaBufInfo*   pDmaBufInfo = (GcDmaBufInfo*)pDmaBufferPrivateData;

    NT_ASSERT(pDmaBufInfo->m_DmaBufState.m_bGdi);
    // Forced context switch is required even when src is accessed as Storage Image
    pDmaBufInfo->m_DmaBufState.m_bGdiSamplerUsed = 1;

    return GdiOpBuildCommandBuffer(
            GetGdiOp(DXGK_GDIOP_COLORFILL, ActualRop),
            pDstDeviceAlloc->m_pGcKmAllocation,
            pDstAllocInfo->VirtualAddress,
            NULL,
            0,
            0,
            NULL,
            0,
            &pCmdColorFill->DstRect,
            NULL,
            pCmdColorFill->NumSubRects,
            pCmdColorFill->pSubRects,
            ActualColor,
            0,
            0,
            ppDmaBuffer,
            pDmaBufferGpuVirtualAddress,
            DmaBufferSize,
            pDmaBufferPrivateData,
            DmaBufferPrivateDataSize,
            pMultipassOffset);
}

NTSTATUS
GcKm7LContext::GdiAlphaBlend(
    DXGK_GDIARG_ALPHABLEND* pCmdAlphaBlend,
    DXGK_ALLOCATIONLIST*    pAllocationList,
    UINT                    AllocationListSize,
    BYTE**                  ppDmaBuffer,
    D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
    UINT                    DmaBufferSize,
    BYTE*                   pDmaBufferPrivateData,
    UINT                    DmaBufferPrivateDataSize,
    UINT*                   pMultipassOffset)
{
    DXGK_ALLOCATIONLIST*    pSrcAllocInfo = pAllocationList + pCmdAlphaBlend->SrcAllocationIndex;
    DXGK_ALLOCATIONLIST*    pDstAllocInfo = pAllocationList + pCmdAlphaBlend->DstAllocationIndex;
    GcKmDeviceAllocation*   pSrcDeviceAlloc = (GcKmDeviceAllocation*)pSrcAllocInfo->hDeviceSpecificAllocation;
    GcKmDeviceAllocation*   pDstDeviceAlloc = (GcKmDeviceAllocation*)pDstAllocInfo->hDeviceSpecificAllocation;

    NT_ASSERT((0 == pCmdAlphaBlend->SrcPitch) || (pCmdAlphaBlend->SrcPitch == pSrcDeviceAlloc->m_pGcKmAllocation->m_hwPitch));

    UINT    AlphaBlendRop;

    RECT*   pDstRect = &pCmdAlphaBlend->DstRect;
    RECT*   pSrcRect = &pCmdAlphaBlend->SrcRect;

    if ((pCmdAlphaBlend->SourceHasAlpha))
    {
        if (pCmdAlphaBlend->SourceConstantAlpha != 0xFF)
        {
            AlphaBlendRop = 1;
        }
        else
        {
            if (((pDstRect->right - pDstRect->left) != (pSrcRect->right - pSrcRect->left)) ||
                ((pDstRect->bottom - pDstRect->top) != (pSrcRect->bottom - pSrcRect->top)))
            {
                AlphaBlendRop = 2;
            }
            else
            {
                AlphaBlendRop = 3;
            }
        }
    }
    else
    {
        if (((pDstRect->right - pDstRect->left) != (pSrcRect->right - pSrcRect->left)) ||
            ((pDstRect->bottom - pDstRect->top) != (pSrcRect->bottom - pSrcRect->top)))
        {
            AlphaBlendRop = 4;
        }
        else
        {
            AlphaBlendRop = 5;
        }
    }

    GcDmaBufInfo*   pDmaBufInfo = (GcDmaBufInfo*)pDmaBufferPrivateData;

    NT_ASSERT(pDmaBufInfo->m_DmaBufState.m_bGdi);
    // Forced context switch is required even when src is accessed as Storage Image
    // if ((AlphaBlendRop != 3) && (AlphaBlendRop != 5))
    {
        pDmaBufInfo->m_DmaBufState.m_bGdiSamplerUsed = 1;
    }

    return GdiOpBuildCommandBuffer(
            GetGdiOp(DXGK_GDIOP_ALPHABLEND, AlphaBlendRop),
            pDstDeviceAlloc->m_pGcKmAllocation,
            pDstAllocInfo->VirtualAddress,
            pSrcDeviceAlloc->m_pGcKmAllocation,
            pSrcAllocInfo->VirtualAddress,
            pCmdAlphaBlend->SrcPitch,
            NULL,
            0,
            &pCmdAlphaBlend->DstRect,
            &pCmdAlphaBlend->SrcRect,
            pCmdAlphaBlend->NumSubRects,
            pCmdAlphaBlend->pSubRects,
            ((UINT)pCmdAlphaBlend->SourceConstantAlpha) << 24,
            0,
            0,
            ppDmaBuffer,
            pDmaBufferGpuVirtualAddress,
            DmaBufferSize,
            pDmaBufferPrivateData,
            DmaBufferPrivateDataSize,
            pMultipassOffset);
}

NTSTATUS
GcKm7LContext::GdiStretchBlt(
    DXGK_GDIARG_STRETCHBLT* pCmdStretchBlt,
    DXGK_ALLOCATIONLIST*    pAllocationList,
    UINT                    AllocationListSize,
    BYTE**                  ppDmaBuffer,
    D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
    UINT                    DmaBufferSize,
    BYTE*                   pDmaBufferPrivateData,
    UINT                    DmaBufferPrivateDataSize,
    UINT*                   pMultipassOffset)
{
    DXGK_ALLOCATIONLIST*    pSrcAllocInfo = pAllocationList + pCmdStretchBlt->SrcAllocationIndex;
    DXGK_ALLOCATIONLIST*    pDstAllocInfo = pAllocationList + pCmdStretchBlt->DstAllocationIndex;
    GcKmDeviceAllocation*   pSrcDeviceAlloc = (GcKmDeviceAllocation*)pSrcAllocInfo->hDeviceSpecificAllocation;
    GcKmDeviceAllocation*   pDstDeviceAlloc = (GcKmDeviceAllocation*)pDstAllocInfo->hDeviceSpecificAllocation;

    NT_ASSERT((0 == pCmdStretchBlt->MirrorX) &&
              (0 == pCmdStretchBlt->MirrorY) &&
              (COLORONCOLOR == pCmdStretchBlt->Mode));

    NT_ASSERT((0 == pCmdStretchBlt->SrcPitch) || (pCmdStretchBlt->SrcPitch == pSrcDeviceAlloc->m_pGcKmAllocation->m_hwPitch));

    GcDmaBufInfo*   pDmaBufInfo = (GcDmaBufInfo*)pDmaBufferPrivateData;

    NT_ASSERT(pDmaBufInfo->m_DmaBufState.m_bGdi);
    pDmaBufInfo->m_DmaBufState.m_bGdiSamplerUsed = 1;

    return GdiOpBuildCommandBuffer(
            GetGdiOp(DXGK_GDIOP_STRETCHBLT, 0),
            pDstDeviceAlloc->m_pGcKmAllocation,
            pDstAllocInfo->VirtualAddress,
            pSrcDeviceAlloc->m_pGcKmAllocation,
            pSrcAllocInfo->VirtualAddress,
            pCmdStretchBlt->SrcPitch,
            NULL,
            0,
            &pCmdStretchBlt->DstRect,
            &pCmdStretchBlt->SrcRect,
            pCmdStretchBlt->NumSubRects,
            pCmdStretchBlt->pSubRects,
            0,
            0,
            0,
            ppDmaBuffer,
            pDmaBufferGpuVirtualAddress,
            DmaBufferSize,
            pDmaBufferPrivateData,
            DmaBufferPrivateDataSize,
            pMultipassOffset);
}

NTSTATUS
GcKm7LContext::GdiTransparentBlt(
    DXGK_GDIARG_TRANSPARENTBLT* pCmdTransparentBlt,
    DXGK_ALLOCATIONLIST*        pAllocationList,
    UINT                        AllocationListSize,
    BYTE**                      ppDmaBuffer,
    D3DGPU_VIRTUAL_ADDRESS*     pDmaBufferGpuVirtualAddress,
    UINT                        DmaBufferSize,
    BYTE*                       pDmaBufferPrivateData,
    UINT                        DmaBufferPrivateDataSize,
    UINT*                       pMultipassOffset)
{
    DXGK_ALLOCATIONLIST*    pSrcAllocInfo = pAllocationList + pCmdTransparentBlt->SrcAllocationIndex;
    DXGK_ALLOCATIONLIST*    pDstAllocInfo = pAllocationList + pCmdTransparentBlt->DstAllocationIndex;
    GcKmDeviceAllocation*   pSrcDeviceAlloc = (GcKmDeviceAllocation*)pSrcAllocInfo->hDeviceSpecificAllocation;
    GcKmDeviceAllocation*   pDstDeviceAlloc = (GcKmDeviceAllocation*)pDstAllocInfo->hDeviceSpecificAllocation;

    NT_ASSERT((0 == pCmdTransparentBlt->SrcPitch) || (pCmdTransparentBlt->SrcPitch == pSrcDeviceAlloc->m_pGcKmAllocation->m_hwPitch));

    UINT    TransparentBltRop;

    if (pCmdTransparentBlt->Flags.HonorAlpha)
    {
        TransparentBltRop = 1;
    }
    else
    {
        TransparentBltRop = 2;
    }

    GcDmaBufInfo*   pDmaBufInfo = (GcDmaBufInfo*)pDmaBufferPrivateData;

    NT_ASSERT(pDmaBufInfo->m_DmaBufState.m_bGdi);
    pDmaBufInfo->m_DmaBufState.m_bGdiSamplerUsed = 1;

    return GdiOpBuildCommandBuffer(
            GetGdiOp(DXGK_GDIOP_TRANSPARENTBLT, TransparentBltRop),
            pDstDeviceAlloc->m_pGcKmAllocation,
            pDstAllocInfo->VirtualAddress,
            pSrcDeviceAlloc->m_pGcKmAllocation,
            pSrcAllocInfo->VirtualAddress,
            pCmdTransparentBlt->SrcPitch,
            NULL,
            0,
            &pCmdTransparentBlt->DstRect,
            &pCmdTransparentBlt->SrcRect,
            pCmdTransparentBlt->NumSubRects,
            pCmdTransparentBlt->pSubRects,
            pCmdTransparentBlt->Color,
            0,
            0,
            ppDmaBuffer,
            pDmaBufferGpuVirtualAddress,
            DmaBufferSize,
            pDmaBufferPrivateData,
            DmaBufferPrivateDataSize,
            pMultipassOffset);
}

NTSTATUS
GcKm7LContext::GdiClearTypeBlend(
    DXGK_GDIARG_CLEARTYPEBLEND* pCmdClearTypeBlend,
    DXGK_ALLOCATIONLIST*        pAllocationList,
    UINT                        AllocationListSize,
    BYTE**                      ppDmaBuffer,
    D3DGPU_VIRTUAL_ADDRESS*     pDmaBufferGpuVirtualAddress,
    UINT                        DmaBufferSize,
    BYTE*                       pDmaBufferPrivateData,
    UINT                        DmaBufferPrivateDataSize,
    UINT*                       pMultipassOffset)
{
    DXGK_ALLOCATIONLIST*    pTmpAllocInfo = pAllocationList + pCmdClearTypeBlend->TmpSurfAllocationIndex;
    DXGK_ALLOCATIONLIST*    pGammaAllocInfo = pAllocationList + pCmdClearTypeBlend->GammaSurfAllocationIndex;
    DXGK_ALLOCATIONLIST*    pAlphaAllocInfo = pAllocationList + pCmdClearTypeBlend->AlphaSurfAllocationIndex;
    DXGK_ALLOCATIONLIST*    pDstAllocInfo = pAllocationList + pCmdClearTypeBlend->DstAllocationIndex;
    GcKmDeviceAllocation*   pTmpDeviceAlloc = (GcKmDeviceAllocation*)pTmpAllocInfo->hDeviceSpecificAllocation;
    GcKmDeviceAllocation*   pGammaDeviceAlloc = (GcKmDeviceAllocation*)pGammaAllocInfo->hDeviceSpecificAllocation;
    GcKmDeviceAllocation*   pAlphaDeviceAlloc = (GcKmDeviceAllocation*)pAlphaAllocInfo->hDeviceSpecificAllocation;
    GcKmDeviceAllocation*   pDstDeviceAlloc = (GcKmDeviceAllocation*)pDstAllocInfo->hDeviceSpecificAllocation;

    NT_ASSERT((0 == pCmdClearTypeBlend->AlphaSurfPitch) || (pCmdClearTypeBlend->AlphaSurfPitch == pAlphaDeviceAlloc->m_pGcKmAllocation->m_hwPitch));

    UINT                    ClearTypeBlendRop;
    GcKmAllocation*         pGmmaAllocation;
    D3DGPU_VIRTUAL_ADDRESS  GammaGpuVa;

    if (pCmdClearTypeBlend->Gamma != D3DKM_INVALID_GAMMA_INDEX)
    {
        ClearTypeBlendRop = 1;

        pGmmaAllocation   = pGammaDeviceAlloc->m_pGcKmAllocation;
        GammaGpuVa        = pGammaAllocInfo->VirtualAddress;
    }
    else
    {
        ClearTypeBlendRop = 2;

        pGmmaAllocation   = NULL;
        GammaGpuVa        = 0;
    }

    RECT    SrcRect = pCmdClearTypeBlend->DstRect;

    SrcRect.left   += pCmdClearTypeBlend->DstToAlphaOffsetX;
    SrcRect.top    += pCmdClearTypeBlend->DstToAlphaOffsetY;
    SrcRect.right  += pCmdClearTypeBlend->DstToAlphaOffsetX;
    SrcRect.bottom += pCmdClearTypeBlend->DstToAlphaOffsetY;

    return GdiOpBuildCommandBuffer(
            GetGdiOp(DXGK_GDIOP_CLEARTYPEBLEND, ClearTypeBlendRop),
            pDstDeviceAlloc->m_pGcKmAllocation,
            pDstAllocInfo->VirtualAddress,
            pAlphaDeviceAlloc->m_pGcKmAllocation,
            pAlphaAllocInfo->VirtualAddress,
            pCmdClearTypeBlend->AlphaSurfPitch,
            pGmmaAllocation,
            GammaGpuVa,
            &pCmdClearTypeBlend->DstRect,
            &SrcRect,
            pCmdClearTypeBlend->NumSubRects,
            pCmdClearTypeBlend->pSubRects,
            pCmdClearTypeBlend->Color,
            pCmdClearTypeBlend->Gamma,
            pCmdClearTypeBlend->Color2,
            ppDmaBuffer,
            pDmaBufferGpuVirtualAddress,
            DmaBufferSize,
            pDmaBufferPrivateData,
            DmaBufferPrivateDataSize,
            pMultipassOffset);
}


NTSTATUS
GcKm7LContext::GdiOpBuildCommandBuffer(
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
    UINT*                       pMultipassOffset)
{
    //
    // TODO: Remove when all shaders become ready
    //

    if (0 == pGdiOp->InstGpuVa)
    {
        return STATUS_SUCCESS;
    }

    if (g_bGdiSingleCommandType)
    {
        if (*pMultipassOffset == NumSubRects)
        {
            return STATUS_SUCCESS;
        }
    }

    NT_ASSERT((DXGI_FORMAT_B8G8R8A8_UNORM == pDstAllocation->m_format) || (DXGI_FORMAT_B8G8R8A8_UNORM_SRGB == pDstAllocation->m_format) || (DXGI_FORMAT_A8_UNORM == pDstAllocation->m_format));
    NT_ASSERT((nullptr == pSrcAllocation) || (DXGI_FORMAT_B8G8R8A8_UNORM == pSrcAllocation->m_format) || (DXGI_FORMAT_A8_UNORM == pDstAllocation->m_format));
    NT_ASSERT((nullptr == pGammaAllocation) || (DXGI_FORMAT_A8_UNORM == pGammaAllocation->m_format));

    // TODO: Add support for sampled image
    UINT    RectCmdSize = pGdiOp->CmdBufSize;
    UINT*   pDmaBuffer = (UINT*)*ppDmaBuffer;

    const RECT* pDstSubRect = pDstSubRects + (*pMultipassOffset);

    float ScaleX = 0.0;
    float ScaleY = 0.0;

    if (pSrcRect)
    {
        UINT SrcExtentX, SrcExtentY;
        UINT DstExtentX, DstExtentY;

        SrcExtentX = pSrcRect->right  - pSrcRect->left;
        SrcExtentY = pSrcRect->bottom - pSrcRect->top;

        DstExtentX = pDstRect->right  - pDstRect->left;
        DstExtentY = pDstRect->bottom - pDstRect->top;

        if (DstExtentX)
        {
            ScaleX = ((float)SrcExtentX)/((float)DstExtentX);
        }

        if (DstExtentY)
        {
            ScaleY = ((float)SrcExtentY)/((float)DstExtentY);
        }
    }

    for (UINT i = (*pMultipassOffset); i < NumSubRects; i++)
    {
        UINT            UboGpuVa;
        GdiOpParams*    pParams;

        pParams = GdiAllocateUniformBuffer(
                    (GcDmaBufInfo*)pDmaBufferPrivateData,
                    &UboGpuVa);

        if ((nullptr == pParams) || (DmaBufferSize < RectCmdSize))
        {
            *ppDmaBuffer = (BYTE*)pDmaBuffer;
            *pMultipassOffset = i;

            return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
        }

        memcpy(pDmaBuffer, pGdiOp->pCmdBuf, pGdiOp->CmdBufSize);

        UINT    DstSubExtentX = pDstSubRect->right - pDstSubRect->left;
        UINT    DstSubExtentY = pDstSubRect->bottom - pDstSubRect->top;

        if (0 == DstSubExtentX)
        {
            DstSubExtentX = 1;
        }
        if (0 == DstSubExtentY)
        {
            DstSubExtentY = 1;
        }

        for (UINT j = 0; j < pGdiOp->NumPatchEntries; j++)
        {
            const GC_7L_PATCH_ENTRY*    pPatchEntry = pGdiOp->pPatchTable + j;

            switch (pPatchEntry->Type)
            {
            case GC7L_PATCH_INST_GPUVA:
                *(pDmaBuffer + pPatchEntry->Offset) = pGdiOp->InstGpuVa;
                break;

            case GC7L_PATCH_DST_STORAGE_IMAGE:
                GdiOpPatchStorageImage(
                    pDmaBuffer + pPatchEntry->Offset,
                    pDstAllocation,
                    DstAllocationGpuVa,
                    0);
                break;

            case GC7L_PATCH_SRC_STORAGE_IMAGE:
                GdiOpPatchStorageImage(
                    pDmaBuffer + pPatchEntry->Offset,
                    pSrcAllocation,
                    SrcAllocationGpuVa,
                    SrcPitchOverride);
                break;

            case GC7L_PATCH_SRC_STORAGE_IMAGE_2:
                GdiOpPatchStorageImage(
                    pDmaBuffer + pPatchEntry->Offset,
                    pGammaAllocation,
                    GammaAllocationGpuVa,
                    0);
                break;

            case GC7L_PATCH_SRC_SAMPLED_IMAGE:
                ((GcKm7LAdapter*)m_pDevice->m_pGcKmAdapter)->UpdateTextureDescriptor(
                                                                pSrcAllocation,
                                                                SrcAllocationGpuVa);
                *(pDmaBuffer + pPatchEntry->Offset) = (UINT)pSrcAllocation->m_texDescGpuVa;
                break;

            case GC7L_PATCH_UNIFORM_GPUVA:
            {
                pParams->color.x = Color & 0x000000FF;
                pParams->color.y = (Color & 0x0000FF00) >> 8;
                pParams->color.z = (Color & 0x00FF0000) >> 16;
                pParams->color.w = (Color & 0xFF000000) >> 24;

                pParams->color2.x = Color2 & 0x000000FF;
                pParams->color2.y = (Color2 & 0x0000FF00) >> 8;
                pParams->color2.z = (Color2 & 0x00FF0000) >> 16;
                pParams->color2.w = (Color2 & 0xFF000000) >> 24;

                pParams->gamma = Gamma;

                pParams->dstOffset.x = pDstSubRect->left;
                pParams->dstOffset.y = pDstSubRect->top;

                pParams->dstExtent.x = DstSubExtentX;
                pParams->dstExtent.y = DstSubExtentY;

                if (pSrcRect)
                {
                    pParams->scale.x = ScaleX;
                    pParams->scale.y = ScaleY;

                    pParams->srcOffset.x = ((float)(pDstSubRect->left - pDstRect->left))*ScaleX + ((float)pSrcRect->left);
                    pParams->srcOffset.y = ((float)(pDstSubRect->top  - pDstRect->top ))*ScaleY + ((float)pSrcRect->top );

                    pParams->srcSize.x = (float)pSrcAllocation->m_mip0Info.TexelWidth;
                    pParams->srcSize.y = (float)pSrcAllocation->m_mip0Info.TexelHeight;
                }

                *(pDmaBuffer + pPatchEntry->Offset) = UboGpuVa;
            }
            break;

            case GC7L_PATCH_DISPATCH_WG:
                *(pDmaBuffer + pPatchEntry->Offset    ) = (DstSubExtentX + 0x10 - 1)/0x10 - 1;
                *(pDmaBuffer + pPatchEntry->Offset + 1) = (DstSubExtentY + 0x10 - 1)/0x10 - 1;
                break;

            case GC7L_PATCH_DISPATCH_WG2:
                *(pDmaBuffer + pPatchEntry->Offset    ) = (DstSubExtentX + 0x10 - 1)/0x10;
                *(pDmaBuffer + pPatchEntry->Offset + 1) = (DstSubExtentY + 0x10 - 1)/0x10;
                break;

            }
        }

        pDmaBuffer += RectCmdSize/sizeof(UINT);
        DmaBufferSize -= RectCmdSize;
        *pDmaBufferGpuVirtualAddress += RectCmdSize;

        pDstSubRect++;
    }

    *ppDmaBuffer = (BYTE*)pDmaBuffer;

    if (g_bGdiSingleCommandType)
    {
        *pMultipassOffset = NumSubRects;

        return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
    }
    else
    {
        return STATUS_SUCCESS;
    }
}

void
GcKm7LContext::GdiOpPatchStorageImage(
    UINT*                       pDmaBuffer,
    GcKmAllocation*             pAllocation,
    D3DGPU_VIRTUAL_ADDRESS      AllocationGpuVa,
    UINT                        PitchOverride)
{
    NT_ASSERT((DXGI_FORMAT_B8G8R8A8_UNORM == pAllocation->m_format) ||
              (DXGI_FORMAT_B8G8R8A8_UNORM_SRGB == pAllocation->m_format) ||
              (DXGI_FORMAT_A8_UNORM == pAllocation->m_format));

    pDmaBuffer[0] = (UINT)AllocationGpuVa;

    if (PitchOverride)
    {
        pDmaBuffer[1] = PitchOverride;
    }
    else
    {
        pDmaBuffer[1] = pAllocation->m_hwPitch;
    }

    pDmaBuffer[2] = pAllocation->m_mip0Info.PhysicalHeight << 16 |
                    pAllocation->m_mip0Info.PhysicalWidth;

    switch (pAllocation->m_hwTileMode)
    {
    case Linear:
        if (DXGI_FORMAT_A8_UNORM == pAllocation->m_format)
        {
            pDmaBuffer[3] = 0x544051d0;
        }
        else
        {
            pDmaBuffer[3] = 0x321011d2;
        }
        break;
    case StandardTiled:
        NT_ASSERT(false);
        break;
    case SuperTiled:
        if (DXGI_FORMAT_A8_UNORM == pAllocation->m_format)
        {
            pDmaBuffer[3] = 0x544059d0;
        }
        else
        {
            pDmaBuffer[3] = 0x321019d2;
        }
        break;
    }
}

GdiOpParams*
GcKm7LContext::GdiAllocateUniformBuffer(
    GcDmaBufInfo*   pDmaBufInfo,
    UINT*           pUboGpuVa)
{
    if (!pDmaBufInfo->m_DmaBufState.m_bGdiInitialized)
    {
        UINT    UboChunkSize = m_GdiUboHeap.m_Size/m_MaxPendingDmaBuffers;

        m_GdiUboChunk.m_Size = UboChunkSize;
        m_GdiUboChunk.m_GpuVa = m_GdiUboHeap.m_GpuVa + m_CurGdiUboChunk*UboChunkSize;
        m_GdiUboChunk.m_pMem = m_GdiUboHeap.m_pMem + m_CurGdiUboChunk*UboChunkSize;

        m_CurGdiUboChunk++;
        if (m_CurGdiUboChunk == m_MaxPendingDmaBuffers)
        {
            m_CurGdiUboChunk = 0;
        }

        pDmaBufInfo->m_DmaBufState.m_bGdiInitialized = true;
    }

    if (m_GdiUboChunk.m_Size < s_GdiUniformBufferSize)
    {
        return nullptr;
    }

    GdiOpParams*    pGdiOpParams = (GdiOpParams*)m_GdiUboChunk.m_pMem;
    *pUboGpuVa = m_GdiUboChunk.m_GpuVa;

    m_GdiUboChunk.m_Size  -= s_GdiUniformBufferSize;
    m_GdiUboChunk.m_GpuVa += s_GdiUniformBufferSize;
    m_GdiUboChunk.m_pMem  += s_GdiUniformBufferSize;

    return pGdiOpParams;
}


namespace MQ
{
extern GC_7L_GDI_OP GC_7L_GDI_OP_TABLE[];
};

GC_7L_GDI_OP*
GcKm7LMQContext::GetGdiOp(
    DXGK_RENDERKM_OPERATION GdiOp,
    UINT                    GdiOpRop)
{
    return &MQ::GC_7L_GDI_OP_TABLE[GC_7L_GDI_OP_STARTS[GdiOp] + GdiOpRop];
}

NTSTATUS
GcKm7LMQContext::FillImage(
    GcKmAllocation*         pDstAllocation,
    D3DGPU_VIRTUAL_ADDRESS  DstAllocationVirtualAddress,
    const RECT*             pDstRect,
    UINT                    SubRectCnt,
    const RECT*             pDstSubRects,
    UINT                    ClearColor,
    void**                  ppDmaBuffer,
    UINT                    DmaBufferSize,
    UINT*                   pMultipassOffset)
{
    if (g_bGdiSingleCommandType)
    {
        if (*pMultipassOffset == SubRectCnt)
        {
            return STATUS_SUCCESS;
        }
    }

    //
    // Fill surface with solid color of ClearColor
    //

    UINT* pDmaBuffer = (UINT*)*ppDmaBuffer;
    const RECT* pDstSubRect = pDstSubRects + (*pMultipassOffset);

    for (UINT i = (*pMultipassOffset); i < SubRectCnt; i++)
    {
        if (DmaBufferSize < sizeof(GC_7L_CMD_FILL_IMG))
        {
            *ppDmaBuffer = pDmaBuffer;
            *pMultipassOffset = i;

            return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
        }

        memcpy(pDmaBuffer, GC_7L_CMD_FILL_IMG, sizeof(GC_7L_CMD_FILL_IMG));

        pDmaBuffer[GC_7L_CMD_FILL_IMG_DST_START_GPUVA] = (UINT)DstAllocationVirtualAddress;
        pDmaBuffer[GC_7L_CMD_FILL_IMG_SRC_START_GPUVA] = (UINT)DstAllocationVirtualAddress;
        pDmaBuffer[GC_7L_CMD_FILL_IMG_DST_CONFIG] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 20:0) - (0 ?
 20:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 20:0) - (0 ?
 20:0) + 1))))))) << (0 ?
 20:0))) | (((gctUINT32) ((gctUINT32) (pDstAllocation->m_hwPitch) & ((gctUINT32) ((((1 ?
 20:0) - (0 ?
 20:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 20:0) - (0 ? 20:0) + 1))))))) << (0 ? 20:0))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 28:27) - (0 ?
 28:27) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 28:27) - (0 ?
 28:27) + 1))))))) << (0 ?
 28:27))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 28:27) - (0 ?
 28:27) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 28:27) - (0 ? 28:27) + 1))))))) << (0 ? 28:27))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 29:29) - (0 ?
 29:29) + 1))))))) << (0 ?
 29:29))) | (((gctUINT32) ((gctUINT32) (((pDstAllocation->m_hwTileMode == StandardTiled) || (pDstAllocation->m_hwTileMode == SuperTiled))) & ((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 29:29) - (0 ? 29:29) + 1))))))) << (0 ? 29:29))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))))) << (0 ?
 30:30))) | (((gctUINT32) ((gctUINT32) ((pDstAllocation->m_hwTileMode == SuperTiled)) & ((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 30:30) - (0 ? 30:30) + 1))))))) << (0 ? 30:30))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31)));
        pDmaBuffer[GC_7L_CMD_FILL_IMG_DST_CONFIG_EX] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 27:26) - (0 ?
 27:26) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 27:26) - (0 ?
 27:26) + 1))))))) << (0 ?
 27:26))) | (((gctUINT32) ((gctUINT32) ((pDstAllocation->m_hwTileMode == SuperTiled ?
 1 : 0)) & ((gctUINT32) ((((1 ?
 27:26) - (0 ?
 27:26) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 27:26) - (0 ? 27:26) + 1))))))) << (0 ? 27:26)));
        pDmaBuffer[GC_7L_CMD_FILL_IMG_SRC_CONFIG] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 20:0) - (0 ?
 20:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 20:0) - (0 ?
 20:0) + 1))))))) << (0 ?
 20:0))) | (((gctUINT32) ((gctUINT32) (pDstAllocation->m_hwPitch) & ((gctUINT32) ((((1 ?
 20:0) - (0 ?
 20:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 20:0) - (0 ? 20:0) + 1))))))) << (0 ? 20:0))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 28:27) - (0 ?
 28:27) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 28:27) - (0 ?
 28:27) + 1))))))) << (0 ?
 28:27))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 28:27) - (0 ?
 28:27) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 28:27) - (0 ? 28:27) + 1))))))) << (0 ? 28:27))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 29:29) - (0 ?
 29:29) + 1))))))) << (0 ?
 29:29))) | (((gctUINT32) ((gctUINT32) (((pDstAllocation->m_hwTileMode == StandardTiled) || (pDstAllocation->m_hwTileMode == SuperTiled))) & ((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 29:29) - (0 ? 29:29) + 1))))))) << (0 ? 29:29))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))))) << (0 ?
 30:30))) | (((gctUINT32) ((gctUINT32) ((pDstAllocation->m_hwTileMode == SuperTiled)) & ((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 30:30) - (0 ? 30:30) + 1))))))) << (0 ? 30:30))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31)));
        pDmaBuffer[GC_7L_CMD_FILL_IMG_SRC_CONFIG_EX] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:21) - (0 ?
 22:21) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 22:21) - (0 ?
 22:21) + 1))))))) << (0 ?
 22:21))) | (((gctUINT32) ((gctUINT32) ((pDstAllocation->m_hwTileMode == SuperTiled ?
 1 : 0)) & ((gctUINT32) ((((1 ?
 22:21) - (0 ?
 22:21) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 22:21) - (0 ? 22:21) + 1))))))) << (0 ? 22:21)));
        pDmaBuffer[GC_7L_CMD_FILL_IMG_SRC_END_GPUVA] = (UINT)(DstAllocationVirtualAddress + pDstAllocation->m_hwSizeBytes - 1);
        pDmaBuffer[GC_7L_CMD_FILL_IMG_DST_END_GPUVA] = (UINT)(DstAllocationVirtualAddress + pDstAllocation->m_hwSizeBytes - 1);
        pDmaBuffer[GC_7L_CMD_FILL_IMG_BLT_NEW_CLEAR_VALUE1] = ClearColor;
        pDmaBuffer[GC_7L_CMD_FILL_IMG_BLT_NEW_CLEAR_VALUE2] = ClearColor;
        pDmaBuffer[GC_7L_CMD_FILL_IMG_DST_ORIGIN_YX] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (pDstSubRect->left) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0))) |
                                                       ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 31:16) - (0 ?
 31:16) + 1))))))) << (0 ?
 31:16))) | (((gctUINT32) ((gctUINT32) (pDstSubRect->top) & ((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 31:16) - (0 ? 31:16) + 1))))))) << (0 ? 31:16)));
        pDmaBuffer[GC_7L_CMD_FILL_IMG_WINDOW_SIZE_YX] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) ((pDstSubRect->right - pDstSubRect->left)) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0))) |
                                                        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 31:16) - (0 ?
 31:16) + 1))))))) << (0 ?
 31:16))) | (((gctUINT32) ((gctUINT32) ((pDstSubRect->bottom - pDstSubRect->top)) & ((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 31:16) - (0 ? 31:16) + 1))))))) << (0 ? 31:16)));

        pDmaBuffer += sizeof(GC_7L_CMD_FILL_IMG) / sizeof(UINT);
        DmaBufferSize -= sizeof(GC_7L_CMD_FILL_IMG);
        pDstSubRect++;
    }

    *ppDmaBuffer = pDmaBuffer;

    if (g_bGdiSingleCommandType)
    {
        *pMultipassOffset = SubRectCnt;

        return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
    }
    else
    {
        return STATUS_SUCCESS;
    }
}

NTSTATUS
GcKm7LMQContext::CopyImage(
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
    UINT*                   pMultipassOffset)
{
    if (g_bGdiSingleCommandType)
    {
        if (*pMultipassOffset == SubRectCnt)
        {
            return STATUS_SUCCESS;
        }
    }

    NT_ASSERT(pDstAllocation->m_format == pSrcAllocation->m_format);
    NT_ASSERT((DXGI_FORMAT_B8G8R8A8_UNORM == pDstAllocation->m_format) ||
              (DXGI_FORMAT_A8_UNORM == pDstAllocation->m_format));

    //
    // Used by CDD for GDI copy between primary and shadow
    //

    UINT*   pDmaBuffer = (UINT*)*ppDmaBuffer;
    LONG    SrcOffsetX = pSrcRect->left - pDstRect->left;
    LONG    SrcOffsetY = pSrcRect->top - pDstRect->top;
    UINT    Format;

    if (pDstAllocation->m_format == DXGI_FORMAT_B8G8R8A8_UNORM)
    {
        Format = 0x06;
    }
    else
    {
        Format = 0x10;
    }

    const RECT* pDstSubRect = pDstSubRects + (*pMultipassOffset);

    for (UINT i = (*pMultipassOffset); i < SubRectCnt; i++)
    {
        if (DmaBufferSize < sizeof(GC_7L_CMD_COPY_IMG))
        {
            *ppDmaBuffer = pDmaBuffer;
            *pMultipassOffset = i;

            return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
        }

        memcpy(pDmaBuffer, GC_7L_CMD_COPY_IMG, sizeof(GC_7L_CMD_COPY_IMG));

        pDmaBuffer[GC_7L_CMD_COPY_IMG_SRC_CONFIG] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 20:0) - (0 ?
 20:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 20:0) - (0 ?
 20:0) + 1))))))) << (0 ?
 20:0))) | (((gctUINT32) ((gctUINT32) (pSrcAllocation->m_hwPitch) & ((gctUINT32) ((((1 ?
 20:0) - (0 ?
 20:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 20:0) - (0 ? 20:0) + 1))))))) << (0 ? 20:0))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:21) - (0 ?
 26:21) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 26:21) - (0 ?
 26:21) + 1))))))) << (0 ?
 26:21))) | (((gctUINT32) ((gctUINT32) (Format) & ((gctUINT32) ((((1 ?
 26:21) - (0 ?
 26:21) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 26:21) - (0 ? 26:21) + 1))))))) << (0 ? 26:21))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 28:27) - (0 ?
 28:27) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 28:27) - (0 ?
 28:27) + 1))))))) << (0 ?
 28:27))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 28:27) - (0 ?
 28:27) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 28:27) - (0 ? 28:27) + 1))))))) << (0 ? 28:27))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 29:29) - (0 ?
 29:29) + 1))))))) << (0 ?
 29:29))) | (((gctUINT32) ((gctUINT32) (((pSrcAllocation->m_hwTileMode == StandardTiled) || (pSrcAllocation->m_hwTileMode == SuperTiled))) & ((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 29:29) - (0 ? 29:29) + 1))))))) << (0 ? 29:29))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))))) << (0 ?
 30:30))) | (((gctUINT32) ((gctUINT32) ((pSrcAllocation->m_hwTileMode == SuperTiled)) & ((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 30:30) - (0 ? 30:30) + 1))))))) << (0 ? 30:30))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31)));
        pDmaBuffer[GC_7L_CMD_COPY_IMG_SRC_CONFIG_EX] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:21) - (0 ?
 22:21) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 22:21) - (0 ?
 22:21) + 1))))))) << (0 ?
 22:21))) | (((gctUINT32) ((gctUINT32) ((pSrcAllocation->m_hwTileMode == SuperTiled ?
 1 : 0)) & ((gctUINT32) ((((1 ?
 22:21) - (0 ?
 22:21) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 22:21) - (0 ? 22:21) + 1))))))) << (0 ? 22:21)));
        pDmaBuffer[GC_7L_CMD_COPY_IMG_SRC_START_GPUVA] = (UINT)SrcAllocationVirtualAddress;
        pDmaBuffer[GC_7L_CMD_COPY_IMG_DST_CONFIG] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 20:0) - (0 ?
 20:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 20:0) - (0 ?
 20:0) + 1))))))) << (0 ?
 20:0))) | (((gctUINT32) ((gctUINT32) (pDstAllocation->m_hwPitch) & ((gctUINT32) ((((1 ?
 20:0) - (0 ?
 20:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 20:0) - (0 ? 20:0) + 1))))))) << (0 ? 20:0))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:21) - (0 ?
 26:21) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 26:21) - (0 ?
 26:21) + 1))))))) << (0 ?
 26:21))) | (((gctUINT32) ((gctUINT32) (Format) & ((gctUINT32) ((((1 ?
 26:21) - (0 ?
 26:21) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 26:21) - (0 ? 26:21) + 1))))))) << (0 ? 26:21))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 28:27) - (0 ?
 28:27) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 28:27) - (0 ?
 28:27) + 1))))))) << (0 ?
 28:27))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 28:27) - (0 ?
 28:27) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 28:27) - (0 ? 28:27) + 1))))))) << (0 ? 28:27))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 29:29) - (0 ?
 29:29) + 1))))))) << (0 ?
 29:29))) | (((gctUINT32) ((gctUINT32) (((pDstAllocation->m_hwTileMode == StandardTiled) || (pDstAllocation->m_hwTileMode == SuperTiled))) & ((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 29:29) - (0 ? 29:29) + 1))))))) << (0 ? 29:29))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 30:30) - (0 ?
 30:30) + 1))))))) << (0 ?
 30:30))) | (((gctUINT32) ((gctUINT32) ((pDstAllocation->m_hwTileMode == SuperTiled)) & ((gctUINT32) ((((1 ?
 30:30) - (0 ?
 30:30) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 30:30) - (0 ? 30:30) + 1))))))) << (0 ? 30:30))) |
                                                    ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 31:31) - (0 ?
 31:31) + 1))))))) << (0 ?
 31:31))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 31:31) - (0 ?
 31:31) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 31:31) - (0 ? 31:31) + 1))))))) << (0 ? 31:31)));
        pDmaBuffer[GC_7L_CMD_COPY_IMG_DST_CONFIG_EX] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 27:26) - (0 ?
 27:26) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 27:26) - (0 ?
 27:26) + 1))))))) << (0 ?
 27:26))) | (((gctUINT32) ((gctUINT32) ((pDstAllocation->m_hwTileMode == SuperTiled ?
 1 : 0)) & ((gctUINT32) ((((1 ?
 27:26) - (0 ?
 27:26) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 27:26) - (0 ? 27:26) + 1))))))) << (0 ? 27:26))) |
                                                       ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 22:22) - (0 ?
 22:22) + 1))))))) << (0 ?
 22:22))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 22:22) - (0 ?
 22:22) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 22:22) - (0 ? 22:22) + 1))))))) << (0 ? 22:22)));
        pDmaBuffer[GC_7L_CMD_COPY_IMG_DST_START_GPUVA] = (UINT)DstAllocationVirtualAddress;
        pDmaBuffer[GC_7L_CMD_COPY_IMG_SRC_END_GPUVA] = (UINT)(SrcAllocationVirtualAddress + pSrcAllocation->m_hwSizeBytes - 1);
        pDmaBuffer[GC_7L_CMD_COPY_IMG_DST_END_GPUVA] = (UINT)(DstAllocationVirtualAddress + pDstAllocation->m_hwSizeBytes - 1);
        pDmaBuffer[GC_7L_CMD_COPY_IMG_SRC_ORIGIN_YX] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) ((pDstSubRect->left + SrcOffsetX)) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0))) |
                                                       ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 31:16) - (0 ?
 31:16) + 1))))))) << (0 ?
 31:16))) | (((gctUINT32) ((gctUINT32) ((pDstSubRect->top + SrcOffsetY)) & ((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 31:16) - (0 ? 31:16) + 1))))))) << (0 ? 31:16)));
        pDmaBuffer[GC_7L_CMD_COPY_IMG_DST_ORIGIN_YX] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (pDstSubRect->left) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0))) |
                                                       ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 31:16) - (0 ?
 31:16) + 1))))))) << (0 ?
 31:16))) | (((gctUINT32) ((gctUINT32) (pDstSubRect->top) & ((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 31:16) - (0 ? 31:16) + 1))))))) << (0 ? 31:16)));
        pDmaBuffer[GC_7L_CMD_COPY_IMG_WINDOW_SIZE_YX] = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) ((pDstSubRect->right - pDstSubRect->left)) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0))) |
                                                        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ?
 31:16) - (0 ?
 31:16) + 1))))))) << (0 ?
 31:16))) | (((gctUINT32) ((gctUINT32) ((pDstSubRect->bottom - pDstSubRect->top)) & ((gctUINT32) ((((1 ?
 31:16) - (0 ?
 31:16) + 1) == 32) ?
 ~0 : (~(~0 << ((1 ? 31:16) - (0 ? 31:16) + 1))))))) << (0 ? 31:16)));

        pDmaBuffer += sizeof(GC_7L_CMD_COPY_IMG)/sizeof(UINT);
        DmaBufferSize -= sizeof(GC_7L_CMD_COPY_IMG);
        pDstSubRect++;
    }

    *ppDmaBuffer = pDmaBuffer;

    if (g_bGdiSingleCommandType)
    {
        *pMultipassOffset = SubRectCnt;

        return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
    }
    else
    {
        return STATUS_SUCCESS;
    }
}

BOOL    g_bBBUse3DBlt = true;

NTSTATUS
GcKm7LMQContext::GdiBitBlt(
    DXGK_GDIARG_BITBLT*     pCmdBitBlt,
    DXGK_ALLOCATIONLIST*    pAllocationList,
    UINT                    AllocationListSize,
    BYTE**                  ppDmaBuffer,
    D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
    UINT                    DmaBufferSize,
    BYTE*                   pDmaBufferPrivateData,
    UINT                    DmaBufferPrivateDataSize,
    UINT*                   pMultipassOffset)
{
    DXGK_ALLOCATIONLIST*    pSrcAllocInfo = pAllocationList + pCmdBitBlt->SrcAllocationIndex;
    DXGK_ALLOCATIONLIST*    pDstAllocInfo = pAllocationList + pCmdBitBlt->DstAllocationIndex;
    GcKmDeviceAllocation*   pSrcDeviceAlloc = (GcKmDeviceAllocation*)pSrcAllocInfo->hDeviceSpecificAllocation;
    GcKmDeviceAllocation*   pDstDeviceAlloc = (GcKmDeviceAllocation*)pDstAllocInfo->hDeviceSpecificAllocation;

    NT_ASSERT(pCmdBitBlt->Rop != DXGK_GDIROP_ROP3);

    NT_ASSERT((0 == pCmdBitBlt->SrcPitch) || (pCmdBitBlt->SrcPitch == pSrcDeviceAlloc->m_pGcKmAllocation->m_hwPitch));
    NT_ASSERT((0 == pCmdBitBlt->DstPitch) || (pCmdBitBlt->DstPitch == pDstDeviceAlloc->m_pGcKmAllocation->m_hwPitch));

    if ((DXGK_GDIROP_SRCCOPY == pCmdBitBlt->Rop) &&
        ((pSrcDeviceAlloc->m_pGcKmAllocation->m_format == DXGI_FORMAT_A8_UNORM) ||
         g_bBBUse3DBlt))
    {
        BYTE*       pCurDmaBuffer = *ppDmaBuffer;
        NTSTATUS    Status;

        Status = CopyImage(
                    pSrcDeviceAlloc->m_pGcKmAllocation,
                    pSrcAllocInfo->VirtualAddress,
                    pDstDeviceAlloc->m_pGcKmAllocation,
                    pDstAllocInfo->VirtualAddress,
                    &pCmdBitBlt->SrcRect,
                    &pCmdBitBlt->DstRect,
                    pCmdBitBlt->NumSubRects,
                    pCmdBitBlt->pSubRects,
                    (void**)ppDmaBuffer,
                    pDmaBufferGpuVirtualAddress,
                    DmaBufferSize,
                    pDmaBufferPrivateData,
                    DmaBufferPrivateDataSize,
                    pMultipassOffset);

        *pDmaBufferGpuVirtualAddress += (*ppDmaBuffer - pCurDmaBuffer);

        return Status;
    }

    return GcKm7LContext::GdiBitBlt(
                            pCmdBitBlt,
                            pAllocationList,
                            AllocationListSize,
                            ppDmaBuffer,
                            pDmaBufferGpuVirtualAddress,
                            DmaBufferSize,
                            pDmaBufferPrivateData,
                            DmaBufferPrivateDataSize,
                            pMultipassOffset);
}

BOOL    g_bCFUse3DBlt = true;

NTSTATUS
GcKm7LMQContext::GdiColorFill(
    DXGK_GDIARG_COLORFILL*  pCmdColorFill,
    DXGK_ALLOCATIONLIST*    pAllocationList,
    UINT                    AllocationListSize,
    BYTE**                  ppDmaBuffer,
    D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
    UINT                    DmaBufferSize,
    BYTE*                   pDmaBufferPrivateData,
    UINT                    DmaBufferPrivateDataSize,
    UINT*                   pMultipassOffset)
{
    DXGK_ALLOCATIONLIST*    pDstAllocInfo = pAllocationList + pCmdColorFill->DstAllocationIndex;
    GcKmDeviceAllocation*   pDstDeviceAlloc = (GcKmDeviceAllocation*)pDstAllocInfo->hDeviceSpecificAllocation;

    NT_ASSERT(pCmdColorFill->Rop != DXGK_GDIROPCF_ROP3);

    if ((DXGK_GDIROPCF_PATCOPY == pCmdColorFill->Rop) && g_bCFUse3DBlt)
    {
        BYTE*       pCurDmaBuffer = *ppDmaBuffer;
        NTSTATUS    Status;

        Status = FillImage(
                    pDstDeviceAlloc->m_pGcKmAllocation,
                    pDstAllocInfo->VirtualAddress,
                    &pCmdColorFill->DstRect,
                    pCmdColorFill->NumSubRects,
                    pCmdColorFill->pSubRects,
                    pCmdColorFill->Color,
                    (void**)ppDmaBuffer,
                    DmaBufferSize,
                    pMultipassOffset);

        *pDmaBufferGpuVirtualAddress += (*ppDmaBuffer - pCurDmaBuffer);

        return Status;
    }

    return GcKm7LContext::GdiColorFill(
                            pCmdColorFill,
                            pAllocationList,
                            AllocationListSize,
                            ppDmaBuffer,
                            pDmaBufferGpuVirtualAddress,
                            DmaBufferSize,
                            pDmaBufferPrivateData,
                            DmaBufferPrivateDataSize,
                            pMultipassOffset);
}

NTSTATUS
GcKm7LMQContext::FillBuffer(
    DXGKARG_BUILDPAGINGBUFFER* pBuildPagingBuffer)
{
    DXGK_BUILDPAGINGBUFFER_FILLVIRTUAL *pFillVirtual;
    GcDmaBufInfo   *pDmaBufInfo;

    NT_ASSERT(DXGK_OPERATION_VIRTUAL_FILL == pBuildPagingBuffer->Operation);

    pFillVirtual = &pBuildPagingBuffer->FillVirtual;
    pDmaBufInfo = (GcDmaBufInfo *)pBuildPagingBuffer->pDmaBufferPrivateData;

    UINT    FillSize = (UINT)pFillVirtual->FillSizeInBytes;

    // Pre-canned fill command buffer only handles whole page
    NT_ASSERT(0 == (FillSize % PAGE_SIZE));

    UINT    NumFillCommands;

    NumFillCommands = FillSize/GC_7L_CMD_FILL_BUF_MAXIMAL_FILL;
    if (FillSize % GC_7L_CMD_FILL_BUF_MAXIMAL_FILL)
    {
        NumFillCommands++;
    }

    if (NumFillCommands*sizeof(GC_7L_CMD_FILL_BUF) > pBuildPagingBuffer->DmaSize)
    {
        return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
    }

    UINT    FillStartGpuVa = (UINT)pFillVirtual->DestinationVirtualAddress;
    UINT    FillEndGpuVa;
    UINT    FillHeight, SubFillSize;
    UINT   *pCmdFillBuf = (UINT *)pBuildPagingBuffer->pDmaBuffer;

    for (UINT i = 0; i < NumFillCommands; i++)
    {
        memcpy(pCmdFillBuf, GC_7L_CMD_FILL_BUF, sizeof(GC_7L_CMD_FILL_BUF));

        if (FillSize < GC_7L_CMD_FILL_BUF_MAXIMAL_FILL)
        {
            SubFillSize = FillSize;
        }
        else
        {
            SubFillSize = GC_7L_CMD_FILL_BUF_MAXIMAL_FILL;
        }

        FillEndGpuVa = FillStartGpuVa + SubFillSize - 1;
        FillHeight = SubFillSize/PAGE_SIZE;

        pCmdFillBuf[GC_7L_CMD_FILL_BUF_DST_START_GPUVA] = FillStartGpuVa;
        pCmdFillBuf[GC_7L_CMD_FILL_BUF_SRC_START_GPUVA] = FillStartGpuVa;
        pCmdFillBuf[GC_7L_CMD_FILL_BUF_SRC_END_GPUVA]   = FillEndGpuVa;
        pCmdFillBuf[GC_7L_CMD_FILL_BUF_DST_END_GPUVA]   = FillEndGpuVa;
        pCmdFillBuf[GC_7L_CMD_FILL_BUF_FILL_RECT]       = FillHeight << 16 | (PAGE_SIZE/sizeof(UINT));
        pCmdFillBuf[GC_7L_CMD_FILL_BUF_FILL_PATTERN]    = pFillVirtual->FillPattern;

        pCmdFillBuf     += (sizeof(GC_7L_CMD_FILL_BUF)/sizeof(UINT));
        FillStartGpuVa  += SubFillSize;
        FillSize        -= SubFillSize;
    }

    pBuildPagingBuffer->pDmaBuffer = pCmdFillBuf;

    pDmaBufInfo->m_DmaBufState.m_bPaging = 1;

    return STATUS_SUCCESS;
}

NTSTATUS
GcKm7LMQContext::TransferBuffer(
    DXGKARG_BUILDPAGINGBUFFER* pBuildPagingBuffer)
{
    DXGK_BUILDPAGINGBUFFER_TRANSFERVIRTUAL *pTransferVirtual;
    GcDmaBufInfo   *pDmaBufInfo;

    NT_ASSERT(DXGK_OPERATION_VIRTUAL_TRANSFER == pBuildPagingBuffer->Operation);

    pTransferVirtual = &pBuildPagingBuffer->TransferVirtual;
    pDmaBufInfo = (GcDmaBufInfo *)pBuildPagingBuffer->pDmaBufferPrivateData;

    if (pBuildPagingBuffer->DmaSize < sizeof(GC_7L_CMD_COPY_BUF))
    {
        return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
    }

    if (GcKmdGlobal::s_bLimitAllocBelow4gbPa &&
        (pTransferVirtual->TransferDirection == DXGK_MEMORY_TRANSFER_LOCAL_TO_SYSTEM ||
         pTransferVirtual->TransferDirection == DXGK_MEMORY_TRANSFER_SYSTEM_TO_LOCAL))
    {
        // If this buffer already has other cmds in it, ask for a new buffer.
        if (pBuildPagingBuffer->DmaBufferWriteOffset != 0)
        {
            return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
        }

        // Handle the transfer using CPU copy to avoid accessing addresses over 4GB in GPU
        pDmaBufInfo->m_DmaBufState.m_bCpuTransfer = 1;
        pDmaBufInfo->m_TransferVirtual = *pTransferVirtual;
    }

    // Prepare DMA buffer even if m_bCpuTransfer = 1 in case we end up using GPU based on
    // whether physical addresses are above 4GB or not
    UINT   *pCmdCopyBuf = (UINT *)pBuildPagingBuffer->pDmaBuffer;

    memcpy(pCmdCopyBuf, GC_7L_CMD_COPY_BUF, sizeof(GC_7L_CMD_COPY_BUF));

    UINT    TransferSize = (UINT)pTransferVirtual->TransferSizeInBytes;

    pCmdCopyBuf[GC_7L_CMD_COPY_BUF_SRC_START_GPUVA] = (UINT)pTransferVirtual->SourceVirtualAddress;
    pCmdCopyBuf[GC_7L_CMD_COPY_BUF_DST_START_GPUVA] = (UINT)pTransferVirtual->DestinationVirtualAddress;
    pCmdCopyBuf[GC_7L_CMD_COPY_BUF_COPY_SIZE]       = TransferSize;
    pCmdCopyBuf[GC_7L_CMD_COPY_BUF_SRC_END_GPUVA]   = (UINT)(pTransferVirtual->SourceVirtualAddress + TransferSize - 1);
    pCmdCopyBuf[GC_7L_CMD_COPY_BUF_DST_END_GPUVA]   = (UINT)(pTransferVirtual->DestinationVirtualAddress + TransferSize - 1);

    if (pDmaBufInfo->m_DmaBufState.m_bCpuTransfer)
    {
        // Consume the full buffer
        pBuildPagingBuffer->pDmaBuffer = ((BYTE *)pBuildPagingBuffer->pDmaBuffer) + pBuildPagingBuffer->DmaSize;
    }
    else
    {
        pBuildPagingBuffer->pDmaBuffer = ((BYTE *)pBuildPagingBuffer->pDmaBuffer) + sizeof(GC_7L_CMD_COPY_BUF);
    }

    pDmaBufInfo->m_DmaBufState.m_bPaging = 1;

    return STATUS_SUCCESS;
}


namespace MP
{
extern GC_7L_GDI_OP GC_7L_GDI_OP_TABLE[];
};

GcKm7LMPContext::~GcKm7LMPContext()
{
    if (m_hGdiAllocation)
    {
        auto pDxgkInterface = m_pDevice->m_pGcKmAdapter->GetDxgkInterface();

        pDxgkInterface->DxgkCbDestroyContextAllocation(
                            pDxgkInterface->DeviceHandle,
                            m_hGdiAllocation);
    }
}

GC_7L_GDI_OP*
GcKm7LMPContext::GetGdiOp(
    DXGK_RENDERKM_OPERATION GdiOp,
    UINT                    GdiOpRop)
{
    return &MP::GC_7L_GDI_OP_TABLE[GC_7L_GDI_OP_STARTS[GdiOp] + GdiOpRop];
}

NTSTATUS
GcKm7LMPContext::CopyImage(
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
    UINT*                   pMultipassOffset)
{
    if (0 == m_Flags.GdiContext)
    {
        if (NULL == m_hGdiAllocation)
        {
            m_hGdiAllocation = ((GcKm7LAdapter*)m_pDevice->m_pGcKmAdapter)->GetGdiUniformBufferHeap(
                                                                                m_Flags,
                                                                                m_pDevice->GetRtDevice(),
                                                                                m_hRTContext,
                                                                                &m_GdiUboHeap,
                                                                                &m_MaxPendingDmaBuffers);
            if (NULL == m_hGdiAllocation)
            {
                return STATUS_NO_MEMORY;
            }
        }

        GcDmaBufInfo* pDmaBufInfo = (GcDmaBufInfo*)pDmaBufferPrivateData;

        pDmaBufInfo->m_DmaBufState.m_bGdiSamplerUsed = 1;
    }

    return GdiOpBuildCommandBuffer(
            GetGdiOp(DXGK_GDIOP_BITBLT, DXGK_GDIROP_SRCCOPY),
            pDstAllocation,
            DstAllocationVirtualAddress,
            pSrcAllocation,
            SrcAllocationVirtualAddress,
            0,
            NULL,
            0,
            (RECT*)pDstRect,
            (RECT*)pSrcRect,
            SubRectCnt,
            (RECT*)pDstSubRects,
            0,
            0,
            0,
            (BYTE**)ppDmaBuffer,
            pDmaBufferGpuVirtualAddress,
            DmaBufferSize,
            pDmaBufferPrivateData,
            DmaBufferPrivateDataSize,
            pMultipassOffset);
}

NTSTATUS
GcKm7LMPContext::FillBuffer(
    DXGKARG_BUILDPAGINGBUFFER* pBuildPagingBuffer)
{
    NT_ASSERT(DXGK_OPERATION_VIRTUAL_FILL == pBuildPagingBuffer->Operation);

    DXGK_BUILDPAGINGBUFFER_FILLVIRTUAL* pFillVirtual = &pBuildPagingBuffer->FillVirtual;
    GcKmAllocation  PagingDstAlloc;
    GcDmaBufInfo*   pDmaBufInfo = (GcDmaBufInfo*)pBuildPagingBuffer->pDmaBufferPrivateData;

    pDmaBufInfo->m_DmaBufState.m_bPaging = 1;

    DXGKARG_GETSTANDARDALLOCATIONDRIVERDATA GetAllocDriverData = {};
    D3DKMDT_STAGINGSURFACEDATA  StagingSurfData;

    StagingSurfData.Width = 64;
    StagingSurfData.Height = (UINT)pFillVirtual->FillSizeInBytes/(64*4);

    GetAllocDriverData.StandardAllocationType = D3DKMDT_STANDARDALLOCATION_STAGINGSURFACE;
    GetAllocDriverData.pCreateStagingSurfaceData = &StagingSurfData;
    GetAllocDriverData.AllocationPrivateDriverDataSize = sizeof(AllocationExchangeData);
    GetAllocDriverData.pAllocationPrivateDriverData = &PagingDstAlloc;

    m_pDevice->m_pGcKmAdapter->GetStandardAllocationDriverData(&GetAllocDriverData);

    RECT    DstRect = { 0, 0, 64, (LONG)StagingSurfData.Height };
    UINT    MultipassOffset = 0;

    return GdiOpBuildCommandBuffer(
            GetGdiOp(DXGK_GDIOP_COLORFILL, DXGK_GDIROPCF_PATCOPY),
            &PagingDstAlloc,
            pFillVirtual->DestinationVirtualAddress,
            NULL,
            0,
            0,
            NULL,
            0,
            &DstRect,
            NULL,
            1,
            &DstRect,
            pFillVirtual->FillPattern,
            0,
            0,
            (BYTE**)&pBuildPagingBuffer->pDmaBuffer,
            &pBuildPagingBuffer->DmaBufferGpuVirtualAddress,
            pBuildPagingBuffer->DmaSize,
            (BYTE*)pBuildPagingBuffer->pDmaBufferPrivateData,
            pBuildPagingBuffer->DmaBufferPrivateDataSize,
            &MultipassOffset);
}

NTSTATUS
GcKm7LMPContext::TransferBuffer(
    DXGKARG_BUILDPAGINGBUFFER* pBuildPagingBuffer)
{
    NT_ASSERT(DXGK_OPERATION_VIRTUAL_TRANSFER == pBuildPagingBuffer->Operation);

    DXGK_BUILDPAGINGBUFFER_TRANSFERVIRTUAL* pTransferVirtual = &pBuildPagingBuffer->TransferVirtual;
    GcKmAllocation  PagingDstAlloc;
    GcKmAllocation  PagingSrcAlloc;
    GcDmaBufInfo*   pDmaBufInfo = (GcDmaBufInfo*)pBuildPagingBuffer->pDmaBufferPrivateData;

    pDmaBufInfo->m_DmaBufState.m_bPaging = 1;

    DXGKARG_GETSTANDARDALLOCATIONDRIVERDATA GetAllocDriverData = {};
    D3DKMDT_STAGINGSURFACEDATA  StagingSurfData;

    StagingSurfData.Width = 64;
    StagingSurfData.Height = (UINT)pTransferVirtual->TransferSizeInBytes/(64*4);

    GetAllocDriverData.StandardAllocationType = D3DKMDT_STANDARDALLOCATION_STAGINGSURFACE;
    GetAllocDriverData.pCreateStagingSurfaceData = &StagingSurfData;
    GetAllocDriverData.AllocationPrivateDriverDataSize = sizeof(AllocationExchangeData);
    GetAllocDriverData.pAllocationPrivateDriverData = &PagingDstAlloc;

    m_pDevice->m_pGcKmAdapter->GetStandardAllocationDriverData(&GetAllocDriverData);

    GetAllocDriverData.pAllocationPrivateDriverData = &PagingSrcAlloc;

    m_pDevice->m_pGcKmAdapter->GetStandardAllocationDriverData(&GetAllocDriverData);

    RECT    DstRect = { 0, 0, 64, (LONG)StagingSurfData.Height };
    UINT    MultipassOffset = 0;

    return GdiOpBuildCommandBuffer(
            GetGdiOp(DXGK_GDIOP_BITBLT, DXGK_GDIROP_SRCCOPY),
            &PagingDstAlloc,
            pTransferVirtual->DestinationVirtualAddress,
            &PagingSrcAlloc,
            pTransferVirtual->SourceVirtualAddress,
            0,
            NULL,
            0,
            &DstRect,
            &DstRect,
            1,
            &DstRect,
            0,
            0,
            0,
            (BYTE**)&pBuildPagingBuffer->pDmaBuffer,
            &pBuildPagingBuffer->DmaBufferGpuVirtualAddress,
            pBuildPagingBuffer->DmaSize,
            (BYTE*)pBuildPagingBuffer->pDmaBufferPrivateData,
            pBuildPagingBuffer->DmaBufferPrivateDataSize,
            &MultipassOffset);
}


