/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmdContext.tmh"

#include "GcKmdGlobal.h"
#include "GcKmdAdapter.h"
#include "GcKmdDevice.h"
#include "GcKmdContext.h"
#include "GcKmdUtil.h"

#include "GcKmdSwQueueNode.h"

#include "d3d_km_interface.h"

GcKmContext::GcKmContext()
{
    m_pUmdPresentEvent = nullptr;
    m_pKmdPresentEvent = nullptr;
}

NTSTATUS
GcKmContext::Initialize(
    GcKmDevice*             pDevice,
    DXGKARG_CREATECONTEXT*  pCreateContext)
{
    if (!(pCreateContext->Flags.VirtualAddressing || pCreateContext->Flags.SystemContext))
    {
        GC_LOG_ERROR("GcKmContext supports only Virtual Addressing.");
        return STATUS_INVALID_PARAMETER;
    }

    m_hRTContext = pCreateContext->hContext;
    m_pDevice = pDevice;
    m_Flags = pCreateContext->Flags;

    m_Node = pCreateContext->NodeOrdinal;

    if ((pCreateContext->pPrivateDriverData) &&
        (pCreateContext->PrivateDriverDataSize == sizeof(D3dKmPresentSyncInfo)))
    {
        D3dKmPresentSyncInfo* pD3dKmPresentInfo = (D3dKmPresentSyncInfo*)pCreateContext->pPrivateDriverData;
        NTSTATUS Status;

        Status = ObReferenceObjectByHandle(
                    pD3dKmPresentInfo->hUmdEvent,
                    EVENT_MODIFY_STATE,
                    *ExEventObjectType,
                    UserMode,
                    (PVOID*)&m_pUmdPresentEvent,
                    NULL);

        Status = ObReferenceObjectByHandle(
                    pD3dKmPresentInfo->hKmdEvent,
                    EVENT_MODIFY_STATE,
                    *ExEventObjectType,
                    UserMode,
                    (PVOID*)&m_pKmdPresentEvent,
                    NULL);

        if ((m_pUmdPresentEvent == nullptr) ||
            (m_pKmdPresentEvent == nullptr))
        {
            GC_LOG_LOW_MEMORY("Failed to open Present events from UMD.");
            return STATUS_NO_MEMORY;
        }
    }

    //
    // Set up info returned to runtime
    //

    DXGK_CONTEXTINFO* pContextInfo = &pCreateContext->ContextInfo;

    pContextInfo->Caps.DriverManagesResidency = 1;

    if (pCreateContext->Flags.SystemContext || pCreateContext->Flags.SystemProtectedContext)
    {
        pContextInfo->DmaBufferPrivateDataSize = sizeof(GcDmaBufInfo);
    }
    else
    {
        //
        // The private command buffer data from UMD SubmitCommandCb is at
        // the beginning of DMA buffer private data from kernel runtime.
        // This requires that the UMD private data size must not exceed
        // the (DmaBufferPrivateDataSize - sizeof(GcDmaBufInfo)).
        //

        C_ASSERT(PAGE_SIZE > sizeof(GcDmaBufInfo));
        pContextInfo->DmaBufferPrivateDataSize = PAGE_SIZE;
    }

    if (pCreateContext->Flags.GdiContext)
    {
        pContextInfo->DmaBufferSize = GcKmdGlobal::s_GdiDmaBufferSize;
    }
    else
    {
        pContextInfo->DmaBufferSize = GcKmdGlobal::s_DmaBufferSize;
    }

    pContextInfo->DmaBufferSegmentSet = 1 << (APERTURE_SEGMENT_ID - 1);

    if (pCreateContext->Flags.GdiContext)
    {
        pContextInfo->AllocationListSize = DXGK_ALLOCATION_LIST_SIZE_GDICONTEXT;
    }

    return STATUS_SUCCESS;
}

GcKmContext::~GcKmContext()
{
    if (m_pUmdPresentEvent)
    {
        ObDereferenceObject(m_pUmdPresentEvent);
    }

    if (m_pKmdPresentEvent)
    {
        ObDereferenceObject(m_pKmdPresentEvent);
    }

}

NTSTATUS
GcKmContext::Present(
    INOUT_PDXGKARG_PRESENT pPresent)
{
    if (pPresent->Flags.Blt)
    {
        GcDmaBufInfo* pDmaBufInfo = (GcDmaBufInfo*)pPresent->pDmaBufferPrivateData;

        memset(pDmaBufInfo, 0, sizeof(GcDmaBufInfo));

        if (pPresent->pPrivateDriverData)
        {
            if ((m_pUmdPresentEvent == nullptr) ||
                (m_pKmdPresentEvent == nullptr))
            {
                return STATUS_NO_MEMORY;
            }

            D3dKmPresentPrivateData*    pPresentPrivateData = (D3dKmPresentPrivateData*)pPresent->pPrivateDriverData;
            D3dUmBltPresentInfo*        pUmBltPresentInfo = pPresentPrivateData->pUmBltPresentInfo;
            RECT*   pUmDstSubRects = NULL;

            __try
            {
                ProbeForWrite(pUmBltPresentInfo, sizeof(D3dUmBltPresentInfo), sizeof(DWORD));

                if (pPresent->SubRectCnt < (sizeof(pUmBltPresentInfo->RectStash)/sizeof(RECT)))
                {
                    pUmBltPresentInfo->pDstSubRects = pUmBltPresentInfo->RectStash;
                }
                else
                {
                    NTSTATUS    Status;
                    SIZE_T      AllocSize = sizeof(RECT)*pPresent->SubRectCnt;

                    Status = ZwAllocateVirtualMemory(
                                NtCurrentProcess(),
                                (PVOID*)&pUmDstSubRects,
                                0,
                                &AllocSize,
                                MEM_COMMIT,
                                PAGE_READWRITE);
                    if (!NT_SUCCESS(Status))
                    {
                        return STATUS_NO_MEMORY;
                    }

                    pUmBltPresentInfo->pDstSubRects = pUmDstSubRects;
                }

                pUmBltPresentInfo->DstRect = pPresent->DstRect;
                pUmBltPresentInfo->SrcRect = pPresent->SrcRect;

                pUmBltPresentInfo->SubRectCnt = pPresent->SubRectCnt;
                RtlCopyMemory(
                    pUmBltPresentInfo->pDstSubRects,
                    pPresent->pDstSubRects,
                    pPresent->SubRectCnt*sizeof(RECT));

                pUmBltPresentInfo->KmdAckCode = KMD_UM_BLT_PRESENT_ACK;
            }
            __except(EXCEPTION_EXECUTE_HANDLER)
            {
                if (pUmDstSubRects)
                {
                    SIZE_T  AllocSize = 0;

                    ZwFreeVirtualMemory(
                        NtCurrentProcess(),
                        (PVOID*)&pUmDstSubRects,
                        &AllocSize,
                        MEM_RELEASE);
                }

                return STATUS_NO_MEMORY;
            }

            KeSetEvent(m_pUmdPresentEvent, 0, FALSE);

#ifndef DEBUG_UM_BLT_PRESENT
            NTSTATUS    Status;
            LARGE_INTEGER   Timeout;

            Timeout.QuadPart = -10*1000*1000*10;

            Status = KeWaitForSingleObject(m_pKmdPresentEvent, Executive, KernelMode, TRUE, &Timeout);
            if (Status != STATUS_SUCCESS)
            {
                return STATUS_NO_MEMORY;
            }
#endif

            __try
            {
                pDmaBufInfo->m_DmaBufferGpuVa = pUmBltPresentInfo->UmBltCmdBufGpuVa;
                pDmaBufInfo->m_DmaBufferSize = pUmBltPresentInfo->UmBltCmdBufSize;
            }
            __except (EXCEPTION_EXECUTE_HANDLER)
            {
                return STATUS_NO_MEMORY;
            }

#ifndef DEBUG_UM_BLT_PRESENT
            pDmaBufInfo->m_DmaBufState.m_bUmBltPresent = 1;
#endif

            pPresent->pDmaBuffer = ((BYTE*)pPresent->pDmaBuffer) + 0x10;

            return STATUS_SUCCESS;
        }

        //
        // Stretching is not supported
        //

        if (((pPresent->SrcRect.right - pPresent->SrcRect.left) != (pPresent->DstRect.right - pPresent->DstRect.left)) ||
            ((pPresent->SrcRect.bottom - pPresent->SrcRect.top) != (pPresent->DstRect.bottom - pPresent->DstRect.top)))
        {
            NT_ASSERT(false);
            return STATUS_SUCCESS;
        }

        NTSTATUS    Status = STATUS_SUCCESS;
        BYTE*   pDmaBuffer = (BYTE*)pPresent->pDmaBuffer;
        UINT    DmaBufferSize = pPresent->DmaSize;
        BYTE*   pMultipassRestart = NULL;

        DXGK_PRESENTALLOCATIONINFO* pSrcAllocationInfo = pPresent->pAllocationInfo + DXGK_PRESENT_SOURCE_INDEX;
        DXGK_PRESENTALLOCATIONINFO* pDstAllocationInfo = pPresent->pAllocationInfo + DXGK_PRESENT_DESTINATION_INDEX;
        GcKmDeviceAllocation*   pSrcDeviceAllocation = (GcKmDeviceAllocation*)pSrcAllocationInfo->hDeviceSpecificAllocation;
        GcKmDeviceAllocation*   pDstDeviceAllocation = (GcKmDeviceAllocation*)pDstAllocationInfo->hDeviceSpecificAllocation;

        //
        // Set up header new command buffer
        //

        UINT    CommandBufferHeaderSize = GetCommandBufferHeaderSize();
        UINT    CommandBufferFooterSize = GetCommandBufferFooterSize();

        if ((CommandBufferHeaderSize + CommandBufferFooterSize) >= DmaBufferSize)
        {
            return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
        }

        memset(pDmaBuffer, 0, CommandBufferHeaderSize);

        pDmaBuffer += CommandBufferHeaderSize;
        DmaBufferSize -= CommandBufferHeaderSize;

        // Reserve the space for Command buffer footer
        DmaBufferSize -= CommandBufferFooterSize;

        BYTE*   pInitialDmaBuffer = pDmaBuffer;

        Status = CopyImage(
                    pSrcDeviceAllocation->m_pGcKmAllocation,
                    pSrcAllocationInfo->AllocationVirtualAddress,
                    pDstDeviceAllocation->m_pGcKmAllocation,
                    pDstAllocationInfo->AllocationVirtualAddress,
                    &pPresent->SrcRect,
                    &pPresent->DstRect,
                    pPresent->SubRectCnt,
                    pPresent->pDstSubRects,
                    (void**)&pDmaBuffer,
                    &pPresent->DmaBufferGpuVirtualAddress,
                    DmaBufferSize,
                    (BYTE*)pPresent->pDmaBufferPrivateData,
                    pPresent->DmaBufferPrivateDataSize,
                    &pPresent->MultipassOffset);

        //
        // Zero out command buffer footer or back out the header
        //

        if (pDmaBuffer != pInitialDmaBuffer)
        {
            memset(pDmaBuffer, 0, CommandBufferFooterSize);

            pDmaBuffer += CommandBufferFooterSize;
        }
        else
        {
            pDmaBuffer -= CommandBufferHeaderSize;
        }

        //
        // Update DMA buffer pointer
        //

        pPresent->pDmaBuffer = pDmaBuffer;

        return Status;
    }
    else if (pPresent->Flags.Flip)
    {
        return STATUS_SUCCESS;
    }
    else if (pPresent->Flags.ColorFill)
    {
        return STATUS_NOT_SUPPORTED;
    }
    else
    {
        NT_ASSERT(false);

        return STATUS_NOT_SUPPORTED;
    }
}

VOID
GcKmContext::SetRootPageTable(
    IN_CONST_PDXGKARG_SETROOTPAGETABLE  pSetPageTable)
{
    m_RootPageTableAddress = pSetPageTable->Address;
    m_NumRootPageTableEntries = pSetPageTable->NumEntries;
}

NTSTATUS
GcKmContext::RenderGdi(
    INOUT_PDXGKARG_RENDERGDI    pRenderGdi)
{
    NTSTATUS    Status = STATUS_SUCCESS;

    if (0 == pRenderGdi->CommandLength)
    {
        return Status;
    }

    BYTE*   pCurGdiCmd = (BYTE*)pRenderGdi->pCommand;
    BYTE*   pEndGdiCmd = pCurGdiCmd + pRenderGdi->CommandLength;
    BYTE*   pMultipassRect = pCurGdiCmd + pRenderGdi->MultipassOffset;
    DXGK_RENDERKM_COMMAND*  pRenderKmCmd = (DXGK_RENDERKM_COMMAND*)pCurGdiCmd;
    BYTE*   pDmaBuffer = (BYTE*)pRenderGdi->pDmaBuffer;
    UINT    DmaBufferSize = pRenderGdi->DmaSize;
    D3DGPU_VIRTUAL_ADDRESS  DmaBufferGpuVirtualAddress = pRenderGdi->DmaBufferGpuVirtualAddress;
    BYTE*   pMultipassRestart = NULL;
    GcDmaBufInfo*   pDmaBufInfo = (GcDmaBufInfo*)pRenderGdi->pDmaBufferPrivateData;

    XSTATE_SAVE FloatingSave;

    ULONG64 ExtendedFeatures = RtlGetEnabledExtendedFeatures((ULONG64)-1LL);
    Status = KeSaveExtendedProcessorState(ExtendedFeatures, &FloatingSave);

    //
    // Indicate the start of new GDI command buffer
    //

    memset(pDmaBufInfo, 0, sizeof(GcDmaBufInfo));

    pDmaBufInfo->m_DmaBufState.m_bGdi = true;
    pDmaBufInfo->m_DmaBufState.m_bGdiInitialized = false;

    //
    // Set up header for the new command buffer
    //

    UINT    CommandBufferHeaderSize = GetCommandBufferHeaderSize();
    UINT    CommandBufferFooterSize = GetCommandBufferFooterSize();

    if ((CommandBufferHeaderSize + CommandBufferFooterSize) >= DmaBufferSize)
    {
        return STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER;
    }

    memset(pDmaBuffer, 0, CommandBufferHeaderSize);

    pDmaBuffer += CommandBufferHeaderSize;
    DmaBufferGpuVirtualAddress += CommandBufferHeaderSize;
    DmaBufferSize -= CommandBufferHeaderSize;

    //
    // Reserve the space for Command buffer footer
    //

    DmaBufferSize -= CommandBufferFooterSize;

    BYTE*   pInitialDmaBuffer = pDmaBuffer;

    for (; pCurGdiCmd < pEndGdiCmd; pCurGdiCmd += pRenderKmCmd->CommandSize)
    {
        pRenderKmCmd = (DXGK_RENDERKM_COMMAND*)pCurGdiCmd;
        BYTE*   pCurGdiCmdEnd = pCurGdiCmd + pRenderKmCmd->CommandSize;
        BYTE*   pCurDmaBuffer = pDmaBuffer;
        UINT    MultipassOffset;

        if (pCurGdiCmdEnd < pMultipassRect)
        {
            continue;
        }

        MultipassOffset = 0;

        switch (pRenderKmCmd->OpCode)
        {
        case DXGK_GDIOP_BITBLT:
        {
            DXGK_GDIARG_BITBLT* pCmdBitBlt = &pRenderKmCmd->Command.BitBlt;

            NT_ASSERT(((BYTE*)(pCmdBitBlt->pSubRects + pCmdBitBlt->NumSubRects)) <= pCurGdiCmdEnd);

            if (pCurGdiCmd < pMultipassRect)
            {
                MultipassOffset = (UINT)(((RECT*)pMultipassRect) - pCmdBitBlt->pSubRects);
            }

            Status = GdiBitBlt(
                        pCmdBitBlt,
                        pRenderGdi->pAllocationList,
                        pRenderGdi->AllocationListSize,
                        &pDmaBuffer,
                        &DmaBufferGpuVirtualAddress,
                        DmaBufferSize,
                        (BYTE*)pRenderGdi->pDmaBufferPrivateData,
                        pRenderGdi->DmaBufferPrivateDataSize,
                        &MultipassOffset);

            if (STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER == Status)
            {
                pMultipassRestart = (BYTE*)(pCmdBitBlt->pSubRects + MultipassOffset);
            }
        }
        break;

        case DXGK_GDIOP_COLORFILL:
        {
            DXGK_GDIARG_COLORFILL*  pCmdColorFill = &pRenderKmCmd->Command.ColorFill;

            NT_ASSERT(((BYTE*)(pCmdColorFill->pSubRects + pCmdColorFill->NumSubRects)) <= pCurGdiCmdEnd);

            if (pCurGdiCmd < pMultipassRect)
            {
                MultipassOffset = (UINT)(((RECT*)pMultipassRect) - pCmdColorFill->pSubRects);
            }

            Status = GdiColorFill(
                        pCmdColorFill,
                        pRenderGdi->pAllocationList,
                        pRenderGdi->AllocationListSize,
                        &pDmaBuffer,
                        &DmaBufferGpuVirtualAddress,
                        DmaBufferSize,
                        (BYTE*)pRenderGdi->pDmaBufferPrivateData,
                        pRenderGdi->DmaBufferPrivateDataSize,
                        &MultipassOffset);

            if (STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER == Status)
            {
                pMultipassRestart = (BYTE*)(pCmdColorFill->pSubRects + MultipassOffset);
            }
        }
        break;

        case DXGK_GDIOP_ALPHABLEND:
        {
            DXGK_GDIARG_ALPHABLEND* pCmdAlphaBlend = &pRenderKmCmd->Command.AlphaBlend;

            NT_ASSERT(((BYTE*)(pCmdAlphaBlend->pSubRects + pCmdAlphaBlend->NumSubRects)) <= pCurGdiCmdEnd);

            if (pCurGdiCmd < pMultipassRect)
            {
                MultipassOffset = (UINT)(((RECT*)pMultipassRect) - pCmdAlphaBlend->pSubRects);
            }

            Status = GdiAlphaBlend(
                        pCmdAlphaBlend,
                        pRenderGdi->pAllocationList,
                        pRenderGdi->AllocationListSize,
                        &pDmaBuffer,
                        &DmaBufferGpuVirtualAddress,
                        DmaBufferSize,
                        (BYTE*)pRenderGdi->pDmaBufferPrivateData,
                        pRenderGdi->DmaBufferPrivateDataSize,
                        &MultipassOffset);

            if (STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER == Status)
            {
                pMultipassRestart = (BYTE*)(pCmdAlphaBlend->pSubRects + MultipassOffset);
            }
        }
        break;

        case DXGK_GDIOP_STRETCHBLT:
        {
            DXGK_GDIARG_STRETCHBLT* pCmdStretchBlt = &pRenderKmCmd->Command.StretchBlt;

            NT_ASSERT(((BYTE*)(pCmdStretchBlt->pSubRects + pCmdStretchBlt->NumSubRects)) <= pCurGdiCmdEnd);

            if (pCurGdiCmd < pMultipassRect)
            {
                MultipassOffset = (UINT)(((RECT*)pMultipassRect) - pCmdStretchBlt->pSubRects);
            }

            Status = GdiStretchBlt(
                        pCmdStretchBlt,
                        pRenderGdi->pAllocationList,
                        pRenderGdi->AllocationListSize,
                        &pDmaBuffer,
                        &DmaBufferGpuVirtualAddress,
                        DmaBufferSize,
                        (BYTE*)pRenderGdi->pDmaBufferPrivateData,
                        pRenderGdi->DmaBufferPrivateDataSize,
                        &MultipassOffset);

            if (STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER == Status)
            {
                pMultipassRestart = (BYTE*)(pCmdStretchBlt->pSubRects + MultipassOffset);
            }
        }
        break;

        case DXGK_GDIOP_ESCAPE:
            break;

        case DXGK_GDIOP_TRANSPARENTBLT:
        {
            DXGK_GDIARG_TRANSPARENTBLT* pCmdTransparentBlt = &pRenderKmCmd->Command.TransparentBlt;

            NT_ASSERT(((BYTE*)(pCmdTransparentBlt->pSubRects + pCmdTransparentBlt->NumSubRects)) <= pCurGdiCmdEnd);

            if (pCurGdiCmd < pMultipassRect)
            {
                MultipassOffset = (UINT)(((RECT*)pMultipassRect) - pCmdTransparentBlt->pSubRects);
            }

            Status = GdiTransparentBlt(
                        pCmdTransparentBlt,
                        pRenderGdi->pAllocationList,
                        pRenderGdi->AllocationListSize,
                        &pDmaBuffer,
                        &DmaBufferGpuVirtualAddress,
                        DmaBufferSize,
                        (BYTE*)pRenderGdi->pDmaBufferPrivateData,
                        pRenderGdi->DmaBufferPrivateDataSize,
                        &MultipassOffset);

            if (STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER == Status)
            {
                pMultipassRestart = (BYTE*)(pCmdTransparentBlt->pSubRects + MultipassOffset);
            }
        }
        break;

        case DXGK_GDIOP_CLEARTYPEBLEND:
        {
            DXGK_GDIARG_CLEARTYPEBLEND* pCmdClearTypeBlend = &pRenderKmCmd->Command.ClearTypeBlend;

            NT_ASSERT(((BYTE*)(pCmdClearTypeBlend->pSubRects + pCmdClearTypeBlend->NumSubRects)) <= pCurGdiCmdEnd);

            if (pCurGdiCmd < pMultipassRect)
            {
                MultipassOffset = (UINT)(((RECT*)pMultipassRect) - pCmdClearTypeBlend->pSubRects);
            }

            Status = GdiClearTypeBlend(
                        pCmdClearTypeBlend,
                        pRenderGdi->pAllocationList,
                        pRenderGdi->AllocationListSize,
                        &pDmaBuffer,
                        &DmaBufferGpuVirtualAddress,
                        DmaBufferSize,
                        (BYTE*)pRenderGdi->pDmaBufferPrivateData,
                        pRenderGdi->DmaBufferPrivateDataSize,
                        &MultipassOffset);

            if (STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER == Status)
            {
                pMultipassRestart = (BYTE*)(pCmdClearTypeBlend->pSubRects + MultipassOffset);
            }
        }
        break;

        default:
            NT_ASSERT(false);
            Status = STATUS_INVALID_PARAMETER;
            break;
        }

        DmaBufferSize -= (UINT)(pDmaBuffer - pCurDmaBuffer);

        if (STATUS_SUCCESS != Status)
        {
            break;
        }
    }

    if (STATUS_GRAPHICS_INSUFFICIENT_DMA_BUFFER == Status)
    {
        pRenderGdi->MultipassOffset = (UINT)(pMultipassRestart - (BYTE*)pRenderGdi->pCommand);
    }

    //
    // Zero out command buffer footer or back out the header
    //

    if (pDmaBuffer != pInitialDmaBuffer)
    {
        memset(pDmaBuffer, 0, CommandBufferFooterSize);

        pDmaBuffer += CommandBufferFooterSize;
    }
    else
    {
        pDmaBuffer -= CommandBufferHeaderSize;
    }

    //
    // Update DMA buffer pointer
    //

    pRenderGdi->pDmaBuffer = pDmaBuffer;

    KeRestoreExtendedProcessorState(&FloatingSave);

    return Status;
}

NTSTATUS
GcKmContext::CopyImage(
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
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmContext::GdiBitBlt(
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
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmContext::GdiColorFill(
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
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmContext::GdiAlphaBlend(
    DXGK_GDIARG_ALPHABLEND* pCmdAlphaBlt,
    DXGK_ALLOCATIONLIST*    pAllocationList,
    UINT                    AllocationListSize,
    BYTE**                  ppDmaBuffer,
    D3DGPU_VIRTUAL_ADDRESS* pDmaBufferGpuVirtualAddress,
    UINT                    DmaBufferSize,
    BYTE*                   pDmaBufferPrivateData,
    UINT                    DmaBufferPrivateDataSize,
    UINT*                   pMultipassOffset)
{
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmContext::GdiStretchBlt(
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
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmContext::GdiTransparentBlt(
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
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmContext::GdiClearTypeBlend(
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
    return STATUS_SUCCESS;
}

