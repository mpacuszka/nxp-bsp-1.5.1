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
#include "GcKmd7LEngine.tmh"

#include "GcKmd7LEngine.h"

#include "GcKmdUtil.h"
#include "GcKmd7LUtil.h"

#include "GcKmd7LContext.h"

extern "C"
{


};

GcKm7L3DEngine::GcKm7L3DEngine(
    bool   *pbResetRequested,
    GcKmMmu    *pMmu)
{
    for (UINT i = 0; i < ARRAYSIZE(m_GlobalContexts); i++)
    {
        m_GlobalContexts[i] = NULL;
        m_GlobalContextHandles[i] = 0;
    }

    m_nGlobalContextForSubmission = 0;

    //
    // Initialize last context used for submission
    //

    m_pLastContextForSubmission = NULL;

    m_pDmaBufCompletionSignal = NULL;

    m_pbResetRequested = pbResetRequested;

    m_pMmu = pMmu;
}

GcKm7L3DEngine::~GcKm7L3DEngine()
{
}

NTSTATUS
GcKm7L3DEngine::Start()
{
    gceSTATUS   GcStatus = gcvSTATUS_OK;

    do
    {
        gckKERNEL   kernel = galDevice->kernels[0];

        for (UINT i = 0; i < ARRAYSIZE(m_GlobalContexts); i++)
        {
            GcStatus = gckCONTEXT_Construct(
                        kernel->os,
                        kernel->hardware,
                        0,
                        &m_GlobalContexts[i]);
            if (gcvSTATUS_OK != GcStatus)
            {
                break;
            }

            m_GlobalContextHandles[i] = gcmPTR_TO_NAME(m_GlobalContexts[i]);

            if (0 == m_GlobalContextHandles[i])
            {
                GcStatus = gcvSTATUS_OUT_OF_MEMORY;
                break;
            }
        }

        if (gcvSTATUS_OK != GcStatus)
        {
            break;
        }

        GcStatus = gckOS_CreateSignal(
                    galDevice->os,
                    FALSE,
                    &m_pDmaBufCompletionSignal);

    } while (FALSE);

    return gcvSTATUS_OK == GcStatus ? STATUS_SUCCESS : STATUS_NO_MEMORY;
}

void
GcKm7L3DEngine::Stop()
{
    for (UINT i = 0; i < ARRAYSIZE(m_GlobalContexts); i++)
    {
        if (0 != m_GlobalContextHandles[i])
        {
            gckKERNEL   kernel = galDevice->kernels[0];

            gcmRELEASE_NAME(m_GlobalContextHandles[i]);
        }

        if (NULL != m_GlobalContexts[i])
        {
            gckCONTEXT_Destroy(m_GlobalContexts[i]);
        }
    }

    if (m_pDmaBufCompletionSignal)
    {
        gckOS_DestroySignal(
            galDevice->os,
            m_pDmaBufCompletionSignal);
    }
}

BOOLEAN g_bAllowForcedContextSwitch = FALSE;

NTSTATUS
GcKm7L3DEngine::SubmitCommandBuffer(
    D3DGPU_VIRTUAL_ADDRESS  DmaBufGpuVa,
    UINT                    DmaBufSize,
    GcKmContext            *pContext,
    ULONGLONG               RootPhysicalAddress,
    BOOLEAN                 bForceContextSwitch)
{
#if 1
    GcKm7LPageTableWalker2  GpuPageTableWalker(RootPhysicalAddress, m_pMmu);
#else
    //
    // Use the global page table
    //

    GcKm7LPageTableWalker   GpuPageTableWalker(galDevice->kernels[0]->mmu->mtlbPhysical);
#endif
    GcKm7LGpuMemMapper  GpuMemMapper(&GpuPageTableWalker);
    PBYTE   pDmaBuf;

    NT_ASSERT(DmaBufGpuVa);
    NT_ASSERT(DmaBufSize);

    pDmaBuf = GpuMemMapper.MapToCpuVa((UINT)DmaBufGpuVa, DmaBufSize);
    if (NULL == pDmaBuf)
    {
        return STATUS_NO_MEMORY;
    }

    gceSTATUS   GcStatus;
    gcsHAL_COMMIT   Commit = { 0 };
    gcsHAL_COMMAND_LOCATION    *pCommandBuffer = &Commit.subCommit.commandBuffer;

    pCommandBuffer->address = (gctUINT32)DmaBufGpuVa;
    pCommandBuffer->logical = (gctUINT64)pDmaBuf;
    pCommandBuffer->startOffset = 0;
    pCommandBuffer->size = DmaBufSize;
    pCommandBuffer->reservedHead = 0x20;
    pCommandBuffer->reservedTail = 0x20;

    GcKm7LContext* pKm7LContext = static_cast<GcKm7LContext*>(pContext);
    D3DGPU_VIRTUAL_ADDRESS ShadowDmaBufferGpuVa = pKm7LContext->GetShadowDmaBufferGpuVa();
    if (ShadowDmaBufferGpuVa)
    {
        void* pShadowDmaBuffer = pKm7LContext->GetShadowDmaBufferCpuVa();

        NT_ASSERT(pShadowDmaBuffer);
        if (pShadowDmaBuffer)
        {
            memcpy(pShadowDmaBuffer, pDmaBuf, DmaBufSize);
            // overwrite to shadow buffer
            pCommandBuffer->address = (gctUINT32)ShadowDmaBufferGpuVa;
            pCommandBuffer->logical = (gctUINT64)pShadowDmaBuffer;
        }
    }

    bForceContextSwitch &= g_bAllowForcedContextSwitch;

    //
    // Alternate the global context to set HW state back to default
    // for submission on a new WDDM context
    //

    if ((pContext != m_pLastContextForSubmission) || bForceContextSwitch)
    {
        m_nGlobalContextForSubmission++;
        if (m_nGlobalContextForSubmission >= ARRAYSIZE(m_GlobalContexts))
        {
            m_nGlobalContextForSubmission = 0;
        }
    }

    Commit.subCommit.context = m_GlobalContextHandles[m_nGlobalContextForSubmission];

    //
    // Commit.commitStamp is the timestamp returned from Ring Buffer
    //

    GcStatus = _Commit(
                galDevice->device,
                gcvHARDWARE_3D,
                gcvENGINE_RENDER,
                0,
                &Commit);

    LARGE_INTEGER   DmaBufferCommitTime;
    KeQuerySystemTime(&DmaBufferCommitTime);

    //
    // Record the last conext used for submission
    //

    m_pLastContextForSubmission = pContext;

    if (gcvSTATUS_OK != GcStatus)
    {
        return STATUS_UNSUCCESSFUL;
    }

    //
    // Wait for the command buffer to complete
    //
    // The event is signaled when command buffer completion interrupt is handled
    //

    static UINT s_MaxWaitCount = 0;
    UINT    WaitCount = 0;
    BOOLEAN bSubmitEvent = true;

    do
    {
        if (*m_pbResetRequested)
        {
            return STATUS_TIMEOUT;
        }

        if (bSubmitEvent)
        {
            GcStatus = gckOS_Signal(
                        galDevice->os,
                        m_pDmaBufCompletionSignal,
                        gcvFALSE);
            NT_ASSERT(gcvSTATUS_OK == GcStatus);

            GcStatus = gckEVENT_Signal(
                        galDevice->kernels[0]->eventObj,
                        m_pDmaBufCompletionSignal,
                        gcvKERNEL_PIXEL);
            NT_ASSERT(gcvSTATUS_OK == GcStatus);

            GcStatus = gckEVENT_Commit(
                        galDevice->kernels[0]->eventObj,
                        0,
                        FALSE);
            NT_ASSERT(gcvSTATUS_OK == GcStatus);

            GcStatus = gckOS_WaitSignal(
                        galDevice->os,
                        m_pDmaBufCompletionSignal,
                        FALSE,
                        2*1000);    // Standard 2 seconds of TDR timeout

            WaitCount++;
            if (WaitCount > s_MaxWaitCount)
            {
                s_MaxWaitCount = WaitCount;
            }

            switch (GcStatus)
            {
            case gcvSTATUS_OK:
                break;
            case gcvSTATUS_TIMEOUT:
                bSubmitEvent = false;
                break;
            default:
                return STATUS_UNSUCCESSFUL;
            }
        }
        else
        {
            //
            // Continue to wait for TDR settings:
            // https://docs.microsoft.com/en-us/windows-hardware/drivers/display/tdr-registry-keys
            //

            LARGE_INTEGER   Delay;

            Delay.QuadPart = -500*1000*10L;
            KeDelayExecutionThread(KernelMode, FALSE, &Delay);
        }

        gctUINT64   FenceValue = *(volatile gctUINT64 *)(galDevice->kernels[0]->command->fence->logical);
        if (Commit.commitStamp <= FenceValue)
        {
            break;
        }

    } while (TRUE);

#if 0
    //
    // Check if Pixel and 3D Blt engines are idle as expected
    //

    volatile UINT   *pRegBase = (volatile UINT *)galDevice->registerBases[0];
    UINT    IdleBits = pRegBase[0x0001];
    UINT    PeAndBltIdle = (1 << 2) | (1 << 12);

    NT_ASSERT((IdleBits & PeAndBltIdle) == PeAndBltIdle);
#endif

    //
    // GcKm7LGpuMemMapper::~GcKm7LGpuMemMapper() unmaps DMA buffer
    //

    return STATUS_SUCCESS;
}

NTSTATUS
GcKm7L3DEngine::CpuTransfer(
    ULONGLONG               RootPhysicalAddress,
    DXGK_BUILDPAGINGBUFFER_TRANSFERVIRTUAL* pTransferVirtual)
{
    GcKm7LPageTableWalker2  GpuPageTableWalker(RootPhysicalAddress, m_pMmu);
    GcKm7LGpuMemMapper  GpuMemMapperSource(&GpuPageTableWalker);
    GcKm7LGpuMemMapper  GpuMemMapperDest(&GpuPageTableWalker);

    ULONG RemainingSize = static_cast<UINT>(pTransferVirtual->TransferSizeInBytes);
    UINT SourceVirtualAddress = static_cast<UINT>(pTransferVirtual->SourceVirtualAddress);
    UINT DestinationVirtualAddress = static_cast<UINT>(pTransferVirtual->DestinationVirtualAddress);
    // MapToCpuVa has a limit on how much it can map at one time
    UINT MaxSizeSupportedByMapFn = (sizeof(GcKmMdl64Pages::m_PageNumbers)/sizeof(GcKmMdl64Pages::m_PageNumbers[0])) * PAGE_SIZE;

    // TODO: Should we dynamically switch between CPU and GPU based on whether we're moving
    //       anything above 4GB or not?
    while (RemainingSize)
    {
        UINT TransferSize = min(RemainingSize, MaxSizeSupportedByMapFn);

        PBYTE pSource = GpuMemMapperSource.MapToCpuVa(SourceVirtualAddress, TransferSize);
        if (NULL == pSource)
        {
            NT_ASSERT(0);
            return STATUS_NO_MEMORY;
        }
        PBYTE pDest = GpuMemMapperDest.MapToCpuVa(DestinationVirtualAddress, TransferSize);
        if (NULL == pDest)
        {
            NT_ASSERT(0);
            return STATUS_NO_MEMORY;
        }

        memcpy(pDest, pSource, TransferSize);

        SourceVirtualAddress += TransferSize;
        DestinationVirtualAddress += TransferSize;
        RemainingSize -= TransferSize;
    }

    return STATUS_SUCCESS;
}


GcKmCopyEngine::GcKmCopyEngine()
{
}

GcKmCopyEngine::~GcKmCopyEngine()
{
}

NTSTATUS
GcKmCopyEngine::Start()
{
    return STATUS_SUCCESS;
}

void
GcKmCopyEngine::Stop()
{
}

NTSTATUS
GcKmCopyEngine::SubmitCommandBuffer(
    D3DGPU_VIRTUAL_ADDRESS  DmaBufGpuVa,
    UINT                    DmaBufSize,
    GcKmContext            *pContext,
    ULONGLONG               RootPhysicalAddress,
    BOOLEAN                 bForceContextSwitch)
{
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmCopyEngine::CpuTransfer(
    ULONGLONG               RootPhysicalAddress,
    DXGK_BUILDPAGINGBUFFER_TRANSFERVIRTUAL* pTransferVirtual)
{
    return STATUS_SUCCESS;
}


