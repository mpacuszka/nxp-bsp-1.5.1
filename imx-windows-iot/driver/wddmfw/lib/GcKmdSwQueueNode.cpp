/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmdSwQueueNode.tmh"

#include "GcKmdSwQueueNode.h"
#include "GcKmdContext.h"

#include "GcKmdUtil.h"


NTSTATUS
GcKmSwQueueNode::Start(
    IN_PDXGKRNL_INTERFACE   pDxgkInterface)
{
    m_DxgkInterface = *pDxgkInterface;

    NTSTATUS Status;

    Status = m_pHwEngine->Start();
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    //
    // Initialize work thread event
    //

    KeInitializeEvent(&m_WorkerThreadEvent, SynchronizationEvent, FALSE);
    m_bPreemptionRequested = false;

    //
    // Intialize DMA buffer queue and lock
    //

    InitializeListHead(&m_DmaBufSubmissionFree);
    for (UINT i = 0; i < m_MaxDmaBufQueueLength; i++)
    {
        InsertHeadList(&m_DmaBufSubmissionFree, &m_DmaBufSubmissions[i].m_QueueEntry);
    }

    InitializeListHead(&m_DmaBufQueue);
    KeInitializeSpinLock(&m_DmaBufQueueLock);

    //
    // Initialize HW DMA buffer compeletion DPC and event
    //

    KeInitializeEvent(&m_HwDmaBufCompletionEvent, SynchronizationEvent, FALSE);
    KeInitializeDpc(&m_HwDmaBufCompletionDpc, HwDmaBufCompletionDpcRoutine, this);

    //
    // Initialize Fence IDs
    //

    m_LastSubmittedFenceId = 0;
    m_LastProcessedFenceId = 0;
    m_LastCompletetdFenceId = 0;

    m_LastCompeletedPreemptionFenceId = 0;

    //
    // Initialize TDR related fields
    //

    KeInitializeEvent(&m_ResetCompletionEvent, SynchronizationEvent, FALSE);
    m_bResetRequested = false;
    m_bWaitingForReset = false;

    m_bWorkerExit = false;

    //
    // Initialize worker thread
    //

    OBJECT_ATTRIBUTES   ObjectAttributes;
    HANDLE  hWorkerThread;

    InitializeObjectAttributes(&ObjectAttributes, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);

    Status = PsCreateSystemThread(
                &hWorkerThread,
                THREAD_ALL_ACCESS,
                &ObjectAttributes,
                NULL,
                NULL,
                (PKSTART_ROUTINE)GcKmSwQueueNode::WorkerThread,
                this);

    if (Status != STATUS_SUCCESS)
    {
        GC_LOG_ERROR(
            "PsCreateSystemThread(...) failed for GcKmSwQueueNode::WorkerThread. (Status=%!STATUS!)",
            Status);
        return Status;
    }

    Status = ObReferenceObjectByHandle(
                hWorkerThread,
                THREAD_ALL_ACCESS,
                *PsThreadType,
                KernelMode,
                (PVOID*)&m_pWorkerThread,
                NULL);

    ZwClose(hWorkerThread);

    if (!NT_SUCCESS(Status))
    {
        GC_LOG_ERROR(
            "ObReferenceObjectByHandle(...) failed for worker thread. (Status=%!STATUS!)",
            Status);
        return Status;
    }

    GC_LOG_TRACE("Node was successfully started.");

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmSwQueueNode::Stop()
{
    if (!m_pWorkerThread)
    {
        return STATUS_SUCCESS;
    }

    m_bWorkerExit = true;

    KeSetEvent(&m_WorkerThreadEvent, 0, FALSE);

    NTSTATUS Status;
        
    Status = KeWaitForSingleObject(
                m_pWorkerThread,
                Executive,
                KernelMode,
                FALSE,
                NULL);

    NT_ASSERT(Status == STATUS_SUCCESS);

    ObDereferenceObject(m_pWorkerThread);

    GC_LOG_TRACE("Node was successfully stopped.");

    m_pHwEngine->Stop();

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmSwQueueNode::SubmitCommandVirtual(
    IN_CONST_PDXGKARG_SUBMITCOMMANDVIRTUAL pSubmitCommandVirtual)
{
    QueueDmaBuffer(pSubmitCommandVirtual);

    //
    // Wake up the worker thread for the GPU node
    //

    KeSetEvent(&m_WorkerThreadEvent, 0, FALSE);

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmSwQueueNode::Preempt(
    IN_CONST_PDXGKARG_PREEMPTCOMMAND pPreemptCommand)
{
    m_PreemptCommand = *pPreemptCommand;

    //
    // Wake up the worker thread for the GPU node to handle preemption
    //

    m_bPreemptionRequested = true;
    KeSetEvent(&m_WorkerThreadEvent, 0, FALSE);

    return STATUS_SUCCESS;
}

void
GcKmSwQueueNode::PrepareToReset()
{
    //
    // Request the worker thread to reset
    //

    m_bResetRequested = true;
    KeSetEvent(&m_WorkerThreadEvent, 0, FALSE);

    KeWaitForSingleObject(
        &m_ResetCompletionEvent,
        Executive,
        KernelMode,
        FALSE,          // Alertable
        NULL);

    //
    // Implicitly sync up : Graphics runtime considers all submitted Fence Id as completed.
    //

    m_LastCompletetdFenceId = m_LastProcessedFenceId;

    m_bWaitingForReset = false;
}

void
GcKmSwQueueNode::WorkerThread(
    void   *pThis)
{
    GcKmSwQueueNode    *pGcKmSwQueueNode = GcKmSwQueueNode::Cast(pThis);

    pGcKmSwQueueNode->DoWork();
}

void
GcKmSwQueueNode::DoWork(void)
{
    bool    bDone = false;

    while (!bDone)
    {
        NTSTATUS    Status;

        Status = KeWaitForSingleObject(
                    &m_WorkerThreadEvent,
                    Executive,
                    KernelMode,
                    FALSE,          // Alertable
                    NULL);
        NT_ASSERT(Status == STATUS_WAIT_0);

        for (;;)
        {
            if (m_bResetRequested)
            {
                m_bResetRequested = false;

                EmptyDmaBufferQueue(false);

                //
                // Clear pending preemption request
                //

                m_bPreemptionRequested = false;

                //
                // Signal back to the waiting DDI thread
                //

                KeSetEvent(&m_ResetCompletionEvent, 0, FALSE);

                break;
            }

            if (m_bPreemptionRequested)
            {
                m_bPreemptionRequested = false;

                EmptyDmaBufferQueue(true);

                //
                // Notify completion of Preemption request
                //

                NotifyPreemptionCompletion();

                break;
            }

            if (m_bWorkerExit)
            {
                bDone = true;
                break;
            }

            if (m_bWaitingForReset)
            {
                break;
            }

            GcDmaBufSubmission *pDmaBufSubmission = DequeueDmaBuffer(&m_DmaBufQueueLock);
            if (pDmaBufSubmission == NULL)
            {
                break;
            }

            GcDmaBufInfo* pDmaBufInfo = pDmaBufSubmission->m_pDmaBufInfo;

            pDmaBufInfo->m_DmaBufState.m_bRun = 1;

            m_LastProcessedFenceId = pDmaBufSubmission->m_SubmissionFenceId;

            do
            {
                //
                // Complete empty DMA buffe directly which references backbuffer 
                // to activate Deferred Wait of GPU synchronization object
                //

                if ((0 == pDmaBufInfo->m_NumCommandBufferChunks) &&
                    ((pDmaBufInfo->m_DmaBufferGpuVa == 0) ||
                     (pDmaBufInfo->m_DmaBufferSize < 0x40)))
                {
                    break;
                }

                D3DGPU_PHYSICAL_ADDRESS RootPageTable;

                pDmaBufSubmission->m_pContext->GetRootPageTable(&RootPageTable);

                m_pMmu->UpdateHwPageTable(pDmaBufSubmission->m_pContext);

                Status = m_pMmu->SwitchGpuVaSpace(&RootPageTable);
                if (!NT_SUCCESS(Status))
                {
                    break;
                }

                if (pDmaBufInfo->m_DmaBufState.m_bPaging && pDmaBufInfo->m_DmaBufState.m_bCpuTransfer)
                {
                    // Need to handle paging using CPU
                    Status = m_pHwEngine->CpuTransfer(m_pMmu->GetPhysicalAddress(&RootPageTable), 
                                                      &pDmaBufInfo->m_TransferVirtual);
                    break;
                }

                if (pDmaBufInfo->m_NumCommandBufferChunks)
                {
                    CommandBufferChunk* pCommandBufferChunk = pDmaBufInfo->m_pCommandBufferChunks;

                    for (UINT i = 0; i < pDmaBufInfo->m_NumCommandBufferChunks; i++, pCommandBufferChunk++)
                    {
                        if ((pCommandBufferChunk->m_commandBufferChunkGpuVa == 0) ||
                            (pCommandBufferChunk->m_commandBufferChunkSize < 0x40))
                        {
                            continue;
                        }

                        Status = m_pHwEngine->SubmitCommandBuffer(
                                                pCommandBufferChunk->m_commandBufferChunkGpuVa,
                                                pCommandBufferChunk->m_commandBufferChunkSize,
                                                pDmaBufSubmission->m_pContext,
                                                m_pMmu->GetPhysicalAddress(&RootPageTable),
                                                false);

                        if (!NT_SUCCESS(Status))
                        {
                            break;
                        }
                    }
                }
                else
                {
                    Status = m_pHwEngine->SubmitCommandBuffer(
                                            pDmaBufInfo->m_DmaBufferGpuVa,
                                            pDmaBufInfo->m_DmaBufferSize,
                                            pDmaBufSubmission->m_pContext,
                                            m_pMmu->GetPhysicalAddress(&RootPageTable),
                                            pDmaBufInfo->m_DmaBufState.m_bGdiSamplerUsed);
                }
            } while (FALSE);

            if (STATUS_SUCCESS == Status)
            {
                // Report back to VidSch that DMA buffer has successfully completed
                NotifyDmaBufCompletion(pDmaBufSubmission);
            }

            KIRQL OldIrql;

            KeAcquireSpinLock(&m_DmaBufQueueLock, &OldIrql);
            InsertTailList(&m_DmaBufSubmissionFree, &pDmaBufSubmission->m_QueueEntry);
            KeReleaseSpinLock(&m_DmaBufQueueLock, OldIrql);

            if (!((STATUS_SUCCESS == Status) || (STATUS_TIMEOUT == Status)))
            {
                m_bWaitingForReset = true;

                break;
            }
        }
    }
}

GcDmaBufSubmission*
GcKmSwQueueNode::DequeueDmaBuffer(
    KSPIN_LOCK *pDmaBufQueueLock)
{
    LIST_ENTRY *pQueueEntry;
    KIRQL OldIrql = {};

    if (pDmaBufQueueLock)
    {
        KeAcquireSpinLock(pDmaBufQueueLock, &OldIrql);
    }

    pQueueEntry = IsListEmpty(&m_DmaBufQueue) ? NULL : RemoveHeadList(&m_DmaBufQueue);

    if (pDmaBufQueueLock)
    {
        KeReleaseSpinLock(pDmaBufQueueLock, OldIrql);
    }

    return pQueueEntry ? CONTAINING_RECORD(pQueueEntry, GcDmaBufSubmission, m_QueueEntry) : NULL;
}

void
GcKmSwQueueNode::EmptyDmaBufferQueue(
    bool bForPreemption)
{
    KIRQL   OldIrql;

    KeAcquireSpinLock(&m_DmaBufQueueLock, &OldIrql);

    LIST_ENTRY* pQueueEntry;
    GcDmaBufSubmission* pDmaBufSubmission;
    BOOLEAN bHasEntry = false;

    while (!IsListEmpty(&m_DmaBufQueue))
    {
        pQueueEntry = RemoveHeadList(&m_DmaBufQueue);

        pDmaBufSubmission = CONTAINING_RECORD(pQueueEntry, GcDmaBufSubmission, m_QueueEntry);

        if (!bHasEntry)
        {
            bHasEntry = true;
            m_LastProcessedFenceId = pDmaBufSubmission->m_SubmissionFenceId;
        }

        if (bForPreemption)
        {
            pDmaBufSubmission->m_pDmaBufInfo->m_DmaBufState.m_bPreempted = 1;
        }
        else
        {
            pDmaBufSubmission->m_pDmaBufInfo->m_DmaBufState.m_bReset = 1;
        }

        InsertTailList(&m_DmaBufSubmissionFree, pQueueEntry);
    }

    KeReleaseSpinLock(&m_DmaBufQueueLock, OldIrql);
}

void
GcKmSwQueueNode::NotifyDmaBufCompletion(
    GcDmaBufSubmission *pDmaBufSubmission)
{
    GcDmaBufInfo   *pDmaBufInfo = pDmaBufSubmission->m_pDmaBufInfo;

    if (!pDmaBufInfo->m_DmaBufState.m_bPaging)
    {
        pDmaBufInfo->m_DmaBufState.m_bCompleted = 1;
    }

    //
    // Notify VidSch completion of the DMA buffer
    //
    NTSTATUS    Status;

    RtlZeroMemory(&m_InterruptData, sizeof(m_InterruptData));

    m_InterruptData.InterruptType = DXGK_INTERRUPT_DMA_COMPLETED;

    m_InterruptData.DmaCompleted.SubmissionFenceId = pDmaBufSubmission->m_SubmissionFenceId;
    m_InterruptData.DmaCompleted.NodeOrdinal = pDmaBufSubmission->m_NodeOrdinal;
    m_InterruptData.DmaCompleted.EngineOrdinal = pDmaBufSubmission->m_EngineOrdinal;

    BOOLEAN bRet;

    Status = m_DxgkInterface.DxgkCbSynchronizeExecution(
                                m_DxgkInterface.DeviceHandle,
                                SynchronizeNotifyInterrupt,
                                this,
                                0,
                                &bRet);
    if (!NT_SUCCESS(Status))
    {
        m_ErrorHit.m_NotifyDmaBufCompletion = 1;
    }

    //
    // Keep track of last completed fence ID for Preemption request afterward
    //

    m_LastCompletetdFenceId = pDmaBufSubmission->m_SubmissionFenceId;
}

void
GcKmSwQueueNode::NotifyPreemptionCompletion()
{
    //
    // Notify the VidSch of the completion of the Preemption request
    //

    RtlZeroMemory(&m_InterruptData, sizeof(m_InterruptData));

    m_InterruptData.InterruptType = DXGK_INTERRUPT_DMA_PREEMPTED;
    m_InterruptData.DmaPreempted.PreemptionFenceId = m_PreemptCommand.PreemptionFenceId;
    m_InterruptData.DmaPreempted.LastCompletedFenceId = m_LastCompletetdFenceId;
    m_InterruptData.DmaPreempted.NodeOrdinal = m_PreemptCommand.NodeOrdinal;
    m_InterruptData.DmaPreempted.EngineOrdinal = m_PreemptCommand.EngineOrdinal;

    m_LastCompeletedPreemptionFenceId = m_PreemptCommand.PreemptionFenceId;

    BOOLEAN     bRet;
    NTSTATUS    Status;

    Status = m_DxgkInterface.DxgkCbSynchronizeExecution(
                                m_DxgkInterface.DeviceHandle,
                                SynchronizeNotifyInterrupt,
                                this,
                                0,
                                &bRet);
    if (!NT_SUCCESS(Status))
    {
        m_ErrorHit.m_NotifyPreemptionCompletion = 1;
    }
}

BOOLEAN
GcKmSwQueueNode::SynchronizeNotifyInterrupt(
    PVOID   pThis)
{
    GcKmSwQueueNode    *pGcKmSwQueueNode = GcKmSwQueueNode::Cast(pThis);

    return pGcKmSwQueueNode->SynchronizeNotifyInterrupt();
}

BOOLEAN
GcKmSwQueueNode::SynchronizeNotifyInterrupt(void)
{
    m_DxgkInterface.DxgkCbNotifyInterrupt(m_DxgkInterface.DeviceHandle, &m_InterruptData);

    return m_DxgkInterface.DxgkCbQueueDpc(m_DxgkInterface.DeviceHandle);
}

void
GcKmSwQueueNode::QueueDmaBuffer(
    IN_CONST_PDXGKARG_SUBMITCOMMANDVIRTUAL pSubmitCommandVirtual)
{
    UINT                DmaBufferUmdPrivateDataSize = pSubmitCommandVirtual->DmaBufferUmdPrivateDataSize;
    GcDmaBufInfo       *pDmaBufInfo;
    KIRQL               OldIrql;
    GcDmaBufSubmission *pDmaBufSubmission;

    if (DmaBufferUmdPrivateDataSize &&
        ((pSubmitCommandVirtual->DmaBufferPrivateDataSize - DmaBufferUmdPrivateDataSize) < sizeof(GcDmaBufInfo)))
    {
        DmaBufferUmdPrivateDataSize = 0;
    }

    if (!pSubmitCommandVirtual->Flags.ContextSwitch)
    {
        pDmaBufInfo = (GcDmaBufInfo*)(((PBYTE)pSubmitCommandVirtual->pDmaBufferPrivateData) +
                                      DmaBufferUmdPrivateDataSize);
    }
    else
    {
        pDmaBufInfo = &m_DefaultDmaBufInfo;
    }

    KeAcquireSpinLock(&m_DmaBufQueueLock, &OldIrql);

    NT_ASSERT(!IsListEmpty(&m_DmaBufSubmissionFree));

    pDmaBufSubmission = CONTAINING_RECORD(RemoveHeadList(&m_DmaBufSubmissionFree), GcDmaBufSubmission, m_QueueEntry);

    if (!(pSubmitCommandVirtual->Flags.Resubmission || pSubmitCommandVirtual->Flags.ContextSwitch))
    {
        if (!((pSubmitCommandVirtual->Flags.Present) &&
              (pDmaBufInfo->m_DmaBufState.m_bUmBltPresent)))
        {
            pDmaBufInfo->m_DmaBufferGpuVa = pSubmitCommandVirtual->DmaBufferVirtualAddress;
            pDmaBufInfo->m_DmaBufferSize = pSubmitCommandVirtual->DmaBufferSize;
        }

        if (pSubmitCommandVirtual->DmaBufferUmdPrivateDataSize || 
           (pSubmitCommandVirtual->Flags.Paging && !pDmaBufInfo->m_DmaBufState.m_bCpuTransfer))
        {
            pDmaBufInfo->m_DmaBufState.m_Value = 0;
        }

        pDmaBufInfo->m_DmaBufState.m_bPaging = pSubmitCommandVirtual->Flags.Paging;

        pDmaBufInfo->m_NumCommandBufferChunks = 0;
        pDmaBufInfo->m_pCommandBufferChunks = nullptr;

        if (DmaBufferUmdPrivateDataSize >= sizeof(SubmissionExchangeData))
        {
            SubmissionExchangeData* pSubmissionExchangeData = (SubmissionExchangeData*)pSubmitCommandVirtual->pDmaBufferPrivateData;

            if ((pDmaBufInfo->m_DmaBufferGpuVa == pSubmissionExchangeData->m_commandBufferChunks[0].m_commandBufferChunkGpuVa) &&
                (pDmaBufInfo->m_DmaBufferSize == pSubmissionExchangeData->m_commandBufferChunks[0].m_commandBufferChunkSize) &&
                (DmaBufferUmdPrivateDataSize == (FIELD_OFFSET(SubmissionExchangeData, m_commandBufferChunks) +
                                                 pSubmissionExchangeData->m_numCommandBufferChunks*sizeof(CommandBufferChunk))))
            {
                pDmaBufInfo->m_NumCommandBufferChunks = pSubmissionExchangeData->m_numCommandBufferChunks;
                pDmaBufInfo->m_pCommandBufferChunks = pSubmissionExchangeData->m_commandBufferChunks;
            }
        }

        pDmaBufInfo->m_DmaBufState.m_bSubmittedOnce = 1;
    }

    NT_ASSERT(pDmaBufInfo->m_DmaBufState.m_bPreempted == pSubmitCommandVirtual->Flags.Resubmission);
    pDmaBufSubmission->m_pDmaBufInfo = pDmaBufInfo;

    pDmaBufSubmission->m_pContext = (GcKmContext *)pSubmitCommandVirtual->hContext;

    pDmaBufSubmission->m_EngineOrdinal = pSubmitCommandVirtual->EngineOrdinal;
    pDmaBufSubmission->m_NodeOrdinal = pSubmitCommandVirtual->NodeOrdinal;

    pDmaBufSubmission->m_SubmissionFenceId = pSubmitCommandVirtual->SubmissionFenceId;

    InsertTailList(&m_DmaBufQueue, &pDmaBufSubmission->m_QueueEntry);

    m_LastSubmittedFenceId = pSubmitCommandVirtual->SubmissionFenceId;

    KeReleaseSpinLock(&m_DmaBufQueueLock, OldIrql);
}

void
GcKmSwQueueNode::HwDmaBufCompletionDpcRoutine(
    KDPC   *pDPC,
    PVOID   deferredContext,
    PVOID   systemArgument1,
    PVOID   systemArgument2)
{
    GcKmSwQueueNode    *pGcKmSwQueueNode = GcKmSwQueueNode::Cast(deferredContext);

    UNREFERENCED_PARAMETER(pDPC);
    UNREFERENCED_PARAMETER(systemArgument1);
    UNREFERENCED_PARAMETER(systemArgument2);

    // Signal to the worker thread that a HW DMA buffer has completed
    KeSetEvent(&pGcKmSwQueueNode->m_HwDmaBufCompletionEvent, 0, FALSE);
}

