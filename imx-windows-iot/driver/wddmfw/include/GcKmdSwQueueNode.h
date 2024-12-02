/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "GcKmd.h"

#include "GcKmdAdapter.h"
#include "GcCommandBuffer.h"

class GcKmContext;

typedef struct _GcDmaBufState
{
    union
    {
        struct
        {
            UINT    m_bRender           : 1;
            UINT    m_bPresent          : 1;
            UINT    m_bUmBltPresent     : 1;
            UINT    m_bGdi              : 1;
            UINT    m_bGdiInitialized   : 1;
            UINT    m_bGdiSamplerUsed   : 1;
            UINT    m_bPaging           : 1;
            UINT    m_bSubmittedOnce    : 1;
            UINT    m_bRun              : 1;
            UINT    m_bPreempted        : 1;
            UINT    m_bReset            : 1;
            UINT    m_bCompleted        : 1;
            UINT    m_bCpuTransfer      : 1;
        };

        UINT        m_Value;
    };
} GcDmaBufState;

//
// Data in the GcUmdDmaPrivateData comes from UMD
//

struct GcDmaBufInfo
{
    PBYTE                       m_pDmaBuffer;
    D3DGPU_VIRTUAL_ADDRESS      m_DmaBufferGpuVa;
    UINT                        m_DmaBufferSize;
    GcDmaBufState               m_DmaBufState;

    UINT                        m_NumCommandBufferChunks;
    CommandBufferChunk         *m_pCommandBufferChunks;

    DXGK_BUILDPAGINGBUFFER_TRANSFERVIRTUAL m_TransferVirtual;
};

struct GcDmaBufSubmission
{
    LIST_ENTRY      m_QueueEntry;
    GcDmaBufInfo   *m_pDmaBufInfo;
    GcKmContext    *m_pContext;
    UINT            m_EngineOrdinal;
    UINT            m_NodeOrdinal;
    UINT            m_SubmissionFenceId;
};

class GcKmSwQueueNode : public GcKmNode
{
public:

    static GcKmSwQueueNode* Cast(void* ptr)
    {
        GcKmSwQueueNode    *pGcKmSwQueueNode = static_cast<GcKmSwQueueNode *>(ptr);

        return pGcKmSwQueueNode;
    }

    GcKmSwQueueNode(
        GcKmMmu    *pMmu)
    {
        m_pMmu = pMmu;

        m_pWorkerThread = nullptr;

        memset(&m_DefaultDmaBufInfo, 0, sizeof(m_DefaultDmaBufInfo));
    }

    virtual NTSTATUS Start(
        IN_PDXGKRNL_INTERFACE   DxgkInterface);

    virtual NTSTATUS Stop();

    virtual NTSTATUS SubmitCommandVirtual(
        IN_CONST_PDXGKARG_SUBMITCOMMANDVIRTUAL pSubmitCommand);

    virtual NTSTATUS Preempt(
        IN_CONST_PDXGKARG_PREEMPTCOMMAND pPreemptCommand);

    virtual void PrepareToReset();

    static void HwDmaBufCompletionDpcRoutine(KDPC *, PVOID, PVOID, PVOID);

private:

    static void WorkerThread(void* StartContext);
    void DoWork(void);
    void QueueDmaBuffer(IN_CONST_PDXGKARG_SUBMITCOMMANDVIRTUAL pSubmitCommand);
    void NotifyDmaBufCompletion(GcDmaBufSubmission* pDmaBufSubmission);
    void NotifyPreemptionCompletion();
    static BOOLEAN SynchronizeNotifyInterrupt(PVOID SynchronizeContext);
    BOOLEAN SynchronizeNotifyInterrupt();
    GcDmaBufSubmission *DequeueDmaBuffer(KSPIN_LOCK* pDmaBufQueueLock);
    void EmptyDmaBufferQueue(bool bForPreemption);

protected:

    void RegisterEngine(
        GcKmEngine  *pEngine)
    {
        m_pHwEngine = pEngine;
    }

protected:

    PKTHREAD                    m_pWorkerThread;
    KEVENT                      m_WorkerThreadEvent;
    bool                        m_bWorkerExit;

    // m_MaxDmaBufQueueLength should be the same as m_DxgkStartInfo::RequiredDmaQueueEntry
    const static UINT           m_MaxDmaBufQueueLength = 32;
    GcDmaBufSubmission          m_DmaBufSubmissions[m_MaxDmaBufQueueLength];

    LIST_ENTRY                  m_DmaBufSubmissionFree;
    LIST_ENTRY                  m_DmaBufQueue;
    KSPIN_LOCK                  m_DmaBufQueueLock;

    GcDmaBufInfo                m_DefaultDmaBufInfo;

    UINT                        m_LastSubmittedFenceId;
    UINT                        m_LastProcessedFenceId;
    UINT                        m_LastCompletetdFenceId;

    UINT                        m_LastCompeletedPreemptionFenceId;

    DXGKARG_PREEMPTCOMMAND      m_PreemptCommand;

    bool                        m_bPreemptionRequested;

    bool                        m_bResetRequested;
    KEVENT                      m_ResetCompletionEvent;
    bool                        m_bWaitingForReset;

    KDPC                        m_HwDmaBufCompletionDpc;
    KEVENT                      m_HwDmaBufCompletionEvent;

    DXGKARGCB_NOTIFY_INTERRUPT_DATA m_InterruptData;

    DXGKRNL_INTERFACE           m_DxgkInterface;

    GcKmMmu                    *m_pMmu;
    GcKmEngine                 *m_pHwEngine;
};

#pragma once
