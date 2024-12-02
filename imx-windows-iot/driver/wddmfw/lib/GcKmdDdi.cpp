/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmdDdi.tmh"

#include "GcKmdDdi.h"
#include "GcKmdAdapter.h"

#include "GcKmdDevice.h"
#include "GcKmdContext.h"

#if 0
#include "GcKmdAllocation.h"
#include "GcKmdDdi.h"
#include "GcKmdContext.h"
#include "GcKmdResource.h"
#endif

#include "GcKmdUtil.h"

NTSTATUS __stdcall
GcKmdDdi::DdiAddAdapter(
    IN_CONST_PDEVICE_OBJECT     PhysicalDeviceObject,
    OUT_PPVOID                  MiniportDeviceContext)
{
    return GcKmAdapter::AddAdapter(PhysicalDeviceObject, MiniportDeviceContext);
}

NTSTATUS __stdcall
GcKmdDdi::DdiStartAdapter(
    IN_CONST_PVOID          MiniportDeviceContext,
    IN_PDXGK_START_INFO     DxgkStartInfo,
    IN_PDXGKRNL_INTERFACE   DxgkInterface,
    OUT_PULONG              NumberOfVideoPresentSources,
    OUT_PULONG              NumberOfChildren)
{
    GcKmAdapter  *pGcKmAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    return pGcKmAdapter->Start(DxgkStartInfo, DxgkInterface, NumberOfVideoPresentSources, NumberOfChildren);
}


NTSTATUS __stdcall
GcKmdDdi::DdiStopAdapter(
    IN_CONST_PVOID  MiniportDeviceContext)
{
    GcKmAdapter  *pGcKmAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    return pGcKmAdapter->Stop();
}


NTSTATUS __stdcall
GcKmdDdi::DdiRemoveAdapter(
    IN_CONST_PVOID  MiniportDeviceContext)
{
    GcKmAdapter  *pGcKmAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    delete pGcKmAdapter;

    return STATUS_SUCCESS;
}


NTSTATUS
GcKmdDdi::DdiDispatchIoRequest(
    IN_CONST_PVOID              MiniportDeviceContext,
    IN_ULONG                    VidPnSourceId,
    IN_PVIDEO_REQUEST_PACKET    VideoRequestPacket)
{
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n",
        __FUNCTION__, MiniportDeviceContext);

    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);
    return pGcKmdAdapter->DispatchIoRequest(VidPnSourceId, VideoRequestPacket);
}


BOOLEAN
GcKmdDdi::DdiInterruptRoutine(
    IN_CONST_PVOID  MiniportDeviceContext,
    IN_ULONG        MessageNumber)
{
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n",
        __FUNCTION__, MiniportDeviceContext);

    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    return pGcKmdAdapter->InterruptRoutine(MessageNumber);
}


void
GcKmdDdi::DdiDpcRoutine(
    IN_CONST_PVOID  MiniportDeviceContext)
{
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n",
        __FUNCTION__, MiniportDeviceContext);

    GcKmAdapter* pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    pGcKmdAdapter->DpcRoutine();
}


NTSTATUS
__stdcall
GcKmdDdi::DdiBuildPagingBuffer(
    IN_CONST_HANDLE                 hAdapter,
    IN_PDXGKARG_BUILDPAGINGBUFFER   pArgs)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->BuildPagingBuffer(pArgs);
}


NTSTATUS __stdcall GcKmdDdi::DdiSubmitCommand(
    IN_CONST_HANDLE                     hAdapter,
    IN_CONST_PDXGKARG_SUBMITCOMMAND     pSubmitCommand)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    //
    // This only happens when Context Allocation is created
    //

    if (pSubmitCommand->Flags.ContextSwitch)
    {
        DXGKARG_SUBMITCOMMANDVIRTUAL    SubmitCommandVirtual = {};

        SubmitCommandVirtual.Flags.ContextSwitch = 1;

        SubmitCommandVirtual.EngineOrdinal = pSubmitCommand->EngineOrdinal;
        SubmitCommandVirtual.NodeOrdinal = pSubmitCommand->NodeOrdinal;
        SubmitCommandVirtual.SubmissionFenceId = pSubmitCommand->SubmissionFenceId;

        return DdiSubmitCommandVirtual(hAdapter, &SubmitCommandVirtual);
    }

    return STATUS_NOT_IMPLEMENTED;
}


NTSTATUS __stdcall GcKmdDdi::DdiCreateAllocation(
    IN_CONST_HANDLE                     hAdapter,
    INOUT_PDXGKARG_CREATEALLOCATION     pCreateAllocation)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->CreateAllocation(pCreateAllocation);
}


NTSTATUS __stdcall GcKmdDdi::DdiDestroyAllocation(
    IN_CONST_HANDLE                         hAdapter,
    IN_CONST_PDXGKARG_DESTROYALLOCATION     pDestroyAllocation)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->DestroyAllocation(pDestroyAllocation);;
}


NTSTATUS __stdcall GcKmdDdi::DdiQueryAdapterInfo(
    IN_CONST_HANDLE                         hAdapter,
    IN_CONST_PDXGKARG_QUERYADAPTERINFO      pQueryAdapterInfo)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->QueryAdapterInfo(pQueryAdapterInfo);
}


NTSTATUS __stdcall GcKmdDdi::DdiCreateDevice(
    IN_CONST_HANDLE             hAdapter,
    INOUT_PDXGKARG_CREATEDEVICE pCreateDevice)
{
    GcKmAdapter* pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->CreateDevice(pCreateDevice);
}


NTSTATUS __stdcall GcKmdDdi::DdiDestroyDevice(
    IN_CONST_HANDLE hDevice)
{
    GcKmDevice* pGcKmDevice = GcKmDevice::Cast(hDevice);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hDevice=%lx\n", __FUNCTION__, hDevice);

    delete pGcKmDevice;

    return STATUS_SUCCESS;
}

NTSTATUS __stdcall GcKmdDdi::DdiDescribeAllocation(
    IN_CONST_HANDLE                         hAdapter,
    INOUT_PDXGKARG_DESCRIBEALLOCATION       pDescribeAllocation)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->DescribeAllocation(pDescribeAllocation);
}


NTSTATUS __stdcall GcKmdDdi::DdiGetNodeMetadata(
    IN_CONST_HANDLE                 hAdapter,
    UINT                            NodeOrdinal,
    OUT_PDXGKARG_GETNODEMETADATA    pGetNodeMetadata)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->GetNodeMetaData(NodeOrdinal, pGetNodeMetadata);
}


NTSTATUS __stdcall GcKmdDdi::DdiGetStandardAllocationDriverData(
    IN_CONST_HANDLE                                 hAdapter,
    INOUT_PDXGKARG_GETSTANDARDALLOCATIONDRIVERDATA  pGetStandardAllocationDriverData)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->GetStandardAllocationDriverData(pGetStandardAllocationDriverData);
}


#if GC_GPUVA_SUPPORT

NTSTATUS
__stdcall
GcKmdDdi::DdiSubmitCommandVirtual(
    IN_CONST_HANDLE                         hAdapter,
    IN_CONST_PDXGKARG_SUBMITCOMMANDVIRTUAL  pSubmitCommandVirtual)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->SubmitCommandVirtual(pSubmitCommandVirtual);
}


SIZE_T
__stdcall
GcKmdDdi::DdiGetRootPageTableSize(
    IN_CONST_HANDLE                     hAdapter,
    INOUT_PDXGKARG_GETROOTPAGETABLESIZE pArgs)
{
    GcKmAdapter    *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->GetRootPageTableSize(pArgs);
}


VOID
__stdcall
GcKmdDdi::DdiSetRootPageTable(
    IN_CONST_HANDLE                     /*hAdapter*/,
    IN_CONST_PDXGKARG_SETROOTPAGETABLE  pSetPageTable)
{
    GcKmContext    *pGcKmContext = GcKmContext::Cast(pSetPageTable->hContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hContext=%lx\n", __FUNCTION__, pSetPageTable->hContext);

    pGcKmContext->SetRootPageTable(pSetPageTable);
}

#endif


NTSTATUS
__stdcall
GcKmdDdi::DdiPreemptCommand(
    IN_CONST_HANDLE                     hAdapter,
    IN_CONST_PDXGKARG_PREEMPTCOMMAND    pPreemptCommand)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->PreemptCommand(pPreemptCommand);
}


NTSTATUS
__stdcall
GcKmdDdi::DdiCancelCommand(
    IN_CONST_HANDLE                 hAdapter,
    IN_CONST_PDXGKARG_CANCELCOMMAND pCancelCommand)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    // NT_ASSERT(false);

    return STATUS_SUCCESS;
}


NTSTATUS
__stdcall
GcKmdDdi::DdiResetFromTimeout(
    IN_CONST_HANDLE     hAdapter)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->ResetFromTimeout();
}


NTSTATUS
__stdcall CALLBACK
GcKmdDdi::DdiRestartFromTimeout(
    IN_CONST_HANDLE     hAdapter)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->RestartFromTimeout();
}


NTSTATUS
__stdcall
GcKmdDdi::DdiCollectDbgInfo(
    IN_CONST_HANDLE                         hAdapter,
    IN_CONST_PDXGKARG_COLLECTDBGINFO        pCollectDbgInfo)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->CollectDbgInfo(pCollectDbgInfo);
}


#if 0

_Use_decl_annotations_
NTSTATUS GcKmdDdi::DdiQueryDependentEngineGroup(
    HANDLE const                        hAdapter,
    DXGKARG_QUERYDEPENDENTENGINEGROUP  *ArgsPtr
)
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(hAdapter)->QueryDependentEngineGroup(ArgsPtr);
}


NTSTATUS
__stdcall
GcKmdDdi::DdiQueryEngineStatus(
    IN_CONST_HANDLE                     hAdapter,
    INOUT_PDXGKARG_QUERYENGINESTATUS    pQueryEngineStatus)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->QueryEngineStatus(pQueryEngineStatus);
}

#endif


NTSTATUS
__stdcall
GcKmdDdi::DdiResetEngine(
    IN_CONST_HANDLE             hAdapter,
    INOUT_PDXGKARG_RESETENGINE  pResetEngine)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    // return pGcKmdAdapter->ResetEngine(pResetEngine);
    return STATUS_NOT_IMPLEMENTED;
}


#if GC_GPUVA_SUPPORT

NTSTATUS
__stdcall
GcKmdDdi::DdiCreateProcess(
    IN_PVOID                    pMiniportDeviceContext,
    IN DXGKARG_CREATEPROCESS   *pArgs)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(pMiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s pMiniportDeviceContext=%lx\n", __FUNCTION__, pMiniportDeviceContext);

    return pGcKmdAdapter->CreateProcess(pArgs);
}


NTSTATUS
__stdcall
GcKmdDdi::DdiDestroyProcess(
    IN_PVOID    pMiniportDeviceContext,
    IN HANDLE   KmdProcessHandle)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(pMiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s pMiniportDeviceContext=%lx\n", __FUNCTION__, pMiniportDeviceContext);

    return pGcKmdAdapter->DestroyProcess(KmdProcessHandle);
}

#endif


void
__stdcall
GcKmdDdi::DdiSetStablePowerState(
    IN_CONST_HANDLE                        hAdapter,
    IN_CONST_PDXGKARG_SETSTABLEPOWERSTATE  pArgs)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    NT_ASSERT(FALSE);
}


NTSTATUS
__stdcall
GcKmdDdi::DdiCalibrateGpuClock(
    IN_CONST_HANDLE                             hAdapter,
    IN UINT32                                   NodeOrdinal,
    IN UINT32                                   EngineOrdinal,
    OUT_PDXGKARG_CALIBRATEGPUCLOCK              pClockCalibration
    )
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return STATUS_NOT_SUPPORTED;
}

NTSTATUS
__stdcall
GcKmdDdi::DdiEscape(
    IN_CONST_HANDLE                 hAdapter,
    IN_CONST_PDXGKARG_ESCAPE        pEscape)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->Escape(pEscape);
}


NTSTATUS
GcKmdDdi::DdiQueryInterface(
    IN_CONST_PVOID          MiniportDeviceContext,
    IN_PQUERY_INTERFACE     QueryInterface)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n", __FUNCTION__, MiniportDeviceContext);

    return pGcKmdAdapter->QueryInterface(QueryInterface);
}


NTSTATUS
GcKmdDdi::DdiQueryChildRelations(
    IN_CONST_PVOID                  MiniportDeviceContext,
    INOUT_PDXGK_CHILD_DESCRIPTOR    ChildRelations,
    IN_ULONG                        ChildRelationsSize)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n", __FUNCTION__, MiniportDeviceContext);

    return pGcKmdAdapter->QueryChildRelations(ChildRelations, ChildRelationsSize);
}


NTSTATUS
GcKmdDdi::DdiQueryChildStatus(
    IN_CONST_PVOID          MiniportDeviceContext,
    IN_PDXGK_CHILD_STATUS   ChildStatus,
    IN_BOOLEAN              NonDestructiveOnly)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n", __FUNCTION__, MiniportDeviceContext);

    return pGcKmdAdapter->QueryChildStatus(ChildStatus, NonDestructiveOnly);
}


NTSTATUS
GcKmdDdi::DdiQueryDeviceDescriptor(
    IN_CONST_PVOID                  MiniportDeviceContext,
    IN_ULONG                        ChildUid,
    INOUT_PDXGK_DEVICE_DESCRIPTOR   pDeviceDescriptor)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n", __FUNCTION__, MiniportDeviceContext);

    return pGcKmdAdapter->QueryDeviceDescriptor(ChildUid, pDeviceDescriptor);
}


NTSTATUS
GcKmdDdi::DdiSetPowerState(
    IN_CONST_PVOID          MiniportDeviceContext,
    IN_ULONG                DeviceUid,
    IN_DEVICE_POWER_STATE   DevicePowerState,
    IN_POWER_ACTION         ActionType)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n", __FUNCTION__, MiniportDeviceContext);

    return pGcKmdAdapter->SetPowerState(DeviceUid, DevicePowerState, ActionType);
}

#if 0

NTSTATUS
GcKmdDdi::DdiSetPowerComponentFState(
    IN_CONST_PVOID       MiniportDeviceContext,
    IN UINT              ComponentIndex,
    IN UINT              FState)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n", __FUNCTION__, MiniportDeviceContext);

    return pGcKmdAdapter->SetPowerComponentFState(ComponentIndex, FState);
}


NTSTATUS
GcKmdDdi::DdiPowerRuntimeControlRequest(
    IN_CONST_PVOID       MiniportDeviceContext,
    IN LPCGUID           PowerControlCode,
    IN OPTIONAL PVOID    InBuffer,
    IN SIZE_T            InBufferSize,
    OUT OPTIONAL PVOID   OutBuffer,
    IN SIZE_T            OutBufferSize,
    OUT OPTIONAL PSIZE_T BytesReturned)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n", __FUNCTION__, MiniportDeviceContext);

    return pGcKmdAdapter->PowerRuntimeControlRequest(PowerControlCode, InBuffer, InBufferSize, OutBuffer, OutBufferSize, BytesReturned);
}


NTSTATUS
GcKmdDdi::DdiNotifyAcpiEvent(
    IN_CONST_PVOID      MiniportDeviceContext,
    IN_DXGK_EVENT_TYPE  EventType,
    IN_ULONG            Event,
    IN_PVOID            Argument,
    OUT_PULONG          AcpiFlags)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n", __FUNCTION__, MiniportDeviceContext);

    return pGcKmdAdapter->NotifyAcpiEvent(EventType, Event, Argument, AcpiFlags);
}

#endif

void
GcKmdDdi::DdiResetDevice(
    IN_CONST_PVOID  MiniportDeviceContext)
{
    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(MiniportDeviceContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s MiniportDeviceContext=%lx\n", __FUNCTION__, MiniportDeviceContext);

    return pGcKmdAdapter->ResetDevice();
}

GC_NONPAGED_SEGMENT_BEGIN; //================================================

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiSetVidPnSourceAddress (
    HANDLE const hAdapter,
    const DXGKARG_SETVIDPNSOURCEADDRESS* SetVidPnSourceAddressPtr
    )
{
    return GcKmAdapter::Cast(hAdapter)->SetVidPnSourceAddress(
            SetVidPnSourceAddressPtr);
}

GC_NONPAGED_SEGMENT_END; //==================================================
GC_PAGED_SEGMENT_BEGIN; //===================================================
// TODO: put PASSIVE_LEVEL DDIs in the paged section

//
// GcKmdDdi
//

NTSTATUS
GcKmdDdi::DdiQueryCurrentFence(
    IN_CONST_HANDLE                     hAdapter,
    INOUT_PDXGKARG_QUERYCURRENTFENCE    pCurrentFence)
{
    // Deprecated in 19H1
    return STATUS_NOT_IMPLEMENTED;
}

NTSTATUS
GcKmdDdi::DdiControlInterrupt(
    HANDLE const        hAdapter,
    DXGK_INTERRUPT_TYPE InterruptType,
    BOOLEAN             EnableInterrupt
    )
{
    return GcKmAdapter::Cast(hAdapter)->ControlInterrupt(
            InterruptType,
            EnableInterrupt);
}

_Use_decl_annotations_
NTSTATUS
GcKmdDdi::DdiOpenAllocation (
    HANDLE const                    hDevice,
    const DXGKARG_OPENALLOCATION   *ArgsPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmDevice::Cast(hDevice)->OpenAllocation(ArgsPtr);
}

NTSTATUS
GcKmdDdi::DdiPresent(
    IN_CONST_HANDLE         hContext,
    INOUT_PDXGKARG_PRESENT  pPresent)
{
    GcKmContext    *pGcKmContext = GcKmContext::Cast(hContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hContext=%lx\n", __FUNCTION__, hContext);

    return pGcKmContext->Present(pPresent);
}

NTSTATUS
GcKmdDdi::DdiCreateContext(
    IN_CONST_HANDLE                 hDevice,
    INOUT_PDXGKARG_CREATECONTEXT    pCreateContext)
{
    GcKmDevice* pGcKmDevice = GcKmDevice::Cast(hDevice);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hDevice=%lx\n", __FUNCTION__, hDevice);

    return pGcKmDevice->CreateContext(pCreateContext);
}

NTSTATUS
GcKmdDdi::DdiDestroyContext(
    IN_CONST_HANDLE hContext)
{
    GcKmContext*    pGcKmContext = GcKmContext::Cast(hContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hContext=%lx\n", __FUNCTION__, hContext);

    delete pGcKmContext;

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmdDdi::DdiRenderGdi(
    IN_CONST_HANDLE             hContext,
    INOUT_PDXGKARG_RENDERGDI    pRenderGdi)
{
    GcKmContext    *pGcKmContext = GcKmContext::Cast(hContext);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hContext=%lx\n", __FUNCTION__, hContext);

    return pGcKmContext->RenderGdi(pRenderGdi);
}

NTSTATUS
GcKmdDdi::DdiRenderKm(
    IN_CONST_HANDLE         hContext,
    INOUT_PDXGKARG_RENDER   pRenderGdi)
{
    return STATUS_NOT_IMPLEMENTED;
}

#if 0

NTSTATUS
GcKmdDdi::DdiSetVirtualMachineData(
    IN_CONST_HANDLE                         hAdapter,
    IN_CONST_PDXGKARG_SETVIRTUALMACHINEDATA Args)
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    GcKmAdapter  *pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->SetVirtualMachineData(Args);
}

NTSTATUS
GcKmdDdi::DdiBeginExclusiveAccess(
    IN_CONST_HANDLE                  hAdapter,
    IN_PDXGKARG_BEGINEXCLUSIVEACCESS pBeginExclusiveAccess)
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    GcKmAdapter* pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->BeginExclusiveAccess(pBeginExclusiveAccess);
}

NTSTATUS
GcKmdDdi::DdiEndExclusiveAccess(
    IN_CONST_HANDLE                hAdapter,
    IN_PDXGKARG_ENDEXCLUSIVEACCESS pEndExclusiveAccess)
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    GcKmAdapter* pGcKmdAdapter = GcKmAdapter::Cast(hAdapter);

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s hAdapter=%lx\n", __FUNCTION__, hAdapter);

    return pGcKmdAdapter->EndExclusiveAccess(pEndExclusiveAccess);
}

#endif

//
// GcKmdDisplayDdi
//

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiSetPalette (
    HANDLE const AdapterPtr,
    const DXGKARG_SETPALETTE* SetPalettePtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(AdapterPtr)->SetPalette(SetPalettePtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiSetPointerPosition (
    HANDLE const AdapterPtr,
    const DXGKARG_SETPOINTERPOSITION* SetPointerPositionPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(AdapterPtr)->SetPointerPosition(
        SetPointerPositionPtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiSetPointerShape (
    HANDLE const AdapterPtr,
    const DXGKARG_SETPOINTERSHAPE* SetPointerShapePtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(AdapterPtr)->SetPointerShape(SetPointerShapePtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiIsSupportedVidPn (
    VOID* const MiniportDeviceContextPtr,
    DXGKARG_ISSUPPORTEDVIDPN* IsSupportedVidPnPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->IsSupportedVidPn(
            IsSupportedVidPnPtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiRecommendFunctionalVidPn (
    VOID* const MiniportDeviceContextPtr,
    const DXGKARG_RECOMMENDFUNCTIONALVIDPN* const RecommendFunctionalVidPnPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->RecommendFunctionalVidPn(
            RecommendFunctionalVidPnPtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiEnumVidPnCofuncModality (
    VOID* const MiniportDeviceContextPtr,
    const DXGKARG_ENUMVIDPNCOFUNCMODALITY* const EnumCofuncModalityPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->EnumVidPnCofuncModality(
            EnumCofuncModalityPtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiSetVidPnSourceVisibility (
    VOID* const MiniportDeviceContextPtr,
    const DXGKARG_SETVIDPNSOURCEVISIBILITY* SetVidPnSourceVisibilityPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->SetVidPnSourceVisibility(
            SetVidPnSourceVisibilityPtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiCommitVidPn (
    VOID* const MiniportDeviceContextPtr,
    const DXGKARG_COMMITVIDPN* const CommitVidPnPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->CommitVidPn(
            CommitVidPnPtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiUpdateActiveVidPnPresentPath (
    VOID* const MiniportDeviceContextPtr,
    const DXGKARG_UPDATEACTIVEVIDPNPRESENTPATH* const UpdateActiveVidPnPresentPathPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->UpdateActiveVidPnPresentPath(
            UpdateActiveVidPnPresentPathPtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiRecommendMonitorModes (
    VOID* const MiniportDeviceContextPtr,
    const DXGKARG_RECOMMENDMONITORMODES* const RecommendMonitorModesPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->RecommendMonitorModes(
            RecommendMonitorModesPtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiGetScanLine (
    HANDLE const AdapterPtr,
    DXGKARG_GETSCANLINE*  GetScanLinePtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(AdapterPtr)->GetScanLine(GetScanLinePtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiQueryVidPnHWCapability (
    VOID* const MiniportDeviceContextPtr,
    DXGKARG_QUERYVIDPNHWCAPABILITY* VidPnHWCapsPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->QueryVidPnHWCapability(
            VidPnHWCapsPtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiPresentDisplayOnly(
    VOID* const MiniportDeviceContextPtr,
    IN_CONST_PDXGKARG_PRESENT_DISPLAYONLY   PresentDisplayOnlyPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->PresentDisplayOnly(
            PresentDisplayOnlyPtr);
}

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiStopDeviceAndReleasePostDisplayOwnership (
    VOID* const MiniportDeviceContextPtr,
    D3DDDI_VIDEO_PRESENT_TARGET_ID TargetId,
    DXGK_DISPLAY_INFORMATION* DisplayInfoPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->StopDeviceAndReleasePostDisplayOwnership(
            TargetId,
            DisplayInfoPtr);
}

GC_PAGED_SEGMENT_END; //=====================================================
GC_NONPAGED_SEGMENT_BEGIN; //================================================

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiSetVidPnSourceAddressWithMultiPlaneOverlay3(
    VOID* const MiniportDeviceContextPtr,
    DXGKARG_SETVIDPNSOURCEADDRESSWITHMULTIPLANEOVERLAY3* SetVidPnSourceAddressWithMpo3Ptr
    )
{
    GC_ASSERT_MAX_IRQL(PROFILE_LEVEL - 1);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->SetVidPnSourceAddressWithMultiPlaneOverlay3(
            SetVidPnSourceAddressWithMpo3Ptr);
}

GC_NONPAGED_SEGMENT_END; //====================================================
GC_PAGED_SEGMENT_BEGIN; //=====================================================

_Use_decl_annotations_
NTSTATUS GcKmdDisplayDdi::DdiUpdateMonitorLinkInfo(
    VOID* const MiniportDeviceContextPtr,
    DXGKARG_UPDATEMONITORLINKINFO* UpdateMonitorLinkInfoPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->UpdateMonitorLinkInfo(
            UpdateMonitorLinkInfoPtr);
}

NTSTATUS GcKmdDisplayDdi::DdiGetChildContainerId(
    VOID* const MiniportDeviceContextPtr,
    ULONG ChildUid,
    DXGK_CHILD_CONTAINER_ID* ContainerIdPtr
    )
{
    PAGED_CODE();
    GC_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    return GcKmAdapter::Cast(MiniportDeviceContextPtr)->GetChildContainerId(
            ChildUid,
            ContainerIdPtr);
}

GC_PAGED_SEGMENT_END; //=====================================================
