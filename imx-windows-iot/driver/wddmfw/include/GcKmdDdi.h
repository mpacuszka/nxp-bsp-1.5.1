/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include "GcKmd.h"

class GcKmdDdi
{
public:

    static NTSTATUS __stdcall
    DdiAddAdapter(
        IN_CONST_PDEVICE_OBJECT     PhysicalDeviceObject,
        OUT_PPVOID                  MiniportDeviceContext);

    static NTSTATUS __stdcall
    DdiStartAdapter(
        IN_CONST_PVOID          MiniportDeviceContext,
        IN_PDXGK_START_INFO     DxgkStartInfo,
        IN_PDXGKRNL_INTERFACE   DxgkInterface,
        OUT_PULONG              NumberOfVideoPresentSources,
        OUT_PULONG              NumberOfChildren);

    static NTSTATUS __stdcall
    DdiStopAdapter(
        IN_CONST_PVOID  MiniportDeviceContext);

    static NTSTATUS __stdcall
    DdiRemoveAdapter(
        IN_CONST_PVOID  MiniportDeviceContext);

    static void __stdcall
    DdiDpcRoutine(
        IN_CONST_PVOID  MiniportDeviceContext);

    static NTSTATUS
    DdiDispatchIoRequest(
        IN_CONST_PVOID              MiniportDeviceContext,
        IN_ULONG                    VidPnSourceId,
        IN_PVIDEO_REQUEST_PACKET    VideoRequestPacket);

    static BOOLEAN
    DdiInterruptRoutine(
        IN_CONST_PVOID  MiniportDeviceContext,
        IN_ULONG        MessageNumber);

    static NTSTATUS __stdcall
    DdiBuildPagingBuffer(
        IN_CONST_HANDLE                 hAdapter,
        IN_PDXGKARG_BUILDPAGINGBUFFER   pArgs);

    static NTSTATUS __stdcall
    DdiSubmitCommand(
        IN_CONST_HANDLE                     hAdapter,
        IN_CONST_PDXGKARG_SUBMITCOMMAND     pSubmitCommand);

#if GC_PHYSICAL_SUPPORT
    static NTSTATUS __stdcall
    DdiPatch(
        IN_CONST_HANDLE             hAdapter,
        IN_CONST_PDXGKARG_PATCH     pPatch);
#endif

    static NTSTATUS __stdcall
    DdiCreateAllocation(
        IN_CONST_HANDLE                     hAdapter,
        INOUT_PDXGKARG_CREATEALLOCATION     pCreateAllocation);

    static NTSTATUS __stdcall
    DdiDestroyAllocation(
        IN_CONST_HANDLE                         hAdapter,
        IN_CONST_PDXGKARG_DESTROYALLOCATION     pDestroyAllocation);

    static NTSTATUS __stdcall 
    DdiQueryAdapterInfo(
        IN_CONST_HANDLE                         hAdapter,
        IN_CONST_PDXGKARG_QUERYADAPTERINFO      pQueryAdapterInfo);

    static NTSTATUS __stdcall
    DdiCreateDevice(
        IN_CONST_HANDLE             hAdapter,
        INOUT_PDXGKARG_CREATEDEVICE pCreateDevice);

    static NTSTATUS __stdcall
    DdiDestroyDevice(
        IN_CONST_HANDLE hDevice);

    static NTSTATUS __stdcall
    DdiDescribeAllocation(
        IN_CONST_HANDLE                         hAdapter,
        INOUT_PDXGKARG_DESCRIBEALLOCATION       pDescribeAllocation);

    static NTSTATUS __stdcall
    DdiGetNodeMetadata(
        IN_CONST_HANDLE                 hAdapter,
        UINT                            NodeOrdinal,
        OUT_PDXGKARG_GETNODEMETADATA    pGetNodeMetadata);

    static NTSTATUS __stdcall
    DdiGetStandardAllocationDriverData(
        IN_CONST_HANDLE                                 hAdapter,
        INOUT_PDXGKARG_GETSTANDARDALLOCATIONDRIVERDATA  pGetStandardAllocationDriverData);

    static NTSTATUS __stdcall
    DdiSubmitCommandVirtual(
        IN_CONST_HANDLE                         hAdapter,
        IN_CONST_PDXGKARG_SUBMITCOMMANDVIRTUAL  pSubmitCommandVirtual);

    static VOID __stdcall
    DdiSetRootPageTable(
        IN_CONST_HANDLE                     hAdapter,
        IN_CONST_PDXGKARG_SETROOTPAGETABLE  pSetPageTable);

    static SIZE_T __stdcall
    DdiGetRootPageTableSize(
        IN_CONST_HANDLE                     hAdapter,
        INOUT_PDXGKARG_GETROOTPAGETABLESIZE pArgs);

    static NTSTATUS __stdcall
    DdiPreemptCommand(
        IN_CONST_HANDLE                     hAdapter,
        IN_CONST_PDXGKARG_PREEMPTCOMMAND    pPreemptCommand);

    static NTSTATUS __stdcall CALLBACK
    DdiRestartFromTimeout(
        IN_CONST_HANDLE     hAdapter);

    static NTSTATUS __stdcall
    DdiCancelCommand(
        IN_CONST_HANDLE                 hAdapter,
        IN_CONST_PDXGKARG_CANCELCOMMAND pCancelCommand);

    static NTSTATUS __stdcall
    DdiResetEngine(
        IN_CONST_HANDLE             hAdapter,
        INOUT_PDXGKARG_RESETENGINE  pResetEngine);

    static NTSTATUS __stdcall
    DdiQueryEngineStatus(
        IN_CONST_HANDLE                     hAdapter,
        INOUT_PDXGKARG_QUERYENGINESTATUS    pQueryEngineStatus);

    static NTSTATUS __stdcall
    DdiCollectDbgInfo(
        IN_CONST_HANDLE                         hAdapter,
        IN_CONST_PDXGKARG_COLLECTDBGINFO        pCollectDbgInfo);

#if GC_GPUVA_SUPPORT

    static NTSTATUS __stdcall
    DdiCreateProcess(
        IN_PVOID  pMiniportDeviceContext,
        IN DXGKARG_CREATEPROCESS* pArgs);

    static NTSTATUS __stdcall
    DdiDestroyProcess(
        IN_PVOID pMiniportDeviceContext,
        IN HANDLE KmdProcessHandle);

#endif

    static void __stdcall
    DdiSetStablePowerState(
        IN_CONST_HANDLE                        hAdapter,
        IN_CONST_PDXGKARG_SETSTABLEPOWERSTATE  pArgs);

    static NTSTATUS __stdcall
    DdiCalibrateGpuClock(
        IN_CONST_HANDLE                 hAdapter,
        IN UINT32                       NodeOrdinal,
        IN UINT32                       EngineOrdinal,
        OUT_PDXGKARG_CALIBRATEGPUCLOCK  pClockCalibration);

    static NTSTATUS __stdcall
    DdiEscape(
        IN_CONST_HANDLE                 hAdapter,
        IN_CONST_PDXGKARG_ESCAPE        pEscape);

    static NTSTATUS __stdcall
    DdiResetFromTimeout(
        IN_CONST_HANDLE     hAdapter);

    static NTSTATUS
    DdiQueryInterface(
        IN_CONST_PVOID          MiniportDeviceContext,
        IN_PQUERY_INTERFACE     QueryInterface);

    static NTSTATUS
    DdiQueryChildRelations(
        IN_CONST_PVOID                  MiniportDeviceContext,
        INOUT_PDXGK_CHILD_DESCRIPTOR    ChildRelations,
        IN_ULONG                        ChildRelationsSize);

    static NTSTATUS
    DdiQueryChildStatus(
        IN_CONST_PVOID          MiniportDeviceContext,
        IN_PDXGK_CHILD_STATUS   ChildStatus,
        IN_BOOLEAN              NonDestructiveOnly);

    static NTSTATUS
    DdiQueryDeviceDescriptor(
        IN_CONST_PVOID                  MiniportDeviceContext,
        IN_ULONG                        ChildUid,
        INOUT_PDXGK_DEVICE_DESCRIPTOR   pDeviceDescriptor);

    static NTSTATUS
    DdiSetPowerState(
        IN_CONST_PVOID          MiniportDeviceContext,
        IN_ULONG                DeviceUid,
        IN_DEVICE_POWER_STATE   DevicePowerState,
        IN_POWER_ACTION         ActionType);

    static NTSTATUS
    DdiSetPowerComponentFState(
        IN_CONST_PVOID       MiniportDeviceContext,
        IN UINT              ComponentIndex,
        IN UINT              FState);

    static NTSTATUS
    DdiPowerRuntimeControlRequest(
        IN_CONST_PVOID       MiniportDeviceContext,
        IN LPCGUID           PowerControlCode,
        IN OPTIONAL PVOID    InBuffer,
        IN SIZE_T            InBufferSize,
        OUT OPTIONAL PVOID   OutBuffer,
        IN SIZE_T            OutBufferSize,
        OUT OPTIONAL PSIZE_T BytesReturned);

    static NTSTATUS
    DdiNotifyAcpiEvent(
        IN_CONST_PVOID      MiniportDeviceContext,
        IN_DXGK_EVENT_TYPE  EventType,
        IN_ULONG            Event,
        IN_PVOID            Argument,
        OUT_PULONG          AcpiFlags);

    static void
    DdiResetDevice(
        IN_CONST_PVOID  MiniportDeviceContext);

    static NTSTATUS
    DdiQueryCurrentFence(
        IN_CONST_HANDLE                     hAdapter,
        INOUT_PDXGKARG_QUERYCURRENTFENCE    pCurrentFence);

    static NTSTATUS
    DdiControlInterrupt(
        HANDLE const        hAdapter,
        DXGK_INTERRUPT_TYPE InterruptType,
        BOOLEAN             EnableInterrupt);

    static NTSTATUS
    DdiPresent(
        IN_CONST_HANDLE         hContext,
        INOUT_PDXGKARG_PRESENT  pPresent);

    static NTSTATUS
    DdiCreateContext(
        IN_CONST_HANDLE                 hDevice,
        INOUT_PDXGKARG_CREATECONTEXT    pCreateContext);

    static NTSTATUS
    DdiDestroyContext(
        IN_CONST_HANDLE hContext);

#ifdef DXGKDDI_INTERFACE_VERSION_WDDM2_4
    static NTSTATUS
    DdiRenderGdi(
        IN_CONST_HANDLE             hContext,
        INOUT_PDXGKARG_RENDERGDI    pRenderGdi);

    static NTSTATUS
    DdiRenderKm(
        IN_CONST_HANDLE         hContext,
        INOUT_PDXGKARG_RENDER   pRenderGdi);

    static NTSTATUS
    DdiSetVirtualMachineData(
        IN_CONST_HANDLE                         hAdapter,
        IN_CONST_PDXGKARG_SETVIRTUALMACHINEDATA Args);

    static NTSTATUS
    DdiBeginExclusiveAccess(
        IN_CONST_HANDLE                  hAdapter,
        IN_PDXGKARG_BEGINEXCLUSIVEACCESS pBeginExclusiveAccess);

    static NTSTATUS
    DdiEndExclusiveAccess(
        IN_CONST_HANDLE                hAdapter,
        IN_PDXGKARG_ENDEXCLUSIVEACCESS pEndExclusiveAccess);
#endif

public: // PAGED

    static DXGKDDI_OPENALLOCATIONINFO DdiOpenAllocation;
    static DXGKDDI_QUERYDEPENDENTENGINEGROUP DdiQueryDependentEngineGroup;

};


//
// DDIs that don't have to be registered in render only mode.
//
class GcKmdDisplayDdi
{
public:  // NONPAGED

    static DXGKDDI_SETVIDPNSOURCEADDRESS DdiSetVidPnSourceAddress;

private: // NONPAGED
public: // PAGED

    static DXGKDDI_SETPALETTE DdiSetPalette;
    static DXGKDDI_SETPOINTERPOSITION DdiSetPointerPosition;
    static DXGKDDI_SETPOINTERSHAPE DdiSetPointerShape;

    static DXGKDDI_ISSUPPORTEDVIDPN DdiIsSupportedVidPn;
    static DXGKDDI_RECOMMENDFUNCTIONALVIDPN DdiRecommendFunctionalVidPn;
    static DXGKDDI_ENUMVIDPNCOFUNCMODALITY DdiEnumVidPnCofuncModality;
    static DXGKDDI_SETVIDPNSOURCEVISIBILITY DdiSetVidPnSourceVisibility;
    static DXGKDDI_COMMITVIDPN DdiCommitVidPn;
    static DXGKDDI_UPDATEACTIVEVIDPNPRESENTPATH DdiUpdateActiveVidPnPresentPath;

    static DXGKDDI_RECOMMENDMONITORMODES DdiRecommendMonitorModes;
    static DXGKDDI_GETSCANLINE DdiGetScanLine;
    static DXGKDDI_QUERYVIDPNHWCAPABILITY DdiQueryVidPnHWCapability;
    static DXGKDDI_PRESENTDISPLAYONLY DdiPresentDisplayOnly;
    static DXGKDDI_STOP_DEVICE_AND_RELEASE_POST_DISPLAY_OWNERSHIP DdiStopDeviceAndReleasePostDisplayOwnership;

    static DXGKDDI_SETVIDPNSOURCEADDRESSWITHMULTIPLANEOVERLAY3 DdiSetVidPnSourceAddressWithMultiPlaneOverlay3;

    static DXGKDDI_UPDATEMONITORLINKINFO DdiUpdateMonitorLinkInfo;

    static DXGKDDI_GET_CHILD_CONTAINER_ID DdiGetChildContainerId;

private: // PAGED

};

