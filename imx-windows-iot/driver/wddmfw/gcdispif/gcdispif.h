/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

enum GcKmdDriverMode
{
    RenderOnly = 1,
    DisplayOnly = 2,
    FullDriver = 3
};

class GcKmDisplay
{
public:

    virtual NTSTATUS Start(
        DXGKRNL_INTERFACE  *pDxgkInterface,
        UINT                LocalVidMemPhysicalBase,
        ULONG              *pNumberOfVideoPresentSources,
        ULONG              *pNumberOfChildren) = NULL;

    virtual NTSTATUS Stop() = NULL;

    virtual NTSTATUS QueryAdapterInfo(
        CONST DXGKARG_QUERYADAPTERINFO *pQueryAdapterInfo) = NULL;

    virtual NTSTATUS QueryInterface(
        IN_PQUERY_INTERFACE QueryInterface) = NULL;

    virtual NTSTATUS QueryChildRelations(
        INOUT_PDXGK_CHILD_DESCRIPTOR    ChildRelations,
        IN_ULONG                        ChildRelationsSize) = NULL;

    virtual NTSTATUS QueryChildStatus(
        IN_PDXGK_CHILD_STATUS   ChildStatus,
        IN_BOOLEAN              NonDestructiveOnly) = NULL;

    virtual NTSTATUS QueryDeviceDescriptor(
        IN_ULONG                        ChildUid,
        INOUT_PDXGK_DEVICE_DESCRIPTOR   pDeviceDescriptor) = NULL;

    virtual NTSTATUS SetPowerState(
        IN_ULONG                DeviceUid,
        IN_DEVICE_POWER_STATE   DevicePowerState,
        IN_POWER_ACTION         ActionType) = NULL;

    virtual NTSTATUS SetPointerPosition(
        IN_CONST_PDXGKARG_SETPOINTERPOSITION    pSetPointerPosition) = NULL;

    virtual NTSTATUS SetPointerShape(
        IN_CONST_PDXGKARG_SETPOINTERSHAPE   pSetPointerShape) = NULL;

    virtual NTSTATUS IsSupportedVidPn(
        INOUT_PDXGKARG_ISSUPPORTEDVIDPN pIsSupportedVidPn) = NULL;

    virtual NTSTATUS RecommendFunctionalVidPn (
        IN_CONST_PDXGKARG_RECOMMENDFUNCTIONALVIDPN_CONST    pRecommendFunctionalVidPn) = NULL;

    virtual NTSTATUS EnumVidPnCofuncModality (
        IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality) = NULL;

    virtual NTSTATUS SetVidPnSourceAddress(
        IN_CONST_PDXGKARG_SETVIDPNSOURCEADDRESS pSetVidPnSourceAddress) = NULL;

    virtual NTSTATUS SetVidPnSourceVisibility(
        IN_CONST_PDXGKARG_SETVIDPNSOURCEVISIBILITY  pSetVidPnSourceVisibility) = NULL;

    virtual NTSTATUS CommitVidPn(
        IN_CONST_PDXGKARG_COMMITVIDPN_CONST pCommitVidPn) = NULL;

    virtual NTSTATUS UpdateActiveVidPnPresentPath(
        IN_CONST_PDXGKARG_UPDATEACTIVEVIDPNPRESENTPATH_CONST    pUpdateActiveVidPnPresentPath) = NULL;

    virtual NTSTATUS RecommendMonitorModes(
        IN_CONST_PDXGKARG_RECOMMENDMONITORMODES_CONST   pRecommendMonitorMode) = NULL;

    virtual NTSTATUS GetScanLine(
        INOUT_PDXGKARG_GETSCANLINE  pGetScanLine) = NULL;

    virtual BOOLEAN InterruptRoutine(
        UINT    MessageNumber) = NULL;

    virtual NTSTATUS ControlInterrupt(
        IN_CONST_DXGK_INTERRUPT_TYPE    InterruptType,
        IN_BOOLEAN  EnableInterrupt) = NULL;

    virtual NTSTATUS QueryVidPnHWCapability(
        INOUT_PDXGKARG_QUERYVIDPNHWCAPABILITY   pVidPnHWCaps) = NULL;

    virtual NTSTATUS PresentDisplayOnly(
        IN_CONST_PDXGKARG_PRESENT_DISPLAYONLY  pPresentDisplayOnly) = NULL;

    virtual NTSTATUS StopDeviceAndReleasePostDisplayOwnership(
        _In_ D3DDDI_VIDEO_PRESENT_TARGET_ID TargetId,
        _Out_ PDXGK_DISPLAY_INFORMATION     pDisplayInfo) = NULL;

    virtual NTSTATUS SetVidPnSourceAddressWithMultiPlaneOverlay3(
        IN_OUT_PDXGKARG_SETVIDPNSOURCEADDRESSWITHMULTIPLANEOVERLAY3 pSetVidPnSourceAddressWithMpo3) = NULL;

    virtual NTSTATUS UpdateMonitorLinkInfo(
        INOUT_PDXGKARG_UPDATEMONITORLINKINFO    pUpdateMonitorLinkInfoArg) = NULL;

    virtual NTSTATUS GetChildContainerId(
        _In_ ULONG  ChildUid,
        _Inout_ PDXGK_CHILD_CONTAINER_ID    pChildChildContainId) = NULL;
};


extern "C"
{

DECLSPEC_IMPORT
NTSTATUS
DisplayStartController(
    GcKmdDriverMode     DriverMode,
    DXGKRNL_INTERFACE  *pDxgkInterface,
    UINT                LocalVidMemPhysicalBase,
    GcKmDisplay       **ppDisplay,
    ULONG              *pNumberOfVideoPresentSources,
    ULONG              *pNumberOfChildren);

typedef NTSTATUS (__stdcall *PFN_DisplayStartController)(GcKmdDriverMode, DXGKRNL_INTERFACE*, UINT, GcKmDisplay**, ULONG*, ULONG*);

DECLSPEC_IMPORT
void
DisplayStopController(
    GcKmDisplay    *pGcDisplay);

typedef void (__stdcall *PFN_DisplayStopController)(GcKmDisplay*);

}

