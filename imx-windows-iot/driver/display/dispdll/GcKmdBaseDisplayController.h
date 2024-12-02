/* Copyright (c) Microsoft Corporation.
 * Copyright 2023 NXP
   Licensed under the MIT License. */

#pragma once

#include "gcdispif.h"
#include "GcKmdBaseDisplay.h"


struct GcKmDisplayPipelineInfo
{
    GcKmBaseDisplay*        m_pDisplayPipeline;
    BOOLEAN                 m_bInterruptForStarting = false;
    BOOLEAN                 m_bStarted = false;
    ULONG                   m_NumSources = 0;
    ULONG                   m_NumChildren = 0;
};

struct GcKmDisplayTargetInfo
{
    GcKmBaseDisplay*        m_pDisplayPipeline;
    DXGK_CHILD_DESCRIPTOR   m_ChildDescriptor;
};

struct GcKmDisplayPathInfo
{
    GcKmBaseDisplay*                        m_pDisplayPipeline;
    D3DDDI_VIDEO_PRESENT_TARGET_ID          m_TargetId;
    D3DKMDT_VIDPN_PRESENT_PATH_IMPORTANCE   m_Importance;
    D3DKMDT_VIDPN_SOURCE_MODE               m_CurSourceMode;
    D3DKMDT_VIDPN_TARGET_MODE               m_CurTargetMode;
};

class GcKmBaseDisplayController : public GcKmDisplay
{
public:
    GcKmBaseDisplayController(
        DXGKRNL_INTERFACE*  pDxgkInterface)
    {
        m_IsInitialized = TRUE;
    }

    virtual ~GcKmBaseDisplayController() {}

    //
    // GcKmDisplay interface implementation
    //

    virtual NTSTATUS Start(
        DXGKRNL_INTERFACE  *pDxgkInterface,
        UINT                LocalVidMemPhysicalBase,
        ULONG              *pNumberOfVideoPresentSources,
        ULONG              *pNumberOfChildren);

    virtual NTSTATUS Stop();

    virtual NTSTATUS QueryAdapterInfo(
        const DXGKARG_QUERYADAPTERINFO *pQueryAdapterInfo);

    virtual NTSTATUS QueryInterface(
        IN_PQUERY_INTERFACE QueryInterface);

    virtual NTSTATUS QueryChildRelations(
        INOUT_PDXGK_CHILD_DESCRIPTOR    ChildRelations,
        IN_ULONG                        ChildRelationsSize);

    virtual NTSTATUS QueryChildStatus(
        IN_PDXGK_CHILD_STATUS   ChildStatus,
        IN_BOOLEAN              NonDestructiveOnly);

    virtual NTSTATUS QueryDeviceDescriptor(
        IN_ULONG                        ChildUid,
        INOUT_PDXGK_DEVICE_DESCRIPTOR   pDeviceDescriptor);

    virtual NTSTATUS SetPowerState(
        IN_ULONG                DeviceUid,
        IN_DEVICE_POWER_STATE   DevicePowerState,
        IN_POWER_ACTION         ActionType);

    virtual NTSTATUS SetPointerPosition(
        IN_CONST_PDXGKARG_SETPOINTERPOSITION    pSetPointerPosition);

    virtual NTSTATUS SetPointerShape(
        IN_CONST_PDXGKARG_SETPOINTERSHAPE   pSetPointerShape);

    virtual NTSTATUS IsSupportedVidPn(
        INOUT_PDXGKARG_ISSUPPORTEDVIDPN pIsSupportedVidPn);

    virtual NTSTATUS RecommendFunctionalVidPn(
        IN_CONST_PDXGKARG_RECOMMENDFUNCTIONALVIDPN_CONST    pRecommendFunctionalVidPn);

    virtual NTSTATUS EnumVidPnCofuncModality(
        IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality);

    virtual NTSTATUS SetVidPnSourceAddress(
        IN_CONST_PDXGKARG_SETVIDPNSOURCEADDRESS pSetVidPnSourceAddress);

    virtual NTSTATUS SetVidPnSourceVisibility(
        IN_CONST_PDXGKARG_SETVIDPNSOURCEVISIBILITY  pSetVidPnSourceVisibility);

    virtual NTSTATUS CommitVidPn(
        IN_CONST_PDXGKARG_COMMITVIDPN_CONST pCommitVidPn);

    virtual NTSTATUS UpdateActiveVidPnPresentPath(
        IN_CONST_PDXGKARG_UPDATEACTIVEVIDPNPRESENTPATH_CONST    pUpdateActiveVidPnPresentPath);

    virtual NTSTATUS RecommendMonitorModes(
        IN_CONST_PDXGKARG_RECOMMENDMONITORMODES_CONST   pRecommendMonitorMode);

    virtual NTSTATUS GetScanLine(
        INOUT_PDXGKARG_GETSCANLINE  pGetScanLine);

    virtual BOOLEAN InterruptRoutine(
        UINT    MessageNumber);

    virtual NTSTATUS ControlInterrupt(
        IN_CONST_DXGK_INTERRUPT_TYPE    InterruptType,
        IN_BOOLEAN  EnableInterrupt);

    virtual NTSTATUS QueryVidPnHWCapability(
        INOUT_PDXGKARG_QUERYVIDPNHWCAPABILITY   pVidPnHWCaps);

    virtual NTSTATUS PresentDisplayOnly(
        IN_CONST_PDXGKARG_PRESENT_DISPLAYONLY  pPresentDisplayOnly);

    virtual NTSTATUS StopDeviceAndReleasePostDisplayOwnership(
        _In_ D3DDDI_VIDEO_PRESENT_TARGET_ID TargetId,
        _Out_ PDXGK_DISPLAY_INFORMATION     pDisplayInfo);

    virtual NTSTATUS SetVidPnSourceAddressWithMultiPlaneOverlay3(
        IN_OUT_PDXGKARG_SETVIDPNSOURCEADDRESSWITHMULTIPLANEOVERLAY3 pSetVidPnSourceAddressWithMpo3);

    virtual NTSTATUS UpdateMonitorLinkInfo(
        INOUT_PDXGKARG_UPDATEMONITORLINKINFO    pUpdateMonitorLinkInfo);

    virtual NTSTATUS GetChildContainerId(
        _In_ ULONG  ChildUid,
        _Inout_ PDXGK_CHILD_CONTAINER_ID    pChildChildContainId);

    virtual BOOLEAN IsInitialized(VOID)
    {
        return m_IsInitialized;
    }

protected:

    NTSTATUS RegisterDisplayPipeline(
        GcKmBaseDisplay*    pDisplay,
        BOOLEAN             bInterruptForStarting = false);

    GcKmBaseDisplay* FindDisplayPipeline(D3DDDI_VIDEO_PRESENT_TARGET_ID TargetId)
    {
        for (UINT i = 0; i < m_NumTargets; i++)
        {
            if (TargetId == m_Targets[i].m_ChildDescriptor.ChildUid)
            {
                return m_Targets[i].m_pDisplayPipeline;
            }
        }

        return nullptr;
    }

protected:

    BOOLEAN m_IsInitialized;

    static const UINT   GC_KM_MAX_PIPELINES = 8;
    static const UINT   GC_KM_MAX_SOURCES   = 16;
    static const UINT   GC_KM_MAX_TARGETS   = 32;
    static const UINT   GC_KM_MAX_CLONES    = 3;

    DXGKRNL_INTERFACE*  m_pDxgkInterface;

    // Each type of display pipeline can support multiple VidPn targets/monitors
    GcKmDisplayPipelineInfo     m_Pipelines[GC_KM_MAX_PIPELINES] = {};
    UINT                        m_NumPipelines = 0;

    // Simple list of VidPn targets and their Target Id
    GcKmDisplayTargetInfo       m_Targets[GC_KM_MAX_TARGETS] = {};
    UINT                        m_NumTargets = 0;

    // Active display/VidPn targets ordered by Source Id
    GcKmDisplayPathInfo         m_Paths[GC_KM_MAX_SOURCES][GC_KM_MAX_CLONES] = {};
    UINT                        m_NumSources = 0;
};
