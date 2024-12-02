/* Copyright (c) Microsoft Corporation.
 * Copyright 2022-2023 NXP
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmdLogging.h"
#include "GcKmdVidPnMode.tmh"

#include "GcKmdBaseDisplay.h"
#include "GcKmdGlobal.h"

#include "GcKmdUtil.h"
#include "GcKmdGuard.h"
#include "GcKmdErroHandling.h"

GC_PAGED_SEGMENT_BEGIN; //======================================================

BOOLEAN g_bUsePreviousPostDisplayInfo = FALSE;

extern videomode g_StandardModeTimings[];
extern unsigned int g_NumStandardModeTimings;


NTSTATUS
GcKmBaseDisplay::EnumVidPnCofuncModality(
    IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality)
{
    // Get VidPN interface
    const DXGK_VIDPN_INTERFACE* VidPnInterface;
    auto Status = m_pDxgkInterface->DxgkCbQueryVidPnInterface(
        pEnumCofuncModality->hConstrainingVidPn,
        DXGK_VIDPN_INTERFACE_VERSION_V1,
        &VidPnInterface);
    RETURN_ON_FAILURE(Status);

    // Get the topology
    D3DKMDT_HVIDPNTOPOLOGY Topology;
    DXGK_VIDPNTOPOLOGY_INTERFACE const * TopologyInterface;
    Status = VidPnInterface->pfnGetTopology(
        pEnumCofuncModality->hConstrainingVidPn,
        &Topology,
        &TopologyInterface);
    RETURN_ON_FAILURE(Status);

    return ProcessVidPnPaths(pEnumCofuncModality, VidPnInterface, Topology, TopologyInterface);
}

NTSTATUS
GcKmBaseDisplay::ProcessVidPnPaths(
    IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality,
    const DXGK_VIDPN_INTERFACE* pVidPnInterface,
    D3DKMDT_HVIDPNTOPOLOGY hVidPnTopology,
    DXGK_VIDPNTOPOLOGY_INTERFACE const* pTopologyInterface)
{
    const D3DKMDT_VIDPN_PRESENT_PATH* pPath;
    auto Status = pTopologyInterface->pfnAcquireFirstPathInfo(hVidPnTopology, &pPath);
    RETURN_ON_FAILURE(Status);

    while (NT_SUCCESS(Status))
    {
        NT_ASSERT(pPath);
        auto PathGuard = MakeScopeExitGuard([&, pPath]()
            {
                auto Status = pTopologyInterface->pfnReleasePathInfo(hVidPnTopology, pPath);
                DEBUG_CHECK(Status);
            });

        Status = ProcessSourceModeSet(pEnumCofuncModality, pVidPnInterface, pPath);
        RETURN_ON_FAILURE(Status);

        Status = ProcessTargetModeSet(pEnumCofuncModality, pVidPnInterface, pPath);
        RETURN_ON_FAILURE(Status);

        Status = ProcessVidPnPathAttributes(pEnumCofuncModality,pPath, hVidPnTopology, pTopologyInterface);
        RETURN_ON_FAILURE(Status);

        const D3DKMDT_VIDPN_PRESENT_PATH* pNextPath;
        Status = pTopologyInterface->pfnAcquireNextPathInfo(hVidPnTopology, pPath, &pNextPath);
        if (Status == STATUS_GRAPHICS_NO_MORE_ELEMENTS_IN_DATASET)
        {
            // No more paths to process.
            break;
        }
        RETURN_ON_FAILURE(Status);

        pPath = pNextPath;
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmBaseDisplay::ProcessSourceModeSet(
    IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality,
    const DXGK_VIDPN_INTERFACE* pVidPnInterface,
    const D3DKMDT_VIDPN_PRESENT_PATH* pVidPnPath)
{
    BOOLEAN bModeSetNeedsUpdate;
    auto Status = SourceModeSetNeedsUpdate(pEnumCofuncModality, pVidPnInterface, pVidPnPath, &bModeSetNeedsUpdate);
    RETURN_ON_FAILURE(Status);

    if (bModeSetNeedsUpdate)
    {
        const D3DDDIFORMAT* pFormatList;
        UINT NumFormats = GetFormatList(&pFormatList);

        Status = AddNewSourceModeSet(pVidPnInterface, pEnumCofuncModality->hConstrainingVidPn, pVidPnPath->VidPnSourceId, SupportStandardModes(), pFormatList, NumFormats);
        RETURN_ON_FAILURE(Status);
    }

    return STATUS_SUCCESS;
}


NTSTATUS
GcKmBaseDisplay::ProcessTargetModeSet(
    IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality,
    const DXGK_VIDPN_INTERFACE* pVidPnInterface,
    const D3DKMDT_VIDPN_PRESENT_PATH* pVidPnPath)
{
    BOOLEAN bModeSetNeedsUpdate;
    auto Status = TargetModeSetNeedsUpdate(pEnumCofuncModality, pVidPnInterface, pVidPnPath, &bModeSetNeedsUpdate);
    RETURN_ON_FAILURE(Status);

    if (bModeSetNeedsUpdate)
    {
        Status = AddNewTargetModeSet(pVidPnInterface, pEnumCofuncModality->hConstrainingVidPn, pVidPnPath->VidPnTargetId, SupportStandardModes());
        RETURN_ON_FAILURE(Status);
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmBaseDisplay::ProcessVidPnPathAttributes(
    IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality,
    const D3DKMDT_VIDPN_PRESENT_PATH* pVidPnPath,
    D3DKMDT_HVIDPNTOPOLOGY hVidPnTopology,
    DXGK_VIDPNTOPOLOGY_INTERFACE const* pTopologyInterface)
{
    D3DKMDT_VIDPN_PRESENT_PATH ModifiedVidPnPath = *pVidPnPath; // Copy of VidPN path.
    BOOLEAN bVidPnPathNeedsUpdate = FALSE;

    if (ScalingNeedsUpdate(pEnumCofuncModality, pVidPnPath))
    {
        ModifiedVidPnPath.ContentTransformation.ScalingSupport = D3DKMDT_VIDPN_PRESENT_PATH_SCALING_SUPPORT{};
        ModifiedVidPnPath.ContentTransformation.ScalingSupport.Identity = TRUE;
        bVidPnPathNeedsUpdate = TRUE;
    }

    if (RotationNeedsUpdate(pEnumCofuncModality, pVidPnPath))
    {
        ModifiedVidPnPath.ContentTransformation.RotationSupport = D3DKMDT_VIDPN_PRESENT_PATH_ROTATION_SUPPORT{};
        ModifiedVidPnPath.ContentTransformation.RotationSupport.Identity = TRUE;
        ModifiedVidPnPath.ContentTransformation.RotationSupport.Offset0 = TRUE;
        bVidPnPathNeedsUpdate = TRUE;
    }

    if (bVidPnPathNeedsUpdate)
    {
        auto Status = pTopologyInterface->pfnUpdatePathSupportInfo(hVidPnTopology, &ModifiedVidPnPath);
        RETURN_ON_FAILURE(Status);
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmBaseDisplay::AddNewSourceModeSet(
    const DXGK_VIDPN_INTERFACE* pVidPnInterface,
    D3DKMDT_HVIDPN hVidPn,
    D3DDDI_VIDEO_PRESENT_SOURCE_ID VidPnSourceId,
    bool bSupportStandardModes,
    const D3DDDIFORMAT* pFormatList,
    UINT NumFormats)
{
    // Create a new source mode set.
    D3DKMDT_HVIDPNSOURCEMODESET hSourceModeSet = 0;
    const DXGK_VIDPNSOURCEMODESET_INTERFACE* pSourceModeSetInterface = NULL;
    auto Status = pVidPnInterface->pfnCreateNewSourceModeSet(
        hVidPn,
        VidPnSourceId,
        &hSourceModeSet,
        &pSourceModeSetInterface);
    RETURN_ON_FAILURE(Status);

    auto ModeSetGuard = MakeScopeExitGuard([&]()
        {
            auto Status = pVidPnInterface->pfnReleaseSourceModeSet(hVidPn, hSourceModeSet);
            DEBUG_CHECK(Status);
        });

    Status = AddNewSourceModeInfo(hSourceModeSet, pSourceModeSetInterface, bSupportStandardModes, pFormatList, NumFormats);
    RETURN_ON_FAILURE(Status);

    Status = pVidPnInterface->pfnAssignSourceModeSet(
        hVidPn,
        VidPnSourceId,
        hSourceModeSet);
    RETURN_ON_FAILURE(Status);

    ModeSetGuard.Deactivate();
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmBaseDisplay::AddNewTargetModeSet(
    const DXGK_VIDPN_INTERFACE* pVidPnInterface,
    D3DKMDT_HVIDPN hVidPn,
    D3DKMDT_VIDEO_PRESENT_TARGET_MODE_ID VidPnTargeId,
    bool bSupportStandardModes)
{
    // Create a new target mode set.
    D3DKMDT_HVIDPNTARGETMODESET hTargetModeSet = 0;
    const DXGK_VIDPNTARGETMODESET_INTERFACE* pTargetModeSetInterface = NULL;
    auto Status = pVidPnInterface->pfnCreateNewTargetModeSet(
        hVidPn,
        VidPnTargeId,
        &hTargetModeSet,
        &pTargetModeSetInterface);
    RETURN_ON_FAILURE(Status);

    auto ModeSetGuard = MakeScopeExitGuard([&]()
        {
            auto Status = pVidPnInterface->pfnReleaseTargetModeSet(hVidPn, hTargetModeSet);
            DEBUG_CHECK(Status);
        });

    Status = AddNewTargetModeInfo(hTargetModeSet, pTargetModeSetInterface, bSupportStandardModes);
    RETURN_ON_FAILURE(Status);

    Status = pVidPnInterface->pfnAssignTargetModeSet(
        hVidPn,
        VidPnTargeId,
        hTargetModeSet);
    RETURN_ON_FAILURE(Status);

    ModeSetGuard.Deactivate();
    return STATUS_SUCCESS;
}

NTSTATUS
GcKmBaseDisplay::AddNewSourceModeInfo(
    D3DKMDT_HVIDPNSOURCEMODESET hSourceModeSet,
    const DXGK_VIDPNSOURCEMODESET_INTERFACE* pSourceModeSetInterface,
    bool bSupportStandardModes,
    const D3DDDIFORMAT* pFormatList,
    UINT NumFormats)
{
    D3DKMDT_VIDPN_SOURCE_MODE* pSourceModeInfo = NULL;
    auto Status = pSourceModeSetInterface->pfnCreateNewModeInfo(hSourceModeSet, &pSourceModeInfo);
    RETURN_ON_FAILURE(Status);

    auto ModeInfoGuard = MakeScopeExitGuard([&]()
    {
        auto Status = pSourceModeSetInterface->pfnReleaseModeInfo(hSourceModeSet, pSourceModeInfo);
        DEBUG_CHECK(Status);
    });

    if (g_bUsePreviousPostDisplayInfo)
    {
        SetSourceModeInfoToPreviousPostDisplayInfo(pSourceModeInfo);
    }
    else
    {
        SetSourceModeInfoToNative(pSourceModeInfo);
    }

    const D3DKMDT_VIDPN_SOURCE_MODE BaseSourceModeInfo = *pSourceModeInfo;

    Status = pSourceModeSetInterface->pfnAddMode(hSourceModeSet, pSourceModeInfo);
    RETURN_ON_FAILURE(Status);

    ModeInfoGuard.Deactivate();

    const D3DKMDT_2DREGION BaseModeSize = BaseSourceModeInfo.Format.Graphics.PrimSurfSize;

    int i = g_NumStandardModeTimings - 1;

    do
    {
        UINT FormatStart = 0;

        if ((g_StandardModeTimings[i].hactive == BaseModeSize.cx) &&
            (g_StandardModeTimings[i].vactive == BaseModeSize.cy))
        {
            FormatStart = 1;
        }

        if ((g_StandardModeTimings[i].hactive > BaseModeSize.cx) ||
            (g_StandardModeTimings[i].vactive > BaseModeSize.cy))
        {
            continue;
        }

        D3DKMDT_VIDPN_SOURCE_MODE SourceMode;
        auto& Graphics = SourceMode.Format.Graphics;

        SourceMode.Type = BaseSourceModeInfo.Type;
        Graphics = BaseSourceModeInfo.Format.Graphics;

        Graphics.PrimSurfSize.cx = g_StandardModeTimings[i].hactive;
        Graphics.PrimSurfSize.cy = g_StandardModeTimings[i].vactive;

        Graphics.VisibleRegionSize = Graphics.PrimSurfSize;

        Graphics.Stride = Graphics.PrimSurfSize.cx*4;

        D3DKMDT_VIDPN_SOURCE_MODE* pSourceModeInfo = NULL;

        for (UINT j = FormatStart; j < NumFormats; j++)
        {
            Status = pSourceModeSetInterface->pfnCreateNewModeInfo(hSourceModeSet, &pSourceModeInfo);
            if (STATUS_SUCCESS != Status)
            {
                break;
            }

            auto Id = pSourceModeInfo->Id;
            *pSourceModeInfo = SourceMode;
            pSourceModeInfo->Id = Id;

            pSourceModeInfo->Format.Graphics.PixelFormat = pFormatList[j];

            Status = pSourceModeSetInterface->pfnAddMode(hSourceModeSet, pSourceModeInfo);
            if (STATUS_SUCCESS != Status)
            {
                pSourceModeSetInterface->pfnReleaseModeInfo(hSourceModeSet, pSourceModeInfo);
                break;
            }
        }

        if (STATUS_SUCCESS != Status)
        {
            break;
        }

        if ((FormatStart > 0) && (!bSupportStandardModes))
        {
            break;
        }
    } while (--i >= 0);

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmBaseDisplay::AddNewTargetModeInfo(
    D3DKMDT_HVIDPNTARGETMODESET hTargetModeSet,
    const DXGK_VIDPNTARGETMODESET_INTERFACE* pTargetModeSetInterface,
    bool bSupportStandardModes)
{
    D3DKMDT_VIDPN_TARGET_MODE* pTargetModeInfo = NULL;
    auto Status = pTargetModeSetInterface->pfnCreateNewModeInfo(hTargetModeSet, &pTargetModeInfo);
    RETURN_ON_FAILURE(Status);

    auto ModeInfoGuard = MakeScopeExitGuard([&]()
    {
        auto Status = pTargetModeSetInterface->pfnReleaseModeInfo(hTargetModeSet, pTargetModeInfo);
        DEBUG_CHECK(Status);
    });

    if (g_bUsePreviousPostDisplayInfo)
    {
        SetTargetModeInfoToPreviousPostDisplayInfo(pTargetModeInfo);
    }
    else
    {
        SetTargetModeInfoToNative(pTargetModeInfo);
    }

    const D3DKMDT_VIDPN_TARGET_MODE BaseTargetModeInfo = *pTargetModeInfo;

    Status = pTargetModeSetInterface->pfnAddMode(hTargetModeSet, pTargetModeInfo);
    RETURN_ON_FAILURE(Status);

    ModeInfoGuard.Deactivate();

    if (!bSupportStandardModes)
    {
        return STATUS_SUCCESS;
    }

    int i = g_NumStandardModeTimings - 1;

    const D3DKMDT_2DREGION BaseModeSize = BaseTargetModeInfo.VideoSignalInfo.ActiveSize;

    do
    {
        if ((g_StandardModeTimings[i].hactive == BaseModeSize.cx) ||
            (g_StandardModeTimings[i].vactive == BaseModeSize.cy))
        {
            continue;
        }

        if ((g_StandardModeTimings[i].hactive > BaseModeSize.cx) ||
            (g_StandardModeTimings[i].vactive > BaseModeSize.cy))
        {
            continue;
        }

        D3DKMDT_VIDPN_TARGET_MODE* pTargetModeInfo = NULL;
        auto Status = pTargetModeSetInterface->pfnCreateNewModeInfo(hTargetModeSet, &pTargetModeInfo);
        if (STATUS_SUCCESS != Status)
        {
            break;
        }

        auto pStandardModeTiming = &g_StandardModeTimings[i];

        pTargetModeInfo->WireFormatAndPreference = BaseTargetModeInfo.WireFormatAndPreference;
        pTargetModeInfo->Preference = D3DKMDT_MP_NOTPREFERRED;

        pTargetModeInfo->VideoSignalInfo = BaseTargetModeInfo.VideoSignalInfo;

        pTargetModeInfo->VideoSignalInfo.TotalSize.cx = pStandardModeTiming->hactive +
                                                        pStandardModeTiming->hfront_porch +
                                                        pStandardModeTiming->hsync_len + 
                                                        pStandardModeTiming->hback_porch;
        pTargetModeInfo->VideoSignalInfo.TotalSize.cy = pStandardModeTiming->vactive +
                                                        pStandardModeTiming->vfront_porch +
                                                        pStandardModeTiming->vsync_len + 
                                                        pStandardModeTiming->vback_porch;

        pTargetModeInfo->VideoSignalInfo.ActiveSize.cx = pStandardModeTiming->hactive;
        pTargetModeInfo->VideoSignalInfo.ActiveSize.cy = pStandardModeTiming->vactive;

        if ((FullDriver == GcKmdGlobal::s_DriverMode) || (TRUE == GcKmdGlobal::s_bDodUseHwVSync))
        {
            pTargetModeInfo->VideoSignalInfo.VideoStandard = D3DKMDT_VSS_VESA_DMT;

            pTargetModeInfo->VideoSignalInfo.PixelRate = g_StandardModeTimings[i].pixelclock;
            pTargetModeInfo->VideoSignalInfo.HSyncFreq.Numerator = (UINT)pTargetModeInfo->VideoSignalInfo.PixelRate;
            pTargetModeInfo->VideoSignalInfo.HSyncFreq.Denominator = pTargetModeInfo->VideoSignalInfo.TotalSize.cx;
            pTargetModeInfo->VideoSignalInfo.VSyncFreq.Numerator = (UINT)pTargetModeInfo->VideoSignalInfo.PixelRate;
            pTargetModeInfo->VideoSignalInfo.VSyncFreq.Denominator = pTargetModeInfo->VideoSignalInfo.TotalSize.cx *
                                                                     pTargetModeInfo->VideoSignalInfo.TotalSize.cy;
        }

        Status = pTargetModeSetInterface->pfnAddMode(hTargetModeSet, pTargetModeInfo);
        if (STATUS_SUCCESS != Status)
        {
            pTargetModeSetInterface->pfnReleaseModeInfo(hTargetModeSet, pTargetModeInfo);
            break;
        }
    } while (--i >= 0);

    return STATUS_SUCCESS;
}

void
GcKmBaseDisplay::SetSourceModeInfoToNative(D3DKMDT_VIDPN_SOURCE_MODE* pModeInfo)
{
    pModeInfo->Type = D3DKMDT_RMT_GRAPHICS;

    auto& Format = pModeInfo->Format.Graphics;
    const auto& ActiveSize = m_NativeMonitorMode.VideoSignalInfo.ActiveSize;

    Format.PrimSurfSize = Format.VisibleRegionSize = ActiveSize;
    Format.Stride = ActiveSize.cx * 4;
    Format.PixelFormat = D3DDDIFMT_A8R8G8B8;
    Format.ColorBasis = m_NativeMonitorMode.ColorBasis;
    Format.PixelValueAccessMode = D3DKMDT_PVAM_DIRECT;
}

void
GcKmBaseDisplay::SetSourceModeInfoToPreviousPostDisplayInfo(D3DKMDT_VIDPN_SOURCE_MODE* pModeInfo)
{
    pModeInfo->Type = D3DKMDT_RMT_GRAPHICS;

    auto& Format = pModeInfo->Format.Graphics;

    Format.PrimSurfSize.cx = m_PreviousPostDisplayInfo.Width;
    Format.PrimSurfSize.cy = m_PreviousPostDisplayInfo.Height;
    Format.VisibleRegionSize = Format.PrimSurfSize;
    Format.Stride = m_PreviousPostDisplayInfo.Pitch;
    Format.PixelFormat = D3DDDIFMT_A8R8G8B8;
    Format.ColorBasis = D3DKMDT_CB_SCRGB;
    Format.PixelValueAccessMode = D3DKMDT_PVAM_DIRECT;
}

void
GcKmBaseDisplay::SetTargetModeInfoToNative(D3DKMDT_VIDPN_TARGET_MODE* pModeInfo)
{
    if ((FullDriver == GcKmdGlobal::s_DriverMode) || (TRUE == GcKmdGlobal::s_bDodUseHwVSync))
    {
        pModeInfo->VideoSignalInfo = m_NativeMonitorMode.VideoSignalInfo;
    }
    else
    {
        auto& SignalInfo = pModeInfo->VideoSignalInfo;

        SignalInfo.VideoStandard = D3DKMDT_VSS_OTHER;

        SignalInfo.TotalSize = m_NativeMonitorMode.VideoSignalInfo.TotalSize;
        SignalInfo.ActiveSize = m_NativeMonitorMode.VideoSignalInfo.ActiveSize;

        SignalInfo.VSyncFreq.Numerator = D3DKMDT_FREQUENCY_NOTSPECIFIED;
        SignalInfo.VSyncFreq.Denominator = D3DKMDT_FREQUENCY_NOTSPECIFIED;

        SignalInfo.HSyncFreq.Numerator = D3DKMDT_FREQUENCY_NOTSPECIFIED;
        SignalInfo.HSyncFreq.Denominator = D3DKMDT_FREQUENCY_NOTSPECIFIED;

        SignalInfo.PixelRate = D3DKMDT_FREQUENCY_NOTSPECIFIED;
        SignalInfo.ScanLineOrdering = D3DDDI_VSSLO_PROGRESSIVE;
    }

    pModeInfo->Preference = D3DKMDT_MP_PREFERRED;
}

void
GcKmBaseDisplay::SetTargetModeInfoToPreviousPostDisplayInfo(D3DKMDT_VIDPN_TARGET_MODE* pModeInfo)
{
    auto& SignalInfo = pModeInfo->VideoSignalInfo;

    SignalInfo.VideoStandard = D3DKMDT_VSS_OTHER;

    SignalInfo.TotalSize.cx = m_PreviousPostDisplayInfo.Width;
    SignalInfo.TotalSize.cy = m_PreviousPostDisplayInfo.Height;
    SignalInfo.ActiveSize = SignalInfo.TotalSize;

    if ((DisplayOnly == GcKmdGlobal::s_DriverMode) && (FALSE == GcKmdGlobal::s_bDodUseHwVSync))
    {
        SignalInfo.VSyncFreq.Numerator = D3DKMDT_FREQUENCY_NOTSPECIFIED;
        SignalInfo.VSyncFreq.Denominator = D3DKMDT_FREQUENCY_NOTSPECIFIED;

        SignalInfo.HSyncFreq.Numerator = D3DKMDT_FREQUENCY_NOTSPECIFIED;
        SignalInfo.HSyncFreq.Denominator = D3DKMDT_FREQUENCY_NOTSPECIFIED;

        SignalInfo.PixelRate = D3DKMDT_FREQUENCY_NOTSPECIFIED;
        SignalInfo.ScanLineOrdering = D3DDDI_VSSLO_PROGRESSIVE;
    }
    else
    {
        //
        // Assuming refresh rate of 60 Hz
        //

        SignalInfo.VSyncFreq.Numerator = 60;
        SignalInfo.VSyncFreq.Denominator = 1;

        SignalInfo.HSyncFreq.Numerator = SignalInfo.TotalSize.cy*SignalInfo.VSyncFreq.Numerator/SignalInfo.VSyncFreq.Denominator;
        SignalInfo.HSyncFreq.Denominator = 1;

        SignalInfo.PixelRate = SignalInfo.TotalSize.cx*SignalInfo.HSyncFreq.Numerator/SignalInfo.HSyncFreq.Denominator;
        SignalInfo.ScanLineOrdering = D3DDDI_VSSLO_PROGRESSIVE;
    }

    pModeInfo->Preference = D3DKMDT_MP_PREFERRED;
}

NTSTATUS
GcKmBaseDisplay::SourceModeSetNeedsUpdate(
    IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality,
    const DXGK_VIDPN_INTERFACE* pVidPnInterface,
    const D3DKMDT_VIDPN_PRESENT_PATH* pVidPnPath,
    BOOLEAN* bNeedsUpdate)
{
    *bNeedsUpdate = FALSE;

    if (pEnumCofuncModality->EnumPivotType == D3DKMDT_EPT_VIDPNSOURCE
        && pEnumCofuncModality->EnumPivot.VidPnSourceId == pVidPnPath->VidPnSourceId)
    {
        // Surce mode set is pivot. No updates are needed.
        return STATUS_SUCCESS;
    }

    BOOLEAN bHasPinnedMode;
    auto Status = SourceHasPinnedMode(
        pEnumCofuncModality->hConstrainingVidPn,
        pVidPnInterface,
        pVidPnPath,
        &bHasPinnedMode);
    RETURN_ON_FAILURE(Status);

    if (!bHasPinnedMode)
    {
        // Source mode set has no pinned mode. Updates are needed.
        *bNeedsUpdate = TRUE;
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmBaseDisplay::TargetModeSetNeedsUpdate(
    IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality,
    const DXGK_VIDPN_INTERFACE* pVidPnInterface,
    const D3DKMDT_VIDPN_PRESENT_PATH* pVidPnPath,
    BOOLEAN* bNeedsUpdate)
{
    *bNeedsUpdate = FALSE;

    if (pEnumCofuncModality->EnumPivotType == D3DKMDT_EPT_VIDPNTARGET
        && pEnumCofuncModality->EnumPivot.VidPnTargetId == pVidPnPath->VidPnTargetId)
    {
        // Target mode set is pivot. No updates are needed.
        return STATUS_SUCCESS;
    }

    BOOLEAN bHasPinnedMode;
    auto Status = TargetHasPinnedMode(
        pEnumCofuncModality->hConstrainingVidPn,
        pVidPnInterface,
        pVidPnPath,
        &bHasPinnedMode);
    RETURN_ON_FAILURE(Status);

    if (!bHasPinnedMode)
    {
        // Source mode set has no pinned mode. Updates are needed.
        *bNeedsUpdate = TRUE;
    }

    return STATUS_SUCCESS;
}

BOOLEAN
GcKmBaseDisplay::ScalingNeedsUpdate(
    IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality,
    const D3DKMDT_VIDPN_PRESENT_PATH* pVidPnPath)
{
    return pEnumCofuncModality->EnumPivotType != D3DKMDT_EPT_SCALING
        || pEnumCofuncModality->EnumPivot.VidPnSourceId != pVidPnPath->VidPnSourceId
        || pEnumCofuncModality->EnumPivot.VidPnTargetId != pVidPnPath->VidPnTargetId
        || pVidPnPath->ContentTransformation.Scaling == D3DKMDT_VPPS_UNPINNED;
}

BOOLEAN
GcKmBaseDisplay::RotationNeedsUpdate(
    IN_CONST_PDXGKARG_ENUMVIDPNCOFUNCMODALITY_CONST pEnumCofuncModality,
    const D3DKMDT_VIDPN_PRESENT_PATH* pVidPnPath)
{
    return pEnumCofuncModality->EnumPivotType != D3DKMDT_EPT_ROTATION
        || pEnumCofuncModality->EnumPivot.VidPnSourceId != pVidPnPath->VidPnSourceId
        || pEnumCofuncModality->EnumPivot.VidPnTargetId != pVidPnPath->VidPnTargetId
        || pVidPnPath->ContentTransformation.Rotation == D3DKMDT_VPPR_UNPINNED;
}

NTSTATUS
GcKmBaseDisplay::SourceHasPinnedMode(
    D3DKMDT_HVIDPN hVidPn,
    const DXGK_VIDPN_INTERFACE* pVidPnInterface,
    const D3DKMDT_VIDPN_PRESENT_PATH* pVidPnPath,
    BOOLEAN* bHasPinnedMode)
{
    *bHasPinnedMode = FALSE;

    // Get source mode set.
    D3DKMDT_HVIDPNSOURCEMODESET hSourceModeSet = 0;
    const DXGK_VIDPNSOURCEMODESET_INTERFACE* pSourceModeSetInterface = NULL;
    auto Status = pVidPnInterface->pfnAcquireSourceModeSet(
        hVidPn,
        pVidPnPath->VidPnSourceId,
        &hSourceModeSet,
        &pSourceModeSetInterface);
    RETURN_ON_FAILURE(Status);

    auto SourceModeSetGuard = MakeScopeExitGuard([&]()
        {
            auto Status = pVidPnInterface->pfnReleaseSourceModeSet(hVidPn, hSourceModeSet);
            DEBUG_CHECK(Status);
        });

    // Get pinned mode from source mode set.
    const D3DKMDT_VIDPN_SOURCE_MODE* pPinnedSourceModeInfo = NULL;
    Status = pSourceModeSetInterface->pfnAcquirePinnedModeInfo(hSourceModeSet, &pPinnedSourceModeInfo);
    RETURN_ON_FAILURE(Status);

    auto PinnedSourceModeInfoGuard = MakeScopeExitGuard([&]()
        {
            if (NULL != pPinnedSourceModeInfo)
            {
                pSourceModeSetInterface->pfnReleaseModeInfo(hSourceModeSet, pPinnedSourceModeInfo);
            }
        });

    if (NULL != pPinnedSourceModeInfo)
    {
        *bHasPinnedMode = TRUE;
    }

    return STATUS_SUCCESS;
}


NTSTATUS
GcKmBaseDisplay::TargetHasPinnedMode(
    D3DKMDT_HVIDPN hVidPn,
    const DXGK_VIDPN_INTERFACE* pVidPnInterface,
    const D3DKMDT_VIDPN_PRESENT_PATH* pVidPnPath,
    BOOLEAN* bHasPinnedMode)
{
    *bHasPinnedMode = FALSE;

    // Get target mode set.
    D3DKMDT_HVIDPNTARGETMODESET hTargetModeSet = 0;
    const DXGK_VIDPNTARGETMODESET_INTERFACE* pTargetModeSetInterface = NULL;
    auto Status = pVidPnInterface->pfnAcquireTargetModeSet(
        hVidPn,
        pVidPnPath->VidPnTargetId,
        &hTargetModeSet,
        &pTargetModeSetInterface);
    RETURN_ON_FAILURE(Status);

    auto TargetModeSetGuard = MakeScopeExitGuard([&]()
        {
            pVidPnInterface->pfnReleaseTargetModeSet(hVidPn, hTargetModeSet);
        });

    // Get pinned mode from target mode set.
    const D3DKMDT_VIDPN_TARGET_MODE* pPinnedTargetModeInfo = NULL;
    Status = pTargetModeSetInterface->pfnAcquirePinnedModeInfo(hTargetModeSet, &pPinnedTargetModeInfo);
    RETURN_ON_FAILURE(Status);

    auto PinnedTargetModeInfoGuard = MakeScopeExitGuard([&]()
        {
            if (NULL == pPinnedTargetModeInfo)
            {
                pTargetModeSetInterface->pfnReleaseModeInfo(hTargetModeSet, pPinnedTargetModeInfo);
            }
        });

    if (NULL != pPinnedTargetModeInfo)
    {
        *bHasPinnedMode = TRUE;
    }

    return STATUS_SUCCESS;
}

GC_PAGED_SEGMENT_END; //========================================================

