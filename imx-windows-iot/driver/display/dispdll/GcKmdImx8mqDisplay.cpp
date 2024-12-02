/* Copyright (c) Microsoft Corporation.
 * Copyright 2022-2023 NXP
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmdLogging.h"
#include "GcKmdImx8mqDisplay.tmh"

#include "GcKmdImx8mqDisplay.h"
#include "GcKmdGlobal.h"

#include "GcKmdUtil.h"
#include "GcKmdGuard.h"
#include "GcKmdErroHandling.h"

#include "edidparser.h"

extern "C" {

#include "linux/interrupt.h"
#include "boot/dts/freescale/board.h"

}

GC_PAGED_SEGMENT_BEGIN; //======================================================

#define USE_PREVIOUS_POST_DISPLAY_INFO

NTSTATUS
GcKmImx8mqDisplay::HwStart(DXGKRNL_INTERFACE* pDxgkInterface)
{
    clk_tree = clk_init_imx8mq();
    if (!clk_tree)
    {
        return STATUS_INTERNAL_ERROR;
    }
#ifdef CLK_DUMP
    clk_dump_clock_tree_imx8mq(clk_tree);
#endif

    m_dcss_pdev.name = "dcss";
    m_dcss_pdev.plat_name = "mq";
    m_dcss_pdev.data = pDxgkInterface;
    board_init(&m_dcss_pdev);

    m_irqsteer_pdev.name = "irqsteer";
    m_irqsteer_pdev.plat_name = "mq";
    m_irqsteer_pdev.data = pDxgkInterface;
    board_init(&m_irqsteer_pdev);

    m_dcss_dev = dcss_dev_create(&m_dcss_pdev.dev, true);
    if (IS_ERR(m_dcss_dev))
    {
        return STATUS_INTERNAL_ERROR;
    }
    dev_set_drvdata(&m_dcss_pdev.dev, m_dcss_dev);

    dcss_crtc_init(&m_crtc, m_dcss_dev);

    if (dcss_crtc_in_use(m_dcss_dev))
    {
        m_crtc_state.base.active = true;
    }

    m_mhdp_pdev.name = "hdmi";
    m_mhdp_pdev.plat_name = "mq";
    m_mhdp_pdev.data = pDxgkInterface;
    board_init(&m_mhdp_pdev);
    if (cdns_mhdp_imx_bind(&m_mhdp_pdev.dev, nullptr, nullptr))
    {
        return STATUS_INTERNAL_ERROR;
    }

    struct cdns_mhdp_device *mhdp = (cdns_mhdp_device*)dev_get_drvdata(&m_mhdp_pdev.dev);
    mhdp->connector.base.state = &m_mhdp_dev.connector.state;
    mhdp->bridge.base.encoder->crtc = &m_crtc.base;

    return m_HdmiTransmitter.Start();
}

NTSTATUS
GcKmImx8mqDisplay::HwStop(
    DXGK_DISPLAY_INFORMATION   *pFwDisplayInfo,
    BOOLEAN DoCommitFwFb)
{
    // Free resources and flip framebuffer to the linear mode
    // but keep display running so when the display/GPU driver
    // is disabled/uninstalled, the desktop should continue to work
    // (with Basic Display Driver).
    if (DoCommitFwFb) {
        SetupFramebuffer(
            m_PreviousPostDisplayInfo.PhysicAddress.LowPart,
            m_PreviousPostDisplayInfo.Pitch,
            TranslateD3dDdiToDrmFormat(m_PreviousPostDisplayInfo.ColorFormat),
            GetDrmTileFromHwTile(Linear));
        dcss_plane_atomic_update(&m_crtc.plane[0]->base,
            &m_old_plane_state);
        dcss_crtc_atomic_flush(&m_crtc.base, &m_crtc_state.base);
    }

    dcss_dev_destroy(m_dcss_dev, false);
    board_deinit(&m_dcss_pdev);
    board_deinit(&m_irqsteer_pdev);

    cdns_mhdp_imx_unbind(&m_mhdp_pdev.dev, nullptr, nullptr);

    clk_deinit_imx8mq(clk_tree);

    return STATUS_SUCCESS;
}

void
GcKmImx8mqDisplay::HwSetPowerState(
    IN_ULONG                DeviceUid,
    IN_DEVICE_POWER_STATE   DevicePowerState,
    IN_POWER_ACTION         ActionType)
{
    /* A placeholder for the board specific activity during changing of the display power mode */
}

void
GcKmImx8mqDisplay::HwStopScanning(
    D3DDDI_VIDEO_PRESENT_TARGET_ID  TargetId)
{
    struct cdns_mhdp_device* mhdp =
        (cdns_mhdp_device*)dev_get_drvdata(&m_mhdp_pdev.dev);
    struct imx_mhdp_device* imx_mhdp =
        container_of(mhdp, struct imx_mhdp_device, mhdp);

    cdns_mhdp_imx_encoder_disable(&imx_mhdp->encoder);
    dcss_crtc_atomic_disable(&m_crtc.base, NULL, &m_old_dmode);
    m_crtc_state.base.active = false;
}

GC_PAGED_SEGMENT_END; //========================================================

GC_NONPAGED_SEGMENT_BEGIN; //===================================================

void
GcKmImx8mqDisplay::SetupFramebuffer(
    UINT FrameBufferPhysicalAddress,
    UINT Pitch,
    UINT DrmFormat,
    UINT64 TileMode)
{
    struct drm_framebuffer *fb = &m_crtc.plane[0]->fb;

    fb->pitches[0] = Pitch;
    fb->format = drm_format_info(DrmFormat);
    fb->paddr[0] = FrameBufferPhysicalAddress;
    fb->modifier = GetDrmTileFromHwTile((Gc7LHwTileMode)TileMode);
    fb->flags = fb->modifier ? DRM_MODE_FB_MODIFIERS : 0;
}

NTSTATUS
GcKmImx8mqDisplay::SetVidPnSourceAddress(
    IN_CONST_PDXGKARG_SETVIDPNSOURCEADDRESS pSetVidPnSourceAddress)
{
    GcKmAllocation *pAllocation = (GcKmAllocation *)pSetVidPnSourceAddress->hAllocation;

    m_FrontBufferSegmentOffset = pSetVidPnSourceAddress->PrimaryAddress;

    SetupFramebuffer(
        m_LocalVidMemPhysicalBase + m_FrontBufferSegmentOffset.LowPart,
        pAllocation->m_hwPitch,
        TranslateDxgiToDrmFormat(pAllocation->m_format),
        GetDrmTileFromHwTile((Gc7LHwTileMode)pAllocation->m_hwTileMode));

    dcss_plane_atomic_page_flip(&m_crtc.plane[0]->base,
        &m_old_plane_state);

    dcss_crtc_atomic_flush(&m_crtc.base, &m_crtc_state.base);

    return STATUS_SUCCESS;
}

GC_NONPAGED_SEGMENT_END; //=====================================================

GC_PAGED_SEGMENT_BEGIN; //======================================================

extern BOOLEAN  g_bUsePreviousPostDisplayInfo;

void GcKmImx8mqDisplay::SetupPlaneState(
    const D3DKMDT_VIDPN_SOURCE_MODE* pSourceMode,
    const D3DKMDT_VIDPN_TARGET_MODE* pTargetMode)
{
    struct drm_plane_state *state = &m_crtc.plane[0]->state;
    state->rotation = DRM_MODE_ROTATE_0; // rotation is not supported yet
    state->alpha = 0xffff;
    state->crtc = &m_crtc.base;
    // src_w/src_h is in 16.16 fixed point format
    state->src_w = pSourceMode->Format.Graphics.VisibleRegionSize.cx << 16;
    state->src_h = pSourceMode->Format.Graphics.VisibleRegionSize.cy << 16;
    state->src.x2 = state->src_w;
    state->src.y2 = state->src_h;
    state->dst.x2 = pTargetMode->VideoSignalInfo.ActiveSize.cx;
    state->dst.y2 = pTargetMode->VideoSignalInfo.ActiveSize.cy;
    state->fb = &m_crtc.plane[0]->fb;
    state->visible = true;
}

NTSTATUS
GcKmImx8mqDisplay::SetupHwCommit(
    const D3DKMDT_VIDPN_SOURCE_MODE* pSourceMode,
    const D3DKMDT_VIDPN_TARGET_MODE* pTargetMode,
    struct videomode* vm,
    UINT FrameBufferPhysicalAddress,
    UINT Pitch,
    UINT TileMode)
{
    struct drm_display_mode dmode;
    drm_display_mode_from_videomode(vm, &dmode);

    if (dcss_crtc_mode_valid(&m_crtc.base, &dmode) != MODE_OK)
    {
        return STATUS_NO_MEMORY;
    }

    struct cdns_mhdp_device *mhdp =
        (cdns_mhdp_device*)dev_get_drvdata(&m_mhdp_pdev.dev);
    struct imx_mhdp_device *imx_mhdp =
        container_of(mhdp, struct imx_mhdp_device, mhdp);
    struct drm_display_mode adj_mode;

    if (!cdns_hdmi_bridge_mode_fixup(imx_mhdp->encoder.bridge,
        &dmode, &adj_mode))
    {
        return STATUS_NO_MEMORY;
    }

    if (dcss_plane_atomic_check(&m_crtc.plane[0]->base,
        &m_crtc.plane[0]->state))
    {
        return STATUS_NO_MEMORY;
    }

    dcss_kms_atomic_check(&m_crtc.base, &dmode, &m_old_dmode,
        &m_mhdp_dev.connector.state);

    dcss_crtc_copy_display_mode(&dmode, &m_crtc.base.state->adjusted_mode);
    dcss_crtc_copy_display_mode(&dmode, &m_crtc_state.base.mode);

    SetupFramebuffer(
        FrameBufferPhysicalAddress,
        Pitch,
        TranslateD3dDdiToDrmFormat(pSourceMode->Format.Graphics.PixelFormat),
        TileMode);

    SetupPlaneState(pSourceMode, pTargetMode);

    m_crtc.plane[0]->base.state = &m_crtc.plane[0]->state;
    m_crtc.plane[0]->state.plane = &m_crtc.plane[0]->base;

    return STATUS_SUCCESS;
}

void
GcKmImx8mqDisplay::HwAtomicCommit(struct videomode* vm)
{
    // We should always do a full modeset because:
    // 1. Display runtime already filters out modeset request for the same mode.
    // 2. The HW state can get corrupt and
    //      Shift + Ctrl + Windows + B is meant to correct that.

    struct drm_display_mode dmode;
    drm_display_mode_from_videomode(vm, &dmode);

    struct cdns_mhdp_device *mhdp =
        (cdns_mhdp_device*)dev_get_drvdata(&m_mhdp_pdev.dev);
    struct imx_mhdp_device *imx_mhdp =
        container_of(mhdp, struct imx_mhdp_device, mhdp);

    if (m_crtc_state.base.active)
    {
        cdns_mhdp_imx_encoder_disable(&imx_mhdp->encoder);
        dcss_crtc_atomic_disable(&m_crtc.base, &dmode, &m_old_dmode);
    }

    cdns_hdmi_bridge_mode_set(imx_mhdp->encoder.bridge,
        &m_old_dmode, &dmode);

    dcss_crtc_atomic_enable(&m_crtc.base, &dmode, &m_old_dmode);
    dcss_crtc_copy_display_mode(&dmode, &m_old_dmode);

    m_crtc_state.base.active = true;

    cdns_mhdp_imx_encoder_enable(&imx_mhdp->encoder);

    dcss_plane_atomic_update(&m_crtc.plane[0]->base, &m_old_plane_state);
    dcss_plane_copy_state(m_crtc.plane[0]->base.state, &m_old_plane_state);
}

NTSTATUS
GcKmImx8mqDisplay::HwCommitVidPn(
    const D3DKMDT_VIDPN_SOURCE_MODE* pSourceMode,
    const D3DKMDT_VIDPN_TARGET_MODE* pTargetMode,
    IN_CONST_PDXGKARG_COMMITVIDPN_CONST pCommitVidPn)
{
    UINT FrameBufferPhysicalAddress = 0;
    UINT TileMode = 0;
    UINT Pitch = 0;

    m_CurSourceModes[0] = { 0 };
    m_CurTargetModes[0] = { 0 };

    if (GcKmdGlobal::s_DriverMode == FullDriver)
    {
        GcKmAllocation* pPrimaryAllocation = (GcKmAllocation*)pCommitVidPn->hPrimaryAllocation;

        FrameBufferPhysicalAddress = (UINT)(pPrimaryAllocation->m_gpuPhysicalAddress.SegmentOffset + m_LocalVidMemPhysicalBase);
        Pitch = pPrimaryAllocation->m_hwPitch;
        TileMode = pPrimaryAllocation->m_hwTileMode;
    }
    else
    {
        FrameBufferPhysicalAddress = m_PreviousPostDisplayInfo.PhysicAddress.LowPart;
        Pitch = m_PreviousPostDisplayInfo.Pitch;
        TileMode = 0;
    }

    //
    // For detailed mode timing info, a monitor mode (commonly native)
    // in EDID should be matched to based on the target/source mode
    //
    // And then code ported from dcss_crtc_atomic_enable() can be used
    // to actually set the monitor mode on the DCSS display controller
    //

    void *pEdid;
    struct videomode vm;

    UINT EdidSize = m_HdmiTransmitter.GetCachedEdid(&pEdid);

    if (!GetDisplayModeTiming(pEdid, EdidSize, pTargetMode, &vm))
    {
        // TODO: Return the status of actual mode setting
        return STATUS_NO_MEMORY;
    }

    NTSTATUS Status = SetupHwCommit(pSourceMode, pTargetMode,
        &vm, FrameBufferPhysicalAddress, Pitch, TileMode);
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    HwAtomicCommit(&vm);

    m_CurSourceModes[0] = *pSourceMode;
    m_CurTargetModes[0] = *pTargetMode;

    m_bNotifyVSync = true;

    m_ScanoutFormat = TranslateD3dDdiToDxgiFormat(pSourceMode->Format.Graphics.PixelFormat);

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmImx8mqDisplay::ControlInterrupt(
    IN_CONST_DXGK_INTERRUPT_TYPE    InterruptType,
    IN_BOOLEAN  EnableInterrupt)
{
    switch (InterruptType)
    {
    case DXGK_INTERRUPT_CRTC_VSYNC:
    case DXGK_INTERRUPT_DISPLAYONLY_VSYNC:
        if (EnableInterrupt)
        {
            dcss_enable_vblank(&m_crtc);
        }
        else
        {
            dcss_disable_vblank(&m_crtc);
        }
        break;
    }

    return STATUS_SUCCESS;
}

GC_PAGED_SEGMENT_END; //========================================================

GC_NONPAGED_SEGMENT_BEGIN; //===================================================

NTSTATUS
GcKmImx8mqDisplay::SetVidPnSourceAddressWithMultiPlaneOverlay3(
    IN_OUT_PDXGKARG_SETVIDPNSOURCEADDRESSWITHMULTIPLANEOVERLAY3 pSetMpo3)
{
    NT_ASSERT(pSetMpo3->PlaneCount == 1);

    if (pSetMpo3->ppPlanes[0]->InputFlags.Enabled)
    {
        NT_ASSERT(pSetMpo3->ppPlanes[0]->ContextCount == 1);

        GcKmAllocation *pAllocation = (GcKmAllocation *)pSetMpo3->ppPlanes[0]->ppContextData[0]->hAllocation;

        m_FrontBufferSegmentOffset = pSetMpo3->ppPlanes[0]->ppContextData[0]->SegmentAddress;

        SetupFramebuffer(
            m_LocalVidMemPhysicalBase + m_FrontBufferSegmentOffset.LowPart,
            pAllocation->m_hwPitch,
            TranslateDxgiToDrmFormat(pAllocation->m_format),
            pAllocation->m_hwTileMode);

        if (pAllocation->m_format != m_ScanoutFormat)
        {
            dcss_plane_atomic_update(&m_crtc.plane[0]->base,
                &m_old_plane_state);

            m_ScanoutFormat = pAllocation->m_format;
        }
        else
        {
            dcss_plane_atomic_page_flip(&m_crtc.plane[0]->base,
                &m_old_plane_state);
        }

        dcss_crtc_atomic_flush(&m_crtc.base, &m_crtc_state.base);
    }
    else
    {
        m_FrontBufferSegmentOffset.QuadPart = -1L;
    }

    return STATUS_SUCCESS;
}

void
GcKmImx8mqDisplay::HandleVsyncInterrupt(void)
{
    RtlZeroMemory(&m_InterruptData, sizeof(m_InterruptData));

    if (GcKmdGlobal::s_DriverMode == FullDriver)
    {
        m_InterruptData.InterruptType = DXGK_INTERRUPT_CRTC_VSYNC;
    }
    else
    {
        m_InterruptData.InterruptType = DXGK_INTERRUPT_DISPLAYONLY_VSYNC;
    }
    m_InterruptData.CrtcVsync.VidPnTargetId = BaseTransmitter::HDMI_CHILD_UID;

    // If we missed CTXLD kick (CTXLD is still armed at vsync),
    // this means that requested buffer address has not been loaded into DC.
    // We should report back actual hw address instead of requested one.
    if (dcss_ctxld_is_armed(m_dcss_dev->ctxld))
    {
        m_InterruptData.CrtcVsync.PhysicalAddress.LowPart =
            dcss_get_dpr_base_addr(m_dcss_dev->dpr, 0);
    }
    else
    {
        m_InterruptData.CrtcVsync.PhysicalAddress = m_FrontBufferSegmentOffset;
    }
    m_InterruptData.CrtcVsync.PhysicalAdapterMask = 1;
    m_InterruptData.Flags.ValidPhysicalAdapterMask = TRUE;

    if (m_bNotifyVSync)
    {
        m_pDxgkInterface->DxgkCbNotifyInterrupt(m_pDxgkInterface->DeviceHandle, &m_InterruptData);
    }

    dcss_dtg_vblank_irq_clear(m_dcss_dev->dtg);

    if (m_bNotifyVSync)
    {
        m_pDxgkInterface->DxgkCbQueueDpc(m_pDxgkInterface->DeviceHandle);
    }
}

BOOLEAN
GcKmImx8mqDisplay::InterruptRoutine(UINT MessageNumber)
{
    BOOLEAN ret = FALSE;

    if (GcKmdGlobal::s_DriverMode == RenderOnly)
    {
        return FALSE;
    }

    // driver internal hw irqs
    if (irq_handle(DCSS_CTXLD_DONE))
    {
        ret = TRUE;
    }
    if (irq_handle(DCSS_CTXLD_KICK))
    {
        ret = TRUE;
    }

    // Vsync
    if (dcss_dtg_vblank_irq_valid(m_dcss_dev->dtg))
    {
        HandleVsyncInterrupt();
        ret = TRUE;
    }

    return ret;
}

GC_NONPAGED_SEGMENT_END; //=====================================================
