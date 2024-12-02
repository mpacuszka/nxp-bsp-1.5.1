/* Copyright (c) Microsoft Corporation.
 * Copyright 2022-2023 NXP
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmdLogging.h"
#include "GcKmdImx8Display.h"
#include "GcKmdImx8qxpDisplay.tmh"

#include "GcKmdImx8qxpDisplay.h"
#include "GcKmdGlobal.h"

#include "GcKmdUtil.h"
#include "GcKmdGuard.h"
#include "GcKmdErroHandling.h"

#include "edidparser.h"
#include "getresrc.h"

extern "C" {

#include <svc/ipc.h>

#include "linux/bits.h"
#include "linux/firmware/imx/ipc.h"
#include "linux/interrupt.h"
#include "linux/media-bus-format.h"
#include "boot/dts/freescale/board.h"
#include "lvds/imx8qxp-ldb.h"
#include "lvds/it6263.h"

}

GC_PAGED_SEGMENT_BEGIN; //======================================================

#define USE_PREVIOUS_POST_DISPLAY_INFO

#define printk(x, ...) \
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, __VA_ARGS__)

//#define QXP_DISPLAY_DEBUG
#ifdef QXP_DISPLAY_DEBUG
#define printk_debug printk
#else
#define printk_debug
#endif

NTSTATUS
GcKmImx8qxpDisplay::HwStart(DXGKRNL_INTERFACE* pDxgkInterface)
{
    NTSTATUS Status;

    if (*m_p_refCount == 0) {
        /* Initialize SCU resource only once for all display objects */
        sc_ipc_id_struct_t ipc_id = {};
        Status = GetI2CResource(pDxgkInterface, 0, &ipc_id.ipc_id);
        if (!NT_SUCCESS(Status))
        {
            return Status;
        }

        if (imx_scu_probe(&ipc_id))
        {
            return STATUS_INTERNAL_ERROR;
        }
    }

    m_clk_tree = clk_init_imx8qxp();
    if (!m_clk_tree)
    {
        return STATUS_INTERNAL_ERROR;
    }
#ifdef CLK_DUMP
    clk_dump_clock_tree_imx8qxp(m_clk_tree);
#endif

    if (*m_p_refCount == 0) {
        /* Initialize DPU, DPRC, PRG and INTSTEER objects only once for DPU (first display) */
        m_p_dpu_pdev->name = "dpu";
        m_p_dpu_pdev->plat_name = "qxp";
        m_p_dpu_pdev->data = pDxgkInterface;
        board_init(m_p_dpu_pdev);

        m_p_irqsteer_pdev->name = "irqsteer";
        m_p_irqsteer_pdev->plat_name = "qxp";
        m_p_irqsteer_pdev->data = pDxgkInterface;
        board_init(m_p_irqsteer_pdev);

        prg_res_init(m_p_prg_pdev, PRG_CNT);
        for (int i = 0; i < PRG_CNT; i++)
        {
            if (prg_probe(&m_p_prg_pdev[i]))
            {
                return STATUS_INTERNAL_ERROR;
            }
        }

        dprc_res_init(m_p_dprc_pdev, DPRC_CNT);
        for (int i = 0; i < DPRC_CNT; i++)
        {
            if (dprc_probe(&m_p_dprc_pdev[i], m_p_prg_pdev, PRG_CNT))
            {
                return STATUS_INTERNAL_ERROR;
            }
        }

        if (dpu_probe(m_p_dpu_pdev, m_p_client_devices, CLIENT_DEVICE_CNT,
            m_p_dprc_pdev, DPRC_CNT))
        {
            return STATUS_INTERNAL_ERROR;
        }
    }
    (*m_p_refCount)++;
    if (dpu_crtc_probe(&m_p_client_devices[m_di.DisplayInterfaceIndex], &m_crtc))
    {
        return STATUS_INTERNAL_ERROR;
    }

    //Dynamic allocation of states are being freed in HWStop
    dpu_plane_reset(&m_crtc.plane[m_PlaneId]->base);
    dpu_drm_crtc_reset(&m_crtc.base);
    m_crtc.base.state->enable = true;
    // select plane attached to crtc
    m_crtc.base.state->plane_mask |= drm_plane_mask(&m_crtc.plane[m_PlaneId]->base);

    return m_LvdsTransmitter.Start(pDxgkInterface, m_di.RegistryIndex);
}

NTSTATUS
GcKmImx8qxpDisplay::HwStop(
    DXGK_DISPLAY_INFORMATION   *pFwDisplayInfo,
    BOOLEAN DoCommitFwFb)
{
    // Free resources and flip framebuffer to the linear mode
    // but keep display running so when the display/GPU driver
    // is disabled/uninstalled, the desktop should continue to work
    // (with Basic Display Driver).
    if (DoCommitFwFb) {
        SetupFramebuffer(
            m_FbPhysicalAddr.LowPart,
            m_CurSourceModes[0].Format.Graphics.Stride,
            TranslateD3dDdiToDrmFormat(m_CurSourceModes[0].Format.Graphics.PixelFormat),
            Linear);

        dpu_plane_atomic_page_flip(&m_crtc.plane[m_PlaneId]->base, &m_old_plane_state);
        dpu_crtc_atomic_flush(&m_crtc.base, m_crtc.base.state);
    }

    m_LvdsTransmitter.Stop();

    dpu_drm_atomic_plane_destroy_state(&m_crtc.plane[m_PlaneId]->base,
        m_crtc.plane[m_PlaneId]->base.state);

    dpu_crtc_remove(&m_p_client_devices[m_di.DisplayInterfaceIndex]);

    if (*m_p_refCount == 1) {
        /* Remove DPU, DPRC, PRGand INTSTEER objects only once for DPU(last display) */
        dpu_remove(m_p_dpu_pdev, m_p_client_devices, CLIENT_DEVICE_CNT);

        for (int i = 0; i < PRG_CNT; i++)
        {
            prg_remove(&m_p_prg_pdev[i], false);
        }

        for (int i = 0; i < DPRC_CNT; i++)
        {
            dprc_remove(&m_p_dprc_pdev[i], false);
        }

        board_deinit(m_p_irqsteer_pdev);

        imx_scu_remove();
    }

    clk_deinit_imx8qxp(m_clk_tree);
    (*m_p_refCount)--;

    return STATUS_SUCCESS;
}

void
GcKmImx8qxpDisplay::HwSetPowerState(
    IN_ULONG                DeviceUid,
    IN_DEVICE_POWER_STATE   DevicePowerState,
    IN_POWER_ACTION         ActionType)
{
    /* A placeholder for the board specific activity during changing of the display power mode */
}

void
GcKmImx8qxpDisplay::HwStopScanning(
    D3DDDI_VIDEO_PRESENT_TARGET_ID  TargetId)
{
    struct drm_encoder* enc = m_LvdsTransmitter.GetEncoder(0);
    NT_ASSERT(enc);
    imx8qxp_ldb_encoder_disable(enc);
    (void)dpu_crtc_atomic_disable(&m_crtc.base, m_crtc.base.state);
}

GC_PAGED_SEGMENT_END; //========================================================

GC_NONPAGED_SEGMENT_BEGIN; //===================================================

void
GcKmImx8qxpDisplay::SetupFramebuffer(
    UINT FrameBufferPhysicalAddress,
    UINT Pitch,
    UINT DrmFormat,
    UINT64 TileMode)
{
    struct drm_framebuffer *fb = &m_crtc.plane[m_PlaneId]->fb;

    fb->pitches[0] = Pitch;
    fb->format = drm_format_info(DrmFormat);
    fb->paddr[0] = FrameBufferPhysicalAddress;
    fb->modifier = GetDrmTileFromHwTile((Gc7LHwTileMode)TileMode);
    fb->flags = fb->modifier ? DRM_MODE_FB_MODIFIERS : 0;
}

NTSTATUS
GcKmImx8qxpDisplay::SetVidPnSourceAddress(
    IN_CONST_PDXGKARG_SETVIDPNSOURCEADDRESS pSetVidPnSourceAddress)
{
    GcKmAllocation *pAllocation = (GcKmAllocation *)pSetVidPnSourceAddress->hAllocation;

    m_FrontBufferSegmentOffset = pSetVidPnSourceAddress->PrimaryAddress;

    SetupFramebuffer(
        m_LocalVidMemPhysicalBase + m_FrontBufferSegmentOffset.LowPart,
        pAllocation->m_hwPitch,
        TranslateDxgiToDrmFormat(pAllocation->m_format),
        pAllocation->m_hwTileMode);

    dpu_plane_atomic_page_flip(&m_crtc.plane[m_PlaneId]->base, &m_old_plane_state);

    return dpu_crtc_atomic_flush(&m_crtc.base, m_crtc.base.state);
}

GC_NONPAGED_SEGMENT_END; //=====================================================

GC_PAGED_SEGMENT_BEGIN; //======================================================

extern BOOLEAN  g_bUsePreviousPostDisplayInfo;

void GcKmImx8qxpDisplay::SetupPlaneState(
    const D3DKMDT_VIDPN_SOURCE_MODE* pSourceMode,
    const D3DKMDT_VIDPN_TARGET_MODE* pTargetMode)
{
    struct drm_plane_state* state = m_crtc.plane[m_PlaneId]->base.state;
    state->rotation = DRM_MODE_ROTATE_0; // rotation is not supported yet
    state->alpha = 0xffff;
    state->crtc = &m_crtc.base;
    // scaling is not supported; crtc w/h == src w/h
    state->crtc_w = pSourceMode->Format.Graphics.VisibleRegionSize.cx;
    state->crtc_h = pSourceMode->Format.Graphics.VisibleRegionSize.cy;
    // src_w/src_h is in 16.16 fixed point format
    state->src_w = pSourceMode->Format.Graphics.VisibleRegionSize.cx << 16;
    state->src_h = pSourceMode->Format.Graphics.VisibleRegionSize.cy << 16;
    state->src.x2 = state->src_w;
    state->src.y2 = state->src_h;
    state->dst.x2 = pTargetMode->VideoSignalInfo.ActiveSize.cx;
    state->dst.y2 = pTargetMode->VideoSignalInfo.ActiveSize.cy;
    state->fb = &m_crtc.plane[m_PlaneId]->fb;
    state->visible = true;
    printk_debug("HwCommitVidPn: plane_state cw=%d ch=%d cwh=0x%x chh=0x%x sw=0x%x sh=0x%x\n",
        state->crtc_w, state->crtc_h, state->crtc_w, state->crtc_h, state->src_w, state->src_h);
}

NTSTATUS
GcKmImx8qxpDisplay::SetupHwCommit(
    const D3DKMDT_VIDPN_SOURCE_MODE* pSourceMode,
    const D3DKMDT_VIDPN_TARGET_MODE* pTargetMode,
    struct videomode* vm,
    UINT FrameBufferPhysicalAddress,
    UINT Pitch,
    UINT TileMode)
{
    struct drm_display_mode dmode = { 0 };
    drm_display_mode_from_videomode(vm, &dmode);
    drm_mode_set_crtcinfo(&dmode, 0);

    SetupFramebuffer(
        FrameBufferPhysicalAddress,
        Pitch,
        TranslateD3dDdiToDrmFormat(pSourceMode->Format.Graphics.PixelFormat),
        TileMode);

    SetupPlaneState(pSourceMode, pTargetMode);

    struct drm_encoder* enc = m_LvdsTransmitter.GetEncoder(0);
    if (!enc)
    {
        return STATUS_NO_MEMORY;
    }

    // connector state intentionally empty here,
    // we pass bus format in crtc_state
    struct drm_connector_state conn_state;
    imx8qxp_ldb_encoder_atomic_check(enc, m_crtc.base.state,
        &conn_state);

    dpu_crtc_copy_display_mode(&dmode, &m_crtc.base.state->adjusted_mode);

    struct drm_crtc* crtc_list[1] = { &m_crtc.base };
    if (dpu_drm_atomic_check(nullptr, crtc_list, ARRAY_SIZE(crtc_list)))
    {
        return STATUS_NO_MEMORY;
    }

    if (dpu_crtc_atomic_check(&m_crtc.base, m_crtc.base.state))
    {
        return STATUS_NO_MEMORY;
    }

    if (dpu_plane_atomic_check(&m_crtc.plane[m_PlaneId]->base,
        m_crtc.plane[m_PlaneId]->base.state))
    {
        return STATUS_NO_MEMORY;
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmImx8qxpDisplay::HwAtomicCommit(struct videomode* vm)
{
    // We should always do a full modeset because:
    // 1. Display runtime already filters out modeset request for the same mode.
    // 2. The HW state can get corrupt and
    //      Shift + Ctrl + Windows + B is meant to correct that.

    struct i2c_client* bridge = m_LvdsTransmitter.GetBridge();
    NT_ASSERT(bridge);
    if (m_LvdsTransmitter.BridgeIsInitialized())
    {
        it6263_bridge_disable(bridge);
    }

    printk_debug("CRTC %d enabled: %d plane: %d\n", m_di.DisplayInterfaceIndex, dpu_crtc_is_enabled(&m_crtc.base), m_PlaneId);

    struct drm_encoder* enc = m_LvdsTransmitter.GetEncoder(0);
    NT_ASSERT(enc);
    if (dpu_crtc_is_enabled(&m_crtc.base)) {
        imx8qxp_ldb_encoder_disable(enc);

        if (dpu_crtc_atomic_disable(&m_crtc.base, m_crtc.base.state))
        {
            return STATUS_UNSUCCESSFUL;
        }
    }

    dpu_crtc_mode_set_nofb(&m_crtc.base, enc);

    // connector state intentionally empty here,
    // we pass bus format in crtc_state
    struct drm_connector_state connector_state;
    imx8qxp_ldb_encoder_atomic_mode_set(enc, m_crtc.base.state,
        &connector_state);

    if (m_LvdsTransmitter.BridgeIsInitialized())
    {
        it6263_bridge_mode_set(bridge, vm);
    }

    m_crtc_state.imx_crtc_state.base.active = true;

    // TODO: old_pane_state is never used in plane helpers it can be removed
    if (dpu_plane_atomic_update(&m_crtc.plane[m_PlaneId]->base, &m_old_plane_state))
    {
        return STATUS_UNSUCCESSFUL;
    }

    if (dpu_crtc_atomic_enable(&m_crtc.base, m_crtc.base.state))
    {
        return STATUS_UNSUCCESSFUL;
    }

    imx8qxp_ldb_encoder_enable(enc);

    if (m_LvdsTransmitter.BridgeIsInitialized())
    {
        it6263_bridge_enable(bridge);
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmImx8qxpDisplay::HwCommitVidPn(
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
        FrameBufferPhysicalAddress = m_FbPhysicalAddr.LowPart;
        TileMode = 0;

        if (!g_bUsePreviousPostDisplayInfo) {
            Pitch = pSourceMode->Format.Graphics.Stride;
        }
        else {
            Pitch = m_PreviousPostDisplayInfo.Pitch;
        }
    }

    m_OldFbPhysicalAddress = FrameBufferPhysicalAddress;

    //
    // For detailed mode timing info, a monitor mode (commonly native)
    // in EDID should be matched to based on the target/source mode
    //
    // And then code ported from dcss_crtc_atomic_enable() can be used
    // to actually set the monitor mode on the DCSS display controller
    //

    void *pEdid;
    struct videomode vm;

    UINT EdidSize = m_LvdsTransmitter.GetCachedEdid(&pEdid);

    if (!GetDisplayModeTiming(pEdid, EdidSize, pTargetMode, &vm))
    {
        // TODO: Return the status of actual mode setting
        return STATUS_NO_MEMORY;
    }

    printk_debug("HwCommitVidPn: target_mode actw=%d acth=%d pclk=%d\n",
        pTargetMode->VideoSignalInfo.ActiveSize.cx, pTargetMode->VideoSignalInfo.ActiveSize.cy, pTargetMode->VideoSignalInfo.PixelRate);

    NTSTATUS Status = SetupHwCommit(pSourceMode, pTargetMode,
        &vm, FrameBufferPhysicalAddress, Pitch, TileMode);
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    Status = HwAtomicCommit(&vm);
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    m_CurSourceModes[0] = *pSourceMode;
    m_CurTargetModes[0] = *pTargetMode;

    m_bNotifyVSync = true;

    m_ScanoutFormat = TranslateD3dDdiToDxgiFormat(pSourceMode->Format.Graphics.PixelFormat);

    return Status;
}

NTSTATUS
GcKmImx8qxpDisplay::ControlInterrupt(
    IN_CONST_DXGK_INTERRUPT_TYPE    InterruptType,
    IN_BOOLEAN  EnableInterrupt)
{
    struct dpu_crtc* dpu_crtc =
        (struct dpu_crtc*)platform_get_drvdata(&m_p_client_devices[m_di.DisplayInterfaceIndex]);

    switch (InterruptType)
    {
    case DXGK_INTERRUPT_CRTC_VSYNC:
    case DXGK_INTERRUPT_DISPLAYONLY_VSYNC:
        if (EnableInterrupt)
        {
            dpu_enable_vblank(dpu_crtc);
        }
        else
        {
            dpu_disable_vblank(dpu_crtc);
        }
        break;
    }

    return STATUS_SUCCESS;
}

GC_PAGED_SEGMENT_END; //========================================================

GC_NONPAGED_SEGMENT_BEGIN; //===================================================

// TODO: move this to the helper ?
NTSTATUS
GcKmImx8qxpDisplay::GetI2CResource(
    DXGKRNL_INTERFACE* pDxgkInterface,
    ULONG I2CId,
    LARGE_INTEGER* pI2CConnectioId)
{
    NTSTATUS Status;
    WCHAR Buff[512];

    Status = GetReslist(pDxgkInterface, (PWCHAR)&Buff, sizeof(Buff));
    if (!NT_SUCCESS(Status)) {
        return Status;
    }

    Status = ParseReslist((PCM_RESOURCE_LIST)&Buff, CmResourceTypeConnection,
        pI2CConnectioId, NULL, I2CId, ResourceType_I2C);
    if (!NT_SUCCESS(Status)) {
        return Status;
    }

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmImx8qxpDisplay::SetVidPnSourceAddressWithMultiPlaneOverlay3(
    IN_OUT_PDXGKARG_SETVIDPNSOURCEADDRESSWITHMULTIPLANEOVERLAY3 pSetMpo3)
{
    NT_ASSERT(pSetMpo3->PlaneCount == 1);

    if (!pSetMpo3->ppPlanes[0]->InputFlags.Enabled)
    {
        m_FrontBufferSegmentOffset.QuadPart = -1L;
        return STATUS_SUCCESS;
    }

    NT_ASSERT(pSetMpo3->ppPlanes[0]->ContextCount == 1);

    GcKmAllocation *pAllocation = (GcKmAllocation *)pSetMpo3->ppPlanes[0]->ppContextData[0]->hAllocation;

    m_FrontBufferSegmentOffset = pSetMpo3->ppPlanes[0]->ppContextData[0]->SegmentAddress;

    SetupFramebuffer(
        m_LocalVidMemPhysicalBase + m_FrontBufferSegmentOffset.LowPart,
        pAllocation->m_hwPitch,
        TranslateDxgiToDrmFormat(pAllocation->m_format),
        pAllocation->m_hwTileMode);

    dpu_plane_atomic_page_flip(&m_crtc.plane[m_PlaneId]->base, &m_old_plane_state);

    return dpu_crtc_atomic_flush(&m_crtc.base, m_crtc.base.state);
}

void
GcKmImx8qxpDisplay::HandleVsyncInterrupt(UINT Disp)
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

    m_InterruptData.CrtcVsync.VidPnTargetId = Disp ?
        BaseTransmitter::LVDS1_CHILD_UID :
        BaseTransmitter::LVDS0_CHILD_UID;

    // If we missed Shadow load update requested buffer address
    // has not been loaded into DC.
    // We should report back previous address instead of requested one.
    if (!dpu_crtc_get_shdld_status(&m_crtc.base))
    {
        m_InterruptData.CrtcVsync.PhysicalAddress.LowPart =
            m_OldFbPhysicalAddress;
    }
    else
    {
        m_InterruptData.CrtcVsync.PhysicalAddress = m_FrontBufferSegmentOffset;
        m_OldFbPhysicalAddress = m_FrontBufferSegmentOffset.LowPart;
    }

    m_InterruptData.CrtcVsync.PhysicalAdapterMask = 1;
    m_InterruptData.Flags.ValidPhysicalAdapterMask = TRUE;

    if (m_bNotifyVSync)
    {
        m_pDxgkInterface->DxgkCbNotifyInterrupt(m_pDxgkInterface->DeviceHandle,
            &m_InterruptData);

        m_pDxgkInterface->DxgkCbQueueDpc(m_pDxgkInterface->DeviceHandle);
    }
}

BOOLEAN
GcKmImx8qxpDisplay::InterruptRoutine(UINT MessageNumber)
{
    BOOLEAN ret = FALSE;

    if (GcKmdGlobal::s_DriverMode == RenderOnly)
    {
        return FALSE;
    }

    struct dpu_soc* dpu = (struct dpu_soc*)platform_get_drvdata(m_p_dpu_pdev);
    const struct dpu_data* data = dpu->data;
    const struct cm_reg_ofs* ofs = data->cm_reg_ofs;

    UINT32 status0 = dpu_cm_read(dpu, USERINTERRUPTSTATUS(ofs, 0));

    status0 &= dpu_cm_read(dpu, USERINTERRUPTENABLE(ofs, 0));

    if (m_di.DisplayInterfaceIndex == 0) {
        if (status0 & BIT(IRQ_DISENGCFG_FRAMECOMPLETE0))
        {
            HandleVsyncInterrupt(0);
            dpu_irq_ack(dpu, IRQ_DISENGCFG_FRAMECOMPLETE0);
            ret = TRUE;
        }
        if (status0 & BIT(IRQ_EXTDST0_SHDLOAD))
        {
            irq_handle(IRQ_EXTDST0_SHDLOAD);
            dpu_irq_ack(dpu, IRQ_EXTDST0_SHDLOAD);
            ret = TRUE;
        }
        if (status0 & BIT(IRQ_EXTDST4_SHDLOAD))
        {
            irq_handle(IRQ_EXTDST4_SHDLOAD);
            dpu_irq_ack(dpu, IRQ_EXTDST4_SHDLOAD);
            ret = TRUE;
        }
        if (status0 & BIT(IRQ_DISENGCFG_SHDLOAD0))
        {
            irq_handle(IRQ_DISENGCFG_SHDLOAD0);
            dpu_irq_ack(dpu, IRQ_DISENGCFG_SHDLOAD0);
            ret = TRUE;
        }
    }
    if (m_di.DisplayInterfaceIndex == 1) {
        if (status0 & BIT(IRQ_DISENGCFG_FRAMECOMPLETE1))
        {
            HandleVsyncInterrupt(1);
            dpu_irq_ack(dpu, IRQ_DISENGCFG_FRAMECOMPLETE1);
            ret = TRUE;
        }
        if (status0 & BIT(IRQ_EXTDST1_SHDLOAD))
        {
            irq_handle(IRQ_EXTDST1_SHDLOAD);
            dpu_irq_ack(dpu, IRQ_EXTDST1_SHDLOAD);
            ret = TRUE;
        }
        if (status0 & BIT(IRQ_EXTDST5_SHDLOAD))
        {
            irq_handle(IRQ_EXTDST5_SHDLOAD);
            dpu_irq_ack(dpu, IRQ_EXTDST5_SHDLOAD);
            ret = TRUE;
        }
        if (status0 & BIT(IRQ_DISENGCFG_SHDLOAD1))
        {
            irq_handle(IRQ_DISENGCFG_SHDLOAD1);
            dpu_irq_ack(dpu, IRQ_DISENGCFG_SHDLOAD1);
            ret = TRUE;
        }
    }
    return ret;
}

GC_NONPAGED_SEGMENT_END; //=====================================================
