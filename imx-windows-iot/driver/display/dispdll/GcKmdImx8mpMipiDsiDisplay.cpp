/* Copyright (c) Microsoft Corporation.
 * Copyright 2023-2024 NXP
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmdLogging.h"
#include "GcKmdImx8Display.h"
#include "GcKmdImx8mpMipiDsiDisplay.tmh"

#include "GcKmdImx8mpMipiDsiDisplay.h"
#include "GcKmdGlobal.h"

#include "GcKmdUtil.h"
#include "GcKmdGuard.h"
#include "GcKmdErroHandling.h"

#include "edidparser.h"

extern "C" {
#include "linux/interrupt.h"
#include "boot/dts/freescale/board.h"
#include "lcdifv3/imx-lcdifv3.h"
#include "lcdifv3/lcdifv3-plane.h"
#include "mipi_dsi/adv7511.h"

#include <drm/drm_fourcc.h>
}

GC_PAGED_SEGMENT_BEGIN; //======================================================

#define USE_PREVIOUS_POST_DISPLAY_INFO

#define printk(x, ...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, __VA_ARGS__)

/* #define MP_DISPLAY_DEBUG */
#ifdef MP_DISPLAY_DEBUG
    #define printk_debug printk
#else
    #define printk_debug
#endif

NTSTATUS
GcKmImx8mpMipiDsiDisplay::HwStart(DXGKRNL_INTERFACE* pDxgkInterface)
{
    NTSTATUS ret = STATUS_SUCCESS;

    clk_tree = clk_init_imx8mp(imx_mipi_dsi, m_di.Shared);

    lcdif_pdev.name = "lcdif1_dev";
    lcdif_pdev.plat_name = "mp";
    lcdif_pdev.data = pDxgkInterface;
    board_init(&lcdif_pdev);
    /* Create lcdifv3_soc (low level device) */
    if (imx_lcdifv3_probe(&lcdif_pdev) != 0) {
        imx_lcdifv3_remove(&lcdif_pdev);
        printk("MP Mipi display: ERROR imx_lcdifv3_probe failed.\n");
        return STATUS_INTERNAL_ERROR;
    }

    lcdif_crtc_pdev.name = "lcdif1_crtc";
    lcdif_crtc_pdev.plat_name = "mp";
    lcdif_crtc_pdev.data = pDxgkInterface;
    /* Create lcdifv3_crtc (crtc device) */
    if (lcdifv3_crtc_probe(&lcdif_crtc_pdev) != 0) {
        lcdifv3_crtc_remove(&lcdif_crtc_pdev);
        imx_lcdifv3_remove(&lcdif_pdev);
        printk("MP Mipi display: ERROR lcdifv3_crtc_probe failed.\n");
        return STATUS_INTERNAL_ERROR;
    }

    /* Set lcdifv3_soc (low level device) as a parent of lcdifv3_crtc (crtc device) */
    lcdif_crtc_pdev.dev.parent = &lcdif_pdev.dev;
    /* Bind planes to lcdif_crtc */
    if (lcdifv3_crtc_bind(&lcdif_crtc_pdev.dev) != 0) {
        lcdifv3_crtc_remove(&lcdif_crtc_pdev);
        imx_lcdifv3_remove(&lcdif_pdev);
        printk("MP Mipi display: ERROR lcdif_crtc_bind failed.\n");
        return STATUS_INTERNAL_ERROR;
    }

    /* Make sure interrupt is disabled */
    lcdifv3_disable_vblank(&lcdif_crtc_pdev);
    /* Start clocks in lcdifv3_soc directly */
    imx_lcdifv3_runtime_resume(&lcdif_pdev.dev);
    ret = m_DsiTransmitter.Start(pDxgkInterface, "mp", m_di.RegistryIndex);

    return ret;
}

NTSTATUS
GcKmImx8mpMipiDsiDisplay::HwStop(
    DXGK_DISPLAY_INFORMATION   *pFwDisplayInfo,
    BOOLEAN DoCommitFwFb)
{
    struct lcdifv3_plane_state plane_state;
    NTSTATUS ret = STATUS_SUCCESS;

    if (DoCommitFwFb) {
        /* page-flip back to firmware framebuffer */
        if (m_CurSourceModes[0].Format.Graphics.Stride != m_Pitch) {
            plane_state.format = TranslateD3dDdiToDrmFormat(m_CurSourceModes[0].Format.Graphics.PixelFormat);
            plane_state.pitch = m_CurSourceModes[0].Format.Graphics.Stride;
            plane_state.src_w = m_CurSourceModes[0].Format.Graphics.VisibleRegionSize.cx;
            plane_state.src_h = m_CurSourceModes[0].Format.Graphics.VisibleRegionSize.cy;
            plane_state.mode_change = true;
        }
        else {
            plane_state.mode_change = false;
        }
        plane_state.fb_addr = m_FbPhysicalAddr.LowPart;
        lcdifv3_plane_atomic_update(&lcdif_crtc_pdev, CRTC_PLANE_INDEX_PRIMARY, &plane_state);
        lcdifv3_crtc_atomic_flush(&lcdif_crtc_pdev);
    }

    /* To full stop the hardware, disable display controller: lcdifv3_crtc_atomic_disable(&lcdif_crtc_pdev); */

    ret = m_DsiTransmitter.Stop();

    /* To full stop the hardware, Stop clocks in lcdifv3_soc directly: imx_lcdifv3_runtime_suspend(&lcdif_pdev.dev); */

    lcdifv3_crtc_remove(&lcdif_crtc_pdev);
    imx_lcdifv3_remove(&lcdif_pdev);
    board_deinit(&lcdif_pdev);

    if (!DoCommitFwFb) {
        clk_stop_imx8mp(clk_tree, imx_mipi_dsi);
    }
    clk_deinit_imx8mp(clk_tree, imx_mipi_dsi);

    return ret;
}



void
GcKmImx8mpMipiDsiDisplay::HwSetPowerState(
    IN_ULONG                DeviceUid,
    IN_DEVICE_POWER_STATE   DevicePowerState,
    IN_POWER_ACTION         ActionType)
{
    /* A placeholder for the board specific activity during changing of the display power mode */
}

void
GcKmImx8mpMipiDsiDisplay::HwStopScanning(
    D3DDDI_VIDEO_PRESENT_TARGET_ID  TargetId)
{
    sec_mipi_dsim_bridge_disable(&m_DsiTransmitter.dsi_pdev);
    imx_sec_dsim_encoder_helper_disable(&m_DsiTransmitter.dsi_pdev);
    lcdifv3_crtc_atomic_disable(&lcdif_crtc_pdev);
}

GC_PAGED_SEGMENT_END; //========================================================

GC_NONPAGED_SEGMENT_BEGIN; //===================================================

NTSTATUS
GcKmImx8mpMipiDsiDisplay::SetVidPnSourceAddress(
    IN_CONST_PDXGKARG_SETVIDPNSOURCEADDRESS pSetVidPnSourceAddress)
{
    GcKmAllocation *pAllocation = (GcKmAllocation *)pSetVidPnSourceAddress->hAllocation;
    struct lcdifv3_plane_state plane_state;

    m_FrontBufferSegmentOffset = pSetVidPnSourceAddress->PrimaryAddress;

    plane_state.fb_addr = m_LocalVidMemPhysicalBase + m_FrontBufferSegmentOffset.LowPart + pAllocation->m_linearOffset;

    plane_state.format = TranslateDxgiToDrmFormat(pAllocation->m_format);
    plane_state.pitch = pAllocation->m_hwPitch;
    plane_state.src_w = pAllocation->m_mip0Info.PhysicalWidth;
    plane_state.src_h = pAllocation->m_mip0Info.PhysicalHeight;

    /* Assume only address has changed */
    plane_state.mode_change = false;

    lcdifv3_plane_atomic_update(&lcdif_crtc_pdev, CRTC_PLANE_INDEX_PRIMARY, &plane_state);
    lcdifv3_crtc_atomic_flush(&lcdif_crtc_pdev);

    return STATUS_SUCCESS;
}

GC_NONPAGED_SEGMENT_END; //=====================================================

GC_PAGED_SEGMENT_BEGIN; //======================================================

extern BOOLEAN  g_bUsePreviousPostDisplayInfo;

NTSTATUS
GcKmImx8mpMipiDsiDisplay::HwCommitVidPn(
    const D3DKMDT_VIDPN_SOURCE_MODE* pSourceMode,
    const D3DKMDT_VIDPN_TARGET_MODE* pTargetMode,
    IN_CONST_PDXGKARG_COMMITVIDPN_CONST pCommitVidPn)
{
    struct lcdifv3_plane_state plane_state;

    UINT FrameBufferPhysicalAddress = 0;
    UINT TileMode = 0;
    DXGI_FORMAT ColorFormat;

    m_CurSourceModes[0] = { 0 };
    m_CurTargetModes[0] = { 0 };

    if (GcKmdGlobal::s_DriverMode == FullDriver)
    {
        GcKmAllocation* pPrimaryAllocation = (GcKmAllocation*)pCommitVidPn->hPrimaryAllocation;

        FrameBufferPhysicalAddress = (UINT)(pPrimaryAllocation->m_gpuPhysicalAddress.SegmentOffset + m_LocalVidMemPhysicalBase + pPrimaryAllocation->m_linearOffset);
        TileMode = pPrimaryAllocation->m_hwTileMode;

        ColorFormat = pPrimaryAllocation->m_format;
        plane_state.pitch = pPrimaryAllocation->m_hwPitch;
        /* After Wakeup from sleep, pPrimaryAllocation->m_hwWidthPixelsand and pPrimaryAllocation->m_hwHeightPixels are zero. */
        plane_state.src_w = pPrimaryAllocation->m_mip0Info.PhysicalWidth;
        plane_state.src_h = pPrimaryAllocation->m_mip0Info.PhysicalHeight;
    }
    else {
        FrameBufferPhysicalAddress = m_FbPhysicalAddr.LowPart;
        TileMode = 0;

        if (!g_bUsePreviousPostDisplayInfo) {
            ColorFormat = TranslateD3dDdiToDxgiFormat(pSourceMode->Format.Graphics.PixelFormat);
            plane_state.pitch = pSourceMode->Format.Graphics.Stride;
            plane_state.src_w = pSourceMode->Format.Graphics.VisibleRegionSize.cx;
            plane_state.src_h = pSourceMode->Format.Graphics.VisibleRegionSize.cy;
        }
        else {
            ColorFormat = TranslateD3dDdiToDxgiFormat(m_PreviousPostDisplayInfo.ColorFormat);
            plane_state.pitch = m_PreviousPostDisplayInfo.Pitch;
            plane_state.src_w = m_PreviousPostDisplayInfo.Width;
            plane_state.src_h = m_PreviousPostDisplayInfo.Height;
        }
    }
    plane_state.format = TranslateDxgiToDrmFormat(ColorFormat);
    plane_state.fb_addr = FrameBufferPhysicalAddress;
    plane_state.mode_change = true;
    m_Pitch = plane_state.pitch;

    printk_debug("HwCommitVidPn: plane_state w=%d h=%d pitch=%d wincolorfmt=%d drmcolorfmt=%d addr=0x%x\n", plane_state.src_w, plane_state.src_h, plane_state.pitch, (UINT)ColorFormat, plane_state.format, plane_state.fb_addr);
    printk_debug("HwCommitVidPn: target_mode totw=%d toth=%d actw=%d acth=%d pclk=%d\n", pTargetMode->VideoSignalInfo.TotalSize.cx, pTargetMode->VideoSignalInfo.TotalSize.cy,
        pTargetMode->VideoSignalInfo.ActiveSize.cx, pTargetMode->VideoSignalInfo.ActiveSize.cy, pTargetMode->VideoSignalInfo.PixelRate);

    //
    // For detailed mode timing info, a monitor mode (commonly native)
    // in EDID should be matched to based on the target/source mode
    //
    // And then code ported from crtc_atomic_enable() can be used
    // to actually set the monitor mode on the DCSS display controller
    //

    void *pEdid;
    struct videomode vm;

    UINT EdidSize = m_DsiTransmitter.GetCachedEdid(&pEdid);
    if (!GetDisplayModeTiming(pEdid, EdidSize, pTargetMode, &vm))
    {
        printk("HwCommitVidPn: Error getting display timing from EDID.\n");
        return STATUS_INVALID_PARAMETER;
    }
    /* Check mode */
    UINT bus_format = (UINT)sec_mipi_dsim_atomic_get_input_bus_fmts(&m_DsiTransmitter.dsi_pdev);

    /* Check bus format */
    if (lcdifv3_crtc_atomic_check(&lcdif_crtc_pdev, bus_format) != 0) {
        printk("HwCommitVidPn: Error Display mode not supported by LCDIF controller.\n");
        return STATUS_INVALID_PARAMETER;
    }
    /* update videomode to comply to HW configuration/limits */
    sec_mipi_dsim_bridge_atomic_check(&m_DsiTransmitter.dsi_pdev, &vm);

    if (m_DsiTransmitter.m_i2c_main.is_initialized) {
        if (adv7511_bridge_mode_valid(&m_DsiTransmitter.m_i2c_main, &vm) != MODE_OK) {
            printk("HwCommitVidPn: Error Display mode not supported by ADV7535 converter.\n");
            return STATUS_INVALID_PARAMETER;
        }
        adv7511_bridge_disable(&m_DsiTransmitter.m_i2c_main);
    }

    sec_mipi_dsim_bridge_disable(&m_DsiTransmitter.dsi_pdev);
    /* imx_sec_dsim_encoder_helper_disable just assert mipi reset - call after sec_mipi_dsim_bridge_disable */
    imx_sec_dsim_encoder_helper_disable(&m_DsiTransmitter.dsi_pdev);

    lcdifv3_crtc_atomic_disable(&lcdif_crtc_pdev);

    lcdifv3_plane_atomic_update(&lcdif_crtc_pdev, CRTC_PLANE_INDEX_PRIMARY, &plane_state);

    lcdifv3_crtc_mode_set(&lcdif_crtc_pdev, &vm, bus_format);

    /* Stores the videomode, and set reference clock, peripheral is configured later in sec_mipi_dsim_bridge_enable
       Must be called after PLL reference is determined in lcdifv3_crtc_mode_set, where pixel clock (PLL) is set */
    sec_mipi_dsim_bridge_mode_set(&m_DsiTransmitter.dsi_pdev, &vm);
    /* Check DSI PLL settings supports videomode pixel clock
       must be called after PLL reference is set in sec_mipi_dsim_bridge_mode_set */
    if (imx_sec_dsim_encoder_helper_atomic_check(&m_DsiTransmitter.dsi_pdev, &vm) != 0) {
        return STATUS_INVALID_PARAMETER;
    }

    if (m_DsiTransmitter.m_i2c_main.is_initialized) {
        adv7511_bridge_mode_set(&m_DsiTransmitter.m_i2c_main, &vm);
    }

    /* Enable DSI before LCDIF, so the panel have the chance to get initialized before the stream of frames starts */
    imx_sec_dsim_encoder_helper_enable(&m_DsiTransmitter.dsi_pdev);
    sec_mipi_dsim_bridge_enable(&m_DsiTransmitter.dsi_pdev);

    /* Finally enable the controller */
    lcdifv3_crtc_atomic_enable(&lcdif_crtc_pdev);

    if (m_DsiTransmitter.m_i2c_main.is_initialized) {
        adv7511_bridge_enable(&m_DsiTransmitter.m_i2c_main);
    }

    m_CurSourceModes[0] = *pSourceMode;
    m_CurTargetModes[0] = *pTargetMode;

    m_bNotifyVSync = true;

    m_ScanoutFormat = ColorFormat;

    return STATUS_SUCCESS;
}

NTSTATUS
GcKmImx8mpMipiDsiDisplay::ControlInterrupt(
    IN_CONST_DXGK_INTERRUPT_TYPE    InterruptType,
    IN_BOOLEAN  EnableInterrupt)
{
    switch (InterruptType)
    {
    case DXGK_INTERRUPT_CRTC_VSYNC:
    case DXGK_INTERRUPT_DISPLAYONLY_VSYNC:
        if (EnableInterrupt) {
            lcdifv3_enable_vblank(&lcdif_crtc_pdev);
        }
        else {
            lcdifv3_disable_vblank(&lcdif_crtc_pdev);
        }
        break;
    }

    return STATUS_SUCCESS;
}

GC_PAGED_SEGMENT_END; //========================================================

GC_NONPAGED_SEGMENT_BEGIN; //===================================================

NTSTATUS
GcKmImx8mpMipiDsiDisplay::SetVidPnSourceAddressWithMultiPlaneOverlay3(
    IN_OUT_PDXGKARG_SETVIDPNSOURCEADDRESSWITHMULTIPLANEOVERLAY3 pSetMpo3)
{
    struct lcdifv3_plane_state plane_state;

    NT_ASSERT(pSetMpo3->PlaneCount == 1);
    if (pSetMpo3->ppPlanes[0]->InputFlags.Enabled)
    {
        NT_ASSERT(pSetMpo3->ppPlanes[0]->ContextCount == 1);

        GcKmAllocation *pAllocation = (GcKmAllocation *)pSetMpo3->ppPlanes[0]->ppContextData[0]->hAllocation;

        m_FrontBufferSegmentOffset = pSetMpo3->ppPlanes[0]->ppContextData[0]->SegmentAddress;
        plane_state.fb_addr = m_LocalVidMemPhysicalBase + m_FrontBufferSegmentOffset.LowPart + pAllocation->m_linearOffset;

        plane_state.format = TranslateDxgiToDrmFormat(pAllocation->m_format);
        plane_state.pitch = pAllocation->m_hwPitch;
        plane_state.src_w = pAllocation->m_mip0Info.PhysicalWidth;
        plane_state.src_h = pAllocation->m_mip0Info.PhysicalHeight;

        if (pAllocation->m_format != m_ScanoutFormat)
        {
            plane_state.mode_change = true;
            m_ScanoutFormat = pAllocation->m_format;
        }
        else
        {
            plane_state.mode_change = false;
        }

        lcdifv3_plane_atomic_update(&lcdif_crtc_pdev, CRTC_PLANE_INDEX_PRIMARY, &plane_state);
        lcdifv3_crtc_atomic_flush(&lcdif_crtc_pdev);
    }
    else
    {
        m_FrontBufferSegmentOffset.QuadPart = -1L;
    }
    return STATUS_SUCCESS;
}

BOOLEAN
GcKmImx8mpMipiDsiDisplay::InterruptRoutine(UINT MessageNumber)
{
    BOOLEAN handled = FALSE;
    if (GcKmdGlobal::s_DriverMode == RenderOnly)
    {
        return FALSE;
    }

    if (sec_mipi_dsim_irq_handler(&m_DsiTransmitter.dsi_pdev.dev) == IRQ_HANDLED) {
        handled |= TRUE;
    }
    if (lcdifv3_poll_vblank(&lcdif_crtc_pdev))
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
        m_InterruptData.CrtcVsync.VidPnTargetId = BaseTransmitter::MIPI_DSI0_CHILD_UID;
        m_InterruptData.CrtcVsync.PhysicalAddress = m_FrontBufferSegmentOffset;
        m_InterruptData.CrtcVsync.PhysicalAdapterMask = 1;
        m_InterruptData.Flags.ValidPhysicalAdapterMask = TRUE;

        if (m_bNotifyVSync)
        {
            m_pDxgkInterface->DxgkCbNotifyInterrupt(m_pDxgkInterface->DeviceHandle, &m_InterruptData);
        }

        lcdifv3_clear_vblank(&lcdif_crtc_pdev);

        if (m_bNotifyVSync)
        {
            m_pDxgkInterface->DxgkCbQueueDpc(m_pDxgkInterface->DeviceHandle);
        }

        handled |= TRUE;
    }

    return handled;
}

GC_NONPAGED_SEGMENT_END; //=====================================================
