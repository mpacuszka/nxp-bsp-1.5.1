/* Copyright (c) Microsoft Corporation.
 * Copyright 2022-2023 NXP
   Licensed under the MIT License. */

#pragma once

#include "GcKmdHdmiTransmitter.h"

#include "GcKmdBaseDisplay.h"

extern "C" {

#include "linux/platform_device.h"
#include "clk/clk_imx8mq.h"
#include "dcss/dcss-dev.h"
#include "dcss/dcss-kms.h"
#include "mhdp/cdns-mhdp-imx.h"

}

class GcKmImx8mqDisplay : public GcKmBaseDisplay
{
public:

    GcKmImx8mqDisplay()
    {
        m_FrontBufferSegmentOffset.QuadPart = 0L;
        m_InterruptData = {};
        m_dcss_pdev = {};
        m_dcss_dev = nullptr;
        m_pTransmitter = &m_HdmiTransmitter;
        m_crtc = {};
        m_crtc_state = {};
        m_crtc.base.state = &m_crtc_state.base;
        m_mhdp_pdev = {};
        m_mhdp_dev = {};
        m_mhdp_dev.video_info.color_fmt = PXL_RGB;
        // TODO: get colorspace cfg from the userspace
        m_mhdp_dev.connector.state.colorspace = DRM_MODE_COLORIMETRY_DEFAULT;
        m_mhdp_dev.connector.state.connector = &m_mhdp_dev.connector.base;
        m_old_dmode = {};
        m_old_plane_state = {};
        m_old_fb = {};
        m_old_plane_state.fb = &m_old_fb;
        m_irqsteer_pdev = {};
        clk_tree = nullptr;
    }

    virtual NTSTATUS HwStart(DXGKRNL_INTERFACE* pDxgkInterface) override;

    virtual NTSTATUS HwStop(
        DXGK_DISPLAY_INFORMATION   *pFwDisplayInfo,
        BOOLEAN DoCommitFwFb) override;

    virtual void HwStopScanning(D3DDDI_VIDEO_PRESENT_TARGET_ID TargetId) override;

    virtual void HwSetPowerState(
        IN_ULONG                DeviceUid,
        IN_DEVICE_POWER_STATE   DevicePowerState,
        IN_POWER_ACTION         ActionType) override;

    virtual NTSTATUS SetVidPnSourceAddress(
        IN_CONST_PDXGKARG_SETVIDPNSOURCEADDRESS pSetVidPnSourceAddress) override;

    virtual NTSTATUS HwCommitVidPn(
        const D3DKMDT_VIDPN_SOURCE_MODE* pSourceMode,
        const D3DKMDT_VIDPN_TARGET_MODE* pTargetMode,
        IN_CONST_PDXGKARG_COMMITVIDPN_CONST pCommitVidPn) override;

    virtual BOOLEAN InterruptRoutine(
        UINT    MessageNumber) override;

    virtual NTSTATUS ControlInterrupt(
        IN_CONST_DXGK_INTERRUPT_TYPE    InterruptType,
        IN_BOOLEAN  EnableInterrupt) override;

    virtual NTSTATUS SetVidPnSourceAddressWithMultiPlaneOverlay3(
        IN_OUT_PDXGKARG_SETVIDPNSOURCEADDRESSWITHMULTIPLANEOVERLAY3 pSetVidPnSourceAddressWithMpo3) override;

private:

    void SetupFramebuffer(
        UINT FrameBufferPhysicalAddress,
        UINT Pitch,
        UINT DrmFormat,
        UINT64 TileMode);

    void SetupPlaneState(
        const D3DKMDT_VIDPN_SOURCE_MODE* pSourceMode,
        const D3DKMDT_VIDPN_TARGET_MODE* pTargetMode);

    NTSTATUS SetupHwCommit(
        const D3DKMDT_VIDPN_SOURCE_MODE* pSourceMode,
        const D3DKMDT_VIDPN_TARGET_MODE* pTargetMode,
        struct videomode* vm,
        UINT FrameBufferPhysicalAddress,
        UINT Pitch,
        UINT TileMode);

    void HwAtomicCommit(struct videomode* vm);

    void HandleVsyncInterrupt(void);

    HdmiTransmitter m_HdmiTransmitter;

    DXGKARGCB_NOTIFY_INTERRUPT_DATA m_InterruptData;

    PHYSICAL_ADDRESS    m_FrontBufferSegmentOffset;

    struct platform_device m_dcss_pdev;
    struct dcss_dev *m_dcss_dev;

    struct dcss_crtc m_crtc;
    struct dcss_crtc_state m_crtc_state;

    struct drm_display_mode m_old_dmode;
    struct drm_plane_state m_old_plane_state;
    struct drm_framebuffer m_old_fb;

    struct platform_device m_mhdp_pdev;
    //struct imx_mhdp_device *m_imx_mhdp;
    // TODO: get this device from m_mhdp_pdev
    struct cdns_mhdp_device m_mhdp_dev;

    struct platform_device m_irqsteer_pdev;

    imx8mq_clk_device_t *clk_tree;
};

