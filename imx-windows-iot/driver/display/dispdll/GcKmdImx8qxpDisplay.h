/* Copyright (c) Microsoft Corporation.
 * Copyright 2022-2024 NXP
   Licensed under the MIT License. */

#pragma once

#include "GcKmdQxpLvdsTransmitter.h"

#include "GcKmdBaseDisplay.h"

extern "C" {

#include <svc/scfw.h>

#include "linux/platform_device.h"
#include "clk/clk_imx8qxp.h"
#include "dpu/dpu-prv.h"
#include "dpu/dpu-crtc.h"
#include "dpu/dpu-plane.h"
#include "video/imx8-prefetch.h"

}

class GcKmImx8qxpDisplay : public GcKmBaseDisplay
{
public:

    GcKmImx8qxpDisplay(struct DisplayInterface *di, struct Imx8qxpDisplayPDEVs *pdevs)
    {
        m_FrontBufferSegmentOffset.QuadPart = 0L;
        m_OldFbPhysicalAddress = 0L;
        m_InterruptData = {};
        m_dpu_pdev = {};
        m_dpu = nullptr;
        RtlZeroMemory(m_client_devices, sizeof(m_client_devices));
        m_pTransmitter = &m_LvdsTransmitter;
        m_crtc = {};
        m_crtc_state = {};
        m_aux_crtc = {};
        m_crtc.aux_dpu_crtc = &m_aux_crtc;
        m_crtc.base.state = &m_crtc_state.imx_crtc_state.base;
        m_old_dmode = {};
        m_old_plane_state = {};
        m_old_fb = {};
        m_old_plane_state.fb = &m_old_fb;
        m_irqsteer_pdev = {};
        m_clk_tree = nullptr;
        RtlCopyMemory(&m_di, di, sizeof(struct DisplayInterface));
        //For now just assign plane 0 to LVDS0 (dpu stream 0), plane 1 for LVDS1 (dpi stream 1)
        m_PlaneId = m_di.DisplayInterfaceIndex;
        if (pdevs) {
            /* Multi display - use provided pdev objects*/
            m_p_dpu_pdev = pdevs->p_dpu_pdev;
            m_p_irqsteer_pdev = pdevs->p_irqsteer_pdev;
            m_p_prg_pdev = pdevs->p_prg_pdev;
            m_p_dprc_pdev = pdevs->p_dprc_pdev;
            m_p_client_devices = pdevs->p_client_devices;
            m_p_refCount = pdevs->p_refCount;
        }
        else {
            /* Single display - use local pdev objects*/
            m_p_dpu_pdev = &m_dpu_pdev;
            m_p_irqsteer_pdev = &m_irqsteer_pdev;
            m_p_prg_pdev = m_prg_pdev;
            m_p_dprc_pdev = m_dprc_pdev;
            m_p_client_devices = m_client_devices;
            m_p_refCount = &m_refCount;
            m_refCount = 0;
        }
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

    NTSTATUS HwAtomicCommit(struct videomode* vm);

    void HandleVsyncInterrupt(UINT Disp);

    NTSTATUS GetI2CResource(
        DXGKRNL_INTERFACE* pDxgkInterface,
        ULONG I2CId,
        LARGE_INTEGER* pI2CConnectionId);

    QxpLvdsTransmitter m_LvdsTransmitter;

    DXGKARGCB_NOTIFY_INTERRUPT_DATA m_InterruptData;

    PHYSICAL_ADDRESS    m_FrontBufferSegmentOffset;

    ULONG m_OldFbPhysicalAddress;

    struct platform_device m_prg_pdev[PRG_CNT];
    struct platform_device *m_p_prg_pdev;

    struct platform_device m_dprc_pdev[DPRC_CNT];
    struct platform_device *m_p_dprc_pdev;

    struct platform_device m_dpu_pdev;
    struct platform_device *m_p_dpu_pdev;
    struct dpu_soc *m_dpu;

    struct platform_device m_client_devices[CLIENT_DEVICE_CNT];
    struct platform_device *m_p_client_devices;

    struct dpu_crtc m_crtc;
    struct dpu_crtc_state m_crtc_state;

    // unused on QXP; added here to keep Linux drv flow consistent
    struct dpu_crtc m_aux_crtc;

    struct drm_display_mode m_old_dmode;
    struct drm_plane_state m_old_plane_state;
    struct drm_framebuffer m_old_fb;

    struct platform_device m_irqsteer_pdev;
    struct platform_device *m_p_irqsteer_pdev;

    struct imx8qxp_clk_device *m_clk_tree;

    struct DisplayInterface m_di;
    UINT m_PlaneId;
    UINT m_refCount; //reference count to display objects
    UINT *m_p_refCount;
};

