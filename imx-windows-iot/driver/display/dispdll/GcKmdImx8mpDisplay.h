/* Copyright (c) Microsoft Corporation.
 * Copyright 2022-2023 NXP
   Licensed under the MIT License. */

#pragma once

#include "GcKmdBaseDisplay.h"
#include "GcKmdMpLvdsTransmitter.h"

extern "C" {
#include "linux/platform_device.h"
#include "clk/clk_imx8mp.h"
}

class GcKmImx8mpDisplay : public GcKmBaseDisplay
{
public:

    GcKmImx8mpDisplay(struct DisplayInterface* di)
    {
        m_FrontBufferSegmentOffset.QuadPart = 0L;
        m_InterruptData = {};
        m_pTransmitter = &m_LvdsTransmitter;
        RtlCopyMemory(&m_di, di, sizeof(struct DisplayInterface));
    }

    virtual NTSTATUS HwStart(DXGKRNL_INTERFACE* pDxgkInterface) override;

    virtual NTSTATUS HwStop(
        DXGK_DISPLAY_INFORMATION   *pFwDisplayInfo, BOOLEAN DoCommitFwFb) override;

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

    MpLvdsTransmitter m_LvdsTransmitter;

    DXGKARGCB_NOTIFY_INTERRUPT_DATA m_InterruptData;

    PHYSICAL_ADDRESS    m_FrontBufferSegmentOffset;

    struct platform_device lcdif_pdev;
    struct platform_device lcdif_crtc_pdev;
    imx8mp_clk_device *clk_tree;
    struct DisplayInterface m_di;

};

