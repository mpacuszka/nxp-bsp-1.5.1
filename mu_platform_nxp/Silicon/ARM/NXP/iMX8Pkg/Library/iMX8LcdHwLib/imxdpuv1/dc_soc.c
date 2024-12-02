/*
* Copyright 2023-2024 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <Base.h>
#include <Uefi/UefiBaseType.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <iMXDisplay.h>
#include "iMX8.h"
#include "svc/types.h"
#include "svc/pm/pm_api.h"
#include "svc/misc/misc_api.h"
#include <imxdpuv1.h>
#include <imxdpuv1_registers.h>
#include <imxdpuv1_events.h>

#define LPCG_CLOCK_MASK         0x3U
#define LPCG_CLOCK_OFF          0x0U
#define LPCG_CLOCK_ON           0x2U

#define DC_0_LPCG      0x56010000
#define DI_MIPI0_LPCG  0x56223000
#define DI_MIPI1_LPCG  0x56243000

/* Level of debug messages, error reports are not included */
#define DCSOC_DEBUG_LEVEL DEBUG_INFO
/* #define DCSOC_DEBUG_CLOCK */

static VOID LpcgWrite(UINT32 lpcgVal, UINT32 addr)
{
  /*
  * Write twice with 4x DSC clock cycles (40x IPS clock cycles) interval
  * to work around LPCG issue
  */
  MmioWrite32(addr, lpcgVal);
  MicroSecondDelay(10);
  MmioWrite32(addr, lpcgVal);
  MicroSecondDelay(10);
}

static VOID LpcgOn(UINT32 addr, UINT8 clk)
{
  UINT32 lpcgVal;

  lpcgVal = MmioRead32(addr);
  lpcgVal &= ~((UINT32)(LPCG_CLOCK_MASK) << (clk * 4U));
  lpcgVal |= ((UINT32)(LPCG_CLOCK_ON) << (clk * 4U));
  LpcgWrite(lpcgVal, addr);
}

static BOOLEAN LpcgIsOn(UINT32 addr, UINT8 clk)
{
  UINT32 lpcgVal;

  lpcgVal = MmioRead32(addr);
  lpcgVal = (lpcgVal >> (clk * 4U)) & (UINT32)(LPCG_CLOCK_MASK);

  if (lpcgVal == LPCG_CLOCK_ON) {
    return TRUE;
  }

  return FALSE;
}

EFI_STATUS DcSocInit(imxDisplayInterfaceType displayInterface, UINT32 PixelClk)
{
  UINT32 err;
  sc_rsrc_t DcResource = SC_R_DC_0, PllResource;
  UINT32 PllClock;
  const UINT32 MaxPllRate = 1200000000;
  const UINT32 MinPllRate = 650000000;
  sc_pm_clk_t MiscClk;
  sc_ctrl_t LinkAddr, LinkEnable, LinkValid, Sync;
  UINT32 disp_id = ((displayInterface == imxMipiDsi1) || (displayInterface == imxLvds1)) ? 1 : 0;
  UINT32 DcLpcg = DC_0_LPCG;

  DEBUG((DCSOC_DEBUG_LEVEL, "DcSocInit: disp_id = %d\n", disp_id));

  if (disp_id == 0) {
    PllResource = SC_R_DC_0_PLL_0;
  } else {
    PllResource = SC_R_DC_0_PLL_1;
  }

  if (disp_id == 0) {
    MiscClk = SC_PM_CLK_MISC0;
    LinkAddr = SC_C_PXL_LINK_MST1_ADDR;
    LinkEnable = SC_C_PXL_LINK_MST1_ENB;
    LinkValid = SC_C_PXL_LINK_MST1_VLD;
    Sync = SC_C_SYNC_CTRL0;
  } else {
    MiscClk = SC_PM_CLK_MISC1;
    LinkAddr = SC_C_PXL_LINK_MST2_ADDR;
    LinkEnable = SC_C_PXL_LINK_MST2_ENB;
    LinkValid = SC_C_PXL_LINK_MST2_VLD;
    Sync = SC_C_SYNC_CTRL1;
  }

  PllClock = 0;
  while (PllClock <= MaxPllRate) {
    PllClock += PixelClk;
  }
  PllClock -= PixelClk;
  if (PllClock < MinPllRate) {
    DEBUG((DEBUG_ERROR, "ERROR: setting dpu pll clock (%u) for requested pixel clock (%u).\n", PllClock, PixelClk));
    return EFI_INVALID_PARAMETER;
  }

  DEBUG((DCSOC_DEBUG_LEVEL, "DcSocInit: PllClock = %d PixelClk = %d\n", PllClock, PixelClk));

  err = sc_pm_set_clock_rate(SC_IPC_HDL, PllResource, SC_PM_CLK_PLL, &PllClock);
  if (err) {
    DEBUG((DEBUG_ERROR, "PLL%d set clock rate failed! (error = %d)\n", disp_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_set_clock_parent(SC_IPC_HDL, DcResource, MiscClk,
                              (MiscClk == SC_PM_CLK_MISC0) ? 2 : 3);
  if (err) {
    DEBUG((DEBUG_ERROR, "DISP%d set clock parent failed! (error = %d)\n", disp_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_set_clock_rate(SC_IPC_HDL, DcResource, MiscClk, &PixelClk);
  if (err) {
    DEBUG((DEBUG_ERROR, "DISP%d set clock rate failed! (error = %d)\n", disp_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_clock_enable(SC_IPC_HDL, PllResource, SC_PM_CLK_PLL, true, false);
  if (err) {
    DEBUG((DEBUG_ERROR, "PLL%d clock enable failed! (error = %d)\n", disp_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_clock_enable(SC_IPC_HDL, DcResource, MiscClk, true, false);
  if (err) {
    DEBUG((DEBUG_ERROR, "Disp%d clock enable failed! (error = %d)\n", disp_id, err));
    return EFI_DEVICE_ERROR;
  }

  LpcgOn(DcLpcg, disp_id);
  while (!LpcgIsOn(DcLpcg, disp_id)) {}

  err = sc_misc_set_control(SC_IPC_HDL, DcResource, LinkAddr, 0);
  if (err) {
    DEBUG((DEBUG_ERROR, "DC MST%d_ADDR failed! (error = %d)\n", disp_id + 1, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_misc_set_control(SC_IPC_HDL, DcResource, LinkEnable, 0); /*Enabled later*/
  if (err) {
    DEBUG((DEBUG_ERROR, "DC MST%d_ENB failed! (error = %d)\n", disp_id + 1, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_misc_set_control(SC_IPC_HDL, DcResource, LinkValid, 1);
  if (err) {
    DEBUG((DEBUG_ERROR, "DC MST%d_VLD failed! (error = %d)\n", disp_id + 1, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_misc_set_control(SC_IPC_HDL, DcResource, Sync, 1);
  if (err) {
    DEBUG((DEBUG_ERROR, "DC SYNC_CTRL%d failed! (error = %d)\n", disp_id, err));
    return EFI_DEVICE_ERROR;
  }

#ifdef DCSOC_DEBUG_CLOCK
  err = sc_pm_get_clock_rate(SC_IPC_HDL, PllResource, SC_PM_CLK_PLL, &PllClock);
  DEBUG((DCSOC_DEBUG_LEVEL, "DcSocInit: SC_PM_CLK_PLL rate = %d err=%d\n", PllClock, err));
  err = sc_pm_get_clock_rate(SC_IPC_HDL, DcResource, MiscClk, &PllClock);
  DEBUG((DCSOC_DEBUG_LEVEL, "DcSocInit: SC_PM_CLK_MISC%d rate = %d err=%d\n", disp_id, PllClock, err));
#endif

  return EFI_SUCCESS;
}

EFI_STATUS DcVideoInit(imxDisplayInterfaceType displayInterface, IMX_DISPLAY_TIMING *Timing,
                   EFI_PHYSICAL_ADDRESS  FrameBaseAddress, BOOLEAN WaitFrmCntrMove)
{
  struct imxdpuv1_videomode mode;
  imxdpuv1_channel_params_t channel;
  imxdpuv1_layer_t layer;
  imxdpuv1_layer_idx_t layer_idx;
  int8_t imxdpuv1_id = 0;
  UINT32 disp_id = ((displayInterface == imxMipiDsi1) || (displayInterface == imxLvds1)) ? 1 : 0;
  UINT32 gpixfmt = IMXDPUV1_PIX_FMT_BGRA32;

  DEBUG((DCSOC_DEBUG_LEVEL, "DcVideoInit: disp_id = %d\n", disp_id));
  DEBUG((DCSOC_DEBUG_LEVEL, "DcVideoInit: FB addr: 0x%llx\n", FrameBaseAddress));

  mode.pixelclock = Timing->PixelClock;
  mode.hlen = Timing->HActive;
  mode.hbp = Timing->HBlank - Timing->HSyncOffset - Timing->HSync;
  mode.hfp = Timing->HSyncOffset;
  mode.vlen = Timing->VActive;
  mode.vbp = Timing->VBlank - Timing->VSyncOffset - Timing->VSync;
  mode.vfp = Timing->VSyncOffset;
  mode.hsync = Timing->HSync;
  mode.vsync = Timing->VSync;
  mode.flags = IMXDPUV1_MODE_FLAGS_DE_POL;

  imxdpuv1_init(imxdpuv1_id);
  imxdpuv1_disp_enable_frame_gen(imxdpuv1_id, 0, IMXDPUV1_FALSE);
  imxdpuv1_disp_enable_frame_gen(imxdpuv1_id, 1, IMXDPUV1_FALSE);

  imxdpuv1_disp_setup_frame_gen(imxdpuv1_id, disp_id,
    (const struct imxdpuv1_videomode *)&mode,
    0x3ff, 0, 0, 1, IMXDPUV1_DISABLE);

  if (!WaitFrmCntrMove) {
    /* For oled panel start TCON early*/
    imxdpuv1_disp_setup_tcon_operation_mode(imxdpuv1_id, disp_id);
  }

  imxdpuv1_disp_init(imxdpuv1_id, disp_id);
  imxdpuv1_disp_setup_constframe(imxdpuv1_id, disp_id, 0, 0, 0xff, 0);

  if (disp_id == 0) {
    channel.common.chan = IMXDPUV1_CHAN_VIDEO_0;
  } else {
    channel.common.chan = IMXDPUV1_CHAN_VIDEO_1;
  }
  channel.common.src_pixel_fmt = gpixfmt;
  channel.common.dest_pixel_fmt = gpixfmt;
  channel.common.src_width = mode.hlen;
  channel.common.src_height = mode.vlen;

  channel.common.clip_width = 0;
  channel.common.clip_height = 0;
  channel.common.clip_top = 0;
  channel.common.clip_left = 0;

  channel.common.dest_width = mode.hlen;
  channel.common.dest_height = mode.vlen;
  channel.common.dest_top = 0;
  channel.common.dest_left = 0;
  channel.common.stride = mode.hlen * imxdpuv1_bytes_per_pixel(IMXDPUV1_PIX_FMT_BGRA32);

  channel.common.disp_id = disp_id;
  channel.common.const_color = 0;
  channel.common.use_global_alpha = 0;
  channel.common.use_local_alpha = 0;
  imxdpuv1_init_channel(imxdpuv1_id, &channel);

  imxdpuv1_init_channel_buffer(imxdpuv1_id,
    channel.common.chan, mode.hlen * imxdpuv1_bytes_per_pixel(IMXDPUV1_PIX_FMT_RGB32),
    IMXDPUV1_ROTATE_NONE, (dma_addr_t)FrameBaseAddress, 0, 0);

  layer.enable    = IMXDPUV1_TRUE;
  layer.secondary = get_channel_blk(channel.common.chan);

  if (disp_id == 0) {
    layer_idx = IMXDPUV1_LAYER_0;
    layer.stream    = IMXDPUV1_DISPLAY_STREAM_0;
    layer.primary   = IMXDPUV1_ID_CONSTFRAME0;
  } else {
    layer_idx = IMXDPUV1_LAYER_2;
    layer.stream    = IMXDPUV1_DISPLAY_STREAM_1;
    layer.primary   = IMXDPUV1_ID_CONSTFRAME1;
  }

  imxdpuv1_disp_setup_layer(imxdpuv1_id, &layer, layer_idx, 1);
  imxdpuv1_disp_set_layer_global_alpha(imxdpuv1_id, layer_idx, 0xff);

  imxdpuv1_disp_set_layer_position(imxdpuv1_id, layer_idx, 0, 0);
  imxdpuv1_disp_set_chan_position(imxdpuv1_id, channel.common.chan, 0, 0);

  imxdpuv1_disp_enable_frame_gen(imxdpuv1_id, disp_id, IMXDPUV1_ENABLE);

  if (WaitFrmCntrMove) {
    /* ADV7535 converter needs this to synchronize image */
    imxdpuv1_disp_framegen_wait_frm_cntr_move(imxdpuv1_id, disp_id);
    imxdpuv1_disp_setup_tcon_operation_mode(imxdpuv1_id, disp_id);
  }

  return EFI_SUCCESS;
}

EFI_STATUS DcPixelLinkStop(imxDisplayInterfaceType displayInterface)
{
  UINT32 err;
  sc_ctrl_t LinkValid, Sync;
  UINT32 disp_id = ((displayInterface == imxMipiDsi1) || (displayInterface == imxLvds1)) ? 1 : 0;
  sc_rsrc_t DcResource = SC_R_DC_0;

  if (disp_id == 0) {
    LinkValid = SC_C_PXL_LINK_MST1_VLD;
    Sync = SC_C_SYNC_CTRL0;
  } else {
    LinkValid = SC_C_PXL_LINK_MST2_VLD;
    Sync = SC_C_SYNC_CTRL1;
  }

  err = sc_misc_set_control(SC_IPC_HDL, DcResource, LinkValid, 0);
  if (err) {
    DEBUG((DEBUG_ERROR, "DC MST%d_VLD failed! (error = %d)\n", disp_id + 1, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_misc_set_control(SC_IPC_HDL, DcResource, Sync, 0);
  if (err) {
    DEBUG((DEBUG_ERROR, "DC SYNC_CTRL%d failed! (error = %d)\n", disp_id, err));
    return EFI_DEVICE_ERROR;
  }

  DEBUG((DCSOC_DEBUG_LEVEL, "DcPixelLinkStop: disp_id = %d\n", disp_id));
  return EFI_SUCCESS;
}

EFI_STATUS DcPixelLinkEnDi(imxDisplayInterfaceType displayInterface, BOOLEAN enable)
{
  UINT32 err, en;
  sc_ctrl_t LinkEnable;
  UINT32 disp_id = ((displayInterface == imxMipiDsi1) || (displayInterface == imxLvds1)) ? 1 : 0;
  sc_rsrc_t DcResource = SC_R_DC_0;

  if (disp_id == 0) {
    LinkEnable = SC_C_PXL_LINK_MST1_ENB;
  } else {
    LinkEnable = SC_C_PXL_LINK_MST2_ENB;
  }

  en = enable ? 1 : 0;
  err = sc_misc_set_control(SC_IPC_HDL, DcResource, LinkEnable, en);
  if (err) {
    DEBUG((DEBUG_ERROR, "DC MST%d_ENB failed! (error = %d)\n", disp_id + 1, err));
    return EFI_DEVICE_ERROR;
  }

  DEBUG((DCSOC_DEBUG_LEVEL, "DcPixelLinkEnDi: disp_id = %d enable = %d\n", disp_id, enable));

  return EFI_SUCCESS;
}


EFI_STATUS DcVideoStop(imxDisplayInterfaceType displayInterface)
{
  int8_t imxdpuv1_id = 0;
  UINT32 disp_id = ((displayInterface == imxMipiDsi1) || (displayInterface == imxLvds1)) ? 1 : 0;

  imxdpuv1_init_drv(imxdpuv1_id);
  imxdpuv1_disp_enable_frame_gen(imxdpuv1_id, disp_id, IMXDPUV1_DISABLE);

  imxdpuv1_disp_wait_framegen_done(imxdpuv1_id, disp_id);

  DEBUG((DCSOC_DEBUG_LEVEL, "DcVideoStop: disp_id = %d\n", disp_id));

  return EFI_SUCCESS;
}

EFI_STATUS DcClockStop(imxDisplayInterfaceType displayInterface)
{
  UINT32 err;
  UINT32 disp_id = ((displayInterface == imxMipiDsi1) || (displayInterface == imxLvds1)) ? 1 : 0;
  sc_rsrc_t DcResource = SC_R_DC_0, PllResource;
  sc_pm_clk_t MiscClk;

  if (disp_id == 0) {
    PllResource = SC_R_DC_0_PLL_0;
    MiscClk = SC_PM_CLK_MISC0;
  } else {
    PllResource = SC_R_DC_0_PLL_1;
    MiscClk = SC_PM_CLK_MISC1;
  }

  err = sc_pm_clock_enable(SC_IPC_HDL, PllResource, SC_PM_CLK_PLL, false, false);
  if (err) {
    DEBUG((DEBUG_ERROR, "PLL%d clock disable failed! (error = %d)\n", disp_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_clock_enable(SC_IPC_HDL, DcResource, MiscClk, false, false);
  if (err) {
    DEBUG((DEBUG_ERROR, "Disp%d clock disable failed! (error = %d)\n", disp_id, err));
    return EFI_DEVICE_ERROR;
  }

  DEBUG((DCSOC_DEBUG_LEVEL, "DcClockStop: disp_id = %d\n", disp_id));

  return EFI_SUCCESS;
}
