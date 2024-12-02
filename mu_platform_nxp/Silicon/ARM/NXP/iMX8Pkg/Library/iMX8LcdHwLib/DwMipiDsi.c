/*
 * Copyright 2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <Uefi/UefiBaseType.h>

#include <stdlib.h>

#include "MipiCommon.h"
#include "DwMipiDsi.h"
#include "DwMipiDsiRegisters.h"
#include "iMX93MipiDphy.h"
#include "Clk/Mediamix.h"

#define DSI_DEBUG_LEVEL DEBUG_INFO

/* Fixed clock rates and limits */
#define DSI_MAX_ESC_CLOCK_KHZ     20000

#define HSTT(_maxfreq, _c_lp2hs, _c_hs2lp, _d_lp2hs, _d_hs2lp)  \
{                                                               \
  .Maxfreq = (_maxfreq),                                        \
  .Timing = {                                                   \
    .DataHs2lp = (_d_hs2lp),                                    \
    .DataLp2hs = (_d_lp2hs),                                    \
    .ClkHs2lp = (_c_hs2lp),                                     \
    .ClkLp2hs = (_c_lp2hs),                                     \
  }                                                             \
}

/* High-Speed Transition Times */
struct HSTT HsttTable[] = {
  HSTT(80,    21,  17,  15, 10),
  HSTT(90,    23,  17,  16, 10),
  HSTT(100,   22,  17,  16, 10),
  HSTT(110,   25,  18,  17, 11),
  HSTT(120,   26,  20,  18, 11),
  HSTT(130,   27,  19,  19, 11),
  HSTT(140,   27,  19,  19, 11),
  HSTT(150,   28,  20,  20, 12),
  HSTT(160,   30,  21,  22, 13),
  HSTT(170,   30,  21,  23, 13),
  HSTT(180,   31,  21,  23, 13),
  HSTT(190,   32,  22,  24, 13),
  HSTT(205,   35,  22,  25, 13),
  HSTT(220,   37,  26,  27, 15),
  HSTT(235,   38,  28,  27, 16),
  HSTT(250,   41,  29,  30, 17),
  HSTT(275,   43,  29,  32, 18),
  HSTT(300,   45,  32,  35, 19),
  HSTT(325,   48,  33,  36, 18),
  HSTT(350,   51,  35,  40, 20),
  HSTT(400,   59,  37,  44, 21),
  HSTT(450,   65,  40,  49, 23),
  HSTT(500,   71,  41,  54, 24),
  HSTT(550,   77,  44,  57, 26),
  HSTT(600,   82,  46,  64, 27),
  HSTT(650,   87,  48,  67, 28),
  HSTT(700,   94,  52,  71, 29),
  HSTT(750,   99,  52,  75, 31),
  HSTT(800,  105,  55,  82, 32),
  HSTT(850,  110,  58,  85, 32),
  HSTT(900,  115,  58,  88, 35),
  HSTT(950,  120,  62,  93, 36),
  HSTT(1000, 128,  63,  99, 38),
  HSTT(1050, 132,  65, 102, 38),
  HSTT(1100, 138,  67, 106, 39),
  HSTT(1150, 146,  69, 112, 42),
  HSTT(1200, 151,  71, 117, 43),
  HSTT(1250, 153,  74, 120, 45),
  HSTT(1300, 160,  73, 124, 46),
  HSTT(1350, 165,  76, 130, 47),
  HSTT(1400, 172,  78, 134, 49),
  HSTT(1450, 177,  80, 138, 49),
  HSTT(1500, 183,  81, 143, 52),
  HSTT(1550, 191,  84, 147, 52),
  HSTT(1600, 194,  85, 152, 52),
  HSTT(1650, 201,  86, 155, 53),
  HSTT(1700, 208,  88, 161, 53),
  HSTT(1750, 212,  89, 165, 53),
  HSTT(1800, 220,  90, 171, 54),
  HSTT(1850, 223,  92, 175, 54),
  HSTT(1900, 231,  91, 180, 55),
  HSTT(1950, 236,  95, 185, 56),
  HSTT(2000, 243,  97, 190, 56),
  HSTT(2050, 248,  99, 194, 58),
  HSTT(2100, 252, 100, 199, 59),
  HSTT(2150, 259, 102, 204, 61),
  HSTT(2200, 266, 105, 210, 62),
  HSTT(2250, 269, 109, 213, 63),
  HSTT(2300, 272, 109, 217, 65),
  HSTT(2350, 281, 112, 225, 66),
  HSTT(2400, 283, 115, 226, 66),
  HSTT(2450, 282, 115, 226, 67),
  HSTT(2500, 281, 118, 227, 67),
};

STATIC EFI_STATUS DwMipiDsiPhyGetTiming(UINTN LaneRateMbps,
  struct DW_MIPI_DSI_DPHY_TIMING *Timing)
{
  ASSERT(Timing);

  UINTN Idx;
  for (Idx = 0; Idx < ARRAY_SIZE(HsttTable); Idx++) {
    if (LaneRateMbps <= HsttTable[Idx].Maxfreq) {
      break;
    }
  }
  if (Idx == ARRAY_SIZE(HsttTable)) {
    Idx--;
  }
  *Timing = HsttTable[Idx].Timing;

  DebugPrint(DSI_DEBUG_LEVEL, "Found MIPI-DSI-PHY timing for %u <= %u (lane rate Mbps)\n",
    LaneRateMbps, HsttTable[Idx].Maxfreq);

  return EFI_SUCCESS;
}

STATIC VOID DwMipiDsiDphyTimingConfig(IMX_DISPLAY_TIMING* Timing)
{
  ASSERT(Timing);
  struct DW_MIPI_DSI_DPHY_TIMING Dtiming;

  INTN LaneRateMbps = MipiGetLaneRateMbps(Timing);

  DwMipiDsiPhyGetTiming(LaneRateMbps, &Dtiming);

  UINT32 HwVersion = MIPI_DSI_VERSION;
  DebugPrint(DSI_DEBUG_LEVEL, "Detected MIPI-DSI version: 0x%x\n", HwVersion);
  DebugPrint(DSI_DEBUG_LEVEL, "lane-rate-mbps-%d, DataHs2lp-0x%x, DataLp2hs-0x%x\n",
    LaneRateMbps, Dtiming.DataHs2lp, Dtiming.DataLp2hs);

  if (HwVersion >= HWVER_131) {
    MIPI_DSI_PHY_TMR_CFG = PHY_HS2LP_TIME_V131(Dtiming.DataHs2lp) |
      PHY_LP2HS_TIME_V131(Dtiming.DataLp2hs);
    MIPI_DSI_PHY_TMR_RD_CFG = MAX_RD_TIME_V131(10000);
  } else {
    DebugPrint(DEBUG_ERROR, "Unsupported MIPI IP version-0x%x\n", HwVersion);
  }

  MIPI_DSI_PHY_TMR_LPCLK_CFG = PHY_CLKHS2LP_TIME(Dtiming.ClkHs2lp) |
    PHY_CLKLP2HS_TIME(Dtiming.ClkLp2hs);
}

STATIC EFI_STATUS DwMipiDsiDphyEnable()
{

  MIPI_DSI_PHY_RSTZ = PHY_FORCEPLL_MASK | PHY_ENABLECLK_MASK |
    PHY_RSTZ_MASK | PHY_SHUTDOWNZ_MASK;

  INTN Loop = 0;
  while ((MIPI_DSI_PHY_STATUS & PHY_LOCK_MASK) == 0) {
    MicroSecondDelay(10U);
    Loop++;
    if (Loop > 100U) {
      DebugPrint(DEBUG_ERROR, "Wait on DSI_PHY_STATUS failed (PHY_STATUS 0x%x)\n",
        MIPI_DSI_PHY_STATUS);
      return EFI_DEVICE_ERROR;
    }
  }

  Loop = 0;
  while ((MIPI_DSI_PHY_STATUS & PHY_STOP_STATE_CLK_LANE_MASK) == 0) {
    MicroSecondDelay(10U);
    Loop++;
    if (Loop > 100U) {
      DebugPrint(DEBUG_ERROR, "Failed to wait phy clk lane stop state (PHY_STATUS 0x%x)\n",
        MIPI_DSI_PHY_STATUS);
      return EFI_DEVICE_ERROR;
    }
  }

  return EFI_SUCCESS;
}

STATIC VOID DwMipiDsiSetMode(UINTN ModeFlags)
{
  MIPI_DSI_PWR_UP = RESET;

  if (ModeFlags) {
    MIPI_DSI_MODE_CFG = ENABLE_VIDEO_MODE;
    MIPI_DSI_VID_MODE_CFG = ENABLE_LOW_POWER_MASK |
      VID_MODE_TYPE_NON_BURST_SYNC_PULSES;
    MIPI_DSI_LPCLK_CTRL = PHY_TXREQUESTCLKHS;
  } else {
    MIPI_DSI_MODE_CFG = ENABLE_CMD_MODE;
  }

  MIPI_DSI_LPCLK_CTRL = PHY_TXREQUESTCLKHS;
  MIPI_DSI_PWR_UP = POWERUP;
}

/* Get lane byte clock cycles */
STATIC UINT32 DwMipiDsiGetHcomponentLbcc(UINT32 Hcomponent,
  IMX_DISPLAY_TIMING* Timing)
{
  ASSERT(Timing);

  INTN LaneRateKbps = MipiGetLaneRateKbps(Timing);
  if (LaneRateKbps < 0) {
    DebugPrint(DEBUG_ERROR, "Failed to get lane rate\n");
    return 0;
  }

  UINT32 Lbcc = Hcomponent * LaneRateKbps / 8;

  return DIV_RD_UP(Lbcc, GetPixelClockKHz(Timing));
}

STATIC VOID DwMipiDsiCommandModeConfig()
{
  MIPI_DSI_TO_CNT_CFG = HSTX_TO_CNT(1000) | LPRX_TO_CNT(1000);
  MIPI_DSI_BTA_TO_CNT = 0xD00;
  MIPI_DSI_MODE_CFG = ENABLE_CMD_MODE;
}

STATIC EFI_STATUS DwMipiDsiDpiConfig(IMX_DISPLAY_TIMING *Timing)
{
  ASSERT(Timing);

  if (Timing->PixelFormat != PIXEL_FORMAT_ARGB32 &&
      Timing->PixelFormat != PIXEL_FORMAT_BGRA32) {
    DebugPrint(DEBUG_ERROR, "Unsupported pixel format 0x%x\n",
      Timing->PixelFormat);
    return -EFI_INVALID_PARAMETER;
  }

  UINT32 Pol = 0;
  if (Timing->Flags & DISP_TIMING_INVERT_VSYNC) {
    Pol |= VSYNC_ACTIVE_LOW;
  }
  if (Timing->Flags & DISP_TIMING_INVERT_HSYNC) {
    Pol |= HSYNC_ACTIVE_LOW;
  }

  MIPI_DSI_DPI_VCID = DPI_VCID(0);
  MIPI_DSI_DPI_COLOR_CODING = DPI_COLOR_CODING_24BIT;
  MIPI_DSI_DPI_CFG_POL = Pol;
  MIPI_DSI_DPI_LP_CMD_TIM = OUTVACT_LPCMD_TIME(0x10) | INVACT_LPCMD_TIME(0x4);

  return EFI_SUCCESS;
}

EFI_STATUS DwDsiConfig(IMX_DISPLAY_TIMING* Timing)
{
  ASSERT(Timing);

  INTN LaneRateKbps = MipiGetLaneRateKbps(Timing);
  if (LaneRateKbps < 0) {
    DebugPrint(DEBUG_ERROR, "Failed to retrieve lane rate.\n");
    return EFI_DEVICE_ERROR;
  }

  MIPI_DSI_PWR_UP = RESET;

  UINT32 EscClkDiv = DIV_RD_UP((LaneRateKbps >> 3), DSI_MAX_ESC_CLOCK_KHZ);
  MIPI_DSI_CLKMGR_CFG = TO_CLK_DIVISION(0) | TX_ESC_CLK_DIVISION(EscClkDiv);

  DwMipiDsiDpiConfig(Timing);

  /* Packet handler */
  MIPI_DSI_PCKHDL_CFG = CRC_RX_EN | ECC_RX_EN | BTA_EN;
  /* Video mode */
  MIPI_DSI_VID_MODE_CFG = ENABLE_LOW_POWER_MASK |
    VID_MODE_TYPE_NON_BURST_SYNC_PULSES;
  /* Video packet */
  MIPI_DSI_VID_PKT_SIZE = VID_PKT_SIZE(Timing->HActive);

  DwMipiDsiCommandModeConfig();

  /* Horizontal timer config */
  UINT32 Lbcc = DwMipiDsiGetHcomponentLbcc(Timing->HActive + Timing->HBlank,
    Timing);
  MIPI_DSI_VID_HLINE_TIME = Lbcc;

  Lbcc = DwMipiDsiGetHcomponentLbcc(Timing->HSync, Timing);
  MIPI_DSI_VID_HSA_TIME = Lbcc;

  Lbcc = DwMipiDsiGetHcomponentLbcc(
    Timing->HBlank - Timing->HSync - Timing->HSyncOffset, Timing);
  MIPI_DSI_VID_HBP_TIME = Lbcc;

  /* Vertical timing config */
  MIPI_DSI_VID_VACTIVE_LINES = Timing->VActive;
  MIPI_DSI_VID_VSA_LINES = Timing->VSync;
  MIPI_DSI_VID_VFP_LINES = Timing->VSyncOffset;
  MIPI_DSI_VID_VBP_LINES = Timing->VBlank - Timing->VSync - Timing->VSyncOffset;

  /* DPHY */
  /* Clear PHY state */
  MIPI_DSI_PHY_RSTZ = 0;
  MIPI_DSI_PHY_TST_CTRL0 = PHY_UNTESTCLR;
  MIPI_DSI_PHY_TST_CTRL0 = PHY_TESTCLR;
  MIPI_DSI_PHY_TST_CTRL0 = PHY_UNTESTCLR;

  DwMipiDsiDphyTimingConfig(Timing);

  /* PHY Interface config */
  MIPI_DSI_PHY_IF_CFG = PHY_STOP_WAIT_TIME(0x20) | N_LANES(4);

  DsiPhyConfigure(Timing);

  DwMipiDsiDphyEnable();

  /* Clear possible error */
  MIPI_DSI_INT_ST0;
  MIPI_DSI_INT_ST1;
  MIPI_DSI_INT_MSK0 = 0;
  MIPI_DSI_INT_MSK1 = 0;

  DwMipiDsiSetMode(1);

  DwDsiDumpRegs();
  MediamixDumpReg();

  return EFI_SUCCESS;
}

VOID DwDsiStop()
{
  MIPI_DSI_PWR_UP = 0;
  MIPI_DSI_PHY_RSTZ = 0;
}

VOID DwDsiDumpRegs()
{
  DebugPrint(DSI_DEBUG_LEVEL, "VERSION            0x%x\n", MIPI_DSI_VERSION);
  DebugPrint(DSI_DEBUG_LEVEL, "PWR_UP             0x%x\n", MIPI_DSI_PWR_UP);
  DebugPrint(DSI_DEBUG_LEVEL, "CLKMGR_CFG         0x%x\n", MIPI_DSI_CLKMGR_CFG);
  DebugPrint(DSI_DEBUG_LEVEL, "DPI_VCID           0x%x\n", MIPI_DSI_DPI_VCID);
  DebugPrint(DSI_DEBUG_LEVEL, "DPI_COLOR_CODING   0x%x\n", MIPI_DSI_DPI_COLOR_CODING);
  DebugPrint(DSI_DEBUG_LEVEL, "DPI_CFG_POL        0x%x\n", MIPI_DSI_DPI_CFG_POL);
  DebugPrint(DSI_DEBUG_LEVEL, "DPI_LP_CMD_TIM     0x%x\n", MIPI_DSI_DPI_LP_CMD_TIM);
  DebugPrint(DSI_DEBUG_LEVEL, "PCKHDL_CFG         0x%x\n", MIPI_DSI_PCKHDL_CFG);
  DebugPrint(DSI_DEBUG_LEVEL, "GEN_VCID           0x%x\n", MIPI_DSI_GEN_VCID);
  DebugPrint(DSI_DEBUG_LEVEL, "MODE_CFG           0x%x\n", MIPI_DSI_MODE_CFG);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_MODE_CFG       0x%x\n", MIPI_DSI_VID_MODE_CFG);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_PKT_SIZE       0x%x\n", MIPI_DSI_VID_PKT_SIZE);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_NUM_CHUNKS     0x%x\n", MIPI_DSI_VID_NUM_CHUNKS);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_NULL_SIZE      0x%x\n", MIPI_DSI_VID_NULL_SIZE);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_HSA_TIME       0x%x\n", MIPI_DSI_VID_HSA_TIME);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_HBP_TIME       0x%x\n", MIPI_DSI_VID_HBP_TIME);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_HLINE_TIME     0x%x\n", MIPI_DSI_VID_HLINE_TIME);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_VSA_LINES      0x%x\n", MIPI_DSI_VID_VSA_LINES);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_VBP_LINES      0x%x\n", MIPI_DSI_VID_VBP_LINES);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_VFP_LINES      0x%x\n", MIPI_DSI_VID_VFP_LINES);
  DebugPrint(DSI_DEBUG_LEVEL, "VID_VACTIVE_LINES  0x%x\n", MIPI_DSI_VID_VACTIVE_LINES);
  DebugPrint(DSI_DEBUG_LEVEL, "CMD_MODE_CFG       0x%x\n", MIPI_DSI_CMD_MODE_CFG);
  DebugPrint(DSI_DEBUG_LEVEL, "GEN_HDR            0x%x\n", MIPI_DSI_GEN_HDR);
  DebugPrint(DSI_DEBUG_LEVEL, "GEN_PLD_DATA       0x%x\n", MIPI_DSI_GEN_PLD_DATA);
  DebugPrint(DSI_DEBUG_LEVEL, "CMD_PKT_STATUS     0x%x\n", MIPI_DSI_CMD_PKT_STATUS);
  DebugPrint(DSI_DEBUG_LEVEL, "TO_CNT_CFG         0x%x\n", MIPI_DSI_TO_CNT_CFG);
  DebugPrint(DSI_DEBUG_LEVEL, "HS_RD_TO_CNT       0x%x\n", MIPI_DSI_HS_RD_TO_CNT);
  DebugPrint(DSI_DEBUG_LEVEL, "LP_RD_TO_CNT       0x%x\n", MIPI_DSI_LP_RD_TO_CNT);
  DebugPrint(DSI_DEBUG_LEVEL, "HS_WR_TO_CNT       0x%x\n", MIPI_DSI_HS_WR_TO_CNT);
  DebugPrint(DSI_DEBUG_LEVEL, "LP_WR_TO_CNT       0x%x\n", MIPI_DSI_LP_WR_TO_CNT);
  DebugPrint(DSI_DEBUG_LEVEL, "BTA_TO_CNT         0x%x\n", MIPI_DSI_BTA_TO_CNT);
  DebugPrint(DSI_DEBUG_LEVEL, "LPCLK_CTRL         0x%x\n", MIPI_DSI_LPCLK_CTRL);
  DebugPrint(DSI_DEBUG_LEVEL, "PHY_TMR_LPCLK_CFG  0x%x\n", MIPI_DSI_PHY_TMR_LPCLK_CFG);
  DebugPrint(DSI_DEBUG_LEVEL, "PHY_TMR_CFG        0x%x\n", MIPI_DSI_PHY_TMR_CFG);
  DebugPrint(DSI_DEBUG_LEVEL, "PHY_RSTZ           0x%x\n", MIPI_DSI_PHY_RSTZ);
  DebugPrint(DSI_DEBUG_LEVEL, "PHY_IF_CFG         0x%x\n", MIPI_DSI_PHY_IF_CFG);
  DebugPrint(DSI_DEBUG_LEVEL, "PHY_ULPS_CTRL      0x%x\n", MIPI_DSI_PHY_ULPS_CTRL);
  DebugPrint(DSI_DEBUG_LEVEL, "PHY_STATUS         0x%x\n", MIPI_DSI_PHY_STATUS);
  DebugPrint(DSI_DEBUG_LEVEL, "PHY_TST_CTRL0      0x%x\n", MIPI_DSI_PHY_TST_CTRL0);
  DebugPrint(DSI_DEBUG_LEVEL, "PHY_TST_CTRL1      0x%x\n", MIPI_DSI_PHY_TST_CTRL1);
  DebugPrint(DSI_DEBUG_LEVEL, "INT_ST0            0x%x\n", MIPI_DSI_INT_ST0);
  DebugPrint(DSI_DEBUG_LEVEL, "INT_ST1            0x%x\n", MIPI_DSI_INT_ST1);
  DebugPrint(DSI_DEBUG_LEVEL, "INT_MSK0           0x%x\n", MIPI_DSI_INT_MSK0);
  DebugPrint(DSI_DEBUG_LEVEL, "INT_MSK1           0x%x\n", MIPI_DSI_INT_MSK1);
  DebugPrint(DSI_DEBUG_LEVEL, "PHY_TMR_RD_CFG     0x%x\n", MIPI_DSI_PHY_TMR_RD_CFG);
}


