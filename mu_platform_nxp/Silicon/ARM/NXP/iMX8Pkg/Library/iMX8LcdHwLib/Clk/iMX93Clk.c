/*
* Copyright 2023 NXP
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the
*   names of its contributors may be used to endorse or promote products
*   derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <fcntl.h>

#include "iMX93Clk.h"
#include "iMX9xClkRegisters.h"
#include "MipiCommon.h"

#include <iMX8.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <Uefi/UefiBaseType.h>

#define FRAC_PLL_RATE(_Rate, _Mfi, _Mfn, _Mfd, _Rdiv, _Odiv)  \
  {                                                           \
    .Rate = (_Rate),                                          \
    .Rdiv = (_Rdiv),                                          \
    .Mfi = (_Mfi),                                            \
    .Odiv = (_Odiv),                                          \
    .Mfn = (_Mfn),                                            \
    .Mfd = (_Mfd),                                            \
  }

#define IMX_PLL_LOCK_TIMEOUT_US 200U

#define IMX_CLK_FREQ_1039_5M    1039500000U
#define IMX_CLK_FREQ_1000M      1000000000U
#define IMX_CLK_FREQ_800M       800000000U
#define IMX_CLK_FREQ_786_4M     786432000U
#define IMX_CLK_FREQ_750M       750000000U
#define IMX_CLK_FREQ_722_5M     722534400U
#define IMX_CLK_FREQ_625M       625000000U
#define IMX_CLK_FREQ_594M       594000000U
#define IMX_CLK_FREQ_519_7M     519750000U
#define IMX_CLK_FREQ_500M       500000000U
#define IMX_CLK_FREQ_400M       400000000U
#define IMX_CLK_FREQ_333_3M     333333333U
#define IMX_CLK_FREQ_266_6M     266666666U
#define IMX_CLK_FREQ_250M       250000000U
#define IMX_CLK_FREQ_200M       200000000U
#define IMX_CLK_FREQ_166_6M     166666666U
#define IMX_CLK_FREQ_160M       160000000U
#define IMX_CLK_FREQ_148_5M     148500000U
#define IMX_CLK_FREQ_133_3M     133333333U
#define IMX_CLK_FREQ_125M       125000000U
#define IMX_CLK_FREQ_100M       100000000U
#define IMX_CLK_FREQ_80M        80000000U
#define IMX_CLK_FREQ_50M        50000000U
#define IMX_CLK_FREQ_40M        40000000U
#define IMX_CLK_FREQ_27M        27000000U
#define IMX_CLK_FREQ_24M        24000000U
#define IMX_CLK_FREQ_12M        12000000U
#define IMX_CLK_FREQ_0M         0U

struct PLL_RATE_TABLE {
  UINT32 Rate;
  UINT32 Rdiv;
  UINT32 Mfi;
  UINT32 Odiv;
  UINT32 Mfn;
  UINT32 Mfd;
};

/**
  Video PLL rates and parameters for 24 MHz input clock

  Fvco = Fref * (MFI + MFN / MFD)
  Fout = Fvco / (rdiv * odiv)

  NOTE: Fvco in range 2.5GHz - 5GHz

  Rate;
  Mfi;  range: 0-511
  Mfn;  range: 0-1073741823
  Mfd;  range: 0-1073741823
  Rdiv; range: 0-7   (!!!!!rdiv 0 = 1)
  Odiv; range: 0-255 (!!!!!odiv 0 = 2 1 = 3)
**/
STATIC CONST struct PLL_RATE_TABLE VideoPllTab24mMipiDsi[] = {
  /* Rate - Mfi - Mfn - Mfd - Rdiv - Odiv */
  FRAC_PLL_RATE(154000000U, 154, 0, 1, 1,  24),  /* 154 MHz pixel clock */
  FRAC_PLL_RATE(148500000U, 167, 0, 1, 1,  27),  /* 148.5 Cant't be set accurately, using 148.4444 MHz */
  FRAC_PLL_RATE(135000000U, 135, 0, 1, 1,  24),  /* 135 MHz pixel clock */
  FRAC_PLL_RATE(108000000U, 126, 0, 1, 1,  28),  /* 108 MHz pixel clock */
  FRAC_PLL_RATE(106500000U, 142, 0, 1, 1,  32),  /* 106.5 MHz pixel clock */
  FRAC_PLL_RATE(101000000U, 202, 0, 1, 1,  48),  /* 101 MHz pixel clock */
  FRAC_PLL_RATE(80000000U,  140, 0, 1, 1,  42),  /* 80 MHz pixel clock */
  FRAC_PLL_RATE(78750000U,  105, 0, 1, 1,  32),  /* 78.75 MHz pixel clock */
  FRAC_PLL_RATE(74250000U,  167, 0, 1, 1,  54),  /* 74.25 MHz can't be set accurately, using 74.2222 MHz */
  FRAC_PLL_RATE(67500000U,  135, 0, 1, 1,  48),  /* 67.5 MHz pixel clock */
  FRAC_PLL_RATE(65000000U,  130, 0, 1, 1,  48),  /* 65 MHz pixel clock */
  FRAC_PLL_RATE(40500000U,  138, 0, 1, 1,  80),  /* 40.5 MHz pixel clock */
  FRAC_PLL_RATE(40000000U,  120, 0, 1, 1,  72),  /* 40 MHz pixel clock */
  FRAC_PLL_RATE(30000000U,  115, 0, 1, 1,  92),  /* 30 MHz pixel clock */
  FRAC_PLL_RATE(28800000U,  120, 0, 1, 1, 100),  /* 28.8 MHz pixel clock */
  FRAC_PLL_RATE(0U, 0, 0, 0, 0, 0), /* dummy entry to indicate end of the table */
};

/* Table is used for lvds. The same equations as above are valid */
STATIC CONST struct PLL_RATE_TABLE VideoPllTab24mLvds[] = {
  /* Rate - Mfi - Mfn - Mfd - Rdiv - Odiv */
  FRAC_PLL_RATE(560000000U,  140,  0,   1, 1,  6),  /* 80 MHz pixel clock */
  FRAC_PLL_RATE(519750000U,  173, 25, 100, 1,  8),  /* 74.25 MHz pixel clock */
  FRAC_PLL_RATE(506800000U,  126,  7,  10, 1,  6),  /* 72.4 MHz pixel clock */
  FRAC_PLL_RATE(472500000U,  157,  5,  10, 1,  8),  /* 67.5 MHz pixel clock */
  FRAC_PLL_RATE(455000000U,  113, 75,  10, 1,  6),  /* 65 MHz pixel clock */
  FRAC_PLL_RATE(283500000U,  189,  0,   1, 1, 16),  /* 40.5 MHz pixel clock  */
  FRAC_PLL_RATE(280000000U,  140,  0,   1, 1, 12),  /* 40 MHz pixel clock */
  FRAC_PLL_RATE(201600000U,  126,  0,   1, 1, 15),  /* 28.8 MHz pixel clock */
  FRAC_PLL_RATE(0U, 0U, 0U, 0U, 0U, 0U), /* dummy entry to indicate end of the table */
};

/**
  Sets the PLL to given rate

  @param  Rate           Requested rate.
  @param  DispInterface  Display interface.

  @retval  Rate that was set.
**/
STATIC UINT32 SetRateVideoPll(UINT32 Rate, imxDisplayInterfaceType DispInterface)
{
  CONST struct PLL_RATE_TABLE *RateTbl =
    DispInterface == imxMipiDsi || DispInterface == imxRgb ?
    VideoPllTab24mMipiDsi : VideoPllTab24mLvds;

  while (RateTbl->Rate && Rate < RateTbl->Rate) {
    RateTbl++;
  }

  if (RateTbl->Rate == 0) {
    DebugPrint(DEBUG_ERROR, "PLL params for rate-%d not found.\n", Rate);
    return 0;
  }
  DebugPrint(DEBUG_INFO, "PLL params for rate-%d: Mfi=%d Mfn=%d Mfd=%d Rdiv=%d Odiv=%d\n",
    Rate, RateTbl->Mfi, RateTbl->Mfn, RateTbl->Mfd, RateTbl->Rdiv, RateTbl->Odiv);

  VIDEOPLL_CTRL_SET = PLL_CTRL_CLKMUX_BYPASS_MASK;

  VIDEOPLL_CTRL_CLR = PLL_CTRL_POWERUP_MASK | PLL_CTRL_CLKMUX_EN_MASK;

  VIDEOPLL_DIV = PLL_DIV_ODIV(RateTbl->Odiv) | PLL_DIV_RDIV(RateTbl->Rdiv) |
                 PLL_DIV_MFI(RateTbl->Mfi);

  VIDEOPLL_SPREAD_SPECTRUM_CLR = PLL_SPREAD_SPECTRUM_ENABLE_MASK;

  VIDEOPLL_NUMERATOR = PLL_NUMERATOR_MFN(RateTbl->Mfn);

  VIDEOPLL_DENOMINATOR = PLL_DENOMINATOR_MFD(RateTbl->Mfd);

  NanoSecondDelay(5000);

  VIDEOPLL_CTRL_SET = PLL_CTRL_POWERUP_MASK;

  UINT32 Loop = 0;
  while ((VIDEOPLL_PLL_STATUS & PLL_PLL_STATUS_PLL_LOCK_MASK) == 0) {
    MicroSecondDelay(10U);
    Loop++;
    if (Loop > 20U) {
      DEBUG((DEBUG_ERROR, "Wait on PLL lock failed.\n"));
      return 0;
    }
  }

  UINT32 PllStatMfn = (VIDEOPLL_PLL_STATUS & PLL_PLL_STATUS_ANA_MFN_MASK) >>
                      PLL_PLL_STATUS_ANA_MFN_SHIFT;
  if (PllStatMfn != RateTbl->Mfn) {
    DEBUG((DEBUG_ERROR,
      "MFN does not match! read: %d set: %d\n", PllStatMfn, RateTbl->Mfn));
    return 0;
  }

  VIDEOPLL_CTRL_SET = PLL_CTRL_CLKMUX_EN_MASK;

  VIDEOPLL_CTRL_CLR = PLL_CTRL_CLKMUX_BYPASS_MASK;

  return RateTbl->Rate;
}

/**
  Enable/disable clock gate.

  @param  Clk     Clock Id.
  @param  Enable  True = enable the clock.

  @retval  EFI_SUCCESS            Clock enabled.
  @retval  EFI_INVALID_PARAMETER  Invalid clock id.
**/
STATIC EFI_STATUS LpcgEnable(INTN Clk, BOOLEAN Enable)
{
  if (Clk < 0 || Clk > 94) {
    return EFI_INVALID_PARAMETER;
  }

  volatile UINT32 Lpcg = CCM_LPCG_DIRECT_REG(CCM_CTRL_BASE_PTR, Clk);
  if (Enable) {
    Lpcg |= CCM_LPCG_DIRECT_ON_MASK;
  } else {
    Lpcg &= ~CCM_LPCG_DIRECT_ON_MASK;
  }
  CCM_LPCG_DIRECT_REG(CCM_CTRL_BASE_PTR, Clk) = Lpcg;

  return EFI_SUCCESS;
}

/**
  Enable/disable clock root.

  @param  Clk     Clock Id.
  @param  Enable  True = enable the clock.

  @retval  EFI_SUCCESS            Clock enabled.
  @retval  EFI_INVALID_PARAMETER  Invalid clock id.
  @retval  EFI_DEVICE_ERROR       Wait on clock gate register update failed.
**/
STATIC EFI_STATUS ClkRootEnable(INTN Clk, BOOLEAN Enable)
{
  if (Clk < 0 || Clk > 94) {
    return EFI_INVALID_PARAMETER;
  }

  if (Enable) {
    CCM_CLOCK_ROOT_CLOCK_ROOT_CONTROL_RW_REG(CCM_CTRL_BASE_PTR, Clk) &=
      ~CCM_CLOCK_ROOT_OFF_MASK;
  } else {
    CCM_CLOCK_ROOT_CLOCK_ROOT_CONTROL_RW_REG(CCM_CTRL_BASE_PTR, Clk) |=
      CCM_CLOCK_ROOT_OFF_MASK;
  }

  if (Enable) {
    UINT32 Loop = 0;
    while (CCM_CLOCK_ROOT_STATUS0_REG(CCM_CTRL_BASE_PTR,Clk) &
      CCM_CLOCK_ROOT_STATUS0_CHANGING_MASK) {
      MicroSecondDelay(10U);
      Loop++;
      if (Loop > 20U) {
        DEBUG((DEBUG_ERROR,
          "Wait on clock gate register update failed.\n"));
        return EFI_DEVICE_ERROR;
      }
    }
  }

  return EFI_SUCCESS;
}

/**
  Set clock root parent.

  @param  Clk     Clock Id.
  @param  Parent  Parent Id.

  @retval  EFI_SUCCESS            Parent was configured for the clock.
  @retval  EFI_INVALID_PARAMETER  Invalid clock id.
**/
STATIC EFI_STATUS ClkRootSetParent(INTN Clk, INTN Parent)
{
  if (Parent < 0 || Parent > 3) {
    return EFI_INVALID_PARAMETER;
  }

  if (Clk < 0 || Clk > 94) {
    return EFI_INVALID_PARAMETER;
  }

  volatile UINT32 Val =
    CCM_CLOCK_ROOT_CLOCK_ROOT_CONTROL_RW_REG(CCM_CTRL_BASE_PTR, Clk);
  Val &= ~CCM_CLOCK_ROOT_MUX_MASK;
  Val = CCM_CLOCK_ROOT_MUX(Parent);

  CCM_CLOCK_ROOT_CLOCK_ROOT_CONTROL_RW_REG(CCM_CTRL_BASE_PTR, Clk) = Val;

  return EFI_SUCCESS;
}

/**
  Set the clk root to a given rate

  @param  Clk         Clock Id.
  @param  Rate        Requested rate.
  @param  ParentRate  Parent rate.

  @retval  Rate that was set.
**/
STATIC UINT32 ClkRootSetRate(INTN Clk, UINT32 Rate, UINT32 ParentRate)
{
  if (Clk < 0 || Clk > 94) {
    return 0;
  }

  UINT32 RateTmp, DiffTmp;
  UINT32 Diff = ParentRate;
  UINT32 Div = 1;
  UINT32 RateReal = 0;

  for (UINT32 Cdiv = 1; Cdiv <= IMX_CCM_CLOCK_ROOT_CONTROL_DIV_MAX; Cdiv++) {
    RateTmp = ParentRate / Cdiv;
    if (Rate > RateTmp) {
      DiffTmp = Rate - RateTmp;
    } else {
      DiffTmp = RateTmp - Rate;
    }
    if (DiffTmp < Diff) {
      Div = Cdiv;
      Diff = DiffTmp;
      RateReal = RateTmp;
    }
  }

  if (RateReal) {
    volatile UINT32 Val =
      CCM_CLOCK_ROOT_CLOCK_ROOT_CONTROL_RW_REG(CCM_CTRL_BASE_PTR, Clk);
    Val &= ~IMX_CCM_CLOCK_ROOT_CONTROL_DIV_MASK;
    Val |= IMX_CCM_CLOCK_ROOT_CONTROL_DIV(Div - 1);
    CCM_CLOCK_ROOT_CLOCK_ROOT_CONTROL_RW_REG(CCM_CTRL_BASE_PTR, Clk) = Val;
  } else {
    DEBUG((DEBUG_ERROR,
      "ClkRootSetRate(%d) requested rate %d couldn't be set\n", Clk, Rate));
  }

  DEBUG((DEBUG_INFO,
    "ClkRootSetRate(%d) Div=%d, Rate=%d\n", Clk, Div, RateReal));

  return RateReal;
}

/**
  Set port clock for MIPI.

  @param  Pclk          Requested pixel clock.
  @param  PllRate       Pll rate.
  @param  DsiPhyRateHz  Dsi phy rate in hz.

  @retval  EFI_SUCCESS  MIPI clock configured.
**/
STATIC EFI_STATUS SetPortClockMipi(UINT32 Pclk, UINT32 PllRate,
  UINT32 DsiPhyRateHz)
{
  ClkRootEnable(IMX_CCM_TARGET_MIPI_PHY_CFG, FALSE);
  ClkRootEnable(IMX_CCM_TARGET_MEDIA_DISP_PIX, FALSE);
#ifdef DEBUG_CKO02
  ClkRootEnable(IMX_CCM_TARGET_CCM_CKO2, FALSE);
#endif

  /* Set Video PLL*/
  UINT32 ParentRate = SetRateVideoPll(PllRate, imxMipiDsi);

  /* Set LCDIF_PIXEL slice */
  UINT32 CfgPclk = ClkRootSetRate(IMX_CCM_TARGET_MEDIA_DISP_PIX,
    Pclk, ParentRate);
  if (CfgPclk != Pclk) {
    DEBUG((DEBUG_INFO,
      "Configured pixel clock is different the requested (%d vs %d).\n",
       CfgPclk, Pclk));
  }
  ClkRootEnable(IMX_CCM_TARGET_MEDIA_DISP_PIX, TRUE);

#ifdef DEBUG_CKO02
  ClkRootSetRate(IMX_CCM_TARGET_CCM_CKO2, Pclk, ParentRate);
  ClkRootEnable(IMX_CCM_TARGET_CCM_CKO2, TRUE);
#endif

  /* Set DSI_PHY_REF slice */
  UINT32 PhyRate = ClkRootSetRate(IMX_CCM_TARGET_MIPI_PHY_CFG,
    DsiPhyRateHz, IMX_CLK_FREQ_24M);
  if (PhyRate == 0) {
    DEBUG((DEBUG_INFO, "Can't set requested phy clock %d.\n", PhyRate));
  }
  ClkRootEnable(IMX_CCM_TARGET_MIPI_PHY_CFG, TRUE);

  LpcgEnable(IMX_CCM_LPCG_MIPI_DSI, TRUE);
  LpcgEnable(IMX_CCM_LPCG_LCDIF, TRUE);

  return EFI_SUCCESS;
}

/**
  Set port clock for LVDS.

  @param  Pclk     Requested pixel clock.
  @param  PllRate  Pll rate.

  @retval  EFI_SUCCESS  LVDS clock configured.
**/
STATIC EFI_STATUS SetPortClockLvds(UINT32 Pclk, UINT32 PllRate)
{
  ClkRootEnable(IMX_CCM_TARGET_MEDIA_LDB, FALSE);
  ClkRootEnable(IMX_CCM_TARGET_MEDIA_DISP_PIX, FALSE);

  /* Set Video PLL*/
  UINT32 ParentRate = SetRateVideoPll(PllRate, imxLvds0);

  /* Set LCDIF_PIXEL clock */
  UINT32 CfgPclk = ClkRootSetRate(IMX_CCM_TARGET_MEDIA_DISP_PIX,
    Pclk, ParentRate);
  if (CfgPclk != Pclk) {
    DEBUG((DEBUG_INFO,
      "Configured pixel clock is different the requested (%d vs %d).\n",
       CfgPclk, Pclk));
  }

  /* Set LDB serial clock */
  CfgPclk = ClkRootSetRate(IMX_CCM_TARGET_MEDIA_LDB,
    Pclk * 7, ParentRate);
  if (CfgPclk != Pclk * 7) {
    DEBUG((DEBUG_INFO,
      "Configured serial  clock is different the requested (%d vs %d).\n",
       CfgPclk, Pclk * 7));
  }

  ClkRootEnable(IMX_CCM_TARGET_MEDIA_DISP_PIX, TRUE);
  ClkRootEnable(IMX_CCM_TARGET_MEDIA_LDB, TRUE);

  LpcgEnable(IMX_CCM_LPCG_LVDS, TRUE);
  LpcgEnable(IMX_CCM_LPCG_LCDIF, TRUE);

  return EFI_SUCCESS;
}

/**
  Set port clock for RGB.

  @param  Pclk     Requested pixel clock.
  @param  PllRate  Pll rate.

  @retval  EFI_SUCCESS  RGB clock configured.
**/
STATIC EFI_STATUS SetPortClockRgb(UINT32 Pclk, UINT32 PllRate)
{
  /* placeholder for future implementation */
  return EFI_SUCCESS;
}

/**
  Re-configure the clock according to the requested timing.

  @param  Timing         Requested timing configuration.
  @param  DispInterface  Display interface.

  @retval  EFI_SUCCESS            Clock configured.
  @retval  EFI_INVALID_PARAMETER  Invalid display interface.
**/
EFI_STATUS iMX9xDispClkConfigure(IMX_DISPLAY_TIMING* Timing,
  imxDisplayInterfaceType DispInterface)
{
  ASSERT(Timing);

  LpcgEnable(IMX_CCM_LPCG_MIPI_DSI, FALSE);
  LpcgEnable(IMX_CCM_LPCG_LCDIF, FALSE);

  /* set pixel clock parent to Video Pll */
  ClkRootEnable(IMX_CCM_TARGET_MEDIA_DISP_PIX, FALSE);
  ClkRootSetParent(IMX_CCM_TARGET_MEDIA_DISP_PIX, 2);

  /* set MIPI PHY parent to Osc 24MHz */
  ClkRootEnable(IMX_CCM_TARGET_MIPI_PHY_CFG, FALSE);
  ClkRootSetParent(IMX_CCM_TARGET_MIPI_PHY_CFG, 0);

  /* set LDB parent to Video Pll */
  ClkRootEnable(IMX_CCM_TARGET_MEDIA_LDB, FALSE);
  ClkRootSetParent(IMX_CCM_TARGET_MEDIA_LDB, 2);

  /* Try to find requested rate in the fixed videopll_tab_24m mode */
  CONST struct PLL_RATE_TABLE *RateTable;
  if (DispInterface == imxMipiDsi || DispInterface == imxRgb) {
    RateTable = VideoPllTab24mMipiDsi;
  } else {
    RateTable = VideoPllTab24mLvds;
  }

  UINT32 PllRate = 0;
  UINT32 Pclk = 1000 * GetPixelClockKHz(Timing);

  while (RateTable->Rate) {
    if (RateTable->Rate % Pclk == 0) {
      PllRate = RateTable->Rate;
      break;
    }
    RateTable++;
  }

  /* If no match found, select default rate */
  if (PllRate == 0) {
    PllRate = IMX_CLK_FREQ_1039_5M;
    DEBUG((DEBUG_INFO,
      "Video PLL frequency not found for pclk-%d. Use default value-%d\n",
       Pclk, IMX_CLK_FREQ_1039_5M));
  }

  switch (DispInterface) {
    case imxMipiDsi:
      SetPortClockMipi(Pclk, PllRate, IMX_CLK_FREQ_24M);
      break;
    case imxLvds0:
      SetPortClockLvds(Pclk, PllRate);
      break;
    case imxRgb:
      SetPortClockRgb(Pclk, PllRate);
      break;
    default:
      DEBUG((DEBUG_ERROR, "Unsupported disp interface 0x%x\n",
       DispInterface));
     return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

