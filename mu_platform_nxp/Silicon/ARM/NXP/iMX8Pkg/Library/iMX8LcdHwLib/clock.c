/*
* Copyright 2022 NXP
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

#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include "clock.h"

/* Set to 1 to include registers dump into log */
#define CLOCK_DUMP_CCM_REGS  0
/* Level of debug messages. Error reports are not included,
   they have fixed level DEBUG_ERROR */
#define CLOCK_DEBUG_LEVEL DEBUG_INFO

/* Video PLL rates and parameters for required pixel clock derived from 24 MHz input clock */
/* Fvco = Fin * ((m * 65536) + k) / p / 65536 */
/* Simplify: if k = 0, Fvco = Fin * m / p */
/* rate = Fout = Fvco / 2^s */
/* Fvco = <1600 - 3200 MHz>, Fout <= 650 MHz, Fin = 24 MHz */
/* ref_rate = reference clock for mipi-dsi phy pll, PCLK = pixel clock */
/* ref_rate - must be produced by integer divider from pll rate. */
/* table is used for MIPI_DSI clock settings */
static struct videoPllRateTable videoPllTab24m_mipi_dsi[] = {
/*       PCLK      rate (must be descending order), m, p, s, k, ref rate */
  PLL_RATE(65000000U, 650000000U, 325, 3, 2, 0, 26000000U), /* 65 MHz pixel clock */
  PLL_RATE(108000000U,648000000U, 216, 2, 2, 0, 27000000U), /* 108 MHz pixel clock  */
  PLL_RATE(40500000U, 648000000U, 216, 2, 2, 0, 27000000U), /* 40.5 MHz pixel clock  */
  PLL_RATE(40000000U, 640000000U, 320, 3, 2, 0, 25600000U), /* 40 MHz pixel clock */
  PLL_RATE(31500000U, 630000000U, 210, 2, 2, 0, 25200000U), /* 31.5 MHz pixel clock */
  PLL_RATE(25200000U, 630000000U, 210, 2, 2, 0, 25200000U), /* 25.2 MHz pixel clock */
  PLL_RATE(101000000U,606000000U, 202, 2, 2, 0, 25250000U), /* 101 MHz pixel clock */
  PLL_RATE(148500000U,594000000U, 198, 2, 2, 0, 27000000U), /* 148.5 MHz pixel clock */
  PLL_RATE(74250000U, 594000000U, 198, 2, 2, 0, 27000000U), /* 74.25 MHz pixel clock */
  PLL_RATE(27000000U, 594000000U, 198, 2, 2, 0, 27000000U), /* 27 MHz pixel clock */
  PLL_RATE(49500000U, 594000000U, 198, 2, 2, 0, 27000000U), /* 49.5 MHz pixel clock */
  PLL_RATE(28800000U, 576000000U, 192, 2, 2, 0, 24000000U), /* 28.8 MHz pixel clock */
  PLL_RATE(28300000U, 566000000U, 283, 3, 2, 0, 28300000U), /* 28.3 MHz pixel clock */
  PLL_RATE(135000000U,540000000U, 180, 2, 2, 0, 27000000U), /* 135 MHz pixel clock */
  PLL_RATE(67500000U, 540000000U, 180, 2, 2, 0, 27000000U), /* 67.5 MHz pixel clock */
  PLL_RATE(106500000U,426000000U, 284, 2, 3, 0, 28400000U), /* 106.50 MHz pixel clock */
  PLL_RATE(121750000U,365250000U, 487, 2, 4, 0, 24350000U), /* 121.75 MHz pixel clock */
  PLL_RATE(78800000U, 394000000U, 394, 3, 3, 0, 19700000U), /* 78.8 MHz pixel clock */
  PLL_RATE(146250000U,292500000U, 390, 2, 4, 0, 22500000U), /* 146.25 MHz pixel clock */
  PLL_RATE(0U, 0U, 0U, 0U, 0U, 0U, 0U), /* dummy entry to indicate end of the table */
};

/* table is used for lvds. The same equations as above are valid. PCLK = pixel clock*/
/* MEDIA_DISP2_CLK_ROOT = PCLK */
/* MEDIA_LDB_CLK_ROOT = 7 * PCLK in single mode */
/* MEDIA_LDB_CLK_ROOT = 3.5 * PCLK in dual mode */
/* PLL rate: Fout = PCLK * 7 */
/* single mode: div(MEDIA_LDB_CLK_ROOT) = 1, div(MEDIA_DISP2_CLK_ROOT) = 7 */
/* dual mode: div(MEDIA_LDB_CLK_ROOT) = 2, div(MEDIA_DISP2_CLK_ROOT) = 7 */
/* ref rate is not used */
static struct videoPllRateTable videoPllTab24m_lvds[] = {
/* PCLK, PLL rate (must be descending order), m, p, s, k, ref rate */
  PLL_RATE(156685000U,1096795000U, 457, 5, 1, 0, 0), /* 156.685 MHz pixel clock - dual lvds panel */
  PLL_RATE(148500000U,1039500000U, 693, 8, 1, 0, 0), /* 148.5 MHz pixel clock */
  PLL_RATE(135000000U,945000000U, 315, 4, 1, 0, 0), /* 135 MHz pixel clock */
  PLL_RATE(108000000U,756000000U, 252, 2, 2, 0, 0), /* 108 MHz pixel clock  */
  PLL_RATE(106500000U,745500000U, 497, 4, 2, 0, 0), /* 106.50 MHz pixel clock */
  PLL_RATE(101000000U,707000000U, 707, 6, 2, 0, 0), /* 101 MHz pixel clock */
  PLL_RATE(74250000U, 519750000U, 693, 8, 2, 0, 0), /* 74.25 MHz pixel clock */
  PLL_RATE(67500000U, 472500000U, 315, 4, 2, 0, 0), /* 67.5 MHz pixel clock */
  PLL_RATE(65000000U, 455000000U, 455, 6, 2, 0, 0), /* 65 MHz pixel clock */
  PLL_RATE(49500000U, 346500000U, 231, 2, 3, 0, 0), /* 49.5 MHz pixel clock */
  PLL_RATE(40500000U, 283500000U, 189, 2, 3, 0, 0), /* 40.5 MHz pixel clock  */
  PLL_RATE(40000000U, 280000000U, 280, 3, 3, 0, 0), /* 40 MHz pixel clock */
  PLL_RATE(31500000U, 220500000U, 147, 2, 3, 0, 0), /* 31.5 MHz pixel clock */
  PLL_RATE(28800000U, 201600000U, 336, 5, 3, 0, 0), /* 28.8 MHz pixel clock */
  PLL_RATE(27000000U, 189000000U, 126, 1, 4, 0, 0), /* 27 MHz pixel clock */
  PLL_RATE(25200000U, 176400000U, 588, 5, 4, 0, 0), /* 25.2 MHz pixel clock */
  PLL_RATE(0U, 0U, 0U, 0U, 0U, 0U, 0U), /* dummy entry to indicate end of the table */
};

/**
  Returns mipi dsi phy ref clock for given pixelclock.
  If no suitable found, returns 0 Hz

  @param Timing         Input Timing of the display.
  @param phyRefClockHz  Output dsi phy reference clock rate.
  @retval EFI_SUCCESS            success.
  @retval EFI_DEVICE_ERROR       error.
**/
EFI_STATUS
CcmGetDsiPhyRefClk (
  IMX_DISPLAY_TIMING* Timing,
  uint32_t *phyRefClockHz
  )
{
  if ((Timing == NULL) || (phyRefClockHz == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  *phyRefClockHz = 0;
  for (uint32_t idx = 0U; videoPllTab24m_mipi_dsi[idx].pixClockRate != 0U; idx++) {
    if (videoPllTab24m_mipi_dsi[idx].pixClockRate == Timing->PixelClock) {
      *phyRefClockHz = videoPllTab24m_mipi_dsi[idx].dsiPhyRefRate;
    }
  }
  return EFI_SUCCESS;
}

/**
  Calculation of PRE and POST dividers for required clock slice.

  @param inputClock Input clock rate.
  @param rateRequired Output clock rate.
  @param prediv Calculated PRE dividider.
  @param postdiv Calculated POST dividider.
  @param rateReal Really calculated output clock rate.
  @retval EFI_SUCCESS            success.
  @retval EFI_DEVICE_ERROR       error.
**/
static EFI_STATUS CcmClockSliceConfig (
  uint32_t inputClock,
  uint32_t rateRequired,
  uint32_t *prediv,
  uint32_t *postdiv,
  uint32_t *rateReal
  )
{
  EFI_STATUS status = EFI_SUCCESS;
  uint32_t diff, diffTmp, rateTmp, div1, div2;

  diff = inputClock;
  *prediv = 1;
  *postdiv = 1;
  *rateReal = 0;
  /* Slice output clock = Clock source / prediv / postdiv */
  for (div1 = 1; div1 <= CCM_TARGET_ROOT_PRE_MAX_DIV; div1++) {
    for (div2 = 1; div2 <= CCM_TARGET_ROOT_POST_MAX_DIV; div2++) {
      rateTmp = (inputClock / div1) / div2;
      if (rateRequired > rateTmp) {
        diffTmp = rateRequired - rateTmp;
      } else {
        diffTmp = rateTmp - rateRequired;
      }
      if (diffTmp < diff) {
        *prediv = div1;
        *postdiv = div2;
        diff = diffTmp;
        *rateReal = rateTmp;
      }
    }
  }

  if (*rateReal == 0) {
    DEBUG ((DEBUG_ERROR, "Unable to achieve required output clock.\n"));
    status = EFI_DEVICE_ERROR;
  }
  return status;
}

/* Method sets Video PLL in CCM module
  @param  Timing                 Display timing.
  @param  pllRateOut             Output video PLL rate.
  @param  videoPllRateTable      VideoPll table that should be used for PLL parameters - see top of this file.
  @retval EFI_SUCCESS            success.
  @retval EFI_DEVICE_ERROR       error.
*/
static EFI_STATUS CcmSetVideoPll (
  IMX_DISPLAY_TIMING *Timing,
  uint32_t *pllRateOut,
  struct videoPllRateTable* videoPllTab24m
  )
{
  int    idx = 0, best_idx = 0;
  int    loop = 0;
  uint32_t pllRate = 0, best_diff, diff;
  
  if ((Timing == NULL) || (pllRateOut == 0)) {
    return EFI_INVALID_PARAMETER;
  }
  
  *pllRateOut = 0;
  /* For required pixel clock, look for suitable Video PLL rate */
  best_diff = videoPllTab24m[0].pllRate;
  while (videoPllTab24m[idx].pllRate != 0) {
    if (videoPllTab24m[idx].pixClockRate > Timing->PixelClock) {
      diff = videoPllTab24m[idx].pixClockRate - Timing->PixelClock;
    } else {
      diff = Timing->PixelClock - videoPllTab24m[idx].pixClockRate;
    }
    if (diff < best_diff) {
      best_diff = diff;
      best_idx = idx;
      pllRate = videoPllTab24m[idx].pllRate;
    }
    idx++;
  }

  if(pllRate == 0) {
    DEBUG ((DEBUG_ERROR, "Video Pll settings not found for pixel clock %d Hz\n", Timing->PixelClock));
    return EFI_DEVICE_ERROR;
  } else {
    if(videoPllTab24m[best_idx].pixClockRate == Timing->PixelClock) {
      DEBUG ((DEBUG_ERROR, "Video Pll settings found. Pll rate = %d Hz Pclk = %d Hz\n",
          pllRate, Timing->PixelClock));
    } else {
      DEBUG ((DEBUG_ERROR, "Video Pll settings not found for pixel clock %d Hz. Using Pll rate = %d Hz Pclk = %d Hz instead",
          Timing->PixelClock, videoPllTab24m[best_idx].pllRate, videoPllTab24m[best_idx].pixClockRate));
    }
  }

  /* Video PLL configuration */
  /* Bypass PLL */
  CCM_ANALOG_VIDEO_PLL1_GEN_CTRL |= CCM_ANALOG_VIDEO_PLL1_GEN_CTRL_PLL_BYPASS_MASK;
  /* Reset PLL */
  CCM_ANALOG_VIDEO_PLL1_GEN_CTRL &= ~CCM_ANALOG_VIDEO_PLL1_GEN_CTRL_PLL_RST_MASK;

  CCM_ANALOG_VIDEO_PLL1_FDIV_CTL0 = CCM_ANALOG_VIDEO_PLL1_FDIV_CTL0_PLL_MAIN_DIV(videoPllTab24m[best_idx].mdiv) |
                    CCM_ANALOG_VIDEO_PLL1_FDIV_CTL0_PLL_PRE_DIV(videoPllTab24m[best_idx].pdiv) |
                    CCM_ANALOG_VIDEO_PLL1_FDIV_CTL0_PLL_POST_DIV(videoPllTab24m[best_idx].sdiv);

  CCM_ANALOG_VIDEO_PLL1_FDIV_CTL1 = CCM_ANALOG_VIDEO_PLL1_FDIV_CTL1_PLL_DSM(videoPllTab24m[best_idx].kdiv);

  /* Deasert reset */
  CCM_ANALOG_VIDEO_PLL1_GEN_CTRL |= CCM_ANALOG_VIDEO_PLL1_GEN_CTRL_PLL_RST_MASK;

  /* Wait for PLL to lock */
  while((CCM_ANALOG_VIDEO_PLL1_GEN_CTRL & CCM_ANALOG_VIDEO_PLL1_GEN_CTRL_PLL_LOCK_MASK) == 0) {
    if(loop > CCM_PLL_LOCK_TIMEOUT) {
      DEBUG ((DEBUG_ERROR, "Video Pll not locked.\n"));
      return EFI_DEVICE_ERROR;
    }
    MicroSecondDelay(1);
    loop++;
  }

  /* Disable PLL bypass */
  CCM_ANALOG_VIDEO_PLL1_GEN_CTRL &= ~CCM_ANALOG_VIDEO_PLL1_GEN_CTRL_PLL_BYPASS_MASK;

  *pllRateOut = pllRate;
  DEBUG ((CLOCK_DEBUG_LEVEL, "Video Pll ready.\n"));
  return EFI_SUCCESS;
}

/* Method sets CCM module for LVDS display.
   Sets Video PLL, MEDIA_DISP2, MEDIA_LDB, MEDIA_AXI, MEDIA_APB clock.
  @param  Timing                 Display timing.
  @param  Dual                   TRUE = lvds dual mode, FALSE = LVDS single mode.
  @retval EFI_SUCCESS            success.
  @retval EFI_DEVICE_ERROR       error.
*/
EFI_STATUS LdbDisplayClockConfig (
  IMX_DISPLAY_TIMING *Timing,
  bool Dual
  )
{
  uint32_t pllRate = 0;
#ifdef LDB_CLK_ROOT_PRESENT
  uint32_t prediv;
#endif

  if (Timing == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  
  if (Dual && (Timing->PixelClock > 160000000)) {
    DEBUG ((DEBUG_ERROR, "LDB pixel clock exceeds 160MHz in dual mode - %d\n", Timing->PixelClock));
    return EFI_DEVICE_ERROR;
  }
  if (!Dual && (Timing->PixelClock > 80000000)) {
    DEBUG ((DEBUG_ERROR, "LDB pixel clock exceeds 80MHz in single mode - %d\n", Timing->PixelClock));
    return EFI_DEVICE_ERROR;
  }

  CCM_CLK_GATING_REG = 0x00;

  if ((CcmSetVideoPll(Timing, &pllRate, videoPllTab24m_lvds) != EFI_SUCCESS) || (pllRate == 0)) {
    DEBUG ((DEBUG_ERROR, "Video Pll for required pixel clock not found\n"));
    return EFI_DEVICE_ERROR;
  }

#ifdef LDB_CLK_ROOT_PRESENT
  /* Enable LCDIF pixel clock */
  /* Set MUX to VIDEO_PLL_CLK, divider hard fixed to 7 */
  CCM_TARGET_ROOT_MEDIA_DISP2 = (CCM_TARGET_ROOT_MUX(1) |
                   CCM_TARGET_ROOT_PRE_PODF(7 - 1) |
                   CCM_TARGET_ROOT_POST_PODF(1 - 1) |
                   CCM_TARGET_ROOT_ENABLE_MASK);
  DEBUG ((CLOCK_DEBUG_LEVEL, "PIXEL2 clock ready %dHz.PreDiv=7, PostDiv=1\n", (pllRate / 7)));

  /* Set LDB slice */
  /* Set MUX to VIDEO_PLL_CLK, divider hard fixed to 1 in single mode, 2 in dual mode */
  prediv = (Dual) ? 2 : 1;

  CCM_TARGET_ROOT_MEDIA_LDB = (CCM_TARGET_ROOT_MUX(7) | /* Set MUX to VIDEO_PLL_CLK */
                   CCM_TARGET_ROOT_PRE_PODF(prediv - 1) |
                   CCM_TARGET_ROOT_POST_PODF(1 - 1) |
                   CCM_TARGET_ROOT_ENABLE_MASK);
  DEBUG ((CLOCK_DEBUG_LEVEL, "LDB clock ready %dHz.PreDiv=%d, PostDiv=1\n", (pllRate / prediv), prediv));
#endif

  /* Set DISPLAY_AXI_CLK_ROOT clock root to SYSTEM_PLL2_CLK => 1000MHz */
  CCM_TARGET_ROOT_DISPLAY_AXI = (CCM_TARGET_ROOT_MUX(1)    /* Set MUX to SYSTEM_PLL2_CLK */
                 | CCM_TARGET_ROOT_PRE_PODF(1)   /* Set PRE_PODF post divider to /2 to reach 500MHz */
                 | CCM_TARGET_ROOT_ENABLE_MASK); /* Enable clock root */

  /* Set DISPLAY_APB_CLK_ROOT clock root to SYSTEM_PLL1_CLK => 800MHz */
  CCM_TARGET_ROOT_DISPLAY_APB = (CCM_TARGET_ROOT_MUX(2)    /* Set MUX to SYSTEM_PLL1_CLK */
                 | CCM_TARGET_ROOT_PRE_PODF(3)   /* Set PRE_PODF post divider to /4 to reach 200MHz */
                 | CCM_TARGET_ROOT_ENABLE_MASK); /* Enable clock root */

#if (CLOCK_DUMP_CCM_REGS != 0)
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_ANALOG_VIDEO_PLL1_GEN_CTRL   = 0x%08X\n", CCM_ANALOG_VIDEO_PLL1_GEN_CTRL));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_ANALOG_VIDEO_PLL1_FDIV_CTL0  = 0x%08X\n", CCM_ANALOG_VIDEO_PLL1_FDIV_CTL0));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_ANALOG_VIDEO_PLL1_FDIV_CTL1  = 0x%08X\n", CCM_ANALOG_VIDEO_PLL1_FDIV_CTL1));
#ifdef LDB_CLK_ROOT_PRESENT
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_LCDIF2_PIXEL    = 0x%08X\n", CCM_TARGET_ROOT_MEDIA_DISP2));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_MEDIA_LDB      = 0x%08X\n", CCM_TARGET_ROOT_MEDIA_LDB));
#endif
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_DISPLAY_AXI    = 0x%08X\n", CCM_TARGET_ROOT_DISPLAY_AXI));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_DISPLAY_APB    = 0x%08X\n", CCM_TARGET_ROOT_DISPLAY_APB));
#endif

  /* Enable CCGR 93 common for both DISPLAY_AXI_CLK_ROOT & DISPLAY_APB_CLK_ROOT clock roots */
  CCM_CLK_GATING_REG = 0x03;

  return EFI_SUCCESS;
}

/* Method sets CCM module for MIPI_DSI display.
   Sets Video PLL, LCDIF_PIXEL, MIPI_DSI_PHY_REF, MIPI_DSI_CORE, MEDIA_AXI, MEDIA_APB clock.
  @param  Timing                 Display timing.
  @param  Dual                   TRUE = lvds dual mode, FALSE = LVDS single mode.
  @retval EFI_SUCCESS            success.
  @retval EFI_DEVICE_ERROR       error.
*/
EFI_STATUS MipiDsiDisplayClockConfig (
  IMX_DISPLAY_TIMING *Timing
  )
{
  EFI_STATUS status = EFI_SUCCESS;
  uint32_t pllRate = 0;
  uint32_t phyRefRate = 0;
  uint32_t prediv, postdiv, rateReal;

  if (Timing == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  CCM_CLK_GATING_REG = 0x00;

  if ((CcmSetVideoPll(Timing, &pllRate, videoPllTab24m_mipi_dsi) != EFI_SUCCESS) || (pllRate == 0)) {
    DEBUG ((DEBUG_ERROR, "Video Pll for required pixel clock not found\n"));
    return EFI_DEVICE_ERROR;
  }

  /* Enable LCDIF pixel clock */
  if ((status = CcmClockSliceConfig(pllRate, Timing->PixelClock, &prediv, &postdiv, &rateReal)) != EFI_SUCCESS) {
    DEBUG ((DEBUG_ERROR, "Unable to calculate pixel clock.\n"));
    return EFI_DEVICE_ERROR;
  }

  CCM_TARGET_ROOT_LCDIF_PIXEL_REG = (CCM_TARGET_ROOT_MUX(1) | /* Set MUX to VIDEO_PLL_CLK */
                   CCM_TARGET_ROOT_PRE_PODF(prediv - 1) |
                   CCM_TARGET_ROOT_POST_PODF(postdiv - 1) |
                   CCM_TARGET_ROOT_ENABLE_MASK);
  DEBUG ((CLOCK_DEBUG_LEVEL, "PIXEL clock ready %dHz.PreDiv=%d, PostDiv=%d\n", rateReal, prediv, postdiv));

  if ((CcmGetDsiPhyRefClk(Timing, &phyRefRate) != EFI_SUCCESS) || (phyRefRate == 0)) {
    DEBUG ((DEBUG_ERROR, "Dsi phy ref clock for required pixel clock not found\n"));
    return EFI_DEVICE_ERROR;
  }
  /* Set DSI_PHY_REF slice */
  /* Note: linux driver uses CLK_24M source instead of VIDEO_PLL_CLK and divides down to 12MHz */
  if ((status = CcmClockSliceConfig(pllRate, phyRefRate, &prediv, &postdiv, &rateReal)) != EFI_SUCCESS) {
    DEBUG ((DEBUG_ERROR, "Unable to calculate DSI_PHY_REF clock.\n"));
    return EFI_DEVICE_ERROR;
  }

  CCM_TARGET_ROOT_MIPI_DSI_PHY_REF_REG = (CCM_TARGET_ROOT_MUX(7) | /* Set MUX to VIDEO_PLL_CLK */
                   CCM_TARGET_ROOT_PRE_PODF(prediv - 1) |
                   CCM_TARGET_ROOT_POST_PODF(postdiv - 1) |
                   CCM_TARGET_ROOT_ENABLE_MASK);
  DEBUG ((CLOCK_DEBUG_LEVEL, "DSI_PHY_REF clock ready %dHz.PreDiv=%d, PostDiv=%d\n", rateReal, prediv, postdiv));

#ifdef MIPI_DSI_CORE_CLK_ROOT_PRESENT
  /* Set MIPI_DSI_CORE_CLK_ROOT clock root to SYSTEM_PLL1_DIV3 => 266.6MHz */
  CCM_TARGET_ROOT_MIPI_DSI_CORE = (CCM_TARGET_ROOT_MUX(1)    /* Set MUX to SYSTEM_PLL1_DIV3 */
                   | CCM_TARGET_ROOT_ENABLE_MASK); /* Enable clock root */
#endif

  /* Set DISPLAY_AXI_CLK_ROOT clock root to SYSTEM_PLL2_CLK => 1000MHz */
  CCM_TARGET_ROOT_DISPLAY_AXI = (CCM_TARGET_ROOT_MUX(1)    /* Set MUX to SYSTEM_PLL2_CLK */
                 | CCM_TARGET_ROOT_PRE_PODF(1)   /* Set PRE_PODF post divider to /2 to reach 500MHz */
                 | CCM_TARGET_ROOT_ENABLE_MASK); /* Enable clock root */

  /* Set DISPLAY_APB_CLK_ROOT clock root to SYSTEM_PLL1_CLK => 800MHz */
  CCM_TARGET_ROOT_DISPLAY_APB = (CCM_TARGET_ROOT_MUX(2)    /* Set MUX to SYSTEM_PLL1_CLK */
                 | CCM_TARGET_ROOT_PRE_PODF(3)   /* Set PRE_PODF post divider to /4 to reach 200MHz */
                 | CCM_TARGET_ROOT_ENABLE_MASK); /* Enable clock root */

#if (CLOCK_DUMP_CCM_REGS != 0)
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_ANALOG_VIDEO_PLL1_GEN_CTRL   = 0x%08X\n", CCM_ANALOG_VIDEO_PLL1_GEN_CTRL));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_ANALOG_VIDEO_PLL1_FDIV_CTL0  = 0x%08X\n", CCM_ANALOG_VIDEO_PLL1_FDIV_CTL0));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_ANALOG_VIDEO_PLL1_FDIV_CTL1  = 0x%08X\n", CCM_ANALOG_VIDEO_PLL1_FDIV_CTL1));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_LCDIF_PIXEL      = 0x%08X\n", CCM_TARGET_ROOT_LCDIF_PIXEL_REG));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_MIPI_DSI_PHY_REF = 0x%08X\n", CCM_TARGET_ROOT_MIPI_DSI_PHY_REF_REG));
#ifdef MIPI_DSI_CORE_CLK_ROOT_PRESENT
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_MIPI_DSI_CORE    = 0x%08X\n", CCM_TARGET_ROOT_MIPI_DSI_CORE));
#endif
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_DISPLAY_AXI      = 0x%08X\n", CCM_TARGET_ROOT_DISPLAY_AXI));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_DISPLAY_APB      = 0x%08X\n", CCM_TARGET_ROOT_DISPLAY_APB));
#endif

  /* Enable CCGR 93 common for both DISPLAY_AXI_CLK_ROOT & DISPLAY_APB_CLK_ROOT clock roots */
  CCM_CLK_GATING_REG = 0x03;

  return EFI_SUCCESS;
}

/* Method sets CCM module for HDMI display (imx8mp).
   Sets HDMI_24M, HDMI_AXI, HDMI_APB, HDMI_REF_266 clock.
*/
VOID HdmiDisplayClockConfig(VOID)
{
#ifdef HDMI_PRESENT
  CCM_CCGR_HDMI = 0x00;

  /* Set HDMI_AXI_CLK_ROOT clock root to SYS_PLL2_DIV2 => 1000MHz/2 = 500 MHz */
  CCM_TARGET_ROOT_HDMI_AXI = (CCM_TARGET_ROOT_MUX(7)    /* Set MUX to SYS_PLL2_DIV2 */
                 | CCM_TARGET_ROOT_PRE_PODF(0)   /* Set PRE_PODF post divider to /1 to reach 500MHz */
                 | CCM_TARGET_ROOT_ENABLE_MASK); /* Enable clock root */

  /* Set HDMI_APB_CLK_ROOT clock root to SYS_PLL1_DIV6 => 800MHz/6 = 133,3 MHz */
  CCM_TARGET_ROOT_HDMI_APB = (CCM_TARGET_ROOT_MUX(7)    /* Set MUX to SYS_PLL1_DIV6 */
                 | CCM_TARGET_ROOT_PRE_PODF(0)   /* Set PRE_PODF post divider to /1 to reach 133,3MHz */
                 | CCM_TARGET_ROOT_ENABLE_MASK); /* Enable clock root */


  /* Set HDMI_REF_266_CLK_ROOT to 24MHz, divider 1, that is default, just enable */
  CCM_TARGET_ROOT_HDMI_REF_266M = (CCM_TARGET_ROOT_MUX(0) |
                   CCM_TARGET_ROOT_PRE_PODF(0) |
                   CCM_TARGET_ROOT_POST_PODF(0) |
                   CCM_TARGET_ROOT_ENABLE_MASK);

  /* Set HDMI_24M_CLK_ROOT to 24MHz, divider 1 */
  CCM_TARGET_ROOT_HDMI_24M = (CCM_TARGET_ROOT_MUX(0) | /* Set MUX to 24MHz clock */
                   CCM_TARGET_ROOT_PRE_PODF(0) | /* PRE_DIV = 1 */
                   CCM_TARGET_ROOT_POST_PODF(0) | /* POST_DIV = 1 */
                   CCM_TARGET_ROOT_ENABLE_MASK);

#if (CLOCK_DUMP_CCM_REGS != 0)
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_HDMI_24M    = 0x%08X\n", CCM_TARGET_ROOT_HDMI_24M));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCCM_TARGET_ROOT_HDMI_REF_266M      = 0x%08X\n", CCM_TARGET_ROOT_HDMI_REF_266M));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_HDMI_AXI    = 0x%08X\n", CCM_TARGET_ROOT_HDMI_AXI));
  DEBUG ((CLOCK_DEBUG_LEVEL, "CCM_TARGET_ROOT_HDMI_APB    = 0x%08X\n", CCM_TARGET_ROOT_HDMI_APB));
#endif

  /* Enable CCGR 95 common for HDMI clock roots */
  CCM_CCGR_HDMI = 0x03;
#endif /* HDMI_PRESENT */
}

