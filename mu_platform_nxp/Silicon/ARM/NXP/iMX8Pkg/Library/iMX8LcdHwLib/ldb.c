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
#include "ldb.h"
#include "clock.h"

/**
  Reset LDB block.
**/
VOID
LdbReset (
  VOID   
  )
{
    uint32_t tmp;

#if defined(CPU_IMX8MP)
    /* De-assert LCDIF2 resets within MEDIA_MIX block.*/
    tmp = MEDIA_BLK_CTRL_SFT_RSTN;
    tmp |= (MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_BUS_BLK_CLK_RESETN_MASK |
            MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_LCDIF2_PIXEL_CLK_RESETN_MASK |
            MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_LCDIF2_APB_CLK_RESETN_MASK |
            MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_LCDIF2_AXI_CLK_RESETN_MASK);
    MEDIA_BLK_CTRL_SFT_RSTN = tmp;
    /* Enable LCDIF2 clocks within MEDIA_MIX block. */
    tmp = MEDIA_BLK_CTRL_CLK_EN;
    tmp |= (MEDIA_BLK_CTRL_CLK_EN_SFT_EN_BUS_BLK_CLK_MASK |
            MEDIA_BLK_CTRL_CLK_EN_SFT_EN_LCDIF2_PIXEL_CLK_MASK |
            MEDIA_BLK_CTRL_CLK_EN_SFT_EN_LCDIF2_APB_CLK_MASK |
            MEDIA_BLK_CTRL_CLK_EN_SFT_EN_LCDIF2_AXI_CLK_MASK);
    MEDIA_BLK_CTRL_CLK_EN = tmp;
#endif

    /* power off ldb phy */
    MEDIA_BLK_CTRL_LDB_CTRL = 0;
    tmp = MEDIA_BLK_CTRL_LVDS_CTRL;
    tmp &= (MEDIA_BLK_CTRL_LVDS_CTRL_CH0_EN_MASK | MEDIA_BLK_CTRL_LVDS_CTRL_CH1_EN_MASK);
    MEDIA_BLK_CTRL_LVDS_CTRL = tmp;
}

/**
  Confiure LDB block.
**/
EFI_STATUS
LdbConfig (
  IMX_DISPLAY_TIMING* Timing,
  imxDisplayInterfaceType DispInterface 
  )
{
  uint32_t phy = 0;
  uint32_t ctrl = 0;
  uint32_t bg = 0;
  bool dual = (DispInterface == imxLvds0dual) ? TRUE : FALSE;

  if (Timing == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  LdbDisplayClockConfig(Timing, dual);

  /* LDB PHY initialization */
  phy = MEDIA_BLK_CTRL_LVDS_CTRL;
  /* Store whether BG was already enabled */
  bg = phy & MEDIA_BLK_CTRL_LVDS_CTRL_VBG_EN_MASK;
  phy &= (MEDIA_BLK_CTRL_LVDS_CTRL_CC_ADJ_MASK | MEDIA_BLK_CTRL_LVDS_CTRL_PRE_EMPH_ADJ_MASK);
  phy |= (MEDIA_BLK_CTRL_LVDS_CTRL_VBG_EN_MASK |
          MEDIA_BLK_CTRL_LVDS_CTRL_CC_ADJ(0x2) | 
          MEDIA_BLK_CTRL_LVDS_CTRL_PRE_EMPH_EN_MASK |
          MEDIA_BLK_CTRL_LVDS_CTRL_PRE_EMPH_ADJ(0x3));
  MEDIA_BLK_CTRL_LVDS_CTRL = phy;

  if (bg == 0) {
    /* if we just switched on BG, wait to stabilize bandgap */
    MicroSecondDelay(15U);
  }

  phy = MEDIA_BLK_CTRL_LVDS_CTRL;
  if ((DispInterface == imxLvds0) || dual) {
    phy |= MEDIA_BLK_CTRL_LVDS_CTRL_CH0_EN_MASK;
  }
  if ((DispInterface == imxLvds1) || dual) {
    phy |= MEDIA_BLK_CTRL_LVDS_CTRL_CH1_EN_MASK;
  }
  MEDIA_BLK_CTRL_LVDS_CTRL = phy;
  /* Wait 5us to settle the phy */
  MicroSecondDelay(5U);

  /* LDB initialization */
  /* Leave default negative polarity, SPWG mapping, set 24bit data width, LDB data always from source 0 */
  switch (DispInterface) {
    case imxLvds0dual:
      ctrl |= (MEDIA_BLK_CTRL_LDB_CTRL_CH0_DATA_WIDTH_MASK | 
               MEDIA_BLK_CTRL_LDB_CTRL_CH1_DATA_WIDTH_MASK | 
               MEDIA_BLK_CTRL_LDB_CTRL_SPLIT_MODE_MASK |
               MEDIA_BLK_CTRL_LDB_CTRL_CH0_ENABLE_MASK |
               MEDIA_BLK_CTRL_LDB_CTRL_CH1_ENABLE_MASK);
      break;
    case imxLvds1:
      ctrl |= (MEDIA_BLK_CTRL_LDB_CTRL_CH1_DATA_WIDTH_MASK | 
               MEDIA_BLK_CTRL_LDB_CTRL_CH1_DI_SELECT_MASK |
               MEDIA_BLK_CTRL_LDB_CTRL_CH1_ENABLE_MASK);
      break;
    case imxLvds0:
      ctrl |= (MEDIA_BLK_CTRL_LDB_CTRL_CH0_DATA_WIDTH_MASK | 
               MEDIA_BLK_CTRL_LDB_CTRL_CH0_ENABLE_MASK);
      break;
    default:
      DEBUG ((DEBUG_ERROR, "Unsupporte display interface: %d\n", DispInterface));
      return EFI_INVALID_PARAMETER;
  }
  MEDIA_BLK_CTRL_LDB_CTRL = ctrl;

  return EFI_SUCCESS;
}

/**
  Dump LDB registers.
**/
VOID
LdbDump (
  VOID
  )
{
  DEBUG ((DEBUG_INFO, "------------------------LDB------------------------\n"));
  DEBUG ((DEBUG_INFO, "MEDIA_BLK_CTRL_LDB_CTRL        = 0x%08X\n", MEDIA_BLK_CTRL_LDB_CTRL));
  DEBUG ((DEBUG_INFO, "MEDIA_BLK_CTRL_LVDS_CTRL[phy]  = 0x%08X\n", MEDIA_BLK_CTRL_LVDS_CTRL));
}

