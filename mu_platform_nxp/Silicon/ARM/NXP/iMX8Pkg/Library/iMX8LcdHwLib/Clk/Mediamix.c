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

#include "Mediamix.h"
#include <iMX8.h>
#include <Library/DebugLib.h>

VOID MediamixDsiPwrOn()
{
  UINT32 Reg = BLK_CTRL_MEDIAMIX_RESET;
  Reg |= MEDIAMIX_BLK_CTRL_RESET_lcdif_apb_en_MASK |
    MEDIAMIX_BLK_CTRL_RESET_lcdif_axi_en_MASK |
    MEDIAMIX_BLK_CTRL_RESET_lcdif_pix_en_MASK |
    MEDIAMIX_BLK_CTRL_RESET_dsi_apb_en_MASK |
    MEDIAMIX_BLK_CTRL_RESET_ref_clk_en_MASK;
  BLK_CTRL_MEDIAMIX_RESET = Reg;

  Reg = BLK_CTRL_MEDIAMIX_CLK;
  Reg &= ~(MEDIAMIX_BLK_CTRL_CLK_lcdif_apb_en_MASK |
    MEDIAMIX_BLK_CTRL_CLK_lcdif_axi_en_MASK |
    MEDIAMIX_BLK_CTRL_CLK_lcdif_pix_en_MASK |
    MEDIAMIX_BLK_CTRL_CLK_dsi_apb_en_MASK |
    MEDIAMIX_BLK_CTRL_CLK_ref_clk_en_MASK);
  BLK_CTRL_MEDIAMIX_CLK = Reg;
}

VOID MediamixLvdsPwrOn()
{
  UINT32 Reg = BLK_CTRL_MEDIAMIX_RESET;
  Reg |= MEDIAMIX_BLK_CTRL_RESET_lcdif_apb_en_MASK |
    MEDIAMIX_BLK_CTRL_RESET_lcdif_axi_en_MASK |
    MEDIAMIX_BLK_CTRL_RESET_lcdif_pix_en_MASK;
  BLK_CTRL_MEDIAMIX_RESET = Reg;

  Reg = BLK_CTRL_MEDIAMIX_CLK;
  Reg &= ~(MEDIAMIX_BLK_CTRL_CLK_lcdif_apb_en_MASK |
    MEDIAMIX_BLK_CTRL_CLK_lcdif_axi_en_MASK |
    MEDIAMIX_BLK_CTRL_CLK_lcdif_pix_en_MASK);
  BLK_CTRL_MEDIAMIX_CLK = Reg;
}

VOID MediamixDumpReg()
{
  DebugPrint(DEBUG_INFO, "IMX_MEDIAMIX_RESET: offset=0x%x val=0x%x\n",
    IMX_MEDIAMIX_RESET, BLK_CTRL_MEDIAMIX_RESET);
  DebugPrint(DEBUG_INFO, "IMX_MEDIAMIX_CLK: offset=0x%x val=0x%x\n",
    IMX_MEDIAMIX_CLK, BLK_CTRL_MEDIAMIX_CLK);
  DebugPrint(DEBUG_INFO, "IMX_MEDIAMIX_LCDIF: offset=0x%x val=0x%x\n",
    IMX_MEDIAMIX_LCDIF, BLK_CTRL_MEDIAMIX_LCDIF);
  DebugPrint(DEBUG_INFO, "IMX-MEDIAMIX_LDB_CTRL: offset=0x%x val=0x%x\n",
    IMX_MEDIAMIX_LDB_CTRL,BLK_CTRL_MEDIAMIX_LDB_CTRL);
  DebugPrint(DEBUG_INFO, "IMX_MEDIAMIX_LVDS: offset=0x%x val=0x%x\n",
    IMX_MEDIAMIX_LVDS, BLK_CTRL_MEDIAMIX_LVDS);
  DebugPrint(DEBUG_INFO, "IMX_MEDIAMIX_DSI: offset=0x%x val=0x%x\n",
    IMX_MEDIAMIX_DSI, BLK_CTRL_MEDIAMIX_DSI);
  DebugPrint(DEBUG_INFO, "IMX_MEDIAMIX_DSI_W0: offset=0x%x val=0x%x\n",
    IMX_MEDIAMIX_DSI_W0, BLK_CTRL_MEDIAMIX_DSI_W0);
  DebugPrint(DEBUG_INFO, "IMX_MEDIAMIX_DSI_W1: offset=0x%x val=0x%x\n",
    IMX_MEDIAMIX_DSI_W1, BLK_CTRL_MEDIAMIX_DSI_W1);
  DebugPrint(DEBUG_INFO, "IMX_MEDIAMIX_DSI_R0: offset=0x%x val=0x%x\n",
    IMX_MEDIAMIX_DSI_R0, BLK_CTRL_MEDIAMIX_DSI_R0);
  DebugPrint(DEBUG_INFO, "IMX_MEDIAMIX_DSI_R1: offset=0x%x val=0x%x\n",
    IMX_MEDIAMIX_DSI_R1, BLK_CTRL_MEDIAMIX_DSI_R1);
}

