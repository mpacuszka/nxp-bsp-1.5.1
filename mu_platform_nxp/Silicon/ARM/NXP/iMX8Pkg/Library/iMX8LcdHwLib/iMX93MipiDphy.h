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

#ifndef PHY_IMX93_MIPI_DPHY_H
#define PHY_IMX93_MIPI_DPHY_H

#include <iMX8.h>
#include <iMXDisplay.h>

/* Software Reset Register */
#define IMX_MEDIAMIX_RESET                   0x0
#define RESET_LCDIF_APB_EN_MASK              (1 << 4)
#define RESET_LCDIF_AXI_EN_MASK              (1 << 5)
#define RESET_LCDIF_PIX_EN_MASK              (1 << 6)
#define RESET_DSI_APB_EN_MASK                (1 << 11)
#define RESET_DSI_REF_CLK_EN_MASK            (1 << 12)

/* Clock Enable Register */
#define IMX_MEDIAMIX_CLK                     0x4
#define CLK_LCDIF_APB_EN_MASK                (1 << 4)
#define CLK_LCDIF_AXI_EN_MASK                (1 << 5)
#define CLK_LCDIF_PIX_EN_MASK                (1 << 6)
#define CLK_DSI_APB_EN_MASK                  (1 << 11)
#define CLK_DSI_REF_CLK_EN_MASK              (1 << 12)

/* QOS and cache of LCDIF */
#define IMX_MEDIAMIX_LCDIF                   0xC

/* LDB Control Register */
#define IMX_MEDIAMIX_LDB_CTRL                0x20
#define LDB_CH0_ENABLE_MASK                  (1 << 0)
#define LDB_CH0_DATA_WIDTH_MASK              (1 << 5)
#define LDB_CH0_BIT_MAPPING_MASK             (1 << 6)
#define LDB_CH0_VSYNC_POLARITY_MASK          (1 << 9)

/* LVDS-PHY Control Register */
#define IMX_MEDIAMIX_LVDS                    0x24
#define LVDS_CH0_EN_MASK                     (1 << 0)
#define LVDS_LVDS_EN_MASK                    (1 << 1)
#define LVDS_BG_EN_MASK                      (1 << 2)
#define LVDS_PRE_EMPH_EN_MASK                (1 << 4)
#define LVDS_PRE_EMPH_ADJ_MASK               (3 << 5)
#define LVDS_PRE_EMPH_ADJ(v)                 (((v) << 5) & LVDS_PRE_EMPH_ADJ_MASK)
#define LVDS_CM_ADJ_MASK                     (3 << 8)
#define LVDS_CM_ADJ(v)                       (((v) << 8) & LVDS_CM_ADJ_MASK)
#define LVDS_CC_ADJ_MASK                     (3 << 11)
#define LVDS_CC_ADJ(v)                       (((v) << 11) & LVDS_CC_ADJ_MASK)

/* DSI Control Register */
#define IMX_MEDIAMIX_DSI                     0x4C
#define DSI_CFGCLKFREQRANGE_MASK             (0x3F << 0)
#define DSI_CFGCLKFREQRANGE(v)               (((v) << 0) & DSI_CFGCLKFREQRANGE_MASK)
#define DSI_CLKSEL_MASK                      (0x3 << 6)
#define DSI_CLKSEL(v)                        (((v) << 6) & DSI_CLKSEL_MASK)
  #define DSI_CLKSEL_STOP                      0
  #define DSI_CLKSEL_GEN                       1
  #define DSI_CLKSEL_EXT                       2
#define DSI_HSFREQRANGE_MASK                 (0x7F << 8)
#define DSI_HSFREQRANGE(v)                   (((v) << 8) & DSI_HSFREQRANGE_MASK)
#define DSI_UPDATE_PLL_MASK                  (1 << 17)
#define DSI_SHADOW_CLR_MASK                  (1 << 18)
#define DSI_CLK_EXT_MASK                     (1 << 19)

/* DSI write register 0 */
#define IMX_MEDIAMIX_DSI_W0                  0x50
#define DSI_W0_M_MASK                        (0x3FF << 0)
#define DSI_W0_M(v)                          ((((v) - 2) << 0) & DSI_W0_M_MASK)
#define DSI_W0_N_MASK                        (0xF << 10)
#define DSI_W0_N(v)                          ((((v) - 1) << 10) & DSI_W0_N_MASK)
#define DSI_W0_VCO_CTRL_MASK                 (0x3F << 14)
#define DSI_W0_VCO_CTRL(v)                   (((v) << 14) & DSI_W0_VCO_CTRL_MASK)
#define DSI_W0_PROP_CTRL_MASK                (0x3F << 20)
#define DSI_W0_PROP_CTRL(v)                  (((v) << 20) & DSI_W0_PROP_CTRL_MASK)
#define DSI_W0_INT_CTRL_MASK                 (0x3F << 26)
#define DSI_W0_INT_CTRL(v)                   (((v) << 26) & DSI_W0_INT_CTRL_MASK)

/* DSI write register 1 */
#define IMX_MEDIAMIX_DSI_W1                  0x54
#define DSI_W1_GMP_CTRL_MASK                 (0x3 << 0)
#define DSI_W1_GMP_CTRL(v)                   (((v) << 0) & DSI_W1_GMP_CTRL_MASK)
#define DSI_W1_CPBIAS_CTRL_MASK              (0x7F << 2)
#define DSI_W1_CPBIAS_CTRL(v)                (((v) << 2) & DSI_W1_CPBIAS_CTRL_MASK)
#define DSI_W1_PLL_SHADOW_CTRL               (1 << 9)

/* DSI write register 1 */
#define IMX_MEDIAMIX_DSI_R0                  0x58
/* DSI write register 1 */
#define IMX_MEDIAMIX_DSI_R1                  0x5C

/* #define IMX_MEDIAMIX_DEBUG 1 */

EFI_STATUS DsiPhyConfigure(IMX_DISPLAY_TIMING* Timing);
VOID DsiPhyStop();

#endif /* PHY_IMX93_MIPI_DPHY_H */

