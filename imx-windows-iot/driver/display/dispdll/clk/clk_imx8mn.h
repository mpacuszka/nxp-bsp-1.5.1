/*
 * Copyright 2022 NXP
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

#ifndef IMX_CLK_IMX8MN_H
#define IMX_CLK_IMX8MN_H

#include "clk_imx8m.h"

/* Clock tree items - START */
/* fixed clocks */
#define IMX8MN_CLK_DUMMY                         0
#define IMX8MN_CLK_24M                           1
#define IMX8MN_CLK_32K                           2
#define IMX8MN_CLK_EXT1                          3
#define IMX8MN_CLK_EXT2                          4
#define IMX8MN_CLK_EXT3                          5
#define IMX8MN_CLK_EXT4                          6
#define IMX8MN_SYS_PLL1                          7
#define IMX8MN_SYS_PLL1_DIV2                     8
#define IMX8MN_SYS_PLL1_DIV3                     9
#define IMX8MN_SYS_PLL1_DIV4                     10
#define IMX8MN_SYS_PLL1_DIV5                     11
#define IMX8MN_SYS_PLL1_DIV6                     12
#define IMX8MN_SYS_PLL1_DIV8                     13
#define IMX8MN_SYS_PLL1_DIV10                    14
#define IMX8MN_SYS_PLL1_DIV20                    15
#define IMX8MN_SYS_PLL2                          16
#define IMX8MN_SYS_PLL2_DIV2                     17
#define IMX8MN_SYS_PLL2_DIV3                     18
#define IMX8MN_SYS_PLL2_DIV4                     19
#define IMX8MN_SYS_PLL2_DIV5                     20
#define IMX8MN_SYS_PLL2_DIV6                     21
#define IMX8MN_SYS_PLL2_DIV8                     22
#define IMX8MN_SYS_PLL2_DIV10                    23
#define IMX8MN_SYS_PLL2_DIV20                    24
#define IMX8MN_SYS_PLL3                          25
#define IMX8MN_AUDIO_PLL1                        26
#define IMX8MN_AUDIO_PLL2                        27
/* pll */
#define IMX8MN_VIDEO_PLL1                        28
/* clock slices */
#define IMX8MN_CLK_DISP_AXI                      29
#define IMX8MN_CLK_DISP_APB                      30
#define IMX8MN_CLK_LCDIF_PIXEL                   31
#define IMX8MN_CLK_DSI_CORE                      32
#define IMX8MN_CLK_DSI_PHY_REF                   33
/* lpcg */
#define IMX8MN_CLK_DISP_AXI_ROOT                 34
#define IMX8MN_CLK_DISP_APB_ROOT                 35

#define IMX8MN_DC_CLK_END                        36
/* Clock tree items - END */

/**
 * Device object which holds clock tree and memory handles of registers
 */
struct imx8mn_clk_device {
    imx_clk_t *clks[IMX8MN_DC_CLK_END];
    void __iomem *ccm_regptr;
    void __iomem *ccm_analog_regptr;
};

typedef struct imx8mn_clk_device imx8mn_clk_device_t;

imx8mn_clk_device_t *clk_init_imx8mn(void);
int clk_stop_imx8mn(imx8mn_clk_device_t *dev);
int clk_deinit_imx8mn(imx8mn_clk_device_t *dev);
void clk_dump_clock_tree_imx8mn(const imx8mn_clk_device_t *dev);
imx_clk_t *clk_get_item_imx8mn(int index);

#endif
