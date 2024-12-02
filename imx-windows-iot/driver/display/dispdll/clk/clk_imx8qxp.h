/*
 * Copyright 2022-2024 NXP
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

#ifndef IMX_CLK_IMX8QXP_H
#define IMX_CLK_IMX8QXP_H

#include "clk_imx8q.h"

/* Clock tree items - START */
/* fixed clocks */
#define IMX8QXP_CLK_DUMMY                            0
#define IMX8QXP_DC0_AXI_EXT_CLK                      1
#define IMX8QXP_DC0_AXI_INT_CLK                      2
#define IMX8QXP_DC0_CFG_CLK                          3
/* lpcg */
#define IMX8QXP_DC0_DISP0_LPCG_CLK                   4
#define IMX8QXP_DC0_DISP1_LPCG_CLK                   5
#define IMX8QXP_DC0_PRG0_RTRAM_CLK                   6
#define IMX8QXP_DC0_PRG0_APB_CLK                     7
#define IMX8QXP_DC0_PRG1_RTRAM_CLK                   8
#define IMX8QXP_DC0_PRG1_APB_CLK                     9
#define IMX8QXP_DC0_PRG2_RTRAM_CLK                   10
#define IMX8QXP_DC0_PRG2_APB_CLK                     11
#define IMX8QXP_DC0_PRG3_RTRAM_CLK                   12
#define IMX8QXP_DC0_PRG3_APB_CLK                     13
#define IMX8QXP_DC0_PRG4_RTRAM_CLK                   14
#define IMX8QXP_DC0_PRG4_APB_CLK                     15
#define IMX8QXP_DC0_PRG5_RTRAM_CLK                   16
#define IMX8QXP_DC0_PRG5_APB_CLK                     17
#define IMX8QXP_DC0_PRG6_RTRAM_CLK                   18
#define IMX8QXP_DC0_PRG6_APB_CLK                     19
#define IMX8QXP_DC0_PRG7_RTRAM_CLK                   20
#define IMX8QXP_DC0_PRG7_APB_CLK                     21
#define IMX8QXP_DC0_PRG8_RTRAM_CLK                   22
#define IMX8QXP_DC0_PRG8_APB_CLK                     23
#define IMX8QXP_DC0_DPR0_APB_CLK                     24
#define IMX8QXP_DC0_DPR0_B_CLK                       25
#define IMX8QXP_DC0_DPR1_APB_CLK                     26
#define IMX8QXP_DC0_DPR1_B_CLK                       27
#define IMX8QXP_DC0_RTRAM0_CLK                       28
#define IMX8QXP_DC0_RTRAM1_CLK                       29
/* DC clocks */
#define IMX8QXP_DC0_DISP0_CLK                        30
#define IMX8QXP_DC0_DISP1_CLK                        31
#define IMX8QXP_DC0_PLL0_CLK                         32
#define IMX8QXP_DC0_PLL1_CLK                         33
#define IMX8QXP_DC0_BYPASS0_CLK                      34
#define IMX8QXP_DC0_BYPASS1_CLK                      35
/* MIPI-LVDS */
#define IMX8QXP_LVDS0_PIX_CLK                        36
#define IMX8QXP_LVDS0_BYPASS_CLK                     37
#define IMX8QXP_LVDS0_PHY_CLK                        38
#define IMX8QXP_LVDS1_PIX_CLK                        39
#define IMX8QXP_LVDS1_BYPASS_CLK                     40
#define IMX8QXP_LVDS1_PHY_CLK                        41

#define IMX8QXP_MIPI0_PIXEL_CLK                      42
#define IMX8QXP_MIPI0_BYPASS_CLK                     43
#define IMX8QXP_MIPI0_PHY_REF_CLK                    44
#define IMX8QXP_MIPI0_TX_ESC_CLK                     45
#define IMX8QXP_MIPI0_RX_ESC_CLK                     46
#define IMX8QXP_MIPI_PLL_DIV2_CLK                    47
#define IMX8QXP_MIPI1_PIXEL_CLK                      48
#define IMX8QXP_MIPI1_BYPASS_CLK                     49
#define IMX8QXP_MIPI1_PHY_REF_CLK                    50
#define IMX8QXP_MIPI1_TX_ESC_CLK                     51
#define IMX8QXP_MIPI1_RX_ESC_CLK                     52

#define IMX8QXP_DC_CLK_END                           53
/* Clock tree items - END */

/**
 * Device object which holds clock tree and memory handles of registers
 */
struct imx8qxp_clk_device {
    struct clk *clks[IMX8QXP_DC_CLK_END];
    char *lpcg_reg;
    unsigned int disp_num; /* number of active display interfaces */
};

struct imx8qxp_clk_device *clk_init_imx8qxp();
int clk_deinit_imx8qxp(struct imx8qxp_clk_device *dev);
void clk_dump_clock_tree_imx8qxp(const struct imx8qxp_clk_device *dev);
struct clk *clk_get_item_imx8qxp(int index);

#endif
