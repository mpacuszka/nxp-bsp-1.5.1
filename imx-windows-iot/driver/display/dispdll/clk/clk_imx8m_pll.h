/*
 * Copyright 2022-2023 NXP
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

#ifndef IMX_CLK_IMX8M_PLL_H_
#define IMX_CLK_IMX8M_PLL_H_

#include "clk_imx8m.h"

#define PLL_RATE(pix_clock_val, rate_val, m_val, p_val, s_val, k_val, ref_rate_val) \
{ \
    .pclk_rate = (pix_clock_val), \
    .pll_rate = (rate_val), \
    .mdiv = (m_val), \
    .pdiv = (p_val), \
    .sdiv = (s_val), \
    .kdiv = (k_val), \
    .dsi_phy_ref_rate = (ref_rate_val), \
} 

/* Video PLL rates and parameters for 24 MHz input clock */
/* Fvco = Fin * ((m * 65536) + k) / p / 65536 */
/* Simplify: if k = 0, Fvco = Fin * m / p */
/* rate = Fout = Fvco / 2^s */
/* Fvco = <1600 - 3200 MHz>, Fout <= 650 MHz, Fin = 24 MHz */

/*--------------------------MIPI-DSI---------------------------------------------*/
/* ref_rate = reference clock for mipi-dsi phy pll <6-300MHz> */
/* ref_rate - must be produced by integer divider from pll rate. */
static const struct imx_pll_rate_table videopll_tab_24m_mipi[] = {
/* PCLK, PLL rate (must be descending order), m, p, s, k, ref rate */
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
    PLL_RATE(71000000U, 426000000U, 284, 2, 3, 0, 28400000U), /* 71.00 MHz pixel clock */
    PLL_RATE(121750000U,365250000U, 487, 2, 4, 0, 24350000U), /* 121.75 MHz pixel clock */
    PLL_RATE(78800000U, 394000000U, 394, 3, 3, 0, 19700000U), /* 78.8 MHz pixel clock */
    PLL_RATE(146250000U,292500000U, 390, 2, 4, 0, 22500000U), /* 146.25 MHz pixel clock */
    PLL_RATE(0U, 0U, 0U, 0U, 0U, 0U, 0U), /* dummy entry to indicate end of the table */ 
};

/*--------------------------LVDS---------------------------------------------*/
/* table is used for lvds. PCLK = pixel clock*/
/* MEDIA_DISP2_CLK_ROOT = PCLK */
/* MEDIA_LDB_CLK_ROOT = 7 * PCLK in single mode */
/* MEDIA_LDB_CLK_ROOT = 3.5 * PCLK in dual mode */
/* PLL rate: Fout = PCLK * 7 */
/* single mode: div(MEDIA_LDB_CLK_ROOT) = 1, div(MEDIA_DISP2_CLK_ROOT) = 7 */
/* dual mode: div(MEDIA_LDB_CLK_ROOT) = 2, div(MEDIA_DISP2_CLK_ROOT) = 7 */
/* ref rate is not used */
static const struct imx_pll_rate_table videopll_tab_24m_lvds[] = {
/* PCLK, PLL rate (must be descending order), m, p, s, k, ref rate */
    PLL_RATE(156680000U,1096760000U, 457, 5, 1, 0, 0), /* 156.68 MHz pixel clock - dual lvds panel */
    PLL_RATE(148500000U,1039500000U, 693, 8, 1, 0, 0), /* 148.5 MHz pixel clock */
    PLL_RATE(135000000U,945000000U, 315, 4, 1, 0, 0), /* 135 MHz pixel clock */
    PLL_RATE(108000000U,756000000U, 252, 2, 2, 0, 0), /* 108 MHz pixel clock  */
    PLL_RATE(106500000U,745500000U, 497, 4, 2, 0, 0), /* 106.50 MHz pixel clock */
    PLL_RATE(101000000U,707000000U, 707, 6, 2, 0, 0), /* 101 MHz pixel clock */
    PLL_RATE(85500000U, 598500000U, 798, 8, 2, 0, 0), /* 85.5 MHz pixel clock */
    PLL_RATE(74250000U, 1039500000U, 693, 8, 1, 0, 0), /* 74.25 MHz pixel clock */
    PLL_RATE(71000000U, 497000000U, 497, 6, 2, 0, 0), /* 71 MHz pixel clock */
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
#endif

