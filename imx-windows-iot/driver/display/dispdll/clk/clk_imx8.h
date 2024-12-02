/*
 * Copyright 2022,2024 NXP
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

#ifndef IMX_CLK_IMX8_H
#define IMX_CLK_IMX8_H

#define IMX_CLK_FREQ_1039_5M    1039500000U
#define IMX_CLK_FREQ_1000M      1000000000U
#define IMX_CLK_FREQ_800M       800000000U
#define IMX_CLK_FREQ_786_4M     786432000U
#define IMX_CLK_FREQ_750M       750000000U
#define IMX_CLK_FREQ_722_5M     722534400U
#define IMX_CLK_FREQ_650M       650000000U
#define IMX_CLK_FREQ_600M       600000000U
#define IMX_CLK_FREQ_594M       594000000U
#define IMX_CLK_FREQ_519_7M     519750000U
#define IMX_CLK_FREQ_500M       500000000U
#define IMX_CLK_FREQ_432M       432000000U
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
#define IMX_CLK_FREQ_25M        25000000U
#define IMX_CLK_FREQ_24M        24000000U
#define IMX_CLK_FREQ_12M        12000000U
#define IMX_CLK_FREQ_32K        32000U
#define IMX_CLK_FREQ_0K         0U

/**
 * Enumeration of clock type
 */
enum imx_clk_type {
    CLK_DUMMY,    /* Dummy entry - start node of the clock tree */
    CLK_FIXED,    /* Clock has fixed frequency that cannot be changed */
    CLK_FRAC_PLL, /* Fractional PLL type for example VIDEO PLL */
    CLK_SYS_PLL,  /* System PLL type */
    CLK_GATE,     /* Clock gate item - represents CCGR register setting */
    CLK_SLICE,    /* Represents clock composite clock slice block */
    CLK_CUSTOM,   /* Custom clock node */
    CLK_LPCG_SCU, /* SCU LPCG clock*/
    CLK_SCU,      /* SCU clock */
};

#endif
