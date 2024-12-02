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

#ifndef IMX_CLK_IMX8M_H
#define IMX_CLK_IMX8M_H

#include <linux/io.h>
#include "clk_imx8.h"

/* #define IMX_CLK_DEBUG */

#define IMX_CLK_ROOT_MUX_MAX_PARENTS  8

/**
 * VIDEO PLL rates, PLL parameters, Reference clock rate
 */
struct imx_pll_rate_table {
    uint32_t pclk_rate;
    uint32_t pll_rate;
    uint32_t pdiv;
    uint32_t mdiv;
    uint32_t sdiv;
    uint32_t kdiv;
    uint32_t dsi_phy_ref_rate;
};

/**
 * Clk node parameters for CLK_FRAC_PLL clock type
 */
struct imx_clk_pll {
    volatile uint8_t __iomem *reg;
    const struct imx_pll_rate_table *pll_rates;
    int rate_index;
    unsigned phy_offset;
    uint32_t ref_rate_hz; /* PLL reference rate requested for given PLL and pixel clock */
};

/**
 * Clk node parameters for CLK_FIXED clock type
 */
struct imx_clk_fixed {
    uint32_t rate_hz;
};

/**
 * Clk node parameters for CLK_GATE clock type
 */
struct imx_clk_gate {
    volatile uint8_t __iomem *reg_base;
    unsigned shift;
    unsigned val;
    unsigned mask;
    unsigned *share_count;
    unsigned phy_base;
    unsigned phy_offset;
};

/**
 * Enumeration of clock sources for mux in the clock slice
 */
typedef enum {
    IMX_NO_PARENT = 0,
    IMX_24M_REF,
    IMX_25M_REF,
    IMX_EXT1,
    IMX_EXT2,
    IMX_EXT3,
    IMX_EXT4,
    IMX_SYS_PLL1,
    IMX_SYS_PLL1_DIV2,
    IMX_SYS_PLL1_DIV3,
    IMX_SYS_PLL1_DIV4,
    IMX_SYS_PLL1_DIV5,
    IMX_SYS_PLL1_DIV6,
    IMX_SYS_PLL1_DIV8,
    IMX_SYS_PLL1_DIV10,
    IMX_SYS_PLL1_DIV20,
    IMX_SYS_PLL2,
    IMX_SYS_PLL2_DIV2,
    IMX_SYS_PLL2_DIV3,
    IMX_SYS_PLL2_DIV4,
    IMX_SYS_PLL2_DIV5,
    IMX_SYS_PLL2_DIV6,
    IMX_SYS_PLL2_DIV8,
    IMX_SYS_PLL2_DIV10,
    IMX_SYS_PLL2_DIV20,
    IMX_SYS_PLL3,
    IMX_AUDIO_PLL1,
    IMX_AUDIO_PLL2,
    IMX_VIDEO_PLL,
    IMX_VIDEO_PLL1,
    IMX_VIDEO_PLL2
} imx_clk_source_t;

struct clk;
/**
 * Describes selected clock parent in the clock slice, i.e. one specific setting of the mux in the clock slice
 */
struct clk_root_mux{
    imx_clk_source_t parent_id;
    struct clk*  parent_clock;
};

/**
 * Clk node parameters for CLK_SLICE clock type
 */
struct imx_clk_slice {
    volatile uint8_t __iomem *base_reg;
    unsigned parent_index;
    unsigned prediv;
    unsigned postdiv;
    struct clk_root_mux* parents;
    unsigned *share_count;
    unsigned phy_offset;
};

typedef struct clk imx_clk_t;
/**
 * Auxiliary structure for initialization data of the parent clocks of clock slice 
 */
struct clk_root_mux_data {
    imx_clk_source_t parent_id;
    int parent_clock;
};

imx_clk_t * imx_clk_fixed(const char *name, imx_clk_t *parent, uint32_t rate);
imx_clk_t * imx_clk_pll(const char *name, imx_clk_t *parent, void __iomem *reg, unsigned offset,
                        const struct imx_pll_rate_table *rate_table);
imx_clk_t * imx_clk_gate(const char *name, imx_clk_t *parent, void __iomem *reg_base,
                         unsigned offset, unsigned base_addr,
                         unsigned shift, unsigned val, unsigned mask, unsigned *share_count);
imx_clk_t * imx_clk_slice(const char *name, imx_clk_t *parent, void __iomem *reg, unsigned offset,
                                            struct clk_root_mux *parents, unsigned *share_count);
void imx_clk_uninit(imx_clk_t *clk);

int imx_clk_set_parent(imx_clk_t *clk, imx_clk_t *parent);
imx_clk_t *imx_clk_get_parent(imx_clk_t *clk);
int imx_clk_enable(imx_clk_t *clk, bool enable, bool recursive);
unsigned long imx_clk_set_rate(imx_clk_t *clk, unsigned long rate);
unsigned long imx_clk_get_rate(imx_clk_t *clk);
unsigned long imx_clk_get_reference_rate(imx_clk_t *clk);
int imx_clk_dump(imx_clk_t *clk);

#endif
