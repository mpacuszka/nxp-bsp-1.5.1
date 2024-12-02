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

#include "precomp.h"

extern "C" {

#include <stdlib.h>

#include "boot\dts\freescale\board.h"
#include "dt-bindings\firmware\imx\rsrc.h"
#include "clk\clk_imx8qxp.h"
#include "linux\of.h"
#include "linux\platform_device.h"
#include "linux\irqchip\irq-imx-irqsteer.h"
#include "kernel\irq\internals.h"

//-----------------------------------------------------------------------
// LDB1_PHY
//-----------------------------------------------------------------------

static resource ldb1_phy_res_list[] = {
    { 0x56221000, 0x56221100, "ldb1-phy-csr", IORESOURCE_MEM },
    { 0x56228000, 0x56229000, "ldb1-phy-ctrl", IORESOURCE_MEM },
};

static struct clk_init_data_desc ldb1_phy_clock_desc[] = {
    { "phy", IMX8QXP_LVDS0_PHY_CLK, IMX8QXP_LVDS0_BYPASS_CLK, 0, 0 },
    {""},
};

//-----------------------------------------------------------------------
// LDB2_PHY
//-----------------------------------------------------------------------

static resource ldb2_phy_res_list[] = {
    { 0x56241000, 0x56241100, "ldb2-phy-csr", IORESOURCE_MEM },
    { 0x56248000, 0x56249000, "ldb2-phy-ctrl", IORESOURCE_MEM },
};

static struct clk_init_data_desc ldb2_phy_clock_desc[] = {
    { "phy", IMX8QXP_LVDS1_PHY_CLK, IMX8QXP_LVDS1_BYPASS_CLK, 0, 0 },
    {""},
};

//-----------------------------------------------------------------------
// LDB1
//-----------------------------------------------------------------------

static resource ldb1_res_list[] = {
    { 0x56221000, 0x562210F0, "ldb1", IORESOURCE_MEM },
};

static int ldb1_id = 0;
static int ldb1_pwr_domains[] = {
    IMX_SC_R_LVDS_0,
    IMX_SC_R_LVDS_1,
};
static char* ldb_pwr_domain_names[] = { "main", "aux" };

static property ldb1_properties[] = {
    { "ldb-id", 1, &ldb1_id },
    { "power-domains", ARRAYSIZE(ldb1_pwr_domains), &ldb1_pwr_domains },
    { "power-domain-names", ARRAYSIZE(ldb_pwr_domain_names),
        &ldb_pwr_domain_names },
    { "" },
};

static int ldb1_ch0_reg = 0;
static const char* ldb1_ch0_data_mapping = "jeida";
static int ldb1_ch0_data_width = 24;
static char* ldb1_ch0_phy_names[] = { "ldb_phy" };

static property ldb1_ch0_properties[] = {
    { "reg", 1, &ldb1_ch0_reg },
    { "fsl,data-mapping", 5, &ldb1_ch0_data_mapping },
    { "fsl,data-width", 1, &ldb1_ch0_data_width },
    { "phy-names", ARRAYSIZE(ldb1_ch0_phy_names), &ldb1_ch0_phy_names },
    { "" },
};

static int ldb1_ch1_reg = 1;
static const char* ldb1_ch1_data_mapping = "jeida";
static int ldb1_ch1_data_width = 24;
static char* ldb1_ch1_phy_names[] = { "ldb_phy" };

static property ldb1_ch1_properties[] = {
    { "reg", 1, &ldb1_ch1_reg },
    { "fsl,data-mapping", 5, &ldb1_ch1_data_mapping },
    { "fsl,data-width", 1, &ldb1_ch1_data_width },
    { "phy-names", ARRAYSIZE(ldb1_ch1_phy_names), &ldb1_ch1_phy_names },
    { "" },
};

static struct device_node ldb1_child[2];

static struct clk_init_data_desc ldb1_clock_desc[] = {
    { "pixel", IMX8QXP_LVDS0_PIX_CLK, IMX8QXP_LVDS0_BYPASS_CLK, 0, 0 },
    { "bypass", IMX8QXP_LVDS0_BYPASS_CLK, 0, 0, 0 },
    { "aux_pixel", IMX8QXP_LVDS1_PIX_CLK, IMX8QXP_LVDS1_BYPASS_CLK, 0, 0 },
    { "aux_bypass", IMX8QXP_LVDS1_BYPASS_CLK, 0, 0, 0 },
    {""},
};

//-----------------------------------------------------------------------
// LDB2
//-----------------------------------------------------------------------

static resource ldb2_res_list[] = {
    { 0x56241000, 0x562410F0, "ldb2", IORESOURCE_MEM },
};

static int ldb2_id = 1;
static int ldb2_pwr_domains[] = {
    IMX_SC_R_LVDS_1,
    IMX_SC_R_LVDS_0,
};

static property ldb2_properties[] = {
    { "ldb-id", 1, &ldb2_id },
    { "power-domains", ARRAYSIZE(ldb2_pwr_domains), &ldb2_pwr_domains },
    { "power-domain-names", ARRAYSIZE(ldb_pwr_domain_names),
        &ldb_pwr_domain_names },
    { "" },
};

static int ldb2_ch0_reg = 0;
static const char* ldb2_ch0_data_mapping = "jeida";
static int ldb2_ch0_data_width = 24;

static property ldb2_ch0_properties[] = {
    { "reg", 1, &ldb2_ch0_reg },
    { "fsl,data-mapping", 5, &ldb2_ch0_data_mapping },
    { "fsl,data-width", 1, &ldb2_ch0_data_width },
    { "phy-names", ARRAYSIZE(ldb1_ch0_phy_names), &ldb1_ch0_phy_names },
    { "" },
};

static int ldb2_ch1_reg = 1;
static const char* ldb2_ch1_data_mapping = "jeida";
static int ldb2_ch1_data_width = 24;

static property ldb2_ch1_properties[] = {
    { "reg", 1, &ldb2_ch1_reg },
    { "fsl,data-mapping", 5, &ldb2_ch1_data_mapping },
    { "fsl,data-width", 1, &ldb2_ch1_data_width },
    { "phy-names", ARRAYSIZE(ldb1_ch1_phy_names), &ldb1_ch1_phy_names },
    { "" },
};

static struct device_node ldb2_child[2];

static struct clk_init_data_desc ldb2_clock_desc[] = {
    { "pixel", IMX8QXP_LVDS1_PIX_CLK, IMX8QXP_LVDS1_BYPASS_CLK, 0, 0 },
    { "bypass", IMX8QXP_LVDS1_BYPASS_CLK, 0, 0, 0 },
    { "aux_pixel", IMX8QXP_LVDS0_PIX_CLK, IMX8QXP_LVDS0_BYPASS_CLK, 0, 0 },
    { "aux_bypass", IMX8QXP_LVDS0_BYPASS_CLK, 0, 0, 0 },
    {""},
};

static void fill_ldb_child(int ldb)
{
    if (ldb == 1)
    {
        ldb1_child[0].name = "lvds-channel0";
        ldb1_child[0].properties = ldb1_ch0_properties;
        ldb1_child[1].name = "lvds-channel1";
        ldb1_child[1].properties = ldb1_ch1_properties;
    }
    if (ldb == 2)
    {
        ldb2_child[0].name = "lvds-channel0";
        ldb2_child[0].properties = ldb2_ch0_properties;
        ldb2_child[1].name = "lvds-channel1";
        ldb2_child[1].properties = ldb2_ch1_properties;
    }
}

//-----------------------------------------------------------------------
// PRG
//-----------------------------------------------------------------------

static resource prg_res_list[] = {
    { 0x56040000, 0x56050000, "dc0_prg1", IORESOURCE_MEM },
    { 0x56050000, 0x56060000, "dc0_prg2", IORESOURCE_MEM },
    { 0x56060000, 0x56070000, "dc0_prg3", IORESOURCE_MEM },
    { 0x56070000, 0x56080000, "dc0_prg4", IORESOURCE_MEM },
    { 0x56080000, 0x56090000, "dc0_prg5", IORESOURCE_MEM },
    { 0x56090000, 0x560a0000, "dc0_prg6", IORESOURCE_MEM },
    { 0x560a0000, 0x560b0000, "dc0_prg7", IORESOURCE_MEM },
    { 0x560b0000, 0x560c0000, "dc0_prg8", IORESOURCE_MEM },
    { 0x560c0000, 0x560d0000, "dc0_prg9", IORESOURCE_MEM },
};

static struct clk_init_data_desc dc0_prg_clk_desc[][3] = {
    {
        { "apb", IMX8QXP_DC0_PRG0_APB_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_PRG0_RTRAM_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_PRG1_APB_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_PRG1_RTRAM_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_PRG2_APB_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_PRG2_RTRAM_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_PRG3_APB_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_PRG3_RTRAM_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_PRG4_APB_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_PRG4_RTRAM_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_PRG5_APB_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_PRG5_RTRAM_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_PRG6_APB_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_PRG6_RTRAM_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_PRG7_APB_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_PRG7_RTRAM_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_PRG8_APB_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_PRG8_RTRAM_CLK, 0, 0, 0 },
        { "" },
    },
};

//-----------------------------------------------------------------------
// DPR
//-----------------------------------------------------------------------

static resource dprc_res_list[] = {
    { 0x560d0000, 0x560e0000, "dc0_dpr1_channel1", IORESOURCE_MEM },
    { 0x560e0000, 0x560f0000, "dc0_dpr1_channel2", IORESOURCE_MEM },
    { 0x560f0000, 0x56100000, "dc0_dpr1_channel3", IORESOURCE_MEM },
    { 0x56100000, 0x56110000, "dc0_dpr2_channel1", IORESOURCE_MEM },
    { 0x56110000, 0x56120000, "dc0_dpr2_channel2", IORESOURCE_MEM },
    { 0x56120000, 0x56130000, "dc0_dpr2_channel3", IORESOURCE_MEM },
};

static int sc_res[] = {
    IMX_SC_R_DC_0_BLIT0,
    IMX_SC_R_DC_0_BLIT1,
    IMX_SC_R_DC_0_FRAC0,
    IMX_SC_R_DC_0_VIDEO0,
    IMX_SC_R_DC_0_VIDEO1,
    IMX_SC_R_DC_0_WARP,
};

static int prgs[][2] = {
    { 0 },
    { 1, 0 },
    { 2 },
    { 3, 4 },
    { 5, 6 },
    { 7, 8 },
};

static property dprc_properties[][3] = {
    {
        { "fsl,sc-resource", 1, &sc_res[0] },
        { "fsl,prgs", 2, &prgs[0] },
        { "" },
    },
    {
        { "fsl,sc-resource", 1, &sc_res[1] },
        { "fsl,prgs", 2, &prgs[1] },
        { "" },
    },
    {
        { "fsl,sc-resource", 1, &sc_res[2] },
        { "fsl,prgs", 2, &prgs[2] },
        { "" },
    },
    {
        { "fsl,sc-resource", 1, &sc_res[3] },
        { "fsl,prgs", 2, &prgs[3] },
        { "" },
    },
    {
        { "fsl,sc-resource", 1, &sc_res[4] },
        { "fsl,prgs", 2, &prgs[4] },
        { "" },
    },
    {
        { "fsl,sc-resource", 1, &sc_res[5] },
        { "fsl,prgs", 2, &prgs[5] },
        { "" },
    },
};

static struct clk_init_data_desc dc0_dpr_clk_desc[][4] = {
    {
        { "apb", IMX8QXP_DC0_DPR0_APB_CLK, 0, 0, 0 },
        { "b", IMX8QXP_DC0_DPR0_B_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_RTRAM0_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_DPR0_APB_CLK, 0, 0, 0 },
        { "b", IMX8QXP_DC0_DPR0_B_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_RTRAM0_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_DPR0_APB_CLK, 0, 0, 0 },
        { "b", IMX8QXP_DC0_DPR0_B_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_RTRAM0_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_DPR1_APB_CLK, 0, 0, 0 },
        { "b", IMX8QXP_DC0_DPR1_B_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_RTRAM1_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_DPR1_APB_CLK, 0, 0, 0 },
        { "b", IMX8QXP_DC0_DPR1_B_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_RTRAM1_CLK, 0, 0, 0 },
        { "" },
    },
    {
        { "apb", IMX8QXP_DC0_DPR1_APB_CLK, 0, 0, 0 },
        { "b", IMX8QXP_DC0_DPR1_B_CLK, 0, 0, 0 },
        { "rtram", IMX8QXP_DC0_RTRAM1_CLK, 0, 0, 0 },
        { "" },
    },
};

//-----------------------------------------------------------------------
// DPU
//-----------------------------------------------------------------------

static int dpu_interrupts[] = {
    448, 449, 450,  64,
    65,  66,  67,  68,
    69,  70, 193, 194,
    195, 196, 197,  72,
    73,  74,  75,  76,
    77,  78,  79,  80,
    81, 199, 200, 201,
    202, 203, 204, 205,
    206, 207, 208,   5,
    0,   1,   2,   3,
    4,  82,  83,  84,
    85, 209, 210, 211,
    212,
};

static char* dpu_interrupt_names[] = {
    "store9_shdload",
    "store9_framecomplete",
    "store9_seqcomplete",
    "extdst0_shdload",
    "extdst0_framecomplete",
    "extdst0_seqcomplete",
    "extdst4_shdload",
    "extdst4_framecomplete",
    "extdst4_seqcomplete",
    "extdst1_shdload",
    "extdst1_framecomplete",
    "extdst1_seqcomplete",
    "extdst5_shdload",
    "extdst5_framecomplete",
    "extdst5_seqcomplete",
    "disengcfg_shdload0",
    "disengcfg_framecomplete0",
    "disengcfg_seqcomplete0",
    "framegen0_int0",
    "framegen0_int1",
    "framegen0_int2",
    "framegen0_int3",
    "sig0_shdload",
    "sig0_valid",
    "sig0_error",
    "disengcfg_shdload1",
    "disengcfg_framecomplete1",
    "disengcfg_seqcomplete1",
    "framegen1_int0",
    "framegen1_int1",
    "framegen1_int2",
    "framegen1_int3",
    "sig1_shdload",
    "sig1_valid",
    "sig1_error",
    "reserved",
    "cmdseq_error",
    "comctrl_sw0",
    "comctrl_sw1",
    "comctrl_sw2",
    "comctrl_sw3",
    "framegen0_primsync_on",
    "framegen0_primsync_off",
    "framegen0_secsync_on",
    "framegen0_secsync_off",
    "framegen1_primsync_on",
    "framegen1_primsync_off",
    "framegen1_secsync_on",
    "framegen1_secsync_off",
};

static resource dpu_res_list[] = {
    { 0x56180000, 0x561C0000, "dpu", IORESOURCE_MEM },
};

static int dpu_id = 0;
static int pwr_domains[] = {
    IMX_SC_R_DC_0,
    IMX_SC_R_DC_0_PLL_0,
    IMX_SC_R_DC_0_PLL_1,
};
static char* pwr_domain_names[] = { "dc", "pll0", "pll1" };

static property dpu_properties[] = {
    { "dpu-id", 1, &dpu_id },
    { "power-domains", ARRAYSIZE(pwr_domains), &pwr_domains },
    { "power-domain-names", ARRAYSIZE(pwr_domain_names), &pwr_domain_names },
    { "" },
};

static struct clk_init_data_desc dpu_clock_desc[] = {
    { "pll0", IMX8QXP_DC0_PLL0_CLK, 0, 0, 0 },
    { "pll1", IMX8QXP_DC0_PLL1_CLK, 0, 0, 0 },
    { "bypass0", IMX8QXP_DC0_BYPASS0_CLK, 0, 0, 0 },
    { "disp0", IMX8QXP_DC0_DISP0_CLK, 0, 0, 0 },
    { "disp1", IMX8QXP_DC0_DISP1_CLK, 0, 0, 0 },
    { "disp0_lpcg", IMX8QXP_DC0_DISP0_LPCG_CLK, 0, 0, 0 },
    { "disp1_lpcg", IMX8QXP_DC0_DISP1_LPCG_CLK, 0, 0, 0 },
    {""}
};

//-----------------------------------------------------------------------
// IRQ Steer
//-----------------------------------------------------------------------

static resource irqsteer_res_list[] = {
    { 0x56000000, 0x56010000, "irqsteer", IORESOURCE_MEM },
};

static int channel = 0;
static int num_irqs = 512;

static property irqsteer_properties[] = {
    { "fsl,channel", 1, &channel },
    { "fsl,num-irqs", 1, &num_irqs },
    { "" },
};

//-----------------------------------------------------------------------
// mipi-dsi0
//-----------------------------------------------------------------------

static struct clk_init_data_desc mipi_dsi0_clock_desc[] = {
    { "pixel", IMX8QXP_MIPI0_PIXEL_CLK, 0, 0, 0 },
    { "bypass", IMX8QXP_MIPI0_BYPASS_CLK, 0, 0, 0 },
    { "phy_ref", IMX8QXP_MIPI0_PHY_REF_CLK, 0, 0, 0 },
    { "tx_esc", IMX8QXP_MIPI0_TX_ESC_CLK, 0, 0, 0 },
    { "rx_esc", IMX8QXP_MIPI0_RX_ESC_CLK, 0, 0, 0 },
    { "phy_parent", IMX8QXP_MIPI_PLL_DIV2_CLK, 0, 0, 0 },
    {""}
};

static resource mipi_dsi0_res_list[] = {
    { 0x56228000, 0x56229000, "mipi0-reg", IORESOURCE_MEM },
    { 0x56221000, 0x56222000, "mipi0-csr", IORESOURCE_MEM },
};

static int instance0 = 0;
static int clk_drop_lvl_0 = 0;
static property mipi_dsi0_properties[] = {
    { "compatible", 1, "fsl,imx8qx-nwl-dsi" },
    { "mipi-dsi-id", 1, &instance0 },
    { "fsl,clock-drop-level", 1, &clk_drop_lvl_0 },
    { "" },
};
//-----------------------------------------------------------------------
// mipi-dsi-phy0
//-----------------------------------------------------------------------

static struct clk_init_data_desc mipi_dsi_phy0_clock_desc[] = {
    { "phy_ref", IMX8QXP_MIPI0_PHY_REF_CLK, 0, 0, 0 },
    {""}
};

static resource mipi_dsi_phy0_res_list[] = {
    { 0x56228300, 0x56228400, "mipi-phy0-reg", IORESOURCE_MEM },
};

static property mipi_dsi_phy0_properties[] = {
    { "compatible", 1, "fsl,imx8qx-mipi-dphy" },
    { "" },
};

//-----------------------------------------------------------------------
// mipi-dsi1
//-----------------------------------------------------------------------

static struct clk_init_data_desc mipi_dsi1_clock_desc[] = {
    { "pixel", IMX8QXP_MIPI1_PIXEL_CLK, 0, 0, 0 },
    { "bypass", IMX8QXP_MIPI1_BYPASS_CLK, 0, 0, 0 },
    { "phy_ref", IMX8QXP_MIPI1_PHY_REF_CLK, 0, 0, 0 },
    { "tx_esc", IMX8QXP_MIPI1_TX_ESC_CLK, 0, 0, 0 },
    { "rx_esc", IMX8QXP_MIPI1_RX_ESC_CLK, 0, 0, 0 },
    { "phy_parent", IMX8QXP_MIPI_PLL_DIV2_CLK, 0, 0, 0 },
    {""}
};

static resource mipi_dsi1_res_list[] = {
    { 0x56248000, 0x56249000, "mipi1-reg", IORESOURCE_MEM },
    { 0x56241000, 0x56242000, "mipi1-csr", IORESOURCE_MEM },
};

static int instance1 = 1;
static int clk_drop_lvl_1 = 0;
static property mipi_dsi1_properties[] = {
    { "compatible", 1, "fsl,imx8qx-nwl-dsi" },
    { "mipi-dsi-id", 1, &instance1 },
    { "fsl,clock-drop-level", 1, &clk_drop_lvl_1 },
    { "" },
};
//-----------------------------------------------------------------------
// mipi-dsi-phy1
//-----------------------------------------------------------------------

static struct clk_init_data_desc mipi_dsi_phy1_clock_desc[] = {
    { "phy_ref", IMX8QXP_MIPI1_PHY_REF_CLK, 0, 0, 0 },
    {""}
};

static resource mipi_dsi_phy1_res_list[] = {
    { 0x56248300, 0x56248400, "mipi-phy1-reg", IORESOURCE_MEM },
};

static property mipi_dsi_phy1_properties[] = {
    { "compatible", 1, "fsl,imx8qx-mipi-dphy" },
    { "" },
};

//-----------------------------------------------------------------------
// Init functions
//-----------------------------------------------------------------------
/*
static void mipi_irq_init(unsigned int mipi_index)
{
    if (mipi_index == 1)
    {
        irq_desc[IRQ_DESC_MIPI_DSI0].name = "mipi-dsi";
        irq_desc[IRQ_DESC_MIPI_DSI0].irq_data.hwirq = 16;
        irq_desc[IRQ_DESC_MIPI_DSI0].handler = nullptr;
        irq_desc[IRQ_DESC_MIPI_DSI0].irq_data.chip.type = IRQCHIP_TYPE_IRQ_STEER;
    }
    if (mipi_index == 2)
    {
        irq_desc[IRQ_DESC_MIPI_DSI1].name = "mipi-dsi";
        irq_desc[IRQ_DESC_MIPI_DSI1].irq_data.hwirq = 16;
        irq_desc[IRQ_DESC_MIPI_DSI1].handler = nullptr;
        irq_desc[IRQ_DESC_MIPI_DSI1].irq_data.chip.type = IRQCHIP_TYPE_IRQ_STEER;
    }
}
*/
static void mipi_res_init(platform_device* pdev, int mipi)
{
    if (mipi == 1)
    {
        pdev->resource = &mipi_dsi0_res_list[0];
        pdev->num_resources = ARRAYSIZE(mipi_dsi0_res_list);
        pdev->dev.of_node.properties = mipi_dsi0_properties;
        pdev->dev.of_clk = (struct clk_init_data_desc*)&mipi_dsi0_clock_desc;
        pdev->dev.get_clock_item = &clk_get_item_imx8qxp;
    }

    if (mipi == 2)
    {
        pdev->resource = &mipi_dsi1_res_list[0];
        pdev->num_resources = ARRAYSIZE(mipi_dsi1_res_list);
        pdev->dev.of_node.properties = mipi_dsi1_properties;
        pdev->dev.of_clk = (struct clk_init_data_desc*)&mipi_dsi1_clock_desc;
        pdev->dev.get_clock_item = &clk_get_item_imx8qxp;
    }
}

static void mipi_dphy_res_init(platform_device* pdev, int phy)
{
    if (phy == 1)
    {
        pdev->resource = &mipi_dsi_phy0_res_list[0];
        pdev->num_resources = ARRAYSIZE(mipi_dsi_phy0_res_list);
        pdev->dev.of_node.properties = mipi_dsi_phy0_properties;
        pdev->dev.of_clk = (struct clk_init_data_desc*)&mipi_dsi_phy0_clock_desc;
        pdev->dev.get_clock_item = &clk_get_item_imx8qxp;
    }
    if (phy == 2)
    {
        pdev->resource = &mipi_dsi_phy1_res_list[0];
        pdev->num_resources = ARRAYSIZE(mipi_dsi_phy1_res_list);
        pdev->dev.of_node.properties = mipi_dsi_phy1_properties;
        pdev->dev.of_clk = (struct clk_init_data_desc*)&mipi_dsi_phy1_clock_desc;
        pdev->dev.get_clock_item = &clk_get_item_imx8qxp;
    }
}

void ldb_prop_init(unsigned int ldb_index, unsigned int data_width, const char *bus_mapping)
{
    if (ldb_index == 1)
    {
        ldb1_ch0_data_mapping = bus_mapping;
        ldb1_ch0_data_width = data_width;
        ldb1_ch1_data_mapping = bus_mapping;
        ldb1_ch1_data_width = data_width;
    }

    if (ldb_index == 2)
    {
        ldb2_ch0_data_mapping = bus_mapping;
        ldb2_ch0_data_width = data_width;
        ldb2_ch1_data_mapping = bus_mapping;
        ldb2_ch1_data_width = data_width;
    }
}

static void dpu_irq_init(void)
{
    for (int i = 0; i < ARRAYSIZE(dpu_interrupts); i++)
    {
        irq_desc[i].name = dpu_interrupt_names[i];
        irq_desc[i].irq_data.hwirq = dpu_interrupts[i];
        irq_desc[i].handler = nullptr;
        irq_desc[i].irq_data.chip.type = IRQCHIP_TYPE_IRQ_STEER;
        if (i > 2)  /* Skip interrupts from DPU Blit engine */
        {
            irq_state_set_disabled(&irq_desc[i]);
            irq_state_set_masked(&irq_desc[i]);
        }
    }
}

static inline void dpu_res_init(platform_device* pdev)
{
    pdev->resource = &dpu_res_list[0];
    pdev->num_resources = ARRAYSIZE(dpu_res_list);
    pdev->dev.of_node.properties = dpu_properties;
    pdev->dev.of_clk = (struct clk_init_data_desc *)&dpu_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8qxp;
}

static inline void irqsteer_res_init(platform_device* pdev)
{
    pdev->resource = &irqsteer_res_list[0];
    pdev->num_resources = ARRAYSIZE(irqsteer_res_list);
    pdev->dev.of_node.properties = irqsteer_properties;
}

static inline void irqsteer_res_deinit(platform_device* pdev)
{
    pdev->resource = nullptr;
    pdev->dev.of_node.properties = nullptr;
}

static void ldb_res_init(platform_device* pdev, int ldb)
{
    if (ldb == 1)
    {
        pdev->resource = &ldb1_res_list[0];
        pdev->num_resources = ARRAYSIZE(ldb1_res_list);
        pdev->dev.of_node.properties = ldb1_properties;
        fill_ldb_child(1);
        pdev->dev.of_node.child = ldb1_child;
        pdev->dev.of_node.num_childs = 1;
        pdev->dev.of_clk = (struct clk_init_data_desc*)&ldb1_clock_desc;
        pdev->dev.get_clock_item = &clk_get_item_imx8qxp;
    }

    if (ldb == 2)
    {
        pdev->resource = &ldb2_res_list[0];
        pdev->num_resources = ARRAYSIZE(ldb2_res_list);
        pdev->dev.of_node.properties = ldb2_properties;
        fill_ldb_child(2);
        pdev->dev.of_node.child = ldb2_child;
        pdev->dev.of_node.num_childs = 1;
        pdev->dev.of_clk = (struct clk_init_data_desc*)&ldb2_clock_desc;
        pdev->dev.get_clock_item = &clk_get_item_imx8qxp;
    }
}

static void phy_res_init(platform_device *pdev, int phy)
{
    if (phy == 1)
    {
        pdev->resource = &ldb1_phy_res_list[0];
        pdev->num_resources = ARRAYSIZE(ldb1_phy_res_list);
        pdev->dev.of_clk = (struct clk_init_data_desc*)&ldb1_phy_clock_desc;
        pdev->dev.get_clock_item = &clk_get_item_imx8qxp;
    }
    if (phy == 2)
    {
        pdev->resource = &ldb2_phy_res_list[0];
        pdev->num_resources = ARRAYSIZE(ldb2_phy_res_list);
        pdev->dev.of_clk = (struct clk_init_data_desc*)&ldb2_phy_clock_desc;
        pdev->dev.get_clock_item = &clk_get_item_imx8qxp;
    }
}

void qxp_board_init(platform_device* pdev)
{
    NT_ASSERT(pdev);

    if (!strcmp(pdev->name, "dpu"))
    {
        dpu_res_init(pdev);
        dpu_irq_init();
    }
    else if (!strcmp(pdev->name, "irqsteer"))
    {
        irqsteer_res_init(pdev);
        imx_irqsteer_probe(pdev);
    }
    else if (!strcmp(pdev->name, "ldb1"))
    {
        ldb_res_init(pdev, 1);
    }
    else if (!strcmp(pdev->name, "ldb1_phy"))
    {
        phy_res_init(pdev, 1);
    }
    else if (!strcmp(pdev->name, "ldb2"))
    {
        ldb_res_init(pdev, 2);
    }
    else if (!strcmp(pdev->name, "ldb2_phy"))
    {
        phy_res_init(pdev, 2);
    }
    else if (!strcmp(pdev->name, "mipi0_dsi_host"))
    {
        mipi_res_init(pdev, 1);
    }
    else if (!strcmp(pdev->name, "mipi1_dsi_host"))
    {
        mipi_res_init(pdev, 2);
    }
    else if (!strcmp(pdev->name, "mipi0_dphy"))
    {
        mipi_dphy_res_init(pdev, 1);
    }
    else if (!strcmp(pdev->name, "mipi1_dphy"))
    {
        mipi_dphy_res_init(pdev, 2);
    }
}

void qxp_board_deinit(platform_device* pdev)
{
    NT_ASSERT(pdev);

    if (!strcmp(pdev->name, "irqsteer"))
    {
        imx_irqsteer_remove(pdev);
        irqsteer_res_deinit(pdev);
    }
}

int qxp_irq_count()
{
    return ARRAYSIZE(dpu_interrupts);
}

void prg_res_init(platform_device* pdev, int cnt)
{
    if (cnt > ARRAYSIZE(prg_res_list)) {
        return;
    }

    for (int i = 0; i < cnt; i++) {
        pdev[i].resource = &prg_res_list[i];
        pdev[i].num_resources = 1;
        pdev[i].dev.of_clk = (struct clk_init_data_desc*)&dc0_prg_clk_desc[i];
        pdev[i].dev.get_clock_item = &clk_get_item_imx8qxp;
    }
}

void dprc_res_init(platform_device* pdev, int cnt)
{
    if (cnt > ARRAYSIZE(dprc_res_list)) {
        return;
    }

    for (int i = 0; i < cnt; i++) {
        pdev[i].resource = &dprc_res_list[i];
        pdev[i].num_resources = 1;
        pdev[i].dev.of_node.properties = (struct property*)&dprc_properties[i];
        pdev[i].dev.of_clk = (struct clk_init_data_desc*)&dc0_dpr_clk_desc[i];
        pdev[i].dev.get_clock_item = &clk_get_item_imx8qxp;
    }
}

} // extern "C"
