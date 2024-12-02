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

#include "precomp.h"

extern "C" {

#include <stdlib.h>

#include "boot\dts\freescale\board.h"
#include "linux\of.h"
#include "linux\platform_device.h"
#include "clk\clk_imx8mp.h"
#include "kernel\irq\internals.h"

/* Resources such as registers memory area allocated for devices */
static resource mp_lcdif1_res_list[] = {
    { 0x32E80000, 0x32E8FFFF, "lcdif1", IORESOURCE_MEM },
};

static resource mp_lcdif2_res_list[] = {
    { 0x32E90000, 0x32E9FFFF, "lcdif2", IORESOURCE_MEM },
};

static resource mp_lvds_res_list[] = {
    { 0x32EC0000, 0x32ECFFFF, "media-blk-ctl", IORESOURCE_MEM },
};

static resource mp_mipi_res_list[] = {
    { 0x32E60000, 0x32E6FFFF, "mipi-dsi1", IORESOURCE_MEM },
    { 0x32EC0000, 0x32ECFFFF, "media-blk-ctl", IORESOURCE_MEM },
};

static resource mp_lcdif3_res_list[] = {
    { 0x32FC6000, 0x32FC6FFF, "lcdif3", IORESOURCE_MEM },
    { 0x32FC0000, 0x32FC0FFF, "hdmimix-blk-ctl", IORESOURCE_MEM },
};

static resource mp_hdmi_res_list[] = {
    /* Alocate space for hdmi+phy together, because phy is not page aligned */
    { 0x32FD8000, 0x32FDFFFF, "hdmi", IORESOURCE_MEM },
    { 0x32FC0000, 0x32FC0FFF, "hdmimix-blk-ctl", IORESOURCE_MEM },
    { 0x32FC4000, 0x32FC4FFF, "hdmi-pai-pvi", IORESOURCE_MEM },
};

static resource mp_hdmi_phy_res_list[] = {
    { 0x32FDFF00, 0x32FDFFFF, "hdmi-phy", IORESOURCE_MEM },
};

static resource irqsteer_res_list[] = {
    { 0x32FC2000, 0x32FC2FFF, "irqsteer", IORESOURCE_MEM },
};

/* Properties such as compatible assigned to devices */
static property mp_lcdif2_properties[] = {
    { "compatible", 1, "fsl,imx8mp-lcdif2" },
    { "" } // marks end of the list
};

static property mp_lcdif1_properties[] = {
    { "compatible", 1, "fsl,imx8mp-lcdif1" },
    { "" } // marks end of the list
};

static property mp_lcdif3_properties[] = {
    { "compatible", 1, "fsl,imx8mp-lcdif3" },
    { "" } // marks end of the list
};

static property mp_hdmi_properties[] = {
    { "compatible", 1, "fsl,imx8mp-hdmi" },
    { "" } // marks end of the list
};

static property mp_mipi_properties[] = {
    { "compatible", 1, "fsl,imx8mp-mipi-dsim" },
    { "" } // marks end of the list
};

static int channel = 0;
static int num_irqs = 64;

static property irqsteer_properties[] = {
    { "fsl,channel", 1, &channel },
    { "fsl,num-irqs", 1, &num_irqs },
    { "" } // marks end of the list
};

/* Clock config for devices:
   name: The same name as used in the device driver in "devm_clk_get" call. Mandatory.
   id: index of the clock item in the clock tree. See "Clock tree items - START" table.
       Mandatory.
   parent_sel: Initial value of the parent selector for attached clock slice.
               Set during "devm_clk_get" call. See "Clock tree items - START" table.
               Optional, set to zero if not used.
   rate: initial rate of the clock in Hz set during "devm_clk_get" call.
         Optional, set to zero if not used.
   parent_enable: Non-zero = initially enable all attached parents
                 (doesn't apply for the clock itself, it's state remains the same).
                 Zero if not used.
*/
static struct clk_init_data_desc mp_lcdif1_clock_desc[] = {
    /* name, id, parent_sel, rate, parent_enable */
    { "pix", IMX8MP_CLK_MEDIA_LCDIF1_PIXEL, IMX8MP_VIDEO_PLL1, 0, 1 },
    { "disp-axi", IMX8MP_CLK_MEDIA_LCDIF1_AXI, IMX8MP_SYS_PLL2, IMX_CLK_FREQ_500M, 1 },
    { "disp-apb", IMX8MP_CLK_MEDIA_LCDIF1_APB, IMX8MP_SYS_PLL1, IMX_CLK_FREQ_200M, 1 },
    {""} // marks end of the list
};

static struct clk_init_data_desc mp_lcdif2_clock_desc[] = {
    /* name, id, parent_sel, rate, parent_enable */
    { "pix", IMX8MP_CLK_MEDIA_LCDIF2_PIXEL, IMX8MP_VIDEO_PLL1, 0, 1 },
    { "disp-axi", IMX8MP_CLK_MEDIA_LCDIF2_AXI, IMX8MP_SYS_PLL2, IMX_CLK_FREQ_500M, 1 },
    { "disp-apb", IMX8MP_CLK_MEDIA_LCDIF2_APB, IMX8MP_SYS_PLL1, IMX_CLK_FREQ_200M, 1 },
    {""} // marks end of the list
};

static struct clk_init_data_desc mp_lvds_clock_desc[] = {
    /* name, id, parent_sel, rate, parent_enable */
    { "ldb", IMX8MP_CLK_MEDIA_LDB, IMX8MP_VIDEO_PLL1, 0, 0 },
    { "apb", IMX8MP_CLK_MEDIA_APB, 0, 0,  0 }, /* APB initialized by LCDIF */
    {""} // marks end of the list
};

static struct clk_init_data_desc mp_mipi_clock_desc[] = {
    /* name, id, parent_sel, rate, parent_enable */
    { "pll-ref", IMX8MP_CLK_MEDIA_MIPI_DSI_CLKREF, IMX8MP_CLK_24M, IMX_CLK_FREQ_12M, 1 },
    { "cfg", IMX8MP_CLK_MEDIA_MIPI_DSI_PCLK, 0, 0, 0 }, /* Parent of MIPI_DSI_PCLK is APB, which is initialized by LCDIF */
    {""} // marks end of the list
};

static struct clk_init_data_desc mp_irqsteer_clock_desc[] = {
    /* name, id, parent_sel, rate, parent_enable */
    { "ipg", IMX8MP_CLK_HDMI_IRQS_STEER, 0, 0, 0 },
    {""} // marks end of the list
};

static struct clk_init_data_desc mp_lcdif3_clock_desc[] = {
    /* name, id, parent_sel, rate, parent_enable */
    { "pix", IMX8MP_CLK_HDMI_PHY, 0, 0, 0 },
    { "disp-axi", IMX8MP_CLK_HDMI_AXI, IMX8MP_SYS_PLL2_DIV2, IMX_CLK_FREQ_500M, 1 },
    { "disp-apb", IMX8MP_CLK_HDMI_APB, IMX8MP_SYS_PLL1_DIV6, IMX_CLK_FREQ_133_3M, 1 },
    { "mix_apb", IMX8MP_CLK_HDMI_GLOBAL_APB, 0, 0, 0 },
    { "mix_axi", IMX8MP_CLK_HDMI_GLOBAL_B, 0, 0, 0 },
    { "xtl_24m", IMX8MP_CLK_HDMI_GLOBAL_XTAL24M, 0, 0, 0 },
    { "mix_pix", IMX8MP_CLK_HDMI_GLOBAL_TX_PIX, 0, 0, 0 },
    { "lcdif_apb", IMX8MP_CLK_HDMI_LCDIF_APB, 0, 0, 0 },
    { "lcdif_axi", IMX8MP_CLK_HDMI_LCDIF_B, 0, 0, 0 },
    { "lcdif_pdi", IMX8MP_CLK_HDMI_LCDIF_PDI, 0, 0, 0 },
    { "lcdif_pix", IMX8MP_CLK_HDMI_LCDIF_PIX, 0, 0, 0 },
    { "lcdif_spu", IMX8MP_CLK_HDMI_LCDIF_SPU, 0, 0, 0 },
    { "noc_hdmi", IMX8MP_CLK_HDMI_NOC_HDMI, 0, 0, 0 },
    {""} // marks end of the list
};

static struct clk_init_data_desc mp_hdmi_phy_clock_desc[] = {
    /* name, id, parent_sel, rate, parent_enable */
    { "apb", IMX8MP_CLK_HDMI_TX_PHY_APB, 0, 0, 0 }, /* PHY clk */
    { "ref", IMX8MP_CLK_HDMI_GLOBAL_XTAL24M, 0, 0, 0 }, /* PHY clk */
    {""} // marks end of the list
};

static struct clk_init_data_desc mp_hdmi_clock_desc[] = {
    /* name, id, parent_sel, rate, parent_enable */
    { "pvi_clk", IMX8MP_CLK_HDMI_TX_VID_LINK_PIX, 0, 0, 0 }, /* PAVI*/
    { "pai_clk", IMX8MP_CLK_HDMI_TX_GPA, 0, 0, 0 }, /* PAVI*/
    { "iahb", IMX8MP_CLK_HDMI_APB, 0, 0, 0 }, /* shared with lcdif3 - initialized there */
    { "isfr", IMX8MP_CLK_HDMI_24M, IMX8MP_CLK_24M, IMX_CLK_FREQ_24M, 1 },
    { "phy_int", IMX8MP_CLK_HDMI_TX_PHY_INT, 0, 0, 0 },
    { "prep_clk", IMX8MP_CLK_HDMI_TX_PREP, 0, 0, 0 },
    { "skp_clk", IMX8MP_CLK_HDMI_TX_SKP, 0, 0, 0 },
    { "sfr_clk", IMX8MP_CLK_HDMI_TX_SFR, 0, 0, 0 },
    { "pix_clk", IMX8MP_CLK_HDMI_TX_PIXEL, 0, 0, 0 },
    { "cec_clk", IMX8MP_CLK_HDMI_TX_CEC, 0, 0, 0 },
    { "apb_clk", IMX8MP_CLK_HDMI_TX_APB, 0, 0, 0 },
    { "hpi_clk", IMX8MP_CLK_HDMI_TX_HPI, 0, 0, 0 },
    { "fdcc_ref", IMX8MP_CLK_HDMI_FDCC_REF, 0, 0, 0 },
    { "pipe_clk", IMX8MP_CLK_HDMI_TX_PIPE, 0, 0, 0 },
    {""} // marks end of the list
};

static void lcdif2_irq_init(void)
{
    irq_desc[IRQ_DESC_VBLANK].name = "vblank";
    irq_desc[IRQ_DESC_VBLANK].irq_data.hwirq = 6;
    irq_desc[IRQ_DESC_VBLANK].handler = nullptr;
    irq_desc[IRQ_DESC_VBLANK].irq_data.chip.type = IRQCHIP_TYPE_GIC;
}

static void lcdif1_irq_init(void)
{
    irq_desc[IRQ_DESC_VBLANK_LCDIF1].name = "vblank";
    irq_desc[IRQ_DESC_VBLANK_LCDIF1].irq_data.hwirq = 5;
    irq_desc[IRQ_DESC_VBLANK_LCDIF1].handler = nullptr;
    irq_desc[IRQ_DESC_VBLANK_LCDIF1].irq_data.chip.type = IRQCHIP_TYPE_GIC;
}

static void lcdif3_irq_init(void)
{
    irq_desc[IRQ_DESC_VBLANK].name = "vblank";
    irq_desc[IRQ_DESC_VBLANK].irq_data.hwirq = 8;
    irq_desc[IRQ_DESC_VBLANK].handler = nullptr;
    irq_desc[IRQ_DESC_VBLANK].irq_data.chip.type = IRQCHIP_TYPE_IRQ_STEER;
    irq_state_set_disabled(&irq_desc[IRQ_DESC_VBLANK]);
    irq_state_set_masked(&irq_desc[IRQ_DESC_VBLANK]);
    /* Initialize also HDMI interrupt here, so irqsteer probe has complete data */
    irq_desc[IRQ_DESC_HDMI_TX].name = "u_hdmi_tx_ointerrupt";
    irq_desc[IRQ_DESC_HDMI_TX].irq_data.hwirq = 0;
    irq_desc[IRQ_DESC_HDMI_TX].handler = nullptr;
    irq_desc[IRQ_DESC_HDMI_TX].irq_data.chip.type = IRQCHIP_TYPE_IRQ_STEER;
    irq_state_set_disabled(&irq_desc[IRQ_DESC_HDMI_TX]);
    irq_state_set_masked(&irq_desc[IRQ_DESC_HDMI_TX]);
}

static inline void lvds_res_init(platform_device* pdev)
{
    pdev->resource = &mp_lvds_res_list[0];
    pdev->num_resources = ARRAYSIZE(mp_lvds_res_list);
    pdev->dev.of_clk = (struct clk_init_data_desc *)&mp_lvds_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8mp;
}

static inline void mipi_res_init(platform_device* pdev)
{
    pdev->resource = &mp_mipi_res_list[0];
    pdev->num_resources = ARRAYSIZE(mp_mipi_res_list);
    pdev->dev.of_node.properties = (property*)&mp_mipi_properties;
    pdev->dev.of_clk = (struct clk_init_data_desc *)&mp_mipi_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8mp;
}

static inline void lcdif1_res_init(platform_device* pdev)
{
    pdev->resource = &mp_lcdif1_res_list[0];
    pdev->num_resources = ARRAYSIZE(mp_lcdif1_res_list);
    pdev->dev.of_node.properties = (property *) &mp_lcdif1_properties;
    pdev->dev.of_clk = (struct clk_init_data_desc *)&mp_lcdif1_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8mp;
}

static inline void lcdif2_res_init(platform_device* pdev)
{
    pdev->resource = &mp_lcdif2_res_list[0];
    pdev->num_resources = ARRAYSIZE(mp_lcdif2_res_list);
    pdev->dev.of_node.properties = (property *) &mp_lcdif2_properties;
    pdev->dev.of_clk = (struct clk_init_data_desc *)&mp_lcdif2_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8mp;
}

static inline void irqsteer_res_init(platform_device* pdev)
{
    pdev->resource = &irqsteer_res_list[0];
    pdev->num_resources = ARRAYSIZE(irqsteer_res_list);
    pdev->dev.of_clk = (struct clk_init_data_desc*)&mp_irqsteer_clock_desc;
    pdev->dev.of_node.properties = irqsteer_properties;
    pdev->dev.get_clock_item = &clk_get_item_imx8mp;
}

static inline void lcdif3_res_init(platform_device* pdev)
{
    pdev->resource = &mp_lcdif3_res_list[0];
    pdev->num_resources = ARRAYSIZE(mp_lcdif3_res_list);
    pdev->dev.of_node.properties = (property*)&mp_lcdif3_properties;
    pdev->dev.of_clk = (struct clk_init_data_desc*)&mp_lcdif3_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8mp;
}

static inline void hdmi_res_init(platform_device* pdev)
{
    pdev->resource = &mp_hdmi_res_list[0];
    pdev->num_resources = ARRAYSIZE(mp_hdmi_res_list);
    pdev->dev.of_node.properties = (property *) &mp_hdmi_properties;
    pdev->dev.of_clk = (struct clk_init_data_desc *)&mp_hdmi_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8mp;
}

static inline void hdmi_phy_res_init(platform_device* pdev)
{
    pdev->resource = &mp_hdmi_phy_res_list[0];
    pdev->num_resources = ARRAYSIZE(mp_hdmi_phy_res_list);
    pdev->dev.of_clk = (struct clk_init_data_desc*)&mp_hdmi_phy_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8mp;
}

void mp_board_init(platform_device* pdev)
{
    NT_ASSERT(pdev);

    if (!strcmp(pdev->name, "lcdif1_dev"))
    {
        lcdif1_res_init(pdev);
        lcdif1_irq_init();
    }
    else if (!strcmp(pdev->name, "lcdif2_dev"))
    {
        lcdif2_res_init(pdev);
        lcdif2_irq_init();
    }
    else if ((!strcmp(pdev->name, "lvds_dev")) || (!strcmp(pdev->name, "lvds_phy")))
    {
        lvds_res_init(pdev);
    }
    else if (!strcmp(pdev->name, "mipi_dsi_dev"))
    {
        mipi_res_init(pdev);
    }
    else if (!strcmp(pdev->name, "hdmi_dev"))
    {
        hdmi_res_init(pdev);
    }
    else if (!strcmp(pdev->name, "lcdif3_dev"))
    {
        lcdif3_res_init(pdev);
        lcdif3_irq_init();
    }
    else if (!strcmp(pdev->name, "irqsteer"))
    {
        irqsteer_res_init(pdev);
    }
    else if (!strcmp(pdev->name, "hdmi_phy"))
    {
        hdmi_phy_res_init(pdev);
    }

}

void mp_board_deinit(platform_device* pdev)
{
    NT_ASSERT(pdev);
}

} // extern "C"
