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

#include "precomp.h"

extern "C" {

#include <stdlib.h>

#include "boot\dts\freescale\board.h"
#include "linux\of.h"
#include "linux\platform_device.h"
#include "clk\clk_imx8mn.h"

/* Resources such as registers memory area allocated for devices */
static resource mn_lcdif_res_list[] = {
    { 0x32E00000, 0x32E0FFFF, "lcdif", IORESOURCE_MEM },
    { 0x32E28000, 0x32E2FFFF, "media-blk-ctl", IORESOURCE_MEM },
};

static resource mn_mipi_res_list[] = {
    { 0x32E10000, 0x32E1FFFF, "mipi-dsi", IORESOURCE_MEM },
    { 0x32E28000, 0x32E2FFFF, "media-blk-ctl", IORESOURCE_MEM },
};

static property mn_lcdif_properties[] = {
    { "compatible", 1, "fsl,imx8mn-lcdif" },
    { "" } // marks end of the list
};

static property mn_mipi_properties[] = {
    { "compatible", 1, "fsl,imx8mn-mipi-dsim" },
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

static struct clk_init_data_desc mn_lcdif_clock_desc[] = {
    /* name, id, parent_sel, rate, parent_enable */
    { "pix", IMX8MN_CLK_LCDIF_PIXEL, IMX8MN_VIDEO_PLL1, 0, 1 },
    { "disp-axi", IMX8MN_CLK_DISP_AXI_ROOT, IMX8MN_SYS_PLL2, IMX_CLK_FREQ_500M, 1 },
    { "disp-apb", IMX8MN_CLK_DISP_APB_ROOT, IMX8MN_SYS_PLL1, IMX_CLK_FREQ_200M, 1 },
    {""} // marks end of the list
};

static struct clk_init_data_desc mn_mipi_clock_desc[] = {
    /* name, id, parent_sel, rate, parent_enable */
    { "pll-ref", IMX8MN_CLK_DSI_PHY_REF, IMX8MN_VIDEO_PLL1, 0, 0 }, /* Parent enabled by lcdif pix clk init */
    { "cfg", IMX8MN_CLK_DSI_CORE, IMX8MN_SYS_PLL1_DIV3, IMX_CLK_FREQ_266_6M, 1 },
    {""} // marks end of the list
};

static void lcdif_irq_init(void)
{
    irq_desc[0].name = "vblank";
    irq_desc[0].irq_data.hwirq = 5;
    irq_desc[0].handler = nullptr;
    irq_desc[0].irq_data.chip.type = IRQCHIP_TYPE_GIC;
}

static inline void mipi_res_init(platform_device* pdev)
{
    pdev->resource = &mn_mipi_res_list[0];
    pdev->num_resources = ARRAYSIZE(mn_mipi_res_list);
    pdev->dev.of_node.properties = (property *)&mn_mipi_properties;
    pdev->dev.of_clk = (struct clk_init_data_desc *)&mn_mipi_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8mn;
}

static inline void lcdif_res_init(platform_device* pdev)
{
    pdev->resource = &mn_lcdif_res_list[0];
    pdev->num_resources = ARRAYSIZE(mn_lcdif_res_list);
    pdev->dev.of_node.properties = (property *) &mn_lcdif_properties;
    pdev->dev.of_clk = (struct clk_init_data_desc *)&mn_lcdif_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8mn;
}

void mn_board_init(platform_device* pdev)
{
    NT_ASSERT(pdev);

    if (!strcmp(pdev->name, "lcdif_dev"))
    {
        lcdif_res_init(pdev);
        lcdif_irq_init();
    }
    /* Note: mipi_dsi_adv7535 is test device */
    else if ((!strcmp(pdev->name, "mipi_dsi_dev")) || (!strcmp(pdev->name, "mipi_dsi_adv7535")))
    {
        mipi_res_init(pdev);
        //mipi_irq_init(pdev);
    }
}

void mn_board_deinit(platform_device* pdev)
{
    NT_ASSERT(pdev);

}

} // extern "C"
