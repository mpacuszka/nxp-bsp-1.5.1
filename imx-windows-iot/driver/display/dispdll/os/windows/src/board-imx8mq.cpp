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
#include "clk\clk_imx8mq.h"
#include "linux\of.h"
#include "linux\platform_device.h"
#include "linux\irqchip\irq-imx-irqsteer.h"
#include "kernel\irq\internals.h"

#define DCSS_NR_IRQS 5
#define HDMI_NR_IRQS 2

static int dcss_interrupts[DCSS_NR_IRQS] = { 6, 8, 9, 16, 17 };
static char *dcss_interrupt_names[DCSS_NR_IRQS] = {
    "ctxld", "ctxld_kick", "vblank", "dtrc_ch1", "dtrc_ch2" };

static resource dcss_res_list[] = {
    { 0x32E00000, 0x32E2D000, "dcss", IORESOURCE_MEM },
    { 0x32E2F000, 0x32E30000, "dcss-blk-ctl", IORESOURCE_MEM },
};

/* We do not include pixel clock here because
* it is not handled by the clock driver.
* We use HDMI PHY to generate clock for the DCSS.
* So clock is controlled by the mhdp phy driver.
*/
static struct clk_init_data_desc dcss_clock_desc[] = {
    { "apb", IMX8MQ_CLK_DISP_APB, 0, 0, 0 },
    { "axi", IMX8MQ_CLK_DISP_AXI, IMX8MQ_SYS_PLL1, IMX_CLK_FREQ_800M, 1 },
    { "rtrm", IMX8MQ_CLK_DISP_RTRM, IMX8MQ_SYS_PLL1, IMX_CLK_FREQ_400M, 1 },
    { "dtrc", IMX8MQ_CLK_DISP_DTRC, 0, 0, 0 },
    {""} // marks end of the list
};

static resource hdmi_res_list[] = {
    { 0x32C00000, 0x32D00000, "hdmi-ctrl", IORESOURCE_MEM },
    { 0x32E40000, 0x32E80000, "hdmi-sec", IORESOURCE_MEM },
};

static int lane_mapping = 0xe4;
static int hdcp_config = 0x3;

static property hdmi_properties[] = {
    { "lane-mapping", 1, &lane_mapping },
    { "hdcp-config", 1, &hdcp_config },
    { "" } // marks end of the list
};

static resource irqsteer_res_list[] = {
    { 0x32E2D000, 0x32E2E000, "irqsteer", IORESOURCE_MEM },
};

static int channel = 0;
static int num_irqs = 64;

static property irqsteer_properties[] = {
    { "fsl,channel", 1, &channel },
    { "fsl,num-irqs", 1, &num_irqs },
    { "" } // marks end of the list
};

static void dcss_irq_init(void)
{
    for (int i = 0; i < DCSS_NR_IRQS; i++)
    {
        irq_desc[i].name = dcss_interrupt_names[i];
        irq_desc[i].irq_data.hwirq = dcss_interrupts[i];
        irq_desc[i].handler = nullptr;
        irq_desc[i].irq_data.chip.type = IRQCHIP_TYPE_IRQ_STEER;
        irq_state_set_disabled(&irq_desc[i]);
        irq_state_set_masked(&irq_desc[i]);
    }
}

static int hdmi_irq_init(void)
{
    return 0;
}

static inline void dcss_res_init(platform_device* pdev)
{
    pdev->resource = &dcss_res_list[0];
    pdev->num_resources = ARRAYSIZE(dcss_res_list);
    pdev->dev.of_clk = (struct clk_init_data_desc *)&dcss_clock_desc;
    pdev->dev.get_clock_item = &clk_get_item_imx8mq;
}

static inline void hdmi_res_init(platform_device* pdev)
{
    pdev->resource = &hdmi_res_list[0];
    pdev->num_resources = ARRAYSIZE(hdmi_res_list);
    pdev->dev.of_node.properties = hdmi_properties;
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

void mq_board_init(platform_device* pdev)
{
    NT_ASSERT(pdev);

    if (!strcmp(pdev->name, "dcss"))
    {
        dcss_res_init(pdev);
        dcss_irq_init();
    }
    else if (!strcmp(pdev->name, "hdmi"))
    {
        hdmi_res_init(pdev);
        hdmi_irq_init();
    }
    else if (!strcmp(pdev->name, "irqsteer"))
    {
        irqsteer_res_init(pdev);
        imx_irqsteer_probe(pdev);
    }
}

void mq_board_deinit(platform_device* pdev)
{
    NT_ASSERT(pdev);

    if (!strcmp(pdev->name, "dcss"))
    {
        //dcss_irq_deinit();
        //dcss_res_deinit(pdev);
    }
    else if (!strcmp(pdev->name, "hdmi"))
    {
        //hdmi_irq_deinit();
        //hdmi_res_deinit(pdev);
    }
    else if (!strcmp(pdev->name, "irqsteer"))
    {
        imx_irqsteer_remove(pdev);
        irqsteer_res_deinit(pdev);
    }
}

} // extern "C"
