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

#include <ntddk.h>
#include <stddef.h>
#include "clk_imx8m_registers.h"
#include "clk_imx8m_pll.h"
#include "clk_imx8mq.h"
#include "linux/dev_printk.h"

/* #define IMX_CLK_DEBUG_DUMP_WHOLE_CLK_TREE */

#define IMX_CLK_GATE_DOMAIN_ALL_MODES 3

/* Clock Root Selects for clock root offsets and muxing (Clock Root Table in RM) */
#define IMX8MQ_DISPLAY_AXI_CLK_ROOT               20U
#define IMX8MQ_DISPLAY_APB_CLK_ROOT               21U
#define IMX8MQ_DISPLAY_RTRM_CLK_ROOT              22U
#define IMX8MQ_DISPLAY_DTRC_CLK_ROOT              68U

/* CCGRx device clock gate mapping */
#define IMX8MQ_CCM_CCGR                           93U

/* Main object of the clock tree - single instance for all monitors*/
static imx8mq_clk_device_t clktree;

static unsigned share_display_clockgate = 0; /* Represents shared CCGR93 LPCG clock gate */

/* --------------------------- Clock slice mux initialization data --------------------------------*/

static struct clk_root_mux_data
imx8mq_disp_axi_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_25M_REF, IMX8MQ_CLK_25M},
    {IMX_SYS_PLL2_DIV8, IMX8MQ_SYS_PLL2_DIV8},
    {IMX_SYS_PLL1, IMX8MQ_SYS_PLL1},
    {IMX_SYS_PLL3, IMX8MQ_SYS_PLL3},
    {IMX_SYS_PLL1_DIV20, IMX8MQ_SYS_PLL1_DIV20},
    {IMX_AUDIO_PLL2, IMX8MQ_AUDIO_PLL2},
    {IMX_EXT1, IMX8MQ_CLK_EXT1},
    {IMX_EXT4, IMX8MQ_CLK_EXT4}
};

static struct clk_root_mux_data
imx8mq_disp_apb_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_25M_REF, IMX8MQ_CLK_25M},
    {IMX_SYS_PLL2_DIV8, IMX8MQ_SYS_PLL2_DIV8},
    {IMX_SYS_PLL1, IMX8MQ_SYS_PLL1},
    {IMX_SYS_PLL3, IMX8MQ_SYS_PLL3},
    {IMX_SYS_PLL1_DIV20, IMX8MQ_SYS_PLL1_DIV20},
    {IMX_AUDIO_PLL2, IMX8MQ_AUDIO_PLL2},
    {IMX_EXT1, IMX8MQ_CLK_EXT1},
    {IMX_EXT3, IMX8MQ_CLK_EXT3}
};

static struct clk_root_mux_data
imx8mq_disp_rtrm_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_25M_REF, IMX8MQ_CLK_25M},
    {IMX_SYS_PLL1, IMX8MQ_SYS_PLL1},
    {IMX_SYS_PLL2_DIV5, IMX8MQ_SYS_PLL2_DIV5},
    {IMX_SYS_PLL1_DIV2, IMX8MQ_SYS_PLL1_DIV2},
    {IMX_AUDIO_PLL1, IMX8MQ_AUDIO_PLL1},
    {IMX_VIDEO_PLL1, IMX8MQ_VIDEO_PLL1},
    {IMX_EXT2, IMX8MQ_CLK_EXT2},
    {IMX_EXT3, IMX8MQ_CLK_EXT3}
};

static struct clk_root_mux_data
imx8mq_disp_dtrc_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_25M_REF, IMX8MQ_CLK_25M},
    {IMX_VIDEO_PLL2, IMX8MQ_VIDEO_PLL2},
    {IMX_SYS_PLL1, IMX8MQ_SYS_PLL1},
    {IMX_SYS_PLL2, IMX8MQ_SYS_PLL2},
    {IMX_SYS_PLL1_DIV5, IMX8MQ_SYS_PLL1_DIV5},
    {IMX_VIDEO_PLL1, IMX8MQ_VIDEO_PLL1},
    {IMX_SYS_PLL3, IMX8MQ_SYS_PLL3},
    {IMX_AUDIO_PLL2, IMX8MQ_AUDIO_PLL2}
};

static struct clk_root_mux imx8mq_disp_axi_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mq_disp_apb_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mq_disp_rtrm_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mq_disp_dtrc_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];

/**
 * Initalize clock slice selector structure.
 *
 * @param  dev              device with the clock tree
 * @param  clk_sels_data    structure prefilled with parent ids of the clock slice
 * @param  clk_sels         clock slice object to be initialized
 */
static void fill_clk_sels(const imx8mq_clk_device_t *dev,
    const struct clk_root_mux_data *clk_sels_data, struct clk_root_mux *clk_sels)
{
    NT_ASSERT(dev);

    for (int i = 0; i < IMX_CLK_ROOT_MUX_MAX_PARENTS; i++) {
        clk_sels[i].parent_id = clk_sels_data[i].parent_id;
        clk_sels[i].parent_clock = dev->clks[clk_sels_data[i].parent_clock];
    }
}

/**
 * Check integrity of the clock tree.
 *
 * @param  dev              device with the clock tree
 * @param  cnt              number of clock tree nodes (typically IMX8_DC_CLK_END)
 * @return 0 on success, -1 on fail
 */
static int imx_check_clks(const imx8mq_clk_device_t *dev, int cnt)
{
    NT_ASSERT(dev);

    for (int i = 0; i < cnt; i++) {
        if (dev->clks[i] == NULL) {
            dev_err(NULL, "Clock #%d initialization failed\n", i);
            return -1;
        }
    }
    return 0;
}

/**
 * Get specific clock node according to index.
 *
 * @param  index index of clock node. See "Clock tree items - START" table
 * @return 0 on success, -1 on fail
 */
imx_clk_t *clk_get_item_imx8mq(int index)
{
    if (index < IMX8MQ_DC_CLK_END) {
        return clktree.clks[index];
    }
    return NULL;
}

/* --------------------------- High-level clock configuration functions --------------------------------*/

/**
 * Initializes clock tree. Call for each display interface, so shared mode can be addressed accordingly
 *
 * @param  dev device for which initialize the clock tree.
 * @return NULL on fail, pointer to clktree on success.
 */
imx8mq_clk_device_t *clk_init_imx8mq()
{
    imx8mq_clk_device_t *dev = &clktree;

    if (dev == NULL) {
        return NULL;
    }

    dev->ccm_regptr = ioremap(IMX_CCM_BASE, IMX_CCM_SIZE);
    if (dev->ccm_regptr == NULL) {
        dev_err(NULL, "CLK: mmap of CCM regs failed!\n");
        return NULL;
    }
    dev->ccm_analog_regptr = ioremap(IMX_CCM_ANALOG_BASE, IMX_CCM_ANALOG_SIZE);
    if (dev->ccm_analog_regptr == NULL) {
        dev_err(NULL, "CLK: mmap of CCM ANALOG regs failed!\n");
        return NULL;
    }

    /* clock sources */
    dev->clks[IMX8MQ_CLK_DUMMY] = imx_clk_fixed("dummy", NULL, IMX_CLK_FREQ_0K);
    dev->clks[IMX8MQ_CLK_25M] = imx_clk_fixed("osc_25m", NULL, IMX_CLK_FREQ_25M);
    dev->clks[IMX8MQ_CLK_27M] = imx_clk_fixed("osc_27m", NULL, IMX_CLK_FREQ_27M);
    dev->clks[IMX8MQ_CLK_32K] = imx_clk_fixed("osc_32k", NULL, IMX_CLK_FREQ_32K);
    dev->clks[IMX8MQ_CLK_EXT1] = imx_clk_fixed("clk_ext1", NULL, IMX_CLK_FREQ_0K);
    dev->clks[IMX8MQ_CLK_EXT2] = imx_clk_fixed("clk_ext2", NULL, IMX_CLK_FREQ_0K);
    dev->clks[IMX8MQ_CLK_EXT3] = imx_clk_fixed("clk_ext3", NULL, IMX_CLK_FREQ_0K);
    dev->clks[IMX8MQ_CLK_EXT4] = imx_clk_fixed("clk_ext4", NULL, IMX_CLK_FREQ_0K);
    /* PLLs with fixed frequencies configured in IPL/BSP startup */
    dev->clks[IMX8MQ_SYS_PLL1] = imx_clk_fixed("sys_pll1", NULL, IMX_CLK_FREQ_800M);
    dev->clks[IMX8MQ_SYS_PLL1_DIV2] = imx_clk_fixed("sys_pll1_div2", NULL, IMX_CLK_FREQ_400M);
    dev->clks[IMX8MQ_SYS_PLL1_DIV3] = imx_clk_fixed("sys_pll1_div3", NULL, IMX_CLK_FREQ_266_6M);
    dev->clks[IMX8MQ_SYS_PLL1_DIV4] = imx_clk_fixed("sys_pll1_div4", NULL, IMX_CLK_FREQ_200M);
    dev->clks[IMX8MQ_SYS_PLL1_DIV5] = imx_clk_fixed("sys_pll1_div5", NULL, IMX_CLK_FREQ_160M);
    dev->clks[IMX8MQ_SYS_PLL1_DIV6] = imx_clk_fixed("sys_pll1_div6", NULL, IMX_CLK_FREQ_133_3M);
    dev->clks[IMX8MQ_SYS_PLL1_DIV8] = imx_clk_fixed("sys_pll1_div8", NULL, IMX_CLK_FREQ_100M);
    dev->clks[IMX8MQ_SYS_PLL1_DIV10] = imx_clk_fixed("sys_pll1_div10", NULL, IMX_CLK_FREQ_80M);
    dev->clks[IMX8MQ_SYS_PLL1_DIV20] = imx_clk_fixed("sys_pll1_div20", NULL, IMX_CLK_FREQ_40M);
    dev->clks[IMX8MQ_SYS_PLL2] = imx_clk_fixed("sys_pll2", NULL, IMX_CLK_FREQ_1000M);
    dev->clks[IMX8MQ_SYS_PLL2_DIV2] = imx_clk_fixed("sys_pll2_div2", NULL, IMX_CLK_FREQ_500M);
    dev->clks[IMX8MQ_SYS_PLL2_DIV3] = imx_clk_fixed("sys_pll2_div3", NULL, IMX_CLK_FREQ_333_3M);
    dev->clks[IMX8MQ_SYS_PLL2_DIV4] = imx_clk_fixed("sys_pll2_div4", NULL, IMX_CLK_FREQ_250M);
    dev->clks[IMX8MQ_SYS_PLL2_DIV5] = imx_clk_fixed("sys_pll2_div5", NULL, IMX_CLK_FREQ_200M);
    dev->clks[IMX8MQ_SYS_PLL2_DIV6] = imx_clk_fixed("sys_pll2_div6", NULL, IMX_CLK_FREQ_166_6M);
    dev->clks[IMX8MQ_SYS_PLL2_DIV8] = imx_clk_fixed("sys_pll2_div8", NULL, IMX_CLK_FREQ_125M);
    dev->clks[IMX8MQ_SYS_PLL2_DIV10] = imx_clk_fixed("sys_pll2_div10", NULL, IMX_CLK_FREQ_100M);
    dev->clks[IMX8MQ_SYS_PLL2_DIV20] = imx_clk_fixed("sys_pll2_div20", NULL, IMX_CLK_FREQ_50M);
    dev->clks[IMX8MQ_SYS_PLL3] = imx_clk_fixed("sys_pll3", NULL, IMX_CLK_FREQ_750M);
    dev->clks[IMX8MQ_AUDIO_PLL1] = imx_clk_fixed("audio_pll1", NULL, IMX_CLK_FREQ_786_4M);
    dev->clks[IMX8MQ_AUDIO_PLL2] = imx_clk_fixed("audio_pll2", NULL, IMX_CLK_FREQ_722_5M);
    /* Current implementation configures VIDEO PLLs as fixed sources */
    dev->clks[IMX8MQ_VIDEO_PLL1] = imx_clk_fixed("video_pll1", NULL, IMX_CLK_FREQ_650M);
    dev->clks[IMX8MQ_VIDEO_PLL2] = imx_clk_fixed("video_pll2", NULL, IMX_CLK_FREQ_600M);

    /* fill cock sources for CCM slices */
    fill_clk_sels(dev, imx8mq_disp_axi_sels_data, imx8mq_disp_axi_sels);
    fill_clk_sels(dev, imx8mq_disp_apb_sels_data, imx8mq_disp_apb_sels);
    fill_clk_sels(dev, imx8mq_disp_rtrm_sels_data, imx8mq_disp_rtrm_sels);
    fill_clk_sels(dev, imx8mq_disp_dtrc_sels_data, imx8mq_disp_dtrc_sels);

    /* clock slices in CCM */
    dev->clks[IMX8MQ_CLK_DISP_AXI] = imx_clk_slice("disp_axi",
        dev->clks[IMX8MQ_CLK_25M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MQ_DISPLAY_AXI_CLK_ROOT),
        imx8mq_disp_axi_sels, NULL);
    dev->clks[IMX8MQ_CLK_DISP_APB] = imx_clk_slice("disp_apb",
        dev->clks[IMX8MQ_CLK_25M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MQ_DISPLAY_APB_CLK_ROOT),
        imx8mq_disp_apb_sels, NULL);
    dev->clks[IMX8MQ_CLK_DISP_RTRM] = imx_clk_slice("disp_rtrm",
        dev->clks[IMX8MQ_CLK_25M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MQ_DISPLAY_RTRM_CLK_ROOT),
        imx8mq_disp_rtrm_sels, NULL);
    dev->clks[IMX8MQ_CLK_DISP_DTRC] = imx_clk_slice("disp_dtrc",
        dev->clks[IMX8MQ_CLK_25M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MQ_DISPLAY_DTRC_CLK_ROOT),
        imx8mq_disp_dtrc_sels, NULL);

    /* clock gate in LPCG Note: only one LPCG shared among all clocks */
    dev->clks[IMX8MQ_CLK_DISP_AXI_ROOT] = imx_clk_gate(
        "disp_axi_root", dev->clks[IMX8MQ_CLK_DISP_AXI],
        dev->ccm_regptr, IMX_CCM_CCGRn(IMX8MQ_CCM_CCGR), IMX_CCM_BASE,
        0, IMX_CLK_GATE_DOMAIN_ALL_MODES, 3, &share_display_clockgate);
    dev->clks[IMX8MQ_CLK_DISP_APB_ROOT] = imx_clk_gate(
        "disp_apb_root", dev->clks[IMX8MQ_CLK_DISP_APB],
        dev->ccm_regptr, IMX_CCM_CCGRn(IMX8MQ_CCM_CCGR), IMX_CCM_BASE,
        0, IMX_CLK_GATE_DOMAIN_ALL_MODES, 3, &share_display_clockgate);
    dev->clks[IMX8MQ_CLK_DISP_RTRM_ROOT] = imx_clk_gate(
        "disp_rtrm_root", dev->clks[IMX8MQ_CLK_DISP_RTRM],
        dev->ccm_regptr, IMX_CCM_CCGRn(IMX8MQ_CCM_CCGR), IMX_CCM_BASE,
        0, IMX_CLK_GATE_DOMAIN_ALL_MODES, 3, &share_display_clockgate);

    if (imx_check_clks(dev, IMX8MQ_DC_CLK_END)) {
        return NULL;
    }

    return dev;
}

void clk_dump_clock_tree_imx8mq(const imx8mq_clk_device_t *dev)
{
    if (dev == NULL) {
        return;
    }

    for (int i = 0; i < IMX8MQ_DC_CLK_END; i++) {
        imx_clk_dump(dev->clks[i]);
    }
}

/**
 * De-initialize clock tree.
 *
 * @param  dev device to de-init clock tree for
 *
 * @return 0 on success. Returns -1 on failure.
 */

int clk_deinit_imx8mq(imx8mq_clk_device_t *dev)
{
    if (dev == NULL) {
        return -1;
    }

    for (int i = 0; i < IMX8MQ_DC_CLK_END; i++) {
        if (dev->clks[i] != NULL) {
            imx_clk_uninit(dev->clks[i]);
        }
    }
    if (dev->ccm_regptr != 0) {
        iounmap(dev->ccm_regptr, IMX_CCM_SIZE);
    }
    if (dev->ccm_analog_regptr != 0) {
        iounmap(dev->ccm_analog_regptr, IMX_CCM_ANALOG_SIZE);
    }

    return 0;
}
