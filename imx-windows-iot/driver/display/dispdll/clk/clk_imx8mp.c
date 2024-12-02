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

#include <ntddk.h>
#include <stddef.h>
#include "clk_imx8m_registers.h"
#include "clk_imx8m_pll.h"
#include "clk_imx8mp.h"
#include "linux/clk.h"
#include "linux/slab.h"
#include "bridge/synopsys/dw_hdmi.h"

/* #define IMX_CLK_DEBUG_DUMP_WHOLE_CLK_TREE */

#define CLK_PRINTK(...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, __VA_ARGS__)

#ifdef IMX_CLK_DEBUG
#define CLK_DEBUG(...) CLK_PRINTK(__VA_ARGS__)
#else
#define CLK_DEBUG(...)
#endif

#define CHECK_SUCCESS(e) do { if ((e) != 0) { \
                               CLK_PRINTK("Display CLK: CHECK_SUCCESS failed."); \
                               return -1; \
                           } \
                      } while (0);

#define IMX_CLK_GATE_DOMAIN_ALL_MODES 3

/* Clock Root Selects for clock root offsets and muxing (Clock Root Table in RM) */
#define IMX8MP_MEDIA_AXI_CLK_ROOT               20U
#define IMX8MP_MEDIA_APB_CLK_ROOT               21U
#define IMX8MP_MEDIA_DISP2_PIX_CLK_ROOT         38U
#define IMX8MP_MEDIA_MIPI_PHY1_REF_CLK_ROOT     123U
#define IMX8MP_MEDIA_DISP1_PIX_CLK_ROOT         124U
#define IMX8MP_MEDIA_LDB_CLK_ROOT               126U
/* hdmi clock roots */
#define IMX8MP_HDMI_APB_CLK_ROOT                22U
#define IMX8MP_HDMI_AXI_CLK_ROOT                23U
#define IMX8MP_HDMI_24M_CLK_ROOT                119U
#define IMX8MP_HDMI_REF_266M_CLK_ROOT           120U

/* CCGRx device clock gate mapping */
#define IMX8MP_CCM_CCGR_MEDIA                   93U
#define IMX8MP_CCM_CCGR_HDMI                    95U

/* Main object of the clock tree - single instance for all monitors*/
static imx8mp_clk_device_t clktree = {
    .disp_lvds = imx_interface_undefined,
    .disp_mipi = imx_interface_undefined,
    .disp_hdmi = imx_interface_undefined
};

static unsigned share_display_clockgate = 0; /* Represents shared CCGR93 LPCG clock gate */
static unsigned share_apb_slice = 0; /* Shared enable/disable of IMX8MP_CLK_MEDIA_APB */
static unsigned share_axi_slice = 0; /* Shared enable/disable of IMX8MP_CLK_MEDIA_AXI */
static unsigned share_video_pll1 = 0; /* Shared Video_pll */
static unsigned share_hdmi_apb_slice = 0; /* Shared HDMI_APB */
static unsigned share_hdmi_xtal24_blkctrl = 0; /* Shared HDMI_GLOBAL_XTAL24_CLK */

/* --------------------------- Clock slice mux initialization data --------------------------------*/

static struct clk_root_mux_data
imx8mp_media_axi_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_24M_REF, IMX8MP_CLK_24M},
    {IMX_SYS_PLL2, IMX8MP_SYS_PLL2},
    {IMX_SYS_PLL1, IMX8MP_SYS_PLL1},
    {IMX_SYS_PLL3, IMX8MP_SYS_PLL3},
    {IMX_SYS_PLL1_DIV20, IMX8MP_SYS_PLL1_DIV20},
    {IMX_AUDIO_PLL2, IMX8MP_AUDIO_PLL2},
    {IMX_EXT1, IMX8MP_CLK_EXT1},
    {IMX_SYS_PLL2_DIV2, IMX8MP_SYS_PLL2_DIV2}
};

static struct clk_root_mux_data
imx8mp_media_apb_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_24M_REF, IMX8MP_CLK_24M},
    {IMX_SYS_PLL2_DIV8, IMX8MP_SYS_PLL2_DIV8},
    {IMX_SYS_PLL1, IMX8MP_SYS_PLL1},
    {IMX_SYS_PLL3, IMX8MP_SYS_PLL3},
    {IMX_SYS_PLL1_DIV20, IMX8MP_SYS_PLL1_DIV20},
    {IMX_AUDIO_PLL2, IMX8MP_AUDIO_PLL2},
    {IMX_EXT1, IMX8MP_CLK_EXT1},
    {IMX_SYS_PLL1_DIV6, IMX8MP_SYS_PLL1_DIV6}
};

static struct clk_root_mux_data
imx8mp_media_disp1_pix_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_24M_REF, IMX8MP_CLK_24M},
    {IMX_VIDEO_PLL, IMX8MP_VIDEO_PLL1},
    {IMX_AUDIO_PLL2, IMX8MP_AUDIO_PLL2},
    {IMX_AUDIO_PLL1, IMX8MP_AUDIO_PLL1},
    {IMX_SYS_PLL1, IMX8MP_SYS_PLL1},
    {IMX_SYS_PLL2, IMX8MP_SYS_PLL2},
    {IMX_SYS_PLL3, IMX8MP_SYS_PLL3},
    {IMX_EXT4, IMX8MP_CLK_EXT4}
};

static struct clk_root_mux_data
imx8mp_media_disp2_pix_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_24M_REF, IMX8MP_CLK_24M},
    {IMX_VIDEO_PLL, IMX8MP_VIDEO_PLL1},
    {IMX_AUDIO_PLL2, IMX8MP_AUDIO_PLL2},
    {IMX_AUDIO_PLL1, IMX8MP_AUDIO_PLL1},
    {IMX_SYS_PLL1, IMX8MP_SYS_PLL1},
    {IMX_SYS_PLL2, IMX8MP_SYS_PLL2},
    {IMX_SYS_PLL3, IMX8MP_SYS_PLL3},
    {IMX_EXT4, IMX8MP_CLK_EXT4}
};

static struct clk_root_mux_data
imx8mp_media_mipi_phy1_ref_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_24M_REF, IMX8MP_CLK_24M},
    {IMX_SYS_PLL2_DIV3, IMX8MP_SYS_PLL2_DIV3},
    {IMX_SYS_PLL2_DIV10, IMX8MP_SYS_PLL2_DIV10},
    {IMX_SYS_PLL1, IMX8MP_SYS_PLL1},
    {IMX_SYS_PLL2, IMX8MP_SYS_PLL2},
    {IMX_EXT2, IMX8MP_CLK_EXT2},
    {IMX_AUDIO_PLL2, IMX8MP_AUDIO_PLL2},
    {IMX_VIDEO_PLL, IMX8MP_VIDEO_PLL1}
};

static struct clk_root_mux_data
imx8mp_media_ldb_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_24M_REF, IMX8MP_CLK_24M},
    {IMX_SYS_PLL2_DIV3, IMX8MP_SYS_PLL2_DIV3},
    {IMX_SYS_PLL2_DIV10, IMX8MP_SYS_PLL2_DIV10},
    {IMX_SYS_PLL1, IMX8MP_SYS_PLL1},
    {IMX_SYS_PLL2, IMX8MP_SYS_PLL2},
    {IMX_EXT2, IMX8MP_CLK_EXT2},
    {IMX_AUDIO_PLL2, IMX8MP_AUDIO_PLL2},
    {IMX_VIDEO_PLL, IMX8MP_VIDEO_PLL1}
};

static struct clk_root_mux_data
imx8mp_hdmi_ref_266m_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_24M_REF, IMX8MP_CLK_24M},
    {IMX_SYS_PLL1_DIV2, IMX8MP_SYS_PLL1_DIV2},
    {IMX_SYS_PLL3, IMX8MP_SYS_PLL3},
    {IMX_SYS_PLL2_DIV3, IMX8MP_SYS_PLL2_DIV3},
    {IMX_SYS_PLL1_DIV3, IMX8MP_SYS_PLL1_DIV3},
    {IMX_SYS_PLL2_DIV5, IMX8MP_SYS_PLL2_DIV5},
    {IMX_AUDIO_PLL2, IMX8MP_AUDIO_PLL2},
    {IMX_VIDEO_PLL, IMX8MP_VIDEO_PLL1}
};

static struct clk_root_mux_data
imx8mp_hdmi_24m_sels_data[IMX_CLK_ROOT_MUX_MAX_PARENTS] = {
    {IMX_24M_REF, IMX8MP_CLK_24M},
    {IMX_SYS_PLL1_DIV5, IMX8MP_SYS_PLL1_DIV5},
    {IMX_SYS_PLL2_DIV20, IMX8MP_SYS_PLL2_DIV20},
    {IMX_SYS_PLL3, IMX8MP_SYS_PLL3},
    {IMX_AUDIO_PLL1, IMX8MP_AUDIO_PLL1},
    {IMX_VIDEO_PLL, IMX8MP_VIDEO_PLL1},
    {IMX_AUDIO_PLL2, IMX8MP_AUDIO_PLL2},
    {IMX_SYS_PLL1_DIV6, IMX8MP_SYS_PLL1_DIV6}
};

static struct clk_root_mux imx8mp_media_axi_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mp_media_apb_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mp_media_disp1_pix_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mp_media_disp2_pix_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mp_media_mipi_phy1_ref_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mp_media_ldb_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];

static struct clk_root_mux imx8mp_hdmi_axi_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mp_hdmi_apb_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mp_hdmi_ref_266m_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];
static struct clk_root_mux imx8mp_hdmi_24m_sels[IMX_CLK_ROOT_MUX_MAX_PARENTS];

/**
 * Initalize clock slice selector structure.
 *
 * @param  dev              device with the clock tree
 * @param  clk_sels_data    structure prefilled with parent ids of the clock slice
 * @param  clk_sels         clock slice object to be pre=filled
 */
static void fill_clk_sels(const imx8mp_clk_device_t *dev,
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
static int imx_check_clks(const imx8mp_clk_device_t *dev, int cnt)
{
    NT_ASSERT(dev);

    for (int i = 0; i < cnt; i++) {
        if (dev->clks[i] == NULL) {
            CLK_PRINTK("Display CLK: Clock #%d initialization failed", i);
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
imx_clk_t *clk_get_item_imx8mp(int index)
{
    if (index < IMX8MP_DC_CLK_END) {
        return clktree.clks[index];
    }
    return NULL;
}

/* HDMI PHY clock access functions */
static unsigned long clk_hdmi_phy_get_rate_imx8mp(imx_clk_t* clk)
{
    long rate;

    if (clk == NULL) {
        return 0;
    }
    rate = samsung_hdmi_phy_clk_round_rate(clk->clk_pll.rate_index);
    if (rate < 0) {
        return 0;
    }
    return (unsigned long)rate;

}

unsigned long clk_hdmi_phy_set_rate_imx8mp(imx_clk_t* clk, unsigned long rate)
{
    struct platform_device* pdev;
    int ret;

    if (clk == NULL) {
        return 0;
    }
    pdev = (struct platform_device*)clk->clk_pll.reg;
    ret = samsung_hdmi_phy_clk_set_rate(pdev, rate);
    if (ret) {
        return 0;
    }
    clk->clk_pll.rate_index = rate;
    return rate;
}

unsigned long clk_hdmi_phy_get_ref_rate_imx8mp(imx_clk_t* clk)
{
    return (unsigned long)clk->clk_pll.ref_rate_hz;
}

/**
 * Specific function to initialize HDMI PHY PLL clock. Uses struct imx_clk_pll clk_pll;
 *
 * @param  name       clock name
 * @param  parent     parent of the clock in the clock tree
 * @param  reg        register base address of the CCM
 * @param  offset      offset of the selected PLL block
 * @param  pdev pointer to the platfor_device object of hdmi phy driver
 *
 * @return clk structure for given name and parameters on success. NULL on failure.
 */
imx_clk_t* clk_hdmi_phy_imx8mp(const char* name, imx_clk_t* parent, struct platform_device* pdev)
{
    imx_clk_t* clk = kmalloc(sizeof(imx_clk_t), __GFP_ZERO);

    if (!clk) {
        CLK_PRINTK("Display CLK: Could not allocate clock structure\n");
        return NULL;
    }

    clk->type = CLK_CUSTOM;
    clk->name = name;
    clk->clk_pll.reg = (uint8_t*)pdev; /* Store hdmi phy platform device as reg pointer */
    clk->clk_pll.rate_index = -1;
    clk->clk_pll.ref_rate_hz = imx_clk_get_rate(parent);
    clk->parent = parent;
    clk->clk_enable = NULL;
    clk->clk_get_rate = &clk_hdmi_phy_get_rate_imx8mp;
    clk->clk_set_rate = &clk_hdmi_phy_set_rate_imx8mp;
    clk->clk_set_parent = NULL;
    clk->clk_get_parent = NULL;
    clk->clk_get_reference_rate = &clk_hdmi_phy_get_ref_rate_imx8mp;
    clk->flags = 0;

    return clk;
}

/* --------------------------- High-level clock configuration functions --------------------------------*/
/**
 * Start selected clocks. This functions is called repeatedly for every started disp interface
 *
 * @param  dev device to start the clock for
 * @param  new_disp display interface to start clocks for
 *
 * @return 0 on success. Returns -1 on failure.
 */

static int clk_start_imx8mp(imx8mp_clk_device_t *dev, imx_display_interface new_disp)
{
    if (dev == NULL) {
        return -1;
    }

    switch (new_disp) {
    case imx_lvds:
        mediamix_lvds_reset(dev->mediamix_regptr, 0);
        imx_clk_enable(dev->clks[IMX8MP_CLK_MEDIA_BUS_BLK], true, false);
        break;
    case imx_mipi_dsi:
        mediamix_dsi_reset(dev->mediamix_regptr, 0);
        imx_clk_enable(dev->clks[IMX8MP_CLK_MEDIA_BUS_BLK], true, false);
        break;
    case imx_hdmi:
        imx_clk_enable(dev->clks[IMX8MP_CLK_HDMI_ROOT], true, false);
        imx_clk_set_rate(dev->clks[IMX8MP_CLK_HDMI_REF_266M], IMX_CLK_FREQ_24M);
        imx_clk_enable(dev->clks[IMX8MP_CLK_HDMI_REF_266M], true, false);
        break;
    default:
        CLK_PRINTK("Display CLK: unsupported interface %d!", new_disp);
        break;
    }

    return 0;
}

/**
 * Initializes clock tree. Call for each display interface, so shared mode can be addressed accordingly
 *
 * @param  dev device for which initialize the clock tree.
 * @return NULL on fail, poiter to clktree on success.
 */
imx8mp_clk_device_t *clk_init_imx8mp(imx_display_interface new_disp, bool shared)
{
    const struct imx_pll_rate_table *rate_table;
    imx8mp_clk_device_t *dev = &clktree;
    imx_display_interface last_disp_lvds = dev->disp_lvds;
    imx_display_interface last_disp_mipi = dev->disp_mipi;
    imx_display_interface last_disp_hdmi = dev->disp_hdmi;

    if (dev == NULL) {
        return NULL;
    }

    switch (new_disp) {
    case imx_lvds:
        rate_table = videopll_tab_24m_lvds;
        dev->disp_lvds = new_disp;
        break;
    case imx_mipi_dsi:
        rate_table = shared ? videopll_tab_24m_lvds : videopll_tab_24m_mipi;
        dev->disp_mipi = new_disp;
        break;
    case imx_hdmi:
        /* rate_table not important, but need to set something */
        rate_table = videopll_tab_24m_lvds;
        dev->disp_hdmi = new_disp;
        break;
    default:
        CLK_PRINTK("Display CLK: unsupported interface %d!", new_disp);
        return NULL;
    }

    if ((last_disp_lvds != imx_interface_undefined) || (last_disp_mipi != imx_interface_undefined) || (last_disp_hdmi != imx_interface_undefined)) {
        /* One of the display interface has already been initialized - static clock tree is ready */
        if ((last_disp_hdmi != imx_interface_undefined) && (last_disp_lvds == imx_interface_undefined) && (last_disp_mipi == imx_interface_undefined)) {
            /* Only HDMI interface has been initialized before. We can safely reset rate_table, because VIDEO_PLL is still not shared. */
            imx_clk_uninit(dev->clks[IMX8MP_VIDEO_PLL1]);
            dev->clks[IMX8MP_VIDEO_PLL1] = imx_clk_pll("video_pll1",
                dev->clks[IMX8MP_CLK_24M],
                dev->ccm_analog_regptr, IMX_CCM_ANALOG_VIDEO_PLL1_GEN_CTRL,
                rate_table);
            /* re-fill the selector table to refresh new pointer to IMX8MP_VIDEO_PLL1 clock */
            fill_clk_sels(dev, imx8mp_media_mipi_phy1_ref_sels_data,
                imx8mp_media_mipi_phy1_ref_sels);
            fill_clk_sels(dev, imx8mp_media_disp1_pix_sels_data,
                imx8mp_media_disp1_pix_sels);
            fill_clk_sels(dev, imx8mp_media_disp2_pix_sels_data,
                imx8mp_media_disp2_pix_sels);
            fill_clk_sels(dev, imx8mp_media_ldb_sels_data,
                imx8mp_media_ldb_sels);
        }

        if (((last_disp_lvds != imx_interface_undefined) || (last_disp_mipi != imx_interface_undefined)) && (new_disp != imx_hdmi)) {
            /* TODO: clock tree has already been initialized for either LVDS or MIPI_DSI. That means VIDEO_PLL becomes shared.
               initialize rate_table to values that can be used for both interfaces
               and then set dev->clks[IMX8MP_VIDEO_PLL1] with new rate_table */
        }
        clk_start_imx8mp(dev, new_disp);
        /* Don't initialize the rest of the clock tree again */
        return dev;
    }

    dev->ccm_regptr = ioremap(IMX_CCM_BASE, IMX_CCM_SIZE);
    if (dev->ccm_regptr == NULL) {
        CLK_PRINTK("Display CLK: mmap of CCM regs failed!");
        return NULL;
    }
    dev->ccm_analog_regptr = ioremap(IMX_CCM_ANALOG_BASE, IMX_CCM_ANALOG_SIZE);
    if (dev->ccm_analog_regptr == NULL) {
        CLK_PRINTK("Display CLK: mmap of CCM ANALOG regs failed!");
        return NULL;
    }
    dev->mediamix_regptr = ioremap(IMX_MEDIAMIX_BASE, IMX_MEDIAMIX_SIZE);
    if (dev->mediamix_regptr == NULL) {
        CLK_PRINTK("Display CLK: mmap of MEIDIAMIX regs failed!");
        return NULL;
    }
    dev->hdmi_blkctrl_regptr = ioremap(IMX_HDMIMIX_BASE, IMX_HDMIMIX_SIZE);
    if (dev->hdmi_blkctrl_regptr == NULL) {
        CLK_PRINTK("Display CLK: mmap of HDMIMIX regs failed!");
        return NULL;
    }

    /* clock sources */
    dev->clks[IMX8MP_CLK_DUMMY] = imx_clk_fixed("dummy", NULL, IMX_CLK_FREQ_0K);
    dev->clks[IMX8MP_CLK_24M] = imx_clk_fixed("osc_24m", NULL, IMX_CLK_FREQ_24M);
    dev->clks[IMX8MP_CLK_32K] = imx_clk_fixed("osc_32k", NULL, IMX_CLK_FREQ_32K);
    dev->clks[IMX8MP_CLK_EXT1] = imx_clk_fixed("clk_ext1", NULL, IMX_CLK_FREQ_0K);
    dev->clks[IMX8MP_CLK_EXT2] = imx_clk_fixed("clk_ext2", NULL, IMX_CLK_FREQ_0K);
    dev->clks[IMX8MP_CLK_EXT3] = imx_clk_fixed("clk_ext3", NULL, IMX_CLK_FREQ_0K);
    dev->clks[IMX8MP_CLK_EXT4] = imx_clk_fixed("clk_ext4", NULL, IMX_CLK_FREQ_0K);
    /* PLLs with fixed frequencies configured in IPL/BSP startup */
    dev->clks[IMX8MP_SYS_PLL1] = imx_clk_fixed("sys_pll1", NULL, IMX_CLK_FREQ_800M);
    dev->clks[IMX8MP_SYS_PLL1_DIV2] = imx_clk_fixed("sys_pll1_div2", NULL, IMX_CLK_FREQ_400M);
    dev->clks[IMX8MP_SYS_PLL1_DIV3] = imx_clk_fixed("sys_pll1_div3", NULL, IMX_CLK_FREQ_266_6M);
    dev->clks[IMX8MP_SYS_PLL1_DIV4] = imx_clk_fixed("sys_pll1_div4", NULL, IMX_CLK_FREQ_200M);
    dev->clks[IMX8MP_SYS_PLL1_DIV5] = imx_clk_fixed("sys_pll1_div5", NULL, IMX_CLK_FREQ_160M);
    dev->clks[IMX8MP_SYS_PLL1_DIV6] = imx_clk_fixed("sys_pll1_div6", NULL, IMX_CLK_FREQ_133_3M);
    dev->clks[IMX8MP_SYS_PLL1_DIV8] = imx_clk_fixed("sys_pll1_div8", NULL, IMX_CLK_FREQ_100M);
    dev->clks[IMX8MP_SYS_PLL1_DIV10] = imx_clk_fixed("sys_pll1_div10", NULL, IMX_CLK_FREQ_80M);
    dev->clks[IMX8MP_SYS_PLL1_DIV20] = imx_clk_fixed("sys_pll1_div20", NULL, IMX_CLK_FREQ_40M);
    dev->clks[IMX8MP_SYS_PLL2] = imx_clk_fixed("sys_pll2", NULL, IMX_CLK_FREQ_1000M);
    dev->clks[IMX8MP_SYS_PLL2_DIV2] = imx_clk_fixed("sys_pll2_div2", NULL, IMX_CLK_FREQ_500M);
    dev->clks[IMX8MP_SYS_PLL2_DIV3] = imx_clk_fixed("sys_pll2_div3", NULL, IMX_CLK_FREQ_333_3M);
    dev->clks[IMX8MP_SYS_PLL2_DIV4] = imx_clk_fixed("sys_pll2_div4", NULL, IMX_CLK_FREQ_250M);
    dev->clks[IMX8MP_SYS_PLL2_DIV5] = imx_clk_fixed("sys_pll2_div5", NULL, IMX_CLK_FREQ_200M);
    dev->clks[IMX8MP_SYS_PLL2_DIV6] = imx_clk_fixed("sys_pll2_div6", NULL, IMX_CLK_FREQ_166_6M);
    dev->clks[IMX8MP_SYS_PLL2_DIV8] = imx_clk_fixed("sys_pll2_div8", NULL, IMX_CLK_FREQ_125M);
    dev->clks[IMX8MP_SYS_PLL2_DIV10] = imx_clk_fixed("sys_pll2_div10", NULL, IMX_CLK_FREQ_100M);
    dev->clks[IMX8MP_SYS_PLL2_DIV20] = imx_clk_fixed("sys_pll2_div20", NULL, IMX_CLK_FREQ_50M);
    dev->clks[IMX8MP_SYS_PLL3] = imx_clk_fixed("sys_pll3", NULL, IMX_CLK_FREQ_750M);
    dev->clks[IMX8MP_AUDIO_PLL1] = imx_clk_fixed("audio_pll1", NULL, IMX_CLK_FREQ_786_4M);
    dev->clks[IMX8MP_AUDIO_PLL2] = imx_clk_fixed("audio_pll2", NULL, IMX_CLK_FREQ_722_5M);

    /* configurable video pll*/
    dev->clks[IMX8MP_VIDEO_PLL1] = imx_clk_pll("video_pll1",
        dev->clks[IMX8MP_CLK_24M],
        dev->ccm_analog_regptr, IMX_CCM_ANALOG_VIDEO_PLL1_GEN_CTRL,
        rate_table);

    /* fill cock sources for CCM slices */
    fill_clk_sels(dev, imx8mp_media_axi_sels_data, imx8mp_media_axi_sels);
    fill_clk_sels(dev, imx8mp_media_apb_sels_data, imx8mp_media_apb_sels);
    fill_clk_sels(dev, imx8mp_media_mipi_phy1_ref_sels_data,
        imx8mp_media_mipi_phy1_ref_sels);
    fill_clk_sels(dev, imx8mp_media_disp1_pix_sels_data,
        imx8mp_media_disp1_pix_sels);
    fill_clk_sels(dev, imx8mp_media_disp2_pix_sels_data,
        imx8mp_media_disp2_pix_sels);
    fill_clk_sels(dev, imx8mp_media_ldb_sels_data,
        imx8mp_media_ldb_sels);
    /* hdmi selectors */
    fill_clk_sels(dev, imx8mp_media_axi_sels_data, imx8mp_hdmi_axi_sels);
    fill_clk_sels(dev, imx8mp_media_apb_sels_data, imx8mp_hdmi_apb_sels);
    fill_clk_sels(dev, imx8mp_hdmi_ref_266m_sels_data, imx8mp_hdmi_ref_266m_sels);
    fill_clk_sels(dev, imx8mp_hdmi_24m_sels_data, imx8mp_hdmi_24m_sels);

    /* clock slices in CCM */
    dev->clks[IMX8MP_CLK_MEDIA_AXI] = imx_clk_slice("media_axi",
        dev->clks[IMX8MP_CLK_24M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MP_MEDIA_AXI_CLK_ROOT),
        imx8mp_media_axi_sels, &share_axi_slice);
    dev->clks[IMX8MP_CLK_MEDIA_APB] = imx_clk_slice("media_apb",
        dev->clks[IMX8MP_CLK_24M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MP_MEDIA_APB_CLK_ROOT),
        imx8mp_media_apb_sels, &share_apb_slice);
    dev->clks[IMX8MP_CLK_MEDIA_DISP2_PIX] = imx_clk_slice("media_disp2_pix",
        dev->clks[IMX8MP_CLK_24M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MP_MEDIA_DISP2_PIX_CLK_ROOT),
        imx8mp_media_disp2_pix_sels, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_MIPI_PHY1_REF] = imx_clk_slice(
        "media_mipi_phy1_ref", dev->clks[IMX8MP_CLK_24M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MP_MEDIA_MIPI_PHY1_REF_CLK_ROOT),
        imx8mp_media_mipi_phy1_ref_sels, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_DISP1_PIX] = imx_clk_slice("media_disp1_pix",
        dev->clks[IMX8MP_CLK_24M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MP_MEDIA_DISP1_PIX_CLK_ROOT),
        imx8mp_media_disp1_pix_sels, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_LDB] = imx_clk_slice(
        "media_ldb", dev->clks[IMX8MP_CLK_24M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MP_MEDIA_LDB_CLK_ROOT),
        imx8mp_media_ldb_sels, NULL);
    /* hdmi slices */
    dev->clks[IMX8MP_CLK_HDMI_AXI] = imx_clk_slice("hdmi_axi",
        dev->clks[IMX8MP_CLK_24M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MP_HDMI_AXI_CLK_ROOT),
        imx8mp_hdmi_axi_sels, NULL);
    dev->clks[IMX8MP_CLK_HDMI_APB] = imx_clk_slice("hdmi_apb",
        dev->clks[IMX8MP_CLK_24M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MP_HDMI_APB_CLK_ROOT),
        imx8mp_hdmi_apb_sels, &share_hdmi_apb_slice);
    dev->clks[IMX8MP_CLK_HDMI_REF_266M] = imx_clk_slice("hdmi_ref_266m",
        dev->clks[IMX8MP_CLK_24M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MP_HDMI_REF_266M_CLK_ROOT),
        imx8mp_hdmi_ref_266m_sels, NULL);
    dev->clks[IMX8MP_CLK_HDMI_24M] = imx_clk_slice("hdmi_24m",
        dev->clks[IMX8MP_CLK_24M],
        dev->ccm_regptr, IMX_CCM_TARGET_ROOTn(IMX8MP_HDMI_24M_CLK_ROOT),
        imx8mp_hdmi_24m_sels, NULL);
    /* Phy PLL. hdmi_phy platform device pointer not known at this point (NULL),
   it must be updated later */
    dev->clks[IMX8MP_CLK_HDMI_PHY] = clk_hdmi_phy_imx8mp("hdmi_phy_pclk",
        dev->clks[IMX8MP_CLK_24M], NULL);

    /* clock gate in LPCG Note: only one LPCG shared among all clocks */
    dev->clks[IMX8MP_CLK_MEDIA_AXI_ROOT] = imx_clk_gate(
        "media_axi_root", dev->clks[IMX8MP_CLK_MEDIA_AXI],
        dev->ccm_regptr, IMX_CCM_CCGRn(IMX8MP_CCM_CCGR_MEDIA), IMX_CCM_BASE,
        0, IMX_CLK_GATE_DOMAIN_ALL_MODES, 3, &share_display_clockgate);
    dev->clks[IMX8MP_CLK_MEDIA_APB_ROOT] = imx_clk_gate(
        "media_apb_root", dev->clks[IMX8MP_CLK_MEDIA_APB],
        dev->ccm_regptr, IMX_CCM_CCGRn(IMX8MP_CCM_CCGR_MEDIA), IMX_CCM_BASE,
        0, IMX_CLK_GATE_DOMAIN_ALL_MODES, 3, &share_display_clockgate);
    dev->clks[IMX8MP_CLK_MEDIA_DISP2_PIX_ROOT] = imx_clk_gate(
        "media_disp2_pix_root", dev->clks[IMX8MP_CLK_MEDIA_DISP2_PIX],
        dev->ccm_regptr, IMX_CCM_CCGRn(IMX8MP_CCM_CCGR_MEDIA), IMX_CCM_BASE,
        0, IMX_CLK_GATE_DOMAIN_ALL_MODES, 3, &share_display_clockgate);
    dev->clks[IMX8MP_CLK_MEDIA_DISP1_PIX_ROOT] = imx_clk_gate(
        "media_disp1_pix_root", dev->clks[IMX8MP_CLK_MEDIA_DISP1_PIX],
        dev->ccm_regptr, IMX_CCM_CCGRn(IMX8MP_CCM_CCGR_MEDIA), IMX_CCM_BASE,
        0, IMX_CLK_GATE_DOMAIN_ALL_MODES, 3, &share_display_clockgate);
    dev->clks[IMX8MP_CLK_MEDIA_LDB_ROOT] = imx_clk_gate(
        "media_ldb_root", dev->clks[IMX8MP_CLK_MEDIA_LDB],
        dev->ccm_regptr, IMX_CCM_CCGRn(IMX8MP_CCM_CCGR_MEDIA), IMX_CCM_BASE,
        0, IMX_CLK_GATE_DOMAIN_ALL_MODES, 3, &share_display_clockgate);
    dev->clks[IMX8MP_CLK_MEDIA_MIPI_PHY1_REF_ROOT] = imx_clk_gate(
        "media_mipi_phy1_ref_root", dev->clks[IMX8MP_CLK_MEDIA_MIPI_PHY1_REF],
        dev->ccm_regptr, IMX_CCM_CCGRn(IMX8MP_CCM_CCGR_MEDIA), IMX_CCM_BASE,
        0, IMX_CLK_GATE_DOMAIN_ALL_MODES, 3, &share_display_clockgate);
    /* hdmi lpcg */
    dev->clks[IMX8MP_CLK_HDMI_ROOT] = imx_clk_gate(
        "hdmi_root", dev->clks[IMX8MP_CLK_HDMI_AXI],
        dev->ccm_regptr, IMX_CCM_CCGRn(IMX8MP_CCM_CCGR_HDMI), IMX_CCM_BASE,
        0, IMX_CLK_GATE_DOMAIN_ALL_MODES, 3, NULL);

    /* clock gates in BLK_CTRL */
    dev->clks[IMX8MP_CLK_MEDIA_LCDIF1_AXI] = imx_clk_gate(
        "lcdif1_axi_clk", dev->clks[IMX8MP_CLK_MEDIA_AXI_ROOT],
        dev->mediamix_regptr, MEDIAMIX_CLK_EN, IMX_MEDIAMIX_BASE,
        SFT_EN_LCDIF_AXI_CLK_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_LCDIF1_APB] = imx_clk_gate(
        "lcdif1_apb_clk", dev->clks[IMX8MP_CLK_MEDIA_APB_ROOT],
        dev->mediamix_regptr, MEDIAMIX_CLK_EN, IMX_MEDIAMIX_BASE,
        SFT_EN_LCDIF_APB_CLK_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_MIPI_DSI_PCLK] = imx_clk_gate(
        "mipi_dsi_pclk", dev->clks[IMX8MP_CLK_MEDIA_APB_ROOT],
        dev->mediamix_regptr, MEDIAMIX_CLK_EN, IMX_MEDIAMIX_BASE,
        SFT_EN_MIPI_DSI_PCLK_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_LCDIF1_PIXEL] = imx_clk_gate(
        "lcdif1_pixel_clk", dev->clks[IMX8MP_CLK_MEDIA_DISP1_PIX_ROOT],
        dev->mediamix_regptr, MEDIAMIX_CLK_EN, IMX_MEDIAMIX_BASE,
        SFT_EN_LCDIF_PIXEL_CLK_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_LCDIF2_AXI] = imx_clk_gate(
        "lcdif2_axi_clk", dev->clks[IMX8MP_CLK_MEDIA_AXI_ROOT],
        dev->mediamix_regptr, MEDIAMIX_CLK_EN, IMX_MEDIAMIX_BASE,
        SFT_EN_LCDIF2_AXI_CLK_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_LCDIF2_APB] = imx_clk_gate(
        "lcdif2_apb_clk", dev->clks[IMX8MP_CLK_MEDIA_APB_ROOT],
        dev->mediamix_regptr, MEDIAMIX_CLK_EN, IMX_MEDIAMIX_BASE,
        SFT_EN_LCDIF2_APB_CLK_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_LCDIF2_PIXEL] = imx_clk_gate(
        "lcdif2_pixel_clk", dev->clks[IMX8MP_CLK_MEDIA_DISP2_PIX_ROOT],
        dev->mediamix_regptr, MEDIAMIX_CLK_EN, IMX_MEDIAMIX_BASE,
        SFT_EN_LCDIF2_PIXEL_CLK_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_MIPI_DSI_CLKREF] = imx_clk_gate(
        "mipi_dsi_clkref", dev->clks[IMX8MP_CLK_MEDIA_MIPI_PHY1_REF_ROOT],
        dev->mediamix_regptr, MEDIAMIX_CLK_EN, IMX_MEDIAMIX_BASE,
        SFT_EN_MIPI_DSI_CLKREF_MASK, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_MEDIA_BUS_BLK] = imx_clk_gate(
        "bus_blk", dev->clks[IMX8MP_CLK_24M],
        dev->mediamix_regptr, MEDIAMIX_CLK_EN, IMX_MEDIAMIX_BASE,
        SFT_EN_BUS_BLK_CLK_SHIFT, 1, 1, NULL);
    /* clock gates in HDMI BLK_CTRL */
    dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB] = imx_clk_gate(
        "hdmi_glb_apb_clk", dev->clks[IMX8MP_CLK_HDMI_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_GLOBAL_APB_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_GLOBAL_B] = imx_clk_gate(
        "hdmi_glb_b_clk", dev->clks[IMX8MP_CLK_HDMI_AXI],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_GLOBAL_B_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_GLOBAL_XTAL24M] = imx_clk_gate(
        "hdmi_glb_24m_clk", dev->clks[IMX8MP_CLK_HDMI_24M],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_GLOBAL_XTAL24M_CLK_EN_SHIFT, 1, 1, &share_hdmi_xtal24_blkctrl);
    dev->clks[IMX8MP_CLK_HDMI_GLOBAL_TX_PIX] = imx_clk_gate(
        "hdmi_glb_pix_clk", dev->clks[IMX8MP_CLK_HDMI_PHY],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_GLOBAL_TX_PIX_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_LCDIF_APB] = imx_clk_gate(
        "hdmi_lcdif3_apb_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_LCDIF_APB_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_LCDIF_B] = imx_clk_gate(
        "hdmi_lcdif3_b_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_B],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_LCDIF_B_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_LCDIF_PDI] = imx_clk_gate(
        "hdmi_lcdif3_pdi_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_LCDIF_PDI_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_LCDIF_PIX] = imx_clk_gate(
        "hdmi_lcdif3_pxl_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_TX_PIX],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_LCDIF_PIX_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_LCDIF_SPU] = imx_clk_gate(
        "hdmi_lcdif3_spu_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_LCDIF_SPU_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_NOC_HDMI] = imx_clk_gate(
        "hdmi_noc_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_NOC_HDMI_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_IRQS_STEER] = imx_clk_gate(
        "hdmi_irq_steer_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL0, IMX_HDMIMIX_BASE,
        HDMI_IRQS_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_VID_LINK_PIX] = imx_clk_gate(
        "hdmi_vid_pix_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_TX_PIX],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_TX_VID_LINK_PIX_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_GPA] = imx_clk_gate(
        "hdmi_tx_gpa_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_TX_GPA_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_PHY_APB] = imx_clk_gate(
        "hdmi_phy_apb_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_TX_PHY_APB_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_PHY_INT] = imx_clk_gate(
        "hdmi_phy_int_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_TX_PHY_INT_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_PREP] = imx_clk_gate(
        "hdmi_tx_prep_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HMDI_TX_PREP_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_SKP] = imx_clk_gate(
        "hdmi_tx_skp_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_TX_SKP_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_SFR] = imx_clk_gate(
        "hdmi_tx_sfr_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_TX_SFR_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_PIXEL] = imx_clk_gate(
        "hdmi_tx_pix_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_TX_PIX],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_TX_PIXEL_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_CEC] = imx_clk_gate(
        "hdmi_cec_clk", dev->clks[IMX8MP_CLK_32K],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_TX_CEC_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_APB] = imx_clk_gate(
        "hdmi_tx_apb_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_TX_APB_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_HPI] = imx_clk_gate(
        "hdmi_tx_hpi_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_TX_HPI_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_FDCC_REF] = imx_clk_gate(
        "hdmi_fdcc_ref_clk", dev->clks[IMX8MP_CLK_HDMI_24M], /* parent HDMI_FDCC */
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HDMI_FDCC_REF_CLK_EN_SHIFT, 1, 1, NULL);
    dev->clks[IMX8MP_CLK_HDMI_TX_PIPE] = imx_clk_gate(
        "hdmi_pipe_sel_clk", dev->clks[IMX8MP_CLK_HDMI_GLOBAL_TX_PIX],
        dev->hdmi_blkctrl_regptr, HDMI_RTX_CLK_CTL1, IMX_HDMIMIX_BASE,
        HMDI_HTX_PIPE_CLK_SEL_SHIFT, 1, 1, NULL);

#ifdef IMX_CLK_DEBUG
    {
        uint32_t val;
        /* Connect sellected clocks to a test pin */
        val = IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_CKE_MASK |
            IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_SEL(IMX_CCM_ANALOG_CLKOUT_SYSTEM_PLL3_CLK) |
            IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_DIV(3) |
            IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_CKE_MASK |
            IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_SEL(IMX_CCM_ANALOG_CLKOUT_VIDEO_PLL1_CLK) |
            IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_DIV(3);
        *(uint32_t *)(((uint8_t*)dev->ccm_analog_regptr) + IMX_CCM_ANALOG_ANAMIX_PLL_MNIT_CTL) = val;
    }
#endif

    if (imx_check_clks(dev, IMX8MP_DC_CLK_END) != 0) {
        return NULL;
    }

    clk_start_imx8mp(dev, new_disp);

#ifdef IMX_CLK_DEBUG
    mediamix_dump_reg(dev->mediamix_regptr);
#endif
    return dev;
}

void clk_dump_clock_tree_imx8mp(const imx8mp_clk_device_t *dev)
{
    if (dev == NULL) {
        return;
    }
#ifdef IMX_CLK_DEBUG_DUMP_WHOLE_CLK_TREE
    imx_clk_dump(dev->clks[IMX8MP_CLK_DUMMY]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_24M]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_32K]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_EXT1]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_EXT2]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_EXT3]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_EXT4]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL1]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL1_DIV2]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL1_DIV3]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL1_DIV4]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL1_DIV5]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL1_DIV6]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL1_DIV8]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL1_DIV10]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL1_DIV20]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL2]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL2_DIV2]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL2_DIV3]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL2_DIV4]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL2_DIV5]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL2_DIV6]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL2_DIV8]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL2_DIV10]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL2_DIV20]);
    imx_clk_dump(dev->clks[IMX8MP_SYS_PLL3]);
    imx_clk_dump(dev->clks[IMX8MP_AUDIO_PLL1]);
    imx_clk_dump(dev->clks[IMX8MP_AUDIO_PLL2]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_AXI]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_APB]);
#endif
    imx_clk_dump(dev->clks[IMX8MP_VIDEO_PLL1]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_DISP1_PIX]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_MIPI_PHY1_REF]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_DISP2_PIX]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_LDB]);
#ifdef IMX_CLK_DEBUG_DUMP_WHOLE_CLK_TREE
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_AXI_ROOT]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_APB_ROOT]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_DISP1_PIX_ROOT]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_MIPI_PHY1_REF_ROOT]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_DISP2_PIX_ROOT]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_LDB_ROOT]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_LCDIF1_AXI]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_LCDIF1_APB]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_LCDIF1_PIXEL]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_LCDIF2_AXI]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_LCDIF2_APB]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_LCDIF2_PIXEL]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_MIPI_DSI_PCLK]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_MIPI_DSI_CLKREF]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_MEDIA_BUS_BLK]);
    /*hdmi*/
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_AXI]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_APB]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_REF_266M]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_24M]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_ROOT]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_GLOBAL_APB]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_GLOBAL_B]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_GLOBAL_XTAL24M]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_GLOBAL_TX_PIX]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_IRQS_STEER]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_NOC_HDMI]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_LCDIF_APB]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_LCDIF_B]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_LCDIF_PDI]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_LCDIF_PIX]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_LCDIF_SPU]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_VID_LINK_PIX]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_GPA]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_PHY_APB]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_PHY_INT]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_PREP]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_SKP]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_SFR]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_PIXEL]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_CEC]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_APB]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_HPI]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_FDCC_REF]);
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_TX_PIPE]);
#endif
    imx_clk_dump(dev->clks[IMX8MP_CLK_HDMI_PHY]);
}

/**
 * Stop selected clocks.
 *
 * @param  dev device to stop the clock for
 * @param  stop_disp display to be stopped
 *
 * @return 0 on success. Returns -1 on failure.
 */

int clk_stop_imx8mp(imx8mp_clk_device_t *dev, imx_display_interface stop_disp)
{
    if (dev == NULL) {
        return -1;
    }

    if ((stop_disp == imx_mipi_dsi) && (dev->disp_mipi != imx_interface_undefined)) {
        CHECK_SUCCESS(imx_clk_enable(dev->clks[IMX8MP_CLK_MEDIA_DISP1_PIX], false, false));
    }
    if ((stop_disp == imx_lvds) && (dev->disp_lvds != imx_interface_undefined)) {
        CHECK_SUCCESS(imx_clk_enable(dev->clks[IMX8MP_CLK_MEDIA_DISP2_PIX], false, false));
    }

    return 0;
}

/**
 * De-initialize clock tree.
 *
 * @param  dev device to init clock tree for
 * @param  stop_disp display to deinit
 *
 * @return 0 on success. Returns -1 on failure.
 */

int clk_deinit_imx8mp(imx8mp_clk_device_t *dev, imx_display_interface stop_disp)
{
    int i;

    if (dev == NULL) {
        return -1;
    }
    switch (stop_disp) {
    case imx_lvds:
        dev->disp_lvds = imx_interface_undefined;
        break;
    case imx_mipi_dsi:
        dev->disp_mipi = imx_interface_undefined;
        break;
    case imx_hdmi:
        dev->disp_hdmi = imx_interface_undefined;
        break;
    default:
        CLK_PRINTK("Display CLK: unsupported interface %d!", stop_disp);
        return -1;
    }
    if ((dev->disp_lvds != imx_interface_undefined) || (dev->disp_mipi != imx_interface_undefined) || (dev->disp_hdmi != imx_interface_undefined)) {
        /* Another display interface still running, keep the clock tree alive. The last one will do the cleaning. */
        return 0;
    }

    for (i = 0; i < IMX8MP_DC_CLK_END; i++) {
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
    if (dev->mediamix_regptr != 0) {
        iounmap(dev->mediamix_regptr, IMX_MEDIAMIX_SIZE);
    }
    if (dev->hdmi_blkctrl_regptr != 0) {
        iounmap(dev->hdmi_blkctrl_regptr, IMX_HDMIMIX_SIZE);
    }

    return 0;
}
