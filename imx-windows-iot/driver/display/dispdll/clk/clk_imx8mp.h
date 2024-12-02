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

#ifndef IMX_CLK_IMX8MP_H
#define IMX_CLK_IMX8MP_H

#include "clk_imx8m.h"
#include "mediamix_mp.h"

/* Clock tree items - START */
/* fixed clocks */
#define IMX8MP_CLK_DUMMY                         0
#define IMX8MP_CLK_24M                           1
#define IMX8MP_CLK_32K                           2
#define IMX8MP_CLK_EXT1                          3
#define IMX8MP_CLK_EXT2                          4
#define IMX8MP_CLK_EXT3                          5
#define IMX8MP_CLK_EXT4                          6
#define IMX8MP_SYS_PLL1                          7
#define IMX8MP_SYS_PLL1_DIV2                     8
#define IMX8MP_SYS_PLL1_DIV3                     9
#define IMX8MP_SYS_PLL1_DIV4                     10
#define IMX8MP_SYS_PLL1_DIV5                     11
#define IMX8MP_SYS_PLL1_DIV6                     12
#define IMX8MP_SYS_PLL1_DIV8                     13
#define IMX8MP_SYS_PLL1_DIV10                    14
#define IMX8MP_SYS_PLL1_DIV20                    15
#define IMX8MP_SYS_PLL2                          16
#define IMX8MP_SYS_PLL2_DIV2                     17
#define IMX8MP_SYS_PLL2_DIV3                     18
#define IMX8MP_SYS_PLL2_DIV4                     19
#define IMX8MP_SYS_PLL2_DIV5                     20
#define IMX8MP_SYS_PLL2_DIV6                     21
#define IMX8MP_SYS_PLL2_DIV8                     22
#define IMX8MP_SYS_PLL2_DIV10                    23
#define IMX8MP_SYS_PLL2_DIV20                    24
#define IMX8MP_SYS_PLL3                          25
#define IMX8MP_AUDIO_PLL1                        26
#define IMX8MP_AUDIO_PLL2                        27
/* pll */
#define IMX8MP_VIDEO_PLL1                        28
/* clock slices */
#define IMX8MP_CLK_MEDIA_AXI                     29
#define IMX8MP_CLK_MEDIA_APB                     30
#define IMX8MP_CLK_MEDIA_MIPI_PHY1_REF           31
#define IMX8MP_CLK_MEDIA_DISP1_PIX               32
#define IMX8MP_CLK_MEDIA_DISP2_PIX               33
#define IMX8MP_CLK_MEDIA_LDB                     34
/* lpcg */
#define IMX8MP_CLK_MEDIA_AXI_ROOT                35
#define IMX8MP_CLK_MEDIA_APB_ROOT                36
#define IMX8MP_CLK_MEDIA_MIPI_PHY1_REF_ROOT      37
#define IMX8MP_CLK_MEDIA_DISP1_PIX_ROOT          38
#define IMX8MP_CLK_MEDIA_DISP2_PIX_ROOT          39
#define IMX8MP_CLK_MEDIA_LDB_ROOT                40
/* BLK_CTRL */
#define IMX8MP_CLK_MEDIA_LCDIF1_AXI              41
#define IMX8MP_CLK_MEDIA_LCDIF1_APB              42
#define IMX8MP_CLK_MEDIA_MIPI_DSI_PCLK           43
#define IMX8MP_CLK_MEDIA_LCDIF1_PIXEL            44
#define IMX8MP_CLK_MEDIA_LCDIF2_AXI              45
#define IMX8MP_CLK_MEDIA_LCDIF2_APB              46
#define IMX8MP_CLK_MEDIA_LCDIF2_PIXEL            47
#define IMX8MP_CLK_MEDIA_BUS_BLK                 48
#define IMX8MP_CLK_MEDIA_MIPI_DSI_CLKREF         49
/* hdmi slices */
#define IMX8MP_CLK_HDMI_AXI                      50
#define IMX8MP_CLK_HDMI_APB                      51
#define IMX8MP_CLK_HDMI_REF_266M                 52
#define IMX8MP_CLK_HDMI_24M                      53
/* hdmi lpcg*/
#define IMX8MP_CLK_HDMI_ROOT                     54
/* hdmi BLK_CTRL */
#define IMX8MP_CLK_HDMI_GLOBAL_APB               55
#define IMX8MP_CLK_HDMI_GLOBAL_B                 56
#define IMX8MP_CLK_HDMI_GLOBAL_XTAL24M           57
#define IMX8MP_CLK_HDMI_GLOBAL_TX_PIX            58
#define IMX8MP_CLK_HDMI_IRQS_STEER               59
#define IMX8MP_CLK_HDMI_NOC_HDMI                 60
#define IMX8MP_CLK_HDMI_LCDIF_APB                61
#define IMX8MP_CLK_HDMI_LCDIF_B                  62
#define IMX8MP_CLK_HDMI_LCDIF_PDI                63
#define IMX8MP_CLK_HDMI_LCDIF_PIX                64
#define IMX8MP_CLK_HDMI_LCDIF_SPU                65
#define IMX8MP_CLK_HDMI_TX_VID_LINK_PIX          66
#define IMX8MP_CLK_HDMI_TX_GPA                   67
#define IMX8MP_CLK_HDMI_TX_PHY_APB               68
#define IMX8MP_CLK_HDMI_TX_PHY_INT               69
#define IMX8MP_CLK_HDMI_TX_PREP                  70
#define IMX8MP_CLK_HDMI_TX_SKP                   71
#define IMX8MP_CLK_HDMI_TX_SFR                   72
#define IMX8MP_CLK_HDMI_TX_PIXEL                 73
#define IMX8MP_CLK_HDMI_TX_CEC                   74
#define IMX8MP_CLK_HDMI_TX_APB                   75
#define IMX8MP_CLK_HDMI_TX_HPI                   76
#define IMX8MP_CLK_HDMI_FDCC_REF                 77
#define IMX8MP_CLK_HDMI_TX_PIPE                  78
#define IMX8MP_CLK_HDMI_PHY                      79

#define IMX8MP_DC_CLK_END                        80
/* Clock tree items - END */

 /* Display interfaces */
typedef enum {
    imx_interface_undefined = 0,
    imx_lvds,
    imx_mipi_dsi,
    imx_hdmi,
} imx_display_interface; 

/**
 * Device object which holds clock tree and memory handles of registers
 */
struct imx8mp_clk_device {
    imx_clk_t *clks[IMX8MP_DC_CLK_END];
    void __iomem *ccm_regptr;
    void __iomem *ccm_analog_regptr;
    void __iomem *mediamix_regptr;
    void __iomem *hdmi_blkctrl_regptr;
    imx_display_interface disp_lvds;
    imx_display_interface disp_mipi;
    imx_display_interface disp_hdmi;
};

typedef struct imx8mp_clk_device imx8mp_clk_device_t;

imx8mp_clk_device_t *clk_init_imx8mp(imx_display_interface disp, bool shared);
int clk_stop_imx8mp(imx8mp_clk_device_t *dev, imx_display_interface stop_disp);
int clk_deinit_imx8mp(imx8mp_clk_device_t *dev, imx_display_interface stop_disp);
void clk_dump_clock_tree_imx8mp(const imx8mp_clk_device_t *dev);
imx_clk_t *clk_get_item_imx8mp(int index);

#endif
