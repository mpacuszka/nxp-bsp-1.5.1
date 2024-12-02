/*
* Copyright 2022 NXP
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the
*   names of its contributors may be used to endorse or promote products
*   derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include "clock.h"
#include "dw_hdmi.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)             (sizeof(x) / sizeof(x[0]))
#endif

static int (*phy_set)(const IMX_DISPLAY_TIMING *mode);

struct clock_to_ncts {
	uint32_t clock;
	uint32_t cts;
	uint32_t n;
};

static const struct clock_to_ncts time_tab[] = {
	{.clock = 25175000, .cts = 25175,  .n = 6144,},
	{.clock = 25200000, .cts = 25200,  .n = 6144,},
	{.clock = 27000000, .cts = 27000,  .n = 6144,},
	{.clock = 27027000, .cts = 27027,  .n = 6144,},
	{.clock = 40000000, .cts = 40000,  .n = 6144,},
	{.clock = 54000000, .cts = 54000,  .n = 6144,},
	{.clock = 54054000, .cts = 54054,  .n = 6144,},
	{.clock = 65000000, .cts = 65000,  .n = 6144,},
	{.clock = 74176000, .cts = 140625, .n = 11648,},
	{.clock = 74250000, .cts = 74250,  .n = 6144,},
	{.clock = 83500000, .cts = 83500,  .n = 6144,},
	{.clock = 106500000,.cts = 106500, .n = 6144,},
	{.clock = 108000000,.cts = 108000, .n = 6144,},
	{.clock = 148352000,.cts = 140625, .n = 5824,},
	{.clock = 148500000,.cts = 148500, .n = 6144,},
	{.clock = 297000000,.cts = 247500, .n = 5120,}
};

static const uint16_t csc_cf_def[3][4] = {
	{ 0x2000, 0x0000, 0x0000, 0x0000 },
	{ 0x0000, 0x2000, 0x0000, 0x0000 },
	{ 0x0000, 0x0000, 0x2000, 0x0000 }
};

static const uint16_t csc_cf_rgbi_eitu601[3][4] = {
	{ 0x2591, 0x1322, 0x074b, 0x0000 },
	{ 0x6535, 0x2000, 0x7acc, 0x0200 },
	{ 0x6acd, 0x7534, 0x2000, 0x0200 }
};

static const uint16_t csc_cf_rgbo_eitu601[3][4] = {
	{ 0x2000, 0x6926, 0x74fd, 0x010e },
	{ 0x2000, 0x2cdd, 0x0000, 0x7e9a },
	{ 0x2000, 0x0000, 0x38b4, 0x7e3b }
};

static const uint16_t csc_cf_rgb_limited[3][4] = {
	{ 0x1b7c, 0x0000, 0x0000, 0x0020 },
	{ 0x0000, 0x1b7c, 0x0000, 0x0020 },
	{ 0x0000, 0x0000, 0x1b7c, 0x0020 }
};

static void hdmi_wr_reg(uint8_t value, uint32_t offset)
{
	*(volatile uint8_t *)((uint64_t)(HDMI_REG_TX_BASE + offset)) = value;
}

static uint8_t hdmi_rd_reg(uint32_t offset)
{
	uint8_t tmp = *(volatile uint8_t *)((uint64_t)(HDMI_REG_TX_BASE + offset));
	return tmp;
}

static void hdmi_mod_reg(uint32_t offset, uint8_t mask, uint8_t value)
{
	uint8_t val = hdmi_rd_reg(offset) & ~mask;

	val |= value & mask;
	hdmi_wr_reg(val, offset);
}

static void hdmi_audio_clk_regen(uint32_t pclk, bool enable_audio)
{
	uint32_t n, cts;
	uint32_t i;
	uint32_t cts3;
	uint32_t n3;

	for (i = 0; i < ARRAY_SIZE(time_tab); i++) {
		if (pclk <= time_tab[i].clock) {
			break;
		}
	}

	if (i >= ARRAY_SIZE(time_tab)) {
		DEBUG ((DEBUG_ERROR, "PCLK %d not supported for audio.\n", pclk));
		return;
	}

	n = time_tab[i].n;
	cts = time_tab[i].cts;

	if (!enable_audio) {
		n = 0;
	}
	/* first select ncts atomic write (if present) */
	n3 = HDMI_REG_AUD_N3_NCTS_ATOMIC_WRITE;
	hdmi_wr_reg(n3, HDMI_REG_AUD_N3);

	/* select cts manual (if present) */
	cts3 = HDMI_REG_AUD_CTS3_CTS_MANUAL;

	cts3 |= HDMI_REG_AUD_CTS3_N_SHIFT_1 << HDMI_REG_AUD_CTS3_N_SHIFT_OFFSET;
	cts3 |= (cts >> 16) & HDMI_REG_AUD_CTS3_AUDCTS19_16_MASK;

	/* set cts in regsters, write cts3 first */
	hdmi_wr_reg(cts3, HDMI_REG_AUD_CTS3);
	hdmi_wr_reg((cts >> 8) & 0xff, HDMI_REG_AUD_CTS2);
	hdmi_wr_reg(cts & 0xff, HDMI_REG_AUD_CTS1);

	/* set n in registers, write n1 last */
	n3 |= (n >> 16) & HDMI_REG_AUD_N3_AUDN19_16_MASK;
	hdmi_wr_reg(n3, HDMI_REG_AUD_N3);
	hdmi_wr_reg((n >> 8) & 0xff, HDMI_REG_AUD_N2);
	hdmi_wr_reg(n & 0xff, HDMI_REG_AUD_N3);

	hdmi_wr_reg(HDMI_REG_AUD_INPUTCLKFS_128, HDMI_REG_AUD_INPUTCLKFS);
}

static void hdmi_video_smpl(uint32_t in_bus_fmt)
{
	uint32_t col_fmt;
	uint32_t val;

	switch (in_bus_fmt) {
	case FMT_RGB888:
		col_fmt = 0x01;
		break;
	case FMT_RGB101010:
		col_fmt = 0x03;
		break;
	case FMT_RGB121212:
		col_fmt = 0x05;
		break;
	case FMT_RGB161616:
		col_fmt = 0x07;
		break;
	default:
		col_fmt = 0x01;
		break;
	}

	val = HDMI_REG_TX_INVID0_INTERNAL_DE_GENERATOR_DISABLE |
	      ((col_fmt << HDMI_REG_TX_INVID0_VIDEO_MAPPING_OFFSET) &
	      HDMI_REG_TX_INVID0_VIDEO_MAPPING_MASK);

	hdmi_wr_reg(val, HDMI_REG_TX_INVID0);

	/* enable tx stuffing: when de is inactive, fix the output data to 0 */
	val = HDMI_REG_TX_INSTUFFING_BDBDATA_STUFFING_ENABLE |
	      HDMI_REG_TX_INSTUFFING_RCRDATA_STUFFING_ENABLE |
	      HDMI_REG_TX_INSTUFFING_GYDATA_STUFFING_ENABLE;
	hdmi_wr_reg(val, HDMI_REG_TX_INSTUFFING);
	hdmi_wr_reg(0x0, HDMI_REG_TX_GYDATA0);
	hdmi_wr_reg(0x0, HDMI_REG_TX_GYDATA1);
	hdmi_wr_reg(0x0, HDMI_REG_TX_RCRDATA0);
	hdmi_wr_reg(0x0, HDMI_REG_TX_RCRDATA1);
	hdmi_wr_reg(0x0, HDMI_REG_TX_BCBDATA0);
	hdmi_wr_reg(0x0, HDMI_REG_TX_BCBDATA1);
}

static void hdmi_video_pcktz(uint32_t out_bus_fmt)
{
	uint32_t val, vcfg;

	/* set the packetizer registers */
	val = ((4 << HDMI_REG_VP_PR_CD_COLOR_DEPTH_OFFSET) & HDMI_REG_VP_PR_CD_COLOR_DEPTH_MASK) |
		((0 << HDMI_REG_VP_PR_CD_DESIRED_PR_FACTOR_OFFSET) & HDMI_REG_VP_PR_CD_DESIRED_PR_FACTOR_MASK);
	hdmi_wr_reg(val, HDMI_REG_VP_PR_CD);

	val = hdmi_rd_reg(HDMI_REG_FC_DATAUTO3);
	if (out_bus_fmt == FMT_RGB888) {
		/* disable Auto GCP when bpp 24. Needs to be done for all formats with color depth = 4 */
		val &= ~0x4;
	} else {
		val |= 0x4;
	}
	hdmi_wr_reg(val, HDMI_REG_FC_DATAUTO3);

	hdmi_mod_reg(HDMI_REG_VP_STUFF, HDMI_REG_VP_STUFF_PR_STUFFING_MASK,
		HDMI_REG_VP_STUFF_PR_STUFFING_STUFFING_MODE);

	vcfg = HDMI_REG_VP_CONF_PR_EN_DISABLE | HDMI_REG_VP_CONF_BYPASS_SELECT_VID_PACKETIZER;

	hdmi_mod_reg(HDMI_REG_VP_CONF, HDMI_REG_VP_CONF_PR_EN_MASK |
		HDMI_REG_VP_CONF_BYPASS_SELECT_MASK, vcfg);

	hdmi_mod_reg(HDMI_REG_VP_STUFF, HDMI_REG_VP_STUFF_IDEFAULT_PHASE_MASK,
		1 << HDMI_REG_VP_STUFF_IDEFAULT_PHASE_OFFSET);

	hdmi_wr_reg(HDMI_REG_VP_REMAP_YCC422_16BIT, HDMI_REG_VP_REMAP);

	vcfg = HDMI_REG_VP_CONF_BYPASS_EN_ENABLE | HDMI_REG_VP_CONF_PP_EN_DISABLE |
		HDMI_REG_VP_CONF_YCC422_EN_DISABLE;

	hdmi_mod_reg(HDMI_REG_VP_CONF, HDMI_REG_VP_CONF_BYPASS_EN_MASK |
		HDMI_REG_VP_CONF_PP_EN_ENMASK | HDMI_REG_VP_CONF_YCC422_EN_MASK, vcfg);

	hdmi_mod_reg(HDMI_REG_VP_STUFF, HDMI_REG_VP_STUFF_PP_STUFFING_MASK |
		HDMI_REG_VP_STUFF_YCC422_STUFFING_MASK,
		HDMI_REG_VP_STUFF_PP_STUFFING_STUFFING_MODE |
		HDMI_REG_VP_STUFF_YCC422_STUFFING_STUFFING_MODE);

	hdmi_mod_reg(HDMI_REG_VP_CONF, HDMI_REG_VP_CONF_OUTPUT_SELECTOR_MASK,
		HDMI_REG_VP_CONF_OUTPUT_SELECTOR_BYPASS);
}

void dw_hdmi_phy_reset(void)
{
	hdmi_wr_reg(HDMI_REG_MC_PHYRSTZ_ASSERT, HDMI_REG_MC_PHYRSTZ);
	hdmi_wr_reg(HDMI_REG_MC_PHYRSTZ_DEASSERT, HDMI_REG_MC_PHYRSTZ);
}

static void hdmi_timing_composition(const IMX_DISPLAY_TIMING *timing)
{
	bool data_en_pol = true;
	uint32_t val;
	uint32_t hbl;
	uint32_t vbl;

	hbl = timing->HBlank;
	vbl = timing->VBlank;

	/* polarity of signals */
	val = HDMI_REG_FC_INVIDCONF_HDCP_KEEPOUT_INACTIVE;

	val |= (timing->Flags & EDID_FLAGS_VSYNC_HIGH ?
		   HDMI_REG_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_HIGH :
		   HDMI_REG_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_LOW);

	val |= (timing->Flags & EDID_FLAGS_HSYNC_HIGH ?
		   HDMI_REG_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_HIGH :
		   HDMI_REG_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_LOW);

	val |= (data_en_pol ?
		   HDMI_REG_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_HIGH :
		   HDMI_REG_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_LOW);

	val |= ((timing->Flags & EDID_FLAGS_HDMI) ?
		   HDMI_REG_FC_INVIDCONF_DVI_MODEZ_HDMI_REG_MODE :
		   HDMI_REG_FC_INVIDCONF_DVI_MODEZ_DVI_MODE);

	val |= HDMI_REG_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_LOW;

	val |= HDMI_REG_FC_INVIDCONF_IN_I_P_PROGRESSIVE;

	hdmi_wr_reg(val, HDMI_REG_FC_INVIDCONF);

	/* horizontal active */
	hdmi_wr_reg(timing->HActive >> 8, HDMI_REG_FC_INHACTV1);
	hdmi_wr_reg(timing->HActive, HDMI_REG_FC_INHACTV0);

	/* vertical active */
	hdmi_wr_reg(timing->VActive >> 8, HDMI_REG_FC_INVACTV1);
	hdmi_wr_reg(timing->VActive, HDMI_REG_FC_INVACTV0);

	/* horizontal blanking */
	hdmi_wr_reg(hbl >> 8, HDMI_REG_FC_INHBLANK1);
	hdmi_wr_reg(hbl, HDMI_REG_FC_INHBLANK0);

	/* vertical blanking */
	hdmi_wr_reg(vbl, HDMI_REG_FC_INVBLANK);

	/* horizontal back porch */
	hdmi_wr_reg(timing->HSyncOffset >> 8, HDMI_REG_FC_HSYNCINDELAY1);
	hdmi_wr_reg(timing->HSyncOffset, HDMI_REG_FC_HSYNCINDELAY0);

	/* vertical back porch */
	hdmi_wr_reg(timing->VSyncOffset, HDMI_REG_FC_VSYNCINDELAY);

	/* hsync pulse */
	hdmi_wr_reg(timing->HSync >> 8, HDMI_REG_FC_HSYNCINWIDTH1);
	hdmi_wr_reg(timing->HSync, HDMI_REG_FC_HSYNCINWIDTH0);

	/* vsync pulse */
	hdmi_wr_reg(timing->VSync, HDMI_REG_FC_VSYNCINWIDTH);
}

static bool hdmi_fmt_is_rgb(uint32_t fmt)
{
	switch (fmt) {
	case FMT_RGB888:
	case FMT_RGB101010:
	case FMT_RGB121212:
	case FMT_RGB161616:
		return true;

	default:
		return false;
	}
}

static uint32_t hdmi_fmt_color_depth(uint32_t bus_format)
{
	switch (bus_format) {
	case FMT_RGB888:
		return HDMI_REG_CSC_SCALE_CSC_COLORDE_PTH_24BPP;

	case FMT_RGB101010:
		return HDMI_REG_CSC_SCALE_CSC_COLORDE_PTH_30BPP;

	case FMT_RGB121212:
		return HDMI_REG_CSC_SCALE_CSC_COLORDE_PTH_36BPP;

	case FMT_RGB161616:
		return HDMI_REG_CSC_SCALE_CSC_COLORDE_PTH_48BPP;

	default:
		return 0xFF;
	}
}

static void dw_hdmi_write_csc_coef(uint32_t in_bus_fmt, uint32_t out_bus_fmt, bool rgb_limited_range)
{
	const uint16_t (*csc_coeff)[3][4] = &csc_cf_def;
	uint32_t i;
	uint32_t csc_scale = 1;

	if (in_bus_fmt != out_bus_fmt) {
		if (hdmi_fmt_is_rgb(out_bus_fmt)) {
			csc_coeff = &csc_cf_rgbo_eitu601;
		} else if (hdmi_fmt_is_rgb(in_bus_fmt)) {
			csc_coeff = &csc_cf_rgbi_eitu601;
			csc_scale = 0;
		}
	} else {
		if (hdmi_fmt_is_rgb(out_bus_fmt) && rgb_limited_range) {
			csc_coeff = &csc_cf_rgb_limited;
		}
	}

	for (i = 0; i < ARRAY_SIZE(csc_cf_def[0]); i++) {
		uint16_t coeff_a = (*csc_coeff)[0][i];
		uint16_t coeff_b = (*csc_coeff)[1][i];
		uint16_t coeff_c = (*csc_coeff)[2][i];

		hdmi_wr_reg(coeff_a & 0xff, HDMI_REG_CSC_COEF_A1_LSB + i * 2);
		hdmi_wr_reg(coeff_a >> 8, HDMI_REG_CSC_COEF_A1_MSB + i * 2);
		hdmi_wr_reg(coeff_b & 0xff, HDMI_REG_CSC_COEF_B1_LSB + i * 2);
		hdmi_wr_reg(coeff_b >> 8, HDMI_REG_CSC_COEF_B1_MSB + i * 2);
		hdmi_wr_reg(coeff_c & 0xff, HDMI_REG_CSC_COEF_C1_LSB + i * 2);
		hdmi_wr_reg(coeff_c >> 8, HDMI_REG_CSC_COEF_C1_MSB + i * 2);
	}

	hdmi_mod_reg(HDMI_REG_CSC_SCALE, HDMI_REG_CSC_SCALE_CSCSCALE_MASK, csc_scale);
}

static void hdmi_video_csc(uint32_t in_bus_fmt, uint32_t out_bus_fmt, bool rgb_limited_range)
{

	uint32_t col_dep = 0;

	col_dep = hdmi_fmt_color_depth(out_bus_fmt);
	if (col_dep == 0xFF) {
		return;
	}

	hdmi_wr_reg(HDMI_REG_CSC_CFG_INTMODE_DISABLE | HDMI_REG_CSC_CFG_DECMODE_DISABLE, HDMI_REG_CSC_CFG);

	hdmi_mod_reg(HDMI_REG_CSC_SCALE, HDMI_REG_CSC_SCALE_CSC_COLORDE_PTH_MASK,
		 col_dep);

	dw_hdmi_write_csc_coef(in_bus_fmt, out_bus_fmt, rgb_limited_range);
}

static void hdmi_enable_vid_path(uint32_t in_bus_fmt, uint32_t out_bus_fmt, bool audio, bool rgb_limited_range)
{
	uint32_t clkdis;
	bool csc_enable = FALSE;

	/* minimum duration */
	hdmi_wr_reg(12, HDMI_REG_FC_CTRLDUR);
	hdmi_wr_reg(32, HDMI_REG_FC_EXCTRLDUR);
	hdmi_wr_reg(1, HDMI_REG_FC_EXCTRLSPAC);

	/* set to fill tmds data channels */
	hdmi_wr_reg(0x0b, HDMI_REG_FC_CH0PREAM);
	hdmi_wr_reg(0x16, HDMI_REG_FC_CH1PREAM);
	hdmi_wr_reg(0x21, HDMI_REG_FC_CH2PREAM);

	hdmi_wr_reg(HDMI_REG_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS,
		   HDMI_REG_MC_FLOWCTRL);

	/* enable pixel clock and tmds data path */
	clkdis = 0x7f;
	clkdis &= ~HDMI_REG_MC_CLKDIS_PIXELCLK_DISABLE;
	hdmi_wr_reg(clkdis, HDMI_REG_MC_CLKDIS);

	clkdis &= ~HDMI_REG_MC_CLKDIS_TMDSCLK_DISABLE;
	hdmi_wr_reg(clkdis, HDMI_REG_MC_CLKDIS);

	/* Enable csc path */
	if (in_bus_fmt != out_bus_fmt) {
		csc_enable = TRUE;
	} else if (rgb_limited_range && hdmi_fmt_is_rgb(out_bus_fmt)) {
		csc_enable = TRUE;
	}

	/* Enable color space conversion if needed */
	if (csc_enable) {
		clkdis &= ~HDMI_REG_MC_CLKDIS_CSCCLK_DISABLE;
		hdmi_wr_reg(clkdis, HDMI_REG_MC_CLKDIS);

		hdmi_wr_reg( HDMI_REG_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_IN_PATH,
			   HDMI_REG_MC_FLOWCTRL);
	} else {
		hdmi_wr_reg(HDMI_REG_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS,
			   HDMI_REG_MC_FLOWCTRL);
	}
	
	if (audio) {
		clkdis &= ~HDMI_REG_MC_CLKDIS_AUDCLK_DISABLE;
		hdmi_wr_reg(clkdis, HDMI_REG_MC_CLKDIS);
	}
}

static uint32_t hdmi_get_vic(IMX_DISPLAY_TIMING *timing)
{
	uint32_t vic;
	if ((timing->HActive == 720) && (timing->VActive == 480) && (timing->PixelClock == 27000000)) {
		vic = 4; /* 480p60 */
	} else if ((timing->HActive == 1280) && (timing->VActive == 720) && (timing->PixelClock == 74250000)) {
		vic = 4; /* 720p60 */
	} else if ((timing->HActive == 1920) && (timing->VActive == 1080) && (timing->PixelClock == 148500000)) {
		vic = 16; /* 1080p60 */
	} else if ((timing->HActive == 3840) && (timing->VActive == 2160) && (timing->PixelClock == 297000000)) {
		vic = 95; /* 2160p30 */
	} else if ((timing->HActive == 3840) && (timing->VActive == 2160) && (timing->PixelClock == 594000000)) {
		vic = 97; /* 2160p60 */
	} else {
		DEBUG((DEBUG_ERROR, "Resolution not supported %dx%d pclk=%d returning vic = 0\n",
			timing->HActive, timing->VActive, timing->PixelClock));
		vic = 0;
	}
		
	return vic;
}

static void hdmi_cfg_AVI(IMX_DISPLAY_TIMING *timing, uint32_t out_bus_fmt, bool rgb_limited_range)
{
	uint8_t val, col, ext_col, y_q;

	val = (HDMI_REG_SCAN_MODE_UNDERSCAN) << 4;
	val |= HDMI_REG_COLORSPACE_RGB;
	col = HDMI_REG_COLORIMETRY_NONE;
	ext_col = HDMI_REG_EXTENDED_COLORIMETRY_XV_YCC_601;

	if (!hdmi_fmt_is_rgb(out_bus_fmt)) {
		/* val, col, ext_col variables need to be set properly for non RGB */
		DEBUG((DEBUG_ERROR, "non RGB colorspace not supported\n"));
	}
	val |= HDMI_REG_FC_AVICONF0_ACTIVE_FMT_INFO_PRESENT;
	hdmi_wr_reg(val, HDMI_REG_FC_AVICONF0);

	val = ((col & 0x3) << 6) |
	      ((HDMI_REG_PICTURE_ASPECT_16_9 & 0x3) << 4) |
	      (HDMI_REG_ACTIVE_ASPECT_PICTURE & 0xf);
	hdmi_wr_reg(val, HDMI_REG_FC_AVICONF1);

	val = ((ext_col & 0x7) << 4);
	if (rgb_limited_range) {
		val |= ((HDMI_REG_QUANTIZATION_RANGE_LIMITED & 0x3) << 2);
		y_q = HDMI_REG_YCC_QUANTIZATION_RANGE_LIMITED;
	} else {
		val |= ((HDMI_REG_QUANTIZATION_RANGE_FULL & 0x3) << 2);
		y_q = HDMI_REG_YCC_QUANTIZATION_RANGE_FULL;
	}
	hdmi_wr_reg(val, HDMI_REG_FC_AVICONF2);

	val = hdmi_get_vic(timing) & 0x7f;
	hdmi_wr_reg(val, HDMI_REG_FC_AVIVID);

	val = (((0 + 1) <<	HDMI_REG_FC_PRCONF_INCOMING_PR_FACTOR_OFFSET) &
		HDMI_REG_FC_PRCONF_INCOMING_PR_FACTOR_MASK) |
		((0 <<	HDMI_REG_FC_PRCONF_OUTPUT_PR_FACTOR_OFFSET) &
		HDMI_REG_FC_PRCONF_OUTPUT_PR_FACTOR_MASK);
	hdmi_wr_reg(val, HDMI_REG_FC_PRCONF);

	val = ((y_q & 0x3) << 2) | (HDMI_REG_CONTENT_TYPE_GRAPHICS & 0x3);
	hdmi_wr_reg(val, HDMI_REG_FC_AVICONF3);

	hdmi_wr_reg(0, HDMI_REG_FC_AVIETB0);
	hdmi_wr_reg(0, HDMI_REG_FC_AVIETB1);
	hdmi_wr_reg(0, HDMI_REG_FC_AVISBB0);
	hdmi_wr_reg(0, HDMI_REG_FC_AVISBB1);
	hdmi_wr_reg(0, HDMI_REG_FC_AVIELB0);
	hdmi_wr_reg(0, HDMI_REG_FC_AVIELB1);
	hdmi_wr_reg(0, HDMI_REG_FC_AVISRB0);
	hdmi_wr_reg(0, HDMI_REG_FC_AVISRB1);
}

static void hdmi_tx_hdcp_cfg(void)
{
	uint8_t de = HDMI_REG_A_VIDPOLCFG_DATAENPOL_ACTIVE_HIGH;

	/* disable rx detect */
	hdmi_mod_reg(HDMI_REG_A_HDCPCFG0, HDMI_REG_A_HDCPCFG0_RXDETECT_MASK, HDMI_REG_A_HDCPCFG0_RXDETECT_DISABLE);

	hdmi_mod_reg(HDMI_REG_A_VIDPOLCFG, HDMI_REG_A_VIDPOLCFG_DATAENPOL_MASK, de);

	hdmi_mod_reg(HDMI_REG_A_HDCPCFG1, HDMI_REG_A_HDCPCFG1_ENCRYPTIONDISABLE_MASK,
		HDMI_REG_A_HDCPCFG1_ENCRYPTIONDISABLE_DISABLE);
}

static void hdmi_clear_ovf(void)
{
	uint32_t val, cnt;

	/* tmds software reset */
	hdmi_wr_reg((uint8_t)~HDMI_REG_MC_SWRSTZ_TMDSSWRST_REQ, HDMI_REG_MC_SWRSTZ);

	val = hdmi_rd_reg(HDMI_REG_FC_INVIDCONF);

	for (cnt = 0; cnt < 4; cnt++)
		hdmi_wr_reg(val, HDMI_REG_FC_INVIDCONF);
}

static uint32_t hdmi_ddc_wait_i2c_done(uint32_t msec)
{
	uint32_t val;
	uint32_t loop = msec * 10;

	do {
		val = hdmi_rd_reg(HDMI_REG_IH_I2CM_STAT0);
		if (val & HDMI_REG_I2C_MASTER_ERROR) {
			hdmi_wr_reg(val, HDMI_REG_IH_I2CM_STAT0);
			return 1;
		}
		if (val & HDMI_REG_I2C_MASTER_DONE) {
			hdmi_wr_reg(val, HDMI_REG_IH_I2CM_STAT0);
			return 0;
		}

		MicroSecondDelay(100);
		loop--;
	} while (loop);

	return 1;
}

EFI_STATUS dw_hdmi_read_edid(uint8_t *buff, uint32_t block, uint32_t length)
{
	EFI_STATUS error = EFI_SUCCESS;
	uint32_t try = 2000;
	uint32_t i;
	uint32_t offset = (block % 2) * 0x80;

	hdmi_mod_reg(HDMI_REG_I2CM_SOFTRSTZ, HDMI_REG_I2CM_SOFTRSTZ_MASK, 0);

	/* set divider from SFR clock for 100khz speed */
	hdmi_wr_reg(I2C_CLK_HIGH, HDMI_REG_I2CM_SS_SCL_HCNT_0_ADDR);
	hdmi_wr_reg(I2C_CLK_LOW, HDMI_REG_I2CM_SS_SCL_LCNT_0_ADDR);

	hdmi_mod_reg(HDMI_REG_I2CM_DIV, HDMI_REG_I2CM_DIV_FAST_STD_MODE,
		 HDMI_REG_I2CM_DIV_STD_MODE);

	hdmi_wr_reg(HDMI_REG_I2CM_SLAVE_DDC_ADDR, HDMI_REG_I2CM_SLAVE);
	hdmi_wr_reg(HDMI_REG_I2CM_SEGADDR_DDC, HDMI_REG_I2CM_SEGADDR);
	hdmi_wr_reg(block >> 1, HDMI_REG_I2CM_SEGPTR);

	while (try--) {
		error = EFI_SUCCESS;
		for (i = 0; i < length; i++) {
			hdmi_wr_reg(offset + i, HDMI_REG_I2CM_ADDRESS);

			if (block == 0) {
				hdmi_wr_reg(HDMI_REG_I2CM_OP_RD8, HDMI_REG_I2CM_OPERATION);
			} else {
				hdmi_wr_reg(HDMI_REG_I2CM_OP_RD8_EXT, HDMI_REG_I2CM_OPERATION);
			}

			if (hdmi_ddc_wait_i2c_done(10)) {
				hdmi_mod_reg(HDMI_REG_I2CM_SOFTRSTZ, HDMI_REG_I2CM_SOFTRSTZ_MASK, 0);
				error = EFI_DEVICE_ERROR;
				break;
			}

			buff[i] = hdmi_rd_reg(HDMI_REG_I2CM_DATAI);
		}

		if (error == EFI_SUCCESS) {
			DEBUG ((DEBUG_ERROR, "dw_hdmi_read_edid: successfully read EDID. try=%d\n", try));
			break;
		}
	}

	return error;
}

EFI_STATUS dw_hdmi_phy_wait_for_hpd(void)
{
	uint32_t loop = 30000U;

	do {
		if (hdmi_rd_reg(HDMI_REG_PHY_STAT0) & HDMI_REG_PHY_HPD) {
			return EFI_SUCCESS;
		}
		MicroSecondDelay(100);
		loop--;
	} while (loop);

	DEBUG ((DEBUG_ERROR, "HDMI hot plug not detected.\n"));
	return EFI_DEVICE_ERROR;
}

void dw_hdmi_phy_init(void)
{
	/* enable phy i2c done irq */
	hdmi_wr_reg(HDMI_REG_PHY_I2CM_INT_ADDR_DONE_POL,
		   HDMI_REG_PHY_I2CM_INT_ADDR);

	/* enable phy i2c nack & arbitr. irq */
	hdmi_wr_reg(HDMI_REG_PHY_I2CM_CTLINT_ADDR_NAC_POL |
		   HDMI_REG_PHY_I2CM_CTLINT_ADDR_ARBITRATION_POL,
		   HDMI_REG_PHY_I2CM_CTLINT_ADDR);

	/* enable hpd irq */
	hdmi_wr_reg((uint8_t)~HDMI_REG_PHY_HPD, HDMI_REG_PHY_MASK0);

	/* clear hpd interrupts */
	hdmi_wr_reg(HDMI_REG_IH_PHY_STAT0_HPD, HDMI_REG_IH_PHY_STAT0);
}

EFI_STATUS dw_hdmi_enable(IMX_DISPLAY_TIMING *timing, uint32_t in_bus_fmt, uint32_t out_bus_fmt)
{
	uint32_t ret;
	bool rgb_limited_range = FALSE;

	if (timing->EdidFlags & EDID_DIGITAL_SYNC_MASK) {
		timing->Flags |= (timing->EdidFlags & EDID_VERTICAL_SYNC_MASK) ? EDID_FLAGS_VSYNC_HIGH : 0;
		timing->Flags |= (timing->EdidFlags & EDID_HORIZONTAL_SYNC_MASK) ? EDID_FLAGS_HSYNC_HIGH : 0;
	}
	if (timing->Flags & EDID_FLAGS_HDMI) {
		if ((timing->HActive != 640) || (timing->VActive != 480)) {
			rgb_limited_range = TRUE;
		}
	}

	DEBUG ((DEBUG_INFO, "Monitor is_hdmi=%d pixel clk = %d resolution %d x %d vsync=%d hsync=%d limit=%d\n",
		(timing->Flags & EDID_FLAGS_HDMI) ? 1 : 0,
		timing->PixelClock, timing->HActive, timing->VActive,
		(timing->Flags & EDID_FLAGS_VSYNC_HIGH) ? 1 : 0,
		(timing->Flags & EDID_FLAGS_HSYNC_HIGH) ? 1 : 0, rgb_limited_range));

	hdmi_timing_composition(timing);

	if (phy_set) {
		ret = phy_set(timing);
		if (ret) {
			return EFI_DEVICE_ERROR;
		}
	}

	/* Disable audio, originaly enabled for (timing->Flags & EDID_FLAGS_HDMI) */
	hdmi_enable_vid_path(in_bus_fmt, out_bus_fmt, false, rgb_limited_range);

	if (timing->Flags & EDID_FLAGS_HDMI) {
		/* fifo reset */
		hdmi_wr_reg((uint8_t)~HDMI_REG_MC_SWRSTZ_II2SSWRST_REQ, HDMI_REG_MC_SWRSTZ);
		hdmi_wr_reg(HDMI_REG_AUD_CONF0_SW_AUDIO_FIFO_RST, HDMI_REG_AUD_CONF0);

		hdmi_wr_reg(0x00, HDMI_REG_AUD_INT);
		hdmi_wr_reg(0x00, HDMI_REG_AUD_INT1);
		
		/* audio set format */
		hdmi_wr_reg(HDMI_REG_AUD_CONF0_I2S_SELECT | HDMI_REG_AUD_CONF0_I2S_IN_EN_0,
			HDMI_REG_AUD_CONF0);
		hdmi_wr_reg(HDMI_REG_AUD_CONF1_I2S_MODE_STANDARD_MODE |
			HDMI_REG_AUD_CONF1_I2S_WIDTH_16BIT, HDMI_REG_AUD_CONF1);
		hdmi_wr_reg(0x00, HDMI_REG_AUD_CONF2);

		hdmi_audio_clk_regen(timing->PixelClock, false);

		if ((timing->HActive != 640) || (timing->VActive != 480)) {
			rgb_limited_range = TRUE;
		}
		hdmi_cfg_AVI(timing, out_bus_fmt, rgb_limited_range);
	}

	hdmi_video_pcktz(out_bus_fmt);
	hdmi_video_csc(in_bus_fmt, out_bus_fmt, rgb_limited_range);
	hdmi_video_smpl(in_bus_fmt);
	hdmi_tx_hdcp_cfg();

	hdmi_clear_ovf();

	return EFI_SUCCESS;
}

void dw_hdmi_init(int (*phy_init)(const IMX_DISPLAY_TIMING *mode))
{
	uint32_t ih_mute;

	if (phy_init) {
		phy_set = phy_init;
	} else {
		phy_set = NULL;
	}
	
	/* disable interrupts in hdmi tx */
	ih_mute = /*hdmi_rd_reg(HDMI_REG_IH_MUTE) |*/
		  HDMI_REG_IH_MUTE_MUTE_WAKEUP_INTERRUPT |
		  HDMI_REG_IH_MUTE_MUTE_ALL_INTERRUPT;

	hdmi_wr_reg(ih_mute, HDMI_REG_IH_MUTE);

	/* enable i2c master done irq */
	hdmi_wr_reg(~0x04, HDMI_REG_I2CM_INT);

	/* enable i2c client nack and arbitr irq */
	hdmi_wr_reg(~0x44, HDMI_REG_I2CM_CTLINT);

	hdmi_wr_reg((HDMI_REG_I2C_SCDC_READ_REQ|HDMI_REG_I2C_MASTER_DONE|HDMI_REG_I2C_MASTER_ERROR), HDMI_REG_IH_I2CM_STAT0);
}


#define debug_print(x, y) DEBUG ((DEBUG_ERROR, x, y, hdmi_rd_reg(y)));

void dw_hdmi_dump_regs(void)
{
	debug_print("HDMI_REG_PHY_I2CM_INT_ADDR: offset=0x%X val= 0x%08X\n", HDMI_REG_PHY_I2CM_INT_ADDR);
	debug_print("HDMI_REG_PHY_I2CM_CTLINT_ADDR: offset=0x%X val= 0x%08X\n", HDMI_REG_PHY_I2CM_CTLINT_ADDR);
	debug_print("HDMI_REG_PHY_MASK0: offset=0x%X val= 0x%08X\n", HDMI_REG_PHY_MASK0);
	debug_print("HDMI_REG_PHY_POL0: offset=0x%X val= 0x%08X\n", HDMI_REG_PHY_POL0);
	debug_print("HDMI_REG_I2CM_SOFTRSTZ: offset=0x%X val= 0x%08X\n", HDMI_REG_I2CM_SOFTRSTZ);
	debug_print("HDMI_REG_I2CM_DIV: offset=0x%X val= 0x%08X\n", HDMI_REG_I2CM_DIV);
	debug_print("HDMI_REG_I2CM_INT: offset=0x%X val= 0x%08X\n", HDMI_REG_I2CM_INT);
	debug_print("HDMI_REG_I2CM_CTLINT: offset=0x%X val= 0x%08X\n", HDMI_REG_I2CM_CTLINT);
	debug_print("HDMI_REG_I2CM_ADDRESS: offset=0x%X val= 0x%08X\n", HDMI_REG_I2CM_ADDRESS);
	debug_print("HDMI_REG_I2CM_OPERATION: offset=0x%X val= 0x%08X\n", HDMI_REG_I2CM_OPERATION);
	debug_print("HDMI_REG_I2CM_SLAVE: offset=0x%X val= 0x%08X\n", HDMI_REG_I2CM_SLAVE);
	debug_print("HDMI_REG_I2CM_SEGADDR: offset=0x%X val= 0x%08X\n", HDMI_REG_I2CM_SEGADDR);
	debug_print("HDMI_REG_I2CM_SEGPTR: offset=0x%X val= 0x%08X\n", HDMI_REG_I2CM_SEGPTR);
	debug_print("HDMI_REG_IH_I2CM_STAT0: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_I2CM_STAT0);
	debug_print("HDMI_REG_IH_MUTE: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE);
	debug_print("HDMI_REG_IH_MUTE_I2CM_STAT0: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE_I2CM_STAT0);
	debug_print("HDMI_REG_IH_MUTE_I2CMPHY_STAT0: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE_I2CMPHY_STAT0);
	debug_print("HDMI_REG_IH_MUTE_AHBDMAAUD_STAT0: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE_AHBDMAAUD_STAT0);
	debug_print("HDMI_REG_IH_MUTE_PHY_STAT0: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE_PHY_STAT0);
	debug_print("HDMI_REG_IH_MUTE_FC_STAT0: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE_FC_STAT0);
	debug_print("HDMI_REG_IH_MUTE_FC_STAT1: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE_FC_STAT1);
	debug_print("HDMI_REG_IH_MUTE_FC_STAT2: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE_FC_STAT2);
	debug_print("HDMI_REG_IH_PHY_STAT0: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_PHY_STAT0);
	debug_print("HDMI_REG_IH_MUTE_AS_STAT0: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE_AS_STAT0);
	debug_print("HDMI_REG_IH_MUTE_CEC_STAT0: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE_CEC_STAT0);
	debug_print("HDMI_REG_IH_MUTE_VP_STAT0: offset=0x%X val= 0x%08X\n", HDMI_REG_IH_MUTE_VP_STAT0);
	debug_print("HDMI_REG_AUD_CTS3: offset=0x%X val= 0x%08X\n", HDMI_REG_AUD_CTS3);
	debug_print("HDMI_REG_AUD_CTS2: offset=0x%X val= 0x%08X\n", HDMI_REG_AUD_CTS2);
	debug_print("HDMI_REG_AUD_CTS1: offset=0x%X val= 0x%08X\n", HDMI_REG_AUD_CTS1);
	debug_print("HDMI_REG_AUD_N3: offset=0x%X val= 0x%08X\n", HDMI_REG_AUD_N3);
	debug_print("HDMI_REG_AUD_N2: offset=0x%X val= 0x%08X\n", HDMI_REG_AUD_N2);
	debug_print("HDMI_REG_AUD_N1: offset=0x%X val= 0x%08X\n", HDMI_REG_AUD_N1);
	debug_print("HDMI_REG_AUD_INT: offset=0x%X val= 0x%08X\n", HDMI_REG_AUD_INT);
	debug_print("HDMI_REG_AUD_SPDIFINT: offset=0x%X val= 0x%08X\n", HDMI_REG_AUD_SPDIFINT);
	debug_print("HDMI_REG_AUD_HBR_MASK: offset=0x%X val= 0x%08X\n", HDMI_REG_AUD_HBR_MASK);
	debug_print("HDMI_REG_FC_AUDSCONF: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AUDSCONF);
	debug_print("HDMI_REG_FC_DATAUTO3: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_DATAUTO3);
	debug_print("HDMI_REG_FC_AVICONF0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVICONF0);
	debug_print("HDMI_REG_FC_AVICONF1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVICONF1);
	debug_print("HDMI_REG_FC_AVICONF2: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVICONF2);
	debug_print("HDMI_REG_FC_AVICONF3: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVICONF3);
	debug_print("HDMI_REG_FC_AVIVID: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVIVID);
	debug_print("HDMI_REG_FC_PRCONF: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_PRCONF);
	debug_print("HDMI_REG_FC_AVIETB0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVIETB0);
	debug_print("HDMI_REG_FC_AVIETB1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVIETB1);
	debug_print("HDMI_REG_FC_AVISBB0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVISBB0);
	debug_print("HDMI_REG_FC_AVISBB1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVISBB1);
	debug_print("HDMI_REG_FC_AVIELB0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVIELB0);
	debug_print("HDMI_REG_FC_AVIELB1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVIELB1);
	debug_print("HDMI_REG_FC_AVISRB0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVISRB0);
	debug_print("HDMI_REG_FC_AVISRB1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_AVISRB1);
	debug_print("HDMI_REG_FC_INVIDCONF: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_INVIDCONF);
	debug_print("HDMI_REG_FC_INHACTV1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_INHACTV1);
	debug_print("HDMI_REG_FC_INHACTV0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_INHACTV0);
	debug_print("HDMI_REG_FC_INVACTV1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_INVACTV1);
	debug_print("HDMI_REG_FC_INVACTV0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_INVACTV0);
	debug_print("HDMI_REG_FC_INHBLANK1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_INHBLANK1);
	debug_print("HDMI_REG_FC_INHBLANK0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_INHBLANK0);
	debug_print("HDMI_REG_FC_INVBLANK: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_INVBLANK);
	debug_print("HDMI_REG_FC_HSYNCINDELAY1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_HSYNCINDELAY1);
	debug_print("HDMI_REG_FC_HSYNCINDELAY0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_HSYNCINDELAY0);
	debug_print("HDMI_REG_FC_VSYNCINDELAY: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_VSYNCINDELAY);
	debug_print("HDMI_REG_FC_HSYNCINWIDTH1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_HSYNCINWIDTH1);
	debug_print("HDMI_REG_FC_HSYNCINWIDTH0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_HSYNCINWIDTH0);
	debug_print("HDMI_REG_FC_VSYNCINWIDTH: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_VSYNCINWIDTH);
	debug_print("HDMI_REG_FC_CTRLDUR: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_CTRLDUR);
	debug_print("HDMI_REG_FC_EXCTRLDUR: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_EXCTRLDUR);
	debug_print("HDMI_REG_FC_EXCTRLSPAC: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_EXCTRLSPAC);
	debug_print("HDMI_REG_FC_CH0PREAM: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_CH0PREAM);
	debug_print("HDMI_REG_FC_CH1PREAM: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_CH1PREAM);
	debug_print("HDMI_REG_FC_CH2PREAM: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_CH2PREAM);
	debug_print("HDMI_REG_FC_MASK0: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_MASK0);
	debug_print("HDMI_REG_FC_MASK1: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_MASK1);
	debug_print("HDMI_REG_FC_MASK2: offset=0x%X val= 0x%08X\n", HDMI_REG_FC_MASK2);
	debug_print("HDMI_REG_MC_SWRSTZ: offset=0x%X val= 0x%08X\n", HDMI_REG_MC_SWRSTZ);
	debug_print("HDMI_REG_MC_FLOWCTRL: offset=0x%X val= 0x%08X\n", HDMI_REG_MC_FLOWCTRL);
	debug_print("HDMI_REG_MC_CLKDIS: offset=0x%X val= 0x%08X\n", HDMI_REG_MC_CLKDIS);
	debug_print("HDMI_REG_MC_PHYRSTZ: offset=0x%X val= 0x%08X\n", HDMI_REG_MC_PHYRSTZ);
	debug_print("HDMI_REG_TX_INVID0: offset=0x%X val= 0x%08X\n", HDMI_REG_TX_INVID0);
	debug_print("HDMI_REG_TX_INSTUFFING: offset=0x%X val= 0x%08X\n", HDMI_REG_TX_INSTUFFING);
	debug_print("HDMI_REG_TX_GYDATA0: offset=0x%X val= 0x%08X\n", HDMI_REG_TX_GYDATA0);
	debug_print("HDMI_REG_TX_GYDATA1: offset=0x%X val= 0x%08X\n", HDMI_REG_TX_GYDATA1);
	debug_print("HDMI_REG_TX_RCRDATA0: offset=0x%X val= 0x%08X\n", HDMI_REG_TX_RCRDATA0);
	debug_print("HDMI_REG_TX_RCRDATA1: offset=0x%X val= 0x%08X\n", HDMI_REG_TX_RCRDATA1);
	debug_print("HDMI_REG_TX_BCBDATA0: offset=0x%X val= 0x%08X\n", HDMI_REG_TX_BCBDATA0);
	debug_print("HDMI_REG_TX_BCBDATA1: offset=0x%X val= 0x%08X\n", HDMI_REG_TX_BCBDATA1);
	debug_print("HDMI_REG_CSC_COEF_A1_LSB: offset=0x%X val= 0x%08X\n", HDMI_REG_CSC_COEF_A1_LSB);
	debug_print("HDMI_REG_CSC_COEF_A1_MSB: offset=0x%X val= 0x%08X\n", HDMI_REG_CSC_COEF_A1_MSB);
	debug_print("HDMI_REG_CSC_COEF_B1_LSB: offset=0x%X val= 0x%08X\n", HDMI_REG_CSC_COEF_B1_LSB);
	debug_print("HDMI_REG_CSC_COEF_B1_MSB: offset=0x%X val= 0x%08X\n", HDMI_REG_CSC_COEF_B1_MSB);
	debug_print("HDMI_REG_CSC_COEF_C1_LSB: offset=0x%X val= 0x%08X\n", HDMI_REG_CSC_COEF_C1_LSB);
	debug_print("HDMI_REG_CSC_COEF_C1_MSB: offset=0x%X val= 0x%08X\n", HDMI_REG_CSC_COEF_C1_MSB);
	debug_print("HDMI_REG_CSC_SCALE: offset=0x%X val= 0x%08X\n", HDMI_REG_CSC_SCALE);
	debug_print("HDMI_REG_CSC_CFG: offset=0x%X val= 0x%08X\n", HDMI_REG_CSC_CFG);
	debug_print("HDMI_REG_VP_PR_CD: offset=0x%X val= 0x%08X\n", HDMI_REG_VP_PR_CD);
	debug_print("HDMI_REG_VP_STUFF: offset=0x%X val= 0x%08X\n", HDMI_REG_VP_STUFF);
	debug_print("HDMI_REG_VP_CONF: offset=0x%X val= 0x%08X\n", HDMI_REG_VP_CONF);
	debug_print("HDMI_REG_VP_REMAP: offset=0x%X val= 0x%08X\n", HDMI_REG_VP_REMAP);
	debug_print("HDMI_REG_VP_MASK: offset=0x%X val= 0x%08X\n", HDMI_REG_VP_MASK);
	debug_print("HDMI_REG_A_HDCPCFG0: offset=0x%X val= 0x%08X\n", HDMI_REG_A_HDCPCFG0);
	debug_print("HDMI_REG_A_VIDPOLCFG: offset=0x%X val= 0x%08X\n", HDMI_REG_A_VIDPOLCFG);
	debug_print("HDMI_REG_A_HDCPCFG1: offset=0x%X val= 0x%08X\n", HDMI_REG_A_HDCPCFG1);
	debug_print("HDMI_REG_A_APIINTMSK: offset=0x%X val= 0x%08X\n", HDMI_REG_A_APIINTMSK);
	debug_print("HDMI_REG_GP_MASK: offset=0x%X val= 0x%08X\n", HDMI_REG_GP_MASK);
	debug_print("HDMI_REG_CEC_TX_CNT: offset=0x%X val= 0x%08X\n", HDMI_REG_CEC_TX_CNT);
	debug_print("HDMI_REG_CEC_MASK: offset=0x%X val= 0x%08X\n", HDMI_REG_CEC_MASK);
	debug_print("HDMI_REG_CEC_POLARITY: offset=0x%X val= 0x%08X\n", HDMI_REG_CEC_POLARITY);
}

