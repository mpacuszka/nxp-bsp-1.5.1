// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices ADV7511 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 * Copyright 2022-2023 NXP
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#include <drm/drm_atomic.h>
#include <drm/drm_edid.h>
#include <drm/drm_print.h>

#include "mipi_dsi/adv7511.h"

/* ADI recommended values for proper operation. */
static const struct reg_sequence adv7511_fixed_registers[] = {
	{ 0x98, 0x03 },
	{ 0x9a, 0xe0 },
	{ 0x9c, 0x30 },
	{ 0x9d, 0x61 },
	{ 0xa2, 0xa4 },
	{ 0xa3, 0xa4 },
	{ 0xe0, 0xd0 },
	{ 0xf9, 0x00 },
	{ 0x55, 0x02 },
};

/*
 * TODO: Currently, filter-out unsupported modes by their clocks.
 * Need to find a better way to do this.
 * These are the pixel clocks that the converter can handle successfully.
 */

static const int valid_clocks[] = {
	148500,
	135000,
	132000,
	108000,
	78750,
	74250,
	71000,
	65000,
	49500,
	40000,
	31500,
};

/* -----------------------------------------------------------------------------
 * Hardware configuration
 */

static void adv7511_set_colormap(struct adv7511 *adv7511, bool enable,
				 const uint16_t *coeff,
				 unsigned int scaling_factor)
{
	unsigned int i;

	regmap_update_bits(adv7511->regmap, ADV7511_REG_CSC_UPPER(1),
			   ADV7511_CSC_UPDATE_MODE, ADV7511_CSC_UPDATE_MODE);

	if (enable) {
		for (i = 0; i < 12; ++i) {
			regmap_update_bits(adv7511->regmap,
					   ADV7511_REG_CSC_UPPER(i),
					   0x1f, coeff[i] >> 8);
			regmap_write(adv7511->regmap,
				     ADV7511_REG_CSC_LOWER(i),
				     coeff[i] & 0xff);
		}
	}

	if (enable)
		regmap_update_bits(adv7511->regmap, ADV7511_REG_CSC_UPPER(0),
				   0xe0, 0x80 | (scaling_factor << 5));
	else
		regmap_update_bits(adv7511->regmap, ADV7511_REG_CSC_UPPER(0),
				   0x80, 0x00);

	regmap_update_bits(adv7511->regmap, ADV7511_REG_CSC_UPPER(1),
			   ADV7511_CSC_UPDATE_MODE, 0);
}

static int adv7511_packet_enable(struct adv7511 *adv7511, unsigned int packet)
{
	if (packet & 0xff)
		regmap_update_bits(adv7511->regmap, ADV7511_REG_PACKET_ENABLE0,
				   packet, 0xff);

	if (packet & 0xff00) {
		packet >>= 8;
		regmap_update_bits(adv7511->regmap, ADV7511_REG_PACKET_ENABLE1,
				   packet, 0xff);
	}

	return 0;
}

static int adv7511_packet_disable(struct adv7511 *adv7511, unsigned int packet)
{
	if (packet & 0xff)
		regmap_update_bits(adv7511->regmap, ADV7511_REG_PACKET_ENABLE0,
				   packet, 0x00);

	if (packet & 0xff00) {
		packet >>= 8;
		regmap_update_bits(adv7511->regmap, ADV7511_REG_PACKET_ENABLE1,
				   packet, 0x00);
	}

	return 0;
}

/* Coefficients for adv7511 color space conversion */
static const uint16_t adv7511_csc_ycbcr_to_rgb[] = {
	0x0734, 0x04ad, 0x0000, 0x1c1b,
	0x1ddc, 0x04ad, 0x1f24, 0x0135,
	0x0000, 0x04ad, 0x087c, 0x1b77,
};

static void adv7511_set_config_csc(struct adv7511 *adv7511,
				   u32 color_formats,
				   bool rgb, bool hdmi_mode)
{
	struct adv7511_video_config config;
	bool output_format_422, output_format_ycbcr;
	unsigned int mode;
	uint8_t infoframe[17];

	config.hdmi_mode = hdmi_mode;

	hdmi_avi_infoframe_init(&config.avi_infoframe);

	config.avi_infoframe.scan_mode = HDMI_SCAN_MODE_UNDERSCAN;

	if (rgb) {
		config.csc_enable = false;
		config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
	} else {
		config.csc_scaling_factor = ADV7511_CSC_SCALING_4;
		config.csc_coefficents = adv7511_csc_ycbcr_to_rgb;

		if ((color_formats & DRM_COLOR_FORMAT_YCRCB422) && config.hdmi_mode) {
			config.csc_enable = false;
			config.avi_infoframe.colorspace =
				HDMI_COLORSPACE_YUV422;
		} else {
			config.csc_enable = true;
			config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
		}
	}

	if (config.hdmi_mode) {
		mode = ADV7511_HDMI_CFG_MODE_HDMI;

		switch (config.avi_infoframe.colorspace) {
		case HDMI_COLORSPACE_YUV444:
			output_format_422 = false;
			output_format_ycbcr = true;
			break;
		case HDMI_COLORSPACE_YUV422:
			output_format_422 = true;
			output_format_ycbcr = true;
			break;
		default:
			output_format_422 = false;
			output_format_ycbcr = false;
			break;
		}
	} else {
		mode = ADV7511_HDMI_CFG_MODE_DVI;
		output_format_422 = false;
		output_format_ycbcr = false;
	}

	adv7511_packet_disable(adv7511, ADV7511_PACKET_ENABLE_AVI_INFOFRAME);

	adv7511_set_colormap(adv7511, config.csc_enable,
			     config.csc_coefficents,
			     config.csc_scaling_factor);

	/* Originally, all set Bit[7+0], but ADV7x35 must set Bit[6+0] */
	if (adv7511->type == ADV7533 || adv7511->type == ADV7535)
		regmap_update_bits(adv7511->regmap, ADV7511_REG_VIDEO_INPUT_CFG1, 0x41,
			(output_format_422 << 6) | output_format_ycbcr);
	else
		regmap_update_bits(adv7511->regmap, ADV7511_REG_VIDEO_INPUT_CFG1, 0x81,
			(output_format_422 << 7) | output_format_ycbcr);

	regmap_update_bits(adv7511->regmap, ADV7511_REG_HDCP_HDMI_CFG,
			   ADV7511_HDMI_CFG_MODE_MASK, mode);

	hdmi_avi_infoframe_pack(&config.avi_infoframe, infoframe,
				sizeof(infoframe));

	/* The AVI infoframe id is not configurable */
	regmap_bulk_write(adv7511->regmap, ADV7511_REG_AVI_INFOFRAME_VERSION,
			  infoframe + 1, sizeof(infoframe) - 1);

	adv7511_packet_enable(adv7511, ADV7511_PACKET_ENABLE_AVI_INFOFRAME);
}

static void __adv7511_power_on(struct adv7511 *adv7511)
{
	u32 value = 0;
//	adv7511->current_edid_segment = -1;

	/* 01-02 Power */
	regmap_update_bits(adv7511->regmap, ADV7511_REG_POWER,
			   ADV7511_POWER_POWER_DOWN, 0);
	if (adv7511->i2c_main->irq) {
		/*
		 * Documentation says the INT_ENABLE registers are reset in
		 * POWER_DOWN mode. My 7511w preserved the bits, however.
		 * Still, let's be safe and stick to the documentation.
		 */
		regmap_write(adv7511->regmap, ADV7511_REG_INT_ENABLE(0),
			     ADV7511_INT0_EDID_READY | ADV7511_INT0_HPD);
		regmap_update_bits(adv7511->regmap,
				   ADV7511_REG_INT_ENABLE(1),
				   ADV7511_INT1_DDC_ERROR,
				   ADV7511_INT1_DDC_ERROR);
	}

	/*
	 * 01-01 HPD Manual Override
	 * Per spec it is allowed to pulse the HPD signal to indicate that the
	 * EDID information has changed. Some monitors do this when they wakeup
	 * from standby or are enabled. When the HPD goes low the adv7511 is
	 * reset and the outputs are disabled which might cause the monitor to
	 * go to standby again. To avoid this we ignore the HPD pin for the
	 * first few seconds after enabling the output.
	 */
	/* originaly, ADV7511_REG_POWER2_HPD_SRC_NONE was written for all, 
	   but in ADV7x35 the 8. bit is reserved zero */
	if (adv7511->type == ADV7533 || adv7511->type == ADV7535)
		value = ADV7511_REG_POWER2_HPD_SRC_HPD;
	else
		value = ADV7511_REG_POWER2_HPD_SRC_NONE;
	regmap_update_bits(adv7511->regmap, ADV7511_REG_POWER2,
			   ADV7511_REG_POWER2_HPD_SRC_MASK, value);
}

static void adv7511_power_on(struct adv7511 *adv7511)
{
	__adv7511_power_on(adv7511);

	if (adv7511->type == ADV7533 || adv7511->type == ADV7535)
		adv7533_dsi_power_on(adv7511);
	adv7511->powered = true;
}

static void __adv7511_power_off(struct adv7511 *adv7511)
{
	/* TODO: setup additional power down modes */
	regmap_update_bits(adv7511->regmap, ADV7511_REG_POWER,
			   ADV7511_POWER_POWER_DOWN,
			   ADV7511_POWER_POWER_DOWN);
	regmap_update_bits(adv7511->regmap,
			   ADV7511_REG_INT_ENABLE(1),
			   ADV7511_INT1_DDC_ERROR, 0);
}

static void adv7511_power_off(struct adv7511 *adv7511)
{
	__adv7511_power_off(adv7511);
	if (adv7511->type == ADV7533 || adv7511->type == ADV7535)
		adv7533_dsi_power_off(adv7511);
	adv7511->powered = false;
}

static enum drm_mode_status adv7511_mode_valid(struct adv7511 *adv7511,
			      const struct drm_display_mode *mode)
{
	size_t i, num_modes = ARRAY_SIZE(valid_clocks);
	bool clock_ok = false;

	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;

	for (i = 0; i < num_modes; i++)
		if (mode->clock == valid_clocks[i]) {
			clock_ok = true;
			break;
		}

	if (!clock_ok)
		return MODE_NOCLOCK;

	return MODE_OK;
}

static void adv7511_mode_set(struct adv7511 *adv7511,
			     const struct drm_display_mode *mode)
{
	int vrefresh;
	unsigned int low_refresh_rate;
	unsigned int hsync_polarity = 0;
	unsigned int vsync_polarity = 0;

	if (adv7511->embedded_sync) {
		hsync_polarity = !(mode->flags & DRM_MODE_FLAG_PHSYNC);
		vsync_polarity = !(mode->flags & DRM_MODE_FLAG_PVSYNC);
	} else {
		enum adv7511_sync_polarity mode_hsync_polarity;
		enum adv7511_sync_polarity mode_vsync_polarity;

		/**
		 * If the input signal is always low or always high we want to
		 * invert or let it passthrough depending on the polarity of the
		 * current mode.
		 **/
		if (mode->flags & DRM_MODE_FLAG_NHSYNC)
			mode_hsync_polarity = ADV7511_SYNC_POLARITY_LOW;
		else
			mode_hsync_polarity = ADV7511_SYNC_POLARITY_HIGH;

		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			mode_vsync_polarity = ADV7511_SYNC_POLARITY_LOW;
		else
			mode_vsync_polarity = ADV7511_SYNC_POLARITY_HIGH;

		if (adv7511->hsync_polarity != mode_hsync_polarity &&
		    adv7511->hsync_polarity !=
		    ADV7511_SYNC_POLARITY_PASSTHROUGH)
			hsync_polarity = 1;

		if (adv7511->vsync_polarity != mode_vsync_polarity &&
		    adv7511->vsync_polarity !=
		    ADV7511_SYNC_POLARITY_PASSTHROUGH)
			vsync_polarity = 1;
	}

	vrefresh = drm_mode_vrefresh(mode);
	if (vrefresh <= 24)
		low_refresh_rate = ADV7511_LOW_REFRESH_RATE_24HZ;
	else if (vrefresh <= 25)
		low_refresh_rate = ADV7511_LOW_REFRESH_RATE_25HZ;
	else if (vrefresh <= 30)
		low_refresh_rate = ADV7511_LOW_REFRESH_RATE_30HZ;
	else
		low_refresh_rate = ADV7511_LOW_REFRESH_RATE_NONE;

	if (adv7511->type == ADV7535)
		regmap_update_bits(adv7511->regmap, 0x4a,
			0xc, low_refresh_rate << 2);
	else
		regmap_update_bits(adv7511->regmap, 0xfb,
			0x6, low_refresh_rate << 1);

	regmap_update_bits(adv7511->regmap, 0x17,
		0x60, (vsync_polarity << 6) | (hsync_polarity << 5));

	drm_mode_copy(&adv7511->curr_mode, mode);

	/*
	 * TODO Test first order 4:2:2 to 4:4:4 up conversion method, which is
	 * supposed to give better results.
	 */

	adv7511->f_tmds = mode->clock;
}

/* -----------------------------------------------------------------------------
 * DRM Bridge Operations
 */

void adv7511_bridge_enable(struct i2c_client *i2c_main)
{
	struct adv7511 *adv = i2c_get_clientdata(i2c_main);

	adv7511_power_on(adv);
}

void adv7511_bridge_disable(struct i2c_client *i2c_main)
{
	struct adv7511 *adv = i2c_get_clientdata(i2c_main);

	adv7511_power_off(adv);
}

enum drm_mode_status adv7511_bridge_mode_valid(struct i2c_client *i2c_main,
					   const struct videomode *vm)
{
	struct adv7511 *adv = i2c_get_clientdata(i2c_main);
	struct drm_display_mode mode = { 0 };

	/* Unlike original driver, we need to convert from struct videomode */
	drm_display_mode_from_videomode(vm, &mode);
	return adv7511_mode_valid(adv, &mode);
}

void adv7511_bridge_mode_set(struct i2c_client *i2c_main,
				    const struct videomode *vm)
{
	struct adv7511 *adv = i2c_get_clientdata(i2c_main);
	struct drm_display_mode mode = { 0 };

	/* Unlike original driver, we need to convert from struct videomode */
	drm_display_mode_from_videomode(vm, &mode);
	/* Originaly was called from adv7511_get_modes (get_edid).
	   Since only fixed mode is set, it was moved here. */
	adv7511_set_config_csc(adv, DRM_COLOR_FORMAT_RGB444, adv->rgb, true);

	adv7511_mode_set(adv, &mode);
}

int adv7511_bridge_attach(struct i2c_client *i2c_main)
{
	struct adv7511 *adv = i2c_get_clientdata(i2c_main);
	int ret = 0;

	if (adv->type == ADV7533 || adv->type == ADV7535)
		ret = adv7533_attach_dsi(adv);

	if (adv->i2c_main->irq)
		regmap_write(adv->regmap, ADV7511_REG_INT_ENABLE(0),
			     ADV7511_INT0_HPD);

	return ret;
}

void adv7511_bridge_detach(struct i2c_client *i2c_main)
{
	struct adv7511 *adv = i2c_get_clientdata(i2c_main);

	if (adv->i2c_main->irq)
		regmap_write(adv->regmap, ADV7511_REG_INT_ENABLE(0), 0);

	if (adv->type == ADV7533 || adv->type == ADV7535)
		adv7533_detach_dsi(adv);
}

/* -----------------------------------------------------------------------------
 * Probe & remove
 */

static int adv7511_init_cec_regmap(struct adv7511 *adv)
{
	int ret;

	i2c_set_clientdata(adv->i2c_cec, adv);

	adv->i2c_cec->addr = (unsigned short)adv->addr_cec;
	adv->regmap_cec = devm_regmap_init_i2c(adv->i2c_cec);
	if (IS_ERR(adv->regmap_cec)) {
		ret = PTR_ERR(adv->regmap_cec);
		return ret;
	}

	if (adv->type == ADV7533 || adv->type == ADV7535) {
		ret = adv7533_patch_cec_registers(adv);
		if (ret)
			goto err;
	}

	return 0;
err:
	regmap_release_i2c(adv->regmap_cec);
	return ret;
}

static const struct of_device_id adv7511_of_ids[] = {
	{ .compatible = "adi,adv7511", .data = (void *)ADV7511 },
	{ .compatible = "adi,adv7511w", .data = (void *)ADV7511 },
	{ .compatible = "adi,adv7513", .data = (void *)ADV7511 },
	{ .compatible = "adi,adv7533", .data = (void *)ADV7533 },
	{ .compatible = "adi,adv7535", .data = (void *)ADV7535 },
	{/* sentinel */ 0 }
};

int adv7511_probe(struct i2c_client *i2c_main,
				struct i2c_client *i2c_edid,
				struct i2c_client *i2c_cec,
				u8 num_lanes, u8 channel_id)
{
	struct adv7511 *adv7511;
	struct device *dev = &i2c_main->dev;
	const struct of_device_id *of_id =
			of_match_device(adv7511_of_ids, dev);
	uint64_t tmp;
	unsigned int main_i2c_addr = ADV7535_MAIN_ADDR << 1;
	unsigned int edid_i2c_addr = main_i2c_addr + 4;
	unsigned int cec_i2c_addr = main_i2c_addr - 2;
	unsigned int pkt_i2c_addr = main_i2c_addr - 0xa;
	unsigned int val;
	int ret;

	adv7511 = devm_kzalloc(dev, sizeof(*adv7511), GFP_KERNEL);
	if (!adv7511)
		return -ENOMEM;

	adv7511->i2c_main = i2c_main;
	adv7511->i2c_main->addr = ADV7535_MAIN_ADDR;
	adv7511->i2c_main->irq = 0;
	adv7511->i2c_cec = i2c_cec;
	adv7511->i2c_edid = i2c_edid;
	adv7511->powered = false;
	tmp = (uint64_t)of_id->data;
	adv7511->type = (enum adv7511_type) tmp;

	i2c_set_clientdata(i2c_main, adv7511);

	ret = adv7533_parse_dt(adv7511, num_lanes, channel_id);
	if (ret)
		return ret;

	if (adv7511->addr_cec != 0)
		cec_i2c_addr = adv7511->addr_cec << 1;
	else
		adv7511->addr_cec = cec_i2c_addr >> 1;

	if (adv7511->addr_edid != 0)
		edid_i2c_addr = adv7511->addr_edid << 1;
	else
		adv7511->addr_edid = edid_i2c_addr >> 1;

	if (adv7511->addr_pkt != 0)
		pkt_i2c_addr = adv7511->addr_pkt << 1;
	else
		adv7511->addr_pkt = pkt_i2c_addr >> 1;

	adv7511->regmap = devm_regmap_init_i2c(i2c_main);
	if (IS_ERR(adv7511->regmap)) {
		ret = PTR_ERR(adv7511->regmap);
		return ret;
	}

	ret = regmap_read(adv7511->regmap, ADV7511_REG_CHIP_REVISION, &val);
	if (ret)
		goto err_i2c_unregister_main;
	dev_dbg(dev, "Rev. %d\n", val);

	ret = adv7533_patch_registers(adv7511);
	if (ret)
		goto err_i2c_unregister_main;

	/* Originaly, all were written 0xFFFF, but Rm states
	   bit[0] of 0x44 (ADV7511_REG_PACKET_ENABLE1) must be 1 */
	if (adv7511->type == ADV7533 || adv7511->type == ADV7535)
		adv7511_packet_disable(adv7511, 0xfeff);
	else
		adv7511_packet_disable(adv7511, 0xffff);

	regmap_write(adv7511->regmap, ADV7511_REG_EDID_I2C_ADDR,
			edid_i2c_addr);

	adv7511->i2c_edid->addr = (unsigned short)adv7511->addr_edid;

	regmap_write(adv7511->regmap, ADV7511_REG_PACKET_I2C_ADDR,
			pkt_i2c_addr);

	regmap_write(adv7511->regmap, ADV7511_REG_CEC_I2C_ADDR,
			cec_i2c_addr);

	ret = adv7511_init_cec_regmap(adv7511);
	if (ret)
		goto err_i2c_unregister_main;

	adv7511_power_off(adv7511);

	ret = adv7511_cec_init(dev, adv7511);
	if (ret)
		goto err_unregister_cec;

	return 0;

err_unregister_cec:
	regmap_release_i2c(adv7511->regmap_cec);
	adv7511->regmap_cec = NULL;
err_i2c_unregister_main:
	regmap_release_i2c(adv7511->regmap);
	adv7511->regmap = NULL;

	return ret;
}

int adv7511_remove(struct i2c_client *i2c_main)
{
	struct adv7511 *adv7511 = i2c_get_clientdata(i2c_main);

	if (adv7511) {
		if (adv7511->regmap_cec)
			regmap_release_i2c(adv7511->regmap_cec);
		if (adv7511->regmap)
			regmap_release_i2c(adv7511->regmap);
		devm_kfree(&i2c_main->dev, adv7511);
	}

	return 0;
}
