// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 * Copyright 2022 NXP
 */

#include "mipi_dsi/adv7511.h"

static const struct reg_sequence adv7533_fixed_registers[] = {
	{ 0x16, 0x20 },
	{ 0x9a, 0xe0 },
	{ 0xba, 0x70 },
	{ 0xde, 0x82 },
	{ 0xe4, 0x40 },
	{ 0xe5, 0x80 },
};

static const struct reg_sequence adv7533_cec_fixed_registers[] = {
	{ 0x15, 0xd0 },
	{ 0x17, 0xd0 },
	{ 0x24, 0x20 },
	{ 0x57, 0x11 },
	{ 0x05, 0xc8 },
};

static void adv7511_dsi_config_timing_gen(struct adv7511 *adv)
{
	struct drm_display_mode *mode = &adv->curr_mode;
	unsigned int hsw, hfp, hbp, vsw, vfp, vbp;

	hsw = mode->hsync_end - mode->hsync_start;
	hfp = mode->hsync_start - mode->hdisplay;
	hbp = mode->htotal - mode->hsync_end;
	vsw = mode->vsync_end - mode->vsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;

	/* 03-01 Enable Internal Timing Generator */
	regmap_write(adv->regmap_cec, 0x27, 0xcb);

	/* 03-08 Timing Configuration */

	/* horizontal porch params */
	regmap_write(adv->regmap_cec, 0x28, mode->htotal >> 4);
	regmap_write(adv->regmap_cec, 0x29, (mode->htotal << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x2a, hsw >> 4);
	regmap_write(adv->regmap_cec, 0x2b, (hsw << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x2c, hfp >> 4);
	regmap_write(adv->regmap_cec, 0x2d, (hfp << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x2e, hbp >> 4);
	regmap_write(adv->regmap_cec, 0x2f, (hbp << 4) & 0xff);

	/* vertical porch params */
	regmap_write(adv->regmap_cec, 0x30, mode->vtotal >> 4);
	regmap_write(adv->regmap_cec, 0x31, (mode->vtotal << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x32, vsw >> 4);
	regmap_write(adv->regmap_cec, 0x33, (vsw << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x34, vfp >> 4);
	regmap_write(adv->regmap_cec, 0x35, (vfp << 4) & 0xff);
	regmap_write(adv->regmap_cec, 0x36, vbp >> 4);
	regmap_write(adv->regmap_cec, 0x37, (vbp << 4) & 0xff);

	/* 03-03 Reset Internal Timing Generator */
	regmap_write(adv->regmap_cec, 0x27, 0xcb);
	regmap_write(adv->regmap_cec, 0x27, 0x8b);
	regmap_write(adv->regmap_cec, 0x27, 0xcb);

}

void adv7533_dsi_power_on(struct adv7511 *adv)
{
	struct mipi_dsi_device *dsi = adv->dsi;
	struct drm_display_mode *mode = &adv->curr_mode;
	u8 clock_div_by_lanes[] = { 6, 4, 3 };	/* 2, 3, 4 lanes */

	/* Gate DSI LP Oscillator */
	regmap_update_bits(adv->regmap_cec, 0x03, 0x02, 0x00);

	/* Originaly, this patch was only in probe, but subsequent power_down
	   resets the register. Do the patch once again here. */
	adv7533_patch_registers(adv);
	
	/* 01-03 Initialisation (Fixed) Registers */
	regmap_register_patch(adv->regmap_cec, adv7533_cec_fixed_registers,
			      ARRAY_SIZE(adv7533_cec_fixed_registers));

	/* 02-04 DSI Lanes */
	regmap_write(adv->regmap_cec, 0x1c, dsi->lanes << 4);

	/* 02-05 DSI Pixel Clock Divider */
	regmap_write(adv->regmap_cec, 0x16,
		     clock_div_by_lanes[dsi->lanes - 2] << 3);

	if (adv->use_timing_gen)
		adv7511_dsi_config_timing_gen(adv);
	else
		regmap_write(adv->regmap_cec, 0x27, 0x0b);

	/* 04-01 HDMI Output */
	regmap_write(adv->regmap, 0xaf, 0x16);

	/* 09-03 AVI Infoframe - RGB - 16-9 Aspect Ratio */
	regmap_write(adv->regmap, ADV7511_REG_AVI_INFOFRAME(0), 0x10);
	if (FORMAT_RATIO(mode->hdisplay, mode->vdisplay) == RATIO_16_9)
		regmap_write(adv->regmap, ADV7511_REG_AVI_INFOFRAME(1), 0x28);
	else if (FORMAT_RATIO(mode->hdisplay, mode->vdisplay) == RATIO_4_3)
		regmap_write(adv->regmap, ADV7511_REG_AVI_INFOFRAME(1), 0x18);

	/* 04-04 GC Packet Enable */
	regmap_write(adv->regmap, ADV7511_REG_PACKET_ENABLE0, 0x80);

	/* 04-06 GC Colour Depth - 24 Bit */
	regmap_write(adv->regmap, 0x4c, 0x04);

	/* 04-09 Down Dither Output Colour Depth - 8 Bit (default) */
	regmap_write(adv->regmap, 0x49, 0x00);

	/* 07-01 CEC Power Mode - Always Active */
	regmap_write(adv->regmap_cec, 0xbe, 0x3d);

	/* 04-03 HDMI Output Enable  */
	regmap_write(adv->regmap_cec, 0x03, 0x89);
	/* disable test mode */
	regmap_write(adv->regmap_cec, 0x55, 0x00);
}

void adv7533_dsi_power_off(struct adv7511 *adv)
{
	/* disable hdmi */
	regmap_write(adv->regmap_cec, 0x03, 0x0b);
	/* disable internal timing generator */
	regmap_write(adv->regmap_cec, 0x27, 0x0b);
}

int adv7533_patch_registers(struct adv7511 *adv)
{
	return regmap_register_patch(adv->regmap,
				     adv7533_fixed_registers,
				     ARRAY_SIZE(adv7533_fixed_registers));
}

int adv7533_patch_cec_registers(struct adv7511 *adv)
{
	return regmap_register_patch(adv->regmap_cec,
				    adv7533_cec_fixed_registers,
				    ARRAY_SIZE(adv7533_cec_fixed_registers));
}

int adv7533_attach_dsi(struct adv7511 *adv)
{
	struct device *dev = &adv->i2c_main->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret = 0;
	struct mipi_dsi_device_info info;

	strcpy(info.type, "adv7533");
	info.channel = adv->channel_id;
	info.node = NULL;

	host = of_find_mipi_dsi_host_by_node(adv->host_node);
	if (!host) {
		dev_err(dev, "failed to find dsi host\n");
		return -EINVAL;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		dev_err(dev, "failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_device;
	}

	adv->dsi = dsi;

	dsi->lanes = adv->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "failed to attach dsi to host\n");
		goto err_dsi_attach;
	}

	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_device:
	return ret;
}

void adv7533_detach_dsi(struct adv7511 *adv)
{
	if (adv->dsi) {
		mipi_dsi_detach(adv->dsi);
		mipi_dsi_device_unregister(adv->dsi);
	}
}

int adv7533_parse_dt(struct adv7511 *adv, u8 num_lanes, u8 channel_id)
{
	struct device *dev = &adv->i2c_main->dev;

	if (num_lanes < 1 || num_lanes > 4) {
		dev_err(dev, "Invalid dsi-lanes: %d\n", num_lanes);
		return -EINVAL;
	}

	if (channel_id > 3) {
		dev_err(dev, "Invalid dsi-channel: %d\n", channel_id);
		return -EINVAL;
	}

	adv->num_dsi_lanes = num_lanes;
	adv->channel_id = channel_id;
	adv->host_node = &dev->parent->of_node;
	adv->use_timing_gen = true;

	adv->addr_cec = 0x3b;
	adv->addr_edid = 0;
	adv->addr_pkt = 0;

	adv->rgb = true;
	adv->embedded_sync = false;

	/* Added in comparison with original driver otherwise uninitialized */
	adv->vsync_polarity = ADV7511_SYNC_POLARITY_PASSTHROUGH;
	adv->hsync_polarity = ADV7511_SYNC_POLARITY_PASSTHROUGH;

	return 0;
}

#ifdef ADV7535_DEBUG
/**
  Dump ADV7535 registers.
**/
#include "wdm.h"

void adv7535_dumpregs(struct i2c_client *i2c_main, u32 reg_area)
{
    struct adv7511 *adv = i2c_get_clientdata(i2c_main);
    unsigned int i;
    unsigned int status = 0;

    if (reg_area & ADV7535_DEBUG_MAIN) {
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "------------------------ADV7535-------------------------------------\n");
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "----------------------------MAIN_REGS-------------------------------\n");
        regmap_read(adv->regmap, 0x16, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x16 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x17, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x17 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x18, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x18 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x20, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x20 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x40, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x40 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x41, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x41 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x43, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x43 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x44, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x44 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x45, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x45 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x49, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x49 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x4a, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x4a = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x4c, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x4c = 0x%08X\n", status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "----------------------------MAIN_REGS infoframe begin----------------\n");
        for (i = 0x52; i <= 0x6f; i++) {
            regmap_read(adv->regmap, i, &status);
            DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x%08X = 0x%08X\n", i, status);
        }
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "----------------------------MAIN_REGS infoframe end  ----------------\n");
        regmap_read(adv->regmap, 0x94, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x94 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x95, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x95 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0x9a, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x9a = 0x%08X\n", status);
        regmap_read(adv->regmap, 0xaf, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xaf = 0x%08X\n", status);
        regmap_read(adv->regmap, 0xba, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xba = 0x%08X\n", status);
        regmap_read(adv->regmap, 0xd6, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xd6 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0xde, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xde = 0x%08X\n", status);
        regmap_read(adv->regmap, 0xe1, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xe1 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0xe2, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xe2 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0xe4, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xe4 = 0x%08X\n", status);
        regmap_read(adv->regmap, 0xe5, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xe5 = 0x%08X\n", status);
    }
    if (reg_area & ADV7535_DEBUG_CEC) {
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "--------------------------------CEC REGS ------------------------------\n");
        regmap_read(adv->regmap_cec, 0x03, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x03 = 0x%08X\n", status);
        regmap_read(adv->regmap_cec, 0x05, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x05 = 0x%08X\n", status);
        regmap_read(adv->regmap_cec, 0x15, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x15 = 0x%08X\n", status);
        regmap_read(adv->regmap_cec, 0x16, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x16 = 0x%08X\n", status);
        regmap_read(adv->regmap_cec, 0x17, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x17 = 0x%08X\n", status);
        regmap_read(adv->regmap_cec, 0x1c, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x1c = 0x%08X\n", status);
        regmap_read(adv->regmap_cec, 0x24, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x24 = 0x%08X\n", status);
        for (i = 0x27; i <= 0x38; i++) {
            regmap_read(adv->regmap, i, &status);
            DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x%08X = 0x%08X\n", i, status);
        }
        regmap_read(adv->regmap_cec, 0x55, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x55 = 0x%08X\n", status);
        regmap_read(adv->regmap_cec, 0x57, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x57 = 0x%08X\n", status);
        regmap_read(adv->regmap_cec, 0xbe, &status);
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xbe = 0x%08X\n", status);
    }
}
#endif
