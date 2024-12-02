// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
 * Copyright 2022 NXP
 *
 * derived from imx-hdmi.c(renamed to bridge/dw_hdmi.c now)
 */

#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/of_device.h>

#include <bridge/synopsys/dw_hdmi.h>
#include <drm/drm_edid.h>
#include "video/videomode.h"

#include "imx8mp-hdmi-pavi.h"
#include "clk/mediamix_mp.h"
#include "uapi/linux/media-bus-format.h"

/* GPR reg */
struct imx_hdmi_chip_data {
	int	reg_offset;
	u32	mask_bits;
	u32	shift_bit;
};

struct imx_hdmi {
	struct device *dev;
	struct dw_hdmi *hdmi;
	struct regmap *regmap;
	const struct imx_hdmi_chip_data *chip_data;
};

struct clk_bulk_data imx8mp_clocks[] = {
	{ .id = "pix_clk"  },
	{ .id = "phy_int"  },
	{ .id = "prep_clk" },
	{ .id = "skp_clk"  },
	{ .id = "sfr_clk"  },
	{ .id = "cec_clk"  },
	{ .id = "apb_clk"  },
	{ .id = "hpi_clk"  },
	{ .id = "fdcc_ref" },
	{ .id = "pipe_clk" },
};

int dw_hdmi_imx_atomic_check(u32 *bus_format, struct videomode* vm)
{

	if (bus_format)
		*bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	if (vm) {
		vm->flags = (enum display_flags)(vm->flags & ~DISPLAY_FLAGS_DE_LOW);
		vm->flags = (enum display_flags)(vm->flags | DISPLAY_FLAGS_DE_HIGH);
	}
	return 0;
}

static enum drm_mode_status
imx8mp_hdmi_mode_valid(struct dw_hdmi *hdmi, void *data,
		       const struct drm_display_info *info,
		       struct drm_display_mode *mode)
{
	int rate, req_rate;

	if (mode->clock < 13500)
		return MODE_CLOCK_LOW;
	if (mode->clock > 297000)
		return MODE_CLOCK_HIGH;

	/* We don't support double-clocked and Interlaced modes */
	if (mode->flags & DRM_MODE_FLAG_DBLCLK ||
			mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_BAD;

	req_rate = mode->clock * 1000;
	rate = samsung_hdmi_phy_clk_round_rate(req_rate);
	if (rate <= 0) {
		return MODE_CLOCK_RANGE;
	}
	if (rate != req_rate) {
		mode->clock = rate / 1000;
	}

	return MODE_OK;
}

static int imx8mp_hdmi_phy_init(struct dw_hdmi *dw_hdmi, void *data,
				const struct drm_display_info *display,
				const struct drm_display_mode *mode)
{
	struct imx_hdmi *hdmi = (struct imx_hdmi *)data;
	unsigned int val;

	dev_dbg(hdmi->dev, "%s\n", __func__);

	dw_hdmi_phy_reset(dw_hdmi);

	/* enable PVI */
	imx8mp_hdmi_pavi_powerup();
	imx8mp_hdmi_pvi_enable(mode);

	regmap_read(hdmi->regmap, 0x200, &val);
	/* HDMI PHY power off */
	val |= 0x8;
	regmap_write(hdmi->regmap, 0x200, val);
	/* HDMI PHY power on */
	val &= ~0x8;
	/* Enable CEC */
	val |= 0x2;
	regmap_write(hdmi->regmap, 0x200, val);

	return 0;
}

static void imx8mp_hdmi_phy_disable(struct dw_hdmi *dw_hdmi, void *data)
{
	struct imx_hdmi *hdmi = (struct imx_hdmi *)data;
	unsigned int val;

	dev_dbg(hdmi->dev, "%s\n", __func__);

	/* disable PVI */
	imx8mp_hdmi_pvi_disable();
	imx8mp_hdmi_pavi_powerdown();

	/* TODO */
	regmap_read(hdmi->regmap, 0x200, &val);
	/* Disable CEC */
	val &= ~0x2;
	/* Power down HDMI PHY
	 * TODO move PHY power off to hdmi phy driver
	 * val |= 0x8;
	 * regmap_write(hdmi->regmap, 0x200, val);
	*/
}

static int imx8mp_hdmimix_setup(struct imx_hdmi *hdmi)
{
	int ret;

	if (NULL == imx8mp_hdmi_pavi_init()) {
		dev_err(hdmi->dev, "No pavi info found\n");
		return -ENODEV;
	}

	/* device_reset */
	hdmimix_hdmi_reset(hdmi->regmap->io_target.base, false);

	ret = devm_clk_bulk_get(hdmi->dev, ARRAY_SIZE(imx8mp_clocks), imx8mp_clocks);
	if (ret < 0) {
		dev_err(hdmi->dev, "No hdmimix bulk clk got\n");
		return -ENODEV;
	}

	return clk_bulk_prepare_enable(ARRAY_SIZE(imx8mp_clocks), imx8mp_clocks);
}

static void imx8mp_hdmi_enable_audio(struct dw_hdmi *dw_hdmi, void *data, int channel,
			      int width, int rate, int non_pcm)
{
	imx8mp_hdmi_pai_enable(channel, width, rate, non_pcm);
}

static imx8mp_hdmi_disable_audio(struct dw_hdmi *dw_hdmi, void *data)
{
	imx8mp_hdmi_pai_disable();
}

static const struct dw_hdmi_phy_ops imx8mp_hdmi_phy_ops = {
	.init		= imx8mp_hdmi_phy_init,
	.disable	= imx8mp_hdmi_phy_disable,
	.read_hpd = dw_hdmi_phy_read_hpd,
	.update_hpd = dw_hdmi_phy_update_hpd,
	.setup_hpd = dw_hdmi_phy_setup_hpd,
	.enable_audio	= imx8mp_hdmi_enable_audio,
	.disable_audio  = imx8mp_hdmi_disable_audio,
};

struct imx_hdmi_chip_data imx8mp_chip_data = {
	.reg_offset = -1,
};

static const struct dw_hdmi_plat_data imx8mp_hdmi_drv_data = {
	.mode_valid = imx8mp_hdmi_mode_valid,
	.phy_data   = &imx8mp_chip_data,
	.phy_ops    = &imx8mp_hdmi_phy_ops,
	.phy_name   = "samsung_dw_hdmi_phy2",
	.phy_force_vendor = true,
};

static const struct of_device_id dw_hdmi_imx_dt_ids[] = {
	{
	  .compatible = "fsl,imx8mp-hdmi",
	  .data = &imx8mp_hdmi_drv_data
	},
	{/* sentinel */ 0 },
};

int dw_hdmi_imx_probe(struct platform_device *pdev, unsigned int uid)
{
	struct device_node *np = &pdev->dev.of_node;
	const struct of_device_id *match = of_match_device(dw_hdmi_imx_dt_ids, &pdev->dev);
	struct dw_hdmi_plat_data *plat_data;
	struct imx_hdmi *hdmi;
	struct regmap_config regmap_cfg;
	int ret;
	struct resource *res;

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	platform_set_drvdata(pdev, hdmi);
	hdmi->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -ENODEV;
	regmap_cfg.val_bits = 32;
	hdmi->regmap = devm_regmap_init_mmio(hdmi->dev, res, &regmap_cfg);
	if (IS_ERR(hdmi->regmap)) {
		dev_err(hdmi->dev, "Unable to get blk-ctrl\n");
		return PTR_ERR(hdmi->regmap);
	}

	ret = imx8mp_hdmi_pavi_probe(pdev, hdmi->regmap->io_target.base);
	if (ret < 0) {
		dev_err(hdmi->dev, "PAVI probe failed\n");
		return ret;
	}

	/* reset phy */
	hdmimix_phy_reset(hdmi->regmap->io_target.base, false);

	plat_data = devm_kmemdup(&pdev->dev, match->data,
				sizeof(*plat_data), GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;

	hdmi->chip_data = plat_data->phy_data;
	plat_data->phy_data = hdmi;
	plat_data->output_port = uid;

	if (of_device_is_compatible(&pdev->dev.of_node, "fsl,imx8mp-hdmi")) {
		ret = imx8mp_hdmimix_setup(hdmi);
		if (ret < 0) {
			devm_kfree(&pdev->dev, plat_data);
			return ret;
		}
	}

	hdmi->hdmi = dw_hdmi_probe(pdev, plat_data);
	if (IS_ERR(hdmi->hdmi)) {
		devm_kfree(&pdev->dev, plat_data);
		return PTR_ERR(hdmi->hdmi);
	}

	return 0;
}

int dw_hdmi_imx_remove(struct platform_device *pdev)
{
	struct imx_hdmi *hdmi = platform_get_drvdata(pdev);
	struct resource *res;

	if (hdmi) {
		if (hdmi->hdmi) {
			dw_hdmi_remove(hdmi->hdmi, pdev);
		}
		imx8mp_hdmi_pavi_remove(pdev);
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (res && hdmi->regmap) {
			regmap_release_mmio(hdmi->regmap, res);
		}
		devm_kfree(&pdev->dev, hdmi);
	}

	return 0;
}

void dw_hdmi_imx_dumpreg(struct platform_device* pdev)
{
	struct imx_hdmi* hdmi = platform_get_drvdata(pdev);

	hdmimix_dump_reg(hdmi->regmap->io_target.base);
}
