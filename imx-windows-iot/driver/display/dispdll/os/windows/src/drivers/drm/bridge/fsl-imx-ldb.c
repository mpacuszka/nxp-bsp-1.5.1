// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012 Sascha Hauer, Pengutronix
 * Copyright 2020 NXP
 */
/*
 * Modifications Copyright 2022 NXP
 */

#include <linux/media-bus-format.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <drm/bridge/fsl_imx_ldb.h>
#include <drm/drm_panel.h>

#include <string.h>

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_CH1_MODE_EN_TO_DI0		(1 << 2)
#define LDB_CH1_MODE_EN_TO_DI1		(3 << 2)
#define LDB_CH1_MODE_EN_MASK		(3 << 2)
#define LDB_SPLIT_MODE_EN		(1 << 4)
#define LDB_DATA_WIDTH_CH0_24		(1 << 5)
#define LDB_BIT_MAP_CH0_JEIDA		(1 << 6)
#define LDB_DATA_WIDTH_CH1_24		(1 << 7)
#define LDB_BIT_MAP_CH1_JEIDA		(1 << 8)
#define LDB_DI0_VS_POL_ACT_LOW		(1 << 9)
#define LDB_DI1_VS_POL_ACT_LOW		(1 << 10)

struct ldb_bit_mapping {
	u32 bus_format;
	u32 datawidth;
	const char * const mapping;
};

static const struct ldb_bit_mapping ldb_bit_mappings[] = {
	{ MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,  18, "spwg" },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,  24, "spwg" },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA, 24, "jeida" },
};

static int of_get_bus_format(struct device *dev, struct device_node *np)
{
	const char *bm;
	u32 datawidth = 0;
	int ret, i;

	ret = of_property_read_string(np, "fsl,data-mapping", &bm);
	if (ret < 0)
		return ret;

	of_property_read_u32(np, "fsl,data-width", &datawidth);

	for (i = 0; i < ARRAY_SIZE(ldb_bit_mappings); i++) {
		if (!_stricmp(bm, ldb_bit_mappings[i].mapping) &&
		    datawidth == ldb_bit_mappings[i].datawidth)
			return ldb_bit_mappings[i].bus_format;
	}

	dev_err(dev, "invalid data mapping: %d-bit \"%s\"\n", datawidth, bm);

	return -ENOENT;
}

static inline struct ldb_channel *bridge_to_ldb_ch(struct drm_bridge *b)
{
	return container_of(b, struct ldb_channel, bridge);
}

static void ldb_ch_set_bus_format(struct ldb_channel *ldb_ch, u32 bus_format)
{
	struct ldb *ldb = ldb_ch->ldb;

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH0_24;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH1_24;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH0_24 |
					 LDB_BIT_MAP_CH0_JEIDA;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH1_24 |
					 LDB_BIT_MAP_CH1_JEIDA;
		break;
	}
}

static void ldb_bridge_mode_set(struct drm_bridge *bridge,
				const struct drm_display_mode *mode,
				const struct drm_display_mode *adjusted_mode)
{
	struct ldb_channel *ldb_ch = bridge_to_ldb_ch(bridge);
	struct ldb *ldb = ldb_ch->ldb;

	/* FIXME - assumes straight connections DI0 --> CH0, DI1 --> CH1 */
	if (ldb_ch == ldb->channel[0] || ldb->dual) {
		if (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC)
			ldb->ldb_ctrl |= LDB_DI0_VS_POL_ACT_LOW;
		else if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
			ldb->ldb_ctrl &= ~LDB_DI0_VS_POL_ACT_LOW;
	}
	if (ldb_ch == ldb->channel[1] || ldb->dual) {
		if (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC)
			ldb->ldb_ctrl |= LDB_DI1_VS_POL_ACT_LOW;
		else if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
			ldb->ldb_ctrl &= ~LDB_DI1_VS_POL_ACT_LOW;
	}

	ldb_ch_set_bus_format(ldb_ch, ldb_ch->bus_format);
}

static void ldb_bridge_enable(struct drm_bridge *bridge)
{
	struct ldb_channel *ldb_ch = bridge_to_ldb_ch(bridge);
	struct ldb *ldb = ldb_ch->ldb;

	regmap_write(ldb->regmap, ldb->ctrl_reg, ldb->ldb_ctrl);
}

static void ldb_bridge_disable(struct drm_bridge *bridge)
{
	struct ldb_channel *ldb_ch = bridge_to_ldb_ch(bridge);
	struct ldb *ldb = ldb_ch->ldb;

	if (ldb_ch == ldb->channel[0] || ldb->dual)
		ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
	if (ldb_ch == ldb->channel[1] || ldb->dual)
		ldb->ldb_ctrl &= ~LDB_CH1_MODE_EN_MASK;

	regmap_write(ldb->regmap, ldb->ctrl_reg, ldb->ldb_ctrl);
}

static int ldb_bridge_attach(struct drm_bridge *bridge)
{
	struct ldb_channel *ldb_ch = bridge_to_ldb_ch(bridge);
	struct ldb *ldb = ldb_ch->ldb;

	if (!bridge->encoder) {
		dev_err(ldb->dev, "failed to find encoder object\n");
		return -ENODEV;
	}

	if (!ldb_ch->next_bridge)
		return 0;

	return 0;
}

int ldb_bind(struct ldb *ldb, struct drm_encoder **encoder)
{
	struct device *dev = ldb->dev;
	struct device_node *np = &dev->of_node;
	struct device_node *child;
	struct platform_device *pdev = to_platform_device(dev);
	struct regmap_config regmap_cfg;
	struct resource *res;
	int ret = 0;
	int i, j;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	regmap_cfg.val_bits = 32;
	ldb->regmap = devm_regmap_init_mmio(dev, res, &regmap_cfg);
	if (IS_ERR(ldb->regmap)) {
		dev_err(dev, "Failed to get regmap\n");
		return PTR_ERR(ldb->regmap);
	}

	/* disable LDB by resetting the control register to POR default */
	regmap_write(ldb->regmap, ldb->ctrl_reg, 0);

	ldb->dual = of_property_read_bool(np, "fsl,dual-channel");
	if (ldb->dual)
		ldb->ldb_ctrl |= LDB_SPLIT_MODE_EN;

	for (j = 0; j < np->num_childs; j++) {
		struct ldb_channel *ldb_ch;
		int bus_format;
		child = &np->child[j];
		if (!child) {
			ret = -EINVAL;
			goto free_child;
		}

		ret = of_property_read_u32(child, "reg", &(u32)i);
		if (ret || i < 0 || i > 1) {
			ret = -EINVAL;
			goto free_child;
		}

		if (ldb->dual && i > 0) {
			dev_warn(dev, "dual-channel mode, ignoring second output\n");
			continue;
		}

		ldb_ch = ldb->channel[i];
		ldb_ch->ldb = ldb;
		ldb_ch->chno = i;
		ldb_ch->is_valid = false;

		bus_format = of_get_bus_format(dev, child);
		if (bus_format == -EINVAL) {
			dev_err(dev, "could not determine data mapping: %d\n",
				bus_format);
			ret = bus_format;
			goto free_child;
		}
		ldb_ch->bus_format = bus_format;
		ldb_ch->child = child;

		ldb_ch->is_valid = true;
	}

	return 0;

free_child:
	return ret;
}
