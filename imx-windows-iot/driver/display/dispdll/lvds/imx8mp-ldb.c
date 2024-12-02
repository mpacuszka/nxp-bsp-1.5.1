// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020,2022 NXP
 */

#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/media-bus-format.h>
#include <video/videomode.h>
#include <string.h>
#include "imx8mp-ldb.h"

#define LDB_SPLIT_MODE_EN		(1 << 4)
#define LDB_DATA_WIDTH_CH0_24		(1 << 5)
#define LDB_BIT_MAP_CH0_JEIDA		(1 << 6)
#define LDB_DATA_WIDTH_CH1_24		(1 << 7)
#define LDB_BIT_MAP_CH1_JEIDA		(1 << 8)
#define LDB_DI0_VS_POL_ACT_LOW		(1 << 9)
#define LDB_DI1_VS_POL_ACT_LOW		(1 << 10) 

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_CH1_MODE_EN_TO_DI0		(1 << 2)
#define LDB_CH1_MODE_EN_TO_DI1		(3 << 2)
#define LDB_CH1_MODE_EN_MASK		(3 << 2)
#define LDB_REG_CH0_FIFO_RESET		(1 << 11)
#define LDB_REG_CH1_FIFO_RESET		(1 << 12)
#define LDB_REG_ASYNC_FIFO_EN		(1 << 24)
#define LDB_FIFO_THRESHOLD		(4 << 25)
#define LDB_CTRL_REG		(0x5c)

struct imx8mp_ldb {
	uint8_t __iomem *base;
	struct clk *clk_root;
	bool phy_is_on[LDB_CH_NUM];
	u32 bus_format;
	unsigned int ctrl_reg;
	u32 ldb_ctrl;
	bool dual; /* dual-channel mode */
	u32 active_chan; /* single channel mode stores active chan id here */
};

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

void imx8mp_ldb_encoder_enable(struct platform_device *pdev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(&pdev->dev);
	struct device *phy_dev = pdev->dev.parent;
	u32 chan = imx8mp_ldb->active_chan;

	clk_prepare_enable(imx8mp_ldb->clk_root);

	if (chan == 0 || imx8mp_ldb->dual) {
		imx8mp_ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
		imx8mp_ldb->ldb_ctrl |= LDB_CH0_MODE_EN_TO_DI0;
	}
	if (chan == 1 || imx8mp_ldb->dual) {
		imx8mp_ldb->ldb_ctrl &= ~LDB_CH1_MODE_EN_MASK;
		imx8mp_ldb->ldb_ctrl |= imx8mp_ldb->dual ?
				LDB_CH1_MODE_EN_TO_DI0 : LDB_CH1_MODE_EN_TO_DI1;
	}

	if (imx8mp_ldb->dual) {
		imx8mp_lvds_phy_power_on(phy_dev, 0);
		imx8mp_lvds_phy_power_on(phy_dev, 1);

		imx8mp_ldb->phy_is_on[0] = true;
		imx8mp_ldb->phy_is_on[1] = true;
	} else {
		imx8mp_lvds_phy_power_on(phy_dev, chan);

		imx8mp_ldb->phy_is_on[chan] = true;
	}
	writel(imx8mp_ldb->ldb_ctrl, imx8mp_ldb->base + imx8mp_ldb->ctrl_reg);
}

static void ldb_ch_set_bus_format(struct imx8mp_ldb *imx8mp_ldb, u32 bus_format)
{
	u32 chan = imx8mp_ldb->active_chan;

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
		if (chan == 0 || imx8mp_ldb->dual)
			imx8mp_ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH0_24;
		if (chan == 1 || imx8mp_ldb->dual)
			imx8mp_ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH1_24;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		if (chan == 0 || imx8mp_ldb->dual)
			imx8mp_ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH0_24 |
			LDB_BIT_MAP_CH0_JEIDA;
		if (chan == 1 || imx8mp_ldb->dual)
			imx8mp_ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH1_24 |
			LDB_BIT_MAP_CH1_JEIDA;
		break;
	}
}

static void ldb_bridge_mode_set(struct imx8mp_ldb *imx8mp_ldb, struct videomode *vm)
{
	u32 chan = imx8mp_ldb->active_chan;

	/* FIXME - assumes straight connections DI0 --> CH0, DI1 --> CH1 */
	if (chan == 0 || imx8mp_ldb->dual) {
		if (vm->flags & DISPLAY_FLAGS_VSYNC_LOW)
			imx8mp_ldb->ldb_ctrl |= LDB_DI0_VS_POL_ACT_LOW;
		else if (vm->flags & DISPLAY_FLAGS_VSYNC_HIGH)
			imx8mp_ldb->ldb_ctrl &= ~LDB_DI0_VS_POL_ACT_LOW;
	}
	if (chan == 1 || imx8mp_ldb->dual) {
		if (vm->flags & DISPLAY_FLAGS_VSYNC_LOW)
			imx8mp_ldb->ldb_ctrl |= LDB_DI1_VS_POL_ACT_LOW;
		else if (vm->flags & DISPLAY_FLAGS_VSYNC_HIGH)
			imx8mp_ldb->ldb_ctrl &= ~LDB_DI1_VS_POL_ACT_LOW;
	}

	ldb_ch_set_bus_format(imx8mp_ldb, imx8mp_ldb->bus_format);
}

void imx8mp_ldb_encoder_atomic_mode_set(struct platform_device *pdev, struct videomode *vm)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(&pdev->dev);
	unsigned long serial_clk;

	if (vm->pixelclock > 160000000) {
		dev_warn(&pdev->dev,
			 "%s: mode exceeds 160 MHz pixel clock\n", __func__);
	}
	if (vm->pixelclock > 80000000 && !imx8mp_ldb->dual) {
		dev_warn(&pdev->dev,
			 "%s: mode exceeds 80 MHz pixel clock\n", __func__);
	}

	serial_clk = vm->pixelclock * 7UL;
	if (imx8mp_ldb->dual) {
		serial_clk /= 2;
	}
	clk_set_rate(imx8mp_ldb->clk_root, serial_clk);

	ldb_bridge_mode_set(imx8mp_ldb, vm);

}

void imx8mp_ldb_encoder_disable(struct platform_device *pdev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(&pdev->dev);
	struct device *phy_dev = pdev->dev.parent;
	u32 chan = imx8mp_ldb->active_chan;

	if (chan == 0 || imx8mp_ldb->dual)
		imx8mp_ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
	if (chan == 1 || imx8mp_ldb->dual)
		imx8mp_ldb->ldb_ctrl &= ~LDB_CH1_MODE_EN_MASK;

	writel(imx8mp_ldb->ldb_ctrl, imx8mp_ldb->base + imx8mp_ldb->ctrl_reg);

	if (imx8mp_ldb->dual) {
		imx8mp_lvds_phy_power_off(phy_dev, 0);
		imx8mp_lvds_phy_power_off(phy_dev, 1);

		imx8mp_ldb->phy_is_on[0] = false;
		imx8mp_ldb->phy_is_on[1] = false;
	} else {
		imx8mp_lvds_phy_power_off(phy_dev, chan);

		imx8mp_ldb->phy_is_on[chan] = false;
	}

	clk_disable_unprepare(imx8mp_ldb->clk_root);
}

int imx8mp_ldb_encoder_get_crtc_format(struct platform_device *pdev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(&pdev->dev);

	switch (imx8mp_ldb->bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		return MEDIA_BUS_FMT_RGB666_1X18;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		return MEDIA_BUS_FMT_RGB888_1X24;
		break;
	default:
		return -EINVAL;
	}
}

unsigned int imx8mp_ldb_encoder_get_ldb_bus_format(struct platform_device *pdev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(&pdev->dev);
	return imx8mp_ldb->bus_format;
}

static int get_bus_format(struct device *dev, u32 bus_data_width, const char * const bus_mapping)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ldb_bit_mappings); i++) {
		if (!_stricmp(bus_mapping, ldb_bit_mappings[i].mapping) &&
			bus_data_width == ldb_bit_mappings[i].datawidth)
			return ldb_bit_mappings[i].bus_format;
	}

	dev_err(dev, "invalid data mapping: %d-bit \"%s\"\n", bus_data_width, bus_mapping);

	return -ENOENT;
}

static bool imx8mp_ldb_is_dual(imx_lvds_display_interface interface)
{
	return (interface == imx_lvds0_dual) ? true : false;
}

static int imx8mp_ldb_get_phy_chan(imx_lvds_display_interface interface)
{
	if (interface == imx_lvds0) {
		return 0;
	} else if (interface == imx_lvds1) {
		return 1;
	} else {
		return -ENOENT;
	}
}

int
imx8mp_ldb_bind(struct device *dev, unsigned int bus_data_width,
	const char * const bus_mapping, imx_lvds_display_interface interface)
{
	int bus_format, chan;
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(dev);
	int ret, i;

	imx8mp_ldb->ctrl_reg = LDB_CTRL_REG;
	imx8mp_ldb->dual = imx8mp_ldb_is_dual(interface);
	chan = imx8mp_ldb_get_phy_chan(interface);
	if (!imx8mp_ldb->dual && chan < 0) {
		dev_err(dev, "could not determine display interface: %d\n", chan);
		return chan;
	}
	imx8mp_ldb->active_chan = (u32)chan;
	bus_format = get_bus_format(dev, bus_data_width, bus_mapping);
	if (bus_format < 0) {
		dev_err(dev, "could not determine data mapping: %d\n",
			bus_format);
		return bus_format;
	}
	imx8mp_ldb->bus_format = (u32)bus_format;

	imx8mp_ldb->clk_root = devm_clk_get(dev, "ldb");
	if (IS_ERR(imx8mp_ldb->clk_root))
		return PTR_ERR(imx8mp_ldb->clk_root);

	clk_prepare_enable(imx8mp_ldb->clk_root);

	/* disable LDB by resetting the control register to POR default */
	writel(0, imx8mp_ldb->base + imx8mp_ldb->ctrl_reg);
	imx8mp_ldb->ldb_ctrl = 0;

	/* Originaly clk_disable_unprepare(imx8mp_ldb->clk_root) was here,
		but must be left enabled, otherwise Enable/disable pairs don't match */

	if (imx8mp_ldb->dual)
		imx8mp_ldb->ldb_ctrl |= LDB_SPLIT_MODE_EN;

	for (i = 0; i < LDB_CH_NUM; i++) {
		ret = imx8mp_lvds_phy_init(dev->parent, i);
		if (ret < 0) {
			dev_err(dev, "failed to initialize channel%d phy: %d\n", i, ret);
			return ret;
		}
	}

	return 0;
}

void imx8mp_ldb_unbind(struct device *dev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < LDB_CH_NUM; i++) {
		if (imx8mp_ldb->phy_is_on[i])
			imx8mp_lvds_phy_power_off(dev->parent, i);

		imx8mp_lvds_phy_exit(dev->parent, i);
	}
}

int imx8mp_ldb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx8mp_ldb *imx8mp_ldb;
	struct resource *res;

	imx8mp_ldb = devm_kzalloc(dev, sizeof(*imx8mp_ldb), GFP_KERNEL);
	if (!imx8mp_ldb)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	imx8mp_ldb->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(imx8mp_ldb->base))
		return -ENOMEM;

	dev_set_drvdata(dev, imx8mp_ldb);

	return 0;
}

int imx8mp_ldb_remove(struct platform_device *pdev)
{
	struct imx8mp_ldb *imx8mp_ldb = platform_get_drvdata(pdev);
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res && imx8mp_ldb && imx8mp_ldb->base) {
		iounmap(imx8mp_ldb->base, resource_size(res));
	}
	if (imx8mp_ldb) {
		devm_kfree(&pdev->dev, imx8mp_ldb);
	}
	return 0;
}

int imx8mp_ldb_suspend(struct device *dev)
{
	int i;

	if ((dev == NULL) || (dev->parent == NULL))
		return 0;

	for (i = 0; i < LDB_CH_NUM; i++)
		imx8mp_lvds_phy_exit(dev->parent, i);

	return 0;
}

int imx8mp_ldb_resume(struct device *dev)
{
	int i;

	if ((dev == NULL) || (dev->parent == NULL))
		return 0;

	for (i = 0; i < LDB_CH_NUM; i++)
		imx8mp_lvds_phy_init(dev->parent, i);

	return 0;
}

#ifdef IMX8MP_LDB_DEBUG
/**
  Dump LCDIF registers.
**/
#include "wdm.h"
#define LVDS_CTRL		0x128

void imx8mp_ldb_dumpregs(struct platform_device *pdev)
{
	struct imx8mp_ldb *imx8mp_ldb = dev_get_drvdata(&pdev->dev);

	DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "------------------------LDB-----------\n");
	DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "MEDIA_BLK_CTRL_LDB_CTRL       = 0x%08X\n", readl(imx8mp_ldb->base + LDB_CTRL_REG));
	DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "MEDIA_BLK_CTRL_LVDS_CTRL[phy] = 0x%08X\n", readl(imx8mp_ldb->base + LVDS_CTRL));
	DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "--------------------------------------\n");
}
#endif
