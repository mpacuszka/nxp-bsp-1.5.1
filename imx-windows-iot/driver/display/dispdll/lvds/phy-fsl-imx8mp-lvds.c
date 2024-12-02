// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2020,2022 NXP
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define LVDS_CTRL		0x128
#define SPARE_IN(n)		(((n) & 0x7) << 25)
#define SPARE_IN_MASK		0xe000000
#define TEST_RANDOM_NUM_EN	BIT(24)
#define TEST_MUX_SRC(n)		(((n) & 0x3) << 22)
#define TEST_MUX_SRC_MASK	0xc00000
#define TEST_EN			BIT(21)
#define TEST_DIV4_EN		BIT(20)
#define VBG_ADJ(n)		(((n) & 0x7) << 17)
#define VBG_ADJ_MASK		0xe0000
#define SLEW_ADJ(n)		(((n) & 0x7) << 14)
#define SLEW_ADJ_MASK		0x1c000
#define CC_ADJ(n)		(((n) & 0x7) << 11)
#define CC_ADJ_MASK		0x3800
#define CM_ADJ(n)		(((n) & 0x7) << 8)
#define CM_ADJ_MASK		0x700
#define PRE_EMPH_ADJ(n)		(((n) & 0x7) << 5)
#define PRE_EMPH_ADJ_MASK	0xe0
#define PRE_EMPH_EN		BIT(4)
#define HS_EN			BIT(3)
#define BG_EN			BIT(2)
#define CH_EN(id)		BIT(id)

struct imx8mp_lvds_phy {
	struct device *dev;
	uint8_t __iomem *base;
	struct clk *apb_clk;
};

static inline unsigned int phy_read(struct imx8mp_lvds_phy *priv, unsigned int reg)
{
	unsigned int val;

	val = readl(priv->base + reg);

	return val;
}

static inline void
phy_write(struct imx8mp_lvds_phy *priv, unsigned int reg, unsigned int value)
{
	writel(value, priv->base + reg);
}

int imx8mp_lvds_phy_exit(struct device *dev, unsigned int chan)
{
	return 0;
}

int imx8mp_lvds_phy_init(struct device *dev, unsigned int chan)
{
	struct imx8mp_lvds_phy *priv = dev_get_drvdata(dev);

	clk_prepare_enable(priv->apb_clk);

	phy_write(priv, LVDS_CTRL,
			CC_ADJ(0x2) | PRE_EMPH_EN | PRE_EMPH_ADJ(0x3));

	clk_disable_unprepare(priv->apb_clk);

	return 0;
}

int imx8mp_lvds_phy_power_on(struct device *dev, unsigned int chan)
{
	struct imx8mp_lvds_phy *priv = dev_get_drvdata(dev);
	unsigned int val;
	bool bg_en;

	clk_prepare_enable(priv->apb_clk);

	val = phy_read(priv, LVDS_CTRL);
	bg_en = !!(val & BG_EN);
	val |= BG_EN;
	phy_write(priv, LVDS_CTRL, val);

	/* Wait 15us to make sure the bandgap to be stable. */
	if (!bg_en)
		usleep_range(15, 20);

	val = phy_read(priv, LVDS_CTRL);
	val |= CH_EN(chan);
	phy_write(priv, LVDS_CTRL, val);

	clk_disable_unprepare(priv->apb_clk);

	/* Wait 5us to ensure the phy be settling. */
	usleep_range(5, 10);

	return 0;
}

int imx8mp_lvds_phy_power_off(struct device *dev, unsigned int chan)
{
	struct imx8mp_lvds_phy *priv = dev_get_drvdata(dev);
	unsigned int val;

	clk_prepare_enable(priv->apb_clk);

	val = phy_read(priv, LVDS_CTRL);
	val &= ~BG_EN;
	phy_write(priv, LVDS_CTRL, val);

	val = phy_read(priv, LVDS_CTRL);
	val &= ~CH_EN(chan);
	phy_write(priv, LVDS_CTRL, val);

	clk_disable_unprepare(priv->apb_clk);

	return 0;
}

int imx8mp_lvds_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct imx8mp_lvds_phy *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->dev = dev;

	priv->apb_clk = devm_clk_get(dev, "apb");
	if (IS_ERR(priv->apb_clk)) {
		dev_err(dev, "cannot get apb clock\n");
		return PTR_ERR(priv->apb_clk);
	}

	dev_set_drvdata(dev, priv);

	return 0;
}

int imx8mp_lvds_phy_remove(struct platform_device *pdev)
{
	struct imx8mp_lvds_phy *priv = platform_get_drvdata(pdev);
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res && priv && priv->base) {
		iounmap(priv->base, resource_size(res));
	}

	if (priv) {
		devm_kfree(&pdev->dev, priv);
	}
	return 0;
}
