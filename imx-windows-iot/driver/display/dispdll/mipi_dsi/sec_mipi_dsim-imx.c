/*
 * Samsung MIPI DSI Host Controller on IMX
 *
 * Copyright 2018-2020,2022-2023 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "mipi_dsi/sec_mipi_dsim.h"
#include "mipi_dsi/sec_mipi_dphy_ln14lpp.h"
#include "mipi_dsi/sec_mipi_pll_1432x.h"
#include "clk/displaymix_mn.h"

/* fixed phy ref clk rate */
#define PHY_REF_CLK		27000

struct imx_sec_dsim_device {
	struct device *dev;
	uint8_t __iomem *base;
	uint8_t __iomem *displaymix_base;
	int irq;
	struct clk *clk_cfg;
	struct clk *clk_pllref;

	void (*soft_resetn)(volatile uint8_t __iomem *regptr, bool enable);
	void (*clk_enable)(volatile uint8_t __iomem *regptr, bool enable);
	void (*mipi_reset)(volatile uint8_t __iomem *regptr, bool enable);

	atomic_t rpm_suspended;
};

static struct imx_sec_dsim_device *dsim_dev = NULL;

int imx_sec_dsim_runtime_suspend(struct device *dev);
int imx_sec_dsim_runtime_resume(struct device *dev);

static int sec_dsim_rstc_reset(
	void (*rstc)(volatile uint8_t __iomem *regptr, bool enable),
	bool assert)
{
	if (!rstc)
		return 0;

	rstc(dsim_dev->displaymix_base, assert);

	return 0;
}

void imx_sec_dsim_encoder_helper_enable(struct platform_device *pdev)
{
	int ret;

	ret = sec_dsim_rstc_reset(dsim_dev->mipi_reset, false);
	if (ret)
		dev_err(dsim_dev->dev, "deassert mipi_reset failed\n");
}

void imx_sec_dsim_encoder_helper_disable(struct platform_device *pdev)
{
	int ret;

	ret = sec_dsim_rstc_reset(dsim_dev->mipi_reset, true);
	if (ret)
		dev_err(dsim_dev->dev, "deassert mipi_reset failed\n");
}

int imx_sec_dsim_encoder_helper_atomic_check(struct platform_device *pdev,
						struct videomode *vm)
{

	int ret;

	/* check pll out */
	ret = sec_mipi_dsim_check_pll_out(pdev, vm);

	return ret;
}

static int sec_dsim_determine_pll_ref_rate(u32 *rate, u32 min, u32 max)
{
	int ret;
	struct device *dev = dsim_dev->dev;
	u32 req_rate = 0;

	ret = of_property_read_u32(&dev->of_node, "pref-rate", &req_rate);
	if (!ret) {
		if (req_rate != clamp(req_rate, min, max)) {
			dev_err(dev, "invalid requested PLL ref clock rate : %u\n", req_rate);
			req_rate = PHY_REF_CLK;
			dev_err(dev, "use default clock rate : %u\n", req_rate);
		}
	}

	*rate = req_rate;

	return 0;
}

static int sec_dsim_set_pll_ref_rate(u32 *rate)
{
	int ret;
	struct device *dev = dsim_dev->dev;
	unsigned long get_rate, req_rate;

	/* For others (Nano), reference rate is determined from PLL table */
	req_rate = clk_get_reference_rate(dsim_dev->clk_pllref);

	if (!req_rate)
		return -EINVAL;

	ret = clk_set_rate(dsim_dev->clk_pllref, req_rate);
	if (ret)
		return ret;

	get_rate = clk_get_rate(dsim_dev->clk_pllref);
	if (!get_rate)
		return -EINVAL;

	/* PLL ref clock rate should be set precisely */
	if (get_rate != req_rate) {
		dev_err(dev, "request rate %u cannot be satisfied\n", req_rate);
		return -EINVAL;
	}
	*rate = req_rate;
	return 0;
}

static int sec_dsim_set_pll_ref_rate_mp(u32* rate)
{
	int ret;
	struct device* dev = dsim_dev->dev;
	unsigned long get_rate, req_rate;

	/*For MP reference rate is fixed, already set in the probe function*/
	req_rate = clk_get_rate(dsim_dev->clk_pllref);

	if (!req_rate)
		return -EINVAL;

	ret = clk_set_rate(dsim_dev->clk_pllref, req_rate);
	if (ret)
		return ret;

	get_rate = clk_get_rate(dsim_dev->clk_pllref);
	if (!get_rate)
		return -EINVAL;

	/* PLL ref clock rate should be set precisely */
	if (get_rate != req_rate) {
		dev_err(dev, "request rate %u cannot be satisfied\n", req_rate);
		return -EINVAL;
	}
	*rate = req_rate;
	return 0;
}

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.dphy_pll	= &pll_1432x,
	.dphy_timing	= dphy_timing_ln14lpp_v1p2,
	.num_dphy_timing = ARRAY_SIZE(dphy_timing_ln14lpp_v1p2),
	.dphy_timing_cmp = dphy_timing_default_cmp,
	.determine_pll_ref_rate = sec_dsim_determine_pll_ref_rate,
	.set_pll_ref_rate = sec_dsim_set_pll_ref_rate,
};

static const struct sec_mipi_dsim_plat_data imx8mp_mipi_dsim_plat_data = {
	.version = 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate = 1500000000ULL,
	.dphy_pll = &pll_1432x,
	.dphy_timing = dphy_timing_ln14lpp_v1p2,
	.num_dphy_timing = ARRAY_SIZE(dphy_timing_ln14lpp_v1p2),
	.dphy_timing_cmp = dphy_timing_default_cmp,
	.determine_pll_ref_rate = sec_dsim_determine_pll_ref_rate,
	.set_pll_ref_rate = sec_dsim_set_pll_ref_rate_mp,
};

static const struct of_device_id imx_sec_dsim_dt_ids[] = {
	{
		.compatible = "fsl,imx8mm-mipi-dsim",
		.data = &imx8mm_mipi_dsim_plat_data,
	},
	{
		.compatible = "fsl,imx8mn-mipi-dsim",
		.data = &imx8mm_mipi_dsim_plat_data,
	},
	{
		.compatible = "fsl,imx8mp-mipi-dsim",
		.data = &imx8mp_mipi_dsim_plat_data,
	},
	{ /* sentinel */ 0 }
};

static int sec_dsim_of_parse_resets(struct imx_sec_dsim_device *dsim)
{
	struct device *dev = dsim->dev;
	struct device_node *np = &dev->of_node;
	const char *compat;
	size_t len;

	compat = of_get_property(np, "compatible", NULL);
	if (unlikely(!compat))
		return -ENODEV;

	len = strlen(compat);

	if (!of_compat_cmp("fsl,imx8mn-mipi-dsim", compat, len)) {
		dsim->soft_resetn = &displaymix_mn_dsi_soft_resetn;
		dsim->clk_enable = &displaymix_mn_dsi_clk_enable;
		dsim->mipi_reset = &displaymix_mn_dsi_mipi_reset;
	} else if (!of_compat_cmp(compat, "fsl,imx8mp-mipi-dsim", len)) {
		/* TODO: bypass resets for imx8mp platform */
		dsim->soft_resetn = NULL;
		dsim->clk_enable = NULL;
		dsim->mipi_reset = NULL;
	}
	else {
		dev_warn(dev, "unsupported derivative: %s\n", compat);
		return -ENODEV;
	}

	return 0;
}

int imx_sec_dsim_bind(struct device *dev)
{

	int ret;

	const struct of_device_id *of_id = of_match_device(imx_sec_dsim_dt_ids,
							   dev);
	const struct sec_mipi_dsim_plat_data *pdata;

	dev_dbg(dev, "%s: dsim bind begin\n", __func__);

	if (!of_id)
		return -ENODEV;
	pdata = of_id->data;

	/* bind sec dsim bridge */
	ret = sec_mipi_dsim_bind(dev, dsim_dev->base, dsim_dev->irq, pdata);
	if (ret) {
		dev_err(dev, "failed to bind sec dsim bridge: %d\n", ret);

		/* If no panel or bridge connected, just return 0
		 * to make component core to believe it is bound
		 * successfully to allow other components can be
		 * bound continuously, since in component core,
		 * it follows 'one fails, all fail'. It is useful
		 * when there exists multiple heads display.
		 */
		if (ret == -ENODEV)
			return 0;

		return ret;
	}

	dev_dbg(dev, "%s: dsim bind end\n", __func__);

	return 0;
}

void imx_sec_dsim_unbind(struct device *dev)
{
	sec_mipi_dsim_unbind(dev);
}


int imx_sec_dsim_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "%s: dsim probe begin\n", __func__);

	dsim_dev = devm_kzalloc(dev, sizeof(*dsim_dev), GFP_KERNEL);
	if (!dsim_dev) {
		dev_err(dev, "Unable to allocate 'dsim_dev'\n");
		return -ENOMEM;
	}
	dsim_dev->dev = dev;

	dsim_dev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsim_dev->base))
		return PTR_ERR(dsim_dev->base);

	dsim_dev->displaymix_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(dsim_dev->displaymix_base))
		return PTR_ERR(dsim_dev->displaymix_base);

	dsim_dev->irq = platform_get_irq(pdev, 0);
	if (dsim_dev->irq < 0)
		return -ENODEV;

	dsim_dev->clk_cfg = devm_clk_get(dev, "cfg");
	if (IS_ERR(dsim_dev->clk_cfg))
		return PTR_ERR(dsim_dev->clk_cfg);

	dsim_dev->clk_pllref = devm_clk_get(dev, "pll-ref");
	if (IS_ERR(dsim_dev->clk_pllref))
		return PTR_ERR(dsim_dev->clk_pllref);

	ret = sec_dsim_of_parse_resets(dsim_dev);
	if (ret)
		return ret;

	atomic_set(&dsim_dev->rpm_suspended, 1);

	return 0;
}

int imx_sec_dsim_remove(struct platform_device *pdev)
{
	struct resource *res;

	if (dsim_dev) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (res && dsim_dev->base) {
			iounmap(dsim_dev->base, resource_size(res));
		}
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (res && dsim_dev->displaymix_base) {
			iounmap(dsim_dev->displaymix_base, resource_size(res));
		}

		devm_kfree(&pdev->dev, dsim_dev);
		dsim_dev = NULL;
	}

	return 0;
}

int imx_sec_dsim_runtime_suspend(struct device *dev)
{
	if (atomic_inc_return(&dsim_dev->rpm_suspended) > 1)
		return 0;

	sec_mipi_dsim_suspend(dev);

	clk_disable_unprepare(dsim_dev->clk_cfg);
	clk_disable_unprepare(dsim_dev->clk_pllref);

	return 0;
}

int imx_sec_dsim_runtime_resume(struct device *dev)
{
	int ret;

	if (unlikely(!atomic_read(&dsim_dev->rpm_suspended))) {
		dev_warn(dsim_dev->dev,
			 "Unbalanced %s!\n", __func__);
		return 0;
	}

	if (!atomic_dec_and_test(&dsim_dev->rpm_suspended))
		return 0;

	ret = clk_prepare_enable(dsim_dev->clk_pllref);
	WARN_ON(unlikely(ret));
	if (ret)
		return ret;

	ret = clk_prepare_enable(dsim_dev->clk_cfg);
	WARN_ON(unlikely(ret))
	if (ret)
		return ret;

	ret = sec_dsim_rstc_reset(dsim_dev->soft_resetn, false);
	if (ret) {
		dev_err(dev, "deassert soft_resetn failed\n");
		return ret;
	}

	ret = sec_dsim_rstc_reset(dsim_dev->clk_enable, true);
	if (ret) {
		dev_err(dev, "assert clk_enable failed\n");
		return ret;
	}

	ret = sec_dsim_rstc_reset(dsim_dev->mipi_reset, false);
	if (ret) {
		dev_err(dev, "deassert mipi_reset failed\n");
		return ret;
	}

	sec_mipi_dsim_resume(dev);

	return 0;
}
