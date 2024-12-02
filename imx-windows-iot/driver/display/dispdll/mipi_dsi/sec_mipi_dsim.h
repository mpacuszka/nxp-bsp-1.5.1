/*
 * Copyright 2018-2019,2022 NXP
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

#ifndef __SEC_MIPI_DSIM_H__
#define __SEC_MIPI_DSIM_H__

#include <linux/bsearch.h>
#include <linux/interrupt.h>

#define SEC_MIPI_DSIM_DEBUG

struct sec_mipi_dsim_dphy_timing;
struct sec_mipi_dsim_pll;
struct videomode;
struct platform_device;
struct drm_panel;

struct sec_mipi_dsim_plat_data {
	uint32_t version;
	uint32_t max_data_lanes;
	uint64_t max_data_rate;
	const struct sec_mipi_dsim_dphy_timing *dphy_timing;
	uint32_t num_dphy_timing;
	const struct sec_mipi_dsim_pll *dphy_pll;
	int (*dphy_timing_cmp)(const void *key, const void *elt);
	int (*determine_pll_ref_rate)(u32 *rate, u32 min, u32 max);
	int (*set_pll_ref_rate)(u32 *rate);
};

/* DPHY PLL structure */
struct sec_mipi_dsim_range {
	uint32_t min;
	uint32_t max;
};

struct sec_mipi_dsim_pll {
	struct sec_mipi_dsim_range p;
	struct sec_mipi_dsim_range m;
	struct sec_mipi_dsim_range s;
	struct sec_mipi_dsim_range k;
	struct sec_mipi_dsim_range fin;
	struct sec_mipi_dsim_range fpref;
	struct sec_mipi_dsim_range fvco;
};

/* DPHY timings structure */
struct sec_mipi_dsim_dphy_timing {
	uint32_t bit_clk;	/* MHz */

	uint32_t clk_prepare;
	uint32_t clk_zero;
	uint32_t clk_post;
	uint32_t clk_trail;

	uint32_t hs_prepare;
	uint32_t hs_zero;
	uint32_t hs_trail;

	uint32_t lpx;
	uint32_t hs_exit;
};

#define DSIM_DPHY_TIMING(bclk, cpre, czero, cpost, ctrail,	\
			 hpre, hzero, htrail, lp, hexit)	\
	.bit_clk	= bclk,					\
	.clk_prepare	= cpre,					\
	.clk_zero	= czero,				\
	.clk_post	= cpost,				\
	.clk_trail	= ctrail,				\
	.hs_prepare	= hpre,					\
	.hs_zero	= hzero,				\
	.hs_trail	= htrail,				\
	.lpx		= lp,					\
	.hs_exit	= hexit

static inline int dphy_timing_default_cmp(const void *key, const void *elt)
{
	const struct sec_mipi_dsim_dphy_timing *_key = (const struct sec_mipi_dsim_dphy_timing *)key;
	const struct sec_mipi_dsim_dphy_timing *_elt = (const struct sec_mipi_dsim_dphy_timing *)elt;

	/* find an element whose 'bit_clk' is equal to the
	 * the key's 'bit_clk' value or, the difference
	 * between them is less than 5.
	 */
	if (abs((int)(_elt->bit_clk - _key->bit_clk)) <= 5)
		return 0;

	if (_key->bit_clk < _elt->bit_clk)
		/* search bottom half */
		return 1;
	else
		/* search top half */
		return -1;
}

int sec_mipi_dsim_check_pll_out(struct platform_device *pdev,
				const struct videomode *vm);
int sec_mipi_dsim_bind(struct device *dev, void __iomem *base,
		int irq, const struct sec_mipi_dsim_plat_data *pdata);
void sec_mipi_dsim_unbind(struct device *dev);

void sec_mipi_dsim_suspend(struct device *dev);
void sec_mipi_dsim_resume(struct device *dev);
irqreturn_t sec_mipi_dsim_irq_handler(struct device *dev);
int sec_mipi_dsim_bridge_atomic_check(struct platform_device *pdev,
	struct videomode *vm);
void sec_mipi_dsim_bridge_disable(struct platform_device *pdev);
void sec_mipi_dsim_bridge_enable(struct platform_device *pdev);
u32 sec_mipi_dsim_atomic_get_input_bus_fmts(struct platform_device *pdev);
void sec_mipi_dsim_bridge_mode_set(struct platform_device *pdev, struct videomode *vm);

/*IMX interface*/
int imx_sec_dsim_probe(struct platform_device *pdev);
int imx_sec_dsim_remove(struct platform_device *pdev);
int imx_sec_dsim_runtime_suspend(struct device *dev);
int imx_sec_dsim_runtime_resume(struct device *dev);
int imx_sec_dsim_bind(struct device *dev);
void imx_sec_dsim_unbind(struct device *dev);
int imx_sec_dsim_encoder_helper_atomic_check(struct platform_device *pdev,
						struct videomode *vm);
void imx_sec_dsim_encoder_helper_disable(struct platform_device *pdev);
void imx_sec_dsim_encoder_helper_enable(struct platform_device *pdev);
#ifdef SEC_MIPI_DSIM_DEBUG
void sec_mipi_dsim_dumpregs(struct platform_device *pdev);
#endif

int sec_mipi_dsim_test_attach_dsi(struct platform_device *pdev, u8 num_lanes, u8 channel_id);
void sec_mipi_dsim_test_detach_dsi(struct platform_device *pdev);

#endif
