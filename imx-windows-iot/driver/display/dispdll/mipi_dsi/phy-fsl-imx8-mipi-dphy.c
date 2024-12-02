// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2017,2018,2021,2024 NXP
 * Copyright 2019 Purism SPC
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* DPHY registers */
#define DPHY_PD_DPHY			0x300
#define DPHY_M_PRG_HS_PREPARE		0x304
#define DPHY_MC_PRG_HS_PREPARE		0x308
#define DPHY_M_PRG_HS_ZERO		0x30c
#define DPHY_MC_PRG_HS_ZERO		0x310
#define DPHY_M_PRG_HS_TRAIL		0x314
#define DPHY_MC_PRG_HS_TRAIL		0x318
#define DPHY_REG_BYPASS_PLL		0x34C

#define MBPS(x) ((x) * 1000000)

#define DATA_RATE_MAX_SPEED MBPS(1500)
#define DATA_RATE_MIN_SPEED MBPS(80)

#define PLL_LOCK_SLEEP 10
#define PLL_LOCK_TIMEOUT 1000

#define CN_BUF	0xcb7a89c0
#define CO_BUF	0x63
#define CM(x)	(				  \
		((x) <	32) ? 0xe0 | ((x) - 16) : \
		((x) <	64) ? 0xc0 | ((x) - 32) : \
		((x) < 128) ? 0x80 | ((x) - 64) : \
		((x) - 128))
#define CN(x)	(((x) == 1) ? 0x1f : (((CN_BUF) >> ((x) - 1)) & 0x1f))
#define CO(x)	((CO_BUF) >> (8 - (x)) & 0x03)

/* PHY power on is active low */
#define PWR_ON	0
#define PWR_OFF	1

/* not available register */
#define REG_NA	0xffffffff

enum mixel_dphy_devtype {
	MIXEL_IMX8MQ,
	MIXEL_IMX8QM,
	MIXEL_IMX8QX,
	MIXEL_IMX8ULP,
};

struct mixel_dphy_devdata {
	unsigned int reg_mc_prg_rxhs_settle;
	unsigned int reg_m_prg_rxhs_settle;
	unsigned int reg_pd_pll;
	unsigned int reg_tst;
	unsigned int reg_cn;
	unsigned int reg_cm;
	unsigned int reg_co;
	unsigned int reg_lock;
	unsigned int reg_lock_byp;
	unsigned int reg_tx_rcal;
	unsigned int reg_auto_pd_en;
	unsigned int reg_rxlprp;
	unsigned int reg_rxcdrp;
	unsigned int reg_rxhs_settle;
};

static const struct mixel_dphy_devdata mixel_dphy_devdata[] = {
	[MIXEL_IMX8MQ] = {
		.reg_mc_prg_rxhs_settle = REG_NA,
		.reg_m_prg_rxhs_settle = REG_NA,
		.reg_pd_pll = 0x31c,
		.reg_tst = 0x320,
		.reg_cn = 0x324,
		.reg_cm = 0x328,
		.reg_co = 0x32c,
		.reg_lock = 0x330,
		.reg_lock_byp = 0x334,
		.reg_tx_rcal = 0x338,
		.reg_auto_pd_en = 0x33c,
		.reg_rxlprp = 0x340,
		.reg_rxcdrp = 0x344,
		.reg_rxhs_settle = 0x348,
	},
	[MIXEL_IMX8QM] = {
		.reg_mc_prg_rxhs_settle = REG_NA,
		.reg_m_prg_rxhs_settle = REG_NA,
		.reg_pd_pll = 0x31c,
		.reg_tst = 0x320,
		.reg_cn = 0x324,
		.reg_cm = 0x328,
		.reg_co = 0x32c,
		.reg_lock = 0x330,
		.reg_lock_byp = 0x334,
		.reg_tx_rcal = REG_NA,
		.reg_auto_pd_en = 0x338,
		.reg_rxlprp = 0x33c,
		.reg_rxcdrp = 0x340,
		.reg_rxhs_settle = 0x344,
	},
	[MIXEL_IMX8QX] = {
		.reg_mc_prg_rxhs_settle = REG_NA,
		.reg_m_prg_rxhs_settle = REG_NA,
		.reg_pd_pll = 0x31c,
		.reg_tst = 0x320,
		.reg_cn = 0x324,
		.reg_cm = 0x328,
		.reg_co = 0x32c,
		.reg_lock = 0x330,
		.reg_lock_byp = 0x334,
		.reg_tx_rcal = REG_NA,
		.reg_auto_pd_en = 0x338,
		.reg_rxlprp = 0x33c,
		.reg_rxcdrp = 0x340,
		.reg_rxhs_settle = 0x344,
	},
};

struct mixel_dphy_cfg {
	/* DPHY PLL parameters */
	u32 cm;
	u32 cn;
	u32 co;
	/* DPHY register values */
	u8 mc_prg_hs_prepare;
	u8 m_prg_hs_prepare;
	u8 mc_prg_hs_zero;
	u8 m_prg_hs_zero;
	u8 mc_prg_hs_trail;
	u8 m_prg_hs_trail;
	u8 rxhs_settle;
};

struct mixel_dphy_priv {
	struct mixel_dphy_cfg cfg;
	struct regmap *regmap;
	struct clk *phy_ref_clk;
	const struct mixel_dphy_devdata *devdata;
	bool regmap_allocated;
};

static const struct regmap_config mixel_dphy_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = DPHY_REG_BYPASS_PLL,
};

static int phy_write(struct mixel_dphy_priv *priv, u32 value, unsigned int reg)
{
	int ret;

	ret = regmap_write(priv->regmap, reg, value);
	if (ret < 0)
		dev_err(NULL, "Failed to write DPHY reg %d: %d\n", reg,
			ret);
	return ret;
}

/*
 * Find a ratio close to the desired one using continued fraction
 * approximation ending either at exact match or maximum allowed
 * nominator, denominator.
 */
static void get_best_ratio(u32 *pnum, u32 *pdenom, u32 max_n, u32 max_d)
{
	u32 a = *pnum;
	u32 b = *pdenom;
	u32 c;
	u32 n[] = {0, 1};
	u32 d[] = {1, 0};
	u32 whole;
	unsigned int i = 1;

	while (b) {
		i ^= 1;
		whole = a / b;
		n[i] += (n[i ^ 1] * whole);
		d[i] += (d[i ^ 1] * whole);
		if ((n[i] > max_n) || (d[i] > max_d)) {
			i ^= 1;
			break;
		}
		c = a - (b * whole);
		a = b;
		b = c;
	}
	*pnum = n[i];
	*pdenom = d[i];
}

static int mixel_dphy_config_from_opts(struct platform_device *pdev,
	       struct phy_configure_opts_mipi_dphy *dphy_opts,
	       struct mixel_dphy_cfg *cfg)
{
	struct mixel_dphy_priv *priv = dev_get_drvdata(&pdev->dev);
	unsigned long ref_clk = clk_get_rate(priv->phy_ref_clk);
	u32 lp_t, numerator, denominator;
	unsigned long long tmp;
	u32 n;
	int i;

	if (dphy_opts->hs_clk_rate > DATA_RATE_MAX_SPEED ||
	    dphy_opts->hs_clk_rate < DATA_RATE_MIN_SPEED)
		return -EINVAL;

	numerator = dphy_opts->hs_clk_rate;
	denominator = ref_clk;
	get_best_ratio(&numerator, &denominator, 255, 256);
	if (!numerator || !denominator) {
		dev_err(&pdev->dev, "Invalid %d/%d for %ld/%ld\n",
			numerator, denominator,
			dphy_opts->hs_clk_rate, ref_clk);
		return -EINVAL;
	}

	while ((numerator < 16) && (denominator <= 128)) {
		numerator <<= 1;
		denominator <<= 1;
	}
	/*
	 * CM ranges between 16 and 255
	 * CN ranges between 1 and 32
	 * CO is power of 2: 1, 2, 4, 8
	 */
	i = __ffs(denominator);
	if (i > 3)
		i = 3;
	cfg->cn = denominator >> i;
	cfg->co = 1 << i;
	cfg->cm = numerator;

	if (cfg->cm < 16 || cfg->cm > 255 ||
	    cfg->cn < 1 || cfg->cn > 32 ||
	    cfg->co < 1 || cfg->co > 8) {
		dev_err(&pdev->dev, "Invalid CM/CN/CO values: %u/%u/%u\n",
			cfg->cm, cfg->cn, cfg->co);
		dev_err(&pdev->dev, "for hs_clk/ref_clk=%ld/%ld ~ %d/%d\n",
			dphy_opts->hs_clk_rate, ref_clk,
			numerator, denominator);
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "hs_clk/ref_clk=%ld/%ld ~ %d/%d\n",
		dphy_opts->hs_clk_rate, ref_clk, numerator, denominator);

	/* LP clock period */
	tmp = 1000000000000LL;
	do_div(&tmp, dphy_opts->lp_clk_rate); /* ps */
	if (tmp > ULONG_MAX)
		return -EINVAL;

	lp_t = (u32)tmp;
	dev_dbg(&pdev->dev, "LP clock %lu, period: %u ps\n",
		dphy_opts->lp_clk_rate, lp_t);

	/* hs_prepare: in lp clock periods */
	if (2 * dphy_opts->hs_prepare > 5 * lp_t) {
		dev_err(&pdev->dev,
			"hs_prepare (%u) > 2.5 * lp clock period (%u)\n",
			dphy_opts->hs_prepare, lp_t);
		return -EINVAL;
	}
	/* 00: lp_t, 01: 1.5 * lp_t, 10: 2 * lp_t, 11: 2.5 * lp_t */
	if (dphy_opts->hs_prepare < lp_t) {
		n = 0;
	} else {
		tmp = 2 * (dphy_opts->hs_prepare - lp_t);
		do_div(&tmp, lp_t);
		n = (u32)tmp;
	}
	cfg->m_prg_hs_prepare = (u8)n;

	/* clk_prepare: in lp clock periods */
	if (2 * dphy_opts->clk_prepare > 3 * lp_t) {
		dev_err(&pdev->dev,
			"clk_prepare (%u) > 1.5 * lp clock period (%u)\n",
			dphy_opts->clk_prepare, lp_t);
		return -EINVAL;
	}
	/* 00: lp_t, 01: 1.5 * lp_t */
	cfg->mc_prg_hs_prepare = dphy_opts->clk_prepare > lp_t ? 1 : 0;

	/* hs_zero: formula from NXP BSP */
	n = (144 * (dphy_opts->hs_clk_rate / 1000000) - 47500) / 10000;
	cfg->m_prg_hs_zero = n < 1 ? 1 : (u8)n;

	/* clk_zero: formula from NXP BSP */
	n = (34 * (dphy_opts->hs_clk_rate / 1000000) - 2500) / 1000;
	cfg->mc_prg_hs_zero = n < 1 ? 1 : (u8)n;

	/* clk_trail, hs_trail: formula from NXP BSP */
	n = (103 * (dphy_opts->hs_clk_rate / 1000000) + 10000) / 10000;
	if (n > 15)
		n = 15;
	if (n < 1)
		n = 1;
	cfg->m_prg_hs_trail = (u8)n;
	cfg->mc_prg_hs_trail = (u8)n;

	/* rxhs_settle: formula from NXP BSP */
	if (priv->devdata->reg_rxhs_settle != REG_NA) {
		if (dphy_opts->hs_clk_rate < MBPS(80))
			cfg->rxhs_settle = 0x0d;
		else if (dphy_opts->hs_clk_rate < MBPS(90))
			cfg->rxhs_settle = 0x0c;
		else if (dphy_opts->hs_clk_rate < MBPS(125))
			cfg->rxhs_settle = 0x0b;
		else if (dphy_opts->hs_clk_rate < MBPS(150))
			cfg->rxhs_settle = 0x0a;
		else if (dphy_opts->hs_clk_rate < MBPS(225))
			cfg->rxhs_settle = 0x09;
		else if (dphy_opts->hs_clk_rate < MBPS(500))
			cfg->rxhs_settle = 0x08;
		else
			cfg->rxhs_settle = 0x07;
	} else if (priv->devdata->reg_m_prg_rxhs_settle != REG_NA) {
		if (dphy_opts->hs_clk_rate < MBPS(80))
			cfg->rxhs_settle = 0x01;
		else if (dphy_opts->hs_clk_rate < MBPS(250))
			cfg->rxhs_settle = 0x06;
		else if (dphy_opts->hs_clk_rate < MBPS(500))
			cfg->rxhs_settle = 0x08;
		else
			cfg->rxhs_settle = 0x0a;
	}

	dev_dbg(&pdev->dev, "phy_config: %u %u %u %u %u %u %u\n",
		cfg->m_prg_hs_prepare, cfg->mc_prg_hs_prepare,
		cfg->m_prg_hs_zero, cfg->mc_prg_hs_zero,
		cfg->m_prg_hs_trail, cfg->mc_prg_hs_trail,
		cfg->rxhs_settle);

	return 0;
}

static void mixel_phy_set_hs_timings(struct platform_device *pdev)
{
	struct mixel_dphy_priv *priv = dev_get_drvdata(&pdev->dev);

	phy_write(priv, priv->cfg.m_prg_hs_prepare, DPHY_M_PRG_HS_PREPARE);
	phy_write(priv, priv->cfg.mc_prg_hs_prepare, DPHY_MC_PRG_HS_PREPARE);
	phy_write(priv, priv->cfg.m_prg_hs_zero, DPHY_M_PRG_HS_ZERO);
	phy_write(priv, priv->cfg.mc_prg_hs_zero, DPHY_MC_PRG_HS_ZERO);
	phy_write(priv, priv->cfg.m_prg_hs_trail, DPHY_M_PRG_HS_TRAIL);
	phy_write(priv, priv->cfg.mc_prg_hs_trail, DPHY_MC_PRG_HS_TRAIL);
	if (priv->devdata->reg_rxhs_settle != REG_NA)
		phy_write(priv, priv->cfg.rxhs_settle,
			  priv->devdata->reg_rxhs_settle);
	else if (priv->devdata->reg_m_prg_rxhs_settle != REG_NA)
		phy_write(priv, priv->cfg.rxhs_settle,
			  priv->devdata->reg_m_prg_rxhs_settle);

	if (priv->devdata->reg_mc_prg_rxhs_settle != REG_NA)
		phy_write(priv, 0x10, priv->devdata->reg_mc_prg_rxhs_settle);
}

static int mixel_dphy_set_pll_params(struct platform_device *pdev)
{
	struct mixel_dphy_priv *priv = dev_get_drvdata(&pdev->dev);

	if (priv->cfg.cm < 16 || priv->cfg.cm > 255 ||
	    priv->cfg.cn < 1 || priv->cfg.cn > 32 ||
	    priv->cfg.co < 1 || priv->cfg.co > 8) {
		dev_err(&pdev->dev, "Invalid CM/CN/CO values! (%u/%u/%u)\n",
			priv->cfg.cm, priv->cfg.cn, priv->cfg.co);
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "Using CM:%u CN:%u CO:%u\n",
		priv->cfg.cm, priv->cfg.cn, priv->cfg.co);
	phy_write(priv, CM(priv->cfg.cm), priv->devdata->reg_cm);
	phy_write(priv, CN(priv->cfg.cn), priv->devdata->reg_cn);
	phy_write(priv, CO(priv->cfg.co), priv->devdata->reg_co);
	return 0;
}

int mixel_dphy_configure(struct platform_device *pdev, union phy_configure_opts *opts)
{
	struct mixel_dphy_priv *priv = dev_get_drvdata(&pdev->dev);
	struct mixel_dphy_cfg cfg = { 0 };
	int ret;

	ret = mixel_dphy_config_from_opts(pdev, &opts->mipi_dphy, &cfg);
	if (ret)
		return ret;

	/* Update the configuration */
	memcpy(&priv->cfg, &cfg, sizeof(struct mixel_dphy_cfg));

	phy_write(priv, 0x00, priv->devdata->reg_lock_byp);
	if (priv->devdata->reg_tx_rcal != REG_NA) {
		phy_write(priv, 0x01, priv->devdata->reg_tx_rcal);
	}
	phy_write(priv, 0x00, priv->devdata->reg_auto_pd_en);
	phy_write(priv, 0x02, priv->devdata->reg_rxlprp);
	phy_write(priv, 0x02, priv->devdata->reg_rxcdrp);
	phy_write(priv, 0x25, priv->devdata->reg_tst);

	mixel_phy_set_hs_timings(pdev);
	ret = mixel_dphy_set_pll_params(pdev);
	if (ret < 0)
		return ret;

	return 0;
}

int mixel_dphy_validate(struct platform_device *pdev, enum phy_mode mode, int submode,
					union phy_configure_opts *opts)
{
	struct mixel_dphy_cfg cfg = { 0 };

	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	return mixel_dphy_config_from_opts(pdev, &opts->mipi_dphy, &cfg);
}

int mixel_dphy_init(struct platform_device *pdev)
{
	struct mixel_dphy_priv *priv = dev_get_drvdata(&pdev->dev);

	phy_write(priv, PWR_OFF, priv->devdata->reg_pd_pll);
	phy_write(priv, PWR_OFF, DPHY_PD_DPHY);

	return 0;
}

int mixel_dphy_exit(struct platform_device *pdev)
{
	struct mixel_dphy_priv *priv = dev_get_drvdata(&pdev->dev);

	phy_write(priv, 0, priv->devdata->reg_cm);
	phy_write(priv, 0, priv->devdata->reg_cn);
	phy_write(priv, 0, priv->devdata->reg_co);

	return 0;
}

int mixel_dphy_power_on(struct platform_device *pdev)
{
	struct mixel_dphy_priv *priv = dev_get_drvdata(&pdev->dev);
	int ret;

	ret = clk_prepare_enable(priv->phy_ref_clk);
	if (ret < 0)
		return ret;

	phy_write(priv, PWR_ON, priv->devdata->reg_pd_pll);
	ret = regmap_read_poll_timeout_ex(priv->regmap, priv->devdata->reg_lock, 
					1, 0, TRUE, PLL_LOCK_SLEEP, PLL_LOCK_TIMEOUT);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not get DPHY lock (%d)!\n", ret);
		goto clock_disable;
	}
	phy_write(priv, PWR_ON, DPHY_PD_DPHY);

	return 0;
clock_disable:
	clk_disable_unprepare(priv->phy_ref_clk);
	return ret;
}

int mixel_dphy_power_off(struct platform_device *pdev)
{
	struct mixel_dphy_priv *priv = dev_get_drvdata(&pdev->dev);

	phy_write(priv, PWR_OFF, priv->devdata->reg_pd_pll);
	phy_write(priv, PWR_OFF, DPHY_PD_DPHY);

	clk_disable_unprepare(priv->phy_ref_clk);

	return 0;
}

static const struct of_device_id mixel_dphy_of_match[] = {
	{ .compatible = "fsl,imx8mq-mipi-dphy",
	  .data = &mixel_dphy_devdata[MIXEL_IMX8MQ] },
	{ .compatible = "fsl,imx8qm-mipi-dphy",
	  .data = &mixel_dphy_devdata[MIXEL_IMX8QM] },
	{ .compatible = "fsl,imx8qx-mipi-dphy",
	  .data = &mixel_dphy_devdata[MIXEL_IMX8QX] },
	{ .compatible = "fsl,imx8ulp-mipi-dphy",
	  .data = &mixel_dphy_devdata[MIXEL_IMX8ULP] },
	{ /* sentinel */ 0 },
};

int mixel_dphy_probe(struct platform_device *pdev, struct regmap *mipi_map)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = &dev->of_node;
	struct mixel_dphy_priv *priv;
	struct resource* iores = NULL;
	const struct of_device_id *of_id;

	if (!np)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	of_id = of_match_device(mixel_dphy_of_match, dev);
	priv->devdata = of_id->data;
	if (!priv->devdata)
		return -EINVAL;

	if (mipi_map) {
		priv->regmap = mipi_map;
		priv->regmap_allocated = false;
	} else {
		iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!iores)
			return -ENODEV;

		priv->regmap = devm_regmap_init_mmio(&pdev->dev, iores,
						     &mixel_dphy_regmap_config);
		if (IS_ERR(priv->regmap)) {
			dev_err(dev, "Couldn't create the DPHY regmap\n");
			return PTR_ERR(priv->regmap);
		}
		priv->regmap_allocated = true;
	}

	priv->phy_ref_clk = devm_clk_get(&pdev->dev, "phy_ref");
	if (IS_ERR(priv->phy_ref_clk)) {
		dev_err(dev, "No phy_ref clock found\n");
		return PTR_ERR(priv->phy_ref_clk);
	}
	dev_dbg(dev, "phy_ref clock rate: %lu\n",
		clk_get_rate(priv->phy_ref_clk));

	dev_set_drvdata(dev, priv);

	return 0;	
}

int mixel_dphy_remove(struct platform_device *pdev)
{
	struct mixel_dphy_priv *priv = platform_get_drvdata(pdev);
	struct resource *res;

	if (priv) {
		if (priv->regmap && priv->regmap_allocated) {
			res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
			if (res) {
				regmap_release_mmio(priv->regmap, res);
			}
		}
		devm_kfree(&pdev->dev, priv);
	}

	return 0;
}

static void mixel_dphy_dump_reg(struct mixel_dphy_priv* priv, struct device* dev, uint32_t reg, char* reg_name)
{
	uint32_t status;
	uint32_t base = 0x56228000;

	(void)regmap_read(priv->regmap, reg, &status);
	dev_err(dev, "%s(0x%x) = 0x%02X\n", reg_name, (base + reg), status);
}

void mixel_dphy_dump(struct platform_device* pdev)
{
	struct mixel_dphy_priv* priv = platform_get_drvdata(pdev);
	struct device* dev = &pdev->dev;

	if (priv == NULL) {
		return;
	}

	/* PHY initialization */
	dev_err(dev, "*************************** PHY init ****************************************\n");
	mixel_dphy_dump_reg(priv, dev, DPHY_PD_DPHY, "DPHY_PD_DPHY");
	mixel_dphy_dump_reg(priv, dev, DPHY_M_PRG_HS_PREPARE, "DPHY_M_PRG_HS_PREPARE");
	mixel_dphy_dump_reg(priv, dev, DPHY_MC_PRG_HS_PREPARE, "DPHY_MC_PRG_HS_PREPARE");
	mixel_dphy_dump_reg(priv, dev, DPHY_M_PRG_HS_ZERO, "DPHY_M_PRG_HS_ZERO");
	mixel_dphy_dump_reg(priv, dev, DPHY_MC_PRG_HS_ZERO, "DPHY_MC_PRG_HS_ZERO");
	mixel_dphy_dump_reg(priv, dev, DPHY_M_PRG_HS_TRAIL, "DPHY_M_PRG_HS_TRAIL");
	mixel_dphy_dump_reg(priv, dev, DPHY_MC_PRG_HS_TRAIL, "DPHY_MC_PRG_HS_TRAIL");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_pd_pll, "DPHY_PD_PLL");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_tst, "DPHY_TST");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_cn, "DPHY_CN");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_cm, "DPHY_CM");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_co, "DPHY_CO");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_lock, "DPHY_LOCK");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_lock_byp, "DPHY_LOCK_BYP");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_auto_pd_en, "DPHY_AUTO_PD_EN");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_rxlprp, "DPHY_RXLPRP");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_rxcdrp, "DPHY_RXCDRP");
	mixel_dphy_dump_reg(priv, dev, priv->devdata->reg_rxhs_settle, "DPHY_RXHS_SETTLE");
}
