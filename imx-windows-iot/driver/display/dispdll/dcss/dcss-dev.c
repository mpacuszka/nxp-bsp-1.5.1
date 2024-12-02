// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019, 2022 NXP.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "dcss-dev.h"
#include "dcss-kms.h"

struct dcss_dev *dcss_drv_dev_to_dcss(struct device *dev)
{
	return dev_get_drvdata(dev);
}

static void dcss_clocks_enable(struct dcss_dev *dcss)
{
	clk_prepare_enable(dcss->axi_clk);
	clk_prepare_enable(dcss->apb_clk);
	clk_prepare_enable(dcss->rtrm_clk);
	clk_prepare_enable(dcss->dtrc_clk);
}

static void dcss_clocks_disable(struct dcss_dev *dcss)
{
	clk_disable_unprepare(dcss->dtrc_clk);
	clk_disable_unprepare(dcss->rtrm_clk);
	clk_disable_unprepare(dcss->apb_clk);
	clk_disable_unprepare(dcss->axi_clk);
}

static void dcss_disable_dtg_and_ss_cb(void *data)
{
	struct dcss_dev *dcss = data;

	dcss->disable_callback = NULL;

	dcss_ss_shutoff(dcss->ss);
	dcss_dtg_shutoff(dcss->dtg);

	complete(&dcss->disable_completion);
}

void dcss_disable_dtg_and_ss(struct dcss_dev *dcss)
{
	dcss->disable_callback = dcss_disable_dtg_and_ss_cb;
}

void dcss_enable_dtg_and_ss(struct dcss_dev *dcss)
{
	if (dcss->disable_callback)
		dcss->disable_callback = NULL;

	dcss_dtg_enable(dcss->dtg);
	dcss_ss_enable(dcss->ss);
}

static int dcss_submodules_init(struct dcss_dev *dcss)
{
	int ret = 0;
	u32 base_addr = dcss->start_addr;
	const struct dcss_type_data *devtype = dcss->devtype;

	dcss_clocks_enable(dcss);

	ret = dcss_blkctl_init(dcss, base_addr + devtype->blkctl_ofs);
	if (ret)
		return ret;

	ret = dcss_ctxld_init(dcss, base_addr + devtype->ctxld_ofs);
	if (ret)
		goto ctxld_err;

	ret = dcss_dtg_init(dcss, base_addr + devtype->dtg_ofs);
	if (ret)
		goto dtg_err;

	ret = dcss_ss_init(dcss, base_addr + devtype->ss_ofs);
	if (ret)
		goto ss_err;

	ret = dcss_dtrc_init(dcss, base_addr + devtype->dtrc_ofs);
	if (ret)
		goto dtrc_err;

	ret = dcss_dpr_init(dcss, base_addr + devtype->dpr_ofs);
	if (ret)
		goto dpr_err;

	ret = dcss_wrscl_init(dcss, base_addr + devtype->wrscl_ofs);
	if (ret)
		goto wrscl_err;

	ret = dcss_rdsrc_init(dcss, base_addr + devtype->rdsrc_ofs);
	if (ret)
		goto rdsrc_err;

	ret = dcss_scaler_init(dcss, base_addr + devtype->scaler_ofs);
	if (ret)
		goto scaler_err;

	ret = dcss_dec400d_init(dcss, base_addr + devtype->dec400d_ofs);
	if (ret)
		goto dec400d_err;

	ret = dcss_hdr10_init(dcss, base_addr + devtype->hdr10_ofs);
	if (ret)
		goto hdr10_err;

	return 0;

hdr10_err:
	dcss_dec400d_exit(dcss->dec400d);

dec400d_err:
	dcss_scaler_exit(dcss->scaler, true);

scaler_err:
	dcss_rdsrc_exit(dcss->rdsrc);

rdsrc_err:
	dcss_wrscl_exit(dcss->wrscl);

wrscl_err:
	dcss_dpr_exit(dcss->dpr, true);

dpr_err:
	dcss_dtrc_exit(dcss->dtrc);

dtrc_err:
	dcss_ss_exit(dcss->ss, true);

ss_err:
	dcss_dtg_exit(dcss->dtg);

dtg_err:
	dcss_ctxld_exit(dcss->ctxld);

ctxld_err:
	dcss_blkctl_exit(dcss->blkctl);

	dcss_clocks_disable(dcss);

	return ret;
}

static void dcss_submodules_stop(struct dcss_dev *dcss, bool stop)
{
	dcss_clocks_enable(dcss);
	dcss_hdr10_exit(dcss->hdr10);
	dcss_dec400d_exit(dcss->dec400d);
	dcss_scaler_exit(dcss->scaler, stop);
	dcss_rdsrc_exit(dcss->rdsrc);
	dcss_wrscl_exit(dcss->wrscl);
	dcss_dpr_exit(dcss->dpr, stop);
	dcss_dtrc_exit(dcss->dtrc);
	dcss_ss_exit(dcss->ss, stop);
	dcss_dtg_exit(dcss->dtg);
	dcss_ctxld_exit(dcss->ctxld);
	dcss_blkctl_exit(dcss->blkctl);
	if (stop)
		dcss_clocks_disable(dcss);
}

static int dcss_clks_init(struct dcss_dev *dcss)
{
	dcss->apb_clk = devm_clk_get(dcss->dev, "apb");
	if (IS_ERR(dcss->apb_clk)) {
		dev_err(dcss->dev, "failed to get %s clock\n", "apb");
		return PTR_ERR(dcss->apb_clk);
	}
	dcss->axi_clk = devm_clk_get(dcss->dev, "axi");
	if (IS_ERR(dcss->axi_clk)) {
		dev_err(dcss->dev, "failed to get %s clock\n", "axi");
		return PTR_ERR(dcss->axi_clk);
	}
	dcss->rtrm_clk = devm_clk_get(dcss->dev, "rtrm");
	if (IS_ERR(dcss->rtrm_clk)) {
		dev_err(dcss->dev, "failed to get %s clock\n", "rtrm");
		return PTR_ERR(dcss->rtrm_clk);
	}
	dcss->dtrc_clk = devm_clk_get(dcss->dev, "dtrc");
	if (IS_ERR(dcss->dtrc_clk)) {
		dev_err(dcss->dev, "failed to get %s clock\n", "dtrc");
		return PTR_ERR(dcss->dtrc_clk);
	}

	return 0;
}

static struct dcss_type_data dcss_types[] = {
	[DCSS_IMX8MQ] = {
		.name = "DCSS_IMX8MQ",
		.blkctl_ofs = 0x2F000,
		.ctxld_ofs = 0x23000,
		.dtg_ofs = 0x20000,
		.rdsrc_ofs = 0x22000,
		.wrscl_ofs = 0x21000,
		.scaler_ofs = 0x1C000,
		.ss_ofs = 0x1B000,
		.dpr_ofs = 0x18000,
		.dec400d_ofs = 0x15000,
		.hdr10_ofs = 0x00000,
		.dtrc_ofs = 0x16000,
	},
};

struct dcss_dev *dcss_dev_create(struct device *dev, bool hdmi_output)
{
	struct platform_device *pdev = to_platform_device(dev);
	int ret;
	struct resource *res;
	struct dcss_dev *dcss;
	const struct dcss_type_data *devtype;

	devtype = &dcss_types[DCSS_IMX8MQ];
	if (!devtype) {
		dev_err(dev, "no device match found\n");
		return ERR_PTR(-ENODEV);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "cannot get memory resource\n");
		return ERR_PTR(-EINVAL);
	}

	dcss = kzalloc(sizeof(*dcss), GFP_KERNEL);
	if (!dcss)
		return ERR_PTR(-ENOMEM);

	dcss->dev = dev;
	dcss->devtype = devtype;
	dcss->hdmi_output = hdmi_output;

	ret = dcss_clks_init(dcss);
	if (ret) {
		dev_err(dev, "clocks initialization failed\n");
		goto err;
	}

	dcss->start_addr = (u32)res->start;

	ret = dcss_submodules_init(dcss);
	if (ret) {
		dev_err(dev, "submodules initialization failed\n");
		goto err;
	}

	init_completion(&dcss->disable_completion);

	return dcss;

err:
	kfree(dcss);

	return ERR_PTR(ret);
}

void dcss_dev_destroy(struct dcss_dev *dcss, bool stop)
{
	dcss_ctxld_suspend(dcss->ctxld);

	dcss_submodules_stop(dcss, stop);

	kfree(dcss);
}

#ifdef CONFIG_PM_SLEEP
int dcss_dev_suspend(struct device *dev)
{
	struct dcss_dev *dcss = dcss_drv_dev_to_dcss(dev);
	struct drm_device *ddev = dcss_drv_dev_to_drm(dev);
	struct dcss_kms_dev *kms = container_of(ddev, struct dcss_kms_dev, base);
	int ret;

	if (!dcss)
		return 0;

	if (!dcss_drv_is_componentized(dev))
		drm_bridge_connector_disable_hpd(kms->connector);

	drm_mode_config_helper_suspend(ddev);

	if (pm_runtime_suspended(dev))
		return 0;

	ret = dcss_ctxld_suspend(dcss->ctxld);
	if (ret)
		return ret;

	dcss_clocks_disable(dcss);

	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

int dcss_dev_resume(struct device *dev)
{
	struct dcss_dev *dcss = dcss_drv_dev_to_dcss(dev);
	struct drm_device *ddev = dcss_drv_dev_to_drm(dev);
	struct dcss_kms_dev *kms = container_of(ddev, struct dcss_kms_dev, base);

	if (!dcss)
		return 0;

	if (pm_runtime_suspended(dev)) {
		drm_mode_config_helper_resume(ddev);
		return 0;
	}

	request_bus_freq(BUS_FREQ_HIGH);

	dcss_clocks_enable(dcss);

	dcss_blkctl_cfg(dcss->blkctl);

	dcss_ctxld_resume(dcss->ctxld);

	drm_mode_config_helper_resume(ddev);

	if (!dcss_drv_is_componentized(dev))
		drm_bridge_connector_enable_hpd(kms->connector);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM
int dcss_dev_runtime_suspend(struct device *dev)
{
	struct dcss_dev *dcss = dcss_drv_dev_to_dcss(dev);
	int ret;

	if (!dcss)
		return 0;

	ret = dcss_ctxld_suspend(dcss->ctxld);
	if (ret)
		return ret;

	dcss_clocks_disable(dcss);

	return 0;
}

int dcss_dev_runtime_resume(struct device *dev)
{
	struct dcss_dev *dcss = dcss_drv_dev_to_dcss(dev);

	if (!dcss)
		return 0;

	dcss_clocks_enable(dcss);

	dcss_blkctl_cfg(dcss->blkctl);

	dcss_ctxld_resume(dcss->ctxld);

	return 0;
}
#endif /* CONFIG_PM */
