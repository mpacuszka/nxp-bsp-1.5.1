/*
 * Copyright 2019-2020, 2022 NXP
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the gnu general public license version 2 as
 * published by the free software foundation.
 */
#include <linux/err.h>
#include <linux/platform_device.h>
#include <drm/drm_crtc.h>

#include "cdns-mhdp-imx.h"
#include "cdns-mhdp-phy.h"

struct drm_encoder;
struct drm_connector_state;

void cdns_mhdp_imx_encoder_disable(struct drm_encoder *encoder)
{
	struct drm_bridge *bridge = encoder->bridge;
	struct cdns_mhdp_device *mhdp = bridge->driver_private;

	cdns_hdmi_phy_shutdown(mhdp);
	cdns_mhdp_plat_call_void(mhdp, plat_deinit);
}

void cdns_mhdp_imx_encoder_enable(struct drm_encoder *encoder)
{
	struct drm_bridge *bridge = encoder->bridge;
	struct cdns_mhdp_device *mhdp = bridge->driver_private;

	cdns_mhdp_plat_call_void(mhdp, plat_init);
	cdns_hdmi_phy_power_up(mhdp);
}

static struct cdns_plat_data imx8mq_hdmi_drv_data = {
	.plat_name = "imx8mq-hdmi",
	.bind = cdns_hdmi_bind,
	.unbind = cdns_hdmi_unbind,
	.phy_set = cdns_hdmi_phy_set_imx8mq,
	.phy_video_valid = cdns_hdmi_phy_video_valid_imx8mq,
	.bus_type = BUS_TYPE_NORMAL_APB,
};

int cdns_mhdp_imx_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct cdns_plat_data *plat_data;
	struct drm_encoder *encoder;
	struct imx_mhdp_device *imx_mhdp;
	int ret;

	imx_mhdp = devm_kzalloc(&pdev->dev, sizeof(*imx_mhdp), GFP_KERNEL);
	if (!imx_mhdp)
		return -ENOMEM;

	plat_data = &imx8mq_hdmi_drv_data;
	encoder = &imx_mhdp->encoder;

	encoder->encoder_type = DRM_MODE_ENCODER_TMDS;

	imx_mhdp->mhdp.plat_data = plat_data;
	imx_mhdp->mhdp.dev = dev;
	imx_mhdp->mhdp.bus_type = plat_data->bus_type;
	ret = plat_data->bind(pdev, encoder, &imx_mhdp->mhdp);

	return ret;
}

void cdns_mhdp_imx_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct imx_mhdp_device *imx_mhdp = dev_get_drvdata(dev);

	imx_mhdp->mhdp.plat_data->unbind(dev);
}

static int cdns_mhdp_imx_suspend(struct device *dev)
{
	struct imx_mhdp_device *imx_mhdp = dev_get_drvdata(dev);

	cdns_mhdp_plat_call(&imx_mhdp->mhdp, suspend);

	return 0;
}

static int cdns_mhdp_imx_resume(struct device *dev)
{
	struct imx_mhdp_device *imx_mhdp = dev_get_drvdata(dev);

	cdns_mhdp_plat_call(&imx_mhdp->mhdp, resume);

	return 0;
}
