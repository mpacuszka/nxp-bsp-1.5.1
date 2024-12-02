/*
 * Cadence High-Definition Multimedia Interface (HDMI) driver
 *
 * Copyright 2019-2022 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <asm-generic/bug.h>
#include <drm/bridge/cdns-mhdp.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder.h>
#include <drm/drm_print.h>
#include <drm/drm_scdc_helper.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hdmi.h>
#include <linux/irq.h>
#include <linux/of_device.h>

#include "cdns-mhdp-hdcp.h"

static void hdmi_sink_config(struct cdns_mhdp_device *mhdp)
{
	struct drm_scdc *scdc = &mhdp->connector.base.display_info.hdmi.scdc;
	u8 buff = 0;

	/* return if hdmi work in DVI mode */
	if (mhdp->hdmi.hdmi_type == MODE_DVI)
		return;

	/* check sink support SCDC or not */
	if (scdc->supported != true) {
		DRM_INFO("Sink Not Support SCDC\n");
		return;
	}

	if (mhdp->hdmi.char_rate > 340000) {
		/*
		 * TMDS Character Rate above 340MHz should working in HDMI2.0
		 * Enable scrambling and TMDS_Bit_Clock_Ratio
		 */
		buff = SCDC_TMDS_BIT_CLOCK_RATIO_BY_40 | SCDC_SCRAMBLING_ENABLE;
		mhdp->hdmi.hdmi_type = MODE_HDMI_2_0;
	} else  if (scdc->scrambling.low_rates) {
		/*
		 * Enable scrambling and HDMI2.0 when scrambling capability of sink
		 * be indicated in the HF-VSDB LTE_340Mcsc_scramble bit
		 */
		buff = SCDC_SCRAMBLING_ENABLE;
		mhdp->hdmi.hdmi_type = MODE_HDMI_2_0;
	}

	/* TMDS config */
	cdns_hdmi_scdc_write(mhdp, 0x20, buff);
}

static void hdmi_lanes_config(struct cdns_mhdp_device *mhdp)
{
	/* Line swaping */
	cdns_mhdp_reg_write(mhdp, LANES_CONFIG, 0x00400000 | mhdp->lane_mapping);
}

static int hdmi_avi_info_set(struct cdns_mhdp_device *mhdp,
	struct drm_display_mode *mode)
{
	struct hdmi_avi_infoframe frame;
	int format = mhdp->video_info.color_fmt;
	struct drm_connector_state *conn_state = mhdp->connector.base.state;
	struct drm_display_mode *adj_mode;
	enum hdmi_quantization_range qr;
	u8 buf[32];
	int ret;

	/* Initialise info frame from DRM mode */
	drm_hdmi_avi_infoframe_from_display_mode(&frame, &mhdp->connector.base,
		mode);

	switch (format) {
	case YCBCR_4_4_4:
		frame.colorspace = HDMI_COLORSPACE_YUV444;
		break;
	case YCBCR_4_2_2:
		frame.colorspace = HDMI_COLORSPACE_YUV422;
		break;
	case YCBCR_4_2_0:
		frame.colorspace = HDMI_COLORSPACE_YUV420;
		break;
	default:
		frame.colorspace = HDMI_COLORSPACE_RGB;
		break;
	}

	drm_hdmi_avi_infoframe_colorspace(&frame, conn_state);

	adj_mode = &mhdp->bridge.base.encoder->crtc->state->adjusted_mode;

	qr = drm_default_rgb_quant_range(adj_mode);

	drm_hdmi_avi_infoframe_quant_range(&frame, &mhdp->connector.base,
		adj_mode, qr);

	ret = hdmi_avi_infoframe_check(&frame);
	if (ret < 0) {
		WARN_ON(ret);
		return false;
	}

	ret = (int)hdmi_avi_infoframe_pack(&frame, buf + 1, (int)sizeof(buf) - 1);
	if (ret < 0) {
		DRM_ERROR("failed to pack AVI infoframe: %d\n", ret);
		return -1;
	}

	buf[0] = 0;
	cdns_mhdp_infoframe_set(mhdp, 0, sizeof(buf), buf, HDMI_INFOFRAME_TYPE_AVI);
	return 0;
}

static void hdmi_vendor_info_set(struct cdns_mhdp_device *mhdp,
	struct drm_display_mode *mode)
{
	struct hdmi_vendor_infoframe frame;
	u8 buf[32];
	int ret;

	/* Initialise vendor frame from DRM mode */
	ret = drm_hdmi_vendor_infoframe_from_display_mode(&frame, &mhdp->connector.base, mode);
	if (ret < 0) {
		DRM_INFO("No vendor infoframe\n");
		return;
	}

	ret = (int)hdmi_vendor_infoframe_pack(&frame, buf + 1, (int)sizeof(buf) - 1);
	if (ret < 0) {
		DRM_WARN("Unable to pack vendor infoframe: %d\n", ret);
		return;
	}

	buf[0] = 0;
	cdns_mhdp_infoframe_set(mhdp, 3, sizeof(buf), buf, HDMI_INFOFRAME_TYPE_VENDOR);
}

static void hdmi_drm_info_set(struct cdns_mhdp_device *mhdp)
{
	struct drm_connector_state *conn_state;
	struct hdmi_drm_infoframe frame;
	u8 buf[32];
	int ret;

	conn_state = mhdp->connector.base.state;

	if (!conn_state->hdr_output_metadata)
		return;

	ret = drm_hdmi_infoframe_set_hdr_metadata(&frame, conn_state);
	if (ret < 0) {
		DRM_DEBUG_KMS("couldn't set HDR metadata in infoframe\n");
		return;
	}

	ret = (int)hdmi_drm_infoframe_pack(&frame, buf + 1, (int)sizeof(buf) - 1);
	if (ret < 0) {
		DRM_DEBUG_KMS("couldn't pack HDR infoframe\n");
		return;
	}

	buf[0] = 0;
	cdns_mhdp_infoframe_set(mhdp, 3, sizeof(buf),
		buf, HDMI_INFOFRAME_TYPE_DRM);
}

void cdns_hdmi_mode_set(struct cdns_mhdp_device *mhdp)
{
	struct drm_display_mode *mode = &mhdp->mode;
	int ret;

	/* video mode valid check */
	if (mode->clock == 0 || mode->hdisplay == 0 || mode->vdisplay == 0)
		return;

	hdmi_lanes_config(mhdp);

	cdns_mhdp_plat_call_void(mhdp, pclk_rate);

	/* delay for HDMI FW stable after pixel clock relock */
	msleep(20);

	cdns_mhdp_plat_call(mhdp, phy_set);

	hdmi_sink_config(mhdp);

	ret = cdns_hdmi_ctrl_init(mhdp, mhdp->hdmi.hdmi_type, mhdp->hdmi.char_rate);
	if (ret < 0) {
		DRM_ERROR("%s, ret = %d\n", __func__, ret);
		return;
	}

	/* Config GCP */
	if (mhdp->video_info.color_depth == 8)
		cdns_hdmi_disable_gcp(mhdp);
	else
		cdns_hdmi_enable_gcp(mhdp);

	ret = hdmi_avi_info_set(mhdp, mode);
	if (ret < 0) {
		DRM_ERROR("%s ret = %d\n", __func__, ret);
		return;
	}

	/* vendor info frame is enable only  when HDMI1.4 4K mode */
	hdmi_vendor_info_set(mhdp, mode);

	hdmi_drm_info_set(mhdp);

	ret = cdns_hdmi_mode_config(mhdp, mode, &mhdp->video_info);
	if (ret < 0) {
		DRM_ERROR("CDN_API_HDMITX_SetVic_blocking ret = %d\n", ret);
		return;
	}
}

void cdns_hdmi_bridge_mode_set(struct drm_bridge *bridge,
	const struct drm_display_mode *orig_mode,
	const struct drm_display_mode *mode)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	struct video_info *video = &mhdp->video_info;

	video->v_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NVSYNC);
	video->h_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NHSYNC);

	DRM_INFO("Mode: %dx%dp%d\n", mode->hdisplay, mode->vdisplay, mode->clock);
	memcpy(&mhdp->mode, mode, sizeof(struct drm_display_mode));

	mutex_lock(&mhdp->lock);
	cdns_hdmi_mode_set(mhdp);
	mutex_unlock(&mhdp->lock);
}

bool cdns_hdmi_bridge_mode_fixup(struct drm_bridge *bridge,
	const struct drm_display_mode *mode,
	struct drm_display_mode *adjusted_mode)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	struct drm_connector_state *conn_state = mhdp->connector.base.state;
	struct drm_display_info *di = &mhdp->connector.base.display_info;
	struct video_info *video = &mhdp->video_info;
	int vic = drm_match_cea_mode(mode);

	video->color_depth = 8;
	video->color_fmt = PXL_RGB;

	/* for all other platforms, other than imx8mq */
	if (strncmp("imx8mq-hdmi", mhdp->plat_data->plat_name, 11)) {
		if (di->bpc == 10 || di->bpc == 6)
			video->color_depth = di->bpc;

		return true;
	}

	/* imx8mq */
	if (conn_state->colorspace == DRM_MODE_COLORIMETRY_DEFAULT)
		return !drm_mode_is_420_only(di, mode);

	if (conn_state->colorspace == DRM_MODE_COLORIMETRY_BT2020_RGB) {
		if (drm_mode_is_420_only(di, mode))
			return false;

		/* 10b RGB is not supported for following VICs */
		if (vic == 97 || vic == 96 || vic == 95 || vic == 93 || vic == 94)
			return false;

		video->color_depth = 10;

		return true;
	}

	if (conn_state->colorspace == DRM_MODE_COLORIMETRY_BT2020_CYCC ||
		conn_state->colorspace == DRM_MODE_COLORIMETRY_BT2020_YCC) {
		if (drm_mode_is_420_only(di, mode)) {
			video->color_fmt = YCBCR_4_2_0;

			if (di->hdmi.y420_dc_modes & DRM_EDID_YCBCR420_DC_36)
				video->color_depth = 12;
			else if (di->hdmi.y420_dc_modes & DRM_EDID_YCBCR420_DC_30)
				video->color_depth = 10;
			else
				return false;

			return true;
		}

		video->color_fmt = YCBCR_4_2_2;

		if (!(di->edid_hdmi_dc_modes & DRM_EDID_HDMI_DC_36))
			return false;

		video->color_depth = 12;

		return true;
	}

	video->color_fmt = drm_mode_is_420_only(di, mode) ? YCBCR_4_2_0 : YCBCR_4_4_4;
	video->color_depth = 8;

	return true;
}

static void cdns_hdmi_parse_dt(struct cdns_mhdp_device *mhdp)
{
	struct device_node *of_node = &mhdp->dev->of_node;
	int ret;

	ret = of_property_read_u32(of_node, "lane-mapping", &mhdp->lane_mapping);
	if (ret) {
		mhdp->lane_mapping = 0xc6;
		dev_warn(mhdp->dev, "Failed to get lane_mapping - using default 0xc6\n");
	}
	dev_info(mhdp->dev, "lane-mapping 0x%02x\n", mhdp->lane_mapping);
}

static int __cdns_hdmi_probe(struct platform_device *pdev,
	struct cdns_mhdp_device *mhdp)
{
	struct device *dev = &pdev->dev;
	struct resource *iores = NULL;
	int ret;

	mutex_init(&mhdp->lock);
	mutex_init(&mhdp->iolock);

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mhdp->regs_base = devm_ioremap(dev, iores->start, resource_size(iores));
	if (IS_ERR(mhdp->regs_base)) {
		dev_err(dev, "No regs_base memory\n");
		return -ENOMEM;
	}

	/* sec register base */
	iores = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mhdp->regs_sec = devm_ioremap(dev, iores->start, resource_size(iores));
	if (IS_ERR(mhdp->regs_sec)) {
		dev_err(dev, "No regs_sec memory\n");
		return -ENOMEM;
	}

	cdns_mhdp_plat_call(mhdp, power_on);

	/* Initialize FW */
	cdns_mhdp_plat_call(mhdp, firmware_init);

	/* HDMI FW alive check */
	ret = cdns_mhdp_check_alive(mhdp);
	if (ret == false) {
		dev_err(dev, "NO HDMI FW running\n");
		return -ENXIO;
	}

	cdns_hdmi_parse_dt(mhdp);

	/* TODO: get actual hdmi type from the EDID */
	mhdp->hdmi.hdmi_type = MODE_HDMI_1_4;

	if (cdns_mhdp_read_hpd(mhdp))
		DRM_INFO("hdmi cable connected");
	else
		DRM_INFO("hdmi cable disconnected");

	mhdp->bridge.base.driver_private = mhdp;

	dev_set_drvdata(dev, mhdp);

	return 0;
}

/* -----------------------------------------------------------------------------
 * Bind/unbind API, used from platforms based on the component framework.
 */
int cdns_hdmi_bind(struct platform_device *pdev, struct drm_encoder *encoder,
	struct cdns_mhdp_device *mhdp)
{
	int ret;

	ret = __cdns_hdmi_probe(pdev, mhdp);
	if (ret)
		return ret;

	encoder->bridge = &mhdp->bridge.base;
	mhdp->bridge.base.encoder = encoder;

	return 0;
}

void cdns_hdmi_unbind(struct device *dev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	kfree(mhdp);
}
