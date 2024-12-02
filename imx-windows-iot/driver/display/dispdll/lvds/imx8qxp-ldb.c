// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020, 2022 NXP
 */

#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include "linux/firmware/imx/ipc.h"
#include <linux/media-bus-format.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mixel-lvds-combo.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>

#include <drm/bridge/fsl_imx_ldb.h>
#include <drm/drm_encoder.h>

#include <svc/pm/pm_api.h>
#include <svc/misc/misc_api.h>

#include "imx8qxp-ldb.h"
#include "drm/imx/imx-drm.h"

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_CH1_MODE_EN_TO_DI0		(1 << 2)
#define LDB_CH1_MODE_EN_TO_DI1		(3 << 2)
#define LDB_CH1_MODE_EN_MASK		(3 << 2)
#define LDB_CH0_DATA_WIDTH_24BIT	(1 << 5)
#define LDB_BIT_MAP_CH0_JEIDA		(1 << 6)
#define LDB_CH1_DATA_WIDTH_24BIT	(1 << 7)
#define LDB_BIT_MAP_CH1_JEIDA		(1 << 8)
#define LDB_DI0_VS_POL_ACT_LOW		(1 << 9)
#define LDB_DI1_VS_POL_ACT_LOW		(1 << 10)
#define LDB_CH0_10BIT_EN		(1 << 22)
#define LDB_CH1_10BIT_EN		(1 << 23)
#define LDB_CH0_DATA_WIDTH_30BIT	(2 << 24)
#define LDB_CH1_DATA_WIDTH_30BIT	(2 << 26)
#define LDB_CH_SEL			(1 << 28)

struct imx8qxp_ldb;

struct imx8qxp_ldb_channel {
	struct ldb_channel base;
	struct imx8qxp_ldb *imx8qxp_ldb;

	struct drm_connector connector;
	struct drm_encoder encoder;

	struct phy *phy;
	struct phy *aux_phy;
	bool phy_is_on;

	u32 bus_flags;
};

static inline struct imx8qxp_ldb_channel *
con_to_imx8qxp_ldb_ch(struct drm_connector *c)
{
	return container_of(c, struct imx8qxp_ldb_channel, connector);
}

static inline struct imx8qxp_ldb_channel *
enc_to_imx8qxp_ldb_ch(struct drm_encoder *e)
{
	return container_of(e, struct imx8qxp_ldb_channel, encoder);
}

struct imx8qxp_ldb {
	struct ldb base;
	struct regmap *aux_regmap;
	struct imx8qxp_ldb_channel channel[LDB_CH_NUM];
	struct clk *clk_pixel;
	struct clk *clk_bypass;
	struct clk *clk_aux_pixel;
	struct clk *clk_aux_bypass;
	struct _sc_ipc_struct_t *handle;

	int pd_main_dev;
	int pd_aux_dev;

	int id;
};

struct drm_encoder *imx8qxp_ldb_get_encoder(struct platform_device *pdev,
	int ch)
{
	if (!pdev)
		return NULL;
	if (ch < 0 || ch >= LDB_CH_NUM)
		return NULL;

	struct imx8qxp_ldb *ldb = dev_get_drvdata(&pdev->dev);
	if (!ldb)
		return NULL;

	return &ldb->channel[ch].encoder;
}

u32 imx8qxp_ldb_get_bus_format(struct platform_device *pdev, int ch)
{
	if (!pdev)
		return 0;
	if (ch < 0 || ch >= LDB_CH_NUM)
		return 0;

	struct imx8qxp_ldb *ldb = dev_get_drvdata(&pdev->dev);
	if (!ldb)
		return 0;

	return ldb->channel[ch].base.bus_format;
}

static void
imx8qxp_ldb_ch_set_bus_format(struct imx8qxp_ldb_channel *imx8qxp_ldb_ch,
			      u32 bus_format)
{
	struct imx8qxp_ldb *imx8qxp_ldb = imx8qxp_ldb_ch->imx8qxp_ldb;
	struct ldb *ldb = &imx8qxp_ldb->base;
	struct ldb_channel *ldb_ch = &imx8qxp_ldb_ch->base;

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH0_DATA_WIDTH_24BIT;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH1_DATA_WIDTH_24BIT;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH0_DATA_WIDTH_24BIT |
						LDB_BIT_MAP_CH0_JEIDA;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH1_DATA_WIDTH_24BIT |
						LDB_BIT_MAP_CH1_JEIDA;
		break;
	case MEDIA_BUS_FMT_RGB101010_1X7X5_SPWG:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH0_10BIT_EN |
					 LDB_CH0_DATA_WIDTH_30BIT;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH1_10BIT_EN |
					 LDB_CH1_DATA_WIDTH_30BIT;
		break;
	case MEDIA_BUS_FMT_RGB101010_1X7X5_JEIDA:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH0_10BIT_EN |
					 LDB_CH0_DATA_WIDTH_30BIT |
					 LDB_BIT_MAP_CH0_JEIDA;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_CH1_10BIT_EN |
					 LDB_CH1_DATA_WIDTH_30BIT |
					 LDB_BIT_MAP_CH1_JEIDA;
		break;
	}
}

static inline int ldb_sc_misc_set_ctrl(struct imx8qxp_ldb *imx8qxp_ldb,
	u32 rsc, u8 ctrl, u32 val)
{
	if (!imx8qxp_ldb || !imx8qxp_ldb->handle) {
		WARN_ON(1);
		return -EINVAL;
	}

	sc_err_t err = sc_misc_set_control(imx8qxp_ldb->handle, (u16)rsc,
		ctrl, val);
	if (err != SC_ERR_NONE) {
		dev_err(NULL, "sc_misc_set_control() failed! (err-%d)\n", err);
		return -EIO;
	}

	return 0;
}

static void imx8qxp_ldb_pxlink_enable(struct imx8qxp_ldb *imx8qxp_ldb,
				      int stream_id, bool enable)
{
	u8 ctrl = stream_id ?
		IMX_SC_C_PXL_LINK_MST2_ENB : IMX_SC_C_PXL_LINK_MST1_ENB;

	ldb_sc_misc_set_ctrl(imx8qxp_ldb, IMX_SC_R_DC_0, ctrl, enable);
}

static void imx8qxp_ldb_pxlink_set_mst_valid(struct imx8qxp_ldb *imx8qxp_ldb,
					     int stream_id, bool enable)
{
	u8 ctrl = stream_id ?
		IMX_SC_C_PXL_LINK_MST2_VLD : IMX_SC_C_PXL_LINK_MST1_VLD;

	ldb_sc_misc_set_ctrl(imx8qxp_ldb, IMX_SC_R_DC_0, ctrl, enable);
}

static void imx8qxp_ldb_pxlink_set_sync_ctrl(struct imx8qxp_ldb *imx8qxp_ldb,
					     int stream_id, bool enable)
{
	u8 ctrl = stream_id ? IMX_SC_C_SYNC_CTRL1 : IMX_SC_C_SYNC_CTRL0;

	ldb_sc_misc_set_ctrl(imx8qxp_ldb, IMX_SC_R_DC_0, ctrl, enable);
}

void imx8qxp_ldb_encoder_enable(struct drm_encoder *encoder)
{
	struct imx8qxp_ldb_channel *imx8qxp_ldb_ch =
						enc_to_imx8qxp_ldb_ch(encoder);
	struct imx8qxp_ldb *imx8qxp_ldb = imx8qxp_ldb_ch->imx8qxp_ldb;
	struct ldb *ldb = &imx8qxp_ldb->base;

	clk_prepare_enable(imx8qxp_ldb->clk_pixel);
	clk_prepare_enable(imx8qxp_ldb->clk_bypass);

	if (ldb->dual) {
		clk_prepare_enable(imx8qxp_ldb->clk_aux_pixel);
		clk_prepare_enable(imx8qxp_ldb->clk_aux_bypass);
	}

	/*
	 * LDB frontend doesn't know if the auxiliary LDB is used or not.
	 * Enable pixel link after dual or single LDB clocks are enabled
	 * so that the dual LDBs are synchronized.
	 */
	imx8qxp_ldb_pxlink_enable(imx8qxp_ldb, imx8qxp_ldb->id, true);

	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[0] || ldb->dual) {
		ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
		ldb->ldb_ctrl |= LDB_CH0_MODE_EN_TO_DI0;
	}
	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[1] || ldb->dual) {
		ldb->ldb_ctrl &= ~LDB_CH1_MODE_EN_MASK;
		ldb->ldb_ctrl |= ldb->dual ?
				LDB_CH1_MODE_EN_TO_DI0 : LDB_CH1_MODE_EN_TO_DI1;
	}

	regmap_write(ldb->regmap, ldb->ctrl_reg, ldb->ldb_ctrl);
	if (ldb->dual)
		regmap_write(imx8qxp_ldb->aux_regmap, ldb->ctrl_reg,
						ldb->ldb_ctrl | LDB_CH_SEL);

	if (ldb->dual) {
		mixel_lvds_combo_phy_power_on(imx8qxp_ldb->channel[0].phy);
		mixel_lvds_combo_phy_power_on(imx8qxp_ldb->channel[0].aux_phy);

		imx8qxp_ldb->channel[0].phy_is_on = true;
	} else {
		mixel_lvds_combo_phy_power_on(imx8qxp_ldb_ch->phy);

		imx8qxp_ldb_ch->phy_is_on = true;
	}

	imx8qxp_ldb_pxlink_set_mst_valid(imx8qxp_ldb, imx8qxp_ldb->id, true);
	imx8qxp_ldb_pxlink_set_sync_ctrl(imx8qxp_ldb, imx8qxp_ldb->id, true);
}

void
imx8qxp_ldb_encoder_atomic_mode_set(struct drm_encoder *encoder,
				    struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *connector_state)
{
	struct imx8qxp_ldb_channel *imx8qxp_ldb_ch =
						enc_to_imx8qxp_ldb_ch(encoder);
	struct imx8qxp_ldb *imx8qxp_ldb = imx8qxp_ldb_ch->imx8qxp_ldb;
	struct ldb_channel *ldb_ch = &imx8qxp_ldb_ch->base;
	struct ldb *ldb = &imx8qxp_ldb->base;
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	unsigned long di_clk = mode->clock * 1000;

	if (mode->clock > 300000) {
		dev_warn(ldb->dev,
			 "%s: mode exceeds 300 MHz pixel clock\n", __func__);
	}
	if (mode->clock > 150000 && !ldb->dual) {
		dev_warn(ldb->dev,
			 "%s: mode exceeds 150 MHz pixel clock\n", __func__);
	}

	clk_set_rate(imx8qxp_ldb->clk_bypass, di_clk);
	clk_set_rate(imx8qxp_ldb->clk_pixel, di_clk);

	if (ldb->dual) {
		clk_set_rate(imx8qxp_ldb->clk_aux_bypass, di_clk);
		clk_set_rate(imx8qxp_ldb->clk_aux_pixel, di_clk);
	}

	if (ldb->dual) {
		mixel_phy_combo_lvds_set_phy_speed(imx8qxp_ldb->channel[0].phy,
								di_clk / 2);
		mixel_phy_combo_lvds_set_phy_speed(imx8qxp_ldb->channel[0].aux_phy,
								di_clk / 2);
	} else {
		mixel_phy_combo_lvds_set_phy_speed(imx8qxp_ldb_ch->phy, di_clk);
	}

	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[0])
		ldb->ldb_ctrl &= ~LDB_CH_SEL;
	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[1])
		ldb->ldb_ctrl |= LDB_CH_SEL;

	/* FIXME - assumes straight connections DI0 --> CH0, DI1 --> CH1 */
	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[0] || ldb->dual) {
		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			ldb->ldb_ctrl |= LDB_DI0_VS_POL_ACT_LOW;
		else if (mode->flags & DRM_MODE_FLAG_PVSYNC)
			ldb->ldb_ctrl &= ~LDB_DI0_VS_POL_ACT_LOW;
	}
	if (imx8qxp_ldb_ch == &imx8qxp_ldb->channel[1] || ldb->dual) {
		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			ldb->ldb_ctrl |= LDB_DI1_VS_POL_ACT_LOW;
		else if (mode->flags & DRM_MODE_FLAG_PVSYNC)
			ldb->ldb_ctrl &= ~LDB_DI1_VS_POL_ACT_LOW;
	}

	/* settle vsync polarity and channel selection down early */
	if (ldb->dual) {
		regmap_write(ldb->regmap, ldb->ctrl_reg, ldb->ldb_ctrl);
		regmap_write(imx8qxp_ldb->aux_regmap, ldb->ctrl_reg,
						ldb->ldb_ctrl | LDB_CH_SEL);
	}

	if (ldb->dual) {
		/* VSYNC */
		if (mode->flags & DRM_MODE_FLAG_NVSYNC) {
			mixel_phy_combo_lvds_set_vsync_pol(
				imx8qxp_ldb->channel[0].phy, false);
			mixel_phy_combo_lvds_set_vsync_pol(
				imx8qxp_ldb->channel[0].aux_phy, false);
		} else {
			mixel_phy_combo_lvds_set_vsync_pol(
				imx8qxp_ldb->channel[0].phy, true);
			mixel_phy_combo_lvds_set_vsync_pol(
				imx8qxp_ldb->channel[0].aux_phy, true);
		}
		/* HSYNC */
		if (mode->flags & DRM_MODE_FLAG_NHSYNC) {
			mixel_phy_combo_lvds_set_hsync_pol(
				imx8qxp_ldb->channel[0].phy, false);
			mixel_phy_combo_lvds_set_hsync_pol(
				imx8qxp_ldb->channel[0].aux_phy, false);
		} else {
			mixel_phy_combo_lvds_set_hsync_pol(
				imx8qxp_ldb->channel[0].phy, true);
			mixel_phy_combo_lvds_set_hsync_pol(
				imx8qxp_ldb->channel[0].aux_phy, true);
		}
	} else {
		/* VSYNC */
		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			mixel_phy_combo_lvds_set_vsync_pol(imx8qxp_ldb_ch->phy,
							   false);
		else if (mode->flags & DRM_MODE_FLAG_PVSYNC)
			mixel_phy_combo_lvds_set_vsync_pol(imx8qxp_ldb_ch->phy,
							   true);
		/* HSYNC */
		if (mode->flags & DRM_MODE_FLAG_NHSYNC)
			mixel_phy_combo_lvds_set_hsync_pol(imx8qxp_ldb_ch->phy,
							   false);
		else if (mode->flags & DRM_MODE_FLAG_PHSYNC)
			mixel_phy_combo_lvds_set_hsync_pol(imx8qxp_ldb_ch->phy,
							   true);
	}

	if (!ldb_ch->bus_format) {
		dev_warn(ldb->dev,
			"bus format undefined; getting format from connector\n");
		struct drm_connector *connector = connector_state->connector;
		struct drm_display_info *di = &connector->display_info;

		if (di->num_bus_formats)
			ldb_ch->bus_format = di->bus_formats[0];
	}
	imx8qxp_ldb_ch_set_bus_format(imx8qxp_ldb_ch, ldb_ch->bus_format);
}

void imx8qxp_ldb_encoder_disable(struct drm_encoder *encoder)
{
	struct imx8qxp_ldb_channel *imx8qxp_ldb_ch =
						enc_to_imx8qxp_ldb_ch(encoder);
	struct imx8qxp_ldb *imx8qxp_ldb = imx8qxp_ldb_ch->imx8qxp_ldb;
	struct ldb *ldb = &imx8qxp_ldb->base;

	imx8qxp_ldb_pxlink_set_mst_valid(imx8qxp_ldb, imx8qxp_ldb->id, false);
	imx8qxp_ldb_pxlink_set_sync_ctrl(imx8qxp_ldb, imx8qxp_ldb->id, false);

	if (ldb->dual) {
		mixel_lvds_combo_phy_power_off(imx8qxp_ldb->channel[0].phy);
		mixel_lvds_combo_phy_power_off(imx8qxp_ldb->channel[0].aux_phy);

		imx8qxp_ldb->channel[0].phy_is_on = false;
	} else {
		mixel_lvds_combo_phy_power_off(imx8qxp_ldb_ch->phy);

		imx8qxp_ldb_ch->phy_is_on = false;
	}

	if (ldb->dual)
		regmap_write(imx8qxp_ldb->aux_regmap,
					ldb->ctrl_reg, ldb->ldb_ctrl);

	clk_disable_unprepare(imx8qxp_ldb->clk_bypass);
	clk_disable_unprepare(imx8qxp_ldb->clk_pixel);

	if (ldb->dual) {
		clk_disable_unprepare(imx8qxp_ldb->clk_aux_bypass);
		clk_disable_unprepare(imx8qxp_ldb->clk_aux_pixel);
	}

	imx8qxp_ldb_pxlink_enable(imx8qxp_ldb, imx8qxp_ldb->id, false);
}

int
imx8qxp_ldb_encoder_atomic_check(struct drm_encoder *encoder,
				 struct drm_crtc_state *crtc_state,
				 struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct imx8qxp_ldb_channel *imx8qxp_ldb_ch =
						enc_to_imx8qxp_ldb_ch(encoder);
	struct ldb_channel *ldb_ch = &imx8qxp_ldb_ch->base;
	struct drm_display_info *di = &conn_state->connector->display_info;
	u32 bus_format = ldb_ch->bus_format;

	/* Bus format description in DT overrides connector display info. */
	if (!bus_format && di->num_bus_formats) {
		bus_format = di->bus_formats[0];
		imx_crtc_state->bus_flags = di->bus_flags;
	} else {
		bus_format = ldb_ch->bus_format;
		imx_crtc_state->bus_flags = imx8qxp_ldb_ch->bus_flags;
	}
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB666_1X30_PADLO;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB888_1X30_PADLO;
		break;
	case MEDIA_BUS_FMT_RGB101010_1X7X5_SPWG:
	case MEDIA_BUS_FMT_RGB101010_1X7X5_JEIDA:
		imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB101010_1X30;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void imx8qxp_ldb_detach_pm_domains(struct imx8qxp_ldb *imx8qxp_ldb)
{
	imx8qxp_ldb->pd_aux_dev = -1;
	imx8qxp_ldb->pd_main_dev = -1;
}

static int
imx8qxp_ldb_attach_pm_domains(struct imx8qxp_ldb *imx8qxp_ldb, bool dual)
{
	struct ldb *ldb = &imx8qxp_ldb->base;
	struct device *dev = ldb->dev;
	int ret = 0;

	imx8qxp_ldb->pd_main_dev = dev_pm_domain_attach_by_name(dev, "main");
	if (imx8qxp_ldb->pd_main_dev < 0) {
		ret = -EINVAL;
		dev_err(dev, "Failed to attach main pd dev: %d\n", ret);
		goto fail;
	}

	if (!dual)
		goto out;

	imx8qxp_ldb->pd_aux_dev = dev_pm_domain_attach_by_name(dev, "aux");
	if (imx8qxp_ldb->pd_aux_dev < 0) {
		ret = -EINVAL;
		dev_err(dev, "Failed to attach aux pd dev: %d\n", ret);
		goto fail;
	}

out:
	return ret;
fail:
	imx8qxp_ldb_detach_pm_domains(imx8qxp_ldb);
	return ret;
}

static int ldb_pwr_disable(struct imx8qxp_ldb *imx8qxp_ldb, bool dual)
{
	if (!imx8qxp_ldb || !imx8qxp_ldb->handle) {
		WARN_ON(1);
		return -EINVAL;
	}

	sc_err_t err = sc_pm_set_resource_power_mode(imx8qxp_ldb->handle,
		(u16)imx8qxp_ldb->pd_main_dev, SC_PM_PW_MODE_OFF);
	if (err != SC_ERR_NONE) {
		dev_err(imx8qxp_ldb->base.dev,
			"Failed to set pwr for imx8qxp_ldb main pd dev: %d\n", err);
		return -EIO;
	}

	if (!dual)
		return 0;

	err = sc_pm_set_resource_power_mode(imx8qxp_ldb->handle,
		(u16)imx8qxp_ldb->pd_aux_dev, SC_PM_PW_MODE_OFF);
	if (err != SC_ERR_NONE) {
		dev_err(imx8qxp_ldb->base.dev,
			"Failed to set pwr for imx8qxp_ldb aux pd dev: %d\n", err);
		return -EIO;
	}

	return 0;
}

static int ldb_pwr_enable(struct imx8qxp_ldb* imx8qxp_ldb, bool dual)
{
	if (!imx8qxp_ldb || !imx8qxp_ldb->handle) {
		WARN_ON(1);
		return -EINVAL;
	}

	sc_err_t err = sc_pm_set_resource_power_mode(imx8qxp_ldb->handle,
		(u16)imx8qxp_ldb->pd_main_dev, SC_PM_PW_MODE_ON);
	if (err != SC_ERR_NONE) {
		dev_err(imx8qxp_ldb->base.dev,
			"Failed to set pwr for imx8qxp_ldb main pd dev: %d\n", err);
		return -EIO;
	}

	if (!dual)
		return 0;

	err = sc_pm_set_resource_power_mode(imx8qxp_ldb->handle,
		(u16)imx8qxp_ldb->pd_aux_dev, SC_PM_PW_MODE_ON);
	if (err != SC_ERR_NONE) {
		dev_err(imx8qxp_ldb->base.dev,
			"Failed to set pwr for imx8qxp_ldb aux pd dev: %d\n", err);
		return -EIO;
	}

	return 0;
}

static int imx8qxp_ldb_init_sc_misc(int ldb_id, bool dual)
{
	struct _sc_ipc_struct_t *handle;
	u16 rsc;
	bool is_aux = false;
	int ret = 0;

	handle = imx_scu_get_handle();

again:
	rsc = ldb_id ? IMX_SC_R_MIPI_1 : IMX_SC_R_MIPI_0;

	ret |= sc_misc_set_control(handle,
				       rsc, IMX_SC_C_MODE, 1);
	ret |= sc_misc_set_control(handle,
				       rsc, IMX_SC_C_DUAL_MODE, is_aux);
	ret |= sc_misc_set_control(handle,
				       rsc, IMX_SC_C_PXL_LINK_SEL, is_aux);

	if (dual && !is_aux) {
		ldb_id ^= 1;
		is_aux = true;
		goto again;
	}

	return ret;
}

static int imx8qxp_ldb_bind(struct device *dev, struct device *aux_dev,
	struct phy *phy, struct phy *aux_phy)
{
	struct device_node *np = &dev->of_node;
	struct device_node *auxldb_np = NULL, *child;
	struct imx8qxp_ldb *imx8qxp_ldb = dev_get_drvdata(dev);
	struct ldb *ldb;
	struct drm_encoder *encoder[LDB_CH_NUM];
	bool dual;
	int ret;
	int i, j;

	ldb = &imx8qxp_ldb->base;
	ldb->dev = dev;
	ldb->ctrl_reg = 0xe0;
	ldb->output_port = 1;

	for (i = 0; i < LDB_CH_NUM; i++) {
		imx8qxp_ldb->channel[i].imx8qxp_ldb = imx8qxp_ldb;
		ldb->channel[i] = &imx8qxp_ldb->channel[i].base;
	}

	imx8qxp_ldb->handle = imx_scu_get_handle();
	if (!imx8qxp_ldb->handle) {
		dev_err(dev, "failed to get scu ipc handle\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(&dev->of_node, "ldb-id",
		&(u32)imx8qxp_ldb->id);
	if (ret)
		return ret;

	for (i = 0; i < LDB_CH_NUM; i++) {
		encoder[i] = &imx8qxp_ldb->channel[i].encoder;
		encoder[i]->encoder_type = DRM_MODE_ENCODER_LVDS;
	}

	dual = of_property_read_bool(np, "fsl,dual-channel");

	ret = imx8qxp_ldb_attach_pm_domains(imx8qxp_ldb, dual);
	if (ret) {
		dev_err(dev, "failed to attach pm domains %d\n", ret);
		return ret;
	}

	ldb_pwr_enable(imx8qxp_ldb, dual);

	ret = ldb_bind(ldb, encoder);
	if (ret)
		goto disable_pm_runtime;

	ret = imx8qxp_ldb_init_sc_misc(imx8qxp_ldb->id, ldb->dual);
	if (ret) {
		dev_err(dev, "failed to initialize sc misc %d\n", ret);
		goto disable_pm_runtime;
	}

	imx8qxp_ldb->clk_pixel = devm_clk_get(dev, "pixel");
	if (IS_ERR(imx8qxp_ldb->clk_pixel)) {
		ret = PTR_ERR(imx8qxp_ldb->clk_pixel);
		goto disable_pm_runtime;
	}

	imx8qxp_ldb->clk_bypass = devm_clk_get(dev, "bypass");
	if (IS_ERR(imx8qxp_ldb->clk_bypass)) {
		ret = PTR_ERR(imx8qxp_ldb->clk_bypass);
		goto disable_pm_runtime;
	}

	if (ldb->dual) {
		struct regmap_config aux_regmap_cfg;
		struct resource *aux_res;
		struct platform_device *aux_pdev = to_platform_device(aux_dev);

		imx8qxp_ldb->clk_aux_pixel = devm_clk_get(dev, "aux_pixel");
		if (IS_ERR(imx8qxp_ldb->clk_aux_pixel)) {
			ret = PTR_ERR(imx8qxp_ldb->clk_aux_pixel);
			goto disable_pm_runtime;
		}

		imx8qxp_ldb->clk_aux_bypass = devm_clk_get(dev, "aux_bypass");
		if (IS_ERR(imx8qxp_ldb->clk_aux_bypass)) {
			ret = PTR_ERR(imx8qxp_ldb->clk_aux_bypass);
			goto disable_pm_runtime;
		}

		auxldb_np = &aux_dev->of_node;
		if (!auxldb_np) {
			dev_err(dev,
				"failed to get aux LDB node\n");
			ret = -ENODEV;
			goto disable_pm_runtime;
		}

		aux_res = platform_get_resource(aux_pdev, IORESOURCE_MEM, 0);
		if (!aux_res)
			return -ENODEV;

		aux_regmap_cfg.val_bits = 32;
		imx8qxp_ldb->aux_regmap = devm_regmap_init_mmio(dev,
			aux_res, &aux_regmap_cfg);
		if (IS_ERR(imx8qxp_ldb->aux_regmap)) {
			dev_err(dev, "Failed to get aux regmap\n");
			return PTR_ERR(imx8qxp_ldb->aux_regmap);
		}

		regmap_write(imx8qxp_ldb->aux_regmap, ldb->ctrl_reg, 0);
	}

	for (j = 0; j < np->num_childs; j++) {
		struct imx8qxp_ldb_channel *imx8qxp_ldb_ch;
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

		imx8qxp_ldb_ch = &imx8qxp_ldb->channel[i];

		imx8qxp_ldb_ch->phy = phy;
		if (!imx8qxp_ldb_ch->phy) {
			dev_err(dev, "can't get channel%d phy: %d\n",
						i, ret);
			goto free_child;
		}

		ret = mixel_lvds_combo_phy_init(imx8qxp_ldb_ch->phy);
		if (ret < 0) {
			dev_err(dev, "failed to initialize channel%d phy: %d\n",
				i, ret);
			goto free_child;
		}

		if (ldb->dual) {
			imx8qxp_ldb_ch->aux_phy = aux_phy;
			if (!imx8qxp_ldb_ch->aux_phy) {
				dev_err(dev, "can't get channel0 aux phy: %d\n",
					ret);
				goto free_child;
			}

			ret = mixel_lvds_combo_phy_init(imx8qxp_ldb_ch->aux_phy);
			if (ret < 0) {
				dev_err(dev, "failed to initialize channel%d aux phy: %d\n",
					i, ret);
				goto free_child;
			}
		}
	}

	return 0;

free_child:
disable_pm_runtime:
	imx8qxp_ldb_detach_pm_domains(imx8qxp_ldb);

	return ret;
}

static void imx8qxp_ldb_unbind(struct device *dev, bool stop)
{
	struct imx8qxp_ldb *imx8qxp_ldb = dev_get_drvdata(dev);
	struct ldb *ldb = &imx8qxp_ldb->base;
	int i;

	if (stop) {
		for (i = 0; i < LDB_CH_NUM; i++) {
			struct imx8qxp_ldb_channel* imx8qxp_ldb_ch =
				&imx8qxp_ldb->channel[i];
			if (imx8qxp_ldb_ch->phy_is_on) {
				mixel_lvds_combo_phy_power_off(imx8qxp_ldb_ch->phy);
				if (ldb->dual)
					mixel_lvds_combo_phy_power_off(imx8qxp_ldb_ch->aux_phy);
			}
		}

		ldb_pwr_disable(imx8qxp_ldb, ldb->dual);
	}

	imx8qxp_ldb_detach_pm_domains(imx8qxp_ldb);
}

int imx8qxp_ldb_probe(struct platform_device *pdev,
	struct platform_device *aux_pdev, struct platform_device *phy_pdev,
	struct platform_device *aux_phy_pdev)
{
	struct device *dev = &pdev->dev;
	struct device *aux_dev = NULL;
	struct imx8qxp_ldb *imx8qxp_ldb;
	struct phy *phy, *aux_phy = NULL;

	phy = dev_get_drvdata(&phy_pdev->dev);
	if (!phy)
		return -ENOMEM;

	if (aux_pdev && aux_phy_pdev) {
		aux_dev = &aux_pdev->dev;
		aux_phy = dev_get_drvdata(&aux_phy_pdev->dev);
	}

	imx8qxp_ldb = devm_kzalloc(dev, sizeof(*imx8qxp_ldb), GFP_KERNEL);
	if (!imx8qxp_ldb)
		return -ENOMEM;

	dev_set_drvdata(dev, imx8qxp_ldb);

	return imx8qxp_ldb_bind(dev, aux_dev, phy, aux_phy);
}

int imx8qxp_ldb_remove(struct platform_device *pdev,
	struct platform_device *aux_pdev, bool stop)
{
	struct imx8qxp_ldb *imx8qxp_ldb = dev_get_drvdata(&pdev->dev);

	imx8qxp_ldb_unbind(&pdev->dev, stop);

	struct resource *res = platform_get_resource(pdev,
		IORESOURCE_MEM, 0);
	if (res && imx8qxp_ldb->base.regmap)
		regmap_release_mmio(imx8qxp_ldb->base.regmap, res);

	if (aux_pdev) {
		struct resource* aux_res = platform_get_resource(aux_pdev,
			IORESOURCE_MEM, 0);
		if (aux_res && imx8qxp_ldb->aux_regmap)
			regmap_release_mmio(imx8qxp_ldb->aux_regmap, aux_res);
	}

	if (imx8qxp_ldb)
		kfree(imx8qxp_ldb);

	return 0;
}

int imx8qxp_ldb_suspend(struct device *dev)
{
	struct imx8qxp_ldb *imx8qxp_ldb = dev_get_drvdata(dev);
	struct ldb *ldb = &imx8qxp_ldb->base;

	/* do nothing; original linux driver calls phy_exit here
	which is not supported by QXP LVDS PHY */
	return 0;
}

int imx8qxp_ldb_resume(struct device *dev)
{
	struct imx8qxp_ldb *imx8qxp_ldb = dev_get_drvdata(dev);
	struct ldb *ldb = &imx8qxp_ldb->base;
	int i;

	if (imx8qxp_ldb == NULL)
		return 0;

	for (i = 0; i < LDB_CH_NUM; i++) {
		mixel_lvds_combo_phy_init(imx8qxp_ldb->channel[i].phy);

		if (ldb->dual && i == 0)
			mixel_lvds_combo_phy_init(imx8qxp_ldb->channel[i].aux_phy);
	}

	imx8qxp_ldb_init_sc_misc(imx8qxp_ldb->id, ldb->dual);

	return 0;
}
