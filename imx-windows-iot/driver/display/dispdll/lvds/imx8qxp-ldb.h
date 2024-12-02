// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#ifndef __IMX8QXP_LDB__
#define __IMX8QXP_LDB__

#include <drm/bridge/fsl_imx_ldb.h>
#include <drm/drm_encoder.h>

enum {
	IMX_INTERFACE_UNKNOWN = 0,
	IMX_LVDS0,
	IMX_LVDS1,
	IMX_LVDS0_DUAL,
};

struct platform_device;
struct device;
struct videomode;

struct drm_encoder *imx8qxp_ldb_get_encoder(struct platform_device *pdev, int ch);
u32 imx8qxp_ldb_get_bus_format(struct platform_device *pdev, int ch);
void imx8qxp_ldb_encoder_enable(struct drm_encoder *encoder);
void imx8qxp_ldb_encoder_atomic_mode_set(struct drm_encoder *encoder,
	struct drm_crtc_state *crtc_state,
	struct drm_connector_state *connector_state);
void imx8qxp_ldb_encoder_disable(struct drm_encoder *encoder);
int imx8qxp_ldb_encoder_atomic_check(struct drm_encoder *encoder,
	struct drm_crtc_state *crtc_state,
	struct drm_connector_state *conn_state);
int imx8qxp_ldb_remove(struct platform_device *pdev,
	struct platform_device *aux_pdev, bool stop);
int imx8qxp_ldb_probe(struct platform_device *pdev,
	struct platform_device *aux_pdev, struct platform_device *phy_pdev,
	struct platform_device *aux_phy_pdev);
int imx8qxp_ldb_suspend(struct device *dev);
int imx8qxp_ldb_resume(struct device *dev);

#endif /* __IMX8QXP_LDB__ */
