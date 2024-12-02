// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022,2024 NXP
 */

#ifndef __IMX8MP_LDB__
#define __IMX8MP_LDB__

#define LDB_CH_NUM	2 

 /* Display interfaces */
typedef enum {
	imx_lvds_interface_unknown = 0,
	imx_lvds0,
	imx_lvds1,
	imx_lvds0_dual,
} imx_lvds_display_interface;

struct platform_device;
struct device;
struct videomode;

int imx8mp_lvds_phy_probe(struct platform_device *pdev);
int imx8mp_lvds_phy_remove(struct platform_device *pdev);
int imx8mp_lvds_phy_power_off(struct device *dev, unsigned int chan);
int imx8mp_lvds_phy_power_on(struct device *dev, unsigned int chan);
int imx8mp_lvds_phy_init(struct device *dev, unsigned int chan);
int imx8mp_lvds_phy_exit(struct device *dev, unsigned int chan);

int imx8mp_ldb_remove(struct platform_device *pdev);
int imx8mp_ldb_probe(struct platform_device *pdev);
int imx8mp_ldb_bind(struct device *dev, unsigned int bus_data_width, const char * const bus_mapping, imx_lvds_display_interface interface);
void imx8mp_ldb_unbind(struct device *dev);
int imx8mp_ldb_suspend(struct device *dev);
int imx8mp_ldb_resume(struct device *dev);
int imx8mp_ldb_encoder_get_crtc_format(struct platform_device *pdev);
void imx8mp_ldb_encoder_disable(struct platform_device *pdev);
void imx8mp_ldb_encoder_enable(struct platform_device *pdev);
void imx8mp_ldb_encoder_atomic_mode_set(struct platform_device *pdev, struct videomode *vm);
unsigned int imx8mp_ldb_encoder_get_ldb_bus_format(struct platform_device *pdev);

/* #define IMX8MP_LDB_DEBUG */
#ifdef IMX8MP_LDB_DEBUG
void imx8mp_ldb_dumpregs(struct platform_device *pdev);
#endif
#endif /* __IMX8MP_LDB__ */
