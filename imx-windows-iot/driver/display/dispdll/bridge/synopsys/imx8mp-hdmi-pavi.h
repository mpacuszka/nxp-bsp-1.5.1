// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2020,2022 NXP
 *
 * PAI/PVI Head file
 *
 */
#ifndef _IMX8MP_HDMI_AV_CTL_H_
#define _IMX8MP_HDMI_AV_CTL_H_

#include <linux/clk.h>
#include <drm/drm_modes.h>

struct imx8mp_hdmi_pavi {
	struct device *dev;

	uint8_t __iomem *base;
	uint8_t __iomem *base_blkctl;

	struct clk *clk_pai;
	struct clk *clk_pvi;
	void(*reset_pai)(volatile uint8_t __iomem *regptr, bool enable);
	void(*reset_pvi)(volatile uint8_t __iomem *regptr, bool enable);
};

void imx8mp_hdmi_pai_enable(int channel, int width, int rate, int non_pcm);
void imx8mp_hdmi_pai_disable(void);

void imx8mp_hdmi_pvi_enable(const struct drm_display_mode *mode);
void imx8mp_hdmi_pvi_disable(void);

void imx8mp_hdmi_pavi_powerup(void);
void imx8mp_hdmi_pavi_powerdown(void);

struct imx8mp_hdmi_pavi *imx8mp_hdmi_pavi_init(void);

int imx8mp_hdmi_pavi_remove(struct platform_device *pdev);
int imx8mp_hdmi_pavi_probe(struct platform_device* pdev, uint8_t __iomem* blkctl);

#endif /* _IMX8MP_HDMI_PAVI_H_ */
