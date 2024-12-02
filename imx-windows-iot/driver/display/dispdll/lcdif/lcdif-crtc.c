/*
 * Copyright 2018,2022 NXP
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

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <video/videomode.h>
#include <linux/media-bus-format.h>

#include "imx-lcdif.h"
#include "lcdif-plane.h"

int lcdif_crtc_atomic_check(struct platform_device *pdev, u32 bus_format)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(pdev);

	/* check the requested bus format can be
	 * supported by LCDIF CTRC or not
	 */
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB565_1X16:
	case MEDIA_BUS_FMT_RGB666_1X18:
	case MEDIA_BUS_FMT_RGB888_1X24:
		break;
	default:
		dev_err(lcdif_crtc->dev,
			"unsupported bus format: %#x\n",
			bus_format);
		return -EINVAL;
	}

	return 0;
}

void lcdif_crtc_atomic_enable(struct platform_device *pdev, struct videomode *vm, u32 bus_format)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(pdev);
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	lcdif_set_mode(lcdif, vm);

	/* config LCDIF output bus format */
	lcdif_set_bus_fmt(lcdif, bus_format);

	/* defer the lcdif controller enable to plane update,
	 * since until then the lcdif config is complete to
	 * enable the controller to run actually.
	 */
}

void lcdif_crtc_atomic_disable(struct platform_device *pdev)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(pdev);
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	lcdif_disable_controller(lcdif);
}

int lcdif_enable_vblank(struct platform_device *pdev)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(pdev);
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	lcdif_vblank_irq_enable(lcdif);
	enable_irq(lcdif_crtc->vbl_irq);

	return 0;
}

void lcdif_disable_vblank(struct platform_device *pdev)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(pdev);
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	disable_irq_nosync(lcdif_crtc->vbl_irq);
	lcdif_vblank_irq_disable(lcdif);
}

bool lcdif_poll_vblank(struct platform_device *pdev)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(pdev);
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	return lcdif_vblank_irq_poll(lcdif);
}

void lcdif_clear_vblank(struct platform_device *pdev)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(pdev);
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	lcdif_vblank_irq_clear(lcdif);
}

static int lcdif_crtc_init(struct lcdif_crtc *lcdif_crtc)
{
	struct lcdif_plane *primary = lcdif_crtc->plane[0];
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	/* Primary plane
	 * The 'possible_crtcs' of primary plane will be
	 * recalculated during the 'crtc' initialization
	 * later.
	 */
	primary = lcdif_plane_init(lcdif, 0, DRM_PLANE_TYPE_PRIMARY, 0);
	if (IS_ERR(primary))
		return PTR_ERR(primary);
	lcdif_crtc->plane[0] = primary;

	/* TODO: Overlay plane */

	lcdif_crtc->vbl_irq = lcdif_vblank_irq_get(lcdif);
	WARN_ON(lcdif_crtc->vbl_irq < 0);

	disable_irq(lcdif_crtc->vbl_irq);

	return 0;
}

int lcdif_crtc_bind(struct device *dev)
{
	int ret;
	struct lcdif_crtc *lcdif_crtc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s: lcdif crtc bind begin\n", __func__);

	ret = lcdif_crtc_init(lcdif_crtc);
	if (ret)
		return ret;

	dev_dbg(dev, "%s: lcdif crtc bind end\n", __func__);

	return 0;
}

void lcdif_crtc_unbind(struct device *dev)
{
	struct lcdif_crtc *lcdif_crtc = dev_get_drvdata(dev);

	if (lcdif_crtc->plane[0]) {
		lcdif_plane_deinit(lcdif_crtc->plane[0]);
	}
}

int lcdif_crtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lcdif_crtc *lcdif_crtc;

	dev_dbg(&pdev->dev, "%s: lcdif crtc probe begin\n", __func__);

	lcdif_crtc = devm_kzalloc(dev, sizeof(*lcdif_crtc), GFP_KERNEL);
	if (!lcdif_crtc)
		return -ENOMEM;

	lcdif_crtc->dev = dev;

	dev_set_drvdata(dev, lcdif_crtc);

	return 0;
}

int lcdif_crtc_remove(struct platform_device *pdev)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(pdev);

	if (lcdif_crtc) {
		devm_kfree(&pdev->dev, lcdif_crtc);
	}

	return 0;
}
