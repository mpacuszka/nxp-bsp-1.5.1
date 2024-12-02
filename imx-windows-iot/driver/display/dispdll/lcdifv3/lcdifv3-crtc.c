// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019,2020,2022 NXP
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <lcdifv3/imx-lcdifv3.h>
#include <video/videomode.h>
#include <asm-generic/bug.h>
#include <linux/err.h>

#include <linux/media-bus-format.h>
#include "lcdifv3/lcdifv3-plane.h"
#include <linux/delay.h>

int lcdifv3_crtc_atomic_check(struct platform_device *pdev, u32 bus_format)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB565_1X16:
	case MEDIA_BUS_FMT_RGB666_1X18:
	case MEDIA_BUS_FMT_RGB888_1X24:
		break;
	default:
		dev_err(lcdifv3_crtc->dev,
			"unsupported bus format: %#x\n",
			bus_format);
		return -EINVAL;
	}

	return 0;
}

void lcdifv3_crtc_atomic_flush(struct platform_device *pdev)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	/* kick shadow load for plane config */
	lcdifv3_en_shadow_load(lcdifv3);
}

void lcdifv3_crtc_atomic_enable(struct platform_device *pdev)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	/* Note: original driver included here a call to lcdifv3_set_mode(lcdifv3, vm); and lcdifv3_set_bus_fmt(lcdifv3, bus_format);
	   Both were moved to lcdifv3_crtc_mode_set, so that mode set can be done independently on controller enable */

	/* run LCDIFv3 */
	lcdifv3_enable_controller(lcdifv3);
}

/* mod_set function contains what was formerly in lcdifv3_crtc_atomic_enable
   in order to separate clock settings from running the controller itself */
void lcdifv3_crtc_mode_set(struct platform_device *pdev, struct videomode *vm, u32 bus_format)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	lcdifv3_set_mode(lcdifv3, vm);

	/* config LCDIF output bus format */
	lcdifv3_set_bus_fmt(lcdifv3, bus_format);
}


void lcdifv3_crtc_atomic_disable(struct platform_device *pdev)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	lcdifv3_disable_controller(lcdifv3);
	lcdifv3_reset_controller(lcdifv3);
}

int lcdifv3_enable_vblank(struct platform_device *pdev)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	lcdifv3_vblank_irq_enable(lcdifv3);
	enable_irq(lcdifv3_crtc->vbl_irq);

	return 0;
}

void lcdifv3_disable_vblank(struct platform_device *pdev)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	disable_irq(lcdifv3_crtc->vbl_irq);
	lcdifv3_vblank_irq_disable(lcdifv3);
}

bool lcdifv3_poll_vblank(struct platform_device *pdev)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	return lcdifv3_vblank_irq_poll(lcdifv3);
}

void lcdifv3_clear_vblank(struct platform_device *pdev)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	lcdifv3_vblank_irq_clear(lcdifv3);
}

static int lcdifv3_crtc_init(struct lcdifv3_crtc *lcdifv3_crtc)
{
	int ret;
	struct lcdifv3_plane *primary = lcdifv3_crtc->plane[0];
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	/* Primary plane
	 * The 'possible_crtcs' of primary plane will be
	 * recalculated during the 'crtc' initialization
	 * later.
	 */
	primary = lcdifv3_plane_init(lcdifv3, 0, DRM_PLANE_TYPE_PRIMARY, 0);
	if (IS_ERR(primary))
		return PTR_ERR(primary);
	lcdifv3_crtc->plane[0] = primary;
	/* TODO: Overlay plane */

	lcdifv3_crtc->vbl_irq = lcdifv3_vblank_irq_get(lcdifv3);
	WARN_ON(lcdifv3_crtc->vbl_irq < 0);

	ret = devm_request_irq(lcdifv3_crtc->dev, lcdifv3_crtc->vbl_irq,
		NULL, 0, "lcdif_irq", lcdifv3_crtc);
	if (ret) {
		dev_err(lcdifv3_crtc->dev,
			"vblank irq request failed: %d\n", ret);
		return ret;
	}

	disable_irq(lcdifv3_crtc->vbl_irq);

	return 0;
}


int lcdifv3_crtc_bind(struct device *dev)
{
	int ret;
	struct lcdifv3_crtc *lcdifv3_crtc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s: lcdifv3 crtc bind begin\n", __func__);

	ret = lcdifv3_crtc_init(lcdifv3_crtc);
	if (ret)
		return ret;

	dev_dbg(dev, "%s: lcdifv3 crtc bind end\n", __func__);

	return 0;
}

int lcdifv3_crtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lcdifv3_crtc *lcdifv3_crtc;

	dev_dbg(&pdev->dev, "%s: lcdifv3 crtc probe begin\n", __func__);

	lcdifv3_crtc = devm_kzalloc(dev, sizeof(*lcdifv3_crtc), GFP_KERNEL);
	if (!lcdifv3_crtc)
		return -ENOMEM;

	lcdifv3_crtc->dev = dev;
	dev_set_drvdata(dev, lcdifv3_crtc);

	return 0;
}

int lcdifv3_crtc_remove(struct platform_device *pdev)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);

	if (lcdifv3_crtc) {
		lcdifv3_plane_destroy(lcdifv3_crtc->plane[0]);
		devm_kfree(&pdev->dev, lcdifv3_crtc);
	}

	return 0;
}
