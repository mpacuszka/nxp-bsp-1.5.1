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

#ifndef __IMX_LCDIF_H__
#define __IMX_LCDIF_H__

struct lcdif_soc;
struct videomode;
struct platform_device;
struct device;

struct lcdif_crtc {
	struct device *dev;

	struct lcdif_plane *plane[2];

	int vbl_irq;
	u32 pix_fmt;		/* drm fourcc */
};

#define LCDIF_CRTC_DEBUG

/* pdev expected be "struct platform_device *" type of "lcdifv3_crtc" object */
#define to_lcdif_crtc(pdev) ((struct lcdif_crtc *)dev_get_drvdata(&((pdev)->dev)))

/* Common */
int  lcdif_vblank_irq_get(struct lcdif_soc *lcdif);
void lcdif_vblank_irq_enable(struct lcdif_soc *lcdif);
void lcdif_vblank_irq_disable(struct lcdif_soc *lcdif);
void lcdif_vblank_irq_clear(struct lcdif_soc *lcdif);
bool lcdif_vblank_irq_poll(struct lcdif_soc *lcdif);

uint32_t lcdif_get_bpp_from_fmt(uint32_t format);
int  lcdif_get_bus_fmt_from_pix_fmt(struct lcdif_soc *lcdif,
				    uint32_t format);
int  lcdif_set_pix_fmt(struct lcdif_soc *lcdif, u32 format);
void lcdif_set_bus_fmt(struct lcdif_soc *lcdif, u32 bus_format);
void lcdif_set_fb_addr(struct lcdif_soc *lcdif, int id, u32 addr);
void lcdif_set_mode(struct lcdif_soc *lcdif, struct videomode *vmode);
void lcdif_set_fb_hcrop(struct lcdif_soc *lcdif, u32 src_w,
			u32 fb_w, bool crop);
void lcdif_enable_controller(struct lcdif_soc *lcdif);
void lcdif_disable_controller(struct lcdif_soc *lcdif);
#ifdef LCDIF_CRTC_DEBUG
void lcdif_dump_registers(struct platform_device *pdev);
#endif
long lcdif_pix_clk_round_rate(struct lcdif_soc *lcdif,
			      unsigned long rate);

int imx_lcdif_remove(struct platform_device *pdev);
int imx_lcdif_probe(struct platform_device *pdev);
int imx_lcdif_runtime_suspend(struct device *dev);
int imx_lcdif_runtime_resume(struct device *dev);

/* CRTC */
int lcdif_crtc_atomic_check(struct platform_device *pdev, u32 bus_format);
void lcdif_crtc_atomic_enable(struct platform_device *pdev, struct videomode *vm, u32 bus_format);
void lcdif_crtc_atomic_disable(struct platform_device *pdev);
int lcdif_enable_vblank(struct platform_device *pdev);
void lcdif_disable_vblank(struct platform_device *pdev);
bool lcdif_poll_vblank(struct platform_device *pdev);
void lcdif_clear_vblank(struct platform_device *pdev);
int lcdif_crtc_bind(struct device *dev);
void lcdif_crtc_unbind(struct device *dev);
int lcdif_crtc_probe(struct platform_device *pdev);
int lcdif_crtc_remove(struct platform_device *pdev);

#endif
