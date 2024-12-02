/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019-2022 NXP
 */

#ifndef __IMX_LCDIFV3_H__
#define __IMX_LCDIFV3_H__

struct lcdifv3_soc;
struct videomode;
struct platform_device;
struct device;

struct lcdifv3_crtc {
	struct device *dev;

	struct lcdifv3_plane *plane[2];

	int vbl_irq;
	u32 pix_fmt;		/* drm fourcc */
};

/* pdev expected be "struct platform_device *" type of "lcdifv3_crtc" object */
#define to_lcdifv3_crtc(pdev) ((struct lcdifv3_crtc *)dev_get_drvdata(&((pdev)->dev)))

/* #define LCDIFV3_CRTC_DEBUG */
#ifdef LCDIFV3_CRTC_DEBUG
void imx_lcdifv3_dumpregs(struct platform_device *pdev);
#endif

/* Common */
int  lcdifv3_vblank_irq_get(struct lcdifv3_soc *lcdifv3);
void lcdifv3_vblank_irq_enable(struct lcdifv3_soc *lcdifv3);
void lcdifv3_vblank_irq_disable(struct lcdifv3_soc *lcdifv3);
void lcdifv3_vblank_irq_clear(struct lcdifv3_soc *lcdifv3);
bool lcdifv3_vblank_irq_poll(struct lcdifv3_soc *lcdifv3);

uint32_t lcdifv3_get_bpp_from_fmt(uint32_t format);
int  lcdifv3_get_bus_fmt_from_pix_fmt(struct lcdifv3_soc *lcdifv3, uint32_t format);
int  lcdifv3_set_pix_fmt(struct lcdifv3_soc *lcdifv3, u32 format);
void lcdifv3_set_bus_fmt(struct lcdifv3_soc *lcdifv3, u32 bus_format);
void lcdifv3_set_fb_addr(struct lcdifv3_soc *lcdifv3, int id, u32 addr);
void lcdifv3_set_mode(struct lcdifv3_soc *lcdifv3, struct videomode *vmode);
void lcdifv3_set_fb_hcrop(struct lcdifv3_soc *lcdifv3, u32 src_w, u32 fb_w, bool crop);
void lcdifv3_en_shadow_load(struct lcdifv3_soc *lcdifv3);
void lcdifv3_enable_controller(struct lcdifv3_soc *lcdifv3);
void lcdifv3_disable_controller(struct lcdifv3_soc *lcdifv3);
void lcdifv3_reset_controller(struct lcdifv3_soc *lcdifv3);
int imx_lcdifv3_remove(struct platform_device *pdev);
int imx_lcdifv3_probe(struct platform_device *pdev);
int imx_lcdifv3_runtime_suspend(struct device *dev);
int imx_lcdifv3_runtime_resume(struct device *dev);

/* CRTC */
int lcdifv3_crtc_probe(struct platform_device *pdev);
int lcdifv3_crtc_remove(struct platform_device *pdev);
int lcdifv3_crtc_bind(struct device *dev);
void lcdifv3_crtc_atomic_flush(struct platform_device *pdev);
void lcdifv3_crtc_mode_set(struct platform_device *pdev, struct videomode *vm, u32 bus_format);
void lcdifv3_crtc_atomic_enable(struct platform_device *pdev);
void lcdifv3_crtc_atomic_disable(struct platform_device *pdev);
int lcdifv3_crtc_atomic_check(struct platform_device *pdev, u32 bus_format);
int lcdifv3_enable_vblank(struct platform_device *pdev);
void lcdifv3_disable_vblank(struct platform_device *pdev);
bool lcdifv3_poll_vblank(struct platform_device *pdev);
void lcdifv3_clear_vblank(struct platform_device *pdev);

#endif
