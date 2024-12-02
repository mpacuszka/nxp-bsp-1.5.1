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

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/media-bus-format.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <drm/drm_fourcc.h>
#include <video/videomode.h>
#include <asm-generic/bug.h>

#include "lcdif-regs.h"
#include "imx-lcdif.h"
#include "clk/displaymix_mn.h"

struct lcdif_soc {
	struct device *dev;

	int irq;
	uint8_t __iomem *base;
	uint8_t __iomem *displaymix_base;
	void(*soft_resetn)(volatile uint8_t __iomem *regptr, bool enable);
	void(*clk_enable)(volatile uint8_t __iomem *regptr, bool enable);
	atomic_t rpm_suspended;

	struct clk *clk_pix;
	struct clk *clk_disp_axi;
	struct clk *clk_disp_apb;
};

struct lcdif_soc_pdata {
	bool hsync_invert;
	bool vsync_invert;
	bool de_invert;
};

struct lcdif_soc_pdata imx8mm_pdata = {
	.hsync_invert = true,
	.vsync_invert = true,
	.de_invert    = true,
};

static const struct of_device_id imx_lcdif_dt_ids[] = {
	{ .compatible = "fsl,imx8mm-lcdif", .data = &imx8mm_pdata, },
	{ .compatible = "fsl,imx8mn-lcdif", .data = &imx8mm_pdata, },
	{ .compatible = "", /* sentinel */ }
};

static int lcdif_rstc_reset(
	struct lcdif_soc *lcdif,
	void(*rstc)(volatile uint8_t __iomem *regptr, bool enable),
	bool assert)
{
	if (!rstc || !lcdif)
		return 0;

	rstc((volatile uint8_t __iomem *)lcdif->displaymix_base, assert);

	return 0;
}

static int lcdif_enable_clocks(struct lcdif_soc *lcdif)
{
	int ret;

	if (lcdif->clk_disp_axi) {
		ret = clk_prepare_enable(lcdif->clk_disp_axi);
		if (ret)
			return ret;
	}

	if (lcdif->clk_disp_apb) {
		ret = clk_prepare_enable(lcdif->clk_disp_apb);
		if (ret)
			goto disable_disp_axi;
	}

	ret = clk_prepare_enable(lcdif->clk_pix);
	if (ret)
		goto disable_disp_apb;

	return 0;

disable_disp_apb:
	if (lcdif->clk_disp_apb)
		clk_disable_unprepare(lcdif->clk_disp_apb);
disable_disp_axi:
	if (lcdif->clk_disp_axi)
		clk_disable_unprepare(lcdif->clk_disp_axi);

	return ret;
}

static void lcdif_disable_clocks(struct lcdif_soc *lcdif)
{
	clk_disable_unprepare(lcdif->clk_pix);

	if (lcdif->clk_disp_axi)
		clk_disable_unprepare(lcdif->clk_disp_axi);

	if (lcdif->clk_disp_apb)
		clk_disable_unprepare(lcdif->clk_disp_apb);
}

int lcdif_vblank_irq_get(struct lcdif_soc *lcdif)
{
	return lcdif->irq;
}

#ifdef LCDIF_CRTC_DEBUG
void lcdif_dump_registers(struct platform_device *pdev)
{
	struct lcdif_soc *lcdif = dev_get_drvdata(&pdev->dev);

	pr_err("%s	: %#x\n", "LCDIF_CTRL",
				  readl(lcdif->base + LCDIF_CTRL));
	pr_err("%s	: %#x\n", "LCDIF_CTRL1",
				  readl(lcdif->base + LCDIF_CTRL1));
	pr_err("%s	: %#x\n", "LCDIF_CTRL2",
				  readl(lcdif->base + LCDIF_CTRL2));
	pr_err("%s	: %#x\n", "LCDIF_TRANSFER_COUNT",
				  readl(lcdif->base + LCDIF_TRANSFER_COUNT));
	pr_err("%s	: %#x\n", "LCDIF_CUR_BUF",
				  readl(lcdif->base + LCDIF_CUR_BUF));
	pr_err("%s	: %#x\n", "LCDIF_NEXT_BUF",
				  readl(lcdif->base + LCDIF_NEXT_BUF));
	pr_err("%s	: %#x\n", "LCDIF_VDCTRL0",
				  readl(lcdif->base + LCDIF_VDCTRL0));
	pr_err("%s	: %#x\n", "LCDIF_VDCTRL1",
				  readl(lcdif->base + LCDIF_VDCTRL1));
	pr_err("%s	: %#x\n", "LCDIF_VDCTRL2",
				  readl(lcdif->base + LCDIF_VDCTRL2));
	pr_err("%s	: %#x\n", "LCDIF_VDCTRL3",
				  readl(lcdif->base + LCDIF_VDCTRL3));
	pr_err("%s	: %#x\n", "LCDIF_VDCTRL4",
				  readl(lcdif->base + LCDIF_VDCTRL4));
}
#endif

void lcdif_vblank_irq_enable(struct lcdif_soc *lcdif)
{
	writel(CTRL1_CUR_FRAME_DONE_IRQ, lcdif->base + LCDIF_CTRL1 + REG_CLR);
	writel(CTRL1_CUR_FRAME_DONE_IRQ_EN, lcdif->base + LCDIF_CTRL1 + REG_SET);
}

void lcdif_vblank_irq_disable(struct lcdif_soc *lcdif)
{
	writel(CTRL1_CUR_FRAME_DONE_IRQ_EN, lcdif->base + LCDIF_CTRL1 + REG_CLR);
	writel(CTRL1_CUR_FRAME_DONE_IRQ, lcdif->base + LCDIF_CTRL1 + REG_CLR);
}

void lcdif_vblank_irq_clear(struct lcdif_soc *lcdif)
{
	writel(CTRL1_CUR_FRAME_DONE_IRQ, lcdif->base + LCDIF_CTRL1 + REG_CLR);
}

bool lcdif_vblank_irq_poll(struct lcdif_soc *lcdif)
{
	uint32_t int_status;

	int_status = readl(lcdif->base + LCDIF_CTRL1);
	if ((int_status & CTRL1_CUR_FRAME_DONE_IRQ) && (int_status & CTRL1_CUR_FRAME_DONE_IRQ_EN))
		return true;
	else
		return false;
}

uint32_t lcdif_get_bpp_from_fmt(uint32_t format)
{
	/* TODO: only support RGB for now */

	switch (format) {
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_XBGR1555:
		return 16;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
		return 32;
	default:
		/* unsupported format */
		return 0;
	}
}

/*
 * Get the bus format supported by LCDIF
 * according to drm fourcc format
 */
int lcdif_get_bus_fmt_from_pix_fmt(struct lcdif_soc *lcdif,
				   uint32_t format)
{
	uint32_t bpp;

	bpp = lcdif_get_bpp_from_fmt(format);
	if (!bpp)
		return -EINVAL;

	switch (bpp) {
	case 16:
		return MEDIA_BUS_FMT_RGB565_1X16;
	case 18:
		return MEDIA_BUS_FMT_RGB666_1X18;
	case 24:
	case 32:
		return MEDIA_BUS_FMT_RGB888_1X24;
	default:
		return -EINVAL;
	}
}

int lcdif_set_pix_fmt(struct lcdif_soc *lcdif, u32 format)
{
	struct drm_format_name_buf format_name;
	u32 ctrl = 0, ctrl1 = 0;

	/* TODO: lcdif should be disabled to set pixel format */

	ctrl  = readl(lcdif->base + LCDIF_CTRL);
	ctrl1 = readl(lcdif->base + LCDIF_CTRL1);

	/* clear pixel format related bits */
	ctrl  &= ~(CTRL_SHIFT_NUM(0x3f)  | CTRL_INPUT_SWIZZLE(0x3) |
		   CTRL_CSC_SWIZZLE(0x3) | CTRL_SET_WORD_LENGTH(0x3));

	ctrl1 &= ~CTRL1_SET_BYTE_PACKAGING(0xf);

	/* default is 'RGB' order */
	writel(CTRL2_ODD_LINE_PATTERN(0x7) |
	       CTRL2_EVEN_LINE_PATTERN(0x7),
	       lcdif->base + LCDIF_CTRL2 + REG_CLR);

	switch (format) {
		/* bpp 16 */
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_XBGR1555:
		/* Data format */
		ctrl = (format == DRM_FORMAT_RGB565 ||
			format == DRM_FORMAT_BGR565) ?
			(ctrl & ~CTRL_DF16) : (ctrl | CTRL_DF16);

		ctrl |= CTRL_SET_WORD_LENGTH(0x0);

		/* Byte packing */
		ctrl1 |= CTRL1_SET_BYTE_PACKAGING(0xf);

		/* 'BGR' order */
		if (format == DRM_FORMAT_BGR565		||
		    format == DRM_FORMAT_ABGR1555	||
		    format == DRM_FORMAT_XBGR1555)
			writel(CTRL2_ODD_LINE_PATTERN(0x5) |
			       CTRL2_EVEN_LINE_PATTERN(0x5),
			       lcdif->base + LCDIF_CTRL2 + REG_SET);
		break;
		/* bpp 32 */
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
		/*Data format */
		ctrl &= ~CTRL_DF24;
		ctrl |= CTRL_SET_WORD_LENGTH(3);

		if (format == DRM_FORMAT_RGBA8888 ||
		    format == DRM_FORMAT_RGBX8888)
			ctrl |= CTRL_SHIFT_DIR(1) | CTRL_SHIFT_NUM(8);

		/* Byte packing */
		ctrl1 |= CTRL1_SET_BYTE_PACKAGING(0x7);

		/* 'BGR' order */
		if (format == DRM_FORMAT_ABGR8888 ||
		    format == DRM_FORMAT_XBGR8888)
			writel(CTRL2_ODD_LINE_PATTERN(0x5) |
			       CTRL2_EVEN_LINE_PATTERN(0x5),
			       lcdif->base + LCDIF_CTRL2 + REG_SET);
		break;
	default:
		dev_err(lcdif->dev, "unsupported pixel format: %s\n",
			drm_get_format_name(format, &format_name));
		return -EINVAL;
	}

	writel(ctrl,  lcdif->base + LCDIF_CTRL);
	writel(ctrl1, lcdif->base + LCDIF_CTRL1);

	return 0;
}

void lcdif_set_bus_fmt(struct lcdif_soc *lcdif, u32 bus_format)
{
	u32 bus_width;

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		bus_width = CTRL_SET_BUS_WIDTH(STMLCDIF_16BIT);
		break;
	case MEDIA_BUS_FMT_RGB666_1X18:
		bus_width = CTRL_SET_BUS_WIDTH(STMLCDIF_18BIT);
		break;
	case MEDIA_BUS_FMT_RGB888_1X24:
		bus_width = CTRL_SET_BUS_WIDTH(STMLCDIF_24BIT);
		break;
	default:
		dev_err(lcdif->dev, "unknown bus format: %#x\n", bus_format);
		return;
	}

	writel(CTRL_SET_BUS_WIDTH(0x3), lcdif->base + LCDIF_CTRL + REG_CLR);
	writel(bus_width, lcdif->base + LCDIF_CTRL + REG_SET);
}

void lcdif_set_fb_addr(struct lcdif_soc *lcdif, int id, u32 addr)
{
	switch (id) {
	case 0:
		/* primary plane */
		writel(addr, lcdif->base + LCDIF_NEXT_BUF);
		break;
	default:
		/* TODO: add overlay support */
		return;
	}
}

void lcdif_set_fb_hcrop(struct lcdif_soc *lcdif, u32 src_w,
			u32 fb_w, bool crop)
{
	u32 mask_cnt, htotal, hcount;
	u32 vdctrl2, vdctrl3, vdctrl4, transfer_count;
	u32 pigeon_12_0, pigeon_12_1, pigeon_12_2;

	if (!crop) {
		writel(0x0, lcdif->base + HW_EPDC_PIGEON_12_0);
		writel(0x0, lcdif->base + HW_EPDC_PIGEON_12_1);

		return;
	}

	/* transfer_count's hcount, vdctrl2's htotal and vdctrl4's
	 * H_VALID_DATA_CNT should use fb width instead of hactive
	 * when requires cropping.
	 * */
	transfer_count = readl(lcdif->base + LCDIF_TRANSFER_COUNT);
	hcount = TRANSFER_COUNT_GET_HCOUNT(transfer_count);

	transfer_count &= ~TRANSFER_COUNT_SET_HCOUNT(0xffff);
	transfer_count |= TRANSFER_COUNT_SET_HCOUNT(fb_w);
	writel(transfer_count, lcdif->base + LCDIF_TRANSFER_COUNT);

	vdctrl2 = readl(lcdif->base + LCDIF_VDCTRL2);
	htotal  = VDCTRL2_GET_HSYNC_PERIOD(vdctrl2);
	htotal  += fb_w - hcount;
	vdctrl2 &= ~VDCTRL2_SET_HSYNC_PERIOD(0x3ffff);
	vdctrl2 |= VDCTRL2_SET_HSYNC_PERIOD(htotal);
	writel(vdctrl2, lcdif->base + LCDIF_VDCTRL2);

	vdctrl4 = readl(lcdif->base + LCDIF_VDCTRL4);
	vdctrl4 &= ~SET_DOTCLK_H_VALID_DATA_CNT(0x3ffff);
	vdctrl4 |= SET_DOTCLK_H_VALID_DATA_CNT(fb_w);
	writel(vdctrl4, lcdif->base + LCDIF_VDCTRL4);

	/* configure related pigeon registers */
	vdctrl3  = readl(lcdif->base + LCDIF_VDCTRL3);
	mask_cnt = GET_HOR_WAIT_CNT(vdctrl3) - 5;

	pigeon_12_0 = PIGEON_12_0_SET_STATE_MASK(0x24)		|
		      PIGEON_12_0_SET_MASK_CNT(mask_cnt)	|
		      PIGEON_12_0_SET_MASK_CNT_SEL(0x6)		|
		      PIGEON_12_0_POL_ACTIVE_LOW		|
		      PIGEON_12_0_EN;
	writel(pigeon_12_0, lcdif->base + HW_EPDC_PIGEON_12_0);

	pigeon_12_1 = PIGEON_12_1_SET_CLR_CNT(src_w) |
		      PIGEON_12_1_SET_SET_CNT(0x0);
	writel(pigeon_12_1, lcdif->base + HW_EPDC_PIGEON_12_1);

	pigeon_12_2 = 0x0;
	writel(pigeon_12_2, lcdif->base + HW_EPDC_PIGEON_12_2);
}

void lcdif_set_mode(struct lcdif_soc *lcdif, struct videomode *vmode)
{
	const struct of_device_id *of_id =
			of_match_device(imx_lcdif_dt_ids, lcdif->dev);
	const struct lcdif_soc_pdata *soc_pdata;
	u32 vdctrl0, vdctrl1, vdctrl2, vdctrl3, vdctrl4, htotal;

	if (unlikely(!of_id))
		return;
	soc_pdata = of_id->data;

	/* Clear the FIFO */
	writel(CTRL1_FIFO_CLEAR, lcdif->base + LCDIF_CTRL1 + REG_SET);
	writel(CTRL1_FIFO_CLEAR, lcdif->base + LCDIF_CTRL1 + REG_CLR);

	/* set pixel clock rate */
	clk_disable_unprepare(lcdif->clk_pix);
	clk_set_rate(lcdif->clk_pix, vmode->pixelclock);
	clk_prepare_enable(lcdif->clk_pix);

	/* config display timings */
	writel(TRANSFER_COUNT_SET_VCOUNT(vmode->vactive) |
	       TRANSFER_COUNT_SET_HCOUNT(vmode->hactive),
	       lcdif->base + LCDIF_TRANSFER_COUNT);

	vdctrl0 = VDCTRL0_ENABLE_PRESENT		|
		  VDCTRL0_VSYNC_PERIOD_UNIT 		|
		  VDCTRL0_VSYNC_PULSE_WIDTH_UNIT	|
		  VDCTRL0_SET_VSYNC_PULSE_WIDTH(vmode->vsync_len);

	/* Polarities */
	if (soc_pdata) {
		if ((soc_pdata->hsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_HSYNC_LOW) ||
		    (!soc_pdata->hsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_HSYNC_HIGH))
				vdctrl0 |= VDCTRL0_HSYNC_ACT_HIGH;

		if ((soc_pdata->vsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_VSYNC_LOW) ||
		    (!soc_pdata->vsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_VSYNC_HIGH))
				vdctrl0 |= VDCTRL0_VSYNC_ACT_HIGH;

		if ((soc_pdata->de_invert &&
		     vmode->flags & DISPLAY_FLAGS_DE_LOW) ||
		    (!soc_pdata->de_invert &&
		     vmode->flags & DISPLAY_FLAGS_DE_HIGH))
				vdctrl0 |= VDCTRL0_ENABLE_ACT_HIGH;
	} else {
		if (vmode->flags & DISPLAY_FLAGS_HSYNC_HIGH)
			vdctrl0 |= VDCTRL0_HSYNC_ACT_HIGH;
		if (vmode->flags & DISPLAY_FLAGS_VSYNC_HIGH)
			vdctrl0 |= VDCTRL0_VSYNC_ACT_HIGH;
		if (vmode->flags & DISPLAY_FLAGS_DE_HIGH)
			vdctrl0 |= VDCTRL0_ENABLE_ACT_HIGH;
	}

	if (vmode->flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		vdctrl0 |= VDCTRL0_DOTCLK_ACT_FALLING;

	writel(vdctrl0, lcdif->base + LCDIF_VDCTRL0);

	vdctrl1 = vmode->vactive + vmode->vsync_len +
		  vmode->vfront_porch + vmode->vback_porch;
	writel(vdctrl1, lcdif->base + LCDIF_VDCTRL1);

	htotal = vmode->hactive + vmode->hsync_len +
		 vmode->hfront_porch + vmode->hback_porch;
	vdctrl2 = VDCTRL2_SET_HSYNC_PULSE_WIDTH(vmode->hsync_len) |
		  VDCTRL2_SET_HSYNC_PERIOD(htotal);
	writel(vdctrl2, lcdif->base + LCDIF_VDCTRL2);

	vdctrl3 = SET_HOR_WAIT_CNT(vmode->hsync_len + vmode->hback_porch) |
		  SET_VERT_WAIT_CNT(vmode->vsync_len + vmode->vback_porch);
	writel(vdctrl3, lcdif->base + LCDIF_VDCTRL3);

	vdctrl4 = SET_DOTCLK_H_VALID_DATA_CNT(vmode->hactive);
	writel(vdctrl4, lcdif->base + LCDIF_VDCTRL4);
}

void lcdif_enable_controller(struct lcdif_soc *lcdif)
{
	u32 ctrl2, vdctrl4;

	ctrl2	= readl(lcdif->base + LCDIF_CTRL2);
	vdctrl4 = readl(lcdif->base + LCDIF_VDCTRL4);

	ctrl2 &= ~CTRL2_OUTSTANDING_REQS(0x7);
	ctrl2 |= CTRL2_OUTSTANDING_REQS(REQ_16);
	writel(ctrl2, lcdif->base + LCDIF_CTRL2);

	/* Continous dotclock mode */
	writel(CTRL_BYPASS_COUNT | CTRL_DOTCLK_MODE,
	       lcdif->base + LCDIF_CTRL + REG_SET);

	/* enable the SYNC signals first, then the DMA engine */
	vdctrl4 |= VDCTRL4_SYNC_SIGNALS_ON;
	writel(vdctrl4, lcdif->base + LCDIF_VDCTRL4);

	/* enable underflow recovery */
	writel(CTRL1_RECOVERY_ON_UNDERFLOW,
	       lcdif->base + LCDIF_CTRL1 + REG_SET);

	/* run lcdif */
	writel(CTRL_MASTER, lcdif->base + LCDIF_CTRL + REG_SET);
	writel(CTRL_RUN, lcdif->base + LCDIF_CTRL + REG_SET);
}

void lcdif_disable_controller(struct lcdif_soc *lcdif)
{
	int ret;
	u32 vdctrl4;

	writel(CTRL_RUN, lcdif->base + LCDIF_CTRL + REG_CLR);
	writel(CTRL_DOTCLK_MODE, lcdif->base + LCDIF_CTRL + REG_CLR);
	/* Wait until CTRL_RUN is cleared */
	ret = readl_poll_timeout((u32 *)(lcdif->base + LCDIF_CTRL), CTRL_RUN,
		0, false, 0, 1000);
	WARN_ON(ret);
	if (ret)
		dev_err(lcdif->dev, "disable lcdif run timeout\n");

	writel(CTRL_MASTER, lcdif->base + LCDIF_CTRL + REG_CLR);

	vdctrl4 = readl(lcdif->base + LCDIF_VDCTRL4);
	vdctrl4 &= ~VDCTRL4_SYNC_SIGNALS_ON;
	writel(vdctrl4, lcdif->base + LCDIF_VDCTRL4);
}

static int lcdif_of_parse_resets(struct lcdif_soc *lcdif)
{
	struct device *dev = lcdif->dev;
	struct device_node *np = &dev->of_node;
	const char *compat;
	size_t len;

	compat = of_get_property(np, "compatible", NULL);
	if (!compat)
		return -ENODEV;

	len = strlen(compat);
	if (!of_compat_cmp("fsl,imx8mn-lcdif", compat, len)) {
		lcdif->soft_resetn = &displaymix_mn_lcdif_soft_resetn;
		lcdif->clk_enable = &displaymix_mn_lcdif_clk_enable;
	}
	else {
		dev_warn(dev, "unsupported derivative: %s\n", compat);
		return -ENODEV;
	}
	return 0;
}

int imx_lcdif_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct lcdif_soc *lcdif;
	struct resource *res;

	dev_dbg(dev, "%s: probe begin\n", __func__);

	lcdif = devm_kzalloc(dev, sizeof(*lcdif), GFP_KERNEL);
	if (!lcdif) {
		dev_err(dev, "Can't allocate 'lcdif_soc' structure\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	lcdif->irq = platform_get_irq(pdev, 0);
	if (lcdif->irq < 0)
		return -ENODEV;

	lcdif->clk_pix = devm_clk_get(dev, "pix");
	if (IS_ERR(lcdif->clk_pix))
		return PTR_ERR(lcdif->clk_pix);

	lcdif->clk_disp_axi = devm_clk_get(dev, "disp-axi");
	if (IS_ERR(lcdif->clk_disp_axi))
		lcdif->clk_disp_axi = NULL;

	lcdif->clk_disp_apb = devm_clk_get(dev, "disp-apb");
	if (IS_ERR(lcdif->clk_disp_apb))
		lcdif->clk_disp_apb = NULL;

	lcdif->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(lcdif->base))
		return PTR_ERR(lcdif->base);

	lcdif->displaymix_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(lcdif->displaymix_base))
		return PTR_ERR(lcdif->displaymix_base);

	lcdif->dev = dev;
	ret = lcdif_of_parse_resets(lcdif);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, lcdif);

	atomic_set(&lcdif->rpm_suspended, 0);
	atomic_inc(&lcdif->rpm_suspended);

	dev_dbg(dev, "%s: probe end\n", __func__);

	return 0;
}

int imx_lcdif_remove(struct platform_device *pdev)
{
	struct lcdif_soc *lcdif = dev_get_drvdata(&pdev->dev);
	struct resource *res;

	if (lcdif) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (res && lcdif->base) {
			iounmap(lcdif->base, resource_size(res));
		}
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (res && lcdif->displaymix_base) {
			iounmap(lcdif->displaymix_base, resource_size(res));
		}
		devm_kfree(&pdev->dev, lcdif);
	}

	return 0;
}

int imx_lcdif_runtime_suspend(struct device *dev)
{
	struct lcdif_soc *lcdif = dev_get_drvdata(dev);

	if (atomic_inc_return(&lcdif->rpm_suspended) > 1)
		return 0;

	lcdif_disable_clocks(lcdif);

	return 0;
}

int imx_lcdif_runtime_resume(struct device *dev)
{
	int ret = 0;
	u32 ctrl = 0;
	struct lcdif_soc *lcdif = dev_get_drvdata(dev);

	if (unlikely(!atomic_read(&lcdif->rpm_suspended))) {
		dev_warn(lcdif->dev, "Unbalanced %s!\n", __func__);
		return 0;
	}

	if (!atomic_dec_and_test(&lcdif->rpm_suspended))
		return 0;

	ret = lcdif_enable_clocks(lcdif);
	if (ret) {
		return ret;
	}

	ret = lcdif_rstc_reset(lcdif, lcdif->soft_resetn, false);
	if (ret) {
		dev_err(dev, "deassert soft_resetn failed\n");
		return ret;
	}

	ret = lcdif_rstc_reset(lcdif, lcdif->clk_enable, true);
	if (ret) {
		dev_err(dev, "assert clk_enable failed\n");
		return ret;
	}

	/* Pull LCDIF out of reset */
	/* Originally, whole register was just written zero, which disrupts the controller settings if running from boot */
	ctrl = readl(lcdif->base + LCDIF_CTRL);
	ctrl &= ~(CTRL_SFTRST | CTRL_CLKGATE);
	writel(ctrl, lcdif->base + LCDIF_CTRL);

	return ret;
}
