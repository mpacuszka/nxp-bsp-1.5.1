// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2020,2022-2023 NXP
 *
 * Programe Video/Audio Interface between LCDIF and HDMI Ctrl in HDMIMIX
 *
 */

#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <drm/drm_fourcc.h>

#include "imx8mp-hdmi-pavi.h"
#include "clk/mediamix_mp.h"

#define HTX_PVI_CTRL         0x0
#define HTX_PVI_IRQ_MASK     0x04
#define HTX_TMG_GEN_DISP_LRC 0x10
#define HTX_TMG_GEN_DE_ULC   0x14
#define HTX_TMG_GEN_DE_LRC   0x18
#define HTX_TMG_GEN_HSYNC    0x1c
#define HTX_TMG_GEN_VSYNC    0x20
#define HTX_TMG_GEN_IRQ0     0x24
#define HTX_TMG_GEN_IRQ1     0x28
#define HTX_TMG_GEN_IRQ2     0x2c
#define HTX_TMG_GEN_IRQ3     0x30
#define HTX_TMG_GEN_CFG      0x40

#define HTX_PAI_CTRL        0x800
#define HTX_PAI_CTRL_EXT    0x804
#define HTX_PAI_FIELD_CTRL  0x808
#define HTX_PAI_STAT        0x80C
#define HTX_PAI_IRQ_NOMASK  0x810
#define HTX_PAI_IRQ_MASKED  0x814
#define HTX_PAI_IRQ_MASK    0x818

#define HTX_PAI_CTRL_ENABLE 1

#define HTX_PAI_IRQ_FLG_OVF 1
#define HTX_PAI_IRQ_FLG_UND 2


static struct imx8mp_hdmi_pavi *gpavi;

/* PAI APIs  */
void imx8mp_hdmi_pai_enable(int channel, int width, int rate, int non_pcm)
{
	/* PAI set */
	writel((0x3030000 | ((channel-1) << 8)),
			gpavi->base + HTX_PAI_CTRL_EXT);

	/* hbr */
	if (non_pcm && width == 32 && channel == 8 && rate == 192000)
		writel(0x004e77df, gpavi->base + HTX_PAI_FIELD_CTRL);
	else if (width == 32)
		writel(0x1c8c675b, gpavi->base + HTX_PAI_FIELD_CTRL);
	else
		writel(0x1c0c675b, gpavi->base + HTX_PAI_FIELD_CTRL);

	/* PAI start running */
	writel(HTX_PAI_CTRL_ENABLE, gpavi->base + HTX_PAI_CTRL);
}

void imx8mp_hdmi_pai_disable(void)
{
	/* stop PAI */
	writel(0, gpavi->base + HTX_PAI_CTRL);
}

/* PVI APIs  */
void imx8mp_hdmi_pvi_enable(const struct drm_display_mode *mode)
{
	writel(0x00000003, gpavi->base + HTX_PVI_IRQ_MASK);
	writel(0x08970464, gpavi->base + HTX_TMG_GEN_DISP_LRC);
	writel(0x00bf0029, gpavi->base + HTX_TMG_GEN_DE_ULC);
	writel(0x083f0460, gpavi->base + HTX_TMG_GEN_DE_LRC);
	writel(0x0897002b, gpavi->base + HTX_TMG_GEN_HSYNC);
	writel(0x04640004, gpavi->base + HTX_TMG_GEN_VSYNC);
	writel(0x000100ff, gpavi->base + HTX_TMG_GEN_IRQ0);
	writel(0x000100f0, gpavi->base + HTX_TMG_GEN_IRQ1);
	writel(0x00010315, gpavi->base + HTX_TMG_GEN_IRQ2);
	writel(0x00010207, gpavi->base + HTX_TMG_GEN_IRQ3);
	writel(0x84640000, gpavi->base + HTX_TMG_GEN_CFG);

	/* DE/VSYN/HSYNC pol */
	if ((mode->flags & DRM_MODE_FLAG_PVSYNC) &&
			(mode->flags & DRM_MODE_FLAG_PHSYNC)) {
		writel(0x00377004, gpavi->base + HTX_PVI_CTRL);
		writel(0x00377005, gpavi->base + HTX_PVI_CTRL);
	} else {
		writel(0x00311004, gpavi->base + HTX_PVI_CTRL);
		writel(0x00311005, gpavi->base + HTX_PVI_CTRL);
	}
}

void imx8mp_hdmi_pvi_disable(void)
{
	/* Stop PVI */
	writel(0x0, gpavi->base + HTX_PVI_CTRL);
}

void imx8mp_hdmi_pavi_powerup(void)
{
	clk_prepare_enable(gpavi->clk_pvi);
	clk_prepare_enable(gpavi->clk_pai);

	/* deassert pai reset */
	if (gpavi->reset_pai)
		gpavi->reset_pai(gpavi->base_blkctl, false);

	/* deassert pvi reset */
	if (gpavi->reset_pvi)
		gpavi->reset_pvi(gpavi->base_blkctl, false);
}

void imx8mp_hdmi_pavi_powerdown(void)
{
	/* set pvi reset */
	if (gpavi->reset_pvi)
		gpavi->reset_pvi(gpavi->base_blkctl, true);

	/* set pai reset */
	if (gpavi->reset_pai)
		gpavi->reset_pai(gpavi->base_blkctl, true);

	clk_disable_unprepare(gpavi->clk_pai);
	clk_disable_unprepare(gpavi->clk_pvi);
}

struct imx8mp_hdmi_pavi *imx8mp_hdmi_pavi_init(void)
{
	return gpavi;
}

int imx8mp_hdmi_pavi_probe(struct platform_device *pdev, uint8_t __iomem* blkctl)
{
	struct device *dev = &pdev->dev;
	struct imx8mp_hdmi_pavi *pavi;
	struct resource *res;

	dev_dbg(dev, "%s: probe begin\n", __func__);

	pavi = devm_kzalloc(dev, sizeof(*pavi), GFP_KERNEL);
	if (!pavi) {
		dev_err(dev, "Can't allocate 'imx8mp pavi' structure\n");
		return -ENOMEM;
	}
	pavi->base_blkctl = blkctl;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res)
		return -ENODEV;

	pavi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pavi->base))
		return PTR_ERR(pavi->base);

	gpavi = pavi;

	pavi->clk_pvi = devm_clk_get(dev, "pvi_clk");
	if (IS_ERR(pavi->clk_pvi)) {
		dev_err(dev, "No pvi clock get\n");
		return -ENODEV;
	}

	pavi->clk_pai = devm_clk_get(dev, "pai_clk");
	if (IS_ERR(pavi->clk_pai)) {
		dev_err(dev, "No pai clock get\n");
		return -ENODEV;
	}

	pavi->reset_pai = &hdmimix_pai_reset;
	pavi->reset_pvi = &hdmimix_pvi_reset;

	dev_dbg(dev, "%s: probe success\n", __func__);
	return 0;
}

int imx8mp_hdmi_pavi_remove(struct platform_device *pdev)
{
	struct resource* res;

	if (gpavi) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (res && gpavi->base) {
			iounmap(gpavi->base, resource_size(res));
		}
		devm_kfree(&pdev->dev, gpavi);
		gpavi = NULL;
	}
	return 0;
}

#define debug_print(x, y) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, readl(gpavi->base + y));
void imx8mp_hdmi_pavi_dumpreg(void)
{
	debug_print("HTX_PVI_CTRL         = 0x%08X\n", HTX_PVI_CTRL);
	debug_print("HTX_PVI_IRQ_MASK     = 0x%08X\n", HTX_PVI_IRQ_MASK);
	debug_print("HTX_TMG_GEN_DISP_LRC = 0x%08X\n", HTX_TMG_GEN_DISP_LRC);
	debug_print("HTX_TMG_GEN_DE_ULC   = 0x%08X\n", HTX_TMG_GEN_DE_ULC);
	debug_print("HTX_TMG_GEN_DE_LRC   = 0x%08X\n", HTX_TMG_GEN_DE_LRC);
	debug_print("HTX_TMG_GEN_HSYNC    = 0x%08X\n", HTX_TMG_GEN_HSYNC);
	debug_print("HTX_TMG_GEN_VSYNC    = 0x%08X\n", HTX_TMG_GEN_VSYNC);
	debug_print("HTX_TMG_GEN_IRQ0     = 0x%08X\n", HTX_TMG_GEN_IRQ0);
	debug_print("HTX_TMG_GEN_IRQ1     = 0x%08X\n", HTX_TMG_GEN_IRQ1);
	debug_print("HTX_TMG_GEN_IRQ2     = 0x%08X\n", HTX_TMG_GEN_IRQ2);
	debug_print("HTX_TMG_GEN_IRQ3     = 0x%08X\n", HTX_TMG_GEN_IRQ3);
	debug_print("HTX_TMG_GEN_CFG      = 0x%08X\n", HTX_TMG_GEN_CFG);
	debug_print("HTX_PAI_CTRL         = 0x%08X\n", HTX_PAI_CTRL);
	debug_print("HTX_PAI_CTRL_EXT     = 0x%08X\n", HTX_PAI_CTRL_EXT);
	debug_print("HTX_PAI_FIELD_CTRL   = 0x%08X\n", HTX_PAI_FIELD_CTRL);
	debug_print("HTX_PAI_STAT         = 0x%08X\n", HTX_PAI_STAT);
	debug_print("HTX_PAI_IRQ_NOMASK   = 0x%08X\n", HTX_PAI_IRQ_NOMASK);
	debug_print("HTX_PAI_IRQ_MASKED   = 0x%08X\n", HTX_PAI_IRQ_MASKED);
	debug_print("HTX_PAI_IRQ_MASK     = 0x%08X\n", HTX_PAI_IRQ_MASK);
}
