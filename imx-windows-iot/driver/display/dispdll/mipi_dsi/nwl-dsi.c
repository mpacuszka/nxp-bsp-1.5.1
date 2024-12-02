// SPDX-License-Identifier: GPL-2.0+
/*
 * i.MX8 NWL MIPI DSI host driver
 *
 * Copyright 2017, 2023-2024 NXP
 * Copyright (C) 2020 Purism SPC
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/ktime.h>
#include <linux/completion.h>
#include <linux/irq.h>
#include <linux/firmware/imx/ipc.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <vdso/time64.h>
#include <linux/media-bus-format.h>
#include <linux/interrupt.h>
#include "drm_mipi_dsi.h"
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_crtc.h>
#include <drm/drm_encoder.h>

#include <dt-bindings/firmware/imx/rsrc.h>
#include <svc/pm/pm_api.h>
#include <svc/misc/misc_api.h>

#include "mipi_display.h"

#include "nwl-dsi.h"

#define DRV_NAME "nwl-dsi"

/* i.MX8 NWL quirks */
/* i.MX8MQ errata E11418 */
#define E11418_HS_MODE_QUIRK	BIT(0)

#define NWL_DSI_MIPI_FIFO_TIMEOUT msecs_to_jiffies(500)

/* Maximum Video PLL frequency: 1.2GHz */
#define MAX_PLL_FREQ 1200000000

#define MBPS(x) ((x) * 1000000)
#define MIN_PHY_RATE MBPS(24)
#define MAX_PHY_RATE MBPS(30)

#define DC_ID(x)	IMX_SC_R_DC_ ## x
#define MIPI_ID(x)	IMX_SC_R_MIPI_ ## x
#define SYNC_CTRL(x)	IMX_SC_C_SYNC_CTRL ## x
#define PXL_VLD(x)	IMX_SC_C_PXL_LINK_MST ## x ## _VLD
#define PXL_ADDR(x)	IMX_SC_C_PXL_LINK_MST ## x ## _ADDR

#define IMX8ULP_DSI_CM_MASK	BIT(1)
#define IMX8ULP_DSI_CM_NORMAL	BIT(1)

/* Possible valid PHY reference clock rates*/
static u32 phyref_rates[] = {
	27000000,
	24000000,
};

/*
 * TODO: find a better way to access imx_crtc_state
 */
struct imx_crtc_state {
	struct drm_crtc_state			base;
	u32					bus_format;
	u32					bus_flags;
	int					di_hsync_pin;
	int					di_vsync_pin;
};

static inline struct imx_crtc_state *to_imx_crtc_state(struct drm_crtc_state *s)
{
	return container_of(s, struct imx_crtc_state, base);
}

enum transfer_direction {
	DSI_PACKET_SEND,
	DSI_PACKET_RECEIVE,
};

#define NWL_DSI_ENDPOINT_LCDIF	0
#define NWL_DSI_ENDPOINT_DCSS	1
#define NWL_DSI_ENDPOINT_DCNANO	0
#define NWL_DSI_ENDPOINT_EPDC	1

struct nwl_dsi_transfer {
	const struct mipi_dsi_msg *msg;
	struct mipi_dsi_packet packet;
	struct completion completed;

	int status; /* status of transmission */
	enum transfer_direction direction;
	bool need_bta;
	u8 cmd;
	u16 rx_word_count;
	size_t tx_len; /* in bytes */
	size_t rx_len; /* in bytes */
};

struct mode_config {
	unsigned int		clock;
	unsigned int			crtc_clock;
	unsigned int			lanes;
	unsigned long			bitclock;
	unsigned long			phy_rates[3];
	unsigned long			pll_rates[3];
	unsigned int			phy_rate_idx;
};

struct nwl_dsi_platform_data;

struct nwl_dsi {
	struct drm_encoder encoder;
	struct mipi_dsi_host dsi_host;
	struct drm_panel* panel;
	struct device *dev;
	struct platform_device* phy_pdev;
	union phy_configure_opts phy_cfg;
	unsigned int quirks;
	unsigned int instance;
	const struct nwl_dsi_platform_data *pdata;

	struct regmap *regmap;
	struct regmap *csr;
	/*
	 * The DSI host controller needs this reset sequence according to NWL:
	 * 1. Deassert pclk reset to get access to DSI regs
	 * 2. Configure DSI Host and DPHY and enable DPHY
	 * 3. Deassert ESC and BYTE resets to allow host TX operations)
	 * 4. Send DSI cmds to configure peripheral (handled by panel drv)
	 * 5. Deassert DPI reset so DPI receives pixels and starts sending
	 *    DSI data
	 *
	 * TODO: Since panel_bridges do their DSI setup in enable we
	 * currently have 4. and 5. swapped.
	 */
	/* TODO: support reset for MQ: struct reset_control *rst_byte */
	/* TODO: mux control for MQ: struct mux_control *mux; */

	/* DSI clocks */
	struct clk *phy_ref_clk;
	struct clk *rx_esc_clk;
	struct clk *tx_esc_clk;
	struct clk *core_clk;
	struct clk *bypass_clk;
	struct clk *pixel_clk;
	struct clk *pll_clk;
	/*
	 * hardware bug: the i.MX8MQ needs this clock on during reset
	 * even when not using LCDIF.
	 */
	struct clk *lcdif_clk;

	/* dsi lanes */
	u32 lanes;
	enum mipi_dsi_pixel_format format;
	struct drm_display_mode mode;
	unsigned long dsi_mode_flags;
	int error;

	struct nwl_dsi_transfer *xfer;
	struct mode_config valid_mode;
	u32 clk_drop_lvl;
	bool use_dcss;
};

static const struct regmap_config nwl_dsi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = NWL_DSI_IRQ_MASK2,
};

typedef enum {
	NWL_DSI_CORE_CLK = BIT(1),
	NWL_DSI_LCDIF_CLK = BIT(2),
	NWL_DSI_BYPASS_CLK = BIT(3),
	NWL_DSI_PIXEL_CLK = BIT(4),
} nwl_dsi_clks;

struct nwl_dsi_platform_data {
	int (*pclk_reset)(struct nwl_dsi *dsi, bool reset);
	int (*mipi_reset)(struct nwl_dsi *dsi, bool reset);
	int (*dpi_reset)(struct nwl_dsi *dsi, bool reset);
	nwl_dsi_clks clks;
	u32 reg_tx_ulps;
	u32 reg_pxl2dpi;
	u32 reg_cm;
	u32 max_instances;
	u32 tx_clk_rate;
	u32 rx_clk_rate;
	bool mux_present;
	bool shared_phy;
	u32 bit_bta_timeout;
	u32 bit_hs_tx_timeout;
	bool use_lcdif_or_dcss;
	bool use_dcnano_or_epdc;
	bool rx_clk_quirk;	/* enable rx_esc clock to access registers */
	u32 quirk;	/* content of original nwl_dsi_quirks_match data stored here */
};

struct drm_encoder *nwl_dsi_get_encoder(struct platform_device *pdev)
{
	if (!pdev)
		return NULL;

	struct nwl_dsi *dsi = dev_get_drvdata(&pdev->dev);
	if (!dsi)
		return NULL;

	return &dsi->encoder;
}

struct regmap* nwl_dsi_get_regmap(struct platform_device* pdev)
{
	struct nwl_dsi* dsi = dev_get_drvdata(&pdev->dev);

	if (dsi) {
		return dsi->regmap;
	}
	return NULL;
}

void nwl_dsi_adjust_clk_drop_lvl(struct platform_device* pdev, unsigned int lvl)
{
	if (!pdev)
		return;

	struct nwl_dsi* dsi = dev_get_drvdata(&pdev->dev);
	if (!dsi)
		return;

	dsi->clk_drop_lvl = lvl;
}

static int nwl_dsi_clear_error(struct nwl_dsi *dsi)
{
	int ret = dsi->error;

	dsi->error = 0;
	return ret;
}

static void nwl_dsi_write(struct nwl_dsi *dsi, unsigned int reg, u32 val)
{
	int ret;

	if (dsi->error)
		return;

	ret = regmap_write(dsi->regmap, reg, val);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev,
			      "Failed to write NWL DSI reg 0x%x: %d\n", reg,
			      ret);
		dsi->error = ret;
	}
}

static u32 nwl_dsi_read(struct nwl_dsi *dsi, u32 reg)
{
	unsigned int val;
	int ret;

	if (dsi->error)
		return 0;

	ret = regmap_read(dsi->regmap, reg, &val);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to read NWL DSI reg 0x%x: %d\n",
			      reg, ret);
		dsi->error = ret;
	}
	return val;
}

static int nwl_dsi_get_dpi_pixel_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return NWL_DSI_PIXEL_FORMAT_16;
	case MIPI_DSI_FMT_RGB666:
		return NWL_DSI_PIXEL_FORMAT_18L;
	case MIPI_DSI_FMT_RGB666_PACKED:
		return NWL_DSI_PIXEL_FORMAT_18;
	case MIPI_DSI_FMT_RGB888:
		return NWL_DSI_PIXEL_FORMAT_24;
	default:
		return -EINVAL;
	}
}

/*
 * ps2bc - Picoseconds to byte clock cycles
 */
static u32 ps2bc(struct nwl_dsi *dsi, unsigned long long ps)
{
	u32 bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	return (u32)DIV64_U64_ROUND_UP(ps * dsi->mode.clock * bpp,
				  dsi->lanes * 8ULL * NSEC_PER_SEC);
}

/*
 * ui2bc - UI time periods to byte clock cycles
 */
static u32 ui2bc(unsigned int ui)
{
	return DIV_ROUND_UP(ui, BITS_PER_BYTE);
}

/*
 * us2bc - micro seconds to lp clock cycles
 */
static u32 us2lp(u32 lp_clk_rate, unsigned long us)
{
	u32 n = DIV_ROUND_UP(lp_clk_rate, USEC_PER_SEC);
	return (n * us);
}

static int nwl_dsi_config_host(struct nwl_dsi *dsi)
{
	u32 cycles;
	struct phy_configure_opts_mipi_dphy *cfg = &dsi->phy_cfg.mipi_dphy;

	if (dsi->lanes < 1 || dsi->lanes > 4)
		return -EINVAL;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "DSI Lanes %d\n", dsi->lanes);
	nwl_dsi_write(dsi, NWL_DSI_CFG_NUM_LANES, dsi->lanes - 1);

	if (dsi->dsi_mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		nwl_dsi_write(dsi, NWL_DSI_CFG_NONCONTINUOUS_CLK, 0x01);
	else
		nwl_dsi_write(dsi, NWL_DSI_CFG_NONCONTINUOUS_CLK, 0x00);

	if (dsi->dsi_mode_flags & MIPI_DSI_MODE_EOT_PACKET)
		nwl_dsi_write(dsi, NWL_DSI_CFG_AUTOINSERT_EOTP, 0x00);
	else
		nwl_dsi_write(dsi, NWL_DSI_CFG_AUTOINSERT_EOTP, 0x01);

	/* values in byte clock cycles */
	cycles = ui2bc(cfg->clk_pre);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "cfg_t_pre: 0x%x\n", cycles);
	nwl_dsi_write(dsi, NWL_DSI_CFG_T_PRE, cycles);
	cycles = ps2bc(dsi, cfg->lpx + cfg->clk_prepare + cfg->clk_zero);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "cfg_tx_gap (pre): 0x%x\n", cycles);
	cycles += ui2bc(cfg->clk_pre);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "cfg_t_post: 0x%x\n", cycles);
	nwl_dsi_write(dsi, NWL_DSI_CFG_T_POST, cycles);
	cycles = ps2bc(dsi, cfg->hs_exit);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "cfg_tx_gap: 0x%x\n", cycles);
	nwl_dsi_write(dsi, NWL_DSI_CFG_TX_GAP, cycles);

	nwl_dsi_write(dsi, NWL_DSI_CFG_EXTRA_CMDS_AFTER_EOTP, 0x01);
	nwl_dsi_write(dsi, NWL_DSI_CFG_HTX_TO_COUNT, 0x00);
	nwl_dsi_write(dsi, NWL_DSI_CFG_LRX_H_TO_COUNT, 0x00);
	nwl_dsi_write(dsi, NWL_DSI_CFG_BTA_H_TO_COUNT, 0x00);
	/* In LP clock cycles */
	cycles = us2lp(cfg->lp_clk_rate, cfg->wakeup);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "cfg_twakeup: 0x%x\n", cycles);
	nwl_dsi_write(dsi, NWL_DSI_CFG_TWAKEUP, cycles);

	return nwl_dsi_clear_error(dsi);
}

static int nwl_dsi_config_dpi(struct nwl_dsi *dsi)
{
	u32 mode;
	int color_format;
	bool burst_mode;
	int hfront_porch, hback_porch, vfront_porch, vback_porch;
	int hsync_len, vsync_len;
	int hfp, hbp, hsa;

	hfront_porch = dsi->mode.hsync_start - dsi->mode.hdisplay;
	hsync_len = dsi->mode.hsync_end - dsi->mode.hsync_start;
	hback_porch = dsi->mode.htotal - dsi->mode.hsync_end;

	vfront_porch = dsi->mode.vsync_start - dsi->mode.vdisplay;
	vsync_len = dsi->mode.vsync_end - dsi->mode.vsync_start;
	vback_porch = dsi->mode.vtotal - dsi->mode.vsync_end;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "hfront_porch = %d\n", hfront_porch);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "hback_porch = %d\n", hback_porch);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "hsync_len = %d\n", hsync_len);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "hdisplay = %d\n", dsi->mode.hdisplay);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "vfront_porch = %d\n", vfront_porch);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "vback_porch = %d\n", vback_porch);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "vsync_len = %d\n", vsync_len);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "vactive = %d\n", dsi->mode.vdisplay);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "clock = %d kHz\n", dsi->mode.clock);

	color_format = nwl_dsi_get_dpi_pixel_format(dsi->format);
	if (color_format < 0) {
		DRM_DEV_ERROR(dsi->dev, "Invalid color format 0x%x\n",
			      dsi->format);
		return color_format;
	}
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "pixel fmt = %d\n", dsi->format);

	nwl_dsi_write(dsi, NWL_DSI_INTERFACE_COLOR_CODING, NWL_DSI_DPI_24_BIT);
	nwl_dsi_write(dsi, NWL_DSI_PIXEL_FORMAT, color_format);
	/*
	 * Adjusting input polarity based on the video mode results in
	 * a black screen so always pick active low:
	 */
	nwl_dsi_write(dsi, NWL_DSI_VSYNC_POLARITY,
		      NWL_DSI_VSYNC_POLARITY_ACTIVE_LOW);
	nwl_dsi_write(dsi, NWL_DSI_HSYNC_POLARITY,
		      NWL_DSI_HSYNC_POLARITY_ACTIVE_LOW);

	burst_mode = (dsi->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_BURST) &&
		     !(dsi->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE);

	if (burst_mode) {
		nwl_dsi_write(dsi, NWL_DSI_VIDEO_MODE, NWL_DSI_VM_BURST_MODE);
		nwl_dsi_write(dsi, NWL_DSI_PIXEL_FIFO_SEND_LEVEL, 256);
	} else {
		mode = ((dsi->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) ?
				NWL_DSI_VM_BURST_MODE_WITH_SYNC_PULSES :
				NWL_DSI_VM_NON_BURST_MODE_WITH_SYNC_EVENTS);
		nwl_dsi_write(dsi, NWL_DSI_VIDEO_MODE, mode);
		nwl_dsi_write(dsi, NWL_DSI_PIXEL_FIFO_SEND_LEVEL,
			      dsi->mode.hdisplay);
	}

	/* TODO: Support computing according to datasheet and subtract packet overhead */
	hfp = hfront_porch;
	hbp = hback_porch;
	hsa = hsync_len;

	nwl_dsi_write(dsi, NWL_DSI_HFP, hfp);
	nwl_dsi_write(dsi, NWL_DSI_HBP, hbp);
	nwl_dsi_write(dsi, NWL_DSI_HSA, hsa);

	nwl_dsi_write(dsi, NWL_DSI_ENABLE_MULT_PKTS, 0x0);
	nwl_dsi_write(dsi, NWL_DSI_BLLP_MODE, 0x1);
	nwl_dsi_write(dsi, NWL_DSI_USE_NULL_PKT_BLLP, 0x0);
	nwl_dsi_write(dsi, NWL_DSI_VC, 0x0);

	nwl_dsi_write(dsi, NWL_DSI_PIXEL_PAYLOAD_SIZE, dsi->mode.hdisplay);
	nwl_dsi_write(dsi, NWL_DSI_VACTIVE, dsi->mode.vdisplay - 1);
	nwl_dsi_write(dsi, NWL_DSI_VBP, vback_porch);
	nwl_dsi_write(dsi, NWL_DSI_VFP, vfront_porch);

	return nwl_dsi_clear_error(dsi);
}

static int nwl_dsi_init_interrupts(struct nwl_dsi *dsi)
{
	u32 irq_enable = 0xffffffff;

	nwl_dsi_write(dsi, NWL_DSI_IRQ_MASK2, 0x7);

	if (dsi->panel) {
		irq_enable = ~(u32)(NWL_DSI_TX_PKT_DONE_MASK |
			NWL_DSI_RX_PKT_HDR_RCVD_MASK |
			NWL_DSI_TX_FIFO_OVFLW_MASK |
			dsi->pdata->bit_hs_tx_timeout);
	}

	nwl_dsi_write(dsi, NWL_DSI_IRQ_MASK, irq_enable);

	return nwl_dsi_clear_error(dsi);
}

static int nwl_dsi_host_attach(struct mipi_dsi_host *dsi_host,
			       struct mipi_dsi_device *device)
{
	struct nwl_dsi *dsi = container_of(dsi_host, struct nwl_dsi, dsi_host);
	struct device *dev = dsi->dev;
	struct drm_panel* panel;

	DRM_DEV_INFO(dev, "lanes=%u, format=0x%x flags=0x%lx\n", device->lanes,
		     device->format, device->mode_flags);

	if (device->lanes < 1 || device->lanes > 4)
		return -EINVAL;

	dsi->lanes = device->lanes;
	dsi->format = device->format;
	dsi->dsi_mode_flags = device->mode_flags;

	panel = of_drm_find_panel(&device->dev.of_node);
	if (IS_ERR(panel)) {
		panel = NULL;
	}
	dsi->panel = panel;

	return 0;
}

static int nwl_dsi_host_detach(struct mipi_dsi_host *dsi_host,
			       struct mipi_dsi_device *device)
{
	struct nwl_dsi *dsi = container_of(dsi_host, struct nwl_dsi, dsi_host);

	dsi->lanes = 0;
	dsi->format = 0;
	dsi->dsi_mode_flags = 0;
	dsi->panel = NULL;
	return 0;
}

static bool nwl_dsi_read_packet(struct nwl_dsi *dsi, u32 status)
{
	struct device *dev = dsi->dev;
	struct nwl_dsi_transfer *xfer = dsi->xfer;
	int err;
	u8 *payload = xfer->msg->rx_buf;
	u32 val;
	u16 word_count;
	u8 channel;
	u8 data_type;

	xfer->status = 0;

	if (xfer->rx_word_count == 0) {
		if (!(status & NWL_DSI_RX_PKT_HDR_RCVD))
			return false;
		/* Get the RX header and parse it */
		val = nwl_dsi_read(dsi, NWL_DSI_RX_PKT_HEADER);
		err = nwl_dsi_clear_error(dsi);
		if (err)
			xfer->status = err;
		word_count = NWL_DSI_WC(val);
		channel = (u8)NWL_DSI_RX_VC(val);
		data_type = (u8)NWL_DSI_RX_DT(val);

		if (channel != xfer->msg->channel) {
			DRM_DEV_ERROR(dev,
				      "[%02X] Channel mismatch (%u != %u)\n",
				      xfer->cmd, channel, xfer->msg->channel);
			xfer->status = -EINVAL;
			return true;
		}

		switch (data_type) {
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
			if (xfer->msg->rx_len > 1) {
				/* read second byte */
				payload[1] = word_count >> 8;
				++xfer->rx_len;
			}
			/* fallthrough */
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
			if (xfer->msg->rx_len > 0) {
				/* read first byte */
				payload[0] = word_count & 0xff;
				++xfer->rx_len;
			}
			xfer->status = (int)xfer->rx_len;
			return true;
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			word_count &= 0xff;
			DRM_DEV_ERROR(dev, "[%02X] DSI error report: 0x%02x\n",
				      xfer->cmd, word_count);
			xfer->status = -EPROTO;
			return true;
		}

		if (word_count > xfer->msg->rx_len) {
			DRM_DEV_ERROR(dev,
				"[%02X] Receive buffer too small: %zu (< %u)\n",
				xfer->cmd, xfer->msg->rx_len, word_count);
			xfer->status = -EINVAL;
			return true;
		}

		xfer->rx_word_count = word_count;
	} else {
		/* Set word_count from previous header read */
		word_count = xfer->rx_word_count;
	}

	/* If RX payload is not yet received, wait for it */
	if (!(status & NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD))
		return false;

	/* Read the RX payload */
	while (word_count >= 4) {
		val = nwl_dsi_read(dsi, NWL_DSI_RX_PAYLOAD);
		payload[0] = (val >> 0) & 0xff;
		payload[1] = (val >> 8) & 0xff;
		payload[2] = (val >> 16) & 0xff;
		payload[3] = (val >> 24) & 0xff;
		payload += 4;
		xfer->rx_len += 4;
		word_count -= 4;
	}

	if (word_count > 0) {
		val = nwl_dsi_read(dsi, NWL_DSI_RX_PAYLOAD);
		switch (word_count) {
		case 3:
			payload[2] = (val >> 16) & 0xff;
			++xfer->rx_len;
			/* fallthrough */
		case 2:
			payload[1] = (val >> 8) & 0xff;
			++xfer->rx_len;
			/* fallthrough */
		case 1:
			payload[0] = (val >> 0) & 0xff;
			++xfer->rx_len;
			break;
		}
	}

	xfer->status = (int)xfer->rx_len;
	err = nwl_dsi_clear_error(dsi);
	if (err)
		xfer->status = err;

	return true;
}

static void nwl_dsi_finish_transmission(struct nwl_dsi *dsi, u32 status)
{
	struct nwl_dsi_transfer *xfer = dsi->xfer;
	bool end_packet = false;

	if (!xfer)
		return;

	if (xfer->direction == DSI_PACKET_SEND &&
	    status & NWL_DSI_TX_PKT_DONE) {
		xfer->status = (int)xfer->tx_len;
		end_packet = true;
	} else if (status & NWL_DSI_DPHY_DIRECTION &&
		   ((status & (NWL_DSI_RX_PKT_HDR_RCVD |
			       NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD)))) {
		end_packet = nwl_dsi_read_packet(dsi, status);
	}

	if (end_packet)
		complete(&xfer->completed);
}

static void nwl_dsi_begin_transmission(struct nwl_dsi *dsi)
{
	struct nwl_dsi_transfer *xfer = dsi->xfer;
	struct mipi_dsi_packet *pkt = &xfer->packet;
	const u8 *payload;
	size_t length;
	u16 word_count;
	u8 hs_mode;
	u32 val;
	u32 hs_workaround = 0;

	/* Send the payload, if any */
	length = pkt->payload_length;
	payload = pkt->payload;

	while (length >= 4) {
		val = *(u32 *)payload;
		hs_workaround |= !(val & 0xFFFF00);
		nwl_dsi_write(dsi, NWL_DSI_TX_PAYLOAD, val);
		payload += 4;
		length -= 4;
	}
	/* Send the rest of the payload */
	val = 0;
	switch (length) {
	case 3:
		val |= payload[2] << 16;
		/* fallthrough */
	case 2:
		val |= payload[1] << 8;
		hs_workaround |= !(val & 0xFFFF00);
		/* fallthrough */
	case 1:
		val |= payload[0];
		nwl_dsi_write(dsi, NWL_DSI_TX_PAYLOAD, val);
		break;
	}
	xfer->tx_len = pkt->payload_length;

	/*
	 * Send the header
	 * header[0] = Virtual Channel + Data Type
	 * header[1] = Word Count LSB (LP) or first param (SP)
	 * header[2] = Word Count MSB (LP) or second param (SP)
	 */
	DRM_DEV_DEBUG(dsi->dev, "Pkt: 0x%x, 0x%x, 0x%x\n", pkt->header[0], pkt->header[1], pkt->header[2]);
	word_count = pkt->header[1] | (pkt->header[2] << 8);
	if (hs_workaround && (dsi->quirks & E11418_HS_MODE_QUIRK)) {
		DRM_DEV_DEBUG_DRIVER(dsi->dev,
				     "Using hs mode workaround for cmd 0x%x\n",
				     xfer->cmd);
		hs_mode = 1;
	} else {
		hs_mode = (xfer->msg->flags & MIPI_DSI_MSG_USE_LPM) ? 0 : 1;
	}
	val = NWL_DSI_WC(word_count) | NWL_DSI_TX_VC(xfer->msg->channel) |
	      NWL_DSI_TX_DT(xfer->msg->type) | NWL_DSI_HS_SEL(hs_mode) |
	      NWL_DSI_BTA_TX(xfer->need_bta);
	nwl_dsi_write(dsi, NWL_DSI_PKT_CONTROL, val);

	/* Send packet command */
	nwl_dsi_write(dsi, NWL_DSI_SEND_PACKET, 0x1);
}

static ssize_t nwl_dsi_host_transfer(struct mipi_dsi_host *dsi_host,
				     const struct mipi_dsi_msg *msg)
{
	struct nwl_dsi *dsi = container_of(dsi_host, struct nwl_dsi, dsi_host);
	struct nwl_dsi_transfer xfer;
	ssize_t ret = 0;

	/* Create packet to be sent */
	dsi->xfer = &xfer;
	ret = mipi_dsi_create_packet(&xfer.packet, msg);
	if (ret < 0) {
		dsi->xfer = NULL;
		return ret;
	}

	if ((msg->type & MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM ||
	     msg->type & MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM ||
	     msg->type & MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM ||
	     msg->type & MIPI_DSI_DCS_READ) &&
	    msg->rx_len > 0 && msg->rx_buf)
		xfer.direction = DSI_PACKET_RECEIVE;
	else
		xfer.direction = DSI_PACKET_SEND;

	xfer.need_bta = (xfer.direction == DSI_PACKET_RECEIVE);
	xfer.need_bta |= (msg->flags & MIPI_DSI_MSG_REQ_ACK) ? 1 : 0;
	xfer.msg = msg;
	xfer.status = -ETIMEDOUT;
	xfer.rx_word_count = 0;
	xfer.rx_len = 0;
	xfer.cmd = 0x00;
	if (msg->tx_len > 0)
		xfer.cmd = ((u8 *)(msg->tx_buf))[0];
	init_completion(&xfer.completed);

	ret = clk_prepare_enable(dsi->rx_esc_clk);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to enable rx_esc clk: %zd\n",
			      ret);
		return ret;
	}
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Enabled rx_esc clk @%lu Hz\n",
			     clk_get_rate(dsi->rx_esc_clk));

	/* Initiate the DSI packet transmision */
	nwl_dsi_begin_transmission(dsi);

	if (!wait_for_completion_timeout(&xfer.completed,
					 NWL_DSI_MIPI_FIFO_TIMEOUT)) {
		DRM_DEV_ERROR(dsi_host->dev, "[%02X] DSI transfer timed out\n",
			      xfer.cmd);
		ret = -ETIMEDOUT;
	} else {
		ret = xfer.status;
	}

	clk_disable_unprepare(dsi->rx_esc_clk);

	return ret;
}

static const struct mipi_dsi_host_ops nwl_dsi_host_ops = {
	.attach = nwl_dsi_host_attach,
	.detach = nwl_dsi_host_detach,
	.transfer = nwl_dsi_host_transfer,
};

irqreturn_t nwl_dsi_irq_handler(struct platform_device* pdev)
{
	u32 irq_status;
	irqreturn_t ret = IRQ_NONE;
	struct nwl_dsi* dsi = dev_get_drvdata(&pdev->dev);

	irq_status = nwl_dsi_read(dsi, NWL_DSI_IRQ_STATUS);

	if (irq_status & NWL_DSI_TX_FIFO_OVFLW) {
		DRM_DEV_ERROR_RATELIMITED(dsi->dev, "tx fifo overflow\n");
		ret = IRQ_HANDLED;
	}

	if (irq_status & dsi->pdata->bit_hs_tx_timeout) {
		DRM_DEV_ERROR_RATELIMITED(dsi->dev, "HS tx timeout\n");
		ret = IRQ_HANDLED;
	}

	if (irq_status & NWL_DSI_TX_PKT_DONE ||
		irq_status & NWL_DSI_RX_PKT_HDR_RCVD ||
		irq_status & NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD) {
		nwl_dsi_finish_transmission(dsi, irq_status);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static int nwl_dsi_enable(struct nwl_dsi *dsi)
{
	struct device *dev = dsi->dev;
	union phy_configure_opts *phy_cfg = &dsi->phy_cfg;
	int ret;

	if (!dsi->lanes) {
		DRM_DEV_ERROR(dev, "Need DSI lanes: %d\n", dsi->lanes);
		return -EINVAL;
	}

	ret = mixel_dphy_init(dsi->phy_pdev);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to init DSI phy: %d\n", ret);
		return ret;
	}

	ret = mixel_dphy_configure(dsi->phy_pdev, phy_cfg);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to configure DSI phy: %d\n", ret);
		goto uninit_phy;
	}

	ret = clk_prepare_enable(dsi->tx_esc_clk);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to enable tx_esc clk: %d\n",
			      ret);
		goto uninit_phy;
	}
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Enabled tx_esc clk @%lu Hz\n",
			     clk_get_rate(dsi->tx_esc_clk));

	ret = nwl_dsi_config_host(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set up DSI: %d", ret);
		goto disable_clock;
	}

	ret = nwl_dsi_config_dpi(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set up DPI: %d", ret);
		goto disable_clock;
	}

	ret = mixel_dphy_power_on(dsi->phy_pdev);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to power on DPHY (%d)\n", ret);
		goto disable_clock;
	}

	ret = nwl_dsi_init_interrupts(dsi);
	if (ret < 0)
		goto power_off_phy;

	return ret;

power_off_phy:
	mixel_dphy_power_off(dsi->phy_pdev);
disable_clock:
	clk_disable_unprepare(dsi->tx_esc_clk);
uninit_phy:
	mixel_dphy_exit(dsi->phy_pdev);

	return ret;
}

static int nwl_dsi_disable(struct nwl_dsi *dsi)
{
	struct device *dev = dsi->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "Disabling clocks and phy\n");

	mixel_dphy_power_off(dsi->phy_pdev);
	mixel_dphy_exit(dsi->phy_pdev);

	/* Disabling the clock before the phy breaks enabling dsi again */
	clk_disable_unprepare(dsi->tx_esc_clk);

	/* Disable rx_esc clock as registers are not accessed any more. */
	if (dsi->pdata->rx_clk_quirk)
		clk_disable_unprepare(dsi->rx_esc_clk);

	return 0;
}

void
nwl_dsi_bridge_atomic_post_disable(struct platform_device *pdev)
{
	struct nwl_dsi *dsi = dev_get_drvdata(&pdev->dev);
	int ret;

	/*
	 * Call panel's disable() callback(if any) so that
	 * it may send any MIPI DSI command before this MIPI DSI controller
	 * and it's PHY are disabled.
	 */
	if (dsi->panel) {
		ret = drm_panel_disable(dsi->panel);
		if (unlikely(ret)) {
			dev_err(dsi->dev, "panel disable failed: %d\n", ret);
			return;
		}
	}

	nwl_dsi_disable(dsi);

	ret = dsi->pdata->dpi_reset(dsi, true);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to assert DPI: %d\n", ret);
		return;
	}
	ret = dsi->pdata->mipi_reset(dsi, true);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to assert DSI: %d\n", ret);
		return;
	}
	ret = dsi->pdata->pclk_reset(dsi, true);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to assert PCLK: %d\n", ret);
		return;
	}

	/* unprepare panel if exists */
	if (dsi->panel) {
		ret = drm_panel_unprepare(dsi->panel);
		if (unlikely(ret)) {
			dev_err(dsi->dev, "panel unprepare failed: %d\n", ret);
			return;
		}
	}

	if (dsi->core_clk)
		clk_disable_unprepare(dsi->core_clk);
	if (dsi->bypass_clk)
		clk_disable_unprepare(dsi->bypass_clk);
	if (dsi->pixel_clk)
		clk_disable_unprepare(dsi->pixel_clk);
	if (dsi->lcdif_clk)
		clk_disable_unprepare(dsi->lcdif_clk);
}

static unsigned long nwl_dsi_get_bit_clock(struct nwl_dsi *dsi,
		unsigned long pixclock, u32 lanes)
{
	int bpp;

	if (lanes < 1 || lanes > 4)
		return 0;

	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	return (pixclock * bpp) / lanes;
}

/*
 * Utility function to calculate least commom multiple, using an improved
 * version of the Euclidean algorithm for greatest common factor.
 */
static unsigned long nwl_dsi_get_lcm(unsigned long a, unsigned long b)
{
	u32 gcf = 0; /* greatest common factor */
	unsigned long tmp_a = a;
	unsigned long tmp_b = b;

	if (!a || !b)
		return 0;

	while (tmp_a % tmp_b) {
		gcf = tmp_a % tmp_b;
		tmp_a = tmp_b;
		tmp_b = gcf;
	}

	if (!gcf)
		return a;

	return ((unsigned long long)a * b) / gcf;
}

/*
 * This function tries to adjust the crtc_clock for a DSI device in such a way
 * that the video pll will be able to satisfy both Display Controller pixel
 * clock (feeding out DPI interface) and our input phy_ref clock.
 * Also, the DC pixel clock must be lower than the actual clock in order to
 * have enough blanking space to send DSI commands, if the device is a panel.
 */
static void nwl_dsi_setup_pll_config(struct mode_config *config, u32 lvl)
{
	unsigned long pll_rate;
	int div;
	size_t i, num_rates = ARRAY_SIZE(config->phy_rates);

	config->crtc_clock = 0;

	for (i = 0; i < num_rates; i++) {
		int crtc_clock;

		if (!config->phy_rates[i])
			break;
		/*
		 * First, we need to check if phy_ref can actually be obtained
		 * from pixel clock. To do this, we check their lowest common
		 * multiple, which has to be in PLL range.
		 */
		pll_rate = nwl_dsi_get_lcm(config->clock, config->phy_rates[i]);
		if (pll_rate > MAX_PLL_FREQ) {
			/* Drop pll_rate to a realistic value */
			while (pll_rate > MAX_PLL_FREQ)
				pll_rate >>= 1;
			/* Make sure pll_rate can provide phy_ref rate */
			div = DIV_ROUND_UP(pll_rate, config->phy_rates[i]);
			pll_rate = config->phy_rates[i] * div;
		} else {
			/*
			 * Increase the pll rate to highest possible rate for
			 * better accuracy.
			 */
			while (pll_rate <= MAX_PLL_FREQ)
				pll_rate <<= 1;
			pll_rate >>= 1;
		}

		/*
		 * Next, we need to tweak the pll_rate to a value that can also
		 * satisfy the crtc_clock.
		 */
		div = DIV_ROUND_CLOSEST(pll_rate, config->clock);
		if (lvl)
			pll_rate -= config->phy_rates[i] * lvl;
		crtc_clock = pll_rate / div;
		config->pll_rates[i] = pll_rate;

		/*
		 * Pick a crtc_clock which is closest to pixel clock.
		 * Also, make sure that the pixel clock is a multiply of
		 * 50Hz.
		 */
		if (!(crtc_clock % 50) &&
		    abs(config->clock - crtc_clock) <
		    abs(config->clock - config->crtc_clock)) {
			config->crtc_clock = crtc_clock;
			config->phy_rate_idx = (unsigned int)i;
		}
	}
}


/*
 * This function will try the required phy speed for current mode
 * If the phy speed can be achieved, the phy will save the speed
 * configuration
 */
static struct mode_config *nwl_dsi_mode_probe(struct nwl_dsi *dsi,
			    const struct drm_display_mode *mode)
{
	struct device *dev = dsi->dev;
	union phy_configure_opts phy_opts;
	unsigned long clock = mode->clock * 1000;
	unsigned long bit_clk = 0;
	unsigned long phy_rates[3] = {0};
	int match_rates = 0;
	u32 lanes = dsi->lanes;
	size_t i = 0, num_rates = ARRAY_SIZE(phyref_rates);

	if (dsi->valid_mode.clock == clock)
		return &dsi->valid_mode;

	phy_mipi_dphy_get_default_config(clock,
			mipi_dsi_pixel_format_to_bpp(dsi->format),
			lanes, &phy_opts.mipi_dphy);
	phy_opts.mipi_dphy.lp_clk_rate = clk_get_rate(dsi->tx_esc_clk);

	while (i < num_rates) {
		int ret;

		bit_clk = nwl_dsi_get_bit_clock(dsi, clock, lanes);

		clk_set_rate(dsi->pll_clk, phyref_rates[i] * 32);
		clk_set_rate(dsi->phy_ref_clk, phyref_rates[i]);
		ret = mixel_dphy_validate(dsi->phy_pdev, PHY_MODE_MIPI_DPHY, 0, &phy_opts);

		/* Pick the non-failing rate, and search for more */
		if (!ret) {
			phy_rates[match_rates++] = phyref_rates[i++];
			continue;
		}

		if (match_rates)
			break;

		/* Reached the end of phyref_rates, try another lane config */
		if ((i++ == num_rates - 1) && (--lanes > 2)) {
			i = 0;
			continue;
		}
	}

	/*
	 * Try swinging between min and max pll rates and see what rate (in terms
	 * of kHz) we can custom use to get the required bit-clock.
	 */
	if (!match_rates) {
		int min_div, max_div;
		int bit_clk_khz;

		lanes = dsi->lanes;
		bit_clk = nwl_dsi_get_bit_clock(dsi, clock, lanes);

		min_div = DIV_ROUND_UP(bit_clk, MAX_PHY_RATE);
		max_div = (int)DIV_ROUND_DOWN_ULL(bit_clk, MIN_PHY_RATE);
		bit_clk_khz = bit_clk / 1000;

		for (i = max_div; i > min_div; i--) {
			if (!(bit_clk_khz % i)) {
				phy_rates[0] = bit_clk / (unsigned long)i;
				match_rates = 1;
				break;
			}
		}
	}

	if (!match_rates) {
		DRM_DEV_DEBUG_DRIVER(dev,
			"Cannot setup PHY for mode: %ux%u @%d kHz\n",
			mode->hdisplay,
			mode->vdisplay,
			mode->clock);

		return NULL;
	}

	dsi->valid_mode.clock = clock;
	dsi->valid_mode.lanes = lanes;
	dsi->valid_mode.bitclock = bit_clk;
	memcpy(&dsi->valid_mode.phy_rates, &phy_rates, sizeof(phy_rates));

	return &dsi->valid_mode;
}


static int nwl_dsi_get_dphy_params(struct nwl_dsi *dsi,
				   const struct drm_display_mode *mode,
				   union phy_configure_opts *phy_opts)
{
	unsigned long rate;
	int ret;

	if (dsi->lanes < 1 || dsi->lanes > 4)
		return -EINVAL;

	/*
	 * So far the DPHY spec minimal timings work for both mixel
	 * dphy and nwl dsi host
	 */
	ret = phy_mipi_dphy_get_default_config(mode->clock * 1000,
		mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes,
		&phy_opts->mipi_dphy);
	if (ret < 0)
		return ret;

	rate = clk_get_rate(dsi->tx_esc_clk);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "LP clk is @%lu Hz\n", rate);
	phy_opts->mipi_dphy.lp_clk_rate = rate;

	return 0;
}

enum drm_mode_status
nwl_dsi_bridge_mode_valid(struct platform_device *pdev,
			  const struct drm_display_mode *mode)
{
	struct nwl_dsi *dsi = dev_get_drvdata(&pdev->dev);
	struct mode_config *config;
	unsigned long pll_rate;
	int bit_rate;

	bit_rate = nwl_dsi_get_bit_clock(dsi, mode->clock * 1000, dsi->lanes);
 
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Validating mode:");
	drm_mode_debug_printmodeline(mode);

	if (bit_rate > MBPS(1500))
		return MODE_CLOCK_HIGH;

	if (bit_rate < MBPS(80))
		return MODE_CLOCK_LOW;

	config = nwl_dsi_mode_probe(dsi, mode);
	if (!config)
		return MODE_NOCLOCK;

	pll_rate = config->pll_rates[config->phy_rate_idx];
	if (dsi->pll_clk && !pll_rate)
		nwl_dsi_setup_pll_config(config, dsi->clk_drop_lvl);

	return MODE_OK;
}

int nwl_dsi_bridge_atomic_check(struct platform_device *pdev,
				       struct drm_crtc_state *crtc_state)
{
	struct nwl_dsi *dsi = dev_get_drvdata(&pdev->dev);
	struct drm_display_mode *adjusted = &crtc_state->adjusted_mode;
	struct mode_config *config;
	unsigned long pll_rate;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "atomic check:\n");
	drm_mode_debug_printmodeline(adjusted);

	config = nwl_dsi_mode_probe(dsi, adjusted);
	if (!config)
		return -EINVAL;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "lanes=%u, data_rate=%lu\n",
			     config->lanes, config->bitclock);
	if (config->lanes < 2 || config->lanes > 4)
		return -EINVAL;

	/* Max data rate for this controller is 1.5Gbps */
	if (config->bitclock > 1500000000)
		return -EINVAL;

	pll_rate = config->pll_rates[config->phy_rate_idx];
	if (dsi->pll_clk && pll_rate) {
		clk_set_rate(dsi->pll_clk, pll_rate);
		DRM_DEV_DEBUG_DRIVER(dsi->dev,
				     "Video pll rate: %lu (actual: %lu)",
				     pll_rate, clk_get_rate(dsi->pll_clk));
	}
	/* Update the crtc_clock to be used by display controller */
	if (config->crtc_clock)
		adjusted->crtc_clock = config->crtc_clock / 1000;
	else if (dsi->clk_drop_lvl) {
		int div;
		unsigned long phy_ref_rate;

		phy_ref_rate = config->phy_rates[config->phy_rate_idx];
		pll_rate = config->bitclock;
		div = DIV_ROUND_CLOSEST(pll_rate, config->clock);
		pll_rate -= phy_ref_rate * dsi->clk_drop_lvl;
		adjusted->crtc_clock = (pll_rate / div) / 1000;
	}

	if (!dsi->use_dcss && !dsi->pdata->use_dcnano_or_epdc) {
		/* At least LCDIF + NWL needs active high sync */
		adjusted->flags |= (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
		adjusted->flags &= ~(DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
	} else {
		adjusted->flags &= ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
		adjusted->flags |= (DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
	}

	return 0;
}

void
nwl_dsi_bridge_mode_set(struct platform_device *pdev,
			const struct drm_display_mode *adjusted_mode)
{
	struct nwl_dsi *dsi = dev_get_drvdata(&pdev->dev);
	struct device *dev = dsi->dev;
	union phy_configure_opts new_cfg;
	unsigned long phy_ref_rate;
	struct mode_config *config;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Setting mode:\n");
	drm_mode_debug_printmodeline(adjusted_mode);

	config = nwl_dsi_mode_probe(dsi, adjusted_mode);
	/* New mode? This should NOT happen */
	if (!config) {
		DRM_DEV_ERROR(dsi->dev, "Unsupported mode provided:\n");
		return;
	}

	/*
	 * If bypass and pixel clocks are present, we need to set their rates
	 * now.
	 */
	if (dsi->bypass_clk)
		clk_set_rate(dsi->bypass_clk, adjusted_mode->crtc_clock * 1000);
	if (dsi->pixel_clk)
		clk_set_rate(dsi->pixel_clk, adjusted_mode->crtc_clock * 1000);

	memcpy(&dsi->mode, adjusted_mode, sizeof(dsi->mode));

	phy_ref_rate = config->phy_rates[config->phy_rate_idx];
	clk_set_rate(dsi->phy_ref_clk, phy_ref_rate);
	ret = nwl_dsi_get_dphy_params(dsi, adjusted_mode, &new_cfg);
	if (ret < 0)
		return;

	DRM_WARN("PHY at ref rate: %lu (actual: %lu)\n",
			     phy_ref_rate, clk_get_rate(dsi->phy_ref_clk));

	/* Save the new desired phy config */
	memcpy(&dsi->phy_cfg, &new_cfg, sizeof(new_cfg));
}

void
nwl_dsi_bridge_atomic_pre_enable(struct platform_device *pdev)
{
	struct nwl_dsi *dsi = dev_get_drvdata(&pdev->dev);
	int ret;

	dsi->pdata->dpi_reset(dsi, true);
	dsi->pdata->mipi_reset(dsi, true);
	dsi->pdata->pclk_reset(dsi, true);

	if (dsi->lcdif_clk && clk_prepare_enable(dsi->lcdif_clk) < 0)
		return;
	if (dsi->core_clk && clk_prepare_enable(dsi->core_clk) < 0)
		return;
	if (dsi->bypass_clk && clk_prepare_enable(dsi->bypass_clk) < 0)
		return;
	if (dsi->pixel_clk && clk_prepare_enable(dsi->pixel_clk) < 0)
		return;
	/*
	 * Enable rx_esc clock for some platforms to access DSI host controller
	 * and PHY registers.
	 */
	if (dsi->pdata->rx_clk_quirk && clk_prepare_enable(dsi->rx_esc_clk) < 0)
		return;

    /* prepare panel if exists */
	if (dsi->panel) {
		ret = drm_panel_prepare(dsi->panel);
		if (unlikely(ret)) {
			DRM_DEV_ERROR(dsi->dev, "panel prepare failed: %d\n", ret);
			return;
		}
	}

	/* Always use normal mode(full mode) for Type-4 display */
	if (dsi->pdata->reg_cm)
		regmap_update_bits(dsi->csr, dsi->pdata->reg_cm,
				   IMX8ULP_DSI_CM_MASK, IMX8ULP_DSI_CM_NORMAL);

	/* Step 1 from DSI reset-out instructions */
	ret = dsi->pdata->pclk_reset(dsi, false);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to deassert PCLK: %d\n", ret);
		return;
	}

	/* Step 2 from DSI reset-out instructions */
	nwl_dsi_enable(dsi);

	/* Step 3 from DSI reset-out instructions */
	ret = dsi->pdata->mipi_reset(dsi, false);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to deassert DSI: %d\n", ret);
		return;
	}

	/*
	 * We need to force call enable for the panel here, in order to
	 * make the panel initialization execute before our call to
	 * bridge_enable, where we will enable the DPI and start streaming
	 * pixels on the data lanes.
	 */
	/* enable panel if exists */
	if (dsi->panel) {
		ret = drm_panel_enable(dsi->panel);
		if (unlikely(ret)) {
			DRM_DEV_ERROR(dsi->dev, "panel enable failed: %d\n", ret);
		}
	}
}

void
nwl_dsi_bridge_atomic_enable(struct platform_device *pdev)
{
	struct nwl_dsi *dsi = dev_get_drvdata(&pdev->dev);
	int ret;

	/* Step 5 from DSI reset-out instructions */
	ret = dsi->pdata->dpi_reset(dsi, false);
	if (ret < 0)
		DRM_DEV_ERROR(dsi->dev, "Failed to deassert DPI: %d\n", ret);
}

int nwl_dsi_bridge_attach(struct platform_device *pdev)
{
	struct nwl_dsi *dsi = dev_get_drvdata(&pdev->dev);
	struct clk *phy_parent;
	int ret;

	phy_parent = devm_clk_get(dsi->dev, "phy_parent");
	if (!IS_ERR_OR_NULL(phy_parent)) {
		clk_disable_unprepare(dsi->tx_esc_clk);
		clk_disable_unprepare(dsi->rx_esc_clk);
		clk_disable_unprepare(dsi->phy_ref_clk);
		ret = clk_set_parent(dsi->phy_ref_clk, phy_parent);
		ret |= clk_set_parent(dsi->tx_esc_clk, phy_parent);
		ret |= clk_set_parent(dsi->rx_esc_clk, phy_parent);

		if (ret) {
			dev_err(dsi->dev,
				"Error re-parenting phy/tx/rx clocks: %d",
				ret);

			return ret;
		}

		if (dsi->pdata->tx_clk_rate)
			clk_set_rate(dsi->tx_esc_clk, dsi->pdata->tx_clk_rate);

		if (dsi->pdata->rx_clk_rate)
			clk_set_rate(dsi->rx_esc_clk, dsi->pdata->rx_clk_rate);
	}

	return 0;
}

void nwl_dsi_bridge_detach(struct platform_device *pdev)
{
}

static int nwl_dsi_parse_dt(struct nwl_dsi *dsi)
{
	struct platform_device *pdev = to_platform_device(dsi->dev);
	struct device_node *np = &dsi->dev->of_node;
	struct clk *clk;
	struct resource* iores = NULL;
	int ret;
	unsigned int id;

	of_property_read_u32(np, "mipi-dsi-id", &id);
	if (id > 0) {
		if (id > dsi->pdata->max_instances - 1) {
			dev_err(dsi->dev,
				"Too many instances! (cur: %d, max: %d)\n",
				id, dsi->pdata->max_instances);
			return -ENODEV;
		}
	}
	dsi->instance = id;

	if (dsi->pdata->clks & NWL_DSI_LCDIF_CLK) {
		clk = devm_clk_get(dsi->dev, "lcdif");
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			DRM_DEV_ERROR(dsi->dev,
				      "Failed to get lcdif clock: %d\n",
				      ret);
			return ret;
		}
		dsi->lcdif_clk = clk;
	}

	if (dsi->pdata->clks & NWL_DSI_CORE_CLK) {
		clk = devm_clk_get(dsi->dev, "core");
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			DRM_DEV_ERROR(dsi->dev,
				      "Failed to get core clock: %d\n",
				      ret);
			return ret;
		}
		dsi->core_clk = clk;
	}

	if (dsi->pdata->clks & NWL_DSI_BYPASS_CLK) {
		clk = devm_clk_get(dsi->dev, "bypass");
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			DRM_DEV_ERROR(dsi->dev,
				      "Failed to get bypass clock: %d\n",
				      ret);
			return ret;
		}
		dsi->bypass_clk = clk;
	}

	if (dsi->pdata->clks & NWL_DSI_PIXEL_CLK) {
		clk = devm_clk_get(dsi->dev, "pixel");
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			DRM_DEV_ERROR(dsi->dev,
				      "Failed to get pixel clock: %d\n",
				      ret);
			return ret;
		}
		dsi->pixel_clk = clk;
	}

	clk = devm_clk_get(dsi->dev, "phy_ref");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		DRM_DEV_ERROR(dsi->dev, "Failed to get phy_ref clock: %d\n",
			      ret);
		return ret;
	}
	dsi->phy_ref_clk = clk;

	clk = devm_clk_get(dsi->dev, "rx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		DRM_DEV_ERROR(dsi->dev, "Failed to get rx_esc clock: %d\n",
			      ret);
		return ret;
	}
	dsi->rx_esc_clk = clk;

	clk = devm_clk_get(dsi->dev, "tx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		DRM_DEV_ERROR(dsi->dev, "Failed to get tx_esc clock: %d\n",
			      ret);
		return ret;
	}
	dsi->tx_esc_clk = clk;

	/* The video_pll clock is optional */
	clk = devm_clk_get(dsi->dev, "video_pll");
	if (!IS_ERR(clk))
		dsi->pll_clk = clk;

 
	if (dsi->pdata->mux_present) {
		/* TODO: dsi->mux = xxx for platforms with mux present */
	}

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iores)
		return -ENODEV;

	dsi->regmap =
		devm_regmap_init_mmio(dsi->dev, iores, &nwl_dsi_regmap_config);
	if (IS_ERR(dsi->regmap)) {
		ret = PTR_ERR(dsi->regmap);
		DRM_DEV_ERROR(dsi->dev, "Failed to create NWL DSI regmap: %d\n",
			      ret);
		return ret;
	}

	/* For these three regs, we need a mapping to MIPI-DSI CSR */
	if (dsi->pdata->reg_tx_ulps || dsi->pdata->reg_pxl2dpi ||
	    dsi->pdata->reg_cm) {
		iores = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!iores)
			return -ENODEV;

		dsi->csr = devm_regmap_init_mmio(dsi->dev, iores, &nwl_dsi_regmap_config);
		if (IS_ERR(dsi->csr)) {
			ret = PTR_ERR(dsi->csr);
			dev_err(dsi->dev,
				"Failed to get CSR regmap: %d\n", ret);
			return ret;
		}
	}

	/* TODO: Initialize reset control for MQ */

	of_property_read_u32(np, "fsl,clock-drop-level", &dsi->clk_drop_lvl);

	memset(&dsi->valid_mode, 0, sizeof(struct mode_config));

	return 0;
}

static int nwl_dsi_select_input(struct nwl_dsi *dsi)
{
	u32 use_dcss_or_epdc = 1;
	int ret = 0;

	/* If there is no mux, nothing to do here */
	if (!dsi->pdata->mux_present)
		return 0;

	/* TODO: support mux for certain platforms	ret = mux_control_try_select(dsi->mux, use_dcss_or_epdc); */

	if (of_device_is_compatible(&dsi->dev->of_node, "fsl,imx8mq-nwl-dsi"))
		dsi->use_dcss = use_dcss_or_epdc;

	return ret;
}

static int nwl_dsi_deselect_input(struct nwl_dsi *dsi)
{
	int ret = 0;

	/* If there is no mux, nothing to do here */
	if (!dsi->pdata->mux_present)
		return 0;

	/* TODO: support mux for certain platforms	ret = mux_control_deselect(dsi->mux); */

	return ret;
}

static int imx8_common_dsi_pclk_reset(struct nwl_dsi *dsi, bool reset)
{
	int ret = 0;
	/* TODO: support reset for MQ */
	return ret;

}

static int imx8_common_dsi_mipi_reset(struct nwl_dsi *dsi, bool reset)
{
	int ret = 0;
	/* TODO: support reset for MQ */
	return ret;

}

static int imx8_common_dsi_dpi_reset(struct nwl_dsi *dsi, bool reset)
{
	int ret = 0;
	/* TODO: support reset for MQ */
	return ret;

}

static int imx8q_dsi_pclk_reset(struct nwl_dsi *dsi, bool reset)
{
	struct _sc_ipc_struct_t *handle;
	sc_rsrc_t mipi_id, dc_id;
	u8 ctrl;
	bool shared_phy = dsi->pdata->shared_phy;
	int ret = 0;

	handle = imx_scu_get_handle();
	if (!handle) {
		DRM_DEV_ERROR(dsi->dev,
			      "Failed to get scu ipc handle (%d)\n", ret);
		return ret;
	}

	mipi_id = (dsi->instance)?MIPI_ID(1):MIPI_ID(0);
	dc_id = (!shared_phy && dsi->instance)?DC_ID(1):DC_ID(0);
	DRM_DEV_DEBUG_DRIVER(dsi->dev,
			     "Power %s PCLK MIPI:%u DC:%u\n",
			     (reset)?"OFF":"ON", mipi_id, dc_id);

	if (shared_phy) {
		ret |= sc_misc_set_control(handle,
				mipi_id, IMX_SC_C_MODE, reset);
		ret |= sc_misc_set_control(handle,
				mipi_id, IMX_SC_C_DUAL_MODE, reset);
		ret |= sc_misc_set_control(handle,
				mipi_id, IMX_SC_C_PXL_LINK_SEL, reset);
	}

	ctrl = (shared_phy && dsi->instance)?PXL_VLD(2):PXL_VLD(1);
	ret |= sc_misc_set_control(handle, dc_id, ctrl, !reset);

	ctrl = (shared_phy && dsi->instance)?SYNC_CTRL(1):SYNC_CTRL(0);
	ret |= sc_misc_set_control(handle, dc_id, ctrl, !reset);

	return ret;
}

static int imx8q_dsi_mipi_reset(struct nwl_dsi *dsi, bool reset)
{
	struct _sc_ipc_struct_t *handle;
	sc_rsrc_t mipi_id;
	int ret = 0;

	handle = imx_scu_get_handle();
	if (!handle) {
		DRM_DEV_ERROR(dsi->dev,
			      "Failed to get scu ipc handle (%d)\n", ret);
		return ret;
	}

	mipi_id = (dsi->instance)?MIPI_ID(1):MIPI_ID(0);
	DRM_DEV_DEBUG_DRIVER(dsi->dev,
			     "Power %s HOST MIPI:%u\n",
			     (reset)?"OFF":"ON", mipi_id);

	ret |= sc_misc_set_control(handle, mipi_id,
				       IMX_SC_C_PHY_RESET, !reset);
	ret |= sc_misc_set_control(handle, mipi_id,
				       IMX_SC_C_MIPI_RESET, !reset);

	return ret;
}

static int imx8q_dsi_dpi_reset(struct nwl_dsi *dsi, bool reset)
{
	struct _sc_ipc_struct_t *handle = NULL;
	sc_rsrc_t mipi_id;
	int ret = 0;

	handle = imx_scu_get_handle();
	if (!handle) {
		DRM_DEV_ERROR(dsi->dev,
			      "Failed to get scu ipc handle (%d)\n", ret);
		return -ENODEV;
	}

	mipi_id = (dsi->instance)?MIPI_ID(1):MIPI_ID(0);
	DRM_DEV_DEBUG_DRIVER(dsi->dev,
			     "Power %s DPI MIPI:%u\n",
			     (reset)?"OFF":"ON", mipi_id);

	regmap_write(dsi->csr, dsi->pdata->reg_tx_ulps, 0);
	regmap_write(dsi->csr, dsi->pdata->reg_pxl2dpi, NWL_DSI_DPI_24_BIT);

	ret |= sc_misc_set_control(handle, mipi_id,
				       IMX_SC_C_DPI_RESET, !reset);

	return ret;
}

static const struct nwl_dsi_platform_data imx8mq_dev = {
	.pclk_reset = &imx8_common_dsi_pclk_reset,
	.mipi_reset = &imx8_common_dsi_mipi_reset,
	.dpi_reset = &imx8_common_dsi_dpi_reset,
	.clks = NWL_DSI_CORE_CLK | NWL_DSI_LCDIF_CLK,
	.mux_present = true,
	.bit_hs_tx_timeout = BIT(29),
	.bit_bta_timeout = BIT(31),
	.use_lcdif_or_dcss = true,
	.quirk = E11418_HS_MODE_QUIRK,
};

static const struct nwl_dsi_platform_data imx8qm_dev = {
	.pclk_reset = &imx8q_dsi_pclk_reset,
	.mipi_reset = &imx8q_dsi_mipi_reset,
	.dpi_reset = &imx8q_dsi_dpi_reset,
	.clks = NWL_DSI_BYPASS_CLK | NWL_DSI_PIXEL_CLK,
	.reg_tx_ulps = 0x00,
	.reg_pxl2dpi = 0x04,
	.max_instances = 2,
	.tx_clk_rate = 18000000,
	.rx_clk_rate = 72000000,
	.shared_phy = false,
	.bit_hs_tx_timeout = BIT(29),
	.bit_bta_timeout = BIT(31),
};

static const struct nwl_dsi_platform_data imx8qx_dev = {
	.pclk_reset = &imx8q_dsi_pclk_reset,
	.mipi_reset = &imx8q_dsi_mipi_reset,
	.dpi_reset = &imx8q_dsi_dpi_reset,
	.clks = NWL_DSI_BYPASS_CLK | NWL_DSI_PIXEL_CLK,
	.reg_tx_ulps = 0x30,
	.reg_pxl2dpi = 0x40,
	.max_instances = 2,
	.tx_clk_rate = 18000000,
	.rx_clk_rate = 72000000,
	.shared_phy = true,
	.bit_hs_tx_timeout = BIT(29),
	.bit_bta_timeout = BIT(31),
};

static const struct of_device_id nwl_dsi_dt_ids[] = {
	{ .compatible = "fsl,imx8mq-nwl-dsi", .data = &imx8mq_dev, },
	{ .compatible = "fsl,imx8qm-nwl-dsi", .data = &imx8qm_dev, },
	{ .compatible = "fsl,imx8qx-nwl-dsi", .data = &imx8qx_dev, },
	{ /* sentinel */ 0 }
};

int nwl_dsi_encoder_atomic_check(struct drm_encoder *encoder,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);

	imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB101010_1X30;

	return 0;
}

int nwl_dsi_probe(struct platform_device *pdev, struct platform_device *phy_pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id = of_match_device(nwl_dsi_dt_ids, dev);
	struct nwl_dsi *dsi;
	int ret;

	if (!of_id || !of_id->data)
		return -ENODEV;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	memset(dsi, 0, sizeof(struct nwl_dsi));
	dsi->dev = dev;
	dsi->pdata = of_id->data;
	dsi->phy_pdev = phy_pdev;

	dsi->quirks = dsi->pdata->quirk;

	ret = nwl_dsi_parse_dt(dsi);
	if (ret)
		return ret;

	dsi->dsi_host.ops = &nwl_dsi_host_ops;
	dsi->dsi_host.dev = dev;
	ret = mipi_dsi_host_register(&dsi->dsi_host);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to register MIPI host: %d\n", ret);
		return ret;
	}

	dsi->encoder.encoder_type = DRM_MODE_ENCODER_DSI;

	dev_set_drvdata(dev, dsi);

	ret = nwl_dsi_select_input(dsi);
	if (ret < 0) {
		mipi_dsi_host_unregister(&dsi->dsi_host);
		return ret;
	}

	return ret;
}

int nwl_dsi_remove(struct platform_device *pdev)
{
	struct nwl_dsi *dsi = platform_get_drvdata(pdev);
	struct resource *res;

	if (dsi) {
		/* TODO: add nwl_dsi_deselect_input(dsi) for MQ platform */
		mipi_dsi_host_unregister(&dsi->dsi_host);
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (res && dsi->regmap) {
			regmap_release_mmio(dsi->regmap, res);
		}
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (res && dsi->csr) {
			regmap_release_mmio(dsi->csr, res);
		}
		devm_kfree(&pdev->dev, dsi);
	}
	return 0;
}

static void nwl_dsi_dump_reg(struct nwl_dsi* dsi, struct regmap* regmap, uint32_t reg, char* reg_name)
{
	uint32_t status;
	uint32_t base;

	if (dsi->csr == regmap) {
		base = dsi->instance ? 0x56241000 : 0x56221000;
	}
	else {
		base = dsi->instance ? 0x56248000 : 0x56228000;
	}

	(void)regmap_read(regmap, reg, &status);
	DRM_DEV_ERROR(dsi->dev, "%s(0x%x) = 0x%02X\n", reg_name, (base + reg), status);
}


void nwl_dsi_dump(struct platform_device* pdev)
{
	struct nwl_dsi* dsi = platform_get_drvdata(pdev);

	if (dsi == NULL) {
		return;
	}

	if (dsi->pdata->reg_tx_ulps || dsi->pdata->reg_pxl2dpi) {
		/* CSR initialization */
		DRM_DEV_ERROR(dsi->dev, "*************************** CSR init ****************************************\n");
		nwl_dsi_dump_reg(dsi, dsi->csr, dsi->pdata->reg_tx_ulps, "MIPIv2_CSR_TX_ULPS");
		nwl_dsi_dump_reg(dsi, dsi->csr, dsi->pdata->reg_pxl2dpi, "MIPIv2_CSR_PXL2DPI");
	}

	/* IRQ initialization */
	DRM_DEV_ERROR(dsi->dev, "*************************** IRQ init ****************************************\n");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_IRQ_MASK, "HOST_IRQ_MASK");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_IRQ_MASK2, "HOST_IRQ_MASK2");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_IRQ_STATUS, "HOST_IRQ_STATUS");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_IRQ_STATUS2, "HOST_IRQ_STATUS2");

	/* HOST initialization */
	DRM_DEV_ERROR(dsi->dev, "*************************** HOST init ****************************************\n");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_NUM_LANES, "HOST_CFG_NUM_LANES");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_NONCONTINUOUS_CLK, "HOST_CFG_NONCONTINUOUS_CLK");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_T_PRE, "HOST_CFG_T_PRE");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_T_POST, "HOST_CFG_T_POST");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_TX_GAP, "HOST_CFG_TX_GAP");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_AUTOINSERT_EOTP, "HOST_CFG_AUTOINSERT_EOTP");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_EXTRA_CMDS_AFTER_EOTP, "HOST_CFG_EXTRA_CMDS_AFTER_EOTP");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_HTX_TO_COUNT, "HOST_CFG_HTX_TO_COUNT");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_LRX_H_TO_COUNT, "HOST_CFG_LRX_H_TO_COUNT");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_BTA_H_TO_COUNT, "HOST_CFG_BTA_H_TO_COUNT");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_TWAKEUP, "HOST_CFG_TWAKEUP");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_CFG_STATUS_OUT, "HOST_CFG_STATUS_OUT");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_RX_ERROR_STATUS, "HOST_RX_ERROR_STATUS");

	/* DPI initialization */
	DRM_DEV_ERROR(dsi->dev, "*************************** DPI init ****************************************\n");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_PIXEL_PAYLOAD_SIZE, "DPI_PIXEL_PAYLOAD_SIZE");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_PIXEL_FIFO_SEND_LEVEL, "DPI_PIXEL_FIFO_SEND_LEVEL");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_INTERFACE_COLOR_CODING, "DPI_INTERFACE_COLOR_CODING");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_PIXEL_FORMAT, "DPI_PIXEL_FORMAT");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_VSYNC_POLARITY, "DPI_VSYNC_POLARITY");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_HSYNC_POLARITY, "DPI_HSYNC_POLARITY");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_VIDEO_MODE, "DPI_VIDEO_MODE");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_HFP, "DPI_HFP");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_HBP, "DPI_HBP");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_HSA, "DPI_HSA");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_ENABLE_MULT_PKTS, "DPI_ENABLE_MULT_PKTS");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_VBP, "DPI_VBP");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_VFP, "DPI_VFP");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_BLLP_MODE, "DPI_BLLP_MODE");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_USE_NULL_PKT_BLLP, "DPI_USE_NULL_PKT_BLLP");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_VACTIVE, "DPI_VACTIVE");
	nwl_dsi_dump_reg(dsi, dsi->regmap, NWL_DSI_VC, "DPI_VC");
	/* PHY initialization */
	mixel_dphy_dump(dsi->phy_pdev);
}
