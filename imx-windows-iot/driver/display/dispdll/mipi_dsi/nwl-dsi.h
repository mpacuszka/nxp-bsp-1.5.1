/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 * Copyright 2024 NXP
 */
#ifndef __NWL_DSI_H__
#define __NWL_DSI_H__

#include <linux/interrupt.h>

/* DSI HOST registers */
#define NWL_DSI_CFG_NUM_LANES			0x0
#define NWL_DSI_CFG_NONCONTINUOUS_CLK		0x4
#define NWL_DSI_CFG_T_PRE			0x8
#define NWL_DSI_CFG_T_POST			0xc
#define NWL_DSI_CFG_TX_GAP			0x10
#define NWL_DSI_CFG_AUTOINSERT_EOTP		0x14
#define NWL_DSI_CFG_EXTRA_CMDS_AFTER_EOTP	0x18
#define NWL_DSI_CFG_HTX_TO_COUNT		0x1c
#define NWL_DSI_CFG_LRX_H_TO_COUNT		0x20
#define NWL_DSI_CFG_BTA_H_TO_COUNT		0x24
#define NWL_DSI_CFG_TWAKEUP			0x28
#define NWL_DSI_CFG_STATUS_OUT			0x2c
#define NWL_DSI_RX_ERROR_STATUS			0x30

/* DSI DPI registers */
#define NWL_DSI_PIXEL_PAYLOAD_SIZE		0x200
#define NWL_DSI_PIXEL_FIFO_SEND_LEVEL		0x204
#define NWL_DSI_INTERFACE_COLOR_CODING		0x208
#define NWL_DSI_PIXEL_FORMAT			0x20c
#define NWL_DSI_VSYNC_POLARITY			0x210
#define NWL_DSI_VSYNC_POLARITY_ACTIVE_LOW	0
#define NWL_DSI_VSYNC_POLARITY_ACTIVE_HIGH	BIT(1)

#define NWL_DSI_HSYNC_POLARITY			0x214
#define NWL_DSI_HSYNC_POLARITY_ACTIVE_LOW	0
#define NWL_DSI_HSYNC_POLARITY_ACTIVE_HIGH	BIT(1)

#define NWL_DSI_VIDEO_MODE			0x218
#define NWL_DSI_HFP				0x21c
#define NWL_DSI_HBP				0x220
#define NWL_DSI_HSA				0x224
#define NWL_DSI_ENABLE_MULT_PKTS		0x228
#define NWL_DSI_VBP				0x22c
#define NWL_DSI_VFP				0x230
#define NWL_DSI_BLLP_MODE			0x234
#define NWL_DSI_USE_NULL_PKT_BLLP		0x238
#define NWL_DSI_VACTIVE				0x23c
#define NWL_DSI_VC				0x240

/* DSI APB PKT control */
#define NWL_DSI_TX_PAYLOAD			0x280
#define NWL_DSI_PKT_CONTROL			0x284
#define NWL_DSI_SEND_PACKET			0x288
#define NWL_DSI_PKT_STATUS			0x28c
#define NWL_DSI_PKT_FIFO_WR_LEVEL		0x290
#define NWL_DSI_PKT_FIFO_RD_LEVEL		0x294
#define NWL_DSI_RX_PAYLOAD			0x298
#define NWL_DSI_RX_PKT_HEADER			0x29c

/* DSI IRQ handling */
#define NWL_DSI_IRQ_STATUS			0x2a0
#define NWL_DSI_SM_NOT_IDLE			BIT(0)
#define NWL_DSI_TX_PKT_DONE			BIT(1)
#define NWL_DSI_DPHY_DIRECTION			BIT(2)
#define NWL_DSI_TX_FIFO_OVFLW			BIT(3)
#define NWL_DSI_TX_FIFO_UDFLW			BIT(4)
#define NWL_DSI_RX_FIFO_OVFLW			BIT(5)
#define NWL_DSI_RX_FIFO_UDFLW			BIT(6)
#define NWL_DSI_RX_PKT_HDR_RCVD			BIT(7)
#define NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD	BIT(8)
#define NWL_DSI_LP_RX_TIMEOUT			BIT(30)

#define NWL_DSI_IRQ_STATUS2			0x2a4
#define NWL_DSI_SINGLE_BIT_ECC_ERR		BIT(0)
#define NWL_DSI_MULTI_BIT_ECC_ERR		BIT(1)
#define NWL_DSI_CRC_ERR				BIT(2)

#define NWL_DSI_IRQ_MASK			0x2a8
#define NWL_DSI_SM_NOT_IDLE_MASK		BIT(0)
#define NWL_DSI_TX_PKT_DONE_MASK		BIT(1)
#define NWL_DSI_DPHY_DIRECTION_MASK		BIT(2)
#define NWL_DSI_TX_FIFO_OVFLW_MASK		BIT(3)
#define NWL_DSI_TX_FIFO_UDFLW_MASK		BIT(4)
#define NWL_DSI_RX_FIFO_OVFLW_MASK		BIT(5)
#define NWL_DSI_RX_FIFO_UDFLW_MASK		BIT(6)
#define NWL_DSI_RX_PKT_HDR_RCVD_MASK		BIT(7)
#define NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD_MASK	BIT(8)
#define NWL_DSI_LP_RX_TIMEOUT_MASK		BIT(30)

#define NWL_DSI_IRQ_MASK2			0x2ac
#define NWL_DSI_SINGLE_BIT_ECC_ERR_MASK		BIT(0)
#define NWL_DSI_MULTI_BIT_ECC_ERR_MASK		BIT(1)
#define NWL_DSI_CRC_ERR_MASK			BIT(2)

/*
 * PKT_CONTROL format:
 * [15: 0] - word count
 * [17:16] - virtual channel
 * [23:18] - data type
 * [24]	   - LP or HS select (0 - LP, 1 - HS)
 * [25]	   - perform BTA after packet is sent
 * [26]	   - perform BTA only, no packet tx
 */
#define NWL_DSI_WC(x)		FIELD_PREP(GENMASK(15, 0), (x))
#define NWL_DSI_TX_VC(x)	FIELD_PREP(GENMASK(17, 16), (x))
#define NWL_DSI_TX_DT(x)	FIELD_PREP(GENMASK(23, 18), (x))
#define NWL_DSI_HS_SEL(x)	FIELD_PREP(GENMASK(24, 24), (x))
#define NWL_DSI_BTA_TX(x)	FIELD_PREP(GENMASK(25, 25), (x))
#define NWL_DSI_BTA_NO_TX(x)	FIELD_PREP(GENMASK(26, 26), (x))

/*
 * RX_PKT_HEADER format:
 * [15: 0] - word count
 * [21:16] - data type
 * [23:22] - virtual channel
 */
#define NWL_DSI_RX_DT(x)	FIELD_GET(GENMASK(21, 16), (x))
#define NWL_DSI_RX_VC(x)	FIELD_GET(GENMASK(23, 22), (x))

/* DSI Video mode */
#define NWL_DSI_VM_BURST_MODE_WITH_SYNC_PULSES		0
#define NWL_DSI_VM_NON_BURST_MODE_WITH_SYNC_EVENTS	BIT(0)
#define NWL_DSI_VM_BURST_MODE				BIT(1)

/* * DPI color coding */
#define NWL_DSI_DPI_16_BIT_565_PACKED	0
#define NWL_DSI_DPI_16_BIT_565_ALIGNED	1
#define NWL_DSI_DPI_16_BIT_565_SHIFTED	2
#define NWL_DSI_DPI_18_BIT_PACKED	3
#define NWL_DSI_DPI_18_BIT_ALIGNED	4
#define NWL_DSI_DPI_24_BIT		5

/* * DPI Pixel format */
#define NWL_DSI_PIXEL_FORMAT_16  0
#define NWL_DSI_PIXEL_FORMAT_18  BIT(0)
#define NWL_DSI_PIXEL_FORMAT_18L BIT(1)
#define NWL_DSI_PIXEL_FORMAT_24  (BIT(0) | BIT(1))

typedef enum {
	nwl_dsi_interface_unknown = 0,
	nwl_dsi0,
	nwl_dsi1,
} imx_mipi_display_interface;

int nwl_dsi_probe(struct platform_device *pdev, struct platform_device *phy_pdev);
int nwl_dsi_remove(struct platform_device *pdev);
struct drm_encoder *nwl_dsi_get_encoder(struct platform_device *pdev);
int nwl_dsi_encoder_atomic_check(struct drm_encoder *encoder,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state);
int nwl_dsi_bridge_attach(struct platform_device *pdev);
void nwl_dsi_bridge_detach(struct platform_device *pdev);
void nwl_dsi_bridge_atomic_enable(struct platform_device *pdev);
void nwl_dsi_bridge_atomic_pre_enable(struct platform_device *pdev);
void nwl_dsi_bridge_atomic_post_disable(struct platform_device *pdev);
void nwl_dsi_bridge_mode_set(struct platform_device *pdev,
			const struct drm_display_mode *adjusted_mode);
int nwl_dsi_bridge_atomic_check(struct platform_device *pdev,
				       struct drm_crtc_state *crtc_state);
enum drm_mode_status nwl_dsi_bridge_mode_valid(struct platform_device *pdev,
			  const struct drm_display_mode *mode);
irqreturn_t nwl_dsi_irq_handler(struct platform_device* pdev);
struct regmap* nwl_dsi_get_regmap(struct platform_device* pdev);
void nwl_dsi_adjust_clk_drop_lvl(struct platform_device* pdev, unsigned int lvl);
void nwl_dsi_dump(struct platform_device* pdev);

int mixel_dphy_validate(struct platform_device *pdev, enum phy_mode mode, int submode,
					union phy_configure_opts *opts);
int mixel_dphy_exit(struct platform_device *pdev);
int mixel_dphy_power_off(struct platform_device *pdev);
int mixel_dphy_init(struct platform_device *pdev);
int mixel_dphy_configure(struct platform_device *pdev, union phy_configure_opts *opts);
int mixel_dphy_power_on(struct platform_device *pdev);
int mixel_dphy_probe(struct platform_device* pdev, struct regmap* mipi_map);
int mixel_dphy_remove(struct platform_device* pdev);
void mixel_dphy_dump(struct platform_device* pdev);

#endif /* __NWL_DSI_H__ */
