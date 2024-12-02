/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2017, 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef __MIPI_DSI_NORTHWEST_REGS_H
#define __MIPI_DSI_NORTHWEST_REGS_H

/* ----------------------------	register offsets --------------------------- */

/* dphy */
#define DPHY_PD_DPHY			0x300u
#define DPHY_M_PRG_HS_PREPARE	0x304u
#define DPHY_MC_PRG_HS_PREPARE	0x308u
#define DPHY_M_PRG_HS_ZERO		0x30Cu
#define DPHY_MC_PRG_HS_ZERO		0x310u
#define DPHY_M_PRG_HS_TRAIL		0x314u
#define DPHY_MC_PRG_HS_TRAIL	0x318u

#define DPHY_PD_PLL			0x31Cu
#define DPHY_TST			0x320u
#define DPHY_CN				0x324u
#define DPHY_CM				0x328u
#define DPHY_CO				0x32Cu
#define DPHY_LOCK			0x330u
#define DPHY_LOCK_BYP		0x334u

#if defined(CPU_IMX8QXP)
#define DPHY_AUTO_PD_EN		0x338u
#define DPHY_RXLPRP			0x33Cu
#define DPHY_RXCDRP			0x340u
#else
#define DPHY_TX_RCAL		0x338u
#define DPHY_AUTO_PD_EN		0x33Cu
#define DPHY_RXLPRP			0x340u
#define DPHY_RXCDRP			0x344u
#endif

/* host */
#define HOST_CFG_NUM_LANES			0x0u
#define HOST_CFG_NONCONTINUOUS_CLK	0x4u
#define HOST_CFG_T_PRE				0x8u
#define HOST_CFG_T_POST				0xCu
#define HOST_CFG_TX_GAP				0x10
#define HOST_CFG_AUTOINSERT_EOTP	0x14u
#define HOST_CFG_EXTRA_CMDS_AFTER_EOTP	0x18u
#define HOST_CFG_HTX_TO_COUNT		0x1Cu
#define HOST_CFG_LRX_H_TO_COUNT		0x20u
#define HOST_CFG_BTA_H_TO_COUNT		0x24u
#define HOST_CFG_TWAKEUP			0x28u
#define HOST_CFG_STATUS_OUT			0x2Cu
#define HOST_RX_ERROR_STATUS		0x30u

/* dpi */
#define DPI_PIXEL_PAYLOAD_SIZE		0x200u
#define DPI_PIXEL_FIFO_SEND_LEVEL	0x204u
#define DPI_INTERFACE_COLOR_CODING	0x208u
#define DPI_PIXEL_FORMAT			0x20Cu
#define DPI_VSYNC_POLARITY			0x210u
#define DPI_HSYNC_POLARITY			0x214u
#define DPI_VIDEO_MODE				0x218u
#define DPI_VIDEO_MODE_NONBURST_SYNC_PULSES        0x0U
#define DPI_VIDEO_MODE_NONBURST_SYNC_EVENTS        0x1U
#define DPI_VIDEO_MODE_BURST                       0x2U

#define DPI_HFP						0x21Cu
#define DPI_HBP						0x220u
#define DPI_HSA						0x224u
#define DPI_ENABLE_MULT_PKTS		0x228u
#define DPI_VBP						0x22Cu
#define DPI_VFP						0x230u
#define DPI_BLLP_MODE				0x234u
#define DPI_USE_NULL_PKT_BLLP		0x238u
#define DPI_VACTIVE					0x23Cu
#define DPI_VC						0x240u

/* apb pkt */
#define HOST_TX_PAYLOAD			0x280u

#define HOST_PKT_CONTROL		0x284u
#define HOST_PKT_CONTROL_WC(x)		(((x) & 0xffff) << 0)
#define HOST_PKT_CONTROL_VC(x)		(((x) & 0x3) << 16)
#define HOST_PKT_CONTROL_DT(x)		(((x) & 0x3f) << 18)
#define HOST_PKT_CONTROL_HS_SEL(x)	(((x) & 0x1) << 24)
#define HOST_PKT_CONTROL_BTA_TX(x)	(((x) & 0x1) << 25)
#define HOST_PKT_CONTROL_BTA_NO_TX(x)	(((x) & 0x1) << 26)

#define HOST_SEND_PACKET		0x288u
#define HOST_PKT_STATUS			0x28Cu
#define HOST_PKT_FIFO_WR_LEVEL	0x290u
#define HOST_PKT_FIFO_RD_LEVEL	0x294u
#define HOST_PKT_RX_PAYLOAD		0x298u

#define HOST_PKT_RX_PKT_HEADER	0x29Cu
#define HOST_PKT_RX_PKT_HEADER_WC(x)	(((x) & 0xffff) << 0)
#define HOST_PKT_RX_PKT_HEADER_DT(x)	(((x) & 0x3f) << 16)
#define HOST_PKT_RX_PKT_HEADER_VC(x)	(((x) & 0x3) << 22)

#define HOST_IRQ_STATUS			0x2A0u
#define HOST_IRQ_STATUS_SM_NOT_IDLE				(1 << 0)
#define HOST_IRQ_STATUS_TX_PKT_DONE				(1 << 1)
#define HOST_IRQ_STATUS_DPHY_DIRECTION			(1 << 2)
#define HOST_IRQ_STATUS_TX_FIFO_OVFLW			(1 << 3)
#define HOST_IRQ_STATUS_TX_FIFO_UDFLW			(1 << 4)
#define HOST_IRQ_STATUS_RX_FIFO_OVFLW			(1 << 5)
#define HOST_IRQ_STATUS_RX_FIFO_UDFLW			(1 << 6)
#define HOST_IRQ_STATUS_RX_PKT_HDR_RCVD			(1 << 7)
#define HOST_IRQ_STATUS_RX_PKT_PAYLOAD_DATA_RCVD	(1 << 8)
#define HOST_IRQ_STATUS_HOST_BTA_TIMEOUT		(1 << 29)
#define HOST_IRQ_STATUS_LP_RX_TIMEOUT			(1 << 30)
#define HOST_IRQ_STATUS_HS_TX_TIMEOUT			(1 << 31)

#define HOST_IRQ_STATUS2		0x2A4u
#define HOST_IRQ_STATUS2_SINGLE_BIT_ECC_ERR		(1 << 0)
#define HOST_IRQ_STATUS2_MULTI_BIT_ECC_ERR		(1 << 1)
#define HOST_IRQ_STATUS2_CRC_ERR			(1 << 2)

#define HOST_IRQ_MASK			0x2A8u
#define HOST_IRQ_MASK_SM_NOT_IDLE_MASK			(1 << 0)
#define HOST_IRQ_MASK_TX_PKT_DONE_MASK			(1 << 1)
#define HOST_IRQ_MASK_DPHY_DIRECTION_MASK		(1 << 2)
#define HOST_IRQ_MASK_TX_FIFO_OVFLW_MASK		(1 << 3)
#define HOST_IRQ_MASK_TX_FIFO_UDFLW_MASK		(1 << 4)
#define HOST_IRQ_MASK_RX_FIFO_OVFLW_MASK		(1 << 5)
#define HOST_IRQ_MASK_RX_FIFO_UDFLW_MASK		(1 << 6)
#define HOST_IRQ_MASK_RX_PKT_HDR_RCVD_MASK		(1 << 7)
#define HOST_IRQ_MASK_RX_PKT_PAYLOAD_DATA_RCVD_MASK	(1 << 8)
#define HOST_IRQ_MASK_HOST_BTA_TIMEOUT_MASK		(1 << 29)
#define HOST_IRQ_MASK_LP_RX_TIMEOUT_MASK		(1 << 30)
#define HOST_IRQ_MASK_HS_TX_TIMEOUT_MASK		(1 << 31)

#define HOST_IRQ_MASK2			0x2ACu
#define HOST_IRQ_MASK2_SINGLE_BIT_ECC_ERR_MASK		(1 << 0)
#define HOST_IRQ_MASK2_MULTI_BIT_ECC_ERR_MASK		(1 << 1)
#define HOST_IRQ_MASK2_CRC_ERR_MASK			(1 << 2)

#endif
