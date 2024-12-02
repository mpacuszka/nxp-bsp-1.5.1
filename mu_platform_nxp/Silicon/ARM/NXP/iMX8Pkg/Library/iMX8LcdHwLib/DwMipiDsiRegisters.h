/*
 * Copyright 2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef DW_MIPI_DSI_IO_H_
#define DW_MIPI_DSI_IO_H_


/* Memory map for Synopsys dw mipi-dsi module */

#define HWVER_131						0x31333100	/* IP version 1.31 */

#define IMX_DSI_VERSION					0x00

#define IMX_DSI_PWR_UP					0x04
	#define RESET						0
	#define POWERUP						1

#define IMX_DSI_CLKMGR_CFG				0x08
#define TO_CLK_DIVISION(div)			(((div) & 0xff) << 8)
#define TX_ESC_CLK_DIVISION(div)		((div) & 0xff)

#define IMX_DSI_DPI_VCID				0x0C
	#define DPI_VCID(vcid)				((vcid) & 0x3)

#define IMX_DSI_DPI_COLOR_CODING		0x10
	#define LOOSELY18_EN				(1 << 8)
	#define DPI_COLOR_CODING_16BIT_1	0x0
	#define DPI_COLOR_CODING_16BIT_2	0x1
	#define DPI_COLOR_CODING_16BIT_3	0x2
	#define DPI_COLOR_CODING_18BIT_1	0x3
	#define DPI_COLOR_CODING_18BIT_2	0x4
	#define DPI_COLOR_CODING_24BIT		0x5

#define IMX_DSI_DPI_CFG_POL				0x14
	#define COLORM_ACTIVE_LOW			(1 << 4)
	#define SHUTD_ACTIVE_LOW			(1 << 3)
	#define HSYNC_ACTIVE_LOW			(1 << 2)
	#define VSYNC_ACTIVE_LOW			(1 << 1)
	#define DATAEN_ACTIVE_LOW			(1 << 0)

#define IMX_DSI_DPI_LP_CMD_TIM			0x18
	#define OUTVACT_LPCMD_TIME(p)		(((p) & 0xff) << 16)
	#define INVACT_LPCMD_TIME(p)		((p) & 0xff)

#define IMX_DSI_DBI_VCID				0x1C
#define DSI_DBI_CFG						0x20
#define DSI_DBI_PARTITIONING_EN			0x24
#define DSI_DBI_CMDSIZE					0x28

#define IMX_DSI_PCKHDL_CFG				0x2C
	#define CRC_RX_EN					0x10
	#define ECC_RX_EN					0x8
	#define BTA_EN						0x4
	#define EOTP_RX_EN					0x2
	#define EOTP_TX_EN					0x1

#define IMX_DSI_GEN_VCID				0x30

#define IMX_DSI_MODE_CFG				0x34
	#define ENABLE_VIDEO_MODE			0
	#define ENABLE_CMD_MODE				1

#define IMX_DSI_VID_MODE_CFG			0x38
	#define ENABLE_LOW_POWER_MASK		(0x3F << 8)
	#define VID_MODE_VPG_ENABLE			(1 << 16)
	#define VID_MODE_TYPE_NON_BURST_SYNC_PULSES	0x0
	#define VID_MODE_TYPE_NON_BURST_SYNC_EVENTS	0x1
	#define VID_MODE_TYPE_BURST			0x2
	#define VID_MODE_TYPE_MASK			0x3

#define IMX_DSI_VID_PKT_SIZE			0x3C
	#define VID_PKT_SIZE(p)				((p) & 0x3fff)

#define IMX_DSI_VID_NUM_CHUNKS			0x40
	#define VID_NUM_CHUNKS(c)			((c) & 0x1fff)

#define IMX_DSI_VID_NULL_SIZE			0x44
	#define VID_NULL_SIZE(b)			((b) & 0x1fff)

#define IMX_DSI_VID_HSA_TIME			0x48
#define IMX_DSI_VID_HBP_TIME			0x4C
#define IMX_DSI_VID_HLINE_TIME			0x50
#define IMX_DSI_VID_VSA_LINES			0x54
#define IMX_DSI_VID_VBP_LINES			0x58
#define IMX_DSI_VID_VFP_LINES			0x5C
#define IMX_DSI_VID_VACTIVE_LINES		0x60
#define IMX_DSI_EDPI_CMD_SIZE			0x64

#define IMX_DSI_CMD_MODE_CFG			0x68
	#define MAX_RD_PKT_SIZE_LP			(1 << 24)
	#define DCS_LW_TX_LP				(1 << 19)
	#define DCS_SR_0P_TX_LP				(1 << 18)
	#define DCS_SW_1P_TX_LP				(1 << 17)
	#define DCS_SW_0P_TX_LP				(1 << 16)
	#define GEN_LW_TX_LP				(1 << 14)
	#define GEN_SR_2P_TX_LP				(1 << 13)
	#define GEN_SR_1P_TX_LP				(1 << 12)
	#define GEN_SR_0P_TX_LP				(1 << 11)
	#define GEN_SW_2P_TX_LP				(1 << 10)
	#define GEN_SW_1P_TX_LP				(1 << 9)
	#define GEN_SW_0P_TX_LP				(1 << 8)
	#define ACK_RQST_EN					(1 << 1)
	#define TEAR_FX_EN					(1 << 0)

#define CMD_MODE_ALL_LP			(MAX_RD_PKT_SIZE_LP | \
					 DCS_LW_TX_LP | \
					 DCS_SR_0P_TX_LP | \
					 DCS_SW_1P_TX_LP | \
					 DCS_SW_0P_TX_LP | \
					 GEN_LW_TX_LP | \
					 GEN_SR_2P_TX_LP | \
					 GEN_SR_1P_TX_LP | \
					 GEN_SR_0P_TX_LP | \
					 GEN_SW_2P_TX_LP | \
					 GEN_SW_1P_TX_LP | \
					 GEN_SW_0P_TX_LP)

#define IMX_DSI_GEN_HDR					0x6c
#define IMX_DSI_GEN_PLD_DATA			0x70

#define IMX_DSI_CMD_PKT_STATUS			0x74
	#define GEN_RD_CMD_BUSY				(1 << 6)
	#define GEN_PLD_R_FULL				(1 << 5)
	#define GEN_PLD_R_EMPTY				(1 << 4)
	#define GEN_PLD_W_FULL				(1 << 3)
	#define GEN_PLD_W_EMPTY				(1 << 2)
	#define GEN_CMD_FULL				(1 << 1)
	#define GEN_CMD_EMPTY				(1 << 0)

#define IMX_DSI_TO_CNT_CFG			0x78
	#define HSTX_TO_CNT(p)			(((p) & 0xffff) << 16)
	#define LPRX_TO_CNT(p)			((p) & 0xffff)

#define IMX_DSI_HS_RD_TO_CNT		0x7C
#define IMX_DSI_LP_RD_TO_CNT		0x80
#define IMX_DSI_HS_WR_TO_CNT		0x84
#define IMX_DSI_LP_WR_TO_CNT		0x88
#define IMX_DSI_BTA_TO_CNT			0x8c

#define IMX_DSI_LPCLK_CTRL			0x94
	#define AUTO_CLKLANE_CTRL		0x2
	#define PHY_TXREQUESTCLKHS		0x1

#define IMX_DSI_PHY_TMR_LPCLK_CFG	0x98
	#define PHY_CLKHS2LP_TIME(lbcc)	(((lbcc) & 0x3ff) << 16)
	#define PHY_CLKLP2HS_TIME(lbcc)	((lbcc) & 0x3ff)

#define IMX_DSI_PHY_TMR_CFG			0x9C
	#define PHY_HS2LP_TIME(lbcc)	(((lbcc) & 0xff) << 24)
	#define PHY_LP2HS_TIME(lbcc)	(((lbcc) & 0xff) << 16)
	#define MAX_RD_TIME(lbcc)		((lbcc) & 0x7fff)
	#define PHY_HS2LP_TIME_V131(lbcc)	(((lbcc) & 0x3ff) << 16)
	#define PHY_LP2HS_TIME_V131(lbcc)	((lbcc) & 0x3ff)

#define IMX_DSI_PHY_RSTZ			0xA0
	#define PHY_FORCEPLL_MASK		(0x1 << 3)
	#define PHY_ENABLECLK_MASK		(0x1 << 2)
	#define PHY_RSTZ_MASK			(0x1 << 1)
	#define PHY_SHUTDOWNZ_MASK		(0x1 << 0)

#define IMX_DSI_PHY_IF_CFG			0xA4
	#define PHY_STOP_WAIT_TIME(cycle)	(((cycle) & 0xff) << 8)
	#define N_LANES(n)			(((n) - 1) & 0x3)

#define IMX_DSI_PHY_ULPS_CTRL		0xA8
#define IMX_DSI_PHY_TX_TRIGGERS		0xAC

#define IMX_DSI_PHY_STATUS			0xB0
	#define PHY_STOP_STATE_CLK_LANE_MASK	(0x1 << 2)
	#define PHY_LOCK_MASK					(0x1 << 0)

#define IMX_DSI_PHY_TST_CTRL0		0xB4
	#define PHY_TESTCLK				2
	#define PHY_UNTESTCLK			0
	#define PHY_TESTCLR				1
	#define PHY_UNTESTCLR			0

#define IMX_DSI_PHY_TST_CTRL1		0xB8
	#define PHY_TESTEN				(1 << 16)
	#define PHY_UNTESTEN			0
	#define PHY_TESTDOUT(n)			(((n) & 0xff) << 8)
	#define PHY_TESTDIN(n)			((n) & 0xff)

#define IMX_DSI_INT_ST0				0xBC
#define IMX_DSI_INT_ST1				0xC0
#define IMX_DSI_INT_MSK0			0xC4
#define IMX_DSI_INT_MSK1			0xC8

#define IMX_DSI_PHY_TMR_RD_CFG		0xF4
	#define MAX_RD_TIME_V131(lbcc)	((lbcc) & 0x7fff)

#define PHY_STATUS_TIMEOUT_US		10000
#define CMD_PKT_STATUS_TIMEOUT_US	20000

#endif /* DW_MIPI_DSI_IO_H_ */
