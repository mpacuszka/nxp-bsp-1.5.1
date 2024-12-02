/*
* Copyright 2022 NXP
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the
*   names of its contributors may be used to endorse or promote products
*   derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include "iMX8.h"
#include "clock.h"
#include "dw_hdmi.h"

#define HDMI_DEBUG_LEVEL DEBUG_ERROR

#define HDMI_PAVI_BASE       0x32FC4000
#define HDMI_BLKCTL_BASE     0x32FC0000

#define HDMI_HTX_PVI_CTRL         0x0
#define HDMI_HTX_PVI_IRQ_MASK     0x04
#define HDMI_HTX_TMG_GEN_DISP_LRC 0x10
#define HDMI_HTX_TMG_GEN_DE_ULC   0x14
#define HDMI_HTX_TMG_GEN_DE_LRC   0x18
#define HDMI_HTX_TMG_GEN_HSYNC    0x1c
#define HDMI_HTX_TMG_GEN_VSYNC    0x20
#define HDMI_HTX_TMG_GEN_IRQ0     0x24
#define HDMI_HTX_TMG_GEN_IRQ1     0x28
#define HDMI_HTX_TMG_GEN_IRQ2     0x2c
#define HDMI_HTX_TMG_GEN_IRQ3     0x30
#define HDMI_HTX_TMG_GEN_CFG      0x40

#define HDMI_HTX_PAI_CTRL         0x800
#define HDMI_HTX_PAI_CTRL_EXT     0x804
#define HDMI_HTX_PAI_FIELD_CTRL   0x808

#define HTX_PAI_CTRL_ENABLE  1

static uint32_t hdmi_read_phy_reg(uint32_t offset)
{
	return 	*(volatile uint32_t *)((uint64_t)(HDMI_BLKCTL_BASE + offset));
}

static void hdmi_write_phy_reg(uint32_t value, uint32_t offset)
{
	*(volatile uint32_t *)((uint64_t)(HDMI_BLKCTL_BASE + offset)) = value;
}


static void hdmi_write_pavi_reg(uint32_t value, uint32_t offset)
{
	*(volatile uint32_t *)((uint64_t)(HDMI_PAVI_BASE + offset)) = value;
}

static void hdmi_pvi_enable(const IMX_DISPLAY_TIMING *mode)
{
	hdmi_write_pavi_reg(0x00000003, HDMI_HTX_PVI_IRQ_MASK);
	hdmi_write_pavi_reg(0x08970464, HDMI_HTX_TMG_GEN_DISP_LRC);
	hdmi_write_pavi_reg(0x00bf0029, HDMI_HTX_TMG_GEN_DE_ULC);
	hdmi_write_pavi_reg(0x083f0460, HDMI_HTX_TMG_GEN_DE_LRC);
	hdmi_write_pavi_reg(0x0897002b, HDMI_HTX_TMG_GEN_HSYNC);
	hdmi_write_pavi_reg(0x04640004, HDMI_HTX_TMG_GEN_VSYNC);
	hdmi_write_pavi_reg(0x000100ff, HDMI_HTX_TMG_GEN_IRQ0);
	hdmi_write_pavi_reg(0x000100f0, HDMI_HTX_TMG_GEN_IRQ1);
	hdmi_write_pavi_reg(0x00010315, HDMI_HTX_TMG_GEN_IRQ2);
	hdmi_write_pavi_reg(0x00010207, HDMI_HTX_TMG_GEN_IRQ3);
	hdmi_write_pavi_reg(0x84640000, HDMI_HTX_TMG_GEN_CFG);

	if ((mode->Flags & EDID_FLAGS_VSYNC_HIGH) &&
			(mode->Flags & EDID_FLAGS_HSYNC_HIGH)) {
		hdmi_write_pavi_reg(0x00377004, HDMI_HTX_PVI_CTRL);
		hdmi_write_pavi_reg(0x00377005, HDMI_HTX_PVI_CTRL);
	} else {
		hdmi_write_pavi_reg(0x00311004, HDMI_HTX_PVI_CTRL);
		hdmi_write_pavi_reg(0x00311005, HDMI_HTX_PVI_CTRL);
	}
}

static void hdmi_pvi_disable(void)
{
	hdmi_write_pavi_reg(0x0, HDMI_HTX_PVI_CTRL);
}

static void hdmi_pavi_powerup(void)
{
	uint32_t tmp;

	tmp = HDMI_TX_BLK_CTRL_RTX_CLK_CTL1;
	tmp |= (HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_GPA_CLK_EN_MASK |
			HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_VID_LINK_PIX_CLK_EN_MASK);
	HDMI_TX_BLK_CTRL_RTX_CLK_CTL1 = tmp;

	tmp = HDMI_TX_BLK_CTRL_RTX_RESET_CTL0;
	tmp |= (HDMI_TX_BLK_CTL_RTX_RESET_CTL0_PAI_RESETN_MASK |
			HDMI_TX_BLK_CTL_RTX_RESET_CTL0_VID_LINK_SLV_RESETN_MASK);
	HDMI_TX_BLK_CTRL_RTX_RESET_CTL0 = tmp;
}

static void hdmi_pavi_powerdown(void)
{
	uint32_t tmp;

	tmp = HDMI_TX_BLK_CTRL_RTX_RESET_CTL0;
	tmp &= ~(HDMI_TX_BLK_CTL_RTX_RESET_CTL0_PAI_RESETN_MASK |
			HDMI_TX_BLK_CTL_RTX_RESET_CTL0_VID_LINK_SLV_RESETN_MASK);
	HDMI_TX_BLK_CTRL_RTX_RESET_CTL0 = tmp;

	tmp = HDMI_TX_BLK_CTRL_RTX_CLK_CTL1;
	tmp &= ~(HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_GPA_CLK_EN_MASK |
			HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_VID_LINK_PIX_CLK_EN_MASK);
	HDMI_TX_BLK_CTRL_RTX_CLK_CTL1 = tmp;
}


static int dw_hdmi_imx_phy_init(const IMX_DISPLAY_TIMING *mode)
{
	uint32_t val;

	dw_hdmi_phy_reset();

	/* enable PVI */
	hdmi_pavi_powerup();
	hdmi_pvi_enable(mode);

	val = hdmi_read_phy_reg(0x200);
	/* HDMI PHY power off */
	val |= 0x8;
	hdmi_write_phy_reg(val, 0x200);
	/* HDMI PHY power on */
	val &= ~0x8;
	/* Enable CEC */
	val |= 0x2;
	hdmi_write_phy_reg(val, 0x200);

	return 0;
}

void dw_hdmi_imx_phy_disable(void)
{
	uint32_t val;

	/* disable PVI */
	hdmi_pvi_disable();
	hdmi_pavi_powerdown();

	val = hdmi_read_phy_reg(0x200);
	/* Disable CEC */
	val &= ~0x2;
	/* Power down HDMI PHY
	 * val |= 0x8;
	 * hdmi_write_phy_reg(val, 0x200);
	*/
}

/*
configure = true - configure for operation deassert reset
configure = false - assert reset, disable clock
*/
static void dw_hdmi_imx_clock_reset(bool configure)
{
	uint32_t tmp;

	if (configure) {
		/* De-assert hdmi+lcdif3 resets within HDMI_MIX block.*/
		tmp = HDMI_TX_BLK_CTRL_RTX_RESET_CTL0;
		tmp |= (HDMI_TX_BLK_CTL_RTX_RESET_CTL0_LCDIF_ASYNC_RESET_N_MASK |
				HDMI_TX_BLK_CTL_RTX_RESET_CTL0_LCDIF_APB_RESET_N_MASK |
				HDMI_TX_BLK_CTL_RTX_RESET_CTL0_FDCC_RESETN_MASK |
				HDMI_TX_BLK_CTL_RTX_RESET_CTL0_FDCC_HDMI_RESETN_MASK |
				HDMI_TX_BLK_CTL_RTX_RESET_CTL0_TX_RSTZ_MASK |
				HDMI_TX_BLK_CTL_RTX_RESET_CTL0_TX_APBRSTZ_MASK |
				HDMI_TX_BLK_CTL_RTX_RESET_CTL0_TX_PHY_PRESETN_MASK |
				HDMI_TX_BLK_CTL_RTX_RESET_CTL0_IRQ_RESETN_MASK);
		HDMI_TX_BLK_CTRL_RTX_RESET_CTL0 = tmp;

		/* Enable hdmi+lcdif3 clocks within HDMI_MIX block. */
		tmp = HDMI_TX_BLK_CTRL_RTX_CLK_CTL0;
		tmp |= (HDMI_TX_BLK_CTL_RTX_CLK_CTL0_GLOBAL_APB_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL0_GLOBAL_B_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL0_GLOBAL_XTAL24M_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL0_GLOBAL_TX_PIX_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL0_IRQS_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL0_NOC_HDMI_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL0_LCDIF_APB_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL0_LCDIF_B_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL0_LCDIF_PDI_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL0_LCDIF_PIX_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL0_LCDIF_SPU_CLK_EN_MASK);
		HDMI_TX_BLK_CTRL_RTX_CLK_CTL0 = tmp;

		tmp = HDMI_TX_BLK_CTRL_RTX_CLK_CTL1;
		tmp |= (HDMI_TX_BLK_CTL_RTX_CLK_CTL1_FDCC_REF_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL1_HTX_PIPE_CLK_SEL_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_HPI_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_APB_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_CEC_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_PIXEL_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_SFR_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_SKP_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_PREP_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_PHY_APB_CLK_EN_MASK |
				HDMI_TX_BLK_CTL_RTX_CLK_CTL1_TX_PHY_INT_CLK_EN_MASK);
		HDMI_TX_BLK_CTRL_RTX_CLK_CTL1 = tmp;
	} else {
		HDMI_TX_BLK_CTRL_RTX_RESET_CTL0 = 0;
		HDMI_TX_BLK_CTRL_RTX_CLK_CTL0 = 0;
		HDMI_TX_BLK_CTRL_RTX_CLK_CTL1 = 0;
		
	}
}

void dw_hdmi_imx_init(void)
{
	HdmiDisplayClockConfig();
	dw_hdmi_imx_clock_reset(true);

	dw_hdmi_init(&dw_hdmi_imx_phy_init);
	dw_hdmi_phy_init();

}

void dw_hdmi_imx_deinit(void)
{
	dw_hdmi_imx_clock_reset(false);
}

/**
  Dump LDB registers.
**/
void ClkResetDump (void)
{
  DEBUG ((HDMI_DEBUG_LEVEL, "------------------------dw_hdmi clk reset------------\n"));
  DEBUG ((HDMI_DEBUG_LEVEL, "HDMI_TX_BLK_CTRL_RTX_RESET_CTL0 = 0x%08X\n", HDMI_TX_BLK_CTRL_RTX_RESET_CTL0));
  DEBUG ((HDMI_DEBUG_LEVEL, "HDMI_TX_BLK_CTRL_RTX_CLK_CTL0   = 0x%08X\n", HDMI_TX_BLK_CTRL_RTX_CLK_CTL0));
  DEBUG ((HDMI_DEBUG_LEVEL, "HDMI_TX_BLK_CTRL_RTX_CLK_CTL1   = 0x%08X\n", HDMI_TX_BLK_CTRL_RTX_CLK_CTL1));
  DEBUG ((HDMI_DEBUG_LEVEL, "TX_BLK_CONTROL0 (0x200)         = 0x%08X\n", HDMI_TX_BLK_CTRL_TX_CONTROL0));
}

void PaviPhyDump (void)
{
  DEBUG ((HDMI_DEBUG_LEVEL, "------------------------dw_hdmi pvi,phy------------\n"));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_PVI_CTRL         = 0x%08X\n", HTX_PVI_HTX_PVI_CTRL));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_PVI_IRQ_MASK     = 0x%08X\n", HTX_PVI_HTX_PVI_IRQ_MASK));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_TMG_GEN_DISP_LRC = 0x%08X\n", HTX_PVI_HTX_TMG_GEN_DISP_LRC));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_TMG_GEN_DE_ULC   = 0x%08X\n", HTX_PVI_HTX_TMG_GEN_DE_ULC));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_TMG_GEN_DE_LRC   = 0x%08X\n", HTX_PVI_HTX_TMG_GEN_DE_LRC));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_TMG_GEN_HSYNC    = 0x%08X\n", HTX_PVI_HTX_TMG_GEN_HSYNC));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_TMG_GEN_VSYNC    = 0x%08X\n", HTX_PVI_HTX_TMG_GEN_VSYNC));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_TMG_GEN_IRQ0     = 0x%08X\n", HTX_PVI_HTX_TMG_GEN_IRQ0));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_TMG_GEN_IRQ1     = 0x%08X\n", HTX_PVI_HTX_TMG_GEN_IRQ1));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_TMG_GEN_IRQ2     = 0x%08X\n", HTX_PVI_HTX_TMG_GEN_IRQ2));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_TMG_GEN_IRQ3     = 0x%08X\n", HTX_PVI_HTX_TMG_GEN_IRQ3));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_TMG_GEN_CFG      = 0x%08X\n", HTX_PVI_HTX_TMG_GEN_CFG));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_PAI_CTRL         = 0x%08X\n", HTX_PAI_HTX_PAI_CTRL));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_PAI_CTRL_EXT     = 0x%08X\n", HTX_PAI_HTX_PAI_CTRL_EXT));
  DEBUG ((HDMI_DEBUG_LEVEL, "HTX_PAI_FIELD_CTRL   = 0x%08X\n", HTX_PAI_HTX_PAI_FIELD_CTRL));
}

