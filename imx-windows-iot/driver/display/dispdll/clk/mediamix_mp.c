/*
 * Copyright 2022 NXP
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

#include <ntddk.h>
#include <stddef.h>
#include <linux/io.h>
#include "mediamix_mp.h"

#define IMX_MEDIAMIX_DEBUG

#ifdef IMX_MEDIAMIX_DEBUG
#define MEDIAMIX_DEBUG(...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, __VA_ARGS__)
#else
#define MEDIAMIX_DEBUG(...)
#endif

#define mediamix_write(val, addr)  (*(volatile uint32_t __force *)(addr) = (uint32_t)(val))
#define mediamix_read(addr)        (*(volatile uint32_t __force *)(addr))

/**
 * Function to initialize reset related to lcdif1 + mipi-dsi module in MEDIAMIX GPR.
 * @param regptr Pointer to the mapped MEDIAMIX GPR base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void mediamix_dsi_reset(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    if (enable) {
        /* Set lcdif1 + dsi blk reset */
        reg = mediamix_read(regptr + MEDIAMIX_SFT_RSTN);
        reg &= ~(SFT_EN_MIPI_DSI_CLKREF_RESETN_MASK | SFT_EN_MIPI_DSI_PCLK_RESETN_MASK |
            SFT_EN_LCDIF_PIXEL_CLK_RESETN_MASK | SFT_EN_LCDIF_APB_CLK_RESETN_MASK |
            SFT_EN_LCDIF_AXI_CLK_RESETN_MASK);
        mediamix_write(reg, regptr + MEDIAMIX_SFT_RSTN);

        /* Lanes reset */
        reg = mediamix_read(regptr + GPR_MIPI_RESET_DIV);
        reg &= GPR_MIPI_M_RESETN_MASK;
        mediamix_write(reg, regptr + GPR_MIPI_RESET_DIV);
    }
    else {
        /* Release lcdif1 + dsi blk reset */
        reg = mediamix_read(regptr + MEDIAMIX_SFT_RSTN);
        reg |= (SFT_EN_MIPI_DSI_CLKREF_RESETN_MASK | SFT_EN_MIPI_DSI_PCLK_RESETN_MASK |
            SFT_EN_LCDIF_PIXEL_CLK_RESETN_MASK | SFT_EN_LCDIF_APB_CLK_RESETN_MASK |
            SFT_EN_LCDIF_AXI_CLK_RESETN_MASK | SFT_EN_BUS_BLK_CLK_RESETN_MASK);
        mediamix_write(reg, regptr + MEDIAMIX_SFT_RSTN);

        /* Release lanes reset */
        reg = mediamix_read(regptr + GPR_MIPI_RESET_DIV);
        reg |= GPR_MIPI_M_RESETN_MASK;
        mediamix_write(reg, regptr + GPR_MIPI_RESET_DIV);
    }
}

/**
 * Function to initialize reset related to lcdif2 +  LVDS module in MEDIAMIX.
 * @param regptr Pointer to the mapped MEDIAMIX base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void mediamix_lvds_reset(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    if (enable) {
        reg = mediamix_read(regptr + MEDIAMIX_SFT_RSTN);
        reg &= ~(SFT_EN_LCDIF2_PIXEL_CLK_RESETN_MASK | SFT_EN_LCDIF2_APB_CLK_RESETN_MASK |
            SFT_EN_LCDIF2_AXI_CLK_RESETN_MASK);
        mediamix_write(reg, regptr + MEDIAMIX_SFT_RSTN);
    }
    else {
        reg = mediamix_read(regptr + MEDIAMIX_SFT_RSTN);
        reg |= (SFT_EN_LCDIF2_PIXEL_CLK_RESETN_MASK | SFT_EN_LCDIF2_APB_CLK_RESETN_MASK |
            SFT_EN_LCDIF2_AXI_CLK_RESETN_MASK | SFT_EN_BUS_BLK_CLK_RESETN_MASK);
        mediamix_write(reg, regptr + MEDIAMIX_SFT_RSTN);
    }
}

/**
 * Function to read and log the content of DISPMIX GPR registers.
 * @param regptr Pointer to the mapped DISPMIX GPR register address space.
 */
void mediamix_dump_reg(volatile uint8_t __iomem *regptr)
{
    NT_ASSERT(regptr);
    MEDIAMIX_DEBUG("MEDIAMIX_SFT_RSTN: offset=0x%x val=0x%x\n",
        MEDIAMIX_SFT_RSTN, mediamix_read(regptr + MEDIAMIX_SFT_RSTN));
    MEDIAMIX_DEBUG("MEDIAMIX_CLK_EN: offset=0x%x val=0x%x\n",
        MEDIAMIX_CLK_EN, mediamix_read(regptr + MEDIAMIX_CLK_EN));
    MEDIAMIX_DEBUG("GPR_MIPI_RESET_DIV: offset=0x%x val=0x%x\n",
        GPR_MIPI_RESET_DIV, mediamix_read(regptr + GPR_MIPI_RESET_DIV));
}

/**
 * Function to initialize reset related to hdmi tx in HDMIMIX BLK CTRL.
 * @param regptr Pointer to the mapped HDMIMIX base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void hdmimix_hdmi_reset(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = mediamix_read(regptr + HDMI_RESET_CTL0);
    if (enable) {
        reg &= ~(HDMI_RESET_CTL0_FDCC_RESETN_MASK | HDMI_RESET_CTL0_FDCC_HDMI_RESETN_MASK |
            HDMI_RESET_CTL0_TX_RSTZ_MASK | HDMI_RESET_CTL0_TX_APBRSTZ_MASK);
    }
    else {
        reg |= (HDMI_RESET_CTL0_FDCC_RESETN_MASK | HDMI_RESET_CTL0_FDCC_HDMI_RESETN_MASK |
            HDMI_RESET_CTL0_TX_RSTZ_MASK | HDMI_RESET_CTL0_TX_APBRSTZ_MASK);
    }
    mediamix_write(reg, regptr + HDMI_RESET_CTL0);
}

/**
 * Function to initialize reset related to hdmi pai in HDMIMIX BLK CTRL.
 * @param regptr Pointer to the mapped HDMIMIX base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void hdmimix_pai_reset(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = mediamix_read(regptr + HDMI_RESET_CTL0);
    if (enable) {
        reg &= ~(HDMI_RESET_CTL0_PAI_RESETN_MASK);
    }
    else {
        reg |= HDMI_RESET_CTL0_PAI_RESETN_MASK;
    }
    mediamix_write(reg, regptr + HDMI_RESET_CTL0);
}

/**
 * Function to initialize reset related to hdmi pvi in HDMIMIX BLK CTRL.
 * @param regptr Pointer to the mapped HDMIMIX base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void hdmimix_pvi_reset(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = mediamix_read(regptr + HDMI_RESET_CTL0);
    if (enable) {
        reg &= ~(HDMI_RESET_CTL0_VID_LINK_SLV_RESETN_MASK);
    }
    else {
        reg |= HDMI_RESET_CTL0_VID_LINK_SLV_RESETN_MASK;
    }
    mediamix_write(reg, regptr + HDMI_RESET_CTL0);
}

/**
 * Function to initialize reset related to hdmi phy in HDMIMIX BLK CTRL.
 * @param regptr Pointer to the mapped HDMIMIX base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void hdmimix_phy_reset(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = mediamix_read(regptr + HDMI_RESET_CTL0);
    if (enable) {
        reg &= ~(HDMI_RESET_CTL0_TX_PHY_PRESETN_MASK);
    }
    else {
        reg |= HDMI_RESET_CTL0_TX_PHY_PRESETN_MASK;
    }
    mediamix_write(reg, regptr + HDMI_RESET_CTL0);
}

/**
 * Function to initialize reset related to lcdif3 in HDMIMIX BLK CTRL.
 * @param regptr Pointer to the mapped HDMIMIX base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void hdmimix_lcdif_reset(volatile uint8_t __iomem* regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = mediamix_read(regptr + HDMI_RESET_CTL0);
    if (enable) {
        reg &= ~(HDMI_RESET_CTL0_LCDIF_APB_RESET_N_MASK | HDMI_RESET_CTL0_LCDIF_ASYNC_RESET_N_MASK);
    }
    else {
        reg |= (HDMI_RESET_CTL0_LCDIF_APB_RESET_N_MASK | HDMI_RESET_CTL0_LCDIF_ASYNC_RESET_N_MASK);
    }
    mediamix_write(reg, regptr + HDMI_RESET_CTL0);
}

/**
 * Function to initialize reset related to irqsteer in HDMIMIX BLK CTRL.
 * @param regptr Pointer to the mapped HDMIMIX base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void hdmimix_irqsteer_reset(volatile uint8_t __iomem* regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = mediamix_read(regptr + HDMI_RESET_CTL0);
    if (enable) {
        reg &= ~(HDMI_RESET_CTL0_IRQ_RESETN_MASK);
    }
    else {
        reg |= (HDMI_RESET_CTL0_IRQ_RESETN_MASK);
    }
    mediamix_write(reg, regptr + HDMI_RESET_CTL0);
}

/**
 * Function to read and log the content of HDMIMIX registers.
 * @param regptr Pointer to the mapped HDMIMIX GPR register address space.
 */
void hdmimix_dump_reg(volatile uint8_t __iomem* regptr)
{
    NT_ASSERT(regptr);
    MEDIAMIX_DEBUG("HDMI_TX_BLK_CTRL_RTX_RESET_CTL0: offset=0x%X val=0x%08X\n",
        HDMI_RESET_CTL0, mediamix_read(regptr + HDMI_RESET_CTL0));
    MEDIAMIX_DEBUG("HDMI_TX_BLK_CTRL_RTX_CLK_CTL0: offset=0x%X val=0x%08X\n",
        HDMI_RTX_CLK_CTL0, mediamix_read(regptr + HDMI_RTX_CLK_CTL0));
    MEDIAMIX_DEBUG("HDMI_TX_BLK_CTRL_RTX_CLK_CTL1: offset=0x%X val=0x%08X\n",
        HDMI_RTX_CLK_CTL1, mediamix_read(regptr + HDMI_RTX_CLK_CTL1));
    MEDIAMIX_DEBUG("TX_BLK_CONTROL0: offset=0x%X val=0x%08X\n",
        TX_BLK_CONTROL0, mediamix_read(regptr + TX_BLK_CONTROL0));

}