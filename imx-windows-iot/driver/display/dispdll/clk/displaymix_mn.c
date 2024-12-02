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
#include "displaymix_mn.h"

#define IMX_DISPLAYMIX_DEBUG

#ifdef IMX_DISPLAYMIX_DEBUG
#define DISPLAYMIX_DEBUG(...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, __VA_ARGS__)
#else
#define DISPLAYMIX_DEBUG(...)
#endif

#define displaymix_write(val, addr)  writel(val,addr)
#define displaymix_read(addr)        readl(addr)

/* Dispmix GPR registers */
/* DISPLAY_MIX_SFT_RSTN_CSR register */
#define DISPLAY_MIX_SFT_RSTN_CSR         0x00

#define SFT_EN_BUS_BLK_CLK_RESETN_SHIFT     (8)
#define SFT_EN_BUS_BLK_CLK_RESETN_MASK      (1UL << SFT_EN_BUS_BLK_CLK_RESETN_SHIFT)
#define SFT_EN_LCDIF_APB_CLK_RESETN_SHIFT   (5)
#define SFT_EN_LCDIF_APB_CLK_RESETN_MASK    (1UL << SFT_EN_LCDIF_APB_CLK_RESETN_SHIFT)
#define SFT_EN_LCDIF_PIXEL_CLK_RESETN_SHIFT (4)
#define SFT_EN_LCDIF_PIXEL_CLK_RESETN_MASK  (1UL << SFT_EN_LCDIF_PIXEL_CLK_RESETN_SHIFT)
#define SFT_EN_MIPI_DSI_CLKREF_RESETN_SHIFT (1)
#define SFT_EN_MIPI_DSI_CLKREF_RESETN_MASK  (1UL << SFT_EN_MIPI_DSI_CLKREF_RESETN_SHIFT)
#define SFT_EN_MIPI_DSI_PCLK_RESETN_SHIFT   (0)
#define SFT_EN_MIPI_DSI_PCLK_RESETN_MASK    (1UL << SFT_EN_MIPI_DSI_PCLK_RESETN_SHIFT)

/* DISPLAY_MIX_CLK_EN_CSR register */
#define DISPLAY_MIX_CLK_EN_CSR           0x04

#define SFT_EN_BUS_BLK_CLK_SHIFT         (8)
#define SFT_EN_BUS_BLK_CLK_MASK          (1UL << SFT_EN_BUS_BLK_CLK_SHIFT)
#define SFT_EN_LCDIF_APB_CLK_SHIFT       (5)
#define SFT_EN_LCDIF_APB_CLK_MASK        (1UL << SFT_EN_LCDIF_APB_CLK_SHIFT)
#define SFT_EN_LCDIF_PIXEL_CLK_SHIFT     (4)
#define SFT_EN_LCDIF_PIXEL_CLK_MASK      (1UL << SFT_EN_LCDIF_PIXEL_CLK_SHIFT)
#define SFT_EN_MIPI_DSI_CLKREF_SHIFT     (1)
#define SFT_EN_MIPI_DSI_CLKREF_MASK      (1UL << SFT_EN_MIPI_DSI_CLKREF_SHIFT)
#define SFT_EN_MIPI_DSI_PCLK_SHIFT       (0)
#define SFT_EN_MIPI_DSI_PCLK_MASK        (1UL << SFT_EN_MIPI_DSI_PCLK_SHIFT)

/* GPR_MIPI_RESET_DIV register */
#define GPR_MIPI_RESET_DIV               0x08
/* Clock & Data lanes reset: Active Low */
#define GPR_MIPI_M_RESETN_SHIFT          (17)
#define GPR_MIPI_M_RESETN_MASK           (1UL << GPR_MIPI_M_RESETN_SHIFT)
#define GPR_MIPI_S_RESETN_SHIFT          (16)
#define GPR_MIPI_S_RESETN_MASK           (1UL << GPR_MIPI_S_RESETN_SHIFT)

/**
 * Function to control lane reset related to mipi-dsi module
 * @param regptr Pointer to the mapped DISPLAYMIX GPR base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void displaymix_mn_dsi_mipi_reset(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = displaymix_read(regptr + GPR_MIPI_RESET_DIV);
    if (enable) {
        reg &= ~(GPR_MIPI_M_RESETN_MASK);
    } else {
        reg |= (GPR_MIPI_M_RESETN_MASK);
    }
    displaymix_write(reg, regptr + GPR_MIPI_RESET_DIV);
}

/**
 * Function to control soft reset related to mipi-dsi module
 * @param regptr Pointer to the mapped DISPLAYMIX GPR base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void displaymix_mn_dsi_soft_resetn(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = displaymix_read(regptr + DISPLAY_MIX_SFT_RSTN_CSR);
    if (enable) {
        reg &= ~(SFT_EN_MIPI_DSI_CLKREF_RESETN_MASK|SFT_EN_MIPI_DSI_PCLK_RESETN_MASK);
    } else {
        reg |= (SFT_EN_MIPI_DSI_CLKREF_RESETN_MASK|SFT_EN_MIPI_DSI_PCLK_RESETN_MASK);
    }
    displaymix_write(reg, regptr + DISPLAY_MIX_SFT_RSTN_CSR);
}

/**
 * Function to control clock enable related to mipi-dsi module
 * @param regptr Pointer to the mapped DISPLAYMIX GPR base address space.
 * @enable 1=enable clock, 0=disable clock.
 */
void displaymix_mn_dsi_clk_enable(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = displaymix_read(regptr + DISPLAY_MIX_CLK_EN_CSR);
    if (enable) {
        reg |= (SFT_EN_MIPI_DSI_CLKREF_MASK | SFT_EN_MIPI_DSI_PCLK_MASK);
    } else {
        reg &= ~(SFT_EN_MIPI_DSI_CLKREF_MASK | SFT_EN_MIPI_DSI_PCLK_MASK);
    }
    displaymix_write(reg, regptr + DISPLAY_MIX_CLK_EN_CSR);
}

/**
 * Function to control reset related to lcdif module
 * @param regptr Pointer to the mapped DISPLAYMIX GPR base address space.
 * @enable 1=assert reset, 0=deassert reset.
 */
void displaymix_mn_lcdif_soft_resetn(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = displaymix_read(regptr + DISPLAY_MIX_SFT_RSTN_CSR);
    if (enable) {
        reg &= ~(SFT_EN_LCDIF_APB_CLK_RESETN_MASK|SFT_EN_LCDIF_PIXEL_CLK_RESETN_MASK);
    
    } else {
        reg |= (SFT_EN_LCDIF_APB_CLK_RESETN_MASK|SFT_EN_LCDIF_PIXEL_CLK_RESETN_MASK|
            SFT_EN_BUS_BLK_CLK_RESETN_MASK);
    }
    displaymix_write(reg, regptr + DISPLAY_MIX_SFT_RSTN_CSR);
}

/**
 * Function to control clock enable related to lcdif module
 * @param regptr Pointer to the mapped DISPLAYMIX GPR base address space.
 * @enable 1=enable clock, 0=disable clock.
 */
void displaymix_mn_lcdif_clk_enable(volatile uint8_t __iomem *regptr, bool enable)
{
    uint32_t reg;

    NT_ASSERT(regptr);

    reg = displaymix_read(regptr + DISPLAY_MIX_CLK_EN_CSR);
    if (enable) {
        reg |= (SFT_EN_LCDIF_APB_CLK_MASK | SFT_EN_LCDIF_PIXEL_CLK_MASK |
            SFT_EN_BUS_BLK_CLK_MASK);
    } else {
        reg &= ~(SFT_EN_LCDIF_APB_CLK_MASK | SFT_EN_LCDIF_PIXEL_CLK_MASK);
    }
    displaymix_write(reg, regptr + DISPLAY_MIX_CLK_EN_CSR);
}

/**
 * Function to read and log the content of DISPMIX GPR registers.
 * @param regptr Pointer to the mapped DISPMIX GPR register address space.
 */
void displaymix_dump_reg(volatile uint8_t __iomem *regptr)
{
    NT_ASSERT(regptr);
    DISPLAYMIX_DEBUG("DISPLAY_MIX_SFT_RSTN_CSR: offset=0x%x val=0x%x\n",
        DISPLAY_MIX_SFT_RSTN_CSR, displaymix_read(regptr + DISPLAY_MIX_SFT_RSTN_CSR));
    DISPLAYMIX_DEBUG("DISPLAY_MIX_CLK_EN_CSR: offset=0x%x val=0x%x\n",
        DISPLAY_MIX_CLK_EN_CSR, displaymix_read(regptr + DISPLAY_MIX_CLK_EN_CSR));
    DISPLAYMIX_DEBUG("GPR_MIPI_RESET_DIV: offset=0x%x val=0x%x\n",
        GPR_MIPI_RESET_DIV, displaymix_read(regptr + GPR_MIPI_RESET_DIV));
}
