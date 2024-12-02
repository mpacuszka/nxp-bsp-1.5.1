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

#ifndef IMX_MEDIAMIX_H
#define IMX_MEDIAMIX_H

#include <linux/io.h>

#define IMX_MEDIAMIX_BASE  0x32EC0000
#define IMX_MEDIAMIX_SIZE  0x10000
#define IMX_HDMIMIX_BASE   0x32FC0000
#define IMX_HDMIMIX_SIZE   0x1000

 /* Media Mix Software Reset Register */
#define MEDIAMIX_SFT_RSTN                    0x00

#define SFT_EN_MIPI_DSI_PCLK_RESETN_MASK     (1UL << 0)
#define SFT_EN_MIPI_DSI_CLKREF_RESETN_MASK   (1UL << 1)
#define SFT_EN_LCDIF_PIXEL_CLK_RESETN_MASK   (1UL << 4)
#define SFT_EN_LCDIF_APB_CLK_RESETN_MASK     (1UL << 5)
#define SFT_EN_BUS_BLK_CLK_RESETN_MASK       (1UL << 8)
#define SFT_EN_LCDIF2_PIXEL_CLK_RESETN_MASK  (1UL << 11)
#define SFT_EN_LCDIF2_APB_CLK_RESETN_MASK    (1UL << 12)
#define SFT_EN_LCDIF_AXI_CLK_RESETN_MASK     (1UL << 23)
#define SFT_EN_LCDIF2_AXI_CLK_RESETN_MASK    (1UL << 24)



/* Media Mix Clock Enable Register */
#define MEDIAMIX_CLK_EN                      0x04

#define SFT_EN_MIPI_DSI_PCLK_MASK            (1UL << 0)
#define SFT_EN_MIPI_DSI_PCLK_SHIFT           (0)
#define SFT_EN_MIPI_DSI_CLKREF_MASK          (1UL << 1)
#define SFT_EN_MIPI_DSI_CLKREF_SHIFT         (1)
#define SFT_EN_LCDIF_PIXEL_CLK_MASK          (1UL << 4)
#define SFT_EN_LCDIF_PIXEL_CLK_SHIFT         (4)
#define SFT_EN_LCDIF_APB_CLK_MASK            (1UL << 5)
#define SFT_EN_LCDIF_APB_CLK_SHIFT           (5)
#define SFT_EN_BUS_BLK_CLK_MASK              (1UL << 8)
#define SFT_EN_BUS_BLK_CLK_SHIFT             (8)
#define SFT_EN_LCDIF2_PIXEL_CLK_MASK         (1UL << 11)
#define SFT_EN_LCDIF2_PIXEL_CLK_SHIFT        (11)
#define SFT_EN_LCDIF2_APB_CLK_MASK           (1UL << 12)
#define SFT_EN_LCDIF2_APB_CLK_SHIFT          (12)
#define SFT_EN_LCDIF_AXI_CLK_MASK            (1UL << 23)
#define SFT_EN_LCDIF_AXI_CLK_SHIFT           (23)
#define SFT_EN_LCDIF2_AXI_CLK_MASK           (1UL << 24)
#define SFT_EN_LCDIF2_AXI_CLK_SHIFT          (24)



/* MIPI PHY Control Register */
#define GPR_MIPI_RESET_DIV                   0x08

#define GPR_MIPI_S_RESETN_MASK               (1UL << 16)
#define GPR_MIPI_M_RESETN_MASK               (1UL << 17)

/* HDMIMIX RESET_CTL0 */
#define HDMI_RESET_CTL0                          0x20
#define HDMI_RESET_CTL0_VID_LINK_SLV_RESETN_MASK (1UL << 22)
#define HDMI_RESET_CTL0_PAI_RESETN_MASK          (1UL << 18)
#define HDMI_RESET_CTL0_IRQ_RESETN_MASK          (1UL << 16)
#define HDMI_RESET_CTL0_TX_PHY_PRESETN_MASK      (1UL << 12)
#define HDMI_RESET_CTL0_TX_APBRSTZ_MASK          (1UL << 11)
#define HDMI_RESET_CTL0_TX_RSTZ_MASK             (1UL << 10)
#define HDMI_RESET_CTL0_FDCC_HDMI_RESETN_MASK    (1UL << 7)
#define HDMI_RESET_CTL0_FDCC_RESETN_MASK         (1UL << 6)
#define HDMI_RESET_CTL0_LCDIF_APB_RESET_N_MASK   (1UL << 5)
#define HDMI_RESET_CTL0_LCDIF_ASYNC_RESET_N_MASK (1UL << 4)

#define HDMI_RTX_CLK_CTL0                        0x40
#define HDMI_LCDIF_SPU_CLK_EN_SHIFT              (20)
#define HDMI_LCDIF_PIX_CLK_EN_SHIFT              (19)
#define HDMI_LCDIF_PDI_CLK_EN_SHIFT              (18)
#define HDMI_LCDIF_B_CLK_EN_SHIFT                (17)
#define HDMI_LCDIF_APB_CLK_EN_SHIFT              (16)
#define HDMI_NOC_HDMI_CLK_EN_SHIFT               (10)
#define HDMI_IRQS_CLK_EN_SHIFT                   (9)
#define HDMI_GLOBAL_TX_PIX_CLK_EN_SHIFT          (7)
#define HDMI_GLOBAL_XTAL24M_CLK_EN_SHIFT         (4)
#define HDMI_GLOBAL_REF266M_CLK_EN_SHIFT         (2)
#define HDMI_GLOBAL_B_CLK_EN_SHIFT               (1)
#define HDMI_GLOBAL_APB_CLK_EN_SHIFT             (0)

#define HDMI_RTX_CLK_CTL1                        0x50
#define HDMI_TX_VID_LINK_PIX_CLK_EN_SHIFT        (28)
#define HDMI_TX_PHY_INT_CLK_EN_SHIFT             (24)
#define HDMI_TX_PHY_APB_CLK_EN_SHIFT             (22)
#define HMDI_TX_PREP_CLK_EN_SHIFT                (21)
#define HDMI_TX_SKP_CLK_EN_SHIFT                 (20)
#define HDMI_TX_SFR_CLK_EN_SHIFT                 (19)
#define HDMI_TX_PIXEL_CLK_EN_SHIFT               (18)
#define HDMI_TX_GPA_CLK_EN_SHIFT                 (17)
#define HDMI_TX_CEC_CLK_EN_SHIFT                 (15)
#define HDMI_TX_APB_CLK_EN_SHIFT                 (14)
#define HDMI_TX_HPI_CLK_EN_SHIFT                 (13)
#define HMDI_HTX_PIPE_CLK_SEL_SHIFT              (12)
#define HMDI_LCDIF_CLK_SEL_SHIFT                 (11)
#define HDMI_HTXPHY_CLK_SEL_SHIFT                (10)
#define HDMI_FDCC_REF_CLK_EN_SHIFT               (2)

#define TX_BLK_CONTROL0                          0x200

void mediamix_dsi_reset(volatile uint8_t __iomem *regptr, bool enable);
void mediamix_lvds_reset(volatile uint8_t __iomem *regptr, bool enable);
void hdmimix_hdmi_reset(volatile uint8_t __iomem *regptr, bool enable);
void hdmimix_pai_reset(volatile uint8_t __iomem *regptr, bool enable);
void hdmimix_pvi_reset(volatile uint8_t __iomem *regptr, bool enable);
void hdmimix_phy_reset(volatile uint8_t __iomem *regptr, bool enable);
void hdmimix_lcdif_reset(volatile uint8_t __iomem* regptr, bool enable);
void hdmimix_irqsteer_reset(volatile uint8_t __iomem* regptr, bool enable);

void mediamix_dump_reg(volatile uint8_t __iomem *regptr);
void hdmimix_dump_reg(volatile uint8_t __iomem* regptr);

#endif
