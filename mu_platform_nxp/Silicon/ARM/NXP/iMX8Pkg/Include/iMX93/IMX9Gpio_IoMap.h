/*
 * Copyright 2023 NXP
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

#ifndef _IMX_GPIO_IOMAP_H_
#define _IMX_GPIO_IOMAP_H_

#define ALT0    0
#define ALT1    1
#define ALT2    2
#define ALT3    3
#define ALT4    4
#define ALT5    5
#define ALT6    6
#define ALT7    7


/* Pad muxing registres offsets */

#define IMX_SW_MUX_CTL_PAD_NONE                                   0xFF, 0x0F

#define IMX_SW_MUX_CTL_PAD_DAP_TDI                                0x00, 0x00
#define IMX_SW_MUX_CTL_PAD_DAP_TMS_SWDIO                          0x04, 0x00
#define IMX_SW_MUX_CTL_PAD_DAP_TCLK_SWCLK                         0x08, 0x00
#define IMX_SW_MUX_CTL_PAD_DAP_TDO_TRACESWO                       0x0C, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO00                              0x10, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO01                              0x14, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO02                              0x18, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO03                              0x1C, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO04                              0x20, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO05                              0x24, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO06                              0x28, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO07                              0x2C, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO08                              0x30, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO09                              0x34, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO10                              0x38, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO11                              0x3C, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO12                              0x40, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO13                              0x44, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO14                              0x48, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO15                              0x4C, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO16                              0x50, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO17                              0x54, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO18                              0x58, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO19                              0x5C, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO20                              0x60, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO21                              0x64, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO22                              0x68, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO23                              0x6C, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO24                              0x70, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO25                              0x74, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO26                              0x78, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO27                              0x7C, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO28                              0x80, 0x00
#define IMX_SW_MUX_CTL_PAD_GPIO_IO29                              0x84, 0x00
#define IMX_SW_MUX_CTL_PAD_CCM_CLKO1                              0x88, 0x00
#define IMX_SW_MUX_CTL_PAD_CCM_CLKO2                              0x8C, 0x00
#define IMX_SW_MUX_CTL_PAD_CCM_CLKO3                              0x90, 0x00
#define IMX_SW_MUX_CTL_PAD_CCM_CLKO4                              0x94, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_MDC                              0x98, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_MDIO                             0x9C, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_TD3                              0xA0, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_TD2                              0xA4, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_TD1                              0xA8, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_TD0                              0xAC, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_TX_CTL                           0xB0, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_TXC                              0xB4, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_RX_CTL                           0xB8, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_RXC                              0xBC, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_RD0                              0xC0, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_RD1                              0xC4, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_RD2                              0xC8, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET1_RD3                              0xCC, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_MDC                              0xD0, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_MDIO                             0xD4, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_TD3                              0xD8, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_TD2                              0xDC, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_TD1                              0xE0, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_TD0                              0xE4, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_TX_CTL                           0xE8, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_TXC                              0xEC, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_RX_CTL                           0xF0, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_RXC                              0xF4, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_RD0                              0xF8, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_RD1                              0xFC, 0x00
#define IMX_SW_MUX_CTL_PAD_ENET2_RD2                              0x00, 0x01
#define IMX_SW_MUX_CTL_PAD_ENET2_RD3                              0x04, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_CLK                                0x08, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_CMD                                0x0C, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_DATA0                              0x10, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_DATA1                              0x14, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_DATA2                              0x18, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_DATA3                              0x1C, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_DATA4                              0x20, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_DATA5                              0x24, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_DATA6                              0x28, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_DATA7                              0x2C, 0x01
#define IMX_SW_MUX_CTL_PAD_SD1_STROBE                             0x30, 0x01
#define IMX_SW_MUX_CTL_PAD_SD2_VSELECT                            0x34, 0x01
#define IMX_SW_MUX_CTL_PAD_SD3_CLK                                0x38, 0x01
#define IMX_SW_MUX_CTL_PAD_SD3_CMD                                0x3C, 0x01
#define IMX_SW_MUX_CTL_PAD_SD3_DATA0                              0x40, 0x01
#define IMX_SW_MUX_CTL_PAD_SD3_DATA1                              0x44, 0x01
#define IMX_SW_MUX_CTL_PAD_SD3_DATA2                              0x48, 0x01
#define IMX_SW_MUX_CTL_PAD_SD3_DATA3                              0x4C, 0x01
#define IMX_SW_MUX_CTL_PAD_SD2_CD_B                               0x50, 0x01
#define IMX_SW_MUX_CTL_PAD_SD2_CLK                                0x54, 0x01
#define IMX_SW_MUX_CTL_PAD_SD2_CMD                                0x58, 0x01
#define IMX_SW_MUX_CTL_PAD_SD2_DATA0                              0x5C, 0x01
#define IMX_SW_MUX_CTL_PAD_SD2_DATA1                              0x60, 0x01
#define IMX_SW_MUX_CTL_PAD_SD2_DATA2                              0x64, 0x01
#define IMX_SW_MUX_CTL_PAD_SD2_DATA3                              0x68, 0x01
#define IMX_SW_MUX_CTL_PAD_SD2_RESET_B                            0x6C, 0x01
#define IMX_SW_MUX_CTL_PAD_I2C1_SCL                               0x70, 0x01
#define IMX_SW_MUX_CTL_PAD_I2C1_SDA                               0x74, 0x01
#define IMX_SW_MUX_CTL_PAD_I2C2_SCL                               0x78, 0x01
#define IMX_SW_MUX_CTL_PAD_I2C2_SDA                               0x7C, 0x01
#define IMX_SW_MUX_CTL_PAD_UART1_RXD                              0x80, 0x01
#define IMX_SW_MUX_CTL_PAD_UART1_TXD                              0x84, 0x01
#define IMX_SW_MUX_CTL_PAD_UART2_RXD                              0x88, 0x01
#define IMX_SW_MUX_CTL_PAD_UART2_TXD                              0x8C, 0x01
#define IMX_SW_MUX_CTL_PAD_PDM_CLK                                0x90, 0x01
#define IMX_SW_MUX_CTL_PAD_PDM_BIT_STREAM0                        0x94, 0x01
#define IMX_SW_MUX_CTL_PAD_PDM_BIT_STREAM1                        0x98, 0x01
#define IMX_SW_MUX_CTL_PAD_SAI1_TXFS                              0x9C, 0x01
#define IMX_SW_MUX_CTL_PAD_SAI1_TXC                               0xA0, 0x01
#define IMX_SW_MUX_CTL_PAD_SAI1_TXD0                              0xA4, 0x01
#define IMX_SW_MUX_CTL_PAD_SAI1_RXD0                              0xA8, 0x01
#define IMX_SW_MUX_CTL_PAD_WDOG_ANY                               0xAC, 0x01

/* Pad configurations registres offsets */

#define IMX_SW_PAD_CTL_PAD_NONE                                   0xFF, 0x0F

#define IMX_SW_PAD_CTL_PAD_DAP_TDI                                0xB0, 0x01
#define IMX_SW_PAD_CTL_PAD_DAP_TMS_SWDIO                          0xB4, 0x01
#define IMX_SW_PAD_CTL_PAD_DAP_TCLK_SWCLK                         0xB8, 0x01
#define IMX_SW_PAD_CTL_PAD_DAP_TDO_TRACESWO                       0xBC, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO00                              0xC0, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO01                              0xC4, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO02                              0xC8, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO03                              0xCC, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO04                              0xD0, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO05                              0xD4, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO06                              0xD8, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO07                              0xDC, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO08                              0xE0, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO09                              0xE4, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO10                              0xE8, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO11                              0xEC, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO12                              0xF0, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO13                              0xF4, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO14                              0xF8, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO15                              0xFC, 0x01
#define IMX_SW_PAD_CTL_PAD_GPIO_IO16                              0x00, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO17                              0x04, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO18                              0x08, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO19                              0x0C, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO20                              0x10, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO21                              0x14, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO22                              0x18, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO23                              0x1C, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO24                              0x20, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO25                              0x24, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO26                              0x28, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO27                              0x2C, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO28                              0x30, 0x02
#define IMX_SW_PAD_CTL_PAD_GPIO_IO29                              0x34, 0x02
#define IMX_SW_PAD_CTL_PAD_CCM_CLKO1                              0x38, 0x02
#define IMX_SW_PAD_CTL_PAD_CCM_CLKO2                              0x3C, 0x02
#define IMX_SW_PAD_CTL_PAD_CCM_CLKO3                              0x40, 0x02
#define IMX_SW_PAD_CTL_PAD_CCM_CLKO4                              0x44, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_MDC                              0x48, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_MDIO                             0x4C, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_TD3                              0x50, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_TD2                              0x54, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_TD1                              0x58, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_TD0                              0x5C, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_TX_CTL                           0x60, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_TXC                              0x64, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_RX_CTL                           0x68, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_RXC                              0x6C, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_RD0                              0x70, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_RD1                              0x74, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_RD2                              0x78, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET1_RD3                              0x7C, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_MDC                              0x80, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_MDIO                             0x84, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_TD3                              0x88, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_TD2                              0x8C, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_TD1                              0x90, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_TD0                              0x94, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_TX_CTL                           0x98, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_TXC                              0x9C, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_RX_CTL                           0xA0, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_RXC                              0xA4, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_RD0                              0xA8, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_RD1                              0xAC, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_RD2                              0xB0, 0x02
#define IMX_SW_PAD_CTL_PAD_ENET2_RD3                              0xB4, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_CLK                                0xB8, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_CMD                                0xBC, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_DATA0                              0xC0, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_DATA1                              0xC4, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_DATA2                              0xC8, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_DATA3                              0xCC, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_DATA4                              0xD0, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_DATA5                              0xD4, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_DATA6                              0xD8, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_DATA7                              0xDC, 0x02
#define IMX_SW_PAD_CTL_PAD_SD1_STROBE                             0xE0, 0x02
#define IMX_SW_PAD_CTL_PAD_SD2_VSELECT                            0xE4, 0x02
#define IMX_SW_PAD_CTL_PAD_SD3_CLK                                0xE8, 0x02
#define IMX_SW_PAD_CTL_PAD_SD3_CMD                                0xEC, 0x02
#define IMX_SW_PAD_CTL_PAD_SD3_DATA0                              0xF0, 0x02
#define IMX_SW_PAD_CTL_PAD_SD3_DATA1                              0xF4, 0x02
#define IMX_SW_PAD_CTL_PAD_SD3_DATA2                              0xF8, 0x02
#define IMX_SW_PAD_CTL_PAD_SD3_DATA3                              0xFC, 0x02
#define IMX_SW_PAD_CTL_PAD_SD2_CD_B                               0x00, 0x03
#define IMX_SW_PAD_CTL_PAD_SD2_CLK                                0x04, 0x03
#define IMX_SW_PAD_CTL_PAD_SD2_CMD                                0x08, 0x03
#define IMX_SW_PAD_CTL_PAD_SD2_DATA0                              0x0C, 0x03
#define IMX_SW_PAD_CTL_PAD_SD2_DATA1                              0x10, 0x03
#define IMX_SW_PAD_CTL_PAD_SD2_DATA2                              0x14, 0x03
#define IMX_SW_PAD_CTL_PAD_SD2_DATA3                              0x18, 0x03
#define IMX_SW_PAD_CTL_PAD_SD2_RESET_B                            0x1C, 0x03
#define IMX_SW_PAD_CTL_PAD_I2C1_SCL                               0x20, 0x03
#define IMX_SW_PAD_CTL_PAD_I2C1_SDA                               0x24, 0x03
#define IMX_SW_PAD_CTL_PAD_I2C2_SCL                               0x28, 0x03
#define IMX_SW_PAD_CTL_PAD_I2C2_SDA                               0x2C, 0x03
#define IMX_SW_PAD_CTL_PAD_UART1_RXD                              0x30, 0x03
#define IMX_SW_PAD_CTL_PAD_UART1_TXD                              0x34, 0x03
#define IMX_SW_PAD_CTL_PAD_UART2_RXD                              0x38, 0x03
#define IMX_SW_PAD_CTL_PAD_UART2_TXD                              0x3C, 0x03
#define IMX_SW_PAD_CTL_PAD_PDM_CLK                                0x40, 0x03
#define IMX_SW_PAD_CTL_PAD_PDM_BIT_STREAM0                        0x44, 0x03
#define IMX_SW_PAD_CTL_PAD_PDM_BIT_STREAM1                        0x48, 0x03
#define IMX_SW_PAD_CTL_PAD_SAI1_TXFS                              0x4C, 0x03
#define IMX_SW_PAD_CTL_PAD_SAI1_TXC                               0x50, 0x03
#define IMX_SW_PAD_CTL_PAD_SAI1_TXD0                              0x54, 0x03
#define IMX_SW_PAD_CTL_PAD_SAI1_RXD0                              0x58, 0x03
#define IMX_SW_PAD_CTL_PAD_WDOG_ANY                               0x5C, 0x03

/* Peripheral input pin/signal multiplexor registres offsets */

#define IMX_PIN_CAN1_IPP_IND_CANRX_SELECT_INPUT_REG               0x60, 0x03
#define IMX_PIN_CAN2_IPP_IND_CANRX_SELECT_INPUT_REG               0x64, 0x03
#define IMX_PIN_CCMSRCGPCMIX_EXT1_CLK_SELECT_INPUT_REG            0x68, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_0_REG         0x6C, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_1_REG         0x70, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_2_REG         0x74, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_3_REG         0x78, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_4_REG         0x7C, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_5_REG         0x80, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_6_REG         0x84, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_7_REG         0x88, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_8_REG         0x8C, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_9_REG         0x90, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_10_REG        0x94, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_11_REG        0x98, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_13_REG        0x9C, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_14_REG        0xA0, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_15_REG        0xA4, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_16_REG        0xA8, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_17_REG        0xAC, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_18_REG        0xB0, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_20_REG        0xB4, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_22_REG        0xB8, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_23_REG        0xBC, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_24_REG        0xC0, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_25_REG        0xC4, 0x03
#define IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_27_REG        0xC8, 0x03
#define IMX_PIN_I3C2_PIN_SCL_IN_SELECT_INPUT_REG                  0xCC, 0x03
#define IMX_PIN_I3C2_PIN_SDA_IN_SELECT_INPUT_REG                  0xD0, 0x03
#define IMX_PIN_JTAG_MUX_TCK_SELECT_INPUT_REG                     0xD4, 0x03
#define IMX_PIN_JTAG_MUX_TDI_SELECT_INPUT_REG                     0xD8, 0x03
#define IMX_PIN_JTAG_MUX_TMS_SELECT_INPUT_REG                     0xDC, 0x03
#define IMX_PIN_LPI2C3_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG         0xE0, 0x03
#define IMX_PIN_LPI2C3_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG         0xE4, 0x03
#define IMX_PIN_LPI2C5_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG         0xE8, 0x03
#define IMX_PIN_LPI2C5_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG         0xEC, 0x03
#define IMX_PIN_LPI2C6_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG         0xF0, 0x03
#define IMX_PIN_LPI2C6_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG         0xF4, 0x03
#define IMX_PIN_LPI2C7_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG         0xF8, 0x03
#define IMX_PIN_LPI2C7_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG         0xFC, 0x03
#define IMX_PIN_LPI2C8_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG         0x00, 0x04
#define IMX_PIN_LPI2C8_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG         0x04, 0x04
#define IMX_PIN_LPTMR2_IPP_IND_LPTIMER_SELECT_INPUT_0_REG         0x08, 0x04
#define IMX_PIN_LPTMR2_IPP_IND_LPTIMER_SELECT_INPUT_1_REG         0x0C, 0x04
#define IMX_PIN_LPTMR2_IPP_IND_LPTIMER_SELECT_INPUT_2_REG         0x10, 0x04
#define IMX_PIN_LPUART3_IPP_IND_LPUART_CTS_N_SELECT_INPUT_REG     0x14, 0x04
#define IMX_PIN_LPUART3_IPP_IND_LPUART_RXD_SELECT_INPUT_REG       0x18, 0x04
#define IMX_PIN_LPUART3_IPP_IND_LPUART_TXD_SELECT_INPUT_REG       0x1C, 0x04
#define IMX_PIN_LPUART4_IPP_IND_LPUART_CTS_N_SELECT_INPUT_REG     0x20, 0x04
#define IMX_PIN_LPUART4_IPP_IND_LPUART_RXD_SELECT_INPUT_REG       0x24, 0x04
#define IMX_PIN_LPUART4_IPP_IND_LPUART_TXD_SELECT_INPUT_REG       0x28, 0x04
#define IMX_PIN_LPUART5_IPP_IND_LPUART_CTS_N_SELECT_INPUT_REG     0x2C, 0x04
#define IMX_PIN_LPUART5_IPP_IND_LPUART_RXD_SELECT_INPUT_REG       0x30, 0x04
#define IMX_PIN_LPUART5_IPP_IND_LPUART_TXD_SELECT_INPUT_REG       0x34, 0x04
#define IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_0_REG  0x38, 0x04
#define IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_1_REG  0x3C, 0x04
#define IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_2_REG  0x40, 0x04
#define IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_3_REG  0x44, 0x04
#define IMX_PIN_SAI1_IPP_IND_SAI_MCLK_SELECT_INPUT_REG            0x48, 0x04
#define IMX_PIN_SAI3_IPP_IND_SAI_RXBCLK_SELECT_INPUT_REG          0x4C, 0x04
#define IMX_PIN_SAI3_IPP_IND_SAI_RXSYNC_SELECT_INPUT_REG          0x50, 0x04
#define IMX_PIN_SPDIF_SPDIF_I_SELECT_INPUT_REG                    0x54, 0x04
#define IMX_PIN_USDHC3_IPP_CARD_CLK_IN_SELECT_INPUT_REG           0x58, 0x04
#define IMX_PIN_USDHC3_IPP_CMD_IN_SELECT_INPUT_REG                0x5C, 0x04
#define IMX_PIN_USDHC3_IPP_DAT0_IN_SELECT_INPUT_REG               0x60, 0x04
#define IMX_PIN_USDHC3_IPP_DAT1_IN_SELECT_INPUT_REG               0x64, 0x04
#define IMX_PIN_USDHC3_IPP_DAT2_IN_SELECT_INPUT_REG               0x68, 0x04
#define IMX_PIN_USDHC3_IPP_DAT3_IN_SELECT_INPUT_REG               0x6C, 0x04

 #endif // _IMX_GPIO_IOMAP_H_