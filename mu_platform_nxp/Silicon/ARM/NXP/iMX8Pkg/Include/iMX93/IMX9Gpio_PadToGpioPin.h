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

#ifndef _IMX_GPIO_H_
#define _IMX_GPIO_H_

#include "IMX9GpioPinNumbers.h"

/* PAD to GPIO maping */

#define IMX_PAD_I2C1_SCL                                IMX_PIN_NUM_GPIO1_IO00
#define IMX_PAD_I2C1_SDA                                IMX_PIN_NUM_GPIO1_IO01
#define IMX_PAD_I2C2_SCL                                IMX_PIN_NUM_GPIO1_IO02
#define IMX_PAD_I2C2_SDA                                IMX_PIN_NUM_GPIO1_IO03
#define IMX_PAD_UART1_RXD                               IMX_PIN_NUM_GPIO1_IO04
#define IMX_PAD_UART1_TXD                               IMX_PIN_NUM_GPIO1_IO05
#define IMX_PAD_UART2_RXD                               IMX_PIN_NUM_GPIO1_IO06
#define IMX_PAD_UART2_TXD                               IMX_PIN_NUM_GPIO1_IO07
#define IMX_PAD_PDM_CLK                                 IMX_PIN_NUM_GPIO1_IO08
#define IMX_PAD_PDM_BIT_STREAM0                         IMX_PIN_NUM_GPIO1_IO09
#define IMX_PAD_PDM_BIT_STREAM1                         IMX_PIN_NUM_GPIO1_IO10
#define IMX_PAD_SAI1_TXFS                               IMX_PIN_NUM_GPIO1_IO11
#define IMX_PAD_SAI1_TXC                                IMX_PIN_NUM_GPIO1_IO12
#define IMX_PAD_SAI1_TXD0                               IMX_PIN_NUM_GPIO1_IO13
#define IMX_PAD_SAI1_RXD0                               IMX_PIN_NUM_GPIO1_IO14
#define IMX_PAD_WDOG_ANY                                IMX_PIN_NUM_GPIO1_IO15
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO16
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO17
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO18
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO19
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO20
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO21
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO22
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO23
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO24
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO25
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO26
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO27
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO28
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO29
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO30
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO1_IO31

#define IMX_PAD_GPIO_IO00                               IMX_PIN_NUM_GPIO2_IO00
#define IMX_PAD_GPIO_IO01                               IMX_PIN_NUM_GPIO2_IO01
#define IMX_PAD_GPIO_IO02                               IMX_PIN_NUM_GPIO2_IO02
#define IMX_PAD_GPIO_IO03                               IMX_PIN_NUM_GPIO2_IO03
#define IMX_PAD_GPIO_IO04                               IMX_PIN_NUM_GPIO2_IO04
#define IMX_PAD_GPIO_IO05                               IMX_PIN_NUM_GPIO2_IO05
#define IMX_PAD_GPIO_IO06                               IMX_PIN_NUM_GPIO2_IO06
#define IMX_PAD_GPIO_IO07                               IMX_PIN_NUM_GPIO2_IO07
#define IMX_PAD_GPIO_IO08                               IMX_PIN_NUM_GPIO2_IO08
#define IMX_PAD_GPIO_IO09                               IMX_PIN_NUM_GPIO2_IO09
#define IMX_PAD_GPIO_IO10                               IMX_PIN_NUM_GPIO2_IO10
#define IMX_PAD_GPIO_IO11                               IMX_PIN_NUM_GPIO2_IO11
#define IMX_PAD_GPIO_IO12                               IMX_PIN_NUM_GPIO2_IO12
#define IMX_PAD_GPIO_IO13                               IMX_PIN_NUM_GPIO2_IO13
#define IMX_PAD_GPIO_IO14                               IMX_PIN_NUM_GPIO2_IO14
#define IMX_PAD_GPIO_IO15                               IMX_PIN_NUM_GPIO2_IO15
#define IMX_PAD_GPIO_IO16                               IMX_PIN_NUM_GPIO2_IO16
#define IMX_PAD_GPIO_IO17                               IMX_PIN_NUM_GPIO2_IO17
#define IMX_PAD_GPIO_IO18                               IMX_PIN_NUM_GPIO2_IO18
#define IMX_PAD_GPIO_IO19                               IMX_PIN_NUM_GPIO2_IO19
#define IMX_PAD_GPIO_IO20                               IMX_PIN_NUM_GPIO2_IO20
#define IMX_PAD_GPIO_IO21                               IMX_PIN_NUM_GPIO2_IO21
#define IMX_PAD_GPIO_IO22                               IMX_PIN_NUM_GPIO2_IO22
#define IMX_PAD_GPIO_IO23                               IMX_PIN_NUM_GPIO2_IO23
#define IMX_PAD_GPIO_IO24                               IMX_PIN_NUM_GPIO2_IO24
#define IMX_PAD_GPIO_IO25                               IMX_PIN_NUM_GPIO2_IO25
#define IMX_PAD_GPIO_IO26                               IMX_PIN_NUM_GPIO2_IO26
#define IMX_PAD_GPIO_IO27                               IMX_PIN_NUM_GPIO2_IO27
#define IMX_PAD_GPIO_IO28                               IMX_PIN_NUM_GPIO2_IO28
#define IMX_PAD_GPIO_IO29                               IMX_PIN_NUM_GPIO2_IO29
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO2_IO30
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO2_IO31

#define IMX_PAD_SD2_CD_B                                IMX_PIN_NUM_GPIO3_IO00
#define IMX_PAD_SD2_CLK                                 IMX_PIN_NUM_GPIO3_IO01
#define IMX_PAD_SD2_CMD                                 IMX_PIN_NUM_GPIO3_IO02
#define IMX_PAD_SD2_DATA0                               IMX_PIN_NUM_GPIO3_IO03
#define IMX_PAD_SD2_DATA1                               IMX_PIN_NUM_GPIO3_IO04
#define IMX_PAD_SD2_DATA2                               IMX_PIN_NUM_GPIO3_IO05
#define IMX_PAD_SD2_DATA3                               IMX_PIN_NUM_GPIO3_IO06
#define IMX_PAD_SD2_RESET_B                             IMX_PIN_NUM_GPIO3_IO07
#define IMX_PAD_SD1_CLK                                 IMX_PIN_NUM_GPIO3_IO08
#define IMX_PAD_SD1_CMD                                 IMX_PIN_NUM_GPIO3_IO09
#define IMX_PAD_SD1_DATA0                               IMX_PIN_NUM_GPIO3_IO10
#define IMX_PAD_SD1_DATA1                               IMX_PIN_NUM_GPIO3_IO11
#define IMX_PAD_SD1_DATA2                               IMX_PIN_NUM_GPIO3_IO12
#define IMX_PAD_SD1_DATA3                               IMX_PIN_NUM_GPIO3_IO13
#define IMX_PAD_SD1_DATA4                               IMX_PIN_NUM_GPIO3_IO14
#define IMX_PAD_SD1_DATA5                               IMX_PIN_NUM_GPIO3_IO15
#define IMX_PAD_SD1_DATA6                               IMX_PIN_NUM_GPIO3_IO16
#define IMX_PAD_SD1_DATA7                               IMX_PIN_NUM_GPIO3_IO17
#define IMX_PAD_SD1_STROBE                              IMX_PIN_NUM_GPIO3_IO18
#define IMX_PAD_SD2_VSELECT                             IMX_PIN_NUM_GPIO3_IO19
#define IMX_PAD_SD3_CLK                                 IMX_PIN_NUM_GPIO3_IO20
#define IMX_PAD_SD3_CMD                                 IMX_PIN_NUM_GPIO3_IO21
#define IMX_PAD_SD3_DATA0                               IMX_PIN_NUM_GPIO3_IO22
#define IMX_PAD_SD3_DATA1                               IMX_PIN_NUM_GPIO3_IO23
#define IMX_PAD_SD3_DATA2                               IMX_PIN_NUM_GPIO3_IO24
#define IMX_PAD_SD3_DATA3                               IMX_PIN_NUM_GPIO3_IO25
#define IMX_PAD_CCM_CLKO1                               IMX_PIN_NUM_GPIO3_IO26
#define IMX_PAD_CCM_CLKO2                               IMX_PIN_NUM_GPIO3_IO27
#define IMX_PAD_DAP_TDI                                 IMX_PIN_NUM_GPIO3_IO28
#define IMX_PAD_DAP_TMS_SWDIO                           IMX_PIN_NUM_GPIO3_IO29
#define IMX_PAD_DAP_TCLK_SWCLK                          IMX_PIN_NUM_GPIO3_IO30
#define IMX_PAD_DAP_TDO_TRACESWO                        IMX_PIN_NUM_GPIO3_IO31

#define IMX_PAD_ENET1_MDC                               IMX_PIN_NUM_GPIO4_IO00
#define IMX_PAD_ENET1_MDIO                              IMX_PIN_NUM_GPIO4_IO01
#define IMX_PAD_ENET1_TD3                               IMX_PIN_NUM_GPIO4_IO02
#define IMX_PAD_ENET1_TD2                               IMX_PIN_NUM_GPIO4_IO03
#define IMX_PAD_ENET1_TD1                               IMX_PIN_NUM_GPIO4_IO04
#define IMX_PAD_ENET1_TD0                               IMX_PIN_NUM_GPIO4_IO05
#define IMX_PAD_ENET1_TX_CTL                            IMX_PIN_NUM_GPIO4_IO06
#define IMX_PAD_ENET1_TXC                               IMX_PIN_NUM_GPIO4_IO07
#define IMX_PAD_ENET1_RX_CTL                            IMX_PIN_NUM_GPIO4_IO08
#define IMX_PAD_ENET1_RXC                               IMX_PIN_NUM_GPIO4_IO09
#define IMX_PAD_ENET1_RD0                               IMX_PIN_NUM_GPIO4_IO10
#define IMX_PAD_ENET1_RD1                               IMX_PIN_NUM_GPIO4_IO11
#define IMX_PAD_ENET1_RD2                               IMX_PIN_NUM_GPIO4_IO12
#define IMX_PAD_ENET1_RD3                               IMX_PIN_NUM_GPIO4_IO13
#define IMX_PAD_ENET2_MDC                               IMX_PIN_NUM_GPIO4_IO14
#define IMX_PAD_ENET2_MDIO                              IMX_PIN_NUM_GPIO4_IO15
#define IMX_PAD_ENET2_TD3                               IMX_PIN_NUM_GPIO4_IO16
#define IMX_PAD_ENET2_TD2                               IMX_PIN_NUM_GPIO4_IO17
#define IMX_PAD_ENET2_TD1                               IMX_PIN_NUM_GPIO4_IO18
#define IMX_PAD_ENET2_TD0                               IMX_PIN_NUM_GPIO4_IO19
#define IMX_PAD_ENET2_TX_CTL                            IMX_PIN_NUM_GPIO4_IO20
#define IMX_PAD_ENET2_TXC                               IMX_PIN_NUM_GPIO4_IO21
#define IMX_PAD_ENET2_RX_CTL                            IMX_PIN_NUM_GPIO4_IO22
#define IMX_PAD_ENET2_RXC                               IMX_PIN_NUM_GPIO4_IO23
#define IMX_PAD_ENET2_RD0                               IMX_PIN_NUM_GPIO4_IO24
#define IMX_PAD_ENET2_RD1                               IMX_PIN_NUM_GPIO4_IO25
#define IMX_PAD_ENET2_RD2                               IMX_PIN_NUM_GPIO4_IO26
#define IMX_PAD_ENET2_RD3                               IMX_PIN_NUM_GPIO4_IO27
#define IMX_PAD_CCM_CLKO3                               IMX_PIN_NUM_GPIO4_IO28
#define IMX_PAD_CCM_CLKO4                               IMX_PIN_NUM_GPIO4_IO29
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO4_IO30
// No pad for this GPIO pin                             IMX_PIN_NUM_GPIO4_IO31

#endif  // _IMX_GPIO_H_