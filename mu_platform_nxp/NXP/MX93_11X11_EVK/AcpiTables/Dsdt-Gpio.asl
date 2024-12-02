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

#include "iMX93/IMX9Gpio_IoMap.h"

Device (GPIO)
{
    Name (_HID, "NXP0124")
    Name (_UID, 0x0)
    Method (_STA)
    {
        Return(0xf)
    }
    Method (_CRS, 0x0, NotSerialized) {
        Name (RBUF, ResourceTemplate () {
                MEMORY32FIXED(ReadWrite, 0x47400000, 0x10000, )  // GPIO_1
                MEMORY32FIXED(ReadWrite, 0x43810000, 0x10000, )  // GPIO_2
                MEMORY32FIXED(ReadWrite, 0x43820000, 0x10000, )  // GPIO_3
                MEMORY32FIXED(ReadWrite, 0x43830000, 0x10000, )  // GPIO_4
                MEMORY32FIXED(ReadWrite, 0x443C0000, 0x10000, )  // IOMUXC
                Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) {  42 }   // GPIO_1 ( 10 + 32)
                Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) {  89 }   // GPIO_2 ( 57 + 32)
                Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) {  91 }   // GPIO_3 ( 59 + 32)
                Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 221 }   // GPIO_4 (221 + 32)
        })
        Return(RBUF)
    }
    Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
            Package (2) {"Pad_Config",
                //         GPIO                    PAD Control Register offset           MUX Control register offset       PAD_GPIO_ALT   GPIO Name    Pad Name
                Buffer () {IMX_PIN_NUM_GPIO1_IO00, IMX_SW_PAD_CTL_PAD_I2C1_SCL         , IMX_SW_MUX_CTL_PAD_I2C1_SCL         , ALT5 ,  // GPIO1_IO00,  Pad: I2C1_SCL
                           IMX_PIN_NUM_GPIO1_IO01, IMX_SW_PAD_CTL_PAD_I2C1_SDA         , IMX_SW_MUX_CTL_PAD_I2C1_SDA         , ALT5 ,  // GPIO1_IO01,  Pad: I2C1_SDA
                           IMX_PIN_NUM_GPIO1_IO02, IMX_SW_PAD_CTL_PAD_I2C2_SCL         , IMX_SW_MUX_CTL_PAD_I2C2_SCL         , ALT5 ,  // GPIO1_IO02,  Pad: I2C2_SCL
                           IMX_PIN_NUM_GPIO1_IO03, IMX_SW_PAD_CTL_PAD_I2C2_SDA         , IMX_SW_MUX_CTL_PAD_I2C2_SDA         , ALT5 ,  // GPIO1_IO03,  Pad: I2C2_SDA
                           IMX_PIN_NUM_GPIO1_IO04, IMX_SW_PAD_CTL_PAD_UART1_RXD        , IMX_SW_MUX_CTL_PAD_UART1_RXD        , ALT5 ,  // GPIO1_IO04,  Pad: UART1_RXD
                           IMX_PIN_NUM_GPIO1_IO05, IMX_SW_PAD_CTL_PAD_UART1_TXD        , IMX_SW_MUX_CTL_PAD_UART1_TXD        , ALT5 ,  // GPIO1_IO05,  Pad: UART1_TXD
                           IMX_PIN_NUM_GPIO1_IO06, IMX_SW_PAD_CTL_PAD_UART2_RXD        , IMX_SW_MUX_CTL_PAD_UART2_RXD        , ALT5 ,  // GPIO1_IO06,  Pad: UART2_RXD
                           IMX_PIN_NUM_GPIO1_IO07, IMX_SW_PAD_CTL_PAD_UART2_TXD        , IMX_SW_MUX_CTL_PAD_UART2_TXD        , ALT5 ,  // GPIO1_IO07,  Pad: UART2_TXD
                           IMX_PIN_NUM_GPIO1_IO08, IMX_SW_PAD_CTL_PAD_PDM_CLK          , IMX_SW_MUX_CTL_PAD_PDM_CLK          , ALT5 ,  // GPIO1_IO08,  Pad: PDM_CLK
                           IMX_PIN_NUM_GPIO1_IO09, IMX_SW_PAD_CTL_PAD_PDM_BIT_STREAM0  , IMX_SW_MUX_CTL_PAD_PDM_BIT_STREAM0  , ALT5 ,  // GPIO1_IO09,  Pad: PDM_BIT_STREAM0
                           IMX_PIN_NUM_GPIO1_IO10, IMX_SW_PAD_CTL_PAD_PDM_BIT_STREAM1  , IMX_SW_MUX_CTL_PAD_PDM_BIT_STREAM1  , ALT5 ,  // GPIO1_IO10,  Pad: PDM_BIT_STREAM1
                           IMX_PIN_NUM_GPIO1_IO11, IMX_SW_PAD_CTL_PAD_SAI1_TXFS        , IMX_SW_MUX_CTL_PAD_SAI1_TXFS        , ALT5 ,  // GPIO1_IO11,  Pad: SAI1_TXFS
                           IMX_PIN_NUM_GPIO1_IO12, IMX_SW_PAD_CTL_PAD_SAI1_TXC         , IMX_SW_MUX_CTL_PAD_SAI1_TXC         , ALT5 ,  // GPIO1_IO12,  Pad: SAI1_TXC
                           IMX_PIN_NUM_GPIO1_IO13, IMX_SW_PAD_CTL_PAD_SAI1_TXD0        , IMX_SW_MUX_CTL_PAD_SAI1_TXD0        , ALT5 ,  // GPIO1_IO13,  Pad: SAI1_TXD0
                           IMX_PIN_NUM_GPIO1_IO14, IMX_SW_PAD_CTL_PAD_SAI1_RXD0        , IMX_SW_MUX_CTL_PAD_SAI1_RXD0        , ALT5 ,  // GPIO1_IO14,  Pad: SAI1_RXD0
                           IMX_PIN_NUM_GPIO1_IO15, IMX_SW_PAD_CTL_PAD_WDOG_ANY         , IMX_SW_MUX_CTL_PAD_WDOG_ANY         , ALT5 ,  // GPIO1_IO15,  Pad: WDOG_ANY

                           IMX_PIN_NUM_GPIO2_IO00, IMX_SW_PAD_CTL_PAD_GPIO_IO00        , IMX_SW_MUX_CTL_PAD_GPIO_IO00        , ALT0 ,  // GPIO2_IO00,  Pad: GPIO_IO00
                           IMX_PIN_NUM_GPIO2_IO01, IMX_SW_PAD_CTL_PAD_GPIO_IO01        , IMX_SW_MUX_CTL_PAD_GPIO_IO01        , ALT0 ,  // GPIO2_IO01,  Pad: GPIO_IO01
                           IMX_PIN_NUM_GPIO2_IO02, IMX_SW_PAD_CTL_PAD_GPIO_IO02        , IMX_SW_MUX_CTL_PAD_GPIO_IO02        , ALT0 ,  // GPIO2_IO02,  Pad: GPIO_IO02
                           IMX_PIN_NUM_GPIO2_IO03, IMX_SW_PAD_CTL_PAD_GPIO_IO03        , IMX_SW_MUX_CTL_PAD_GPIO_IO03        , ALT0 ,  // GPIO2_IO03,  Pad: GPIO_IO03
                           IMX_PIN_NUM_GPIO2_IO04, IMX_SW_PAD_CTL_PAD_GPIO_IO04        , IMX_SW_MUX_CTL_PAD_GPIO_IO04        , ALT0 ,  // GPIO2_IO04,  Pad: GPIO_IO04
                           IMX_PIN_NUM_GPIO2_IO05, IMX_SW_PAD_CTL_PAD_GPIO_IO05        , IMX_SW_MUX_CTL_PAD_GPIO_IO05        , ALT0 ,  // GPIO2_IO05,  Pad: GPIO_IO05
                           IMX_PIN_NUM_GPIO2_IO06, IMX_SW_PAD_CTL_PAD_GPIO_IO06        , IMX_SW_MUX_CTL_PAD_GPIO_IO06        , ALT0 ,  // GPIO2_IO06,  Pad: GPIO_IO06
                           IMX_PIN_NUM_GPIO2_IO07, IMX_SW_PAD_CTL_PAD_GPIO_IO07        , IMX_SW_MUX_CTL_PAD_GPIO_IO07        , ALT0 ,  // GPIO2_IO07,  Pad: GPIO_IO07
                           IMX_PIN_NUM_GPIO2_IO08, IMX_SW_PAD_CTL_PAD_GPIO_IO08        , IMX_SW_MUX_CTL_PAD_GPIO_IO08        , ALT0 ,  // GPIO2_IO08,  Pad: GPIO_IO08
                           IMX_PIN_NUM_GPIO2_IO09, IMX_SW_PAD_CTL_PAD_GPIO_IO09        , IMX_SW_MUX_CTL_PAD_GPIO_IO09        , ALT0 ,  // GPIO2_IO09,  Pad: GPIO_IO09
                           IMX_PIN_NUM_GPIO2_IO10, IMX_SW_PAD_CTL_PAD_GPIO_IO10        , IMX_SW_MUX_CTL_PAD_GPIO_IO10        , ALT0 ,  // GPIO2_IO10,  Pad: GPIO_IO10
                           IMX_PIN_NUM_GPIO2_IO11, IMX_SW_PAD_CTL_PAD_GPIO_IO11        , IMX_SW_MUX_CTL_PAD_GPIO_IO11        , ALT0 ,  // GPIO2_IO11,  Pad: GPIO_IO11
                           IMX_PIN_NUM_GPIO2_IO12, IMX_SW_PAD_CTL_PAD_GPIO_IO12        , IMX_SW_MUX_CTL_PAD_GPIO_IO12        , ALT0 ,  // GPIO2_IO12,  Pad: GPIO_IO12
                           IMX_PIN_NUM_GPIO2_IO13, IMX_SW_PAD_CTL_PAD_GPIO_IO13        , IMX_SW_MUX_CTL_PAD_GPIO_IO13        , ALT0 ,  // GPIO2_IO13,  Pad: GPIO_IO13
                           IMX_PIN_NUM_GPIO2_IO14, IMX_SW_PAD_CTL_PAD_GPIO_IO14        , IMX_SW_MUX_CTL_PAD_GPIO_IO14        , ALT0 ,  // GPIO2_IO14,  Pad: GPIO_IO14
                           IMX_PIN_NUM_GPIO2_IO15, IMX_SW_PAD_CTL_PAD_GPIO_IO15        , IMX_SW_MUX_CTL_PAD_GPIO_IO15        , ALT0 ,  // GPIO2_IO15,  Pad: GPIO_IO15
                           IMX_PIN_NUM_GPIO2_IO16, IMX_SW_PAD_CTL_PAD_GPIO_IO16        , IMX_SW_MUX_CTL_PAD_GPIO_IO16        , ALT0 ,  // GPIO2_IO16,  Pad: GPIO_IO16
                           IMX_PIN_NUM_GPIO2_IO17, IMX_SW_PAD_CTL_PAD_GPIO_IO17        , IMX_SW_MUX_CTL_PAD_GPIO_IO17        , ALT0 ,  // GPIO2_IO17,  Pad: GPIO_IO17
                           IMX_PIN_NUM_GPIO2_IO18, IMX_SW_PAD_CTL_PAD_GPIO_IO18        , IMX_SW_MUX_CTL_PAD_GPIO_IO18        , ALT0 ,  // GPIO2_IO18,  Pad: GPIO_IO18
                           IMX_PIN_NUM_GPIO2_IO19, IMX_SW_PAD_CTL_PAD_GPIO_IO19        , IMX_SW_MUX_CTL_PAD_GPIO_IO19        , ALT0 ,  // GPIO2_IO19,  Pad: GPIO_IO19
                           IMX_PIN_NUM_GPIO2_IO20, IMX_SW_PAD_CTL_PAD_GPIO_IO20        , IMX_SW_MUX_CTL_PAD_GPIO_IO20        , ALT0 ,  // GPIO2_IO20,  Pad: GPIO_IO20
                           IMX_PIN_NUM_GPIO2_IO21, IMX_SW_PAD_CTL_PAD_GPIO_IO21        , IMX_SW_MUX_CTL_PAD_GPIO_IO21        , ALT0 ,  // GPIO2_IO21,  Pad: GPIO_IO21
                           IMX_PIN_NUM_GPIO2_IO22, IMX_SW_PAD_CTL_PAD_GPIO_IO22        , IMX_SW_MUX_CTL_PAD_GPIO_IO22        , ALT0 ,  // GPIO2_IO22,  Pad: GPIO_IO22
                           IMX_PIN_NUM_GPIO2_IO23, IMX_SW_PAD_CTL_PAD_GPIO_IO23        , IMX_SW_MUX_CTL_PAD_GPIO_IO23        , ALT0 ,  // GPIO2_IO23,  Pad: GPIO_IO23
                           IMX_PIN_NUM_GPIO2_IO24, IMX_SW_PAD_CTL_PAD_GPIO_IO24        , IMX_SW_MUX_CTL_PAD_GPIO_IO24        , ALT0 ,  // GPIO2_IO24,  Pad: GPIO_IO24
                           IMX_PIN_NUM_GPIO2_IO25, IMX_SW_PAD_CTL_PAD_GPIO_IO25        , IMX_SW_MUX_CTL_PAD_GPIO_IO25        , ALT0 ,  // GPIO2_IO25,  Pad: GPIO_IO25
                           IMX_PIN_NUM_GPIO2_IO26, IMX_SW_PAD_CTL_PAD_GPIO_IO26        , IMX_SW_MUX_CTL_PAD_GPIO_IO26        , ALT0 ,  // GPIO2_IO26,  Pad: GPIO_IO26
                           IMX_PIN_NUM_GPIO2_IO27, IMX_SW_PAD_CTL_PAD_GPIO_IO27        , IMX_SW_MUX_CTL_PAD_GPIO_IO27        , ALT0 ,  // GPIO2_IO27,  Pad: GPIO_IO27
                           IMX_PIN_NUM_GPIO2_IO28, IMX_SW_PAD_CTL_PAD_GPIO_IO28        , IMX_SW_MUX_CTL_PAD_GPIO_IO28        , ALT0 ,  // GPIO2_IO28,  Pad: GPIO_IO28
                           IMX_PIN_NUM_GPIO2_IO29, IMX_SW_PAD_CTL_PAD_GPIO_IO29        , IMX_SW_MUX_CTL_PAD_GPIO_IO29        , ALT0 ,  // GPIO2_IO29,  Pad: GPIO_IO29

                           IMX_PIN_NUM_GPIO3_IO00, IMX_SW_PAD_CTL_PAD_SD2_CD_B         , IMX_SW_MUX_CTL_PAD_SD2_CD_B         , ALT5 ,  // GPIO3_IO00,  Pad: SD2_CD_B
                           IMX_PIN_NUM_GPIO3_IO01, IMX_SW_PAD_CTL_PAD_SD2_CLK          , IMX_SW_MUX_CTL_PAD_SD2_CLK          , ALT5 ,  // GPIO3_IO01,  Pad: SD2_CLK
                           IMX_PIN_NUM_GPIO3_IO02, IMX_SW_PAD_CTL_PAD_SD2_CMD          , IMX_SW_MUX_CTL_PAD_SD2_CMD          , ALT5 ,  // GPIO3_IO02,  Pad: SD2_CMD
                           IMX_PIN_NUM_GPIO3_IO03, IMX_SW_PAD_CTL_PAD_SD2_DATA0        , IMX_SW_MUX_CTL_PAD_SD2_DATA0        , ALT5 ,  // GPIO3_IO03,  Pad: SD2_DATA0
                           IMX_PIN_NUM_GPIO3_IO04, IMX_SW_PAD_CTL_PAD_SD2_DATA1        , IMX_SW_MUX_CTL_PAD_SD2_DATA1        , ALT5 ,  // GPIO3_IO04,  Pad: SD2_DATA1
                           IMX_PIN_NUM_GPIO3_IO05, IMX_SW_PAD_CTL_PAD_SD2_DATA2        , IMX_SW_MUX_CTL_PAD_SD2_DATA2        , ALT5 ,  // GPIO3_IO05,  Pad: SD2_DATA2
                           IMX_PIN_NUM_GPIO3_IO06, IMX_SW_PAD_CTL_PAD_SD2_DATA3        , IMX_SW_MUX_CTL_PAD_SD2_DATA3        , ALT5 ,  // GPIO3_IO06,  Pad: SD2_DATA3
                           IMX_PIN_NUM_GPIO3_IO07, IMX_SW_PAD_CTL_PAD_SD2_RESET_B      , IMX_SW_MUX_CTL_PAD_SD2_RESET_B      , ALT5 ,  // GPIO3_IO07,  Pad: SD2_RESET_B
                           IMX_PIN_NUM_GPIO3_IO08, IMX_SW_PAD_CTL_PAD_SD1_CLK          , IMX_SW_MUX_CTL_PAD_SD1_CLK          , ALT5 ,  // GPIO3_IO08,  Pad: SD1_CLK
                           IMX_PIN_NUM_GPIO3_IO09, IMX_SW_PAD_CTL_PAD_SD1_CMD          , IMX_SW_MUX_CTL_PAD_SD1_CMD          , ALT5 ,  // GPIO3_IO09,  Pad: SD1_CMD
                           IMX_PIN_NUM_GPIO3_IO10, IMX_SW_PAD_CTL_PAD_SD1_DATA0        , IMX_SW_MUX_CTL_PAD_SD1_DATA0        , ALT5 ,  // GPIO3_IO10,  Pad: SD1_DATA0
                           IMX_PIN_NUM_GPIO3_IO11, IMX_SW_PAD_CTL_PAD_SD1_DATA1        , IMX_SW_MUX_CTL_PAD_SD1_DATA1        , ALT5 ,  // GPIO3_IO11,  Pad: SD1_DATA1
                           IMX_PIN_NUM_GPIO3_IO12, IMX_SW_PAD_CTL_PAD_SD1_DATA2        , IMX_SW_MUX_CTL_PAD_SD1_DATA2        , ALT5 ,  // GPIO3_IO12,  Pad: SD1_DATA2
                           IMX_PIN_NUM_GPIO3_IO13, IMX_SW_PAD_CTL_PAD_SD1_DATA3        , IMX_SW_MUX_CTL_PAD_SD1_DATA3        , ALT5 ,  // GPIO3_IO13,  Pad: SD1_DATA3
                           IMX_PIN_NUM_GPIO3_IO14, IMX_SW_PAD_CTL_PAD_SD1_DATA4        , IMX_SW_MUX_CTL_PAD_SD1_DATA4        , ALT5 ,  // GPIO3_IO14,  Pad: SD1_DATA4
                           IMX_PIN_NUM_GPIO3_IO15, IMX_SW_PAD_CTL_PAD_SD1_DATA5        , IMX_SW_MUX_CTL_PAD_SD1_DATA5        , ALT5 ,  // GPIO3_IO15,  Pad: SD1_DATA5
                           IMX_PIN_NUM_GPIO3_IO16, IMX_SW_PAD_CTL_PAD_SD1_DATA6        , IMX_SW_MUX_CTL_PAD_SD1_DATA6        , ALT5 ,  // GPIO3_IO16,  Pad: SD1_DATA6
                           IMX_PIN_NUM_GPIO3_IO17, IMX_SW_PAD_CTL_PAD_SD1_DATA7        , IMX_SW_MUX_CTL_PAD_SD1_DATA7        , ALT5 ,  // GPIO3_IO17,  Pad: SD1_DATA7
                           IMX_PIN_NUM_GPIO3_IO18, IMX_SW_PAD_CTL_PAD_SD1_STROBE       , IMX_SW_MUX_CTL_PAD_SD1_STROBE       , ALT5 ,  // GPIO3_IO18,  Pad: SD1_STROBE
                           IMX_PIN_NUM_GPIO3_IO19, IMX_SW_PAD_CTL_PAD_SD2_VSELECT      , IMX_SW_MUX_CTL_PAD_SD2_VSELECT      , ALT5 ,  // GPIO3_IO19,  Pad: SD2_VSELECT
                           IMX_PIN_NUM_GPIO3_IO20, IMX_SW_PAD_CTL_PAD_SD3_CLK          , IMX_SW_MUX_CTL_PAD_SD3_CLK          , ALT5 ,  // GPIO3_IO20,  Pad: SD3_CLK
                           IMX_PIN_NUM_GPIO3_IO21, IMX_SW_PAD_CTL_PAD_SD3_CMD          , IMX_SW_MUX_CTL_PAD_SD3_CMD          , ALT5 ,  // GPIO3_IO21,  Pad: SD3_CMD
                           IMX_PIN_NUM_GPIO3_IO22, IMX_SW_PAD_CTL_PAD_SD3_DATA0        , IMX_SW_MUX_CTL_PAD_SD3_DATA0        , ALT5 ,  // GPIO3_IO22,  Pad: SD3_DATA0
                           IMX_PIN_NUM_GPIO3_IO23, IMX_SW_PAD_CTL_PAD_SD3_DATA1        , IMX_SW_MUX_CTL_PAD_SD3_DATA1        , ALT5 ,  // GPIO3_IO23,  Pad: SD3_DATA1
                           IMX_PIN_NUM_GPIO3_IO24, IMX_SW_PAD_CTL_PAD_SD3_DATA2        , IMX_SW_MUX_CTL_PAD_SD3_DATA2        , ALT5 ,  // GPIO3_IO24,  Pad: SD3_DATA2
                           IMX_PIN_NUM_GPIO3_IO25, IMX_SW_PAD_CTL_PAD_SD3_DATA3        , IMX_SW_MUX_CTL_PAD_SD3_DATA3        , ALT5 ,  // GPIO3_IO25,  Pad: SD3_DATA3
                           IMX_PIN_NUM_GPIO3_IO26, IMX_SW_PAD_CTL_PAD_CCM_CLKO1        , IMX_SW_MUX_CTL_PAD_CCM_CLKO1        , ALT5 ,  // GPIO3_IO26,  Pad: CCM_CLKO1
                           IMX_PIN_NUM_GPIO3_IO27, IMX_SW_PAD_CTL_PAD_CCM_CLKO2        , IMX_SW_MUX_CTL_PAD_CCM_CLKO2        , ALT5 ,  // GPIO3_IO27,  Pad: CCM_CLKO2
                           IMX_PIN_NUM_GPIO3_IO28, IMX_SW_PAD_CTL_PAD_DAP_TDI          , IMX_SW_MUX_CTL_PAD_DAP_TDI          , ALT5 ,  // GPIO3_IO28,  Pad: DAP_TDI
                           IMX_PIN_NUM_GPIO3_IO29, IMX_SW_PAD_CTL_PAD_DAP_TMS_SWDIO    , IMX_SW_MUX_CTL_PAD_DAP_TMS_SWDIO    , ALT5 ,  // GPIO3_IO29,  Pad: DAP_TMS_SWDIO
                           IMX_PIN_NUM_GPIO3_IO30, IMX_SW_PAD_CTL_PAD_DAP_TCLK_SWCLK   , IMX_SW_MUX_CTL_PAD_DAP_TCLK_SWCLK   , ALT5 ,  // GPIO3_IO30,  Pad: DAP_TCLK_SWCLK
                           IMX_PIN_NUM_GPIO3_IO31, IMX_SW_PAD_CTL_PAD_DAP_TDO_TRACESWO , IMX_SW_MUX_CTL_PAD_DAP_TDO_TRACESWO , ALT5 ,  // GPIO3_IO31,  Pad: DAP_TDO_TRACESWO

                           IMX_PIN_NUM_GPIO4_IO00, IMX_SW_PAD_CTL_PAD_ENET1_MDC        , IMX_SW_MUX_CTL_PAD_ENET1_MDC        , ALT5 ,  // GPIO4_IO00,  Pad: ENET1_MDC
                           IMX_PIN_NUM_GPIO4_IO01, IMX_SW_PAD_CTL_PAD_ENET1_MDIO       , IMX_SW_MUX_CTL_PAD_ENET1_MDIO       , ALT5 ,  // GPIO4_IO01,  Pad: ENET1_MDIO
                           IMX_PIN_NUM_GPIO4_IO02, IMX_SW_PAD_CTL_PAD_ENET1_TD3        , IMX_SW_MUX_CTL_PAD_ENET1_TD3        , ALT5 ,  // GPIO4_IO02,  Pad: ENET1_TD3
                           IMX_PIN_NUM_GPIO4_IO03, IMX_SW_PAD_CTL_PAD_ENET1_TD2        , IMX_SW_MUX_CTL_PAD_ENET1_TD2        , ALT5 ,  // GPIO4_IO03,  Pad: ENET1_TD2
                           IMX_PIN_NUM_GPIO4_IO04, IMX_SW_PAD_CTL_PAD_ENET1_TD1        , IMX_SW_MUX_CTL_PAD_ENET1_TD1        , ALT5 ,  // GPIO4_IO04,  Pad: ENET1_TD1
                           IMX_PIN_NUM_GPIO4_IO05, IMX_SW_PAD_CTL_PAD_ENET1_TD0        , IMX_SW_MUX_CTL_PAD_ENET1_TD0        , ALT5 ,  // GPIO4_IO05,  Pad: ENET1_TD0
                           IMX_PIN_NUM_GPIO4_IO06, IMX_SW_PAD_CTL_PAD_ENET1_TX_CTL     , IMX_SW_MUX_CTL_PAD_ENET1_TX_CTL     , ALT5 ,  // GPIO4_IO06,  Pad: ENET1_TX_CTL
                           IMX_PIN_NUM_GPIO4_IO07, IMX_SW_PAD_CTL_PAD_ENET1_TXC        , IMX_SW_MUX_CTL_PAD_ENET1_TXC        , ALT5 ,  // GPIO4_IO07,  Pad: ENET1_TXC
                           IMX_PIN_NUM_GPIO4_IO08, IMX_SW_PAD_CTL_PAD_ENET1_RX_CTL     , IMX_SW_MUX_CTL_PAD_ENET1_RX_CTL     , ALT5 ,  // GPIO4_IO08,  Pad: ENET1_RX_CTL
                           IMX_PIN_NUM_GPIO4_IO09, IMX_SW_PAD_CTL_PAD_ENET1_RXC        , IMX_SW_MUX_CTL_PAD_ENET1_RXC        , ALT5 ,  // GPIO4_IO09,  Pad: ENET1_RXC
                           IMX_PIN_NUM_GPIO4_IO10, IMX_SW_PAD_CTL_PAD_ENET1_RD0        , IMX_SW_MUX_CTL_PAD_ENET1_RD0        , ALT5 ,  // GPIO4_IO10,  Pad: ENET1_RD0
                           IMX_PIN_NUM_GPIO4_IO11, IMX_SW_PAD_CTL_PAD_ENET1_RD1        , IMX_SW_MUX_CTL_PAD_ENET1_RD1        , ALT5 ,  // GPIO4_IO11,  Pad: ENET1_RD1
                           IMX_PIN_NUM_GPIO4_IO12, IMX_SW_PAD_CTL_PAD_ENET1_RD2        , IMX_SW_MUX_CTL_PAD_ENET1_RD2        , ALT5 ,  // GPIO4_IO12,  Pad: ENET1_RD2
                           IMX_PIN_NUM_GPIO4_IO13, IMX_SW_PAD_CTL_PAD_ENET1_RD3        , IMX_SW_MUX_CTL_PAD_ENET1_RD3        , ALT5 ,  // GPIO4_IO13,  Pad: ENET1_RD3
                           IMX_PIN_NUM_GPIO4_IO14, IMX_SW_PAD_CTL_PAD_ENET2_MDC        , IMX_SW_MUX_CTL_PAD_ENET2_MDC        , ALT5 ,  // GPIO4_IO14,  Pad: ENET2_MDC
                           IMX_PIN_NUM_GPIO4_IO15, IMX_SW_PAD_CTL_PAD_ENET2_MDIO       , IMX_SW_MUX_CTL_PAD_ENET2_MDIO       , ALT5 ,  // GPIO4_IO15,  Pad: ENET2_MDIO
                           IMX_PIN_NUM_GPIO4_IO16, IMX_SW_PAD_CTL_PAD_ENET2_TD3        , IMX_SW_MUX_CTL_PAD_ENET2_TD3        , ALT5 ,  // GPIO4_IO16,  Pad: ENET2_TD3
                           IMX_PIN_NUM_GPIO4_IO17, IMX_SW_PAD_CTL_PAD_ENET2_TD2        , IMX_SW_MUX_CTL_PAD_ENET2_TD2        , ALT5 ,  // GPIO4_IO17,  Pad: ENET2_TD2
                           IMX_PIN_NUM_GPIO4_IO18, IMX_SW_PAD_CTL_PAD_ENET2_TD1        , IMX_SW_MUX_CTL_PAD_ENET2_TD1        , ALT5 ,  // GPIO4_IO18,  Pad: ENET2_TD1
                           IMX_PIN_NUM_GPIO4_IO19, IMX_SW_PAD_CTL_PAD_ENET2_TD0        , IMX_SW_MUX_CTL_PAD_ENET2_TD0        , ALT5 ,  // GPIO4_IO19,  Pad: ENET2_TD0
                           IMX_PIN_NUM_GPIO4_IO20, IMX_SW_PAD_CTL_PAD_ENET2_TX_CTL     , IMX_SW_MUX_CTL_PAD_ENET2_TX_CTL     , ALT5 ,  // GPIO4_IO20,  Pad: ENET2_TX_CTL
                           IMX_PIN_NUM_GPIO4_IO21, IMX_SW_PAD_CTL_PAD_ENET2_TXC        , IMX_SW_MUX_CTL_PAD_ENET2_TXC        , ALT5 ,  // GPIO4_IO21,  Pad: ENET2_TXC
                           IMX_PIN_NUM_GPIO4_IO22, IMX_SW_PAD_CTL_PAD_ENET2_RX_CTL     , IMX_SW_MUX_CTL_PAD_ENET2_RX_CTL     , ALT5 ,  // GPIO4_IO22,  Pad: ENET2_RX_CTL
                           IMX_PIN_NUM_GPIO4_IO23, IMX_SW_PAD_CTL_PAD_ENET2_RXC        , IMX_SW_MUX_CTL_PAD_ENET2_RXC        , ALT5 ,  // GPIO4_IO23,  Pad: ENET2_RXC
                           IMX_PIN_NUM_GPIO4_IO24, IMX_SW_PAD_CTL_PAD_ENET2_RD0        , IMX_SW_MUX_CTL_PAD_ENET2_RD0        , ALT5 ,  // GPIO4_IO24,  Pad: ENET2_RD0
                           IMX_PIN_NUM_GPIO4_IO25, IMX_SW_PAD_CTL_PAD_ENET2_RD1        , IMX_SW_MUX_CTL_PAD_ENET2_RD1        , ALT5 ,  // GPIO4_IO25,  Pad: ENET2_RD1
                           IMX_PIN_NUM_GPIO4_IO26, IMX_SW_PAD_CTL_PAD_ENET2_RD2        , IMX_SW_MUX_CTL_PAD_ENET2_RD2        , ALT5 ,  // GPIO4_IO26,  Pad: ENET2_RD2
                           IMX_PIN_NUM_GPIO4_IO27, IMX_SW_PAD_CTL_PAD_ENET2_RD3        , IMX_SW_MUX_CTL_PAD_ENET2_RD3        , ALT5 ,  // GPIO4_IO27,  Pad: ENET2_RD3
                           IMX_PIN_NUM_GPIO4_IO28, IMX_SW_PAD_CTL_PAD_CCM_CLKO3        , IMX_SW_MUX_CTL_PAD_CCM_CLKO3        , ALT5 ,  // GPIO4_IO28,  Pad: CCM_CLKO3
                           IMX_PIN_NUM_GPIO4_IO29, IMX_SW_PAD_CTL_PAD_CCM_CLKO4        , IMX_SW_MUX_CTL_PAD_CCM_CLKO4        , ALT5 ,  // GPIO4_IO29,  Pad: CCM_CLKO4
                }
            },
            Package (2) {"Pin_Config",
                //         PAD                      PAD_ALT Input select (DAYISY) reg. offset    DAISY_ALT
                Buffer () {// CAN1_RX
                           IMX_PAD_PDM_BIT_STREAM0,  ALT6, IMX_PIN_CAN1_IPP_IND_CANRX_SELECT_INPUT_REG, ALT0,             // CAN1_RX
                           IMX_PAD_SAI1_TXC,         ALT4, IMX_PIN_CAN1_IPP_IND_CANRX_SELECT_INPUT_REG, ALT1,             // CAN1_RX
                           // CAN2_RX
                           IMX_PAD_DAP_TDO_TRACESWO, ALT3, IMX_PIN_CAN2_IPP_IND_CANRX_SELECT_INPUT_REG, ALT0,             // CAN2_RX
                           IMX_PAD_GPIO_IO27,        ALT2, IMX_PIN_CAN2_IPP_IND_CANRX_SELECT_INPUT_REG, ALT1,             // CAN2_RX
                           IMX_PAD_ENET1_TD2,        ALT2, IMX_PIN_CAN2_IPP_IND_CANRX_SELECT_INPUT_REG, ALT2,             // CAN2_RX
                           IMX_PAD_SD2_DATA1,        ALT2, IMX_PIN_CAN2_IPP_IND_CANRX_SELECT_INPUT_REG, ALT3,             // CAN2_RX
                           // EXT1_CLK
                           IMX_PAD_SD2_VSELECT,      ALT6, IMX_PIN_CCMSRCGPCMIX_EXT1_CLK_SELECT_INPUT_REG, ALT0,          // EXT1_CLK
                           IMX_PAD_PDM_BIT_STREAM1,  ALT6, IMX_PIN_CCMSRCGPCMIX_EXT1_CLK_SELECT_INPUT_REG, ALT1,          // EXT1_CLK
                           // FLEXIO1_0
                           IMX_PAD_GPIO_IO00,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_0_REG, ALT0,       // FLEXIO1_0
                           IMX_PAD_SD2_CD_B,         ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_0_REG, ALT1,       // FLEXIO1_0
                           // FLEXIO1_1
                           IMX_PAD_GPIO_IO01,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_1_REG, ALT0,       // FLEXIO1_1
                           IMX_PAD_SD2_CLK,          ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_1_REG, ALT1,       // FLEXIO1_1
                           // FLEXIO1_2
                           IMX_PAD_GPIO_IO02,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_2_REG, ALT0,       // FLEXIO1_2
                           IMX_PAD_SD2_CMD,          ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_2_REG, ALT1,       // FLEXIO1_2
                           // FLEXIO1_3
                           IMX_PAD_GPIO_IO03,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_3_REG, ALT0,       // FLEXIO1_3
                           IMX_PAD_SD2_DATA0,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_3_REG, ALT1,       // FLEXIO1_3
                           // FLEXIO1_4
                           IMX_PAD_GPIO_IO04,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_4_REG, ALT0,       // FLEXIO1_4
                           IMX_PAD_SD2_DATA1,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_4_REG, ALT1,       // FLEXIO1_4
                           // FLEXIO1_5
                           IMX_PAD_GPIO_IO05,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_5_REG, ALT0,       // FLEXIO1_5
                           IMX_PAD_SD2_DATA2,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_5_REG, ALT1,       // FLEXIO1_5
                           // FLEXIO1_6
                           IMX_PAD_GPIO_IO06,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_6_REG, ALT0,       // FLEXIO1_6
                           IMX_PAD_SD2_DATA3,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_6_REG, ALT1,       // FLEXIO1_6
                           // FLEXIO1_7
                           IMX_PAD_GPIO_IO07,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_7_REG, ALT0,       // FLEXIO1_7
                           IMX_PAD_SD2_RESET_B,      ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_7_REG, ALT1,       // FLEXIO1_7
                           // FLEXIO1_8
                           IMX_PAD_GPIO_IO08,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_8_REG, ALT0,       // FLEXIO1_8
                           IMX_PAD_SD1_CLK,          ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_8_REG, ALT1,       // FLEXIO1_8
                           // FLEXIO1_9
                           IMX_PAD_GPIO_IO09,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_9_REG, ALT0,       // FLEXIO1_9
                           IMX_PAD_SD1_CMD,          ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_9_REG, ALT1,       // FLEXIO1_9
                           // FLEXIO1_10
                           IMX_PAD_GPIO_IO10,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_10_REG, ALT0,      // FLEXIO1_10
                           IMX_PAD_SD1_DATA0,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_10_REG, ALT1,      // FLEXIO1_10
                           // FLEXIO1_14
                           IMX_PAD_GPIO_IO11,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_11_REG, ALT0,      // FLEXIO1_11
                           IMX_PAD_SD1_DATA1,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_11_REG, ALT1,      // FLEXIO1_11
                           // FLEXIO1_13
                           IMX_PAD_GPIO_IO13,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_13_REG, ALT0,      // FLEXIO1_13
                           IMX_PAD_SD1_DATA3,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_13_REG, ALT1,      // FLEXIO1_13
                           // FLEXIO1_14
                           IMX_PAD_GPIO_IO14,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_14_REG, ALT0,      // FLEXIO1_14
                           IMX_PAD_SD1_DATA4,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_14_REG, ALT1,      // FLEXIO1_14
                           // FLEXIO1_15
                           IMX_PAD_GPIO_IO15,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_15_REG, ALT0,      // FLEXIO1_15
                           IMX_PAD_SD1_DATA5,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_15_REG, ALT1,      // FLEXIO1_15
                           // FLEXIO1_16
                           IMX_PAD_GPIO_IO16,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_16_REG, ALT0,      // FLEXIO1_16
                           IMX_PAD_SD1_DATA6,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_16_REG, ALT1,      // FLEXIO1_16
                           // FLEXIO1_17
                           IMX_PAD_GPIO_IO17,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_17_REG, ALT0,      // FLEXIO1_17
                           IMX_PAD_SD1_DATA7,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_17_REG, ALT1,      // FLEXIO1_17
                           // FLEXIO1_18
                           IMX_PAD_GPIO_IO18,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_18_REG, ALT0,      // FLEXIO1_18
                           IMX_PAD_SD1_STROBE,       ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_18_REG, ALT1,      // FLEXIO1_18
                           // FLEXIO1_20
                           IMX_PAD_GPIO_IO20,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_20_REG, ALT0,      // FLEXIO1_20
                           IMX_PAD_SD3_CLK,          ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_20_REG, ALT1,      // FLEXIO1_20
                           // FLEXIO1_22
                           IMX_PAD_GPIO_IO22,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_22_REG, ALT0,      // FLEXIO1_22
                           IMX_PAD_SD3_DATA0,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_22_REG, ALT1,      // FLEXIO1_22
                           // FLEXIO1_23
                           IMX_PAD_GPIO_IO23,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_23_REG, ALT0,      // FLEXIO1_23
                           IMX_PAD_SD3_DATA1,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_23_REG, ALT1,      // FLEXIO1_23
                           // FLEXIO1_24
                           IMX_PAD_GPIO_IO24,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_24_REG, ALT0,      // FLEXIO1_24
                           IMX_PAD_SD3_DATA2,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_24_REG, ALT1,      // FLEXIO1_24
                           // FLEXIO1_25
                           IMX_PAD_GPIO_IO25,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_25_REG, ALT0,      // FLEXIO1_25
                           IMX_PAD_SD3_DATA3,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_25_REG, ALT1,      // FLEXIO1_25
                           // FLEXIO1_27
                           IMX_PAD_GPIO_IO27,        ALT7, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_27_REG, ALT0,      // FLEXIO1_27
                           IMX_PAD_CCM_CLKO2,        ALT4, IMX_PIN_FLEXIO1_IPP_IND_FLEXIO_SELECT_INPUT_27_REG, ALT1,      // FLEXIO1_27
                           // I3C2_SCL
                           IMX_PAD_ENET1_MDC,        ALT2, IMX_PIN_I3C2_PIN_SCL_IN_SELECT_INPUT_REG, ALT0,                // I3C2_SCL
                           IMX_PAD_SD2_CD_B,         ALT2, IMX_PIN_I3C2_PIN_SCL_IN_SELECT_INPUT_REG, ALT1,                // I3C2_SCL
                           // I3C2_SDA
                           IMX_PAD_ENET1_MDIO,       ALT2, IMX_PIN_I3C2_PIN_SDA_IN_SELECT_INPUT_REG, ALT0,                // I3C2_SDA
                           IMX_PAD_SD2_CLK,          ALT2, IMX_PIN_I3C2_PIN_SDA_IN_SELECT_INPUT_REG, ALT1,                // I3C2_SDA
                           // JTAG_MUX_TCK
                           IMX_PAD_DAP_TCLK_SWCLK,   ALT0, IMX_PIN_JTAG_MUX_TCK_SELECT_INPUT_REG, ALT0,                   // JTAG_MUX_TCK
                           IMX_PAD_GPIO_IO25,        ALT5, IMX_PIN_JTAG_MUX_TCK_SELECT_INPUT_REG, ALT1,                   // JTAG_MUX_TCK
                           // JTAG_MUX_TDI
                           IMX_PAD_DAP_TDI,          ALT0, IMX_PIN_JTAG_MUX_TDI_SELECT_INPUT_REG, ALT0,                   // JTAG_MUX_TDI
                           IMX_PAD_GPIO_IO26,        ALT5, IMX_PIN_JTAG_MUX_TDI_SELECT_INPUT_REG, ALT1,                   // JTAG_MUX_TDI
                           // JTAG_MUX_TMS
                           IMX_PAD_DAP_TMS_SWDIO,    ALT0, IMX_PIN_JTAG_MUX_TMS_SELECT_INPUT_REG, ALT0,                   // JTAG_MUX_TMS
                           IMX_PAD_GPIO_IO27,        ALT5, IMX_PIN_JTAG_MUX_TMS_SELECT_INPUT_REG, ALT1,                   // JTAG_MUX_TMS
                           // LPI2C3_SCL
                           IMX_PAD_GPIO_IO01,        ALT1, IMX_PIN_LPI2C3_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG, ALT0,       // LPI2C3_SCL
                           IMX_PAD_GPIO_IO29,        ALT1, IMX_PIN_LPI2C3_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG, ALT1,       // LPI2C3_SCL
                           // LPI2C3_SDA
                           IMX_PAD_GPIO_IO00,        ALT1, IMX_PIN_LPI2C3_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG, ALT0,       // LPI2C3_SDA
                           IMX_PAD_GPIO_IO28,        ALT1, IMX_PIN_LPI2C3_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG, ALT1,       // LPI2C3_SDA
                           // LPI2C5_SCL
                           IMX_PAD_GPIO_IO01,        ALT6, IMX_PIN_LPI2C5_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG, ALT0,       // LPI2C5_SCL
                           IMX_PAD_GPIO_IO23,        ALT6, IMX_PIN_LPI2C5_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG, ALT1,       // LPI2C5_SCL
                           // LPI2C5_SDA
                           IMX_PAD_GPIO_IO00,        ALT6, IMX_PIN_LPI2C5_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG, ALT0,       // LPI2C5_SDA
                           IMX_PAD_GPIO_IO22,        ALT6, IMX_PIN_LPI2C5_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG, ALT1,       // LPI2C5_SDA
                           // LPI2C6_SCL
                           IMX_PAD_GPIO_IO03,        ALT6, IMX_PIN_LPI2C6_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG, ALT0,       // LPI2C6_SCL
                           IMX_PAD_GPIO_IO05,        ALT6, IMX_PIN_LPI2C6_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG, ALT1,       // LPI2C6_SCL
                           // LPI2C6_SDA
                           IMX_PAD_GPIO_IO02,        ALT6, IMX_PIN_LPI2C6_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG, ALT0,       // LPI2C6_SDA
                           IMX_PAD_GPIO_IO04,        ALT6, IMX_PIN_LPI2C6_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG, ALT1,       // LPI2C6_SDA
                           // LPI2C7_SCL
                           IMX_PAD_GPIO_IO07,        ALT6, IMX_PIN_LPI2C7_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG, ALT0,       // LPI2C7_SCL
                           IMX_PAD_GPIO_IO09,        ALT6, IMX_PIN_LPI2C7_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG, ALT1,       // LPI2C7_SCL
                           // LPI2C7_SDA
                           IMX_PAD_GPIO_IO06,        ALT6, IMX_PIN_LPI2C7_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG, ALT0,       // LPI2C7_SDA
                           IMX_PAD_GPIO_IO08,        ALT6, IMX_PIN_LPI2C7_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG, ALT1,       // LPI2C7_SDA
                           // LPI2C8_SCL
                           IMX_PAD_GPIO_IO11,        ALT6, IMX_PIN_LPI2C8_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG, ALT0,       // LPI2C8_SCL
                           IMX_PAD_GPIO_IO13,        ALT6, IMX_PIN_LPI2C8_IPP_IND_LPI2C_SCL_SELECT_INPUT_REG, ALT1,       // LPI2C8_SCL
                           // LPI2C8_SDA
                           IMX_PAD_GPIO_IO10,        ALT6, IMX_PIN_LPI2C8_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG, ALT0,       // LPI2C8_SDA
                           IMX_PAD_GPIO_IO12,        ALT6, IMX_PIN_LPI2C8_IPP_IND_LPI2C_SDA_SELECT_INPUT_REG, ALT1,       // LPI2C8_SDA
                           // LPTMR2_TMR0
                           IMX_PAD_ENET1_RD1,        ALT3, IMX_PIN_LPTMR2_IPP_IND_LPTIMER_SELECT_INPUT_0_REG, ALT0,       // LPTMR2_TMR0
                           IMX_PAD_SD2_DATA3,        ALT1, IMX_PIN_LPTMR2_IPP_IND_LPTIMER_SELECT_INPUT_0_REG, ALT1,       // LPTMR2_TMR0
                           // LPTMR2_TMR1
                           IMX_PAD_ENET1_RD2,        ALT3, IMX_PIN_LPTMR2_IPP_IND_LPTIMER_SELECT_INPUT_1_REG, ALT0,       // LPTMR2_TMR1
                           IMX_PAD_SD2_RESET_B,      ALT1, IMX_PIN_LPTMR2_IPP_IND_LPTIMER_SELECT_INPUT_1_REG, ALT1,       // LPTMR2_TMR1
                           // LPTMR2_TMR2
                           IMX_PAD_ENET1_RD3,        ALT3, IMX_PIN_LPTMR2_IPP_IND_LPTIMER_SELECT_INPUT_2_REG, ALT0,       // LPTMR2_TMR2
                           IMX_PAD_SD2_VSELECT,      ALT2, IMX_PIN_LPTMR2_IPP_IND_LPTIMER_SELECT_INPUT_2_REG, ALT1,       // LPTMR2_TMR2
                           // LPUART3_CTS_N
                           IMX_PAD_GPIO_IO16,        ALT4, IMX_PIN_LPUART3_IPP_IND_LPUART_CTS_N_SELECT_INPUT_REG, ALT0,   // LPUART3_CTS_N
                           IMX_PAD_ENET1_RD1,        ALT1, IMX_PIN_LPUART3_IPP_IND_LPUART_CTS_N_SELECT_INPUT_REG, ALT1,   // LPUART3_CTS_N
                           // LPUART3_RXD
                           IMX_PAD_GPIO_IO15,        ALT1, IMX_PIN_LPUART3_IPP_IND_LPUART_RXD_SELECT_INPUT_REG, ALT0,     // LPUART3_RXD
                           IMX_PAD_ENET1_RD0,        ALT1, IMX_PIN_LPUART3_IPP_IND_LPUART_RXD_SELECT_INPUT_REG, ALT1,     // LPUART3_RXD
                           // LPUART3_TXD
                           IMX_PAD_GPIO_IO14,        ALT1, IMX_PIN_LPUART3_IPP_IND_LPUART_TXD_SELECT_INPUT_REG, ALT0,     // LPUART3_TXD
                           IMX_PAD_ENET1_TD0,        ALT1, IMX_PIN_LPUART3_IPP_IND_LPUART_TXD_SELECT_INPUT_REG, ALT1,     // LPUART3_TXD
                           // LPUART4_CTS_N
                           IMX_PAD_GPIO_IO16,        ALT6, IMX_PIN_LPUART4_IPP_IND_LPUART_CTS_N_SELECT_INPUT_REG, ALT0,   // LPUART4_CTS_N
                           IMX_PAD_ENET2_RD2,        ALT1, IMX_PIN_LPUART4_IPP_IND_LPUART_CTS_N_SELECT_INPUT_REG, ALT1,   // LPUART4_CTS_N
                           // LPUART4_RXD
                           IMX_PAD_GPIO_IO15,        ALT6, IMX_PIN_LPUART4_IPP_IND_LPUART_RXD_SELECT_INPUT_REG, ALT0,     // LPUART4_RXD
                           IMX_PAD_ENET2_RD0,        ALT1, IMX_PIN_LPUART4_IPP_IND_LPUART_RXD_SELECT_INPUT_REG, ALT1,     // LPUART4_RXD
                           // LPUART4_TXD
                           IMX_PAD_GPIO_IO14,        ALT6, IMX_PIN_LPUART4_IPP_IND_LPUART_TXD_SELECT_INPUT_REG, ALT0,     // LPUART4_TXD
                           IMX_PAD_ENET2_TD0,        ALT1, IMX_PIN_LPUART4_IPP_IND_LPUART_TXD_SELECT_INPUT_REG, ALT1,     // LPUART4_TXD
                           // LPUART5_CTS_N
                           IMX_PAD_DAP_TCLK_SWCLK,   ALT6, IMX_PIN_LPUART5_IPP_IND_LPUART_CTS_N_SELECT_INPUT_REG, ALT0,   // LPUART5_CTS_N
                           IMX_PAD_GPIO_IO02,        ALT5, IMX_PIN_LPUART5_IPP_IND_LPUART_CTS_N_SELECT_INPUT_REG, ALT1,   // LPUART5_CTS_N
                           // LPUART5_RXD
                           IMX_PAD_DAP_TDI,          ALT6, IMX_PIN_LPUART5_IPP_IND_LPUART_RXD_SELECT_INPUT_REG, ALT0,     // LPUART5_RXD
                           IMX_PAD_GPIO_IO01,        ALT5, IMX_PIN_LPUART5_IPP_IND_LPUART_RXD_SELECT_INPUT_REG, ALT1,     // LPUART5_RXD
                           // LPUART5_TXD
                           IMX_PAD_DAP_TDO_TRACESWO, ALT6, IMX_PIN_LPUART5_IPP_IND_LPUART_TXD_SELECT_INPUT_REG, ALT0,     // LPUART5_TXD
                           IMX_PAD_GPIO_IO00,        ALT5, IMX_PIN_LPUART5_IPP_IND_LPUART_TXD_SELECT_INPUT_REG, ALT1,     // LPUART5_TXD
                           // MIC_PDM_BITSTREAM_0
                           IMX_PAD_GPIO_IO05,        ALT2, IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_0_REG, ALT0,// MIC_PDM_BITSTREAM_0
                           IMX_PAD_GPIO_IO20,        ALT2, IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_0_REG, ALT1,// MIC_PDM_BITSTREAM_0
                           IMX_PAD_PDM_BIT_STREAM0,  ALT0, IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_0_REG, ALT2,// MIC_PDM_BITSTREAM_0
                           // MIC_PDM_BITSTREAM_1
                           IMX_PAD_GPIO_IO06,        ALT2, IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_1_REG, ALT0,// MIC_PDM_BITSTREAM_1
                           IMX_PAD_GPIO_IO26,        ALT2, IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_1_REG, ALT1,// MIC_PDM_BITSTREAM_1
                           IMX_PAD_PDM_BIT_STREAM1,  ALT0, IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_1_REG, ALT2,// MIC_PDM_BITSTREAM_1
                           // MIC_PDM_BITSTREAM_2
                           IMX_PAD_GPIO_IO12,        ALT2, IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_2_REG, ALT0,// MIC_PDM_BITSTREAM_2
                           IMX_PAD_GPIO_IO16,        ALT2, IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_2_REG, ALT1,// MIC_PDM_BITSTREAM_2
                           // MIC_PDM_BITSTREAM_3
                           IMX_PAD_GPIO_IO13,        ALT2, IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_3_REG, ALT0,// MIC_PDM_BITSTREAM_3
                           IMX_PAD_GPIO_IO19,        ALT2, IMX_PIN_PDM_IPP_IND_MIC_PDM_BITSTREAM_SELECT_INPUT_3_REG, ALT1,// MIC_PDM_BITSTREAM_3
                           // SAI1_MCLK
                           IMX_PAD_UART2_RXD,        ALT4, IMX_PIN_SAI1_IPP_IND_SAI_MCLK_SELECT_INPUT_REG, ALT0,          // SAI1_MCLK
                           IMX_PAD_SAI1_RXD0,        ALT1, IMX_PIN_SAI1_IPP_IND_SAI_MCLK_SELECT_INPUT_REG, ALT1,          // SAI1_MCLK
                           // SAI3_RXBCLK
                           IMX_PAD_GPIO_IO18,        ALT1, IMX_PIN_SAI3_IPP_IND_SAI_RXBCLK_SELECT_INPUT_REG, ALT0,        // SAI3_RXBCLK
                           IMX_PAD_GPIO_IO21,        ALT7, IMX_PIN_SAI3_IPP_IND_SAI_RXBCLK_SELECT_INPUT_REG, ALT1,        // SAI3_RXBCLK
                           // SAI3_RXSYNC
                           IMX_PAD_GPIO_IO12,        ALT7, IMX_PIN_SAI3_IPP_IND_SAI_RXSYNC_SELECT_INPUT_REG, ALT0,        // SAI3_RXSYNC
                           IMX_PAD_GPIO_IO19,        ALT1, IMX_PIN_SAI3_IPP_IND_SAI_RXSYNC_SELECT_INPUT_REG, ALT1,        // SAI3_RXSYNC
                           // SPDIF_I
                           IMX_PAD_GPIO_IO22,        ALT2, IMX_PIN_SAI3_IPP_IND_SAI_RXSYNC_SELECT_INPUT_REG, ALT0,        // SPDIF_I
                           IMX_PAD_ENET2_RD1,        ALT1, IMX_PIN_SAI3_IPP_IND_SAI_RXSYNC_SELECT_INPUT_REG, ALT1,        // SPDIF_I
                           IMX_PAD_ENET2_RD3,        ALT2, IMX_PIN_SAI3_IPP_IND_SAI_RXSYNC_SELECT_INPUT_REG, ALT2,        // SPDIF_I
                           // USDHC3_CARD_CLK_IN
                           IMX_PAD_GPIO_IO22,        ALT1, IMX_PIN_USDHC3_IPP_CARD_CLK_IN_SELECT_INPUT_REG, ALT0,         // USDHC3_CARD_CLK_IN
                           IMX_PAD_SD3_CLK,          ALT0, IMX_PIN_USDHC3_IPP_CARD_CLK_IN_SELECT_INPUT_REG, ALT1,         // USDHC3_CARD_CLK_IN
                           // USDHC3_CMD_IN
                           IMX_PAD_GPIO_IO23,        ALT1, IMX_PIN_USDHC3_IPP_CMD_IN_SELECT_INPUT_REG, ALT0,              // USDHC3_CMD_IN
                           IMX_PAD_SD3_CMD,          ALT0, IMX_PIN_USDHC3_IPP_CMD_IN_SELECT_INPUT_REG, ALT1,              // USDHC3_CMD_IN
                           // USDHC3_DAT0_IN
                           IMX_PAD_GPIO_IO24,        ALT1, IMX_PIN_USDHC3_IPP_DAT0_IN_SELECT_INPUT_REG, ALT0,             // USDHC3_DAT0_IN
                           IMX_PAD_SD3_DATA0,        ALT0, IMX_PIN_USDHC3_IPP_DAT0_IN_SELECT_INPUT_REG, ALT1,             // USDHC3_DAT0_IN
                           // USDHC3_DAT1_IN
                           IMX_PAD_GPIO_IO25,        ALT1, IMX_PIN_USDHC3_IPP_DAT1_IN_SELECT_INPUT_REG, ALT0,             // USDHC3_DAT1_IN
                           IMX_PAD_SD3_DATA1,        ALT0, IMX_PIN_USDHC3_IPP_DAT1_IN_SELECT_INPUT_REG, ALT1,             // USDHC3_DAT1_IN
                           // USDHC3_DAT2_IN
                           IMX_PAD_GPIO_IO26,        ALT1, IMX_PIN_USDHC3_IPP_DAT2_IN_SELECT_INPUT_REG, ALT0,             // USDHC3_DAT2_IN
                           IMX_PAD_SD3_DATA2,        ALT0, IMX_PIN_USDHC3_IPP_DAT2_IN_SELECT_INPUT_REG, ALT1,             // USDHC3_DAT2_IN
                           // USDHC3_DAT3_IN
                           IMX_PAD_GPIO_IO27,        ALT1, IMX_PIN_USDHC3_IPP_DAT3_IN_SELECT_INPUT_REG, ALT0,             // USDHC3_DAT3_IN
                           IMX_PAD_SD3_DATA3,        ALT0, IMX_PIN_USDHC3_IPP_DAT3_IN_SELECT_INPUT_REG, ALT1,             // USDHC3_DAT3_IN
                }
            }
        }
    })
}
