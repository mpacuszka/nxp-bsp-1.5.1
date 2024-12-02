/*
* Copyright 2022 NXP
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the disclaimer
* below) provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* * Neither the name of NXP nor the names of its contributors may be used to
* endorse or promote products derived from this software without specific prior
* written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#define SC_PAD_TYPE_1V8_3V3  0x00
#define SC_PAD_TYPE_3V3      0x01
#define SC_PAD_TYPE_1V8      0x02
#define SC_PAD_TYPE_HSIC     0x03

#define DefinePad(_PadId, _GpioAlt, PadType)  _PadId, ((_GpioAlt << 4) + _PadId)

Device (GPIO)
{
    Name (_HID, "NXP0102")
    Name (_UID, 0x0)
    Method (_STA)
    {
        Return(0xf)
    }
    Method (_CRS, 0x0, NotSerialized) {
        Name (RBUF, ResourceTemplate () {
                MEMORY32FIXED(ReadWrite, 0x5D080000, 0x10000, )  // GPIO_0
                MEMORY32FIXED(ReadWrite, 0x5D090000, 0x10000, )  // GPIO_1
                MEMORY32FIXED(ReadWrite, 0x5D0A0000, 0x10000, )  // GPIO_2
                MEMORY32FIXED(ReadWrite, 0x5D0B0000, 0x10000, )  // GPIO_3
                MEMORY32FIXED(ReadWrite, 0x5D0C0000, 0x10000, )  // GPIO_4
                MEMORY32FIXED(ReadWrite, 0x5D0D0000, 0x10000, )  // GPIO_5
                Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 168 }   // GPIO_0
                Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 169 }   // GPIO_1
                Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 170 }   // GPIO_2
                Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 171 }   // GPIO_3
                Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 172 }   // GPIO_4
                Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 173 }   // GPIO_6
                I2CSerialBus(0x60, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
        })
        Return(RBUF)
    }
    Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () { //
            Package (2) {"Pin_Config",
                //                    PAD ID                  GPIO_ALT     Pad Type               GPIO name
                Buffer () { SC_P_ESAI0_FSR,          0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO00  NO GPIO funct
                            SC_P_ESAI0_FST,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO01
                            SC_P_ESAI0_SCKR,            4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO02
                            SC_P_ESAI0_SCKT,            4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO03
                            SC_P_ESAI0_TX0,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO04
                            SC_P_ESAI0_TX1,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO05
                            SC_P_ESAI0_TX2_RX3,         4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO06
                            SC_P_ESAI0_TX3_RX2,         4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO07
                            SC_P_ESAI0_TX4_RX1,         4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO08
                            SC_P_ESAI0_TX5_RX0,         4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO09
                            SC_P_SPDIF0_RX,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO10
                            SC_P_SPDIF0_TX,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO11
                            SC_P_SPDIF0_EXT_CLK,        4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO12
                            SC_P_SPI3_SCK,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO13
                            SC_P_SPI3_SDO,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO14
                            SC_P_SPI3_SDI,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO15
                            SC_P_SPI3_CS0,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO16
                            SC_P_SPI3_CS1,           0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO17  NO GPIO funct
                            SC_P_MCLK_IN1,           0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO18  NO GPIO funct
                            SC_P_MCLK_IN0,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO19
                            SC_P_MCLK_OUT0,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO20
                            SC_P_UART1_TX,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO21
                            SC_P_UART1_RX,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO22
                            SC_P_UART1_RTS_B,        0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO23  NO GPIO funct
                            SC_P_UART1_CTS_B,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO24
                            SC_P_SAI0_TXD,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO25
                            SC_P_SAI0_TXC,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO26
                            SC_P_SAI0_RXD,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO27
                            SC_P_SAI0_TXFS,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO28
                            SC_P_SAI1_RXD,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO29
                            SC_P_SAI1_RXC,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO30
                            SC_P_SAI1_RXFS,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO0_IO31

                            SC_P_SPI2_CS0,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO00
                            SC_P_SPI2_SDO,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO01
                            SC_P_SPI2_SDI,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO02
                            SC_P_SPI2_SCK,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO03
                            SC_P_SPI0_SCK,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO04
                            SC_P_SPI0_SDI,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO05
                            SC_P_SPI0_SDO,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO06
                            SC_P_SPI0_CS1,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO07
                            SC_P_SPI0_CS0,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO08
                            SC_P_ADC_IN1,               4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO09
                            SC_P_ADC_IN0,               4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO10
                            SC_P_ADC_IN3,               4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO11
                            SC_P_ADC_IN2,               4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO12
                            SC_P_ADC_IN5,               4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO13
                            SC_P_ADC_IN4,               4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO14
                            SC_P_FLEXCAN0_RX,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO15
                            SC_P_FLEXCAN0_TX,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO16
                            SC_P_FLEXCAN1_RX,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO17
                            SC_P_FLEXCAN1_TX,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO18
                            SC_P_FLEXCAN2_RX,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO19
                            SC_P_FLEXCAN2_TX,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO20
                            SC_P_UART0_RX,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO21
                            SC_P_UART0_TX,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO22
                            SC_P_UART2_TX,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO23
                            SC_P_UART2_RX,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO24
                            SC_P_MIPI_DSI0_I2C0_SCL,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO25
                            SC_P_MIPI_DSI0_I2C0_SDA,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO26
                            SC_P_MIPI_DSI0_GPIO0_00,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO27
                            SC_P_MIPI_DSI0_GPIO0_01,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO28
                            SC_P_MIPI_DSI1_I2C0_SCL,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO29
                            SC_P_MIPI_DSI1_I2C0_SDA,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO30
                            SC_P_MIPI_DSI1_GPIO0_00,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO1_IO31

                            SC_P_MIPI_DSI1_GPIO0_01,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO00
                            SC_P_PMIC_I2C_SCL,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO01
                            SC_P_PMIC_I2C_SDA,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO02
                            SC_P_SCU_GPIO0_00,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO03
                            SC_P_PMIC_INT_B,         0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO04  NO GPIO funct
                            SC_P_SCU_GPIO0_01,       0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO05  NO GPIO funct
                            SC_P_SCU_PMIC_STANDBY,   0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO06  NO GPIO funct
                            SC_P_SCU_BOOT_MODE0,     0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO07  NO GPIO funct
                            SC_P_SCU_BOOT_MODE1,     0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO08  NO GPIO funct
                            SC_P_SCU_BOOT_MODE2,     0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO09  NO GPIO funct
                            SC_P_SCU_BOOT_MODE3,     0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO10  NO GPIO funct
                            SC_P_CSI_D00,            0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO11  NO GPIO funct
                            SC_P_CSI_D01,            0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO12  NO GPIO funct
                            SC_P_CSI_D02,            0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO13  NO GPIO funct
                            SC_P_CSI_D03,            0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO14  NO GPIO funct
                            SC_P_CSI_D04,            0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO15  NO GPIO funct
                            SC_P_CSI_D05,            0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO16  NO GPIO funct
                            SC_P_CSI_D06,            0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO17  NO GPIO funct
                            SC_P_CSI_D07,            0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO18  NO GPIO funct
                            SC_P_CSI_HSYNC,          0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO19  NO GPIO funct
                            SC_P_CSI_VSYNC,          0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO20  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO21  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO22  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO23  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO24  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO25  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO26  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO27  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO28  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO29  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO30  NO GPIO funct
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO2_IO31  NO GPIO funct

                            SC_P_CSI_PCLK,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO00
                            SC_P_CSI_MCLK,              4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO01
                            SC_P_CSI_EN,                4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO02
                            SC_P_CSI_RESET,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO03
                            SC_P_MIPI_CSI0_MCLK_OUT,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO04
                            SC_P_MIPI_CSI0_I2C0_SCL,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO05
                            SC_P_MIPI_CSI0_I2C0_SDA,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO06
                            SC_P_MIPI_CSI0_GPIO0_01,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO07
                            SC_P_MIPI_CSI0_GPIO0_00,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO08
                            SC_P_QSPI0A_DATA0,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO09
                            SC_P_QSPI0A_DATA1,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO10
                            SC_P_QSPI0A_DATA2,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO11
                            SC_P_QSPI0A_DATA3,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO12
                            SC_P_QSPI0A_DQS,            4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO13
                            SC_P_QSPI0A_SS0_B,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO14
                            SC_P_QSPI0A_SS1_B,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO15
                            SC_P_QSPI0A_SCLK,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO16
                            SC_P_QSPI0B_SCLK,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO17
                            SC_P_QSPI0B_DATA0,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO18
                            SC_P_QSPI0B_DATA1,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO19
                            SC_P_QSPI0B_DATA2,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO20
                            SC_P_QSPI0B_DATA3,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO21
                            SC_P_QSPI0B_DQS,            4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO22
                            SC_P_QSPI0B_SS0_B,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO23
                            SC_P_QSPI0B_SS1_B,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO24
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO25
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO26
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO27
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO28
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO29
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO30
                            SC_P_UNDEF,              0xFF,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO3_IO31

                            SC_P_PCIE_CTRL0_PERST_B,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO00
                            SC_P_PCIE_CTRL0_CLKREQ_B,   4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO01
                            SC_P_PCIE_CTRL0_WAKE_B,     4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO02
                            SC_P_USB_SS3_TC0,           4,  SC_PAD_TYPE_3V3    , // IMX_PIN_RES_GPIO4_IO03
                            SC_P_USB_SS3_TC1,           4,  SC_PAD_TYPE_3V3    , // IMX_PIN_RES_GPIO4_IO04
                            SC_P_USB_SS3_TC2,           4,  SC_PAD_TYPE_3V3    , // IMX_PIN_RES_GPIO4_IO05
                            SC_P_USB_SS3_TC3,           4,  SC_PAD_TYPE_3V3    , // IMX_PIN_RES_GPIO4_IO06
                            SC_P_EMMC0_CLK,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO07
                            SC_P_EMMC0_CMD,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO08
                            SC_P_EMMC0_DATA0,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO09
                            SC_P_EMMC0_DATA1,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO10
                            SC_P_EMMC0_DATA2,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO11
                            SC_P_EMMC0_DATA3,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO12
                            SC_P_EMMC0_DATA4,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO13
                            SC_P_EMMC0_DATA5,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO14
                            SC_P_EMMC0_DATA6,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO15
                            SC_P_EMMC0_DATA7,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO16
                            SC_P_EMMC0_STROBE,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO17
                            SC_P_EMMC0_RESET_B,         4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO18
                            SC_P_USDHC1_RESET_B,        4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO19
                            SC_P_USDHC1_VSELECT,        4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO20
                            SC_P_USDHC1_WP,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO21
                            SC_P_USDHC1_CD_B,           4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO22
                            SC_P_USDHC1_CLK,            4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO23
                            SC_P_USDHC1_CMD,            4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO24
                            SC_P_USDHC1_DATA0,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO25
                            SC_P_USDHC1_DATA1,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO26
                            SC_P_USDHC1_DATA2,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO27
                            SC_P_USDHC1_DATA3,          4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO28
                            SC_P_ENET0_RGMII_TXC,       4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO29
                            SC_P_ENET0_RGMII_TX_CTL,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO30
                            SC_P_ENET0_RGMII_TXD0,      4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO4_IO31

                            SC_P_ENET0_RGMII_TXD1,      4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO00
                            SC_P_ENET0_RGMII_TXD2,      4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO01
                            SC_P_ENET0_RGMII_TXD3,      4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO02
                            SC_P_ENET0_RGMII_RXC,       4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO03
                            SC_P_ENET0_RGMII_RX_CTL,    4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO04
                            SC_P_ENET0_RGMII_RXD0,      4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO05
                            SC_P_ENET0_RGMII_RXD1,      4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO06
                            SC_P_ENET0_RGMII_RXD2,      4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO07
                            SC_P_ENET0_RGMII_RXD3,      4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO08
                            SC_P_ENET0_REFCLK_125M_25M, 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO09
                            SC_P_ENET0_MDIO,            4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO10
                            SC_P_ENET0_MDC,             4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO11
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO12
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO13
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO14
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO15
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO16
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO17
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO18
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO19
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO20
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO21
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO22
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO23
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO24
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO25
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO26
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO27
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO28
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO29
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3, // IMX_PIN_RES_GPIO5_IO30
                            SC_P_UNDEF,                 4,  SC_PAD_TYPE_1V8_3V3  // IMX_PIN_RES_GPIO5_IO31
                }
            }
        }
    })
}