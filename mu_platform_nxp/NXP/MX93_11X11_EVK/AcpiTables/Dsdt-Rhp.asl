/**
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
 
/*
 * Description: NXP Resource Hub Proxy
 */

Device(RHPX)
{
  Name(_HID, "MSFT8000")
  Name(_CID, "MSFT8000")
  Name(_UID, 1)

  Name(_CRS, ResourceTemplate()
  {
    // Index 0
    I2CSerialBus(0xFFFF,, 0,, "\\_SB.I2C1",,,,)
    // Index 1
    I2CSerialBus(0xFFFF,, 0,, "\\_SB.I2C2",,,,)

    // Index 2
    SPISerialBus (
      0,                    // Device selection (CE0)
      PolarityLow,          // Device selection polarity
      FourWireMode,         // wiremode
      0,                    // databit len - placeholder
      ControllerInitiated,  // slave mode
      0,                    // connection speed - placeholder
      ClockPolarityLow,     // clock polarity
      ClockPhaseFirst,      // clock phase
      "\\_SB.SPI3",         // ResourceSource: SPI bus controller name
      0,                    // ResourceSourceIndex
                            // Resource usage
                            // DescriptorName: creates name for offset of resource descriptor
    )                       // Vendor Data

    // PAD: GPIO_IO00 - I2C3_SDA, no device connected, 2.2K PullUp
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO00 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO00 }
    // PAD: GPIO_IO01 - I2C3_SCL, no device connected, 2.2K PullUp
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO01 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO01 }
    // PAD: GPIO_IO00 - I2C4_SDA, no device connected, 2.2K PullUp
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO02 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO02 }
    // PAD: GPIO_IO01 - I2C4_SCL, no device connected, 2.2K PullUp
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO03 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO03 }


    // PAD: GPIO_IO04 - LED_G (Green)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO04 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO04 }
    // PAD: GPIO_IO05 - EXP GPIO
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO05 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO05 }
    // PAD: GPIO_IO06 - EXP GPIO
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO06 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO06 }
    // PAD: GPIO_IO07 - EXP GPIO
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO07 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO07 }


    // PAD: GPIO_IO08 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO08 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO08 }
    // PAD: GPIO_IO09 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO09 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO09 }
    // PAD: GPIO_IO10 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO10 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO10 }
    // PAD: GPIO_IO11 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO11 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO11 }


    // PAD: GPIO_IO12 - LED_B (Blue)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO12 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO12 }
    // PAD: GPIO_IO13 - LED_R (Red)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO13 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO13 }
    // PAD: GPIO_IO14 - EXP GPIO
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO14 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO14 }
    // PAD: GPIO_IO15 - EXP GPIO
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO15 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO15 }

    // PAD: GPIO2_IO16 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO16 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO16 }
    // PAD: GPIO2_IO17 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO17 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO17 }
    // PAD: GPIO2_IO18
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO18 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO18 }
    // PAD: GPIO2_IO19 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO19 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO19 }


    // PAD: GPIO2_IO20 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO20 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO20 }
    // PAD: GPIO2_IO21 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO21 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO21 }
    // PAD: GPIO2_IO22
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO22 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO22 }
    // PAD: GPIO2_IO23 - RFU_BTN1 (Button 1)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO23 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO23 }


    // PAD: GPIO2_IO24 - RFU_BTN1 (Button 2)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO24 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO24 }
    // PAD: GPIO2_IO25 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO25 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO25 }
    // PAD: GPIO2_IO26 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO26 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO26 }
    // PAD: GPIO2_IO27 - EXP GPIO, MUXED (TMUX1574RSVR)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_GPIO_IO27 }
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { IMX_PAD_GPIO_IO27 }

  })

  Name(_DSD, Package()
  {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package()
    {
      // I2C buses 1-2
      Package(2) { "bus-I2C-I2C1", Package() { 0 }},
      Package(2) { "bus-I2C-I2C2", Package() { 1 }},

      // SPI bus 3
      // Reference clock is 24 MHz
      Package(2) { "bus-SPI-SPI3", Package() { 2 }},
      Package(2) { "SPI3-MinClockInHz", 50000 },                           // 50 kHz
      Package(2) { "SPI3-MaxClockInHz", 1000000 },                         // 1 MHz
      Package(2) { "SPI3-SupportedDataBitLengths", Package() { 8,16,32 }}, // Data bit length

      // GPIO Pin Count and supported drive modes
      Package (2) { "GPIO-PinCount", 128 },
      Package (2) { "GPIO-UseDescriptorPinNumbers", 1 },

      // InputHighImpedance, InputPullUp, InputPullDown, OutputCmos
      Package (2) { "GPIO-SupportedDriveModes", 0x0F },
    }
  })
}
