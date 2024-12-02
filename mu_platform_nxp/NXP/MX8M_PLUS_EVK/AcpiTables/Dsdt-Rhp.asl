/*
 * Copyright 2022-2023 NXP
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

/** @file
* Description: NXP iMX8MP EVK Resource Hub Proxy
*/

Device(RHPX)
{
  Name(_HID, "MSFT8000")
  Name(_CID, "MSFT8000")
  Name(_UID, 1)

  Name(_CRS, ResourceTemplate()
  {

    // Index 0
    I2CSerialBus(0xFFFF,, 0,, "\\_SB.I2C2",,,,)

    // Index 1
    I2CSerialBus(0xFFFF,, 0,, "\\_SB.I2C3",,,,)

    // Index 2
    SPISerialBus (          // SCLK - GPIO5_IO10 (138) - EXP CN (J21) pin 23
                            // MOSI - GPIO5_IO11 (139) - EXP CN (J21) pin 19
                            // MISO - GPIO5_IO12 (140) - EXP CN (J21) pin 21
                            // SS0  - GPIO5_IO13 (141) - EXP CN (J21) pin 24
      0,                    // Device selection (CE0)
      PolarityLow,          // Device selection polarity
      FourWireMode,         // wiremode
      0,                    // databit len - placeholder
      ControllerInitiated,  // slave mode
      0,                    // connection speed - placeholder
      ClockPolarityLow,     // clock polarity
      ClockPhaseFirst,      // clock phase
      "\\_SB.SPI2",         // ResourceSource: SPI bus controller name
      0,                    // ResourceSourceIndex
                            // Resource usage
                            // DescriptorName: creates name for offset of resource descriptor
    )                       // Vendor Data

#if !FixedPcdGet32(PcdCAN1InsteadOfI2C5)
    // Index 3
    I2CSerialBus(0xFFFF,, 0,, "\\_SB.I2C5",,,,)
#endif
  // GPIO3_IO19 - PAD_SAI5_RXFS EXP CN pin 32 (GPIO.26)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 83 } // 2 * 32 + 19
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 83 }

  // GPIO3_IO20 - PAD_SAI5_RXC EXP CN pin 40 (GPIO.29)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 84 } // 2 * 32 + 20
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 84 }

  //GPIO3_IO21 - PAD_SAI5_RXD0 EXP CN pin 38 (GPIO.28)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 85 } // 2 * 32 + 21
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 85 }

  // GPIO3_IO22 - PAD_SAI5_RXD1 EXP CN pin 37 (GPIO.25)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 86 } // 2 * 32 + 22
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 86 }
  
  // GPIO3_IO23 - PAD_SAI5_RXD2 EXP CN pin 36 (GPIO.27)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 87 } // 2 * 32 + 23
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 87 }

#if !FixedPcdGet32(PcdCAN2InsteadOfPDMStream3)
  // GPIO3_IO24 - PAD_SAI5_RXD3 EXP CN pin 35 (GPIO.24)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 88 } // 2 * 32 + 24
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 88 }
#endif

  // GPIO5_IO06 PAD_ECSPI1_SCLK - EXP CN pin 10  (RXD)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 134 } // 4 * 32 + 6
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 134 }

  // GPIO5_IO07 PAD_ECSPI1_MOSI - EXP CN pin 8  (TXD)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 135 } // 4 * 32 + 7
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 135 }

  // GPIO5_IO08 PAD_ECSPI1_MISO - EXP CN pin 7  (GPIO0.7)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 136 } // 4 * 32 + 8
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 136 }

  // GPIO5_IO9 PAD_ECSPI1_SS0 - EXP CN pin 11 (GPIO0.0)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 137 } // 4 * 32 + 9
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 137 }

  // GPIO5_IO10 - PAD_ECSPI2_SCLK EXP CN pin 23 (SCLK)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 138 } // 4 * 32 + 10
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 138 }

  // GPIO5_IO11 - PAD_ECSPI2_MOSI EXP CN pin 19  (MOSI)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 139 } // 4 * 32 + 11
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 139 }

  // GPIO5_IO12 - PAD_ECSPI2_MISO EXP CN pin 21  (MISO)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 140 } // 4 * 32 + 12
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 140 }

  // GPIO5_IO13 - PAD_ECSPI2_SS0 EXP CN pin 24 (CE0)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 141 } // 4 * 32 + 13
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 141 }

  // GPIO5_IO18 PAD_I2C3_SCL - EXP CN pin 5  (SCL.1)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 146 } // 4 * 32 + 18
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 146 }

  // GPIO5_IO19 PAD_I2C3_SDA - EXP CN pin 3  (SDA.1)
    GpioIO(Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 147 } // 4 * 32 + 19
    GpioInt(Edge, ActiveBoth, Shared, PullDown, 0, "\\_SB.GPIO",) { 147 }

  })

  Name(_DSD, Package()
  {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package()
    {
      // I2C busses 2-3
      Package(2) { "bus-I2C-I2C2", Package() { 0 }},
      Package(2) { "bus-I2C-I2C3", Package() { 1 }},
      // SPI bus 2
      // Reference clock is 24 MHz
      Package(2) { "bus-SPI-SPI2", Package() { 2 }},
      Package(2) { "SPI2-MinClockInHz", 46 },                              // 46 Hz
      Package(2) { "SPI2-MaxClockInHz", 12000000 },                        // 12 MHz
      Package(2) { "SPI2-SupportedDataBitLengths", Package() { 8,16,32 }}, // Data bit length
#if !FixedPcdGet32(PcdCAN1InsteadOfI2C5)
      Package(2) { "bus-I2C-I2C5", Package() { 3 }},
#endif

      // GPIO Pin Count and supported drive modes
      Package (2) { "GPIO-PinCount", 157 },
      Package (2) { "GPIO-UseDescriptorPinNumbers", 1 },

      // InputHighImpedance, InputPullUp, InputPullDown, OutputCmos
      Package (2) { "GPIO-SupportedDriveModes", 0x0F },
    }
  })
}
