/**
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
    I2CSerialBus(0xFFFF,, 0,, "\\_SB.I2C5",,,,)
    // Index 2
    I2CSerialBus(0xFFFF,, 0,, "\\_SB.I2C8",,,,)
    // Index 3
    I2CSerialBus(0xFFFF,, 0,, "\\_SB.I2CF",,,,)

    // Index 4
    SPISerialBus (          // SCLK (SPI0_SCK) - GPIO1_IO04 (36) - J43E E25, BaseBoard J20 A29
                            // MOSI (SPI0_SDO) - GPIO1_IO06 (38) - J43E E26, BaseBoard J20 A33
                            // MISO (SPI0_SDI) - GPIO1_IO05 (37) - J43E E28, BaseBoard J20 A32
                            // SS0 (SPI0_CS0)  - GPIO1_IO08 (40) - J43E E29, BaseBoard J20 A30
      0,                    // Device selection (CE0)
      PolarityLow,          // Device selection polarity
      FourWireMode,         // wiremode
      0,                    // databit len - placeholder
      ControllerInitiated,  // slave mode
      0,                    // connection speed - placeholder
      ClockPolarityLow,     // clock polarity
      ClockPhaseFirst,      // clock phase
      "\\_SB.SPI0",         // ResourceSource: SPI bus controller name
      0,                    // ResourceSourceIndex
                            // Resource usage
                            // DescriptorName: creates name for offset of resource descriptor
    )                       // Vendor Data

    // Index 5
    SPISerialBus (          // SCLK (SPI3_SCK) - GPIO0_IO13 (13) - J13A A11, J13C C16, BaseBoard J8 SH9 , BaseBoard J20 A21
                            // MOSI (SPI3_SDO) - GPIO0_IO14 (14) - J13A A13, J13C C17, BaseBoard J8 SH7 , BaseBoard J20 A25
                            // MISO (SPI3_SDI) - GPIO0_IO15 (15) - J13A A14,    -    , BaseBoard J8 SH31, BaseBoard J20 A26
                            // SS0 (SPI3_CS0)  - GPIO0_IO16 (16) - J13A A16, J13C C20, BaseBoard J8 SH13, BaseBoard J20 A22
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

    // Index 6
    I2CSerialBus(0xFFFF,, 0,, "\\_SB.I2C9",,,,)

  })

  Name(_DSD, Package()
  {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package()
    {
      // I2C buses
      Package(2) { "bus-I2C-I2C1", Package() { 0 }},
      Package(2) { "bus-I2C-I2C5", Package() { 1 }},
      Package(2) { "bus-I2C-I2C8", Package() { 2 }},
      Package(2) { "bus-I2C-I2CF", Package() { 3 }},

      // SPI bus 0
      // Reference clock is 160 MHz
      Package(2) { "bus-SPI-SPI0", Package() { 4 }},
      Package(2) { "SPI0-MinClockInHz", 10000 },                           // 10 kHz
      Package(2) { "SPI0-MaxClockInHz", 16000000 },                        // 16 MHz
      Package(2) { "SPI0-SupportedDataBitLengths", Package() { 8,16,32 }}, // Data bit length

      // SPI bus 3
      // Reference clock is 160 MHz
      Package(2) { "bus-SPI-SPI3", Package() { 5 }},
      Package(2) { "SPI3-MinClockInHz", 10000 },                           // 10 kHz
      Package(2) { "SPI3-MaxClockInHz", 16000000 },                        // 16 MHz
      Package(2) { "SPI3-SupportedDataBitLengths", Package() { 8,16,32 }}, // Data bit length

      Package(2) { "bus-I2C-I2C9", Package() { 6 }},

      // GPIO Pin Count and supported drive modes
      Package (2) { "GPIO-PinCount", 157 },
      Package (2) { "GPIO-UseDescriptorPinNumbers", 1 },

      // InputHighImpedance, InputPullUp, InputPullDown, OutputCmos
      Package (2) { "GPIO-SupportedDriveModes", 0x0F },
    }
  })
}
