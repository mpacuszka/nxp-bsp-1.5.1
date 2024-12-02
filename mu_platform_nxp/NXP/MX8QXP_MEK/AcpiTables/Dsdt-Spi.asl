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

/* iMX8QXP Low Power Serial Peripheral Interface (LPSPI) */

Device (SPI0)
{
  Name (_HID, "NXP0121")
  Name (_HRV, 0x1)  // REV_0001
  Name (_UID, 0x0)
  Method (_STA)
  {
    Return(0xf)
  }
  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED(ReadWrite, 0x5a000000, 0x10000, )
    Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 368 }

    // SS0 (SPI0_CS0) - GPIO1_IO08 (40) - J43E E29, BaseBoard J20 A30
    GpioIO (Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PIN_SPI0_CS0 }

    // MISO (SPI0_SDI) - GPIO1_IO05 (37) - J43E E28, BaseBoard J20 A32
    // MOSI (SPI0_SDO) - GPIO1_IO06 (38) - J43E E26, BaseBoard J20 A33
    // SCLK (SPI0_SCK) - GPIO1_IO04 (36) - J43E E25, BaseBoard J20 A29
    // MsftFunctionConfig (Exclusive, PullDown, IMX_ALT0, "\\_SB.GPIO", 0,
    //                     ResourceConsumer, ) { IMX_PIN_SPI0_SDI, IMX_PIN_SPI0_SDO, IMX_PIN_SPI0_SCK }
    //
    // MsftFunctionConfig (Arg0, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6) { Pin List }
    // The MISO, MOSI, SCK pads are configured in iMX8BoardInit.c
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 160000000U},
        Package (2) {"MaxConnectionSpeedHz", 0U},  // 0 means no limitation applied
        Package (2) {"SampleOnDelayedSckEdge", 0}, // 0 - sample on SCK edge, 1 - sample on delayed SCK edge
      }
  })
}

Device (SPI1)
{
  Name (_HID, "NXP0121")
  Name (_HRV, 0x1)  // REV_0001
  Name (_UID, 0x1)
  Method (_STA)
  {
    Return(0x0)
  }
  Name (_CRS, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x5a010000, 0x10000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 369 }
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 160000000U},
        Package (2) {"MaxConnectionSpeedHz", 0U},  // 0 means no limitation applied
        Package (2) {"SampleOnDelayedSckEdge", 0}, // 0 - sample on SCK edge, 1 - sample on delayed SCK edge
      }
  })
}

Device (SPI2)
{
  Name (_HID, "NXP0121")
  Name (_HRV, 0x1)  // REV_0001
  Name (_UID, 0x2)
  Method (_STA)
  {
    Return(0x0)
  }
  Name (_CRS, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x5a020000, 0x10000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 370 }
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 160000000U},
        Package (2) {"MaxConnectionSpeedHz", 0U},  // 0 means no limitation applied
        Package (2) {"SampleOnDelayedSckEdge", 0}, // 0 - sample on SCK edge, 1 - sample on delayed SCK edge
      }
  })
}

Device (SPI3)
{
  Name (_HID, "NXP0121")
  Name (_HRV, 0x1)  // REV_0001
  Name (_UID, 0x3)
  Method (_STA)
  {
    Return(0xf)
  }
  Name (_CRS, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x5a030000, 0x10000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 371 }

    // SS0 (SPI3_CS0)  - GPIO0_IO16 (16) - J13A A16, J13C C20, BaseBoard J8 SH13, BaseBoard J20 A22
    GpioIO (Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PIN_SPI3_CS0 }

    // MISO (SPI3_SDI) - GPIO0_IO15 (15) - J13A A14,    -    , BaseBoard J8 SH31, BaseBoard J20 A26
    // MOSI (SPI3_SDO) - GPIO0_IO14 (14) - J13A A13, J13C C17, BaseBoard J8 SH7 , BaseBoard J20 A25
    // SCLK (SPI3_SCK) - GPIO0_IO13 (13) - J13A A11, J13C C16, BaseBoard J8 SH9 , BaseBoard J20 A21
    // MsftFunctionConfig (Exclusive, PullDown, IMX_ALT0, "\\_SB.GPIO", 0,
    //                     ResourceConsumer, ) { IMX_PIN_SPI3_SDI, IMX_PIN_SPI3_SDO, IMX_PIN_SPI3_SCK }
    //
    // MsftFunctionConfig (Arg0, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6) { Pin List }
    // The MISO, MOSI, SCK pads are configured in iMX8BoardInit.c
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 160000000U},
        Package (2) {"MaxConnectionSpeedHz", 0U},  // 0 means no limitation applied
        Package (2) {"SampleOnDelayedSckEdge", 0}, // 0 - sample on SCK edge, 1 - sample on delayed SCK edge
      }
  })
}

