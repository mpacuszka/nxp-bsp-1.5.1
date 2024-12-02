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

/* iMX93 Low Power Serial Peripheral Interface (LPSPI) */

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
    MEMORY32FIXED(ReadWrite, 0x44360000, 0x10000, )
    Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 48 }
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 24000000U},
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
    MEMORY32FIXED(ReadWrite, 0x44370000, 0x10000, )
    Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 49 }
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 24000000U},
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
      MEMORY32FIXED(ReadWrite, 0x42550000, 0x10000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 97 }

    // SS0 (SPI3_CS0)  - GPIO_IO08 (16) - EXP_GPIO_IO08 or M2/NGFF-64 depending on EXP_SEL (iMX8BoardInit.c:IOExpanderAdp5585Init)
    GpioIO (Shared, PullDown, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , RawDataBuffer(){0x01, 0xFE, 0x05, 0x00, 0x00}) { IMX_PAD_GPIO_IO08 }

    // MISO (SPI3_SDI) - GPIO_IO09 (15) - EXP_GPIO_IO09 or M2/NGFF-40 depending on EXP_SEL (iMX8BoardInit.c:IOExpanderAdp5585Init)
    // MOSI (SPI3_SDO) - GPIO_IO10 (14) - EXP_GPIO_IO10 or M2/NGFF-38 depending on EXP_SEL (iMX8BoardInit.c:IOExpanderAdp5585Init)
    // SCLK (SPI3_SCK) - GPIO_IO11 (13) - EXP_GPIO_IO11 or M2/NGFF-42 depending on EXP_SEL (iMX8BoardInit.c:IOExpanderAdp5585Init)
    // Note: The EXP_SEL can be configured using PcdMX93EXPSelSetting in .dsc file
    // MsftFunctionConfig (Exclusive, PullDown, IMX_ALT1, "\\_SB.GPIO", 0,
    //                     ResourceConsumer, ) { IMX_PAD_GPIO_IO09, IMX_PAD_GPIO_IO10, IMX_PAD_GPIO_IO11 }
    //
    // MsftFunctionConfig (Arg0, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6) { Pin List }
    VendorLong () {
      MSFT_UUID,            // Vendor UUID (MSFT UUID)
      MSFT_FUNCTION_CONFIG, // Resource Identifier (MSFT Function Config)
      0x1f,0x00,            // Length (0xF + sizeof(PinList) + sizeof(ResourceName))
      0x01,                 // Revision (0x1)
      RESOURCECONSUMER_EXCLUSIVE, // Flags (Arg5 | Arg0: ResourceConsumer | Exclusive)
      PULL_DOWN,            // Pin configuration (Arg1: PullDown)
      IMX_ALT1,0x00,        // Function Number (Arg2: IMX_ALT0)
      PIN_TABLE_OFFSET,     // Pin Table Offset (0x12)
      0x00,                 // Resource Source Index (Arg4: 0)
      0x18,0x00,            // Resource Source Name Offset (0x12 + sizeof(PinList))
      0x22,0x00,            // Vendor Data Offset (0x12 + sizeof(PinList) + sizeof(ResourceName))
      0x05,0x00,            // Vendor Data Length (sizeof(Arg6) = 0)
      IMX_PAD_GPIO_IO09,0x00,IMX_PAD_GPIO_IO10,0x00,IMX_PAD_GPIO_IO11,0x00,  // Pin List (IMX_PAD_GPIO_IO09, IMX_PAD_GPIO_IO10, IMX_PAD_GPIO_IO11)
      SB_GPIO,              // Resource Name (Arg3: \_SB.GPIO in ASCII)
      0x01, 0xFE, 0x05, 0x00, 0x00
    }
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz",    24000000U},
        Package (2) {"MaxConnectionSpeedHz", 1000000U},  // 0 means no limitation applied
        Package (2) {"SampleOnDelayedSckEdge", 0}, // 0 - sample on SCK edge, 1 - sample on delayed SCK edge
      }
  })
}

Device (SPI4)
{
  Name (_HID, "NXP0121")
  Name (_HRV, 0x1)  // REV_0001
  Name (_UID, 0x4)
  Method (_STA)
  {
    Return(0x0)
  }
  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED(ReadWrite, 0x42560000, 0x10000, )
    Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 98 }
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 24000000U},
        Package (2) {"MaxConnectionSpeedHz", 0U},  // 0 means no limitation applied
        Package (2) {"SampleOnDelayedSckEdge", 0}, // 0 - sample on SCK edge, 1 - sample on delayed SCK edge
      }
  })
}

Device (SPI5)
{
  Name (_HID, "NXP0121")
  Name (_HRV, 0x1)  // REV_0001
  Name (_UID, 0x5)
  Method (_STA)
  {
    Return(0x0)
  }
  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED(ReadWrite, 0x426f0000, 0x10000, )
    Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 223 }
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 24000000U},
        Package (2) {"MaxConnectionSpeedHz", 0U},  // 0 means no limitation applied
        Package (2) {"SampleOnDelayedSckEdge", 0}, // 0 - sample on SCK edge, 1 - sample on delayed SCK edge
      }
  })
}

Device (SPI6)
{
  Name (_HID, "NXP0121")
  Name (_HRV, 0x1)  // REV_0001
  Name (_UID, 0x6)
  Method (_STA)
  {
    Return(0x0)
  }
  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED(ReadWrite, 0x42700000, 0x10000, )
    Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 224 }
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 24000000U},
        Package (2) {"MaxConnectionSpeedHz", 0U},  // 0 means no limitation applied
        Package (2) {"SampleOnDelayedSckEdge", 0}, // 0 - sample on SCK edge, 1 - sample on delayed SCK edge
      }
  })
}

Device (SPI7)
{
  Name (_HID, "NXP0121")
  Name (_HRV, 0x1)  // REV_0001
  Name (_UID, 0x7)
  Method (_STA)
  {
    Return(0x0)
  }
  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED(ReadWrite, 0x42710000, 0x10000, )
    Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 225 }
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 24000000U},
        Package (2) {"MaxConnectionSpeedHz", 0U},  // 0 means no limitation applied
        Package (2) {"SampleOnDelayedSckEdge", 0}, // 0 - sample on SCK edge, 1 - sample on delayed SCK edge
      }
  })
}

Device (SPI8)
{
  Name (_HID, "NXP0121")
  Name (_HRV, 0x1)  // REV_0001
  Name (_UID, 0x8)
  Method (_STA)
  {
    Return(0x0)
  }
  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED(ReadWrite, 0x42720000, 0x10000, )
    Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 226 }
  })
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ReferenceClockHz", 24000000U},
        Package (2) {"MaxConnectionSpeedHz", 0U},  // 0 means no limitation applied
        Package (2) {"SampleOnDelayedSckEdge", 0}, // 0 - sample on SCK edge, 1 - sample on delayed SCK edge
      }
  })
}

