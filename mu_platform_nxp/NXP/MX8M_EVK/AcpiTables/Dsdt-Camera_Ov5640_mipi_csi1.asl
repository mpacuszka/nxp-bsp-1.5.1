/*
 * Copyright 2022 NXP
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

// The i.MX 8M EVK boards differ in routing. Configuration defaults to schematic 29615 rev B4. Set PRE_29615_B3 to 1 for rev B3 board that uses different I2C bus for camera chip.
#define PRE_29615_B3 0

Device (CAM0) // AVStream
{
  Name (_HID, "NXP0C21")  // _HID: Hardware ID
  Name (_UID, "0")  // _UID: Unique ID
  Name (_CCA, 0x0)

#if PRE_29615_B3
  // Uncomment for SCH-29615 PDF: SPF-29615 B3
  Name (_DEP, Package() {\_SB.I2C1.SNS0, \_SB.CSI0, \_SB.MIP0})
#else
  // Uncomment for SCH-29615 PDF: SPF-29615 B4
  Name (_DEP, Package() {\_SB.I2C2.SNS0, \_SB.CSI0, \_SB.MIP0})
#endif

  Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
  {
    Name (SBUF, ResourceTemplate ()
    {
    })
    Return (SBUF) /* \_SB.CAM0._CRS.SBUF */
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"MipiEndpoint0", "\\DosDevices\\MIP0"},
        Package (2) {"CsiEndpoint0", "\\DosDevices\\CSI0"},
        Package (2) {"SnsEndpoint0", "\\DosDevices\\SNS0"},
        Package (2) {"CpuId", 0x82},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
      Return (0x0F)
  }
}

Device (CSI0) // CSI on SoC peripheral
{
  Name (_HID, "NXP0C11")  // _HID: Hardware ID
  Name (_UID, "0")  // _UID: Unique ID
  Name (_CCA, 0x0)

  Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
  {
    Name (SBUF, ResourceTemplate ()
    {
      Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 74 }
      Memory32Fixed (ReadWrite, 0x30A90000, FixedPcdGet32 (PcdMipiCsiDeviceMemorySize), )
    })
    Return (SBUF) /* \_SB.CSI0._CRS.SBUF */
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"CoreClockFrequencyHz", 133000000},
        Package (2) {"Csi1RegResId", 0},
        Package (2) {"Two8bitSensorMode", 1},
        Package (2) {"CpuId", 0x82},
        Package (2) {"DeviceEndpoint0", "\\DosDevices\\CSI0"},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
      Return (0x0F)
  }
}

Device (MIP0) // MIPI on SoC peripheral
{
  Name (_HID, "NXP0C13")  // _HID: Hardware ID
  Name (_UID, "0")  // _UID: Unique ID
  Name (_CCA, 0x0)

  Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
  {
    Name (SBUF, ResourceTemplate ()
    {
      Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 76 }
      Memory32Fixed (ReadWrite, 0x30A80000, FixedPcdGet32 (PcdMipiCsiDeviceMemorySize), )
      Memory32Fixed (ReadWrite, 0x30A70000, FixedPcdGet32 (PcdMipiCsiDeviceMemorySize), )
    })
    Return (SBUF) /* \_SB.MIP0._CRS.SBUF */
  }

  Method (RGPR, 0, NotSerialized)
  // Method WGPR: Write GPR register
  //   Returns  Description
  //   ---  -------------------------------
  //     0  Value in the register
  {
    Return (GR34)
  }

  Method (WGPR, 1, NotSerialized)
  // Method WGPR: Write GPR register
  //   Arg  Description
  //   ---  -------------------------------
  //     0  Value to be written
  {
    Store (Arg0, GR34)
  }

#define MIPI_RCR_ESC_RESET_BIT (1u << 2)
#define MIPI_RCR_PHY_REF_RESET_BIT (1u << 1)
#define MIPI_RCR_CORE_RESET_BIT (1u << 0)
  Method (WRST, 1, NotSerialized)
  // Method WRST: Assert MIPI reset
  //   Arg  Description
  //   ---  -------------------------------
  //     0  0 - clear the bitss, 1 - assert the bits
  {
    Local0 = MI1P
    Local0 &= 0xFFFFFFF8

    if (Arg0 > 0)
    {
      Local0 |= MIPI_RCR_ESC_RESET_BIT | MIPI_RCR_PHY_REF_RESET_BIT | MIPI_RCR_CORE_RESET_BIT
    }
    else
    {
      Local0 &= ~(MIPI_RCR_ESC_RESET_BIT | MIPI_RCR_PHY_REF_RESET_BIT | MIPI_RCR_CORE_RESET_BIT)
    }
    Store (Local0, MI1P)
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"PhyClockFrequencyHz", 100000000},
        Package (2) {"EscClockFrequencyHz", 66666667},
        Package (2) {"Mipi1RegResId", 1},
        Package (2) {"DeviceEndpoint0", "\\DosDevices\\MIP0"},
        Package (2) {"CpuId", 0x82},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
    Return (0x0F)
  }
}

#if PRE_29615_B3
// Uncomment for SCH-29615 PDF: SPF-29615 B3
Scope (\_SB.I2C1)
{
  Device (SNS0)
  {
    Name (_HID, "NXP0C14")  // _HID: Hardware ID
    Name (_UID, "0")  // _UID: Unique ID
    Name (_CCA, 0x0)

    Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
    {
      Name (SBUF, ResourceTemplate ()
      {
        I2CSerialBus(0x3c, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")

        GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 3 } // 0 * 32 + 3
    #if !(FixedPcdGet32(PcdCsi2CameraOv10635) || FixedPcdGet32(PcdCsi2CameraOv5640))
        GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 6 } // 0 * 32 + 6
    #endif
      })
      Return (SBUF) /* \_SB.SNS0._CRS.SBUF */
    }
    Name (_DSD, Package () {
      ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
          Package () {"GpioPwnResId", 0},
    #if !(FixedPcdGet32(PcdCsi2CameraOv10635) || FixedPcdGet32(PcdCsi2CameraOv5640))
          Package () {"GpioRstResId", 1},
    #endif
          Package () {"I2cResId", 0},
          Package () {"CameraClockFrequencyHz", 25000000},
          Package (2) {"DeviceEndpoint0", "\\DosDevices\\SNS0"},
        }
    })

    Method (_STA, 0, NotSerialized)  // _STA: Status
    {
      Return (0x0F)
    }
  }
}

#else
// Uncomment for SCH-29615 PDF: SPF-29615 B4
Scope (\_SB.I2C2)
{
  Device (SNS0)
  {
    Name (_HID, "NXP0C14")  // _HID: Hardware ID
    Name (_UID, "0")  // _UID: Unique ID
    Name (_CCA, 0x0)

    Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
    {
      Name (SBUF, ResourceTemplate ()
      {
        I2CSerialBus(0x3c, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C2")

        GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 3 } // 0 * 32 + 3
    #if !(FixedPcdGet32(PcdCsi2CameraOv10635) || FixedPcdGet32(PcdCsi2CameraOv5640))
        GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 6 } // 0 * 32 + 6
    #endif
      })
      Return (SBUF) /* \_SB.SNS0._CRS.SBUF */
    }
    Name (_DSD, Package () {
      ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
          Package () {"GpioPwnResId", 0},
    #if !(FixedPcdGet32(PcdCsi2CameraOv10635) || FixedPcdGet32(PcdCsi2CameraOv5640))
          Package () {"GpioRstResId", 1},
    #endif
          Package () {"I2cResId", 0},
          Package () {"CameraClockFrequencyHz", 25000000},
          Package (2) {"DeviceEndpoint0", "\\DosDevices\\SNS0"},
        }
    })

    Method (_STA, 0, NotSerialized)  // _STA: Status
    {
      Return (0x0F)
    }
  }
}

#endif
