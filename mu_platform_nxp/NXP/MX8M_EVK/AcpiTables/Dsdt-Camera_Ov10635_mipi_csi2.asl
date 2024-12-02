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

Device (CAM1) // AVStream
{
  Name (_HID, "NXP0C21")  // _HID: Hardware ID
  Name (_UID, "1")  // _UID: Unique ID
  Name (_CCA, 0x0)

#if PRE_29615_B3
  // Uncomment for SCH-29615 PDF: SPF-29615 B3
  Name (_DEP, Package() {\_SB.I2C1.SNS1, \_SB.CSI1, \_SB.MIP1})
#else
  // Uncomment for SCH-29615 PDF: SPF-29615 B4
  Name (_DEP, Package() {\_SB.I2C1.SNS1, \_SB.CSI1, \_SB.MIP1})
#endif

  Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
  {
    Name (SBUF, ResourceTemplate ()
    {
    })
    Return (SBUF) /* \_SB.CAM1._CRS.SBUF */
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"MipiEndpoint0", "\\DosDevices\\MIP1"},
        Package (2) {"CsiEndpoint0", "\\DosDevices\\CSI1"},
        Package (2) {"SnsEndpoint0", "\\DosDevices\\SNS1"},
        Package (2) {"CpuId", 0x82},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
      Return (0x0F)
  }
}

Device (CSI1) // CSI on SoC peripheral
{
  Name (_HID, "NXP0C11")  // _HID: Hardware ID
  Name (_UID, "1")  // _UID: Unique ID
  Name (_CCA, 0x0)

  Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
  {
    Name (SBUF, ResourceTemplate ()
    {
      Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 75 }
      Memory32Fixed (ReadWrite, 0x30B80000, FixedPcdGet32 (PcdMipiCsiDeviceMemorySize), )
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
        Package (2) {"DeviceEndpoint0", "\\DosDevices\\CSI1"},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
      Return (0x0F)
  }
}

Device (MIP1) // MIPI on SoC peripheral
{
  Name (_HID, "NXP0C13")  // _HID: Hardware ID
  Name (_UID, "1")  // _UID: Unique ID
  Name (_CCA, 0x0)

  Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
  {
    Name (SBUF, ResourceTemplate ()
    {
      Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 77 }
      Memory32Fixed (ReadWrite, 0x30B70000, FixedPcdGet32 (PcdMipiCsiDeviceMemorySize), )
      Memory32Fixed (ReadWrite, 0x30B60000, FixedPcdGet32 (PcdMipiCsiDeviceMemorySize), )
    })
    Return (SBUF) /* \_SB.MIP0._CRS.SBUF */
  }

  Method (RGPR, 0, NotSerialized)
  // Method WGPR: Write GPR register
  //   Returns  Description
  //   ---  -------------------------------
  //     0  Value in the register
  {
    Return (GR41)
  }

  Method (WGPR, 1, NotSerialized)
  // Method WGPR: Write GPR register
  //   Arg  Description
  //   ---  -------------------------------
  //     0  Value to be written
  {
    Store (Arg0, GR41)
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
    Local0 = MI2P
    Local0 &= 0xFFFFFFF8

    if (Arg0 > 0)
    {
      Local0 |= MIPI_RCR_ESC_RESET_BIT | MIPI_RCR_PHY_REF_RESET_BIT | MIPI_RCR_CORE_RESET_BIT
    }
    else
    {
      Local0 &= ~(MIPI_RCR_ESC_RESET_BIT | MIPI_RCR_PHY_REF_RESET_BIT | MIPI_RCR_CORE_RESET_BIT)
    }
    Store (Local0, MI2P)
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"PhyClockFrequencyHz", 100000000},
        Package (2) {"EscClockFrequencyHz", 66666667},
        Package (2) {"Mipi1RegResId", 1},
        Package (2) {"DeviceEndpoint0", "\\DosDevices\\MIP1"},
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
  Device (SNS1)
  {
    Name (_HID, "NXP0C16")  // _HID: Hardware ID
    Name (_UID, "1")  // _UID: Unique ID
    Name (_CCA, 0x0)

    Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
    {
      Name (SBUF, ResourceTemplate ()
      {
        //MAX9286(Deserializer) device address = 0x6A
        I2CSerialBus(0x6A, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")

        //MAX9271(Serializer) default device address = 0x40
        I2CSerialBus(0x40, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        //MAX9271(Serializeres) unique device addresses to be programmed
        I2CSerialBus(0x41, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x42, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x43, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x44, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        //MAX9271(Serializer) broadcast device address = 0x40
        I2CSerialBus(0x45, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")

        //OV10635 Camera default address
        I2CSerialBus(0x30, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        //OV10635(Cameras) unique device addresses to be programmed
        I2CSerialBus(0x31, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x32, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x33, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x34, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")

        GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 5 } // 0 * 32 + 3
    #if !(FixedPcdGet32(PcdCsi1CameraOv10635) || FixedPcdGet32(PcdCsi1CameraOv5640))
        GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 6 } // 0 * 32 + 6
    #endif
      })
      Return (SBUF) /* \_SB.SNS1._CRS.SBUF */
    }
    Name (_DSD, Package () {
      ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
          Package () {"GpioPwnResId", 0},
    #if !(FixedPcdGet32(PcdCsi1CameraOv10635) || FixedPcdGet32(PcdCsi1CameraOv5640))
          Package () {"GpioRstResId", 1},
    #endif
          Package () {"I2cResIdMax9286",          0},
          Package () {"I2cResIdMax9271",          1},
          Package () {"I2cResIdMax9271_0",        2},
          Package () {"I2cResIdMax9271_1",        3},
          Package () {"I2cResIdMax9271_2",        4},
          Package () {"I2cResIdMax9271_3",        5},
          Package () {"I2cResIdMax9271_Broadcast", 6},
          Package () {"I2cResIdOv10635"  ,        7},
          Package () {"I2cResIdOv10635_0",        8},
          Package () {"I2cResIdOv10635_1",        9},
          Package () {"I2cResIdOv10635_2",        10},
          Package () {"I2cResIdOv10635_3",        11},
          Package () {"CameraClockFrequencyHz", 25000000},
          Package (2) {"DeviceEndpoint0", "\\DosDevices\\SNS1"},
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
Scope (\_SB.I2C1)
{
  Device (SNS1)
  {
    Name (_HID, "NXP0C16")  // _HID: Hardware ID
    Name (_UID, "1")  // _UID: Unique ID
    Name (_CCA, 0x0)

    Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
    {
      Name (SBUF, ResourceTemplate ()
      {
        //MAX9286(Deserializer) device address = 0x6A
        I2CSerialBus(0x6A, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")

        //MAX9271(Serializer) default device address = 0x40
        I2CSerialBus(0x40, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        //MAX9271(Serializeres) unique device addresses to be programmed
        I2CSerialBus(0x41, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x42, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x43, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x44, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        //MAX9271(Serializer) broadcast device address = 0x40
        I2CSerialBus(0x45, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")

        //OV10635 Camera default address
        I2CSerialBus(0x30, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        //OV10635(Cameras) unique device addresses to be programmed
        I2CSerialBus(0x31, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x32, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x33, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")
        I2CSerialBus(0x34, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C1")

        GpioIO(Shared, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 5 } // 0 * 32 + 3
    #if !(FixedPcdGet32(PcdCsi1CameraOv10635) || FixedPcdGet32(PcdCsi1CameraOv5640))
        GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 6 } // 0 * 32 + 6
    #endif
      })
      Return (SBUF) /* \_SB.SNS1._CRS.SBUF */
    }
    Name (_DSD, Package () {
      ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
          Package () {"GpioPwnResId", 0},
    #if !(FixedPcdGet32(PcdCsi1CameraOv10635) || FixedPcdGet32(PcdCsi1CameraOv5640))
          Package () {"GpioRstResId", 1},
    #endif
          Package () {"I2cResIdMax9286",          0},
          Package () {"I2cResIdMax9271",          1},
          Package () {"I2cResIdMax9271_0",        2},
          Package () {"I2cResIdMax9271_1",        3},
          Package () {"I2cResIdMax9271_2",        4},
          Package () {"I2cResIdMax9271_3",        5},
          Package () {"I2cResIdMax9271_Broadcast", 6},
          Package () {"I2cResIdOv10635"  ,        7},
          Package () {"I2cResIdOv10635_0",        8},
          Package () {"I2cResIdOv10635_1",        9},
          Package () {"I2cResIdOv10635_2",        10},
          Package () {"I2cResIdOv10635_3",        11},
          Package () {"CameraClockFrequencyHz", 25000000},
          Package (2) {"DeviceEndpoint0", "\\DosDevices\\SNS1"},
        }
    })

    Method (_STA, 0, NotSerialized)  // _STA: Status
    {
      Return (0x0F)
    }
  }
}

#endif
