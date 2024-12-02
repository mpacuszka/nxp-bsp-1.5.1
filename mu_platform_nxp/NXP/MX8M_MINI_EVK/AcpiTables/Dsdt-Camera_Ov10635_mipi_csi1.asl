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

Device (CAM0) // AVStream
{
  Name (_HID, "NXP0C21")  // _HID: Hardware ID
  Name (_UID, "0")  // _UID: Unique ID
  Name (_CCA, 0x0)
  Name (_DEP, Package() {\_SB.I2C3.SNS0, \_SB.CSI0, \_SB.MIP0})


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
        Package (2) {"CpuId", 0x85},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
    Return (0x0F)
  }
}

Device (MIP0) // MIPI on SoC peripheral
{
  Name (_HID, "NXP0C15")  // _HID: Hardware ID
  Name (_UID, "0")  // _UID: Unique ID
  Name (_CCA, 0x0)

  Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
  {
    Name (SBUF, ResourceTemplate ()
    {
      Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 49 } // INT_MIPI_CSI 17 + 32
      Memory32Fixed (ReadWrite, 0x32E30000, 0x200, )
    })
    Return (SBUF) /* \_SB.MIP0._CRS.SBUF */
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"PhyClockFrequencyHz", 100000000},
        Package (2) {"EscClockFrequencyHz", 66666667},
        Package (2) {"Mipi1RegResId", 0},
        Package (2) {"Isi1Ch1IsrResId", 0},
        Package (2) {"CpuId", 0x85},
        Package (2) {"PixelMode", 0}, // Single mode - 8b mode
        Package (2) {"DeviceEndpoint0", "\\DosDevices\\MIP0"},
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
      Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 48 } // INT_ISI_CH0 16 + 32
      Memory32Fixed (ReadWrite, 0x32E20000, FixedPcdGet32 (PcdMipiCsiDeviceMemorySize), ) // ISI
    })
    Return (SBUF) /* \_SB.CSI0._CRS.SBUF */
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"CoreClockFrequencyHz", 133000000},
        Package (2) {"Csi1RegResId", 0},
        Package (2) {"CsiTwo8bitSensorMode", 1},
        Package (2) {"CpuId", 0x85},
        Package (2) {"Two8bitSensorMode", 0}, // 8b mode
        Package (2) {"DeviceEndpoint0", "\\DosDevices\\CSI0"},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
    Return (0x0F)
  }
}

Scope (\_SB.I2C3)
{
  Device (SNS0)
  {
    Name (_HID, "NXP0C16")  // _HID: Hardware ID
    Name (_UID, "0")  // _UID: Unique ID
    Name (_CCA, 0x0)  // SDHC is not coherent

    Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
    {
      Name (SBUF, ResourceTemplate ()
      {
        //MAX9286(Deserializer) device address = 0x6A
        I2CSerialBus(0x6A, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")

        //MAX9271(Serializer) default device address = 0x40
        I2CSerialBus(0x40, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")
        //MAX9271(Serializeres) unique device addresses to be programmed
        I2CSerialBus(0x41, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")
        I2CSerialBus(0x42, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")
        I2CSerialBus(0x43, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")
        I2CSerialBus(0x44, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")
        //MAX9271(Serializer) broadcast device address = 0x40
        I2CSerialBus(0x45, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")

        //OV10635 Camera default address
        I2CSerialBus(0x30, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")
        //OV10635(Cameras) unique device addresses to be programmed
        I2CSerialBus(0x31, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")
        I2CSerialBus(0x32, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")
        I2CSerialBus(0x33, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")
        I2CSerialBus(0x34, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C3")

        GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 7 } // 0 * 32 + 7 CSI_PWDN
        GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 6 } // 0 * 32 + 6 CSI_nRST
      })
      Return (SBUF) /* \_SB.SNS0._CRS.SBUF */
    }
    Name (_DSD, Package () {
      ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
          Package (2) {"DeviceEndpoint0", "\\DosDevices\\SNS0"},
          Package () {"I2cResIdMax9286",      0},
          Package () {"I2cResIdMax9271",      1},
          Package () {"I2cResIdMax9271_0",    2},
          Package () {"I2cResIdMax9271_1",    3},
          Package () {"I2cResIdMax9271_2",    4},
          Package () {"I2cResIdMax9271_3",    5},
          Package () {"I2cResIdMax9271_Broadcast", 6},
          Package () {"I2cResIdOv10635"  ,    7},
          Package () {"I2cResIdOv10635_0",    8},
          Package () {"I2cResIdOv10635_1",    9},
          Package () {"I2cResIdOv10635_2",    10},
          Package () {"I2cResIdOv10635_3",    11},
          Package () {"GpioPwnResId", 0},
          Package () {"GpioRstResId", 1},
          Package () {"CameraClockFrequencyHz", 24000000},
        }
    })
     
    Method (_STA, 0, NotSerialized)  // _STA: Status
    {
      Return (0x0F)
    }
  }
}
