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
  Name (_DEP, Package() {\_SB.I2C5.SNS0, \_SB.ISI0, \_SB.MIP0})


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
        Package (2) {"CsiEndpoint0", "\\DosDevices\\ISI0"},
        Package (2) {"SnsEndpoint0", "\\DosDevices\\SNS0"},
        Package (2) {"CpuId", 0x91},
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
      // Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 76 }
      Memory32Fixed (ReadWrite, 0x58227000, FixedPcdGet32 (PcdMipiCsiDeviceMemorySize), ) /* MIPI CSI0 Controler base addr */
      Memory32Fixed (ReadWrite, 0x58221000, 0x1000, ) /* MIPI CSI0 Subsystem CSR base addr  */
      I2CSerialBus(0x39, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    })
    Return (SBUF) /* \_SB.MIP0._CRS.SBUF */
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
          // assigned-clock-rates = <360000000>, <72000000>;
        Package (2) {"PhyClockFrequencyHz", 360000000U},
        Package (2) {"EscClockFrequencyHz", 72000000},
        Package (2) {"Mipi1RegResId", 0},
        Package (2) {"MipiCsrRegResId", 1},
        Package (2) {"ScfwI2cResId", 0},
        Package (2) {"DeviceEndpoint0", "\\DosDevices\\MIP0"},
        Package (2) {"CpuId", 0x91},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
    Return (0x0F)
  }
}

Device (ISI0) // ISI on SoC peripheral
{
  Name (_HID, "NXP0C12")  // _HID: Hardware ID
  Name (_UID, "0")  // _UID: Unique ID
  Name (_CCA, 0x0)

  Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
  {
    Name (SBUF, ResourceTemplate ()
    {
      Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 329 }
      Memory32Fixed (ReadWrite, 0x58100000, FixedPcdGet32 (PcdIsiChannelMemorySize), )  // ISI
      Memory32Fixed (ReadWrite, FixedPcdGet64 (PcdIsiDma0ReservedMemoryBase), FixedPcdGet64(PcdIsiDmaReservedMemorySize), )  // ISI
    })
    Return (SBUF) /* \_SB.CSI0._CRS.SBUF */
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        // assigned-clock-rates = <600000000>;
        Package (2) {"CoreClockFrequencyHz", 600000000},
        // Package (2) {"PhyClockFrequencyHz", 100000000},
        // Package (2) {"EscClockFrequencyHz", 66666667},
        
        // MipiCsiSrc - Desired MIPI CSI input data port. Posible values:
        //                                                  0 - MIPI-CSI2 instance #1
        //                                                  1 - MIPI-CSI2 instance #2
        Package (2) {"MipiCsiSrc", 2},
            // IMX_ISI_SRC_DC0         = 0, /* Pixel Link input 0 */
            // IMX_ISI_SRC_DC1         = 1, /* Pixel Link input 1 */
            // IMX_ISI_SRC_MIPI_CSI_0  = 2, /* Pixel Link input 2 */
            // IMX_ISI_SRC_MIPI_CSI_1  = 3, /* Pixel Link input 3 */
            // IMX_ISI_SRC_CI_PI       = 4, /* Pixel Link input 4 */
            // IMX_ISI_SRC_MEM         = 5  /* Pixel Link input 5 */
        Package (2) {"Isi1RegResId", 0},
        Package (2) {"IsiFbMemReserveResId", 1},
        // Package (2) {"Isi1Ch1IsrResId", 0},
        Package (2) {"CpuId", 0x91},
        Package (2) {"DeviceEndpoint0", "\\DosDevices\\ISI0"},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
    Return (0x0F)
  }
}

Scope (\_SB.I2C5)
{
  Device (SNS0)
  {
    Name (_HID, "NXP0C14")  // _HID: Hardware ID
    Name (_UID, "0")  // _UID: Unique ID
    Name (_CCA, 0x0)  // SDHC is not coherent

    Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
    {
      Name (SBUF, ResourceTemplate ()
      {
        I2CSerialBus(0x3c, ControllerInitiated, 100000, AddressingMode7Bit, "\\_SB.I2C5")

        // GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 6 } // 3 * 32 + 7 CSI_PWDN 
        // GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 7 } // 3 * 32 + 8 CSI_nRST
      })
      Return (SBUF) /* \_SB.SNS0._CRS.SBUF */
    }
    Name (_DSD, Package () {
      ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
          Package () {"I2cResId", 0},
          // Package () {"GpioPwnResId", 0},
          // Package () {"GpioRstResId", 1},
          Package () {"CameraClockFrequencyHz", 24000000},
          Package (2) {"DeviceEndpoint0", "\\DosDevices\\SNS0"},
        }
    })

    Method (_STA, 0, NotSerialized)  // _STA: Status
    {
      Return (0xF)
    }
  }
}
