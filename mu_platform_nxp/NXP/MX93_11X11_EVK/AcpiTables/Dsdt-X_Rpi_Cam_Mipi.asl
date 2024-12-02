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

Device (CAM0) // AVStream
{
  Name (_HID, "NXP0C21")  // _HID: Hardware ID
  Name (_UID, "0")  // _UID: Unique ID
  Name (_CCA, 0x0)
  Name (_DEP, Package() {\_SB.I2C3.SNS0, \_SB.ISI0, \_SB.MIP0})


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
        Package (2) {"CpuId", 0xA1},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
    Return (0x0F)
  }
}


Device (MIP0) // DWC_MIPI_CSI (MIPI CSI camera interface)
{
  Name (_HID, "NXP0C20")  // _HID: Hardware ID
  Name (_UID, "0")  // _UID: Unique ID
  Name (_CCA, 0x0)

  Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
  {
    Name (SBUF, ResourceTemplate ()
    {
      // Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 207 }
      Memory32Fixed (ReadWrite, 0x4AE00000, 0x1000, )
    })
    Return (SBUF) /* \_SB.MIP0._CRS.SBUF */
  }

  Method (RGPR, 1, NotSerialized)
  // Method RGPR: Read GPR register
  //   Arg  Description
  //   ---  -------------------------------
  //     0  Register offset (Access to registers that do not belong to this peripheral returns 1)
  //   Returns  Description
  //   ---  -------------------------------
  //     0  Status: 0 - Success, 1 - Failure (Invalid register)
  //     1  (Only on Success) Value red from the register
  {
#define BLK_CTRL_MEDIAMIX_RESET                   0x00
#define BLK_CTRL_MEDIAMIX_CLK                     0x04
#define BLK_CTRL_MEDIAMIX_LCDIF                   0x0C
#define BLK_CTRL_MEDIAMIX_PXP                     0x10
#define BLK_CTRL_MEDIAMIX_ISI0                    0x14
#define BLK_CTRL_MEDIAMIX_ISI1                    0x1C
#define BLK_CTRL_MEDIAMIX_CAMERA_MUX              0x30
#define BLK_CTRL_MEDIAMIX_CSI                     0x48
#define BLK_CTRL_MEDIAMIX_IF_CTRL_REG             0x70
#define BLK_CTRL_MEDIAMIX_INTERFACE_STATUS        0x74
#define BLK_CTRL_MEDIAMIX_INTERFACE_CTRL_REG      0x78
#define BLK_CTRL_MEDIAMIX_INTERFACE_CTRL_REG1     0x7C

    local0 = 0

    Switch (ToInteger (Arg0)) {
        Case ( BLK_CTRL_MEDIAMIX_RESET ) {
            local0 = RSTN
        }
        Case ( BLK_CTRL_MEDIAMIX_CLK ) {
            local0 = CLKR
        }
        Case ( BLK_CTRL_MEDIAMIX_LCDIF ) {
            local0 = QOSL
        }
        Case ( BLK_CTRL_MEDIAMIX_PXP ) {
            local0 = QOSP
        }
        Case ( BLK_CTRL_MEDIAMIX_ISI0 ) {
            local0 = CACI
        }
        Case ( BLK_CTRL_MEDIAMIX_ISI1 ) {
            local0 = QOSI
        }
        Case ( BLK_CTRL_MEDIAMIX_CAMERA_MUX ) {
            local0 = CAMM
        }
        Case ( BLK_CTRL_MEDIAMIX_CSI ) {
            local0 = CSIR
        }
        Case ( BLK_CTRL_MEDIAMIX_IF_CTRL_REG ) {
            local0 = PCIR
        }
        Case ( BLK_CTRL_MEDIAMIX_INTERFACE_STATUS ) {
            local0 = INSR
        }
        Case ( BLK_CTRL_MEDIAMIX_INTERFACE_CTRL_REG ) {
            local0 = ICR0
        }
        Case ( BLK_CTRL_MEDIAMIX_INTERFACE_CTRL_REG1 ) {
            local0 = ICR1
        }
        default {
            Return(1)
        }
    }
    Name (RGPB, Package(2) {0x0, 0x0}) // Couldn't get "Return (Package(2) {0, local0})" to work, so I name the returned package first.
    Store(local0, Index(RGPB, 1))
    Return (RGPB)
  }

    Method (WGPR, 2, NotSerialized)
  // Method WGPR: Write GPR register
  //   Arg  Description
  //   ---  -------------------------------
  //     0  Register offset (Access to registers that do not belong to this peripheral returns 1)
  //     1  Value to be written
  //   Returns  Description
  //   ---  -------------------------------
  //     0  Status: 0 - Success, 1 - Failure (Invalid register)
  {
    local0 = Arg0
    local2 = 0

    Switch (ToInteger (local0)) {
        Case ( BLK_CTRL_MEDIAMIX_RESET ) {
            Store (Arg1, RSTN)
        }
        Case ( BLK_CTRL_MEDIAMIX_CLK ) {
            Store (Arg1, CLKR)
        }
        Case ( BLK_CTRL_MEDIAMIX_LCDIF ) {
            Store (Arg1, QOSL)
        }
        Case ( BLK_CTRL_MEDIAMIX_PXP ) {
            Store (Arg1, QOSP)
        }
        Case ( BLK_CTRL_MEDIAMIX_ISI0 ) {
            Store (Arg1, CACI)
        }
        Case ( BLK_CTRL_MEDIAMIX_ISI1 ) {
            Store (Arg1, QOSI)
        }
        Case ( BLK_CTRL_MEDIAMIX_CAMERA_MUX ) {
            Store (Arg1, CAMM)
        }
        Case ( BLK_CTRL_MEDIAMIX_CSI ) {
            Store (Arg1, CSIR)
        }
        Case ( BLK_CTRL_MEDIAMIX_IF_CTRL_REG ) {
            Store (Arg1, PCIR)
        }
        Case ( BLK_CTRL_MEDIAMIX_INTERFACE_STATUS ) {
            Store (Arg1, INSR)
        }
        Case ( BLK_CTRL_MEDIAMIX_INTERFACE_CTRL_REG ) {
            Store (Arg1, ICR0)
        }
        Case ( BLK_CTRL_MEDIAMIX_INTERFACE_CTRL_REG1 ) {
            Store (Arg1, ICR1)
        }
        default {
          local2 = 1
        }
    }
    Return (local2)
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"Mipi1RegResId", 0},
        Package (2) {"MipiCsrRegResId", 1},
        Package (2) {"DeviceEndpoint0", "\\DosDevices\\MIP0"},
        Package (2) {"CpuId", 0xA1},
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
      Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 204 }
      Memory32Fixed (ReadWrite, 0x4AE40000, FixedPcdGet32 (PcdIsiChannelMemorySize), )
    })
    Return (SBUF) /* \_SB.CSI0._CRS.SBUF */
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"CoreClockFrequencyHz", 600000000},
  
        // MipiCsiSrc - Desired MIPI CSI input data port.
        Package (2) {"MipiCsiSrc", 2},
            // IMX_ISI_SRC_DC0         = 0, /* Pixel Link input 0 */
            // IMX_ISI_SRC_DC1         = 1, /* Pixel Link input 1 */
            // IMX_ISI_SRC_MIPI_CSI_0  = 2, /* Pixel Link input 2 */
            // IMX_ISI_SRC_MIPI_CSI_1  = 3, /* Pixel Link input 3 */
            // IMX_ISI_SRC_CI_PI       = 4, /* Pixel Link input 4 */
            // IMX_ISI_SRC_MEM         = 5  /* Pixel Link input 5 */
        Package (2) {"Isi1RegResId", 0},
        // Package (2) {"IsiFbMemReserveResId", 1},
        // Package (2) {"Isi1Ch1IsrResId", 0},
        Package (2) {"CpuId", 0xA1},
        Package (2) {"DeviceEndpoint0", "\\DosDevices\\ISI0"},
      }
  })

  Method (_STA, 0, NotSerialized)  // _STA: Status
  {
    Return (0x0F)
  }
}

Scope (\_SB.I2C3)
{
  Device (SNS0) //AP1302
  {
    Name (_HID, "NXP0C19")  // _HID: Hardware ID
    Name (_UID, "0")  // _UID: Unique ID
    Name (_CCA, 0x0)

    Method (_CRS, 0, NotSerialized)  // _CRS: Current Resource Settings
    {
      Name (SBUF, ResourceTemplate ()
      {
        //ADP5585 I/O port expander on X-RPi-CAM-MIPI board, device address = 0x34
        I2CSerialBus(0x34, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C3")
        
        //AP1302 ISP, device address = 0x3C
        I2CSerialBus(0x3C, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C3")

        //ADP5585 I/O port expander on MX93 baseboard, device address = 0x34
        I2CSerialBus(0x34, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C2")

        // GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 36 } // 1 * 32 + 4 RESET 
        // GpioIO(Exclusive, PullUp, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 37 } // 1 * 32 + 5 PWDN
      })
      Return (SBUF) /* \_SB.SNS0._CRS.SBUF */
    }
    Name (_DSD, Package () {
      ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
          // Package () {"GpioPwnResId", 0},
          // Package () {"GpioRstResId", 1},
          Package () {"CameraClockFrequencyHz", 24000000},
          Package (2) {"DeviceEndpoint0", "\\DosDevices\\SNS0"},
          Package () {"I2c1ResIdAdp5585",      0},
          Package () {"I2c1ResIdAp1302",       1},
          Package () {"I2c2ResIdAdp5585",      2},
        }
    })

    Method (_STA, 0, NotSerialized)  // _STA: Status
    {
      Return (0xF)
    }
  }
}