/*
 * Copyright 2022-2024 NXP
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
* Description: iMX8QXP I2C Controllers
*/

// I2C0 is not exposed
Device (I2C0)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x0)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name ( RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x5a800000, 0x4000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 372 }     // GIC_SPI
      I2CSerialBus(0x50, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    })
    Return(RBUF)
  }
}

// I2C1 exposes PTN5110, PCA9646 I2C switch, GPIO expanders, ISL29023, FXAS210002,...
Device (I2C1)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x1)

  Method (_STA)
  {
    Return(0xf)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name ( RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x5a810000, 0x4000, )
      // Comment out Interrupt if polling mode should be used in imxlpi2c driver
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 373 }     // GIC_SPI
      I2CSerialBus(0x51, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    })
    Return(RBUF)
  }
}

// I2C2 is not exposed
Device (I2C2)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x2)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x5a820000, 0x4000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 374 }     // GIC_SPI
      I2CSerialBus(0x52, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    })
    Return(RBUF)
  }
}

// I2C3 is not exposed
Device (I2C3)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x3)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x5a830000, 0x4000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 375 }     // GIC_SPI
      I2CSerialBus(0x53, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    })
    Return(RBUF)
  }
}

//Device (I2C4)   // Reserved for I2C4, which is not on QXP

Device (I2C5)   // I2C0_MIPI_CSI0
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x5)

  Method (_STA)
  {
    Return(0xf)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x58226000, 0x1000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 352 }       // GIC_SPI of irqsteer_csi0 
      I2CSerialBus(0x55, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    })
    Return(RBUF)
  }
}

Device (I2C6)   // PARRALEL
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x6)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x58266000, 0x1000, )
      //Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 354 }     // GIC_SPI of irqsteer_parallel
      I2CSerialBus(0x56, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    })
    Return(RBUF)
  }
}

//Device (I2C7) // Reserved for I2C0_MIPI_CSI1, which is not on QXP

Device (I2C8)   // I2C0_MIPI_LVDS0
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x8)

  Method (_STA)
  {
    Return(0xf)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x56226000, 0x1000, )
/* MIPI-DSI panel with LP communication need the i2C interrupt be commented-out here,
   because it interferes with MIPI-DSI interrupt (irq steer not supported)
   For other displays, i2c interrupt can be enabled here and commented-out in Dsdt-Gfx.asl instead */
//      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 91 }       // GIC SPI
      I2CSerialBus(0x58, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    })
    Return(RBUF)
  }
}

Device (I2C9)   // I2C0_MIPI_LVDS1
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x9)

  Method (_STA)
  {
    Return(0xf)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x56246000, 0x1000, )
/* MIPI-DSI panel with LP communication need the i2C interrupt be commented-out here,
   because it interferes with MIPI-DSI interrupt (irq steer not supported)
   For other displays, i2c interrupt can be enabled here and commented-out in Dsdt-Gfx.asl instead */
//      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 92 }       // GIC SPI
      I2CSerialBus(0x59, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    })
    Return(RBUF)
  }
}

//Device (I2CA)   // Reserved for I2C1_LVDS0, which is not on QXP

//Device (I2CB)   // Reserved for I2C0_LVDS1, which is not on QXP

//Device (I2CC)   // Reserved for I2C1_LVDS1, which is not on QXP

//Device (I2CD)   // Reserved for I2C0_MIPI_DSI1, which is not on QXP

//Device (I2CE)   // Reserved for I2C0_HDMI, which is not on QXP

// CM40_oxposes WM8960 audio codec, wifi-bt module, CAN io expander,...
Device (I2CF)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0xf)

  Method (_STA)
  {
    Return(0xf)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x37230000, 0x1000, )
      // Comment out Interrupt if polling mode should be used in imxlpi2c driver
      // Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 48 }       // Cortex-M4 (48 - 55)
      I2CSerialBus(0x5f, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    })
    Return(RBUF)
  }
}

//Device (I2CG)   // Reserved for CM41_I2C, which is not on QXP
