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

/** @file
* Description: iMX8MP I2C Controllers
*/

// I2C1 exposes the PMIC
Device (I2C1)
{
  Name (_HID, "NXP0104")
  Name (_HRV, 0x1)
  Name (_UID, 0x1)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name ( RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30A20000, 0x14, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 67 }
    })
    Return(RBUF)
  }
}

// I2C2 exposes USB voltage regulator,  PCIe clock generator 
// I2C2 is exposed on M.2, Camera 1#, DSI display, LVDS0 display connectors
Device (I2C2)
{
  Name (_HID, "NXP0104")
  Name (_HRV, 0x1)
  Name (_UID, 0x2)

  Method (_STA)
  {
    Return(0xf)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30A30000, 0x14, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 68 }
    })
    Return(RBUF)
  }
}

// I2C3 exposes IO expander, audio codec
// I2C3 is exposed on Camera 2#, LVDS1 display connectors, EXP CN J21
Device (I2C3)
{
  Name (_HID, "NXP0104")
  Name (_HRV, 0x1)
  Name (_UID, 0x3)

  Method (_STA)
  {
    Return(0xf)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30A40000, 0x14, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 69 }
    })
    Return(RBUF)
  }
}

// I2C5 is exposed on I2C CN J22
Device (I2C5)
{
  Name (_HID, "NXP0104")
  Name (_HRV, 0x1)
  Name (_UID, 0x5)

  Method (_STA)
  {
#if FixedPcdGet32(PcdCAN1InsteadOfI2C5)
    Return(0x0)
#else
    Return(0xf)
#endif
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30AD0000, 0x14, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 108 }
    })
    Return(RBUF)
  }
}

