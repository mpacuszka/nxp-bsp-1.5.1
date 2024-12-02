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

/*
* Description: iMX93 I2C Controllers
*/

// I2C1
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
      MEMORY32FIXED(ReadWrite, 0x44340000, 0x10000, )
      // Comment out Interrupt if polling mode should be used in imxlpi2c driver
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 45 }
    })
    Return(RBUF)
  }
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ModuleClockFrequencyHz", 40000000U},
      }
  })
}

// I2C2
Device (I2C2)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x2)

  Method (_STA)
  {
    Return(0xf)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x44350000, 0x10000, )
      // Comment out Interrupt if polling mode should be used in imxlpi2c driver
      //Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 46 }
    })
    Return(RBUF)
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ModuleClockFrequencyHz", 40000000U},
      }
  })
}

// I2C3
Device (I2C3)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x3)

  Method (_STA)
  {
    Return(0xF)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x42530000, 0x10000, )
      // Comment out Interrupt if polling mode should be used in imxlpi2c driver
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 94 }
    })
    Return(RBUF)
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"ModuleClockFrequencyHz", 40000000U},
      }
  })
}

// I2C4
Device (I2C4)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x4)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x42540000, 0x10000, )
      // Comment out Interrupt if polling mode should be used in imxlpi2c driver
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 95 }
    })
    Return(RBUF)
  }
}

// I2C5
Device (I2C5)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x5)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x426B0000, 0x10000, )
      // Comment out Interrupt if polling mode should be used in imxlpi2c driver
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 227 }
    })
    Return(RBUF)
  }
}

// I2C6
Device (I2C6)
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
      MEMORY32FIXED(ReadWrite, 0x426C0000, 0x10000, )
      // Comment out Interrupt if polling mode should be used in imxlpi2c driver
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 228 }
    })
    Return(RBUF)
  }
}

// I2C7
Device (I2C7)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x7)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x426D0000, 0x10000, )
      // Comment out Interrupt if polling mode should be used in imxlpi2c driver
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 229 }
    })
    Return(RBUF)
  }
}

// I2C8
Device (I2C8)
{
  Name (_HID, "NXP0119")
  Name (_HRV, 0x1)
  Name (_UID, 0x8)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized)
  {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x426E0000, 0x10000, )
      // Comment out Interrupt if polling mode should be used in imxlpi2c driver
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 230 }
    })
    Return(RBUF)
  }
}

