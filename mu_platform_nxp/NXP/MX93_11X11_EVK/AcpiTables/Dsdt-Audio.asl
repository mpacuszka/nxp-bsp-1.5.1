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

/* iMX93 Synchronous Audio Interface (SAI) */

Device (SAI1)
{
  Name (_HID, "NXP0110")
  Name (_UID, 0x1)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x443B0000, 0x10000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 77 }
    })
    Return(RBUF)
  }
}

Device (SAI2)
{
  Name (_HID, "NXP0110")
  Name (_UID, 0x2)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x42650000, 0x10000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 202 }
    })
    Return(RBUF)
  }
}

Device (SAI3)
{
  Name (_HID, "NXP0110")
  Name (_UID, 0x3)

  Method (_STA)
  {
    Return(0xf)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x42660000, 0x10000, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 203 }
    })
    Return(RBUF)
  }
}

