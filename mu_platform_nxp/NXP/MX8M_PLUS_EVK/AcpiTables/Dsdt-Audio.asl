/*
* Description: iMX8M Plus Synchronous Audio Interface (SAI)
*
*  Copyright (c) 2018, Microsoft Corporation. All rights reserved.
*  Copyright 2022-2023 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
*/

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
      MEMORY32FIXED(ReadWrite, 0x30C10000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 127 }
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
      MEMORY32FIXED(ReadWrite, 0x30C20000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 128 }
    })
    Return(RBUF)
  }
}

Device (SAI3)
{
  Name (_HID, "NXP0112")
  Name (_UID, 0x3)

  Method (_STA)
  {
    Return(0xF)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30C30000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 82 }
      // We use SDMA2 for Audio subsystem. The channels are defined with offset +128. 
      // 129 -> channel 1 of SDMA2
      FixedDMA (SDMA_REQ_SAI3_RX, 129, Width32Bit, )
      // 130 -> channel 2 of SDMA2
      FixedDMA (SDMA_REQ_SAI3_TX, 130, Width32Bit, )
    })
    Return(RBUF)
  }

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"VERSION", 2},
        Package (2) {"RX_WORD_MASK", 0xFFFFFFFE},
        Package (2) {"SYNCH", 1}
      }
  })
}

Device (SAI5)
{
  Name (_HID, "NXP0110")
  Name (_UID, 0x5)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30C50000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { 122 }
    })
    Return(RBUF)
  }
}

Device (SAI6)
{
  Name (_HID, "NXP0110")
  Name (_UID, 0x6)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30C60000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { 122 }
    })
    Return(RBUF)
  }
}

Device (SAI7)
{
  Name (_HID, "NXP0110")
  Name (_UID, 0x7)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30C80000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { 143 }
    })
    Return(RBUF)
  }
}

Device (AHTX)
{
  Name (_HID, "NXP0130")
  Name (_UID, 0x1)

  Method (_STA)
  {
    Return(0xF)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30CB0000, 0x100, )  /* AUD2HTX - HDMI TX AUDLNK MSTR */
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { 162 }
      // We use SDMA2 for Audio subsystem. The channels are defined with offset +128.
      // 131 -> channel 3 of SDMA2
      FixedDMA (SDMA_REQ_AUD2HTX, 131, Width32Bit, )
    })
    Return(RBUF)
  }
}
