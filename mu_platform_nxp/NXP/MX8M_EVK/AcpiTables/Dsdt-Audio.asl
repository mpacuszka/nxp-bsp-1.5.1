/*
* Description: iMX8M Synchronous Audio Interface (SAI)
*
*  Copyright (c) 2018, Microsoft Corporation. All rights reserved.
*  Copyright 2023 NXP
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
      MEMORY32FIXED(ReadWrite, 0x30010000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 127 }
    })
    Return(RBUF)
  }
}

Device (SAI2)
{
  Name (_HID, "NXP0112")
  Name (_UID, 0x2)

  Method (_STA)
  {
    Return(0xF)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x308B0000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 128 }
      // channel 7 of SDMA1
      FixedDMA (SDMA_REQ_SAI2_RX, 7, Width32Bit, )
      // channel 8 of SDMA1
      FixedDMA (SDMA_REQ_SAI2_TX, 8, Width32Bit, )
    })
    Return(RBUF)
  }
  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"VERSION", 2},
        // All words masked, capture interface is disabled
        Package (2) {"RX_WORD_MASK", 0xFFFFFFFF},
      }
  })
}

Device (SAI3)
{
  Name (_HID, "NXP0110")
  Name (_UID, 0x3)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x308C0000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 82 }
    })
    Return(RBUF)
  }
}

Device (SAI4)
{
  Name (_HID, "NXP0110")
  Name (_UID, 0x4)

  Method (_STA)
  {
    Return(0x0)
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30050000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 132 }
    })
    Return(RBUF)
  }
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
      MEMORY32FIXED(ReadWrite, 0x30040000, 0x100, )
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
      MEMORY32FIXED(ReadWrite, 0x30030000, 0x100, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Shared) { 122 }
    })
    Return(RBUF)
  }
}

