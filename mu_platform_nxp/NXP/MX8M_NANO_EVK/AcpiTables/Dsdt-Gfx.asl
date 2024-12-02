/** @file
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
*  Copyright 2019 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

// Description: Graphics Processing Unit (GPU)
Device (GPU0)
{
  Name (_HID, "VERI7003")
  Name (_CID, "VERI7003")
  Name (_UID, 0)

  Method (_STA) {
    Return (0xf)
  }

  Name (_CRS, ResourceTemplate () {
    // First memory block must be GPU registers area
    MEMORY32FIXED( ReadWrite, 0x38000000, 0x10000, )
    // Second memory block must be Framebuffer allocated.
    MEMORY32FIXED( ReadWrite, FixedPcdGet32(PcdArmLcdDdrFrameBufferBase), FixedPcdGet32(PcdArmLcdDdrFrameBufferSize), )
    // Third is the size and location of reserved memory area for GPU driver
    MEMORY32FIXED( ReadWrite, FixedPcdGet32(PcdArmGPUReservedMemoryBase), FixedPcdGet32(PcdArmGPUReservedMemorySize), )
    // LCDIF reg
    MEMORY32FIXED (ReadWrite, 0x32E00000, 0x10000, )
    // MIPI-DSI reg
    MEMORY32FIXED (ReadWrite, 0x32E10000, 0x10000, )
    // First must be GPU interrupt
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 35 }
    // LCDIF interrupt
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 37 }
    // MIPI-DSI interrupt
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 50 }
    // MIPI-DSI I2C interface for the IMX-MIPI-HDMI converter (ADV7535).
    // First must be main MIPI I2C connection, Second cec MIPI I2C connection, third edid MIPI I2C connection.
    I2CSerialBus(0x3d, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C2")
    I2CSerialBus(0x3b, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C2")
    I2CSerialBus(0x41, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C2")
    //DSI_EN pin GPIO1_IO8 for IMX-DSI-OLED display reset
    GpioIO(Exclusive, PullNone, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 8 } // 0 * 32 + 8 DSI_EN
  })
}
