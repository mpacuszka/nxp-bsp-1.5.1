/** @file
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
*  Copyright 2019, 2023 NXP
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
  Name (_HID, "VERI7001")
  Name (_CID, "VERI7001")
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
    // LCDIF1 reg
    MEMORY32FIXED (ReadWrite, 0x32E80000, 0x10000, )
    // LCDIF2 reg
    MEMORY32FIXED (ReadWrite, 0x32E90000, 0x10000, )
    // MIPI-DSI reg
    MEMORY32FIXED (ReadWrite, 0x32E60000, 0x10000, )
    // MEDIA BLK_CTL reg
    MEMORY32FIXED (ReadWrite, 0x32EC0000, 0x10000, )
    // HDMI subsystem
    MEMORY32FIXED (ReadWrite, 0x32FC0000, 0x20000, )
    // First must be GPU interrupt
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 35 }
    // LCDIF1 interrupt
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 37 }
    // LCDIF2 interrupt
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 38 }
    // HDMI + LCDIF3 interrupt
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 75 }
    // MIPI-DSI interrupt
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 50 }
    // I2C interface for IMX-LVDS-HDMI converter (IT6263).
    // First must be IT6263 i2c_hdmi connection (bus addr 0x4C). Use "\\_SB.I2C2" for LVDS0, "\\_SB.I2C3" for LVDS1 
    // Second must be IT6263 i2C_lvds connection (bus addr 0x33). Use "\\_SB.I2C2" for LVDS0, "\\_SB.I2C3" for LVDS1 
    I2CSerialBus(0x4C, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C2")
    I2CSerialBus(0x33, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C2")
    // MIPI-DSI I2C interface for the IMX-MIPI-HDMI converter (ADV7535).
    // First must be main MIPI I2C connection, Second cec MIPI I2C connection, third edid MIPI I2C connection.
    I2CSerialBus(0x3d, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C2")
    I2CSerialBus(0x3b, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C2")
    I2CSerialBus(0x41, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C2")
    //DSI_EN pin GPIO1_IO8 for IMX-DSI-OLED display reset
    GpioIO(Exclusive, PullNone, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 8 } // 0 * 32 + 8 DSI_EN
  })
}
