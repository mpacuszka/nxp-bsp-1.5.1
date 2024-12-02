/** @file
*
*  Copyright (c) Microsoft Corporation. All rights reserved.
*  Copyright 2022-2024 NXP
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

Device (GPU1)
{
  Name (_HID, "VERI7004")
  Name (_CID, "VERI7004")
  Name (_UID, 0)

  Method (_STA)
  {
    Return (0xF)
  }

  Name (_CRS, ResourceTemplate () {
    // First memory block must be GPU REG
    MEMORY32FIXED( ReadWrite, 0x53100000, 0x40000, )
    // Second memory block must be Framebuffer allocated.
    MEMORY32FIXED( ReadWrite, FixedPcdGet32(PcdArmLcdDdrFrameBufferBase), FixedPcdGet32(PcdArmLcdDdrFrameBufferSize), )
    // Third is the size and location of reserved memory area for GPU driver
    MEMORY32FIXED( ReadWrite, FixedPcdGet32(PcdArmGPUReservedMemoryBase), FixedPcdGet32(PcdArmGPUReservedMemorySize), )
    MEMORY32FIXED( ReadWrite, 0x56010000, 0x1000, ) // LPCG
    MEMORY32FIXED( ReadWrite, 0x56020000, 0x1000, ) // Pixel Combiner
    MEMORY32FIXED( ReadWrite, 0x56060000, 0x1000, ) // PRG2
    MEMORY32FIXED( ReadWrite, 0x56070000, 0x1000, ) // PRG3
    MEMORY32FIXED( ReadWrite, 0x56080000, 0x1000, ) // PRG4
    MEMORY32FIXED( ReadWrite, 0x56090000, 0x1000, ) // PRG5
    MEMORY32FIXED( ReadWrite, 0x560A0000, 0x1000, ) // PRG6
    MEMORY32FIXED( ReadWrite, 0x560B0000, 0x1000, ) // PRG7
    MEMORY32FIXED( ReadWrite, 0x560C0000, 0x1000, ) // PRG8
    MEMORY32FIXED( ReadWrite, 0x560F0000, 0x1000, ) // DPR0-2
    MEMORY32FIXED( ReadWrite, 0x56100000, 0x1000, ) // DPR1-0
    MEMORY32FIXED( ReadWrite, 0x56110000, 0x1000, ) // DPR1-1
    MEMORY32FIXED( ReadWrite, 0x56200000, 0x1000, ) // DPL0
    // MIPI DSI / LVDS #0
    MEMORY32FIXED( ReadWrite, 0x56221000, 0x1000, )  // CSR
    MEMORY32FIXED( ReadWrite, 0x56228000, 0x1000, )  // MIPI-DSI
    // MIPI DSI / LVDS #1
    MEMORY32FIXED( ReadWrite, 0x56241000, 0x1000, )  // CSR
    MEMORY32FIXED( ReadWrite, 0x56248000, 0x1000, )  // MIPI-DSI
    // GPU interrupt
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 96 }
    // DPU Stream #0
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 73 }
    // DPU Stream #1
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 75 }
    // DPR #0
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 82 }
    // DPR #1
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 83 }
    //MIPI-DSI0 - shared with I2C8
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 91 }
    //MIPI-DSI1 - shared with I2C9
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 92 }
    // SCFW
    I2CSerialBus(0x41, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.SCFW")
    // I2C interface for IMX-LVDS-HDMI converter (IT6263) connected to MIPI-DSI/LVDS #0
    I2CSerialBus(0x4C, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C8") // IT6263 HDMI memory region
    I2CSerialBus(0x33, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C8") // IT6263 LVDS memory region
    // I2C interface for IMX-LVDS-HDMI converter (IT6263) connected to MIPI-DSI/LVDS #1
    I2CSerialBus(0x4C, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C9") // IT6263 HDMI memory region
    I2CSerialBus(0x33, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C9") // IT6263 LVDS memory region
    // I2C interface for the IMX-MIPI-HDMI converter (ADV7535) connected to MIPI-DSI/LVDS #0
    I2CSerialBus(0x3d, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C8") //ADV7535 Main memory region
    I2CSerialBus(0x3b, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C8") //ADV7535 CEC memory region
    I2CSerialBus(0x41, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C8") //ADV7535 EDID memory region
    // I2C interface for the IMX-MIPI-HDMI converter (ADV7535) connected to MIPI-DSI/LVDS #1
    I2CSerialBus(0x3d, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C9") //ADV7535 Main memory region
    I2CSerialBus(0x3b, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C9") //ADV7535 CEC memory region
    I2CSerialBus(0x41, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C9") //ADV7535 EDID memory region
    //I2C interface for Expander driven GPIO pin MIPI_DSI0_EN
    I2CSerialBus(0x1A, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C1") //U25 PCA9557PW
    //I2C interface for Expander driven GPIO pin MIPI_DSI1_EN
    I2CSerialBus(0x1D, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C1") //U187 PCA9557PW
  })
}
