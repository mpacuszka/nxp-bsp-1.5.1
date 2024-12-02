/*
* Description: Processor Devices
*
*  Copyright 2022 NXP
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

// #include "iMX8.h"

/* Shared general purpose registers. */

#define MEDIA_BLK_CTRL_BASE  0x32EC0000
#define MEDIA_BLK_CTRL_SIZE  0x8000
OperationRegion(GPR, SystemMemory, MEDIA_BLK_CTRL_BASE, MEDIA_BLK_CTRL_SIZE)
Field(GPR, DWordAcc, Nolock, Preserve)
{
  RSTN, 32,
  CLKE, 32,
  Offset(0x50),
  ICTL, 32, // ISI CACHE Control Register
  //Gasket 0 registers
  Offset(0x60),
  G0CT, 32, // Gasket 0 Control Register
  G0HS, 32, // Gasket 0 Video Horizontal Size Register
  G0VS, 32, // Gasket 0 Video Vertical Size Register
  G0HF, 32, // Gasket 0 Video Horizontal Front Porch Register
  G0HB, 32, // Gasket 0 Video Horizontal Back Porch Register
  G0VF, 32, // Gasket 0 Video Vertical Front Porch Register
  G0VB, 32, // Gasket 0 Video Vertical Back Porch Register
  G0PC, 32, // Gasket 0 ISI Pixel Count Register
  G0LC, 32, // Gasket 0 ISI Line Count Register
  G0PI, 32, // Gasket 0 ISI Pixel Control Information Register
  //Gasket 1 registers
  Offset(0x90),
  G1CT, 32, // Gasket 1 Control Register
  G1HS, 32, // Gasket 1 Video Horizontal Size Register
  G1VS, 32, // Gasket 1 Video Vertical Size Register
  G1HF, 32, // Gasket 1 Video Horizontal Front Porch Register
  G1HB, 32, // Gasket 1 Video Horizontal Back Porch Register
  G1VF, 32, // Gasket 1 Video Vertical Front Porch Register
  G1VB, 32, // Gasket 1 Video Vertical Back Porch Register
  G1PC, 32, // Gasket 1 ISI Pixel Count Register
  G1LC, 32, // Gasket 1 ISI Line Count Register
  G1PI, 32, // Gasket 1 ISI Pixel Control Information Register
}

