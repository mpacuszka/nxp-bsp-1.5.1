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

#define DISPLAY_MIX_GPR_BASE  0x32E28000
#define DISPLAY_MIX_GPR_SIZE  0x8000
OperationRegion(GPR, SystemMemory, DISPLAY_MIX_GPR_BASE, DISPLAY_MIX_GPR_SIZE)
Field(GPR, DWordAcc, Nolock, Preserve)
{
  RSTN, 32,
  CLKE, 32,
  Offset(0x50),
  ICTL, 32, // ISI CACHE Control Register
  Offset(0x60),
  GCTL, 32, // Gasket 0 Control Register
  GHSZ, 32, // Gasket 0 Video Horizontal Size Register
  GVSZ, 32, // Gasket 0 Video Vertical Size Register
  GHFP, 32, // Gasket 0 Video Horizontal Front Porch Register
  GHBP, 32, // Gasket 0 Video Horizontal Back Porch Register
  GVFP, 32, // Gasket 0 Video Vertical Front Porch Register
  GVBP, 32, // Gasket 0 Video Vertical Back Porch Register
  GIPC, 32, // Gasket 0 ISI Pixel Count Register
  GILC, 32, // Gasket 0 ISI Line Count Register 
  GPCT, 32, // Gasket 0 ISI Pixel Control Information Register
}

