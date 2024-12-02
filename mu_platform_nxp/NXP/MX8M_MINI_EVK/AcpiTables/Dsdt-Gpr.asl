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
#define DISPLAY_MIX_GPR_SIZE  0x100
OperationRegion(GPR, SystemMemory, DISPLAY_MIX_GPR_BASE, DISPLAY_MIX_GPR_SIZE)
Field(GPR, DWordAcc, Nolock, Preserve)
{
  RSTN, 32, // Display Mix Software Reset Register
  CLKE, 32, // Display Mix Clock Enable Register
  MPRD, 32, // MIPI PHY Control Register
  MPMS, 32, // Master PLL PMS Value setting Register (GPR_MIPI_M_PLLPMS)
  MPCL, 32, // Master PLL Control Low Register (GPR_MIPI_M_PLLCTL_LOW)
  MPCH, 32, // Master PLL Control High Register (GPR_MIPI_M_PLLCTL_HIGH)
  Offset(0x18),
  MBCL, 32, // Master and Slave DPHY Control Low Register (GPR_MIPI_B_DPHYCTL_LOW)
  MBCH, 32, // Master and Slave DPHY Control High Register (GPR_MIPI_B_DPHYCTL_HIGH)
  MMCL, 32, // Master and Slave DPHY Control Low Register (GPR_MIPI_M_DPHYCTL_LOW)
  MMCH, 32, // Master and Slave DPHY Control High Register (GPR_MIPI_M_DPHYCTL_HIGH)
  MSCL, 32, // Master and Slave DPHY Control Low Register (GPR_MIPI_S_DPHYCTL_LOW)
  MSCH, 32, // Master and Slave DPHY Control High Register (GPR_MIPI_S_DPHYCTL_HIGH)
}

