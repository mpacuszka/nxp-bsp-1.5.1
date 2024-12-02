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

/* Shared general purpose registers. */

#define GPR_BASE 0x30340000
OperationRegion(GPR, SystemMemory, GPR_BASE, 0x100)
Field(GPR, DWordAcc, Nolock, Preserve)
{
  Offset(0x88),
  GR34, 32,
  GR35, 32,
  Offset(0xA4),
  GR41, 32,
  GR42, 32,
}


#define SRC_BASE 0x30390000
OperationRegion(SRC, SystemMemory, SRC_BASE, 0x100)
Field(SRC, DWordAcc, Nolock, Preserve)
{
  Offset(0x4C),
  MI1P, 32,
  Offset(0x50),
  MI2P, 32,
}
