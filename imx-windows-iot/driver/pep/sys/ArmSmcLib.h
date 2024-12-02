/** @file
*
*  Copyright (c) 2012-2014, ARM Limited. All rights reserved.
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

#ifndef __ARM_SMC_LIB__
#define __ARM_SMC_LIB__

#if defined(_WIN64)
typedef UINT64 UINTN;

#else
typedef UINT32 UINTN;

#endif

/**
 * The size of the SMC arguments are different between AArch64 and AArch32.
 * The native size is used for the arguments.
 */
typedef struct {
  UINTN  Arg0;
  UINTN  Arg1;
  UINTN  Arg2;
  UINTN  Arg3;
  UINTN  Arg4;
  UINTN  Arg5;
  UINTN  Arg6;
  UINTN  Arg7;
} ARM_SMC_ARGS;

/**
  Trigger an SMC call

  SMC calls can take up to 7 arguments and return up to 4 return values.
  Therefore, the 4 first fields in the ARM_SMC_ARGS structure are used
  for both input and output values.

**/
extern "C"
VOID
ArmCallSmc (
  IN OUT ARM_SMC_ARGS *Args
  );

/*******************************************************************************
 * Defines for runtime services function ids
 ******************************************************************************/
#define PSCI_VERSION             ULONG(0x84000000)
#define PSCI_CPU_SUSPEND_AARCH32 ULONG(0x84000001)
#define PSCI_CPU_SUSPEND_AARCH64 ULONG(0xc4000001)
#define PSCI_CPU_OFF             ULONG(0x84000002)
#define PSCI_CPU_ON_AARCH32      ULONG(0x84000003)
#define PSCI_CPU_ON_AARCH64      ULONG(0xc4000003)

#define IMX_PWR_LVL0            0
#define IMX_PWR_LVL1            1
#define IMX_PWR_LVL2            2

#define PSTATE_PWR_LVL_SHIFT    ULONG(24)
#define PSTATE_PWR_LVL_MASK     ULONG(0x3)


#endif
