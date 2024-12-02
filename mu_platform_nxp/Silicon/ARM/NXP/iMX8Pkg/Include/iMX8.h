/** @file
*
*  Header defining the iMX8 constants (Base addresses, sizes, flags)
*
*  Copyright (c), Microsoft Corporation. All rights reserved.
*  Copyright 2019-2020, 2022 NXP
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

#ifndef __IMX8_H__
#define __IMX8_H__

//
// Memory mapping
//

#if defined(CPU_IMX8MM)
#include "iMX8MM/MIMX8MM6_ca53.h"
#include "iMX8DisplayMix.h"
#elif defined(CPU_IMX8MN)
#include "iMX8MN/MIMX8MN6_ca53.h"
#include "iMX8DisplayMix.h"
#elif defined(CPU_IMX8MP)
#include "iMX8MP/MIMX8MP8_ca53.h"
#elif defined(CPU_IMX8QXP)
#include "iMX8QX/MIMX8QX6_ca35.h"
#elif defined(CPU_IMX93)
#include "iMX93/MIMX9352_ca55.h"
#else
#include "iMX8MQ/MIMX8MQ7_ca53.h"
#endif

//
// Interrupts
//
#define TOTAL_PRIVATE_INTERRUPT     32
#define DEFINE_IMX8_INTERRUPT(a)    (a + TOTAL_PRIVATE_INTERRUPT)

//
// Serial debug port
//
#if defined(ADMA__LPUART0_BASE_PTR)
#define UART0_BASE_ADDRESS   ((unsigned long)ADMA__LPUART0_BASE_PTR)
#endif
#if defined(UART1_BASE_PTR)
#define UART1_BASE_ADDRESS   ((unsigned long)UART1_BASE_PTR)
#endif
#if defined(ADMA__LPUART1_BASE_PTR)
#define UART1_BASE_ADDRESS   ((unsigned long)ADMA__LPUART1_BASE_PTR)
#endif
#if defined(LPUART1_BASE_PTR)
#define UART1_BASE_ADDRESS   ((unsigned long)LPUART1_BASE_PTR)
#endif
#if defined(UART2_BASE_PTR)
#define UART2_BASE_ADDRESS   ((unsigned long)UART2_BASE_PTR)
#endif
#if defined(ADMA__LPUART2_BASE_PTR)
#define UART2_BASE_ADDRESS   ((unsigned long)ADMA__LPUART2_BASE_PTR)
#endif
#if defined(LPUART2_BASE_PTR)
#define UART2_BASE_ADDRESS   ((unsigned long)LPUART2_BASE_PTR)
#endif
#if defined(UART3_BASE_PTR)
#define UART3_BASE_ADDRESS   ((unsigned long)UART3_BASE_PTR)
#endif
#if defined(ADMA__LPUART3_BASE_PTR)
#define UART3_BASE_ADDRESS   ((unsigned long)ADMA__LPUART3_BASE_PTR)
#endif
#if defined(LPUART3_BASE_PTR)
#define UART3_BASE_ADDRESS   ((unsigned long)LPUART3_BASE_PTR)
#endif
#if defined(UART4_BASE_PTR)
#define UART4_BASE_ADDRESS   ((unsigned long)UART4_BASE_PTR)
#endif
#if defined(LPUART4_BASE_PTR)
#define UART4_BASE_ADDRESS   ((unsigned long)LPUART4_BASE_PTR)
#endif
#if defined(LPUART5_BASE_PTR)
#define UART5_BASE_ADDRESS   ((unsigned long)LPUART5_BASE_PTR)
#endif
#if defined(LPUART6_BASE_PTR)
#define UART6_BASE_ADDRESS   ((unsigned long)LPUART6_BASE_PTR)
#endif
#if defined(LPUART7_BASE_PTR)
#define UART7_BASE_ADDRESS   ((unsigned long)LPUART7_BASE_PTR)
#endif
#if defined(LPUART8_BASE_PTR)
#define UART18_BASE_ADDRESS   ((unsigned long)LPUART8_BASE_PTR)
#endif

#if defined(UART_BASE_PTRS)
#define UART_IMX8_UART_ADDRESS_SIZE    0x000000BC
#define IMX_SERIAL_DBG_PORT_SUBTYPE     0x000C
#endif
#if defined(LPUART_BASE_PTRS)
#define UART_IMX8_UART_ADDRESS_SIZE    0x00000030
#define IMX_SERIAL_DBG_PORT_SUBTYPE     0x0013
#endif


#define SERIAL_DEBUG_PORT_INIT_MSG "\r\nUEFI Debug Serial Port Init\r\n"

#ifdef MU_BASE_PTRS
    typedef struct MU_MemMap MU_Type;
#define SC_IPC_HDL ((sc_ipc_t)PcdGet64(PcdMuSCFWRegisterBase))
#endif

//
// Clock Source
//
#if defined(CPU_IMX8MM) || defined(CPU_IMX8MN) || defined(CPU_IMX8MP) || defined(CPU_IMX8QXP) || defined(CPU_IMX93)
#define SOC_OSC_FREQUENCY_REF_HZ  24000000  // Oscillator frequency 24Mhz
#else
#define SOC_OSC_FREQUENCY_REF_HZ  25000000  // Oscillator frequency 25Mhz
#endif

// SDMA (Smart DMA) controllers
#if defined(SDMAARM1_BASE_PTR)
#define SDMA1_BASE_ADDRESS      ((unsigned long)SDMAARM1_BASE_PTR)
#define SDMA1_IRQ               INT_SDMA1
#endif
#if defined(SDMAARM2_BASE_PTR)
#define SDMA2_BASE_ADDRESS      ((unsigned long)SDMAARM2_BASE_PTR)
#define SDMA2_IRQ               INT_SDMA2
#endif
#if defined(SDMAARM3_BASE_PTR)
#define SDMA3_BASE_ADDRESS      ((unsigned long)SDMAARM3_BASE_PTR)
#define SDMA3_IRQ               INT_SDMA3
#endif

//
// GPIO
//
#if defined(GPIO1_BASE_PTR)
#define IMX_GPIO_BASE               ((unsigned long)GPIO1_BASE_PTR)
#endif
#if defined(LSIO__GPIO0_BASE_PTR)
#define IMX_GPIO_BASE               ((unsigned long)LSIO__GPIO0_BASE_PTR)
#endif

//
// IOMux:
// IMX8M specific
//
#if defined(IOMUXC_GPR_BASE_PTR)
#define IOMUXC_GPR_BASE_ADDRESS             ((unsigned long)IOMUXC_GPR_BASE_PTR)
#endif
#if defined(IOMUXC_BASE_PTR)
#define IOMUXC_SW_MUX_PAD_BASE_ADDRESS      ((unsigned long)IOMUXC_BASE_PTR)
#define IOMUXC_SELECT_INPUT_BASE_ADDRESS    ((unsigned long)&IOMUXC_SELECT_INPUT_REG(IOMUXC_BASE_PTR,0))
#endif
#if defined(IOMUXC1_BASE_PTR)
#define IOMUXC_SW_MUX_PAD_BASE_ADDRESS      ((unsigned long)IOMUXC1_BASE_PTR)
#define IOMUXC_SELECT_INPUT_BASE_ADDRESS    ((unsigned long)&IOMUXC_CAN1_IPP_IND_CANRX_SELECT_INPUT_REG(IOMUXC1_BASE_PTR))
#define IOMUXC_SELECT_INPUT_COUNT                (68U)
#endif

/* Macro for GPC SIP */
#define IMX_SIP_GPC                     0xC2000000
/* Macro for fill SIP structure */
#define imx_fill_sip(_Arg0, _Arg1, _Arg2, _Arg3, _Arg4, _str)   \
                        {                                       \
                            _str.Arg0 = _Arg0;                  \
                            _str.Arg1 = _Arg1;                  \
                            _str.Arg2 = _Arg2;                  \
                            _str.Arg3 = _Arg3;                  \
                            _str.Arg4 = _Arg4;                  \
                        };


/* TODO New IoMap.h BEGIN */

#define IOMUXC_MUX_SION_DISABLED    0x00000000
#define IOMUXC_MUX_SION_ENABLED     0x00000010
#define IOMUXC_MUX_ALT0             0x00000000
#define IOMUXC_MUX_ALT1             0x00000001
#define IOMUXC_MUX_ALT2             0x00000002
#define IOMUXC_MUX_ALT3             0x00000003
#define IOMUXC_MUX_ALT4             0x00000004
#define IOMUXC_MUX_ALT5             0x00000005
#define IOMUXC_MUX_ALT6             0x00000006
#if defined(CPU_IMX93)
#define IOMUXC_MUX_ALT7             0x00000007
#define IOMUXC_PAD_HYS_DISABLED     0x00000000
#define IOMUXC_PAD_HYS_ENABLED      0x00001000
#define IOMUXC_PAD_ODE_DISABLED     0x00000000
#define IOMUXC_PAD_ODE_ENABLED      0x00000800
#define IOMUXC_PAD_PDE_DISABLED     0x00000000
#define IOMUXC_PAD_PDE_ENABLE       0x00000400
#define IOMUXC_PAD_PUE_DISABLED     0x00000000
#define IOMUXC_PAD_PUE_ENABLE       0x00000200
#define IOMUXC_PAD_SRE_SLOW         0x00000000
#define IOMUXC_PAD_SRE_MEDIUM       0x00000080
#define IOMUXC_PAD_SRE_FAST         0x00000100
#define IOMUXC_PAD_SRE_MAX          0x00000180
#define IOMUXC_PAD_DSE_DISABLED     0x00000000
#define IOMUXC_PAD_DSE_R0_DIV_1     0x00000002  // R0 divided by 1
#define IOMUXC_PAD_DSE_R0_DIV_2     0x00000006  // R0 divided by 2
#define IOMUXC_PAD_DSE_R0_DIV_3     0x0000000E  // R0 divided by 3
#define IOMUXC_PAD_DSE_R0_DIV_4     0x0000001E  // R0 divided by 4
#define IOMUXC_PAD_DSE_R0_DIV_5     0x0000003E  // R0 divided by 5
#define IOMUXC_PAD_DSE_R0_DIV_6     0x0000007E  // R0 divided by 6
#else
#define IOMUXC_MUX_ALT7             0x00000001
#define IOMUXC_LVTTL_DISABLED       0x00000000
#define IOMUXC_LVTTL_ENABLED        0x00000100
#define IOMUXC_PAD_HYS_DISABLED     0x00000000
#define IOMUXC_PAD_HYS_ENABLED      0x00000080
#define IOMUXC_PAD_PUE_DISABLED     0x00000000
#define IOMUXC_PAD_PUE_ENABLE       0x00000040
#define IOMUXC_PAD_ODE_DISABLED     0x00000000
#define IOMUXC_PAD_ODE_ENABLED      0x00000020
#define IOMUXC_PAD_SRE_SLOW         0x00000000
#define IOMUXC_PAD_SRE_MEDIUM       0x00000008
#define IOMUXC_PAD_SRE_FAST         0x00000010
#define IOMUXC_PAD_SRE_MAX          0x00000018

#define IOMUXC_PAD_DSE_DISABLED     0x00000000
#define IOMUXC_PAD_DSE_R0_DIV_1     0x00000001  // R0 divided by 1
#define IOMUXC_PAD_DSE_R0_DIV_2     0x00000002  // R0 divided by 2
#define IOMUXC_PAD_DSE_R0_DIV_3     0x00000003  // R0 divided by 3
#define IOMUXC_PAD_DSE_R0_DIV_4     0x00000004  // R0 divided by 4
#define IOMUXC_PAD_DSE_R0_DIV_5     0x00000005  // R0 divided by 5
#define IOMUXC_PAD_DSE_R0_DIV_6     0x00000006  // R0 divided by 6
#define IOMUXC_PAD_DSE_R0_DIV_7     0x00000007  // R0 divided by 7
#endif

#if defined(CPU_IMX8MP)
#define GPC_PU_PGC_SW_PUP_REQ_PCIE_SW_PUP_REQ_MASK (0x2U)
#endif
#endif // __IMX8_PLATFORM_H__
