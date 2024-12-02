// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022 NXP
// Licensed under the MIT License.
//
//
// Module Name:
//
//   imxlpuarthw.h
//
// Abstract:
//
//   IMX LPUART Hardware Register Definitions
//

#ifndef _IMX_LPUART_HW_H_
#define _IMX_LPUART_HW_H_

//
// Size of RX and TX FIFOs
//
enum : ULONG { IMX_LPUART_FIFO_COUNT = 32 };

#define IMX_LPUART_BAUD_MAEN1          0x80000000
#define IMX_LPUART_BAUD_MAEN2          0x40000000
#define IMX_LPUART_BAUD_M10            0x20000000
#define IMX_LPUART_BAUD_TDMAE          0x00800000
#define IMX_LPUART_BAUD_RDMAE          0x00200000
#define IMX_LPUART_BAUD_MATCFG         0x00400000
#define IMX_LPUART_BAUD_BOTHEDGE       0x00020000
#define IMX_LPUART_BAUD_RESYNCDIS      0x00010000
#define IMX_LPUART_BAUD_LBKDIE         0x00008000
#define IMX_LPUART_BAUD_RXEDGIE        0x00004000
#define IMX_LPUART_BAUD_SBNS           0x00002000
#define IMX_LPUART_BAUD_SBR            0x00000000
#define IMX_LPUART_BAUD_SBR_MASK       0x1FFF
#define IMX_LPUART_BAUD_OSR_MASK       0x1F
#define IMX_LPUART_BAUD_OSR_SHIFT      24

#define IMX_LPUART_STAT_LBKDIF         0x80000000
#define IMX_LPUART_STAT_RXEDGIF        0x40000000
#define IMX_LPUART_STAT_MSBF           0x20000000
#define IMX_LPUART_STAT_RXINV          0x10000000
#define IMX_LPUART_STAT_RWUID          0x08000000
#define IMX_LPUART_STAT_BRK13          0x04000000
#define IMX_LPUART_STAT_LBKDE          0x02000000
#define IMX_LPUART_STAT_RAF            0x01000000
#define IMX_LPUART_STAT_TDRE           0x00800000
#define IMX_LPUART_STAT_TC             0x00400000
#define IMX_LPUART_STAT_RDRF           0x00200000
#define IMX_LPUART_STAT_IDLE           0x00100000
#define IMX_LPUART_STAT_OR             0x00080000
#define IMX_LPUART_STAT_NF             0x00040000
#define IMX_LPUART_STAT_FE             0x00020000
#define IMX_LPUART_STAT_PE             0x00010000
#define IMX_LPUART_STAT_MA1F           0x00008000
#define IMX_LPUART_STAT_M21F           0x00004000

#define IMX_LPUART_CTRL_R8T9           0x80000000              // Receive Bit 8 / Transmit Bit 9
#define IMX_LPUART_CTRL_R9T8           0x40000000              // Receive Bit 9 / Transmit Bit 8
#define IMX_LPUART_CTRL_TXDIR          0x20000000              // TX Pin Direction in Single-Wire Mode
#define IMX_LPUART_CTRL_TXINV          0x10000000              // Transmit Data Inversion
#define IMX_LPUART_CTRL_ORIE           0x08000000              // Overrun Interrupt Enable
#define IMX_LPUART_CTRL_NEIE           0x04000000              // Noise Error Interrupt Enable
#define IMX_LPUART_CTRL_FEIE           0x02000000              // Framing Error Interrupt Enable
#define IMX_LPUART_CTRL_PEIE           0x01000000              // Parity Error Interrupt Enable
#define IMX_LPUART_CTRL_TIE            0x00800000              // Transmit Interrupt Enable
#define IMX_LPUART_CTRL_TCIE           0x00400000              // Transmission Complete Interrupt Enable
#define IMX_LPUART_CTRL_RIE            0x00200000              // Receiver Interrupt Enable
#define IMX_LPUART_CTRL_ILIE           0x00100000              // Idle Line Interrupt Enable
#define IMX_LPUART_CTRL_TE             0x00080000              // Transmitter Enable
#define IMX_LPUART_CTRL_RE             0x00040000              // Receiver Enable
#define IMX_LPUART_CTRL_RWU            0x00020000              // Receiver Wakeup Control
#define IMX_LPUART_CTRL_SBK            0x00010000              // Send Break
#define IMX_LPUART_CTRL_MA1IE          0x00008000              // Match 1 Interrupt Enable
#define IMX_LPUART_CTRL_MA2IE          0x00004000              // Match 2 Interrupt Enable
#define IMX_LPUART_CTRL_M7             0x00000800              // 7-Bit Mode Select
#define IMX_LPUART_CTRL_IDLECFG        0x00000100              // Idle Configuration
#define IMX_LPUART_CTRL_LOOPS          0x00000080              // Loop Mode Select
#define IMX_LPUART_CTRL_DOZEEN         0x00000040              // Doze Enable
#define IMX_LPUART_CTRL_RSRC           0x00000020              // Receiver Source Select
#define IMX_LPUART_CTRL_M              0x00000010              // 9-Bit or 8-Bit Mode Select
#define IMX_LPUART_CTRL_WAKE           0x00000008              // Receiver Wakeup Method Select
#define IMX_LPUART_CTRL_ILT            0x00000004              // Idle Line Type Select
#define IMX_LPUART_CTRL_PE             0x00000002              // Parity Enable
#define IMX_LPUART_CTRL_PT             0x00000001              // Parity Type

#define IMX_LPUART_DATA_NOISY          0x00008000              // The current received dataword contained in DATA[R9:R0] was received with noise.
#define IMX_LPUART_DATA_PARITYE        0x00004000              // The current received dataword contained in DATA[R9:R0] was received with a parity error.
#define IMX_LPUART_DATA_FRETSC         0x00002000              // Frame Error / Transmit Special Character
#define IMX_LPUART_DATA_RXEMPT         0x00001000              // Receive Buffer Empty
#define IMX_LPUART_DATA_IDLINE         0x00000800              // Idle Line
#define IMX_LPUART_DATA_MASK           0x3FF

#define IMX_LPUART_MODIR_IREN          0x00020000              // Infrared enable
#define IMX_LPUART_MODIR_TXCTSSRC      0x00000020              // Transmit CTS Source
#define IMX_LPUART_MODIR_TXCTSC        0x00000010              // Transmit CTS Configuration
#define IMX_LPUART_MODIR_RTSWATER_MASK 0x3F00              // Receiver request-to-send enable
#define IMX_LPUART_MODIR_RTSWATER(x)   ((x << 8) & IMX_LPUART_MODIR_RTSWATER_MASK)     // Receiver request-to-send enable
#define IMX_LPUART_MODIR_RXRTSE        0x00000008              // Receiver request-to-send enable
#define IMX_LPUART_MODIR_TXRTSPOL      0x00000004              // Transmitter request-to-send polarity
#define IMX_LPUART_MODIR_TXRTSE        0x00000002              // Transmitter request-to-send enable
#define IMX_LPUART_MODIR_TXCTSE        0x00000001              // Transmitter clear-to-send enable

#define IMX_LPUART_FIFO_TXEMPT         0x00800000              // Transmit Buffer/FIFO Empty
#define IMX_LPUART_FIFO_RXEMPT         0x00400000              // Receive Buffer/FIFO Empty
#define IMX_LPUART_FIFO_TXOF           0x00020000              // Transmitter Buffer Overflow Flag
#define IMX_LPUART_FIFO_RXUF           0x00010000              // Receiver Buffer Underflow Flag
#define IMX_LPUART_FIFO_TXFLUSH        0x00008000              // Transmit FIFO/Buffer Flush
#define IMX_LPUART_FIFO_RXFLUSH        0x00004000              // Receive FIFO/Buffer Flush

#define IMX_LPUART_FIFO_RXIDEN_32CHARS 0x00001800              // Enables the assertion of RDRF when the receiver is idle
#define IMX_LPUART_FIFO_TXOFE          0x00000200              // Transmit FIFO Overflow Interrupt Enable
#define IMX_LPUART_FIFO_RXUFE          0x00000100              // Receive FIFO Underflow Interrupt Enable
#define IMX_LPUART_FIFO_TXFE           0x00000080              // Transmit FIFO Enable
#define IMX_LPUART_FIFO_SIZE_MASK      0b111
#define IMX_LPUART_FIFO_TXSIZE_OFF     4
#define IMX_LPUART_FIFO_RXFE           0x00000008              // Receive FIFO Enable
#define IMX_LPUART_FIFO_RXSIZE_OFF     0
#define IMX_LPUART_FIFO_DEPTH(x)       (0x1 << ((x) ? ((x) + 1) : 0)) // Zjisti jakou delku ma FIFO v HW

#define IMX_LPUART_WATER_COUNT_MASK    0xFF
#define IMX_LPUART_WATER_TXCNT_OFF     8
#define IMX_LPUART_WATER_RXCNT_OFF     24
#define IMX_LPUART_WATER_WATER_MASK    0xFF
#define IMX_LPUART_WATER_TXWATER_MASK    0xF
#define IMX_LPUART_WATER_TXWATER_OFF   0
#define IMX_LPUART_WATER_RXWATER_OFF   16

#define IMX_LPUART_GLOBAL_RST          0x2

struct IMX_LPUART_REGISTERS {
    ULONG VERID;                // 0x00 Version ID Register (VERID)                32 RO 0401_0003
    ULONG PARAM;                // 0x04 Parameter Register (PARAM)                 32 RO 0000_0505
    ULONG GLOBAL;               // 0x08 LPUART Global Register (GLOBAL)            32 RW 0000_0000
    ULONG PINCNF;               // 0x0C LPUART Pin Configuration Register (PINCFG) 32 RW 0000_0000
    ULONG BAUD;                 // 0x10 LPUART Baud Rate Register (BAUD)           32 RW 0F00_0004
    ULONG STAT;                 // 0x14 LPUART Status Register (STAT)              32 RW 00C0_0000
    ULONG CTRL;                 // 0x18 LPUART Control Register (CTRL)             32 RW 0000_0000
    ULONG DATA;                 // 0x1C LPUART Data Register (DATA)                32 RW 0000_1000
    ULONG MATCH;                // 0x20 LPUART Match Address Register (MATCH)      32 RW 0000_0000
    ULONG MODIR;                // 0x24 LPUART Modem IrDA Register (MODIR)         32 RW 0000_0000
    ULONG FIFO;                 // 0x28 LPUART FIFO Register (FIFO)                32 RW 00C0_0044
    ULONG WATER;                // 0x2C LPUART Watermark Register (WATER)          32 RW 0000_0000
};

#endif // _IMX_LPUART_HW_H_
