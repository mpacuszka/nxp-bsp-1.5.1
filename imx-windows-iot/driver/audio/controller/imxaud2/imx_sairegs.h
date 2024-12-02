/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2022-2023 NXP
   Licensed under the MIT License.

Abstract:
    Typedefs for the IMX SAI register blocks.

*/

#pragma once

#define IMX_SAI_MMAP_SIZE           PAGE_SIZE
#define IMX_SAI_FIFO_SIZE_MAX       0xFFFF

#define IMX7D_SAI_REG_OFFSET         0x00
#define IMX8M_SAI_REG_OFFSET         0x08

#define SAI_VERID_OFFSET             0x00
#define SAI_PARAM_OFFSET             0x04

#define SAI_TCSR_OFFSET             (0x00 + m_saiOffset)
#define SAI_TCR1_OFFSET             (0x04 + m_saiOffset)
#define SAI_TCR2_OFFSET             (0x08 + m_saiOffset)
#define SAI_TCR3_OFFSET             (0x0C + m_saiOffset)
#define SAI_TCR4_OFFSET             (0x10 + m_saiOffset)
#define SAI_TCR5_OFFSET             (0x14 + m_saiOffset)
#define SAI_TXn_OFFSET(n)           (0x20 + 0x4 * n)
#define SAI_TFRn_OFFSET(n)          (0x40 + 0x4 * n)
#define SAI_TMR_OFFSET               0x60

#define SAI_RCSR_OFFSET             (0x80 + m_saiOffset)
#define SAI_RCR1_OFFSET             (0x84 + m_saiOffset)
#define SAI_RCR2_OFFSET             (0x88 + m_saiOffset)
#define SAI_RCR3_OFFSET             (0x8C + m_saiOffset)
#define SAI_RCR4_OFFSET             (0x90 + m_saiOffset)
#define SAI_RCR5_OFFSET             (0x94 + m_saiOffset)
#define SAI_RXn_OFFSET(n)           (0xA0 + 0x4 * n)
#define SAI_RFRn_OFFSET(n)          (0xC0 + 0x4 * n)
#define SAI_RMR_OFFSET               0xE0

#define SAI_V1_X_OFFSET              0x00
#define SAI_V2_X_OFFSET              0x08

//
// IMX8M: SAI Parameter Register (I2Sx_PARAM)
//
typedef union {
    ULONG AsUlong;
    struct {
        ULONG DataLines : 4;                    // bit 0:3
        ULONG Reserved0 : 4;                    // bit 4:7
        ULONG FifoSize : 4;                     // bit 8:11
        ULONG Reserved1 : 4;                    // bit 12:16
        ULONG FrameSize : 4;                    // bit 17:19
        ULONG Reserved2 : 12;                   // bit 20:31
    };
} SAI_PARAM_REGISTER, *PSAI_PARAM_REGISTER;

// IMX7D/8M: SAI Control Register (I2Sx_xCSR)
typedef union {
    ULONG AsUlong;
    struct {
        ULONG FifoRequestDMAEnable : 1;         // bit 0
        ULONG FifoWarningDMAEnable : 1;         // bit 1
        ULONG Reserved0 : 3;                    // bit 2:4
        ULONG Reserved1 : 3;                    // bit 5:7
        ULONG FifoRequestInterruptEnable : 1;   // bit 8
        ULONG FifoWarningInterruptEnable : 1;   // bit 9
        ULONG FifoErrorInterruptEnable : 1;     // bit 10
        ULONG SyncErrorInterruptEnable : 1;     // bit 11
        ULONG WordStartInterruptEnable : 1;     // bit 12
        ULONG Reserved2 : 3;                    // bit 13:15
        ULONG FifoRequestFlag : 1;              // bit 16
        ULONG FifoWarningFlag : 1;              // bit 17
        ULONG FifoErrorFlag : 1;                // bit 18
        ULONG SyncErrorFlag : 1;                // bit 19
        ULONG WordStartFlag : 1;                // bit 20
        ULONG Reserved3 : 3;                    // bit 21:23
        ULONG SoftwareReset : 1;                // bit 24
        ULONG FifoReset : 1;                    // bit 25
        ULONG Reserved4 : 2;                    // bit 26:27
        ULONG BitClockEnable : 1;               // bit 28
        ULONG DebugEnable : 1;                  // bit 29 optional
        ULONG StopEnable : 1;                   // bit 30
        ULONG Enable : 1;                       // bit 31
    };
} SAI_TCSR, * PSAI_TCSR,
  SAI_RCSR, * PSAI_RCSR;

#define SAI_CTRL_INTERRUPTMASK 0x00001F00
#define SAI_CTRL_ERRORFLAGS 0x001C0000
//
// IMX7D/8M: SAI Configuration 1 Register (I2Sx_xCR1)
//
typedef union {
    ULONG AsUlong;
    struct {
        ULONG FifoWatermark : 32;       // Size is peripheral specific
    };
} SAI_TCR1, * PSAI_TCR1,
  SAI_RCR1, * PSAI_RCR1;

//
// IMX7D/8M: SAI Configuration 2 Register (I2Sx_xCR2)
//
typedef union {
    ULONG AsUlong;
    struct {
        ULONG BitClockDivide : 8;       // bit 0:7
        ULONG Reserved0 : 16;           // bit 8:23
        ULONG BitClockDirection : 1;    // bit 24
        ULONG BitClockPolarity : 1;     // bit 25
        ULONG MasterClockSelect : 2;    // bit 26:27
        ULONG BitClockInput : 1;        // bit 28
        ULONG BitClockSwap : 1;         // bit 29
        ULONG SynchronousMode : 2;      // bit 30:31
    };
} SAI_TCR2, * PSAI_TCR2,
  SAI_RCR2, * PSAI_RCR2;

//
// IMX7D/8M: SAI Configuration 3 Register (I2Sx_xCR3)
//
typedef union {
    ULONG AsUlong;
    struct {
        ULONG WordFlagConfiguration : 5;    // bit 0:4
        ULONG Reserved0 : 11;               // bit 5:15
        ULONG ChannelEnable : 8;            // bit 16:23 - real size depends on SAI version
        ULONG ChannelFifoReset : 8;         // bit 24:31 - MX8M only
    };
} SAI_TCR3, * PSAI_TCR3,
  SAI_RCR3, * PSAI_RCR3;

//
// IMX7D/8M: SAI Configuration 4 Register (I2Sx_xCR4)
//
typedef union {
    ULONG AsUlong;
    struct {
        ULONG FrameSyncDirection : 1;  // bit 0
        ULONG FrameSyncPolarity : 1;   // bit 1
        ULONG OnDemandMode : 1;        // bit 2
        ULONG FrameSyncEarly : 1;      // bit 3
        ULONG MSBFirst : 1;            // bit 4
        ULONG ChannelMode : 1;         // bit 5 - optional, supports 8M on TX side only
        ULONG Reserved1 : 2;           // bit 6:7
        ULONG SyncWidth : 5;           // bit 8:12
        ULONG Reserved2 : 3;           // bit 13:15
        ULONG FrameSize : 5;           // bit 16:20
        ULONG Reserved3 : 3;           // bit 21:23
        // 8M specific
        ULONG FifoPackMode : 2;        // bit 24:25
        ULONG FifoCombineMode : 2;     // bit 26:27
        ULONG FifoContinueOnErr : 1;   // bit 28
        ULONG Reserved7 : 3;           // bit 29:31
    };
} SAI_TCR4, * PSAI_TCR4,
  SAI_RCR4, * PSAI_RCR4;

//
// IMX7D/8M: SAI Configuration 5 Register (I2Sx_xCR5)
//
typedef union {
    ULONG AsUlong;
    struct {
        ULONG Reserved0 : 8;        // bit 0:7
        ULONG FirstBitShifted : 5;  // bit 8:12
        ULONG Reserved1 : 3;        // bit 13:15
        ULONG Word0Width : 5;       // bit 16:20
        ULONG Reserved2 : 3;        // bit 21:23
        ULONG WordNWidth : 5;       // bit 24:28
        ULONG Reserved3 : 3;        // bit 29:31
    };
} SAI_TCR5, * PSAI_TCR5,
  SAI_RCR5, * PSAI_RCR5;

//
// IMX7D/8M: SAI Data Register (I2Sx_xDRn)
//
typedef union {
    ULONG AsUlong;
    struct {
        ULONG TransmitDataRegister : 32;    // bit 0:31
    };
} SAI_TDR, * PSAI_TDR,
  SAI_RDR, * PSAI_RDR;

//
// IMX7D/8M: SAI Transmit FIFO Register (I2Sx_TFRn)
//
typedef union {
    ULONG AsUlong;
    struct {
        // TODO
        ULONG ReadFifoPointer : 8;      // bit 0:7   chip specific
        ULONG Reserved0 : 8;            // bit 8:15
        ULONG WriteFifoPointer : 8;     // bit 15:23 chip specific
        ULONG Reserved1 : 7;            // bit 24:30
        ULONG WriteChannelPointer : 1;  // bit 31, MX8M only
    };
} SAI_TFR, * PSAI_TFR;

//
// IMX7D/8M: SAI Receive FIFO Register (I2Sx_RFRn)
//
typedef union {
    ULONG AsUlong;
    struct {
        ULONG ReadFifoPointer : 8;    // bit 0:5
        ULONG Reserved0 : 7;          // bit 8:14
        ULONG ReadChannelPointer : 1; // bit 15 optional
        ULONG WriteFifoPointer : 8;   // bit 16:23
        ULONG Reserved2 : 8;          // bit 24:31
    };
} SAI_RFR, * PSAI_RFR;

//
// IMX7D/8M: SAI Mask Register (I2Sx_xMR)
//
typedef union {
    ULONG AsUlong;
    struct {
        ULONG WordMask : 32;      // bit 0:31
    };
} SAI_TMR, * PSAI_TMR,
  SAI_RMR, * PSAI_RMR;
