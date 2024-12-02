/*
 * Copyright 2022 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once
#pragma warning(push)
#pragma warning(disable:4201) // nameless struct/union

/*
 *  CAN Peripheral Access Layer
 */

    /* CAN - Register Layout Typedef */
typedef struct {
    UINT32 MCR;                             /* Module Configuration register, offset: 0x0 */
    UINT32 CTRL1;                           /* Control 1 register, offset: 0x4 */
    UINT32 TIMER;                           /* Free Running Timer, offset: 0x8 */
    UINT8  RESERVED_0[4];
    UINT32 RXMGMASK;                        /* Rx Mailboxes Global Mask register, offset: 0x10 */
    UINT32 RX14MASK;                        /* Rx 14 Mask register, offset: 0x14 */
    UINT32 RX15MASK;                        /* Rx 15 Mask register, offset: 0x18 */
    UINT32 ECR;                             /* Error Counter, offset: 0x1C */
    UINT32 ESR1;                            /* Error and Status 1 register, offset: 0x20 */
    UINT32 IMASK2;                          /* Interrupt Masks 2 register, offset: 0x24 */
    UINT32 IMASK1;                          /* Interrupt Masks 1 register, offset: 0x28 */
    UINT32 IFLAG2;                          /* Interrupt Flags 2 register, offset: 0x2C */
    UINT32 IFLAG1;                          /* Interrupt Flags 1 register, offset: 0x30 */
    UINT32 CTRL2;                           /* Control 2 register, offset: 0x34 */
    UINT32 ESR2;                            /* Error and Status 2 register, offset: 0x38 */
    UINT8  RESERVED_1[8];
    UINT32 CRCR;                            /* CRC register, offset: 0x44 */
    UINT32 RXFGMASK;                        /* Rx FIFO Global Mask register, offset: 0x48 */
    UINT32 RXFIR;                           /* Rx FIFO Information register, offset: 0x4C */
    UINT32 CBT;                             /* CAN Bit Timing register, offset: 0x50 */
    UINT8  RESERVED_2[44];
    union {                                 /* offset: 0x80 */
        struct {                                /* offset: 0x80, array step: 0x10 */
            UINT32 CS;                              /* Message Buffer 0 CS Register ... Message Buffer 63 CS Register, array offset: 0x80, array step: 0x10 */
            UINT32 ID;                              /* Message Buffer 0 ID Register ... Message Buffer 63 ID Register, array offset: 0x84, array step: 0x10 */
            UINT32 WORD[2];                         /* Message Buffer 0 WORD_8B Register ... Message Buffer 63 WORD_8B Register, array offset: 0x88, array step: index*0x10, index2*0x4 */
        } MB_8B[64];
        struct {                                /* offset: 0x80, array step: 0x18 */
            UINT32 CS;                              /* Message Buffer 0 CS Register ... Message Buffer 41 CS Register, array offset: 0x80, array step: 0x18 */
            UINT32 ID;                              /* Message Buffer 0 ID Register ... Message Buffer 41 ID Register, array offset: 0x84, array step: 0x18 */
            UINT32 WORD[4];                         /* Message Buffer 0 WORD_16B Register ... Message Buffer 41 WORD_16B Register, array offset: 0x88, array step: index*0x18, index2*0x4 */
        } MB_16B[42];
        struct {                                /* offset: 0x80, array step: 0x28 */
            UINT32 CS;                              /* Message Buffer 0 CS Register ... Message Buffer 23 CS Register, array offset: 0x80, array step: 0x28 */
            UINT32 ID;                              /* Message Buffer 0 ID Register ... Message Buffer 23 ID Register, array offset: 0x84, array step: 0x28 */
            UINT32 WORD[8];                         /* Message Buffer 0 WORD_32B Register ... Message Buffer 23 WORD_32B Register, array offset: 0x88, array step: index*0x28, index2*0x4 */
        } MB_32B[24];
        struct {                                /* offset: 0x80, array step: 0x48 */
            UINT32 CS;                              /* Message Buffer 0 CS Register ... Message Buffer 13 CS Register, array offset: 0x80, array step: 0x48 */
            UINT32 ID;                              /* Message Buffer 0 ID Register ... Message Buffer 13 ID Register, array offset: 0x84, array step: 0x48 */
            UINT32 WORD[16];                        /* Message Buffer 0 WORD_64B Register ... Message Buffer 13 WORD_64B Register, array offset: 0x88, array step: index*0x48, index2*0x4 */
        } MB_64B[14];
        struct {                                /* offset: 0x80, array step: 0x10 */
            UINT32 CS;                              /* Message Buffer 0 CS Register ... Message Buffer 63 CS Register, array offset: 0x80, array step: 0x10 */
            UINT32 ID;                              /* Message Buffer 0 ID Register ... Message Buffer 63 ID Register, array offset: 0x84, array step: 0x10 */
            UINT32 WORD0;                           /* Message Buffer 0 WORD0 Register ... Message Buffer 63 WORD0 Register, array offset: 0x88, array step: 0x10 */
            UINT32 WORD1;                           /* Message Buffer 0 WORD1 Register ... Message Buffer 63 WORD1 Register, array offset: 0x8C, array step: 0x10 */
        } MB[64];
    };
    UINT8  RESERVED_3[1024];
    UINT32 RXIMR[64];                       /* Rx Individual Mask registers, array offset: 0x880, array step: 0x4 */
    UINT8  RESERVED_4[352];
    UINT32 MECR;                            /* Memory Error Control register, offset: 0xAE0 */
    UINT32 ERRIAR;                          /* Error Injection Address register, offset: 0xAE4 */
    UINT32 ERRIDPR;                         /* Error Injection Data Pattern register, offset: 0xAE8 */
    UINT32 ERRIPPR;                         /* Error Injection Parity Pattern register, offset: 0xAEC */
    UINT32 RERRAR;                          /* Error Report Address register, offset: 0xAF0 */
    UINT32 RERRDR;                          /* Error Report Data register, offset: 0xAF4 */
    UINT32 RERRSYNR;                        /* Error Report Syndrome register, offset: 0xAF8 */
    UINT32 ERRSR;                           /* Error Status register, offset: 0xAFC */
    UINT8  RESERVED_5[256];
    UINT32 FDCTRL;                          /* CAN FD Control register, offset: 0xC00 */
    UINT32 FDCBT;                           /* CAN FD Bit Timing register, offset: 0xC04 */
    UINT32 FDCRC;                           /* CAN FD CRC register, offset: 0xC08 */
} IMXCAN_REGISTERS;

/*
 *  CAN Register Masks
 */

/* MCR - Module Configuration register */

#define CAN_MCR_MAXMB_MASK                       (0x7FU)
#define CAN_MCR_MAXMB_SHIFT                      (0U)
    /* MAXMB - Number Of The Last Message Buffer */
#define CAN_MCR_MAXMB(x)                         (((UINT32)(((UINT32)(x)) << CAN_MCR_MAXMB_SHIFT)) & CAN_MCR_MAXMB_MASK)

#define CAN_MCR_IDAM_MASK                        (0x300U)
#define CAN_MCR_IDAM_SHIFT                       (8U)
    /*  IDAM - ID Acceptance Mode
     *  0b00..Format A: One full ID (standard and extended) per ID filter table element.
     *  0b01..Format B: Two full standard IDs or two partial 14-bit (standard and extended) IDs per ID filter table element.
     *  0b10..Format C: Four partial 8-bit standard IDs per ID filter table element.
     *  0b11..Format D: All frames rejected.
     */
#define CAN_MCR_IDAM(x)                          (((UINT32)(((UINT32)(x)) << CAN_MCR_IDAM_SHIFT)) & CAN_MCR_IDAM_MASK)

#define CAN_MCR_FDEN_MASK                        (0x800U)
#define CAN_MCR_FDEN_SHIFT                       (11U)
    /*  FDEN - CAN FD operation enable
     *  0b1..CAN FD is enabled. FlexCAN is able to receive and transmit messages in both CAN FD and CAN 2.0 formats.
     *  0b0..CAN FD is disabled. FlexCAN is able to receive and transmit messages in CAN 2.0 format.
     */
#define CAN_MCR_FDEN(x)                          (((UINT32)(((UINT32)(x)) << CAN_MCR_FDEN_SHIFT)) & CAN_MCR_FDEN_MASK)

#define CAN_MCR_AEN_MASK                         (0x1000U)
#define CAN_MCR_AEN_SHIFT                        (12U)
    /*  AEN - Abort Enable
     *  0b0..Abort disabled.
     *  0b1..Abort enabled.
     */
#define CAN_MCR_AEN(x)                           (((UINT32)(((UINT32)(x)) << CAN_MCR_AEN_SHIFT)) & CAN_MCR_AEN_MASK)

#define CAN_MCR_LPRIOEN_MASK                     (0x2000U)
#define CAN_MCR_LPRIOEN_SHIFT                    (13U)
    /*  LPRIOEN - Local Priority Enable
     *  0b0..Local Priority disabled.
     *  0b1..Local Priority enabled.
     */
#define CAN_MCR_LPRIOEN(x)                       (((UINT32)(((UINT32)(x)) << CAN_MCR_LPRIOEN_SHIFT)) & CAN_MCR_LPRIOEN_MASK)

#define CAN_MCR_DMA_MASK                         (0x8000U)
#define CAN_MCR_DMA_SHIFT                        (15U)
    /*  DMA - DMA Enable
     *  0b0..DMA feature for RX FIFO disabled.
     *  0b1..DMA feature for RX FIFO enabled.
     */
#define CAN_MCR_DMA(x)                           (((UINT32)(((UINT32)(x)) << CAN_MCR_DMA_SHIFT)) & CAN_MCR_DMA_MASK)

#define CAN_MCR_IRMQ_MASK                        (0x10000U)
#define CAN_MCR_IRMQ_SHIFT                       (16U)
    /*  IRMQ - Individual Rx Masking And Queue Enable
     *  0b0..Individual Rx masking and queue feature are disabled. For backward compatibility with legacy
     *       applications, the reading of C/S word locks the MB even if it is EMPTY.
     *  0b1..Individual Rx masking and queue feature are enabled.
     */
#define CAN_MCR_IRMQ(x)                          (((UINT32)(((UINT32)(x)) << CAN_MCR_IRMQ_SHIFT)) & CAN_MCR_IRMQ_MASK)

#define CAN_MCR_SRXDIS_MASK                      (0x20000U)
#define CAN_MCR_SRXDIS_SHIFT                     (17U)
    /*  SRXDIS - Self Reception Disable
     *  0b0..Self-reception enabled.
     *  0b1..Self-reception disabled.
     */
#define CAN_MCR_SRXDIS(x)                        (((UINT32)(((UINT32)(x)) << CAN_MCR_SRXDIS_SHIFT)) & CAN_MCR_SRXDIS_MASK)

#define CAN_MCR_DOZE_MASK                        (0x40000U)
#define CAN_MCR_DOZE_SHIFT                       (18U)
    /*  DOZE - Doze Mode Enable
     *  0b0..FlexCAN is not enabled to enter low-power mode when Doze mode is requested.
     *  0b1..FlexCAN is enabled to enter low-power mode when Doze mode is requested.
     */
#define CAN_MCR_DOZE(x)                          (((UINT32)(((UINT32)(x)) << CAN_MCR_DOZE_SHIFT)) & CAN_MCR_DOZE_MASK)

#define CAN_MCR_WAKSRC_MASK                      (0x80000U)
#define CAN_MCR_WAKSRC_SHIFT                     (19U)
    /*  WAKSRC - Wake Up Source
     *  0b0..FlexCAN uses the unfiltered Rx input to detect recessive to dominant edges on the CAN bus.
     *  0b1..FlexCAN uses the filtered Rx input to detect recessive to dominant edges on the CAN bus.
     */
#define CAN_MCR_WAKSRC(x)                        (((UINT32)(((UINT32)(x)) << CAN_MCR_WAKSRC_SHIFT)) & CAN_MCR_WAKSRC_MASK)

#define CAN_MCR_LPMACK_MASK                      (0x100000U)
#define CAN_MCR_LPMACK_SHIFT                     (20U)
    /*  LPMACK - Low-Power Mode Acknowledge
     *  0b0..FlexCAN is not in a low-power mode.
     *  0b1..FlexCAN is in a low-power mode.
     */
#define CAN_MCR_LPMACK(x)                        (((UINT32)(((UINT32)(x)) << CAN_MCR_LPMACK_SHIFT)) & CAN_MCR_LPMACK_MASK)

#define CAN_MCR_WRNEN_MASK                       (0x200000U)
#define CAN_MCR_WRNEN_SHIFT                      (21U)
    /*  WRNEN - Warning Interrupt Enable
     *  0b0..TWRNINT and RWRNINT bits are zero, independent of the values in the error counters.
     *  0b1..TWRNINT and RWRNINT bits are set when the respective error counter transitions from less than 96 to greater than or equal to 96.
     */
#define CAN_MCR_WRNEN(x)                         (((UINT32)(((UINT32)(x)) << CAN_MCR_WRNEN_SHIFT)) & CAN_MCR_WRNEN_MASK)

#define CAN_MCR_SLFWAK_MASK                      (0x400000U)
#define CAN_MCR_SLFWAK_SHIFT                     (22U)
    /*  SLFWAK - Self Wake Up
     *  0b0..FlexCAN Self Wake Up feature is disabled.
     *  0b1..FlexCAN Self Wake Up feature is enabled.
     */
#define CAN_MCR_SLFWAK(x)                        (((UINT32)(((UINT32)(x)) << CAN_MCR_SLFWAK_SHIFT)) & CAN_MCR_SLFWAK_MASK)

#define CAN_MCR_SUPV_MASK                        (0x800000U)
#define CAN_MCR_SUPV_SHIFT                       (23U)
    /*  SUPV - Supervisor Mode
     *  0b0..FlexCAN is in User mode. Affected registers allow both Supervisor and Unrestricted accesses.
     *  0b1..FlexCAN is in Supervisor mode. Affected registers allow only Supervisor access. Unrestricted access
     *       behaves as though the access was done to an unimplemented register location.
     */
#define CAN_MCR_SUPV(x)                          (((UINT32)(((UINT32)(x)) << CAN_MCR_SUPV_SHIFT)) & CAN_MCR_SUPV_MASK)

#define CAN_MCR_FRZACK_MASK                      (0x1000000U)
#define CAN_MCR_FRZACK_SHIFT                     (24U)
    /*  FRZACK - Freeze Mode Acknowledge
     *  0b0..FlexCAN not in Freeze mode, prescaler running.
     *  0b1..FlexCAN in Freeze mode, prescaler stopped.
     */
#define CAN_MCR_FRZACK(x)                        (((UINT32)(((UINT32)(x)) << CAN_MCR_FRZACK_SHIFT)) & CAN_MCR_FRZACK_MASK)

#define CAN_MCR_SOFTRST_MASK                     (0x2000000U)
#define CAN_MCR_SOFTRST_SHIFT                    (25U)
    /*  SOFTRST - Soft Reset
     *  0b0..No reset request.
     *  0b1..Resets the registers affected by soft reset.
     */
#define CAN_MCR_SOFTRST(x)                       (((UINT32)(((UINT32)(x)) << CAN_MCR_SOFTRST_SHIFT)) & CAN_MCR_SOFTRST_MASK)

#define CAN_MCR_WAKMSK_MASK                      (0x4000000U)
#define CAN_MCR_WAKMSK_SHIFT                     (26U)
    /*  WAKMSK - Wake Up Interrupt Mask
     *  0b0..Wake Up interrupt is disabled.
     *  0b1..Wake Up interrupt is enabled.
     */
#define CAN_MCR_WAKMSK(x)                        (((UINT32)(((UINT32)(x)) << CAN_MCR_WAKMSK_SHIFT)) & CAN_MCR_WAKMSK_MASK)

#define CAN_MCR_NOTRDY_MASK                      (0x8000000U)
#define CAN_MCR_NOTRDY_SHIFT                     (27U)
    /*  NOTRDY - FlexCAN Not Ready
     *  0b0..FlexCAN module is either in Normal mode, Listen-Only mode, or Loop-Back mode.
     *  0b1..FlexCAN module is either in Disable mode, Doze mode, Stop mode, or Freeze mode.
     */
#define CAN_MCR_NOTRDY(x)                        (((UINT32)(((UINT32)(x)) << CAN_MCR_NOTRDY_SHIFT)) & CAN_MCR_NOTRDY_MASK)

#define CAN_MCR_HALT_MASK                        (0x10000000U)
#define CAN_MCR_HALT_SHIFT                       (28U)
    /*  HALT - Halt FlexCAN
     *  0b0..No Freeze mode request.
     *  0b1..Enters Freeze mode if the FRZ bit is asserted.
     */
#define CAN_MCR_HALT(x)                          (((UINT32)(((UINT32)(x)) << CAN_MCR_HALT_SHIFT)) & CAN_MCR_HALT_MASK)

#define CAN_MCR_RFEN_MASK                        (0x20000000U)
#define CAN_MCR_RFEN_SHIFT                       (29U)
    /*  RFEN - Rx FIFO Enable
     *  0b0..Rx FIFO not enabled.
     *  0b1..Rx FIFO enabled.
     */
#define CAN_MCR_RFEN(x)                          (((UINT32)(((UINT32)(x)) << CAN_MCR_RFEN_SHIFT)) & CAN_MCR_RFEN_MASK)

#define CAN_MCR_FRZ_MASK                         (0x40000000U)
#define CAN_MCR_FRZ_SHIFT                        (30U)
    /*  FRZ - Freeze Enable
     *  0b0..Not enabled to enter Freeze mode.
     *  0b1..Enabled to enter Freeze mode.
     */
#define CAN_MCR_FRZ(x)                           (((UINT32)(((UINT32)(x)) << CAN_MCR_FRZ_SHIFT)) & CAN_MCR_FRZ_MASK)

#define CAN_MCR_MDIS_MASK                        (0x80000000U)
#define CAN_MCR_MDIS_SHIFT                       (31U)
     /*  MDIS - Module Disable
      *  0b0..Enable the FlexCAN module.
      *  0b1..Disable the FlexCAN module.
      */
#define CAN_MCR_MDIS(x)                          (((UINT32)(((UINT32)(x)) << CAN_MCR_MDIS_SHIFT)) & CAN_MCR_MDIS_MASK)

/* CTRL1 - Control 1 register */

#define CAN_CTRL1_PROPSEG_MASK                   (0x7U)
#define CAN_CTRL1_PROPSEG_SHIFT                  (0U)
    /* PROPSEG - Propagation Segment */
#define CAN_CTRL1_PROPSEG(x)                     (((UINT32)(((UINT32)(x)) << CAN_CTRL1_PROPSEG_SHIFT)) & CAN_CTRL1_PROPSEG_MASK)

#define CAN_CTRL1_LOM_MASK                       (0x8U)
#define CAN_CTRL1_LOM_SHIFT                      (3U)
    /*  LOM - Listen-Only Mode
     *  0b0..Listen-Only mode is deactivated.
     *  0b1..FlexCAN module operates in Listen-Only mode.
     */
#define CAN_CTRL1_LOM(x)                         (((UINT32)(((UINT32)(x)) << CAN_CTRL1_LOM_SHIFT)) & CAN_CTRL1_LOM_MASK)

#define CAN_CTRL1_LBUF_MASK                      (0x10U)
#define CAN_CTRL1_LBUF_SHIFT                     (4U)
    /*  LBUF - Lowest Buffer Transmitted First
     *  0b0..Buffer with highest priority is transmitted first.
     *  0b1..Lowest number buffer is transmitted first.
     */
#define CAN_CTRL1_LBUF(x)                        (((UINT32)(((UINT32)(x)) << CAN_CTRL1_LBUF_SHIFT)) & CAN_CTRL1_LBUF_MASK)

#define CAN_CTRL1_TSYN_MASK                      (0x20U)
#define CAN_CTRL1_TSYN_SHIFT                     (5U)
   /*  TSYN - Timer Sync
    *  0b0..Timer sync feature disabled
    *  0b1..Timer sync feature enabled
    */
#define CAN_CTRL1_TSYN(x)                        (((UINT32)(((UINT32)(x)) << CAN_CTRL1_TSYN_SHIFT)) & CAN_CTRL1_TSYN_MASK)

#define CAN_CTRL1_BOFFREC_MASK                   (0x40U)
#define CAN_CTRL1_BOFFREC_SHIFT                  (6U)
    /*  BOFFREC - Bus Off Recovery
     *  0b0..Automatic recovering from Bus Off state enabled.
     *  0b1..Automatic recovering from Bus Off state disabled.
     */
#define CAN_CTRL1_BOFFREC(x)                     (((UINT32)(((UINT32)(x)) << CAN_CTRL1_BOFFREC_SHIFT)) & CAN_CTRL1_BOFFREC_MASK)

#define CAN_CTRL1_SMP_MASK                       (0x80U)
#define CAN_CTRL1_SMP_SHIFT                      (7U)
    /*  SMP - CAN Bit Sampling
     *  0b0..Just one sample is used to determine the bit value.
     *  0b1..Three samples are used to determine the value of the received bit: the regular one (sample point) and two
     *       preceding samples; a majority rule is used.
     */
#define CAN_CTRL1_SMP(x)                         (((UINT32)(((UINT32)(x)) << CAN_CTRL1_SMP_SHIFT)) & CAN_CTRL1_SMP_MASK)

#define CAN_CTRL1_RWRNMSK_MASK                   (0x400U)
#define CAN_CTRL1_RWRNMSK_SHIFT                  (10U)
    /*  RWRNMSK - Rx Warning Interrupt Mask
     *  0b0..Rx Warning interrupt disabled.
     *  0b1..Rx Warning interrupt enabled.
     */
#define CAN_CTRL1_RWRNMSK(x)                     (((UINT32)(((UINT32)(x)) << CAN_CTRL1_RWRNMSK_SHIFT)) & CAN_CTRL1_RWRNMSK_MASK)

#define CAN_CTRL1_TWRNMSK_MASK                   (0x800U)
#define CAN_CTRL1_TWRNMSK_SHIFT                  (11U)
    /*  TWRNMSK - Tx Warning Interrupt Mask
     *  0b0..Tx Warning interrupt disabled.
     *  0b1..Tx Warning interrupt enabled.
     */
#define CAN_CTRL1_TWRNMSK(x)                     (((UINT32)(((UINT32)(x)) << CAN_CTRL1_TWRNMSK_SHIFT)) & CAN_CTRL1_TWRNMSK_MASK)

#define CAN_CTRL1_LPB_MASK                       (0x1000U)
#define CAN_CTRL1_LPB_SHIFT                      (12U)
    /*  LPB - Loop Back Mode
     *  0b0..Loop Back disabled.
     *  0b1..Loop Back enabled.
     */
#define CAN_CTRL1_LPB(x)                         (((UINT32)(((UINT32)(x)) << CAN_CTRL1_LPB_SHIFT)) & CAN_CTRL1_LPB_MASK)

#define CAN_CTRL1_CLKSRC_MASK                    (0x2000U)
#define CAN_CTRL1_CLKSRC_SHIFT                   (13U)
    /*  CLKSRC - CAN Engine Clock Source
     *  0b0..The CAN engine clock source is the oscillator clock. Under this condition, the oscillator clock frequency must be lower than the bus clock.
     *  0b1..The CAN engine clock source is the peripheral clock.
     */
#define CAN_CTRL1_CLKSRC(x)                      (((UINT32)(((UINT32)(x)) << CAN_CTRL1_CLKSRC_SHIFT)) & CAN_CTRL1_CLKSRC_MASK)

#define CAN_CTRL1_ERRMSK_MASK                    (0x4000U)
#define CAN_CTRL1_ERRMSK_SHIFT                   (14U)
    /*  ERRMSK - Error Interrupt Mask
     *  0b0..Error interrupt disabled.
     *  0b1..Error interrupt enabled.
     */
#define CAN_CTRL1_ERRMSK(x)                      (((UINT32)(((UINT32)(x)) << CAN_CTRL1_ERRMSK_SHIFT)) & CAN_CTRL1_ERRMSK_MASK)

#define CAN_CTRL1_BOFFMSK_MASK                   (0x8000U)
#define CAN_CTRL1_BOFFMSK_SHIFT                  (15U)
    /*  BOFFMSK - Bus Off Interrupt Mask
     *  0b0..Bus Off interrupt disabled.
     *  0b1..Bus Off interrupt enabled.
     */
#define CAN_CTRL1_BOFFMSK(x)                     (((UINT32)(((UINT32)(x)) << CAN_CTRL1_BOFFMSK_SHIFT)) & CAN_CTRL1_BOFFMSK_MASK)

#define CAN_CTRL1_PSEG2_MASK                     (0x70000U)
#define CAN_CTRL1_PSEG2_SHIFT                    (16U)
    /*  PSEG2 - Phase Segment 2 */
#define CAN_CTRL1_PSEG2(x)                       (((UINT32)(((UINT32)(x)) << CAN_CTRL1_PSEG2_SHIFT)) & CAN_CTRL1_PSEG2_MASK)

#define CAN_CTRL1_PSEG1_MASK                     (0x380000U)
#define CAN_CTRL1_PSEG1_SHIFT                    (19U)
    /*  PSEG1 - Phase Segment 1 */
#define CAN_CTRL1_PSEG1(x)                       (((UINT32)(((UINT32)(x)) << CAN_CTRL1_PSEG1_SHIFT)) & CAN_CTRL1_PSEG1_MASK)

#define CAN_CTRL1_RJW_MASK                       (0xC00000U)
#define CAN_CTRL1_RJW_SHIFT                      (22U)
    /*  RJW - Resync Jump Width */
#define CAN_CTRL1_RJW(x)                         (((UINT32)(((UINT32)(x)) << CAN_CTRL1_RJW_SHIFT)) & CAN_CTRL1_RJW_MASK)

#define CAN_CTRL1_PRESDIV_MASK                   (0xFF000000U)
#define CAN_CTRL1_PRESDIV_SHIFT                  (24U)
    /*  PRESDIV - Prescaler Division Factor */
#define CAN_CTRL1_PRESDIV(x)                     (((UINT32)(((UINT32)(x)) << CAN_CTRL1_PRESDIV_SHIFT)) & CAN_CTRL1_PRESDIV_MASK)

/* TIMER - Free Running Timer */

#define CAN_TIMER_TIMER_MASK                     (0xFFFFU)
#define CAN_TIMER_TIMER_SHIFT                    (0U)
    /* TIMER - Timer Value */
#define CAN_TIMER_TIMER(x)                       (((UINT32)(((UINT32)(x)) << CAN_TIMER_TIMER_SHIFT)) & CAN_TIMER_TIMER_MASK)

/* RXMGMASK - Rx Mailboxes Global Mask register */

#define CAN_RXMGMASK_MG_MASK                     (0xFFFFFFFFU)
#define CAN_RXMGMASK_MG_SHIFT                    (0U)
    /* MG - Rx Mailboxes Global Mask Bits */
#define CAN_RXMGMASK_MG(x)                       (((UINT32)(((UINT32)(x)) << CAN_RXMGMASK_MG_SHIFT)) & CAN_RXMGMASK_MG_MASK)

/* RX14MASK - Rx 14 Mask register */

#define CAN_RX14MASK_RX14M_MASK                  (0xFFFFFFFFU)
#define CAN_RX14MASK_RX14M_SHIFT                 (0U)
    /* RX14M - Rx Buffer 14 Mask Bits */
#define CAN_RX14MASK_RX14M(x)                    (((UINT32)(((UINT32)(x)) << CAN_RX14MASK_RX14M_SHIFT)) & CAN_RX14MASK_RX14M_MASK)

 /* RX15MASK - Rx 15 Mask register */

#define CAN_RX15MASK_RX15M_MASK                  (0xFFFFFFFFU)
#define CAN_RX15MASK_RX15M_SHIFT                 (0U)
    /* RX15M - Rx Buffer 15 Mask Bits */
#define CAN_RX15MASK_RX15M(x)                    (((UINT32)(((UINT32)(x)) << CAN_RX15MASK_RX15M_SHIFT)) & CAN_RX15MASK_RX15M_MASK)

/* ECR - Error Counter */

#define CAN_ECR_TXERRCNT_MASK                    (0xFFU)
#define CAN_ECR_TXERRCNT_SHIFT                   (0U)
    /* TXERRCNT - Transmit Error Counter */
#define CAN_ECR_TXERRCNT(x)                      (((UINT32)(((UINT32)(x)) << CAN_ECR_TXERRCNT_SHIFT)) & CAN_ECR_TXERRCNT_MASK)

#define CAN_ECR_RXERRCNT_MASK                    (0xFF00U)
#define CAN_ECR_RXERRCNT_SHIFT                   (8U)
    /* RXERRCNT - Receive Error Counter */
#define CAN_ECR_RXERRCNT(x)                      (((UINT32)(((UINT32)(x)) << CAN_ECR_RXERRCNT_SHIFT)) & CAN_ECR_RXERRCNT_MASK)

#define CAN_ECR_TXERRCNT_FAST_MASK               (0xFF0000U)
#define CAN_ECR_TXERRCNT_FAST_SHIFT              (16U)
    /* TXERRCNT_FAST - Transmit Error Counter for fast bits */
#define CAN_ECR_TXERRCNT_FAST(x)                 (((UINT32)(((UINT32)(x)) << CAN_ECR_TXERRCNT_FAST_SHIFT)) & CAN_ECR_TXERRCNT_FAST_MASK)

#define CAN_ECR_RXERRCNT_FAST_MASK               (0xFF000000U)
#define CAN_ECR_RXERRCNT_FAST_SHIFT              (24U)
    /* RXERRCNT_FAST - Receive Error Counter for fast bits */
#define CAN_ECR_RXERRCNT_FAST(x)                 (((UINT32)(((UINT32)(x)) << CAN_ECR_RXERRCNT_FAST_SHIFT)) & CAN_ECR_RXERRCNT_FAST_MASK)

/* ESR1 - Error and Status 1 register */

#define CAN_ESR1_WAKINT_MASK                     (0x1U)
#define CAN_ESR1_WAKINT_SHIFT                    (0U)
    /*  WAKINT - Wake-Up Interrupt
     *  0b0..No such occurrence.
     *  0b1..Indicates a recessive to dominant transition was received on the CAN bus.
     */
#define CAN_ESR1_WAKINT(x)                       (((UINT32)(((UINT32)(x)) << CAN_ESR1_WAKINT_SHIFT)) & CAN_ESR1_WAKINT_MASK)

#define CAN_ESR1_ERRINT_MASK                     (0x2U)
#define CAN_ESR1_ERRINT_SHIFT                    (1U)
    /*  ERRINT - Error Interrupt
     *  0b0..No such occurrence.
     *  0b1..Indicates setting of any error bit in the Error and Status register.
     */
#define CAN_ESR1_ERRINT(x)                       (((UINT32)(((UINT32)(x)) << CAN_ESR1_ERRINT_SHIFT)) & CAN_ESR1_ERRINT_MASK)

#define CAN_ESR1_BOFFINT_MASK                    (0x4U)
#define CAN_ESR1_BOFFINT_SHIFT                   (2U)
    /*  BOFFINT - Bus Off Interrupt
     *  0b0..No such occurrence.
     *  0b1..FlexCAN module entered Bus Off state.
     */
#define CAN_ESR1_BOFFINT(x)                      (((UINT32)(((UINT32)(x)) << CAN_ESR1_BOFFINT_SHIFT)) & CAN_ESR1_BOFFINT_MASK)

#define CAN_ESR1_RX_MASK                         (0x8U)
#define CAN_ESR1_RX_SHIFT                        (3U)
    /*  RX - FlexCAN In Reception
     *  0b0..FlexCAN is not receiving a message.
     *  0b1..FlexCAN is receiving a message.
     */
#define CAN_ESR1_RX(x)                           (((UINT32)(((UINT32)(x)) << CAN_ESR1_RX_SHIFT)) & CAN_ESR1_RX_MASK)

#define CAN_ESR1_FLTCONF_MASK                    (0x30U)
#define CAN_ESR1_FLTCONF_SHIFT                   (4U)
    /*  FLTCONF - Fault Confinement State
     *  0b00..Error Active
     *  0b01..Error Passive
     *  0b1x..Bus Off
     */
#define CAN_ESR1_FLTCONF(x)                      (((UINT32)(((UINT32)(x)) << CAN_ESR1_FLTCONF_SHIFT)) & CAN_ESR1_FLTCONF_MASK)

#define CAN_ESR1_TX_MASK                         (0x40U)
#define CAN_ESR1_TX_SHIFT                        (6U)
    /*  TX - FlexCAN In Transmission
     *  0b0..FlexCAN is not transmitting a message.
     *  0b1..FlexCAN is transmitting a message.
     */
#define CAN_ESR1_TX(x)                           (((UINT32)(((UINT32)(x)) << CAN_ESR1_TX_SHIFT)) & CAN_ESR1_TX_MASK)

#define CAN_ESR1_IDLE_MASK                       (0x80U)
#define CAN_ESR1_IDLE_SHIFT                      (7U)
    /*  IDLE - IDLE
     *  0b0..No such occurrence.
     *  0b1..CAN bus is now IDLE.
     */
#define CAN_ESR1_IDLE(x)                         (((UINT32)(((UINT32)(x)) << CAN_ESR1_IDLE_SHIFT)) & CAN_ESR1_IDLE_MASK)

#define CAN_ESR1_RXWRN_MASK                      (0x100U)
#define CAN_ESR1_RXWRN_SHIFT                     (8U)
    /*  RXWRN - Rx Error Warning
     *  0b0..No such occurrence.
     *  0b1..RXERRCNT is greater than or equal to 96.
     */
#define CAN_ESR1_RXWRN(x)                        (((UINT32)(((UINT32)(x)) << CAN_ESR1_RXWRN_SHIFT)) & CAN_ESR1_RXWRN_MASK)

#define CAN_ESR1_TXWRN_MASK                      (0x200U)
#define CAN_ESR1_TXWRN_SHIFT                     (9U)
    /*  TXWRN - TX Error Warning
     *  0b0..No such occurrence.
     *  0b1..TXERRCNT is greater than or equal to 96.
     */
#define CAN_ESR1_TXWRN(x)                        (((UINT32)(((UINT32)(x)) << CAN_ESR1_TXWRN_SHIFT)) & CAN_ESR1_TXWRN_MASK)

#define CAN_ESR1_STFERR_MASK                     (0x400U)
#define CAN_ESR1_STFERR_SHIFT                    (10U)
    /*  STFERR - Stuffing Error
     *  0b0..No such occurrence.
     *  0b1..A stuffing error occurred since last read of this register.
     */
#define CAN_ESR1_STFERR(x)                       (((UINT32)(((UINT32)(x)) << CAN_ESR1_STFERR_SHIFT)) & CAN_ESR1_STFERR_MASK)

#define CAN_ESR1_FRMERR_MASK                     (0x800U)
#define CAN_ESR1_FRMERR_SHIFT                    (11U)
    /*  FRMERR - Form Error
     *  0b0..No such occurrence.
     *  0b1..A Form Error occurred since last read of this register.
     */
#define CAN_ESR1_FRMERR(x)                       (((UINT32)(((UINT32)(x)) << CAN_ESR1_FRMERR_SHIFT)) & CAN_ESR1_FRMERR_MASK)

#define CAN_ESR1_CRCERR_MASK                     (0x1000U)
#define CAN_ESR1_CRCERR_SHIFT                    (12U)
    /*  CRCERR - Cyclic Redundancy Check Error
     *  0b0..No such occurrence.
     *  0b1..A CRC error occurred since last read of this register.
     */
#define CAN_ESR1_CRCERR(x)                       (((UINT32)(((UINT32)(x)) << CAN_ESR1_CRCERR_SHIFT)) & CAN_ESR1_CRCERR_MASK)

#define CAN_ESR1_ACKERR_MASK                     (0x2000U)
#define CAN_ESR1_ACKERR_SHIFT                    (13U)
    /*  ACKERR - Acknowledge Error
     *  0b0..No such occurrence.
     *  0b1..An ACK error occurred since last read of this register.
     */
#define CAN_ESR1_ACKERR(x)                       (((UINT32)(((UINT32)(x)) << CAN_ESR1_ACKERR_SHIFT)) & CAN_ESR1_ACKERR_MASK)

#define CAN_ESR1_BIT0ERR_MASK                    (0x4000U)
#define CAN_ESR1_BIT0ERR_SHIFT                   (14U)
    /*  BIT0ERR - Bit0 Error
     *  0b0..No such occurrence.
     *  0b1..At least one bit sent as dominant is received as recessive.
     */
#define CAN_ESR1_BIT0ERR(x)                      (((UINT32)(((UINT32)(x)) << CAN_ESR1_BIT0ERR_SHIFT)) & CAN_ESR1_BIT0ERR_MASK)

#define CAN_ESR1_BIT1ERR_MASK                    (0x8000U)
#define CAN_ESR1_BIT1ERR_SHIFT                   (15U)
    /*  BIT1ERR - Bit1 Error
     *  0b0..No such occurrence.
     *  0b1..At least one bit sent as recessive is received as dominant.
     */
#define CAN_ESR1_BIT1ERR(x)                      (((UINT32)(((UINT32)(x)) << CAN_ESR1_BIT1ERR_SHIFT)) & CAN_ESR1_BIT1ERR_MASK)

#define CAN_ESR1_RWRNINT_MASK                    (0x10000U)
#define CAN_ESR1_RWRNINT_SHIFT                   (16U)
    /*  RWRNINT - Rx Warning Interrupt Flag
     *  0b0..No such occurrence.
     *  0b1..The Rx error counter transitioned from less than 96 to greater than or equal to 96.
     */
#define CAN_ESR1_RWRNINT(x)                      (((UINT32)(((UINT32)(x)) << CAN_ESR1_RWRNINT_SHIFT)) & CAN_ESR1_RWRNINT_MASK)

#define CAN_ESR1_TWRNINT_MASK                    (0x20000U)
#define CAN_ESR1_TWRNINT_SHIFT                   (17U)
    /*  TWRNINT - Tx Warning Interrupt Flag
     *  0b0..No such occurrence.
     *  0b1..The Tx error counter transitioned from less than 96 to greater than or equal to 96.
     */
#define CAN_ESR1_TWRNINT(x)                      (((UINT32)(((UINT32)(x)) << CAN_ESR1_TWRNINT_SHIFT)) & CAN_ESR1_TWRNINT_MASK)

#define CAN_ESR1_SYNCH_MASK                      (0x40000U)
#define CAN_ESR1_SYNCH_SHIFT                     (18U)
    /*  SYNCH - CAN Synchronization Status
     *  0b0..FlexCAN is not synchronized to the CAN bus.
     *  0b1..FlexCAN is synchronized to the CAN bus.
     */
#define CAN_ESR1_SYNCH(x)                        (((UINT32)(((UINT32)(x)) << CAN_ESR1_SYNCH_SHIFT)) & CAN_ESR1_SYNCH_MASK)

#define CAN_ESR1_BOFFDONEINT_MASK                (0x80000U)
#define CAN_ESR1_BOFFDONEINT_SHIFT               (19U)
    /*  BOFFDONEINT - Bus Off Done Interrupt
     *  0b0..No such occurrence.
     *  0b1..FlexCAN module has completed Bus Off process.
     */
#define CAN_ESR1_BOFFDONEINT(x)                  (((UINT32)(((UINT32)(x)) << CAN_ESR1_BOFFDONEINT_SHIFT)) & CAN_ESR1_BOFFDONEINT_MASK)

#define CAN_ESR1_ERRINT_FAST_MASK                (0x100000U)
#define CAN_ESR1_ERRINT_FAST_SHIFT               (20U)
    /*  ERRINT_FAST - Error interrupt for errors detected in Data Phase of CAN FD frames with BRS bit set
     *  0b0..No such occurrence.
     *  0b1..Indicates setting of any error bit detected in the data phase of CAN FD frames with the BRS bit set.
     */
#define CAN_ESR1_ERRINT_FAST(x)                  (((UINT32)(((UINT32)(x)) << CAN_ESR1_ERRINT_FAST_SHIFT)) & CAN_ESR1_ERRINT_FAST_MASK)

#define CAN_ESR1_ERROVR_MASK                     (0x200000U)
#define CAN_ESR1_ERROVR_SHIFT                    (21U)
    /*  ERROVR - Error Overrun
     *  0b0..Overrun has not occurred.
     *  0b1..Overrun has occurred.
     */
#define CAN_ESR1_ERROVR(x)                       (((UINT32)(((UINT32)(x)) << CAN_ESR1_ERROVR_SHIFT)) & CAN_ESR1_ERROVR_MASK)

#define CAN_ESR1_STFERR_FAST_MASK                (0x4000000U)
#define CAN_ESR1_STFERR_FAST_SHIFT               (26U)
    /*  STFERR_FAST - Stuffing Error in the Data Phase of CAN FD frames with the BRS bit set
     *  0b0..No such occurrence.
     *  0b1..A stuffing error occurred since last read of this register.
     */
#define CAN_ESR1_STFERR_FAST(x)                  (((UINT32)(((UINT32)(x)) << CAN_ESR1_STFERR_FAST_SHIFT)) & CAN_ESR1_STFERR_FAST_MASK)

#define CAN_ESR1_FRMERR_FAST_MASK                (0x8000000U)
#define CAN_ESR1_FRMERR_FAST_SHIFT               (27U)
    /*  FRMERR_FAST - Form Error in the Data Phase of CAN FD frames with the BRS bit set
     *  0b0..No such occurrence.
     *  0b1..A form error occurred since last read of this register.
     */
#define CAN_ESR1_FRMERR_FAST(x)                  (((UINT32)(((UINT32)(x)) << CAN_ESR1_FRMERR_FAST_SHIFT)) & CAN_ESR1_FRMERR_FAST_MASK)

#define CAN_ESR1_CRCERR_FAST_MASK                (0x10000000U)
#define CAN_ESR1_CRCERR_FAST_SHIFT               (28U)
    /*  CRCERR_FAST - Cyclic Redundancy Check Error in the CRC field of CAN FD frames with the BRS bit set
     *  0b0..No such occurrence.
     *  0b1..A CRC error occurred since last read of this register.
     */
#define CAN_ESR1_CRCERR_FAST(x)                  (((UINT32)(((UINT32)(x)) << CAN_ESR1_CRCERR_FAST_SHIFT)) & CAN_ESR1_CRCERR_FAST_MASK)

#define CAN_ESR1_BIT0ERR_FAST_MASK               (0x40000000U)
#define CAN_ESR1_BIT0ERR_FAST_SHIFT              (30U)
    /*  BIT0ERR_FAST - Bit0 Error in the Data Phase of CAN FD frames with the BRS bit set
     *  0b0..No such occurrence.
     *  0b1..At least one bit sent as dominant is received as recessive.
     */
#define CAN_ESR1_BIT0ERR_FAST(x)                 (((UINT32)(((UINT32)(x)) << CAN_ESR1_BIT0ERR_FAST_SHIFT)) & CAN_ESR1_BIT0ERR_FAST_MASK)

#define CAN_ESR1_BIT1ERR_FAST_MASK               (0x80000000U)
#define CAN_ESR1_BIT1ERR_FAST_SHIFT              (31U)
    /*  BIT1ERR_FAST - Bit1 Error in the Data Phase of CAN FD frames with the BRS bit set
    *  0b0..No such occurrence.
    *  0b1..At least one bit sent as recessive is received as dominant.
    */
#define CAN_ESR1_BIT1ERR_FAST(x)                 (((UINT32)(((UINT32)(x)) << CAN_ESR1_BIT1ERR_FAST_SHIFT)) & CAN_ESR1_BIT1ERR_FAST_MASK)

/* IMASK2 - Interrupt Masks 2 register */

#define CAN_IMASK2_BUF63TO32M_MASK               (0xFFFFFFFFU)
#define CAN_IMASK2_BUF63TO32M_SHIFT              (0U)
    /* BUF63TO32M - Buffer MBi Mask */
#define CAN_IMASK2_BUF63TO32M(x)                 (((UINT32)(((UINT32)(x)) << CAN_IMASK2_BUF63TO32M_SHIFT)) & CAN_IMASK2_BUF63TO32M_MASK)
 
/* IMASK1 - Interrupt Masks 1 register */

#define CAN_IMASK1_BUF31TO0M_MASK                (0xFFFFFFFFU)
#define CAN_IMASK1_BUF31TO0M_SHIFT               (0U)
    /* BUF31TO0M - Buffer MBi Mask */
#define CAN_IMASK1_BUF31TO0M(x)                  (((UINT32)(((UINT32)(x)) << CAN_IMASK1_BUF31TO0M_SHIFT)) & CAN_IMASK1_BUF31TO0M_MASK)
 
/* IFLAG2 - Interrupt Flags 2 register */

#define CAN_IFLAG2_BUF63TO32I_MASK               (0xFFFFFFFFU)
#define CAN_IFLAG2_BUF63TO32I_SHIFT              (0U)
    /* BUF63TO32I - Buffer MBi Interrupt */
#define CAN_IFLAG2_BUF63TO32I(x)                 (((UINT32)(((UINT32)(x)) << CAN_IFLAG2_BUF63TO32I_SHIFT)) & CAN_IFLAG2_BUF63TO32I_MASK)

/* IFLAG1 - Interrupt Flags 1 register */

#define CAN_IFLAG1_BUF0I_MASK                    (0x1U)
#define CAN_IFLAG1_BUF0I_SHIFT                   (0U)
    /*  BUF0I - Buffer MB0 Interrupt Or Clear FIFO bit
     *  0b0..The corresponding buffer has no occurrence of successfully completed transmission or reception when MCR[RFEN]=0.
     *  0b1..The corresponding buffer has successfully completed transmission or reception when MCR[RFEN]=0.
     */
#define CAN_IFLAG1_BUF0I(x)                      (((UINT32)(((UINT32)(x)) << CAN_IFLAG1_BUF0I_SHIFT)) & CAN_IFLAG1_BUF0I_MASK)

#define CAN_IFLAG1_BUF4TO1I_MASK                 (0x1EU)
#define CAN_IFLAG1_BUF4TO1I_SHIFT                (1U)
    /* BUF4TO1I - Buffer MBi Interrupt Or Reserved */
#define CAN_IFLAG1_BUF4TO1I(x)                   (((UINT32)(((UINT32)(x)) << CAN_IFLAG1_BUF4TO1I_SHIFT)) & CAN_IFLAG1_BUF4TO1I_MASK)

#define CAN_IFLAG1_BUF5I_MASK                    (0x20U)
#define CAN_IFLAG1_BUF5I_SHIFT                   (5U)
    /*  BUF5I - Buffer MB5 Interrupt Or Frames available in Rx FIFO
     *  0b0..No occurrence of MB5 completing transmission/reception when MCR[RFEN]=0, or of frame(s) available in the FIFO, when MCR[RFEN]=1
     *  0b1..MB5 completed transmission/reception when MCR[RFEN]=0, or frame(s) available in the Rx FIFO when
     *       MCR[RFEN]=1. It generates a DMA request in case of MCR[RFEN] and MCR[DMA] are enabled.
     */
#define CAN_IFLAG1_BUF5I(x)                      (((UINT32)(((UINT32)(x)) << CAN_IFLAG1_BUF5I_SHIFT)) & CAN_IFLAG1_BUF5I_MASK)

#define CAN_IFLAG1_BUF6I_MASK                    (0x40U)
#define CAN_IFLAG1_BUF6I_SHIFT                   (6U)
    /*  BUF6I - Buffer MB6 Interrupt Or Rx FIFO Warning
     *  0b0..No occurrence of MB6 completing transmission/reception when MCR[RFEN]=0, or of Rx FIFO almost full when MCR[RFEN]=1
     *  0b1..MB6 completed transmission/reception when MCR[RFEN]=0, or Rx FIFO almost full when MCR[RFEN]=1
     */
#define CAN_IFLAG1_BUF6I(x)                      (((UINT32)(((UINT32)(x)) << CAN_IFLAG1_BUF6I_SHIFT)) & CAN_IFLAG1_BUF6I_MASK)

#define CAN_IFLAG1_BUF7I_MASK                    (0x80U)
#define CAN_IFLAG1_BUF7I_SHIFT                   (7U)
    /*  BUF7I - Buffer MB7 Interrupt Or Rx FIFO Overflow
     *  0b0..No occurrence of MB7 completing transmission/reception when MCR[RFEN]=0, or of Rx FIFO overflow when MCR[RFEN]=1
     *  0b1..MB7 completed transmission/reception when MCR[RFEN]=0, or Rx FIFO overflow when MCR[RFEN]=1
     */
#define CAN_IFLAG1_BUF7I(x)                      (((UINT32)(((UINT32)(x)) << CAN_IFLAG1_BUF7I_SHIFT)) & CAN_IFLAG1_BUF7I_MASK)

#define CAN_IFLAG1_BUF31TO8I_MASK                (0xFFFFFF00U)
#define CAN_IFLAG1_BUF31TO8I_SHIFT               (8U)
    /* BUF31TO8I - Buffer MBi Interrupt */
#define CAN_IFLAG1_BUF31TO8I(x)                  (((UINT32)(((UINT32)(x)) << CAN_IFLAG1_BUF31TO8I_SHIFT)) & CAN_IFLAG1_BUF31TO8I_MASK)

    /* CTRL2 - Control 2 register */

#define CAN_CTRL2_EDFLTDIS_MASK                  (0x800U)
#define CAN_CTRL2_EDFLTDIS_SHIFT                 (11U)
    /*  EDFLTDIS - Edge Filter Disable
     *  0b0..Edge filter is enabled
     *  0b1..Edge filter is disabled
     */
#define CAN_CTRL2_EDFLTDIS(x)                    (((UINT32)(((UINT32)(x)) << CAN_CTRL2_EDFLTDIS_SHIFT)) & CAN_CTRL2_EDFLTDIS_MASK)

#define CAN_CTRL2_ISOCANFDEN_MASK                (0x1000U)
#define CAN_CTRL2_ISOCANFDEN_SHIFT               (12U)
    /*  ISOCANFDEN - ISO CAN FD Enable
     *  0b0..FlexCAN operates using the non-ISO CAN FD protocol.
     *  0b1..FlexCAN operates using the ISO CAN FD protocol (ISO 11898-1).
     */
#define CAN_CTRL2_ISOCANFDEN(x)                  (((UINT32)(((UINT32)(x)) << CAN_CTRL2_ISOCANFDEN_SHIFT)) & CAN_CTRL2_ISOCANFDEN_MASK)

#define CAN_CTRL2_PREXCEN_MASK                   (0x4000U)
#define CAN_CTRL2_PREXCEN_SHIFT                  (14U)
    /*  PREXCEN - Protocol Exception Enable
     *  0b0..Protocol exception is disabled.
     *  0b1..Protocol exception is enabled.
     */
#define CAN_CTRL2_PREXCEN(x)                     (((UINT32)(((UINT32)(x)) << CAN_CTRL2_PREXCEN_SHIFT)) & CAN_CTRL2_PREXCEN_MASK)

#define CAN_CTRL2_TIMER_SRC_MASK                 (0x8000U)
#define CAN_CTRL2_TIMER_SRC_SHIFT                (15U)
    /*  TIMER_SRC - Timer Source
     *  0b0..The free running timer is clocked by the CAN bit clock, which defines the baud rate on the CAN bus.
     *  0b1..The free running timer is clocked by an external time tick. The period can be either adjusted to be equal
     *       to the baud rate on the CAN bus, or a different value as required. See the device-specific section for
     *       details about the external time tick.
     */
#define CAN_CTRL2_TIMER_SRC(x)                   (((UINT32)(((UINT32)(x)) << CAN_CTRL2_TIMER_SRC_SHIFT)) & CAN_CTRL2_TIMER_SRC_MASK)

#define CAN_CTRL2_EACEN_MASK                     (0x10000U)
#define CAN_CTRL2_EACEN_SHIFT                    (16U)
    /*  EACEN - Entire Frame Arbitration Field Comparison Enable For Rx Mailboxes
     *  0b0..Rx mailbox filter's IDE bit is always compared and RTR is never compared despite mask bits.
     *  0b1..Enables the comparison of both Rx mailbox filter's IDE and RTR bit with their corresponding bits within
     *       the incoming frame. Mask bits do apply.
     */
#define CAN_CTRL2_EACEN(x)                       (((UINT32)(((UINT32)(x)) << CAN_CTRL2_EACEN_SHIFT)) & CAN_CTRL2_EACEN_MASK)

#define CAN_CTRL2_RRS_MASK                       (0x20000U)
#define CAN_CTRL2_RRS_SHIFT                      (17U)
    /*  RRS - Remote Request Storing
     *  0b0..Remote response frame is generated.
     *  0b1..Remote request frame is stored.
     */
#define CAN_CTRL2_RRS(x)                         (((UINT32)(((UINT32)(x)) << CAN_CTRL2_RRS_SHIFT)) & CAN_CTRL2_RRS_MASK)

#define CAN_CTRL2_MRP_MASK                       (0x40000U)
#define CAN_CTRL2_MRP_SHIFT                      (18U)
    /*  MRP - Mailboxes Reception Priority
     *  0b0..Matching starts from Rx FIFO and continues on mailboxes.
     *  0b1..Matching starts from mailboxes and continues on Rx FIFO.
     */
#define CAN_CTRL2_MRP(x)                         (((UINT32)(((UINT32)(x)) << CAN_CTRL2_MRP_SHIFT)) & CAN_CTRL2_MRP_MASK)

#define CAN_CTRL2_TASD_MASK                      (0xF80000U)
#define CAN_CTRL2_TASD_SHIFT                     (19U)
    /* TASD - Tx Arbitration Start Delay */
#define CAN_CTRL2_TASD(x)                        (((UINT32)(((UINT32)(x)) << CAN_CTRL2_TASD_SHIFT)) & CAN_CTRL2_TASD_MASK)

#define CAN_CTRL2_RFFN_MASK                      (0xF000000U)
#define CAN_CTRL2_RFFN_SHIFT                     (24U)
    /* RFFN - Number Of Rx FIFO Filters */
#define CAN_CTRL2_RFFN(x)                        (((UINT32)(((UINT32)(x)) << CAN_CTRL2_RFFN_SHIFT)) & CAN_CTRL2_RFFN_MASK)

#define CAN_CTRL2_WRMFRZ_MASK                    (0x10000000U)
#define CAN_CTRL2_WRMFRZ_SHIFT                   (28U)
    /*  WRMFRZ - Write-Access To Memory In Freeze Mode
     *  0b0..Maintain the write access restrictions.
     *  0b1..Enable unrestricted write access to FlexCAN memory.
     */
#define CAN_CTRL2_WRMFRZ(x)                      (((UINT32)(((UINT32)(x)) << CAN_CTRL2_WRMFRZ_SHIFT)) & CAN_CTRL2_WRMFRZ_MASK)

#define CAN_CTRL2_ECRWRE_MASK                    (0x20000000U)
#define CAN_CTRL2_ECRWRE_SHIFT                   (29U)
    /*  ECRWRE - Error-correction Configuration Register Write Enable
     *  0b0..Disable update.
     *  0b1..Enable update.
     */
#define CAN_CTRL2_ECRWRE(x)                      (((UINT32)(((UINT32)(x)) << CAN_CTRL2_ECRWRE_SHIFT)) & CAN_CTRL2_ECRWRE_MASK)

#define CAN_CTRL2_BOFFDONEMSK_MASK               (0x40000000U)
#define CAN_CTRL2_BOFFDONEMSK_SHIFT              (30U)
    /*  BOFFDONEMSK - Bus Off Done Interrupt Mask
     *  0b0..Bus off done interrupt disabled.
     *  0b1..Bus off done interrupt enabled.
     */
#define CAN_CTRL2_BOFFDONEMSK(x)                 (((UINT32)(((UINT32)(x)) << CAN_CTRL2_BOFFDONEMSK_SHIFT)) & CAN_CTRL2_BOFFDONEMSK_MASK)

#define CAN_CTRL2_ERRMSK_FAST_MASK               (0x80000000U)
#define CAN_CTRL2_ERRMSK_FAST_SHIFT              (31U)
    /*  ERRMSK_FAST - Error Interrupt Mask for errors detected in the data phase of fast CAN FD frames
     *  0b0..ERRINT_FAST error interrupt disabled.
     *  0b1..ERRINT_FAST error interrupt enabled.
     */
#define CAN_CTRL2_ERRMSK_FAST(x)                 (((UINT32)(((UINT32)(x)) << CAN_CTRL2_ERRMSK_FAST_SHIFT)) & CAN_CTRL2_ERRMSK_FAST_MASK)

/* ESR2 - Error and Status 2 register */

#define CAN_ESR2_IMB_MASK                        (0x2000U)
#define CAN_ESR2_IMB_SHIFT                       (13U)
    /*  IMB - Inactive Mailbox
     *  0b0..If ESR2[VPS] is asserted, the ESR2[LPTM] is not an inactive mailbox.
     *  0b1..If ESR2[VPS] is asserted, there is at least one inactive mailbox. LPTM content is the number of the first one.
     */
#define CAN_ESR2_IMB(x)                          (((UINT32)(((UINT32)(x)) << CAN_ESR2_IMB_SHIFT)) & CAN_ESR2_IMB_MASK)

#define CAN_ESR2_VPS_MASK                        (0x4000U)
#define CAN_ESR2_VPS_SHIFT                       (14U)
    /*  VPS - Valid Priority Status
     *  0b0..Contents of IMB and LPTM are invalid.
     *  0b1..Contents of IMB and LPTM are valid.
     */
#define CAN_ESR2_VPS(x)                          (((UINT32)(((UINT32)(x)) << CAN_ESR2_VPS_SHIFT)) & CAN_ESR2_VPS_MASK)

#define CAN_ESR2_LPTM_MASK                       (0x7F0000U)
#define CAN_ESR2_LPTM_SHIFT                      (16U)
    /* LPTM - Lowest Priority Tx Mailbox */
#define CAN_ESR2_LPTM(x)                         (((UINT32)(((UINT32)(x)) << CAN_ESR2_LPTM_SHIFT)) & CAN_ESR2_LPTM_MASK)

/* CRCR - CRC register */

#define CAN_CRCR_TXCRC_MASK                      (0x7FFFU)
#define CAN_CRCR_TXCRC_SHIFT                     (0U)
    /* TXCRC - Transmitted CRC value */
#define CAN_CRCR_TXCRC(x)                        (((UINT32)(((UINT32)(x)) << CAN_CRCR_TXCRC_SHIFT)) & CAN_CRCR_TXCRC_MASK)

#define CAN_CRCR_MBCRC_MASK                      (0x7F0000U)
#define CAN_CRCR_MBCRC_SHIFT                     (16U)
    /* MBCRC - CRC Mailbox */
#define CAN_CRCR_MBCRC(x)                        (((UINT32)(((UINT32)(x)) << CAN_CRCR_MBCRC_SHIFT)) & CAN_CRCR_MBCRC_MASK)

/* RXFGMASK - Rx FIFO Global Mask register */

#define CAN_RXFGMASK_FGM_MASK                    (0xFFFFFFFFU)
#define CAN_RXFGMASK_FGM_SHIFT                   (0U)
    /* FGM - Rx FIFO Global Mask Bits */
#define CAN_RXFGMASK_FGM(x)                      (((UINT32)(((UINT32)(x)) << CAN_RXFGMASK_FGM_SHIFT)) & CAN_RXFGMASK_FGM_MASK)

/* RXFIR - Rx FIFO Information register */

#define CAN_RXFIR_IDHIT_MASK                     (0x1FFU)
#define CAN_RXFIR_IDHIT_SHIFT                    (0U)
/*  IDHIT - Identifier Acceptance Filter Hit Indicator
 */
#define CAN_RXFIR_IDHIT(x)                       (((UINT32)(((UINT32)(x)) << CAN_RXFIR_IDHIT_SHIFT)) & CAN_RXFIR_IDHIT_MASK)

/* CBT - CAN Bit Timing register */

#define CAN_CBT_EPSEG2_MASK                      (0x1FU)
#define CAN_CBT_EPSEG2_SHIFT                     (0U)
    /* EPSEG2 - Extended Phase Segment 2 */
#define CAN_CBT_EPSEG2(x)                        (((UINT32)(((UINT32)(x)) << CAN_CBT_EPSEG2_SHIFT)) & CAN_CBT_EPSEG2_MASK)

#define CAN_CBT_EPSEG1_MASK                      (0x3E0U)
#define CAN_CBT_EPSEG1_SHIFT                     (5U)
    /* EPSEG1 - Extended Phase Segment 1 */
#define CAN_CBT_EPSEG1(x)                        (((UINT32)(((UINT32)(x)) << CAN_CBT_EPSEG1_SHIFT)) & CAN_CBT_EPSEG1_MASK)

#define CAN_CBT_EPROPSEG_MASK                    (0xFC00U)
#define CAN_CBT_EPROPSEG_SHIFT                   (10U)
    /* EPROPSEG - Extended Propagation Segment */
#define CAN_CBT_EPROPSEG(x)                      (((UINT32)(((UINT32)(x)) << CAN_CBT_EPROPSEG_SHIFT)) & CAN_CBT_EPROPSEG_MASK)

#define CAN_CBT_ERJW_MASK                        (0x1F0000U)
#define CAN_CBT_ERJW_SHIFT                       (16U)
    /* ERJW - Extended Resync Jump Width */
#define CAN_CBT_ERJW(x)                          (((UINT32)(((UINT32)(x)) << CAN_CBT_ERJW_SHIFT)) & CAN_CBT_ERJW_MASK)

#define CAN_CBT_EPRESDIV_MASK                    (0x7FE00000U)
#define CAN_CBT_EPRESDIV_SHIFT                   (21U)
    /* EPRESDIV - Extended Prescaler Division Factor */
#define CAN_CBT_EPRESDIV(x)                      (((UINT32)(((UINT32)(x)) << CAN_CBT_EPRESDIV_SHIFT)) & CAN_CBT_EPRESDIV_MASK)

#define CAN_CBT_BTF_MASK                         (0x80000000U)
#define CAN_CBT_BTF_SHIFT                        (31U)
    /*  BTF - Bit Timing Format Enable
     *  0b0..Extended bit time definitions disabled.
     *  0b1..Extended bit time definitions enabled.
     */
#define CAN_CBT_BTF(x)                           (((UINT32)(((UINT32)(x)) << CAN_CBT_BTF_SHIFT)) & CAN_CBT_BTF_MASK)

/* Rx Individual Mask registers */

/* The count of CAN_CS */
#define CAN_CS_COUNT_MB8B                        (64U)

/* The count of CAN_ID */
#define CAN_ID_COUNT_MB8B                        (64U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB8B                      (64U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB8B2                     (2U)

/* The count of CAN_CS */
#define CAN_CS_COUNT_MB16B                       (42U)

/* The count of CAN_ID */
#define CAN_ID_COUNT_MB16B                       (42U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB16B                     (42U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB16B2                    (4U)

/* The count of CAN_CS */
#define CAN_CS_COUNT_MB32B                       (24U)

/* The count of CAN_ID */
#define CAN_ID_COUNT_MB32B                       (24U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB32B                     (24U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB32B2                    (8U)

/* CS - Message Buffer 0 CS Register..Message Buffer 13 CS Register */

#define CAN_CS_TIME_STAMP_MASK                   (0xFFFFU)
#define CAN_CS_TIME_STAMP_SHIFT                  (0U)
    /* TIME_STAMP - Free-Running Counter Time stamp. This 16-bit field is a copy of the Free-Running
     * Timer, captured for Tx and Rx frames at the time when the beginning of the Identifier field
     * appears on the CAN bus.
     */
#define CAN_CS_TIME_STAMP(x)                     (((UINT32)(((UINT32)(x)) << CAN_CS_TIME_STAMP_SHIFT)) & CAN_CS_TIME_STAMP_MASK)

#define CAN_CS_DLC_MASK                          (0xF0000U)
#define CAN_CS_DLC_SHIFT                         (16U)
    /* DLC - Length of the data to be stored/transmitted. */
#define CAN_CS_DLC(x)                            (((UINT32)(((UINT32)(x)) << CAN_CS_DLC_SHIFT)) & CAN_CS_DLC_MASK)

#define CAN_CS_RTR_MASK                          (0x100000U)
#define CAN_CS_RTR_SHIFT                         (20U)
    /* RTR - Remote Transmission Request. One/zero for remote/data frame. */
#define CAN_CS_RTR(x)                            (((UINT32)(((UINT32)(x)) << CAN_CS_RTR_SHIFT)) & CAN_CS_RTR_MASK)

#define CAN_CS_IDE_MASK                          (0x200000U)
#define CAN_CS_IDE_SHIFT                         (21U)
    /* IDE - ID Extended. One/zero for extended/standard format frame. */
#define CAN_CS_IDE(x)                            (((UINT32)(((UINT32)(x)) << CAN_CS_IDE_SHIFT)) & CAN_CS_IDE_MASK)

#define CAN_CS_SRR_MASK                          (0x400000U)
#define CAN_CS_SRR_SHIFT                         (22U)
    /*  SRR - Substitute Remote Request. Contains a fixed recessive bit. */
#define CAN_CS_SRR(x)                            (((UINT32)(((UINT32)(x)) << CAN_CS_SRR_SHIFT)) & CAN_CS_SRR_MASK)

#define CAN_CS_CODE_MASK                         (0xF000000U)
#define CAN_CS_CODE_SHIFT                        (24U)
    /* CODE - Message Buffer Code. This 4-bit field can be accessed (read or write) by the CPU and by
     * the FlexCAN module itself, as part of the message buffer matching and arbitration process.
     */
#define CAN_CS_CODE(x)                           (((UINT32)(((UINT32)(x)) << CAN_CS_CODE_SHIFT)) & CAN_CS_CODE_MASK)

#define CAN_CS_ESI_MASK                          (0x20000000U)
#define CAN_CS_ESI_SHIFT                         (29U)
    /* ESI - Error State Indicator. This bit indicates if the transmitting node is error active or error passive. */
#define CAN_CS_ESI(x)                            (((UINT32)(((UINT32)(x)) << CAN_CS_ESI_SHIFT)) & CAN_CS_ESI_MASK)

#define CAN_CS_BRS_MASK                          (0x40000000U)
#define CAN_CS_BRS_SHIFT                         (30U)
    /* BRS - Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS_BRS(x)                            (((UINT32)(((UINT32)(x)) << CAN_CS_BRS_SHIFT)) & CAN_CS_BRS_MASK)

#define CAN_CS_EDL_MASK                          (0x80000000U)
#define CAN_CS_EDL_SHIFT                         (31U)
    /* EDL - Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames.
     * The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010.
     */
#define CAN_CS_EDL(x)                            (((UINT32)(((UINT32)(x)) << CAN_CS_EDL_SHIFT)) & CAN_CS_EDL_MASK)

    /* The count of CAN_CS */
#define CAN_CS_COUNT_MB64B                       (14U)

/* ID - Message Buffer 0 ID Register..Message Buffer 13 ID Register */

#define CAN_ID_EXT_MASK                          (0x3FFFFU)
#define CAN_ID_EXT_SHIFT                         (0U)
    /* EXT - Contains extended (LOW word) identifier of message buffer. */
#define CAN_ID_EXT(x)                            (((UINT32)(((UINT32)(x)) << CAN_ID_EXT_SHIFT)) & CAN_ID_EXT_MASK)

#define CAN_ID_STD_MASK                          (0x1FFC0000U)
#define CAN_ID_STD_SHIFT                         (18U)
    /* STD - Contains standard/extended (HIGH word) identifier of message buffer. */
#define CAN_ID_STD(x)                            (((UINT32)(((UINT32)(x)) << CAN_ID_STD_SHIFT)) & CAN_ID_STD_MASK)

#define CAN_ID_PRIO_MASK                         (0xE0000000U)
#define CAN_ID_PRIO_SHIFT                        (29U)
    /* PRIO - Local priority. This 3-bit fieldis only used when LPRIO_EN bit is set in MCR and it only
     * makes sense for Tx buffers. These bits are not transmitted. They are appended to the regular
     * ID to define the transmission priority.
     */
#define CAN_ID_PRIO(x)                           (((UINT32)(((UINT32)(x)) << CAN_ID_PRIO_SHIFT)) & CAN_ID_PRIO_MASK)

    /* The count of CAN_ID */
#define CAN_ID_COUNT_MB64B                       (14U)

    /* WORD - Message Buffer 0 WORD_64B Register..Message Buffer 13 WORD_64B Register */

#define CAN_WORD_DATA_BYTE_3_MASK                (0xFFU)
#define CAN_WORD_DATA_BYTE_3_SHIFT               (0U)
    /* DATA_BYTE_3 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_3(x)                  (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_3_SHIFT)) & CAN_WORD_DATA_BYTE_3_MASK)

#define CAN_WORD_DATA_BYTE_7_MASK                (0xFFU)
#define CAN_WORD_DATA_BYTE_7_SHIFT               (0U)
    /* DATA_BYTE_7 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_7(x)                  (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_7_SHIFT)) & CAN_WORD_DATA_BYTE_7_MASK)

#define CAN_WORD_DATA_BYTE_11_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_11_SHIFT              (0U)
    /* DATA_BYTE_11 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_11(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_11_SHIFT)) & CAN_WORD_DATA_BYTE_11_MASK)

#define CAN_WORD_DATA_BYTE_15_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_15_SHIFT              (0U)
    /* DATA_BYTE_15 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_15(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_15_SHIFT)) & CAN_WORD_DATA_BYTE_15_MASK)

#define CAN_WORD_DATA_BYTE_19_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_19_SHIFT              (0U)
    /* DATA_BYTE_19 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_19(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_19_SHIFT)) & CAN_WORD_DATA_BYTE_19_MASK)

#define CAN_WORD_DATA_BYTE_23_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_23_SHIFT              (0U)
    /* DATA_BYTE_23 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_23(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_23_SHIFT)) & CAN_WORD_DATA_BYTE_23_MASK)

#define CAN_WORD_DATA_BYTE_27_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_27_SHIFT              (0U)
    /* DATA_BYTE_27 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_27(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_27_SHIFT)) & CAN_WORD_DATA_BYTE_27_MASK)

#define CAN_WORD_DATA_BYTE_31_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_31_SHIFT              (0U)
    /* DATA_BYTE_31 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_31(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_31_SHIFT)) & CAN_WORD_DATA_BYTE_31_MASK)

#define CAN_WORD_DATA_BYTE_35_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_35_SHIFT              (0U)
    /* DATA_BYTE_35 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_35(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_35_SHIFT)) & CAN_WORD_DATA_BYTE_35_MASK)

#define CAN_WORD_DATA_BYTE_39_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_39_SHIFT              (0U)
    /* DATA_BYTE_39 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_39(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_39_SHIFT)) & CAN_WORD_DATA_BYTE_39_MASK)

#define CAN_WORD_DATA_BYTE_43_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_43_SHIFT              (0U)
    /* DATA_BYTE_43 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_43(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_43_SHIFT)) & CAN_WORD_DATA_BYTE_43_MASK)

#define CAN_WORD_DATA_BYTE_47_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_47_SHIFT              (0U)
    /* DATA_BYTE_47 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_47(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_47_SHIFT)) & CAN_WORD_DATA_BYTE_47_MASK)

#define CAN_WORD_DATA_BYTE_51_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_51_SHIFT              (0U)
    /* DATA_BYTE_51 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_51(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_51_SHIFT)) & CAN_WORD_DATA_BYTE_51_MASK)

#define CAN_WORD_DATA_BYTE_55_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_55_SHIFT              (0U)
    /* DATA_BYTE_55 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_55(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_55_SHIFT)) & CAN_WORD_DATA_BYTE_55_MASK)

#define CAN_WORD_DATA_BYTE_59_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_59_SHIFT              (0U)
    /* DATA_BYTE_59 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_59(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_59_SHIFT)) & CAN_WORD_DATA_BYTE_59_MASK)

#define CAN_WORD_DATA_BYTE_63_MASK               (0xFFU)
#define CAN_WORD_DATA_BYTE_63_SHIFT              (0U)
    /* DATA_BYTE_63 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_63(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_63_SHIFT)) & CAN_WORD_DATA_BYTE_63_MASK)

#define CAN_WORD_DATA_BYTE_2_MASK                (0xFF00U)
#define CAN_WORD_DATA_BYTE_2_SHIFT               (8U)
    /* DATA_BYTE_2 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_2(x)                  (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_2_SHIFT)) & CAN_WORD_DATA_BYTE_2_MASK)

#define CAN_WORD_DATA_BYTE_6_MASK                (0xFF00U)
#define CAN_WORD_DATA_BYTE_6_SHIFT               (8U)
    /* DATA_BYTE_6 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_6(x)                  (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_6_SHIFT)) & CAN_WORD_DATA_BYTE_6_MASK)

#define CAN_WORD_DATA_BYTE_10_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_10_SHIFT              (8U)
    /* DATA_BYTE_10 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_10(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_10_SHIFT)) & CAN_WORD_DATA_BYTE_10_MASK)

#define CAN_WORD_DATA_BYTE_14_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_14_SHIFT              (8U)
    /* DATA_BYTE_14 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_14(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_14_SHIFT)) & CAN_WORD_DATA_BYTE_14_MASK)

#define CAN_WORD_DATA_BYTE_18_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_18_SHIFT              (8U)
    /* DATA_BYTE_18 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_18(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_18_SHIFT)) & CAN_WORD_DATA_BYTE_18_MASK)

#define CAN_WORD_DATA_BYTE_22_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_22_SHIFT              (8U)
    /* DATA_BYTE_22 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_22(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_22_SHIFT)) & CAN_WORD_DATA_BYTE_22_MASK)

#define CAN_WORD_DATA_BYTE_26_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_26_SHIFT              (8U)
    /* DATA_BYTE_26 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_26(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_26_SHIFT)) & CAN_WORD_DATA_BYTE_26_MASK)

#define CAN_WORD_DATA_BYTE_30_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_30_SHIFT              (8U)
    /* DATA_BYTE_30 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_30(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_30_SHIFT)) & CAN_WORD_DATA_BYTE_30_MASK)

#define CAN_WORD_DATA_BYTE_34_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_34_SHIFT              (8U)
    /* DATA_BYTE_34 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_34(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_34_SHIFT)) & CAN_WORD_DATA_BYTE_34_MASK)

#define CAN_WORD_DATA_BYTE_38_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_38_SHIFT              (8U)
    /* DATA_BYTE_38 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_38(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_38_SHIFT)) & CAN_WORD_DATA_BYTE_38_MASK)

#define CAN_WORD_DATA_BYTE_42_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_42_SHIFT              (8U)
    /* DATA_BYTE_42 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_42(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_42_SHIFT)) & CAN_WORD_DATA_BYTE_42_MASK)

#define CAN_WORD_DATA_BYTE_46_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_46_SHIFT              (8U)
    /* DATA_BYTE_46 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_46(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_46_SHIFT)) & CAN_WORD_DATA_BYTE_46_MASK)

#define CAN_WORD_DATA_BYTE_50_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_50_SHIFT              (8U)
    /* DATA_BYTE_50 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_50(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_50_SHIFT)) & CAN_WORD_DATA_BYTE_50_MASK)

#define CAN_WORD_DATA_BYTE_54_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_54_SHIFT              (8U)
    /* DATA_BYTE_54 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_54(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_54_SHIFT)) & CAN_WORD_DATA_BYTE_54_MASK)

#define CAN_WORD_DATA_BYTE_58_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_58_SHIFT              (8U)
    /* DATA_BYTE_58 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_58(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_58_SHIFT)) & CAN_WORD_DATA_BYTE_58_MASK)

#define CAN_WORD_DATA_BYTE_62_MASK               (0xFF00U)
#define CAN_WORD_DATA_BYTE_62_SHIFT              (8U)
    /* DATA_BYTE_62 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_62(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_62_SHIFT)) & CAN_WORD_DATA_BYTE_62_MASK)

#define CAN_WORD_DATA_BYTE_1_MASK                (0xFF0000U)
#define CAN_WORD_DATA_BYTE_1_SHIFT               (16U)
    /* DATA_BYTE_1 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_1(x)                  (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_1_SHIFT)) & CAN_WORD_DATA_BYTE_1_MASK)

#define CAN_WORD_DATA_BYTE_5_MASK                (0xFF0000U)
#define CAN_WORD_DATA_BYTE_5_SHIFT               (16U)
    /* DATA_BYTE_5 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_5(x)                  (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_5_SHIFT)) & CAN_WORD_DATA_BYTE_5_MASK)

#define CAN_WORD_DATA_BYTE_9_MASK                (0xFF0000U)
#define CAN_WORD_DATA_BYTE_9_SHIFT               (16U)
    /* DATA_BYTE_9 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_9(x)                  (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_9_SHIFT)) & CAN_WORD_DATA_BYTE_9_MASK)

#define CAN_WORD_DATA_BYTE_13_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_13_SHIFT              (16U)
    /* DATA_BYTE_13 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_13(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_13_SHIFT)) & CAN_WORD_DATA_BYTE_13_MASK)

#define CAN_WORD_DATA_BYTE_17_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_17_SHIFT              (16U)
    /* DATA_BYTE_17 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_17(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_17_SHIFT)) & CAN_WORD_DATA_BYTE_17_MASK)

#define CAN_WORD_DATA_BYTE_21_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_21_SHIFT              (16U)
    /* DATA_BYTE_21 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_21(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_21_SHIFT)) & CAN_WORD_DATA_BYTE_21_MASK)

#define CAN_WORD_DATA_BYTE_25_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_25_SHIFT              (16U)
    /* DATA_BYTE_25 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_25(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_25_SHIFT)) & CAN_WORD_DATA_BYTE_25_MASK)

#define CAN_WORD_DATA_BYTE_29_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_29_SHIFT              (16U)
    /* DATA_BYTE_29 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_29(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_29_SHIFT)) & CAN_WORD_DATA_BYTE_29_MASK)

#define CAN_WORD_DATA_BYTE_33_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_33_SHIFT              (16U)
    /* DATA_BYTE_33 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_33(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_33_SHIFT)) & CAN_WORD_DATA_BYTE_33_MASK)

#define CAN_WORD_DATA_BYTE_37_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_37_SHIFT              (16U)
    /* DATA_BYTE_37 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_37(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_37_SHIFT)) & CAN_WORD_DATA_BYTE_37_MASK)

#define CAN_WORD_DATA_BYTE_41_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_41_SHIFT              (16U)
    /* DATA_BYTE_41 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_41(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_41_SHIFT)) & CAN_WORD_DATA_BYTE_41_MASK)

#define CAN_WORD_DATA_BYTE_45_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_45_SHIFT              (16U)
    /* DATA_BYTE_45 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_45(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_45_SHIFT)) & CAN_WORD_DATA_BYTE_45_MASK)

#define CAN_WORD_DATA_BYTE_49_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_49_SHIFT              (16U)
    /* DATA_BYTE_49 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_49(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_49_SHIFT)) & CAN_WORD_DATA_BYTE_49_MASK)

#define CAN_WORD_DATA_BYTE_53_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_53_SHIFT              (16U)
    /* DATA_BYTE_53 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_53(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_53_SHIFT)) & CAN_WORD_DATA_BYTE_53_MASK)

#define CAN_WORD_DATA_BYTE_57_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_57_SHIFT              (16U)
    /* DATA_BYTE_57 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_57(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_57_SHIFT)) & CAN_WORD_DATA_BYTE_57_MASK)

#define CAN_WORD_DATA_BYTE_61_MASK               (0xFF0000U)
#define CAN_WORD_DATA_BYTE_61_SHIFT              (16U)
    /* DATA_BYTE_61 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_61(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_61_SHIFT)) & CAN_WORD_DATA_BYTE_61_MASK)

#define CAN_WORD_DATA_BYTE_0_MASK                (0xFF000000U)
#define CAN_WORD_DATA_BYTE_0_SHIFT               (24U)
    /* DATA_BYTE_0 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_0(x)                  (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_0_SHIFT)) & CAN_WORD_DATA_BYTE_0_MASK)

#define CAN_WORD_DATA_BYTE_4_MASK                (0xFF000000U)
#define CAN_WORD_DATA_BYTE_4_SHIFT               (24U)
    /* DATA_BYTE_4 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_4(x)                  (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_4_SHIFT)) & CAN_WORD_DATA_BYTE_4_MASK)

#define CAN_WORD_DATA_BYTE_8_MASK                (0xFF000000U)
#define CAN_WORD_DATA_BYTE_8_SHIFT               (24U)
    /* DATA_BYTE_8 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_8(x)                  (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_8_SHIFT)) & CAN_WORD_DATA_BYTE_8_MASK)

#define CAN_WORD_DATA_BYTE_12_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_12_SHIFT              (24U)
    /* DATA_BYTE_12 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_12(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_12_SHIFT)) & CAN_WORD_DATA_BYTE_12_MASK)

#define CAN_WORD_DATA_BYTE_16_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_16_SHIFT              (24U)
    /* DATA_BYTE_16 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_16(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_16_SHIFT)) & CAN_WORD_DATA_BYTE_16_MASK)

#define CAN_WORD_DATA_BYTE_20_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_20_SHIFT              (24U)
    /* DATA_BYTE_20 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_20(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_20_SHIFT)) & CAN_WORD_DATA_BYTE_20_MASK)

#define CAN_WORD_DATA_BYTE_24_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_24_SHIFT              (24U)
    /* DATA_BYTE_24 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_24(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_24_SHIFT)) & CAN_WORD_DATA_BYTE_24_MASK)

#define CAN_WORD_DATA_BYTE_28_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_28_SHIFT              (24U)
    /* DATA_BYTE_28 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_28(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_28_SHIFT)) & CAN_WORD_DATA_BYTE_28_MASK)

#define CAN_WORD_DATA_BYTE_32_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_32_SHIFT              (24U)
    /* DATA_BYTE_32 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_32(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_32_SHIFT)) & CAN_WORD_DATA_BYTE_32_MASK)

#define CAN_WORD_DATA_BYTE_36_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_36_SHIFT              (24U)
    /* DATA_BYTE_36 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_36(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_36_SHIFT)) & CAN_WORD_DATA_BYTE_36_MASK)

#define CAN_WORD_DATA_BYTE_40_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_40_SHIFT              (24U)
    /* DATA_BYTE_40 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_40(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_40_SHIFT)) & CAN_WORD_DATA_BYTE_40_MASK)

#define CAN_WORD_DATA_BYTE_44_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_44_SHIFT              (24U)
    /* DATA_BYTE_44 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_44(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_44_SHIFT)) & CAN_WORD_DATA_BYTE_44_MASK)

#define CAN_WORD_DATA_BYTE_48_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_48_SHIFT              (24U)
    /* DATA_BYTE_48 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_48(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_48_SHIFT)) & CAN_WORD_DATA_BYTE_48_MASK)

#define CAN_WORD_DATA_BYTE_52_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_52_SHIFT              (24U)
    /* DATA_BYTE_52 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_52(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_52_SHIFT)) & CAN_WORD_DATA_BYTE_52_MASK)

#define CAN_WORD_DATA_BYTE_56_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_56_SHIFT              (24U)
    /* DATA_BYTE_56 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_56(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_56_SHIFT)) & CAN_WORD_DATA_BYTE_56_MASK)

#define CAN_WORD_DATA_BYTE_60_MASK               (0xFF000000U)
#define CAN_WORD_DATA_BYTE_60_SHIFT              (24U)
    /* DATA_BYTE_60 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD_DATA_BYTE_60(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD_DATA_BYTE_60_SHIFT)) & CAN_WORD_DATA_BYTE_60_MASK)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB64B                     (14U)

/* The count of CAN_WORD */
#define CAN_WORD_COUNT_MB64B2                    (16U)

/* The count of CAN_CS */
#define CAN_CS_COUNT                             (64U)

/* The count of CAN_ID */
#define CAN_ID_COUNT                             (64U)

/* WORD0 - Message Buffer 0 WORD0 Register ... Message Buffer 63 WORD0 Register */

#define CAN_WORD0_DATA_BYTE_3_MASK               (0xFFU)
#define CAN_WORD0_DATA_BYTE_3_SHIFT              (0U)
    /* DATA_BYTE_3 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD0_DATA_BYTE_3(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD0_DATA_BYTE_3_SHIFT)) & CAN_WORD0_DATA_BYTE_3_MASK)

#define CAN_WORD0_DATA_BYTE_2_MASK               (0xFF00U)
#define CAN_WORD0_DATA_BYTE_2_SHIFT              (8U)
    /* DATA_BYTE_2 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD0_DATA_BYTE_2(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD0_DATA_BYTE_2_SHIFT)) & CAN_WORD0_DATA_BYTE_2_MASK)

#define CAN_WORD0_DATA_BYTE_1_MASK               (0xFF0000U)
#define CAN_WORD0_DATA_BYTE_1_SHIFT              (16U)
    /* DATA_BYTE_1 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD0_DATA_BYTE_1(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD0_DATA_BYTE_1_SHIFT)) & CAN_WORD0_DATA_BYTE_1_MASK)

#define CAN_WORD0_DATA_BYTE_0_MASK               (0xFF000000U)
#define CAN_WORD0_DATA_BYTE_0_SHIFT              (24U)
    /* DATA_BYTE_0 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD0_DATA_BYTE_0(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD0_DATA_BYTE_0_SHIFT)) & CAN_WORD0_DATA_BYTE_0_MASK)

    /* The count of CAN_WORD0 */
#define CAN_WORD0_COUNT                          (64U)

/* WORD1 - Message Buffer 0 WORD1 Register ... Message Buffer 63 WORD1 Register */

#define CAN_WORD1_DATA_BYTE_7_MASK               (0xFFU)
#define CAN_WORD1_DATA_BYTE_7_SHIFT              (0U)
    /* DATA_BYTE_7 - Data byte 0 of Rx/Tx frame. */
#define CAN_WORD1_DATA_BYTE_7(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD1_DATA_BYTE_7_SHIFT)) & CAN_WORD1_DATA_BYTE_7_MASK)

#define CAN_WORD1_DATA_BYTE_6_MASK               (0xFF00U)
#define CAN_WORD1_DATA_BYTE_6_SHIFT              (8U)
    /* DATA_BYTE_6 - Data byte 1 of Rx/Tx frame. */
#define CAN_WORD1_DATA_BYTE_6(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD1_DATA_BYTE_6_SHIFT)) & CAN_WORD1_DATA_BYTE_6_MASK)

#define CAN_WORD1_DATA_BYTE_5_MASK               (0xFF0000U)
#define CAN_WORD1_DATA_BYTE_5_SHIFT              (16U)
    /* DATA_BYTE_5 - Data byte 2 of Rx/Tx frame. */
#define CAN_WORD1_DATA_BYTE_5(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD1_DATA_BYTE_5_SHIFT)) & CAN_WORD1_DATA_BYTE_5_MASK)

#define CAN_WORD1_DATA_BYTE_4_MASK               (0xFF000000U)
#define CAN_WORD1_DATA_BYTE_4_SHIFT              (24U)
    /* DATA_BYTE_4 - Data byte 3 of Rx/Tx frame. */
#define CAN_WORD1_DATA_BYTE_4(x)                 (((UINT32)(((UINT32)(x)) << CAN_WORD1_DATA_BYTE_4_SHIFT)) & CAN_WORD1_DATA_BYTE_4_MASK)

    /* The count of CAN_WORD1 */
#define CAN_WORD1_COUNT                          (64U)

/* RXIMR - Rx Individual Mask registers */

#define CAN_RXIMR_MI_MASK                        (0xFFFFFFFFU)
#define CAN_RXIMR_MI_SHIFT                       (0U)
    /* MI - Individual Mask Bits */
#define CAN_RXIMR_MI(x)                          (((UINT32)(((UINT32)(x)) << CAN_RXIMR_MI_SHIFT)) & CAN_RXIMR_MI_MASK)

    /* The count of CAN_RXIMR */
#define CAN_RXIMR_COUNT                          (64U)

/* MECR - Memory Error Control register */

#define CAN_MECR_NCEFAFRZ_MASK                   (0x80U)
#define CAN_MECR_NCEFAFRZ_SHIFT                  (7U)
    /*  NCEFAFRZ - Non-Correctable Errors In FlexCAN Access Put Device In Freeze Mode
     *  0b0..Keep normal operation.
     *  0b1..Put FlexCAN in Freeze mode (see section "Freeze mode").
     */
#define CAN_MECR_NCEFAFRZ(x)                     (((UINT32)(((UINT32)(x)) << CAN_MECR_NCEFAFRZ_SHIFT)) & CAN_MECR_NCEFAFRZ_MASK)

#define CAN_MECR_ECCDIS_MASK                     (0x100U)
#define CAN_MECR_ECCDIS_SHIFT                    (8U)
    /*  ECCDIS - Error Correction Disable
     *  0b0..Enable memory error correction.
     *  0b1..Disable memory error correction.
     */
#define CAN_MECR_ECCDIS(x)                       (((UINT32)(((UINT32)(x)) << CAN_MECR_ECCDIS_SHIFT)) & CAN_MECR_ECCDIS_MASK)

#define CAN_MECR_RERRDIS_MASK                    (0x200U)
#define CAN_MECR_RERRDIS_SHIFT                   (9U)
    /*  RERRDIS - Error Report Disable
     *  0b0..Enable updates of the error report registers.
     *  0b1..Disable updates of the error report registers.
     */
#define CAN_MECR_RERRDIS(x)                      (((UINT32)(((UINT32)(x)) << CAN_MECR_RERRDIS_SHIFT)) & CAN_MECR_RERRDIS_MASK)

#define CAN_MECR_EXTERRIE_MASK                   (0x2000U)
#define CAN_MECR_EXTERRIE_SHIFT                  (13U)
   /*  EXTERRIE - Extended Error Injection Enable
    *  0b0..Error injection is applied only to the 32-bit word.
    *  0b1..Error injection is applied to the 64-bit word.
    */
#define CAN_MECR_EXTERRIE(x)                     (((UINT32)(((UINT32)(x)) << CAN_MECR_EXTERRIE_SHIFT)) & CAN_MECR_EXTERRIE_MASK)

#define CAN_MECR_FAERRIE_MASK                    (0x4000U)
#define CAN_MECR_FAERRIE_SHIFT                   (14U)
   /*  FAERRIE - FlexCAN Access Error Injection Enable
    *  0b0..Injection is disabled.
    *  0b1..Injection is enabled.
    */
#define CAN_MECR_FAERRIE(x)                      (((UINT32)(((UINT32)(x)) << CAN_MECR_FAERRIE_SHIFT)) & CAN_MECR_FAERRIE_MASK)

#define CAN_MECR_HAERRIE_MASK                    (0x8000U)
#define CAN_MECR_HAERRIE_SHIFT                   (15U)
   /*  HAERRIE - Host Access Error Injection Enable
    *  0b0..Injection is disabled.
    *  0b1..Injection is enabled.
    */
#define CAN_MECR_HAERRIE(x)                      (((UINT32)(((UINT32)(x)) << CAN_MECR_HAERRIE_SHIFT)) & CAN_MECR_HAERRIE_MASK)

#define CAN_MECR_CEI_MSK_MASK                    (0x10000U)
#define CAN_MECR_CEI_MSK_SHIFT                   (16U)
   /*  CEI_MSK - Correctable Errors Interrupt Mask
    *  0b0..Interrupt is disabled.
    *  0b1..Interrupt is enabled.
    */
#define CAN_MECR_CEI_MSK(x)                      (((UINT32)(((UINT32)(x)) << CAN_MECR_CEI_MSK_SHIFT)) & CAN_MECR_CEI_MSK_MASK)

#define CAN_MECR_FANCEI_MSK_MASK                 (0x40000U)
#define CAN_MECR_FANCEI_MSK_SHIFT                (18U)
   /*  FANCEI_MSK - FlexCAN Access With Non-Correctable Errors Interrupt Mask
    *  0b0..Interrupt is disabled.
    *  0b1..Interrupt is enabled.
    */
#define CAN_MECR_FANCEI_MSK(x)                   (((UINT32)(((UINT32)(x)) << CAN_MECR_FANCEI_MSK_SHIFT)) & CAN_MECR_FANCEI_MSK_MASK)

#define CAN_MECR_HANCEI_MSK_MASK                 (0x80000U)
#define CAN_MECR_HANCEI_MSK_SHIFT                (19U)
   /*  HANCEI_MSK - Host Access With Non-Correctable Errors Interrupt Mask
    *  0b0..Interrupt is disabled.
    *  0b1..Interrupt is enabled.
    */
#define CAN_MECR_HANCEI_MSK(x)                   (((UINT32)(((UINT32)(x)) << CAN_MECR_HANCEI_MSK_SHIFT)) & CAN_MECR_HANCEI_MSK_MASK)

#define CAN_MECR_ECRWRDIS_MASK                   (0x80000000U)
#define CAN_MECR_ECRWRDIS_SHIFT                  (31U)
   /*  ECRWRDIS - Error Configuration Register Write Disable
    *  0b0..Write is enabled.
    *  0b1..Write is disabled.
    */
#define CAN_MECR_ECRWRDIS(x)                     (((UINT32)(((UINT32)(x)) << CAN_MECR_ECRWRDIS_SHIFT)) & CAN_MECR_ECRWRDIS_MASK)

/* ERRIAR - Error Injection Address register */

#define CAN_ERRIAR_INJADDR_L_MASK                (0x3U)
#define CAN_ERRIAR_INJADDR_L_SHIFT               (0U)
    /* INJADDR_L - Error Injection Address Low */
#define CAN_ERRIAR_INJADDR_L(x)                  (((UINT32)(((UINT32)(x)) << CAN_ERRIAR_INJADDR_L_SHIFT)) & CAN_ERRIAR_INJADDR_L_MASK)

#define CAN_ERRIAR_INJADDR_H_MASK                (0x3FFCU)
#define CAN_ERRIAR_INJADDR_H_SHIFT               (2U)
    /* INJADDR_H - Error Injection Address High */
#define CAN_ERRIAR_INJADDR_H(x)                  (((UINT32)(((UINT32)(x)) << CAN_ERRIAR_INJADDR_H_SHIFT)) & CAN_ERRIAR_INJADDR_H_MASK)

/* ERRIDPR - Error Injection Data Pattern register */

#define CAN_ERRIDPR_DFLIP_MASK                   (0xFFFFFFFFU)
#define CAN_ERRIDPR_DFLIP_SHIFT                  (0U)
    /* DFLIP - Data flip pattern */
#define CAN_ERRIDPR_DFLIP(x)                     (((UINT32)(((UINT32)(x)) << CAN_ERRIDPR_DFLIP_SHIFT)) & CAN_ERRIDPR_DFLIP_MASK)

/* ERRIPPR - Error Injection Parity Pattern register */

#define CAN_ERRIPPR_PFLIP0_MASK                  (0x1FU)
#define CAN_ERRIPPR_PFLIP0_SHIFT                 (0U)
    /* PFLIP0 - Parity Flip Pattern For Byte 0 (Least Significant) */
#define CAN_ERRIPPR_PFLIP0(x)                    (((UINT32)(((UINT32)(x)) << CAN_ERRIPPR_PFLIP0_SHIFT)) & CAN_ERRIPPR_PFLIP0_MASK)

#define CAN_ERRIPPR_PFLIP1_MASK                  (0x1F00U)
#define CAN_ERRIPPR_PFLIP1_SHIFT                 (8U)
    /* PFLIP1 - Parity Flip Pattern For Byte 1 */
#define CAN_ERRIPPR_PFLIP1(x)                    (((UINT32)(((UINT32)(x)) << CAN_ERRIPPR_PFLIP1_SHIFT)) & CAN_ERRIPPR_PFLIP1_MASK)

#define CAN_ERRIPPR_PFLIP2_MASK                  (0x1F0000U)
#define CAN_ERRIPPR_PFLIP2_SHIFT                 (16U)
    /* PFLIP2 - Parity Flip Pattern For Byte 2 */
#define CAN_ERRIPPR_PFLIP2(x)                    (((UINT32)(((UINT32)(x)) << CAN_ERRIPPR_PFLIP2_SHIFT)) & CAN_ERRIPPR_PFLIP2_MASK)

#define CAN_ERRIPPR_PFLIP3_MASK                  (0x1F000000U)
#define CAN_ERRIPPR_PFLIP3_SHIFT                 (24U)
    /* PFLIP3 - Parity Flip Pattern For Byte 3 (most significant) */
#define CAN_ERRIPPR_PFLIP3(x)                    (((UINT32)(((UINT32)(x)) << CAN_ERRIPPR_PFLIP3_SHIFT)) & CAN_ERRIPPR_PFLIP3_MASK)

/* RERRAR - Error Report Address register */

#define CAN_RERRAR_ERRADDR_MASK                  (0x3FFFU)
#define CAN_RERRAR_ERRADDR_SHIFT                 (0U)
    /* ERRADDR - Address Where Error Detected */
#define CAN_RERRAR_ERRADDR(x)                    (((UINT32)(((UINT32)(x)) << CAN_RERRAR_ERRADDR_SHIFT)) & CAN_RERRAR_ERRADDR_MASK)

#define CAN_RERRAR_SAID_MASK                     (0x70000U)
#define CAN_RERRAR_SAID_SHIFT                    (16U)
    /* SAID - SAID */
#define CAN_RERRAR_SAID(x)                       (((UINT32)(((UINT32)(x)) << CAN_RERRAR_SAID_SHIFT)) & CAN_RERRAR_SAID_MASK)

#define CAN_RERRAR_NCE_MASK                      (0x1000000U)
#define CAN_RERRAR_NCE_SHIFT                     (24U)
  /*  NCE - Non-Correctable Error
   *  0b0..Reporting a correctable error
   *  0b1..Reporting a non-correctable error
   */
#define CAN_RERRAR_NCE(x)                        (((UINT32)(((UINT32)(x)) << CAN_RERRAR_NCE_SHIFT)) & CAN_RERRAR_NCE_MASK)

/* RERRDR - Error Report Data register */

#define CAN_RERRDR_RDATA_MASK                    (0xFFFFFFFFU)
#define CAN_RERRDR_RDATA_SHIFT                   (0U)
    /* RDATA - Raw data word read from memory with error */
#define CAN_RERRDR_RDATA(x)                      (((UINT32)(((UINT32)(x)) << CAN_RERRDR_RDATA_SHIFT)) & CAN_RERRDR_RDATA_MASK)

/* RERRSYNR - Error Report Syndrome register */

#define CAN_RERRSYNR_SYND0_MASK                  (0x1FU)
#define CAN_RERRSYNR_SYND0_SHIFT                 (0U)
    /* SYND0 - Error Syndrome For Byte 0 (least significant) */
#define CAN_RERRSYNR_SYND0(x)                    (((UINT32)(((UINT32)(x)) << CAN_RERRSYNR_SYND0_SHIFT)) & CAN_RERRSYNR_SYND0_MASK)

#define CAN_RERRSYNR_BE0_MASK                    (0x80U)
#define CAN_RERRSYNR_BE0_SHIFT                   (7U)
    /*  BE0 - Byte Enabled For Byte 0 (least significant)
     *  0b0..The byte was not read.
     *  0b1..The byte was read.
     */
#define CAN_RERRSYNR_BE0(x)                      (((UINT32)(((UINT32)(x)) << CAN_RERRSYNR_BE0_SHIFT)) & CAN_RERRSYNR_BE0_MASK)

#define CAN_RERRSYNR_SYND1_MASK                  (0x1F00U)
#define CAN_RERRSYNR_SYND1_SHIFT                 (8U)
    /* SYND1 - Error Syndrome for Byte 1 */
#define CAN_RERRSYNR_SYND1(x)                    (((UINT32)(((UINT32)(x)) << CAN_RERRSYNR_SYND1_SHIFT)) & CAN_RERRSYNR_SYND1_MASK)

#define CAN_RERRSYNR_BE1_MASK                    (0x8000U)
#define CAN_RERRSYNR_BE1_SHIFT                   (15U)
   /*  BE1 - Byte Enabled For Byte 1
    *  0b0..The byte was not read.
    *  0b1..The byte was read.
    */
#define CAN_RERRSYNR_BE1(x)                      (((UINT32)(((UINT32)(x)) << CAN_RERRSYNR_BE1_SHIFT)) & CAN_RERRSYNR_BE1_MASK)

#define CAN_RERRSYNR_SYND2_MASK                  (0x1F0000U)
#define CAN_RERRSYNR_SYND2_SHIFT                 (16U)
    /* SYND2 - Error Syndrome For Byte 2 */
#define CAN_RERRSYNR_SYND2(x)                    (((UINT32)(((UINT32)(x)) << CAN_RERRSYNR_SYND2_SHIFT)) & CAN_RERRSYNR_SYND2_MASK)

#define CAN_RERRSYNR_BE2_MASK                    (0x800000U)
#define CAN_RERRSYNR_BE2_SHIFT                   (23U)
    /*  BE2 - Byte Enabled For Byte 2
     *  0b0..The byte was not read.
     *  0b1..The byte was read.
     */
#define CAN_RERRSYNR_BE2(x)                      (((UINT32)(((UINT32)(x)) << CAN_RERRSYNR_BE2_SHIFT)) & CAN_RERRSYNR_BE2_MASK)

#define CAN_RERRSYNR_SYND3_MASK                  (0x1F000000U)
#define CAN_RERRSYNR_SYND3_SHIFT                 (24U)
    /* SYND3 - Error Syndrome For Byte 3 (most significant) */
#define CAN_RERRSYNR_SYND3(x)                    (((UINT32)(((UINT32)(x)) << CAN_RERRSYNR_SYND3_SHIFT)) & CAN_RERRSYNR_SYND3_MASK)

#define CAN_RERRSYNR_BE3_MASK                    (0x80000000U)
#define CAN_RERRSYNR_BE3_SHIFT                   (31U)
    /*  BE3 - Byte Enabled For Byte 3 (most significant)
     *  0b0..The byte was not read.
     *  0b1..The byte was read.
     */
#define CAN_RERRSYNR_BE3(x)                      (((UINT32)(((UINT32)(x)) << CAN_RERRSYNR_BE3_SHIFT)) & CAN_RERRSYNR_BE3_MASK)

/* ERRSR - Error Status register */

#define CAN_ERRSR_CEIOF_MASK                     (0x1U)
#define CAN_ERRSR_CEIOF_SHIFT                    (0U)
    /*  CEIOF - Correctable Error Interrupt Overrun Flag
     *  0b0..No overrun on correctable errors
     *  0b1..Overrun on correctable errors
     */
#define CAN_ERRSR_CEIOF(x)                       (((UINT32)(((UINT32)(x)) << CAN_ERRSR_CEIOF_SHIFT)) & CAN_ERRSR_CEIOF_MASK)

#define CAN_ERRSR_FANCEIOF_MASK                  (0x4U)
#define CAN_ERRSR_FANCEIOF_SHIFT                 (2U)
    /*  FANCEIOF - FlexCAN Access With Non-Correctable Error Interrupt Overrun Flag
     *  0b0..No overrun on non-correctable errors in FlexCAN access
     *  0b1..Overrun on non-correctable errors in FlexCAN access
     */
#define CAN_ERRSR_FANCEIOF(x)                    (((UINT32)(((UINT32)(x)) << CAN_ERRSR_FANCEIOF_SHIFT)) & CAN_ERRSR_FANCEIOF_MASK)

#define CAN_ERRSR_HANCEIOF_MASK                  (0x8U)
#define CAN_ERRSR_HANCEIOF_SHIFT                 (3U)
    /*  HANCEIOF - Host Access With Non-Correctable Error Interrupt Overrun Flag
     *  0b0..No overrun on non-correctable errors in host access
     *  0b1..Overrun on non-correctable errors in host access
     */
#define CAN_ERRSR_HANCEIOF(x)                    (((UINT32)(((UINT32)(x)) << CAN_ERRSR_HANCEIOF_SHIFT)) & CAN_ERRSR_HANCEIOF_MASK)

#define CAN_ERRSR_CEIF_MASK                      (0x10000U)
#define CAN_ERRSR_CEIF_SHIFT                     (16U)
    /*  CEIF - Correctable Error Interrupt Flag
     *  0b0..No correctable errors were detected so far.
     *  0b1..A correctable error was detected.
     */
#define CAN_ERRSR_CEIF(x)                        (((UINT32)(((UINT32)(x)) << CAN_ERRSR_CEIF_SHIFT)) & CAN_ERRSR_CEIF_MASK)

#define CAN_ERRSR_FANCEIF_MASK                   (0x40000U)
#define CAN_ERRSR_FANCEIF_SHIFT                  (18U)
    /*  FANCEIF - FlexCAN Access With Non-Correctable Error Interrupt Flag
     *  0b0..No non-correctable errors were detected in FlexCAN accesses so far.
     *  0b1..A non-correctable error was detected in a FlexCAN access.
     */
#define CAN_ERRSR_FANCEIF(x)                     (((UINT32)(((UINT32)(x)) << CAN_ERRSR_FANCEIF_SHIFT)) & CAN_ERRSR_FANCEIF_MASK)

#define CAN_ERRSR_HANCEIF_MASK                   (0x80000U)
#define CAN_ERRSR_HANCEIF_SHIFT                  (19U)
    /*  HANCEIF - Host Access With Non-Correctable Error Interrupt Flag
     *  0b0..No non-correctable errors were detected in host accesses so far.
     *  0b1..A non-correctable error was detected in a host access.
     */
#define CAN_ERRSR_HANCEIF(x)                     (((UINT32)(((UINT32)(x)) << CAN_ERRSR_HANCEIF_SHIFT)) & CAN_ERRSR_HANCEIF_MASK)

/* FDCTRL - CAN FD Control register */

#define CAN_FDCTRL_TDCVAL_MASK                   (0x3FU)
#define CAN_FDCTRL_TDCVAL_SHIFT                  (0U)
    /* TDCVAL - Transceiver Delay Compensation Value */
#define CAN_FDCTRL_TDCVAL(x)                     (((UINT32)(((UINT32)(x)) << CAN_FDCTRL_TDCVAL_SHIFT)) & CAN_FDCTRL_TDCVAL_MASK)

#define CAN_FDCTRL_TDCOFF_MASK                   (0x1F00U)
#define CAN_FDCTRL_TDCOFF_SHIFT                  (8U)
    /*  TDCOFF - Transceiver Delay Compensation Offset */
#define CAN_FDCTRL_TDCOFF(x)                     (((UINT32)(((UINT32)(x)) << CAN_FDCTRL_TDCOFF_SHIFT)) & CAN_FDCTRL_TDCOFF_MASK)

#define CAN_FDCTRL_TDCFAIL_MASK                  (0x4000U)
#define CAN_FDCTRL_TDCFAIL_SHIFT                 (14U)
    /*  TDCFAIL - Transceiver Delay Compensation Fail
     *  0b0..Measured loop delay is in range.
     *  0b1..Measured loop delay is out of range.
     */
#define CAN_FDCTRL_TDCFAIL(x)                    (((UINT32)(((UINT32)(x)) << CAN_FDCTRL_TDCFAIL_SHIFT)) & CAN_FDCTRL_TDCFAIL_MASK)

#define CAN_FDCTRL_TDCEN_MASK                    (0x8000U)
#define CAN_FDCTRL_TDCEN_SHIFT                   (15U)
    /*  TDCEN - Transceiver Delay Compensation Enable
     *  0b0..TDC is disabled
     *  0b1..TDC is enabled
     */
#define CAN_FDCTRL_TDCEN(x)                      (((UINT32)(((UINT32)(x)) << CAN_FDCTRL_TDCEN_SHIFT)) & CAN_FDCTRL_TDCEN_MASK)

#define CAN_FDCTRL_MBDSR0_MASK                   (0x30000U)
#define CAN_FDCTRL_MBDSR0_SHIFT                  (16U)
    /*  MBDSR0 - Message Buffer Data Size for Region 0
     *  0b00..Selects 8 bytes per message buffer.
     *  0b01..Selects 16 bytes per message buffer.
     *  0b10..Selects 32 bytes per message buffer.
     *  0b11..Selects 64 bytes per message buffer.
     */
#define CAN_FDCTRL_MBDSR0(x)                     (((UINT32)(((UINT32)(x)) << CAN_FDCTRL_MBDSR0_SHIFT)) & CAN_FDCTRL_MBDSR0_MASK)

#define CAN_FDCTRL_MBDSR1_MASK                   (0x180000U)
#define CAN_FDCTRL_MBDSR1_SHIFT                  (19U)
    /*  MBDSR1 - Message Buffer Data Size for Region 1
     *  0b00..Selects 8 bytes per message buffer.
     *  0b01..Selects 16 bytes per message buffer.
     *  0b10..Selects 32 bytes per message buffer.
     *  0b11..Selects 64 bytes per message buffer.
     */
#define CAN_FDCTRL_MBDSR1(x)                     (((UINT32)(((UINT32)(x)) << CAN_FDCTRL_MBDSR1_SHIFT)) & CAN_FDCTRL_MBDSR1_MASK)

#define CAN_FDCTRL_FDRATE_MASK                   (0x80000000U)
#define CAN_FDCTRL_FDRATE_SHIFT                  (31U)
    /*  FDRATE - Bit Rate Switch Enable
     *  0b0..Transmit a frame in nominal rate. The BRS bit in the Tx MB has no effect.
     *  0b1..Transmit a frame with bit rate switching if the BRS bit in the Tx MB is recessive.
     */
#define CAN_FDCTRL_FDRATE(x)                     (((UINT32)(((UINT32)(x)) << CAN_FDCTRL_FDRATE_SHIFT)) & CAN_FDCTRL_FDRATE_MASK)

/* FDCBT - CAN FD Bit Timing register */

#define CAN_FDCBT_FPSEG2_MASK                    (0x7U)
#define CAN_FDCBT_FPSEG2_SHIFT                   (0U)
    /* FPSEG2 - Fast Phase Segment 2 */
#define CAN_FDCBT_FPSEG2(x)                      (((UINT32)(((UINT32)(x)) << CAN_FDCBT_FPSEG2_SHIFT)) & CAN_FDCBT_FPSEG2_MASK)

#define CAN_FDCBT_FPSEG1_MASK                    (0xE0U)
#define CAN_FDCBT_FPSEG1_SHIFT                   (5U)
    /* FPSEG1 - Fast Phase Segment 1 */
#define CAN_FDCBT_FPSEG1(x)                      (((UINT32)(((UINT32)(x)) << CAN_FDCBT_FPSEG1_SHIFT)) & CAN_FDCBT_FPSEG1_MASK)

#define CAN_FDCBT_FPROPSEG_MASK                  (0x7C00U)
#define CAN_FDCBT_FPROPSEG_SHIFT                 (10U)
    /* FPROPSEG - Fast Propagation Segment */
#define CAN_FDCBT_FPROPSEG(x)                    (((UINT32)(((UINT32)(x)) << CAN_FDCBT_FPROPSEG_SHIFT)) & CAN_FDCBT_FPROPSEG_MASK)

#define CAN_FDCBT_FRJW_MASK                      (0x70000U)
#define CAN_FDCBT_FRJW_SHIFT                     (16U)
   /*  FRJW - Fast Resync Jump Width
    */
#define CAN_FDCBT_FRJW(x)                        (((UINT32)(((UINT32)(x)) << CAN_FDCBT_FRJW_SHIFT)) & CAN_FDCBT_FRJW_MASK)

#define CAN_FDCBT_FPRESDIV_MASK                  (0x3FF00000U)
#define CAN_FDCBT_FPRESDIV_SHIFT                 (20U)
    /* FPRESDIV - Fast Prescaler Division Factor */
#define CAN_FDCBT_FPRESDIV(x)                    (((UINT32)(((UINT32)(x)) << CAN_FDCBT_FPRESDIV_SHIFT)) & CAN_FDCBT_FPRESDIV_MASK)

/* FDCRC - CAN FD CRC register */

#define CAN_FDCRC_FD_TXCRC_MASK                  (0x1FFFFFU)
#define CAN_FDCRC_FD_TXCRC_SHIFT                 (0U)
    /* FD_TXCRC - Extended Transmitted CRC value */
#define CAN_FDCRC_FD_TXCRC(x)                    (((UINT32)(((UINT32)(x)) << CAN_FDCRC_FD_TXCRC_SHIFT)) & CAN_FDCRC_FD_TXCRC_MASK)

#define CAN_FDCRC_FD_MBCRC_MASK                  (0x7F000000U)
#define CAN_FDCRC_FD_MBCRC_SHIFT                 (24U)
    /* FD_MBCRC - CRC Mailbox Number for FD_TXCRC */
#define CAN_FDCRC_FD_MBCRC(x)                    (((UINT32)(((UINT32)(x)) << CAN_FDCRC_FD_MBCRC_SHIFT)) & CAN_FDCRC_FD_MBCRC_MASK)

// Should be in ACPI

/* CAN - Peripheral instance base addresses */
/* Peripheral CAN1 base address */
#define CAN1_BASE                                (0x02090000u) //(0x400C4000u)  MQ
/* Peripheral CAN1 base pointer */
#define CAN1                                     ((IMXCAN_REGISTERS *)CAN1_BASE)
/* Peripheral CAN2 base address */
#define CAN2_BASE                                (0x02094000u) //(0x400C8000u)  MQ
/* Peripheral CAN2 base pointer */
#define CAN2                                     ((IMXCAN_REGISTERS *)CAN2_BASE)
/* Peripheral CAN3 base address */
#define CAN3_BASE                                (0x0u)        //(0x40C3C000u)  MQ
/* Peripheral CAN3 base pointer */
#define CAN3                                     ((IMXCAN_REGISTERS *)CAN3_BASE)
/* Array initializer of CAN peripheral base addresses */
#define CAN_BASE_ADDRS                           { 0u, CAN1_BASE, CAN2_BASE, CAN3_BASE }
/* Array initializer of CAN peripheral base pointers */
#define CAN_BASE_PTRS                            { (IMXCAN_REGISTERS *)0u, CAN1, CAN2, CAN3 }
/* Interrupt vectors for the CAN peripheral type
#define CAN_Rx_Warning_IRQS                      { NotAvail_IRQn, CAN1_IRQn, CAN2_IRQn, CAN3_IRQn }
#define CAN_Tx_Warning_IRQS                      { NotAvail_IRQn, CAN1_IRQn, CAN2_IRQn, CAN3_IRQn }
#define CAN_Wake_Up_IRQS                         { NotAvail_IRQn, CAN1_IRQn, CAN2_IRQn, CAN3_IRQn }
#define CAN_Error_IRQS                           { NotAvail_IRQn, CAN1_IRQn, CAN2_IRQn, CAN3_IRQn }
#define CAN_Bus_Off_IRQS                         { NotAvail_IRQn, CAN1_IRQn, CAN2_IRQn, CAN3_IRQn }
#define CAN_ORed_Message_buffer_IRQS             { NotAvail_IRQn, CAN1_IRQn, CAN2_IRQn, CAN3_IRQn }
*/

/*
 *  CAN_WRAPPER Peripheral Access Layer
 */

 /* CAN_WRAPPER - Register Layout Typedef */
typedef struct {
    UINT8 RESERVED_0[2528];
    UINT32 GFWR;                            /* Glitch Filter Width Register, offset: 0x9E0 */
} CAN_WRAPPER_Type;

/*
 *  CAN_WRAPPER Register Masks
 */

/* GFWR - Glitch Filter Width Register */

#define CAN_WRAPPER_GFWR_GFWR_MASK               (0xFFU)
#define CAN_WRAPPER_GFWR_GFWR_SHIFT              (0U)
    /* GFWR - Glitch Filter Width */
#define CAN_WRAPPER_GFWR_GFWR(x)                 (((UINT32)(((UINT32)(x)) << CAN_WRAPPER_GFWR_GFWR_SHIFT)) & CAN_WRAPPER_GFWR_GFWR_MASK)

/* CAN_WRAPPER - Peripheral instance base addresses */
/* Peripheral CAN1_WRAPPER base address */
#define CAN1_WRAPPER_BASE                        (0x400C4000u)
/* Peripheral CAN1_WRAPPER base pointer */
#define CAN1_WRAPPER                             ((CAN_WRAPPER_Type *)CAN1_WRAPPER_BASE)
/* Peripheral CAN2_WRAPPER base address */
#define CAN2_WRAPPER_BASE                        (0x400C8000u)
/* Peripheral CAN2_WRAPPER base pointer */
#define CAN2_WRAPPER                             ((CAN_WRAPPER_Type *)CAN2_WRAPPER_BASE)
/* Peripheral CAN3_WRAPPER base address */
#define CAN3_WRAPPER_BASE                        (0x40C3C000u)
/* Peripheral CAN3_WRAPPER base pointer */
#define CAN3_WRAPPER                             ((CAN_WRAPPER_Type *)CAN3_WRAPPER_BASE)
/* Array initializer of CAN_WRAPPER peripheral base addresses */
#define CAN_WRAPPER_BASE_ADDRS                   { 0u, CAN1_WRAPPER_BASE, CAN2_WRAPPER_BASE, CAN3_WRAPPER_BASE }
/* Array initializer of CAN_WRAPPER peripheral base pointers */
#define CAN_WRAPPER_BASE_PTRS                    { (CAN_WRAPPER_Type *)0u, CAN1_WRAPPER, CAN2_WRAPPER, CAN3_WRAPPER }



/******************************************************************************************************************************************************/

 /* FLEXCAN module features */

 /* Message buffer size */
#define FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(x) (64)
/* Has doze mode support (register bit field MCR[DOZE]). */
#define FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT (1)
/* Instance has doze mode support (register bit field MCR[DOZE]). */
#define FSL_FEATURE_FLEXCAN_INSTANCE_HAS_DOZE_MODE_SUPPORTn(x) (1)
/* Has a glitch filter on the receive pin (register bit field MCR[WAKSRC]). */
#define FSL_FEATURE_FLEXCAN_HAS_GLITCH_FILTER (1)
#if 0
/* Has extended interrupt mask and flag register (register IMASK2, IFLAG2). */
#define FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER (1)
#endif
/* Instance has extended bit timing register (register CBT). */
#define FSL_FEATURE_FLEXCAN_INSTANCE_HAS_EXTENDED_TIMING_REGISTERn(x) (1)
/* Has a receive FIFO DMA feature (register bit field MCR[DMA]). */
#define FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA (1)
/* Instance has a receive FIFO DMA feature (register bit field MCR[DMA]). */
#define FSL_FEATURE_FLEXCAN_INSTANCE_HAS_RX_FIFO_DMAn(x) (1)
/* Remove CAN Engine Clock Source Selection from unsupported part. */
#define FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE (0)
/* Instance remove CAN Engine Clock Source Selection from unsupported part. */
#define FSL_FEATURE_FLEXCAN_INSTANCE_SUPPORT_ENGINE_CLK_SEL_REMOVEn(x) (0)
/* Is affected by errata with ID 5641 (Module does not transmit a message that is enabled to be transmitted at a specific moment during the arbitration process). */
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641 (0)
/* Is affected by errata with ID 5829 (FlexCAN: FlexCAN does not transmit a message that is enabled to be transmitted in a specific moment during the arbitration process). */
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829 (0)
/* Is affected by errata with ID 6032 (FlexCAN: A frame with wrong ID or payload is transmitted into the CAN bus when the Message Buffer under transmission is either aborted or deactivated while the CAN bus is in the Bus Idle state). */
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032 (0)
/* Is affected by errata with ID 9595 (FlexCAN: Corrupt frame possible if the Freeze Mode or the Low-Power Mode are entered during a Bus-Off state). */
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_9595 (0)
/* CAN instance support Flexible Data rate (CAN FD) protocol. */
#define FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(x) (1)
/* Has extra MB interrupt or common one. */
#define FSL_FEATURE_FLEXCAN_HAS_EXTRA_MB_INT (1)
/* Init memory base 1 */
#define FSL_FEATURE_FLEXCAN_INIT_MEMORY_BASE_1 (0x80)
/* Init memory size 1 */
#define FSL_FEATURE_FLEXCAN_INIT_MEMORY_SIZE_1 (0xA60)
/* Init memory base 2 */
#define FSL_FEATURE_FLEXCAN_INIT_MEMORY_BASE_2 (0xF28)
/* Init memory size 2 */
#define FSL_FEATURE_FLEXCAN_INIT_MEMORY_SIZE_2 (0xD8)

#pragma warning(pop)