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
#include "precomp.h"
#pragma hdrstop

#include "imxcanhw.h"
#include "imxcan.h"

IMXCAN_NONPAGED_SEGMENT_BEGIN;

#pragma warning(push)
#pragma warning(disable:4127) // conditional expression is constant

 /*******************************************************************************
  * Definitions
  ******************************************************************************/

  /* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.flexcan"
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032)
#define RXINTERMISSION (CAN_DBG1_CFSM(0x2f))
#define TXINTERMISSION (CAN_DBG1_CFSM(0x14))
#define BUSIDLE        (CAN_DBG1_CFSM(0x02))
#define CBN_VALUE3     (CAN_DBG1_CBN(0x03))
#define DELAY_BUSIDLE  (200)
#endif

/* According to CiA doc 1301 v1.0.0, specified data/nominal phase sample point postion for CAN FD at 80 MHz. */
#define IDEAL_DATA_SP_1  (800U)
#define IDEAL_DATA_SP_2  (750U)
#define IDEAL_DATA_SP_3  (700U)
#define IDEAL_DATA_SP_4  (625U)
#define IDEAL_NOMINAL_SP (800U)

/* According to CiA doc 301 v4.2.0 and previous version. */
#define IDEAL_SP_LOW  (750U)
#define IDEAL_SP_MID  (800U)
#define IDEAL_SP_HIGH (875U)

#define IDEAL_SP_FACTOR (1000U)

/* Define the max value of bit timing segments when use different timing register. */
#define MAX_PROPSEG           (CAN_CTRL1_PROPSEG_MASK >> CAN_CTRL1_PROPSEG_SHIFT)
#define MAX_PSEG1             (CAN_CTRL1_PSEG1_MASK >> CAN_CTRL1_PSEG1_SHIFT)
#define MAX_PSEG2             (CAN_CTRL1_PSEG2_MASK >> CAN_CTRL1_PSEG2_SHIFT)
#define MAX_RJW               (CAN_CTRL1_RJW_MASK >> CAN_CTRL1_RJW_SHIFT)
#define MAX_PRESDIV           (CAN_CTRL1_PRESDIV_MASK >> CAN_CTRL1_PRESDIV_SHIFT)
#define CTRL1_MAX_TIME_QUANTA (1U + MAX_PROPSEG + 1U + MAX_PSEG1 + 1U + MAX_PSEG2 + 1U)
#define CTRL1_MIN_TIME_QUANTA (8U)

#define MAX_EPROPSEG        (CAN_CBT_EPROPSEG_MASK >> CAN_CBT_EPROPSEG_SHIFT)
#define MAX_EPSEG1          (CAN_CBT_EPSEG1_MASK >> CAN_CBT_EPSEG1_SHIFT)
#define MAX_EPSEG2          (CAN_CBT_EPSEG2_MASK >> CAN_CBT_EPSEG2_SHIFT)
#define MAX_ERJW            (CAN_CBT_ERJW_MASK >> CAN_CBT_ERJW_SHIFT)
#define MAX_EPRESDIV        (CAN_CBT_EPRESDIV_MASK >> CAN_CBT_EPRESDIV_SHIFT)
#define CBT_MAX_TIME_QUANTA (1U + MAX_EPROPSEG + 1U + MAX_EPSEG1 + 1U + MAX_EPSEG2 + 1U)
#define CBT_MIN_TIME_QUANTA (8U)

#define MAX_FPROPSEG          (CAN_FDCBT_FPROPSEG_MASK >> CAN_FDCBT_FPROPSEG_SHIFT)
#define MAX_FPSEG1            (CAN_FDCBT_FPSEG1_MASK >> CAN_FDCBT_FPSEG1_SHIFT)
#define MAX_FPSEG2            (CAN_FDCBT_FPSEG2_MASK >> CAN_FDCBT_FPSEG2_SHIFT)
#define MAX_FRJW              (CAN_FDCBT_FRJW_MASK >> CAN_FDCBT_FRJW_SHIFT)
#define MAX_FPRESDIV          (CAN_FDCBT_FPRESDIV_MASK >> CAN_FDCBT_FPRESDIV_SHIFT)
#define FDCBT_MAX_TIME_QUANTA (1U + MAX_FPROPSEG + 0U + MAX_FPSEG1 + 1U + MAX_FPSEG2 + 1U)
#define FDCBT_MIN_TIME_QUANTA (5U)

#define MAX_TDCOFF ((UINT32)CAN_FDCTRL_TDCOFF_MASK >> CAN_FDCTRL_TDCOFF_SHIFT)

#define MAX_NTSEG1            (CAN_ENCBT_NTSEG1_MASK >> CAN_ENCBT_NTSEG1_SHIFT)
#define MAX_NTSEG2            (CAN_ENCBT_NTSEG2_MASK >> CAN_ENCBT_NTSEG2_SHIFT)
#define MAX_NRJW              (CAN_ENCBT_NRJW_MASK >> CAN_ENCBT_NRJW_SHIFT)
#define MAX_ENPRESDIV         (CAN_EPRS_ENPRESDIV_MASK >> CAN_EPRS_ENPRESDIV_SHIFT)
#define ENCBT_MAX_TIME_QUANTA (1U + MAX_NTSEG1 + 1U + MAX_NTSEG2 + 1U)
#define ENCBT_MIN_TIME_QUANTA (8U)

#define MAX_DTSEG1            (CAN_EDCBT_DTSEG1_MASK >> CAN_EDCBT_DTSEG1_SHIFT)
#define MAX_DTSEG2            (CAN_EDCBT_DTSEG2_MASK >> CAN_EDCBT_DTSEG2_SHIFT)
#define MAX_DRJW              (CAN_EDCBT_DRJW_MASK >> CAN_EDCBT_DRJW_SHIFT)
#define MAX_EDPRESDIV         (CAN_EPRS_EDPRESDIV_MASK >> CAN_EPRS_EDPRESDIV_SHIFT)
#define EDCBT_MAX_TIME_QUANTA (1U + MAX_DTSEG1 + 1U + MAX_DTSEG2 + 1U)
#define EDCBT_MIN_TIME_QUANTA (5U)

#define MAX_ETDCOFF ((UINT32)CAN_ETDC_ETDCOFF_MASK >> CAN_ETDC_ETDCOFF_SHIFT)

/* TSEG1 corresponds to the sum of xPROPSEG and xPSEG1, TSEG2 corresponds to the xPSEG2 value. */
#define MIN_TIME_SEGMENT1 (2U)
#define MIN_TIME_SEGMENT2 (2U)

/* Define maximum CAN and CAN FD bit rate supported by FLEXCAN. */
#define MAX_CANFD_BITRATE (8000000U)
#define MAX_CAN_BITRATE   (1000000U)

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_9595) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_9595)
#define CAN_ESR1_FLTCONF_BUSOFF CAN_ESR1_FLTCONF(2U)
#endif

/* Define the range of memory that needs to be initialized when the device has memory error detection feature. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
#define CAN_INIT_RXFIR         ((UINT64)base + 0x4Cu)
#define CAN_INIT_MEMORY_BASE_1 (UINT64 *)((UINT64)base + (UINT64)FSL_FEATURE_FLEXCAN_INIT_MEMORY_BASE_1)
#define CAN_INIT_MEMORY_SIZE_1 FSL_FEATURE_FLEXCAN_INIT_MEMORY_SIZE_1
#define CAN_INIT_MEMORY_BASE_2 (UINT64 *)((UINT64)base + (UINT64)FSL_FEATURE_FLEXCAN_INIT_MEMORY_BASE_2)
#define CAN_INIT_MEMORY_SIZE_2 FSL_FEATURE_FLEXCAN_INIT_MEMORY_SIZE_2
#endif

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
#ifndef CAN_CLOCK_CHECK_NO_AFFECTS
/* If no define such MACRO, it mean that the CAN in current device have no clock affect issue. */
#define CAN_CLOCK_CHECK_NO_AFFECTS (TRUE)
#endif /* CAN_CLOCK_CHECK_NO_AFFECTS */
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*  FlexCAN Internal State. */
enum _flexcan_state
{
    kFLEXCAN_StateIdle = 0x0,       /*< MB/RxFIFO idle. */
    kFLEXCAN_StateRxData = 0x1,     /*< MB receiving. */
    kFLEXCAN_StateRxRemote = 0x2,   /*< MB receiving remote reply. */
    kFLEXCAN_StateTxData = 0x3,     /*< MB transmitting. */
    kFLEXCAN_StateTxRemote = 0x4,   /*< MB transmitting remote request. */
    kFLEXCAN_StateRxFifo = 0x5,     /*< RxFIFO receiving. */
};

/* FlexCAN message buffer CODE for Rx buffers. */
enum _flexcan_mb_code_rx
{
    kFLEXCAN_RxMbInactive = 0x0,    /*< MB is not active. */
    kFLEXCAN_RxMbFull = 0x2,        /*< MB is full. */
    kFLEXCAN_RxMbEmpty = 0x4,       /*< MB is active and empty. */
    kFLEXCAN_RxMbOverrun = 0x6,     /*< MB is overwritten into a full buffer. */
    kFLEXCAN_RxMbBusy = 0x8,        /*< FlexCAN is updating the contents of the MB, The CPU must not access the MB. */
    kFLEXCAN_RxMbRanswer = 0xA,     /*< A frame was configured to recognize a Remote Request Frame and transmit a Response Frame in return. */
    kFLEXCAN_RxMbNotUsed = 0xF,     /*< Not used.*/
};

/* FlexCAN message buffer CODE FOR Tx buffers. */
enum _flexcan_mb_code_tx
{
    kFLEXCAN_TxMbInactive = 0x8,    /*< MB is not active.*/
    kFLEXCAN_TxMbAbort = 0x9,       /*< MB is aborted.*/
    kFLEXCAN_TxMbDataOrRemote = 0xC, /*< MB is a TX Data Frame(when MB RTR = 0) or MB is a TX Remote Request Frame (when MB RTR = 1). */
    kFLEXCAN_TxMbTanswer = 0xE,     /*< MB is a TX Response Request Frame from an incoming Remote Request Frame. */
    kFLEXCAN_TxMbNotUsed = 0xF,     /*< Not used. */
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#if !defined(NDEBUG)
 /*
  * Check if Message Buffer is occupied by Rx FIFO.
  *
  * This function check if Message Buffer is occupied by Rx FIFO.
  *
  * param base FlexCAN peripheral base address.
  * param mbIdx The FlexCAN Message Buffer index.
  * return TRUE if the index MB is occupied by Rx FIFO, FALSE if the index MB not occupied by Rx FIFO.
  */
static BOOLEAN FLEXCAN_IsMbOccupied(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx);
#endif

#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
/*
 * Get the first valid Message buffer ID of give FlexCAN instance.
 *
 * This function is a helper function for Errata 5641 workaround.
 *
 * param base FlexCAN peripheral base address.
 * return The first valid Message Buffer Number.
 */
static UINT8 FLEXCAN_GetFirstValidMb(volatile IMXCAN_REGISTERS *base);
#endif

/*
 * Check if Message Buffer interrupt is enabled.
 *
 * This function check if Message Buffer interrupt is enabled.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 *
 * return TRUE if the index MB interrupt mask enabled, FALSE if the index MB interrupt mask disabled.
 */
static BOOLEAN FLEXCAN_IsMbIntEnabled(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx);

/*
 * Set bit rate of FlexCAN classical CAN frame or CAN FD frame nominal phase.
 *
 * This function set the bit rate of classical CAN frame or CAN FD frame nominal phase base on the value of the
 * parameter passed in. Users need to ensure that the timing segment values (phaseSeg1, phaseSeg2 and propSeg) match the
 * clock and bit rate, if not match, the final output bit rate may not equal the bitRate_Bps value. Suggest use
 * FLEXCAN_CalculateImprovedTimingValues() to get timing configuration.
 *
 * param base FlexCAN peripheral base address.
 * param sourceClock_Hz Source Clock in Hz.
 * param bitRate_Bps Bit rate in Bps.
 * param timingConfig FlexCAN timingConfig.
 */
static void FLEXCAN_SetBitRate(volatile IMXCAN_REGISTERS *base,
    UINT32 sourceClock_Hz,
    UINT32 bitRate_Bps,
    flexcan_timing_config_t timingConfig);

/*
 * Calculates the segment values for a single bit time for classical CAN.
 *
 * This function use to calculates the Classical CAN segment values which will be set in CTRL1/CBT/ENCBT register.
 *
 * param base FlexCAN peripheral base address.
 * param tqNum Number of time quantas per bit, range in 8 ~ 25 when use CTRL1, range in 8 ~ 129 when use CBT, range in
 *             8 ~ 385 when use ENCBT. param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 */
static void FLEXCAN_GetSegments(volatile IMXCAN_REGISTERS *base,
    UINT32 bitRate,
    UINT32 tqNum,
    flexcan_timing_config_t *pTimingConfig);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Set data phase bit rate of FlexCAN FD frame.
 *
 * This function set the data phase bit rate of CAN FD frame base on the value of the parameter
 * passed in. Users need to ensure that the timing segment values (fphaseSeg1, fphaseSeg2 and fpropSeg) match the clock
 * and bit rate, if not match, the final output bit rate may not equal the bitRateFD value. Suggest use
 * FLEXCAN_FDCalculateImprovedTimingValues() to get timing configuration.
 *
 *
 * param base FlexCAN peripheral base address.
 * param sourceClock_Hz Source Clock in Hz.
 * param bitRateFD_Bps FD frame data phase bit rate in Bps.
 * param timingConfig FlexCAN timingConfig.
 */
static void FLEXCAN_SetFDBitRate(volatile IMXCAN_REGISTERS *base,
    UINT32 sourceClock_Hz,
    UINT32 bitRateFD_Bps,
    flexcan_timing_config_t timingConfig);

/*
 * Get Mailbox offset number by dword.
 *
 * This function gets the offset number of the specified mailbox.
 * Mailbox is not consecutive between memory regions when payload is not 8 bytes
 * so need to calculate the specified mailbox address.
 * For example, in the first memory region, MB[0].CS address is 0x4002_4080. For 32 bytes
 * payload frame, the second mailbox is ((1/12)*512 + 1%12*40)/4 = 10, meaning 10 dword
 * after the 0x4002_4080, which is actually the address of mailbox MB[1].CS.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx Mailbox index.
 */
static UINT32 FLEXCAN_GetFDMailboxOffset(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx);

/*
 * Calculates the segment values for a single bit time for CAN FD data phase.
 *
 * This function use to calculates the CAN FD data phase segment values which will be set in CFDCBT/EDCBT
 * register.
 *
 * param bitRateFD Data phase bit rate
 * param tqNum Number of time quanta per bit
 * param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 */
static void FLEXCAN_FDGetSegments(UINT32 bitRateFD, UINT32 tqNum, flexcan_timing_config_t *pTimingConfig);

/*
 * Calculates the improved timing values by specific bit rate for CAN FD nominal phase.
 *
 * This function use to calculates the CAN FD nominal phase timing values according to the given nominal phase bit rate.
 * The Calculated timing values will be set in CBT/ENCBT registers. The calculation is based on the recommendation of
 * the CiA 1301 v1.0.0 document.
 *
 * param bitRate  The CAN FD nominal phase speed in bps defined by user, should be less than or equal to 1Mbps.
 * param sourceClock_Hz The Source clock frequency in Hz.
 * param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 *
 * return TRUE if timing configuration found, FALSE if failed to find configuration.
 */
static BOOLEAN FLEXCAN_CalculateImprovedNominalTimingValues(UINT32 bitRate,
    UINT32 sourceClock_Hz,
    flexcan_timing_config_t *pTimingConfig);

#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Sub Handler Ehanced Rx FIFO event
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param flags FlexCAN interrupt flags.
 *
 * return the status after handle Ehanced Rx FIFO event.
 */
static status_t FLEXCAN_SubHandlerForEhancedRxFifo(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT64 flags);
#endif

/*
 * Sets the FlexCAN classical protocol timing characteristic.
 *
 * This function gives user settings to classical CAN or CAN FD nominal phase timing characteristic.
 * The function is for an experienced user. For less experienced users, call the FLEXCAN_GetDefaultConfig()
 * and get the default timing characteristicsthe, then call FLEXCAN_Init() and fill the
 * bit rate field.
 *
 * note Calling FLEXCAN_SetTimingConfig() overrides the bit rate set
 * in FLEXCAN_Init().
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the timing configuration structure.
 */
void FLEXCAN_SetTimingConfig(volatile IMXCAN_REGISTERS *base, const flexcan_timing_config_t *pConfig);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Sets the FlexCAN FD data phase timing characteristic.
 *
 * This function gives user settings to CAN FD data phase timing characteristic.
 * The function is for an experienced user. For less experienced users, call the FLEXCAN_GetDefaultConfig()
 * and get the default timing characteristicsthe, then call FLEXCAN_FDInit() and fill the
 * data phase bit rate field.
 *
 * note Calling FLEXCAN_SetFDTimingConfig() overrides the bit rate set
 * in FLEXCAN_FDInit().
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the timing configuration structure.
 */
void FLEXCAN_SetFDTimingConfig(volatile IMXCAN_REGISTERS *base, const flexcan_timing_config_t *pConfig);
#endif


/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/

#if DBG
 /*
  * Print control and status registers
  *
  * param base FlexCAN peripheral base address.
  */
void FLEXCAN_DumpRegisters(volatile IMXCAN_REGISTERS *base)
{
    UINT32 tmpMCR       = base->MCR;
    UINT32 tmpCTRL1     = base->CTRL1;
    UINT32 tmpESR1      = base->ESR1;
    UINT32 tmpCTRL2     = base->CTRL2;
    UINT32 tmpCBT       = base->CBT; 
    UINT32 tmpFDCTRL    = base->FDCTRL;
    //tmpMCR, tmpCTRL1, tmpESR1, tmpCTRL2, tmpCBT, tmpFDCTRL;
    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
        "\n           FlexCAN  base: 0x%p\n"
        "MCR: 0x%X  CTRL1: 0x%X  ESR1: 0x%X  CTRL2: 0x%X  CBT: 0x%X  FDCTRL: 0x%X  \n", base,
         tmpMCR,    tmpCTRL1,    tmpESR1,    tmpCTRL2,    tmpCBT,    tmpFDCTRL));
}
#endif


/*
 * Enter FlexCAN Freeze Mode.
 *
 * This function makes the FlexCAN work under Freeze Mode.
 *
 * param base FlexCAN peripheral base address.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_9595) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_9595)
void FLEXCAN_EnterFreezeMode(volatile IMXCAN_REGISTERS *base)
{
    UINT32 u32TimeoutCount = 0U;
    UINT32 u32TempMCR = 0U;
    UINT32 u32TempIMASK1 = 0U;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT32 u32TempIMASK2 = 0U;
#endif

    /* Step1: set FRZ enable in MCR. */
    base->MCR |= CAN_MCR_FRZ_MASK;

    /* Step2: to check if MDIS bit set in MCR. if yes, clear it. */
    if (0U != (base->MCR & CAN_MCR_MDIS_MASK))
    {
        base->MCR &= ~CAN_MCR_MDIS_MASK;
    }

    /* Step3: polling LPMACK. */
    u32TimeoutCount = (UINT32)FLEXCAN_WAIT_TIMEOUT;
    while ((0U == (base->MCR & CAN_MCR_LPMACK_MASK)) && (u32TimeoutCount > 0U))
    {
        u32TimeoutCount--;
    }

    /* Step4: to check FLTCONF in ESR1 register */
    if (0U == (base->ESR1 & CAN_ESR1_FLTCONF_BUSOFF))
    {
        /* Step5B: Set Halt bits. */
        base->MCR |= CAN_MCR_HALT_MASK;

        /* Step6B: Poll the MCR register until the Freeze Acknowledge (FRZACK) bit is set, timeout need more than 178
         * CAN bit length, so 20 multiply timeout is enough. */
        u32TimeoutCount = (UINT32)FLEXCAN_WAIT_TIMEOUT * 20U;
        while ((0U == (base->MCR & CAN_MCR_FRZACK_MASK)) && (u32TimeoutCount > 0U))
        {
            u32TimeoutCount--;
        }
    }
    else
    {
        /* backup MCR and IMASK register. Errata document not descript it, but we need backup for step 8A and 9A. */
        u32TempMCR = base->MCR;
        u32TempIMASK1 = base->IMASK1;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        u32TempIMASK2 = base->IMASK2;
#endif

        /* Step5A: Set the Soft Reset bit ((SOFTRST) in the MCR.*/
        base->MCR |= CAN_MCR_SOFTRST_MASK;

        /* Step6A: Poll the MCR register until the Soft Reset (SOFTRST) bit is cleared. */
        u32TimeoutCount = (UINT32)FLEXCAN_WAIT_TIMEOUT;
        while ((CAN_MCR_SOFTRST_MASK == (base->MCR & CAN_MCR_SOFTRST_MASK)) && (u32TimeoutCount > 0U))
        {
            u32TimeoutCount--;
        }

        /* Step7A: Poll the MCR register until the Freeze Acknowledge (FRZACK) bit is set. */
        u32TimeoutCount = (UINT32)FLEXCAN_WAIT_TIMEOUT;
        while ((0U == (base->MCR & CAN_MCR_FRZACK_MASK)) && (u32TimeoutCount > 0U))
        {
            u32TimeoutCount--;
        }

        /* Step8A: reconfig MCR. */
        base->MCR = u32TempMCR;

        /* Step9A: reconfig IMASK. */
        base->IMASK1 = u32TempIMASK1;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        base->IMASK2 = u32TempIMASK2;
#endif
    }
}
#elif (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_8341) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_8341)
void FLEXCAN_EnterFreezeMode(volatile IMXCAN_REGISTERS *base)
{
    UINT32 u32TimeoutCount = 0U;
    UINT32 u32TempMCR = 0U;
    UINT32 u32TempIMASK1 = 0U;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT32 u32TempIMASK2 = 0U;
#endif

    /* Step1: set FRZ and HALT bit enable in MCR. */
    base->MCR |= CAN_MCR_FRZ_MASK;
    base->MCR |= CAN_MCR_HALT_MASK;

    /* Step2: to check if MDIS bit set in MCR. if yes, clear it. */
    if (0U != (base->MCR & CAN_MCR_MDIS_MASK))
    {
        base->MCR &= ~CAN_MCR_MDIS_MASK;
    }

    /* Step3: Poll the MCR register until the Freeze Acknowledge (FRZACK) bit is set. */
    u32TimeoutCount = (UINT32)FLEXCAN_WAIT_TIMEOUT * 100U;
    while ((0U == (base->MCR & CAN_MCR_FRZACK_MASK)) && (u32TimeoutCount > 0U))
    {
        u32TimeoutCount--;
    }

    /* Step4: check whether the timeout reached. if no skip step5 to step8. */
    if (0U == u32TimeoutCount)
    {
        /* backup MCR and IMASK register. Errata document not descript it, but we need backup for step 8A and 9A. */
        u32TempMCR = base->MCR;
        u32TempIMASK1 = base->IMASK1;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        u32TempIMASK2 = base->IMASK2;
#endif
        /* Step5: Set the Soft Reset bit ((SOFTRST) in the MCR.*/
        base->MCR |= CAN_MCR_SOFTRST_MASK;

        /* Step6: Poll the MCR register until the Soft Reset (SOFTRST) bit is cleared. */
        while (CAN_MCR_SOFTRST_MASK == (base->MCR & CAN_MCR_SOFTRST_MASK))
        {
        }

        /* Step7: reconfig MCR. */
        base->MCR = u32TempMCR;

        /* Step8: reconfig IMASK. */
        base->IMASK1 = u32TempIMASK1;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        base->IMASK2 = u32TempIMASK2;
#endif
    }
}
#else
void FLEXCAN_EnterFreezeMode(volatile IMXCAN_REGISTERS *base)
{
    /* Set Freeze, Halt bits. */
    base->MCR |= CAN_MCR_FRZ_MASK;
    base->MCR |= CAN_MCR_HALT_MASK;
    while (0U == (base->MCR & CAN_MCR_FRZACK_MASK))
    {
    }
}
#endif

/*
 * Exit FlexCAN Freeze Mode.
 *
 * This function makes the FlexCAN leave Freeze Mode.
 *
 * param base FlexCAN peripheral base address.
 */
void FLEXCAN_ExitFreezeMode(volatile IMXCAN_REGISTERS *base)
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    /* Clean FlexCAN Access With Non-Correctable Error Interrupt Flag to avoid be put in freeze mode. */
    FLEXCAN_ClearStatusFlags(base, (UINT64)kFLEXCAN_FlexCanAccessNonCorrectableErrorIntFlag |
        (UINT64)kFLEXCAN_FlexCanAccessNonCorrectableErrorOverrunFlag);
#endif

    /* Clear Freeze, Halt bits. */
    base->MCR &= ~CAN_MCR_HALT_MASK;
    base->MCR &= ~CAN_MCR_FRZ_MASK;

    /* Wait until the FlexCAN Module exit freeze mode. */
    while (0U != (base->MCR & CAN_MCR_FRZACK_MASK))
    {
    }
}

#if !defined(NDEBUG)
/*
 * Check if Message Buffer is occupied by Rx FIFO.
 *
 * This function check if Message Buffer is occupied by Rx FIFO.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 * return TRUE if the index MB is occupied by Rx FIFO, FALSE if the index MB not occupied by Rx FIFO.
 */
static BOOLEAN FLEXCAN_IsMbOccupied(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx)
{
    UINT8 lastOccupiedMb;
    BOOLEAN fgRet;

    /* Is Rx FIFO enabled? */
    if (0U != (base->MCR & CAN_MCR_RFEN_MASK))
    {
        /* Get RFFN value. */
        lastOccupiedMb = (UINT8)((base->CTRL2 & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* Calculate the number of last Message Buffer occupied by Rx FIFO. */
        lastOccupiedMb = ((lastOccupiedMb + 1U) * 2U) + 5U;

#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
        /* the first valid MB should be occupied by ERRATA 5461 or 5829. */
        lastOccupiedMb += 1U;
#endif
        fgRet = (mbIdx <= lastOccupiedMb);
    }
    else
    {
#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
        if (0U == mbIdx)
        {
            fgRet = TRUE;
        }
        else
#endif
        {
            fgRet = FALSE;
        }
    }

    return fgRet;
}
#endif

#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
/*
 * Get the first valid Message buffer ID of give FlexCAN instance.
 *
 * This function is a helper function for Errata 5641 workaround.
 *
 * param base FlexCAN peripheral base address.
 * return The first valid Message Buffer Number.
 */
static UINT8 FLEXCAN_GetFirstValidMb(volatile IMXCAN_REGISTERS *base)
{
    UINT8 firstValidMbNum;

    if (0U != (base->MCR & CAN_MCR_RFEN_MASK))
    {
        firstValidMbNum = (UINT8)((base->CTRL2 & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        firstValidMbNum = ((firstValidMbNum + 1U) * 2U) + 6U;
    }
    else
    {
        firstValidMbNum = 0U;
    }

    return firstValidMbNum;
}
#endif

/*
 * Check if Message Buffer interrupt is enabled.
 *
 * This function check if Message Buffer interrupt is enabled.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 *
 * return TRUE if the index MB interrupt mask enabled, FALSE if the index MB interrupt mask disabled.
 */
static BOOLEAN FLEXCAN_IsMbIntEnabled(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx)
{
    /* Assertion. */
    ASSERT(mbIdx < (UINT8)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base));

    UINT32 flag = 1U;
    BOOLEAN fgRet = FALSE;

#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    if (mbIdx >= 32U)
    {
        fgRet = (0U != (base->IMASK2 & (flag << (mbIdx - 32U))));
    }
    else
#endif
    {
        fgRet = (0U != (base->IMASK1 & (flag << mbIdx)));
    }

    return fgRet;
}

/*
 * Reset the FlexCAN Instance.
 *
 * Restores the FlexCAN module to reset state, notice that this function
 * will set all the registers to reset state so the FlexCAN module can not work
 * after calling this API.
 *
 * param base FlexCAN peripheral base address.
 */
void FLEXCAN_Reset(volatile IMXCAN_REGISTERS *base)
{
    /* The module must should be first exit from low power
     * mode, and then soft reset can be applied.
     */
    ASSERT(0U == (base->MCR & CAN_MCR_MDIS_MASK));

    UINT8 i;

#if (defined(FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    if (0 != (FSL_FEATURE_FLEXCAN_INSTANCE_HAS_DOZE_MODE_SUPPORTn(base)))
    {
        /* De-assert DOZE Enable Bit. */
        base->MCR &= ~CAN_MCR_DOZE_MASK;
    }
#endif

    /* Wait until FlexCAN exit from any Low Power Mode. */
    while (0U != (base->MCR & CAN_MCR_LPMACK_MASK))
    {
    }

    /* Assert Soft Reset Signal. */
    base->MCR |= CAN_MCR_SOFTRST_MASK;
    /* Wait until FlexCAN reset completes. */
    while (0U != (base->MCR & CAN_MCR_SOFTRST_MASK))
    {
    }

    /* Reset MCR register. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_GLITCH_FILTER) && FSL_FEATURE_FLEXCAN_HAS_GLITCH_FILTER)
    base->MCR |= CAN_MCR_WRNEN_MASK | CAN_MCR_WAKSRC_MASK |
        CAN_MCR_MAXMB((UINT32)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base) - 1U);
#else
    base->MCR |=
        CAN_MCR_WRNEN_MASK | CAN_MCR_MAXMB((UINT32)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base) - 1U);
#endif

    /* Reset CTRL1 and CTRL2 register. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    /* SMP bit cannot be asserted when CAN FD is enabled */
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base))
    {
        base->CTRL1 = 0x0;
    }
    else
    {
        base->CTRL1 = CAN_CTRL1_SMP_MASK;
    }
#else
    base->CTRL1 = CAN_CTRL1_SMP_MASK;
#endif
    base->CTRL2 = CAN_CTRL2_TASD(0x16) | CAN_CTRL2_RRS_MASK | CAN_CTRL2_EACEN_MASK;
    base->CTRL1 = CAN_CTRL1_SMP_MASK;

#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    /* Clean all Wake Up Message Buffer memory. */
    (void)memset((void *)&base->WMB[0], 0, sizeof(base->WMB));
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    /* Enable unrestricted write access to FlexCAN memory. */
    base->CTRL2 |= CAN_CTRL2_WRMFRZ_MASK;
    /* Do memory initialization for all FlexCAN RAM in order to have the parity bits in memory properly
       updated. */
    *(volatile UINT32 *)CAN_INIT_RXFIR = 0x0U;
    (void)memset((void *)CAN_INIT_MEMORY_BASE_1, 0, CAN_INIT_MEMORY_SIZE_1);
    (void)memset((void *)CAN_INIT_MEMORY_BASE_2, 0, CAN_INIT_MEMORY_SIZE_2);
    /* Disable unrestricted write access to FlexCAN memory. */
    base->CTRL2 &= ~CAN_CTRL2_WRMFRZ_MASK;

    /* Clean all memory error flags. */
    FLEXCAN_ClearStatusFlags(base, (UINT64)kFLEXCAN_AllMemoryErrorFlag);
#else
    /* Only need clean all Message Buffer memory. */
    (void)memset((void *)&base->MB[0], 0, sizeof(base->MB));
#endif

    /* Clean all individual Rx Mask of Message Buffers. */
    for (i = 0; i < (UINT32)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base); i++)
    {
        base->RXIMR[i] = 0x3FFFFFFF;
    }

    /* Clean Global Mask of Message Buffers. */
    base->RXMGMASK = 0x3FFFFFFF;
    /* Clean Global Mask of Message Buffer 14. */
    base->RX14MASK = 0x3FFFFFFF;
    /* Clean Global Mask of Message Buffer 15. */
    base->RX15MASK = 0x3FFFFFFF;
    /* Clean Global Mask of Rx FIFO. */
    base->RXFGMASK = 0x3FFFFFFF;
}

/*
 * Set bit rate of FlexCAN classical CAN frame or CAN FD frame nominal phase.
 *
 * This function set the bit rate of classical CAN frame or CAN FD frame nominal phase base on the value of the
 * parameter passed in. Users need to ensure that the timing segment values (phaseSeg1, phaseSeg2 and propSeg) match the
 * clock and bit rate, if not match, the final output bit rate may not equal the bitRate_Bps value. Suggest use
 * FLEXCAN_CalculateImprovedTimingValues() to get timing configuration.
 *
 * param base FlexCAN peripheral base address.
 * param sourceClock_Hz Source Clock in Hz.
 * param bitRate_Bps Bit rate in Bps.
 * param timingConfig FlexCAN timingConfig.
 */
static void FLEXCAN_SetBitRate(volatile IMXCAN_REGISTERS *base,
    UINT32 sourceClock_Hz,
    UINT32 bitRate_Bps,
    flexcan_timing_config_t timingConfig)
{
    /* FlexCAN classical CAN frame or CAN FD frame nominal phase timing setting formula:
     * quantum = 1 + (phaseSeg1 + 1) + (phaseSeg2 + 1) + (propSeg + 1);
     */
    UINT32 quantum = (1U + ((UINT32)timingConfig.phaseSeg1 + 1U) + ((UINT32)timingConfig.phaseSeg2 + 1U) +
        ((UINT32)timingConfig.propSeg + 1U));

    /* Assertion: Desired bit rate is too high. */
    ASSERT(bitRate_Bps <= 1000000U);
    /* Assertion: Source clock should greater than or equal to bit rate * quantum. */
    ASSERT((bitRate_Bps * quantum) <= sourceClock_Hz);
    /* Assertion: Desired bit rate is too low, the bit rate * quantum * max prescaler divider value should greater than
       or equal to source clock. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base))
    {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
        ASSERT((bitRate_Bps * quantum * MAX_ENPRESDIV) >= sourceClock_Hz);
#else
        ASSERT((bitRate_Bps * quantum * MAX_EPRESDIV) >= sourceClock_Hz);
#endif
    }
    else
    {
        ASSERT((bitRate_Bps * quantum * MAX_PRESDIV) >= sourceClock_Hz);
    }
#else
    ASSERT((bitRate_Bps * quantum * MAX_PRESDIV) >= sourceClock_Hz);
#endif
    if (quantum < (MIN_TIME_SEGMENT1 + MIN_TIME_SEGMENT2 + 1U))
    {
        /* No valid timing configuration. */
        timingConfig.preDivider = 0U;
    }
    else
    {
        timingConfig.preDivider = (UINT16)((sourceClock_Hz / (bitRate_Bps * quantum)) - 1U);
    }

    /* Update actual timing characteristic. */
    FLEXCAN_SetTimingConfig(base, (const flexcan_timing_config_t *)&timingConfig);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Set data phase bit rate of FlexCAN FD frame.
 *
 * This function set the data phase bit rate of CAN FD frame base on the value of the parameter
 * passed in. Users need to ensure that the timing segment values (fphaseSeg1, fphaseSeg2 and fpropSeg) match the clock
 * and bit rate, if not match, the final output bit rate may not equal the bitRateFD value. Suggest use
 * FLEXCAN_FDCalculateImprovedTimingValues() to get timing configuration.
 *
 *
 * param base FlexCAN peripheral base address.
 * param sourceClock_Hz Source Clock in Hz.
 * param bitRateFD_Bps FD frame data phase bit rate in Bps.
 * param timingConfig FlexCAN timingConfig.
 */
static void FLEXCAN_SetFDBitRate(volatile IMXCAN_REGISTERS *base,
    UINT32 sourceClock_Hz,
    UINT32 bitRateFD_Bps,
    flexcan_timing_config_t timingConfig)
{
    /* FlexCAN FD frame data phase timing setting formula:
     * quantum = 1 + (fphaseSeg1 + 1) + (fphaseSeg2 + 1) + fpropSeg;
     */
    UINT32 quantum = (1U + ((UINT32)timingConfig.fphaseSeg1 + 1U) + ((UINT32)timingConfig.fphaseSeg2 + 1U) +
        (UINT32)timingConfig.fpropSeg);

    /* Assertion: Desired bit rate is too high. */
    ASSERT(bitRateFD_Bps <= 8000000U);
    /* Assertion: Source clock should greater than or equal to bit rate * quantum. */
    ASSERT((bitRateFD_Bps * quantum) <= sourceClock_Hz);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
    /* ASSERTion: Desired bit rate is too low, the bit rate * quantum * max prescaler divider value should greater than
       or equal to source clock. */
    ASSERT((bitRateFD_Bps * quantum * MAX_EDPRESDIV) >= sourceClock_Hz);
#else
    ASSERT((bitRateFD_Bps * quantum * MAX_FPRESDIV) >= sourceClock_Hz);
#endif
    if (quantum < (MIN_TIME_SEGMENT1 + MIN_TIME_SEGMENT2 + 1U))
    {
        /* No valid data phase timing configuration. */
        timingConfig.fpreDivider = 0U;
    }
    else
    {
        timingConfig.fpreDivider = (UINT16)((sourceClock_Hz / (bitRateFD_Bps * quantum)) - 1U);
    }

    /* Update actual timing characteristic. */
    FLEXCAN_SetFDTimingConfig(base, (const flexcan_timing_config_t *)&timingConfig);
}
#endif

/*
 * Initializes a FlexCAN instance.
 *
 * This function initializes the FlexCAN module with user-defined settings.
 * This example shows how to set up the flexcan_config_t parameters and how
 * to call the FLEXCAN_Init function by passing in these parameters.
 *  code
 *   flexcan_config_t flexcanConfig;
 *   flexcanConfig.clkSrc               = kFLEXCAN_ClkSrc0;
 *   flexcanConfig.bitRate              = 1000000U;
 *   flexcanConfig.maxMbNum             = 16;
 *   flexcanConfig.enableLoopBack       = FALSE;
 *   flexcanConfig.enableSelfWakeup     = FALSE;
 *   flexcanConfig.enableIndividMask    = FALSE;
 *   flexcanConfig.disableSelfReception = FALSE;
 *   flexcanConfig.enableListenOnlyMode = FALSE;
 *   flexcanConfig.enableDoze           = FALSE;
 *   flexcanConfig.timingConfig         = timingConfig;
 *   FLEXCAN_Init(CAN0, &flexcanConfig, 40000000UL);
 *   endcode
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the user-defined configuration structure.
 * param sourceClock_Hz FlexCAN Protocol Engine clock source frequency in Hz.
 */
void FLEXCAN_Init(volatile IMXCAN_REGISTERS *base, const flexcan_config_t *pConfig, UINT32 sourceClock_Hz)
{
    /* Assertion. */
    ASSERT(NULL != pConfig);
    ASSERT((pConfig->maxMbNum > 0U) &&
        (pConfig->maxMbNum <= (UINT8)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base)));
    ASSERT(pConfig->bitRate > 0U);

    UINT32 mcrTemp;
    UINT32 ctrl1Temp;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    //UINT32 instance;
#endif

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    //instance = FLEXCAN_GetInstance(base);
    /* Enable FlexCAN clock. */
    //(void)CLOCK_EnableClock(s_flexcanClock[instance]);
    /*
     * Check the CAN clock in this device whether affected by Other clock gate
     * If it affected, we'd better to change other clock source,
     * If user insist on using that clock source, user need open these gate at same time,
     * In this scene, User need to care the power consumption.
     */
    //ASSERT(CAN_CLOCK_CHECK_NO_AFFECTS);
#if defined(FLEXCAN_PERIPH_CLOCKS)
    /* Enable FlexCAN serial clock. */
    //(void)CLOCK_EnableClock(s_flexcanPeriphClock[instance]);
#endif /* FLEXCAN_PERIPH_CLOCKS */
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if defined(CAN_CTRL1_CLKSRC_MASK)
#if (defined(FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE) && FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE)
    if (0 == FSL_FEATURE_FLEXCAN_INSTANCE_SUPPORT_ENGINE_CLK_SEL_REMOVEn(base))
#endif /* FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE */
    {
        /* Disable FlexCAN Module. */
        FLEXCAN_Enable(base, FALSE);

        /* Protocol-Engine clock source selection, This bit must be set
         * when FlexCAN Module in Disable Mode.
         */
        base->CTRL1 = (kFLEXCAN_ClkSrc0 == pConfig->clkSrc) ? (base->CTRL1 & ~CAN_CTRL1_CLKSRC_MASK) :
            (base->CTRL1 | CAN_CTRL1_CLKSRC_MASK);
    }
#endif /* CAN_CTRL1_CLKSRC_MASK */

    /* Enable FlexCAN Module for configuration. */
    FLEXCAN_Enable(base, TRUE);

    /* Reset to known status. */
    FLEXCAN_Reset(base);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    /* Enable to update in MCER. */
    base->CTRL2 |= CAN_CTRL2_ECRWRE_MASK;
    base->MECR &= ~CAN_MECR_ECRWRDIS_MASK;

    /* Enable/Disable Memory Error Detection and Correction.*/
    base->MECR = (pConfig->enableMemoryErrorControl) ? (base->MECR & ~CAN_MECR_ECCDIS_MASK) :
        (base->MECR | CAN_MECR_ECCDIS_MASK);

    /* Enable/Disable Non-Correctable Errors In FlexCAN Access Put Device In Freeze Mode. */
    base->MECR = (pConfig->enableNonCorrectableErrorEnterFreeze) ? (base->MECR | CAN_MECR_NCEFAFRZ_MASK) :
        (base->MECR & ~CAN_MECR_NCEFAFRZ_MASK);
    /* Lock MCER register. */
    base->CTRL2 &= ~CAN_CTRL2_ECRWRE_MASK;
#endif

    /* Save current CTRL1 value and enable to enter Freeze mode(enabled by default). */
    ctrl1Temp = base->CTRL1;

    /* Save current MCR value and enable to enter Freeze mode(enabled by default). */
    mcrTemp = base->MCR;

    /* Enable Loop Back Mode? */
    ctrl1Temp = (pConfig->enableLoopBack) ? (ctrl1Temp | CAN_CTRL1_LPB_MASK) : (ctrl1Temp & ~CAN_CTRL1_LPB_MASK);

    /* Enable Timer Sync? */
    ctrl1Temp = (pConfig->enableTimerSync) ? (ctrl1Temp | CAN_CTRL1_TSYN_MASK) : (ctrl1Temp & ~CAN_CTRL1_TSYN_MASK);

    /* Enable Listen Only Mode? */
    ctrl1Temp = (pConfig->enableListenOnlyMode) ? ctrl1Temp | CAN_CTRL1_LOM_MASK : ctrl1Temp & ~CAN_CTRL1_LOM_MASK;

    /* Set the maximum number of Message Buffers */
    mcrTemp = (mcrTemp & ~CAN_MCR_MAXMB_MASK) | CAN_MCR_MAXMB((UINT32)pConfig->maxMbNum - 1U);

    /* Enable Self Wake Up Mode and configure the wake up source. */
    mcrTemp = (pConfig->enableSelfWakeup) ? (mcrTemp | CAN_MCR_SLFWAK_MASK) : (mcrTemp & ~CAN_MCR_SLFWAK_MASK);
    mcrTemp = (kFLEXCAN_WakeupSrcFiltered == pConfig->wakeupSrc) ? (mcrTemp | CAN_MCR_WAKSRC_MASK) :
        (mcrTemp & ~CAN_MCR_WAKSRC_MASK);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    /* Enable Pretended Networking Mode? When Pretended Networking mode is set, Self Wake Up feature must be disabled.*/
    mcrTemp = (pConfig->enablePretendedeNetworking) ? ((mcrTemp & ~CAN_MCR_SLFWAK_MASK) | CAN_MCR_PNET_EN_MASK) :
        (mcrTemp & ~CAN_MCR_PNET_EN_MASK);
#endif

    /* Enable Individual Rx Masking and Queue feature? */
    mcrTemp = (pConfig->enableIndividMask) ? (mcrTemp | CAN_MCR_IRMQ_MASK) : (mcrTemp & ~CAN_MCR_IRMQ_MASK);

    /* Disable Self Reception? */
    mcrTemp = (pConfig->disableSelfReception) ? mcrTemp | CAN_MCR_SRXDIS_MASK : mcrTemp & ~CAN_MCR_SRXDIS_MASK;

#if (defined(FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_DOZE_MODE_SUPPORTn(base))
    {
        /* Enable Doze Mode? */
        mcrTemp = (pConfig->enableDoze) ? (mcrTemp | CAN_MCR_DOZE_MASK) : (mcrTemp & ~CAN_MCR_DOZE_MASK);
    }
#endif

    /* Write back CTRL1 Configuration to register. */
    base->CTRL1 = ctrl1Temp;

    /* Write back MCR Configuration to register. */
    base->MCR = mcrTemp;

    /* Bit Rate Configuration.*/
    FLEXCAN_SetBitRate(base, sourceClock_Hz, pConfig->bitRate, pConfig->timingConfig);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Initializes a FlexCAN instance.
 *
 * This function initializes the FlexCAN module with user-defined settings.
 * This example shows how to set up the flexcan_config_t parameters and how
 * to call the FLEXCAN_FDInit function by passing in these parameters.
 *  code
 *   flexcan_config_t flexcanConfig;
 *   flexcanConfig.clkSrc               = kFLEXCAN_ClkSrc0;
 *   flexcanConfig.bitRate              = 1000000U;
 *   flexcanConfig.bitRateFD            = 2000000U;
 *   flexcanConfig.maxMbNum             = 16;
 *   flexcanConfig.enableLoopBack       = FALSE;
 *   flexcanConfig.enableSelfWakeup     = FALSE;
 *   flexcanConfig.enableIndividMask    = FALSE;
 *   flexcanConfig.disableSelfReception = FALSE;
 *   flexcanConfig.enableListenOnlyMode = FALSE;
 *   flexcanConfig.enableDoze           = FALSE;
 *   flexcanConfig.timingConfig         = timingConfig;
 *   FLEXCAN_FDInit(CAN0, &flexcanConfig, 80000000UL, kFLEXCAN_16BperMB, TRUE);
 *   endcode
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the user-defined configuration structure.
 * param sourceClock_Hz FlexCAN Protocol Engine clock source frequency in Hz.
 * param dataSize FlexCAN Message Buffer payload size. The actual transmitted or received CAN FD frame data size needs
 *                to be less than or equal to this value.
 * param brs TRUE if bit rate switch is enabled in FD mode.
 */
void FLEXCAN_FDInit(volatile IMXCAN_REGISTERS *base, const flexcan_config_t *pConfig, UINT32 sourceClock_Hz, flexcan_mb_size_t dataSize, BOOLEAN brs)
{
    ASSERT((UINT32)dataSize <= 3U);
    ASSERT(((pConfig->bitRate < pConfig->bitRateFD) && brs) || ((pConfig->bitRate == pConfig->bitRateFD) && (!brs)));

    UINT32 fdctrl = 0U;

    /* Initialization of classical CAN. */
    FLEXCAN_Init(base, pConfig, sourceClock_Hz);

    /* Extra bit rate setting for CAN FD data phase. */
    FLEXCAN_SetFDBitRate(base, sourceClock_Hz, pConfig->bitRateFD, pConfig->timingConfig);

    /* read FDCTRL register. */
    fdctrl = base->FDCTRL;

    /* Enable FD operation and set bit rate switch. */
    if (brs)
    {
        fdctrl |= CAN_FDCTRL_FDRATE_MASK;
    }
    else
    {
        fdctrl &= ~CAN_FDCTRL_FDRATE_MASK;
    }

    /* Before use "|=" operation for multi-bits field, CPU should clean previous Setting. */
    fdctrl = (fdctrl & ~CAN_FDCTRL_MBDSR0_MASK) | CAN_FDCTRL_MBDSR0(dataSize);
#if defined(CAN_FDCTRL_MBDSR1_MASK)
    fdctrl = (fdctrl & ~CAN_FDCTRL_MBDSR1_MASK) | CAN_FDCTRL_MBDSR1(dataSize);
#endif
#if defined(CAN_FDCTRL_MBDSR2_MASK)
    fdctrl = (fdctrl & ~CAN_FDCTRL_MBDSR2_MASK) | CAN_FDCTRL_MBDSR2(dataSize);
#endif
#if defined(CAN_FDCTRL_MBDSR3_MASK)
    fdctrl = (fdctrl & ~CAN_FDCTRL_MBDSR3_MASK) | CAN_FDCTRL_MBDSR3(dataSize);
#endif

    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);
    /* Enable CAN FD operation. */
    base->MCR |= CAN_MCR_FDEN_MASK;

    if (brs && !(pConfig->enableLoopBack))
    {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
        /* The TDC offset should be configured as shown in this equation : offset = DTSEG1 + 2 */
        if (((UINT32)pConfig->timingConfig.fphaseSeg1 + pConfig->timingConfig.fpropSeg + 2U) *
            (pConfig->timingConfig.fpreDivider + 1U) <
            MAX_ETDCOFF)
        {
            base->ETDC =
                CAN_ETDC_ETDCEN_MASK | CAN_ETDC_TDMDIS(!pConfig->enableTransceiverDelayMeasure) |
                CAN_ETDC_ETDCOFF(((UINT32)pConfig->timingConfig.fphaseSeg1 + pConfig->timingConfig.fpropSeg + 2U) *
                (pConfig->timingConfig.fpreDivider + 1U));
        }
        else
        {
            /* Enable the Transceiver Delay Compensation */
            base->ETDC = CAN_ETDC_ETDCEN_MASK | CAN_ETDC_TDMDIS(!pConfig->enableTransceiverDelayMeasure) |
                CAN_ETDC_ETDCOFF(MAX_ETDCOFF);
        }
#else
        /* The TDC offset should be configured as shown in this equation : offset = PSEG1 + PROPSEG + 2 */
        if (((UINT32)pConfig->timingConfig.fphaseSeg1 + pConfig->timingConfig.fpropSeg + 2U) *
            (pConfig->timingConfig.fpreDivider + 1U) <
            MAX_TDCOFF)
        {
            fdctrl =
                (fdctrl & ~CAN_FDCTRL_TDCOFF_MASK) |
                CAN_FDCTRL_TDCOFF(((UINT32)pConfig->timingConfig.fphaseSeg1 + pConfig->timingConfig.fpropSeg + 2U) *
                (pConfig->timingConfig.fpreDivider + 1U));
        }
        else
        {
            fdctrl = (fdctrl & ~CAN_FDCTRL_TDCOFF_MASK) | CAN_FDCTRL_TDCOFF(MAX_TDCOFF);
        }
        /* Enable the Transceiver Delay Compensation */
        fdctrl = (fdctrl & ~CAN_FDCTRL_TDCEN_MASK) | CAN_FDCTRL_TDCEN_MASK;
#endif
    }

    /* update the FDCTL register. */
    base->FDCTRL = fdctrl;

    /* Enable CAN FD ISO mode by default. */
    base->CTRL2 |= CAN_CTRL2_ISOCANFDEN_MASK;

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}
#endif

/*
 * De-initializes a FlexCAN instance.
 *
 * This function disables the FlexCAN module clock and sets all register values
 * to the reset value.
 *
 * param base FlexCAN peripheral base address.
 */
void FLEXCAN_Deinit(volatile IMXCAN_REGISTERS *base)
{
    /* Reset all Register Contents. */
    FLEXCAN_Reset(base);

    /* Disable FlexCAN module. */
    FLEXCAN_Enable(base, FALSE);
}

/*
 * Gets the default configuration structure.
 *
 * This function initializes the FlexCAN configuration structure to default values. The default
 * values are as follows.
 *   flexcanConfig->clkSrc                               = kFLEXCAN_ClkSrc0;
 *   flexcanConfig->bitRate                              = 1000000U;
 *   flexcanConfig->bitRateFD                            = 2000000U;
 *   flexcanConfig->maxMbNum                             = 16;
 *   flexcanConfig->enableLoopBack                       = FALSE;
 *   flexcanConfig->enableSelfWakeup                     = FALSE;
 *   flexcanConfig->enableIndividMask                    = FALSE;
 *   flexcanConfig->disableSelfReception                 = FALSE;
 *   flexcanConfig->enableListenOnlyMode                 = FALSE;
 *   flexcanConfig->enableDoze                           = FALSE;
 *   flexcanConfig->enablePretendedeNetworking           = FALSE;
 *   flexcanConfig->enableMemoryErrorControl             = TRUE;
 *   flexcanConfig->enableNonCorrectableErrorEnterFreeze = TRUE;
 *   flexcanConfig->enableTransceiverDelayMeasure        = TRUE;
 *   flexcanConfig.timingConfig                          = timingConfig;
 *
 * param pConfig Pointer to the FlexCAN configuration structure.
 */
void FLEXCAN_GetDefaultConfig(flexcan_config_t *pConfig)
{
    /* Assertion. */
    ASSERT(NULL != pConfig);

    /* Initializes the configure structure to zero. */
    (void)memset(pConfig, 0, sizeof(*pConfig));

    /* Initialize FlexCAN Module config struct with default value. */
    pConfig->clkSrc = kFLEXCAN_ClkSrc0;
    pConfig->bitRate = 1000000U;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    pConfig->bitRateFD = 2000000U;
#endif
    pConfig->maxMbNum = 16;
    pConfig->enableLoopBack = FALSE;
    pConfig->enableTimerSync = TRUE;
    pConfig->enableSelfWakeup = FALSE;
    pConfig->wakeupSrc = kFLEXCAN_WakeupSrcUnfiltered;
    pConfig->enableIndividMask = FALSE;
    pConfig->disableSelfReception = FALSE;
    pConfig->enableListenOnlyMode = FALSE;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    pConfig->enableDoze = FALSE;
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    pConfig->enablePretendedeNetworking = FALSE;
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    pConfig->enableMemoryErrorControl = TRUE;
    pConfig->enableNonCorrectableErrorEnterFreeze = TRUE;
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
    pConfig->enableTransceiverDelayMeasure = TRUE;
#endif

    /* Default protocol timing configuration, nominal bit time quantum is 10 (80% SP), data bit time quantum is 5
     * (60%). Suggest use FLEXCAN_CalculateImprovedTimingValues/FLEXCAN_FDCalculateImprovedTimingValues to get the
     * improved timing configuration.*/
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    pConfig->timingConfig.phaseSeg1 = 1;
    pConfig->timingConfig.phaseSeg2 = 1;
    pConfig->timingConfig.propSeg = 4;
    pConfig->timingConfig.rJumpwidth = 1;
    pConfig->timingConfig.fphaseSeg1 = 1;
    pConfig->timingConfig.fphaseSeg2 = 1;
    pConfig->timingConfig.fpropSeg = 0;
    pConfig->timingConfig.frJumpwidth = 1;
#else
    pConfig->timingConfig.phaseSeg1 = 1;
    pConfig->timingConfig.phaseSeg2 = 1;
    pConfig->timingConfig.propSeg = 4;
    pConfig->timingConfig.rJumpwidth = 1;
#endif
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
/*
 * Configures the FlexCAN Pretended Networking mode.
 *
 * This function configures the FlexCAN Pretended Networking mode with given configuration.
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the FlexCAN Rx FIFO configuration structure.
 */
void FLEXCAN_SetPNConfig(volatile IMXCAN_REGISTERS *base, const flexcan_pn_config_t *pConfig)
{
    /* Assertion. */
    ASSERT(NULL != pConfig);
    ASSERT(0U != pConfig->matchNum);
    UINT32 pnctrl;
    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);
    pnctrl = (pConfig->matchNum > 1U) ? CAN_CTRL1_PN_FCS(0x2U | (UINT32)pConfig->matchSrc) :
        CAN_CTRL1_PN_FCS(pConfig->matchSrc);
    pnctrl |= (pConfig->enableMatch) ? (CAN_CTRL1_PN_WUMF_MSK_MASK) : 0U;
    pnctrl |= (pConfig->enableTimeout) ? (CAN_CTRL1_PN_WTOF_MSK_MASK) : 0U;
    pnctrl |= CAN_CTRL1_PN_NMATCH(pConfig->matchNum) | CAN_CTRL1_PN_IDFS(pConfig->idMatchMode) |
        CAN_CTRL1_PN_PLFS(pConfig->dataMatchMode);
    base->CTRL1_PN = pnctrl;
    base->CTRL2_PN = CAN_CTRL2_PN_MATCHTO(pConfig->timeoutValue);
    base->FLT_ID1 = pConfig->idLower;
    base->FLT_ID2_IDMASK = pConfig->idUpper;
    base->FLT_DLC = CAN_FLT_DLC_FLT_DLC_LO(pConfig->lengthLower) | CAN_FLT_DLC_FLT_DLC_HI(pConfig->lengthUpper);
    base->PL1_LO = pConfig->lowerWord0;
    base->PL1_HI = pConfig->lowerWord1;
    base->PL2_PLMASK_LO = pConfig->upperWord0;
    base->PL2_PLMASK_HI = pConfig->upperWord1;

    FLEXCAN_ClearStatusFlags(base, (UINT64)kFLEXCAN_PNMatchIntFlag | (UINT64)kFLEXCAN_PNTimeoutIntFlag);

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}

/*
 * Reads a FlexCAN Message from Wake Up MB.
 *
 * This function reads a CAN message from the FlexCAN Wake up Message Buffers. There are four Wake up Message Buffers
 * (WMBs) used to store incoming messages in Pretended Networking mode. The WMB index indicates the arrival order. The
 * last message is stored in WMB3.
 *
 * param base FlexCAN peripheral base address.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * param mbIdx The FlexCAN Wake up Message Buffer index. Range in 0x0 ~ 0x3.
 * retval kStatus_Success - Read Message from Wake up Message Buffer successfully.
 * retval kStatus_Fail    - Wake up Message Buffer has no valid content.
 */
status_t FLEXCAN_ReadPNWakeUpMB(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_frame_t *pRxFrame)
{
    /* Assertion. */
    ASSERT(NULL != pRxFrame);
    ASSERT(mbIdx <= 0x3U);

    UINT32 cs_temp;
    status_t status;

    /* Check if Wake Up MB has valid content. */
    if (CAN_WU_MTC_MCOUNTER(mbIdx) <= (base->WU_MTC & CAN_WU_MTC_MCOUNTER_MASK))
    {
        /* Read CS field of wake up Message Buffer. */
        cs_temp = base->WMB[mbIdx].CS;

        /* Store Message ID. */
        pRxFrame->id = base->WMB[mbIdx].ID & (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);

        /* Get the message ID and format. */
        pRxFrame->format = (cs_temp & CAN_CS_IDE_MASK) != 0U ? (UINT8)kFLEXCAN_FrameFormatExtend :
            (UINT8)kFLEXCAN_FrameFormatStandard;

        /* Get the message type. */
        pRxFrame->type =
            (cs_temp & CAN_CS_RTR_MASK) != 0U ? (UINT8)kFLEXCAN_FrameTypeRemote : (UINT8)kFLEXCAN_FrameTypeData;

        /* Get the message length. */
        pRxFrame->length = (UINT8)((cs_temp & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);

        /* Messages received during Pretended Networking mode don't have time stamps, and the respective field in the
           WMB structure must be ignored. */
        pRxFrame->timestamp = 0x0;

        /* Store Message Payload. */
        pRxFrame->dataWord0 = base->WMB[mbIdx].D03;
        pRxFrame->dataWord1 = base->WMB[mbIdx].D47;

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}
#endif

/*
 * Sets the FlexCAN classical protocol timing characteristic.
 *
 * This function gives user settings to classical CAN or CAN FD nominal phase timing characteristic.
 * The function is for an experienced user. For less experienced users, call the FLEXCAN_GetDefaultConfig()
 * and get the default timing characteristicsthe, then call FLEXCAN_Init() and fill the
 * bit rate field.
 *
 * note Calling FLEXCAN_SetTimingConfig() overrides the bit rate set
 * in FLEXCAN_Init().
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the timing configuration structure.
 */
void FLEXCAN_SetTimingConfig(volatile IMXCAN_REGISTERS *base, const flexcan_timing_config_t *pConfig)
{
    /* Assertion. */
    ASSERT(NULL != pConfig);

    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base))
    {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
        /* Enable extended Bit Timing register ENCBT. */
        base->CTRL2 |= CAN_CTRL2_BTE_MASK;

        /* Updating Timing Setting according to configuration structure. */
        base->EPRS = (base->EPRS & (~CAN_EPRS_ENPRESDIV_MASK)) | CAN_EPRS_ENPRESDIV(pConfig->preDivider);
        base->ENCBT = CAN_ENCBT_NRJW(pConfig->rJumpwidth) |
            CAN_ENCBT_NTSEG1(pConfig->phaseSeg1 + pConfig->propSeg + 1U) |
            CAN_ENCBT_NTSEG2(pConfig->phaseSeg2);
#else
        /* Enable Bit Timing register CBT, updating Timing Setting according to configuration structure. */
        base->CBT = CAN_CBT_BTF_MASK | CAN_CBT_EPRESDIV(pConfig->preDivider) | CAN_CBT_ERJW(pConfig->rJumpwidth) |
            CAN_CBT_EPSEG1(pConfig->phaseSeg1) | CAN_CBT_EPSEG2(pConfig->phaseSeg2) |
            CAN_CBT_EPROPSEG(pConfig->propSeg);
#endif
    }
    else
    {
        /* Cleaning previous Timing Setting. */
        base->CTRL1 &= ~(CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_RJW_MASK | CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
            CAN_CTRL1_PROPSEG_MASK);

        /* Updating Timing Setting according to configuration structure. */
        base->CTRL1 |= (CAN_CTRL1_PRESDIV(pConfig->preDivider) | CAN_CTRL1_RJW(pConfig->rJumpwidth) |
            CAN_CTRL1_PSEG1(pConfig->phaseSeg1) | CAN_CTRL1_PSEG2(pConfig->phaseSeg2) |
            CAN_CTRL1_PROPSEG(pConfig->propSeg));
    }
#else
    /* Cleaning previous Timing Setting. */
    base->CTRL1 &= ~(CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_RJW_MASK | CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
        CAN_CTRL1_PROPSEG_MASK);

    /* Updating Timing Setting according to configuration structure. */
    base->CTRL1 |= (CAN_CTRL1_PRESDIV(pConfig->preDivider) | CAN_CTRL1_RJW(pConfig->rJumpwidth) |
        CAN_CTRL1_PSEG1(pConfig->phaseSeg1) | CAN_CTRL1_PSEG2(pConfig->phaseSeg2) |
        CAN_CTRL1_PROPSEG(pConfig->propSeg));
#endif

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Sets the FlexCAN FD data phase timing characteristic.
 *
 * This function gives user settings to CAN FD data phase timing characteristic.
 * The function is for an experienced user. For less experienced users, call the FLEXCAN_GetDefaultConfig()
 * and get the default timing characteristicsthe, then call FLEXCAN_FDInit() and fill the
 * data phase bit rate field.
 *
 * note Calling FLEXCAN_SetFDTimingConfig() overrides the bit rate set
 * in FLEXCAN_FDInit().
 *
 * param base FlexCAN peripheral base address.
 * param pConfig Pointer to the timing configuration structure.
 */
void FLEXCAN_SetFDTimingConfig(volatile IMXCAN_REGISTERS *base, const flexcan_timing_config_t *pConfig)
{
    /* Assertion. */
    ASSERT(NULL != pConfig);

    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
    /* Enable extended Bit Timing register EDCBT. */
    base->CTRL2 |= CAN_CTRL2_BTE_MASK;

    base->EPRS = (base->EPRS & (~CAN_EPRS_EDPRESDIV_MASK)) | CAN_EPRS_EDPRESDIV(pConfig->fpreDivider);
    base->EDCBT = CAN_EDCBT_DRJW(pConfig->frJumpwidth) | CAN_EDCBT_DTSEG2(pConfig->fphaseSeg2) |
        CAN_EDCBT_DTSEG1((UINT32)pConfig->fphaseSeg1 + pConfig->fpropSeg);
#else
    /* Enable Bit Timing register FDCBT,*/
    base->CBT |= CAN_CBT_BTF_MASK;
    /* Cleaning previous Timing Setting. */
    base->FDCBT &= ~(CAN_FDCBT_FPRESDIV_MASK | CAN_FDCBT_FRJW_MASK | CAN_FDCBT_FPSEG1_MASK | CAN_FDCBT_FPSEG2_MASK |
        CAN_FDCBT_FPROPSEG_MASK);

    /* Updating Timing Setting according to configuration structure. */
    base->FDCBT |= (CAN_FDCBT_FPRESDIV(pConfig->fpreDivider) | CAN_FDCBT_FRJW(pConfig->frJumpwidth) |
        CAN_FDCBT_FPSEG1(pConfig->fphaseSeg1) | CAN_FDCBT_FPSEG2(pConfig->fphaseSeg2) |
        CAN_FDCBT_FPROPSEG(pConfig->fpropSeg));
#endif
    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}
#endif

/*
 * Sets the FlexCAN receive message buffer global mask.
 *
 * This function sets the global mask for the FlexCAN message buffer in a matching process.
 * The configuration is only effective when the Rx individual mask is disabled in the FLEXCAN_Init().
 *
 * param base FlexCAN peripheral base address.
 * param mask Rx Message Buffer Global Mask value.
 */
void FLEXCAN_SetRxMbGlobalMask(volatile IMXCAN_REGISTERS *base, UINT32 mask)
{
    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

    /* Setting Rx Message Buffer Global Mask value. */
    base->RXMGMASK = mask;
    base->RX14MASK = mask;
    base->RX15MASK = mask;

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}

/*
 * Sets the FlexCAN receive FIFO global mask.
 *
 * This function sets the global mask for FlexCAN FIFO in a matching process.
 *
 * param base FlexCAN peripheral base address.
 * param mask Rx Fifo Global Mask value.
 */
void FLEXCAN_SetRxFifoGlobalMask(volatile IMXCAN_REGISTERS *base, UINT32 mask)
{
    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

    /* Setting Rx FIFO Global Mask value. */
    base->RXFGMASK = mask;

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}

/*
 * Sets the FlexCAN receive individual mask.
 *
 * This function sets the individual mask for the FlexCAN matching process.
 * The configuration is only effective when the Rx individual mask is enabled in the FLEXCAN_Init().
 * If the Rx FIFO is disabled, the individual mask is applied to the corresponding Message Buffer.
 * If the Rx FIFO is enabled, the individual mask for Rx FIFO occupied Message Buffer is applied to
 * the Rx Filter with the same index. Note that only the first 32
 * individual masks can be used as the Rx FIFO filter mask.
 *
 * param base FlexCAN peripheral base address.
 * param maskIdx The Index of individual Mask.
 * param mask Rx Individual Mask value.
 */
void FLEXCAN_SetRxIndividualMask(volatile IMXCAN_REGISTERS *base, UINT8 maskIdx, UINT32 mask)
{
    ASSERT(maskIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));

    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

    /* Setting Rx Individual Mask value. */
    base->RXIMR[maskIdx] = mask;

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}

/*
 * Configures a FlexCAN transmit message buffer.
 *
 * This function aborts the previous transmission, cleans the Message Buffer, and
 * configures it as a Transmit Message Buffer.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param enable Enable/disable Tx Message Buffer.
 *               - TRUE: Enable Tx Message Buffer.
 *               - FALSE: Disable Tx Message Buffer.
 */
void FLEXCAN_SetTxMbConfig(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, BOOLEAN enable)
{
    /* Assertion. */
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    /* Inactivate Message Buffer. */
    if (enable)
    {
        base->MB[mbIdx].CS = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
    }
    else
    {
        base->MB[mbIdx].CS = 0;
    }

    /* Clean Message Buffer content. */
    base->MB[mbIdx].ID = 0x0;
    base->MB[mbIdx].WORD0 = 0x0;
    base->MB[mbIdx].WORD1 = 0x0;
}

/*
 * Calculates the segment values for a single bit time for classical CAN.
 *
 * This function use to calculates the Classical CAN segment values which will be set in CTRL1/CBT/ENCBT register.
 *
 * param bitRate The classical CAN bit rate in bps.
 * param base FlexCAN peripheral base address.
 * param tqNum Number of time quantas per bit, range in 8 ~ 25 when use CTRL1, range in 8 ~ 129 when use CBT, range in
 *             8 ~ 385 when use ENCBT. param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 */
static void FLEXCAN_GetSegments(volatile IMXCAN_REGISTERS *base,
    UINT32 bitRate,
    UINT32 tqNum,
    flexcan_timing_config_t *pTimingConfig)
{
    UINT32 ideal_sp;
    UINT32 seg1Max, proSegMax, sjwMAX;
    UINT32 seg1Temp;

    UNREFERENCED_PARAMETER(base);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base))
    {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
        /* Maximum value allowed in ENCBT register. */
        seg1Max = MAX_NTSEG2 + 1U;
        proSegMax = MAX_NTSEG1 - MAX_NTSEG2;
        sjwMAX = MAX_NRJW + 1U;
#else
        /* Maximum value allowed in CBT register. */
        seg1Max = MAX_EPSEG1 + 1U;
        proSegMax = MAX_EPROPSEG + 1U;
        sjwMAX = MAX_ERJW + 1U;
#endif
    }
    else
    {
        /* Maximum value allowed in CTRL1 register. */
        seg1Max = MAX_PSEG1 + 1U;
        proSegMax = MAX_PROPSEG + 1U;
        sjwMAX = MAX_RJW + 1U;
    }
#else
    /* Maximum value allowed in CTRL1 register. */
    seg1Max = MAX_PSEG1 + 1U;
    proSegMax = MAX_PROPSEG + 1U;
    sjwMAX = MAX_RJW + 1U;
#endif

    /* Try to find the ideal sample point, according to CiA 301 doc.*/
    if (bitRate == 1000000U)
    {
        ideal_sp = IDEAL_SP_LOW;
    }
    else if (bitRate >= 800000U)
    {
        ideal_sp = IDEAL_SP_MID;
    }
    else
    {
        ideal_sp = IDEAL_SP_HIGH;
    }
    /* Calculates phaseSeg2. */
    pTimingConfig->phaseSeg2 = (UINT8)(tqNum - (tqNum * ideal_sp) / (UINT32)IDEAL_SP_FACTOR);
    if (pTimingConfig->phaseSeg2 < MIN_TIME_SEGMENT2)
    {
        pTimingConfig->phaseSeg2 = MIN_TIME_SEGMENT2;
    }
    else if (pTimingConfig->phaseSeg2 > seg1Max)
    {
        pTimingConfig->phaseSeg2 = (UINT8)seg1Max;
    }
    else
    {
        ; /* Intentional empty */
    }

    /* Calculates phaseSeg1 and propSeg. */
    seg1Temp = tqNum - pTimingConfig->phaseSeg2 - 1U;

    if (seg1Temp > (seg1Max + proSegMax))
    {
        pTimingConfig->phaseSeg2 += (UINT8)(seg1Temp - seg1Max - proSegMax);
        pTimingConfig->propSeg = (UINT8)proSegMax;
        pTimingConfig->phaseSeg1 = (UINT8)seg1Max;
    }
    else if (seg1Temp > proSegMax)
    {
        pTimingConfig->propSeg = (UINT8)proSegMax;
        pTimingConfig->phaseSeg1 = (UINT8)(seg1Temp - proSegMax);
    }
    else
    {
        pTimingConfig->propSeg = (UINT8)(seg1Temp - 1U);
        pTimingConfig->phaseSeg1 = 1U;
    }

    /* try to make phaseSeg1 equal to phaseSeg2*/
    if (pTimingConfig->phaseSeg1 < pTimingConfig->phaseSeg2)
    {
        seg1Temp =
            ((pTimingConfig->phaseSeg2 - pTimingConfig->phaseSeg1) > ((UINT8)proSegMax - pTimingConfig->propSeg)) ?
            (proSegMax - pTimingConfig->propSeg) :
            (pTimingConfig->phaseSeg2 - pTimingConfig->phaseSeg1);
        pTimingConfig->propSeg -= (UINT8)seg1Temp;
        pTimingConfig->phaseSeg1 += (UINT8)seg1Temp;
    }
    else
    {
        seg1Temp =
            ((pTimingConfig->phaseSeg1 - pTimingConfig->phaseSeg2) > ((UINT8)proSegMax - pTimingConfig->propSeg)) ?
            (proSegMax - pTimingConfig->propSeg) :
            (pTimingConfig->phaseSeg1 - pTimingConfig->phaseSeg2);
        pTimingConfig->propSeg += (UINT8)seg1Temp;
        pTimingConfig->phaseSeg1 -= (UINT8)seg1Temp;
    }

    /* rJumpwidth (sjw) is the minimum value of phaseSeg1 and phaseSeg2. */
    pTimingConfig->rJumpwidth =
        (pTimingConfig->phaseSeg1 > pTimingConfig->phaseSeg2) ? pTimingConfig->phaseSeg2 : pTimingConfig->phaseSeg1;
    if (pTimingConfig->rJumpwidth > sjwMAX)
    {
        pTimingConfig->rJumpwidth = (UINT8)sjwMAX;
    }

    pTimingConfig->phaseSeg1 -= 1U;
    pTimingConfig->phaseSeg2 -= 1U;
    pTimingConfig->propSeg -= 1U;
    pTimingConfig->rJumpwidth -= 1U;
}

/*
 * Calculates the improved timing values by specific bit Rates for classical CAN.
 *
 * This function use to calculates the Classical CAN timing values according to the given bit rate. The Calculated
 * timing values will be set in CTRL1/CBT/ENCBT register. The calculation is based on the recommendation of the CiA 301
 * v4.2.0 and previous version document.
 *
 * param base FlexCAN peripheral base address.
 * param bitRate  The classical CAN speed in bps defined by user, should be less than or equal to 1Mbps.
 * param sourceClock_Hz The Source clock frequency in Hz.
 * param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 *
 * return TRUE if timing configuration found, FALSE if failed to find configuration.
 */
BOOLEAN FLEXCAN_CalculateImprovedTimingValues(volatile IMXCAN_REGISTERS *base,
    UINT32 bitRate,
    UINT32 sourceClock_Hz,
    flexcan_timing_config_t *pTimingConfig)
{
    /* Observe bit rate maximums. */
    ASSERT(bitRate <= MAX_CAN_BITRATE);

    UINT32 clk;
    UINT32 tqNum, tqMin, pdivMAX;
    UINT32 spTemp = 1000U;
    flexcan_timing_config_t configTemp = { 0 };
    BOOLEAN fgRet = FALSE;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base))
    {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
        /*  Auto Improved Protocal timing for ENCBT. */
        tqNum = ENCBT_MAX_TIME_QUANTA;
        tqMin = ENCBT_MIN_TIME_QUANTA;
        pdivMAX = MAX_ENPRESDIV;
#else
        /*  Auto Improved Protocal timing for CBT. */
        tqNum = CBT_MAX_TIME_QUANTA;
        tqMin = CBT_MIN_TIME_QUANTA;
        pdivMAX = MAX_PRESDIV;
#endif
    }
    else
    {
        /*  Auto Improved Protocal timing for CTRL1. */
        tqNum = CTRL1_MAX_TIME_QUANTA;
        tqMin = CTRL1_MIN_TIME_QUANTA;
        pdivMAX = MAX_PRESDIV;
    }
#else
    /*  Auto Improved Protocal timing for CTRL1. */
    tqNum = CTRL1_MAX_TIME_QUANTA;
    tqMin = CTRL1_MIN_TIME_QUANTA;
    pdivMAX = MAX_PRESDIV;
#endif
    do
    {
        clk = bitRate * tqNum;
        if (clk > sourceClock_Hz)
        {
            continue; /* tqNum too large, clk has been exceed sourceClock_Hz. */
        }

        if ((sourceClock_Hz / clk * clk) != sourceClock_Hz)
        {
            continue; /* Non-supporting: the frequency of clock source is not divisible by target bit rate, the user
                      should change a divisible bit rate. */
        }

        configTemp.preDivider = (UINT16)(sourceClock_Hz / clk) - 1U;
        if (configTemp.preDivider > pdivMAX)
        {
            break; /* The frequency of source clock is too large or the bit rate is too small, the pre-divider could
                      not handle it. */
        }

        /* Calculates the best timing configuration under current tqNum. */
        FLEXCAN_GetSegments(base, bitRate, tqNum, &configTemp);
        /* Determine whether the calculated timing configuration can get the optimal sampling point. */
        if (((((UINT32)configTemp.phaseSeg2 + 1U) * 1000U) / tqNum) < spTemp)
        {
            spTemp = (((UINT32)configTemp.phaseSeg2 + 1U) * 1000U) / tqNum;
            (void)memcpy(pTimingConfig, &configTemp, sizeof(configTemp));
        }
        fgRet = TRUE;
    } while (--tqNum >= tqMin);

    return fgRet;
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Get Mailbox offset number by dword.
 *
 * This function gets the offset number of the specified mailbox.
 * Mailbox is not consecutive between memory regions when payload is not 8 bytes
 * so need to calculate the specified mailbox address.
 * For example, in the first memory region, MB[0].CS address is 0x4002_4080. For 32 bytes
 * payload frame, the second mailbox is ((1/12)*512 + 1%12*40)/4 = 10, meaning 10 dword
 * after the 0x4002_4080, which is actually the address of mailbox MB[1].CS.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx Mailbox index.
 */
static UINT32 FLEXCAN_GetFDMailboxOffset(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx)
{
    UINT32 offset = 0;
    UINT32 dataSize = (base->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT;
    switch (dataSize)
    {
    case (UINT32)kFLEXCAN_8BperMB:
        offset = (((UINT32)mbIdx / 32U) * 512U + ((UINT32)mbIdx % 32U) * 16U);
        break;
    case (UINT32)kFLEXCAN_16BperMB:
        offset = (((UINT32)mbIdx / 21U) * 512U + ((UINT32)mbIdx % 21U) * 24U);
        break;
    case (UINT32)kFLEXCAN_32BperMB:
        offset = (((UINT32)mbIdx / 12U) * 512U + ((UINT32)mbIdx % 12U) * 40U);
        break;
    case (UINT32)kFLEXCAN_64BperMB:
        offset = (((UINT32)mbIdx / 7U) * 512U + ((UINT32)mbIdx % 7U) * 72U);
        break;
    default:
        /* All the cases have been listed above, the default clause should not be reached. */
        ASSERT(FALSE);
        break;
    }
    /* To get the dword aligned offset, need to divide by 4. */
    offset = offset / 4U;
    return offset;
}

/*
 * Calculates the segment values for a single bit time for CAN FD data phase.
 *
 * This function use to calculates the CAN FD data phase segment values which will be set in CFDCBT/EDCBT
 * register.
 *
 * param bitRateFD CAN FD data phase bit rate.
 * param tqNum Number of time quanta per bit
 * param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 */
static void FLEXCAN_FDGetSegments(UINT32 bitRateFD, UINT32 tqNum, flexcan_timing_config_t *pTimingConfig)
{
    UINT32 ideal_sp;
    UINT32 seg1Max, proSegMax, sjwMAX;
    UINT32 seg1Temp;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
    /* Maximum value allowed in ENCBT register. */
    seg1Max = MAX_DTSEG1;
    proSegMax = 0U;
    sjwMAX = MAX_DRJW + 1U;
#else
    /* Maximum value allowed in CBT register. */
    seg1Max = MAX_FPSEG1 + 1U;
    proSegMax = MAX_FPROPSEG;
    sjwMAX = MAX_FRJW + 1U;
#endif

    /* According to CiA doc 1301 v1.0.0, which specified data phase sample point postion for CAN FD at 80 MHz. */
    if (bitRateFD <= 1000000U)
    {
        ideal_sp = IDEAL_DATA_SP_1;
    }
    else if (bitRateFD <= 2000000U)
    {
        ideal_sp = IDEAL_DATA_SP_2;
    }
    else if (bitRateFD <= 4000000U)
    {
        ideal_sp = IDEAL_DATA_SP_3;
    }
    else
    {
        ideal_sp = IDEAL_DATA_SP_4;
    }

    /* Calculates fphaseSeg2. */
    pTimingConfig->fphaseSeg2 = (UINT8)(tqNum - (tqNum * ideal_sp) / (UINT32)IDEAL_SP_FACTOR);
    if (pTimingConfig->fphaseSeg2 < MIN_TIME_SEGMENT2)
    {
        pTimingConfig->fphaseSeg2 = MIN_TIME_SEGMENT2;
    }
    else if (pTimingConfig->fphaseSeg2 > seg1Max)
    {
        pTimingConfig->fphaseSeg2 = (UINT8)seg1Max;
    }
    else
    {
        ; /* Intentional empty */
    }

    /* Calculates phaseSeg1 and propSeg. */
    seg1Temp = tqNum - pTimingConfig->fphaseSeg2 - 1U;

    if (seg1Temp > (seg1Max + proSegMax))
    {
        pTimingConfig->fphaseSeg2 += (UINT8)(seg1Temp - seg1Max - proSegMax);
        pTimingConfig->fpropSeg = (UINT8)proSegMax;
        pTimingConfig->fphaseSeg1 = (UINT8)seg1Max;
    }
    else if (seg1Temp > proSegMax)
    {
        pTimingConfig->fpropSeg = (UINT8)proSegMax;
        pTimingConfig->fphaseSeg1 = (UINT8)(seg1Temp - proSegMax);
    }
    else
    {
        pTimingConfig->fpropSeg = (UINT8)(seg1Temp - 1U);
        pTimingConfig->fphaseSeg1 = 1U;
    }

    /* try to make phaseSeg1 equal to phaseSeg2*/
    if (pTimingConfig->fphaseSeg1 < pTimingConfig->fphaseSeg2)
    {
        seg1Temp =
            ((pTimingConfig->fphaseSeg2 - pTimingConfig->fphaseSeg1) > ((UINT8)proSegMax - pTimingConfig->fpropSeg)) ?
            (proSegMax - pTimingConfig->fpropSeg) :
            (pTimingConfig->fphaseSeg2 - pTimingConfig->fphaseSeg1);
        pTimingConfig->fpropSeg -= (UINT8)seg1Temp;
        pTimingConfig->fphaseSeg1 += (UINT8)seg1Temp;
    }
    else
    {
        seg1Temp =
            ((pTimingConfig->fphaseSeg1 - pTimingConfig->fphaseSeg2) > ((UINT8)proSegMax - pTimingConfig->fpropSeg)) ?
            (proSegMax - pTimingConfig->fpropSeg) :
            (pTimingConfig->fphaseSeg1 - pTimingConfig->fphaseSeg2);
        pTimingConfig->fpropSeg += (UINT8)seg1Temp;
        pTimingConfig->fphaseSeg1 -= (UINT8)seg1Temp;
    }

    /* rJumpwidth (sjw) is the minimum value of phaseSeg1 and phaseSeg2. */
    pTimingConfig->frJumpwidth =
        (pTimingConfig->fphaseSeg1 > pTimingConfig->fphaseSeg2) ? pTimingConfig->fphaseSeg2 : pTimingConfig->fphaseSeg1;
    if (pTimingConfig->frJumpwidth > sjwMAX)
    {
        pTimingConfig->frJumpwidth = (UINT8)sjwMAX;
    }

    pTimingConfig->fphaseSeg1 -= 1U;
    pTimingConfig->fphaseSeg2 -= 1U;
    pTimingConfig->frJumpwidth -= 1U;
}

/*
 * Calculates the improved timing values by specific bit rate for CAN FD nominal phase.
 *
 * This function use to calculates the CAN FD nominal phase timing values according to the given nominal phase bit rate.
 * The Calculated timing values will be set in CBT/ENCBT registers. The calculation is based on the recommendation of
 * the CiA 1301 v1.0.0 document.
 *
 * param bitRate  The CAN FD nominal phase speed in bps defined by user, should be less than or equal to 1Mbps.
 * param sourceClock_Hz The Source clock frequency in Hz.
 * param pTimingConfig Pointer to the FlexCAN timing configuration structure.
 *
 * return TRUE if timing configuration found, FALSE if failed to find configuration.
 */
static BOOLEAN FLEXCAN_CalculateImprovedNominalTimingValues(UINT32 bitRate,
    UINT32 sourceClock_Hz,
    flexcan_timing_config_t *pTimingConfig)
{
    /* Observe bit rate maximums. */
    ASSERT(bitRate <= MAX_CAN_BITRATE);

    UINT32 clk;
    UINT32 tqNum, tqMin, pdivMAX, seg1Max, proSegMax, sjwMAX, seg1Temp;
    UINT32 spTemp = 1000U;
    flexcan_timing_config_t configTemp = { 0 };
    BOOLEAN fgRet = FALSE;

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
    /*  Auto Improved Protocal timing for ENCBT. */
    tqNum = ENCBT_MAX_TIME_QUANTA;
    tqMin = ENCBT_MIN_TIME_QUANTA;
    pdivMAX = MAX_ENPRESDIV;
    seg1Max = MAX_NTSEG2 + 1U;
    proSegMax = MAX_NTSEG1 - MAX_NTSEG2;
    sjwMAX = MAX_NRJW + 1U;
#else
    /*  Auto Improved Protocal timing for CBT. */
    tqNum = CBT_MAX_TIME_QUANTA;
    tqMin = CBT_MIN_TIME_QUANTA;
    pdivMAX = MAX_PRESDIV;
    seg1Max = MAX_EPSEG1 + 1U;
    proSegMax = MAX_EPROPSEG + 1U;
    sjwMAX = MAX_ERJW + 1U;
#endif

    do
    {
        clk = bitRate * tqNum;
        if (clk > sourceClock_Hz)
        {
            continue; /* tqNum too large, clk has been exceed sourceClock_Hz. */
        }

        if ((sourceClock_Hz / clk * clk) != sourceClock_Hz)
        {
            continue; /* Non-supporting: the frequency of clock source is not divisible by target bit rate, the user
                      should change a divisible bit rate. */
        }

        configTemp.preDivider = (UINT16)(sourceClock_Hz / clk) - 1U;
        if (configTemp.preDivider > pdivMAX)
        {
            break; /* The frequency of source clock is too large or the bit rate is too small, the pre-divider could
                      not handle it. */
        }

        /* Calculates the best timing configuration under current tqNum. */
        configTemp.phaseSeg2 = (UINT8)(tqNum - (tqNum * IDEAL_NOMINAL_SP) / (UINT32)IDEAL_SP_FACTOR);
        if (configTemp.phaseSeg2 < MIN_TIME_SEGMENT2)
        {
            configTemp.phaseSeg2 = MIN_TIME_SEGMENT2;
        }
        else if (configTemp.phaseSeg2 > seg1Max)
        {
            configTemp.phaseSeg2 = (UINT8)seg1Max;
        }
        else
        {
            ; /* Intentional empty */
        }

        /* Calculates phaseSeg1 and propSeg. */
        seg1Temp = tqNum - configTemp.phaseSeg2 - 1U;

        if (seg1Temp > (seg1Max + proSegMax))
        {
            configTemp.phaseSeg2 += (UINT8)(seg1Temp - seg1Max - proSegMax);
            configTemp.propSeg = (UINT8)proSegMax;
            configTemp.phaseSeg1 = (UINT8)seg1Max;
        }
        else if (seg1Temp > proSegMax)
        {
            configTemp.propSeg = (UINT8)proSegMax;
            configTemp.phaseSeg1 = (UINT8)(seg1Temp - proSegMax);
        }
        else
        {
            configTemp.propSeg = (UINT8)(seg1Temp - 1U);
            configTemp.phaseSeg1 = 1U;
        }

        /* try to make phaseSeg1 equal to phaseSeg2*/
        if (configTemp.phaseSeg1 < configTemp.phaseSeg2)
        {
            seg1Temp = ((configTemp.phaseSeg2 - configTemp.phaseSeg1) > ((UINT8)proSegMax - configTemp.propSeg)) ?
                (proSegMax - configTemp.propSeg) :
                (configTemp.phaseSeg2 - configTemp.phaseSeg1);
            configTemp.propSeg -= (UINT8)seg1Temp;
            configTemp.phaseSeg1 += (UINT8)seg1Temp;
        }
        else
        {
            seg1Temp = ((configTemp.phaseSeg1 - configTemp.phaseSeg2) > ((UINT8)proSegMax - configTemp.propSeg)) ?
                (proSegMax - configTemp.propSeg) :
                (configTemp.phaseSeg1 - configTemp.phaseSeg2);
            configTemp.propSeg += (UINT8)seg1Temp;
            configTemp.phaseSeg1 -= (UINT8)seg1Temp;
        }

        /* rJumpwidth (sjw) is the minimum value of phaseSeg1 and phaseSeg2. */
        configTemp.rJumpwidth =
            (configTemp.phaseSeg1 > configTemp.phaseSeg2) ? configTemp.phaseSeg2 : configTemp.phaseSeg1;
        if (configTemp.rJumpwidth > sjwMAX)
        {
            configTemp.rJumpwidth = (UINT8)sjwMAX;
        }
        configTemp.phaseSeg1 -= 1U;
        configTemp.phaseSeg2 -= 1U;
        configTemp.propSeg -= 1U;
        configTemp.rJumpwidth -= 1U;

        if (((((UINT32)configTemp.phaseSeg2 + 1U) * 1000U) / tqNum) < spTemp)
        {
            spTemp = (((UINT32)configTemp.phaseSeg2 + 1U) * 1000U) / tqNum;
            (void)memcpy(pTimingConfig, &configTemp, sizeof(configTemp));
        }
        fgRet = TRUE;
    } while (--tqNum >= tqMin);

    return fgRet;
}

/*
 * Calculates the improved timing values by specific bit rates for CAN FD.
 *
 * This function use to calculates the CAN FD timing values according to the given nominal phase bit rate and data phase
 * bit rate. The Calculated timing values will be set in CBT/ENCBT and FDCBT/EDCBT registers. The calculation is based
 * on the recommendation of the CiA 1301 v1.0.0 document.
 *
 * param bitRate  The CAN FD nominal phase speed in bps defined by user.
 * param bitRateFD  The CAN FD data phase speed in bps defined by user. Equal to bitRate means disable bit rate
 * switching. param sourceClock_Hz The Source clock frequency in Hz. param pTimingConfig Pointer to the FlexCAN timing
 * configuration structure.
 *
 * return TRUE if timing configuration found, FALSE if failed to find configuration
 */
BOOLEAN FLEXCAN_FDCalculateImprovedTimingValues(volatile IMXCAN_REGISTERS *base,
    UINT32 bitRate,
    UINT32 bitRateFD,
    UINT32 sourceClock_Hz,
    flexcan_timing_config_t *pTimingConfig)
{
    UNREFERENCED_PARAMETER(base);

    /* Observe bit rate maximums */
    ASSERT(bitRate <= MAX_CANFD_BITRATE);
    ASSERT(bitRateFD <= MAX_CANFD_BITRATE);
    /* Data phase bit rate need greater or equal to nominal phase bit rate. */
    ASSERT(bitRate <= bitRateFD);

    UINT32 clk;
    UINT16 preDividerTemp;
    UINT32 tqMax, tqMin, pdivMAX, tqTemp;
    BOOLEAN fgRet = FALSE;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
    /*  Auto Improved Protocal timing for EDCBT. */
    tqMax = EDCBT_MAX_TIME_QUANTA;
    tqMin = EDCBT_MIN_TIME_QUANTA;
    pdivMAX = MAX_EDPRESDIV;
#else
    /*  Auto Improved Protocal timing for FDCBT. */
    tqMax = FDCBT_MAX_TIME_QUANTA;
    tqMin = FDCBT_MIN_TIME_QUANTA;
    pdivMAX = MAX_FPRESDIV;
#endif

    if (bitRate != bitRateFD)
    {
        /* To minimize errors when processing FD frames, try to get the same bit rate prescaler value for nominal phase
           and data phase. */
        preDividerTemp = 1U;
        while (FLEXCAN_CalculateImprovedNominalTimingValues(bitRate, sourceClock_Hz / preDividerTemp, pTimingConfig))
        {
            tqTemp = tqMax;
            do
            {
                clk = bitRateFD * tqTemp;
                if (clk > sourceClock_Hz)
                {
                    continue; /* tqTemp too large, clk x tqTemp has been exceed sourceClock_Hz. */
                }

                if ((sourceClock_Hz / clk * clk) != sourceClock_Hz)
                {
                    continue; /*  the frequency of clock source is not divisible by target bit rate. */
                }

                pTimingConfig->fpreDivider = (UINT16)(sourceClock_Hz / clk - 1U);

                if (pTimingConfig->fpreDivider > pdivMAX)
                {
                    break; /* The frequency of source clock is too large or the bit rate is too small, the pre-divider
                              could not handle it. */
                }

                if (pTimingConfig->fpreDivider != ((pTimingConfig->preDivider + 1U) * preDividerTemp - 1U))
                {
                    continue;
                }
                else
                {
                    /* Calculates the best data phase timing configuration. */
                    FLEXCAN_FDGetSegments(bitRateFD, tqTemp, pTimingConfig);
                    fgRet = TRUE;
                    break;
                }
            } while (--tqTemp >= tqMin);

            if (fgRet)
            {
                /* Find same bit rate prescaler (BRP) configuration in both nominal and data bit timing configurations.
                 */
                pTimingConfig->preDivider = (pTimingConfig->preDivider + 1U) * preDividerTemp - 1U;
                break;
            }
            else if (pTimingConfig->fpreDivider != 0U)
            {
                /* Can't find same data bit rate prescaler (BRP) configuration under current nominal phase bit rate
                   prescaler, double the nominal phase bit rate prescaler and recalculate. */
                preDividerTemp++;
            }
            else
            {
                break;
            }
        }
    }
    else
    {
        if (FLEXCAN_CalculateImprovedNominalTimingValues(bitRate, sourceClock_Hz, pTimingConfig))
        {
            /* No need data phase timing configuration, data phase rate equal to nominal phase rate, user don't use Brs
               feature. */
            fgRet = TRUE;
        }
    }
    return fgRet;
}

/*
 * Configures a FlexCAN transmit message buffer.
 *
 * This function aborts the previous transmission, cleans the Message Buffer, and
 * configures it as a Transmit Message Buffer.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param enable Enable/disable Tx Message Buffer.
 *               - TRUE: Enable Tx Message Buffer.
 *               - FALSE: Disable Tx Message Buffer.
 */
void FLEXCAN_SetFDTxMbConfig(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, BOOLEAN enable)
{
    /* Assertion. */
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    UINT8 cnt = 0;
    UINT8 payload_dword = 1;
    UINT32 dataSize;
    dataSize = (base->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT;
    volatile UINT32 *mbAddr = &(base->MB[0].CS);
    UINT32 offset = FLEXCAN_GetFDMailboxOffset(base, mbIdx);
#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
    UINT32 availoffset = FLEXCAN_GetFDMailboxOffset(base, FLEXCAN_GetFirstValidMb(base));
#endif

    /* Inactivate Message Buffer. */
    if (enable)
    {
        /* Inactivate by writing CS. */
        mbAddr[offset] = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
    }
    else
    {
        mbAddr[offset] = 0x0;
    }

    /* Calculate the DWORD number, dataSize 0/1/2/3 corresponds to 8/16/32/64
       Bytes payload. */
    for (cnt = 0; cnt < (dataSize + 1U); cnt++)
    {
        payload_dword *= 2U;
    }

    /* Clean ID. */
    mbAddr[offset + 1U] = 0x0U;
    /* Clean Message Buffer content, DWORD by DWORD. */
    for (cnt = 0; cnt < payload_dword; cnt++)
    {
        mbAddr[offset + 2U + cnt] = 0x0U;
    }

#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
    mbAddr[availoffset] = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
#endif
}
#endif /* FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE */

/*
 * Configures a FlexCAN Receive Message Buffer.
 *
 * This function cleans a FlexCAN build-in Message Buffer and configures it
 * as a Receive Message Buffer.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param pRxMbConfig Pointer to the FlexCAN Message Buffer configuration structure.
 * param enable Enable/disable Rx Message Buffer.
 *               - TRUE: Enable Rx Message Buffer.
 *               - FALSE: Disable Rx Message Buffer.
 */
void FLEXCAN_SetRxMbConfig(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, BOOLEAN enable)
{
    /* Assertion. */
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    ASSERT(((NULL != pRxMbConfig) || (FALSE == enable)));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    UINT32 cs_temp = 0;

    /* Inactivate Message Buffer. */
    base->MB[mbIdx].CS = 0;

    /* Clean Message Buffer content. */
    base->MB[mbIdx].ID = 0x0;
    base->MB[mbIdx].WORD0 = 0x0;
    base->MB[mbIdx].WORD1 = 0x0;

    if (enable)
    {
        /* Setup Message Buffer ID. */
        base->MB[mbIdx].ID = pRxMbConfig->id;

        /* Setup Message Buffer format. */
        if (kFLEXCAN_FrameFormatExtend == pRxMbConfig->format)
        {
            cs_temp |= CAN_CS_IDE_MASK;
        }

        /* Setup Message Buffer type. */
        if (kFLEXCAN_FrameTypeRemote == pRxMbConfig->type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }

        /* Activate Rx Message Buffer. */
        cs_temp |= CAN_CS_CODE(kFLEXCAN_RxMbEmpty);
        base->MB[mbIdx].CS = cs_temp;
    }
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Configures a FlexCAN Receive Message Buffer.
 *
 * This function cleans a FlexCAN build-in Message Buffer and configures it
 * as a Receive Message Buffer.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param pRxMbConfig Pointer to the FlexCAN Message Buffer configuration structure.
 * param enable Enable/disable Rx Message Buffer.
 *               - TRUE: Enable Rx Message Buffer.
 *               - FALSE: Disable Rx Message Buffer.
 */
void FLEXCAN_SetFDRxMbConfig(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, BOOLEAN enable)
{
    /* Assertion. */
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    ASSERT(((NULL != pRxMbConfig) || (FALSE == enable)));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    UINT32 cs_temp = 0;
    UINT8 cnt = 0;
    volatile UINT32 *mbAddr = &(base->MB[0].CS);
    UINT32 offset = FLEXCAN_GetFDMailboxOffset(base, mbIdx);
    UINT8 payload_dword;
    UINT32 dataSize = (base->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT;

    /* Inactivate Message Buffer. */
    mbAddr[offset] = 0U;

    /* Clean Message Buffer content. */
    mbAddr[offset + 1U] = 0U;
    /* Calculate the DWORD number, dataSize 0/1/2/3 corresponds to 8/16/32/64
       Bytes payload. */
    payload_dword = (2U << dataSize);
    for (cnt = 0; cnt < payload_dword; cnt++)
    {
        mbAddr[offset + 2U + cnt] = 0x0;
    }

    if (enable)
    {
        /* Setup Message Buffer ID. */
        mbAddr[offset + 1U] = pRxMbConfig->id;

        /* Setup Message Buffer format. */
        if (kFLEXCAN_FrameFormatExtend == pRxMbConfig->format)
        {
            cs_temp |= CAN_CS_IDE_MASK;
        }

        /* Setup Message Buffer type. */
        if (kFLEXCAN_FrameTypeRemote == pRxMbConfig->type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }

        /* Activate Rx Message Buffer. */
        cs_temp |= CAN_CS_CODE(kFLEXCAN_RxMbEmpty);
        mbAddr[offset] = cs_temp;
    }
}
#endif

/*
 * Configures the FlexCAN Legacy Rx FIFO.
 *
 * This function configures the FlexCAN Rx FIFO with given configuration.
 * note Legacy Rx FIFO only can receive classic CAN message.
 *
 * param base FlexCAN peripheral base address.
 * param pRxFifoConfig Pointer to the FlexCAN Legacy Rx FIFO configuration structure. Can be NULL when enable parameter
 *                      is FALSE.
 * param enable Enable/disable Legacy Rx FIFO.
 *              - TRUE: Enable Legacy Rx FIFO.
 *              - FALSE: Disable Legacy Rx FIFO.
 */
void FLEXCAN_SetRxFifoConfig(volatile IMXCAN_REGISTERS *base, const flexcan_rx_fifo_config_t *pRxFifoConfig, BOOLEAN enable)
{
    /* Assertion. */
    ASSERT((NULL != pRxFifoConfig) || (FALSE == enable));

    volatile UINT32 *mbAddr;
    UINT8 i, j, k, rffn = 0, numMbOccupy;
    UINT32 setup_mb = 0;

    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

    if (enable)
    {
        ASSERT(pRxFifoConfig->idFilterNum <= 128U);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
        /* Legacy Rx FIFO and Enhanced Rx FIFO cannot be enabled at the same time. */
        ASSERT((base->ERFCR & CAN_ERFCR_ERFEN_MASK) == 0U);
#endif

        /* Get the setup_mb value. */
        setup_mb = (UINT8)((base->MCR & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT);
        setup_mb = (setup_mb < (UINT32)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base)) ?
            setup_mb :
            (UINT32)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base);

        /* Determine RFFN value. */
        for (i = 0; i <= 0xFU; i++)
        {
            if ((8U * (i + 1U)) >= pRxFifoConfig->idFilterNum)
            {
                rffn = i;
                ASSERT(((setup_mb - 8U) - (2U * rffn)) > 0U);

                base->CTRL2 = (base->CTRL2 & ~CAN_CTRL2_RFFN_MASK) | CAN_CTRL2_RFFN(rffn);
                break;
            }
        }

        /* caculate the Number of Mailboxes occupied by RX Legacy FIFO and the filter. */
        numMbOccupy = 6U + (rffn + 1U) * 2U;

        /* Copy ID filter table to Message Buffer Region (Fix MISRA_C-2012 Rule 18.1). */
        j = 0U;
        for (i = 6U; i < numMbOccupy; i++)
        {
            /* Get address for current mail box.  */
            mbAddr = &(base->MB[i].CS);

            /* One Mail box contain 4U DWORD registers. */
            for (k = 0; k < 4U; k++)
            {
                /* Fill all valid filter in the mail box occupied by filter.
                 * Disable unused Rx FIFO Filter, the other rest of register in the last Mail box occupied by fiter set
                 * as 0xffffffff.
                 */
                mbAddr[k] = (j < pRxFifoConfig->idFilterNum) ? (pRxFifoConfig->idFilterTable[j]) : 0xFFFFFFFFU;

                /* Try to fill next filter in current Mail Box.  */
                j++;
            }
        }

        /* Setup ID Fitlter Type. */
        switch (pRxFifoConfig->idFilterType)
        {
        case kFLEXCAN_RxFifoFilterTypeA:
            base->MCR = (base->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x0);
            break;
        case kFLEXCAN_RxFifoFilterTypeB:
            base->MCR = (base->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x1);
            break;
        case kFLEXCAN_RxFifoFilterTypeC:
            base->MCR = (base->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x2);
            break;
        case kFLEXCAN_RxFifoFilterTypeD:
            /* All frames rejected. */
            base->MCR = (base->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x3);
            break;
        default:
            /* All the cases have been listed above, the default clause should not be reached. */
            ASSERT(FALSE);
            break;
        }

        /* Setting Message Reception Priority. */
        base->CTRL2 = (pRxFifoConfig->priority == kFLEXCAN_RxFifoPrioHigh) ? (base->CTRL2 & ~CAN_CTRL2_MRP_MASK) :
            (base->CTRL2 | CAN_CTRL2_MRP_MASK);

        /* Enable Rx Message FIFO. */
        base->MCR |= CAN_MCR_RFEN_MASK;
    }
    else
    {
        rffn = (UINT8)((base->CTRL2 & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* caculate the Number of Mailboxes occupied by RX Legacy FIFO and the filter. */
        numMbOccupy = 6U + (rffn + 1U) * 2U;

        /* Disable Rx Message FIFO. */
        base->MCR &= ~CAN_MCR_RFEN_MASK;

        /* Clean MB0 ~ MB5 and all MB occupied by ID filters (Fix MISRA_C-2012 Rule 18.1). */

        for (i = 0; i < numMbOccupy; i++)
        {
            FLEXCAN_SetRxMbConfig(base, i, NULL, FALSE);
        }
    }

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Configures the FlexCAN Enhanced Rx FIFO.
 *
 * This function configures the Enhanced Rx FIFO with given configuration.
 * note  Enhanced Rx FIFO support receive classic CAN or CAN FD messages, Legacy Rx FIFO and Enhanced Rx FIFO
 *       cannot be enabled at the same time.
 *
 * param base    FlexCAN peripheral base address.
 * param pConfig Pointer to the FlexCAN Enhanced Rx FIFO configuration structure. Can be NULL when enable parameter
 *               is FALSE.
 * param enable  Enable/disable Enhanced Rx FIFO.
 *               - TRUE: Enable Enhanced Rx FIFO.
 *               - FALSE: Disable Enhanced Rx FIFO.
 */
void FLEXCAN_SetEnhancedRxFifoConfig(volatile IMXCAN_REGISTERS *base, const flexcan_enhanced_rx_fifo_config_t *pConfig, BOOLEAN enable)
{
    /* Assertion. */
    ASSERT((NULL != pConfig) || (FALSE == enable));
    UINT32 i;
    /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(base);

    if (enable)
    {
        /* Each pair of filter elements occupies 2 words and can consist of one extended ID filter element or two
         * standard ID filter elements. */
        ASSERT((pConfig->idFilterPairNum < (UINT32)FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO_FILTER_MAX_NUMBER) &&
            (pConfig->extendIdFilterNum <= (pConfig->idFilterPairNum + 1U)));

        /* The Enhanced Rx FIFO Watermark cannot be greater than the enhanced Rx FIFO size. */
        ASSERT(pConfig->fifoWatermark < (UINT32)FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO_SIZE);

        /* Legacy Rx FIFO and Enhanced Rx FIFO cannot be enabled at the same time. */
        ASSERT((base->MCR & CAN_MCR_RFEN_MASK) == 0U);

        /* Enable Enhanced Rx FIFO. */
        base->ERFCR = CAN_ERFCR_ERFEN_MASK;
        /* Reset Enhanced Rx FIFO engine and clear flags. */
        base->ERFSR |= CAN_ERFSR_ERFCLR_MASK | CAN_ERFSR_ERFUFW_MASK | CAN_ERFSR_ERFOVF_MASK | CAN_ERFSR_ERFWMI_MASK |
            CAN_ERFSR_ERFDA_MASK;
        /* Setting Enhanced Rx FIFO. */
        base->ERFCR |= CAN_ERFCR_DMALW(pConfig->dmaPerReadLength) | CAN_ERFCR_NEXIF(pConfig->extendIdFilterNum) |
            CAN_ERFCR_NFE(pConfig->idFilterPairNum) | CAN_ERFCR_ERFWM(pConfig->fifoWatermark);
        /* Copy ID filter table to Enhanced Rx FIFO Filter Element registers. */
        for (i = 0; i < (UINT32)FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO_FILTER_MAX_NUMBER; i++)
        {
            base->ERFFEL[i] = (i < ((UINT32)pConfig->idFilterPairNum * 2U)) ? pConfig->idFilterTable[i] : 0xFFFFFFFFU;
        }

        /* Setting Message Reception Priority. */
        base->CTRL2 = (pConfig->priority == kFLEXCAN_RxFifoPrioHigh) ? (base->CTRL2 & ~CAN_CTRL2_MRP_MASK) :
            (base->CTRL2 | CAN_CTRL2_MRP_MASK);
    }
    else
    {
        /* Disable Enhanced Rx FIFO. */
        base->ERFCR &= ~CAN_ERFCR_ERFEN_MASK;
        /* Clean all Enhanced Rx FIFO Filter Element registers. */
        for (i = 0; i < (UINT32)FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO_FILTER_MAX_NUMBER; i++)
        {
            base->ERFFEL[i] = 0xFFFFFFFFU;
        }
    }

    /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(base);
}
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA) && FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA)
/*
 * Enables or disables the FlexCAN Legacy/Enhanced Rx FIFO DMA request.
 *
 * This function enables or disables the DMA feature of FlexCAN build-in Rx FIFO.
 *
 * param base FlexCAN peripheral base address.
 * param enable TRUE to enable, FALSE to disable.
 */
void FLEXCAN_EnableRxFifoDMA(volatile IMXCAN_REGISTERS *base, BOOLEAN enable)
{
    if (enable)
    {
        /* Enter Freeze Mode. */
        FLEXCAN_EnterFreezeMode(base);

        /* Enable FlexCAN DMA. */
        base->MCR |= CAN_MCR_DMA_MASK;

        /* Exit Freeze Mode. */
        FLEXCAN_ExitFreezeMode(base);
    }
    else
    {
        /* Enter Freeze Mode. */
        FLEXCAN_EnterFreezeMode(base);

        /* Disable FlexCAN DMA. */
        base->MCR &= ~CAN_MCR_DMA_MASK;

        /* Exit Freeze Mode. */
        FLEXCAN_ExitFreezeMode(base);
    }
}
#endif /* FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA */

#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
/*
 * Gets the FlexCAN Memory Error Report registers status.
 *
 * This function gets the FlexCAN Memory Error Report registers status.
 *
 * param base FlexCAN peripheral base address.
 * param errorStatus Pointer to FlexCAN Memory Error Report registers status structure.
 */
void FLEXCAN_GetMemoryErrorReportStatus(volatile IMXCAN_REGISTERS *base, flexcan_memory_error_report_status_t *errorStatus)
{
    UINT32 temp;
    /*  Disable updates of the error report registers. */
    base->MECR |= CAN_MECR_RERRDIS_MASK;

    errorStatus->accessAddress = (UINT16)(base->RERRAR & CAN_RERRAR_ERRADDR_MASK);
    errorStatus->errorData = base->RERRDR;
    errorStatus->errorType =
        (base->RERRAR & CAN_RERRAR_NCE_MASK) == 0U ? kFLEXCAN_CorrectableError : kFLEXCAN_NonCorrectableError;

    temp = (base->RERRAR & CAN_RERRAR_SAID_MASK) >> CAN_RERRAR_SAID_SHIFT;
    switch (temp)
    {
    case (UINT32)kFLEXCAN_MoveOutFlexCanAccess:
    case (UINT32)kFLEXCAN_MoveInAccess:
    case (UINT32)kFLEXCAN_TxArbitrationAccess:
    case (UINT32)kFLEXCAN_RxMatchingAccess:
    case (UINT32)kFLEXCAN_MoveOutHostAccess:
        errorStatus->accessType = (flexcan_memory_access_type_t)temp;
        break;
    default:
        ASSERT(FALSE);
        break;
    }

    for (UINT32 i = 0; i < 4U; i++)
    {
        temp = (base->RERRSYNR & ((UINT32)CAN_RERRSYNR_SYND0_MASK << (i * 8U))) >> (i * 8U);
        errorStatus->byteStatus[i].byteIsRead = (base->RERRSYNR & ((UINT32)CAN_RERRSYNR_BE0_MASK << (i * 8U))) != 0U;
        switch (temp)
        {
        case CAN_RERRSYNR_SYND0(kFLEXCAN_NoError):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_ParityBits0Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_ParityBits1Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_ParityBits2Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_ParityBits3Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_ParityBits4Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_DataBits0Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_DataBits1Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_DataBits2Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_DataBits3Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_DataBits4Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_DataBits5Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_DataBits6Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_DataBits7Error):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_AllZeroError):
        case CAN_RERRSYNR_SYND0(kFLEXCAN_AllOneError):
            errorStatus->byteStatus[i].bitAffected = (flexcan_byte_error_syndrome_t)temp;
            break;
        default:
            errorStatus->byteStatus[i].bitAffected = kFLEXCAN_NonCorrectableErrors;
            break;
        }
    }

    /*  Re-enable updates of the error report registers. */
    base->MECR &= CAN_MECR_RERRDIS_MASK;
}
#endif

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032)
/*
 * FlexCAN: A frame with wrong ID or payload is transmitted into
 * the CAN bus when the Message Buffer under transmission is
 * either aborted or deactivated while the CAN bus is in the Bus Idle state
 *
 * This function to do workaround for ERR006032
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 */
static void FLEXCAN_ERRATA_6032(volatile IMXCAN_REGISTERS *base, volatile UINT32 *mbCSAddr)
{
    UINT32 dbg_temp = 0U;
    UINT32 u32TempCS = 0U;
    UINT32 u32Timeout = DELAY_BUSIDLE;
    UINT32 u32TempIMASK1 = base->IMASK1;
    /*after backup all interruption, disable ALL interruption*/
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT32 u32TempIMASK2 = base->IMASK2;
    base->IMASK2 = 0;
#endif
    base->IMASK1 = 0;
    dbg_temp = (UINT32)(base->DBG1);
    switch (dbg_temp & CAN_DBG1_CFSM_MASK)
    {
    case RXINTERMISSION:
        if (CBN_VALUE3 == (dbg_temp & CAN_DBG1_CBN_MASK))
        {
            /*wait until CFSM is different from RXINTERMISSION */
            while (RXINTERMISSION == (base->DBG1 & CAN_DBG1_CFSM_MASK))
            {
                __NOP();
            }
        }
        break;
    case TXINTERMISSION:
        if (CBN_VALUE3 == (dbg_temp & CAN_DBG1_CBN_MASK))
        {
            /*wait until CFSM is different from TXINTERMISSION*/
            while (TXINTERMISSION == (base->DBG1 & CAN_DBG1_CFSM_MASK))
            {
                __NOP();
            }
        }
        break;
    default:
        /* To avoid MISRA-C 2012 rule 16.4 issue. */
        break;
    }
    /*Anyway, BUSIDLE need to delay*/
    if (BUSIDLE == (base->DBG1 & CAN_DBG1_CFSM_MASK))
    {
        while (u32Timeout-- > 0U)
        {
            __NOP();
        }

        /*Write 0x0 into Code field of CS word.*/
        u32TempCS = (UINT32)(*mbCSAddr);
        u32TempCS &= ~CAN_CS_CODE_MASK;
        *mbCSAddr = u32TempCS;
    }
    /*restore interruption*/
    base->IMASK1 = u32TempIMASK1;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    base->IMASK2 = u32TempIMASK2;
#endif
}
#endif

/*
 * Writes a FlexCAN Message to the Transmit Message Buffer.
 *
 * This function writes a CAN Message to the specified Transmit Message Buffer
 * and changes the Message Buffer state to start CAN Message transmit. After
 * that the function returns immediately.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pTxFrame Pointer to CAN message frame to be sent.
 * retval kStatus_Success - Write Tx Message Buffer Successfully.
 * retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_WriteTxMb(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, const flexcan_frame_t *pTxFrame)
{
    /* Assertion. */
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    ASSERT(NULL != pTxFrame);
    ASSERT(pTxFrame->mfs_0.length <= 8U);
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    UINT32 cs_temp = 0;
    status_t status;

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032)
    FLEXCAN_ERRATA_6032(base, &(base->MB[mbIdx].CS));
#endif
    /* Check if Message Buffer is available. */
    if (CAN_CS_CODE(kFLEXCAN_TxMbDataOrRemote) != (base->MB[mbIdx].CS & CAN_CS_CODE_MASK))
    {
        /* Inactive Tx Message Buffer. */
        base->MB[mbIdx].CS = (base->MB[mbIdx].CS & ~CAN_CS_CODE_MASK) | CAN_CS_CODE(kFLEXCAN_TxMbInactive);

        /* Fill Message ID field. */
        base->MB[mbIdx].ID = pTxFrame->mfs_1.id;

        /* Fill Message Format field. */
        if ((UINT32)kFLEXCAN_FrameFormatExtend == pTxFrame->mfs_0.format)
        {
            cs_temp |= CAN_CS_SRR_MASK | CAN_CS_IDE_MASK;
        }

        /* Fill Message Type field. */
        if ((UINT32)kFLEXCAN_FrameTypeRemote == pTxFrame->mfs_0.type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }

        cs_temp |= CAN_CS_CODE(kFLEXCAN_TxMbDataOrRemote) | CAN_CS_DLC(pTxFrame->mfs_0.length);

        /* Load Message Payload. */
        base->MB[mbIdx].WORD0 = pTxFrame->mfp.mfp_w.dataWord0;
        base->MB[mbIdx].WORD1 = pTxFrame->mfp.mfp_w.dataWord1;

        /* Activate Tx Message Buffer. */
        base->MB[mbIdx].CS = cs_temp;

#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
        base->MB[FLEXCAN_GetFirstValidMb(base)].CS = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
        base->MB[FLEXCAN_GetFirstValidMb(base)].CS = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
#endif

        status = kStatus_Success;
    }
    else
    {
        /* Tx Message Buffer is activated, return immediately. */
        status = kStatus_Fail;
    }

    return status;
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Writes a FlexCAN FD Message to the Transmit Message Buffer.
 *
 * This function writes a CAN FD Message to the specified Transmit Message Buffer
 * and changes the Message Buffer state to start CAN FD Message transmit. After
 * that the function returns immediately.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN FD Message Buffer index.
 * param pTxFrame Pointer to CAN FD message frame to be sent.
 * retval kStatus_Success - Write Tx Message Buffer Successfully.
 * retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_WriteFDTxMb(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, const flexcan_fd_frame_t *pTxFrame)
{
    /* Assertion. */
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    ASSERT(NULL != pTxFrame);
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    status_t status;
    UINT32 cs_temp = 0;
    UINT8 cnt = 0;
    UINT32 can_cs = 0;
    UINT8 payload_dword = 1;
    UINT32 dataSize = (base->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT;
#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
    UINT32 availoffset = FLEXCAN_GetFDMailboxOffset(base, FLEXCAN_GetFirstValidMb(base));
#endif
    volatile UINT32 *mbAddr = &(base->MB[0].CS);
    UINT32 offset = FLEXCAN_GetFDMailboxOffset(base, mbIdx);

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032)
    FLEXCAN_ERRATA_6032(base, &(mbAddr[offset]));
#endif

    can_cs = mbAddr[offset];
    /* Check if Message Buffer is available. */
    if (CAN_CS_CODE(kFLEXCAN_TxMbDataOrRemote) != (can_cs & CAN_CS_CODE_MASK))
    {
        /* Inactive Tx Message Buffer and Fill Message ID field. */
        mbAddr[offset] = (can_cs & ~CAN_CS_CODE_MASK) | CAN_CS_CODE(kFLEXCAN_TxMbInactive);
        mbAddr[offset + 1U] = pTxFrame->fdmfs_1.id;

        /* Fill Message Format field. */
        if ((UINT32)kFLEXCAN_FrameFormatExtend == pTxFrame->fdmfs_0.format)
        {
            cs_temp |= CAN_CS_SRR_MASK | CAN_CS_IDE_MASK;
        }

        /* Fill Message Type field. */
        if ((UINT32)kFLEXCAN_FrameTypeRemote == pTxFrame->fdmfs_0.type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }

        cs_temp |= CAN_CS_CODE(kFLEXCAN_TxMbDataOrRemote) | CAN_CS_DLC(pTxFrame->fdmfs_0.length) | CAN_CS_EDL(1) |
            CAN_CS_BRS(pTxFrame->fdmfs_0.brs);

        /* Calculate the DWORD number, dataSize 0/1/2/3 corresponds to 8/16/32/64
           Bytes payload. */
        for (cnt = 0; cnt < (dataSize + 1U); cnt++)
        {
            payload_dword *= 2U;
        }

        /* Load Message Payload and Activate Tx Message Buffer. */
        for (cnt = 0; cnt < payload_dword; cnt++)
        {
            mbAddr[offset + 2U + cnt] = pTxFrame->fdmfp.fdmfp_w.dataWord[cnt];
        }
        mbAddr[offset] = cs_temp;

#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
        mbAddr[availoffset] = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
        mbAddr[availoffset] = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
#endif

        status = kStatus_Success;
    }
    else
    {
        /* Tx Message Buffer is activated, return immediately. */
        status = kStatus_Fail;
    }

    return status;
}
#endif

/*
 * Reads a FlexCAN Message from Receive Message Buffer.
 *
 * This function reads a CAN message from a specified Receive Message Buffer.
 * The function fills a receive CAN message frame structure with
 * just received data and activates the Message Buffer again.
 * The function returns immediately.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_ReadRxMb(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_frame_t *pRxFrame)
{
    /* Assertion. */
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    ASSERT(NULL != pRxFrame);
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    UINT32 cs_temp;
    UINT32 rx_code;
    status_t status;

    /* Read CS field of Rx Message Buffer to lock Message Buffer. */
    cs_temp = base->MB[mbIdx].CS;
    /* Get Rx Message Buffer Code field. */
    rx_code = (cs_temp & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT;

    /* Check to see if Rx Message Buffer is full. */
    if (((UINT32)kFLEXCAN_RxMbFull == rx_code) || ((UINT32)kFLEXCAN_RxMbOverrun == rx_code))
    {
        /* Store Message ID. */
        pRxFrame->mfs_1.id = base->MB[mbIdx].ID & (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);

        /* Get the message ID and format. */
        pRxFrame->mfs_0.format = (cs_temp & CAN_CS_IDE_MASK) != 0U ? (UINT8)kFLEXCAN_FrameFormatExtend :
            (UINT8)kFLEXCAN_FrameFormatStandard;

        /* Get the message type. */
        pRxFrame->mfs_0.type = (cs_temp & CAN_CS_RTR_MASK) != 0U ? (UINT8)kFLEXCAN_FrameTypeRemote : (UINT8)kFLEXCAN_FrameTypeData;

        /* Get the message length. */
        pRxFrame->mfs_0.length = (UINT8)((cs_temp & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);

        /* Get the time stamp. */
        pRxFrame->mfs_0.timestamp = (UINT16)((cs_temp & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        /* Store Message Payload. */
        pRxFrame->mfp.mfp_w.dataWord0 = base->MB[mbIdx].WORD0;
        pRxFrame->mfp.mfp_w.dataWord1 = base->MB[mbIdx].WORD1;

        /* Read free-running timer to unlock Rx Message Buffer. */
        (void)base->TIMER;

        if ((UINT32)kFLEXCAN_RxMbFull == rx_code)
        {
            status = kStatus_Success;
        }
        else
        {
            status = kStatus_FLEXCAN_RxOverflow;
        }
    }
    else
    {
        /* Read free-running timer to unlock Rx Message Buffer. */
        (void)base->TIMER;

        status = kStatus_Fail;
    }

    return status;
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Reads a FlexCAN FD Message from Receive Message Buffer.
 *
 * This function reads a CAN FD message from a specified Receive Message Buffer.
 * The function fills a receive CAN FD message frame structure with
 * just received data and activates the Message Buffer again.
 * The function returns immediately.
 *
 * param base FlexCAN peripheral base address.
 * param mbIdx The FlexCAN FD Message Buffer index.
 * param pRxFrame Pointer to CAN FD message frame structure for reception.
 * retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_ReadFDRxMb(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_fd_frame_t *pRxFrame)
{
    /* Assertion. */
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    ASSERT(NULL != pRxFrame);
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    status_t status;
    UINT32 cs_temp;
    UINT8 rx_code;
    UINT8 cnt = 0;
    UINT32 can_id = 0;
    UINT32 dataSize;
    dataSize = (base->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT;
    UINT8 payload_dword = 1;
    volatile UINT32 *mbAddr = &(base->MB[0].CS);
    UINT32 offset = FLEXCAN_GetFDMailboxOffset(base, mbIdx);

    /* Read CS field of Rx Message Buffer to lock Message Buffer. */
    cs_temp = mbAddr[offset];
    can_id = mbAddr[offset + 1U];

    /* Get Rx Message Buffer Code field. */
    rx_code = (UINT8)((cs_temp & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT);

    /* Check to see if Rx Message Buffer is full. */
    if (((UINT8)kFLEXCAN_RxMbFull == rx_code) || ((UINT8)kFLEXCAN_RxMbOverrun == rx_code))
    {
        /* Store Message ID. */
        pRxFrame->fdmfs_1.id = can_id & (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);

        /* Get the message ID and format. */
        pRxFrame->fdmfs_0.format = (cs_temp & CAN_CS_IDE_MASK) != 0U ? (UINT8)kFLEXCAN_FrameFormatExtend :
            (UINT8)kFLEXCAN_FrameFormatStandard;

        /* Get the message type. */
        pRxFrame->fdmfs_0.type = (cs_temp & CAN_CS_RTR_MASK) != 0U ? (UINT8)kFLEXCAN_FrameTypeRemote : (UINT8)kFLEXCAN_FrameTypeData;

        /* Get the message length. */
        pRxFrame->fdmfs_0.length = (UINT8)((cs_temp & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);

        /* Get the time stamp. */
        pRxFrame->fdmfs_0.timestamp = (UINT16)((cs_temp & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        /* Calculate the DWORD number, dataSize 0/1/2/3 corresponds to 8/16/32/64
           Bytes payload. */
        for (cnt = 0; cnt < (dataSize + 1U); cnt++)
        {
            payload_dword *= 2U;
        }

        /* Store Message Payload. */
        for (cnt = 0; cnt < payload_dword; cnt++)
        {
            pRxFrame->fdmfp.fdmfp_w.dataWord[cnt] = mbAddr[offset + 2U + cnt];
        }

        /* Read free-running timer to unlock Rx Message Buffer. */
        (void)base->TIMER;

        if ((UINT32)kFLEXCAN_RxMbFull == rx_code)
        {
            status = kStatus_Success;
        }
        else
        {
            status = kStatus_FLEXCAN_RxOverflow;
        }
    }
    else
    {
        /* Read free-running timer to unlock Rx Message Buffer. */
        (void)base->TIMER;

        status = kStatus_Fail;
    }

    return status;
}
#endif

/*
 * Reads a FlexCAN Message from Legacy Rx FIFO.
 *
 * This function reads a CAN message from the FlexCAN Legacy Rx FIFO.
 *
 * param base FlexCAN peripheral base address.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * retval kStatus_Success - Read Message from Rx FIFO successfully.
 * retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_ReadRxFifo(volatile IMXCAN_REGISTERS *base, flexcan_frame_t *pRxFrame)
{
    /* Assertion. */
    ASSERT(NULL != pRxFrame);

    UINT32 cs_temp;
    status_t status;

    /* Check if Legacy Rx FIFO is Enabled. */
    if (0U != (base->MCR & CAN_MCR_RFEN_MASK))
    {
        /* Read CS field of Rx Message Buffer to lock Message Buffer. */
        cs_temp = base->MB[0].CS;

        /* Read data from Rx FIFO output port. */
        /* Store Message ID. */
        pRxFrame->mfs_1.id = base->MB[0].ID & (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);

        /* Get the message ID and format. */
        pRxFrame->mfs_0.format = (cs_temp & CAN_CS_IDE_MASK) != 0U ? (UINT8)kFLEXCAN_FrameFormatExtend :
            (UINT8)kFLEXCAN_FrameFormatStandard;

        /* Get the message type. */
        pRxFrame->mfs_0.type = (cs_temp & CAN_CS_RTR_MASK) != 0U ? (UINT8)kFLEXCAN_FrameTypeRemote : (UINT8)kFLEXCAN_FrameTypeData;

        /* Get the message length. */
        pRxFrame->mfs_0.length = (UINT8)((cs_temp & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);

        /* Get the time stamp. */
        pRxFrame->mfs_0.timestamp = (UINT16)((cs_temp & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        /* Store Message Payload. */
        pRxFrame->mfp.mfp_w.dataWord0 = base->MB[0].WORD0;
        pRxFrame->mfp.mfp_w.dataWord1 = base->MB[0].WORD1;

        /* Store ID Filter Hit Index. */
        pRxFrame->mfs_0.idhit = (UINT16)(base->RXFIR & CAN_RXFIR_IDHIT_MASK);

        /* Read free-running timer to unlock Rx Message Buffer. */
        (void)base->TIMER;

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Reads a FlexCAN Message from Enhanced Rx FIFO.
 *
 * This function reads a CAN or CAN FD message from the FlexCAN Enhanced Rx FIFO.
 *
 * param base FlexCAN peripheral base address.
 * param pRxFrame Pointer to CAN FD message frame structure for reception.
 * retval kStatus_Success - Read Message from Rx FIFO successfully.
 * retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_ReadEnhancedRxFifo(volatile IMXCAN_REGISTERS *base, flexcan_fd_frame_t *pRxFrame)
{
    /* Assertion. */
    ASSERT(NULL != pRxFrame);

    status_t status;
    UINT32 idHitOff;

    /* Check if Enhanced Rx FIFO is Enabled. */
    if (0U != (base->ERFCR & CAN_ERFCR_ERFEN_MASK))
    {
        /* Enhanced Rx FIFO ID HIT offset is changed dynamically according to data length code (DLC) . */
        idHitOff = (DLC_LENGTH_DECODE(((flexcan_fd_frame_t *)E_RX_FIFO(base))->length) + 3U) / 4U + 3U;
        /* Copy CAN FD Message from Enhanced Rx FIFO, should use the DLC value to identify the bytes that belong to the
         * message which is being read. */
        (void)memcpy((void *)pRxFrame, (void *)(UINT32 *)E_RX_FIFO(base), sizeof(UINT32) * idHitOff);
        pRxFrame->idhit = pRxFrame->dataWord[idHitOff - 3U];
        /* Clear the unused frame data. */
        for (UINT32 i = (idHitOff - 3U); i < 16U; i++)
        {
            pRxFrame->dataWord[i] = 0x0;
        }

        /* Clear data available flag to let FlexCAN know one frame has been read from the Enhanced Rx FIFO. */
        base->ERFSR |= CAN_ERFSR_ERFDA_MASK;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}
#endif

/*
 * Performs a polling send transaction on the CAN bus.
 *
 * note  A transfer handle does not need to be created  before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pTxFrame Pointer to CAN message frame to be sent.
 * retval kStatus_Success - Write Tx Message Buffer Successfully.
 * retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_TransferSendBlocking(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_frame_t *pTxFrame)
{
    status_t status;

    /* Write Tx Message Buffer to initiate a data sending. */
    if (kStatus_Success == FLEXCAN_WriteTxMb(base, mbIdx, (const flexcan_frame_t *)pTxFrame))
    {
        /* Wait until CAN Message send out. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        UINT64 u64flag = 1;
        while (0U == FLEXCAN_GetMbStatusFlags(base, u64flag << mbIdx))
#else
        UINT32 u32flag = 1;
        while (0U == FLEXCAN_GetMbStatusFlags(base, u32flag << mbIdx))
#endif
        {
        }

        /* Clean Tx Message Buffer Flag. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        FLEXCAN_ClearMbStatusFlags(base, u64flag << mbIdx);
#else
        FLEXCAN_ClearMbStatusFlags(base, u32flag << mbIdx);
#endif
        /*After TX MB tranfered success, update the Timestamp from MB[mbIdx].CS register*/
        pTxFrame->mfs_0.timestamp = (UINT16)((base->MB[mbIdx].CS & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

/*
 * Performs a polling receive transaction on the CAN bus.
 *
 * note  A transfer handle does not need to be created  before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_TransferReceiveBlocking(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_frame_t *pRxFrame)
{
    /* Wait until Rx Message Buffer non-empty. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 u64flag = 1;
    while (0U == FLEXCAN_GetMbStatusFlags(base, u64flag << mbIdx))
#else
    UINT32 u32flag = 1;
    while (0U == FLEXCAN_GetMbStatusFlags(base, u32flag << mbIdx))
#endif
    {
    }

    /* Clean Rx Message Buffer Flag. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    FLEXCAN_ClearMbStatusFlags(base, u64flag << mbIdx);
#else
    FLEXCAN_ClearMbStatusFlags(base, u32flag << mbIdx);
#endif

    /* Read Received CAN Message. */
    return FLEXCAN_ReadRxMb(base, mbIdx, pRxFrame);
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Performs a polling send transaction on the CAN bus.
 *
 * note  A transfer handle does not need to be created before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param mbIdx The FlexCAN FD Message Buffer index.
 * param pTxFrame Pointer to CAN FD message frame to be sent.
 * retval kStatus_Success - Write Tx Message Buffer Successfully.
 * retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t FLEXCAN_TransferFDSendBlocking(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_fd_frame_t *pTxFrame)
{
    status_t status;

    /* Write Tx Message Buffer to initiate a data sending. */
    if (kStatus_Success == FLEXCAN_WriteFDTxMb(base, mbIdx, (const flexcan_fd_frame_t *)pTxFrame))
    {
        /* Wait until CAN Message send out. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        UINT64 u64flag = 1;
        while (0U == FLEXCAN_GetMbStatusFlags(base, u64flag << mbIdx))
#else
        UINT32 u32flag = 1;
        while (0U == FLEXCAN_GetMbStatusFlags(base, u32flag << mbIdx))
#endif
        {
        }

        /* Clean Tx Message Buffer Flag. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        FLEXCAN_ClearMbStatusFlags(base, u64flag << mbIdx);
#else
        FLEXCAN_ClearMbStatusFlags(base, u32flag << mbIdx);
#endif
        /*After TX MB tranfered success, update the Timestamp from base->MB[offset for CAN FD].CS register*/
        volatile UINT32 *mbAddr = &(base->MB[0].CS);
        UINT32 offset = FLEXCAN_GetFDMailboxOffset(base, mbIdx);
        pTxFrame->fdmfs_0.timestamp = (UINT16)((mbAddr[offset] & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

/*
 * Performs a polling receive transaction on the CAN bus.
 *
 * note  A transfer handle does not need to be created before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param mbIdx The FlexCAN FD Message Buffer index.
 * param pRxFrame Pointer to CAN FD message frame structure for reception.
 * retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * retval kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 * retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t FLEXCAN_TransferFDReceiveBlocking(volatile IMXCAN_REGISTERS *base, UINT8 mbIdx, flexcan_fd_frame_t *pRxFrame)
{
    /* Wait until Rx Message Buffer non-empty. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 u64flag = 1;
    while (0U == FLEXCAN_GetMbStatusFlags(base, u64flag << mbIdx))
#else
    UINT32 u32flag = 1;
    while (0U == FLEXCAN_GetMbStatusFlags(base, u32flag << mbIdx))
#endif
    {
    }

    /* Clean Rx Message Buffer Flag. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    FLEXCAN_ClearMbStatusFlags(base, u64flag << mbIdx);
#else
    FLEXCAN_ClearMbStatusFlags(base, u32flag << mbIdx);
#endif

    /* Read Received CAN Message. */
    return FLEXCAN_ReadFDRxMb(base, mbIdx, pRxFrame);
}
#endif

/*
 * Performs a polling receive transaction from Legacy Rx FIFO on the CAN bus.
 *
 * note  A transfer handle does not need to be created before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param pRxFrame Pointer to CAN message frame structure for reception.
 * retval kStatus_Success - Read Message from Rx FIFO successfully.
 * retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_TransferReceiveFifoBlocking(volatile IMXCAN_REGISTERS *base, flexcan_frame_t *pRxFrame)
{
    status_t rxFifoStatus;

    /* Wait until Legacy Rx FIFO non-empty. */
    while (0U == FLEXCAN_GetMbStatusFlags(base, (UINT32)kFLEXCAN_RxFifoFrameAvlFlag))
    {
    }

    /* Read data from Legacy Rx FIFO. */
    rxFifoStatus = FLEXCAN_ReadRxFifo(base, pRxFrame);

    /* Clean Rx Fifo available flag. */
    FLEXCAN_ClearMbStatusFlags(base, (UINT32)kFLEXCAN_RxFifoFrameAvlFlag);

    return rxFifoStatus;
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Performs a polling receive transaction from Enhanced Rx FIFO on the CAN bus.
 *
 * note  A transfer handle does not need to be created before calling this API.
 *
 * param base FlexCAN peripheral base pointer.
 * param pRxFrame Pointer to CAN FD message frame structure for reception.
 * retval kStatus_Success - Read Message from Rx FIFO successfully.
 * retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t FLEXCAN_TransferReceiveEnhancedFifoBlocking(volatile IMXCAN_REGISTERS *base, flexcan_fd_frame_t *pRxFrame)
{
    status_t rxFifoStatus;

    /* Wait until Enhanced Rx FIFO non-empty. */
    while (0U == (FLEXCAN_GetStatusFlags(base) & (UINT64)kFLEXCAN_ERxFifoDataAvlIntFlag))
    {
    }

    /* Read data from Enhanced Rx FIFO */
    rxFifoStatus = FLEXCAN_ReadEnhancedRxFifo(base, pRxFrame);

    /* Clean Enhanced Rx Fifo data available flag. */
    FLEXCAN_ClearStatusFlags(base, (UINT64)kFLEXCAN_ERxFifoDataAvlIntFlag);

    return rxFifoStatus;
}
#endif

/*
 * Initializes the FlexCAN handle.
 *
 * This function initializes the FlexCAN handle, which can be used for other FlexCAN
 * transactional APIs. Usually, for a specified FlexCAN instance,
 * call this API once to get the initialized handle.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param callback The callback function.
 * param userData The parameter of the callback function.
 */
void FLEXCAN_TransferCreateHandle(volatile IMXCAN_REGISTERS *base,
    flexcan_handle_t *handle,
    //flexcan_transfer_callback_t callback,
    void *userData)
{
    ASSERT(NULL != handle);

    //UINT8 instance;

    /* Clean FlexCAN transfer handle. */
    //(void)memset(handle, 0, sizeof(*handle));

    /* Get instance from peripheral base address. */
    //instance = (UINT8)FLEXCAN_GetInstance(base);

    /* Save the context in global variables to support the double weak mechanism. */
    //s_flexcanHandle = handle;

    /* Register Callback function. */
    //handle->callback = callback;
    handle->userData = userData;

    //s_flexcanIsr = FLEXCAN_TransferHandleIRQ;

    /* We Enable Error & Status interrupt here, because this interrupt just
     * report current status of FlexCAN module through Callback function.
     * It is insignificance without a available callback function.
     */
    //if (handle->callback != NULL)
    if (0)
    {
        FLEXCAN_EnableInterrupts(
            base, (UINT32)kFLEXCAN_BusOffInterruptEnable | (UINT32)kFLEXCAN_ErrorInterruptEnable |
            (UINT32)kFLEXCAN_RxWarningInterruptEnable | (UINT32)kFLEXCAN_TxWarningInterruptEnable |
            (UINT32)kFLEXCAN_WakeUpInterruptEnable
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
            | (UINT64)kFLEXCAN_PNMatchWakeUpInterruptEnable |
            (UINT64)kFLEXCAN_PNTimeoutWakeUpInterruptEnable
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
            | (UINT64)kFLEXCAN_HostAccessNCErrorInterruptEnable |
            (UINT64)kFLEXCAN_FlexCanAccessNCErrorInterruptEnable |
            (UINT64)kFLEXCAN_HostOrFlexCanCErrorInterruptEnable
#endif
        );
    }
    else
    {
        FLEXCAN_DisableInterrupts(
            base, (UINT32)kFLEXCAN_BusOffInterruptEnable | (UINT32)kFLEXCAN_ErrorInterruptEnable |
            (UINT32)kFLEXCAN_RxWarningInterruptEnable | (UINT32)kFLEXCAN_TxWarningInterruptEnable |
            (UINT32)kFLEXCAN_WakeUpInterruptEnable
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
            | (UINT64)kFLEXCAN_PNMatchWakeUpInterruptEnable |
            (UINT64)kFLEXCAN_PNTimeoutWakeUpInterruptEnable
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
            | (UINT64)kFLEXCAN_HostAccessNCErrorInterruptEnable |
            (UINT64)kFLEXCAN_FlexCanAccessNCErrorInterruptEnable |
            (UINT64)kFLEXCAN_HostOrFlexCanCErrorInterruptEnable
#endif
        );
    }

    /* Enable interrupts in NVIC.
    (void)EnableIRQ((IRQn_Type)(s_flexcanRxWarningIRQ[instance]));
    (void)EnableIRQ((IRQn_Type)(s_flexcanTxWarningIRQ[instance]));
    (void)EnableIRQ((IRQn_Type)(s_flexcanWakeUpIRQ[instance]));
    (void)EnableIRQ((IRQn_Type)(s_flexcanErrorIRQ[instance]));
    (void)EnableIRQ((IRQn_Type)(s_flexcanBusOffIRQ[instance]));
    (void)EnableIRQ((IRQn_Type)(s_flexcanMbIRQ[instance]));
    */
}

/*
 * Sends a message using IRQ.
 *
 * This function sends a message using IRQ. This is a non-blocking function, which returns
 * right away. When messages have been sent out, the send callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pMbXfer FlexCAN Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * retval kStatus_Success        Start Tx Message Buffer sending process successfully.
 * retval kStatus_Fail           Write Tx Message Buffer failed.
 * retval kStatus_FLEXCAN_TxBusy Tx Message Buffer is in use.
 */
status_t FLEXCAN_TransferSendNonBlocking(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    /* Assertion. */
    ASSERT(NULL != handle);
    ASSERT(NULL != pMbXfer);
    ASSERT(pMbXfer->mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, pMbXfer->mbIdx));
#endif

    status_t status;

    /* Check if Message Buffer is idle. */
    if ((UINT8)kFLEXCAN_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
        /* Distinguish transmit type. */
        if ((UINT32)kFLEXCAN_FrameTypeRemote == pMbXfer->frame->mfs_0.type)
        {
            handle->mbState[pMbXfer->mbIdx] = (UINT8)kFLEXCAN_StateTxRemote;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (UINT8)kFLEXCAN_StateTxData;
        }

        if (kStatus_Success ==
            FLEXCAN_WriteTxMb(base, pMbXfer->mbIdx, (const flexcan_frame_t *)pMbXfer->frame))
        {
            /* Enable Message Buffer Interrupt. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
            UINT64 u64mask = 1;
            FLEXCAN_EnableMbInterrupts(base, u64mask << pMbXfer->mbIdx);
#else
            UINT32 u32mask = 1;
            FLEXCAN_EnableMbInterrupts(base, u32mask << pMbXfer->mbIdx);
#endif
            status = kStatus_Success;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (UINT8)kFLEXCAN_StateIdle;
            status = kStatus_Fail;
        }
    }
    else
    {
        status = kStatus_FLEXCAN_TxBusy;
    }

    return status;
}

/*
 * Receives a message using IRQ.
 *
 * This function receives a message using IRQ. This is non-blocking function, which returns
 * right away. When the message has been received, the receive callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pMbXfer FlexCAN Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * retval kStatus_Success        - Start Rx Message Buffer receiving process successfully.
 * retval kStatus_FLEXCAN_RxBusy - Rx Message Buffer is in use.
 */
status_t FLEXCAN_TransferReceiveNonBlocking(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    status_t status;

    /* Assertion. */
    ASSERT(NULL != handle);
    ASSERT(NULL != pMbXfer);
    ASSERT(pMbXfer->mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, pMbXfer->mbIdx));
#endif

    /* Check if Message Buffer is idle. */
    if ((UINT8)kFLEXCAN_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
        handle->mbState[pMbXfer->mbIdx] = (UINT8)kFLEXCAN_StateRxData;

        /* Register Message Buffer. */
        handle->mbFrameBuf[pMbXfer->mbIdx] = pMbXfer->frame;

        /* Enable Message Buffer Interrupt. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        UINT64 u64mask = 1;
        FLEXCAN_EnableMbInterrupts(base, u64mask << pMbXfer->mbIdx);
#else
        UINT32 u32mask = 1;
        FLEXCAN_EnableMbInterrupts(base, u32mask << pMbXfer->mbIdx);
#endif

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_FLEXCAN_RxBusy;
    }

    return status;
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Sends a message using IRQ.
 *
 * This function sends a message using IRQ. This is a non-blocking function, which returns
 * right away. When messages have been sent out, the send callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pMbXfer FlexCAN FD Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * retval kStatus_Success        Start Tx Message Buffer sending process successfully.
 * retval kStatus_Fail           Write Tx Message Buffer failed.
 * retval kStatus_FLEXCAN_TxBusy Tx Message Buffer is in use.
 */
status_t FLEXCAN_TransferFDSendNonBlocking(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    /* Assertion. */
    ASSERT(NULL != handle);
    ASSERT(NULL != pMbXfer);
    ASSERT(pMbXfer->mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, pMbXfer->mbIdx));
#endif

    status_t status;

    /* Check if Message Buffer is idle. */
    if ((UINT8)kFLEXCAN_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
        /* Distinguish transmit type. */
        if ((UINT32)kFLEXCAN_FrameTypeRemote == pMbXfer->framefd->fdmfs_0.type)
        {
            handle->mbState[pMbXfer->mbIdx] = (UINT8)kFLEXCAN_StateTxRemote;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (UINT8)kFLEXCAN_StateTxData;
        }

        if (kStatus_Success ==
            FLEXCAN_WriteFDTxMb(base, pMbXfer->mbIdx, (const flexcan_fd_frame_t *)pMbXfer->framefd))
        {
            /* Enable Message Buffer Interrupt. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
            UINT64 u64mask = 1;
            FLEXCAN_EnableMbInterrupts(base, u64mask << pMbXfer->mbIdx);
#else
            UINT32 u32mask = 1;
            FLEXCAN_EnableMbInterrupts(base, u32mask << pMbXfer->mbIdx);
#endif

            status = kStatus_Success;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (UINT8)kFLEXCAN_StateIdle;
            status = kStatus_Fail;
        }
    }
    else
    {
        status = kStatus_FLEXCAN_TxBusy;
    }

    return status;
}

/*
 * Receives a message using IRQ.
 *
 * This function receives a message using IRQ. This is non-blocking function, which returns
 * right away. When the message has been received, the receive callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pMbXfer FlexCAN FD Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
 * retval kStatus_Success        - Start Rx Message Buffer receiving process successfully.
 * retval kStatus_FLEXCAN_RxBusy - Rx Message Buffer is in use.
 */
status_t FLEXCAN_TransferFDReceiveNonBlocking(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    /* Assertion. */
    ASSERT(NULL != handle);
    ASSERT(NULL != pMbXfer);
    ASSERT(pMbXfer->mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, pMbXfer->mbIdx));
#endif

    status_t status;

    /* Check if Message Buffer is idle. */
    if ((UINT8)kFLEXCAN_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
        handle->mbState[pMbXfer->mbIdx] = (UINT8)kFLEXCAN_StateRxData;

        /* Register Message Buffer. */
        handle->mbFDFrameBuf[pMbXfer->mbIdx] = pMbXfer->framefd;

        /* Enable Message Buffer Interrupt. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        UINT64 u64mask = 1;
        FLEXCAN_EnableMbInterrupts(base, u64mask << pMbXfer->mbIdx);
#else
        UINT32 u32mask = 1;
        FLEXCAN_EnableMbInterrupts(base, u32mask << pMbXfer->mbIdx);
#endif

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_FLEXCAN_RxBusy;
    }

    return status;
}
#endif

/*
 * Receives a message from Legacy Rx FIFO using IRQ.
 *
 * This function receives a message using IRQ. This is a non-blocking function, which returns
 * right away. When all messages have been received, the receive callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pFifoXfer FlexCAN Rx FIFO transfer structure. See the ref flexcan_fifo_transfer_t.
 * retval kStatus_Success            - Start Rx FIFO receiving process successfully.
 * retval kStatus_FLEXCAN_RxFifoBusy - Rx FIFO is currently in use.
 */
status_t FLEXCAN_TransferReceiveFifoNonBlocking(volatile IMXCAN_REGISTERS *base,
    flexcan_handle_t *handle,
    flexcan_fifo_transfer_t *pFifoXfer)
{
    /* Assertion. */
    ASSERT(NULL != handle);
    ASSERT(NULL != pFifoXfer);

    status_t status;

    /* Check if Message Buffer is idle. */
    if ((UINT8)kFLEXCAN_StateIdle == handle->rxFifoState)
    {
        handle->rxFifoState = (UINT8)kFLEXCAN_StateRxFifo;

        /* Register Message Buffer. */
        handle->rxFifoFrameBuf = pFifoXfer->frame;

        /* Enable Message Buffer Interrupt. */
        FLEXCAN_EnableMbInterrupts(base, (UINT32)kFLEXCAN_RxFifoOverflowFlag | (UINT32)kFLEXCAN_RxFifoWarningFlag |
            (UINT32)kFLEXCAN_RxFifoFrameAvlFlag);

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_FLEXCAN_RxFifoBusy;
    }

    return status;
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Receives a message from Enhanced Rx FIFO using IRQ.
 *
 * This function receives a message using IRQ. This is a non-blocking function, which returns
 * right away. When all messages have been received, the receive callback function is called.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pFifoXfer FlexCAN Rx FIFO transfer structure. See the ref flexcan_fifo_transfer_t.
 * retval kStatus_Success            - Start Rx FIFO receiving process successfully.
 * retval kStatus_FLEXCAN_RxFifoBusy - Rx FIFO is currently in use.
 */
status_t FLEXCAN_TransferReceiveEnhancedFifoNonBlocking(volatile IMXCAN_REGISTERS *base,
    flexcan_handle_t *handle,
    flexcan_fifo_transfer_t *pFifoXfer)
{
    /* Assertion. */
    ASSERT(NULL != handle);
    ASSERT(NULL != pFifoXfer);

    status_t status;
    UINT32 watermark = ((base->ERFCR & CAN_ERFCR_ERFWM_MASK) >> CAN_ERFCR_ERFWM_SHIFT) + 1U;
    UINT64 irqMask =
        (UINT64)kFLEXCAN_ERxFifoUnderflowInterruptEnable | (UINT64)kFLEXCAN_ERxFifoOverflowInterruptEnable;

    /* Check if Enhanced Rx FIFO is idle. */
    if ((UINT8)kFLEXCAN_StateIdle == handle->rxFifoState)
    {
        handle->rxFifoState = (UINT8)kFLEXCAN_StateRxFifo;

        /* Register Message Buffer. */
        handle->rxFifoFDFrameBuf = pFifoXfer->framefd;
        handle->frameNum = pFifoXfer->frameNum;
        handle->transferTotalNum = pFifoXfer->frameNum;

        if (handle->transferTotalNum >= watermark)
        {
            /* Enable watermark interrupt. */
            irqMask |= (UINT64)kFLEXCAN_ERxFifoWatermarkInterruptEnable;
        }
        else
        {
            /* Enable data available interrupt. */
            irqMask |= (UINT64)kFLEXCAN_ERxFifoDataAvlInterruptEnable;
        }
        /* Enable Enhanced Rx FIFO Interrupt. */
        FLEXCAN_EnableInterrupts(base, irqMask);

        status = kStatus_Success;
    }
    else
    {
        status = kStatus_FLEXCAN_RxFifoBusy;
    }

    return status;
}

/*
 * Gets the Enhanced Rx Fifo transfer status during a interrupt non-blocking receive.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param count Number of CAN messages receive so far by the non-blocking transaction.
 * retval kStatus_InvalidArgument count is Invalid.
 * retval kStatus_Success Successfully return the count.
 */

status_t FLEXCAN_TransferGetReceiveEnhancedFifoCount(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, size_t *count)
{
    ASSERT(NULL != handle);

    status_t result = kStatus_Success;

    if (handle->rxFifoState == (UINT32)kFLEXCAN_StateIdle)
    {
        result = kStatus_NoTransferInProgress;
    }
    else
    {
        *count = handle->transferTotalNum - handle->frameNum;
    }

    return result;
}
#endif
/*
 * Aborts the interrupt driven message send process.
 *
 * This function aborts the interrupt driven message send process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 */
void FLEXCAN_TransferAbortSend(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT8 mbIdx)
{
    UINT16 timestamp;

    /* Assertion. */
    ASSERT(NULL != handle);
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    /* Disable Message Buffer Interrupt. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 u64mask = 1;
    FLEXCAN_DisableMbInterrupts(base, u64mask << mbIdx);
#else
    UINT32 u32mask = 1;
    FLEXCAN_DisableMbInterrupts(base, u32mask << mbIdx);
#endif

    /* Update the TX frame 's time stamp by MB[mbIdx].cs. */
    timestamp = (UINT16)((base->MB[mbIdx].CS & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);
    handle->timestamp[mbIdx] = timestamp;

    /* Clean Message Buffer. */
    FLEXCAN_SetTxMbConfig(base, mbIdx, TRUE);

    handle->mbState[mbIdx] = (UINT8)kFLEXCAN_StateIdle;
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
/*
 * Aborts the interrupt driven message send process.
 *
 * This function aborts the interrupt driven message send process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN FD Message Buffer index.
 */
void FLEXCAN_TransferFDAbortSend(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT8 mbIdx)
{
    volatile UINT32 *mbAddr;
    UINT32 offset;
    UINT16 timestamp;

    /* Assertion. */
    ASSERT(NULL != handle);
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    /* Disable Message Buffer Interrupt. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 u64mask = 1;
    FLEXCAN_DisableMbInterrupts(base, u64mask << mbIdx);
#else
    UINT32 u32mask = 1;
    FLEXCAN_DisableMbInterrupts(base, u32mask << mbIdx);
#endif

    /* Update the TX frame 's time stamp by base->MB[offset for CAN FD].CS. */
    mbAddr = &(base->MB[0].CS);
    offset = FLEXCAN_GetFDMailboxOffset(base, mbIdx);
    timestamp = (UINT16)((mbAddr[offset] & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);
    handle->timestamp[mbIdx] = timestamp;

    /* Clean Message Buffer. */
    FLEXCAN_SetFDTxMbConfig(base, mbIdx, TRUE);

    handle->mbState[mbIdx] = (UINT8)kFLEXCAN_StateIdle;
}

/*
 * Aborts the interrupt driven message receive process.
 *
 * This function aborts the interrupt driven message receive process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN FD Message Buffer index.
 */
void FLEXCAN_TransferFDAbortReceive(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT8 mbIdx)
{
    /* Assertion. */
    ASSERT(NULL != handle);
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    /* Disable Message Buffer Interrupt. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 u64mask = 1;
    FLEXCAN_DisableMbInterrupts(base, u64mask << mbIdx);
#else
    UINT32 u32mask = 1;
    FLEXCAN_DisableMbInterrupts(base, u32mask << mbIdx);
#endif

    /* Un-register handle. */
    handle->mbFDFrameBuf[mbIdx] = NULL;
    handle->mbState[mbIdx] = (UINT8)kFLEXCAN_StateIdle;
}
#endif

/*
 * Aborts the interrupt driven message receive process.
 *
 * This function aborts the interrupt driven message receive process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN Message Buffer index.
 */
void FLEXCAN_TransferAbortReceive(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT8 mbIdx)
{
    /* Assertion. */
    ASSERT(NULL != handle);
    ASSERT(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    ASSERT(!FLEXCAN_IsMbOccupied(base, mbIdx));
#endif

    /* Disable Message Buffer Interrupt. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    UINT64 u64mask = 1;
    FLEXCAN_DisableMbInterrupts(base, (u64mask << mbIdx));
#else
    UINT32 u32mask = 1;
    FLEXCAN_DisableMbInterrupts(base, (u32mask << mbIdx));
#endif

    /* Un-register handle. */
    handle->mbFrameBuf[mbIdx] = NULL;
    handle->mbState[mbIdx] = (UINT8)kFLEXCAN_StateIdle;
}

/*
 * Aborts the interrupt driven message receive from Legacy Rx FIFO process.
 *
 * This function aborts the interrupt driven message receive from Legacy Rx FIFO process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 */
void FLEXCAN_TransferAbortReceiveFifo(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle)
{
    /* Assertion. */
    ASSERT(NULL != handle);

    /* Check if Rx FIFO is enabled. */
    if (0U != (base->MCR & CAN_MCR_RFEN_MASK))
    {
        /* Disable Rx Message FIFO Interrupts. */
        FLEXCAN_DisableMbInterrupts(base, (UINT32)kFLEXCAN_RxFifoOverflowFlag | (UINT32)kFLEXCAN_RxFifoWarningFlag |
            (UINT32)kFLEXCAN_RxFifoFrameAvlFlag);

        /* Un-register handle. */
        handle->rxFifoFrameBuf = NULL;
    }

    handle->rxFifoState = (UINT8)kFLEXCAN_StateIdle;
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Aborts the interrupt driven message receive from Enhanced Rx FIFO process.
 *
 * This function aborts the interrupt driven message receive from Rx FIFO process.
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 */
void FLEXCAN_TransferAbortReceiveEnhancedFifo(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle)
{
    /* Assertion. */
    ASSERT(NULL != handle);

    /* Check if Enhanced Rx FIFO is enabled. */
    if (0U != (base->ERFCR & CAN_ERFCR_ERFEN_MASK))
    {
        /* Disable all Rx Message FIFO interrupts. */
        FLEXCAN_DisableInterrupts(base, (UINT64)kFLEXCAN_ERxFifoUnderflowInterruptEnable |
            (UINT64)kFLEXCAN_ERxFifoOverflowInterruptEnable |
            (UINT64)kFLEXCAN_ERxFifoWatermarkInterruptEnable |
            (UINT64)kFLEXCAN_ERxFifoDataAvlInterruptEnable);

        /* Un-register handle. */
        handle->rxFifoFDFrameBuf = NULL;
        /* Clear transfer count. */
        handle->frameNum = 0U;
        handle->transferTotalNum = 0U;
    }

    handle->rxFifoState = (UINT8)kFLEXCAN_StateIdle;
}
#endif

/*
 * Gets the detail index of Mailbox's Timestamp by handle.
 *
 * Then function can only be used when calling non-blocking Data transfer (TX/RX) API,
 * After TX/RX data transfer done (User can get the status by handler's callback function),
 * we can get the detail index of Mailbox's timestamp by handle,
 * Detail non-blocking data transfer API (TX/RX) contain.
 *   -FLEXCAN_TransferSendNonBlocking
 *   -FLEXCAN_TransferFDSendNonBlocking
 *   -FLEXCAN_TransferReceiveNonBlocking
 *   -FLEXCAN_TransferFDReceiveNonBlocking
 *   -FLEXCAN_TransferReceiveFifoNonBlocking
 *
 * param handle FlexCAN handle pointer.
 * param mbIdx The FlexCAN FD Message Buffer index.
 * return the index of mailbox 's timestamp stored in the handle.
 *
 */
UINT32 FLEXCAN_GetTimeStamp(flexcan_handle_t *handle, UINT8 mbIdx)
{
    /* Assertion. */
    ASSERT(NULL != handle);

    return (UINT32)(handle->timestamp[mbIdx]);
}

/*
 * Check unhandle interrupt events
 *
 * param base FlexCAN peripheral base address.
 * return TRUE if unhandled interrupt action exist, FALSE if no unhandlered interrupt action exist.
 */
BOOLEAN FLEXCAN_CheckUnhandleInterruptEvents(volatile IMXCAN_REGISTERS *base)
{
    UINT64 tempmask;
    UINT64 tempflag;
    BOOLEAN fgRet = FALSE;

    /* Checking exist error or status flag. */
    if (0U == (FLEXCAN_GetStatusFlags(base) & (FLEXCAN_ERROR_AND_STATUS_INIT_FLAG | FLEXCAN_WAKE_UP_FLAG)))
    {
        tempmask = (UINT64)base->IMASK1;
        tempflag = (UINT64)base->IFLAG1;

#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        /* Checking whether exist MB interrupt status and legacy RX FIFO interrupt status. */
        tempmask |= ((UINT64)base->IMASK2) << 32;
        tempflag |= ((UINT64)base->IFLAG2) << 32;
#endif
        fgRet = (0U != (tempmask & tempflag));
    }
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
    else if (0U == (FLEXCAN_GetStatusFlags(base) & FLEXCAN_MEMORY_ENHANCED_RX_FIFO_INIT_FLAG))
    {
        /* Checking whether exist enhanced RX FIFO interrupt status. */
        tempmask = (UINT64)base->ERFIER;
        tempflag = (UINT64)base->ERFSR;
        fgRet = (0U != (tempmask & tempflag));
    }
#endif
    else
    {
        fgRet = TRUE;
    }

    return fgRet;
}

/*
 * Sub Handler Data Trasfered Events
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param pResult Pointer to the Handle result.
 *
 * return the status after handle each data transfered event.
 */
status_t FLEXCAN_SubHandlerForDataTransfered(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT32 *pResult)
{
    status_t status = kStatus_FLEXCAN_UnHandled;
    UINT32 result = 0xFFU;

    /* For this implementation, we solve the Message with lowest MB index first. */
    for (result = 0U; result < (UINT32)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base); result++)
    {
        /* Get the lowest unhandled Message Buffer */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        UINT64 u64flag = 1;
        if (0U != FLEXCAN_GetMbStatusFlags(base, u64flag << result))
#else
        UINT32 u32flag = 1;
        if (0U != FLEXCAN_GetMbStatusFlags(base, u32flag << result))
#endif
        {
            if (FLEXCAN_IsMbIntEnabled(base, (UINT8)result))
            {
                break;
            }
        }
    }

    /* find Message to deal with. */
    if (result < (UINT32)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base))
    {
        /* Solve Legacy Rx FIFO interrupt. */
        if (((UINT8)kFLEXCAN_StateIdle != handle->rxFifoState) && (result <= (UINT32)CAN_IFLAG1_BUF7I_SHIFT) &&
            ((base->MCR & CAN_MCR_RFEN_MASK) != 0U))
        {
            UINT32 u32mask = 1;
            switch (u32mask << result)
            {
            case kFLEXCAN_RxFifoOverflowFlag:
                status = kStatus_FLEXCAN_RxFifoOverflow;
                break;

            case kFLEXCAN_RxFifoWarningFlag:
                status = kStatus_FLEXCAN_RxFifoWarning;
                break;

            case kFLEXCAN_RxFifoFrameAvlFlag:
                status = FLEXCAN_ReadRxFifo(base, handle->rxFifoFrameBuf);
                if (kStatus_Success == status)
                {
                    /* Align the current (index 0) rxfifo timestamp to the timestamp array by handle. */
                    handle->timestamp[0] = handle->rxFifoFrameBuf->mfs_0.timestamp;
                    status = kStatus_FLEXCAN_RxFifoIdle;
                }
                FLEXCAN_TransferAbortReceiveFifo(base, handle);
                break;

            default:
                status = kStatus_FLEXCAN_UnHandled;
                break;
            }
        }
        else
        {
            /* Get current State of Message Buffer. */
            switch (handle->mbState[result])
            {
                /* Solve Rx Data Frame. */
            case (UINT8)kFLEXCAN_StateRxData:
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
                if (0U != (base->MCR & CAN_MCR_FDEN_MASK))
                {
                    status = FLEXCAN_ReadFDRxMb(base, (UINT8)result, handle->mbFDFrameBuf[result]);
                    if (kStatus_Success == status)
                    {
                        /* Align the current index of RX MB timestamp to the timestamp array by handle. */
                        handle->timestamp[result] = handle->mbFDFrameBuf[result]->fdmfs_0.timestamp;
                        status = kStatus_FLEXCAN_RxIdle;
                    }
                    handle->copymbFDFrame = handle->mbFDFrameBuf[result];
                }
                else
#endif
                {
                    status = FLEXCAN_ReadRxMb(base, (UINT8)result, handle->mbFrameBuf[result]);
                    if (kStatus_Success == status)
                    {
                        /* Align the current index of RX MB timestamp to the timestamp array by handle. */
                        handle->timestamp[result] = handle->mbFrameBuf[result]->mfs_0.timestamp;
                        status = kStatus_FLEXCAN_RxIdle;
                    }
                    handle->copymbFrame = handle->mbFrameBuf[result];
                }
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
                if (0U != (base->MCR & CAN_MCR_FDEN_MASK))
                {
                    FLEXCAN_TransferFDAbortReceive(base, handle, (UINT8)result);
                }
                else
#endif
                {
                    FLEXCAN_TransferAbortReceive(base, handle, (UINT8)result);
                }
                break;

                /* Solve Rx Remote Frame.  User need to Read the frame in Mail box in time by Read from MB API. */
            case (UINT8)kFLEXCAN_StateRxRemote:
                status = kStatus_FLEXCAN_RxRemote;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
                if (0U != (base->MCR & CAN_MCR_FDEN_MASK))
                {
                    FLEXCAN_TransferFDAbortReceive(base, handle, (UINT8)result);
                }
                else
#endif
                {
                    FLEXCAN_TransferAbortReceive(base, handle, (UINT8)result);
                }
                break;

                /* Solve Tx Data Frame. */
            case (UINT8)kFLEXCAN_StateTxData:
                status = kStatus_FLEXCAN_TxIdle;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
                if (0U != (base->MCR & CAN_MCR_FDEN_MASK))
                {
                    FLEXCAN_TransferFDAbortSend(base, handle, (UINT8)result);
                }
                else
#endif
                {
                    FLEXCAN_TransferAbortSend(base, handle, (UINT8)result);
                }
                break;

                /* Solve Tx Remote Frame. */
            case (UINT8)kFLEXCAN_StateTxRemote:
                handle->mbState[result] = (UINT8)kFLEXCAN_StateRxRemote;
                status = kStatus_FLEXCAN_TxSwitchToRx;
                break;

            default:
                status = kStatus_FLEXCAN_UnHandled;
                break;
            }
        }

        /* Clear resolved Message Buffer IRQ. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        UINT64 u64flag = 1;
        FLEXCAN_ClearMbStatusFlags(base, u64flag << result);
#else
        UINT32 u32flag = 1;
        FLEXCAN_ClearMbStatusFlags(base, u32flag << result);
#endif
    }

    *pResult = result;

    return status;
}

#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
/*
 * Sub Handler Ehanced Rx FIFO event
 *
 * param base FlexCAN peripheral base address.
 * param handle FlexCAN handle pointer.
 * param flags FlexCAN interrupt flags.
 *
 * return the status after handle Ehanced Rx FIFO event.
 */
static status_t FLEXCAN_SubHandlerForEhancedRxFifo(volatile IMXCAN_REGISTERS *base, flexcan_handle_t *handle, UINT64 flags)
{
    UINT32 watermark = ((base->ERFCR & CAN_ERFCR_ERFWM_MASK) >> CAN_ERFCR_ERFWM_SHIFT) + 1U;
    UINT32 transferFrames;

    status_t status;
    /* Solve Ehanced Rx FIFO interrupt. */
    if ((0u != (flags & (UINT64)kFLEXCAN_ERxFifoDataAvlIntFlag)) && (0u != (base->ERFIER & CAN_ERFIER_ERFDAIE_MASK)))
    {
        /* Whether still has CAN messages remaining to be received. */
        if (handle->frameNum > 0U)
        {
            status = FLEXCAN_ReadEnhancedRxFifo(base, handle->rxFifoFDFrameBuf);

            if (kStatus_Success == status)
            {
                handle->rxFifoFDFrameBuf++;
                handle->frameNum--;
            }
            else
            {
                return status;
            }
        }
        if (handle->frameNum == 0U)
        {
            /* Stop receiving Ehanced Rx FIFO when the transmission is over. */
            FLEXCAN_TransferAbortReceiveEnhancedFifo(base, handle);
            status = kStatus_FLEXCAN_RxFifoIdle;
        }
        else
        {
            /* Continue use data avaliable interrupt. */
            status = kStatus_FLEXCAN_RxFifoBusy;
        }
    }
    else if ((0u != (flags & (UINT64)kFLEXCAN_ERxFifoWatermarkIntFlag)) &&
        (0u != (base->ERFIER & CAN_ERFIER_ERFWMIIE_MASK)))
    {
        /* Whether the number of CAN messages remaining to be received is greater than the watermark. */
        transferFrames = (handle->frameNum > watermark) ? watermark : handle->frameNum;

        for (UINT32 i = 0; i < transferFrames; i++)
        {
            status = FLEXCAN_ReadEnhancedRxFifo(base, handle->rxFifoFDFrameBuf);

            if (kStatus_Success == status)
            {
                handle->rxFifoFDFrameBuf++;
                handle->frameNum--;
            }
            else
            {
                return status;
            }
        }
        if (handle->frameNum == 0U)
        {
            /* Stop receiving Ehanced Rx FIFO when the transmission is over. */
            FLEXCAN_TransferAbortReceiveEnhancedFifo(base, handle);
            status = kStatus_FLEXCAN_RxFifoIdle;
        }
        else if (handle->frameNum < watermark)
        {
            /* Disable watermark interrupt and enable data avaliable interrupt. */
            FLEXCAN_DisableInterrupts(base, (UINT64)kFLEXCAN_ERxFifoWatermarkInterruptEnable);
            FLEXCAN_EnableInterrupts(base, (UINT64)kFLEXCAN_ERxFifoDataAvlInterruptEnable);
            status = kStatus_FLEXCAN_RxFifoBusy;
        }
        else
        {
            /* Continue use watermark interrupt. */
            status = kStatus_FLEXCAN_RxFifoBusy;
        }
    }
    else if ((0u != (flags & (UINT64)kFLEXCAN_ERxFifoUnderflowIntFlag)) &&
        (0u != (base->ERFIER & CAN_ERFIER_ERFUFWIE_MASK)))
    {
        status = kStatus_FLEXCAN_RxFifoUnderflow;
        FLEXCAN_ClearStatusFlags(base, (UINT64)kFLEXCAN_ERxFifoUnderflowIntFlag);
    }
    else if ((0u != (flags & (UINT64)kFLEXCAN_ERxFifoOverflowIntFlag)) &&
        (0u != (base->ERFIER & CAN_ERFIER_ERFOVFIE_MASK)))
    {
        status = kStatus_FLEXCAN_RxOverflow;
        FLEXCAN_ClearStatusFlags(base, (UINT64)kFLEXCAN_ERxFifoOverflowIntFlag);
    }
    else
    {
        status = kStatus_FLEXCAN_UnHandled;
    }

    return status;
}
#endif




#pragma warning(pop)

IMXCAN_NONPAGED_SEGMENT_END;