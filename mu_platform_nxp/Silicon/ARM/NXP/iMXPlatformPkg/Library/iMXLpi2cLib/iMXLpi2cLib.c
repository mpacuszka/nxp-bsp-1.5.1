/*
 * Copyright 2022-2023 NXP
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

#include <Base.h>
#include <Uefi.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>

#include "iMXLpi2cLib_hw.h"
#include "iMXI2cLib.h"

/* IMX8 LPI2C - Peripheral instance base addresses */

/** Peripheral LPI2C0 base address */
#define IMX8_LPI2C0_BASE                         (0x5a800000)
/** Peripheral LPI2C0 base pointer */
#define IMX8_LPI2C0                              ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C0_BASE)
/** Peripheral LPI2C1 base address */
#define IMX8_LPI2C1_BASE                         (0x5a810000)
/** Peripheral LPI2C1 base pointer */
#define IMX8_LPI2C1                              ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C1_BASE)
/** Peripheral LPI2C2 base address */
#define IMX8_LPI2C2_BASE                         (0x5a820000)
/** Peripheral LPI2C2 base pointer */
#define IMX8_LPI2C2                              ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C2_BASE)
/** Peripheral LPI2C3 base address */
#define IMX8_LPI2C3_BASE                         (0x5a830000)
/** Peripheral LPI2C3 base pointer */
#define IMX8_LPI2C3                              ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C3_BASE)
/** Peripheral LPI2C4 base address */
#define IMX8_LPI2C4_BASE                         (0x5a840000)
/** Peripheral LPI2C4 base pointer */
#define IMX8_LPI2C4                              ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C4_BASE)
/** Peripheral I2C0_MIPI_CSI0 base address */
#define IMX8_LPI2C5_BASE                         (0x58226000)
/** Peripheral I2C0_MIPI_CSI0 base pointer */
#define IMX8_LPI2C5                              ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C5_BASE)
/** Peripheral I2C0_PARALLEL base address */
#define IMX8_LPI2C6_BASE                         (0x58266000)
/** Peripheral I2C0_PARALLEL base pointer */
#define IMX8_LPI2C6                              ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C6_BASE)
/** Peripheral I2C0_MIPI_CSI1 base address */
#define IMX8_LPI2C7_BASE                         (0x58246000)
/** Peripheral I2C0_MIPI_CSI1 base pointer */
#define IMX8_LPI2C7                              ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C7_BASE)
/** Peripheral I2C0_MIPI_LVDS0_OR_I2C0_MIPI_DSI0 base address */
#define IMX8_LPI2C8_BASE                         (0x56226000)
/** Peripheral I2C0_MIPI_LVDS0_OR_I2C0_MIPI_DSI0 base pointer */
#define IMX8_LPI2C8                              ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C8_BASE)
/** Peripheral I2C0_MIPI_LVDS1_OR_I2C0_LVDS0 base address */
#define IMX8_LPI2C9_BASE                         (0x56246000)
/** Peripheral I2C0_MIPI_LVDS1_OR_I2C0_LVDS0 base pointer */
#define IMX8_LPI2C9                              ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C9_BASE)
/** Peripheral I2C1_LVDS0 base address */
#define IMX8_LPI2C10_BASE                        (0x56247000)
/** Peripheral I2C1_LVDS0 base pointer */
#define IMX8_LPI2C10                             ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C10_BASE)
/** Peripheral I2C0_LVDS1 base address */
#define IMX8_LPI2C11_BASE                        (0x57246000)
/** Peripheral I2C0_LVDS1 base pointer */
#define IMX8_LPI2C11                             ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C11_BASE)
/** Peripheral I2C1_LVDS1 base address */
#define IMX8_LPI2C12_BASE                        (0x57247000)
/** Peripheral I2C1_LVDS1 base pointer */
#define IMX8_LPI2C12                             ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C12_BASE)
/** Peripheral I2C0_MIPI_DSI1 base address */
#define IMX8_LPI2C13_BASE                        (0x57226000)
/** Peripheral I2C0_MIPI_DSI1 base pointer */
#define IMX8_LPI2C13                             ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C13_BASE)
/** Peripheral I2C0_HDMI base address */
#define IMX8_LPI2C14_BASE                        (0x57226000)
/** Peripheral I2C0_HDMI base pointer */
#define IMX8_LPI2C14                             ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C14_BASE)
/** Peripheral CM40_I2C base address */
#define IMX8_LPI2C15_BASE                        (0x37230000)
/** Peripheral CM40_I2C base pointer */
#define IMX8_LPI2C15                             ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C15_BASE)
/** Peripheral CM41_I2C base address */
#define IMX8_LPI2C16_BASE                        (0x3b230000)
/** Peripheral CM41_I2C base pointer */
#define IMX8_LPI2C16                             ((IMX_LPI2C_REGISTERS *)IMX8_LPI2C16_BASE)

/* IMX9 LPI2C - Peripheral instance base addresses */

/** Peripheral LPI2C1 base address */
#define IMX9_LPI2C1_BASE                         (0x44340000)
/** Peripheral LPI2C1 base pointer */
#define IMX9_LPI2C1                              ((IMX_LPI2C_REGISTERS *)IMX9_LPI2C1_BASE)
/** Peripheral LPI2C2 base address */
#define IMX9_LPI2C2_BASE                         (0x44350000)
/** Peripheral LPI2C2 base pointer */
#define IMX9_LPI2C2                              ((IMX_LPI2C_REGISTERS *)IMX9_LPI2C2_BASE)
/** Peripheral LPI2C3 base address */
#define IMX9_LPI2C3_BASE                         (0x42530000)
/** Peripheral LPI2C3 base pointer */
#define IMX9_LPI2C3                              ((IMX_LPI2C_REGISTERS *)IMX9_LPI2C3_BASE)
/** Peripheral LPI2C4 base address */
#define IMX9_LPI2C4_BASE                         (0x42540000)
/** Peripheral LPI2C4 base pointer */
#define IMX9_LPI2C4                              ((IMX_LPI2C_REGISTERS *)IMX9_LPI2C4_BASE)
/** Peripheral LPI2C5 base address */
#define IMX9_LPI2C5_BASE                         (0x426B0000)
/** Peripheral LPI2C5 base pointer */
#define IMX9_LPI2C5                              ((IMX_LPI2C_REGISTERS *)IMX9_LPI2C5_BASE)
/** Peripheral LPI2C6 base address */
#define IMX9_LPI2C6_BASE                         (0x426C0000)
/** Peripheral LPI2C6 base pointer */
#define IMX9_LPI2C6                              ((IMX_LPI2C_REGISTERS *)IMX9_LPI2C6_BASE)
/** Peripheral LPI2C7 base address */
#define IMX9_LPI2C7_BASE                         (0x426D0000)
/** Peripheral LPI2C7 base pointer */
#define IMX9_LPI2C7                              ((IMX_LPI2C_REGISTERS *)IMX9_LPI2C7_BASE)
/** Peripheral LPI2C8 base address */
#define IMX9_LPI2C8_BASE                         (0x426E0000)
/** Peripheral LPI2C8 base pointer */
#define IMX9_LPI2C8                              ((IMX_LPI2C_REGISTERS *)IMX9_LPI2C8_BASE)

/** Array initializer of IMX8 LPI2C peripheral base addresses */
#define IMX8_LPI2C_BASE_ADDRS                    { IMX8_LPI2C0_BASE, IMX8_LPI2C1_BASE, IMX8_LPI2C2_BASE, IMX8_LPI2C3_BASE, IMX8_LPI2C4_BASE, IMX8_LPI2C5_BASE, IMX8_LPI2C6_BASE, IMX8_LPI2C7_BASE, IMX8_LPI2C8_BASE, IMX8_LPI2C9_BASE, IMX8_LPI2C10_BASE, IMX8_LPI2C11_BASE, IMX8_LPI2C12_BASE, IMX8_LPI2C13_BASE, IMX8_LPI2C14_BASE, IMX8_LPI2C15_BASE, IMX8_LPI2C16_BASE }
/** Array initializer of IMX8 LPI2C peripheral base pointers */
#define IMX8_LPI2C_BASE_PTRS                     { IMX8_LPI2C0, IMX8_LPI2C1, IMX8_LPI2C2, IMX8_LPI2C3, IMX8_LPI2C4, IMX8_LPI2C5, IMX8_LPI2C6, IMX8_LPI2C7, IMX8_LPI2C8, IMX8_LPI2C9, IMX8_LPI2C10, IMX8_LPI2C11, IMX8_LPI2C12, IMX8_LPI2C13, IMX8_LPI2C14, IMX8_LPI2C15, IMX8_LPI2C16 }

/** Array initializer of IMX9 LPI2C peripheral base addresses */
#define IMX9_LPI2C_BASE_ADDRS                    { 0x0U, IMX9_LPI2C1_BASE, IMX9_LPI2C2_BASE, IMX9_LPI2C3_BASE, IMX9_LPI2C4_BASE, IMX9_LPI2C5_BASE, IMX9_LPI2C6_BASE, IMX9_LPI2C7_BASE, IMX9_LPI2C8_BASE }
/** Array initializer of IMX9 LPI2C peripheral base pointers */
#define IMX9_LPI2C_BASE_PTRS                     { (IMX_LPI2C_REGISTERS *)0x0U, IMX9_LPI2C1, IMX9_LPI2C2, IMX9_LPI2C3, IMX9_LPI2C4, IMX9_LPI2C5, IMX9_LPI2C6, IMX9_LPI2C7, IMX9_LPI2C8 }

/*! @brief Array to map IMX8 LPI2C instance number to base pointer. */
static IMX_LPI2C_REGISTERS * const imx8Lpi2cBases[] = IMX8_LPI2C_BASE_PTRS;

/*! @brief Array to map IMX9 LPI2C instance number to base pointer. */
static IMX_LPI2C_REGISTERS * const imx9Lpi2cBases[] = IMX9_LPI2C_BASE_PTRS;

#define INSTANCES_ARRAY_SIZE ((ARRAY_SIZE(imx8Lpi2cBases) > ARRAY_SIZE(imx9Lpi2cBases))? ARRAY_SIZE(imx8Lpi2cBases) : ARRAY_SIZE(imx9Lpi2cBases))

/*! @brief LPI2C driver context. */
typedef struct _IMX_LPI2C_CONTEXT {

    UINT8 ControllerID;

    BOOLEAN IsInitialized;
 
} IMX_LPI2C_CONTEXT, *PIMX_LPI2C_CONTEXT;

/*! @brief Array to store LPI2C driver context. */
static IMX_LPI2C_CONTEXT DeviceCtxPtrs[INSTANCES_ARRAY_SIZE] = { 0 };

/* ! @brief LPI2C master fifo commands. */
enum
{
    kTxDataCmd = IMX_LPI2C_MTDR_CMD(0x0U), /*!< Transmit DATA[7:0] */
    kRxDataCmd = IMX_LPI2C_MTDR_CMD(0X1U), /*!< Receive (DATA[7:0] + 1) bytes */
    kStopCmd = IMX_LPI2C_MTDR_CMD(0x2U), /*!< Generate STOP condition */
    kStartCmd = IMX_LPI2C_MTDR_CMD(0x4U), /*!< Generate(repeated) START and transmit address in DATA[[7:0] */
};

/*!
 * @brief Default watermark values.
 *
 * The default watermarks are set to zero.
 */
enum
{
    kDefaultTxWatermark = 0,
    kDefaultRxWatermark = 0,
};


void LPI2C_MasterSetBaudRate(IMX_LPI2C_REGISTERS *base, UINT32 sourceClock_Hz, UINT32 baudRate_Hz);

/*!
 * brief Returns an instance number given a base address.
 *
 * If an invalid base address is passed, -1 is returned.
 *                                                                                         
 * param base The LPI2C peripheral base address.
 * return LPI2C instance number starting from 0. 
 */
INT8 LPI2C_GetInstance(IMX_LPI2C_REGISTERS *base)
{
    INT8 result = -1;
    INT8 instance;

    if (base == NULL) {
        DEBUG((DEBUG_ERROR, "%a: NULL base address\n", __FUNCTION__));
        return result;
    }
    for (instance = 0U; instance < ARRAY_SIZE(imx8Lpi2cBases); ++instance) {
        if (imx8Lpi2cBases[instance] == base) {
            result = (INT8)instance;
            break;
        }
    }
    if (result == -1) {
        for (instance = 0U; instance < ARRAY_SIZE(imx9Lpi2cBases); ++instance) {
            if (imx9Lpi2cBases[instance] == base) {
                result = (INT8)instance;
                break;
            }
        }
    }
    if (result < 0) {
        DEBUG((DEBUG_ERROR, "%a: Invalid base address 0x%lx\n", __FUNCTION__, base));
    }
    return result;
}

/*!
 * @brief Computes a cycle count for a given time in nanoseconds.
 * @param sourceClock_Hz LPI2C functional clock frequency in Hertz.
 * @param width_ns Desired with in nanoseconds.
 * @param maxCycles Maximum cycle count, determined by the number of bits wide the cycle count field is.
 * @param prescaler LPI2C prescaler setting.
 */
static UINT32 LPI2C_GetCyclesForWidth(UINT32 sourceClock_Hz,
    UINT32 width_ns,
    UINT32 maxCycles,
    UINT32 prescaler)
{
    if (sourceClock_Hz == 0U) {
        DEBUG((DEBUG_ERROR, "%a: Invalid input sourceClock_Hz parameter %u\n", __FUNCTION__, sourceClock_Hz));
        return 0;
    }

    UINT32 divider = 1U;

    while (prescaler != 0U) {
        divider *= 2U;
        prescaler--;
    }

    UINT32 busCycle_ns = 1000000U / (sourceClock_Hz / divider / 1000U);
    UINT32 cycles = 0U;

    /* Search for the cycle count just below the desired glitch width. */
    while ((((cycles + 1U) * busCycle_ns) < width_ns) && (cycles + 1U < maxCycles)) {
        ++cycles;
    }

    /* If we end up with zero cycles, then set the filter to a single cycle unless the */
    /* bus clock is greater than 10x the desired glitch width. */
    if ((cycles == 0U) && (busCycle_ns <= (width_ns * 10U))) {
        cycles = 1U;
    }

    return cycles;
}

/*!
 * @brief Enables or disables the LPI2C module as master.
 *
 * @param base The LPI2C peripheral base address.
 * @param enable Pass true to enable or false to disable the specified LPI2C as master.
 */
static inline void LPI2C_MasterEnable(IMX_LPI2C_REGISTERS *base, BOOLEAN enable)
{
    MmioWrite32((UINTN)&base->MCR, (MmioRead32((UINTN)&base->MCR) &
        ~IMX_LPI2C_MCR_MEN_MASK) | IMX_LPI2C_MCR_MEN(enable));
}

/*!
 * @brief Sets the watermarks for LPI2C master FIFOs.
 *
 * @param base The LPI2C peripheral base address.
 * @param txWords Transmit FIFO watermark value in words. The #kLPI2C_MasterTxReadyFlag flag is set whenever
 *      the number of words in the transmit FIFO is equal or less than @a txWords. Writing a value equal or
 *      greater than the FIFO size is truncated.
 * @param rxWords Receive FIFO watermark value in words. The #kLPI2C_MasterRxReadyFlag flag is set whenever
 *      the number of words in the receive FIFO is greater than @a rxWords. Writing a value equal or greater
 *      than the FIFO size is truncated.
 */
static inline void LPI2C_MasterSetWatermarks(IMX_LPI2C_REGISTERS *base, size_t txWords, size_t rxWords)
{
    MmioWrite32((UINTN)&base->MFCR, IMX_LPI2C_MFCR_TXWATER(txWords) | IMX_LPI2C_MFCR_RXWATER(rxWords));
}

/*!
 * @brief Gets the current number of words in the LPI2C master FIFOs.
 *
 * @param base The LPI2C peripheral base address.
 * @param[out] txCount Pointer through which the current number of words in the transmit FIFO is returned.
 *      Pass NULL if this value is not required.
 * @param[out] rxCount Pointer through which the current number of words in the receive FIFO is returned.
 *      Pass NULL if this value is not required.
 */
static inline void LPI2C_MasterGetFifoCounts(IMX_LPI2C_REGISTERS *base, size_t *rxCount, size_t* txCount)
{
    if (NULL != txCount)
    {
        *txCount = (MmioRead32((UINTN)&base->MFSR) & IMX_LPI2C_MFSR_TXCOUNT_MASK) >> IMX_LPI2C_MFSR_TXCOUNT_SHIFT;
    }
    if (NULL != rxCount)
    {
        *rxCount = (MmioRead32((UINTN)&base->MFSR) & IMX_LPI2C_MFSR_RXCOUNT_MASK) >> IMX_LPI2C_MFSR_RXCOUNT_SHIFT;
    }
}

/*! @name Status */
/*@{*/

/*!
 * @brief Gets the LPI2C master status flags.
 *
 * A bit mask with the state of all LPI2C master status flags is returned. For each flag, the corresponding bit
 * in the return value is set if the flag is asserted.
 *
 * @param base The LPI2C peripheral base address.
 * @return State of the status flags:
 *         - 1: related status flag is set.
 *         - 0: related status flag is not set.
 * @see _lpi2c_master_flags
 */
static inline UINT32 LPI2C_MasterGetStatusFlags(IMX_LPI2C_REGISTERS *base)
{
    return MmioRead32((UINTN)&base->MSR);
}

/*!
 * @brief Clears the LPI2C master status flag state.
 *
 * The following status register flags can be cleared:
 * - #kLPI2C_MasterEndOfPacketFlag
 * - #kLPI2C_MasterStopDetectFlag
 * - #kLPI2C_MasterNackDetectFlag
 * - #kLPI2C_MasterArbitrationLostFlag
 * - #kLPI2C_MasterFifoErrFlag
 * - #kLPI2C_MasterPinLowTimeoutFlag
 * - #kLPI2C_MasterDataMatchFlag
 *
 * Attempts to clear other flags has no effect.
 *
 * @param base The LPI2C peripheral base address.
 * @param statusMask A bitmask of status flags that are to be cleared. The mask is composed of
 *  _lpi2c_master_flags enumerators OR'd together. You may pass the result of a previous call to
 *  LPI2C_MasterGetStatusFlags().
 * @see _lpi2c_master_flags.
 */
static inline void LPI2C_MasterClearStatusFlags(IMX_LPI2C_REGISTERS *base, UINT32 statusMask)
{
    MmioWrite32((UINTN)&base->MSR, statusMask);
}

/*!
 * @brief Gets Tx Fifo size.
 *
 * @param base The LPI2C peripheral base address.

* @return Size of LPI2C Tx Fifo
*/
static inline UINT32 LPI2C_GetTxFifoSize(IMX_LPI2C_REGISTERS *base)
{
    return (1UL << ((MmioRead32((UINTN)&base->PARAM) & IMX_LPI2C_PARAM_MTXFIFO_MASK) >>
        IMX_LPI2C_PARAM_MTXFIFO_SHIFT));
}

/*!
 * @brief Gets Rx Fifo size.
 *
 * @param base The LPI2C peripheral base address.

* @return Size of LPI2C Rx Fifo
*/
static inline UINT32 LPI2C_GetRxFifoSize(IMX_LPI2C_REGISTERS *base)
{
    return (1UL << ((MmioRead32((UINTN)&base->PARAM) & IMX_LPI2C_PARAM_MRXFIFO_MASK) >>
        IMX_LPI2C_PARAM_MRXFIFO_SHIFT));
}

/*!
 * @brief Convert provided flags to status code, and clear any errors if present.
 * @param base The LPI2C peripheral base address.
 * @param status Current status flags value that will be checked.
 * @retval #kStatus_Success
 * @retval #kStatus_LPI2C_PinLowTimeout
 * @retval #kStatus_LPI2C_ArbitrationLost
 * @retval #kStatus_LPI2C_Nak
 * @retval #kStatus_LPI2C_FifoError
 */
 /* Not static so it can be used from fsl_lpi2c_edma.c. */
i2c_status LPI2C_MasterCheckAndClearError(IMX_LPI2C_REGISTERS *base, UINT32 status)
{
    i2c_status result = kStatus_Success;

    /* Check for error. These errors cause a stop to automatically be sent. We must */
    /* clear the errors before a new transfer can start. */
    status &= (UINT32)kLPI2C_MasterErrorFlags;
    if (0U != status) {
        /* Select the correct error code. Ordered by severity, with bus issues first. */
        if (0U != (status & (UINT32)kLPI2C_MasterPinLowTimeoutFlag)) {
            result = kStatus_LPI2C_PinLowTimeout;
        }
        else {
            if (0U != (status & (UINT32)kLPI2C_MasterArbitrationLostFlag)) {
                result = kStatus_LPI2C_ArbitrationLost;
            }
            else {
                if (0U != (status & (UINT32)kLPI2C_MasterNackDetectFlag)) {
                    result = kStatus_LPI2C_Nak;
                }
                else {
                    if (0U != (status & (UINT32)kLPI2C_MasterFifoErrFlag)) {
                        result = kStatus_LPI2C_FifoError;
                    }
                    else {
                        ; /* Intentional empty */
                    }
                }
            }
        }
        /* Clear the flags. */
        LPI2C_MasterClearStatusFlags(base, status);

        /* Reset fifos. These flags clear automatically. */
        MmioWrite32((UINTN)&base->MCR, MmioRead32((UINTN)&base->MCR) | 
            IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK);
    }
    else {
        ; /* Intentional empty */
    }

    return result;
}

/*!
 * @brief Wait until there is room in the tx fifo.
 * @param base The LPI2C peripheral base address.
 * @retval #kStatus_Success
 * @retval #kStatus_LPI2C_PinLowTimeout
 * @retval #kStatus_LPI2C_ArbitrationLost
 * @retval #kStatus_LPI2C_Nak
 * @retval #kStatus_LPI2C_FifoError
 */
static i2c_status LPI2C_MasterWaitForTxReady(IMX_LPI2C_REGISTERS *base)
{
    i2c_status result = kStatus_Success;
    UINT32 status;
    size_t txCount;
    size_t txFifoSize = (size_t)LPI2C_GetTxFifoSize(base);

#if I2C_RETRY_TIMES != 0U
    UINT32 waitTimes = I2C_RETRY_TIMES;
#endif
    do {
        /* Get the number of words in the tx fifo and compute empty slots. */
        LPI2C_MasterGetFifoCounts(base, NULL, &txCount);
        txCount = txFifoSize - txCount;

        /* Check for error flags. */
        status = LPI2C_MasterGetStatusFlags(base);
        result = LPI2C_MasterCheckAndClearError(base, status);
        if (kStatus_Success != result) {
            break;
        }
#if I2C_RETRY_TIMES != 0U
        waitTimes--;
    } while ((0U == txCount) && (0U != waitTimes));

    if (0U == waitTimes) {
        result = kStatus_LPI2C_Timeout;
    }
#else
    } while (0U == txCount);
#endif

    return result;
}

/*!
 * @brief Make sure the bus isn't already busy.
 *
 * A busy bus is allowed if we are the one driving it.
 *
 * @param base The LPI2C peripheral base address.
 * @retval #kStatus_Success
 * @retval #kStatus_LPI2C_Busy
 */
 /* Not static so it can be used from fsl_lpi2c_edma.c. */
i2c_status LPI2C_CheckForBusyBus(IMX_LPI2C_REGISTERS *base)
{
    i2c_status ret = kStatus_Success;

    UINT32 status = LPI2C_MasterGetStatusFlags(base);
    if ((0U != (status & (UINT32)kLPI2C_MasterBusBusyFlag)) && 
        (0U == (status & (UINT32)kLPI2C_MasterBusyFlag))) {
        ret = kStatus_LPI2C_Busy;
    }

    return ret;
}

/*!
 * brief Provides a default configuration for the LPI2C master peripheral.
 *
 * This function provides the following default configuration for the LPI2C master peripheral:
 * code
 *  masterConfig->enableMaster            = true;
 *  masterConfig->debugEnable             = false;
 *  masterConfig->ignoreAck               = false;
 *  masterConfig->pinConfig               = kLPI2C_2PinOpenDrain;
 *  masterConfig->baudRate_Hz             = 100000U;
 *  masterConfig->busIdleTimeout_ns       = 0U;
 *  masterConfig->pinLowTimeout_ns        = 0U;
 *  masterConfig->sdaGlitchFilterWidth_ns = 0U;
 *  masterConfig->sclGlitchFilterWidth_ns = 0U;
 *  masterConfig->hostRequest.enable      = false;
 *  masterConfig->hostRequest.source      = kLPI2C_HostRequestExternalPin;
 *  masterConfig->hostRequest.polarity    = kLPI2C_HostRequestPinActiveHigh;
 * endcode
 *
 * After calling this function, you can override any settings in order to customize the configuration,
 * prior to initializing the master driver with LPI2C_MasterInit().
 *
 * param[out] masterConfig User provided configuration structure for default values. Refer to #lpi2c_master_config_t.
 */
static void LPI2C_MasterGetDefaultConfig(lpi2c_master_config_t* masterConfig)
{
    masterConfig->baudRate_Hz = 100000U;
    masterConfig->busIdleTimeout_ns = 0U;
    masterConfig->pinLowTimeout_ns = 0U;
    masterConfig->sdaGlitchFilterWidth_ns = 0U;
    masterConfig->sclGlitchFilterWidth_ns = 0U;
}


/*!
 * brief Initializes the LPI2C master peripheral.
 *
 * This function enables the peripheral clock and initializes the LPI2C master peripheral as described by the user
 * provided configuration. A software reset is performed prior to configuration.
 *
 * param base The LPI2C peripheral base address.
 * param masterConfig User provided peripheral configuration. Use LPI2C_MasterGetDefaultConfig() to get a set of
 * defaults
 *      that you can override.
 * param sourceClock_Hz Frequency in Hertz of the LPI2C functional clock. Used to calculate the baud rate divisors,
 *      filter widths, and timeout periods.
 */
void LPI2C_MasterInit(IMX_LPI2C_REGISTERS *base, const lpi2c_master_config_t* masterConfig, UINT32 sourceClock_Hz)
{
    UINT32 prescaler;
    UINT32 cycles;
    UINT32 cfgr2;
    UINT32 value;

    /* Reset peripheral before configuring it. */
    LPI2C_MasterReset(base);

    /* host request */
    value = MmioRead32((UINTN)&base->MCFGR0);
    value &= (~(IMX_LPI2C_MCFGR0_HREN_MASK | IMX_LPI2C_MCFGR0_HRPOL_MASK | IMX_LPI2C_MCFGR0_HRSEL_MASK));
    MmioWrite32((UINTN)&base->MCFGR0, value);

    /* pin config and ignore ack */
    value = MmioRead32((UINTN)&base->MCFGR1);
    value &= ~(IMX_LPI2C_MCFGR1_PINCFG_MASK | IMX_LPI2C_MCFGR1_IGNACK_MASK);
    MmioWrite32((UINTN)&base->MCFGR1, value);

    LPI2C_MasterSetWatermarks(base, (size_t)kDefaultTxWatermark, (size_t)kDefaultRxWatermark);

    /* Configure glitch filters. */
    cfgr2 = MmioRead32((UINTN)&base->MCFGR2);

    /* Configure baudrate after the SDA/SCL glitch filter setting,
       since the baudrate calculation needs them as parameter. */
    LPI2C_MasterSetBaudRate(base, sourceClock_Hz, masterConfig->baudRate_Hz);

    /* Configure bus idle and pin low timeouts after baudrate setting,
       since the timeout calculation needs prescaler as parameter. */
    prescaler = (MmioRead32((UINTN)&base->MCFGR1) & IMX_LPI2C_MCFGR1_PRESCALE_MASK) >> 
        IMX_LPI2C_MCFGR1_PRESCALE_SHIFT;

    if (0U != (masterConfig->busIdleTimeout_ns)) {
        cycles = LPI2C_GetCyclesForWidth(sourceClock_Hz, masterConfig->busIdleTimeout_ns,
            (IMX_LPI2C_MCFGR2_BUSIDLE_MASK >> IMX_LPI2C_MCFGR2_BUSIDLE_SHIFT), prescaler);
        cfgr2 &= ~IMX_LPI2C_MCFGR2_BUSIDLE_MASK;
        cfgr2 |= IMX_LPI2C_MCFGR2_BUSIDLE(cycles);
    }
    MmioWrite32((UINTN)&base->MCFGR2, cfgr2);
    if (0U != masterConfig->pinLowTimeout_ns) {
        cycles = LPI2C_GetCyclesForWidth(sourceClock_Hz, masterConfig->pinLowTimeout_ns / 256U,
            (IMX_LPI2C_MCFGR2_BUSIDLE_MASK >> IMX_LPI2C_MCFGR2_BUSIDLE_SHIFT), prescaler);
        MmioWrite32((UINTN)&base->MCFGR3, (MmioRead32((UINTN)&base->MCFGR3) & ~IMX_LPI2C_MCFGR3_PINLOW_MASK) |
            IMX_LPI2C_MCFGR3_PINLOW(cycles));
    }

    LPI2C_MasterEnable(base, TRUE);
}

/*!
 * brief Sends a START signal and slave address on the I2C bus.
 *
 * This function is used to initiate a new master mode transfer. First, the bus state is checked to ensure
 * that another master is not occupying the bus. Then a START signal is transmitted, followed by the
 * 7-bit address specified in the a address parameter. Note that this function does not actually wait
 * until the START and address are successfully sent on the bus before returning.
 *
 * param base The LPI2C peripheral base address.
 * param address 7-bit slave device address, in bits [6:0].
 * param dir Master transfer direction, either #kLPI2C_Read or #kLPI2C_Write. This parameter is used to set
 *      the R/w bit (bit 0) in the transmitted slave address.
 * retval #kStatus_Success START signal and address were successfully enqueued in the transmit FIFO.
 * retval #kStatus_LPI2C_Busy Another master is currently utilizing the bus.
 */
i2c_status LPI2C_MasterStart(IMX_LPI2C_REGISTERS *base, UINT8 address, lpi2c_direction_t dir)
{
    /* Return an error if the bus is already in use not by us. */
    i2c_status result = LPI2C_CheckForBusyBus(base);
    if (kStatus_Success == result) {
        /* Clear all flags. */
        LPI2C_MasterClearStatusFlags(base, (UINT32)kLPI2C_MasterClearFlags);

        /* Turn off auto-stop option. */
        MmioWrite32((UINTN)&base->MCFGR1, MmioRead32((UINTN)&base->MCFGR1) & ~IMX_LPI2C_MCFGR1_AUTOSTOP_MASK);

        /* Wait until there is room in the fifo. */
        result = LPI2C_MasterWaitForTxReady(base);
        if (kStatus_Success == result) {
            /* Issue start command. */
            MmioWrite32((UINTN)&base->MTDR, (UINT32)kStartCmd | (((UINT32)address << 1U) | (UINT32)dir));
        }
    }

    return result;
}

/*!
 * brief Sends a STOP signal on the I2C bus.
 *
 * This function does not return until the STOP signal is seen on the bus, or an error occurs.
 *
 * param base The LPI2C peripheral base address.
 * retval #kStatus_Success The STOP signal was successfully sent on the bus and the transaction terminated.
 * retval #kStatus_LPI2C_Busy Another master is currently utilizing the bus.
 * retval #kStatus_LPI2C_Nak The slave device sent a NAK in response to a byte.
 * retval #kStatus_LPI2C_FifoError FIFO under run or overrun.
 * retval #kStatus_LPI2C_ArbitrationLost Arbitration lost error.
 * retval #kStatus_LPI2C_PinLowTimeout SCL or SDA were held low longer than the timeout.
 */
i2c_status LPI2C_MasterStop(IMX_LPI2C_REGISTERS *base)
{
    /* Wait until there is room in the fifo. */
    i2c_status result = LPI2C_MasterWaitForTxReady(base);
    if (kStatus_Success == result) {
        /* Send the STOP signal */
        MmioWrite32((UINTN)&base->MTDR, (UINT32)kStopCmd);

        /* Wait for the stop detected flag to set, indicating the transfer has completed on the bus. */
        /* Also check for errors while waiting. */
#if I2C_RETRY_TIMES != 0U
        uint32_t waitTimes = I2C_RETRY_TIMES;
#endif

#if I2C_RETRY_TIMES != 0U
        while ((result == kStatus_Success) && (0U != waitTimes))
        {
            waitTimes--;
#else
        while (result == kStatus_Success) {
#endif
            UINT32 status = LPI2C_MasterGetStatusFlags(base);

            /* Check for error flags. */
            result = LPI2C_MasterCheckAndClearError(base, status);

            /* Check if the stop was sent successfully. */
            if ((0U != (status & (UINT32)kLPI2C_MasterStopDetectFlag)) &&
                (0U != (status & (UINT32)kLPI2C_MasterTxReadyFlag))) {
                LPI2C_MasterClearStatusFlags(base, (UINT32)kLPI2C_MasterStopDetectFlag);
                break;
            }
        }

#if I2C_RETRY_TIMES != 0U
        if (0U == waitTimes) {
            result = kStatus_LPI2C_Timeout;
        }
#endif
    }

    return result;
}

/*!
 * brief Performs a polling receive transfer on the I2C bus.
 *
 * param base  The LPI2C peripheral base address.
 * param rxBuff The pointer to the data to be transferred.
 * param rxSize The length in bytes of the data to be transferred.
 * retval #kStatus_Success Data was received successfully.
 * retval #kStatus_InvalidArgument Invalid input rxBuff or rxSize argument.
 * retval #kStatus_LPI2C_Busy Another master is currently utilizing the bus.
 * retval #kStatus_LPI2C_Nak The slave device sent a NAK in response to a byte.
 * retval #kStatus_LPI2C_FifoError FIFO under run or overrun.
 * retval #kStatus_LPI2C_ArbitrationLost Arbitration lost error.
 * retval #kStatus_LPI2C_PinLowTimeout SCL or SDA were held low longer than the timeout.
 */
i2c_status LPI2C_MasterReceive(IMX_LPI2C_REGISTERS *base, void *rxBuff, size_t rxSize)
{
    if (NULL == rxBuff) {
        DEBUG((DEBUG_ERROR, "%a: Invalid input rxBuff parameter %u\n", __FUNCTION__));
        return kStatus_InvalidArgument;
    }

    i2c_status result = kStatus_Success;
    UINT8* buf;
    size_t tmpRxSize = rxSize;
#if I2C_RETRY_TIMES != 0U
    UINT32 waitTimes;
#endif

    /* Check transfer data size. */
    if (rxSize > (256U * LPI2C_GetRxFifoSize(base))) {
        return kStatus_InvalidArgument;
    }

    /* Handle empty read. */
    if (rxSize != 0U) {
        /* Wait until there is room in the command fifo. */
        result = LPI2C_MasterWaitForTxReady(base);
        if (kStatus_Success == result) {
            /* Issue command to receive data. A single write to MTDR can issue read operation of 0xFFU + 1 byte of data
               at most, so when the rxSize is larger than 0x100U, push multiple read commands to MTDR until rxSize is
               reached. */
            while (tmpRxSize != 0U) {
                if (tmpRxSize > 256U) {
                    MmioWrite32((UINTN)&base->MTDR, (UINT32)(kRxDataCmd) | (UINT32)IMX_LPI2C_MTDR_DATA(0xFFU));
                    tmpRxSize -= 256U;
                }
                else {
                    MmioWrite32((UINTN)&base->MTDR, (UINT32)(kRxDataCmd) | (UINT32)IMX_LPI2C_MTDR_DATA(tmpRxSize - 1U));
                    tmpRxSize = 0U;
                }
            }

            /* Receive data */
            buf = (UINT8*)rxBuff;
            while (0U != (rxSize--)) {
#if I2C_RETRY_TIMES != 0U
                waitTimes = I2C_RETRY_TIMES;
#endif
                /* Read LPI2C receive fifo register. The register includes a flag to indicate whether */
                /* the FIFO is empty, so we can both get the data and check if we need to keep reading */
                /* using a single register read. */
                UINT32 value = 0U;
                do {
                    /* Check for errors. */
                    result = LPI2C_MasterCheckAndClearError(base, LPI2C_MasterGetStatusFlags(base));
                    if (kStatus_Success != result) {
                        break;
                    }

                    value = MmioRead32((UINTN)&base->MRDR);
#if I2C_RETRY_TIMES != 0U
                    waitTimes--;
                } while ((0U != (value & LPI2C_MRDR_RXEMPTY_MASK)) && (0U != waitTimes));
                if (0U == waitTimes) {
                    result = kStatus_LPI2C_Timeout;
                }
#else
                } while (0U != (value & IMX_LPI2C_MRDR_RXEMPTY_MASK));
#endif
                if ((i2c_status)kStatus_Success != result) {
                    break;
                }

                *buf++ = (UINT8)(value & IMX_LPI2C_MRDR_DATA_MASK);
            }
        }
    }

    return result;
}

/*!
 * brief Performs a polling send transfer on the I2C bus.
 *
 * Sends up to a txSize number of bytes to the previously addressed slave device. The slave may
 * reply with a NAK to any byte in order to terminate the transfer early. If this happens, this
 * function returns #kStatus_LPI2C_Nak.
 *
 * param base  The LPI2C peripheral base address.
 * param txBuff The pointer to the data to be transferred.
 * param txSize The length in bytes of the data to be transferred.
 * retval #kStatus_Success Data was sent successfully.
 * retval #kStatus_InvalidArgument Invalid input txBuff argument.
 * retval #kStatus_LPI2C_Busy Another master is currently utilizing the bus.
 * retval #kStatus_LPI2C_Nak The slave device sent a NAK in response to a byte.
 * retval #kStatus_LPI2C_FifoError FIFO under run or over run.
 * retval #kStatus_LPI2C_ArbitrationLost Arbitration lost error.
 * retval #kStatus_LPI2C_PinLowTimeout SCL or SDA were held low longer than the timeout.
 */
i2c_status LPI2C_MasterSend(IMX_LPI2C_REGISTERS *base, void *txBuff, size_t txSize)
{
    i2c_status result = kStatus_Success;
    UINT8* buf = (UINT8*)txBuff;

    if (NULL == txBuff) {
        DEBUG((DEBUG_ERROR, "%a: Invalid input txBuff parameter %u\n", __FUNCTION__));
        return kStatus_InvalidArgument;
    }

    /* Send data buffer */
    while (0U != (txSize--))
    {
        /* Wait until there is room in the fifo. This also checks for errors. */
        result = LPI2C_MasterWaitForTxReady(base);
        if (kStatus_Success != result)
        {
            break;
        }

        /* Write byte into LPI2C master data register. */
        MmioWrite32((UINTN)&base->MTDR, *buf++);
    }

    return result;
}


/*!
 * brief Sets the I2C bus frequency for master transactions.
 *
 * The LPI2C master is automatically disabled and re-enabled as necessary to configure the baud
 * rate. Do not call this function during a transfer, or the transfer is aborted.
 *
 * note Please note that the second parameter is the clock frequency of LPI2C module, the third
 * parameter means user configured bus baudrate, this implementation is different from other I2C drivers
 * which use baudrate configuration as second parameter and source clock frequency as third parameter.
 *
 * param base The LPI2C peripheral base address.
 * param sourceClock_Hz LPI2C functional clock frequency in Hertz.
 * param baudRate_Hz Requested bus frequency in Hertz.
 */
void LPI2C_MasterSetBaudRate(IMX_LPI2C_REGISTERS *base, UINT32 sourceClock_Hz, UINT32 baudRate_Hz)
{
    BOOLEAN wasEnabled;
    UINT8 filtScl = (UINT8)((MmioRead32((UINTN)&base->MCFGR2) & IMX_LPI2C_MCFGR2_FILTSCL_MASK) >> IMX_LPI2C_MCFGR2_FILTSCL_SHIFT);

    UINT8 divider = 1U;
    UINT8 bestDivider = 1U;
    UINT8 prescale = 0U;
    UINT8 bestPre = 0U;

    UINT8 clkCycle;
    UINT8 bestclkCycle = 0U;

    UINT32 absError = 0U;
    UINT32 bestError = 0xffffffffu;
    UINT32 computedRate;

    UINT32 tmpReg = 0U;

    /* Disable master mode. */
    wasEnabled = (0U != ((MmioRead32((UINTN)&base->MCR) & IMX_LPI2C_MCR_MEN_MASK) >> IMX_LPI2C_MCR_MEN_SHIFT));
    LPI2C_MasterEnable(base, FALSE);

    /* Baud rate = (sourceClock_Hz / 2 ^ prescale) / (CLKLO + 1 + CLKHI + 1 + SCL_LATENCY)
     * SCL_LATENCY = ROUNDDOWN((2 + FILTSCL) / (2 ^ prescale))
     */
    for (prescale = 0U; prescale <= 7U; prescale++) {
        /* Calculate the clkCycle, clkCycle = CLKLO + CLKHI, divider = 2 ^ prescale */
        clkCycle = (UINT8)((10U * sourceClock_Hz / divider / baudRate_Hz + 5U) / 10U - (2U + filtScl) / divider - 2U);
        /* According to register description, The max value for CLKLO and CLKHI is 63.
           however to meet the I2C specification of tBUF, CLKHI should be less than
           clkCycle - 0.52 x sourceClock_Hz / baudRate_Hz / divider + 1U. Refer to the comment of the tmpHigh's
           calculation for details. So we have:
           CLKHI < clkCycle - 0.52 x sourceClock_Hz / baudRate_Hz / divider + 1U,
           clkCycle = CLKHI + CLKLO and
           sourceClock_Hz / baudRate_Hz / divider = clkCycle + 2 + ROUNDDOWN((2 + FILTSCL) / divider),
           we can come up with: CLKHI < 0.92 x CLKLO - ROUNDDOWN(2 + FILTSCL) / divider
           so the max boundary of CLKHI should be 0.92 x 63 - ROUNDDOWN(2 + FILTSCL) / divider,
           and the max boundary of clkCycle is 1.92 x 63 - ROUNDDOWN(2 + FILTSCL) / divider. */
        if (clkCycle > (120U - (2U + filtScl) / divider)) {
            divider *= 2U;
            continue;
        }
        /* Calculate the computed baudrate and compare it with the desired baudrate */
        computedRate = (sourceClock_Hz / (UINT32)divider) /
            ((UINT32)clkCycle + 2U + (2U + (UINT32)filtScl) / (UINT32)divider);
        absError = baudRate_Hz > computedRate ? baudRate_Hz - computedRate : computedRate - baudRate_Hz;
        if (absError < bestError) {
            bestPre = prescale;
            bestDivider = divider;
            bestclkCycle = clkCycle;
            bestError = absError;

            /* If the error is 0, then we can stop searching because we won't find a better match. */
            if (absError == 0U) {
                break;
            }
        }
        divider *= 2U;
    }

    /* SCL low time tLO should be larger than or equal to SCL high time tHI:
       tLO = ((CLKLO + 1) x (2 ^ PRESCALE)) >= tHI = ((CLKHI + 1 + SCL_LATENCY) x (2 ^ PRESCALE)),
       which is CLKLO >= CLKHI + (2U + filtScl) / bestDivider.
       Also since bestclkCycle = CLKLO + CLKHI, bestDivider = 2 ^ PRESCALE
       which makes CLKHI <= (bestclkCycle - (2U + filtScl) / bestDivider) / 2U.

       The max tBUF should be at least 0.52 times of the SCL clock cycle:
       tBUF = ((CLKLO + 1) x (2 ^ PRESCALE) / sourceClock_Hz) > (0.52 / baudRate_Hz),
       plus bestDivider = 2 ^ PRESCALE, bestclkCycle = CLKLO + CLKHI we can come up with
       CLKHI <= (bestclkCycle - 0.52 x sourceClock_Hz / baudRate_Hz / bestDivider + 1U).
       In this case to get a safe CLKHI calculation, we can assume:
    */
    UINT8 tmpHigh = (bestclkCycle - (2U + filtScl) / bestDivider) / 2U;
    while (tmpHigh > (bestclkCycle - 52U * sourceClock_Hz / baudRate_Hz / bestDivider / 100U + 1U)) {
        tmpHigh = tmpHigh - 1U;
    }

    /* Calculate DATAVD and SETHOLD.
       To meet the timing requirement of I2C spec for standard mode, fast mode and fast mode plus: */
       /* The min tHD:STA/tSU:STA/tSU:STO should be at least 0.4 times of the SCL clock cycle, use 0.5 to be safe:
          tHD:STA = ((SETHOLD + 1) x (2 ^ PRESCALE) / sourceClock_Hz) > (0.5 / baudRate_Hz), bestDivider = 2 ^ PRESCALE */
    UINT8 tmpHold = (UINT8)(sourceClock_Hz / baudRate_Hz / bestDivider / 2U) - 1U;

    /* The max tVD:DAT/tVD:ACK/tHD:DAT should be at most 0.345 times of the SCL clock cycle, use 0.25 to be safe:
       tVD:DAT = ((DATAVD + 1) x (2 ^ PRESCALE) / sourceClock_Hz) < (0.25 / baudRate_Hz), bestDivider = 2 ^ PRESCALE */
    UINT8 tmpDataVd = (UINT8)(sourceClock_Hz / baudRate_Hz / bestDivider / 4U) - 1U;

    /* The min tSU:DAT should be at least 0.05 times of the SCL clock cycle:
       tSU:DAT = ((2 + FILTSDA + 2 ^ PRESCALE) / sourceClock_Hz) >= (0.05 / baud),
       plus bestDivider = 2 ^ PRESCALE, we can come up with:
       FILTSDA >= (0.05 x sourceClock_Hz / baudRate_Hz - bestDivider - 2) */
    if ((sourceClock_Hz / baudRate_Hz / 20U) > (bestDivider + 2U)) {
        /* Read out the FILTSDA configuration, if it is smaller than expected, change the setting. */
        UINT8 filtSda = (UINT8)((MmioRead32((UINTN)&base->MCFGR2) & IMX_LPI2C_MCFGR2_FILTSDA_MASK) >> 
            IMX_LPI2C_MCFGR2_FILTSDA_SHIFT);
        if (filtSda < (sourceClock_Hz / baudRate_Hz / 20U - bestDivider - 2U)) {
            filtSda = (UINT8)(sourceClock_Hz / baudRate_Hz / 20U) - bestDivider - 2U;
        }
        MmioWrite32((UINTN)&base->MCFGR2, (MmioRead32((UINTN)&base->MCFGR2) & 
            ~IMX_LPI2C_MCFGR2_FILTSDA_MASK) | IMX_LPI2C_MCFGR2_FILTSDA(filtSda));
    }

    /* Set CLKHI, CLKLO, SETHOLD, DATAVD value. */
    tmpReg = IMX_LPI2C_MCCR0_CLKHI((UINT32)tmpHigh) |
        IMX_LPI2C_MCCR0_CLKLO((UINT32)((UINT32)bestclkCycle - (UINT32)tmpHigh)) |
        IMX_LPI2C_MCCR0_SETHOLD((UINT32)tmpHold) | IMX_LPI2C_MCCR0_DATAVD((UINT32)tmpDataVd);
    base->MCCR0 = tmpReg;

    /* Set PRESCALE value. */
    MmioWrite32((UINTN)&base->MCFGR1, (MmioRead32((UINTN)&base->MCFGR1) & 
        ~IMX_LPI2C_MCFGR1_PRESCALE_MASK) | IMX_LPI2C_MCFGR1_PRESCALE(bestPre));

    /* Restore master mode. */
    if (wasEnabled) {
        LPI2C_MasterEnable(base, TRUE);
    }
}

/*!
 * brief Performs a master polling transfer on the I2C bus.
 *
 * note The API does not return until the transfer succeeds or fails due
 * to error happens during transfer.
 *
 * param base The LPI2C peripheral base address.
 * param transfer Pointer to the transfer structure.
 * retval #kStatus_Success Data was received successfully.
 * retval #kStatus_LPI2C_Busy Another master is currently utilizing the bus.
 * retval #kStatus_LPI2C_Nak The slave device sent a NAK in response to a byte.
 * retval #kStatus_LPI2C_FifoError FIFO under run or overrun.
 * retval #kStatus_LPI2C_ArbitrationLost Arbitration lost error.
 * retval #kStatus_LPI2C_PinLowTimeout SCL or SDA were held low longer than the timeout.
 */
i2c_status LPI2C_MasterTransferBlocking(IMX_LPI2C_REGISTERS *base, lpi2c_master_transfer_t *transfer)
{
    i2c_status result = kStatus_Success;
    UINT16 commandBuffer[7];
    UINT32 cmdCount = 0U;

    /* Check arguments */
    if ((NULL == transfer) || (transfer->subaddressSize > sizeof(transfer->subaddress))) {
        return kStatus_InvalidArgument;
    }

    /* Check transfer data size in read operation. */
    if ((transfer->direction == kLPI2C_Read) && (transfer->dataSize > (256U * LPI2C_GetRxFifoSize(base)))) {
        return kStatus_InvalidArgument;
    }

    /* Return an error if the bus is already in use not by us. */
    result = LPI2C_CheckForBusyBus(base);
    if (kStatus_Success == result) {
        /* Clear all flags. */
        LPI2C_MasterClearStatusFlags(base, (UINT32)kLPI2C_MasterClearFlags);

        /* Turn off auto-stop option. */
        MmioWrite32((UINTN)&base->MCFGR1, MmioRead32((UINTN)&base->MCFGR1) & ~IMX_LPI2C_MCFGR1_AUTOSTOP_MASK);

        lpi2c_direction_t direction = (0U != transfer->subaddressSize) ? kLPI2C_Write : transfer->direction;
        if (0U == (transfer->flags & (UINT32)kLPI2C_TransferNoStartFlag)) {
            commandBuffer[cmdCount++] =
                (UINT16)kStartCmd |
                (UINT16)((UINT16)((UINT16)transfer->slaveAddress << 1U) | (UINT16)direction);
        }

        /* Subaddress, MSB first. */
        if (0U != transfer->subaddressSize) {
            UINT32 subaddressRemaining = (UINT32)transfer->subaddressSize;
            while (0U != subaddressRemaining--) {
                UINT8 subaddressByte = (UINT8)((transfer->subaddress >> (8U * subaddressRemaining)) & 0xffU);
                commandBuffer[cmdCount++] = subaddressByte;
            }
        }

        /* Reads need special handling. */
        if ((0U != transfer->dataSize) && (transfer->direction == kLPI2C_Read)) {
            /* Need to send repeated start if switching directions to read. */
            if (direction == kLPI2C_Write) {
                commandBuffer[cmdCount++] =
                    (UINT16)kStartCmd |
                    (UINT16)((UINT16)((UINT16)transfer->slaveAddress << 1U) | (UINT16)kLPI2C_Read);
            }
        }

        /* Send command buffer */
        UINT32 index = 0U;
        while (0U != cmdCount--) {
            /* Wait until there is room in the fifo. This also checks for errors. */
            result = LPI2C_MasterWaitForTxReady(base);
            if (kStatus_Success != result) {
                break;
            }

            /* Write byte into LPI2C master data register. */
            MmioWrite32((UINTN)&base->MTDR, commandBuffer[index]);
            index++;
        }

        if (kStatus_Success == result) {
            /* Transmit data. */
            if ((transfer->direction == kLPI2C_Write) && (transfer->dataSize > 0U)) {
                /* Send Data. */
                result = LPI2C_MasterSend(base, transfer->data, transfer->dataSize);
            }

            /* Receive Data. */
            if ((transfer->direction == kLPI2C_Read) && (transfer->dataSize > 0U)) {
                result = LPI2C_MasterReceive(base, transfer->data, transfer->dataSize);
            }

            if (kStatus_Success == result) {
                if ((transfer->flags & (UINT32)kLPI2C_TransferNoStopFlag) == 0U) {
                    result = LPI2C_MasterStop(base);
                }
            }
        }
    }

    return result;
}

static void iMXI2cInit(IMX_LPI2C_REGISTERS *base, IMX_I2C_CONTEXT *i2cContext)
{
    lpi2c_master_config_t lpi2cConfig = { 0 };

    /* Default:
     * lpi2cConfig.baudRate_Hz = 100000U;
     * lpi2cConfig.busIdleTimeout_ns = 0;
     * lpi2cConfig.pinLowTimeout_ns = 0;
     * lpi2cConfig.sdaGlitchFilterWidth_ns = 0;
     * lpi2cConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&lpi2cConfig);
    lpi2cConfig.baudRate_Hz = i2cContext->TargetFrequency;
    LPI2C_MasterInit(base, &lpi2cConfig, i2cContext->ReferenceFrequency);

}

/**
  Gets whether LPI2C master has been initialized.

  @param[in]    instance          LPI2C instance number

  @retval   TRUE                  LPI2C master has been initialized.
  @retval   FALSE                 LPI2C master has no been initialized yet.

**/
static BOOLEAN LPI2C_IsMasterInitialized(IMX_LPI2C_REGISTERS *base, INT8 instance)
{
    UINT32 MCRBit;

    MCRBit = ((MmioRead32((UINTN)&base->MCR) & IMX_LPI2C_MCR_MEN_MASK) >> IMX_LPI2C_MCR_MEN_SHIFT);

    return ((0U != MCRBit) && (DeviceCtxPtrs[instance].IsInitialized));
}

/**
  Sets that LPI2C master has already been initialized.

  @param[in]    instance          LPI2C instance number

**/
static void LPI2C_SetMasterInitializedFlag(INT8 instance)
{
    DeviceCtxPtrs[instance].IsInitialized = TRUE;
}

/**
  Perform I2C read operation.

  The iMXI2cRead perform I2C read operation by programming the I2C controller.
  The caller is responsible to provide I2C controller configuration.

  @param[in]    I2cContext        Pointer to structure containing the targeted
                                  I2C controller to be used for I2C operation.
  @param[in]    RegisterAddress   Targeted device 1-byte-register address to start read.
  @param[out]   ReadBufferPtr     Caller supplied buffer that would be written
                                  into with data from the read operation.
  @param[in]    ReadBufferSize    Size of caller supplied buffer.

  @retval   RETURN_SUCCESS        I2C Read operation succeeded.
  @retval   RETURN_DEVICE_ERROR   The I2C device is not functioning correctly.

**/
RETURN_STATUS
iMXI2cRead(
    IN IMX_I2C_CONTEXT *I2cContext,
    IN UINT8 RegisterAddress,
    OUT UINT8 *ReadBufferPtr,
    IN UINT32 ReadBufferSize
)
{
    return iMXI2cReadE(I2cContext, RegisterAddress, 1, ReadBufferPtr, ReadBufferSize);
}

/**
  Perform I2C write operation.

  The iMXI2cWrite perform I2C write operation by programming the I2C
  controller. The caller is responsible to provide I2C controller
  configuration.

  @param[in]    I2cContext        Pointer to structure containing the targeted
                                  I2C controller to be used for I2C operation.
  @param[in]    RegisterAddress   Targeted device 1-byte-register address to start write.
  @param[out]   WriteBufferPtr    Caller supplied buffer that contained data that
                                  would be read from for I2C write operation.
  @param[in]    WriteBufferSize   Size of caller supplied buffer.

  @retval   RETURN_SUCCESS        I2C Write operation succeeded.
  @retval   RETURN_DEVICE_ERROR   The I2C device is not functioning correctly.

**/
RETURN_STATUS
iMXI2cWrite(
    IN IMX_I2C_CONTEXT *I2cContext,
    IN UINT8 RegisterAddress,
    IN UINT8 *WriteBufferPtr,
    IN UINT32 WriteBufferSize
)
{
    return iMXI2cWriteE(I2cContext, RegisterAddress, 1, WriteBufferPtr, WriteBufferSize);
}

/**
  Perform I2C read operation.

  The iMXI2cRead perform I2C read operation by programming the I2C controller.
  The caller is responsible to provide I2C controller configuration.

  @param[in]    I2cContext        Pointer to structure containing the targeted
                                  I2C controller to be used for I2C operation.
  @param[in]    RegisterAddress   Targeted device N-byte-register address to start read.
  @param[in]    RegisterAddressSize  Length of sub address to send in bytes. Maximum size is 4 bytes.
  @param[out]   ReadBufferPtr     Caller supplied buffer that would be written
                                  into with data from the read operation.
  @param[in]    ReadBufferSize    Size of caller supplied buffer.

  @retval   RETURN_SUCCESS        I2C Read operation succeeded.
  @retval   RETURN_DEVICE_ERROR   The I2C device is not functioning correctly.

**/
RETURN_STATUS
iMXI2cReadE(
    IN IMX_I2C_CONTEXT *I2cContext,
    IN UINT32 RegisterAddress,
    IN UINT8 RegisterAddressSize,
    OUT UINT8* ReadBufferPtr,
    IN UINT32 ReadBufferSize
)
{
    lpi2c_master_transfer_t transfer = { 0 };
    i2c_status i2cStatus;
    IMX_LPI2C_REGISTERS *baseAddress;
    INT8 instance;

    baseAddress = (IMX_LPI2C_REGISTERS*)I2cContext->ControllerAddress;

    instance = LPI2C_GetInstance(baseAddress);
    if (instance == -1) {
        DEBUG((DEBUG_ERROR, "%a: Invalid base address 0x%lx\n", __FUNCTION__, baseAddress));
        return RETURN_INVALID_PARAMETER;
    }

    if (!LPI2C_IsMasterInitialized(baseAddress, instance)) {
        iMXI2cInit(baseAddress, I2cContext);
        LPI2C_SetMasterInitializedFlag(instance);
    }

    transfer.flags = kLPI2C_TransferDefaultFlag;
    transfer.slaveAddress = (UINT16)I2cContext->SlaveAddress;
    transfer.direction = kLPI2C_Read;
    transfer.subaddress = RegisterAddress;
    transfer.subaddressSize = RegisterAddressSize;
    transfer.data = (void*)ReadBufferPtr;
    transfer.dataSize = (size_t)ReadBufferSize;
    i2cStatus = LPI2C_MasterTransferBlocking(baseAddress, &transfer);

    return (i2cStatus == kStatus_Success) ? RETURN_SUCCESS : RETURN_DEVICE_ERROR;
}

/**
  Perform I2C write operation.

  The iMXI2cWrite perform I2C write operation by programming the I2C
  controller. The caller is responsible to provide I2C controller
  configuration.

  @param[in]    I2cContext        Pointer to structure containing the targeted
                                  I2C controller to be used for I2C operation.
  @param[in]    RegisterAddress   Targeted device N-byte-register address to start write.
  @param[in]    RegisterAddressSize  Length of sub address to send in bytes. Maximum size is 4 bytes.
  @param[out]   WriteBufferPtr    Caller supplied buffer that contained data that
                                  would be read from for I2C write operation.
  @param[in]    WriteBufferSize   Size of caller supplied buffer.

  @retval   RETURN_SUCCESS        I2C Write operation succeeded.
  @retval   RETURN_DEVICE_ERROR   The I2C device is not functioning correctly.

**/
RETURN_STATUS
iMXI2cWriteE(
    IN IMX_I2C_CONTEXT *I2cContext,
    IN UINT32 RegisterAddress,
    IN UINT8 RegisterAddressSize,
    IN UINT8* WriteBufferPtr,
    IN UINT32 WriteBufferSize
)
{
    lpi2c_master_transfer_t transfer = { 0 };
    i2c_status i2cStatus;
    IMX_LPI2C_REGISTERS *baseAddress;
    INT8 instance;

    baseAddress = (IMX_LPI2C_REGISTERS *)I2cContext->ControllerAddress;

    instance = LPI2C_GetInstance(baseAddress);
    if (instance == -1) {
        DEBUG((DEBUG_ERROR, "%a: Invalid base address 0x%lx\n", __FUNCTION__, baseAddress));
        return RETURN_INVALID_PARAMETER;
    }

    if (!LPI2C_IsMasterInitialized(baseAddress, instance)) {
        iMXI2cInit(baseAddress, I2cContext);
        LPI2C_SetMasterInitializedFlag(instance);
    }

    transfer.flags = kLPI2C_TransferDefaultFlag;
    transfer.slaveAddress = (UINT16)I2cContext->SlaveAddress;
    transfer.direction = kLPI2C_Write;
    transfer.subaddress = RegisterAddress;
    transfer.subaddressSize = RegisterAddressSize;
    transfer.data = (void*)WriteBufferPtr;
    transfer.dataSize = (size_t)WriteBufferSize;
    i2cStatus = LPI2C_MasterTransferBlocking(baseAddress, &transfer);

    return (i2cStatus == kStatus_Success) ? RETURN_SUCCESS : RETURN_DEVICE_ERROR;
}
