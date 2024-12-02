/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/SerialPortLib.h>
#include <Library/IoLib.h>
#include "fsl_mu.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.mu"
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to mu clocks for each instance. */
static const clock_ip_name_t s_muClocks[] = MU_CLOCKS;
static MU_Type *const s_muBases[] = MU_BASE_PTRS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
/*! @brief Pointers to mu bases for each instance. */

/******************************************************************************
 * Code
 *****************************************************************************/
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
static uint32_t MU_GetInstance(MU_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0U; instance < (sizeof(s_muBases) / sizeof(s_muBases[0])); instance++)
    {
        if (s_muBases[instance] == base)
        {
            break;
        }
    }

    ASSERT(instance < (sizeof(s_muBases) / sizeof(s_muBases[0])));

    return instance;
}
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*!
 * brief Initializes the MU module.
 *
 * This function enables the MU clock only.
 *
 * param base MU peripheral base address.
 */
void MU_Init(MU_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_EnableClock(s_muClocks[MU_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

/*!
 * brief De-initializes the MU module.
 *
 * This function disables the MU clock only.
 *
 * param base MU peripheral base address.
 */
void MU_Deinit(MU_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_DisableClock(s_muClocks[MU_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

/*!
 * brief Blocks to send a message.
 *
 * This function waits until the TX register is empty and sends the message.
 *
 * param base MU peripheral base address.
 * param regIndex  TX register index.
 * param msg      Message to send.
 * return Error state.
 * retval 0    Message have been sent successfully.
 * retval -1       Message have not been sent.
 */
int MU_SendMsg(MU_Type *base, uint32_t regIndex, uint32_t msg)
{
    int result = 0;
    if ((base == NULL) || (regIndex >= MU_TR_COUNT)){
        result = -1;
        ASSERT(0);
    }
    
    if (result == 0) {
        result = -1;
        /* Wait TX register to be empty. */
        for (int cnt = 0; cnt < 200; cnt++)
        {
            if (cnt != 0) {
                MicroSecondDelay (50);
            }
            if (0U != (MmioRead32((UINTN)&base->SR) & (((uint32_t)kMU_Tx0EmptyFlag) >> regIndex))) {
                MmioWrite32((UINTN)&base->TR[regIndex], msg);
                result = 0;
                break;
            }
        }
    }
    return result;
}

/*!
 * brief Blocks to receive a message.
 *
 * This function waits until the RX register is full and receives the message.
 *
 * param base MU peripheral base address.
 * param regIndex  RX register index.
 * param message  pointer to the message buffer.
 * return Error state.
 * retval 0    Message have been received successfully.
 * retval -1       Message have not been received.
 */
int MU_ReceiveMsg(MU_Type *base, uint32_t regIndex, uint32_t *message)
{
    int result = 0;
    if ((base == NULL) || (regIndex >= MU_TR_COUNT) || (message == NULL)) {
        result = -1;
        ASSERT(0);
    }
    
    if (result == 0) {
        result = -1;
        /* Wait RX register to be full. */
        for (int cnt = 0; cnt < 200; cnt++)
        {
            if (cnt != 0) {
                MicroSecondDelay (50);
            }
            if (0U != (MmioRead32((UINTN)&base->SR) & (((uint32_t)kMU_Rx0FullFlag) >> regIndex))) {
                *message = MmioRead32((UINTN)&base->RR[regIndex]);
                result = 0;
                break;
            }
        }
    }
    return result;
}

/*!
 * brief Blocks setting the 3-bit MU flags reflect on the other MU side.
 *
 * This function blocks setting the 3-bit MU flags. Every time the 3-bit MU flags are changed,
 * the status flag \c kMU_FlagsUpdatingFlag ASSERTs indicating the 3-bit MU flags are
 * updating to the other side. After the 3-bit MU flags are updated, the status flag
 * \c kMU_FlagsUpdatingFlag is cleared by hardware. During the flags updating period,
 * the flags cannot be changed. This function waits for the MU status flag
 * \c kMU_FlagsUpdatingFlag cleared and sets the 3-bit MU flags.
 *
 * param base MU peripheral base address.
 * param flags The 3-bit MU flags to set.
 */
void MU_SetFlags(MU_Type *base, uint32_t flags)
{
    /* Wait for update finished. */
    while (0U != (MmioRead32((UINTN)&base->SR) & ((uint32_t)MU_SR_FUP_MASK)))
    {
        ; /* Intentional empty while*/
    }

    MU_SetFlagsNonBlocking(base, flags);
}

/*!
 * brief Triggers interrupts to the other core.
 *
 * This function triggers the specific interrupts to the other core. The interrupts
 * to trigger are passed in as bit mask. See \ref _mu_interrupt_trigger.
 * The MU should not trigger an interrupt to the other core when the previous interrupt
 * has not been processed by the other core. This function checks whether the
 * previous interrupts have been processed. If not, it returns an error.
 *
 * code
 * if (kStatus_Success != MU_TriggerInterrupts(base, kMU_GenInt0InterruptTrigger | kMU_GenInt2InterruptTrigger))
 * {
 *     Previous general purpose interrupt 0 or general purpose interrupt 2
 *     has not been processed by the other core.
 * }
 * endcode
 *
 * param base MU peripheral base address.
 * param mask Bit mask of the interrupts to trigger. See _mu_interrupt_trigger.
 * retval 0    Interrupts have been triggered successfully.
 * retval -1       Previous interrupts have not been accepted.
 */
int MU_TriggerInterrupts(MU_Type *base, uint32_t mask)
{
    int status = 0;
    uint32_t reg    = MmioRead32((UINTN)&base->CR);

    /* Previous interrupt has been accepted. */
    if (0U == (reg & mask))
    {
        /* All interrupts have been accepted, trigger now. */
        reg      = (reg & ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | mask;
        MmioWrite32((UINTN)&base->CR, reg);
        status   = 0;
    }
    else
    {
        status = -1;
    }

    return status;
}

#if !(defined(FSL_FEATURE_MU_NO_RSTH) && FSL_FEATURE_MU_NO_RSTH)
/*!
 * brief Boots the core at B side.
 *
 * This function sets the B side core's boot configuration and releases the
 * core from reset.
 *
 * param base MU peripheral base address.
 * param mode Core B boot mode.
 * note Only MU side A can use this function.
 */
void MU_BootCoreB(MU_Type *base, mu_core_boot_mode_t mode)
{
#if (defined(FSL_FEATURE_MU_HAS_RESET_DEASSERT_INT) && FSL_FEATURE_MU_HAS_RESET_ASSERT_INT)
    /* Clean the reset de-ASSERT pending flag. */
    MmioWrite32((UINTN)&base->SR, MU_SR_RDIP_MASK);
#endif

#if (defined(FSL_FEATURE_MU_HAS_CCR) && FSL_FEATURE_MU_HAS_CCR)
    uint32_t reg = MmioRead32((UINTN)&base->CCR);

    reg = (reg & ~(MU_CCR_HR_MASK | MU_CCR_RSTH_MASK | MU_CCR_BOOT_MASK)) | MU_CCR_BOOT(mode);

    MmioWrite32((UINTN)&base->CCR, reg);
#else
    uint32_t reg = MmioRead32((UINTN)&base->CR);

    reg = (reg & ~((MU_CR_GIRn_MASK | MU_CR_NMI_MASK) | MU_CR_HR_MASK | MU_CR_RSTH_MASK | MU_CR_BBOOT_MASK)) |
          MU_CR_BBOOT(mode);

    MmioWrite32((UINTN)&base->CR, reg);
#endif
}

/*!
 * brief Boots the other core.
 *
 * This function boots the other core with a boot configuration.
 *
 * param base MU peripheral base address.
 * param mode The other core boot mode.
 */
void MU_BootOtherCore(MU_Type *base, mu_core_boot_mode_t mode)
{
    /*
     * MU_BootOtherCore and MU_BootCoreB are the same, MU_BootCoreB is kept
     * for compatible with older platforms.
     */
    MU_BootCoreB(base, mode);
}
#endif /* FSL_FEATURE_MU_NO_RSTH */

#if !(defined(FSL_FEATURE_MU_NO_HR) && FSL_FEATURE_MU_NO_HR)
#if (defined(FSL_FEATURE_MU_HAS_CCR) && FSL_FEATURE_MU_HAS_CCR)
/*!
 * brief Hardware reset the other core.
 *
 * This function resets the other core, the other core could mask the
 * hardware reset by calling ref MU_MaskHardwareReset. The hardware reset
 * mask feature is only available for some platforms.
 * This function could be used together with MU_BootOtherCore to control the
 * other core reset workflow.
 *
 * Example 1: Reset the other core, and no hold reset
 * code
 * MU_HardwareResetOtherCore(MU_A, true, false, bootMode);
 * endcode
 * In this example, the core at MU side B will reset with the specified boot mode.
 *
 * Example 2: Reset the other core and hold it, then boot the other core later.
 * code
 *    Here the other core enters reset, and the reset is hold
 * MU_HardwareResetOtherCore(MU_A, true, true, modeDontCare);
 *    Current core boot the other core when necessary.
 * MU_BootOtherCore(MU_A, bootMode);
 * endcode
 *
 * param base MU peripheral base address.
 * param waitReset Wait the other core enters reset.
 *                    - true: Wait until the other core enters reset, if the other
 *                      core has masked the hardware reset, then this function will
 *                      be blocked.
 *                    - false: Don't wait the reset.
 * param holdReset Hold the other core reset or not.
 *                    - true: Hold the other core in reset, this function returns
 *                      directly when the other core enters reset.
 *                    - false: Don't hold the other core in reset, this function
 *                      waits until the other core out of reset.
 * param bootMode Boot mode of the other core, if p holdReset is true, this
 *                 parameter is useless.
 */
void MU_HardwareResetOtherCore(MU_Type *base, bool waitReset, bool holdReset, mu_core_boot_mode_t bootMode)
{
#if (defined(FSL_FEATURE_MU_NO_RSTH) && FSL_FEATURE_MU_NO_RSTH)
    /* If MU does not support hold reset, then the parameter must be false. */
    ASSERT(false == holdReset);
#endif
    uint32_t ccr = MmioRead32((UINTN)&base->CCR) & ~(MU_CCR_HR_MASK | MU_CCR_RSTH_MASK | MU_CCR_BOOT_MASK);

    ccr |= MU_CCR_BOOT(bootMode);

    if (holdReset)
    {
        ccr |= MU_CCR_RSTH_MASK;
    }

    /* Clean the reset ASSERT pending flag. */
    MmioWrite32((UINTN)&base->SR, (MU_SR_RAIP_MASK | MU_SR_RDIP_MASK));

    /* Set CCR[HR] to trigger hardware reset. */
    MmioWrite32((UINTN)&base->CCR, ccr | MU_CCR_HR_MASK);

    /* If wait the other core enters reset. */
    if (waitReset)
    {
        /* Wait for the other core go to reset. */
        while (0U == (MmioRead32((UINTN)&base->SR) & MU_SR_RAIP_MASK))
        {
            ; /* Intentional empty while*/
        }

        if (!holdReset)
        {
            /* Clear CCR[HR]. */
            MmioWrite32((UINTN)&base->CCR, ccr);

            /* Wait for the other core out of reset. */
            while (0U == (MmioRead32((UINTN)&base->SR) & MU_SR_RDIP_MASK))
            {
                ; /* Intentional empty while*/
            }
        }
    }
}
#else /* FSL_FEATURE_MU_HAS_CCR */
/*!
 * brief Hardware reset the other core.
 *
 * This function resets the other core, the other core could mask the
 * hardware reset by calling ref MU_MaskHardwareReset. The hardware reset
 * mask feature is only available for some platforms.
 * This function could be used together with MU_BootOtherCore to control the
 * other core reset workflow.
 *
 * Example 1: Reset the other core, and no hold reset
 * code
 * MU_HardwareResetOtherCore(MU_A, true, false, bootMode);
 * endcode
 * In this example, the core at MU side B will reset with the specified boot mode.
 *
 * Example 2: Reset the other core and hold it, then boot the other core later.
 * code
 *    Here the other core enters reset, and the reset is hold
 * MU_HardwareResetOtherCore(MU_A, true, true, modeDontCare);
 *    Current core boot the other core when necessary.
 * MU_BootOtherCore(MU_A, bootMode);
 * endcode
 *
 * param base MU peripheral base address.
 * param waitReset Wait the other core enters reset.
 *                    - true: Wait until the other core enters reset, if the other
 *                      core has masked the hardware reset, then this function will
 *                      be blocked.
 *                    - false: Don't wait the reset.
 * param holdReset Hold the other core reset or not.
 *                    - true: Hold the other core in reset, this function returns
 *                      directly when the other core enters reset.
 *                    - false: Don't hold the other core in reset, this function
 *                      waits until the other core out of reset.
 * param bootMode Boot mode of the other core, if p holdReset is true, this
 *                 parameter is useless.
 */
void MU_HardwareResetOtherCore(MU_Type *base, bool waitReset, bool holdReset, mu_core_boot_mode_t bootMode)
{
#if (defined(FSL_FEATURE_MU_NO_RSTH) && FSL_FEATURE_MU_NO_RSTH)
    /* If MU does not support hold reset, then the parameter must be false. */
    ASSERT(false == holdReset);
#endif
    uint32_t resetFlag = 0;

    uint32_t cr = MmioRead32((UINTN)&base->CR) & ~(MU_CR_HR_MASK | MU_CR_RSTH_MASK | MU_CR_BOOT_MASK | MU_CR_GIRn_MASK | MU_CR_NMI_MASK);

    cr |= MU_CR_BOOT(bootMode);

    if (holdReset)
    {
        cr |= MU_CR_RSTH_MASK;
    }

#if (defined(FSL_FEATURE_MU_HAS_RESET_ASSERT_INT) && FSL_FEATURE_MU_HAS_RESET_ASSERT_INT)
    resetFlag |= MU_SR_RAIP_MASK;
#endif
#if (defined(FSL_FEATURE_MU_HAS_RESET_DEASSERT_INT) && FSL_FEATURE_MU_HAS_RESET_ASSERT_INT)
    resetFlag |= MU_SR_RDIP_MASK;
#endif
    /* Clean the reset ASSERT pending flag. */
    MmioWrite32((UINTN)&base->SR, resetFlag);

    /* Set CR[HR] to trigger hardware reset. */
    MmioWrite32((UINTN)&base->CR, cr | MU_CR_HR_MASK);

    /* If wait the other core enters reset. */
    if (waitReset)
    {
#if (defined(FSL_FEATURE_MU_HAS_RESET_ASSERT_INT) && FSL_FEATURE_MU_HAS_RESET_ASSERT_INT)
        /* Wait for the other core go to reset. */
        while (0U == (MmioRead32((UINTN)&base->SR) & MU_SR_RAIP_MASK))
        {
            ; /* Intentional empty while*/
        }
#endif

        if (!holdReset)
        {
            /* Clear CR[HR]. */
            MmioWrite32((UINTN)&base->CR, cr);

#if (defined(FSL_FEATURE_MU_HAS_RESET_DEASSERT_INT) && FSL_FEATURE_MU_HAS_RESET_ASSERT_INT)
            /* Wait for the other core out of reset. */
            while (0U == (MmioRead32((UINTN)&base->SR) & MU_SR_RDIP_MASK))
            {
                ; /* Intentional empty while*/
            }
#endif
        }
    }
}
#endif /* FSL_FEATURE_MU_HAS_CCR  */
#endif /* FSL_FEATURE_MU_NO_HR */
