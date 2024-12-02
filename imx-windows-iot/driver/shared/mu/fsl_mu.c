/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <parent_device.h>
#include "fsl_mu.h"

/******************************************************************************
 * Code
 *****************************************************************************/

/**
 * MU Event service routine
 * @param Cookie address of mu_device_data_t structure
 */
BOOLEAN MU_InterruptHandler(WDFINTERRUPT WdfInterrupt, ULONG MessageID)
{
    mu_interrupt_data_t interrupt_data;
    mu_device_data_t device_data;
    mu_isr_dpc_calls_t calls;
    MU_Type *base;
    UINT32 schedule_dpc = 0U;
    UINT32 rc_dis = (UINT32)(~(UINT32)0U);
    UINT32 pending_flags;

    interrupt_data = MU_GET_INTERRUPT_CONTEXT(WdfInterrupt);
    ASSERT(interrupt_data != NULL);
    device_data = MU_GET_DEVICE_CONTEXT(WdfInterruptGetDevice(WdfInterrupt));
    ASSERT(device_data != NULL);
    calls = &interrupt_data->isr_dpc_calls;
    base = device_data->base;
    ASSERT(base != NULL);
    pending_flags = base->SR & (base->CR & (MU_CR_GIEn_MASK | MU_CR_TIEn_MASK | MU_CR_RIEn_MASK));
    if (pending_flags == 0U) {
        return FALSE;
    }
    for (int regIndex = 0; regIndex < MU_TR_COUNT; regIndex++) {
        if (pending_flags & (UINT32)(((UINT32)kMU_Rx0FullFlag) >> regIndex)) {
            mu_isr_call_t call = calls->rx_full_isr[regIndex];
            BOOLEAN done = TRUE;
            if (call == NULL) {
                rc_dis &= (UINT32)(~((UINT32)(((UINT32)kMU_Rx0FullInterruptEnable) >> regIndex)));
            }
            else {
                done = call(WdfInterrupt, MessageID,
#ifdef MU_ISR_USER_CONTAINER_T
                     &interrupt_data->isr_user_container,
#else
                    NULL,
#endif
#ifdef MU_DEVICE_USER_CONTAINER_T
                     &device_data->device_user_container
#else
                    NULL
#endif
                    );
            }
            if (done && (calls->rx_full_dpc[regIndex] != NULL)) {
                schedule_dpc |= (UINT32)(((UINT32)kMU_Rx0FullFlag) >> regIndex);
            }
        }
        if (pending_flags & (UINT32)(((UINT32)kMU_Tx0EmptyFlag) >> regIndex)) {
            mu_isr_call_t call = calls->tx_empty_isr[regIndex];
            BOOLEAN done = TRUE;
            if (call == NULL) {
                rc_dis &= (UINT32)(~((UINT32)(((UINT32)kMU_Tx0EmptyInterruptEnable) >> regIndex)));
            }
            else {
                done = call(WdfInterrupt, MessageID,
#ifdef MU_ISR_USER_CONTAINER_T
                     &interrupt_data->isr_user_container,
#else
                    NULL,
#endif
#ifdef MU_DEVICE_USER_CONTAINER_T
                     &device_data->device_user_container
#else
                    NULL
#endif
                    );
            }
            if (done && (calls->tx_empty_dpc[regIndex] != NULL)) {
                schedule_dpc |= (UINT32)(((UINT32)kMU_Tx0EmptyFlag) >> regIndex);
            }
        }
        if (pending_flags & (UINT32)(((UINT32)kMU_GenInt0Flag) >> regIndex)) {
            mu_isr_call_t call = calls->gen_int_isr[regIndex];
            BOOLEAN done = TRUE;
            if (call == NULL) {
                rc_dis &= (UINT32)(~((UINT32)(((UINT32)kMU_GenInt0InterruptEnable) >> regIndex)));
            }
            else {
                done = call(WdfInterrupt, MessageID,
#ifdef MU_ISR_USER_CONTAINER_T
                     &interrupt_data->isr_user_container,
#else
                    NULL,
#endif
#ifdef MU_DEVICE_USER_CONTAINER_T
                     &device_data->device_user_container
#else
                    NULL
#endif
                    );
            }
            if (done && (calls->gen_int_dpc[regIndex] != NULL)) {
                schedule_dpc |= (UINT32)(((UINT32)kMU_GenInt0Flag) >> regIndex);
            }
        }
    }
    if (rc_dis != (UINT32)(~(UINT32)0U)) {
        _DataSynchronizationBarrier();
        base->CR &= rc_dis;
		device_data->CR_inten |= (UINT32)(~rc_dis);
        _DataSynchronizationBarrier();
    }
    if (schedule_dpc != 0U) {
		interrupt_data->schedule_dpc |= schedule_dpc;
        WdfInterruptQueueDpcForIsr(WdfInterrupt);
    }
    return TRUE;
}

VOID MU_DeferredProcedureCall(WDFINTERRUPT WdfInterrupt, WDFOBJECT MessageID)
{
    mu_interrupt_data_t interrupt_data;
    mu_device_data_t device_data;
    mu_isr_dpc_calls_t calls;
    MU_Type *base;
    UINT32 schedule_dpc;
    UINT32 cr_reg;

    interrupt_data = MU_GET_INTERRUPT_CONTEXT(WdfInterrupt);
    ASSERT(interrupt_data != NULL);
    device_data = MU_GET_DEVICE_CONTEXT(WdfInterruptGetDevice(WdfInterrupt));
    ASSERT(device_data != NULL);
    calls = &interrupt_data->isr_dpc_calls;
    base = device_data->base;
    ASSERT(base != NULL);
    WdfInterruptAcquireLock(WdfInterrupt);
    schedule_dpc = interrupt_data->schedule_dpc;
    interrupt_data->schedule_dpc = 0U;
    cr_reg = base->CR;
    device_data->CR_inten |= cr_reg & (MU_CR_GIEn_MASK | MU_CR_TIEn_MASK | MU_CR_RIEn_MASK);
    base->CR = cr_reg & ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK |
                          MU_CR_GIEn_MASK | MU_CR_TIEn_MASK | MU_CR_RIEn_MASK);
    WdfInterruptReleaseLock(WdfInterrupt);
    for (int regIndex = 0; regIndex < MU_TR_COUNT; regIndex++) {
        if (schedule_dpc & (UINT32)(((UINT32)kMU_Rx0FullFlag) >> regIndex)) {
            mu_dpc_call_t call = calls->rx_full_dpc[regIndex];
            if (call != NULL) {
                call(WdfInterrupt, MessageID,
#ifdef MU_ISR_USER_CONTAINER_T
                    & interrupt_data->isr_user_container,
#else
                    NULL,
#endif
#ifdef MU_DEVICE_USER_CONTAINER_T
                    & device_data->device_user_container
#else
                    NULL
#endif
                );
            }
        }
        if (schedule_dpc & (UINT32)(((UINT32)kMU_Tx0EmptyFlag) >> regIndex)) {
            mu_dpc_call_t call = calls->tx_empty_dpc[regIndex];
            if (call != NULL) {
                call(WdfInterrupt, MessageID,
#ifdef MU_ISR_USER_CONTAINER_T
                    & interrupt_data->isr_user_container,
#else
                    NULL,
#endif
#ifdef MU_DEVICE_USER_CONTAINER_T
                    & device_data->device_user_container
#else
                    NULL
#endif
                );
            }
        }
        if (schedule_dpc & (UINT32)(((UINT32)kMU_GenInt0Flag) >> regIndex)) {
            mu_dpc_call_t call = calls->gen_int_dpc[regIndex];
            if (call != NULL) {
                call(WdfInterrupt, MessageID,
#ifdef MU_ISR_USER_CONTAINER_T
                    & interrupt_data->isr_user_container,
#else
                    NULL,
#endif
#ifdef MU_DEVICE_USER_CONTAINER_T
                    & device_data->device_user_container
#else
                    NULL
#endif
                );
            }
        }
    }
    WdfInterruptAcquireLock(WdfInterrupt);
    if (device_data->CR_inten != 0U) {
        UINT32 CR_inten = device_data->CR_inten;
        device_data->CR_inten = 0U;
        MU_EnableInterrupts(base, CR_inten);
    }
    WdfInterruptReleaseLock(WdfInterrupt);
    return;
}

NTSTATUS MU_FromDevicePrepareHardware(WDFDEVICE device, MU_Type *base)
{
    NTSTATUS status = STATUS_SUCCESS;
    mu_device_data_t device_data = MU_GET_DEVICE_CONTEXT(device);
    device_data->CR_inten = 0U;
    device_data->base = base;
    MU_DisableInterrupts(base, (UINT32)(kMU_Tx0EmptyInterruptEnable)
        | (UINT32)(kMU_Tx1EmptyInterruptEnable) | (UINT32)(kMU_Tx2EmptyInterruptEnable)
        | (UINT32)(kMU_Tx3EmptyInterruptEnable) | (UINT32)(kMU_Rx0FullInterruptEnable)
        | (UINT32)(kMU_Rx1FullInterruptEnable) | (UINT32)(kMU_Rx2FullInterruptEnable)
        | (UINT32)(kMU_Rx3FullInterruptEnable) | (UINT32)(kMU_GenInt0InterruptEnable)
        | (UINT32)(kMU_GenInt1InterruptEnable) | (UINT32)(kMU_GenInt2InterruptEnable)
        | (UINT32)(kMU_GenInt3InterruptEnable) | (UINT32)(kMU_GenInt0InterruptTrigger)
        | (UINT32)(kMU_GenInt1InterruptTrigger) | (UINT32)(kMU_GenInt2InterruptTrigger)
        | (UINT32)(kMU_GenInt3InterruptTrigger));
    MU_ClearStatusFlags(base, (UINT32)(kMU_GenInt0Flag) | (UINT32)(kMU_GenInt1Flag)
        | (UINT32)(kMU_GenInt2Flag) | (UINT32)(kMU_GenInt3Flag));
    return status;
}

NTSTATUS MU_FromDeviceReleaseHardware(WDFDEVICE device)
{
    NTSTATUS status = STATUS_SUCCESS;
    mu_device_data_t device_data = MU_GET_DEVICE_CONTEXT(device);
    MU_Type *base = device_data->base;
    MU_DisableInterrupts(base, (UINT32)(kMU_Tx0EmptyInterruptEnable)
        | (UINT32)(kMU_Tx1EmptyInterruptEnable) | (UINT32)(kMU_Tx2EmptyInterruptEnable)
        | (UINT32)(kMU_Tx3EmptyInterruptEnable) | (UINT32)(kMU_Rx0FullInterruptEnable)
        | (UINT32)(kMU_Rx1FullInterruptEnable) | (UINT32)(kMU_Rx2FullInterruptEnable)
        | (UINT32)(kMU_Rx3FullInterruptEnable) | (UINT32)(kMU_GenInt0InterruptEnable)
        | (UINT32)(kMU_GenInt1InterruptEnable) | (UINT32)(kMU_GenInt2InterruptEnable)
        | (UINT32)(kMU_GenInt3InterruptEnable) | (UINT32)(kMU_GenInt0InterruptTrigger)
        | (UINT32)(kMU_GenInt1InterruptTrigger) | (UINT32)(kMU_GenInt2InterruptTrigger)
        | (UINT32)(kMU_GenInt3InterruptTrigger));
    MU_ClearStatusFlags(base, (UINT32)(kMU_GenInt0Flag) | (UINT32)(kMU_GenInt1Flag)
        | (UINT32)(kMU_GenInt2Flag) | (UINT32)(kMU_GenInt3Flag));
    return status;
}

NTSTATUS MU_FromAddDevice(WDFDEVICE device, WDFINTERRUPT interrupt, mu_isr_dpc_calls_t calls,
                            void *isr_container, void *device_container)
{
    NTSTATUS status = STATUS_SUCCESS;
    mu_device_data_t device_data = MU_GET_DEVICE_CONTEXT(device);
    mu_interrupt_data_t interrupt_data = MU_GET_INTERRUPT_CONTEXT(interrupt);
    interrupt_data->device = device_data;
#ifdef MU_ISR_USER_CONTAINER_T
    if (isr_container != NULL) {
        RtlCopyMemory(&interrupt_data->isr_user_container, isr_container, sizeof(MU_ISR_USER_CONTAINER_T));
    }
#else
    UNREFERENCED_PARAMETER(isr_container);
#endif
#ifdef MU_DEVICE_USER_CONTAINER_T
    if (device_container != NULL) {
        RtlCopyMemory(&device_data->device_user_container, device_container, sizeof(MU_DEVICE_USER_CONTAINER_T));
    }
#else
    UNREFERENCED_PARAMETER(device_container);
#endif
    if (calls == NULL) {
        RtlZeroMemory(&interrupt_data->isr_dpc_calls, sizeof(mu_isr_dpc_calls_struct));
    }
    else {
        RtlCopyMemory(&interrupt_data->isr_dpc_calls, calls, sizeof(mu_isr_dpc_calls_struct));
    }
    return status;
}

NTSTATUS MU_SendMsg(MU_Type *base, UINT32 regIndex, UINT32 msg)
{
    NTSTATUS result = STATUS_SUCCESS;
    if ((base == NULL) || (regIndex >= MU_TR_COUNT)){
        result = STATUS_INVALID_PARAMETER;
    }
    
    if (result == STATUS_SUCCESS) {
        result = STATUS_DEVICE_BUSY;
        /* Check TX register to be empty. */
        if (0U != (base->SR & (((UINT32)kMU_Tx0EmptyFlag) >> regIndex))) {
            _DataSynchronizationBarrier();
            base->TR[regIndex] = msg;
            result = STATUS_SUCCESS;
        }
    }
    return result;
}

NTSTATUS MU_ReceiveMsg(MU_Type *base, UINT32 regIndex, UINT32 *message)
{
    NTSTATUS result = STATUS_SUCCESS;
    if ((base == NULL) || (regIndex >= MU_TR_COUNT) || (message == NULL)) {
        result = STATUS_INVALID_PARAMETER;
    }
    
    if (result == STATUS_SUCCESS) {
        result = STATUS_DEVICE_BUSY;
        /* Check RX register to be full. */
        if (0U != (base->SR & (((UINT32)kMU_Rx0FullFlag) >> regIndex))) {
            _DataSynchronizationBarrier();
            *message = base->RR[regIndex];
            result = STATUS_SUCCESS;
        }
    }
    return result;
}

NTSTATUS MU_SetFlags(MU_Type *base, UINT32 flags)
{
    /* Check for update finished. */
    if (base->SR & MU_SR_FUP_MASK) {
        return STATUS_DEVICE_BUSY;
    }

    MU_SetFlagsNonBlocking(base, flags);
    return STATUS_SUCCESS;
}

NTSTATUS MU_TriggerInterrupts(MU_Type *base, UINT32 mask)
{
    NTSTATUS result = STATUS_SUCCESS;
    UINT32 reg;
    reg = base->CR;

    /* Previous interrupt has been accepted. */
    if (!(reg & mask)) {
        /* All interrupts have been accepted, trigger now. */
        reg = (reg & ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | mask;
        base->CR = reg;
    } else {
        result = STATUS_DEVICE_BUSY;
    }
    return result;
}

void MU_BootCoreB(MU_Type *base, mu_core_boot_mode_t mode)
{
    UINT32 reg = base->CR;

    reg = (reg & ~((MU_CR_GIRn_MASK | MU_CR_NMI_MASK) | MU_CR_BRSTH_MASK | MU_CR_BBOOT_MASK)) | MU_CR_BBOOT(mode);

    base->CR = reg;
}
