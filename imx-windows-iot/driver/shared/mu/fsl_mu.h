/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
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
#ifndef _FSL_MU_H_
#define _FSL_MU_H_

/**
 * @addtogroup mu
 * @{
 */

/** @file */

/* ----------------------------------------------------------------------------
   -- MU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MU_Peripheral_Access_Layer MU Peripheral Access Layer
 * @{
 */

/** MU - Register Layout Typedef */
typedef struct _mu_strust_t {
    volatile UINT32 TR[4];                             /**< Transmit Register, array offset: 0x00, array step: 0x4 */
    volatile UINT32 RR[4];                             /**< Receive Register, array offset: 0x10, array step: 0x4 */
    volatile UINT32 SR;                                /**< Status Register, offset: 0x20 */
    volatile UINT32 CR;                                /**< Control Register, offset: 0x24 */
} MU_Type;

/* ----------------------------------------------------------------------------
   -- MU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MU_Register_Masks MU Register Masks
 * @{
 */

/*! @name TR - Transmit Register */
#define MU_TR_DATA_MASK                          (0xFFFFFFFFU)
#define MU_TR_DATA_SHIFT                         (0U)
#define MU_TR_DATA(x)                            (((UINT32)(((UINT32)(x)) << MU_TR_DATA_SHIFT)) & MU_TR_DATA_MASK)

/* The count of MU_TR */
#define MU_TR_COUNT                              (4U)

/*! @name RR - Receive Register */
#define MU_RR_DATA_MASK                          (0xFFFFFFFFU)
#define MU_RR_DATA_SHIFT                         (0U)
#define MU_RR_DATA(x)                            (((UINT32)(((UINT32)(x)) << MU_RR_DATA_SHIFT)) & MU_RR_DATA_MASK)

/* The count of MU_RR */
#define MU_RR_COUNT                              (4U)

/*! @name SR - Status Register */
#define MU_SR_Fn_MASK                            (0x7U)
#define MU_SR_Fn_SHIFT                           (0U)
#define MU_SR_Fn(x)                              (((UINT32)(((UINT32)(x)) << MU_SR_Fn_SHIFT)) & MU_SR_Fn_MASK)
#define MU_SR_NMIC_MASK                          (0x8U)
#define MU_SR_NMIC_SHIFT                         (3U)
#define MU_SR_NMIC(x)                            (((UINT32)(((UINT32)(x)) << MU_SR_NMIC_SHIFT)) & MU_SR_NMIC_MASK)
#define MU_SR_EP_MASK                            (0x10U)
#define MU_SR_EP_SHIFT                           (4U)
#define MU_SR_EP(x)                              (((UINT32)(((UINT32)(x)) << MU_SR_EP_SHIFT)) & MU_SR_EP_MASK)
#define MU_SR_PM_MASK                            (0x60U)
#define MU_SR_PM_SHIFT                           (5U)
#define MU_SR_PM(x)                              (((UINT32)(((UINT32)(x)) << MU_SR_PM_SHIFT)) & MU_SR_PM_MASK)
#define MU_SR_ARS_MASK                           (0x80U)
#define MU_SR_ARS_SHIFT                          (7U)
#define MU_SR_ARS(x)                             (((UINT32)(((UINT32)(x)) << MU_SR_ARS_SHIFT)) & MU_SR_ARS_MASK)
#define MU_SR_BRS_MASK                           (0x80U)
#define MU_SR_BRS_SHIFT                          (7U)
#define MU_SR_BRS(x)                             (((UINT32)(((UINT32)(x)) << MU_SR_BRS_SHIFT)) & MU_SR_BRS_MASK)
#define MU_SR_FUP_MASK                           (0x100U)
#define MU_SR_FUP_SHIFT                          (8U)
#define MU_SR_FUP(x)                             (((UINT32)(((UINT32)(x)) << MU_SR_FUP_SHIFT)) & MU_SR_FUP_MASK)
#define MU_SR_BRDIP_MASK                         (0x200U)
#define MU_SR_BRDIP_SHIFT                        (9U)
#define MU_SR_BRDIP(x)                           (((UINT32)(((UINT32)(x)) << MU_SR_BRDIP_SHIFT)) & MU_SR_BRDIP_MASK)
#define MU_SR_TEn_MASK                           (0xF00000U)
#define MU_SR_TEn_SHIFT                          (20U)
#define MU_SR_TEn(x)                             (((UINT32)(((UINT32)(x)) << MU_SR_TEn_SHIFT)) & MU_SR_TEn_MASK)
#define MU_SR_RFn_MASK                           (0xF000000U)
#define MU_SR_RFn_SHIFT                          (24U)
#define MU_SR_RFn(x)                             (((UINT32)(((UINT32)(x)) << MU_SR_RFn_SHIFT)) & MU_SR_RFn_MASK)
#define MU_SR_GIPn_MASK                          (0xF0000000U)
#define MU_SR_GIPn_SHIFT                         (28U)
#define MU_SR_GIPn(x)                            (((UINT32)(((UINT32)(x)) << MU_SR_GIPn_SHIFT)) & MU_SR_GIPn_MASK)

/*! @name CR - Control Register */
#define MU_CR_Fn_MASK                            (0x7U)
#define MU_CR_Fn_SHIFT                           (0U)
#define MU_CR_Fn(x)                              (((UINT32)(((UINT32)(x)) << MU_CR_Fn_SHIFT)) & MU_CR_Fn_MASK)
#define MU_CR_NMI_MASK                           (0x8U)
#define MU_CR_NMI_SHIFT                          (3U)
#define MU_CR_NMI(x)                             (((UINT32)(((UINT32)(x)) << MU_CR_NMI_SHIFT)) & MU_CR_NMI_MASK)
#define MU_CR_BHR_MASK                           (0x10U)
#define MU_CR_BHR_SHIFT                          (4U)
#define MU_SR_BHR(x)                             (((UINT32)(((UINT32)(x)) << MU_SR_BHR_SHIFT)) & MU_SR_BHR_MASK)
#define MU_CR_MUR_MASK                           (0x20U)
#define MU_CR_MUR_SHIFT                          (5U)
#define MU_CR_MUR(x)                             (((UINT32)(((UINT32)(x)) << MU_CR_MUR_SHIFT)) & MU_CR_MUR_MASK)
#define MU_CR_BRDIE_MASK                         (0x40U)
#define MU_CR_BRDIE_SHIFT                        (6U)
#define MU_SR_BRDIE(x)                           (((UINT32)(((UINT32)(x)) << MU_SR_BRDIE_SHIFT)) & MU_SR_BRDIE_MASK)
#define MU_CR_BRSTH_MASK                         (0x80U)
#define MU_CR_BRSTH_SHIFT                        (7U)
#define MU_CR_BRSTH(x)                           (((UINT32)(((UINT32)(x)) << MU_CR_BRSTH_SHIFT)) & MU_CR_BRSTH_MASK)
#define MU_CR_CLKE_MASK                          (0x100U)
#define MU_CR_CLKE_SHIFT                         (8U)
#define MU_CR_CLKE(x)                            (((UINT32)(((UINT32)(x)) << MU_CR_CLKE_SHIFT)) & MU_CR_CLKE_MASK)
#define MU_CR_BBOOT_MASK                         (0x600U)
#define MU_CR_BBOOT_SHIFT                        (9U)
#define MU_CR_BBOOT(x)                           (((UINT32)(((UINT32)(x)) << MU_CR_BBOOT_SHIFT)) & MU_CR_BBOOT_MASK)
#define MU_CR_GIRn_MASK                          (0xF0000U)
#define MU_CR_GIRn_SHIFT                         (16U)
#define MU_CR_GIRn(x)                            (((UINT32)(((UINT32)(x)) << MU_CR_GIRn_SHIFT)) & MU_CR_GIRn_MASK)
#define MU_CR_TIEn_MASK                          (0xF00000U)
#define MU_CR_TIEn_SHIFT                         (20U)
#define MU_CR_TIEn(x)                            (((UINT32)(((UINT32)(x)) << MU_CR_TIEn_SHIFT)) & MU_CR_TIEn_MASK)
#define MU_CR_RIEn_MASK                          (0xF000000U)
#define MU_CR_RIEn_SHIFT                         (24U)
#define MU_CR_RIEn(x)                            (((UINT32)(((UINT32)(x)) << MU_CR_RIEn_SHIFT)) & MU_CR_RIEn_MASK)
#define MU_CR_GIEn_MASK                          (0xF0000000U)
#define MU_CR_GIEn_SHIFT                         (28U)
#define MU_CR_GIEn(x)                            (((UINT32)(((UINT32)(x)) << MU_CR_GIEn_SHIFT)) & MU_CR_GIEn_MASK)


/*!
 * @}
 */ /* end of group MU_Register_Masks */

/*!
 * @}
 */ /* end of group MU_Peripheral_Access_Layer */

/******************************************************************************
 * Definitions
 *****************************************************************************/

/** @name Driver version */
/*@{*/
/** @brief MU driver version 2.0.0. */
#define FSL_MU_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/** @name MU status flags */
/*@{*/
#define kMU_Tx0EmptyFlag (MU_SR_TEn(8U)) /**< TX0 empty. */
#define kMU_Tx1EmptyFlag (MU_SR_TEn(4U)) /**< TX1 empty. */
#define kMU_Tx2EmptyFlag (MU_SR_TEn(2U)) /**< TX2 empty. */
#define kMU_Tx3EmptyFlag (MU_SR_TEn(1U)) /**< TX3 empty. */

#define kMU_Rx0FullFlag (MU_SR_RFn(8U)) /**< RX0 full.  */
#define kMU_Rx1FullFlag (MU_SR_RFn(4U)) /**< RX1 full.  */
#define kMU_Rx2FullFlag (MU_SR_RFn(2U)) /**< RX2 full.  */
#define kMU_Rx3FullFlag (MU_SR_RFn(1U)) /**< RX3 full.  */

#define kMU_GenInt0Flag (MU_SR_GIPn(8U)) /**< General purpose interrupt 0 pending. */
#define kMU_GenInt1Flag (MU_SR_GIPn(4U)) /**< General purpose interrupt 1 pending. */
#define kMU_GenInt2Flag (MU_SR_GIPn(2U)) /**< General purpose interrupt 2 pending. */
#define kMU_GenInt3Flag (MU_SR_GIPn(1U)) /**< General purpose interrupt 3 pending. */

#define kMU_EventPendingFlag MU_SR_EP_MASK   /**< MU event pending.            */
#define kMU_FlagsUpdatingFlag MU_SR_FUP_MASK /**< MU flags update is on-going. */
/*@}*/


/** @name MU interrupt source to enable. */
/*@{*/
#define kMU_Tx0EmptyInterruptEnable (MU_CR_TIEn(8U)) /**< TX0 empty. */
#define kMU_Tx1EmptyInterruptEnable (MU_CR_TIEn(4U)) /**< TX1 empty. */
#define kMU_Tx2EmptyInterruptEnable (MU_CR_TIEn(2U)) /**< TX2 empty. */
#define kMU_Tx3EmptyInterruptEnable (MU_CR_TIEn(1U)) /**< TX3 empty. */

#define kMU_Rx0FullInterruptEnable (MU_CR_RIEn(8U)) /**< RX0 full. */
#define kMU_Rx1FullInterruptEnable (MU_CR_RIEn(4U)) /**< RX1 full. */
#define kMU_Rx2FullInterruptEnable (MU_CR_RIEn(2U)) /**< RX2 full. */
#define kMU_Rx3FullInterruptEnable (MU_CR_RIEn(1U)) /**< RX3 full. */

#define kMU_GenInt0InterruptEnable (MU_CR_GIEn(8U)) /**< General purpose interrupt 0. */
#define kMU_GenInt1InterruptEnable (MU_CR_GIEn(4U)) /**< General purpose interrupt 1. */
#define kMU_GenInt2InterruptEnable (MU_CR_GIEn(2U)) /**< General purpose interrupt 2. */
#define kMU_GenInt3InterruptEnable (MU_CR_GIEn(1U)) /**< General purpose interrupt 3. */
/*@}*/

/** @name MU interrupt that could be triggered to the other core. */
/*@{*/
#define kMU_NmiInterruptTrigger MU_CR_NMI_MASK       /**< NMI interrupt.               */
#define kMU_GenInt0InterruptTrigger (MU_CR_GIRn(8U)) /**< General purpose interrupt 0. */
#define kMU_GenInt1InterruptTrigger (MU_CR_GIRn(4U)) /**< General purpose interrupt 1. */
#define kMU_GenInt2InterruptTrigger (MU_CR_GIRn(2U)) /**< General purpose interrupt 2. */
#define kMU_GenInt3InterruptTrigger (MU_CR_GIRn(1U)) /**< General purpose interrupt 3. */
/*@}*/

/**
 * @brief Core B boot mode.
 */
typedef enum _mu_core_boot_mode {
    kMU_CoreBootFromAddr0 = 0x00U, /**< Boot from 0x00.      */
    kMU_CoreBootFromDmem = 0x01U,  /**< Boot from DMEM base. */
    kMU_CoreBootFromImem = 0x02U   /**< Boot from IMEM base. */
} mu_core_boot_mode_t;

/**
 * @brief Power mode definition.
 */
typedef enum _mu_power_mode {
    kMU_PowerModeRun = 0x00U,  /**< Run mode.           */
    kMU_PowerModeWait = 0x01U, /**< WAIT mode.          */
    kMU_PowerModeStop = 0x02U, /**< STOP/VLPS mode.     */
    kMU_PowerModeDsm = 0x03U   /**< DSM: LLS/VLLS mode. */
} mu_power_mode_t;

/**
 * @brief Messaging Unit device configuration structure.
 *
 * This structure holds peripheral base address, the interrupt vector,
 * semaphores used during send/receive, general request call-backs,
 * and pointer to a user data.
 */
typedef struct _mu_device_data_struct {
#ifdef MU_DEVICE_USER_CONTAINER_T
    MU_DEVICE_USER_CONTAINER_T device_user_container;
#endif /* MU_DEVICE_USER_CONTAINER_T */
    MU_Type *base; /**< peripheral base address */
    UINT32 CR_inten; /**< stores enablet interrupts in MU_CR register */
} mu_device_data_struct_t, *mu_device_data_t;

typedef BOOLEAN (*mu_isr_call_t)(WDFINTERRUPT, ULONG, void*, void*);
typedef VOID(*mu_dpc_call_t)(WDFINTERRUPT, WDFOBJECT, void*, void*);


/**
 * @brief Messaging Unit device configuration structure.
 *
 * This structure holds peripheral base address, the interrupt vector,
 * semaphores used during send/receive, general request call-backs,
 * and pointer to a user data.
 */
typedef struct _mu_isr_dpc_calls_struct {
    mu_isr_call_t tx_empty_isr[MU_TR_COUNT]; /**< tx buffers empty isr calls */
    mu_isr_call_t rx_full_isr[MU_RR_COUNT]; /**< rx buffers full isr calls */
    mu_isr_call_t gen_int_isr[4]; /**< general requests isr calls */
    mu_dpc_call_t tx_empty_dpc[MU_TR_COUNT]; /**< tx buffers empty dpc calls */
    mu_dpc_call_t rx_full_dpc[MU_RR_COUNT]; /**< rx buffers full dpc calls */
    mu_dpc_call_t gen_int_dpc[4]; /**< general requests dpc calls */
} mu_isr_dpc_calls_struct, *mu_isr_dpc_calls_t;

/**
 * @brief Messaging Unit device configuration structure.
 *
 * This structure holds peripheral base address, the interrupt vector,
 * semaphores used during send/receive, general request call-backs,
 * and pointer to a user data.
 */
typedef struct _mu_interrupt_data_struct {
#ifdef MU_ISR_USER_CONTAINER_T
    MU_ISR_USER_CONTAINER_T isr_user_container;
#endif /* MU_ISR_USER_CONTAINER_T */
    mu_device_data_t device; /**< device configuration structure */
    mu_isr_dpc_calls_struct isr_dpc_calls; /**< isr + dpc callbacks structure */
    UINT32 schedule_dpc; /**< remembers DPC calls to invoke */
} mu_interrupt_data_struct_t, *mu_interrupt_data_t;

#ifndef MU_GET_INTERRUPT_CONTEXT
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(
    mu_interrupt_data_struct_t,
    MU_GET_INTERRUPT_CONTEXT);
#endif /* MU_GET_INTERRUPT_CONTEXT */

#ifndef MU_GET_DEVICE_CONTEXT
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(
    mu_device_data_struct_t,
    MU_GET_DEVICE_CONTEXT);
#endif /* MU_GET_DEVICE_CONTEXT */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

#ifdef MU_ISR_USER_CONTAINER_T
/**
 * @brief Retrieves ISR user data container.
 *
 * Extracts ISR user structure from MU interrupt data structure.
 *
 * @param[in] interrupt interrupt where is MU registered
 *
 * @return pointer to isr user container
 */
inline MU_ISR_USER_CONTAINER_T* get_isr_user_container(WDFINTERRUPT interrupt)
{
    mu_interrupt_data_t interrupt_data = MU_GET_INTERRUPT_CONTEXT(interrupt);
    return &interrupt_data->isr_user_container;
}
#endif

#ifdef MU_DEVICE_USER_CONTAINER_T
/**
 * @brief Retrieves Device user data container.
 *
 * Extracts Device user structure from MU device data structure.
 *
 * @param[in] device device where is MU registered
 *
 * @return pointer to isr user container
 */
inline MU_DEVICE_USER_CONTAINER_T* get_device_user_container(WDFDEVICE device)
{
    mu_device_data_t device_data = MU_GET_DEVICE_CONTEXT(device);
    return &device_data->device_user_container;
}
#endif

/**
 * @brief Interrupt handler.
 *
 * @param[in] WdfInterrupt filled automatically
 * @param[in] MessageID filled automatically
 *
 * @return reports whether ISR was handled or not.
 * @retval TRUE handlet.
 * @retval FALSE not.
 */
BOOLEAN MU_InterruptHandler(WDFINTERRUPT WdfInterrupt, ULONG MessageID);

/**
 * @brief Deferred procedure call.
 *
 * @param[in] WdfInterrupt filled automatically
 * @param[in] MessageID filled automatically
 *
 * @return reports whether ISR was handled or not.
 * @retval TRUE handlet.
 * @retval FALSE not.
 */
VOID MU_DeferredProcedureCall(WDFINTERRUPT WdfInterrupt, WDFOBJECT MessageID);

/**
 * @name MU initialization.
 * @{
 */

/**
 * @brief Initializes the MU device and data structure.
 *
 * It is should called from EvtDevicePrepareHardware function
 * (not from WDF_PNPPOWER_EVENT_CALLBACKS).
 *
 * @code
 * const CM_PARTIAL_RESOURCE_DESCRIPTOR* mem_resource =
 *     WdfCmResourceListGetDescriptor(ResourcesTranslated, mem_index);
 * MU_Type *base = MmMapIoSpaceEx(mem_resource->u.Memory.Start,
 *     sizeof(MU_Type), PAGE_READWRITE | PAGE_NOCACHE);
 * MU_FromDevicePrepareHardware(device, base);
 * @endcode
 *
 * @param[in] device device where is MU registered
 * @param[in] base device base address
 *
 * @return reports error code of the function.
 * @retval Failure function fails.
 */
NTSTATUS MU_FromDevicePrepareHardware(WDFDEVICE device, MU_Type *base);

/**
 * @brief De-initializes the MU device.
 *
 * It is should called from EvtDeviceReleaseHardware function
 * (not from WDF_PNPPOWER_EVENT_CALLBACKS).
 *
 * @code
 * MU_FromDevicePrepareHardware(device);
 * @endcode
 *
 * @param[in] device device where is MU registered
 *
 * @return reports error code of the function.
 * @retval Failure function fails.
 */
NTSTATUS MU_FromDeviceReleaseHardware(WDFDEVICE device);

/**
 * @brief Initializes the MU structures and user's data containers.
 *
 * It is should called from EvtDriverDeviceAdd function (not from WDF_DRIVER_CONFIG).
 *
 * @code
 * WDFDEVICE device;
 * WDF_OBJECT_ATTRIBUTES d_attributes;
 * WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&d_attributes, mu_device_data_struct_t);
 * WdfDeviceCreate(&DeviceInitPtr, &d_attributes, &device);
 * WDF_OBJECT_ATTRIBUTES i_attributes;
 * WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&i_attributes, mu_interrupt_data_struct_t);
 * WDF_INTERRUPT_CONFIG i_config;
 * WDF_INTERRUPT_CONFIG_INIT(&i_config, MU_InterruptHandler, MU_DeferredProcedureCall);
 * WDFINTERRUPT interrupt;
 * WdfInterruptCreate(device, &i_config, &i_attributes, &interrupt);
 * mu_isr_dpc_calls_struct calls = {.tx_empty_isr[2] = func_ch2tx_isr};
 * MU_FromAddDevice(device, interrupt, &calls, &isr_user_container, &device_user_container);
 * @endcode
 *
 * @param[in] device device where is MU registered
 *
 * @return reports error code of the function.
 * @retval Failure function fails.
 */
NTSTATUS MU_FromAddDevice(WDFDEVICE device, WDFINTERRUPT interrupt, mu_isr_dpc_calls_t calls,
                            void *isr_container, void *device_container);

/* @} */

/**
 * @name MU Message
 * @{
 */

/**
 * @brief Writes a message to the TX register.
 *
 * This function writes a message to the specific TX register. It does not check
 * whether the TX register is empty or not. The upper layer should make sure the TX
 * register is empty before calling this function. This function can be used
 * in ISR for better performance.
 *
 * @code
 * while (!(kMU_Tx0EmptyFlag & MU_GetStatusFlags(base))) { } // Wait for TX0 register empty.
 * MU_SendMsgNonBlocking(base, 0U, MSG_VAL); // Write message to the TX0 register.
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param regIndex  TX register index.
 * @param msg      Message to send.
 */
static inline void MU_SendMsgNonBlocking(MU_Type *base, UINT32 regIndex, UINT32 msg)
{
    ASSERT(regIndex < MU_TR_COUNT);

    base->TR[regIndex] = msg;
}

/**
 * @brief Blocks to send a message.
 *
 * This function waits until the TX register is empty and sends the message.
 *
 * @param[in] base MU peripheral base address.
 * @param regIndex  TX register index.
 * @param msg      Message to send.
 * @return Error state.
 * @retval STATUS_SUCCESS    Message have been sent successfully.
 * @retval STATUS_DEVICE_BUSY       Transmitter is full.
 * @retval STATUS_INVALID_PARAMETER Parameter is NULL or out of range.
 */
NTSTATUS MU_SendMsg(MU_Type *base, UINT32 regIndex, UINT32 msg);

/**
 * @brief Reads a message from the RX register.
 *
 * This function reads a message from the specific RX register. It does not check
 * whether the RX register is full or not. The upper layer should make sure the RX
 * register is full before calling this function. This function can be used
 * in ISR for better performance.
 *
 * @code
 * UINT32 msg;
 * while (!(kMU_Rx0FullFlag & MU_GetStatusFlags(base)))
 * {
 * } // Wait for the RX0 register full.
 *
 * msg = MU_ReceiveMsgNonBlocking(base, 0U);  // Read message from RX0 register.
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param regIndex  TX register index.
 * @return The received message.
 */
static inline UINT32 MU_ReceiveMsgNonBlocking(MU_Type *base, UINT32 regIndex)
{
    ASSERT(regIndex < MU_TR_COUNT);

    return base->RR[regIndex];
}

/**
 * @brief Blocks to receive a message.
 *
 * This function waits until the RX register is full and receives the message.
 *
 * @param[in] base MU peripheral base address.
 * @param regIndex  RX register index.
 * @param message  pointer to the message buffer.
 * @return Error state.
 * @retval STATUS_SUCCESS    Message have been sent successfully.
 * @retval STATUS_DEVICE_BUSY       Receiver is empty.
 * @retval STATUS_INVALID_PARAMETER Parameter is NULL or out of range.
 */
NTSTATUS MU_ReceiveMsg(MU_Type *base, UINT32 regIndex, UINT32 *message);

/* @} */

/**
 * @name MU Flags
 * @{
 */

/**
 * @brief Sets the 3-bit MU flags reflect on the other MU side.
 *
 * This function sets the 3-bit MU flags directly. Every time the 3-bit MU flags are changed,
 * the status flag \c kMU_FlagsUpdatingFlag Asserts indicating the 3-bit MU flags are
 * updating to the other side. After the 3-bit MU flags are updated, the status flag
 * \c kMU_FlagsUpdatingFlag is cleared by hardware. During the flags updating period,
 * the flags cannot be changed. The upper layer should make sure the status flag
 * \c kMU_FlagsUpdatingFlag is cleared before calling this function.
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * while (kMU_FlagsUpdatingFlag & MU_GetStatusFlags(base))
 * {
 * } // Wait for previous MU flags updating.
 *
 * MU_SetFlagsNonBlocking(base, 0U); // Set the mU flags.
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param flags The 3-bit MU flags to set.
 */
static inline void MU_SetFlagsNonBlocking(MU_Type *base, UINT32 flags)
{
    UINT32 reg;
    reg = base->CR;
    reg = (reg & ~((MU_CR_GIRn_MASK | MU_CR_NMI_MASK) | MU_CR_Fn_MASK)) | MU_CR_Fn(flags);
    base->CR = reg;
}

/**
 * @brief Blocks setting the 3-bit MU flags reflect on the other MU side.
 *
 * This function blocks setting the 3-bit MU flags. Every time the 3-bit MU flags are changed,
 * the status flag \c kMU_FlagsUpdatingFlag Asserts indicating the 3-bit MU flags are
 * updating to the other side. After the 3-bit MU flags are updated, the status flag
 * \c kMU_FlagsUpdatingFlag is cleared by hardware. During the flags updating period,
 * the flags cannot be changed. This function waits for the MU status flag
 * \c kMU_FlagsUpdatingFlag cleared and sets the 3-bit MU flags.
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @param[in] base MU peripheral base address.
 * @param flags The 3-bit MU flags to set.
 * @return Error state.
 * @retval STATUS_SUCCESS    Message have been sent successfully.
 * @retval STATUS_DEVICE_BUSY       Transmitter is full.
 */
NTSTATUS MU_SetFlags(MU_Type *base, UINT32 flags);

/**
 * @brief Gets the current value of the 3-bit MU flags set by the other side.
 *
 * This functions gets the current 3-bit MU flags on the current side.
 *
 * @param[in] base MU peripheral base address.
 * @return flags   Current value of the 3-bit flags.
 */
static inline UINT32 MU_GetFlags(MU_Type *base)
{
    return (base->SR & MU_SR_Fn_MASK) >> MU_SR_Fn_SHIFT;
}

/*!
 * @brief Gets the general purpose interrupt request acceptance by the other side.
 *
 * This function gets acceptance of general purpose interrupt.
 *
 * @param base MU peripheral base address.
 * @return Bit mask of the MU GIRn, see _mu_interrupt_trigger.
 */
static inline UINT32 MU_GetAcceptance(MU_Type *base)
{
    return base->CR & MU_CR_GIRn_MASK;
}

/* @} */

/**
 * @name Status and Interrupt.
 * @{
 */

/**
 * @brief Gets the MU status flags.
 *
 * This function returns the bit mask of the MU status flags. See "MU status flags".
 *
 * @code
 * UINT32 flags;
 * flags = MU_GetStatusFlags(base); // Get all status flags.
 * if (kMU_Tx0EmptyFlag & flags)
 * {
 *     // The TX0 register is empty. Message can be sent.
 *     MU_SendMsgNonBlocking(base, 0U, MSG0_VAL);
 * }
 * if (kMU_Tx1EmptyFlag & flags)
 * {
 *     // The TX1 register is empty. Message can be sent.
 *     MU_SendMsgNonBlocking(base, 1U, MSG1_VAL);
 * }
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @return      Bit mask of the MU status flags, see "MU status flags".
 */
static inline UINT32 MU_GetStatusFlags(MU_Type *base)
{
    return (base->SR & (MU_SR_TEn_MASK | MU_SR_RFn_MASK | MU_SR_GIPn_MASK |
                        MU_SR_BRDIP_MASK | MU_SR_FUP_MASK | MU_SR_BRS_MASK | MU_SR_EP_MASK | MU_SR_Fn_MASK));
}

/**
 * @brief Gets the MU IRQ pending status.
 *
 * This function returns the bit mask of the pending MU IRQs.
 *
 * @param[in] base MU peripheral base address.
 * @return      Bit mask of the MU IRQs pending.
 */
static inline UINT32 MU_GetInterruptsPending(MU_Type *base)
{
    UINT32 irqMask = base->CR & (MU_CR_GIEn_MASK | MU_CR_TIEn_MASK | MU_CR_RIEn_MASK);
    return (base->SR & irqMask);
}

/**
 * @brief Clears the specific MU status flags.
 *
 * This function clears the specific MU status flags. The flags to clear should
 * be passed in as bit mask. See "MU status flags".
 *
 * @code
 * //Clear general interrupt 0 and general interrupt 1 pending flags.
 * MU_ClearStatusFlags(base, kMU_GenInt0Flag | kMU_GenInt1Flag);
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param mask  Bit mask of the MU status flags. See "MU status flags".
 *
 * @note Only clear the general purpose interrupt flags using this function.
 * The other flags, such as kMU_Rx0FullFlag, are cleared by hardware.
 */
static inline void MU_ClearStatusFlags(MU_Type *base, UINT32 mask)
{
    base->SR = (mask & MU_SR_GIPn_MASK);
}

/**
 * @brief Enables the specific MU interrupts.
 *
 * This function enables the specific MU interrupts. The interrupts to enable
 * should be passed in as bit mask. See "MU interrupt source to enable".
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * // Enable general interrupt 0 and TX0 empty interrupt.
 * MU_EnableInterrupts(base, kMU_GenInt0InterruptEnable | kMU_Tx0EmptyInterruptEnable);
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param mask  Bit mask of the MU interrupts. See "MU interrupt source to enable".
 */
static inline void MU_EnableInterrupts(MU_Type *base, UINT32 mask)
{
    UINT32 reg;
    reg = base->CR;
    reg = (reg & ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | mask;
    base->CR = reg;
}

/**
 * @brief Restore requested MU interrupts settings.
 *
 * Interrupt routine could disable some interruts (and saves the info).
 * This function re-enables them.
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * // Restore requested interrupt settings.
 * MU_ReEnableInterrupts(device);
 * @endcode
 *
 * @param[in] device MU device.
 */
static inline void MU_ReEnableInterrupts(WDFDEVICE device)
{
    mu_device_data_t device_data = MU_GET_DEVICE_CONTEXT(device);
    MU_Type *base = device_data->base;
    UINT32 reg = base->CR;
    reg = (reg & ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | device_data->CR_inten;
    base->CR = reg;
}

/**
 * @brief Enables the Tx MU interrupts.
 *
 * This function enables one Tx MU interrupt (specified by regIndex).
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * // Enable TX2 empty interrupt.
 * MU_EnableSendInterrupts(base, 2U);
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param regIndex specifies interrupt index.
 */
static inline void MU_EnableSendInterrupts(MU_Type *base, UINT32 regIndex)
{
    UINT32 reg;
    reg = base->CR;
    reg = (reg & ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | (UINT32)(((UINT32)kMU_Tx0EmptyInterruptEnable) >> regIndex);
    base->CR = reg;
}

/**
 * @brief Enables the Rx MU interrupts.
 *
 * This function enables one Rx MU interrupt (specified by regIndex).
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * // Enable RX3 full interrupt.
 * MU_EnableReceiveInterrupts(base, 3U);
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param regIndex specifies interrupt index.
 */
static inline void MU_EnableReceiveInterrupts(MU_Type *base, UINT32 regIndex)
{
    UINT32 reg;
    reg = base->CR;
    reg = (reg & ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | (UINT32)(((UINT32)kMU_Rx0FullInterruptEnable) >> regIndex);
    base->CR = reg;
}

/**
 * @brief Enables the general MU interrupts.
 *
 * This function enables one general MU interrupt (specified by regIndex).
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * // Enable general interrupt 1.
 * MU_EnableTriggerInterrupts(base, 1U);
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param regIndex specifies interrupt index.
 */
static inline void MU_EnableTriggerInterrupts(MU_Type *base, UINT32 regIndex)
{
    UINT32 reg;
    reg = base->CR;
    reg = (reg & ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | (UINT32)(((UINT32)kMU_GenInt0InterruptEnable) >> regIndex);
    base->CR = reg;
}

/**
 * @brief Disables the specific MU interrupts.
 *
 * This function disables the specific MU interrupts. The interrupts to disable
 * should be passed in as bit mask. See "MU interrupt source to enable".
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * // Disable general interrupt 0 and TX0 empty interrupt.
 * MU_DisableInterrupts(base, kMU_GenInt0InterruptEnable | kMU_Tx0EmptyInterruptEnable);
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param mask  Bit mask of the MU interrupts. See "MU interrupt source to enable".
 */
static inline void MU_DisableInterrupts(MU_Type *base, UINT32 mask)
{
    UINT32 reg;
    reg = base->CR;
    reg &= ~((MU_CR_GIRn_MASK | MU_CR_NMI_MASK) | mask);
    base->CR = reg;
}

/**
 * @brief Disables the specific MU interrupts from DPC.
 *
 * This function disables the specific MU interrupts from DPC. The interrupts to disable
 * should be passed in as bit mask. See "MU interrupt source to enable".
 * This function is intended to be called from DPC and cannot be executed concurently with MU's ISR,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * // Disable general interrupt 0 and TX0 empty interrupt.
 * MU_DisableDPCInterrupts(device, kMU_GenInt0InterruptEnable | kMU_Tx0EmptyInterruptEnable);
 * @endcode
 *
 * @param[in] device MU device.
 */
static inline void MU_DisableDPCInterrupts(WDFDEVICE device, UINT32 mask)
{
    mu_device_data_t device_data = MU_GET_DEVICE_CONTEXT(device);
    device_data->CR_inten &= ~(mask & (MU_CR_GIEn_MASK | MU_CR_TIEn_MASK | MU_CR_RIEn_MASK));
}

/**
 * @brief Disables the Tx MU interrupts.
 *
 * This function disables one Tx MU interrupt (specified by regIndex).
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * // Disable TX2 empty interrupt.
 * MU_DisableSendInterrupts(base, 2U);
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param regIndex specifies interrupt index.
 */
static inline void MU_DisableSendInterrupts(MU_Type *base, UINT32 regIndex)
{
    UINT32 reg;
    reg = base->CR;
    reg &= ~((MU_CR_GIRn_MASK | MU_CR_NMI_MASK) | (UINT32)(((UINT32)kMU_Tx0EmptyInterruptEnable) >> regIndex));
    base->CR = reg;
}

/**
 * @brief Disables the Rx MU interrupts.
 *
 * This function disables one Rx MU interrupt (specified by regIndex).
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * // Enable RX3 full interrupt.
 * MU_DisableReceiveInterrupts(base, 3U);
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param regIndex specifies interrupt index.
 */
static inline void MU_DisableReceiveInterrupts(MU_Type *base, UINT32 regIndex)
{
    UINT32 reg;
    reg = base->CR;
    reg &= ~((MU_CR_GIRn_MASK | MU_CR_NMI_MASK) | (UINT32)(((UINT32)kMU_Rx0FullInterruptEnable) >> regIndex));
    base->CR = reg;
}

/**
 * @brief Disables the general MU interrupts.
 *
 * This function disables one general MU interrupt (specified by regIndex).
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * // Disable general interrupt 1.
 * MU_DisableTriggerInterrupts(base, 1U);
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param regIndex specifies interrupt index.
 */
static inline void MU_DisableTriggerInterrupts(MU_Type *base, UINT32 regIndex)
{
    UINT32 reg;
    reg = base->CR;
    reg &= ~((MU_CR_GIRn_MASK | MU_CR_NMI_MASK) | (UINT32)(((UINT32)kMU_GenInt0InterruptEnable) >> regIndex));
    base->CR = reg;
}

/**
 * @brief Triggers interrupts to the other core.
 *
 * This function triggers the specific interrupts to the other core. The interrupts
 * to trigger are passed in as bit mask.
 * See \ref "MU interrupt that could be triggered to the other core".
 * The MU should not trigger an interrupt to the other core when the previous interrupt
 * has not been processed by the other core. This function checks whether the
 * previous interrupts have been processed. If not, it returns an error.
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @code
 * if (STATUS_SUCCESS != MU_TriggerInterrupts(base, kMU_GenInt0InterruptTrigger | kMU_GenInt2InterruptTrigger))
 * {
 *     // Previous general purpose interrupt 0 or general purpose interrupt 2
 *     // has not been processed by the other core.
 * }
 * @endcode
 *
 * @param[in] base MU peripheral base address.
 * @param mask Bit mask of the interrupts to trigger.
 *        See "MU interrupt that could be triggered to the other core".
 * @retval STATUS_SUCCESS    Interrupts have been triggered successfully.
 * @retval STATUS_DEVICE_BUSY       Previous interrupts have not been accepted.
 */
NTSTATUS MU_TriggerInterrupts(MU_Type *base, UINT32 mask);

/**
 * @brief Clear non-maskable interrupt (NMI) sent by the other core.
 *
 * This functions clears non-maskable interrupt (NMI) sent by the other core.
 *
 * @param[in] base MU peripheral base address.
 */
static inline void MU_ClearNmi(MU_Type *base)
{
    base->SR = MU_SR_NMIC_MASK;
}

/* @} */

/**
 * @name MU misc functions
 * @{
 */

/**
 * @brief Boots the core B.
 *
 * This function boots the core B with a boot configuration.
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @param[in] base MU peripheral base address.
 * @param mode Core B boot mode.
 * @note Only MU side A can use this function.
 */
void MU_BootCoreB(MU_Type *base, mu_core_boot_mode_t mode);

/**
 * @brief Holds the core B reset.
 *
 * This function causes the core B to be held in reset following any reset event.
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @param[in] base MU peripheral base address.
 */
static inline void MU_HoldCoreBReset(MU_Type *base)
{
    UINT32 reg = base->CR;
    reg = (reg & ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | MU_CR_BRSTH_MASK;
    base->CR = reg;
}

/**
 * @brief Resets the MU for both A side and B side.
 *
 * This function resets the MU for both A side and B side. Before reset, it is
 * recommended to interrupt processor B, because this function may affect the
 * ongoing processor B programs.
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @param[in] base MU peripheral base address.
 * @note Only MU side A could use this function.
 */
static inline void MU_ResetBothSides(MU_Type *base)
{
    UINT32 reg = base->CR;
    reg = (reg & ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | MU_CR_MUR_MASK;
    base->CR = reg;
}

/**
 * @brief Enables or disables the clock on the other core.
 *
 * This function enables or disables the platform clock on the other core when
 * that core enters a stop mode. If disabled, the platform clock for the other
 * core is disabled when it enters stop mode. If enabled, the platform clock
 * keeps running on the other core in stop mode, until this core also enters
 * stop mode.
 * This function cannot be executed concurently with MU's ISR or DPC,
 * it can be executed from WdfInterruptAcquireLock - WdfInterruptReleaseLock block,
 * or from WdfInterruptSynchronize.
 *
 * @param[in] base MU peripheral base address.
 * @param enable   Enable or disable the clock on the other core.
 */
static inline void MU_SetClockOnOtherCoreEnable(MU_Type *base, BOOLEAN enable)
{
    UINT32 reg = base->CR;

    reg &= ~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK);

    if (enable) {
        reg |= MU_CR_CLKE_MASK;
    } else {
        reg &= ~MU_CR_CLKE_MASK;
    }

    base->CR = reg;
}

/**
 * @brief Gets the power mode of the other core.
 *
 * This function gets the power mode of the other core.
 *
 * @param[in] base MU peripheral base address.
 * @return Power mode of the other core.
 */
static inline mu_power_mode_t MU_GetOtherCorePowerMode(MU_Type *base)
{
    return (mu_power_mode_t)((base->SR & MU_SR_PM_MASK) >> MU_SR_PM_SHIFT);
}

/* @} */

#if defined(__cplusplus)
}
#endif /*_cplusplus*/
/*@}*/

#endif /* _FSL_MU_H_*/
