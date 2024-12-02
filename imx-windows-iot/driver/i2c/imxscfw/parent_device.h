// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022 NXP
// Licensed under the MIT License.
//
// Module Name:
//    parent_device.h
// Abstract:
//    Describes SCFW communication device for MU module.
//

#include <ntddk.h>
#include <wdf.h>
#include <initguid.h>
#include "SPBCx.h"

WDF_EXTERN_C_START

typedef struct _mu_strust_t MU_Type;
typedef struct _mu_isr_dpc_calls_struct *mu_isr_dpc_calls_t;

#ifdef ALLOC_PRAGMA
EVT_WDF_INTERRUPT_ISR MU_InterruptHandler; // NONPAGED
EVT_WDF_INTERRUPT_DPC MU_DeferredProcedureCall; // NONPAGED
NTSTATUS MU_FromDevicePrepareHardware(WDFDEVICE, MU_Type*);
#pragma alloc_text (PAGE, MU_FromDevicePrepareHardware)
NTSTATUS MU_FromDeviceReleaseHardware(WDFDEVICE);
#pragma alloc_text (PAGE, MU_FromDeviceReleaseHardware)
NTSTATUS MU_FromAddDevice(WDFDEVICE, WDFINTERRUPT, mu_isr_dpc_calls_t, void*, void*);
#pragma alloc_text (PAGE, MU_FromAddDevice)
NTSTATUS MU_SendMsg(MU_Type*, UINT32, UINT32); // NONPAGED
NTSTATUS MU_ReceiveMsg(MU_Type*, UINT32, UINT32*); // NONPAGED
NTSTATUS MU_TriggerInterrupts(MU_Type*, UINT32); // NONPAGED
#endif
WDF_EXTERN_C_END

#define MU_DEVICE_USER_CONTAINER_T scfw_device_struct_t
#define MU_ISR_USER_CONTAINER_T scfw_interrupt_struct_t

#define SCFW_RPC_MSG_WLEN 8U

typedef enum {
    scfw_ipc_tx,
    scfw_ipc_rx,
    scfw_ipc_rx_dpc,
    scfw_ipc_done,
    scfw_ipc_txonly,
    scfw_ipc_txonly_dpc,
    scfw_ipc_timeout,
    scfw_ipc_reset
} scfw_ipc_states_t;

typedef struct _mu_strust_t *mu_ptr_t;

typedef union {
    UINT8 raw8[SCFW_RPC_MSG_WLEN * 4U];
    UINT32 raw32[SCFW_RPC_MSG_WLEN];
    struct {
        UINT8 version;
        UINT8 size;
        UINT8 svc;
        UINT8 func;
        UINT8 data[(SCFW_RPC_MSG_WLEN - 1U) * 4U];
    } body;
} scfw_msg_buff_t;

typedef struct _scfw_device_struct_t {
    WDFDEVICE device;
	mu_ptr_t base;
    PHYSICAL_ADDRESS physical_address;
    ULONG registers_size;
	WDFSPINLOCK lock;
    WDFTIMER timeout_timer;
    WDFTIMER start_timer;
    WDFINTERRUPT interrupt;
} scfw_device_struct_t, *scfw_device_t;

typedef struct _scfw_interrupt_struct_t {
    SPBREQUEST request;
    scfw_msg_buff_t rx_buffer;
    scfw_msg_buff_t tx_buffer;
    PMDL rx_mdl;
    size_t rmdl_offset;
    size_t rbuf_offset;
    size_t tx_next;
    size_t rx_next;
    scfw_ipc_states_t state; // IPC state
} scfw_interrupt_struct_t, *scfw_interrupt_t;