// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022 NXP
// Licensed under the MIT License.
//
// Module Name:
//    imxscfw.c
// Abstract:
//    The module implements SCFW communication device.
//

#include "parent_device.h"
#include <ntddser.h>
#include <devpkey.h>
#include <acpiioct.h>

#include "fsl_mu.h"

#include "trace.h"
#include "imxscfw.tmh"

#define IMX_SCFW_POOL_TAG ((ULONG) 'SCFW')

// {12B21D25-AF9F-4394-BDD3-D755FBDD1D50}
DEFINE_GUID(GUID_SCFW_IPC,
    0x12B21D25, 0xAF9F, 0x4394, 0xBD, 0xD3, 0xD7, 0x55, 0xFB, 0xDD, 0x1D, 0x50);

WDF_EXTERN_C_START
#ifdef ALLOC_PRAGMA
EVT_WDF_DRIVER_DEVICE_ADD EvtDriverDeviceAdd;
#pragma alloc_text (PAGE, EvtDriverDeviceAdd)
EVT_WDF_OBJECT_CONTEXT_CLEANUP EvtCleanupCallback;
#pragma alloc_text (PAGE, EvtCleanupCallback)
EVT_WDF_DEVICE_D0_ENTRY EvtDeviceD0Entry;
#pragma alloc_text (PAGE, EvtDeviceD0Entry)
EVT_WDF_DEVICE_D0_EXIT EvtDeviceD0Exit; // NONPAGED
EVT_WDF_DEVICE_PREPARE_HARDWARE EvtDevicePrepareHardware;
#pragma alloc_text (PAGE, EvtDevicePrepareHardware)
EVT_WDF_DEVICE_RELEASE_HARDWARE EvtDeviceReleaseHardware;
#pragma alloc_text (PAGE, EvtDeviceReleaseHardware)
EVT_SPB_CONTROLLER_WRITE EvtSpbIoWrite; // NONPAGED
EVT_SPB_CONTROLLER_READ EvtSpbIoRead;
#pragma alloc_text (PAGE, EvtSpbIoRead)
EVT_SPB_CONTROLLER_SEQUENCE EvtSpbIoSequence; // NONPAGED
EVT_WDF_TIMER EvtTimerFunc; // NONPAGED
EVT_WDF_TIMER EvtStartFunc; // NONPAGED
VOID scfw_initiate_ipc_reset(MU_Type*); // NONPAGED
BOOLEAN scfw_isr_call(WDFINTERRUPT, ULONG, void*, void*); // NONPAGED
VOID scfw_dpc_call(WDFINTERRUPT, WDFOBJECT, void*, void*); // NONPAGED
NTSTATUS DriverEntry(PDRIVER_OBJECT, PUNICODE_STRING);
#pragma alloc_text (INIT, DriverEntry)
#endif
WDF_EXTERN_C_END

extern "C" DRIVER_INITIALIZE DriverEntry;

VOID EvtCleanupCallback(WDFOBJECT object)
{
    PDRIVER_OBJECT driver_object = WdfDriverWdmGetDriverObject((WDFDRIVER)object);
    IMXSCFW_LOG_TRACE("Driver end");
    WPP_CLEANUP(driver_object);
}

VOID scfw_initiate_ipc_reset(MU_Type* base) {
    (void)base->RR[0];
    (void)base->RR[1];
    (void)base->RR[2];
    (void)base->RR[3];
    base->TR[0] = 0U;
    base->TR[1] = 0U;
    base->TR[2] = 0U;
    base->TR[3] = 0U;
    MU_TriggerInterrupts(base, kMU_GenInt3InterruptTrigger);
}

VOID EvtTimerFunc(WDFTIMER timeout_timer)
{
    NTSTATUS result = STATUS_SUCCESS;
    WDFDEVICE device;
    scfw_device_t device_data;
    scfw_interrupt_t interrupt_data;
    SPBREQUEST request = NULL;
    scfw_ipc_states_t state = scfw_ipc_reset;
    MU_Type* base = NULL;

    IMXSCFW_LOG_TRACE("Enter");
    device = (WDFDEVICE)WdfTimerGetParentObject(timeout_timer);
    do {
        if (device == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("WdfTimerGetParentObject() retrieved NULL");
            break;
        }
        device_data = get_device_user_container(device);
        if (device_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_device_user_container() retrieved NULL");
            break;
        }
        interrupt_data = get_isr_user_container(device_data->interrupt);
        if (interrupt_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_isr_user_container() retrieved NULL");
            break;
        }
        base = device_data->base;
        if (base == NULL) {
            break;
        }
        WdfSpinLockAcquire(device_data->lock);
        IMXSCFW_LOG_TRACE("LockAcq state = %d\n", interrupt_data->state);
        WdfInterruptAcquireLock(device_data->interrupt);
        MU_DisableInterrupts(base, kMU_Tx0EmptyInterruptEnable |
            kMU_Tx1EmptyInterruptEnable |
            kMU_Tx2EmptyInterruptEnable |
            kMU_Tx3EmptyInterruptEnable |
            kMU_Rx0FullInterruptEnable |
            kMU_Rx1FullInterruptEnable |
            kMU_Rx2FullInterruptEnable |
            kMU_Rx3FullInterruptEnable);
        _DataSynchronizationBarrier();
        if ((interrupt_data->state != scfw_ipc_reset) && (interrupt_data->state != scfw_ipc_done) &&
            (interrupt_data->state != scfw_ipc_rx_dpc) && (interrupt_data->state != scfw_ipc_txonly_dpc)) {
            interrupt_data->state = scfw_ipc_reset;
            if (interrupt_data->tx_next > 0U) {
                scfw_initiate_ipc_reset(base);
            }
            request = interrupt_data->request;
            if (request != NULL) {
                result = STATUS_IO_TIMEOUT;
                state = scfw_ipc_done;
                interrupt_data->request = NULL;
                interrupt_data->tx_next = 0U;
                interrupt_data->tx_buffer.body.size = 0U;
                interrupt_data->rx_next = 0U;
                interrupt_data->rx_buffer.body.size = 0U;
                interrupt_data->rx_mdl = NULL;
            }
        }
        WdfInterruptReleaseLock(device_data->interrupt);
        IMXSCFW_LOG_TRACE("LockRel\n");
        WdfSpinLockRelease(device_data->lock);
        if (state == scfw_ipc_done) {
            SpbRequestComplete(request, result);
        }
    } while (FALSE);
    IMXSCFW_LOG_TRACE("Exit (result = %!STATUS!)", result);
}

VOID EvtStartFunc(WDFTIMER start_timer)
{
    NTSTATUS result = STATUS_SUCCESS;
    WDFDEVICE device;
    scfw_device_t device_data;
    scfw_interrupt_t interrupt_data;
    SPBREQUEST request = NULL;
    scfw_ipc_states_t state = scfw_ipc_reset;
    MU_Type* base = NULL;

    IMXSCFW_LOG_TRACE("Enter");
    device = (WDFDEVICE)WdfTimerGetParentObject(start_timer);
    do {
        if (device == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("WdfTimerGetParentObject() retrieved NULL");
            break;
        }
        device_data = get_device_user_container(device);
        if (device_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_device_user_container() retrieved NULL");
            break;
        }
        interrupt_data = get_isr_user_container(device_data->interrupt);
        if (interrupt_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_isr_user_container() retrieved NULL");
            break;
        }
        base = device_data->base;
        if (base == NULL) {
            break;
        }
        WdfSpinLockAcquire(device_data->lock);
        IMXSCFW_LOG_TRACE("LockAcq state = %d\n", interrupt_data->state);
        if (((interrupt_data->state == scfw_ipc_tx) || (interrupt_data->state == scfw_ipc_txonly)) &&
            (interrupt_data->tx_next == 0U)) {
            if (MU_GetAcceptance(base) & (UINT32)kMU_GenInt3InterruptTrigger) {
                MU_EnableInterrupts(base, (UINT32)kMU_Tx0EmptyInterruptEnable);
            }
            else {
                WdfTimerStop(device_data->timeout_timer, FALSE);
                interrupt_data->state = scfw_ipc_reset;
                request = interrupt_data->request;
                if (request != NULL) {
                    state = scfw_ipc_done;
                    IMXSCFW_LOG_ERROR("IPC reset wasn't performed on-time.");
                }
                else {
                    IMXSCFW_LOG_WARNING("Transfer delayed start called without request.");
                }
                scfw_initiate_ipc_reset(device_data->base);
                result = STATUS_UNSUCCESSFUL;
            }
        }
        IMXSCFW_LOG_TRACE("LockRel\n");
        WdfSpinLockRelease(device_data->lock);
        if (state == scfw_ipc_done) {
            SpbRequestComplete(interrupt_data->request, result);
        }
    } while (FALSE);
    IMXSCFW_LOG_TRACE("Exit (result = %!STATUS!)", result);
}

BOOLEAN scfw_isr_call(WDFINTERRUPT /* unused */, ULONG /* unused */, void* pint, void* pdev)
{
    NTSTATUS status = STATUS_SUCCESS;
    scfw_device_t device_data = (scfw_device_t)pdev;
    scfw_interrupt_t interrupt_data = (scfw_interrupt_t)pint;
    MU_Type* base = device_data->base;
    UINT32 cr_dis = (UINT32)(~(UINT32)0U);
    UINT32 cr_ena = 0U;
    BOOLEAN result = FALSE;
    IMXSCFW_LOG_TRACE("Enter state %d\n", interrupt_data->state);
    if (base == NULL) {
        return result;
    }
    if (interrupt_data->state == scfw_ipc_reset) {
        cr_dis &= (UINT32)(~(UINT32)(MU_CR_GIEn_MASK | MU_CR_TIEn_MASK | MU_CR_RIEn_MASK));
    }
    if ((interrupt_data->state == scfw_ipc_tx) ||
        (interrupt_data->state == scfw_ipc_txonly)) {
        size_t response_length = interrupt_data->tx_buffer.body.size;
        UINT32 register_index = 0U;
        size_t message_index;
        for (message_index = interrupt_data->tx_next;
            message_index < response_length;
            message_index++) {
            register_index = (UINT32)(message_index % 4);
            status = MU_SendMsg(base, register_index,
                interrupt_data->tx_buffer.raw32[message_index]);
            if (status == STATUS_DEVICE_BUSY) {
                IMXSCFW_LOG_TRACE("message_index=%lld response_length=%lld\n", message_index, response_length);
                break;
            }
        }
        if (message_index == response_length) {
            cr_dis &= (UINT32)(~(UINT32)MU_CR_TIEn_MASK);
            if (interrupt_data->state == scfw_ipc_txonly) {
                interrupt_data->state = scfw_ipc_txonly_dpc;
                result = TRUE;
                cr_dis &= (UINT32)(~(UINT32)MU_CR_RIEn_MASK);
            }
            else {
                interrupt_data->state = scfw_ipc_rx;
                IMXSCFW_LOG_TRACE("message_index == response_length; scfw_ipc_tx => scfw_ipc_rx\n");
            }
        }
        else {
            UINT32 tx_interrupt_enable = ((UINT32)kMU_Tx0EmptyInterruptEnable) >> register_index;
            cr_ena |= tx_interrupt_enable;
            cr_dis &= (UINT32)(~(UINT32)(MU_CR_TIEn_MASK | MU_CR_RIEn_MASK));
            cr_dis |= tx_interrupt_enable;
        }
        interrupt_data->tx_next = message_index;
    }
    IMXSCFW_LOG_TRACE("state %d\n", interrupt_data->state);
    if (interrupt_data->state == scfw_ipc_rx) {
        size_t response_length = interrupt_data->rx_buffer.body.size;
        UINT32 register_index = 0U;
        size_t message_index;
        for (message_index = interrupt_data->rx_next;
            message_index < response_length;
            message_index++) {
            register_index = (UINT32)(message_index % 4);
            status = MU_ReceiveMsg(base, register_index,
                &interrupt_data->rx_buffer.raw32[message_index]);
            if (status == STATUS_DEVICE_BUSY) {
                break;
            }
            else {
                if (message_index == 0U) {
                    response_length = interrupt_data->rx_buffer.body.size;
                    if (response_length > SCFW_RPC_MSG_WLEN * 4U) {
                        response_length = SCFW_RPC_MSG_WLEN * 4U;
                        IMXSCFW_LOG_WARNING("response_length=%lld > size of rx_buffer\n", response_length);
                    }
                }
            }
        }
        if (message_index == response_length) {
            interrupt_data->state = scfw_ipc_rx_dpc;
            result = TRUE;
            cr_dis &= (UINT32)(~(UINT32)(MU_CR_RIEn_MASK | MU_CR_TIEn_MASK));
            IMXSCFW_LOG_TRACE("message_index == response_length; scfw_ipc_rx => scfw_ipc_rx_dpc\n");
        }
        else {
            UINT32 rx_interrupt_enable = ((UINT32)kMU_Rx0FullInterruptEnable) >> register_index;
            cr_ena |= rx_interrupt_enable;
            cr_dis &= (UINT32)(~(UINT32)(MU_CR_TIEn_MASK | MU_CR_RIEn_MASK));
            cr_dis |= rx_interrupt_enable;
        }
        interrupt_data->rx_next = message_index;
    }
    if ((cr_dis != (UINT32)(~(UINT32)0U)) || (cr_ena != 0U)) {
        _DataSynchronizationBarrier();
        UINT32 reg = base->CR;
        reg &= (UINT32)~((UINT32)(MU_CR_GIRn_MASK | MU_CR_NMI_MASK));
        reg &= cr_dis;
        reg |= cr_ena;
        base->CR = reg;
    }
    IMXSCFW_LOG_TRACE("Exit state=%d cr_dis=0x%x cr_ena=0x%x", interrupt_data->state, cr_dis, cr_ena);
    return result;
}

VOID scfw_dpc_call(WDFINTERRUPT /* unused */, WDFOBJECT /* unused */, void* pint, void* pdev)
{
    NTSTATUS result = STATUS_SUCCESS;
    scfw_device_t device_data = (scfw_device_t)pdev;
    scfw_interrupt_t interrupt_data = (scfw_interrupt_t)pint;
    scfw_ipc_states_t state;
    SPBREQUEST request = NULL;
    size_t response_length;
    IMXSCFW_LOG_TRACE("Entr\n");
    WdfSpinLockAcquire(device_data->lock);
    state = interrupt_data->state;
    IMXSCFW_LOG_TRACE("LockAcq state=%d\n", state);
    if ((state == scfw_ipc_txonly_dpc) ||
        (state == scfw_ipc_rx_dpc)) {
        WdfTimerStop(device_data->timeout_timer, FALSE);
    }
    response_length = interrupt_data->rx_buffer.body.size;
    if ((state == scfw_ipc_rx_dpc) &&
        (interrupt_data->rx_next == response_length)) {
        size_t rmdl_offset = interrupt_data->rmdl_offset;
        size_t body_length = interrupt_data->rbuf_offset;
        for (PMDL mdl = interrupt_data->rx_mdl; (mdl != nullptr) && (body_length < response_length * 4U); mdl = mdl->Next) {
            PUCHAR body = &(((PUCHAR)(mdl->MappedSystemVa))[rmdl_offset]);
            size_t chunk_length = MmGetMdlByteCount(mdl);
            chunk_length -= rmdl_offset;
            if (body_length + chunk_length > response_length * 4U) {
                chunk_length = (response_length * 4U) - body_length;
            }
            RtlCopyMemory((void*)body, (void*)&(interrupt_data->rx_buffer.raw8[body_length]), chunk_length);
            body_length += chunk_length;
            rmdl_offset = 0U;
        }
        IMXSCFW_LOG_TRACE("MDL filed from %lld to %lld\n", rmdl_offset, body_length);
    }
    else {
        if ((state == scfw_ipc_txonly_dpc) &&
            (interrupt_data->tx_next == interrupt_data->tx_buffer.body.size)) {
            response_length = 0u;
        }
        else {
            if ((state == scfw_ipc_txonly_dpc) ||
                (state == scfw_ipc_rx_dpc)) {
                result = STATUS_INVALID_PARAMETER;
                scfw_initiate_ipc_reset(device_data->base);
                IMXSCFW_LOG_ERROR("Wrong length of received message.");
            }
        }
    }
    if ((state == scfw_ipc_txonly_dpc) ||
        (state == scfw_ipc_rx_dpc)) {
        request = interrupt_data->request;
        interrupt_data->request = NULL;
        interrupt_data->tx_next = 0U;
        interrupt_data->rx_next = 0U;
        interrupt_data->rx_mdl = NULL;
        interrupt_data->rmdl_offset = 0U;
        interrupt_data->rbuf_offset = 0U;
        interrupt_data->rx_buffer.body.size = 0U;
        if (NT_SUCCESS(result)) {
            interrupt_data->state = scfw_ipc_done;
        }
        else {
            interrupt_data->state = scfw_ipc_reset;
        }
        if (request != NULL) {
            if (NT_SUCCESS(result)) {
                WdfRequestSetInformation(request, (ULONG_PTR)response_length);
            }
            state = scfw_ipc_done;
        }
    }
    if (request == NULL) {
        IMXSCFW_LOG_TRACE("lockRel state=%d request==NULL (result = %!STATUS!)", interrupt_data->state, result);
    }
    else {
        IMXSCFW_LOG_TRACE("lockRel state=%d (request result = %!STATUS!)", interrupt_data->state, result);
    }
    WdfSpinLockRelease(device_data->lock);
    if (state == scfw_ipc_done) {
        SpbRequestComplete(request, result);
    }
    IMXSCFW_LOG_TRACE("Exit local_state=%d global_state=%d (request result = %!STATUS!)", state, interrupt_data->state, result);
}

VOID EvtSpbIoSequence(WDFDEVICE device, SPBTARGET /* unused */, SPBREQUEST request, ULONG transfer_count)
{
    NTSTATUS result = STATUS_SUCCESS;
    ULONG_PTR response_length = 1UL;
    size_t body_length;
    SPB_TRANSFER_DESCRIPTOR tx_descriptor;
    SPB_TRANSFER_DESCRIPTOR rx_descriptor;
    PMDL tx_base_mdl;
    PMDL rx_base_mdl;
    ULONG delay_in_us;
    scfw_device_t device_data;
    scfw_interrupt_t interrupt_data = NULL;
    MU_Type* base = NULL;
    BOOLEAN reset = FALSE;

    IMXSCFW_LOG_TRACE("Enter");
    do {
        if (transfer_count != 2) {
            result = STATUS_INVALID_PARAMETER;
            break;
        }
        SPB_TRANSFER_DESCRIPTOR_INIT(&tx_descriptor);
        SPB_TRANSFER_DESCRIPTOR_INIT(&rx_descriptor);
        SpbRequestGetTransferParameters(request, 0, &tx_descriptor, &tx_base_mdl);
        SpbRequestGetTransferParameters(request, 1, &rx_descriptor, &rx_base_mdl);
        if ((tx_descriptor.TransferLength < 4) || (tx_descriptor.TransferLength > SCFW_RPC_MSG_WLEN * 4U) ||
            (rx_descriptor.TransferLength < 4) || (rx_descriptor.TransferLength > SCFW_RPC_MSG_WLEN * 4U)) {
            result = STATUS_INVALID_PARAMETER;
            break;
        }
        if (tx_descriptor.DelayInUs != rx_descriptor.DelayInUs) {
            result = STATUS_INVALID_PARAMETER;
            break;
        }
        delay_in_us = rx_descriptor.DelayInUs;
        if (delay_in_us == 0UL) {
            delay_in_us = 900UL;
        }
        device_data = get_device_user_container(device);
        if (device_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_device_user_container() retrieved NULL");
            break;
        }
        interrupt_data = get_isr_user_container(device_data->interrupt);
        if (interrupt_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_isr_user_container() retrieved NULL");
            break;
        }
        interrupt_data->tx_next = 0U;
        interrupt_data->rx_next = 0U;
        interrupt_data->rx_mdl = rx_base_mdl;
        interrupt_data->rmdl_offset = 0U;
        interrupt_data->rbuf_offset = 0U;
        interrupt_data->rx_buffer.body.size = 1U;
        base = device_data->base;
        if ((MU_GetAcceptance(base) & (UINT32)kMU_GenInt3InterruptTrigger) != 0U) {
            reset = TRUE;
            interrupt_data->state = scfw_ipc_reset;
            IMXSCFW_LOG_WARNING("IPC in reset");
        }
        else {
            if ((MU_GetStatusFlags(base) & (UINT32)MU_SR_RFn_MASK) != 0U) {
                reset = TRUE;
                scfw_initiate_ipc_reset(base);
                interrupt_data->state = scfw_ipc_reset;
                IMXSCFW_LOG_WARNING("RX full during start - IPC reset");
            }
        }
        if (reset == TRUE) {
            if (delay_in_us > 50) {
                result = STATUS_DEVICE_BUSY;
            }
            else {
                result = STATUS_INTERNAL_ERROR;
            }
        }
        body_length = 0;
        for (PMDL mdl = tx_base_mdl; mdl != nullptr; mdl = mdl->Next) {
            size_t chunk_length = MmGetMdlByteCount(mdl);
            PUCHAR body = (PUCHAR)MmGetSystemAddressForMdlSafe(mdl,
                NormalPagePriority |
                MdlMappingNoExecute |
                MdlMappingNoWrite);
            if (body == NULL) {
                result = STATUS_INSUFFICIENT_RESOURCES;
                if (interrupt_data->tx_next > 0U) {
                    scfw_initiate_ipc_reset(base);
                }
                IMXSCFW_LOG_ERROR("MmGetSystemAddressForMdlSafe() retrieved NULL");
                break;
            }
            if (body_length + chunk_length > SCFW_RPC_MSG_WLEN * 4U) {
                result = STATUS_INVALID_PARAMETER;
                if (interrupt_data->tx_next > 0U) {
                    scfw_initiate_ipc_reset(base);
                }
                IMXSCFW_LOG_ERROR("MmGetMdlByteCount() > TransferLength");
                break;
            }
            for (size_t i = 0U, message_index = body_length / 4;
                (i < chunk_length) && (message_index < response_length);
                i++, message_index = (++body_length) / 4) {
                interrupt_data->tx_buffer.raw8[body_length] = body[i];
                if ((body_length + 1) % 4 == 0) {
                    if (message_index == 0U) {
                        response_length = interrupt_data->tx_buffer.body.size;
                        if (response_length > tx_descriptor.TransferLength / 4U) {
                            result = STATUS_INVALID_PARAMETER;
                            IMXSCFW_LOG_ERROR("Wrong length of send message.");
                            break;
                        }
                    }
                    if (NT_SUCCESS(result)) {
                        UINT32 register_index = (UINT32)(message_index % 4);
                        result = MU_SendMsg(base, register_index,
                            interrupt_data->tx_buffer.raw32[message_index]);
                        if (result == STATUS_DEVICE_BUSY) {
                            interrupt_data->tx_next = message_index;
                            interrupt_data->state = scfw_ipc_tx;
                            if (delay_in_us == 0U) {
                                result = STATUS_INVALID_PARAMETER;
                            }
                        }
                    }
                }
            }
        }
        if ((result != STATUS_DEVICE_BUSY) && (!NT_SUCCESS(result))) {
            break;
        }
        else {
            if (body_length / 4U < response_length) {
                result = STATUS_INSUFFICIENT_RESOURCES;
                if (interrupt_data->tx_next > 0U) {
                    scfw_initiate_ipc_reset(base);
                }
                IMXSCFW_LOG_ERROR("MmGetMdlByteCount() < TransferLength");
                break;
            }
        }
        body_length = 0;
        if (NT_SUCCESS(result)) {
            response_length = 1UL;
        }
        else {
            response_length = 0UL;
        }
        for (PMDL mdl = rx_base_mdl;
            (mdl != nullptr) && (NT_SUCCESS(result) || (result == STATUS_DEVICE_BUSY));
            mdl = mdl->Next) {
            size_t chunk_length = MmGetMdlByteCount(mdl);
            PUCHAR body = (PUCHAR)MmGetSystemAddressForMdlSafe(mdl,
                NormalPagePriority |
                MdlMappingNoExecute);
            if (body == NULL) {
                result = STATUS_INSUFFICIENT_RESOURCES;
                if (interrupt_data->rx_next > 0U) {
                    scfw_initiate_ipc_reset(base);
                }
                IMXSCFW_LOG_ERROR("MmGetSystemAddressForMdlSafe() retrieved NULL");
                break;
            }
            if (body_length + chunk_length > SCFW_RPC_MSG_WLEN * 4U) {
                result = STATUS_INVALID_PARAMETER;
                if (interrupt_data->rx_next > 0U) {
                    scfw_initiate_ipc_reset(base);
                }
                IMXSCFW_LOG_ERROR("MmGetMdlByteCount() > TransferLength");
                break;
            }
            for (size_t i = 0U, message_index = body_length / 4;
                NT_SUCCESS(result) && (i < chunk_length) && (message_index < response_length);
                i++, message_index = (++body_length) / 4) {
                if (body_length % 4 == 0) {
                    UINT32 register_index = (UINT32)(message_index % 4);
                    result = MU_ReceiveMsg(base, register_index,
                        &interrupt_data->rx_buffer.raw32[message_index]);
                    if ((result == STATUS_DEVICE_BUSY) && (delay_in_us == 0U)) {
                        result = STATUS_INVALID_PARAMETER;
                    }
                }
                if (NT_SUCCESS(result)) {
                    body[i] = interrupt_data->rx_buffer.raw8[body_length];
                    if (body_length == 0U) {
                        response_length = interrupt_data->rx_buffer.body.size;
                        if (response_length > SCFW_RPC_MSG_WLEN * 4U) {
                            result = STATUS_INVALID_PARAMETER;
                            scfw_initiate_ipc_reset(base);
                            IMXSCFW_LOG_ERROR("Wrong length of received message.");
                            break;
                        }
                    }
                }
                else {
                    interrupt_data->rx_next = message_index;
                    interrupt_data->rx_mdl = mdl;
                    interrupt_data->rmdl_offset = i;
                    interrupt_data->rbuf_offset = body_length;
                    interrupt_data->state = scfw_ipc_rx;
                    response_length = 0UL;
                }
            }
        }
        if ((result != STATUS_DEVICE_BUSY) && (!NT_SUCCESS(result))) {
            break;
        }
        else {
            if (body_length < response_length * 4) {
                result = STATUS_INSUFFICIENT_RESOURCES;
                if (interrupt_data->rx_next > 0U) {
                    scfw_initiate_ipc_reset(base);
                }
                IMXSCFW_LOG_ERROR("MmGetMdlByteCount() < IPC message");
                break;
            }
        }
        if (result == STATUS_DEVICE_BUSY) {
            interrupt_data->request = request;
            if (reset) {
                IMXSCFW_LOG_TRACE("before WdfTimerStart(start_timer)\n");
                if (WdfTimerStart(device_data->start_timer, WDF_REL_TIMEOUT_IN_US(50))) {
                    result = STATUS_INTERNAL_ERROR;
                    WdfTimerStop(device_data->start_timer, FALSE);
                    IMXSCFW_LOG_ERROR("WdfTimerStart(start_timer) == TRUE\n");
                    break;
                }
            }
            else {
                WdfInterruptAcquireLock(device_data->interrupt);
                if (interrupt_data->state == scfw_ipc_rx) {
                    MU_EnableInterrupts(device_data->base,
                        ((UINT32)kMU_Rx0FullInterruptEnable) >> (UINT32)(interrupt_data->rx_next % 4));
                }
                else {
                    MU_EnableInterrupts(device_data->base,
                        ((UINT32)kMU_Tx0EmptyInterruptEnable) >> (UINT32)(interrupt_data->tx_next % 4));
                }
                WdfInterruptReleaseLock(device_data->interrupt);
            }
            WdfSpinLockAcquire(device_data->lock);
            scfw_ipc_states_t state = interrupt_data->state;
            if ((state == scfw_ipc_tx) || (state == scfw_ipc_rx) || (state == scfw_ipc_rx_dpc)) {
                WDFTIMER timeout_timer = device_data->timeout_timer;
                if (WdfTimerStart(timeout_timer, WDF_REL_TIMEOUT_IN_US(delay_in_us))) {
                    WdfTimerStop(timeout_timer, TRUE);
                    IMXSCFW_LOG_TRACE("WdfTimerStart(timeout_timer) == TRUE\n");
                }
            }
            WdfSpinLockRelease(device_data->lock);
        }
    } while (FALSE);
    if (NT_SUCCESS(result)) {
        WdfRequestSetInformation(request, response_length);
    }
    if (result != STATUS_DEVICE_BUSY) {
        if (interrupt_data != NULL) {
            interrupt_data->state = scfw_ipc_done;
            interrupt_data->request = NULL;
        }
        SpbRequestComplete(request, result);
    }
    IMXSCFW_LOG_TRACE("Exit (result = %!STATUS!)", result);
}

VOID EvtSpbIoWrite(WDFDEVICE device, SPBTARGET /* unused */, SPBREQUEST request, size_t request_length)
{
    NTSTATUS result = STATUS_SUCCESS;
    ULONG_PTR response_length = 1UL;
    size_t body_length;
    SPB_TRANSFER_DESCRIPTOR tx_descriptor;
    PMDL tx_base_mdl;
    ULONG delay_in_us;
    scfw_device_t device_data;
    scfw_interrupt_t interrupt_data = NULL;
    MU_Type* base = NULL;
    BOOLEAN reset = FALSE;

    IMXSCFW_LOG_TRACE("Enter");
    SPB_TRANSFER_DESCRIPTOR_INIT(&tx_descriptor);
    SpbRequestGetTransferParameters(request, 0, &tx_descriptor, &tx_base_mdl);
    do {
        if ((request_length < 4) || (request_length > SCFW_RPC_MSG_WLEN * 4U)) {
            result = STATUS_INVALID_PARAMETER;
            break;
        }
        delay_in_us = tx_descriptor.DelayInUs;
        if (delay_in_us == 0UL) {
            delay_in_us = 900UL;
        }
        device_data = get_device_user_container(device);
        if (device_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_device_user_container() retrieved NULL");
            break;
        }
        interrupt_data = get_isr_user_container(device_data->interrupt);
        if (interrupt_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_isr_user_container() retrieved NULL");
            break;
        }
        interrupt_data->tx_next = 0U;
        interrupt_data->rx_next = 0U;
        interrupt_data->rx_mdl = NULL;
        interrupt_data->rmdl_offset = 0U;
        interrupt_data->rbuf_offset = 0U;
        interrupt_data->rx_buffer.body.size = 0U;
        base = device_data->base;
        if ((MU_GetAcceptance(base) & (UINT32)kMU_GenInt3InterruptTrigger) != 0U) {
            reset = TRUE;
            interrupt_data->state = scfw_ipc_reset;
            IMXSCFW_LOG_WARNING("IPC in reset");
        }
        else {
            if ((MU_GetStatusFlags(base) & (UINT32)MU_SR_RFn_MASK) != 0U) {
                reset = TRUE;
                scfw_initiate_ipc_reset(base);
                interrupt_data->state = scfw_ipc_reset;
                IMXSCFW_LOG_WARNING("RX full during start - IPC reset");
            }
        }
        if (reset == TRUE) {
            if (delay_in_us > 50) {
                result = STATUS_DEVICE_BUSY;
            }
            else {
                result = STATUS_INTERNAL_ERROR;
            }
        }
        body_length = 0;
        for (PMDL mdl = tx_base_mdl; mdl != nullptr; mdl = mdl->Next) {
            size_t chunk_length = MmGetMdlByteCount(mdl);
            PUCHAR body = (PUCHAR)MmGetSystemAddressForMdlSafe(mdl,
                NormalPagePriority |
                MdlMappingNoExecute |
                MdlMappingNoWrite);
            if (body == NULL) {
                result = STATUS_INSUFFICIENT_RESOURCES;
                if (interrupt_data->tx_next > 0U) {
                    scfw_initiate_ipc_reset(base);
                }
                IMXSCFW_LOG_ERROR("MmGetSystemAddressForMdlSafe() retrieved NULL");
                break;
            }
            if (body_length + chunk_length > SCFW_RPC_MSG_WLEN * 4U) {
                result = STATUS_INVALID_PARAMETER;
                if (interrupt_data->tx_next > 0U) {
                    scfw_initiate_ipc_reset(base);
                }
                IMXSCFW_LOG_ERROR("MmGetMdlByteCount() > TransferLength");
                break;
            }
            for (size_t i = 0U, message_index = body_length / 4;
                (i < chunk_length) && (message_index < response_length);
                i++, message_index = (++body_length) / 4) {
                interrupt_data->tx_buffer.raw8[body_length] = body[i];
                if ((body_length + 1) % 4 == 0) {
                    if (message_index == 0U) {
                        response_length = interrupt_data->tx_buffer.body.size;
                        if (response_length > request_length / 4U) {
                            result = STATUS_INVALID_PARAMETER;
                            IMXSCFW_LOG_ERROR("Wrong length of send message.");
                            break;
                        }
                    }
                    if (NT_SUCCESS(result)) {
                        UINT32 register_index = (UINT32)(message_index % 4);
                        result = MU_SendMsg(base, register_index,
                            interrupt_data->tx_buffer.raw32[message_index]);
                        if (result == STATUS_DEVICE_BUSY) {
                            interrupt_data->tx_next = message_index;
                            interrupt_data->state = scfw_ipc_txonly;
                            if (delay_in_us == 0U) {
                                result = STATUS_INVALID_PARAMETER;
                            }
                        }
                    }
                }
            }
        }
        if ((result != STATUS_DEVICE_BUSY) && (!NT_SUCCESS(result))) {
            break;
        }
        else {
            if (body_length / 4U < response_length) {
                result = STATUS_INSUFFICIENT_RESOURCES;
                if (interrupt_data->tx_next > 0U) {
                    scfw_initiate_ipc_reset(base);
                }
                IMXSCFW_LOG_ERROR("MmGetMdlByteCount() < TransferLength");
                break;
            }
        }
        if (result == STATUS_DEVICE_BUSY) {
            interrupt_data->request = request;
            if (reset) {
                IMXSCFW_LOG_TRACE("before WdfTimerStart(start_timer)\n");
                if (WdfTimerStart(device_data->start_timer, WDF_REL_TIMEOUT_IN_US(50))) {
                    result = STATUS_INTERNAL_ERROR;
                    WdfTimerStop(device_data->start_timer, FALSE);
                    IMXSCFW_LOG_ERROR("WdfTimerStart(start_timer) == TRUE\n");
                    break;
                }
            }
            else {
                WdfInterruptAcquireLock(device_data->interrupt);
                MU_EnableInterrupts(device_data->base,
                    ((UINT32)kMU_Tx0EmptyInterruptEnable) >> (UINT32)(interrupt_data->tx_next % 4));
                WdfInterruptReleaseLock(device_data->interrupt);
            }
            WdfSpinLockAcquire(device_data->lock);
            scfw_ipc_states_t state = interrupt_data->state;
            if ((state == scfw_ipc_txonly) || (state == scfw_ipc_txonly_dpc)) {
                WDFTIMER timeout_timer = device_data->timeout_timer;
                if (WdfTimerStart(timeout_timer, WDF_REL_TIMEOUT_IN_US(delay_in_us))) {
                    WdfTimerStop(timeout_timer, TRUE);
                    IMXSCFW_LOG_TRACE("WdfTimerStart(timeout_timer) == TRUE\n");
                }
            }
            WdfSpinLockRelease(device_data->lock);
        }
    } while (FALSE);
    if (NT_SUCCESS(result)) {
        WdfRequestSetInformation(request, 0UL);
    }
    if (result != STATUS_DEVICE_BUSY) {
        if (interrupt_data != NULL) {
            interrupt_data->state = scfw_ipc_done;
            interrupt_data->request = NULL;
        }
        SpbRequestComplete(request, result);
    }
    IMXSCFW_LOG_TRACE("Exit (result = %!STATUS!)", result);
}

VOID EvtSpbIoRead(WDFDEVICE /* unused */, SPBTARGET /* unused */, SPBREQUEST request, size_t /* unused */)
{
    IMXSCFW_LOG_TRACE("Enter");
    SpbRequestComplete(request, STATUS_INVALID_DEVICE_REQUEST);
    IMXSCFW_LOG_TRACE("Exit (result = INVALID DEVICE REQUEST)");
}

NTSTATUS EvtDeviceReleaseHardware(WDFDEVICE device, WDFCMRESLIST /* unused */)
{
    NTSTATUS result = STATUS_SUCCESS;
    scfw_device_t device_data;
    MU_Type* base = NULL;

    IMXSCFW_LOG_TRACE("Enter");
    device_data = get_device_user_container(device);
    do {
        if (device_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_device_user_container() retrieved NULL");
            break;
        }
        result = MU_FromDeviceReleaseHardware(device);
        base = device_data->base;
        device_data->base = NULL;
        MmUnmapIoSpace(base, device_data->registers_size);
    } while (FALSE);
    IMXSCFW_LOG_TRACE("Exit (result = %!STATUS!)", result);
    return result;
}

NTSTATUS EvtDeviceD0Exit(WDFDEVICE device, WDF_POWER_DEVICE_STATE /* unused */)
{
    NTSTATUS result = STATUS_SUCCESS;
    scfw_device_t device_data;
    scfw_interrupt_t interrupt_data;
    MU_Type* base = NULL;
    SPBREQUEST request = NULL;
    scfw_ipc_states_t state = scfw_ipc_reset;

    IMXSCFW_LOG_TRACE("Enter");
    device_data = get_device_user_container(device);
    do {
        if (device_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_device_user_container() retrieved NULL");
            break;
        }
        interrupt_data = get_isr_user_container(device_data->interrupt);
        if (interrupt_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_isr_user_container() retrieved NULL");
            break;
        }
        base = device_data->base;
        if (base == NULL) {
            break;
        }
        MU_DisableInterrupts(base, (UINT32)(kMU_Tx0EmptyInterruptEnable)
            | (UINT32)(kMU_Tx1EmptyInterruptEnable) | (UINT32)(kMU_Tx2EmptyInterruptEnable)
            | (UINT32)(kMU_Tx3EmptyInterruptEnable) | (UINT32)(kMU_Rx0FullInterruptEnable)
            | (UINT32)(kMU_Rx1FullInterruptEnable) | (UINT32)(kMU_Rx2FullInterruptEnable)
            | (UINT32)(kMU_Rx3FullInterruptEnable) | (UINT32)(kMU_GenInt0InterruptEnable)
            | (UINT32)(kMU_GenInt1InterruptEnable) | (UINT32)(kMU_GenInt2InterruptEnable)
            | (UINT32)(kMU_GenInt3InterruptEnable) | (UINT32)(kMU_GenInt0InterruptTrigger)
            | (UINT32)(kMU_GenInt1InterruptTrigger) | (UINT32)(kMU_GenInt2InterruptTrigger)
            | (UINT32)(kMU_GenInt3InterruptTrigger));
        _DataSynchronizationBarrier();
        WdfSpinLockAcquire(device_data->lock);
        IMXSCFW_LOG_TRACE("LockAcq state = %d\n", interrupt_data->state);
        if ((interrupt_data->state != scfw_ipc_reset) && (interrupt_data->state != scfw_ipc_done)) {
            interrupt_data->state = scfw_ipc_reset;
            if (interrupt_data->tx_next > 0U) {
                scfw_initiate_ipc_reset(base);
            }
            WdfTimerStop(device_data->timeout_timer, FALSE);
            WdfTimerStop(device_data->start_timer, FALSE);
            request = interrupt_data->request;
            if (request != NULL) {
                state = scfw_ipc_done;
                interrupt_data->request = NULL;
                interrupt_data->tx_next = 0U;
                interrupt_data->tx_buffer.body.size = 0U;
                interrupt_data->rx_next = 0U;
                interrupt_data->rx_buffer.body.size = 0U;
                interrupt_data->rx_mdl = NULL;
            }
        }
        IMXSCFW_LOG_TRACE("LockRel\n");
        WdfSpinLockRelease(device_data->lock);
        if (state == scfw_ipc_done) {
            SpbRequestComplete(request, STATUS_CANCELLED);
        }
    } while (FALSE);
    IMXSCFW_LOG_TRACE("Exit (result = %!STATUS!)", result);
    return result;
}

NTSTATUS EvtDevicePrepareHardware(WDFDEVICE device, WDFCMRESLIST /* unused */,
    WDFCMRESLIST translated_resources)
{
    NTSTATUS result = STATUS_SUCCESS;
    ULONG count = 0;
    scfw_device_t device_data;

    IMXSCFW_LOG_TRACE("Enter");
    count = WdfCmResourceListGetCount(translated_resources);
    device_data = get_device_user_container(device);
    do {
        if (device_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_device_user_container() retrieved NULL");
            break;
        }
        device_data->physical_address = { NULL };
        device_data->registers_size = 0UL;
        device_data->base = NULL;
        for (ULONG i = 0; i < count; i++) {
            PCM_PARTIAL_RESOURCE_DESCRIPTOR resource = WdfCmResourceListGetDescriptor(translated_resources, i);
            if (resource->Type == CmResourceTypeMemory) {
                device_data->physical_address = resource->u.Memory.Start;
                device_data->registers_size = resource->u.Memory.Length;
                device_data->base = (MU_Type*)MmMapIoSpaceEx(device_data->physical_address,
                    device_data->registers_size,
                    PAGE_NOCACHE | PAGE_READWRITE);
                break;
            }
        }
        if (device_data->base == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("Base address from ACPI is NULL");
            break;
        }
        result = MU_FromDevicePrepareHardware(device, device_data->base);
    } while (FALSE);
    IMXSCFW_LOG_TRACE("Exit (result = %!STATUS!)", result);
    return result;
}

NTSTATUS EvtDeviceD0Entry(WDFDEVICE /* unused */, WDF_POWER_DEVICE_STATE /* unused */)
{
    IMXSCFW_LOG_TRACE("Occured");
    return STATUS_SUCCESS;
}

NTSTATUS EvtDriverDeviceAdd(WDFDRIVER /* unused */, PWDFDEVICE_INIT device_init)
{
    NTSTATUS result;
    WDF_PNPPOWER_EVENT_CALLBACKS pnp_calls;
    WDFDEVICE device;
    WDF_OBJECT_ATTRIBUTES device_attributes;
    scfw_device_t device_data;
    WDF_DEVICE_STATE device_state;
    SPB_CONTROLLER_CONFIG spb_config;
    WDFINTERRUPT interrupt;
    WDF_INTERRUPT_CONFIG interrupt_config;
    WDF_OBJECT_ATTRIBUTES interrupt_attributes;
    scfw_interrupt_t interrupt_data;
    WDF_TIMER_CONFIG timer_config;
    WDF_TIMER_CONFIG start_config;
    WDF_OBJECT_ATTRIBUTES timer_attributes;
    WDF_OBJECT_ATTRIBUTES start_attributes;
    WDF_OBJECT_ATTRIBUTES lock_attributes;
    mu_isr_dpc_calls_struct calls = { 0 };

    IMXSCFW_LOG_TRACE("Enter");
    do {
        result = SpbDeviceInitConfig(device_init);
        if (!NT_SUCCESS(result)) {
            IMXSCFW_LOG_ERROR("SpbDeviceInitConfig() failed with result %!STATUS!)", result);
            break;
        }
        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnp_calls);
        pnp_calls.EvtDevicePrepareHardware = EvtDevicePrepareHardware;
        pnp_calls.EvtDeviceReleaseHardware = EvtDeviceReleaseHardware;
        pnp_calls.EvtDeviceD0Entry = EvtDeviceD0Entry;
        pnp_calls.EvtDeviceD0Exit = EvtDeviceD0Exit;
        WdfDeviceInitSetPnpPowerEventCallbacks(device_init, &pnp_calls);
        WdfDeviceInitSetExclusive(device_init, FALSE);
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&device_attributes, mu_device_data_struct_t);
        result = WdfDeviceCreate(&device_init, &device_attributes, &device);
        if (!NT_SUCCESS(result)) {
            IMXSCFW_LOG_ERROR("WdfDeviceCreate() failed with result %!STATUS!)", result);
            break;
        }
        device_data = get_device_user_container(device);
        if (device_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_device_user_container() retrieved NULL");
            break;
        }
        device_data->device = device;
        WDF_DEVICE_STATE_INIT(&device_state);
        device_state.NotDisableable = WdfFalse;
        WdfDeviceSetDeviceState(device, &device_state);
        SPB_CONTROLLER_CONFIG_INIT(&spb_config);
        spb_config.EvtSpbTargetConnect = NULL;
        spb_config.EvtSpbTargetDisconnect = NULL;
        spb_config.ControllerDispatchType = WdfIoQueueDispatchSequential;
        spb_config.PowerManaged = WdfTrue;
        spb_config.EvtSpbIoWrite = EvtSpbIoWrite;
        spb_config.EvtSpbIoRead = EvtSpbIoRead;
        spb_config.EvtSpbIoSequence = EvtSpbIoSequence;
        spb_config.EvtSpbControllerLock = NULL;
        spb_config.EvtSpbControllerUnlock = NULL;
        result = SpbDeviceInitialize(device, &spb_config);
        if (!NT_SUCCESS(result)) {
            IMXSCFW_LOG_ERROR("SpbDeviceInitialize() failed with result %!STATUS!)", result);
            break;
        }
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&interrupt_attributes,
            mu_interrupt_data_struct_t);
        WDF_INTERRUPT_CONFIG_INIT(&interrupt_config, MU_InterruptHandler,
            MU_DeferredProcedureCall);
        result = WdfInterruptCreate(device, &interrupt_config,
            &interrupt_attributes, &interrupt);
        if (!NT_SUCCESS(result)) {
            IMXSCFW_LOG_ERROR("WdfInterruptCreate() failed with result %!STATUS!)", result);
            break;
        }
        device_data->interrupt = interrupt;
        interrupt_data = get_isr_user_container(interrupt);
        if (interrupt_data == NULL) {
            result = STATUS_INSUFFICIENT_RESOURCES;
            IMXSCFW_LOG_ERROR("get_isr_user_container() retrieved NULL");
            break;
        }
        WDF_TIMER_CONFIG_INIT(&timer_config, EvtTimerFunc);
        WDF_OBJECT_ATTRIBUTES_INIT(&timer_attributes);
        timer_attributes.ParentObject = device;
        result = WdfTimerCreate(&timer_config, &timer_attributes, &(device_data->timeout_timer));
        if (!NT_SUCCESS(result)) {
            IMXSCFW_LOG_ERROR("WdfTimerCreate() failed with result %!STATUS!)", result);
            break;
        }
        WDF_TIMER_CONFIG_INIT(&start_config, EvtStartFunc);
        WDF_OBJECT_ATTRIBUTES_INIT(&start_attributes);
        start_attributes.ParentObject = device;
        result = WdfTimerCreate(&start_config, &start_attributes, &(device_data->start_timer));
        if (!NT_SUCCESS(result)) {
            IMXSCFW_LOG_ERROR("WdfTimerCreate() failed with result %!STATUS!)", result);
            break;
        }
        WDF_OBJECT_ATTRIBUTES_INIT(&lock_attributes);
        lock_attributes.ParentObject = device;
        WdfSpinLockCreate(&lock_attributes, &device_data->lock);
        calls.tx_empty_isr[0] = scfw_isr_call;
        calls.tx_empty_isr[1] = scfw_isr_call;
        calls.tx_empty_isr[2] = scfw_isr_call;
        calls.tx_empty_isr[3] = scfw_isr_call;
        calls.rx_full_isr[0] = scfw_isr_call;
        calls.rx_full_isr[1] = scfw_isr_call;
        calls.rx_full_isr[2] = scfw_isr_call;
        calls.rx_full_isr[3] = scfw_isr_call;
        calls.tx_empty_dpc[0] = scfw_dpc_call;
        calls.tx_empty_dpc[1] = scfw_dpc_call;
        calls.tx_empty_dpc[2] = scfw_dpc_call;
        calls.tx_empty_dpc[3] = scfw_dpc_call;
        calls.rx_full_dpc[0] = scfw_dpc_call;
        calls.rx_full_dpc[1] = scfw_dpc_call;
        calls.rx_full_dpc[2] = scfw_dpc_call;
        calls.rx_full_dpc[3] = scfw_dpc_call;
        result = MU_FromAddDevice(device, interrupt, &calls, NULL, NULL);
    } while (false);
    IMXSCFW_LOG_TRACE("Exit (result = %!STATUS!)", result);
    return result;
}

NTSTATUS DriverEntry(PDRIVER_OBJECT driver_object, PUNICODE_STRING registry_path)
{
    NTSTATUS result;
    WDF_OBJECT_ATTRIBUTES driver_attributes;
    WDF_DRIVER_CONFIG driver_config;

    // Initialize logging
    WPP_INIT_TRACING(driver_object, registry_path);
    RECORDER_CONFIGURE_PARAMS wpp_configure_params;
    RECORDER_CONFIGURE_PARAMS_INIT(&wpp_configure_params);
    WppRecorderConfigure(&wpp_configure_params);
#if DBG
    WPP_RECORDER_LEVEL_FILTER(IMXSCFW_TRACING_DEFAULT) = TRUE;
#endif // DBG

    WDF_DRIVER_CONFIG_INIT(&driver_config, EvtDriverDeviceAdd);
    driver_config.DriverPoolTag = IMX_SCFW_POOL_TAG;

    // Initialize attributes and a context area for the driver object.
    WDF_OBJECT_ATTRIBUTES_INIT(&driver_attributes);
    driver_attributes.SynchronizationScope = WdfSynchronizationScopeNone;
    driver_attributes.EvtCleanupCallback = EvtCleanupCallback;

    // Create the driver object
    result = WdfDriverCreate(driver_object,
        registry_path,
        &driver_attributes,
        &driver_config,
        WDF_NO_HANDLE);

    if (!NT_SUCCESS(result)) {
        IMXSCFW_LOG_ERROR("WdfDriverCreate() failed with result %!STATUS!)", result);
    }

    IMXSCFW_LOG_TRACE("Exit (result = %!STATUS!)", result);

    if (!NT_SUCCESS(result)) {
        WPP_CLEANUP(driver_object);
    }
    return result;
}