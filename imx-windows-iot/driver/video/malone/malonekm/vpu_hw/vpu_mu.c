/*
 * Copyright 2023 NXP
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

#include "Trace.h"
#if (MALONE_TRACE == MALONE_TRACE_WPP)
    #include "vpu_mu.tmh"
#endif

#include "device.h"
#include "vpu_dec/vdec.h"  /* vpu_api_event_handler function */

typedef struct {
    struct event_msg rpc_msg;
} rpc_entry_t;

/**
 * @brief Retrieves Device user data container.
 * Extracts Device user structure from MU device data structure.
 * @param[in] device device where is MU registered
 * @return pointer to isr user container
 */
mu_device_data_t get_device_MU_container(WDFDEVICE WdfDevice)
{
    MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr = DeviceGetContext(WdfDevice);

    return &deviceContextPtr->MUDeviceContext;
}

/**
 * @brief Retrieves ISR user data container.
 * Extracts ISR user structure from MU interrupt data structure.
 * @param[in] interrupt interrupt where is MU registered
 * @return pointer to isr user container
 */
mu_interrupt_data_t get_isr_MU_container(WDFINTERRUPT interrupt)
{
    MALONE_VPU_ISR_CONTEXT *interrupt_data = InterruptGetContext(interrupt);
    return &interrupt_data->MUInterruptContext;
}

/**
 * Send an interrupt to VPU core
 *
 * @param imx_mu_t * MU instance handler,
 * @param MSG_Type Type of a message
 * @param uint32_t Value to be sent
 *
 */
void mu_send_msg_to_FW(MU_Type *base, UINT32 type, UINT32 value)
{
    while (STATUS_DEVICE_BUSY == MU_SendMsg(base, MSG, value));
    while (STATUS_DEVICE_BUSY == MU_SendMsg(base, MSG_TYPE, type));

    return;
}

_Use_decl_annotations_
BOOLEAN mu_isr(WDFINTERRUPT interrupt, ULONG id, void *isr_container, void *device_container)
{
    MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr = DeviceGetContext(WdfInterruptGetDevice(interrupt));
    MALONE_MU_REGISTERS *registersMUPtr = deviceContextPtr->RegistersMUPtr;
    MALONE_VPU_ISR_CONTEXT *interruptContextPtr = InterruptGetContext(interrupt);
    QUEUE_ITEM item = { 0 };
    UNREFERENCED_PARAMETER(id);
    UNREFERENCED_PARAMETER(isr_container);
    UNREFERENCED_PARAMETER(device_container);

    DBG_RPC_METHOD_BEG();

    item.msg = MU_ReceiveMsgNonBlocking((MU_Type *)registersMUPtr, 0);
    DBG_RPC_PRINT_INFO("Received MU interrupt 0x%x. Pushing to circular queue.", item.msg);
    /* Driver Verifier : Bugcheck initiated with Error Code : 0x20005 Error Message : 'WdfSpinLockAcquire should only be called at IRQL <= APC_LEVEL.' */
    /* WdfSpinLockAcquire(&deviceContextPtr->rpc_mutex); */
    CircQueuePush(&interruptContextPtr->MsgQueue, &item);
    /* WdfSpinLockRelease(&deviceContextPtr->rpc_mutex); */
    DBG_RPC_METHOD_END();

    return TRUE;
}


_Use_decl_annotations_
VOID mu_dpc(WDFINTERRUPT interrupt, WDFOBJECT AssociatedWdfObject, void *isr_container, void *device_container)
{
    UNREFERENCED_PARAMETER(AssociatedWdfObject);
    UNREFERENCED_PARAMETER(isr_container);
    UNREFERENCED_PARAMETER(device_container);
    MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr = DeviceGetContext(WdfInterruptGetDevice(interrupt));
    MALONE_VPU_ISR_CONTEXT *interruptContextPtr = InterruptGetContext(interrupt);
    QUEUE_ITEM item = { 0 };
    int idx = 0;
    struct shared_addr *This = &deviceContextPtr->shared_mem;
    int retval = 0;
    DBG_RPC_METHOD_BEG();

    do {
        WdfInterruptAcquireLock(deviceContextPtr->MUInterrupt); /* Lock interrupt */
        retval = CircQueuePop(&interruptContextPtr->MsgQueue, &item);
        WdfInterruptReleaseLock(deviceContextPtr->MUInterrupt); /* Unlock interrupt */
        if (retval == -1) {
            DBG_PRINT_ERROR("\t\t\tPopping RPC message failed");
            break;
        }

        if (item.msg == 0xAA) { /* VPU Core Init done, set shared memory */
            DBG_PRINT_ERROR("\t\t\tIt is not expected to receive 0xAA here!!!");
        } else if (item.msg == 0x55) {
            DBG_PRINT_ERROR("\t\t\tIt is not expected to receive 0x55 here!!!");
        } else {
            while (rpc_MediaIPFW_Video_message_check(This) == API_MSG_AVAILABLE) {

                rpc_entry_t re;
                memset(&re, 0x0, sizeof(rpc_entry_t));
                rpc_receive_msg_buf(This, &re.rpc_msg);
                DBG_RPC_PRINT_INFO("player%u, msgnum:%u, msgid:0x%02x", re.rpc_msg.idx, re.rpc_msg.msgnum, re.rpc_msg.msgid);
                idx = re.rpc_msg.idx;
                DBG_RPC_PRINT_INFO(" I have a RPC message!");

                vpu_api_event_handler(deviceContextPtr, re.rpc_msg.idx, re.rpc_msg.msgid, re.rpc_msg.msgdata);

            }
            if (rpc_MediaIPFW_Video_message_check(This) == API_MSG_BUFFER_ERROR) {
                DBG_PRINT_ERROR("RPC API_MSG_BUFFER_ERROR!");
            }
        }
    } while (interruptContextPtr->MsgQueue.count != 0);

    DBG_RPC_METHOD_END();
}
