/*
** ###################################################################
**
**     Copyright (c) 2016 Freescale Semiconductor, Inc.
**     Copyright 2017-2022 NXP
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** ###################################################################
*/

/*==========================================================================*/
/*!
 * @file
 *
 * Implementation of the IPC functions using MUs (client side).
 */
/*==========================================================================*/

/* Includes */

#include <svc/rpc.h>

/* Local Defines */

/* Local Types */

/* Local Functions */

/* Local Variables */

/*----------------------------------------------------------------------*/
/* Converts error code to readable string description                   */
/*----------------------------------------------------------------------*/
const char *sc_err_to_string(sc_err_t err)
{
    static const char* names[] = { "Success",
                                   "Incompatible API version",
                                   "Configuration error",
                                   "Bad parameter",
                                   "Permission error (no access)",
                                   "Permission error (locked)",
                                   "Unavailable (out of resources)",
                                   "Not found",
                                   "No power",
                                   "Generic IPC error",
                                   "Resource is currently busy/active",
                                   "General I/O failure",
                                   "Unknown error code"};
    if ((err < SC_ERR_NONE) || (err >SC_ERR_LAST)) {
        err = SC_ERR_LAST;
    }
    return names[err];
}

/*----------------------------------------------------------------------*/
/* RPC command/response                                                 */
/*----------------------------------------------------------------------*/
void sc_call_rpc(sc_ipc_t ipc, sc_rpc_msg_t *msg, sc_bool_t no_resp)
{
    NTSTATUS status = STATUS_SUCCESS;
    IO_STATUS_BLOCK ipc_status = { 0 };
    PIRP irp = NULL;
    LARGE_INTEGER timeout;

    do {
        if ((ipc == NULL) || (msg == NULL)) {
            status = STATUS_INVALID_PARAMETER;
            break;
        }
        timeout.QuadPart = -10 * 1000LL * ipc->ipc_timeout_us;
        KeClearEvent(&ipc->ipc_event);
        if (no_resp == SC_FALSE) {
            PIO_STACK_LOCATION stack_location = NULL;
            SPB_TRANSFER_LIST_AND_ENTRIES(2)  tx_rx_list;
            SPB_TRANSFER_LIST_INIT(&(tx_rx_list.List), 2);
            tx_rx_list.List.Transfers[0] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(SpbTransferDirectionToDevice, ipc->ipc_timeout_us,
                                                                               msg, 4 * SCFW_RPC_MSG_WLEN);
            tx_rx_list.List.Transfers[1] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(SpbTransferDirectionFromDevice, ipc->ipc_timeout_us,
                                                                               &ipc->rx_buffer[0], 4 * SCFW_RPC_MSG_WLEN);
            irp = IoBuildDeviceIoControlRequest(IOCTL_SPB_EXECUTE_SEQUENCE, ipc->ipc_device, &tx_rx_list, sizeof(tx_rx_list), NULL, 0,
                                                FALSE, &ipc->ipc_event, &ipc_status);
            if (irp == NULL) {
                status = STATUS_INSUFFICIENT_RESOURCES;
                break;
            }
            irp->RequestorMode = KernelMode;
            stack_location = IoGetNextIrpStackLocation(irp);
            if (stack_location == NULL) {
                IoFreeIrp(irp);
                status = STATUS_INVALID_DEVICE_REQUEST;
                break;
            }
            stack_location->FileObject = ipc->ipc_file;
        }
        else {
            LARGE_INTEGER starting_offset;
            starting_offset.QuadPart = (LONGLONG)0;
            irp = IoBuildSynchronousFsdRequest(IRP_MJ_WRITE, ipc->ipc_device, msg, 4 * SCFW_RPC_MSG_WLEN, &starting_offset,
                                               &ipc->ipc_event, &ipc_status);
            if (irp == NULL) {
                status = STATUS_INSUFFICIENT_RESOURCES;
                break;
            }
            irp->RequestorMode = KernelMode;
            PIO_STACK_LOCATION stack_location = IoGetNextIrpStackLocation(irp);
            if (stack_location == NULL) {
                IoFreeIrp(irp);
                status = STATUS_INVALID_DEVICE_REQUEST;
                break;
            }
            stack_location->FileObject = ipc->ipc_file;
        }
        status = IoCallDriver(ipc->ipc_device, irp);
        if (STATUS_PENDING == status)
        {
            status = KeWaitForSingleObject(&ipc->ipc_event, Executive, KernelMode, FALSE, &timeout);
            if (status == STATUS_TIMEOUT) {
                IoCancelIrp(irp);
                status = KeWaitForSingleObject(&ipc->ipc_event, Executive, KernelMode, FALSE, NULL);
            }
        }
    } while (FALSE);
    if ((irp != NULL) && (!NT_SUCCESS(status))) {
        IoFreeIrp(irp);
    }
    if ((no_resp == SC_FALSE) && (msg != NULL)) {
        if (status == STATUS_SUCCESS) {
            status = ipc_status.Status;
        }
        switch (status) {
        case STATUS_SUCCESS:
            RtlCopyMemory(msg, &ipc->rx_buffer[0], SCFW_RPC_MSG_WLEN * 4);
            break;
        case STATUS_IO_TIMEOUT:
            RPC_R8(msg) = SC_ERR_BUSY;
            break;
        case STATUS_INVALID_PARAMETER:
            RPC_R8(msg) = SC_ERR_PARM;
            break;
        case STATUS_INSUFFICIENT_RESOURCES:
            RPC_R8(msg) = SC_ERR_UNAVAILABLE;
            break;
        case STATUS_INTERNAL_ERROR:
            RPC_R8(msg) = SC_ERR_CONFIG;
            break;
        default:
            RPC_R8(msg) = SC_ERR_IPC;
        }
    }
    return;
}

/*--------------------------------------------------------------------------*/
/* Open an IPC channel                                                      */
/*--------------------------------------------------------------------------*/
NTSTATUS sc_ipc_open(sc_ipc_t ipc, sc_ipc_id_t id)
{
    NTSTATUS result = STATUS_SUCCESS;
    DECLARE_UNICODE_STRING_SIZE(device_path, RESOURCE_HUB_PATH_SIZE);
    HANDLE ipc_target = NULL;
    OBJECT_ATTRIBUTES ipc_attributes;
    IO_STATUS_BLOCK ipc_status = { 0 };
    PFILE_OBJECT ipc_file = NULL;
    PDEVICE_OBJECT ipc_device = NULL;

    do {
        if ((ipc == NULL) || (id == NULL)) {
            result = STATUS_INVALID_PARAMETER;
            break;
        }
        result = RESOURCE_HUB_CREATE_PATH_FROM_ID(&device_path, id->ipc_id.LowPart, id->ipc_id.HighPart);
        if (!NT_SUCCESS(result)) {
            break;
        }
        InitializeObjectAttributes(&ipc_attributes, &device_path, OBJ_KERNEL_HANDLE, NULL, NULL);
        result = ZwOpenFile(&ipc_target, (GENERIC_READ | GENERIC_WRITE), &ipc_attributes, &ipc_status, 0, 0);
        if (!NT_SUCCESS(result)) {
            break;
        }
        result = ObReferenceObjectByHandle(ipc_target, (GENERIC_READ | GENERIC_WRITE), *IoFileObjectType, KernelMode, (PVOID*)&ipc_file, NULL);
        if (!NT_SUCCESS(result)) {
            break;
        }
        ipc_device = IoGetRelatedDeviceObject(ipc_file);
        if (ipc_device == NULL) {
            result = STATUS_NO_SUCH_DEVICE;
            break;
        }
        if (id->ipc_timeout_us == 0U) {
            ipc->ipc_timeout_us = SCFW_SYNCHRONOUS_TIMEOUT_US;
        }
        else {
            ipc->ipc_timeout_us = id->ipc_timeout_us;
        }
        KeInitializeEvent(&ipc->ipc_event, NotificationEvent, FALSE);
        ipc->ipc_device = ipc_device;
        ipc->ipc_file = ipc_file;
        ipc->ipc_target = ipc_target;
    } while (FALSE);
    if (!NT_SUCCESS(result)) {
        if (ipc_file != NULL) {
            ObDereferenceObject(ipc_file);
        }
        if (ipc_target != NULL) {
            ZwClose(ipc_target);
        }
    }
    return result;
}

/*--------------------------------------------------------------------------*/
/* Close an IPC channel                                                     */
/*--------------------------------------------------------------------------*/
void sc_ipc_close(sc_ipc_t ipc)
{
    if (ipc != NULL)
    {
        if (ipc->ipc_file != NULL) {
            ObDereferenceObject(ipc->ipc_file);
        }
        if (ipc->ipc_target != NULL) {
            ZwClose(ipc->ipc_target);
        }
    }
}
