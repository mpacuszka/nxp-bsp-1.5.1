/*
 * Copyright 2022-2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
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
 *
 */

#include "precomp.h"

extern "C"
{

#ifndef RESHUB_USE_HELPER_ROUTINES
#define RESHUB_USE_HELPER_ROUTINES /* Needed for reshub.h */
#endif

#include <reshub.h>
#include <spb.h>
#include <gpio.h>
#include "comm.h"

#define I2C_TX_TRANSFER_COUNT 1
#define I2C_RX_TRANSFER_COUNT 2
#define I2C_REGISTER_ADDR_SIZE 1
#define I2C_BUFFER_SIZE (64 + I2C_REGISTER_ADDR_SIZE)
// Timeout in milliseconds for synchronous I2C reads/writes.
#define COMM_TIMEOUT_MS 900


#define PRINT_INFO(x, ...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, __VA_ARGS__)

static NTSTATUS comm_send_irp(iotarget_handles *tgt, ULONG ioctrl, PVOID in_buffer, ULONG in_buffer_length,
                             PVOID out_buffer, ULONG out_buffer_length, PIO_STATUS_BLOCK io_status)
{
    PIRP irp;
    PIO_STACK_LOCATION stack_loc;
    NTSTATUS status;
    LARGE_INTEGER timeout;

    timeout.QuadPart = -10 * 1000 * COMM_TIMEOUT_MS;
    RtlZeroMemory(io_status, sizeof(IO_STATUS_BLOCK));
    KeClearEvent(&tgt->m_event);

    irp = IoBuildDeviceIoControlRequest(ioctrl,
        tgt->m_iotarget_device_object_ptr,
        in_buffer,
        in_buffer_length,
        out_buffer,
        out_buffer_length,
        FALSE,
        &tgt->m_event,
        io_status);
    if (irp == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    irp->RequestorMode = KernelMode;
    stack_loc = IoGetNextIrpStackLocation(irp);
    if (stack_loc == NULL) {
        IoFreeIrp(irp);
        return STATUS_INVALID_DEVICE_REQUEST;
    }
    stack_loc->FileObject = tgt->m_iotarget_file_object_ptr;

    status = IoCallDriver(tgt->m_iotarget_device_object_ptr, irp);
    if (STATUS_PENDING == status)
    {
        NTSTATUS loc_status = KeWaitForSingleObject(&tgt->m_event,
            Executive, KernelMode, FALSE, &timeout);
        if (loc_status == STATUS_TIMEOUT)
        {
            PRINT_INFO("comm_send_irp: KeWaitForSingleObject() timeout:"
                "status-0x%lx\n",
                loc_status);
        }
    }
    if (!NT_SUCCESS(status)) {
        IoFreeIrp(irp);
    }
    else {
        status = io_status->Status;
    }
    return status;
}

NTSTATUS comm_initialize(LARGE_INTEGER connection_id, iotarget_handles *tgt, ACCESS_MASK desired_access)
{
    NTSTATUS status;

    DECLARE_UNICODE_STRING_SIZE(device_name, RESOURCE_HUB_PATH_SIZE);
    status = RESOURCE_HUB_CREATE_PATH_FROM_ID(&device_name,
        connection_id.LowPart, connection_id.HighPart);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    OBJECT_ATTRIBUTES comm_attributes;
    IO_STATUS_BLOCK io_status;
    InitializeObjectAttributes(&comm_attributes, &device_name, OBJ_KERNEL_HANDLE, NULL, NULL);
    status = ZwOpenFile(&tgt->m_iotarget_handle, desired_access, &comm_attributes, &io_status, 0, 0);
    if (!NT_SUCCESS(status)) {
        PRINT_INFO("ZwOpenFile() failed: dev-name-%S, handle-0x%p, status-0x%lx\n",
            device_name.Buffer, tgt->m_iotarget_handle, status);
        return status;
    }

    status = ObReferenceObjectByHandle(tgt->m_iotarget_handle, desired_access, *IoFileObjectType, KernelMode, (PVOID*)&tgt->m_iotarget_file_object_ptr, NULL);
    if (!NT_SUCCESS(status)) {
        comm_close(tgt);
        return status;
    }

    tgt->m_iotarget_device_object_ptr = IoGetRelatedDeviceObject(tgt->m_iotarget_file_object_ptr);
    if (tgt->m_iotarget_device_object_ptr == NULL) {
        comm_close(tgt);
        return STATUS_NO_SUCH_DEVICE;
    }

    KeInitializeEvent(&tgt->m_event, NotificationEvent, FALSE);

    return STATUS_SUCCESS;
}

void comm_close(iotarget_handles *tgt)
{
    if (tgt->m_iotarget_file_object_ptr) {
        ObDereferenceObject(tgt->m_iotarget_file_object_ptr);
        tgt->m_iotarget_file_object_ptr = NULL;
    }
    if (tgt->m_iotarget_handle != NULL) {
        ZwClose(tgt->m_iotarget_handle);
        tgt->m_iotarget_handle = NULL;
    }

}

NTSTATUS i2c_write(iotarget_handles *i2c_tgt, ULONG reg_addr, PVOID buffer, ULONG buffer_length)
{
    UINT8 transfer_buffer[I2C_BUFFER_SIZE];
    SPB_TRANSFER_LIST tx_list;
    NTSTATUS status;
    IO_STATUS_BLOCK io_status;

    if ((buffer_length == 0) || (I2C_REGISTER_ADDR_SIZE + buffer_length > I2C_BUFFER_SIZE) || (!i2c_tgt) || (!buffer))
    {
        return STATUS_INVALID_PARAMETER;
    }

    transfer_buffer[0] = (UINT8)reg_addr;
    RtlCopyMemory(&transfer_buffer[I2C_REGISTER_ADDR_SIZE], buffer, buffer_length);

    SPB_TRANSFER_LIST_INIT(&tx_list, 1);
    tx_list.Transfers[0] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(SpbTransferDirectionToDevice, 0, transfer_buffer, (I2C_REGISTER_ADDR_SIZE + buffer_length));

    status = comm_send_irp(i2c_tgt, IOCTL_SPB_EXECUTE_SEQUENCE, (PVOID)&tx_list, sizeof(tx_list), NULL, 0, &io_status);
    return status;
}

NTSTATUS i2c_read(iotarget_handles *i2c_tgt, ULONG reg_addr, PVOID buffer, ULONG buffer_length)
{
    NTSTATUS status;
    IO_STATUS_BLOCK io_status;

    if ((buffer_length == 0) || (!i2c_tgt) || (!buffer))
    {
        return STATUS_INVALID_PARAMETER;
    }

    SPB_TRANSFER_LIST_AND_ENTRIES(I2C_RX_TRANSFER_COUNT)  rx_list;
    SPB_TRANSFER_LIST_INIT(&(rx_list.List), I2C_RX_TRANSFER_COUNT);
    rx_list.List.Transfers[0] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(SpbTransferDirectionToDevice, 0, &reg_addr, I2C_REGISTER_ADDR_SIZE);
    rx_list.List.Transfers[1] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(SpbTransferDirectionFromDevice, 0, buffer, buffer_length);

    status = comm_send_irp(i2c_tgt, IOCTL_SPB_EXECUTE_SEQUENCE, (PVOID)&rx_list, sizeof(rx_list), NULL, 0, &io_status);
    return status;
}

NTSTATUS gpio_write(iotarget_handles *gpio_tgt, ULONG val)
{
    NTSTATUS status;
    IO_STATUS_BLOCK io_status;

    if (!gpio_tgt)
    {
        return STATUS_INVALID_PARAMETER;
    }

    status = comm_send_irp(gpio_tgt, IOCTL_GPIO_WRITE_PINS, (PVOID)&val, sizeof(val), (PVOID)&val, sizeof(val), &io_status);
    return status;
}

void comm_clear_handle(iotarget_handles *tgt)
{
    tgt->m_iotarget_handle = NULL;
    tgt->m_iotarget_file_object_ptr = NULL;
    tgt->m_iotarget_device_object_ptr = NULL;
}

} // extern "C"
