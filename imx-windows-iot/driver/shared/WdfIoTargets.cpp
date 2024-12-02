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

#include "WdfIoTargets.hpp"
#include "WdfIoTargets.tmh"
#include <gpio.h>

#if (DBG)
#define _DbgKdPrint 
 // KdPrint
#else // !DBG
#define _DbgKdPrint(...)
#endif // !DBG

NTSTATUS io::IoTargetInit(const connection_res* aConnectionRes)
/*!
 * Open target device.
 *
 * @param aConnectionRes Structure containing Resource Hub device information.
 * @param poolType from which to allocate memory eg. NonPagedPool.
 * @param tag user specified four character (32b) tag for debugging purposes. Usually spelled backwards.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    if (aConnectionRes == NULL) {
        status = STATUS_INSUFFICIENT_RESOURCES;
    }
    if (NT_SUCCESS(status)) {
        status = RESOURCE_HUB_CREATE_PATH_FROM_ID(&m_usDevicePath, aConnectionRes->m_IdLowPart, aConnectionRes->m_IdHighPart);
        if (NT_SUCCESS(status))
        {
            _DbgKdPrint(("IoTargetInit RESOURCE_HUB_CREATE_PATH_FROM_ID  %wZ\r\n", &m_usDevicePath));
            status = WdfIoTargetCreate(m_ParentWdfDevice, WDF_NO_OBJECT_ATTRIBUTES, &m_WdfIoTarget);
            if (NT_SUCCESS(status))
            {
                WDF_IO_TARGET_OPEN_PARAMS openParams;
                WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME(&openParams, &m_usDevicePath, (GENERIC_READ | GENERIC_WRITE));

                openParams.ShareAccess = 0;
                openParams.CreateDisposition = FILE_OPEN;
                openParams.FileAttributes = FILE_ATTRIBUTE_NORMAL;

                status = WdfIoTargetOpen(m_WdfIoTarget, &openParams);
            }
        }
    }
    return status;
}

NTSTATUS io::ctx_acpi_csr_stub::Get_CrsAcpiResources(WDFCMRESLIST ResourcesTranslated)
/*!
 * Parse resource list to m_MeResList, m_I2cResList, m_GpioResList, m_IntResList.
 * Unknown resource will result in an error condition.
 * For more information see https://docs.microsoft.com/en-us/windows-hardware/drivers/ddi/wdm/ns-wdm-_cm_resource_list .
 *
 * @param ResourcesTranslated Structure containing assigned resources.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;
    ULONG resourceCount = WdfCmResourceListGetCount(ResourcesTranslated);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnPrepareHardware()");
    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_DEVICE, "OnPrepareHardware() - %lu resources found.", resourceCount);

    for (ULONG i = 0; i < resourceCount; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR pDescriptor = WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        auto Flags = pDescriptor->Flags;
        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "Found ACPI resource [%lu], type: 0x%x 0x%x 0x%x", i, (unsigned)pDescriptor->Type, (unsigned)pDescriptor->ShareDisposition, (unsigned)pDescriptor->Flags);
        switch (pDescriptor->Type)
        {
        case CmResourceTypeMemory:
        {
            ++m_MemResCnt;
            status = m_MeResList.Emplace(pDescriptor->u.Memory.Start, pDescriptor->u.Memory.Length);
            if (!NT_SUCCESS(status)) {
                _DbgKdPrint(("Failed to add CmResourceTypeMemory.\r\n"));
            }
        }
        break;
        case CmResourceTypeInterrupt:
        {
            ++m_IrqResCnt;
            if (Flags & CM_RESOURCE_INTERRUPT_MESSAGE) {
                status = m_IntResList.Emplace(pDescriptor->u.MessageInterrupt.Translated.Vector, Flags);
            }
            else {
                status = m_IntResList.Emplace(pDescriptor->u.Interrupt.Vector, Flags);
            }
        }
        break;
        case CmResourceTypeConnection:
        {
            auto Class = pDescriptor->u.Connection.Class;
            auto Type = pDescriptor->u.Connection.Type;
            if (Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL) {
                if (Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C) {
                    ++m_I2cResCnt;
                    status = m_I2cResList.Emplace(pDescriptor->u.Connection.IdLowPart, pDescriptor->u.Connection.IdHighPart);
                }
            }
            else {
                if (Class == CM_RESOURCE_CONNECTION_CLASS_GPIO) {
                    if (Type != CM_RESOURCE_CONNECTION_TYPE_GPIO_IO) {
                        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unexpected connection class/type (%lu/%lu), only GPIO class/type (%lu/%lu) is expected!",
                            Class, Type, CM_RESOURCE_CONNECTION_CLASS_GPIO, CM_RESOURCE_CONNECTION_TYPE_GPIO_IO);
                        status = STATUS_DEVICE_CONFIGURATION_ERROR;
                    }
                    else {
                        ++m_GpioResCnt;
                        status = m_GpioResList.Emplace(pDescriptor->u.Connection.IdLowPart, pDescriptor->u.Connection.IdHighPart);
                    }
                }
            }
        }
        break;
        default:
            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "Unknown ACPI resource [%lu], type: 0x%x", i, (unsigned)pDescriptor->Type);
            break;
        }
    }

    return status;
}

NTSTATUS i2c_bus::ReadAddr8(_In_ UINT8 Address, PVOID pBuff, _In_ ULONG Length)
/*!
 * Builds and send synchronous write-read IRP to the I2C io target with 1B register address.
 *
 * @param Address address written to the device before the read.
 * @param pBuff destination bufer for read data.
 * @param Length number of entries to read.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    UCHAR Index;
    const unsigned TRANSFER_COUNT = 2;
    SPB_TRANSFER_LIST_AND_ENTRIES(TRANSFER_COUNT) TransferList;
    PUCHAR TransferBuffer[4];

    if (Length > sizeof(TransferBuffer)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_IOTARGETS, "Unexpected value of data length. Length: %lu. Size of buffer: %lu", Length, sizeof(TransferBuffer));
        status = STATUS_INVALID_PARAMETER;
    }
    else {
        ULONG_PTR BytesTransferred = 0;

        SPB_TRANSFER_LIST_INIT(&(TransferList.List), TRANSFER_COUNT);

        Index = 0; // silence prefast
        TransferList.List.Transfers[Index] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(SpbTransferDirectionToDevice, 0, &Address, sizeof(Address));

        Index += 1; // silence prefast
        TransferList.List.Transfers[Index] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(SpbTransferDirectionFromDevice, 0x0, TransferBuffer, Length);

        //
        // Send the read as a sequence request to the SPB target.
        //
        WDF_MEMORY_DESCRIPTOR MemoryDescriptor;
        WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(&MemoryDescriptor, &TransferList, sizeof(TransferList));

        WDF_REQUEST_SEND_OPTIONS RequestOptions;
        WDF_REQUEST_SEND_OPTIONS_INIT(&RequestOptions, WDF_REQUEST_SEND_OPTION_TIMEOUT);
        RequestOptions.Timeout = WDF_REL_TIMEOUT_IN_MS(I2C_SYNCHRONOUS_TIMEOUT);

        // Send the request to the I2C I/O Target.
        status = WdfIoTargetSendIoctlSynchronously(m_WdfIoTarget, NULL, IOCTL_SPB_EXECUTE_SEQUENCE, &MemoryDescriptor, NULL, &RequestOptions, &BytesTransferred);
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_IOTARGETS, "%s: Failed sending SPB sequence request! Bytes:%lu status:%#x", __FUNCTION__, (ULONG)BytesTransferred, status);
            // Consider device reset
        }
        else {

            if (BytesTransferred != sizeof(Address) + Length) { // REGISTER_ADDR_SIZE + Length
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_IOTARGETS, "Unexpected number of bytes transferred. Expected %lu, transferred %Iu.", 2 + Length, BytesTransferred);
                status = STATUS_INFO_LENGTH_MISMATCH;
            }
            else {
                // Get the returned data out of the buffer.
                RtlCopyMemory(pBuff, TransferBuffer, Length);
            }
        }
    }
    return status;
}
NTSTATUS i2c_bus::ReadAddr16(_In_ UINT16 Address, PVOID pBuff, _In_ ULONG Length)
/*!
 * Builds and send synchronous write-read IRP to the I2C io target.
 *
 * @param Address address written to the device before the read.
 * @param pBuff destination bufer for read data.
 * @param Length number of entries to read.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    UCHAR Index;
    const unsigned TRANSFER_COUNT = 2;
    SPB_TRANSFER_LIST_AND_ENTRIES(TRANSFER_COUNT) TransferList;
    PUCHAR TransferBuffer[4];

    if (Length > sizeof(TransferBuffer)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_IOTARGETS, "Unexpected value of data length. Length: %lu. Size of buffer: %lu", Length, sizeof(TransferBuffer));
        status = STATUS_INVALID_PARAMETER;
    }
    else {
        ULONG_PTR BytesTransferred = 0;

        SPB_TRANSFER_LIST_INIT(&(TransferList.List), TRANSFER_COUNT);

        Index = 0; // silence prefast
        TransferList.List.Transfers[Index] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(SpbTransferDirectionToDevice, 0, &Address, sizeof(UINT16));

        Index += 1; // silence prefast
        TransferList.List.Transfers[Index] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(SpbTransferDirectionFromDevice, 0x0, TransferBuffer, Length);

        {
            //
            // Send the read as a sequence request to the SPB target.
            //
            WDF_MEMORY_DESCRIPTOR MemoryDescriptor;
            WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(&MemoryDescriptor, &TransferList, sizeof(TransferList));

            WDF_REQUEST_SEND_OPTIONS RequestOptions;
            WDF_REQUEST_SEND_OPTIONS_INIT(&RequestOptions, WDF_REQUEST_SEND_OPTION_TIMEOUT);
            RequestOptions.Timeout = WDF_REL_TIMEOUT_IN_MS(I2C_SYNCHRONOUS_TIMEOUT);

            // Send the request to the I2C I/O Target.
            status = WdfIoTargetSendIoctlSynchronously(m_WdfIoTarget, NULL, IOCTL_SPB_EXECUTE_SEQUENCE, &MemoryDescriptor, NULL, &RequestOptions, &BytesTransferred);
            if (!NT_SUCCESS(status)) {
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_IOTARGETS, "%s: Failed sending SPB sequence request! Bytes:%lu status:%#x", __FUNCTION__, (ULONG)BytesTransferred, status);
                // Consider device reset
            }
            else {

                if (BytesTransferred != 2 + Length) { // REGISTER_ADDR_SIZE + Length
                    TraceEvents(TRACE_LEVEL_ERROR, TRACE_IOTARGETS, "Unexpected number of bytes transferred. Expected %lu, transferred %Iu.", 2 + Length, BytesTransferred);
                    status = STATUS_INFO_LENGTH_MISMATCH;
                }
                else {
                    // Get the returned data out of the buffer.
                    RtlCopyMemory(pBuff, TransferBuffer, Length);
                }
            }
        }
    }
    return status;
}

NTSTATUS i2c_bus::Write(_In_ CODEC_COMMAND* CodecCommandPtr)
/*!
 * Builds and send synchronous write IRP to the I2C io target.
 *
 * @param CodecCommand pointer to structure containing address:data. Address gets written first.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    ULONG_PTR bytesWritten;
    WDF_MEMORY_DESCRIPTOR MemDescriptor;

    PAGED_CODE();
    if (m_WdfIoTarget == NULL) {
        status = STATUS_INSUFFICIENT_RESOURCES;
    }
    else {
        WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(&MemDescriptor, &CodecCommandPtr->m_Bytes[0], 2);
        status = WdfIoTargetSendWriteSynchronously(m_WdfIoTarget, NULL, &MemDescriptor, NULL, NULL, &bytesWritten);

        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_IOTARGETS, "WdfIoTargetSendWriteSynchronously failed %!status!", status);
        }
        else {
            if (CodecCommandPtr->m_MsSleepTime.QuadPart != 0) {
                status = KeDelayExecutionThread(KernelMode, false, &CodecCommandPtr->m_MsSleepTime);
            }
        }
    }
    return status;
}

NTSTATUS i2c_bus::WriteBytes(_In_ PVOID Buffer, _In_ ULONG Number)
/*!
 * Builds and send synchronous write IRP to the I2C io target.
 *
 * @param Buffer pointer to array of data to be written to target device.
 * @param Number length of the Buffer array.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    ULONG_PTR bytesWritten;
    WDF_MEMORY_DESCRIPTOR MemDescriptor;

    if (m_WdfIoTarget == NULL) {
        status = STATUS_INSUFFICIENT_RESOURCES;
    }
    else {
        WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(&MemDescriptor, Buffer, Number);
        status = WdfIoTargetSendWriteSynchronously(m_WdfIoTarget, NULL, &MemDescriptor, NULL, NULL, &bytesWritten);

        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_IOTARGETS, "WdfIoTargetSendWriteSynchronously failed %!status!", status);
        }
    }
    return status;
}

NTSTATUS i2c_bus::WriteArray(_In_reads_(NumCommands) CODEC_COMMAND CodecCommand[], USHORT NumCommands)
/*!
 * Builds and send synchronous write IRP to the I2C io target.
 *
 * @param CodecCommand array of address:data pairs. Address gets written first.
 * @param NumCommands length of the CodecCommand array.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    ULONG i;
    NTSTATUS status;

    for (i = 0; i < NumCommands; i++) {
        status = Write(&CodecCommand[i]);

        if (!NT_SUCCESS(status)) {
            return status;
        }
    }

    return STATUS_SUCCESS;
}

NTSTATUS i2c_bus::ReadArray(_In_ USHORT Address, PVOID pBuff, size_t BuffLength)
/*!
 * Builds and send synchronous write IRP to the I2C io target.
 *
 * @param Address byte written to the device before read.
 * @param pBuff pointer to destination array.
 * @param BuffLength length of the pBuff array.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UNREFERENCED_PARAMETER(Address);

    NTSTATUS status;
    ULONG_PTR BytesReturned;
    WDF_MEMORY_DESCRIPTOR MemoryDescriptor;
    WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(&MemoryDescriptor, pBuff, static_cast<ULONG>(BuffLength));

    //
    // Send the SPB sequence IOCTL.
    //
    status = WdfIoTargetSendIoctlSynchronously(m_WdfIoTarget, NULL, IOCTL_SPB_EXECUTE_SEQUENCE, NULL, &MemoryDescriptor, NULL, &BytesReturned);

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_IOTARGETS, "%s: Failed sending SPB sequence request! Bytes:%lu status:%#x", __FUNCTION__, (ULONG)BytesReturned, status);
    }

    return STATUS_SUCCESS;
}

NTSTATUS gpio_t::Set(_In_ UCHAR GpioValue)
/*!
 * Builds and send synchronous write IRP to the Gpio io target.
 *
 * @param GpioValue byte written to the device.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    WDF_OBJECT_ATTRIBUTES RequestAttributes;
    WDF_OBJECT_ATTRIBUTES Attributes;
    WDF_REQUEST_SEND_OPTIONS SendOptions;
    WDFREQUEST IoctlRequest;
    WDFMEMORY WdfMemory;

    if (m_WdfIoTarget == NULL) {
        status = STATUS_INSUFFICIENT_RESOURCES;
    }
    else {
        WDF_OBJECT_ATTRIBUTES_INIT(&RequestAttributes);
        status = WdfRequestCreate(&RequestAttributes, m_WdfIoTarget, &IoctlRequest);
        if (NT_SUCCESS(status)) {
            WDF_OBJECT_ATTRIBUTES_INIT(&Attributes);
            Attributes.ParentObject = IoctlRequest;
            status = WdfMemoryCreatePreallocated(&Attributes, &GpioValue, sizeof(GpioValue), &WdfMemory);
            if (NT_SUCCESS(status)) {
                status = WdfIoTargetFormatRequestForIoctl(m_WdfIoTarget, IoctlRequest, IOCTL_GPIO_WRITE_PINS, WdfMemory, 0, NULL, 0);
                if (NT_SUCCESS(status)) {
                    WDF_REQUEST_SEND_OPTIONS_INIT(&SendOptions, WDF_REQUEST_SEND_OPTION_SYNCHRONOUS);
                    WDF_REQUEST_SEND_OPTIONS_SET_TIMEOUT(&SendOptions, WDF_REL_TIMEOUT_IN_SEC(60));
                    status = WdfRequestAllocateTimer(IoctlRequest);
                    if (NT_SUCCESS(status)) {
                        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "Gpio.Set(%ud).", (unsigned)GpioValue);
                        if (!WdfRequestSend(IoctlRequest, m_WdfIoTarget, &SendOptions)) {
                            status = WdfRequestGetStatus(IoctlRequest);
                        }
                    }
                }
            }
        }
    }

    return status;
}

void test_list()
/*!
 * Simple test for list_item_t implementation. WinDbg breaks in on failure.
 */
{
    struct c_res : list_item_t {
        int a;

        c_res(int A) : a(A) {};
    };

    list_t<c_res> testList;
    testList.Emplace(1);
    testList.Emplace(2);
    testList.Emplace(3);

    auto a = testList.at(0)->a;
    if (a != 1)
        DbgBreakPoint();
    a = testList.at(2)->a;
    if (a != 3)
        DbgBreakPoint();
    auto b = testList.Pop(1);
    if (b != NULL) {
        a = b->a;
        if (a != 2)
            DbgBreakPoint();
    }
    auto d = testList.at(2);
    if (d != NULL)
        DbgBreakPoint();
}