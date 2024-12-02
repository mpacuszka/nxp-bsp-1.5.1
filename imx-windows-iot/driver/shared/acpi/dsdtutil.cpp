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

#include "dsdtutil.hpp"


NTSTATUS AcpiDsdRes_t::LoadDsd()
/*!
 * Loads DSD from PDO.
 *
 * @param WdfDevice handle to Wdf device object.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;

    m_DsdBufferPtr = nullptr;
    if (m_PdoPtr == NULL) {
        status = STATUS_INVALID_ADDRESS;
    } else {
        status = AcpiQueryDsd(m_PdoPtr, &m_DsdBufferPtr);
        if (!NT_SUCCESS(status)) {
            DbgPrint("AcpiQueryDsd Failed 0x%x\r\n", status);
        } else {
            status = AcpiParseDsdAsDeviceProperties(m_DsdBufferPtr, &m_DevicePropertiesPkgPtr);
            if (!NT_SUCCESS(status)) {
                DbgPrint("AcpiParseDsdAsDeviceProperties Failed 0x%x\r\n", status);
            }
        }
    }
    return status;
}

void AcpiDsdRes_t::Cleanup()
/*!
 * Frees DSD data.
 */
{
    if (m_DsdBufferPtr != nullptr) {
        ExFreePool(m_DsdBufferPtr);
        m_DsdBufferPtr = NULL;
    }
}


NTSTATUS AcpiDsdRes_t::EvalMethodSync(
    ACPI_EVAL_INPUT_BUFFER* InputBufferPtr,
    SIZE_T InputBufferSize,
    PACPI_EVAL_OUTPUT_BUFFER* OutBufferPtr)
    /*!
     * Call ACPI method.
     *
     * @param InputBufferPtr pointer to structure containing arguments.
     * @param InputBufferSize Size of InputBuffer structure.
     * @param OutBufferPtr Returned structure allocated by this method. Caller is supposed to free it.
     *
     * @returns STATUS_SUCCESS or error code.
     */
{
    NTSTATUS status;
    UINT32 retries = 2;
    ACPI_EVAL_OUTPUT_BUFFER UNALIGNED* outBufferPtr;
    UINT32 outputBufferSize = sizeof(ACPI_EVAL_OUTPUT_BUFFER) + 8;
    ULONG sizeReturned;

    if (m_PdoPtr == NULL) {
        return STATUS_INVALID_PARAMETER_1;
    }
    if (InputBufferPtr->Signature == 0) {
        return STATUS_INVALID_PARAMETER_2;
    }
    if (OutBufferPtr == NULL) {
        return STATUS_INVALID_PARAMETER_4;
    }

    do
    {
        outBufferPtr = static_cast<ACPI_EVAL_OUTPUT_BUFFER*>(ExAllocatePoolWithTag(NonPagedPoolNx, outputBufferSize, ACPI_TAG_EVAL_OUTPUT_BUFFER));
        if (outBufferPtr == nullptr) {
            status = STATUS_INSUFFICIENT_RESOURCES;
            break;    // goto Cleanup;
        } else {
            RtlZeroMemory(outBufferPtr, outputBufferSize);

            status = AcpiSendIoctlSynchronously(m_PdoPtr, IOCTL_ACPI_EVAL_METHOD, InputBufferPtr, (ULONG)InputBufferSize, outBufferPtr, (ULONG)outputBufferSize, &sizeReturned);
            if (status == STATUS_BUFFER_OVERFLOW) {
                outputBufferSize = outBufferPtr->Length;
                ExFreePoolWithTag(outBufferPtr, ACPI_TAG_EVAL_OUTPUT_BUFFER);
                outBufferPtr = nullptr;
            }
            --retries;
        }
    } while ((status == STATUS_BUFFER_OVERFLOW) && (retries > 0));

    if (!NT_SUCCESS(status)) {
        DbgPrint("AcpiDsdRes_t::EvalMethodSync Failed 0x%x\r\n", status);
        if (outBufferPtr != nullptr) {
            ExFreePoolWithTag(outBufferPtr, ACPI_TAG_EVAL_OUTPUT_BUFFER);
            outBufferPtr = nullptr;
        }
    }
    *OutBufferPtr = outBufferPtr;

    return status;
}

NTSTATUS AcpiDsdRes_t::GetString(const CHAR* ValueName, SIZE_T DestinationSize, UINT32* Length, PCHAR DestinationPtr)
{
    NTSTATUS status = STATUS_SUCCESS;

    status = AcpiDevicePropertiesQueryStringValue(m_DevicePropertiesPkgPtr, ValueName, (UINT32)DestinationSize, Length, DestinationPtr);
    return status;
}

NTSTATUS AcpiDsdRes_t::GetInteger(const CHAR* ValueName, UINT8* DestinationPtr)
{
    NTSTATUS status = STATUS_SUCCESS;

    status = AcpiDevicePropertiesQueryIntegerValue(m_DevicePropertiesPkgPtr, ValueName, DestinationPtr);
    return status;
}

NTSTATUS AcpiDsdRes_t::GetInteger(const CHAR* ValueName, UINT32* DestinationPtr)
{
    NTSTATUS status = STATUS_SUCCESS;

    status = AcpiDevicePropertiesQueryIntegerValue(m_DevicePropertiesPkgPtr, ValueName, DestinationPtr);
    return status;
}

NTSTATUS AcpiDsdRes_t::GetDsdResources(const _DSDVAL_GET_DESCRIPTOR DescTable[], UINT16 DescriptorLen)
/*!
 * Parses required information from DSD to required locations.
 *
 * @param DescTable array of NAME:Destination information.
 * @param DescriptorLen number of entries in DescTable.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    for (ULONG i = 0; i < DescriptorLen; ++i) {
        const _DSDVAL_GET_DESCRIPTOR* currEntryPtr = &DescTable[i];

        switch (currEntryPtr->Type) {
        case ACPI_METHOD_ARGUMENT_INTEGER:
        {
            switch (currEntryPtr->DestinationSize) {
            case 1:
                status = AcpiDevicePropertiesQueryIntegerValue(m_DevicePropertiesPkgPtr, currEntryPtr->ValueName, (UINT8*)currEntryPtr->DestinationPtr);
                break;
            case 2:
                status = AcpiDevicePropertiesQueryIntegerValue(m_DevicePropertiesPkgPtr, currEntryPtr->ValueName, (UINT16*)currEntryPtr->DestinationPtr);
                break;
            case 4:
                status = AcpiDevicePropertiesQueryIntegerValue(m_DevicePropertiesPkgPtr, currEntryPtr->ValueName, (UINT32*)currEntryPtr->DestinationPtr);
                break;
            default:
                status = STATUS_INVALID_PARAMETER;
                break;
            }
            break; // case ACPI_METHOD_ARGUMENT_INTEGER
        }
        case ACPI_METHOD_ARGUMENT_STRING:
        {
            UINT32 length = 0;
                
            status = AcpiDevicePropertiesQueryStringValue(m_DevicePropertiesPkgPtr, currEntryPtr->ValueName, (UINT32)currEntryPtr->DestinationSize, &length, (const PCHAR)currEntryPtr->DestinationPtr);
            break; // case ACPI_METHOD_ARGUMENT_STRING:
        }
        default:
            // Do nothing.
            status = STATUS_ACPI_INVALID_ARGTYPE;
            break;
        }
        if (!NT_SUCCESS(status)) {
            DbgPrint("AcpiDsdRes_t failed to query value of %s; Status 0x%x.\r\n", currEntryPtr->ValueName, status);
            break; // Break the for loop and return failure.
        }
    }
    return status;
}