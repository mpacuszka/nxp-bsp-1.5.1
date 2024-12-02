/*
 * Copyright 2022-2023 NXP
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

#include "imx_acpi_utils.h"
#include <initguid.h>


NTSTATUS AcpiOuputBuffer_GetNextArgument(_In_ ACPI_EVAL_OUTPUT_BUFFER UNALIGNED* pOutputBuffer, _Inout_ ACPI_METHOD_ARGUMENT UNALIGNED** ppArgument, _In_ USHORT ArgumentType);
NTSTATUS AcpiPackage_GetNextArgument(_In_ ACPI_METHOD_ARGUMENT UNALIGNED* pPackage, _Inout_ ACPI_METHOD_ARGUMENT UNALIGNED** ppArgument, _In_ USHORT ArgumentType);
NTSTATUS Acpi_GetDevicePropertyValueArgument(_In_ IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext, _In_ const CHAR* pPropertName, _Inout_ ACPI_METHOD_ARGUMENT UNALIGNED** ppPropertyValueArgument, _In_ USHORT ArgumentType);
_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS Acpi_GetDevicePropertiesPackage(_In_ IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext);

#pragma alloc_text(PAGE, AcpiOuputBuffer_GetNextArgument)
#pragma alloc_text(PAGE, AcpiPackage_GetNextArgument)
#pragma alloc_text(PAGE, Acpi_GetDevicePropertiesPackage)
#pragma alloc_text(PAGE, Acpi_GetDevicePropertyValueArgument)
#pragma alloc_text(PAGE, Acpi_GetIntegerPropertyValue)
#pragma alloc_text(PAGE, Acpi_GetBufferPropertyAddress)
#pragma alloc_text(PAGE, Acpi_GetStringPropertyAddress)

DEFINE_GUID(ACPI_DEVICE_PROPERTIES_DSD_GUID, 0xDAFFD814, 0x6EBA, 0x4D8C, 0x8A, 0x91, 0xBC, 0x9B, 0xBF, 0x4A, 0xA3, 0x01);

#define ACPI_DSD_STRING ((ULONG)'DSD_')
// _DSM Execution Timeout (in 100-ns units)
#define IMX_ACPI_EXECUTION_TIMEOUT 30000    // 3 seconds


#ifdef  DBG

LARGE_INTEGER        DriverStartTime;

static inline LONG GetTime() {
    LARGE_INTEGER  CurrentSystemTime;
    KeQuerySystemTimePrecise(&CurrentSystemTime);
    return (LONG)((CurrentSystemTime.QuadPart - DriverStartTime.QuadPart) / 10000);
}

#ifndef DBG_MSG_DRV_PREFIX
#define DBG_MSG_DRV_PREFIX "Acpi"
#endif

#define DBG_PRINT_ERROR(_format_str_,...)                            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s: !!! ERROR !!! %s() "_format_str_"\n"          ,GetTime() ,KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_PRINT_ERROR_WITH_STATUS(_status_,_format_str_,...)       DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s: !!! ERROR !!! %s() "_format_str_" [0x%.8X]\n" ,GetTime() ,KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__,_status_)

//#define DBG_ACPI
#ifdef DBG_ACPI
#define DBG_ACPI_METHOD_BEG()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__)
#define DBG_ACPI_METHOD_BEG_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_ACPI_METHOD_END()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__)
#define DBG_ACPI_METHOD_END_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_ACPI_METHOD_END_WITH_STATUS(_status_)                    DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X]\n"             ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,_status_)
#define DBG_ACPI_PRINT_WARNING(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s() "_format_str_"\n"       ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_ACPI_PRINT_VERBOSE(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s() "_format_str_"\n"       ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_ACPI_PRINT_INFO(_format_str_,...)                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s() "_format_str_"\n"       ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#endif

#endif

#ifndef DBG_PRINT_ERROR
#define DBG_PRINT_ERROR(...)
#endif
#ifndef DBG_PRINT_ERROR_WITH_STATUS
#define DBG_PRINT_ERROR_WITH_STATUS(...)
#endif
#ifndef DBG_ACPI_METHOD_BEG
#define DBG_ACPI_METHOD_BEG(...)
#endif
#ifndef DBG_ACPI_METHOD_BEG_WITH_PARAMS
#define DBG_ACPI_METHOD_BEG_WITH_PARAMS(...)
#endif
#ifndef DBG_ACPI_METHOD_END
#define DBG_ACPI_METHOD_END(...)
#endif
#ifndef DBG_ACPI_METHOD_END_WITH_PARAMS
#define DBG_ACPI_METHOD_END_WITH_PARAMS(...)
#endif
#ifndef DBG_ACPI_METHOD_END_WITH_STATUS
#define DBG_ACPI_METHOD_END_WITH_STATUS(...)
#endif
#ifndef DBG_ACPI_PRINT_WARNING
#define DBG_ACPI_PRINT_WARNING(...)
#endif
#ifndef DBG_ACPI_PRINT_VERBOSE
#define DBG_ACPI_PRINT_VERBOSE(...)
#endif
#ifndef DBG_ACPI_PRINT_INFO
#define DBG_ACPI_PRINT_INFO(...)
#endif

/*++
Routine Description:
    Returns next argumet of the output buffer.
Arguments:
    pOutputBuffer     - Pointer to the ouptut buffer.
    ppArgument        - Pointer to pointer that will contain next argument in the output buffer.
    ArgumentType      - Argunent type.
Return Value:
    NDIS_STATUS_SUCCESS, STATUS_NO_MORE_ENTRIES or STATUS_ACPI_INVALID_ARGTYPE
--*/
_Use_decl_annotations_
NTSTATUS AcpiOuputBuffer_GetNextArgument(ACPI_EVAL_OUTPUT_BUFFER UNALIGNED* pOutputBuffer, ACPI_METHOD_ARGUMENT UNALIGNED** ppArgument, USHORT ArgumentType)
{
    ACPI_METHOD_ARGUMENT UNALIGNED* pNewArgument;

    PAGED_CODE();
    // A null value indicates start enumeration from first argument
    if (*ppArgument == NULL) {
        pNewArgument = pOutputBuffer->Argument;
    }
    else {
        pNewArgument = ACPI_METHOD_NEXT_ARGUMENT(*ppArgument);
    }
    // Check end of buffer
    if ((pNewArgument >= ACPI_EVAL_OUTPUT_BUFFER_ARGUMENTS_END(pOutputBuffer))) {
        return STATUS_NO_MORE_ENTRIES;
    }
    if (pNewArgument->Type != ArgumentType) {
        return STATUS_ACPI_INVALID_ARGTYPE;
    }
    *ppArgument = pNewArgument;
    return STATUS_SUCCESS;
}

/*++
Routine Description:
    Returns next argumet of the package.
Arguments:
    pOutputBuffer     - Pointer to the package.
    ppArgument        - Pointer to pointer that will contain next argument in the package.
    ArgumentType      - Argunent type.
Return Value:
    NDIS_STATUS_SUCCESS, STATUS_NO_MORE_ENTRIES or STATUS_ACPI_INVALID_ARGTYPE
--*/
_Use_decl_annotations_
NTSTATUS AcpiPackage_GetNextArgument(_In_ ACPI_METHOD_ARGUMENT UNALIGNED* pPackage, _Inout_ ACPI_METHOD_ARGUMENT UNALIGNED** ppArgument, _In_ USHORT ArgumentType)
{
    ACPI_METHOD_ARGUMENT UNALIGNED* pNewArgument;

    PAGED_CODE();
    // Package is either empty or incomplete
    if (pPackage == NULL) {
        return STATUS_NO_MORE_ENTRIES;
    }
    // Package is either empty or incomplete
    if (pPackage->DataLength < sizeof(ACPI_METHOD_ARGUMENT)) {
        return STATUS_NO_MORE_ENTRIES;
    }
    // A null value indicates start enumeration from first argument
    if (*ppArgument == NULL) {
        pNewArgument = (PACPI_METHOD_ARGUMENT) & (pPackage->Data);
    }
    else {
        pNewArgument = ACPI_METHOD_NEXT_ARGUMENT(*ppArgument);
    }
    // Check end of package
    if (pNewArgument >= (PACPI_METHOD_ARGUMENT)(pPackage->Data + pPackage->DataLength)) {
        return STATUS_NO_MORE_ENTRIES;
    }
    if (pNewArgument->Type != ArgumentType) {
        return STATUS_ACPI_INVALID_ARGTYPE;
    }
    *ppArgument = pNewArgument;
    return STATUS_SUCCESS;
}

/*++
Routine Description:
    Allocates the output buffer and finds the "Property package" in _DSD.
    User must dealocate output buffer as soon as it is not needed.
Arguments:
    pAcpiDevContext - Pointer to the ACPI device context structure.
Return Value:
    NDIS_STATUS_SUCCESS or STATUS_ACPI_INVALID_ARGTYPE
--*/
_Use_decl_annotations_
NTSTATUS Acpi_GetDevicePropertiesPackage(IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext) {
    NTSTATUS                              Status;
    PIRP                                  pIrp = NULL;
    IO_STATUS_BLOCK                       IoStatus = { 0 };
    KEVENT                                Event;
    LARGE_INTEGER                         Timeout;
    ACPI_EVAL_OUTPUT_BUFFER* pOutputBuffer;
    ACPI_EVAL_INPUT_BUFFER                InputBuffer;
    int                                   retries = 2;
    ULONG                                 OutputBufferLength = sizeof(ACPI_EVAL_OUTPUT_BUFFER);
    ACPI_METHOD_ARGUMENT UNALIGNED* pArgument = NULL;

    DBG_ACPI_METHOD_BEG();
    PAGED_CODE();
    do {
        // Get _DSD from ACPI (alocate buffer for data)
        do {
            KeInitializeEvent(&Event, NotificationEvent, FALSE);
            if ((pOutputBuffer = (ACPI_EVAL_OUTPUT_BUFFER*)(ExAllocatePoolWithTag(NonPagedPoolNx, OutputBufferLength, pAcpiDevContext->MemPoolTag))) == NULL) {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                break;
            }
            RtlZeroMemory(pOutputBuffer, OutputBufferLength);
            InputBuffer.Signature = ACPI_EVAL_INPUT_BUFFER_SIGNATURE;
            InputBuffer.MethodNameAsUlong = ACPI_DSD_STRING;
            if ((pIrp = IoBuildDeviceIoControlRequest(IOCTL_ACPI_EVAL_METHOD, pAcpiDevContext->Pdo, &InputBuffer, sizeof(ACPI_EVAL_INPUT_BUFFER), pOutputBuffer, OutputBufferLength, FALSE, &Event, &IoStatus)) == NULL) {
                Status = STATUS_INSUFFICIENT_RESOURCES;
                DBG_PRINT_ERROR_WITH_STATUS(Status, "IoBuildDeviceIoControlRequest() failed.");
                break;
            }
            if ((Status = IoCallDriver(pAcpiDevContext->Pdo, pIrp)) == STATUS_PENDING) {
                Timeout.QuadPart = IMX_ACPI_EXECUTION_TIMEOUT;
                KeWaitForSingleObject(&Event, Executive, KernelMode, FALSE, &Timeout);
                Status = IoStatus.Status;
            }
            if (Status == STATUS_BUFFER_OVERFLOW) {
                OutputBufferLength = pOutputBuffer->Length;
                ExFreePoolWithTag(pOutputBuffer, pAcpiDevContext->MemPoolTag);
                pOutputBuffer = NULL;
            }
            retries--;
        } while ((Status == STATUS_BUFFER_OVERFLOW) && (retries > 0));
        if (!NT_SUCCESS(Status)) {
            DBG_ACPI_PRINT_WARNING("IOCTL_ACPI_EVAL_METHOD for _DSD failed.");
            break;
        }
        // Check output buffer
        if (pOutputBuffer->Signature != ACPI_EVAL_OUTPUT_BUFFER_SIGNATURE) {
            Status = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR_WITH_STATUS(Status, "pOutputBuffer->Signature != ACPI_EVAL_OUTPUT_BUFFER_SIGNATURE.");
            break;
        }
        if (pOutputBuffer->Count != 2) {
            Status = STATUS_ACPI_INCORRECT_ARGUMENT_COUNT;
            DBG_PRINT_ERROR_WITH_STATUS(Status, "pOutputBuffer->Count != 2.");
        }
        // First argument of the _DSD must be a GUID
        if (!NT_SUCCESS(Status = AcpiOuputBuffer_GetNextArgument(pOutputBuffer, &pArgument, ACPI_METHOD_ARGUMENT_BUFFER))) {
            DBG_PRINT_ERROR_WITH_STATUS(Status, "GUID argument is missing or its type is not BUFFER.");
            break;
        }
        if (pArgument->DataLength != sizeof(GUID)) {
            Status = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR_WITH_STATUS(Status, "GUID argument size is not valid.");
            break;
        }
        // Check GUID value
        if (!(((unsigned long*)&pArgument->Data)[0] == ((unsigned long*)&ACPI_DEVICE_PROPERTIES_DSD_GUID)[0] &&
            ((unsigned long*)&pArgument->Data)[1] == ((unsigned long*)&ACPI_DEVICE_PROPERTIES_DSD_GUID)[1] &&
            ((unsigned long*)&pArgument->Data)[2] == ((unsigned long*)&ACPI_DEVICE_PROPERTIES_DSD_GUID)[2] &&
            ((unsigned long*)&pArgument->Data)[3] == ((unsigned long*)&ACPI_DEVICE_PROPERTIES_DSD_GUID)[3])) {
            Status = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR_WITH_STATUS(Status, "GUID argument value is not ACPI_DEVICE_PROPERTIES_DSD_GUID.");
            break;
        }
        // Next argument must be Package of two items packages - Package () { Package(2) {"Property 1 name", Property 1 value}, {"Property 2 name", Property 2 value} }
        if (!NT_SUCCESS(Status = AcpiOuputBuffer_GetNextArgument(pOutputBuffer, &pArgument, ACPI_METHOD_ARGUMENT_PACKAGE))) {
            DBG_PRINT_ERROR_WITH_STATUS(Status, "No PACKAGE argument found after GUID argument.");
            break;
        }
    } while (0);
    if (NT_SUCCESS(Status)) {
        pAcpiDevContext->pOutputBuffer = pOutputBuffer;
        pAcpiDevContext->pPropertiesPackage = pArgument;
    }
    else {
        if (pOutputBuffer != NULL) {
            ExFreePoolWithTag(pOutputBuffer, pAcpiDevContext->MemPoolTag);
        }
    }
    DBG_ACPI_METHOD_END_WITH_STATUS(Status);
    return Status;
}

/*++
Routine Description:
    Returns the property value argument.
Arguments:
    pAcpiDevContext          - Pointer to the ACPI device context structure.
    pPropertName             - Property name.
    ppPropertyValueArgument  - Pointer to pointer to the requested peroperty value argument.
    ArgumentType             - Argunent type.
Return Value:
    NDIS_STATUS_SUCCESS, STATUS_NO_MORE_ENTRIES or STATUS_ACPI_INVALID_ARGTYPE
--*/
_Use_decl_annotations_
NTSTATUS Acpi_GetDevicePropertyValueArgument(IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext, const CHAR* pPropertName, ACPI_METHOD_ARGUMENT UNALIGNED** ppPropertyValueArgument, USHORT ArgumentType)
{
    NTSTATUS                        Status;
    ACPI_METHOD_ARGUMENT UNALIGNED* pCurrentPropertyPackage = NULL;
    ANSI_STRING                     sKeyName;
    __analysis_assume_nullterminated(pPropertName);

    PAGED_CODE();
    *ppPropertyValueArgument = NULL;
    if (!NT_SUCCESS(Status = RtlInitAnsiStringEx(&sKeyName, pPropertName))) {
        return Status;
    }
    // Example Device Properties
    // Package () {
    //   Package (2) { "Key1", Value1 },
    //   Package (2) { "Key2", Package () {Value2, Value3, ..} },
    //   Package (2) { "Key3", ValueX }
    // }
    for (;;) {
        // Each element in the Device Properties package is a non-empty package key/value pair
        if (!NT_SUCCESS(Status = AcpiPackage_GetNextArgument(pAcpiDevContext->pPropertiesPackage, &pCurrentPropertyPackage, ACPI_METHOD_ARGUMENT_PACKAGE))) {
            return Status;
        }
        ACPI_METHOD_ARGUMENT UNALIGNED* pPropertyName = NULL;
        // Key should be string
        if (!NT_SUCCESS(Status = AcpiPackage_GetNextArgument(pCurrentPropertyPackage, &pPropertyName, ACPI_METHOD_ARGUMENT_STRING))) {
            return Status;
        }
        ANSI_STRING currentKeyNameStr;
        _Analysis_assume_nullterminated_(pPropertyName->Data);
        RtlInitAnsiStringEx(&currentKeyNameStr, (PCSZ)(pPropertyName->Data));
        if (!RtlEqualString(&currentKeyNameStr, &sKeyName, FALSE)) {
            continue;
        }
        if (!NT_SUCCESS(Status = AcpiPackage_GetNextArgument(pCurrentPropertyPackage, &pPropertyName, ArgumentType))) {
            return Status;
        }
        *ppPropertyValueArgument = pPropertyName;
        break;
    }
    return STATUS_SUCCESS;
}

/*++
Routine Description:
    Returns the integer property value.
Arguments:
    pAcpiDevContext- Pointer to the ACPI device context structure.
    pPropertName   - Property name.
    pBuffer        - Pointer to pointer to the data buffer.
Return Value: none
--*/
_Use_decl_annotations_
void Acpi_GetIntegerPropertyValue(IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext, const CHAR* pPropertName, ULONG* pBuffer)
{
    NTSTATUS                        Status;
    ACPI_METHOD_ARGUMENT UNALIGNED* pPropertyValue = NULL;

    DBG_ACPI_METHOD_BEG();
    PAGED_CODE();
    if (!NT_SUCCESS(Status = Acpi_GetDevicePropertyValueArgument(pAcpiDevContext, pPropertName, &pPropertyValue, ACPI_METHOD_ARGUMENT_INTEGER))) {
        DBG_ACPI_PRINT_INFO("Propery %s not found, default value 0x%08X used", pPropertName, *pBuffer);
    }
    else {
        *pBuffer = pPropertyValue->Argument;
        DBG_ACPI_PRINT_INFO("Propery %s found in ACPI, value: 0x%08X", pPropertName, *pBuffer);
    }
    DBG_ACPI_METHOD_END();
    return;
}

/*++
Routine Description:
    Returns address and size of buffer property.
Arguments:
    pAcpiDevContext - Pointer to the ACPI device context structure.
    pPropertName    - Property name.
    pBuffer         - Pointer to pointer to the data buffer.
Return Value:
        NDIS_STATUS_SUCCESS, STATUS_NO_MORE_ENTRIES or STATUS_ACPI_INVALID_ARGTYPE
--*/
_Use_decl_annotations_
void Acpi_GetBufferPropertyAddress(IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext, const CHAR* pPropertName, UINT8** pBuffer, ULONG* BuffSize)
{
    NTSTATUS                        Status;
    ACPI_METHOD_ARGUMENT UNALIGNED* pPropertyValue = NULL;

    DBG_ACPI_METHOD_BEG();
    PAGED_CODE();
    if (!NT_SUCCESS(Status = Acpi_GetDevicePropertyValueArgument(pAcpiDevContext, pPropertName, &pPropertyValue, ACPI_METHOD_ARGUMENT_BUFFER))) {
        *BuffSize = 0;
        DBG_ACPI_PRINT_INFO("Propery %s not found", pPropertName);
    }
    else {
        *pBuffer = (UINT8*)&pPropertyValue->Data;
        *BuffSize = pPropertyValue->DataLength;
        DBG_ACPI_PRINT_INFO("Propery %s found in ACPI, Address: 0x%016p, size: %d ", pPropertName, *pBuffer, *BuffSize);
    }
    DBG_ACPI_METHOD_END();
    return;
}

/*++
Routine Description:
    Returns address and size of string property.
Arguments:
    pAcpiDevContext - Pointer to the ACPI device context structure.
    pPropertName    - Property name.
    pBuffer         - Pointer to pointer to the data buffer.
Return Value: none
--*/
_Use_decl_annotations_
void Acpi_GetStringPropertyAddress(IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext, const CHAR* pPropertName, char** pBuffer, ULONG* BuffSize)
{
    NTSTATUS                        Status;
    ACPI_METHOD_ARGUMENT UNALIGNED* pPropertyValue = NULL;

    DBG_ACPI_METHOD_BEG();
    PAGED_CODE();
    if (!NT_SUCCESS(Status = Acpi_GetDevicePropertyValueArgument(pAcpiDevContext, pPropertName, &pPropertyValue, ACPI_METHOD_ARGUMENT_STRING))) {
        *BuffSize = 0;
        DBG_ACPI_PRINT_INFO("Propery %s not found", pPropertName);
    }
    else {
        *pBuffer = (char*)&pPropertyValue->Data;
        *BuffSize = pPropertyValue->DataLength;
        DBG_ACPI_PRINT_INFO("Propery %s found in ACPI, Address: 0x%016p, size: %d ", pPropertName, *pBuffer, *BuffSize);
    }
    DBG_ACPI_METHOD_END();
    return;
}

/*++
Routine Description:
    Initialzes acpi support.
Arguments:
    pAcpiDevContext       - Pointer to the ACPI context data structure.
Return Value: STATUS_SUCCESS, STATUS_UNSUCCESSFUL.
--*/
_Use_decl_annotations_
NTSTATUS Acpi_Init(IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext) {
#if DBG
    KeQuerySystemTimePrecise(&DriverStartTime);
#endif
    DBG_ACPI_METHOD_BEG();
    NTSTATUS ntStatus = STATUS_SUCCESS;
    do {
        if (pAcpiDevContext->Pdo == NULL) {
            ntStatus = STATUS_UNSUCCESSFUL;
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "Inavlid PDO object");
            break;
        }
        pAcpiDevContext->pOutputBuffer = NULL;
        if (!NT_SUCCESS(ntStatus = Acpi_GetDevicePropertiesPackage(pAcpiDevContext))) {
            break;
        }
    } while (0);
    DBG_ACPI_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/*++
Routine Description:
    Deinitialzes acpi support.
Arguments:
    pAcpiDevContext - Pointer to the ACPI device context structure.
Return Value: none
--*/
_Use_decl_annotations_
void Acpi_Deinit(IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext) {
    DBG_ACPI_METHOD_BEG();
    if (pAcpiDevContext->pOutputBuffer != NULL) {
        ExFreePoolWithTag(pAcpiDevContext->pOutputBuffer, pAcpiDevContext->MemPoolTag);
        pAcpiDevContext->pOutputBuffer = NULL;
    }
    DBG_ACPI_METHOD_END();
}
