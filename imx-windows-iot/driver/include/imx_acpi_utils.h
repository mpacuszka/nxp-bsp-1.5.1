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

#include <Ntddk.h>
#include <acpiioct.h>

#pragma once

typedef struct IMX_ACPI_UTILS_DEV_CONTEXT_S {
    PDEVICE_OBJECT                     Pdo;
    ULONG                              MemPoolTag;               /*  The pool tag to use for the allocated memory */
    ACPI_EVAL_OUTPUT_BUFFER UNALIGNED* pOutputBuffer;
    ACPI_METHOD_ARGUMENT    UNALIGNED* pPropertiesPackage;
} IMX_ACPI_UTILS_DEV_CONTEXT, * PIMX_ACPI_UTILS_DEV_CONTEXT;

EXTERN_C_START

_IRQL_requires_max_(PASSIVE_LEVEL)
void Acpi_GetIntegerPropertyValue(_In_ IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext, _In_ const CHAR* pPropertName, _Inout_ ULONG* pBuffer);
_IRQL_requires_max_(PASSIVE_LEVEL)
void Acpi_GetBufferPropertyAddress(_In_ IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext, _In_ const CHAR* pPropertName, _Inout_ UINT8** pBuffer, _Out_ ULONG* BuffSize);
_IRQL_requires_max_(PASSIVE_LEVEL)
void Acpi_GetStringPropertyAddress(_In_ IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext, _In_ const CHAR* pPropertName, _Inout_ char** pBuffer, _Out_ ULONG* BuffSize);
_IRQL_requires_max_(PASSIVE_LEVEL)
NTSTATUS Acpi_Init(_Inout_ IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext);
_IRQL_requires_max_(PASSIVE_LEVEL)
void Acpi_Deinit(_Inout_ IMX_ACPI_UTILS_DEV_CONTEXT* pAcpiDevContext);

EXTERN_C_END