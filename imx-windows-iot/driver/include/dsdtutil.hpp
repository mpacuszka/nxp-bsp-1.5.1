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

extern "C" {
#include <wdm.h>
#include <acpiioct.h>
} // extern "C"
#include "acpiutil.hpp"

class AcpiDsdRes_t
{
    const PDEVICE_OBJECT m_PdoPtr;

public:
    struct _DSDVAL_GET_DESCRIPTOR {
        PCSTR ValueName;
        void* const DestinationPtr;
        const SIZE_T DestinationSize;
        const USHORT Type;

        _DSDVAL_GET_DESCRIPTOR(const char* const ValueName, UINT32* (&& DestinationPtr), USHORT&& Type)
            :ValueName(ValueName), DestinationPtr((void*)DestinationPtr), Type(Type), DestinationSize(sizeof(UINT32)) {};

        _DSDVAL_GET_DESCRIPTOR(const char* const ValueName, UINT8* (&& DestinationPtr), USHORT&& Type)
            :ValueName(ValueName), DestinationPtr((void*)DestinationPtr), Type(Type), DestinationSize(sizeof(UINT8)) {};

        _DSDVAL_GET_DESCRIPTOR(const char* const ValueName, UINT32* (&& DestinationPtr), SIZE_T&& DestinationSize, USHORT&& Type)
            :ValueName(ValueName), DestinationPtr((void*)DestinationPtr), DestinationSize(DestinationSize), Type(Type) {};

    };
    const ACPI_METHOD_ARGUMENT UNALIGNED* m_DevicePropertiesPkgPtr;

    // Init
    PACPI_EVAL_OUTPUT_BUFFER m_DsdBufferPtr;
    AcpiDsdRes_t(const PDEVICE_OBJECT PdoPtr) : m_PdoPtr(PdoPtr), m_DevicePropertiesPkgPtr(NULL) { ASSERT(PdoPtr != NULL); };
    NTSTATUS LoadDsd();
    void Cleanup();
    // Load values from ACPI DSD
    NTSTATUS GetString(const CHAR* ValueName, SIZE_T DestinationSize, UINT32* Length, PCHAR DestinationPtr);
    NTSTATUS GetInteger(const CHAR* ValueName, UINT32* DestinationPtr);
    NTSTATUS GetInteger(const CHAR* ValueName, UINT8* DestinationPtr);
    NTSTATUS GetDsdResources(const _DSDVAL_GET_DESCRIPTOR ValDescriptor[], UINT16 DescriptorLen);

    NTSTATUS EvalMethodSync(ACPI_EVAL_INPUT_BUFFER* InputBufferPtr, SIZE_T InputBufferSize, PACPI_EVAL_OUTPUT_BUFFER* OutBufferPtr);
    NTSTATUS EvalMethodSync(ULONG MethodNameUlong, PACPI_EVAL_OUTPUT_BUFFER* OutBufferPtr)
    {
        ACPI_EVAL_INPUT_BUFFER inputBuffer;
    
        RtlZeroMemory(&inputBuffer, sizeof(inputBuffer));
        inputBuffer.Signature = ACPI_EVAL_INPUT_BUFFER_SIGNATURE;
        inputBuffer.MethodNameAsUlong = MethodNameUlong; //'RPGR'; // Has to be spelled backwards because of endianity
        return EvalMethodSync(&inputBuffer, sizeof(inputBuffer), OutBufferPtr);
    }
};
