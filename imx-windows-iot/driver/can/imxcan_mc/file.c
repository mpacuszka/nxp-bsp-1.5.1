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

#include "precomp.h"
#pragma hdrstop

#include "imxcanhw.h"
#include "imxcan.h"

NTSTATUS CanParsePinPath(UNICODE_STRING* PinPath, ULONG* PinNumber);
VOID ImxCanCreateRequestGetAccess(WDFREQUEST WdfRequest, ACCESS_MASK* DesiredAccessPtr, ULONG* ShareAccessPtr);

IMXCAN_PAGED_SEGMENT_BEGIN;

VOID ImxCanEvtDeviceFileCreate(
    WDFDEVICE WdfDevice,
    WDFREQUEST WdfRequest,
    WDFFILEOBJECT WdfFileObject
)
{
    PAGED_CODE();

    UNICODE_STRING* filenamePtr = WdfFileObjectGetFileName(WdfFileObject);
    PDEV_CONTEXT deviceContextPtr = ImxCanGetDeviceContext(WdfDevice);
    NTSTATUS status;
    ULONG pinNumber = ULONG_MAX;

    /* Parse and validate the filename associated with the file object. */
    BOOLEAN isPinInterface;
    if (filenamePtr == NULL) {
        WdfRequestComplete(WdfRequest, STATUS_INVALID_DEVICE_REQUEST);
        return;
    }
    else if (filenamePtr->Length > 0) {
        /* A non-empty filename means to open a pin under the controller namespace. */
        status = CanParsePinPath(filenamePtr, &pinNumber);
        if (!NT_SUCCESS(status)) {
            WdfRequestComplete(WdfRequest, status);
            return;
        }
        NT_ASSERT(deviceContextPtr->ControllerInfo.PinCount == 1);
        if (pinNumber >= deviceContextPtr->ControllerInfo.PinCount) {
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Requested pin number out of bounds. (pinNumber = %lu)\n", pinNumber));
            WdfRequestComplete(WdfRequest, STATUS_NO_SUCH_FILE);
            return;
        }
        isPinInterface =TRUE;
    }
    else {
        /* An empty filename means that the create is against the root controller. */
        isPinInterface = FALSE;
    }

    ACCESS_MASK desiredAccess;
    ULONG shareAccess;
    ImxCanCreateRequestGetAccess(WdfRequest, &desiredAccess, &shareAccess);

    /* Verify request desired access. */
    const BOOLEAN hasWriteAccess = ((desiredAccess & FILE_WRITE_DATA) != 0);

    if (isPinInterface) {
        IMXCAN_PIN_STATE *pinPtr = &deviceContextPtr->Pin;

        WdfWaitLockAcquire(pinPtr->Lock, NULL);
        if (hasWriteAccess) {
            if (pinPtr->IsOpenForWrite) {
                WdfWaitLockRelease(pinPtr->Lock);
                KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Pin access denied.\n"));
                WdfRequestComplete(WdfRequest, STATUS_SHARING_VIOLATION);
                return;
            }
            pinPtr->IsOpenForWrite = TRUE;
        }
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Pin Opened. (IsOpenForWrite = %!bool!)\n", pinPtr->IsOpenForWrite));
        WdfWaitLockRelease(pinPtr->Lock);

    }
    else {
        WdfWaitLockAcquire(deviceContextPtr->ControllerLock, NULL);
        if (hasWriteAccess) {
            if (deviceContextPtr->IsControllerOpenForWrite) {
                WdfWaitLockRelease(deviceContextPtr->ControllerLock);
                KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Controller access denied.\n"));
                WdfRequestComplete(WdfRequest, STATUS_SHARING_VIOLATION);
                return;
            }
            deviceContextPtr->IsControllerOpenForWrite = TRUE;
        }
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Controller Opened. (IsControllerOpenForWrite = %!bool!)\n", deviceContextPtr->IsControllerOpenForWrite));
        WdfWaitLockRelease(deviceContextPtr->ControllerLock);
    }

    /* Allocate and fill a file object context. */
    IMXCAN_FILE_OBJECT_CONTEXT *fileObjectContextPtr;
    {
        WDF_OBJECT_ATTRIBUTES wdfObjectAttributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(
            &wdfObjectAttributes,
            IMXCAN_FILE_OBJECT_CONTEXT);

        void* contextPtr;
        status = WdfObjectAllocateContext(
            WdfFileObject,
            &wdfObjectAttributes,
            &contextPtr);
        if (!NT_SUCCESS(status)) {
            KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfObjectAllocateContext(...) failed. (status = %!STATUS!)\n", status));
            WdfRequestComplete(WdfRequest, status);
            return;
        }
        fileObjectContextPtr = (IMXCAN_FILE_OBJECT_CONTEXT*)(contextPtr);
        NT_ASSERT(fileObjectContextPtr != NULL);
        fileObjectContextPtr->IsPinInterface = isPinInterface;
        fileObjectContextPtr->IsOpenForWrite = hasWriteAccess;
    }
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

VOID ImxCanEvtFileClose(
    WDFFILEOBJECT WdfFileObject
)
{
    PAGED_CODE();

    WDFDEVICE wdfDevice = WdfFileObjectGetDevice(WdfFileObject);
    PDEV_CONTEXT deviceContextPtr = ImxCanGetDeviceContext(wdfDevice);
    IMXCAN_FILE_OBJECT_CONTEXT* fileObjectContextPtr = ImxCanGetFileObjectContext(WdfFileObject);

    if (fileObjectContextPtr->IsPinInterface) {
        WdfWaitLockAcquire(deviceContextPtr->Pin.Lock, NULL);
        if (fileObjectContextPtr->IsOpenForWrite) {
            NTSTATUS status = ImxCanResetPinDefaults(deviceContextPtr);
            if (!NT_SUCCESS(status)) {
                KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "ImxCanResetPinDefaults(...) failed. (status = %!STATUS!)\n", status));
            }
            NT_ASSERT(deviceContextPtr->Pin.IsOpenForWrite);
            deviceContextPtr->Pin.IsOpenForWrite = FALSE;
        }
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Pin Closed. (IsOpenForWrite = %lu)\n", (deviceContextPtr->Pin.IsOpenForWrite ? 1 : 0)));
        WdfWaitLockRelease(deviceContextPtr->Pin.Lock);
    }
    else {

        WdfWaitLockAcquire(deviceContextPtr->ControllerLock, NULL);

        if (fileObjectContextPtr->IsOpenForWrite) {
            NTSTATUS status = ImxCanResetControllerDefaults(deviceContextPtr);
            if (!NT_SUCCESS(status)) {
                KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "ImxCanResetControllerDefaults(...) failed. (status = %!STATUS!)\n", status));
            }
            NT_ASSERT(deviceContextPtr->IsControllerOpenForWrite);
            deviceContextPtr->IsControllerOpenForWrite = FALSE;
        }
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "Controller Closed. (IsControllerOpenForWrite = %lu)\n", (deviceContextPtr->IsControllerOpenForWrite ? 1 : 0)));
        WdfWaitLockRelease(deviceContextPtr->ControllerLock);
    }
}

/*

Routine Description:

    This routine parses a pin path under the CAN controller namespace to validate
    its format and extract the pin number where the pin path string should be in
    the form: "\<PinNumber>".

Arguments:

    PinPath [in] - Supplies a pointer to Unicode string.

    PinNumber [out, optional] - Supplies a pointer to variable which receives pin number.

Return Value:

    STATUS_SUCCESS
        Extracted pin path successfully from supplied pin path.

    STATUS_INVALID_PARAMETER
        The supplied pin path pointer is invalid or its Unicode string is
        invalid.

    STATUS_NO_SUCH_FILE
        The supplied pin path does not constitute a valid pin path. The pin path
        should be in the form: "\<PinNumber>".

*/
NTSTATUS CanParsePinPath(
    _In_ UNICODE_STRING* PinPath,
    _Out_opt_ ULONG* PinNumber
)
{
    PAGED_CODE();

    NTSTATUS status;
    ULONG pinNumber;
    ULONG nChars;
    const WCHAR* currentChar;
    const WCHAR* strEnd;

    if (PinPath == NULL) {
        return STATUS_INVALID_PARAMETER;
    }

    status = RtlUnicodeStringValidate(PinPath);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    nChars = PinPath->Length / sizeof(WCHAR);

    
    /* Fast fail if the buffer isn't big enough to hold at least one backslash
       and a single decimal digit, or if the first character isn't a backslash. */
    if ((nChars < 2) || (*PinPath->Buffer != L'\\')) {
        return STATUS_NO_SUCH_FILE;
    }

    currentChar = PinPath->Buffer + 1;
    strEnd = PinPath->Buffer + nChars;
    pinNumber = 0;

    while (currentChar < strEnd) {
        if ((*currentChar < L'0') || (*currentChar > L'9')) {
            return STATUS_NO_SUCH_FILE;
        }

        pinNumber *= 10;
        pinNumber += (*currentChar - L'0');
        ++currentChar;
    }

    if (PinNumber != NULL) {
        *PinNumber = pinNumber;
    }

    return STATUS_SUCCESS;
}

VOID ImxCanCreateRequestGetAccess(
    _In_ WDFREQUEST WdfRequest,
    _Out_ ACCESS_MASK* DesiredAccessPtr,
    _Out_ ULONG* ShareAccessPtr
)
{
    PAGED_CODE();
    NT_ASSERT(ARGUMENT_PRESENT(DesiredAccessPtr));
    NT_ASSERT(ARGUMENT_PRESENT(ShareAccessPtr));

    WDF_REQUEST_PARAMETERS wdfRequestParameters;
    WDF_REQUEST_PARAMETERS_INIT(&wdfRequestParameters);
    WdfRequestGetParameters(WdfRequest, &wdfRequestParameters);

    NT_ASSERTMSG("Expected create request", wdfRequestParameters.Type == WdfRequestTypeCreate);

    *DesiredAccessPtr = wdfRequestParameters.Parameters.Create.SecurityContext->DesiredAccess;
    *ShareAccessPtr = wdfRequestParameters.Parameters.Create.ShareAccess;
}

NTSTATUS ImxCanResetPinDefaults(
    PDEV_CONTEXT DeviceContextPtr
)
{
    PAGED_CODE();
    IMXCAN_PIN_STATE* pinPtr = &DeviceContextPtr->Pin;
    volatile IMXCAN_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;

    if (pinPtr->IsStarted) {
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FlexCAN: Deinit()   \n"));
        FLEXCAN_Deinit(registersPtr);
    }
    return STATUS_SUCCESS;
}

NTSTATUS ImxCanResetControllerDefaults(
    PDEV_CONTEXT DeviceContextPtr
)
{
    UNREFERENCED_PARAMETER(DeviceContextPtr);
    PAGED_CODE();

    KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "FlexCAN: ImxCanResetControllerDefaults()   \n"));

    return STATUS_SUCCESS;
}

IMXCAN_PAGED_SEGMENT_END;