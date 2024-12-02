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

/* 

Module Name:

    imxlpi2ccontroller.h

Abstract:

    This module contains the controller-specific function
    declarations.

Environment:

    kernel-mode only

*/

#ifndef _IMXI2CCONTROLLER_H_
#define _IMXI2CCONTROLLER_H_

//
// Controller specific function prototypes.
//


ULONG FORCEINLINE GetTxFifoCount(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr)
{
    return ((DeviceCtxPtr->RegistersPtr->MFSR & IMX_LPI2C_MFSR_TXCOUNT_MASK) >> IMX_LPI2C_MFSR_TXCOUNT_SHIFT);
}

ULONG FORCEINLINE GetRxFifoCount(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr)
{
    return ((DeviceCtxPtr->RegistersPtr->MFSR & IMX_LPI2C_MFSR_RXCOUNT_MASK) >> IMX_LPI2C_MFSR_RXCOUNT_SHIFT);
}

VOID ControllerClearFifos(_In_ PDEVICE_CONTEXT DeviceCtxPtr, _In_ PPBC_REQUEST RequestPtr);

VOID ControllerRequestForStopGen(_In_ PDEVICE_CONTEXT DeviceCtxPtr, _In_ PPBC_REQUEST RequestPtr);

VOID ControllerCancel(_In_ PDEVICE_CONTEXT DeviceCtxPtr);

VOID ControllerPollingMain(_In_ WDFWORKITEM WorkItem);

NTSTATUS ControllerInitialize(_In_ PDEVICE_CONTEXT DeviceCtxPtr);

NTSTATUS ControllerUninitialize(_In_ PDEVICE_CONTEXT DeviceCtxPtr);

NTSTATUS ResetController(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr);

NTSTATUS FindControllerClockSettings(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_ ULONG ClockFrequencyHz);

VOID SetControllerClockDiv(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr);

NTSTATUS ControllerGenerateStart(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  PPBC_REQUEST RequestPtr);

NTSTATUS ControllerGenerateRepeatedStart(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  PPBC_REQUEST RequestPtr);

NTSTATUS ControllerGenerateStop(_In_ PDEVICE_CONTEXT DeviceCtxPtr);


#if (defined(DBG) || defined(DEBUG))
VOID
ControllerDumpRegisters(_In_ PDEVICE_CONTEXT DeviceCtxPtr);

#endif
VOID
ControllerConfigureForTransfer(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_ PPBC_REQUEST pRequest);

NTSTATUS
ControllerTransferDataMultp(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_ PPBC_REQUEST pRequest);

VOID
ControllerCompleteTransfer(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_ PPBC_REQUEST pRequest,
    _In_ BOOLEAN AbortSequence);
VOID
ControllerEnableInterrupts(
    _In_  PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  ULONG InterruptMask);

VOID
ControllerDisableInterrupts(
    _In_  PDEVICE_CONTEXT DeviceCtxPtr);

ULONG
ControllerGetInterruptStatus(
    _In_  PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  ULONG InterruptMask);

VOID
ControllerAcknowledgeInterrupts(
    _In_  PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  ULONG InterruptMask);

VOID
ControllerProcessInterrupts(
    _In_  PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  PPBC_REQUEST pRequest,
    _In_  ULONG InterruptStatus);

#endif // of _IMXI2CCONTROLLER_H_

