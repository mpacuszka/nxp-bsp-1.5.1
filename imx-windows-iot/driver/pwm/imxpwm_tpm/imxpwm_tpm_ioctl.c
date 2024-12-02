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

#include "precomp.h"
#pragma hdrstop
#include "imxpwm_tpm.h"
#include "imxpwm_tpm_dbg.h"


IMXPWM_NONPAGED_SEGMENT_BEGIN;

_Use_decl_annotations_
NTSTATUS ImxTpmSetDesiredPeriod (IMXPWM_DEVICE_CONTEXT* pDevContext, PWM_PERIOD DesiredPeriod) {
    NTSTATUS   ntStatus = STATUS_SUCCESS;
    PWM_PERIOD ullAtualPeriod;
    UINT64     ullModulo;
    TPM_t*     pRegs = pDevContext->pRegs;
    int i;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("Desired period: %llu", DesiredPeriod);

    NT_ASSERT((DesiredPeriod >= pDevContext->ControllerInfo.MinimumPeriod) && (DesiredPeriod <= pDevContext->ControllerInfo.MaximumPeriod));
    ullModulo = (pDevContext->ClockSourceFrequency * DesiredPeriod) / PICOSECONDS_IN_1_SECOND;
    for (i = 0; i <= TPM_PRESCALER_MAX; i++) {
        if (ullModulo <= 0xFFFF) {
            break;
        } else {
            ullModulo >>= 1;
        }
    }
    if (ullModulo <= 0xFFFF) {
        pRegs->SC.B.CMOD = 0;                         /* Disable counter */
        while (pRegs->SC.B.CMOD);
        pRegs->SC.B.PS   = i;                         /* Setprescaler    */
        pRegs->CNT.R     = 0;
        pRegs->MOD.R     = (UINT32)(ullModulo - 1);   /* Set modulus     */
        pRegs->SC.B.CMOD = 1;                         /* Enable counter  */
        ullAtualPeriod = CalculatePeriod_in_picosecs(pDevContext->ClockSourceFrequency, pRegs->SC.B.PS, pRegs->MOD.R);
        pDevContext->DesiredPeriod = DesiredPeriod;
        pDevContext->ActualPeriod = ullAtualPeriod;
        DBG_DEV_PRINT_INFO("Setting new period. (DesiredPeriod = %llups(%lluHz), ActualPeriod = %llups(%lluHz))", DesiredPeriod, PICOSECONDS_IN_1_SECOND / DesiredPeriod, ullAtualPeriod, PICOSECONDS_IN_1_SECOND / ullAtualPeriod);
    }  else {
        /* ERROR */
        ntStatus = STATUS_INTEGER_OVERFLOW;
        DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "MODULO(%llu) is not < 0xFFFE", ullModulo);
    }
    DBG_DEV_DUMP_REGS(pDevContext);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}


_Use_decl_annotations_
NTSTATUS ImxPwmIoctlPinGetActiveDutyCycle (const IMXPWM_DEVICE_CONTEXT* pDevContext, WDFREQUEST WdfRequest, ULONG PinNumber, PULONG pOutBufferSize) {
    NTSTATUS                                         ntStatus;
    PWM_PIN_GET_ACTIVE_DUTY_CYCLE_PERCENTAGE_OUTPUT* pOutputBuffer;

    DBG_DEV_METHOD_BEG();
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    if (!NT_SUCCESS(ntStatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*pOutputBuffer), (PVOID *)(&pOutputBuffer), NULL))) {
        DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfRequestRetrieveOutputBuffer(..) failed. Completing request with error.");
    } else {
        *pOutBufferSize = sizeof(*pOutputBuffer);
        pOutputBuffer->Percentage = pDevContext->Pin[PinNumber].ActiveDutyCycle;
    }
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("Percentage = %llu", pDevContext->Pin[PinNumber].ActiveDutyCycle, ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
NTSTATUS ImxPwmIoctlPinSetActiveDutyCycle (IMXPWM_DEVICE_CONTEXT* pDevContext, WDFREQUEST WdfRequest, ULONG PinNumber) {
    NTSTATUS                                        ntStatus;
    PWM_PIN_SET_ACTIVE_DUTY_CYCLE_PERCENTAGE_INPUT* pInputBuffer;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("Pin: %d", PinNumber);
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    if (!NT_SUCCESS(ntStatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*pInputBuffer), (PVOID*)(&pInputBuffer), NULL))) {
        DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfRequestRetrieveInputBuffer(..) failed.");
    } else {
        pDevContext->Pin[PinNumber].ActiveDutyCycle = pInputBuffer->Percentage;
        SetChnVal(pDevContext->pRegs, PinNumber, (UINT32)CalculateDuty_in_ticks(pDevContext->pRegs->MOD.R, pInputBuffer->Percentage));
        DBG_DEV_DUMP_REGS(pDevContext);
    }
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
NTSTATUS ImxPwmIoctlPinGetPolarity(const IMXPWM_DEVICE_CONTEXT* pDevContext, WDFREQUEST WdfRequest, ULONG PinNumber, PULONG pOutBufferSize) {
    NTSTATUS                     ntStatus;
    PWM_PIN_GET_POLARITY_OUTPUT* pOutputBuffer;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("Pin: %d", PinNumber);
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
     if (!NT_SUCCESS(ntStatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*pOutputBuffer), (PVOID*)(&pOutputBuffer), NULL))) {
         DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfRequestRetrieveOutputBuffer(..) failed. Completing request with error.");
     } else {
        *pOutBufferSize = sizeof(*pOutputBuffer);
        if (IsLowPolarity(pDevContext->pRegs, PinNumber)) {
            pOutputBuffer->Polarity = PWM_ACTIVE_LOW;
        } else {
            pOutputBuffer->Polarity = PWM_ACTIVE_HIGH;
        }
     }
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("Pin_%d polarity = %s", PinNumber, (pOutputBuffer->Polarity == PWM_ACTIVE_HIGH) ? "HIGH" : (pOutputBuffer->Polarity == PWM_ACTIVE_LOW) ? "LOW" : "UNKNOWN", ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
NTSTATUS ImxPwmIoctlPinSetPolarity (IMXPWM_DEVICE_CONTEXT* pDevContext, WDFREQUEST WdfRequest, ULONG PinNumber) {
    NTSTATUS                    ntStatus = STATUS_SUCCESS;
    PWM_PIN_SET_POLARITY_INPUT* pInputBuffer;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("Pin: %d", PinNumber);
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    do {
        if (!NT_SUCCESS(ntStatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*pInputBuffer), (PVOID*)(&pInputBuffer), NULL))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfRequestRetrieveInputBuffer(..) failed. Completing request with error.");
            break;
        }
        if ((pInputBuffer->Polarity == PWM_ACTIVE_LOW) && IsLowPolarity(pDevContext->pRegs, PinNumber)) {
            break;
        }
        if ((pInputBuffer->Polarity == PWM_ACTIVE_HIGH) && !IsLowPolarity(pDevContext->pRegs, PinNumber)) {
            break;
        }
        if (pDevContext->Pin[PinNumber].IsStarted) {
            ntStatus = STATUS_INVALID_DEVICE_REQUEST;
            break;
        }
        if (pInputBuffer->Polarity == PWM_ACTIVE_HIGH) {
            SetHighPolarity(pDevContext->pRegs, PinNumber);
        } else if (pInputBuffer->Polarity == PWM_ACTIVE_LOW) {
            SetLowPolarity(pDevContext->pRegs, PinNumber);
        }  else {
            ntStatus = STATUS_INVALID_PARAMETER;
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "Unkown polarity: %d.", pInputBuffer->Polarity);
        }
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("Pin_%d polarity = %s", PinNumber, (pInputBuffer->Polarity == PWM_ACTIVE_HIGH) ? "HIGH" : (pInputBuffer->Polarity == PWM_ACTIVE_LOW) ? "LOW" : "UNKNOWN", ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
NTSTATUS ImxPwmIoctlPinStart (IMXPWM_DEVICE_CONTEXT* pDevContext, ULONG PinNumber) {
    NTSTATUS          ntStatus = STATUS_SUCCESS;
    IMXPWM_PIN_STATE* pPinContext = &pDevContext->Pin[PinNumber];

    DBG_DEV_METHOD_BEG_WITH_PARAMS("Pin: %d", PinNumber);
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    if (!pPinContext->IsStarted) {
        pPinContext->IsStarted = TRUE;
        DBG_DEV_DUMP_REGS(pDevContext);
        EnableChn(pDevContext->pRegs, PinNumber);
        DBG_DEV_DUMP_REGS(pDevContext);
    } else {
        DBG_DEV_PRINT_INFO("Pis is already started");
    }
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
NTSTATUS ImxPwmIoctlPinStop (IMXPWM_DEVICE_CONTEXT* pDevContext, ULONG PinNumber) {
    NTSTATUS ntStatus = STATUS_SUCCESS;;
    IMXPWM_PIN_STATE* pPinContext = &pDevContext->Pin[PinNumber];

    DBG_DEV_METHOD_BEG_WITH_PARAMS("Pin: %d", PinNumber);
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    if (pPinContext->IsStarted) {
        pPinContext->IsStarted = FALSE;
        DisableChn(pDevContext->pRegs, PinNumber);
    } else {
        DBG_DEV_PRINT_INFO("Pis is already stopped");
    }
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
NTSTATUS ImxPwmIoctlPinIsStarted(IMXPWM_DEVICE_CONTEXT* pDevContext, WDFREQUEST WdfRequest, ULONG PinNumber, PULONG pOutputSize) {
    NTSTATUS                   ntStatus;
    PWM_PIN_IS_STARTED_OUTPUT* pOutputBuffer;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("Pin: %d", PinNumber);
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    do {
        if (!NT_SUCCESS(ntStatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*pOutputBuffer), (PVOID*)(&pOutputBuffer), NULL))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfRequestRetrieveOutputBuffer(..) failed. Completing request with error.");
            break;
        }
        *pOutputSize = sizeof(*pOutputBuffer);
        pOutputBuffer->IsStarted = pDevContext->Pin[PinNumber].IsStarted;
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("IsStarted = %d", pOutputBuffer->IsStarted, ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
NTSTATUS ImxPwmIoctlControllerGetInfo(const IMXPWM_DEVICE_CONTEXT* pDevContext, WDFREQUEST WdfRequest, PULONG pOutputSize) {
    NTSTATUS                        ntStatus;
    PWM_CONTROLLER_GET_INFO_OUTPUT* pOutputBuffer;

    DBG_DEV_METHOD_BEG();
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    do {
        if (!NT_SUCCESS(ntStatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*pOutputBuffer), (PVOID*)(&pOutputBuffer), NULL))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfRequestRetrieveOutputBuffer(..) failed. Completing request with error.");
            break;
        }
        *pOutputSize = sizeof(*pOutputBuffer);
        RtlCopyMemory(pOutputBuffer, &pDevContext->ControllerInfo, sizeof(*pOutputBuffer));
        DBG_DEV_PRINT_INFO(
            "Controller Info: Size: %d, PinCount = %lu, MinimumPeriod = %llups(%lluHz), MaximumPeriod = %llups(%lluHz)",
            (INT32)pOutputBuffer->Size, pOutputBuffer->PinCount,
            pOutputBuffer->MinimumPeriod, Period_in_ps_to_frequency_in_Hz(pOutputBuffer->MinimumPeriod),
            pOutputBuffer->MaximumPeriod, Period_in_ps_to_frequency_in_Hz(pOutputBuffer->MaximumPeriod));
    } while (0);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
NTSTATUS ImxPwmIoctlControllerGetActualPeriod (const IMXPWM_DEVICE_CONTEXT* pDevContext, WDFREQUEST WdfRequest, PULONG pOutputSize) {
    NTSTATUS                                 ntStatus;
    PWM_CONTROLLER_GET_ACTUAL_PERIOD_OUTPUT* pOutputBuffer;

    DBG_DEV_METHOD_BEG();
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    do {
        if (!NT_SUCCESS(ntStatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*pOutputBuffer), (PVOID *)(&pOutputBuffer), NULL))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfRequestRetrieveOutputBuffer(..) failed. Completing request with error.");
            break;
        }
        *pOutputSize = sizeof(*pOutputBuffer);
        pOutputBuffer->ActualPeriod = pDevContext->ActualPeriod;
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("ActualPeriod = % llups(% lluHz)", pOutputBuffer->ActualPeriod, Period_in_ps_to_frequency_in_Hz(pOutputBuffer->ActualPeriod), ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
NTSTATUS ImxPwmIoctlControllerSetDesiredPeriod (IMXPWM_DEVICE_CONTEXT* pDevContext, WDFREQUEST WdfRequest, PULONG pOutputSize) {
    NTSTATUS                                  ntStatus;
    PWM_CONTROLLER_SET_DESIRED_PERIOD_INPUT*  pInputBuffer;
    PWM_CONTROLLER_SET_DESIRED_PERIOD_OUTPUT* pOutputBuffer;

    DBG_DEV_METHOD_BEG();
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    do {
        if (!NT_SUCCESS(ntStatus = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*pInputBuffer), (PVOID *)(&pInputBuffer), NULL))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfRequestRetrieveInputBuffer(..) failed. Completing request with error.");
            break;
        }
        if (!NT_SUCCESS(ntStatus = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*pOutputBuffer), (PVOID *)(&pOutputBuffer), NULL))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfRequestRetrieveOutputBuffer(..) failed. Completing request with error.");
            break;
        }
        if ((pInputBuffer->DesiredPeriod < pDevContext->ControllerInfo.MinimumPeriod) ||
            (pInputBuffer->DesiredPeriod > pDevContext->ControllerInfo.MaximumPeriod)) {
            ntStatus = STATUS_INVALID_PARAMETER;
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "DesiredPeriod %llu out of controller limits. (MinimumPeriod = %llu, MaximumPeriod = %llu)",
                                        pInputBuffer->DesiredPeriod, pDevContext->ControllerInfo.MinimumPeriod, pDevContext->ControllerInfo.MaximumPeriod);
            break;
        }
        if (!NT_SUCCESS(ntStatus = ImxTpmSetDesiredPeriod(pDevContext, pInputBuffer->DesiredPeriod))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "ImxTpmSetDesiredPeriod(..) failed. Completing request with error.");
            break;
        }
        *pOutputSize = sizeof(*pOutputBuffer);
        pOutputBuffer->ActualPeriod = pDevContext->ActualPeriod;
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("ActualPeriod = %llups(%lluHz)", pDevContext->ActualPeriod, Period_in_ps_to_frequency_in_Hz(pDevContext->ActualPeriod), ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
VOID ImxTpmEvtIoDeviceControl (WDFQUEUE WdfQueue, WDFREQUEST WdfRequest, size_t OutputBufferLength, size_t InputBufferLength, ULONG IoControlCode) {
    WDFDEVICE                   wdfDevice            = WdfIoQueueGetDevice(WdfQueue);
    WDFFILEOBJECT               wdfFileObject        = WdfRequestGetFileObject(WdfRequest);
    IMXPWM_DEVICE_CONTEXT*      pDevContext          = ImxPwmGetDeviceContext(wdfDevice);
    IMXPWM_FILE_OBJECT_CONTEXT* pFileObjectContext = ImxPwmGetFileObjectContext(wdfFileObject);
    ULONG                       uOutBufferSize       = 0;
    NTSTATUS                    ntStatus             = STATUS_SUCCESS;
    ULONG                       PinNumber            = pFileObjectContext->PinNumber;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("%s PinNum %d", Dbg_GetIOCTLName(IoControlCode), PinNumber);
    IMXPWM_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    UNREFERENCED_PARAMETER(OutputBufferLength);
    UNREFERENCED_PARAMETER(InputBufferLength);
    if (pFileObjectContext->IsPinInterface) {
        /* Is Pin Interface */
        NT_ASSERT(PinNumber < pDevContext->ControllerInfo.PinCount);
        switch (IoControlCode) {
        case IOCTL_PWM_CONTROLLER_GET_INFO:
        case IOCTL_PWM_CONTROLLER_GET_ACTUAL_PERIOD:
        case IOCTL_PWM_CONTROLLER_SET_DESIRED_PERIOD:
            ntStatus = STATUS_INVALID_DEVICE_REQUEST;
            DBG_DEV_PRINT_INFO("Controller %s directed to a pin.", Dbg_GetIOCTLName(IoControlCode));
            break;
        case IOCTL_PWM_PIN_GET_POLARITY:
            ntStatus = ImxPwmIoctlPinGetPolarity(pDevContext, WdfRequest, PinNumber, &uOutBufferSize);
            break;
        case IOCTL_PWM_PIN_SET_POLARITY:
            ntStatus = ImxPwmIoctlPinSetPolarity(pDevContext, WdfRequest, PinNumber);
            break;
        case IOCTL_PWM_PIN_GET_ACTIVE_DUTY_CYCLE_PERCENTAGE:
            ntStatus = ImxPwmIoctlPinGetActiveDutyCycle(pDevContext, WdfRequest, PinNumber, &uOutBufferSize);
            break;
        case IOCTL_PWM_PIN_SET_ACTIVE_DUTY_CYCLE_PERCENTAGE:
            ntStatus = ImxPwmIoctlPinSetActiveDutyCycle(pDevContext, WdfRequest, PinNumber);
            break;
        case IOCTL_PWM_PIN_START:
            ntStatus = ImxPwmIoctlPinStart(pDevContext, PinNumber);
            break;
        case IOCTL_PWM_PIN_STOP:
            ntStatus = ImxPwmIoctlPinStop(pDevContext, PinNumber);
            break;
        case IOCTL_PWM_PIN_IS_STARTED:
            ntStatus = ImxPwmIoctlPinIsStarted(pDevContext, WdfRequest, PinNumber, &uOutBufferSize);
            break;
        default:
            ntStatus = STATUS_NOT_SUPPORTED;
            DBG_DEV_PRINT_INFO("IOCTL 0x%08X not supported.", IoControlCode);
        }
    } else {
        /* Is Controller Interface */
        switch (IoControlCode) {
        case IOCTL_PWM_CONTROLLER_GET_INFO:
            ntStatus = ImxPwmIoctlControllerGetInfo(pDevContext, WdfRequest, &uOutBufferSize);
            break;
        case IOCTL_PWM_CONTROLLER_GET_ACTUAL_PERIOD:
            ntStatus = ImxPwmIoctlControllerGetActualPeriod(pDevContext, WdfRequest, &uOutBufferSize);
            break;
        case IOCTL_PWM_CONTROLLER_SET_DESIRED_PERIOD:
            ntStatus = ImxPwmIoctlControllerSetDesiredPeriod(pDevContext, WdfRequest, &uOutBufferSize);
            break;
        case IOCTL_PWM_PIN_GET_POLARITY:
        case IOCTL_PWM_PIN_SET_POLARITY:
        case IOCTL_PWM_PIN_GET_ACTIVE_DUTY_CYCLE_PERCENTAGE:
        case IOCTL_PWM_PIN_SET_ACTIVE_DUTY_CYCLE_PERCENTAGE:
        case IOCTL_PWM_PIN_START:
        case IOCTL_PWM_PIN_STOP:
        case IOCTL_PWM_PIN_IS_STARTED:
            ntStatus = STATUS_INVALID_DEVICE_REQUEST;
            DBG_DEV_PRINT_INFO("Pin %s directed to a controller.", Dbg_GetIOCTLName(IoControlCode));
            break;
        default:
            ntStatus = STATUS_NOT_SUPPORTED;
            DBG_DEV_PRINT_INFO("IOCTL 0x%08X not supported.", IoControlCode);
        }
    }
    if (uOutBufferSize != 0) {
        WdfRequestCompleteWithInformation(WdfRequest, ntStatus, uOutBufferSize);
        DBG_DEV_PRINT_INFO("Calling WdfRequestComplete(uOutBufferSize: %d) [0x%.8X]", uOutBufferSize, ntStatus);
    } else {
        DBG_DEV_PRINT_INFO("Calling WdfRequestComplete() [0x%.8X]", ntStatus);
        WdfRequestComplete(WdfRequest, ntStatus);
    }
    DBG_DEV_METHOD_END_WITH_PARAMS("%s", Dbg_GetIOCTLName(IoControlCode));
    return;
}

IMXPWM_NONPAGED_SEGMENT_END;