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

#ifndef _IMXPWM_TPM_H_
#define _IMXPWM_TPM_H_

#include "imx_acpi_utils.h"
#include "imxtpm_io_map.h"

/* Macros to be used for proper PAGED / NON - PAGED code placement */
#define IMXPWM_NONPAGED_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    //__pragma(code_seg(.text))

#define IMXPWM_NONPAGED_SEGMENT_END \
    __pragma(code_seg(pop))

#define IMXPWM_PAGED_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    __pragma(code_seg("PAGE"))

#define IMXPWM_PAGED_SEGMENT_END \
    __pragma(code_seg(pop))

#define IMXPWM_INIT_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    __pragma(code_seg("INIT"))

#define IMXPWM_INIT_SEGMENT_END \
    __pragma(code_seg(pop))

#define IMXPWM_ASSERT_MAX_IRQL(Irql) NT_ASSERT(KeGetCurrentIrql() <= (Irql))

#define TPM_MEM_TAG_DRV   ((ULONG)'MPTi')
#define TPM_MEM_TAG_ACPI  ((ULONG)'MPTa')

#define MAX_PWM_SCHEMATIC_NAME_LENGHT  64
#define MAX_PWM_PINS                   4

#define TPM_PRESCALER_MAX              7
#define TPM_PRESCALER_MIN              0
				       
#define TPM_MODULO_MIN                 1
#define TPM_MODULO_MAX                 65534
				       
#define PICOSECONDS_IN_1_SECOND        1000000000000

#define Period_in_ps_to_frequency_in_Hz(_Period_in_picoseconds_)  ((UINT64)((PICOSECONDS_IN_1_SECOND + (_Period_in_picoseconds_ / 2)) / _Period_in_picoseconds_))
#define CalculatePeriod_in_picosecs(_ClockSourceFrequency_, _ClockPrescaler_, _Modulo_) ((((UINT64)PICOSECONDS_IN_1_SECOND * ((UINT64)1 << (_ClockPrescaler_)) * ((UINT64)(_Modulo_) + 1)) + ((_ClockSourceFrequency_) / 2 )) / (_ClockSourceFrequency_))
#define CalculateDuty_in_ticks(_Modulo_, _Duty_) (((((UINT64)(_Modulo_) + 1) * ((_Duty_) >> 32)) + (PWM_PERCENTAGE_MAX >> 33)) / (PWM_PERCENTAGE_MAX >> 32))

#define EnableTpmCounter( _pRegs_)            (_pRegs_->SC.B.CMOD = 1)                              /* Enable TPM counter                    */
#define DisableTpmCounter(_pRegs_)            (_pRegs_->SC.B.CMOD = 0)                              /* Disable TPM counter                   */
#define EnableChn(_pRegs_, _ChnNum_)          (((UINT32 *)(&_pRegs_->C0SC))[_ChnNum_ << 1] = 0x28)  /* Configure channel as edge-aligned PWM */
#define DisableChn(_pRegs_, _ChnNum_)         (((UINT32 *)(&_pRegs_->C0SC))[_ChnNum_ << 1] = 0x00)  /* Disable channel                       */
#define SetHighPolarity(_pRegs_, _ChnNum_)    (_pRegs_->POL.R &= ~(1 << _ChnNum_))                  /* PWM_ACTIVE_HIGH                       */
#define SetLowPolarity(_pRegs_, _ChnNum_)     (_pRegs_->POL.R |= (1 << _ChnNum_))                   /* PWM_ACTIVE_LOW                        */
#define IsLowPolarity(_pRegs_, _ChnNum_)      (_pRegs_->POL.R & (1 << PinNumber))

#define GetChnVal(_pRegs_, _ChnNum_)          (((UINT32 *)(&_pRegs_->C0V))[_ChnNum_ << 1])          /* Return channel value                  */
#define SetChnVal(_pRegs_, _ChnNum_, _Val_)   (((UINT32 *)(&_pRegs_->C0V))[_ChnNum_ << 1] = _Val_)  /* Set channel value                     */

/* Pin context structure */
typedef struct IMXPWM_PIN_STATE_s {
    PWM_POLARITY   Polarity;
    PWM_PERCENTAGE ActiveDutyCycle;
    BOOLEAN        IsStarted;
    BOOLEAN        IsOpenForWrite;    
    WDFWAITLOCK    Lock;                 /* A lock to protect IsOpenForWrite during file create / close callbacks */
} IMXPWM_PIN_STATE;

/* Device context structure */
typedef struct IMXPWM_DEVICE_CONTEXT_s {
    WDFDEVICE                  WdfDevice;
    TPM_t*                     pRegs;                              /* TPM registers vitrual adress                         */
    ULONG                      ClockSourceFrequency;               /* TPM device root clock frequency [HZ]                 */
    PWM_PERIOD                 DesiredPeriod;
    PWM_PERIOD                 ActualPeriod;
    BOOLEAN                    IsControllerOpenForWrite;
    BOOLEAN                    IsActive;                           
    PWM_CONTROLLER_INFO        ControllerInfo;
    WDFWAITLOCK                ControllerLock;                     /* A lock to protect the controller IsOpenForWrite during file create / close callbacks */
    IMXPWM_PIN_STATE           Pin[MAX_PWM_PINS];
    IMX_ACPI_UTILS_DEV_CONTEXT pACPI_UtilsContext;
    WDFSTRING                  DeviceInterfaceSymlinkName;
    UNICODE_STRING             DeviceInterfaceSymlinkNameWsz;
    ULONG                      RegsSize;                           /* TPM registers size (required by MmUnmapIoSpace())    */
} IMXPWM_DEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(IMXPWM_DEVICE_CONTEXT, ImxPwmGetDeviceContext);

typedef struct IMXPWM_FILE_OBJECT_CONTEXT_s {
    BOOLEAN IsOpenForWrite;
    BOOLEAN IsPinInterface;
    ULONG   PinNumber;
} IMXPWM_FILE_OBJECT_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(IMXPWM_FILE_OBJECT_CONTEXT, ImxPwmGetFileObjectContext);


_IRQL_requires_same_                NTSTATUS ImxTpmSetDesiredPeriod                (_In_       IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ PWM_PERIOD DesiredPeriod);
_IRQL_requires_max_(DISPATCH_LEVEL) NTSTATUS ImxPwmIoctlControllerGetInfo          (_In_ const IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ WDFREQUEST WdfRequest, _Out_ PULONG pOutputSize);
_IRQL_requires_max_(DISPATCH_LEVEL) NTSTATUS ImxPwmIoctlControllerGetActualPeriod  (_In_ const IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ WDFREQUEST WdfRequest, _Out_ PULONG pOutputSize);
_IRQL_requires_max_(DISPATCH_LEVEL) NTSTATUS ImxPwmIoctlControllerSetDesiredPeriod (_In_       IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ WDFREQUEST WdfRequest, _Out_ PULONG pOutputSize);
_IRQL_requires_max_(DISPATCH_LEVEL) NTSTATUS ImxPwmIoctlPinGetActiveDutyCycle      (_In_ const IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ WDFREQUEST WdfRequest, _In_ ULONG PinNumber, _Out_ PULONG pOutputSize);
_IRQL_requires_max_(DISPATCH_LEVEL) NTSTATUS ImxPwmIoctlPinSetActiveDutyCycle      (_In_       IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ WDFREQUEST WdfRequest, _In_ ULONG PinNumber);
_IRQL_requires_max_(DISPATCH_LEVEL) NTSTATUS ImxPwmIoctlPinGetPolarity             (_In_ const IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ WDFREQUEST WdfRequest, _In_ ULONG PinNumber, _Out_ PULONG pOutputSize);
_IRQL_requires_max_(DISPATCH_LEVEL) NTSTATUS ImxPwmIoctlPinSetPolarity             (_In_       IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ WDFREQUEST WdfRequest, _In_ ULONG PinNumber);
_IRQL_requires_max_(DISPATCH_LEVEL) NTSTATUS ImxPwmIoctlPinStart                   (_In_       IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ ULONG PinNumber);
_IRQL_requires_max_(DISPATCH_LEVEL) NTSTATUS ImxPwmIoctlPinStop                    (_In_       IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ ULONG PinNumber);
_IRQL_requires_max_(DISPATCH_LEVEL) NTSTATUS ImxPwmIoctlPinIsStarted               (_In_       IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ WDFREQUEST WdfRequest, _In_ ULONG PinNumber, _Out_ PULONG pOutputSize);

EXTERN_C_START

EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL ImxTpmEvtIoDeviceControl;
EVT_WDF_DEVICE_FILE_CREATE         ImxTpmEvtDeviceFileCreate;
EVT_WDF_FILE_CLOSE                 ImxTpmEvtFileClose;
EVT_WDF_DEVICE_D0_ENTRY            ImxTpmEvtDeviceD0Entry;
EVT_WDF_DEVICE_PREPARE_HARDWARE    ImxTpmEvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE    ImxTpmEvtDeviceReleaseHardware;
EVT_WDF_DRIVER_DEVICE_ADD          ImxTpmEvtDeviceAdd;
EVT_WDF_DRIVER_UNLOAD              ImxTpmEvtDriverUnload;
DRIVER_INITIALIZE                  DriverEntry;

EXTERN_C_END

#endif // _IMXPWM_TPM_H_