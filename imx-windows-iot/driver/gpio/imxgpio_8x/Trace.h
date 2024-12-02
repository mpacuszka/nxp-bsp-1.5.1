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

#ifndef trace_H_
#define trace_H_

#pragma once

#ifdef DBG

#ifndef DBG_MSG_DRV_PREFIX
#define DBG_MSG_DRV_PREFIX "GPIO"
#endif

#define ToString_SetPinOpMode(_OpMpde) (\
    (_OpMpde == IMX_CONNECT_PIN_SET_VALUES)?  "Set":         \
    (_OpMpde == IMX_CONNECT_PIN_CHECK_OLNY)?  "Check":       \
    (_OpMpde == IMX_CONNECT_PIN_SET_DEFAULT)? "SetDefault":  \
                                              "Unknown")
#define ToString_AcpiPullCfg(_ACPI_CFG_VAL) (\
    (_ACPI_CFG_VAL == GPIO_PIN_PULL_CONFIGURATION_DEFAULT)?  "Default":  \
    (_ACPI_CFG_VAL == GPIO_PIN_PULL_CONFIGURATION_PULLUP)?   "PullUp":   \
    (_ACPI_CFG_VAL == GPIO_PIN_PULL_CONFIGURATION_PULLDOWN)? "PullDown": \
    (_ACPI_CFG_VAL == GPIO_PIN_PULL_CONFIGURATION_NONE)?     "PullNone": \
                                                             "Unknown")

#define ToString_ConnectioMode(_CONN_MODE) (\
    (_CONN_MODE == ConnectModeInvalid)? "Invalid": \
    (_CONN_MODE == ConnectModeInput)?   "Input":   \
    (_CONN_MODE == ConnectModeOutput)?  "Output":  \
                                        "Unknown")

#define ToString_WdfPowerDeviceState(_WDF_PWR_DEV_STATE) (\
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceInvalid)?                "Invalid":                \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceD0)?                     "Do":                     \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceD1)?                     "D1":                     \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceD2)?                     "D2":                     \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceD3)?                     "D3":                     \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceD3Final)?                "D3Final":                \
    (_WDF_PWR_DEV_STATE == WdfPowerDevicePrepareForHibernation)?  "PrepareForHibernation":  \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceMaximum)?                "Max":                    \
                                                                  "Unknown")

#define ToString_kInterruptMode(_WDF_INT_MODE) (\
    (_WDF_INT_MODE == LevelSensitive)?  "LevelSensitive": \
    (_WDF_INT_MODE == Latched)?         "Latched":        \
                                        "Unknown")


#define ToString_kInterruptPolarity(_WDF_INT_POLARITY) (\
    (_WDF_INT_POLARITY == InterruptPolarityUnknown)?  "PolarityUnknown":           \
    (_WDF_INT_POLARITY == InterruptActiveHigh)?       "RisingEdge or ActiveHigh":  \
    (_WDF_INT_POLARITY == InterruptActiveLow)?        "FollingEdge or ActiveLow":  \
                                                      "Unknown")

extern LARGE_INTEGER        DriverStartTime;

static inline LONG GetTime() {
    LARGE_INTEGER  CurrentSystemTime;
    KeQuerySystemTimePrecise(&CurrentSystemTime);
    return (LONG)((CurrentSystemTime.QuadPart - DriverStartTime.QuadPart) / 10000);
}

EXTERN_C_START
EXTERN_C_END

#define DBG_PRINT_ERROR(_format_str_,...)                           DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:ERROR %s() "_format_str_"\n"          ,GetTime() ,KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_PRINT_ERROR_WITH_STATUS(_status_,_format_str_,...)      DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:ERROR %s() "_format_str_" [0x%.8X]\n" ,GetTime() ,KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__,_status_)

//#define DBG_DEV
#ifdef DBG_DEV
#define DBG_DEV_METHOD_BEG()                                                 DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s()\n"                         ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__)
#define DBG_DEV_METHOD_BEG_WITH_PARAMS(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s("_format_str_")\n"           ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_DEV_METHOD_END()                                                 DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s()\n"                         ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__)
#define DBG_DEV_METHOD_END_WITH_PARAMS(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_")\n"           ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_DEV_METHOD_END_WITH_STATUS(_status_)                             DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X]\n"                ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,_status_)
#define DBG_DEV_METHOD_END_WITH_STATUS_AND_PARAMS(_status_,_format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X] "_format_str_"\n" ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,_status_,__VA_ARGS__)
#define DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS(_format_str_,...)          DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_") [0x%.8X] \n" ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_DEV_PRINT_WARNING(_format_str_,...)                              DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s() "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_DEV_PRINT_VERBOSE(_format_str_,...)                              DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s() "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_DEV_PRINT_INFO(_format_str_,...)                                 DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s() "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
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


/* ************************************************************************************ */
/* GPIO pin interrupt debug messages                                                    */
/* ************************************************************************************ */

/* Uncomment next line to see GPIO pin interrupt debug messages */
//#define DBG_DUMP_INTERRUPT
#ifdef DBG_DUMP_INTERRUPT
#define DBG_DUMP_ENABLE_INTERRUPT_PARAMETERS(_pEnParams)   \
DBG_DEV_PRINT_INFO("\n"                                    \
    "EnableInterruptParameters:\n"                         \
    "  BankId:              %d\n"                          \
    "  PinNumber:           %d\n"                          \
    "  Flags:               %d\n"                          \
    "  OS_IntMode:          %s\n"                          \
    "  OS_IntPolarity:      %s\n"                          \
    "  PullConfiguration:   %s\n"                          \
    "  DebounceTimeout[us]: %d\n"                          \
    "  VendorData:          0x%016p%\n"                    \
    "  VendorDataLength:    %d",                           \
    _pEnParams->BankId,                                    \
    _pEnParams->PinNumber,                                 \
    _pEnParams->Flags.AsULONG,                             \
    ToString_kInterruptMode(_pEnParams->InterruptMode),    \
    ToString_kInterruptPolarity(_pEnParams->Polarity),     \
    ToString_AcpiPullCfg(_pEnParams->PullConfiguration),   \
    _pEnParams->DebounceTimeout * 100,                     \
    _pEnParams->VendorData, _pEnParams->VendorDataLength);

#define DBG_DUMP_DISABLE_INTERRUPT_PARAMETERS(_pDisParams) \
DBG_DEV_PRINT_INFO("\n"                                    \
    "DisableInterruptParameters:\n"                        \
    "  BankId:              %d\n"                          \
    "  PinNumber:           %d\n"                          \
    "  Flags:               %d",                           \
    _pDisParams->BankId,                                   \
    _pDisParams->PinNumber,                                \
    _pDisParams->Flags.AsULONG)

#define DBG_DUMP_MASK_INTERRUPT_PARAMETERS(_pMaskParams)   \
DBG_DEV_PRINT_INFO("\n"                                    \
    "MaskInterruptsParameters:\n"                          \
    "  BankId:              %d\n"                          \
    "  PinMask:             0x%016llX\n"                   \
    "  Flags:               %llu",                         \
    _pMaskParams->BankId,                                  \
    _pMaskParams->PinMask,                                 \
    _pMaskParams->FailedMask)


#define DBG_DUMP_QUERY_ACTIVE_INTERRUPTS_PARAMETERS(_pQueryActIntParams)   \
DBG_DEV_PRINT_INFO("\n"                                                    \
    "QueryActiveInterruptsParameters:\n"                                   \
    "  BankId:              %d\n"                                          \
    "  EnabledMask:         0x%016llX\n"                                   \
    "  ActiveMask:          0x%016llX",                                    \
    _pQueryActIntParams->BankId,                                           \
    _pQueryActIntParams->EnabledMask,                                      \
    _pQueryActIntParams->ActiveMask)

#define DBG_DUMP_CLEAR_ACTIVE_INTERRUPTS_PARAMETERS(_pClrActIntParams)   \
DBG_DEV_PRINT_INFO("\n"                                                  \
    "ClearActiveInterruptsParameters:\n"                                 \
    "  BankId:              %d\n"                                        \
    "  ClearActiveMask:     0x%016llX\n"                                 \
    "  FailedClearMask:     0x%016llX",                                  \
    _pClrActIntParams->BankId,                                           \
    _pClrActIntParams->ClearActiveMask,                                  \
    _pClrActIntParams->FailedClearMask)

#define DBG_DUMP_RECONFIGURE_INTERRUPTS_PARAMETERS(_pRecIntParams)    \
DBG_DEV_PRINT_INFO("\n"                                               \
    "ReconfigureInterruptsParameters:\n"                              \
    "  BankId:              %d\n"                                     \
    "  PinNumber:           %d\n"                                     \
    "  OS_IntMode:          %s\n"                                     \
    "  OS_IntPolarity:      %s\n"                                     \
    "  Flags:               %d",                                      \
    _pRecIntParams->BankId,                                           \
    _pRecIntParams->PinNumber,                                        \
    ToString_kInterruptMode(_pRecIntParams->InterruptMode),           \
    ToString_kInterruptPolarity(_pRecIntParams->Polarity),            \
    _pRecIntParams->Flags.AsULONG)

#define DBG_DUMP_QUERY_ENABLED_INTERRUPTS_PARAMETERS(_pQueryEnIntParams)   \
DBG_DEV_PRINT_INFO("\n"                                                    \
    "QueryEnabledInterruptsParameters:\n"                                  \
    "  BankId:              %d\n"                                          \
    "  EnabledMask:         0x%016llX",                                    \
    _pQueryEnIntParams->BankId,                                            \
    _pQueryEnIntParams->EnabledMask)

#define DBG_DUMP_ALL_REGS(_pRegs, _msg)  \
DBG_DEV_PRINT_INFO(_msg" \n"             \
    "DR:   0x%08X\n"                     \
    "GDIR: 0x%08X\n"                     \
    "PSR:  0x%08X\n"                     \
    "ICR1: 0x%08X\n"                     \
    "ICR2: 0x%08X\n"                     \
    "IMR:  0x%08X\n"                     \
    "ISR:  0x%08X (IMR&ISR: 0x%08X)",    \
    _pRegs->DR.R,                        \
    _pRegs->GDIR.R,                      \
    _pRegs->PSR.R,                       \
    _pRegs->ICR1.R,                      \
    _pRegs->ICR2.R,                      \
    _pRegs->IMR.R,                       \
    _pRegs->ISR.R,                       \
    (_pRegs->ISR.R & _pRegs->IMR.R))

#define DBG_DUMP_INT_REGS(_pRegs, _msg)  \
DBG_DEV_PRINT_INFO(_msg" \n"             \
    "PSR:  0x%08X\n"                     \
    "ICR1: 0x%08X\n"                     \
    "ICR2: 0x%08X\n"                     \
    "IMR:  0x%08X\n"                     \
    "ISR:  0x%08X (IMR&ISR: 0x%08X)",    \
    _pRegs->PSR.R,                       \
    _pRegs->ICR1.R,                      \
    _pRegs->ICR2.R,                      \
    _pRegs->IMR.R,                       \
    _pRegs->ISR.R,                       \
    (_pRegs->ISR.R & _pRegs->IMR.R))

#endif


/* ************************************************************************************ */
/* GPIO pin data debug messages                                                         */
/* ************************************************************************************ */
/* Uncomment next line to see GPIO pin data debug messages */

//#define DBG_DUMP_DATA
#ifdef DBG_DUMP_DATA

#define DBG_DUMP_DATA_REGS(_pRegs, _msg)  \
DBG_DEV_PRINT_INFO(_msg" \n"              \
    "DR:  0x%08X\n"                       \
    "DIR: 0x%08X",                        \
    _pRegs->DR.R,                         \
    _pRegs->GDIR.R)


#endif
#endif

#ifndef DBG
#define DBG_PRINT_ERROR(...)
#define DBG_PRINT_ERROR_WITH_STATUS(...)
#endif 

#ifndef DBG_DEV
#define DBG_DEV_METHOD_BEG(...)
#define DBG_DEV_METHOD_BEG_WITH_PARAMS(...)
#define DBG_DEV_METHOD_END(...)
#define DBG_DEV_METHOD_END_WITH_PARAMS(...)
#define DBG_DEV_METHOD_END_WITH_STATUS(...)
#define DBG_DEV_METHOD_END_WITH_STATUS_AND_PARAMS(...)
#define DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS(...)
#define DBG_DEV_PRINT_ERROR(...)
#define DBG_DEV_PRINT_ERROR_WITH_STATUS(...)
#define DBG_DEV_PRINT_WARNING(...)
#define DBG_DEV_PRINT_VERBOSE(...)
#define DBG_DEV_PRINT_INFO(...)
#endif // DEV

#ifndef DBG_DUMP_INTERRUPT
#define DBG_DUMP_ENABLE_INTERRUPT_PARAMETERS(...)
#define DBG_DUMP_DISABLE_INTERRUPT_PARAMETERS(...)
#define DBG_DUMP_MASK_INTERRUPT_PARAMETERS(...)
#define DBG_DUMP_QUERY_ACTIVE_INTERRUPTS_PARAMETERS(...)
#define DBG_DUMP_CLEAR_ACTIVE_INTERRUPTS_PARAMETERS(...)
#define DBG_DUMP_RECONFIGURE_INTERRUPTS_PARAMETERS(...)
#define DBG_DUMP_QUERY_ENABLED_INTERRUPTS_PARAMETERS(...)
#define DBG_DUMP_ALL_REGS(...)
#define DBG_DUMP_INT_REGS(...)
#endif // DBG_DUMP_INTERRUPT


#ifndef DBG_DUMP_DATA
#define DBG_DUMP_DATA_REGS(...)
#endif // DBG_DUMP_DATA

#endif