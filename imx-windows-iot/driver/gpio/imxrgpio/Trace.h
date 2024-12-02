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

#ifndef trace_H_
#define trace_H_

#pragma once

#ifdef DBG

EXTERN_C_START
EXTERN_C_END

#ifndef DBG_MSG_DRV_PREFIX
#define DBG_MSG_DRV_PREFIX "GPIO"
#endif

#define ToString_IoConnectPinCmd(_Cmd) (                \
    (_Cmd & CFG_PIN_CMD_CHECK_ONLY)?     "CHECK_OLNY":  \
    (_Cmd & CFG_PIN_CMD_SET)?            "SET":         \
    (_Cmd & CFG_PIN_CMD_SET_DEFAULT)?    "SET DEFAULT": \
                                         "!!! UNKNOWN !!!")

#define ToString_AcpiPullCfg(_ACPI_CFG_VAL) (\
    (_ACPI_CFG_VAL == GPIO_PIN_PULL_CONFIGURATION_DEFAULT)?  "Default":  \
    (_ACPI_CFG_VAL == GPIO_PIN_PULL_CONFIGURATION_PULLUP)?   "PullUp":   \
    (_ACPI_CFG_VAL == GPIO_PIN_PULL_CONFIGURATION_PULLDOWN)? "PullDown": \
    (_ACPI_CFG_VAL == GPIO_PIN_PULL_CONFIGURATION_NONE)?     "PullNone": \
                                                             "!!! UNKNOWN !!!")

#define ToString_ConnectioMode(_CONN_MODE) (\
    (_CONN_MODE == ConnectModeInvalid)? "Invalid": \
    (_CONN_MODE == ConnectModeInput)?   "Input":   \
    (_CONN_MODE == ConnectModeOutput)?  "Output":  \
                                        "!!! UNKNOWN !!!")

#define ToString_WdfPowerDeviceState(_WDF_PWR_DEV_STATE) (\
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceInvalid)?                "Invalid":                \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceD0)?                     "D0":                     \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceD1)?                     "D1":                     \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceD2)?                     "D2":                     \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceD3)?                     "D3":                     \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceD3Final)?                "D3Final":                \
    (_WDF_PWR_DEV_STATE == WdfPowerDevicePrepareForHibernation)?  "PrepareForHibernation":  \
    (_WDF_PWR_DEV_STATE == WdfPowerDeviceMaximum)?                "Max":                    \
                                                                  "!!! UNKNOWN !!!")

#define ToString_kInterruptMode(_WDF_INT_MODE) (\
    (_WDF_INT_MODE == LevelSensitive)?  "LevelSensitive": \
    (_WDF_INT_MODE == Latched)?         "Latched":        \
                                        "!!! UNKNOWN !!!")


#define ToString_kInterruptPolarity(_WDF_INT_POLARITY) (\
    (_WDF_INT_POLARITY == InterruptPolarityUnknown)?  "PolarityUnknown":           \
    (_WDF_INT_POLARITY == InterruptActiveHigh)?       "RisingEdge or ActiveHigh":  \
    (_WDF_INT_POLARITY == InterruptActiveLow)?        "FollingEdge or ActiveLow":  \
                                                      "!!! UNKNOWN !!!")

#define GPIO_x_IOy_MSG "GPIO_%d_IO%02d "
#define GPIO_x_IOy_VAL pImxConnectPin->BankId + 1, pImxConnectPin->PinNumber

#define DBG_DEV_PRINT_INFO_MUX_AND_PAD_CTL_DUMP(_Msg, _MuxReg, _PadReg) DBG_DEV_PRINT_INFO(GPIO_x_IOy_MSG"PadMux: 0x%08X (SION=%d, MUX=%d), PadCtl: 0x%08X (HYS=%d, OD=%d, PD=%d, PU=%d, FSEL=%d, DSE=%d)"_Msg, GPIO_x_IOy_VAL, (_MuxReg)->R, (_MuxReg)->B.SION, (_MuxReg)->B.MUX_MODE, (_PadReg)->R, (_PadReg)->B.HYS_EN, (_PadReg)->B.OD_EN, (_PadReg)->B.PD_EN, (_PadReg)->B.PU_EN, (_PadReg)->B.FSEL, (_PadReg)->B.DSE);

extern LARGE_INTEGER        DriverStartTime;

static inline LONG GetTime() {
    LARGE_INTEGER  CurrentSystemTime;
    KeQuerySystemTimePrecise(&CurrentSystemTime);
    return (LONG)((CurrentSystemTime.QuadPart - DriverStartTime.QuadPart) / 10000);
}

#define DBG_PRINT_ERROR(_format_str_,...)                           DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:ERROR %s() "_format_str_"\n"          ,GetTime() ,KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__)
#define DBG_PRINT_ERROR_WITH_STATUS(_status_,_format_str_,...)      DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:ERROR %s() "_format_str_" [0x%.8X]\n" ,GetTime() ,KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),DBG_MSG_DRV_PREFIX,__FUNCTION__,__VA_ARGS__,_status_)

// Uncomment next define for debug message printing
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

#ifndef DBG_PRINT_ERROR
#define DBG_PRINT_ERROR(...)
#endif
#ifndef DBG_PRINT_ERROR_WITH_STATUS
#define DBG_PRINT_ERROR_WITH_STATUS(...)
#endif 
#ifndef DBG_DEV_METHOD_BEG
#define DBG_DEV_METHOD_BEG(...)
#endif 
#ifndef DBG_DEV_METHOD_BEG_WITH_PARAMS
#define DBG_DEV_METHOD_BEG_WITH_PARAMS(...)
#endif 
#ifndef DBG_DEV_METHOD_END
#define DBG_DEV_METHOD_END(...)
#endif 
#ifndef DBG_DEV_METHOD_END_WITH_PARAMS
#define DBG_DEV_METHOD_END_WITH_PARAMS(...)
#endif 
#ifndef DBG_DEV_METHOD_END_WITH_STATUS
#define DBG_DEV_METHOD_END_WITH_STATUS(...)
#endif 
#ifndef DBG_DEV_METHOD_END_WITH_STATUS_AND_PARAMS
#define DBG_DEV_METHOD_END_WITH_STATUS_AND_PARAMS(...)
#endif 
#ifndef DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS
#define DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS(...)
#endif 
#ifndef DBG_DEV_PRINT_WARNING
#define DBG_DEV_PRINT_WARNING(...)
#endif 
#ifndef DBG_DEV_PRINT_VERBOSE
#define DBG_DEV_PRINT_VERBOSE(...)
#endif 
#ifndef DBG_DEV_PRINT_INFO
#define DBG_DEV_PRINT_INFO(...)
#endif 

#ifndef DBG_DEV_PRINT_INFO_MUX_AND_PAD_CTL_DUMP
#define DBG_DEV_PRINT_INFO_MUX_AND_PAD_CTL_DUMP(...)
#endif 

#endif