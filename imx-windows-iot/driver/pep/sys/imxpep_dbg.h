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
*
* * Neither the name of the copyright holder nor the
*   names of its contributors may be used to endorse or promote products
*   derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS AND CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef _IMXPEP_DBG_H
#define _IMXPEP_DBG_H

#ifdef DBG

extern LARGE_INTEGER        DriverStartTime;

// Uncomment next line for debug message printing
//#define DBG_MESSAGE_PRINTING
#ifdef DBG_MESSAGE_PRINTING

#define DBG_DRV_METHOD_BEG()                             DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:%s +++\n"                     ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"imxpep",__FUNCTION__)
#define DBG_DRV_METHOD_BEG_WITH_PARAMS(_format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:%s +++ "_format_str_"\n"      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"imxpep",__FUNCTION__,__VA_ARGS__)
#define DBG_DRV_METHOD_END()                             DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:%s ---\n"                     ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"imxpep",__FUNCTION__)
#define DBG_DRV_METHOD_END_WITH_STATUS(_status_)         DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:%s --- [0x%.8X]\n"            ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"imxpep",__FUNCTION__,_status_)

static inline LONG GetTime() {
    LARGE_INTEGER  CurrentSystemTime;
    KeQuerySystemTimePrecise(&CurrentSystemTime);
    return (LONG)((CurrentSystemTime.QuadPart - DriverStartTime.QuadPart) / 10000);
}

_Use_decl_annotations_ const char* Dbg_GetPpmNotifyName(ULONG i);
_Use_decl_annotations_ const char* Dbg_GetDpmNotifyName(ULONG i);
_Use_decl_annotations_ const char* Dbg_GetAcpiNotifyName(ULONG i);
_Use_decl_annotations_ const char* Dbg_GetPlatformStateName(ULONG PlatformState);
_Use_decl_annotations_ const char* Dbg_GetProcessorStateName(ULONG ProcessorState);
_Use_decl_annotations_ const char* Dbg_GetDeviceName(ULONG DeviceId);

#endif // DBG_MESSAGE_PRINTING

#endif

#ifndef DBG_DRV_METHOD_BEG
#define DBG_DRV_METHOD_BEG(...)
#endif
#ifndef DBG_DRV_METHOD_BEG_WITH_PARAMS
#define DBG_DRV_METHOD_BEG_WITH_PARAMS(...)
#endif
#ifndef DBG_DRV_METHOD_END
#define DBG_DRV_METHOD_END(...)
#endif
#ifndef DBG_DRV_METHOD_END_WITH_STATUS
#define DBG_DRV_METHOD_END_WITH_STATUS(...)
#endif

#endif
