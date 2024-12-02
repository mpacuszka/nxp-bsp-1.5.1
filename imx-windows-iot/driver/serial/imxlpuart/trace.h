// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef _IMX_LPUART_TRACE_H_
#define _IMX_LPUART_TRACE_H_ 1

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

//
// Defining control guids, including this is required to happen before
// including the tmh file (if the WppRecorder API is used)
//
#include <WppRecorder.h>

//
// Tracing GUID - {05A01BCE-130A-11ED-A51A-00155D2578B4}
//
#define WPP_CONTROL_GUIDS \
    WPP_DEFINE_CONTROL_GUID(IMXLPUART, (05A01BCE,130A,11ED,A51A,00155D2578B4), \
        WPP_DEFINE_BIT(IMX_LPUART_TRACING_DEFAULT) \
    )

// begin_wpp config
//
// USEPREFIX (IMX_LPUART_LOG_ERROR, "%!STDPREFIX! [%s @ %u] ERROR :", __FILE__, __LINE__);
// FUNC IMX_LPUART_LOG_ERROR{LEVEL=TRACE_LEVEL_ERROR, FLAGS=IMX_LPUART_TRACING_DEFAULT}(MSG, ...);
//
// USEPREFIX (IMX_LPUART_LOG_LOW_MEMORY, "%!STDPREFIX! [%s @ %u] LOW MEMORY :", __FILE__, __LINE__);
// FUNC IMX_LPUART_LOG_LOW_MEMORY{LEVEL=TRACE_LEVEL_ERROR, FLAGS=IMX_LPUART_TRACING_DEFAULT}(MSG, ...);
//
// USEPREFIX (IMX_LPUART_LOG_WARNING, "%!STDPREFIX! [%s @ %u] WARNING :", __FILE__, __LINE__);
// FUNC IMX_LPUART_LOG_WARNING{LEVEL=TRACE_LEVEL_WARNING, FLAGS=IMX_LPUART_TRACING_DEFAULT}(MSG, ...);
//
// USEPREFIX (IMX_LPUART_LOG_INFORMATION, "%!STDPREFIX! [%s @ %u] INFO :", __FILE__, __LINE__);
// FUNC IMX_LPUART_LOG_INFORMATION{LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=IMX_LPUART_TRACING_DEFAULT}(MSG, ...);
//
// USEPREFIX (IMX_LPUART_LOG_TRACE, "%!STDPREFIX! [%s @ %u] TRACE :", __FILE__, __LINE__);
// FUNC IMX_LPUART_LOG_TRACE{LEVEL=TRACE_LEVEL_VERBOSE, FLAGS=IMX_LPUART_TRACING_DEFAULT}(MSG, ...);
//
// end_wpp

// begin_wpp config 
// CUSTOM_TYPE(IMX_LPUART_STATE, ItemEnum(IMX_LPUART_STATE) );
// end_wpp

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // _IMX_LPUART_TRACE_H_

