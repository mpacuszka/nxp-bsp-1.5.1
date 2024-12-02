// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name: 
//
//    LPSPItrace.h
//
// Abstract:
//
//    This module contains the tracing definitions for the 
//    IMX LPSPI controller driver.
//
// Environment:
//
//    kernel-mode only
//

#ifndef _LPSPI_TRACE_H_
#define _LPSPI_TRACE_H_

WDF_EXTERN_C_START


//
// Defining control guids, including this is required to happen before
// including the tmh file (if the WppRecorder API is used)
//
#include <WppRecorder.h>

//
// Debug support
//
extern BOOLEAN LPSPIIsDebuggerPresent ();
extern BOOLEAN LPSPIBreakPoint ();


//
// Trace log buffer size
//
#if DBG
    #define LPSPI_RECORDER_TOTAL_BUFFER_SIZE       (PAGE_SIZE * 4)
    #define LPSPI_RECORDER_ERROR_PARTITION_SIZE    (LPSPI_RECORDER_TOTAL_BUFFER_SIZE / 4)
#else
    #define LPSPI_RECORDER_TOTAL_BUFFER_SIZE       (PAGE_SIZE)
    #define LPSPI_RECORDER_ERROR_PARTITION_SIZE    (LPSPI_RECORDER_TOTAL_BUFFER_SIZE / 4)
#endif 


//
// The trace log identifier for each LPSPI controller (device)
//
#define LPSPI_RECORDER_LOG_ID   "LPSPI%08X"


//
// Tracing GUID - 32175a97-8234-4349-96d7-11b4d8082551
//
#define WPP_CONTROL_GUIDS \
    WPP_DEFINE_CONTROL_GUID(IMXLPSPI, (32175a97,8234,4349,96d7,11b4d8082551), \
        WPP_DEFINE_BIT(LPSPI_TRACING_DEFAULT) \
        WPP_DEFINE_BIT(LPSPI_TRACING_VERBOSE) \
        WPP_DEFINE_BIT(LPSPI_TRACING_DEBUG) \
    )

// begin_wpp config
//
// FUNC LPSPI_LOG_ASSERTION{LEVEL=TRACE_LEVEL_ERROR, FLAGS=LPSPI_TRACING_DEBUG}(IFRLOG, MSG, ...);
// USEPREFIX (LPSPI_LOG_ASSERTION, "%!STDPREFIX! [%s @ %u] ASSERTION :", __FILE__, __LINE__);
//
// FUNC LPSPI_LOG_ERROR{LEVEL=TRACE_LEVEL_ERROR, FLAGS=LPSPI_TRACING_DEFAULT}(IFRLOG, MSG, ...);
// USEPREFIX (LPSPI_LOG_ERROR, "%!STDPREFIX! [%s @ %u] ERROR :", __FILE__, __LINE__);
//
// FUNC LPSPI_LOG_WARNING{LEVEL=TRACE_LEVEL_WARNING, FLAGS=LPSPI_TRACING_DEFAULT}(IFRLOG, MSG, ...);
// USEPREFIX (LPSPI_LOG_WARNING, "%!STDPREFIX! [%s @ %u] WARNING :", __FILE__, __LINE__);
//
// FUNC LPSPI_LOG_INFORMATION{LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=LPSPI_TRACING_DEFAULT}(IFRLOG, MSG, ...);
// USEPREFIX (LPSPI_LOG_INFORMATION, "%!STDPREFIX! [%s @ %u] INFO :", __FILE__, __LINE__);
//
// FUNC LPSPI_LOG_TRACE{LEVEL=TRACE_LEVEL_VERBOSE, FLAGS=LPSPI_TRACING_VERBOSE}(IFRLOG, MSG, ...);
// USEPREFIX (LPSPI_LOG_TRACE, "%!STDPREFIX! [%s @ %u] TRACE :", __FILE__, __LINE__);
//
// FUNC LPSPI_ASSERT{LEVEL=TRACE_LEVEL_ERROR, FLAGS=LPSPI_TRACING_DEBUG}(IFRLOG, LPSPI_ASSERT_EXP);
// USEPREFIX (LPSPI_ASSERT, "%!STDPREFIX! [%s @ %u] ASSERTION :%s", __FILE__, __LINE__, #LPSPI_ASSERT_EXP);
//
// end_wpp

//
// LPSPI_ASSERT customization
//
#define WPP_RECORDER_LEVEL_FLAGS_IFRLOG_LPSPI_ASSERT_EXP_FILTER(LEVEL, FLAGS, IFRLOG, LPSPI_ASSERT_EXP) \
    (!(LPSPI_ASSERT_EXP))

#define WPP_RECORDER_LEVEL_FLAGS_IFRLOG_LPSPI_ASSERT_EXP_ARGS(LEVEL, FLAGS, IFRLOG, LPSPI_ASSERT_EXP) \
    IFRLOG, LEVEL, WPP_BIT_ ## FLAGS  

#define WPP_LEVEL_FLAGS_IFRLOG_LPSPI_ASSERT_EXP_POST(LEVEL, FLAGS, IFRLOG, LPSPIP_ASSERT_EXP) \
    ,((!(LPSPIP_ASSERT_EXP)) ? LPSPIBreakPoint() : 1)


//
// Custom types
//

// begin_wpp config
// CUSTOM_TYPE(REQUESTTYPE, ItemEnum(LPSPI_REQUEST_TYPE));
// end_wpp


WDF_EXTERN_C_END

#endif // !_LPSPI_TRACE_H_