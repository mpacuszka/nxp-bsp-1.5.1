/****************************************************************************
* Copyright (c) Microsoft Corporation.
*
*    Licensed under the MIT License.
*    Licensed under the GPL License.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/

#ifndef _GCLOGGING_H_
#define _GCLOGGING_H_ 1

//
// Copyright (C) Microsoft. All rights reserved.
//
// WPP tracing configuration file shared between usermode and kernel mode.
//

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

//
// Debug/Bugcheck helpers used by tracing macros.
// NOTE: These are not intended to be called from anywhere else
//
extern int _GcLogBugcheck (ULONG Level);
extern int _GcLogDebug (ULONG Level);
    
// begin_wpp config
//
// FUNC GC_LOG_CRITICAL_ERROR{LEVEL=TRACE_LEVEL_CRITICAL, FLAGS=GC_TRACING_BUGCHECK}(MSG, ...);
// USEPREFIX (GC_LOG_CRITICAL_ERROR, "%!STDPREFIX! [%s @ %u] CRITICAL ERROR:", __FILE__, __LINE__);
//
// FUNC GC_LOG_ASSERTION{LEVEL=TRACE_LEVEL_ERROR, FLAGS=GC_TRACING_DEBUG}(MSG, ...);
// USEPREFIX (GC_LOG_ASSERTION, "%!STDPREFIX! [%s @ %u] ASSERTION :", __FILE__, __LINE__);
//
// FUNC GC_LOG_ERROR{LEVEL=TRACE_LEVEL_ERROR, FLAGS=GC_TRACING_DEFAULT}(MSG, ...);
// USEPREFIX (GC_LOG_ERROR, "%!STDPREFIX! [%s @ %u] ERROR :", __FILE__, __LINE__);
//
// FUNC GC_LOG_LOW_MEMORY{LEVEL=TRACE_LEVEL_ERROR, FLAGS=GC_TRACING_DEFAULT}(MSG, ...);
// USEPREFIX (GC_LOG_LOW_MEMORY, "%!STDPREFIX! [%s @ %u] LOW MEMORY :", __FILE__, __LINE__);
//
// FUNC GC_LOG_WARNING{LEVEL=TRACE_LEVEL_WARNING, FLAGS=GC_TRACING_DEFAULT}(MSG, ...);
// USEPREFIX (GC_LOG_WARNING, "%!STDPREFIX! [%s @ %u] WARNING :", __FILE__, __LINE__);
//
// FUNC GC_LOG_INFORMATION{LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=GC_TRACING_DEFAULT}(MSG, ...);
// USEPREFIX (GC_LOG_INFORMATION, "%!STDPREFIX! [%s @ %u] INFO :", __FILE__, __LINE__);
//
// FUNC GC_LOG_TRACE{LEVEL=TRACE_LEVEL_VERBOSE, FLAGS=GC_TRACING_DEFAULT}(MSG, ...);
// USEPREFIX (GC_LOG_TRACE, "%!STDPREFIX! [%s @ %u] TRACE :", __FILE__, __LINE__);
//
// FUNC GC_TRACE_EVENTS(LEVEL, FLAGS, MSG, ...);
// USEPREFIX (GC_TRACE_EVENTS, "%!STDPREFIX! [%s @ %u] TRACE :", __FILE__, __LINE__);
//
// FUNC GC_CRITICAL_ASSERT{LEVEL=TRACE_LEVEL_CRITICAL, FLAGS=GC_TRACING_BUGCHECK}(GC_CRIT_ASSERT_EXP);
// USEPREFIX (GC_CRITICAL_ASSERT, "%!STDPREFIX! [%s @ %u] CRITICAL ASSERTION :%s", __FILE__, __LINE__, #GC_CRIT_ASSERT_EXP);
//
// FUNC GC_ASSERT{LEVEL=TRACE_LEVEL_ERROR, FLAGS=GC_TRACING_DEBUG}(GC_ASSERT_EXP);
// USEPREFIX (GC_ASSERT, "%!STDPREFIX! [%s @ %u] ASSERTION :%s", __FILE__, __LINE__, #GC_ASSERT_EXP);
//
// end_wpp


//
// GC_LOG... customization
//

#define WPP_LEVEL_FLAGS_POST(LEVEL,FLAGS) \
    ,(((WPP_BIT_ ## FLAGS) == WPP_BIT_GC_TRACING_BUGCHECK) ? \
            _GcLogBugcheck(LEVEL) : \
            (((WPP_BIT_ ## FLAGS) == WPP_BIT_GC_TRACING_DEBUG) ? \
                _GcLogDebug(LEVEL) : 1))

//
// GC_CRTITICAL_ASSERT customization
//

#define WPP_RECORDER_LEVEL_FLAGS_GC_CRIT_ASSERT_EXP_FILTER(LEVEL, FLAGS, GC_CRIT_ASSERT_EXP) \
    (!(GC_CRIT_ASSERT_EXP))

#define WPP_RECORDER_LEVEL_FLAGS_GC_CRIT_ASSERT_EXP_ARGS(LEVEL, FLAGS, GC_CRIT_ASSERT_EXP) \
    WPP_CONTROL(WPP_BIT_ ## FLAGS).AutoLogContext, LEVEL, WPP_BIT_ ## FLAGS

#define WPP_LEVEL_FLAGS_GC_CRIT_ASSERT_EXP_POST(LEVEL, FLAGS, GC_CRIT_ASSERT_EXP) \
    ,((!(GC_CRIT_ASSERT_EXP)) ? _GcLogBugcheck(LEVEL) : 1)

//
// GC_ASSERT customization
//

#define WPP_RECORDER_LEVEL_FLAGS_GC_ASSERT_EXP_FILTER(LEVEL, FLAGS, GC_ASSERT_EXP) \
    (!(GC_ASSERT_EXP))

#define WPP_RECORDER_LEVEL_FLAGS_GC_ASSERT_EXP_ARGS(LEVEL, FLAGS, GC_ASSERT_EXP) \
    WPP_CONTROL(WPP_BIT_ ## FLAGS).AutoLogContext, LEVEL, WPP_BIT_ ## FLAGS

#define WPP_LEVEL_FLAGS_GC_ASSERT_EXP_POST(LEVEL, FLAGS, GC_ASSERT_EXP) \
    ,((!(GC_ASSERT_EXP)) ? _GcLogDebug(LEVEL) : 1)


#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // _GCLOGGING_H_

