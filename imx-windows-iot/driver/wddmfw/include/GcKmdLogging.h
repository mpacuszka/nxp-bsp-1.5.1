/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#ifndef _GCKMDLOGGING_H_
#define _GCKMDLOGGING_H_ 1


//
// Copyright (C) Microsoft. All rights reserved.
//
// WPP tracing configuration file. Logs can be dumped from windbg with
// the following command:
//
//   For Now: !rcdrkd.rcdrlogdump galcore
//
//   !rcdrkd.rcdrlogdump gckmd
//

#define GC_ASSERTION(x, ...) { \
    DbgPrintEx( DPFLTR_IHVVIDEO_ID, DPFLTR_INFO_LEVEL, x, __VA_ARGS__); \
    /*DbgBreakPoint();*/ \
    }


//
// Tracing GUID - B5B486C1-F57B-4993-8ED7-E3C2F5E4E65A
//
#define WPP_CONTROL_GUIDS \
    WPP_DEFINE_CONTROL_GUID(GCKMD, (212AEB83,28D9,4620,9F8A,91AA6E990749), \
        WPP_DEFINE_BIT(GC_TRACING_DEFAULT) \
        WPP_DEFINE_BIT(GC_TRACING_PRESENT) \
        WPP_DEFINE_BIT(GC_TRACING_VIDPN) \
        WPP_DEFINE_BIT(GC_TRACING_DEBUG) \
        WPP_DEFINE_BIT(GC_TRACING_BUGCHECK) \
    )

#include <GcLogging.h>

#endif // _GCKMDLOGGING_H_
