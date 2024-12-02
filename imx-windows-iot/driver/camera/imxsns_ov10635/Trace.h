/*++

Module Name:

    Trace.h

Abstract:

    Header file for the debug tracing related function defintions and macros.

Environment:

    Kernel mode

--*/
extern "C" {
#include <initguid.h>
}
//
// Define the tracing flags.
//
// Tracing GUID - {4b647554-ed7f-4019-b2f9-ae6a060e35ae}
//

#define WPP_CONTROL_GUIDS                                              \
    WPP_DEFINE_CONTROL_GUID(                                           \
        IMXSNSOV10635TraceGuid, (4b647554,ed7f,4019,b2f9,ae6a060e35ae),  \
                                                                       \
        WPP_DEFINE_BIT(TRACING_DEFAULT) \
        WPP_DEFINE_BIT(TRACE_DRIVER)                                   \
        WPP_DEFINE_BIT(TRACE_DEVICE)                                   \
        WPP_DEFINE_BIT(TRACE_QUEUE)                                    \
        WPP_DEFINE_BIT(TRACE_IOTARGETS)                                \
        WPP_DEFINE_BIT(TRACE_FSLOV10635)                               \
        )

        //WPP_DEFINE_BIT(MYDRIVER_ALL_INFO)

#define WPP_FLAG_LEVEL_LOGGER(flag, level)                                  \
    WPP_LEVEL_LOGGER(flag)

#define WPP_FLAG_LEVEL_ENABLED(flag, level)                                 \
    (WPP_LEVEL_ENABLED(flag) &&                                             \
     WPP_CONTROL(WPP_BIT_ ## flag).Level >= level)

#define WPP_LEVEL_FLAGS_LOGGER(lvl,flags) \
           WPP_LEVEL_LOGGER(flags)

#define WPP_LEVEL_FLAGS_ENABLED(lvl, flags) \
           (WPP_LEVEL_ENABLED(flags) && WPP_CONTROL(WPP_BIT_ ## flags).Level >= lvl)

//
// WPP orders static parameters before dynamic parameters. To support the Trace function
// defined below which sets FLAGS=MYDRIVER_ALL_INFO, a custom macro must be defined to
// reorder the arguments to what the .tpl configuration file expects.
//
// #define WPP_RECORDER_FLAGS_LEVEL_ARGS(flags, lvl) WPP_RECORDER_LEVEL_FLAGS_ARGS(lvl, flags)
// #define WPP_RECORDER_FLAGS_LEVEL_FILTER(flags, lvl) WPP_RECORDER_LEVEL_FLAGS_FILTER(lvl, flags)


//
// This comment block is scanned by the trace preprocessor to define our
// Trace function.
//
// begin_wpp config
// FUNC TraceEvents(LEVEL, FLAGS, MSG, ...);
//
// FUNC TRACE_FUNC_ENTRY{LEVEL=TRACE_LEVEL_VERBOSE}(FLAGS, ...);
// USESUFFIX(TRACE_FUNC_ENTRY, "%!FUNC! Entry");
//
// FUNC TRACE_FUNC_EXIT{LEVEL=TRACE_LEVEL_VERBOSE}(FLAGS, ...);
// USESUFFIX(TRACE_FUNC_EXIT, "%!FUNC! Exit");
// 
// 
// USEPREFIX (IMX_LOG_ERROR, "%!STDPREFIX! [%s @ %u] ERROR :", __FILE__, __LINE__);
// FUNC LOG_ERROR{LEVEL=TRACE_LEVEL_ERROR, FLAGS=TRACING_DEFAULT}(MSG, ...);
//
// USEPREFIX (LOG_WARNING, "%!STDPREFIX! [%s @ %u] WARNING :", __FILE__, __LINE__);
// FUNC LOG_WARNING{LEVEL=TRACE_LEVEL_WARNING, FLAGS=TRACING_DEFAULT}(MSG, ...);
//
// USEPREFIX (LOG_INFORMATION, "%!STDPREFIX! [%s @ %u] INFO :", __FILE__, __LINE__);
// FUNC LOG_INFORMATION{LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACING_DEFAULT}(MSG, ...);
//
// USEPREFIX (LOG_TRACE, "%!STDPREFIX! [%s @ %u] TRACE :", __FILE__, __LINE__);
// FUNC LOG_TRACE{LEVEL=TRACE_LEVEL_VERBOSE, FLAGS=TRACING_DEFAULT}(MSG, ...);
// 
// end_wpp
//
