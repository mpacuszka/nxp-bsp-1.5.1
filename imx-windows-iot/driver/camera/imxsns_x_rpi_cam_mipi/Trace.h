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
// Tracing GUID - 0A750B66-3193-4A82-B3F3-1C20A2B60F0C
//

#define WPP_CONTROL_GUIDS                                              \
    WPP_DEFINE_CONTROL_GUID(                                           \
        IMXSNSRXRpiCamMipiTraceGuid, (0A750B66,3193,4A82,B3F3,1C20A2B60F0C),  \
                                                                       \
        WPP_DEFINE_BIT(TRACE_DRIVER)                                   \
        WPP_DEFINE_BIT(TRACE_DEVICE)                                   \
        WPP_DEFINE_BIT(TRACE_QUEUE)                                    \
        WPP_DEFINE_BIT(TRACE_IOTARGETS)                                \
        WPP_DEFINE_BIT(TRACE_FSLXRPICAMMIPI)                           \
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
// end_wpp
//
