#pragma once

#include <windows.h>
#ifdef  _HMFTLOGGING_
    #include <Evntprov.h>

    extern UCHAR    g_ucLevel;
#endif

namespace CHMFTTracing
{
enum eTraceLeve {
    TRACE_MEMORY        = 8,
    TRACE_INFORMATION   = 4,
    TRACE_ERROR         = 2
};
}

void TraceInitialize(void);

void TraceUninitialize(void);

void TraceString(
    const UCHAR ucLevel,
    const WCHAR *pwszFormat,
    ...
);
