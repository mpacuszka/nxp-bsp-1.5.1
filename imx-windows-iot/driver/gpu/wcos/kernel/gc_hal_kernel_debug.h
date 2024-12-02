/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2022 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2022 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
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

#ifndef __gc_hal_kernel_debug_h_
#define __gc_hal_kernel_debug_h_

#include <gc_hal_kernel_wcos.h>
#if 0
#include <gc_hal_kernel_linux.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#endif

#include <stdarg.h>

#if 0
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
#include <linux/nmi.h>
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************\
****************************** OS-dependent Macros *****************************
\******************************************************************************/

typedef va_list gctARGUMENTS;

#define gcmkARGUMENTS_START(Arguments, Pointer) \
    va_start(Arguments, Pointer)

#define gcmkARGUMENTS_END(Arguments) \
    va_end(Arguments)

#define gcmkARGUMENTS_ARG(Arguments, Type) \
    va_arg(Arguments, Type)

#define gcmkDECLARE_MUTEX(__mutex__) \
    mutex_t __mutex__ = { 0 }

#define gcmkMUTEX_LOCK(__mutex__) mutex_lock(&__mutex__)
#define gcmkMUTEX_UNLOCK(__mutex__) mutex_unlock(&__mutex__)

#   define gcmkGETPROCESSID() \
        GetCurrentProcessId()

#   define gcmkGETTHREADID() \
        GetCurrentThreadId()

#define gcmkOUTPUT_STRING(String) \
    printk("%s", String); \

inline void gcmkDUMP_STRING(gckOS Os, const char * String) {

    /* mutex_lock(&Os->dumpFilpMutex); */
    if (Os->dumpTarget == 0)
    {
        printk("%s", String);
    }
    else if (Os->dumpFile && Os->dumpTarget == 1)
    {
        printk("%s", String);

#define kBufferLen (1024 * 1024)
        static buffer[kBufferLen]; // 1 Meg dump buffer
        static int bufferPos = 0;
        ULONG len = (ULONG) strlen(String);

        if (len + bufferPos < kBufferLen) {
            memcpy(&buffer[bufferPos], String, len);
            bufferPos += len;
        } else {
            __debugbreak();
        }

        if(KeGetCurrentIrql() == PASSIVE_LEVEL && bufferPos > 0) {
            IO_STATUS_BLOCK IoStatusBlock;
            NTSTATUS status = ZwWriteFile(Os->dumpFile, NULL, NULL, NULL,
                &IoStatusBlock, (PVOID) buffer, bufferPos, NULL, NULL);
            if(!NT_SUCCESS(status)) __debugbreak();
            bufferPos = 0;
        }
    }

    /* mutex_unlock(&Os->dumpFilpMutex); */
}

/* Print Support */

inline int gcmkVSPRINTF(char * Destination, size_t Size, const char * Message, va_list * Arguments)
{
    RtlStringCchVPrintfA(Destination, Size, Message, *((va_list*)Arguments));
    size_t length;
    RtlStringCchLengthA(Destination, Size, &length);
    return (int) length;
}

inline int gcmkSPRINTF(char * Destination, size_t size, const char * message, ...)
{
    va_list args;
    va_start(args, message);
    RtlStringCchVPrintfA(Destination, size, message, args);
    va_end(args);

    size_t length;
    RtlStringCchLengthA(Destination, size, &length);
    return (int) length;
}

#if 0
#define gcmkSTRCATSAFE(Destination, Size, String) \
    RtlStringCchCatA(Destination, Size, String)
#endif

#define gcmkMEMCPY(Destination, Source, Size) \
    memcpy(Destination, Source, Size)

#define gcmkSTRLEN(String) \
    strlen(String)

/* If not zero, forces data alignment in the variable argument list
   by its individual size. */
#define gcdALIGNBYSIZE      1

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_kernel_debug_h_ */

