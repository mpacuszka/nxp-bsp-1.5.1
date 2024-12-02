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

#ifndef _TRACE_H_
#define _TRACE_H_
#pragma once

#include <ntddk.h>// For GetTime function


// MALONE_TRACE_OFF          - no no debug messages are added to the driver binary.
// MALONE_TRACE_WPP          - WPP is used to log debug messages.
// MALONE_TRACE_DBG_PRINT_EX - Messages are printed to the "windbg command" window. Note: This option inserts time delays and can cause protocol timeouts.
//                              Uncomment #define(s) in the second part of "Trace.h" to print reqested messages.

#define MALONE_TRACE_OFF             0
#define MALONE_TRACE_WPP             1
#define MALONE_TRACE_DBG_PRINT_EX    2

// Uncomment one of the next define to enable debug message printing. If both of them are commented out, no debug messages are added to the driver binary.
#define MALONE_TRACE                 MALONE_TRACE_WPP
//#define MALONE_TRACE                   MALONE_TRACE_DBG_PRINT_EX
//#define MALONE_TRACE                   MALONE_TRACE_OFF

#ifndef MALONE_TRACE
    #define MALONE_TRACE                 1
#endif

#define PRINT
#define DRV
#define EVENTS
//#define PERF
//#define DEV
//#define RPC
//#define IOCTL
//#define VDEC
//#define FBL


#if (MALONE_TRACE == MALONE_TRACE_WPP)

//
// Define the tracing flags.
//
// Tracing GUID - 11762860-4082-4d5b-ae8b-5a8e080f2c4b
//

#define WPP_CONTROL_GUIDS                                              \
    WPP_DEFINE_CONTROL_GUID(                                           \
        malonekmTraceGuid, (11762860,4082,4d5b,ae8b,5a8e080f2c4b),   \
        WPP_DEFINE_BIT(TRACE_ERROR)           /* bit 0 = 0x00000001 */ \
        WPP_DEFINE_BIT(TRACE_DRIVER)          /* bit 1 = 0x00000002 */ \
        WPP_DEFINE_BIT(TRACE_DEVICE)          /* bit 2 = 0x00000004 */ \
        WPP_DEFINE_BIT(TRACE_IOCTL)           /* bit 3 = 0x00000008 */ \
        WPP_DEFINE_BIT(TRACE_VDEC)            /* bit 4 = 0x00000010 */ \
        WPP_DEFINE_BIT(TRACE_RPC)             /* bit 5 = 0x00000020 */ \
        WPP_DEFINE_BIT(TRACE_FBL)             /* bit 6 = 0x00000040 */ \
        WPP_DEFINE_BIT(TRACE_EVENTS)          /* bit 7 = 0x00000080 */ \
        WPP_DEFINE_BIT(PERF_EVENTS)           /* bit 8 = 0x00000100 */ \
)

/*
#define WPP_FLAG_LEVEL_LOGGER(flag, level)    WPP_LEVEL_LOGGER(flag)
#define WPP_FLAG_LEVEL_ENABLED(flag, level)  (WPP_LEVEL_ENABLED(flag) && WPP_CONTROL(WPP_BIT_ ## flag).Level >= level)
*/
#define WPP_LEVEL_FLAGS_LOGGER(lvl,flags)     WPP_LEVEL_LOGGER(flags)
#define WPP_LEVEL_FLAGS_ENABLED(lvl, flags)  (WPP_LEVEL_ENABLED(flags) && WPP_CONTROL(WPP_BIT_ ## flags).Level >= lvl)

#define WPP_LEVEL_FLAGS_EXP_LOGGER(lvl, flags, EXP)    WPP_LEVEL_LOGGER (flags)
#define WPP_LEVEL_FLAGS_EXP_ENABLED(lvl, flags, EXP)  (WPP_LEVEL_ENABLED(flags) && WPP_CONTROL(WPP_BIT_ ## flags).Level >= lvl)

#define WPP_RECORDER_LEVEL_FLAGS_EXP_FILTER(lvl, FLAGS, EXP)   WPP_RECORDER_LEVEL_FLAGS_FILTER(lvl, FLAGS)
#define WPP_RECORDER_LEVEL_FLAGS_EXP_ARGS(lvl, FLAGS, EXP)     WPP_RECORDER_LEVEL_FLAGS_ARGS(lvl, FLAGS)

//
// This comment block is scanned by the trace preprocessor to define our
// Trace function.
//
// begin_wpp config
//
// FUNC DBG_PRINT_INFO{LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DRIVER}(MSG, ...);
// FUNC DBG_PRINT_ERROR{LEVEL=TRACE_LEVEL_ERROR, FLAGS=TRACE_ERROR}(MSG, ...);
// FUNC DBG_PRINT_ERROR_WITH_STATUS{LEVEL=TRACE_LEVEL_ERROR, FLAGS=TRACE_ERROR}(EXP, MSG, ...);

// USEPREFIX(DBG_DRV_METHOD_BEG,              "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_DRV_METHOD_BEG_WITH_PARAMS,  "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_DRV_METHOD_END,              "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_DRV_METHOD_END_WITH_PARAMS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_DRV_METHOD_END_WITH_STATUS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_DRV_PRINT_ERROR,             "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_DRV_PRINT_ERROR_WITH_STATUS, "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_DRV_PRINT_WARNING,           "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_DRV_PRINT_INFO,              "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_DRV_PRINT_VERBOSE,           "%!STDPREFIX!   %!FUNC!");
// USESUFFIX(DBG_DRV_METHOD_BEG,              ")");
// USESUFFIX(DBG_DRV_METHOD_BEG_WITH_PARAMS,  ")");
// USESUFFIX(DBG_DRV_METHOD_END,              ")");
// USESUFFIX(DBG_DRV_METHOD_END_WITH_PARAMS,  ")");
// USESUFFIX(DBG_DRV_METHOD_END_WITH_STATUS,  ") [%!STATUS!]", EXP);
// USESUFFIX(DBG_DRV_PRINT_ERROR_WITH_STATUS, " [%!STATUS!] !!!", EXP);
// FUNC      DBG_DRV_METHOD_BEG{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DRIVER}(...);
// FUNC      DBG_DRV_METHOD_BEG_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DRIVER}(MSG, ...);
// FUNC      DBG_DRV_METHOD_END{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DRIVER}(...);
// FUNC      DBG_DRV_METHOD_END_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DRIVER}(MSG, ...);
// FUNC      DBG_DRV_METHOD_END_WITH_STATUS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DRIVER}(EXP);
// FUNC      DBG_DRV_PRINT_ERROR{             LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_DRIVER}(MSG, ...);
// FUNC      DBG_DRV_PRINT_ERROR_WITH_STATUS{ LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_DRIVER}(EXP, MSG, ...);
// FUNC      DBG_DRV_PRINT_WARNING{           LEVEL=TRACE_LEVEL_WARNING,     FLAGS=TRACE_DRIVER}(MSG, ...);
// FUNC      DBG_DRV_PRINT_INFO{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DRIVER}(MSG, ...);
// FUNC      DBG_DRV_PRINT_VERBOSE{           LEVEL=TRACE_LEVEL_VERBOSE,     FLAGS=TRACE_DRIVER}(MSG, ...);
//
// USEPREFIX(DBG_DEV_METHOD_BEG,              "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_DEV_METHOD_BEG_WITH_PARAMS,  "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_DEV_METHOD_END,              "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_DEV_METHOD_END_WITH_PARAMS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_DEV_METHOD_END_WITH_STATUS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_DEV_PRINT_ERROR,             "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_DEV_PRINT_ERROR_WITH_STATUS, "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_DEV_PRINT_WARNING,           "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_DEV_PRINT_INFO,              "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_DEV_PRINT_VERBOSE,           "%!STDPREFIX!   %!FUNC!");
// USESUFFIX(DBG_DEV_METHOD_BEG,              ")");
// USESUFFIX(DBG_DEV_METHOD_BEG_WITH_PARAMS,  ")");
// USESUFFIX(DBG_DEV_METHOD_END,              ")");
// USESUFFIX(DBG_DEV_METHOD_END_WITH_PARAMS,  ")");
// USESUFFIX(DBG_DEV_METHOD_END_WITH_STATUS,  ") [%!STATUS!]", EXP);
// USESUFFIX(DBG_DEV_PRINT_ERROR_WITH_STATUS, " [%!STATUS!] !!!", EXP);
// FUNC      DBG_DEV_METHOD_BEG{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DEVICE}(...);
// FUNC      DBG_DEV_METHOD_BEG_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DEVICE}(MSG, ...);
// FUNC      DBG_DEV_METHOD_END{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DEVICE}(...);
// FUNC      DBG_DEV_METHOD_END_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DEVICE}(MSG, ...);
// FUNC      DBG_DEV_METHOD_END_WITH_STATUS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DEVICE}(EXP);
// FUNC      DBG_DEV_PRINT_ERROR{             LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_DEVICE}(MSG, ...);
// FUNC      DBG_DEV_PRINT_ERROR_WITH_STATUS{ LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_DEVICE}(EXP, MSG, ...);
// FUNC      DBG_DEV_PRINT_WARNING{           LEVEL=TRACE_LEVEL_WARNING,     FLAGS=TRACE_DEVICE}(MSG, ...);
// FUNC      DBG_DEV_PRINT_INFO{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_DEVICE}(MSG, ...);
// FUNC      DBG_DEV_PRINT_VERBOSE{           LEVEL=TRACE_LEVEL_VERBOSE,     FLAGS=TRACE_DEVICE}(MSG, ...);
//
// USEPREFIX(DBG_RPC_METHOD_BEG,              "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_RPC_METHOD_BEG_WITH_PARAMS,  "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_RPC_METHOD_END,              "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_RPC_METHOD_END_WITH_PARAMS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_RPC_METHOD_END_WITH_STATUS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_RPC_PRINT_ERROR,             "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_RPC_PRINT_ERROR_WITH_STATUS, "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_RPC_PRINT_WARNING,           "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_RPC_PRINT_INFO,              "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_RPC_PRINT_VERBOSE,           "%!STDPREFIX!   %!FUNC!");
// USESUFFIX(DBG_RPC_METHOD_BEG,              ")");
// USESUFFIX(DBG_RPC_METHOD_BEG_WITH_PARAMS,  ")");
// USESUFFIX(DBG_RPC_METHOD_END,              ")");
// USESUFFIX(DBG_RPC_METHOD_END_WITH_PARAMS,  ")");
// USESUFFIX(DBG_RPC_METHOD_END_WITH_STATUS,  ") [%!STATUS!]", EXP);
// USESUFFIX(DBG_RPC_PRINT_ERROR_WITH_STATUS, " [%!STATUS!] !!!", EXP);
// FUNC      DBG_RPC_METHOD_BEG{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_RPC}(...);
// FUNC      DBG_RPC_METHOD_BEG_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_RPC}(MSG, ...);
// FUNC      DBG_RPC_METHOD_END{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_RPC}(...);
// FUNC      DBG_RPC_METHOD_END_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_RPC}(MSG, ...);
// FUNC      DBG_RPC_METHOD_END_WITH_STATUS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_RPC}(EXP);
// FUNC      DBG_RPC_PRINT_ERROR{             LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_RPC}(MSG, ...);
// FUNC      DBG_RPC_PRINT_ERROR_WITH_STATUS{ LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_RPC}(EXP, MSG, ...);
// FUNC      DBG_RPC_PRINT_WARNING{           LEVEL=TRACE_LEVEL_WARNING,     FLAGS=TRACE_RPC}(MSG, ...);
// FUNC      DBG_RPC_PRINT_INFO{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_RPC}(MSG, ...);
// FUNC      DBG_RPC_PRINT_VERBOSE{           LEVEL=TRACE_LEVEL_VERBOSE,     FLAGS=TRACE_RPC}(MSG, ...);
//
// USEPREFIX(DBG_IOCTL_METHOD_BEG,              "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_IOCTL_METHOD_BEG_WITH_PARAMS,  "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_IOCTL_METHOD_END,              "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_IOCTL_METHOD_END_WITH_PARAMS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_IOCTL_METHOD_END_WITH_STATUS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_IOCTL_PRINT_ERROR,             "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_IOCTL_PRINT_ERROR_WITH_STATUS, "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_IOCTL_PRINT_WARNING,           "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_IOCTL_PRINT_INFO,              "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_IOCTL_PRINT_VERBOSE,           "%!STDPREFIX!   %!FUNC!");
// USESUFFIX(DBG_IOCTL_METHOD_BEG,              ")");
// USESUFFIX(DBG_IOCTL_METHOD_BEG_WITH_PARAMS,  ")");
// USESUFFIX(DBG_IOCTL_METHOD_END,              ")");
// USESUFFIX(DBG_IOCTL_METHOD_END_WITH_PARAMS,  ")");
// USESUFFIX(DBG_IOCTL_METHOD_END_WITH_STATUS,  ") [%!STATUS!]", EXP);
// USESUFFIX(DBG_IOCTL_PRINT_ERROR_WITH_STATUS, " [%!STATUS!] !!!", EXP);
// FUNC      DBG_IOCTL_METHOD_BEG{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_IOCTL}(...);
// FUNC      DBG_IOCTL_METHOD_BEG_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_IOCTL}(MSG, ...);
// FUNC      DBG_IOCTL_METHOD_END{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_IOCTL}(...);
// FUNC      DBG_IOCTL_METHOD_END_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_IOCTL}(MSG, ...);
// FUNC      DBG_IOCTL_METHOD_END_WITH_STATUS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_IOCTL}(EXP);
// FUNC      DBG_IOCTL_PRINT_ERROR{             LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_IOCTL}(MSG, ...);
// FUNC      DBG_IOCTL_PRINT_ERROR_WITH_STATUS{ LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_IOCTL}(EXP, MSG, ...);
// FUNC      DBG_IOCTL_PRINT_WARNING{           LEVEL=TRACE_LEVEL_WARNING,     FLAGS=TRACE_IOCTL}(MSG, ...);
// FUNC      DBG_IOCTL_PRINT_INFO{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_IOCTL}(MSG, ...);
// FUNC      DBG_IOCTL_PRINT_VERBOSE{           LEVEL=TRACE_LEVEL_VERBOSE,     FLAGS=TRACE_IOCTL}(MSG, ...);
//
// USEPREFIX(DBG_VDEC_METHOD_BEG,              "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_VDEC_METHOD_BEG_WITH_PARAMS,  "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_VDEC_METHOD_END,              "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_VDEC_METHOD_END_WITH_PARAMS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_VDEC_METHOD_END_WITH_STATUS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_VDEC_PRINT_ERROR,             "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_VDEC_PRINT_ERROR_WITH_STATUS, "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_VDEC_PRINT_WARNING,           "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_VDEC_PRINT_INFO,              "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_VDEC_PRINT_VERBOSE,           "%!STDPREFIX!   %!FUNC!");
// USESUFFIX(DBG_VDEC_METHOD_BEG,              ")");
// USESUFFIX(DBG_VDEC_METHOD_BEG_WITH_PARAMS,  ")");
// USESUFFIX(DBG_VDEC_METHOD_END,              ")");
// USESUFFIX(DBG_VDEC_METHOD_END_WITH_PARAMS,  ")");
// USESUFFIX(DBG_VDEC_METHOD_END_WITH_STATUS,  ") [%!STATUS!]", EXP);
// USESUFFIX(DBG_VDEC_PRINT_ERROR_WITH_STATUS, " [%!STATUS!] !!!", EXP);
// FUNC      DBG_VDEC_METHOD_BEG{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_VDEC}(...);
// FUNC      DBG_VDEC_METHOD_BEG_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_VDEC}(MSG, ...);
// FUNC      DBG_VDEC_METHOD_END{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_VDEC}(...);
// FUNC      DBG_VDEC_METHOD_END_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_VDEC}(MSG, ...);
// FUNC      DBG_VDEC_METHOD_END_WITH_STATUS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_VDEC}(EXP);
// FUNC      DBG_VDEC_PRINT_ERROR{             LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_VDEC}(MSG, ...);
// FUNC      DBG_VDEC_PRINT_ERROR_WITH_STATUS{ LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_VDEC}(EXP, MSG, ...);
// FUNC      DBG_VDEC_PRINT_WARNING{           LEVEL=TRACE_LEVEL_WARNING,     FLAGS=TRACE_VDEC}(MSG, ...);
// FUNC      DBG_VDEC_PRINT_INFO{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_VDEC}(MSG, ...);
// FUNC      DBG_VDEC_PRINT_VERBOSE{           LEVEL=TRACE_LEVEL_VERBOSE,     FLAGS=TRACE_VDEC}(MSG, ...);
// //
// USEPREFIX(DBG_FBL_METHOD_BEG,              "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_FBL_METHOD_BEG_WITH_PARAMS,  "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_FBL_METHOD_END,              "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_FBL_METHOD_END_WITH_PARAMS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_FBL_METHOD_END_WITH_STATUS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_FBL_PRINT_ERROR,             "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_FBL_PRINT_ERROR_WITH_STATUS, "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_FBL_PRINT_WARNING,           "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_FBL_PRINT_INFO,              "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_FBL_PRINT_VERBOSE,           "%!STDPREFIX!   %!FUNC!");
// USESUFFIX(DBG_FBL_METHOD_BEG,              ")");
// USESUFFIX(DBG_FBL_METHOD_BEG_WITH_PARAMS,  ")");
// USESUFFIX(DBG_FBL_METHOD_END,              ")");
// USESUFFIX(DBG_FBL_METHOD_END_WITH_PARAMS,  ")");
// USESUFFIX(DBG_FBL_METHOD_END_WITH_STATUS,  ") [%!STATUS!]", EXP);
// USESUFFIX(DBG_FBL_PRINT_ERROR_WITH_STATUS, " [%!STATUS!] !!!", EXP);
// FUNC      DBG_FBL_METHOD_BEG{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_FBL}(...);
// FUNC      DBG_FBL_METHOD_BEG_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_FBL}(MSG, ...);
// FUNC      DBG_FBL_METHOD_END{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_FBL}(...);
// FUNC      DBG_FBL_METHOD_END_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_FBL}(MSG, ...);
// FUNC      DBG_FBL_METHOD_END_WITH_STATUS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_FBL}(EXP);
// FUNC      DBG_FBL_PRINT_ERROR{             LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_FBL}(MSG, ...);
// FUNC      DBG_FBL_PRINT_ERROR_WITH_STATUS{ LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_FBL}(EXP, MSG, ...);
// FUNC      DBG_FBL_PRINT_WARNING{           LEVEL=TRACE_LEVEL_WARNING,     FLAGS=TRACE_FBL}(MSG, ...);
// FUNC      DBG_FBL_PRINT_INFO{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_FBL}(MSG, ...);
// FUNC      DBG_FBL_PRINT_VERBOSE{           LEVEL=TRACE_LEVEL_VERBOSE,     FLAGS=TRACE_FBL}(MSG, ...);
// // //
// USEPREFIX(DBG_EVENTS_METHOD_BEG,              "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_EVENTS_METHOD_BEG_WITH_PARAMS,  "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_EVENTS_METHOD_END,              "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_EVENTS_METHOD_END_WITH_PARAMS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_EVENTS_METHOD_END_WITH_STATUS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_EVENTS_PRINT_ERROR,             "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_EVENTS_PRINT_ERROR_WITH_STATUS, "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_EVENTS_PRINT_WARNING,           "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_EVENTS_PRINT_INFO,              "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_EVENTS_PRINT_VERBOSE,           "%!STDPREFIX!   %!FUNC!");
// USESUFFIX(DBG_EVENTS_METHOD_BEG,              ")");
// USESUFFIX(DBG_EVENTS_METHOD_BEG_WITH_PARAMS,  ")");
// USESUFFIX(DBG_EVENTS_METHOD_END,              ")");
// USESUFFIX(DBG_EVENTS_METHOD_END_WITH_PARAMS,  ")");
// USESUFFIX(DBG_EVENTS_METHOD_END_WITH_STATUS,  ") [%!STATUS!]", EXP);
// USESUFFIX(DBG_EVENTS_PRINT_ERROR_WITH_STATUS, " [%!STATUS!] !!!", EXP);
// FUNC      DBG_EVENTS_METHOD_BEG{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(...);
// FUNC      DBG_EVENTS_METHOD_BEG_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(MSG, ...);
// FUNC      DBG_EVENTS_METHOD_END{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(...);
// FUNC      DBG_EVENTS_METHOD_END_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(MSG, ...);
// FUNC      DBG_EVENTS_METHOD_END_WITH_STATUS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(EXP);
// FUNC      DBG_EVENTS_PRINT_ERROR{             LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_EVENTS}(MSG, ...);
// FUNC      DBG_EVENTS_PRINT_ERROR_WITH_STATUS{ LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_EVENTS}(EXP, MSG, ...);
// FUNC      DBG_EVENTS_PRINT_WARNING{           LEVEL=TRACE_LEVEL_WARNING,     FLAGS=TRACE_EVENTS}(MSG, ...);
// FUNC      DBG_EVENTS_PRINT_INFO{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(MSG, ...);
// FUNC      DBG_EVENTS_PRINT_VERBOSE{           LEVEL=TRACE_LEVEL_VERBOSE,     FLAGS=TRACE_EVENTS}(MSG, ...);
// //
// USEPREFIX(DBG_PERF_METHOD_BEG,              "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_PERF_METHOD_BEG_WITH_PARAMS,  "%!STDPREFIX!+++%!FUNC!(");
// USEPREFIX(DBG_PERF_METHOD_END,              "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_PERF_METHOD_END_WITH_PARAMS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_PERF_METHOD_END_WITH_STATUS,  "%!STDPREFIX!---%!FUNC!(");
// USEPREFIX(DBG_PERF_PRINT_ERROR,             "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_PERF_PRINT_ERROR_WITH_STATUS, "%!STDPREFIX!!!!%!FUNC!");
// USEPREFIX(DBG_PERF_PRINT_WARNING,           "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_PERF_PRINT_INFO,              "%!STDPREFIX!   %!FUNC!");
// USEPREFIX(DBG_PERF_PRINT_VERBOSE,           "%!STDPREFIX!   %!FUNC!");
// USESUFFIX(DBG_PERF_METHOD_BEG,              ")");
// USESUFFIX(DBG_PERF_METHOD_BEG_WITH_PARAMS,  ")");
// USESUFFIX(DBG_PERF_METHOD_END,              ")");
// USESUFFIX(DBG_PERF_METHOD_END_WITH_PARAMS,  ")");
// USESUFFIX(DBG_PERF_METHOD_END_WITH_STATUS,  ") [%!STATUS!]", EXP);
// USESUFFIX(DBG_PERF_PRINT_ERROR_WITH_STATUS, " [%!STATUS!] !!!", EXP);
// FUNC      DBG_PERF_METHOD_BEG{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(...);
// FUNC      DBG_PERF_METHOD_BEG_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(MSG, ...);
// FUNC      DBG_PERF_METHOD_END{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(...);
// FUNC      DBG_PERF_METHOD_END_WITH_PARAMS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(MSG, ...);
// FUNC      DBG_PERF_METHOD_END_WITH_STATUS{  LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(EXP);
// FUNC      DBG_PERF_PRINT_ERROR{             LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_EVENTS}(MSG, ...);
// FUNC      DBG_PERF_PRINT_ERROR_WITH_STATUS{ LEVEL=TRACE_LEVEL_ERROR,       FLAGS=TRACE_EVENTS}(EXP, MSG, ...);
// FUNC      DBG_PERF_PRINT_WARNING{           LEVEL=TRACE_LEVEL_WARNING,     FLAGS=TRACE_EVENTS}(MSG, ...);
// FUNC      DBG_PERF_PRINT_INFO{              LEVEL=TRACE_LEVEL_INFORMATION, FLAGS=TRACE_EVENTS}(MSG, ...);
// FUNC      DBG_PERF_PRINT_VERBOSE{           LEVEL=TRACE_LEVEL_VERBOSE,     FLAGS=TRACE_EVENTS}(MSG, ...);
// end_wpp
//

#elif (MALONE_TRACE == MALONE_TRACE_DBG_PRINT_EX)

#define WPP_INIT_TRACING(...)  KeQuerySystemTimePrecise(&DriverStartTime);
#define WPP_CLEANUP(...)

extern LARGE_INTEGER        DriverStartTime;

static inline LONG GetTime()
{
    LARGE_INTEGER  CurrentSystemTime;
    KeQuerySystemTimePrecise(&CurrentSystemTime);
    return (LONG)((CurrentSystemTime.QuadPart - DriverStartTime.QuadPart) / 10000);
}

// ...

#ifdef PRINT
    #define DBG_PRINT_ERROR(_format_str_,...)                      DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s: "_format_str_"\n"          ,GetTime() ,KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__VA_ARGS__)
    #define DBG_PRINT_ERROR_WITH_STATUS(_status_,_format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s: "_format_str_" [0x%.8X]\n" ,GetTime() ,KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__VA_ARGS__,_status_)
    #define DBG_PRINT_INFO(_format_str_,...)                       DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s: "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__VA_ARGS__)

#else // PRINT
    #define DBG_PRINT_ERROR(...)
    #define DBG_PRINT_ERROR_WITH_STATUS(...)
#endif // PRINT


#ifdef DRV
    #define DBG_DRV_METHOD_BEG()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_DRV_METHOD_BEG_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_DRV_METHOD_END()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_DRV_METHOD_END_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_DRV_METHOD_END_WITH_STATUS(_status_)                    DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X]\n"             ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,_status_)
    #define DBG_DRV_PRINT_ERROR(_format_str_,...)                       DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_DRV_PRINT_ERROR_WITH_STATUS(_status_, _format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_" [0x%.8X]\n" ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__,_status_)
    #define DBG_DRV_PRINT_WARNING(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_DRV_PRINT_VERBOSE(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_DRV_PRINT_INFO(_format_str_,...)                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#else // DRV
    #define DBG_DRV_METHOD_BEG(...)
    #define DBG_DRV_METHOD_BEG_WITH_PARAMS(...)
    #define DBG_DRV_METHOD_END(...)
    #define DBG_DRV_METHOD_END_WITH_PARAMS(...)
    #define DBG_DRV_METHOD_END_WITH_STATUS(...)
    #define DBG_DRV_PRINT_ERROR(...)
    #define DBG_DRV_PRINT_ERROR_WITH_STATUS(...)
    #define DBG_DRV_PRINT_WARNING(...)
    #define DBG_DRV_PRINT_VERBOSE(...)
    #define DBG_DRV_PRINT_INFO(...)
#endif // DRV


#ifdef DEV
    #define DBG_DEV_METHOD_BEG()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_DEV_METHOD_BEG_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_DEV_METHOD_END()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_DEV_METHOD_END_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_DEV_METHOD_END_WITH_STATUS(_status_)                    DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X]\n"             ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,_status_)
    #define DBG_DEV_PRINT_ERROR(_format_str_,...)                       DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_DEV_PRINT_ERROR_WITH_STATUS(_status_, _format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_" [0x%.8X]\n" ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__,_status_)
    #define DBG_DEV_PRINT_WARNING(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_DEV_PRINT_VERBOSE(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_DEV_PRINT_INFO(_format_str_,...)                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#else // DEV
    #define DBG_DEV_METHOD_BEG(...)
    #define DBG_DEV_METHOD_BEG_WITH_PARAMS(...)
    #define DBG_DEV_METHOD_END(...)
    #define DBG_DEV_METHOD_END_WITH_PARAMS(...)
    #define DBG_DEV_METHOD_END_WITH_STATUS(...)
    #define DBG_DEV_PRINT_ERROR(...)
    #define DBG_DEV_PRINT_ERROR_WITH_STATUS(...)
    #define DBG_DEV_PRINT_WARNING(...)
    #define DBG_DEV_PRINT_VERBOSE(...)
    #define DBG_DEV_PRINT_INFO(...)
#endif // DEV

#ifdef RPC
    #define DBG_RPC_METHOD_BEG()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_RPC_METHOD_BEG_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_RPC_METHOD_END()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_RPC_METHOD_END_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_RPC_METHOD_END_WITH_STATUS(_status_)                    DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X]\n"             ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,_status_)
    #define DBG_RPC_PRINT_ERROR(_format_str_,...)                       DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_RPC_PRINT_ERROR_WITH_STATUS(_status_, _format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_" [0x%.8X]\n" ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__,_status_)
    #define DBG_RPC_PRINT_WARNING(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_RPC_PRINT_VERBOSE(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_RPC_PRINT_INFO(_format_str_,...)                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#else // RPC
    #define DBG_RPC_METHOD_BEG(...)
    #define DBG_RPC_METHOD_BEG_WITH_PARAMS(...)
    #define DBG_RPC_METHOD_END(...)
    #define DBG_RPC_METHOD_END_WITH_PARAMS(...)
    #define DBG_RPC_METHOD_END_WITH_STATUS(...)
    #define DBG_RPC_PRINT_ERROR(...)
    #define DBG_RPC_PRINT_ERROR_WITH_STATUS(...)
    #define DBG_RPC_PRINT_WARNING(...)
    #define DBG_RPC_PRINT_VERBOSE(...)
    #define DBG_RPC_PRINT_INFO(...)
#endif // RPC


#ifdef IOCTL
    #define DBG_IOCTL_METHOD_BEG()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_IOCTL_METHOD_BEG_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_IOCTL_METHOD_END()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_IOCTL_METHOD_END_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_IOCTL_METHOD_END_WITH_STATUS(_status_)                    DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X]\n"             ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,_status_)
    #define DBG_IOCTL_PRINT_ERROR(_format_str_,...)                       DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_IOCTL_PRINT_ERROR_WITH_STATUS(_status_, _format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_" [0x%.8X]\n" ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__,_status_)
    #define DBG_IOCTL_PRINT_WARNING(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_IOCTL_PRINT_VERBOSE(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_IOCTL_PRINT_INFO(_format_str_,...)                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#else // IOCTL
    #define DBG_IOCTL_METHOD_BEG(...)
    #define DBG_IOCTL_METHOD_BEG_WITH_PARAMS(...)
    #define DBG_IOCTL_METHOD_END(...)
    #define DBG_IOCTL_METHOD_END_WITH_PARAMS(...)
    #define DBG_IOCTL_METHOD_END_WITH_STATUS(...)
    #define DBG_IOCTL_PRINT_ERROR(...)
    #define DBG_IOCTL_PRINT_ERROR_WITH_STATUS(...)
    #define DBG_IOCTL_PRINT_WARNING(...)
    #define DBG_IOCTL_PRINT_VERBOSE(...)
    #define DBG_IOCTL_PRINT_INFO(...)
#endif // IOCTL

#ifdef VDEC
    #define DBG_VDEC_METHOD_BEG()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_VDEC_METHOD_BEG_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_VDEC_METHOD_END()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_VDEC_METHOD_END_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_VDEC_METHOD_END_WITH_STATUS(_status_)                    DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X]\n"             ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,_status_)
    #define DBG_VDEC_PRINT_ERROR(_format_str_,...)                       DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_VDEC_PRINT_ERROR_WITH_STATUS(_status_, _format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_" [0x%.8X]\n" ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__,_status_)
    #define DBG_VDEC_PRINT_WARNING(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_VDEC_PRINT_VERBOSE(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_VDEC_PRINT_INFO(_format_str_,...)                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#else // VDEC
    #define DBG_VDEC_METHOD_BEG(...)
    #define DBG_VDEC_METHOD_BEG_WITH_PARAMS(...)
    #define DBG_VDEC_METHOD_END(...)
    #define DBG_VDEC_METHOD_END_WITH_PARAMS(...)
    #define DBG_VDEC_METHOD_END_WITH_STATUS(...)
    #define DBG_VDEC_PRINT_ERROR(...)
    #define DBG_VDEC_PRINT_ERROR_WITH_STATUS(...)
    #define DBG_VDEC_PRINT_WARNING(...)
    #define DBG_VDEC_PRINT_VERBOSE(...)
    #define DBG_VDEC_PRINT_INFO(...)
#endif // VDEC


#ifdef FBL
    #define DBG_FBL_METHOD_BEG()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_FBL_METHOD_BEG_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_FBL_METHOD_END()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_FBL_METHOD_END_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_FBL_METHOD_END_WITH_STATUS(_status_)                    DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X]\n"             ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,_status_)
    #define DBG_FBL_PRINT_ERROR(_format_str_,...)                       DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_FBL_PRINT_ERROR_WITH_STATUS(_status_, _format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_" [0x%.8X]\n" ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__,_status_)
    #define DBG_FBL_PRINT_WARNING(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_FBL_PRINT_VERBOSE(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_FBL_PRINT_INFO(_format_str_,...)                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#else // FBL
    #define DBG_FBL_METHOD_BEG(...)
    #define DBG_FBL_METHOD_BEG_WITH_PARAMS(...)
    #define DBG_FBL_METHOD_END(...)
    #define DBG_FBL_METHOD_END_WITH_PARAMS(...)
    #define DBG_FBL_METHOD_END_WITH_STATUS(...)
    #define DBG_FBL_PRINT_ERROR(...)
    #define DBG_FBL_PRINT_ERROR_WITH_STATUS(...)
    #define DBG_FBL_PRINT_WARNING(...)
    #define DBG_FBL_PRINT_VERBOSE(...)
    #define DBG_FBL_PRINT_INFO(...)
#endif // FBL

#ifdef EVENTS
    #define DBG_EVENTS_METHOD_BEG()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_EVENTS_METHOD_BEG_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_EVENTS_METHOD_END()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
    #define DBG_EVENTS_METHOD_END_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_EVENTS_METHOD_END_WITH_STATUS(_status_)                    DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X]\n"             ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,_status_)
    #define DBG_EVENTS_PRINT_ERROR(_format_str_,...)                       DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_EVENTS_PRINT_ERROR_WITH_STATUS(_status_, _format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_" [0x%.8X]\n" ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__,_status_)
    #define DBG_EVENTS_PRINT_WARNING(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_EVENTS_PRINT_VERBOSE(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
    #define DBG_EVENTS_PRINT_INFO(_format_str_,...)                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__VA_ARGS__)
#else // EVENTS
    #define DBG_EVENTS_METHOD_BEG(...)
    #define DBG_EVENTS_METHOD_BEG_WITH_PARAMS(...)
    #define DBG_EVENTS_METHOD_END(...)
    #define DBG_EVENTS_METHOD_END_WITH_PARAMS(...)
    #define DBG_EVENTS_METHOD_END_WITH_STATUS(...)
    #define DBG_EVENTS_PRINT_ERROR(...)
    #define DBG_EVENTS_PRINT_ERROR_WITH_STATUS(...)
    #define DBG_EVENTS_PRINT_WARNING(...)
    #define DBG_EVENTS_PRINT_VERBOSE(...)
    #define DBG_EVENTS_PRINT_INFO(...)
#endif // EVENTS

#ifdef PERF
#define DBG_PERF_METHOD_BEG()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
#define DBG_PERF_METHOD_BEG_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:+++%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#define DBG_PERF_METHOD_END()                                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s()\n"                      ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__)
#define DBG_PERF_METHOD_END_WITH_PARAMS(_format_str_,...)            DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s("_format_str_")\n"        ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#define DBG_PERF_METHOD_END_WITH_STATUS(_status_)                    DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:---%s() [0x%.8X]\n"             ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,_status_)
#define DBG_PERF_PRINT_ERROR(_format_str_,...)                       DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#define DBG_PERF_PRINT_ERROR_WITH_STATUS(_status_, _format_str_,...) DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:!!!%s "_format_str_" [0x%.8X]\n" ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__,_status_)
#define DBG_PERF_PRINT_WARNING(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#define DBG_PERF_PRINT_VERBOSE(_format_str_,...)                     DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#define DBG_PERF_PRINT_INFO(_format_str_,...)                        DbgPrintEx(DPFLTR_IHVDRIVER_ID,0xFFFFFFFE,"%08d C%d D%d %s:   %s "_format_str_"\n"          ,GetTime(),KeGetCurrentProcessorNumber(),KeGetCurrentIrql(),"MaloneDrv",__FUNCTION__,__VA_ARGS__)
#else // EVENTS
#define DBG_PERF_METHOD_BEG(...)
#define DBG_PERF_METHOD_BEG_WITH_PARAMS(...)
#define DBG_PERF_METHOD_END(...)
#define DBG_PERF_METHOD_END_WITH_PARAMS(...)
#define DBG_PERF_METHOD_END_WITH_STATUS(...)
#define DBG_PERF_PRINT_ERROR(...)
#define DBG_PERF_PRINT_ERROR_WITH_STATUS(...)
#define DBG_PERF_PRINT_WARNING(...)
#define DBG_PERF_PRINT_VERBOSE(...)
#define DBG_PERF_PRINT_INFO(...)
#endif // EVENTS

#else

#define WPP_INIT_TRACING(...)
#define WPP_CLEANUP(...)

#define DBG_PRINT_ERROR(...)
#define DBG_PRINT_ERROR_WITH_STATUS(...)
#define DBG_DRV_METHOD_BEG(...)
#define DBG_DRV_METHOD_BEG_WITH_PARAMS(...)
#define DBG_DRV_METHOD_END(...)
#define DBG_DRV_METHOD_END_WITH_PARAMS(...)
#define DBG_DRV_METHOD_END_WITH_STATUS(...)
#define DBG_DRV_PRINT_ERROR(...)
#define DBG_DRV_PRINT_ERROR_WITH_STATUS(...)
#define DBG_DRV_PRINT_WARNING(...)
#define DBG_DRV_PRINT_VERBOSE(...)
#define DBG_DRV_PRINT_INFO(...)

#define DBG_DEV_METHOD_BEG(...)
#define DBG_DEV_METHOD_BEG_WITH_PARAMS(...)
#define DBG_DEV_METHOD_END(...)
#define DBG_DEV_METHOD_END_WITH_PARAMS(...)
#define DBG_DEV_METHOD_END_WITH_STATUS(...)
#define DBG_DEV_PRINT_ERROR(...)
#define DBG_DEV_PRINT_ERROR_WITH_STATUS(...)
#define DBG_DEV_PRINT_WARNING(...)
#define DBG_DEV_PRINT_VERBOSE(...)
#define DBG_DEV_PRINT_INFO(...)

#define DBG_RPC_METHOD_BEG(...)
#define DBG_RPC_METHOD_BEG_WITH_PARAMS(...)
#define DBG_RPC_METHOD_END(...)
#define DBG_RPC_METHOD_END_WITH_PARAMS(...)
#define DBG_RPC_METHOD_END_WITH_STATUS(...)
#define DBG_RPC_PRINT_ERROR(...)
#define DBG_RPC_PRINT_ERROR_WITH_STATUS(...)
#define DBG_RPC_PRINT_WARNING(...)
#define DBG_RPC_PRINT_VERBOSE(...)
#define DBG_RPC_PRINT_INFO(...)

#define DBG_IOCTL_METHOD_BEG(...)
#define DBG_IOCTL_METHOD_BEG_WITH_PARAMS(...)
#define DBG_IOCTL_METHOD_END(...)
#define DBG_IOCTL_METHOD_END_WITH_PARAMS(...)
#define DBG_IOCTL_METHOD_END_WITH_STATUS(...)
#define DBG_IOCTL_PRINT_ERROR(...)
#define DBG_IOCTL_PRINT_ERROR_WITH_STATUS(...)
#define DBG_IOCTL_PRINT_WARNING(...)
#define DBG_IOCTL_PRINT_VERBOSE(...)
#define DBG_IOCTL_PRINT_INFO(...)

#define DBG_VDEC_METHOD_BEG(...)
#define DBG_VDEC_METHOD_BEG_WITH_PARAMS(...)
#define DBG_VDEC_METHOD_END(...)
#define DBG_VDEC_METHOD_END_WITH_PARAMS(...)
#define DBG_VDEC_METHOD_END_WITH_STATUS(...)
#define DBG_VDEC_PRINT_ERROR(...)
#define DBG_VDEC_PRINT_ERROR_WITH_STATUS(...)
#define DBG_VDEC_PRINT_WARNING(...)
#define DBG_VDEC_PRINT_VERBOSE(...)
#define DBG_VDEC_PRINT_INFO(...)

#define DBG_FBL_METHOD_BEG(...)
#define DBG_FBL_METHOD_BEG_WITH_PARAMS(...)
#define DBG_FBL_METHOD_END(...)
#define DBG_FBL_METHOD_END_WITH_PARAMS(...)
#define DBG_FBL_METHOD_END_WITH_STATUS(...)
#define DBG_FBL_PRINT_ERROR(...)
#define DBG_FBL_PRINT_ERROR_WITH_STATUS(...)
#define DBG_FBL_PRINT_WARNING(...)
#define DBG_FBL_PRINT_VERBOSE(...)
#define DBG_FBL_PRINT_INFO(...)

#define DBG_EVENTS_METHOD_BEG(...)
#define DBG_EVENTS_METHOD_BEG_WITH_PARAMS(...)
#define DBG_EVENTS_METHOD_END(...)
#define DBG_EVENTS_METHOD_END_WITH_PARAMS(...)
#define DBG_EVENTS_METHOD_END_WITH_STATUS(...)
#define DBG_EVENTS_PRINT_ERROR(...)
#define DBG_EVENTS_PRINT_ERROR_WITH_STATUS(...)
#define DBG_EVENTS_PRINT_WARNING(...)
#define DBG_EVENTS_PRINT_VERBOSE(...)
#define DBG_EVENTS_PRINT_INFO(...)

#define DBG_PERF_METHOD_BEG(...)
#define DBG_PERF_METHOD_BEG_WITH_PARAMS(...)
#define DBG_PERF_METHOD_END(...)
#define DBG_PERF_METHOD_END_WITH_PARAMS(...)
#define DBG_PERF_METHOD_END_WITH_STATUS(...)
#define DBG_PERF_PRINT_ERROR(...)
#define DBG_PERF_PRINT_ERROR_WITH_STATUS(...)
#define DBG_PERF_PRINT_WARNING(...)
#define DBG_PERF_PRINT_VERBOSE(...)
#define DBG_PERF_PRINT_INFO(...)

#endif


#endif
