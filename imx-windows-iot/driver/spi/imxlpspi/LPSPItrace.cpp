// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//    LPSPItrace.cpp
//
// Abstract:
//
//    This module contains implementation of various trace/debugging support
//    routines for the IMX LPSPI controller driver.
//
// Environment:
//
//    kernel-mode only
//
#include "precomp.h"
#pragma hdrstop

#define _LPSPI_TRACE_CPP_

// Logging
#include "LPSPItrace.h"
#include "LPSPItrace.tmh"

// Module specific header files


//
// Routine Description:
//
//  LPSPIIsDebuggerPresent is called to check if the debugger is present
//  and enabled.
//
// Arguments:
//
// Return Value:
//  
//  TRUE debugger is present, otherwise FALSE.
//
BOOLEAN
LPSPIIsDebuggerPresent ()
{
    static LONG isDebuggerStateUpToDate = FALSE;

    if (InterlockedExchange(&isDebuggerStateUpToDate, TRUE) == 0) {

        KdRefreshDebuggerNotPresent();
    }

    return (KD_DEBUGGER_ENABLED && !KD_DEBUGGER_NOT_PRESENT);
}


//
// Routine Description:
//
//  LPSPIBreakPoint is called by LPSPI_ASSERT to break if debugger
//  is present and enabled.
//
// Arguments:
//
// Return Value:
//
//  Always TRUE to continue...
//
BOOLEAN
LPSPIBreakPoint ()
{
    if (LPSPIIsDebuggerPresent()) {

        DbgBreakPoint();
    }

    return TRUE;
}


#undef _LPSPI_TRACE_CPP_
