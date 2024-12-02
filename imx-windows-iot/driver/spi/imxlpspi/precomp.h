// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//    precomp.h
//
// Abstract:
//
//    This is the standard header for all of the source modules within the
//    IMX LPSPI controller driver.
//    This controller driver uses the SPB WDF class extension (SpbCx).
//
// Environment:
//
//    kernel-mode only
//

#include <ntddk.h>
#include <wdf.h>
#include <SpbCx.h>
#include <gpio.h>

#define RESHUB_USE_HELPER_ROUTINES
#include <reshub.h>

#pragma warning(disable:4201)   // nameless struct/union
