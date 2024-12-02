// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//    LPSPIcommon.h
//
// Abstract:
//
//    This module contains common enum, types, and other definitions common
//    to the IMX LPSPI controller driver device driver.
//    This controller driver uses the SPB WDF class extension (SpbCx).
//
// Environment:
//
//    kernel-mode only
//

#ifndef _LPSPI_COMMON_H_
#define _LPSPI_COMMON_H_

WDF_EXTERN_C_START


//
// LPSPI pool allocation tags
//
enum class LPSPI_ALLOC_TAG : ULONG {

    LPSPI_ALLOC_TAG_TEMP    = '0IPS', // Temporary be freed in the same routine
    LPSPI_ALLOC_TAG_WDF      = '@IPS'  // Allocations WDF makes on our behalf

}; // enum LPSPI_ALLOC_TAG


//
// Forward declaration of the LPSPI extension/contexts
//

typedef struct _LPSPI_DEVICE_EXTENSION LPSPI_DEVICE_EXTENSION;
typedef struct _LPSPI_TARGET_CONTEXT LPSPI_TARGET_CONTEXT;
typedef struct _LPSPI_TARGET_SETTINGS LPSPI_TARGET_SETTINGS;
typedef struct _LPSPI_SPB_REQUEST LPSPI_SPB_REQUEST;
typedef struct _LPSPI_SPB_TRANSFER LPSPI_SPB_TRANSFER;


//
// LPSPI common public methods
//

//
// Routine Description:
//
//  UlongWordSwap swaps the two 16bit words of a 32 bit value.
//
// Arguments:
//
//  Source - The original 32 bit value.
//
// Return Value:
//
//  A ULONG value with the 16bit words swapped.
//
__forceinline
ULONG
UlongWordSwap(
    _In_ ULONG Source
    )
{
    return ((Source & 0xFFFF) << 16) | (Source >> 16);
}


WDF_EXTERN_C_END


#endif // !_LPSPI_COMMON_H_
