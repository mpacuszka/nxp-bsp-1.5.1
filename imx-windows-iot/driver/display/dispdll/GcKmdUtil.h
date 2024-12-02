/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include "GcKmdAllocation.h"

typedef enum _Gc7LHwTileMode
{
    Linear,
    StandardTiled,
    SuperTiled
} Gc7LHwTileMode;

//
// Macros to be used for proper PAGED/NON-PAGED code placement
//

#define GC_NONPAGED_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    //__pragma(code_seg(.text))

#define GC_NONPAGED_SEGMENT_END \
    __pragma(code_seg(pop))

#define GC_PAGED_SEGMENT_BEGIN \
    __pragma(code_seg(push)) \
    __pragma(code_seg("PAGE"))

#define GC_PAGED_SEGMENT_END \
    __pragma(code_seg(pop))

//
// We have some non-paged functions that supposed to be called on low IRQL.
// The following macro defines unified assert to be put at the beginning of
// such functions.
//
// NOTE: We can't use standard PAGED_CODE macro as it requires function to be
// placed in paged segment during compilation.
//
#define GC_ASSERT_MAX_IRQL(Irql) NT_ASSERT(KeGetCurrentIrql() <= (Irql))
#define GC_ASSERT_LOW_IRQL() GC_ASSERT_MAX_IRQL(APC_LEVEL)

//
// KMD new and delete operator
//

void* __cdecl operator new (
    size_t      size,
    POOL_TYPE   poolType,
    UINT        tag
    ) throw ();

void __cdecl operator delete (
    void   *ptr,
    size_t  size
    ) throw ();

DXGI_FORMAT __cdecl TranslateD3dDdiToDxgiFormat(D3DDDIFORMAT Format);
UINT32 __cdecl TranslateD3dDdiToDrmFormat(D3DDDIFORMAT Format);
UINT32 __cdecl TranslateDxgiToDrmFormat(DXGI_FORMAT format);
UINT64 __cdecl GetDrmTileFromHwTile(Gc7LHwTileMode TileMode);
