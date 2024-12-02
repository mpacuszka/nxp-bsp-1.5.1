/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include <ntddk.h>

inline UINT32 ReadDoubleWord(BYTE* Address)
{
    return *reinterpret_cast<const volatile UINT32*>(Address);
}

inline void WriteDoubleWord(BYTE* Address, UINT32 Value)
{
    *reinterpret_cast<volatile UINT32*>(Address) = Value;
}

inline void UpdateDoubleWord(BYTE* Address, UINT32 Value, UINT32 Mask)
{
    WriteDoubleWord(Address, (ReadDoubleWord(Address) & ~Mask) | Value);
}