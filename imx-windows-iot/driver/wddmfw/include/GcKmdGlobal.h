/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include "GcKmd.h"
#include "gcdispif.h"

class GcKmdGlobal
{
public:

    static void DdiUnload(void);

    static void DdiControlEtwLogging(
        IN_BOOLEAN  Enable,
        IN_ULONG    Flags,
        IN_UCHAR    Level);

    static NTSTATUS DriverEntry(__in IN DRIVER_OBJECT* pDriverObject, __in IN UNICODE_STRING* pRegistryPath);

    static DRIVER_OBJECT* s_pDriverObject;

    static GcKmdDriverMode  s_DriverMode;

    static BOOLEAN s_bShadowGpuVa;
    static BOOLEAN s_bSwitchGpuVa;
    static BOOLEAN s_bLocalVidMemGeneralUse;
    static BOOLEAN s_bGdiHwAcceleration;
    static BOOLEAN s_bMapDummyPage;

    static UINT    s_GdiDmaBufferSize;
    static UINT    s_DmaBufferSize;

    static BOOLEAN s_bDodUseHwVSync;

    static BOOLEAN s_bLimitAllocBelow4gbPa;
};
