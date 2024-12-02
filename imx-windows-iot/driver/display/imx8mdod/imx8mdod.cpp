/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"

extern "C"
{

NTSTATUS
DriverEntry(
    _In_ PDRIVER_OBJECT   DriverObject,
    _In_ PUNICODE_STRING  RegistryPath)
{
    GcKmdGlobal::s_DriverMode = DisplayOnly;

    return GcKmdGlobal::DriverEntry(DriverObject, RegistryPath);
}

}

