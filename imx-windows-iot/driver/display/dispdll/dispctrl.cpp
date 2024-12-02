/* Copyright (c) Microsoft Corporation.
 * Copyright 2022-2024 NXP
   Licensed under the MIT License. */

#include "precomp.h"
#include "GcKmdGlobal.h"
#include "GcKmdBaseDisplay.h"
#include "GcKmdImx8Display.h"
#include "GcKmdImx8mqDisplay.h"
#include "GcKmdImx8mpDisplay.h"
#include "GcKmdImx8mpHdmiDisplay.h"
#include "GcKmdImx8mpMipiDsiDisplay.h"
#include "GcKmdImx8mpDisplayController.h"
#include "GcKmdImx8qxpDisplayController.h"
#include "GcKmdImx8mnDisplay.h"
#include "GcKmdImx8qxpDisplay.h"
#include "GcKmdImx8qxpMipiDsiDisplay.h"
#include "GcKmdUtil.h"

#include "GcKmdLogging.h"
#include "dispctrl.tmh"
#include "getresrc.h"



extern "C"
{

NTSTATUS DllInitialize(
    _In_ PUNICODE_STRING RegistryPath)
{
    UNREFERENCED_PARAMETER(RegistryPath);

    return STATUS_SUCCESS;
}

NTSTATUS DllUnload(
    void)
{
    return STATUS_SUCCESS;
}


GcKmdDriverMode GcKmdGlobal::s_DriverMode = FullDriver;
BOOLEAN         GcKmdGlobal::s_bDodUseHwVSync = TRUE;


void
DisplayStopController(
    GcKmDisplay    *pGcDisplay)
{
    GcKmBaseDisplay    *pDisplay = (GcKmBaseDisplay*)pGcDisplay;

    WPP_CLEANUP(NULL);

    if (pDisplay)
    {
        pDisplay->Stop();
        delete pDisplay;
    }
}

//
// helpers to get display instance based on the platform
//
enum PlatformName {
    iMX8MM,
    iMX8MN,
    iMX8MQ,
    iMX8MP,
    iMX8QXP,
    PLAT_NAME_UNDEF,
};

struct Platform {
    enum PlatformName PlatformName;
    WCHAR* DeviceId;
};

static const struct Platform PlatformList[] = {
    { iMX8MQ,  L"ACPI\\VEN_VERI&DEV_7000" },
    { iMX8MP,  L"ACPI\\VEN_VERI&DEV_7001" },
    { iMX8MM,  L"ACPI\\VEN_VERI&DEV_7002" },
    { iMX8MN,  L"ACPI\\VEN_VERI&DEV_7003" },
    { iMX8QXP, L"ACPI\\VEN_VERI&DEV_7004" },
};


static NTSTATUS GetDeviceHID(
    DXGKRNL_INTERFACE* pDxgkInterface,
    ULONG              BufferLength,
    WCHAR*             DeviceId,
    ULONG*             ResultLen)
{
    DXGK_DEVICE_INFO DeviceInfo;

    NTSTATUS Status = pDxgkInterface->DxgkCbGetDeviceInformation(
        pDxgkInterface->DeviceHandle,
        &DeviceInfo);
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    return IoGetDeviceProperty(
        DeviceInfo.PhysicalDeviceObject,
        DevicePropertyHardwareID, BufferLength,
        DeviceId, ResultLen);
}

static enum PlatformName GetPlatform(DXGKRNL_INTERFACE* pDxgkInterface)
{
    WCHAR DeviceId[512];
    ULONG ResultLen;

    NTSTATUS Status = GetDeviceHID(pDxgkInterface, sizeof(DeviceId), DeviceId, &ResultLen);
    if (!NT_SUCCESS(Status))
    {
        return PLAT_NAME_UNDEF;
    }

    for (int i = 0; i < ARRAYSIZE(PlatformList); i++)
    {
        if (!wcscmp(DeviceId, PlatformList[i].DeviceId))
        {
            return PlatformList[i].PlatformName;
        }
    }

    return PLAT_NAME_UNDEF;
}

static GcKmDisplay* GetDisplay(
    enum PlatformName   Name,
    DXGKRNL_INTERFACE*  pDxgkInterface)
{
    NTSTATUS    Status;
    ULONG   DisplaySel;
    ULONG   EnableMultiMon = 0;

    GetDwordRegistryParam(pDxgkInterface, L"EnableMultiMon", &EnableMultiMon);

    Status = GetDwordRegistryParam(pDxgkInterface, L"Display0Interface", &DisplaySel);
    if (!NT_SUCCESS(Status)) {
        DisplaySel = DISP_INTERFACE_DISABLED;
    }

    switch (Name)
    {
    case iMX8MM:
        //return new (NonPagedPoolNx, 'PSID') GcKmImx8mmDisplay();
        return nullptr;
    case iMX8MN:
        return new (NonPagedPoolNx, 'PSID') GcKmImx8mnDisplay();
    case iMX8MQ:
        return new (NonPagedPoolNx, 'PSID') GcKmImx8mqDisplay();
    case iMX8MP:
        if ((DisplayOnly == GcKmdGlobal::s_DriverMode) || (FALSE == EnableMultiMon))
        { 
            struct DisplayInterface di;
            di.UseDisplay = TRUE;
            di.RegistryIndex = 0;
            switch (DisplaySel)
            {
            case DISP_INTERFACE_LVDS0:
            case DISP_INTERFACE_LVDS_DUAL0:
                di.DisplayInterfaceIndex = 0;
                return new (NonPagedPoolNx, 'PSID') GcKmImx8mpDisplay(&di);
            case DISP_INTERFACE_LVDS1:
                di.DisplayInterfaceIndex = 1;
                return new (NonPagedPoolNx, 'PSID') GcKmImx8mpDisplay(&di);
            case DISP_INTERFACE_HDMI:
                return new (NonPagedPoolNx, 'PSID') GcKmImx8mpHdmiDisplay();
            case DISP_INTERFACE_MIPI_DSI0:
                return new (NonPagedPoolNx, 'PSID') GcKmImx8mpMipiDsiDisplay(&di);
            default:
                return nullptr;
            }
        }
        else
        {
            GcKmBaseDisplayController* dispctrl = new (NonPagedPoolNx, 'PSID') GcKmImx8mpDisplayController(pDxgkInterface);
            if (dispctrl && dispctrl->IsInitialized() == FALSE) {
                delete dispctrl;
                dispctrl = nullptr;
            }
            return dispctrl;
        }
    case iMX8QXP:
        if ((DisplayOnly == GcKmdGlobal::s_DriverMode) || (FALSE == EnableMultiMon))
        {
            struct DisplayInterface di;
            di.UseDisplay = TRUE;
            di.RegistryIndex = 0;
            switch (DisplaySel)
            {
            case DISP_INTERFACE_LVDS0:
            case DISP_INTERFACE_LVDS_DUAL0:
                di.DisplayInterfaceIndex = 0;
                return new (NonPagedPoolNx, 'PSID') GcKmImx8qxpDisplay(&di, nullptr);
            case DISP_INTERFACE_LVDS1:
                di.DisplayInterfaceIndex = 1;
                return new (NonPagedPoolNx, 'PSID') GcKmImx8qxpDisplay(&di, nullptr);
            case DISP_INTERFACE_MIPI_DSI0:
                di.DisplayInterfaceIndex = 0;
                return new (NonPagedPoolNx, 'PSID') GcKmImx8qxpMipiDsiDisplay(&di, nullptr);
            case DISP_INTERFACE_MIPI_DSI1:
                di.DisplayInterfaceIndex = 1;
                return new (NonPagedPoolNx, 'PSID') GcKmImx8qxpMipiDsiDisplay(&di, nullptr);
            default:
                return nullptr;
            }
        }
        else
        {
            GcKmBaseDisplayController* dispctrl = new (NonPagedPoolNx, 'PSID') GcKmImx8qxpDisplayController(pDxgkInterface);
            if (dispctrl && dispctrl->IsInitialized() == FALSE) {
                delete dispctrl;
                dispctrl = nullptr;
            }
            return dispctrl;
        }
    default:
        return nullptr;
    }
}

NTSTATUS
DisplayStartController(
    GcKmdDriverMode     DriverMode,
    DXGKRNL_INTERFACE  *pDxgkInterface,
    UINT                LocalVidMemPhysicalBase,
    GcKmDisplay       **ppDisplay,
    ULONG              *pNumberOfVideoPresentSources,
    ULONG              *pNumberOfChildren)
{
    NTSTATUS    Status;

    WPP_INIT_TRACING(NULL, NULL);

    *ppDisplay = nullptr;
    *pNumberOfVideoPresentSources = 0;
    *pNumberOfChildren = 0;

    GcKmdGlobal::s_DriverMode = DriverMode;

    if ((FullDriver != DriverMode) &&
        (DisplayOnly != DriverMode))
    {
        return STATUS_SUCCESS;
    }

    enum PlatformName PlatName = GetPlatform(pDxgkInterface);
    if (PlatName == PLAT_NAME_UNDEF)
    {
        return STATUS_INVALID_PARAMETER;
    }

    GcKmDisplay    *pDisplay = GetDisplay(PlatName, pDxgkInterface);
    if (nullptr == pDisplay)
    {
        return STATUS_NO_MEMORY;
    }

    *ppDisplay = pDisplay;

    Status = pDisplay->Start(
                        pDxgkInterface,
                        LocalVidMemPhysicalBase,
                        pNumberOfVideoPresentSources,
                        pNumberOfChildren);
    if (!NT_SUCCESS(Status))
    {
        DisplayStopController(pDisplay);
        *ppDisplay = nullptr;
        return Status;
    }

    return STATUS_SUCCESS;
}

}
