// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2023 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//   device.cpp
//
// Abstract:
//
//  This module contains methods implementation for the device initialization.
//
// Environment:
//
//  Kernel mode only
//

#include "precomp.h"
#pragma hdrstop

#include "imxpwmhw.hpp"
#include "imxpwm.hpp"
#include "acpiutil.hpp"
#include "ImxCpuRev.h"
#include "trace.h"
#include "device.tmh"
#include "imxpwm_pofx.h"

IMXPWM_PAGED_SEGMENT_BEGIN; //==================================================

_Use_decl_annotations_
NTSTATUS
ImxPwmEvtDevicePrepareHardware (
    WDFDEVICE WdfDevice,
    WDFCMRESLIST /*ResourcesRaw*/,
    WDFCMRESLIST ResourcesTranslated
    )
{
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    const CM_PARTIAL_RESOURCE_DESCRIPTOR* memResourcePtr = nullptr;
    ULONG interruptResourceCount = 0;

    //
    // Look for single memory and interrupt resource.
    //
    const ULONG resourceCount = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < resourceCount; ++i) {
        const CM_PARTIAL_RESOURCE_DESCRIPTOR* resourcePtr =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);

        switch (resourcePtr->Type) {
        case CmResourceTypeMemory:
            if (memResourcePtr != nullptr) {
                IMXPWM_LOG_ERROR(
                    "Received unexpected memory resource. (resourcePtr = 0x%p)",
                    resourcePtr);

                return STATUS_DEVICE_CONFIGURATION_ERROR;
            }

            memResourcePtr = resourcePtr;
            break;

        case CmResourceTypeInterrupt:
            if (interruptResourceCount > 0) {
                IMXPWM_LOG_ERROR(
                    "Received unexpected interrupt resource. "
                    "(interruptResourceCount = %lu, resourcePtr = 0x%p)",
                    interruptResourceCount,
                    resourcePtr);

                return STATUS_DEVICE_CONFIGURATION_ERROR;
            }

            ++interruptResourceCount;
            break;
        }
    }

    if ((memResourcePtr == nullptr) || (interruptResourceCount < 1)) {
        IMXPWM_LOG_ERROR(
            "Did not receive required memory resource and interrupt resource. "
            "(memResourcePtr = 0x%p, interruptResourceCount = %lu)",
            memResourcePtr,
            interruptResourceCount);

        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }

    if (memResourcePtr->u.Memory.Length < sizeof(IMXPWM_REGISTERS)) {
        IMXPWM_LOG_ERROR(
            "Memory resource is too small. "
            "(memResourcePtr->u.Memory.Length = %lu, "
            "sizeof(IMXPWM_REGISTERS) = %lu)",
            memResourcePtr->u.Memory.Length,
            sizeof(IMXPWM_REGISTERS));

        return STATUS_DEVICE_CONFIGURATION_ERROR;
    }

    //
    // ReleaseHardware is ALWAYS called, even if PrepareHardware fails, so
    // the cleanup of registersPtr is handled there
    //
    IMXPWM_DEVICE_CONTEXT* deviceContextPtr =
            ImxPwmGetDeviceContext(WdfDevice);

    NT_ASSERT(memResourcePtr->Type == CmResourceTypeMemory);
    deviceContextPtr->RegistersPtr = static_cast<IMXPWM_REGISTERS*>(
        MmMapIoSpaceEx(
            memResourcePtr->u.Memory.Start,
            sizeof(IMXPWM_REGISTERS),
            PAGE_READWRITE | PAGE_NOCACHE));

    if (deviceContextPtr->RegistersPtr == nullptr) {
        IMXPWM_LOG_LOW_MEMORY(
            "MmMapIoSpaceEx(...) failed. "
            "(memResourcePtr->u.Memory.Start = 0x%llx, "
            "sizeof(IMXPWM_REGISTERS) = %lu)",
            memResourcePtr->u.Memory.Start.QuadPart,
            sizeof(IMXPWM_REGISTERS));

        return STATUS_INSUFFICIENT_RESOURCES;
    }

    // Pull optional Pwm-SchematicName value from ACPI _DSD table and publish as SchematicName property
    DEVICE_OBJECT* pdoPtr = WdfDeviceWdmGetPhysicalDevice(WdfDevice);
    NT_ASSERT(pdoPtr != nullptr);

    ACPI_EVAL_OUTPUT_BUFFER *dsdBufferPtr = nullptr;
    NTSTATUS status;

    status = AcpiQueryDsd(pdoPtr, &dsdBufferPtr);
    if (!NT_SUCCESS(status))
    {
        IMXPWM_LOG_WARNING(
            "AcpiQueryDsd() failed with error %!STATUS!",
            status);
        status = STATUS_SUCCESS;
        goto Cleanup;
    }

    const ACPI_METHOD_ARGUMENT UNALIGNED* devicePropertiesPkgPtr;
    status = AcpiParseDsdAsDeviceProperties(dsdBufferPtr, &devicePropertiesPkgPtr);
    if (!NT_SUCCESS(status)) {
        IMXPWM_LOG_WARNING(
            "AcpiParseDsdAsDeviceProperties() failed with error %!STATUS!",
            status);
        status = STATUS_SUCCESS;
        goto Cleanup;
    }

    CHAR schematicNameA[64];
    WCHAR schematicNameW[64];
    UINT32 length = 0;
    size_t wlen = 0;

    status = AcpiDevicePropertiesQueryStringValue(
        devicePropertiesPkgPtr,
        "Pwm-SchematicName",
        ARRAYSIZE(schematicNameA),
        &length,
        schematicNameA);
    if (!NT_SUCCESS(status)) {
        IMXPWM_LOG_WARNING(
            "AcpiDevicePropertiesQueryStringValue(Pwm-SchematicName) failed with error %!STATUS!",
            status);
        status = STATUS_SUCCESS;
        goto Cleanup;
    }

    status = RtlStringCbPrintfW(schematicNameW, ARRAYSIZE(schematicNameW) * sizeof(WCHAR), L"%S", schematicNameA);
    if (!NT_SUCCESS(status)) {
        IMXPWM_LOG_ERROR(
            "RtlStringCbPrintfW(Pwm-SchematicName) failed with error %!STATUS!",
            status);
        goto Cleanup;
    }
    RtlStringCbLengthW(schematicNameW, ARRAYSIZE(schematicNameW) * sizeof(WCHAR), &wlen);
    wlen += sizeof(WCHAR);

    NT_ASSERT(wlen == (length * sizeof(WCHAR)));

    status = IoSetDeviceInterfacePropertyData(
        &deviceContextPtr->DeviceInterfaceSymlinkNameWsz,
        &DEVPKEY_DeviceInterface_SchematicName,
        LOCALE_NEUTRAL,
        0, // Flags
        DEVPROP_TYPE_STRING,
        (ULONG)wlen,
        schematicNameW);
    if (!NT_SUCCESS(status)) {
        IMXPWM_LOG_ERROR(
            "IoSetDeviceInterfacePropertyData(DeviceInterface-SchematicName) failed with error %!STATUS!",
            status);
        goto Cleanup;
    }

Cleanup:

    if (dsdBufferPtr != nullptr) {
        ExFreePoolWithTag(dsdBufferPtr, ACPI_TAG_EVAL_OUTPUT_BUFFER);
    }

    return status;
}

_Use_decl_annotations_
NTSTATUS
ImxPwmEvtDeviceReleaseHardware (
    WDFDEVICE WdfDevice,
    WDFCMRESLIST /*ResourcesTranslated*/
    )
{
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    IMXPWM_DEVICE_CONTEXT* deviceContextPtr =
            ImxPwmGetDeviceContext(WdfDevice);

    if (deviceContextPtr->RegistersPtr != nullptr) {
        MmUnmapIoSpace(
            deviceContextPtr->RegistersPtr,
            sizeof(IMXPWM_REGISTERS));

        deviceContextPtr->RegistersPtr = nullptr;
    }

    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS
ImxPwmResetControllerDefaults (
    IMXPWM_DEVICE_CONTEXT* DeviceContextPtr
    )
{
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    IMXPWM_LOG_TRACE("()");

    NTSTATUS status =
        ImxPwmSetDesiredPeriod(
            DeviceContextPtr,
            DeviceContextPtr->DefaultDesiredPeriod);
    if (!NT_SUCCESS(status)) {
        IMXPWM_LOG_ERROR(
            "ImxPwmSetDesiredPeriod(...) failed. (status = %!STATUS!)",
            status);
        return status;
    }

    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS
ImxPwmResetPinDefaults (
    IMXPWM_DEVICE_CONTEXT* DeviceContextPtr
    )
{
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    IMXPWM_LOG_TRACE("()");

    IMXPWM_PIN_STATE* pinPtr = &DeviceContextPtr->Pin;
    NTSTATUS status;

    if (pinPtr->IsStarted) {
        status = ImxPwmStop(DeviceContextPtr);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "ImxPwmStop(...) failed. (status = %!STATUS!)",
                status);
            return status;
        }
    }

    DeviceContextPtr->Pin.ActiveDutyCycle = 0;

    status = ImxPwmSetPolarity(DeviceContextPtr, PWM_ACTIVE_HIGH);
    if (!NT_SUCCESS(status)) {
        IMXPWM_LOG_ERROR(
            "ImxPwmSetPolarity(...) failed. (status = %!STATUS!)",
            status);
        return status;
    }

    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS
ImxPwmEvtDeviceD0Entry (
    WDFDEVICE WdfDevice,
    WDF_POWER_DEVICE_STATE /* PreviousState */
    )
{
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    IMXPWM_DEVICE_CONTEXT* deviceContextPtr = ImxPwmGetDeviceContext(WdfDevice);
    NTSTATUS status = ImxPwmSoftReset(deviceContextPtr);
    if (!NT_SUCCESS(status)) {
        IMXPWM_LOG_ERROR(
            "D0Entry - ImxPwmSoftReset(...) failed. (status = %!STATUS!)",
            status);
        return status;
    }
    if (deviceContextPtr->RestorePwmState) {
       status = ImxPwmSetDesiredPeriod(deviceContextPtr, deviceContextPtr->RestoreDesiredPeriod);
       if (!NT_SUCCESS(status)) {
          IMXPWM_LOG_ERROR(
             "D0Entry - ImxPwmSetDesiredPeriod(...) failed. (status = %!STATUS!)",
             status);
          return status;
       }
       status = ImxPwmSetPolarity(deviceContextPtr, deviceContextPtr->RestorePolarity);
       if (!NT_SUCCESS(status)) {
          IMXPWM_LOG_ERROR(
             "D0Entry - ImxPwmSetPolarity(...) failed. (status = %!STATUS!)",
             status);
          return status;
       }
       status = ImxPwmStart(deviceContextPtr);
       if (!NT_SUCCESS(status)) {
          IMXPWM_LOG_ERROR(
             "D0Entry - ImxPwmStart(...) failed. (status = %!STATUS!)",
             status);
          return status;
       }
       status = ImxPwmSetActiveDutyCycle(deviceContextPtr, deviceContextPtr->RestoreActiveDutyCycle);
       if (!NT_SUCCESS(status)) {
          IMXPWM_LOG_ERROR(
             "D0Entry - ImxPwmSetActiveDutyCycle(...) failed. (status = %!STATUS!)",
             status);
          return status;
       }
    }
    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS
ImxPwmEvtDeviceD0Exit(
    WDFDEVICE WdfDevice,
    WDF_POWER_DEVICE_STATE /*TargetState*/
)
{
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    NTSTATUS status = STATUS_SUCCESS;

    IMXPWM_DEVICE_CONTEXT* deviceContextPtr = ImxPwmGetDeviceContext(WdfDevice);
    IMXPWM_PIN_STATE* pinPtr = &deviceContextPtr->Pin;

    if (pinPtr->IsStarted) {
       //Store current values to allow restore when power state will again go to running (d0)
       deviceContextPtr->RestorePwmState = TRUE;
       deviceContextPtr->RestoreDesiredPeriod = deviceContextPtr->DesiredPeriod;
       deviceContextPtr->RestoreActiveDutyCycle = deviceContextPtr->Pin.ActiveDutyCycle;
       deviceContextPtr->RestorePolarity = deviceContextPtr->Pin.Polarity;
        status = ImxPwmStop(deviceContextPtr);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "ImxPwmEvtDeviceD0Exit ImxPwmStop(...) failed. (status = %!STATUS!)",
                status);
        }
    } else {
       deviceContextPtr->RestorePwmState = FALSE;
    }

    return status;
}

_Use_decl_annotations_
NTSTATUS
ImxPwmCreateDeviceInterface (
    IMXPWM_DEVICE_CONTEXT* DeviceContextPtr
    )
{
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    NTSTATUS status =
        WdfDeviceCreateDeviceInterface(
                DeviceContextPtr->WdfDevice,
                &GUID_DEVINTERFACE_PWM_CONTROLLER,
                nullptr);
    if (!NT_SUCCESS(status)) {
        IMXPWM_LOG_ERROR(
            "WdfDeviceCreateDeviceInterface(...) failed. (status = %!STATUS!)",
            status);

        return status;
    }

    //
    // Retrieve device interface symbolic link string
    //
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = DeviceContextPtr->WdfDevice;
        status =
            WdfStringCreate(
                nullptr,
                &attributes,
                &DeviceContextPtr->DeviceInterfaceSymlinkName);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "WdfStringCreate(...) failed. (status = %!STATUS!)",
                status);

            return status;
        }

        status =
            WdfDeviceRetrieveDeviceInterfaceString(
                DeviceContextPtr->WdfDevice,
                &GUID_DEVINTERFACE_PWM_CONTROLLER,
                nullptr,
                DeviceContextPtr->DeviceInterfaceSymlinkName);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "WdfDeviceRetrieveDeviceInterfaceString(...) failed. "
                "(status = %!STATUS!, GUID_DEVINTERFACE_PWM_CONTROLLER = %!GUID!)",
                status,
                &GUID_DEVINTERFACE_PWM_CONTROLLER);

            return status;
        }

        WdfStringGetUnicodeString(
            DeviceContextPtr->DeviceInterfaceSymlinkName,
            &DeviceContextPtr->DeviceInterfaceSymlinkNameWsz);
    }

    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS
ImxPwmEvtDeviceAdd (
    WDFDRIVER /*WdfDriver*/,
    WDFDEVICE_INIT* DeviceInitPtr
    )
{
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    //
    // Set PNP and Power callbacks
    //
    {
        WDF_PNPPOWER_EVENT_CALLBACKS wdfPnpPowerEventCallbacks;
        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&wdfPnpPowerEventCallbacks);
        wdfPnpPowerEventCallbacks.EvtDevicePrepareHardware =
            ImxPwmEvtDevicePrepareHardware;

        wdfPnpPowerEventCallbacks.EvtDeviceReleaseHardware =
            ImxPwmEvtDeviceReleaseHardware;
        wdfPnpPowerEventCallbacks.EvtDeviceD0Entry =
            ImxPwmEvtDeviceD0Entry;
#ifdef PWM_POWER_MANAGEMENT
        wdfPnpPowerEventCallbacks.EvtDeviceD0Exit =
            ImxPwmEvtDeviceD0Exit;
#endif
        WdfDeviceInitSetPnpPowerEventCallbacks(
            DeviceInitPtr,
            &wdfPnpPowerEventCallbacks);

    }

    //
    // Configure file create/close callbacks
    //
    {
        WDF_FILEOBJECT_CONFIG wdfFileObjectConfig;
        WDF_FILEOBJECT_CONFIG_INIT(
            &wdfFileObjectConfig,
            ImxPwmEvtDeviceFileCreate,
            ImxPwmEvtFileClose,
            WDF_NO_EVENT_CALLBACK); // not interested in Cleanup

        WdfDeviceInitSetFileObjectConfig(
            DeviceInitPtr,
            &wdfFileObjectConfig,
            WDF_NO_OBJECT_ATTRIBUTES);
    }

    NTSTATUS status;

    //
    // Create and initialize the WDF device
    //
    WDFDEVICE wdfDevice;
    IMXPWM_DEVICE_CONTEXT* deviceContextPtr;
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(
            &attributes,
            IMXPWM_DEVICE_CONTEXT);

        status = WdfDeviceCreate(&DeviceInitPtr, &attributes, &wdfDevice);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "WdfDeviceCreate(...) failed. (status = %!STATUS!)",
                status);

            return status;
        }

        WDF_OBJECT_ATTRIBUTES wdfObjectAttributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(
            &wdfObjectAttributes,
            IMXPWM_DEVICE_CONTEXT);

        void* contextPtr;
        status =
            WdfObjectAllocateContext(
                wdfDevice,
                &wdfObjectAttributes,
                &contextPtr);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "WdfObjectAllocateContext(...) failed. (status = %!STATUS!)",
                status);

            return status;
        }

        deviceContextPtr = static_cast<IMXPWM_DEVICE_CONTEXT*>(contextPtr);
        deviceContextPtr->WdfDevice = wdfDevice;
    }

    //
    // Create an interrupt object with an associated context.
    //
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(
            &attributes,
            IMXPWM_INTERRUPT_CONTEXT);

        WDF_INTERRUPT_CONFIG interruptConfig;
        WDF_INTERRUPT_CONFIG_INIT(
            &interruptConfig,
            ImxPwmEvtInterruptIsr,
            ImxPwmEvtInterruptDpc);

        WDFINTERRUPT wdfInterrupt;
        status =
            WdfInterruptCreate(
                wdfDevice,
                &interruptConfig,
                &attributes,
                &wdfInterrupt);

        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "Failed to create interrupt object. (status = %!STATUS!)",
                status);

            return status;
        }

        deviceContextPtr->WdfInterrupt = wdfInterrupt;
    }

    //
    // Create controller and pin locks
    //
    {
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = wdfDevice;
        status =
            WdfWaitLockCreate(&attributes, &deviceContextPtr->ControllerLock);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "WdfWaitLockCreate(...) failed. (status = %!STATUS!)",
                status);

            return status;
        }

        status = WdfWaitLockCreate(&attributes, &deviceContextPtr->Pin.Lock);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "WdfWaitLockCreate(...) failed. (status = %!STATUS!)",
                status);

            return status;
        }
    }

    //
    // Set controller info and defaults
    //
    {
        UINT32 bootOn = 0;
        UINT32 bootPolarity = 0;
        ULARGE_INTEGER bootDesiredPeriod = { 0 };
        ULARGE_INTEGER bootActiveDutyCycle = { 0 };

        //Use hardcoded values first. Can be overriden by ACPI values later
        deviceContextPtr->ClockSource = IMXPWM_PWMCR_CLKSRC(IMXPWM_DEFAULT_CLKSRC);
        switch (ImxGetSocType()) {
        case IMX_SOC_MX8M:
            deviceContextPtr->ClockSourceFrequency = IMXPWM_25MHZ_CLKSRC_FREQ;
            break;
        case IMX_SOC_MX7:
            deviceContextPtr->ClockSourceFrequency = IMXPWM_24MHZ_CLKSRC_FREQ;
            break;
        default:
            deviceContextPtr->ClockSourceFrequency = IMXPWM_66MHZ_CLKSRC_FREQ;
            break;
        }
        //set default values, if ACPI values are not defined
        deviceContextPtr->RovEventCounterCompare = IMXPWM_ROV_EVT_COUNTER_COMPARE;
        auto& info = deviceContextPtr->ControllerInfo;
        info.Size = sizeof(PWM_CONTROLLER_INFO);
        info.PinCount = IMXPWM_PIN_COUNT;

        // Pull optional Pwm-SchematicName value from ACPI _DSD table and publish as SchematicName property
        DEVICE_OBJECT* pdoPtr = WdfDeviceWdmGetPhysicalDevice(wdfDevice);
        NT_ASSERT(pdoPtr != nullptr);

        ACPI_EVAL_OUTPUT_BUFFER* dsdBufferPtr = nullptr;

        bool acpiAvailable = false;
        const ACPI_METHOD_ARGUMENT UNALIGNED* devicePropertiesPkgPtr = nullptr;

        status = AcpiQueryDsd(pdoPtr, &dsdBufferPtr);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_WARNING(
                "DevAdd - AcpiQueryDsd() failed with error %!STATUS!",
                status);
        }
        else {
            status = AcpiParseDsdAsDeviceProperties(dsdBufferPtr, &devicePropertiesPkgPtr);
            if (!NT_SUCCESS(status)) {
                IMXPWM_LOG_WARNING(
                    "DevAdd - AcpiParseDsdAsDeviceProperties() failed with error %!STATUS!",
                    status);
            }
            else {
                acpiAvailable = true;
            }
        }
        if (acpiAvailable) {
            UINT32 freq = 0;
            status = AcpiDevicePropertiesQueryIntegerValue(
                devicePropertiesPkgPtr,
                "ClockFrequency_Hz",
                &freq);
            if (!NT_SUCCESS(status)) {
                IMXPWM_LOG_WARNING(
                    "DevAdd - AcpiDevicePropertiesQueryIntegerValue(ClockFrequency_Hz) failed with error %!STATUS!",
                    status);
                
            }
            else {
                deviceContextPtr->ClockSourceFrequency = freq;
            }
            UINT32 pinCnt = 0;
            status = AcpiDevicePropertiesQueryIntegerValue(
                devicePropertiesPkgPtr,
                "PinCount",
                &pinCnt);
            if (!NT_SUCCESS(status)) {
                IMXPWM_LOG_WARNING(
                    "DevAdd - AcpiDevicePropertiesQueryIntegerValue(PinCount) failed with error %!STATUS!",
                    status);
            }
            else {
                info.PinCount = pinCnt;
            }
        }
        ///calculate period, from ACPIvalues or from hardcoded defaults
        status =
            ImxPwmCalculatePeriod(
                deviceContextPtr,
                IMXPWM_PRESCALER_MIN,
                &info.MinimumPeriod);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "Failed to compute minimum period. (status = %!STATUS!)",
                status);
            if (dsdBufferPtr != nullptr) {
                ExFreePoolWithTag(dsdBufferPtr, ACPI_TAG_EVAL_OUTPUT_BUFFER);
            }
            return status;
        }

        status =
            ImxPwmCalculatePeriod(
                deviceContextPtr,
                IMXPWM_PRESCALER_MAX,
                &info.MaximumPeriod);
        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "Failed to compute maximum period. (status = %!STATUS!)",
                status);
            if (dsdBufferPtr != nullptr) {
                ExFreePoolWithTag(dsdBufferPtr, ACPI_TAG_EVAL_OUTPUT_BUFFER);
            }
            return status;
        }

        deviceContextPtr->DefaultDesiredPeriod = info.MinimumPeriod;

        NT_ASSERT(
            (info.MinimumPeriod > 0) &&
            (info.MinimumPeriod <= info.MaximumPeriod) &&
            (deviceContextPtr->DefaultDesiredPeriod > 0));
        IMXPWM_LOG_INFORMATION(
            "Controller Info: PinCount = %lu, MinimumPeriod = %llups(%lluHz), "
            "MaximumPeriod = %llups(%lluHz), DefaultPeriod = %llups(%lluHz)",
            info.PinCount,
            info.MinimumPeriod,
            ImxPwmPeriodToFrequency(info.MinimumPeriod),
            info.MaximumPeriod,
            ImxPwmPeriodToFrequency(info.MaximumPeriod),
            deviceContextPtr->DefaultDesiredPeriod,
            ImxPwmPeriodToFrequency(deviceContextPtr->DefaultDesiredPeriod));

        //boot configuration - check if enable the PWM device after boot is requested
        if (acpiAvailable) {
            status = AcpiDevicePropertiesQueryIntegerValue(
                devicePropertiesPkgPtr,
                "BootOn",
                &bootOn);
            if (!NT_SUCCESS(status)) {
                IMXPWM_LOG_WARNING(
                    "DevAdd - AcpiDevicePropertiesQueryIntegerValue(BootOn) failed with error %!STATUS!",
                    status);
                status = STATUS_SUCCESS;
                bootOn = false;
                goto bootCfgEnd;
            }
            if (bootOn) {
                status = AcpiDevicePropertiesQueryIntegerValue(
                    devicePropertiesPkgPtr,
                    "BootDesiredPeriodLowPart",
                    &bootDesiredPeriod.LowPart);
                if (!NT_SUCCESS(status)) {
                    IMXPWM_LOG_WARNING(
                        "DevAdd - AcpiDevicePropertiesQueryIntegerValue(BootDesiredPeriodLowPart) failed with error %!STATUS!",
                        status);
                    bootOn = false;
                    goto bootCfgEnd;
                }
                status = AcpiDevicePropertiesQueryIntegerValue(
                    devicePropertiesPkgPtr,
                    "BootDesiredPeriodHighPart",
                    &bootDesiredPeriod.HighPart);
                if (!NT_SUCCESS(status)) {
                    IMXPWM_LOG_WARNING(
                        "DevAdd - AcpiDevicePropertiesQueryIntegerValue(BootDesiredPeriodHighPart) failed with error %!STATUS!",
                        status);
                    bootOn = false;
                    goto bootCfgEnd;
                }
                if ((bootDesiredPeriod.QuadPart < info.MinimumPeriod) || (bootDesiredPeriod.QuadPart > info.MaximumPeriod)) {
                    IMXPWM_LOG_ERROR("DevAdd - BootDesiredPeriod is out of allowed range!!!");
                    bootOn = false;
                    goto bootCfgEnd;

                }
                status = AcpiDevicePropertiesQueryIntegerValue(
                    devicePropertiesPkgPtr,
                    "BootActiveDutyCycleLowPart",
                    &bootActiveDutyCycle.LowPart);
                if (!NT_SUCCESS(status)) {
                    IMXPWM_LOG_WARNING(
                        "DevAdd - AcpiDevicePropertiesQueryIntegerValue(BootActiveDutyCycleLowPart) failed with error %!STATUS!",
                        status);
                    bootOn = false;
                    goto bootCfgEnd;
                }
                status = AcpiDevicePropertiesQueryIntegerValue(
                    devicePropertiesPkgPtr,
                    "BootActiveDutyCycleHighPart",
                    &bootActiveDutyCycle.HighPart);
                if (!NT_SUCCESS(status)) {
                    IMXPWM_LOG_WARNING(
                        "DevAdd - AcpiDevicePropertiesQueryIntegerValue(BootActiveDutyCycleHighPart) failed with error %!STATUS!",
                        status);
                    bootOn = false;
                    goto bootCfgEnd;
                }

                status = AcpiDevicePropertiesQueryIntegerValue(
                    devicePropertiesPkgPtr,
                    "BootPolarity",
                    &bootPolarity);
                if (!NT_SUCCESS(status)) {
                    IMXPWM_LOG_WARNING(
                        "DevAdd - AcpiDevicePropertiesQueryIntegerValue(BootPolarity) failed with error %!STATUS!",
                        status);
                    bootPolarity = 1;
                    goto bootCfgEnd;
                }
            }
        }
 bootCfgEnd:
        if (bootOn) {
            deviceContextPtr->RestorePwmState = TRUE;
            deviceContextPtr->RestoreActiveDutyCycle = bootActiveDutyCycle.QuadPart;
            deviceContextPtr->RestoreDesiredPeriod = bootDesiredPeriod.QuadPart;
            deviceContextPtr->Pin.Polarity = (bootPolarity > 0) ? PWM_POLARITY::PWM_ACTIVE_HIGH : PWM_POLARITY::PWM_ACTIVE_LOW;
        }
        if (dsdBufferPtr != nullptr) {
            ExFreePoolWithTag(dsdBufferPtr, ACPI_TAG_EVAL_OUTPUT_BUFFER);
        }
    }

    //
    // Set PNP capabilities
    //
    {
        WDF_DEVICE_PNP_CAPABILITIES pnpCaps;
        WDF_DEVICE_PNP_CAPABILITIES_INIT(&pnpCaps);

        pnpCaps.Removable = WdfFalse;
        pnpCaps.SurpriseRemovalOK = WdfFalse;
        WdfDeviceSetPnpCapabilities(wdfDevice, &pnpCaps);
    }

    //
    // Make the device disable-able
    //
    {
        WDF_DEVICE_STATE wdfDeviceState;
        WDF_DEVICE_STATE_INIT(&wdfDeviceState);

        wdfDeviceState.NotDisableable = WdfFalse;
        WdfDeviceSetDeviceState(wdfDevice, &wdfDeviceState);
    }

    //
    // Create default sequential dispatch queue
    //
    {
        WDF_IO_QUEUE_CONFIG wdfQueueConfig;
        WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(
            &wdfQueueConfig,
            WdfIoQueueDispatchSequential);
        wdfQueueConfig.EvtIoDeviceControl = ImxPwmEvtIoDeviceControl;

        WDF_OBJECT_ATTRIBUTES wdfQueueAttributes;
        WDF_OBJECT_ATTRIBUTES_INIT(&wdfQueueAttributes);
        wdfQueueAttributes.ExecutionLevel = WdfExecutionLevelPassive;

        WDFQUEUE wdfQueue;
        status = WdfIoQueueCreate(
                wdfDevice,
                &wdfQueueConfig,
                &wdfQueueAttributes,
                &wdfQueue);

        if (!NT_SUCCESS(status)) {
            IMXPWM_LOG_ERROR(
                "WdfIoQueueCreate(..) failed. (status=%!STATUS!)",
                status);

            return status;
        }
    }

    //
    // Publish controller device interface
    //
    status = ImxPwmCreateDeviceInterface(deviceContextPtr);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    IMXPWM_LOG_INFORMATION(
        "Published device interface %wZ",
        &deviceContextPtr->DeviceInterfaceSymlinkNameWsz);

#ifdef PWM_POWER_MANAGEMENT
    status = PowerManagementSetup(wdfDevice);
    if (!NT_SUCCESS(status)) {
        IMXPWM_LOG_ERROR("SingleCompEvtDeviceAdd(..) failed. (status=%!STATUS!)", status);
        return status;
    }
#endif

    return STATUS_SUCCESS;
}

IMXPWM_PAGED_SEGMENT_END; //===================================================