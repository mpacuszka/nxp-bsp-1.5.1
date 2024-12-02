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

#include "precomp.h"
#include "imxpwm_tpm.h"
#include "imxpwm_tpm_dbg.h"

IMXPWM_PAGED_SEGMENT_BEGIN;

_IRQL_requires_same_
void ImxTpmResetControllerDefaults(_In_ IMXPWM_DEVICE_CONTEXT* pDevContext) {
    DBG_DEV_METHOD_BEG_WITH_PARAMS("Setting default desired period (min. period): %llu [ps]", pDevContext->ControllerInfo.MinimumPeriod);
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    ImxTpmSetDesiredPeriod(pDevContext, pDevContext->ControllerInfo.MinimumPeriod);
    DBG_DEV_METHOD_END();
    return;
}

_IRQL_requires_same_
void ImxTpmResetPinDefaults(_In_ IMXPWM_DEVICE_CONTEXT* pDevContext, _In_ ULONG PinNumber) {
    DBG_DEV_METHOD_BEG_WITH_PARAMS("Disabling pin %d: ActiveDuty = 0%%, Polarity = HIGH", PinNumber);
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    if (pDevContext->Pin[PinNumber].IsStarted) {
        pDevContext->Pin[PinNumber].IsStarted = FALSE;
        DisableChn(pDevContext->pRegs, PinNumber);
    }
    pDevContext->Pin[PinNumber].ActiveDutyCycle = 0;
    SetChnVal(pDevContext->pRegs, PinNumber, 0);
    SetHighPolarity(pDevContext->pRegs, PinNumber);
    DBG_DEV_METHOD_END();
    return;
}

_IRQL_requires_same_
void ImxTpmSoftReset(_In_ IMXPWM_DEVICE_CONTEXT* pDevContext) {
    TPM_t* pRegs = pDevContext->pRegs;

    DBG_DEV_METHOD_BEG();
    PAGED_CODE();
    pRegs->GLOBAL.B.RST = 1;
    pRegs->GLOBAL.B.RST = 0;
    pRegs->MOD.R = 1;           /* Set minimal period  */
    pRegs->C0V.R = 0;           /* Set duty cycle = 0% */
    pRegs->C1V.R = 0;           /* Set duty cycle = 0% */
    pRegs->C2V.R = 0;           /* Set duty cycle = 0% */
    pRegs->C3V.R = 0;           /* Set duty cycle = 0% */
    for (int i = 0; i < MAX_PWM_PINS; i++) {
        pDevContext->Pin[i].ActiveDutyCycle = 0;
        pDevContext->Pin[i].Polarity        = PWM_ACTIVE_HIGH;
        pDevContext->Pin[i].IsStarted       = FALSE;
    }
    DBG_DEV_DUMP_REGS(pDevContext);
    pDevContext->ActualPeriod = CalculatePeriod_in_picosecs(pDevContext->ClockSourceFrequency, pDevContext->pRegs->SC.B.PS, pDevContext->pRegs->MOD.R);
    DBG_DEV_METHOD_END_WITH_PARAMS("Actual period: %llu", pDevContext->ActualPeriod);
    return;
}

_Use_decl_annotations_
VOID ImxTpmEvtDeviceFileCreate(WDFDEVICE WdfDevice, WDFREQUEST WdfRequest, WDFFILEOBJECT WdfFileObject) {
    UNICODE_STRING*        pFileName   = WdfFileObjectGetFileName(WdfFileObject);
    IMXPWM_DEVICE_CONTEXT* pDevContext = ImxPwmGetDeviceContext(WdfDevice);
    NTSTATUS               ntStatus;
    ULONG                  pinNumber = ULONG_MAX;
    BOOLEAN                isPinInterface;

    DBG_DEV_METHOD_BEG();
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    do {
        /* Parseand validate the filename associated with the file object. */
        if (pFileName == NULL) {
            DBG_PRINT_ERROR_WITH_STATUS(STATUS_INVALID_DEVICE_REQUEST, "pFileName == NULL.");
            WdfRequestComplete(WdfRequest, STATUS_INVALID_DEVICE_REQUEST);
            break;
        } else if (pFileName->Length > 0) {
            /* A non - empty filename means to open a pin under the controller namespace. */
            if (!NT_SUCCESS(ntStatus = PwmParsePinPath(pFileName, &pinNumber))) {
                DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "PwmParsePinPath(%ws)", pFileName->Buffer);
                WdfRequestComplete(WdfRequest, ntStatus);
                break;
            }
            if (pinNumber >= pDevContext->ControllerInfo.PinCount) {
                DBG_DEV_PRINT_INFO("Requested pin number out of bounds. (pinNumber = %d)", pinNumber);
                WdfRequestComplete(WdfRequest, STATUS_NO_SUCH_FILE);
                break;
            }
            isPinInterface = TRUE;
        } else {
            /* An empty filename means that the create is against the root controller. */
            isPinInterface = FALSE;
            pinNumber = (ULONG)-1;
        }
        WDF_REQUEST_PARAMETERS wdfRequestParameters;
        WDF_REQUEST_PARAMETERS_INIT(&wdfRequestParameters);
        WdfRequestGetParameters(WdfRequest, &wdfRequestParameters);        
        NT_ASSERTMSG("Expected create request", wdfRequestParameters.Type == WdfRequestTypeCreate);
        /* ShareAccess will not be honored as it has no meaning currently in the PWM DDI. */
        if (wdfRequestParameters.Parameters.Create.ShareAccess != 0) {
            DBG_DEV_PRINT_INFO("Requested share access is not supported and request ShareAccess parameter should be zero. Access denied. (shareAccess = %lu)", wdfRequestParameters.Parameters.Create.ShareAccess);
            WdfRequestComplete(WdfRequest, STATUS_SHARING_VIOLATION);
            break;
        }
        /* Verify request desired access.*/
        const BOOLEAN hasWriteAccess = ((wdfRequestParameters.Parameters.Create.SecurityContext->DesiredAccess & FILE_WRITE_DATA) != 0);
        if (isPinInterface) {
            IMXPWM_PIN_STATE* pinPtr = &pDevContext->Pin[pinNumber];
            WdfWaitLockAcquire(pinPtr->Lock, NULL);
            if (hasWriteAccess) {
                if (pinPtr->IsOpenForWrite) {
                    WdfWaitLockRelease(pinPtr->Lock);
                    DBG_PRINT_ERROR_WITH_STATUS(STATUS_SHARING_VIOLATION, "Pin access denied.");
                    WdfRequestComplete(WdfRequest, STATUS_SHARING_VIOLATION);
                    break;
                }
                pinPtr->IsOpenForWrite = TRUE;
            }
            DBG_DEV_PRINT_INFO("Pin %d opened. (IsOpenForWrite = %d)", pinNumber, pinPtr->IsOpenForWrite);
            WdfWaitLockRelease(pinPtr->Lock);
        } else {
            WdfWaitLockAcquire(pDevContext->ControllerLock, NULL);
            if (hasWriteAccess) {
                if (pDevContext->IsControllerOpenForWrite) {
                    WdfWaitLockRelease(pDevContext->ControllerLock);
                    DBG_PRINT_ERROR_WITH_STATUS(STATUS_SHARING_VIOLATION, "Controller access denied.");
                    WdfRequestComplete(WdfRequest, STATUS_SHARING_VIOLATION);
                    break;
                }
                pDevContext->IsControllerOpenForWrite = TRUE;
            }
            DBG_DEV_PRINT_INFO("Controller Opened. (IsControllerOpenForWrite = %d)", pDevContext->IsControllerOpenForWrite);
            WdfWaitLockRelease(pDevContext->ControllerLock);
        }
        /* Allocateand fill a file object context. */
        IMXPWM_FILE_OBJECT_CONTEXT* pFileObjectContext;
        {
            WDF_OBJECT_ATTRIBUTES wdfObjectAttributes;
            WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&wdfObjectAttributes, IMXPWM_FILE_OBJECT_CONTEXT);

            if (!NT_SUCCESS(ntStatus = WdfObjectAllocateContext(WdfFileObject, &wdfObjectAttributes, (PVOID)&pFileObjectContext))) {
                DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfObjectAllocateContext(...) failed.");
                WdfRequestComplete(WdfRequest, ntStatus);
                break;
            }
            NT_ASSERT(pFileObjectContext != NULL);
            pFileObjectContext->IsPinInterface = isPinInterface;
            pFileObjectContext->IsOpenForWrite = hasWriteAccess;
            pFileObjectContext->IsOpenForWrite = hasWriteAccess;
            pFileObjectContext->PinNumber = pinNumber;
        }
        WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
    } while (0);
    DBG_DEV_METHOD_END();
    return;
}

_Use_decl_annotations_
VOID ImxTpmEvtFileClose(WDFFILEOBJECT WdfFileObject) {
    WDFDEVICE                   wdfDevice          = WdfFileObjectGetDevice(WdfFileObject);
    IMXPWM_DEVICE_CONTEXT*      pDevContext        = ImxPwmGetDeviceContext(wdfDevice);
    IMXPWM_FILE_OBJECT_CONTEXT* pFileObjectContext = ImxPwmGetFileObjectContext(WdfFileObject);

    DBG_DEV_METHOD_BEG();
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    if (pFileObjectContext->IsPinInterface) {
        WdfWaitLockAcquire(pDevContext->Pin[pFileObjectContext->PinNumber].Lock, NULL);
        if (pFileObjectContext->IsOpenForWrite) {
            ImxTpmResetPinDefaults(pDevContext, pFileObjectContext->PinNumber);
            NT_ASSERT(pDevContext->Pin[pFileObjectContext->PinNumber].IsOpenForWrite);
            pDevContext->Pin[pFileObjectContext->PinNumber].IsOpenForWrite = FALSE;
        }
        DBG_DEV_PRINT_INFO("Pin %d closed. (IsOpenForWrite = %d)", pFileObjectContext->PinNumber, (pDevContext->Pin[pFileObjectContext->PinNumber].IsOpenForWrite ? 1 : 0));
        WdfWaitLockRelease(pDevContext->Pin[pFileObjectContext->PinNumber].Lock);
    } else {
        WdfWaitLockAcquire(pDevContext->ControllerLock, NULL);
        if (pFileObjectContext->IsOpenForWrite) {
            ImxTpmResetControllerDefaults(pDevContext);
            NT_ASSERT(pDevContext->IsControllerOpenForWrite);
            pDevContext->IsControllerOpenForWrite = FALSE;
        }
        DBG_DEV_PRINT_INFO("Controller Closed. (IsControllerOpenForWrite = %d)", (pDevContext->IsControllerOpenForWrite ? 1 : 0));
        WdfWaitLockRelease(pDevContext->ControllerLock);
    }
    DBG_DEV_METHOD_END();
    return;
}

_Use_decl_annotations_
NTSTATUS ImxTpmEvtDeviceD0Entry(WDFDEVICE WdfDevice, WDF_POWER_DEVICE_STATE PreviousState) {
    IMXPWM_DEVICE_CONTEXT* pDevContext = ImxPwmGetDeviceContext(WdfDevice);

    DBG_DEV_METHOD_BEG();
    UNREFERENCED_PARAMETER(PreviousState);
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    ImxTpmSoftReset(ImxPwmGetDeviceContext(WdfDevice));
    EnableTpmCounter(pDevContext->pRegs);
    DBG_DEV_METHOD_END_WITH_STATUS(STATUS_SUCCESS);
    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS ImxTpmEvtDevicePrepareHardware (WDFDEVICE WdfDevice, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated) {
    NTSTATUS                         ntStatus     = STATUS_SUCCESS;
    IMXPWM_DEVICE_CONTEXT*           pDevContext  = ImxPwmGetDeviceContext(WdfDevice);
    ULONG                            uIntResCount = 0;
    ULONG                            uMemResCount = 0;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR  pResMemDscr  = NULL;
    ULONG                            uRresourceCount;
    PWM_CONTROLLER_INFO*             pInfo;
    char*                            pString;
    ULONG                            uLength;
    WCHAR                            wSchematicName[MAX_PWM_SCHEMATIC_NAME_LENGHT];

    DBG_DEV_METHOD_BEG();
    UNREFERENCED_PARAMETER(ResourcesRaw);
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    do {
        /* Look for single memoryand interrupt resource. */
        uRresourceCount = WdfCmResourceListGetCount(ResourcesTranslated);
        for (ULONG i = 0; i < uRresourceCount; ++i) {
            PCM_PARTIAL_RESOURCE_DESCRIPTOR pTmpResDscr = WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
            switch (pTmpResDscr->Type) {
            case CmResourceTypeMemory:
                pResMemDscr = pTmpResDscr;
                uMemResCount++;
                break;
            case CmResourceTypeInterrupt:
                uIntResCount++;
                break;
            }
        }
        if ((uMemResCount != 1) || (uIntResCount != 1)) {
            DBG_PRINT_ERROR("Found %d memory resources and %d interrupt resources but only one inetrrupt and one memory resource is expected.", uMemResCount, uIntResCount);
            ntStatus = STATUS_DEVICE_CONFIGURATION_ERROR;
            break;
        }
        pDevContext->RegsSize = pResMemDscr->u.Memory.Length;
        if ((pDevContext->pRegs = (TPM_t*)(MmMapIoSpaceEx(pResMemDscr->u.Memory.Start, pResMemDscr->u.Memory.Length, PAGE_READWRITE | PAGE_NOCACHE))) == NULL) {
            DBG_PRINT_ERROR("MmMapIoSpaceEx() failed.");
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
            break;
        }
        DBG_DEV_PRINT_INFO("TPM_RegPhyAddr: 0x%016llX, RegsSize: 0x%08X, VirtAddr: 0x%p", pResMemDscr->u.Memory.Start.QuadPart, pResMemDscr->u.Memory.Length, pDevContext->pRegs);

        pDevContext->pACPI_UtilsContext.Pdo = WdfDeviceWdmGetPhysicalDevice(WdfDevice);
        pDevContext->pACPI_UtilsContext.MemPoolTag = TPM_MEM_TAG_ACPI;
        if (!NT_SUCCESS(ntStatus = Acpi_Init(&pDevContext->pACPI_UtilsContext))) {
            DBG_DEV_PRINT_INFO("Acpi_Init() failed");
            break;
        }
        Acpi_GetIntegerPropertyValue(&pDevContext->pACPI_UtilsContext, "ClockFrequency_Hz", &pDevContext->ClockSourceFrequency);
        if (pDevContext->ClockSourceFrequency == 0) {
            ntStatus = STATUS_DEVICE_CONFIGURATION_ERROR;
            DBG_PRINT_ERROR("ClockFrequency_Hz property not found in ACPI.");
            break;
        }
        pInfo = &pDevContext->ControllerInfo;
        pInfo->Size = sizeof(PWM_CONTROLLER_INFO);
        Acpi_GetIntegerPropertyValue(&pDevContext->pACPI_UtilsContext, "PinCount", &pInfo->PinCount);
        if (pInfo->PinCount == 0) {
            ntStatus = STATUS_DEVICE_CONFIGURATION_ERROR;
            DBG_PRINT_ERROR("PinCount not found in ACPI.");
            break;
        }
        Acpi_GetStringPropertyAddress(&pDevContext->pACPI_UtilsContext, "Pwm-SchematicName", &pString, &uLength);
        if (pString == NULL) {
            DBG_DEV_PRINT_INFO("Pwm-SchematicName not found in ACPI.");
        } if (uLength > MAX_PWM_SCHEMATIC_NAME_LENGHT) {
            DBG_PRINT_ERROR("Pwm-SchematicName size (%d) is greather the driver supported size value %d .", uLength, MAX_PWM_SCHEMATIC_NAME_LENGHT);
            break;
        } else {
            if (!NT_SUCCESS(ntStatus = RtlStringCbPrintfW(wSchematicName, ARRAYSIZE(wSchematicName) * sizeof(WCHAR), L"%S", pString))) {
                DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "RtlStringCbPrintfW(Pwm-SchematicName) failed.");
                break;
            }
            if (!NT_SUCCESS(ntStatus = IoSetDeviceInterfacePropertyData(&pDevContext->DeviceInterfaceSymlinkNameWsz, &DEVPKEY_DeviceInterface_SchematicName, LOCALE_NEUTRAL, 0, /* Flags */ DEVPROP_TYPE_STRING, 2 * uLength, wSchematicName))) {
                DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "IoSetDeviceInterfacePropertyData(DeviceInterface-SchematicName) failed");
                break;
            }
        }
        pDevContext->ActualPeriod = CalculatePeriod_in_picosecs(pDevContext->ClockSourceFrequency, pDevContext->pRegs->SC.B.PS, pDevContext->pRegs->MOD.R);
        pInfo->MinimumPeriod      = CalculatePeriod_in_picosecs(pDevContext->ClockSourceFrequency, TPM_PRESCALER_MIN, TPM_MODULO_MIN);
        pInfo->MaximumPeriod      = CalculatePeriod_in_picosecs(pDevContext->ClockSourceFrequency, TPM_PRESCALER_MAX, TPM_MODULO_MAX);
        NT_ASSERT((pInfo->MinimumPeriod > 0) && (pInfo->MinimumPeriod <= pInfo->MaximumPeriod));
        DBG_DEV_PRINT_INFO(
            "Controller Info: Pwm-SchematicName: \"%s\", InputClock: %d[Mhz], PinCount: %lu, MinPeriod: %llups(%lluHz), MaxPeriod: %llu[ps](%llu[Hz]), DefaultPeriod: %llu[ps](%llu[Hz])",
            (pString == NULL)? "": pString, pDevContext->ClockSourceFrequency/1000000, pInfo->PinCount,
            pInfo->MinimumPeriod, Period_in_ps_to_frequency_in_Hz(pInfo->MinimumPeriod),
            pInfo->MaximumPeriod, Period_in_ps_to_frequency_in_Hz(pInfo->MaximumPeriod),
            pInfo->MinimumPeriod, Period_in_ps_to_frequency_in_Hz(pInfo->MinimumPeriod));
    } while (0);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

_Use_decl_annotations_
NTSTATUS ImxTpmEvtDeviceReleaseHardware (WDFDEVICE WdfDevice, WDFCMRESLIST ResourcesTranslated) {
    IMXPWM_DEVICE_CONTEXT* pDevContext = ImxPwmGetDeviceContext(WdfDevice);

    DBG_DEV_METHOD_BEG();
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    if (pDevContext->pRegs != NULL) {
        DBG_DEV_PRINT_INFO("MmUnmapIoSpace: 0x%016p, TPMRegsSize: 0x%08X", (PVOID)pDevContext->pRegs, pDevContext->RegsSize);
        MmUnmapIoSpace((PVOID)pDevContext->pRegs, pDevContext->RegsSize);
        pDevContext->pRegs = NULL;
    }
    Acpi_Deinit(&pDevContext->pACPI_UtilsContext);
    DBG_DEV_METHOD_END_WITH_STATUS(STATUS_SUCCESS);
    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS ImxTpmEvtDeviceAdd(WDFDRIVER WdfDriver, PWDFDEVICE_INIT DeviceInitPtr) {
    NTSTATUS                     ntStatus    = STATUS_UNSUCCESSFUL;
    IMXPWM_DEVICE_CONTEXT*       pDevContext = NULL;
    WDFDEVICE                    wdfDevice;
    WDF_PNPPOWER_EVENT_CALLBACKS wdfPnpPowerEventCallbacks;
    WDF_OBJECT_ATTRIBUTES        wdfObjectAttributes;
    WDF_FILEOBJECT_CONFIG        wdfFileObjectConfig;
    WDF_IO_QUEUE_CONFIG          wdfQueueConfig;
    WDFQUEUE                     wdfQueue;
    WDF_DEVICE_PNP_CAPABILITIES  pnpCaps;

    DBG_DEV_METHOD_BEG();
    UNREFERENCED_PARAMETER(WdfDriver);
    PAGED_CODE();
    IMXPWM_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    /* Set PNPand Power callbacks */
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&wdfPnpPowerEventCallbacks);
    wdfPnpPowerEventCallbacks.EvtDevicePrepareHardware = ImxTpmEvtDevicePrepareHardware;
    wdfPnpPowerEventCallbacks.EvtDeviceReleaseHardware = ImxTpmEvtDeviceReleaseHardware;
    wdfPnpPowerEventCallbacks.EvtDeviceD0Entry         = ImxTpmEvtDeviceD0Entry;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInitPtr, &wdfPnpPowerEventCallbacks);
    /* Configure file create / close callbacks */
    WDF_FILEOBJECT_CONFIG_INIT(&wdfFileObjectConfig, ImxTpmEvtDeviceFileCreate, ImxTpmEvtFileClose, WDF_NO_EVENT_CALLBACK); // not interested in Cleanup
    WdfDeviceInitSetFileObjectConfig(DeviceInitPtr, &wdfFileObjectConfig, WDF_NO_OBJECT_ATTRIBUTES);
    /* Createand initialize the WDF device */
    do {
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&wdfObjectAttributes, IMXPWM_DEVICE_CONTEXT);
        if (!NT_SUCCESS(ntStatus = WdfDeviceCreate(&DeviceInitPtr, &wdfObjectAttributes, &wdfDevice))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfDeviceCreate(...) failed.");
            break;
        }
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&wdfObjectAttributes, IMXPWM_DEVICE_CONTEXT);
        if (!NT_SUCCESS(ntStatus = WdfObjectAllocateContext(wdfDevice, &wdfObjectAttributes, (PVOID *)&pDevContext))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfObjectAllocateContext(...) failed.");
            break;
        }
        pDevContext->WdfDevice = wdfDevice;
        // Create controller and pin locks
        WDF_OBJECT_ATTRIBUTES_INIT(&wdfObjectAttributes);
        wdfObjectAttributes.ParentObject = wdfDevice;
        if (!NT_SUCCESS(ntStatus = WdfWaitLockCreate(&wdfObjectAttributes, &pDevContext->ControllerLock))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfWaitLockCreate(...) failed.");
            break;
        }
        for (int i = 0; i < MAX_PWM_PINS; i++) {
            if (!NT_SUCCESS(ntStatus = WdfWaitLockCreate(&wdfObjectAttributes, &pDevContext->Pin[i].Lock))) {
                DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfWaitLockCreate(...) failed.");
                break;
            }
        }
        if (!NT_SUCCESS(ntStatus)) {
            break;
        }
        /* Set PNP capabilities */
        WDF_DEVICE_PNP_CAPABILITIES_INIT(&pnpCaps);
        pnpCaps.Removable = WdfFalse;
        pnpCaps.SurpriseRemovalOK = WdfFalse;
        WdfDeviceSetPnpCapabilities(wdfDevice, &pnpCaps);
        /* Make the device disable-able */
        WDF_DEVICE_STATE wdfDeviceState;
        WDF_DEVICE_STATE_INIT(&wdfDeviceState);
        wdfDeviceState.NotDisableable = WdfFalse;
        WdfDeviceSetDeviceState(wdfDevice, &wdfDeviceState);
        /* Create default sequential dispatch queue */
        WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&wdfQueueConfig, WdfIoQueueDispatchSequential);
        wdfQueueConfig.EvtIoDeviceControl = ImxTpmEvtIoDeviceControl;
        WDF_OBJECT_ATTRIBUTES_INIT(&wdfObjectAttributes);
        wdfObjectAttributes.ExecutionLevel = WdfExecutionLevelPassive;
        if (!NT_SUCCESS(ntStatus = WdfIoQueueCreate(wdfDevice, &wdfQueueConfig, &wdfObjectAttributes, &wdfQueue))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfIoQueueCreate(...) failed.");
            break;
        }
        /* Publish controller device interface */
        if (!NT_SUCCESS(ntStatus = WdfDeviceCreateDeviceInterface(pDevContext->WdfDevice, &GUID_DEVINTERFACE_PWM_CONTROLLER, NULL))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfDeviceCreateDeviceInterface(...) failed.");
            break;
        }
        /* Retrieve device interface symbolic link string */
        WDF_OBJECT_ATTRIBUTES_INIT(&wdfObjectAttributes);
        wdfObjectAttributes.ParentObject = pDevContext->WdfDevice;
        if (!NT_SUCCESS(ntStatus = WdfStringCreate(NULL, &wdfObjectAttributes, &pDevContext->DeviceInterfaceSymlinkName))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfStringCreate(...) failed.");
            break;
        }
        if (!NT_SUCCESS(ntStatus = WdfDeviceRetrieveDeviceInterfaceString(pDevContext->WdfDevice, &GUID_DEVINTERFACE_PWM_CONTROLLER, NULL, pDevContext->DeviceInterfaceSymlinkName))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfDeviceRetrieveDeviceInterfaceString(...) failed.");
            break;
        }
        WdfStringGetUnicodeString(pDevContext->DeviceInterfaceSymlinkName, &pDevContext->DeviceInterfaceSymlinkNameWsz);
        DBG_DEV_PRINT_INFO("Published device interface %wZ", &pDevContext->DeviceInterfaceSymlinkNameWsz);
    } while (0);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

VOID ImxTpmEvtDriverUnload(WDFDRIVER WdfDriver) {
    PAGED_CODE();
    DBG_DEV_METHOD_BEG();
    UNREFERENCED_PARAMETER(WdfDriver);
    DBG_DEV_METHOD_END();
}

IMXPWM_PAGED_SEGMENT_END;

IMXPWM_INIT_SEGMENT_BEGIN;

/*++
Routine Description:
    DriverEntry initializes the driver and is the first routine called by the
    system after the driver is loaded. DriverEntry specifies the other entry
    points in the function driver, such as EvtDevice and DriverUnload.

Parameters Description:

    DriverObject - represents the instance of the function driver that is loaded
    into memory. DriverEntry must initialize members of DriverObject before it
    returns to the caller. DriverObject is allocated by the system before the
    driver is loaded, and it is released by the system after the system unloads
    the function driver from memory.

    RegistryPath - represents the driver specific path in the Registry.
    The function driver can use the path to store driver related data between
    reboots. The path does not store hardware instance specific data.

Return Value:

    STATUS_SUCCESS if successful,
    STATUS_UNSUCCESSFUL otherwise.

--*/
_Use_decl_annotations_
NTSTATUS DriverEntry(DRIVER_OBJECT* DriverObjectPtr, UNICODE_STRING* RegistryPathPtr) {
    NTSTATUS  ntStatus;
    WDFDRIVER wdfDriver;
#if DBG
    KeQuerySystemTimePrecise(&DriverStartTime);
#endif
    PAGED_CODE();
    DBG_DEV_METHOD_BEG_WITH_PARAMS("Driver: 0x%016p, '%S'", DriverObjectPtr, ((PUNICODE_STRING)RegistryPathPtr)->Buffer);
    DBG_DEV_PRINT_INFO("***********************************************************************************");
    DBG_DEV_PRINT_INFO("*** NXP TPM(PWM) miniport driver, date: %s %s                    ***", __DATE__, __TIME__);
    DBG_DEV_PRINT_INFO("***********************************************************************************");

    WDF_DRIVER_CONFIG wdfDriverConfig;
    WDF_DRIVER_CONFIG_INIT(&wdfDriverConfig, ImxTpmEvtDeviceAdd);
    wdfDriverConfig.DriverPoolTag = TPM_MEM_TAG_DRV;
    wdfDriverConfig.EvtDriverUnload = ImxTpmEvtDriverUnload;

    if (!NT_SUCCESS(ntStatus = WdfDriverCreate(DriverObjectPtr, RegistryPathPtr, WDF_NO_OBJECT_ATTRIBUTES, &wdfDriverConfig, &wdfDriver))) {
        DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfDriverCreate() failed");
    }
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

IMXPWM_INIT_SEGMENT_END;
