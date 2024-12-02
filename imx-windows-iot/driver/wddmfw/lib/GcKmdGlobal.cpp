/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmdGlobal.tmh"

#include "GcKmdGlobal.h"
#include "GcKmdDdi.h"

#include "GcKmdDevice.h"
#include "GcKmdContext.h"

DRIVER_OBJECT* GcKmdGlobal::s_pDriverObject;

#if _AMD64_

GcKmdDriverMode GcKmdGlobal::s_DriverMode = RenderOnly;

#else

GcKmdDriverMode GcKmdGlobal::s_DriverMode = FullDriver;

#endif

BOOLEAN GcKmdGlobal::s_bShadowGpuVa = false;    // Can only be changed at driver load time
BOOLEAN GcKmdGlobal::s_bSwitchGpuVa = true;
BOOLEAN GcKmdGlobal::s_bLocalVidMemGeneralUse = false;
BOOLEAN GcKmdGlobal::s_bGdiHwAcceleration = true;
BOOLEAN GcKmdGlobal::s_bMapDummyPage = false;
BOOLEAN GcKmdGlobal::s_bLimitAllocBelow4gbPa = false;

UINT GcKmdGlobal::s_GdiDmaBufferSize = 16*PAGE_SIZE;
UINT GcKmdGlobal::s_DmaBufferSize    = 4*PAGE_SIZE;

BOOLEAN GcKmdGlobal::s_bDodUseHwVSync = true;

extern "C" {

NTSTATUS GcWddmDriverEntry(__in IN DRIVER_OBJECT* pDriverObject, __in IN UNICODE_STRING* pRegistryPath)
{
    return GcKmdGlobal::DriverEntry(pDriverObject, pRegistryPath);
}

} // extern "C"

void
GcKmdGlobal::DdiUnload(
    void)
{
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, "%s\n",
        __FUNCTION__);

    NT_ASSERT(s_pDriverObject);
    WPP_CLEANUP(s_pDriverObject);
    s_pDriverObject = nullptr;
}

#define printk(x, ...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, __VA_ARGS__)

NTSTATUS GcKmdGlobal::DriverEntry(__in IN DRIVER_OBJECT* pDriverObject, __in IN UNICODE_STRING* pRegistryPath)
{
    NTSTATUS    Status;

    GcKmdGlobal::s_pDriverObject = pDriverObject;

    switch (s_DriverMode)
    {
    case RenderOnly:
        printk(" s_DriverMode = RenderOnly\n");
        break;
    case DisplayOnly:
        printk(" s_DriverMode = DisplayOnly\n");
        break;
    case FullDriver:
        printk(" s_DriverMode = FullDriver\n");
        break;
    default:
        printk(" s_DriverMode = undefined\n");
    }

    //
    // Initialize logging
    //
    {
        WPP_INIT_TRACING(pDriverObject, pRegistryPath);

        RECORDER_CONFIGURE_PARAMS recorderConfigureParams;

        RECORDER_CONFIGURE_PARAMS_INIT(&recorderConfigureParams);
        WppRecorderConfigure(&recorderConfigureParams);

        WPP_RECORDER_LEVEL_FILTER(GC_TRACING_VIDPN) = FALSE;
        WPP_RECORDER_LEVEL_FILTER(GC_TRACING_PRESENT) = FALSE;
#if DBG
        WPP_RECORDER_LEVEL_FILTER(GC_TRACING_DEFAULT) = TRUE;
#endif // DBG
    }

    GC_LOG_INFORMATION(
        "Initializing gckmd. (pDriverObject=0x%p, pRegistryPath=%wZ)",
        pDriverObject,
        pRegistryPath);

    if (s_DriverMode != DisplayOnly)
    {
        DRIVER_INITIALIZATION_DATA  DriverInitializationData = {};

        DriverInitializationData.Version = DXGKDDI_INTERFACE_VERSION;

        DriverInitializationData.DxgkDdiAddDevice               = GcKmdDdi::DdiAddAdapter;
        DriverInitializationData.DxgkDdiStartDevice             = GcKmdDdi::DdiStartAdapter;
        DriverInitializationData.DxgkDdiStopDevice              = GcKmdDdi::DdiStopAdapter;
        DriverInitializationData.DxgkDdiRemoveDevice            = GcKmdDdi::DdiRemoveAdapter;

        DriverInitializationData.DxgkDdiDispatchIoRequest       = GcKmdDdi::DdiDispatchIoRequest;
        DriverInitializationData.DxgkDdiInterruptRoutine        = GcKmdDdi::DdiInterruptRoutine;
        DriverInitializationData.DxgkDdiDpcRoutine              = GcKmdDdi::DdiDpcRoutine;

        DriverInitializationData.DxgkDdiQueryChildRelations     = GcKmdDdi::DdiQueryChildRelations;
        DriverInitializationData.DxgkDdiQueryChildStatus        = GcKmdDdi::DdiQueryChildStatus;
        DriverInitializationData.DxgkDdiQueryDeviceDescriptor   = GcKmdDdi::DdiQueryDeviceDescriptor;

        DriverInitializationData.DxgkDdiSetPowerState           = GcKmdDdi::DdiSetPowerState;
        DriverInitializationData.DxgkDdiResetDevice             = GcKmdDdi::DdiResetDevice;
        DriverInitializationData.DxgkDdiUnload                  = GcKmdGlobal::DdiUnload;

        DriverInitializationData.DxgkDdiQueryInterface          = GcKmdDdi::DdiQueryInterface;

        DriverInitializationData.DxgkDdiQueryAdapterInfo        = GcKmdDdi::DdiQueryAdapterInfo;

        DriverInitializationData.DxgkDdiCreateDevice            = GcKmdDdi::DdiCreateDevice;
        DriverInitializationData.DxgkDdiDestroyDevice           = GcKmdDdi::DdiDestroyDevice;

        DriverInitializationData.DxgkDdiCreateAllocation        = GcKmdDdi::DdiCreateAllocation;
        DriverInitializationData.DxgkDdiDestroyAllocation       = GcKmdDdi::DdiDestroyAllocation;

        DriverInitializationData.DxgkDdiOpenAllocation          = GcKmdDdi::DdiOpenAllocation;
        DriverInitializationData.DxgkDdiCloseAllocation         = GcKmDevice::DdiCloseAllocation;
 
        DriverInitializationData.DxgkDdiDescribeAllocation      = GcKmdDdi::DdiDescribeAllocation;
        DriverInitializationData.DxgkDdiGetStandardAllocationDriverData = GcKmdDdi::DdiGetStandardAllocationDriverData;

        DriverInitializationData.DxgkDdiPresent                 = GcKmdDdi::DdiPresent;

        DriverInitializationData.DxgkDdiCreateContext           = GcKmdDdi::DdiCreateContext;
        DriverInitializationData.DxgkDdiDestroyContext          = GcKmdDdi::DdiDestroyContext;

        DriverInitializationData.DxgkDdiSubmitCommand           = GcKmdDdi::DdiSubmitCommand;
        DriverInitializationData.DxgkDdiPreemptCommand          = GcKmdDdi::DdiPreemptCommand;
        DriverInitializationData.DxgkDdiBuildPagingBuffer       = GcKmdDdi::DdiBuildPagingBuffer;

        DriverInitializationData.DxgkDdiResetFromTimeout        = GcKmdDdi::DdiResetFromTimeout;
        DriverInitializationData.DxgkDdiRestartFromTimeout      = GcKmdDdi::DdiRestartFromTimeout;
        DriverInitializationData.DxgkDdiCollectDbgInfo          = GcKmdDdi::DdiCollectDbgInfo;

        DriverInitializationData.DxgkDdiEscape                  = GcKmdDdi::DdiEscape;

        DriverInitializationData.DxgkDdiQueryCurrentFence       = GcKmdDdi::DdiQueryCurrentFence;
        DriverInitializationData.DxgkDdiControlInterrupt        = GcKmdDdi::DdiControlInterrupt;

        DriverInitializationData.DxgkDdiResetEngine             = GcKmdDdi::DdiResetEngine;

        DriverInitializationData.DxgkDdiCancelCommand           = GcKmdDdi::DdiCancelCommand;

        DriverInitializationData.DxgkDdiGetNodeMetadata         = GcKmdDdi::DdiGetNodeMetadata;

        DriverInitializationData.DxgkDdiCalibrateGpuClock       = GcKmdDdi::DdiCalibrateGpuClock;
        DriverInitializationData.DxgkDdiSetStablePowerState     = GcKmdDdi::DdiSetStablePowerState;

        DriverInitializationData.DxgkDdiSubmitCommandVirtual    = GcKmdDdi::DdiSubmitCommandVirtual;
        DriverInitializationData.DxgkDdiSetRootPageTable        = GcKmdDdi::DdiSetRootPageTable;
        DriverInitializationData.DxgkDdiGetRootPageTableSize    = GcKmdDdi::DdiGetRootPageTableSize;

        DriverInitializationData.DxgkDdiCreateProcess           = GcKmdDdi::DdiCreateProcess;
        DriverInitializationData.DxgkDdiDestroyProcess          = GcKmdDdi::DdiDestroyProcess;

        if (GcKmdGlobal::s_bGdiHwAcceleration)
        {
            DriverInitializationData.DxgkDdiRenderGdi   = GcKmdDdi::DdiRenderGdi;
            DriverInitializationData.DxgkDdiRenderKm    = GcKmdDdi::DdiRenderKm;
        }

        if (FullDriver == s_DriverMode)
        {
            DriverInitializationData.DxgkDdiSetPalette                  = GcKmdDisplayDdi::DdiSetPalette;
            DriverInitializationData.DxgkDdiSetPointerPosition          = GcKmdDisplayDdi::DdiSetPointerPosition;
            DriverInitializationData.DxgkDdiSetPointerShape             = GcKmdDisplayDdi::DdiSetPointerShape;
            DriverInitializationData.DxgkDdiIsSupportedVidPn            = GcKmdDisplayDdi::DdiIsSupportedVidPn;
            DriverInitializationData.DxgkDdiRecommendFunctionalVidPn    = GcKmdDisplayDdi::DdiRecommendFunctionalVidPn;
            DriverInitializationData.DxgkDdiEnumVidPnCofuncModality     = GcKmdDisplayDdi::DdiEnumVidPnCofuncModality;
            DriverInitializationData.DxgkDdiSetVidPnSourceAddress       = GcKmdDisplayDdi::DdiSetVidPnSourceAddress;
            DriverInitializationData.DxgkDdiSetVidPnSourceVisibility    = GcKmdDisplayDdi::DdiSetVidPnSourceVisibility;
            DriverInitializationData.DxgkDdiCommitVidPn                 = GcKmdDisplayDdi::DdiCommitVidPn;
            DriverInitializationData.DxgkDdiUpdateActiveVidPnPresentPath    = GcKmdDisplayDdi::DdiUpdateActiveVidPnPresentPath;

            DriverInitializationData.DxgkDdiRecommendMonitorModes       = GcKmdDisplayDdi::DdiRecommendMonitorModes;

            DriverInitializationData.DxgkDdiGetScanLine                 = GcKmdDisplayDdi::DdiGetScanLine;

            // DriverInitializationData.DxgkDdiQueryVidPnHWCapability   = GcKmdDisplayDdi::DdiQueryVidPnHWCapability;

            DriverInitializationData.DxgkDdiStopDeviceAndReleasePostDisplayOwnership = GcKmdDisplayDdi::DdiStopDeviceAndReleasePostDisplayOwnership;

            DriverInitializationData.DxgkDdiSystemDisplayEnable;
            DriverInitializationData.DxgkDdiSystemDisplayWrite;

            DriverInitializationData.DxgkDdiSetVidPnSourceAddressWithMultiPlaneOverlay3 = GcKmdDisplayDdi::DdiSetVidPnSourceAddressWithMultiPlaneOverlay3;

            DriverInitializationData.DxgkDdiUpdateMonitorLinkInfo       = GcKmdDisplayDdi::DdiUpdateMonitorLinkInfo;

            DriverInitializationData.DxgkDdiGetChildContainerId         = GcKmdDisplayDdi::DdiGetChildContainerId;

            DriverInitializationData.DxgkDdiControlInterrupt;

            DriverInitializationData.DxgkDdiSetPowerComponentFState;
            DriverInitializationData.DxgkDdiPowerRuntimeControlRequest;

            DriverInitializationData.DxgkDdiNotifySurpriseRemoval;

            DriverInitializationData.DxgkDdiPowerRuntimeSetDeviceHandle;
        }

        Status = DxgkInitialize(pDriverObject, pRegistryPath, &DriverInitializationData);
    }
    else
    {
        KMDDOD_INITIALIZATION_DATA  KmDodInitializationData = {};

        KmDodInitializationData.Version = DXGKDDI_INTERFACE_VERSION;

        KmDodInitializationData.DxgkDdiAddDevice                = GcKmdDdi::DdiAddAdapter;
        KmDodInitializationData.DxgkDdiStartDevice              = GcKmdDdi::DdiStartAdapter;
        KmDodInitializationData.DxgkDdiStopDevice               = GcKmdDdi::DdiStopAdapter;
        KmDodInitializationData.DxgkDdiRemoveDevice             = GcKmdDdi::DdiRemoveAdapter;
        KmDodInitializationData.DxgkDdiDispatchIoRequest        = GcKmdDdi::DdiDispatchIoRequest;
        KmDodInitializationData.DxgkDdiInterruptRoutine         = GcKmdDdi::DdiInterruptRoutine;
        KmDodInitializationData.DxgkDdiDpcRoutine               = GcKmdDdi::DdiDpcRoutine;
        KmDodInitializationData.DxgkDdiQueryChildRelations      = GcKmdDdi::DdiQueryChildRelations;
        KmDodInitializationData.DxgkDdiQueryChildStatus         = GcKmdDdi::DdiQueryChildStatus;
        KmDodInitializationData.DxgkDdiQueryDeviceDescriptor    = GcKmdDdi::DdiQueryDeviceDescriptor;
        KmDodInitializationData.DxgkDdiSetPowerState            = GcKmdDdi::DdiSetPowerState;
        KmDodInitializationData.DxgkDdiNotifyAcpiEvent;
        KmDodInitializationData.DxgkDdiResetDevice              = GcKmdDdi::DdiResetDevice;
        KmDodInitializationData.DxgkDdiUnload                   = GcKmdGlobal::DdiUnload;
        KmDodInitializationData.DxgkDdiQueryInterface           = GcKmdDdi::DdiQueryInterface;
        KmDodInitializationData.DxgkDdiControlEtwLogging;
        KmDodInitializationData.DxgkDdiQueryAdapterInfo         = GcKmdDdi::DdiQueryAdapterInfo;
        KmDodInitializationData.DxgkDdiSetPalette               = GcKmdDisplayDdi::DdiSetPalette;
        KmDodInitializationData.DxgkDdiSetPointerPosition       = GcKmdDisplayDdi::DdiSetPointerPosition;
        KmDodInitializationData.DxgkDdiSetPointerShape          = GcKmdDisplayDdi::DdiSetPointerShape;
        KmDodInitializationData.DxgkDdiEscape                   = GcKmdDdi::DdiEscape;
        KmDodInitializationData.DxgkDdiCollectDbgInfo;
        KmDodInitializationData.DxgkDdiIsSupportedVidPn         = GcKmdDisplayDdi::DdiIsSupportedVidPn;
        KmDodInitializationData.DxgkDdiRecommendFunctionalVidPn = GcKmdDisplayDdi::DdiRecommendFunctionalVidPn;
        KmDodInitializationData.DxgkDdiEnumVidPnCofuncModality  = GcKmdDisplayDdi::DdiEnumVidPnCofuncModality;
        KmDodInitializationData.DxgkDdiSetVidPnSourceVisibility = GcKmdDisplayDdi::DdiSetVidPnSourceVisibility;
        KmDodInitializationData.DxgkDdiCommitVidPn              = GcKmdDisplayDdi::DdiCommitVidPn;
        KmDodInitializationData.DxgkDdiUpdateActiveVidPnPresentPath = GcKmdDisplayDdi::DdiUpdateActiveVidPnPresentPath;

        //
        // For DOD DdiGetScanLine and DdiControlInterrupt need to implemented together or neither.
        //
        
        if (GcKmdGlobal::s_bDodUseHwVSync)
        {
            KmDodInitializationData.DxgkDdiControlInterrupt     = GcKmdDdi::DdiControlInterrupt;
            KmDodInitializationData.DxgkDdiGetScanLine          = GcKmdDisplayDdi::DdiGetScanLine;
        }

        //
        // DdiRecommendMonitorModes and DdiQueryVidPnHWCapability are optional
        //

        KmDodInitializationData.DxgkDdiRecommendMonitorModes    = GcKmdDisplayDdi::DdiRecommendMonitorModes;

        
        // KmDodInitializationData.DxgkDdiQueryVidPnHWCapability   = GcKmdDisplayDdi::DdiQueryVidPnHWCapability;
        
        KmDodInitializationData.DxgkDdiPresentDisplayOnly       = GcKmdDisplayDdi::DdiPresentDisplayOnly;

        KmDodInitializationData.DxgkDdiStopDeviceAndReleasePostDisplayOwnership = GcKmdDisplayDdi::DdiStopDeviceAndReleasePostDisplayOwnership;

        KmDodInitializationData.DxgkDdiSystemDisplayEnable;
        KmDodInitializationData.DxgkDdiSystemDisplayWrite;

        KmDodInitializationData.DxgkDdiGetChildContainerId      = GcKmdDisplayDdi::DdiGetChildContainerId;;


        KmDodInitializationData.DxgkDdiSetPowerComponentFState;
        KmDodInitializationData.DxgkDdiPowerRuntimeControlRequest;

        KmDodInitializationData.DxgkDdiNotifySurpriseRemoval;

        KmDodInitializationData.DxgkDdiPowerRuntimeSetDeviceHandle;

        Status = DxgkInitializeDisplayOnlyDriver(pDriverObject, pRegistryPath, &KmDodInitializationData);
    }

    return Status;
}

