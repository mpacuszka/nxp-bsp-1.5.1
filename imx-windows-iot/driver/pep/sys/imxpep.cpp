// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022-2023 NXP
// Licensed under the MIT License.
//
//
// Module Name:
//
//   imxpep.cpp
//
// Abstract:
//
//   IMX Power Engine Plugin (PEP)
//

#include "precomp.h"

#include "trace.h"
#include "imxpep.tmh"

#include "imxpeputil.h"
#include "imxpepioctl.h"
#include "imxpephw.h"
#include "imxpep.h"
#include "imxpep_dbg.h"

IMX_NONPAGED_SEGMENT_BEGIN; //==============================================

namespace { // static

    //IMX_PEP* pepGlobalContextPtr;

    _IRQL_requires_max_(DISPATCH_LEVEL)
    NTSTATUS IMXCompleteRequest (
        IRP* IrpPtr,
        NTSTATUS Status,
        ULONG_PTR Information = 0
        )
    {
        IrpPtr->IoStatus.Status = Status;
        IrpPtr->IoStatus.Information = Information,
        IoCompleteRequest(IrpPtr, IO_NO_INCREMENT);
        return Status;
    }

    _Function_class_(IO_COMPLETION_ROUTINE)
    _IRQL_requires_same_
    _IRQL_requires_max_(DISPATCH_LEVEL)
    NTSTATUS IMXSyncCompletion (
        _In_ DEVICE_OBJECT* /*DeviceObjectPtr*/,
        _In_ IRP* /*IrpPtr*/,
        _In_ void* ContextPtr
        )
    {
        auto eventPtr = static_cast<KEVENT*>(ContextPtr);
        KeSetEvent(eventPtr, IO_NO_INCREMENT, FALSE);
        return STATUS_MORE_PROCESSING_REQUIRED;
    }

    _IRQL_requires_max_(DISPATCH_LEVEL)
    NTSTATUS IMXForwardAndWait (
        DEVICE_OBJECT* DeviceObjectPtr,
        IRP* IrpPtr
        )
    {
        KEVENT event;
        KeInitializeEvent(&event, SynchronizationEvent, FALSE);

        IoCopyCurrentIrpStackLocationToNext(IrpPtr);
        IoSetCompletionRoutine(IrpPtr, IMXSyncCompletion, &event, TRUE, TRUE, TRUE);

        NTSTATUS status = IoCallDriver(DeviceObjectPtr, IrpPtr);
        if (status == STATUS_PENDING) {
            KeWaitForSingleObject(&event, Executive, KernelMode, FALSE, NULL);
            status = IrpPtr->IoStatus.Status;
        }

        return status;
    }

    template <typename TInputBuffer>
    NTSTATUS IMXRetrieveInputBuffer (IRP* IrpPtr, TInputBuffer** InputBufferPPtr)
    {
        IO_STACK_LOCATION* ioStackPtr = IoGetCurrentIrpStackLocation(IrpPtr);
        NT_ASSERT(ioStackPtr->MajorFunction == IRP_MJ_DEVICE_CONTROL);
        NT_ASSERT((IrpPtr->Flags & IRP_BUFFERED_IO) != 0);

        // validate input buffer size
        if (ioStackPtr->Parameters.DeviceIoControl.InputBufferLength <
            sizeof(TInputBuffer)) {

            return STATUS_INVALID_PARAMETER;
        }

        *InputBufferPPtr = static_cast<TInputBuffer*>(
                IrpPtr->AssociatedIrp.SystemBuffer);

        return STATUS_SUCCESS;
    }

    template <typename TOutputBuffer>
    NTSTATUS IMXRetrieveOutputBuffer (IRP* IrpPtr, TOutputBuffer** OutputBufferPPtr)
    {
        IO_STACK_LOCATION* ioStackPtr = IoGetCurrentIrpStackLocation(IrpPtr);
        NT_ASSERT(ioStackPtr->MajorFunction == IRP_MJ_DEVICE_CONTROL);
        NT_ASSERT((IrpPtr->Flags & IRP_BUFFERED_IO) != 0);

        // validate output buffer size
        if (ioStackPtr->Parameters.DeviceIoControl.OutputBufferLength <
            sizeof(TOutputBuffer)) {

            return STATUS_INVALID_PARAMETER;
        }

        *OutputBufferPPtr = static_cast<TOutputBuffer*>(
                IrpPtr->AssociatedIrp.SystemBuffer);

        return STATUS_SUCCESS;
    }

} // namespace "static"

//void __cdecl operator delete ( void* ) {}

void __cdecl operator delete (void*, UINT64) {}

_Use_decl_annotations_
BOOLEAN IMX_PEP::AcceptAcpiNotification (ULONG Notification, void* DataPtr)
{
    IMX_ASSERT_MAX_IRQL(DISPATCH_LEVEL);

    DBG_DRV_METHOD_BEG_WITH_PARAMS("AcceptAcpiNotification: 0x%02x %s", Notification, Dbg_GetAcpiNotifyName(Notification));

    IMX_PEP* thisPtr = pepGlobalContextPtr;

    switch (Notification) {
    case PEP_NOTIFY_ACPI_PREPARE_DEVICE:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->AcpiPrepareDevice(
            static_cast<PEP_ACPI_PREPARE_DEVICE*>(DataPtr));

    case PEP_NOTIFY_ACPI_ABANDON_DEVICE:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->AcpiAbandonDevice(
            static_cast<PEP_ACPI_ABANDON_DEVICE*>(DataPtr));

    case PEP_NOTIFY_ACPI_REGISTER_DEVICE:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->AcpiRegisterDevice(
            static_cast<PEP_ACPI_REGISTER_DEVICE*>(DataPtr));

    case PEP_NOTIFY_ACPI_ENUMERATE_DEVICE_NAMESPACE:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->AcpiEnumerateDeviceNamespace(
            static_cast<PEP_ACPI_ENUMERATE_DEVICE_NAMESPACE*>(DataPtr));

    case PEP_NOTIFY_ACPI_QUERY_OBJECT_INFORMATION:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->AcpiQueryObjectInformation(
            static_cast<PEP_ACPI_QUERY_OBJECT_INFORMATION*>(DataPtr));

    case PEP_NOTIFY_ACPI_EVALUATE_CONTROL_METHOD:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->AcpiEvaluateControlMethod(
            static_cast<PEP_ACPI_EVALUATE_CONTROL_METHOD*>(DataPtr));

    case PEP_NOTIFY_ACPI_UNREGISTER_DEVICE:
    case PEP_NOTIFY_ACPI_QUERY_DEVICE_CONTROL_RESOURCES:
    case PEP_NOTIFY_ACPI_TRANSLATED_DEVICE_CONTROL_RESOURCES:
    case PEP_NOTIFY_ACPI_WORK:
    default:
        IMX_LOG_TRACE(
            "Unsupported ACPI notification. (Notification = 0x%x)",
            Notification);
        return FALSE;
    }
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::AcceptDeviceNotification (ULONG Notification, void* DataPtr)
{
    IMX_ASSERT_MAX_IRQL(DISPATCH_LEVEL);

    DBG_DRV_METHOD_BEG_WITH_PARAMS("AcceptDeviceNotification: 0x%02x %s", Notification, Dbg_GetDpmNotifyName(Notification));

    IMX_PEP* thisPtr = pepGlobalContextPtr;

    switch (Notification) {
    case PEP_DPM_PREPARE_DEVICE:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->DpmPrepareDevice(
            static_cast<PEP_PREPARE_DEVICE*>(DataPtr));

    case PEP_DPM_ABANDON_DEVICE:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->DpmAbandonDevice(
            static_cast<PEP_ABANDON_DEVICE*>(DataPtr));

    case PEP_DPM_REGISTER_DEVICE:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->DpmRegisterDevice(
            static_cast<PEP_REGISTER_DEVICE_V2*>(DataPtr));

    case PEP_DPM_UNREGISTER_DEVICE:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->DpmUnregisterDevice(
            static_cast<PEP_UNREGISTER_DEVICE*>(DataPtr));

    case PEP_DPM_DEVICE_POWER_STATE:
        return thisPtr->DpmDevicePowerState(
            static_cast<PEP_DEVICE_POWER_STATE*>(DataPtr));

    case PEP_DPM_REGISTER_DEBUGGER:
        return thisPtr->DpmRegisterDebugger(
            static_cast<PEP_REGISTER_DEBUGGER*>(DataPtr));

    case PEP_DPM_DEVICE_IDLE_CONSTRAINTS:
        return thisPtr->DpmDeviceIdleContraints(
            static_cast<PEP_DEVICE_PLATFORM_CONSTRAINTS*>(DataPtr));

    case PEP_DPM_NOTIFY_COMPONENT_IDLE_STATE:
        return thisPtr->DpmComponentIdleState(
            static_cast<PEP_NOTIFY_COMPONENT_IDLE_STATE*>(DataPtr));

    case PEP_DPM_COMPONENT_IDLE_CONSTRAINTS:
        return thisPtr->DpmComponentIdleConstraints(
            static_cast<PEP_COMPONENT_PLATFORM_CONSTRAINTS*>(DataPtr));

    case PEP_DPM_REGISTER_CRASHDUMP_DEVICE:
        return thisPtr->DpmRegisterCrashdumpDevice(
            static_cast<PEP_REGISTER_CRASHDUMP_DEVICE*>(DataPtr));

    case PEP_DPM_WORK:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->DpmWork(static_cast<PEP_WORK*>(DataPtr));

    #define PEP_DPM_LOW_POWER_EPOCH_PRIVATE 0x18 // Deprecated, but it is comming so we can see the value for debug purposes
    case PEP_DPM_LOW_POWER_EPOCH_PRIVATE:
        return thisPtr->DpmLowPowerEpoch(DataPtr);

    case PEP_DPM_SYSTEM_LATENCY_UPDATE:
        return thisPtr->DpmSystemLatencyUpdate(
            static_cast<PEP_SYSTEM_LATENCY*>(DataPtr));

    case PEP_DPM_DEVICE_STARTED:
    case PEP_DPM_QUERY_COMPONENT_PERF_CAPABILTIES:
    case PEP_DPM_QUERY_COMPONENT_PERF_SET:
    case PEP_DPM_QUERY_COMPONENT_PERF_SET_NAME:
    case PEP_DPM_QUERY_COMPONENT_PERF_STATES:
    case PEP_DPM_REGISTER_COMPONENT_PERF_STATES:
    case PEP_DPM_REQUEST_COMPONENT_PERF_STATE:
    case PEP_DPM_QUERY_CURRENT_COMPONENT_PERF_STATE:
    case PEP_DPM_QUERY_DEBUGGER_TRANSITION_REQUIREMENTS:
    case PEP_DPM_QUERY_SOC_SUBSYSTEM_COUNT:
    case PEP_DPM_QUERY_SOC_SUBSYSTEM:
    case PEP_DPM_RESET_SOC_SUBSYSTEM_ACCOUNTING:
    case PEP_DPM_QUERY_SOC_SUBSYSTEM_BLOCKING_TIME:
    case PEP_DPM_QUERY_SOC_SUBSYSTEM_METADATA:
    case PEP_DPM_POWER_CONTROL_REQUEST:
    case PEP_DPM_POWER_CONTROL_COMPLETE:
    default:
        IMX_LOG_TRACE(
            "Unsupported DPM notification. (Notification = 0x%x)",
            Notification);

        return FALSE;
    }
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::AcceptProcessorNotification (
    PEPHANDLE Handle,
    ULONG Notification,
    PVOID DataPtr
    )
{
    IMX_PEP* thisPtr = pepGlobalContextPtr;

    if (Notification != PEP_NOTIFY_PPM_IDLE_EXECUTE && 
        Notification != PEP_NOTIFY_PPM_IDLE_COMPLETE && 
        Notification != PEP_NOTIFY_PPM_IDLE_PRE_EXECUTE)
    {
        DBG_DRV_METHOD_BEG_WITH_PARAMS("AcceptProcessorNotification: 0x%02x %s", Notification, Dbg_GetPpmNotifyName(Notification));
    }

    switch (Notification) {
    case PEP_NOTIFY_PPM_QUERY_CAPABILITIES:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->PpmQueryCapabilities(
            Handle,
            static_cast<PEP_PPM_QUERY_CAPABILITIES*>(DataPtr));

    case PEP_NOTIFY_PPM_QUERY_IDLE_STATES_V2:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->PpmQueryIdleStatesV2(
            Handle,
            static_cast<PEP_PPM_QUERY_IDLE_STATES_V2*>(DataPtr));

    case PEP_NOTIFY_PPM_QUERY_PLATFORM_STATES:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->PpmQueryPlatformStates(
            Handle,
            static_cast<PEP_PPM_QUERY_PLATFORM_STATES*>(DataPtr));

    case PEP_NOTIFY_PPM_QUERY_COORDINATED_STATES:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->PpmQueryCoordinatedStates(
            Handle,
            static_cast<PEP_PPM_QUERY_COORDINATED_STATES*>(DataPtr));

    case PEP_NOTIFY_PPM_QUERY_COORDINATED_DEPENDENCY:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->PpmQueryCoordinatedDependency(
            Handle,
            static_cast<PEP_PPM_QUERY_COORDINATED_DEPENDENCY*>(DataPtr));

    case PEP_NOTIFY_PPM_IDLE_EXECUTE:
        return thisPtr->PpmIdleExecute(
            Handle,
            static_cast<PEP_PPM_IDLE_EXECUTE_V2*>(DataPtr));

    case PEP_NOTIFY_PPM_IDLE_COMPLETE:
        return thisPtr->PpmIdleComplete(
            Handle,
            static_cast<PEP_PPM_IDLE_COMPLETE_V2*>(DataPtr));

    case PEP_NOTIFY_PPM_QUERY_VETO_REASONS:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->PpmQueryVetoReasons(
            Handle,
            static_cast<PEP_PPM_QUERY_VETO_REASONS*>(DataPtr));

    case PEP_NOTIFY_PPM_QUERY_VETO_REASON:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->PpmQueryVetoReason(
            Handle,
            static_cast<PEP_PPM_QUERY_VETO_REASON*>(DataPtr));

    case PEP_NOTIFY_PPM_ENUMERATE_BOOT_VETOES:
        IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
        return thisPtr->PpmEnumerateBootVetoes(Handle);

    case PEP_NOTIFY_PPM_TEST_IDLE_STATE:
        return thisPtr->PpmTestIdleState(
            Handle,
            static_cast<PEP_PPM_TEST_IDLE_STATE*>(DataPtr));

    case PEP_NOTIFY_PPM_QUERY_PROCESSOR_STATE_NAME:
        return thisPtr->PpmQueryProcessorStateName(
            Handle,
            static_cast<PEP_PPM_QUERY_STATE_NAME*>(DataPtr));

    case PEP_NOTIFY_PPM_IS_PROCESSOR_HALTED:
    case PEP_NOTIFY_PPM_INITIATE_WAKE:
    case PEP_NOTIFY_PPM_QUERY_FEEDBACK_COUNTERS:
    case PEP_NOTIFY_PPM_FEEDBACK_READ:
    case PEP_NOTIFY_PPM_QUERY_PERF_CAPABILITIES:
    case PEP_NOTIFY_PPM_PERF_CONSTRAINTS:
    case PEP_NOTIFY_PPM_PERF_SET:
    case PEP_NOTIFY_PPM_PARK_SELECTION:
    case PEP_NOTIFY_PPM_CST_STATES:
    case PEP_NOTIFY_PPM_QUERY_PLATFORM_STATE:
    case PEP_NOTIFY_PPM_IDLE_PRE_EXECUTE:
    case PEP_NOTIFY_PPM_UPDATE_PLATFORM_STATE:
    case PEP_NOTIFY_PPM_QUERY_PLATFORM_STATE_RESIDENCIES:
    case PEP_NOTIFY_PPM_QUERY_COORDINATED_STATE_NAME:
    case PEP_NOTIFY_PPM_PARK_SELECTION_V2:
    case PEP_NOTIFY_PPM_PARK_MASK:
    case PEP_NOTIFY_PPM_PERF_CHECK_COMPLETE:
    case PEP_NOTIFY_PPM_LPI_SUPPORTED:
    case PEP_NOTIFY_PPM_LPI_PROCESSOR_STATES:
    default:
        return FALSE;
    }
}


_Use_decl_annotations_
NTSTATUS IMX_PEP::DispatchDeviceIoControl (DEVICE_OBJECT* DeviceObjectPtr, IRP* IrpPtr)
{
    IMX_ASSERT_MAX_IRQL(DISPATCH_LEVEL);

    const IO_STACK_LOCATION* ioStackPtr = IoGetCurrentIrpStackLocation(IrpPtr);
    NT_ASSERT(ioStackPtr->MajorFunction == IRP_MJ_DEVICE_CONTROL);

    if (KeGetCurrentIrql() > PASSIVE_LEVEL) {
        IMX_LOG_ERROR(
            "imxpep may only be called at PASSIVE_LEVEL. (KeGetCurrentIrql() = %!irql!)",
            KeGetCurrentIrql());

        return IMXCompleteRequest(IrpPtr, STATUS_INVALID_LEVEL);
    }

    auto DeviceExtensionPtr = static_cast<_DEVICE_EXTENSION*>(
                                        DeviceObjectPtr->DeviceExtension);
    auto thisPtr = DeviceExtensionPtr->ImxPepPtr;

    // serialize all IRPs for simplicity
    NTSTATUS status = KeWaitForSingleObject(
            &thisPtr->ioRequestSemaphore,
            Executive,
            KernelMode,
            FALSE,
            nullptr);

    UNREFERENCED_PARAMETER(status);
    NT_ASSERT(status == STATUS_SUCCESS);

    auto releaseSemaphore = IMX_FINALLY::Do([&] {
        KeReleaseSemaphore(&thisPtr->ioRequestSemaphore, IO_NO_INCREMENT, 1, FALSE);
    });

    switch (ioStackPtr->Parameters.DeviceIoControl.IoControlCode) {
    case IOCTL_IMXPEP_DUMP_REGISTERS:
        return thisPtr->ioctlDumpRegisters(DeviceObjectPtr, IrpPtr);

    case IOCTL_IMXPEP_STAT:
        return thisPtr->ioctlStat(DeviceObjectPtr, IrpPtr);

    default:
        return IMXCompleteRequest(IrpPtr, STATUS_INVALID_DEVICE_REQUEST);
    }
}

_Use_decl_annotations_
NTSTATUS IMX_PEP::DispatchUnhandledIrp (DEVICE_OBJECT* DeviceObjectPtr, IRP* IrpPtr)
{
    IMX_ASSERT_MAX_IRQL(DISPATCH_LEVEL);

    auto DeviceExtensionPtr = static_cast<_DEVICE_EXTENSION*>(
                                        DeviceObjectPtr->DeviceExtension);
    auto thisPtr = DeviceExtensionPtr->ImxPepPtr;

    // Forward all IRPs
    IoSkipCurrentIrpStackLocation(IrpPtr);
    return IoCallDriver(thisPtr->lowerDeviceObjectPtr, IrpPtr);
}

IMX_NONPAGED_SEGMENT_END; //================================================
IMX_PAGED_SEGMENT_BEGIN; //=================================================

// Toggle this variable in WinDbg to disable PPM (Processor Power Management)
volatile int usePpm = 1;

_Use_decl_annotations_
NTSTATUS IMX_PEP::RegisterPlugin ()
{
    PAGED_CODE();

    NT_ASSERT(pepGlobalContextPtr == nullptr);
    pepGlobalContextPtr = this;

    // Do not use processor power management on 8MQ because of USB wake-up issue, 93 needs more validation
    usePpm = IMX_CPU_TYPE(cpuRevision) != IMX_CPU_MX8MQ && IMX_CPU_TYPE(cpuRevision) != IMX_CPU_MX93;

    // Register with PEP framework
    PEP_INFORMATION pepInfo = {0};
    pepInfo.Version = PEP_INFORMATION_VERSION;
    pepInfo.Size = sizeof(pepInfo);
    pepInfo.AcceptAcpiNotification = IMX_PEP::AcceptAcpiNotification;
    pepInfo.AcceptDeviceNotification = IMX_PEP::AcceptDeviceNotification;
    if (usePpm)
    {
        pepInfo.AcceptProcessorNotification = IMX_PEP::AcceptProcessorNotification;
    }
    else
    {
        pepInfo.AcceptProcessorNotification = nullptr;
    }

    this->pepKernelInfo = PEP_KERNEL_INFORMATION();
    this->pepKernelInfo.Version = PEP_INFORMATION_VERSION;
    this->pepKernelInfo.Size = sizeof(this->pepKernelInfo);
    NTSTATUS status = PoFxRegisterPlugin(&pepInfo, &this->pepKernelInfo);
    if (!NT_SUCCESS(status)) {
        IMX_LOG_ERROR(
            "Failed to register with the PEP framework. (status = %!STATUS!)",
            status);

        NT_ASSERT(FALSE);
        pepGlobalContextPtr = nullptr;
        return status;
    }

    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS IMX_PEP::ioctlDumpRegisters (DEVICE_OBJECT* /*DeviceObjectPtr*/, IRP* IrpPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();

    IMXPEP_DUMP_REGISTERS_OUTPUT* outputBufferPtr;
    NTSTATUS status = IMXRetrieveOutputBuffer(IrpPtr, &outputBufferPtr);
    if (!NT_SUCCESS(status)) {
        return IMXCompleteRequest(IrpPtr, status);
    }

    outputBufferPtr->Ccm.CCR = 0x12345678; //A test value, TBD remove

    // Analog
    outputBufferPtr->Analog.PLL_ARM =
        READ_REGISTER_NOFENCE_ULONG(&this->analogRegistersPtr->ARM_PLL_CFG0);

    return IMXCompleteRequest(IrpPtr, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

_Use_decl_annotations_
NTSTATUS IMX_PEP::ioctlStat(DEVICE_OBJECT* /*DeviceObjectPtr*/, IRP* IrpPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();

    IMXPEP_STAT_OUTPUT* outputBufferPtr;
    NTSTATUS status = IMXRetrieveOutputBuffer(IrpPtr, &outputBufferPtr);
    if (!NT_SUCCESS(status)) {
        return IMXCompleteRequest(IrpPtr, status);
    }

    Stat.FillIoctlBuffer(outputBufferPtr);

    return IMXCompleteRequest(IrpPtr, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

_Use_decl_annotations_
NTSTATUS IMX_PEP::InitializeResources ()
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();

    NTSTATUS status;
    PHYSICAL_ADDRESS ccmPhysAddress = {};
    PHYSICAL_ADDRESS analogPhysAddress = {};
    PHYSICAL_ADDRESS gpcPhysAddress = {};

    //
    // Map CCM Registers
    //
    ccmPhysAddress.QuadPart = MX8_CCM_BASE;

    //
    // Map Analog Registers
    //
    analogPhysAddress.QuadPart = MX8_CCM_ANALOG_BASE;
    this->analogRegistersPtr = static_cast<MX8_CCM_ANALOG_REGISTERS*>(
            MmMapIoSpaceEx(
                analogPhysAddress,
                sizeof(*this->analogRegistersPtr),
                PAGE_READWRITE | PAGE_NOCACHE));

    if (this->analogRegistersPtr == nullptr) {
        IMX_LOG_LOW_MEMORY("Failed to map memory for ANALOG registers.");
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    //
    // Map GPC Registers
    //
    gpcPhysAddress.QuadPart = MX8_GPC_BASE;
    this->gpcRegistersPtr = static_cast<MX8_GPC_REGISTERS*>(
            MmMapIoSpaceEx(
                gpcPhysAddress,
                MX8_GPC_LENGTH,
                PAGE_READWRITE | PAGE_NOCACHE));

    if (this->gpcRegistersPtr == nullptr) {
        IMX_LOG_LOW_MEMORY("Failed to map memory for GPC registers.");
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    status = ExInitializeLookasideListEx(
        &this->workQueue.LookasideList,
        nullptr,                // Allocate
        nullptr,                // Free
        NonPagedPoolNx,
        0,                      // Flags
        sizeof(_WORK_ITEM),
        _POOL_TAG,
        0);                     // Depth

    if (!NT_SUCCESS(status)) {
        IMX_LOG_ERROR(
            "Failed to initialize work item lookaside list. (status = %!STATUS!)",
            status);

        return status;
    }


    //
    // Register with PEP framework
    //
    return this->RegisterPlugin();
}

_Use_decl_annotations_
NTSTATUS IMX_PEP::startDevice (DEVICE_OBJECT* /*DeviceObjectPtr*/, IRP* /*IrpPtr*/)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();

    NTSTATUS status = IoSetDeviceInterfaceState(
            &this->deviceInterfaceName,
            TRUE);

    if (!NT_SUCCESS(status)) {
        IMX_LOG_ERROR(
            "Failed to enable device interface. (status = %!STATUS!, this->deviceInterfaceName = %wZ)",
            status,
            &this->deviceInterfaceName);

        return status;
    }

    return STATUS_SUCCESS;
}


_Use_decl_annotations_
NTSTATUS IMX_PEP::DispatchPnp (DEVICE_OBJECT* DeviceObjectPtr, IRP* IrpPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();

    NTSTATUS status;

    IO_STACK_LOCATION* ioStackPtr = IoGetCurrentIrpStackLocation(IrpPtr);
    NT_ASSERT(ioStackPtr->MajorFunction == IRP_MJ_PNP);

    auto DeviceExtensionPtr = static_cast<_DEVICE_EXTENSION*>(
                                        DeviceObjectPtr->DeviceExtension);
    auto thisPtr = DeviceExtensionPtr->ImxPepPtr;

    switch (ioStackPtr->MinorFunction) {
    case IRP_MN_START_DEVICE:
    {
        status = IMXForwardAndWait(
                thisPtr->lowerDeviceObjectPtr,
                IrpPtr);

        if (!NT_SUCCESS(status)) {
            return IMXCompleteRequest(IrpPtr, status);
        }

        status = thisPtr->startDevice(DeviceObjectPtr, IrpPtr);
        return IMXCompleteRequest(IrpPtr, status);
    }
    case IRP_MN_QUERY_CAPABILITIES:
    {
        status = IMXForwardAndWait(
                thisPtr->lowerDeviceObjectPtr,
                IrpPtr);

        if (!NT_SUCCESS(status)) {
            return IMXCompleteRequest(IrpPtr, status);
        }

        ioStackPtr = IoGetCurrentIrpStackLocation(IrpPtr);
        DEVICE_CAPABILITIES* deviceCapsPtr =
                ioStackPtr->Parameters.DeviceCapabilities.Capabilities;

        deviceCapsPtr->EjectSupported = FALSE;
        deviceCapsPtr->Removable = FALSE;
        deviceCapsPtr->SilentInstall = TRUE;

        IMX_LOG_INFORMATION("PNP Capabilities queried");
        return IMXCompleteRequest(IrpPtr, STATUS_SUCCESS);
    }
    case IRP_MN_QUERY_REMOVE_DEVICE:
        // The device cannot be removed
        return IMXCompleteRequest(IrpPtr, STATUS_UNSUCCESSFUL);

    case IRP_MN_REMOVE_DEVICE:
    {
        //
        // Although the PEP device is not removable, this IRP can be sent
        // if this device (or another device in the stack) failed to start.
        //

        DEVICE_OBJECT* lowerDeviceObjectPtr = thisPtr->lowerDeviceObjectPtr;

        // pass to next lower level driver
        IrpPtr->IoStatus.Status = STATUS_SUCCESS;
        IoSkipCurrentIrpStackLocation(IrpPtr);
        status = IoCallDriver(lowerDeviceObjectPtr, IrpPtr);

        IoDetachDevice(lowerDeviceObjectPtr);
        IoDeleteDevice(DeviceObjectPtr);

        return status;
    }

    default:
        // Forward all other PNP IRPs to next lower device
        IoSkipCurrentIrpStackLocation(IrpPtr);
        return IoCallDriver(thisPtr->lowerDeviceObjectPtr, IrpPtr);

    } // switch
}

//
// Do-nothing dispatch handler for IRP_MJ_CREATE and IRP_MJ_CLOSE
//
_Use_decl_annotations_
NTSTATUS IMX_PEP::DispatchFileCreateClose (
    DEVICE_OBJECT* /*DeviceObjectPtr*/,
    IRP* IrpPtr
    )
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    return IMXCompleteRequest(IrpPtr, STATUS_SUCCESS);
}

IMX_PEP* IMX_PEP::pepGlobalContextPtr = nullptr;

_Use_decl_annotations_
IMX_PEP::IMX_PEP (UINT32 cpuRev) :
    cpuRevision(cpuRev),
    activeProcessorCount(),
    //ccmRegistersPtr(),
    analogRegistersPtr(),
    /*gpcRegistersPtr(),
    mmdcRegistersPtr(),
    iomuxcRegistersPtr(),
    armMpRegistersPtr(),
    lowerDeviceObjectPtr(),*/
    pdoPtr(),
    fdoPtr(),
    deviceInterfaceName(),
    pepKernelInfo(),
    workQueue()
{
    PAGED_CODE();

    for (ULONG i = 0; i < ARRAYSIZE(this->unmaskedInterruptsCopy.Mask); ++i) {
        this->unmaskedInterruptsCopy.Mask[i] = 0xffffffff;
    }

    KeInitializeSemaphore(&this->ioRequestSemaphore, 1, 1);
    RtlZeroMemory(this->deviceData, sizeof(this->deviceData));
    KeInitializeSpinLock(&this->workQueue.ListLock);
    InitializeListHead(&this->workQueue.ListHead);
}

_Use_decl_annotations_
IMX_PEP::~IMX_PEP ()
{
    PAGED_CODE();
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    // This destructed object should not be accessible
    NT_ASSERT(pepGlobalContextPtr == nullptr);

    // Disable device interface
    if (this->deviceInterfaceName.Buffer != nullptr) {
        NTSTATUS status = IoSetDeviceInterfaceState(
                &this->deviceInterfaceName,
                FALSE);

        UNREFERENCED_PARAMETER(status);
        NT_ASSERT(
            NT_SUCCESS(status) || (status == STATUS_OBJECT_NAME_NOT_FOUND));

        RtlFreeUnicodeString(&this->deviceInterfaceName);
        this->deviceInterfaceName = UNICODE_STRING();
    }

    if (this->analogRegistersPtr != nullptr) {
        MmUnmapIoSpace(
            const_cast<MX8_CCM_ANALOG_REGISTERS*>(this->analogRegistersPtr),
            sizeof(*this->analogRegistersPtr));

        this->analogRegistersPtr = nullptr;
    }
}

_Use_decl_annotations_
NTSTATUS IMX_PEP::AddDevice (
    DRIVER_OBJECT* DriverObjectPtr,
    DEVICE_OBJECT* PhysicalDeviceObjectPtr
    )
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    DEVICE_OBJECT* fdoPtr;
    NTSTATUS status;

    IMX_PEP* thisPtr = pepGlobalContextPtr;

    status = IoCreateDevice(
            DriverObjectPtr,
            sizeof(_DEVICE_EXTENSION),
            nullptr,                // DeviceName
            FILE_DEVICE_UNKNOWN,
            FILE_DEVICE_SECURE_OPEN | FILE_AUTOGENERATED_DEVICE_NAME,
            FALSE,                  // Exclusive
            &fdoPtr);

    if (!NT_SUCCESS(status)) {
        IMX_LOG_ERROR(
            "IoCreateDevice() failed. (status = %!STATUS!)",
            status);
        return status;
    }

    // Set device extension to IMX_PEP class
    auto DeviceExtensionPtr = static_cast<_DEVICE_EXTENSION*>(
                                    fdoPtr->DeviceExtension);
    DeviceExtensionPtr->ImxPepPtr = thisPtr;

    // Set Device Objects
    NT_ASSERT(thisPtr->fdoPtr == nullptr);
    thisPtr->fdoPtr = fdoPtr;
    thisPtr->pdoPtr = PhysicalDeviceObjectPtr;

    // Set device object flags
    thisPtr->fdoPtr->Flags &= ~DO_POWER_PAGABLE;
    thisPtr->fdoPtr->Flags |= DO_BUFFERED_IO;
    auto deleteDevice = IMX_FINALLY::DoUnless([&] {
        PAGED_CODE();
        IoDeleteDevice(thisPtr->fdoPtr);
    });

    // Register device interface
    status = IoRegisterDeviceInterface(
            PhysicalDeviceObjectPtr,
            &GUID_DEVINTERFACE_IMXPEP,
            nullptr,    // ReferenceString
            &thisPtr->deviceInterfaceName);

    if (!NT_SUCCESS(status)) {
        IMX_LOG_ERROR(
            "IoRegisterDeviceInterface() failed. (status = %!STATUS!, PhysicalDeviceObjectPtr = 0x%p)",
            status,
            PhysicalDeviceObjectPtr);

        return status;
    }

    thisPtr->lowerDeviceObjectPtr = IoAttachDeviceToDeviceStack(
            thisPtr->fdoPtr,
            PhysicalDeviceObjectPtr);

    deleteDevice.DoNot();
    thisPtr->fdoPtr->Flags &= ~DO_DEVICE_INITIALIZING;

    IMX_LOG_INFORMATION(
        "PEP device added. (PhysicalDeviceObjectPtr = 0x%p, fdoPtr = 0x%p)",
        PhysicalDeviceObjectPtr,
        thisPtr->fdoPtr);

    return STATUS_SUCCESS;
}

_Use_decl_annotations_
VOID IMXPepDriverUnload (DRIVER_OBJECT* DriverObjectPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();

    //
    // If we haven't cleaned up prior to driver unload, clean up now
    //
    if (IMX_PEP::pepGlobalContextPtr != nullptr) {
        IMX_PEP* thisPtr = IMX_PEP::pepGlobalContextPtr;
        IMX_PEP::pepGlobalContextPtr = nullptr;
        thisPtr->~IMX_PEP();
        ExFreePoolWithTag(thisPtr, IMXPEP_POOL_TAG);
    }

    WPP_CLEANUP(DriverObjectPtr);
}

_Use_decl_annotations_
NTSTATUS InitializePepDevice ()
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();

    NTSTATUS status;
    UINT32 cpuRev;

    //
    // The PEP could be loaded on SOC families other than IMX
    // because it is included in UpdateOS, and UpdateOS is shared
    // between IMX. Only proceed with initialization
    // if we're on IMX8.
    //
    status = ImxGetCpuRev(&cpuRev);
    if (!NT_SUCCESS(status)) {
        IMX_LOG_ERROR("Failed to get CPU rev/type.");
        return status;
    }

    if (IMX_SOC_TYPE(cpuRev) != IMX_SOC_MX8M && IMX_SOC_TYPE(cpuRev) != IMX_SOC_MX8X && IMX_SOC_TYPE(cpuRev) != IMX_SOC_MX9) {
        IMX_LOG_ERROR(
            "Skipping initialization of PEP on non-IIMX chip. "
            "(cpuRev = 0x%x)",
            cpuRev);

        return STATUS_NOT_SUPPORTED;
    }

    auto deviceContextPtr = static_cast<IMX_PEP*>(
        ExAllocatePoolWithTag(
            NonPagedPoolNx,
            sizeof(IMX_PEP),
            IMXPEP_POOL_TAG
            )
    );

    if (deviceContextPtr == nullptr) {
        IMX_LOG_LOW_MEMORY("Failed to map memory for IMX class.");
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    RtlZeroMemory(deviceContextPtr, sizeof(*deviceContextPtr));
    deviceContextPtr->IMX_PEP::IMX_PEP(cpuRev);

    status = deviceContextPtr->IMX_PEP::InitializeResources();
    if (!NT_SUCCESS(status)) {
        IMX_LOG_ERROR(
            "InitializeResources() failed. (status = %!STATUS!)",
            status);
        deviceContextPtr->IMX_PEP::~IMX_PEP();
        ExFreePoolWithTag(deviceContextPtr, IMXPEP_POOL_TAG);
        return status;
    }

    return STATUS_SUCCESS;
}

IMX_PAGED_SEGMENT_END; //===================================================
IMX_INIT_SEGMENT_BEGIN; //==================================================

_Use_decl_annotations_
NTSTATUS DriverEntry (
    DRIVER_OBJECT* DriverObjectPtr,
    UNICODE_STRING* RegistryPathPtr
    )
{
    PAGED_CODE();

#if DBG
    KeQuerySystemTimePrecise(&DriverStartTime);
#endif

    // Initialize logging
    {
        WPP_INIT_TRACING(DriverObjectPtr, RegistryPathPtr);
        RECORDER_CONFIGURE_PARAMS recorderConfigureParams;
        RECORDER_CONFIGURE_PARAMS_INIT(&recorderConfigureParams);
        WppRecorderConfigure(&recorderConfigureParams);
#if DBG
        WPP_RECORDER_LEVEL_FILTER(IMX_TRACING_DEFAULT) = TRUE;
#endif // DBG
    }

    DriverObjectPtr->DriverExtension->AddDevice = IMX_PEP::AddDevice;
    DriverObjectPtr->DriverUnload = IMXPepDriverUnload;

    DriverObjectPtr->MajorFunction[IRP_MJ_PNP] = IMX_PEP::DispatchPnp;
    DriverObjectPtr->MajorFunction[IRP_MJ_POWER] = IMX_PEP::DispatchUnhandledIrp;
    DriverObjectPtr->MajorFunction[IRP_MJ_SYSTEM_CONTROL] = IMX_PEP::DispatchUnhandledIrp;
    DriverObjectPtr->MajorFunction[IRP_MJ_CREATE] = IMX_PEP::DispatchFileCreateClose;
    DriverObjectPtr->MajorFunction[IRP_MJ_CLOSE] = IMX_PEP::DispatchFileCreateClose;
    DriverObjectPtr->MajorFunction[IRP_MJ_DEVICE_CONTROL] = IMX_PEP::DispatchDeviceIoControl;

    // Initialize and Register Pep
    NTSTATUS status = InitializePepDevice();
    if (!NT_SUCCESS(status)) {
        WPP_CLEANUP(DriverObjectPtr);
        return status;
    }

    return STATUS_SUCCESS;
}

IMX_INIT_SEGMENT_END; //====================================================
