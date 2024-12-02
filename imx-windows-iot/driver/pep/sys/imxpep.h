// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022-2023 NXP
// Licensed under the MIT License.
//
//
// Module Name:
//
//   imxpep.h
//
// Abstract:
//
//   IMX PEP Declarations
//

#ifndef _IMXPEP_H_
#define _IMXPEP_H_

#include "imxpepstat.h"

class IMX_PEP {
public: // NONPAGED

    //
    // Devices managed by the PEP
    //
    enum class _DEVICE_ID {
        CPU0,
        CPU1,
        CPU2,
        CPU3,
        PEP0,
/*        GPT,
        EPIT1,*/
        I2C1,
        I2C2,
        I2C3,
/*        SPI1,
        SPI2,
        SPI3,
        SPI4,
        SPI5,
        UART1,
        UART2,
        UART3,
        UART4,
        UART5,*/
        USDHC1,
        USDHC2,
/*        USDHC3,
        USDHC4,
        VPU,
        SSI1,
        SSI2,
        SSI3,
        ASRC,
        URS0,
        USB0,
        USB1,
        ENET,
        GPU,
        PCI0,
        GPIO,*/
        _COUNT,
        _INVALID = _COUNT,
    };

    enum class _COMPONENT_ID {
        GPU,
        _COUNT,
    };

    enum CPU_IDLE_STATE : ULONG {
        CPU_IDLE_STATE_WFI,
        CPU_IDLE_STATE_WFI2,
        CPU_IDLE_STATE_POWER_GATED,
        CPU_IDLE_STATE_COUNT,
    };

    enum PLATFORM_IDLE_STATE : ULONG {
        PLATFORM_IDLE_STATE_WAIT,
        PLATFORM_IDLE_STATE_STOP_LIGHT,
        PLATFORM_IDLE_STATE_ARM_OFF,
        PLATFORM_IDLE_STATE_COUNT,
    };

    enum IMX_VETO_REASON : ULONG {
        IMX_VETO_REASON_DEBUGGER = 1,
        IMX_VETO_REASON_DISABLED,
        IMX_VETO_REASON_MAX,
        IMX_VETO_REASON_COUNT = IMX_VETO_REASON_MAX - 1,
    };

    struct UNMASKED_INTERRUPTS {
        ULONG Mask[4];
    };

    static PEPCALLBACKNOTIFYACPI AcceptAcpiNotification;
    static PEPCALLBACKNOTIFYDPM AcceptDeviceNotification;
    static PEPCALLBACKNOTIFYPPM AcceptProcessorNotification;

    //
    // PEP ACPI Notification Callbacks
    //
    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN AcpiPrepareDevice(PEP_ACPI_PREPARE_DEVICE* ArgsPtr);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN AcpiAbandonDevice (PEP_ACPI_ABANDON_DEVICE* ArgsPtr);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN AcpiRegisterDevice (PEP_ACPI_REGISTER_DEVICE* ArgsPtr);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN AcpiEnumerateDeviceNamespace (
        PEP_ACPI_ENUMERATE_DEVICE_NAMESPACE* ArgsPtr
        );

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN AcpiQueryObjectInformation (
        PEP_ACPI_QUERY_OBJECT_INFORMATION* ArgsPtr
        );

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN AcpiEvaluateControlMethod (
        PEP_ACPI_EVALUATE_CONTROL_METHOD* ArgsPtr
        );

    //
    // PEP Device Power Management Callbacks
    //

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN DpmPrepareDevice (PEP_PREPARE_DEVICE* ArgsPtr);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN DpmAbandonDevice (PEP_ABANDON_DEVICE* ArgsPtr);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN DpmRegisterDevice (PEP_REGISTER_DEVICE_V2* ArgsPtr);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN DpmUnregisterDevice (PEP_UNREGISTER_DEVICE* ArgsPtr);

    _IRQL_requires_max_(DISPATCH_LEVEL)
    BOOLEAN DpmDevicePowerState (PEP_DEVICE_POWER_STATE* ArgsPtr);

    _IRQL_requires_max_(DISPATCH_LEVEL)
    BOOLEAN DpmRegisterDebugger (PEP_REGISTER_DEBUGGER* ArgsPtr);

    _IRQL_requires_max_(DISPATCH_LEVEL)
    BOOLEAN DpmDeviceIdleContraints (PEP_DEVICE_PLATFORM_CONSTRAINTS* ArgsPtr);

    _IRQL_requires_max_(DISPATCH_LEVEL)
    BOOLEAN DpmComponentIdleConstraints (PEP_COMPONENT_PLATFORM_CONSTRAINTS* ArgsPtr);

    _IRQL_requires_max_(DISPATCH_LEVEL)
    BOOLEAN DpmComponentIdleState (PEP_NOTIFY_COMPONENT_IDLE_STATE* ArgsPtr);

    _IRQL_requires_max_(HIGH_LEVEL)
    BOOLEAN DpmRegisterCrashdumpDevice (PEP_REGISTER_CRASHDUMP_DEVICE* ArgsPtr);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN DpmWork (PEP_WORK* ArgsPtr);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN DpmLowPowerEpoch(void* Data);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN DpmSystemLatencyUpdate(PEP_SYSTEM_LATENCY* ArgsPtr);

    //
    // PEP Processor Power Management Callbacks
    //

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN PpmQueryCapabilities (
        PEPHANDLE Handle,
        _Inout_ PEP_PPM_QUERY_CAPABILITIES* ArgsPtr
        );

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN PpmQueryIdleStatesV2 (
        PEPHANDLE Handle,
        _Inout_ PEP_PPM_QUERY_IDLE_STATES_V2* ArgsPtr
        );

    _IRQL_requires_max_(PASSIVE_LEVEL)
     BOOLEAN IMX_PEP::PpmQueryProcessorStateName(
         PEPHANDLE Handle,
         _Inout_ PEP_PPM_QUERY_STATE_NAME* ArgsPtr
     );

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN PpmQueryPlatformStates (
        PEPHANDLE Handle,
        _Inout_ PEP_PPM_QUERY_PLATFORM_STATES* ArgsPtr
        );

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN PpmQueryCoordinatedStates (
        PEPHANDLE Handle,
        _Inout_ PEP_PPM_QUERY_COORDINATED_STATES* ArgsPtr
        );

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN PpmQueryCoordinatedDependency (
        PEPHANDLE Handle,
        _Inout_ PEP_PPM_QUERY_COORDINATED_DEPENDENCY* ArgsPtr
        );

    _IRQL_requires_max_(HIGH_LEVEL)
    BOOLEAN PpmIdleExecute (
        PEPHANDLE Handle,
        _Inout_ PEP_PPM_IDLE_EXECUTE_V2* ArgsPtr
        );

    _IRQL_requires_max_(HIGH_LEVEL)
    BOOLEAN PpmIdleComplete (
        PEPHANDLE Handle,
        _Inout_ PEP_PPM_IDLE_COMPLETE_V2* ArgsPtr
        );

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN PpmQueryVetoReasons (
        PEPHANDLE Handle,
        _Inout_ PEP_PPM_QUERY_VETO_REASONS* ArgsPtr
        );

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN PpmQueryVetoReason (
        PEPHANDLE Handle,
        _Inout_ PEP_PPM_QUERY_VETO_REASON* ArgsPtr
        );

    _IRQL_requires_max_(PASSIVE_LEVEL)
    BOOLEAN PpmEnumerateBootVetoes (PEPHANDLE Handle);

    _IRQL_requires_max_(HIGH_LEVEL)
    BOOLEAN PpmTestIdleState (
        PEPHANDLE Handle,
        _Inout_ PEP_PPM_TEST_IDLE_STATE* ArgsPtr
        );

    //
    // Nonpaged WDM Dispatch Functions
    //

    _Dispatch_type_(IRP_MJ_POWER)
    _Dispatch_type_(IRP_MJ_SYSTEM_CONTROL)
    static DRIVER_DISPATCH DispatchUnhandledIrp;

    _Dispatch_type_(IRP_MJ_DEVICE_CONTROL)
    static DRIVER_DISPATCH DispatchDeviceIoControl;

private: // NONPAGED

    enum : ULONG { _POOL_TAG = 'PPXM' };

    struct _DEVICE_EXTENSION {
        IMX_PEP* ImxPepPtr;
    };

    // Per-device context information
    struct _DEVICE_CONTEXT {
        POHANDLE KernelHandle;
        DEVICE_POWER_STATE PowerState;
        BOOLEAN isDeviceReserved;
    };

    typedef
    _IRQL_requires_max_(PASSIVE_LEVEL)
    VOID _WORKITEM_ROUTINE (PVOID ContextPtr, PEP_WORK* PepWork);

    typedef _WORKITEM_ROUTINE *_PWORKITEM_ROUTINE;

    struct _WORK_ITEM {
        LIST_ENTRY ListEntry;
        _PWORKITEM_ROUTINE WorkRoutine;
        PVOID ContextPtr;
    };

    static _DEVICE_ID pepDeviceIdFromPnpDeviceId (
        const UNICODE_STRING* DeviceIdPtr
        );

    __forceinline static _DEVICE_ID pepDeviceIdFromPepHandle (PEPHANDLE PepHandle)
    {
        UINT64 deviceId = reinterpret_cast<UINT64>(PepHandle);
        NT_ASSERT(
            deviceId < UINT64(_DEVICE_ID::_COUNT));

        return static_cast<_DEVICE_ID>(deviceId);
    }

    __forceinline static PEPHANDLE pepHandleFromDeviceId (_DEVICE_ID DeviceId)
    {
        return reinterpret_cast<PEPHANDLE>(DeviceId);
    }

    __forceinline _DEVICE_CONTEXT* contextFromDeviceId (_DEVICE_ID DeviceId)
    {
        return &this->deviceData[int(DeviceId)];
    }

    _IRQL_requires_max_(PASSIVE_LEVEL)
    void setDeviceD0 (_DEVICE_ID DeviceId);

    _IRQL_requires_max_(DISPATCH_LEVEL)
    void setDeviceDx (_DEVICE_ID DeviceId, DEVICE_POWER_STATE NewPowerState);

    _IRQL_requires_max_(DISPATCH_LEVEL)
    void setClockGate (IMX_CLK_GATE ClockGate, IMX_CCM_CCGR State);

    IMX_CCM_CCGR getClockGate (IMX_CLK_GATE ClockGate);

    static _WORKITEM_ROUTINE setGpuF0WorkRoutine;

    _IRQL_requires_max_(DISPATCH_LEVEL)
    BOOLEAN setComponentF0 (PEP_NOTIFY_COMPONENT_IDLE_STATE* ArgsPtr);

    _IRQL_requires_max_(DISPATCH_LEVEL)
    BOOLEAN setComponentFx (PEP_NOTIFY_COMPONENT_IDLE_STATE* ArgsPtr);

    _IRQL_requires_max_(DISPATCH_LEVEL)
    void queueWorkItem (_PWORKITEM_ROUTINE WorkRoutine, PVOID ContextPtr);

    //
    // PPM Functions
    //

    static _DEVICE_ID deviceIdFromDependencyIndex (ULONG DependencyIndex);

    _IRQL_requires_max_(HIGH_LEVEL)
    void executePlatformIdleWait (_Inout_ PEP_PPM_IDLE_EXECUTE_V2* ArgsPtr);

    _IRQL_requires_max_(HIGH_LEVEL)
    void executePlatformIdleStopLight (_Inout_ PEP_PPM_IDLE_EXECUTE_V2* ArgsPtr);

    _IRQL_requires_max_(HIGH_LEVEL)
    void executePlatformIdleArmOff (_Inout_ PEP_PPM_IDLE_EXECUTE_V2* ArgsPtr);

    _IRQL_requires_max_(HIGH_LEVEL)
    NTSTATUS updateGpcInterruptController ();

    void maskGpcInterrupts (ULONG cpuId);

    void unmaskGpcInterrupts ();

    static NTSTATUS smcCpuOff(_Inout_opt_ PVOID /*Context*/);
    static NTSTATUS smcCpuSuspend(_Inout_opt_ PVOID /*Context*/);


    //
    // private members
    //
    UINT32 cpuRevision;
    volatile LONG activeProcessorCount;
    UNMASKED_INTERRUPTS unmaskedInterruptsCopy;

    volatile MX8_CCM_ANALOG_REGISTERS* analogRegistersPtr;
    volatile MX8_GPC_REGISTERS* gpcRegistersPtr;

    DEVICE_OBJECT* lowerDeviceObjectPtr;
    DEVICE_OBJECT* pdoPtr;
    DEVICE_OBJECT* fdoPtr;
    UNICODE_STRING deviceInterfaceName;
    PEP_KERNEL_INFORMATION pepKernelInfo;
    KSEMAPHORE ioRequestSemaphore;

    // Stores per-device state information. Use contextFromDeviceId() to access.
    _DEVICE_CONTEXT deviceData[unsigned(_DEVICE_ID::_COUNT)];

    struct {
        KSPIN_LOCK ListLock;
        LIST_ENTRY ListHead;
        LOOKASIDE_LIST_EX LookasideList;
    } workQueue;

public: // PAGED

    static IMX_PEP* pepGlobalContextPtr;

    static DRIVER_ADD_DEVICE AddDevice;

    //
    // PAGED WDM Dispatch Routines
    //
    _Dispatch_type_(IRP_MJ_PNP)
    static DRIVER_DISPATCH DispatchPnp;

    _Dispatch_type_(IRP_MJ_CREATE)
    _Dispatch_type_(IRP_MJ_CLOSE)
    static DRIVER_DISPATCH DispatchFileCreateClose;

    _IRQL_requires_max_(PASSIVE_LEVEL)
    IMX_PEP (UINT32 cpuRevision);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    ~IMX_PEP ();

    _IRQL_requires_max_(PASSIVE_LEVEL)
    NTSTATUS InitializeResources ();

    _IRQL_requires_max_(PASSIVE_LEVEL)
    NTSTATUS RegisterPlugin ();

private: // PAGED

    IMX_PEP_STAT Stat;

    _IRQL_requires_max_(PASSIVE_LEVEL)
    NTSTATUS startDevice (DEVICE_OBJECT* DeviceObjectPtr, IRP* IrpPtr);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    _Requires_lock_held_(this->ioRequestSemaphore)
    NTSTATUS ioctlDumpRegisters (DEVICE_OBJECT* DeviceObjectPtr, IRP* IrpPtr);

    _IRQL_requires_max_(PASSIVE_LEVEL)
    _Requires_lock_held_(this->ioRequestSemaphore)
    NTSTATUS ioctlStat(DEVICE_OBJECT* DeviceObjectPtr, IRP* IrpPtr);
};

// PAGED
extern "C" DRIVER_UNLOAD IMXPepDriverUnload;
extern "C" DRIVER_INITIALIZE DriverEntry;

#endif // _IMXPEP_H_
