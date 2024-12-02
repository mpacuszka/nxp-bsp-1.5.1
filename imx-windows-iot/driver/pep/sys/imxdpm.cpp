// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022-2023 NXP
// Licensed under the MIT License.
//
//
// Module Name:
//
//   imxdpm.cpp
//
// Abstract:
//
//   IMX PEP Device Power Management Routines
//

#include "precomp.h"

#include "trace.h"
#include "imxdpm.tmh"

//#include "imxpowerdef.h"
#include "imxpeputil.h"
#include "imxpepioctl.h"
#include "imxpephw.h"
#include "imxpep.h"
#include "imxpep_dbg.h"

IMX_NONPAGED_SEGMENT_BEGIN; //==============================================

namespace { // static

    //
    // This table maps PNP Device IDs to IMX_PEP::_DEVICE_ID's.
    // The index in the table corresponds to the _DEVICE_ID
    //
    const UNICODE_STRING DeviceIdMap[] = {
        RTL_CONSTANT_STRING(L"\\_SB.CPU0"),
        RTL_CONSTANT_STRING(L"\\_SB.CPU1"),
        RTL_CONSTANT_STRING(L"\\_SB.CPU2"),
        RTL_CONSTANT_STRING(L"\\_SB.CPU3"),
        RTL_CONSTANT_STRING(L"\\_SB.PEP0"),
        RTL_CONSTANT_STRING(L"\\_SB.I2C1"),
        RTL_CONSTANT_STRING(L"\\_SB.I2C2"),
        RTL_CONSTANT_STRING(L"\\_SB.I2C3"),
        RTL_CONSTANT_STRING(L"\\_SB.SDH1"),      // uSDHC1 - eMMC
        RTL_CONSTANT_STRING(L"\\_SB.SDH2"),      // uSDHC2 - uSD
    };
} // namespace "static"

//
// Clocks and powers a device so it can be accessed by the driver
//
_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmPrepareDevice (PEP_PREPARE_DEVICE* ArgsPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    _DEVICE_ID pepDeviceId = this->pepDeviceIdFromPnpDeviceId(ArgsPtr->DeviceId);

    DBG_DRV_METHOD_BEG_WITH_PARAMS("DpmPrepareDevice: DeviceId=0x%02x DeviceName=%s AcpiName=%ls)",
        pepDeviceId, Dbg_GetDeviceName((ULONG)pepDeviceId), ArgsPtr->DeviceId->Buffer);

    if (pepDeviceId == _DEVICE_ID::_INVALID) {
        ArgsPtr->DeviceAccepted = FALSE;
        return TRUE;
    }

    ArgsPtr->DeviceAccepted = TRUE;
    DBG_DRV_METHOD_BEG_WITH_PARAMS("DpmPrepareDevice: ACCEPTED DeviceName=%s)", Dbg_GetDeviceName((ULONG)pepDeviceId));

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmAbandonDevice (PEP_ABANDON_DEVICE* ArgsPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    _DEVICE_ID pepDeviceId = this->pepDeviceIdFromPnpDeviceId(ArgsPtr->DeviceId);
    if (pepDeviceId == _DEVICE_ID::_INVALID) {
        ArgsPtr->DeviceAccepted = FALSE;
        return TRUE;
    }

    IMX_LOG_TRACE(
        "Accepting AbandonDevice request and powering off device. (DeviceId = %wZ)",
        ArgsPtr->DeviceId);

    this->setDeviceDx(pepDeviceId, PowerDeviceMaximum);

    ArgsPtr->DeviceAccepted = TRUE;
    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmRegisterDevice (PEP_REGISTER_DEVICE_V2* ArgsPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    _DEVICE_ID pepDeviceId = this->pepDeviceIdFromPnpDeviceId(ArgsPtr->DeviceId);
    DBG_DRV_METHOD_BEG_WITH_PARAMS("DpmRegisterDevice: DeviceId=0x%02x DeviceName=%s AcpiName=%ls)",
        pepDeviceId, Dbg_GetDeviceName((ULONG)pepDeviceId), ArgsPtr->DeviceId->Buffer);

    if (pepDeviceId == _DEVICE_ID::_INVALID) {
        ArgsPtr->DeviceAccepted = PepDeviceNotAccepted;
        return TRUE;
    }

    IMX_LOG_TRACE(
        "Accepting RegisterDevice request. (DeviceId = %wZ, pepDeviceId = %d)",
        ArgsPtr->DeviceId,
        int(pepDeviceId));

    static_assert(
        sizeof(ArgsPtr->DeviceHandle) >= sizeof(pepDeviceId),
        "Verifying _DEVICE_ID can be stored in PEPHANDLE");

    _DEVICE_CONTEXT* contextPtr = this->contextFromDeviceId(pepDeviceId);
    contextPtr->KernelHandle = ArgsPtr->KernelHandle;

    // Use the device ID as the handle value
    ArgsPtr->DeviceHandle = pepHandleFromDeviceId(pepDeviceId);
    ArgsPtr->DeviceAccepted = PepDeviceAccepted;

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmUnregisterDevice (PEP_UNREGISTER_DEVICE* ArgsPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    _DEVICE_ID deviceId = pepDeviceIdFromPepHandle(ArgsPtr->DeviceHandle);
    IMX_LOG_TRACE(
        "Unregistering device. (DeviceId = %d)",
        int(deviceId));

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmDevicePowerState (PEP_DEVICE_POWER_STATE* ArgsPtr)
{
    IMX_ASSERT_MAX_IRQL(DISPATCH_LEVEL);
    UNREFERENCED_PARAMETER(ArgsPtr);

#ifdef DBG_MESSAGE_PRINTING
    _DEVICE_ID deviceId = pepDeviceIdFromPepHandle(ArgsPtr->DeviceHandle);
    DBG_DRV_METHOD_BEG_WITH_PARAMS("DpmDevicePowerState: DeviceId=0x%02x (%s) PowerState=%u Complete=%s", 
        deviceId, Dbg_GetDeviceName((ULONG)deviceId), ArgsPtr->PowerState, ArgsPtr->Complete ? "TRUE" : "FALSE");
#endif

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmRegisterDebugger (PEP_REGISTER_DEBUGGER* ArgsPtr)
{
    IMX_ASSERT_MAX_IRQL(DISPATCH_LEVEL);

    _DEVICE_ID deviceId = pepDeviceIdFromPepHandle(ArgsPtr->DeviceHandle);
    IMX_LOG_TRACE("Registering debugger. (deviceId = %d)", int(deviceId));

    return TRUE;
}


_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmDeviceIdleContraints (
    PEP_DEVICE_PLATFORM_CONSTRAINTS* ArgsPtr
    )
{
    UNREFERENCED_PARAMETER(ArgsPtr);
    NT_ASSERT(ArgsPtr->PlatformStateCount == PLATFORM_IDLE_STATE_COUNT);

    const struct {
        DEVICE_POWER_STATE States[PLATFORM_IDLE_STATE_COUNT];
    } dependencies[] = {
        // WAIT, STOP_LIGHT, ARM_OFF
        {{ PowerDeviceD0, PowerDeviceD0, PowerDeviceD0 }},              // CPU0
        {{ PowerDeviceD0, PowerDeviceD0, PowerDeviceD0 }},              // CPU1
        {{ PowerDeviceD0, PowerDeviceD0, PowerDeviceD0 }},              // CPU2
        {{ PowerDeviceD0, PowerDeviceD0, PowerDeviceD0 }},              // CPU3
        {{ PowerDeviceUnspecified }},                                   // PEP0
        {{ PowerDeviceUnspecified }},                                   // I2C1
        {{ PowerDeviceUnspecified }},                                   // I2C2
        {{ PowerDeviceUnspecified }},                                   // I2C3
        {{ PowerDeviceUnspecified }},                                   // USDHC1
        {{ PowerDeviceUnspecified }},                                   // USDHC2
    };

    static_assert(
        ARRAYSIZE(dependencies) == int(_DEVICE_ID::_COUNT),
        "Verifying size of dependencies array");

    _DEVICE_ID deviceId = pepDeviceIdFromPepHandle(ArgsPtr->DeviceHandle);

    NT_ASSERT(int(deviceId) < ARRAYSIZE(dependencies));
    if (dependencies[int(deviceId)].States[0] == PowerDeviceUnspecified) {
        return FALSE;
    }

    for (ULONG i = 0; i < ArgsPtr->PlatformStateCount; ++i) {
        ArgsPtr->MinimumDStates[i] = dependencies[int(deviceId)].States[i];
    }

    return TRUE;
}

//
// The PEP writes a ULONG array to this buffer that specifies the
// lowest-powered Fx state that the component can be in for each platform idle
// state. An element with a value of 0 indicates F0, a value of 1 indicates F1,
// and so on. If the platform supports M idle states, array elements 0 to M-1
// specify the Fx states corresponding to platform idle states 0 to M-1.
//
_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmComponentIdleConstraints (
    PEP_COMPONENT_PLATFORM_CONSTRAINTS* ArgsPtr
    )
{
    UNREFERENCED_PARAMETER(ArgsPtr);
    NT_ASSERT(ArgsPtr->PlatformStateCount == PLATFORM_IDLE_STATE_COUNT);

    /*_DEVICE_ID deviceId = pepDeviceIdFromPepHandle(ArgsPtr->DeviceHandle);
    switch (deviceId) {
    case _DEVICE_ID::GPU:
    {
        switch (ArgsPtr->Component) {
        case IMX_PWRCOMPONENT_DISPLAY_IPU:

            //
            // Platform can enter WAIT when IPU is in F0
            //
            ArgsPtr->MinimumFStates[PLATFORM_IDLE_STATE_WAIT] = 0;

            //
            // IPU must be in F1 or below before system can enter STOP
            // because IPU clocks get turned off in STOP mode.
            //
            ArgsPtr->MinimumFStates[PLATFORM_IDLE_STATE_STOP_LIGHT] = 1;
            ArgsPtr->MinimumFStates[PLATFORM_IDLE_STATE_ARM_OFF] = 1;

            return TRUE;

        default:
            return FALSE;
        }
    } // _DEVICE_ID::GPU

    case _DEVICE_ID::USDHC1:
    case _DEVICE_ID::USDHC2:
    case _DEVICE_ID::USDHC3:
    case _DEVICE_ID::USDHC4:
    {
        switch (ArgsPtr->Component) {
        case 0:

            //
            // Platform can enter WAIT when SDHC is in F0
            //
            ArgsPtr->MinimumFStates[PLATFORM_IDLE_STATE_WAIT] = 0;

            //
            // SDHC must be in F1 or below before system can enter STOP
            // because PLL2 is stopped in STOP mode.
            // USDHCN_CLK_ROOT -> PLL2_PFD0 -> PLL2_MAIN_CLK
            //
            ArgsPtr->MinimumFStates[PLATFORM_IDLE_STATE_STOP_LIGHT] = 1;
            ArgsPtr->MinimumFStates[PLATFORM_IDLE_STATE_ARM_OFF] = 1;

            return TRUE;

        default:
            NT_ASSERT(!"Unexpected component!");
            return FALSE;
        }
    } // _DEVICE_ID::SDHC

    default:*/
        return FALSE;
    //} // switch (deviceId)
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmComponentIdleState (PEP_NOTIFY_COMPONENT_IDLE_STATE* ArgsPtr)
{
    if (ArgsPtr->IdleState == 0) {
        // F0 - put component into active state
        if (ArgsPtr->DriverNotified != FALSE) {
            // handle pre-notification only
            return FALSE;
        }

        // set component to f0
        return this->setComponentF0(ArgsPtr);
    } else {
        // Fx - put component in low power state
        if (ArgsPtr->DriverNotified == FALSE) {
            // handle post-notification only
            return FALSE;
        }

        // set component to fx state
        return this->setComponentFx(ArgsPtr);
    }
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmRegisterCrashdumpDevice (PEP_REGISTER_CRASHDUMP_DEVICE* /*ArgsPtr*/)
{
    /*_DEVICE_ID deviceId = pepDeviceIdFromPepHandle(ArgsPtr->DeviceHandle);

    switch (deviceId) {
    case _DEVICE_ID::USDHC1:
    case _DEVICE_ID::USDHC2:
    case _DEVICE_ID::USDHC3:
    case _DEVICE_ID::USDHC4:
        break;

    default:*/
        return FALSE;
    //}

    //ArgsPtr->PowerOnDumpDeviceCallback = IMX_PEP::PowerOnCrashdumpDevice;
    //return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmWork (PEP_WORK* ArgsPtr)
{
    LIST_ENTRY* entryPtr = ExInterlockedRemoveHeadList(
            &this->workQueue.ListHead,
            &this->workQueue.ListLock);

    //
    // A work item should have been queued if PEP_DPM_WORK was sent
    //
    NT_ASSERT(entryPtr != nullptr);

    _WORK_ITEM* workItemPtr = CONTAINING_RECORD(
            entryPtr,
            _WORK_ITEM,
            ListEntry);

    workItemPtr->WorkRoutine(workItemPtr->ContextPtr, ArgsPtr);

    ExFreeToLookasideListEx(&this->workQueue.LookasideList, workItemPtr);

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmLowPowerEpoch(void* DataPtr)
{
#ifdef DBG_MESSAGE_PRINTING
    typedef struct _PEP_LOW_POWER_EPOCH_PRIVATE {
        BOOLEAN LowPowerEpoch;
    } PEP_LOW_POWER_EPOCH_PRIVATE, * PPEP_LOW_POWER_EPOCH_PRIVATE;

    PEP_LOW_POWER_EPOCH_PRIVATE* epoch =  static_cast<PEP_LOW_POWER_EPOCH_PRIVATE*>(DataPtr);

    DBG_DRV_METHOD_BEG_WITH_PARAMS("LowPowerEpoch: %s", epoch->LowPowerEpoch ? "TRUE" : "FALSE");
#else
    UNREFERENCED_PARAMETER(DataPtr);
#endif
    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::DpmSystemLatencyUpdate(PEP_SYSTEM_LATENCY* ArgsPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    UNREFERENCED_PARAMETER(ArgsPtr);
    DBG_DRV_METHOD_BEG_WITH_PARAMS("DpmSystemLatencyUpdate Latency=%llu", ArgsPtr->Latency);

    return TRUE;
}


//
// Convert PNP Device Instance ID to PEP ID. If the device ID is not found,
// returns _DEVICE_ID::_INVALID
//
IMX_PEP::_DEVICE_ID IMX_PEP::pepDeviceIdFromPnpDeviceId (
    const UNICODE_STRING* DeviceIdPtr
    )
{
    const int deviceIdCount = static_cast<int>(_DEVICE_ID::_COUNT);

    static_assert(
        ARRAYSIZE(DeviceIdMap) == deviceIdCount,
        "Verifying DeviceIdMap matches up with _DEVICE_ID");

    for (int i = 0; i < deviceIdCount; ++i) {
        if (RtlEqualUnicodeString(
                DeviceIdPtr,
                &DeviceIdMap[i],
                TRUE)) {            // CaseInSensitive

            return static_cast<_DEVICE_ID>(i);
        }
    }

    return _DEVICE_ID::_INVALID;
}

//
// Put a device in the D0 (working) state. Multiple devices can be powered
// up in parallel.
//
_Use_decl_annotations_
void IMX_PEP::setDeviceD0 (_DEVICE_ID DeviceId)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    IMX_LOG_TRACE(
        "Setting device to D0. (DeviceId = %d)",
        int(DeviceId));

    /*_DEVICE_CONTEXT* contextPtr = this->contextFromDeviceId(DeviceId);
    switch (DeviceId) {
    case _DEVICE_ID::I2C1:
        this->setClockGate(IMX_I2C1_SERIAL_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::I2C2:
        this->setClockGate(IMX_I2C2_SERIAL_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::I2C3:
        this->setClockGate(IMX_I2C3_SERIAL_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::SPI1:
        this->setClockGate(IMX_ECSPI1_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::SPI2:
        this->setClockGate(IMX_ECSPI2_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::SPI3:
        this->setClockGate(IMX_ECSPI3_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::SPI4:
        this->setClockGate(IMX_ECSPI4_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::SPI5:
        this->setClockGate(IMX_ECSPI5_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::UART1:
    case _DEVICE_ID::UART2:
    case _DEVICE_ID::UART3:
    case _DEVICE_ID::UART4:
    case _DEVICE_ID::UART5:
        this->setUartD0(contextPtr);
        return;
    case _DEVICE_ID::VPU:
    {
        // Reference the power domain if we're coming from D3 or below.
        // Must not add a duplicate power domain reference if we're
        // already in D0-D2.
        if ((contextPtr->PowerState >= PowerDeviceD3) ||
            (contextPtr->PowerState == PowerDeviceUnspecified)) {

            this->referenceGpuVpuPowerDomain();
        }

        this->setClockGate(IMX_VPU_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    }
    case _DEVICE_ID::SSI1:
        this->setClockGate(IMX_SSI1_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::SSI2:
        this->setClockGate(IMX_SSI2_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::SSI3:
        this->setClockGate(IMX_SSI3_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::ASRC:
        this->setClockGate(IMX_ASRC_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    case _DEVICE_ID::ENET:
        this->setClockGate(IMX_ENET_CLK_ENABLE, IMX_CCM_CCGR_ON);
        break;
    default:
        IMX_LOG_TRACE(
            "Taking default behavior for device. (DeviceId = %d)",
            int(DeviceId));

        break;
    }

    contextPtr->PowerState = PowerDeviceD0;*/
}

//
// Move device to the specified low-power (Dx) state
//
_Use_decl_annotations_
void IMX_PEP::setDeviceDx (_DEVICE_ID DeviceId, DEVICE_POWER_STATE NewPowerState)
{
    _DEVICE_CONTEXT* contextPtr = this->contextFromDeviceId(DeviceId);
    DEVICE_POWER_STATE currentPowerState = contextPtr->PowerState;

    IMX_LOG_TRACE(
        "Setting device to low power state. (DeviceId = %d, currentPowerState = %d, NewPowerState = %d)",
        int(DeviceId),
        currentPowerState,
        NewPowerState);

    /*NT_ASSERT(currentPowerState == PowerDeviceD0);
    NT_ASSERT(NewPowerState > PowerDeviceD0);

    switch (DeviceId) {
    case _DEVICE_ID::I2C1:
        this->setClockGate(IMX_I2C1_SERIAL_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::I2C2:
        this->setClockGate(IMX_I2C2_SERIAL_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::I2C3:
        this->setClockGate(IMX_I2C3_SERIAL_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::SPI1:
        this->setClockGate(IMX_ECSPI1_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::SPI2:
        this->setClockGate(IMX_ECSPI2_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::SPI3:
        this->setClockGate(IMX_ECSPI3_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::SPI4:
        this->setClockGate(IMX_ECSPI4_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::SPI5:
        this->setClockGate(IMX_ECSPI5_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::UART1:
    case _DEVICE_ID::UART2:
    case _DEVICE_ID::UART3:
    case _DEVICE_ID::UART4:
    case _DEVICE_ID::UART5:
    {
        this->unreferenceUartClocks();
        break;
    }
    case _DEVICE_ID::VPU:
    {
        this->setClockGate(IMX_VPU_CLK_ENABLE, IMX_CCM_CCGR_OFF);

        // Unreference the GPU/VPU power domain if we're going to D3 or below
        if (NewPowerState >= PowerDeviceD3) {
            this->unreferenceGpuVpuPowerDomain();
        }

        break;
    }
    case _DEVICE_ID::SSI1:
        this->setClockGate(IMX_SSI1_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::SSI2:
        this->setClockGate(IMX_SSI2_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::SSI3:
        this->setClockGate(IMX_SSI3_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::ASRC:
        this->setClockGate(IMX_ASRC_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    case _DEVICE_ID::ENET:
        this->setClockGate(IMX_ENET_CLK_ENABLE, IMX_CCM_CCGR_OFF);
        break;
    default:
        IMX_LOG_TRACE(
            "Taking default behavior for device. (DeviceId = %d)",
            int(DeviceId));

        break;
    }

    contextPtr->PowerState = NewPowerState;*/
}

_Use_decl_annotations_
void IMX_PEP::setClockGate (IMX_CLK_GATE ClockGate, IMX_CCM_CCGR State)
{
    UNREFERENCED_PARAMETER(State);
    UNREFERENCED_PARAMETER(ClockGate);

    /*IMX_CCGR_INDEX index = ImxCcgrIndexFromClkGate(ClockGate);
    const ULONG bitOffset = 2 * index.GateNumber;

    IMX_SPINLOCK_GUARD lock(&this->ccgrRegistersSpinLock);

    ULONG ccgrValue = this->ccgrRegistersShadow[index.RegisterIndex];

    auto currentState = static_cast<IMX_CCM_CCGR>(
            (ccgrValue >> bitOffset) & 0x3);

    if (currentState == State) {
        return;
    }

    ccgrValue = (ccgrValue & ~(0x3 << bitOffset)) | (State << bitOffset);

    IMX_LOG_TRACE(
        "Setting clock gate. (ClockGate = 0x%x, State = %d)",
        ClockGate,
        State);

    this->ccgrRegistersShadow[index.RegisterIndex] = ccgrValue;
    WRITE_REGISTER_NOFENCE_ULONG(
        &registersPtr->CCGR[index.RegisterIndex],
        ccgrValue);*/
}

IMX_CCM_CCGR IMX_PEP::getClockGate (IMX_CLK_GATE ClockGate)
{
    UNREFERENCED_PARAMETER(ClockGate);
    /*volatile IMX_CCM_REGISTERS* registersPtr = this->ccmRegistersPtr;

    const IMX_CCGR_INDEX index = ImxCcgrIndexFromClkGate(ClockGate);

    const ULONG ccgrValue = READ_REGISTER_NOFENCE_ULONG(
            &registersPtr->CCGR[index.RegisterIndex]);

    const ULONG bitOffset = 2 * index.GateNumber;
    return static_cast<IMX_CCM_CCGR>((ccgrValue >> bitOffset) & 0x3);*/

    return IMX_CCM_CCGR_OFF;
}

_Use_decl_annotations_
void IMX_PEP::setGpuF0WorkRoutine (PVOID /*ContextPtr*/, PEP_WORK* /*PepWork*/)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    /*auto thisPtr = static_cast<IMX_PEP*>(ContextPtr);

    thisPtr->setGpuF0();

    PepWork->NeedWork = TRUE;
    PepWork->WorkInformation->WorkType = PepWorkCompleteIdleState;
    PepWork->WorkInformation->CompleteIdleState.DeviceHandle =
            thisPtr->contextFromDeviceId(_DEVICE_ID::GPU)->KernelHandle;

    PepWork->WorkInformation->CompleteIdleState.Component =
            IMX_PWRCOMPONENT_DISPLAY_3DENGINE;*/
}


_Use_decl_annotations_
BOOLEAN IMX_PEP::setComponentF0 (PEP_NOTIFY_COMPONENT_IDLE_STATE* ArgsPtr)
{
    //_DEVICE_ID deviceId = pepDeviceIdFromPepHandle(ArgsPtr->DeviceHandle);
    UNREFERENCED_PARAMETER(ArgsPtr);
    NT_ASSERT(ArgsPtr->IdleState == 0);

    /*switch (deviceId) {
    case _DEVICE_ID::GPU:
        switch (ArgsPtr->Component) {
        case IMX_PWRCOMPONENT_DISPLAY_3DENGINE:
            this->queueWorkItem(setGpuF0WorkRoutine, this);
            ArgsPtr->Completed = FALSE;
            return TRUE;

        case IMX_PWRCOMPONENT_DISPLAY_IPU:
        case IMX_PWRCOMPONENT_DISPLAY_MONITOR:
        default:
            return FALSE;
        }

    case _DEVICE_ID::USDHC1:
    case _DEVICE_ID::USDHC2:
    case _DEVICE_ID::USDHC3:
    case _DEVICE_ID::USDHC4:
    {
        this->setSdhcClockGate(deviceId, true);
        ArgsPtr->Completed = TRUE;
        return TRUE;
    } // USDHC

    default:*/
        return FALSE;
    //} // switch (deviceId)
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::setComponentFx (PEP_NOTIFY_COMPONENT_IDLE_STATE* ArgsPtr)
{
    //_DEVICE_ID deviceId = pepDeviceIdFromPepHandle(ArgsPtr->DeviceHandle);
    UNREFERENCED_PARAMETER(ArgsPtr);
    NT_ASSERT(ArgsPtr->IdleState > 0);

    /*switch (deviceId) {
    case _DEVICE_ID::GPU:
        switch (ArgsPtr->Component) {
        case IMX_PWRCOMPONENT_DISPLAY_3DENGINE:
            this->setGpuFx(ArgsPtr->IdleState);
            ArgsPtr->Completed = TRUE;
            return TRUE;

        case IMX_PWRCOMPONENT_DISPLAY_IPU:
        case IMX_PWRCOMPONENT_DISPLAY_MONITOR:
        default:
            return FALSE;
        }

    case _DEVICE_ID::USDHC1:
    case _DEVICE_ID::USDHC2:
    case _DEVICE_ID::USDHC3:
    case _DEVICE_ID::USDHC4:
    {
        this->setSdhcClockGate(deviceId, false);
        ArgsPtr->Completed = TRUE;
        return TRUE;
    } // USDHC

    default:*/
        return FALSE;
    //} // switch (deviceId)
}

/*void IMX_PEP::setSdhcClockGate(_DEVICE_ID DeviceId, bool On)
{
    IMX_CLK_GATE gate;
    switch (DeviceId) {
    case _DEVICE_ID::USDHC1:
        gate = IMX_USDHC1_CLK_ENABLE;
        break;
    case _DEVICE_ID::USDHC2:
        gate = IMX_USDHC2_CLK_ENABLE;
        break;
    case _DEVICE_ID::USDHC3:
        gate = IMX_USDHC3_CLK_ENABLE;
        break;
    case _DEVICE_ID::USDHC4:
        gate = IMX_USDHC4_CLK_ENABLE;
        break;
    default:
        NT_ASSERT(!"Invalid DeviceId");
        return;
    }

    this->setClockGate(
        gate,
        On ? IMX_CCM_CCGR_ON : IMX_CCM_CCGR_OFF);
}*/

_Use_decl_annotations_
void IMX_PEP::queueWorkItem (_PWORKITEM_ROUTINE WorkRoutine, PVOID ContextPtr)
{
    auto workItemPtr = static_cast<_WORK_ITEM*>(
            ExAllocateFromLookasideListEx(&this->workQueue.LookasideList));

    //
    // Failure of ExAllocateFromLookasideListEx() is catastrophic. Let the
    // system AV if the allocation failed.
    //

    workItemPtr->WorkRoutine = WorkRoutine;
    workItemPtr->ContextPtr = ContextPtr;

    ExInterlockedInsertTailList(
        &this->workQueue.ListHead,
        &workItemPtr->ListEntry,
        &this->workQueue.ListLock);

    //
    // The OS will deliver PEP_DPM_WORK when a worker thread is available
    //
    this->pepKernelInfo.RequestWorker(this->pepKernelInfo.Plugin);
}

IMX_NONPAGED_SEGMENT_END; //================================================
