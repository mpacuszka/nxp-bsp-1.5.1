// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022-2023 NXP
// Licensed under the MIT License.
//
//
// Module Name:
//
//   imxppm.cpp
//
// Abstract:
//
//   IMX PEP Processor Power Management Routines
//

#include "precomp.h"

#include "trace.h"
#include "imxppm.tmh"

#include "imxpeputil.h"
#include "imxpepioctl.h"
#include "imxpephw.h"
#include "imxpep.h"
#include "imxpep_dbg.h"

#include "ArmSmcLib.h"

IMX_NONPAGED_SEGMENT_BEGIN; //==============================================

extern "C" BOOLEAN enumerateUnmaskedInterruptsCallback (
    VOID* CallbackContextPtr,
    PEP_UNMASKED_INTERRUPT_INFORMATION* InterruptInformationPtr
    )
{
    const ULONG gsiv = InterruptInformationPtr->Gsiv;
#ifdef DBG_MESSAGE_PRINTING
    ULONG cpuId = KeGetCurrentProcessorNumber(); //cpuId number 0,1,2,3
    DBG_DRV_METHOD_BEG_WITH_PARAMS("enumerateUnmaskedInterruptsCallback: Gsiv= %u CPU=%u", gsiv, cpuId);
#endif

    if (gsiv < 32) {            // lowest IRQ number
        return TRUE;
    } else if (gsiv > 159) {    // highest IRQ number
        return FALSE;
    }

    auto unmaskedInterruptsPtr =
        static_cast<IMX_PEP::UNMASKED_INTERRUPTS*>(CallbackContextPtr);

    unmaskedInterruptsPtr->Mask[(gsiv - 32) / 32] |= (1 << (gsiv % 32));
    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmQueryCapabilities (
    PEPHANDLE Handle,
    PEP_PPM_QUERY_CAPABILITIES* ArgsPtr
    )
{
    // DBG
    {
        _DEVICE_ID deviceId = pepDeviceIdFromPepHandle(Handle);
        UNREFERENCED_PARAMETER(deviceId);
        NT_ASSERT(
            (deviceId == _DEVICE_ID::CPU0) ||
            (deviceId == _DEVICE_ID::CPU1) ||
            (deviceId == _DEVICE_ID::CPU2) ||
            (deviceId == _DEVICE_ID::CPU3));
    }

    ArgsPtr->FeedbackCounterCount = 0;
    ArgsPtr->IdleStateCount = CPU_IDLE_STATE_COUNT;
    ArgsPtr->PerformanceStatesSupported = FALSE;
    ArgsPtr->ParkingSupported = FALSE;

    ++this->activeProcessorCount;

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmQueryIdleStatesV2 (
    PEPHANDLE /*Handle*/,
    PEP_PPM_QUERY_IDLE_STATES_V2* ArgsPtr
    )
{
    NT_ASSERT(ArgsPtr->Count == CPU_IDLE_STATE_COUNT);

    //
    // WFI State
    //
    {
        PEP_PROCESSOR_IDLE_STATE_V2* statePtr =
                &ArgsPtr->IdleStates[CPU_IDLE_STATE_WFI];

        statePtr->Interruptible = TRUE;
        statePtr->CacheCoherent = TRUE;
        statePtr->ThreadContextRetained = TRUE;
        statePtr->CStateType = 0;
        statePtr->WakesSpuriously = TRUE;
        statePtr->PlatformOnly = FALSE;
        statePtr->Autonomous = FALSE;
        statePtr->Latency = 0;
        statePtr->BreakEvenDuration = 0;
    }

    //
    // Second processor idle state also representing WFI, to be used as
    // a dependency of coordinated idle states
    //
    {
        PEP_PROCESSOR_IDLE_STATE_V2* statePtr =
                &ArgsPtr->IdleStates[CPU_IDLE_STATE_WFI2];

        statePtr->Interruptible = TRUE;
        statePtr->CacheCoherent = TRUE;
        statePtr->ThreadContextRetained = TRUE;
        statePtr->CStateType = 0;
        statePtr->WakesSpuriously = TRUE;
        statePtr->PlatformOnly = FALSE;
        statePtr->Autonomous = FALSE;
        statePtr->Latency = 0;
        statePtr->BreakEvenDuration = 0;
    }

    //
    // Processor idle state representing power gated state in which all context
    // is lost.
    //
    {
        PEP_PROCESSOR_IDLE_STATE_V2* statePtr =
                &ArgsPtr->IdleStates[CPU_IDLE_STATE_POWER_GATED];

        statePtr->Interruptible = TRUE;
        statePtr->CacheCoherent = FALSE;
        statePtr->ThreadContextRetained = FALSE;
        statePtr->CStateType = 0;

        //
        // Pending interrupt when WFI is executed could cause execution to
        // continue instead of entering STOP mode
        //
        statePtr->WakesSpuriously = TRUE;
        //statePtr->PlatformOnly = TRUE;
        statePtr->PlatformOnly = FALSE;
        statePtr->Autonomous = FALSE;
        statePtr->Latency = 0;
        statePtr->BreakEvenDuration = 0;
    }

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmQueryProcessorStateName(
    PEPHANDLE /*Handle*/,
    PEP_PPM_QUERY_STATE_NAME* ArgsPtr
)
{
    static const PCWSTR processorStateNames[CPU_IDLE_STATE_COUNT] = {
        L"PROCESSOR_STATE_WFI",                                 // The processor state WFI
        L"PROCESSOR_STATE_WFI2",                                // The processor state WFI2
        L"PROCESSOR_STATE_POWER_GATED",                         // The processor state POWER_GATED
    };

    static_assert(
        ARRAYSIZE(processorStateNames) == CPU_IDLE_STATE_COUNT,
        "Verifying size of processorStateNames array");

    NT_ASSERT(
        (ArgsPtr->StateIndex < CPU_IDLE_STATE_COUNT));

    //
    // First call is for the length of the string, second call is for
    // the actual string.
    //
    if (ArgsPtr->Name == nullptr) {
        ArgsPtr->NameSize =
            static_cast<USHORT>(wcslen(processorStateNames[ArgsPtr->StateIndex]) + 1);

    }
    else {
        NTSTATUS status = RtlStringCchCopyW(
            ArgsPtr->Name,
            ArgsPtr->NameSize,
            processorStateNames[ArgsPtr->StateIndex]);

        UNREFERENCED_PARAMETER(status);
        NT_ASSERT(NT_SUCCESS(status));
    }

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmQueryPlatformStates (
    PEPHANDLE /*Handle*/,
    PEP_PPM_QUERY_PLATFORM_STATES* ArgsPtr
    )
{
    ArgsPtr->PlatformStateCount = PLATFORM_IDLE_STATE_COUNT;
    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmQueryCoordinatedStates (
    PEPHANDLE /*Handle*/,
    PEP_PPM_QUERY_COORDINATED_STATES* ArgsPtr
    )
{
    NT_ASSERT(ArgsPtr->Count == PLATFORM_IDLE_STATE_COUNT);

    //
    // WAIT mode
    //
    {
        PEP_COORDINATED_IDLE_STATE* statePtr =
                &ArgsPtr->States[PLATFORM_IDLE_STATE_WAIT];

        statePtr->Latency = 0;
        statePtr->BreakEvenDuration = 0;
        statePtr->DependencyCount = this->activeProcessorCount;
        statePtr->MaximumDependencySize = 1;
    }

    //
    // STOP (light) state
    //
    {
        PEP_COORDINATED_IDLE_STATE* statePtr =
                &ArgsPtr->States[PLATFORM_IDLE_STATE_STOP_LIGHT];

        statePtr->Latency = 5000;             // 1 ms
        statePtr->BreakEvenDuration = 5000;   // 1 ms
        statePtr->DependencyCount = this->activeProcessorCount;
        statePtr->MaximumDependencySize = 1;
    }

    //
    // STOP (deep) state - all clocks on SOC turned off, ARM domain power
    // removed, standby voltage applied, DDR in self refresh
    //
    {
        PEP_COORDINATED_IDLE_STATE* statePtr =
                &ArgsPtr->States[PLATFORM_IDLE_STATE_ARM_OFF];

        statePtr->Latency = 5000;              // 0.5 ms
        statePtr->BreakEvenDuration = 5000;
        statePtr->DependencyCount = this->activeProcessorCount;
        statePtr->MaximumDependencySize = 1;
    }

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmQueryCoordinatedDependency (
    PEPHANDLE /*Handle*/,
    PEP_PPM_QUERY_COORDINATED_DEPENDENCY* ArgsPtr
    )
{
    DBG_DRV_METHOD_BEG_WITH_PARAMS("PpmQueryCoordinatedDependency:StateIndex=%d DependencyIndex=%d", ArgsPtr->StateIndex, ArgsPtr->DependencyIndex);

    switch (ArgsPtr->StateIndex) {
    case PLATFORM_IDLE_STATE_WAIT:
    case PLATFORM_IDLE_STATE_STOP_LIGHT:
    {
        switch (ArgsPtr->DependencyIndex) {
        case 0:     // CPU0
        case 1:     // CPU1
        case 2:     // CPU2
        case 3:     // CPU3
        {
            //
            // Should match MaximumDependencySize above
            //
            NT_ASSERT(ArgsPtr->DependencySize == 1);

            const _DEVICE_ID deviceId =
                    deviceIdFromDependencyIndex(ArgsPtr->DependencyIndex);

            const _DEVICE_CONTEXT* deviceContextPtr =
                    this->contextFromDeviceId(deviceId);

            ArgsPtr->DependencySizeUsed = 1;
            ArgsPtr->TargetProcessor = deviceContextPtr->KernelHandle;
            ArgsPtr->Options[0].ExpectedStateIndex = CPU_IDLE_STATE_WFI2;

            //
            // must be TRUE since WakesSpuriously flag is set
            //
            ArgsPtr->Options[0].LooseDependency = TRUE;

            //
            // OS may simultaneously enter the state referred to by this dependency
            //
            ArgsPtr->Options[0].InitiatingState = TRUE;

            //
            // Dependent state is a valid state for target processor to be in
            //
            ArgsPtr->Options[0].DependentState = TRUE;

            break;
        }

        default:
            NT_ASSERT(!"Invalid dependency index");
            return FALSE;
        }
        break;
    }  // PLATFORM_IDLE_STATE_WAIT, PLATFORM_IDLE_STATE_STOP_LIGHT

    case PLATFORM_IDLE_STATE_ARM_OFF:
    {
        switch (ArgsPtr->DependencyIndex) {
        case 0:     // CPU0
        case 1:     // CPU1
        case 2:     // CPU2
        case 3:     // CPU3
        {
            //
            // Should match MaximumDependencySize above
            //
            NT_ASSERT(ArgsPtr->DependencySize == 1);

            const _DEVICE_ID deviceId =
                    deviceIdFromDependencyIndex(ArgsPtr->DependencyIndex);

            const _DEVICE_CONTEXT* deviceContextPtr =
                    this->contextFromDeviceId(deviceId);

            ArgsPtr->DependencySizeUsed = 1;
            ArgsPtr->TargetProcessor = deviceContextPtr->KernelHandle;
            ArgsPtr->Options[0].ExpectedStateIndex = CPU_IDLE_STATE_POWER_GATED;

            //
            // OS must guarantee the processor remains in the expected idle
            // state for the duration of the platform idle transition.
            // Must be TRUE if WakesSpuriously is set.
            //
            ArgsPtr->Options[0].LooseDependency = TRUE;

            //
            // OS may simultaneously enter the state referred to by this dependency
            //
            ArgsPtr->Options[0].InitiatingState = TRUE;

            //
            // Dependent state is a valid state for target processor to be in
            //
            ArgsPtr->Options[0].DependentState = TRUE;

            break;
        }

        default:
            NT_ASSERT(!"Invalid dependency index");
            return FALSE;
        }
        break;
    }  // PLATFORM_IDLE_STATE_ARM_OFF

    default:
        NT_ASSERT(!"Invalid coordinated idle state index");
        return FALSE;
    } // switch (ArgsPtr->StateIndex)

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmIdleExecute (
    PEPHANDLE Handle,
    PEP_PPM_IDLE_EXECUTE_V2* ArgsPtr
    )
{
    NT_ASSERT(ArgsPtr->Status == STATUS_SUCCESS);

    DBG_DRV_METHOD_BEG_WITH_PARAMS("PpmIdleExecute: 0x%02x Platform=%s Processor=0x%02x %s", ArgsPtr->PlatformState, 
        Dbg_GetPlatformStateName(ArgsPtr->PlatformState), ArgsPtr->ProcessorState, Dbg_GetProcessorStateName(ArgsPtr->ProcessorState));

    LARGE_INTEGER  stateEntryTime;
    KeQuerySystemTimePrecise(&stateEntryTime);

    BOOLEAN result = TRUE;

    switch (ArgsPtr->PlatformState) {
    case PEP_PLATFORM_IDLE_STATE_NONE:
    {
        //
        // This is a processor-only transition
        //
        switch (ArgsPtr->ProcessorState) {
        case CPU_IDLE_STATE_WFI:
        case CPU_IDLE_STATE_WFI2:
        case CPU_IDLE_STATE_POWER_GATED: {

            //PSCI_CPU_SUSPEND_POWER_STATE state = {};
            //state.StateId = 1;
            //state.StateType = PSCI_CPU_SUSPEND_POWER_STATE_TYPE_POWER_DOWN;

            // NTSTATUS POFXCALLBACKPROCESSORHALT(_In_ ULONG Flags, _Inout_opt_ PVOID Context, _In_ PPROCESSOR_HALT_ROUTINE Halt);
            // NTSTATUS PROCESSOR_HALT_ROUTINE(_Inout_opt_ PVOID Context);
            //ArgsPtr->Status = this->pepKernelInfo.ProcessorHalt(
            //    PROCESSOR_HALT_CONTEXT_RETAINED | PROCESSOR_HALT_CACHE_COHERENT,
            //    &state,
            //    IMX_PEP::smcCpuSuspend);

            _DataSynchronizationBarrier();
            __wfi();
            break;
        }
        default:
            NT_ASSERT(!"Invalid processor idle state");
            result = FALSE;
            break;
        } // switch (ArgsPtr->ProcessorState)
    }

    case PLATFORM_IDLE_STATE_WAIT:
    {
        this->executePlatformIdleWait(ArgsPtr);
        break;
    }

    case PLATFORM_IDLE_STATE_STOP_LIGHT:
    {
        this->executePlatformIdleStopLight(ArgsPtr);
        break;
    }

    case PLATFORM_IDLE_STATE_ARM_OFF:
    {
        this->executePlatformIdleArmOff(ArgsPtr);
        break;
    }

    default:
        NT_ASSERT(!"Invalid platform idle state");
        result = FALSE;
        break;
    } // ArgsPtr->PlatformState

    _DEVICE_ID deviceId = pepDeviceIdFromPepHandle(Handle);
    Stat.Count((ULONG)ArgsPtr->PlatformState, (ULONG)deviceId, (ULONG)ArgsPtr->ProcessorState, stateEntryTime.QuadPart);

    return result;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmIdleComplete (
    PEPHANDLE /*Handle*/,
    PEP_PPM_IDLE_COMPLETE_V2* ArgsPtr
    )
{
    UNREFERENCED_PARAMETER(ArgsPtr);
    DBG_DRV_METHOD_BEG_WITH_PARAMS("PpmIdleComplete: 0x%02x Platform=%s Processor=0x%02x %s", ArgsPtr->PlatformState,
        Dbg_GetPlatformStateName(ArgsPtr->PlatformState), ArgsPtr->ProcessorState, Dbg_GetProcessorStateName(ArgsPtr->ProcessorState));

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmQueryVetoReasons (
    PEPHANDLE /*Handle*/,
    PEP_PPM_QUERY_VETO_REASONS* ArgsPtr
    )
{
    ArgsPtr->VetoReasonCount = 0; // IMX_VETO_REASON_COUNT;
    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmQueryVetoReason (
    PEPHANDLE /*Handle*/,
    PEP_PPM_QUERY_VETO_REASON* ArgsPtr
    )
{
    static const PCWSTR vetoNames[] = {
        L"Debug break",                                 // IMX_VETO_REASON_DEBUGGER
        L"This state is intentionally disabled",        // IMX_VETO_REASON_DISABLED
    };

    static_assert(
        ARRAYSIZE(vetoNames) == IMX_VETO_REASON_COUNT,
        "Verifying size of vetoNames array");

    NT_ASSERT(
        (ArgsPtr->VetoReason < IMX_VETO_REASON_MAX) &&
        ArgsPtr->VetoReason <= ARRAYSIZE(vetoNames));

    //
    // First call is for the length of the string, second call is for
    // the actual string.
    //
    if (ArgsPtr->Name == nullptr) {
        ArgsPtr->NameSize =
            static_cast<USHORT>(wcslen(vetoNames[ArgsPtr->VetoReason - 1]) + 1);

    } else {
        NTSTATUS status = RtlStringCchCopyW(
                ArgsPtr->Name,
                ArgsPtr->NameSize,
                vetoNames[ArgsPtr->VetoReason - 1]);

        UNREFERENCED_PARAMETER(status);
        NT_ASSERT(NT_SUCCESS(status));
    }

    return TRUE;
}

_IRQL_requires_max_(PASSIVE_LEVEL)
BOOLEAN IMX_PEP::PpmEnumerateBootVetoes (PEPHANDLE /*Handle*/)
{
    /*NTSTATUS status;
    POHANDLE cpu0Handle =
            this->contextFromDeviceId(_DEVICE_ID::CPU0)->KernelHandle;

    //
    // Disable stop light mode due to always-on counter dependency
    //
    status = this->pepKernelInfo.PlatformIdleVeto(
            cpu0Handle,
            PLATFORM_IDLE_STATE_STOP_LIGHT,
            IMX_VETO_REASON_DISABLED,
            TRUE);

    UNREFERENCED_PARAMETER(status);
    NT_ASSERT(NT_SUCCESS(status));

    //
    // ARM off platform idle state disabled due to lack of PSCI support
    //
    status = this->pepKernelInfo.PlatformIdleVeto(
            cpu0Handle,
            PLATFORM_IDLE_STATE_ARM_OFF,
            IMX_VETO_REASON_DISABLED,
            TRUE);

    UNREFERENCED_PARAMETER(status);
    NT_ASSERT(NT_SUCCESS(status)); */

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::PpmTestIdleState (
    PEPHANDLE /*Handle*/,
    PEP_PPM_TEST_IDLE_STATE* /*ArgsPtr*/
    )
{
    return TRUE;
}

IMX_PEP::_DEVICE_ID IMX_PEP::deviceIdFromDependencyIndex (ULONG DependencyIndex)
{
    switch (DependencyIndex) {
    case 0: return _DEVICE_ID::CPU0;
    case 1: return _DEVICE_ID::CPU1;
    case 2: return _DEVICE_ID::CPU2;
    case 3: return _DEVICE_ID::CPU3;
    default:
        NT_ASSERT(FALSE);
        return _DEVICE_ID::_INVALID;
    }
}

_Use_decl_annotations_
void IMX_PEP::executePlatformIdleWait (PEP_PPM_IDLE_EXECUTE_V2* /*ArgsPtr*/)
{
    /*//
    // This is a coordinated idle transition to WAIT mode
    //
    UNREFERENCED_PARAMETER(ArgsPtr);
    NT_ASSERT(ArgsPtr->PlatformState == PLATFORM_IDLE_STATE_WAIT);

    //
    // Set required bits in CGPR
    //
    {
        IMX_CCM_CGPR_REG cgpr =
        { READ_REGISTER_NOFENCE_ULONG(&this->ccmRegistersPtr->CGPR) };

        cgpr.must_be_one = 1;
        cgpr.INT_MEM_CLK_LPM = 1;

        WRITE_REGISTER_NOFENCE_ULONG(
            &this->ccmRegistersPtr->CGPR,
            cgpr.AsUlong);
    }

    //
    // Set LPM=WAIT, VSTBY=0, SBYOS=0
    //
    IMX_CCM_CLPCR_REG clpcr =
        { READ_REGISTER_NOFENCE_ULONG(&this->ccmRegistersPtr->CLPCR) };

    clpcr.LPM = IMX_CCM_CLPCR_LPM_WAIT;
    clpcr.ARM_clk_dis_on_lpm = 1;
    clpcr.SBYOS = 0;
    clpcr.VSTBY = 0;

    this->writeClpcrWaitStop(clpcr.AsUlong);

    this->updateGpcInterruptController();

    //
    // Execute WFI, triggering entry to WAIT mode
    //
    _DataSynchronizationBarrier();
    __wfi();*/
}

_Use_decl_annotations_
void IMX_PEP::executePlatformIdleStopLight (PEP_PPM_IDLE_EXECUTE_V2* /*ArgsPtr*/)
{
    /*//
    // This is a coordinated idle transition to STOP mode
    //
    UNREFERENCED_PARAMETER(ArgsPtr);
    NT_ASSERT(ArgsPtr->PlatformState == PLATFORM_IDLE_STATE_STOP_LIGHT);

    //
    // Enable the debugger UART to generate an AWAKE interrupt
    //
    this->enableDebuggerWake();

    //
    // Set LPM=STOP, VSTBY=0, SBYOS=0
    //
    IMX_CCM_CLPCR_REG clpcr =
        { READ_REGISTER_NOFENCE_ULONG(&this->ccmRegistersPtr->CLPCR) };

    clpcr.LPM = IMX_CCM_CLPCR_LPM_STOP;
    clpcr.ARM_clk_dis_on_lpm = 1;
    clpcr.SBYOS = 0;
    clpcr.VSTBY = 0;

    this->writeClpcrWaitStop(clpcr.AsUlong);

    //
    // Set GPC_PGC_CPU_PDN to 0, so that power does not get removed from CPU
    //
    WRITE_REGISTER_NOFENCE_ULONG(&this->gpcRegistersPtr->PGC_CPU.CTRL, 0);

    //
    // Configure PMU_MISC0 for light sleep mode
    //
    WRITE_REGISTER_NOFENCE_ULONG(
        &this->analogRegistersPtr->MISC0_SET,
        IMX_PMU_MISC0_STOP_MODE_CONFIG);

    this->updateGpcInterruptController();

    //
    // Notify system that GPT is going offline
    //
    this->pepKernelInfo.TransitionCriticalResource(
        this->contextFromDeviceId(_DEVICE_ID::GPT)->KernelHandle,
        0,
        FALSE);
*/
    //
    // Execute WFI, triggering entry to STOP mode
    //
    _DataSynchronizationBarrier();
    __wfi();
    /*PSCI_CPU_SUSPEND_POWER_STATE state = {};
    state.StateId = 1;
    state.StateType = PSCI_CPU_SUSPEND_POWER_STATE_TYPE_POWER_DOWN;

    // NTSTATUS POFXCALLBACKPROCESSORHALT(_In_ ULONG Flags, _Inout_opt_ PVOID Context, _In_ PPROCESSOR_HALT_ROUTINE Halt);
    // NTSTATUS PROCESSOR_HALT_ROUTINE(_Inout_opt_ PVOID Context);
    NTSTATUS status = this->pepKernelInfo.ProcessorHalt(
        PROCESSOR_HALT_CONTEXT_RETAINED | PROCESSOR_HALT_CACHE_COHERENT,
        &state,
        IMX_PEP::smcCpuSuspend);*/
}

_Use_decl_annotations_
void IMX_PEP::executePlatformIdleArmOff (PEP_PPM_IDLE_EXECUTE_V2* ArgsPtr)
{
    //
    // This is a coordinated idle transition to STOP mode
    //
    UNREFERENCED_PARAMETER(ArgsPtr);
    NT_ASSERT(ArgsPtr->PlatformState == PLATFORM_IDLE_STATE_ARM_OFF);

    this->updateGpcInterruptController();

    //BOOLEAN usePsci = IMX_CPU_TYPE(cpuRevision) != IMX_CPU_MX8QXP;
    BOOLEAN usePsci = FALSE; // Needs more integration and testing with imx_atf subsystem before imx_atf can be used

    if (usePsci) {
        PSCI_CPU_SUSPEND_POWER_STATE state = {};
        state.StateId = 1;
        state.StateType = PSCI_CPU_SUSPEND_POWER_STATE_TYPE_POWER_DOWN;

        ArgsPtr->Status = this->pepKernelInfo.ProcessorHalt(
            PROCESSOR_HALT_CONTEXT_RETAINED | PROCESSOR_HALT_CACHE_COHERENT,
            &state,
            IMX_PEP::smcCpuSuspend);
    } else {
        _DataSynchronizationBarrier();
        __wfi();
    }
}

// NTSTATUS PROCESSOR_HALT_ROUTINE(_Inout_opt_ PVOID Context);
NTSTATUS IMX_PEP::smcCpuOff(_Inout_opt_ PVOID /*Context*/)
{
    ARM_SMC_ARGS ArmSmcArgs = { 0 };
    ArmSmcArgs.Arg0 = PSCI_CPU_OFF;
    ArmSmcArgs.Arg1 = 0;
    ArmSmcArgs.Arg2 = 0;

    ArmCallSmc(&ArmSmcArgs);
    return STATUS_SUCCESS;
}

// NTSTATUS PROCESSOR_HALT_ROUTINE(_Inout_opt_ PVOID Context);
NTSTATUS IMX_PEP::smcCpuSuspend(_Inout_opt_ PVOID /*Context*/)
{
    UINTN power_state = ((UINTN)IMX_PWR_LVL0) << PSTATE_PWR_LVL_SHIFT;

    ARM_SMC_ARGS ArmSmcArgs = { 0 };
    ArmSmcArgs.Arg0 = PSCI_CPU_SUSPEND_AARCH64;
    ArmSmcArgs.Arg1 = power_state;
    ArmSmcArgs.Arg2 = 0;
    ArmSmcArgs.Arg3 = 0;

    ArmCallSmc(&ArmSmcArgs);

    DBG_DRV_METHOD_BEG_WITH_PARAMS("smcCpuSuspend PSCI return value: 0x%016lx Arg1: 0x%016lx", ArmSmcArgs.Arg0, ArmSmcArgs.Arg1);
    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS IMX_PEP::updateGpcInterruptController ()
{
    UNMASKED_INTERRUPTS unmaskedInterrupts = {0};
    PEP_UNMASKED_INTERRUPT_INFORMATION info;
    info.Version = PEP_UNMASKED_INTERRUPT_INFORMATION_V1;
    info.Size = sizeof(info);
    NTSTATUS status = this->pepKernelInfo.EnumerateUnmaskedInterrupts(
            nullptr,
            PEP_ENUMERATE_UNMASKED_INTERRUPT_FLAGS_NONE,
            enumerateUnmaskedInterruptsCallback,
            &unmaskedInterrupts,
            &info);

    if (!NT_SUCCESS(status)) {

        //
        // This will result in a bugcheck
        //
        NT_ASSERT(FALSE);
        return status;
    }

    //
    // Program changed values into IMR registers
    //
    /*for (ULONG i = 0; i < ARRAYSIZE(unmaskedInterrupts.Mask); ++i) {
        if (unmaskedInterrupts.Mask[i] != this->unmaskedInterruptsCopy.Mask[i]) {
            this->unmaskedInterruptsCopy.Mask[i] = unmaskedInterrupts.Mask[i];
            WRITE_REGISTER_NOFENCE_ULONG(
                &this->gpcRegistersPtr->IMR[i],
                ~unmaskedInterrupts.Mask[i]);
        }
    }*/

    return STATUS_SUCCESS;
}

void IMX_PEP::maskGpcInterrupts (ULONG cpuId)
{
    volatile ULONG* IMR_FOR_CORE[4] = {
        this->gpcRegistersPtr->IMR_CORE0_A53,
        this->gpcRegistersPtr->IMR_CORE1_A53,
        this->gpcRegistersPtr->IMR_CORE2_A53,
        this->gpcRegistersPtr->IMR_CORE3_A53
    };

    if (cpuId > 3)
    {
        // Invalid CPU id
        return;
    }

    volatile ULONG* IMR = IMR_FOR_CORE[cpuId];

    for (ULONG i = 0; i < 4; ++i) {
        WRITE_REGISTER_NOFENCE_ULONG(
            &IMR[i],
            0xffffffff);
    }
}

void IMX_PEP::unmaskGpcInterrupts ()
{
/*    for (ULONG i = 0; i < ARRAYSIZE(this->gpcRegistersPtr->IMR); ++i) {
        WRITE_REGISTER_NOFENCE_ULONG(
            &this->gpcRegistersPtr->IMR[i],
            ~this->unmaskedInterruptsCopy.Mask[i]);
    }*/
}

IMX_NONPAGED_SEGMENT_END; //================================================

