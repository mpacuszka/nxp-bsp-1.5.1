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

#include <ntddk.h>
#include <wdf.h>

EXTERN_C_START

DRIVER_INITIALIZE                                    DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD                            EvtDriverDeviceAdd;
EVT_WDF_OBJECT_CONTEXT_CLEANUP                       EvtDriverUnload;


// Client Driver interfaces: General WDF interfaces.
GPIO_CLIENT_PREPARE_CONTROLLER                       PrepareController;
GPIO_CLIENT_RELEASE_CONTROLLER                       ReleaseController;
GPIO_CLIENT_START_CONTROLLER                         StartController;
GPIO_CLIENT_STOP_CONTROLLER                          StopController;
// Client Driver interfaces: Controller information interface.
GPIO_CLIENT_QUERY_CONTROLLER_BASIC_INFORMATION       QueryControllerBasicInformation;
GPIO_CLIENT_QUERY_SET_CONTROLLER_INFORMATION         QuerySetControllerInformation;
// Client Driver interfaces: Interrupt-related interfaces.
GPIO_CLIENT_ENABLE_INTERRUPT                         EnableInterrupt;
GPIO_CLIENT_DISABLE_INTERRUPT                        DisableInterrupt;
GPIO_CLIENT_UNMASK_INTERRUPT                         UnmaskInterrupt;
GPIO_CLIENT_MASK_INTERRUPTS                          MaskInterrupts;
GPIO_CLIENT_QUERY_ACTIVE_INTERRUPTS                  QueryActiveInterrupts;
GPIO_CLIENT_CLEAR_ACTIVE_INTERRUPTS                  ClearActiveInterrupts;
// Client Driver interfaces: IO-related interfaces.
GPIO_CLIENT_CONNECT_IO_PINS                          ConnectIoPins;
GPIO_CLIENT_DISCONNECT_IO_PINS                       DisconnectIoPins;
GPIO_CLIENT_READ_PINS_MASK                           ReadGpioPinsUsingMask;
GPIO_CLIENT_WRITE_PINS_MASK                          WriteGpioPinsUsingMask;
// Client Driver interfaces: Bank power management.
GPIO_CLIENT_SAVE_BANK_HARDWARE_CONTEXT               SaveBankHardwareContext;
GPIO_CLIENT_RESTORE_BANK_HARDWARE_CONTEXT            RestoreBankHardwareContext;
// Client Driver interfaces: Miscellaneous.          
GPIO_CLIENT_PRE_PROCESS_CONTROLLER_INTERRUPT         PreProcessControllerInterrupt;
GPIO_CLIENT_CONTROLLER_SPECIFIC_FUNCTION             ControllerSpecificFunction;
// Client Driver interfaces: Reconfigure interrupt.
GPIO_CLIENT_RECONFIGURE_INTERRUPT                    ReconfigureInterrupt;
// Client Driver Version 2 interfaces: Interrupt-related interfaces.
GPIO_CLIENT_QUERY_ENABLED_INTERRUPTS                 QueryEnabledInterrupts;
// Client Driver Version 3 interfaces: Function config related interfaces.
GPIO_CLIENT_CONNECT_FUNCTION_CONFIG_PINS             ConnectFunctionConfigPins;
GPIO_CLIENT_DISCONNECT_FUNCTION_CONFIG_PINS          DisconnectFunctionConfigPins;

EXTERN_C_END

//
// We have some non-paged functions that supposed to be called on low IRQL.
// The following macro defines unified assert to be put at the beginning of such functions.
//
// NOTE: We can't use standard PAGED_CODE macro as it requires function to be placed in paged segment during compilation.
//
#define IMX_ASSERT_MAX_IRQL(Irql) NT_ASSERT(KeGetCurrentIrql() <= (Irql))
#define IMX_ASSERT_LOW_IRQL()     IMX_ASSERT_MAX_IRQL(APC_LEVEL)