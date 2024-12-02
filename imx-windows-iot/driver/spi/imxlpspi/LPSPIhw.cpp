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
 
//
// Module Name:
//
//    LPSPIhw.cpp
//
// Abstract:
//
//    This module contains methods for accessing the IMX LPSPI
//    controller hardware.
//    This controller driver uses the SPB WDF class extension (SpbCx).
//
// Environment:
//
//    kernel-mode only
//
#include "precomp.h"
#pragma hdrstop

#define _LPSPI_HW_CPP_

// Logging header files
#include "LPSPItrace.h"
#include "LPSPIhw.tmh"

// Common driver header files
#include "LPSPIcommon.h"

// Module specific header files
#include "LPSPIhw.h"
#include "LPSPIspb.h"
#include "LPSPIdriver.h"
#include "LPSPIdevice.h"

#if (defined(DBG) || defined(DEBUG))
//
//  Routine Description:
//
//    This routine dumps hardware registers.
//
//  Arguments:
//
//  DevExtPtr - The device extension.
//
//  Return Value:
//
//    None.
//
//
_Use_decl_annotations_
inline VOID LPSPIHwDumpRegisters(LPSPI_DEVICE_EXTENSION* DevExtPtr)
{
    if (DevExtPtr->ControllerHwVersion.Major < 2) {
        LPSPI_LOG_INFORMATION(
            DevExtPtr->IfrLogHandle,
            "M VERID=0x%08X PARAM=0x%08X "
            "CR=0x%08X SR=0x%08X IER=0x%08X DER=0x%08X "
            "CFGR0=0x%08X CFGR1=0x%08X DMR0=0x%08X DMR1=0x%08X "
            "CCR=0x%08XFCR=0x%08X FSR=0x%08X "
            "TCR=0x%08X RSR=0x%08X",
            DevExtPtr->LPSPIRegsPtr->VERID,
            DevExtPtr->LPSPIRegsPtr->PARAM,
            DevExtPtr->LPSPIRegsPtr->CR,
            DevExtPtr->LPSPIRegsPtr->SR,
            DevExtPtr->LPSPIRegsPtr->IER,
            DevExtPtr->LPSPIRegsPtr->DER,
            DevExtPtr->LPSPIRegsPtr->CFGR0,
            DevExtPtr->LPSPIRegsPtr->CFGR1,
            DevExtPtr->LPSPIRegsPtr->DMR0,
            DevExtPtr->LPSPIRegsPtr->DMR1,
            DevExtPtr->LPSPIRegsPtr->CCR,
            DevExtPtr->LPSPIRegsPtr->FCR,
            DevExtPtr->LPSPIRegsPtr->FSR,
            DevExtPtr->LPSPIRegsPtr->TCR,
            DevExtPtr->LPSPIRegsPtr->RSR
        );
    }
    else {
        LPSPI_LOG_INFORMATION(
            DevExtPtr->IfrLogHandle,
            "M VERID=0x%08X PARAM=0x%08X "
            "CR=0x%08X SR=0x%08X IER=0x%08X DER=0x%08X "
            "CFGR0=0x%08X CFGR1=0x%08X DMR0=0x%08X DMR1=0x%08X "
            "CCR=0x%08X CCR1=0x%08X FCR=0x%08X FSR=0x%08X "
            "TCR=0x%08X RSR=0x%08X RDROR=0x%08X",
            DevExtPtr->LPSPIRegsPtr->VERID,
            DevExtPtr->LPSPIRegsPtr->PARAM,
            DevExtPtr->LPSPIRegsPtr->CR,
            DevExtPtr->LPSPIRegsPtr->SR,
            DevExtPtr->LPSPIRegsPtr->IER,
            DevExtPtr->LPSPIRegsPtr->DER,
            DevExtPtr->LPSPIRegsPtr->CFGR0,
            DevExtPtr->LPSPIRegsPtr->CFGR1,
            DevExtPtr->LPSPIRegsPtr->DMR0,
            DevExtPtr->LPSPIRegsPtr->DMR1,
            DevExtPtr->LPSPIRegsPtr->CCR,
            DevExtPtr->LPSPIRegsPtr->CCR1,  // optional
            DevExtPtr->LPSPIRegsPtr->FCR,
            DevExtPtr->LPSPIRegsPtr->FSR,
            DevExtPtr->LPSPIRegsPtr->TCR,
            DevExtPtr->LPSPIRegsPtr->RSR,
            DevExtPtr->LPSPIRegsPtr->RDROR  // optional
        );
    }
}
#endif

//
// Routine Description:
//
//  LPSPIHwIsTxFifoFull returns TRUE if TX FIFO is full.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
// Return Value:
//
//  TRUE if TX FIFO is full, otherwise FALSE.
//
__forceinline
BOOLEAN
LPSPIHwIsTxFifoFull(
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr
)
{
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;

    LPSPI_FSR fifoStatReg = {
        READ_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->FSR)
    };
    return fifoStatReg.TXCOUNT >= DevExtPtr->TxFifoSize;
}

//
// Routine Description:
//
//  LPSPIHwQueryTxFifoSpace returns the current space in TX FIFO.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
// Return Value:
//
//  Current RX FIFO count.
//
__forceinline
ULONG
LPSPIHwQueryTxFifoSpace(
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr
)
{
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;

    LPSPI_FSR fifoStatReg = {
        READ_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->FSR)
    };
    return (ULONG)(DevExtPtr->TxFifoSize - fifoStatReg.TXCOUNT);
}

//
// Routine Description:
//
//  LPSPIHwInitController initializes the ARM LPSPI controller, and  
//  sets it to a known state.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object the represent the LPSPI this instance of
//      the LPSPI controller.
//
// Return Value:
//
//  Controller initialization status.
//
_Use_decl_annotations_
NTSTATUS
LPSPIHwInitController (
    LPSPI_DEVICE_EXTENSION* DevExtPtr
    )
{
    size_t txFifoSize, rxFifoSize;
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;

    //
    // Check controller compatibility
    //
    {
        LPSPI_VERID veridReg; 
        veridReg.AsUlong = READ_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->VERID);
        ULONG feature = veridReg.FEATURE;
        if (feature != LPSPI_FEATURE_32_BIT_SHIFT_REG) {
            LPSPI_LOG_ERROR(
                DevExtPtr->IfrLogHandle,
                "LPSPIHwInitController() Not supported size of shift register (FEATURE: %lld)",
                feature
            );

        }
        
        DevExtPtr->ControllerHwVersion.Major = veridReg.MAJOR;
        DevExtPtr->ControllerHwVersion.Minor = veridReg.MINOR;

        if (veridReg.MAJOR < 2U) {
            /* Full range for PRESCALE bit group */
            DevExtPtr->PrescaleMax = 7U;
        }
        else {
            /* Limitation for PRESCALE bit group on imx93. */
            DevExtPtr->PrescaleMax = 1U;

            if (DevExtPtr->SampleOnDelayedSckEdge) {
                /* If SAMPLE=1 then LPSPI receiver does not work on imx93. */
                DevExtPtr->SampleOnDelayedSckEdge = FALSE;
                LPSPI_LOG_WARNING(
                    DevExtPtr->IfrLogHandle,
                    "Not allowed to set SAMPLE bit = 1 on this LPSPI HW version. "
                    "It is related to SampleOnDelayedSckEdge parameter in ACPI. SAMPLE bit will be set to 0."
                );
            }

        }
    }

    //
    // Get Tx and Rx Fifo size parameters
    //
    {
        LPSPI_PARAM paramReg; 
        paramReg.AsUlong = READ_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->PARAM);

        // Get FIFO size from PARAM register
        txFifoSize = 1ULL << paramReg.TXFIFO;
        rxFifoSize = 1ULL << paramReg.RXFIFO;

        if (txFifoSize < 4) {
            LPSPI_LOG_ERROR(
                DevExtPtr->IfrLogHandle,
                "LPSPIHwInitController() Tx FIFO size %lld is too small",
                txFifoSize
            );

            return STATUS_NOT_SUPPORTED;
        }
        if (rxFifoSize < 4) {
            LPSPI_LOG_ERROR(
                DevExtPtr->IfrLogHandle,
                "LPSPIHwInitController() Rx FIFO size %lld is too small",
                rxFifoSize
            );

            return STATUS_NOT_SUPPORTED;
        }

        DevExtPtr->TxFifoSize = txFifoSize;
        DevExtPtr->RxFifoSize = rxFifoSize;
        // Prepare Tx watermark
        DevExtPtr->TxFifoWatermarkMax = (txFifoSize * 1 / 4);
        // Prepare Rx watermark
        DevExtPtr->RxFifoWatermarkMax = (rxFifoSize * 3 / 4) - 1;

    }

    //
    // Reset the block
    //
    LPSPI_CR ctrlReg = { 0 };

    ctrlReg.RST = 1;
    WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CR, ctrlReg.AsUlong);

    WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CR, 0);

    return STATUS_SUCCESS;
}


//
// Routine Description:
//
//  LPSPIHwSetTargetConfiguration calculates the controller HW configuration
//  according to the target required settings.
//  The routine save the control and configuration HW registers image on the 
//  target context so it can be applied when the target is referenced.
//
// Arguments:
//
//  TrgCtxPtr - The target context.
//
// Return Value:
//
//  STATUS_SUCCESS, or STATUS_NOT_SUPPORTED if the desired speed yields
//  an error that more than the allowed range.
//
_Use_decl_annotations_
NTSTATUS
LPSPIHwSetTargetConfiguration (
    LPSPI_TARGET_CONTEXT* TrgCtxPtr
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;
    LPSPI_TARGET_SETTINGS* trgSettingsPtr = &TrgCtxPtr->Settings;
    LPSPI_CHANNEL spiChannel = static_cast<LPSPI_CHANNEL>(
        trgSettingsPtr->DeviceSelection
        );

    //
    // Control register
    //
    {
        LPSPI_CR* ctrlRegPtr = &trgSettingsPtr->CtrlReg;
        ctrlRegPtr->AsUlong = 0;
    } // Control register
        
    //
    // Configuration 1 register
    //
    {
        LPSPI_CFGR1* config1RegPtr = &trgSettingsPtr->Config1Reg;
        config1RegPtr->AsUlong = 0;
        // Output data retains last value when chip select is negated
        config1RegPtr->OUTCFG = LPSPI_DATA_OUTPUT_CTL::DATA_LAST_VALUE;
        // Configure pins SDI as input and SDO as output
        config1RegPtr->PINCFG = LPSPI_PIN_CTL::SDI_AS_INPUT_SDO_AS_OUTPUT;
        // Disable Match function
        config1RegPtr->MATCFG = LPSPI_MATCH_CTL::MATCH_DISABLED;
        // Device polarity, configure the CS (SS) polarity.
        if (trgSettingsPtr->CsActiveValue == 1) {

            config1RegPtr->CSPOL = LPSPI_PCS_POL::PCS_ACTIVE_HIGH;
        }
        else {

            config1RegPtr->CSPOL = LPSPI_PCS_POL::PCS_ACTIVE_LOW;
        }
        // Enable Stall
        config1RegPtr->NOSTALL = LPSPI_STALL_CTL::STALL_YES;
        // Disable automatic CS generation
        config1RegPtr->AUTOCS = LPSPI_AUTO_CS_CTL::NO_AUTO_CS;
        // Set sample point on SCK edge or on delayed SCK edge
        if (!LPSPIDeviceIsSampleOnDelayedSckEdge(devExtPtr)) {
            config1RegPtr->SAMPLE = LPSPI_INPUT_DATA_SAMPLE_POINT_CTL::ON_SCK_EDGE;
        }
        else {
            config1RegPtr->SAMPLE = LPSPI_INPUT_DATA_SAMPLE_POINT_CTL::ON_DELAYED_SCK_EDGE;
        }
        // Master vs Slave mode
        config1RegPtr->MASTER = LPSPI_MASTER::MASTER;
    } // Configuration 1 register

    //
    // Calculate the SPI clock settings.
    //
    ULONG prescaler = 0;
    ULONG sckScaler = 0;
    NTSTATUS status = LPSPIpHwCalcFreqDivider(
        TrgCtxPtr->DevExtPtr,
        trgSettingsPtr->ConnectionSpeed, 
        &prescaler,
        &sckScaler
        );
    if (!NT_SUCCESS(status)) {

        return status;
    }

    //
    // Clock Configuration register
    //
    {
        LPSPI_CCR* clockConfigRegPtr = &trgSettingsPtr->ClockConfigReg;
        clockConfigRegPtr->AsUlong = 0;

        clockConfigRegPtr->SCKCS = sckScaler >> 1;
        clockConfigRegPtr->CSSCK = sckScaler >> 1;
        clockConfigRegPtr->DBT = sckScaler;
        clockConfigRegPtr->SCKDIV = sckScaler;
    } // Clock Configuration register

    //
    // Transmit Command register
    //
    {
        LPSPI_TCR* transmitCmdRegPtr = &trgSettingsPtr->TransmitCmdReg;
        transmitCmdRegPtr->AsUlong = 0;

        // SCLK polarity
        if (trgSettingsPtr->Polarity) {
            transmitCmdRegPtr->CPOL = LPSPI_SCLK_POL::SCLK_POL1;
        } else {
            transmitCmdRegPtr->CPOL = LPSPI_SCLK_POL::SCLK_POL0;
        }
        // SCLK phase
        if (trgSettingsPtr->Phase) {
            transmitCmdRegPtr->CPHA = LPSPI_SCLK_PHA::SCLK_PHASE1;

        }  else {
            transmitCmdRegPtr->CPHA = LPSPI_SCLK_PHA::SCLK_PHASE0;
        }
        // Set Prescale
        transmitCmdRegPtr->PRESCALE = prescaler;
        // Peripheral chip select
        transmitCmdRegPtr->CS = spiChannel;
        // LSB first
        transmitCmdRegPtr->LSBF = LPSPI_LSB_MSB_FIRST::MSB_FIRST;
        // No byte swap
        transmitCmdRegPtr->BYSW = LPSPI_BYTE_SWAP::BYTE_SWAP_NO;

    } // Transmit Command register

    return STATUS_SUCCESS;
}


//
// Routine Description:
//
//  LPSPIHwSelectTarget is from EvtSpbControllerLock callback
//  to select a given target.
//  The routine resets the block and apply the configuration 
//  saved on the target context.
//
// Arguments:
//
//  TrgCtxPtr - The context to target to select.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIHwSelectTarget (
    LPSPI_TARGET_CONTEXT* TrgCtxPtr
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;
    volatile LPSPI_REGISTERS* lpspiRegsPtr = devExtPtr->LPSPIRegsPtr;
    const LPSPI_TARGET_SETTINGS* trgSettingsPtr = &TrgCtxPtr->Settings;

    //
    // Reset the block
    //
    {
        LPSPI_CR ctrlReg = { trgSettingsPtr->CtrlReg.AsUlong };

        ctrlReg.RST = 1;
        WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CR, ctrlReg.AsUlong);

        ctrlReg = { trgSettingsPtr->CtrlReg.AsUlong };
        ctrlReg.RRF = 1;
        ctrlReg.RTF = 1;
        WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CR, ctrlReg.AsUlong);
    }

    //
    // Configuration register 0: RDMO = 0, CIRFIFO = 0
    //
    WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CFGR0, 0);

    //
    // Configuration register 1
    //
    {
        LPSPI_CFGR1 config1Reg = { trgSettingsPtr->Config1Reg.AsUlong };
        WRITE_REGISTER_NOFENCE_ULONG(
            &lpspiRegsPtr->CFGR1,
            config1Reg.AsUlong
        );
    }

    {
        LPSPI_TCR transmitCmdReg = { trgSettingsPtr->TransmitCmdReg.AsUlong };
        WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->TCR, transmitCmdReg.AsUlong);
    }

    //
    // Clock Configuration register
    //
    {
        LPSPI_CCR clockConfigReg = { trgSettingsPtr->ClockConfigReg.AsUlong };
        WRITE_REGISTER_NOFENCE_ULONG(
            &lpspiRegsPtr->CCR,
            clockConfigReg.AsUlong
        );
    }

    //
    // Enable the block
    //
    {
        LPSPI_CR ctrlReg = { trgSettingsPtr->CtrlReg.AsUlong };

        ctrlReg.MEN = 1;
        WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CR, ctrlReg.AsUlong);
    }

#if (defined(DBG) || defined(DEBUG))
    LPSPIHwDumpRegisters(devExtPtr);
#endif
}


//
// Routine Description:
//
//  LPSPIHwUnselectTarget is from LPSPIEvtSpbControllerUnlock callback
//  to un-select a given target.
//  
//
// Arguments:
//
//  TrgCtxPtr - The context to target to select.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIHwUnselectTarget (
    LPSPI_TARGET_CONTEXT* TrgCtxPtr
    )
{
    LPSPI_DEVICE_EXTENSION* devExtPtr = TrgCtxPtr->DevExtPtr;
    volatile LPSPI_REGISTERS* lpspiRegsPtr = devExtPtr->LPSPIRegsPtr;

    //
    // Disable and ACK all LPSPI interrupt
    //
    WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->IER, 0);
    WRITE_REGISTER_NOFENCE_ULONG(
        &lpspiRegsPtr->SR, 
        LPSPI_INTERRUPT_GROUP::LPSPI_ACKABLE_INTERRUPTS
        );

    //
    // Reset the block
    //
    {
        LPSPI_CR ctrlReg = { 0 };

        ctrlReg.RST = 1;
        WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CR, ctrlReg.AsUlong);

        ctrlReg = { 0 };
        ctrlReg.RRF = 1;
        ctrlReg.RTF = 1;
        WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CR, ctrlReg.AsUlong);

        WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CR, 0);
    }
}


//
// Routine Description:
//
//  LPSPIHwGetClockRange gets the min and max supported clock frequencies.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
//  MinClockPtr - Caller min clock address.
//
//  MaxClockPtr - Caller max clock address.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIHwGetClockRange (
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    ULONG* MinClockHzPtr,
    ULONG* MaxClockHzPtr
    )
{
    *MinClockHzPtr = LPSPIDeviceGetReferenceClock(DevExtPtr) / ((1 << DevExtPtr->PrescaleMax) * LPSPI_SCALER_MAX);
    *MaxClockHzPtr = LPSPIDeviceGetMaxSpeed(DevExtPtr);
}


//
// Routine Description:
//
//  LPSPIHwInterruptControl is called to enable/disable interrupts
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
//  IsEnable - TRUE enable interrupts, otherwise FALSE
//
//  InterruptMask -  A combination of LPSPI_INTERRUPT_TYPE values.
//
// Return Value:
//
//  The new interrupt mask
//
_Use_decl_annotations_
ULONG
LPSPIHwInterruptControl (
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    ULONG InterruptDisableMask,
    ULONG InterruptEnableMask
    )
{
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;

    LPSPI_ASSERT(
        DevExtPtr->IfrLogHandle,
        (InterruptDisableMask & ~LPSPI_INTERRUPT_TYPE::ALL) == 0x0
        );

    LPSPI_ASSERT(
        DevExtPtr->IfrLogHandle,
        (InterruptEnableMask & ~LPSPI_INTERRUPT_TYPE::ALL) == 0x0
    );

    //
    // Update interrupt control
    //
    LPSPI_IER intEnableReg;
    {
        intEnableReg.AsUlong = READ_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->IER);
        intEnableReg.AsUlong &= ~InterruptDisableMask;
        intEnableReg.AsUlong |= InterruptEnableMask;
        WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->IER, intEnableReg.AsUlong);

    } // // Update interrupt control

    return intEnableReg.AsUlong;
}

//
// Routine Description:
//
//  LPSPIHwConfigureTransfer is called to calculate the HW configuration
//  based on the given transfer information.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
//  TransferPtr - The transfer descriptor
//
//  IsInitialSetup - If this is the transfer initial setup.
//      When IsInitialSetup is TRUE the routine configure the HW based
//      on the transfer parameters.
//      When IsFirstTransfer is FALSE, the routine adds the transfer configuration to
//      the existing settings.
//
//  InterruptRegPtr - Address of the LPSPI_INTREG image
//      to receive the required interrupt control settings.
//      After all transfers have been configured, caller uses this value 
//      when calling LPSPIHwInterruptControl.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIHwConfigureTransfer (
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    LPSPI_SPB_TRANSFER* TransferPtr,
    BOOLEAN IsInitialSetup,
    PULONG InterruptRegPtr
    )
{
    LPSPI_SPB_REQUEST* requestPtr = TransferPtr->AssociatedRequestPtr;
    LPSPI_TARGET_SETTINGS* trgSettingsPtr = &requestPtr->SpbTargetPtr->Settings;
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;

    LPSPI_IER intEnableReg = { 0 };
    LPSPI_FCR fifoCtrlReg = { 0 };
    LPSPI_TCR transmitCmdReg = { trgSettingsPtr->TransmitCmdReg.AsUlong };
    if (!IsInitialSetup) {

        intEnableReg.AsUlong = *InterruptRegPtr;
        fifoCtrlReg.AsUlong = READ_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->FCR);
    }

    //
    // Set/update RX/TX thresholds
    //
    {
        if (LPSPISpbIsWriteTransfer(TransferPtr)) {
            //
            // Write configuration
            //
            size_t wordsLeftToTransfer = LPSPISpbWordsLeftToTransfer(TransferPtr);

            intEnableReg.AsUlong &= ~LPSPI_TX_INTERRUPTS;

            if (wordsLeftToTransfer > 0) {
                fifoCtrlReg.TXWATER = DevExtPtr->TxFifoWatermarkMax;
                intEnableReg.TDIE = 1; // TX threshold interrupt
            }
            else {
                fifoCtrlReg.TXWATER = 0;
            }
            if (requestPtr->Type != LPSPI_REQUEST_TYPE::FULL_DUPLEX) {
                intEnableReg.TCIE = 1; // Transfer complete interrupt
            }
        
        } else {
            //
            // Read configuration
            //
            size_t wordsLeftToTransfer = LPSPISpbWordsLeftInBurst(TransferPtr);

            LPSPI_ASSERT(
                DevExtPtr->IfrLogHandle,
                wordsLeftToTransfer > 0
                );

            intEnableReg.AsUlong &= ~LPSPI_RX_INTERRUPTS;
            if (wordsLeftToTransfer > 0) {
                if (wordsLeftToTransfer > DevExtPtr->TxFifoWatermarkMax) {
                    fifoCtrlReg.RXWATER = DevExtPtr->TxFifoWatermarkMax;
                }
                else {
                    fifoCtrlReg.RXWATER = wordsLeftToTransfer - 1;
                }
                intEnableReg.RDIE = 1; // RX threshold interrupt
            }
        }

    } // Set RX/TX thresholds

    WRITE_REGISTER_NOFENCE_ULONG(
        &lpspiRegsPtr->FCR,
        fifoCtrlReg.AsUlong
        );
    *InterruptRegPtr = intEnableReg.AsUlong;
}


//
// Routine Description:
//
//  LPSPIHwUpdateTransferConfiguration is called to update transfer 
//  RX/TX thresholds configuration due to transfer progress.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
//  TransferPtr - The transfer descriptor
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIHwUpdateTransferConfiguration (
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;
    LPSPI_IER intEnableReg = {
        READ_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->IER)
        };

    LPSPIHwConfigureTransfer(
        DevExtPtr,
        TransferPtr,
        FALSE, // Not initial setup
        &intEnableReg.AsUlong
        );

    WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->IER, intEnableReg.AsUlong);
}

//
// Routine Description:
//
//  LPSPIHwWriteCmdInTxFIFO is called to put Transmit Command into Tx FIFO.
//  Transmit Command value is based on prepared value during SpbTargetConnect event 
//  where Frame Size is updated accordig to a currently started burst (BurstLength).
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
//  TransferPtr - The transfer descriptor
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIHwWriteCmdInTxFIFO(
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    LPSPI_SPB_TRANSFER* TransferPtr
)
{
    LPSPI_SPB_REQUEST* requestPtr = TransferPtr->AssociatedRequestPtr;
    LPSPI_TARGET_SETTINGS* trgSettingsPtr = &requestPtr->SpbTargetPtr->Settings;
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;
    LPSPI_TCR transmitCmdReg = { trgSettingsPtr->TransmitCmdReg.AsUlong };
    BOOLEAN isFirstBurst = TransferPtr->CurrentBurst == 0;
    BOOLEAN isLastBurst = TransferPtr->CurrentBurst == (TransferPtr->NumberOfBursts - 1);

    transmitCmdReg.FRAMESZ = (TransferPtr->BurstLength * 8) - 1;
    if (LPSPISpbIsWriteTransfer(TransferPtr)) {
        if (requestPtr->Type != LPSPI_REQUEST_TYPE::FULL_DUPLEX) {
            transmitCmdReg.RXMSK = 1;
            if (!isFirstBurst || !isLastBurst ) {
                if (!isLastBurst) {
                    // CONT = 1 and CONTC = 0
                    transmitCmdReg.CONT = 1;
                }
                if (!isFirstBurst) {
                    // CONT = 0 and CONTC = 1
                    transmitCmdReg.CONTC = 1;
                }
            }
        }
    }
    else {
        transmitCmdReg.TXMSK = 1;
    }
    WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->TCR, transmitCmdReg.AsUlong);
}

//
// Routine Description:
//
//  LPSPIHwWriteTxFIFO is called to write outgoing data from MDL to
//  TX FIFO.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
//  TransferPtr - The transfer descriptor
//
//  IsInitialSetup - If this is the transfer initial setup.
//      When IsInitialSetup is TRUE and the transfer is done, the routine does not enable or 
//      disable Tx/Transfer complete interrupts.
//      When IsFirstTransfer is FALSE and the transfer is done, the routine enables or 
//      disables Tx/Transfer complete interrupts.
//
// Return Value:
//
//  TRUE: transfer is done, otherwise FALSE.
//
_Use_decl_annotations_
BOOLEAN
LPSPIHwWriteTxFIFO (
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    LPSPI_SPB_TRANSFER* TransferPtr,
    BOOLEAN IsInitialSetup
    )
{
    LPSPI_SPB_REQUEST* requestPtr = TransferPtr->AssociatedRequestPtr;
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;
    volatile ULONG* txDataRegPtr = &lpspiRegsPtr->TDR;

    ULONG maxWordsToWrite = LPSPIHwQueryTxFifoSpace(DevExtPtr);
    ULONG totalBytesRead = 0;
    while ((maxWordsToWrite != 0) &&
           !LPSPISpbIsAllDataTransferred(TransferPtr)) {

        --maxWordsToWrite;
        if (TransferPtr->IsStartBurst) {
            LPSPIHwWriteCmdInTxFIFO(DevExtPtr, TransferPtr);
            TransferPtr->IsStartBurst = FALSE;
            continue;
        }

        ULONG txFifoWord;
        ULONG bytesRead = LPSPIpHwReadWordFromMdl(TransferPtr, &txFifoWord);

        WRITE_REGISTER_NOFENCE_ULONG(txDataRegPtr, txFifoWord);
        --TransferPtr->BurstWords;

        LPSPIpHwUpdateTransfer(TransferPtr, bytesRead);

        totalBytesRead += bytesRead;
    }
    requestPtr->TotalBytesTransferred += totalBytesRead;

    if (LPSPISpbIsTransferDone(TransferPtr)) {

        if (requestPtr->Type != LPSPI_REQUEST_TYPE::FULL_DUPLEX) {
            // WRITE
            (void)LPSPIHwInterruptControl(
                DevExtPtr,
                LPSPI_TX_INTERRUPTS,   // Disable Tx interrupt
                LPSPI_INTERRUPT_TYPE::TRANSFER_COMPLETE     // And enable Transfer Complete interrupt
            );
        }
        else {
            // FULL_DUPLEX
            (void)LPSPIHwInterruptControl(
                DevExtPtr,
                LPSPI_TX_INTERRUPTS,   // Disable Tx interrupt
                0x0                    // No additional interrupt to enable (Transfer Complete interrupt is enabled in LPSPIHwReadRxFIFO)
            );
        }
        return TRUE;

    } else {

        if (!IsInitialSetup) {
            LPSPIHwUpdateTransferConfiguration(DevExtPtr, TransferPtr);
        }
        return FALSE;
    }
}


//
// Routine Description:
//
//  LPSPIHwReadRxFIFO is called to read incoming data from RX FIFO
//  to MDL.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
//  TransferPtr - The transfer descriptor
//
// Return Value:
//
//  TRUE: transfer is done, otherwise FALSE.
//
_Use_decl_annotations_
BOOLEAN
LPSPIHwReadRxFIFO (
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;
    volatile ULONG* rxDataRegPtr = &lpspiRegsPtr->RDR;
    LPSPI_SPB_REQUEST* requestPtr = TransferPtr->AssociatedRequestPtr;

    ULONG totalBytesWritten = 0;
    while (!LPSPIHwIsRxFifoEmpty(lpspiRegsPtr) && 
           !LPSPISpbIsAllDataTransferred(TransferPtr)) {

        ULONG rxFifoWord = READ_REGISTER_NOFENCE_ULONG(rxDataRegPtr);
        ULONG bytesWritten = LPSPIpHwWriteWordToMdl(TransferPtr, rxFifoWord);

        LPSPIpHwUpdateTransfer(TransferPtr, bytesWritten);

        totalBytesWritten += bytesWritten;
    }
    requestPtr->TotalBytesTransferred += totalBytesWritten;

    if (LPSPISpbIsTransferDone(TransferPtr)) {

        return TRUE;
    } else {

        LPSPIHwUpdateTransferConfiguration(DevExtPtr, TransferPtr);

        if (requestPtr->Type != LPSPI_REQUEST_TYPE::FULL_DUPLEX) {
             return LPSPIHwWriteZerosTxFIFO(DevExtPtr, TransferPtr);
        }
        return FALSE;
    }
}


//
// Routine Description:
//
//  LPSPIHwWriteZerosTxFIFO is called to write 0 to TX FIFO for
//  clocking in RX data.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
//  TransferPtr - The transfer object
//
// Return Value:
//  TRUE if burst scheduled but not started
//
_Use_decl_annotations_
BOOLEAN
LPSPIHwWriteZerosTxFIFO (
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    if (TransferPtr->IsStartBurst) {
        ULONG maxWordsToWrite = LPSPIHwQueryTxFifoSpace(DevExtPtr);
        if (maxWordsToWrite < 1) {
            return TRUE;
        }
        LPSPIHwWriteCmdInTxFIFO(DevExtPtr, TransferPtr);
        TransferPtr->IsStartBurst = FALSE;
    }

    return FALSE;
}


//
// Routine Description:
//
//  LPSPIHwClearFIFOs clears RX and TX FIFOs.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIHwClearFIFOs (
    LPSPI_DEVICE_EXTENSION* DevExtPtr
    )
{
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;

    LPSPI_CR ctrlReg = {
        READ_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CR)
    };
    ctrlReg.RRF = 1;
    ctrlReg.RTF = 1;
    WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->CR, ctrlReg.AsUlong);

    LPSPI_ASSERT(
        DevExtPtr->IfrLogHandle,
        LPSPIHwIsTxFifoEmpty(lpspiRegsPtr) && LPSPIHwIsRxFifoEmpty(lpspiRegsPtr)
        );
}

//
// Routine Description:
//
//  LPSPIHwpDisableTransferInterrupts disables the interrupts associated with 
//  the given transfer.
//
// Arguments:
//
//  DevExtPtr - The device extension
//
//  TransferPtr - The transfer to start.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIHwDisableTransferInterrupts (
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    const LPSPI_SPB_TRANSFER* TransferPtr
    )
{
    volatile LPSPI_REGISTERS* lpspiRegsPtr = DevExtPtr->LPSPIRegsPtr;

    LPSPI_IER intReg = {
        READ_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->IER)
        };

    if (LPSPISpbIsWriteTransfer(TransferPtr)) {

        intReg.AsUlong &= ~LPSPI_TX_INTERRUPTS;

    } else {

        intReg.AsUlong &= ~LPSPI_RX_INTERRUPTS;
    }

    WRITE_REGISTER_NOFENCE_ULONG(&lpspiRegsPtr->IER, intReg.AsUlong);
}


//
// Routine Description:
//
//  LPSPIHwIsSupportedDataBitLength checked if the given data bit length 
//  is supported by the driver.
//
// Arguments:
//
//  DataBitLength - Desired bit length
//
// Return Value:
//
//  TRUE - The give data bit length is supported by the driver, otherwise
//      FALSE.
//
_Use_decl_annotations_
BOOLEAN
LPSPIHwIsSupportedDataBitLength (
    ULONG DataBitLength
    )
{
    switch (DataBitLength) {
    case 8:
    case 16:
    case 32:
        return TRUE;

    default:
        return FALSE;
    }
}


// 
// LPSPIhw private methods
// -----------------------
//


//
// Routine Description:
//
//  LPSPIpHwCalcFreqDivider calculate the frequency divider settings
//  based on the required connection speed.
//
// Arguments:
//
//  DevExtPtr - The device extension.
//  
//  ConnectionSpeedHz - Desired connection speed in Hz.
//
//  PrescalerPtr - Address of a Prescaler value
//
//  ScalerPtr - Address of a Prescaler value
//
// Return Value:
//  
//  NTSTATUS
//
_Use_decl_annotations_
NTSTATUS
LPSPIpHwCalcFreqDivider (
    LPSPI_DEVICE_EXTENSION* DevExtPtr,
    ULONG ConnectionSpeedHz,
    ULONG* PrescalerPtr,
    ULONG* ScalerPtr
    )
{
    NTSTATUS result = STATUS_SUCCESS;
    //
    // Connection speed has been validated during target connection.
    //
    ULONG refClockHz = LPSPIDeviceGetReferenceClock(DevExtPtr);

    ULONG prescaler, bestPrescaler;
    ULONG scaler, bestScaler;
    ULONG realConnectionSpeed, bestConnectionSpeed;
    ULONG diff, min_diff;
    ULONG desiredConnectionSpeed = ConnectionSpeedHz;

    LPSPI_ASSERT(
        DevExtPtr->IfrLogHandle,
        DevExtPtr->PrescaleMax <= 7U
    );

    /* find combination of prescaler and scaler resulting in ConnectionSpeed closest to the
     * requested value
     */
    min_diff = 0xFFFFFFFFU;

    /* Set to maximum divisor value bit settings so that if baud rate passed in is less
     * than the minimum possible baud rate, then the SPI will be configured to the lowest
     * possible baud rate
     */
    bestPrescaler = DevExtPtr->PrescaleMax;
    bestScaler = 255U;

    bestConnectionSpeed = 0;

    /* In all for loops, if min_diff = 0, the exit for loop */
    for (prescaler = 0U; prescaler <= DevExtPtr->PrescaleMax; prescaler++)
    {
        if (min_diff == 0U)
        {
            break;
        }
        for (scaler = 0U; scaler < 256U; scaler++)
        {
            if (min_diff == 0U)
            {
                break;
            }
            realConnectionSpeed = (refClockHz / ((1 << prescaler) * (scaler + 2U)));

            /* calculate the baud rate difference based on the conditional statement
             * that states that the calculated baud rate must not exceed the desired baud rate
             */
            if (desiredConnectionSpeed >= realConnectionSpeed)
            {
                diff = desiredConnectionSpeed - realConnectionSpeed;
                if (min_diff > diff)
                {
                    /* a better match found */
                    min_diff = diff;
                    bestPrescaler = prescaler;
                    bestScaler = scaler;
                    bestConnectionSpeed = realConnectionSpeed;
                }
            }
        }
    }

    //
    // Verify connection speed error is in range
    //
    {
        ULONG speedErrorPercent = (min_diff * 100) / ConnectionSpeedHz;

        if (speedErrorPercent > MAX_SPEED_ERROR_PERCENT) {

            LPSPI_LOG_ERROR(
                DevExtPtr->IfrLogHandle,
                "Connection speed error %lu%% is out of range. "
                "Max connection error %lu%%",
                speedErrorPercent,
                MAX_SPEED_ERROR_PERCENT
                );
            result = STATUS_NOT_SUPPORTED;
        }

        LPSPI_LOG_INFORMATION(
            DevExtPtr->IfrLogHandle,
            "Connection speed set to %luHz, reference clock %luHz. "
            "PRESCALER %lu, SCALER %lu, error %lu%%",
            ConnectionSpeedHz,
            refClockHz,
            bestPrescaler,
            bestScaler,
            speedErrorPercent
            );

    } // Verify connection speed error is in range

    if (PrescalerPtr != NULL) {
        *PrescalerPtr = bestPrescaler;
    }
    if (ScalerPtr != NULL) {
        *ScalerPtr = bestScaler;
    }
    return result;
}


//
// Routine Description:
//
//  LPSPIpHwReadWordFromMdl reads a single word from transfer MDL, 
//  and updates the transfer.
//
// Arguments:
//
//  TransferPtr - The transfer descriptor
//
//  DataPtr - Address of a caller ULONG to receive the word to be
//      written to TX FIFO.
//
// Return Value:
//  
//  Number of bytes read from MDL
//
_Use_decl_annotations_
ULONG
LPSPIpHwReadWordFromMdl (
    LPSPI_SPB_TRANSFER* TransferPtr,
    PULONG DataPtr
    )
{
    PMDL mdlPtr = TransferPtr->CurrentMdlPtr;
    size_t mdlOffset = TransferPtr->CurrentMdlOffset;
    ULONG bytesToRead = sizeof(ULONG);
    ULONG readData = 0;
    UCHAR* dataBytePtr = reinterpret_cast<UCHAR*>(&readData);

    bytesToRead = TransferPtr->BytesLeftInBurst % sizeof(ULONG);
    // Is the last word
    if (bytesToRead != TransferPtr->BytesLeftInBurst) {
        bytesToRead = sizeof(ULONG);
    }

    ULONG bytesLeftToRead = bytesToRead;
    while (mdlPtr != nullptr) {

        const UCHAR* mdlAddr =
            reinterpret_cast<const UCHAR*>(mdlPtr->MappedSystemVa) + mdlOffset;
        size_t byteCount = MmGetMdlByteCount(mdlPtr) - mdlOffset;

        while (byteCount > 0) {

            *dataBytePtr = *mdlAddr;

            ++dataBytePtr;
            ++mdlAddr;
            ++mdlOffset;

            --bytesLeftToRead;
            if (bytesLeftToRead == 0) {

                goto done;
            }

            --byteCount;

        } // More bytes to write

        mdlPtr = mdlPtr->Next;
        mdlOffset = 0;

    } // More MDLs

done:

    NT_ASSERT(bytesLeftToRead == 0);

    *DataPtr = LPSPIpHwDataSwap(readData, bytesToRead, TransferPtr->BufferStride);

    TransferPtr->CurrentMdlPtr = mdlPtr;
    TransferPtr->CurrentMdlOffset = mdlOffset;
    return bytesToRead - bytesLeftToRead;
}


//
// Routine Description:
//
//  LPSPIpHwWriteWordToMdl writes a single word to transfer MDL, 
//  and updates the transfer.
//
// Arguments:
//
//  TransferPtr - The transfer descriptor
//
//  DataPtr - Data to be written to MDL.
//
// Return Value:
//  
//  Number of bytes written to MDL
//
_Use_decl_annotations_
ULONG
LPSPIpHwWriteWordToMdl (
    LPSPI_SPB_TRANSFER* TransferPtr,
    ULONG Data
    )
{
    PMDL mdlPtr = TransferPtr->CurrentMdlPtr;
    size_t mdlOffset = TransferPtr->CurrentMdlOffset;
    ULONG bytesToWrite = sizeof(ULONG);
    ULONG dataToWrite;
    const UCHAR* dataBytePtr = reinterpret_cast<const UCHAR*>(&dataToWrite);

    bytesToWrite = TransferPtr->BytesLeftInBurst % sizeof(ULONG);
    // Is the last word
    if (bytesToWrite != TransferPtr->BytesLeftInBurst) {
        bytesToWrite = sizeof(ULONG);
    }

    dataToWrite = LPSPIpHwDataSwap(Data, bytesToWrite, TransferPtr->BufferStride);

    ULONG bytesLeftToWrite = bytesToWrite;
    while (mdlPtr != nullptr) {

        UCHAR* mdlAddr = 
            reinterpret_cast<UCHAR*>(mdlPtr->MappedSystemVa) + mdlOffset;
        size_t byteCount = MmGetMdlByteCount(mdlPtr) - mdlOffset;

        while (byteCount > 0) {

            *mdlAddr = *dataBytePtr;

            ++dataBytePtr;
            ++mdlAddr;
            ++mdlOffset;

            --bytesLeftToWrite;
            if (bytesLeftToWrite == 0) {

                goto done;
            }

            --byteCount;

        } // More bytes to write

        mdlPtr = mdlPtr->Next;
        mdlOffset = 0;

    } // More MDLs

done:

    NT_ASSERT(bytesLeftToWrite == 0);

    TransferPtr->CurrentMdlPtr = mdlPtr;
    TransferPtr->CurrentMdlOffset = mdlOffset;
    return bytesToWrite - bytesLeftToWrite;
}


//
// Routine Description:
//  LPSPIpHwUpdateTransfer updates the transfer/burst progress with the number
//  of bytes transferred, and figures out if a new burst needs to be started. 
//
// Arguments:
//
//  TransferPtr - The transfer object.
//
//  BytesTransferred - Number of bytes transfered.
//
// Return Value:
//
_Use_decl_annotations_
VOID
LPSPIpHwUpdateTransfer (
    LPSPI_SPB_TRANSFER* TransferPtr,
    ULONG BytesTransferred
    )
{
    TransferPtr->BytesTransferred += BytesTransferred;
    TransferPtr->BytesLeftInBurst -= BytesTransferred;
    
    if (TransferPtr->BytesLeftInBurst == 0) {

        TransferPtr->BytesLeftInBurst =
            LPSPISpbBytesLeftToTransfer(TransferPtr);
        TransferPtr->BytesLeftInBurst =
            min(TransferPtr->BytesLeftInBurst, LPSPI_MAX_BURST_LENGTH_BYTES);
        
        if (TransferPtr->BytesLeftInBurst > 0) {

            TransferPtr->BurstLength = TransferPtr->BytesLeftInBurst;
            TransferPtr->BurstWords = LPSPISpbWordsLeftInBurst(TransferPtr);
            TransferPtr->IsStartBurst = TRUE;
            TransferPtr->CurrentBurst++;
        }
    }
}

#undef _LPSPI_HW_CPP_