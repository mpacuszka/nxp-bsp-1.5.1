/*
 * Copyright 2022 NXP
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

/* 

Module Name:

    imxcontroller.cpp

Abstract:

    This module contains the controller-specific functions
    for handling transfers.

Environment:

    kernel-mode only

Revision History:

*/

#include "imxlpi2cinternal.h"
#include "imxlpi2ccontroller.h"
#include "imxlpi2cdevice.h"

#include "imxcontroller.tmh"

// Min/Max defines.
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

const PBC_TRANSFER_SETTINGS g_TransferSettings[] =
{
    // Bus condition        IsStart  IsEnd
    {BusConditionDontCare,  FALSE,   FALSE}, // SpbRequestTypeInvalid
    {BusConditionFree,      TRUE,    TRUE},  // SpbRequestTypeSingle
    {BusConditionFree,      TRUE,    FALSE}, // SpbRequestTypeFirst
    {BusConditionBusy,      FALSE,   FALSE}, // SpbRequestTypeContinue
    {BusConditionBusy,      FALSE,   TRUE}   // SpbRequestTypeLast
};

typedef struct I2C_SPEC_TIMING {
    USHORT TimeSuStaSuStoMinSpec;    // Setup time for a repeated START condition or STOP condition
    USHORT TimeHdStaMinSpec;         // Hold time (repeated) START condition
    USHORT TimeSclLowMinSpec;        // LOW period of the SCL clock
    USHORT TimeSclHighMinSpec;       // HIGH period of the SCL clock
}
I2C_SPEC_TIMING, * PI2C_SPEC_TIMING;

const I2C_SPEC_TIMING g_I2CSpecTiminParameters[3] =
{
    // SuStaSuStoMin  HdStaMin  SclLowMin  SclHighMin    [ns]
    {     4700,        4000,      4700,      4000 },  // Standard mode (SCL frequency <= 100KHz)
    {      600,         600,      1300,       600 },  // FAST mode (SCL frequency (100KHz; 400kHz>)
    {      260,         260,       500,       260 }   // FAST+ mode (SCL frequency (400KHz; 1MHz>)
};

#define MAX_BIT_GROUP_VALUE(size) ((1 << (size)) - 1)  

static ULONG GetTimeInCyclesRoundedUp(ULONG Frequency_Hz, UCHAR Prescale, ULONG Time_ns)
{
    return ((((Time_ns * Frequency_Hz) >> Prescale) / 10000000) + 99) / 100;
}

static ULONG GetTimeInNs(ULONG Frequency_Hz, UCHAR Prescale, ULONG Time_clocks)
{
    return ((1000000000UL * Time_clocks) << Prescale) / Frequency_Hz;
}

#define IMX_LPI2C_PERMANENT_INT_MASK (IMX_LPI2C_MIER_PLTIE_MASK | IMX_LPI2C_MIER_FEIE_MASK | \
IMX_LPI2C_MIER_ALIE_MASK | IMX_LPI2C_MIER_NDIE_MASK | IMX_LPI2C_MIER_SDIE_MASK)


static VOID ControllerRequestForStartAndAddrGen(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_ PPBC_REQUEST RequestPtr,
    _In_ UCHAR SlaveAddress
);

static NTSTATUS ControllerArrangeNextBytesTransfer(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_ PPBC_REQUEST RequestPtr,
    _In_ BOOLEAN PutGetBytesInFromI2C,
    _Out_ BOOLEAN *RequestDone
);

static VOID ControllerUpdateTxInformationByTxFifo(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_ PPBC_REQUEST RequestPtr,
    _In_ size_t TxFifoCount
);

static NTSTATUS ControllerProcessAlreadyTransferedBytes(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  PPBC_REQUEST RequestPtr
);

#if (defined(DBG) || defined(DEBUG))
/*++

  Routine Description:

    This routine dumps hardware registers.

  Arguments:

    DeviceCtxPtr - a pointer to the PBC device context

  Return Value:

    None.

--*/
_Use_decl_annotations_
VOID ControllerDumpRegisters(PDEVICE_CONTEXT DeviceCtxPtr)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "M VERID=0x%08X PARAM=0x%08X "
        "MCR=0x%08X MSR=0x%08X MIER=0x%08X MDER=0x%08X "
        "MCFGR0=0x%08X MCFGR1=0x%08X MCFGR2=0x%08X MCFGR3=0x%08X "
        "MDMR=0x%08X MCCR0=0x%08X MCCR1=0x%08X MFCR=0x%08X MFSR=0x%08X",
        DeviceCtxPtr->RegistersPtr->VERID,
        DeviceCtxPtr->RegistersPtr->PARAM,
        DeviceCtxPtr->RegistersPtr->MCR,
        DeviceCtxPtr->RegistersPtr->MSR,
        DeviceCtxPtr->RegistersPtr->MIER,
        DeviceCtxPtr->RegistersPtr->MDER,
        DeviceCtxPtr->RegistersPtr->MCFGR0,
        DeviceCtxPtr->RegistersPtr->MCFGR1,
        DeviceCtxPtr->RegistersPtr->MCFGR2,
        DeviceCtxPtr->RegistersPtr->MCFGR3,
        DeviceCtxPtr->RegistersPtr->MDMR,
        DeviceCtxPtr->RegistersPtr->MCCR0,
        DeviceCtxPtr->RegistersPtr->MCCR1,
        DeviceCtxPtr->RegistersPtr->MFCR,
        DeviceCtxPtr->RegistersPtr->MFSR
    );
}
#endif

/*++

  Routine Description:

    This routine initializes the controller hardware.

  Arguments:

    DeviceCtxPtr - a pointer to the PBC device context

  Return Value:

    None.

--*/
_Use_decl_annotations_
NTSTATUS ControllerInitialize(PDEVICE_CONTEXT DeviceCtxPtr)
{
    NTSTATUS status = STATUS_SUCCESS;
    size_t txFifoSize, rxFifoSize;
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerInitialize()");

    // Disable I2C Module Initially
    DeviceCtxPtr->RegistersPtr->MCR = (ULONG)0x00;
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "ControllerInitialize() i2c block is disabled");


    // Find out I2C clock related configurations to default connection frequency.
    status = FindControllerClockSettings(DeviceCtxPtr, IMX_I2C_INIT_CONNECTION_SPEED);

    if (status != STATUS_SUCCESS) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "ControllerInitialize() I2C connection speed %lu is not complied with all requirements.",
            IMX_I2C_INIT_CONNECTION_SPEED
        );
        goto EndControllerInitialize;
    }

    // Reset controller.
    status = ResetController(DeviceCtxPtr);

    // Get FIFO size from VERID register
    txFifoSize = 1ULL << ((DeviceCtxPtr->RegistersPtr->PARAM & IMX_LPI2C_PARAM_MTXFIFO_MASK) >>
        IMX_LPI2C_PARAM_MTXFIFO_SHIFT);
    rxFifoSize = 1ULL << ((DeviceCtxPtr->RegistersPtr->PARAM & IMX_LPI2C_PARAM_MRXFIFO_MASK) >>
        IMX_LPI2C_PARAM_MRXFIFO_SHIFT);

    if (txFifoSize < 4) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "ControllerInitialize() Tx FIFO size %lld is too small",
            txFifoSize
        );

        status = STATUS_UNSUCCESSFUL;
        goto EndControllerInitialize;
    }
    
    DeviceCtxPtr->TxFifoSize = txFifoSize;
    DeviceCtxPtr->RxFifoSize = rxFifoSize;
    // Prepare Rx watermark max to half Rx FIFO size
    DeviceCtxPtr->RxFifoWatermarkMax = (rxFifoSize >> 1) - 1;

    // Prepare Tx watermark to half Tx FIFO size (the 4 is reserved for commands of a next transfer)
    DeviceCtxPtr->TxFifoWatermarkMax = (txFifoSize - 4) >> 1;

    // Enable I2C device
    DeviceCtxPtr->RegistersPtr->MCR |= IMX_LPI2C_MCR_MEN_MASK;

    if (status != STATUS_SUCCESS) {

       TraceEvents(TRACE_LEVEL_ERROR, TRACE_CTRLR,
                    "ControllerInitialize() i2c block reset failed!");
       goto EndControllerInitialize;
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "ControllerInitialize() i2c block is reset and disabled.");

    // Clear both Fifo
    DeviceCtxPtr->RegistersPtr->MCR |= IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "ControllerInitialize() MCR reg=%04Xh",
                DeviceCtxPtr->RegistersPtr->MCR);

    // Set I2C clock related registers
    SetControllerClockDiv(DeviceCtxPtr);

EndControllerInitialize:
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerInitialize()=%Xh", status);
    return status;
}

/*++

  Routine Description:

    This routine finds best fit clock divider for given i2c clock speed in kHz

  Arguments:

    DeviceCtxPtr - a pointer to device context
    DesiredClockFrequencyHz - desired bus clock frequency in kHz

  Return Value:

    status_success if it was possible to find close enough clock configuration 
    error if not

--*/
_Use_decl_annotations_
NTSTATUS FindControllerClockSettings(
    PDEVICE_CONTEXT DeviceCtxPtr,
    ULONG DesiredClockFrequencyHz
)
{
    NTSTATUS status = STATUS_SUCCESS;
    ULONG sdaGlitchFilter = 0;
    ULONG sclGlitchFilter = 0;
    ULONG glitchFilterMaxBits;
    ULONG clk, clkHighMaxBits;
    ULONG clkHighMinBits, clkLowMin;
    ULONG clkHighBits, clkLow, prescaledClkHigh;
    ULONG setHoldBits, setHoldMinBits;
    ULONG hdStaMinBits, suStaSuStoMinBits;
    UINT8 prescale;
    ULONG sclLatency, sdaLatency;
    ULONG busIdleMinBits, dataValidBits, dataValidMaxBits;
    ULONG busIdleBits = 0;
    ULONG pinLowTimeoutBits = 0;
    ULONG sclRisetime, sdaRisetime;
    ULONG ratio3_2, ratio, ratioDiv;
    ULONG clockFrequency, clockFrequencyErr;
    ULONG resultClockFrequencyErr = MAX_BIT_GROUP_VALUE(sizeof(ULONG));
    ULONG resultRatioDiv = MAX_BIT_GROUP_VALUE(sizeof(ULONG));
    ULONG resultClockFrequency = 0;
    UCHAR resultPrescaleBits = 0;
    ULONG resultClk = 0;
    ULONG resultClkHighBits = 0;
    ULONG resultClkLowBits = 0;
    ULONG resultSetHoldBits = 0;
    ULONG resultBusIdleBits = 0;
    ULONG resultDataValidBits = 0;
    ULONG resultPinLowTimeoutBits = 0;
    ULONG resultSclLatency = 0;
    ULONG resultSdaLatency = 0;
    const I2C_SPEC_TIMING *I2CSpecTiminParameters = NULL;
    ULONG clkHighClocks, clkLowClocks, ratioMul10, tBuf_VdDat;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "++FindControllerClockSettings(%lu Hz)",
        DesiredClockFrequencyHz);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "Input parameters: DesiredClockFrequency: %luHz, ModuleClock: %luHz, "
        "SdaGlitchFilter: %luns, SdaRisetime %luns, "
        "SclGlitchFilter: %luns, SclRisetime %luns, "
        "BusIdle: %luns, PinLowTimeout %luns",
        DesiredClockFrequencyHz, DeviceCtxPtr->ModuleClock_Hz,
        DeviceCtxPtr->SdaGlitchFilter_ns, DeviceCtxPtr->SdaRisetime_ns,
        DeviceCtxPtr->SclGlitchFilter_ns, DeviceCtxPtr->SclRisetime_ns,
        DeviceCtxPtr->BusIdle_ns, DeviceCtxPtr->RegistersPtr->MCFGR2);

    if ((DesiredClockFrequencyHz > IMX_I2C_MAX_CONNECTION_SPEED) ||
        (DesiredClockFrequencyHz < IMX_I2C_MIN_CONNECTION_SPEED)) {

        // error - out of range
        status = STATUS_NOT_SUPPORTED;
        goto DoneFindControllerClockSettings;
    }

    if (DesiredClockFrequencyHz <= 100000) {
        // Standard mode
        I2CSpecTiminParameters = &g_I2CSpecTiminParameters[0];
    }
    else {
        if (DesiredClockFrequencyHz <= 400000) {
            // FAST mode
            I2CSpecTiminParameters = &g_I2CSpecTiminParameters[1];
        }
        else {
            if (DesiredClockFrequencyHz <= 1000000) {
                // FAST+ mode
                I2CSpecTiminParameters = &g_I2CSpecTiminParameters[2];
            }
            else {
                TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
                    "FindControllerClockSettings() Error Desired clock frequency %luHz not supported",
                    DesiredClockFrequencyHz);

                status = STATUS_NOT_SUPPORTED;
                goto DoneFindControllerClockSettings;
            }
        }
    }


    if (DeviceCtxPtr->SdaGlitchFilter_ns < DeviceCtxPtr->SclGlitchFilter_ns) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "FindControllerClockSettings() Error SdaGlitchFilter %luns is less than SclGlitchFilter %luns",
            DeviceCtxPtr->SdaGlitchFilter_ns, DeviceCtxPtr->SclGlitchFilter_ns);

        status = STATUS_NOT_SUPPORTED;
        goto DoneFindControllerClockSettings;
    }

    // Find SCL glitch filter setting
    sclGlitchFilter = GetTimeInCyclesRoundedUp(DeviceCtxPtr->ModuleClock_Hz, 0,
        DeviceCtxPtr->SclGlitchFilter_ns);

    if (sclGlitchFilter > MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCFGR2_FILTSCL_SIZE)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "FindControllerClockSettings() Error SclGlitchFilter %luns cannot be set into 4 bits",
            DeviceCtxPtr->SclGlitchFilter_ns);

        status = STATUS_NOT_SUPPORTED;
        goto DoneFindControllerClockSettings;
    }

    // Find SDA glitch filter setting
    sdaGlitchFilter = GetTimeInCyclesRoundedUp(DeviceCtxPtr->ModuleClock_Hz, 0,
        DeviceCtxPtr->SdaGlitchFilter_ns);

    if ((sdaGlitchFilter < sclGlitchFilter) ||
        (sdaGlitchFilter > MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCFGR2_FILTSCL_SIZE))) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "FindControllerClockSettings() Error SdaGlitchFilter %luns cannot be set into 4 bits or "
            "it is less than SclGlitchFilter %dns",
            DeviceCtxPtr->SdaGlitchFilter_ns, DeviceCtxPtr->SclGlitchFilter_ns);

        status = STATUS_NOT_SUPPORTED;
        goto DoneFindControllerClockSettings;
    }

    sclRisetime = GetTimeInCyclesRoundedUp(DeviceCtxPtr->ModuleClock_Hz, 0,
        DeviceCtxPtr->SclRisetime_ns);

    sdaRisetime = GetTimeInCyclesRoundedUp(DeviceCtxPtr->ModuleClock_Hz, 0,
        DeviceCtxPtr->SdaRisetime_ns);

    // Iterate through all prescaler values and find the best I2C clock configuration
    for (prescale = 0; prescale < 8; prescale++) {

        if (DeviceCtxPtr->BusIdle_ns > 0) {
            busIdleBits = GetTimeInCyclesRoundedUp(DeviceCtxPtr->ModuleClock_Hz, prescale,
                DeviceCtxPtr->BusIdle_ns);
            if (busIdleBits > MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCFGR2_BUSIDLE_SIZE)) {
                continue;
            }
        }

        if (DeviceCtxPtr->PinLowTimeout_ns > 0) {
            pinLowTimeoutBits = GetTimeInCyclesRoundedUp(DeviceCtxPtr->ModuleClock_Hz, prescale,
                DeviceCtxPtr->PinLowTimeout_ns);
            if (pinLowTimeoutBits > MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCFGR3_PINLOW_SIZE)) {
                continue;
            }
        }

        sclLatency = (2 + sclGlitchFilter + sclRisetime) >> prescale;
        sdaLatency = (2 + sdaGlitchFilter + sdaRisetime) >> prescale;

        // Get potential number of Module clocks inside one SCL period (rounded up)
        clk = (DeviceCtxPtr->ModuleClock_Hz + (DesiredClockFrequencyHz << prescale) - 1) /
            (DesiredClockFrequencyHz << prescale);
        if ((clk - 2 - sclLatency) > (MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCCR0_CLKLO_SIZE) +
            MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCCR0_CLKHI_SIZE))) {
            continue;
        }
        clockFrequency = DeviceCtxPtr->ModuleClock_Hz / (clk << prescale);
        if (clockFrequency > DesiredClockFrequencyHz) {
            continue;
        }
        clockFrequencyErr = DesiredClockFrequencyHz - clockFrequency;
        if (clockFrequencyErr > resultClockFrequencyErr) {
            continue;
        }
        clkHighMinBits = GetTimeInCyclesRoundedUp(DeviceCtxPtr->ModuleClock_Hz, prescale,
            I2CSpecTiminParameters->TimeSclHighMinSpec);  // (SCL_LATENCY + 1) will be subtracted below
        clkLowMin = GetTimeInCyclesRoundedUp(DeviceCtxPtr->ModuleClock_Hz, prescale,
            I2CSpecTiminParameters->TimeSclLowMinSpec);
        hdStaMinBits = GetTimeInCyclesRoundedUp(DeviceCtxPtr->ModuleClock_Hz, prescale,
            I2CSpecTiminParameters->TimeHdStaMinSpec);
        suStaSuStoMinBits = GetTimeInCyclesRoundedUp(DeviceCtxPtr->ModuleClock_Hz, prescale,
            I2CSpecTiminParameters->TimeSuStaSuStoMinSpec);
        if ((hdStaMinBits == 0) || (suStaSuStoMinBits == 0)) {
            break;
        }
        hdStaMinBits--;
        suStaSuStoMinBits--;

        // Determine CLKHI (clkHighMin) minimal register value according to clkHigh = CLKHI + 1 + SCL_LATENCY
        if (clkHighMinBits <= sclLatency) {  // there are clocks in clkHighMinBits; no bit value yet
            clkHighMinBits = 1;              // CLKHI >= 1
        }
        else {
            clkHighMinBits -= sclLatency;    // there are clocks in clkHighMinBits; no bit value yet
            if (clkHighMinBits > 1) {
                clkHighMinBits--;            // minus 1 to get bit value in clkHighMinBits
            }
            else {
                clkHighMinBits = 1;          // CLKHI >= 1
            }
        }

        clkHighMaxBits = MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCCR0_CLKHI_SIZE);
        if (clkHighMinBits > clkHighMaxBits) {
            continue;
        }

        ratio3_2 = ((3 * clk) << 10) / 5;      // low:high ideally 3:2

        // Iterate through clkHigh (CLKHI bits) value starting found minimal value and 
        // ending with max value for given bit group size or clkLow to low and
        // search for meeting with the following conditions:
        // - clk_high + clk_low = clk (CLKHI + 1 + SCL_LATENCY + CLKLO + 1 = clk)
        // - ratio clkHigh:clkLow -> 3:2
        // - CLKLO >= 3; CLKLO x (2 ^ PRESCALE) > SCL_LATENCY
        // - CLKHI >= 1; SETHOLD >= 2; 1 <= DATAVD <= CLKLO - SDA_LATENCY - 1
        // - 0 <= FILTSCL <= [CLKLO x (2 ^ PRESCALE)] - 3
        // - FILTSCL <= FILTSDA <= [CLKLO x (2 ^ PRESCALE)] - 3
        // - BUSIDLE >= (CLKLO + SETHOLD + 2) x 2; BUSIDLE > CLKHI + 1
        // Note: clkHigh is increasing and clkLow is decreasing in this for
        for (clkHighBits = clkHighMinBits; clkHighBits <= clkHighMaxBits; clkHighBits++) {
            
            if (clk < (clkHighBits + 4)) {  // 4 ~ CLKHI >= 1 && CLKLO >= 3 where CLKHI >= 1 is already satisfied
                continue;                   // continue if CLKLO < 3
            }
            
            prescaledClkHigh = clkHighBits + 1 + sclLatency;
//            NT_ASSERT(prescaledClkHigh >= clkHighMinBits);

            clkLow = clk - prescaledClkHigh;  
            if (clkLow < clkLowMin) {
                break;           // does not make sense to continue with this for
            }
            
            clkLow -= 1;         // from now there is value which can be directly written into register           
            if (clkLow < 3) {    // CLKLO >= 3
                break;           // does not make sense to continue with this for because next value will be less
            }
            if (clkLow > MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCCR0_CLKLO_SIZE)) {
                continue;
            }
            if (clkLow <= sclLatency) {
                continue;
            }
            // Check already found SCL and SDA glitch filters settings for the limits
            glitchFilterMaxBits = (clkLow << prescale) - 3;
            if ((sclGlitchFilter > glitchFilterMaxBits) ||
                (sdaGlitchFilter > glitchFilterMaxBits)) {
                continue;
            }

            // Lets try to set Setup Hold Delay as clock high
            setHoldBits = clkHighBits + sclLatency;
            setHoldMinBits = MAX(2, MAX(suStaSuStoMinBits, hdStaMinBits));
            if (setHoldBits < setHoldMinBits) {
                setHoldBits = setHoldMinBits;
            }
            if (setHoldBits > MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCCR0_SETHOLD_SIZE)) {
                setHoldBits = MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCCR0_SETHOLD_SIZE);
            }
            if ((setHoldBits < suStaSuStoMinBits) || (setHoldBits < hdStaMinBits)) {
                continue;
            }

            // Let's try to set Valid Delay as an half clock high
            dataValidBits = clkHighBits >> 1;
            if (dataValidBits < 1) {
                dataValidBits = 1;
            }
            if (clkLow <= (sdaLatency + 1)) {
                continue;
            }
            dataValidMaxBits = clkLow - (sdaLatency + 1);
            if (dataValidBits > dataValidMaxBits) {
                dataValidBits = dataValidMaxBits;
            }
            NT_ASSERT(dataValidBits <= MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCCR0_DATAVD_SIZE));

            // Try to satisfy Bus Idle requirement
            if (DeviceCtxPtr->BusIdle_ns > 0) {
                busIdleMinBits = MAX(((clkLow + setHoldBits + 2) * 2), (clkHighBits + 2));
                if (busIdleBits < busIdleMinBits) {
                    busIdleBits = busIdleMinBits;
                }
                if (busIdleBits > MAX_BIT_GROUP_VALUE(IMX_LPI2C_MCFGR2_BUSIDLE_SIZE)) {
                    continue;
                }
            }

            ratio = (clkLow + 1) << 10;
            ratioDiv = (ratio > ratio3_2) ? (ratio - ratio3_2) : (ratio3_2 - ratio);
            
            // Let's take such clock configuration for which the SCL frequency is the closest to
            // the desired one and has the best duty
            if ((clockFrequencyErr < resultClockFrequencyErr) || (ratioDiv < resultRatioDiv)) {
                resultClockFrequencyErr = clockFrequencyErr;
                resultRatioDiv = ratioDiv;
                resultClockFrequency = clockFrequency;
                resultPrescaleBits = prescale;
                resultClk = clk;
                resultClkHighBits = clkHighBits;
                resultClkLowBits = clkLow;
                resultSetHoldBits = setHoldBits;
                resultBusIdleBits = busIdleBits;
                resultDataValidBits = dataValidBits;
                resultPinLowTimeoutBits = pinLowTimeoutBits;
                resultSclLatency = sclLatency;
                resultSdaLatency = sdaLatency;
            }
        }

        if (resultClockFrequencyErr != 0) {
            break;
        }
    }

    if (resultClockFrequency == 0) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "FindControllerClockSettings() Error Valid clock configuration could not be found");

        status = STATUS_NOT_SUPPORTED;
        goto DoneFindControllerClockSettings;
    }

    // Store found bit values for SetControllerClockDiv function
    DeviceCtxPtr->Mcrr0Reg = IMX_LPI2C_MCCR0_DATAVD(resultDataValidBits) |
        IMX_LPI2C_MCCR0_SETHOLD(resultSetHoldBits) |
        IMX_LPI2C_MCCR0_CLKHI(resultClkHighBits) |
        IMX_LPI2C_MCCR0_CLKLO(resultClkLowBits);
    DeviceCtxPtr->Mcfgr1RegPrescaleBits = IMX_LPI2C_MCFGR1_PRESCALE(resultPrescaleBits);
    DeviceCtxPtr->Mcfgr2Reg = IMX_LPI2C_MCFGR2_FILTSDA(sdaGlitchFilter)
        | IMX_LPI2C_MCFGR2_FILTSCL(sclGlitchFilter)
        | IMX_LPI2C_MCFGR2_BUSIDLE(resultBusIdleBits);
    DeviceCtxPtr->Mcfgr3Reg = IMX_LPI2C_MCFGR3_PINLOW(resultPinLowTimeoutBits);

    clkHighClocks = resultClkHighBits + 1 + resultSclLatency;
    clkLowClocks = resultClkLowBits + 1;
    ratioMul10 = (2 * clkLowClocks * 10) / clkHighClocks;
    tBuf_VdDat = GetTimeInNs(resultClockFrequency, resultPrescaleBits, resultClkLowBits + 1 + resultSdaLatency);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "Found clock configuration: PRESCALE: 0x%02Xh, CLKHI: 0x%02Xh, CLKLO: 0x%02Xh, "
        "DATAVD: 0x%04Xh, SETHOLD: 0x%04Xh, BUSIDLE: 0x%04Xh, "
        "FILTSDA: 0x%02Xh, FILTSCL: 0x%02Xh, PINLOW: 0x%04Xh, ",
        resultPrescaleBits, resultClkHighBits, resultClkLowBits,
        resultDataValidBits, resultSetHoldBits, resultBusIdleBits,
        sdaGlitchFilter, sclGlitchFilter, resultPinLowTimeoutBits);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "  ClockFrequency: %lu (%luclocks), DesiredClockFrequencyHz: %lu, ",
        resultClockFrequency, resultClk, DesiredClockFrequencyHz);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "  ClkHigh: %luns (%luclocks), ClkLow: %luns (%luclocks), ratio(high:low): %d.%d:2, ",
        GetTimeInNs(resultClockFrequency, resultPrescaleBits, clkHighClocks), clkHighClocks,
        GetTimeInNs(resultClockFrequency, resultPrescaleBits, clkLowClocks), clkLowClocks,
        ratioMul10 / 10, ratioMul10 % 10);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "  tHD:STA: %luns, tSU:STA/STO: %luns, tHD:DAT: %luns, "
        "tSU:DAT: %luns, tBUF: %luns, tVD:DAT/ACK: %luns ",
        GetTimeInNs(resultClockFrequency, resultPrescaleBits, resultSetHoldBits + 1),  // tHD:STA
        GetTimeInNs(resultClockFrequency, resultPrescaleBits, resultSetHoldBits + 1 + resultSclLatency),  // tSU:STA/STO
        GetTimeInNs(resultClockFrequency, resultPrescaleBits, resultDataValidBits + 1),  // tHD:DAT
        GetTimeInNs(resultClockFrequency, resultPrescaleBits, resultSdaLatency + 1),  // tSU:DAT
        tBuf_VdDat, tBuf_VdDat);

DoneFindControllerClockSettings:

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "--FindControllerClockSettings()=%Xh",
                status);
    return status;
}

/*++

  Routine Description:

    This routine applies previously found clock configuration for given i2c clock speed in kHz

  Arguments:

    DeviceCtxPtr - a pointer to device context

  Return Value:

    None

--*/
_Use_decl_annotations_
VOID SetControllerClockDiv(
    PDEVICE_CONTEXT DeviceCtxPtr
)
{
    ULONG isI2CEnabled;
    UNREFERENCED_PARAMETER(DeviceCtxPtr);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "++SetControllerClockDiv()");

    isI2CEnabled = DeviceCtxPtr->RegistersPtr->MCR & IMX_LPI2C_MCR_MEN_MASK;
    if (isI2CEnabled) {
        DeviceCtxPtr->RegistersPtr->MCR &= ~IMX_LPI2C_MCR_MEN_MASK;
    }

    DeviceCtxPtr->RegistersPtr->MCCR0 = DeviceCtxPtr->Mcrr0Reg;
    DeviceCtxPtr->RegistersPtr->MCFGR1 = (DeviceCtxPtr->RegistersPtr->MCFGR1 & ~IMX_LPI2C_MCFGR1_PRESCALE_MASK) |
        DeviceCtxPtr->Mcfgr1RegPrescaleBits;
    DeviceCtxPtr->RegistersPtr->MCFGR2 = DeviceCtxPtr->Mcfgr2Reg;
    DeviceCtxPtr->RegistersPtr->MCFGR3 = DeviceCtxPtr->Mcfgr3Reg;

    if (isI2CEnabled) {
        DeviceCtxPtr->RegistersPtr->MCR |= IMX_LPI2C_MCR_MEN_MASK;
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "--SetControllerClockDiv()");
}

/*++

  Routine Description:

    This routine resets iMX i2c controller

  Arguments:

    DeviceCtxPtr - a pointer to device context
    DisableFirst - boolean to specify should controller be disabled first

  Return Value:

    Ntstatus of operation.

--*/
_Use_decl_annotations_
NTSTATUS ResetController(
    PDEVICE_CONTEXT DeviceCtxPtr
    )
{
    NTSTATUS status = STATUS_SUCCESS;
    LARGE_INTEGER liTimeout = {0};

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ResetController()");

    liTimeout.QuadPart = (LONGLONG)(-10); // 1us in 100ns units

    // enable and reset given i2c controller hw
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "ResetController() MCR reg=%04Xh upon entry",
                DeviceCtxPtr->RegistersPtr->MCR);

    // Disable and reset I2C Master Module .
    // Note: WinCE and iMX data flow example clears entire i2cr register
    DeviceCtxPtr->RegistersPtr->MCR = IMX_LPI2C_MCR_RST_MASK;
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "ResetController() i2c block is disabled and starting reset");

    // Wait a bit 
    KeDelayExecutionThread(KernelMode, FALSE, &liTimeout);

    // Leave reset state with I2C master module disabled
    DeviceCtxPtr->RegistersPtr->MCR = (ULONG)0x0;
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "ResetController() i2c block is ending reset");

    // Wait a bit 
    KeDelayExecutionThread(KernelMode, FALSE, &liTimeout);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "--ResetController()=%Xh",
                status);

    return status;
}

/*++

  Routine Description:

    This routine uninitializes the controller hardware.

  Arguments:

    DeviceCtxPtr - a pointer to device context

  Return Value:

    Ntstatus of operation.

--*/
_Use_decl_annotations_
NTSTATUS ControllerUninitialize(PDEVICE_CONTEXT DeviceCtxPtr)
{
    NTSTATUS status = STATUS_SUCCESS;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "++ControllerUninitialize()");
    NT_ASSERT(DeviceCtxPtr != NULL);

    // Uninitialize controller hardware via the pDevice->RegistersPtr->* register interface
    DeviceCtxPtr->RegistersPtr->MCR = (ULONG)0x0;
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "--ControllerUninitialize()=%Xh",
                status);

    return status;
}

/*++

  Routine Description:

    This routine configures and starts the controller
    for a transfer.

  Arguments:

    DeviceCtxPtr - a pointer to the PBC device context
    RequestPtr - a pointer to the PBC request context

  Return Value:

    None. The request is completed asynchronously.

--*/
_Use_decl_annotations_
VOID ControllerConfigureForTransfer(
    PDEVICE_CONTEXT DeviceCtxPtr,
    PPBC_REQUEST RequestPtr
    )
{
    NTSTATUS status = STATUS_SUCCESS;
    BOOLEAN interruptLockAcquired = FALSE;
    BOOLEAN requestDone = FALSE;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerConfigureForTransfer()");

    NT_ASSERT(DeviceCtxPtr  != NULL);
    NT_ASSERT(RequestPtr != NULL);

    // Initialize request context for transfer.

    RequestPtr->Settings = g_TransferSettings[RequestPtr->SequencePosition];
    RequestPtr->Status = STATUS_SUCCESS;

    // Configure hardware for transfer.

    // Initialize controller hardware for a general
    // transfer via the pDevice->RegistersPtr->* register interface.

    if (RequestPtr->SequencePosition == SpbRequestSequencePositionFirst  ||
        RequestPtr->SequencePosition == SpbRequestSequencePositionSingle) {

        if (DeviceCtxPtr->RegistersPtr->MSR & (IMX_LPI2C_MSR_BBF_MASK | IMX_LPI2C_MSR_MBF_MASK)) {
            status = STATUS_UNSUCCESSFUL;
            RequestPtr->Status = status;
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_CTRLR,
                "ControllerConfigureForTransfer() I2C device is unexpectedly busy");
#if (defined(DBG) || defined(DEBUG))
            ControllerDumpRegisters(DeviceCtxPtr);
#endif
            goto ControllerConfigureForTransferDone;
        }

        RequestPtr->StartInTxFifo = 0;
        RequestPtr->StopInTxFifo = 0;
        RequestPtr->StopCommandRequested = FALSE;
        DeviceCtxPtr->WaitingForStopDetect = FALSE;

        // Set data sampling rate SCL frequency
        SetControllerClockDiv(DeviceCtxPtr);

        // Reset the Tx and Rx FIFO
        DeviceCtxPtr->RegistersPtr->MCR |= IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK;

        // Clear out I2C flags
        DeviceCtxPtr->RegistersPtr->MSR = IMX_LPI2C_MSR_DMF_MASK | IMX_LPI2C_MSR_PLTF_MASK
            | IMX_LPI2C_MSR_FEF_MASK | IMX_LPI2C_MSR_ALF_MASK | IMX_LPI2C_MSR_NDF_MASK
            | IMX_LPI2C_MSR_SDF_MASK | IMX_LPI2C_MSR_EPF_MASK;

#if (defined(DBG) || defined(DEBUG))
        ControllerDumpRegisters(DeviceCtxPtr);
#endif
    }

    if (DeviceCtxPtr->WaitingForFirstOrSingleSeqPos) {
        status = STATUS_UNSUCCESSFUL;
        RequestPtr->Status = status;
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_CTRLR,
            "ControllerConfigureForTransfer() This transfer needs to have First or Single position in sequence. Unlock call would help.");
#if (defined(DBG) || defined(DEBUG))
        ControllerDumpRegisters(DeviceCtxPtr);
#endif
        goto ControllerConfigureForTransferDone;
    }

    if (DeviceCtxPtr->UseInterruptMode) {
        // Synchronize access to device context with ISR.
        WdfInterruptAcquireLock(DeviceCtxPtr->InterruptObject);
        interruptLockAcquired = TRUE;
    }

    // Preset Fifo watermrk. Set interrupt mask and clear current status.
    // Save desired interrupt mask.
    if (RequestPtr->Direction == SpbTransferDirectionFromDevice) {
        DeviceCtxPtr->RegistersPtr->MFCR = (DeviceCtxPtr->RegistersPtr->MFCR & ~IMX_LPI2C_MFCR_RXWATER_MASK) |
            IMX_LPI2C_MFCR_RXWATER(DeviceCtxPtr->RxFifoWatermarkMax);
        PbcDeviceSetInterruptMask(DeviceCtxPtr, IMX_LPI2C_MIER_RDIE_MASK | IMX_LPI2C_PERMANENT_INT_MASK);
    }
    else {
        DeviceCtxPtr->RegistersPtr->MFCR = (DeviceCtxPtr->RegistersPtr->MFCR & ~IMX_LPI2C_MFCR_TXWATER_MASK) |
            IMX_LPI2C_MFCR_TXWATER(DeviceCtxPtr->TxFifoWatermarkMax);
        PbcDeviceSetInterruptMask(DeviceCtxPtr, IMX_LPI2C_MIER_TDIE_MASK | IMX_LPI2C_PERMANENT_INT_MASK);
    }
    DeviceCtxPtr->InterruptStatus = 0;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "Controller configured for %s of %Iu bytes %s address 0x%lx "
                "(SPBREQUEST %p, WDFDEVICE %p)",
                RequestPtr->Direction == SpbTransferDirectionFromDevice ? "read" : "write",
                RequestPtr->Length,
                RequestPtr->Direction == SpbTransferDirectionFromDevice ? "from" : "to",
                DeviceCtxPtr->CurrentTargetPtr->Settings.Address,
                RequestPtr->SpbRequest,
                DeviceCtxPtr->WdfDevice);

    // Perform necessary action to begin transfer.
#if (defined(DBG) || defined(DEBUG))
    ControllerDumpRegisters(DeviceCtxPtr);
#endif

    // Perform necessary action to begin transfer.
    // First data byte (slave address + direction)
    UCHAR uchSlaveAddress = (UCHAR)(DeviceCtxPtr->CurrentTargetPtr->Settings.Address << 1);
    uchSlaveAddress = uchSlaveAddress |
        (RequestPtr->Direction == SpbTransferDirectionToDevice ? (UCHAR)0 : (UCHAR)1);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "ControllerConfigureForTransfer() i2c slave addr (first) data byte %02Xh",
        uchSlaveAddress);

    // Put Start command with the first byte, write calling address to data register.
    ControllerRequestForStartAndAddrGen(DeviceCtxPtr, RequestPtr, uchSlaveAddress);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, 
        "ControllerConfigureForTransfer() slave address written");

    /* The third parameter has always false to avoid calling MmMapLockedPagesSpecifyCache 
    in time of too elevated IRQL because of interrupt acquired lock. The following commented out 
    code could be used if data is acquired from buffer at IRQL <= DISPATCH_LEVEL.
        ControllerArrangeNextBytesTransfer(DeviceCtxPtr, RequestPtr, 
        (RequestPtr->Direction == SpbTransferDirectionToDevice)? TRUE : FALSE, 
        &requestDone); */
    status = ControllerArrangeNextBytesTransfer(DeviceCtxPtr, RequestPtr, FALSE, &requestDone);

    if (status != STATUS_SUCCESS) {
        goto ControllerConfigureForTransferDone;
    }

    NT_ASSERT(requestDone == FALSE);
    
    if (DeviceCtxPtr->UseInterruptMode) {
        // Enable controller interrupts 
        ControllerEnableInterrupts(
            DeviceCtxPtr,
            PbcDeviceGetInterruptMask(DeviceCtxPtr));

        WdfInterruptReleaseLock(DeviceCtxPtr->InterruptObject);
    }
    else {
        WdfWorkItemEnqueue(DeviceCtxPtr->WorkItem);
    }

ControllerConfigureForTransferDone:
    if (interruptLockAcquired) {
        WdfInterruptReleaseLock(DeviceCtxPtr->InterruptObject);
    }

    if (status != STATUS_SUCCESS) {
        RequestPtr->Status = status;

        ControllerCompleteTransfer(DeviceCtxPtr, RequestPtr, TRUE);
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerConfigureForTransfer()");
    return;
}

/*++

  Routine Description:

    This routine processes a hardware interrupt. Activities
    include checking for errors and transferring data.

  Arguments:

    DeviceCtxPtr - a pointer to the PBC device context
    RequestPtr - a pointer to the PBC request context
    InterruptStatus - saved interrupt status bits from the ISR.
        These have already been acknowledged and disabled

  Return Value:

    None. The request is completed asynchronously.

--*/
_Use_decl_annotations_
VOID
ControllerProcessInterrupts(
    _In_  PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  PPBC_REQUEST RequestPtr,
    _In_  ULONG InterruptStatus
)
{
    NTSTATUS status;
    BOOLEAN requestDone = FALSE;

    NT_ASSERT(DeviceCtxPtr != NULL);
    NT_ASSERT(RequestPtr != NULL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerProcessInterrupts()");

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "Ready to process interrupts with status 0x%lx for WDFDEVICE %p",
        InterruptStatus,
        DeviceCtxPtr->WdfDevice);

    // Check for Pin Low Timeout (PLTF) and Arbitration Lost (ALF). If so just completes 
    // (with abort) the sequence without any special actions.
    if (InterruptStatus & (IMX_LPI2C_MSR_PLTF_MASK | IMX_LPI2C_MSR_ALF_MASK)) {
        // Update Information counter and store received data for reception
        (void)ControllerProcessAlreadyTransferedBytes(DeviceCtxPtr, RequestPtr);
        if (InterruptStatus & IMX_LPI2C_MSR_PLTF_MASK) {
            RequestPtr->Status = STATUS_IO_TIMEOUT;
        }
        else {
            RequestPtr->Status = STATUS_UNSUCCESSFUL;
        }
        ControllerCompleteTransfer(DeviceCtxPtr, RequestPtr, TRUE);
        goto exit;
    }

    // Check for address and data NACK and FEF.
    if ((InterruptStatus & (IMX_LPI2C_MSR_FEF_MASK | IMX_LPI2C_MSR_NDF_MASK)) ||
        (DeviceCtxPtr->WaitingForStopDetect)) {
        //
        // In case of FIFO error (FEF) and NACK (NDF), the LPI2C should generate STOP automatically.
        // NDF may not occur if last byte was sent, Tx FIFO is empty and slave delayed NACK/ACK using 
        // SCL stretching.
        // In such case the driver must return normaly SUCCESS on the transfer and must detect that 
        // STOP could be generated automatically by LPI2C. Then an ERROR must be returned 
        // on the following transfer.
        //

        // Update Information counter and store received data for reception
        status = ControllerProcessAlreadyTransferedBytes(DeviceCtxPtr, RequestPtr);

        if ((status != STATUS_SUCCESS) && (RequestPtr->Status == STATUS_SUCCESS)) {
            // Complete the transfer and stop processing interrupts.
            RequestPtr->Status = status;
        }

        if (!(DeviceCtxPtr->WaitingForStopDetect) || 
            ((InterruptStatus & IMX_LPI2C_MSR_NDF_MASK) && 
                (RequestPtr->Status != STATUS_NO_SUCH_DEVICE))) {

            if (InterruptStatus & IMX_LPI2C_MSR_NDF_MASK) {
                if (RequestPtr->Information == 0) {
                    // An address NACK indicates that a device is
                    // not present at that address or is not responding.
                    // Set the error status accordingly.
                    RequestPtr->Status = STATUS_NO_SUCH_DEVICE;
                }
                else {
                    // A data NACK is not necessarily an error.
                    // Set the error status to STATUS_SUCCESS and
                    // indicate the number of bytes successfully
                    // transferred in the information field. The
                    // client will determine success or failure of
                    // the IO based on this length.
                    RequestPtr->Status = STATUS_SUCCESS;
                }
            }
        }

        if (!(DeviceCtxPtr->WaitingForStopDetect)) {
            // And if Stop not detected... 
            if ((InterruptStatus & IMX_LPI2C_MSR_SDF_MASK) == 0U) {
                // If NACK but not FEF...
                if ((InterruptStatus & (IMX_LPI2C_MSR_FEF_MASK | IMX_LPI2C_MSR_NDF_MASK)) == IMX_LPI2C_MSR_NDF_MASK) {
                    // Clear FIFOs. There can be additional commands in Tx FIFO
                    ControllerClearFifos(DeviceCtxPtr, RequestPtr);

                    // Force Stop. If Stop was already generated automatically then it causes FEF
                    ControllerRequestForStopGen(DeviceCtxPtr, RequestPtr);
                    DeviceCtxPtr->WaitingForStopDetect = TRUE;
                }
            }
            if (InterruptStatus & (IMX_LPI2C_MSR_FEF_MASK | IMX_LPI2C_MSR_NDF_MASK)) {
                DeviceCtxPtr->WaitingForStopDetect = TRUE;
            }
        }
        else {
            // Complete the transfer and stop processing interrupts.
            ControllerCompleteTransfer(DeviceCtxPtr, RequestPtr, TRUE);
            goto exit;
        }
        

        if (InterruptStatus & IMX_LPI2C_MSR_SDF_MASK) {
            
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_CTRLR,
                "NACK on address 0x%lx (WDFDEVICE %p) - %!STATUS!",
                DeviceCtxPtr->CurrentTargetPtr->Settings.Address,
                DeviceCtxPtr->WdfDevice,
                RequestPtr->Status);

            // Complete the transfer and stop processing
            // interrupts.
            ControllerCompleteTransfer(DeviceCtxPtr, RequestPtr, TRUE);
            goto exit;
        }
        else {
            if (!(DeviceCtxPtr->WaitingForStopDetect)) {
 
                // Clear FIFOs. There can be additional commands in Tx FIFO
                ControllerClearFifos(DeviceCtxPtr, RequestPtr);
                
                // Force Stop. If Stop was already generated automatically then it causes FEF
                ControllerRequestForStopGen(DeviceCtxPtr, RequestPtr);
                DeviceCtxPtr->WaitingForStopDetect = TRUE;

            }
        }
        goto exit;
    }

    // Process Stop Detect
    if (InterruptStatus & IMX_LPI2C_MSR_SDF_MASK) {
        // Update Information counter and store received data for reception
        status = ControllerProcessAlreadyTransferedBytes(DeviceCtxPtr, RequestPtr);
        
        if (status != STATUS_SUCCESS) {
            // Complete the transfer and stop processing interrupts.
            RequestPtr->Status = status;
        }

        // Complete the transfer and stop processing interrupts.
        // Transfer fully completed?
        if ((RequestPtr->Status == STATUS_SUCCESS) && 
            (RequestPtr->Information == RequestPtr->Length)) {
            ControllerCompleteTransfer(DeviceCtxPtr, RequestPtr, FALSE);
        }
        else {
            ControllerCompleteTransfer(DeviceCtxPtr, RequestPtr, TRUE);
        }
        goto exit;
    }

    // Process data transfer and other I2C commands (Start, Stop)
    status = ControllerArrangeNextBytesTransfer(DeviceCtxPtr, RequestPtr, TRUE, &requestDone);

    if (status != STATUS_SUCCESS) {
        // Complete the transfer and stop processing interrupts.
        RequestPtr->Status = status;
        ControllerCompleteTransfer(DeviceCtxPtr, RequestPtr, TRUE);
    }

    // Complete the transfer and stop processing interrupts if the transfer fully completed.
    if (requestDone) {
        ControllerCompleteTransfer(DeviceCtxPtr, RequestPtr, FALSE);
    }

exit:

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerProcessInterrupts()");
}

/*++

  Routine Description:

    This routine checks and call processing hardware interrupt flags settings. 

  Arguments:

    DeviceCtxPtr - a pointer to the PBC device context
    RequestPtr - a pointer to the PBC request context

  Return Value:

    None. The request is completed asynchronously.

--*/
_Use_decl_annotations_
VOID
ControllerPollingMain(
    _In_  WDFWORKITEM WorkItem
)
{
#define IMX_LPI2C_MAX_TIMEOUT_US (5000UL)
    PDEVICE_CONTEXT DeviceCtxPtr;
    PPBC_TARGET TargetPtr;
    PPBC_REQUEST RequestPtr = NULL;
    ULONG mask;
    ULONG stat;
    BOOLEAN bInterruptsProcessed = FALSE;
    BOOLEAN completeRequest = FALSE;
    int delay_us = 20;
    int retriesMax;
    int retries;
    BOOLEAN finishMainWithTimeout = FALSE;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerPollingMain()");

    DeviceCtxPtr = GetDeviceContext((WDFDEVICE)WdfWorkItemGetParentObject(WorkItem));
    NT_ASSERT(DeviceCtxPtr != NULL);

    //delay = (1000 * 1000000) / DeviceCtxPtr->CurrentTargetPtr->Settings.ConnectionSpeed;

    retriesMax = MAX((IMX_LPI2C_MAX_TIMEOUT_US + delay_us - 1) / delay_us, 1);
    retries = retriesMax;

    do {
        // Acquire the device lock.
        WdfSpinLockAcquire(DeviceCtxPtr->Lock);

        mask = PbcDeviceGetInterruptMask(DeviceCtxPtr);
        if (mask == 0) {  // if cancelled

            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
                "PollingMain cancelled with Interrupt mask = 0, current target for WDFDEVICE %p, ",
                DeviceCtxPtr->WdfDevice);
            // Release the device lock.
            WdfSpinLockRelease(DeviceCtxPtr->Lock);
            break;
        }
        stat = ControllerGetInterruptStatus(
            DeviceCtxPtr,
            0xFFFFFFFF);

#if (defined(DBG) || defined(DEBUG))
        ControllerDumpRegisters(DeviceCtxPtr);
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
            "PollingMain Interrupt status 0x%lx, Interrupt mask 0x%lx",
            stat, mask);
#endif

        stat &= mask;

        if (stat == 0) {
            // Release the device lock.
            WdfSpinLockRelease(DeviceCtxPtr->Lock);
            if (--retries) {
                KeStallExecutionProcessor(delay_us);
                continue;
            }
            finishMainWithTimeout = TRUE;
        }

        // CLear timeout retries counter
        retries = retriesMax;

        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
            "PollingMain loop with status 0x%lx (interrupt mask 0x%lx) for WDFDEVICE %p",
            stat, mask,
            DeviceCtxPtr->WdfDevice);

        // Make sure the target and request are
        // still valid.
        TargetPtr = DeviceCtxPtr->CurrentTargetPtr;

        if (TargetPtr == NULL) {
            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
                "PollingMain loop without a valid current target for WDFDEVICE %p, "
                "this should only occur if the request was already cancelled",
                DeviceCtxPtr->WdfDevice);

            // Release the device lock.
            WdfSpinLockRelease(DeviceCtxPtr->Lock);
            break;
        }

        RequestPtr = TargetPtr->CurrentRequestPtr;

        if (RequestPtr == NULL) {
            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
                "PollingMain loop without a valid current request for SPBTARGET %p, "
                "this should only occur if the request was already cancelled",
                TargetPtr->SpbTarget);

            // Release the device lock.
            WdfSpinLockRelease(DeviceCtxPtr->Lock);
            break;
        }

        NT_ASSERT(RequestPtr->SpbRequest != NULL);

        // Acknowledge and process interrupts.
        ControllerAcknowledgeInterrupts(DeviceCtxPtr, stat);

        // Set TIMEOUT in the request's status and complete the transfer
        if (finishMainWithTimeout) {
            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
                "PollingMain loop completes the transfer with STATUS_IO_TIMEOUT");
            RequestPtr->Status = STATUS_IO_TIMEOUT;
            ControllerCompleteTransfer(DeviceCtxPtr, RequestPtr, TRUE);
            ControllerCancel(DeviceCtxPtr);
        }
        else {
            // Process interrupts
            ControllerProcessInterrupts(DeviceCtxPtr, RequestPtr, stat);
        }
        bInterruptsProcessed = TRUE;
        if (RequestPtr->bIoComplete) {
            completeRequest = TRUE;
        }

        // Release the device lock.
        WdfSpinLockRelease(DeviceCtxPtr->Lock);

        // Complete the request if necessary.
        // This must be done outside of the locked code.
        if (bInterruptsProcessed) {
            if (completeRequest) {
                PbcRequestComplete(RequestPtr);
                break;
            }
        }

    } while (1);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerPollingMain()");
}
/*++

  Routine Description:

    This routine completes a data transfer. Unless there are
    more transfers remaining in the sequence, the request is
    completed.

  Arguments:

    DeviceCtxPtr - a pointer to the device context
    RequestPtr - a pointer to the PBC request context
    AbortSequence - specifies whether the driver should abort the
        ongoing sequence or begin the next transfer

  Return Value:

    None. The request is completed asynchronously.

--*/
_Use_decl_annotations_
VOID ControllerCompleteTransfer(
    PDEVICE_CONTEXT DeviceCtxPtr,
    PPBC_REQUEST RequestPtr,
    BOOLEAN AbortSequence)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "++ControllerCompleteTransfer()");

    NT_ASSERT(DeviceCtxPtr != NULL);
    NT_ASSERT(RequestPtr != NULL);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "Transfer (index %lu) %s with %Iu bytes for address 0x%lx "
        "(SPBREQUEST %p)",
        RequestPtr->TransferIndex,
        NT_SUCCESS(RequestPtr->Status) ? "complete" : "error",
        RequestPtr->Information,
        DeviceCtxPtr->CurrentTargetPtr->Settings.Address,
        RequestPtr->SpbRequest);

    // Update request context with information from this transfer.
    RequestPtr->TotalInformation += RequestPtr->Information;
    RequestPtr->Information = 0;
    RequestPtr->BytesAlreadyRequested = 0;
    RequestPtr->StartInTxFifo = 0;
    RequestPtr->StopInTxFifo = 0;
    RequestPtr->StopCommandRequested = FALSE;

    // Check if there are more transfers in the sequence.
    if (FALSE == AbortSequence) {

        RequestPtr->TransferIndex++;
        if (RequestPtr->TransferIndex < RequestPtr->TransferCount) {
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                        "Configure request for the next transfer %u",
                        RequestPtr->TransferIndex);

            // Configure the request for the next transfer.
            RequestPtr->Status = PbcRequestConfigureForIndex(RequestPtr,
                                                            RequestPtr->TransferIndex);
            if (NT_SUCCESS(RequestPtr->Status)) {
                // Configure controller and kick-off next read or write
                PbcRequestDoTransfer(DeviceCtxPtr, RequestPtr);
                goto exit;
            }

        } else {
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                        "No more requests in sequence.");
        }
    }

    // If not already cancelled, unmark request cancellable.

    if (RequestPtr->Status != STATUS_CANCELLED) {
        NTSTATUS cancelStatus;
        cancelStatus = WdfRequestUnmarkCancelable(RequestPtr->SpbRequest);

        if (!NT_SUCCESS(cancelStatus)) {
            // WdfRequestUnmarkCancelable should only fail if the request
            // has already been or is about to be cancelled. If it does fail
            // the request must NOT be completed - the cancel callback will do this.
            NT_ASSERTMSG("WdfRequestUnmarkCancelable should only fail if"
                        " the request has already been or is about to be cancelled",
                        cancelStatus == STATUS_CANCELLED);

            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                        "Failed to unmark SPBREQUEST %p as cancelable - %Xh",
                        RequestPtr->SpbRequest,
                        cancelStatus);

            goto exit;
        }
    }

    // Done or error occurred. Set interrupt mask to 0.
    // Doing this keeps the DPC from re-enabling interrupts.
    PbcDeviceSetInterruptMask(DeviceCtxPtr, 0);

    // Clear the target's current request. This will prevent
    // the request context from being accessed once the request
    // is completed (and the context is invalid).
    DeviceCtxPtr->CurrentTargetPtr->CurrentRequestPtr = NULL;

    // Clear the controller's current target if any of
    //   1. request is type sequence
    //   2. request position is single
    //      (did not come between lock/unlock)
    // Otherwise wait until unlock.
    if ((RequestPtr->Type == SpbRequestTypeSequence) ||
        (RequestPtr->SequencePosition == SpbRequestSequencePositionSingle)) {

        DeviceCtxPtr->CurrentTargetPtr = NULL;
    }

    // Mark the IO complete. Request not completed here.
    RequestPtr->bIoComplete = TRUE;

exit:

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerCompleteTransfer()");
}

/*++

  Routine Description:

    This routine is invoked whenever the controller is to
    be locked for a single target. The request is only completed
    if there is an error configuring the transfer.

  Arguments:

    SpbController - a handle to the framework device object
        representing an SPB controller
    SpbTarget - a handle to the SPBTARGET object
    SpbRequest - a handle to the SPBREQUEST object

  Return Value:

    None. The request is completed synchronously.

--*/
_Use_decl_annotations_
VOID OnControllerLock(
    WDFDEVICE SpbController,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest
    )
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnControllerLock()");

    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(SpbController);
    PPBC_TARGET targetPtr = GetTargetContext(SpbTarget);

    NT_ASSERT(deviceCtxPtr  != NULL);
    NT_ASSERT(targetPtr  != NULL);

    // Acquire the device lock.
    WdfSpinLockAcquire(deviceCtxPtr->Lock);

    // Assign current target.
    NT_ASSERT(deviceCtxPtr->CurrentTargetPtr == NULL);

    deviceCtxPtr->CurrentTargetPtr = targetPtr;

    WdfSpinLockRelease(deviceCtxPtr->Lock);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
                "Controller locked for SPBTARGET %p at address 0x%lx (WDFDEVICE %p)",
                targetPtr->SpbTarget,
                targetPtr->Settings.Address,
                deviceCtxPtr->WdfDevice);

    // Complete lock request.
    SpbRequestComplete(SpbRequest, STATUS_SUCCESS);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnControllerLock()");
    return;
}

/*++

  Routine Description:

    This routine is invoked whenever the controller is to
    be unlocked for a single target. The request is only completed
    if there is an error configuring the transfer.

  Arguments:

    SpbController - a handle to the framework device object
        representing an SPB controller
    SpbTarget - a handle to the SPBTARGET object
    SpbRequest - a handle to the SPBREQUEST object

  Return Value:

    None. The request is completed asynchronously.

--*/
_Use_decl_annotations_
VOID OnControllerUnlock(
    WDFDEVICE SpbController,
    SPBTARGET SpbTarget,
    SPBREQUEST SpbRequest
    )
{
    PDEVICE_CONTEXT deviceCtxPtr = GetDeviceContext(SpbController);
    PPBC_TARGET targetPtr = GetTargetContext(SpbTarget);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "++OnControllerUnlock()");

    NT_ASSERT(deviceCtxPtr != NULL);
    NT_ASSERT(targetPtr != NULL);

    // Acquire the device lock.
    WdfSpinLockAcquire(deviceCtxPtr->Lock);

    // Check if there is an active sequence and if so perform any action 
    // necessary to stop the transfer in process.

    // If the bus is busy then send STOP
    if (deviceCtxPtr->RegistersPtr->MSR & IMX_LPI2C_MSR_BBF_MASK) {
        deviceCtxPtr->RegistersPtr->MTDR = IMX_I2C_TX_CMD_STOP;
    }

    // Remove current target.
    NT_ASSERT(deviceCtxPtr->CurrentTargetPtr == targetPtr);

    deviceCtxPtr->CurrentTargetPtr = NULL;

    WdfSpinLockRelease(deviceCtxPtr->Lock);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER,
                "Controller unlocked for SPBTARGET %p at address 0x%lx (WDFDEVICE %p)",
                targetPtr->SpbTarget,
                targetPtr->Settings.Address,
                deviceCtxPtr->WdfDevice);

    // Complete unlock request.
    SpbRequestComplete(SpbRequest, STATUS_SUCCESS);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "--OnControllerUnlock()");
    return;
}

/*++

  Routine Description:

    This routine clears bot Rx and Tx FIFOs.

  Arguments:

    DeviceCtxPtr - device object
    RequestPtr - a handle to the SPBREQUEST object

  Return Value:

    None.

--*/
_Use_decl_annotations_
VOID ControllerClearFifos(
    PDEVICE_CONTEXT DeviceCtxPtr,
    PPBC_REQUEST RequestPtr
)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerClearFifos()");

    NT_ASSERT(DeviceCtxPtr != NULL);
    NT_ASSERT(RequestPtr != NULL);
    NT_ASSERT(GetTxFifoCount(DeviceCtxPtr) < DeviceCtxPtr->TxFifoSize);

    // Clear both Rx and Tx FIFOs
    DeviceCtxPtr->RegistersPtr->MCR |= IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK;
    RequestPtr->StartInTxFifo = 0;
    RequestPtr->StopInTxFifo = 0;
    RequestPtr->StopCommandRequested = FALSE;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerClearFifos()");
}

/*++

  Routine Description:

    This routine increases Information counter according to truly transfered bytes.
    This can also clear StartInTxFifo and StopInTxFifo variables if there is certainty that
    they were issued from Tx FIFO.

  Arguments:

    DeviceCtxPtr - device object
    RequestPtr - a handle to the SPBREQUEST object
    TxFifoCount - number of commands/data in Tx Fifo

  Return Value:

    None.

--*/
_Use_decl_annotations_
static VOID ControllerUpdateTxInformationByTxFifo(
    PDEVICE_CONTEXT DeviceCtxPtr,
    PPBC_REQUEST RequestPtr,
    size_t TxFifoCount
)
{
    size_t requestedCommandCount;
    size_t remainingTxInFifoCount;
    UNREFERENCED_PARAMETER(DeviceCtxPtr);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerUpdateTxInformationByTxFifo()");

    NT_ASSERT(RequestPtr->Direction == SpbTransferDirectionToDevice);
    requestedCommandCount = RequestPtr->BytesAlreadyRequested + RequestPtr->StartInTxFifo +
        RequestPtr->StopInTxFifo - RequestPtr->Information;
    NT_ASSERT((TxFifoCount <= requestedCommandCount));
    remainingTxInFifoCount = TxFifoCount;
    while (requestedCommandCount && (remainingTxInFifoCount < requestedCommandCount)) {
        if (RequestPtr->StartInTxFifo) {
            // START command already issued from Tx FIFO
            NT_ASSERT(RequestPtr->Information == 0);
            RequestPtr->StartInTxFifo = 0;
            requestedCommandCount--;
            continue;
        }
        if (RequestPtr->Information < RequestPtr->BytesAlreadyRequested) {
            RequestPtr->Information++;
            requestedCommandCount--;
            continue;
        }
        if (RequestPtr->StopInTxFifo) {
            NT_ASSERT(RequestPtr->StopInTxFifo == 1);
            // One issued STOP is enough
            RequestPtr->StopInTxFifo = 0;
            requestedCommandCount--;
            continue;
        }
        break;
    }
    
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerUpdateTxInformationByTxFifo()");
}

/*++

  Routine Description:

    This routine puts Transmit Data commands into Tx FIFO till Tx FIFO is not full.

  Arguments:

    DeviceCtxPtr - device object
    RequestPtr - a handle to the SPBREQUEST object
    RemainingCmdCount - number of empty command/data items in Tx Fifo

  Return Value:

    Ntstatus of operation.

--*/
static NTSTATUS ControllerPutBytesInI2C(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_ PPBC_REQUEST RequestPtr,
    _Inout_ size_t *RemainingCmdCount
)
{
    NTSTATUS status = STATUS_SUCCESS;
    UCHAR uchOneByteToWrite = 0x00;
    size_t remainingCommandsCount;
    size_t remainingToRequest;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerPutBytesInI2C()");

    remainingCommandsCount = *RemainingCmdCount;
    NT_ASSERT(RequestPtr->Length >= RequestPtr->BytesAlreadyRequested);
    remainingToRequest = RequestPtr->Length - RequestPtr->BytesAlreadyRequested;
    while ((remainingCommandsCount) && (remainingToRequest)) {
        // Obtain data byte form transfer buffer
        status = PbcRequestGetByte(RequestPtr, RequestPtr->BytesAlreadyRequested, &uchOneByteToWrite);
        if (!NT_SUCCESS(status)) {
            goto ControllerConfigureForTransferDone;
        }

        // And put it in I2C
        DeviceCtxPtr->RegistersPtr->MTDR = (((ULONG)uchOneByteToWrite) & IMX_LPI2C_MTDR_DATA_MASK) | 
            IMX_I2C_TX_CMD_TX_DATA;
        RequestPtr->BytesAlreadyRequested++;
        remainingCommandsCount--;
        remainingToRequest--;
    }

ControllerConfigureForTransferDone:
    *RemainingCmdCount = remainingCommandsCount;
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerPutBytesInI2C()=%Xh", status);
    return status;
}

/*++

  Routine Description:

    This routine puts Receive Data commands into Tx FIFO till Tx FIFO is not full.

  Arguments:

    DeviceCtxPtr - device object
    RequestPtr - a handle to the SPBREQUEST object

  Return Value:

    Number of remaining command/data positions in Tx FIFO

--*/
static size_t ControllerRequestForRxData(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_ PPBC_REQUEST RequestPtr
)
{
#define IMX_I2C_MAX_REQUEST_FOR_RX_DATA ((ULONG)256U)  // must be 1 to 256
    size_t remainingCommandsCount;
    size_t remainingToRequest;
    
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerRequestForRxData()");

    NT_ASSERT(DeviceCtxPtr->TxFifoSize >= GetTxFifoCount(DeviceCtxPtr));
    remainingCommandsCount = DeviceCtxPtr->TxFifoSize - GetTxFifoCount(DeviceCtxPtr);
    while (remainingCommandsCount > 0) {
        NT_ASSERT(RequestPtr->Length >= RequestPtr->BytesAlreadyRequested);
        remainingToRequest = RequestPtr->Length - RequestPtr->BytesAlreadyRequested;
        if ((remainingToRequest == 0) || 
            ((RequestPtr->BytesAlreadyRequested - RequestPtr->Information) >= IMX_I2C_MAX_REQUEST_FOR_RX_DATA)) {
                break;
        }
        if (remainingToRequest > IMX_I2C_MAX_REQUEST_FOR_RX_DATA) {
            RequestPtr->BytesAlreadyRequested += IMX_I2C_MAX_REQUEST_FOR_RX_DATA;
            DeviceCtxPtr->RegistersPtr->MTDR = (((ULONG)(IMX_I2C_MAX_REQUEST_FOR_RX_DATA - 1)) & 
                IMX_LPI2C_MTDR_DATA_MASK) | IMX_I2C_TX_CMD_RX_DATA;
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, 
                "ControllerRequestForRxData() IMX_I2C_TX_CMD_RX_DATA command put with 0x%lx", 
                (IMX_I2C_MAX_REQUEST_FOR_RX_DATA - 1));
        }
        else {
            RequestPtr->BytesAlreadyRequested += remainingToRequest;
            NT_ASSERT(RequestPtr->Length >= RequestPtr->BytesAlreadyRequested);
            DeviceCtxPtr->RegistersPtr->MTDR = (((ULONG)(remainingToRequest - 1)) & 
                IMX_LPI2C_MTDR_DATA_MASK) | IMX_I2C_TX_CMD_RX_DATA;
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                "ControllerRequestForRxData() IMX_I2C_TX_CMD_RX_DATA command put with 0x%llx",
                remainingToRequest - 1);
        }
        remainingCommandsCount--;
        // Only one RX_DATA command
        break;
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerRequestForRxData()=%llu remaining items in Tx Fifo",
        remainingCommandsCount);
    return remainingCommandsCount;
}

/*++

  Routine Description:

    This routine gets received bytes from Rx FIFO.

  Arguments:

    DeviceCtxPtr - device object
    RequestPtr - a handle to the SPBREQUEST object

  Return Value:

    Ntstatus of operation.

--*/
static NTSTATUS ControllerGetBytesFromI2C(
    _In_ PDEVICE_CONTEXT DeviceCtxPtr,
    _In_ PPBC_REQUEST RequestPtr
)
{
    NTSTATUS status = STATUS_SUCCESS;
    UCHAR uchOneByte = 0x00;
    size_t remainingRxBytesInRxFifo;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerGetBytesFromI2C()");

    remainingRxBytesInRxFifo = GetRxFifoCount(DeviceCtxPtr);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
        "ControllerGetBytesFromI2C() %llu bytes in Rx Fifo",
        remainingRxBytesInRxFifo);

    while ((remainingRxBytesInRxFifo) && (RequestPtr->Information < RequestPtr->Length)) {
        // Obtain data byte from I2C
        uchOneByte = DeviceCtxPtr->RegistersPtr->MRDR & 0xFF;

        // And put in transfer buffer 
        status = PbcRequestSetByte(RequestPtr, RequestPtr->Information, uchOneByte);
        if (!NT_SUCCESS(status)) {
            goto ControllerGetBytesFromI2CDone;
        }
        RequestPtr->Information++;
        remainingRxBytesInRxFifo--;
    }

    if ((remainingRxBytesInRxFifo) && (RequestPtr->Information == RequestPtr->Length)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_CTRLR, "ControllerGetBytesFromI2C() Unrequested %Iu bytes in Rx Fifo despite all data already received!", remainingRxBytesInRxFifo);
    }

ControllerGetBytesFromI2CDone:
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerGetBytesFromI2C()=%Xh", 
        status);
    return status;
}

/*++

  Routine Description:

    This routine puts START command with slave address and R/W into Tx FIFO. 
    It does not check whether Tx FIFO is not full.

  Arguments:

    DeviceCtxPtr - device object
    RequestPtr - a handle to the SPBREQUEST object

  Return Value:

    None.

--*/
_Use_decl_annotations_
static VOID ControllerRequestForStartAndAddrGen(
    PDEVICE_CONTEXT DeviceCtxPtr,
    PPBC_REQUEST RequestPtr,
    UCHAR SlaveAddress
)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerRequestForStartAndAddrGen()");

    NT_ASSERT(DeviceCtxPtr != NULL);
    NT_ASSERT(RequestPtr != NULL);

    // Because of ControllerUpdateTxInformationByTxFifo's algorithm
    NT_ASSERT(RequestPtr->StopInTxFifo == 0);

    // Put START command with slave address in Tx FIFO
    DeviceCtxPtr->RegistersPtr->MTDR = SlaveAddress | IMX_I2C_TX_CMD_START;
    RequestPtr->StartInTxFifo++;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerRequestForStartAndAddrGen()");
}

/*++

  Routine Description:

    This routine puts STOP command into Tx FIFO. It does not check whether Tx FIFO is not full.

  Arguments:

    DeviceCtxPtr - device object
    RequestPtr - a handle to the SPBREQUEST object

  Return Value:

    None.

--*/
_Use_decl_annotations_
VOID ControllerRequestForStopGen(
    PDEVICE_CONTEXT DeviceCtxPtr,
    PPBC_REQUEST RequestPtr
)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerRequestForStopGen()");

    NT_ASSERT(DeviceCtxPtr != NULL);
    NT_ASSERT(RequestPtr != NULL);

    // Put STOP command in Tx FIFO

    DeviceCtxPtr->RegistersPtr->MTDR = IMX_I2C_TX_CMD_STOP;
    RequestPtr->StopInTxFifo++;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerRequestForStopGen()");
}

/*++

  Routine Description:

    This routine cancels I2C communication. It clears FIFOs and request Stop generation.

  Arguments:

    DeviceCtxPtr - device object

  Return Value:
    
    None.

--*/
_Use_decl_annotations_
VOID ControllerCancel(
    PDEVICE_CONTEXT DeviceCtxPtr
)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerCancel()");

    NT_ASSERT(DeviceCtxPtr != NULL);

    // Clear both Rx and Tx FIFOs
    DeviceCtxPtr->RegistersPtr->MCR |= IMX_LPI2C_MCR_RRF_MASK | IMX_LPI2C_MCR_RTF_MASK;

    // Force Stop
    DeviceCtxPtr->RegistersPtr->MTDR = IMX_I2C_TX_CMD_STOP;

    // Next transfer needs to have First or Single position in sequence
    DeviceCtxPtr->WaitingForFirstOrSingleSeqPos = TRUE;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerCancel()");
}

/*++

  Routine Description:

    This routine update Information for transmit or store already received data for reception.

  Arguments:

    DeviceCtxPtr - device object
    RequestPtr - a handle to the SPBREQUEST object

  Return Value:

      Ntstatus of operation.

--*/
_Use_decl_annotations_
static NTSTATUS ControllerProcessAlreadyTransferedBytes(
    PDEVICE_CONTEXT DeviceCtxPtr,
    PPBC_REQUEST RequestPtr
)
{
    NTSTATUS status = STATUS_SUCCESS;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerProcessAlreadyTransferedBytes()");

    NT_ASSERT(DeviceCtxPtr != NULL);
    NT_ASSERT(RequestPtr != NULL);
    if (RequestPtr->Direction == SpbTransferDirectionToDevice) {
        // TX
        // Update Information counter
        ControllerUpdateTxInformationByTxFifo(DeviceCtxPtr, RequestPtr,
            GetTxFifoCount(DeviceCtxPtr));
    }
    else {
        // RX
        // Get received bytes
        status = ControllerGetBytesFromI2C(DeviceCtxPtr, RequestPtr);
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerProcessAlreadyTransferedBytes()=%Xh", status);
    return status;
}

/*++

  Routine Description:

    This routine transfers data to or from the device.

  Arguments:

    DeviceCtxPtr - device object
    RequestPtr - a handle to the SPBREQUEST object
    PutGetBytesInFromI2C - flag that put/get bytes from transfer buffer/I2C in I2C/transfer buffer
    RequestDone - indicates whether the request has been already processed by I2C and can be completed

  Return Value:

    Ntstatus of operation.

--*/
_Use_decl_annotations_
static NTSTATUS ControllerArrangeNextBytesTransfer(
    PDEVICE_CONTEXT DeviceCtxPtr,
    PPBC_REQUEST RequestPtr,
    BOOLEAN PutGetBytesInFromI2C,
    BOOLEAN *RequestDone
)
{
    NTSTATUS status = STATUS_SUCCESS;
    size_t txFifoCount;
    size_t remainingCommandCount;
    BOOLEAN minimizeTxWatermark = FALSE;
    ULONG newTxWatermark = 0;
    size_t remainingRxBytes;
    BOOLEAN reqDone = FALSE;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerArrangeNextBytesTransfer()");

    if (RequestPtr->Direction == SpbTransferDirectionToDevice) {
        // TX
        // Get current Tx FIFO counter

        txFifoCount = GetTxFifoCount(DeviceCtxPtr);
        NT_ASSERT(DeviceCtxPtr->TxFifoSize >= txFifoCount);

        // Update Information counter
        ControllerUpdateTxInformationByTxFifo(DeviceCtxPtr, RequestPtr, txFifoCount);

        remainingCommandCount = DeviceCtxPtr->TxFifoSize - txFifoCount;

        if (PutGetBytesInFromI2C) {
            // Enter Tx Data commands in Tx FIFO
            status = ControllerPutBytesInI2C(DeviceCtxPtr, RequestPtr, &remainingCommandCount);

            if (status != STATUS_SUCCESS) {
                goto ControllerArrangeNextBytesTransferDone;
            }
        }

        if (RequestPtr->BytesAlreadyRequested < RequestPtr->Length) {
            // Continue transmitting
            goto ControllerArrangeNextBytesTransferDone;
        }
        
        // Now RequestPtr->BytesAlreadyRequested == RequestPtr->Length
        if (RequestPtr->Settings.IsEnd) {
            if (!RequestPtr->StopCommandRequested) {
                if (remainingCommandCount) {
                    // Enter STOP command in Tx FIFO
                    ControllerRequestForStopGen(DeviceCtxPtr, RequestPtr);
                    RequestPtr->StopCommandRequested = TRUE;
                    minimizeTxWatermark = TRUE;
                    newTxWatermark = 1;
                }
                else {
                    // Wait for at least one free item in Tx Fifo in order to put the STOP
                    goto ControllerArrangeNextBytesTransferDone;
                }
            }
        }
        else {
            minimizeTxWatermark = TRUE;
            newTxWatermark = 0;
        }

        if (RequestPtr->Information < RequestPtr->Length) {
            if (minimizeTxWatermark) {
                // Reconfigure Tx Watermark because of updating the Information counter 
                DeviceCtxPtr->RegistersPtr->MFCR = (DeviceCtxPtr->RegistersPtr->MFCR & ~IMX_LPI2C_MFCR_TXWATER_MASK) |
                    IMX_LPI2C_MFCR_TXWATER(newTxWatermark);
            }

            // Continue transmitting
            goto ControllerArrangeNextBytesTransferDone;
        }

        // Now RequestPtr->Information == RequestPtr->Length
#if (defined(DBG) || defined(DEBUG))
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
            "ControllerArrangeNextBytesTransfer() MIER_TDIE interrupt switched off. InterruptMask was 0x%llx before",
            DeviceCtxPtr->InterruptMask);
#endif
        // Disable transmit data interrupt
        PbcDeviceAndInterruptMask(DeviceCtxPtr, ~IMX_LPI2C_MIER_TDIE_MASK);

        // Return that the request is done and can be completed
        if (!RequestPtr->Settings.IsEnd) {
            reqDone = TRUE;
        }

    }
    else {
        // RX
        if (PutGetBytesInFromI2C) {
            // Get received bytes
            status = ControllerGetBytesFromI2C(DeviceCtxPtr, RequestPtr);

            if (status != STATUS_SUCCESS) {
                goto ControllerArrangeNextBytesTransferDone;
            }
        }

        // Enter Rx Data commands in Tx FIFO
        remainingCommandCount = ControllerRequestForRxData(DeviceCtxPtr, RequestPtr);

        if (RequestPtr->BytesAlreadyRequested < RequestPtr->Length) {
            // Continue transmitting
            goto ControllerArrangeNextBytesTransferDone;
        }

        // Now RequestPtr->BytesAlreadyRequested == RequestPtr->Length
        if (RequestPtr->Information < RequestPtr->Length) {
            remainingRxBytes = RequestPtr->Length - RequestPtr->Information;
            if (remainingRxBytes <= DeviceCtxPtr->RxFifoWatermarkMax) {
                // Reconfigure Rx Watermark because of updating the Information counter 
                DeviceCtxPtr->RegistersPtr->MFCR = (DeviceCtxPtr->RegistersPtr->MFCR & ~IMX_LPI2C_MFCR_RXWATER_MASK) |
                    IMX_LPI2C_MFCR_RXWATER(remainingRxBytes - 1);
#if (defined(DBG) || defined(DEBUG))
                TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR,
                    "ControllerArrangeNextBytesTransfer() Reconfigure Tx watermark to 0x%llx",
                    remainingRxBytes - 1);
#endif
            }

            // Continue receiving
            goto ControllerArrangeNextBytesTransferDone;
        }

        // Now RequestPtr->Information == RequestPtr->Length
        if (RequestPtr->Settings.IsEnd) {
            if (!RequestPtr->StopCommandRequested) {
                if (remainingCommandCount) {
                    // Enter STOP command in Tx FIFO
                    ControllerRequestForStopGen(DeviceCtxPtr, RequestPtr);
                    RequestPtr->StopCommandRequested = TRUE;
                }
                else {
                    // Wait for at least one free item in Tx Fifo in order to put the STOP
                    goto ControllerArrangeNextBytesTransferDone;
                }
            }
        }

#if (defined(DBG) || defined(DEBUG))
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, 
            "ControllerArrangeNextBytesTransfer() MIER_RDIE interrupt switched off. InterruptMask was 0x%llx before",
            DeviceCtxPtr->InterruptMask);
#endif
        // Disable receive data interrupt
        PbcDeviceAndInterruptMask(DeviceCtxPtr, ~IMX_LPI2C_MIER_RDIE_MASK);

        // Return that the request is done and can be completed
        if (!RequestPtr->Settings.IsEnd) {
            reqDone = TRUE;
        }

    }

ControllerArrangeNextBytesTransferDone:
    if (RequestDone != NULL) {
        if (reqDone) {
            // Current request has been already processed by I2C and can be completed
            *RequestDone = TRUE;
        }
        else {
            // Current request has not been processed by I2C yet and cannot be completed
            *RequestDone = FALSE;
        }
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerArrangeNextBytesTransfer()=%Xh", status);
    return status;
}

/*++

  Routine Description:

    This routine enables the hardware interrupts for the
    specificed mask.

  Arguments:

    DeviceCtxPtr - a pointer to the PBC device context
    InterruptMask - interrupt bits to enable

  Return Value:

    None.

--*/
VOID
ControllerEnableInterrupts(
    _In_  PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  ULONG InterruptMask
)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerEnableInterrupts()");

    NT_ASSERT(DeviceCtxPtr != NULL);

    TraceEvents(TRACE_LEVEL_VERBOSE, TRACE_CTRLR,
        "Enable interrupts with mask 0x%lx (WDFDEVICE %p)",
        InterruptMask,
        DeviceCtxPtr->WdfDevice);

    // Enable interrupts as requested
    DeviceCtxPtr->RegistersPtr->MIER = InterruptMask;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerEnableInterrupts()");
}

/*++

  Routine Description:

    This routine disables all controller interrupts.

  Arguments:

    DeviceCtxPtr - a pointer to the PBC device context

  Return Value:

    None.

--*/
VOID
ControllerDisableInterrupts(
    _In_  PDEVICE_CONTEXT DeviceCtxPtr
)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerDisableInterrupts()");

    NT_ASSERT(DeviceCtxPtr != NULL);

    // Disable all interrupts.
    DeviceCtxPtr->RegistersPtr->MIER = 0;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerDisableInterrupts()");
}

/*++

  Routine Description:

    This routine gets the interrupt status of the
    specificed interrupt bits.

  Arguments:

    DeviceCtxPtr - a pointer to the PBC device context
    InterruptMask - interrupt bits to check

  Return Value:

    A bitmap indicating which interrupts are set.

--*/
ULONG
ControllerGetInterruptStatus(
    _In_  PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  ULONG InterruptMask
)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerGetInterruptStatus()");

    ULONG interruptStatus = 0;

    NT_ASSERT(DeviceCtxPtr != NULL);

    // Check if any of the interrupt mask bits have triggered an interrupt.
    interruptStatus = (DeviceCtxPtr->RegistersPtr->MSR & InterruptMask);

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerGetInterruptStatus()");

    return interruptStatus;
}

/*++

  Routine Description:

    This routine acknowledges the
    specificed interrupt bits.

  Arguments:

    DeviceCtxPtr - a pointer to the PBC device context
    InterruptMask - interrupt bits to acknowledge

  Return Value:

    None.

--*/
VOID
ControllerAcknowledgeInterrupts(
    _In_  PDEVICE_CONTEXT DeviceCtxPtr,
    _In_  ULONG InterruptMask
)
{
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "++ControllerAcknowledgeInterrupts()");

    NT_ASSERT(DeviceCtxPtr != NULL);

    DeviceCtxPtr->RegistersPtr->MSR = InterruptMask;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_CTRLR, "--ControllerAcknowledgeInterrupts()");
}
