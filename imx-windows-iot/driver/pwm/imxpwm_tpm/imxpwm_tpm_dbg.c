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

#if DBG
LARGE_INTEGER        DriverStartTime;

void DbgTpmDumpReg(const char* CallerName, TPM_t* pRegs, UINT32 RegAddress) {
    UNREFERENCED_PARAMETER(CallerName);
    UNREFERENCED_PARAMETER(pRegs);

    switch (RegAddress) {
    case TPM_VERID:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (Mayor: %d, Minor: %d, feature:%d)", CallerName, "TPM_VERID", TPM_VERID, pRegs->VERID.R,
            pRegs->VERID.B.MAJOR, pRegs->VERID.B.MINOR, pRegs->VERID.B.FEATURE);
        break;
    case TPM_PARAM:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (CntWidth: %d, TrigCnt: %d, ChCnt: %d)", CallerName, "TPM_PARAM", TPM_PARAM, pRegs->PARAM.R,
            pRegs->PARAM.B.WIDTH, pRegs->PARAM.B.TRIG, pRegs->PARAM.B.CHAN);
        break;
    case TPM_GLOBAL:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (SwRst: %d, NoUpdate: %d)", CallerName, "TPM_GLOBAL", TPM_GLOBAL, pRegs->GLOBAL.R,
            pRegs->GLOBAL.B.RST, pRegs->GLOBAL.B.NOUPDATE);
        break;
    case TPM_SC:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (DmaEn: %d, Tof: %d, TofIntEn: %d, CenterAlignedPwm: %d, ClkMode: %s, Ps: %d)", CallerName, "TPM_SC", TPM_SC, pRegs->SC.R,
            pRegs->SC.B.DMA, pRegs->SC.B.TOF, pRegs->SC.B.TOIE, pRegs->SC.B.CPWMS,
            (pRegs->SC.B.CMOD == 0) ? "TPM disabled" : (pRegs->SC.B.CMOD == 1) ? "TPM clock" : (pRegs->SC.B.CMOD == 2) ? "EXTCLK" : "ExtTrigger",
            (1 << pRegs->SC.B.PS));
        break;
    case TPM_CNT:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (Counter: %d)", CallerName, "TPM_CNT", TPM_CNT, pRegs->CNT.R, pRegs->CNT.R);
        break;
    case TPM_MOD:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (Modulo:  %d)", CallerName, "TPM_MOD", TPM_MOD, pRegs->MOD.R, pRegs->MOD.R);
        break;
    case TPM_STATUS:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (Tof: %d, Ch3f: %d, Ch2f: %d, Ch1f: %d, Ch0f: %d)", CallerName, "TPM_STATUS", TPM_STATUS, pRegs->STATUS.R,
            pRegs->STATUS.B.TOF, pRegs->STATUS.B.CH3, pRegs->STATUS.B.CH2, pRegs->STATUS.B.CH1, pRegs->STATUS.B.CH0);
        break;
    case TPM_C0V:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (Compare: %d)", CallerName, "TPM_C0V", TPM_C0V, pRegs->C0V.R, pRegs->C0V.R);
        break;
    case TPM_C1V:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (Compare: %d)", CallerName, "TPM_C1V", TPM_C0V, pRegs->C1V.R, pRegs->C1V.R);
        break;
    case TPM_C2V:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (Compare: %d)", CallerName, "TPM_C2V", TPM_C0V, pRegs->C2V.R, pRegs->C2V.R);
        break;
    case TPM_C3V:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (Compare: %d)", CallerName, "TPM_C3V", TPM_C0V, pRegs->C3V.R, pRegs->C3V.R);
        break;
    case TPM_C0SC:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (ChnIntFlag: %d, ChnIntEn: %d Ms: %d, Els: %d DmaEn: %d)", CallerName, "TPM_C0SC", TPM_C0SC, pRegs->C0SC.R,
            pRegs->C0SC.B.CHF, pRegs->C0SC.B.CHIE, ((pRegs->C0SC.R & (TPM_C0SC_MSB_MASK | TPM_C0SC_MSA_MASK)) >> TPM_C0SC_MSA_SHIFT), ((pRegs->C0SC.R & (TPM_C0SC_ELSB_MASK | TPM_C0SC_ELSA_MASK)) >> TPM_C0SC_ELSA_SHIFT), pRegs->C0SC.B.DMA);
        break;
    case TPM_C1SC:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (ChnIntFlag: %d, ChnIntEn: %d Ms: %d, Els: %d DmaEn: %d)", CallerName, "TPM_C1SC", TPM_C1SC, pRegs->C1SC.R,
            pRegs->C1SC.B.CHF, pRegs->C1SC.B.CHIE, ((pRegs->C1SC.R & (TPM_C0SC_MSB_MASK | TPM_C0SC_MSA_MASK)) >> TPM_C0SC_MSA_SHIFT), ((pRegs->C1SC.R & (TPM_C0SC_ELSB_MASK | TPM_C0SC_ELSA_MASK)) >> TPM_C0SC_ELSA_SHIFT), pRegs->C1SC.B.DMA);
        break;
    case TPM_C2SC:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (ChnIntFlag: %d, ChnIntEn: %d Ms: %d, Els: %d DmaEn: %d)", CallerName, "TPM_C2SC", TPM_C2SC, pRegs->C2SC.R,
            pRegs->C2SC.B.CHF, pRegs->C2SC.B.CHIE, ((pRegs->C2SC.R & (TPM_C0SC_MSB_MASK | TPM_C0SC_MSA_MASK)) >> TPM_C0SC_MSA_SHIFT), ((pRegs->C2SC.R & (TPM_C0SC_ELSB_MASK | TPM_C0SC_ELSA_MASK)) >> TPM_C0SC_ELSA_SHIFT), pRegs->C2SC.B.DMA);
        break;
    case TPM_C3SC:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (ChnIntFlag: %d, ChnIntEn: %d Ms: %d, Els: %d DmaEn: %d)", CallerName, "TPM_C3SC", TPM_C3SC, pRegs->C3SC.R,
            pRegs->C3SC.B.CHF, pRegs->C3SC.B.CHIE, ((pRegs->C3SC.R & (TPM_C0SC_MSB_MASK | TPM_C0SC_MSA_MASK)) >> TPM_C0SC_MSA_SHIFT), ((pRegs->C3SC.R & (TPM_C0SC_ELSB_MASK | TPM_C0SC_ELSA_MASK)) >> TPM_C0SC_ELSA_SHIFT), pRegs->C3SC.B.DMA);
        break;
    case TPM_COMBINE:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (COMSWAP1: %d, COMBINE1: %d, COMSWAP0: %d, COMBINE0: %d)", CallerName, "TPM_COMBINE", TPM_COMBINE, pRegs->COMBINE.R,
            pRegs->COMBINE.B.COMSWAP1, pRegs->COMBINE.B.COMBINE1, pRegs->COMBINE.B.COMSWAP0, pRegs->COMBINE.B.COMBINE0);
        break;
    case TPM_TRIG:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (TRIG3: %d, TRIG2: %d, TRIG1: %d, TRIG0: %d)", CallerName, "TPM_TRIG", TPM_TRIG, pRegs->TRIG.R,
            pRegs->TRIG.B.TRIG3, pRegs->TRIG.B.TRIG2, pRegs->TRIG.B.TRIG1, pRegs->TRIG.B.TRIG0);
        break;
    case TPM_POL:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X (POL3: %d, POL2: %d, POL1: %d, POL0: %d)", CallerName, "TPM_POL", TPM_POL, pRegs->POL.R,
            pRegs->POL.B.POL3, pRegs->POL.B.POL2, pRegs->POL.B.POL1, pRegs->POL.B.POL0);
        break;
    case TPM_FILTER:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X ()", CallerName, "TPM_FILTER", TPM_FILTER, pRegs->FILTER.R);
        break;
    case TPM_QDCTRL:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X ()", CallerName, "TPM_QDCTRL", TPM_QDCTRL, pRegs->QDCTRL.R);
        break;
    case TPM_CONF:
        DBG_DEV_PRINT_INFO("%-10s %-15s(0x%02X), Value: 0x%04X ()", CallerName, "TPM_CONF", TPM_CONF, pRegs->CONF.R);
        break;
    default:
        DBG_DEV_PRINT_INFO("Unkown register, offset: 0x%04X", RegAddress);
    }
}

_Use_decl_annotations_
void DbgTpmDumpRegs(const IMXPWM_DEVICE_CONTEXT* pDevContext) {
    TPM_t* pRegs = pDevContext->pRegs;
    UNREFERENCED_PARAMETER(pRegs);

    DbgTpmDumpReg("XXX", pRegs, TPM_VERID);
    DbgTpmDumpReg("XXX", pRegs, TPM_VERID);
    DbgTpmDumpReg("XXX", pRegs, TPM_PARAM);
    DbgTpmDumpReg("XXX", pRegs, TPM_GLOBAL);
    DbgTpmDumpReg("XXX", pRegs, TPM_SC);
    DbgTpmDumpReg("XXX", pRegs, TPM_STATUS);
    DbgTpmDumpReg("XXX", pRegs, TPM_C0SC);
    DbgTpmDumpReg("XXX", pRegs, TPM_C1SC);
    DbgTpmDumpReg("XXX", pRegs, TPM_C2SC);
    DbgTpmDumpReg("XXX", pRegs, TPM_C3SC);
    DbgTpmDumpReg("XXX", pRegs, TPM_CNT);
    DbgTpmDumpReg("XXX", pRegs, TPM_MOD);
    DbgTpmDumpReg("XXX", pRegs, TPM_C0V);
    DbgTpmDumpReg("XXX", pRegs, TPM_C1V);
    DbgTpmDumpReg("XXX", pRegs, TPM_C2V);
    DbgTpmDumpReg("XXX", pRegs, TPM_C3V);
    DbgTpmDumpReg("XXX", pRegs, TPM_COMBINE);
    DbgTpmDumpReg("XXX", pRegs, TPM_TRIG);
    DbgTpmDumpReg("XXX", pRegs, TPM_POL);
    DbgTpmDumpReg("XXX", pRegs, TPM_FILTER);
    DbgTpmDumpReg("XXX", pRegs, TPM_QDCTRL);
    DbgTpmDumpReg("XXX", pRegs, TPM_CONF);
}

#undef MAKECASE
#undef MAKECASE1
#undef MAKEDEFAULT
#define MAKECASE(Value) case Value:  return #Value;
#define MAKECASE1(Value,Name) case Value:  return #Name;
#define MAKEDEFAULT(Message) default: return"!!! "Message" name unknown !!!";

const char* Dbg_GetIOCTLName(ULONG i) {
    switch (i) {
        MAKECASE(IOCTL_PWM_CONTROLLER_GET_INFO)
            MAKECASE(IOCTL_PWM_CONTROLLER_GET_ACTUAL_PERIOD)
            MAKECASE(IOCTL_PWM_CONTROLLER_SET_DESIRED_PERIOD)
            MAKECASE(IOCTL_PWM_PIN_GET_ACTIVE_DUTY_CYCLE_PERCENTAGE)
            MAKECASE(IOCTL_PWM_PIN_SET_ACTIVE_DUTY_CYCLE_PERCENTAGE)
            MAKECASE(IOCTL_PWM_PIN_GET_POLARITY)
            MAKECASE(IOCTL_PWM_PIN_SET_POLARITY)
            MAKECASE(IOCTL_PWM_PIN_START)
            MAKECASE(IOCTL_PWM_PIN_STOP)
            MAKECASE(IOCTL_PWM_PIN_IS_STARTED)
            MAKEDEFAULT("Unknown IOCTL")
    }
}

#endif