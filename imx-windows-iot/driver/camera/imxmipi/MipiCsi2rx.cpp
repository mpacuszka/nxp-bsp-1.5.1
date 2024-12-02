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

#include "imxmipi.h"
#ifdef _ARM64_
#include <ImxCpuRev.h>
#endif
#include "MipiCsi2rx.tmh"

#define MIPI_PLL_TIMEOUT 1000000
#define MIPI_FENCE_TIMEOUT 10000000

// i.MX 8MQ

NTSTATUS MipiCsi2_t::Phy8mqAssertReset()
/*!
 * Assert reset of MIPI D-Phy.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;

    status = m_ResourcePtr->AcpiSetMipiRcr(1);
    return status;
}

NTSTATUS MipiCsi2_t::Phy8mqDeassertReset()
/*!
 * De-assert reset of MIPI D-Phy.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;

    status = m_ResourcePtr->AcpiSetMipiRcr(0);
    return status;
}

NTSTATUS MipiCsi2_t::Phy8mqEnable(const camera_config_t& Config, UINT8 ThsSettleEscClk)
/*!
 * Configures Hs Settle delay and enables Mipi D-Phy.
 *
 * @param Config Information about video stream being processed (resolution, frame rate, color format ..).
 * @param ThsSettleEscClk Hs Settle delay as number of Esc clock periods.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    UINT32 GPR34;

    status = m_ResourcePtr->AcpiRgpr(GPR34);
    if (NT_SUCCESS(status)) {
        GPR34 = (GPR34 & ~0x3FFF)
            | MIPI_GPR_REGS::GPR34_RX_ENABLE
            | MIPI_GPR_REGS::GPR34_VID_INTFC_ENB
            | (Config.ContClkMode ? MIPI_GPR_REGS::GPR34_CONT_CLK_MODE : 0)
            | MIPI_GPR_REGS::GPR34_S_PRG_RXHS_SETTLE(ThsSettleEscClk - 1UL)
            | MIPI_GPR_REGS::GPR34_RX_RCAL_HIGHER;

        UNREFERENCED_PARAMETER(ThsSettleEscClk);
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "MipiCsi2_t::EnablePhy (settle = 0x%x)", (unsigned)ThsSettleEscClk);
        status = m_ResourcePtr->AcpiWgpr(GPR34);
    }
    return status;
}

NTSTATUS MipiCsi2_t::Phy8mqDisable()
/*!
 * Disables MIPI D-Phy.
 */
{
    NTSTATUS status;
    UINT32 GPR34;

    status = m_ResourcePtr->AcpiRgpr(GPR34);
    if (NT_SUCCESS(status)) {
        GPR34 = (GPR34 & ~0x3FFF)
            | MIPI_GPR_REGS::GPR34_PD_RX; // Power down D-PHY

        status = m_ResourcePtr->AcpiWgpr(GPR34);
        if (NT_SUCCESS(status)) {
            status = PhyDeassertReset();
        }
    }
    return status;
}

// i.MX 8X

NTSTATUS MipiCsi2_t::Phy8xEnable(const camera_config_t& Config, UINT8 ThsSettleEscClk)
/*!
 * Configures Hs Settle delay and enables Mipi D-Phy.
 *
 * @param Config Information about video stream being processed (resolution, frame rate, color format ..).
 * @param ThsSettleEscClk Hs Settle delay as number of Esc clock periods.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    ASSERT(m_MipiCsrRegistersPtr != NULL);
    {
        UINT32 plm_status = 1U;
        UINT32 timeout = MIPI_PLL_TIMEOUT;
        
        m_MipiCsrRegistersPtr->PHY_FENCE_CTRL = 0xF;
        m_MipiCsrRegistersPtr->CTRL_CLK_RESET_CTRL = (m_MipiCsrRegistersPtr->CTRL_CLK_RESET_CTRL_SW_RESETN_BIT | m_MipiCsrRegistersPtr->CTRL_CLK_RESET_CTRL_CTL_CLK_OFF_BIT); // CSR setup - assert reset and connect clock to LPCG
        
        // Wait for plm clock
        while (plm_status != 0U) {
            plm_status = (m_MipiCsrRegistersPtr->PLM_CTRL & m_MipiCsrRegistersPtr->PLM_PL_CLK_RUN_BIT);
            if (--timeout == 0) {
                return status = STATUS_TIMEOUT;
            }
        }
    }
    m_MipiCsrRegistersPtr->VC_INTERLACED = 0U;
    m_MipiCsrRegistersPtr->DATA_TYPE_DISABLE = 0U;
    m_MipiCsrRegistersPtr->YUV420_FIRST_LINE_TYPE = 0U;

    m_MipiCsrRegistersPtr->PHY_CTRL |= (m_MipiCsrRegistersPtr->PHY_CTRL_RX_ENABLE_BIT |
                                        m_MipiCsrRegistersPtr->PHY_CTRL_DDRCLK_EN_BIT |
                                        (Config.ContClkMode?m_MipiCsrRegistersPtr->PHY_CTRL_CONT_CLK_MODE_BIT:0) |
                                        m_MipiCsrRegistersPtr->PHY_CTRL_AUTO_PD_EN_BIT |
                                        m_MipiCsrRegistersPtr->PHY_CTRL_S_PRG_RXHS_SETTLE(ThsSettleEscClk - 1UL) |
                                        m_MipiCsrRegistersPtr->PHY_CTRL_RTERM_SEL_BIT |
                                        m_MipiCsrRegistersPtr->PHY_CTRL_PD_BIT);

    m_MipiCsrRegistersPtr->PLM_CTRL = (m_MipiCsrRegistersPtr->PLM_CTRL_VALID_OVERRIDE_BIT | m_MipiCsrRegistersPtr->PLM_CTRL_ENABLE_BIT); // Enable pixel link master // FIXME Why PLM_CTRL_VALID_OVERRIDE_BIT?
    m_MipiCsrRegistersPtr->PHY_CTRL &= ~(m_MipiCsrRegistersPtr->PHY_CTRL_PD_BIT); // Power up PHY
    m_MipiCsrRegistersPtr->CTRL_CLK_RESET_CTRL = m_MipiCsrRegistersPtr->CTRL_CLK_RESET_CTRL_SW_RESETN_BIT; // Deassert reset

    m_MipiCsrRegistersPtr->PHY_FENCE_CTRL = Config.DisableLaneMask;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "MipiCsi2_t::EnablePhy (settle = 0x%x)", (unsigned)ThsSettleEscClk);

    return status;
}

NTSTATUS MipiCsi2_t::Phy8xDisable()
/*!
 * Disables MIPI D-Phy.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    m_MipiCsrRegistersPtr->PHY_FENCE_CTRL = 0xF;
    m_MipiCsrRegistersPtr->PHY_CTRL = m_MipiCsrRegistersPtr->PHY_CTRL_PD_BIT;
    m_MipiCsrRegistersPtr->PLM_CTRL = 0x0;

    return status;
}

// Common

NTSTATUS MipiCsi2_t::PhyAssertReset()
/*!
 * Assert reset of MIPI D-Phy.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;

    switch (m_ResourcePtr->m_CpuId) {
    case IMX_CPU_MX8MQ:
        status = m_ResourcePtr->AcpiSetMipiRcr(1);
        break;
    case IMX_CPU_MX8QXP:
        status = m_ResourcePtr->ScfwSetMipiRcr(1);
        break;
    default:
        status = STATUS_NOT_IMPLEMENTED;
        break;
    }
    { // 20 ms delay. 
        LARGE_INTEGER Interval;
        Interval.QuadPart = -20 * 100; // Interval is number of msecs in 100 ns units.
        KeDelayExecutionThread(KernelMode, FALSE, &Interval);
    }
    return status;
}

NTSTATUS MipiCsi2_t::PhyDeassertReset()
/*!
 * De-assert reset of MIPI D-Phy.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    switch (m_ResourcePtr->m_CpuId) {
    case IMX_CPU_MX8MQ:
        status = Phy8mqDeassertReset();
        break;
    case IMX_CPU_MX8QXP:
        status = m_ResourcePtr->ScfwSetMipiRcr(0);
        break;
    default:
        status = STATUS_NOT_IMPLEMENTED;
        break;
    }
    { // 20 ms delay. 
        LARGE_INTEGER Interval;
        Interval.QuadPart = -20 * 100; // Interval is number of msecs in 100 ns units.
        KeDelayExecutionThread(KernelMode, FALSE, &Interval);
    }
    return status;
}

NTSTATUS MipiCsi2_t::PhyEnable(const camera_config_t& Config, UINT8 ThsSettleEscClk) // , UINT32 DisableLaneMask)
/*!
 * Configures Hs Settle delay and enables Mipi D-Phy.
 *
 * @param Config Information about video stream being processed (resolution, frame rate, color format ..).
 * @param ThsSettleEscClk Hs Settle delay as number of Esc clock periods.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;

    switch (m_ResourcePtr->m_CpuId) {
    case IMX_CPU_MX8MQ:
        status = Phy8mqEnable(Config, ThsSettleEscClk);
        break;
    case IMX_CPU_MX8QXP:
        status = Phy8xEnable(Config, ThsSettleEscClk);
        break;
    default:
        status = STATUS_NOT_IMPLEMENTED;
        break;
    }
    return status;
}

NTSTATUS MipiCsi2_t::PhyDisable()
/*!
 * Disables MIPI D-Phy.
 */
{
    NTSTATUS status;

    switch (m_ResourcePtr->m_CpuId) {
    case IMX_CPU_MX8MQ:
        status = Phy8mqDisable();
        break;
    case IMX_CPU_MX8QXP:
        status = Phy8xDisable();
        break;
    default:
        status = STATUS_NOT_IMPLEMENTED;
        break;
    }
    return status;
}

void MipiCsi2_t::DisableInterrupts(UINT32 mask)
/*!
 * Disable the MIPI interrupts.
 *
 * @param mask OR'ed interrupt mask.
 */
{
    m_RegistersPtr->IRQ_MASK |= mask;
}

NTSTATUS MipiCsi2_t::Init(const camera_config_t &Config)
/*!
 * Configures the CSI2RX peripheral module.
 *
 * @param Config information about video stream being processed (resolution, frame rate, color format ..).
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(Config);
    // 1. Reset DPHY
    // 2. Init MIPI
    // 3. Enable DPHY
    PhyDisable();
    PhyAssertReset();

    m_RegistersPtr->DISABLE_PAYLOAD_0 &=
        ~(MIPI_CSI2RX_REGS::DISABLE_PAYLOAD_0_RGB888
            | MIPI_CSI2RX_REGS::DISABLE_PAYLOAD_0_RGB565
            | MIPI_CSI2RX_REGS::DISABLE_PAYLOAD_0_YUV422_10B
            | MIPI_CSI2RX_REGS::DISABLE_PAYLOAD_0_YUV422_8B
            | MIPI_CSI2RX_REGS::DISABLE_PAYLOAD_0_YUV420_8B
        );

    if (NT_SUCCESS(status)) {
        m_RegistersPtr->IGNORE_VC = 1;
        m_RegistersPtr->VID_VC = 1;
        m_RegistersPtr->VID_P_FIFO_SEND_LEVEL = 64;
    }
    DisableInterrupts(0x1FF);

    return status;
}

NTSTATUS MipiCsi2_t::Start(const camera_config_t &Config)
/*!
 * Enables configured number data lanes and configures Hs Settle delay.
 *
 * @param Config information about video stream being processed (resolution, frame rate, color format ..).
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    UINT32 hsSettle, hsPrgSettle;

    /* Hs-settle timeout = 115 + 8 * data_rate[Gbps] */
    hsSettle = 115 + ((8 * 1000) / (Config.mipiLaneClk / 1000000));
    hsPrgSettle = ((hsSettle * (m_ResourcePtr->m_EscClockFrequencyHz / 1000000)) / 1000) - 1;

    m_RegistersPtr->NUM_LANES = Config.csiLanes - 1UL;
    m_RegistersPtr->DISABLE_DATA_LANES = Config.DisableLaneMask; //  0xF & ~((1UL << (UINT32)Config.csiLanes) - 1UL);
    m_RegistersPtr->VID_VSYNC = 2600u;

    PhyEnable(Config, (UINT8)hsPrgSettle);

    return STATUS_SUCCESS;
}

NTSTATUS MipiCsi2_t::Stop()
/*!
 * Disables all CSI2RX data lanes. Keeps CSI fifo state.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    m_RegistersPtr->DISABLE_DATA_LANES |= 0xF;

    return STATUS_SUCCESS;
}

NTSTATUS MipiCsi2_t::Deinit()
/*!
 * Disables the CSI2RX peripheral module.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    Stop();
    PhyDisable();

    return STATUS_SUCCESS;
}

NTSTATUS MipiCsi2_t::PrepareHw(Resources_t &MipiRes)
/*!
 * Passes Initializes CSI2RX peripheral module defaults.
 *
 * @param MipiRes containing resources and information from ACPI.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    ASSERT(this != NULL);
    if (this == NULL) {
        status = STATUS_EXPIRED_HANDLE;
    }
    m_ResourcePtr = &MipiRes;
    m_RegistersPtr = (MIPI_CSI2RX_REGS*)MipiRes.m_MipiRegistersPtr; // Shouldn't probably cast here
    if (m_RegistersPtr == NULL) {
        status = STATUS_INVALID_PARAMETER_1;
    }
    m_MipiCsrRegistersPtr = MipiRes.m_MipiCsrRegistersPtr; // Shouldn't probably cast here
    if ((MipiRes.m_HasMipiCsrRegRes) && (m_RegistersPtr == NULL)) {
        status = STATUS_INVALID_PARAMETER_2;
    }
    /* Don't disable data types. */
    m_RegistersPtr->DISABLE_PAYLOAD_0 = 0xFFFFFFFF;
    m_RegistersPtr->DISABLE_PAYLOAD_1 = 0xFFFFFFFF;
    DisableInterrupts(0x1FF);

    /* Disable all interrupts. */
    m_RegistersPtr->IRQ_MASK = MIPI_CSI2RX_IRQ_MASK_IRQ_MASK_MASK;

    return status;
}
