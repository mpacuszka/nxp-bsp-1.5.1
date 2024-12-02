/*
 * Copyright 2022-2023 NXP
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

#include "imxmipi_csis.h"
#include "mipi_csi2csis.tmh"

NTSTATUS MipiCsi2_t::Gpr8mnGasketConfigure(const camera_config_t &Config)
{
    NTSTATUS status;
    UINT32 gasketCtrl;
    UINT32 pixFmt = 0;
    const UINT32 width = FSL_VIDEO_EXTRACT_WIDTH((UINT32)Config.resolution);
    const UINT32 height = FSL_VIDEO_EXTRACT_HEIGHT((UINT32)Config.resolution);

    switch (Config.cameraPixelFormat)
    {
    case kVIDEO_PixelFormatRGB888:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "kVIDEO_PixelFormatRGB888");
        pixFmt = GPR_GASKET_0_CTRL_DATA_TYPE_RGB888;
        break;
    case kVIDEO_PixelFormatRGB565:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "TRACE_LEVEL_INFORMATION");
        pixFmt = GPR_GASKET_0_CTRL_DATA_TYPE_RGB565;
        break;
    case kVIDEO_PixelFormatYUYV:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "TRACE_LEVEL_INFORMATION");
        pixFmt = GPR_GASKET_0_CTRL_DATA_TYPE_YUV422_8;
        break;
    case kVIDEO_PixelFormatUYVY:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "kVIDEO_PixelFormatUYVY");
        pixFmt = GPR_GASKET_0_CTRL_DATA_TYPE_YUV422_8;
        break;
    case kVIDEO_PixelFormatNV12:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "kVIDEO_PixelFormatNV12");
        pixFmt = GPR_GASKET_0_CTRL_DATA_TYPE_YUV420_8;
        break;
    default:
        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DRIVER, "kVIDEO_Invalid: 0x%x", Config.cameraPixelFormat);
        status = STATUS_INVALID_PARAMETER;
    }

    status = m_ResourcePtr->AcpiRgpr(GPR_GASKET_0_CTRL_REG_OFFSET, gasketCtrl);
    if (NT_SUCCESS(status)) {
        gasketCtrl = (gasketCtrl & ~0x3FFF)
            | (((Config.cameraPixelFormat == kVIDEO_PixelFormatYUYV) || (Config.cameraPixelFormat == kVIDEO_PixelFormatUYVY)) ? GPR_GASKET_0_CTRL_DOUBLE_COMPONENT_BIT : 0)
            | pixFmt;
        status = m_ResourcePtr->AcpiWgpr(GPR_GASKET_0_CTRL_REG_OFFSET, gasketCtrl);
    }

    if (NT_SUCCESS(status)) {
        status = m_ResourcePtr->AcpiWgpr(GPR_GASKET_0_VSIZE_REG_OFFSET, height);
    }
    if (NT_SUCCESS(status)) {
        status = m_ResourcePtr->AcpiWgpr(GPR_GASKET_0_HSIZE_REG_OFFSET, width);
    }
    return status;
}

NTSTATUS MipiCsi2_t::Gpr8mnGasketStart(bool Enable)
{
    NTSTATUS status;
    UINT32 gasketCtrl;

    status = m_ResourcePtr->AcpiRgpr(GPR_GASKET_0_CTRL_REG_OFFSET, gasketCtrl);
    if (NT_SUCCESS(status)) {
        if (Enable) {
            gasketCtrl |= GPR_GASKET_0_CTRL_ENABLE_BIT;
        }
        else {
            gasketCtrl &= ~GPR_GASKET_0_CTRL_ENABLE_BIT;
        }
        status = m_ResourcePtr->AcpiWgpr(GPR_GASKET_0_CTRL_REG_OFFSET, gasketCtrl);
    }
    return status;
}

void MipiCsi2_t::SwReset()
/*!
 * Assert reset of MIPI D-Phy.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    volatile UINT16 d = 1;

    m_RegistersPtr->CSIS_COMMON_CTRL |= CSIS_COMMON_CTRL_SW_RESET_BIT;
    while (d > 0) {
        ++d;
    }
}

void MipiCsi2_t::DisableInterrupts()
/*!
 * Disable the MIPI interrupts.
 *
 * @param mask OR'ed interrupt mask.
 */
{
    m_RegistersPtr->INTERRUPT_MASK_0 = 0;
    m_RegistersPtr->INTERRUPT_MASK_1 = 0;
}

NTSTATUS MipiCsi2_t::EnablePhy(const camera_config_t &Config)
/*!
 * Configures Hs Settle delay and enables Mipi D-Phy.
 *
 * @param tHsSettle_EscClk Hs Settle delay as number of Esc clock periods.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;
    UINT32 tHsSettleEscClk = 12;
    UINT32 settleCtl = 2;

    m_RegistersPtr->DPHY_MASTER_SLAVE_CTRL_LOW = 0x1F4;
    m_RegistersPtr->DPHY_MASTER_SLAVE_CTRL_HIGH = 0;

    m_RegistersPtr->DPHY_COMMON_CTRL =
        ( m_RegistersPtr->DPHY_COMMON_CTRL &
            ~(DPHY_COMMON_CTRL_HSSETTLE_MASK
                | DPHY_COMMON_CTRL_S_CLKSETTLECTL_MASK
                | DPHY_COMMON_CTRL_ENABLE_DAT_MASK
            )
        )
        | DPHY_COMMON_CTRL_ENABLE_DAT((1UL << (UINT32)Config.csiLanes) - 1UL)
        | DPHY_COMMON_CTRL_ENABLE_CLK_BIT
        | DPHY_COMMON_CTRL_HSSETTLE(tHsSettleEscClk - 1UL)
        | DPHY_COMMON_CTRL_S_CLKSETTLECTL(settleCtl)
        ;
        /* DPHY_COMMON_CTRL_CLKSETTLECTL
            2'b0x - 110ns to 280ns (v0.87 to v1.00)
            2'b10 - 150 ns to 430ns (v0.83 to v0.86)
            2'b11 - 60 ns to 140ns (v0.82)
        */
    return status;
}

NTSTATUS MipiCsi2_t::DisablePhy()
/*!
 * Disables MIPI D-Phy.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    m_RegistersPtr->DPHY_COMMON_CTRL &=
        ~(DPHY_COMMON_CTRL_HSSETTLE_MASK
              | DPHY_COMMON_CTRL_S_CLKSETTLECTL_MASK
              | DPHY_COMMON_CTRL_ENABLE_DAT_MASK
          );
    return status;
}

NTSTATUS MipiCsi2_t::SetFormat(const camera_config_t &Config)
{
    NTSTATUS status = STATUS_SUCCESS;
    UINT32 pixFmt = 0;

    switch (Config.cameraPixelFormat)
    {
    case kVIDEO_PixelFormatRGB888:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "kVIDEO_PixelFormatRGB888");
        pixFmt = ISP_CONFIG_DATAFORMAT_RGB888;
        break;
    case kVIDEO_PixelFormatRGB565:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "kVIDEO_PixelFormatRGB565");
        pixFmt = ISP_CONFIG_DATAFORMAT_RGB565;
        break;
    case kVIDEO_PixelFormatYUYV:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "kVIDEO_PixelFormatYUYV");
        pixFmt = ISP_CONFIG_DATAFORMAT_YUV422_8;
        break;
    case kVIDEO_PixelFormatUYVY:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "kVIDEO_PixelFormatUYVY");
        pixFmt = ISP_CONFIG_DATAFORMAT_YUV422_8;
        break;
    case kVIDEO_PixelFormatNV12:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "kVIDEO_PixelFormatNV12");
        pixFmt = ISP_CONFIG_DATAFORMAT_YUV420_8;
        break;
    default:
        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DRIVER, "kVIDEO_Invalid: 0x%x", Config.cameraPixelFormat);
        status = STATUS_INVALID_PARAMETER;
    }
    if (NT_SUCCESS(status)) {
        m_RegistersPtr->ISP_CONFIGn[0].ISP_CONFIG = (
            (m_RegistersPtr->ISP_CONFIGn[0].ISP_CONFIG &
                ~(ISP_CONFIG_DATAFORMAT_MASK
                  | ISP_CONFIG_PIXEL_MODE_MASK
                  | ISP_CONFIG_PARALLEL_BIT
                 )
                | pixFmt
                | ISP_CONFIG_PIXEL_MODE(m_PixelMode)
            ));
        {
            const UINT32 width = FSL_VIDEO_EXTRACT_WIDTH((UINT32)Config.resolution);
            const UINT32 height = FSL_VIDEO_EXTRACT_HEIGHT((UINT32)Config.resolution);

            m_RegistersPtr->ISP_CONFIGn[0].ISP_RESOLUTION = ISP_RESOLUTION_VRESOL(height) | ISP_RESOLUTION_HRESOL(width);
        }
        m_RegistersPtr->ISP_CONFIGn[0].ISP_SYNC = 0;

        switch (m_CpuId) {
        case IMX_CPU_MX8MM:
            // TODO: Consider adding GPR controls for 8MM.
            break;
        case IMX_CPU_MX8MN:
        case IMX_CPU_MX8MP:
            status = Gpr8mnGasketConfigure(Config);
            break;
        default:
            status = STATUS_NOT_IMPLEMENTED;
            break;
        }
    }
    return status;
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

    // 1. Reset DPHY
    // 2. Init MIPI
    // 3. Enable DPHY
    status = DisablePhy();
    if (NT_SUCCESS(status)) {
        SwReset();
        status = SetFormat(Config);
        if (NT_SUCCESS(status)) {
            m_RegistersPtr->CSIS_CLOCK_CTRL = CSIS_CLOCK_CTRL_CLKGATE_TRAIL_CH0(0xF);
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "CSIS_COMMON_CTRL_LANE_NUMBER 0x%x", Config.csiLanes);
            m_RegistersPtr->CSIS_COMMON_CTRL |= CSIS_COMMON_CTRL_LANE_NUMBER(Config.csiLanes - 1UL) | CSIS_COMMON_CTRL_UPDATE_SHADOW_MASK;
        }
    }
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
    NTSTATUS status;

    m_RegistersPtr->CSIS_COMMON_CTRL |= CSIS_COMMON_CTRL_CSI_EN_BIT;
    status = EnablePhy(Config);
    if (NT_SUCCESS(status)) {
        switch (m_CpuId) {
        case IMX_CPU_MX8MM:
            // TODO: Consider adding GPR controls for 8MM.
            break;
        case IMX_CPU_MX8MN:
        case IMX_CPU_MX8MP:
            status = Gpr8mnGasketStart(true);
            break;
        default:
            status = STATUS_NOT_IMPLEMENTED;
            break;
        }
    }
    return status;
}

NTSTATUS MipiCsi2_t::Stop()
/*!
 * Disables all CSI2RX data lanes. Keeps CSI fifo state.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    DisablePhy();
    m_RegistersPtr->CSIS_COMMON_CTRL &= ~(CSIS_COMMON_CTRL_LANE_NUMBER_MASK | CSIS_COMMON_CTRL_CSI_EN_BIT);
    switch (m_CpuId) {
    case IMX_CPU_MX8MM:
        // TODO: Consider adding GPR controls for 8MM.
        break;
    case IMX_CPU_MX8MN:
    case IMX_CPU_MX8MP:
        status = Gpr8mnGasketStart(false);
        break;
    default:
        status = STATUS_NOT_IMPLEMENTED;
        break;
    }
    return status;
}

NTSTATUS MipiCsi2_t::Deinit()
/*!
 * Disables the CSI2RX peripheral module.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    return Stop();
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
    m_CpuId = MipiRes.m_CpuId;
    m_PixelMode = MipiRes.m_PixelMode;
    m_RegistersPtr = (MIPI_CSI2CSIS_REGS*)MipiRes.m_MipiRegistersPtr; // Shouldn't probably cast here
    if (m_RegistersPtr == NULL) {
        status = STATUS_INVALID_PARAMETER_1;
    }
    // Do not check for status, disable everything we can.
    SwReset();
    DisableInterrupts();
    return status;
}
