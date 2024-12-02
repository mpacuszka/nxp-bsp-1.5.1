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

#include "imxmipi_csi2_dwc.h"
#include "imxmipi_csi2_dwc.tmh"

/* Camera data format */
enum DataType {
    YUV420_8  = 0x18,
    YUV420_10 = 0x19,
    YUV422_8  = 0x1E,
    YUV422_10 = 0x1F,
    RGB565    = 0x22,
    RGB888    = 0x24,
    RAW8      = 0x2A,
    RAW10     = 0x2B,
    RAW12     = 0x2C,
};

#if DBG
NTSTATUS MipiCsi2_t::MipiCsi2DwcDump()
/*!
 * Dump MIPI CSI2 Block.
 */
{
    NTSTATUS status = STATUS_SUCCESS;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_VERSION = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_VERSION);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_N_LANES = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_N_LANES);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_CSI2_RESETN = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_CSI2_RESETN);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_MAIN = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_MAIN);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_DATA_IDS1 = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_DATA_IDS1);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_DATA_IDS2 = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_DATA_IDS2);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_DPHY_CFG = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_CFG);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_DPHY_MODE = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_MODE);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_AP_MAIN = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_AP_MAIN);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_DPHY_RSTZ = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_RSTZ);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_PHY_RX = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_PHY_RX);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_PHY_STOPSTATE = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_PHY_STOPSTATE);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_PHY_TEST_CTRL0 = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_PHY_TEST_CTRL0);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_PHY_TEST_CTRL1 = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_PHY_TEST_CTRL1);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_PPI_PG_PATTERN_VRES = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_PPI_PG_PATTERN_VRES);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_PPI_PG_PATTERN_HRES = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_PPI_PG_PATTERN_HRES);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_PPI_PG_CONFIG = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_PPI_PG_CONFIG);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_PPI_PG_ENABLE = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_PPI_PG_ENABLE);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_PPI_PG_STATUS = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_PPI_PG_STATUS);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_MODE = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_MODE);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_VCID = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_VCID);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_DATA_TYPE = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_DATA_TYPE);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_MEM_FLUSH = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_MEM_FLUSH);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_HSA_TIME = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_HSA_TIME);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_HBP_TIME = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_HBP_TIME);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_HSD_TIME = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_HSD_TIME);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_HLINE_TIME = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_HLINE_TIME);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_SOFTRSTN = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_SOFTRSTN);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_ADV_FEATURES = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_ADV_FEATURES);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_VSA_LINES = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_VSA_LINES);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_VBP_LINES = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_VBP_LINES);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_VFP_LINES = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_VFP_LINES);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_IPI_VACTIVE_LINES = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_VACTIVE_LINES);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_VC_EXTENSION = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_VC_EXTENSION);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_PHY_CAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_PHY_CAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_PHY_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_PHY_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_MSK_PHY_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_PHY_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_FORCE_PHY_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_FORCE_PHY_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_PKT_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_PKT_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_MSK_PKT_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_PKT_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_FORCE_PKT_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_FORCE_PKT_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_PHY = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_PHY);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_MSK_PHY = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_PHY);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_FORCE_PHY = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_FORCE_PHY);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_MSK_LINE = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_LINE);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_IPI_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_IPI_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_MSK_IPI_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_IPI_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_FORCE_IPI_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_FORCE_IPI_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_MSK_AP_GENERIC = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_AP_GENERIC);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_BNDRY_FRAME_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_BNDRY_FRAME_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_SEQ_FRAME_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_SEQ_FRAME_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_SEQ_FRAME_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_SEQ_FRAME_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_CRC_FRAME_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_CRC_FRAME_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_MSK_CRC_FRAME_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_CRC_FRAME_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_FORCE_CRC_FRAME_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_FORCE_CRC_FRAME_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_PLD_CRC_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_PLD_CRC_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_MSK_PLD_CRC_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_PLD_CRC_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_FORCE_PLD_CRC_FATAL = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_FORCE_PLD_CRC_FATAL);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_DATA_ID = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_DATA_ID);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_MSK_DATA_ID = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_DATA_ID);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_FORCE_DATA_ID = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_FORCE_DATA_ID);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_ST_ECC_CORRECTED = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_ST_ECC_CORRECTED);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_MSK_ECC_CORRECTED = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_ECC_CORRECTED);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_INT_FORCE_ECC_CORRECTED = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_FORCE_ECC_CORRECTED);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_SCRAMBLING = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_SCRAMBLING);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_SCRAMBLING_SEED1 = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_SCRAMBLING_SEED1);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_DWC_MIPI_CSI2_SCRAMBLING_SEED2 = 0x%08X", m_RegistersPtr->IMX_DWC_MIPI_CSI2_SCRAMBLING_SEED2);
    return status;
}

NTSTATUS MipiCsi2_t::MediaMixGasketDump()
/*!
 * Dump MEDIAMIX gasket block;.
 */
{
    NTSTATUS status = STATUS_SUCCESS;
    UINT32 val = 0U;

    status |= m_ResourcePtr->AcpiRgpr(IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX, val);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX = 0x%08X", val);
    status |= m_ResourcePtr->AcpiRgpr(IMX_BLK_CTRL_MEDIAMIX_CSI, val);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_BLK_CTRL_MEDIAMIX_CSI = 0x%08X", val);
    status |= m_ResourcePtr->AcpiRgpr(IMX_BLK_CTRL_MEDIAMIX_ISI0, val);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_BLK_CTRL_MEDIAMIX_ISI0 = 0x%08X", val);
    status |= m_ResourcePtr->AcpiRgpr(IMX_BLK_CTRL_MEDIAMIX_ISI1, val);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "IMX_BLK_CTRL_MEDIAMIX_ISI1 = 0x%08X", val);

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "MediaMix dump failed, status = 0x%08X", status);
    }
    return status;
}
#endif

void MipiCsi2_t::MipiCsi2HostReset()
/*!
 * MIPI CSI2 host reset.
 */
{
    /* Reset is active low */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_CSI2_RESETN = 0U;
    /* Deasert reset */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_CSI2_RESETN = 1U;
}

void MipiCsi2_t::MipiCsi2DphyReset()
/*!
 * MIPI CSI2 DPhy reset.
 */
{
    /* Reset is active low */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_RSTZ = 0U;
    /* Power down analog blocks */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_SHUTDOWNZ = 0U;
    KeStallExecutionProcessor(50); //50us

    /* Power Up all analog blocks*/
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_SHUTDOWNZ = 1U;
    KeStallExecutionProcessor(50); //50us
    /* Deasert reset */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_RSTZ = 1U;
}

void MipiCsi2_t::MipiCsi2TestControlInterfaceReset()
/*!
 * Reset DPhy test and control interface.
 */
{
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_PHY_TEST_CTRL0 |= IMX_DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLR;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_PHY_TEST_CTRL0 &= ~IMX_DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLR;
}

void MipiCsi2_t::MediamixGasketReset()
/*!
 * Reset DPhy test and control interface.
 */
{
    UINT32 val = 0;
    UINT32 rstMask = (MEDIAMIX_BLK_CTRL_RESET_ISI_APB_EN_MASK | MEDIAMIX_BLK_CTRL_RESET_ISI_PROC_EN_MASK |
                     MEDIAMIX_BLK_CTRL_RESET_PXP_APB_EN_MASK | MEDIAMIX_BLK_CTRL_RESET_PXP_AXI_EN_MASK |
                     MEDIAMIX_BLK_CTRL_RESET_CSI_APB_EN_MASK | MEDIAMIX_BLK_CTRL_RESET_CAM_CLK_EN_MASK);

    /* Reset camera related HW blocks */
    m_ResourcePtr->AcpiRgpr(IMX_BLK_CTRL_MEDIAMIX_RESET, val);
    val &= ~rstMask;
    m_ResourcePtr->AcpiWgpr(IMX_BLK_CTRL_MEDIAMIX_RESET, val);

    /* Gate clocks */
    m_ResourcePtr->AcpiRgpr(IMX_BLK_CTRL_MEDIAMIX_CLK, val);
    val |= rstMask;
    m_ResourcePtr->AcpiWgpr(IMX_BLK_CTRL_MEDIAMIX_CLK, val);

    KeStallExecutionProcessor(50); // 50us delay

    /* Deasert resets */
    m_ResourcePtr->AcpiRgpr(IMX_BLK_CTRL_MEDIAMIX_RESET, val);
    val |= rstMask;
    m_ResourcePtr->AcpiWgpr(IMX_BLK_CTRL_MEDIAMIX_RESET, val);

    /* Ungate clocks */
    m_ResourcePtr->AcpiRgpr(IMX_BLK_CTRL_MEDIAMIX_CLK, val);
    val &= ~rstMask;
    m_ResourcePtr->AcpiWgpr(IMX_BLK_CTRL_MEDIAMIX_CLK, val);
}

NTSTATUS MipiCsi2_t::Init(const camera_config_t &Config)
/*!
 * Configure DWC MIPI CSI2 block.
 *
 * @param Config information about video stream being processed (resolution, frame rate, color format ..).
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status = STATUS_SUCCESS;
    UINT32 val = 0;

    MediamixGasketReset();
    MipiCsi2HostReset();
    MipiCsi2DphyReset();
    MipiCsi2TestControlInterfaceReset();

    /* Reset PHY */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_RSTZ      = 0U;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_SHUTDOWNZ = 0U;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_CSI2_RESETN    = 0U;

    /* An initial high pulse after power up */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_PHY_TEST_CTRL0 |= IMX_DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLR;
    KeStallExecutionProcessor(1); //1us
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_PHY_TEST_CTRL0 &= ~IMX_DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLR;

    /* Write gasket IMX_BLK_CTRL_MEDIAMIX_CSI */
    status = m_ResourcePtr->AcpiWgpr(IMX_BLK_CTRL_MEDIAMIX_CSI, ((0x1C << IMX_BLK_CTRL_MEDIAMIX_CSI_CFGCLKFREQRANGE_SHIFT) & IMX_BLK_CTRL_MEDIAMIX_CSI_CFGCLKFREQRANGE_MASK) |
                                                                ((0x2B << IMX_BLK_CTRL_MEDIAMIX_CSI_HSCLKFREQRANGE_SHIFT) & IMX_BLK_CTRL_MEDIAMIX_CSI_HSCLKFREQRANGE_MASK));

    /* Set number of data lanes */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_N_LANES = Config.csiLanes - 1UL;

    /* PPI8 */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_CFG = 0U;

    /* Deasert PHY reset */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_SHUTDOWNZ = 1U;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DPHY_RSTZ      = 1U;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_CSI2_RESETN    = 1U;

    /* Errors to be masked */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_PHY_FATAL  = 0xFFFF;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_PKT_FATAL  = 0xFFFF;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_IPI_FATAL  = 0xFFFF;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_AP_GENERIC = 0xFFFF;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_PHY        = 0xFFFF;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_INT_MSK_LINE       = 0xFFFF;

    /* IDI data type config */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DATA_IDS1 = YUV422_8;

    /* IDI virtual chnl config */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_DATA_IDS_VC1 = 0U;

    UINT32 loop = 1000U;
    while (true) {
        if ((m_RegistersPtr->IMX_DWC_MIPI_CSI2_PHY_STOPSTATE == 0x10003) || loop == 0) {
            break;
        }
        KeStallExecutionProcessor(10); // 10us delay
        loop--;
    }
    if (loop == 0U) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DRIVER, "Clock or data lanes are not in STOP state");
        status = STATUS_UNEXPECTED_IO_ERROR;
        goto End;
    }

    /* IPI SW reset */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_SOFTRSTN = 0U;
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_SOFTRSTN = 0xFFFFU;

    /* Configure IPI data type */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_DATA_TYPE = YUV422_8;

    /* Configure IPI virtual chnl to 0 */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_VCID = 0;

    /* Camera mode, color mode components 48bit */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_MODE = 0;

    /* IPI cut through enable */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_MODE |= MIPI_CSI_IPI_MODE_IPI_CUT_THROUGH_MASK;

    /* IPI horizontal timing */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_HSA_TIME = 0;       /*  Horizontal Synchronism Active */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_HBP_TIME = 0;       /* Horizontal Back Porch */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_HSD_TIME = 0;       /* Horizontal Sync Delay */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_HLINE_TIME = 0x500; /* Size of the line time counted in pixclk cycles */
#if 0
    /* IPI vertical timing */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_VSA_LINES = 0;       /*  Horizontal Synchronism Active */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_VBP_LINES = 0;       /* Horizontal Back Porch */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_VFP_LINES = 0;       /* Horizontal Sync Delay */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_VACTIVE_LINES = 0x320; /* Size of the line time counted in pixclk cycles */
#endif
    /* Flush IPI memory on each Frame Start */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_MEM_FLUSH = MIPI_CSI_IPI_MEM_FLUSH_IPI_AUTO_FLUSH_MASK;

    /* MEDIAMIX data type configure */
    status = m_ResourcePtr->AcpiWgpr(IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX, 0);
    status = m_ResourcePtr->AcpiWgpr(IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX, IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX_DATA_TYPE(YUV422_8));

    /* MEDIAMIX PHY freq range */
    status = m_ResourcePtr->AcpiWgpr(IMX_BLK_CTRL_MEDIAMIX_CSI, ((0x1C << IMX_BLK_CTRL_MEDIAMIX_CSI_CFGCLKFREQRANGE_SHIFT) & IMX_BLK_CTRL_MEDIAMIX_CSI_CFGCLKFREQRANGE_MASK) |
        ((0x2B << IMX_BLK_CTRL_MEDIAMIX_CSI_HSCLKFREQRANGE_SHIFT) & IMX_BLK_CTRL_MEDIAMIX_CSI_HSCLKFREQRANGE_MASK));

    /* MEDIAMIX gasket enable */
    status = m_ResourcePtr->AcpiRgpr(IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX, val);
    val |= MEDIAMIX_BLK_CTRL_CAMERA_MUX_ENABLE_MASK;
    status = m_ResourcePtr->AcpiWgpr(IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX, val);

#if DBG
    status = MipiCsi2DwcDump();
    status = MediaMixGasketDump();
#endif

End:
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
    NTSTATUS status = STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(Config);
    /* Enable IPI */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_MODE |= MIPI_CSI_IPI_MODE_IPI_ENABLE_MASK;

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

    /* Disable IPI */
    m_RegistersPtr->IMX_DWC_MIPI_CSI2_IPI_MODE &= ~MIPI_CSI_IPI_MODE_IPI_ENABLE_MASK;
    /* DPHY reret */
    MipiCsi2DphyReset();

    return status;
}

NTSTATUS MipiCsi2_t::Deinit()
/*!
 * Disables the CSI2 peripheral module.
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
    m_RegistersPtr = (MIPI_CSI2_DWC_REGS*)MipiRes.m_MipiRegistersPtr; // Shouldn't probably cast here
    if (m_RegistersPtr == NULL) {
        status = STATUS_INVALID_PARAMETER_1;
    }

    return status;
}
