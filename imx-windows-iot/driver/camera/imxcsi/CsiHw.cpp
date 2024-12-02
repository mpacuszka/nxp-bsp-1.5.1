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

#include "imxcsi.h"
#include "CsiIomap.h"

void WdfCsi_ctx::DisableAllInterruptsNofence()
/*!
 * Disable all interrupts.
 */
{
    m_CsiRegistersPtr->CR1 &= ~(CR1_INT_EN_MASK | CR1_FRAME_INT_EN_MASK);
    m_CsiRegistersPtr->CR3 &= ~(CR3_INT_EN_MASK | CR3_FRAME_INT_EN_MASK);
    m_CsiRegistersPtr->CR18 &= ~(CR18_INT_EN_MASK | CR18_FRAME_INT_EN_MASK);

    m_CurrSrErrorMask = 0;
}

void WdfCsi_ctx::EnableErrorInterruptsNofence(bool Enable, bool Clear)
/*!
 * Enable interrupts for errors.
 *
 * @param enable If true interrupts will be enabled otherwise disabled.
 * @param clear  If true error flags will be cleared in status register.
 */
{
    if (Enable) {
        m_CsiRegistersPtr->SR = 0xFFFFFFF;
    }
    if (Clear) {
        m_CurrSrErrorMask = SR_HRESP_ERR_INT_BIT | SR_RF_OR_INT_BIT | SR_BASEADDR_CHHANGE_ERROR_BIT;
        m_CsiRegistersPtr->CR1 |= CR1_INT_EN_MASK;
        m_CsiRegistersPtr->CR3 |= CR3_INT_EN_MASK;
        m_CsiRegistersPtr->CR18 |= CR18_INT_EN_MASK; // This happens if DMA address is reloaded on SOF instead of DMA done. However this mode mitigates frame drops caused by HW fifo stalls.
    }
    else {
        m_CsiRegistersPtr->CR1 &= ~CR1_INT_EN_MASK;
        m_CsiRegistersPtr->CR3 &= ~CR3_INT_EN_MASK;
        m_CsiRegistersPtr->CR18 &= ~CR18_INT_EN_MASK;

        m_CurrSrErrorMask &= ~(SR_HRESP_ERR_INT_BIT | SR_RF_OR_INT_BIT | SR_BASEADDR_CHHANGE_ERROR_BIT);
    }
}

void WdfCsi_ctx::EnableFrameInterruptsNofence(bool enable, bool clear)
/*!
 * Enable interrupts for complete frames.
 *
 * @param enable If true interrupts will be enabled otherwise disabled.
 * @param clear  If true finished frame flags will be cleared in status register.
 */
{

    if (clear) {
        m_CsiRegistersPtr->SR = SR_SOF_INT_BIT
            | SR_DMA_FIELD1_DONE_BIT | SR_DMA_FIELD2_DONE_BIT
            | SR_DMA_TSF_DONE_FB1_BIT | SR_DMA_TSF_DONE_FB2_BIT;
    }
    if (enable) {
        m_CsiRegistersPtr->CR1 |= CR1_FRAME_INT_EN_MASK;
    }
    else {
        m_CsiRegistersPtr->CR1 &= ~CR1_FRAME_INT_EN_MASK;
    }
}

UINT8 WdfCsi_ctx::CsiPopSrFrameId(volatile CSI_REGS *RegistersPtr, UINT32 Sr)
/*!
 * Try to get Id <1 or 2> of received frame.
 *
 * @param RegistersPtr Pointer to the CSI register so handled frame statuses could bee cleared.
 * @param Sr           State of status register to extract frame id from.
 *
 * @returns 1 if only buffer one has been finished, 2 if only buffer two has been finished otherwise 0.
 */
{
    UNREFERENCED_PARAMETER(RegistersPtr);
    UINT8 id = 0;
    UINT32 tsfDone = (Sr & SR_SOF_INT_BIT)>1?Sr & (SR_DMA_TSF_DONE_FB1_BIT | SR_DMA_TSF_DONE_FB2_BIT):0; //  (SR_DMA_TSF_DONE_FB1_MASK | SR_DMA_TSF_DONE_FB2_MASK); // SR is already loaded, check that first.

    RegistersPtr->SR = tsfDone; // Zero status
    if ((tsfDone == 0) || ((tsfDone & ~SR_SOF_INT_BIT) == (SR_DMA_TSF_DONE_FB1_BIT | SR_DMA_TSF_DONE_FB2_BIT))) {
        ++m_ErrorDoubleFrameDoneCnt;
        ++m_DiscardedFrameCnt;
    }
    else {
        id = (0 < (tsfDone & SR_DMA_TSF_DONE_FB2_BIT)) + 1;
    }
    return id;
}

void WdfCsi_ctx::CsiStart(bool enable)
/*!
 * Start or stop CSI.
 * Enables IRQ, clears status and enables DMA. Disables IRQs, DMA and CSI and sets S_STOPPED state.
 *
 * @param enable Set true to start the CSI.
 */
{

    if (enable) {
        CsiEnableFifoDmaRequest(true);
        EnableErrorInterruptsNofence(true, true);
        EnableFrameInterruptsNofence(true, true);
        m_CsiRegistersPtr->CR18 |= CR18_CSI_ENABLE_BIT;
    }
    else {
        // Doing the following would be redundant here. Method that scheduled this DPC is responsible for this.
        //   - DisableInterruptsNofence()
        DisableAllInterruptsNofence();
        // Disable peripheral. Previous DPC might have re-enabled it.
        m_CsiRegistersPtr->CR18 &= ~CR18_CSI_ENABLE_BIT;
        CsiEnableFifoDmaRequest(false);
        m_State = S_STOPPED;
    }
}

void WdfCsi_ctx::CsiResetAndStop()
/*!
* Stop the CSI and load registers with defaults.
*/
{
    {
        CsiStart(false); /* Disable transfer first. */
        m_CsiRegistersPtr->CR3 = 0U; /* Then Disable DMA request. */
        m_CsiRegistersPtr->CR3 |= CR3_FRMCNT_RST_BIT; /* Then Reset the fame count. */
#ifdef HYPERV

#else
        while (0U != (m_CsiRegistersPtr->CR3 & CR3_FRMCNT_RST_BIT)) {}
#endif
        CsiClearRxFifo();
        CsiReflashFifoDma();

        /* Clear the status. */
        {
            UINT32 csisr = m_CsiRegistersPtr->SR;

            m_CsiRegistersPtr->SR = csisr;
        }

        /* Set the control registers to default value. */
        m_CsiRegistersPtr->CR1 = CR1_HSYNC_POL_BIT;
        m_CsiRegistersPtr->CR2 = 0U;
        m_CsiRegistersPtr->CR3 = 0U;
#if defined(CR18_LCDIF_BUFFER_LINES)
        m_CsiRegistersPtr->CR18 = CR18_AHB_HPROT(0x0DU) | CR18_LCDIF_BUFFER_LINES(0x02U);
#else
        m_CsiRegistersPtr->CR18 = CR18_AHB_HPROT(0x0DU);
#endif
        m_CsiRegistersPtr->FBUF_PARA = 0U;
        m_CsiRegistersPtr->IMAG_PARA = 0U;
    }
}

void WdfCsi_ctx::CsiClearRxFifo()
/*!
* Clear the RX FIFO.
*/
{
    UINT32 tmpCR1 = m_CsiRegistersPtr->CR1;

    m_CsiRegistersPtr->CR18 &= ~CR18_CSI_ENABLE_BIT;
    m_CsiRegistersPtr->CR1 &= ~CR1_FCC_BIT;
    m_CsiRegistersPtr->CR1 |= CR1_CLR_RXFIFO_BIT | CR1_CLR_STATFIFO_BIT;
    while ((m_CsiRegistersPtr->CR1 & CR1_CLR_RXFIFO_BIT) > 0) {};
    m_CsiRegistersPtr->CR1 = tmpCR1; // Restore state, actually only CR1_FCC_BIT was modified.
}

void WdfCsi_ctx::CsiReflashFifoDma()
/*!
 * Re-flash DMA. This is needed to update configuration and to recover from errors.
 */
{
    if ((m_FinishedBuffPtr != NULL) && (m_State != WdfCsi_ctx::S_FRAME_REQUESTED)) {
        m_FinishedBuffPtr->state = m_FinishedBuffPtr->FREE;
        m_FinishedBuffPtr = NULL;
    }
    m_CsiRegistersPtr->CR3 |= CR3_DMA_REFLASH_RFF_BIT;
    while ((m_CsiRegistersPtr->CR3 & CR3_DMA_REFLASH_RFF_BIT) > 0) {};
}

void WdfCsi_ctx::CsiEnableFifoDmaRequest(bool enable)
/*!
 * Enable or disable the CSI FIFO DMA request.
 *
 * @param enable True to enable, false to disable.
 */
{
    UINT32 cr3 = 0U;

    cr3 |= CR3_DMA_REQ_EN_RFF_BIT;

    if (enable) {
        m_CsiRegistersPtr->CR3 |= cr3;
    }
    else {
        m_CsiRegistersPtr->CR3 &= ~cr3;
    }
}

NTSTATUS WdfCsi_ctx::CsiInit(const camera_config_t &Config)
/*!
 * Enable or disable the CSI FIFO DMA request.
 *
 * @param config Video properties (resolution, framerate, color format ..).
 *
 * @return Rate that was set.
 */
{
    NTSTATUS status = STATUS_SUCCESS;
    UINT32 pixFmt = 0;
    UINT32 imgWidth_Bytes = FSL_VIDEO_EXTRACT_WIDTH((UINT32)Config.resolution) * Config.GetPixelSizeBits(Config.resultPixelFormat);

    if ((0U != (imgWidth_Bytes % 8)) || (imgWidth_Bytes == 0)) {
        _DbgKdPrint(("The image width and frame buffer pitch should be multiple of 8-bytes.\r\n"));
        status = STATUS_INVALID_PARAMETER;
    }
    if (NT_SUCCESS(status)) {
        switch (Config.cameraPixelFormat) {
        case kVIDEO_PixelFormatRGB888:
            _DbgKdPrint(("kVIDEO_PixelFormatRGB888\r\n"));
            pixFmt = CR18_MIPI_DATA_FORMAT_RGB888;
            break;
        case kVIDEO_PixelFormatRGB565:
            _DbgKdPrint(("kVIDEO_PixelFormatRGB565\r\n"));
            pixFmt = CR18_MIPI_DATA_FORMAT_RGB565;
            break;
        case kVIDEO_PixelFormatYUYV:
            _DbgKdPrint(("kVIDEO_PixelFormatYUYV\r\n"));
            pixFmt = CR18_MIPI_DATA_FORMAT_YUV422_8B;
            break;
        case kVIDEO_PixelFormatUYVY:
            _DbgKdPrint(("kVIDEO_PixelFormatUYVY\r\n"));
            pixFmt = CR18_MIPI_DATA_FORMAT_YUV422_8B;
            break;
        case kVIDEO_PixelFormatNV12:
            _DbgKdPrint(("kVIDEO_PixelFormatNV12\r\n"));
            pixFmt = CR18_MIPI_DATA_FORMAT_YUV420_8B;
            break;
        default:
            _DbgKdPrint(("kVIDEO_Invalid: 0x%x\r\n", Config.cameraPixelFormat));
            status = STATUS_INVALID_PARAMETER;
        }
    }
    if (NT_SUCCESS(status)) {
        CsiResetAndStop();
        ASSERT(m_DiscardBuff[0].phys.QuadPart < UINT32_MAX);
        ASSERT(m_DiscardBuff[0].phys.QuadPart != NULL);
        ASSERT(m_DiscardBuff[1].phys.QuadPart < UINT32_MAX);
        ASSERT(m_DiscardBuff[1].phys.QuadPart != NULL);

        // Frame buffer state initialization
        m_FinishedBuffPtr = NULL;
        // Buffer 0
        m_DiscardBuff[0].state = m_DiscardBuff[0].WORKING;
        m_CsiRegistersPtr->DMASA_FB1 = (UINT32)m_DiscardBuff[0].phys.QuadPart;
        // Buffer 1
        m_DiscardBuff[1].state = m_DiscardBuff[1].WORKING;
        m_CsiRegistersPtr->DMASA_FB2 = (UINT32)m_DiscardBuff[1].phys.QuadPart;
        // Buffer 2
        m_DiscardBuff[2].state = m_DiscardBuff[2].FREE;

        { // Image resolution
            UINT32 busCyclePerPixel = 1; /* 2 cycles only for yuv over parallel 8-bit sensor input. */
            const UINT32 width = FSL_VIDEO_EXTRACT_WIDTH((UINT32)Config.resolution) * busCyclePerPixel;
            const UINT32 height = FSL_VIDEO_EXTRACT_HEIGHT((UINT32)Config.resolution);

            m_CsiRegistersPtr->RXCNT = (width * height);
            m_CsiRegistersPtr->IMAG_PARA = (height << IMAG_PARA_IMAGE_HEIGHT_SHIFT) | (width << IMAG_PARA_IMAGE_WIDTH_SHIFT);
            m_CsiRegistersPtr->FBUF_PARA = 0x0; // No stride, Note stride is Total_len - Line_len
        }
        { // Misc configuration
            UINT32 cr1 = m_CsiRegistersPtr->CR1;
            UINT32 cr2 = m_CsiRegistersPtr->CR2;
            UINT32 cr3 = m_CsiRegistersPtr->CR3;

            cr1 = CR1_SOF_POL_BIT | CR1_REDGE_BIT | CR1_HSYNC_POL_BIT | CR1_FCC_BIT | CR1_MCLKDIV_BIT1 | CR1_MCLKEN_BIT;

            if (m_CsiTwo8bitSensorMode) { // True for i.MX 8M
                cr3 |= CR3_SENSOR_16BITS_BIT;
            }

            cr2 |= CR2_DMA_BURST_TYPE_RFF(0x3);
            cr3 |= m_RxLevel << CR3_RxFF_LEVEL_SHIFT;

            m_CsiRegistersPtr->CR1 = cr1;
            m_CsiRegistersPtr->CR2 = cr2;
            m_CsiRegistersPtr->CR3 = cr3;
            m_CsiRegistersPtr->CR18 = ((~CR18_MIPI_DATA_FORMAT_MASK) & m_CsiRegistersPtr->CR18)
                | CR18_DATA_FROM_MIPI_BIT
                | CR18_BASEADDR_SWITCH_SEL_BIT
                | CR18_BASEADDR_SWITCH_EN_BIT
                | pixFmt
                | CR18_BASEADDR_CHANGE_ERROR_IE_BIT
                | CR18_MASK_OPTION_SKIP_SECOND_FRAME
                | CR18_LCDIF_BUFFER_LINES(0x2);

        }
        // Reload DMA addresses
        m_CsiRegistersPtr->CR3 |= CR3_DMA_REFLASH_RFF_BIT;
    }
    return status;
}

bool WdfCsi_ctx::DpcCsiResetDmaIfError(UINT32 csiSr)
/*!
 * Check for errors and restart DMAs if needed.
 * Had any error occurred the DMA is re-flashed, FIFO cleared and CSI Stopped.
 *
 * @param csiSr status register copy to determine whether re-flash is required.
 *
 * @return Returns 1 if peripheral has been restarted to stop state.
 */
{
    _DbgFrameKdPrint(("WdfCsi_ctx::DpcResetDmaIfError\r\n"));
    UINT32 csiError = csiSr & (SR_HRESP_ERR_INT_BIT | SR_RF_OR_INT_BIT | SR_BASEADDR_CHHANGE_ERROR_BIT);

    if (csiError > 0) {
        if ((csiSr & SR_HRESP_ERR_INT_BIT) > 0) {
            _DbgFrameKdPrint(("SR_HRESP_ERR_INT_BIT\r\n"));
        }
        if ((csiSr & SR_RF_OR_INT_BIT) > 0) {
            _DbgFrameKdPrint(("SR_RF_OR_INT_BIT\r\n"));
        }
        if ((csiSr & SR_BASEADDR_CHHANGE_ERROR_BIT) > 0) {
            _DbgFrameKdPrint(("SR_BASEADDR_CHHANGE_ERROR_BIT\r\n"));
        }
        m_CsiRegistersPtr->SR = csiError; // Clear only error flags, leave frame completion others. But nobody really cares of these.

        CsiClearRxFifo();
        CsiReflashFifoDma();
        m_ErrorAddrChangeCnt += (csiError & SR_BASEADDR_CHHANGE_ERROR_BIT) > 0;
        m_ErrorHrespCnt += (csiError & SR_HRESP_ERR_INT_BIT) > 0;
        m_ErrorOverflowCnt += (csiError & SR_RF_OR_INT_BIT) > 0;
    }
    return csiError > 0;
}

