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

#include "imxisi.h"
#include "isi_iomap.h"

void WdfIsi_ctx::DisableAllInterruptsNofence()
/*!
 * Disable all interrupts.
 */
{
    m_IsiRegistersPtr->IER &= ~IER_ALL_MASK;
    m_CurrSrErrorMask = 0;
}

void WdfIsi_ctx::EnableErrorInterruptsNofence(bool Enable, bool Clear)
/*!
 * Enable interrupts for errors.
 *
 * @param enable If true interrupts will be enabled otherwise disabled.
 * @param clear  If true error flags will be cleared in status register.
 */
{
    UINT32 ieMask = (IER_OFLW_BUF_MASK | IER_EARLY_VSYNC_ERR_EN_BIT | IER_AXI_WR_ERR_MASK);
    if (Clear) {
        m_IsiRegistersPtr->STS = ieMask;
    }
    if (Enable) {
        m_CurrSrErrorMask = ieMask;
        m_IsiRegistersPtr->IER |= ieMask;
    }
    else {
        m_IsiRegistersPtr->IER &= ~ieMask;
        m_CurrSrErrorMask = 0;
    }
}

void WdfIsi_ctx::EnableFrameInterruptsNofence(bool enable, bool clear)
/*!
 * Enable interrupts for complete frames.
 *
 * @param enable If true interrupts will be enabled otherwise disabled.
 * @param clear  If true finished frame flags will be cleared in status register.
 */
{
    UINT32 ieMask = (IER_FRM_RCVD_EN_BIT);

    if (clear) {
        m_IsiRegistersPtr->STS = ieMask;
    }
    if (enable) {
        m_IsiRegistersPtr->IER |= ieMask;
    }
    else {
        m_IsiRegistersPtr->IER &= ~ieMask;
    }
}

void WdfIsi_ctx::setFrameBuffer(UINT8 FrameBudId, DiscardBuffInfo_t &discardBuff)
{
    if (FrameBudId == 0) {
        m_IsiRegistersPtr->OUT_BUF1_ADDR_Y = (UINT32)discardBuff.physY.QuadPart;
        m_IsiRegistersPtr->OUT_BUF1_ADDR_U = (UINT32)discardBuff.physU.QuadPart;
        m_IsiRegistersPtr->OUT_BUF_CTRL ^= OUT_BUF_CTRL_LOAD_BUF1_ADDR_BIT;
    }
    else {
        m_IsiRegistersPtr->OUT_BUF2_ADDR_Y = (UINT32)discardBuff.physY.QuadPart;
        m_IsiRegistersPtr->OUT_BUF2_ADDR_U = (UINT32)discardBuff.physU.QuadPart;
        m_IsiRegistersPtr->OUT_BUF_CTRL ^= OUT_BUF_CTRL_LOAD_BUF2_ADDR_BIT;
    }
}

void WdfIsi_ctx::IsiStart(bool enable)
/*!
 * Start or stop CSI.
 * Enables IRQ, clears status and enables DMA. Disables IRQs, DMA and CSI and sets S_STOPPED state.
 *
 * @param enable Set true to start the CSI.
 */
{
    if (enable) {
        EnableErrorInterruptsNofence(true, true);
        EnableFrameInterruptsNofence(true, true);
        m_IsiRegistersPtr->CTRL |= CTRL_EN_BIT;
    }
    else {
        DisableAllInterruptsNofence();
        // Disable peripheral. Previous DPC might have re-enabled it.
        m_IsiRegistersPtr->CTRL &= ~CTRL_EN_BIT;
        m_State = S_STOPPED;
    }
}

void WdfIsi_ctx::IsiResetAndStop()
/*!
* Stop the CSI and load registers with defaults.
*/
{
    IsiStart(false); // This might not be required when setting CTRL_SW_RST_BIT.
    m_IsiRegistersPtr->CTRL |= CTRL_SW_RST_BIT;
    m_IsiRegistersPtr->CTRL &= ~CTRL_SW_RST_BIT;

    {
        UINT32 a = 6000;

        while (--a > 0) { ; }
    }
}

NTSTATUS WdfIsi_ctx::IsiInit(const camera_config_t &Config)
/*!
 * Enable or disable the CSI FIFO DMA request.
 *
 * @param config Video properties (resolution, framerate, color format ..).
 *
 * @return Rate that was set.
 */
{
    NTSTATUS status = STATUS_SUCCESS;
    UINT32 resultPixFmt = 0;
    bool outpIsYuv = false;
    UINT32 imgWidth_Bytes = FSL_VIDEO_EXTRACT_WIDTH((UINT32)Config.resolution) * Config.GetPixelSizeBits(Config.resultPixelFormat);

    if ((0U != (imgWidth_Bytes % 8)) || (imgWidth_Bytes == 0)) {
        _DbgKdPrint(("The image width and frame buffer pitch should be multiple of 8-bytes.\r\n"));
        status = STATUS_INVALID_PARAMETER;
    }
    // Correct input format could be asserted here.
    if (NT_SUCCESS(status)) {
        m_NumPlanes = 1;
        m_PlaneBytesPerPixel = 2;
        switch (Config.resultPixelFormat) {
        case kVIDEO_PixelFormatYUYV:
            _DbgKdPrint(("kVIDEO_PixelFormatYUYV\r\n"));
            resultPixFmt = IMG_CTRL_FORMAT_YUV422_8B_1P;
            outpIsYuv = true;
            break;
        case kVIDEO_PixelFormatUYVY:
            _DbgKdPrint(("kVIDEO_PixelFormatUYVY\r\n"));
            resultPixFmt = IMG_CTRL_FORMAT_YUV422_8B_1P;
            outpIsYuv = true;
            break;
        case kVIDEO_PixelFormatNV12:
            _DbgKdPrint(("kVIDEO_PixelFormatNV12\r\n"));
            resultPixFmt = IMG_CTRL_FORMAT_YUV420_8B_2P;
            outpIsYuv = true;
            m_NumPlanes = 2;
            m_PlaneBytesPerPixel = 1;
            break;
        default:
            _DbgKdPrint(("kVIDEO_Invalid: 0x%x\r\n", Config.resultPixelFormat));
            status = STATUS_INVALID_PARAMETER;
        }
        ReSplitFrameBuffers(Config);
    }
    if (NT_SUCCESS(status)) {
        IsiResetAndStop();
        m_IsiRegistersPtr->CTRL = CTRL_CLK_EN_BIT; // Power up registers.

        ASSERT(m_DiscardBuff[0].physY.QuadPart < UINT32_MAX); // Only Y shall be set
        ASSERT(m_DiscardBuff[0].physY.QuadPart != NULL);
        ASSERT(m_DiscardBuff[1].physY.QuadPart < UINT32_MAX);
        ASSERT(m_DiscardBuff[1].physY.QuadPart != NULL);

        // Frame buffer state initialization
        m_FinishedBuffPtr = NULL;
        // Buffer 0
        m_DiscardBuff[0].state = m_DiscardBuff[0].WORKING;
        setFrameBuffer(0, m_DiscardBuff[0]);
        // Buffer 1
        m_DiscardBuff[1].state = m_DiscardBuff[1].WORKING;
        setFrameBuffer(1, m_DiscardBuff[1]);
        // Buffer 2
        m_DiscardBuff[2].state = m_DiscardBuff[2].FREE;

#if ( COMMON_FRAME_BUFFER_NUM > 3 )
        m_DiscardBuff[3].state = m_DiscardBuff[3].FREE;
#endif

        { // Image resolution
            const UINT32 width = FSL_VIDEO_EXTRACT_WIDTH((UINT32)Config.resolution);
            const UINT32 height = FSL_VIDEO_EXTRACT_HEIGHT((UINT32)Config.resolution);
            _DbgKdPrint(("ISI W:%d H:%d\r\n", width, height));
            m_IsiRegistersPtr->IMG_CFG = IMG_CFG_HEIGHT(height) | IMG_CFG_WIDTH(width);
            m_IsiRegistersPtr->SCL_IMG_CFG = SCL_IMG_CFG_HEIGHT(height) | SCL_IMG_CFG_WIDTH(width);
            m_IsiRegistersPtr->OUT_BUF_PITCH = width * m_PlaneBytesPerPixel; // FIXME bytes per pixel
        }

        {
            UINT32 imgCtrl = IMG_CTRL_CSC_BYP_BIT; // Bypass color conversion and scaler.

            m_IsiRegistersPtr->IMG_CTRL = imgCtrl | resultPixFmt; //Output image format

            /* set scale ratio 1:1. 0x1000 means 01.0000 0000 0000 (2-bit integer and 12 bit fraction) */
            m_IsiRegistersPtr->SCALE_FACTOR = SCALE_FACTOR_X_SCALE(0x1000) | SCALE_FACTOR_Y_SCALE(0x1000);
            /* set initial vertical offsetto 0 */
            m_IsiRegistersPtr->SCALE_OFFSET = 0;
        }
        {
            m_IsiRegistersPtr->OUT_BUF_CTRL = OUT_BUF_CTRL_MAX_WR_BEATS_Y_BIT | OUT_BUF_CTRL_MAX_WR_BEATS_UV_BIT | OUT_BUF_CTRL_PANIC_SET_THD_Y(0xA); // 16 beat axi write burst instead of 8 beat.
        }

        {
            m_IsiRegistersPtr->CTRL = CTRL_CLK_EN_BIT
                | CTRL_BYPASS_BIT
                | CTRL_VC_ID(Config.mipiChannel)
                | CTRL_SRC(m_MipiCsiSrc);
        }
    }
    return status;
}

