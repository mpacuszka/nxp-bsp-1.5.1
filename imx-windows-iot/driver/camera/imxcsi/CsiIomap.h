/*
 * Copyright 2017-2022 NXP
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

#pragma once

#include <Ntintsafe.h>

/** CSI - Register Layout Typedef */
#pragma pack( push )
#pragma pack( 4 )
struct CSI_REGS
{
    /*__IO*/ UINT32 CR1;                               /**< CSI Control Register 1, offset: 0x0 */
    /*__IO*/ UINT32 CR2;                               /**< CSI Control Register 2, offset: 0x4 */
    /*__IO*/ UINT32 CR3;                               /**< CSI Control Register 3, offset: 0x8 */
    /*__I */ UINT32 STATFIFO;                          /**< CSI Statistic FIFO Register, offset: 0xC */
    /*__I */ UINT32 RFIFO;                             /**< CSI RX FIFO Register, offset: 0x10 */
    /*__IO*/ UINT32 RXCNT;                             /**< CSI RX Count Register, offset: 0x14 */
    /*__IO*/ UINT32 SR;                                /**< CSI status Register, offset: 0x18 */
    /*    */ UINT8  RESERVED_0[4];
    /*__IO*/ UINT32 DMASA_STATFIFO;                    /**< CSI DMA Start Address Register - for STATFIFO, offset: 0x20 */
    /*__IO*/ UINT32 DMATS_STATFIFO;                    /**< CSI DMA Transfer Size Register - for STATFIFO, offset: 0x24 */
    /*__IO*/ UINT32 DMASA_FB1;                         /**< CSI DMA Start Address Register - for Frame Buffer1, offset: 0x28 */
    /*__IO*/ UINT32 DMASA_FB2;                         /**< CSI DMA Transfer Size Register - for Frame Buffer2, offset: 0x2C */
    /*__IO*/ UINT32 FBUF_PARA;                         /**< CSI Frame Buffer Parameter Register, offset: 0x30 */
    /*__IO*/ UINT32 IMAG_PARA;                         /**< CSI Image Parameter Register, offset: 0x34 */
    /*    */ UINT8  RESERVED_1[16];
    /*__IO*/ UINT32 CR18;                              /**< CSI Control Register 18, offset: 0x48 */
    /*__IO*/ UINT32 CR19;                              /**< CSI Control Register 19, offset: 0x4C */
    /*__IO*/ UINT32 CR20;                              /**< CSI Control Register 20, offset: 0x50 */
    /*__IO*/ UINT32 CR[256];                           /**< CSI Control Register, array offset: 0x54, array step: 0x4 */
};
#pragma pack( pop )

#pragma warning( push )
#pragma warning(disable: 4505) //Unreferenced local function has been removed

    /*! @name CR1 - CSI Control Register 1 */
    /*! @{ */

    const static UINT32 CR1_10B_PIXEL_BIT = 0x1U; // 10-bit/(0)8-bit data for each pixel
    const static UINT32 CR1_REDGE_BIT = 0x2U; // Pixel data is latched at the rising edge of CSI_PIXCLK
    const static UINT32 CR1_INV_PCLK_BIT = 0x4U; // CSI_PIXCLK is inverted before applied to internal circuitry
    const static UINT32 CR1_INV_DATA_BIT = 0x8U; // CSI_D[7:0] data lines are inverted/(0)not inverted before applied to internal circuitry
    const static UINT32 CR1_GCLK_MODE_BIT = 0x10U; // Gated clock mode. Pixel clock signal is valid only when HSYNC is active. (0) Non-gated clock mode. All incoming pixel clocks are valid. HSYNC is ignored.
    const static UINT32 CR1_CLR_RXFIFO_BIT = 0x20U; //
    const static UINT32 CR1_CLR_STATFIFO_BIT = 0x40U; //
    const static UINT32 CR1_PACK_DIR_BIT = 0x80U; //
     /*! PACK_DIR
          *  0b0..Pack from LSB first. For image data, 0x11, 0x22, 0x33, 0x44, it will appear as 0x44332211 in RX FIFO. For
          *       stat data, 0xAAAA, 0xBBBB, it will appear as 0xBBBBAAAA in STAT FIFO.
          *  0b1..Pack from MSB first. For image data, 0x11, 0x22, 0x33, 0x44, it will appear as 0x11223344 in RX FIFO. For
          *       stat data, 0xAAAA, 0xBBBB, it will appear as 0xAAAABBBB in STAT FIFO.
          */
    const static UINT32 CR1_FCC_BIT = 0x100U; // Synchronous/(0)Asynchronous FIFO clear is selected.
    const static UINT32 CR1_MCLKEN_BIT = 0x200U; // Reserved
    const static UINT32 CR1_CCIR_EN_BIT = 0x400U; // BT.656/(0)Traditional interface is selected.
    const static UINT32 CR1_HSYNC_POL_BIT = 0x800U; //HSYNC is active high/(0)low
    const static UINT32 CR1_MCLKDIV_BIT1 = 0x1000U; // Reserved
    // const static UINT32 CR1_HISTOGRAM_CALC_DONE_IE_BIT = 0x1000U; //Histogram done interrupt enable
    const static UINT32 CR1_SOF_INTEN_BIT = 0x10000U; // SOF interrupt enable
    const static UINT32 CR1_SOF_POL_BIT = 0x20000U; // interrupt is generated on SOF rising/(0)falling edge
    const static UINT32 CR1_RXFF_INTEN_BIT = 0x40000U; /// RxFIFO full interrupt enable
    const static UINT32 CR1_FB1_DMA_DONE_INTEN_BIT = 0x80000U; /// Frame Buffer1 DMA Transfer Done interrupt enable
    const static UINT32 CR1_FB2_DMA_DONE_INTEN_BIT = 0x100000U; /// Frame Buffer2 DMA Transfer Done interrupt enable
    const static UINT32 CR1_STATFF_INTEN_BIT = 0x200000U; // /STATFIFO full interrupt enable
    const static UINT32 CR1_SFF_DMA_DONE_INTEN_BIT = 0x400000U; // STATFIFO DMA Transfer Done interrupt enable
    const static UINT32 CR1_RF_OR_INTEN_BIT = 0x1000000U; // RxFIFO overrun interrupt is enabled
    const static UINT32 CR1_SF_OR_INTEN_BIT = 0x2000000U; // STATFIFO overrun interrupt is enabled

    const static UINT32 CR1_COF_INT_EN_BIT = 0x4000000U; // COF interrupt is enabled
    const static UINT32 CR1_VIDEO_MODE_BIT = 0x8000000U; // Interlace mode is selected
    const static UINT32 CR1_EOF_INT_EN_BIT = 0x20000000U; // EOF interrupt is generated when RX count value is reached.
    const static UINT32 CR1_EXT_VSYNC_BIT = 0x40000000U; // Internal/(0)External VSYNC mode
    const static UINT32 CR1_SWAP16_EN_BIT = 0x80000000U; // Enable swapping
    /*! @} */

    /*! @name CR2 - CSI Control Register 2 */
    /*! @{ */
    const static UINT32 CR2_HSC_MASK = 0xFFU;
    const static UINT32 CR2_VSC_MASK = 0xFF00U;
    const static UINT32 CR2_VSC_SHIFT = 8U;

    const static UINT32 CR2_LVRM_MASK = 0x70000U;
    const static UINT32 CR2_LVRM(UINT32 LVRM)
    /*! LVRM
     *  0b000..512 x 384
     *  0b001..448 x 336
     *  0b010..384 x 288
     *  0b011..384 x 256
     *  0b100..320 x 240
     *  0b101..288 x 216
     *  0b110..400 x 300
     */
    {
        return (LVRM << 16U) & CR2_LVRM_MASK;
    }

    const static UINT32 CR2_BTS_MASK = 0x180000U;
    const static UINT32 CR2_BTS(UINT32 BTS)
    /*! BTS
     *  0b00..GR
     *  0b01..RG
     *  0b10..BG
     *  0b11..GB
     */
    {
        return (BTS << 19U) & CR2_BTS_MASK;
    }

    const static UINT32 CR2_SCE_BIT = 0x800000U; // Skip count enable
    const static UINT32 CR2_AFS_MASK = 0x3000000U;
    const static UINT32 CR2_AFS(UINT32 AFS)
    /*! AFS
     *  0b00..Abs Diff on consecutive green pixels
     *  0b01..Abs Diff on every third green pixels
     *  0b1x..Abs Diff on every four green pixels
     */
    {
        return (AFS << 24U) & CR2_AFS_MASK;
    }

    const static UINT32 CR2_DRM_BIT = 0x4000000U; // 0 - Stats grid of 8 x 6, 1 - Stats grid of 8 x 12
    /*! DRM
     *  0b0..Stats grid of 8 x 6
     *  0b1..Stats grid of 8 x 12
     */
    const static UINT32 CR2_DMA_BURST_TYPE_SFF_MASK = 0x3000000U;
    const static UINT32 CR2_DMA_BURST_TYPE_SFF(UINT32 DMA_BURST_TYPE_SFF)
    /*! DMA_BURST_TYPE_SFF
     *  0bx0..INCR8
     *  0b01..INCR4
     *  0b11..INCR16
     */
    {
        return (DMA_BURST_TYPE_SFF << 28U) & CR2_DMA_BURST_TYPE_SFF_MASK;
    }
    const static UINT32 CR2_DMA_BURST_TYPE_RFF_MASK = 0xC0000000U;
    const static UINT32 CR2_DMA_BURST_TYPE_RFF(UINT32 DMA_BURST_TYPE_RFF)
    /*! DMA_BURST_TYPE_RFF
     *  0bx0..INCR8
     *  0b01..INCR4
     *  0b11..INCR16
     */
    {
        return (DMA_BURST_TYPE_RFF << 30U) & CR2_DMA_BURST_TYPE_RFF_MASK;
    }

    /*! @} */

    /*! @name CR3 - CSI Control Register 3 */
    /*! @{ */
    const static UINT32 CR3_ECC_AUTO_EN_BIT = 0x1U; // Auto Error correction is enabled.
    const static UINT32 CR3_ECC_INT_EN_BIT = 0x2U; // Interrupt is generated when error is detected.
    const static UINT32 CR3_ZERO_PACK_EN_BIT = 0x4U; // Zero packing enabled
    const static UINT32 CR3_SENSOR_16BITS_BIT = 0x8U; // 0 - One 8-bit sensor is connected. 1 - One 16-bit sensor is connected.
    const static UINT32 CR3_RxFF_LEVEL_MASK = 0x70U;
    const static UINT32 CR3_RxFF_LEVEL_SHIFT = 0x4U;
    /*! RxFF_LEVEL
     *  0b000..4 Double words
     *  0b001..8 Double words
     *  0b010..16 Double words
     *  0b011..24 Double words
     *  0b100..32 Double words
     *  0b101..48 Double words
     *  0b110..64 Double words
     *  0b111..96 Double words
     */
    const static UINT32 CR3_HRESP_ERR_EN_BIT = 0x80U; // Enable hresponse error interrupt
    const static UINT32 CR3_STATFF_LEVEL_MASK = 0x700U;
    const static UINT32 CR3_STATFF_LEVEL_SHIFT = 8U;
    /*! STATFF_LEVEL
     *  0b000..4 Double words
     *  0b001..8 Double words
     *  0b010..12 Double words
     *  0b011..16 Double words
     *  0b100..24 Double words
     *  0b101..32 Double words
     *  0b110..48 Double words
     *  0b111..64 Double words
     */
    const static UINT32 CR3_DMA_REQ_EN_SFF_BIT = 0x800U; // Enable the dma request
    const static UINT32 CR3_DMA_REQ_EN_RFF_BIT = 0x1000U; // Enable the dma request
    const static UINT32 CR3_DMA_REFLASH_SFF_BIT = 0x2000U; // Reflash the embedded DMA controller
    const static UINT32 CR3_DMA_REFLASH_RFF_BIT = 0x4000U; // Reflash the embedded DMA controller
    const static UINT32 CR3_FRMCNT_RST_BIT = 0x8000U; // Reset frame counter immediately


    const static UINT32 CR3_FRMCNT_GET(UINT32 CR3)
    {
        return (CR3 & 0xFFFF0000U) >> 16U;
    }
    /*! @} */


    /*! @name FBUF_PARA - CSI Frame Buffer Parameter Register */
    /*! @{ */
    const static UINT32  FBUF_PARA_FBUF_STRIDE_SHIFT = 0U;
    const static UINT32  FBUF_PARA_DEINTERLACE_STRIDE_SHIFT = 16U;
    /*! @} */

    /*! @name IMAG_PARA - CSI Image Parameter Register */
    /*! @{ */
    const static UINT32 IMAG_PARA_IMAGE_HEIGHT_SHIFT = 0U;
    const static UINT32 IMAG_PARA_IMAGE_WIDTH_SHIFT = 16U;
    /*! @} */

    /*! @name SR - CSI status Register */
    /*! @{ */
    const static UINT32 SR_DRDY_BIT = 0x1U; // At least 1 datum (word) is ready in RXFIFO
    const static UINT32 SR_ECC_INT_BIT = 0x2U; // Error is detected in BT.656 coding
    const static UINT32 SR_HISTOGRAM_CALC_DONE_INT_BIT = 0x4U; // Histogram calculation is done and driver can access the PIXEL_COUNTERS(CSI_CSICR21~CSI_CSICR276) to get the gray level
    const static UINT32 SR_HRESP_ERR_INT_BIT = 0x80U; // Hresponse error is detected.
    const static UINT32 SR_COF_INT_BIT = 0x2000U; // Change of video field is detected.
    const static UINT32 SR_F1_INT_BIT = 0x4000U; // Field 1 of video is about to start.
    const static UINT32 SR_F2_INT_BIT = 0x8000U; // Field 2 of video is about to start.
    const static UINT32 SR_SOF_INT_BIT = 0x10000U; // SOF is detected.
    const static UINT32 SR_EOF_INT_BIT = 0x20000U; // EOF is detected.
    const static UINT32 SR_RxFF_INT_BIT = 0x40000U; // RxFIFO is full.
    const static UINT32 SR_DMA_TSF_DONE_FB1_BIT = 0x80000U; // DMA transfer is completed.
    const static UINT32 SR_DMA_TSF_DONE_FB2_BIT = 0x100000U; // DMA transfer is completed.
    const static UINT32 SR_STATFF_FULL_INT_BIT =  0x200000U; // STATFIFO is full.
    const static UINT32 SR_DMA_TSF_DONE_SFF_BIT = 0x400000U; // DMA transfer is completed.
    const static UINT32 SR_RF_OR_INT_BIT = 0x1000000U; // RXFIFO has overflowed.
    const static UINT32 SR_SF_OR_INT_BIT = 0x2000000U; // STATFIFO has overflowed.
    const static UINT32 SR_DMA_FIELD1_DONE_BIT = 0x4000000U;
    const static UINT32 SR_DMA_FIELD2_DONE_BIT = 0x8000000U;
    const static UINT32 SR_BASEADDR_CHHANGE_ERROR_BIT = 0x10000000U; // When using base address switching enable, this bit will be 1 when switching occur before DMA complete. This bit will be clear by writing 1. When this interrupt happens, follow the steps listed below. 1. Unassert the CSI enable, CSIx_CSICR18 bit31, 2. Reflash the DMA, assert the CSIX_CSICR3 bit 14, 3. Assert the CSI enable, CSIx_CSICR18 bit31
    /*! @} */

    /*! @name CR18 - CSI Control Register 18 */
    /*! @{ */
    // Reserved const static UINT32 CR18_NTSC_EN_BIT = 0x1U; // 0 - PAL, 1 - NTSC
    // Reserved const static UINT32 CR18_TVDECODER_IN_EN_BIT = 0x2U; //
    const static UINT32 CR18_DEINTERLACE_EN_BIT = 0x4U; // Deinterlace enable. This bit is used to select the output method When input is standard CCIR656 video.
    const static UINT32 CR18_PARALLEL24_EN_BIT = 0x8U; // When input is parallel rgb888/yuv444 24bit, this bit can be enabled.
    const static UINT32 CR18_BASEADDR_SWITCH_EN_BIT = 0x10U; // When this bit is enabled, CSI DMA will switch the base address according to BASEADDR_SWITCH_SEL rather than atomically by DMA completed.
    const static UINT32 CR18_BASEADDR_SWITCH_SEL_BIT = 0x20U; // Switching base address at the edge of (0)vsync (1) the first data of each frame
    const static UINT32 CR18_DMA_FIELD0_DONE_IE_BIT = 0x40U; // FIELD0_DONE interrupt enable
    const static UINT32 CR18_DMA_FIELD1_DONE_IE_BIT = 0x80U; // DMA_FIELD1_DONE_IE interrupt enabled
    const static UINT32 CR18_LAST_DMA_REQ_SEL_BIT = 0x100U; // LAST_DMA_REQ_SEL (0)fifo_full_level , (1)hburst_length
    const static UINT32 CR18_BASEADDR_CHANGE_ERROR_IE_BIT = 0x200U; // BASEADDR_CHANGE_ERROR_IE interrupt enable

    const static UINT32 CR18_RGB888A_FORMAT_SEL_BIT = 0x400U; // RGB888A_FORMAT_SEL (0) {8'h0, data[23:0]}, (1) {data[23:0], 8'h0}
    const static UINT32 CR18_AHB_HPROT_MASK = 0xF000U;
    const static UINT32 CR18_AHB_HPROT(UINT32 AHB_HPROT) // Hprot value in AHB bus protocol.
    {
        return (((UINT32)(AHB_HPROT << 12U)) & CR18_AHB_HPROT_MASK);
    }
    const static UINT32 CR18_LCDIF_BUFFER_LINES_MASK = 0x30000U;
    const static UINT32 CR18_LCDIF_BUFFER_LINES(UINT32 LCDIF_BUFFER_LINES) // Hprot value in AHB bus protocol.
    {
        return (((UINT32)(LCDIF_BUFFER_LINES << 16U)) & CR18_LCDIF_BUFFER_LINES_MASK);
    }
    const static UINT32 CR18_MASK_OPTION_MASK = 0xC0000U;
    const static UINT32 CR18_MASK_OPTION_DONT_SKIP = (0x1 << 18U);
    const static UINT32 CR18_MASK_OPTION_SKIP_FIRST_FRAME = (0x0 << 18U);
    const static UINT32 CR18_MASK_OPTION_SKIP_SECOND_FRAME = (0x2 << 18U);
    const static UINT32 CR18_MASK_OPTION_IGNORE_CSI_ENABLE = (0x3 << 18U);
    // #define CSI_CR18_MASK_OPTION(x)                  (((uint32_t)(((uint32_t)(x)) << CSI_CR18_MASK_OPTION_SHIFT)) & CSI_CR18_MASK_OPTION_MASK)
    /*! MASK_OPTION
     *  0b00..Writing to memory (OCRAM or external DDR) from first completely frame, when using this option, the CSI_ENABLE should be 1.
     *  0b01..Writing to memory when CSI_ENABLE is 1.
     *  0b10..Writing to memory from second completely frame, when using this option, the CSI_ENABLE should be 1.
     *  0b11..Writing to memory when data comes in, not matter the CSI_ENABLE is 1 or 0.
     */
    const static UINT32 CR18_MIPI_DOUBLE_CMPNT_BIT = 0x100000U; // Double component per clock cycle in YUV422 formats. (0)Single (half pixel per clock cycle), (1)Double component per clock cycle (a pixel per clock cycle)
    const static UINT32 CR18_MIPI_YU_SWAP_BIT = 0x200000U; // MIPI_YU_SWAP - It only works in MIPI CSI YUV422 double component mode.
    const static UINT32 CR18_DATA_FROM_MIPI_BIT = 0x400000U; // (0) Data from parallel sensor, (1) Data from MIPI
    const static UINT32 CR18_LINE_STRIDE_EN_BIT = 0x1000000U; // When the line width are not the multiple of the burst length, assert this bit.
    const static UINT32 CR18_MIPI_DATA_FORMAT_MASK = 0x7E000000U; // Image Data Format
    const static UINT32 CR18_MIPI_DATA_FORMAT_YUV420_8B = (0x18 << 25U);
    const static UINT32 CR18_MIPI_DATA_FORMAT_YUV420_10B = (0x19 << 25U);
    const static UINT32 CR18_MIPI_DATA_FORMAT_YUV422_8B = (0x1E << 25U);
    const static UINT32 CR18_MIPI_DATA_FORMAT_YUV422_10B = (0x1F << 25U);
    const static UINT32 CR18_MIPI_DATA_FORMAT_RGB565 = (0x22 << 25U);
    const static UINT32 CR18_MIPI_DATA_FORMAT_RGB888 = (0x24 << 25U);
    const static UINT32 CR18_MIPI_DATA_FORMAT_RAW8 = (0x2a << 25U); //
    const static UINT32 CR18_MIPI_DATA_FORMAT_RAW10 = (0x2b << 25U);
    const static UINT32 CR18_MIPI_DATA_FORMAT_RAW12 = (0x2c << 25U);
    const static UINT32 CR18_MIPI_DATA_FORMAT(UINT32 DATA_FORMAT)
    {
        return (((UINT32)(DATA_FORMAT << 25U)) & CR18_MIPI_DATA_FORMAT_MASK);
    }
    const static UINT32 CR18_CSI_ENABLE_BIT = 0x80000000U; // Image Data Format
    /*! @} */

    /*
     * The interrupt enable bits are in registers CSICR1[16:31], CSICR3[0:7],
     * and CSICR18[2:9]. So merge them into an UINT32 value, place CSICR18 control
     * bits to [8:15].
     */
    const static UINT32 CR1_INT_EN_MASK = CR1_RF_OR_INTEN_BIT;
    const static UINT32 CR3_INT_EN_MASK = CR3_HRESP_ERR_EN_BIT;
    const static UINT32 CR18_INT_EN_MASK = CR18_BASEADDR_CHANGE_ERROR_IE_BIT;

    const static UINT32 CR1_FRAME_INT_EN_MASK = (CR1_SOF_INTEN_BIT | CR1_FB1_DMA_DONE_INTEN_BIT | CR1_FB2_DMA_DONE_INTEN_BIT);
    const static UINT32 CR3_FRAME_INT_EN_MASK = 0x0;
    const static UINT32 CR18_FRAME_INT_EN_MASK = 0x0;

#pragma warning( pop ) // disable: 4505


struct Resources_t {
    CSI_REGS *m_CsiRegistersPtr;
    UINT32 coreClockFrequencyHz;
    PCHAR  csiDevNameA;
    UINT32 m_CpuId;
};
