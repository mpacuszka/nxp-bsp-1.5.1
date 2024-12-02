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

/** ISI - Register Layout */
#pragma pack( push )
#pragma pack( 4 )
struct ISI_REGS
{
    UINT32 CTRL;            /**< Channel Control Register, offset: 0x0 */
    UINT32 IMG_CTRL;        /**< Channel Image Control Register, offset: 0x4 */
    UINT32 OUT_BUF_CTRL;    /**< Channel Output Buffer Control Register, offset: 0x8 */
    UINT32 IMG_CFG;         /**< Channel Image Configuration, offset: 0xC */
    UINT32 IER;             /**< Channel Interrupt Enable Register, offset: 0x10 */
    UINT32 STS;             /**< Channel Status Register, offset: 0x14 */
    UINT32 SCALE_FACTOR;    /**< Channel Scale Factor Register, offset: 0x18 */
    UINT32 SCALE_OFFSET;    /**< Channel Scale Offset Register, offset: 0x1C */
    UINT32 CROP_ULC;        /**< Channel Crop Upper Left Corner Coordinate Register, offset: 0x20 */
    UINT32 CROP_LRC;        /**< Channel Crop Lower Right Corner Coordinate Register, offset: 0x24 */
    UINT32 CSC_COEFF0;      /**< Channel Color Space Conversion Coefficient Register 0, offset: 0x28 */
    UINT32 CSC_COEFF1;      /**< Channel Color Space Conversion Coefficient Register 1, offset: 0x2C */
    UINT32 CSC_COEFF2;      /**< Channel Color Space Conversion Coefficient Register 2, offset: 0x30 */
    UINT32 CSC_COEFF3;      /**< Channel Color Space Conversion Coefficient Register 3, offset: 0x34 */
    UINT32 CSC_COEFF4;      /**< Channel Color Space Conversion Coefficient Register 4, offset: 0x38 */
    UINT32 CSC_COEFF5;      /**< Channel Color Space Conversion Coefficient Register 5, offset: 0x3C */
    UINT32 ROI_0_ALPHA;     /**< Channel Alpha Value Register for Region of Interest 0, offset: 0x40 */
    UINT32 ROI_0_ULC;       /**< Channel Upper Left Coordinate Register for Region of Interest 0, offset: 0x44 */
    UINT32 ROI_0_LRC;       /**< Channel Lower Right Coordinate Register for Region of Interest 0, offset: 0x48 */
    UINT32 ROI_1_ALPHA;     /**< Channel Alpha Value Register for Region of Interest 1, offset: 0x4C */
    UINT32 ROI_1_ULC;       /**< Channel Upper Left Coordinate Register for Region of Interest 1, offset: 0x50 */
    UINT32 ROI_1_LRC;       /**< Channel Lower Right Coordinate Register for Region of Interest 1, offset: 0x54 */
    UINT32 ROI_2_ALPHA;     /**< Channel Alpha Value Register for Region of Interest 2, offset: 0x58 */
    UINT32 ROI_2_ULC;       /**< Channel Upper Left Coordinate Register for Region of Interest 2, offset: 0x5C */
    UINT32 ROI_2_LRC;       /**< Channel Lower Right Coordinate Register for Region of Interest 2, offset: 0x60 */
    UINT32 ROI_3_ALPHA;     /**< Channel Alpha Value Register for Region of Interest 3, offset: 0x64 */
    UINT32 ROI_3_ULC;       /**< Channel Upper Left Coordinate Register for Region of Interest 3, offset: 0x68 */
    UINT32 ROI_3_LRC;       /**< Channel Lower Right Coordinate Register for Region of Interest 3, offset: 0x6C */
    UINT32 OUT_BUF1_ADDR_Y; /**< Channel RGB or Luma (Y) Output Buffer 1 Address, offset: 0x70 */
    UINT32 OUT_BUF1_ADDR_U; /**< Channel Chroma (U/Cb/UV/CbCr) Output Buffer 1 Address, offset: 0x74 */
    UINT32 OUT_BUF1_ADDR_V; /**< Channel Chroma (V/Cr) Output Buffer 1 Address, offset: 0x78 */
    UINT32 OUT_BUF_PITCH;   /**< Channel Output Buffer Pitch, offset: 0x7C */
    UINT32 IN_BUF_ADDR;     /**< Channel Input Buffer Address, offset: 0x80 */
    UINT32 IN_BUF_PITCH;    /**< Channel Input Buffer Pitch, offset: 0x84 */
    UINT32 MEM_RD_CTRL;     /**< Channel Memory Read Control, offset: 0x88 */
    UINT32 OUT_BUF2_ADDR_Y; /**< Channel RGB or Luma (Y) Output Buffer 2 Address, offset: 0x8C */
    UINT32 OUT_BUF2_ADDR_U; /**< Channel Chroma (U/Cb/UV/CbCr) Output Buffer 2 Address, offset: 0x90 */
    UINT32 OUT_BUF2_ADDR_V; /**< Channel Chroma (V/Cr) Output Buffer 2 Address, offset: 0x94 */
    UINT32 SCL_IMG_CFG;     /**< Channel Scaled Image Configuration, offset: 0x98 */
    UINT32 FLOW_CTRL;       /**< Channel Flow Control Register, offset: 0x9C */
};
#pragma pack( pop )

#pragma warning( push )
#pragma warning(disable: 4505) //Unreferenced local function has been removed
namespace ISI_REG_BITFIELDS
{
    /*! @name CTRL - Channel Control Register */
    /*! @{ */
    const static UINT32 CTRL_SRC_MASK = 0x3; // Three bits on QXP, Single bit 8MP
    const static UINT32 CTRL_SRC_SHIFT = 0U;
    const static UINT32 CTRL_SRC(UINT32 SRC_ID)
    {
        return (SRC_ID << CTRL_SRC_SHIFT) & CTRL_SRC_MASK;
    }
    const static UINT32 CTRL_SRC_TYPE_BIT = 0x10U;
    const static UINT32 CTRL_SRC_TYPE_DEVICE = 0x0U;
    const static UINT32 CTRL_SRC_TYPE_MEMORY = 0x10U;
    const static UINT32 CTRL_VC_ID_MASK = 0xC0U;
    const static UINT32 CTRL_VC_ID_SHIFT = 6U;
    const static UINT32 CTRL_VC_ID(UINT32 VIRTUAL_CHANNEL_ID)
    /*! VC_ID - Virtual channel ID
     *  0b00..Virtual Channel 0 selected or no virtual channel used
     *  0b01..Virtual Channel 1 selected
     *  0b10..Virtual Channel 2 selected
     *  0b11..Virtual Channel 3 selected
     */
    {
        return (VIRTUAL_CHANNEL_ID << CTRL_VC_ID_SHIFT) & CTRL_VC_ID_MASK;
    }
    const static UINT32 CTRL_SW_RST_BIT = 0x1000000U;
    const static UINT32 CTRL_CHAIN_BUF_MASK = 0x6000000U;
    const static UINT32 CTRL_CHAIN_BUF_SHIFT = 25U;
    const static UINT32 CTRL_CHAIN_BUF(UINT32 MODE)
    /*! CHAIN_BUF - Chain line buffer control
     *  0b00..No line buffers chained (supports 2048 or less horizontal resolution)
     *  0b01..2 line buffers chained (supports 4096 horizontal resolution). Line buffers of channels 'n' and 'n+1' are chained.
     *  0b10..4 line buffers chained (supports 8192 horizontal resolution). Line buffers of channels 'n', 'n+1', 'n+2' and 'n+3' are chained.
     *  0b11..Reserved for future use
     */
    {
        return (MODE << CTRL_CHAIN_BUF_SHIFT) & CTRL_CHAIN_BUF_MASK;
    }
    const static UINT32 CTRL_BYPASS_BIT = 0x20000000U;
    const static UINT32 CTRL_CLK_EN_BIT = 0x40000000U;
    const static UINT32 CTRL_EN_BIT = 0x80000000U;
    /*! @} */

    /*! @name IMG_CTRL - Channel Image Control Register */
    /*! @{ */
    const static UINT32 IMG_CTRL_CSC_BYP_BIT = 0x1U;
    const static UINT32 IMG_CTRL_CSC_MODE_MASK = 0x6U;
    const static UINT32 IMG_CTRL_CSC_MODE_SHIFT = 0x1U;
    const static UINT32 IMG_CTRL_CSC_MODE_YUV_TO_RGB = (0x0U << IMG_CTRL_CSC_MODE_SHIFT); // 0b00..Convert from YUV to RGB
    const static UINT32 IMG_CTRL_CSC_MODE_YCbCr_TO_RGB = (0x1U << IMG_CTRL_CSC_MODE_SHIFT); // 0b01..Convert from YCbCr to RGB
    const static UINT32 IMG_CTRL_CSC_MODE_RGB_TO_YUV = (0x2U << IMG_CTRL_CSC_MODE_SHIFT); // 0b10..Convert from RGB to YUV
    const static UINT32 IMG_CTRL_CSC_MODE_RGB_TO_YCbCr = (0x3U << IMG_CTRL_CSC_MODE_SHIFT); // 0b11..Convert from RGB to YCbCr

    const static UINT32 IMG_CTRL_YCBCR_MODE_BIT = 0x8U;
    const static UINT32 IMG_CTRL_HFLIP_EN_BIT = 0x20U;
    const static UINT32 IMG_CTRL_VFLIP_EN_BIT = 0x40U;
    const static UINT32 IMG_CTRL_CROP_EN_BIT = 0x80U;
    const static UINT32 IMG_CTRL_DEC_Y_SHIFT = 0x8U;
    const static UINT32 IMG_CTRL_DEC_Y_MASK = 0x300U;
    const static UINT32 IMG_CTRL_DEC_Y(UINT32 PREDECIMATE)
    /*! DEC_Y - Vertical pre-decimation control
     *  0b00..Pre-decimation filter is disabled. Bilinear scaling filter is still operational.
     *  0b01..Decimate by 2
     *  0b10..Decimate by 4
     *  0b11..Decimate by 8
     */
    {
        return (PREDECIMATE << IMG_CTRL_DEC_Y_SHIFT) & IMG_CTRL_DEC_Y_MASK;
    }
    const static UINT32 IMG_CTRL_DEC_X_SHIFT = 10U;
    const static UINT32 IMG_CTRL_DEC_X_MASK = 0xC00U;
    const static UINT32 IMG_CTRL_DEC_X(UINT32 PREDECIMATE)
    /*! DEC_X - Horizontal pre-decimation control
     *  0b00..Pre-decimation filter is disabled. Bilinear scaling filter is still operational.
     *  0b01..Decimate by 2
     *  0b10..Decimate by 4
     *  0b11..Decimate by 8
     */
    {
        return (PREDECIMATE << IMG_CTRL_DEC_X_SHIFT) & IMG_CTRL_DEC_X_MASK;
    }
    const static UINT32 IMG_CTRL_DEINT_SHIFT = 12U;
    const static UINT32 IMG_CTRL_DEINT_MASK = 0x7000U;
    const static UINT32 IMG_CTRL_DEINT(UINT32 MODE)
    /*! DEINT - De-interlace control
     *  0b000, 0b001..No de-interlacing done
     *  0b010..Weave de-interlacing (Odd, Even) method used
     *  0b011..Weave de-interlacing (Even, Odd) method used
     *  0b100..Blending or linear interpolation (Odd + Even) de-interlacing method used
     *  0b101..Blending or linear interpolation (Even + Odd) de-interlacing method used
     *  0b110, 0b111..Line doubling de-interlacing method used. Both Odd and Even fields are doubled.
     */
    {
        return (MODE << IMG_CTRL_DEINT_SHIFT) & IMG_CTRL_DEINT_MASK;
    }
    const static UINT32 IMG_CTRL_GBL_ALPHA_EN_BIT = 0x8000U;
    const static UINT32 IMG_CTRL_GBL_ALPHA_VAL_SHIFT = 10U;
    const static UINT32 IMG_CTRL_GBL_ALPHA_VAL_MASK = 0xC00U;
    const static UINT32 IMG_CTRL_GBL_ALPHA_VAL(UINT32 ALPHA_VAL)
    /*! GBL_ALPHA_VAL - Global alpha value
     *  0b00000000-0b11111111..Alpha value to be inserted with all RGB pixels
     */
    {
        return (ALPHA_VAL << IMG_CTRL_GBL_ALPHA_VAL_SHIFT) & IMG_CTRL_GBL_ALPHA_VAL_MASK;
    }

    const static UINT32 IMG_CTRL_FORMAT_MASK = 0x3F000000U; // FORMAT - Output image format
    const static UINT32 IMG_CTRL_FORMAT_SHIFT = 24U;
    const static UINT32 IMG_CTRL_FORMAT_RGBA8888 = (0x0 << IMG_CTRL_FORMAT_SHIFT); // RGB format with alpha in LSB; 8-bits per component. 'A' indicates alpha value.
    const static UINT32 IMG_CTRL_FORMAT_ABGR8888 = (0x1 << IMG_CTRL_FORMAT_SHIFT); // BGR format with alpha in MSB; 8-bits per component. 'A' indicates alpha value.
    const static UINT32 IMG_CTRL_FORMAT_ARGB8888 = (0x2 << IMG_CTRL_FORMAT_SHIFT); // RGB format with alpha in MSB; 8-bits per component. 'A' indicates alpha value.
    const static UINT32 IMG_CTRL_FORMAT_XRGB8888 = (0x5 << IMG_CTRL_FORMAT_SHIFT); // RGB format with waste bits in MSB; 8-bits per component. 'X' indicates waste bits.
    const static UINT32 IMG_CTRL_FORMAT_RGB888P = (0x6 << IMG_CTRL_FORMAT_SHIFT); // RGB format with 8-bits per color component (packed into 24-bits).
    const static UINT32 IMG_CTRL_FORMAT_BGR888P = (0x7 << IMG_CTRL_FORMAT_SHIFT); // BGR format with 8-bits per color component (packed into 24-bits).
    const static UINT32 IMG_CTRL_FORMAT_RGB565 = (0xA << IMG_CTRL_FORMAT_SHIFT); // RGB format with 5-bits of R, B; 6-bits of G (packed into 16-bits WORD).
    const static UINT32 IMG_CTRL_FORMAT_YUV422_8B_1P = (0x20 << IMG_CTRL_FORMAT_SHIFT); // 8-bits per color component; 1-plane, YUV interleaved packed bytes
    const static UINT32 IMG_CTRL_FORMAT_YUV420_8B_2P = (0x31 << IMG_CTRL_FORMAT_SHIFT); // 8-bits per color component; 2-plane, UV interleaved packed bytes
    const static UINT32 IMG_CTRL_FORMAT(UINT32 DATA_FORMAT)
        /*! FORMAT - Output image format
         *  0b000000..RGBA8888 - RGB format with alpha in LSB; 8-bits per component. 'A' indicates alpha value.
         *  0b000001..ABGR8888 - BGR format with alpha in MSB; 8-bits per component. 'A' indicates alpha value.
         *  0b000010..ARGB8888 - RGB format with alpha in MSB; 8-bits per component. 'A' indicates alpha value.
         *  0b000011..RGBX888 - RGB format with 8-bits per color component (unpacked and MSB-aligned in 32-bit DWORD). 'X' indicates the waste bits.
         *  0b000100..XBGR888 - BGR format with 8-bits per color component (unpacked and LSB aligned in 32-bit DWORD). 'X' indicates the waste bits.
         *  0b000101..XRGB888 - RGB format with 8-bits per color component (unpacked and LSB aligned in 32-bit DWORD). 'X' indicates the waste bits.
         *  0b000110..RGB888P - RGB format with 8-bits per color component (packed into 24-bits). No waste bits.
         *  0b000111..BGR888P - BGR format with 8-bits per color component (packed into 24-bits). No waste bits.
         *  0b001000..A2BGR10 - BGR format with 2-bits alpha in MSB; 10-bits per color component. 'A' indicates alpha value.
         *  0b001001..A2RGB10 - RGB format with 2-bits alpha in MSB; 10-bits per color component. 'A' indicates alpha value.
         *  0b001010..RGB565 - RGB format with 5-bits of R, B; 6-bits of G (packed into 16-bits WORD). No waste bits.
         *  0b001011..RAW8 - 8-bit RAW data packed into 32-bit DWORD
         *  0b001100..RAW10 - 10-bit RAW data packed into 16-bit WORD with 6 LSBs waste bits
         *  0b001101..RAW10P - 10-bit RAW data packed into 32-bit DWORD
         *  0b001110..RAW12 - 12-bit RAW data packed into 16-bit DWORD with 4 LSBs waste bits
         *  0b001111..RAW16 - 16-bit RAW data packed into 32-bit DWORD
         *  0b010000..YUV444_1P8P with 8-bits per color component; 1-plane, YUV interleaved packed bytes
         *  0b010001..YUV444_2P8P with 8-bits per color component; 2-plane, UV interleaved packed bytes
         *  0b010010..YUV444_3P8P with 8-bits per color component; 3-plane, non-interleaved packed bytes
         *  0b010011..YUV444_1P8 with 8-bits per color component; 1-plane YUV interleaved unpacked bytes (8 MSBs waste bits in 32-bit DWORD)
         *  0b010100..YUV444_1P10 with 10-bits per color component; 1-plane, YUV interleaved unpacked bytes (6 LSBs waste bits in 16-bit WORD)
         *  0b010101..YUV444_2P10 with 10-bits per color component; 2-plane, UV interleaved unpacked bytes (6 LSBs waste bits in 16-bit WORD)
         *  0b010110..YUV444_3P10 with 10-bits per color component; 3-plane, non-interleaved unpacked bytes (6 LSBs waste bits in 16-bit WORD)
         *  0b010111..Reserved for future use
         *  0b011000..YUV444_1P10P with 10-bits per color component; 1-plane, YUV interleaved packed bytes (2 MSBs waste bits in 32-bit DWORD)
         *  0b011001..YUV444_2P10P with 10-bits per color component; 2-plane, UV interleaved packed bytes (2 MSBs waste bits in 32-bit DWORD)
         *  0b011010..YUV444_3P10P with 10-bits per color component; 3-plane, non-interleaved packed bytes (2 MSBs waste bits in 32-bit DWORD)
         *  0b011011..Reserved for future use
         *  0b011100..YUV444_1P12 with 12-bits per color component; 1-plane, YUV interleaved unpacked bytes (4 LSBs waste bits in 16-bit WORD)
         *  0b011101..YUV444_2P12 with 12-bits per color component; 2-plane, UV interleaved unpacked bytes (4 LSBs waste bits in 16-bit WORD)
         *  0b011110..YUV444_3P12 with 12-bits per color component; 3-plane, non-interleaved unpacked bytes (4 LSBs waste bits in 16-bit WORD)
         *  0b011111..Reserved for future use
         *  0b100000..YUV422_1P8P with 8-bits per color component; 1-plane, YUV interleaved packed bytes
         *  0b100001..YUV422_2P8P with 8-bits per color component; 2-plane, UV interleaved packed bytes
         *  0b100010..YUV422_3P8P with 8-bits per color component; 3-plane, non-interleaved packed bytes
         *  0b100011..Reserved for future use
         *  0b100100..YUV422_1P10 with 10-bits per color component; 1-plane, YUV interleaved unpacked bytes (6 LSBs waste bits in 16-bit WORD)
         *  0b100101..YUV422_2P10 with 10-bits per color component; 2-plane, UV interleaved unpacked bytes (6 LSBs waste bits in 16-bit WORD)
         *  0b100110..YUV422_3P10 with 10-bits per color component; 3-plane, non-interleaved unpacked bytes (6 LSBs waste bits in 16-bit WORD)
         *  0b100111..Reserved for future use
         *  0b101000..YUV422_1P10P with 10-bits per color component; 1-plane, YUV interleaved packed bytes (2 MSBs waste bits in 32-bit DWORD)
         *  0b101001..YUV422_2P10P with 10-bits per color component; 2-plane, UV interleaved packed bytes (2 MSBs waste bits in 32-bit DWORD)
         *  0b101010..YUV422_3P10P with 10-bits per color component; 3-plane, non-interleaved packed bytes (2 MSBs waste bits in 32-bit DWORD)
         *  0b101011..Reserved for future use
         *  0b101100..YUV422_1P12 with 12-bits per color component; 1-plane, YUV interleaved unpacked bytes (4 LSBs waste bits in 16-bit WORD)
         *  0b101101..YUV422_2P12 with 12-bits per color component; 2-plane, UV interleaved unpacked bytes (4 LSBs waste bits in 16-bit WORD)
         *  0b101110..YUV422_3P12 with 12-bits per color component; 3-plane, non-interleaved unpacked bytes (4 LSBs waste bits in 16-bit WORD)
         *  0b101111..Reserved for future use
         *  0b110000..Reserved for future use
         *  0b110001..YUV420_2P8P with 8-bits per color component; 2-plane, UV interleaved packed bytes
         *  0b110010..YUV420_3P8P with 8-bits per color component; 3-plane, non-interleaved packed bytes
         *  0b110011..Reserved for future use
         *  0b110100..Reserved for future use
         *  0b110101..YUV420_2P10 with 10-bits per color component; 2-plane, UV interleaved unpacked bytes (6 LSBs waste bits in 16-bit WORD)
         *  0b110110..YUV420_3P10 with 10-bits per color component; 3-plane, non-interleaved unpacked bytes (6 LSBs waste bits in 16-bit WORD)
         *  0b110111..Reserved for future use
         *  0b111000..Reserved for future use
         *  0b111001..YUV420_2P10P with 10-bits per color component; 2-plane, UV interleaved packed bytes (2 MSBs waste bits in 32-bit DWORD)
         *  0b111010..YUV420_3P10P with 10-bits per color component; 3-plane, non-interleaved packed bytes (2 MSBs waste bits in 32-bit DWORD)
         *  0b111011..Reserved for future use
         *  0b111100..Reserved for future use
         *  0b111101..YUV420_2P12 with 12-bits per color component; 2-plane, UV interleaved unpacked bytes (4 LSBs waste bits in 16-bit WORD)
         *  0b111110..YUV420_3P12 with 12-bits per color component; 3-plane, non-interleaved unpacked bytes (4 LSBs waste bits in 16-bit WORD)
         *  0b111111..Reserved for future use
         */
    {
        return (((UINT32)(DATA_FORMAT << IMG_CTRL_FORMAT_SHIFT)) & IMG_CTRL_FORMAT_MASK);
    }
    /*! @} */

    /*! @name IMG_CFG - Channel Image Configuration */
    /*! @{ */

    const static UINT32 IMG_CFG_WIDTH_SHIFT = 0U;
    const static UINT32 IMG_CFG_WIDTH_MASK = 0x1FFFU;
    const static UINT32 IMG_CFG_WIDTH(UINT32 WIDTH)
    {
        return (WIDTH << IMG_CFG_WIDTH_SHIFT) & IMG_CFG_WIDTH_MASK;
    }
    const static UINT32 IMG_CFG_HEIGHT_SHIFT = 16U;
    const static UINT32 IMG_CFG_HEIGHT_MASK = 0x1FFF0000U;
    const static UINT32 IMG_CFG_HEIGHT(UINT32 HEIGHT)
    {
        return (HEIGHT << IMG_CFG_HEIGHT_SHIFT) & IMG_CFG_HEIGHT_MASK;
    }
    /*! @} */

    /*! @name SCALE_FACTOR - Channel Scale Factor Register */
    /*! @{ */
    const static UINT32 SCALE_FACTOR_X_SCALE_MASK = 0x3FFFU;
    const static UINT32 SCALE_FACTOR_X_SCALE_SHIFT = 0U;
    const static UINT32 SCALE_FACTOR_X_SCALE(UINT32 SCALE)
    {
        return (SCALE << SCALE_FACTOR_X_SCALE_SHIFT) & SCALE_FACTOR_X_SCALE_MASK;
    }

    const static UINT32 SCALE_FACTOR_Y_SCALE_MASK = 0x3FFF0000U;
    const static UINT32 SCALE_FACTOR_Y_SCALE_SHIFT = 16U;
    const static UINT32 SCALE_FACTOR_Y_SCALE(UINT32 SCALE)
    {
        return (SCALE << SCALE_FACTOR_Y_SCALE_SHIFT) & SCALE_FACTOR_Y_SCALE_MASK;
    }
    /*! @} */

    /*! @name SCALE_OFFSET - Channel Scale Offset Register */
    /*! @{ */
    const static UINT32 SCALE_OFFSET_X_SCALE_MASK = 0xFFFU;
    const static UINT32 SCALE_OFFSET_X_SCALE_SHIFT = 0U;
    const static UINT32 SCALE_OFFSET_X_SCALE(UINT32 OFFSET)
    {
        return (OFFSET << SCALE_OFFSET_X_SCALE_SHIFT) & SCALE_OFFSET_X_SCALE_MASK;
    }

    const static UINT32 SCALE_OFFSET_Y_SCALE_MASK = 0x3FFF0000U;
    const static UINT32 SCALE_OFFSET_Y_SCALE_SHIFT = 16U;
    const static UINT32 SCALE_OFFSET_Y_SCALE(UINT32 OFFSET)
    {
        return (OFFSET << SCALE_OFFSET_Y_SCALE_SHIFT) & SCALE_OFFSET_Y_SCALE_MASK;
    }
    /*! @} */

    /*! @name CROP - Channel Crop Corner Coordinate Registers  */
    /*! @{ */
    const static UINT32 CROP_Y_MASK = 0xFFFU;
    const static UINT32 CROP_Y_SHIFT = 0U;
    const static UINT32 CROP_Y(UINT32 COORDINATE)
    {
        return (COORDINATE << CROP_Y_SHIFT) & CROP_Y_MASK;
    }

    const static UINT32 CROP_X_MASK = 0x3FFF0000U;
    const static UINT32 CROP_X_SHIFT = 16U;
    const static UINT32 CROP_X(UINT32 OFFSET)
    {
        return (OFFSET << CROP_X_SHIFT) & CROP_X_MASK;
    }
    /*! @} */

    /*! @name CSC_COEFF - Channel Color Space Conversion Coefficient Registers */
    /*! @{ */
    const static UINT32 CSC_COEFF_LOW_MASK = 0x7FFU;
    const static UINT32 CSC_COEFF_LOW_SHIFT = 0U;
    const static UINT32 CSC_COEFF_LOW(UINT32 COEFF)
    {
        return (COEFF << CSC_COEFF_LOW_SHIFT) & CSC_COEFF_LOW_MASK;
    }

    const static UINT32 CSC_COEFF_HIGH_MASK = 0x7FF0000U;
    const static UINT32 CSC_COEFF_HIGH_SHIFT = 16U;
    const static UINT32 CSC_COEFF_HIGH(UINT32 COEFF)
    {
        return (COEFF << CSC_COEFF_HIGH_SHIFT) & CSC_COEFF_HIGH_MASK;
    }
    /*! @} */

    /*! @name ROI - Channel Coordinate Registers for Region of Interest */
    /*! @{ */
    const static UINT32 ROI_Y_MASK = 0xFFFU;
    const static UINT32 ROI_Y_SHIFT = 0U;
    const static UINT32 ROI_Y(UINT32 COORDINATE)
    {
        return (COORDINATE << ROI_Y_SHIFT) & ROI_Y_MASK;
    }

    const static UINT32 ROI_X_MASK = 0xFFF0000U;
    const static UINT32 ROI_X_SHIFT = 16U;
    const static UINT32 ROI_X(UINT32 OFFSET)
    {
        return (OFFSET << ROI_X_SHIFT) & ROI_X_MASK;
    }
    /*! @} */

    /*! @name ROI_ALPHA - Channel Alpha Value Registers for Region of Interest */
    /*! @{ */
    const static UINT32 ROI_ALPHA_EN_BIT = 0x10000U;
    const static UINT32 ROI_ALPHA_VALUE_MASK = 0xFF000000U;
    const static UINT32 ROI_ALPHA_VALUE_SHIFT = 24U;
    const static UINT32 ROI_ALPHA_VALUE(UINT32 VALUE)
    {
        return (VALUE << ROI_ALPHA_VALUE_SHIFT) & ROI_ALPHA_VALUE_MASK;
    }
    /*! @} */

    /*! @name IN_BUF_PITCH - Channel Input Buffer Pitch */
    /*! @{ */
    const static UINT32 IN_BUF_PITCH_LINE_PITCH_MASK = 0xFFFFU;
    const static UINT32 IN_BUF_PITCH_LINE_PITCH_SHIFT = 0U;
    const static UINT32 IN_BUF_PITCH_LINE_PITCH(UINT32 PITCH)
    {
        return (PITCH << IN_BUF_PITCH_LINE_PITCH_SHIFT) & IN_BUF_PITCH_LINE_PITCH_MASK;
    }

    const static UINT32 IN_BUF_PITCH_FRM_PITCH_MASK = 0xFFFF0000U;
    const static UINT32 IN_BUF_PITCH_FRM_PITCH_SHIFT = 16U;
    const static UINT32 IN_BUF_PITCH_FRM_PITCH(UINT32 PITCH)
    {
        return (PITCH << IN_BUF_PITCH_LINE_PITCH_SHIFT) & IN_BUF_PITCH_LINE_PITCH_MASK;
    }
    /*! @} */

    /*! @name CHNL_MEM_RD_CTRL - Channel Memory Read Control */
    /*! @{ */
    const static UINT32 MEM_RD_CTRL_READ_MEM_BIT = 0x1U;
    const static UINT32 MEM_RD_CTRL_IMG_TYPE_MASK = 0xF0000000U;
    const static UINT32 MEM_RD_CTRL_IMG_TYPE_SHIFT = 28U;
    const static UINT32 MEM_RD_CTRL_IMG_TYPE_RGB8P = (0x1U << MEM_RD_CTRL_IMG_TYPE_SHIFT); // RGB format with 8-bits per color component (packed into 32-bit DWORD)
    const static UINT32 MEM_RD_CTRL_IMG_TYPE_BGR8P = (0x0U << MEM_RD_CTRL_IMG_TYPE_SHIFT); // BGR format with 8-bits per color component (packed into 32-bit DWORD)
    const static UINT32 MEM_RD_CTRL_IMG_TYPE_RGB565 = (0x5U << MEM_RD_CTRL_IMG_TYPE_SHIFT); // RGB format with 5-bits of R, B; 6-bits of G (packed into 32-bit DWORD)
    const static UINT32 MEM_RD_CTRL_IMG_TYPE_YUV422_8B_1P = (0xDU << MEM_RD_CTRL_IMG_TYPE_SHIFT); // 8-bits per color component; 1-plane YUV interleaved packed bytes
    const static UINT32 MEM_RD_CTRL_IMG_TYPE(UINT32 VALUE)
    /*! IMG_TYPE - Input image format
     *  0b0000..BGR8P - BGR format with 8-bits per color component (packed into 32-bit DWORD)
     *  0b0001..RGB8P - RGB format with 8-bits per color component (packed into 32-bit DWORD)
     *  0b0010..XRGB8 - RGB format with 8-bits per color component (unpacked and LSB aligned in 32-bit DWORD)
     *  0b0011..RGBX8 - RGB format with 8-bits per color component (unpacked and MSBalinged in 32-bit DWORD)
     *  0b0100..XBGR8 - BGR format with 8-bits per color component (unpacked and LSB aligned in 32-bit DWORD)
     *  0b0101..RGB565 - RGB format with 5-bits of R, B; 6-bits of G (packed into 32-bit DWORD)
     *  0b0110..A2BGR10 - BGR format with 2-bits alpha in MSB; 10-bits per color component
     *  0b0111..A2RGB10 - RGB format with 2-bits alpha in MSB; 10-bits per color component
     *  0b1000..YUV444_1P8P with 8-bits per color component; 1-plane, YUV interleaved packed bytes
     *  0b1001..YUV444_1P10 with 10-bits per color component; 1-plane, YUV interleaved unpacked bytes (6 LSBs waste bits in 16-bit WORD)
     *  0b1010..YUV444_1P10P with 10-bits per color component; 1-plane, YUV interleaved packed bytes (2 MSBs waste bits in 32-bit WORD)
     *  0b1011..YUV444_1P12 with 12-bits per color component; 1-plane, YUV interleaved unpacked bytes (4 LSBs waste bits in 16-bit WORD)
     *  0b1100..YUV444_1P8 with 8-bits per color component; 1-plane YUV interleaved unpacked bytes (8 MSBs waste bits in 32-bit DWORD)
     *  0b1101..YUV422_1P8P with 8-bits per color component; 1-plane YUV interleaved packed bytes
     *  0b1110..YUV422_1P10 with 10-bits per color component; 1-plane, YUV interleaved unpacked bytes (6 LSBs waste bits in 16-bit WORD)
     *  0b1111..YUV422_1P12 with 12-bits per color component; 1-plane, YUV interleaved packed bytes (4 MSBs waste bits in 16-bit WORD)
     */
    {
        return (VALUE << ROI_ALPHA_VALUE_SHIFT) & ROI_ALPHA_VALUE_MASK;
    }
    /*! @} */

    /*! @name SCL_IMG_CFG - Channel Scaled Image Configuration */
    /*! @{ */
    const static UINT32 SCL_IMG_CFG_WIDTH_MASK = 0x1FFFU;
    const static UINT32 SCL_IMG_CFG_WIDTH_SHIFT = 0U;
    const static UINT32 SCL_IMG_CFG_WIDTH(UINT32 WIDTH)
    {
        return (WIDTH << SCL_IMG_CFG_WIDTH_SHIFT) & SCL_IMG_CFG_WIDTH_MASK;
    }

    const static UINT32 SCL_IMG_CFG_HEIGHT_MASK = 0x1FFF0000U;
    const static UINT32 SCL_IMG_CFG_HEIGHT_SHIFT = 16U;
    const static UINT32 SCL_IMG_CFG_HEIGHT(UINT32 HEIGHT)
    {
        return (HEIGHT << SCL_IMG_CFG_HEIGHT_SHIFT) & SCL_IMG_CFG_HEIGHT_MASK;
    }
    /*! @} */

    /*! @name FLOW_CTRL - Channel Scaled Image Configuration */
    /*! @{ */
    const static UINT32 FLOW_CTRL_FC_DENOM_MASK = 0xFFU;
    const static UINT32 FLOW_CTRL_FC_DENOM_SHIFT = 0U;
    const static UINT32 FLOW_CTRL_FC_DENOM(UINT32 FC_DENOM)
    /*! FC_DENOM - Denominator value of fraction of usable bandwidth
     *  0b00000000..Invalid value. Flow control will be disabled.
     */
    {
        return (FC_DENOM << FLOW_CTRL_FC_DENOM_SHIFT) & FLOW_CTRL_FC_DENOM_MASK;
    }

    const static UINT32 FLOW_CTRL_FC_NUM_MASK = 0xFF0000U;
    const static UINT32 FLOW_CTRL_FC_NUM_SHIFT = 16U;
    const static UINT32 FLOW_CTRL_FC_NUM(UINT32 FC_NUM)
    /*! FC_NUMER - Numerator value of fraction of usable bandwidth
     *  0b00000000..Flow control is disabled.
     */
    {
        return (FC_NUM << FLOW_CTRL_FC_NUM_SHIFT) & FLOW_CTRL_FC_NUM_MASK;
    }
}

#if 1
namespace ISI_REG_BITFIELDS_V2
{
    /*! @name OUT_BUF_CTRL - Channel Output Buffer Control Register */
    /*! @{ */

    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_Y_SHIFT = 0U;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_Y_MASK = 0x3U;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_Y(UINT32 PANIC_THD)
        /*! PANIC_SET_THD_Y - Overflow panic set threshold value for Y/RGB output buffer
         *  0b0000..No panic alert will be asserted
         *  0b0001-0b1111..Panic will assert when buffer is n * 6.25% full, where n = 1 to 15
         */
    {
        return (PANIC_THD << OUT_BUF_CTRL_PANIC_SET_THD_Y_SHIFT) & OUT_BUF_CTRL_PANIC_SET_THD_Y_MASK;
    }
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_U_SHIFT = 3U;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_U_MASK = 0x18U;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_U(UINT32 PANIC_THD)
        /*! PANIC_SET_THD_U - Overflow panic set threshold value for U output buffer
         *  0b0000..No panic alert will be asserted
         *  0b0001-0b1111..Panic will assert when buffer is n * 6.25% full, where n = 1 to 15
         */
    {
        return (PANIC_THD << OUT_BUF_CTRL_PANIC_SET_THD_U_SHIFT) & OUT_BUF_CTRL_PANIC_SET_THD_U_MASK;
    }

    const static UINT32 OUT_BUF_CTRL_LOAD_BUF1_ADDR_BIT = 0x4000U;
    const static UINT32 OUT_BUF_CTRL_LOAD_BUF2_ADDR_BIT = 0x8000U;

    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_V_SHIFT = 6U;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_V_MASK = 0xC0;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_V(UINT32 PANIC_THD)
        /*! PANIC_SET_THD_V - Overflow panic set threshold value for V output buffer
         *  0b0000..No panic alert will be asserted
         *  0b0001-0b1111..Panic will assert when buffer is n * 6.25% full, where n = 1 to 15
         */
    {
        return (PANIC_THD << OUT_BUF_CTRL_PANIC_SET_THD_V_SHIFT) & OUT_BUF_CTRL_PANIC_SET_THD_V_MASK;
    }
    const static UINT32 THD_NO_PANIC = 0U;
    const static UINT32 THD_PANIC_25 = 1U;
    const static UINT32 THD_PANIC_50 = 2U;
    const static UINT32 THD_PANIC_75 = 3U;

    const static UINT32 OUT_BUF_CTRL_MAX_WR_BEATS_UV_BIT = 0x40000000U;
    const static UINT32 OUT_BUF_CTRL_MAX_WR_BEATS_Y_BIT = 0x80000000U;

    /*! @name IER - Channel Interrupt Enable Register */
    /*! @{ */
    const static UINT32 IER_ALL_MASK = 0xFFFFFFFF;
    const static UINT32 IER_LATE_VSYNC_ERR_EN_BIT = 0x4000U;
    const static UINT32 IER_EARLY_VSYNC_ERR_EN_BIT = 0x8000U;
    const static UINT32 IER_OFLW_Y_BUF_EN_BIT = 0x10000U;
    const static UINT32 IER_EXCS_OFLW_Y_BUF_EN_BIT = 0x20000U;
    const static UINT32 IER_PANIC_Y_BUF_EN_BIT = 0x40000U;
    const static UINT32 IER_OFLW_U_BUF_EN_BIT = 0x80000U;
    const static UINT32 IER_EXCS_OFLW_U_BUF_EN_BIT = 0x100000U;
    const static UINT32 IER_PANIC_U_BUF_EN_BIT = 0x200000U;
    const static UINT32 IER_OFLW_V_BUF_EN_BIT = 0x400000U;
    const static UINT32 IER_EXCS_OFLW_V_BUF_EN_BIT = 0x800000U;
    const static UINT32 IER_PANIC_V_BUF_EN_BIT = 0x1000000U;
    const static UINT32 IER_AXI_RD_ERR_EN_BIT = 0x2000000U;
    const static UINT32 IER_AXI_WR_ERR_Y_EN_BIT = 0x4000000U;
    const static UINT32 IER_AXI_WR_ERR_U_EN_BIT = 0x8000000U;
    const static UINT32 IER_AXI_WR_ERR_V_EN_BIT = 0x10000000U;
    const static UINT32 IER_FRM_RCVD_EN_BIT = 0x20000000U;
    const static UINT32 IER_LINE_RCVD_EN_BIT = 0x40000000U;
    const static UINT32 IER_MEM_RD_DONE_EN_BIT = 0x80000000U;
    /*! @} */

    /*! @name STS - Channel Status Register */
    /*! @{ */
    const static UINT32 STS_BUF1_ACTIVE_BIT = 0x100U;
    const static UINT32 STS_BUF2_ACTIVE_BIT = 0x200U;
    const static UINT32 STS_BUF2_ACTIVE_GET(UINT32 STS)
    {
        return ((STS & STS_BUF2_ACTIVE_BIT) > 0);
    }
    const static UINT32 STS_MEM_RD_OFLOW_BIT = 0x400U;

    const static UINT32 STS_LATE_VSYNC_ERR_BIT = 0x4000U;
    const static UINT32 STS_EARLY_VSYNC_ERR_BIT = 0x8000U;
    const static UINT32 STS_OFLW_Y_BUF_BIT = 0x10000U;
    const static UINT32 STS_EXCS_OFLW_Y_BUF_BIT = 0x20000U;
    const static UINT32 STS_PANIC_Y_BUF_BIT = 0x40000U;
    const static UINT32 STS_OFLW_U_BUF_BIT = 0x80000U;
    const static UINT32 STS_EXCS_OFLW_U_BUF_BIT = 0x100000U;
    const static UINT32 STS_PANIC_U_BUF_BIT = 0x200000U;
    const static UINT32 STS_OFLW_V_BUF_BIT = 0x400000U;
    const static UINT32 STS_EXCS_OFLW_V_BUF_BIT = 0x800000U;
    const static UINT32 STS_PANIC_V_BUF_BIT = 0x1000000U;
    const static UINT32 STS_AXI_RD_ERR_BIT = 0x2000000U;
    const static UINT32 STS_AXI_WR_ERR_Y_BIT = 0x4000000U;
    const static UINT32 STS_AXI_WR_ERR_U_BIT = 0x8000000U;
    const static UINT32 STS_AXI_WR_ERR_V_BIT = 0x10000000U;
    const static UINT32 STS_FRM_STRD_BIT = 0x20000000U;
    const static UINT32 STS_LINE_STRD_BIT = 0x40000000U;
    const static UINT32 STS_MEM_RD_DONE_BIT = 0x80000000U;
    /*! @} */

    /*! @} */
}
#endif
namespace ISI_REG_BITFIELDS_V3
{
    /*! @name OUT_BUF_CTRL - Channel Output Buffer Control Register */
    /*! @{ */

    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_Y_SHIFT = 0U;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_Y_MASK = 0xFU;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_Y(UINT32 PANIC_THD)
    /*! PANIC_SET_THD_Y - Overflow panic set threshold value for Y/RGB output buffer
     *  0b0000..No panic alert will be asserted
     *  0b0001-0b1111..Panic will assert when buffer is n * 6.25% full, where n = 1 to 15
     */
    {
        return (PANIC_THD << OUT_BUF_CTRL_PANIC_SET_THD_Y_SHIFT) & OUT_BUF_CTRL_PANIC_SET_THD_Y_MASK;
    }
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_U_SHIFT = 8U;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_U_MASK = 0xF00U;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_U(UINT32 PANIC_THD)
    /*! PANIC_SET_THD_U - Overflow panic set threshold value for U output buffer
     *  0b0000..No panic alert will be asserted
     *  0b0001-0b1111..Panic will assert when buffer is n * 6.25% full, where n = 1 to 15
     */
    {
        return (PANIC_THD << OUT_BUF_CTRL_PANIC_SET_THD_U_SHIFT) & OUT_BUF_CTRL_PANIC_SET_THD_U_MASK;
    }

    const static UINT32 OUT_BUF_CTRL_LOAD_BUF1_ADDR_BIT = 0x4000U;
    const static UINT32 OUT_BUF_CTRL_LOAD_BUF2_ADDR_BIT = 0x8000U;

    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_V_SHIFT = 16U;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_V_MASK = 0xF0000U;
    const static UINT32 OUT_BUF_CTRL_PANIC_SET_THD_V(UINT32 PANIC_THD)
    /*! PANIC_SET_THD_V - Overflow panic set threshold value for V output buffer
     *  0b0000..No panic alert will be asserted
     *  0b0001-0b1111..Panic will assert when buffer is n * 6.25% full, where n = 1 to 15
     */
    {
        return (PANIC_THD << OUT_BUF_CTRL_PANIC_SET_THD_V_SHIFT) & OUT_BUF_CTRL_PANIC_SET_THD_V_MASK;
    }
    const static UINT32 THD_NO_PANIC = 0U;
    const static UINT32 THD_PANIC_25 = 1U;
    const static UINT32 THD_PANIC_50 = 2U;
    const static UINT32 THD_PANIC_75 = 3U;

    const static UINT32 OUT_BUF_CTRL_MAX_WR_BEATS_UV_BIT = 0x40000000U;
    const static UINT32 OUT_BUF_CTRL_MAX_WR_BEATS_Y_BIT = 0x80000000U;

    /*! @} */

    /*! @name IER - Channel Interrupt Enable Register */
    /*! @{ */
    const static UINT32 IER_LATE_VSYNC_ERR_EN_BIT = 0x10000U;
    const static UINT32 IER_EARLY_VSYNC_ERR_EN_BIT = 0x20000U;
    // FIXME The V3 is a shifted by one bit. See comments on the right.
    const static UINT32 IER_OFLW_Y_BUF_EN_BIT = 0x40000U; // 0x00080000
    const static UINT32 IER_PANIC_Y_BUF_EN_BIT = 0x80000U; // 0x100000U
    const static UINT32 IER_OFLW_U_BUF_EN_BIT = 0x100000U; // 0x100000U
    const static UINT32 IER_PANIC_U_BUF_EN_BIT = 0x200000U; // 0x00400000
    const static UINT32 IER_OFLW_V_BUF_EN_BIT = 0x400000U; // 0x00800000
    const static UINT32 IER_PANIC_V_BUF_EN_BIT = 0x800000U; // 0x01000000
    const static UINT32 IER_AXI_RD_ERR_EN_BIT = 0x2000000U;
    const static UINT32 IER_AXI_WR_ERR_Y_EN_BIT = 0x4000000U;
    const static UINT32 IER_AXI_WR_ERR_U_EN_BIT = 0x8000000U;
    const static UINT32 IER_AXI_WR_ERR_V_EN_BIT = 0x10000000U;
    const static UINT32 IER_FRM_RCVD_EN_BIT = 0x20000000U;
    const static UINT32 IER_LINE_RCVD_EN_BIT = 0x40000000U;
    const static UINT32 IER_MEM_RD_DONE_EN_BIT = 0x80000000U;

    const static UINT32 IER_ALL_MASK = 0xFFFFFFFF;
    const static UINT32 IER_OFLW_BUF_MASK = IER_OFLW_Y_BUF_EN_BIT | IER_OFLW_U_BUF_EN_BIT | IER_OFLW_V_BUF_EN_BIT;
    const static UINT32 IER_AXI_WR_ERR_MASK = IER_AXI_WR_ERR_Y_EN_BIT | IER_AXI_WR_ERR_U_EN_BIT | IER_AXI_WR_ERR_V_EN_BIT;
    /*! @} */

    /*! @name STS - Channel Status Register */
    /*! @{ */
    const static UINT32 STS_BUF1_ACTIVE_BIT = 0x100U;
    const static UINT32 STS_BUF2_ACTIVE_BIT = 0x200U;
    const static UINT32 STS_BUF2_ACTIVE_GET(UINT32 STS)
    {
        return ((STS & STS_BUF2_ACTIVE_BIT) > 0);
    }
    const static UINT32 STS_MEM_RD_OFLOW_BIT = 0x400U;

    const static UINT32 STS_LATE_VSYNC_ERR_BIT = 0x10000U;
    const static UINT32 STS_EARLY_VSYNC_ERR_BIT = 0x20000U;
    const static UINT32 STS_OFLW_Y_BUF_BIT = 0x40000U;
    const static UINT32 STS_PANIC_Y_BUF_BIT = 0x80000U;
    const static UINT32 STS_OFLW_U_BUF_BIT = 0x100000U;
    const static UINT32 STS_PANIC_U_BUF_BIT = 0x200000U;
    const static UINT32 STS_OFLW_V_BUF_BIT = 0x400000U;
    const static UINT32 STS_PANIC_V_BUF_BIT = 0x800000U;
    const static UINT32 STS_AXI_RD_ERR_BIT = 0x2000000U;
    const static UINT32 STS_AXI_WR_ERR_Y_BIT = 0x4000000U;
    const static UINT32 STS_AXI_WR_ERR_U_BIT = 0x8000000U;
    const static UINT32 STS_AXI_WR_ERR_V_BIT = 0x10000000U;
    const static UINT32 STS_FRM_STRD_BIT = 0x20000000U;
    const static UINT32 STS_LINE_STRD_BIT = 0x40000000U;
    const static UINT32 STS_MEM_RD_DONE_BIT = 0x80000000U; 
    /*! @} */
}
/*! @} */

#pragma warning( pop ) // disable: 4505


struct Resources_t {
    ISI_REGS *m_IsiRegistersPtr;
    UINT32 coreClockFrequencyHz;
    PCHAR  csiDevNameA;
};
