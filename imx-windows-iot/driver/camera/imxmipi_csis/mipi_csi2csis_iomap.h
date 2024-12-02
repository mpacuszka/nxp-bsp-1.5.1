/*
 * Copyright 2017, 2019-2020,2022 NXP
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

#include <ntddk.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

struct Resources_t;

typedef NTSTATUS ACPI_READ_UINT32(UINT32 &Val);
typedef NTSTATUS ACPI_READ_UINT32_FROM_OFFSET(UINT32 Offset, UINT32 &Val);
typedef NTSTATUS ACPI_WRITE_UINT32(UINT32 Val);
typedef NTSTATUS ACPI_WRITE_UINT32_TO_OFFSET(UINT32 Offset, UINT32 Val);

class MipiCsi2_t {
public: // Type definitions

#pragma pack( push )
#pragma pack( 4 )
    struct MIPI_CSI2CSIS_REGS
    /** MIPI_CSI2CSIS - Register Layout */
    {
        /* ____ */ UINT8 RESERVED_0[4];
        /* __IO */ UINT32 CSIS_COMMON_CTRL;                 /**< CSIS Common Control Register, offset: 0x4 */
        /* __IO */ UINT32 CSIS_CLOCK_CTRL;                  /**< CSIS Clock Control Register, offset: 0x8 */
        /* ____ */ UINT8 RESERVED_1[4];
        /* __I  */ UINT32 INTERRUPT_MASK_0;                 /**< Interrupt mask register 0, offset: 0x10 */
        /* __I  */ UINT32 INTERRUPT_SOURCE_0;               /**< Interrupt source register 0, offset: 0x14 */
        /* __IO */ UINT32 INTERRUPT_MASK_1;                 /**< Interrupt mask register 1, offset: 0x18 */
        /* __I  */ UINT32 INTERRUPT_SOURCE_1;               /**< Interrupt source register 1, offset: 0x1C */
        /* __I  */ UINT32 DPHY_STATUS;                      /**< D-PHY status register, offset: 0x20 */
        /* __I  */ UINT32 DPHY_COMMON_CTRL;                 /**< D-PHY common control register, offset: 0x24 */
        /* __I  */ UINT8 RESERVED_2[8];
        /* __I  */ UINT32 DPHY_MASTER_SLAVE_CTRL_LOW;       /**< D-PHY Master and Slave Control register Low, offset: 0x30 */
        /* __I  */ UINT32 DPHY_MASTER_SLAVE_CTRL_HIGH;      /**< D-PHY Master and Slave Control register HIGH, offset: 0x34 */
        /* __IO */ UINT32 DPHY_SLAVE_CTRL_LOW;              /**< D-PHY Slave Control register Low, offset: 0x38 */
        /* __IO */ UINT32 DPHY_SLAVE_CTRL_HIGH;             /**< D-PHY Slave Control register High, offset: 0x3C */
                   struct {                                 /* offset: 0x40, array step: 0x10 */
                     UINT32 ISP_CONFIG;                     /**< ISP Configuration Register, array offset: 0x40, array step: 0x10 */
                     UINT32 ISP_RESOLUTION;                 /**< ISP Resolution Register, array offset: 0x44, array step: 0x10 */
                     UINT32 ISP_SYNC;                       /**< ISP SYNC Register, array offset: 0x48, array step: 0x10 */
                     UINT8 RESERVED_0[4];
                   } ISP_CONFIGn[4];
                   struct {                                 /* offset: 0x80, array step: 0x10 */
                     UINT32 SHADOW_CONFIG;                  /**< Shadow Configuration Register, array offset: 0x80, array step: 0x10 */
                     UINT32 SHADOW_RESOLUTION;              /**< Shadow Resolution Register, array offset: 0x84, array step: 0x10 */
                     UINT32 SHADOW_SYNC;                    /**< Shadow SYNC Register, array offset: 0x88, array step: 0x10 */
                     UINT8 RESERVED_0[4];
                   } SHADOW_CONFIGn[4];
        /* __IO */ UINT8 RESERVED_3[64];
        /* __IO */ UINT32 FRAME_COUNTER[4];                 /**< Frame Counter, array offset: 0x100, array step: 0x4 */
        /* __IO */ UINT32 LINE_INTERRUPT_RATIO[4];          /**< Line Interrupt Ratio, array offset: 0x110, array step: 0x4 */
    };
    /*! @name CSIS_COMMON_CTRL - CSIS Common Control Register */
    /*! @{ */
    const static UINT32 CSIS_COMMON_CTRL_CSI_EN_BIT = 0x1U;
    const static UINT32 CSIS_COMMON_CTRL_SW_RESET_BIT = 0x2U;
    const static UINT32 CSIS_COMMON_CTRL_UPDATE_SHADOW_CTRL_BIT = 0x4U;
    const static UINT32 CSIS_COMMON_CTRL_LANE_NUMBER_SHIFT = 8;
    const static UINT32 CSIS_COMMON_CTRL_LANE_NUMBER_MASK = 0x300;
    const static UINT32 CSIS_COMMON_CTRL_LANE_NUMBER(UINT32 LANE_NUMBER)
    {
        return (LANE_NUMBER << CSIS_COMMON_CTRL_LANE_NUMBER_SHIFT) & CSIS_COMMON_CTRL_LANE_NUMBER_MASK;
    }
    const static UINT32 CSIS_COMMON_CTRL_UPDATE_SHADOW_SHIFT = 16; // UPDATE_SHADOW - Strobe of updating shadow registers
    const static UINT32 CSIS_COMMON_CTRL_UPDATE_SHADOW_MASK = 0xF0000;
    const static UINT32 CSIS_COMMON_CTRL_UPDATE_SHADOW(UINT32 STROBE)
    {
        return (STROBE << CSIS_COMMON_CTRL_UPDATE_SHADOW_SHIFT) & CSIS_COMMON_CTRL_UPDATE_SHADOW_MASK;
    }
    /*! @} */

    /*! @name CSIS_CLOCK_CTRL - CSIS Clock Control Register */
    /*! @{ */
    const static UINT32 CSIS_CLOCK_CTRL_WCLK_SRC_BIT = 0x1;
    const static UINT32 CSIS_CLOCK_CTRL_CLKGATE_EN_BIT = 0x10; // 0: Pixel clock is always alive, 1: Pixel clock is alive during the interval of frame.
    const static UINT32 CSIS_CLOCK_CTRL_CLKGATE_TRAIL_SHIFT = 16; // 0 ~ 3 (1~4 Trailing clocks)
    const static UINT32 CSIS_CLOCK_CTRL_CLKGATE_TRAIL_MASK = 0xFFFF0000;
    const static UINT32 CSIS_CLOCK_CTRL_CLKGATE_TRAIL_CH0(UINT32 TRAIL_CLOCKS)
    {
        return (TRAIL_CLOCKS << CSIS_CLOCK_CTRL_CLKGATE_TRAIL_SHIFT) & CSIS_CLOCK_CTRL_CLKGATE_TRAIL_MASK;
    }
    const static UINT32 CSIS_CLOCK_CTRL_CLKGATE_TRAIL_CH1(UINT32 TRAIL_CLOCKS)
    {
        return (TRAIL_CLOCKS << 20) & CSIS_CLOCK_CTRL_CLKGATE_TRAIL_MASK;
    }
    const static UINT32 CSIS_CLOCK_CTRL_CLKGATE_TRAIL_CH2(UINT32 TRAIL_CLOCKS)
    {
        return (TRAIL_CLOCKS << 24) & CSIS_CLOCK_CTRL_CLKGATE_TRAIL_MASK;
    }
    const static UINT32 CSIS_CLOCK_CTRL_CLKGATE_TRAIL_CH3(UINT32 TRAIL_CLOCKS)
    {
        return (TRAIL_CLOCKS << 28) & CSIS_CLOCK_CTRL_CLKGATE_TRAIL_MASK;
    }
    /*! @} */

    /*! @name DPHY_COMMON_CTRL - D-PHY common control register */
    /*! @{ */
    const static UINT32 DPHY_COMMON_CTRL_ENABLE_CLK_BIT = 0x1;
    const static UINT32 DPHY_COMMON_CTRL_ENABLE_DAT_SHIFT = 1;
    const static UINT32 DPHY_COMMON_CTRL_ENABLE_DAT_MASK = 0x1E;
    const static UINT32 DPHY_COMMON_CTRL_ENABLE_DAT(UINT32 ENABLE_DAT)
    {
        return (ENABLE_DAT << DPHY_COMMON_CTRL_ENABLE_DAT_SHIFT) & DPHY_COMMON_CTRL_ENABLE_DAT_MASK;
    }
    const static UINT32 DPHY_COMMON_CTRL_S_DPDN_SWAP_DAT_BIT = 0x20; // Field S_DPDN_SWAP_DAT - Swapping Dp and Dn channel of data lanes.
    const static UINT32 DPHY_COMMON_CTRL_S_DPDN_SWAP_CLK_BIT = 0x40;
    const static UINT32 DPHY_COMMON_CTRL_S_CLKSETTLECTL_SHIFT = 22; // Field S_CLKSETTLECTL - D-PHY control register for standard spec v0.9 of MIPI CSI2
    const static UINT32 DPHY_COMMON_CTRL_S_CLKSETTLECTL_MASK = 0xC00000;
    const static UINT32 DPHY_COMMON_CTRL_S_CLKSETTLECTL(UINT32 CLKSETTLECTL)
    {
        return (CLKSETTLECTL << DPHY_COMMON_CTRL_S_CLKSETTLECTL_SHIFT) & DPHY_COMMON_CTRL_S_CLKSETTLECTL_MASK;
    }
    const static UINT32 DPHY_COMMON_CTRL_HSSETTLE_SHIFT = 24; // Field HSSETTLE - HS-RX settle time control register.
    const static UINT32 DPHY_COMMON_CTRL_HSSETTLE_MASK = 0xFF000000;
    const static UINT32 DPHY_COMMON_CTRL_HSSETTLE(UINT32 HSSETTLE)
    {
        return (HSSETTLE << DPHY_COMMON_CTRL_HSSETTLE_SHIFT) & DPHY_COMMON_CTRL_HSSETTLE_MASK;
    }
    /*! @} */

    /*! @name ISP_CONFIG - ISP Configuration Register */
    /*! @{ */
    const static UINT32 ISP_CONFIG_DATAFORMAT_SHIFT = 2;
    const static UINT32 ISP_CONFIG_DATAFORMAT_MASK = 0xFC;
    const static UINT32 ISP_CONFIG_DATAFORMAT_YUV420_8         = (0x18 << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_YUV420_10        = (0x19 << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_LEGACY_YUV420_8  = (0x1A << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_CS_YUV420_8      = (0x1C << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_CS_YUV420_10     = (0x1D << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_YUV422_8         = (0x1E << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_YUV422_10        = (0x1F << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_RGB565           = (0x22 << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_RGB666           = (0x23 << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_RGB888           = (0x24 << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_RAW6             = (0x28 << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_RAW7             = (0x29 << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_RAW8             = (0x2A << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_RAW10            = (0x2B << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_RAW12            = (0x2C << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT_RAW14            = (0x2D << ISP_CONFIG_DATAFORMAT_SHIFT);
    const static UINT32 ISP_CONFIG_DATAFORMAT(UINT32 DATAFORMAT)
    /*! Gasket 0 data type
     *  011000b - YUV420 8-bit
     *    011001b - YUV420 10-bit
     *    011010b - Legacy YUV420 8-bit
     *    011100b - YUV420 8-bit(Chroma Shifted Pixel Sampling)
     *    011101b - YUV420 10-bit(Chroma Shifted Pixel Sampling)
     *    011110b - YUV422 8-bit
     *    011111b - YUV422 10-bit
     *    100010b - RGB565
     *    100011b - RGB666
     *    100100b - RGB888
     *    101000b - RAW6
     *    101001b - RAW7
     *    101010b - RAW8
     *    101011b - RAW10
     *    101100b - RAW12
     *    101101b - RAW14
     */
    {
        return (DATAFORMAT << ISP_CONFIG_DATAFORMAT_SHIFT) & ISP_CONFIG_DATAFORMAT_MASK;
    }

    const static UINT32 ISP_CONFIG_RGB_SWAP_BIT = 0x400;
    const static UINT32 ISP_CONFIG_PARALLEL_BIT = 0x800;
    const static UINT32 ISP_CONFIG_PIXEL_MODE_SHIFT = 12; // Pixel mode should match ISI and CSI input width.
    const static UINT32 ISP_CONFIG_PIXEL_MODE_MASK = 0x3000;
    const static UINT32 ISP_CONFIG_PIXEL_MODE(UINT32 PIXEL_MODE)
    {
        return (PIXEL_MODE << ISP_CONFIG_PIXEL_MODE_SHIFT) & ISP_CONFIG_PIXEL_MODE_MASK;
    }
    const static UINT32 ISP_CONFIG_PIXEL_MODE_QUAD = 0x2000; // RAW8/10/12
    const static UINT32 ISP_CONFIG_PIXEL_MODE_DUAL = 0x1000; // RAW8/10/12, YUV422
    const static UINT32 ISP_CONFIG_PIXEL_MODE_SINGLE = 0x0000;
    /*! @} */

    /*! @name ISP_RESOLUTION - ISP Resolution Register */
    /*! @{ */
    const static UINT32 ISP_RESOLUTION_HRESOL_SHIFT = 0;
    const static UINT32 ISP_RESOLUTION_HRESOL_MASK = 0xFFFF;
    const static UINT32 ISP_RESOLUTION_HRESOL(UINT32 HRESOL)
    {
        return (HRESOL << ISP_RESOLUTION_HRESOL_SHIFT) & ISP_RESOLUTION_HRESOL_MASK;
    }
    const static UINT32 ISP_RESOLUTION_VRESOL_SHIFT = 16;
    const static UINT32 ISP_RESOLUTION_VRESOL_MASK = 0xFFFF0000;
    const static UINT32 ISP_RESOLUTION_VRESOL(UINT32 VRESOL)
    {
        return (VRESOL << ISP_RESOLUTION_VRESOL_SHIFT) & ISP_RESOLUTION_VRESOL_MASK;
    }
    /*! @} */

#pragma pack( pop )

    /*! @name GPR_GASKET_0_CTRL - Gasket 0 Control Register */
    /*! @{ */
    static constexpr UINT32 GPR_GASKET_0_CTRL_REG_OFFSET = 0x60;

    static constexpr UINT32 GPR_GASKET_0_CTRL_ENABLE_BIT = (1u << 0);
    static constexpr UINT32 GPR_GASKET_0_CTRL_DOUBLE_COMPONENT_BIT = (1u << 1);

    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_MASK = 0x00003F00; // FORMAT - Output image format
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT = 0x8U;
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_YUV420_8         = (0x18 << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_YUV420_10        = (0x19 << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_LEGACY_YUV420_8  = (0x1A << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_CS_YUV420_8      = (0x1C << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_CS_YUV420_10     = (0x1D << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_YUV422_8         = (0x1E << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_YUV422_10        = (0x1F << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_RGB565           = (0x22 << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_RGB666           = (0x23 << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_RGB888           = (0x24 << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_RAW6             = (0x28 << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_RAW7             = (0x29 << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_RAW8             = (0x2A << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_RAW10            = (0x2B << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_RAW12            = (0x2C << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE_RAW14            = (0x2D << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT);
    const static UINT32 GPR_GASKET_0_CTRL_DATA_TYPE(UINT32 DATA_TYPE)
    /*! Gasket 0 data type
     *  011000b - YUV420 8-bit
     *    011001b - YUV420 10-bit
     *    011010b - Legacy YUV420 8-bit
     *    011100b - YUV420 8-bit(Chroma Shifted Pixel Sampling)
     *    011101b - YUV420 10-bit(Chroma Shifted Pixel Sampling)
     *    011110b - YUV422 8-bit
     *    011111b - YUV422 10-bit
     *    100010b - RGB565
     *    100011b - RGB666
     *    100100b - RGB888
     *    101000b - RAW6
     *    101001b - RAW7
     *    101010b - RAW8
     *    101011b - RAW10
     *    101100b - RAW12
     *    101101b - RAW14
     */
    {
        return (((UINT32)(DATA_TYPE << GPR_GASKET_0_CTRL_DATA_TYPE_SHIFT)) & GPR_GASKET_0_CTRL_DATA_TYPE_MASK);
    }
    /*! @} */

    /*! @name GPR_GASKET_0_HSIZE - Gasket 0 Video Horizontal Size Register */
    /*! @{ */
    static constexpr UINT32 GPR_GASKET_0_HSIZE_REG_OFFSET = 0x64;
    /*! @} */

    /*! @name GPR_GASKET_0_VSIZE - Gasket 0 Video Vertical Size Register */
    /*! @{ */
    static constexpr UINT32 GPR_GASKET_0_VSIZE_REG_OFFSET = 0x68;
    /*! @} */

    /*! @name GPR_GASKET_0_ISI_PIXEL_CNT - Gasket 0 Video Vertical Size Register */
    /*! @{ */
    static constexpr UINT32 GPR_GASKET_0_ISI_PIXEL_CNT = 0x7C;
    /*! @} */

    /*! @name GPR_GASKET_0_ISI_LINE_CNT - Gasket 0 Video Vertical Size Register */
    /*! @{ */
    static constexpr UINT32 GPR_GASKET_0_ISI_LINE_CNT = 0x80;
    /*! @} */

private:
    volatile MIPI_CSI2CSIS_REGS *m_RegistersPtr;
    Resources_t *m_ResourcePtr;
    UINT32 m_CpuId;
    UINT32 m_PixelMode;

    NTSTATUS PhyAssertReset();
    NTSTATUS PhyDeassertReset();
    NTSTATUS EnablePhy(const camera_config_t &Config);
    NTSTATUS DisablePhy();
    void DisableInterrupts();


    NTSTATUS Gpr8mnGasketConfigure(const camera_config_t &Config);
    NTSTATUS Gpr8mnGasketStart(bool Enable);
    void SwReset();
    NTSTATUS SetFormat(const camera_config_t &Config);

public:
    MipiCsi2_t() :m_ResourcePtr(NULL) {};

    // Public interface -------------------------------------------------
    NTSTATUS Check() {return STATUS_SUCCESS; };

    NTSTATUS Init(const camera_config_t &config);

    NTSTATUS Deinit();

    NTSTATUS MipiCsi2_t::Start(const camera_config_t &Config);
    NTSTATUS Stop();
    NTSTATUS PrepareHw(Resources_t &MipiRes);
};


struct Resources_t {
    DEVICE_CONTEXT *m_DeviceCtxPtr;
    MipiCsi2_t::MIPI_CSI2CSIS_REGS *m_MipiRegistersPtr;

    ACPI_READ_UINT32_FROM_OFFSET AcpiRgpr;
    ACPI_WRITE_UINT32_TO_OFFSET AcpiWgpr;

    UINT32 coreClockFrequencyHz;
    UINT32 phyClockFrequencyHz;
    UINT32 escClockFrequencyHz;
    PCHAR  csiDevNameA;
    UINT32 m_CpuId;
    UINT32 m_PixelMode;

    Resources_t(DEVICE_CONTEXT *deviceCtxPtr) : m_DeviceCtxPtr(deviceCtxPtr) {};
};
