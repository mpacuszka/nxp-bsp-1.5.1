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
    struct MIPI_CSI2_DWC_REGS
        /** MIPI_CSI2CSIS - Register Layout */
    {
        UINT32 IMX_DWC_MIPI_CSI2_VERSION;                     /**< Core version., offset: 0x0 */
        UINT32 IMX_DWC_MIPI_CSI2_N_LANES;                     /**< Number of lanes., offset: 0x4 */
        UINT32 IMX_DWC_MIPI_CSI2_CSI2_RESETN;                 /**< Logic Reset., offset: 0x8 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_MAIN;                 /**< Main interrupt status., offset: 0xC */
        UINT32 IMX_DWC_MIPI_CSI2_DATA_IDS1;                   /**< Data IDS., offset: 0x10 */
        UINT32 IMX_DWC_MIPI_CSI2_DATA_IDS2;                   /**< Data IDS., offset: 0x14 */
        UINT32 IMX_DWC_MIPI_CSI2_DPHY_CFG ;                   /**< DPHY cfg., offset: 0x18 */
        UINT32 IMX_DWC_MIPI_CSI2_DPHY_MODE;                   /**< DPHY mode., offset: 0x1C */
        UINT32 RESERVED_0[3];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_AP_MAIN;                   /**< INT status main., offset: 0x2C */
        UINT32 IMX_DWC_MIPI_CSI2_DATA_IDS_VC1;                   /**< Data VC 1., offset: 0x30 */
        UINT32 IMX_DWC_MIPI_CSI2_DATA_IDS_VC2;                   /**< Data VC 2., offset: 0x34 */
        UINT32 RESERVED_1[2];
        UINT32 IMX_DWC_MIPI_CSI2_DPHY_SHUTDOWNZ;               /**< PHY Shutdown., offset: 0x40 */
        UINT32 IMX_DWC_MIPI_CSI2_DPHY_RSTZ;                   /**< DPHY reset., offset: 0x44 */
        UINT32 IMX_DWC_MIPI_CSI2_PHY_RX;                      /**< RX PHY status., offset: 0x48 */
        UINT32 IMX_DWC_MIPI_CSI2_PHY_STOPSTATE;               /**< STOPSTATE PHY status., offset: 0x4C */
        UINT32 IMX_DWC_MIPI_CSI2_PHY_TEST_CTRL0;              /**< Synopsys D-PHY Test and Control interface 1., offset: 0x50 */
        UINT32 IMX_DWC_MIPI_CSI2_PHY_TEST_CTRL1;              /**< Synopsys D-PHY Test and Control interface 2., offset: 0x54 */
        UINT32 RESERVED_2[2];
        UINT32 IMX_DWC_MIPI_CSI2_PPI_PG_PATTERN_VRES;         /**< Patern Generator vertical resolution., offset: 0x60 */
        UINT32 IMX_DWC_MIPI_CSI2_PPI_PG_PATTERN_HRES;         /**< Patern Generator horizontal resolution., offset: 0x64 */
        UINT32 IMX_DWC_MIPI_CSI2_PPI_PG_CONFIG;               /**< Patern Generator., offset: 0x68 */
        UINT32 IMX_DWC_MIPI_CSI2_PPI_PG_ENABLE;               /**< Patern Generator enable., offset: 0x6C */
        UINT32 IMX_DWC_MIPI_CSI2_PPI_PG_STATUS;               /**< Patern Generator status., offset: 0x70 */
        UINT32 RESERVED_3[3];
        UINT32 IMX_DWC_MIPI_CSI2_IPI_MODE;                    /**< IPI Mode., offset: 0x80 */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_VCID;                    /**< IPI Virtual Channel., offset: 0x84 */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_DATA_TYPE;               /**< IPI Data Type., offset: 0x88 */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_MEM_FLUSH;               /**< IPI Flush Memory., offset: 0x8C */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_HSA_TIME;                /**< IPI HSA., offset: 0x90 */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_HBP_TIME;                /**< IPI_HBP., offset: 0x94 */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_HSD_TIME;                /**< IPI_HSD., offset: 0x98 */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_HLINE_TIME;              /**< IPI_HLINE., offset: 0x9C */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_SOFTRSTN;                /**< IPI Soft Reset., offset: 0xA0 */
        UINT32 RESERVED_4[2];
        UINT32 IMX_DWC_MIPI_CSI2_IPI_ADV_FEATURES;            /**< IPI Advanced Features., offset: 0xAC */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_VSA_LINES;               /**< IPI VSA., offset: 0xB0 */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_VBP_LINES;               /**< IPI VBP., offset: 0xB4 */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_VFP_LINES;               /**< IPI_VFP., offset: 0xB8 */
        UINT32 IMX_DWC_MIPI_CSI2_IPI_VACTIVE_LINES;           /**< IPI VACTIVE., offset: 0xBC */
        UINT32 RESERVED_5[2];
        UINT32 IMX_DWC_MIPI_CSI2_VC_EXTENSION;                /**< Virtual Channel Extesnsion., offset: 0xC8 */
        UINT32 IMX_DWC_MIPI_CSI2_PHY_CAL;                     /**< PHY Calibration., offset: 0xCC */
        UINT32 RESERVED_6[4];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_PHY_FATAL;            /**< Fatal interruption caused by PHY., offset: 0xE0 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_PHY_FATAL;           /**< Mask for fatal interruption caused by PHY., offset: 0xE4 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_FORCE_PHY_FATAL;         /**< Force for fatal interruption caused by PHY., offset: 0xE8 */
        UINT32 RESERVED_7[1];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_PKT_FATAL;            /**< Fatal interruption caused during Packet Construction., offset: 0xF0 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_PKT_FATAL;           /**< Mask for fatal interruption caused during Packet Construction., offset: 0xF4 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_FORCE_PKT_FATAL;         /**< Force for fatal interruption caused during Packet Construction., offset: 0xF8 */
        UINT32 RESERVED_8[5];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_PHY;                  /**< Interruption caused by PHY., offset: 0x110 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_PHY;                 /**< Mask for interruption caused by PHY., offset: 0x114 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_FORCE_PHY;               /**< Force for interruption caused by PHY., offset: 0x118 */
        UINT32 RESERVED_9[6];
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_LINE;                /**< Mask for fatal line., offset: 0x134 */
        UINT32 RESERVED_10[2];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_IPI_FATAL;            /**< Fatal Interruption caused by IPI interface., offset: 0x140 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_IPI_FATAL;           /**< Mask for fatal interruption caused by IPI interface., offset: 0x144 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_FORCE_IPI_FATAL;         /**< Force for fatal interruption caused by IPI interface., offset: 0x148 */
        UINT32 RESERVED_11[14];
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_AP_GENERIC;          /**< Mask for fatal AP generic., offset: 0x184 */
        UINT32 RESERVED_12[62];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_BNDRY_FRAME_FATAL;    /**< Fatal Interruption caused by Frame Boundaries., offset: 0x280 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_BNDRY_FRAME_FATAL;   /**< Mask for fatal interruption caused by Frame Boundaries., offset: 0x284 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_FORCE_BNDRY_FRAME_FATAL; /**< Force for fatal interruption caused by Frame Boundaries., offset: 0x288 */
        UINT32 RESERVED_13[1];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_SEQ_FRAME_FATAL;      /**< Fatal Interruption caused by Frame Sequence., offset: 0x290 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_SEQ_FRAME_FATAL;     /**< Mask for fatal interruption caused by Frame Sequence., offset: 0x294 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_FORCE_SEQ_FRAME_FATAL;   /**< Force for fatal interruption caused by Frame Sequence., offset: 0x298 */
        UINT32 RESERVED_14[1];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_CRC_FRAME_FATAL;      /**< Fatal Interruption caused by Frame CRC., offset: 0x2A0 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_CRC_FRAME_FATAL;     /**< Mask for fatal interruption caused by Frame CRC., offset: 0x2A4 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_FORCE_CRC_FRAME_FATAL;   /**< Force for fatal interruption caused by Frame CRC., offset: 0x2A8 */
        UINT32 RESERVED_15[1];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_PLD_CRC_FATAL;        /**< Fatal Interruption caused by Payload CRC., offset: 0x2B0 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_PLD_CRC_FATAL;       /**< Mask for fatal interruption caused by Payload CRC., offset: 0x2B4 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_FORCE_PLD_CRC_FATAL;     /**< Force for fatal interruption caused by Payload CRC., offset: 0x2B8 */
        UINT32 RESERVED_16[1];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_DATA_ID;              /**< Interruption caused by Data Type., offset: 0x2C0 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_DATA_ID;             /**< Mask for interruption caused by Data Type., offset: 0x2C4 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_FORCE_DATA_ID;           /**< Force for interruption caused by Data Type., offset: 0x2C8 */
        UINT32 RESERVED_17[1];
        UINT32 IMX_DWC_MIPI_CSI2_INT_ST_ECC_CORRECTED;        /**< Interruption caused by Header single bit errors., offset: 0x2D0 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_MSK_ECC_CORRECTED;       /**< Mas for interruption caused by Header single bit errors., offset: 0x2D4 */
        UINT32 IMX_DWC_MIPI_CSI2_INT_FORCE_ECC_CORRECTED;     /**< Force for interruption caused by Header single bit errors., offset: 0x2D8 */
        UINT32 RESERVED_18[9];
        UINT32 IMX_DWC_MIPI_CSI2_SCRAMBLING;                  /**< Data De-Scrambling., offset: 0x300 */
        UINT32 IMX_DWC_MIPI_CSI2_SCRAMBLING_SEED1;            /**< De-scrambler seed for lane1., offset: 0x304 */
        UINT32 IMX_DWC_MIPI_CSI2_SCRAMBLING_SEED2;            /**< De-scrambler seed for lane2., offset: 0x308 */
    };

    const static UINT32 IMX_DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLR  = 0x1U;
    const static UINT32 MIPI_CSI_IPI_MODE_IPI_ENABLE_MASK           = 0x1000000U;
    const static UINT32 MIPI_CSI_IPI_MODE_IPI_ENABLE_SHIFT          = 24U;
    const static UINT32 MIPI_CSI_IPI_MODE_IPI_CUT_THROUGH_MASK      = 0x10000U;
    const static UINT32 MIPI_CSI_IPI_MODE_IPI_CUT_THROUGH_SHIFT     = 16U;
    const static UINT32 MIPI_CSI_IPI_MEM_FLUSH_IPI_AUTO_FLUSH_MASK = 0x100U;
    const static UINT32 MIPI_CSI_IPI_MEM_FLUSH_IPI_AUTO_FLUSH_SHIFT = 8U;
#pragma pack( pop )

    /*! @name IMX_BLK_CTRL_MEDIAMIX_RESET - Reset control register */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_RESET = 0x00;

    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_ISI_APB_EN_MASK  = 0x4U;
    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_ISI_APB_EN_SHIFT = 2U;

    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_ISI_PROC_EN_MASK  = 0x8U;
    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_ISI_PROC_EN_SHIFT = 3U;

    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_PXP_APB_EN_MASK  = 0x80U;
    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_PXP_APB_EN_SHIFT = 7U;

    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_PXP_AXI_EN_MASK  = 0x100U;
    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_PXP_AXI_EN_SHIFT = 8U;

    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_CSI_APB_EN_MASK  = 0x200U;
    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_CSI_APB_EN_SHIFT = 9U;

    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_CAM_CLK_EN_MASK  = 0x400U;
    const static UINT32  MEDIAMIX_BLK_CTRL_RESET_CAM_CLK_EN_SHIFT = 10U;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_CLK - Clock control register */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_CLK                    = 0x04;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_LCDIF -LCDIF QoS */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_LCDIF                  = 0x0C;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_PXP - PXP QoS */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_PXP                    = 0x10;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_ISI0 - CACHE os ISI0 */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_ISI0                   = 0x14;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_ISI1 - QOS of ISI1 */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_ISI1                   = 0x1C;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX - Camera mux control register */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX             = 0x30;

    const static UINT32 IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX_DATA_TYPE_MASK = 0x1F8U;
    const static UINT32 IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX_DATA_TYPE_SHIFT = 3U;
    /*! DATA_TYPE - Data type
     *  0b011001..YUV420 10-bit
     *  0b011010..Legacy YUV420 (8-bit)
     *  0b011100..YUV420 8-bit (Chroma Shifted Pixel Sampling)
     *  0b011101..YUV420 10-bit (Chroma Shifted Pixel Sampling)
     *  0b011110..YUV422 8-bit
     *  0b011111..YUV422 10-bit
     *  0b100010..RGB565
     *  0b100011..RGB666
     *  0b100100..RGB888
     *  0b101000..RAW6
     *  0b101001..RAW7
     *  0b101010..RAW8
     *  0b101011..RAW10
     *  0b101100..RAW12
     *  0b101101..RAW14
     *  0b110000..User define32. Align with MIPI CSI ipi_mode[8] configure to 1'b0.
     *  0b011000..YUV420 8-bit
     *  0b110001..User define16. Align with MIPI CSI ipi_mode[8] configure to 1'b1.
     */
    const static UINT32 IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX_DATA_TYPE(UINT32 DATA_TYPE)
    {
        return (((UINT32)(DATA_TYPE << IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX_DATA_TYPE_SHIFT)) & IMX_BLK_CTRL_MEDIAMIX_CAMERA_MUX_DATA_TYPE_MASK);
    }

    const static UINT32 MEDIAMIX_BLK_CTRL_CAMERA_MUX_ENABLE_MASK  = 0x10000U;
    const static UINT32 MEDIAMIX_BLK_CTRL_CAMERA_MUX_ENABLE_SHIFT = 16U;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_PIXEL_CTRL_REG - Pixel control register */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_PIXEL_CTRL_REG = 0x3C;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_PIXEL_COUNT_REG - Pixel count */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_PIXEL_COUNT_REG = 0x40;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_LINE_COUNT_REG - Line count */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_LINE_COUNT_REG = 0x44;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_CSI - CSI register */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_CSI                    = 0x48;

    const static UINT32 IMX_BLK_CTRL_MEDIAMIX_CSI_CFGCLKFREQRANGE_MASK   = 0x0000003F;
    const static UINT32 IMX_BLK_CTRL_MEDIAMIX_CSI_CFGCLKFREQRANGE_SHIFT  = 0U;
    const static UINT32 IMX_BLK_CTRL_MEDIAMIX_CSI_HSCLKFREQRANGE_MASK    = 0x00007F00;
    const static UINT32 IMX_BLK_CTRL_MEDIAMIX_CSI_HSCLKFREQRANGE_SHIFT   = 8U;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_IF_CTRL_REG - Parallel camera interface register */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_IF_CTRL_REG            = 0x70;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_INTERFACE_STATUS - Parallel camera interface status */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_INTERFACE_STATUS       = 0x74;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_INTERFACE_CTRL_REG - Parallel camera interface control register */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_INTERFACE_CTRL_REG     = 0x78;
    /*! @} */

    /*! @name IMX_BLK_CTRL_MEDIAMIX_INTERFACE_CTRL_REG1 - Parallel camera interface control register */
    /*! @{ */
    static constexpr UINT32 IMX_BLK_CTRL_MEDIAMIX_INTERFACE_CTRL_REG1    = 0x7C;
    /*! @} */

private:
    volatile MIPI_CSI2_DWC_REGS *m_RegistersPtr;
    Resources_t *m_ResourcePtr;
    UINT32 m_CpuId;

#if DBG
    NTSTATUS MipiCsi2DwcDump();
    NTSTATUS MediaMixGasketDump();
#endif
    void MipiCsi2HostReset();
    void MipiCsi2DphyReset();
    void MipiCsi2TestControlInterfaceReset();
    void MediamixGasketReset();

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
    MipiCsi2_t::MIPI_CSI2_DWC_REGS *m_MipiRegistersPtr;

    ACPI_READ_UINT32_FROM_OFFSET AcpiRgpr;
    ACPI_WRITE_UINT32_TO_OFFSET AcpiWgpr;

    PCHAR  csiDevNameA;
    UINT32 m_CpuId;

    Resources_t(DEVICE_CONTEXT *deviceCtxPtr) : m_DeviceCtxPtr(deviceCtxPtr) {};
};
