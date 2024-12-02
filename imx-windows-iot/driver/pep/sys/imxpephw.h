// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022 NXP
// Licensed under the MIT License.
//
//
// Module Name:
//
//   imxpephw.h
//
// Abstract:
//
//   IMX CCM and GPC Hardware Definitions
//

#ifndef _IMXPEP_HW_H_
#define _IMXPEP_HW_H_

#pragma warning(push)
#pragma warning(disable:4201) // nameless struct/union

#include <pshpack1.h>

// IMX8
#define MX8_CCM_BASE                            0x30380000
#define MX8_CCM_LENGTH                          0x10000

#define MX8_CCM_ANALOG_BASE                     0x30360000
#define MX8_CCM_ANALOG_SIZE                     0x10000

#define MX8_GPC_BASE                            0x303A0000
#define MX8_GPC_LENGTH                          0x10000

#define CSU_INDEX_INVALID 0xFFFF

#define ACPI_OBJECT_NAME_STA  ((ULONG)'ATS_')

/* IMX8 register structures */
struct MX8_CCM_ANALOG_REGISTERS {
    ULONG AUDIO_PLL1_CFG0;                        /**< AUDIO PLL1 Configuration 0 Register, offset: 0x0 */
    ULONG AUDIO_PLL1_CFG1;                        /**< AUDIO PLL1 Configuration 1 Register, offset: 0x4 */
    ULONG AUDIO_PLL2_CFG0;                        /**< AUDIO PLL2 Configuration 0 Register, offset: 0x8 */
    ULONG AUDIO_PLL2_CFG1;                        /**< AUDIO PLL2 Configuration 1 Register, offset: 0xC */
    ULONG VIDEO_PLL1_CFG0;                        /**< VIDEO PLL Configuration 0 Register, offset: 0x10 */
    ULONG VIDEO_PLL1_CFG1;                        /**< VIDEO PLL Configuration 1 Register, offset: 0x14 */
    ULONG GPU_PLL_CFG0;                           /**< GPU PLL Configuration 0 Register, offset: 0x18 */
    ULONG GPU_PLL_CFG1;                           /**< GPU PLL Configuration 1 Register, offset: 0x1C */
    ULONG VPU_PLL_CFG0;                           /**< VPU PLL Configuration 0 Register, offset: 0x20 */
    ULONG VPU_PLL_CFG1;                           /**< VPU PLL Configuration 1 Register, offset: 0x24 */
    ULONG ARM_PLL_CFG0;                           /**< ARM PLL Configuration 0 Register, offset: 0x28 */
    ULONG ARM_PLL_CFG1;                           /**< ARM PLL Configuration 1 Register, offset: 0x2C */
    ULONG SYS_PLL1_CFG0;                          /**< System PLL Configuration 0 Register, offset: 0x30 */
    ULONG SYS_PLL1_CFG1;                          /**< System_PLL Configuration 1 Register, offset: 0x34 */
    ULONG SYS_PLL1_CFG2;                          /**< System_PLL Configuration 2 Register, offset: 0x38 */
    ULONG SYS_PLL2_CFG0;                          /**< System PLL Configuration 0 Register, offset: 0x3C */
    ULONG SYS_PLL2_CFG1;                          /**< System_PLL Configuration 1 Register, offset: 0x40 */
    ULONG SYS_PLL2_CFG2;                          /**< System_PLL Configuration 2 Register, offset: 0x44 */
    ULONG SYS_PLL3_CFG0;                          /**< System PLL Configuration 0 Register, offset: 0x48 */
    ULONG SYS_PLL3_CFG1;                          /**< System_PLL Configuration 1 Register, offset: 0x4C */
    ULONG SYS_PLL3_CFG2;                          /**< System_PLL Configuration 2 Register, offset: 0x50 */
    ULONG VIDEO_PLL2_CFG0;                        /**< VIDEO PLL2 Configuration 0 Register, offset: 0x54 */
    ULONG VIDEO_PLL2_CFG1;                        /**< VIDEO PLL2 Configuration 1 Register, offset: 0x58 */
    ULONG VIDEO_PLL2_CFG2;                        /**< VIDEO PLL2 Configuration 2 Register, offset: 0x5C */
    ULONG DRAM_PLL_CFG0;                          /**< DRAM PLL Configuration 0 Register, offset: 0x60 */
    ULONG DRAM_PLL_CFG1;                          /**< DRAM PLL Configuration 1 Register, offset: 0x64 */
    ULONG DRAM_PLL_CFG2;                          /**< DRAM PLL Configuration 2 Register, offset: 0x68 */
    ULONG DIGPROG;                                /**< DIGPROG Register, offset: 0x6C */
    ULONG OSC_MISC_CFG;                           /**< Osc Misc Configuration Register, offset: 0x70 */
    ULONG PLLOUT_MONITOR_CFG;                     /**< PLLOUT Monitor Configuration Register, offset: 0x74 */
    ULONG FRAC_PLLOUT_DIV_CFG;                    /**< Fractional PLLOUT Divider Configuration Register, offset: 0x78 */
    ULONG SCCG_PLLOUT_DIV_CFG;                    /**< SCCG PLLOUT Divider Configuration Register, offset: 0x7C */
};

/** IMX8 GPC - Peripheral register structure */
struct MX8_GPC_REGISTERS {
    ULONG LPCR_A53_BSC;                           /**< Basic Low power control register of A53 platform, offset: 0x0 */
    ULONG LPCR_A53_AD;                            /**< Advanced Low power control register of A53 platform, offset: 0x4 */
    ULONG LPCR_M4;                                /**< Low power control register of CPU1, offset: 0x8 */
    UCHAR RESERVED_0[8];
    ULONG SLPCR;                                  /**< System low power control register, offset: 0x14 */
    ULONG MST_CPU_MAPPING;                        /**< MASTER LPM Handshake, offset: 0x18 */
    UCHAR RESERVED_1[4];
    ULONG MLPCR;                                  /**< Memory low power control register, offset: 0x20 */
    ULONG PGC_ACK_SEL_A53;                        /**< PGC acknowledge signal selection of A53 platform, offset: 0x24 */
    ULONG PGC_ACK_SEL_M4;                         /**< PGC acknowledge signal selection of M4 platform, offset: 0x28 */
    ULONG MISC;                                   /**< GPC Miscellaneous register, offset: 0x2C */
    ULONG IMR_CORE0_A53[4];                       /**< IRQ masking register 1 of A53 core0..IRQ masking register 4 of A53 core0, array offset: 0x30, array step: 0x4 */
    ULONG IMR_CORE1_A53[4];                       /**< IRQ masking register 1 of A53 core1..IRQ masking register 4 of A53 core1, array offset: 0x40, array step: 0x4 */
    ULONG IMR_M4[4];                              /**< IRQ masking register 1 of M4..IRQ masking register 4 of M4, array offset: 0x50, array step: 0x4 */
    UCHAR RESERVED_2[16];
    ULONG ISR_A53[4];                             /**< IRQ status register 1 of A53..IRQ status register 4 of A53, array offset: 0x70, array step: 0x4 */
    ULONG ISR_M4[4];                              /**< IRQ status register 1 of M4..IRQ status register 4 of M4, array offset: 0x80, array step: 0x4 */
    UCHAR RESERVED_3[32];
    ULONG SLT0_CFG;                               /**< Slot configure register for A53 core, offset: 0xB0 */
    ULONG SLT1_CFG;                               /**< Slot configure register for A53 core, offset: 0xB4 */
    ULONG SLT2_CFG;                               /**< Slot configure register for A53 core, offset: 0xB8 */
    ULONG SLT3_CFG;                               /**< Slot configure register for A53 core, offset: 0xBC */
    ULONG SLT4_CFG;                               /**< Slot configure register for A53 core, offset: 0xC0 */
    ULONG SLT5_CFG;                               /**< Slot configure register for A53 core, offset: 0xC4 */
    ULONG SLT6_CFG;                               /**< Slot configure register for A53 core, offset: 0xC8 */
    ULONG SLT7_CFG;                               /**< Slot configure register for A53 core, offset: 0xCC */
    ULONG SLT8_CFG;                               /**< Slot configure register for A53 core, offset: 0xD0 */
    ULONG SLT9_CFG;                               /**< Slot configure register for A53 core, offset: 0xD4 */
    ULONG SLT10_CFG;                              /**< Slot configure register for A53 core, offset: 0xD8 */
    ULONG SLT11_CFG;                              /**< Slot configure register for A53 core, offset: 0xDC */
    ULONG SLT12_CFG;                              /**< Slot configure register for A53 core, offset: 0xE0 */
    ULONG SLT13_CFG;                              /**< Slot configure register for A53 core, offset: 0xE4 */
    ULONG SLT14_CFG;                              /**< Slot configure register for A53 core, offset: 0xE8 */
    ULONG PGC_CPU_0_1_MAPPING;                    /**< PGC CPU mapping, offset: 0xEC */
    ULONG CPU_PGC_SW_PUP_REQ;                     /**< CPU PGC software power up trigger, offset: 0xF0 */
    ULONG MIX_PGC_SW_PUP_REQ;                     /**< MIX PGC software power up trigger, offset: 0xF4 */
    ULONG PU_PGC_SW_PUP_REQ;                      /**< PU PGC software up trigger, offset: 0xF8 */
    ULONG CPU_PGC_SW_PDN_REQ;                     /**< CPU PGC software down trigger, offset: 0xFC */
    ULONG MIX_PGC_SW_PDN_REQ;                     /**< MIX PGC software power down trigger, offset: 0x100 */
    ULONG PU_PGC_SW_PDN_REQ;                      /**< PU PGC software down trigger, offset: 0x104 */
    ULONG LPCR_A53_BSC2;                          /**< Basic Low power control register of A53 platform, offset: 0x108 */
    UCHAR RESERVED_4[36];
    ULONG CPU_PGC_PUP_STATUS1;                    /**< CPU PGC software up trigger status1, offset: 0x130 */
    ULONG A53_MIX_PGC_PUP_STATUS[3];              /**< A53 MIX software up trigger status register, array offset: 0x134, array step: 0x4 */
    ULONG M4_MIX_PGC_PUP_STATUS[3];               /**< M4 MIX PGC software up trigger status register, array offset: 0x140, array step: 0x4 */
    ULONG A53_PU_PGC_PUP_STATUS[3];               /**< A53 PU software up trigger status register, array offset: 0x14C, array step: 0x4 */
    ULONG M4_PU_PGC_PUP_STATUS[3];                /**< M4 PU PGC software up trigger status register, array offset: 0x158, array step: 0x4 */
    UCHAR RESERVED_5[12];
    ULONG CPU_PGC_PDN_STATUS1;                    /**< CPU PGC software dn trigger status1, offset: 0x170 */
    ULONG A53_MIX_PGC_PDN_STATUS[3];              /**< A53 MIX software down trigger status register, array offset: 0x174, array step: 0x4 */
    ULONG M4_MIX_PGC_PDN_STATUS[3];               /**< M4 MIX PGC software power down trigger status register, array offset: 0x180, array step: 0x4 */
    ULONG A53_PU_PGC_PDN_STATUS[3];               /**< A53 PU PGC software down trigger status, array offset: 0x18C, array step: 0x4 */
    ULONG M4_PU_PGC_PDN_STATUS[3];                /**< M4 PU PGC software down trigger status, array offset: 0x198, array step: 0x4 */
    UCHAR RESERVED_6[12];
    ULONG A53_MIX_PDN_FLG;                        /**< A53 MIX PDN FLG, offset: 0x1B0 */
    ULONG A53_PU_PDN_FLG;                         /**< A53 PU PDN FLG, offset: 0x1B4 */
    ULONG M4_MIX_PDN_FLG;                         /**< M4 MIX PDN FLG, offset: 0x1B8 */
    ULONG M4_PU_PDN_FLG;                          /**< M4 PU PDN FLG, offset: 0x1BC */
    ULONG IMR_CORE2_A53[4];                       /**< IRQ masking register 1 of A53 core2..IRQ masking register 4 of A53 core2, array offset: 0x1C0, array step: 0x4 */
    ULONG IMR_CORE3_A53[4];                       /**< IRQ masking register 1 of A53 core3..IRQ masking register 4 of A53 core3, array offset: 0x1D0, array step: 0x4 */
    ULONG ACK_SEL_A53_PU;                         /**< PGC acknowledge signal selection of A53 platform for PUs, offset: 0x1E0 */
    ULONG ACK_SEL_M4_PU;                          /**< PGC acknowledge signal selection of M4 platform for PUs, offset: 0x1E4 */
    ULONG SLT15_CFG;                              /**< Slot configure register for A53 core, offset: 0x1E8 */
    ULONG SLT16_CFG;                              /**< Slot configure register for A53 core, offset: 0x1EC */
    ULONG SLT17_CFG;                              /**< Slot configure register for A53 core, offset: 0x1F0 */
    ULONG SLT18_CFG;                              /**< Slot configure register for A53 core, offset: 0x1F4 */
    ULONG SLT19_CFG;                              /**< Slot configure register for A53 core, offset: 0x1F8 */
    ULONG PU_PWRHSK;                              /**< Power handshake register, offset: 0x1FC */
    ULONG SLT_CFG_PU[20];                         /**< Slot configure register for PUs, array offset: 0x200, array step: 0x4 */
};

#define GPC_PGC_A53CORE0_CTRL   0x800
#define GPC_PGC_A53CORE1_CTRL   0x840
#define GPC_PGC_A53CORE2_CTRL   0x880
#define GPC_PGC_A53CORE3_CTRL   0x8C0
#define GPC_PGC_A53SCU_CTRL     0x900
#define GPC_PGC_A53SCU_AUXSW    0x910
#define GPC_PGC_MIX_AUXSW       0xA10

#include <poppack.h> // pshpack1.h
#pragma warning(pop) // nameless struct/union

#endif // _IMXPEP_HW_H_
