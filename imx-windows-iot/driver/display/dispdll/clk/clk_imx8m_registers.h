/*
 * Copyright 2022 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef IMX_CLK_IMX8M_REGISTERS_H_
#define IMX_CLK_IMX8M_REGISTERS_H_


/*!
 * @file CCM_registers
 * @brief memory map for CCM module (iMX8M)
 *
 * Memory map for CCM module for i.MX8M
 */

/*
 * Clock Control Module (CCM)
 */
#define IMX_CCM_BASE                            0x30380000
#define IMX_CCM_SIZE                            0x10000
/* CCM Registers, offset from base address */
#define IMX_CCM_GPR0n(n)                        (0x000 + (4 * n))   /* General Purpose Register */
#define IMX_CCM_PLL_CTRLn(n)                    (0x800 + (16 * n))  /* PLL Control Register */
#define IMX_CCM_PLL_CTRLn_SET(n)                (0x804 + (16 * n))  /* PLL Control Register, Set */
#define IMX_CCM_PLL_CTRLn_CLR(n)                (0x808 + (16 * n))  /* PLL Control Register, Clear */
#define IMX_CCM_PLL_CTRLn_TOG(n)                (0x80C + (16 * n))  /* PLL Control Register, Toggle */
#define IMX_CCM_CCGRn(n)                        (0x4000 + (16 * n))  /* Clock Gating Register */
#define IMX_CCM_CCGRn_SET(n)                    (0x4004 + (16 * n))  /* Clock Gating Register, Set */
#define IMX_CCM_CCGRn_CLR(n)                    (0x4008 + (16 * n))  /* Clock Gating Register, Clear */
#define IMX_CCM_CCGRn_TOG(n)                    (0x400C + (16 * n))  /* Clock Gating Register, Toggle */
#define IMX_CCM_TARGET_ROOTn(n)                 (0x8000 + (128 * n))  /* Target Register */
#define IMX_CCM_TARGET_ROOTn_SET(n)             (0x8004 + (128 * n))  /* Target Register, Set */
#define IMX_CCM_TARGET_ROOTn_CLR(n)             (0x8008 + (128 * n))  /* Target Register, Clear */
#define IMX_CCM_TARGET_ROOTn_TOG(n)             (0x800C + (128 * n))  /* Target Register, Toggle */
#define IMX_CCM_MISCn(n)                        (0x8010 + (128 * n))  /* Miscellaneous Register */
#define IMX_CCM_MISCn_ROOT_SET(n)               (0x8014 + (128 * n))  /* Miscellaneous Register, Set */
#define IMX_CCM_MISCn_ROOT_CLR(n)               (0x8018 + (128 * n))  /* Miscellaneous Register, Clear */
#define IMX_CCM_MISCn_ROOT_TOG(n)               (0x801C + (128 * n))  /* Miscellaneous Register, Toggle */
#define IMX_CCM_POSTn(n)                        (0x8020 + (128 * n))  /* Post Divider Register */
#define IMX_CCM_POST_ROOTn_SET(n)               (0x8024 + (128 * n))  /* Post Divider Register, Set */
#define IMX_CCM_POST_ROOTn_CLR(n)               (0x8028 + (128 * n))  /* Post Divider Register, Clear */
#define IMX_CCM_POST_ROOTn_TOG(n)               (0x802C + (128 * n))  /* Post Divider Register, Toggle */
#define IMX_CCM_PREn(n)                         (0x8030 + (128 * n))  /* Post Divider Register */
#define IMX_CCM_ACCESS_CTRLn(n)                 (0x8070 + (128 * n))  /* Access Control Register */
#define IMX_CCM_ACCESS_CTRL_ROOTn_SET(n)        (0x8074 + (128 * n))  /* Access Control Register, Set */
#define IMX_CCM_ACCESS_CTRL_ROOTn_CLR(n)        (0x8078 + (128 * n))  /* Access Control Register, Clear */
#define IMX_CCM_ACCESS_CTRL_ROOTn_TOG(n)        (0x807C + (128 * n))  /* Access Control Register, Toggle */

/* PLL clock gate mapping */
#define IMX_CCM_ARM_PLL_CLK                     12U
#define IMX_CCM_GPU_PLL_CLK                     13U
#define IMX_CCM_VPU_PLL_CLK                     14U
#define IMX_CCM_SYSTEM_PLL1_CLK                 16U
#define IMX_CCM_SYSTEM_PLL1_DIV2                17U
#define IMX_CCM_SYSTEM_PLL1_DIV3                18U
#define IMX_CCM_SYSTEM_PLL1_DIV4                19U
#define IMX_CCM_SYSTEM_PLL1_DIV5                20U
#define IMX_CCM_SYSTEM_PLL1_DIV6                21U
#define IMX_CCM_SYSTEM_PLL1_DIV8                22U
#define IMX_CCM_SYSTEM_PLL1_DIV10               23U
#define IMX_CCM_SYSTEM_PLL1_DIV20               24U
#define IMX_CCM_SYSTEM_PLL2_CLK                 25U
#define IMX_CCM_SYSTEM_PLL2_DIV2                26U
#define IMX_CCM_SYSTEM_PLL2_DIV3                27U
#define IMX_CCM_SYSTEM_PLL2_DIV4                28U
#define IMX_CCM_SYSTEM_PLL2_DIV5                29U
#define IMX_CCM_SYSTEM_PLL2_DIV6                30U
#define IMX_CCM_SYSTEM_PLL2_DIV8                31U
#define IMX_CCM_SYSTEM_PLL2_DIV10               32U
#define IMX_CCM_SYSTEM_PLL2_DIV20               33U
#define IMX_CCM_SYSTEM_PLL3_CLK                 34U
#define IMX_CCM_AUDIO_PLL1_CLK                  35U
#define IMX_CCM_AUDIO_PLL2_CLK                  36U
#define IMX_CCM_VIDEO_PLL1_CLK                  37U

/* Register PLL_CTRL - CCM PLL Control Register */
#define IMX_CCM_PLL_CTRL                         0x800
/* Register PLL_CTRL_SET - CCM PLL Control Register */
#define IMX_CCM_PLL_CTRL_SET                     0x804
/* Register PLL_CTRL_CLR - CCM PLL Control Register */
#define IMX_CCM_PLL_CTRL_CLR                     0x808
/* Register PLL_CTRL_TOG - CCM PLL Control Register */
#define IMX_CCM_PLL_CTRL_TOG                     0x80C

/* Field SETTING0 -  */
#define IMX_CCM_PLL_CTRL_SETTING0_SHIFT          0
#define IMX_CCM_PLL_CTRL_SETTING0_MASK           0x3
#define IMX_CCM_PLL_CTRL_SETTING0(v)             (((v) << 0) & 0x3)
/* Field SETTING1 -  */
#define IMX_CCM_PLL_CTRL_SETTING1_SHIFT          4
#define IMX_CCM_PLL_CTRL_SETTING1_MASK           0x30
#define IMX_CCM_PLL_CTRL_SETTING1(v)             (((v) << 0x4) & 0x30)
/* Field SETTING2 -  */
#define IMX_CCM_PLL_CTRL_SETTING2_SHIFT          8
#define IMX_CCM_PLL_CTRL_SETTING2_MASK           0x300
#define IMX_CCM_PLL_CTRL_SETTING2(v)             (((v) << 0x8) & 0x300)
/* Field SETTING3 -  */
#define IMX_CCM_PLL_CTRL_SETTING3_SHIFT          12
#define IMX_CCM_PLL_CTRL_SETTING3_MASK           0x3000
#define IMX_CCM_PLL_CTRL_SETTING3(v)             (((v) << 0xC) & 0x3000)

/* Register CCGR - CCM Clock Gating Register */
#define IMX_CCM_CCGR                             0x4000
/* Register CCGR_SET - CCM Clock Gating Register */
#define IMX_CCM_CCGR_SET                         0x4004
/* Register CCGR_CLR - CCM Clock Gating Register */
#define IMX_CCM_CCGR_CLR                         0x4008
/* Register CCGR_TOG - CCM Clock Gating Register */
#define IMX_CCM_CCGR_TOG                         0x400C

/* Field SETTING0 -  */
#define IMX_CCM_CCGR_SETTING0_SHIFT              0
#define IMX_CCM_CCGR_SETTING0_MASK               0x3
#define IMX_CCM_CCGR_SETTING0(v)                 (((v) << 0) & 0x3)
/* Field SETTING1 -  */
#define IMX_CCM_CCGR_SETTING1_SHIFT              4
#define IMX_CCM_CCGR_SETTING1_MASK               0x30
#define IMX_CCM_CCGR_SETTING1(v)                 (((v) << 0x4) & 0x30)
/* Field SETTING2 -  */
#define IMX_CCM_CCGR_SETTING2_SHIFT              8
#define IMX_CCM_CCGR_SETTING2_MASK               0x300
#define IMX_CCM_CCGR_SETTING2(v)                 (((v) << 0x8) & 0x300)
/* Field SETTING3 -  */
#define IMX_CCM_CCGR_SETTING3_SHIFT              12
#define IMX_CCM_CCGR_SETTING3_MASK               0x3000
#define IMX_CCM_CCGR_SETTING3(v)                 (((v) << 0xC) & 0x3000)

/* Register TARGET_ROOT - Target Register */
#define IMX_CCM_TARGET_ROOT                      0x8000
/* Register TARGET_ROOT_SET - Target Register */
#define IMX_CCM_TARGET_ROOT_SET                  0x8004
/* Register TARGET_ROOT_CLR - Target Register */
#define IMX_CCM_TARGET_ROOT_CLR                  0x8008
/* Register TARGET_ROOT_TOG - Target Register */
#define IMX_CCM_TARGET_ROOT_TOG                  0x800C

#define IMX_CCM_TARGET_ROOT_ENABLE_MASK         (0x1 << 28)           /* Enable this clock */
#define IMX_CCM_TARGET_ROOT_ENABLE_SHIFT        28
#define IMX_CCM_TARGET_ROOT_MUX_MASK            (0x07 << 24)          /* Selection of clock sources */
#define IMX_CCM_TARGET_ROOT_MUX_SHIFT           24
#define IMX_CCM_TARGET_ROOT_MUX_VALUE(x)        (((x) & 0x07) << 24)
#define IMX_CCM_TARGET_ROOT_PRE_PODF_MASK       (0x07 << 16)          /* Pre divider divide the number */
#define IMX_CCM_TARGET_ROOT_PRE_PODF_SHIFT      16
#define IMX_CCM_TARGET_ROOT_PRE_PODF(n)         ((n - 1) << 16)
#define IMX_CCM_TARGET_ROOT_PRE_MAX_DIV         8
#define IMX_CCM_TARGET_ROOT_POST_PODF_MASK      (0x3F << 0)           /* Pre divider divide the number */
#define IMX_CCM_TARGET_ROOT_POST_PODF_SHIFT     0
#define IMX_CCM_TARGET_ROOT_POST_PODF(n)        ((((n) - 1) & 0x3F) << 0)
#define IMX_CCM_TARGET_ROOT_POST_MAX_DIV        64

/* Register POST - Post Divider Register */
#define IMX_CCM_POST                             0x8020
/* Register POST_ROOT_SET - Post Divider Register */
#define IMX_CCM_POST_ROOT_SET                    0x8024
/* Register POST_ROOT_CLR - Post Divider Register */
#define IMX_CCM_POST_ROOT_CLR                    0x8028
/* Register POST_ROOT_TOG - Post Divider Register */
#define IMX_CCM_POST_ROOT_TOG                    0x802C

/* Field POST_PODF -  */
#define IMX_CCM_POST_POST_PODF_SHIFT             0
#define IMX_CCM_POST_POST_PODF_MASK              0x3F
#define IMX_CCM_POST_POST_PODF(v)                (((v) << 0) & 0x3F)
/* Field BUSY1 -  */
#define IMX_CCM_POST_BUSY1_SHIFT                 7
#define IMX_CCM_POST_BUSY1_MASK                  0x80
#define IMX_CCM_POST_BUSY1(v)                    (((v) << 0x7) & 0x80)
/* Field SELECT -  */
#define IMX_CCM_POST_SELECT_SHIFT                28
#define IMX_CCM_POST_SELECT_MASK                 0x10000000
#define IMX_CCM_POST_SELECT(v)                   (((v) << 0x1C) & 0x10000000)
/* Field BUSY2 -  */
#define IMX_CCM_POST_BUSY2_SHIFT                 31
#define IMX_CCM_POST_BUSY2_MASK                  0x80000000
#define IMX_CCM_POST_BUSY2(v)                    (((v) << 0x1F) & 0x80000000)

/* Register PRE - Pre Divider Register */
#define IMX_CCM_PRE                              0x8030
/* Register PRE_ROOT_SET - Pre Divider Register */
#define IMX_CCM_PRE_ROOT_SET                     0x8034
/* Register PRE_ROOT_CLR - Pre Divider Register */
#define IMX_CCM_PRE_ROOT_CLR                     0x8038
/* Register PRE_ROOT_TOG - Pre Divider Register */
#define IMX_CCM_PRE_ROOT_TOG                     0x803C

/* Field PRE_PODF_B -  */
#define IMX_CCM_PRE_PRE_PODF_B_SHIFT             0
#define IMX_CCM_PRE_PRE_PODF_B_MASK              0x7
#define IMX_CCM_PRE_PRE_PODF_B(v)                (((v) << 0) & 0x7)
/* Field BUSY0 -  */
#define IMX_CCM_PRE_BUSY0_SHIFT                  3
#define IMX_CCM_PRE_BUSY0_MASK                   0x8
#define IMX_CCM_PRE_BUSY0(v)                     (((v) << 0x3) & 0x8)
/* Field MUX_B -  */
#define IMX_CCM_PRE_MUX_B_SHIFT                  8
#define IMX_CCM_PRE_MUX_B_MASK                   0x700
#define IMX_CCM_PRE_MUX_B(v)                     (((v) << 0x8) & 0x700)
/* Field EN_B -  */
#define IMX_CCM_PRE_EN_B_SHIFT                   12
#define IMX_CCM_PRE_EN_B_MASK                    0x1000
#define IMX_CCM_PRE_EN_B(v)                      (((v) << 0xC) & 0x1000)
/* Field BUSY1 -  */
#define IMX_CCM_PRE_BUSY1_SHIFT                  15
#define IMX_CCM_PRE_BUSY1_MASK                   0x8000
#define IMX_CCM_PRE_BUSY1(v)                     (((v) << 0xF) & 0x8000)
/* Field PRE_PODF_A -  */
#define IMX_CCM_PRE_PRE_PODF_A_SHIFT             16
#define IMX_CCM_PRE_PRE_PODF_A_MASK              0x70000
#define IMX_CCM_PRE_PRE_PODF_A(v)                (((v) << 0x10) & 0x70000)
/* Field BUSY3 -  */
#define IMX_CCM_PRE_BUSY3_SHIFT                  19
#define IMX_CCM_PRE_BUSY3_MASK                   0x80000
#define IMX_CCM_PRE_BUSY3(v)                     (((v) << 0x13) & 0x80000)
/* Field MUX_A -  */
#define IMX_CCM_PRE_MUX_A_SHIFT                  24
#define IMX_CCM_PRE_MUX_A_MASK                   0x7000000
#define IMX_CCM_PRE_MUX_A(v)                     (((v) << 0x18) & 0x7000000)
/* Field EN_A -  */
#define IMX_CCM_PRE_EN_A_SHIFT                   28
#define IMX_CCM_PRE_EN_A_MASK                    0x10000000
#define IMX_CCM_PRE_EN_A(v)                      (((v) << 0x1C) & 0x10000000)
/* Field BUSY4 -  */
#define IMX_CCM_PRE_BUSY4_SHIFT                  31
#define IMX_CCM_PRE_BUSY4_MASK                   0x80000000
#define IMX_CCM_PRE_BUSY4(v)                     (((v) << 0x1F) & 0x80000000)

/*
 * Clock Control Module Analog part (CCM_ANALOG)
 */
#define IMX_CCM_ANALOG_BASE                     0x30360000
#define IMX_CCM_ANALOG_SIZE                     0x10000
/* BEGIN: CCM_ANALOG regsiters specific for IMX8MP/MM/MN */
/* Offsets of the registers from IMX_CCM_ANALOG_BASE */
#define IMX_CCM_ANALOG_VIDEO_PLL1_GEN_CTRL       0x28
#define IMX_CCM_ANALOG_VIDEO_PLL1_FDIV_CTL0      0x2C
#define IMX_CCM_ANALOG_VIDEO_PLL1_FDIV_CTL1      0x30
#define IMX_CCM_ANALOG_VIDEO_PLL1_SSCG_CTRL      0x34
#define IMX_CCM_ANALOG_VIDEO_PLL1_MNIT_CTRL      0x38
#define IMX_CCM_ANALOG_DRAM_PLL_GEN_CTRL         0x50
#define IMX_CCM_ANALOG_DRAM_PLL_FDIV_CTL0        0x54
#define IMX_CCM_ANALOG_ARM_PLL_GEN_CTRL          0x84
#define IMX_CCM_ANALOG_SYS_PLL1_GEN_CTRL         0x94
#define IMX_CCM_ANALOG_SYS_PLL2_GEN_CTRL         0x104
#define IMX_CCM_ANALOG_SYS_PLL3_GEN_CTRL         0x114
#define IMX_CCM_ANALOG_SYS_PLL3_FDIV_CTL0        0x118
#define IMX_CCM_ANALOG_ANAMIX_PLL_MNIT_CTL       0x128

/* Registers offsets - independent of a PLL type */
#define IMX_CCM_ANALOG_GEN_CTRL_OFFSET           0x00
#define IMX_CCM_ANALOG_FDIV_CTL0_OFFSET          0x04
#define IMX_CCM_ANALOG_FDIV_CTL1_OFFSET          0x08
#define IMX_CCM_ANALOG_SSCG_CTRL_OFFSET          0x0C
#define IMX_CCM_ANALOG_MNIT_CTRL_OFFSET          0x10

/* Bitfields - independent of a PLL type */
/* Field PLL_REF_CLK_SEL -  */
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_REF_CLK_SEL_SHIFT   0
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_REF_CLK_SEL_MASK    0x3
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_REF_CLK_SEL(v)      (((v) << 0) & 0x3)
/* Field PAD_CLK_SEL -  */
#define IMX_CCM_ANALOG_GEN_CTRL_PAD_CLK_SEL_SHIFT       2
#define IMX_CCM_ANALOG_GEN_CTRL_PAD_CLK_SEL_MASK        0xC
#define IMX_CCM_ANALOG_GEN_CTRL_PAD_CLK_SEL(v)          (((v) << 2) & 0xC)
/* Field PLL_BYPASS -  */
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_BYPASS_SHIFT        4
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_BYPASS_MASK         0x10
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_BYPASS(v)           (((v) << 4) & 0x10)
/* Field PLL_RST_OVERRIDE -  */
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_RST_OVERRIDE_SHIFT  8
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_RST_OVERRIDE_MASK   0x100
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_RST_OVERRIDE(v)     (((v) << 8) & 0x100)
/* Field PLL_RST -  */
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_RST_SHIFT           9
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_RST_MASK            0x200
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_RST(v)              (((v) << 9) & 0x200)
/* Field PLL_CLKE_OVERRIDE -  */
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_CLKE_OVERRIDE_SHIFT 12
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_CLKE_OVERRIDE_MASK  0x1000
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_CLKE_OVERRIDE(v)    (((v) << 12) & 0x1000)
/* Field PLL_CLKE -  */
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_CLKE_SHIFT          13
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_CLKE_MASK           0x2000
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_CLKE(v)             (((v) << 13) & 0x2000)
/* Field PLL_EXT_BYPASS -  */
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_EXT_BYPASS_SHIFT    16
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_EXT_BYPASS_MASK     0x10000
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_EXT_BYPASS(v)       (((v) << 16) & 0x10000)
/* Field PLL_LOCK -  */
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_LOCK_SHIFT          31
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_LOCK_MASK           0x80000000
#define IMX_CCM_ANALOG_GEN_CTRL_PLL_LOCK(v)             (((v) << 31) & 0x80000000)

/* Field PLL_POST_DIV -  */
#define IMX_CCM_ANALOG_FDIV_CTL0_PLL_POST_DIV_SHIFT     0
#define IMX_CCM_ANALOG_FDIV_CTL0_PLL_POST_DIV_MASK      0x7
#define IMX_CCM_ANALOG_FDIV_CTL0_PLL_POST_DIV(v)        (((v) << 0) & 0x7)
/* Field PLL_PRE_DIV -  */
#define IMX_CCM_ANALOG_FDIV_CTL0_PLL_PRE_DIV_SHIFT      4
#define IMX_CCM_ANALOG_FDIV_CTL0_PLL_PRE_DIV_MASK       0x3F0
#define IMX_CCM_ANALOG_FDIV_CTL0_PLL_PRE_DIV(v)         (((v) << 4) & 0x3F0)
/* Field PLL_MAIN_DIV -  */
#define IMX_CCM_ANALOG_FDIV_CTL0_PLL_MAIN_DIV_SHIFT     12
#define IMX_CCM_ANALOG_FDIV_CTL0_PLL_MAIN_DIV_MASK      0x3FF000
#define IMX_CCM_ANALOG_FDIV_CTL0_PLL_MAIN_DIV(v)        (((v) << 12) & 0x3FF000)

/* Field PLL_DSM -  */
#define IMX_CCM_ANALOG_FDIV_CTL1_PLL_DSM_SHIFT          0
#define IMX_CCM_ANALOG_FDIV_CTL1_PLL_DSM_MASK           0xFFFF
#define IMX_CCM_ANALOG_FDIV_CTL1_PLL_DSM(v)             (((v) << 0) & 0xFFFF)
/* PLL output test enable register */
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_CKE_SHIFT   24
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_CKE_MASK    0x1000000
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_SEL_SHIFT   20
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_SEL_MASK    0xF00000
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_SEL(v)      (((v) << 20) & 0xF00000)
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_DIV_SHIFT   16
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_DIV_MASK    0xF0000
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT2_DIV(v)      (((v) << 16) & 0xF0000)

#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_CKE_SHIFT   8
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_CKE_MASK    0x100
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_SEL_SHIFT   4
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_SEL_MASK    0xF0
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_SEL(v)      (((v) << 4) & 0xF0)
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_DIV_SHIFT   0
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_DIV_MASK    0xF
#define IMX_CCM_ANALOG_PLL_MNIT_CTL_CLKOUT1_DIV(v)      (((v) << 0) & 0xF)
/* Selector values for CLKOUTx_SEL bitfield */
#define IMX_CCM_ANALOG_CLKOUT_AUDIO_PLL1_CLK            0
#define IMX_CCM_ANALOG_CLKOUT_AUDIO_PLL2_CLK            1
#define IMX_CCM_ANALOG_CLKOUT_VIDEO_PLL1_CLK            2
#define IMX_CCM_ANALOG_CLKOUT_MISC_MNIT_CLK             4
#define IMX_CCM_ANALOG_CLKOUT_GPU_PLL_CLK               5
#define IMX_CCM_ANALOG_CLKOUT_VPU_PLL_CLK               6
#define IMX_CCM_ANALOG_CLKOUT_ARM_PLL_CLK               7
#define IMX_CCM_ANALOG_CLKOUT_SYSTEM_PLL1_CLK           8
#define IMX_CCM_ANALOG_CLKOUT_SYSTEM_PLL2_CLK           9
#define IMX_CCM_ANALOG_CLKOUT_SYSTEM_PLL3_CLK           10
#define IMX_CCM_ANALOG_CLKOUT_CLKIN1                    11
#define IMX_CCM_ANALOG_CLKOUT_CLKIN2                    12
#define IMX_CCM_ANALOG_CLKOUT_SYSOSC_24M_CLK            13
#define IMX_CCM_ANALOG_CLKOUT_OSC_32K_CLK               15
/* END: CCM_ANALOG regsiters specific for IMX8MP/MM/MN */

#endif

