/*
 * Copyright 2023 NXP
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
#ifndef _INCL_IMX_ANATOP_H
#define _INCL_IMX_ANATOP_H

#define IMX_ANATOP_PLL_CTRL                       0x00     /* PLL Control Register */
#define IMX_ANATOP_PLL_CTRL_SET                   0x04
#define IMX_ANATOP_PLL_CTRL_CLR                   0x08
#define IMX_ANATOP_PLL_CTRL_TOG                   0x0C
#define IMX_ANATOP_PLL_ANA_PRG                    0x10     /* PLL analog program Register */
#define IMX_ANATOP_PLL_ANA_PRG_SET                0x14
#define IMX_ANATOP_PLL_ANA_PRG_CLR                0x18
#define IMX_ANATOP_PLL_ANA_PRG_TOG                0x1C
#define IMX_ANATOP_PLL_TEST                       0x20     /* PLL Test Register */
#define IMX_ANATOP_PLL_TEST_SET                   0x24
#define IMX_ANATOP_PLL_TEST_CLR                   0x28
#define IMX_ANATOP_PLL_TEST_TOG                   0x2C
#define IMX_ANATOP_PLL_SPREAD_SPECTRUM            0x30     /* PLL Spread Spectrum Register */
#define IMX_ANATOP_PLL_SPREAD_SPECTRUM_SET        0x34
#define IMX_ANATOP_PLL_SPREAD_SPECTRUM_CLR        0x38
#define IMX_ANATOP_PLL_SPREAD_SPECTRUM_TOG        0x3C
#define IMX_ANATOP_PLL_NUMERATOR                  0x40     /* PLL Numerator Register */
#define IMX_ANATOP_PLL_NUMERATOR_SET              0x44
#define IMX_ANATOP_PLL_NUMERATOR_CLR              0x48
#define IMX_ANATOP_PLL_NUMERATOR_TOG              0x4C
#define IMX_ANATOP_PLL_DENOMINATOR                0x50     /* PLL Denominator Register */
#define IMX_ANATOP_PLL_DENOMINATOR_SET            0x54
#define IMX_ANATOP_PLL_DENOMINATOR_CLR            0x58
#define IMX_ANATOP_PLL_DENOMINATOR_TOG            0x5C
#define IMX_ANATOP_PLL_DIV                        0x60     /* PLL Divider Register */
#define IMX_ANATOP_PLL_DIV_SET                    0x64
#define IMX_ANATOP_PLL_DIV_CLR                    0x68
#define IMX_ANATOP_PLL_DIV_TOG                    0x6C
#define IMX_ANATOP_PLL_STATUS                     0xF0     /* PLL Status Register */

/*
 * Video PLL control register bits *
 */
#define IMX_ANATOP_CTRL_HW_CTRL_SEL_MASK        (0x01UL << 16) /* Hardware control select */
#define IMX_ANATOP_CTRL_HW_CTRL_SEL_SHIFT       16
#define IMX_ANATOP_CTRL_MULTIPHASE_SDM_DIS_MASK (0x01UL << 12) /* MULTIPHASE_SDM_DISABLE */
#define IMX_ANATOP_CTRL_MULTIPHASE_SDM_DIS_SHIFT 12
#define IMX_ANATOP_CTRL_DITHER_EN3_MASK         (0x01UL << 11) /* Dither Enable 3 */
#define IMX_ANATOP_CTRL_DITHER_EN3_SHIFT        11
#define IMX_ANATOP_CTRL_DITHER_EN2_MASK         (0x01UL << 10) /* Dither Enable 2 */
#define IMX_ANATOP_CTRL_DITHER_EN2_SHIFT        10
#define IMX_ANATOP_CTRL_DITHER_EN1_MASK         (0x01UL << 9)  /* Dither Enable 1 */
#define IMX_ANATOP_CTRL_DITHER_EN1_SHIFT        9
#define IMX_ANATOP_CTRL_SPREADCTL_MASK          (0x01UL << 8)  /* Modulation type selection */
#define IMX_ANATOP_CTRL_SPREADCTL_SHIFT         8
#define IMX_ANATOP_CTRL_CLKMUX_BYPASS_MASK      (0x01UL << 2)  /* Clock multiplex bypass */
#define IMX_ANATOP_CTRL_CLKMUX_BYPASS_SHIFT     2
#define IMX_ANATOP_CTRL_CLKMUX_EN_MASK          (0x01UL << 1)  /* Clock multiplex enable */
#define IMX_ANATOP_CTRL_CLKMUX_EN_SHIFT         1
#define IMX_ANATOP_CTRL_POWERUP_MASK            (0x01UL << 0)  /* Power up */
#define IMX_ANATOP_CTRL_POWERUP_SHIFT           0

/*
 * Video PLL analog program register bits *
 */
#define IMX_ANATOP_ANA_PRG_ANA_MISC_MASK        (0xFFUL << 24) /* Analog miscellaneous bits */
#define IMX_ANATOP_ANA_PRG_ANA_MISC_SHIFT       24
#define IMX_ANATOP_ANA_PRG_ANA_MISC(x)          (((x) << 24) & 0xFF000000UL)
#define IMX_ANATOP_ANA_PRG_IREF_CTRL_MASK       (0x03UL << 16) /* IREF_CTRL */
#define IMX_ANATOP_ANA_PRG_IREF_CTRL_SHIFT      16
#define IMX_ANATOP_ANA_PRG_IREF_CTRL(x)         (((x) << 16) & 0x00030000UL)
#define IMX_ANATOP_ANA_PRG_V2I_CTRL_MASK        (0x03UL << 14) /* V2I_CTRL */
#define IMX_ANATOP_ANA_PRG_V2I_CTRL_SHIFT       14
#define IMX_ANATOP_ANA_PRG_V2I_CTRL(x)          (((x) << 14) & 0x0000C000UL)
#define IMX_ANATOP_ANA_PRG_ICP_CTRL_MASK        (0x03UL << 12) /* ICP_CTRL */
#define IMX_ANATOP_ANA_PRG_ICP_CTRL_SHIFT       12
#define IMX_ANATOP_ANA_PRG_ICP_CTRL(x)          (((x) << 12) & 0x00003000UL)
#define IMX_ANATOP_ANA_PRG_V2I_CP_CTRL_MASK     (0x03UL << 10) /* V2I_CP_CTRL */
#define IMX_ANATOP_ANA_PRG_V2I_CP_CTRL_SHIFT    10
#define IMX_ANATOP_ANA_PRG_V2I_CP_CTRL(x)       (((x) << 10) & 0x00000C00UL)
#define IMX_ANATOP_ANA_PRG_IPTAT_CP_CTRL_MASK   (0x03UL << 8)  /* IPTAT_CP_CTRL */
#define IMX_ANATOP_ANA_PRG_IPTAT_CP_CTRL_SHIFT  8
#define IMX_ANATOP_ANA_PRG_IPTAT_CP_CTRL(x)     (((x) << 8) & 0x00000300UL)
#define IMX_ANATOP_ANA_PRG_DOUBLE_LF_MASK       (0x01UL << 1)  /* DOUBLE_LF */
#define IMX_ANATOP_ANA_PRG_DOUBLE_LF_SHIFT      1
#define IMX_ANATOP_ANA_PRG_HALF_LF_MASK         (0x01UL << 0)  /* HALF_LF */
#define IMX_ANATOP_ANA_PRG_HALF_LF_SHIFT        0

/*
 * Video PLL TEST register bits *
 */
#define IMX_ANATOP_TEST_STRESS_MODE_MASK        (0x01UL << 10)
#define IMX_ANATOP_TEST_STRESS_MODE_SHIFT       10
#define IMX_ANATOP_TEST_MODE_MASK               (0x01UL << 9)  /* Test mux enable */
#define IMX_ANATOP_TEST_MODE_SHIFT              9
#define IMX_ANATOP_TEST_OPEN_LOOP_MASK          (0x01UL << 8)  /* Open loop mode */
#define IMX_ANATOP_TEST_OPEN_LOOP_SHIFT         8
#define IMX_ANATOP_TEST_TEST_MUX_ENABLE_MASK    (0x01UL << 3)
#define IMX_ANATOP_TEST_TEST_MUX_ENABLE_SHIFT   3
#define IMX_ANATOP_TEST_ANAMUX_CTRL_MASK        (0x07UL << 0)  /* Analog test signal selection. Valid when TEST_MUX_ENABLE is 1'b1 */
#define IMX_ANATOP_TEST_ANAMUX_CTRL_SHIFT       0
#define IMX_ANATOP_TEST_ANAMUX_CTRL(x)          (((x) << 0) & 0x00000007UL)

/*
 * Video PLL Spread Spectrum register bits *
 */
#define IMX_ANATOP_SPREAD_SPECTRUM_STOP_MASK    (0xFFFFUL << 16) /* Stop */
#define IMX_ANATOP_SPREAD_SPECTRUM_STOP_SHIFT   16
#define IMX_ANATOP_SPREAD_SPECTRUM_STOP(x)      (((x) << 16) & 0xFFFF0000UL)
#define IMX_ANATOP_SPREAD_SPECTRUM_ENABLE_MASK  (0x01UL << 15)  /* Enable */
#define IMX_ANATOP_SPREAD_SPECTRUM_ENABLE_SHIFT 15
#define IMX_ANATOP_SPREAD_SPECTRUM_STEP_MASK    (0x7FFFUL << 0) /* Step */
#define IMX_ANATOP_SPREAD_SPECTRUM_STEP_SHIFT   0
#define IMX_ANATOP_SPREAD_SPECTRUM_STEP(x)      (((x) << 0) & 0x00007FFFUL)

/*
 * Video PLL Numerator register bits *
 */
#define IMX_ANATOP_NUMERATOR_MFN_MASK           (0x3FFFFFFFUL << 2) /* Numerator */
#define IMX_ANATOP_NUMERATOR_MFN_SHIFT          2
#define IMX_ANATOP_NUMERATOR_MFN(x)             (((x) << 2) & 0xFFFFFFFCUL)

/*
 * Video PLL Denominator register bits *
 */
#define IMX_ANATOP_DENOMINATOR_MFN_MASK         (0x3FFFFFFFUL << 0) /* Denominator */
#define IMX_ANATOP_DENOMINATOR_MFN_SHIFT        0
#define IMX_ANATOP_DENOMINATOR_MFN(x)           (((x) << 0) & 0x3FFFFFFFUL)

/*
 * Video PLL Dividers register bits *
 */
#define IMX_ANATOP_DIV_MFI_MASK                 (0x01FFUL << 16) /* Integer portion of loop divider */
#define IMX_ANATOP_DIV_MFI_SHIFT                16
#define IMX_ANATOP_DIV_MFI(x)                   (((x) << 16) & 0x01FF0000UL)
#define IMX_ANATOP_DIV_RDIV_MASK                (0x07UL << 13)   /* Input clock predivider */
#define IMX_ANATOP_DIV_RDIV_SHIFT               13
#define IMX_ANATOP_DIV_RDIV(x)                  (((x) << 13) & 0x0000E000UL)
#define IMX_ANATOP_DIV_ODIV_MASK                (0xFFUL << 0)    /* Output frequency divider for clock output */
#define IMX_ANATOP_DIV_ODIV_SHIFT               0
#define IMX_ANATOP_DIV_ODIV(x)                  (((x) << 0) & 0x000000FFUL)

/*
 * Video PLL Status register bits *
 */
#define IMX_ANATOP_STATUS_MFN_MASK              (0x3FFFFFFFUL << 2) /* ANA_MFN */
#define IMX_ANATOP_STATUS_MFN_SHIFT             2
#define IMX_ANATOP_STATUS_MFN(x)                (((x) << 2) & 0xFFFFFFFC)
#define IMX_ANATOP_STATUS_PLL_LOL_MASK          (0x01UL << 1)    /* PLL_LOL */
#define IMX_ANATOP_STATUS_PLL_LOL_SHIFT         1
#define IMX_ANATOP_STATUS_PLL_LOCK_MASK         (0x01UL << 0)    /* PLL_LOCK */
#define IMX_ANATOP_STATUS_PLL_LOCK_SHIFT        0

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

#endif
