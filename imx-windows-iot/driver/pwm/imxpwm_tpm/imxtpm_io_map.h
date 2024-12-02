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

#ifndef _TPM_IO_MAP_H_
#define _TPM_IO_MAP_H_


/*
 * TPM registers definition
 */

#define TPM_VERID                                      0x00U
#define TPM_PARAM                                      0x04U
#define TPM_GLOBAL                                     0x08U
#define TPM_SC                                         0x10U
#define TPM_CNT                                        0x14U
#define TPM_MOD                                        0x18U
#define TPM_STATUS                                     0x1CU
#define TPM_C0SC                                       0x20U
#define TPM_C0V                                        0x24U
#define TPM_C1SC                                       0x28U
#define TPM_C1V                                        0x2CU
#define TPM_C2SC                                       0x30U
#define TPM_C2V                                        0x34U
#define TPM_C3SC                                       0x38U
#define TPM_C3V                                        0x3CU
#define TPM_COMBINE                                    0x64U
#define TPM_TRIG                                       0x6CU
#define TPM_POL                                        0x70U
#define TPM_FILTER                                     0x78U
#define TPM_QDCTRL                                     0x80U
#define TPM_CONF                                       0x84U


/*
 * TPM_VERID register bits *
 */
#define TPM_VERID_MAJOR_MASK                           0xFF000000U
#define TPM_VERID_MAJOR_SHIFT                          24U
#define TPM_VERID_MINOR_MASK                           0x00FF0000U
#define TPM_VERID_MINOR_SHIFT                          16U
#define TPM_VERID_FEATURE_MASK                         0x0000FFFFU
#define TPM_VERID_FEATURE_SHIFT                        0U

/*
 * TPM_PARAM register bits *
 */
#define TPM_PARAM_WIDTH_MASK                           0x00FF0000U
#define TPM_PARAM_WIDTH_SHIFT                          16U
#define TPM_PARAM_TRIG_MASK                            0x0000FF00U
#define TPM_PARAM_TRIG_SHIFT                           8U
#define TPM_PARAM_CHAN_MASK                            0x000000FFU
#define TPM_PARAM_CHAN_SHIFT                           0U

/*
 * TPM_GLOBAL register bits *
 */
#define TPM_GLOBAL_RST_MASK                            0x00000002U
#define TPM_GLOBAL_RST_SHIFT                           1U
#define TPM_GLOBAL_NOUPDATE_MASK                       0x00000001U
#define TPM_GLOBAL_NOUPDATE_SHIFT                      0U

/*
 * TPM_SC register bits *
 */
#define TPM_SC_DMA_MASK                                0x00000100U
#define TPM_SC_DMA_SHIFT                               8U
#define TPM_SC_TOF_MASK                                0x00000080U
#define TPM_SC_TOF_SHIFT                               7U
#define TPM_SC_TOIE_MASK                               0x00000040U
#define TPM_SC_TOIE_SHIFT                              6U
#define TPM_SC_CPWMS_MASK                              0x00000020U
#define TPM_SC_CPWMS_SHIFT                             5U
#define TPM_SC_CMOD_MASK                               0x00000018U
#define TPM_SC_CMOD_SHIFT                              3U
#define TPM_SC_PS_MASK                                 0x00000007U
#define TPM_SC_PS_SHIFT                                0U

/*
 * TPM_STATUS register bits *
 */
#define TPM_STATUS_TOF_MASK                            0x00000100U
#define TPM_STATUS_TOF_SHIFT                           8U
#define TPM_STATUS_CH3_MASK                            0x00000008U
#define TPM_STATUS_CH3_SHIFT                           3U
#define TPM_STATUS_CH2_MASK                            0x00000004U
#define TPM_STATUS_CH2_SHIFT                           2U
#define TPM_STATUS_CH1_MASK                            0x00000002U
#define TPM_STATUS_CH1_SHIFT                           1U
#define TPM_STATUS_CH0_MASK                            0x00000001U
#define TPM_STATUS_CH0_SHIFT                           0U

/*
 * TPM_C0SC register bits *
 */
#define TPM_C0SC_CHF_MASK                              0x00000080U
#define TPM_C0SC_CHF_SHIFT                             7U
#define TPM_C0SC_CHIE_MASK                             0x00000040U
#define TPM_C0SC_CHIE_SHIFT                            6U
#define TPM_C0SC_MSB_MASK                              0x00000020U
#define TPM_C0SC_MSB_SHIFT                             5U
#define TPM_C0SC_MSA_MASK                              0x00000010U
#define TPM_C0SC_MSA_SHIFT                             4U
#define TPM_C0SC_ELSB_MASK                             0x00000008U
#define TPM_C0SC_ELSB_SHIFT                            3U
#define TPM_C0SC_ELSA_MASK                             0x00000004U
#define TPM_C0SC_ELSA_SHIFT                            2U
#define TPM_C0SC_DMA_MASK                              0x00000001U
#define TPM_C0SC_DMA_SHIFT                             0U

/*
 * TPM_C1SC register bits *
 */
#define TPM_C1SC_CHF_MASK                              0x00000080U
#define TPM_C1SC_CHF_SHIFT                             7U
#define TPM_C1SC_CHIE_MASK                             0x00000040U
#define TPM_C1SC_CHIE_SHIFT                            6U
#define TPM_C1SC_MSB_MASK                              0x00000020U
#define TPM_C1SC_MSB_SHIFT                             5U
#define TPM_C1SC_MSA_MASK                              0x00000010U
#define TPM_C1SC_MSA_SHIFT                             4U
#define TPM_C1SC_ELSB_MASK                             0x00000008U
#define TPM_C1SC_ELSB_SHIFT                            3U
#define TPM_C1SC_ELSA_MASK                             0x00000004U
#define TPM_C1SC_ELSA_SHIFT                            2U
#define TPM_C1SC_DMA_MASK                              0x00000001U
#define TPM_C1SC_DMA_SHIFT                             0U

/*
 * TPM_C2SC register bits *
 */
#define TPM_C2SC_CHF_MASK                              0x00000080U
#define TPM_C2SC_CHF_SHIFT                             7U
#define TPM_C2SC_CHIE_MASK                             0x00000040U
#define TPM_C2SC_CHIE_SHIFT                            6U
#define TPM_C2SC_MSB_MASK                              0x00000020U
#define TPM_C2SC_MSB_SHIFT                             5U
#define TPM_C2SC_MSA_MASK                              0x00000010U
#define TPM_C2SC_MSA_SHIFT                             4U
#define TPM_C2SC_ELSB_MASK                             0x00000008U
#define TPM_C2SC_ELSB_SHIFT                            3U
#define TPM_C2SC_ELSA_MASK                             0x00000004U
#define TPM_C2SC_ELSA_SHIFT                            2U
#define TPM_C2SC_DMA_MASK                              0x00000001U
#define TPM_C2SC_DMA_SHIFT                             0U

/*
 * TPM_C3SC register bits *
 */
#define TPM_C3SC_CHF_MASK                              0x00000080U
#define TPM_C3SC_CHF_SHIFT                             7U
#define TPM_C3SC_CHIE_MASK                             0x00000040U
#define TPM_C3SC_CHIE_SHIFT                            6U
#define TPM_C3SC_MSB_MASK                              0x00000020U
#define TPM_C3SC_MSB_SHIFT                             5U
#define TPM_C3SC_MSA_MASK                              0x00000010U
#define TPM_C3SC_MSA_SHIFT                             4U
#define TPM_C3SC_ELSB_MASK                             0x00000008U
#define TPM_C3SC_ELSB_SHIFT                            3U
#define TPM_C3SC_ELSA_MASK                             0x00000004U
#define TPM_C3SC_ELSA_SHIFT                            2U
#define TPM_C3SC_DMA_MASK                              0x00000001U
#define TPM_C3SC_DMA_SHIFT                             0U

/*
 * TPM_COMBINE register bits *
 */
#define TPM_COMBINE_COMSWAP1_MASK                      0x00000200U
#define TPM_COMBINE_COMSWAP1_SHIFT                     9U
#define TPM_COMBINE_COMBINE1_MASK                      0x00000100U
#define TPM_COMBINE_COMBINE1_SHIFT                     8U
#define TPM_COMBINE_COMSWAP0_MASK                      0x00000002U
#define TPM_COMBINE_COMSWAP0_SHIFT                     1U
#define TPM_COMBINE_COMBINE0_MASK                      0x00000001U
#define TPM_COMBINE_COMBINE0_SHIFT                     0U

/*
 * TPM_TRIG register bits *
 */
#define TPM_TRIG_TRIG3_MASK                            0x00000008U
#define TPM_TRIG_TRIG3_SHIFT                           3U
#define TPM_TRIG_TRIG2_MASK                            0x00000004U
#define TPM_TRIG_TRIG2_SHIFT                           2U
#define TPM_TRIG_TRIG1_MASK                            0x00000002U
#define TPM_TRIG_TRIG1_SHIFT                           1U
#define TPM_TRIG_TRIG0_MASK                            0x00000001U
#define TPM_TRIG_TRIG0_SHIFT                           0U

/*
 * TPM_POL register bits *
 */
#define TPM_POL_POL3_MASK                              0x00000008U
#define TPM_POL_POL3_SHIFT                             3U
#define TPM_POL_POL2_MASK                              0x00000004U
#define TPM_POL_POL2_SHIFT                             2U
#define TPM_POL_POL1_MASK                              0x00000002U
#define TPM_POL_POL1_SHIFT                             1U
#define TPM_POL_POL0_MASK                              0x00000001U
#define TPM_POL_POL0_SHIFT                             0U

/*
 * TPM_FILTER register bits *
 */
#define TPM_FILTER_CH3FVAL_MASK                        0x0000F000U
#define TPM_FILTER_CH3FVAL_SHIFT                       12U
#define TPM_FILTER_CH2FVAL_MASK                        0x00000F00U
#define TPM_FILTER_CH2FVAL_SHIFT                       8U
#define TPM_FILTER_CH1FVAL_MASK                        0x000000F0U
#define TPM_FILTER_CH1FVAL_SHIFT                       4U
#define TPM_FILTER_CH0FVAL_MASK                        0x0000000FU
#define TPM_FILTER_CH0FVAL_SHIFT                       0U

/*
 * TPM_QDCTRL register bits *
 */
#define TPM_QDCTRL_QUADMODE_MASK                       0x00000008U
#define TPM_QDCTRL_QUADMODE_SHIFT                      3U
#define TPM_QDCTRL_QUADIR_MASK                         0x00000004U
#define TPM_QDCTRL_QUADIR_SHIFT                        2U
#define TPM_QDCTRL_TOFDIR_MASK                         0x00000002U
#define TPM_QDCTRL_TOFDIR_SHIFT                        1U
#define TPM_QDCTRL_QUADEN_MASK                         0x00000001U
#define TPM_QDCTRL_QUADEN_SHIFT                        0U

/*
 * TPM_CONF register bits *
 */
#define TPM_CONF_TRGSEL_MASK                           0x03000000U
#define TPM_CONF_TRGSEL_SHIFT                          24U
#define TPM_CONF_TRGSRC_MASK                           0x00800000U
#define TPM_CONF_TRGSRC_SHIFT                          23U
#define TPM_CONF_TRGPOL_MASK                           0x00400000U
#define TPM_CONF_TRGPOL_SHIFT                          22U
#define TPM_CONF_CPOT_MASK                             0x00080000U
#define TPM_CONF_CPOT_SHIFT                            19U
#define TPM_CONF_CROT_MASK                             0x00040000U
#define TPM_CONF_CROT_SHIFT                            18U
#define TPM_CONF_CSOO_MASK                             0x00020000U
#define TPM_CONF_CSOO_SHIFT                            17U
#define TPM_CONF_CSOT_MASK                             0x00010000U
#define TPM_CONF_CSOT_SHIFT                            16U
#define TPM_CONF_GTBEEN_MASK                           0x00000200U
#define TPM_CONF_GTBEEN_SHIFT                          9U
#define TPM_CONF_GTBSYNC_MASK                          0x00000100U
#define TPM_CONF_GTBSYNC_SHIFT                         8U
#define TPM_CONF_DBGMODE_MASK                          0x000000C0U
#define TPM_CONF_DBGMODE_SHIFT                         6U
#define TPM_CONF_DOZEEN_MASK                           0x00000020U
#define TPM_CONF_DOZEEN_SHIFT                          5U


/*
 * TPM_VERID register bits definition *
 */
typedef union TPM_VERID_union_t {
    UINT32 R;
    struct {
        UINT32 FEATURE  : 16;  /* Feature Specification Number. */
        UINT32 MINOR    :  8;  /* Minor Version Number. */
        UINT32 MAJOR    :  8;  /* Major Version Number. */
    } B;
} TPM_VERID_t;

/*
 * TPM_PARAM register bits definition *
 */
typedef union TPM_PARAM_union_t {
    UINT32 R;
    struct {
        UINT32 CHAN         :  8;  /* Number of timer channels implemented. */
        UINT32 TRIG         :  8;  /* Number of trigger inputs implemented. */
        UINT32 WIDTH        :  8;  /* Width of the counter and timer channels. */
        UINT32 Reserved_24  :  8;  /* Reserved */
    } B;
} TPM_PARAM_t;

/*
 * TPM_GLOBAL register bits definition *
 */
typedef union TPM_GLOBAL_union_t {
    UINT32 R;
    struct {
        UINT32 NOUPDATE    :  1;  /* Blocks updates to all internal double buffered registers while it remains set. */
        UINT32 RST         :  1;  /* Reset all internal logic and registers, except the Global Register. Remains set until cleared by software. */
        UINT32 Reserved_2  : 30;  /* Reserved */
    } B;
} TPM_GLOBAL_t;

/*
 * TPM_SC register bits definition *
 */
typedef union TPM_SC_union_t {
    UINT32 R;
    struct {
        UINT32 PS          :  3;  /* Prescale Factor Selection. Selects one of 8 division factors for the clock mode selected by CMOD. */
        UINT32 CMOD        :  2;  /* Clock Mode Selection. Selects the TPM counter clock modes. */
        UINT32 CPWMS       :  1;  /* Center-Aligned PWM Select. Configures the TPM to operate in up-down counting mode. */
        UINT32 TOIE        :  1;  /* Timer Overflow Interrupt Enable. Enables TPM overflow interrupts. */
        UINT32 TOF         :  1;  /* Timer Overflow Flag. Set by hardware when the TPM counter equals the value in the MOD register and increments. */
        UINT32 DMA         :  1;  /* Enables DMA transfers for the overflow flag. */
        UINT32 Reserved_9  : 23;  /* Reserved */
    } B;
} TPM_SC_t;

/*
 * TPM_CNT register bits definition *
 */
typedef union TPM_CNT_union_t {
    UINT32 R;
    struct {
        UINT32 COUNT  : 32;  /* Counter value. */
    } B;
} TPM_CNT_t;

/*
 * TPM_MOD register bits definition *
 */
typedef union TPM_MOD_union_t {
    UINT32 R;
    struct {
        UINT32 MOD  : 32;  /* Modulo value. This field must be written with single 16-bit or 32-bit access. */
    } B;
} TPM_MOD_t;

/*
 * TPM_STATUS register bits definition *
 */
typedef union TPM_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 CH0         :  1;  /* Channel 0 Flag */
        UINT32 CH1         :  1;  /* Channel 1 Flag */
        UINT32 CH2         :  1;  /* Channel 2 Flag */
        UINT32 CH3         :  1;  /* Channel 3 Flag */
        UINT32 Reserved_4  :  4;  /* Reserved */
        UINT32 TOF         :  1;  /* Timer Overflow Flag. */
        UINT32 Reserved_9  : 23;  /* Reserved */
    } B;
} TPM_STATUS_t;

/*
 * TPM_C0SC register bits definition *
 */
typedef union TPM_C0SC_union_t {
    UINT32 R;
    struct {
        UINT32 DMA         :  1;  /* DMA Enable. Enables DMA transfers for the channel. */
        UINT32 Reserved_1  :  1;  /* Reserved */
        UINT32 ELSA        :  1;  /* Edge or Level Select. */
        UINT32 ELSB        :  1;  /* Edge or Level Select. */
        UINT32 MSA         :  1;  /* Channel Mode Select. */
        UINT32 MSB         :  1;  /* Channel Mode Select. */
        UINT32 CHIE        :  1;  /* Channel Interrupt Enable. Enables channel interrupts. */
        UINT32 CHF         :  1;  /* Channel Flag. */
        UINT32 Reserved_8  : 24;  /* Reserved */
    } B;
} TPM_C0SC_t;

/*
 * TPM_C0V register bits definition *
 */
typedef union TPM_C0V_union_t {
    UINT32 R;
    struct {
        UINT32 VAL  : 32;  /* Channel Value. This field must be written with single 16-bit or 32-bit access. */
    } B;
} TPM_C0V_t;

/*
 * TPM_C1SC register bits definition *
 */
typedef union TPM_C1SC_union_t {
    UINT32 R;
    struct {
        UINT32 DMA         :  1;  /* DMA Enable. Enables DMA transfers for the channel. */
        UINT32 Reserved_1  :  1;  /* Reserved */
        UINT32 ELSA        :  1;  /* Edge or Level Select. */
        UINT32 ELSB        :  1;  /* Edge or Level Select. */
        UINT32 MSA         :  1;  /* Channel Mode Select. */
        UINT32 MSB         :  1;  /* Channel Mode Select. */
        UINT32 CHIE        :  1;  /* Channel Interrupt Enable. Enables channel interrupts. */
        UINT32 CHF         :  1;  /* Channel Flag. */
        UINT32 Reserved_8  : 24;  /* Reserved */
    } B;
} TPM_C1SC_t;

/*
 * TPM_C1V register bits definition *
 */
typedef union TPM_C1V_union_t {
    UINT32 R;
    struct {
        UINT32 VAL  : 32;  /* Channel Value. This field must be written with single 16-bit or 32-bit access. */
    } B;
} TPM_C1V_t;

/*
 * TPM_C2SC register bits definition *
 */
typedef union TPM_C2SC_union_t {
    UINT32 R;
    struct {
        UINT32 DMA         :  1;  /* DMA Enable. Enables DMA transfers for the channel. */
        UINT32 Reserved_1  :  1;  /* Reserved */
        UINT32 ELSA        :  1;  /* Edge or Level Select. */
        UINT32 ELSB        :  1;  /* Edge or Level Select. */
        UINT32 MSA         :  1;  /* Channel Mode Select. */
        UINT32 MSB         :  1;  /* Channel Mode Select. */
        UINT32 CHIE        :  1;  /* Channel Interrupt Enable. Enables channel interrupts. */
        UINT32 CHF         :  1;  /* Channel Flag. */
        UINT32 Reserved_8  : 24;  /* Reserved */
    } B;
} TPM_C2SC_t;

/*
 * TPM_C2V register bits definition *
 */
typedef union TPM_C2V_union_t {
    UINT32 R;
    struct {
        UINT32 VAL  : 32;  /* Channel Value. This field must be written with single 16-bit or 32-bit access. */
    } B;
} TPM_C2V_t;

/*
 * TPM_C3SC register bits definition *
 */
typedef union TPM_C3SC_union_t {
    UINT32 R;
    struct {
        UINT32 DMA         :  1;  /* DMA Enable. Enables DMA transfers for the channel. */
        UINT32 Reserved_1  :  1;  /* Reserved */
        UINT32 ELSA        :  1;  /* Edge or Level Select. */
        UINT32 ELSB        :  1;  /* Edge or Level Select. */
        UINT32 MSA         :  1;  /* Channel Mode Select. */
        UINT32 MSB         :  1;  /* Channel Mode Select. */
        UINT32 CHIE        :  1;  /* Channel Interrupt Enable. Enables channel interrupts. */
        UINT32 CHF         :  1;  /* Channel Flag. */
        UINT32 Reserved_8  : 24;  /* Reserved */
    } B;
} TPM_C3SC_t;

/*
 * TPM_C3V register bits definition *
 */
typedef union TPM_C3V_union_t {
    UINT32 R;
    struct {
        UINT32 VAL  : 32;  /* Channel Value. This field must be written with single 16-bit or 32-bit access. */
    } B;
} TPM_C3V_t;

/*
 * TPM_COMBINE register bits definition *
 */
typedef union TPM_COMBINE_union_t {
    UINT32 R;
    struct {
        UINT32 COMBINE0     :  1;  /* Combine Channels 0 and 1 */
        UINT32 COMSWAP0     :  1;  /* Combine Channels 0 and 1 Swap. */
        UINT32 Reserved_2   :  6;  /* Reserved */
        UINT32 COMBINE1     :  1;  /* Combine Channels 2 and 3 */
        UINT32 COMSWAP1     :  1;  /* Combine Channels 2 and 3 Swap. */
        UINT32 Reserved_10  : 22;  /* Reserved */
    } B;
} TPM_COMBINE_t;

/*
 * TPM_TRIG register bits definition *
 */
typedef union TPM_TRIG_union_t {
    UINT32 R;
    struct {
        UINT32 TRIG0       :  1;  /* Channel 0 Trigger */
        UINT32 TRIG1       :  1;  /* Channel 1 Trigger */
        UINT32 TRIG2       :  1;  /* Channel 2 Trigger */
        UINT32 TRIG3       :  1;  /* Channel 3 Trigger */
        UINT32 Reserved_4  : 28;  /* Reserved */
    } B;
} TPM_TRIG_t;

/*
 * TPM_POL register bits definition *
 */
typedef union TPM_POL_union_t {
    UINT32 R;
    struct {
        UINT32 POL0        :  1;  /* Channel 0 Polarity */
        UINT32 POL1        :  1;  /* Channel 1 Polarity */
        UINT32 POL2        :  1;  /* Channel 2 Polarity */
        UINT32 POL3        :  1;  /* Channel 3 Polarity */
        UINT32 Reserved_4  : 28;  /* Reserved */
    } B;
} TPM_POL_t;

/*
 * TPM_FILTER register bits definition *
 */
typedef union TPM_FILTER_union_t {
    UINT32 R;
    struct {
        UINT32 CH0FVAL      :  4;  /* Channel 0 Filter Value */
        UINT32 CH1FVAL      :  4;  /* Channel 1 Filter Value */
        UINT32 CH2FVAL      :  4;  /* Channel 2 Filter Value */
        UINT32 CH3FVAL      :  4;  /* Channel 3 Filter Value */
        UINT32 Reserved_16  : 16;  /* Reserved */
    } B;
} TPM_FILTER_t;

/*
 * TPM_QDCTRL register bits definition *
 */
typedef union TPM_QDCTRL_union_t {
    UINT32 R;
    struct {
        UINT32 QUADEN      :  1;  /* QUADENEnables the quadrature decoder mode. */
        UINT32 TOFDIR      :  1;  /* TOFDIR. Indicates if the TOF bit was set on the top or the bottom of counting. */
        UINT32 QUADIR      :  1;  /* Counter Direction in Quadrature Decode Mode. Indicates the counting direction. */
        UINT32 QUADMODE    :  1;  /* Quadrature Decoder. ModeSelects the encoding mode used in the quadrature decoder mode. */
        UINT32 Reserved_4  : 28;  /* Reserved */
    } B;
} TPM_QDCTRL_t;

/*
 * TPM_CONF register bits definition *
 */
typedef union TPM_CONF_union_t {
    UINT32 R;
    struct {
        UINT32 Reserved_4   :  5;  /* Reserved */
        UINT32 DOZEEN       :  1;  /* Doze Enable. Configures the TPM behavior in a low-power mode.. */
        UINT32 DBGMODE      :  2;  /* Debug Mode. Configures the TPM behavior in Debug mode. */
        UINT32 GTBSYNC      :  1;  /* Global Time Base Synchronization. */
        UINT32 GTBEEN       :  1;  /* Global timebase enable. */
        UINT32 Reserved_10  :  6;  /* Reserved */
        UINT32 CSOT         :  1;  /* Counter Start on Trigger. */
        UINT32 CSOO         :  1;  /* Counter Stop On Overflow. */
        UINT32 CROT         :  1;  /* Counter Reload On Trigger. */
        UINT32 CPOT         :  1;  /* Counter Pause On Trigger. */
        UINT32 Reserved_20  :  2;  /* Reserved */
        UINT32 TRGPOL       :  1;  /* Trigger Polarity. Selects the polarity of the external trigger source. */
        UINT32 TRGSRC       :  1;  /* Trigger Source. Selects between internal (channel pin input capture) or external trigger sources. */
        UINT32 TRGSEL       :  2;  /* Trigger Select. Selects the input trigger to use for starting, reloading and/or pausing the counter. */
        UINT32 Reserved_26  :  6;  /* Reserved */
    } B;
} TPM_CONF_t;

/*
 * TPM structure definition
 */
typedef struct TPM_s {
    TPM_VERID_t          VERID;                                    /* 0x00000000 Version ID Register */
    TPM_PARAM_t          PARAM;                                    /* 0x00000004 Parameter Register */
    TPM_GLOBAL_t         GLOBAL;                                   /* 0x00000008 TPM Global Register */
    UINT8                Reserved_0x0C[4];                         /* 0x0000000C Reserved */
    TPM_SC_t             SC;                                       /* 0x00000010 Status and Control Register */
    TPM_CNT_t            CNT;                                      /* 0x00000014 Counter Register */
    TPM_MOD_t            MOD;                                      /* 0x00000018 Modulo Register */
    TPM_STATUS_t         STATUS;                                   /* 0x0000001C Capture and Compare Status Register */
    TPM_C0SC_t           C0SC;                                     /* 0x00000020 Channel 0 Status and Control Register */
    TPM_C0V_t            C0V;                                      /* 0x00000024 Channel 0 Value Register */
    TPM_C1SC_t           C1SC;                                     /* 0x00000028 Channel 1 Status and Control Register */
    TPM_C1V_t            C1V;                                      /* 0x0000002C Channel 1 Value Register */
    TPM_C2SC_t           C2SC;                                     /* 0x00000030 Channel 2 Status and Control Register */
    TPM_C2V_t            C2V;                                      /* 0x00000034 Channel 2 Value Register */
    TPM_C3SC_t           C3SC;                                     /* 0x00000038 Channel 3 Status and Control Register */
    TPM_C3V_t            C3V;                                      /* 0x0000003C Channel 3 Value Register */
    UINT8                Reserved_0x40[36];                        /* 0x00000040 Reserved */
    TPM_COMBINE_t        COMBINE;                                  /* 0x00000064 Capture and Compare Status Register */
    UINT8                Reserved_0x68[4];                         /* 0x00000068 Reserved */
    TPM_TRIG_t           TRIG;                                     /* 0x0000006C Channel Trigger Register */
    TPM_POL_t            POL;                                      /* 0x00000070 Channel Polarity Register */
    UINT8                Reserved_0x74[4];                         /* 0x00000074 Reserved */
    TPM_FILTER_t         FILTER;                                   /* 0x00000078 Filter Control Register */
    UINT8                Reserved_0x7C[4];                         /* 0x0000007C Reserved */
    TPM_QDCTRL_t         QDCTRL;                                   /* 0x00000080 Quadrature Decoder Control and Status Register */
    TPM_CONF_t           CONF;                                     /* 0x00000084 Configuration Register */
} volatile TPM_t;

static_assert(sizeof(TPM_t) == 0x88, "sizeof(TPM_t) != 0x88");

#endif /* _TPM_IO_MAP_H_ */