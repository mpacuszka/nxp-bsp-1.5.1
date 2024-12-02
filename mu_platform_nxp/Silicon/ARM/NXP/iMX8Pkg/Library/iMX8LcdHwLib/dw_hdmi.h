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
*
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

#ifndef _DW_HDMI_H
#define _DW_HDMI_H

#include <iMXDisplay.h>

#define HDMI_REG_TX_BASE           0x32FD8000u
#define I2C_CLK_HIGH           0x6c
#define I2C_CLK_LOW            0x7f

#define EDID_FLAGS_HDMI        0x1
#define EDID_FLAGS_VSYNC_HIGH  0x2
#define EDID_FLAGS_HSYNC_HIGH  0x4

#define EDID_DIGITAL_SYNC_MASK      0x10
#define EDID_VERTICAL_SYNC_MASK     0x04
#define EDID_HORIZONTAL_SYNC_MASK   0x02

#define FMT_RGB888             1
#define FMT_RGB101010          2
#define FMT_RGB121212          3
#define FMT_RGB161616          4

#define EDID_SIZE            128

#define HDMI_REG_DESIGN_ID                          0x0000
#define HDMI_REG_REVISION_ID                        0x0001
#define HDMI_REG_PRODUCT_ID0                        0x0002
#define HDMI_REG_PRODUCT_ID1                        0x0003
#define HDMI_REG_CONFIG0_ID                         0x0004
#define HDMI_REG_CONFIG1_ID                         0x0005
#define HDMI_REG_CONFIG2_ID                         0x0006
#define HDMI_REG_CONFIG3_ID                         0x0007

#define HDMI_REG_IH_FC_STAT0                        0x0100
#define HDMI_REG_IH_FC_STAT1                        0x0101
#define HDMI_REG_IH_FC_STAT2                        0x0102
#define HDMI_REG_IH_AS_STAT0                        0x0103
#define HDMI_REG_IH_PHY_STAT0                       0x0104
#define HDMI_REG_IH_I2CM_STAT0                      0x0105
#define HDMI_REG_IH_CEC_STAT0                       0x0106
#define HDMI_REG_IH_VP_STAT0                        0x0107
#define HDMI_REG_IH_I2CMPHY_STAT0                   0x0108
#define HDMI_REG_IH_AHBDMAAUD_STAT0                 0x0109

#define HDMI_REG_IH_MUTE_FC_STAT0                   0x0180
#define HDMI_REG_IH_MUTE_FC_STAT1                   0x0181
#define HDMI_REG_IH_MUTE_FC_STAT2                   0x0182
#define HDMI_REG_IH_MUTE_AS_STAT0                   0x0183
#define HDMI_REG_IH_MUTE_PHY_STAT0                  0x0184
#define HDMI_REG_IH_MUTE_I2CM_STAT0                 0x0185
#define HDMI_REG_IH_MUTE_CEC_STAT0                  0x0186
#define HDMI_REG_IH_MUTE_VP_STAT0                   0x0187
#define HDMI_REG_IH_MUTE_I2CMPHY_STAT0              0x0188
#define HDMI_REG_IH_MUTE_AHBDMAAUD_STAT0            0x0189
#define HDMI_REG_IH_MUTE                            0x01FF

#define HDMI_REG_TX_INVID0                          0x0200
#define HDMI_REG_TX_INSTUFFING                      0x0201
#define HDMI_REG_TX_GYDATA0                         0x0202
#define HDMI_REG_TX_GYDATA1                         0x0203
#define HDMI_REG_TX_RCRDATA0                        0x0204
#define HDMI_REG_TX_RCRDATA1                        0x0205
#define HDMI_REG_TX_BCBDATA0                        0x0206
#define HDMI_REG_TX_BCBDATA1                        0x0207

#define HDMI_REG_VP_STATUS                          0x0800
#define HDMI_REG_VP_PR_CD                           0x0801
#define HDMI_REG_VP_STUFF                           0x0802
#define HDMI_REG_VP_REMAP                           0x0803
#define HDMI_REG_VP_CONF                            0x0804
#define HDMI_REG_VP_STAT                            0x0805
#define HDMI_REG_VP_INT                             0x0806
#define HDMI_REG_VP_MASK                            0x0807
#define HDMI_REG_VP_POL                             0x0808

#define HDMI_REG_FC_INVIDCONF                       0x1000
#define HDMI_REG_FC_INHACTV0                        0x1001
#define HDMI_REG_FC_INHACTV1                        0x1002
#define HDMI_REG_FC_INHBLANK0                       0x1003
#define HDMI_REG_FC_INHBLANK1                       0x1004
#define HDMI_REG_FC_INVACTV0                        0x1005
#define HDMI_REG_FC_INVACTV1                        0x1006
#define HDMI_REG_FC_INVBLANK                        0x1007
#define HDMI_REG_FC_HSYNCINDELAY0                   0x1008
#define HDMI_REG_FC_HSYNCINDELAY1                   0x1009
#define HDMI_REG_FC_HSYNCINWIDTH0                   0x100A
#define HDMI_REG_FC_HSYNCINWIDTH1                   0x100B
#define HDMI_REG_FC_VSYNCINDELAY                    0x100C
#define HDMI_REG_FC_VSYNCINWIDTH                    0x100D
#define HDMI_REG_FC_INFREQ0                         0x100E
#define HDMI_REG_FC_INFREQ1                         0x100F
#define HDMI_REG_FC_INFREQ2                         0x1010
#define HDMI_REG_FC_CTRLDUR                         0x1011
#define HDMI_REG_FC_EXCTRLDUR                       0x1012
#define HDMI_REG_FC_EXCTRLSPAC                      0x1013
#define HDMI_REG_FC_CH0PREAM                        0x1014
#define HDMI_REG_FC_CH1PREAM                        0x1015
#define HDMI_REG_FC_CH2PREAM                        0x1016
#define HDMI_REG_FC_AVICONF3                        0x1017
#define HDMI_REG_FC_GCP                             0x1018
#define HDMI_REG_FC_AVICONF0                        0x1019
#define HDMI_REG_FC_AVICONF1                        0x101A
#define HDMI_REG_FC_AVICONF2                        0x101B
#define HDMI_REG_FC_AVIVID                          0x101C
#define HDMI_REG_FC_AVIETB0                         0x101D
#define HDMI_REG_FC_AVIETB1                         0x101E
#define HDMI_REG_FC_AVISBB0                         0x101F
#define HDMI_REG_FC_AVISBB1                         0x1020
#define HDMI_REG_FC_AVIELB0                         0x1021
#define HDMI_REG_FC_AVIELB1                         0x1022
#define HDMI_REG_FC_AVISRB0                         0x1023
#define HDMI_REG_FC_AVISRB1                         0x1024
#define HDMI_REG_FC_AUDICONF0                       0x1025
#define HDMI_REG_FC_AUDICONF1                       0x1026
#define HDMI_REG_FC_AUDICONF2                       0x1027
#define HDMI_REG_FC_AUDICONF3                       0x1028
#define HDMI_REG_FC_VSDIEEEID0                      0x1029
#define HDMI_REG_FC_VSDSIZE                         0x102A
#define HDMI_REG_FC_AUDSCONF                        0x1063
#define HDMI_REG_FC_DATAUTO3                        0x10B7
#define HDMI_REG_FC_MASK0                           0x10D2
#define HDMI_REG_FC_MASK1                           0x10D6
#define HDMI_REG_FC_MASK2                           0x10DA
#define HDMI_REG_FC_PRCONF                          0x10E0

#define HDMI_REG_PHY_CONF0                          0x3000
#define HDMI_REG_PHY_TST0                           0x3001
#define HDMI_REG_PHY_TST1                           0x3002
#define HDMI_REG_PHY_TST2                           0x3003
#define HDMI_REG_PHY_STAT0                          0x3004
#define HDMI_REG_PHY_INT0                           0x3005
#define HDMI_REG_PHY_MASK0                          0x3006
#define HDMI_REG_PHY_POL0                           0x3007

#define HDMI_REG_PHY_I2CM_SLAVE_ADDR                0x3020
#define HDMI_REG_PHY_I2CM_ADDRESS_ADDR              0x3021
#define HDMI_REG_PHY_I2CM_DATAO_1_ADDR              0x3022
#define HDMI_REG_PHY_I2CM_DATAO_0_ADDR              0x3023
#define HDMI_REG_PHY_I2CM_DATAI_1_ADDR              0x3024
#define HDMI_REG_PHY_I2CM_DATAI_0_ADDR              0x3025
#define HDMI_REG_PHY_I2CM_OPERATION_ADDR            0x3026
#define HDMI_REG_PHY_I2CM_INT_ADDR                  0x3027
#define HDMI_REG_PHY_I2CM_CTLINT_ADDR               0x3028
#define HDMI_REG_PHY_I2CM_DIV_ADDR                  0x3029
#define HDMI_REG_PHY_I2CM_SOFTRSTZ_ADDR             0x302a
#define HDMI_REG_PHY_I2CM_SS_SCL_HCNT_1_ADDR        0x302b
#define HDMI_REG_PHY_I2CM_SS_SCL_HCNT_0_ADDR        0x302c
#define HDMI_REG_PHY_I2CM_SS_SCL_LCNT_1_ADDR        0x302d
#define HDMI_REG_PHY_I2CM_SS_SCL_LCNT_0_ADDR        0x302e
#define HDMI_REG_PHY_I2CM_FS_SCL_HCNT_1_ADDR        0x302f
#define HDMI_REG_PHY_I2CM_FS_SCL_HCNT_0_ADDR        0x3030
#define HDMI_REG_PHY_I2CM_FS_SCL_LCNT_1_ADDR        0x3031
#define HDMI_REG_PHY_I2CM_FS_SCL_LCNT_0_ADDR        0x3032

#define HDMI_REG_AUD_CONF0                          0x3100
#define HDMI_REG_AUD_CONF1                          0x3101
#define HDMI_REG_AUD_INT                            0x3102
#define HDMI_REG_AUD_CONF2                          0x3103
#define HDMI_REG_AUD_INT1                           0x3104
#define HDMI_REG_AUD_N1                             0x3200
#define HDMI_REG_AUD_N2                             0x3201
#define HDMI_REG_AUD_N3                             0x3202
#define HDMI_REG_AUD_CTS1                           0x3203
#define HDMI_REG_AUD_CTS2                           0x3204
#define HDMI_REG_AUD_CTS3                           0x3205
#define HDMI_REG_AUD_INPUTCLKFS                     0x3206
#define HDMI_REG_AUD_SPDIFINT			0x3302
#define HDMI_REG_AUD_CONF0_HBR                      0x3400
#define HDMI_REG_AUD_HBR_STATUS                     0x3401
#define HDMI_REG_AUD_HBR_INT                        0x3402
#define HDMI_REG_AUD_HBR_POL                        0x3403
#define HDMI_REG_AUD_HBR_MASK                       0x3404

#define HDMI_REG_GP_MASK                            0x3505

#define HDMI_REG_MC_SFRDIV                          0x4000
#define HDMI_REG_MC_CLKDIS                          0x4001
#define HDMI_REG_MC_SWRSTZ                          0x4002
#define HDMI_REG_MC_OPCTRL                          0x4003
#define HDMI_REG_MC_FLOWCTRL                        0x4004
#define HDMI_REG_MC_PHYRSTZ                         0x4005
#define HDMI_REG_MC_LOCKONCLOCK                     0x4006
#define HDMI_REG_MC_HEACPHY_RST                     0x4007

#define HDMI_REG_CSC_CFG                            0x4100
#define HDMI_REG_CSC_SCALE                          0x4101
#define HDMI_REG_CSC_COEF_A1_MSB                    0x4102
#define HDMI_REG_CSC_COEF_A1_LSB                    0x4103
#define HDMI_REG_CSC_COEF_A2_MSB                    0x4104
#define HDMI_REG_CSC_COEF_A2_LSB                    0x4105
#define HDMI_REG_CSC_COEF_A3_MSB                    0x4106
#define HDMI_REG_CSC_COEF_A3_LSB                    0x4107
#define HDMI_REG_CSC_COEF_A4_MSB                    0x4108
#define HDMI_REG_CSC_COEF_A4_LSB                    0x4109
#define HDMI_REG_CSC_COEF_B1_MSB                    0x410A
#define HDMI_REG_CSC_COEF_B1_LSB                    0x410B
#define HDMI_REG_CSC_COEF_B2_MSB                    0x410C
#define HDMI_REG_CSC_COEF_B2_LSB                    0x410D
#define HDMI_REG_CSC_COEF_B3_MSB                    0x410E
#define HDMI_REG_CSC_COEF_B3_LSB                    0x410F
#define HDMI_REG_CSC_COEF_B4_MSB                    0x4110
#define HDMI_REG_CSC_COEF_B4_LSB                    0x4111
#define HDMI_REG_CSC_COEF_C1_MSB                    0x4112
#define HDMI_REG_CSC_COEF_C1_LSB                    0x4113
#define HDMI_REG_CSC_COEF_C2_MSB                    0x4114
#define HDMI_REG_CSC_COEF_C2_LSB                    0x4115
#define HDMI_REG_CSC_COEF_C3_MSB                    0x4116
#define HDMI_REG_CSC_COEF_C3_LSB                    0x4117
#define HDMI_REG_CSC_COEF_C4_MSB                    0x4118
#define HDMI_REG_CSC_COEF_C4_LSB                    0x4119

#define HDMI_REG_A_HDCPCFG0                         0x5000
#define HDMI_REG_A_HDCPCFG1                         0x5001
#define HDMI_REG_A_APIINTMSK                        0x5008
#define HDMI_REG_A_VIDPOLCFG                        0x5009

#define HDMI_REG_CEC_MASK                           0x7d02
#define HDMI_REG_CEC_POLARITY                       0x7d03
#define HDMI_REG_CEC_TX_CNT                         0x7d07

#define HDMI_REG_I2CM_SLAVE                         0x7E00
#define HDMI_REG_I2CM_ADDRESS                       0x7E01
#define HDMI_REG_I2CM_DATAO                         0x7E02
#define HDMI_REG_I2CM_DATAI                         0x7E03
#define HDMI_REG_I2CM_OPERATION                     0x7E04
#define HDMI_REG_I2CM_INT                           0x7E05
#define HDMI_REG_I2CM_CTLINT                        0x7E06
#define HDMI_REG_I2CM_DIV                           0x7E07
#define HDMI_REG_I2CM_SEGADDR                       0x7E08
#define HDMI_REG_I2CM_SOFTRSTZ                      0x7E09
#define HDMI_REG_I2CM_SEGPTR                        0x7E0A
#define HDMI_REG_I2CM_SS_SCL_HCNT_1_ADDR            0x7E0B
#define HDMI_REG_I2CM_SS_SCL_HCNT_0_ADDR            0x7E0C
#define HDMI_REG_I2CM_SS_SCL_LCNT_1_ADDR            0x7E0D
#define HDMI_REG_I2CM_SS_SCL_LCNT_0_ADDR            0x7E0E
#define HDMI_REG_I2CM_FS_SCL_HCNT_1_ADDR            0x7E0F
#define HDMI_REG_I2CM_FS_SCL_HCNT_0_ADDR            0x7E10
#define HDMI_REG_I2CM_FS_SCL_LCNT_1_ADDR            0x7E11
#define HDMI_REG_I2CM_FS_SCL_LCNT_0_ADDR            0x7E12
#define HDMI_REG_I2CM_BUF0                          0x7E20

/* register bit definition */
#define PHY_OPMODE_PLLCFG   0x06
#define PHY_CKCALCTRL       0x05
#define PHY_CKSYMTXCTRL     0x09
#define PHY_VLEVCTRL        0x0e
#define PHY_PLLCURRCTRL     0x10
#define PHY_PLLPHBYCTRL     0x13
#define PHY_PLLGMPCTRL      0x15
#define PHY_PLLCLKBISTPHASE 0x17
#define PHY_TXTERM          0x19

#define HDMI_REG_IH_PHY_STAT0_HPD 0x1

#define HDMI_REG_IH_MUTE_MUTE_WAKEUP_INTERRUPT 0x2
#define HDMI_REG_IH_MUTE_MUTE_ALL_INTERRUPT    0x1

#define HDMI_REG_I2C_MASTER_ERROR  0x01
#define HDMI_REG_I2C_MASTER_DONE   0x02
#define HDMI_REG_I2C_SCDC_READ_REQ 0x04

#define HDMI_REG_TX_INSTUFFING_BDBDATA_STUFFING_ENABLE 0x4
#define HDMI_REG_TX_INSTUFFING_RCRDATA_STUFFING_ENABLE 0x2
#define HDMI_REG_TX_INSTUFFING_GYDATA_STUFFING_ENABLE  0x1

#define HDMI_REG_TX_INVID0_INTERNAL_DE_GENERATOR_DISABLE 0x00
#define HDMI_REG_TX_INVID0_VIDEO_MAPPING_MASK            0x1f
#define HDMI_REG_TX_INVID0_VIDEO_MAPPING_OFFSET          0

#define HDMI_REG_VP_STUFF_IDEFAULT_PHASE_MASK           0x20
#define HDMI_REG_VP_STUFF_IDEFAULT_PHASE_OFFSET         5
#define HDMI_REG_VP_STUFF_YCC422_STUFFING_MASK          0x4
#define HDMI_REG_VP_STUFF_YCC422_STUFFING_STUFFING_MODE 0x4
#define HDMI_REG_VP_STUFF_PP_STUFFING_MASK              0x2
#define HDMI_REG_VP_STUFF_PP_STUFFING_STUFFING_MODE     0x2
#define HDMI_REG_VP_STUFF_PR_STUFFING_MASK              0x1
#define HDMI_REG_VP_STUFF_PR_STUFFING_STUFFING_MODE     0x1

#define HDMI_REG_VP_PR_CD_COLOR_DEPTH_MASK         0xf0
#define HDMI_REG_VP_PR_CD_COLOR_DEPTH_OFFSET       4
#define HDMI_REG_VP_PR_CD_DESIRED_PR_FACTOR_MASK   0x0f
#define HDMI_REG_VP_PR_CD_DESIRED_PR_FACTOR_OFFSET 0

#define HDMI_REG_VP_CONF_BYPASS_EN_MASK               0x40
#define HDMI_REG_VP_CONF_BYPASS_EN_ENABLE             0x40
#define HDMI_REG_VP_CONF_PP_EN_ENMASK                 0x20
#define HDMI_REG_VP_CONF_PP_EN_DISABLE                0x00
#define HDMI_REG_VP_CONF_PR_EN_MASK                   0x10
#define HDMI_REG_VP_CONF_PR_EN_DISABLE                0x00
#define HDMI_REG_VP_CONF_YCC422_EN_MASK               0x8
#define HDMI_REG_VP_CONF_YCC422_EN_DISABLE            0x0
#define HDMI_REG_VP_CONF_BYPASS_SELECT_MASK           0x4
#define HDMI_REG_VP_CONF_BYPASS_SELECT_VID_PACKETIZER 0x4
#define HDMI_REG_VP_CONF_OUTPUT_SELECTOR_MASK         0x3
#define HDMI_REG_VP_CONF_OUTPUT_SELECTOR_BYPASS       0x3

#define HDMI_REG_VP_REMAP_YCC422_16BIT 0x0

#define HDMI_REG_FC_AVICONF0_PIX_FMT_MASK            0x03
#define HDMI_REG_FC_AVICONF0_PIX_FMT_RGB             0x00
#define HDMI_REG_FC_AVICONF0_PIX_FMT_YCBCR422        0x01
#define HDMI_REG_FC_AVICONF0_PIX_FMT_YCBCR444        0x02
#define HDMI_REG_FC_AVICONF0_ACTIVE_FMT_MASK         0x40
#define HDMI_REG_FC_AVICONF0_ACTIVE_FMT_INFO_PRESENT 0x40
#define HDMI_REG_FC_AVICONF0_ACTIVE_FMT_NO_INFO      0x00
#define HDMI_REG_FC_AVICONF0_BAR_DATA_MASK           0x0c
#define HDMI_REG_FC_AVICONF0_BAR_DATA_NO_DATA        0x00
#define HDMI_REG_FC_AVICONF0_BAR_DATA_VERT_BAR       0x04
#define HDMI_REG_FC_AVICONF0_BAR_DATA_HORIZ_BAR      0x08
#define HDMI_REG_FC_AVICONF0_BAR_DATA_VERT_HORIZ_BAR 0x0c
#define HDMI_REG_FC_AVICONF0_SCAN_INFO_MASK          0x30
#define HDMI_REG_FC_AVICONF0_SCAN_INFO_OVERSCAN      0x10
#define HDMI_REG_FC_AVICONF0_SCAN_INFO_UNDERSCAN     0x20
#define HDMI_REG_FC_AVICONF0_SCAN_INFO_NODATA        0x00

#define HDMI_REG_FC_AVICONF1_ACTIVE_ASPECT_RATIO_MASK      0x0f
#define HDMI_REG_FC_AVICONF1_ACTIVE_ASPECT_RATIO_USE_CODED 0x08
#define HDMI_REG_FC_AVICONF1_ACTIVE_ASPECT_RATIO_4_3       0x09
#define HDMI_REG_FC_AVICONF1_ACTIVE_ASPECT_RATIO_16_9      0x0a
#define HDMI_REG_FC_AVICONF1_ACTIVE_ASPECT_RATIO_14_9      0x0b
#define HDMI_REG_FC_AVICONF1_CODED_ASPECT_RATIO_MASK       0x30
#define HDMI_REG_FC_AVICONF1_CODED_ASPECT_RATIO_NO_DATA    0x00
#define HDMI_REG_FC_AVICONF1_CODED_ASPECT_RATIO_4_3        0x10
#define HDMI_REG_FC_AVICONF1_CODED_ASPECT_RATIO_16_9       0x20
#define HDMI_REG_FC_AVICONF1_COLORIMETRY_MASK              0xc0
#define HDMI_REG_FC_AVICONF1_COLORIMETRY_NO_DATA           0x00
#define HDMI_REG_FC_AVICONF1_COLORIMETRY_SMPTE             0x40
#define HDMI_REG_FC_AVICONF1_COLORIMETRY_ITUR              0x80
#define HDMI_REG_FC_AVICONF1_COLORIMETRY_EXTENDED_INFO     0xc0

#define HDMI_REG_FC_AVICONF2_SCALING_MASK                 0x03
#define HDMI_REG_FC_AVICONF2_SCALING_NONE                 0x00
#define HDMI_REG_FC_AVICONF2_SCALING_HORIZ                0x01
#define HDMI_REG_FC_AVICONF2_SCALING_VERT                 0x02
#define HDMI_REG_FC_AVICONF2_SCALING_HORIZ_vert           0x03
#define HDMI_REG_FC_AVICONF2_RGB_QUANT_MASK               0x0c
#define HDMI_REG_FC_AVICONF2_RGB_QUANT_DEFAULT            0x00
#define HDMI_REG_FC_AVICONF2_RGB_QUANT_LIMITED_RANGE      0x04
#define HDMI_REG_FC_AVICONF2_RGB_QUANT_FULL_RANGE         0x08
#define HDMI_REG_FC_AVICONF2_EXT_COLORIMETRY_MASK         0x70
#define HDMI_REG_FC_AVICONF2_EXT_COLORIMETRY_XVYCC601     0x00
#define HDMI_REG_FC_AVICONF2_EXT_COLORIMETRY_XVYCC709     0x10
#define HDMI_REG_FC_AVICONF2_EXT_COLORIMETRY_SYCC601      0x20
#define HDMI_REG_FC_AVICONF2_EXT_COLORIMETRY_ADOBE_YCC601 0x30
#define HDMI_REG_FC_AVICONF2_EXT_COLORIMETRY_ADOBE_RGB    0x40
#define HDMI_REG_FC_AVICONF2_IT_CONTENT_MASK              0x80
#define HDMI_REG_FC_AVICONF2_IT_CONTENT_NO_DATA           0x00
#define HDMI_REG_FC_AVICONF2_IT_CONTENT_VALID             0x80

#define HDMI_REG_FC_AVICONF3_IT_CONTENT_TYPE_MASK     0x03
#define HDMI_REG_FC_AVICONF3_IT_CONTENT_TYPE_GRAPHICS 0x00
#define HDMI_REG_FC_AVICONF3_IT_CONTENT_TYPE_PHOTO    0x01
#define HDMI_REG_FC_AVICONF3_IT_CONTENT_TYPE_CINEMA   0x02
#define HDMI_REG_FC_AVICONF3_IT_CONTENT_TYPE_GAME     0x03
#define HDMI_REG_FC_AVICONF3_QUANT_RANGE_MASK         0x0c
#define HDMI_REG_FC_AVICONF3_QUANT_RANGE_LIMITED      0x00
#define HDMI_REG_FC_AVICONF3_QUANT_RANGE_FULL         0x04

#define HDMI_REG_FC_INVIDCONF_HDCP_KEEPOUT_MASK             0x80
#define HDMI_REG_FC_INVIDCONF_HDCP_KEEPOUT_ACTIVE           0x80
#define HDMI_REG_FC_INVIDCONF_HDCP_KEEPOUT_INACTIVE         0x00
#define HDMI_REG_FC_INVIDCONF_VSYNC_IN_POLARITY_MASK        0x40
#define HDMI_REG_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_HIGH 0x40
#define HDMI_REG_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_LOW  0x00
#define HDMI_REG_FC_INVIDCONF_HSYNC_IN_POLARITY_MASK        0x20
#define HDMI_REG_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_HIGH 0x20
#define HDMI_REG_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_LOW  0x00
#define HDMI_REG_FC_INVIDCONF_DE_IN_POLARITY_MASK           0x10
#define HDMI_REG_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_HIGH    0x10
#define HDMI_REG_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_LOW     0x00
#define HDMI_REG_FC_INVIDCONF_DVI_MODEZ_MASK                0x8
#define HDMI_REG_FC_INVIDCONF_DVI_MODEZ_HDMI_REG_MODE           0x8
#define HDMI_REG_FC_INVIDCONF_DVI_MODEZ_DVI_MODE            0x0
#define HDMI_REG_FC_INVIDCONF_R_V_BLANK_IN_OSC_MASK         0x2
#define HDMI_REG_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_HIGH  0x2
#define HDMI_REG_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_LOW   0x0
#define HDMI_REG_FC_INVIDCONF_IN_I_P_MASK                   0x1
#define HDMI_REG_FC_INVIDCONF_IN_I_P_INTERLACED             0x1
#define HDMI_REG_FC_INVIDCONF_IN_I_P_PROGRESSIVE            0x0

#define HDMI_REG_FC_GCP_SET_AVMUTE   0x02
#define HDMI_REG_FC_GCP_CLEAR_AVMUTE 0x01

#define HDMI_REG_FC_PRCONF_INCOMING_PR_FACTOR_OFFSET  0x04
#define HDMI_REG_FC_PRCONF_INCOMING_PR_FACTOR_MASK    0xF0
#define HDMI_REG_FC_PRCONF_OUTPUT_PR_FACTOR_OFFSET    0x00
#define HDMI_REG_FC_PRCONF_OUTPUT_PR_FACTOR_MASK      0x0F

#define HDMI_REG_PHY_CONF0_PDZ_MASK            0x80
#define HDMI_REG_PHY_CONF0_PDZ_OFFSET          7
#define HDMI_REG_PHY_CONF0_ENTMDS_MASK         0x40
#define HDMI_REG_PHY_CONF0_ENTMDS_OFFSET       6
#define HDMI_REG_PHY_CONF0_SPARECTRL_MASK      0x20
#define HDMI_REG_PHY_CONF0_SPARECTRL_OFFSET    5
#define HDMI_REG_PHY_CONF0_GEN2_PDDQ_MASK      0x10
#define HDMI_REG_PHY_CONF0_GEN2_PDDQ_OFFSET    4
#define HDMI_REG_PHY_CONF0_GEN2_TXPWRON_MASK   0x8
#define HDMI_REG_PHY_CONF0_GEN2_TXPWRON_OFFSET 3
#define HDMI_REG_PHY_CONF0_SELDATAENPOL_MASK   0x2
#define HDMI_REG_PHY_CONF0_SELDATAENPOL_OFFSET 1
#define HDMI_REG_PHY_CONF0_SELDIPIF_MASK       0x1
#define HDMI_REG_PHY_CONF0_SELDIPIF_OFFSET     0

#define HDMI_REG_PHY_TST0_TSTCLR_MASK   0x20
#define HDMI_REG_PHY_TST0_TSTCLR_OFFSET 5

#define HDMI_REG_PHY_HPD         0x02
#define HDMI_REG_PHY_TX_PHY_LOCK 0x01

#define HDMI_REG_PHY_I2CM_SLAVE_ADDR_PHY_GEN2 0x69

#define HDMI_REG_PHY_I2CM_OPERATION_ADDR_WRITE 0x10

#define HDMI_REG_PHY_I2CM_INT_ADDR_DONE_POL 0x08

#define HDMI_REG_PHY_I2CM_CTLINT_ADDR_NAC_POL         0x80
#define HDMI_REG_PHY_I2CM_CTLINT_ADDR_ARBITRATION_POL 0x08

#define HDMI_REG_I2CM_SLAVE_DDC_ADDR    0x50
#define HDMI_REG_I2CM_SEGADDR_DDC       0x30
#define HDMI_REG_I2CM_OP_RD8_EXT        0x2
#define HDMI_REG_I2CM_OP_RD8            0x1
#define HDMI_REG_I2CM_DIV_FAST_STD_MODE 0x8
#define HDMI_REG_I2CM_DIV_FAST_MODE     0x8
#define HDMI_REG_I2CM_DIV_STD_MODE      0x0
#define HDMI_REG_I2CM_SOFTRSTZ_MASK     0x1
#define HDMI_REG_I2CM_SOFTRSTZ_MASK     0x1

#define HDMI_REG_AUD_CONF0_SW_AUDIO_FIFO_RST 0x80
#define HDMI_REG_AUD_CONF0_I2S_SELECT        0x20
#define HDMI_REG_AUD_CONF0_I2S_IN_EN_0       0x01
#define HDMI_REG_AUD_CONF0_I2S_IN_EN_1       0x02
#define HDMI_REG_AUD_CONF0_I2S_IN_EN_2       0x04
#define HDMI_REG_AUD_CONF0_I2S_IN_EN_3       0x08

#define HDMI_REG_AUD_CONF1_I2S_MODE_STANDARD_MODE 0x0
#define HDMI_REG_AUD_CONF1_I2S_WIDTH_16BIT        0x10

#define HDMI_REG_AUD_N3_NCTS_ATOMIC_WRITE 0x80
#define HDMI_REG_AUD_N3_AUDN19_16_MASK    0x0f

#define HDMI_REG_AUD_CTS3_N_SHIFT_OFFSET   5
#define HDMI_REG_AUD_CTS3_N_SHIFT_MASK     0xe0
#define HDMI_REG_AUD_CTS3_N_SHIFT_1        0
#define HDMI_REG_AUD_CTS3_N_SHIFT_16       0x20
#define HDMI_REG_AUD_CTS3_N_SHIFT_32       0x40
#define HDMI_REG_AUD_CTS3_N_SHIFT_64       0x60
#define HDMI_REG_AUD_CTS3_N_SHIFT_128      0x80
#define HDMI_REG_AUD_CTS3_N_SHIFT_256      0xa0
#define HDMI_REG_AUD_CTS3_CTS_MANUAL       0x10
#define HDMI_REG_AUD_CTS3_AUDCTS19_16_MASK 0x0f

#define HDMI_REG_AUD_INPUTCLKFS_128 0x0

#define HDMI_REG_MC_CLKDIS_HDCPCLK_DISABLE 0x40
#define HDMI_REG_MC_CLKDIS_CECCLK_DISABLE   0x20
#define HDMI_REG_MC_CLKDIS_CSCCLK_DISABLE   0x10
#define HDMI_REG_MC_CLKDIS_AUDCLK_DISABLE   0x8
#define HDMI_REG_MC_CLKDIS_PREPCLK_DISABLE  0x4
#define HDMI_REG_MC_CLKDIS_TMDSCLK_DISABLE  0x2
#define HDMI_REG_MC_CLKDIS_PIXELCLK_DISABLE 0x1

#define HDMI_REG_MC_SWRSTZ_II2SSWRST_REQ 0x08
#define HDMI_REG_MC_SWRSTZ_TMDSSWRST_REQ 0x02

#define HDMI_REG_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_IN_PATH 0x1
#define HDMI_REG_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS  0x0

#define HDMI_REG_MC_PHYRSTZ_ASSERT   0x0
#define HDMI_REG_MC_PHYRSTZ_DEASSERT 0x1

#define HDMI_REG_MC_HEACPHY_RST_ASSERT 0x1

#define HDMI_REG_CSC_CFG_INTMODE_MASK                0x30
#define HDMI_REG_CSC_CFG_INTMODE_OFFSET              4
#define HDMI_REG_CSC_CFG_INTMODE_DISABLE             0x00
#define HDMI_REG_CSC_CFG_INTMODE_CHROMA_INT_FORMULA1 0x10
#define HDMI_REG_CSC_CFG_INTMODE_CHROMA_INT_FORMULA2 0x20
#define HDMI_REG_CSC_CFG_DECMODE_MASK                0x3
#define HDMI_REG_CSC_CFG_DECMODE_OFFSET              0
#define HDMI_REG_CSC_CFG_DECMODE_DISABLE             0x0
#define HDMI_REG_CSC_CFG_DECMODE_CHROMA_INT_FORMULA1 0x1
#define HDMI_REG_CSC_CFG_DECMODE_CHROMA_INT_FORMULA2 0x2
#define HDMI_REG_CSC_CFG_DECMODE_CHROMA_INT_FORMULA3 0x3

#define HDMI_REG_CSC_SCALE_CSC_COLORDE_PTH_MASK  0xF0
#define HDMI_REG_CSC_SCALE_CSC_COLORDE_PTH_24BPP 0x00
#define HDMI_REG_CSC_SCALE_CSC_COLORDE_PTH_30BPP 0x50
#define HDMI_REG_CSC_SCALE_CSC_COLORDE_PTH_36BPP 0x60
#define HDMI_REG_CSC_SCALE_CSC_COLORDE_PTH_48BPP 0x70
#define HDMI_REG_CSC_SCALE_CSCSCALE_MASK         0x03

#define HDMI_REG_A_HDCPCFG0_RXDETECT_DISABLE          0x0
#define HDMI_REG_A_HDCPCFG0_RXDETECT_MASK             0x4
#define HDMI_REG_A_VIDPOLCFG_DATAENPOL_ACTIVE_HIGH    0x10
#define HDMI_REG_A_VIDPOLCFG_DATAENPOL_MASK           0x10
#define HDMI_REG_A_HDCPCFG1_ENCRYPTIONDISABLE_MASK    0x2
#define HDMI_REG_A_HDCPCFG1_ENCRYPTIONDISABLE_DISABLE 0x2

/*------------------HDMI info frame----------------*/
#define HDMI_REG_SCAN_MODE_UNDERSCAN             0x02
#define HDMI_REG_COLORSPACE_RGB                  0x00
#define HDMI_REG_COLORIMETRY_NONE                0x00
#define HDMI_REG_ACTIVE_ASPECT_PICTURE           0x08
#define HDMI_REG_PICTURE_ASPECT_16_9             0x02
#define HDMI_REG_EXTENDED_COLORIMETRY_XV_YCC_601 0x00
#define HDMI_REG_QUANTIZATION_RANGE_LIMITED      0x01
#define HDMI_REG_QUANTIZATION_RANGE_FULL         0x02
#define HDMI_REG_CONTENT_TYPE_GRAPHICS           0x00
#define HDMI_REG_YCC_QUANTIZATION_RANGE_LIMITED  0x00
#define HDMI_REG_YCC_QUANTIZATION_RANGE_FULL     0x01

EFI_STATUS dw_hdmi_phy_wait_for_hpd(void);
void dw_hdmi_phy_init(void);
void dw_hdmi_phy_reset(void);

EFI_STATUS dw_hdmi_enable(IMX_DISPLAY_TIMING *edid, uint32_t in_bus_fmt, uint32_t out_bus_fmt);
EFI_STATUS dw_hdmi_read_edid(uint8_t *buff, uint32_t block, uint32_t lenght);
void dw_hdmi_init(int (*phy_init)(const IMX_DISPLAY_TIMING *mode));
void dw_hdmi_dump_regs(void);

EFI_STATUS dw_hdmi_imx_phy_clk_set_pll(uint32_t rate, uint32_t *real_rate);
void dw_hdmi_imx_deinit(void);
void dw_hdmi_imx_init(void);

void ClkResetDump (void);
void PaviPhyDump (void);

#endif
