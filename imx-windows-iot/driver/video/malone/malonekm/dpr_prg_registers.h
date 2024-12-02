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

#ifndef DPR_PRG_REGISTERS_H
#define DPR_PRG_REGISTERS_H

#define IMX_DPR_PRG_SET_OFFSET                               0x4U
#define IMX_DPR_PRG_CLR_OFFSET                               0x8U
#define IMX_DPR_PRG_TOG_OFFSET                               0xCU

/* DPR: System Control 0*/
#define IMX_DPR_SYSTEM_CTRL0                                 ((uint32_t)(0x0))
#define IMX_DPR_SYSTEM_CTRL0_RUN_EN_MASK                     0x1U
#define IMX_DPR_SYSTEM_CTRL0_RUN_EN_SHIFT                    0x0U
#define IMX_DPR_SYSTEM_CTRL0_SOFT_RESET_MASK                 0x2U
#define IMX_DPR_SYSTEM_CTRL0_SOFT_RESET_SHIFT                0x1U
#define IMX_DPR_SYSTEM_CTRL0_REPEAT_EN_MASK                  0x4U
#define IMX_DPR_SYSTEM_CTRL0_REPEAT_EN_SHIFT                 0x2U
#define IMX_DPR_SYSTEM_CTRL0_SHADOW_LOAD_EN_MASK             0x8U
#define IMX_DPR_SYSTEM_CTRL0_SHADOW_LOAD_EN_SHIFT            0x3U
#define IMX_DPR_SYSTEM_CTRL0_SW_SHADOW_LOAD_SEL_MASK         0x10U
#define IMX_DPR_SYSTEM_CTRL0_SW_SHADOW_LOAD_SEL_SHIFT        0x4U
#define IMX_DPR_SYSTEM_CTRL0_BCMD2AXI_MSTR_ID_CTRL_MASK      0x010000U
#define IMX_DPR_SYSTEM_CTRL0_BCMD2AXI_MSTR_ID_CTRL_SHIFT     0x10U

/* DPR: Interrupt Mask */
#define IMX_DPR_IRQ_MASK                                     ((uint32_t)(0x20))

/* DPR: Status Register of Masked IRQ */
#define IMX_DPR_IRQ_MASK_STATUS                              ((uint32_t)(0x30))

/* DPR: Status of Non-Masked IRQ */
#define IMX_DPR_IRQ_NONMASK_STATUS                           ((uint32_t)(0x40))

/* DPR: Mode Control 0 */
#define IMX_DPR_MODE_CTRL0                                   ((uint32_t)(0x50))
#define IMX_DPR_MODE_CTRL0_RTR_3BUF_EN_MASK                  0x1U
#define IMX_DPR_MODE_CTRL0_RTR_3BUF_EN_SHIFT                 0x0U
#define IMX_DPR_MODE_CTRL0_RTR_4LINE_BUF_EN_MASK             0x2U
#define IMX_DPR_MODE_CTRL0_RTR_4LINE_BUF_EN_SHIFT            0x1U
#define IMX_DPR_MODE_CTRL0_TILE_TYPE_MASK                    0xCU
#define IMX_DPR_MODE_CTRL0_TILE_TYPE_SHIFT                   0x2U
#define IMX_DPR_MODE_CTRL0_YUV_EN_MASK                       0x10U
#define IMX_DPR_MODE_CTRL0_YUV_EN_SHIFT                      0x4U
#define IMX_DPR_MODE_CTRL0_COMP_2PLANE_EN_MASK               0x20U
#define IMX_DPR_MODE_CTRL0_COMP_2PLANE_EN_SHIFT              0x5U
#define IMX_DPR_MODE_CTRL0_PIX_SIZE_MASK                     0xC0U
#define IMX_DPR_MODE_CTRL0_PIX_SIZE_SHIFT                    0x6U
#define IMX_DPR_MODE_CTRL0_PIX_LUMA_UV_SWAP_MASK             0x100U
#define IMX_DPR_MODE_CTRL0_PIX_LUMA_UV_SWAP_SHIFT            0x8U
#define IMX_DPR_MODE_CTRL0_PIX_UV_SWAP_MASK                  0x200U
#define IMX_DPR_MODE_CTRL0_PIX_UV_SWAP_SHIFT                 0x9U
#define IMX_DPR_MODE_CTRL0_B_COMP_SEL_MASK                   0xC00U
#define IMX_DPR_MODE_CTRL0_B_COMP_SEL_SHIFT                  0xAU
#define IMX_DPR_MODE_CTRL0_G_COMP_SEL_MASK                   0x3000U
#define IMX_DPR_MODE_CTRL0_G_COMP_SEL_SHIFT                  0xCU
#define IMX_DPR_MODE_CTRL0_R_COMP_SEL_MASK                   0xC000U
#define IMX_DPR_MODE_CTRL0_R_COMP_SEL_SHIFT                  0xEU
#define IMX_DPR_MODE_CTRL0_A_COMP_SEL_MASK                   0x30000U
#define IMX_DPR_MODE_CTRL0_A_COMP_SEL_SHIFT                  0x10U

/* DPR: Frame Control 0 */
#define IMX_DPR_FRAME_CTRL0                                  ((uint32_t)(0x70))
#define IMX_DPR_FRAME_CTRL0_HFLIP_EN_MASK                    0x1U
#define IMX_DPR_FRAME_CTRL0_HFLIP_EN_SHIFT                   0x0U
#define IMX_DPR_FRAME_CTRL0_VFLIP_EN_MASK                    0x2U
#define IMX_DPR_FRAME_CTRL0_VFLIP_EN_SHIFT                   0x1U
#define IMX_DPR_FRAME_CTRL0_ROT_ENC_MASK                     0xCU
#define IMX_DPR_FRAME_CTRL0_ROT_ENC_SHIFT                    0x2U
#define IMX_DPR_FRAME_CTRL0_ROT_FLIP_ORDER_EN_MASK           0x10U
#define IMX_DPR_FRAME_CTRL0_ROT_FLIP_ORDER_EN_SHIFT          0x4U
#define IMX_DPR_FRAME_CTRL0_PITCH_MASK                       0xFFFF0000U
#define IMX_DPR_FRAME_CTRL0_PITCH_SHIFT                      0x10U

/* DPR: Frame 1-Plane Control 0 */
#define IMX_DPR_FRAME_1P_CTRL0                               ((uint32_t)(0x90))
#define IMX_DPR_FRAME_1P_CTRL0_MAX_BYTES_PREQ_MASK           0x7U
#define IMX_DPR_FRAME_1P_CTRL0_MAX_BYTES_PREQ_SHIFT          0x0U

/* DPR: Frame 1-Plane Pix X Control */
#define IMX_DPR_FRAME_1P_PIX_X_CTRL                          ((uint32_t)(0xA0))
#define IMX_DPR_FRAME_1P_PIX_X_CTRL_NUM_X_PIX_WIDE_MASK      0xFFFFU
#define IMX_DPR_FRAME_1P_PIX_X_CTRL_NUM_X_PIX_WIDE_SHIFT     0x0U

/* DPR: Frame 1-Plane Pix Y Control */
#define IMX_DPR_FRAME_1P_PIX_Y_CTRL                          ((uint32_t)(0xB0))
#define IMX_DPR_FRAME_1P_PIX_Y_CTRL_NUM_Y_PIX_HIGH_MASK      0xFFFFU
#define IMX_DPR_FRAME_1P_PIX_Y_CTRL_NUM_Y_PIX_HIGH_SHIFT     0x0U

/* DPR: Frame 1-Plane Base Address Control 0 */
#define IMX_DPR_FRAME_1P_BASE_ADDR_CTRL0                     ((uint32_t)(0xC0))

/* DPR: Frame 2-Plane Control 0 */
#define IMX_DPR_FRAME_2P_CTRL0                               ((uint32_t)(0xE0))
#define IMX_DPR_FRAME_2P_CTRL0_MAX_BYTES_PREQ_MASK           0x7U
#define IMX_DPR_FRAME_2P_CTRL0_MAX_BYTES_PREQ_SHIFT          0x0U

/* DPR: Frame Pixel X Upper Left Coordinate Control  */
#define IMX_DPR_FRAME_PIX_X_ULC_CTRL                         ((uint32_t)(0xF0))
#define IMX_DPR_FRAME_PIX_X_ULC_CTRL_CROP_ULC_X_MASK         0xFFFFU
#define IMX_DPR_FRAME_PIX_X_ULC_CTRL_CROP_ULC_X_SHIFT        0x0U

/* DPR: Frame Pixel Y Upper Left Coordinate Control */
#define IMX_DPR_FRAME_PIX_Y_ULC_CTRL                         ((uint32_t)(0x100))
#define IMX_DPR_FRAME_PIX_Y_ULC_CTRL_CROP_ULC_Y_MASK         0xFFFFU
#define IMX_DPR_FRAME_PIX_Y_ULC_CTRL_CROP_ULC_Y_SHIFT        0x0U

/* DPR: Frame 2-Plane Base Address Control 0 */
#define IMX_DPR_FRAME_2P_BASE_ADDR_CTRL0                     ((uint32_t)(0x110))

/* DPR: RTRAM Control 0  */
#define IMX_DPR_RTRAM_CTRL0                                  ((uint32_t)(0x200))
#define IMX_DPR_RTRAM_CTRL0_NUM_ROWS_ACTIVE_MASK             0x1U
#define IMX_DPR_RTRAM_CTRL0_NUM_ROWS_ACTIVE_SHIFT            0x0U
#define IMX_DPR_RTRAM_CTRL0_THRES_HIGH_MASK                  0xEU
#define IMX_DPR_RTRAM_CTRL0_THRES_HIGH_SHIFT                 0x1U
#define IMX_DPR_RTRAM_CTRL0_THRES_LOW_MASK                   0x70U
#define IMX_DPR_RTRAM_CTRL0_THRES_LOW_SHIFT                  0x4U
#define IMX_DPR_RTRAM_CTRL0_ABORT_SEL_MASK                   0x80U
#define IMX_DPR_RTRAM_CTRL0_ABORT_SEL_SHIFT                  0x7U




/* PRG: Control Register */
#define IMX_PRG_CTRL                                         ((uint32_t)(0x0))
#define IMX_PRG_CTRL_BYPASS_MASK                             0x01U
#define IMX_PRG_CTRL_BYPASS_SHIFT                            0x0U
#define IMX_PRG_CTRL_SC_DATA_TYPE_MASK                       0x4U
#define IMX_PRG_CTRL_SC_DATA_TYPE_SHIFT                      0x2U
#define IMX_PRG_CTRL_UV_EN_MASK                              0x8U
#define IMX_PRG_CTRL_UV_EN_SHIFT                             0x3U
#define IMX_PRG_CTRL_HANDSHAKE_MODE_MASK                     0x10U
#define IMX_PRG_CTRL_HANDSHAKE_MODE_SHIFT                    0x4U
#define IMX_PRG_CTRL_SHADOW_LOAD_MODE_MASK                   0x20U
#define IMX_PRG_CTRL_SHADOW_LOAD_MODE_SHIFT                  0x5U
#define IMX_PRG_CTRL_DES_DATA_TYPE_MASK                      0x30000U
#define IMX_PRG_CTRL_DES_DATA_TYPE_SHIFT                     0x10U
#define IMX_PRG_CTRL_SOFTRST_MASK                            0x40000000U
#define IMX_PRG_CTRL_SOFTRST_SHIFT                           0x1EU
#define IMX_PRG_CTRL_SHADOW_EN_MASK                          0x80000000U
#define IMX_PRG_CTRL_SHADOW_EN_SHIFT                         0x1FU

/* PRG: Status Register */
#define IMX_PRG_STATUS                                       ((uint32_t)(0x10))
#define IMX_PRG_STATUS_BUFFER_VALID_A_MASK                   0x1U
#define IMX_PRG_STATUS_BUFFER_VALID_A_SHIFT                  0x0U
#define IMX_PRG_STATUS_BUFFER_VALID_B_MASK                   0x2U
#define IMX_PRG_STATUS_BUFFER_VALID_B_SHIFT                  0x1U

/* PRG: REG Update Register */
#define IMX_PRG_REG_UPDATE                                   ((uint32_t)(0x20))
#define IMX_PRG_REG_UPDATE_REG_UPDATE_MASK                   0x1U
#define IMX_PRG_REG_UPDATE_REG_UPDATE_SHIFT                  0x0U

/* PRG: Stride Register */
#define IMX_PRG_STRIDE                                       ((uint32_t)(0x30))
#define IMX_PRG_STRIDE_STRIDE_MASK                           0xFFFFU
#define IMX_PRG_STRIDE_STRIDE_SHIFT                          0x0U

/* PRG: Height Register */
#define IMX_PRG_HEIGHT                                       ((uint32_t)(0x40))
#define IMX_PRG_HEIGHT_HEIGHT_MASK                           0xFFFFU
#define IMX_PRG_HEIGHT_HEIGHT_SHIFT                          0x0U

/* PRG: Base Address Register */
#define IMX_PRG_BADDR                                        ((uint32_t)(0x50))

/* PRG: Offset Address Register */
#define IMX_PRG_OFFSET                                       ((uint32_t)(0x60))
#define IMX_PRG_OFFSET_X_MASK                                0xFFFFU
#define IMX_PRG_OFFSET_X_SHIFT                               0x0U
#define IMX_PRG_OFFSET_Y_MASK                                0x70000U
#define IMX_PRG_OFFSET_Y_SHIFT                               0x10U

/* PRG: Width Register */
#define IMX_PRG_WIDTH                                        ((uint32_t)(0x70))
#define IMX_PRG_WIDTH_WIDTH_MASK                             0xFFFFU
#define IMX_PRG_WIDTH_WIDTH_SHIFT                            0x0U


#endif
