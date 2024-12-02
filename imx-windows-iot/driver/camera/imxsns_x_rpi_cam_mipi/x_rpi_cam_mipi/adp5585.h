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
#include "ImxCameraInterface.h"

#define EXP_R0_CSI_RST                          0x01U

#define CAM_EXP_R0_DVDD_SEL                     0x01U
#define CAM_EXP_R1_VAA_SEL                      0x02U
#define CAM_EXP_R2_ISP_BYP                      0x04U
#define CAM_EXP_R3_FLED_TOR_TX                  0x08U
#define CAM_EXP_R4_VCM_PWREN                    0x10U
#define CAM_EXP_R5_ISP_STBY                     0x20U
#define CAM_EXP_C0_PWREN_1                      0x01U
#define CAM_EXP_C1_PWREN_2                      0x02U
#define CAM_EXP_C2_PWREN_3                      0x04U
#define CAM_EXP_C3_PWREN_4                      0x08U
#define CAM_EXP_C4_PWREN_5                      0x10U

/* ADP5585 registers */
#define ADP5585_ID_REG					0x00
#define ADP5585_INT_STATUS_REG			0x01
#define ADP5585_STATUS_REG				0x02
#define ADP5585_FIFO_1_REG				0x03
#define ADP5585_FIFO_2_REG				0x04
#define ADP5585_FIFO_3_REG				0x05
#define ADP5585_FIFO_4_REG				0x06
#define ADP5585_FIFO_5_REG				0x07
#define ADP5585_FIFO_6_REG				0x08
#define ADP5585_FIFO_7_REG				0x09
#define ADP5585_FIFO_8_REG				0x0A
#define ADP5585_FIFO_9_REG				0x0B
#define ADP5585_FIFO_10_REG				0x0C
#define ADP5585_FIFO_11_REG				0x0D
#define ADP5585_FIFO_12_REG				0x0E
#define ADP5585_FIFO_13_REG				0x0F
#define ADP5585_FIFO_14_REG				0x10
#define ADP5585_FIFO_15_REG				0x11
#define ADP5585_FIFO_16_REG				0x12
#define ADP5585_GPI_INT_STAT_A_REG		0x13
#define ADP5585_GPI_INT_STAT_B_REG		0x14
#define ADP5585_GPI_STATUS_A_REG		0x15
#define ADP5585_GPI_STATUS_B_REG		0x16
#define ADP5585_RPULL_CONFIG_A_REG		0x17
#define ADP5585_RPULL_CONFIG_B_REG		0x18
#define ADP5585_RPULL_CONFIG_C_REG		0x19
#define ADP5585_RPULL_CONFIG_D_REG		0x1A
#define ADP5585_GPI_INT_LEVEL_A_REG		0x1B
#define ADP5585_GPI_INT_LEVEL_B_REG		0x1C
#define ADP5585_GPI_EVENT_EN_A_REG		0x1D
#define ADP5585_GPI_EVENT_EN_B_REG		0x1E
#define ADP5585_GPI_INTERRUPT_EN_A_REG	0x1F
#define ADP5585_GPI_INTERRUPT_EN_B_REG	0x20
#define ADP5585_DEBOUNCE_DIS_A_REG		0x21
#define ADP5585_DEBOUNCE_DIS_B_REG		0x22
#define ADP5585_GPO_DATA_OUT_A_REG		0x23
#define ADP5585_GPO_DATA_OUT_B_REG		0x24
#define ADP5585_GPO_OUT_MODE_A_REG		0x25
#define ADP5585_GPO_OUT_MODE_B_REG		0x26
#define ADP5585_GPIO_DIRECTION_A_REG	0x27
#define ADP5585_GPIO_DIRECTION_B_REG	0x28
#define ADP5585_RESET1_EVENT_A_REG		0x29
#define ADP5585_RESET1_EVENT_B_REG		0x2A
#define ADP5585_RESET1_EVENT_C_REG		0x2B
#define ADP5585_RESET2_EVENT_A_REG		0x2C
#define ADP5585_RESET2_EVENT_B_REG		0x2D
#define ADP5585_RESET_CFG_REG			0x2E
#define ADP5585_PWM_OFFT_LOW_REG		0x2F
#define ADP5585_PWM_OFFT_HIGH_REG		0x30
#define ADP5585_PWM_ONT_LOW_REG			0x31
#define ADP5585_PWM_ONT_HIGH_REG		0x32
#define ADP5585_PWM_CFG_REG				0x33
#define ADP5585_LOGIC_CFG_REG			0x34
#define ADP5585_LOGIC_FF_CFG_REG		0x35
#define ADP5585_LOGIC_INT_EVENT_EN_REG	0x36
#define ADP5585_POLL_PTIME_CFG_REG		0x37
#define ADP5585_PIN_CONFIG_A_REG		0x38
#define ADP5585_PIN_CONFIG_B_REG		0x39
#define ADP5585_PIN_CONFIG_C_REG		0x3A
#define ADP5585_GENERAL_CFG_REG			0x3B
#define ADP5585_INT_EN_REG				0x3C
