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

#ifndef IMX_CLK_IMX8MP_REGISTERS_H_
#define IMX_CLK_IMX8MP_REGISTERS_H_

#include "AnatopPll.h"

/* Clock Control Module (CCM) */
#define IMX_CCM_BASE                               0x44450000
#define IMX_CCM_SIZE                               0x10000

/* CCM Registers, offset from base address */
#define IMX_CCM_LPCGn_DIRECT(n)                    (0x8000 + (0x40 * (n)))     /* LPCG direct control */
#define IMX_CCM_LPCG_DIRECT_ON_MASK                (1 << 0)
#define IMX_CCM_LPCG_DIRECT_ON(v)                  (((v) << 0) & 0x1)

#define IMX_CCM_CLOCK_ROOTn_CONTROL(n)             (0x0000 + (0x80 * (n)))     /* Clock Root Control */
#define IMX_CCM_CLOCK_ROOTn_CONTROL_SET(n)         (0x0004 + (0x80 * (n)))     /* Clock Root Control */
#define IMX_CCM_CLOCK_ROOTn_CONTROL_CLR(n)         (0x0008 + (0x80 * (n)))     /* Clock Root Control */
#define IMX_CCM_CLOCK_ROOTn_CONTROL_TOG(n)         (0x000C + (0x80 * (n)))     /* Clock Root Control */
#define IMX_CCM_CLOCK_ROOTn_STATUS(n)              (0x0020 + (0x80 * (n)))     /* Clock Root Status */


/* Clock Root Control Register (CLOCK_ROOT0_CONTROL - CLOCK_ROOT94_CONTROL) Register bits */
#define IMX_CCM_CLOCK_ROOT_CONTROL_OFF_MASK        (0x01UL << 24)
#define IMX_CCM_CLOCK_ROOT_CONTROL_OFF_SHIFT       (24)
#define IMX_CCM_CLOCK_ROOT_CONTROL_MUX_MASK        (0x03UL << 8)
#define IMX_CCM_CLOCK_ROOT_CONTROL_MUX_SHIFT       (8)
#define IMX_CCM_CLOCK_ROOT_CONTROL_MUX(x)          (((x) << 8) & 0x0300UL)
#define IMX_CCM_CLOCK_ROOT_CONTROL_DIV_MASK        (0xFFUL << 0)
#define IMX_CCM_CLOCK_ROOT_CONTROL_DIV_SHIFT       (0)
#define IMX_CCM_CLOCK_ROOT_CONTROL_DIV(x)          (((x) << 0) & 0x00FFUL)
#define IMX_CCM_CLOCK_ROOT_CONTROL_DIV_MAX         256

/* Clock root working status (CLOCK_ROOT0_STATUS0 - CLOCK_ROOT94_STATUS0) Register bits */
#define IMX_CCM_CLOCK_ROOT_STATUS_CHANGING_MASK    (0x01UL << 31)
#define IMX_CCM_CLOCK_ROOT_STATUS_CHANGING_SHIFT   (31)
#define IMX_CCM_CLOCK_ROOT_STATUS_SLICE_BUSY_MASK  (0x01UL << 28)
#define IMX_CCM_CLOCK_ROOT_STATUS_SLICE_BUSY_SHIFT (28)
#define IMX_CCM_CLOCK_ROOT_STATUS_POWERDOWN_MASK   (0x01UL << 27)
#define IMX_CCM_CLOCK_ROOT_STATUS_POWERDOWN_SHIFT  (27)
#define IMX_CCM_CLOCK_ROOT_STATUS_OFF_MASK         (0x01UL << 24)      /* Current clock root OFF setting */
#define IMX_CCM_CLOCK_ROOT_STATUS_OFF_SHIFT        (24)
#define IMX_CCM_CLOCK_ROOT_STATUS_MUX_MASK         (0x03UL << 8)       /* Current clock root MUX setting */
#define IMX_CCM_CLOCK_ROOT_STATUS_MUX_SHIFT        (8)
#define IMX_CCM_CLOCK_ROOT_STATUS_DIV_MASK         (0xFFUL << 0)       /* Current clock root DIV setting */
#define IMX_CCM_CLOCK_ROOT_STATUS_DIV_SHIFT        (0)

/* Clock Root Selects for clock root offsets and muxing */
#define IMX_CCM_TARGET_CCM_CKO2                    58U
#define IMX_CCM_TARGET_MEDIA_AXI                   69U
#define IMX_CCM_TARGET_MEDIA_APB                   70U
#define IMX_CCM_TARGET_MEDIA_LDB                   71U
#define IMX_CCM_TARGET_MEDIA_DISP_PIX              72U
#define IMX_CCM_TARGET_MIPI_TEST_BYTE              74U
#define IMX_CCM_TARGET_MIPI_PHY_CFG                75U

/* Clock Gate offsets */
#define IMX_CCM_LPCG_MIPI_DSI                      87U
#define IMX_CCM_LPCG_LVDS                          88U
#define IMX_CCM_LPCG_LCDIF                         89U
#define IMX_CCM_LPCG_PXP                           90U

/*
 * Clock Control Module Analog part (CCM_ANA_TOP)
 */
#define IMX_CCM_ANA_TOP_BASE                       0x44480000
#define IMX_CCM_ANA_TOP_SIZE                       0x10000

/* Offsets of the registers from IMX_CCM_ANA_TOP_BASE */
#define IMX_CCM_ANA_VIDEO_PLL_OFFSET               0x1400

#endif

