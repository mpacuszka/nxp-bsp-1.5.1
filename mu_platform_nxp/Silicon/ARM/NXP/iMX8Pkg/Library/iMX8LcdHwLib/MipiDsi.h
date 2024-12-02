/*
* Copyright 2020, 2022 NXP
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

#ifndef _MIPI_DSI_H_
#define _MIPI_DSI_H_

#include <stdint.h>
#include <stdbool.h>
#include <iMX8.h>
#include <iMXDisplay.h>
#include "iMX8LcdHwLib.h"

/* Fixed clock rates and limits */
#define DSI_MAX_ESC_CLOCK_KHZ         20000
#define DSI_MAX_HS_BIT_CLOCK_KHZ      2100000
#define DSI_MIN_HS_BIT_CLOCK_KHZ      80000
#define DSI_MIN_PLL_FVCO_HZ           1050000000UL
#define DSI_MAX_PLL_FVCO_HZ           2100000000UL
#define DSI_MIN_PLL_P                 1U
#define DSI_MAX_PLL_P                 4U
#define DSI_MIN_PLL_S                 0U
#define DSI_MAX_PLL_S                 5U
#define DSI_MIN_PLL_M                 64UL
#define DSI_MAX_PLL_M                 1023UL

#define DSI_HFP_PKT_OVERHEAD          6U
#define DSI_HBP_PKT_OVERHEAD          6U
#define DSI_HSA_PKT_OVERHEAD          6U

/* Subparts of PMS field in DSI_PLLCTRL */
#define DSI_PLLCTRL_P_SHIFT           (13)
#define DSI_PLLCTRL_P_MASK            (0x7FU << DSI_PLLCTRL_P_SHIFT)
#define DSI_PLLCTRL_P(v)              (((v) << DSI_PLLCTRL_P_SHIFT) & DSI_PLLCTRL_P_MASK)
#define DSI_PLLCTRL_M_SHIFT           (3)
#define DSI_PLLCTRL_M_MASK            (0x3FFU << DSI_PLLCTRL_M_SHIFT)
#define DSI_PLLCTRL_M(v)              (((v) << DSI_PLLCTRL_M_SHIFT) & DSI_PLLCTRL_M_MASK)
#define DSI_PLLCTRL_S_SHIFT           (0)
#define DSI_PLLCTRL_S_MASK            (0x7U << DSI_PLLCTRL_S_SHIFT)
#define DSI_PLLCTRL_S(v)              (((v) << DSI_PLLCTRL_S_SHIFT) & DSI_PLLCTRL_S_MASK)

/* MIPI DSI timing parameters */
struct dsiHblankPar {
    uint32_t hactive;
    uint32_t vactive;
    uint32_t refresh;
    uint32_t hfpWC;
    uint32_t hbpWC;
    uint32_t hsaWC;
    uint32_t lanes;
};

/* Initialization macro for MIPI DSI timing parameters structure above */
#define DSI_HBLANK_PARAM(hact, vact, rfrsh, hfp, hbp, hsa, num) \
    .hactive  = (hact), \
    .vactive  = (vact), \
    .refresh  = (rfrsh), \
    .hfpWC    = (hfp), \
    .hbpWC    = (hbp), \
    .hsaWC    = (hsa), \
    .lanes    = (num)

VOID CcmClockDump(VOID);
VOID MipiDsiReset(VOID);
EFI_STATUS MipiDsiConfig(IMX_DISPLAY_TIMING* Timing, imxConverter MipiDsiConvertor);
VOID MipiDsiDump(VOID);

#endif  /* _MIPI_DSI_H_ */
