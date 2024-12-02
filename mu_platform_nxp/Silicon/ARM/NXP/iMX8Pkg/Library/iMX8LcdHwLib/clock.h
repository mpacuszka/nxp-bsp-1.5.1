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

#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <stdint.h>
#include <stdbool.h>
#include <iMX8.h>
#include <iMXDisplay.h>

#define CCM_TARGET_ROOT_PRE_MAX_DIV   8U
#define CCM_TARGET_ROOT_POST_MAX_DIV  64U
#define CCM_PLL_LOCK_TIMEOUT          10000U

#if defined(CPU_IMX8MP)
    #define CCM_CLK_GATING_REG CCM_CCGR_MEDIA
    #define CCM_TARGET_ROOT_LCDIF_PIXEL_REG       CCM_TARGET_ROOT_MEDIA_DISP1_PIX
    #define CCM_TARGET_ROOT_MIPI_DSI_PHY_REF_REG  CCM_TARGET_ROOT_MEDIA_MIPI_PHY1
    #define LDB_CLK_ROOT_PRESENT
    #define HDMI_PRESENT
#elif defined(CPU_IMX8MM) || defined(CPU_IMX8MN)
    #define CCM_CLK_GATING_REG                    CCM_CCGR_DISPLAY
    #define CCM_TARGET_ROOT_LCDIF_PIXEL_REG       CCM_TARGET_ROOT_LCDIF_PIXEL
    #define CCM_TARGET_ROOT_MIPI_DSI_PHY_REF_REG  CCM_TARGET_ROOT_MIPI_DSI_PHY_REF
    #define MIPI_DSI_CORE_CLK_ROOT_PRESENT
#else
    #error Unsupported target platform
#endif

/* Video PLL parameters */
struct videoPllRateTable {
    uint32_t pixClockRate;
    uint32_t pllRate;
    uint32_t pdiv;
    uint32_t mdiv;
    uint32_t sdiv;
    uint32_t kdiv;
    uint32_t dsiPhyRefRate;
};

/* Initialization macro for Video PLL structure above */
#define PLL_RATE(pix_clock_val, rate_val, m_val, p_val, s_val, k_val, ref_rate_val) \
{ \
    .pixClockRate = (pix_clock_val), \
    .pllRate = (rate_val), \
    .mdiv = (m_val), \
    .pdiv = (p_val), \
    .sdiv = (s_val), \
    .kdiv = (k_val), \
    .dsiPhyRefRate = (ref_rate_val), \
}

EFI_STATUS CcmGetDsiPhyRefClk (IMX_DISPLAY_TIMING* Timing, uint32_t *phyRefClockHz);
EFI_STATUS MipiDsiDisplayClockConfig (IMX_DISPLAY_TIMING *Timing);
EFI_STATUS LdbDisplayClockConfig (IMX_DISPLAY_TIMING *Timing, bool Dual);
VOID HdmiDisplayClockConfig(VOID);

#endif  /* _CLOCK_H_ */
