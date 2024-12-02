/*
* Copyright 2022-2023 NXP
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

#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include "Lcdifv3.h"
#include "Lcdifv3Registers.h"
#include "iMX8LcdHwLib.h"

/* layer encoding formats (input) */
#define BPP16_RGB565    0x4
#define BPP16_ARGB1555  0x5
#define BPP16_ARGB4444  0x6
#define BPP16_YCbCr422  0x7
#define BPP24_RGB888    0x8
#define BPP32_ARGB8888  0x9
#define BPP32_ABGR8888  0xa

#define LCDIFV3_DBG_LVL DEBUG_INFO

/* Base addresses of LCDIF peripherals */
LCDIF_MemMapPtr DP[LCDIFMAX_DEV] = LCDIF_BASE_PTRS;

/**
  Return LCDIF enable status.
**/
BOOLEAN EFIAPI
Is_Lcdifv3_Enabled(Lcdifv3_Device Dev)
{
    if (Dev >= LCDIFMAX_DEV) {
        return FALSE;
    }
    uint32_t reg = MmioRead32((uint64_t)DP[Dev] + LCDIFV3_DISP_PARA);
    return (BOOLEAN)((reg & DISP_PARA_DISP_ON) != 0);
}

/**
  Power down LCDIF block.
**/
EFI_STATUS
Lcdifv3_Power_Down(Lcdifv3_Device Dev)
{
    if (Dev >= LCDIFMAX_DEV) {
        return EFI_DEVICE_ERROR;
    }

    uint64_t base = (uint64_t)DP[Dev];
    int timeout = 1000000;

    /* Disable LCDIF during VBLANK */
    MmioWrite32(base + LCDIFV3_INT_STATUS_D0, INT_STATUS_D0_VS_BLANK);
    while (--timeout) {
        if (MmioRead32(base + LCDIFV3_INT_STATUS_D0) & INT_STATUS_D0_VS_BLANK) {
            break;
        }
        MicroSecondDelay(1);
    }

    /* dma off */
    uint32_t ctrldescl0_5 = MmioRead32(base + LCDIFV3_CTRLDESCL0_5);
    ctrldescl0_5 &= ~CTRLDESCL0_5_EN;
    MmioWrite32(base + LCDIFV3_CTRLDESCL0_5, ctrldescl0_5);

    /* disp off */
    uint32_t disp_para = MmioRead32(base + LCDIFV3_DISP_PARA);
    disp_para &= ~DISP_PARA_DISP_ON;
    MmioWrite32(base + LCDIFV3_DISP_PARA, disp_para);

    return EFI_SUCCESS;
}

/**
  Reset LCDIF block.
**/
EFI_STATUS
Lcdifv3_Reset (
    Lcdifv3_Device Dev
    )
{
    if (Dev >= LCDIFMAX_DEV) {
        return EFI_DEVICE_ERROR;
    }

    uint64_t base = (uint64_t)DP[Dev];

    /* LCDIF reset */
    MmioWrite32(base + LCDIFV3_CTRL_CLR, CTRL_SW_RESET);
    while (MmioRead32(base + LCDIFV3_CTRL) & CTRL_SW_RESET);

    MmioWrite32(base + LCDIFV3_CTRL_SET, CTRL_SW_RESET);
    while (!(MmioRead32(base + LCDIFV3_CTRL) & CTRL_SW_RESET));

    MmioWrite32(base + LCDIFV3_CTRL_CLR, CTRL_SW_RESET);
    while (MmioRead32(base + LCDIFV3_CTRL) & CTRL_SW_RESET);

    return EFI_SUCCESS;
}

/**
  LCDIF enable/disable.

  @param enable Enable/Disable bool flag.
**/
EFI_STATUS
Lcdifv3_Enable (
    Lcdifv3_Device Dev,
    bool Enable
    )
{
    uint32_t loop = 2000U;
    uint32_t disp, ctrl;

    if (Dev >= LCDIFMAX_DEV) {
        return EFI_DEVICE_ERROR;
    }
    uint64_t base = (uint64_t)DP[Dev];

    disp = MmioRead32(base + LCDIFV3_DISP_PARA);
    ctrl = MmioRead32(base + LCDIFV3_CTRLDESCL0_5);

    if (Enable) {
        /* disp on */
        disp |= DISP_PARA_DISP_ON;
        MmioWrite32(base + LCDIFV3_DISP_PARA, disp);
        /* enable shadow load */
        ctrl |= CTRLDESCL0_5_SHADOW_LOAD_EN;
        MmioWrite32(base + LCDIFV3_CTRLDESCL0_5, ctrl);
        /* enable layer dma */
        ctrl |= CTRLDESCL0_5_EN;
        MmioWrite32(base + LCDIFV3_CTRLDESCL0_5, ctrl);
    } else {
        /* disable layer dma */
        ctrl &= ~CTRLDESCL0_5_EN;
        ctrl |= CTRLDESCL0_5_SHADOW_LOAD_EN;
        MmioWrite32(base + LCDIFV3_CTRLDESCL0_5, ctrl);
        /* dma config takes effect at the end of frame,
        so add delay to wait dma disable done before turn off disp. */
        while (--loop) {
            if (MmioRead32(base + LCDIFV3_INT_STATUS_D0) & INT_STATUS_D0_VS_BLANK) {
                break;
            }
            MicroSecondDelay(10);
        }
        disp &= ~DISP_PARA_DISP_ON;
        MmioWrite32(base + LCDIFV3_DISP_PARA, disp);
    }

    return EFI_SUCCESS;
}

/**
  Basic LCDIF init not related to the timing mode.

  @param FrameBuffer Physical address of the SW framebuffer.
**/
EFI_STATUS
Lcdifv3_Init (
    Lcdifv3_Device Dev,
    uintptr_t FrameBuffer
    )
{
    uint32_t panic;
    if ((Dev >= LCDIFMAX_DEV) || (FrameBuffer == 0)) {
        return EFI_DEVICE_ERROR;
    }
    uint64_t base = (uint64_t)DP[Dev];

    /* the thres_low should be 1/3 FIFO, that is 511/3 = 171
     * and thres_high should be 2/3 FIFO, that is 511*2/3 = 340
     */
    panic = PANIC0_THRES_PANIC_THRES_LOW(171) | PANIC0_THRES_PANIC_THRES_HIGH(341);
    MmioWrite32(base + LCDIFV3_PANIC0_THRES, panic);
    /* Enable Panic */
    MmioWrite32(base + LCDIFV3_INT_ENABLE_D1, INT_ENABLE_D1_PLANE_PANIC_EN);
    /* Disable other interrupts */
    MmioWrite32(base + LCDIFV3_INT_ENABLE_D0, 0);

    /* Set address of the frame buffer */
    MmioWrite32(base + LCDIFV3_CTRLDESCL_LOW0_4, FrameBuffer);

    return EFI_SUCCESS;
}

/**
  LCDIF configuration into required timing mode.

  @param Timing Pointer to structure containing detailed timing information.
**/
EFI_STATUS
Lcdifv3_SetTimingMode (
    Lcdifv3_Device Dev,
    IMX_DISPLAY_TIMING* Timing
    )
{
    uint32_t disp_size, hsyn_para, vsyn_para, vsyn_hsyn_width, ctrldescl0_1, polarities;
    uint32_t disp_para = 0;
    uint32_t ctrldescl0_3 = 0;
    uint32_t ctrldescl0_5 = 0;

    if ((Dev >= LCDIFMAX_DEV) || (Timing == NULL)) {
        return EFI_DEVICE_ERROR;
    }
    uint64_t base = (uint64_t)DP[Dev];

    /* config display timings */
    disp_size = DISP_SIZE_DELTA_Y(Timing->VActive) | DISP_SIZE_DELTA_X(Timing->HActive);
    MmioWrite32(base + LCDIFV3_DISP_SIZE, disp_size);

    hsyn_para = HSYN_PARA_BP_H(Timing->HBlank - Timing->HSyncOffset - Timing->HSync) |
                HSYN_PARA_FP_H(Timing->HSyncOffset);
    MmioWrite32(base + LCDIFV3_HSYN_PARA, hsyn_para);

    vsyn_para = VSYN_PARA_BP_V(Timing->VBlank - Timing->VSyncOffset - Timing->VSync) |
                VSYN_PARA_FP_V(Timing->VSyncOffset);
    MmioWrite32(base + LCDIFV3_VSYN_PARA, vsyn_para);

    vsyn_hsyn_width = VSYN_HSYN_WIDTH_PW_V(Timing->VSync)
                      | VSYN_HSYN_WIDTH_PW_H(Timing->HSync);
    MmioWrite32(base + LCDIFV3_VSYN_HSYN_WIDTH, vsyn_hsyn_width);

    /* config layer size */
    ctrldescl0_1 = CTRLDESCL0_1_HEIGHT(Timing->VActive) | CTRLDESCL0_1_WIDTH(Timing->HActive);
    MmioWrite32(base + LCDIFV3_CTRLDESCL0_1, ctrldescl0_1);

    /* Polarities */
    polarities = (CTRL_INV_HS | CTRL_INV_VS |
                  /* SEC MIPI DSI specific */
                  CTRL_INV_PXCK | CTRL_INV_DE);
    MmioWrite32(base + LCDIFV3_CTRL_CLR, polarities);

    /* Set output bus format */
    disp_para = MmioRead32(base + LCDIFV3_DISP_PARA);
    disp_para &= DISP_PARA_LINE_PATTERN(0xf);
    /* 24 bits output (LP_RGB888_OR_YUV444) */
    disp_para |= DISP_PARA_LINE_PATTERN(0);
    /* config display mode: default is normal mode */
    disp_para &= DISP_PARA_DISP_MODE(3);
    disp_para |= DISP_PARA_DISP_MODE(0);
    MmioWrite32(base + LCDIFV3_DISP_PARA, disp_para);

    /* Set input bus format */
    ctrldescl0_5 = MmioRead32(base + LCDIFV3_CTRLDESCL0_5);
    ctrldescl0_5 &= ~(CTRLDESCL0_5_BPP(0xf) | CTRLDESCL0_5_YUV_FORMAT(0x3));
    switch (Timing->PixelFormat) {
        case PIXEL_FORMAT_ARGB32:
            ctrldescl0_5 |= CTRLDESCL0_5_BPP(BPP32_ARGB8888);
            break;
        default:
            DEBUG ((DEBUG_ERROR, "Unsupported pixel format: %u\n", Timing->PixelFormat));
            return EFI_DEVICE_ERROR;
    }
    MmioWrite32(base + LCDIFV3_CTRLDESCL0_5, ctrldescl0_5);

    /* Config P_SIZE and T_SIZE:
     * 1. P_SIZE and T_SIZE should never
     *    be less than AXI bus width.
     * 2. P_SIZE should never be less than T_SIZE.
     */
    ctrldescl0_3 |= CTRLDESCL0_3_P_SIZE(2);
    ctrldescl0_3 |= CTRLDESCL0_3_T_SIZE(2);

    /* Config pitch */
    ctrldescl0_3 |= CTRLDESCL0_3_PITCH(Timing->HActive *
            LcdGetBytesPerPixel(Timing->PixelFormat));

    /* Enable frame clear to clear FIFO data on
     * every vsync blank period to make sure no
     * dirty data exits to affect next frame
     * display, otherwise some flicker issue may
     * be observed in some cases.
     */
    ctrldescl0_3 |= CTRLDESCL0_3_STATE_CLEAR_VSYNC;

    MmioWrite32(base + LCDIFV3_CTRLDESCL0_3, ctrldescl0_3);

    return EFI_SUCCESS;
}

/**
  Dump LCDIF registers.
**/
EFI_STATUS
Lcdifv3_Dump(
    Lcdifv3_Device Dev
    )
{
    if (Dev >= LCDIFMAX_DEV) {
        return EFI_DEVICE_ERROR;
    }
    uint64_t base = (uint64_t)DP[Dev];

    DebugPrint(LCDIFV3_DBG_LVL, "------------------------LCDIFv3------------------------\n");
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CTRL              = 0x%08X\n", MmioRead32(base + LCDIFV3_CTRL));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_DISP_PARA         = 0x%08X\n", MmioRead32(base + LCDIFV3_DISP_PARA));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_DISP_SIZE         = 0x%08X\n", MmioRead32(base + LCDIFV3_DISP_SIZE));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_HSYN_PARA         = 0x%08X\n", MmioRead32(base + LCDIFV3_HSYN_PARA));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_VSYN_PARA         = 0x%08X\n", MmioRead32(base + LCDIFV3_VSYN_PARA));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_VSYN_HSYN_WIDTH   = 0x%08X\n", MmioRead32(base + LCDIFV3_VSYN_HSYN_WIDTH));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_INT_STATUS_D0     = 0x%08X\n", MmioRead32(base + LCDIFV3_INT_STATUS_D0));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_INT_ENABLE_D0     = 0x%08X\n", MmioRead32(base + LCDIFV3_INT_ENABLE_D0));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_INT_STATUS_D1     = 0x%08X\n", MmioRead32(base + LCDIFV3_INT_STATUS_D1));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_INT_ENABLE_D1     = 0x%08X\n", MmioRead32(base + LCDIFV3_INT_ENABLE_D1));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CTRLDESCL0_1      = 0x%08X\n", MmioRead32(base + LCDIFV3_CTRLDESCL0_1));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CTRLDESCL0_3      = 0x%08X\n", MmioRead32(base + LCDIFV3_CTRLDESCL0_3));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CTRLDESCL_LOW0_4   = 0x%08X\n", MmioRead32(base + LCDIFV3_CTRLDESCL_LOW0_4));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CTRLDESCL_HIGH0_4 = 0x%08X\n", MmioRead32(base + LCDIFV3_CTRLDESCL_HIGH0_4));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CTRLDESCL0_5       = 0x%08X\n", MmioRead32(base + LCDIFV3_CTRLDESCL0_5));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CSC0_CTRL         = 0x%08X\n", MmioRead32(base + LCDIFV3_CSC0_CTRL));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CSC0_COEF0        = 0x%08X\n", MmioRead32(base + LCDIFV3_CSC0_COEF0));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CSC0_COEF1        = 0x%08X\n", MmioRead32(base + LCDIFV3_CSC0_COEF1));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CSC0_COEF2        = 0x%08X\n", MmioRead32(base + LCDIFV3_CSC0_COEF2));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CSC0_COEF3        = 0x%08X\n", MmioRead32(base + LCDIFV3_CSC0_COEF3));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CSC0_COEF4        = 0x%08X\n", MmioRead32(base + LCDIFV3_CSC0_COEF4));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_CSC0_COEF5        = 0x%08X\n", MmioRead32(base + LCDIFV3_CSC0_COEF5));
    DebugPrint(LCDIFV3_DBG_LVL, "LCDIF_PANIC0_THRES      = 0x%08X\n", MmioRead32(base + LCDIFV3_PANIC0_THRES));
    DebugPrint(LCDIFV3_DBG_LVL, "-----------------------------------------------------\n");

    return EFI_SUCCESS;
}
