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

#ifndef _LCDIFV3_REGISTERS_H_
#define _LCDIFV3_REGISTERS_H_

/* Register offsets */
#define LCDIFV3_CTRL               0x00
#define LCDIFV3_CTRL_SET           0x04
#define LCDIFV3_CTRL_CLR           0x08
#define LCDIFV3_CTRL_TOG           0x0c
#define LCDIFV3_DISP_PARA          0x10
#define LCDIFV3_DISP_SIZE          0x14
#define LCDIFV3_HSYN_PARA          0x18
#define LCDIFV3_VSYN_PARA          0x1c
#define LCDIFV3_VSYN_HSYN_WIDTH    0x20
#define LCDIFV3_INT_STATUS_D0      0x24
#define LCDIFV3_INT_ENABLE_D0      0x28
#define LCDIFV3_INT_STATUS_D1      0x30
#define LCDIFV3_INT_ENABLE_D1      0x34
#define LCDIFV3_CTRLDESCL0_1       0x200
#define LCDIFV3_CTRLDESCL0_3       0x208
#define LCDIFV3_CTRLDESCL_LOW0_4   0x20c
#define LCDIFV3_CTRLDESCL_HIGH0_4  0x210
#define LCDIFV3_CTRLDESCL0_5       0x214
#define LCDIFV3_CSC0_CTRL          0x21c
#define LCDIFV3_CSC0_COEF0         0x220
#define LCDIFV3_CSC0_COEF1         0x224
#define LCDIFV3_CSC0_COEF2         0x228
#define LCDIFV3_CSC0_COEF3         0x22c
#define LCDIFV3_CSC0_COEF4         0x230
#define LCDIFV3_CSC0_COEF5         0x234
#define LCDIFV3_PANIC0_THRES       0x238

/* Bit fields */
#define CTRL_SW_RESET                     (1U << 31)
#define CTRL_INV_PXCK                     (1U << 3)
#define CTRL_INV_DE                       (1U << 2)
#define CTRL_INV_VS                       (1U << 1)
#define CTRL_INV_HS                       (1U << 0)

#define DISP_PARA_DISP_ON                 (1U << 31)
#define DISP_PARA_LINE_PATTERN(x)         (((x) & 0xf) << 26)
#define DISP_PARA_DISP_MODE(x)            (((x) & 0x3) << 24)

#define DISP_SIZE_DELTA_Y(x)              (((x) & 0xffff) << 16)
#define DISP_SIZE_DELTA_X(x)              ((x) & 0xffff)

#define HSYN_PARA_BP_H(x)                 (((x) & 0xffff) << 16)
#define HSYN_PARA_FP_H(x)                 ((x) & 0xffff)

#define VSYN_PARA_BP_V(x)                 (((x) & 0xffff) << 16)
#define VSYN_PARA_FP_V(x)                 ((x) & 0xffff)

#define VSYN_HSYN_WIDTH_PW_V(x)           (((x) & 0xffff) << 16)
#define VSYN_HSYN_WIDTH_PW_H(x)           ((x) & 0xffff)

#define INT_STATUS_D0_VS_BLANK            (1U << 2)

#define INT_ENABLE_D1_PLANE_PANIC_EN      (1U << 0)

#define CTRLDESCL0_1_HEIGHT(x)            (((x) & 0xffff) << 16)
#define CTRLDESCL0_1_WIDTH(x)             ((x) & 0xffff)

#define CTRLDESCL0_3_STATE_CLEAR_VSYNC                 (1U << 23)
#define CTRLDESCL0_3_P_SIZE(x)                         (((x) & 0x7) << 20)
#define CTRLDESCL0_3_T_SIZE(x)                         (((x) & 0x7) << 16)
#define CTRLDESCL0_3_PITCH(x)             ((x) & 0xffff)

#define CTRLDESCL0_5_EN                   (1U << 31)
#define CTRLDESCL0_5_SHADOW_LOAD_EN       (1U << 30)
#define CTRLDESCL0_5_BPP(x)               (((x) & 0xf) << 24)
#define CTRLDESCL0_5_YUV_FORMAT(x)        (((x) & 0x3) << 14)

#define PANIC0_THRES_PANIC_THRES_LOW(x)   (((x) & 0x1ff) << 16)
#define PANIC0_THRES_PANIC_THRES_HIGH(x)  ((x) & 0x1ff)

#endif  /* _LCDIFV3_REGISTERS_H_ */
