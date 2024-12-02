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

#ifndef _LCDIFV3_H_
#define _LCDIFV3_H_

#include <stdint.h>
#include <stdbool.h>
#include <iMX8.h>
#include <iMXDisplay.h>

/* LCDIF devices */
typedef uint32_t Lcdifv3_Device;
/* Available devices defined for Lcdifv3_Device type */
#define  LCDIF1_DEV 0U
#define  LCDIF2_DEV 1U
#define  LCDIF3_DEV 2U
#define  LCDIFMAX_DEV (LCDIF3_DEV + 1) /* Always should be number of devices + 1 */

BOOLEAN EFIAPI Is_Lcdifv3_Enabled(Lcdifv3_Device Dev);
EFI_STATUS Lcdifv3_Power_Down(Lcdifv3_Device Dev);
EFI_STATUS Lcdifv3_Reset(Lcdifv3_Device Dev);
EFI_STATUS Lcdifv3_Enable(Lcdifv3_Device Dev, bool Enable);
EFI_STATUS Lcdifv3_Init(Lcdifv3_Device Dev, uintptr_t FrameBuffer);
EFI_STATUS Lcdifv3_SetTimingMode(Lcdifv3_Device Dev, IMX_DISPLAY_TIMING* Timing);
EFI_STATUS Lcdifv3_Dump(Lcdifv3_Device Dev);

#endif  /* _LCDIF_H_ */
