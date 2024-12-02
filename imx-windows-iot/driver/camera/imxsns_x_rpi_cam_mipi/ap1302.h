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

#define AP1302_CHIP_ID                  0x265
#define AP1302_CHIP_REV                 0x0206
#define AP1302_I2C_MAX_LEN              65534
#define AP1302_FW_WINDOW_OFFSET         0x8000
#define AP1302_FW_WINDOW_SIZE           0x2000

#define AP1302_REG_CHIP_VERSION         0x0000
#define AP1302_REG_CHIP_REV             0x0050
#define AP1302_REG_MF_ID                0x0004
#define AP1302_REG_ERROR                0x0006
#define AP1302_REG_CTRL                 0x1000
#define AP1302_REG_DZ_TGT_FCT           0x1010
#define AP1302_REG_SFX_MODE             0x1016
#define AP1302_REG_SS_HEAD_PT0          0x1174
#define AP1302_REG_ATOMIC               0x1184
#define AP1302_REG_PREVIEW_WIDTH        0x2000
#define AP1302_REG_PREVIEW_HEIGHT       0x2002
#define AP1302_REG_AE_BV_OFF            0x5014
#define AP1302_REG_AE_BV_BIAS           0x5016
#define AP1302_REG_AF_MODE              0x5058
#define AP1302_REG_AWB_CTRL             0x5100
#define AP1302_REG_FLICK_CTRL           0x5440
#define AP1302_REG_SCENE_CTRL           0x5454
#define AP1302_REG_BOOTDATA_STAGE       0x6002
#define AP1302_REG_SENSOR_SELECT        0x600C
#define AP1302_REG_SYS_START            0x601A
#define AP1302_REG_BOOTDATA_CHECKSUM    0x6134
#define AP1302_REG_SIP_CRC              0xF052

/* AP1302 FW header*/
struct ap1302_fw_header {
	UINT32 crc;
	UINT32 bootDataChksum;
	UINT32 pllInitSize;
	UINT32 totalSize;
};
