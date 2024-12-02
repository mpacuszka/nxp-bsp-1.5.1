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
#if (!defined(CPU_IMX8MP) && !defined(CPU_IMX8QXP) && !defined(CPU_IMX93))
#include "iMX8HdmiCdn.h"

VOID HdmiCdnGeneralReadRegister(UINT32 address, UINT32 *value)
{
    UINT32 value32, respAddress32, length;

    // Read CDN register OpCode
    IMX_HDMICDN_APB_WRITE8(IMX_HDMICDN_GENERAL_READ_REGISTER);
    // Use general module ID
    IMX_HDMICDN_APB_WRITE8(IMX_HDMICDN_MB_MODULE_ID_GENERAL);
    // Length 4
    IMX_HDMICDN_APB_WRITE16(0x0004);
    // Write address
    IMX_HDMICDN_APB_WRITE32(address);
    // Response
    // Operation OpCode
    IMX_HDMICDN_APB_READ(value32);
    if (value32 != IMX_HDMICDN_GENERAL_READ_REGISTER) {
        DEBUG ((DEBUG_ERROR, "Rsponse of OpCode is wrong!\n"));
    }
    // Module ID
    IMX_HDMICDN_APB_READ8(value32);
    if (value32 != IMX_HDMICDN_MB_MODULE_ID_GENERAL) {
        DEBUG ((DEBUG_ERROR, "Module response ID value is wrong!\n"));
    }
    // Length
    IMX_HDMICDN_APB_READ16(length);
    if (length != 8) {
        DEBUG ((DEBUG_ERROR, "esponse length %u is wrong!\n"));
    }
    // Read response address
    IMX_HDMICDN_APB_READ32(respAddress32);
    if (respAddress32 != address) {
        DEBUG ((DEBUG_ERROR, "Module response address value is wrong!\n"));
    }
    // Read response value
    IMX_HDMICDN_APB_READ32(*value);

    DEBUG((DEBUG_INFO, "HDMI cadence read general register, addr 0x%x, val: 0x%x\n", address, *value));
}

VOID HdmiCdnGeneralWriteRegister(UINT32 address, UINT32 value)
{
    DEBUG((DEBUG_INFO, "HDMI cadence write general register, addr 0x%x, val: 0x%x\n", address, value));

    // Read CDN register
    IMX_HDMICDN_APB_WRITE8(IMX_HDMICDN_GENERAL_WRITE_REGISTER);
    IMX_HDMICDN_APB_WRITE8(IMX_HDMICDN_MB_MODULE_ID_GENERAL);
    // Length 8
    IMX_HDMICDN_APB_WRITE16(0x0008);
    // Write address
    IMX_HDMICDN_APB_WRITE32(address);
    // Write value
    IMX_HDMICDN_APB_WRITE32(value);
}
#endif

VOID HdmiCdnOn(VOID)
{
#if (!defined(CPU_IMX8MP) && !defined(CPU_IMX8QXP) && !defined(CPU_IMX93))
    UINT32 value32;

    DEBUG((DEBUG_INFO, "Turning On HDMI cadence \n"));
    HdmiCdnGeneralReadRegister(IMX_HDMICDN_ADDR_SOURCE_MHL_HD + IMX_HDMICDN_HDTX_CONTROLLER,
        &value32);
    value32 |= (IMX_HDMICDN_HDTX_CONTROLLER_F_DATA_EN);  // Enable HDMI
    HdmiCdnGeneralWriteRegister(IMX_HDMICDN_ADDR_SOURCE_MHL_HD + IMX_HDMICDN_HDTX_CONTROLLER,
        value32);
#endif
}

VOID HdmiCdnOff(VOID)
{
#if (!defined(CPU_IMX8MP) && !defined(CPU_IMX8QXP) && !defined(CPU_IMX93))
    UINT32 value32;

    DEBUG((DEBUG_INFO, "Turning Off HDMI cadence \n"));

    HdmiCdnGeneralReadRegister(IMX_HDMICDN_ADDR_SOURCE_MHL_HD + IMX_HDMICDN_HDTX_CONTROLLER,
                               &value32);
    value32 &= ~(IMX_HDMICDN_HDTX_CONTROLLER_F_DATA_EN);  // Disable HDMI
    HdmiCdnGeneralWriteRegister(IMX_HDMICDN_ADDR_SOURCE_MHL_HD + IMX_HDMICDN_HDTX_CONTROLLER,
                               value32);
#endif
}
