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

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define OV5640_CHECK_RET(x)            \
    do                                 \
    {                                  \
        status = (x);                  \
        if (!NT_SUCCESS(status)) {     \
            KdPrint(("OV5640::#x %L\r\n", status)); \
            return status;             \
        }                              \
    } while (false);


class Ov5640_t : CameraOperations_t
{
public:
    struct reg_val_t {
        UINT16 regAddr; /*!< Register address. */
        UINT8 regVal;   /*!<Register value. */
        UINT32 delay;   /*!<Delay. */

        reg_val_t(UINT16 &&RegAddr, UINT8 &&RegVal, unsigned &&= 0, unsigned && Delay=0) : regAddr(RegAddr), regVal(RegVal), delay(Delay) {};
        reg_val_t(UINT16 &&RegAddr, const UINT8 &RegVal, unsigned &&= 0, unsigned && Delay=0) : regAddr(RegAddr), regVal(RegVal), delay(Delay) {};
    };

private:
    // Variables --------------------------------
    CamWdf_Res &m_Camera_res;
    camera_config_t m_Defaults;

    // Internal methods -------------------------------
    NTSTATUS WriteReg(const UINT16 RegAddr, const UINT8 Val)
    {
        UINT8 bytes[3]{ (UINT8)(RegAddr >> 8), (UINT8)(RegAddr & 0xFF), (UINT8)(Val) };

        return m_Camera_res.m_I2c.WriteBytes(bytes, sizeof(bytes));
    };

    NTSTATUS LoadRegVal(const reg_val_t Val[], const UINT32 Num);
    NTSTATUS WriteMultiRegs(const UINT16 RegAddr, const UINT8 Val[], const UINT32 Num);

    NTSTATUS ReadReg(const UINT16 RegAddr, UINT8 &Val)
    {
        UINT16 Reg = _byteswap_ushort(RegAddr);
        return m_Camera_res.m_I2c.ReadAddr16(Reg, &Val, sizeof(UINT8));
    }

    NTSTATUS ModifyReg(const UINT16 RegAddr, const UINT8 ClrMask, const UINT8 Val)
    {
        NTSTATUS Status;
        UINT8 val;

        Status = ReadReg(RegAddr, val);
        if (NT_SUCCESS(Status)) {
            val = (val & ~ClrMask) | Val;
            Status = WriteReg(RegAddr, val);
        }
        return Status;
    }


    NTSTATUS SoftwareReset();
    NTSTATUS GetChipID(UINT16 &chipID);

public:
    Ov5640_t(CamWdf_Res &CameraRes) : m_Camera_res(CameraRes), m_Defaults{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 25, 1, 2} {};

    // Public interface -------------------------------------------------
    NTSTATUS Init();

    NTSTATUS Configure(camera_config_t *aConfigPtr = nullptr);
    NTSTATUS Ov5640_VideoEnable();
    NTSTATUS Deinit();

    NTSTATUS Stop();
};
