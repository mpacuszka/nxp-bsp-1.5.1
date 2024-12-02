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

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if (!DBG)
#define DbgPrintEx(...)
#endif // !DBG

#define CHECK_RET(x)            \
    do                                 \
    {                                  \
        status = (x);                  \
        if (!NT_SUCCESS(status)) {     \
            KdPrint(("X-RPI-CAM_MIPI::#x %L\r\n", status)); \
            return status;             \
        }                              \
    } while (false);


class XRpiCamMipi_t : CameraOperations_t
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

    /* Delay function */
    NTSTATUS DelayMs(UINT32 ms)
    {
        KeStallExecutionProcessor(ms * 1000);
        return STATUS_SUCCESS;
    };

    // Variables --------------------------------
    CamWdf_Res &m_Camera_res;
    camera_config_t m_Defaults;

    // Internal methods -------------------------------
    NTSTATUS Adp5585BBWriteReg(const UINT8 regAddr, const UINT8 val);
    NTSTATUS Adp5585BBReadReg(const UINT8 regAddr, UINT8& val);
    NTSTATUS Adp5585WriteReg(const UINT8 regAddr, const UINT8 val);
    NTSTATUS Adp5585ReadReg(const UINT8 regAddr, UINT8& val);
    NTSTATUS Adp5585Init();

    
    NTSTATUS Ap1302Reset();
    NTSTATUS Ap1302LoadFw(const UINT16 regAddr, const UINT8 buffer[], const UINT32 len);
    NTSTATUS Ap1302WriteReg(const UINT16 regAddr, const UINT16 val);
    NTSTATUS Ap1302ReadReg(const UINT16 regAddr, UINT16& val);
    NTSTATUS Ap1302WriteFwWindow(UINT8 *buffer, UINT32& bufPos, UINT32 len);

public:
    XRpiCamMipi_t(CamWdf_Res &CameraRes) : m_Camera_res(CameraRes), m_Defaults{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 25, 1, 2} {};

    // Public interface -------------------------------------------------
    NTSTATUS Init();
    NTSTATUS Configure(camera_config_t *aConfigPtr = nullptr);
    NTSTATUS Stop();
    NTSTATUS Deinit();
};
