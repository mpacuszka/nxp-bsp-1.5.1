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

#if (!DBG)
#define DbgPrintEx(...)
#endif // !DBG


#define OV10635_CHECK_RET(x)            \
    do                                 \
    {                                  \
        status = (x);                  \
        if (!NT_SUCCESS(status)) {     \
            KdPrint(("OV10635::#x %L\r\n", status)); \
            return status;             \
        }                              \
    } while (false);


#define IMX_MAX9286_I2C_DEVICE_ADDRESS           0x6A
#define IMX_MAX9271_DEFAULT_I2C_DEVICE_ADDRESS   0x40
#define IMX_MAX9271_BROADCAST_I2C_DEVICE_ADDRESS 0x45
#define IMX_OV10635_DEFAULT_I2C_DEVICE_ADDRESS   0x30

#define IMX_MAX9286_CONFIGDET(idx) (1 << ((idx) + 4))
#define IMX_MAX9286_VIDEODET(idx)  (1 << (idx))

#define IMX_MAX9286_CSI_LANE_CNT(x)        (((x) - 1) << 6)
#define IMX_MAX9286_DATA_TYPE_RGB888    (0 << 0)
#define IMX_MAX9286_DATA_TYPE_RGB565    (1 << 0)
#define IMX_MAX9286_DATA_TYPE_YUV422_8_BIT    (3 << 0)

/* Max camera links */
#define IMX_MAX9286_CAMERAS_CNT 4U

/* Enum type to address each camera link separatelly */
enum CHNL_IDX {
    CHNL_0 = 0,
    CHNL_1,
    CHNL_2,
    CHNL_3,
    CHNL_BROADCAST,
    CHNL_DEFAULT,
};

class Ov10635_t : CameraOperations_t
{
public:
    struct reg_val_t {
        UINT16 regAddr; /*!< Register address. */
        UINT8 regVal;   /*!<Register value. */
        UINT32 delay;   /*!<Delay. */

        reg_val_t(UINT16 &&RegAddr, UINT8 &&RegVal, unsigned && Delay=0) : regAddr(RegAddr), regVal(RegVal), delay(Delay) {};
        reg_val_t(UINT16 &&RegAddr, const UINT8 &RegVal, unsigned && Delay=0) : regAddr(RegAddr), regVal(RegVal), delay(Delay) {};
    };

private:
    // Variables --------------------------------
    CamWdf_Res &m_Camera_res;
    camera_config_t m_Defaults;

    bool reverseChnlConfigured = false;
    bool cameraInitialized[IMX_MAX9286_CAMERAS_CNT] = { false, false, false, false };
    bool cameraChainInitialized = false;
    UINT8 Max9271UniqueAddresses[IMX_MAX9286_CAMERAS_CNT] = { 0x41, 0x42, 0x43, 0x44 }; //Those I2C addresses must be aligned with the ACPI!!
    UINT8 Ov10635UniqueAddresses[IMX_MAX9286_CAMERAS_CNT] = { 0x31, 0x32, 0x33, 0x34 }; //Those I2C addresses must be aligned with the ACPI!!

    // Internal methods -------------------------------

    /* MAX9286 I2C functions */
    NTSTATUS ReadRegMax9286(const UINT8 RegAddr, UINT8 &Val)
    {
        return m_Camera_res.m_I2cMax9286.ReadAddr8(RegAddr, &Val, sizeof(UINT8));
    }

    NTSTATUS WriteRegMax9286(const UINT8 RegAddr, const UINT8 Val)
    {
        UINT8 bytes[2]{ RegAddr, Val };
        return m_Camera_res.m_I2cMax9286.WriteBytes(bytes, sizeof(bytes));
    };

    /* MAX9271 I2C Read/Write functions */
    NTSTATUS ReadRegMax9271(const enum CHNL_IDX ChipIdx, const UINT8 RegAddr, UINT8& Val)
    {
        NTSTATUS Status = STATUS_SUCCESS;

        switch (ChipIdx) {
        case CHNL_0:
            Status = m_Camera_res.m_I2cMax9271_0.ReadAddr8(RegAddr, &Val, sizeof(UINT8));
            break;
        case CHNL_1:
            Status = m_Camera_res.m_I2cMax9271_1.ReadAddr8(RegAddr, &Val, sizeof(UINT8));
            break;
        case CHNL_2:
            Status = m_Camera_res.m_I2cMax9271_2.ReadAddr8(RegAddr, &Val, sizeof(UINT8));
            break;
        case CHNL_3:
            Status = m_Camera_res.m_I2cMax9271_3.ReadAddr8(RegAddr, &Val, sizeof(UINT8));
            break;
        case CHNL_DEFAULT:
            Status = m_Camera_res.m_I2cMax9271.ReadAddr8(RegAddr, &Val, sizeof(UINT8));
            break;
        default:
            Status = STATUS_INVALID_PARAMETER_1;
            break;
        }
        return Status;
    }

    NTSTATUS WriteRegMax9271(const enum CHNL_IDX ChipIdx, const UINT8 RegAddr, const UINT8 Val)
    {
        NTSTATUS Status = STATUS_SUCCESS;
        UINT8 bytes[2]{ RegAddr, Val };
        switch (ChipIdx) {
        case CHNL_0:
            Status = m_Camera_res.m_I2cMax9271_0.WriteBytes(bytes, sizeof(bytes));
            break;
        case CHNL_1:
            Status = m_Camera_res.m_I2cMax9271_1.WriteBytes(bytes, sizeof(bytes));
            break;
        case CHNL_2:
            Status = m_Camera_res.m_I2cMax9271_2.WriteBytes(bytes, sizeof(bytes));
            break;
        case CHNL_3:
            Status = m_Camera_res.m_I2cMax9271_3.WriteBytes(bytes, sizeof(bytes));
            break;
        case CHNL_BROADCAST:
            Status = m_Camera_res.m_I2cMax9271_Broadcast.WriteBytes(bytes, sizeof(bytes));
            break;
        case CHNL_DEFAULT:
            Status = m_Camera_res.m_I2cMax9271.WriteBytes(bytes, sizeof(bytes));
            break;
        default:
            Status = STATUS_INVALID_PARAMETER_1;
            break;
        }

        return Status;
    };

    /* OV10635 I2C Read/Write functions */
    NTSTATUS ReadRegOv10635(const enum CHNL_IDX CameraIdx, const UINT16 RegAddr, UINT8& Val)
    {
        NTSTATUS Status = STATUS_SUCCESS;
        UINT16 Reg = _byteswap_ushort(RegAddr);

        switch (CameraIdx) {
        case CHNL_0:
            Status = m_Camera_res.m_I2cOv10635_0.ReadAddr16(Reg, &Val, sizeof(UINT8));
            break;
        case CHNL_1:
            Status = m_Camera_res.m_I2cOv10635_1.ReadAddr16(Reg, &Val, sizeof(UINT8));
            break;
        case CHNL_2:
            Status = m_Camera_res.m_I2cOv10635_2.ReadAddr16(Reg, &Val, sizeof(UINT8));
            break;
        case CHNL_3:
            Status = m_Camera_res.m_I2cOv10635_3.ReadAddr16(Reg, &Val, sizeof(UINT8));
            break;
        case CHNL_DEFAULT:
            Status = m_Camera_res.m_I2cOv10635.ReadAddr16(Reg, &Val, sizeof(UINT8));
            break;
        default:
            Status = STATUS_INVALID_PARAMETER_1;
            break;
        }
        return Status;
    }

    NTSTATUS WriteRegOv10635(const enum CHNL_IDX CameraIdx, const UINT16 RegAddr, const UINT8 Val)
    {
        NTSTATUS Status = STATUS_SUCCESS;
        UINT8 bytes[3]{ (UINT8)(RegAddr >> 8), (UINT8)(RegAddr & 0xFF), (UINT8)(Val) };

        switch (CameraIdx) {
        case CHNL_0:
            Status = m_Camera_res.m_I2cOv10635_0.WriteBytes(bytes, sizeof(bytes));
            break;
        case CHNL_1:
            Status = m_Camera_res.m_I2cOv10635_1.WriteBytes(bytes, sizeof(bytes));
            break;
        case CHNL_2:
            Status = m_Camera_res.m_I2cOv10635_2.WriteBytes(bytes, sizeof(bytes));
            break;
        case CHNL_3:
            Status = m_Camera_res.m_I2cOv10635_3.WriteBytes(bytes, sizeof(bytes));
            break;
        case CHNL_DEFAULT:
            Status = m_Camera_res.m_I2cOv10635.WriteBytes(bytes, sizeof(bytes));
            break;
        default:
            Status = STATUS_INVALID_PARAMETER_1;
            break;
        }
        return Status;
    };

    /* Delay function */
    NTSTATUS DelayMs(UINT32 ms)
    {
        KeStallExecutionProcessor(ms * 1000);
        return STATUS_SUCCESS;
    };

    NTSTATUS Ov10635_t::DumpMax9286();
    NTSTATUS Ov10635_t::DumpMax9271();
    NTSTATUS Max9286_GetChipID(UINT8&chipID);
    NTSTATUS Max9271_GetChipID(UINT8& chipID);

public:
    Ov10635_t(CamWdf_Res &CameraRes) : m_Camera_res(CameraRes), m_Defaults{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 30, 1, 2} {};

    /* Public interface */
    bool GetCameraInitStatus()
    {
        return cameraChainInitialized;
    }
    NTSTATUS Init();
    NTSTATUS Ov10635_Init(camera_config_t* aConfigPtr = nullptr);
    NTSTATUS Max_Init(camera_config_t *aConfigPtr = nullptr);
    NTSTATUS Max_VideoEnable();
    NTSTATUS LinkProbe();
    NTSTATUS Deinit();
    NTSTATUS Stop();
};
