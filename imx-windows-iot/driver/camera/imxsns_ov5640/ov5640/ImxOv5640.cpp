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

#include <Ntddk.h>
#include "ImxOv5640.h"
#include "trace.h"
#include "ImxOv5640.tmh"


#define ARRAY_SIZE(param) (sizeof(param)/ sizeof(param[0]))
#define MUX_YUV 0
#define MUX_RGB 1

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * @brief Writes given data to OV5640 camera registers starting at specified address.
 *
 * @param RegAddr  Starting address from which to start loading data into camera.
 * @param ValArray Array of data to be loaded into camera.
 * @param Num Length of ValArray array.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov5640_t::WriteMultiRegs(const UINT16 RegAddr, const UINT8 ValArray[], const UINT32 Num)
{
    NTSTATUS Status;
    PUINT8 p_Bytes = new (NonPagedPoolNx, 'miSH') UINT8[2 + Num];

    p_Bytes[0] = (UINT8)(RegAddr >> 8);
    p_Bytes[1] = (UINT8)(RegAddr & 0xFF);
    RtlCopyMemory(p_Bytes + 2, ValArray, Num);

    Status = m_Camera_res.m_I2c.WriteBytes(p_Bytes, sizeof(2 + Num));
    delete p_Bytes;
    return Status;
};

/**
 * @brief Writes given Address-data array to OV5640 camera registers.
 *
 * @param RegisterValuesArray Array of Address-data to be loaded into camera.
 * @param RegisterValuesArrayLength Length of RegisterValuesArray array.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov5640_t::LoadRegVal(const reg_val_t RegisterValuesArray[], const UINT32 RegisterValuesArrayLength)
{
    UINT32 i;
    NTSTATUS Status = STATUS_SUCCESS;
    LARGE_INTEGER delay{ 0u };

    // TODO: optimize this to a single SPB request;
    for (i = 0; i < RegisterValuesArrayLength; i++) {
        Status = WriteReg(RegisterValuesArray[i].regAddr, RegisterValuesArray[i].regVal);
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "WriteReg %d: 0x%x, 0x%x %!STATUS!", i, RegisterValuesArray[i].regAddr, RegisterValuesArray[i].regVal, Status);
        if (Status) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "ERROR I2C %!STATUS!, i: %d, Reg: 0x%04X, Val: 0x%04X", Status, i, RegisterValuesArray[i].regAddr, RegisterValuesArray[i].regVal);
        }
        if (RegisterValuesArray[i].delay != 0) {
            /* Delay */
            delay.QuadPart = -1 * (int)RegisterValuesArray[i].delay * 10 * 1000;
            Status = KeDelayExecutionThread(KernelMode, false, &delay);
        }
    }

    return Status;
}

/**
 * @brief Restarts the 0V5640 camera.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov5640_t::SoftwareReset()
{
    return WriteReg(0x3008, 0x82);
}

/**
 * @brief Attempts to read and return chip ID of the 0V5640 camera.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov5640_t::GetChipID(UINT16 &ChipID)
{
    NTSTATUS Status = STATUS_SUCCESS;
    UINT8 id[2];

    Status |= ReadReg(0x300A, id[0]);
    Status |= ReadReg(0x300B, id[1]);

    ChipID = (id[0] << 8U) | id[1];
    return Status;
}

// Public interface -------------------------------------------------
/**
 * @brief Initialize the camera device at boot time to safe state.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov5640_t::Init()
{
    NTSTATUS Status;
    LARGE_INTEGER delay{ 0u };
    UINT16 ChipID = 0U;

    if (m_Camera_res.m_HasRstGpio || m_Camera_res.m_HasPwnGpio) {

        if (m_Camera_res.m_HasRstGpio) {
            m_Camera_res.m_RstGpio.Set(0);

            /* Delay approx 20ms. */
            delay.QuadPart = -20 * 10 * 1000;
            KeDelayExecutionThread(KernelMode, false, &delay);
        }

        if (m_Camera_res.m_HasPwnGpio) {
            m_Camera_res.m_PwnGpio.Set(0);

            /* Delay approx 5ms. */
            delay.QuadPart = -5 * 10 * 1000;
            KeDelayExecutionThread(KernelMode, false, &delay);
        }
        if (m_Camera_res.m_HasRstGpio) {
            m_Camera_res.m_RstGpio.Set(1);

            /* Delay approx 20ms. */
            delay.QuadPart = -20 * 10 * 1000;
            KeDelayExecutionThread(KernelMode, false, &delay);
        }
    }

    /* SW reset */
    Status = SoftwareReset();

    /* Delay approx 20ms. */
    delay.QuadPart = -20 * 10 * 1000;
    KeDelayExecutionThread(KernelMode, false, &delay);

    /* Verify Chip ID */
    Status = GetChipID(ChipID);
    if ((Status) || (ChipID != 0x5640)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Invalid camera ID 0x%04X. (Should be 0x5640)", ChipID);
        return STATUS_NO_SUCH_DEVICE;
    }

    return Status;
}

/**
 * @brief Initialize the camera device to 720p 30fps UYUV 2-lane and start streaming.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov5640_t::Configure(camera_config_t *aConfigPtr)
{
    NTSTATUS Status;
    LARGE_INTEGER delay{ 0u };
    UINT8 pixFmt = 0;
    UINT8 rgbYuvMux = 0; // 0 yuv, 1 rgb



    if (aConfigPtr == nullptr) {
        aConfigPtr = &m_Defaults;
    }

    switch (aConfigPtr->cameraPixelFormat)
    {
    case kVIDEO_PixelFormatRGB888:
        rgbYuvMux = MUX_RGB;
        pixFmt = 0x20;
        break;
    case kVIDEO_PixelFormatRGB565:
        rgbYuvMux = MUX_RGB;
        pixFmt = 0x61;
        break;
    case kVIDEO_PixelFormatYUYV:
        rgbYuvMux = MUX_YUV;
        pixFmt = 0x30;
        break;
    case kVIDEO_PixelFormatUYVY:
        rgbYuvMux = MUX_YUV;
        pixFmt = 0x32;
        break;
    default:
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Invalid cameraPixelFormat 0x%x.", aConfigPtr->cameraPixelFormat);
        return STATUS_INVALID_PARAMETER;
    }

    Ov5640_t::reg_val_t InitParam[] = {
        /* ov5640_init_setting_30fps_VGA */
        {0x3103, 0x11, 0, 0}, {0x3008, 0x82, 0, 5}, {0x3008, 0x42, 0, 0},
        {0x3103, 0x03, 0, 0}, {0x3017, 0x00, 0, 0}, {0x3018, 0x00, 0, 0},
        {0x3034, 0x18, 0, 0}, {0x3035, 0x14, 0, 0}, {0x3036, 0x38, 0, 0},
        {0x3037, 0x13, 0, 0}, {0x3108, 0x01, 0, 0}, {0x3630, 0x36, 0, 0},
        {0x3631, 0x0e, 0, 0}, {0x3632, 0xe2, 0, 0}, {0x3633, 0x12, 0, 0},
        {0x3621, 0xe0, 0, 0}, {0x3704, 0xa0, 0, 0}, {0x3703, 0x5a, 0, 0},
        {0x3715, 0x78, 0, 0}, {0x3717, 0x01, 0, 0}, {0x370b, 0x60, 0, 0},
        {0x3705, 0x1a, 0, 0}, {0x3905, 0x02, 0, 0}, {0x3906, 0x10, 0, 0},
        {0x3901, 0x0a, 0, 0}, {0x3731, 0x12, 0, 0}, {0x3600, 0x08, 0, 0},
        {0x3601, 0x33, 0, 0}, {0x302d, 0x60, 0, 0}, {0x3620, 0x52, 0, 0},
        {0x371b, 0x20, 0, 0}, {0x471c, 0x50, 0, 0}, {0x3a13, 0x43, 0, 0},
        {0x3a18, 0x00, 0, 0}, {0x3a19, 0xf8, 0, 0}, {0x3635, 0x13, 0, 0},
        {0x3636, 0x03, 0, 0}, {0x3634, 0x40, 0, 0}, {0x3622, 0x01, 0, 0},
        {0x3c01, 0xa4, 0, 0}, {0x3c04, 0x28, 0, 0}, {0x3c05, 0x98, 0, 0},
        {0x3c06, 0x00, 0, 0}, {0x3c07, 0x08, 0, 0}, {0x3c08, 0x00, 0, 0},
        {0x3c09, 0x1c, 0, 0}, {0x3c0a, 0x9c, 0, 0}, {0x3c0b, 0x40, 0, 0},
        {0x3820, 0x41, 0, 0}, {0x3821, 0x07, 0, 0}, {0x3814, 0x31, 0, 0},
        {0x3815, 0x31, 0, 0}, {0x3800, 0x00, 0, 0}, {0x3801, 0x00, 0, 0},
        {0x3802, 0x00, 0, 0}, {0x3803, 0x04, 0, 0}, {0x3804, 0x0a, 0, 0},
        {0x3805, 0x3f, 0, 0}, {0x3806, 0x07, 0, 0}, {0x3807, 0x9b, 0, 0},
        {0x3808, 0x02, 0, 0}, {0x3809, 0x80, 0, 0}, {0x380a, 0x01, 0, 0},
        {0x380b, 0xe0, 0, 0}, {0x380c, 0x07, 0, 0}, {0x380d, 0x68, 0, 0},
        {0x380e, 0x03, 0, 0}, {0x380f, 0xd8, 0, 0}, {0x3810, 0x00, 0, 0},
        {0x3811, 0x10, 0, 0}, {0x3812, 0x00, 0, 0}, {0x3813, 0x06, 0, 0},
        {0x3618, 0x00, 0, 0}, {0x3612, 0x29, 0, 0}, {0x3708, 0x64, 0, 0},
        {0x3709, 0x52, 0, 0}, {0x370c, 0x03, 0, 0}, {0x3a02, 0x03, 0, 0},
        {0x3a03, 0xd8, 0, 0}, {0x3a08, 0x01, 0, 0}, {0x3a09, 0x27, 0, 0},
        {0x3a0a, 0x00, 0, 0}, {0x3a0b, 0xf6, 0, 0}, {0x3a0e, 0x03, 0, 0},
        {0x3a0d, 0x04, 0, 0}, {0x3a14, 0x03, 0, 0}, {0x3a15, 0xd8, 0, 0},
        {0x4001, 0x02, 0, 0}, {0x4004, 0x02, 0, 0}, {0x3000, 0x00, 0, 0},
        {0x3002, 0x1c, 0, 0}, {0x3004, 0xff, 0, 0}, {0x3006, 0xc3, 0, 0},
        {0x300e, 0x45, 0, 0}, {0x302e, 0x08, 0, 0}, {0x4300, pixFmt, 0, 0},
        {0x501f, rgbYuvMux, 0, 0}, {0x4713, 0x03, 0, 0}, {0x4407, 0x04, 0, 0},
        {0x440e, 0x00, 0, 0}, {0x460b, 0x35, 0, 0}, {0x460c, 0x22, 0, 0},
        {0x4837, 0x0a, 0, 0}, {0x4800, 0x24, 0, 0}, {0x3824, 0x02, 0, 0},
        {0x5000, 0xa7, 0, 0}, {0x5001, 0xa3, 0, 0}, {0x5180, 0xff, 0, 0},
        {0x5181, 0xf2, 0, 0}, {0x5182, 0x00, 0, 0}, {0x5183, 0x14, 0, 0},
        {0x5184, 0x25, 0, 0}, {0x5185, 0x24, 0, 0}, {0x5186, 0x09, 0, 0},
        {0x5187, 0x09, 0, 0}, {0x5188, 0x09, 0, 0}, {0x5189, 0x88, 0, 0},
        {0x518a, 0x54, 0, 0}, {0x518b, 0xee, 0, 0}, {0x518c, 0xb2, 0, 0},
        {0x518d, 0x50, 0, 0}, {0x518e, 0x34, 0, 0}, {0x518f, 0x6b, 0, 0},
        {0x5190, 0x46, 0, 0}, {0x5191, 0xf8, 0, 0}, {0x5192, 0x04, 0, 0},
        {0x5193, 0x70, 0, 0}, {0x5194, 0xf0, 0, 0}, {0x5195, 0xf0, 0, 0},
        {0x5196, 0x03, 0, 0}, {0x5197, 0x01, 0, 0}, {0x5198, 0x04, 0, 0},
        {0x5199, 0x6c, 0, 0}, {0x519a, 0x04, 0, 0}, {0x519b, 0x00, 0, 0},
        {0x519c, 0x09, 0, 0}, {0x519d, 0x2b, 0, 0}, {0x519e, 0x38, 0, 0},
        {0x5381, 0x1e, 0, 0}, {0x5382, 0x5b, 0, 0}, {0x5383, 0x08, 0, 0},
        {0x5384, 0x0a, 0, 0}, {0x5385, 0x7e, 0, 0}, {0x5386, 0x88, 0, 0},
        {0x5387, 0x7c, 0, 0}, {0x5388, 0x6c, 0, 0}, {0x5389, 0x10, 0, 0},
        {0x538a, 0x01, 0, 0}, {0x538b, 0x98, 0, 0}, {0x5300, 0x08, 0, 0},
        {0x5301, 0x30, 0, 0}, {0x5302, 0x10, 0, 0}, {0x5303, 0x00, 0, 0},
        {0x5304, 0x08, 0, 0}, {0x5305, 0x30, 0, 0}, {0x5306, 0x08, 0, 0},
        {0x5307, 0x16, 0, 0}, {0x5309, 0x08, 0, 0}, {0x530a, 0x30, 0, 0},
        {0x530b, 0x04, 0, 0}, {0x530c, 0x06, 0, 0}, {0x5480, 0x01, 0, 0},
        {0x5481, 0x08, 0, 0}, {0x5482, 0x14, 0, 0}, {0x5483, 0x28, 0, 0},
        {0x5484, 0x51, 0, 0}, {0x5485, 0x65, 0, 0}, {0x5486, 0x71, 0, 0},
        {0x5487, 0x7d, 0, 0}, {0x5488, 0x87, 0, 0}, {0x5489, 0x91, 0, 0},
        {0x548a, 0x9a, 0, 0}, {0x548b, 0xaa, 0, 0}, {0x548c, 0xb8, 0, 0},
        {0x548d, 0xcd, 0, 0}, {0x548e, 0xdd, 0, 0}, {0x548f, 0xea, 0, 0},
        {0x5490, 0x1d, 0, 0}, {0x5580, 0x02, 0, 0}, {0x5583, 0x40, 0, 0},
        {0x5584, 0x10, 0, 0}, {0x5589, 0x10, 0, 0}, {0x558a, 0x00, 0, 0},
        {0x558b, 0xf8, 0, 0}, {0x5800, 0x23, 0, 0}, {0x5801, 0x14, 0, 0},
        {0x5802, 0x0f, 0, 0}, {0x5803, 0x0f, 0, 0}, {0x5804, 0x12, 0, 0},
        {0x5805, 0x26, 0, 0}, {0x5806, 0x0c, 0, 0}, {0x5807, 0x08, 0, 0},
        {0x5808, 0x05, 0, 0}, {0x5809, 0x05, 0, 0}, {0x580a, 0x08, 0, 0},
        {0x580b, 0x0d, 0, 0}, {0x580c, 0x08, 0, 0}, {0x580d, 0x03, 0, 0},
        {0x580e, 0x00, 0, 0}, {0x580f, 0x00, 0, 0}, {0x5810, 0x03, 0, 0},
        {0x5811, 0x09, 0, 0}, {0x5812, 0x07, 0, 0}, {0x5813, 0x03, 0, 0},
        {0x5814, 0x00, 0, 0}, {0x5815, 0x01, 0, 0}, {0x5816, 0x03, 0, 0},
        {0x5817, 0x08, 0, 0}, {0x5818, 0x0d, 0, 0}, {0x5819, 0x08, 0, 0},
        {0x581a, 0x05, 0, 0}, {0x581b, 0x06, 0, 0}, {0x581c, 0x08, 0, 0},
        {0x581d, 0x0e, 0, 0}, {0x581e, 0x29, 0, 0}, {0x581f, 0x17, 0, 0},
        {0x5820, 0x11, 0, 0}, {0x5821, 0x11, 0, 0}, {0x5822, 0x15, 0, 0},
        {0x5823, 0x28, 0, 0}, {0x5824, 0x46, 0, 0}, {0x5825, 0x26, 0, 0},
        {0x5826, 0x08, 0, 0}, {0x5827, 0x26, 0, 0}, {0x5828, 0x64, 0, 0},
        {0x5829, 0x26, 0, 0}, {0x582a, 0x24, 0, 0}, {0x582b, 0x22, 0, 0},
        {0x582c, 0x24, 0, 0}, {0x582d, 0x24, 0, 0}, {0x582e, 0x06, 0, 0},
        {0x582f, 0x22, 0, 0}, {0x5830, 0x40, 0, 0}, {0x5831, 0x42, 0, 0},
        {0x5832, 0x24, 0, 0}, {0x5833, 0x26, 0, 0}, {0x5834, 0x24, 0, 0},
        {0x5835, 0x22, 0, 0}, {0x5836, 0x22, 0, 0}, {0x5837, 0x26, 0, 0},
        {0x5838, 0x44, 0, 0}, {0x5839, 0x24, 0, 0}, {0x583a, 0x26, 0, 0},
        {0x583b, 0x28, 0, 0}, {0x583c, 0x42, 0, 0}, {0x583d, 0xce, 0, 0},
        {0x5025, 0x00, 0, 0}, {0x3a0f, 0x30, 0, 0}, {0x3a10, 0x28, 0, 0},
        {0x3a1b, 0x30, 0, 0}, {0x3a1e, 0x26, 0, 0}, {0x3a11, 0x60, 0, 0},
        {0x3a1f, 0x14, 0, 0}, {0x3c00, 0x04, 0, 0},

        /* 30FPS, 1280x720 */
        {0x3035, 0x21, 0, 0}, {0x3036, 0x54, 0, 0}, {0x3c07, 0x07, 0, 0},
        {0x3c09, 0x1c, 0, 0}, {0x3c0a, 0x9c, 0, 0}, {0x3c0b, 0x40, 0, 0},
        {0x3820, 0x41, 0, 0}, {0x3821, 0x07, 0, 0}, {0x3814, 0x31, 0, 0},
        {0x3815, 0x31, 0, 0}, {0x3800, 0x00, 0, 0}, {0x3801, 0x00, 0, 0},
        {0x3802, 0x00, 0, 0}, {0x3803, 0xfa, 0, 0}, {0x3804, 0x0a, 0, 0},
        {0x3805, 0x3f, 0, 0}, {0x3806, 0x06, 0, 0}, {0x3807, 0xa9, 0, 0},
        {0x3808, 0x05, 0, 0}, {0x3809, 0x00, 0, 0}, {0x380a, 0x02, 0, 0},
        {0x380b, 0xd0, 0, 0}, {0x380c, 0x07, 0, 0}, {0x380d, 0x64, 0, 0},
        {0x380e, 0x02, 0, 0}, {0x380f, 0xe4, 0, 0}, {0x3810, 0x00, 0, 0},
        {0x3811, 0x10, 0, 0}, {0x3812, 0x00, 0, 0}, {0x3813, 0x04, 0, 0},
        {0x3618, 0x00, 0, 0}, {0x3612, 0x29, 0, 0}, {0x3708, 0x64, 0, 0},
        {0x3709, 0x52, 0, 0}, {0x370c, 0x03, 0, 0}, {0x3a02, 0x02, 0, 0},
        {0x3a03, 0xe4, 0, 0}, {0x3a08, 0x01, 0, 0}, {0x3a09, 0xbc, 0, 0},
        {0x3a0a, 0x01, 0, 0}, {0x3a0b, 0x72, 0, 0}, {0x3a0e, 0x01, 0, 0},
        {0x3a0d, 0x02, 0, 0}, {0x3a14, 0x02, 0, 0}, {0x3a15, 0xe4, 0, 0},
        {0x4001, 0x02, 0, 0}, {0x4004, 0x02, 0, 0}, {0x4713, 0x02, 0, 0},
        {0x4407, 0x04, 0, 0}, {0x460b, 0x37, 0, 0}, {0x460c, 0x20, 0, 0},
        {0x3824, 0x04, 0, 0}, {0x5001, 0x83, 0, 0}, {0x4005, 0x1a, 0, 0},
        {0x3503, 0x00, 0, 0},

        // For debug only - Uncoment to slow down the FPS by enlarging total horizontal and vertical size.
        // { 0x380C, 0x0b, 0, 0 }, { 0x380D, 0x64, 0, 0 }, /* Total horizontal size */
        // { 0x380E, 0x03, 0, 0 }, { 0x380F, 0xE4, 0, 0 }, /* Total vertical size */

        { 0x300E, 0x45, 0, 25 }, /* MIPI mode, 2-lane mode */
    };

    Status = LoadRegVal(InitParam, ARRAY_SIZE(InitParam));
    if (Status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "OV5640 Configuration failed %!STATUS!.", Status);
    }

    /* Delay 5ms. */
    delay.QuadPart = 5 * 10;
    KeDelayExecutionThread(KernelMode, false, &delay);

    return Status;
}

/**
 * @brief De-initialize the camera device.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov5640_t::Deinit()
{
    NTSTATUS Status = STATUS_SUCCESS;

    Status |= WriteReg(0x3008, 0x42);
    return Status;
}

/**
 * @brief Start the camera device outputting data.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov5640_t::Ov5640_VideoEnable()
{
    NTSTATUS Status;

    /* Disable power down - START video streaming over MIPI CSI
     *
     * Note: Some cameras did not ACK after power_down -> power_up. Slave(camera) releases
     * the SDA line(SDA goes to HIGH) while SCL is HIGH. This is recognized by Master
     * as Stop condition without preceeding ACK. In such case we try to execute power_up command one again. */
    Status = WriteReg(0x3008, 0x02);
    if (Status) {
        /* Repeat power-up again */
        Status = WriteReg(0x3008, 0x02);
    }

    return Status;
}

/**
 * @brief Stop the camera device outputting data.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov5640_t::Stop()
{
    NTSTATUS Status;

    /* Power down MIPI TX PHY */
    Status = WriteReg(0x300E, 0x10);
    /* Power Down */
    Status |= WriteReg(0x3008, 0x42);
    return Status;
}
