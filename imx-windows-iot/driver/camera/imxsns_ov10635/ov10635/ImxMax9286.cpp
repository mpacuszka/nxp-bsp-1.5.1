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
#include "ImxOv10635.h"
#include "trace.h"
#include "ImxOv10635.tmh"


#define ARRAY_SIZE(param) (sizeof(param)/ sizeof(param[0]))
#define MUX_YUV 0
#define MUX_RGB 1

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * @brief Writes given data to OV10635 camera registers starting at specified address.
 *
 * @param RegAddr  Starting address from which to start loading data into camera.
 * @param ValArray Array of data to be loaded into camera.
 * @param Num Length of ValArray array.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov10635_t::WriteMultiRegs(const UINT16 RegAddr, const UINT8 ValArray[], const UINT32 Num)
{
    NTSTATUS Status;
    PUINT8 p_Bytes = new (NonPagedPoolNx, 'miSH') UINT8[2 + Num];

    p_Bytes[0] = (UINT8)(RegAddr >> 8);
    p_Bytes[1] = (UINT8)(RegAddr & 0xFF);
    RtlCopyMemory(p_Bytes + 2, ValArray, Num);

    Status = m_Camera_res.m_I2cMax9286.WriteBytes(p_Bytes, sizeof(2 + Num));
    delete p_Bytes;
    return Status;
};

/**
 * @brief Writes given Address-data array to OV10635 camera registers.
 *
 * @param RegisterValuesArray Array of Address-data to be loaded into camera.
 * @param RegisterValuesArrayLength Length of RegisterValuesArray array.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov10635_t::LoadRegVal(const reg_val_t RegisterValuesArray[], const UINT32 RegisterValuesArrayLength)
{
    UINT32 i;
    NTSTATUS Status = STATUS_SUCCESS;
    LARGE_INTEGER delay{ 0u };

    // TODO: optimize this to a single SPB request;
    for (i = 0; i < RegisterValuesArrayLength; i++) {
        Status = WriteReg(RegisterValuesArray[i].regAddr, RegisterValuesArray[i].regVal);
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "WriteReg %d: 0x%x, 0x%x %!STATUS!", i, RegisterValuesArray[i].regAddr, RegisterValuesArray[i].regVal, Status);
        if (Status) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "ERROR I2C 0x%08X\n", Status);
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "i: %d\n", i);
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "Reg: 0x%04X\n", RegisterValuesArray[i].regAddr);
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "Val: 0x%04X\n", RegisterValuesArray[i].regVal);
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
 * @brief Restarts the 0V10635 camera.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov10635_t::SoftwareReset()
{
    NTSTATUS Status = STATUS_SUCCESS;

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "OV10635::SoftwareReset\n");

    return Status;
}

/**
 * @brief Attempts to read and return chip ID of the MAX9286 deserializer.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov10635_t::Max9286_GetChipID(UINT8 &ChipID)
{
    NTSTATUS Status = STATUS_SUCCESS;
    UINT8 id = 0;

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "OV10635::Max9286_GetChipID\n");
    Status = ReadRegMax9286(0x1E, id);

    ChipID = id;
    return Status;
}

/**
 * @brief Attempts to read and return chip ID of the MAX9271 serializer.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov10635_t::Max9271_GetChipID(UINT8 &ChipID)
{
    NTSTATUS Status = STATUS_SUCCESS;
    UINT8 id = 0;

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "OV10635::Max9271_GetChipID\n");
    Status = ReadRegMax9271(0x1E, id);

    ChipID = id;
    return Status;
}

// Public interface -------------------------------------------------
/**
 * @brief Initialize the camera device at boot time to safe state.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov10635_t::Init()
{
    NTSTATUS Status;
    LARGE_INTEGER delay{ 0u };
    UINT8 ChipID = 0U;

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "OV10635::Init\n");

    if (m_Camera_res.m_HasRstGpio) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "Clear CSI_nRST to LOW to assert reset\n");
        m_Camera_res.m_RstGpio.Set(0);
    }

    /* Delay approx 20ms. */
    delay.QuadPart = -20 * 10 * 1000;
    KeDelayExecutionThread(KernelMode, false, &delay);

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "Clear CSI_PWDN to LOW to power UP the camera\n");
    m_Camera_res.m_PwnGpio.Set(0);

    /* Delay approx 5ms. */
    delay.QuadPart = -5 * 10 * 1000;
    KeDelayExecutionThread(KernelMode, false, &delay);

    if (m_Camera_res.m_HasRstGpio) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "Set CSI_nRST to HIGH to de-assert reset\n");
        m_Camera_res.m_RstGpio.Set(1);
    }

    /* Delay approx 20ms. */
    delay.QuadPart = -20 * 10 * 1000;
    KeDelayExecutionThread(KernelMode, false, &delay);

    /* SW reset */
    //Status = SoftwareReset();

    /* Delay approx 20ms. */
    delay.QuadPart = -20 * 10 * 1000;
    KeDelayExecutionThread(KernelMode, false, &delay);

    /* Verify MAX9286 Chip ID */
    Status = Max9286_GetChipID(ChipID);
    if ((Status) || (ChipID != 0x40)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "Invalid MAX9286 device ID 0x%04X. (Should be 0x40)\n", ChipID);
        return STATUS_NO_SUCH_DEVICE;
    }



    return Status;
}

/**
 * @brief Initialize the camera device to 720p 30fps UYUV 2-lane and start streaming.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov10635_t::Configure(camera_config_t *aConfigPtr)
{
    NTSTATUS Status = STATUS_SUCCESS;
    LARGE_INTEGER delay{ 0u };
    UINT8 pixFmt = 0;
    UINT8 rgbYuvMux = 0; // 0 yuv, 1 rgb

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "OV10635::Configure\n");


    if (aConfigPtr == nullptr) {
        aConfigPtr = &m_Defaults;
    }

    switch (aConfigPtr->cameraPixelFormat)
    {
    case kVIDEO_PixelFormatRGB888:
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "kVIDEO_PixelFormatRGB888\n");
        rgbYuvMux = MUX_RGB;
        pixFmt = 0x20;
        break;
    case kVIDEO_PixelFormatRGB565:
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "kVIDEO_PixelFormatRGB565\n");
        rgbYuvMux = MUX_RGB;
        pixFmt = 0x61;
        break;
    case kVIDEO_PixelFormatYUYV:
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "kVIDEO_PixelFormatYUYV\n");
        rgbYuvMux = MUX_YUV;
        pixFmt = 0x30;
        break;
    case kVIDEO_PixelFormatUYVY:
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "kVIDEO_PixelFormatUYVY\n");
        rgbYuvMux = MUX_YUV;
        pixFmt = 0x32;
        break;
    default:
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "kVIDEO_Invalid: 0x%x\r\n", aConfigPtr->cameraPixelFormat);
        return STATUS_INVALID_PARAMETER;
    }

    /* Verify MAX9271 Chip ID */
    Status = Max9271_GetChipID(ChipID);
    if ((Status) || (ChipID != 0x09)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "Invalid MAX9271 device ID 0x%04X. (Should be 0x40)\n", ChipID);
        return STATUS_NO_SUCH_DEVICE;
    }

    Ov10635_t::reg_val_t InitParam[] = {

        { 0x300E, 0x45, 0, 25 }
    };

    //Status = LoadRegVal(InitParam, ARRAY_SIZE(InitParam));
    if (Status) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "OV10635 Configuration failed %d\n", Status);
    }

    /* Delay 5ms. */
    delay.QuadPart = 5 * 10;
    KeDelayExecutionThread(KernelMode, false, &delay);

    //Status = WriteReg(0x3008, 0x02);

    return Status;
}

/**
 * @brief De-initialize the camera device.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov10635_t::Deinit()
{
    NTSTATUS Status = STATUS_SUCCESS;

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "OV10635::Deinit\n");

    //Status |= WriteReg(0x3008, 0x42);

    return Status;
}

/**
 * @brief Stop the camera device outputting data.
 *
 * @return Returns @ref STATUS_SUCCESS if success, otherwise returns error code.
 */
NTSTATUS Ov10635_t::Stop()
{
    NTSTATUS Status = STATUS_SUCCESS;

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, 0xFFFFFFFF, "OV10635::STOP\n");

    ///* Power down MIPI TX PHY */
    //Status = WriteReg(0x300E, 0x10);
    ///* Power Down */
    //Status |= WriteReg(0x3008, 0x42);

    return Status;
}
