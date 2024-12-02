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

#include <Ntddk.h>
#include "imx_x_rpi_cam_mipi.h"
#include "trace.h"
#include "imx_x_rpi_cam_mipi.tmh"
#include "adp5585.h"
#include "ap1302.h"
#include "Device.h"

#define ARRAY_SIZE(param) (sizeof(param)/ sizeof(param[0]))
#define MUX_YUV 0
#define MUX_RGB 1

/*******************************************************************************
 * Code
 ******************************************************************************/

 /**
  * @brief Write UINT8 value into ADP5585 I/O expander on imx93 base board.
  *
  * @param regAddr Register address.
  * @param val     Value to write.
  *
  * @return Returns STATUS_SUCCESS, otherwise error code.
  */
NTSTATUS XRpiCamMipi_t::Adp5585BBWriteReg(const UINT8 regAddr, const UINT8 val)
{
    UINT8 data[2]{ regAddr, val };
    return m_Camera_res.m_I2c2ResIdAdp5585.WriteBytes(data, sizeof(data));
};

/**
 * @brief Read UINT8 value from ADP5585 I/O expander on imx93 base board.
 *
 * @param regAddr Register address.
 * @param val     Value read from the register.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Adp5585BBReadReg(const UINT8 regAddr, UINT8& val)
{
    return m_Camera_res.m_I2c2ResIdAdp5585.ReadAddr8(regAddr, &val, sizeof(UINT8));
}

/**
 * @brief Write UINT8 value into ADP5585 I/O expander on X-RPI-CAM-MIPI board.
 *
 * @param regAddr Register address.
 * @param val     Value to write.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Adp5585WriteReg(const UINT8 regAddr, const UINT8 val)
{
    UINT8 data[2]{ regAddr, val };
    return m_Camera_res.m_I2c1ResIdAdp5585.WriteBytes(data, sizeof(data));
};

/**
 * @brief Read UINT8 value from ADP5585 I/O expander on X-RPI-CAM-MIPI board.
 *
 * @param regAddr Register address.
 * @param val     Value read from the register.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Adp5585ReadReg(const UINT8 regAddr, UINT8& val)
{
    return m_Camera_res.m_I2c1ResIdAdp5585.ReadAddr8(regAddr, &val, sizeof(UINT8));
}

/**
 * @brief Function performs I2C icremental write into AP1302. First 2 bytes are used for
 *        address of the register to start from, following are values to write into ISP.
 *
 * @param regAddr Starting address of the ISP register.
 * @param buffer  Data to write.
 * @param len     Data length.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Ap1302LoadFw(const UINT16 regAddr, const UINT8 buffer[], const UINT32 len)
{
    NTSTATUS status;
    PUINT8 data = new (NonPagedPoolNx, 'beef') UINT8[2 + len];

    data[0] = (UINT8)(regAddr >> 8);
    data[1] = (UINT8)(regAddr & 0xFF);
    RtlCopyMemory(data + 2, buffer, len);

    status = m_Camera_res.m_I2c1ResIdAp1302.WriteBytes(data, 2 + len);
    delete data;
    return status;
};

/**
 * @brief Write UINT16 value into AP1302 ISP.
 *
 * @param regAddr Register address.
 * @param val     Value to write.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Ap1302WriteReg(const UINT16 regAddr, const UINT16 val)
{
    UINT8 data[4];
    data[0] = regAddr >> 8U;
    data[1] = regAddr & 0xFF;
    data[2] = val >> 8U;
    data[3] = val & 0xFF;

    return m_Camera_res.m_I2c1ResIdAp1302.WriteBytes(data, sizeof(data));
};

/**
 * @brief Read UINT16 register from AP1302 ISP.
 *
 * @param regAddr Register address.
 * @param val     Value read from the register.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Ap1302ReadReg(const UINT16 regAddr, UINT16& val)
{
    NTSTATUS status;
    UINT16 reg = _byteswap_ushort(regAddr);
    UINT8 data[2];

    status =m_Camera_res.m_I2c1ResIdAp1302.ReadAddr16(reg, &data[0], sizeof(data));
    val = ((UINT16)data[0] << 8U) | ((UINT16)data[1]);
    return status;
}

/**
 * @brief AP1302 ISP HW reset.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Ap1302Reset()
{
    UINT8 reg = 0U;
    UINT8 val = 0U;
    NTSTATUS status;

    reg = ADP5585_GPO_DATA_OUT_A_REG;
    status = Adp5585BBReadReg(reg, val);

    val &= ~EXP_R0_CSI_RST;
    status |= Adp5585BBWriteReg(reg, val);
    DelayMs(5);

    val |= EXP_R0_CSI_RST;
    status |= Adp5585BBWriteReg(reg, val);
    DelayMs(20);

    return status;
}
/**
 * @brief Initialize the camera device at boot time to safe state.
 *
 * @param buffer Buffer containing FW data.
 * @param bufPos Possition within the buffer from what to start.
 * @param len    Length of the FW window to load into the ISP.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Ap1302WriteFwWindow(UINT8* buffer, UINT32& bufPos, UINT32 len)
{
    NTSTATUS status = STATUS_SUCCESS;
    UINT32 pos;
    UINT32 subLen;

#ifdef DBG
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "Len = %d (0x%04X)", len, len);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "bufPos [Start] = %d (0x%08X)", bufPos, bufPos);
#endif
    for (pos = 0; pos < len; pos += subLen) {
        if (len - pos < AP1302_FW_WINDOW_SIZE - bufPos) {
            subLen = len - pos;
        } else {
            subLen = AP1302_FW_WINDOW_SIZE - bufPos;
        }
#ifdef DBG
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "subLen = %d", subLen);
#endif
        /* Write data into ISP */
        status = XRpiCamMipi_t::Ap1302LoadFw(AP1302_FW_WINDOW_OFFSET + (UINT16)bufPos, buffer + pos, subLen);
        if (status) {
            TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to write AP1302 Window, Status = 0x%08X", status);
            return status;
        }

        bufPos += subLen;
        if (bufPos >= AP1302_FW_WINDOW_SIZE) {
#ifdef DBG
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "bufPos = 0x%04X", bufPos);
#endif
            bufPos = 0;
        }
    }
#ifdef DBG
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "bufPos [End] = %d (0x%08X)", bufPos, bufPos);
#endif
    return status;
}


// Public interface -------------------------------------------------
/**
 * @brief Initialize the camera device at boot time to safe state.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Init()
{
    NTSTATUS                status = STATUS_SUCCESS;
    UNICODE_STRING          uniName;
    OBJECT_ATTRIBUTES       objAttr;
    HANDLE                  handle;
    IO_STATUS_BLOCK         ioStatusBlock;
    LARGE_INTEGER           byteOffset;
    struct ap1302_fw_header fwHeader;
    UINT8*                  buffer;
    UINT32                  bufPos = 0;
    UINT16                  val16 = 0U;

    /* Initialize ADP5585 and enable all required regulators for the ISPp */
    XRpiCamMipi_t::Adp5585Init(); 

    /* Reset AP1302 ISP */
    status = XRpiCamMipi_t::Ap1302Reset();
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to reset ISP AP1302");
        return STATUS_NO_SUCH_DEVICE;
    }

    /* Verify AP1302 chip ID */
    status = Ap1302ReadReg(AP1302_REG_CHIP_VERSION, val16);
    if (status || val16 != AP1302_CHIP_ID) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Invalid AP1302 Chip version 0x%04X. (Should be 0x%04X)", AP1302_CHIP_ID, val16);
        return STATUS_NO_SUCH_DEVICE;
    }
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "AP1302 Chip ID: 0x%04X", val16);

    /* Verify AP1302 chip revision */
    status = Ap1302ReadReg(AP1302_REG_CHIP_REV, val16);
    if (status || val16 != AP1302_CHIP_REV) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to read AP1302 Chip revision, Status = 0x%08X", status);
        return STATUS_NO_SUCH_DEVICE;
    }
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "AP1302 Chip revision: 0x%04X", val16);

    /* 4) Open ap1302.fw */
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "Open ap1302.fw");

    RtlInitUnicodeString(&uniName, L"\\SystemRoot\\System32\\ap1302.fw");
    InitializeObjectAttributes(&objAttr, &uniName,
        OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE,
        NULL, NULL);

    /* Open FW file */
    status = ZwCreateFile(&handle,
        GENERIC_READ,
        &objAttr, &ioStatusBlock,
        NULL,
        FILE_ATTRIBUTE_NORMAL,
        0,
        FILE_OPEN,
        FILE_SYNCHRONOUS_IO_NONALERT,
        NULL, 0);

    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to open ap1302.fw, Status = 0x%08X", status);
        goto Fail0;
    }

    /* Read FW header */
    byteOffset.LowPart = byteOffset.HighPart = 0;
    status = ZwReadFile(handle,
        NULL,
        NULL,
        NULL,
        &ioStatusBlock,
        &fwHeader,
        sizeof(struct ap1302_fw_header),
        &byteOffset,
        NULL);
    
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to get FW header, Status = 0x%08X", status);
        goto Fail1;
    }

#ifdef DBG
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "ap1302.fw Bootdata checksum  = 0x%08X", fwHeader.bootDataChksum);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "ap1302.fw CRC                = 0x%08X", fwHeader.crc);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "ap1302.fw PLL init data size = 0x%08X", fwHeader.pllInitSize);
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "ap1302.fw Total FW size      = 0x%08X (%d bytes)", fwHeader.totalSize, fwHeader.totalSize);
#endif

    /* Allocate memmory for the FW content */
    buffer = (UINT8*)ExAllocatePoolWithTag(NonPagedPoolNx, sizeof(UINT8) * fwHeader.totalSize, IMX_X_RPI_CAM_MIPI_0_POOL_TAG);
    if (buffer == NULL) // check alloc status
    {
        status = STATUS_INSUFFICIENT_RESOURCES;
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Out of memory, Status = 0x%08X", status);
        goto Fail1;
    }

    /* Read the rest of the FW into internal buffer */
    byteOffset.LowPart = sizeof(struct ap1302_fw_header);
    byteOffset.HighPart = 0;
    status = ZwReadFile(handle,
        NULL,
        NULL,
        NULL,
        &ioStatusBlock,
        buffer,
        fwHeader.totalSize,
        &byteOffset,
        NULL);

    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Failed to read FW file, Status = 0x%08X", status);
        goto Fail1;
    }

#ifdef DBG
    for (UINT32 i = 0; i < fwHeader.totalSize; i+=16) {
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "Addr[0x%08X]: 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X", i,
                                                                  *(UINT16 *)(&buffer[i]), *(UINT16*)(&buffer[i + 2]), *(UINT16*)(&buffer[i + 4]), *(UINT16*)(&buffer[i + 6]), 
                                                                  *(UINT16*)(&buffer[i + 8]), *(UINT16*)(&buffer[i + 10]), *(UINT16*)(&buffer[i + 12]), *(UINT16*)(&buffer[i + 14]));
    }
#endif

    /* Set AF mode */
    status = XRpiCamMipi_t::Ap1302WriteReg(0x5058, 0x0186);

    /* Clear ISP CRC register */
    status = XRpiCamMipi_t::Ap1302WriteReg(AP1302_REG_SIP_CRC, 0xFFFF);
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to clear AP1302 CRC register, Status = 0x%08X", status);
        goto Fail2;
    }

    /* Load PLL Init Data */
    status = Ap1302WriteFwWindow(buffer, bufPos, fwHeader.pllInitSize);
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to write PLL init data, Status = 0x%08X", status);
        goto Fail2;
    }

    /* Enable PLL by write into bootdata_stage */
    status = XRpiCamMipi_t::Ap1302WriteReg(AP1302_REG_BOOTDATA_STAGE, 0x0002);
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to write AP1302 BOOTDATA_STAGE register, Status = 0x%08X", status);
        goto Fail2;
    }

    /* Wait for PLL lock */
    DelayMs(20);

    /* Load next bootdata stage */
    status = Ap1302WriteFwWindow(buffer + fwHeader.pllInitSize, bufPos, fwHeader.totalSize - fwHeader.pllInitSize);
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to write the rest of the FW, Status = 0x%08X", status);
        goto Fail2;
    }

    /* Confirm FW load is complete */
    status = XRpiCamMipi_t::Ap1302WriteReg(AP1302_REG_BOOTDATA_STAGE, 0xFFFF);
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to write AP1302 BOOTDATA_STAGE register, Status = 0x%08X", status);
        goto Fail2;
    }

    /* Wait 50ms after FW load */
    DelayMs(50);

    /* Read and verify checksum */
    status = XRpiCamMipi_t::Ap1302ReadReg(AP1302_REG_BOOTDATA_CHECKSUM, val16);
    if (status || (val16 != fwHeader.bootDataChksum)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to read AP1302 BOOTDATA_CHEKSUM register, Status = 0x%08X", status);
        goto Fail2;
    }
    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "AP1302 BOOTDATA_CHEKSUM = 0x%04X", val16);

    /* Write undocumented register */
    status = XRpiCamMipi_t::Ap1302WriteReg(0x6124, 0x0001);
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to write 0x6124 register, Status = 0x%08X", status);
        goto Fail2;
    }

    /* Set auto focus mode */
    status = XRpiCamMipi_t::Ap1302WriteReg(AP1302_REG_AF_MODE, 0x0186);
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unable to enable auto focus mode, Status = 0x%08X", status);
        goto Fail2;
    }

    /* Disable MIPI CSI2 data stream outputting from AP1302 */
    status = XRpiCamMipi_t::Ap1302WriteReg(0x601A, 0x0080);
    status = XRpiCamMipi_t::Ap1302WriteReg(0x601A, 0x0180);

Fail2:
    ExFreePoolWithTag(buffer, IMX_X_RPI_CAM_MIPI_0_POOL_TAG);
Fail1:
    ZwClose(handle);
Fail0:
    return status;
}

/**
 * @brief Power UP the AP1302 ISP. Enable all required regulators and voltage level selectors using ADP5585 I/O expander.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Adp5585Init()
{
    NTSTATUS Status = STATUS_SUCCESS;
    UINT8 Data[2];

    /* 1) Clear all outputs */
    Data[0] = 0x00;
    Data[1] = 0x00;
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_A_REG, Data[0]);
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_B_REG, Data[1]);

    /* Write direction regs */
    Data[0] = (CAM_EXP_R0_DVDD_SEL | CAM_EXP_R1_VAA_SEL | CAM_EXP_R2_ISP_BYP | CAM_EXP_R3_FLED_TOR_TX | CAM_EXP_R4_VCM_PWREN | CAM_EXP_R5_ISP_STBY);
    Data[1] = (CAM_EXP_C0_PWREN_1 | CAM_EXP_C1_PWREN_2 | CAM_EXP_C2_PWREN_3 | CAM_EXP_C3_PWREN_4 | CAM_EXP_C4_PWREN_5);
    Status |= Adp5585WriteReg(ADP5585_GPIO_DIRECTION_A_REG, Data[0]);
    Status |= Adp5585WriteReg(ADP5585_GPIO_DIRECTION_B_REG, Data[1]);

    DelayMs(20);

    /* 2) Select the ISP video path by default */
    Data[0] = CAM_EXP_R2_ISP_BYP;
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_A_REG, Data[0]);

    DelayMs(5);

    Data[0] |= (CAM_EXP_R0_DVDD_SEL | CAM_EXP_R1_VAA_SEL);
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_A_REG, Data[0]);

    DelayMs(5);

    /* Enable DVDD_1V2 */
    Data[1] =  CAM_EXP_C1_PWREN_2;
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_B_REG, Data[1]);

    DelayMs(5);

    /* Enable ISP_STBY signal*/
    Data[0] |= (CAM_EXP_R3_FLED_TOR_TX | CAM_EXP_R4_VCM_PWREN | CAM_EXP_R5_ISP_STBY);
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_A_REG, Data[0]);

    /* Enable VDDIO_1V8 */
    Data[1] |= CAM_EXP_C0_PWREN_1;
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_B_REG, Data[1]);

    DelayMs(5);

    /* Enable ISP_IOVDD_HMISC_1V8 */
    Data[1] |= CAM_EXP_C4_PWREN_5;
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_B_REG, Data[1]);

    DelayMs(5);

    /* Enable AVDD_2V7 */
    Data[1] |=CAM_EXP_C2_PWREN_3;
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_B_REG, Data[1]);

    DelayMs(5);
#if 0
    /* We dont need to enable this volatge regulator, but keep it in the code for visibility */
    /* Enable AVDD_2V8 */
    Data[1] |= CAM_EXP_C3_PWREN_4;
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_B_REG, Data[1]);

    DelayMs(5);
#endif
    /* Clear ISP_STBY */
    Data[0] &= ~CAM_EXP_R5_ISP_STBY;
    Status |= Adp5585WriteReg(ADP5585_GPO_DATA_OUT_A_REG, Data[0]);

    return Status;
}

/**
 * @brief Initialize the camera device to 720p 30fps UYUV 2-lane and start streaming.
 *
 * @param aConfigPtr Video parameters to configure.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Configure(camera_config_t *aConfigPtr)
{
    NTSTATUS status = STATUS_SUCCESS;

    if (aConfigPtr == nullptr) {
        aConfigPtr = &m_Defaults;
    }

    /* Enable */
    status = XRpiCamMipi_t::Ap1302WriteReg(0x601A, 0x0080);
    status |= XRpiCamMipi_t::Ap1302WriteReg(0x601A, 0x0380);

    if (aConfigPtr->resolution == kVIDEO_Resolution720P) {
        status = XRpiCamMipi_t::Ap1302WriteReg(AP1302_REG_ATOMIC, 0x01);
        status |= XRpiCamMipi_t::Ap1302WriteReg(AP1302_REG_PREVIEW_WIDTH, FSL_VIDEO_EXTRACT_WIDTH(aConfigPtr->resolution));
        status |= XRpiCamMipi_t::Ap1302WriteReg(AP1302_REG_PREVIEW_HEIGHT, FSL_VIDEO_EXTRACT_HEIGHT(aConfigPtr->resolution));
        status |= XRpiCamMipi_t::Ap1302WriteReg(AP1302_REG_ATOMIC, 0x0B);
    } else {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Unsupported resolution requested");
        status = STATUS_INVALID_PARAMETER;
    }
    DelayMs(50); /* ISP AP1302 requires some delay between START/STOP operation */

    return status;
}

/**
 * @brief Stop the camera device outputting data.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Stop()
{
    NTSTATUS Status = STATUS_SUCCESS;

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "%s() Stop the ISP", __FUNCTION__);

    /* Disable */
    Status = XRpiCamMipi_t::Ap1302WriteReg(0x601A, 0x0080);
    Status |= XRpiCamMipi_t::Ap1302WriteReg(0x601A, 0x0180);

    if (Status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Error while stopping the ISP, Status=0x%08X", Status);
    }
    DelayMs(50); /* ISP AP1302 requires some delay between START/STOP operation */

    return Status;
}

/**
 * @brief De-initialize the camera device.
 *
 * @return Returns STATUS_SUCCESS, otherwise error code.
 */
NTSTATUS XRpiCamMipi_t::Deinit()
{
    NTSTATUS status = STATUS_SUCCESS;
    status = Stop();

    /* Reset AP1302 ISP */
    status |= Ap1302Reset();

    return status;
}