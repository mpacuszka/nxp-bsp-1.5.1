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

#include <Base.h>
#include <Uefi/UefiBaseType.h>
#include <Library/DebugLib.h>
#include <Library/LcdPlatformLib.h>
#include <Library/TimerLib.h>

#include <iMXDisplay.h>
#include <iMXI2cLib.h>
#include <it6263.h>

#define CHECK_STATUS_RETURN_ERR(chkfunc, chkreg) \
    { \
      EFI_STATUS chkstatus; \
      if ((chkstatus = (chkfunc)) != EFI_SUCCESS) { \
        DEBUG ((DEBUG_ERROR, "IT6263 register 0x%x returned error %d\n", (chkreg), chkstatus)); \
        return chkstatus; \
      } \
    }

/* IIC address used to access the TDI registers */
#define IT6263_IIC_SLAVE_ADDR               (0x98 >> 1)
#define IT6263_LVDS_IIC_SLAVE_ADDR          (0x66 >> 1)
/* Size of DDC FIFO */
#define IT6263_DDC_FIFO_SIZE                32

/*
 * Configuration of the I2C device to be used for comunication with external LVDS to HDMI converter IT6263.
 */
IMX_I2C_CONTEXT I2C_IT6263Config = {
    (uintptr_t)FixedPcdGet32(PcdDisplayI2CBaseAddr), /* Base address of the I2C block used for  communication with display */
    0x00,                                            /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */ 
    24000000,                                        /* 24Mhz I2C ReferenceFreq */
    400000,                                          /* 400KHz required TargetFreq */
    IT6263_IIC_SLAVE_ADDR,                           /* IT6263 SlaveAddress */
    100000,                                          /* TimeoutInUs */
    };

/*
 * I2C configuration to be used for LVDS specific part of IT6263.
 */
IMX_I2C_CONTEXT I2C_IT6263_LVDSConfig = {
    (uintptr_t)FixedPcdGet32(PcdDisplayI2CBaseAddr), /* Base address of the I2C block used for  communication with display */
    0x00,                                            /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
    24000000,                                        /* 24Mhz I2C ReferenceFreq */
    400000,                                          /* 400KHz required TargetFreq */
    IT6263_LVDS_IIC_SLAVE_ADDR,                      /* LVDS SlaveAddress */
    100000,                                          /* TimeoutInUs */
    };

/**
  Check presence of IT6263 by read of chip ID.

  @retval EFI_SUCCESS            IT6263 search SUCCEDED.
  @retval EFI_DEVICE_ERROR       IT6263 not found.

**/
EFI_STATUS It6263Discover
  (
    VOID
  )
{
  uint8_t data[3] = {0};

  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x01, &data[0], 3), 0x01);
  if ((data[0] == 0xCA) && (data[1] == 0x13) && (data[2] == 0x76)) {
      return EFI_SUCCESS;
  }
  DEBUG ((DEBUG_ERROR, "IT6263 not found id0=%d id1=%d id2=%d\n", data[0], data[1], data[2]));
  return EFI_DEVICE_ERROR;
}

/**
  Read EDID via IT6263.

  @retval EFI_SUCCESS            IT6263 EDID read completed successfully.
  @retval EFI_DEVICE_ERROR       IT6263 error occured.

**/
EFI_STATUS It6263ReadEdid
  (
    OUT uint8_t *edid,
    IN uint32_t offset,
    IN uint32_t length
  )
{
  uint8_t  i2cData[4U];
  uint32_t i, j;

  if (edid == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  for (i = offset; i < offset + length; i += IT6263_DDC_FIFO_SIZE) {
    i2cData[0] = 0x01;  /* Switch controller to PC */
    CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x10, &i2cData[0], 1), 0x10);
    i2cData[0] = 0xA0;  /* DDC request slave address */
    CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x11, &i2cData[0], 1), 0x11);
    i2cData[0] = i;  /* Register address */
    CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x12, &i2cData[0], 1), 0x12);
    i2cData[0] = IT6263_DDC_FIFO_SIZE;  /* How many bytes */
    CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x13, &i2cData[0], 1), 0x13);
    i2cData[0] = 0;  /* EDID segment */
    CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x14, &i2cData[0], 1), 0x14);
    i2cData[0] = 0x03;  /* Start reading EDID */
    CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x15, &i2cData[0], 1), 0x15);
    MicroSecondDelay(11000U); /* Wait 11 ms */
    /* Check HDMI communication result */
    j = 0;

    do {
      CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x16, &i2cData[0], 2), 0x16);
      if (j++ >= 250) { /* Handle timeout */
        DEBUG ((DEBUG_ERROR, "IT6263 No EDID response\n"));
        return EFI_DEVICE_ERROR;
      }
    } while (!(i2cData[0] & 0x80));

    for (j = 0; j < IT6263_DDC_FIFO_SIZE; j+=2) {
      CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x17, &i2cData[0], 2), 0x17);
      edid[i + j - offset] = i2cData[0];
      edid[i + j + 1 - offset] = i2cData[1];
    }
    i2cData[0] = 0x09; /* DDC FIFO Clear */
    CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x15, &i2cData[0], 1), 0x15);
  }

  return EFI_SUCCESS;
}

/**
  Configure IT6263 to required timing mode.

  @param[in] TimingMode          Timing mode to be set.

  @retval EFI_SUCCESS            IT6263 timing mode set successfully.
  @retval EFI_DEVICE_ERROR       IT6263 error occured.

**/
EFI_STATUS It6263SetMode
  (
    IN IMX_DISPLAY_TIMING *PreferredTiming,
    IN imxDisplayInterfaceType DispInterface
  )
{
  uint8_t    i2cData[2];
  uint8_t col_dep = (DispInterface == imxLvds0dual) ? 0x80 : 0;

  if (PreferredTiming == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  i2cData[0] = 0x3D; /* Reset IT6263 */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x04, &i2cData[0], 1), 0x04);
  i2cData[0] = 0x00; /* Select register bank 0 */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  i2cData[0] = 0x40; /* Interrupt pin setup (open dran, active low), TxClk active */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x05, &i2cData[0], 1), 0x05);
  i2cData[0] = 0x15; /* Disable reset signal, enable video circuit */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x04, &i2cData[0], 1), 0x04);
  i2cData[0] = 0x00; /* Select register bank 0 */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  i2cData[0] = 0x66; /* Set LVDS I2C address */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x1D, &i2cData[0], 1), 0x1D);
  i2cData[0] = 0x01; /* Enable LVDS I2C address */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x1E, &i2cData[0], 1), 0x1E);

  /* ResetLVDS */
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x3C, &i2cData[0], 2), 0x3C);
  i2cData[0] = 0x08; /* LVDS AFE PLL RESET */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x3C, &i2cData[0], 1), 0x3C);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x3C, &i2cData[0], 2), 0x3C);
  i2cData[0] = 0x09; 
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x3C, &i2cData[0], 1), 0x3C);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x05, &i2cData[0], 2), 0x05);
  i2cData[0] = 0x02; /* RESET LVDS PCLK */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x05, &i2cData[0], 1), 0x05);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x05, &i2cData[0], 2), 0x05);
  i2cData[0] = 0x00; 
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x05, &i2cData[0], 1), 0x05);

  /* SetLVDSinterface */
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x2C, &i2cData[0], 2), 0x2C);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x2C, &i2cData[0], 2), 0x2C);
  i2cData[0] = 0x64;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x2C, &i2cData[0], 1), 0x2C);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x2C, &i2cData[0], 2), 0x2C);
  i2cData[0] = 0x65 | col_dep;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x2C, &i2cData[0], 1), 0x2C);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x2C, &i2cData[0], 2), 0x2C);

  /* SetLVDS_AFE */
  i2cData[0] = 0xAA;
  i2cData[1] = 0x02;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x3E, &i2cData[0], 1), 0x3E);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x3F, &i2cData[1], 1), 0x3F);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x47, &i2cData[0], 1), 0x47);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x48, &i2cData[1], 1), 0x48);
  i2cData[0] = 0x11;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x4F, &i2cData[0], 1), 0x4F);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x0B, &i2cData[0], 2), 0x0B);
  i2cData[0] = 0x89;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x0B, &i2cData[0], 1), 0x0B);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x3C, &i2cData[0], 2), 0x3C);
  i2cData[0] = 0x08;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x3C, &i2cData[0], 1), 0x3C);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x2C, &i2cData[0], 2), 0x2C);
  i2cData[0] = 0x65 | col_dep;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x2C, &i2cData[0], 1), 0x2C);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x39, &i2cData[0], 2), 0x39);
  if (PreferredTiming->PixelClock > 100000000L) {
    i2cData[0] &= 0x3F;
    CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x39, &i2cData[0], 1), 0x39);
  } else {
    i2cData[0] |= 0xC0;
    CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x39, &i2cData[0], 1), 0x39);
  }
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x2C, &i2cData[0], 2), 0x2C);
  i2cData[0] = 0x75 | col_dep;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x2C, &i2cData[0], 1), 0x2C);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x39, &i2cData[0], 2), 0x39);
  i2cData[0] |= 0x02;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x39, &i2cData[0], 1), 0x39);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263_LVDSConfig, 0x39, &i2cData[0], 2), 0x39);
  i2cData[0] &= 0xFD;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263_LVDSConfig, 0x39, &i2cData[0], 1), 0x39);
  MicroSecondDelay(1000);

  /* Power down and reset HDMI_TX_DRV */
  i2cData[0] = 0x30;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x61, &i2cData[0], 1), 0x61);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0xF3, &i2cData[0], 2), 0xF3);
  i2cData[0] &= ~0x30;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xF3, &i2cData[0], 1), 0xF3);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0xF3, &i2cData[0], 2), 0xF3);
  i2cData[0] |= 0x20;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xF3, &i2cData[0], 1), 0xF3);

  /* Enable interrupts: REG__AudioOvFlw, REG__DDCBusHang, REG__RxSEN, REG__HPD
     Disable interrupts: REG__DDCNoACKMask, REG__DDCFIFOErrMask */
  i2cData[0] = 0x30;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x09, &i2cData[0], 1), 0x09);
  /* Enable interrupts: REG__KSVListChk, REG__AuthDone, REG__AuthFail
     Disable interrupts: REG__PktAVI, REG__PktISRC, REG__PktACP, REG__PktNull, REG__PktGen */
  i2cData[0] = 0xF8;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0A, &i2cData[0], 1), 0x0A);
  /* Enable interrupts: REG__VidStable
     Disable interrupts:REG_AudCTS, REG__Vsync, REG__PktMpg, REG__PktSPD, REG__PktAud */
  i2cData[0] = 0x37;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0B, &i2cData[0], 1), 0x0B);
  i2cData[0] = 0x0;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xC9, &i2cData[0], 1), 0xC9);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xCA, &i2cData[0], 1), 0xCA);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xCB, &i2cData[0], 1), 0xCB);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xCC, &i2cData[0], 1), 0xCC);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xCD, &i2cData[0], 1), 0xCD);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xCE, &i2cData[0], 1), 0xCE);
  /* Disable Source Product Descriptor (SPD) Infoframe packet */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xCF, &i2cData[0], 1), 0xCF);
  /* Disable MPEG Source Infoframe packet */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xD0, &i2cData[0], 1), 0xD0);
  i2cData[0] = 0x1;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x58, &i2cData[0], 2), 0x58);
  /* Set Output color mode to RGB444 */
  i2cData[0] &= ~(3 << 5);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x58, &i2cData[0], 1), 0x58);
  i2cData[0] = 0x0;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xE1, &i2cData[0], 1), 0xE1);
  i2cData[0] = 0xFF;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0C, &i2cData[0], 1), 0x0C);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0D, &i2cData[0], 1), 0x0D);

  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x0E, &i2cData[0], 2), 0x0E);
  i2cData[0] |= 0x03;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0E, &i2cData[0], 1), 0x0E);
  i2cData[0] &= 0xFE; /* Activate interrupt clear action */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0E, &i2cData[0], 1), 0x0E);

  i2cData[0] = 0x1; /* Select register bank 1 */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  i2cData[0] = 0x0;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x33, &i2cData[0], 1), 0x33);
  i2cData[0] = 0x18;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x34, &i2cData[0], 1), 0x34);
  i2cData[0] = 0x0;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x35, &i2cData[0], 1), 0x35);
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  i2cData[0] = 0xFE; /* Set encryption */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xC4, &i2cData[0], 1), 0xC4);

  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0xC5, &i2cData[0], 2), 0xC5);
  i2cData[0] |= 0x30;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xC5, &i2cData[0], 1), 0xC5);
  i2cData[0] = 0x0;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  i2cData[0] = 0x3D;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x04, &i2cData[0], 1), 0x04);

  i2cData[0] = 0x15; /* Disable reset signal, enable video circuit */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x04, &i2cData[0], 1), 0x04);
  i2cData[0] = 0x0; /* Select register bank 0 */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  i2cData[0] = 0x66; /* Set LVDS I2C address */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x1D, &i2cData[0], 1), 0x1D);
  i2cData[0] = 0x01; /* Enable LVDS I2C address */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x1E, &i2cData[0], 1), 0x1E);

  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0xC1, &i2cData[0], 2), 0xC1);
  i2cData[0] = 0x10; /* Reset HDMI_TX_DRV */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x61, &i2cData[0], 1), 0x61);

  i2cData[0] = 0x88; /* SetupAFE, 80MHz < TMDS Clock < 170MHz */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x62, &i2cData[0], 1), 0x62);
  i2cData[0] = 0x10; 
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x63, &i2cData[0], 1), 0x63);
  i2cData[0] = 0x84; 
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x64, &i2cData[0], 1), 0x64);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0xC1, &i2cData[0], 2), 0xC1);
  i2cData[0] = 0x1D;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x04, &i2cData[0], 1), 0x04);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0xC1, &i2cData[0], 2), 0xC1);
  i2cData[0] = 0x15; /* Disable reset signal, enable video circuit */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x04, &i2cData[0], 1), 0x04);
  /* wait video stable */
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x0E, &i2cData[0], 2), 0x0E);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x0E, &i2cData[0], 2), 0x0E);
  /* =======================reset video==========================*/
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x0D, &i2cData[0], 2), 0x0D);
  i2cData[0] = 0x40; /* Clear REG__VidStable interrupt */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x04, &i2cData[0], 1), 0x04);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x0E, &i2cData[0], 2), 0x0E);
  i2cData[0] = 0x7D; /* Perform interrupt clear action */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0E, &i2cData[0], 1), 0x0E);
  i2cData[0] = 0x7C; /* Disable interrupt clear action */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0E, &i2cData[0], 1), 0x0E);
  i2cData[0] = 0x0; /* Select register bank 0 */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  /* Enable HDMI_TX_DRV */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x61, &i2cData[0], 1), 0x61);
  /* Reg[61] = 00 */
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x61, &i2cData[0], 2), 0x61);
  /* Reg[62] = 88 */
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x62, &i2cData[0], 2), 0x62);
  /* Reg[63] = 10 */
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x63, &i2cData[0], 2), 0x63);
  /* Reg[64] = 84 */
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x64, &i2cData[0], 2), 0x64);
  /* Reg[65] = F0 */
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x65, &i2cData[0], 2), 0x65);
  /* Reg[66] = 00 */
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x66, &i2cData[0], 2), 0x66);
  /* Reg[67] = 3F */
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0x67, &i2cData[0], 2), 0x67);
  i2cData[0] = 0x0; /* Select register bank 0 */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData[0], 1), 0x0F);
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(&I2C_IT6263Config, 0xC1, &i2cData[0], 2), 0xC1);
  i2cData[0] = 0x0;
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xC1, &i2cData[0], 1), 0xC1);
  i2cData[0] = 0x03; /* Enable general control packet */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0xC6, &i2cData[0], 1), 0xC6);

  return EFI_SUCCESS;
}

EFI_STATUS It6263DumpRegister(IN IMX_I2C_CONTEXT *i2cConfig, IN uint8_t reg)
{
  uint8_t    i2cData;
  CHECK_STATUS_RETURN_ERR(iMXI2cRead(i2cConfig, reg, &i2cData, 1), 0xBB);
  DEBUG((DEBUG_ERROR, "Reg addr=0x%x               = 0x%02X\n", reg, i2cData));
  return EFI_SUCCESS;
}

EFI_STATUS It6263Dump(VOID)
{
  uint8_t    i2cData;
  DEBUG((DEBUG_ERROR, "------------------------IT6263-------------------------------------\n"));
  DEBUG((DEBUG_ERROR, "------------------------HDMI register bank0-------------------------------------\n"));
  i2cData = 0x00; /* Select register bank 0 */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData, 1), 0x0F);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x0F), 0x0F);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x04), 0x04);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x05), 0x05);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x09), 0x09);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x0A), 0x0A);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x0B), 0x0B);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x0C), 0x0C);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x0D), 0x0D);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x0E), 0x0E);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x61), 0x61);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x62), 0x62);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x63), 0x63);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x64), 0x64);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x65), 0x65);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x66), 0x66);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x67), 0x67);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x1D), 0x1D);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x1E), 0x1E);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xC1), 0xC1);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xC4), 0xC4);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xC5), 0xC5);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xC6), 0xC6);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xC9), 0xC9);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xCA), 0xCA);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xCB), 0xCB);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xCC), 0xCC);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xCD), 0xCD);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xCE), 0xCE);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xCF), 0xCF);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xD0), 0xD0);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xE1), 0xE1);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0xF3), 0xF3);

  DEBUG((DEBUG_ERROR, "------------------------HDMI register bank1-------------------------------------\n"));
  i2cData = 0x01; /* Select register bank 1 */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData, 1), 0x0F);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x0F), 0x0F);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x33), 0x33);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x34), 0x34);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x35), 0x35);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x58), 0x58);

  DEBUG((DEBUG_ERROR, "------------------------LVDS register ------------------------------------------\n"));
  i2cData = 0x01; /* Select register bank 1 */
  CHECK_STATUS_RETURN_ERR(iMXI2cWrite(&I2C_IT6263Config, 0x0F, &i2cData, 1), 0x0F);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263Config, 0x0F), 0x0F);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263_LVDSConfig, 0x05), 0x05);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263_LVDSConfig, 0x0B), 0x0B);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263_LVDSConfig, 0x2C), 0x2C);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263_LVDSConfig, 0x39), 0x39);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263_LVDSConfig, 0x3C), 0x3C);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263_LVDSConfig, 0x3E), 0x3E);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263_LVDSConfig, 0x3F), 0x3F);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263_LVDSConfig, 0x47), 0x47);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263_LVDSConfig, 0x48), 0x48);
  CHECK_STATUS_RETURN_ERR(It6263DumpRegister(&I2C_IT6263_LVDSConfig, 0x4F), 0x4F);

  return EFI_SUCCESS;
}

