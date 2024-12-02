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

#include <stdint.h>
#include <stdbool.h>
#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <iMXGpio.h>
#include <iMXI2cLib.h>
#include <iMXDisplay.h>
#include "MipiDsi_rm67191.h"

#if defined(CPU_IMX8MM) || defined(CPU_IMX8MN) || defined(CPU_IMX8MP)
  #define RM67191_RST_BANK   IMX_GPIO_BANK1
  #define RM67191_RST_IOPIN  8
#elif defined(CPU_IMX8QXP)
  #define IMX_I2C1_BASE                       0x5a810000
  #define IMX_PCA9557_IO_EXP_A_I2C_ADDRESS    0x1A
  #define IMX_PCA9557_IO_EXP_A_I2C_BASE       IMX_I2C1_BASE

  #define IMX_PCA9557_IO_EXP_B_I2C_ADDRESS    0x1D
  #define IMX_PCA9557_IO_EXP_B_I2C_BASE       IMX_I2C1_BASE

  #define EXP_A_MIPI_DSI0_EN                 0x40
  #define EXP_B_MIPI_DSI1_EN                 0x80

  #define EXP_OUTPUT_PORT_REG                0x01

IMX_I2C_CONTEXT Pca9557IOExpAI2CConfig =
{
  (uintptr_t)IMX_PCA9557_IO_EXP_A_I2C_BASE, /* Base address of the I2C used for communication with PCA9557 Exp A */
  0,                                    /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  24000000,                             /* 24Mhz I2C ReferenceFreq */
  400000,                               /* 400KHz required TargetFreq */
  IMX_PCA9557_IO_EXP_A_I2C_ADDRESS,     /* PCA9557 Exp A SlaveAddress */
  100000,                               /* TimeoutInUs ignored - not implemented in LPI2C driver */
};

IMX_I2C_CONTEXT Pca9557IOExpBI2CConfig =
{
  (uintptr_t)IMX_PCA9557_IO_EXP_B_I2C_BASE, /* Base address of the I2C used for communication with PCA9557 Exp B */
  0,                                    /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  24000000,                             /* 24Mhz I2C ReferenceFreq */
  400000,                               /* 400KHz required TargetFreq */
  IMX_PCA9557_IO_EXP_B_I2C_ADDRESS,     /* PCA9557 Exp B SlaveAddress */
  100000,                               /* TimeoutInUs ignored - not implemented in LPI2C driver */
};

  #define CHECK_I2C_TRANSACTION_STATUS(status, message, label) \
            if (status != EFI_SUCCESS) { \
              DEBUG((DEBUG_ERROR, "I2C error. Register: %a, I2CStatus: %d\n", message, status)); \
              goto label; \
            }
#else
  #error Unsupported derivative!
#endif

#define RETURN_IF_ERROR(x) do { int __Ret = (x); \
                           if (__Ret != 0) { \
                               return __Ret; \
                           } \
                       } while (0);

#define CMD_ITEM_SIZE 2
typedef uint8_t cmd_list_item[CMD_ITEM_SIZE];

/* Manufacturer Command List */
static const cmd_list_item cmd_list[] = {
	{0xFE, 0x0B},
	{0x28, 0x40},
	{0x29, 0x4F},
	{0xFE, 0x0E},
	{0x4B, 0x00},
	{0x4C, 0x0F},
	{0x4D, 0x20},
	{0x4E, 0x40},
	{0x4F, 0x60},
	{0x50, 0xA0},
	{0x51, 0xC0},
	{0x52, 0xE0},
	{0x53, 0xFF},
	{0xFE, 0x0D},
	{0x18, 0x08},
	{0x42, 0x00},
	{0x08, 0x41},
	{0x46, 0x02},
	{0x72, 0x09},
	{0xFE, 0x0A},
	{0x24, 0x17},
	{0x04, 0x07},
	{0x1A, 0x0C},
	{0x0F, 0x44},
	{0xFE, 0x04},
	{0x00, 0x0C},
	{0x05, 0x08},
	{0x06, 0x08},
	{0x08, 0x08},
	{0x09, 0x08},
	{0x0A, 0xE6},
	{0x0B, 0x8C},
	{0x1A, 0x12},
	{0x1E, 0xE0},
	{0x29, 0x93},
	{0x2A, 0x93},
	{0x2F, 0x02},
	{0x31, 0x02},
	{0x33, 0x05},
	{0x37, 0x2D},
	{0x38, 0x2D},
	{0x3A, 0x1E},
	{0x3B, 0x1E},
	{0x3D, 0x27},
	{0x3F, 0x80},
	{0x40, 0x40},
	{0x41, 0xE0},
	{0x4F, 0x2F},
	{0x50, 0x1E},
	{0xFE, 0x06},
	{0x00, 0xCC},
	{0x05, 0x05},
	{0x07, 0xA2},
	{0x08, 0xCC},
	{0x0D, 0x03},
	{0x0F, 0xA2},
	{0x32, 0xCC},
	{0x37, 0x05},
	{0x39, 0x83},
	{0x3A, 0xCC},
	{0x41, 0x04},
	{0x43, 0x83},
	{0x44, 0xCC},
	{0x49, 0x05},
	{0x4B, 0xA2},
	{0x4C, 0xCC},
	{0x51, 0x03},
	{0x53, 0xA2},
	{0x75, 0xCC},
	{0x7A, 0x03},
	{0x7C, 0x83},
	{0x7D, 0xCC},
	{0x82, 0x02},
	{0x84, 0x83},
	{0x85, 0xEC},
	{0x86, 0x0F},
	{0x87, 0xFF},
	{0x88, 0x00},
	{0x8A, 0x02},
	{0x8C, 0xA2},
	{0x8D, 0xEA},
	{0x8E, 0x01},
	{0x8F, 0xE8},
	{0xFE, 0x06},
	{0x90, 0x0A},
	{0x92, 0x06},
	{0x93, 0xA0},
	{0x94, 0xA8},
	{0x95, 0xEC},
	{0x96, 0x0F},
	{0x97, 0xFF},
	{0x98, 0x00},
	{0x9A, 0x02},
	{0x9C, 0xA2},
	{0xAC, 0x04},
	{0xFE, 0x06},
	{0xB1, 0x12},
	{0xB2, 0x17},
	{0xB3, 0x17},
	{0xB4, 0x17},
	{0xB5, 0x17},
	{0xB6, 0x11},
	{0xB7, 0x08},
	{0xB8, 0x09},
	{0xB9, 0x06},
	{0xBA, 0x07},
	{0xBB, 0x17},
	{0xBC, 0x17},
	{0xBD, 0x17},
	{0xBE, 0x17},
	{0xBF, 0x17},
	{0xC0, 0x17},
	{0xC1, 0x17},
	{0xC2, 0x17},
	{0xC3, 0x17},
	{0xC4, 0x0F},
	{0xC5, 0x0E},
	{0xC6, 0x00},
	{0xC7, 0x01},
	{0xC8, 0x10},
	{0xFE, 0x06},
	{0x95, 0xEC},
	{0x8D, 0xEE},
	{0x44, 0xEC},
	{0x4C, 0xEC},
	{0x32, 0xEC},
	{0x3A, 0xEC},
	{0x7D, 0xEC},
	{0x75, 0xEC},
	{0x00, 0xEC},
	{0x08, 0xEC},
	{0x85, 0xEC},
	{0xA6, 0x21},
	{0xA7, 0x05},
	{0xA9, 0x06},
	{0x82, 0x06},
	{0x41, 0x06},
	{0x7A, 0x07},
	{0x37, 0x07},
	{0x05, 0x06},
	{0x49, 0x06},
	{0x0D, 0x04},
	{0x51, 0x04},
};

EFI_STATUS Rm67191Init(imxDisplayInterfaceType displayInterface)
{
    const uint8_t *cmd;
    uint32_t i;
    EFI_STATUS Status = EFI_SUCCESS;

#ifdef RM67191_RST_BANK
    /* Make hw-reset pulse */
    ImxGpioWrite (RM67191_RST_BANK, RM67191_RST_IOPIN, IMX_GPIO_HIGH);
    MicroSecondDelay(10000);
    ImxGpioWrite (RM67191_RST_BANK, RM67191_RST_IOPIN, IMX_GPIO_LOW);
    MicroSecondDelay(15000);
    ImxGpioWrite (RM67191_RST_BANK, RM67191_RST_IOPIN, IMX_GPIO_HIGH);
#elif defined(IMX_PCA9557_IO_EXP_B_I2C_ADDRESS) || defined(IMX_PCA9557_IO_EXP_A_I2C_ADDRESS)
    UINT8 Register = EXP_OUTPUT_PORT_REG, Data, Pin;
    IMX_I2C_CONTEXT *context;

    if (displayInterface == imxMipiDsi) {
        context = &Pca9557IOExpAI2CConfig;
        Pin = EXP_A_MIPI_DSI0_EN;
    } else {
        context = &Pca9557IOExpBI2CConfig;
        Pin = EXP_B_MIPI_DSI1_EN;
    }
    Status = iMXI2cReadE(context, Register, 1, &Data, 1);
    CHECK_I2C_TRANSACTION_STATUS(Status, "PCA9557 IO Expander - reading OUTPUT_PORT_REG", End);
    Data = Data | Pin;
    Status = iMXI2cWriteE(context, Register, 1, &Data, 1);
    CHECK_I2C_TRANSACTION_STATUS(Status, "PCA9557 IO Expander - writing OUTPUT_PORT_REG", End);
    MicroSecondDelay(10000);
    Data = Data & ~Pin;
    Status = iMXI2cWriteE(context, Register, 1, &Data, 1);
    CHECK_I2C_TRANSACTION_STATUS(Status, "PCA9557 IO Expander - writing OUTPUT_PORT_REG", End);
    MicroSecondDelay(15000);
    Data = Data | Pin;
    Status = iMXI2cWriteE(context, Register, 1, &Data, 1);
    CHECK_I2C_TRANSACTION_STATUS(Status, "PCA9557 IO Expander - writing OUTPUT_PORT_REG", End);
#endif
    /* Wait after reset pulse */
    MicroSecondDelay(50000);
    for (i = 0; i < (sizeof(cmd_list) / CMD_ITEM_SIZE) ; i++) {
        cmd = cmd_list[i];
        RETURN_IF_ERROR(MipiDsiPktGenericWrite(cmd, CMD_ITEM_SIZE, FALSE));
    }
    /* Select User Command Set - CMD1 */
    RETURN_IF_ERROR(MipiDsiPktGenericWrite((uint8_t[]){0xFE, 0x00 }, 2, FALSE));
    RETURN_IF_ERROR(MipiDsiPktDcsSoftReset(FALSE));
    MicroSecondDelay(15000);
    /* Set DSI mode: 0xB=Video RAM capture mode, 0x3=Video through mode */
    RETURN_IF_ERROR(MipiDsiPktGenericWrite((uint8_t[]){0xC2, 0x0B }, 2, FALSE));
    RETURN_IF_ERROR(MipiDsiPktDcsSetTearOn(MIPI_DSI_DCS_TEAR_MODE_VBL, FALSE));
    RETURN_IF_ERROR(MipiDsiPktDcsSetTearScanline(0x380, FALSE));
    RETURN_IF_ERROR(MipiDsiPktDcsSetDisplayBrightness(0x8C, FALSE));
    /* Set display brightness control on */
    RETURN_IF_ERROR(MipiDsiPktGenericWrite((uint8_t[]){0x53, 0x20 }, 2, FALSE));
    RETURN_IF_ERROR(MipiDsiPktDcsSetPixelFormat(0x77, FALSE)); /* RGB888 */
    RETURN_IF_ERROR(MipiDsiPktDcsExitSleepMode(FALSE));
    MicroSecondDelay(15000);
    RETURN_IF_ERROR(MipiDsiPktDcsSetDisplayOn(FALSE));
    MicroSecondDelay(15000);
    DEBUG((DEBUG_ERROR, "RM67191 OLED panel initialized.\n"));

#if defined(IMX_PCA9557_IO_EXP_B_I2C_ADDRESS) || defined(IMX_PCA9557_IO_EXP_A_I2C_ADDRESS)
End:
#endif
    return Status;
}

EFI_STATUS Rm67191Shutdown(void)
{
    RETURN_IF_ERROR(MipiDsiPktDcsSetDisplayOff(FALSE));
    MicroSecondDelay(5000);
    /* Enter sleep mode */
    RETURN_IF_ERROR(MipiDsiPktDcsEnterSleepMode(FALSE));
    MicroSecondDelay(10000);
    return 0;
}

