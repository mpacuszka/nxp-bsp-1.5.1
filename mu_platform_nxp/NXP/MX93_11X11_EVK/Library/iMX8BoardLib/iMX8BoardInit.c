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

#include <Library/ArmPlatformLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PrintLib.h>
#include <Library/PcdLib.h>
#include <Library/SerialPortLib.h>
#include <Library/TimerLib.h>
#include <Library/ArmSmcLib.h>
#include <Ppi/ArmMpCoreInfo.h>
#include <iMXI2cLib.h>
#include "iMX8.h"

#define ROOT_STATUS_CHANGE_WAIT_TIMES        100000U

#define I2C2_BASE                    0x44350000U
#define PCAL6524_I2C_BASE            I2C2_BASE
#define ADP5585_I2C_BASE             I2C2_BASE

/* PCAL6524 registers */
#define PCAL6524_INPUT_PORT0                    0x00
#define PCAL6524_INPUT_PORT1                    0x01
#define PCAL6524_INPUT_PORT2                    0x02
#define PCAL6524_OUTPUT_PORT0                   0x04
#define PCAL6524_OUTPUT_PORT1                   0x05
#define PCAL6524_OUTPUT_PORT2                   0x06
#define PCAL6524_POLARITY_INVERSION_PORT0       0x08
#define PCAL6524_POLARITY_INVERSION_PORT1       0x09
#define PCAL6524_POLARITY_INVERSION_PORT2       0x0A
#define PCAL6524_CONFIGURATION_PORT0            0x0C
#define PCAL6524_CONFIGURATION_PORT1            0x0D
#define PCAL6524_CONFIGURATION_PORT2            0x0E
#define PCAL6524_OUT_DRIVE_STRENGTH_PORT0A      0x40
#define PCAL6524_OUT_DRIVE_STRENGTH_PORT0B      0x41
#define PCAL6524_OUT_DRIVE_STRENGTH_PORT1A      0x42
#define PCAL6524_OUT_DRIVE_STRENGTH_PORT1B      0x43
#define PCAL6524_OUT_DRIVE_STRENGTH_PORT2A      0x44
#define PCAL6524_OUT_DRIVE_STRENGTH_PORT2B      0x45
#define PCAL6524_INPUT_LATCH_PORT0              0x48
#define PCAL6524_INPUT_LATCH_PORT1              0x49
#define PCAL6524_INPUT_LATCH_PORT2              0x4A
#define PCAL6524_PULL_ENABLE_PORT0              0x4C
#define PCAL6524_PULL_ENABLE_PORT1              0x4D
#define PCAL6524_PULL_ENABLE_PORT2              0x4E
#define PCAL6524_PULL_SELECT_PORT0              0x50
#define PCAL6524_PULL_SELECT_PORT1              0x51
#define PCAL6524_PULL_SELECT_PORT2              0x52
#define PCAL6524_INTERRUPT_MASK_PORT0           0x54
#define PCAL6524_INTERRUPT_MASK_PORT1           0x55
#define PCAL6524_INTERRUPT_MASK_PORT2           0x56
#define PCAL6524_INTERRUPT_STATUS_PORT0         0x58
#define PCAL6524_INTERRUPT_STATUS_PORT1         0x59
#define PCAL6524_INTERRUPT_STATUS_PORT2         0x5A
#define PCAL6524_OUTPUT_PORT_CONFIGURATION      0x5C
#define PCAL6524_INTERRUPT_EDGE_PORT0A          0x60
#define PCAL6524_INTERRUPT_EDGE_PORT0B          0x61
#define PCAL6524_INTERRUPT_EDGE_PORT1A          0x62
#define PCAL6524_INTERRUPT_EDGE_PORT1B          0x63
#define PCAL6524_INTERRUPT_EDGE_PORT2A          0x64
#define PCAL6524_INTERRUPT_EDGE_PORT2B          0x65
#define PCAL6524_INTERRUPT_CLEAR_PORT0          0x68
#define PCAL6524_INTERRUPT_CLEAR_PORT1          0x69
#define PCAL6524_INTERRUPT_CLEAR_PORT2          0x6A
#define PCAL6524_INPUT_STATUS_PORT0             0x6C
#define PCAL6524_INPUT_STATUS_PORT1             0x6D
#define PCAL6524_INPUT_STATUS_PORT2             0x6E
#define PCAL6524_INVID_PIN_CONFIG_PORT0         0x70
#define PCAL6524_INVID_PIN_CONFIG_PORT1         0x71
#define PCAL6524_INVID_PIN_CONFIG_PORT2         0x72
#define PCAL6524_SWITCH_DEBOUNCE_PORT0          0x74
#define PCAL6524_SWITCH_DEBOUNCE_PORT1          0x75
#define PCAL6524_SWITCH_DEBOUNCE_COUNT_PORT0    0x76

/* ADP5585 registers */
#define ADP5585_ID_REG					0x00
#define ADP5585_INT_STATUS_REG			0x01
#define ADP5585_STATUS_REG				0x02
#define ADP5585_FIFO_1_REG				0x03
#define ADP5585_FIFO_2_REG				0x04
#define ADP5585_FIFO_3_REG				0x05
#define ADP5585_FIFO_4_REG				0x06
#define ADP5585_FIFO_5_REG				0x07
#define ADP5585_FIFO_6_REG				0x08
#define ADP5585_FIFO_7_REG				0x09
#define ADP5585_FIFO_8_REG				0x0A
#define ADP5585_FIFO_9_REG				0x0B
#define ADP5585_FIFO_10_REG				0x0C
#define ADP5585_FIFO_11_REG				0x0D
#define ADP5585_FIFO_12_REG				0x0E
#define ADP5585_FIFO_13_REG				0x0F
#define ADP5585_FIFO_14_REG				0x10
#define ADP5585_FIFO_15_REG				0x11
#define ADP5585_FIFO_16_REG				0x12
#define ADP5585_GPI_INT_STAT_A_REG		0x13
#define ADP5585_GPI_INT_STAT_B_REG		0x14
#define ADP5585_GPI_STATUS_A_REG		0x15
#define ADP5585_GPI_STATUS_B_REG		0x16
#define ADP5585_RPULL_CONFIG_A_REG		0x17
#define ADP5585_RPULL_CONFIG_B_REG		0x18
#define ADP5585_RPULL_CONFIG_C_REG		0x19
#define ADP5585_RPULL_CONFIG_D_REG		0x1A
#define ADP5585_GPI_INT_LEVEL_A_REG		0x1B
#define ADP5585_GPI_INT_LEVEL_B_REG		0x1C
#define ADP5585_GPI_EVENT_EN_A_REG		0x1D
#define ADP5585_GPI_EVENT_EN_B_REG		0x1E
#define ADP5585_GPI_INTERRUPT_EN_A_REG	0x1F
#define ADP5585_GPI_INTERRUPT_EN_B_REG	0x20
#define ADP5585_DEBOUNCE_DIS_A_REG		0x21
#define ADP5585_DEBOUNCE_DIS_B_REG		0x22
#define ADP5585_GPO_DATA_OUT_A_REG		0x23
#define ADP5585_GPO_DATA_OUT_B_REG		0x24
#define ADP5585_GPO_OUT_MODE_A_REG		0x25
#define ADP5585_GPO_OUT_MODE_B_REG		0x26
#define ADP5585_GPIO_DIRECTION_A_REG	0x27
#define ADP5585_GPIO_DIRECTION_B_REG	0x28
#define ADP5585_RESET1_EVENT_A_REG		0x29
#define ADP5585_RESET1_EVENT_B_REG		0x2A
#define ADP5585_RESET1_EVENT_C_REG		0x2B
#define ADP5585_RESET2_EVENT_A_REG		0x2C
#define ADP5585_RESET2_EVENT_B_REG		0x2D
#define ADP5585_RESET_CFG_REG			0x2E
#define ADP5585_PWM_OFFT_LOW_REG		0x2F
#define ADP5585_PWM_OFFT_HIGH_REG		0x30
#define ADP5585_PWM_ONT_LOW_REG			0x31
#define ADP5585_PWM_ONT_HIGH_REG		0x32
#define ADP5585_PWM_CFG_REG				0x33
#define ADP5585_LOGIC_CFG_REG			0x34
#define ADP5585_LOGIC_FF_CFG_REG		0x35
#define ADP5585_LOGIC_INT_EVENT_EN_REG	0x36
#define ADP5585_POLL_PTIME_CFG_REG		0x37
#define ADP5585_PIN_CONFIG_A_REG		0x38
#define ADP5585_PIN_CONFIG_B_REG		0x39
#define ADP5585_PIN_CONFIG_C_REG		0x3A
#define ADP5585_GENERAL_CFG_REG			0x3B
#define ADP5585_INT_EN_REG				0x3C

/* PCAL6524 IO Expander IO pins - port 0 */
#define EXP_P0_0_TCPC0_INT_B                0x01U
#define EXP_P0_1_TCPC1_INT_B                0x02U
#define EXP_P0_2_PCIE_WAKE_B                0x04U
#define EXP_P0_3_M2_UART_WAKE_B             0x08U
#define EXP_P0_4_AUD_INT_B                  0x10U
#define EXP_P0_5_EXP_BTN1                   0x20U
#define EXP_P0_6_EXP_BTN2                   0x40U
#define EXP_P0_7_DSI_CTP_INT_B              0x80U

/* PCAL6524 IO Expander IO pins - port 1 */
#define EXP_P1_0_CTP_INT                    0x01U
#define EXP_P1_1_TCPC2_INT_B                0x02U
#define EXP_P1_2_M2_ALERT_B                 0x04U
#define EXP_P1_3_PMIC_INT_B                 0x08U
#define EXP_P1_4_SD3_RTS_B                  0x10U
#define EXP_P1_5_EXT1_PWREN                 0x20U
#define EXP_P1_6_EXT2_PWREN                 0x40U
#define EXP_P1_7_ENET1_RST_B                0x80U

/* PCAL6524 IO Expander IO pins - port 2 */
#define EXP_P2_0_ENET2_RST_B                0x01U
#define EXP_P2_1_CTP_RST                    0x02U
#define EXP_P2_2_M2_RST_B                   0x04U
#define EXP_P2_3_M2_DIS2_B                  0x08U
#define EXP_P2_4_M2_DIS1_B                  0x10U
#define EXP_P2_5_USB1_SS_SEL                0x20U
#define EXP_P2_6_USB1_SS_XSD                0x40U
#define EXP_P2_7_USB2_SEL                   0x80U

/* ADP5585 IO Expander IO pins */
#define EXP_R0_CSI_RST                      0x01U
#define EXP_R1_AUD_PWREN                    0x02U
#define EXP_R2_PDM_MQS                      0x04U
#define EXP_R3_LVDS_BLT_PWM                 0x08U
#define EXP_R4_EXP_SEL                      0x10U
#define EXP_C0_CAN_STDBY                    0x01U
#define EXP_C1_DSI_EN                       0x02U
#define EXP_C2_DSI_BLT_PWM                  0x04U
#define EXP_C3_LVDS_BLT_EN                  0x08U
#define EXP_C4_DSI_CTP_RST                  0x10U

ARM_CORE_INFO iMX8Ppi[] =
{
  {
    // Cluster 0, Core 0
    GET_MPID(0x0, 0x0),
    // MP Core MailBox Set/Get/Clear Addresses and Clear Value. Not used with i.MX8, set to 0
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (UINT64)0
  },
#if FixedPcdGet32(PcdCoreCount) > 1
  {
    // Cluster 0, Core 1
    GET_MPID(0x0, 0x1),
    // MP Core MailBox Set/Get/Clear Addresses and Clear Value. Not used with i.MX8, set to 0
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (UINT64)0
  },
#endif // FixedPcdGet32(PcdCoreCount) > 1
#if FixedPcdGet32(PcdCoreCount) > 2
  {
    // Cluster 0, Core 2
    GET_MPID(0x0, 0x2),
    // MP Core MailBox Set/Get/Clear Addresses and Clear Value. Not used with i.MX8, set to 0
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (UINT64)0
  },
  {
    // Cluster 0, Core 3
    GET_MPID(0x0, 0x3),
    // MP Core MailBox Set/Get/Clear Addresses and Clear Value. Not used with i.MX8, set to 0
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (UINT64)0
  }
#endif // FixedPcdGet32(PcdCoreCount) > 2
};

IMX_I2C_CONTEXT Pcal6524I2cConfig =
{
  (uintptr_t)PCAL6524_I2C_BASE,         /* Base address of the I2C used for communication with PCAL6524 */
  0,                                    /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  40000000,                             /* 40Mhz I2C ReferenceFreq */
  400000,                               /* 400KHz required TargetFreq */
  0x22,                                 /* PCAL6524 SlaveAddress */
  100000,                               /* TimeoutInUs */
};

IMX_I2C_CONTEXT Adp5585I2cConfig =
{
  (uintptr_t)ADP5585_I2C_BASE,          /* Base address of the I2C used for communication with ADP5585 */
  0,                                    /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  40000000,                             /* 40Mhz I2C ReferenceFreq */
  400000,                               /* 400KHz required TargetFreq */
  0x34,                                 /* ADP5585 SlaveAddress */
  100000,                               /* TimeoutInUs */
};


#define in32(_Addr)          (*(UINT32*)((void*)(UINT64)(_Addr)))
#define out32(_Addr,_Val)    (*(UINT32*)(void*)(UINT64)(_Addr)) = _Val


EFI_STATUS PrePeiCoreGetMpCoreInfo (OUT UINTN *CoreCount, OUT ARM_CORE_INFO **ArmCoreTable)
{
  // Only support one cluster
  *CoreCount = sizeof(iMX8Ppi) / sizeof(ARM_CORE_INFO);
  ASSERT (*CoreCount == FixedPcdGet32 (PcdCoreCount));
  *ArmCoreTable = iMX8Ppi;
  return EFI_SUCCESS;
}

ARM_MP_CORE_INFO_PPI mMpCoreInfoPpi = { PrePeiCoreGetMpCoreInfo };

EFI_PEI_PPI_DESCRIPTOR      gPlatformPpiTable[] = {
  {
    EFI_PEI_PPI_DESCRIPTOR_PPI,
    &gArmMpCoreInfoPpiGuid,
    &mMpCoreInfoPpi
  }
};

VOID ArmPlatformGetPlatformPpiList (OUT UINTN *PpiListSize, OUT EFI_PEI_PPI_DESCRIPTOR **PpiList)
{
  *PpiListSize = sizeof(gPlatformPpiTable);
  *PpiList     = gPlatformPpiTable;
}

/** Configures peripheral clock setting.
**/
static VOID InitPeriphClock(UINT16 TargetIndex, UINT16 LpcgIndex, UINT8 Mux, UINT8 Div)
{
    UINT32 waitTimes = ROOT_STATUS_CHANGE_WAIT_TIMES;
    
    /* Disable given clock root */
    CCM_LPCG_DIRECT_REG(CCM_CTRL_BASE_PTR, LpcgIndex) = 0x0U;
    
    /* Set clock root */
    CCM_CLOCK_ROOT_CLOCK_ROOT_CONTROL_RW_REG(CCM_CTRL_BASE_PTR, TargetIndex) = 
      CCM_CLOCK_ROOT_MUX(Mux) | CCM_CLOCK_ROOT_DIV(Div - 1);
    
    while ((CCM_CLOCK_ROOT_STATUS0_REG(CCM_CTRL_BASE_PTR, TargetIndex) & CCM_CLOCK_ROOT_STATUS0_CHANGING_MASK) |
           (--waitTimes != 0U)) {}

    /* Enable given clock root */
    CCM_LPCG_DIRECT_REG(CCM_CTRL_BASE_PTR, LpcgIndex) = 0x1U;
}

#define CHECK_PCAL6524_I2C_TRANSACTION_STATUS(status, message, label) \
            if (status != EFI_SUCCESS) { \
              DEBUG((DEBUG_ERROR, "PCAL6524 I2C error. Register: %a, I2CStatus: %d\n", message, status)); \
              goto label; \
            }

/**
  Configure pins of PCAL6524 IO expander which are specified by BitMask parameter.
  Note: Dir parameter: 0 = output, 1 = input
**/
EFI_STATUS PCAL6524_ConfigurePins(IN IMX_I2C_CONTEXT *I2cContext, IN UINT8 *BitMask, IN UINT8 *Inv, IN UINT8 *Dir, IN UINT8 *OutVal)
{
  EFI_STATUS Status;
  uint8_t RegAddr;
  uint8_t Data[3];

  /* Configure Inversion registers */
  RegAddr = PCAL6524_POLARITY_INVERSION_PORT0;
  Status = iMXI2cRead(I2cContext, RegAddr, &Data[0], 3);
  CHECK_PCAL6524_I2C_TRANSACTION_STATUS(Status, "PCAL6524 Inversion read", End);
  Data[0] = (Data[0] & ~(BitMask[0])) | (BitMask[0] & Inv[0]);
  Data[1] = (Data[1] & ~(BitMask[1])) | (BitMask[1] & Inv[1]);
  Data[2] = (Data[2] & ~(BitMask[2])) | (BitMask[2] & Inv[2]);
  Status = iMXI2cWrite(I2cContext, RegAddr, &Data[0], 3);
  CHECK_PCAL6524_I2C_TRANSACTION_STATUS(Status, "PCAL6524 Inversion write", End);
  /* Configure Output registers */
  RegAddr = PCAL6524_OUTPUT_PORT0;
  Status = iMXI2cRead(I2cContext, RegAddr, &Data[0], 3);
  CHECK_PCAL6524_I2C_TRANSACTION_STATUS(Status, "PCAL6524 Output Port read", End);
  Data[0] = (Data[0] & ~(BitMask[0])) | (BitMask[0] & OutVal[0]);
  Data[1] = (Data[1] & ~(BitMask[1])) | (BitMask[1] & OutVal[1]);
  Data[2] = (Data[2] & ~(BitMask[2])) | (BitMask[2] & OutVal[2]);
  Status = iMXI2cWrite(I2cContext, RegAddr, &Data[0], 3);
  CHECK_PCAL6524_I2C_TRANSACTION_STATUS(Status, "PCAL6524 Output Port write", End);
  /* Configure Configuration (direction) registers */
  RegAddr = PCAL6524_CONFIGURATION_PORT0;
  Status = iMXI2cRead(I2cContext, RegAddr, &Data[0], 3);
  CHECK_PCAL6524_I2C_TRANSACTION_STATUS(Status, "PCAL6524 Configuration read", End);
  Data[0] = (Data[0] & ~(BitMask[0])) | (BitMask[0] & Dir[0]);
  Data[1] = (Data[1] & ~(BitMask[1])) | (BitMask[1] & Dir[1]);
  Data[2] = (Data[2] & ~(BitMask[2])) | (BitMask[2] & Dir[2]);
  Status = iMXI2cWrite(I2cContext, RegAddr, &Data[0], 3);
  CHECK_PCAL6524_I2C_TRANSACTION_STATUS(Status, "PCAL6524 Configuration write", End);
End:
  return Status;
}

#define CHECK_ADP5585_I2C_TRANSACTION_STATUS(status, message, label) \
            if (status != EFI_SUCCESS) { \
              DEBUG((DEBUG_ERROR, "ADP5585 I2C error. Register: %a, I2CStatus: %d\n", message, status)); \
              goto label; \
            }

/**
  Configure pins of ADP5585 IO expander which are specified by BitMask parameter.
  Note: Dir parameter: 0 = input, 1 = output
**/
EFI_STATUS ADP5585_ConfigurePins(IN IMX_I2C_CONTEXT *I2cContext, IN UINT8 *BitMask, IN UINT8 *Dir, IN UINT8 *OutVal)
{
  EFI_STATUS Status;
  uint8_t RegAddr;
  uint8_t Data[2];

  /* Configure Output registers */
  RegAddr = ADP5585_GPO_DATA_OUT_A_REG;
  Status = iMXI2cRead(I2cContext, RegAddr, &Data[0], 2);
  CHECK_ADP5585_I2C_TRANSACTION_STATUS(Status, "ADP5585 Output Port read", End);
  Data[0] = (Data[0] & ~(BitMask[0])) | (BitMask[0] & OutVal[0]);
  Data[1] = (Data[1] & ~(BitMask[1])) | (BitMask[1] & OutVal[1]);
  Status = iMXI2cWrite(I2cContext, RegAddr, &Data[0], 2);
  CHECK_ADP5585_I2C_TRANSACTION_STATUS(Status, "ADP5585 Output Port write", End);
  /* Configure Direction registers */
  RegAddr = ADP5585_GPIO_DIRECTION_A_REG;
  Status = iMXI2cRead(I2cContext, RegAddr, &Data[0], 2);
  CHECK_ADP5585_I2C_TRANSACTION_STATUS(Status, "ADP5585 Direction read", End);
  Data[0] = (Data[0] & ~(BitMask[0])) | (BitMask[0] & Dir[0]);
  Data[1] = (Data[1] & ~(BitMask[1])) | (BitMask[1] & Dir[1]);
  Status = iMXI2cWrite(I2cContext, RegAddr, &Data[0], 2);
  CHECK_ADP5585_I2C_TRANSACTION_STATUS(Status, "ADP5585 Direction write", End);
End:
  return Status;
}

VOID UngateClocks ()
{
}

/**
 * Initialize Imaging subsystem clocks.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
VOID CameraInit()
{
  /* Configure CCM_TARGET_MEDIA_AXI clock root */
  InitPeriphClock(CCM_TARGET_MEDIA_AXI, CCM_LPCG_ISI, 2U, 2U); //69,0x44452280, Sourced from SYS_PLL_PFD1(800MHz), Div = 2, resulting freq = 400MHz
  /* Configure CCM_TARGET_MEDIA_APB clock root */
  InitPeriphClock(CCM_TARGET_MEDIA_APB, CCM_LPCG_ISI, 2U, 3U); //70,0x44452300, Sourced from SYS_PLL_PFD1_DIV2(400MHz), Div = 3, resulting freq = 133MHz
  /* Configure MIPI_CSI clock root */
  InitPeriphClock(CCM_TARGET_MIPI_PHY_CFG, CCM_LPCG_MIPI_CSI, 0U, 0U); //75,0x44452580, Sourced from OSC_24M_CLK(24MHz), Div = 1, resulting freq = 24MHz
  /* Configure CAM_PIX clock root */
  InitPeriphClock(CCM_TARGET_CAM_PIX, CCM_LPCG_MIPI_CSI, 3U, 5U); //73, 0x44452480, Sourced from SYS_PLL_PFD0(1000MHz), Div = 5, resulting freq = 200MHz

  // Deasert resets
  BLK_CTRL_MEDIAMIX_RESET |= (MEDIAMIX_BLK_CTRL_RESET_cam_clk_en_MASK | MEDIAMIX_BLK_CTRL_RESET_csi_apb_en_MASK
                          | MEDIAMIX_BLK_CTRL_RESET_isi_proc_en_MASK | MEDIAMIX_BLK_CTRL_RESET_isi_apb_en_MASK
                          | MEDIAMIX_BLK_CTRL_RESET_bus_blk_en_MASK | MEDIAMIX_BLK_CTRL_RESET_bus_apb_en_MASK);
  // Ungate clocks
  BLK_CTRL_MEDIAMIX_CLK &= ~(MEDIAMIX_BLK_CTRL_CLK_cam_clk_en_MASK | MEDIAMIX_BLK_CTRL_CLK_csi_apb_en_MASK
                        | MEDIAMIX_BLK_CTRL_CLK_isi_proc_en_MASK | MEDIAMIX_BLK_CTRL_CLK_isi_apb_en_MASK
                        | MEDIAMIX_BLK_CTRL_CLK_bus_blk_en_MASK | MEDIAMIX_BLK_CTRL_CLK_bus_apb_en_MASK);

  return;
}

/**
  Initialize GPIO modules on the SOC and perform required pin-muxing
**/
VOID GpioInit ()
{
}

/**
  Initalize the Audio system
**/
#define SAI_PAD_CFG_OUT (IOMUXC_PAD_PUE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_4)
#define SAI_PAD_CFG_IN  (SAI_PAD_CFG_OUT | IOMUXC_PAD_HYS_ENABLED)
VOID AudioInit(VOID)
{
    UINT32 Loop = 0;

    /* Configure SAI3 pads */
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO26 = IOMUXC_MUX_ALT7;    // SAI3_TX_SYNC
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO16 = IOMUXC_MUX_ALT1;    // SAI3_TX_BCLK
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO19 = IOMUXC_MUX_ALT7;    // SAI3_TX_DATA00
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO20 = IOMUXC_MUX_ALT1;    // SAI3_RX_DATA00
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO17 = IOMUXC_MUX_ALT1;    // SAI3_MCLK

    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO26 = SAI_PAD_CFG_OUT;    // SAI3_TX_SYNC
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO16 = SAI_PAD_CFG_OUT;    // SAI3_TX_BCLK
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO19 = SAI_PAD_CFG_OUT;    // SAI3_TX_DATA00
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO20 = SAI_PAD_CFG_IN;     // SAI3_RX_DATA00
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO17 = SAI_PAD_CFG_OUT;    // SAI3_MCLK

    /* Configure AUDIO PLL */
    /* Bypass PLL */
    AUDIOPLL_CTRL_SET = PLL_CTRL_CLKMUX_BYPASS_MASK;
    /* Disable output and PLL */
    AUDIOPLL_CTRL_CLR = PLL_CTRL_CLKMUX_EN_MASK | PLL_CTRL_POWERUP_MASK;
#if 1
    /* 361267200 Hz */
    /* Set rdiv, mfi, and odiv */
    AUDIOPLL_DIV = PLL_DIV_RDIV(1) | PLL_DIV_MFI(210) | PLL_DIV_ODIV(14);
    /* Disable spread spectrum modulation */
    AUDIOPLL_SPREAD_SPECTRUM_CLR = PLL_SPREAD_SPECTRUM_ENABLE_MASK;
    /* Set mfn and mfd */
    AUDIOPLL_NUMERATOR = PLL_NUMERATOR_MFN(5082);
    AUDIOPLL_DENOMINATOR = PLL_DENOMINATOR_MFD(6875);
#else
    /* 393216000 Hz */
    /* Set rdiv, mfi, and odiv */
    AUDIOPLL_DIV = PLL_DIV_RDIV(1) | PLL_DIV_MFI(163) | PLL_DIV_ODIV(10);
    /* Disable spread spectrum modulation */
    AUDIOPLL_SPREAD_SPECTRUM_CLR = PLL_SPREAD_SPECTRUM_ENABLE_MASK;
    /* Set mfn and mfd */
    AUDIOPLL_NUMERATOR = PLL_NUMERATOR_MFN(84);
    AUDIOPLL_DENOMINATOR = PLL_DENOMINATOR_MFD(100);
#endif
    /* Power up for locking */
    AUDIOPLL_CTRL_SET = PLL_CTRL_POWERUP_MASK;
    /* Wait until lock */
    while ((AUDIOPLL_PLL_STATUS & PLL_PLL_STATUS_PLL_LOCK_MASK) == 0) {
        MicroSecondDelay(10U);
        Loop++;
        if (Loop > 100U) {
            DEBUG((DEBUG_ERROR, "Wait on Audio PLL lock failed.\n"));
            break;
        }
    }
    /* Enable PLL and clean bypass */
    AUDIOPLL_CTRL_SET = PLL_CTRL_CLKMUX_EN_MASK;
    AUDIOPLL_CTRL_CLR = PLL_CTRL_CLKMUX_BYPASS_MASK;

    /* Configure SAI3 clock root to 11.2896 MHz (12.288 MHz), AUDIO_PLL_CLK 361.2672 MHz (393.216 Mhz) divided by 32, turn on clock */
    InitPeriphClock(CCM_TARGET_SAI3, CCM_LPCG_SAI3, 1U, 32U);

    /* SAI3 clock source selections:
       MCLK1: SAI3 CLK_ROOT from CCM
       MCLK2: SAI3 CLK ROOT from CCM
       MCLK3: SAI3 CLK ROOT from CCM
    */
    BLK_CTRL_WAKEUPMIX1_SAI_CLK_SEL = (BLK_CTRL_WAKEUPMIX1_SAI_CLK_SEL & ~(BLK_CTRL_WAKEUPMIX_SAI_CLK_SEL_SAI3_MCLK1_MASK |
        BLK_CTRL_WAKEUPMIX_SAI_CLK_SEL_SAI3_MCLK2_MASK | BLK_CTRL_WAKEUPMIX_SAI_CLK_SEL_SAI3_MCLK3_MASK)) |
        BLK_CTRL_WAKEUPMIX_SAI_CLK_SEL_SAI3_MCLK2(0x2) | BLK_CTRL_WAKEUPMIX_SAI_CLK_SEL_SAI3_MCLK3(0x2);

    /* Configure SAI3_MCLK signal direction to output and its source to MCLK1 SAI clock input divided by 1 */
    SAI3_MCR = I2S_MCR_MOE_MASK;
}

/**
  Initialize I2C modules on the SOC and perform required pin-muxing
**/
#define I2C_PAD_CTRL (IOMUXC_PAD_HYS_ENABLED | IOMUXC_PAD_PUE_ENABLE | IOMUXC_PAD_ODE_ENABLED | IOMUXC_PAD_SRE_SLOW | IOMUXC_PAD_DSE_R0_DIV_3)
VOID I2cInit()
{
    /* Configure I2C pads */
    IOMUXC1_SW_MUX_CTL_PAD_I2C1_SCL  = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
    IOMUXC1_SW_MUX_CTL_PAD_I2C1_SDA  = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
    IOMUXC1_SW_MUX_CTL_PAD_I2C2_SCL  = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
    IOMUXC1_SW_MUX_CTL_PAD_I2C2_SDA  = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO28 = IOMUXC_MUX_ALT1 | IOMUXC_MUX_SION_ENABLED;      /* I2C3 */
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO29 = IOMUXC_MUX_ALT1 | IOMUXC_MUX_SION_ENABLED;      /* I2C3 */

    IOMUXC1_SW_PAD_CTL_PAD_I2C1_SCL  = I2C_PAD_CTRL;
    IOMUXC1_SW_PAD_CTL_PAD_I2C1_SDA  = I2C_PAD_CTRL;
    IOMUXC1_SW_PAD_CTL_PAD_I2C2_SCL  = I2C_PAD_CTRL;
    IOMUXC1_SW_PAD_CTL_PAD_I2C2_SDA  = I2C_PAD_CTRL;
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO28 = I2C_PAD_CTRL;                                   /* I2C3 */
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO29 = I2C_PAD_CTRL;                                   /* I2C3 */

    /* Configure LPI2C1 clock root to 40 MHz, SYS_PLL_PFD1_DIV2 400 MHz divided by 10, turn on clock */
    InitPeriphClock(CCM_TARGET_LPI2C1, CCM_LPCG_LPI2C1, 2U, 10U);
                                                                                           
    /* Configure LPI2C2 clock root to 40 MHz, SYS_PLL_PFD1_DIV2 400 MHz divided by 10, turn on clock */
    InitPeriphClock(CCM_TARGET_LPI2C2, CCM_LPCG_LPI2C2, 2U, 10U);

    /* Configure LPI2C3 clock root to 40 MHz, SYS_PLL_PFD1_DIV2 400 MHz divided by 10, turn on clock */
    InitPeriphClock(CCM_TARGET_LPI2C3, CCM_LPCG_LPI2C3, 2U, 10U);
}

/**
  Initialize SPI modules on the SOC and perform required pin-muxing
**/
VOID SpiInit()
{
    uint8_t i;
    typedef struct {
        UINT16 TargetIndex;
        UINT16 LpcgIndex;
        UINT8 Mux;
        UINT8 Div;
    } spi_clk_cfg_t;
    spi_clk_cfg_t instances[] = {
                                  // {CCM_TARGET_LPSPI1, CCM_LPCG_LPSPI1, 0U, 1U},    // Clock root to 24MHz, OSC24MHZ divided by 1
                                  // {CCM_TARGET_LPSPI2, CCM_LPCG_LPSPI2, 0U, 1U},    // Clock root to 24MHz, OSC24MHZ divided by 1
                                  {CCM_TARGET_LPSPI3, CCM_LPCG_LPSPI3, 0U, 1U},    // Clock root to 24MHz, OSC24MHZ divided by 1
                                  // {CCM_TARGET_LPSPI4, CCM_LPCG_LPSPI4, 0U, 1U},    // Clock root to 24MHz, OSC24MHZ divided by 1
                                  // {CCM_TARGET_LPSPI5, CCM_LPCG_LPSPI5, 0U, 1U},    // Clock root to 24MHz, OSC24MHZ divided by 1
                                  // {CCM_TARGET_LPSPI6, CCM_LPCG_LPSPI6, 0U, 1U},    // Clock root to 24MHz, OSC24MHZ divided by 1
                                  // {CCM_TARGET_LPSPI7, CCM_LPCG_LPSPI7, 0U, 1U},    // Clock root to 24MHz, OSC24MHZ divided by 1
                                  // {CCM_TARGET_LPSPI8, CCM_LPCG_LPSPI8, 0U, 1U},    // Clock root to 24MHz, OSC24MHZ divided by 1
                                };
    uint8_t instances_num = (uint8_t)(sizeof(instances) / sizeof(spi_clk_cfg_t));

    /* Enable SPI clocks with required frequency */
    for (i = 0; i < instances_num; i++) {
        spi_clk_cfg_t instance = instances[i];
        /* Configure SPIn and turn on clock */
        InitPeriphClock(instance.TargetIndex, instance.LpcgIndex, instance.Mux, instance.Div);
    }
}

VOID UartInit()
{
    const UINT32 uart_pad_ctrl = (IOMUXC_PAD_HYS_ENABLED | IOMUXC_PAD_SRE_MEDIUM | IOMUXC_PAD_ODE_DISABLED | IOMUXC_PAD_DSE_R0_DIV_3);
    /* Configure LPUART pads */
    
    IOMUXC1_SW_MUX_CTL_PAD_UART1_RXD = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_MUX_CTL_PAD_UART1_TXD = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_UART1_RXD = uart_pad_ctrl;
    IOMUXC1_SW_PAD_CTL_PAD_UART1_TXD = uart_pad_ctrl;

    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO14 = IOMUXC_MUX_ALT1; // UART3 TX
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO15 = IOMUXC_MUX_ALT1; // UART3 RX
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO14 = uart_pad_ctrl; // UART3 TX
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO15 = uart_pad_ctrl; // UART3 RX
}

/**
  Initialize USB modules on the SOC and perform required pin-muxing
**/
VOID USBInit (VOID)
{

    /* Enable USB_CONTROLLER clock gate */
    CCM_CTRL_LPCG104_DIRECT = CCM_LPCG_DIRECT_ON_MASK;
    /* Enable USB_TEST_60M clock gate */
    CCM_CTRL_LPCG105_DIRECT = CCM_LPCG_DIRECT_ON_MASK;
    /* Enable HSIO_32K clock gate */
    CCM_CTRL_LPCG119_DIRECT = CCM_LPCG_DIRECT_ON_MASK;

    /* Stop OTG controller core */
    USB__USB_OTG1_USBCMD &= ~USB_USBCMD_RS_MASK;
    while (USB__USB_OTG1_USBCMD & USB_USBCMD_RS_MASK);
    /* Reset OTG controller core */
    USB__USB_OTG1_USBCMD &= USB_USBCMD_RST_MASK;
    while (USB__USB_OTG1_USBCMD & USB_USBCMD_RS_MASK);

    /* Stop OTG controller core */
    USB__USB_OTG2_USBCMD &= ~USB_USBCMD_RS_MASK;
    while (USB__USB_OTG2_USBCMD & USB_USBCMD_RS_MASK);
    /* Reset OTG controller core */
    USB__USB_OTG2_USBCMD &= USB_USBCMD_RST_MASK;
    while (USB__USB_OTG2_USBCMD & USB_USBCMD_RS_MASK);
}


/**
  Initialize pads for ENET(FEC) peripheral
**/
VOID EnetPadInit (VOID)
{
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_TXC = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_TXC = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_TX_CTL = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_TX_CTL = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_TD0 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_TD0 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_TD1 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_TD1 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_TD2 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_TD2 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_TD3 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_TD3 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_RXC = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_RXC = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_RX_CTL = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_RX_CTL = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_RD0 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_RD0 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_RD1 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_RD1 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_RD2 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_RD2 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_RD3 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_RD3 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_MDC = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_MDC = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET2_MDIO = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET2_MDIO = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
}

/**
  Initialize clock for ENET
**/
VOID EnetClockInit (VOID)
{
    /* Disable ENET1 clock root */
    CCM_CTRL_LPCG120_DIRECT = 0x00;
    
    /* ENET_REF_CLK */
    CCM_CTRL_CLOCK_ROOT89_CONTROL =  CCM_CLOCK_ROOT_MUX(1) | CCM_CLOCK_ROOT_DIV(2 - 1);
    while (CCM_CTRL_CLOCK_ROOT89_STATUS0 & CCM_CLOCK_ROOT_STATUS0_CHANGING_MASK) {}
    
    /* ENET_TIMER1_CLK */
    CCM_CTRL_CLOCK_ROOT87_CONTROL = CCM_CLOCK_ROOT_MUX(2) | CCM_CLOCK_ROOT_DIV(4 - 1);
    while (CCM_CTRL_CLOCK_ROOT87_STATUS0 & CCM_CLOCK_ROOT_STATUS0_CHANGING_MASK) {}
    
    /* ENET_REF_PHY_CLK */
    CCM_CTRL_CLOCK_ROOT90_CONTROL = CCM_CLOCK_ROOT_MUX(2) | CCM_CLOCK_ROOT_DIV(8U - 1);
    while (CCM_CTRL_CLOCK_ROOT90_STATUS0 & CCM_CLOCK_ROOT_STATUS0_CHANGING_MASK) {}
    
    /* Enable ENET1 clock root */
    CCM_CTRL_LPCG120_DIRECT = CCM_LPCG_DIRECT_ON_MASK;
}

/**
  Initialize pads for ENET_QoS peripheral
**/
VOID QosPadInit (VOID)
{
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_TXC = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_TXC = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_TX_CTL = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_TX_CTL = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_TD0 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_TD0 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_TD1 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_TD1 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_TD2 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_TD2 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_TD3 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_TD3 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_RXC = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_RXC = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_RX_CTL = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_RX_CTL = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_RD0 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_RD0 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_RD1 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_RD1 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_RD2 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_RD2 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_RD3 = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_RD3 = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_MDC = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_MDC = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
    IOMUXC1_SW_MUX_CTL_PAD_ENET1_MDIO = IOMUXC_MUX_ALT0;
    IOMUXC1_SW_PAD_CTL_PAD_ENET1_MDIO = IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5;
}

/**
  Initialize clock for ENET_QOS
**/
VOID QosClockInit (VOID)
{
    /* Disable ENET_QOS clock root */
    CCM_CTRL_LPCG121_DIRECT = 0x00;

    /* Configure QOS INTF as RGMII and enable RGMII TXC clock */
    out32(0x42420000 + 0x28, (in32(0x42420000 + 0x28) & ~(0x0EU)) | (0x01U << 1) | (0x01U << 0));
    
    /* ENET_CLK */
    CCM_CTRL_CLOCK_ROOT86_CONTROL =  CCM_CLOCK_ROOT_MUX(1) | CCM_CLOCK_ROOT_DIV(2 - 1);
    while (CCM_CTRL_CLOCK_ROOT86_STATUS0 & CCM_CLOCK_ROOT_STATUS0_CHANGING_MASK) {}
    
    /* ENET_TIMER2_CLK */
    CCM_CTRL_CLOCK_ROOT88_CONTROL = CCM_CLOCK_ROOT_MUX(2) | CCM_CLOCK_ROOT_DIV(4 - 1);
    while (CCM_CTRL_CLOCK_ROOT88_STATUS0 & CCM_CLOCK_ROOT_STATUS0_CHANGING_MASK) {}
        
    /* Enable ENET_QOS clock root */
    CCM_CTRL_LPCG121_DIRECT = CCM_LPCG_DIRECT_ON_MASK;
}

/**
  Initialize ENETs modules on the SOC and perform required pin-muxing.
**/
VOID EnetInit(VOID)
{
  EnetPadInit();
  EnetClockInit();
  QosPadInit();
  QosClockInit();
}

/**
  Initialize PWM block and perform required pin-muxing.
**/
VOID PwmInit()
{
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO04 = IOMUXC_MUX_ALT1;  // TPM3_CH_0 Green LED
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO12 = IOMUXC_MUX_ALT1;  // TPM3_CH_2 Blue LED
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO13 = IOMUXC_MUX_ALT1;  // TPM4_CH_2 Red LED
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO04 = IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5 | IOMUXC_PAD_PDE_ENABLE;  // TPM3_CH_0 Green LED
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO12 = IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5 | IOMUXC_PAD_PDE_ENABLE;  // TPM3_CH_2 Blue LED
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO13 = IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_5 | IOMUXC_PAD_PDE_ENABLE;  // TPM4_CH_2 Red LED

    /* TPM3 clock root is set 133 MHz in other part of code. (SYS_PLL_PFD1_DIV2 800/2 MHz divided by 3) */
    /* Configure TPM4 clock root to 8 MHz, SYS_PLL_PFD0 1000 MHz divided by 125, turn on clock */
    InitPeriphClock(CCM_TARGET_TPM4, CCM_LPCG_TPM4, 1U, 125U);
}

/**
  Initialize USDHC blocks and perform required pin-muxing.
  InitPeriphClock(,,ClockSource,Divider)
  ClockSource:
    00 - OSC_24M_CLK
    01 - SYS_PLL_PFD0    1000 MHz
    10 - SYS_PLL_PFD1     800 MHz
    11 - SYS_PLL_PFD2     625 MHz
  Divider:
   1   - Div by 1
   2   - Div by 2
   ..
   256 - Div by 256
**/
VOID UsdhcInit()
{
    /* Configure USDHC1 clock root to 400 MHz, SYS_PLL_PFD1 800 MHz divided by 2 */
    InitPeriphClock(CCM_TARGET_USDHC1, CCM_LPCG_USDHC1, 2U, 2);
    /* Configure USDHC2 clock root to 200 MHz, SYS_PLL_PFD1 800 MHz divided by 2 */
    InitPeriphClock(CCM_TARGET_USDHC2, CCM_LPCG_USDHC2, 2U, 2);
    // A1 silicon, ERR052021 fix
    IOMUXC1_SW_MUX_CTL_PAD_SD1_CMD   |= 0x10;
    IOMUXC1_SW_MUX_CTL_PAD_SD1_DATA0 |= 0x10;
    IOMUXC1_SW_MUX_CTL_PAD_SD1_DATA1 |= 0x10;
    IOMUXC1_SW_MUX_CTL_PAD_SD1_DATA2 |= 0x10;
    IOMUXC1_SW_MUX_CTL_PAD_SD1_DATA3 |= 0x10;
    IOMUXC1_SW_MUX_CTL_PAD_SD1_DATA4 |= 0x10;
    IOMUXC1_SW_MUX_CTL_PAD_SD1_DATA5 |= 0x10;
    IOMUXC1_SW_MUX_CTL_PAD_SD1_DATA6 |= 0x10;
    IOMUXC1_SW_MUX_CTL_PAD_SD1_DATA7 |= 0x10;

    /* 200 mHz pad settings */
    IOMUXC1_SW_PAD_CTL_PAD_SD1_CLK    = 0x15FE;
    IOMUXC1_SW_PAD_CTL_PAD_SD1_CMD    = 0x13FE;
    IOMUXC1_SW_PAD_CTL_PAD_SD1_DATA0  = 0x13FE;
    IOMUXC1_SW_PAD_CTL_PAD_SD1_DATA1  = 0x13FE;
    IOMUXC1_SW_PAD_CTL_PAD_SD1_DATA2  = 0x13FE;
    IOMUXC1_SW_PAD_CTL_PAD_SD1_DATA3  = 0x13FE;
    IOMUXC1_SW_PAD_CTL_PAD_SD1_DATA4  = 0x13FE;
    IOMUXC1_SW_PAD_CTL_PAD_SD1_DATA5  = 0x13FE;
    IOMUXC1_SW_PAD_CTL_PAD_SD1_DATA6  = 0x13FE;
    IOMUXC1_SW_PAD_CTL_PAD_SD1_DATA7  = 0x13FE;
    IOMUXC1_SW_PAD_CTL_PAD_SD1_STROBE = 0x15FE;


}

/**
  Initialize DIPLAY_MIX and perform required pin-muxing.
**/
VOID DisplayInit()
{
}

/**
  Initialize VPU and perform required pin-muxing.
**/
VOID VpuInit()
{
}

/**
  Initialize CAN modules on the SOC and perform required pin-muxing
**/
#define CAN_PAD_CTRL (IOMUXC_PAD_PDE_ENABLE | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_DSE_R0_DIV_4 )
VOID CanInit()
{
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO25 = IOMUXC_MUX_ALT2; // CAN2_TX -> PAD_GPIO_IO25
    IOMUXC1_SW_MUX_CTL_PAD_GPIO_IO27 = IOMUXC_MUX_ALT2; // CAN2_RX -> PAD_GPIO_IO27
    IOMUXC1_CAN2_IPP_IND_CANRX_SELECT_INPUT  = 0x01U;   // GPIO_IO27 ALT2
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO25 = CAN_PAD_CTRL;
    IOMUXC1_SW_PAD_CTL_PAD_GPIO_IO27 = CAN_PAD_CTRL;

    /* Configure CAN2 clock root to 40 MHz, SYS_PLL_PFD1_DIV2 400 MHz divided by 10, turn on clock */
    InitPeriphClock(CCM_TARGET_CAN2, CCM_LPCG_CAN2, 2U, 10U);
}

/**
  Initialize PCAL6524 IO expanders.
**/
VOID IOExpanderPcal6524Init()
{
  EFI_STATUS Status = RETURN_SUCCESS;
  UINT8 Data[3];
  UINT8 Mask[3];
  UINT8 Zero[3] = {0};
  UINT8 Data1OutSet, Data1OutClear, Data2OutSet, Data2OutClear;

  do {
    /* CTP_RST = 1, SD3_RTS_B = 0, ENET1_RST_B = 0, ENET2_RST_B = 0, M2_RST_B = 0 */
    Data[0] = 0x0U;
    Data[1] = 0x0U;
    Data[2] = EXP_P2_1_CTP_RST;
    Mask[0] = 0;
    Mask[1] = Data[1] | EXP_P1_4_SD3_RTS_B; // | EXP_P1_7_ENET1_RST_B;
    Mask[2] = Data[2] | EXP_P2_2_M2_RST_B; // | EXP_P2_0_ENET2_RST_B;
    Status = PCAL6524_ConfigurePins(&Pcal6524I2cConfig, Mask, Zero, Zero, Data);
    CHECK_PCAL6524_I2C_TRANSACTION_STATUS(Status, "PCAL6524 IO Expander configuration #1", End);
    /* Delay 10ms */
    MicroSecondDelay(10 * 1000);
    /* CTP_RST = 0 and configuration of other output pins */
    Data1OutSet = EXP_P1_4_SD3_RTS_B | EXP_P1_5_EXT1_PWREN | EXP_P1_6_EXT2_PWREN | EXP_P1_7_ENET1_RST_B;
    Data1OutClear = 0x0U;
    Data2OutSet = EXP_P2_0_ENET2_RST_B | EXP_P2_2_M2_RST_B | EXP_P2_3_M2_DIS2_B | EXP_P2_4_M2_DIS1_B;
    Data2OutClear = EXP_P2_1_CTP_RST;
    Data[0] = 0x0U;
    Data[1] = Data1OutSet;
    Data[2] = Data2OutSet;
    Mask[0] = 0;
    Mask[1] = Data1OutSet | Data1OutClear;
    Mask[2] = Data2OutSet | Data2OutClear;
    Status = PCAL6524_ConfigurePins(&Pcal6524I2cConfig, Mask, Zero, Zero, Data);
    CHECK_PCAL6524_I2C_TRANSACTION_STATUS(Status, "PCAL6524 IO Expander configuration #2", End);
  } while (FALSE);

End:
  return;
}

/**
  Initialize ADP5585 IO expanders.
**/
VOID IOExpanderAdp5585Init()
{
  EFI_STATUS Status = RETURN_SUCCESS;
  UINT8 Data[2];
  UINT8 Mask[2];
  UINT8 Data0OutSet, Data0OutClear, Data1OutSet, Data1OutClear;

  do {
    Data0OutSet = EXP_R0_CSI_RST | EXP_R1_AUD_PWREN;
    Data0OutClear = EXP_R2_PDM_MQS;
    Data1OutSet = EXP_C4_DSI_CTP_RST | EXP_C3_LVDS_BLT_EN;
    Data1OutClear = EXP_C0_CAN_STDBY;
    // Set EXP_SEL signal
    #if FixedPcdGet32(PcdMX93EXPSelSetting)
        // EXP_SEL = 1: SPI3, SAI3, CAN2 to J1001 (RPi/EXP GPIO)
        Data0OutSet |= EXP_R4_EXP_SEL;
    #else
        // EXP_SEL = 0: SPI3 to M.2, SAI3 to CODEC, CAN2 to J1101
        Data0OutClear |= EXP_R4_EXP_SEL;
    #endif
    Data[0] = Data0OutSet;
    Data[1] = Data1OutSet;
    Mask[0] = Data0OutSet | Data0OutClear;
    Mask[1] = Data1OutSet | Data1OutClear;
    Status = ADP5585_ConfigurePins(&Adp5585I2cConfig, Mask, Mask, Data);
    CHECK_ADP5585_I2C_TRANSACTION_STATUS(Status, "ADP5585 IO Expander configuration", End);
  } while (FALSE);

End:
  return;
}

/**
  Initialize I2C IO expanders.
**/
VOID IOExpandersInit()
{
  IOExpanderPcal6524Init();
  IOExpanderAdp5585Init();
}

/**
  Initialize controllers that must setup at the early stage
**/
RETURN_STATUS ArmPlatformInitialize(IN UINTN MpId)
{
  if (!ArmPlatformIsPrimaryCore (MpId)) {
    return RETURN_SUCCESS;
  }
  // Initialize debug serial port
  SerialPortInitialize ();
  SerialPortWrite ((UINT8 *)SERIAL_DEBUG_PORT_INIT_MSG, (UINTN)sizeof(SERIAL_DEBUG_PORT_INIT_MSG));
  
  UngateClocks();
  GpioInit();
  I2cInit();
  UartInit();
  IOExpandersInit();
  CanInit();
  USBInit();
  EnetInit();
  AudioInit();
  PwmInit();
  UsdhcInit();
  DisplayInit();
  VpuInit();
  SpiInit();
  CameraInit();

  return RETURN_SUCCESS;
}

/**
  Return the current Boot Mode. This function returns the boot reason on the platform
**/
EFI_BOOT_MODE ArmPlatformGetBootMode (VOID)
{
  return BOOT_WITH_FULL_CONFIGURATION;
}
