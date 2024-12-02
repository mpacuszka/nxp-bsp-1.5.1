/*
 * Copyright 2022-2024 NXP
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
#include "iMX8.h"
#include <Library/iMX8xScfwLib/main/ipc.h>
#include <Library/iMX8xScfwLib/main/sci.h>
#include <svc/pm/pm_api.h>
#include <svc/misc/misc_api.h>
#include <svc/pad/pad_api.h>
#include <svc/imx8qx_pads.h>
#include <IMX8Usb.h>
#include <iMXI2cLib.h>

/*
   The PWM output signals are shared with UART1 signals connected to the M.2 connector
   and thus conflict with the WIFI module on the MX8QXP_MEK CPU board.
   Enable PWM outputs by defining the symbol below only if the WIFI module is not present.
 */
// #define PWM_OUT_ENABLE 1


#define RdReg16(_RegAddr)            ((UINT16)*((volatile UINT16*)(void *)(uint64_t)(_RegAddr)))
#define WrReg16(_RegAddr, _RegVal)   *((volatile UINT16*)(void *)(uint64_t)(_RegAddr)) = _RegVal;
#define RdReg32(_RegAddr)            ((UINT32)*((volatile uint32_t*)(void *)(uint64_t)(_RegAddr)))
#define WrReg32(_RegAddr, _RegVal)   *((volatile uint32_t*)(void *)(uint64_t)(_RegAddr)) = _RegVal;

#define PORT_EXP_RST_PIN                    1

#define IMX_I2C1_BASE                       0x5a810000
#define IMX_I2C15_BASE                      0x37230000

#define IMX_PCA9646_I2C_SWITCH_I2C_ADDRESS  0x71
#define IMX_PCA9646_I2C_SWITCH_I2C_BASE     IMX_I2C1_BASE

#define IMX_PCA9557_IO_EXP_A_I2C_ADDRESS    0x1A
#define IMX_PCA9557_IO_EXP_A_I2C_BASE       IMX_I2C1_BASE

#define IMX_PCA9557_IO_EXP_B_I2C_ADDRESS    0x1D
#define IMX_PCA9557_IO_EXP_B_I2C_BASE       IMX_I2C1_BASE

#define IMX_PCA6416_IO_EXP_BB_I2C_ADDRESS   0x20
#define IMX_PCA6416_IO_EXP_BB_I2C_BASE      IMX_I2C15_BASE

#define MPL3115A2_I2C_ADDRESS           0x60
#define MPL3115A2_I2C_BASE              IMX_I2C1_BASE
#define MPL3115A2_CTRL_REG3             0x28

/* MPL3115A2 CTRL_REG3 configuration */
#define MPL3115A2_CTRL_REG3_IPOL1_AH    0x20 /*INT1 pin Active High*/
#define MPL3115A2_CTRL_REG3_IPOL1_AL    0x00 /*INT1 pin Active Low*/
#define MPL3115A2_CTRL_REG3_PPOD1_OD    0x10 /*INT1 pin Open Drain*/
#define MPL3115A2_CTRL_REG3_PPOD1_IP    0x00 /*INT1 pin Internal PullUp*/
#define MPL3115A2_CTRL_REG3_IPOL2_AH    0x02 /*INT2 pin Active High*/
#define MPL3115A2_CTRL_REG3_IPOL2_AL    0x00 /*INT2 pin Active Low*/
#define MPL3115A2_CTRL_REG3_PPOD2_OD    0x01 /*INT2 pin Open Drain*/
#define MPL3115A2_CTRL_REG3_PPOD2_IP    0x00 /*INT2 pin Internal PullUp*/

/* PCA9646 channel configuration */
#define I2C_SWITCH_CONTROL_SCL_DIRECTION   0x80
#define I2C_SWITCH_CONTROL_B3              0x08
#define I2C_SWITCH_CONTROL_B2              0x04
#define I2C_SWITCH_CONTROL_B1              0x02
#define I2C_SWITCH_CONTROL_B0              0x01

/* PCA9557 registers */
#define EXP_INPUT_PORT_REG                 0x00
#define EXP_OUTPUT_PORT_REG                0x01
#define EXP_POLARITY_INVERSION_REG         0x02
#define EXP_CONFIGURATION_REG              0x03

/* PCA6416 registers */
#define EXP_INPUT_PORT_0_REG               0x00
#define EXP_INPUT_PORT_1_REG               0x01
#define EXP_OUTPUT_PORT_0_REG              0x02
#define EXP_OUTPUT_PORT_1_REG              0x03
#define EXP_POLARITY_INVERSION_PORT_0_REG  0x04
#define EXP_POLARITY_INVERSION_PORT_1_REG  0x05
#define EXP_CONFIGURATION_PORT_0_REG       0x06
#define EXP_CONFIGURATION_PORT_1_REG       0x07

/* IO Expander A output pins - Cpu Card */
#define EXP_A_USB_TYPEC_EN                 0x80
#define EXP_A_MIPI_DSI0_EN                 0x40
#define EXP_A_SENS_RST_B                   0x20
#define EXP_A_CPU_ENET_PHY_RST_B           0x10
#define EXP_A_PER_RST_B                    0x08
#define EXP_A_WIFI_DISABLE_B               0x04
#define EXP_A_BT_DISABLE_B                 0x02
#define EXP_A_WIFI_EN                      0x01

/* IO Expander B output pins - Cpu Card */
#define EXP_B_MIPI_DSI1_EN                 0x80
#define EXP_B_BB_GPIO_EXP3                 0x40
#define EXP_B_BB_GPIO_EXP2                 0x20
#define EXP_B_BB_GPIO_EXP1                 0x10
#define EXP_B_BB_ARD_MIK_RST_B             0x08
#define EXP_B_BB_USB_OTG1_PWR_ON           0x04
#define EXP_B_BB_AUDIN_RST_B               0x02
#define EXP_B_TP36                         0x01

/* IO CAN Expander output pins - port0 - Base Board */
#define BB_CAN_EXP_MLB_PWDN                0x80
#define BB_CAN_EXP_CAN2_STB_B              0x40
#define BB_CAN_EXP_CAN01_STB_B             0x20
#define BB_CAN_EXP_CAN2_EN                 0x10
#define BB_CAN_EXP_CAN01_EN                0x08
#define BB_CAN_EXP_CAN2_WAKE_3V3_B         0x04
#define BB_CAN_EXP_CAN1_WAKE_3V3_B         0x02
#define BB_CAN_EXP_CAN0_WAKE_3V3_B         0x01

/* IO CAN Expander output pins - port1 - Base Board */
#define BB_CAN_EXP_QM_MERC_EN              0x80
#define BB_CAN_EXP_AUD_SW1_SEL             0x40
#define BB_CAN_EXP_CAN2_INH3V3             0x20
#define BB_CAN_EXP_CAN1_INH3V3             0x10
#define BB_CAN_EXP_CAN0_INH3V3             0x08
#define BB_CAN_EXP_CAN2_ERR_B              0x04
#define BB_CAN_EXP_CAN1_ERR_B              0x02
#define BB_CAN_EXP_CAN0_ERR_B              0x01

IMX_I2C_CONTEXT Pca9646I2cSwitchConfig =
{
  (uintptr_t)IMX_PCA9646_I2C_SWITCH_I2C_BASE, /* Base address of the I2C used for communication with PCA9646 */
  0,                                    /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  24000000,                             /* 24Mhz I2C ReferenceFreq */
  400000,                               /* 400KHz required TargetFreq */
  IMX_PCA9646_I2C_SWITCH_I2C_ADDRESS,   /* PCA6416 SlaveAddress */
  100000,                               /* TimeoutInUs ignored - not implemented in LPI2C driver */
};

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

IMX_I2C_CONTEXT Pca6416IOExpBBI2CConfig =
{
  (uintptr_t)IMX_PCA6416_IO_EXP_BB_I2C_BASE, /* Base address of the I2C used for communication with PCA6416 Exp CAN on BaseBoard */
  0,                                    /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  24000000,                             /* 24Mhz I2C ReferenceFreq */
  100000,                               /* 100KHz required TargetFreq */
  IMX_PCA6416_IO_EXP_BB_I2C_ADDRESS,    /* PCA6416 Exp CAN on BaseBoard SlaveAddress */
  100000,                               /* TimeoutInUs ignored - not implemented in LPI2C driver */
};

IMX_I2C_CONTEXT Mpl3115a2PsI2CConfig =
{
  (uintptr_t)MPL3115A2_I2C_BASE,    /* Base address of the I2C used for communication with PCA6416 Exp CAN on BaseBoard */
  0,                                /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  24000000,                         /* 24Mhz I2C ReferenceFreq */
  400000,                           /* 400KHz required TargetFreq */
  MPL3115A2_I2C_ADDRESS,            /* PCA6416 Exp CAN on BaseBoard SlaveAddress */
  100000,                           /* TimeoutInUs ignored - not implemented in LPI2C driver */
};

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

/* OTP fuses */
#define IMX_SC_OCOTP_MAC0 708
#define IMX_SC_OCOTP_MAC1 709
#define IMX_SC_OCOTP_MAC2 710
#define IMX_SC_OCOTP_MAC3 711

/** Structure encapsulating all pad configuration */
typedef struct {
    sc_pad_t                pad;    /**< PAD name */
    uint8_t                 mux;    /**< PAD mux (pad alternate function) */
    sc_pad_config_t         cfg;    /**< PAD configuration - Normal, Open drain, ... */
    sc_pad_iso_t            iso;    /**< PAD isolation */
    sc_pad_28fdsoi_dse_t    dse;    /**< PAD drive strength */
    sc_pad_28fdsoi_ps_t     ps;     /**< PAD pull selection */
} imx_pad_t;

/** Definition of PADS alternate functions */
#define IMX_PAD_ALT_0   0   /**< PAD alternate function 0 */
#define IMX_PAD_ALT_1   1   /**< PAD alternate function 1 */
#define IMX_PAD_ALT_2   2   /**< PAD alternate function 2 */
#define IMX_PAD_ALT_3   3   /**< PAD alternate function 3 */
#define IMX_PAD_ALT_4   4   /**< PAD alternate function 3 */

/** MAC addresses encapsulation withon global data page */
typedef struct {
  UINT8 Mac0Id;
  UINT8 Mac0Valid;
  UINT8 Mac0Address[6];
  UINT8 Mac1Id;
  UINT8 Mac1Valid;
  UINT8 Mac1Address[6];
} __attribute__((packed)) GLOBAL_PAGE_MAC_ADDR;

/** Offset of the MAC address record within global page */
#define GLOBAL_PAGE_MAC_ADDR_OFFSET 0x800

#define CHECK_I2C_TRANSACTION_STATUS(status, message, label) \
            if (status != EFI_SUCCESS) { \
              DEBUG((DEBUG_ERROR, "I2C error. Register: %a, I2CStatus: %d\n", message, status)); \
              goto label; \
            }

/*
 * Probe I2C device. This method can be used to check if the given device exists/responds on the I2C bus.
 */
EFI_STATUS ProbeI2CDevice(IMX_I2C_CONTEXT *I2cContext)
{
    UINT8 Dummy = 0;
    return iMXI2cWriteE(I2cContext, 0, 0, &Dummy, 1);
//    return iMXI2cWriteE(I2cContext, 0, 0, NULL, 0);
}

/*
 * I2C switch configuration. This method configures I2C switch PCA9646.
 */
EFI_STATUS I2CSwitch_Configure(IMX_I2C_CONTEXT *I2cContext, UINT8 confData)
{
  EFI_STATUS Status;

  Status = iMXI2cWriteE(I2cContext, 0, 0, &confData, 1);
  CHECK_I2C_TRANSACTION_STATUS(Status, "I2C Switch configuration", End);
End:
  return Status;
}

/*
 * IO one-port expander configuration. This method configures IO expander PCA9557.
 */
EFI_STATUS IOExp_PCA9557_Configure(IMX_I2C_CONTEXT *I2cContext,
                                   UINT8 *PinMask, UINT8 *InvVal,
                                   UINT8 *ConfVal, UINT8 *OutVal)
{
  EFI_STATUS Status;
  UINT8 Register, Data;

  /* Inversion register setting */
  Register = EXP_POLARITY_INVERSION_REG;
  Status = iMXI2cReadE(I2cContext, Register, 1, &Data, 1);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA9557 IO Expander - reading POLARITY_INVERSION_REG", End);
  Data = (Data & ~(*PinMask)) | (*PinMask & *InvVal);
  Status = iMXI2cWriteE(I2cContext, Register, 1, &Data, 1);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA9557 IO Expander - writing POLARITY_INVERSION_REG", End);
  /* Output port settings */
  Register = EXP_OUTPUT_PORT_REG;
  Status = iMXI2cReadE(I2cContext, Register, 1, &Data, 1);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA9557 IO Expander - reading OUTPUT_PORT_REG", End);
  Data = (Data & ~(*PinMask)) | (*PinMask & *OutVal);
  Status = iMXI2cWriteE(I2cContext, Register, 1, &Data, 1);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA9557 IO Expander - writing OUTPUT_PORT_REG", End);
  /* Configuration register setting */
  Register = EXP_CONFIGURATION_REG;
  Status = iMXI2cReadE(I2cContext, Register, 1, &Data, 1);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA9557 IO Expander - reading CONFIGURATION_REG", End);
  Data = (Data & ~(*PinMask)) | (*PinMask & *ConfVal);
  Status = iMXI2cWriteE(I2cContext, Register, 1, &Data, 1);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA9557 IO Expander - writing CONFIGURATION_REG", End);
End:
  return Status;
}

/*
 * IO two-port expander configuration. This method configures IO expander PCA6416.
 */
EFI_STATUS IOExp_PCA6416_Configure(IMX_I2C_CONTEXT *I2cContext,
                                   UINT8 *PinMask, UINT8 *InvVal,
                                   UINT8 *ConfVal, UINT8 *OutVal)
{
  EFI_STATUS Status;
  UINT8 Register, Data[2];

  /* Inversion register setting */
  Register = EXP_POLARITY_INVERSION_PORT_0_REG;
  Status = iMXI2cReadE(I2cContext, Register, 1, Data, 2);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA6416 IO Expander - reading POLARITY_INVERSION_REG", End);
  Data[0] = (Data[0] & ~(PinMask[0])) | (PinMask[0] & InvVal[0]);
  Data[1] = (Data[1] & ~(PinMask[1])) | (PinMask[1] & InvVal[1]);
  Status = iMXI2cWriteE(I2cContext, Register, 1, Data, 2);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA6416 IO Expander - writing POLARITY_INVERSION_REG", End);
  /* Output port settings */
  Register = EXP_OUTPUT_PORT_0_REG;
  Status = iMXI2cReadE(I2cContext, Register, 1, Data, 2);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA6416 IO Expander - reading OUTPUT_PORT_REG", End);
  Data[0] = (Data[0] & ~(PinMask[0])) | (PinMask[0] & OutVal[0]);
  Data[1] = (Data[1] & ~(PinMask[1])) | (PinMask[1] & OutVal[1]);
  Status = iMXI2cWriteE(I2cContext, Register, 1, Data, 2);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA6416 IO Expander - writing OUTPUT_PORT_REG", End);
  /* Configuration register setting */
  Register = EXP_CONFIGURATION_PORT_0_REG;
  Status = iMXI2cReadE(I2cContext, Register, 1, Data, 2);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA6416 IO Expander - reading CONFIGURATION_REG", End);
  Data[0] = (Data[0] & ~(PinMask[0])) | (PinMask[0] & ConfVal[0]);
  Data[1] = (Data[1] & ~(PinMask[1])) | (PinMask[1] & ConfVal[1]);
  Status = iMXI2cWriteE(I2cContext, Register, 1, Data, 2);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA6416 IO Expander - writing CONFIGURATION_REG", End);
End:
  return Status;
}

/*!
 * IO two-port expander output value setting. This method sets IO expander PCA6416 output.
 */
EFI_STATUS IOExp_PCA6416_SetVal(IMX_I2C_CONTEXT *I2cContext,
                                UINT8 *PinMask, UINT8 *OutVal)
{
  EFI_STATUS Status;
  UINT8 Register, Data[2];

  /* Output port settings */
  Register = EXP_OUTPUT_PORT_0_REG;
  Status = iMXI2cReadE(I2cContext, Register, 1, Data, 2);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA6416 IO Expander - reading OUTPUT_PORT_REG", End);
  Data[0] = (Data[0] & ~(PinMask[0])) | (PinMask[0] & OutVal[0]);
  Data[1] = (Data[1] & ~(PinMask[1])) | (PinMask[1] & OutVal[1]);
  Status = iMXI2cWriteE(I2cContext, Register, 1, Data, 2);
  CHECK_I2C_TRANSACTION_STATUS(Status, "PCA6416 IO Expander - writing OUTPUT_PORT_REG", End);
End:
  return Status;
}

/**
 * I.MX8 pad config routine using SCFW.
 */
static int set_28fdsoi_pad(sc_ipc_t ipc, imx_pad_t *pads, uint32_t size)
{
    uint32_t i;
    sc_err_t err;

    for (i = 0; i < size; i++) {
        err = sc_pad_set_mux(ipc, pads[i].pad, pads[i].mux, pads[i].cfg, pads[i].iso);
        if (err != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "sc_pad_set_mux failed for pad %u. SC status: %s\n", pads[i].pad, sc_status2str(err)));
            return -1;
        }
        err = sc_pad_set_gp_28fdsoi(ipc, pads[i].pad, pads[i].dse, pads[i].ps);
        if (err != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "sc_pad_set_gp_28fdsoi failed for pad %u. SC status: %s\n", pads[i].pad, sc_status2str(err)));
            return -1;
        }
    }
    return 0;
}

static void sc_pm_err_resource_power_mode(sc_err_t err, sc_rsrc_t resource, sc_pm_power_mode_t mode)
{
  DEBUG ((DEBUG_ERROR, "sc_pm_set_resource_power_mode error:%d resource:%d power:%d\n",
          (int)err, (int)resource, (int)mode));
}

static void sc_pm_err_clock_enable(sc_err_t err, sc_rsrc_t resource, sc_pm_clk_t clk,
                                   sc_bool_t enable, sc_bool_t autog)
{
  DEBUG ((DEBUG_ERROR, "sc_pm_clock_enable error:%d resource:%d clk:%d enable:%d autog:%d\n",
          (int)err, (int)resource, (int)clk, (int)enable, (int)autog));
}

static void sc_pm_err_clock_rate(sc_err_t err, sc_rsrc_t resource, sc_pm_clk_t clk,
                                 sc_pm_clock_rate_t rate)
{
  DEBUG ((DEBUG_ERROR, "sc_pm_set_clock_rate error:%d resource:%d clk:%d rate:%d\n",
          (int)err, (int)resource, (int)clk, (int)rate));
}

/** Configures peripheral power and clock settings.
**/
static sc_err_t init_periph_power_clock_freq(sc_ipc_t ipc, sc_rsrc_t resource, sc_pm_clk_t clock, sc_pm_clock_rate_t *rate,
                                      sc_pm_clock_rate_t *get_rate)
{
    sc_err_t err = SC_ERR_NONE;

    do {
        /* Power on */
        err = sc_pm_set_resource_power_mode(ipc, resource, SC_PM_PW_MODE_ON);
        if (err != SC_ERR_NONE) {
            sc_pm_err_resource_power_mode(err, resource, SC_PM_PW_MODE_ON);
            break;
        }
        if (rate != NULL) {
            /* Disable clock */
            err = sc_pm_clock_enable(ipc, resource, clock, false, false);
            if (err != SC_ERR_NONE) {
                sc_pm_err_clock_enable(err, resource, clock, false, false);
                break;
            }
            /* Set clock frequency */
            err = sc_pm_set_clock_rate(ipc, resource, clock, rate);
            if (err != SC_ERR_NONE) {
                sc_pm_err_clock_rate(err, resource, clock, *rate);
                break;
            }
        }
        err = sc_pm_clock_enable(ipc, resource, clock, true, false);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, resource, clock, true, false);
            break;
        }
        err = sc_pm_get_clock_rate(ipc, resource, clock, get_rate);
        /* For unavailable clocks it returns SC_ERR_NONE */
        if (err == SC_ERR_UNAVAILABLE) {
            err = SC_ERR_NONE;
            break;
        }
        if (err != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "sc_pm_get_clock_rate error:%d resource:%d clk:%d rate%d\n",
                   (int)err, (int)resource, (int)clock, (int)*get_rate));
        }
    } while (0);
    return err;
}

VOID ArmPlatformGetPlatformPpiList (OUT UINTN *PpiListSize, OUT EFI_PEI_PPI_DESCRIPTOR **PpiList)
{
  *PpiListSize = sizeof(gPlatformPpiTable);
  *PpiList     = gPlatformPpiTable;
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
    sc_err_t err = SC_ERR_NONE;

    { // Power domains
        int i;
        sc_rsrc_t powerDomains[] = {SC_R_ISI_CH0,
                                    SC_R_CSI_0
                                    // SC_R_CSI_0_I2C_0, Done inside I2cInit.
                                   }; 
        for (i = 0; i < sizeof(powerDomains) / sizeof(powerDomains[0]); i++) {
            err = sc_pm_set_resource_power_mode(SC_IPC_HDL, powerDomains[i], SC_PM_PW_MODE_ON);
            if (err != SC_ERR_NONE) {
                sc_pm_err_resource_power_mode(err, powerDomains[i], SC_PM_PW_MODE_ON);
                continue;
            }
        }
    }
    { // Clocks
        typedef struct {
          sc_rsrc_t resource;
          sc_pm_clk_t clk_id;
          sc_pm_clock_rate_t req_frequency; // in Hz; 0 means set to default
        } clk_cfg_t;
        
        uint32_t i;
        sc_err_t err = SC_ERR_NONE;
        sc_rsrc_t resource;
        sc_pm_clk_t clk_id;
        sc_pm_clock_rate_t req_frequency;
        sc_pm_clock_rate_t get_rate = 0;
        
        clk_cfg_t instances[] = {
          {SC_R_CSI_0, SC_PM_CLK_PER, 360000000U}, // Set CSI0_CORE_CLK to 360 MHz.
          {SC_R_CSI_0, SC_PM_CLK_MISC, 72000000U}, // Set CSI0_ESC_CLK to 72 MHz.
          // {SC_R_CSI_0_I2C_0, SC_PM_CLK_PER, 24000000U}, Done inside I2cInit.
        };
        uint8_t instances_num = (uint8_t)(sizeof(instances) / sizeof(instances[0]));
            
        // Enable selected clocks with required frequencies
        for (i = 0; i < instances_num; i++) {
            req_frequency = instances[i].req_frequency;
            resource = instances[i].resource;
            clk_id = instances[i].clk_id;
            DEBUG ((EFI_D_VERBOSE, "%s: init_periph_power_clock_freq %d of %d\n", __func__, i, instances_num));
            if (init_periph_power_clock_freq(SC_IPC_HDL, resource, clk_id, ((req_frequency == 0)? NULL : &req_frequency), &get_rate) != SC_ERR_NONE) {
                continue;
            }
            err = sc_pm_get_clock_rate(SC_IPC_HDL, resource, clk_id, &get_rate);
            if ((req_frequency != 0) && (get_rate != req_frequency)) {
              DEBUG ((DEBUG_ERROR, "%s: Frequency check failed - req_frequency: %dHz (requested: %dHz) resource:%d clk:%d\n", __func__,
                     (int)get_rate, (int)req_frequency, (int)resource, (int)clk_id));
            }
        }
    }
    // Set RXCDRP(LP-RX threshold voltage ) and RXLPRP(LP-CD threshold voltage) values for MIPI CSI0.
    err = sc_misc_set_control(SC_IPC_HDL, SC_R_CSI_0, SC_C_CALIB1 , 2);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_CSI_0, SC_C_CALIB1);
        ASSERT (err == SC_ERR_NONE);
    }
    err = sc_misc_set_control(SC_IPC_HDL, SC_R_CSI_0, SC_C_CALIB2 , 2);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_CSI_0, SC_C_CALIB2);
        ASSERT (err == SC_ERR_NONE);
    }
    
    // Reset PHY
    err = sc_misc_set_control(SC_IPC_HDL, SC_R_CSI_0, SC_C_MIPI_RESET, 1);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_CSI_0, SC_C_MIPI_RESET);
        ASSERT (err == SC_ERR_NONE);
    }

    imx_pad_t pads[] = {
        // Route SC_P_MIPI_CSI0_MCLK_OUT to provide 24MHz reference clock for OV5640 camera.
        {SC_P_MIPI_CSI0_MCLK_OUT, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_2MA, SC_PAD_28FDSOI_PS_NONE}, /* MIPI_CSI0.ACM.MCLK_OUT */

        // CSI_nRST, CSI_POWER_DOWN
        {SC_P_MIPI_CSI0_GPIO0_00, IMX_PAD_ALT_4, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /*  LSIO.GPIO3.IO08 */
        {SC_P_MIPI_CSI0_GPIO0_01, IMX_PAD_ALT_4, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /*  LSIO.GPIO3.IO07 */
        
        // I2C pad init done in I2cInit.
    };

    err = set_28fdsoi_pad(SC_IPC_HDL, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
    if (err != SC_ERR_NONE) {
      err = sc_ipc_reset(SC_IPC_HDL);
    }
    ASSERT (err == SC_ERR_NONE);

    // CSI_nRST (MIPI_CSI0_GPIO0_00) = 1, CSI_POWER_DOWN(MIPI_CSI0_GPIO0_01) = 0
    MIPI_CSI__GPIO_DR = GPIO_DR_DR(0x01 << 0);
    // Set output direction
    MIPI_CSI__GPIO_GDIR = GPIO_GDIR_GDIR(0x1 << 1) | (0x1 << 0);
}

/**
  Initialize GPIO modules on the SOC and perform required pin-muxing
**/
VOID GpioInit ()
{
    sc_err_t sc_status;
    sc_rsrc_t index;

    for (index = SC_R_GPIO_0; index <= SC_R_GPIO_5; index++) {
        /* Power up all GPIO[0-5] devices */
        if ((sc_status = sc_pm_set_resource_power_mode(SC_IPC_HDL, index, SC_PM_PW_MODE_ON)) != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR,"GpioInit: sc_pm_set_resource_power_mode(SC_R_GPIO_%d, SC_PM_PW_MODE_ON) failed\n", index));
            return;
        }
    }

    for (index = SC_R_BOARD_R0; index <= SC_R_BOARD_R1; index++) {
        if ((sc_status = sc_pm_set_resource_power_mode(SC_IPC_HDL, index, SC_PM_PW_MODE_ON)) != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR,"GpioInit: sc_pm_set_resource_power_mode(SC_R_BOARD_R%d, SC_PM_PW_MODE_ON) failed\n", index));
            return;
        }
    }
}

#if FixedPcdGet32(PcdPcie1Enable)
/**
  Initialize PCI Express module on the SOC and perform required pin-muxing
**/
VOID PcieInit()
{
    sc_err_t err = SC_ERR_NONE;
    imx_pad_t pads[] = {
        {SC_P_PCIE_CTRL0_CLKREQ_B, IMX_PAD_ALT_4, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* LSIO.GPIO4.IO01 for PCIE_CLKREQ_N */
        {SC_P_PCIE_CTRL0_WAKE_B,   IMX_PAD_ALT_4, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* LSIO.GPIO4.IO02 for PCIE_WAKE_N */
        {SC_P_PCIE_CTRL0_PERST_B,  IMX_PAD_ALT_4, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* LSIO.GPIO4.IO00 for PCIE_RST_N */
    };

    /* Power up PCIe (HSIO) SERDES 1 */
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_SERDES_1, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_SERDES_1, SC_PM_PW_MODE_ON);
        ASSERT (err == SC_ERR_NONE);
    }
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_PCIE_B, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_PCIE_B, SC_PM_PW_MODE_ON);
        ASSERT (err == SC_ERR_NONE);
    }
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_HSIO_GPIO, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_HSIO_GPIO, SC_PM_PW_MODE_ON);
        ASSERT (err == SC_ERR_NONE);
    }
    err = set_28fdsoi_pad(SC_IPC_HDL, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
    if (err != SC_ERR_NONE) {
      err = sc_ipc_reset(SC_IPC_HDL);
    }
    ASSERT (err == SC_ERR_NONE);
    /* Enable ref. clock for PCIe card (EP) device and #PERST to 0 */
    LSIO__GPIO4_DR &= ~(GPIO_DR_DR(0x01 << 1) | GPIO_DR_DR(0x01 << 0));
    LSIO__GPIO4_GDIR |= (GPIO_GDIR_GDIR(0x01 << 1) | GPIO_GDIR_GDIR(0x01 << 0));
}
#endif

/**
  Initalize the Audio system
**/
VOID AudioInit(VOID)
{
    sc_err_t err;
    sc_pm_clock_rate_t frequency;
    sc_pm_clock_rate_t get_rate = 0;
    uint32_t i;
    sc_rsrc_t instances[] = {
                         SC_R_AUDIO_PLL_0,
                         //SC_R_AUDIO_PLL_1,
                         SC_R_AUDIO_CLK_0,
                         //SC_R_AUDIO_CLK_1,
                         SC_R_MCLK_OUT_0,
                         //SC_R_MCLK_OUT_1,
                         //SC_R_SAI_0,
                         SC_R_SAI_1,
                         //SC_R_SAI_2,
                         //SC_R_SAI_3,
                         //SC_R_SAI_4,
                         //SC_R_SAI_5,
                         //SC_R_ESAI_0,
                         //SC_R_ASRC_0,
                         //SC_R_ASRC_1,
                         //SC_R_GPT_5,
                         //SC_R_GPT_6,
                         //SC_R_GPT_7,
                         //SC_R_GPT_8,
                         //SC_R_GPT_9,
                         //SC_R_GPT_10,
                         //SC_R_SPDIF_0,
                         //SC_R_MQS_0
                        };

    do {
        /* Power on audio related power domains */
        for (i = 0; i < (sizeof(instances) / sizeof(sc_rsrc_t)); i++) {
            err = sc_pm_set_resource_power_mode(SC_IPC_HDL, instances[i], SC_PM_PW_MODE_ON);
            if (err != SC_ERR_NONE) {
                sc_pm_err_resource_power_mode(err, instances[i], SC_PM_PW_MODE_ON);
            }
        }
        /* Power up Audio PLL0 and configure for sample rates 44.1, 88.2, ... kHz */
        frequency = 722534400U;
        if ((err = init_periph_power_clock_freq(SC_IPC_HDL, SC_R_AUDIO_PLL_0, SC_PM_CLK_PLL, &frequency, &get_rate)) != SC_ERR_NONE) {
            break;
        }
        /* ACM: MCLK OUT0 SEL to AUD_REC_0 */
        WrReg32(0x59e20000, (RdReg32(0x59e20000) & 0x00000007) | 0x00000000);
        /* ACM: SAI1 MCLK SEL to AUD_PLL_DIV_0 */
        WrReg32(0x59ef0000, (RdReg32(0x59ef0000) & 0x00000003) | 0x00000000);
        /* AUD_PLL_DIV_CLK0 to 22.5792 MHz */
        frequency = 22579200U;
        if ((err = init_periph_power_clock_freq(SC_IPC_HDL, SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC0, &frequency, &get_rate)) != SC_ERR_NONE) {
            break;
        }
        /* AUD_REC_CLK0 to 22.5792 MHz, mainly used for MCLK Out pins */
        frequency = 22579200U;
        if ((err = init_periph_power_clock_freq(SC_IPC_HDL, SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC1, &frequency, &get_rate)) != SC_ERR_NONE) {
            break;
        }
        /* LPCG AUD REC CLK0 */
        WrReg32(0x59d00000, (RdReg32(0x59d00000) & 0x00000000) | 0x00000002);
        /* LPCG MCLKOUT0 */
        WrReg32(0x59d50000, (RdReg32(0x59d50000) & 0x00000000) | 0x00000002);
        /* LPCG_AUD_PLL_DIV_CLK0 */
        WrReg32(0x59d20000, (RdReg32(0x59d20000) & 0x00000000) | 0x00000002);
        /* LPCG SAI1 */
        WrReg32(0x59450000, (RdReg32(0x59450000) & 0x00000000) | 0x00020002);
    } while (FALSE);
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "Audio clocks init failed\n"));
    }

    /* Configure audio related pads */
    imx_pad_t audio_pads[] = {
        {SC_P_SAI1_RXC,      IMX_PAD_ALT_1, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   // TXD
        {SC_P_SAI1_RXD,      IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   // RXD
        {SC_P_SAI1_RXFS,     IMX_PAD_ALT_1, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_SPI0_CS1,      IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_MCLK_OUT0,     IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
    };
    err = set_28fdsoi_pad(SC_IPC_HDL, &audio_pads[0], sizeof(audio_pads) / sizeof(imx_pad_t));
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "Audio pads init failed\n"));
    }

}

/**
  Initialize I2C modules on the SOC and perform required pin-muxing
**/
VOID I2cInit()
{
    uint32_t i;
    sc_err_t err;
    sc_rsrc_t resource;
    sc_pm_clock_rate_t req_frequency, frequency, *frequencyPtr;
    sc_pm_clock_rate_t get_rate = 0;
    typedef struct {
        sc_rsrc_t resource;
        sc_pm_clock_rate_t frequency;       // in Hz; 0 means set to default
    } i2c_clk_cfg_t;
    i2c_clk_cfg_t instances[] = {
                                 //{SC_R_I2C_0, 0},          // I2C0
                                 {SC_R_I2C_1, 24000000U},          // I2C1
                                 //{SC_R_I2C_2, 0},          // I2C2
                                 //{SC_R_I2C_3, 0},          // I2C3
                                 {SC_R_CSI_0_I2C_0,  24000000U},   // I2C0_MIPI_CSI0 - Depends on CameraInit.
                                 //{SC_R_PI_0_I2C_0,  0},    // I2C0_PARALLEL
                                 {SC_R_MIPI_0_I2C_0, 0},   // I2C0_MIPI_LVDS0
                                 {SC_R_MIPI_1_I2C_0, 0},   // I2C0_MIPI_LVDS1
                                 {SC_R_M4_0_I2C, 24000000U},       // CM40_I2C
                                };
    uint8_t instances_num = (uint8_t)(sizeof(instances) / sizeof(i2c_clk_cfg_t));

    /* Enable INTMUX peripheral */
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_M4_0_INTMUX, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_M4_0_INTMUX, SC_PM_PW_MODE_ON);
    }
    /* Enable I2C clocks with required frequency */
    for (i = 0; i < instances_num; i++) {
        req_frequency = instances[i].frequency;
        frequency = req_frequency;
        resource = instances[i].resource;
        frequencyPtr = (req_frequency == 0)? NULL : &frequency;
        if (init_periph_power_clock_freq(SC_IPC_HDL, resource, SC_PM_CLK_PER, frequencyPtr, &get_rate) != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "I2CInit() init_periph_power_clock_freq() returned an error - frequency: %dHz (requested: %dHz) for resource:%d clk:%d\n",
                   (int)get_rate, (int)frequency, (int)resource, (int)SC_PM_CLK_PER));
            continue;
        }
        if ((req_frequency != 0) && (get_rate != req_frequency)) {
            DEBUG ((DEBUG_ERROR, "I2CInit() Frequency check failed - frequency: %dHz (requested: %dHz) resource:%d clk:%d\n",
                   (int)get_rate, (int)frequency, (int)resource, (int)SC_PM_CLK_PER));
        }
    }
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_CSI_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_CSI_0, SC_PM_PW_MODE_ON);
    } else {
        /* Enable LPI2C in MIPI_CSI Local Interrupt Steer. */
        IRQSTEER_CHn_MASK_REG(IRQSTEER_MIPI_CSI_BASE_PTR, 0) = 1U << 8;
    }
    
    /* Configure I2C pads */
    imx_pad_t i2c_pads[] = {
        {SC_P_USB_SS3_TC1, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_33V_4MA, SC_PAD_28FDSOI_PS_PU},       /* I2C1 SCL (SS3_I2C1_SCL) */
        {SC_P_USB_SS3_TC3, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_33V_4MA, SC_PAD_28FDSOI_PS_PU},       /* I2C1 SDA (SS3_I2C1_SDA) */
        {SC_P_MIPI_CSI0_I2C0_SCL, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_4MA, SC_PAD_28FDSOI_PS_PU},/* I2C0_MIPI_CSI0 SCL */
        {SC_P_MIPI_CSI0_I2C0_SDA, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_4MA, SC_PAD_28FDSOI_PS_PU},/* I2C0_MIPI_CSI0 SDA */
        {SC_P_MIPI_DSI0_I2C0_SCL, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU}, /* I2C0_MIPI_LVDS0 SCL */
        {SC_P_MIPI_DSI0_I2C0_SDA, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU}, /* I2C0_MIPI_LVDS0 SDA */
        {SC_P_MIPI_DSI1_I2C0_SCL, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU}, /* I2C0_MIPI_LVDS1 SCL */
        {SC_P_MIPI_DSI1_I2C0_SDA, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU}, /* I2C0_MIPI_LVDS1 SDA */
        {SC_P_ADC_IN1, IMX_PAD_ALT_1, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_12MA, SC_PAD_28FDSOI_PS_PU},          /* CM40_I2C SDA */
        {SC_P_ADC_IN0, IMX_PAD_ALT_1, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_12MA, SC_PAD_28FDSOI_PS_PU},          /* CM40_I2C SCL */
    };
    err = set_28fdsoi_pad(SC_IPC_HDL, &i2c_pads[0], sizeof(i2c_pads) / sizeof(imx_pad_t));
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "I2C pads init failed\n"));
    }

}

#define IMX_USB2OTG1_BASE                      0x5B0D0000
#define IMX_USB2PHY1_BASE                      0x5B100000

#define IMX_USB3_BASE                          0x5B110000
#define IMX_USB3_CTRL_BASE                     0x5B110000
#define IMX_USB3_CORE_BASE                     0x5B120000
#define IMX_USB3_PHY_BASE                      0x5B160000

/**
 * This function initializes USB PHY
 *
 * @param phy_addr USB_PHY base address.
 */
void init_usb2_phy(uint32_t phy_addr)
{
    /* Soft-reset USB PHY */
    WrReg32(phy_addr + IMX_USB2PHYx_CTRL_SET, IMX_USB2PHYx_CTRL_SFTRST_MASK);
    MicroSecondDelay(10);
    /* Remove clock gate and soft reset */
    WrReg32(phy_addr + IMX_USB2PHYx_CTRL_CLR, IMX_USB2PHYx_CTRL_SFTRST_MASK | IMX_USB2PHYx_CTRL_CLKGATE_MASK);
    MicroSecondDelay(10);
    /* Power up PHY */
    WrReg32(phy_addr + IMX_USB2PHYx_PWD, 0);
    /* Enable FS/LS device */
    WrReg32(phy_addr + IMX_USB2PHYx_CTRL_SET, (IMX_USB2PHYx_CTRL_ENAUTOSET_USBCLKS_MASK |
                                                 IMX_USB2PHYx_CTRL_ENAUTOCLR_PHY_PWD_MASK |
                                                 IMX_USB2PHYx_CTRL_ENAUTOCLR_CLKGATE_MASK |
                                                 IMX_USB2PHYx_CTRL_ENAUTO_PWRON_PLL_MASK  |
                                                 IMX_USB2PHYx_CTRL_ENUTMILEVEL3_MASK      |
                                                 IMX_USB2PHYx_CTRL_ENUTMILEVEL2_MASK));
}

/**
 * This function initializes USB PLL
 *
 * @param phy_addr USB_PHY base address.
 */
void init_usb2_pll(uint32_t phy_addr)
{
    unsigned int cnt = 100;

    WrReg32(phy_addr + IMX_USB2PHYx_PLL_SIC_SET, IMX_USB2PHYx_PLL_REG_ENABLE_MASK);   /* Enable regulator */
    MicroSecondDelay(25);
    WrReg32(phy_addr + IMX_USB2PHYx_PLL_SIC_SET, IMX_USB2PHYx_PLL_POWER_MASK);        /* Enable POWER */
    MicroSecondDelay(25);
    WrReg32(phy_addr + IMX_USB2PHYx_PLL_SIC_CLR, IMX_USB2PHYx_PLL_BYPASS_MASK);       /* Disable Bypass */
    MicroSecondDelay(1);
    WrReg32(phy_addr + IMX_USB2PHYx_PLL_SIC_SET, IMX_USB2PHYx_PLL_ENABLE_MASK);       /* Enable PLL */
    while (!(RdReg32(phy_addr + IMX_USB2PHYx_PLL_SIC) & IMX_USB2PHYx_PLL_LOCK_MASK) && (cnt > 0)){
        cnt--;
        MicroSecondDelay(10);
    }
    if(cnt == 0){
        DEBUG ((DEBUG_ERROR, "USB2.0 Init: Timeout waiting on PLL lock"));
    }
    WrReg32(phy_addr + IMX_USB2PHYx_PLL_SIC_SET, IMX_USB2PHYx_PLL_EN_USB_CLKS_MASK);  /* Enable USB Clocks */
}

/**
 * Reset USB3 core.
 *
 * @param phy_addr USB control register address.
 */
static void imx_usb3_reset_core(uint32_t phy_addr)
{
    WrReg32(phy_addr, RdReg32(phy_addr) | IMX_USB3_NON_CORE_ALL_SW_RESET);
    MicroSecondDelay(1);
}


/**
 * This function initializes USB3.0 PHY
 *
 * @param phy_addr USB_PHY base address.
 */
void init_usb3_phy(uint32_t phy_addr)
{
    /* This initialization is based on section "Bring-up initialization sequence" from USB3.0 SSPHY User Guide */
    WrReg16((IMX_USB3_PHY_BASE + 0xC800 * 4), 0x0830);
    WrReg16((IMX_USB3_PHY_BASE + 0x01E0 * 4), 0x0010);
    WrReg16((IMX_USB3_PHY_BASE + 0x0084 * 4), 0x00F0);
    WrReg16((IMX_USB3_PHY_BASE + 0x0085 * 4), 0x0018);
    WrReg16((IMX_USB3_PHY_BASE + 0x0094 * 4), 0x00D0);
    WrReg16((IMX_USB3_PHY_BASE + 0x0095 * 4), 0x4AAA);
    WrReg16((IMX_USB3_PHY_BASE + 0x0096 * 4), 0x0034);
    WrReg16((IMX_USB3_PHY_BASE + 0x0098 * 4), 0x01EE);
    WrReg16((IMX_USB3_PHY_BASE + 0x0099 * 4), 0x7F03);
    WrReg16((IMX_USB3_PHY_BASE + 0x0097 * 4), 0x0020);
    WrReg16((IMX_USB3_PHY_BASE + 0x01C2 * 4), 0x0000);
    WrReg16((IMX_USB3_PHY_BASE + 0x01C0 * 4), 0x0000);
    WrReg16((IMX_USB3_PHY_BASE + 0x01C1 * 4), 0x0000);
    WrReg16((IMX_USB3_PHY_BASE + 0x01C5 * 4), 0x0007);
    WrReg16((IMX_USB3_PHY_BASE + 0x01C6 * 4), 0x0027);
    WrReg16((IMX_USB3_PHY_BASE + 0x01C7 * 4), 0x0008);
    WrReg16((IMX_USB3_PHY_BASE + 0x01C4 * 4), 0x0022);/* Uncommented bit*/
    WrReg16((IMX_USB3_PHY_BASE + 0x0061 * 4), 0x000A);
    WrReg16((IMX_USB3_PHY_BASE + 0x40EA * 4), 0x0139);
    WrReg16((IMX_USB3_PHY_BASE + 0x4001 * 4), 0xBEFC);
    WrReg16((IMX_USB3_PHY_BASE + 0x4100 * 4), 0x7799);
    WrReg16((IMX_USB3_PHY_BASE + 0x4101 * 4), 0x7798);
    WrReg16((IMX_USB3_PHY_BASE + 0x4102 * 4), 0x5098);
    WrReg16((IMX_USB3_PHY_BASE + 0x4103 * 4), 0x5098);
    WrReg16((IMX_USB3_PHY_BASE + 0x4106 * 4), 0x2090);
    WrReg16((IMX_USB3_PHY_BASE + 0x4107 * 4), 0x2090);
    WrReg16((IMX_USB3_PHY_BASE + 0x8000 * 4), 0xA6FD);
    WrReg16((IMX_USB3_PHY_BASE + 0x8001 * 4), 0xA6FD);
    WrReg16((IMX_USB3_PHY_BASE + 0x8002 * 4), 0xA410);
    WrReg16((IMX_USB3_PHY_BASE + 0x8003 * 4), 0x2410);
    WrReg16((IMX_USB3_PHY_BASE + 0x8006 * 4), 0x23FF);
    WrReg16((IMX_USB3_PHY_BASE + 0x8007 * 4), 0x2010);
    WrReg16((IMX_USB3_PHY_BASE + 0x4058 * 4), 0x0020);
    WrReg16((IMX_USB3_PHY_BASE + 0x41E7 * 4), 0x00FF);
    WrReg16((IMX_USB3_PHY_BASE + 0x80E3 * 4), 0x0002);
    WrReg16((IMX_USB3_PHY_BASE + 0x8090 * 4), 0x0013);
    WrReg16((IMX_USB3_PHY_BASE + 0x8058 * 4), 0x0000);
    WrReg16((IMX_USB3_PHY_BASE + 0x81DC * 4), 0x1004);
    WrReg16((IMX_USB3_PHY_BASE + 0x81DF * 4), 0x4041);
    WrReg16((IMX_USB3_PHY_BASE + 0x81F5 * 4), 0x0480);
    WrReg16((IMX_USB3_PHY_BASE + 0x81D3 * 4), 0x8006);
    WrReg16((IMX_USB3_PHY_BASE + 0x81C7 * 4), 0x003F);
    WrReg16((IMX_USB3_PHY_BASE + 0x81C2 * 4), 0x543F);
    WrReg16((IMX_USB3_PHY_BASE + 0x81C1 * 4), 0x543F);
    WrReg16((IMX_USB3_PHY_BASE + 0x81C9 * 4), 0x0000);
    WrReg16((IMX_USB3_PHY_BASE + 0x81F8 * 4), 0x8000);
    WrReg16((IMX_USB3_PHY_BASE + 0x81F9 * 4), 0x0003);
    WrReg16((IMX_USB3_PHY_BASE + 0x81DD * 4), 0x2408);
    WrReg16((IMX_USB3_PHY_BASE + 0x81D5 * 4), 0x05CA);
    WrReg16((IMX_USB3_PHY_BASE + 0x81E1 * 4), 0x0258);
    WrReg16((IMX_USB3_PHY_BASE + 0x81BF * 4), 0x1FFF);
    WrReg16((IMX_USB3_PHY_BASE + 0x4002 * 4), 0x02C6);
    WrReg16((IMX_USB3_PHY_BASE + 0x4004 * 4), 0x0002);
    WrReg16((IMX_USB3_PHY_BASE + 0x4003 * 4), 0x02C6);
    WrReg16((IMX_USB3_PHY_BASE + 0x4005 * 4), 0x0010);
    WrReg16((IMX_USB3_PHY_BASE + 0x4006 * 4), 0x0010);
    WrReg16((IMX_USB3_PHY_BASE + 0x4007 * 4), 0x0010);
    WrReg16((IMX_USB3_PHY_BASE + 0x4008 * 4), 0x0010);
    WrReg16((IMX_USB3_PHY_BASE + 0x4009 * 4), 0x0010);
    WrReg16((IMX_USB3_PHY_BASE + 0x400A * 4), 0x0002);
    WrReg16((IMX_USB3_PHY_BASE + 0x400B * 4), 0x0002);
    WrReg16((IMX_USB3_PHY_BASE + 0x400C * 4), 0x0002);
    WrReg16((IMX_USB3_PHY_BASE + 0x400D * 4), 0x0002);
    WrReg16((IMX_USB3_PHY_BASE + 0x400E * 4), 0x0002);
    WrReg16((IMX_USB3_PHY_BASE + 0x400F * 4), 0x0002);
    WrReg16((IMX_USB3_PHY_BASE + 0x4122 * 4), 0x0FFF);
    WrReg16((IMX_USB3_PHY_BASE + 0x4123 * 4), 0x01E0);
    WrReg16((IMX_USB3_PHY_BASE + 0x40F2 * 4), 0x0090);
    WrReg16((IMX_USB3_PHY_BASE + 0x4102 * 4), 0x509B);
    WrReg16((IMX_USB3_PHY_BASE + 0x41F5 * 4), 0x0003);
}

/**
 * This function configures pin muxing, enables VBUS and calls controller initialization method for OTG1 controller.
 */
VOID USB2Init (VOID)
{
    sc_err_t sc_status;

    do {
        /* Power up USB_OTG1 */
        sc_status = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_USB_0, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "USB2 Init: sc_pm_set_resource_power_mode(SC_R_USB_0, SC_PM_PW_MODE_ON)) falied, sc_status = %d\n", sc_status));
            break;
        }
        sc_status = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_USB_0_PHY, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "USB2 Init: sc_pm_set_resource_power_mode(SC_R_USB_0_PHY, SC_PM_PW_MODE_ON)) falied, sc_status = %d\n", sc_status));
            break;
        }
        /* Enable USB_OTG2 peripheral clock */
        sc_status = sc_pm_clock_enable(SC_IPC_HDL, SC_R_USB_2, SC_PM_CLK_MISC, true, false);
        if (sc_status != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "USB2 Init: sc_pm_clock_enable(SC_R_USB_2, SC_PM_CLK_MISC, true, false) falied, sc_status = %d\n", sc_status));
            break;
        }
        /* Enable USB_OTG2 peripheral clock */
        sc_status = sc_pm_clock_enable(SC_IPC_HDL, SC_R_USB_2, SC_PM_CLK_MST_BUS , true, false);
        if (sc_status != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "USB2 Init: sc_pm_clock_enable(SC_R_USB_2, SC_PM_CLK_MST_BUS , true, false) falied, sc_status = %d\n", sc_status));
            break;
        }
        /* Enable USB_OTG2 peripheral clock */
        sc_status = sc_pm_clock_enable(SC_IPC_HDL, SC_R_USB_2, SC_PM_CLK_PER , true, false);
        if (sc_status != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "USB2 Init: sc_pm_clock_enable(SC_PM_CLK_PER , true, false) falied, sc_status = %d\n", sc_status));
            break;
        }
        WrReg32(CONNECTIVITY__LPCG_USB2_BASE_PTR, 0x22220000);
        /* USB2.0 OTG1 INIT NON-CORE REGISTERS */
        /* Set the USB 5V control switch power polarity high-active */
        WrReg32(IMX_USB2OTG1_BASE + IMX_USBNC_OTG_CTRL1, (RdReg32(IMX_USB2OTG1_BASE + IMX_USBNC_OTG_CTRL1) | IMX_USBNC_OTG_CTRL1_PWR_POL_MASK));
        /* Set the USB 5V control switch over current polarity low-active*/
        WrReg32(IMX_USB2OTG1_BASE + IMX_USBNC_OTG_CTRL1, (RdReg32(IMX_USB2OTG1_BASE + IMX_USBNC_OTG_CTRL1) | IMX_USBNC_OTG_CTRL1_OVER_CUR_POL_MASK));
    #if 0
        /* disable external charge detect to improve signal quality */
        WrReg32(IMX_USB2PHY1_BASE + IMX_USB_ANALOG_CHRG_DETECT, (IMX_USB_ANALOG_CHRG_DETECT_EN_B_MASK | IMX_USB_ANALOG_CHRG_DETECT_CHK_CHRG_B_MASK));
    #endif
        /* USB2.0 OTG1 INIT PLL Clock */
        init_usb2_pll(IMX_USB2PHY1_BASE);
        /* USB2.0 OTG1 INIT CORE REGISTERS */
        /* Stop OTG controller core */
        WrReg32(IMX_USB2OTG1_BASE + IMX_USB_USBCMD, RdReg32(IMX_USB2OTG1_BASE + IMX_USB_USBCMD) & ~IMX_USB_USBCMD_RS_MASK);
        while (RdReg32(IMX_USB2OTG1_BASE + IMX_USB_USBCMD) & IMX_USB_USBCMD_RS_MASK);
        /* Reset OTG controller core */
        WrReg32(IMX_USB2OTG1_BASE + IMX_USB_USBCMD, RdReg32(IMX_USB2OTG1_BASE) | IMX_USB_USBCMD_RST_MASK);
        while (RdReg32(IMX_USB2OTG1_BASE + IMX_USB_USBCMD) & IMX_USB_USBCMD_RST_MASK);
        /* USB2.0 OTG1 INIT PHY */
        init_usb2_phy(IMX_USB2PHY1_BASE);
    } while(0);
}

/**
 * This function configures controller core and calls controller initialization method for OTG2 controller.
 */
VOID USB3Init (VOID)
{
    uint32_t reg_value = 0;
    uint32_t cnt = 0;
    sc_err_t sc_status;

    do {
        sc_status = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_USB_2, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "USB3 Init: sc_pm_set_resource_power_mode(SC_R_USB_2, SC_PM_PW_MODE_ON)) falied, sc_status = %d\n", sc_status));
            break;
        }
        sc_status = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_USB_2_PHY, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "USB3 Init: sc_pm_set_resource_power_mode(SC_R_USB_2, SC_R_USB_2_PHY)) falied, sc_status = %d\n", sc_status));
            break;
        }
        /* Enable USB_OTG2 peripheral clock */
        sc_status = sc_pm_clock_enable(SC_IPC_HDL, SC_R_USB_2, SC_PM_CLK_MISC, true, false);
        if (sc_status != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "USB3 Init: sc_pm_clock_enable(SC_R_USB_2, SC_PM_CLK_MISC, true, false) falied, sc_status = %d\n", sc_status));
            break;
        }
        /* Enable USB_OTG2 peripheral clock */
        sc_status = sc_pm_clock_enable(SC_IPC_HDL, SC_R_USB_2, SC_PM_CLK_MST_BUS , true, false);
        if (sc_status != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "USB3 Init: sc_pm_clock_enable(SC_R_USB_2, SC_PM_CLK_MST_BUS , true, false) falied, sc_status = %d\n", sc_status));
            break;
        }
        /* Enable USB_OTG2 peripheral clock */
        sc_status = sc_pm_clock_enable(SC_IPC_HDL, SC_R_USB_2, SC_PM_CLK_PER , true, false);
        if (sc_status != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "USB3 Init: sc_pm_clock_enable(SC_PM_CLK_PER , true, false) falied, sc_status = %d\n", sc_status));
            break;
        }
        cnt = 10000;
        while (((RdReg32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_SSPHY_STATUS_OFFSET) & IMX_USB3_NON_CORE_SSPHY_STATUS_ALLCLKS_VLD_MASK) == 0) && (cnt > 0)) {
            MicroSecondDelay(10);
            cnt--;
        }
        if (cnt == 0) {
            DEBUG ((DEBUG_ERROR, "USB3 Init: Timeout waiting on All clocks ready Ready\n"));
            break;
        }
        /* Reset USB core */
        imx_usb3_reset_core(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET);
        /* Set XHCI mode and disable overcurrent */
        reg_value = RdReg32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET);    /* Read */
        reg_value &= (uint32_t)~IMX_USB3_NON_CORE_CORE_CTRL11_MODE_STRAP_MASK;              /* Clear mode bits */
        reg_value |= (uint32_t)(IMX_USB3_NON_CORE_CORE_CTRL11_MODE_STRAP_MASK & 0b010);     /* Set XHCI mode */
        reg_value |= (uint32_t)IMX_USB3_NON_CORE_CORE_CTRL11_OVERCURRENT_DISABLE_MASK;      /* Set OverCurrent disabled */
        WrReg32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET, reg_value);    /* Write */
        /* Clear PHY APB reset */
        reg_value = RdReg32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET);    /* Read */
        reg_value &= (uint32_t)~IMX_USB3_NON_CORE_CORE_CTRL11_PHYAPB_SW_RESET_MASK;         /* PHY APB SW Reset disable */
        WrReg32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET, reg_value);    /* Write */
        /* USB3.0 OTG2 INIT PHY */
        init_usb3_phy(IMX_USB3_PHY_BASE);
        /* USB3.0 OTG2 INIT NON-CORE INTERRUPT REGISTERS */
        reg_value = RdReg32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_INT_OFFSET);            /* Read */
        reg_value |= 0x0000001;                                                             /* Enable XHCI interrupt */
        WrReg32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_INT_OFFSET, reg_value);            /* Write */
        /* USB3.0 OTG2 - CLEAR ALL RESETS */
        reg_value = RdReg32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET);    /* Read */
        reg_value &= ~(IMX_USB3_NON_CORE_CORE_CTRL11_PWR_SW_RESET_MASK |
                    IMX_USB3_NON_CORE_CORE_CTRL11_APB_SW_RESET_MASK |
                    IMX_USB3_NON_CORE_CORE_CTRL11_AXI_SW_RESET_MASK |
                    IMX_USB3_NON_CORE_CORE_CTRL11_RW_SW_RESET_MASK  |
                    IMX_USB3_NON_CORE_CORE_CTRL11_PHY_SW_RESET_MASK |
                    IMX_USB3_NON_CORE_CORE_CTRL11_PHYAPB_SW_RESET_MASK);
        WrReg32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET, reg_value);    /* Write */
        /* Wait for XHCI_POWER_ON_READY */
        cnt = 10000;
        while (((RdReg32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_STATUS_OFFSET) & IMX_USB3_NON_CORE_CORE_STATUS_XHCI_POWER_ON_READY_MASK) == 0) && (cnt > 0)) {
            MicroSecondDelay(100);
            cnt--;
        }
        if (cnt == 0) {
            DEBUG ((DEBUG_ERROR, "USB3 Init: Timeout waiting on XHCI_POWER_ON_READY\n"));
            break;
        }
        /* Wait for CNR */
        cnt = 10000;
        while (((RdReg32(IMX_USB3_CORE_BASE + IMX_USB3_USBSS_USBSTS_OFFSET) & IMX_USB3_USBSS_USBSTS_CNR_MASK) == 1) && (cnt > 0)) {
            MicroSecondDelay(100);
            DEBUG ((DEBUG_ERROR, "USB3 Init: Wait for CNR %d\n", cnt));
            cnt--;
        }
        if (cnt == 0) {
            DEBUG ((DEBUG_ERROR, "USB3 Init: Timeout waiting on Controller Not Ready\n"));
            break;
        }
    } while (0);
}

/**
  Initialize ENETs modules on the SOC and perform required pin-muxing.
**/
VOID EnetInit(VOID)
{
    sc_err_t err;
    sc_pm_clock_rate_t rate;
    sc_rsrc_t instance[2] = {SC_R_ENET_0, SC_R_ENET_1};
    int i;
    uint32_t  fuse_MAC0 = 0U;
    uint32_t  fuse_MAC1 = 0U;
    uint32_t  fuse_MAC2 = 0U;
    uint32_t  fuse_MAC3 = 0U;
    GLOBAL_PAGE_MAC_ADDR *GlobalPageMacAddress;

    for (i = 0; i < sizeof(instance) / sizeof(instance[0]); i++) {
        /* Power up ENET */
        err = sc_pm_set_resource_power_mode(SC_IPC_HDL, instance[i], SC_PM_PW_MODE_ON);
        if (err != SC_ERR_NONE) {
            sc_pm_err_resource_power_mode(err, instance[i], SC_PM_PW_MODE_ON);
            continue;
        }

        /* Disable ENET clock roots */
        err = sc_pm_clock_enable(SC_IPC_HDL, instance[i], SC_PM_CLK_MISC0, false, false);   /* 0 */
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, instance[i], SC_PM_CLK_MISC0, false, false);
            continue;
        }
        err = sc_pm_clock_enable(SC_IPC_HDL, instance[i], SC_PM_CLK_PER, false, false);    /* 2 */
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, instance[i], SC_PM_CLK_PER, false, false);
            continue;
        }
        err = sc_pm_clock_enable(SC_IPC_HDL, instance[i], SC_PM_CLK_BYPASS, false, false); /* 4 */
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, instance[i], SC_PM_CLK_BYPASS, false, false);
            continue;
        }

        /* Set ENET clock root to 250 MHz */
        rate = 250000000U;
        err = sc_pm_set_clock_rate(SC_IPC_HDL, instance[i], SC_PM_CLK_PER, &rate);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_rate(err, instance[i], SC_PM_CLK_PER, rate);
            continue;
        }

        /* Set 25M Time clock root to 25 MHz */
        rate = 25000000U;
        err = sc_pm_set_clock_rate(SC_IPC_HDL, instance[i], SC_PM_CLK_BYPASS, &rate);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_rate(err, instance[i], SC_PM_CLK_BYPASS, rate);
            continue;
        }

        /* Set clock divider to reach 125MHz clock. 250MHz / 2 = 125MHz */
        err = sc_misc_set_control(SC_IPC_HDL, instance[i], SC_C_CLKDIV, 1U);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_rate(err, instance[i], SC_C_CLKDIV, rate);
            continue;
        }

        /* Enable ENET clock root */
        err = sc_pm_clock_enable(SC_IPC_HDL, instance[i], SC_PM_CLK_MISC0, true, true);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, instance[i], SC_PM_CLK_MISC0, true, true);
            continue;
        }
        err = sc_pm_clock_enable(SC_IPC_HDL, instance[i], SC_PM_CLK_PER, true, true);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, instance[i], SC_PM_CLK_PER, true, true);
            continue;
        }
        err = sc_pm_clock_enable(SC_IPC_HDL, instance[i], SC_PM_CLK_BYPASS, true, true);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, instance[i], SC_PM_CLK_BYPASS, true, true);
            continue;
        }
    }

    imx_pad_t enet0_rgmii_pads[] = {
        {SC_P_ENET0_RGMII_RX_CTL, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_RXD0,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_RXD1,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_RXD2,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_RXD3,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_RXC,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_TX_CTL, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ENET0_RGMII_TXD0,   IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ENET0_RGMII_TXD1,   IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ENET0_RGMII_TXD2,   IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ENET0_RGMII_TXD3,   IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ENET0_RGMII_TXC,    IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ENET0_MDC,          IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_MDIO,         IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
    };

    err = set_28fdsoi_pad(SC_IPC_HDL, &enet0_rgmii_pads[0], sizeof(enet0_rgmii_pads) / sizeof(imx_pad_t));
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "ENET0 pads init failed\n"));
    }

    imx_pad_t enet1_rgmii_pads[] = {
        {SC_P_SPDIF0_TX,     IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_SPDIF0_RX,     IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ESAI0_TX3_RX2, IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ESAI0_TX2_RX3, IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ESAI0_TX1,     IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ESAI0_TX0,     IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ESAI0_SCKR,    IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ESAI0_TX4_RX1, IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ESAI0_TX5_RX0, IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ESAI0_FST,     IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ESAI0_SCKT,    IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_ESAI0_FSR,     IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
    };

    err = set_28fdsoi_pad(SC_IPC_HDL, &enet1_rgmii_pads[0], sizeof(enet1_rgmii_pads) / sizeof(imx_pad_t));
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "ENET1 pads init failed\n"));
    }

    /* Read OTP fuses using SCFW API */
    err = sc_misc_otp_fuse_read(SC_IPC_HDL, IMX_SC_OCOTP_MAC0, &fuse_MAC0);
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "Can't load MAC0 otp fuse. SC status: %s\n", sc_status2str(err)));
    }

    err = sc_misc_otp_fuse_read(SC_IPC_HDL, IMX_SC_OCOTP_MAC1, &fuse_MAC1);
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "Can't load MAC1 otp fuse. SC status: %s\n", sc_status2str(err)));
    }

    err = sc_misc_otp_fuse_read(SC_IPC_HDL, IMX_SC_OCOTP_MAC2, &fuse_MAC2);
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "Can't load MAC2 otp fuse. SC status: %s\n", sc_status2str(err)));
    }
    err = sc_misc_otp_fuse_read(SC_IPC_HDL, IMX_SC_OCOTP_MAC3, &fuse_MAC3);
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "Can't load MAC3 otp fuse. SC status: %s\n", sc_status2str(err)));
    }

    DEBUG ((EFI_D_VERBOSE, "Fuse MAC0: 0x%x\n", fuse_MAC0));
    DEBUG ((EFI_D_VERBOSE, "Fuse MAC1: 0x%x\n", fuse_MAC1));
    DEBUG ((EFI_D_VERBOSE, "Fuse MAC2: 0x%x\n", fuse_MAC2));
    DEBUG ((EFI_D_VERBOSE, "Fuse MAC3: 0x%x\n", fuse_MAC3));

  /* Get pointer to MAC address region within global page */
  GlobalPageMacAddress = (GLOBAL_PAGE_MAC_ADDR *)(FixedPcdGet32 (PcdGlobalDataBaseAddress) + GLOBAL_PAGE_MAC_ADDR_OFFSET);

  /* Clear global page */
    memset(GlobalPageMacAddress, 0x00, 0x1000);

  if (fuse_MAC0 != 0U) {
      GlobalPageMacAddress->Mac0Valid = 1U;
      GlobalPageMacAddress->Mac0Id = 1U;
      GlobalPageMacAddress->Mac0Address[0] = (fuse_MAC0 & 0xFF);
      GlobalPageMacAddress->Mac0Address[1] = ((fuse_MAC0 >> 8U) & 0xFF);
      GlobalPageMacAddress->Mac0Address[2] = ((fuse_MAC0 >> 16U) & 0xFF);
      GlobalPageMacAddress->Mac0Address[3] = ((fuse_MAC0 >> 24U) & 0xFF);
      GlobalPageMacAddress->Mac0Address[4] = (fuse_MAC1 & 0xFF);
      GlobalPageMacAddress->Mac0Address[5] = ((fuse_MAC1 >> 8U) & 0xFF);
      DEBUG ((EFI_D_VERBOSE, "MAC0 succesfully stored into global page fuses\n"));
  } else {
      GlobalPageMacAddress->Mac0Valid = 0U;
      GlobalPageMacAddress->Mac0Id = 1U;
      DEBUG ((EFI_D_VERBOSE, "MAC0 not burned in fuses\n"));
  }

  if (fuse_MAC2 != 0U) {
      GlobalPageMacAddress->Mac1Valid = 1U;
      GlobalPageMacAddress->Mac1Id = 1U;
      GlobalPageMacAddress->Mac0Address[0] = (fuse_MAC2 & 0xFF);
      GlobalPageMacAddress->Mac0Address[1] = ((fuse_MAC2 >> 8U) & 0xFF);
      GlobalPageMacAddress->Mac0Address[2] = ((fuse_MAC2 >> 16U) & 0xFF);
      GlobalPageMacAddress->Mac0Address[3] = ((fuse_MAC2 >> 24U) & 0xFF);
      GlobalPageMacAddress->Mac0Address[4] = (fuse_MAC3 & 0xFF);
      GlobalPageMacAddress->Mac0Address[5] = ((fuse_MAC3 >> 8U) & 0xFF);
      DEBUG ((EFI_D_VERBOSE, "MAC1 succesfully stored into global page fuses\n"));
  } else {
      GlobalPageMacAddress->Mac1Valid = 0U;
      GlobalPageMacAddress->Mac1Id = 1U;
      DEBUG ((EFI_D_VERBOSE, "MAC1 not burned in fuses\n"));
  }
}

/**
  Initialize PWM block and perform required pin-muxing.
**/
VOID PwmInit()
{
    sc_err_t err = SC_ERR_NONE;
    sc_pm_clock_rate_t rate = 0U; 

    /* ******************************** PWM 0 ******************************** */
    /* Power PWM0 */
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_PWM_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_PWM_0, SC_PM_PW_MODE_ON);
    }    
    /* Disable PWM0 peripheral clock */
    if (err == SC_ERR_NONE) {
        err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_PWM_0, SC_PM_CLK_PER, false, false);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, SC_R_PWM_0, SC_PM_CLK_PER, false, false);
        }
    }
    /* Set PWM0 clock root to 25 MHz */
    if (err == SC_ERR_NONE) {
        rate = 25000000;
        err = sc_pm_set_clock_rate(SC_IPC_HDL, SC_R_PWM_0, SC_PM_CLK_PER, &rate);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_rate(err, SC_R_PWM_0, SC_PM_CLK_PER, rate);
        }
    }
    /* Enable PWM0 peripheral clock */
    if (err == SC_ERR_NONE) {
        err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_PWM_0, SC_PM_CLK_PER, true, true);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, SC_R_PWM_0, SC_PM_CLK_PER, true, true);
        }
    }
    /* ******************************** PWM 1 ******************************** */
    /* Power PWM1 */
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_PWM_1, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_PWM_1, SC_PM_PW_MODE_ON);
    }    
    /* Disable PWM1 peripheral clock */
    if (err == SC_ERR_NONE) {
        err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_PWM_1, SC_PM_CLK_PER, false, false);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, SC_R_PWM_1, SC_PM_CLK_PER, false, false);
        }
    }
    /* Set PWM1 clock root to 25 MHz */
    if (err == SC_ERR_NONE) {
        rate = 25000000;
        err = sc_pm_set_clock_rate(SC_IPC_HDL, SC_R_PWM_1, SC_PM_CLK_PER, &rate);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_rate(err, SC_R_PWM_1, SC_PM_CLK_PER, rate);
        }
    }
    /* Enable PWM1 peripheral clock */
    if (err == SC_ERR_NONE) {
        err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_PWM_1, SC_PM_CLK_PER, true, true);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, SC_R_PWM_1, SC_PM_CLK_PER, true, true);
        }
    }
    /* ******************************** PWM 2 ******************************** */
    /* Power PWM2 */
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_PWM_2, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_PWM_2, SC_PM_PW_MODE_ON);
    }    
    /* Disable PWM2 peripheral clock */
    if (err == SC_ERR_NONE) {
        err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_PWM_2, SC_PM_CLK_PER, false, false);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, SC_R_PWM_2, SC_PM_CLK_PER, false, false);
        }
    }
    /* Set PWM2 clock root to 25 MHz */
    if (err == SC_ERR_NONE) {
        rate = 25000000;
        err = sc_pm_set_clock_rate(SC_IPC_HDL, SC_R_PWM_2, SC_PM_CLK_PER, &rate);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_rate(err, SC_R_PWM_2, SC_PM_CLK_PER, rate);
        }
    }
    /* Enable PWM2 peripheral clock */
    if (err == SC_ERR_NONE) {
        err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_PWM_2, SC_PM_CLK_PER, true, true);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, SC_R_PWM_2, SC_PM_CLK_PER, true, true);
        }
    }
    /* ******************************** PWM 3 ******************************** */
    /* Power PWM3 */
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_PWM_3, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_PWM_3, SC_PM_PW_MODE_ON);
    }    
    /* Disable PWM3 peripheral clock */
    if (err == SC_ERR_NONE) {
        err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_PWM_3, SC_PM_CLK_PER, false, false);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, SC_R_PWM_3, SC_PM_CLK_PER, false, false);
        }
    }
    /* Set PWM3 clock root to 25 MHz */
    if (err == SC_ERR_NONE) {
        rate = 25000000;
        err = sc_pm_set_clock_rate(SC_IPC_HDL, SC_R_PWM_3, SC_PM_CLK_PER, &rate);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_rate(err, SC_R_PWM_3, SC_PM_CLK_PER, rate);
        }
    }
    /* Enable PWM3 peripheral clock */
    if (err == SC_ERR_NONE) {
        err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_PWM_3, SC_PM_CLK_PER, true, true);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, SC_R_PWM_3, SC_PM_CLK_PER, true, true);
        }
    }

    /* Configure PWM pads */ 
    #if (defined(PWM_OUT_ENABLE) && PWM_OUT_ENABLE) 
    /* The PWM output signals are connected to the M.2 connector and thus conflict with the WIFI module on the MX8QXP_MEK CPU board. */
    imx_pad_t i2c_pads[] = {
        /*   SCFW PAD         ALT           CONFIG               lower power        Drive strength           pull                                     signal name                           */
        {SC_P_UART1_TX,    IMX_PAD_ALT_1, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},    /* PWM0 OUT (LSIO_PWM0_OUT) */
        {SC_P_UART1_RX,    IMX_PAD_ALT_1, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},    /* PWM1 OUT (LSIO_PWM1_OUT) */
        {SC_P_UART1_RTS_B, IMX_PAD_ALT_1, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},    /* PWM2 OUT (LSIO_PWM2_OUT) */
        {SC_P_UART1_CTS_B, IMX_PAD_ALT_1, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},    /* PWM3 OUT (LSIO_PWM3_OUT) */
    };
    err = set_28fdsoi_pad(SC_IPC_HDL, &i2c_pads[0], sizeof(i2c_pads) / sizeof(imx_pad_t));
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "PWM pads init failed\n"));
    }
    #endif
    
    ASSERT (err == SC_ERR_NONE);
}

/**
  Initialize USDHC blocks and perform required pin-muxing.
**/
VOID UsdhcInit()
{
    sc_err_t err = SC_ERR_NONE;
  sc_pm_clock_rate_t rate = 0U;

      /* Power up uSDHC0 */
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_SDHC_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_SDHC_0, SC_PM_PW_MODE_ON);
    }
  }
    /* Disable uSDHC0 peripheral clock */
  if (err == SC_ERR_NONE) {
    err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_SDHC_0, SC_PM_CLK_PER, false, false);
    if (err != SC_ERR_NONE) {
        sc_pm_err_clock_enable(err, SC_R_SDHC_0, SC_PM_CLK_PER, false, false);
    }
  }
    /* Set uSDHC0 clock root to 400 MHz */
  if (err == SC_ERR_NONE) {
    rate = 400000000;
    err = sc_pm_set_clock_rate(SC_IPC_HDL, SC_R_SDHC_0, SC_PM_CLK_PER, &rate);
    if (err != SC_ERR_NONE) {
        sc_pm_err_clock_rate(err, SC_R_SDHC_0, SC_PM_CLK_PER, rate);
    }
  }
    /* Enable uSDHC0 peripheral clock */
  if (err == SC_ERR_NONE) {
    err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_SDHC_0, SC_PM_CLK_PER, true, true);
    if (err != SC_ERR_NONE) {
        sc_pm_err_clock_enable(err, SC_R_SDHC_0, SC_PM_CLK_PER, true, true);
    }
  }
    /* ********************************* uSDHC1 ********************************* */
    /* Power up uSDHC1 */
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_SDHC_1, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_SDHC_1, SC_PM_PW_MODE_ON);
    }
  }
    /* Disable uSDHC1 peripheral clock */
  if (err == SC_ERR_NONE) {
    err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_SDHC_1, SC_PM_CLK_PER, false, false);
    if (err != SC_ERR_NONE) {
        sc_pm_err_clock_enable(err, SC_R_SDHC_1, SC_PM_CLK_PER, false, false);
    }
  }
    /* Set uSDHC1 clock root to 198 MHz */
  if (err == SC_ERR_NONE) {
    rate = 198000000;
    err = sc_pm_set_clock_rate(SC_IPC_HDL, SC_R_SDHC_1, SC_PM_CLK_PER, &rate);
    if (err != SC_ERR_NONE) {
        sc_pm_err_clock_rate(err, SC_R_SDHC_1, SC_PM_CLK_PER, rate);
    }
  }
    /* Enable uSDHC0 peripheral clock */
  if (err == SC_ERR_NONE) {
    err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_SDHC_1, SC_PM_CLK_PER, true, false);
    if (err != SC_ERR_NONE) {
        sc_pm_err_clock_enable(err, SC_R_SDHC_1, SC_PM_CLK_PER, true, false);
    }
  }
  if (err != SC_ERR_NONE) {
    err = sc_ipc_reset(SC_IPC_HDL);
  }
  ASSERT (err == SC_ERR_NONE);
}

/**
  Initialize DIPLAY_MIX and perform required pin-muxing.
**/
VOID DisplayInit()
{
    sc_err_t err = SC_ERR_NONE;
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_DC_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_DC_0, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_DC_0_PLL_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_DC_0_PLL_0, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_DC_0_PLL_1, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_DC_0_PLL_0, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_DC_0_VIDEO0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_DC_0_VIDEO0, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_DC_0_VIDEO1, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_DC_0_VIDEO1, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_MIPI_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_MIPI_0, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_LVDS_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_LVDS_0, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_MIPI_1, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_MIPI_1, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_LVDS_1, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_LVDS_1, SC_PM_PW_MODE_ON);
    }
  }
  if (err != SC_ERR_NONE) {
    err = sc_ipc_reset(SC_IPC_HDL);
  }
  ASSERT (err == SC_ERR_NONE);

  /* enable LPI2C0 irq in MIPI-DSI / LVDS #0 Local Interrupt Steer
     Note: this setting is discarded later in UEFI display driver */
  IRQSTEER_CHn_MASK_REG(IRQSTEER_MIPI_LVDS0_BASE_PTR, 0) = 1U << 8;
  /* enable LPI2C0 irq in MIPI-DSI / LVDS #1 Local Interrupt Steer
     Note: this setting is discarded later in UEFI display driver */
  IRQSTEER_CHn_MASK_REG(IRQSTEER_MIPI_LVDS1_BASE_PTR, 0) = 1U << 8;
}


/**
  Initialize DIPLAY_MIX and perform required pin-muxing.
**/
VOID GpuInit()
{
  sc_err_t err = SC_ERR_NONE;
  sc_pm_clock_rate_t gpu_freq = SC_700MHZ;
  sc_pm_clock_rate_t shader_freq = SC_850MHZ;


  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_GPU_0_PID0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
      sc_pm_err_resource_power_mode(err, SC_R_GPU_0_PID0, SC_PM_PW_MODE_ON);
    }
  }

  if (err == SC_ERR_NONE) {
    err = sc_pm_set_clock_rate(SC_IPC_HDL, SC_R_GPU_0_PID0, SC_PM_CLK_PER, &gpu_freq);
    if (err != SC_ERR_NONE) {
      sc_pm_err_clock_rate(err, SC_R_GPU_0_PID0, SC_PM_CLK_PER, gpu_freq);
    }
  }

  if (err == SC_ERR_NONE) {
    err = sc_pm_set_clock_rate(SC_IPC_HDL, SC_R_GPU_0_PID0, SC_PM_CLK_MISC, &shader_freq);
    if (err != SC_ERR_NONE) {
      sc_pm_err_clock_rate(err, SC_R_GPU_0_PID0, SC_PM_CLK_MISC, shader_freq);
    }

  }

  if (err == SC_ERR_NONE) {
    err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_GPU_0_PID0, SC_PM_CLK_MISC, true, false);
    if (err != SC_ERR_NONE) {
      sc_pm_err_clock_enable(err, SC_R_GPU_0_PID0, SC_PM_CLK_MISC, true, false);
    }
  }

  if (err == SC_ERR_NONE) {
    err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_GPU_0_PID0, SC_PM_CLK_PER, true, false);
    if (err != SC_ERR_NONE) {
      sc_pm_err_clock_enable(err, SC_R_GPU_0_PID0, SC_PM_CLK_PER, true, false);
    }
  }

  if (err == SC_ERR_NONE) {
    err = sc_misc_set_control(SC_IPC_HDL, SC_R_GPU_0_PID0, SC_C_ID, 0);
    if (err != SC_ERR_NONE) {
      DEBUG ((DEBUG_ERROR, "Failed to set GPU ID. SC status: %s\n", sc_status2str(err)));
    }
  }

  if (err != SC_ERR_NONE) {
    err = sc_ipc_reset(SC_IPC_HDL);
  }
  ASSERT (err == SC_ERR_NONE);
}

/**
  Initialize LPUART.
**/
VOID LpuartInit()
{
  typedef struct {
    sc_rsrc_t resource;
    sc_pm_clock_rate_t frequency; // in Hz; 0 means set to default
  } clk_cfg_t;

  const clk_cfg_t instances[] = {
   {SC_R_UART_0, 80000000},
#if FixedPcdGetBool(PcdLpuart1Enable)
   {SC_R_UART_1, 80000000},
#endif
   {SC_R_UART_2, 80000000},
  };
  const uint8_t instances_num = (uint8_t)(sizeof(instances) / sizeof(instances[0]));

  imx_pad_t pads[] = {
    // LPUART 0 - Debug, FTDI port 0 UART; Default power-on or U-Boot
    {SC_P_UART0_RX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_PU},
    {SC_P_UART0_TX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},

    // LPUART 1 - M.2 MINI CARD - WiFi UART
#if FixedPcdGetBool(PcdLpuart1Enable)
    {SC_P_UART1_RX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_PU},
    {SC_P_UART1_TX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},
    {SC_P_UART1_RTS_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},
    {SC_P_UART1_CTS_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},
#endif
    // LPUART 2 - BB RS232 J37; Default power-on or U-Boot
    {SC_P_UART2_RX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_PU},
    {SC_P_UART2_TX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},

    // LPUART 3 - BB Audio UART
    // SC_P_SCU_GPIO0_01
    // SC_P_SCU_GPIO0_00
  };
  
  uint32_t i;
  sc_err_t err = SC_ERR_NONE;
  sc_rsrc_t resource;
  sc_pm_clock_rate_t frequency;
  sc_pm_clock_rate_t get_rate = 0;
  
  DEBUG ((DEBUG_ERROR, "LpuartInit\n"));

  err = set_28fdsoi_pad(SC_IPC_HDL, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
  if (err != SC_ERR_NONE) {
    DEBUG ((DEBUG_ERROR, "LpuartInit: set_28fdsoi_pad() failed\n"));
  }
  
  /* Enable LPUART clocks with required frequency */
  for (i = 0; i < instances_num; i++) {
      frequency = instances[i].frequency;
      resource = instances[i].resource;
      DEBUG ((EFI_D_VERBOSE, "LpuartInit: init_periph_power_clock_freq %d of %d\n", i, instances_num));
      if (init_periph_power_clock_freq(SC_IPC_HDL, resource, SC_PM_CLK_PER, &frequency, &get_rate) != SC_ERR_NONE) {
          return;
      }
      err = sc_pm_get_clock_rate(SC_IPC_HDL, resource, SC_PM_CLK_PER, &get_rate);
      if ((frequency != 0) && (get_rate != frequency)) {
          DEBUG ((DEBUG_ERROR, "LpuartInit: Wrong frequency: %dHz (requested: %dHz) for resource:%d clk:%d\n", 
                 (int)get_rate, (int)frequency, (int)resource, (int)SC_PM_CLK_PER));
      }
  }
  ASSERT (err == SC_ERR_NONE);
}

/**
  Initialize VPU and perform required pin-muxing.
**/
VOID VpuInit()
{
  sc_err_t err = SC_ERR_NONE;
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_VPU_MU_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_VPU_MU_0, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_VPU_MU_1, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_VPU_MU_1, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_VPU_DEC_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_VPU_DEC_0, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_VPU_ENC_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_VPU_ENC_0, SC_PM_PW_MODE_ON);
    }
  }
  if (err == SC_ERR_NONE) {
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_VPU, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_VPU, SC_PM_PW_MODE_ON);
    }
  }
  if (err != SC_ERR_NONE) {
    err = sc_ipc_reset(SC_IPC_HDL);
  }
  ASSERT (err == SC_ERR_NONE);
}

/**
  Initialize LPSPI blocks and perform required pin-muxing.
**/
VOID SpiInit()
{
    uint8_t i;
    sc_err_t err;
    sc_rsrc_t resource;
    sc_pm_clock_rate_t req_frequency, frequency, *frequencyPtr;
    sc_pm_clock_rate_t get_rate = 0;
    typedef struct {
        sc_rsrc_t resource;
        sc_pm_clock_rate_t frequency;       // in Hz; 0 means set to default
    } spi_clk_cfg_t;
    spi_clk_cfg_t instances[] = {
                                  {SC_R_SPI_0, 160000000U},
                                  //{SC_R_SPI_1, 160000000U},
                                  //{SC_R_SPI_2, 160000000U},
                                  {SC_R_SPI_3, 160000000U},
                                };
    uint8_t instances_num = (uint8_t)(sizeof(instances) / sizeof(spi_clk_cfg_t));

    /* Enable SPI clocks with required frequency */
    for (i = 0; i < instances_num; i++) {
        req_frequency = instances[i].frequency;
        frequency = req_frequency;
        resource = instances[i].resource;
        frequencyPtr = (req_frequency == 0)? NULL : &frequency;
        if (init_periph_power_clock_freq(SC_IPC_HDL, resource, SC_PM_CLK_PER, frequencyPtr, &get_rate) != SC_ERR_NONE) {
            DEBUG ((DEBUG_ERROR, "SPIInit() init_periph_power_clock_freq() returned an error - frequency: %dHz (requested: %dHz) for resource:%d clk:%d\n",
                   (int)get_rate, (int)frequency, (int)resource, (int)SC_PM_CLK_PER));
            continue;
        }
        if ((req_frequency != 0) && (get_rate != req_frequency)) {
            DEBUG ((DEBUG_ERROR, "SPIInit() Frequency check failed - frequency: %dHz (requested: %dHz) resource:%d clk:%d\n",
                   (int)get_rate, (int)frequency, (int)resource, (int)SC_PM_CLK_PER));
        }
    }

    /* Configure SPI related pads */
    imx_pad_t spi_pads[] = {
        /* LPSPI0 pad settings */
        /* Use SC_PAD_CONFIG_OUT_IN on SCK pad if Input data is sampled on delayed SCK edge (CFGR1[SAMPLE] bit in LPSPI) */
        {SC_P_SPI0_SCK, IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PD},
        {SC_P_SPI0_SDI, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PD},
        {SC_P_SPI0_SDO, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PD},

        /* LPSPI3 pad settings */
        /* Use SC_PAD_CONFIG_OUT_IN on SCK pad if Input data is sampled on delayed SCK edge (CFGR1[SAMPLE] bit in LPSPI) */
        {SC_P_SPI3_SCK, IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PD},
        {SC_P_SPI3_SDI, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PD},
        {SC_P_SPI3_SDO, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PD},
    };
    err = set_28fdsoi_pad(SC_IPC_HDL, &spi_pads[0], sizeof(spi_pads) / sizeof(imx_pad_t));
    if (err != SC_ERR_NONE) {
        DEBUG ((DEBUG_ERROR, "SPI pads init failed\n"));
    }

}

/**
  Initialize FlexCan
**/
VOID FlexCanInit()
{
    sc_err_t err = SC_ERR_NONE;
    sc_pm_clock_rate_t rate = 0U;

    /* ******************************** CAN 0 ******************************** */
    /* Power CAN0 */
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_CAN_0, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_CAN_0, SC_PM_PW_MODE_ON);
    }

    /* ******************************** CAN 1 ******************************** */
    /* Power CAN1 */
    err = sc_pm_set_resource_power_mode(SC_IPC_HDL, SC_R_CAN_1, SC_PM_PW_MODE_ON);
    if (err != SC_ERR_NONE) {
        sc_pm_err_resource_power_mode(err, SC_R_CAN_1, SC_PM_PW_MODE_ON);
    }

    /* Disable CAN01 peripheral clock */
    if (err == SC_ERR_NONE) {
        err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_CAN_0, SC_PM_CLK_PER, false, false);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, SC_R_CAN_0, SC_PM_CLK_PER, false, false);
        }
    }
    /* Set CAN01 clock root to 40 MHz */
    if (err == SC_ERR_NONE) {
        rate = 40000000;
        err = sc_pm_set_clock_rate(SC_IPC_HDL, SC_R_CAN_0, SC_PM_CLK_PER, &rate);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_rate(err, SC_R_CAN_0, SC_PM_CLK_PER, rate);
        }
    }
    /* Enable CAN01 peripheral clock */
    if (err == SC_ERR_NONE) {
        err = sc_pm_clock_enable(SC_IPC_HDL, SC_R_CAN_0, SC_PM_CLK_PER, true, true);
        if (err != SC_ERR_NONE) {
            sc_pm_err_clock_enable(err, SC_R_CAN_0, SC_PM_CLK_PER, true, true);
        }
    }

    imx_pad_t pads[] = {
        /*   SCFW PAD         ALT           CONFIG               lower power        Drive strength           pull                                     signal name                           */
        {SC_P_FLEXCAN0_RX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},    /* FLEXCAN0_RX (ADMA.FLEXCAN0.RX) */
        {SC_P_FLEXCAN0_TX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},    /* FLEXCAN0_TX (ADMA.FLEXCAN0.TX) */
        {SC_P_FLEXCAN1_RX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},    /* FLEXCAN1_RX (ADMA.FLEXCAN1.RX) */
        {SC_P_FLEXCAN1_TX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_1MA, SC_PAD_28FDSOI_PS_NONE},    /* FLEXCAN1_TX (ADMA.FLEXCAN1.TX) */
    };
    err = set_28fdsoi_pad(SC_IPC_HDL, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
    if (err != SC_ERR_NONE) {
        DEBUG((DEBUG_ERROR, "FLEXCAN pads init failed\n"));
    }

    ASSERT(err == SC_ERR_NONE);
}

/**
  Initialize I2C switch and IO expanders.
**/
VOID I2CSwitchIOExpandersInit()
{
  EFI_STATUS Status = RETURN_SUCCESS;
  EFI_STATUS ProbeStatus = RETURN_SUCCESS;
  sc_err_t err = SC_ERR_NONE;
  UINT8 Data[2];
  UINT8 Mask[2];
  UINT8 Zero[2] = {0};

  // Release I2C switch and IO expanders reset
  imx_pad_t pad = {SC_P_SPI2_SDO, IMX_PAD_ALT_4, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}; /*  LSIO.GPIO1.IO01 for IOEXP_RST */

  err = set_28fdsoi_pad(SC_IPC_HDL, &pad, 1);
  if (err != SC_ERR_NONE) {
    DEBUG ((DEBUG_ERROR, "I2C switch and IO expanders reset pin configuration failed\n"));
  }
  LSIO__GPIO1_DR |= (0x01 << PORT_EXP_RST_PIN);                    // Set the pad to the low level
  LSIO__GPIO1_GDIR |= (0x01 << PORT_EXP_RST_PIN);                  // Set output direction

  // Configure I2C switch
  do {
    Data[0] = I2C_SWITCH_CONTROL_B3 | I2C_SWITCH_CONTROL_B2 | I2C_SWITCH_CONTROL_B1 | I2C_SWITCH_CONTROL_B0;
    Status = I2CSwitch_Configure(&Pca9646I2cSwitchConfig, Data[0]);
  } while (FALSE);
  CHECK_I2C_TRANSACTION_STATUS(Status, "I2C Switch configuration", Next);

  // Configure PCA9557 IO expander A
  do {
    Data[0] = EXP_A_CPU_ENET_PHY_RST_B | EXP_A_USB_TYPEC_EN | EXP_A_MIPI_DSI0_EN | EXP_A_SENS_RST_B;
    Status = IOExp_PCA9557_Configure(&Pca9557IOExpAI2CConfig, Data, Zero, Zero, Data);
  } while (FALSE);
  CHECK_I2C_TRANSACTION_STATUS(Status, "IO Expander A confguration", Next);

  // Configure PCA9557 IO expander B
  do {
    Data[0] = EXP_B_MIPI_DSI1_EN | EXP_B_BB_GPIO_EXP1 | EXP_B_BB_GPIO_EXP2 | EXP_B_BB_GPIO_EXP3 | EXP_B_BB_USB_OTG1_PWR_ON;
    Status = IOExp_PCA9557_Configure(&Pca9557IOExpBI2CConfig, Data, Zero, Zero, Data);
  } while (FALSE);
  CHECK_I2C_TRANSACTION_STATUS(Status, "IO Expander B confguration", Next);

Next:
  // Configure PCA6416 IO expander CAN on BaseBoard
  ProbeStatus = ProbeI2CDevice(&Pca6416IOExpBBI2CConfig);
  if (ProbeStatus != RETURN_DEVICE_ERROR) {
    if (ProbeStatus == RETURN_SUCCESS) {
      Mask[0] = BB_CAN_EXP_CAN01_STB_B | BB_CAN_EXP_CAN01_EN | BB_CAN_EXP_CAN1_WAKE_3V3_B |
                   BB_CAN_EXP_CAN0_WAKE_3V3_B;
      Mask[1] = 0;
      Data[0] = Mask[0] & ~(BB_CAN_EXP_CAN1_WAKE_3V3_B | BB_CAN_EXP_CAN0_WAKE_3V3_B);
      Data[1] = 0;
      Status = IOExp_PCA6416_Configure(&Pca6416IOExpBBI2CConfig, Mask, Zero, Zero, Data);
      CHECK_I2C_TRANSACTION_STATUS(Status, "IO Expander CAN (BaseBoard) confguration", End);
    }
    else {
      CHECK_I2C_TRANSACTION_STATUS(Status, "Probing I2C IO Expander CAN on BaseBoard", End);
    }
  }
  // Toggle CAN WAKE and STB signals
  if (ProbeStatus == RETURN_SUCCESS) {
    MicroSecondDelay(500);
    do {
      /* CAN0 and 1 WAKE to 1 */
      Mask[0] = BB_CAN_EXP_CAN1_WAKE_3V3_B | BB_CAN_EXP_CAN0_WAKE_3V3_B;
      Mask[1] = 0;
      Status = IOExp_PCA6416_SetVal(&Pca6416IOExpBBI2CConfig, Mask, Mask);
      CHECK_I2C_TRANSACTION_STATUS(Status, "IO Expander CAN (BaseBoard) - CAN0 and 1 WAKE signals to 1", End);
      MicroSecondDelay(500);
      /* CAN0 and 1 WAKE to 0 */
      Status = IOExp_PCA6416_SetVal(&Pca6416IOExpBBI2CConfig, Mask, Zero);
      CHECK_I2C_TRANSACTION_STATUS(Status, "IO Expander CAN (BaseBoard) - CAN0 and 1 WAKE signals to 0", End);
      MicroSecondDelay(500);
      /* CAN01 STB to 0 */
      Mask[0] = BB_CAN_EXP_CAN01_STB_B;
      Status = IOExp_PCA6416_SetVal(&Pca6416IOExpBBI2CConfig, Mask, Zero);
      CHECK_I2C_TRANSACTION_STATUS(Status, "IO Expander CAN (BaseBoard) - CAN01 STB signal to 0", End);
      MicroSecondDelay(500);
      /* CAN01 STB to 1 */
      Status = IOExp_PCA6416_SetVal(&Pca6416IOExpBBI2CConfig, Mask, Mask);
      CHECK_I2C_TRANSACTION_STATUS(Status, "IO Expander CAN (BaseBoard) - CAN01 STB signal to 1", End);
    } while (FALSE);

  }

End:
  return;
}

/**
  Initialize MPL3115A2 Precision Pressure Sensor
**/
VOID PsInit()
{
    EFI_STATUS Status;
    UINT8 Register, Data;

    /* Output port settings */
    Register = MPL3115A2_CTRL_REG3;
    Data = MPL3115A2_CTRL_REG3_PPOD1_OD | MPL3115A2_CTRL_REG3_PPOD2_OD;

    /* PPS INT pins to Open Drain */
    Status = iMXI2cWriteE(&Mpl3115a2PsI2CConfig, Register, 1, &Data, 1);
    CHECK_I2C_TRANSACTION_STATUS(Status, "MPL3115A2 Precision Pressure Sensor - writing MPL3115A2_CTRL_REG3", End);

End:
    return;
}

void dump_scfw_clock(void);
void dump_scfw_power(void);

/**
  Initialize controllers that must setup at the early stage
**/
RETURN_STATUS ArmPlatformInitialize(IN UINTN MpId)
{
  sc_ipc_t dummy = NULL;

  if (!ArmPlatformIsPrimaryCore (MpId)) {
    return RETURN_SUCCESS;
  }
  // Initialize debug serial port
  SerialPortInitialize ();
  SerialPortWrite ((UINT8 *)SERIAL_DEBUG_PORT_INIT_MSG, (UINTN)sizeof(SERIAL_DEBUG_PORT_INIT_MSG));
  if ((sc_ipc_open(&dummy, SC_IPC_HDL) != SC_ERR_NONE) || (dummy == NULL)) {
    DEBUG ((DEBUG_ERROR, "ArmPlatformInitialize: There is no correct address in the sc_ipc_open\n"));
    return RETURN_UNSUPPORTED;
  }
  UngateClocks();
  GpioInit();
  I2cInit();
  FlexCanInit(); // First set up FlexCanInit then I2CSwitchIOExpandersInit
  I2CSwitchIOExpandersInit(); 
  PsInit();
  LpuartInit();
#if FixedPcdGet32(PcdPcie1Enable)
  PcieInit();
#endif
  USB2Init();
  USB3Init();
  EnetInit();
  AudioInit();
  PwmInit();
  UsdhcInit();
  DisplayInit();
  GpuInit();
  VpuInit();
  SpiInit();
  CameraInit();
  dump_scfw_clock();
  dump_scfw_power();

  return RETURN_SUCCESS;
}

/**
  Return the current Boot Mode. This function returns the boot reason on the platform
**/
EFI_BOOT_MODE ArmPlatformGetBootMode (VOID)
{
  return BOOT_WITH_FULL_CONFIGURATION;
}

static void prn_scfw_clk(const char* name, sc_rsrc_t resource, sc_pm_clk_t clk) {
    sc_pm_clock_rate_t rate = 0;
    sc_pm_power_mode_t mode = 0;
    sc_err_t rerr = SC_ERR_NONE;
    sc_err_t perr = SC_ERR_NONE;
    perr = sc_pm_get_resource_power_mode(SC_IPC_HDL, resource, &mode);
    rerr = sc_pm_get_clock_rate(SC_IPC_HDL, resource, clk, &rate);
    DEBUG ((DEBUG_INFO, "perr:%d rerr:%d resource:%d clk:%d mode:%d rate:%d %a\r\n",
          (int)perr, (int)rerr, (int)resource, (int)clk, (int)mode, (int)rate, name));
}

static void prn_scfw_pwr(const char* name, sc_rsrc_t resource) {
    sc_pm_power_mode_t mode = 0;
    sc_err_t err = SC_ERR_NONE;
    err = sc_pm_get_resource_power_mode(SC_IPC_HDL, resource, &mode);
    DEBUG ((DEBUG_WARN, "perr:%d resource:%d mode:%d %a\r\n",
          (int)err, (int)resource, (int)mode, name));
}

void dump_scfw_power(void) {
  prn_scfw_pwr("VPU DEC MU 0", SC_R_VPU_MU_0);
  prn_scfw_pwr("VPU ENC MU 1", SC_R_VPU_MU_1);
  prn_scfw_pwr("VPU DEC 0", SC_R_VPU_DEC_0);
  prn_scfw_pwr("VPU ENC 0", SC_R_VPU_ENC_0);
  prn_scfw_pwr("VPU", SC_R_VPU);
  prn_scfw_pwr("PWM 0", SC_R_PWM_0);
  prn_scfw_pwr("PWM 1", SC_R_PWM_1);
  prn_scfw_pwr("PWM 2", SC_R_PWM_2);
  prn_scfw_pwr("PWM 3", SC_R_PWM_3);
  prn_scfw_pwr("CAN 0", SC_R_CAN_0);
  prn_scfw_pwr("CAN 1", SC_R_CAN_1);
}

void dump_scfw_clock(void) {
  prn_scfw_clk("AUDIO PLL 0", SC_R_AUDIO_PLL_0, SC_PM_CLK_PLL);
  prn_scfw_clk("AUDIO PLL DIV CLK 0", SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC0);
  prn_scfw_clk("AUDIO REC CLK 0", SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC1);
  prn_scfw_clk("AUDIO PLL 1", SC_R_AUDIO_PLL_1, SC_PM_CLK_PLL);
  prn_scfw_clk("AUDIO PLL DIV CLK 1", SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC0);
  prn_scfw_clk("AUDIO REC CLK 1", SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC1);
  prn_scfw_clk("CAN0 CLK", SC_R_CAN_0, SC_PM_CLK_PER);
  prn_scfw_clk("CAN1 CLK", SC_R_CAN_1, SC_PM_CLK_PER);
  prn_scfw_clk("MIPI 0 escape", SC_R_CSI_0, SC_PM_CLK_MISC);
  prn_scfw_clk("MIPI 0 core", SC_R_CSI_0, SC_PM_CLK_PER);
  prn_scfw_clk("DC 0 PLL 0", SC_R_DC_0_PLL_0, SC_PM_CLK_PLL);
  prn_scfw_clk("DC 0 PLL 1", SC_R_DC_0_PLL_1, SC_PM_CLK_PLL);
  prn_scfw_clk("DC 0 Display 0", SC_R_DC_0, SC_PM_CLK_MISC0);
  prn_scfw_clk("DC 0 Display 1", SC_R_DC_0, SC_PM_CLK_MISC1);
  prn_scfw_clk("DC 0 Bypass 0", SC_R_DC_0_VIDEO0, SC_PM_CLK_MISC);
  prn_scfw_clk("DC 0 Bypass 1", SC_R_DC_0_VIDEO1, SC_PM_CLK_MISC);
  prn_scfw_clk("DRC 0 root", SC_R_DRC_0, SC_PM_CLK_SLV_BUS);
  prn_scfw_clk("LVDS 0 bypass", SC_R_LVDS_0, SC_PM_CLK_BYPASS);
  prn_scfw_clk("LVDS 0 pixel", SC_R_LVDS_0, SC_PM_CLK_PER);
  prn_scfw_clk("LVDS 0 phy", SC_R_LVDS_0, SC_PM_CLK_PHY);
  prn_scfw_clk("LVDS 1 bypass", SC_R_LVDS_1, SC_PM_CLK_BYPASS);
  prn_scfw_clk("LVDS 1 pixel", SC_R_LVDS_1, SC_PM_CLK_PER);
  prn_scfw_clk("LVDS 1 phy", SC_R_LVDS_1, SC_PM_CLK_PHY);
  prn_scfw_clk("MIPI 0 I2C 0 baud", SC_R_MIPI_0_I2C_0, SC_PM_CLK_PER);
  prn_scfw_clk("MIPI 0 I2C 1 baud", SC_R_MIPI_0_I2C_1, SC_PM_CLK_PER);
  prn_scfw_clk("MIPI 0 PWM baud", SC_R_MIPI_0_PWM_0, SC_PM_CLK_PER);
  prn_scfw_clk("MIPI 0 bypass", SC_R_MIPI_0, SC_PM_CLK_BYPASS);
  prn_scfw_clk("MIPI 0 DSI tx escape", SC_R_MIPI_0, SC_PM_CLK_MST_BUS);
  prn_scfw_clk("MIPI 0 DPI (pixel)", SC_R_MIPI_0, SC_PM_CLK_PER);
  prn_scfw_clk("MIPI 0 DPHY PLL ref", SC_R_MIPI_0, SC_PM_CLK_PHY);
  prn_scfw_clk("MIPI 0 DSI rx escape", SC_R_MIPI_0, SC_PM_CLK_SLV_BUS);
  prn_scfw_clk("MIPI 1 I2C 0 baud", SC_R_MIPI_1_I2C_0, SC_PM_CLK_PER);
  prn_scfw_clk("MIPI 1 I2C 1 baud", SC_R_MIPI_1_I2C_1, SC_PM_CLK_PER);
  prn_scfw_clk("MIPI 1 PWM baud", SC_R_MIPI_1_PWM_0, SC_PM_CLK_PER);
  prn_scfw_clk("MIPI 1 bypass", SC_R_MIPI_1, SC_PM_CLK_BYPASS);
  prn_scfw_clk("MIPI 1 DSI tx escape", SC_R_MIPI_1, SC_PM_CLK_MST_BUS);
  prn_scfw_clk("MIPI 1 DPI (pixel)", SC_R_MIPI_1, SC_PM_CLK_PER);
  prn_scfw_clk("MIPI 1 DPHY PLL ref", SC_R_MIPI_1, SC_PM_CLK_PHY);
  prn_scfw_clk("MIPI 1 DSI rx escape", SC_R_MIPI_1, SC_PM_CLK_SLV_BUS);
  prn_scfw_clk("Pixel link 0 I2C baud", SC_R_PI_0_I2C_0, SC_PM_CLK_PER);
  prn_scfw_clk("Pixel link 0 PLL", SC_R_PI_0_PLL, SC_PM_CLK_PLL);
  prn_scfw_clk("Pixel link 0 PWM baud", SC_R_PI_0_PWM_0, SC_PM_CLK_PER);
  prn_scfw_clk("Pixel link 0 bypass", SC_R_PI_0, SC_PM_CLK_BYPASS);
  prn_scfw_clk("Pixel link 0 MCLK", SC_R_PI_0, SC_PM_CLK_MISC0);
  prn_scfw_clk("Pixel link 0 pixel", SC_R_PI_0, SC_PM_CLK_PER);
  prn_scfw_clk("SDHC0", SC_R_SDHC_0, SC_PM_CLK_PER);
  prn_scfw_clk("SDHC1", SC_R_SDHC_1, SC_PM_CLK_PER);
  prn_scfw_clk("PWM0", SC_R_PWM_0, SC_PM_CLK_PER);
  prn_scfw_clk("PWM1", SC_R_PWM_1, SC_PM_CLK_PER);
  prn_scfw_clk("PWM2", SC_R_PWM_2, SC_PM_CLK_PER);
  prn_scfw_clk("PWM3", SC_R_PWM_3, SC_PM_CLK_PER);
  prn_scfw_clk("SPI0", SC_R_SPI_0, SC_PM_CLK_PER);
  prn_scfw_clk("SPI1", SC_R_SPI_1, SC_PM_CLK_PER);
  prn_scfw_clk("SPI2", SC_R_SPI_2, SC_PM_CLK_PER);
  prn_scfw_clk("SPI3", SC_R_SPI_3, SC_PM_CLK_PER);
}
