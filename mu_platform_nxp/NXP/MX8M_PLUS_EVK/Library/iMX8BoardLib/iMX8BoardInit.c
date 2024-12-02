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

/** @file
* NXP iMX8MP EVK Board Init 
**/

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

#define IMX_SIP_CONFIG_GPC_PM_DOMAIN    0x3
#define IMX_PCIE1_PD                    1
#define IMX_USB1_PD                     2
#define IMX_USB2_PD                     3
#define IMX_GPU_BUS                     6
#define IMX_GPU2D                       7
#define IMX_GPU3D                       8
#define IMX_VPU_BU                      9
#define IMX_VPU_G1                      10
#define IMX_VPU_G2                      11
#define IMX_VPU_H1                      12
#define IMX_MEDIAMIX                    13
#define IMX_MIPI_PHY1                   15
#define IMX_HDMIMIX                     17
#define IMX_HDMI_PHY                    18

#define IMX_VPU_BLK_CTL_BASE            0x38330000

#define IMX_I2C3_BASE                   0x30A40000
#define IMX_PCA6416_I2C_BASE            IMX_I2C3_BASE

#define IMX_I2C1_BASE                   0x30A20000
#define IMX_PMIC_I2C_BASE               IMX_I2C1_BASE

/* PCA6416 registers */
#define PCA6416_INPUT_PORT_0_REG               0x00
#define PCA6416_INPUT_PORT_1_REG               0x01
#define PCA6416_OUTPUT_PORT_0_REG              0x02
#define PCA6416_OUTPUT_PORT_1_REG              0x03
#define PCA6416_POLARITY_INVERSION_PORT_0_REG  0x04
#define PCA6416_POLARITY_INVERSION_PORT_1_REG  0x05
#define PCA6416_CONFIGURATION_PORT_0_REG       0x06
#define PCA6416_CONFIGURATION_PORT_1_REG       0x07

/* IO Expander output pins */
#define EXP_P0_EXT_PWREN1                      0x01
#define EXP_P0_EXT_PWREN2                      0x02
#define EXP_P0_CAN1_I2C5_SEL                   0x04
#define EXP_P0_PDM_CAN2_SEL                    0x08
#define EXP_P0_FAN_EN                          0x10
#define EXP_P0_PWR_MEAS_IO1                    0x20
#define EXP_P0_PWR_MEAS_IO2                    0x40
#define EXP_P0_7                               0x80

/* IO Expander output pins */
#define EXP_P1_0                               0x01
#define EXP_P1_1                               0x02
#define EXP_P1_2                               0x04
#define EXP_P1_3                               0x08
#define EXP_P1_4                               0x10
#define EXP_P1_5                               0x20
#define EXP_P1_6                               0x40
#define EXP_P1_7                               0x80


/* USB MIX offset */
#define USBMIX_PHY_OFFSET                            0xF0040

/* DWC3 register and bit-fields definition */
#define DWC3_GHWPARAMS1                              0xC144

#define DWC3_GSNPSID                                 0xC120
#define DWC3_GCTL                                    0xC110
#define DWC3_GCTL_PWRDNSCALE_SHIFT                   19
#define DWC3_GCTL_PWRDNSCALE_MASK                    (0x1FFF << 19)
#define DWC3_GCTL_U2RSTECN_MASK                      (1 << 16)
#define DWC3_GCTL_PRTCAPDIR(n)                       ((n) << 12)
#define DWC3_GCTL_PRTCAP_HOST                        1
#define DWC3_GCTL_PRTCAP_DEVICE                      2
#define DWC3_GCTL_PRTCAP_OTG                         3
#define DWC3_GCTL_CORESOFTRESET_MASK                 (1 << 11)
#define DWC3_GCTL_SCALEDOWN(n)                       ((n) << 4)
#define DWC3_GCTL_SCALEDOWN_MASK                     DWC3_GCTL_SCALEDOWN(3)
#define DWC3_GCTL_DISSCRAMBLE_MASK                   (1 << 3)
#define DWC3_GCTL_DSBLCLKGTNG_MASK                   (1 << 0)

#define DWC3_GUSB2PHYCFG                             0xC200
#define DWC3_GUSB2PHYCFG_PHYSOFTRST_MASK             (1 << 31)
#define DWC3_GUSB2PHYCFG_U2_FREECLK_EXISTS_MASK      (1 << 30)
#define DWC3_GUSB2PHYCFG_ENBLSLPM_MASK               (1 << 8)
#define DWC3_GUSB2PHYCFG_SUSPHY_MASK                 (1 << 6)
#define DWC3_GUSB2PHYCFG_PHYIF_MASK                  (1 << 3)

#define DWC3_GUSB3PIPECTL                            0xC2C0
#define DWC3_GUSB3PIPECTL_PHYSOFTRST_MASK            (1 << 31)

#define DWC3_GFLADJ                                  0xC630
#define GFLADJ_30MHZ_REG_SEL                         (1 << 7)
#define GFLADJ_30MHZ(n)                              ((n) & 0x3f)
#define GFLADJ_30MHZ_DEFAULT                         0x20

/* USB PHYx registers and bit-fields definition */
#define USB_PHY_CTRL0                                0x0
#define USB_PHY_CTRL0_REF_SSP_EN_MASK                (1 << 2)

#define USB_PHY_CTRL1                                0x4
#define USB_PHY_CTRL1_RESET_MASK                     (1 << 0)
#define USB_PHY_CTRL1_COMMONONN_MASK                 (1 << 1)
#define USB_PHY_CTRL1_ATERESET_MASK                  (1 << 3)
#define USB_PHY_CTRL1_VDATSRCENB0_MASK               (1 << 19)
#define USB_PHY_CTRL1_VDATDETENB0_MASK               (1 << 20)

#define USB_PHY_CTRL2                                0x8
#define USB_PHY_CTRL2_TXENABLEN0_MASK                (1 << 8)

#define USB_PHY_CTRL6                                0x18

#define HSIO_GPR_REG_0                               (0x32F10000U)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT     (1)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN           (0x1U << HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT)

#define IMX_USB3_OTG1_BASE ((UINT64)USB1_BASE_PTR)
#define IMX_USB3_OTG2_BASE ((UINT64)USB2_BASE_PTR)


#define in32(_Addr)          (*(UINT32*)((void*)(UINT64)(_Addr)))
#define out32(_Addr,_Val)    (*(UINT32*)(void*)(UINT64)(_Addr)) = _Val

/**
  Initialize USB PHY
**/
void UsbPhyInit(UINT32 base)
{
    UINT32 reg;

    /* enable usb clock via hsio gpr */
    reg = in32(HSIO_GPR_REG_0);
    reg |= HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN;
    out32(HSIO_GPR_REG_0, reg);

    /* USB3.0 PHY signal fsel for 24M ref */
    reg = in32(base + USBMIX_PHY_OFFSET +  USB_PHY_CTRL0);
    reg = (reg & 0xFFFFF81F) | (0x2A << 5);
    out32(base + USBMIX_PHY_OFFSET +  USB_PHY_CTRL0, reg);

    reg = in32(base + USBMIX_PHY_OFFSET +  USB_PHY_CTRL6);
    reg &= ~(0x01);
    out32(base + USBMIX_PHY_OFFSET +  USB_PHY_CTRL6, reg);

    reg = in32(base + USBMIX_PHY_OFFSET + USB_PHY_CTRL1);
    reg &= ~(USB_PHY_CTRL1_VDATSRCENB0_MASK | USB_PHY_CTRL1_VDATDETENB0_MASK);
    reg |= USB_PHY_CTRL1_RESET_MASK | USB_PHY_CTRL1_ATERESET_MASK;
    out32(base + USBMIX_PHY_OFFSET + USB_PHY_CTRL1, reg);

    reg = in32(base + USBMIX_PHY_OFFSET + USB_PHY_CTRL0);
    reg |= USB_PHY_CTRL0_REF_SSP_EN_MASK;
    out32(base + USBMIX_PHY_OFFSET + USB_PHY_CTRL0, reg);

    reg = in32(base + USBMIX_PHY_OFFSET + USB_PHY_CTRL2);
    reg |= USB_PHY_CTRL2_TXENABLEN0_MASK;
    out32(base + USBMIX_PHY_OFFSET + USB_PHY_CTRL2, reg);

    reg = in32(base + USBMIX_PHY_OFFSET + USB_PHY_CTRL1);
    reg &= ~(USB_PHY_CTRL1_RESET_MASK | USB_PHY_CTRL1_ATERESET_MASK);
    out32(base + USBMIX_PHY_OFFSET + USB_PHY_CTRL1, reg);
}

/**
  Reset USB PHY
**/
void UsbPhyReset(UINT32 base)
{
    UINT32 reg;

    /* Before Resetting PHY, put Core in Reset */
    reg = in32(base + DWC3_GCTL);
    reg |= DWC3_GCTL_CORESOFTRESET_MASK;
    out32(base + DWC3_GCTL, reg);

    /* Assert USB3 PHY reset */
    reg = in32(base + DWC3_GUSB3PIPECTL);
    reg |= DWC3_GUSB3PIPECTL_PHYSOFTRST_MASK;
    out32(base + DWC3_GUSB3PIPECTL, reg);

    /* Assert USB2 PHY reset */
    reg = in32(base + DWC3_GUSB2PHYCFG);
    reg |= DWC3_GUSB2PHYCFG_PHYSOFTRST_MASK;
    out32(base + DWC3_GUSB2PHYCFG, reg);

    MicroSecondDelay(100 * 1000);

    /* Clear USB3 PHY reset */
    reg = in32(base + DWC3_GUSB3PIPECTL);
    reg &= ~DWC3_GUSB3PIPECTL_PHYSOFTRST_MASK;
    out32(base + DWC3_GUSB3PIPECTL, reg);

    /* Clear USB2 PHY reset */
    reg = in32(base + DWC3_GUSB2PHYCFG);
    reg &= ~DWC3_GUSB2PHYCFG_PHYSOFTRST_MASK;
    out32(base + DWC3_GUSB2PHYCFG, reg);

    MicroSecondDelay(100 * 1000);

    /* Disable PHY suspend (ERR011231) */
    out32(base + DWC3_GUSB2PHYCFG, (reg | DWC3_GUSB2PHYCFG_SUSPHY_MASK));

    /* After PHYs are stable we can take Core out of reset state */
    reg = in32(base + DWC3_GCTL);
    reg &= ~DWC3_GCTL_CORESOFTRESET_MASK;
    out32(base + DWC3_GCTL, reg);
}

/**
  Initialize USB core
**/
void UsbDwc3CoreInit(UINT32 base)
{
    UINT32 reg;

    UsbPhyReset(base);
    reg = in32(base + DWC3_GCTL);
    reg &= ~DWC3_GCTL_SCALEDOWN_MASK;
    reg &= ~DWC3_GCTL_DISSCRAMBLE_MASK;
    reg &= ~DWC3_GCTL_DSBLCLKGTNG_MASK;
    out32(base + DWC3_GCTL, reg);
}

/**
  Suspend XHCI clock
**/
void UsbXhciSuspendClock(UINT32 base)
{
    UINT32 reg;

    /* Set suspend_clk to be 32KHz */
    reg = in32(base + DWC3_GCTL);
    reg &= ~DWC3_GCTL_PWRDNSCALE_MASK;
    reg |= 2 << DWC3_GCTL_PWRDNSCALE_SHIFT;

    out32(base + DWC3_GCTL, reg);
}

/**
 * This function configures controller core and calls controller initialization method for OTG1, OTG2 controller.
 */
void UsbInit(void)
{
    UINT32 reg;

    UsbPhyInit(IMX_USB3_OTG1_BASE);
    UsbPhyInit(IMX_USB3_OTG2_BASE);

    UsbDwc3CoreInit(IMX_USB3_OTG1_BASE);
    UsbDwc3CoreInit(IMX_USB3_OTG2_BASE);

    UsbXhciSuspendClock(IMX_USB3_OTG1_BASE);
    UsbXhciSuspendClock(IMX_USB3_OTG2_BASE);

    /* Set DWC3 core to Host Mode for OTG1 */
    reg = in32(IMX_USB3_OTG1_BASE + DWC3_GCTL);
    reg &= ~DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG);
    reg |= DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_HOST);
    out32(IMX_USB3_OTG1_BASE + DWC3_GCTL, reg);

    /* Set DWC3 core to Host Mode for OTG2 */
    reg = in32(IMX_USB3_OTG2_BASE + DWC3_GCTL);
    reg &= ~DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG);
    reg |= DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_HOST);
    out32(IMX_USB3_OTG2_BASE + DWC3_GCTL, reg);

    /* Set GFLADJ_30MHZ as 20h as per XHCI spec default value */
    reg = in32(IMX_USB3_OTG1_BASE + DWC3_GFLADJ);
    reg |= GFLADJ_30MHZ_REG_SEL | GFLADJ_30MHZ(GFLADJ_30MHZ_DEFAULT);
    out32(IMX_USB3_OTG1_BASE + DWC3_GFLADJ, reg);

    /* Set GFLADJ_30MHZ as 20h as per XHCI spec default value */
    reg = in32(IMX_USB3_OTG2_BASE + DWC3_GFLADJ);
    reg |= GFLADJ_30MHZ_REG_SEL | GFLADJ_30MHZ(GFLADJ_30MHZ_DEFAULT);
    out32(IMX_USB3_OTG2_BASE + DWC3_GFLADJ, reg);
}

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

IMX_I2C_CONTEXT Pca6416I2cConfig =
{
  (uintptr_t)IMX_PCA6416_I2C_BASE,      /* Base address of the I2C used for communication with PCA6416 */
  0,                                    /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  24000000,                             /* 24Mhz I2C ReferenceFreq */
  400000,                               /* 400KHz required TargetFreq */
  0x20,                                 /* PCA6416 SlaveAddress */
  100000,                               /* TimeoutInUs */
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

VOID ArmPlatformGetPlatformPpiList (OUT UINTN *PpiListSize, OUT EFI_PEI_PPI_DESCRIPTOR **PpiList)
{
  *PpiListSize = sizeof(gPlatformPpiTable);
  *PpiList     = gPlatformPpiTable;
}

VOID UngateClocks ()
{
  // Enable SDMA
  CCM_CCGR_SDMA1 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
  // Enable UARTs
  CCM_CCGR_UART1 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
  CCM_CCGR_UART2 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
  CCM_CCGR_UART3 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
  CCM_CCGR_UART4 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
  // Enable SPIs
  CCM_CCGR_ECSPI1 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
  CCM_CCGR_ECSPI2 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
  CCM_CCGR_ECSPI3 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
}

#if FixedPcdGet32(PcdPcie1Enable)
/**
  Initialize PCI Express module on the SOC and perform required pin-muxing
**/
VOID PcieInit ()
{
  ARM_SMC_ARGS smc_args;

  // Enable HSIOMIX power domain, ID = 0x00
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, 0x00, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);
  // Enable PCIe power domain, ID = 0x01
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, 0x01, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  // Disable PCIE_CTRL clock root
  CCM_CCGR_PCIE = 0x00;
  // PCIE_PHY_CLK_ROOT  ClkSrc = SYSTEM_PLL2_DIV10 (100 MHz), PreDiv = 1, PostDiv = 1, Fout = 100 MHz
  CCM_TARGET_ROOT_PCIE_PHY  = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // PCIE_AUX_CLK_ROOT  ClkSrc = SYSTEM_PLL2_DIV20 ( 50 MHz), PreDiv = 1, PostDiv = 5, Fout =  10 MHz
  CCM_TARGET_ROOT_PCIE_AUX  = CCM_TARGET_ROOT_MUX(2) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(4) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Enable PCIE_CTRL clock root
  CCM_CCGR_PCIE = 0x03;  // Domain 0 clocks needed when in RUN and WAIT
  CCM_CCGR_HSIO = 0x03;  // Domain 0 clocks needed when in RUN and WAIT

  // Configure I2C4_SCL as GPIO to enable PCIe external ref. clock
  IOMUXC_SW_MUX_CTL_PAD_I2C4_SCL = IOMUXC_MUX_ALT5;
  GPIO5_DR &= ~(0x01 << 20);                    // Set the pad to the low level
  GPIO5_GDIR |= (0x01 << 20);                   // Set output direction
  // Configure I2C4_SDA as GPIO to control PCIe nWAKE PAD
  IOMUXC_SW_MUX_CTL_PAD_I2C4_SDA = IOMUXC_MUX_ALT5;
  GPIO5_GDIR &= ~(0x01 << 21);                  // Set input direction
  // Configure SD1_DATA5 as GPIO to control PCIe nPERST PAD
  IOMUXC_SW_MUX_CTL_PAD_SD1_DATA5 = IOMUXC_MUX_ALT5;
  GPIO2_DR &= ~(0x01 << 7);                     // Set the pad to the low level
  GPIO2_GDIR |= (0x01 << 7);                    // Set output direction
  // Configure SD1_DATA4 to control PCIe nDISABLE PAD
  IOMUXC_SW_MUX_CTL_PAD_SD1_DATA4 = IOMUXC_MUX_ALT5;
  GPIO2_DR |= (0x01 << 6);                      // Set the pad to the high level
  GPIO2_GDIR |= (0x01 << 6);                    // Set output direction
}
#endif

/**
  Initialize GPIO modules on the SOC and perform required pin-muxing
**/
VOID GpioInit ()
{
  CCM_CCGR_GPIO1 = 0x03;       // Enable GPIO1 clock root
  CCM_CCGR_GPIO2 = 0x03;       // Enable GPIO2 clock root
  CCM_CCGR_GPIO3 = 0x03;       // Enable GPIO3 clock root
  CCM_CCGR_GPIO4 = 0x03;       // Enable GPIO4 clock root
  CCM_CCGR_GPIO5 = 0x03;       // Enable GPIO5 clock root
}

/**
  Initalize the Audio system
**/
#define SAI_PAD_CFG_OUT (IOMUXC_PAD_PUE_ENABLE | IOMUXC_PAD_DSE_R0_DIV_3 | IOMUXC_PAD_SRE_FAST)
#define SAI_PAD_CFG_IN  (SAI_PAD_CFG_OUT | IOMUXC_PAD_HYS_ENABLED)
VOID AudioInit(VOID)
{
  ARM_SMC_ARGS smc_args;
  INT32 count;
  UINT32 preDivSelectSai3 = 8;
  UINT32 postDivSelectSai3 = 4;

  // Enable AUDIOMIX power domain, ID = 0x05
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, 0x05, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  // Mux the SAI3 pins to wm8960 codec
  IOMUXC_SW_MUX_CTL_PAD_SAI3_TXFS = IOMUXC_MUX_ALT0;
  IOMUXC_SW_MUX_CTL_PAD_SAI3_TXC  = IOMUXC_MUX_ALT0;
  IOMUXC_SW_MUX_CTL_PAD_SAI3_TXD  = IOMUXC_MUX_ALT0;
  IOMUXC_SW_MUX_CTL_PAD_SAI3_RXD  = IOMUXC_MUX_ALT0;
  IOMUXC_SW_MUX_CTL_PAD_SAI3_MCLK = IOMUXC_MUX_ALT0;

  IOMUXC_SW_PAD_CTL_PAD_SAI3_TXFS = SAI_PAD_CFG_OUT;
  IOMUXC_SW_PAD_CTL_PAD_SAI3_TXC  = SAI_PAD_CFG_OUT;
  IOMUXC_SW_PAD_CTL_PAD_SAI3_TXD  = SAI_PAD_CFG_OUT;
  IOMUXC_SW_PAD_CTL_PAD_SAI3_RXD  = SAI_PAD_CFG_IN;
  IOMUXC_SW_PAD_CTL_PAD_SAI3_MCLK = SAI_PAD_CFG_OUT;

  IOMUXC_AUDIOMIX_SAI3_RXDATA_SELECT_INPUT_0 = IOMUXC_MUX_ALT1;

  // Gate the AUDIO device
  CCM_CCGR_AUDIO = 0x00;

  CCM_PLL_CTRL35 = 0x3333;

  /* Bypass and reset the AUDIO PLL1 clock */
  CCM_ANALOG_AUDIO_PLL1_GEN_CTRL = CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_BYPASS_MASK |
                                   CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_CLKE_MASK;
  /* Configure Audio PLL1 */
#if 1
  /* 361267200Hz */
  CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0 = CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV(361) |
                                    CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV(3) |
                                    CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV(3);
  CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1 = CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM(17511);
#else
  /* 393216000Hz */
  CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0 = CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV(262) |
                                    CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV(2) |
                                    CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV(3);
  CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1 = CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM(9437);
#endif
  MicroSecondDelay (4);
  CCM_ANALOG_AUDIO_PLL1_GEN_CTRL |=  CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_RST_MASK;
  for (count = 0 ; count < 100; ++count) {
    if (CCM_ANALOG_AUDIO_PLL1_GEN_CTRL & CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_LOCK_MASK) {
      break;
    }
    MicroSecondDelay (10);
  }
  if (!(CCM_ANALOG_AUDIO_PLL1_GEN_CTRL & CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_LOCK_MASK)) {
    DebugPrint (0xFFFFFFFF, "Time out waiting for PLL to lock\n");
  }
  CCM_ANALOG_AUDIO_PLL1_GEN_CTRL &= ~(CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_BYPASS_MASK);


  // Configure SAI3_CLK_ROOT to Audio PLL1 clock source and divide it down to 11.2896 MHz
  CCM_TARGET_ROOT(77) = CCM_TARGET_ROOT_MUX(0x01) | CCM_TARGET_ROOT_POST_PODF(postDivSelectSai3 - 1) |
                            CCM_TARGET_ROOT_PRE_PODF(preDivSelectSai3 - 1);
  // Configure AUDIO_AHB_CLK_ROOT to System PLL1 clock source, divide by 2
  CCM_TARGET_ROOT(34) = CCM_TARGET_ROOT_MUX(0x02) | CCM_TARGET_ROOT_PRE_PODF(2);
  // Configure AUDIO_AXI_CLK_ROOT to System PLL1 clock source, divide by 2
  CCM_TARGET_ROOT(6) = CCM_TARGET_ROOT_MUX(0x02);

  CCM_TARGET_ROOT_SET(77) = CCM_TARGET_ROOT_ENABLE_MASK;  // Enable SAI3_CLK_ROOT
  CCM_TARGET_ROOT_SET(34) = CCM_TARGET_ROOT_ENABLE_MASK;  // Enable AUDIO_AHB_CLK_ROOT
  CCM_TARGET_ROOT_SET(6) = CCM_TARGET_ROOT_ENABLE_MASK;   // Enable AUDIO_AXI_CLK_ROOT
  CCM_CCGR_AUDIO = 0x03;  // Domain 0 clocks needed when in RUN and WAIT

  // For SAI3, select SAI3_CLK_ROOT to MCLK1 and SAI3.MCLK to MCLK2
  AUDIOMIX_SAI3_MCLK_SEL = AUDIOMIX_SAI3_MCLK_SEL_MCLK2_SEL(9);

  I2S3_MCR = I2S_MCR_MOE_MASK;  // Enable SAI3_MCLK output, the MCLK signal is the audio master clock

}

/**
  Initialize I2C modules on the SOC and perform required pin-muxing
**/
#define I2C_PAD_CTRL (IOMUXC_PAD_HYS_ENABLED | IOMUXC_PAD_PUE_ENABLE | IOMUXC_PAD_ODE_ENABLED | IOMUXC_PAD_SRE_SLOW | IOMUXC_PAD_DSE_R0_DIV_3)
VOID I2cInit()
{
  IOMUXC_SW_MUX_CTL_PAD_I2C1_SCL = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_I2C1_SDA = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_I2C2_SCL = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_I2C2_SDA = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_I2C3_SCL = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_I2C3_SDA = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_SPDIF_TX = IOMUXC_MUX_ALT2 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_SPDIF_RX = IOMUXC_MUX_ALT2 | IOMUXC_MUX_SION_ENABLED;

  IOMUXC_SW_PAD_CTL_PAD_I2C1_SCL = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_I2C1_SDA = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_I2C2_SCL = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_I2C2_SDA = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_I2C3_SCL = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_I2C3_SDA = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_SPDIF_TX = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_SPDIF_RX = I2C_PAD_CTRL;

  CCM_CCGR_I2C1 = 0x00;
  CCM_CCGR_I2C2 = 0x00;
  CCM_CCGR_I2C3 = 0x00;
  CCM_CCGR_I2C5 = 0x00;
  // I2C ClkSrc     = 24M_REF_CLK  (24 MHz), PreDiv = 1, PostDiv = 1, Fout = 24 MHz
  CCM_TARGET_ROOT_I2C1 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_TARGET_ROOT_I2C2 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_TARGET_ROOT_I2C3 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_TARGET_ROOT_I2C5 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;

  CCM_CCGR_I2C1 = 0x03;
  CCM_CCGR_I2C2 = 0x03;
  CCM_CCGR_I2C3 = 0x03;
  CCM_CCGR_I2C5 = 0x03;
}


/**
  Initialize ENETs modules on the SOC and perform required pin-muxing.
**/
#define ENET_MDC_PAD_CTRL     (IOMUXC_PAD_DSE_R0_DIV_3 | IOMUXC_PAD_SRE_SLOW)
#define ENET_MDIO_PAD_CTRL    (IOMUXC_PAD_DSE_R0_DIV_3 | IOMUXC_PAD_SRE_SLOW)
#define ENET_TX_PAD_CTRL      (IOMUXC_PAD_DSE_R0_DIV_7 | IOMUXC_PAD_SRE_MAX)
#define ENET_RX_PAD_CTRL      (IOMUXC_PAD_DSE_R0_DIV_1 | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_HYS_ENABLED)
VOID EnetInit(VOID)
{
  // ENET1/2 MDIO bus (both ENETs share one MDIO bus conected to the ENET1 controller)
  IOMUXC_SW_MUX_CTL_PAD_SAI1_RXD2   = IOMUXC_MUX_ALT4;    // ENET1_MDC               -> PAD_SAI1_RXD2
  IOMUXC_SW_MUX_CTL_PAD_SAI1_RXD3   = IOMUXC_MUX_ALT4;    // ENET1_MDIO              -> ENET1_MDIO_SELECT_INPUT
  IOMUXC_ENET1_MDIO_SELECT_INPUT    = IOMUXC_MUX_ALT1;    // ENET1_MDIO_SELECT_INPUT -> PAD_SAI1_RXD3
  IOMUXC_SW_PAD_CTL_PAD_SAI1_RXD2   = ENET_MDC_PAD_CTRL;  // ENET1_MDC  electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_RXD3   = ENET_MDIO_PAD_CTRL; // ENET1_MDIO electrical settings
  // ENET1 RGMMI pins routing
  IOMUXC_SW_MUX_CTL_PAD_SAI1_TXD3   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_TD3               -> PAD_SAI1_TXD3
  IOMUXC_SW_MUX_CTL_PAD_SAI1_TXD2   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_TD2               -> PAD_SAI1_TXD2
  IOMUXC_SW_MUX_CTL_PAD_SAI1_TXD1   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_TD1               -> PAD_SAI1_TXD1
  IOMUXC_SW_MUX_CTL_PAD_SAI1_TXD0   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_TD0               -> PAD_SAI1_TXD0
  IOMUXC_SW_MUX_CTL_PAD_SAI1_TXD4   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_TX_CTL            -> PAD_SAI1_TXD4
  IOMUXC_SW_MUX_CTL_PAD_SAI1_TXD5   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_TXC               -> PAD_SAI1_TXD5
  IOMUXC_SW_MUX_CTL_PAD_SAI1_RXD7   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_RD3               -> PAD_SAI1_RXD7
  IOMUXC_SW_MUX_CTL_PAD_SAI1_RXD6   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_RD2               -> PAD_SAI1_RXD6
  IOMUXC_SW_MUX_CTL_PAD_SAI1_RXD5   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_RD1               -> ENET1_RGMII_RD1_SELECT_INPUT
  IOMUXC_ENET1_RXDATA_1_SELECT_INPUT= IOMUXC_MUX_ALT1;    // ENET1_RGMII_RD1_SELECT_INPUT  -> PAD_SAI1_RXD5
  IOMUXC_SW_MUX_CTL_PAD_SAI1_RXD4   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_RD0               -> ENET1_RGMII_RD0_SELECT_INPUT
  IOMUXC_ENET1_RXDATA_0_SELECT_INPUT= IOMUXC_MUX_ALT1;    // ENET1_RGMII_RD0_SELECT_INPUT  -> PAD_SAI1_RXD4
  IOMUXC_SW_MUX_CTL_PAD_SAI1_TXFS   = IOMUXC_MUX_ALT4;    // ENET1_RGMII_RX_CTL            -> ENET1_RXEN_SELECT_INPUTS
  IOMUXC_ENET1_RXEN_SELECT_INPUT    = IOMUXC_MUX_ALT1;    // ENET1_RXEN_SELECT_INPUTS      -> PAD_SAI1_TXFS
  IOMUXC_SW_MUX_CTL_PAD_SAI1_TXC    = IOMUXC_MUX_ALT4;    // ENET1_RGMII_RXC               -> PAD_SAI1_TXC
  // ENET1 RGMMI pins electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_RXD7   = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RD3    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_RXD6   = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RD2    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_RXD5   = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RD1    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_RXD4   = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RD0    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_TXFS   = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RX_CTL electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_TXC    = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RXC    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_TXD3   = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD3    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_TXD2   = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD2    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_TXD1   = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD1    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_TXD0   = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD0    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_TXD4   = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TX_CTL electrical settings
  IOMUXC_SW_PAD_CTL_PAD_SAI1_TXD5   = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TXC    electrical settings
  // Configure ENET_nRST signal
  IOMUXC_SW_MUX_CTL_PAD_SAI1_RXD0   = IOMUXC_MUX_ALT5;   // ENET_nRST -> PAD_SAI1_RXD0[ALT5]=GPIO4_IO02
  IOMUXC_SW_PAD_CTL_PAD_SAI1_RXD0   = IOMUXC_PAD_DSE_R0_DIV_1 | IOMUXC_PAD_SRE_SLOW;
  // TODO add config command to the ENET ACPI
#if 0
  GPIO4_DR &= ~(1 << 02);                                // Set ENET_nRST = 0
  GPIO4_GDIR |= (1 << 02);                               // Set direction to output
  MicroSecondDelay(500);
  GPIO4_DR |= (1 << 02);
#endif
  // Disable ENET1 clock(s)
  CCM_CCGR_ENET1    = 0x00;
  CCM_CCGR_SIM_ENET = 0x00;
  // ENET_AXI_CLK_ROOT ClkSrc = SYSTEM_PLL1_DIV3  (266 MHz), PreDiv = 1, PostDiv = 1, Fout = 266 MHz
  CCM_TARGET_ROOT_ENET_AXI = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // ENET_REF ClkSrc: SYSTEM_PLL2_DIV8 (125 MHz), fout = 125 MHz
  CCM_TARGET_ROOT_ENET_REF = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Enable ENET1 clock(s)
  CCM_CCGR_ENET1    = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
  CCM_CCGR_SIM_ENET = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
}

/**
  Initialize ENET_QOS clocks & pads.
**/
VOID EnetQosInit()
{
  uint32_t gpr1;

  /* Tx pads */
  IOMUXC_SW_MUX_CTL_PAD_ENET_TD0    = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD0    = IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);
  IOMUXC_SW_MUX_CTL_PAD_ENET_TD1    = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD1    = IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);
  IOMUXC_SW_MUX_CTL_PAD_ENET_TD2    = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD2    = IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);
  IOMUXC_SW_MUX_CTL_PAD_ENET_TD3    = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD3    = IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);
  IOMUXC_SW_MUX_CTL_PAD_ENET_TXC    = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_TXC    = IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);
  IOMUXC_SW_MUX_CTL_PAD_ENET_TX_CTL = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_TX_CTL = IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);

  /* Rx pads */
  IOMUXC_SW_MUX_CTL_PAD_ENET_RD0 = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_RD0 = IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);
  IOMUXC_SW_MUX_CTL_PAD_ENET_RD1 = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_RD1 = IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);
  IOMUXC_SW_MUX_CTL_PAD_ENET_RD2 = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_RD2 = IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);
  IOMUXC_SW_MUX_CTL_PAD_ENET_RD3 = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_RD3 = IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);
  IOMUXC_SW_MUX_CTL_PAD_ENET_RXC = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_RXC = IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);
  IOMUXC_SW_MUX_CTL_PAD_ENET_RX_CTL = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_RX_CTL = IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(0x03);


  /* MDIO/MDC */
  IOMUXC_SW_MUX_CTL_PAD_ENET_MDC = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_MDC = IOMUXC_SW_PAD_CTL_PAD_DSE(0x02);
  IOMUXC_SW_MUX_CTL_PAD_ENET_MDIO = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO = IOMUXC_SW_PAD_CTL_PAD_DSE(0x02);
  IOMUXC_ENET_QOS_GMII_MDI_I_SELECT_INPUT = 0x01; //Select ENET_MDIO, mux mode ALT0

  /* ENET_QOS PHY reset */
  IOMUXC_SW_MUX_CTL_PAD_SAI2_RXC = IOMUXC_MUX_ALT5;
  GPIO4_DR |= (1 << 22);   //Set reset signal to 1 to deassert PHY reset
  GPIO4_GDIR |= (1 << 22); // Set direction to output

  /* ENET_QOS Clock configuration */
  gpr1 = IOMUXC_GPR_GPR1;
  gpr1 |= IOMUXC_GPR_GPR1_IOMUXC_GPR_ENET_QOS_RGMII_EN_MASK
                | IOMUXC_GPR_GPR1_GPR_ENET_QOS_CLK_GEN_EN_MASK
                | IOMUXC_GPR_GPR1_GPR_ENET_QOS_INTF_SEL(0x01);
  gpr1 &= ~IOMUXC_GPR_GPR1_IOMUXC_GPR_ENET_QOS_CLK_TX_CLK_SEL_MASK;
  IOMUXC_GPR_GPR1 = gpr1;

  /* Disable ENET_QOS clocks */
  CCM_CCGR_QOS_ENET = 0x00;
  CCM_CCGR_ENET_QOS = 0x00;

  /* Set ENET_QOS_CLK_ROOT clock root to 125 MHz, CLK slice 81, SYS_PLL2_DIV8 => mux 1, enable clock */
  CCM_TARGET_ROOT_ENET_QOS = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  /* Set IMX_CCM_TARGET_ENET_QOS_TIMER clock root to 100 MHz, CLK slice 83, SYS_PLL2_DIV10 => mux 1, enable clock */
  CCM_TARGET_ROOT_ENET_QOS_TIMER = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;

  /* Enable ENET_QOS clocks */
  CCM_CCGR_QOS_ENET = 0x03;
  CCM_CCGR_ENET_QOS = 0x03;
}

/**
  Initialize PWM block and perform required pin-muxing.
**/
VOID PwmInit()
{
  int pwm_ccgr_offset = 40;
  int pwm_clk_root_offset = 103;
  // Initialize PWM1-PWM4 clocks
  for(int i = 0; i < 4; i++) {
    CCM_CCGR_CLR(pwm_ccgr_offset + i) = 0x03;
    CCM_TARGET_ROOT(pwm_clk_root_offset + i) = (CCM_TARGET_ROOT_MUX(0x00) | CCM_TARGET_ROOT_ENABLE_MASK);   // Set 25M_REF_CLK as PWM input clock, no PRE nor POST divider, Enable PWM clock
    CCM_CCGR_SET(pwm_ccgr_offset + i) = 0x03;
  }
  // Configure GPIO1_IO01 as PWM1_OUT
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO01 = IOMUXC_MUX_ALT1;
  // Drive strength 85 Ohm @3.3V, 80 Ohm @2.5V, 75 Ohm @1.8V, 90 Ohm @1.2V, Slow Frequency Slew Rate (50Mhz)
  //IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01 = (IOMUXC_SW_PAD_CTL_PAD_DSE(4) | IOMUXC_SW_PAD_CTL_PAD_FSEL(0));
}

static VOID VpuPllInit()
{
  INT32 count;

  // Bypass the VPU PLL clock
  CCM_ANALOG_VPU_PLL_GEN_CTRL = CCM_ANALOG_VPU_PLL_GEN_CTRL_PLL_BYPASS_MASK |
                                   CCM_ANALOG_VPU_PLL_GEN_CTRL_PLL_LOCK_SEL_MASK;
  // Enable reset
  CCM_ANALOG_VPU_PLL_GEN_CTRL &=  ~(CCM_ANALOG_VPU_PLL_GEN_CTRL_PLL_RST_MASK);

  // Set the GPU PLL value: MainDiv=300, PreDiv=3, PostDiv=2
  CCM_ANALOG_VPU_PLL_FDIV_CTL0 = CCM_ANALOG_VPU_PLL_FDIV_CTL0_PLL_MAIN_DIV(300) |
                                    CCM_ANALOG_VPU_PLL_FDIV_CTL0_PLL_PRE_DIV(3) |
                                    CCM_ANALOG_VPU_PLL_FDIV_CTL0_PLL_POST_DIV(2);
  MicroSecondDelay (10);
  // Disable reset
  CCM_ANALOG_VPU_PLL_GEN_CTRL |=  CCM_ANALOG_VPU_PLL_GEN_CTRL_PLL_RST_MASK;

  // Wait for VPU PLL is locked
  for (count = 0 ; count < 100; ++count) {
    if (CCM_ANALOG_VPU_PLL_GEN_CTRL & CCM_ANALOG_VPU_PLL_GEN_CTRL_PLL_LOCK_MASK) {
      break;
    }
    MicroSecondDelay (10);
  }
  // Check if lock succeeded
  if (!(CCM_ANALOG_VPU_PLL_GEN_CTRL & CCM_ANALOG_VPU_PLL_GEN_CTRL_PLL_LOCK_MASK)) {
    DebugPrint (0xFFFFFFFF, "Time out waiting for VPU_PLL to lock\n");
  }
  // Clear bypass the VPU PLL clock
  CCM_ANALOG_VPU_PLL_GEN_CTRL &= ~(CCM_ANALOG_VPU_PLL_GEN_CTRL_PLL_BYPASS_MASK);
  // Clock enable
  CCM_ANALOG_VPU_PLL_GEN_CTRL |= CCM_ANALOG_VPU_PLL_GEN_CTRL_PLL_CLKE_MASK;
  return;
}

/**
  Initialize VPU block.
**/
VOID VpuInit()
{
  ARM_SMC_ARGS smc_args;

  VpuPllInit();

  // Disable VPUMIX clock root
  CCM_CCGR_VPU = 0x00;
  // Disable VPUMIX power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_VPU_BU, 0x00, 0x00, smc_args);
  ArmCallSmc(&smc_args);
  // Disable VPUG1 clock root
  CCM_CCGR_VPU_G1 = 0x00;
  // Disable VPUG1 power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_VPU_G1, 0x00, 0x00, smc_args);
  ArmCallSmc(&smc_args);
  // Disable VPUG2 clock root
  CCM_CCGR_NOC_VPU_G2 = 0x00;
  // Disable VPUG2 power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_VPU_G2, 0x00, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  // Configure VPU_BUS clock 800MHz (SYSTEM_PLL1_CLK), PreDiv = 1, PostDiv = 1
  CCM_TARGET_ROOT_VPU_BUS = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Configure VPU_G1 clock 800MHz (VPU_PLL_CLK), PreDiv = 1, PostDiv = 1
  CCM_TARGET_ROOT_VPU_G1 = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Configure VPU_G2 clock 800MHz (VPU_PLL_CLK), PreDiv = 1, PostDiv = 1
  CCM_TARGET_ROOT_VPU_G2 = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;

  // Enable VPUMIX clock root
  CCM_CCGR_VPU = 0x03;
  // Enable VPUMIX power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_VPU_BU, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);
  // Enable VPUG1 clock root
  CCM_CCGR_VPU_G1 = 0x03;
  // Enable VPUG1 power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_VPU_G1, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);
  // Enable VPUG2 clock root
  CCM_CCGR_NOC_VPU_G2 = 0x03;
  // Enable VPUG2 power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_VPU_G2, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  // G1 fuse decoder enable
  *((volatile UINT32 *)(IMX_VPU_BLK_CTL_BASE + 0x08)) = 0xFFFFFFFF;
  // G1 fuse pp enable
  //*((volatile UINT32 *)(IMX_VPU_BLK_CTL_BASE + 0x0C)) = 0xFFFFFFFF;
  // G2 fuse decoder enable
  *((volatile UINT32 *)(IMX_VPU_BLK_CTL_BASE + 0x10)) = 0xFFFFFFFF;
}

/**
  Initialize USDHC blocks and perform required pin-muxing.
**/
VOID UsdhcInit()
{
  // USDHC2
  // SYSTEM_PLL1_DIV2 = 400MHz
  CCM_TARGET_ROOT_USDHC2 = CCM_TARGET_ROOT_MUX(0x01) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  //USDHC3
  // SYSTEM_PLL1_DIV2 = 400MHz
  CCM_TARGET_ROOT_USDHC3 = CCM_TARGET_ROOT_MUX(0x01) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;

  /* 200 mHz pad settings */
  IOMUXC_SW_PAD_CTL_PAD_NAND_WE_B   = 0x196;
  IOMUXC_SW_PAD_CTL_PAD_NAND_WP_B   = 0x1d6;
  IOMUXC_SW_PAD_CTL_PAD_NAND_DATA04 = 0x1d6;
  IOMUXC_SW_PAD_CTL_PAD_NAND_DATA05 = 0x1d6;
  IOMUXC_SW_PAD_CTL_PAD_NAND_DATA06 = 0x1d6;
  IOMUXC_SW_PAD_CTL_PAD_NAND_DATA07 = 0x1d6;
  IOMUXC_SW_PAD_CTL_PAD_NAND_RE_B   = 0x1d6;
  IOMUXC_SW_PAD_CTL_PAD_NAND_CE2_B  = 0x1d6;
  IOMUXC_SW_PAD_CTL_PAD_NAND_CE3_B  = 0x1d6;
  IOMUXC_SW_PAD_CTL_PAD_NAND_CLE    = 0x1d6;
  IOMUXC_SW_PAD_CTL_PAD_NAND_CE1_B  = 0x196;

}


/**
  Initialize GPU block.
**/
VOID GpuInit()
{
  ARM_SMC_ARGS smc_args;

  // Disable GPU clock root
  CCM_CCGR_GPU = 0x00;
  CCM_CCGR_GPU3D = 0x00;

  // Disable GPU BUS power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_GPU3D, 0x00, 0x00, smc_args);
  ArmCallSmc(&smc_args);
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_GPU_BUS, 0x00, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  // Configure GPU_CORE input clock to 1000MHz (SYSTEM_PLL2)
  CCM_TARGET_ROOT_GPU_CORE = CCM_TARGET_ROOT_MUX(4) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Configure GPU_SHADER input clock to 1000MHz (SYSTEM_PLL2)
  CCM_TARGET_ROOT_GPU_SHADER = CCM_TARGET_ROOT_MUX(4) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Configure GPU_AXI input clock to 800MHz (SYSTEM_PLL1)
  CCM_TARGET_ROOT_GPU_AXI = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Configure GPU_AHB input clock to 400MHz (SYSTEM_PLL1/2)
  CCM_TARGET_ROOT_GPU_AHB = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(1) | CCM_TARGET_ROOT_ENABLE_MASK;

  // Enable GPU clock root
  CCM_CCGR_GPU = 0x03;
  CCM_CCGR_GPU3D = 0x03;

  // Enable GPU BUS power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_GPU_BUS, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_GPU3D, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);
}


/**
  Initialize DIPLAY_MIX and perform required pin-muxing.
**/
#define LVDS_PAD_CTRL (IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | IOMUXC_SW_PAD_CTL_PAD_FSEL_MASK | IOMUXC_SW_PAD_CTL_PAD_DSE(3))
#define HDMI_IN_PAD_CTRL (IOMUXC_PAD_SRE_FAST)
VOID DisplayInit()
{
  ARM_SMC_ARGS smc_args;


  /* Enable APB, AXI clock */
  CCM_CCGR_MEDIA = 0;
    /* Set DISPLAY_AXI_CLK_ROOT clock root to SYSTEM_PLL1_CLK => 800MHz */
  CCM_TARGET_ROOT_DISPLAY_AXI = (CCM_TARGET_ROOT_MUX(2)    /* Set MUX to SYSTEM_PLL1_CLK */
                 | CCM_TARGET_ROOT_PRE_PODF(1)   /* Set PRE_PODF post divider to /2 to reach 400MHz */
                 | CCM_TARGET_ROOT_ENABLE_MASK); /* Enable clock root */

  /* Set DISPLAY_APB_CLK_ROOT clock root to SYSTEM_PLL1_CLK => 800MHz */
  CCM_TARGET_ROOT_DISPLAY_APB = (CCM_TARGET_ROOT_MUX(2)    /* Set MUX to SYSTEM_PLL1_CLK */
                 | CCM_TARGET_ROOT_PRE_PODF(3)   /* Set PRE_PODF post divider to /4 to reach 200MHz */
                 | CCM_TARGET_ROOT_ENABLE_MASK); /* Enable clock root */
  CCM_CCGR_MEDIA = 0x03;

  //
  // Enable MIPI-DSI power domain
  //
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_MIPI_PHY1, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  //
  // Enable MEDIAMIX power domain
  //
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_MEDIAMIX, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  /* Configure GPIO1_IO10 = LVDS_EN as output with log. 1 */
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO10 = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0); /* ALT0 */
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO10 = LVDS_PAD_CTRL;
  GPIO1_DR |= (0x01 << 10);                      // Set the pad to the high level
  GPIO1_GDIR |= (0x01 << 10);                    // Set output direction
  /* Configure GPIO1_IO11 = LVDS_BL_PWM as output with log. 1 */
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO11 = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0); /* ALT0 */
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO11 = LVDS_PAD_CTRL;
  GPIO1_DR |= (0x01 << 11);                      // Set the pad to the high level
  GPIO1_GDIR |= (0x01 << 11);                    // Set output direction

  // Enable HDMIMIX and HDMI_PHY power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_HDMIMIX, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_HDMI_PHY, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  //Configure HDMI pins: DDC_SCL, DDC_SDA, CEC, HPD
  IOMUXC_SW_MUX_CTL_PAD_HDMI_DDC_SCL = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0); /* ALT0 */
  IOMUXC_SW_MUX_CTL_PAD_HDMI_DDC_SDA = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0); /* ALT0 */
  IOMUXC_SW_MUX_CTL_PAD_HDMI_CEC = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0); /* ALT0 */
  IOMUXC_SW_MUX_CTL_PAD_HDMI_HPD = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0); /* ALT0 */
  IOMUXC_SW_PAD_CTL_PAD_HDMI_DDC_SCL = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_HDMI_DDC_SDA = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_HDMI_CEC = HDMI_IN_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_HDMI_HPD = HDMI_IN_PAD_CTRL;
}

/**
  Initialize ECSPI blocks and perform required pin-muxing.
**/
VOID SpiInit()
{
  // SPI2 set input clock to 24 MHz
  CCM_TARGET_ROOT_ECSPI2 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
}

#define CHECK_PCA6416_I2C_TRANSACTION_STATUS(status, message, label) \
            if (status != EFI_SUCCESS) { \
              DEBUG((DEBUG_ERROR, "PCA6416 I2C error. Register: %a, I2CStatus: %d\n", message, status)); \
              goto label; \
            }
/**
  Configure pins of PCA6416 IO expander which are specified by BitMask parameter.
**/
VOID PCA6416_ConfigurePins(IN IMX_I2C_CONTEXT  *I2cContext, IN UINT8 *BitMask, IN UINT8 *Inv, IN UINT8 *Dir, IN UINT8 *OutVal)
{
  EFI_STATUS Status;
  uint8_t RegAddr;
  uint8_t Data[2];

  /* Configure Inversion registers */
  RegAddr = PCA6416_POLARITY_INVERSION_PORT_0_REG;
  Status = iMXI2cRead(I2cContext, RegAddr, &Data[0], 2);
  CHECK_PCA6416_I2C_TRANSACTION_STATUS(Status, "PCA6416 Inversion", End);
  Data[0] = (Data[0] & ~(BitMask[0])) | (BitMask[0] & Inv[0]);
  Data[1] = (Data[1] & ~(BitMask[1])) | (BitMask[1] & Inv[1]);
  Status = iMXI2cWrite(I2cContext, RegAddr, &Data[0], 2);
  CHECK_PCA6416_I2C_TRANSACTION_STATUS(Status, "PCA6416 Inversion", End);
  /* Configure Output registers */
  RegAddr = PCA6416_OUTPUT_PORT_0_REG;
  Status = iMXI2cRead(I2cContext, RegAddr, &Data[0], 2);
  CHECK_PCA6416_I2C_TRANSACTION_STATUS(Status, "PCA6416 Output Port", End);
  Data[0] = (Data[0] & ~(BitMask[0])) | (BitMask[0] & OutVal[0]);
  Data[1] = (Data[1] & ~(BitMask[1])) | (BitMask[1] & OutVal[1]);
  Status = iMXI2cWrite(I2cContext, RegAddr, &Data[0], 2);
  CHECK_PCA6416_I2C_TRANSACTION_STATUS(Status, "PCA6416 Output Port", End);
  /* Configure Configuration (direction) registers */
  RegAddr = PCA6416_CONFIGURATION_PORT_0_REG;
  Status = iMXI2cRead(I2cContext, RegAddr, &Data[0], 2);
  CHECK_PCA6416_I2C_TRANSACTION_STATUS(Status, "PCA6416 Configuration", End);
  Data[0] = (Data[0] & ~(BitMask[0])) | (BitMask[0] & Dir[0]);
  Data[1] = (Data[1] & ~(BitMask[1])) | (BitMask[1] & Dir[1]);
  Status = iMXI2cWrite(I2cContext, RegAddr, &Data[0], 2);
  CHECK_PCA6416_I2C_TRANSACTION_STATUS(Status, "PCA6416 Configuration", End);
End:
  return;
}

/**
  Configure output P0_3 on PCA6416 so that CAN1 or I2C5 can be used.
**/
VOID SelectCAN1InsteadOfI2C5(IN BOOLEAN UseCAN)
{
  uint8_t Zero[2] = {0};
  uint8_t BitMask[2] = {0};
  uint8_t Dir[2] = {0};
  uint8_t OutVal[2] = {0};

  BitMask[0] = EXP_P0_CAN1_I2C5_SEL;
  Dir[0] = BitMask[0] & ~(EXP_P0_CAN1_I2C5_SEL);
  if (UseCAN) {
    OutVal[0] = BitMask[0] & ~(EXP_P0_CAN1_I2C5_SEL);
  } else {
    OutVal[0] = BitMask[0] | (EXP_P0_CAN1_I2C5_SEL);
  }
  PCA6416_ConfigurePins(&Pca6416I2cConfig, BitMask, Zero, Dir, OutVal);
}

/**
  Configure output P0_3 on PCA6416 so that CAN2 or PDM stream 3 can be used.
**/
VOID SelectCAN2InsteadOfPDMStream3(IN BOOLEAN UseCAN)
{
  uint8_t Zero[2] = {0};
  uint8_t BitMask[2] = {0};
  uint8_t Dir[2] = {0};
  uint8_t OutVal[2] = {0};

  BitMask[0] = EXP_P0_PDM_CAN2_SEL;
  Dir[0] = BitMask[0] & ~(EXP_P0_PDM_CAN2_SEL);
  if (UseCAN) {
    OutVal[0] = BitMask[0] | (EXP_P0_PDM_CAN2_SEL);
  } else {
    OutVal[0] = BitMask[0] & ~(EXP_P0_PDM_CAN2_SEL);
  }
  PCA6416_ConfigurePins(&Pca6416I2cConfig, BitMask, Zero, Dir, OutVal);
}

/**
  Initialize CAN modules on the SOC and perform required pin-muxing
**/
#define CAN_PAD_CTRL (IOMUXC_PAD_HYS_ENABLED | IOMUXC_PAD_PUE_ENABLE | IOMUXC_PAD_SRE_SLOW)
VOID CanInit()
{
  // IOMUX CAN PINS
  IOMUXC_SW_MUX_CTL_PAD_SPDIF_TX = IOMUXC_MUX_ALT4;     // CAN1_TX -> PAD_SPDIF_TX
  IOMUXC_SW_MUX_CTL_PAD_SPDIF_RX = IOMUXC_MUX_ALT4;     // CAN1_RX -> CAN1_RX_SELECT_INPUT
  IOMUXC_CAN1_CANRX_SELECT_INPUT = IOMUXC_MUX_ALT2;     // CAN1_RX_SELECT_INPUT -> PAD_SPDIF_RX
  IOMUXC_SW_PAD_CTL_PAD_SPDIF_TX = CAN_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_SPDIF_RX = CAN_PAD_CTRL;
  
  IOMUXC_SW_MUX_CTL_PAD_SAI5_RXD3 = IOMUXC_MUX_ALT6;    // CAN2_TX -> PAD_SAI5_RXD3
  IOMUXC_SW_MUX_CTL_PAD_SAI5_MCLK = IOMUXC_MUX_ALT6;    // CAN2_RX -> CAN2_RX_SELECT_INPUT
  IOMUXC_CAN2_CANRX_SELECT_INPUT = IOMUXC_MUX_ALT0;     // CAN2_RX_SELECT_INPUT -> PAD_SAI5_MCLK
  IOMUXC_SW_PAD_CTL_PAD_SAI5_RXD3 = CAN_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_SAI5_MCLK = CAN_PAD_CTRL;
  
  // Configures GPIO5_IO05 as output with "1". This signal is connected to the STBN1 pin of the TJA1048T chip.
  // The goal is for the CAN transceiver TJA1048T to leave standby mode
  IOMUXC_SW_MUX_CTL_PAD_SPDIF_EXT_CLK = IOMUXC_MUX_ALT5; // GPIO5_IO5 -> PAD_SPDIF_EXT_CLK
  IOMUXC_SW_PAD_CTL_PAD_SPDIF_EXT_CLK = CAN_PAD_CTRL;   
  GPIO5_DR |= (0x01 << 5);                               // Set the pad to the high level
  GPIO5_GDIR |= (0x01 << 5);
  
  // Configures GPIO4_IO27 as output with "1". This signal is connected to the STBN2 pin of the TJA1048T chip.
  IOMUXC_SW_MUX_CTL_PAD_SAI2_MCLK = IOMUXC_MUX_ALT5;     // GPIO4_IO27 -> PAD_SAI2_MCLK
  IOMUXC_SW_PAD_CTL_PAD_SAI2_MCLK = CAN_PAD_CTRL;
  GPIO4_DR |= (0x01 << 27);                              // Set the pad to the high level
  GPIO4_GDIR |= (0x01 << 27);
  

  // CAN clock
  CCM_CCGR_CAN1 = 0x00;
  CCM_CCGR_CAN2 = 0x00;
#if FixedPcdGet32(PcdCAN1XTALEnable)
  // Configure CAN1 ClkSrc = 24M_REF_CLK  (24 MHz), PreDiv = 1, PostDiv = 1, Fout = 24 MHz
  CCM_TARGET_ROOT_CAN1 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
#else
  // Configure CAN1 ClkSrc = SYSTEM_PLL1_DIV20 (40 MHz, PLL1 = 800 MHz), PreDiv = 1, PostDiv = 1, Fout = 40 MHz
  CCM_TARGET_ROOT_CAN1 = CCM_TARGET_ROOT_MUX(2) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
#endif
#if FixedPcdGet32(PcdCAN2XTALEnable)
  // Configure CAN2 ClkSrc = 24M_REF_CLK  (24 MHz), PreDiv = 1, PostDiv = 1, Fout = 24 MHz
  CCM_TARGET_ROOT_CAN2 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
#else
  // Configure CAN2 ClkSrc = SYSTEM_PLL1_DIV20 (40 MHz, PLL1 = 800 MHz), PreDiv = 1, PostDiv = 1, Fout = 40 MHz
  CCM_TARGET_ROOT_CAN2 = CCM_TARGET_ROOT_MUX(2) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
#endif
  CCM_CCGR_CAN1 = 0x03;
  CCM_CCGR_CAN2 = 0x03;
}

VOID CameraInit(VOID)
{
  UINT32 val;

  // Enable MIPI PHY power domain
  val = GPC_PGC_PU0_CTRL; // MIPI_PHY1
  val &= ~(GPC_PGC_PU_CTRL_PCR_MASK);
  GPC_PGC_PU0_CTRL = val;
  /* Enable powe domains for MIPI_PHY1 */
  val = GPC_PU_PGC_SW_PUP_REQ;
  val |= GPC_PU_PGC_SW_PUP_REQ_MIPI_PHY1_SW_PUP_REQ_MASK;
  GPC_PU_PGC_SW_PUP_REQ  = val;
  while (GPC_PU_PGC_SW_PUP_REQ & GPC_PU_PGC_SW_PUP_REQ_MIPI_PHY1_SW_PUP_REQ_MASK) {}


  val = GPC_PGC_PU16_CTRL; // MIPI_PHY2
  val &= ~(GPC_PGC_PU_CTRL_PCR_MASK);
  GPC_PGC_PU16_CTRL = val;
  /* Enable powe domains for MIPI_PHY1 */
  val = GPC_PU_PGC_SW_PUP_REQ;
  val |= GPC_PU_PGC_SW_PUP_REQ_MIPI_PHY2_SW_PUP_REQ_MASK;
  GPC_PU_PGC_SW_PUP_REQ  = val;
  while (GPC_PU_PGC_SW_PUP_REQ & GPC_PU_PGC_SW_PUP_REQ_MIPI_PHY2_SW_PUP_REQ_MASK) {}

  /* De-assert SW resets for MIPI CSI and ISI */
  MEDIA_BLK_CTRL_SFT_RSTN |= (MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_MIPI_CSI_PCLK_RESETN_MASK | MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_MIPI_CSI_ACLK_RESETN_MASK |
                              MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_MIPI_CSI2_PCLK_RESETN_MASK | MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_MIPI_CSI2_ACLK_RESETN_MASK |
                              MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_ISI_PROC_CLK_RESETN_MASK | MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_ISI_APB_CLK_RESETN_MASK |
                              MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_BUS_BLK_CLK_RESETN_MASK);
  /* Enable clocks for MIPI CSI and ISI */
  MEDIA_BLK_CTRL_CLK_EN |= (MEDIA_BLK_CTRL_CLK_EN_SFT_EN_MIPI_CSI_PCLK_MASK | MEDIA_BLK_CTRL_CLK_EN_SFT_EN_MIPI_CSI_ACLK_MASK |
                              MEDIA_BLK_CTRL_CLK_EN_SFT_EN_MIPI_CSI2_PCLK_MASK | MEDIA_BLK_CTRL_CLK_EN_SFT_EN_MIPI_CSI2_ACLK_MASK |
                              MEDIA_BLK_CTRL_CLK_EN_SFT_EN_ISI_PROC_CLK_MASK | MEDIA_BLK_CTRL_CLK_EN_SFT_EN_ISI_APB_CLK_MASK |
                              MEDIA_BLK_CTRL_CLK_EN_SFT_EN_BUS_BLK_CLK_MASK);
  /* De-assert GPR_MIPI_S_RESETN and GPR_MIPI_S2_RESETN resets */
  MEDIA_BLK_CTRL_MIPI_RESET_DIV = (MEDIA_BLK_CTRL_MIPI_RESET_DIV_GPR_MIPI_S_RESETN_MASK | MEDIA_BLK_CTRL_MIPI_RESET_DIV_GPR_MIPI_S2_RESETN_MASK);

 /* NOTE: We rely on initialization of the CCM_TARGET_ROOT_DISPLAY_AXI and CCM_TARGET_ROOT_DISPLAY_APB clock roots in DisplayInit() function */

  /* 500MHz, SYSTEM_PLL2_DIV4, Post diveder 2 */
  CCM_TARGET_ROOT_MEDIA_CAM1_PIX = CCM_TARGET_ROOT_MUX(0x4) | CCM_TARGET_ROOT_POST_PODF(0x1) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;

  /* 266MHz, SYSTEM_PLL1_DIV3 */
  CCM_TARGET_ROOT_MEDIA_CAM2_PIX = CCM_TARGET_ROOT_MUX(0x1) | CCM_TARGET_ROOT_POST_PODF(0x0) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;

  /* CLK_CLKO2 = 24 MHz to provided external clock source for external camera module */
  CCM_TARGET_ROOT_IPP_DO_CLKO2 = CCM_TARGET_ROOT_MUX(0x0) | CCM_TARGET_ROOT_POST_PODF(0x0) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;

  /* Configure GPIO1_IO15 as CCM_CLKO2 to provided external clock source for external camera module */
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO15 = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0x06);
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO15 = (IOMUXC_SW_PAD_CTL_PAD_DSE(0x01) | (0x1 << IOMUXC_SW_PAD_CTL_PAD_FSEL_SHIFT)); // 4x drive strength, fast slew

  /* Configure SD1_STROBE as GPIO2_IO11 to control CSI_PWDN for external camera module */
  IOMUXC_SW_MUX_CTL_PAD_SD1_STROBE = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0x05);
  IOMUXC_SW_PAD_CTL_PAD_SD1_STROBE =  (/* IOMUXC_SW_PAD_CTL_PAD_ODE_MASK | */ IOMUXC_SW_PAD_CTL_PAD_DSE(0x01) | (0x1 << IOMUXC_SW_PAD_CTL_PAD_FSEL_SHIFT));
  //DebugPrint (DEBUG_INFO, "Configure GPIO1_IO06 as CSI_nRST\n");
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO06 = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0x06);
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06 =  (/* IOMUXC_SW_PAD_CTL_PAD_ODE_MASK | */ IOMUXC_SW_PAD_CTL_PAD_DSE(0x01) | (0x1 << IOMUXC_SW_PAD_CTL_PAD_FSEL_SHIFT));
  // IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06 = IOMUXC_SW_PAD_CTL_PAD_DSE(0x01) | IOMUXC_SW_PAD_CTL_PAD_FSEL(0x01); // 4x drive strength, fast slew
  
  /* CSI_nRST */
  GPIO1_BASE_PTR->DR |= (0x01 << 6);
  GPIO1_BASE_PTR->GDIR |= (0x01 << 6); /* Set output direction */  
  /* CSI_POWER_DOWN */
  GPIO2_BASE_PTR->DR &= ~((UINT32)(0x01 << 11)); 
  GPIO2_BASE_PTR->GDIR |= (0x01 << 11); /* Set output direction */  
}

#if FixedPcdGet32(PcdPmicOverDriveEnable)
IMX_I2C_CONTEXT PmicI2cConfig =
{
  (uintptr_t)IMX_PMIC_I2C_BASE,         /* Base address of the I2C used for communication with PMIC */
  0,                                    /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  24000000,                             /* 24 MHz I2C ReferenceFreq */
  400000,                               /* 400 kHz required TargetFreq */
  0x25,                                 /* PMIC SlaveAddress */
  100000,                               /* TimeoutInUs */
};

/**
  Initialize MCUs clock speed and power.
**/
VOID McuClkPwrInit()
{

  EFI_STATUS Status;
  uint8_t Data;

  CCM_TARGET_ROOT_ARM_A53 = CCM_TARGET_ROOT_MUX(2) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK; /* ROOT_ARM_A53 = SYSTEM_PLL1_CLK  */
  CCM_TARGET_ROOT_SEL_ARM_A53 = CCM_TARGET_ROOT_MUX(4);                                                          /* CPU_CLK      = ARM_A53_CLK_ROOT(SYSTEM_PLL1_CLK) */
  /* Set the ARM PLL Fout = 1600 MHz, Fout = Fin/PreDiv*MainDiv/(2^PostDiv) = 24/3*200/(2^0) = 1600 MHz */
  CCM_ANALOG_ARM_PLL_FDIV_CTL0 = CCM_ANALOG_ARM_PLL_FDIV_CTL0_PLL_PRE_DIV(3) | CCM_ANALOG_ARM_PLL_FDIV_CTL0_PLL_MAIN_DIV(200) | CCM_ANALOG_ARM_PLL_FDIV_CTL0_PLL_POST_DIV(0);
  while ((CCM_ANALOG_ARM_PLL_GEN_CTRL & CCM_ANALOG_ARM_PLL_GEN_CTRL_PLL_LOCK_MASK) == 0) {};
  CCM_TARGET_ROOT_SEL_ARM_A53 = CCM_TARGET_ROOT_MUX(1);                                                          /* CPU_CLK      = ARM_PLL_CLK */
  /* Uncomment next line for ARM_PLL_CLK output on CLKOUT1 pin for CPU clock frequency measurement */
  // CCM_ANALOG_ANAMIX_PLL_MNIT_CTL = CCM_ANALOG_ANAMIX_PLL_MNIT_CTL_CLKOUT1_OUTPUT_SEL(7) | CCM_ANALOG_ANAMIX_PLL_MNIT_CTL_CLKOUT1_OUTPUT_DIV_VAL(0x0F) | CCM_ANALOG_ANAMIX_PLL_MNIT_CTL_CLKOUT1_OUTPUT_CKE_MASK;
  /* Write BUCK123_DVS. Set BUCK1 and BUCK2 to 0.95V. BUCK3 is in dual phase with BUCK1 thus no need to set it. */
  Data = (1 << 7) | (1 << 5) | (3 << 3) | (3);
  Status = iMXI2cWrite(&PmicI2cConfig, 0x0C, &Data, 1);
  if (Status != EFI_SUCCESS) {
    DEBUG((DEBUG_ERROR, "PMIC I2C error. Register: %a, I2CStatus: %d\n", "PMIC EnableOverDrive", Status));
  }
}
#endif

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
#if FixedPcdGet32(PcdPmicOverDriveEnable)
  McuClkPwrInit();
#endif
  GpioInit();
#if FixedPcdGet32(PcdPcie1Enable)
  PcieInit();
#endif
  UsbInit();
  EnetInit();
  EnetQosInit();
  AudioInit();
  I2cInit();
  PwmInit();
  VpuInit();
  UsdhcInit();
  GpuInit();
  DisplayInit();
  SpiInit();
  CanInit();
  CameraInit();

  SelectCAN1InsteadOfI2C5(FixedPcdGet32(PcdCAN1InsteadOfI2C5));
  SelectCAN2InsteadOfPDMStream3(FixedPcdGet32(PcdCAN2InsteadOfPDMStream3));

  return RETURN_SUCCESS;
}

/**
  Return the current Boot Mode. This function returns the boot reason on the platform
**/
EFI_BOOT_MODE ArmPlatformGetBootMode (VOID)
{
  return BOOT_WITH_FULL_CONFIGURATION;
}
