/** @file
*
*  Copyright (c) Microsoft Corporation. All rights reserved.
*  Copyright 2019-2020,2022-2023 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
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


#include "iMX8.h"
#include "iMX8ClkPwr.h"
#include "iMX8MIoMux.h"

#define IMX_SIP_CONFIG_GPC_PM_DOMAIN    0x03
#define IMX_PCIE1_PD                    1
#define IMX_USB1_PD                     2
#define IMX_USB2_PD                     3
#define IMX_GPU_BUS                     4
#define IMX_VPU_BUS                     5
#define IMX_MIPI_CSI1_PD                8 /* over id 3, id = ID - 3, Thus DDR changes aren't allowed. */
#define IMX_MIPI_CSI2_PD                9
#define IMX_PCIE2_PD                    10


#define IMX_VPU_BLK_CTL_BASE            0x38320000

/* Clock sources:
    16 SYSTEM_PLL1_CLK   800 System PLL1 output clock
    17 SYSTEM_PLL1_DIV2  400 System PLL1 divided 2 clock output
    18 SYSTEM_PLL1_DIV3  266 System PLL1 divided 3 clock output
    19 SYSTEM_PLL1_DIV4  200 System PLL1 divided 4 clock output
    20 SYSTEM_PLL1_DIV5  160 System PLL1 divided 5 clock output
    21 SYSTEM_PLL1_DIV6  133 System PLL1 divided 6 clock output
    22 SYSTEM_PLL1_DIV8  100 System PLL1 divided 8 clock output
    23 SYSTEM_PLL1_DIV10  80 System PLL1 divided 10 clock output
    24 SYSTEM_PLL1_DIV20  40 System PLL divided 20 clock output
    25 SYSTEM_PLL2_CLK  1000 System PLL2 output clock
    26 SYSTEM_PLL2_DIV2  500 System PLL2 divided 2 clock output
    27 SYSTEM_PLL2_DIV3  333 System PLL2 divided 3 clock output
    28 SYSTEM_PLL2_DIV4  250 System PLL2 divided 4 clock output
    29 SYSTEM_PLL2_DIV5  200 System PLL2 divided 5 clock output
    30 SYSTEM_PLL2_DIV6  166 System PLL2 divided 6 clock output
    31 SYSTEM_PLL2_DIV8  125 System PLL2 divided 8 clock output
    32 SYSTEM_PLL2_DIV10 100 System PLL2 divided 10 clock output
    33 SYSTEM_PLL2_DIV20  50 System PLL2 divided 20 clock output
    34 SYSTEM_PLL3_CLK  1000 System PLL3 output clock
*/


/* USB MIX offset */
#define USBMIX_PHY_OFFSET                        0xF0040

/* DWC3 register and bit-fields definition */    
#define DWC3_GHWPARAMS1                          0xC144

#define DWC3_GSNPSID                             0xC120

#define DWC3_GCTL                                0xC110
#define DWC3_GCTL_PWRDNSCALE_SHIFT               19
#define DWC3_GCTL_PWRDNSCALE_MASK                (0x1FFF << 19)
#define DWC3_GCTL_U2RSTECN_MASK                  (1 << 16)
#define DWC3_GCTL_PRTCAPDIR(n)                   ((n) << 12)
#define DWC3_GCTL_PRTCAP_HOST                    1
#define DWC3_GCTL_PRTCAP_DEVICE                  2
#define DWC3_GCTL_PRTCAP_OTG                     3
#define DWC3_GCTL_CORESOFTRESET_MASK             (1 << 11)
#define DWC3_GCTL_SCALEDOWN(n)                   ((n) << 4)
#define DWC3_GCTL_SCALEDOWN_MASK                 DWC3_GCTL_SCALEDOWN(3)
#define DWC3_GCTL_DISSCRAMBLE_MASK               (1 << 3)
#define DWC3_GCTL_DSBLCLKGTNG_MASK               (1 << 0)

#define DWC3_GUSB2PHYCFG                         0xC200
#define DWC3_GUSB2PHYCFG_PHYSOFTRST_MASK         (1 << 31)
#define DWC3_GUSB2PHYCFG_U2_FREECLK_EXISTS_MASK  (1 << 30)
#define DWC3_GUSB2PHYCFG_ENBLSLPM_MASK           (1 << 8)
#define DWC3_GUSB2PHYCFG_SUSPHY_MASK             (1 << 6)
#define DWC3_GUSB2PHYCFG_PHYIF_MASK              (1 << 3)

#define DWC3_GUSB3PIPECTL                        0xC2C0
#define DWC3_GUSB3PIPECTL_PHYSOFTRST_MASK        (1 << 31)

#define DWC3_GFLADJ                              0xC630
#define GFLADJ_30MHZ_REG_SEL                     (1 << 7)
#define GFLADJ_30MHZ(n)                          ((n) & 0x3f)
#define GFLADJ_30MHZ_DEFAULT                     0x20

/* USB PHYx registers and bit-fields definition */
#define USB_PHY_CTRL0                            0x0
#define USB_PHY_CTRL0_REF_SSP_EN_MASK            (1 << 2)

#define USB_PHY_CTRL1                            0x4
#define USB_PHY_CTRL1_RESET_MASK                 (1 << 0)
#define USB_PHY_CTRL1_COMMONONN_MASK             (1 << 1)
#define USB_PHY_CTRL1_ATERESET_MASK              (1 << 3)
#define USB_PHY_CTRL1_VDATSRCENB0_MASK           (1 << 19)
#define USB_PHY_CTRL1_VDATDETENB0_MASK           (1 << 20)

#define USB_PHY_CTRL2                            0x8
#define USB_PHY_CTRL2_TXENABLEN0_MASK            (1 << 8)

#define USB_PHY_CTRL6                            0x18

#define HSIO_GPR_REG_0                           (0x32F10000U)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT (1)
#define HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN       (0x1U << HSIO_GPR_REG_0_USB_CLOCK_MODULE_EN_SHIFT)

#define IMX_USB3_OTG1_BASE ((UINT64)0x38100000u)
#define IMX_USB3_OTG2_BASE ((UINT64)0x38200000u)


#define in32(_Addr)          (*(UINT32*)((void*)(UINT64)(_Addr)))
#define out32(_Addr,_Val)    (*(UINT32*)(void*)(UINT64)(_Addr)) = _Val

/**
  Initialize USB PHY.
**/
void UsbPhyInit(UINT32 base)
{
    UINT32 reg;

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
  Reset USB PHY.
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

    out32(base + DWC3_GUSB2PHYCFG, (reg | DWC3_GUSB2PHYCFG_SUSPHY_MASK));

    /* After PHYs are stable we can take Core out of reset state */
    reg = in32(base + DWC3_GCTL);
    reg &= ~DWC3_GCTL_CORESOFTRESET_MASK;
    out32(base + DWC3_GCTL, reg);
}

/**
  Initialize USB Core.
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
  Suspend XHCI clock.
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
  Initialize controller core and calls controller initialization method for OTG1, OTG2 controller..
**/
void UsbInit(void)
{
    UINT32 reg;
    ARM_SMC_ARGS smc_args;

    /* Disable USB Controller 1 power domain */
    imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_USB1_PD, 0x01, 0x00, smc_args);
    ArmCallSmc(&smc_args);
    /* Disable USB Controller 2 power domain */
    imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_USB2_PD, 0x01, 0x00, smc_args);
    ArmCallSmc(&smc_args);
    /* Disable USB Controllers clock root */
    CCM_CCGR_CLR(CCM_CCGR_IDX_USB_CTRL1) = 0x03;
    CCM_CCGR_CLR(CCM_CCGR_IDX_USB_CTRL2) = 0x03;
    /* Disable USB PHYs clock root */
    CCM_CCGR_CLR(CCM_CCGR_IDX_USB_PHY1)  = 0x03;
    CCM_CCGR_CLR(CCM_CCGR_IDX_USB_PHY2)  = 0x03;
    CCM_TARGET_ROOT_USB_BUS      = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK; /* 500MHz, SYSTEM_PLL2_DIV2 */
    CCM_TARGET_ROOT_USB_CORE_REF = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK; /* 100MHz, SYSTEM_PLL1_DIV8 */
    CCM_TARGET_ROOT_USB_PHY_REF  = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK; /* 100MHz, SYSTEM_PLL1_DIV8 */
    /* Enable USB Controllers clock root */
    CCM_CCGR_SET(CCM_CCGR_IDX_USB_CTRL1) = 0x03;
    CCM_CCGR_SET(CCM_CCGR_IDX_USB_CTRL2) = 0x03;
    /* Enable USB USB PHYs clock root */
    CCM_CCGR_SET(CCM_CCGR_IDX_USB_PHY1)  = 0x03;
    CCM_CCGR_SET(CCM_CCGR_IDX_USB_PHY2)  = 0x03;

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
    // MP Core MailBox Set/Get/Clear Addresses and Clear Value.
    // Not used with i.MX8, set to 0
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (UINT64)0
  },
#if FixedPcdGet32(PcdCoreCount) > 1
  {
    // Cluster 0, Core 1
    GET_MPID(0x0, 0x1),
    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    // Not used with i.MX8, set to 0
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
    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    // Not used with i.MX8, set to 0
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (UINT64)0
  },
  {
    // Cluster 0, Core 3
    GET_MPID(0x0, 0x3),
    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    // Not used with i.MX8, set to 0
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (EFI_PHYSICAL_ADDRESS)0x00000000,
    (UINT64)0
  }
#endif // FixedPcdGet32(PcdCoreCount) > 2
};

EFI_STATUS PrePeiCoreGetMpCoreInfo(OUT UINTN *CoreCount, OUT ARM_CORE_INFO **ArmCoreTable)
{
  // Only support one cluster
  *CoreCount = sizeof(iMX8Ppi) / sizeof(ARM_CORE_INFO);
  ASSERT (*CoreCount == FixedPcdGet32 (PcdCoreCount));
  *ArmCoreTable = iMX8Ppi;
  return EFI_SUCCESS;
}

ARM_MP_CORE_INFO_PPI mMpCoreInfoPpi = { PrePeiCoreGetMpCoreInfo };

EFI_PEI_PPI_DESCRIPTOR  gPlatformPpiTable[] = {
  {
    EFI_PEI_PPI_DESCRIPTOR_PPI,
    &gArmMpCoreInfoPpiGuid,
    &mMpCoreInfoPpi
  }
};

VOID ArmPlatformGetPlatformPpiList(OUT UINTN *PpiListSize, OUT EFI_PEI_PPI_DESCRIPTOR **PpiList)
{
  *PpiListSize = sizeof(gPlatformPpiTable);
  *PpiList = gPlatformPpiTable;
}

/**
  Initalize the Audio system
**/
#define SAI_PAD_CFG_OUT (IOMUXC_PAD_PUE_ENABLE | IOMUXC_PAD_DSE_R0_DIV_3 | IOMUXC_PAD_SRE_FAST)
VOID AudioInit(VOID)
{
  EFI_STATUS status;
  // Mux the SAI2 pins to wm8524 codec
  IOMUXC_SW_MUX_CTL_PAD_SAI2_TXFS = IOMUXC_MUX_ALT0;
  IOMUXC_SW_MUX_CTL_PAD_SAI2_TXC  = IOMUXC_MUX_ALT0;
  IOMUXC_SW_MUX_CTL_PAD_SAI2_TXD0 = IOMUXC_MUX_ALT0;
  IOMUXC_SW_MUX_CTL_PAD_SAI2_MCLK = IOMUXC_MUX_ALT0;

  IOMUXC_SW_PAD_CTL_PAD_SAI2_TXFS = SAI_PAD_CFG_OUT;
  IOMUXC_SW_PAD_CTL_PAD_SAI2_TXC  = SAI_PAD_CFG_OUT;
  IOMUXC_SW_PAD_CTL_PAD_SAI2_TXD0 = SAI_PAD_CFG_OUT;
  IOMUXC_SW_PAD_CTL_PAD_SAI2_MCLK = SAI_PAD_CFG_OUT;
  // unmute audio
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO08 = IOMUXC_MUX_ALT0;
  GPIO1_DR   |= GPIO_DR_DR(1 << 8);
  GPIO1_GDIR |= GPIO_DR_DR(1 << 8);
  // enable the AUDIO_PLL - 44,100 Hz * 256
  status = ImxSetSAI2ClockRate (11289600);
  if (EFI_ERROR (status)) {
    DebugPrint (DEBUG_ERROR, "AudioInit - ImxSetAudioMclkClockRate failed");
  }
}

/**
  Initialize MIPI CSI block and perform required pin-muxing.
**/
VOID CameraInit(VOID)
{
  ARM_SMC_ARGS smc_args;

  /* Enable MIPI CSI 1 power domain */
  DebugPrint(DEBUG_INFO, "Will do imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_MIPI_CSI1_PD).\n");
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_MIPI_CSI1_PD, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  /* Enable MIPI CSI 2 power domain */
  DebugPrint(DEBUG_INFO, "Will do imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_MIPI_CSI2_PD).\n");
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_MIPI_CSI2_PD, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  CCM_CCGR_MIPI_CSI1 = 0x00;
  CCM_CCGR_MIPI_CSI2 = 0x00;
  MicroSecondDelay(50);
  // CLK_CLKO2 = 23.5 MHz - max 266, Linux 20. SYSTEM_PLL1_DIV2 (400 MHz)
  CCM_TARGET_ROOT_IPP_DO_CLKO2 = CCM_TARGET_ROOT_MUX(0x2) | CCM_TARGET_ROOT_POST_PODF(0x10) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  
  // MIPI_CSI1_CORE = 133 MHz - max 266, Linux 133, QNX 333. SYSTEM_PLL1_DIV3 (266 MHz) / 2
  CCM_TARGET_ROOT_MIPI_CSI1_CORE = CCM_TARGET_ROOT_MUX(0x1) | CCM_TARGET_ROOT_POST_PODF(0x1) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // 100 SYSTEM_PLL2_DIV10
  CCM_TARGET_ROOT_MIPI_CSI1_PHY_REF = CCM_TARGET_ROOT_MUX(0x3) | CCM_TARGET_ROOT_POST_PODF(0x7) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // 66,6 MHz - 800/12 (SYSTEM_PLL1_CLK)
  CCM_TARGET_ROOT_MIPI_CSI1_ESC = CCM_TARGET_ROOT_MUX(0x3) | CCM_TARGET_ROOT_POST_PODF(0xB) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  
  // MIPI_CSI2_CORE = 133 MHz - max 266, Linux 133, QNX 333. SYSTEM_PLL1_DIV3 (266 MHz) / 2
  CCM_TARGET_ROOT_MIPI_CSI2_CORE = CCM_TARGET_ROOT_MUX(0x1) | CCM_TARGET_ROOT_POST_PODF(0x1) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // 100 SYSTEM_PLL2_DIV10
  CCM_TARGET_ROOT_MIPI_CSI2_PHY_REF = CCM_TARGET_ROOT_MUX(0x3) | CCM_TARGET_ROOT_POST_PODF(0x7) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // 66,6 MHz - 800/12 (SYSTEM_PLL1_CLK)
  CCM_TARGET_ROOT_MIPI_CSI2_ESC = CCM_TARGET_ROOT_MUX(0x3) | CCM_TARGET_ROOT_POST_PODF(0xB) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  
  MicroSecondDelay(50);
  // Enable clock gates
  CCM_CCGR_MIPI_CSI2 = 0x03;
  CCM_CCGR_MIPI_CSI1 = 0x03;
  
  MicroSecondDelay(50); 
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06 = 0x19; // pinctrl_csi_rst
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03 = 0x19; // pinctrl_csi1_pwn

  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO15 = 0x59; // pinctrl_CLKO2
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO15 = 0x6; // muxctrl_CLKO2

  /* CSI_nRST */
  GPIO1_BASE_PTR->DR |= (0x01 << 6);
  /* CSI_POWER_DOWN */
  GPIO1_BASE_PTR->DR &= ~((UINT32)(0x01 << 3));
  /* Set output direction */
  GPIO1_BASE_PTR->GDIR |= (0x01 << 3) | (0x01 << 6); 
}

/**
  Initialize I2C modules on the SOC and perform required pin-muxing
**/
#define I2C_PAD_CTRL (IOMUXC_PAD_DSE_R0_DIV_3 | IOMUXC_PAD_SRE_SLOW | IOMUXC_PAD_ODE_ENABLED | \
                      IOMUXC_PAD_HYS_ENABLED | IOMUXC_PAD_PUE_ENABLE)
VOID I2cInit()
{
  IOMUXC_SW_MUX_CTL_PAD_I2C1_SCL = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_I2C1_SDA = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_I2C2_SCL = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_I2C2_SDA = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_I2C3_SCL = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;
  IOMUXC_SW_MUX_CTL_PAD_I2C3_SDA = IOMUXC_MUX_ALT0 | IOMUXC_MUX_SION_ENABLED;

  IOMUXC_SW_PAD_CTL_PAD_I2C1_SCL = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_I2C1_SDA = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_I2C2_SCL = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_I2C2_SDA = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_I2C3_SCL = I2C_PAD_CTRL;
  IOMUXC_SW_PAD_CTL_PAD_I2C3_SDA = I2C_PAD_CTRL;
}

/**
  Initialize ENETs modules on the SOC and perform required pin-muxing.
**/
#define PIN_ENET_RST_B (1<<9)
#define ENET_MDC_PAD_CTRL     (IOMUXC_PAD_DSE_R0_DIV_3 | IOMUXC_PAD_SRE_SLOW)
#define ENET_MDIO_PAD_CTRL    (IOMUXC_PAD_DSE_R0_DIV_3 | IOMUXC_PAD_SRE_SLOW | IOMUXC_PAD_ODE_ENABLED)
#define ENET_TX_PAD_CTRL      (IOMUXC_PAD_DSE_R0_DIV_7 | IOMUXC_PAD_SRE_MAX)
#define ENET_RX_PAD_CTRL      (IOMUXC_PAD_DSE_R0_DIV_1 | IOMUXC_PAD_SRE_FAST | IOMUXC_PAD_HYS_ENABLED)
VOID EnetInit(VOID)
{
  // ENET1/2 MDIO bus (both ENETs share one MDIO bus conected to the ENET1 controller)
  IOMUXC_SW_MUX_CTL_PAD_ENET_MDC    = IOMUXC_MUX_ALT0;    // ENET1_MDC  -> PAD_GPIO1_IO11
  IOMUXC_SW_MUX_CTL_PAD_ENET_MDIO   = IOMUXC_MUX_ALT0;    // ENET1_MDIO -> PAD_PAD_ENET_MDIO
  IOMUXC_ENET1_MDIO_SELECT_INPUT    = IOMUXC_MUX_ALT1;    // ENET1_MDIO <- PAD_PAD_ENET_MDIO
  IOMUXC_SW_PAD_CTL_PAD_ENET_MDC    = ENET_MDC_PAD_CTRL;  // ENET1_MDC  electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO   = ENET_MDIO_PAD_CTRL; // ENET1_MDIO electrical settings
  // ENET1 RGMMI pins routing
  IOMUXC_SW_MUX_CTL_PAD_ENET_TD3    = IOMUXC_MUX_ALT0;    // ENET1_RGMII_TD3    -> PAD_ENET1_TD3
  IOMUXC_SW_MUX_CTL_PAD_ENET_TD2    = IOMUXC_MUX_ALT0;    // ENET1_RGMII_TD2    -> PAD_ENET1_TD2
  IOMUXC_SW_MUX_CTL_PAD_ENET_TD1    = IOMUXC_MUX_ALT0;    // ENET1_RGMII_TD1    -> PAD_ENET1_TD1
  IOMUXC_SW_MUX_CTL_PAD_ENET_TD0    = IOMUXC_MUX_ALT0;    // ENET1_RGMII_TD0    -> PAD_ENET1_TD0
  IOMUXC_SW_MUX_CTL_PAD_ENET_TX_CTL = IOMUXC_MUX_ALT0;    // ENET1_RGMII_TX_CTL -> PAD_ENET1_TX_CTL
  IOMUXC_SW_MUX_CTL_PAD_ENET_TXC    = IOMUXC_MUX_ALT0;    // ENET1_RGMII_TXC    -> PAD_ENET1_TXC
  IOMUXC_SW_MUX_CTL_PAD_ENET_RD3    = IOMUXC_MUX_ALT0;    // ENET1_RGMII_RD3    <- PAD_ENET1_RD3
  IOMUXC_SW_MUX_CTL_PAD_ENET_RD2    = IOMUXC_MUX_ALT0;    // ENET1_RGMII_RD2    <- PAD_ENET1_RD2
  IOMUXC_SW_MUX_CTL_PAD_ENET_RD1    = IOMUXC_MUX_ALT0;    // ENET1_RGMII_RD1    <- PAD_ENET1_RD1
  IOMUXC_SW_MUX_CTL_PAD_ENET_RD0    = IOMUXC_MUX_ALT0;    // ENET1_RGMII_RD0    <- PAD_ENET1_RD0
  IOMUXC_SW_MUX_CTL_PAD_ENET_RX_CTL = IOMUXC_MUX_ALT0;    // ENET1_RGMII_RX_CTL <- PAD_ENET1_RX_CTL
  IOMUXC_SW_MUX_CTL_PAD_ENET_RXC    = IOMUXC_MUX_ALT0;    // ENET1_RGMII_RXC    <- PAD_ENET1_RXC
  // ENET1 RGMMI pins electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_RD3    = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RD3    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_RD2    = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RD2    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_RD1    = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RD1    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_RD0    = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RD0    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_RX_CTL = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RX_CTL electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_RXC    = ENET_RX_PAD_CTRL;   // ENET1_RGMII_RXC    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD3    = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD3    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD2    = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD2    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD1    = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD1    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD0    = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD0    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TX_CTL = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TX_CTL electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TXC    = ENET_TX_PAD_CTRL;   // ENET1_RGMII_TXC    electrical settings
  // Configure ENET_nRST signal
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO09 = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO09 = IOMUXC_PAD_DSE_R0_DIV_1 | IOMUXC_PAD_SRE_SLOW;
  GPIO1_DR &= ~PIN_ENET_RST_B;                                   // Set ENET_nRST = 0
  GPIO1_GDIR |= PIN_ENET_RST_B;                                  // Set direction to output
  MicroSecondDelay(500);
  GPIO1_DR |= PIN_ENET_RST_B;                                    // Set ENET_nRST = 1
  CCM_CCGR_ENET1 = 0x00000003;                                   // ENET1 clock gate
}

#if (FixedPcdGet32(PcdPcie1Enable) || FixedPcdGet32(PcdPcie2Enable))
/**
  Initialize PCI Express module on the SOC and perform required pin-muxing
**/
VOID PcieInit ()
{
  ARM_SMC_ARGS smc_args;

  /* Enable PCIe1 power domain */
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_PCIE1_PD, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);
  /* Enable PCIe2 power domain */
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_PCIE2_PD, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);

#if FixedPcdGet32(PcdPcie1Enable)
  /* Disable PCIE_CTRL clock root */
  CCM_CCGR_CLR(37) = 0x03;
  /* Set PCIe controller input clock to 250MHz (SYSTEM_PLL2_DIV4) */
  CCM_TARGET_ROOT(70) = CCM_TARGET_ROOT_MUX(0x01);
  CCM_TARGET_ROOT_SET(70) = CCM_TARGET_ROOT_ENABLE_MASK;
  /* Set PCIe PHy input clock to 100MHz (SYSTEM_PLL2_DIV10) */
  CCM_TARGET_ROOT(71) = CCM_TARGET_ROOT_MUX(0x01);
  CCM_TARGET_ROOT_SET(71) = CCM_TARGET_ROOT_ENABLE_MASK;
  /* Set PCIe AUX input clock to 10MHz (SYSTEM_PLL2_DIV20 / 5) */
  CCM_TARGET_ROOT(72) = CCM_TARGET_ROOT_MUX(0x02) | CCM_TARGET_ROOT_POST_PODF(4);
  CCM_TARGET_ROOT_SET(72) = CCM_TARGET_ROOT_ENABLE_MASK;
  /* Enable PCIE_CTRL clock root */
  CCM_CCGR_SET(37) = 0x03;

  /* Configure NAND_DQS as GPIO to control PCIe WL_nWAKE PAD */
  IOMUXC_SW_MUX_CTL_PAD_NAND_DQS = IOMUXC_MUX_ALT5;
  GPIO3_GDIR &= ~(0x01 << 14);                   /* Set input direction */

  /* Configure UART4_RXD as GPIO to control PCIe WL_nPERST PAD */
  IOMUXC_SW_MUX_CTL_PAD_UART4_RXD = IOMUXC_MUX_ALT5;
  GPIO5_DR &= ~(0x01 << 28);                    /* Set the pad to the low level */
  GPIO5_GDIR |= (0x01 << 28);                   /* Set output direction */

  /* Configure UART4_TXD to control PCIe WL_REG_ON PAD */
  IOMUXC_SW_MUX_CTL_PAD_UART4_TXD = IOMUXC_MUX_ALT5;
  GPIO5_DR |= (0x01 << 29);                     /* Set the pad to the high level */
  GPIO5_GDIR |= (0x01 << 29);                   /* Set output direction */
#endif
#if FixedPcdGet32(PcdPcie2Enable)
  /* Disable PCIE_CTRL clock root */
  CCM_CCGR_CLR(100) = 0x03;
  /* Set PCIe controller input clock to 250MHz (SYSTEM_PLL2_DIV4) */
  CCM_TARGET_ROOT(128) = CCM_TARGET_ROOT_MUX(0x01);
  CCM_TARGET_ROOT_SET(128) = CCM_TARGET_ROOT_ENABLE_MASK;
  /* Set PCIe PHy input clock to 100MHz (SYSTEM_PLL2_DIV10) */
  CCM_TARGET_ROOT(129) = CCM_TARGET_ROOT_MUX(0x01);
  CCM_TARGET_ROOT_SET(129) = CCM_TARGET_ROOT_ENABLE_MASK;
  /* Set PCIe AUX input clock to 10MHz (SYSTEM_PLL2_DIV20 / 5) */
  CCM_TARGET_ROOT(130) = CCM_TARGET_ROOT_MUX(0x02) | CCM_TARGET_ROOT_POST_PODF(4);
  CCM_TARGET_ROOT_SET(130) = CCM_TARGET_ROOT_ENABLE_MASK;
  /* Enable PCIE_CTRL clock root */
  CCM_CCGR_SET(100) = 0x03;

  /* Configure ECSPI2_MOSI as GPIO to control PCIe nWAKE PAD */
  IOMUXC_SW_MUX_CTL_PAD_ECSPI2_MOSI = IOMUXC_MUX_ALT5;
  GPIO5_GDIR &= ~(0x01 << 7);                   /* Set input direction */

  /* Configure ECSPI2_MISO as GPIO to control PCIe nPERST PAD */
  IOMUXC_SW_MUX_CTL_PAD_ECSPI2_MISO = IOMUXC_MUX_ALT5;
  GPIO5_DR &= ~(0x01 << 12);                    /* Set the pad to the low level */
  GPIO5_GDIR |= (0x01 << 12);                   /* Set output direction */

  /* Configure ECSPI2_SCLK to control PCIe nDISABLE PAD */
  IOMUXC_SW_MUX_CTL_PAD_ECSPI2_SCLK = IOMUXC_MUX_ALT5;
  GPIO5_DR |= (0x01 << 10);                     /* Set the pad to the high level */
  GPIO5_GDIR |= (0x01 << 10);                   /* Set output direction */

  // Configure I2C4_SDA to control PCIE nCLKREQ PAD
  IOMUXC_SW_MUX_CTL_PAD_I2C4_SDA = IOMUXC_MUX_ALT5;
  GPIO5_DR &= ~(0x01 << 21);                    /* Set the pad to the low level */
  GPIO5_GDIR |= (0x01 << 21);                   /* Set output direction */
  IOMUXC_SW_PAD_CTL_PAD_I2C4_SDA = IOMUXC_PAD_ODE_ENABLED | IOMUXC_PAD_DSE_R0_DIV_6;

#endif
}
#endif

/**
  Initialize PWM block and perform required pin-muxing.
**/
VOID PwmInit()
{
  int pwm_ccgr_offset = 40;
  int pwm_clk_root_offset = 103;

  /* Initialize PWM1-PWM4 clocks */
  for(int i = 0; i < 4; i++)
  {
  CCM_CCGR_CLR(pwm_ccgr_offset + i) = 0x03;
  CCM_TARGET_ROOT(pwm_clk_root_offset + i) = (CCM_TARGET_ROOT_MUX(0x00) | CCM_TARGET_ROOT_ENABLE_MASK);       /* Set 25M_REF_CLK as PWM input clock, no PRE nor POST divider, Enable PWM clock */
  CCM_CCGR_SET(pwm_ccgr_offset + i) = 0x03;
  }

  /* Configure GPIO1_IO01 as PWM1_OUT */
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO01 = IOMUXC_MUX_ALT1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01 = (IOMUXC_SW_PAD_CTL_PAD_DSE(4) | /* Drive strength 85 Ohm @3.3V, 80 Ohm @2.5V, 75 Ohm @1.8V, 90 Ohm @1.2V */
                                      IOMUXC_SW_PAD_CTL_PAD_SRE(0)); /* Slow Frequency Slew Rate (50Mhz) */
}

/**
  Initialize VPU_PLL block.
**/
static VOID VpuPllInit()
{
  INT32 count;

  // Bypass the VPU PLL clock
  CCM_ANALOG_VPU_PLL_CFG0 |= CCM_ANALOG_VPU_PLL_CFG0_PLL_BYPASS_MASK;

  CCM_ANALOG_VPU_PLL_CFG1 = CCM_ANALOG_VPU_PLL_CFG1_PLL_INT_DIV_CTL(59);
  CCM_ANALOG_VPU_PLL_CFG0 |=
    CCM_ANALOG_VPU_PLL_CFG0_PLL_BYPASS_MASK |
    CCM_ANALOG_VPU_PLL_CFG0_PLL_CLKE_MASK |
    CCM_ANALOG_VPU_PLL_CFG0_PLL_REFCLK_SEL(0b00) |
    CCM_ANALOG_VPU_PLL_CFG0_PLL_LOCK_SEL_MASK |
    CCM_ANALOG_VPU_PLL_CFG0_PLL_NEWDIV_VAL_MASK |
    CCM_ANALOG_VPU_PLL_CFG0_PLL_REFCLK_DIV_VAL(4) |
    CCM_ANALOG_VPU_PLL_CFG0_PLL_OUTPUT_DIV_VAL(1);

  // Clear bypass the VPU PLL clock
  CCM_ANALOG_VPU_PLL_CFG0 &= ~(CCM_ANALOG_VPU_PLL_CFG0_PLL_BYPASS_MASK);
  // Wait for VPU PLL is locked
  for (count = 0; count < 100; ++count) {
    if (CCM_ANALOG_VPU_PLL_CFG0 & CCM_ANALOG_VPU_PLL_CFG0_PLL_LOCK_MASK) {
      break;
    }
    MicroSecondDelay (10);
  }
  // Check if lock succeeded
  if (!(CCM_ANALOG_VPU_PLL_CFG0 & CCM_ANALOG_VPU_PLL_CFG0_PLL_LOCK_MASK)) {
    DebugPrint (0xFFFFFFFF, "Time out waiting for VPU_PLL to lock\n");
  }

  // Clear PLL new fraction divide input control
  CCM_ANALOG_VPU_PLL_CFG0 &= ~(CCM_ANALOG_VPU_PLL_CFG0_PLL_NEWDIV_VAL_MASK);
  return;
}

/**
  Initialize VPU block.
**/
VOID VpuInit()
{
  ARM_SMC_ARGS smc_args;

  VpuPllInit();

  // Disable VPU_DEC clock root
  CCM_CCGR_VPU_DEC = 0x00;
  // Disable VA53 clock root
  CCM_CCGR_VA53 = 0x00;
  // Disable VP9 clock root
  CCM_CCGR_VP9 = 0x00;

  // Disable VPU BUS power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_VPU_BUS, 0x00, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  // Configure VPU_BUS input clock to 800MHz (SYSTEM_PLL1)
  CCM_TARGET_ROOT_VPU_BUS = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Configure VPU_G1 input clock to 600MHz (VPU_PLL)
  CCM_TARGET_ROOT_VPU_G1 = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Configure VPU_G2 input clock to 600MHz (VPU_PLL)
  CCM_TARGET_ROOT_VPU_G2 = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;

  // Enable VPU_DEC clock root
  CCM_CCGR_VPU_DEC = 0x03;
  // Enable VA53 clock root
  CCM_CCGR_VA53 = 0x03;
  // Enable VP9 clock root
  CCM_CCGR_VP9 = 0x03;

  // Enable VPU BUS power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_VPU_BUS, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  // VPUMIX G1/G2 soft reset control
  *((volatile UINT32 *)(IMX_VPU_BLK_CTL_BASE + 0x00)) = 0x03;
  // VPUMIX G1/G2 block clock enable
  *((volatile UINT32 *)(IMX_VPU_BLK_CTL_BASE + 0x04)) = 0x03;
  // G1 fuse decoder enable
  *((volatile UINT32 *)(IMX_VPU_BLK_CTL_BASE + 0x08)) = 0xFFFFFFFF;
  // G1 fuse pp enable
  *((volatile UINT32 *)(IMX_VPU_BLK_CTL_BASE + 0x0C)) = 0xFFFFFFFF;
  // G2 fuse decoder enable
  *((volatile UINT32 *)(IMX_VPU_BLK_CTL_BASE + 0x10)) = 0xFFFFFFFF;
}

/**
  Initialize GPU block.
**/
VOID GpuInit()
{
  ARM_SMC_ARGS smc_args;

  // Disable GPU clock root
  CCM_CCGR_GPU = 0x00;

  // Disable GPU BUS power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_GPU_BUS, 0x00, 0x00, smc_args);
  ArmCallSmc(&smc_args);

  // Configure GPU_CORE input clock to 800MHz (SYSTEM_PLL1)
  CCM_TARGET_ROOT_GPU_CORE = CCM_TARGET_ROOT_MUX(2) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Configure GPU_SHADER input clock to 1000MHz (SYSTEM_PLL2)
  CCM_TARGET_ROOT_GPU_SHADER = CCM_TARGET_ROOT_MUX(4) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Configure GPU_AXI input clock to 800MHz (SYSTEM_PLL1)
  CCM_TARGET_ROOT_GPU_AXI = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // Configure GPU_AHB input clock to 400MHz (SYSTEM_PLL1/2)
  CCM_TARGET_ROOT_GPU_AHB = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(1) | CCM_TARGET_ROOT_ENABLE_MASK;

  // Enable GPU clock root
  CCM_CCGR_GPU = 0x03;

  // Enable GPU BUS power domain
  imx_fill_sip(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, IMX_GPU_BUS, 0x01, 0x00, smc_args);
  ArmCallSmc(&smc_args);
}

/**
  Initialize USDHC blocks and perform required pin-muxing.
**/
VOID UsdhcInit()
{
  // USDHC1
  // SYSTEM_PLL1_DIV2 = 400MHz
  CCM_TARGET_ROOT_USDHC1 = CCM_TARGET_ROOT_MUX(0x01) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // USDHC2
  // SYSTEM_PLL1_DIV2 = 400MHz
  CCM_TARGET_ROOT_USDHC2 = CCM_TARGET_ROOT_MUX(0x01) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;

  /* 200 mHz pad settings */
  IOMUXC_SW_PAD_CTL_PAD_SD1_CLK     = 0x9f;
  IOMUXC_SW_PAD_CTL_PAD_SD1_CMD     = 0xdf;
  IOMUXC_SW_PAD_CTL_PAD_SD1_DATA0   = 0xdf;
  IOMUXC_SW_PAD_CTL_PAD_SD1_DATA1   = 0xdf;
  IOMUXC_SW_PAD_CTL_PAD_SD1_DATA2   = 0xdf;
  IOMUXC_SW_PAD_CTL_PAD_SD1_DATA3   = 0xdf;
  IOMUXC_SW_PAD_CTL_PAD_SD1_DATA4   = 0xdf;
  IOMUXC_SW_PAD_CTL_PAD_SD1_DATA5   = 0xdf;
  IOMUXC_SW_PAD_CTL_PAD_SD1_DATA6   = 0xdf;
  IOMUXC_SW_PAD_CTL_PAD_SD1_DATA7   = 0xdf;
  IOMUXC_SW_PAD_CTL_PAD_SD1_STROBE  = 0x9f;
  IOMUXC_SW_PAD_CTL_PAD_SD1_RESET_B = 0xc1;

}

#define ARM_PLL_REF_CLK                         25000000UL

#define IMX_OCOTP_TESTER3_SPEED_SHIFT           8
#define IMX_OCOTP_TESTER3_SPEED_MASK            (0x03 << IMX_OCOTP_TESTER3_SPEED_SHIFT)

#define IMX_OCOTP_TESTER3_800MHZ_SPEED          0
#define IMX_OCOTP_TESTER3_1000MHZ_SPEED         1
#define IMX_OCOTP_TESTER3_1300MHZ_SPEED         2
#define IMX_OCOTP_TESTER3_1500MHZ_SPEED         3

/**
  Initialize MCUs clock speed and power.
**/
VOID McuClkPwrInit()
{
  UINT32        val, pll_freq_hz = 0, mcu_core_voltage;
  UINT32        ref_clk, int_div, out_div;
  UINT64        frac_div;

  val = (OCOTP_HW_OCOTP_TESTER3 & IMX_OCOTP_TESTER3_SPEED_MASK) >> IMX_OCOTP_TESTER3_SPEED_SHIFT;
  switch (val) {
    case IMX_OCOTP_TESTER3_800MHZ_SPEED:
      pll_freq_hz = 800000000;
      mcu_core_voltage = 900;
    break;
    case IMX_OCOTP_TESTER3_1000MHZ_SPEED:
      pll_freq_hz = 1000000000;
      mcu_core_voltage = 900;
    break;
    case IMX_OCOTP_TESTER3_1300MHZ_SPEED:
      pll_freq_hz = 1300000000;
      mcu_core_voltage = 1000;
    break;
    case IMX_OCOTP_TESTER3_1500MHZ_SPEED:
      pll_freq_hz = 1500000000;
      mcu_core_voltage = 1000;
    break;
  }
  if (pll_freq_hz != 0) {
    DebugPrint (DEBUG_INFO, "Configure Cortex-A53 cores speed to %dMHz, %dmV\n", pll_freq_hz, mcu_core_voltage);
    ref_clk = (ARM_PLL_REF_CLK / (((CCM_ANALOG_ARM_PLL_CFG0 & CCM_ANALOG_ARM_PLL_CFG0_PLL_REFCLK_DIV_VAL_MASK) >>
              CCM_ANALOG_ARM_PLL_CFG0_PLL_REFCLK_DIV_VAL_SHIFT) + 1)) * 8;

    pll_freq_hz *= 2;
    int_div = pll_freq_hz / ref_clk;
    frac_div = (UINT64)(pll_freq_hz - int_div * ref_clk) * (1UL << 24);
    frac_div /= ref_clk;

    /* Bypass CCM A53 root, use ARM_PLL -> MUX -> cores A53 */
    CCM_TARGET_ROOT_CORE_SEL_CFG = CCM_TARGET_ROOT_MUX(0x01);

    CCM_ANALOG_ARM_PLL_CFG1 = (CCM_ANALOG_ARM_PLL_CFG1 & ~(CCM_ANALOG_ARM_PLL_CFG1_PLL_FRAC_DIV_CTL_MASK |
                              CCM_ANALOG_ARM_PLL_CFG1_PLL_INT_DIV_CTL_MASK)) |
                              (CCM_ANALOG_ARM_PLL_CFG1_PLL_FRAC_DIV_CTL((UINT32)frac_div) |
                              CCM_ANALOG_ARM_PLL_CFG1_PLL_INT_DIV_CTL(int_div - 1));
    out_div = 0x1f; //0x1f = 31 => value (31 + 1)*2 = 64 max value, 2 is min, 64 is max (=no division)
    CCM_ANALOG_ARM_PLL_CFG0 = (CCM_ANALOG_ARM_PLL_CFG0 & ~(CCM_ANALOG_ARM_PLL_CFG0_PLL_OUTPUT_DIV_VAL_MASK)) |
        CCM_ANALOG_ARM_PLL_CFG0_PLL_OUTPUT_DIV_VAL(out_div);

    /* Reload FracDivCtl & IntDivClt values */
    CCM_ANALOG_ARM_PLL_CFG0 |= CCM_ANALOG_ARM_PLL_CFG0_PLL_NEWDIV_VAL_MASK;
    /* Wait for PLL ACK for new configuration */
    while (CCM_ANALOG_ARM_PLL_CFG0 & CCM_ANALOG_ARM_PLL_CFG0_PLL_NEWDIV_ACK_MASK) {}
    CCM_ANALOG_ARM_PLL_CFG0 &= ~(CCM_ANALOG_ARM_PLL_CFG0_PLL_NEWDIV_VAL_MASK);
    CCM_ANALOG_ARM_PLL_CFG0 &= ~(CCM_ANALOG_ARM_PLL_CFG0_PLL_OUTPUT_DIV_VAL_MASK);
    /* Configure cores power supply */
    /* Configure GPIO1_IO13 as GPIO to control PWM_LED PAD */
    IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO13 = IOMUXC_MUX_ALT0;
    if (mcu_core_voltage == 1000) {
      GPIO1_DR &= ~(0x01 << 13);                        /* Set the pad to the low level */
    } else {
      GPIO1_DR |= (0x01 << 13);                         /* Set the pad to the high level */
    }
    GPIO1_GDIR |= (0x01 << 13);                   /* Set output direction */
  }
}

/**
  Initialize controllers that must setup at the early stage
**/
RETURN_STATUS ArmPlatformInitialize(IN UINTN MpId)
{
  if (!ArmPlatformIsPrimaryCore(MpId)) {
    return RETURN_SUCCESS;
  }
  // Initialize debug serial port
  SerialPortInitialize();
  SerialPortWrite((UINT8 *)SERIAL_DEBUG_PORT_INIT_MSG, (UINTN)sizeof(SERIAL_DEBUG_PORT_INIT_MSG));

  // Initialize peripherals
  ImxUngateActiveClock();
  McuClkPwrInit();
  AudioInit();
  I2cInit();
  EnetInit();
#if (FixedPcdGet32(PcdPcie1Enable) || FixedPcdGet32(PcdPcie2Enable))
  PcieInit ();
#endif
  UsbInit();
  PwmInit();
  VpuInit();
  GpuInit();
  UsdhcInit();
  CameraInit();

  return RETURN_SUCCESS;
}

/**
  Return the current Boot Mode
  This function returns the boot reason on the platform
**/
EFI_BOOT_MODE ArmPlatformGetBootMode (VOID)
{
  return BOOT_WITH_FULL_CONFIGURATION;
}
