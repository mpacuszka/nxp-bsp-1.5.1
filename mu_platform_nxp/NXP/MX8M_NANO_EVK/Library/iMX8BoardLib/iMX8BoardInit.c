/** @file
*
*  Copyright (c) 2018, Microsoft Corporation. All rights reserved.
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
#include <iMXI2cLib.h>

#include "iMX8.h"

#define IMX_I2C1_BASE                   0x30A20000
#define IMX_PMIC_I2C_BASE               IMX_I2C1_BASE

#define IMX_SIP_CONFIG_GPC_PM_DOMAIN    0x03
/* Values taken from imx-atf/plat/imx/imx8m/imx8mn/gpc.c */
#define ATF_PU_HSIOMIM                  1
#define ATF_PU_OTG1                     2
#define ATF_PU_GPUMIX                   4
#define ATF_PU_DISPMIX                  9
#define ATF_PU_MIPI                     10

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

VOID ArmPlatformGetPlatformPpiList (OUT UINTN *PpiListSize, OUT EFI_PEI_PPI_DESCRIPTOR **PpiList)
{
  *PpiListSize = sizeof(gPlatformPpiTable);
  *PpiList     = gPlatformPpiTable;
}

VOID UngateClocks ()
{
  // Enable SDMA
  CCM_CCGR_SDMA1 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
  CCM_CCGR_SDMA2 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
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
VOID AudioInit(VOID)
{
  INT32 count;
  UINT32 preDivSelect = 8;
  UINT32 postDivSelect = 4;
  // Mux the SAI3 pins to wm8524 codec
  IOMUXC_SW_MUX_CTL_PAD_SAI3_TXFS = IOMUXC_MUX_ALT0;
  IOMUXC_SW_MUX_CTL_PAD_SAI3_TXC  = IOMUXC_MUX_ALT0;
  IOMUXC_SW_MUX_CTL_PAD_SAI3_TXD  = IOMUXC_MUX_ALT0;
  IOMUXC_SW_MUX_CTL_PAD_SAI3_MCLK = IOMUXC_MUX_ALT0;

  IOMUXC_SW_PAD_CTL_PAD_SAI3_TXFS = SAI_PAD_CFG_OUT;
  IOMUXC_SW_PAD_CTL_PAD_SAI3_TXC  = SAI_PAD_CFG_OUT;
  IOMUXC_SW_PAD_CTL_PAD_SAI3_TXD  = SAI_PAD_CFG_OUT;
  IOMUXC_SW_PAD_CTL_PAD_SAI3_MCLK = SAI_PAD_CFG_OUT;
  // Unmute audio
  IOMUXC_SW_MUX_CTL_PAD_I2C4_SDA = IOMUXC_MUX_ALT5;
  GPIO5_DR   |= GPIO_DR_DR(1 << 21);
  GPIO5_GDIR |= GPIO_DR_DR(1 << 21);
  // Gate the SAI3 AUDIO device
  CCM_CCGR_SAI3 = 0x00;

  CCM_PLL_CTRL35 = 0x3333;
  CCM_ANALOG_AUDIO_PLL1_GEN_CTRL = CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_BYPASS_MASK |
                                   CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_CLKE_MASK;
  CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0 = CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV(361) |
                                    CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV(3) |
                                    CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV(3);
  CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1 = CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM(17511);
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

  // Configure SAI3_CLK_ROOT to Audio PLL clock source and divide it down to 11.2896 MHz
  CCM_TARGET_ROOT(77) = CCM_TARGET_ROOT_MUX(0x01) | CCM_TARGET_ROOT_POST_PODF(postDivSelect - 1) |
                            CCM_TARGET_ROOT_PRE_PODF(preDivSelect - 1);

  // Set SAI3 MSEL2 , MSEL3 clock source to SAI3_CLK_ROOT
  IOMUXC_GPR_GPR7 = IOMUXC_GPR_GPR7_GPR_SAI3_SEL2(2) | IOMUXC_GPR_GPR7_GPR_SAI3_SEL3(2);

  CCM_TARGET_ROOT_SET(77) = CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_TARGET_ROOT_SET(34) = CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_CCGR_SAI3 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
}


/**
  Initialize MIPI CSI block and perform required pin-muxing.
**/
VOID CameraInit(VOID)
{
  /* Disable CCGR */
  CCM_CCGR_SAI7 = 0x00;
  
  // CLK_CLKO2 = 24 MHz
  CCM_TARGET_ROOT_IPP_DO_CLKO1 = CCM_TARGET_ROOT_MUX(0x0) | CCM_TARGET_ROOT_POST_PODF(0x0) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  
  // MIPI_CSI1_CORE = 333 MHz - max 333, Linux 333, qnx 333MHz. SYSTEM_PLL2_CLK (1000MHz) / 3
  CCM_TARGET_ROOT_MIPI_CSI1_CORE = CCM_TARGET_ROOT_MUX(0x4) | CCM_TARGET_ROOT_POST_PODF(0x1) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_TARGET_ROOT_MIPI_CSI1_PHY_REF = CCM_TARGET_ROOT_MUX(0x0) | CCM_TARGET_ROOT_POST_PODF(0x0) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_TARGET_ROOT_MIPI_CSI2_PHY_REF = CCM_TARGET_ROOT_MUX(0x0) | CCM_TARGET_ROOT_POST_PODF(0x0) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_TARGET_ROOT_MIPI_CSI2_ESC = CCM_TARGET_ROOT_MUX(0x0) | CCM_TARGET_ROOT_POST_PODF(0x0) | CCM_TARGET_ROOT_PRE_PODF(0x0) | CCM_TARGET_ROOT_ENABLE_MASK;
  MicroSecondDelay(50);
              
  DISPLAY_MIX_SFT_RSTN_CSR
  |= (DISPLAY_MIX_SFT_EN_BUS_BLK_CLK_RESETN_MASK |
        DISPLAY_MIX_SFT_EN_ISI_APB_CLK_RESETN_MASK |
        DISPLAY_MIX_SFT_EN_ISI_PROC_CLK_RESETN_MASK |
        DISPLAY_MIX_SFT_EN_MIPI_CSI_ACLK_RESETN_MASK |
        DISPLAY_MIX_SFT_EN_MIPI_CSI_PCLK_RESETN_MASK);

  /* De-assert ISI & CSI clocks */
  DISPLAY_MIX_CLK_EN_CSR
   |= (DISPLAY_MIX_CLK_EN_BUS_BLK_CLK_MASK |
              DISPLAY_MIX_CLK_EN_ISI_APB_CLK_MASK |
              DISPLAY_MIX_CLK_EN_ISI_PROC_CLK_MASK |
              DISPLAY_MIX_CLK_EN_MIPI_CSI_ACLK_MASK |
              DISPLAY_MIX_CLK_EN_MIPI_CSI_PCLK_MASK);

  /* De-assert GPR_MIPI_S_RESETN reset */
  DISPLAY_MIX_GPR_MIPI_RESET_DIV
      |= DISPLAY_MIX_GPR_MIPI_S_RESETN_MASK;
  
  
  CCM_CCGR_SAI7 = 0x03;
  CCM_CCGR_REG(CCM_BASE_PTR, 102) = 0x03;

  MicroSecondDelay(500); 
  
  /* Configure GPIO1_IO14 as CCM_CLKO1 to provided external clock source for external camera module */
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO14 = IOMUXC_SW_PAD_CTL_PAD_DSE(0x2) | IOMUXC_SW_PAD_CTL_PAD_FSEL(0x2); // 4x drive strength, fast slew
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO14 = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0x6); // alt 6 - CLKO1
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

  CCM_CCGR_I2C1 = 0x00;
  CCM_CCGR_I2C2 = 0x00;
  CCM_CCGR_I2C3 = 0x00;
  CCM_CCGR_I2C4 = 0x00;
  // I2C ClkSrc     = 24M_REF_CLK  (24 MHz), PreDiv = 1, PostDiv = 1, Fout = 24 MHz
  CCM_TARGET_ROOT_I2C1 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_TARGET_ROOT_I2C2 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_TARGET_ROOT_I2C3 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_TARGET_ROOT_I2C4 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;

  CCM_CCGR_I2C1 = 0x03;
  CCM_CCGR_I2C2 = 0x03;
  CCM_CCGR_I2C3 = 0x03;
  CCM_CCGR_I2C4 = 0x03;
}

/**
  Initialize USB modules on the SOC and perform required pin-muxing
**/
VOID USBInit (VOID)
{
  CCM_CCGR_USB                 = 0x00;
  // USB_BUS_CLK_ROOT ClkSrc     = SYSTEM_PLL2_DIV2  (500 MHz), PreDiv = 1, PostDiv = 1, Fout = 500 MHz
  CCM_TARGET_ROOT_USB_BUS      = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // USB_BUS_CLK_ROOT ClkSrc     = SYSTEM_PLL1_DIV8  (100 MHz), PreDiv = 1, PostDiv = 1, Fout = 100 MHz
  CCM_TARGET_ROOT_USB_CORE_REF = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  // USB_PHY_REF_CLK_ROOT ClkSrc = SYSTEM_PLL1_DIV8  (100 MHz), PreDiv = 1, PostDiv = 1, Fout = 100 MHz
  CCM_TARGET_ROOT_USB_PHY_REF  = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  CCM_CCGR_USB                 = 0x02;  // Domain 0 clocks needed when in RUN and WAIT
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
  IOMUXC_SW_PAD_CTL_PAD_ENET_RXC    = ENET_RX_PAD_CTRL;  // ENET1_RGMII_RXC    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD3    = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD3    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD2    = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD2    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD1    = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD1    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TD0    = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TD0    electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TX_CTL = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TX_CTL electrical settings
  IOMUXC_SW_PAD_CTL_PAD_ENET_TXC    = ENET_TX_PAD_CTRL;  // ENET1_RGMII_TXC    electrical settings
  // Configure ENET_nRST signal
  IOMUXC_SW_MUX_CTL_PAD_SAI2_RXC    = IOMUXC_MUX_ALT5;   // ENET_nRST -> PAD_SAI2_RXC[ALT5]=GPIO4_22
  IOMUXC_SW_PAD_CTL_PAD_SAI2_RXC    = IOMUXC_PAD_DSE_R0_DIV_1 | IOMUXC_PAD_SRE_SLOW;
  GPIO4_DR &= ~(1 << 22);                                // Set ENET_nRST = 0
  GPIO4_GDIR |= (1 << 22);                               // Set direction to output
  MicroSecondDelay(500);
  GPIO4_DR |= (1 << 22);                                 // Set ENET_nRST = 1
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
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01 = (IOMUXC_SW_PAD_CTL_PAD_DSE(4) | IOMUXC_SW_PAD_CTL_PAD_FSEL(0));
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
  IOMUXC_SW_PAD_CTL_PAD_NAND_WE_B   = 0x196; // USDHC3_CLK
  IOMUXC_SW_PAD_CTL_PAD_NAND_WP_B   = 0x1d6; // USDHC3_CMD
  IOMUXC_SW_PAD_CTL_PAD_NAND_DATA04 = 0x1d6; // USDHC3_DATA0
  IOMUXC_SW_PAD_CTL_PAD_NAND_DATA05 = 0x1d6; // USDHC3_DATA1
  IOMUXC_SW_PAD_CTL_PAD_NAND_DATA06 = 0x1d6; // USDHC3_DATA2
  IOMUXC_SW_PAD_CTL_PAD_NAND_DATA07 = 0x1d6; // USDHC3_DATA3
  IOMUXC_SW_PAD_CTL_PAD_NAND_RE_B   = 0x1d6; // USDHC3_DATA4
  IOMUXC_SW_PAD_CTL_PAD_NAND_CE2_B  = 0x1d6; // USDHC3_DATA5
  IOMUXC_SW_PAD_CTL_PAD_NAND_CE3_B  = 0x1d6; // USDHC3_DATA6
  IOMUXC_SW_PAD_CTL_PAD_NAND_CLE    = 0x1d6; // USDHC3_DATA7
  IOMUXC_SW_PAD_CTL_PAD_NAND_CE1_B  = 0x196; // USDHC3_STROBE
}

/**
  Initialize GPU block.
**/
static UINT8 overdriveSuccess = 0;

VOID GpuInit()
{
  UINT32 val;

  val = GPC_PU_PWRHSK;
  val &= ~(GPC_PU_PWRHSK_GPC_GPUPMIX2NOC_PWRDNREQN_MASK);
  GPC_PU_PWRHSK = val;
  while (GPC_PU_PWRHSK & GPC_PU_PWRHSK_GPC_GPUMIX2NOC_PWRDNACKN_MASK);

  /* Power down request to ADB */
  val = GPC_PU_PWRHSK;
  val &= ~(GPC_PU_PWRHSK_GPC_NOC2GPUPMIX_PWRDNREQN_MASK);
  GPC_PU_PWRHSK = val;
  while (GPC_PU_PWRHSK & GPC_PU_PWRHSK_GPC_NOC2GPUMIX_PWRDNACKN_MASK);

  /* Disable GPU clock root */
  CCM_CCGR_GPU = 0x00;
  CCM_CCGR_GPU3D = 0x00;


  val = GPC_PU_PGC_SW_PDN_REQ;
  val |= GPC_PU_PGC_SW_PDN_REQ_GPUMIX_SW_PDN_REQ_MASK;
  GPC_PU_PGC_SW_PDN_REQ = val;

  if (overdriveSuccess) {
    INT32 count;

    CCM_PLL_CTRL13 = 0x3333; //3h=11b Domain clocks needed all the time - for all domains
    /* Setup GPU PLL to 1200MHz */
    CCM_ANALOG_GPU_PLL_GEN_CTRL = CCM_ANALOG_GPU_PLL_GEN_CTRL_PLL_BYPASS_MASK |
                                  CCM_ANALOG_GPU_PLL_GEN_CTRL_PLL_CLKE_MASK;
    CCM_ANALOG_GPU_PLL_FDIV_CTL0 = CCM_ANALOG_GPU_PLL_FDIV_CTL0_PLL_MAIN_DIV(300) |
                                   CCM_ANALOG_GPU_PLL_FDIV_CTL0_PLL_PRE_DIV(3) |
                                   CCM_ANALOG_GPU_PLL_FDIV_CTL0_PLL_POST_DIV(1);
    MicroSecondDelay (4);
    CCM_ANALOG_GPU_PLL_GEN_CTRL |=  CCM_ANALOG_GPU_PLL_GEN_CTRL_PLL_RST_MASK;
    for (count = 0 ; count < 100; ++count) {
      if (CCM_ANALOG_GPU_PLL_GEN_CTRL & CCM_ANALOG_GPU_PLL_GEN_CTRL_PLL_LOCK_MASK) {
        break;
      }
      MicroSecondDelay (10);
    }
    if (!(CCM_ANALOG_GPU_PLL_GEN_CTRL & CCM_ANALOG_GPU_PLL_GEN_CTRL_PLL_LOCK_MASK)) {
      DebugPrint (0xFFFFFFFF, "Time out waiting for PLL to lock\n");
    }
    CCM_ANALOG_GPU_PLL_GEN_CTRL &= ~(CCM_ANALOG_GPU_PLL_GEN_CTRL_PLL_BYPASS_MASK);

    /* Configure GPU_CORE input clock to 600MHz (SYSTEM_GPU_PLL / 2) */
    CCM_TARGET_ROOT_GPU_CORE = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(1) | CCM_TARGET_ROOT_ENABLE_MASK;
    /* Configure GPU_SHADER input clock to 600MHz (SYSTEM_GPU_PLL / 2) */
    CCM_TARGET_ROOT_GPU_SHADER = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(1) | CCM_TARGET_ROOT_ENABLE_MASK;
  } else { //No overdrive
    /* Configure GPU_CORE input clock to 400MHz (SYSTEM_PLL1_CLK / 2) */
    CCM_TARGET_ROOT_GPU_CORE = CCM_TARGET_ROOT_MUX(2) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(1) | CCM_TARGET_ROOT_ENABLE_MASK;
    /* Configure GPU_SHADER input clock to 400MHz (SYSTEM_PLL1_CLK / 2) */
    CCM_TARGET_ROOT_GPU_SHADER = CCM_TARGET_ROOT_MUX(2) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(1) | CCM_TARGET_ROOT_ENABLE_MASK;
  }

  /* Configure GPU_AXI input clock to 800MHz (SYSTEM_PLL1) */
  CCM_TARGET_ROOT_GPU_AXI = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
  /* Configure GPU_AHB input clock to 400MHz (SYSTEM_PLL1/2) */
  CCM_TARGET_ROOT_GPU_AHB = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_POST_PODF(1) | CCM_TARGET_ROOT_ENABLE_MASK;

  val = GPC_PU_PGC_SW_PUP_REQ;
  val |= GPC_PU_PGC_SW_PUP_REQ_GPUMIX_SW_PUP_REQ_MASK;
  GPC_PU_PGC_SW_PUP_REQ = val;

  /* Enable GPU clock root */
  CCM_CCGR_GPU = 0x02;
  CCM_CCGR_GPU3D = 0x02;

  /* SW reset assert */
  val = SRC_GPU_RCR;
  val |= 0x01U;
  SRC_GPU_RCR = val;
  /* SW reset release */
  val &= ~(0x01U);
  SRC_GPU_RCR = val;

  /* Enable GPU BUS power domain */
  val = GPC_PU_PWRHSK;
  val |= GPC_PU_PWRHSK_GPC_NOC2GPUPMIX_PWRDNREQN_MASK;
  GPC_PU_PWRHSK = val;
  /* Wait for NOC2GPUMIX ack */
  while (!(GPC_PU_PWRHSK & GPC_PU_PWRHSK_GPC_NOC2GPUMIX_PWRDNACKN_MASK));

  val = GPC_PU_PWRHSK;
  val |= GPC_PU_PWRHSK_GPC_GPUPMIX2NOC_PWRDNREQN_MASK;
  GPC_PU_PWRHSK = val;
  /* Wait for NOC2GPUMIX ack */
  while (!(GPC_PU_PWRHSK & GPC_PU_PWRHSK_GPC_GPUMIX2NOC_PWRDNACKN_MASK));
}

/**
  Initialize DIPLAY_MIX and perform required pin-muxing.
**/
VOID DisplayInit()
{
  UINT32 val;

  //
  // Enable MIPI-DSI power domain
  //
  val = GPC_PGC_PU0_CTRL; // MIPI_DSI_PHY
  GPC_PGC_PU0_CTRL &= ~(GPC_PGC_CTRL_PCR_MASK);
  GPC_PGC_PU0_CTRL = val;
  val = GPC_PU_PGC_SW_PUP_REQ;
  val |= GPC_PU_PGC_SW_PUP_REQ_MIPI_DSI_SW_PUP_REQ_MASK;
  GPC_PU_PGC_SW_PUP_REQ  = val;
  while (GPC_PU_PGC_SW_PUP_REQ & GPC_PU_PGC_SW_PUP_REQ_MIPI_DSI_SW_PUP_REQ_MASK) {}
  //
  // Enable DISPMIX power domain
  //
  val = GPC_PGC_PU10_CTRL; // DISPMIX
  val &= ~(GPC_PGC_CTRL_PCR_MASK);
  GPC_PGC_PU10_CTRL = val;
  val = GPC_PU_PGC_SW_PUP_REQ;
  val |= GPC_PU_PGC_SW_PUP_REQ_DISPMIX_SW_PUP_REQ_MASK;
  GPC_PU_PGC_SW_PUP_REQ  = val;
  while (GPC_PU_PGC_SW_PUP_REQ & GPC_PU_PGC_SW_PUP_REQ_DISPMIX_SW_PUP_REQ_MASK) {}

  //
  // De-assert LCDIF & MIPI DSI resets within DISPLAY_MIX block.
  //
  val = DISPLAY_MIX_SFT_RSTN_CSR;
  val |= (DISPLAY_MIX_SFT_EN_BUS_BLK_CLK_RESETN_MASK |
          DISPLAY_MIX_SFT_EN_LCDIF_APB_CLK_RESETN_MASK |
          DISPLAY_MIX_SFT_EN_LCDIF_PIXEL_CLK_RESETN_MASK |
          DISPLAY_MIX_SFT_EN_MIPI_DSI_CLKREF_RESETN_MASK |
          DISPLAY_MIX_SFT_EN_MIPI_DSI_PCLK_RESETN_MASK);
  DISPLAY_MIX_SFT_RSTN_CSR = val;
  //
  // Enable LCDIF & MIPI DSI clocks within DISPLAY_MIX block.
  //
  val = DISPLAY_MIX_CLK_EN_CSR;
  val |= (DISPLAY_MIX_CLK_EN_BUS_BLK_CLK_MASK |
          DISPLAY_MIX_CLK_EN_LCDIF_APB_CLK_MASK |
          DISPLAY_MIX_CLK_EN_LCDIF_PIXEL_CLK_MASK |
          DISPLAY_MIX_CLK_EN_MIPI_DSI_CLKREF_MASK |
          DISPLAY_MIX_CLK_EN_MIPI_DSI_PCLK_MASK);
  DISPLAY_MIX_CLK_EN_CSR = val;

  val = DISPLAY_MIX_GPR_MIPI_RESET_DIV;
  val |= DISPLAY_MIX_GPR_MIPI_S_RESETN_MASK | DISPLAY_MIX_GPR_MIPI_M_RESETN_MASK;
  DISPLAY_MIX_GPR_MIPI_RESET_DIV = val;

  //
  // DSI_EN signal
  //
  IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO08 = IOMUXC_MUX_ALT0;
  IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO08 = (IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_FSEL(0));
  GPIO1_DR   |= GPIO_DR_DR(1 << 8);
  GPIO1_GDIR |= GPIO_DR_DR(1 << 8);
}

/**
  Initialize ECSPI blocks and perform required pin-muxing.
**/
VOID SpiInit()
{
  // SPI2 set input clock to 24 MHz
  CCM_TARGET_ROOT_ECSPI2 = CCM_TARGET_ROOT_MUX(0) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_PRE_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK;
}

#if FixedPcdGet32(PcdPmicOverDriveEnable)
IMX_I2C_CONTEXT PmicI2cConfig =
{
  (uintptr_t)IMX_PMIC_I2C_BASE,         /* Base address of the I2C used for communication with PMIC */
  0,                                    /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  24000000,                             /* 2.4 MHz I2C ReferenceFreq */
  400000,                               /* 400 kHz required TargetFreq */
  0x4B,                                 /* PMIC SlaveAddress */
  100000,                               /* TimeoutInUs */
};

static EFI_STATUS WriteI2Creg(IMX_I2C_CONTEXT* i2cConfig, uint8_t reg, uint8_t* Data)
{
  EFI_STATUS Status;

  Status = iMXI2cWrite(i2cConfig, reg, Data, 1);
  if (Status != EFI_SUCCESS) {
    DEBUG((DEBUG_INFO, "PMIC EnableOverDrive I2C Write Error:   Slave=0x%02x Addr=0x%02x Data=0x%02x\n", i2cConfig->SlaveAddress, reg, *Data));
  } else {
    DEBUG((DEBUG_INFO, "PMIC EnableOverDrive I2C Write Success: Slave=0x%02x Addr=0x%02x Data=0x%02x\n", i2cConfig->SlaveAddress, reg, *Data));
  }
  return Status;
}

static EFI_STATUS ReadI2Creg(IMX_I2C_CONTEXT* i2cConfig, uint8_t reg, uint8_t* Data)
{
  EFI_STATUS Status;

  Status = iMXI2cRead(i2cConfig, reg, Data, 1);
  if (Status != EFI_SUCCESS) {
    DEBUG((DEBUG_INFO, "PMIC EnableOverDrive I2C Read Error:   Slave=0x%02x Addr=0x%02x Data=0x%02x\n", i2cConfig->SlaveAddress, reg, *Data));
  } else {
    DEBUG((DEBUG_INFO, "PMIC EnableOverDrive I2C Read Success: Slave=0x%02x Addr=0x%02x Data=0x%02x\n", i2cConfig->SlaveAddress, reg, *Data));
  }
  return Status;
}
/**
  Initialize MCUs clock speed and power.
**/
VOID McuClkSetOverdriveFrequency()
{
  // Select PLL2 1GHz to have stable clock, while ARM_PLL_FDIV is locked to 1400 MHz
  CCM_TARGET_ROOT_ARM_A53 = CCM_TARGET_ROOT_MUX(3) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK; /* ROOT_ARM_A53 = SYSTEM_PLL2_CLK  */

  /* Set the ARM PLL Fout = 1400 MHz, Fout = Fin/PreDiv*MainDiv/(2^PostDiv) = 24/6*700/(2^1) = 1400 MHz */
  CCM_ANALOG_ARM_PLL_FDIV_CTL0 = CCM_ANALOG_ARM_PLL_FDIV_CTL0_PLL_PRE_DIV(6) | CCM_ANALOG_ARM_PLL_FDIV_CTL0_PLL_MAIN_DIV(700) | CCM_ANALOG_ARM_PLL_FDIV_CTL0_PLL_POST_DIV(1);
  while ((CCM_ANALOG_ARM_PLL_GEN_CTRL & CCM_ANALOG_ARM_PLL_GEN_CTRL_PLL_LOCK_MASK) == 0) {};

  // Select ARM_PLL_CLK
  CCM_TARGET_ROOT_ARM_A53 = CCM_TARGET_ROOT_MUX(1) | CCM_TARGET_ROOT_POST_PODF(0) | CCM_TARGET_ROOT_ENABLE_MASK; /* ROOT_ARM_A53 = ARM_PLL_CLK  */

  overdriveSuccess = 1; //This flag indicates to GPU to go to overdrive mode too
  DebugPrint(0xffffffff, "EnableOverDrive Success\n");
}

VOID McuClkPwrInitBD718XX()
{
  uint8_t Data;

  // PMIC register addresses
  uint8_t bd718xxBuck1VoltRun      = 0x0d;
  uint8_t bd718xxBuck2VoltRun      = 0x10;
  uint8_t bd718xxReglock           = 0x2f;
  uint8_t buckRegs[2] = { bd718xxBuck1VoltRun, bd718xxBuck2VoltRun };

  /* unlock the PMIC regs */
  Data = 0x1;
  WriteI2Creg(&PmicI2cConfig, bd718xxReglock, &Data);

  for (int i = 0; i < sizeof(buckRegs)/sizeof(uint8_t); i++) {
    Data = 0x19;  // 0x19 = 0.95 V, see BUCK1/2_VOLT_RUN in Nano DDR4 PMIC BD71837A Datasheet

    uint8_t reg = buckRegs[i];
    WriteI2Creg(&PmicI2cConfig, reg, &Data);

    Data = 0x0; //To verify the read-back
    ReadI2Creg(&PmicI2cConfig, reg, &Data);
    if (Data != 0x19) {
      DEBUG((DEBUG_ERROR, "PMIC EnableOverDrive Readback not matching Error:   Slave=0x%02x Addr=0x%02x Data=0x%02x\n", PmicI2cConfig.SlaveAddress, reg, Data));
      DebugPrint(0xffffffff, "EnableOverDrive with PMIC BD718XX failed\n");
      return; //Skip CPU and GPU frequency increase (overdrive)
    }
  }

  /* lock the PMIC regs */
  Data = 0x11;
  WriteI2Creg(&PmicI2cConfig, bd718xxReglock, &Data);

  McuClkSetOverdriveFrequency();
}

IMX_I2C_CONTEXT PmicPCA9450_I2cConfig =
{
  (uintptr_t)IMX_PMIC_I2C_BASE,         /* Base address of the I2C used for communication with PMIC */
  0,                                    /* iMX I2C Controller SlaveAddress - not used, I2C interface is used in master mode only */
  24000000,                             /* 2.4 MHz I2C ReferenceFreq */
  400000,                               /* 400 kHz required TargetFreq */
  0x25,                                 /* PMIC SlaveAddress */
  100000,                               /* TimeoutInUs */
};

VOID McuClkPwrInitPCA9450()
{
  DEBUG((DEBUG_INFO, "McuClkPwrInit PMIC_PCA9450\n"));

  #define PCA9450_BUCK123_DVS 0x0C

  uint8_t Data;
  EFI_STATUS Status;

  Data = 0x0; //To verify the read-back
  Status = ReadI2Creg(&PmicPCA9450_I2cConfig, PCA9450_BUCK123_DVS, &Data);
  if (Status != EFI_SUCCESS) {
      DEBUG((DEBUG_ERROR, "PMIC PCA9450 read failed. EnableOverDrive skipped\n"));
      return;
  }

  /* Write BUCK123_DVS. Set BUCK1 and BUCK2 to 0.95V. BUCK3 is in dual phase with BUCK1 thus no need to set it. */
  uint8_t DvsValue = (1 << 7) | (1 << 5) | (3 << 3) | (3);
  Data = DvsValue;
  WriteI2Creg(&PmicPCA9450_I2cConfig, PCA9450_BUCK123_DVS, &Data);

  Data = 0x0; //To verify the read-back
  ReadI2Creg(&PmicPCA9450_I2cConfig, PCA9450_BUCK123_DVS, &Data);
  if (Data != DvsValue) {
    DEBUG((DEBUG_ERROR, "PMIC EnableOverDrive Readback not matching Error:   Slave=0x%02x Addr=0x%02x Data=0x%02x\n", PmicPCA9450_I2cConfig.SlaveAddress, PCA9450_BUCK123_DVS, Data));
    DebugPrint(0xffffffff, "EnableOverDrive with PMIC PCA9450 failed\n");
    return; //Skip CPU and GPU frequency increase (overdrive)
  }

  McuClkSetOverdriveFrequency();
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
  GpioInit();
  USBInit();
  EnetInit();
  AudioInit();
  I2cInit();

#if FixedPcdGet32(PcdPmicOverDriveEnable)
  McuClkPwrInitPCA9450();   // For the newer LPDDR Nano boards PMIC = PCA9450
  //McuClkPwrInitBD718XX(); // For the older DDR4 boards PMIC = BD718XX
#endif

  PwmInit();
  UsdhcInit();
  DisplayInit();
  GpuInit();
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
