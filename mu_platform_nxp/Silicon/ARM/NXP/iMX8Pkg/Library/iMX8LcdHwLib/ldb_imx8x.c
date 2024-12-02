/*
* Copyright 2023-2024 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <Base.h>
#include <Uefi/UefiBaseType.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <iMXDisplay.h>
#include "iMX8.h"
#include "svc/types.h"
#include "svc/pm/pm_api.h"
#include "svc/misc/misc_api.h"

#include "ldb_imx8x.h"
#include "mipi_dsi_imx8x.h"
#include "mipi_dsi_northwest_regs.h"

#define FLAG_COMBO 1

#define LDB_PHY_OFFSET MIPI_CSR_OFFSET
#define MIPI_PHY_OFFSET MIPI_DSI_OFFSET


/* Level of debug messages, error reports are not included */
#define LDBIMX8X_DEBUG_LEVEL DEBUG_INFO

EFI_STATUS Imx8xLdbSocSetup(imxDisplayInterfaceType displayInterface, UINT32 PixelClk)
{
  UINT32 err;
  sc_rsrc_t LvdsResource, MipiResource;
  UINT32 flag = FLAG_COMBO;
  UINT32 lvds_id = ((displayInterface == imxMipiDsi1) || (displayInterface == imxLvds1)) ? 1 : 0;

  DEBUG((LDBIMX8X_DEBUG_LEVEL, "mx8xLdbSocSetup: lvds_id = %d\n", lvds_id));

  if (lvds_id == 0) {
    LvdsResource = SC_R_LVDS_0;
    MipiResource = SC_R_MIPI_0;
  } else {
    LvdsResource = SC_R_LVDS_1;
    MipiResource = SC_R_MIPI_1;
  }

  err = sc_pm_set_clock_rate(SC_IPC_HDL, LvdsResource, SC_PM_CLK_BYPASS, &PixelClk);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d set rate SC_PM_CLK_BYPASS failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_set_clock_parent(SC_IPC_HDL, LvdsResource, SC_PM_CLK_PER, SC_PM_PARENT_BYPS);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d set parent SC_PM_CLK_PER failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_set_clock_parent(SC_IPC_HDL, LvdsResource, SC_PM_CLK_PHY, SC_PM_PARENT_BYPS);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d set parent SC_PM_CLK_PHY failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_set_clock_rate(SC_IPC_HDL, LvdsResource, SC_PM_CLK_PER, &PixelClk);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d set rate SC_PM_CLK_PER failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_set_clock_rate(SC_IPC_HDL, LvdsResource, SC_PM_CLK_PHY, &PixelClk);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d set rate SC_PM_CLK_PHY failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  if (flag & FLAG_COMBO) {
    /* For QXP, connect LVDS0 to pixel link 0, lVDS1 to pixel link 1 from DC */
    /* LVDS mode */
    err = sc_misc_set_control(SC_IPC_HDL, MipiResource, SC_C_MODE, 1);
    if (err) {
      DEBUG((DEBUG_ERROR, "LVDS%d sc_misc_set_control SC_C_MODE failed! (error = %d)\n", lvds_id, err));
      return EFI_DEVICE_ERROR;
    }

    /* LVDS mode Single channel */
    err = sc_misc_set_control(SC_IPC_HDL, MipiResource, SC_C_DUAL_MODE, 0);
    if (err) {
      DEBUG((DEBUG_ERROR, "LVDS%d sc_misc_set_control SC_C_DUAL_MODE failed! (error = %d)\n", lvds_id, err));
      return EFI_DEVICE_ERROR;
    }

    err = sc_misc_set_control(SC_IPC_HDL, MipiResource, SC_C_PXL_LINK_SEL, 0);
    if (err) {
      DEBUG((DEBUG_ERROR, "LVDS%d sc_misc_set_control SC_C_PXL_LINK_SEL failed! (error = %d)\n", lvds_id, err));
      return EFI_DEVICE_ERROR;
    }
  }

  err = sc_pm_clock_enable(SC_IPC_HDL, LvdsResource, SC_PM_CLK_BYPASS, true, false);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d enable clock SC_PM_CLK_BYPASS failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_clock_enable(SC_IPC_HDL, LvdsResource, SC_PM_CLK_PER, true, false);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d enable clock SC_PM_CLK_PER failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_clock_enable(SC_IPC_HDL, LvdsResource, SC_PM_CLK_PHY, true, false);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d enable clock SC_PM_CLK_PHY failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

VOID Imx8xLdbConfigure(imxDisplayInterfaceType displayInterface)
{
  UINT32 mode, phy_setting;
  UINT32 flag = FLAG_COMBO;
  UINT32 gpr = ((displayInterface == imxMipiDsi1) || (displayInterface == imxLvds1)) 
               ? DISP_INTERFACE1_BASE : DISP_INTERFACE0_BASE;

  DEBUG((LDBIMX8X_DEBUG_LEVEL, "Imx8xLdbConfigure: gpr = 0x%x\n", gpr));

  if (flag & FLAG_COMBO) {
    mode = IMX_LVDS_SET_FIELD(LVDS_CTRL_CH0_MODE, LVDS_CTRL_CH0_MODE__DI0) |
           IMX_LVDS_SET_FIELD(LVDS_CTRL_CH0_DATA_WIDTH, LVDS_CTRL_CH0_DATA_WIDTH__24BIT) |
           IMX_LVDS_SET_FIELD(LVDS_CTRL_CH0_BIT_MAP, LVDS_CTRL_CH0_BIT_MAP__SWWG);

    phy_setting = 0x4 << 5 | 0x4 << 2 | 1 << 1 | 0x1;
    MmioWrite32(gpr + LDB_PHY_OFFSET + LVDS_PHY_CTRL_REG, phy_setting);
    MmioWrite32(gpr + LDB_PHY_OFFSET + LVDS_CTRL, mode);
    MmioWrite32(gpr + LDB_PHY_OFFSET + MIPIv2_CSR_TX_ULPS, 0);
    MmioWrite32(gpr + LDB_PHY_OFFSET + MIPIv2_CSR_PXL2DPI, MIPI_CSR_PXL2DPI_24_BIT);

    /* Power up PLL in MIPI DSI PHY */
    MmioWrite32(gpr + MIPI_PHY_OFFSET + DPHY_PD_PLL, 0);
    MmioWrite32(gpr + MIPI_PHY_OFFSET + DPHY_PD_DPHY, 0);
  } else {
    mode = IMX_LVDS_SET_FIELD(LVDS_CTRL_CH0_MODE, LVDS_CTRL_CH0_MODE__DI0) |
           IMX_LVDS_SET_FIELD(LVDS_CTRL_CH0_DATA_WIDTH, LVDS_CTRL_CH0_DATA_WIDTH__24BIT) |
           IMX_LVDS_SET_FIELD(LVDS_CTRL_CH0_BIT_MAP, LVDS_CTRL_CH0_BIT_MAP__SWWG) |
           IMX_LVDS_SET_FIELD(LVDS_CTRL_CH0_10BIT_ENABLE, LVDS_CTRL_CH0_10BIT_ENABLE__10BIT) |
           IMX_LVDS_SET_FIELD(LVDS_CTRL_DI0_DATA_WIDTH, LVDS_CTRL_DI0_DATA_WIDTH__USE_30BIT);

    MmioWrite32(gpr + LDB_PHY_OFFSET + LVDS_CTRL, mode);

    phy_setting = LVDS_PHY_CTRL_RFB_MASK |
                  LVDS_PHY_CTRL_CH0_EN_MASK |
                  (0 << LVDS_PHY_CTRL_M_SHIFT) |
                  (0x04 << LVDS_PHY_CTRL_CCM_SHIFT) |
                  (0x04 << LVDS_PHY_CTRL_CA_SHIFT);
    MmioWrite32(gpr + LDB_PHY_OFFSET + LVDS_PHY_CTRL_REG, phy_setting);
  }
}

EFI_STATUS Imx8xLdbStop(imxDisplayInterfaceType displayInterface)
{
  UINT32 err;
  sc_rsrc_t LvdsResource;
  UINT32 flag = FLAG_COMBO;
  UINT32 lvds_id = ((displayInterface == imxMipiDsi1) || (displayInterface == imxLvds1)) ? 1 : 0;
  UINT32 gpr = lvds_id ? DISP_INTERFACE1_BASE : DISP_INTERFACE0_BASE;

  DEBUG((LDBIMX8X_DEBUG_LEVEL, "Imx8xLdbStop: gpr = 0x%x lvds_id = %d\n", gpr, lvds_id));
  
  if (flag & FLAG_COMBO) {
    MmioWrite32(gpr + LDB_PHY_OFFSET + LVDS_PHY_CTRL_REG, 0);
    MmioWrite32(gpr + LDB_PHY_OFFSET + LVDS_CTRL, LVDS_CTRL_CH0_MODE__DISABLED);

    /* Power down MIPI DSI PHY and PLL */
    MmioWrite32(gpr + MIPI_PHY_OFFSET + DPHY_PD_DPHY, 1);
    MmioWrite32(gpr + MIPI_PHY_OFFSET + DPHY_PD_PLL, 1);
  }

  if (lvds_id == 0) {
    LvdsResource = SC_R_LVDS_0;
  } else {
    LvdsResource = SC_R_LVDS_1;
  }

  err = sc_pm_clock_enable(SC_IPC_HDL, LvdsResource, SC_PM_CLK_PHY, false, false);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d enable clock SC_PM_CLK_PHY failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_clock_enable(SC_IPC_HDL, LvdsResource, SC_PM_CLK_BYPASS, false, false);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d enable clock SC_PM_CLK_BYPASS failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  err = sc_pm_clock_enable(SC_IPC_HDL, LvdsResource, SC_PM_CLK_PER, false, false);
  if (err) {
    DEBUG((DEBUG_ERROR, "LVDS%d enable clock SC_PM_CLK_PER failed! (error = %d)\n", lvds_id, err));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

