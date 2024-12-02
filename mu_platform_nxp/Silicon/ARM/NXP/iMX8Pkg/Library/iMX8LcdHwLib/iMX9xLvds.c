/*
 * Copyright 2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Uefi/UefiBaseType.h>
#include <iMXDisplay.h>

#include "iMX9xLvds.h"

#define LDB_CTRL                 0x20
#define CH0_ENABLE               (1 << 0)
#define CH0_DATA_WIDTH           (1 << 5)
#define CH0_BIT_MAPPING          (1 << 6)
#define CH0_VSYNC_POLARITY       (1 << 9)
#define CH0_FIFO_RESET           (1 << 11)
#define ASYNC_FIFO_ENABLE        (1 << 24)
#define ASYNC_FIFO_THRESHOLD(x)  (((x) & 0x7) << 25)

#define LVDS_CTRL                0x24
#define CH_EN(id)                (1 << id)
#define ENABLE_LVDS              (1 << 1)
#define BG_EN                    (1 << 2)
#define HS_EN                    (1 << 3)
#define PRE_EMPH_EN              (1 << 4)
#define PRE_EMPH_ADJ(x)          (((x) & 0x7) << 5)
#define CM_ADJ(x)                (((x) & 0x7) << 8)
#define CC_ADJ(x)                (((x) & 0x7) << 11)
#define SLEW_ADJ(x)              (((x) & 0x7) << 14)
#define VBG_ADJ(x)               (((x) & 0x7) << 17)
#define SPARE_IN(x)              (((x) & 0x7) << 25)

STATIC CONST UINTN BasePtrs[LVDS_MAX_DEV] = LVDS_BASE_PTRS;

EFI_STATUS LdbEnable(IN INTN Ldb, IN CONST IMX_DISPLAY_TIMING *Timing)
{
  if (Ldb < 0 || Ldb >= LVDS_MAX_DEV) {
    return EFI_DEVICE_ERROR;
  }

  (VOID)Timing;

  /*
  * Leave default negative polarity, SPWG mapping,
  * set 24bit data width, LDB data always from source 0.
  */
  MmioWrite32(BasePtrs[Ldb] + LDB_CTRL, CH0_ENABLE | CH0_DATA_WIDTH);

  return EFI_SUCCESS;
}

EFI_STATUS LvdsPhyInit(IN INTN Lvds)
{
  if (Lvds < 0 || Lvds >= LVDS_MAX_DEV) {
    return EFI_DEVICE_ERROR;
  }

  MmioWrite32(BasePtrs[Lvds] + LVDS_CTRL,
    CC_ADJ(0x2) | PRE_EMPH_EN | PRE_EMPH_ADJ(0x3));

  return EFI_SUCCESS;
}

EFI_STATUS LvdsPhyPowerOn(IN INTN Lvds, IN INTN ChId, IN BOOLEAN HasEnable)
{
  if (Lvds < 0 || Lvds >= LVDS_MAX_DEV) {
    return EFI_DEVICE_ERROR;
  }

  UINT32 LvdsCtrl =  MmioRead32(BasePtrs[Lvds] + LVDS_CTRL);
  BOOLEAN BgEn = !!(LvdsCtrl & BG_EN);
  LvdsCtrl |= BG_EN;
  if (HasEnable) {
    LvdsCtrl &= ~ENABLE_LVDS;
  }
  MmioWrite32(BasePtrs[Lvds] + LVDS_CTRL, LvdsCtrl);

  /* Wait 15us to make sure the bandgap is stable. */
  if (!BgEn) {
    MicroSecondDelay(15U);
  }

  LvdsCtrl =  MmioRead32(BasePtrs[Lvds] + LVDS_CTRL);
  LvdsCtrl |= CH_EN(ChId);
  MmioWrite32(BasePtrs[Lvds] + LVDS_CTRL, LvdsCtrl);

  /* Wait 5us to ensure the phy is stable. */
  MicroSecondDelay(5U);

  return EFI_SUCCESS;
}

EFI_STATUS LvdsPhyPowerOff(IN INTN Lvds, IN INTN ChId, IN BOOLEAN HasEnable)
{
  if (Lvds < 0 || Lvds >= LVDS_MAX_DEV) {
    return EFI_DEVICE_ERROR;
  }

  UINT32 LvdsCtrl =  MmioRead32(BasePtrs[Lvds] + LVDS_CTRL);
  LvdsCtrl &= ~BG_EN;
  MmioWrite32(BasePtrs[Lvds] + LVDS_CTRL, LvdsCtrl);

  LvdsCtrl =  MmioRead32(BasePtrs[Lvds] + LVDS_CTRL);
  LvdsCtrl &= ~CH_EN(ChId);
  if (HasEnable) {
    LvdsCtrl |= ENABLE_LVDS;
  }
  MmioWrite32(BasePtrs[Lvds] + LVDS_CTRL, LvdsCtrl);

  return EFI_SUCCESS;
}

