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
#include <Library/TimerLib.h>
#include <Uefi/UefiBaseType.h>

#include "iMX93MipiDphy.h"
#include "MipiCommon.h"

#ifdef IMX_MEDIAMIX_DEBUG
  #define DPHY_DEBUG_LEVEL DEBUG_ERROR
#else
  #define DPHY_DEBUG_LEVEL DEBUG_INFO
#endif
#define IMX_CLK_FREQ_24M        24000000U

struct DSI_PHY_VCO_PROP {
  UINTN MaxFout;
  UINT8 VcoCntl;
  UINT8 PropCntl;
};

struct DSI_PHY_HSFREQ_RANGE {
  UINTN MaxMbps;
  UINT8 HsfreqRange;
};

struct DSI_PHY_M_N {
  UINTN M;
  UINTN N;
};

/* Charge-pump programmability */
STATIC CONST struct DSI_PHY_VCO_PROP VcoPropMap[] = {
  {   55, 0x3f, 0x0d },
  {   82, 0x37, 0x0d },
  {  110, 0x2f, 0x0d },
  {  165, 0x27, 0x0d },
  {  220, 0x1f, 0x0d },
  {  330, 0x17, 0x0d },
  {  440, 0x0f, 0x0d },
  {  660, 0x07, 0x0d },
  { 1149, 0x03, 0x0d },
  { 1152, 0x01, 0x0d },
  { 1250, 0x01, 0x0e },
};

/* HS frequency range */
STATIC CONST struct DSI_PHY_HSFREQ_RANGE HsfreqRangeMap[] = {
  {   89, 0x00 },
  {   99, 0x10 },
  {  109, 0x20 },
  {  119, 0x30 },
  {  129, 0x01 },
  {  139, 0x11 },
  {  149, 0x21 },
  {  159, 0x31 },
  {  169, 0x02 },
  {  179, 0x12 },
  {  189, 0x22 },
  {  204, 0x32 },
  {  219, 0x03 },
  {  234, 0x13 },
  {  249, 0x23 },
  {  274, 0x33 },
  {  299, 0x04 },
  {  324, 0x14 },
  {  349, 0x25 },
  {  399, 0x35 },
  {  449, 0x05 },
  {  499, 0x16 },
  {  549, 0x26 },
  {  599, 0x37 },
  {  649, 0x07 },
  {  699, 0x18 },
  {  749, 0x28 },
  {  799, 0x39 },
  {  849, 0x09 },
  {  899, 0x19 },
  {  949, 0x29 },
  {  999, 0x3a },
  { 1049, 0x0a },
  { 1099, 0x1a },
  { 1149, 0x2a },
  { 1199, 0x3b },
  { 1249, 0x0b },
  { 1299, 0x1b },
  { 1349, 0x2b },
  { 1399, 0x3c },
  { 1449, 0x0c },
  { 1499, 0x1c },
  { 1549, 0x2c },
  { 1599, 0x3d },
  { 1649, 0x0d },
  { 1699, 0x1d },
  { 1749, 0x2e },
  { 1799, 0x3e },
  { 1849, 0x0e },
  { 1899, 0x1e },
  { 1949, 0x2f },
  { 1999, 0x3f },
  { 2049, 0x0f },
  { 2099, 0x40 },
  { 2149, 0x41 },
  { 2199, 0x42 },
  { 2249, 0x43 },
  { 2299, 0x44 },
  { 2349, 0x45 },
  { 2399, 0x46 },
  { 2449, 0x47 },
  { 2499, 0x48 },
  { 2500, 0x49 },
};

#define FCLKIN_RATIO_MAX_HZ 8000000
#define FCLKIN_RATIO_MIN_HZ 2000000

STATIC VOID Sort(UINTN *Arr1, UINTN *Arr2, INTN Cnt) {
  UINTN Tmp;
  for (INTN X = 0; X < Cnt - 1; X++) {
    for (INTN Y = 0; Y < Cnt - X - 1; Y++) {
      if (Arr1[Y] > Arr1[Y + 1]) {
        Tmp = Arr1[Y];
        Arr1[Y] = Arr1[Y + 1];
        Arr1[Y + 1] = Tmp;
        Tmp = Arr2[Y];
        Arr2[Y] = Arr2[Y + 1];
        Arr2[Y + 1] = Tmp;
      }
    }
  }
}

STATIC EFI_STATUS DsiPhyConfigGetMN(IMX_DISPLAY_TIMING* Timing,
  struct DSI_PHY_M_N *Result)
{
  ASSERT(Timing);
  ASSERT(Result);

  UINTN Bpp = 24;
  UINTN Lanes = 4;
  UINTN Pclk = GetPixelClockKHz(Timing);
  UINTN TargetRate = Pclk * (Bpp / Lanes);
  UINTN Fout = TargetRate / 2;

  UINTN Fdiv = 1;
  /* Limit Output Frequency */
  if ((1250000 >= Fout) && (Fout > 330000)) {
    Fdiv = 1;
  } else if ((330000 >= Fout) && (Fout > 165000)) {
    Fdiv = 2;
  } else  if ((165000 >= Fout) && (Fout > 82000)) {
    Fdiv = 4;
  } else  if ((82000 >= Fout) && (Fout >= 40000)) {
    Fdiv = 8;
  } else {
    DebugPrint(DEBUG_ERROR, "MIPI-PHY bound divider calculation error.\n");
    return EFI_DEVICE_ERROR;
  }

  UINTN ModList[16] = { 0 };
  UINTN DivList[16] = { 0 };
  UINT32 PhyFinHz = IMX_CLK_FREQ_24M; /* fixed on 93 */

  for (UINTN Div = 1; Div <= 16; Div++) {
    UINTN Mod = (Fout * Div * Fdiv) % (PhyFinHz / 1000);
    ModList[Div-1] = Mod;
    DivList[Div-1] = Div;
  }

  /* Sort the results based on modulo to find closest match */
  Sort(ModList, DivList, 16);

  for (INTN Idx = 0; Idx < 16; Idx++) {
    UINTN FinRatio = PhyFinHz / DivList[Idx];
    if (FinRatio >= FCLKIN_RATIO_MIN_HZ) {
      if (FinRatio <= FCLKIN_RATIO_MAX_HZ) {
        UINTN TmpM = (Fout * DivList[Idx] * Fdiv) / (PhyFinHz / 1000);
        if ((TmpM > 625) || (TmpM < 64)) {
          continue;
        }
        Result->M = TmpM;
        Result->N = DivList[Idx];
        DebugPrint(DPHY_DEBUG_LEVEL, "Selected: div %d mod %d\n",
          DivList[Idx], ModList[Idx]);
        break;
      }
    }
  }

  if (Result->N == 0) {
    DebugPrint(DEBUG_ERROR, "MPI-PHY m,n ratio calculation error for pclk=%d Fout=%d\n", Pclk, Fout);
    return EFI_DEVICE_ERROR;
  }
  DebugPrint(DPHY_DEBUG_LEVEL, "MIPI-PHY calculated m = %d n = %d Pclk = %d Fout = %d\n", Result->M, Result->N, Pclk, Fout);

  return EFI_SUCCESS;
}

STATIC VOID DsiPhyClrShadow()
{
  BLK_CTRL_MEDIAMIX_DSI = DSI_CLKSEL(DSI_CLKSEL_GEN);

  NanoSecondDelay(2000);
  BLK_CTRL_MEDIAMIX_DSI = DSI_CLKSEL(DSI_CLKSEL_GEN) | DSI_SHADOW_CLR_MASK;

  /* A minimum pulse of 5ns on shadow_clear signal */
  NanoSecondDelay(5000);
  BLK_CTRL_MEDIAMIX_DSI = DSI_CLKSEL(DSI_CLKSEL_GEN);
}

STATIC UINTN DsiPhyGetVco(IMX_DISPLAY_TIMING* Timing)
{
  ASSERT(Timing);

  UINTN Fout = MipiGetLaneRateMbps(Timing) / 2;

  for (UINTN Idx = 0; Idx < ARRAY_SIZE(VcoPropMap); Idx++) {
    if (Fout <= VcoPropMap[Idx].MaxFout) {
      return VcoPropMap[Idx].VcoCntl;
    }
  }

  return 0;
}

STATIC UINTN DsiPhyGetProp(IMX_DISPLAY_TIMING* Timing)
{
  ASSERT(Timing);
  UINTN Fout = MipiGetLaneRateMbps(Timing) / 2;

  for (UINTN Idx = 0; Idx < ARRAY_SIZE(VcoPropMap); Idx++) {
    if (Fout <= VcoPropMap[Idx].MaxFout) {
      return VcoPropMap[Idx].PropCntl;
    }
  }

  return 0;
}

STATIC UINTN DsiPhyGetHsfreqRange(IMX_DISPLAY_TIMING* Timing)
{
  ASSERT(Timing);
  UINTN Mbps = MipiGetLaneRateMbps(Timing);

  for (UINTN Idx = 0; Idx < ARRAY_SIZE(HsfreqRangeMap); Idx++) {
    if (Mbps <= HsfreqRangeMap[Idx].MaxMbps) {
      return HsfreqRangeMap[Idx].HsfreqRange;
    }
  }

  return 0;
}

VOID DsiPhyStop()
{
  BLK_CTRL_MEDIAMIX_DSI = 0;
}

EFI_STATUS DsiPhyConfigure(IMX_DISPLAY_TIMING* Timing)
{
  ASSERT(Timing);

  /* Clear DSI PHY registers */
  BLK_CTRL_MEDIAMIX_DSI = 0;
  BLK_CTRL_MEDIAMIX_DSI_W0 = 0;
  BLK_CTRL_MEDIAMIX_DSI_W1 = 0;

  struct DSI_PHY_M_N MNPair = { 0,0 };
  EFI_STATUS  Status = DsiPhyConfigGetMN(Timing, &MNPair);
  if (Status) {
    DebugPrint(DEBUG_ERROR, "Cannot find exact MIPI-PHY M/N ratio.\n");
    return Status;
  }

  DebugPrint(DPHY_DEBUG_LEVEL, "DSI MIPI-PHY m:%d n:%d\n", MNPair.M, MNPair.N);
  DsiPhyClrShadow();

  /* DSI */
  UINT32 PhyFinHz = IMX_CLK_FREQ_24M;  /* Fixed on iMX93 */
  UINT32 Val = DSI_CLKSEL(DSI_CLKSEL_GEN) |
  DSI_CFGCLKFREQRANGE(((PhyFinHz / 1000000) - 17) * 4) |
    DSI_HSFREQRANGE(DsiPhyGetHsfreqRange(Timing));
  BLK_CTRL_MEDIAMIX_DSI = Val;

  /* W0 */
  Val = DSI_W0_M(MNPair.M) |
    DSI_W0_N(MNPair.N) |
    DSI_W0_INT_CTRL(0) |
    DSI_W0_VCO_CTRL(DsiPhyGetVco(Timing)) |
    DSI_W0_PROP_CTRL(DsiPhyGetProp(Timing));
  BLK_CTRL_MEDIAMIX_DSI_W0 = Val;

  DebugPrint(DPHY_DEBUG_LEVEL, "DSI MIPI-PHY fdiv=%d\n", DsiPhyGetVco(Timing)>>4);

  /* W1 */
  Val = DSI_W1_GMP_CTRL(0x1) | DSI_W1_CPBIAS_CTRL(0x10);
  BLK_CTRL_MEDIAMIX_DSI_W1 = Val;

  /* PLL on - At least 10 refclk cycles are required before updatePLL */
  MicroSecondDelay(10U);

  Val = BLK_CTRL_MEDIAMIX_DSI | DSI_UPDATE_PLL_MASK;
  BLK_CTRL_MEDIAMIX_DSI = Val;

  /* The updatepll signal should be asserted for a minimum of 4 clkin cycles */
  MicroSecondDelay(10U);

  Val = BLK_CTRL_MEDIAMIX_DSI & ~DSI_UPDATE_PLL_MASK;
  BLK_CTRL_MEDIAMIX_DSI = Val;

#ifdef IMX_MEDIAMIX_DEBUG
  MediamixDumpReg();
#endif

  return EFI_SUCCESS;

}

