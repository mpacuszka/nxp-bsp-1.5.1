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
#ifndef MIPI_COMMON_H_
#define MIPI_COMMON_H_

#include <stdint.h>
#include <stdbool.h>

#include <iMXDisplay.h>
#include <Library/DebugLib.h>

#ifndef DIV_RD_UP
#define DIV_RD_UP(a,div)              (((a)+(div)-1)/(div))
#endif

#ifndef DIV_RD_CLOSEST
#define DIV_RD_CLOSEST(a, div)        (((a) + ((div) >> 1)) / div)
#endif

STATIC inline UINT32 GetPixelClockKHz(IMX_DISPLAY_TIMING* Timing)
{
  ASSERT(Timing);
  return Timing->PixelClock / 1000;
}

STATIC inline INTN MipiDsiBusMode2Bpp(IMX_PIXEL_FORMAT PixelFormat)
{
  UINT32 Bpp;

  switch(PixelFormat) {
    /* we support only 32 bit formats */
    case PIXEL_FORMAT_ARGB32:
    case PIXEL_FORMAT_BGRA32:
      Bpp = 24;
      break;
    default:
      DEBUG((DEBUG_WARN, "Unsupported format-0x%x\n", PixelFormat));
      Bpp = -1;
      break;
  }

  return Bpp;
}

STATIC inline INTN MipiGetLaneRateKbps(IMX_DISPLAY_TIMING* Timing)
{
  ASSERT(Timing);

  INTN Bpp = MipiDsiBusMode2Bpp(Timing->PixelFormat);
  if (Bpp < 0) {
    DEBUG((DEBUG_ERROR, "Failed to get bpp for fmt-0x%x\n",
      Timing->PixelFormat));
    return Bpp;
  }

  INTN Lanes = 4;
  return (Timing->PixelClock / 1000) * (Bpp / Lanes);
}

STATIC inline INTN MipiGetLaneRateMbps(IMX_DISPLAY_TIMING* Timing)
{
  ASSERT(Timing);
  return MipiGetLaneRateKbps(Timing) / 1000;
}

#endif

