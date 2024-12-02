/** @file

  Copyright (c) 2020, Linaro, Ltd. All rights reserved.
  Copyright 2023 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Uefi/UefiBaseType.h>
#include <iMXDisplay.h>
#include <iMXI2cLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/TimerLib.h>

#include "adv7535.h"
#include "Clk/iMX93Clk.h"
#include "Clk/Mediamix.h"
#include "DwMipiDsi.h"
#include "iMX8.h"
#include "iMX8LcdHwLib.h"
#include "iMX9xLvds.h"
#include "Lcdifv3.h"

/* Level of debug messages, error reports are not included */
#define LCDHWLIB_DEBUG_LEVEL DEBUG_INFO

/* Maxumum resolution for it6263 and adv7535 converters */
#define CONV_MAX_HACTIVE_1920        1920u
#define CONV_MAX_VACTIVE_1080        1080u
#define CONV_MAX_HACTIVE_1280        1280u
#define CONV_MAX_VACTIVE_720         720u

/* EDID macros */
#define BASIC_EDID_STRUCTURE_LENGTH       128

#define EDID_MANUFACTURER_ID_REG_OFFSET   8
#define EDID_MANUFACTURER_ID_REG_MASK     0xFFFF

#define EDID_VERSION_REG_OFFSET           18
#define EDID_REVISION_REG_OFFSET          19

#define EDID_NUM_OF_EXTENSIONS_REG_OFFSET 126

#define SHORT_VIDEO_BLOCK                 0x02

/* Preferred timing mode. if PcdDisplayReadEDID == TRUE, it is overwritten with edid data */
IMX_DISPLAY_TIMING PreferredTiming;

/* Predefined modes - one selected is copied to PreferredTiming in LcdDisplayDetect */
/* 1920x1080@60Hz */
CONST IMX_DISPLAY_TIMING PreferredTiming_1920x1080_60 = {
  .PixelClock = 148500000,
  .HActive = 1920,
  .HBlank = 280,
  .VActive = 1080,
  .VBlank = 45,
  .HSync = 44,
  .VSync = 5,
  .HSyncOffset = 88,
  .VSyncOffset = 4,
  .HImageSize = 527,
  .VImageSize = 296,
  .HBorder = 0,
  .VBorder = 0,
  .EdidFlags = 0,
  .Flags = 0,
  .PixelRepetition = 0,
  .Bpp = 24,
  .PixelFormat = PIXEL_FORMAT_ARGB32,
};

/* 1920x1200@60Hz - dual lvds panel */
CONST IMX_DISPLAY_TIMING PreferredTiming_1920x1200_60 = {
  .PixelClock = 156685000,
  .HActive = 1920,
  .HBlank = 230,
  .VActive = 1200,
  .VBlank = 15,
  .HSync = 40,
  .VSync = 5,
  .HSyncOffset = 90,
  .VSyncOffset = 5,
  .HImageSize = 527,
  .VImageSize = 296,
  .HBorder = 0,
  .VBorder = 0,
  .EdidFlags = 0,
  .Flags = 0,
  .PixelRepetition = 0,
  .Bpp = 24,
  .PixelFormat = PIXEL_FORMAT_ARGB32,
};

/* 1280x1024@60Hz */
CONST IMX_DISPLAY_TIMING PreferredTiming_1280x1024_60 = {
  .PixelClock = 108000000,
  .HActive = 1280,
  .HBlank = 408,
  .VActive = 1024,
  .VBlank = 42,
  .HSync = 112,
  .VSync = 3,
  .HSyncOffset = 48,
  .VSyncOffset = 1,
  .HImageSize = 527,
  .VImageSize = 296,
  .HBorder = 0,
  .VBorder = 0,
  .EdidFlags = 0,
  .Flags = 0,
  .PixelRepetition = 0,
  .Bpp = 24,
  .PixelFormat = PIXEL_FORMAT_ARGB32,
};

/* 1280x800@60 */
CONST IMX_DISPLAY_TIMING PreferredTiming_1280x800_60 = {
  .PixelClock = 72400000,
  .HActive = 1280,
  .HBlank = 160,
  .VActive = 800,
  .VBlank = 38,
  .HSync = 32,
  .VSync = 15,
  .HSyncOffset = 48,
  .VSyncOffset = 9,
  .HImageSize = 527,
  .VImageSize = 296,
  .HBorder = 0,
  .VBorder = 0,
  .EdidFlags = 0,
  .Flags = 0,
  .PixelRepetition = 0,
  .Bpp = 24,
  .PixelFormat = PIXEL_FORMAT_ARGB32,
};

/* 1280x720@60 */
CONST IMX_DISPLAY_TIMING PreferredTiming_1280x720_60 = {
  .PixelClock = 74250000,
  .HActive = 1280,
  .HBlank = 370,
  .VActive = 720,
  .VBlank = 30,
  .HSync = 40,
  .VSync = 5,
  .HSyncOffset = 110,
  .VSyncOffset = 5,
  .HImageSize = 527,
  .VImageSize = 296,
  .HBorder = 0,
  .VBorder = 0,
  .EdidFlags = 0,
  .Flags = 0,
  .PixelRepetition = 0,
  .Bpp = 24,
  .PixelFormat = PIXEL_FORMAT_ARGB32,
};

/* 1024x768@60 */
CONST IMX_DISPLAY_TIMING PreferredTiming_1024x768_60 = {
  .PixelClock = 65000000,
  .HActive = 1024,
  .HBlank = 320,
  .VActive = 768,
  .VBlank = 38,
  .HSync = 136,
  .VSync = 6,
  .HSyncOffset = 24,
  .VSyncOffset = 3,
  .HImageSize = 527,
  .VImageSize = 296,
  .HBorder = 0,
  .VBorder = 0,
  .EdidFlags = 0,
  .Flags = 0,
  .PixelRepetition = 0,
  .Bpp = 24,
  .PixelFormat = PIXEL_FORMAT_ARGB32,
};

/* 800x600@60 */
CONST IMX_DISPLAY_TIMING PreferredTiming_800x600_60 = {
  .PixelClock = 40000000,
  .HActive = 800,
  .HBlank = 256,
  .VActive = 600,
  .VBlank = 28,
  .HSync = 128,
  .VSync = 4,
  .HSyncOffset = 40,
  .VSyncOffset = 1,
  .HImageSize = 527,
  .VImageSize = 296,
  .HBorder = 0,
  .VBorder = 0,
  .EdidFlags = 0,
  .Flags = 0,
  .PixelRepetition = 0,
  .Bpp = 24,
  .PixelFormat = PIXEL_FORMAT_ARGB32,
};

/* Count of the read modes */
int VideoModesCnt = 0;

/* Get display interface type defined in *.dsc file */
imxDisplayInterfaceType DispInterface = FixedPcdGet32(PcdDisplayInterface);
/* Type of converter */
imxConverter Converter = transmitterUnknown;

#define CHECK_STATUS_RETURN_ERR(chkfunc, chkmessage) \
    { \
      EFI_STATUS chkstatus; \
      if ((chkstatus = (chkfunc)) != EFI_SUCCESS) { \
        DEBUG ((DEBUG_ERROR, "%s returned error %d\n", (chkmessage), chkstatus)); \
        return chkstatus; \
      } \
    }

/* ******************************* Low level functions ******************************* */
/**
  Initialize TargetTiming structure from predefined constant data
  @param  SourceTiming    Predefined constant data of particular mode
  @param  TargetTiming    Output timing structure
**/
STATIC VOID
LcdInitPreferredTiming (
  IN CONST IMX_DISPLAY_TIMING *SourceTiming,
  OUT IMX_DISPLAY_TIMING *TargetTiming
  )
{
  TargetTiming->PixelClock = SourceTiming->PixelClock;
  TargetTiming->HActive = SourceTiming->HActive;
  TargetTiming->HBlank = SourceTiming->HBlank;
  TargetTiming->VActive = SourceTiming->VActive;
  TargetTiming->VBlank = SourceTiming->VBlank;
  TargetTiming->HSync = SourceTiming->HSync;
  TargetTiming->VSync = SourceTiming->VSync;
  TargetTiming->HSyncOffset = SourceTiming->HSyncOffset;
  TargetTiming->VSyncOffset = SourceTiming->VSyncOffset;
  TargetTiming->HImageSize = SourceTiming->HImageSize;
  TargetTiming->VImageSize = SourceTiming->VImageSize;
  TargetTiming->HBorder = SourceTiming->HBorder;
  TargetTiming->VBorder = SourceTiming->VBorder;
  TargetTiming->EdidFlags = SourceTiming->EdidFlags;
  TargetTiming->Flags = SourceTiming->Flags;
  TargetTiming->PixelRepetition = SourceTiming->PixelRepetition;
  TargetTiming->Bpp = SourceTiming->Bpp;
  TargetTiming->PixelFormat = SourceTiming->PixelFormat;
}

/**
  Dump preferred timing mode read from EDID
  @param  DtdOffset                Display timing descriptor offset.
  @param  Timing                   Display timing mode.
**/
STATIC VOID
LcdDumpDisplayTiming (
  IN UINT32              DtdOffset,
  IN IMX_DISPLAY_TIMING *Timing
  )
{
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "*************************************************\n"));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "* DTD(0x%02X)\n",DtdOffset));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "*************************************************\n"));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->PixelClock =      %d\n",    Timing->PixelClock));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->HActive =         %d\n",    Timing->HActive));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->HBlank =          %d\n",    Timing->HBlank));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->VActive =         %d\n",    Timing->VActive));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->VBlank =          %d\n",    Timing->VBlank));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->HSync =           %d\n",    Timing->HSync));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->VSync =           %d\n",    Timing->VSync));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->HSyncOffset =     %d\n",    Timing->HSyncOffset));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->VSyncOffset =     %d\n",    Timing->VSyncOffset));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->HImageSize =      %d\n",    Timing->HImageSize));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->VImageSize =      %d\n",    Timing->VImageSize));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->HBorder =         %d\n",    Timing->HBorder));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->VBorder =         %d\n",    Timing->VBorder));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->EdidFlags =       0x%0X\n", Timing->EdidFlags));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->Flags =           0x%0X\n", Timing->Flags));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->PixelRepetition = %d\n",    Timing->PixelRepetition));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->Bpp =             %d\n",    Timing->Bpp));
  DEBUG ((LCDHWLIB_DEBUG_LEVEL, "Timing->PixelFormat =     %d\n",    Timing->PixelFormat));
}

/**
  Read display EDID.

  @param  Edid                   Buffer for EDID data.
  @retval EFI_SUCCESS            Platform implements display.
  @retval EFI_NOT_FOUND          Display not found on the platform.

**/
STATIC EFI_STATUS
LcdReadEdid (
  OUT UINT8 *Edid,
  IN UINT32 Offset,
  IN UINT32 Length
  )
{
  ASSERT(Edid);
  EFI_STATUS Status = EFI_SUCCESS;

  /* For now read only standard EDID structure , ignore EDID extensions */
  if(DispInterface == imxMipiDsi) {
    if(Converter == ADV7535) {
      Status = Adv7535ReadEdid(Edid, Offset, Length);
    }
  } else {
    DEBUG((DEBUG_ERROR, "Usupported display interface: %d\n",
      (INTN)DispInterface));
    Status = EFI_NOT_FOUND;
  }

  return Status;
}

/**
  Check converter resolution is supported by EDID data

  @param  DTDPtr                 Pointer to EDID Detailed Timing Descriptor

  @retval TRUE                   EDID resolution is lower or equal to Maximum
  @retval FALSE                  EDID resolution exceeds Maximum

**/
STATIC BOOLEAN
LcdConvSuppResolution(
  IN IMX_DETAILED_TIMING_DESCRIPTOR *DTDPtr
)
{
  UINT32 Hact = (DTDPtr->HActiveBlank & 0xF0);
  Hact = (Hact << 4) | DTDPtr->HActive;
  UINT32 Vact = (DTDPtr->VActiveBlank & 0xF0);
  Vact = (Vact << 4) | DTDPtr->VActive;

  UINT32 Hmax = 0;
  UINT32 Vmax = 0;
  switch (DispInterface) {
    case imxMipiDsi:
    case imxLvds0dual:
    case imxNativeHdmi:
      Hmax = CONV_MAX_HACTIVE_1920;
      Vmax = CONV_MAX_VACTIVE_1080;
      break;
    case imxLvds0:
    case imxLvds1:
      Hmax = CONV_MAX_HACTIVE_1280;
      Vmax = CONV_MAX_VACTIVE_720;
      break;
    default:
      DEBUG((DEBUG_ERROR, "Usupported display interface: %d\n", (INTN)DispInterface));
      return FALSE;
  }
  
  if ((Hact <= Hmax)  && (Vact <= Vmax)) {
    return TRUE;
  }

  return FALSE;
  
}

/* ******************************* High level functions ******************************* */
/**
  Check for presence of display

  @retval EFI_SUCCESS            Platform implements display.
  @retval EFI_NOT_FOUND          Display not found on the platform.

**/
EFI_STATUS
LcdDisplayDetect (
  VOID
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT8 *Edid;
  INTN EdidExtensions = 0;
  BOOLEAN ConvSetEdidAlways = TRUE;

  /* Try to autodetect connected converter regardless of selected DispInterface
  mipi-dsi with ADV7535 inteface has priority over LVDS Panel */
  if (Converter == transmitterUnknown) {
    do {
      /* Search for ADV7535 */
      Status = Adv7535Discover();
      if (Status == EFI_SUCCESS) {
        DEBUG((DEBUG_ERROR, "ADV7535 probe SUCCEDED. Mipi-dsi display interface selected.\n"));
        Converter = ADV7535;
        DispInterface = imxMipiDsi;
        LcdInitPreferredTiming (&PreferredTiming_1920x1080_60, &PreferredTiming);
        break;
      }
    } while(0);
  }

  /* Converter was not detected - select fixed default timing */
  if (Converter == transmitterUnknown) {
    if (DispInterface == imxMipiDsi) {
      VideoModesCnt++;
      LcdInitPreferredTiming (&PreferredTiming_1920x1080_60, &PreferredTiming);
      DEBUG((DEBUG_ERROR,
        "Mipi-dsi display interface. Default resolution used. %dx%d pclk=%d Hz\n", 
        PreferredTiming.HActive, PreferredTiming.VActive, PreferredTiming.PixelClock));
      return EFI_SUCCESS;
    } else if (DispInterface == imxLvds0) {
      VideoModesCnt++;
      LcdInitPreferredTiming (&PreferredTiming_1280x800_60, &PreferredTiming);
      DEBUG((DEBUG_ERROR,
        "Lvds0 display interface. Default resolution used. %dx%d pclk=%d Hz\n", 
        PreferredTiming.HActive, PreferredTiming.VActive, PreferredTiming.PixelClock));
      return EFI_SUCCESS;
    } else {
      DEBUG((DEBUG_ERROR, "Usupported display interface: %d\n",
        (INTN)DispInterface));
      return EFI_NOT_FOUND;
    }
  }

  if (FixedPcdGet32(PcdDisplayReadEDID) == TRUE) {
    /* Allocate memory for EDID structure */
    Edid = AllocatePool(BASIC_EDID_STRUCTURE_LENGTH + 1);

    /* Read EDID */
    Status = LcdReadEdid(Edid, 0, BASIC_EDID_STRUCTURE_LENGTH);
    if (EFI_ERROR(Status)) {
      DEBUG((DEBUG_ERROR, "Unable to read EDID\n"));
      goto ErrorEdid;
    }

    /* Ignore extensions */
    EdidExtensions = Edid[126];
    DEBUG((LCDHWLIB_DEBUG_LEVEL,
      "EDID Version: %d.%d\n",
      Edid[EDID_VERSION_REG_OFFSET],Edid[EDID_REVISION_REG_OFFSET]));
    DEBUG((LCDHWLIB_DEBUG_LEVEL, "EDID Num of Extensions: %d\n",
      EdidExtensions));

    /* Validate EDID data to */
    Status = ImxValidateEdidData(Edid);
    if (EFI_ERROR(Status)) {
      DEBUG((DEBUG_ERROR, "EDID data not valid\n"));
      goto ErrorEdid;
    }

    if (Converter == ADV7535) {
      if (FixedPcdGet32(PcdDisplayForceConverterMaxResolution) == TRUE) {
        ConvSetEdidAlways = FALSE;
      }
    }

    /* Read first DTD, which is the most preferred */
    for (INTN Idx = IMX_EDID_DTD_1_OFFSET; Idx <= IMX_EDID_DTD_1_OFFSET;
      Idx += IMX_EDID_DTD_SIZE) {
      /* Convert EDID data into internal format */
      if (ConvSetEdidAlways || 
          LcdConvSuppResolution((IMX_DETAILED_TIMING_DESCRIPTOR *)&Edid[Idx])) {
        Status = ImxConvertDTDToDisplayTiming(
          (IMX_DETAILED_TIMING_DESCRIPTOR *)&Edid[Idx],
          &PreferredTiming);
        if (Status != EFI_SUCCESS) {
          DEBUG((DEBUG_ERROR, "Conversion to display timing failed\n"));
          goto End;
        }
      }
      VideoModesCnt++;
      /* BPP is fixed to 24 (8 bits per color component) */
      PreferredTiming.Bpp = 24;
      PreferredTiming.PixelFormat = PIXEL_FORMAT_ARGB32;
    }
    DEBUG((DEBUG_ERROR, "Selected resolution %dx%d pclk=%d Hz\n",
      PreferredTiming.HActive, PreferredTiming.VActive, PreferredTiming.PixelClock));
End:
    FreePool(Edid);
  } else {
    VideoModesCnt++;
    DEBUG((DEBUG_ERROR, "Selected default resolution %dx%d pclk=%d Hz\n",
      PreferredTiming.HActive, PreferredTiming.VActive, PreferredTiming.PixelClock));
  }

  LcdDumpDisplayTiming(0, &PreferredTiming);

  return Status;

ErrorEdid:
  FreePool(Edid);
  VideoModesCnt++;
  DEBUG((DEBUG_ERROR, "Selected default resolution %dx%d pclk=%d Hz\n",
       PreferredTiming.HActive, PreferredTiming.VActive, PreferredTiming.PixelClock));
  LcdDumpDisplayTiming(0, &PreferredTiming);
  return EFI_SUCCESS;
}

/**
  Initialize display.

  @param  FrameBaseAddress       Address of the frame buffer.
  @retval EFI_SUCCESS            Display initialization success.
  @retval !(EFI_SUCCESS)         Display initialization failure.

**/
EFI_STATUS
LcdInitialize (
  EFI_PHYSICAL_ADDRESS  FrameBaseAddress
  )
{
  if (DispInterface == imxMipiDsi) {
    MediamixDsiPwrOn();
    if (Is_Lcdifv3_Enabled(LCDIF1_DEV)) {
        CHECK_STATUS_RETURN_ERR(Lcdifv3_Power_Down(LCDIF1_DEV),
            "Lcdifv3_Power_Down");
        MicroSecondDelay(20000);
    }
  } else if ((DispInterface == imxLvds0)) {
    MediamixLvdsPwrOn();
    CHECK_STATUS_RETURN_ERR(LvdsPhyInit(0), "LvdsPhyInit()");
  } else {
    DEBUG((DEBUG_ERROR, "Usupported display interface: %d\n",
      (INTN)DispInterface));
    return EFI_NOT_FOUND;
  }

  CHECK_STATUS_RETURN_ERR(Lcdifv3_Reset(LCDIF1_DEV), "Lcdifv3_Reset");
  CHECK_STATUS_RETURN_ERR(Lcdifv3_Init(LCDIF1_DEV, FrameBaseAddress),
    "Lcdifv3_Init");

  return EFI_SUCCESS;
}

/** Return information for the requested mode number.

  @param[in]  ModeNumber          Mode Number.

  @param[out] Info                Pointer for returned mode information
                                  (on success).

  @retval EFI_SUCCESS             Mode information for the requested mode
                                  returned successfully.
  @retval EFI_INVALID_PARAMETER   Requested mode not found.
**/
EFI_STATUS
LcdQueryMode (
  IN  UINT32                                  ModeNumber,
  OUT EFI_GRAPHICS_OUTPUT_MODE_INFORMATION  *Info
  )
{
  if (ModeNumber >= VideoModesCnt){
    return EFI_INVALID_PARAMETER;
  }

  ASSERT (Info != NULL);

  Info->Version = 0;

  Info->HorizontalResolution = PreferredTiming.HActive;
  Info->VerticalResolution = PreferredTiming.VActive;
  Info->PixelsPerScanLine = PreferredTiming.HActive;

  Info->PixelFormat = PixelBlueGreenRedReserved8BitPerColor;

  return EFI_SUCCESS;
}

/** Return total number of modes supported.

  Note: Valid mode numbers are 0 to MaxMode - 1
  See Section 12.9 of the UEFI Specification 2.7

  @retval UINT32             Number of video modes.
**/
UINT32
LcdGetVideoModesCnt (VOID)
{
  return VideoModesCnt;
}

/** Return bits per pixel information for a mode number.

  @param  ModeNumber          Mode Number.
  @param Bpp                 Pointer to bits per pixel information.

  @retval EFI_SUCCESS             Bits per pixel information for the requested
                                  mode returned successfully.
  @retval EFI_INVALID_PARAMETER   Requested mode not found.
**/
EFI_STATUS
LcdGetBpp (
  IN  UINT32     ModeNumber,
  OUT LCD_BPP  * Bpp
  )
{
  EFI_STATUS status;

  if (ModeNumber >= VideoModesCnt) {
     return EFI_INVALID_PARAMETER;
  }
  ASSERT (Bpp != NULL);

  switch (PreferredTiming.Bpp) {
    case 24U:
      *Bpp = LCD_BITS_PER_PIXEL_24;
      break;
    default:
      DEBUG ((DEBUG_ERROR, "LcdGetBpp() - Unsupported bpp\n"));
      status = EFI_INVALID_PARAMETER;
      break;
  }

  return EFI_SUCCESS;
}

/**
  Set requested mode of the display.

  @param  ModeNumber             Display mode number.
  @retval EFI_SUCCESS            Display set mode success.
  @retval EFI_DEVICE_ERROR       If mode not found/supported.

**/
EFI_STATUS
LcdSetMode (
  IN  UINT32     ModeNumber
  )
{
  IMX_DISPLAY_TIMING *Timing = &PreferredTiming;

  if (ModeNumber >= VideoModesCnt) {
    return EFI_INVALID_PARAMETER;
  }

  if (DispInterface == imxMipiDsi) {
    CHECK_STATUS_RETURN_ERR(iMX9xDispClkConfigure(Timing, imxMipiDsi), "MIPI-DSI Clk config");
    CHECK_STATUS_RETURN_ERR(DwDsiConfig(Timing), "MIPI DSI config");
    if (Converter == ADV7535) {
      CHECK_STATUS_RETURN_ERR(Adv7535SetMode(Timing), "ADV7535 config");
    }
  } else if (DispInterface == imxLvds0) {
    CHECK_STATUS_RETURN_ERR(iMX9xDispClkConfigure(Timing, imxLvds0), "Lvds clk config")
    CHECK_STATUS_RETURN_ERR(LdbEnable(0, Timing),"LDB config");
    CHECK_STATUS_RETURN_ERR(LvdsPhyPowerOn(0, 0, TRUE), "LvdsPhyPowerOn");
  } else {
    DEBUG ((DEBUG_ERROR, "Unsupported display interface %d\n",
      (INTN)DispInterface));
    return EFI_INVALID_PARAMETER;
  }

  CHECK_STATUS_RETURN_ERR(Lcdifv3_SetTimingMode(LCDIF1_DEV, Timing),
    "Lcdifv3_SetTimingMode");
  CHECK_STATUS_RETURN_ERR(Lcdifv3_Enable(LCDIF1_DEV, TRUE), "Lcdifv3_Enable");

  return EFI_SUCCESS;
}

/**
  De-initializes the display.
**/
VOID
LcdShutdown (
  VOID
  )
{
  if ((DispInterface != imxMipiDsi) && (DispInterface != imxLvds0)) {
    DEBUG ((DEBUG_ERROR, "Unsupported display interface %d\n",
      (INTN)DispInterface));
  }

  EFI_STATUS Status = Lcdifv3_Enable(LCDIF1_DEV, FALSE);
  if (Status != EFI_SUCCESS) {
    DEBUG ((DEBUG_ERROR, "LCD Shutdown failed - %d\n", Status));
  }
}

/*
 * Function returns number of Bytes per pixel.
 */
UINTN
LcdGetBytesPerPixel (
  IN  IMX_PIXEL_FORMAT PixelFormat
  )
{
  switch (PixelFormat) {
    case PIXEL_FORMAT_ARGB32:
    case PIXEL_FORMAT_BGRA32:
      return 4;

    default:
      return 0;
  }
}

