/** @file

  Copyright (c) 2020, Linaro, Ltd. All rights reserved.
  Copyright 2020, 2022-2023 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Uefi/UefiBaseType.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>

#include <iMXDisplay.h>
#include <iMXI2cLib.h>
#include "adv7535.h"
#include "MipiDsi.h"
#include "ldb.h"
#include "iMX8.h"
#include "it6263.h"
#include "dw_hdmi.h"
#include "MipiDsi_rm67191.h"

#if defined(CPU_IMX8MP)
  #define DISP_CTRL_LCDIFV3
  #define DISP_CTRL_LDB
  #define DISP_CTRL_HDMI
#elif defined(CPU_IMX8MM) || defined(CPU_IMX8MN)
  #define DISP_CTRL_LCDIF
#else
  #error Unsupported derivative!
#endif

#if defined(DISP_CTRL_LCDIFV3)
#include "Lcdifv3.h"
#endif
#if defined(DISP_CTRL_LCDIF)
#include "Lcdif.h"
#endif

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
/* 1080x1920@60Hz */
const IMX_DISPLAY_TIMING PreferredTiming_1080x1920_60 = {
  .PixelClock = 108000000,
  .HActive = 1080,
  .HBlank = 56,
  .VActive = 1920,
  .VBlank = 16,
  .HSync = 2,
  .VSync = 2,
  .HSyncOffset = 34,
  .VSyncOffset = 4,
  .HImageSize = 296,
  .VImageSize = 527,
  .HBorder = 0,
  .VBorder = 0,
  .EdidFlags = 0,
  .Flags = 0,
  .PixelRepetition = 0,
  .Bpp = 24,
  .PixelFormat = PIXEL_FORMAT_ARGB32,
};

/* 1920x1080@60Hz */
const IMX_DISPLAY_TIMING PreferredTiming_1920x1080_60 = {
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
const IMX_DISPLAY_TIMING PreferredTiming_1920x1200_60 = {
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
const IMX_DISPLAY_TIMING PreferredTiming_1280x1024_60 = {
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

/* 1366x768@60 */
const IMX_DISPLAY_TIMING PreferredTiming_1366x768_60 = {
  .PixelClock = 85500000,
  .HActive = 1366,
  .HBlank = 426,
  .VActive = 768,
  .VBlank = 30,
  .HSync = 143,
  .VSync = 3,
  .HSyncOffset = 70,
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

/* 1280x720@60 */
const IMX_DISPLAY_TIMING PreferredTiming_1280x720_60 = {
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
const IMX_DISPLAY_TIMING PreferredTiming_1024x768_60 = {
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
const IMX_DISPLAY_TIMING PreferredTiming_800x600_60 = {
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
int videoModesCnt = 0;

/* Get display interface type defined in *.dsc file */
imxDisplayInterfaceType displayInterface = FixedPcdGet32(PcdDisplayInterface);
/* Type of converter */
imxConverter converter = transmitterUnknown;

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
static VOID
LcdInitPreferredTiming (
  IN const IMX_DISPLAY_TIMING *SourceTiming,
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
static VOID
LcdDumpDisplayTiming (
  IN uint32_t            DtdOffset,
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

  @param  edid                   Buffer for EDID data.
  @retval EFI_SUCCESS            Platform implements display.
  @retval EFI_NOT_FOUND          Display not found on the platform.

**/
static EFI_STATUS
LcdReadEdid (
  OUT uint8_t *edid,
  IN uint32_t offset,
  IN uint32_t length
  )
{
  EFI_STATUS status = EFI_SUCCESS;

  /* For now read only standard EDID structure , ignore EDID extensions */
  if(displayInterface == imxMipiDsi) {
    if(converter == ADV7535) {
      status = Adv7535ReadEdid(edid, offset, length);
    }
#if defined(DISP_CTRL_LDB)
  } else if ((displayInterface == imxLvds0) || (displayInterface == imxLvds1) || (displayInterface == imxLvds0dual)) {
    if (converter == IT6263) {
      status = It6263ReadEdid(edid, offset, length);
    }
#endif
#if defined(DISP_CTRL_HDMI)
  } else if ((displayInterface == imxNativeHdmi) && (converter == NativeHDMI)) {
    status = dw_hdmi_read_edid(edid, offset, length);
#endif
  } else {
    DEBUG((DEBUG_ERROR, "Usupported display interface: %d\n", (int)displayInterface));
    status = EFI_NOT_FOUND;
  }

  return status;
}

/**
  Check converter resolution is supported by EDID data

  @param  DTDPtr                 Pointer to EDID Detailed Timing Descriptor
  @param  MaxHactive             Maxumum horizontal resolution
  @param  MaxVactive             Maxumum vertical resolution

  @retval TRUE                   EDID resolution is lower or equal to Maximum
  @retval FALSE                  EDID resolution exceeds Maximum

**/
static BOOLEAN
LcdConvSuppResolution(
  IN IMX_DETAILED_TIMING_DESCRIPTOR   *DTDPtr
)
{
  uint32_t hactive;
  uint32_t vactive;
  uint32_t hmax;
  uint32_t vmax;
  
  hactive = (DTDPtr->HActiveBlank & 0xF0);
  hactive = (hactive << 4) | DTDPtr->HActive;
  vactive = (DTDPtr->VActiveBlank & 0xF0);
  vactive = (vactive << 4) | DTDPtr->VActive;

  switch (displayInterface) {
    case imxMipiDsi:
    case imxLvds0dual:
    case imxNativeHdmi:
        hmax = CONV_MAX_HACTIVE_1920;
        vmax = CONV_MAX_VACTIVE_1080;
        break;
    case imxLvds0:
    case imxLvds1:
        hmax = CONV_MAX_HACTIVE_1280;
        vmax = CONV_MAX_VACTIVE_720;
        break;
    default:
        DEBUG((DEBUG_ERROR, "Usupported display interface: %d\n", (int)displayInterface));
        return FALSE;
  }
  
  if ((hactive <= hmax)  && (vactive <= vmax)) {
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
  EFI_STATUS         status = EFI_SUCCESS;
  uint8_t           *edid;
  int                edid_extensions = 0;
  BOOLEAN            conv_set_edid_always = 1;

  /* Try to autodetect connected converter regardless of selected displayInterface
     mipi-dsi with ADV7535 inteface has priority over lvds with IT6263 */
  if (converter == transmitterUnknown) {
    do {
      /* Search for ADV7535 */
      status = Adv7535Discover();
      if (status == EFI_SUCCESS) {
        DEBUG((DEBUG_ERROR, "ADV7535 probe SUCCEDED. Mipi-dsi display interface selected.\n"));
        converter = ADV7535;
        displayInterface = imxMipiDsi;
        LcdInitPreferredTiming (&PreferredTiming_1920x1080_60, &PreferredTiming);
        break;
      }
#if defined(DISP_CTRL_LDB)
      /* Search for IT6263 */
      status = It6263Discover();
      if (status == EFI_SUCCESS) {
        converter = IT6263;
        if ((displayInterface != imxLvds0) && (displayInterface != imxLvds1) && (displayInterface != imxLvds0dual)) {
            /* lvds is already requested, don't change it, so we can test DLVDS and LVDS1 interfaces.
               Otherwise select lvds0 */
            displayInterface = imxLvds0;
        }
        DEBUG((DEBUG_ERROR, "IT6263 probe SUCCEDED. LVDS%d display interface selected.\n", (displayInterface-2)));
        LcdInitPreferredTiming (&PreferredTiming_1280x720_60, &PreferredTiming);
        break;
      }
#endif
#if defined(DISP_CTRL_HDMI)
      /* Detect HDMI */
      dw_hdmi_imx_init();
      status = dw_hdmi_phy_wait_for_hpd();
      if (status == EFI_SUCCESS) {
        DEBUG((DEBUG_ERROR, "Native HDMI hot plug detected. Native HDMI display interface selected.\n"));
        converter = NativeHDMI;
        displayInterface = imxNativeHdmi;
        LcdInitPreferredTiming (&PreferredTiming_1920x1080_60, &PreferredTiming);
        break;
      }
#endif
    } while(0);
  }
  DEBUG((DEBUG_INFO, "HDMI converter: %d\n", converter));
  /* Converter was not detected - select fixed default timimng */
  if (converter == transmitterUnknown) {
    if (displayInterface == imxMipiDsi) {
      videoModesCnt++;
      LcdInitPreferredTiming (&PreferredTiming_1080x1920_60, &PreferredTiming);
      DEBUG((DEBUG_ERROR, "Mipi-dsi display interface. Default resolution used. %dx%d pclk=%d Hz\n", 
            PreferredTiming.HActive, PreferredTiming.VActive, PreferredTiming.PixelClock));
      LcdDumpDisplayTiming(0, &PreferredTiming);
      return EFI_SUCCESS;
    } else if ((displayInterface == imxLvds0) || (displayInterface == imxLvds1) || (displayInterface == imxLvds0dual)) {
      videoModesCnt++;
      LcdInitPreferredTiming (&PreferredTiming_1280x720_60, &PreferredTiming);
      DEBUG((DEBUG_ERROR, "LVDS%d display interface. Default resolution used. %dx%d pclk=%d Hz\n",
            displayInterface-2, PreferredTiming.HActive, PreferredTiming.VActive, PreferredTiming.PixelClock));
      LcdDumpDisplayTiming(0, &PreferredTiming);
      return EFI_SUCCESS;
    } else if (displayInterface == imxNativeHdmi) {
      videoModesCnt++;
      LcdInitPreferredTiming (&PreferredTiming_1920x1080_60, &PreferredTiming);
      DEBUG((DEBUG_ERROR, "HDMI display interface. Fixed default resolution used. %dx%d pclk=%d Hz\n",
            PreferredTiming.HActive, PreferredTiming.VActive, PreferredTiming.PixelClock));
      LcdDumpDisplayTiming(0, &PreferredTiming);
      return EFI_SUCCESS;
    } else {
      DEBUG((DEBUG_ERROR, "Usupported display interface: %d\n", (int)displayInterface));
      return EFI_NOT_FOUND;
    }
  }

  if (FixedPcdGet32(PcdDisplayReadEDID) == TRUE) {
    /* Allocate memory for EDID structure */
    edid = AllocatePool(BASIC_EDID_STRUCTURE_LENGTH + 1);

    /* Read EDID */
    status = LcdReadEdid(edid, 0, BASIC_EDID_STRUCTURE_LENGTH);
    if (EFI_ERROR(status)) {
      DEBUG((DEBUG_ERROR, "Unable to read EDID\n"));
      goto ErrorEdid;
    }

    /* Ignore extensions */
    edid_extensions = edid[126];
    DEBUG((LCDHWLIB_DEBUG_LEVEL, "EDID Version: %d.%d\n",edid[EDID_VERSION_REG_OFFSET],edid[EDID_REVISION_REG_OFFSET]));
    DEBUG((LCDHWLIB_DEBUG_LEVEL, "EDID Num of Extensions: %d\n", edid_extensions));


    /* Validate EDID data to */
    status = ImxValidateEdidData(edid);
    if (EFI_ERROR(status)) {
      DEBUG((DEBUG_ERROR, "EDID data not valid\n"));
      goto ErrorEdid;
    }

    if (converter == ADV7535 || converter == IT6263 || converter == NativeHDMI) {
      if (FixedPcdGet32(PcdDisplayForceConverterMaxResolution) == TRUE) {
        conv_set_edid_always = 0;
      }
    }

    /* Read first DTD, which is the most preferred */
    for (int dtd_idx = IMX_EDID_DTD_1_OFFSET; dtd_idx <= IMX_EDID_DTD_1_OFFSET; dtd_idx += IMX_EDID_DTD_SIZE) {
      /* Convert EDID data into internal format */
      if (conv_set_edid_always || 
          LcdConvSuppResolution((IMX_DETAILED_TIMING_DESCRIPTOR *)&edid[dtd_idx])) {
        status = ImxConvertDTDToDisplayTiming((IMX_DETAILED_TIMING_DESCRIPTOR *)&edid[dtd_idx],
                                              &PreferredTiming);
        if (status != EFI_SUCCESS) {
          DEBUG((DEBUG_ERROR, "EDID Conversion to display timing failed\n"));
          goto End;
        }
        DEBUG((DEBUG_ERROR, "EDID Conversion to display timing successfull.\n"));
      } else {
        DEBUG((DEBUG_ERROR, "EDID Conversion to display timing skipped. Using default fixed resolution instead.\n"));
      }
      videoModesCnt++;
      /* BPP is fixed to 24 (8 bits per color component) */
      PreferredTiming.Bpp = 24;
      PreferredTiming.PixelFormat = PIXEL_FORMAT_ARGB32;
      LcdDumpDisplayTiming(dtd_idx, &PreferredTiming);
    }
    DEBUG((DEBUG_ERROR, "Selected resolution %dx%d pclk=%d Hz\n",
           PreferredTiming.HActive, PreferredTiming.VActive, PreferredTiming.PixelClock));
End:
    FreePool(edid);
  } else {
    videoModesCnt++;
    DEBUG((DEBUG_ERROR, "Selected default resolution %dx%d pclk=%d Hz\n",
           PreferredTiming.HActive, PreferredTiming.VActive, PreferredTiming.PixelClock));
    LcdDumpDisplayTiming(0, &PreferredTiming);
  }
  return status;

ErrorEdid:
    FreePool(edid);
    videoModesCnt++;
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
  /* Reset MIPI-DSI & LCDIF */
  if (displayInterface == imxMipiDsi) {
    MipiDsiReset();
#if defined(DISP_CTRL_LCDIFV3)
    CHECK_STATUS_RETURN_ERR(Lcdifv3_Reset(LCDIF1_DEV), "Lcdifv3_Reset");
    CHECK_STATUS_RETURN_ERR(Lcdifv3_Init(LCDIF1_DEV, FrameBaseAddress), "Lcdifv3_Init");
#endif
#if defined(DISP_CTRL_LCDIF)
    LcdifReset();
    LcdifInit(FrameBaseAddress);
#endif
#if defined(DISP_CTRL_LDB)
  } else if ((displayInterface == imxLvds0) || (displayInterface == imxLvds1) || (displayInterface == imxLvds0dual)) {
    LdbReset();
#if defined(DISP_CTRL_LCDIFV3)
    CHECK_STATUS_RETURN_ERR(Lcdifv3_Reset(LCDIF2_DEV), "Lcdifv3_Reset");
    CHECK_STATUS_RETURN_ERR(Lcdifv3_Init(LCDIF2_DEV, FrameBaseAddress), "Lcdifv3_Init");
#endif /* DISP_CTRL_LCDIFV3 */
#endif /* DISP_CTRL_LDB */
#if defined(DISP_CTRL_HDMI)
  } else if (displayInterface == imxNativeHdmi) {
    /* HDMI init was done in LcdDisplayDetect */
#if defined(DISP_CTRL_LCDIFV3)
    CHECK_STATUS_RETURN_ERR(Lcdifv3_Reset(LCDIF3_DEV), "Lcdifv3_Reset");
    CHECK_STATUS_RETURN_ERR(Lcdifv3_Init(LCDIF3_DEV, FrameBaseAddress), "Lcdifv3_Init");
#endif /* DISP_CTRL_LCDIFV3 */
#endif /* DISP_CTRL_HDMI */
  } else {
    DEBUG((DEBUG_ERROR, "Usupported display interface: %d\n", (int)displayInterface));
    return EFI_NOT_FOUND;
  }

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
  OUT EFI_GRAPHICS_OUTPUT_MODE_INFORMATION  * Info
  )
{
  if (ModeNumber >= videoModesCnt){
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
  return videoModesCnt;
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

  if (ModeNumber >= videoModesCnt) {
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
  
  if (ModeNumber >= videoModesCnt) {
    return EFI_INVALID_PARAMETER;
  }

  if (displayInterface == imxMipiDsi) {
  /*--------------------------MIPI----------------------------------------------*/
    /* Mipi DSI set timing mode */
    CHECK_STATUS_RETURN_ERR(MipiDsiConfig(Timing, converter), "MIPI DSI config");
    if (converter == ADV7535) {
      /* ADV7535 set timing mode */
      CHECK_STATUS_RETURN_ERR(Adv7535SetMode(Timing), "ADV7535 config");
    } else {
      /* MIPI-DSI panel init must be called after MipiDsiConfig() */
      CHECK_STATUS_RETURN_ERR(Rm67191Init(displayInterface), "RM67191 config");
    }
#if defined(DISP_CTRL_LCDIFV3)
    /* LCDIF set timing mode */
    CHECK_STATUS_RETURN_ERR(Lcdifv3_SetTimingMode(LCDIF1_DEV, Timing), "Lcdifv3_SetTimingMode");
    /* Enable LCDIF */
    CHECK_STATUS_RETURN_ERR(Lcdifv3_Enable(LCDIF1_DEV, TRUE), "Lcdifv3_Enable");
#endif /* defined(CPU_IMX8MP) */
#if defined(DISP_CTRL_LCDIF)
    /* LCDIF set timing mode */
    CHECK_STATUS_RETURN_ERR(LcdifSetTimingMode(Timing), "LCDIF config")
    /* Enable LCDIF */
    CHECK_STATUS_RETURN_ERR(LcdifEnable(TRUE), "LCDIF enable")
#endif /* ifndef CPU_IMX8MP */
#if defined(DISP_CTRL_LDB)
  } else if ((displayInterface == imxLvds0) || (displayInterface == imxLvds1) || (displayInterface == imxLvds0dual)) {
  /*--------------------------LVDS----------------------------------------------*/
    CHECK_STATUS_RETURN_ERR(LdbConfig(Timing, displayInterface),"LDB config");
    if (converter == IT6263) {
      /* ADV7535 set timing mode */
      CHECK_STATUS_RETURN_ERR(It6263SetMode(Timing, displayInterface), "IT6263 config");
    }

#if defined(DISP_CTRL_LCDIFV3)
    /* LCDIF set timing mode */
    CHECK_STATUS_RETURN_ERR(Lcdifv3_SetTimingMode(LCDIF2_DEV, Timing), "Lcdifv3_SetTimingMode");
    /* Enable LCDIF */
    CHECK_STATUS_RETURN_ERR(Lcdifv3_Enable(LCDIF2_DEV, TRUE), "Lcdifv3_Enable");
#endif /* DISP_CTRL_LCDIFV3 */
#endif /* DISP_CTRL_LDB */
#if defined(DISP_CTRL_HDMI)
  } else if (displayInterface == imxNativeHdmi) {
  /*--------------------------HDMI----------------------------------------------*/
    uint32_t real_rate = 0;
    CHECK_STATUS_RETURN_ERR(dw_hdmi_imx_phy_clk_set_pll(Timing->PixelClock, &real_rate), "HDMI Phy PLL config");
    if (Timing->PixelClock != real_rate) {
      Timing->PixelClock = real_rate;
    }
#if defined(DISP_CTRL_LCDIFV3)
    /* LCDIF set timing mode */
    CHECK_STATUS_RETURN_ERR(Lcdifv3_SetTimingMode(LCDIF3_DEV, Timing), "Lcdifv3_SetTimingMode");
    /* Enable LCDIF */
    CHECK_STATUS_RETURN_ERR(Lcdifv3_Enable(LCDIF3_DEV, TRUE), "Lcdifv3_Enable");
#endif /* DISP_CTRL_LCDIFV3 */
    /* HDMI monitor: Timing->Flags |= EDID_FLAGS_HDMI; */
    CHECK_STATUS_RETURN_ERR(dw_hdmi_enable(Timing, FMT_RGB888, FMT_RGB888),"HDMI config");
#endif /* DISP_CTRL_HDMI */
  } else {
    DEBUG ((DEBUG_ERROR, "Unsupported display interface %d\n", (int)displayInterface));
    return EFI_INVALID_PARAMETER;
  }

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
  EFI_STATUS         status = EFI_SUCCESS;

  if (displayInterface == imxMipiDsi) {
#if defined(DISP_CTRL_LCDIFV3)
    status = Lcdifv3_Enable(LCDIF1_DEV, FALSE);
#endif
#if defined(DISP_CTRL_LCDIF)
    status = LcdifEnable(FALSE);
#endif
    if (status != EFI_SUCCESS) {
      DEBUG ((DEBUG_ERROR, "LCD Shutdown failed - %d\n", status));
    }
  } else if ((displayInterface == imxLvds0) || (displayInterface == imxLvds1) || (displayInterface == imxLvds0dual)) {
#if defined(DISP_CTRL_LCDIFV3)
    status = Lcdifv3_Enable(LCDIF2_DEV, FALSE);
    if (status != EFI_SUCCESS) {
      DEBUG ((DEBUG_ERROR, "LCD Shutdown failed - %d\n", status));
    }
#endif
#if defined(DISP_CTRL_HDMI)
  } else if (displayInterface == imxNativeHdmi) {
#if defined(DISP_CTRL_LCDIFV3)
    status = Lcdifv3_Enable(LCDIF3_DEV, FALSE);
    if (status != EFI_SUCCESS) {
      DEBUG ((DEBUG_ERROR, "LCD Shutdown failed - %d\n", status));
    }
#endif
#endif
  } else {
    DEBUG ((DEBUG_ERROR, "Unsupported display interface %d\n", (int)displayInterface));
  }

  return;
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
