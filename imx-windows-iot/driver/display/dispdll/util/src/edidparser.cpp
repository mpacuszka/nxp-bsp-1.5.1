/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"
#include "edidparser.h"

//
// VESA info
//
// https://en.wikipedia.org/wiki/Extended_Display_Identification_Data
// https://web.archive.org/web/20170821002634/https://www.vesa.org/vesa-standards/free-standards/
// https://glenwing.github.io/docs/
// 
// Display Monitor Timing: https://glenwing.github.io/docs/VESA-DMT-1.13.pdf
//


videomode g_StandardModeTimings[] = 
{
    {   // 800 x 600 x 60.004Hz
        40000000,

        800,
        40,
        88,
        128,

        600,
        1,
        23,
        4,

        (display_flags)(DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW | DISPLAY_FLAGS_PIXDATA_NEGEDGE)
    },

    {   // 1024 x 768 x 60.004Hz
        65000000,

        1024,
        24,
        160,
        136,

        768,
        3,
        29,
        6,

        (display_flags)(DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_VSYNC_LOW | DISPLAY_FLAGS_PIXDATA_NEGEDGE)
    },

    {   // 1152 x 864 x 75Hz
        108000000,

        1152,
        64,
        256,
        128,

        864,
        1,
        32,
        3,

        (display_flags)(DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_PIXDATA_NEGEDGE)
    },

    {   // 1280 x 720 x 60Hz : Used by u-boots\u-boot-imx_arm64\arch\arm\dts\imx8mq-evk.dts
        74250000,

        1280,
        110,
        220,
        40,

        720,
        5,
        20,
        5,

        (display_flags)(DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_PIXDATA_NEGEDGE)
    },

    {   // 1280 x 800 x 60Hz
        71000000,

        1280,
        48,
        80,
        32,

        800,
        3,
        14,
        6,

        (display_flags)(DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_PIXDATA_NEGEDGE)
    },

    {   // 1280 x 1024 x 60Hz : Used by u-boots\u-boot-imx_arm64\arch\arm\dts\imx8mq-evk.dts
        108000000,

        1280,
        48,
        248,
        112,

        1024,
        1,
        38,
        3,

        (display_flags)(DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_PIXDATA_NEGEDGE)
    },

    {   // 1366 x 768 x 59.790Hz
        85500000,

        1366,
        70,
        213,
        143,

        768,
        3,
        24,
        3,

        (display_flags)(DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH)
    },

    {   // 1920 x 1080 x 60Hz
        148500000,

        1920,
        88,
        148,
        44,

        1080,
        4,
        36,
        5,

        (display_flags)(DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_PIXDATA_NEGEDGE)
    }

    //
    // TODO: Add more VESA Standard Mode Timing
    //
};

unsigned int g_NumStandardModeTimings = sizeof(g_StandardModeTimings) / sizeof(videomode);

bool GetModeSize(
    void* pEdid,
    unsigned int edid_size,
    int* hactive, int* vactive)
{
    if (edid_size < EDID_SIZE)
    {
        return false;
    }

    auto pDetailedTimingDesc = (EDID_Detailed_Timing_Descriptor*)(((char*)pEdid) + EDID_DESCRIPTOR_OFFSET);
    Number_Assembler_8_4 Assembler84;

    Assembler84.value = 0;
    Assembler84.ls8 = pDetailedTimingDesc->horizontalActivePixels_8ls;
    Assembler84.ms4 = pDetailedTimingDesc->horizontalActivePixels_4ms;
    *hactive = Assembler84.value;

    Assembler84.value = 0;
    Assembler84.ls8 = pDetailedTimingDesc->verticalActiveLines_8ls;
    Assembler84.ms4 = pDetailedTimingDesc->verticalActiveLines_4ms;
    *vactive = Assembler84.value;

    return true;
}

bool GetDisplayModeTiming(
    void *pEdid,
    unsigned int edid_size,
    const D3DKMDT_VIDPN_TARGET_MODE *pModeIn,
    struct videomode *vm)
{
    //
    // TODO: Check and validate EDID
    //

    if (edid_size < EDID_SIZE)
    {
        return false;
    }

    //
    // Decode EDID Detailed Timing Descriptor
    //

    auto pDetailedTimingDesc = (EDID_Detailed_Timing_Descriptor*)(((char *)pEdid) + EDID_DESCRIPTOR_OFFSET);
    Number_Assembler_8_4 Assembler84;
    Number_Assembler_8_2 Assembler82;
    Number_Assembler_4_2 Assembler42;
    unsigned int HTotal, VTotal;

    memset(vm, 0, sizeof(struct videomode));

    vm->pixelclock = pDetailedTimingDesc->pixelClock*10*1000;

    Assembler84.value = 0;
    Assembler84.ls8 = pDetailedTimingDesc->horizontalActivePixels_8ls;
    Assembler84.ms4 = pDetailedTimingDesc->horizontalActivePixels_4ms;
    vm->hactive = Assembler84.value;

    Assembler82.value = 0;
    Assembler82.ls8 = pDetailedTimingDesc->horizontalFrontPorch_8ls;
    Assembler82.ms2 = pDetailedTimingDesc->horizontalFrontPorch_2ms;
    vm->hfront_porch = Assembler82.value;

    Assembler82.value = 0;
    Assembler82.ls8 = pDetailedTimingDesc->horizontalSyncPulseWidth_8ls;
    Assembler82.ms2 = pDetailedTimingDesc->horizontalSyncPulseWidth_2ms;
    vm->hsync_len = Assembler82.value;

    Assembler84.value = 0;
    Assembler84.ls8 = pDetailedTimingDesc->horizontalBlankingPixels_8ls;
    Assembler84.ms4 = pDetailedTimingDesc->horizontalBlankingPixels_4ms;
    HTotal = Assembler84.value;
    vm->hback_porch = HTotal - (vm->hfront_porch + vm->hsync_len);

    Assembler84.value = 0;
    Assembler84.ls8 = pDetailedTimingDesc->verticalActiveLines_8ls;
    Assembler84.ms4 = pDetailedTimingDesc->verticalActiveLines_4ms;
    vm->vactive = Assembler84.value;

    Assembler42.value = 0;
    Assembler42.ls4 = pDetailedTimingDesc->verticalFrontPorch_4ls;
    Assembler42.ms2 = pDetailedTimingDesc->verticalFrontPorch_2ms;
    vm->vfront_porch = Assembler42.value;

    Assembler42.value = 0;
    Assembler42.ls4 = pDetailedTimingDesc->verticalSyncPulseWidth_4ls;
    Assembler42.ms2 = pDetailedTimingDesc->verticalSyncPulseWidth_2ms;
    vm->vsync_len = Assembler42.value;

    Assembler84.value = 0;
    Assembler84.ls8 = pDetailedTimingDesc->verticalBlankingLines_8ls;
    Assembler84.ms4 = pDetailedTimingDesc->verticalBlankingLines_4ms;
    VTotal = Assembler84.value;
    vm->vback_porch = VTotal - (vm->vfront_porch + vm->vsync_len);

    EDID_Sync_Type  SyncType;
    unsigned int ModeFlags = 0;

    SyncType.value = pDetailedTimingDesc->syncDetails;

    switch (SyncType.type)
    {
    case 0:
    case 1:
        break;
    case 2:
    {
        EDID_Sync_Digital_Composite SyncDigitalComp;

        SyncDigitalComp.value = pDetailedTimingDesc->syncDetails;

        if (SyncDigitalComp.hSyncPolarity)
        {
            ModeFlags |= DISPLAY_FLAGS_HSYNC_HIGH;
            ModeFlags |= DISPLAY_FLAGS_VSYNC_HIGH;
        }
        else
        {
            ModeFlags |= DISPLAY_FLAGS_HSYNC_LOW;
            ModeFlags |= DISPLAY_FLAGS_VSYNC_LOW;
        }
    }
    break;

    case 3:
    {
        EDID_Sync_Digital_Separate  SyncDigitalSep;

        SyncDigitalSep.value = pDetailedTimingDesc->syncDetails;

        if (SyncDigitalSep.hSyncPolarity)
        {
            ModeFlags |= DISPLAY_FLAGS_HSYNC_HIGH;
        }
        else
        {
            ModeFlags |= DISPLAY_FLAGS_HSYNC_LOW;
        }

        if (SyncDigitalSep.vSyncPolarity)
        {
            ModeFlags |= DISPLAY_FLAGS_VSYNC_HIGH;
        }
        else
        {
            ModeFlags |= DISPLAY_FLAGS_VSYNC_LOW;
        }
    }
    break;

    default:
        return false;
    }

    vm->flags = (display_flags)ModeFlags;

    //
    // Check if input mode is the native mode
    //

    // HTotal contains HBlank period, need to add HActive to get real HTotal, the same for VTotal
    HTotal += vm->hactive;
    VTotal += vm->vactive;
    if ((pModeIn->VideoSignalInfo.TotalSize.cx == HTotal) &&
        (pModeIn->VideoSignalInfo.TotalSize.cy == VTotal) &&
        (pModeIn->VideoSignalInfo.ActiveSize.cx == vm->hactive) &&
        (pModeIn->VideoSignalInfo.ActiveSize.cy == vm->vactive) &&
        (pModeIn->VideoSignalInfo.PixelRate == vm->pixelclock))
    {
        return true;
    }

    //
    // Return VESA standard timing for the input mode
    //

    for (unsigned int i = 0; i < sizeof(g_StandardModeTimings)/sizeof(videomode); i++)
    {
        if ((pModeIn->VideoSignalInfo.ActiveSize.cx == g_StandardModeTimings[i].hactive) &&
            (pModeIn->VideoSignalInfo.ActiveSize.cy == g_StandardModeTimings[i].vactive))
        {
            bool bModeMatched = false;

            if (D3DKMDT_FREQUENCY_NOTSPECIFIED != pModeIn->VideoSignalInfo.VSyncFreq.Denominator)
            {
                unsigned int HTotal = g_StandardModeTimings[i].hactive +
                    g_StandardModeTimings[i].hfront_porch +
                    g_StandardModeTimings[i].hsync_len +
                    g_StandardModeTimings[i].hback_porch;

                unsigned int VTotal = g_StandardModeTimings[i].vactive +
                    g_StandardModeTimings[i].vfront_porch +
                    g_StandardModeTimings[i].vsync_len +
                    g_StandardModeTimings[i].vback_porch;

                unsigned int FreshRate = g_StandardModeTimings[i].pixelclock/(HTotal*VTotal);
                unsigned int FreshRateIn = pModeIn->VideoSignalInfo.VSyncFreq.Numerator/pModeIn->VideoSignalInfo.VSyncFreq.Denominator;

                if (((FreshRate - 2) < FreshRateIn) &&
                    ((FreshRate + 2) > FreshRateIn))
                {
                    bModeMatched = true;
                }
            }
            else
            {
                //
                // When DoD doesn't HW VSync, VSync frequence can't be specified
                //

                bModeMatched = true;
            }

            if (bModeMatched)
            {
                *vm = g_StandardModeTimings[i];

                return true;
            }
        }
    }

    return false;
}

