/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

typedef struct _EDID_Detailed_Timing_Descriptor
{
    unsigned short  pixelClock;
    unsigned char   horizontalActivePixels_8ls;
    unsigned char   horizontalBlankingPixels_8ls;
    union
    {
        struct
        {
            unsigned char   horizontalBlankingPixels_4ms: 4;
            unsigned char   horizontalActivePixels_4ms  : 4;
        };
        unsigned char   value;
    };
    unsigned char   verticalActiveLines_8ls;
    unsigned char   verticalBlankingLines_8ls;
    union
    {
        struct
        {
            unsigned char   verticalBlankingLines_4ms   : 4;
            unsigned char   verticalActiveLines_4ms     : 4;
        };
        unsigned char   value;
    };
    unsigned char   horizontalFrontPorch_8ls;
    unsigned char   horizontalSyncPulseWidth_8ls;
    union
    {
        struct
        {
            unsigned char   verticalSyncPulseWidth_4ls  : 4;
            unsigned char   verticalFrontPorch_4ls      : 4;
        };
        unsigned char value;
    };
    union
    {
        struct
        {
            unsigned char   verticalSyncPulseWidth_2ms  : 2;
            unsigned char   verticalFrontPorch_2ms      : 2;
            unsigned char   horizontalSyncPulseWidth_2ms: 2;
            unsigned char   horizontalFrontPorch_2ms    : 2;
        };
        unsigned char value;
    };
    unsigned char   horizontalImageSize_8ls;
    unsigned char   verticalImageSize_8ls;
    union
    {
        struct
        {
            unsigned char   verticalImageSize_4ms       : 4;
            unsigned char   horizontalImageSize_4ms     : 4;
        };
        unsigned char value;
    };
    unsigned char   horizontalBorderPixels;
    unsigned char   verticalBorderLines;
    union
    {
        struct
        {
            unsigned char   stereoMode_1ls              : 1;
            unsigned char   syncDetails                 : 4;
            unsigned char   stereoMode_2ms              : 2;
            unsigned char   signalInterfaceType         : 1;
        };
        unsigned char value;
    };
} EDID_Detailed_Timing_Descriptor;

typedef union _EDID_Sync_Type
{
    struct
    {
        unsigned char                   : 2;
        unsigned char   type            : 2;
        unsigned char                   : 4;
    };
    unsigned char   value;
} EDID_Sync_Type;

typedef union _EDID_Sync_Analog
{
    struct
    {
        unsigned char   allChannels     : 1;
        unsigned char   serrations      : 1;
        unsigned char   bipolar         : 1;
        unsigned char   type            : 1;
        unsigned char                   : 4;
    };
    unsigned char   value;
} EDID_Sync_Analog;

typedef union _EDID_Sync_Digital_Composite
{
    struct
    {
        unsigned char   hSyncPolarity   : 1;
        unsigned char   serrations      : 1;
        unsigned char   type            : 2;    // type of 2
        unsigned char                   : 4;
    };
    unsigned char   value;
} EDID_Sync_Digital_Composite;

typedef union _EDID_Sync_Digital_Separate
{
    struct
    {
        unsigned char   hSyncPolarity   : 1;
        unsigned char   vSyncPolarity   : 1;
        unsigned char   type            : 2;    // type of 3;
        unsigned char                   : 4;
    };
    unsigned char   value;
} EDID_Sync_Digital_Separate;


typedef union _Number_Assembler_8_4
{
    struct
    {
        unsigned int    ls8     : 8;
        unsigned int    ms4     : 4;
        unsigned int    zeroes  : 20;
    };
    unsigned int    value;
} Number_Assembler_8_4;

typedef union _Number_Assembler_8_2
{
    struct
    {
        unsigned int    ls8 : 8;
        unsigned int    ms2 : 2;
        unsigned int    zeroes : 22;
    };
    unsigned int    value;
} Number_Assembler_8_2;

typedef union _Number_Assembler_4_2
{
    struct
    {
        unsigned int    ls4     : 4;
        unsigned int    ms2     : 2;
        unsigned int    zeroes  : 26;
    };
    unsigned int    value;
} Number_Assembler_4_2;

const unsigned int  EDID_SIZE = 128;
const unsigned int  EDID_DESCRIPTOR_OFFSET = 54;

