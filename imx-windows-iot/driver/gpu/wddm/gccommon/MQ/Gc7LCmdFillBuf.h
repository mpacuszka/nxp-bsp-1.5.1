/****************************************************************************
* Copyright (c) Microsoft Corporation.
*
*    Licensed under the MIT License.
*    Licensed under the GPL License.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/

#pragma once

const UINT  GC_7L_CMD_FILL_BUF[] =
{
    0x00000000, 0x00000000,     // Header
    0x00000000, 0x00000000,
    0x00000000, 0x00000000,
    0x00000000, 0x00000000,

    0x08010e00, 0x00000000,
    0x08010e80, 0x30000000,
    0x080101f6, 0x00000000,
    0x08010380, 0x00000001,
    0x0801028b, 0x34000001,
    0x08010221, 0x00000808,
    0x08015310, 0x00000001,
    0x08010e03, 0x00004c27,
    0x08010e03, 0x00000010,
    0x08010e02, 0x00000701,
    0x48000000, 0x00000701,
    0x08010e03, 0x00000c67,
    0x08010e03, 0x00000010,
    0x0801022c, 0x0000001f,
    0x08015311, 0x00000000,
    0x08010e02, 0x00000701,
    0x48000000, 0x00000701,
    0x0801502e, 0x00000001,
    0x08015019, 0x00000180,
    0x08015009, 0x00c01000,
    0x0801500a, 0x00000000,
    0x08015002, 0x00c01000,
    0x08015003, 0x00000000,
    0x08015006, 0x00091000,     // DST_START_GPUVA
    0x08015000, 0x00091000,     // SRC_START_GPUVA
    0x0801503d, 0x00093fff,     // SRC_END_GPUVA
    0x080150cd, 0x00093fff,     // DST_END_GPUVA
    0x0801500b, 0x00000000,
    0x0801500c, 0x00030400,     // FILL_RECT (31 : 16 for height, 16 : 0 for width in UINTs)
    0x08015011, 0xf0804020,     // FILL_PATTERN
    0x08015013, 0xffffffff,
    0x0801502b, 0x00000003,
    0x08015018, 0x00000001,
    0x0801502b, 0x00000003,
    0x0801502e, 0x00000000,
    0x0801502e, 0x00000001,
    0x08010e02, 0x30001001,
    0x48000000, 0x30001001,
    0x0801502e, 0x00000000,
    0x08010e02, 0x00000701,
    0x48000000, 0x00000701,
    0x08010e03, 0x00000c67,
    0x08010e03, 0x00000010,
    0x0801022c, 0x0000001f,
    0x08015311, 0x00000000,
    0x08010e02, 0x00000701,
    0x48000000, 0x00000701,

    0x00000000, 0x00000000,     // Footer
    0x00000000, 0x00000000,
    0x00000000, 0x00000000,
    0x00000000, 0x00000000
};

enum GC_7L_CMD_FILL_BUF_PATCH_LOCATION
{
    GC_7L_CMD_FILL_BUF_DST_START_GPUVA  =  0xDC/sizeof(UINT),
    GC_7L_CMD_FILL_BUF_SRC_START_GPUVA  =  0xE4/sizeof(UINT),
    GC_7L_CMD_FILL_BUF_SRC_END_GPUVA    =  0xEC/sizeof(UINT),
    GC_7L_CMD_FILL_BUF_DST_END_GPUVA    =  0xF4/sizeof(UINT),
    GC_7L_CMD_FILL_BUF_FILL_RECT        = 0x104/sizeof(UINT),
    GC_7L_CMD_FILL_BUF_FILL_PATTERN     = 0x10C/sizeof(UINT)
};

#ifndef GC_7L_RESTRAINED_FILL
const UINT  GC_7L_CMD_FILL_BUF_MAXIMAL_FILL = 0xFFFF*0x400*4; // Theoretical maximal height of 0xFFFF * width of 0x400 UINT
#else
const UINT  GC_7L_CMD_FILL_BUF_MAXIMAL_FILL = 0x200*0x400*4;  // Maximal height of 0x200 * width of 0x400 UINT
#endif

