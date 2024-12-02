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

const UINT  GC_7L_CMD_COPY_IMG[] =
{
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
    0x08015019, 0x00000000,
    0x08015016, 0xffffffff,
    0x08015017, 0xffffffff,
    0x08015002, 0x00c04000,     // SRC_CONFIG : TILING, MSAA, FORMAT, STRIDE
    0x08015003, 0x00000000,     // SRC_CONFIG_EX: TILE_MODE(22:21)
    0x08015000, 0x00106000,     // SRC_START_GPUVA
    0x08015009, 0x60c04000,     // DST_CONIFG : TILING, MSAA, FORMAT, STRIDE
    0x0801500a, 0x04400000,     // DST_CONFIG_EX: TILE_MODE(27:26), BLOCK_SIZE(22:22)
    0x08015006, 0x1c107000,     // DST_START_GPUVA
    0x0801502f, 0x00688688,
    0x0801503d, 0x04105fff,     // SRC_END_GPUVA
    0x080150cd, 0x20106fff,     // DST_END_GPUVA
    0x08015005, 0x04560345,     // SRC_ORIGIN_YX
    0x0801500b, 0x02340123,     // DST_ORIGIN_YX
    0x0801500c, 0x02000200,     // WINDOW_SIZE_YX
    0x08015028, 0x00040004,
    0x08015027, 0x00400040,
    0x0801502b, 0x00000003,
    0x08015018, 0x00000002,
    0x0801502b, 0x00000003,
    0x0801502e, 0x00000000,
    0x0801502e, 0x00000001,
    0x08010e20, 0x00001001,
    0x48000000, 0x00001001,
    0x0801502e, 0x00000000,
    0x08010e02, 0x00000701,
    0x48000000, 0x00000701,
    0x08010e03, 0x00000c67,
    0x08010e03, 0x00000010,
    0x0801022c, 0x0000001f,
    0x08015311, 0x00000000,
    0x08010e02, 0x00000701,
    0x48000000, 0x00000701
};

enum GC_7L_CMD_COPY_IMG_PATCH_LOCATION
{
    GC_7L_CMD_COPY_IMG_SRC_CONFIG       = 0x0AC/sizeof(UINT),
    GC_7L_CMD_COPY_IMG_SRC_CONFIG_EX    = 0x0B4/sizeof(UINT),
    GC_7L_CMD_COPY_IMG_SRC_START_GPUVA  = 0x0BC/sizeof(UINT),
    GC_7L_CMD_COPY_IMG_DST_CONFIG       = 0x0C4/sizeof(UINT),
    GC_7L_CMD_COPY_IMG_DST_CONFIG_EX    = 0x0CC/sizeof(UINT),
    GC_7L_CMD_COPY_IMG_DST_START_GPUVA  = 0x0D4/sizeof(UINT),
    GC_7L_CMD_COPY_IMG_SRC_END_GPUVA    = 0x0E4/sizeof(UINT),
    GC_7L_CMD_COPY_IMG_DST_END_GPUVA    = 0x0EC/sizeof(UINT),
    GC_7L_CMD_COPY_IMG_SRC_ORIGIN_YX    = 0x0F4/sizeof(UINT),
    GC_7L_CMD_COPY_IMG_DST_ORIGIN_YX    = 0x0FC/sizeof(UINT),
    GC_7L_CMD_COPY_IMG_WINDOW_SIZE_YX   = 0x104/sizeof(UINT),
};

