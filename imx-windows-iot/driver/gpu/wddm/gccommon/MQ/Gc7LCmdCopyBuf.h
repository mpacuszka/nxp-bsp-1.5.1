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

const UINT  GC_7L_CMD_COPY_BUF[] =
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
    0x08015000, 0x00091000,     // SRC_START_GPUVA
    0x08015006, 0x00094000,     // DST_START_GPUVA
    0x08015015, 0x00003000,     // COPY_SIZE
    0x0801503d, 0x00093fff,     // SRC_END_GPUVA
    0x080150cd, 0x00096fff,     // DST_END_GPUVA
    0x0801502b, 0x00000003,
    0x08015018, 0x00000003,
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

enum GC_7L_CMD_COPY_BUF_PATCH_LOCATION
{
    GC_7L_CMD_COPY_BUF_SRC_START_GPUVA  = 0xB4/sizeof(UINT),
    GC_7L_CMD_COPY_BUF_DST_START_GPUVA  = 0xBC/sizeof(UINT),
    GC_7L_CMD_COPY_BUF_COPY_SIZE        = 0xC4/sizeof(UINT),
    GC_7L_CMD_COPY_BUF_SRC_END_GPUVA    = 0xCC/sizeof(UINT),
    GC_7L_CMD_COPY_BUF_DST_END_GPUVA    = 0xD4/sizeof(UINT)
};

