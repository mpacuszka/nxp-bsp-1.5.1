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

enum GC_7L_PATCH_TYPE
{
    GC7L_PATCH_INST_GPUVA,
    GC7L_PATCH_DST_STORAGE_IMAGE,
    GC7L_PATCH_SRC_STORAGE_IMAGE,
    GC7L_PATCH_SRC_STORAGE_IMAGE_2,
    GC7L_PATCH_SRC_SAMPLED_IMAGE,
    GC7L_PATCH_UNIFORM_GPUVA,
    GC7L_PATCH_DISPATCH_WG,
    GC7L_PATCH_DISPATCH_WG2
};

typedef struct _GC_7L_PATCH_ENTRY
{
    unsigned int        Offset;
    GC_7L_PATCH_TYPE    Type;
} GC_7L_PATCH_ENTRY;

