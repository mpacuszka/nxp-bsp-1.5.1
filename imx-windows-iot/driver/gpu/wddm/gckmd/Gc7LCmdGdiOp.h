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

#include "d3dkmddi.h"
#include "Gc7LCmdBufPatch.h"

typedef struct _GC_7L_GDI_OP
{
    const unsigned int*         pInst;
    const unsigned int          InstSize;
    unsigned int                InstGpuVa;
    const unsigned int*         pCmdBuf;
    unsigned int                CmdBufSize;
    const GC_7L_PATCH_ENTRY*    pPatchTable;
    unsigned int                NumPatchEntries;
} GC_7L_GDI_OP;


