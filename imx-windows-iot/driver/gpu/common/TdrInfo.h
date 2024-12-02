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

#include <d3dkmddi.h>

#define RegisterRangeStartEnd(x,y) {x, y-x+1}
#define SizeOfRange(x) sizeof(Gc7LRegisterRangeDescriptor)+x*sizeof(UINT);

enum Gc7LBlobType : UINT
{
    RegisterRanges = 0
};

struct Gc7LRegisterRangeDescriptor
{
    UINT startRegIndex;
    UINT numberOfRegisters;
};

struct Gc7LBlobHeader
{
    Gc7LBlobType type;
    UINT size;
};

struct Gc7LRegisterRange
{
    Gc7LRegisterRangeDescriptor rangeDescriptor;
    UINT registers[1];
};

struct Gc7LRegisterRangeBlob
{
    Gc7LBlobHeader header;
    Gc7LRegisterRange ranges[1];
};

