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

typedef struct _ivec2
{
    int x;
    int y;
} ivec2;

typedef struct _uvec2
{
    unsigned int    x;
    unsigned int    y;
} uvec2;

typedef struct _vec2
{
    float   x;
    float   y;
} vec2;

typedef struct _uvec4
{
    unsigned int    x;
    unsigned int    y;
    unsigned int    z;
    unsigned int    w;
} uvec4;

typedef unsigned int uint;

//
// This definition must stay in sync with what shaders use
//

typedef struct _GdiOpParams 
{
    ivec2   dstOffset;
    uvec2   dstExtent;

    vec2    scale;
    vec2    srcOffset;

    vec2    srcSize;
    vec2    padding;    // Due to shader compiler packing rule

    uvec4   color;
    uvec4   color2;

    uint    gamma;
    uint    padding2[3];
} GdiOpParams;

