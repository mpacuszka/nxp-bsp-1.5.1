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

#include "GcKmd.h"

//
// Aperture segment base address is not used
//

#define GC_7L_APERTURE_SEGMENT_BASE_ADDRESS 0xC0000000

#define GC_7L_APERTURE_SEGMENT_SIZE         16*1024*1024

//
// Local video memory segment size need to be multiple of 16MB
// 128MB is the minimal size to declare WDDM Large Page support
//

#define GC_7L_LOCAL_VIDMEM_SEGMENT_SIZE     128*1024*1024

//
// Maximal size of Local VidMem segment
// See OFFSET_16M_READ_ONLY_STLB in GcKmd7LMmu.h for more details
//

#define GC_7L_LOCAL_VIDMEM_SEGMENT_SIZE_MAX 256*1024*1024

//
// Local VidMem segment need to be big enough for DWM and app primaries and page tables
//

#define GC_7L_LOCAL_VIDMEM_SEGMENT_SIZE_MIN 128*1024*1024


