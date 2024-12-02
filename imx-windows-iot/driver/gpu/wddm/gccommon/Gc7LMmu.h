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

//
// WDDM Video Memory manager creates an implicit system memory segment,
// its ID is maximal driver memory segment ID + 1.
//

#if _GC7L_EMULATOR_

//
// Emulator driver has 2 segments (1: Aperture, 2: Implicit System Memory /DRAM)
//

#else

//
// GC7L HW driver has 3 (1: Aperture, 2: Local Video Memory, 3: Implicit System Memory)
//

#endif


typedef enum _Gc7LHwTileMode
{
    Linear,
    StandardTiled,
    SuperTiled
} Gc7LHwTileMode;


//
// GPU VA range internally mapped using 1MB page
//

#define GC_7L_1MB_PAGE_GPU_VA_START 0xC0000000   // 3GB
#define GC_7L_1MB_PAGE_GPU_VA_END   0x100000000  // 4GB

