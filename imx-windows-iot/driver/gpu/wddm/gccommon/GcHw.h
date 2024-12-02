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
// GC GPU device ID
//

#define GC_7000_LITE_MQ L"ACPI\\VEN_VERI&DEV_7000"
#define GC_7000_LITE_MP L"ACPI\\VEN_VERI&DEV_7001"
#define GC_7000_LITE_MN L"ACPI\\VEN_VERI&DEV_7003"
#define GC_7000_LITE_8X L"ACPI\\VEN_VERI&DEV_7004"

typedef enum _GC_7L_ENGINE_ORDINAL
{
    GC_7L_ENGINE_ORDINAL_3D = 0,
    GC_7L_ENGINE_ORDINAL_COPY = 1,
    GC_7L_ENGINE_COUNT
} GC_7L_ENGINE_ORDINAL;

