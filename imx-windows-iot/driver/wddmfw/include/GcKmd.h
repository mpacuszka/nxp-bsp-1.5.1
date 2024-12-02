/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#define GC_GPUVA_SUPPORT    1

enum GcPoolTags
{
    GC_POOL_TAG_PROCESS         = 'RPCG',
    GC_POOL_TAG_ALLOCATION      = 'LACG',
    GC_POOL_TAG_RESOURCE        = 'SRCG',
    GC_POOL_TAG_DEVICE_ALLOC    = 'ADCG',
};

