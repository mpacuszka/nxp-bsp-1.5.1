/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include "edid.h"
#include "video/videomode.h"

bool GetModeSize(
    void* pEdid,
    unsigned int edid_size,
    int* hactive, int* vactive);

bool GetDisplayModeTiming(
    void *pEdid,
    unsigned int edid_size,
    const D3DKMDT_VIDPN_TARGET_MODE *pModeIn,
    struct videomode *vm);

