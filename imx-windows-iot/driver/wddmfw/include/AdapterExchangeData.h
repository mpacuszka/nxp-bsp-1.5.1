/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include <d3dkmddi.h>

const UINT GC_DRIVER_VERSION    = 1;
const UINT MAX_DEVICE_ID_LENGTH = 32;

//
// Structure for exchange adapter info between UMD and KMD
//
typedef struct AdapterExchangeData
{
    UINT                m_version;
    DXGK_WDDMVERSION    m_wddmVersion;
    BOOL                m_isSoftwareDevice;
    WCHAR               m_deviceId[MAX_DEVICE_ID_LENGTH];
} AdapterExchangeData;
