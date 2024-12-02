/* Copyright (c) Microsoft Corporation.
 * Copyright 2023-2024 NXP
   Licensed under the MIT License. */

#pragma once

#include <ntddk.h>
#include "GcKmdBaseTransmitter.h"
#include "GcKmd7LIO.h"
#include "edidparser.h"

extern "C" {
#include "linux/platform_device.h"
#include "mipi_dsi/nwl-dsi.h"
#include "linux/i2c.h"
}

class NwlDsiTransmitter : public BaseTransmitter
{
public:

    NwlDsiTransmitter() : m_CachedEdid{},
        m_dsi_pdev{},
        m_panel_pdev{},
        m_dsi_phy_pdev{},
        m_disp_interface(0),
        m_num_lanes(0),
        m_channel_id(0),
        m_i2c_main{},
        m_i2c_edid{},
        m_i2c_cec{}
    { }

    NTSTATUS Start(DXGKRNL_INTERFACE* pDxgkInterface, const char* plat_name, UINT registryIndex);

    NTSTATUS Stop();

    int GetCachedEdid(PVOID *pEdid);

    struct platform_device* GetTransmitter()
    {
        return (&m_dsi_pdev);
    }

    struct drm_encoder* GetEncoder()
    {
        return nwl_dsi_get_encoder(&m_dsi_pdev);
    }

    struct i2c_client* GetBridge()
    {
        return &m_i2c_main;
    }
    BOOLEAN BridgeIsInitialized()
    {
        return m_i2c_main.is_initialized;
    }


    virtual NTSTATUS GetHotPlugDetectStatus(UINT8* status) override;

    virtual NTSTATUS GetEdid(PVOID Data, ULONG Length, UINT8 Block, UINT8 Segment) override;

    virtual void GetChildDescriptor(DXGK_CHILD_DESCRIPTOR* pDescriptor) override;

private:

    NTSTATUS GetI2CresourceNum(DXGKRNL_INTERFACE* pDxgkInterface,
        ULONG I2cIndex, LARGE_INTEGER* I2cConnectionId);

    NTSTATUS GetResourceNum(DXGKRNL_INTERFACE* pDxgkInterface, ULONG index, LARGE_INTEGER *connection_id, enum ResType restype);

    NTSTATUS GetRegistryParams(DXGKRNL_INTERFACE* pDxgkInterface, UINT registryIndex);
    CHAR* GetPrintableDispInterface();

    BYTE m_CachedEdid[EDID_SIZE];

    struct platform_device m_dsi_pdev;
    struct i2c_client m_i2c_main;
    struct i2c_client m_i2c_edid;
    struct i2c_client m_i2c_cec;

    struct platform_device m_panel_pdev;

    struct platform_device m_dsi_phy_pdev;
    UINT32 m_disp_interface;
    UCHAR m_num_lanes;
    UCHAR m_channel_id;

};
