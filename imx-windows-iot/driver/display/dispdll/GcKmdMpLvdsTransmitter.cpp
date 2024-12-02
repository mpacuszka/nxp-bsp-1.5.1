/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Copyright 2022-2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. THE SOFTWARE IS PROVIDED "AS IS",
 * WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************
 */

#include "precomp.h"
#include "GcKmdLogging.h"
#include "GcKmdMpLvdsTransmitter.tmh"
#include "GcKmdMpLvdsTransmitter.h"
#include "getresrc.h"

extern "C" {
#include "boot/dts/freescale/board.h"
#include "lvds/imx8mp-ldb.h"
#include "lvds/it6263.h"
#include <drm/drm_fourcc.h>
#include <edidtst.h>
}

#define printk(x, ...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, __VA_ARGS__)

/* #define LVDS_TRANSMITTER_DEBUG */
#ifdef LVDS_TRANSMITTER_DEBUG
    #define printk_debug printk
#else
    #define printk_debug
#endif

static UCHAR *default_edid = (UCHAR *)edid_mon_1280_720_60;

/*number of entries that are queried from registry */
#define NUM_REG_ENTRIES 4

/*LVDS interfaces read from registry */
#define DISP_INTERFACE_LVDS0      0x4
#define DISP_INTERFACE_LVDS1      0x5
#define DISP_INTERFACE_LVDS_DUAL0 0x6
/*Bus mappings read from registry */
#define DISP_BUS_MAPPING_SPWG     0x1
#define DISP_BUS_MAPPING_JEIDA    0x2

/* Use local default_edid defined above instead of registry edid */
static const bool lvds_transmitter_override_registry_edid = FALSE;

NTSTATUS MpLvdsTransmitter::GetEdid(PVOID Data, ULONG Length, UINT8 Block, UINT8 Segment)
{
    if (Data == NULL) {
        return STATUS_INVALID_PARAMETER;
    }

    if (lvds_transmitter_override_registry_edid) {
        RtlCopyMemory(m_CachedEdid, default_edid, EDID_SIZE);
    }
    RtlCopyMemory(Data, m_CachedEdid, min(Length, EDID_SIZE));

    return STATUS_SUCCESS;
}

CHAR* MpLvdsTransmitter::GetPrintableDispInterface(void)
{
    switch (m_disp_interface) {
    case imx_lvds0:
        return "lvds0";
        break;
    case imx_lvds1:
        return "lvds1";
        break;
    case imx_lvds0_dual:
        return "lvds0_dual";
        break;
    default:
        return "unknown";
        break;
    }
}

void MpLvdsTransmitter::GetChildDescriptor(DXGK_CHILD_DESCRIPTOR* pDescriptor)
{
    pDescriptor->ChildDeviceType = TypeVideoOutput;
    pDescriptor->ChildCapabilities.HpdAwareness = HpdAwarenessAlwaysConnected;
    pDescriptor->ChildCapabilities.Type.VideoOutput.InterfaceTechnology = D3DKMDT_VOT_LVDS;
    pDescriptor->ChildCapabilities.Type.VideoOutput.MonitorOrientationAwareness = D3DKMDT_MOA_NONE;
    pDescriptor->ChildCapabilities.Type.VideoOutput.SupportsSdtvModes = FALSE;
    /* child device is not an ACPI device */
    pDescriptor->AcpiUid = 0;
    pDescriptor->ChildUid = LVDS0_CHILD_UID;
}

RTL_QUERY_REGISTRY_ROUTINE MpLvdsTransmitter_EDIDQueryRoutine;

_Use_decl_annotations_
NTSTATUS NTAPI MpLvdsTransmitter_EDIDQueryRoutine(PWSTR ValueName, ULONG ValueType, PVOID ValueData,
    ULONG ValueLength, PVOID Context, PVOID EntryContext)
{
    if ((ValueType != REG_BINARY) || (ValueLength != EDID_SIZE) ||
        (EntryContext == nullptr) || ValueData == nullptr)
    {
        return STATUS_UNSUCCESSFUL;
    }

    /* Validate first 8 bytes of EDID header */
    BOOLEAN check_ok = TRUE;
    for (UCHAR i = 0; i < 8; i++) {
        if (*(((UCHAR *)ValueData)+i) != *(default_edid + i)) {
            check_ok = FALSE;
            break;
        }
    }
    if (!check_ok) {
        return STATUS_UNSUCCESSFUL;
    }

    RtlCopyMemory(EntryContext, ValueData, EDID_SIZE);
    return STATUS_SUCCESS;
}

NTSTATUS MpLvdsTransmitter::GetRegistryParams(DXGKRNL_INTERFACE* pDxgkInterface, UINT registryIndex)
{
    NTSTATUS status;
    DXGK_DEVICE_INFO device_info;
    /* query_table must be number of items + 1 (last item must be zero) */
    RTL_QUERY_REGISTRY_TABLE query_table[NUM_REG_ENTRIES + 1];
    USHORT len;
    PUCHAR reg_path;
    ULONG tmp_disp_interface;
    ULONG tmp_disp_bus_mapping;
    ULONG tmp_disp_bus_data_width;

    status = pDxgkInterface->DxgkCbGetDeviceInformation(pDxgkInterface->DeviceHandle, &device_info);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    /* Convert registry path from unicode string to wide string and ensure zero termination (__GFP_ZERO in kmalloc) */
    len = device_info.DeviceRegistryPath.Length;
    reg_path = (PUCHAR)kmalloc(len + sizeof(WCHAR), __GFP_ZERO);
    if (reg_path == NULL) {
        return STATUS_NO_MEMORY;
    }
    RtlCopyMemory(reg_path, device_info.DeviceRegistryPath.Buffer, len);

    /* Initialize query table - read 4 items from registry, last item in query_table is zeroed */
    RtlZeroMemory(query_table, sizeof(query_table));
    query_table[0].Flags = RTL_QUERY_REGISTRY_DIRECT | RTL_QUERY_REGISTRY_TYPECHECK;
    query_table[0].Name = GetRegParamString(RegParInterface, registryIndex);
    query_table[0].EntryContext = &tmp_disp_interface;
    query_table[0].DefaultType = (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    query_table[1].Flags = RTL_QUERY_REGISTRY_DIRECT | RTL_QUERY_REGISTRY_TYPECHECK;
    query_table[1].Name = GetRegParamString(RegParBusDataWidth, registryIndex);
    query_table[1].EntryContext = &tmp_disp_bus_data_width;
    query_table[1].DefaultType = (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    query_table[2].Flags = RTL_QUERY_REGISTRY_DIRECT | RTL_QUERY_REGISTRY_TYPECHECK;
    query_table[2].Name = GetRegParamString(RegParBusMapping, registryIndex);
    query_table[2].EntryContext = &tmp_disp_bus_mapping;
    query_table[2].DefaultType = (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    query_table[3].Flags = 0;
    query_table[3].Name = GetRegParamString(RegParEdid, registryIndex);
    query_table[3].EntryContext = &m_CachedEdid;
    query_table[3].DefaultType = REG_BINARY;
    query_table[3].QueryRoutine = MpLvdsTransmitter_EDIDQueryRoutine;
    query_table[3].DefaultData = NULL;
    query_table[3].DefaultLength = 0;
    status = RtlQueryRegistryValues(RTL_REGISTRY_ABSOLUTE, (PCWSTR)reg_path, query_table, NULL, NULL);
    if (!NT_SUCCESS(status)) {
        /* Fall back with default values */
        tmp_disp_interface = DISP_INTERFACE_LVDS0;
        tmp_disp_bus_data_width = 24;
        tmp_disp_bus_mapping = DISP_BUS_MAPPING_SPWG;
        RtlCopyMemory(m_CachedEdid, default_edid, EDID_SIZE);
        printk("LVDS display: ERROR reading registry parameters failed.\n");
    }
    kfree(reg_path);

    /* Check validity of REG_DWORD parameters. REG_BINARY is validated inside the QueryRoutine */
    switch (tmp_disp_interface) {
    case DISP_INTERFACE_LVDS0:
        m_disp_interface = imx_lvds0;
        break;
    case DISP_INTERFACE_LVDS1:
        m_disp_interface = imx_lvds1;
        break;
    case DISP_INTERFACE_LVDS_DUAL0:
        m_disp_interface = imx_lvds0_dual;
        break;
    default:
        m_disp_interface = imx_lvds0;
        status = STATUS_INVALID_PARAMETER;
        break;
    }
    switch (tmp_disp_bus_mapping) {
    case DISP_BUS_MAPPING_SPWG:
        m_bus_mapping = "spwg";
        break;
    case DISP_BUS_MAPPING_JEIDA:
        m_bus_mapping = "jeida";
        break;
    default:
        m_bus_mapping = "spwg";
        status = STATUS_INVALID_PARAMETER;
        break;
    }
    if ((tmp_disp_bus_data_width != 18) && (tmp_disp_bus_data_width != 24)) {
        m_bus_data_width = 24;
        status = STATUS_INVALID_PARAMETER;
    } else {
        m_bus_data_width = tmp_disp_bus_data_width;
    }

    return status;
}

NTSTATUS MpLvdsTransmitter::Start(DXGKRNL_INTERFACE* pDxgkInterface, UINT registryIndex)
{
    NTSTATUS status;

    status = GetRegistryParams(pDxgkInterface, registryIndex);
    if (!NT_SUCCESS(status)) {
        printk("LVDS display: WARNING parameters not valid. Switching to default settings: interface=%s bus_data_width=%d bus_mapping=%s\n",
            GetPrintableDispInterface(), m_bus_data_width, m_bus_mapping);
    }
    printk_debug("LVDS display: interface=%s bus_data_width=%d bus_mapping=%s\n",
        GetPrintableDispInterface(), m_bus_data_width, m_bus_mapping);

    lvds_phy_pdev.name = "lvds_phy";
    lvds_phy_pdev.plat_name = "mp";
    board_init(&lvds_phy_pdev);
    if (imx8mp_lvds_phy_probe(&lvds_phy_pdev) != 0) {
        imx8mp_lvds_phy_remove(&lvds_phy_pdev);
        printk("LVDS display: ERROR imx8mp_lvds_phy_probe failed.\n");
        return STATUS_INTERNAL_ERROR;
    }

    lvds_pdev.name = "lvds_dev";
    lvds_pdev.plat_name = "mp";
    board_init(&lvds_pdev);
    if (imx8mp_ldb_probe(&lvds_pdev) != 0) {
        imx8mp_ldb_remove(&lvds_pdev);
        imx8mp_lvds_phy_remove(&lvds_phy_pdev);
        printk("LVDS display: ERROR imx8mp_ldb_probe failed.\n");
        return STATUS_INTERNAL_ERROR;
    }

    /* Set lvds phy as a parent of lvds device */
    lvds_pdev.dev.parent = &lvds_phy_pdev.dev;

    if (imx8mp_ldb_bind(&lvds_pdev.dev, m_bus_data_width, m_bus_mapping, m_disp_interface) != 0) {
        imx8mp_ldb_unbind(&lvds_pdev.dev);
        imx8mp_ldb_remove(&lvds_pdev);
        imx8mp_lvds_phy_remove(&lvds_phy_pdev);
        printk("LVDS display: ERROR imx8mp_ldb_bind failed.\n");
        return STATUS_INTERNAL_ERROR;
    }

    /* LVDS-HDMI converter initialization is optional */
    do {
        /* Both i2c_hdmi device and i2c_lvds devices refer to LVDS-HDMI converter extension board it6263.
           There are two memory blocks with separate bus address.
           m_i2c_hdmi is the main device with allocated driver data. m_i2c_lvds is auxiliary - only for i2c regmap storage */
        m_i2c_hdmi.is_initialized = 0;
        /* i2c_hdmi converter device, ACPI i2c index 0 (first i2c device) */
        status = GetI2CresourceNum(pDxgkInterface, 0, &m_i2c_hdmi.connection_id);
        if (!NT_SUCCESS(status)) {
            break;
        }
        /* i2c_lvds converter device, ACPI i2c index 1 (second i2c device) */
        status = GetI2CresourceNum(pDxgkInterface, 1, &m_i2c_lvds.connection_id);
        if (!NT_SUCCESS(status)) {
            break;
        }
        /* imx8mp_ldb_bind must have been already called to initialize bus format */
        if (it6263_probe(&m_i2c_hdmi, &m_i2c_lvds, (m_disp_interface == imx_lvds0_dual),
                         imx8mp_ldb_encoder_get_ldb_bus_format(&lvds_pdev), true) != 0) {
            it6263_remove(&m_i2c_hdmi);
            break;
        }
        m_i2c_hdmi.is_initialized = 1;
        printk_debug("LVDS display: IT6263 lvds-hdmi converter initialized.\n");
    } while (0);

    return STATUS_SUCCESS;
}

NTSTATUS MpLvdsTransmitter::Stop()
{
    /* To full stop the hardware, call: imx8mp_ldb_encoder_disable(&lvds_pdev); */
    /* To full stop the hardware, if (m_i2c_hdmi.is_initialized), call: it6263_bridge_disable(&m_i2c_hdmi); */
    /* To full stop the hardware, call: imx8mp_ldb_unbind(&lvds_pdev.dev); */

    imx8mp_ldb_remove(&lvds_pdev);
    imx8mp_lvds_phy_remove(&lvds_phy_pdev);
    if (m_i2c_hdmi.is_initialized) {
        it6263_remove(&m_i2c_hdmi);
    }

    return STATUS_SUCCESS;
}

int MpLvdsTransmitter::GetCachedEdid(PVOID *pEdid)
{
    *pEdid = m_CachedEdid;
    return sizeof(m_CachedEdid);
}

NTSTATUS MpLvdsTransmitter::GetHotPlugDetectStatus(UINT8* status)
{
    /* Display is always connected */
    *status = 1;
    return STATUS_SUCCESS;
}

NTSTATUS MpLvdsTransmitter::GetI2CresourceNum(DXGKRNL_INTERFACE* pDxgkInterface, ULONG i2c_index, LARGE_INTEGER *i2c_connection_id)
{
    NTSTATUS status;
    WCHAR   buff[512];

    status = GetReslist(pDxgkInterface, (PWCHAR)&buff, sizeof(buff));
    if (!NT_SUCCESS(status)) {
        printk("LVDS display: Error getting resource list\n");
        return status;
    }

    status = ParseReslist((PCM_RESOURCE_LIST)&buff, CmResourceTypeConnection, i2c_connection_id, NULL, i2c_index, ResourceType_I2C);
    if (!NT_SUCCESS(status)) {
        printk("LVDS display: Error parsing resource list\n");
        return status;
    }

    printk_debug("LVDS display: I2C resource found with connection id: 0x%llx\n", i2c_connection_id->QuadPart);
    return STATUS_SUCCESS;
}
