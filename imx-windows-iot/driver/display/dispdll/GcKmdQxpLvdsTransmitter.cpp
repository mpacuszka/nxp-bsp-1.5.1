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
#include "GcKmdqxpLvdsTransmitter.tmh"
#include "GcKmdQxpLvdsTransmitter.h"
#include "getresrc.h"

extern "C" {
#include "boot/dts/freescale/board.h"
#include "lvds/imx8qxp-ldb.h"
#include "lvds/it6263.h"
#include <drm/drm_fourcc.h>
#include <linux/phy/phy-mixel-lvds-combo.h>
#include <edidtst.h>
}

#define printk(x, ...) \
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, __VA_ARGS__)

//#define LVDS_TRANSMITTER_DEBUG
#ifdef LVDS_TRANSMITTER_DEBUG
    #define printk_debug printk
#else
    #define printk_debug
#endif

// number of entries that are queried from registry
#define NUM_REG_ENTRIES 4

// LVDS interfaces read from registry
#define DISP_INTERFACE_LVDS0      0x4
#define DISP_INTERFACE_LVDS1      0x5
#define DISP_INTERFACE_LVDS_DUAL0 0x6
// Bus mappings read from registry
#define DISP_BUS_MAPPING_SPWG     0x1
#define DISP_BUS_MAPPING_JEIDA    0x2

static UCHAR* default_edid = (UCHAR*)edid_mon_1280_720_60;
// Use local default_edid defined above instead of registry edid
static const BOOLEAN lvds_transmitter_override_registry_edid = FALSE;

NTSTATUS QxpLvdsTransmitter::GetEdid(
    PVOID Data, ULONG Length, UINT8 Block, UINT8 Segment)
{
    if (Data == NULL)
    {
        return STATUS_INVALID_PARAMETER;
    }

    if (lvds_transmitter_override_registry_edid)
    {
        RtlCopyMemory(m_CachedEdid, default_edid, EDID_SIZE);
    }
    RtlCopyMemory(Data, m_CachedEdid, min(Length, EDID_SIZE));

    return STATUS_SUCCESS;
}

CHAR* QxpLvdsTransmitter::GetPrintableDispInterface(void)
{
    switch (m_disp_interface) {
    case IMX_LVDS0:
        return "LVDS0";
        break;
    case IMX_LVDS1:
        return "LVDS1";
        break;
    case IMX_LVDS0_DUAL:
        return "LVDS0_DUAL";
        break;
    default:
        return "UNKNOWN";
        break;
    }
}

void QxpLvdsTransmitter::GetChildDescriptor(DXGK_CHILD_DESCRIPTOR* pDescriptor)
{
    pDescriptor->ChildDeviceType = TypeVideoOutput;
    pDescriptor->ChildCapabilities.HpdAwareness =
        HpdAwarenessAlwaysConnected;
    pDescriptor->ChildCapabilities.Type.VideoOutput.InterfaceTechnology =
        D3DKMDT_VOT_LVDS;
    pDescriptor->ChildCapabilities.Type.VideoOutput.MonitorOrientationAwareness =
        D3DKMDT_MOA_NONE;
    pDescriptor->ChildCapabilities.Type.VideoOutput.SupportsSdtvModes = FALSE;
    // child device is not an ACPI device
    pDescriptor->AcpiUid = 0;
    switch (m_disp_interface) {
    case IMX_LVDS0:
    case IMX_LVDS0_DUAL:
    default:
        pDescriptor->ChildUid = LVDS0_CHILD_UID;
        break;
    case IMX_LVDS1:
        pDescriptor->ChildUid = LVDS1_CHILD_UID;
        break;
    }
}

RTL_QUERY_REGISTRY_ROUTINE QxpLvdsTransmitter_EDIDQueryRoutine;

_Use_decl_annotations_
NTSTATUS NTAPI QxpLvdsTransmitter_EDIDQueryRoutine(
    PWSTR ValueName, ULONG ValueType, PVOID ValueData,
    ULONG ValueLength, PVOID Context, PVOID EntryContext)
{
    if ((ValueType != REG_BINARY) || (ValueLength != EDID_SIZE) ||
        (EntryContext == nullptr) || ValueData == nullptr)
    {
        return STATUS_UNSUCCESSFUL;
    }

    // Validate first 8 bytes of EDID header
    for (UCHAR i = 0; i < 8; i++)
    {
        if (((UCHAR *)ValueData)[i] != default_edid[i])
        {
            return STATUS_UNSUCCESSFUL;
        }
    }

    RtlCopyMemory(EntryContext, ValueData, EDID_SIZE);
    return STATUS_SUCCESS;
}

NTSTATUS QxpLvdsTransmitter::GetRegistryParams(
    DXGKRNL_INTERFACE* pDxgkInterface, UINT registryIndex)
{
    NTSTATUS Status;
    ULONG TmpDispInterface;
    ULONG TmpDispBusMapping;
    ULONG TmpDispBusDataWidth;

    DXGK_DEVICE_INFO DeviceInfo;
    Status = pDxgkInterface->DxgkCbGetDeviceInformation(
        pDxgkInterface->DeviceHandle, &DeviceInfo);
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    // Convert registry path from unicode string to wide string
    // and ensure zero termination (__GFP_ZERO in kmalloc)
    USHORT Len = DeviceInfo.DeviceRegistryPath.Length;
    PUCHAR RegPath = (PUCHAR)kmalloc(Len + sizeof(WCHAR), __GFP_ZERO);
    if (!RegPath)
    {
        return STATUS_NO_MEMORY;
    }
    RtlCopyMemory(RegPath, DeviceInfo.DeviceRegistryPath.Buffer, Len);

    // Initialize query table - read 4 items from registry,
    // last item in query_table is zeroed
    RTL_QUERY_REGISTRY_TABLE QueryTable[NUM_REG_ENTRIES + 1];

    RtlZeroMemory(QueryTable, sizeof(QueryTable));
    QueryTable[0].Flags = RTL_QUERY_REGISTRY_DIRECT |
                        RTL_QUERY_REGISTRY_TYPECHECK;
    QueryTable[0].Name = GetRegParamString(RegParInterface, registryIndex);
    QueryTable[0].EntryContext = &TmpDispInterface;
    QueryTable[0].DefaultType =
        (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    QueryTable[1].Flags = RTL_QUERY_REGISTRY_DIRECT |
                        RTL_QUERY_REGISTRY_TYPECHECK;
    QueryTable[1].Name = GetRegParamString(RegParBusDataWidth, registryIndex);
    QueryTable[1].EntryContext = &TmpDispBusDataWidth;
    QueryTable[1].DefaultType =
        (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    QueryTable[2].Flags = RTL_QUERY_REGISTRY_DIRECT |
                        RTL_QUERY_REGISTRY_TYPECHECK;
    QueryTable[2].Name = GetRegParamString(RegParBusMapping, registryIndex);
    QueryTable[2].EntryContext = &TmpDispBusMapping;
    QueryTable[2].DefaultType =
        (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    QueryTable[3].Flags = 0;
    QueryTable[3].Name = GetRegParamString(RegParEdid, registryIndex);
    QueryTable[3].EntryContext = &m_CachedEdid;
    QueryTable[3].DefaultType = REG_BINARY;
    QueryTable[3].QueryRoutine = QxpLvdsTransmitter_EDIDQueryRoutine;
    QueryTable[3].DefaultData = NULL;
    QueryTable[3].DefaultLength = 0;

    Status = RtlQueryRegistryValues(RTL_REGISTRY_ABSOLUTE, (PCWSTR)RegPath,
        QueryTable, nullptr, nullptr);
    if (!NT_SUCCESS(Status))
    {
        // Fall back with default values
        TmpDispInterface = DISP_INTERFACE_LVDS0;
        TmpDispBusDataWidth = 24;
        TmpDispBusMapping = DISP_BUS_MAPPING_SPWG;
        RtlCopyMemory(m_CachedEdid, default_edid, EDID_SIZE);
        printk("LVDS display: ERROR reading registry parameters failed.\n");
    }
    kfree(RegPath);

    // Check validity of REG_DWORD parameters.
    // REG_BINARY is validated inside the QueryRoutine */
    switch (TmpDispInterface) {
    case DISP_INTERFACE_LVDS0:
        m_disp_interface = IMX_LVDS0;
        break;
    case DISP_INTERFACE_LVDS1:
        m_disp_interface = IMX_LVDS1;
        break;
    case DISP_INTERFACE_LVDS_DUAL0:
        m_disp_interface = IMX_LVDS0_DUAL;
        break;
    default:
        m_disp_interface = IMX_LVDS0;
        Status = STATUS_INVALID_PARAMETER;
        break;
    }

    switch (TmpDispBusMapping) {
    case DISP_BUS_MAPPING_SPWG:
        m_bus_mapping = "spwg";
        break;
    case DISP_BUS_MAPPING_JEIDA:
        m_bus_mapping = "jeida";
        break;
    default:
        m_bus_mapping = "spwg";
        Status = STATUS_INVALID_PARAMETER;
        break;
    }

    if ((TmpDispBusDataWidth != 18) && (TmpDispBusDataWidth != 24))
    {
        m_bus_data_width = 24;
        Status = STATUS_INVALID_PARAMETER;
    }
    else
    {
        m_bus_data_width = TmpDispBusDataWidth;
    }

    return Status;
}

NTSTATUS QxpLvdsTransmitter::Start(DXGKRNL_INTERFACE* pDxgkInterface, UINT registryIndex)
{
    NTSTATUS Status;

    // currently we are taking LVDS parameters from m_ldb_pdev DT node;
    // use below function to override DT setting
    Status = GetRegistryParams(pDxgkInterface, registryIndex);
    if (!NT_SUCCESS(Status)) {
        printk_debug("LVDS display: WARNING parameters not valid."
            "Switching to default settings:"
            "interface=%s bus_data_width=%d bus_mapping=%s\n",
            GetPrintableDispInterface(), m_bus_data_width, m_bus_mapping);
    }
    printk_debug("LVDS display:"
            "interface=%s bus_data_width=%d bus_mapping=%s\n",
        GetPrintableDispInterface(), m_bus_data_width, m_bus_mapping);

    if (m_disp_interface == IMX_LVDS0 || m_disp_interface == IMX_LVDS0_DUAL) {
        m_ldb_phy_pdev.name = "ldb1_phy";
        m_ldb_pdev.name = "ldb1";
        ldb_prop_init(1, m_bus_data_width, m_bus_mapping);
    }
    else if (m_disp_interface == IMX_LVDS1) {
        m_ldb_phy_pdev.name = "ldb2_phy";
        m_ldb_pdev.name = "ldb2";
        ldb_prop_init(2, m_bus_data_width, m_bus_mapping);
    }
    else {
        return STATUS_INTERNAL_ERROR;
    }
    m_ldb_phy_pdev.plat_name = "qxp";
    m_ldb_phy_pdev.data = pDxgkInterface;
    board_init(&m_ldb_phy_pdev);

    m_ldb_pdev.plat_name = "qxp";
    m_ldb_pdev.data = pDxgkInterface;
    board_init(&m_ldb_pdev);

    if (mixel_lvds_combo_phy_probe(&m_ldb_phy_pdev))
    {
        mixel_lvds_combo_phy_remove(&m_ldb_phy_pdev, false);
        return STATUS_INTERNAL_ERROR;
    }

    // dual channel is not supported; passing null for aux-ldb, aux-phy
    if (imx8qxp_ldb_probe(&m_ldb_pdev, nullptr,
        &m_ldb_phy_pdev, nullptr))
    {
        imx8qxp_ldb_remove(&m_ldb_pdev, nullptr, false);
        mixel_lvds_combo_phy_remove(&m_ldb_phy_pdev, false);
        return STATUS_INTERNAL_ERROR;
    }

    // LVDS-HDMI converter initialization is optional
    do {
        // Both i2c_hdmi device and i2c_lvds devices refer to LVDS-HDMI
        // converter extension board it6263.
        // There are two memory blocks with separate bus address.
        // m_i2c_hdmi is the main device with allocated driver data.
        // m_i2c_lvds is auxiliary - only for i2c regmap storage.
        //
        // i2c_hdmi converter device, ACPI i2c index #1 for LVDS0, #3 for LVDS1
        ULONG i2c_hdmi_index = 1;
        // i2c_lvds converter device, ACPI i2c index #2 for LVDS0, #4 for LVDS1
        ULONG i2c_lvds_index = 2;
        if (m_disp_interface == IMX_LVDS1) {
            i2c_hdmi_index = 3;
            i2c_lvds_index = 4;
        }
        m_i2c_hdmi.is_initialized = 0;
        Status = GetI2CresourceNum(pDxgkInterface,
            i2c_hdmi_index, &m_i2c_hdmi.connection_id);
        if (!NT_SUCCESS(Status))
        {
            break;
        }
        Status = GetI2CresourceNum(pDxgkInterface,
            i2c_lvds_index, &m_i2c_lvds.connection_id);
        if (!NT_SUCCESS(Status))
        {
            break;
        }
        if (it6263_probe(&m_i2c_hdmi, &m_i2c_lvds, false,
                imx8qxp_ldb_get_bus_format(&m_ldb_pdev, 0), false))
        {
            it6263_remove(&m_i2c_hdmi);
            break;
        }
        m_i2c_hdmi.is_initialized = 1;
        printk_debug("LVDS display: IT6263 lvds-hdmi converter initialized.\n");
    } while (0);

    return STATUS_SUCCESS;
}

NTSTATUS QxpLvdsTransmitter::Stop()
{
    // dual channel is not supported; passing null for aux-ldb
    imx8qxp_ldb_remove(&m_ldb_pdev, nullptr, false);
    mixel_lvds_combo_phy_remove(&m_ldb_phy_pdev, false);
    if (m_i2c_hdmi.is_initialized)
    {
        it6263_remove(&m_i2c_hdmi);
    }

    return STATUS_SUCCESS;
}

int QxpLvdsTransmitter::GetCachedEdid(PVOID *pEdid)
{
    *pEdid = m_CachedEdid;
    return sizeof(m_CachedEdid);
}

NTSTATUS QxpLvdsTransmitter::GetI2CresourceNum(
    DXGKRNL_INTERFACE* pDxgkInterface, ULONG I2cIndex,
    LARGE_INTEGER* I2cConnectionId)
{
    NTSTATUS Status;
    WCHAR Buff[512];

    Status = GetReslist(pDxgkInterface, (PWCHAR)&Buff, sizeof(Buff));
    if (!NT_SUCCESS(Status))
    {
        printk("LVDS display: Error getting resource list\n");
        return Status;
    }

    Status = ParseReslist((PCM_RESOURCE_LIST)&Buff,
        CmResourceTypeConnection, I2cConnectionId, NULL, I2cIndex, ResourceType_I2C);
    if (!NT_SUCCESS(Status))
    {
        printk("LVDS display: Error parsing resource list\n");
        return Status;
    }

    printk_debug("LVDS display: I2C resource found with connection id: 0x%llx\n",
        I2cConnectionId->QuadPart);

    return STATUS_SUCCESS;
}
