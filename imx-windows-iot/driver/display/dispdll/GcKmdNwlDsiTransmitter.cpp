/* Copyright (c) Microsoft Corporation.
 * Copyright 2023-2024 NXP
   Licensed under the MIT License. */

#include "precomp.h"
#include "GcKmdLogging.h"
#include "GcKmdNwlDsiTransmitter.tmh"
#include "GcKmdNwlDsiTransmitter.h"
#include "getresrc.h"

extern "C" {
#include "boot/dts/freescale/board.h"
#include "mipi_dsi/adv7511.h"
#include <drm/drm_fourcc.h>
#include <edidtst.h>
#include "mipi_dsi/panel-raydium-rm67191.h"
#include <linux/gpio/consumer.h>
}

#define printk(x, ...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, __VA_ARGS__)

/* #define NWLDSI_TRANSMITTER_DEBUG */
#ifdef NWLDSI_TRANSMITTER_DEBUG
    #define printk_debug printk
#else
    #define printk_debug
#endif

/*number of entries that are queried from registry */
#define NUM_REG_ENTRIES 4

/* MIPI interfaces read from registry */
#define DISP_INTERFACE_MIPI_DSI0 0x2
#define DISP_INTERFACE_MIPI_DSI1 0x3

static UCHAR *nwl_dsi_default_edid = (UCHAR *)edid_mon_1920_1080_60;

/* Use local default_edid defined above instead of registry edid */
static const bool nwldsi_transmitter_override_registry_edid = FALSE;

NTSTATUS NwlDsiTransmitter::GetEdid(PVOID Data, ULONG Length, UINT8 Block, UINT8 Segment)
{
    if (Data == NULL) {
        return STATUS_INVALID_PARAMETER;
    }

    if (nwldsi_transmitter_override_registry_edid) {
        RtlCopyMemory(m_CachedEdid, nwl_dsi_default_edid, EDID_SIZE);
    }
    RtlCopyMemory(Data, m_CachedEdid, min(Length, EDID_SIZE));

    return STATUS_SUCCESS;
}

CHAR* NwlDsiTransmitter::GetPrintableDispInterface(void)
{
    switch (m_disp_interface) {
    case nwl_dsi0:
        return "MIPI-DSI0";
        break;
    case nwl_dsi1:
        return "MIPI-DSI1";
        break;
    default:
        return "UNKNOWN";
        break;
    }
}

void NwlDsiTransmitter::GetChildDescriptor(DXGK_CHILD_DESCRIPTOR* pDescriptor)
{
    pDescriptor->ChildDeviceType = TypeVideoOutput;
    pDescriptor->ChildCapabilities.HpdAwareness = HpdAwarenessAlwaysConnected;
    pDescriptor->ChildCapabilities.Type.VideoOutput.InterfaceTechnology = D3DKMDT_VOT_LVDS;
    pDescriptor->ChildCapabilities.Type.VideoOutput.MonitorOrientationAwareness = D3DKMDT_MOA_NONE;
    pDescriptor->ChildCapabilities.Type.VideoOutput.SupportsSdtvModes = FALSE;
    /* child device is not an ACPI device */
    pDescriptor->AcpiUid = 0;
    switch (m_disp_interface) {
    case nwl_dsi0:
    default:
        pDescriptor->ChildUid = MIPI_DSI0_CHILD_UID;
        break;
    case nwl_dsi1:
        pDescriptor->ChildUid = MIPI_DSI1_CHILD_UID;
        break;
    }
}

RTL_QUERY_REGISTRY_ROUTINE NwlDsiTransmitter_EDIDQueryRoutine;

_Use_decl_annotations_
NTSTATUS NwlDsiTransmitter_EDIDQueryRoutine(PWSTR ValueName, ULONG ValueType, PVOID ValueData,
    ULONG ValueLength, PVOID Context, PVOID EntryContext)
{
    if ((ValueType != REG_BINARY) || (ValueLength != EDID_SIZE) || 
        (EntryContext == NULL) || (ValueData == NULL)) {
        return STATUS_UNSUCCESSFUL;
    }

    /* Validate first 8 bytes of EDID header */
    for (UCHAR i = 0; i < 8; i++) {
        if (*(((UCHAR *)ValueData)+i) != *(nwl_dsi_default_edid + i)) {
            return STATUS_UNSUCCESSFUL;
        }
    }

    RtlCopyMemory(EntryContext, ValueData, EDID_SIZE);
    return STATUS_SUCCESS;
}

NTSTATUS NwlDsiTransmitter::GetRegistryParams(DXGKRNL_INTERFACE* pDxgkInterface, UINT registryIndex)
{
    NTSTATUS Status;
    DXGK_DEVICE_INFO DeviceInfo;
    /* QueryTable must be number of items + 1 (last item must be zero) */
    RTL_QUERY_REGISTRY_TABLE QueryTable[NUM_REG_ENTRIES + 1];
    USHORT Len;
    PUCHAR RegPath;
    ULONG TmpDispInterface;    
    ULONG TmpNumLanes;
    ULONG TmpChannelId;

    Status = pDxgkInterface->DxgkCbGetDeviceInformation(pDxgkInterface->DeviceHandle, &DeviceInfo);
    if (!NT_SUCCESS(Status)) {
        return Status;
    }

    /* Convert registry path from unicode string to wide string and ensure zero termination (__GFP_ZERO in kmalloc) */
    Len = DeviceInfo.DeviceRegistryPath.Length;
    RegPath = (PUCHAR)kmalloc(Len + sizeof(WCHAR), __GFP_ZERO);
    if (RegPath == NULL) {
        return STATUS_NO_MEMORY;
    }
    RtlCopyMemory(RegPath, DeviceInfo.DeviceRegistryPath.Buffer, Len);

    /* Initialize query table - read 4 items from registry, last item in QueryTable is zeroed */
    RtlZeroMemory(QueryTable, sizeof(QueryTable));
    QueryTable[0].Flags = RTL_QUERY_REGISTRY_DIRECT | RTL_QUERY_REGISTRY_TYPECHECK;
    QueryTable[0].Name = GetRegParamString(RegParNumLanes, registryIndex);
    QueryTable[0].EntryContext = &TmpNumLanes;
    QueryTable[0].DefaultType = (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    QueryTable[1].Flags = RTL_QUERY_REGISTRY_DIRECT | RTL_QUERY_REGISTRY_TYPECHECK;
    QueryTable[1].Name = GetRegParamString(RegParChannelId, registryIndex);
    QueryTable[1].EntryContext = &TmpChannelId;
    QueryTable[1].DefaultType = (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    QueryTable[2].Flags = RTL_QUERY_REGISTRY_DIRECT | RTL_QUERY_REGISTRY_TYPECHECK;
    QueryTable[2].Name = GetRegParamString(RegParInterface, registryIndex);
    QueryTable[2].EntryContext = &TmpDispInterface;
    QueryTable[2].DefaultType = (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    QueryTable[3].Flags = 0;
    QueryTable[3].Name = GetRegParamString(RegParEdid, registryIndex);
    QueryTable[3].EntryContext = &m_CachedEdid;
    QueryTable[3].DefaultType = REG_BINARY;
    QueryTable[3].QueryRoutine = NwlDsiTransmitter_EDIDQueryRoutine;
    QueryTable[3].DefaultData = NULL;
    QueryTable[3].DefaultLength = 0;

    Status = RtlQueryRegistryValues(RTL_REGISTRY_ABSOLUTE, (PCWSTR)RegPath, QueryTable, NULL, NULL);
    if (!NT_SUCCESS(Status)) {
        /* Fall back with default values */
        TmpDispInterface = DISP_INTERFACE_MIPI_DSI0;
        TmpNumLanes = 4;
        TmpChannelId = 0;
        RtlCopyMemory(m_CachedEdid, nwl_dsi_default_edid, EDID_SIZE);
        printk("NwlDsi display: ERROR reading registry parameters.\n");
    }
    kfree(RegPath);
    switch (TmpDispInterface) {
    case DISP_INTERFACE_MIPI_DSI0:
        m_disp_interface = nwl_dsi0;
        break;
    case DISP_INTERFACE_MIPI_DSI1:
        m_disp_interface = nwl_dsi1;
        break;
    default:
        m_disp_interface = nwl_dsi0;
        Status = STATUS_INVALID_PARAMETER;
        break;
    }

    if ((TmpNumLanes < 1) || (TmpNumLanes > 4)) {
        m_num_lanes = 4;
        Status = STATUS_INVALID_PARAMETER;
    } else {
        m_num_lanes = (UCHAR)TmpNumLanes;
    }
    if (TmpChannelId != 0) {
        m_channel_id = 0;
        Status = STATUS_INVALID_PARAMETER;
    } else {
        m_channel_id = (UCHAR)TmpChannelId;
    }

    return Status;
}

static property adv_properties[] = {
    { "compatible", 1, "adi,adv7535" },
    { "" /* mark end of the list */ }
};

/* IMX-DSI-OLED1 panel connected to MIPI_DSI0 */
static LONG cid0[5];
static int instance0 = 1;
static property panel_raydium_properties0[] = {
    { "compatible", 1, "raydium,rm67191" },
    { "instance", 1, &instance0 },
    /* MIPI_DSI_EN reset pin: pin name, number of params,
       gpio connection_id LowPart=cid[0] HighPart=cid[1] PinType=cid[2] ExpanderDataReg=cid[3] ExpanderPinMask=cid[4] */
    { "reset", 5, &cid0 },
    { "" /* mark end of the list */ }
};

/* IMX-DSI-OLED1 panel connected to MIPI_DSI1 */
static LONG cid1[5];
static int instance1 = 2;
static property panel_raydium_properties1[] = {
    { "compatible", 1, "raydium,rm67191" },
    { "instance", 1, &instance1 },
    /* MIPI_DSI_EN reset pin: pin name, number of params,
       gpio connection_id LowPart=cid[0] HighPart=cid[1] PinType=cid[2] ExpanderDataReg=cid[3] ExpanderPinMask=cid[4] */
    { "reset", 5, &cid1 },
    { "" /* mark end of the list */ }
};

NTSTATUS NwlDsiTransmitter::Start(DXGKRNL_INTERFACE* pDxgkInterface, const char* plat_name, UINT registryIndex)
{
    NTSTATUS status;

    status = GetRegistryParams(pDxgkInterface, registryIndex);
    if (!NT_SUCCESS(status)) {
        printk("NwlDsi display: WARNING parameters not valid. Switching to default settings: interface=%s num_lanes=%d channel_id=%d.\n",
            GetPrintableDispInterface(), m_num_lanes, m_channel_id);
    }
    printk_debug("NwlDsi display: interface=%s num_lanes=%d channel_id=%d.\n",
        GetPrintableDispInterface(), m_num_lanes, m_channel_id);

    if (m_disp_interface == nwl_dsi0) {
        m_dsi_phy_pdev.name = "mipi0_dphy";
        m_dsi_pdev.name = "mipi0_dsi_host";
    }
    else if (m_disp_interface == nwl_dsi1) {
        m_dsi_phy_pdev.name = "mipi1_dphy";
        m_dsi_pdev.name = "mipi1_dsi_host";
    }
    else {
        return STATUS_INTERNAL_ERROR;
    }

    m_dsi_phy_pdev.plat_name = plat_name;
    m_dsi_pdev.plat_name = plat_name;
    board_init(&m_dsi_phy_pdev);
    board_init(&m_dsi_pdev);
    if (nwl_dsi_probe(&m_dsi_pdev, &m_dsi_phy_pdev) != 0) {
        nwl_dsi_remove(&m_dsi_pdev);
        printk("NwlDsi display: ERROR nwl_dsi_probe failed.\n");
        return STATUS_INTERNAL_ERROR;
    }
    /* Need to have nwl_dsi_probe initialized first, to allocate regmap of dsi, then call mixel_dphy_probe. 
       The phy address is not page aligned, so we can't alocate regmap for phy directly and need
       to use pre-alocated regmap and pass it to phy device. */
    if (mixel_dphy_probe(&m_dsi_phy_pdev, nwl_dsi_get_regmap(&m_dsi_pdev)) != 0) {
        mixel_dphy_remove(&m_dsi_phy_pdev);
        nwl_dsi_remove(&m_dsi_pdev);
        printk("NwlDsi display: ERROR mixel_dphy_probe failed.\n");
        return STATUS_INTERNAL_ERROR;
    }

    if (nwl_dsi_bridge_attach(&m_dsi_pdev) != 0) {
        nwl_dsi_bridge_detach(&m_dsi_pdev);
        mixel_dphy_remove(&m_dsi_phy_pdev);
        nwl_dsi_remove(&m_dsi_pdev);
        printk("NwlDsi display: ERROR nwl_dsi_bridge_attach failed.\n");
        return STATUS_INTERNAL_ERROR;
    }

    /* MIPI-HDMI converter initialization is optional */
    do {
        /* All i2c_main, i2c_edid, i2c_cec, i2c_packet devices refer to MIPI-HDMI converter extension board adv7535.
           There are four memory blocks with separate bus address.
           m_i2c_main is the main device with allocated driver data. others are auxiliary - only for i2c regmap storage */
        m_i2c_main.is_initialized = 0;
        /* Offset of i2c device in ACPI, for QXP, 5 for dsi0 and 8 for dsi1 */
        UINT acpi_i2c_offset = 5;
        if (m_disp_interface == nwl_dsi1) {
            acpi_i2c_offset = 8;
        }
        /* i2c_main device, ACPI i2c index 0 (first i2c device) */
        status = GetResourceNum(pDxgkInterface, acpi_i2c_offset + 0, &m_i2c_main.connection_id, ResourceType_I2C);
        if (!NT_SUCCESS(status)) {
            break;
        }
        /* i2c_cec device, ACPI i2c index 1 (second i2c device) */
        status = GetResourceNum(pDxgkInterface, acpi_i2c_offset + 1, &m_i2c_cec.connection_id, ResourceType_I2C);
        if (!NT_SUCCESS(status)) {
            break;
        }
        /* i2c_edid device, ACPI i2c index 2 (third i2c device) */
        status = GetResourceNum(pDxgkInterface, acpi_i2c_offset + 2, &m_i2c_edid.connection_id, ResourceType_I2C);
        if (!NT_SUCCESS(status)) {
            break;
        }

        m_i2c_main.dev.of_node.properties = (property *)&adv_properties;
        m_i2c_main.dev.parent = &m_dsi_pdev.dev;
        if (adv7511_probe(&m_i2c_main, &m_i2c_edid, &m_i2c_cec, m_num_lanes, m_channel_id) != 0) {
            adv7511_remove(&m_i2c_main);
            break;
        }
        if (adv7511_bridge_attach(&m_i2c_main) != 0) {
            adv7511_bridge_detach(&m_i2c_main);
            adv7511_remove(&m_i2c_main);
            break;
        }
        m_i2c_main.is_initialized = 1;
        printk_debug("NwlDsi display: adv7535 mipi-hdmi converter initialized.\n");
    } while (0);
    if (!m_i2c_main.is_initialized) {
        LARGE_INTEGER m_i2c_connection_id = {0};  /* GPIO connection id for panel reset pin */
        /* Offset of i2c device for GPIO expander in ACPI, for QXP MEK, 11 for dsi0 (U25 PCA9557) and 12 for dsi1 (U187 PCA9557) */
        UINT acpi_i2c_offset = 11;
        if (m_disp_interface == nwl_dsi1) {
            acpi_i2c_offset = 12;
        }
        status = GetResourceNum(pDxgkInterface, acpi_i2c_offset, &m_i2c_connection_id, ResourceType_I2C);
        if (!NT_SUCCESS(status)) {
            printk_debug("NwlDsi display: ERROR getting Panel reset pin resource.\n");
            /* Pin Resource optional, some boards have expander */
        }

        if (m_disp_interface == nwl_dsi0) {
            cid0[0] = m_i2c_connection_id.LowPart;
            cid0[1] = m_i2c_connection_id.HighPart;
            cid0[2] = FLAG_DEV_I2C_EXPANDER; /* PinType = I2C expander PCA9557 */
            cid0[3] = 1; /* Expander output data register */
            cid0[4] = 0x40; /* Expander pin mask for MIPI_DSI0_EN */
            m_panel_pdev.dev.of_node.properties = (property*)&panel_raydium_properties0;
        }
        else if (m_disp_interface == nwl_dsi1) {
            cid1[0] = m_i2c_connection_id.LowPart;
            cid1[1] = m_i2c_connection_id.HighPart;
            cid1[2] = FLAG_DEV_I2C_EXPANDER; /* PinType = I2C expander PCA9557 */
            cid1[3] = 1; /* Expander output data register */
            cid1[4] = 0x80; /* Expander pin mask for MIPI_DSI1_EN */
            m_panel_pdev.dev.of_node.properties = (property*)&panel_raydium_properties1;
        }
        else {
            return STATUS_INTERNAL_ERROR;
        }

        /*Setting nwl_dsi_adjust_clk_drop_lvl to 2 may be needed */

        m_panel_pdev.dev.parent = &m_dsi_pdev.dev;
        if (mipi_dsi_device_attach(&m_panel_pdev, m_channel_id) != 0) {
            mipi_dsi_device_detach(&m_panel_pdev);
            printk("NwlDsi display: ERROR mipi_dsi_device_detach failed.\n");
            return STATUS_INTERNAL_ERROR;
        }
        if (rad_panel_probe(&m_panel_pdev, DSI_VIDEO_MODE_NON_BURST_SYNC_PULSE, m_num_lanes) != 0) {
            rad_panel_remove(&m_panel_pdev);
            mipi_dsi_device_detach(&m_panel_pdev);
            printk("NwlDsi display: ERROR rad_panel_probe failed.\n");
            return STATUS_INTERNAL_ERROR;
        }
    }
    return STATUS_SUCCESS;
}

NTSTATUS NwlDsiTransmitter::Stop()
{
    if (!m_i2c_main.is_initialized) {
        rad_panel_remove(&m_panel_pdev);
        mipi_dsi_device_detach(&m_panel_pdev);
    }

    if (m_i2c_main.is_initialized) {
        adv7511_bridge_detach(&m_i2c_main);
        adv7511_remove(&m_i2c_main);
    }

    nwl_dsi_bridge_detach(&m_dsi_pdev);
    mixel_dphy_remove(&m_dsi_phy_pdev);
    nwl_dsi_remove(&m_dsi_pdev);

    return STATUS_SUCCESS;
}

int NwlDsiTransmitter::GetCachedEdid(PVOID *pEdid)
{
    *pEdid = m_CachedEdid;
    return sizeof(m_CachedEdid);
}

NTSTATUS NwlDsiTransmitter::GetHotPlugDetectStatus(UINT8* status)
{
    /* Display is always connected */
    *status = 1;
    return STATUS_SUCCESS;
}

NTSTATUS NwlDsiTransmitter::GetResourceNum(DXGKRNL_INTERFACE* pDxgkInterface, ULONG i2c_index, LARGE_INTEGER *i2c_connection_id, enum ResType restype)
{
    NTSTATUS status;
    WCHAR   buff[512];

    status = GetReslist(pDxgkInterface, (PWCHAR)&buff, sizeof(buff));
    if (!NT_SUCCESS(status)) {
        printk("NwlDsi display: Error getting resource list\n");
        return status;
    }

    status = ParseReslist((PCM_RESOURCE_LIST)&buff, CmResourceTypeConnection, i2c_connection_id, NULL, i2c_index, restype);
    if (!NT_SUCCESS(status)) {
        printk("NwlDsi display: Error parsing resource list\n");
        return status;
    }

    printk_debug("NwlDsi display: I2C resource found with connection id: 0x%llx\n", i2c_connection_id->QuadPart);
    return STATUS_SUCCESS;
}
