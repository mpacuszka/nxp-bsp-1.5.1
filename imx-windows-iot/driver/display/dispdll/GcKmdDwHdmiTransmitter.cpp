/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Copyright 2022 NXP
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
#include "GcKmdDwHdmiTransmitter.tmh"
#include "GcKmdDwHdmiTransmitter.h"
#include "getresrc.h"

extern "C" {
#include "boot/dts/freescale/board.h"
#include "bridge/synopsys/dw_hdmi.h"
#include <drm/drm_fourcc.h>
#include <edidtst.h>
#include "drm/drm_connector.h"
#include "drm/drm_edid.h"
}

#define printk(x, ...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, __VA_ARGS__)

/* #define DW_HDMI_TRANSMITTER_DEBUG */
#ifdef DW_HDMI_TRANSMITTER_DEBUG
    #define printk_debug printk
#else
    #define printk_debug
#endif

static UCHAR* default_edid = (UCHAR* )edid_mon_1280_720_60;

/* Use local default_edid defined above instead of registry edid */
static const bool dw_hdmi_transmitter_override_registry_edid = FALSE;

void DwHdmiTransmitter::CopyFixedEdid(PVOID Data, ULONG Length, const UCHAR* fixed_edid)
{
    RtlCopyMemory(m_CachedEdid, fixed_edid, EDID_SIZE);
    RtlCopyMemory(Data, m_CachedEdid, min(Length, EDID_SIZE));
    printk_debug("DwHdmiTransmitter::GetEdid: fixed edid copy_len = %d\n", min(Length, EDID_SIZE));
}

NTSTATUS DwHdmiTransmitter::GetEdid(PVOID Data, ULONG Length, UINT8 Block, UINT8 Segment)
{
    struct edid* edid;
    UINT edid_len = EDID_SIZE;
    int hactive = 0, vactive = 0;

    if (Data == NULL) {
        return STATUS_INVALID_PARAMETER;
    }

    printk_debug("DwHdmiTransmitter::GetEdid: Requested: Len = %d, Block = %d, Segment = %d\n", Length, Block, Segment);
    if (dw_hdmi_transmitter_override_registry_edid) {
        CopyFixedEdid(Data, Length, default_edid);
        printk_debug("DwHdmiTransmitter::GetEdid: dw_hdmi_transmitter_override_registry_edid = true, default edid will be used \n");
    } else {
        //TODO: optimize repeated edid read
        edid = dw_hdmi_bridge_get_edid(&hdmi_pdev);
        if (edid) {
            edid_len = (edid->extensions + 1) * EDID_SIZE;
            GetModeSize(edid, edid_len, &hactive, &vactive);
            if (hactive * vactive > 1920 * 1080) {
                printk("DW_HDMI display: monitor default resolution exceeds 1080p. 1080p resolution will be used.\n");
                CopyFixedEdid(Data, Length, edid_mon_1920_1080_60);
            } else {
                RtlCopyMemory(m_CachedEdid, edid, min(Length, EDID_SIZE));
                RtlCopyMemory(Data, edid, min(Length, edid_len));
                printk_debug("DwHdmiTransmitter::GetEdid: read edid_len = %d extensions = %d copy_len = %d\n",
                    edid_len, edid->extensions, min(Length, edid_len));
            }
            kfree(edid);
        } else {
            /* Fall back to default edid */
            printk("DW_HDMI display: Edid read failed, default edid will be used.\n");
            CopyFixedEdid(Data, Length, default_edid);
        }
    }

    return STATUS_SUCCESS;
}

void DwHdmiTransmitter::GetChildDescriptor(DXGK_CHILD_DESCRIPTOR* pDescriptor)
{
    pDescriptor->ChildDeviceType = TypeVideoOutput;
    pDescriptor->ChildCapabilities.HpdAwareness = HpdAwarenessInterruptible;
    pDescriptor->ChildCapabilities.Type.VideoOutput.InterfaceTechnology = D3DKMDT_VOT_HDMI;
    pDescriptor->ChildCapabilities.Type.VideoOutput.MonitorOrientationAwareness = D3DKMDT_MOA_NONE;
    pDescriptor->ChildCapabilities.Type.VideoOutput.SupportsSdtvModes = FALSE;
    /* child device is not an ACPI device */
    pDescriptor->AcpiUid = 0;
    pDescriptor->ChildUid = HDMI_CHILD_UID;
}


NTSTATUS DwHdmiTransmitter::Start(DXGKRNL_INTERFACE* pDxgkInterface)
{
    UINT phy_offset;

    printk_debug("DW_HDMI display: HDMI interface\n");

    hdmi_pdev.name = "hdmi_dev";
    hdmi_pdev.plat_name = "mp";
    hdmi_pdev.data = pDxgkInterface;
    board_init(&hdmi_pdev);
    if (dw_hdmi_imx_probe(&hdmi_pdev, HDMI_CHILD_UID) != 0) {
        dw_hdmi_imx_remove(&hdmi_pdev);
        printk("DW_HDMI display: ERROR imx8mp_ldb_probe failed.\n");
        return STATUS_INTERNAL_ERROR;
    }

    dw_hdmi_bridge_attach(&hdmi_pdev);

    hdmi_phy_pdev.name = "hdmi_phy";
    hdmi_phy_pdev.plat_name = "mp";
    board_init(&hdmi_phy_pdev);
    /* Need to have hdmi_pdev initialized first, to get base address of hdmi. 
       The phy address is not page aligned, so we can't alocate regs for phy directly and need
       to use hdmi alocated regs and compute offset of phy. Phy is at higher address than hdmi. */
    phy_offset = (UINT)(hdmi_phy_pdev.resource[0].start - hdmi_pdev.resource[0].start);
    if (samsung_hdmi_phy_probe(&hdmi_phy_pdev, dw_hdmi_get_reg(&hdmi_pdev), phy_offset) != 0) {
        samsung_hdmi_phy_remove(&hdmi_phy_pdev);
        dw_hdmi_imx_remove(&hdmi_pdev);
        printk("DW_HDMI display: ERROR samsung_hdmi_phy_probe failed.\n");
        return STATUS_INTERNAL_ERROR;
    }

    return STATUS_SUCCESS;
}

NTSTATUS DwHdmiTransmitter::Stop()
{
    /* To full stop the hardware, call: dw_hdmi_bridge_atomic_disable(&hdmi_pdev); */

    samsung_hdmi_phy_remove(&hdmi_phy_pdev);
    dw_hdmi_bridge_detach(&hdmi_pdev);
    dw_hdmi_imx_remove(&hdmi_pdev);

    return STATUS_SUCCESS;
}

int DwHdmiTransmitter::GetCachedEdid(PVOID *pEdid)
{
    *pEdid = m_CachedEdid;
    return sizeof(m_CachedEdid);
}

NTSTATUS DwHdmiTransmitter::GetHotPlugDetectStatus(UINT8* status)
{
    enum drm_connector_status conn_status;

    if (!status) {
        return STATUS_INTERNAL_ERROR;
    }
    conn_status = dw_hdmi_bridge_detect(&hdmi_pdev);
    *status = (conn_status == connector_status_connected) ? TRUE : FALSE;
    printk_debug("DW_HDMI display: GetHotPlugDetectStatus: connected = %d\n", *status);
    return STATUS_SUCCESS;
}
