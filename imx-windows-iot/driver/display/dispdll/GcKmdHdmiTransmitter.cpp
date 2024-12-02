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
#include "GcKmdHdmiTransmitter.h"
#include "edidparser.h"
#include <edidtst.h>

BOOLEAN HdmiTransmitter::CheckAlive(void)
{
    unsigned int alive, newalive;
    UINT8 retries_left = 10;

    AdvancedPeripheralBusRead(KEEP_ALIVE, &alive);

    LARGE_INTEGER interval;
    interval.QuadPart = -10;
    do
    {
        KeDelayExecutionThread(KernelMode, FALSE, &interval);
        AdvancedPeripheralBusRead(KEEP_ALIVE, &newalive);

        if (alive == newalive)
        {
            continue;
        }

        return TRUE;
    }
    while (retries_left--);

    return FALSE;
}

NTSTATUS HdmiTransmitter::EchoMessage(UINT8 const* Data, UINT8* Response, int NumBytes)
{
    if ((NumBytes > GENERAL_TEST_ECHO_MAX_PAYLOAD) ||
        (NumBytes < GENERAL_TEST_ECHO_MIN_PAYLOAD))
    {
        return STATUS_INTERNAL_ERROR;
    }

    auto status = MakeMessage(MB_MODULE_ID_GENERAL, GENERAL_TEST_ECHO, Data, NumBytes);
    if (status != STATUS_SUCCESS)
    {
        return status;
    }

    WriteMsgToMbox();
    ReadMsgFromMbox();

    if (!ValidMsgHeader(m_MailboxState.RxBuffer, MB_MODULE_ID_GENERAL, GENERAL_TEST_ECHO))
    {
        return STATUS_INTERNAL_ERROR;
    }

    ParseMessage(NumBytes, Response);
    return STATUS_SUCCESS;
}

NTSTATUS HdmiTransmitter::GetEdid(PVOID Data, ULONG Length, UINT8 Block, UINT8 Segment)
{
    struct HDMITX_TRANS_DATA
    {
        UINT8* buff;
        UINT8 status;
        UINT16 len;
        UINT8 slave;
        UINT8 offset;
    } hdmi_data;

    auto status = MakeMessage(MB_MODULE_ID_HDMI_TX, HDMI_TX_EDID);
    if (status != STATUS_SUCCESS)
    {
        return status;
    }

    WriteMsgToMbox();
    ReadMsgFromMbox();

    if (!SameTxRxHeader())
    {
        return STATUS_INTERNAL_ERROR;
    }

    status = ParseMessage(
        &hdmi_data.status,
        &hdmi_data.slave,
        &hdmi_data.offset,
        &hdmi_data.len,
        &hdmi_data.buff
    );
    if (status != STATUS_SUCCESS)
    {
        return status;
    }

    int hactive = 0, vactive = 0;
    GetModeSize(hdmi_data.buff, min(Length, hdmi_data.len),
        &hactive, &vactive);

    // if edid.len == 0 it means that monitor is not connected
    // or does not support EDID, we fall back to predefined 720p mode
    if (hdmi_data.len == 0)
    {
        RtlCopyMemory(Data, edid_mon_1280_720_60, EDID_SIZE);
        RtlCopyMemory(m_CachedEdid, edid_mon_1280_720_60, EDID_SIZE);
    }
    // 4K mode is not supported,
    // if mode > 1080p we fall back to predefined 1080p mode
    else if (hactive * vactive > 1920 * 1080)
    {
        RtlCopyMemory(Data, edid_mon_1920_1080_60, EDID_SIZE);
        RtlCopyMemory(m_CachedEdid, edid_mon_1920_1080_60, EDID_SIZE);
    }
    else
    {
        RtlCopyMemory(Data, hdmi_data.buff, min(Length, hdmi_data.len));
        RtlCopyMemory(m_CachedEdid, hdmi_data.buff, min(sizeof(m_CachedEdid),
            hdmi_data.len));
    }

    return STATUS_SUCCESS;
}

void HdmiTransmitter::GetChildDescriptor(DXGK_CHILD_DESCRIPTOR* pDescriptor)
{
    pDescriptor->ChildDeviceType = TypeVideoOutput;
    pDescriptor->ChildCapabilities.HpdAwareness = HpdAwarenessInterruptible;
    pDescriptor->ChildCapabilities.Type.VideoOutput.InterfaceTechnology = D3DKMDT_VOT_HDMI;
    pDescriptor->ChildCapabilities.Type.VideoOutput.MonitorOrientationAwareness = D3DKMDT_MOA_NONE;
    pDescriptor->ChildCapabilities.Type.VideoOutput.SupportsSdtvModes = FALSE;
    // child device is not an ACPI device
    pDescriptor->AcpiUid = 0;
    pDescriptor->ChildUid = HDMI_CHILD_UID;
}

void HdmiTransmitter::WriteMsgToMbox()
{
    auto length =
        (unsigned int)m_MailboxState.TxBuffer[2] << 8 | (unsigned int)m_MailboxState.TxBuffer[3];

    unsigned int i = 0;
    while (i < length + CMD_HEAD_SIZE)
    {
        if (MailboxWrite(m_MailboxState.TxBuffer[i]))
        {
            i++;
        }
    }
}

void HdmiTransmitter::ReadMsgFromMbox()
{
    unsigned int i = 0;
    while (i < CMD_HEAD_SIZE)
    {
        if (MailboxRead(&m_MailboxState.RxBuffer[i]))
        {
            i++;
        }
    }

    auto length =
        (unsigned int)m_MailboxState.RxBuffer[2] << 8 | (unsigned int)m_MailboxState.RxBuffer[3];

    while (i < length + CMD_HEAD_SIZE)
    {
        if (MailboxRead(&m_MailboxState.RxBuffer[i]))
        {
            i++;
        }
    }
}

BOOLEAN HdmiTransmitter::MailboxRead(volatile UINT8* value)
{
    UINT32 empty, data;

    AdvancedPeripheralBusRead(MAILBOX_EMPTY_ADDR, &empty);
    if (empty)
    {
        return FALSE;
    }

    AdvancedPeripheralBusRead(MAILBOX0_RD_DATA, &data);

    *value = (UINT8)data;
    return TRUE;
}

BOOLEAN HdmiTransmitter::MailboxWrite(UINT8 value)
{
    UINT32 full;

    AdvancedPeripheralBusRead(MAILBOX_FULL_ADDR, &full);
    if (full)
    {
        return FALSE;
    }

    AdvancedPeripheralBusWrite(MAILBOX0_WR_DATA, value);
    return TRUE;
}
