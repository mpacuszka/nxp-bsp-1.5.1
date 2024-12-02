/*
* Copyright 2023 NXP
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the
*   names of its contributors may be used to endorse or promote products
*   derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <stdint.h>
#include <stdbool.h>
#include <Uefi.h>
#include <Library/DebugLib.h>
#include "MipiDsi_packet.h"

static MipiDsiPktSend_t PktSendCallback = NULL;

BOOLEAN MipiDsi_IsLong(uint8_t Type)
{
    switch (Type) {
        case DSI_CMD_NULL_PKT:
        case DSI_CMD_BLANK_PKT:
        case DSI_CMD_GEN_LONG_WRITE:
        case DSI_CMD_DCS_LONG_WRITE:
        case DSI_CMD_LOOSELY_PACKED_PXL_STREAM_YUV20:
        case DSI_CMD_PACKED_PXL_STREAM_YUV24:
        case DSI_CMD_PACKED_PXL_STREAM_YUV16:
        case DSI_CMD_PACKED_PXL_STREAM_RGB30:
        case DSI_CMD_PACKED_PXL_STREAM_RGB36:
        case DSI_CMD_PACKED_PXL_STREAM_YUV12:
        case DSI_CMD_PACKED_PXL_STREAM_RGB16:
        case DSI_CMD_PACKED_PXL_STREAM_RGB18:
        case DSI_CMD_LOOSELY_PACKED_PXL_STREAM_RGB18:
        case DSI_CMD_PACKED_PXL_STREAM_RGB24:
            return TRUE;
    }
    return FALSE;
}


static EFI_STATUS MipiDsiPktSend(uint8_t type, uint8_t chan, uint32_t flags, const void *data, uint16_t sz)
{
    if (!PktSendCallback) {
        return EFI_INVALID_PARAMETER;
    }

    return (PktSendCallback(type, chan, flags, data, sz));
}


EFI_STATUS MipiDsiPktRegisterCallback(MipiDsiPktSend_t Callback)
{
    if (!Callback) {
        return EFI_INVALID_PARAMETER;
    }
    PktSendCallback = Callback;
    return EFI_SUCCESS;
}

#define LONG_TXBUFF_SIZE 17
EFI_STATUS MipiDsiPktDcsSend(uint8_t cmd, uint8_t chan, uint32_t flags, const void *data, uint16_t sz)
{
    EFI_STATUS Ret;
    uint8_t long_tx_buf[LONG_TXBUFF_SIZE];
    uint8_t short_tx_buf[2];
    uint8_t *data_byte;
    uint32_t i;
    
    if ((sz + 1) > LONG_TXBUFF_SIZE) {
        return EFI_INVALID_PARAMETER;
    }
    data_byte = (uint8_t *) data;
    if (sz > 1) {
        /* Long packet */
        long_tx_buf[0] = cmd;
        for (i = 0; i < sz; i++) {
            long_tx_buf[i + 1] = *(data_byte + i);
        }
        Ret = MipiDsiPktSend(DSI_CMD_DCS_LONG_WRITE, chan, flags, long_tx_buf, (sz + 1));
    } else {
        /* Short packet */
        short_tx_buf[0] = cmd;
        if (sz == 0) {
            short_tx_buf[1] = 0;
            Ret = MipiDsiPktSend(DSI_CMD_DCS_SHORT_WRITE, chan, flags, short_tx_buf, 1);
        } else {
            short_tx_buf[1] = *data_byte;
            Ret = MipiDsiPktSend(DSI_CMD_DCS_SHORT_WRITE_PAR, chan, flags, short_tx_buf, 2);
        }
    }
    return Ret;
}

EFI_STATUS MipiDsiPktShutdownPeripheral(BOOLEAN HighPower)
{
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

	return MipiDsiPktSend(DSI_CMD_SHUTDOWN_PERIPH, 0, flags, NULL, 0);
}

EFI_STATUS MipiDsiPktTurnOnPeripheral(BOOLEAN HighPower)
{
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

	return MipiDsiPktSend(DSI_CMD_TURN_ON_PERIPH, 0, flags, NULL, 0);
}

EFI_STATUS MipiDsiPktGenericWrite(const void *data, uint16_t sz, BOOLEAN HighPower)
{
    uint8_t type;
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

	switch (sz) {
	case 0:
		type = DSI_CMD_GEN_SHORT_WRITE_0_PAR;
		break;

	case 1:
		type = DSI_CMD_GEN_SHORT_WRITE_1_PAR;
		break;

	case 2:
		type = DSI_CMD_GEN_SHORT_WRITE_2_PAR;
		break;

	default:
		type = DSI_CMD_GEN_LONG_WRITE;
		break;
	}
	return MipiDsiPktSend(type, 0, flags, data, sz);
}

EFI_STATUS MipiDsiPktDcsSetDisplayBrightness(uint8_t brightness, BOOLEAN HighPower)
{
    uint8_t data;
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

    data = brightness;
    return MipiDsiPktDcsSend(DCS_CMD_SET_DISPLAY_BRIGHTNESS, 0, flags, &data, 1);
}

EFI_STATUS MipiDsiPktDcsSetTearOn(enum mipi_dsi_packet_tear_mode mode, BOOLEAN HighPower)
{
    uint8_t val = mode;
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

    return MipiDsiPktDcsSend(DCS_CMD_SET_TEAR_ON, 0, flags, &val, 1);
}

EFI_STATUS MipiDsiPktDcsSetTearOff(BOOLEAN HighPower)
{
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

    return MipiDsiPktDcsSend(DCS_CMD_SET_TEAR_OFF, 0, flags, NULL, 0);
}

EFI_STATUS MipiDsiPktDcsSetTearScanline(uint16_t scanline, BOOLEAN HighPower)
{
    uint8_t data[3];

    data[0] = DCS_CMD_SET_TEAR_SCANLINE;
    data[1] = scanline >> 8;
    data[2] = scanline & 0xFF;
    return MipiDsiPktGenericWrite(data, 3, HighPower);
}

EFI_STATUS MipiDsiPktDcsExitSleepMode(BOOLEAN HighPower)
{
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

    return MipiDsiPktDcsSend(DCS_CMD_EXIT_SLEEP_MODE, 0, flags, NULL, 0);
}

EFI_STATUS MipiDsiPktDcsEnterSleepMode(BOOLEAN HighPower)
{
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

    return MipiDsiPktDcsSend(DCS_CMD_ENTER_SLEEP_MODE, 0, flags, NULL, 0);
}

EFI_STATUS MipiDsiPktDcsSetPixelFormat(uint8_t format, BOOLEAN HighPower)
{
    uint8_t val = format;
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

    return MipiDsiPktDcsSend(DCS_CMD_SET_PIXEL_FORMAT, 0, flags, &val, 1);
}

EFI_STATUS MipiDsiPktDcsSetDisplayOn(BOOLEAN HighPower)
{
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

    return MipiDsiPktDcsSend(DCS_CMD_SET_DISPLAY_ON, 0, flags, NULL, 0);
}

EFI_STATUS MipiDsiPktDcsSetDisplayOff(BOOLEAN HighPower)
{
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

    return MipiDsiPktDcsSend(DCS_CMD_SET_DISPLAY_OFF, 0, flags, NULL, 0);
}

EFI_STATUS MipiDsiPktDcsSoftReset(BOOLEAN HighPower)
{
    uint32_t flags = HighPower ? DSI_MODE_HP : DSI_MODE_LP;

    return MipiDsiPktDcsSend(DCS_CMD_SOFT_RESET, 0, flags, NULL, 0);
}

