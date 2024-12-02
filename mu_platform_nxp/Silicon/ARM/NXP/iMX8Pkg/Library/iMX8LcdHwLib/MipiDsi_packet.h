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

#ifndef __MIPI_DSI_PACKET_H
#define __MIPI_DSI_PACKET_H

/* Following constants are taken from mipi alliance specification for Display Command Set (DCS) */
/* MIPI DCS commands */
enum {
    DCS_CMD_NOP                     = 0x00,
    DCS_CMD_SOFT_RESET              = 0x01,
    DCS_CMD_GET_COMPRESSION_MODE    = 0x03,
    DCS_CMD_GET_DISPLAY_ID          = 0x04,
    DCS_CMD_GET_RED_CHANNEL         = 0x06,
    DCS_CMD_GET_GREEN_CHANNEL       = 0x07,
    DCS_CMD_GET_BLUE_CHANNEL        = 0x08,
    DCS_CMD_GET_DISPLAY_STATUS      = 0x09,
    DCS_CMD_GET_POWER_MODE          = 0x0A,
    DCS_CMD_GET_ADDRESS_MODE        = 0x0B,
    DCS_CMD_GET_PIXEL_FORMAT        = 0x0C,
    DCS_CMD_GET_DISPLAY_MODE        = 0x0D,
    DCS_CMD_GET_SIGNAL_MODE         = 0x0E,
    DCS_CMD_GET_DIAGNOSTIC_RESULT   = 0x0F,
    DCS_CMD_ENTER_SLEEP_MODE        = 0x10,
    DCS_CMD_EXIT_SLEEP_MODE         = 0x11,
    DCS_CMD_ENTER_PARTIAL_MODE      = 0x12,
    DCS_CMD_ENTER_NORMAL_MODE       = 0x13,
    DCS_CMD_EXIT_INVERT_MODE        = 0x20,
    DCS_CMD_ENTER_INVERT_MODE       = 0x21,
    DCS_CMD_SET_GAMMA_CURVE         = 0x26,
    DCS_CMD_SET_DISPLAY_OFF         = 0x28,
    DCS_CMD_SET_DISPLAY_ON          = 0x29,
    DCS_CMD_SET_COLUMN_ADDRESS      = 0x2A,
    DCS_CMD_SET_PAGE_ADDRESS        = 0x2B,
    DCS_CMD_WRITE_MEMORY_START      = 0x2C,
    DCS_CMD_WRITE_LUT               = 0x2D,
    DCS_CMD_READ_MEMORY_START       = 0x2E,
    DCS_CMD_SET_PARTIAL_ROWS        = 0x30,
    DCS_CMD_SET_PARTIAL_COLUMNS     = 0x31,
    DCS_CMD_SET_SCROLL_AREA         = 0x33,
    DCS_CMD_SET_TEAR_OFF            = 0x34,
    DCS_CMD_SET_TEAR_ON             = 0x35,
    DCS_CMD_SET_ADDRESS_MODE        = 0x36,
    DCS_CMD_SET_SCROLL_START        = 0x37,
    DCS_CMD_EXIT_IDLE_MODE          = 0x38,
    DCS_CMD_ENTER_IDLE_MODE         = 0x39,
    DCS_CMD_SET_PIXEL_FORMAT        = 0x3A,
    DCS_CMD_WRITE_MEMORY_CONTINUE   = 0x3C,
    DCS_CMD_SET_3D_CONTROL          = 0x3D,
    DCS_CMD_READ_MEMORY_CONTINUE    = 0x3E,
    DCS_CMD_GET_3D_CONTROL          = 0x3F,
    DCS_CMD_SET_VSYNC_TIMING        = 0x40,
    DCS_CMD_SET_TEAR_SCANLINE       = 0x44,
    DCS_CMD_GET_SCANLINE            = 0x45,
    DCS_CMD_SET_DISPLAY_BRIGHTNESS  = 0x51,
    DCS_CMD_GET_DISPLAY_BRIGHTNESS  = 0x52,
    DCS_CMD_WRITE_CONTROL_DISPLAY   = 0x53,
    DCS_CMD_GET_CONTROL_DISPLAY     = 0x54,
    DCS_CMD_WRITE_POWER_SAVE        = 0x55,
    DCS_CMD_GET_POWER_SAVE          = 0x56,
    DCS_CMD_SET_CABC_MIN_BRIGHTNESS = 0x5E,
    DCS_CMD_GET_CABC_MIN_BRIGHTNESS = 0x5F,
    DCS_CMD_READ_DDB_START          = 0xA1,
    DCS_CMD_READ_DDB_CONTINUE       = 0xA8,
};

/* Following constants are taken from mipi alliance specification for Display Serial Interface (DSI) */
/* Processor-to-Peripheral direction */
enum {
    DSI_CMD_V_SYNC_START                    = 0x01,
    DSI_CMD_V_SYNC_END                      = 0x11,
    DSI_CMD_H_SYNC_START                    = 0x21,
    DSI_CMD_H_SYNC_END                      = 0x31,
    DSI_CMD_COLOR_MODE_OFF                  = 0x02,
    DSI_CMD_COLOR_MODE_ON                   = 0x12,
    DSI_CMD_SHUTDOWN_PERIPH                 = 0x22,
    DSI_CMD_TURN_ON_PERIPH                  = 0x32,
    DSI_CMD_GEN_SHORT_WRITE_0_PAR           = 0x03,
    DSI_CMD_GEN_SHORT_WRITE_1_PAR           = 0x13,
    DSI_CMD_GEN_SHORT_WRITE_2_PAR           = 0x23,
    DSI_CMD_READ_REQ_0_PAR                  = 0x04,
    DSI_CMD_GEN_READ_REQ_1_PAR              = 0x14,
    DSI_CMD_GEN_READ_REQ_2_PAR              = 0x24,
    DSI_CMD_DCS_SHORT_WRITE                 = 0x05,
    DSI_CMD_DCS_SHORT_WRITE_PAR             = 0x15,
    DSI_CMD_DCS_READ                        = 0x06,
    DSI_CMD_SET_MAX_RETURN_PKT_SIZE         = 0x37,
    DSI_CMD_END_OF_TX_PKT                   = 0x08,
    DSI_CMD_NULL_PKT                        = 0x09,
    DSI_CMD_BLANK_PKT                       = 0x19,
    DSI_CMD_GEN_LONG_WRITE                  = 0x29,
    DSI_CMD_DCS_LONG_WRITE                  = 0x39,
    DSI_CMD_LOOSELY_PACKED_PXL_STREAM_YUV20 = 0x0C,
    DSI_CMD_PACKED_PXL_STREAM_YUV24         = 0x1C,
    DSI_CMD_PACKED_PXL_STREAM_YUV16         = 0x2C,
    DSI_CMD_PACKED_PXL_STREAM_RGB30         = 0x0D,
    DSI_CMD_PACKED_PXL_STREAM_RGB36         = 0x1D,
    DSI_CMD_PACKED_PXL_STREAM_YUV12         = 0x3D,
    DSI_CMD_PACKED_PXL_STREAM_RGB16         = 0x0E,
    DSI_CMD_PACKED_PXL_STREAM_RGB18         = 0x1E,
    DSI_CMD_LOOSELY_PACKED_PXL_STREAM_RGB18 = 0x2E,
    DSI_CMD_PACKED_PXL_STREAM_RGB24         = 0x3E,
};

enum mipi_dsi_packet_tear_mode {
	MIPI_DSI_DCS_TEAR_MODE_VBL,
	MIPI_DSI_DCS_TEAR_MODE_VBL_HBL,
};

/* Peripheral-to-Processor direction */
enum {
    DSI_CMD_RX_ACK_AND_ERROR_REPORT         = 0x02,
    DSI_CMD_RX_END_OF_TX_PKT                = 0x08,
    DSI_CMD_RX_GEN_SHORT_READ_RESPONSE_1B   = 0x11,
    DSI_CMD_RX_GEN_SHORT_READ_RESPONSE_2B   = 0x12,
    DSI_CMD_RX_GEN_LONG_READ_RESPONSE       = 0x1A,
    DSI_CMD_RX_DCS_LONG_READ_RESPONSE       = 0x1C,
    DSI_CMD_RX_DCS_SHORT_READ_RESPONSE_1B   = 0x21,
    DSI_CMD_RX_DCS_SHORT_READ_RESPONSE_2B   = 0x22,
};

/* Flags for packet transaction method */
#define DSI_MODE_HP      0x00
#define DSI_MODE_LP       0x01

typedef EFI_STATUS (*MipiDsiPktSend_t)(uint8_t, uint8_t, uint32_t, const void*, uint16_t);
BOOLEAN MipiDsi_IsLong(uint8_t Type);
EFI_STATUS MipiDsiPktRegisterCallback(MipiDsiPktSend_t Callback);
EFI_STATUS MipiDsiPktDcsSend(uint8_t cmd, uint8_t chan, uint32_t flags, const void *data, uint16_t sz);
EFI_STATUS MipiDsiPktShutdownPeripheral(BOOLEAN HighPower);
EFI_STATUS MipiDsiPktTurnOnPeripheral(BOOLEAN HighPower);
EFI_STATUS MipiDsiPktGenericWrite(const void *data, uint16_t sz, BOOLEAN HighPower);
EFI_STATUS MipiDsiPktDcsSetDisplayBrightness(uint8_t brightness, BOOLEAN HighPower);
EFI_STATUS MipiDsiPktDcsSetTearOn(enum mipi_dsi_packet_tear_mode mode, BOOLEAN HighPower);
EFI_STATUS MipiDsiPktDcsSetTearOff(BOOLEAN HighPower);
EFI_STATUS MipiDsiPktDcsSetTearScanline(uint16_t scanline, BOOLEAN HighPower);
EFI_STATUS MipiDsiPktDcsExitSleepMode(BOOLEAN HighPower);
EFI_STATUS MipiDsiPktDcsEnterSleepMode(BOOLEAN HighPower);
EFI_STATUS MipiDsiPktDcsSetPixelFormat(uint8_t format, BOOLEAN HighPower);
EFI_STATUS MipiDsiPktDcsSetDisplayOn(BOOLEAN HighPower);
EFI_STATUS MipiDsiPktDcsSetDisplayOff(BOOLEAN HighPower);
EFI_STATUS MipiDsiPktDcsSoftReset(BOOLEAN HighPower);
#endif

