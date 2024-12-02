/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2017, 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef __MIPI_DSI_NORTHWEST_H
#define __MIPI_DSI_NORTHWEST_H

enum dsi_pixel_format {
	DSI_FMT_RGB565 = 0,
	DSI_FMT_RGB666_PACKED = 1,
	DSI_FMT_RGB666 = 2,
	DSI_FMT_RGB888 = 3,
};

EFI_STATUS mipi_dsi_northwest_init(
			    IMX_DISPLAY_TIMING *timings,
			    unsigned int max_data_lanes,
			    enum dsi_pixel_format fmt,
			    imxDisplayInterfaceType displayInterface);
EFI_STATUS mipi_dsi_northwest_host_attach(VOID);
EFI_STATUS mipi_dsi_northwest_host_dpi_rst_deassert(VOID);
EFI_STATUS mipi_dsi_northwest_enable(VOID);
EFI_STATUS mipi_dsi_northwest_disable(VOID);
EFI_STATUS mipi_dsi_northwest_host_transfer(uint8_t Type, uint8_t Chan, uint32_t Flg, const void *Data, uint16_t Size);
EFI_STATUS mipi_dsi_northwest_dump(VOID);

#endif
