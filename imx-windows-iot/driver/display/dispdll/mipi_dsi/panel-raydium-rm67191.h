// SPDX-License-Identifier: GPL-2.0
/*
 * Raydium RM67191 MIPI-DSI panel driver
 *
 * Copyright 2023 NXP
 */

#ifndef __PANEL_RAYDIUM_RM67191_H__
#define __PANEL_RAYDIUM_RM67191_H__

#define DSI_VIDEO_MODE_BURST                  0
#define DSI_VIDEO_MODE_NON_BURST_SYNC_EVENT   1
#define DSI_VIDEO_MODE_NON_BURST_SYNC_PULSE   2
#define DSI_VIDEO_MODE_COMMAND                3

struct platform_device;
struct drm_panel;

int rad_panel_probe(struct platform_device* pdev, u32 video_mode, u8 lanes);
int rad_panel_remove(struct platform_device* pdev);
void rad_panel_shutdown(struct platform_device* pdev);

#endif

