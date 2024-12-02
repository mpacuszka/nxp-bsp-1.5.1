/* Copyright (c) Microsoft Corporation.
 * Copyright 2023 NXP
   Licensed under the MIT License. */

#pragma once

/*Display interfaces read from registry */
#define DISP_INTERFACE_DISABLED     0x0
#define DISP_INTERFACE_HDMI         0x1
#define DISP_INTERFACE_MIPI_DSI0    0x2
#define DISP_INTERFACE_MIPI_DSI1    0x3
#define DISP_INTERFACE_LVDS0        0x4
#define DISP_INTERFACE_LVDS1        0x5
#define DISP_INTERFACE_LVDS_DUAL0   0x6
#define DISP_INTERFACE_PARALLEL_LCD 0x7

struct DisplayInterface {
    BOOLEAN UseDisplay; //Display interface above is requested in registry
    UINT RegistryIndex; //Index for registry parameters access, e.g. 0 for L"Display0Interface"
    UINT DisplayInterfaceIndex; //Index of the display interface, e.g. 0 for LVDS0, 1 for LVDS1
    BOOLEAN Shared; //Flag indicates shared portion of HW between two display interfaces - platform specific.
};
