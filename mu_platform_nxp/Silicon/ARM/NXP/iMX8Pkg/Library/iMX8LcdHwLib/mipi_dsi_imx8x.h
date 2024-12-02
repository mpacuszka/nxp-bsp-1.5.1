/*
* Copyright 2023 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _MIPI_DSI_IMX8X_H_
#define _MIPI_DSI_IMX8X_H_

#define DISP_INTERFACE0_BASE 0x56220000
#define DISP_INTERFACE1_BASE 0x56240000

#define MIPI_CSR_OFFSET 0x1000 /* Subsystem Control Status Registers (CSR) */
#define MIPI_CSR_TX_ULPS  0x0
#define MIPIv2_CSR_TX_ULPS  0x30
#define MIPI_CSR_TX_ULPS_VALUE  0x1F

#define MIPI_CSR_PXL2DPI         0x4
#define MIPIv2_CSR_PXL2DPI         0x40

#define MIPI_CSR_PXL2DPI_16_BIT_PACKED       0x0
#define MIPI_CSR_PXL2DPI_16_BIT_565_ALIGNED  0x1
#define MIPI_CSR_PXL2DPI_16_BIT_565_SHIFTED  0x2
#define MIPI_CSR_PXL2DPI_18_BIT_PACKED       0x3
#define MIPI_CSR_PXL2DPI_18_BIT_ALIGNED      0x4
#define MIPI_CSR_PXL2DPI_24_BIT              0x5

#define	DSI_CMD_BUF_MAXSIZE         (128)

#define MIPI_DSI_OFFSET 0x8000 /* MIPI DSI Controller */

/* DPI interface pixel color coding map */
enum mipi_dsi_dpi_fmt {
	MIPI_RGB565_PACKED = 0,
	MIPI_RGB565_LOOSELY = 1,
	MIPI_RGB565_CONFIG3 = 2,
	MIPI_RGB666_PACKED = 3,
	MIPI_RGB666_LOOSELY = 4,
	MIPI_RGB888 = 5,
};

#endif /* _MIPI_DSI_IMX8X_H_ */
