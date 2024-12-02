/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef IMX_NOR_DRIVER_H_
#define IMX_NOR_DRIVER_H_

#include <ProcessorBind.h>

/* Find SPI Nor Flash Device */
BOOLEAN iMXFindDevice();

int iMXNorRead(UINT64 from, UINT32 len, UINT32 *retlen, UINT8 *buf);
int iMXNorErase(UINT32 addr, UINT32 len);
int iMXNorWrite(UINT64 to, UINT32 len, UINT32 *retlen, const UINT8 *buf);

#endif //IMX_NOR_DRIVER_H_

