/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 NOR Flash driver for Micron flash chip on iMX boards, connected via FlexSPI module
 */

#ifndef IMX_NOR_FLASH_HW_H_
#define IMX_NOR_FLASH_HW_H_

#include <Protocol/FirmwareVolumeBlock.h>
#include <Library/BaseLib.h>

#define NOR_BLOCK_SIZE 4096
#define NOR_BLOCK_COUNT 8192
#define FLASH_MAPPED_BASE 0x08000000

EFI_STATUS iMXNorHW_Init();
EFI_STATUS iMXNorHW_AddressChangeEvent();
EFI_STATUS iMXNorHW_WriteBlock(UINT64 NvStorageVariableBase, EFI_LBA Lba,
    UINTN Offset, UINTN Size);
EFI_STATUS iMXNorHW_WriteBuffer(UINT8* memPtr, EFI_LBA Lba);

#endif //IMX_NOR_FLASH_HW_H_

