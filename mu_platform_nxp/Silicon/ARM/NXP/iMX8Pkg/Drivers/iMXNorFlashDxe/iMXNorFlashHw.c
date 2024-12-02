/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 NOR Flash driver for Micron flash chip on iMX boards, connected via FlexSPI module

 */

#include <Protocol/FirmwareVolumeBlock.h>
#include <Library/DebugLib.h>
#include <Library/BaseLib.h>
#include <Library/PcdLib.h>
#include <Library/TimerLib.h>
#include <Library/IoLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/UefiRuntimeLib.h>

#include "iMXNorFlashHw.h"
#include "iMXNorDriver.h"
#include "iMX8FlexSpiLib.h"

#define TEST_BUF_LEN 32
static UINT8 TestBuf[TEST_BUF_LEN];

EFI_STATUS iMXNorHW_Init() {
  struct FspiData *fspi = GetFspiData();
  fspi->iobase = (void*) IMX_FLEXSPI_BASE_ADDR;
  fspi->ahb_addr = (void*) 0x08000000;

  BOOLEAN found = iMXFindDevice();
  if (found) {
    DEBUG((DEBUG_INFO, "iMXNorFlash find device success\n"));
  } else {
    DEBUG((DEBUG_ERROR, "iMXNorFlash find device failed\n"));
  }

  // Note: the initial read is necessary to get the Flash mapped at Physical address 0x8000000
  // READ
  UINT32 retlen = 0;
  ZeroMem(TestBuf, TEST_BUF_LEN);

  int res = iMXNorRead(0 /*offset*/, TEST_BUF_LEN, &retlen, TestBuf);
  if (res != 0) {
    DEBUG((DEBUG_ERROR, "iMXNorRead failed, res=%d\n", res));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

//static int8_t erased = 0;

EFI_STATUS iMXNorHW_WriteBlock(UINT64 NvStorageVariableBase, EFI_LBA Lba,
    UINTN Offset, UINTN Size) {
  int res = iMXNorErase(Lba * NOR_BLOCK_SIZE, NOR_BLOCK_SIZE);
  if (res != 0) {
    DEBUG((DEBUG_ERROR, "iMXNorErase failed, res=%d\n", res));
    return EFI_DEVICE_ERROR;
  }

  // WRITE
  UINT32 retlen = 0;
  UINT8 *memPtr = (UINT8*) (NvStorageVariableBase + Lba * NOR_BLOCK_SIZE
      + Offset);

  res = iMXNorWrite(Lba * NOR_BLOCK_SIZE + Offset /*offset*/, Size, &retlen,
      memPtr);

  if (res != 0) {
    DEBUG((DEBUG_ERROR, "iMXNorFlash nor _write failed, res=%d\n", res));
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}

EFI_STATUS iMXNorHW_WriteBuffer(UINT8* Buffer, EFI_LBA Lba) {
  int res = iMXNorErase(Lba * NOR_BLOCK_SIZE, NOR_BLOCK_SIZE);
  if (res != 0) {
    DEBUG((DEBUG_ERROR, "iMXNorErase failed, res=%d\n", res));
    return EFI_DEVICE_ERROR;
  }

  // WRITE
  UINT32 retlen = 0;

  res = iMXNorWrite(Lba * NOR_BLOCK_SIZE, NOR_BLOCK_SIZE, &retlen, Buffer);

  if (res != 0) {
    DEBUG((DEBUG_ERROR, "iMXNorHW_WriteBuffer failed, res=%d\n", res));
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}


EFI_STATUS iMXNorHW_AddressChangeEvent() {
  EFI_STATUS Status;

  struct FspiData *fspi = GetFspiData();
  Status = EfiConvertPointer(0x0, (VOID**) &fspi->iobase);
  if (Status != EFI_SUCCESS)
    goto conv_err;

  Status = EfiConvertPointer(0x0, (VOID**) &fspi->devtype_data);
  if (Status != EFI_SUCCESS)
    goto conv_err;

  DEBUG((DEBUG_INFO, "iMXNorHW_AddressChangeEvent ConvertPointer SUCCESS\n"));
  return EFI_SUCCESS;

conv_err:
  DEBUG((DEBUG_ERROR, "iMXNorHW_AddressChangeEvent ConvertPointer FAILED\n"));
  return Status;
}
