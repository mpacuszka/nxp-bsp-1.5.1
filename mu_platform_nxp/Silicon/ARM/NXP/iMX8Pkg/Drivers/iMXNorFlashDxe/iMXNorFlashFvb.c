/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <Protocol/FirmwareVolumeBlock.h>
#include <Library/DebugLib.h>
#include <Library/BaseLib.h>
#include <Library/PcdLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeLib.h>
#include <Library/DxeServicesTableLib.h>

#include <Guid/VariableFormat.h>

#include "iMXNorFlashHw.h"

STATIC BOOLEAN mEfiAtVirtual = FALSE;
STATIC EFI_EVENT mVirtualAddressChangeEvent = NULL;

typedef struct {
  EFI_HANDLE Handle;
  EFI_FIRMWARE_VOLUME_BLOCK_PROTOCOL FvbInstance;
  UINT32 NvStorageVariableSize;
  UINT64 NvStorageVariableBase;
  UINT32 BlockSize;
  UINT32 NumberOfBlocks;
  UINT64 WriteCounter;
  EFI_FVB_ATTRIBUTES_2 Attributes;

} IMX_FVB_DEVICE;

IMX_FVB_DEVICE ImxFvbDevice;

EFI_STATUS EFIAPI ImxNorFvbGetAttributes(
    IN CONST EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
    OUT EFI_FVB_ATTRIBUTES_2 *Attributes
)
{
  *Attributes = ImxFvbDevice.Attributes;
  return EFI_SUCCESS;
}

EFI_STATUS EFIAPI ImxNorFvbSetAttributes(
    IN CONST EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
    IN OUT EFI_FVB_ATTRIBUTES_2 *Attributes
)
{
  ImxFvbDevice.Attributes = *Attributes;
  return EFI_SUCCESS;
}

EFI_STATUS EFIAPI ImxNorFvbGetPhysicalAddress(
    IN CONST EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
    OUT EFI_PHYSICAL_ADDRESS *Address
)
{
  *Address = ImxFvbDevice.NvStorageVariableBase;
  return EFI_SUCCESS;
}

EFI_STATUS EFIAPI ImxNorFvbGetBlockSize
(
    IN CONST EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
    IN EFI_LBA Lba,
    OUT UINTN *BlockSize,
    OUT UINTN *NumberOfBlocks
)
{
  *BlockSize = ImxFvbDevice.BlockSize;
  *NumberOfBlocks = ImxFvbDevice.NumberOfBlocks;

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI ImxNorFvbRead(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  IN        EFI_LBA                             Lba,
  IN        UINTN                               Offset,
  IN OUT    UINTN                               *NumBytes,
  IN OUT    UINT8                               *Buffer
)
{
  DEBUG(
    (DEBUG_INFO, "ImxNorFvbRead LBA=%lu, Offset=0x%lx, Bytes=%lu Buffer=%p\n", Lba, Offset, *NumBytes, Buffer)); CopyMem(Buffer, (void*)(ImxFvbDevice.NvStorageVariableBase + Lba*ImxFvbDevice.BlockSize + Offset), *NumBytes);
return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI ImxNorFvbWrite (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  IN        EFI_LBA                             Lba,
  IN        UINTN                               Offset,
  IN OUT    UINTN                               *NumBytes,
  IN        UINT8                               *Buffer
)
{
  EFI_STATUS Status;

  DEBUG(
  (DEBUG_INFO, "ImxNorFvbWrite (Cnt=%lu) LBA=%lu, Offset=0x%lx, Bytes=%lu [%02x ..]\n", ImxFvbDevice.WriteCounter++, Lba, Offset, *NumBytes, Buffer[0]));

  if (Lba*ImxFvbDevice.BlockSize < ImxFvbDevice.NvStorageVariableSize) {
    CopyMem((void*)(ImxFvbDevice.NvStorageVariableBase + Lba*ImxFvbDevice.BlockSize + Offset), Buffer, *NumBytes);
    Status = iMXNorHW_WriteBlock(ImxFvbDevice.NvStorageVariableBase, Lba, 0, NOR_BLOCK_SIZE);
  } else {
    Status = iMXNorHW_WriteBuffer(Buffer, Lba);
  }

  return Status;
}

EFI_STATUS
EFIAPI ImxNorFvbEraseBlocks(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  ...
)
{
  DEBUG((DEBUG_INFO, "ImxNorFvbEraseBlocks\n")); return EFI_SUCCESS;
}

VOID
EFIAPI
iMXNorFvb_AddressChangeEvent (
  IN EFI_EVENT                            Event,
  IN VOID                                 *Context
  )
{
  mEfiAtVirtual = TRUE;

  iMXNorHW_AddressChangeEvent();

  VOID* base = (VOID*)ImxFvbDevice.NvStorageVariableBase;
  EFI_STATUS Status = EfiConvertPointer (0, (VOID **) &base);
  if (Status == EFI_SUCCESS) {
    ImxFvbDevice.NvStorageVariableBase = (UINT64)base;
  } else {
      DEBUG((DEBUG_ERROR, "ImxNorFvb ConvertPointer BASE FAILED\n"));
}
}

void InstallIMXNorFirmwareVolumeBlockProtocol(IN EFI_SYSTEM_TABLE *SystemTable)
{
EFI_STATUS Status;

SetMem(&ImxFvbDevice, sizeof(IMX_FVB_DEVICE), 0);

ImxFvbDevice.NvStorageVariableSize = PcdGet32 (PcdFlashNvStorageVariableSize);
ImxFvbDevice.NvStorageVariableBase = (PcdGet64 (PcdFlashNvStorageVariableBase64) != 0) ?
PcdGet64 (PcdFlashNvStorageVariableBase64) : PcdGet32 (PcdFlashNvStorageVariableBase);

ImxFvbDevice.BlockSize = NOR_BLOCK_SIZE;
ImxFvbDevice.NumberOfBlocks = NOR_BLOCK_COUNT;
ImxFvbDevice.WriteCounter = 0;

ImxFvbDevice.Attributes = (EFI_FVB_ATTRIBUTES_2) (
EFI_FVB2_READ_ENABLED_CAP | // Reads may be enabled
EFI_FVB2_READ_STATUS |// Reads are currently enabled
EFI_FVB2_STICKY_WRITE |// A block erase is required to flip bits into EFI_FVB2_ERASE_POLARITY
EFI_FVB2_MEMORY_MAPPED |// It is memory mapped
EFI_FVB2_ERASE_POLARITY |// After erasure all bits take this value (i.e. '1')
EFI_FVB2_WRITE_STATUS |// Writes are currently enabled
EFI_FVB2_WRITE_ENABLED_CAP// Writes may be enabled
);

ImxFvbDevice.FvbInstance.GetAttributes = ImxNorFvbGetAttributes;
ImxFvbDevice.FvbInstance.SetAttributes = ImxNorFvbSetAttributes;
ImxFvbDevice.FvbInstance.GetPhysicalAddress = ImxNorFvbGetPhysicalAddress;
ImxFvbDevice.FvbInstance.GetBlockSize = ImxNorFvbGetBlockSize;
ImxFvbDevice.FvbInstance.Read = ImxNorFvbRead;
ImxFvbDevice.FvbInstance.Write = ImxNorFvbWrite;
ImxFvbDevice.FvbInstance.EraseBlocks = ImxNorFvbEraseBlocks;
ImxFvbDevice.FvbInstance.ParentHandle = NULL;

//
// Install protocol interface
//
Status = gBS->InstallProtocolInterface (
&ImxFvbDevice.Handle,
&gEfiFirmwareVolumeBlockProtocolGuid,
EFI_NATIVE_INTERFACE,
&ImxFvbDevice.FvbInstance
);

Status = SystemTable->BootServices->CreateEventEx (
EVT_NOTIFY_SIGNAL,
TPL_NOTIFY,
iMXNorFvb_AddressChangeEvent,
NULL,
&gEfiEventVirtualAddressChangeGuid,
&mVirtualAddressChangeEvent
);
ASSERT_EFI_ERROR (Status);
}
