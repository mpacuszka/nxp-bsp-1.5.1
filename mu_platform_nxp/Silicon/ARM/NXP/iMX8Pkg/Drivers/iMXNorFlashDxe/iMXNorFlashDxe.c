/** @file  NorFlashDxe.c

  Copyright (c) 2011 - 2021, Arm Limited. All rights reserved.<BR>
  Copyright 2023 NXP

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

/*
 NOR Flash driver for Micron flash chip on iMX boards, connected via FlexSPI module
 */

#include <Protocol/FaultTolerantWrite.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <Library/BaseLib.h>
#include <Library/PcdLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/DxeServicesTableLib.h>

#include <Guid/VariableFormat.h>
#include <Guid/SystemNvDataGuid.h>

#include "iMXNorFlashHw.h"
#include "iMX8FlexSpiLib.h"

static EFI_GUID gLocalEfiAuthenticatedVariableGuid = { 0xaaf32c78, 0x947b,
    0x439a, { 0xa1, 0x80, 0x2e, 0x14, 0x4e, 0xc3, 0x77, 0x92 } };

/**
 Initialises the FV Header and Variable Store Header
 to support variable operations.

 @param[in]  Ptr - Location to initialise the headers

 **/
EFI_STATUS InitializeFvAndVariableStoreHeaders()
{
  EFI_STATUS Status;
  VOID *Headers;
  //UINTN                               HeadersLength;
  EFI_FIRMWARE_VOLUME_HEADER *FirmwareVolumeHeader;
  VARIABLE_STORE_HEADER *VariableStoreHeader;
  UINT32 NvStorageFtwSpareSize;
  UINT32 NvStorageFtwWorkingSize;
  UINT32 NvStorageVariableSize;
  UINT64 NvStorageFtwSpareBase;
  UINT64 NvStorageFtwWorkingBase;
  UINT64 NvStorageVariableBase;

  NvStorageFtwWorkingSize = PcdGet32(PcdFlashNvStorageFtwWorkingSize);
  NvStorageFtwSpareSize = PcdGet32(PcdFlashNvStorageFtwSpareSize);
  NvStorageVariableSize = PcdGet32(PcdFlashNvStorageVariableSize);

  NvStorageFtwSpareBase =
      (PcdGet64 (PcdFlashNvStorageFtwSpareBase64) != 0) ?
          PcdGet64(PcdFlashNvStorageFtwSpareBase64) :
          PcdGet32(PcdFlashNvStorageFtwSpareBase);
  NvStorageFtwWorkingBase =
      (PcdGet64 (PcdFlashNvStorageFtwWorkingBase64) != 0) ?
          PcdGet64(PcdFlashNvStorageFtwWorkingBase64) :
          PcdGet32(PcdFlashNvStorageFtwWorkingBase);
  NvStorageVariableBase =
      (PcdGet64 (PcdFlashNvStorageVariableBase64) != 0) ?
          PcdGet64(PcdFlashNvStorageVariableBase64) :
          PcdGet32(PcdFlashNvStorageVariableBase);

  UINT32 LastBlock = (NvStorageVariableSize + NvStorageFtwWorkingSize
      + NvStorageFtwSpareSize) / NOR_BLOCK_SIZE - 1;

  EFI_GCD_MEMORY_SPACE_DESCRIPTOR GcdDescriptor;
  Status = gDS->GetMemorySpaceDescriptor(NvStorageVariableBase, &GcdDescriptor);
  if (EFI_ERROR(Status)) {
    DEBUG(
        (DEBUG_ERROR, "NvStorageVariableBase failed to get memory attribute.\n"));
  } else {
    if ((GcdDescriptor.Attributes & EFI_MEMORY_RUNTIME) == 0) {
      DEBUG(
          (DEBUG_INFO, "NvStorageVariableBase MEM: PHYS=0x%08llx LEN=0x%08llx Attr=0x%08llx Cap=0x%08llx\n", GcdDescriptor.BaseAddress, GcdDescriptor.Length, GcdDescriptor.Attributes, GcdDescriptor.Capabilities));
      Status = gDS->SetMemorySpaceAttributes(NvStorageVariableBase,
          (NvStorageVariableSize + NvStorageFtwWorkingSize
              + NvStorageFtwSpareSize),
          GcdDescriptor.Attributes | EFI_MEMORY_RUNTIME);
      if (EFI_ERROR(Status)) {
        DEBUG(
            (DEBUG_ERROR, "NvStorageVariableBase failed to add EFI_MEMORY_RUNTIME attribute to Flash area.\n"));
      } else {
        DEBUG(
            (DEBUG_INFO, "NvStorageVariableBase SUCCESS to add EFI_MEMORY_RUNTIME attribute to Flash area.\n"));
      }
    }
  }

  // FirmwareVolumeHeader->FvLength is declared to have the Variable area AND the FTW working area AND the FTW Spare contiguous.
  if ((NvStorageVariableBase + NvStorageVariableSize)
      != NvStorageFtwWorkingBase) {
    DEBUG(
        (DEBUG_ERROR, "%a: NvStorageFtwWorkingBase is not contiguous with NvStorageVariableBase region\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  if ((NvStorageFtwWorkingBase + NvStorageFtwWorkingSize)
      != NvStorageFtwSpareBase) {
    DEBUG(
        (DEBUG_ERROR, "%a: NvStorageFtwSpareBase is not contiguous with NvStorageFtwWorkingBase region\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  // Check if the size of the area is at least one block size
  if ((NvStorageVariableSize <= 0)
      || (NvStorageVariableSize / NOR_BLOCK_SIZE <= 0)) {
    DEBUG(
        (DEBUG_ERROR, "%a: NvStorageVariableSize is 0x%x, should be atleast one block size\n", __FUNCTION__, NvStorageVariableSize));
    return EFI_INVALID_PARAMETER;
  }

  if ((NvStorageFtwWorkingSize <= 0)
      || (NvStorageFtwWorkingSize / NOR_BLOCK_SIZE <= 0)) {
    DEBUG(
        (DEBUG_ERROR, "%a: NvStorageFtwWorkingSize is 0x%x, should be atleast one block size\n", __FUNCTION__, NvStorageFtwWorkingSize));
    return EFI_INVALID_PARAMETER;
  }

  if ((NvStorageFtwSpareSize <= 0)
      || (NvStorageFtwSpareSize / NOR_BLOCK_SIZE <= 0)) {
    DEBUG(
        (DEBUG_ERROR, "%a: NvStorageFtwSpareSize is 0x%x, should be atleast one block size\n", __FUNCTION__, NvStorageFtwSpareSize));
    return EFI_INVALID_PARAMETER;
  }

  // Ensure the Variable area Base Addresses are aligned on a block size boundaries
  if ((NvStorageVariableBase % NOR_BLOCK_SIZE != 0)
      || (NvStorageFtwWorkingBase % NOR_BLOCK_SIZE != 0)
      || (NvStorageFtwSpareBase % NOR_BLOCK_SIZE != 0)) {
    DEBUG(
        (DEBUG_ERROR, "%a: NvStorage Base addresses must be aligned to block size boundaries", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  EFI_FIRMWARE_VOLUME_HEADER *FvHeader =
      (EFI_FIRMWARE_VOLUME_HEADER*) NvStorageVariableBase;
  //
  // Check if the Firmware Volume is not corrupted
  //
  if ((FvHeader->Signature == EFI_FVH_SIGNATURE)
      && (CompareGuid(&gEfiSystemNvDataFvGuid, &FvHeader->FileSystemGuid))) {
    DEBUG((DEBUG_INFO, "iMX Nor Flash already valid contents\n"));
    return EFI_SUCCESS;
  } else if (FvHeader->Signature != EFI_FVH_SIGNATURE) {
    DEBUG(
        (DEBUG_INFO, "iMX Nor Flash Signature not matching: 0x%08x expected 0x%08x\n", FvHeader->Signature, EFI_FVH_SIGNATURE));
  } else if (!CompareGuid(&gEfiSystemNvDataFvGuid, &FvHeader->FileSystemGuid)) {
    DEBUG(
        (DEBUG_INFO, "iMX Nor Flash Signature not matching: %g expected %g\n", &FvHeader->FileSystemGuid, &gEfiSystemNvDataFvGuid));
  }

  //
  // EFI_FIRMWARE_VOLUME_HEADER
  //
  //HeadersLength = sizeof(EFI_FIRMWARE_VOLUME_HEADER) + sizeof(EFI_FV_BLOCK_MAP_ENTRY) + sizeof(VARIABLE_STORE_HEADER);
  //Headers = AllocateZeroPool(HeadersLength);
  Headers = (VOID*) NvStorageVariableBase;

  FirmwareVolumeHeader = (EFI_FIRMWARE_VOLUME_HEADER*) Headers;
  ZeroMem(&FirmwareVolumeHeader->ZeroVector,
      sizeof(EFI_FIRMWARE_VOLUME_HEADER));
  CopyGuid(&FirmwareVolumeHeader->FileSystemGuid, &gEfiSystemNvDataFvGuid);
  FirmwareVolumeHeader->FvLength =
  PcdGet32(PcdFlashNvStorageVariableSize) +
  PcdGet32(PcdFlashNvStorageFtwWorkingSize) +
  PcdGet32(PcdFlashNvStorageFtwSpareSize);
  FirmwareVolumeHeader->Signature = EFI_FVH_SIGNATURE;
  FirmwareVolumeHeader->Attributes =
      (EFI_FVB_ATTRIBUTES_2) (EFI_FVB2_READ_ENABLED_CAP | // Reads may be enabled
          EFI_FVB2_READ_STATUS | // Reads are currently enabled
          EFI_FVB2_STICKY_WRITE | // A block erase is required to flip bits into EFI_FVB2_ERASE_POLARITY
          EFI_FVB2_MEMORY_MAPPED | // It is memory mapped
          EFI_FVB2_ERASE_POLARITY | // After erasure all bits take this value (i.e. '1')
          EFI_FVB2_WRITE_STATUS | // Writes are currently enabled
          EFI_FVB2_WRITE_ENABLED_CAP    // Writes may be enabled
      );
  FirmwareVolumeHeader->HeaderLength = sizeof(EFI_FIRMWARE_VOLUME_HEADER)
      + sizeof(EFI_FV_BLOCK_MAP_ENTRY);
  FirmwareVolumeHeader->Revision = EFI_FVH_REVISION;
  FirmwareVolumeHeader->BlockMap[0].NumBlocks = LastBlock + 1;
  FirmwareVolumeHeader->BlockMap[0].Length = NOR_BLOCK_SIZE;
  FirmwareVolumeHeader->BlockMap[1].NumBlocks = 0;
  FirmwareVolumeHeader->BlockMap[1].Length = 0;
  FirmwareVolumeHeader->Checksum = CalculateCheckSum16(
      (UINT16*) FirmwareVolumeHeader, FirmwareVolumeHeader->HeaderLength);

  //
  // VARIABLE_STORE_HEADER
  //
  VariableStoreHeader = (VARIABLE_STORE_HEADER*) ((UINTN) Headers
      + FirmwareVolumeHeader->HeaderLength);
  CopyGuid(&VariableStoreHeader->Signature,
      &gLocalEfiAuthenticatedVariableGuid);
  VariableStoreHeader->Size = PcdGet32(PcdFlashNvStorageVariableSize)
      - FirmwareVolumeHeader->HeaderLength;
  VariableStoreHeader->Format = VARIABLE_STORE_FORMATTED;
  VariableStoreHeader->State = VARIABLE_STORE_HEALTHY;

  // Install the combined super-header in the NorFlash
  //Status = FvbWrite (&Instance->FvbProtocol, 0, 0, &HeadersLength, Headers);
  //CopyMem((void*)NvStorageVariableBase, (void*)Headers, FirmwareVolumeHeader->FvLength);

  //
  // Check if the Firmware Volume is not corrupted
  //
  if ((FvHeader->Signature != EFI_FVH_SIGNATURE)
      || (!CompareGuid(&gEfiSystemNvDataFvGuid, &FvHeader->FileSystemGuid))) {
    DEBUG(
        (DEBUG_ERROR, "Firmware Volume for Variable Store is corrupted, FvHeader=%p, Sign=0x%08lx StorBase=%p\n", FvHeader, FvHeader->Signature, (UINT8 *) (UINTN) NvStorageVariableBase));
  }

  iMXNorHW_WriteBlock(NvStorageVariableBase, 0, 0,
      FirmwareVolumeHeader->HeaderLength + sizeof(VARIABLE_STORE_HEADER));

  Status = EFI_SUCCESS;

  //FreePool (Headers);
  return Status;
}

/*
 *
 * */
EFI_STATUS
EFIAPI CopyDataFromFlash()
{
  EFI_PHYSICAL_ADDRESS NvStorageBase;
  UINT32 NvStorageSize;

  NvStorageBase = PcdGet32(PcdFlashNvStorageVariableBase);
  NvStorageSize = PcdGet32(PcdFlashNvStorageVariableSize);

  CopyMem((void*) NvStorageBase, (void*) FLASH_MAPPED_BASE, NvStorageSize);

  return EFI_SUCCESS;
}

typedef struct {
  EFI_HANDLE Handle;
  EFI_FAULT_TOLERANT_WRITE_PROTOCOL FtwInstance;
} IMX_FTW_DEVICE;

IMX_FTW_DEVICE ImxFtwDevice;

EFI_STATUS
EFIAPI ImxFtwGetMaxBlockSize(
  IN EFI_FAULT_TOLERANT_WRITE_PROTOCOL    * This,
  OUT UINTN                               *BlkSize
  )
{
  *BlkSize = 0x00040000;
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI ImxFtwAllocate(
  IN EFI_FAULT_TOLERANT_WRITE_PROTOCOL    * This,
  IN EFI_GUID                             * CallerId,
  IN UINTN                                PrivateDataSize,
  IN UINTN                                NumberOfWrites
  )
{
  DEBUG(
  (DEBUG_INFO, "iMXNor FTW ImxFtwAllocate PrivateDataSize 0x%lx NumberOfWrites 0x%lx\n", PrivateDataSize, NumberOfWrites)); return EFI_SUCCESS;
}

/**
 Retrieve the FVB protocol interface by HANDLE.

 @param[in]  FvBlockHandle     The handle of FVB protocol that provides services for
 reading, writing, and erasing the target block.
 @param[out] FvBlock           The interface of FVB protocol

 @retval EFI_SUCCESS           The interface information for the specified protocol was returned.
 @retval EFI_UNSUPPORTED       The device does not support the FVB protocol.
 @retval EFI_INVALID_PARAMETER FvBlockHandle is not a valid EFI_HANDLE or FvBlock is NULL.

 **/
static EFI_STATUS
FtwGetFvbByHandle (
IN EFI_HANDLE FvBlockHandle,
OUT EFI_FIRMWARE_VOLUME_BLOCK_PROTOCOL **FvBlock
)
{
  //
  // To get the FVB protocol interface on the handle
  //
  return gBS->HandleProtocol (
    FvBlockHandle,
    &gEfiFirmwareVolumeBlockProtocolGuid,
    (VOID **) FvBlock
  );
}

EFI_STATUS
EFIAPI ImxFtwWrite (
  IN EFI_FAULT_TOLERANT_WRITE_PROTOCOL     *This,
  IN EFI_LBA                               Lba,
  IN UINTN                                 Offset,
  IN UINTN                                 Length,
  IN VOID                                  *PrivateData,
  IN EFI_HANDLE                            FvbHandle,
  IN VOID                                  *Buffer
  )
{
  EFI_STATUS Status;
  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *Fvb;

  //DEBUG(
  //(DEBUG_INFO, "iMXNor FTW ImxFtwWrite Lba=%ld Offset=0x%lx Length=0x%lx PrivateData=0x%p Buffer=0x%p\n", Lba, Offset, Length, PrivateData, Buffer));

  //
  // Get the FVB protocol by handle
  //
  Status = FtwGetFvbByHandle (FvbHandle, &Fvb);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_INFO, "iMXNor FTW FtwGetFvbByHandle failed\n"));return Status;
  }

  while (Length > 0) {
    UINTN WriteLen = Offset + Length <= NOR_BLOCK_SIZE ? Length : NOR_BLOCK_SIZE - Offset;
    Status = Fvb->Write(Fvb, Lba, Offset, &WriteLen, Buffer);
    if (EFI_ERROR (Status)) {
      DEBUG(
          (DEBUG_INFO, "iMXNor FTW ImxFtwWrite failed\n")); return Status;
    }
    Offset = 0;
    Length -= WriteLen;
    Buffer += WriteLen;
    Lba++;
  }
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI ImxFtwRestart (
  IN EFI_FAULT_TOLERANT_WRITE_PROTOCOL     * This,
  IN EFI_HANDLE                            FvbHandle
  )
{
  DEBUG((DEBUG_INFO, "iMXNor FTW ImxFtwRestart\n")); return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
ImxFtwAbort(IN EFI_FAULT_TOLERANT_WRITE_PROTOCOL* ftw)
{
  DEBUG((DEBUG_INFO, "iMXNor FTW ImxFtwAbort\n")); return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI ImxFtwGetLastWrite(
  IN EFI_FAULT_TOLERANT_WRITE_PROTOCOL     * This,
  OUT EFI_GUID                             * CallerId,
  OUT EFI_LBA                              *Lba,
  OUT UINTN                                *Offset,
  OUT UINTN                                *Length,
  IN OUT UINTN                             *PrivateDataSize,
  OUT VOID                                 *PrivateData,
  OUT BOOLEAN                              *Complete
  )
{
  DEBUG((DEBUG_INFO, "iMXNor FTW ImxFtwGetLastWrite\n")); return EFI_SUCCESS;
}

void InstallFaultTolerantProtocol()
{
  EFI_STATUS Status;

  SetMem(&ImxFtwDevice, sizeof(IMX_FTW_DEVICE), 0);
  ImxFtwDevice.FtwInstance.GetMaxBlockSize = ImxFtwGetMaxBlockSize;
  ImxFtwDevice.FtwInstance.Allocate = ImxFtwAllocate;
  ImxFtwDevice.FtwInstance.Write = ImxFtwWrite;
  ImxFtwDevice.FtwInstance.Restart = ImxFtwRestart;
  ImxFtwDevice.FtwInstance.Abort = ImxFtwAbort;
  ImxFtwDevice.FtwInstance.GetLastWrite = ImxFtwGetLastWrite;

    //
    // Install protocol interface
    //
  Status = gBS->InstallProtocolInterface(&ImxFtwDevice.Handle,
  &gEfiFaultTolerantWriteProtocolGuid, EFI_NATIVE_INTERFACE,
  &ImxFtwDevice.FtwInstance);
  ASSERT_EFI_ERROR(Status);
}

//static VOID TestWriteToFlash()
//{
//    const char* Buf = "HELLO";
//    UINT64 NvStorageVariableBase = (PcdGet64 (PcdFlashNvStorageVariableBase64) != 0) ?
//      PcdGet64 (PcdFlashNvStorageVariableBase64) : PcdGet32 (PcdFlashNvStorageVariableBase);
//    CopyMem((char*)NvStorageVariableBase, Buf, 5);
//    CopyMem((char*)NvStorageVariableBase + 0x1000, Buf, 5);
//
//    //iMXNorHW_WriteBlock(UINT64 NvStorageVariableBase, EFI_LBA Lba, UINTN Offset, UINTN Size)
//    iMXNorHW_WriteBlock(NvStorageVariableBase, 0, 0x0, 5);
//    iMXNorHW_WriteBlock(NvStorageVariableBase, 1, 0x0, 5);
//    // Verify the writes in UEFI Shell by "mem 8000000 2000"
//}

/**
 * Initialize the iMX Nor Flash.
 *
 * @return EFI_STATUS
 */
EFI_STATUS
EFIAPI
iMXNorFlashInitialise (
IN EFI_HANDLE ImageHandle,
IN EFI_SYSTEM_TABLE *SystemTable
)
{
  EFI_STATUS Status = EFI_SUCCESS;

      //DEBUG((DEBUG_INFO, "iMXNorFlashInitialise - DRIVER ENTRY\n"));

      // In Emulated mode, do not do anything
  if (PcdGetBool (PcdEmuVariableNvModeEnable)) {
    return EFI_SUCCESS;
  }

  Status = iMXNorHW_Init();
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = CopyDataFromFlash();

      //TestWriteToFlash();
      //return Status;

  InitializeFvAndVariableStoreHeaders();
  InstallFaultTolerantProtocol();

  extern void InstallIMXNorFirmwareVolumeBlockProtocol(IN EFI_SYSTEM_TABLE *SystemTable);
  InstallIMXNorFirmwareVolumeBlockProtocol(SystemTable);

  return Status;
}
