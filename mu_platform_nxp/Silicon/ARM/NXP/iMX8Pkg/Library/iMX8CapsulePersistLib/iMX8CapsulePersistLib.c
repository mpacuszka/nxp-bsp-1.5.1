/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


/**
Implementation of the CapsulePersistLib using NorFlash.

The library has a simple interface consisting of 2 public methods:
PersistCapsule(IN EFI_CAPSULE_HEADER*)
GetPersistedCapsules(OUT EFI_CAPSULE_HEADER *CapsuleArray, OUT UINTN *CapsuleArraySize)

The Capsules are stored to Nor Flash at the offset 1 MB.
The storage starts with one 4096 bytes block containing a simple header:

typedef struct {
  UINT32 Magic;             //Fixed value of 0xDEAFBEEF to recognize a valid header
  UINT32 CapsuleCount;      //Number of capsules stored
  UINT32 CapsuleArraySize;  //Size in bytes of the stored capsules (does not include this header block of 4096 bytes)
  UINT32 Padding[5];
} CapsuleStorageHdr;

The capsules follow from 2nd block, ie at offset 1MB + 4096, which is 0x101000 in hex.
The capsules follow one right after the previous one.

The GetPersistedCapsules() method removes all the capsules from the storage,
ie. sets CapsuleCount and CapsuleArraySize to 0 in the header.

**/

#include <Library/BaseLib.h>
#include <Library/CapsulePersistLib.h>
#include <Library/DebugLib.h>
#include <Library/PrintLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Protocol/FirmwareVolumeBlock.h>
#include <Library/IoLib.h>

// Macros and definitions used in the module
static EFI_FIRMWARE_VOLUME_BLOCK_PROTOCOL* FvbProtocol = NULL;
static UINTN FlashBlockSize = 0;     //Typically 4096
static UINTN LbaCapsuleOffset = 0;   //Typically 1MB/4096 = 256
#define FLASH_MAPPED_BASE 0x08000000 //Address where contents of the Nor flash is (read-only) memory-mapped by iMXNorFlashDxe driver
#define CAPSULE_STORAGE_OFFSET_1MB 1048576
#define CAPSULE_STORAGE_OFFSET_END (16*1048576)
#define CAPSULE_STORAGE_HDR_MAGIC 0xDEAFBEEF
static EFI_GUID gEfiFVBProtocolGuid = { 0x8f644fa9, 0xe850, 0x4db1, {0x9c, 0xe2, 0xb, 0x44, 0x69, 0x8e, 0x8d, 0xa4 } };

// Forward declarations of the local (static) helper methods
static EFI_STATUS InitializeFvbProtocol ();
static EFI_STATUS GetFirmwareVolumeBlockProtocol ( OUT EFI_FIRMWARE_VOLUME_BLOCK_PROTOCOL  **FvbProt );
static BOOLEAN CapsuleStorageValid ();
static EFI_STATUS EFIAPI WriteCapsuleStorageHeader (UINT32 CapsuleCount, UINT32 CapsuleArraySize);
static EFI_STATUS EFIAPI WriteCapsule (IN EFI_CAPSULE_HEADER *CapsuleHeader);

// The Capsule storage header (located at the offset of 1 MB in NorFlash)
typedef struct {
  UINT32 Magic;             //Fixed value of 0xDEAFBEEF to recognize a valid header
  UINT32 CapsuleCount;      //Number of capsules stored
  UINT32 CapsuleArraySize;  //Size in bytes of the stored capsules (does not include this header block of 4096 bytes)
  UINT32 Padding[5];
} CapsuleStorageHdr;


/**
  Persist a Capsule across reset.

  @param[in]        CapsuleHeader     EFI_CAPSULE_HEADER pointing to Capsule Image to persist.

  @retval     EFI_SUCCESS             Capsule was successfully persisted.
  @retval     EFI_DEVICE_ERROR        Something went wrong while trying to persist the blob.

**/
EFI_STATUS
EFIAPI
PersistCapsule (
  IN EFI_CAPSULE_HEADER *CapsuleHeader
)
{
  if (PcdGetBool (PcdEmuVariableNvModeEnable)) {
    return EFI_SUCCESS;
  }

  EFI_STATUS Status;
  DEBUG((DEBUG_INFO, "PERSIST CAPSULE GUID=%g HdrSize=%u ImgSize=%u Flags=0x%08x\n",
      &CapsuleHeader->CapsuleGuid, CapsuleHeader->HeaderSize, CapsuleHeader->CapsuleImageSize, CapsuleHeader->Flags));

  Status = InitializeFvbProtocol ();
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "PERSIST CAPSULE FVB PROT GET FAILED\n"));
    return Status;
  }

  // Writing Storage header if needed
  if (!CapsuleStorageValid ()) {
    Status = WriteCapsuleStorageHeader (0, 0);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib writing storage header failed\n"));
      return Status;
    }
  }

  // Writing the capsule
  Status = WriteCapsule (CapsuleHeader);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib writing storage header failed\n"));
    return Status;
  }

  return EFI_SUCCESS;
}

/**
  Returns a pointer to a buffer of capsules.

  If no persisted capsules present, CapsuleArray is not modified, and CapsuleArraySize will be set to zero.

  Removes the persistent capsules from whatever the medium of persistence is.
  Note: if return is something other than EFI_SUCESS or EFI_BUFFER_TOO_SMALL, removal of all persistent
  capsules from persistence is not guaranteed.


  @param[out]       CapsuleArray      Pointer to a buffer to hold the capsules.
  @param[out]       CapsuleArraySize  On input, size of CapsuleArray allocation. 
                                      On output, size of actual buffer of capsules.

  @retval       EFI_SUCCESS           Capsules were de-persisted, and ouptut data is valid.
  @retval       EFI_BUFFER_TOO_SMALL  CapsuleArray buffer is too small to hold all the data.
  @retval       EFI_DEVICE_ERROR      Something went wrong while trying to retrive the capsule.

**/

EFI_STATUS
EFIAPI
GetPersistedCapsules (
  OUT EFI_CAPSULE_HEADER *CapsuleArray,
  OUT UINTN              *CapsuleArraySize
)
{
  DEBUG((DEBUG_INFO, "iMX8CapsulePersistLib GetPersistedCapsules\n"));

  if (PcdGetBool (PcdEmuVariableNvModeEnable)) {
    *CapsuleArraySize = 0;
    return EFI_SUCCESS;
  }

  EFI_STATUS Status = InitializeFvbProtocol ();
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib GetPersistedCapsules FVB get failed\n"));
    return Status;
  }

  CapsuleStorageHdr* hdr = (CapsuleStorageHdr*)(FLASH_MAPPED_BASE + CAPSULE_STORAGE_OFFSET_1MB);
  if (!CapsuleStorageValid () || hdr->CapsuleArraySize == 0) {
    *CapsuleArraySize = 0;
    return EFI_SUCCESS;
  }

  if (hdr->CapsuleArraySize > *CapsuleArraySize) {
    *CapsuleArraySize = hdr->CapsuleArraySize;
    return EFI_BUFFER_TOO_SMALL;
  }

  // Copy the Capsules into the provided buffer
  CopyMem((VOID*)CapsuleArray, (VOID*)(FLASH_MAPPED_BASE + CAPSULE_STORAGE_OFFSET_1MB + FlashBlockSize), hdr->CapsuleArraySize);

  // Clean-up the persistant storage
  Status = WriteCapsuleStorageHeader (0, 0);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib writing storage header failed\n"));
    return Status;
  }

  return EFI_SUCCESS;
}

/**
  Helper method to find the Firmware Volume Block (Fvb) interface published by iMXNorFlashDxe driver.
  This method also initializes some library variables, like LbaCapsuleOffset

  @retval     EFI_SUCCESS  Fvb protocol found.
  @retval     EFI_ERROR    Something went wrong while trying to find the Fvb protocol.

**/
static EFI_STATUS InitializeFvbProtocol()
{
  EFI_STATUS Status;

  if (FvbProtocol != NULL) {
    return EFI_SUCCESS;
  }

  Status = GetFirmwareVolumeBlockProtocol(&FvbProtocol);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib InitializeFvbProtocol FVB get failed\n"));
    return Status;
  }

  UINTN DummyNoOfBlocks = 0;
  Status = FvbProtocol->GetBlockSize(FvbProtocol, 0 /*Lba*/, &FlashBlockSize, &DummyNoOfBlocks);
  if (EFI_ERROR (Status) || FlashBlockSize == 0) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib InitializeFvbProtocol FVB GetBlockSize failed\n"));
    return Status;
  }

  LbaCapsuleOffset = CAPSULE_STORAGE_OFFSET_1MB / FlashBlockSize;

  return EFI_SUCCESS;
}

/**
  Find the relevant FirmwareVolumeBlock protocol by the given Flash address.

  @param[out] FvbProt   The result FirmwareVolumeBlockProtocol.

**/
static EFI_STATUS
GetFirmwareVolumeBlockProtocol ( OUT EFI_FIRMWARE_VOLUME_BLOCK_PROTOCOL  **FvbProt )
{
  EFI_STATUS                              Status;
  UINTN                                   Idx;
  EFI_HANDLE                              *Buffer = NULL;
  UINTN                                   NumberOfHandles;

  EFI_PHYSICAL_ADDRESS                    FvbBaseAddress;
  EFI_FIRMWARE_VOLUME_BLOCK_PROTOCOL      *Fvb;

  //
  // Read all handles into the buffer
  //
  Status = gBS->LocateHandleBuffer (
                  ByProtocol,
                  &gEfiFVBProtocolGuid,
                  NULL,
                  &NumberOfHandles,
                  &Buffer
                  );
  if (EFI_ERROR (Status)) {
    return EFI_NOT_FOUND;
  }

  // Find the FVB protocol with PhysicalAddress == PcdFlashNvStorageVariableBase
  Fvb = NULL;
  for (Idx = 0; Idx < NumberOfHandles; Idx += 1, Status = EFI_NOT_FOUND, Fvb = NULL) {
    Status = gBS->HandleProtocol (
                  Buffer[Idx],
                  &gEfiFVBProtocolGuid,
                  (VOID **) &Fvb
                  );

    if (EFI_ERROR (Status)) {
      break;
    }

    Status = Fvb->GetPhysicalAddress (Fvb, &FvbBaseAddress);
    if (EFI_ERROR (Status)) {
      continue;
    }

    // Sharing the Nor Flash with the NvVariables
    EFI_PHYSICAL_ADDRESS NvStorageVariableBase = PcdGet32 (PcdFlashNvStorageVariableBase);
    if (FvbBaseAddress == NvStorageVariableBase) {
      *FvbProt = Fvb;
      break;
    }
  }
  FreePool (Buffer);

  if (Fvb == NULL) {
    Status = EFI_NOT_FOUND;
  }

  return Status;
}

#define CAPSULE_BUF_SIZE 4096
static UINT8 CapsuleBuffer[CAPSULE_BUF_SIZE];

/**
  Write single block into the persistent storage.
  The block may consist of Head, Body and Tail.

  |--------Head--------|------------Body-----------|-----Tail-----|

  @param[in] Lba Logical block address =  index of the block in storage
  @param[in] Buffer The input data
  @param[in] Length The total remaining length of the input data
  @param[in] OffsetInBlock The offset in the block where to write. This can be non-zero only for the first block of capsule.

  @retval EFI_SUCCESS  It successfully writen.
  @retval other values If writing failed.
**/

static EFI_STATUS
EFIAPI WriteSingleBlock (UINT32 Lba, UINT8* Buffer, UINT32 Length, UINT32 OffsetInBlock, UINT32* DataWritten)
{
  EFI_STATUS Status;

  BOOLEAN Head = OffsetInBlock > 0;
  BOOLEAN Tail = OffsetInBlock + Length < FlashBlockSize;

  UINT8* WritePtr = Buffer;
  UINT32 DataLen = FlashBlockSize;

  // The data for Head or Tail are copy of the current content of the Flash mem-mapped to FLASH_MAPPED_BASE
  // In this case we write from CapsuleBuffer instead from the input Buffer
  if (Head || Tail) {
    // 1. HEAD and TAIL data
    CopyMem((VOID*)CapsuleBuffer, (VOID*)(FLASH_MAPPED_BASE + Lba*FlashBlockSize), CAPSULE_BUF_SIZE);

    // 2. BODY data
    DataLen = Tail ? Length : FlashBlockSize - OffsetInBlock;
    CopyMem((((VOID*)CapsuleBuffer) + OffsetInBlock), Buffer, DataLen);

    WritePtr = CapsuleBuffer;
  }

  // Send the whole block to the Flash
  UINTN WriteLen = FlashBlockSize;
  Status = FvbProtocol->Write(FvbProtocol, Lba, 0, &WriteLen, WritePtr);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib WriteSingleBlock FtwWrite failed\n"));
    return Status;
  }

  *DataWritten = DataLen;

  return EFI_SUCCESS;
}

/**
  Write a Buffer of Length bytes into the persistent storage at offset.
  The block may consist of Head, Body and Tail.

  |--------Head--------|------------Body-----------|-----Tail-----|

  @param[in] Buffer The input data
  @param[in] Length The length of the input data
  @param[in] Offset The offset in the Flash storage for Capsules (from 1MB).

  @retval EFI_SUCCESS  It successfully written.
  @retval other values If writing failed.
**/

static EFI_STATUS
EFIAPI WriteToStorage (UINT8* Buffer, UINT32 Length, UINT32 Offset)
{
  EFI_STATUS Status;

  UINT32 Lba = Offset / FlashBlockSize + LbaCapsuleOffset; //The capsule header starts at 1MB offset
  UINT32 OffsetInBlock = Offset % FlashBlockSize;
  UINT32 DataWritten = 0;

  // One loop for each Block (4096) bytes to write
  while (Length > 0) {

    Status = WriteSingleBlock (Lba, Buffer, Length, OffsetInBlock, &DataWritten);
    if (EFI_ERROR (Status)) {
      return Status;
    }
    OffsetInBlock = 0;
    Length -= DataWritten;
    Buffer += DataWritten;
    Lba++;
  }
  return EFI_SUCCESS;
}

/**
  Write new empty capsule storage header.

  @retval EFI_SUCCESS  It successfully written.
  @retval other values If writing failed.
**/
static EFI_STATUS
EFIAPI WriteCapsuleStorageHeader (UINT32 CapsuleCount, UINT32 CapsuleArraySize)
{
  ZeroMem(CapsuleBuffer, CAPSULE_BUF_SIZE);

  CapsuleStorageHdr* hdr = (CapsuleStorageHdr*)CapsuleBuffer;
  hdr->Magic = CAPSULE_STORAGE_HDR_MAGIC;
  hdr->CapsuleCount = CapsuleCount;
  hdr->CapsuleArraySize = CapsuleArraySize;

  return WriteToStorage (CapsuleBuffer, CAPSULE_BUF_SIZE, 0);
}

/**
  Validate, that the contents of the persistent capsule storage is consistent.

  @retval TRUE  If consistent.
  @retval FALSE If not consistent.
**/
static BOOLEAN CapsuleStorageValid ()
{
  CapsuleStorageHdr* hdr = (CapsuleStorageHdr*)(FLASH_MAPPED_BASE + CAPSULE_STORAGE_OFFSET_1MB);
  if (hdr->Magic != CAPSULE_STORAGE_HDR_MAGIC) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib Header MAGIC not matching, expected: 0x%08x inflash: 0x%08x\n",
        CAPSULE_STORAGE_HDR_MAGIC, hdr->Magic));
    return FALSE;
  }
  UINT32 capsuleCnt = hdr->CapsuleCount;
  UINT32 capsuleStartOffset = FlashBlockSize;

  DEBUG((DEBUG_INFO, "iMX8CapsulePersistLib CapsuleCount = %u Offset: %u\n", capsuleCnt, capsuleStartOffset));

  while (capsuleCnt > 0) {
    EFI_CAPSULE_HEADER *CapsuleHeader = (EFI_CAPSULE_HEADER *)(UINTN)(FLASH_MAPPED_BASE + CAPSULE_STORAGE_OFFSET_1MB + capsuleStartOffset);
    if (capsuleStartOffset + CapsuleHeader->CapsuleImageSize > FlashBlockSize + hdr->CapsuleArraySize) {
      DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib CapSize[%u] too big, max: %u inflash: %u\n",
          hdr->CapsuleCount-capsuleCnt, FlashBlockSize + hdr->CapsuleArraySize, capsuleStartOffset + CapsuleHeader->CapsuleImageSize));
      return FALSE;
    }
    capsuleStartOffset += CapsuleHeader->CapsuleImageSize;
    capsuleCnt--;
  }

  if (FlashBlockSize + hdr->CapsuleArraySize != capsuleStartOffset) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib Header invalid CapsuleArraySize: %u expected: %u\n", FlashBlockSize + hdr->CapsuleArraySize, capsuleStartOffset));
    return FALSE;
  }

  return TRUE;
}

/**
  Write a single capsule into the storage at the correct offset.

  @param[in]    CapsuleHeader      Pointer to the capsule to store

  @retval       EFI_SUCCESS           Capsule was persisted.
  @retval       EFI_DEVICE_ERROR      Something went wrong.

**/

static EFI_STATUS
EFIAPI WriteCapsule (IN EFI_CAPSULE_HEADER *CapsuleHeader)
{
  EFI_STATUS Status;

  // Get the Storage header and verify MAGIC
  CapsuleStorageHdr* hdr = (CapsuleStorageHdr*)(FLASH_MAPPED_BASE + CAPSULE_STORAGE_OFFSET_1MB);
  if (hdr->Magic != CAPSULE_STORAGE_HDR_MAGIC) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib Header MAGIC not matching, expected: 0x%08x inflash: 0x%08x\n",
        CAPSULE_STORAGE_HDR_MAGIC, hdr->Magic));
    return EFI_NOT_READY;
  }

  // Write the Capsule
  Status = WriteToStorage ((UINT8*)CapsuleHeader, CapsuleHeader->CapsuleImageSize, FlashBlockSize + hdr->CapsuleArraySize);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib Writing capsule failed\n"));
    return Status;
  }

  // Write the updated Storage header
  Status = WriteCapsuleStorageHeader (hdr->CapsuleCount + 1, hdr->CapsuleArraySize + CapsuleHeader->CapsuleImageSize);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "iMX8CapsulePersistLib WriteCapsuleStorageHeader failed\n"));
    return Status;
  }

  return EFI_SUCCESS;
}
