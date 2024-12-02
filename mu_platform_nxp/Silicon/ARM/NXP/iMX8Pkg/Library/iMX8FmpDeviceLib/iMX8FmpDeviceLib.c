/** @file
  Provides firmware device specific services to support updates of a firmware
  image stored in a firmware device.

  Copyright 2022-2023 NXP

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <PiDxe.h>
#include <Guid/SystemResourceTable.h>
#include <Library/DebugLib.h>
#include <Library/BaseLib.h>
#include <Library/PcdLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>

#include <Protocol/BlockIo.h>
#include <Protocol/SdMmcPart.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/DevicePathLib.h>


#include "iMX8FmpDeviceLib.h"

/**
  Provide a function to install the Firmware Management Protocol instance onto a
  device handle when the device is managed by a driver that follows the UEFI
  Driver Model.  If the device is not managed by a driver that follows the UEFI
  Driver Model, then EFI_UNSUPPORTED is returned.

  @param[in] FmpInstaller  Function that installs the Firmware Management
                           Protocol.

  @retval EFI_SUCCESS      The device is managed by a driver that follows the
                           UEFI Driver Model.  FmpInstaller must be called on
                           each Driver Binding Start().
  @retval EFI_UNSUPPORTED  The device is not managed by a driver that follows
                           the UEFI Driver Model.
  @retval other            The Firmware Management Protocol for this firmware
                           device is not installed.  The firmware device is
                           still locked using FmpDeviceLock().

**/
EFI_STATUS
EFIAPI
RegisterFmpInstaller (
  IN FMP_DEVICE_LIB_REGISTER_FMP_INSTALLER  Function
  )
{
  return EFI_UNSUPPORTED;
}

/**
  Provide a function to uninstall the Firmware Management Protocol instance from a
  device handle when the device is managed by a driver that follows the UEFI
  Driver Model.  If the device is not managed by a driver that follows the UEFI
  Driver Model, then EFI_UNSUPPORTED is returned.

  @param[in] FmpUninstaller  Function that installs the Firmware Management
                             Protocol.

  @retval EFI_SUCCESS      The device is managed by a driver that follows the
                           UEFI Driver Model.  FmpUninstaller must be called on
                           each Driver Binding Stop().
  @retval EFI_UNSUPPORTED  The device is not managed by a driver that follows
                           the UEFI Driver Model.
  @retval other            The Firmware Management Protocol for this firmware
                           device is not installed.  The firmware device is
                           still locked using FmpDeviceLock().

**/
EFI_STATUS
EFIAPI
RegisterFmpUninstaller (
  IN FMP_DEVICE_LIB_REGISTER_FMP_UNINSTALLER  Function
  )
{
  return EFI_UNSUPPORTED;
}

/**
  Set the device context for the FmpDeviceLib services when the device is
  managed by a driver that follows the UEFI Driver Model.  If the device is not
  managed by a driver that follows the UEFI Driver Model, then EFI_UNSUPPORTED
  is returned.  Once a device context is set, the FmpDeviceLib services
  operate on the currently set device context.

  @param[in]      Handle   Device handle for the FmpDeviceLib services.
                           If Handle is NULL, then Context is freed.
  @param[in, out] Context  Device context for the FmpDeviceLib services.
                           If Context is NULL, then a new context is allocated
                           for Handle and the current device context is set and
                           returned in Context.  If Context is not NULL, then
                           the current device context is set.

  @retval EFI_SUCCESS      The device is managed by a driver that follows the
                           UEFI Driver Model.
  @retval EFI_UNSUPPORTED  The device is not managed by a driver that follows
                           the UEFI Driver Model.
  @retval other            The Firmware Management Protocol for this firmware
                           device is not installed.  The firmware device is
                           still locked using FmpDeviceLock().

**/
EFI_STATUS
EFIAPI
FmpDeviceSetContext (
  IN EFI_HANDLE  Handle,
  IN OUT VOID    **Context
  )
{
  return EFI_UNSUPPORTED;
}

/**
  Returns the size, in bytes, of the firmware image currently stored in the
  firmware device.  This function is used to by the GetImage() and
  GetImageInfo() services of the Firmware Management Protocol.  If the image
  size can not be determined from the firmware device, then 0 must be returned.

  @param[out] Size  Pointer to the size, in bytes, of the firmware image
                    currently stored in the firmware device.

  @retval EFI_SUCCESS            The size of the firmware image currently
                                 stored in the firmware device was returned.
  @retval EFI_INVALID_PARAMETER  Size is NULL.
  @retval EFI_UNSUPPORTED        The firmware device does not support reporting
                                 the size of the currently stored firmware image.
  @retval EFI_DEVICE_ERROR       An error occurred attempting to determine the
                                 size of the firmware image currently stored in
                                 in the firmware device.

**/
EFI_STATUS
EFIAPI
FmpDeviceGetSize (
  OUT UINTN  *Size
  )
{
  if (Size == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  *Size = 0;
  return EFI_SUCCESS;
}

/**
  Returns the GUID value used to fill in the ImageTypeId field of the
  EFI_FIRMWARE_IMAGE_DESCRIPTOR structure that is returned by the GetImageInfo()
  service of the Firmware Management Protocol.  If EFI_UNSUPPORTED is returned,
  then the ImageTypeId field is set to gEfiCallerIdGuid.  If EFI_SUCCESS is
  returned, then ImageTypeId is set to the Guid returned from this function.

  @param[out] Guid  Double pointer to a GUID value that is updated to point to
                    to a GUID value.  The GUID value is not allocated and must
                    not be modified or freed by the caller.

  @retval EFI_SUCCESS      EFI_FIRMWARE_IMAGE_DESCRIPTOR ImageTypeId GUID is set
                           to the returned Guid value.
  @retval EFI_UNSUPPORTED  EFI_FIRMWARE_IMAGE_DESCRIPTOR ImageTypeId GUID is set
                           to gEfiCallerIdGuid.

**/
EFI_STATUS
EFIAPI
FmpDeviceGetImageTypeIdGuidPtr (
  OUT EFI_GUID  **Guid
  )
{
  return EFI_UNSUPPORTED;
}

/**
  Returns values used to fill in the AttributesSupported and AttributesSettings
  fields of the EFI_FIRMWARE_IMAGE_DESCRIPTOR structure that is returned by the
  GetImageInfo() service of the Firmware Management Protocol.  The following
  bit values from the Firmware Management Protocol may be combined:
    IMAGE_ATTRIBUTE_IMAGE_UPDATABLE
    IMAGE_ATTRIBUTE_RESET_REQUIRED
    IMAGE_ATTRIBUTE_AUTHENTICATION_REQUIRED
    IMAGE_ATTRIBUTE_IN_USE
    IMAGE_ATTRIBUTE_UEFI_IMAGE

  @param[out] Supported  Attributes supported by this firmware device.
  @param[out] Setting    Attributes settings for this firmware device.

  @retval EFI_SUCCESS            The attributes supported by the firmware
                                 device were returned.
  @retval EFI_INVALID_PARAMETER  Supported is NULL.
  @retval EFI_INVALID_PARAMETER  Setting is NULL.

**/
EFI_STATUS
EFIAPI
FmpDeviceGetAttributes (
  OUT UINT64    *Supported,
  OUT UINT64    *Setting
  )
{
  if (Supported == NULL || Setting == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  *Supported = IMAGE_ATTRIBUTE_IN_USE|IMAGE_ATTRIBUTE_IMAGE_UPDATABLE|IMAGE_ATTRIBUTE_UEFI_IMAGE;
  *Setting   = IMAGE_ATTRIBUTE_IN_USE|IMAGE_ATTRIBUTE_IMAGE_UPDATABLE|IMAGE_ATTRIBUTE_UEFI_IMAGE;

  return EFI_SUCCESS;
}

/**
 * Converts version string like L"2022-06-01" into 32 bit version number
 * 0x07E60601
 * 0xYYYYMMII
 * YYYY = year in hex (7E6 = 2022)
 * MM   = month (1-12) in hex 01...0C
 * II   = issue in that months 00...ff
 *
**/

EFI_STATUS
EFIAPI
VersionStringToVersionNumber (
  IN CONST CHAR16 *VersionString,
  OUT UINT32  *Version
  )
{
    CHAR16* endPointer = NULL;

    UINTN result = 0;
    UINTN partialResult = 0;
    UINTN resultIndex = 0;

    CONST CHAR16* beginPointer = VersionString;

    do {
      RETURN_STATUS status = StrDecimalToUintnS (beginPointer, &endPointer, &partialResult);
      if (status != RETURN_SUCCESS) {
        DEBUG((DEBUG_ERROR, "Parsing VersionStringToVersionNumber failed for: %s\n", beginPointer));
        return EFI_INVALID_PARAMETER;
      }

      result <<= 8;
      result |= partialResult;

      beginPointer = endPointer + 1; //Next number starts after '-'
      resultIndex++;
    } while (*endPointer == L'-' && resultIndex < 3);

    *Version = result;

    return EFI_SUCCESS;
}

/**
  Returns the value used to fill in the LowestSupportedVersion field of the
  EFI_FIRMWARE_IMAGE_DESCRIPTOR structure that is returned by the GetImageInfo()
  service of the Firmware Management Protocol.  If EFI_SUCCESS is returned, then
  the firmware device supports a method to report the LowestSupportedVersion
  value from the currently stored firmware image.  If the value can not be
  reported for the firmware image currently stored in the firmware device, then
  EFI_UNSUPPORTED must be returned.  EFI_DEVICE_ERROR is returned if an error
  occurs attempting to retrieve the LowestSupportedVersion value for the
  currently stored firmware image.

  @note It is recommended that all firmware devices support a method to report
        the LowestSupportedVersion value from the currently stored firmware
        image.

  @param[out] LowestSupportedVersion  LowestSupportedVersion value retrieved
                                      from the currently stored firmware image.

  @retval EFI_SUCCESS       The lowest supported version of currently stored
                            firmware image was returned in LowestSupportedVersion.
  @retval EFI_UNSUPPORTED   The firmware device does not support a method to
                            report the lowest supported version of the currently
                            stored firmware image.
  @retval EFI_DEVICE_ERROR  An error occurred attempting to retrieve the lowest
                            supported version of the currently stored firmware
                            image.

**/
EFI_STATUS
EFIAPI
FmpDeviceGetLowestSupportedVersion (
  OUT UINT32  *LowestSupportedVersion
  )
{
  if (LowestSupportedVersion == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  UINTN FirmwareVerLength = StrLen (PcdGetPtr (PcdFirmwareVersionLowestSupportedString));
  if (FirmwareVerLength == 0) {
    return EFI_INVALID_PARAMETER;
  }

  CONST CHAR16* verString = PcdGetPtr (PcdFirmwareVersionLowestSupportedString);
  if (VersionStringToVersionNumber (verString, LowestSupportedVersion) != EFI_SUCCESS) {
    return EFI_INVALID_PARAMETER;
  }
  DEBUG((DEBUG_INFO, "LSV Version number parsed from %s: 0x%08x\n", verString, *LowestSupportedVersion));

  return EFI_SUCCESS;
}

/**
  Returns the Null-terminated Unicode string that is used to fill in the
  VersionName field of the EFI_FIRMWARE_IMAGE_DESCRIPTOR structure that is
  returned by the GetImageInfo() service of the Firmware Management Protocol.
  The returned string must be allocated using EFI_BOOT_SERVICES.AllocatePool().

  @note It is recommended that all firmware devices support a method to report
        the VersionName string from the currently stored firmware image.

  @param[out] VersionString  The version string retrieved from the currently
                             stored firmware image.

  @retval EFI_SUCCESS            The version string of currently stored
                                 firmware image was returned in Version.
  @retval EFI_INVALID_PARAMETER  VersionString is NULL.
  @retval EFI_UNSUPPORTED        The firmware device does not support a method
                                 to report the version string of the currently
                                 stored firmware image.
  @retval EFI_DEVICE_ERROR       An error occurred attempting to retrieve the
                                 version string of the currently stored
                                 firmware image.
  @retval EFI_OUT_OF_RESOURCES   There are not enough resources to allocate the
                                 buffer for the version string of the currently
                                 stored firmware image.

**/
EFI_STATUS
EFIAPI
FmpDeviceGetVersionString (
  OUT CHAR16  **VersionString
  )
{
  if (VersionString == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  UINTN FirmwareVerLength = StrLen (PcdGetPtr (PcdFirmwareVersionString));
  if (FirmwareVerLength == 0) {
    return EFI_INVALID_PARAMETER;
  }

  *VersionString = AllocateCopyPool (
          (FirmwareVerLength + 1) * 2,
          PcdGetPtr (PcdFirmwareVersionString)
          );
  return EFI_SUCCESS;
}

/**
  Returns the value used to fill in the Version field of the
  EFI_FIRMWARE_IMAGE_DESCRIPTOR structure that is returned by the GetImageInfo()
  service of the Firmware Management Protocol.  If EFI_SUCCESS is returned, then
  the firmware device supports a method to report the Version value from the
  currently stored firmware image.  If the value can not be reported for the
  firmware image currently stored in the firmware device, then EFI_UNSUPPORTED
  must be returned.  EFI_DEVICE_ERROR is returned if an error occurs attempting
  to retrieve the LowestSupportedVersion value for the currently stored firmware
  image.

  @note It is recommended that all firmware devices support a method to report
        the Version value from the currently stored firmware image.

  @param[out] Version  The version value retrieved from the currently stored
                       firmware image.

  @retval EFI_SUCCESS       The version of currently stored firmware image was
                            returned in Version.
  @retval EFI_UNSUPPORTED   The firmware device does not support a method to
                            report the version of the currently stored firmware
                            image.
  @retval EFI_DEVICE_ERROR  An error occurred attempting to retrieve the version
                            of the currently stored firmware image.

**/
EFI_STATUS
EFIAPI
FmpDeviceGetVersion (
  OUT UINT32  *Version
  )
{
  UINTN FirmwareVerLength = StrLen (PcdGetPtr (PcdFirmwareVersionString));
  if (FirmwareVerLength == 0) {
    return EFI_INVALID_PARAMETER;
  }

  CONST CHAR16* verString = PcdGetPtr (PcdFirmwareVersionString);
  if (VersionStringToVersionNumber (verString, Version) != EFI_SUCCESS) {
    return EFI_INVALID_PARAMETER;
  }
  DEBUG((DEBUG_INFO, "Version number parsed from %s: 0x%08x\n", verString, *Version));

  return EFI_SUCCESS;
}

/**
  Returns the value used to fill in the HardwareInstance field of the
  EFI_FIRMWARE_IMAGE_DESCRIPTOR structure that is returned by the GetImageInfo()
  service of the Firmware Management Protocol.  If EFI_SUCCESS is returned, then
  the firmware device supports a method to report the HardwareInstance value.
  If the value can not be reported for the firmware device, then EFI_UNSUPPORTED
  must be returned.  EFI_DEVICE_ERROR is returned if an error occurs attempting
  to retrieve the HardwareInstance value for the firmware device.

  @param[out] HardwareInstance  The hardware instance value for the firmware
                                device.

  @retval EFI_SUCCESS       The hardware instance for the current firmware
                            device is returned in HardwareInstance.
  @retval EFI_UNSUPPORTED   The firmware device does not support a method to
                            report the hardware instance value.
  @retval EFI_DEVICE_ERROR  An error occurred attempting to retrieve the hardware
                            instance value.

**/
EFI_STATUS
EFIAPI
FmpDeviceGetHardwareInstance (
  OUT UINT64  *HardwareInstance
  )
{
  return EFI_UNSUPPORTED;
}

/**
  Returns a copy of the firmware image currently stored in the firmware device.

  @note It is recommended that all firmware devices support a method to retrieve
        a copy currently stored firmware image.  This can be used to support
        features such as recovery and rollback.

  @param[out]     Image     Pointer to a caller allocated buffer where the
                            currently stored firmware image is copied to.
  @param[in, out] ImageSize Pointer the size, in bytes, of the Image buffer.
                            On return, points to the size, in bytes, of firmware
                            image currently stored in the firmware device.

  @retval EFI_SUCCESS            Image contains a copy of the firmware image
                                 currently stored in the firmware device, and
                                 ImageSize contains the size, in bytes, of the
                                 firmware image currently stored in the
                                 firmware device.
  @retval EFI_BUFFER_TOO_SMALL   The buffer specified by ImageSize is too small
                                 to hold the firmware image currently stored in
                                 the firmware device. The buffer size required
                                 is returned in ImageSize.
  @retval EFI_INVALID_PARAMETER  The Image is NULL.
  @retval EFI_INVALID_PARAMETER  The ImageSize is NULL.
  @retval EFI_UNSUPPORTED        The operation is not supported.
  @retval EFI_DEVICE_ERROR       An error occurred attempting to retrieve the
                                 firmware image currently stored in the firmware
                                 device.

**/
EFI_STATUS
EFIAPI
FmpDeviceGetImage (
  OUT    VOID   *Image,
  IN OUT UINTN  *ImageSize
  )
{
  return EFI_UNSUPPORTED;
}

/**
  Checks if a new firmware image is valid for the firmware device.  This
  function allows firmware update operation to validate the firmware image
  before FmpDeviceSetImage() is called.

  @param[in]  Image           Points to a new firmware image.
  @param[in]  ImageSize       Size, in bytes, of a new firmware image.
  @param[out] ImageUpdatable  Indicates if a new firmware image is valid for
                              a firmware update to the firmware device.  The
                              following values from the Firmware Management
                              Protocol are supported:
                                IMAGE_UPDATABLE_VALID
                                IMAGE_UPDATABLE_INVALID
                                IMAGE_UPDATABLE_INVALID_TYPE
                                IMAGE_UPDATABLE_INVALID_OLD
                                IMAGE_UPDATABLE_VALID_WITH_VENDOR_CODE

  @retval EFI_SUCCESS            The image was successfully checked.  Additional
                                 status information is returned in
                                 ImageUpdatable.
  @retval EFI_INVALID_PARAMETER  Image is NULL.
  @retval EFI_INVALID_PARAMETER  ImageUpdatable is NULL.

**/
EFI_STATUS
EFIAPI
FmpDeviceCheckImage (
  IN  CONST VOID  *Image,
  IN  UINTN       ImageSize,
  OUT UINT32      *ImageUpdatable
  )
{
  UINT32  LastAttemptStatus;

  return FmpDeviceCheckImageWithStatus (Image, ImageSize, ImageUpdatable, &LastAttemptStatus);
}

/**
  Checks if a new firmware image is valid for the firmware device.  This
  function allows firmware update operation to validate the firmware image
  before FmpDeviceSetImage() is called.

  @param[in]  Image               Points to a new firmware image.
  @param[in]  ImageSize           Size, in bytes, of a new firmware image.
  @param[out] ImageUpdatable      Indicates if a new firmware image is valid for
                                  a firmware update to the firmware device.  The
                                  following values from the Firmware Management
                                  Protocol are supported:
                                    IMAGE_UPDATABLE_VALID
                                    IMAGE_UPDATABLE_INVALID
                                    IMAGE_UPDATABLE_INVALID_TYPE
                                    IMAGE_UPDATABLE_INVALID_OLD
                                    IMAGE_UPDATABLE_VALID_WITH_VENDOR_CODE
  @param[out] LastAttemptStatus   A pointer to a UINT32 that holds the last attempt
                                  status to report back to the ESRT table in case
                                  of error. This value will only be checked when this
                                  function returns an error.

                                  The return status code must fall in the range of
                                  LAST_ATTEMPT_STATUS_DEVICE_LIBRARY_MIN_ERROR_CODE_VALUE to
                                  LAST_ATTEMPT_STATUS_DEVICE_LIBRARY_MAX_ERROR_CODE_VALUE.

                                  If the value falls outside this range, it will be converted
                                  to LAST_ATTEMPT_STATUS_ERROR_UNSUCCESSFUL.

  @retval EFI_SUCCESS            The image was successfully checked.  Additional
                                 status information is returned in
                                 ImageUpdatable.
  @retval EFI_INVALID_PARAMETER  Image is NULL.
  @retval EFI_INVALID_PARAMETER  ImageUpdatable is NULL.

**/
EFI_STATUS
EFIAPI
FmpDeviceCheckImageWithStatus (
  IN  CONST VOID  *Image,
  IN  UINTN       ImageSize,
  OUT UINT32      *ImageUpdatable,
  OUT UINT32      *LastAttemptStatus
  )
{
  *LastAttemptStatus = LAST_ATTEMPT_STATUS_SUCCESS;

  return EFI_SUCCESS;
}

/**
  Updates a firmware device with a new firmware image.  This function returns
  EFI_UNSUPPORTED if the firmware image is not updatable.  If the firmware image
  is updatable, the function should perform the following minimal validations
  before proceeding to do the firmware image update.
    - Validate that the image is a supported image for this firmware device.
      Return EFI_ABORTED if the image is not supported.  Additional details
      on why the image is not a supported image may be returned in AbortReason.
    - Validate the data from VendorCode if is not NULL.  Firmware image
      validation must be performed before VendorCode data validation.
      VendorCode data is ignored or considered invalid if image validation
      fails.  Return EFI_ABORTED if the VendorCode data is invalid.

  VendorCode enables vendor to implement vendor-specific firmware image update
  policy.  Null if the caller did not specify the policy or use the default
  policy.  As an example, vendor can implement a policy to allow an option to
  force a firmware image update when the abort reason is due to the new firmware
  image version is older than the current firmware image version or bad image
  checksum.  Sensitive operations such as those wiping the entire firmware image
  and render the device to be non-functional should be encoded in the image
  itself rather than passed with the VendorCode.  AbortReason enables vendor to
  have the option to provide a more detailed description of the abort reason to
  the caller.

  @param[in]  Image             Points to the new firmware image.
  @param[in]  ImageSize         Size, in bytes, of the new firmware image.
  @param[in]  VendorCode        This enables vendor to implement vendor-specific
                                firmware image update policy.  NULL indicates
                                the caller did not specify the policy or use the
                                default policy.
  @param[in]  Progress          A function used to report the progress of
                                updating the firmware device with the new
                                firmware image.
  @param[in]  CapsuleFwVersion  The version of the new firmware image from the
                                update capsule that provided the new firmware
                                image.
  @param[out] AbortReason       A pointer to a pointer to a Null-terminated
                                Unicode string providing more details on an
                                aborted operation. The buffer is allocated by
                                this function with
                                EFI_BOOT_SERVICES.AllocatePool().  It is the
                                caller's responsibility to free this buffer with
                                EFI_BOOT_SERVICES.FreePool().

  @retval EFI_SUCCESS            The firmware device was successfully updated
                                 with the new firmware image.
  @retval EFI_ABORTED            The operation is aborted.  Additional details
                                 are provided in AbortReason.
  @retval EFI_INVALID_PARAMETER  The Image was NULL.
  @retval EFI_UNSUPPORTED        The operation is not supported.

**/
EFI_STATUS
EFIAPI
FmpDeviceSetImage (
  IN  CONST VOID                                     *Image,
  IN  UINTN                                          ImageSize,
  IN  CONST VOID                                     *VendorCode,       OPTIONAL
  IN  EFI_FIRMWARE_MANAGEMENT_UPDATE_IMAGE_PROGRESS  Progress,          OPTIONAL
  IN  UINT32                                         CapsuleFwVersion,
  OUT CHAR16                                         **AbortReason
  )
{
  UINT32  LastAttemptStatus;

  return  FmpDeviceSetImageWithStatus (
            Image,
            ImageSize,
            VendorCode,
            Progress,
            CapsuleFwVersion,
            AbortReason,
            &LastAttemptStatus
            );
}

/**
  Write capsule blocks to disk

  @param[in] DevPath      The device path to write the blocks into.
  @param[in] ImageOffset  The offset to start writing.
  @param[in] Image        The bytes to write.
  @param[in] ImageSize    How many bytes to write.

  @retval EFI_SUCCESS   The display was successful.
**/
EFI_STATUS
WriteCapsule(
  IN EFI_BLOCK_IO_PROTOCOL          *BlockIo,
  IN EFI_SDMMCPART_PROTOCOL         *SdMmcPart,
  IN CONST UINT64                   ImageOffset,
  IN  CONST VOID                    *Image,
  IN  UINTN                         ImageSize,
  IN  EFI_FIRMWARE_MANAGEMENT_UPDATE_IMAGE_PROGRESS  Progress          OPTIONAL
  )
{
  EFI_STATUS                Status;
  UINT8                     *Buffer;
  UINT8                     *OriginalBuffer;
  UINTN                     BufferSize;
  UINT16                    originalPartition = (UINT16)-1;

  #define BLOCK_BATCH 20

  if ((ImageOffset % BlockIo->Media->BlockSize) != 0) {
    DEBUG((DEBUG_WARN, "WriteCapsule invalid ImageOffset: %lu\n", ImageOffset));
    goto write_capsule_cleanup;
  }


  IN CONST UINT64 Lba = ImageOffset / BlockIo->Media->BlockSize;

  BufferSize = BlockIo->Media->BlockSize * BLOCK_BATCH;
  UINT32 ioAlign = BlockIo->Media->IoAlign == 0 ? 1 : BlockIo->Media->IoAlign;

  OriginalBuffer = AllocateZeroPool(BufferSize + ioAlign);
  Buffer         = ALIGN_POINTER (OriginalBuffer,ioAlign);

  // We indicate that we want to do SdhcSwitchPartitionMmc(BootPartition1), by setting upper byte of BufferSize to 0xFF in WriteBlocks !
  UINTN remains = ImageSize;
  UINTN writeSize = BlockIo->Media->BlockSize * BLOCK_BATCH; //10kB, so 1F8000 means ~ 200 write cycles
  UINT8* ptr = (UINT8*)Image;
  UINT64 blockIndex = Lba;
  UINTN nextPercentDone = 5; //Reporting percents 5,10...100 to Progress callback function
  UINTN writtenSize = 0;

  Status = SdMmcPart->PartitionSwitch(SdMmcPart, 1 /*MmcExtCsdPartitionAccessBootPartition1*/, &originalPartition);
  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_ERROR, "Capsule partition switch failed Status=%u\n", Status));
    goto write_capsule_cleanup;
  }

  DEBUG((DEBUG_WARN, "Writing CAPSULE to Flash, please wait....\n"));
  while (remains > 0) {
	  if (writeSize >= remains) {
		  // Round up to multiple of Block Size (=512 bytes)
		  writeSize = ((remains + BlockIo->Media->BlockSize - 1) / BlockIo->Media->BlockSize) * BlockIo->Media->BlockSize;
		  // we need to Copy to buffer otherwise the WriteBlocks would touch memory after Image + ImageSize
		  CopyMem(Buffer, ptr, remains);
		  SetMem (Buffer + remains, BufferSize - remains, 0); //Set the remaining bytes to 0 in the buffer
		  ptr = Buffer;
		  writtenSize += remains;
		  remains = writeSize; //To force exiting the loop after this last writing is done
	  }
	  else
	  {
	      CopyMem(Buffer, ptr, writeSize); //ptr might not be aligned to ioAlign e.g. 4 !
	  	  writtenSize += writeSize;
	  }

	  Status = BlockIo->WriteBlocks(BlockIo, BlockIo->Media->MediaId, blockIndex, writeSize, Buffer);
	  if (EFI_ERROR(Status)) {
		  DEBUG((DEBUG_WARN, "Capsule Write failed at remains=%u Status=%u\n", remains, Status));
          FreePool (OriginalBuffer);
      Print (L"\r\nCapsule write failed\r\n");
      goto write_capsule_cleanup;
	  }

	  //typedef EFI_STATUS (EFIAPI *EFI_FIRMWARE_MANAGEMENT_UPDATE_IMAGE_PROGRESS)(IN  UINTN Completion);
	  if (Progress) {
		  UINTN percentDone = (100 * writtenSize) / ImageSize;
		  if (percentDone >= nextPercentDone) {
			  Progress(percentDone);
			  nextPercentDone = percentDone + 5;
			  Print (L"\rUpdate Progress: %3d%% ", percentDone);
		  }
	  }

	  remains -= writeSize;
	  ptr += writeSize;
	  blockIndex += BLOCK_BATCH;
  }
  Print (L"\r\nCapsule write success\r\n");
  DEBUG((DEBUG_WARN, "Write DONE\n"));

write_capsule_cleanup:
  if (originalPartition != (UINT16)-1) {
    Status = SdMmcPart->PartitionSwitch(SdMmcPart, originalPartition, NULL);
    if (EFI_ERROR(Status)) {
      DEBUG((DEBUG_ERROR, "Capsule partition switch back to OriginalPartition %d failed Status=%u\n", originalPartition, Status));
    }
  }

  if (OriginalBuffer != NULL) {
    FreePool (OriginalBuffer);
  }
  return (Status);
}

/**
  Updates a firmware device with a new firmware image.  This function returns
  EFI_UNSUPPORTED if the firmware image is not updatable.  If the firmware image
  is updatable, the function should perform the following minimal validations
  before proceeding to do the firmware image update.
    - Validate that the image is a supported image for this firmware device.
      Return EFI_ABORTED if the image is not supported.  Additional details
      on why the image is not a supported image may be returned in AbortReason.
    - Validate the data from VendorCode if is not NULL.  Firmware image
      validation must be performed before VendorCode data validation.
      VendorCode data is ignored or considered invalid if image validation
      fails.  Return EFI_ABORTED if the VendorCode data is invalid.

  VendorCode enables vendor to implement vendor-specific firmware image update
  policy.  Null if the caller did not specify the policy or use the default
  policy.  As an example, vendor can implement a policy to allow an option to
  force a firmware image update when the abort reason is due to the new firmware
  image version is older than the current firmware image version or bad image
  checksum.  Sensitive operations such as those wiping the entire firmware image
  and render the device to be non-functional should be encoded in the image
  itself rather than passed with the VendorCode.  AbortReason enables vendor to
  have the option to provide a more detailed description of the abort reason to
  the caller.

  @param[in]  Image             Points to the new firmware image.
  @param[in]  ImageSize         Size, in bytes, of the new firmware image.
  @param[in]  VendorCode        This enables vendor to implement vendor-specific
                                firmware image update policy.  NULL indicates
                                the caller did not specify the policy or use the
                                default policy.
  @param[in]  Progress          A function used to report the progress of
                                updating the firmware device with the new
                                firmware image.
  @param[in]  CapsuleFwVersion  The version of the new firmware image from the
                                update capsule that provided the new firmware
                                image.
  @param[out] AbortReason       A pointer to a pointer to a Null-terminated
                                Unicode string providing more details on an
                                aborted operation. The buffer is allocated by
                                this function with
                                EFI_BOOT_SERVICES.AllocatePool().  It is the
                                caller's responsibility to free this buffer with
                                EFI_BOOT_SERVICES.FreePool().
  @param[out] LastAttemptStatus A pointer to a UINT32 that holds the last attempt
                                status to report back to the ESRT table in case
                                of error. This value will only be checked when this
                                function returns an error.

                                The return status code must fall in the range of
                                LAST_ATTEMPT_STATUS_DEVICE_LIBRARY_MIN_ERROR_CODE_VALUE to
                                LAST_ATTEMPT_STATUS_DEVICE_LIBRARY_MAX_ERROR_CODE_VALUE.

                                If the value falls outside this range, it will be converted
                                to LAST_ATTEMPT_STATUS_ERROR_UNSUCCESSFUL.

  @retval EFI_SUCCESS            The firmware device was successfully updated
                                 with the new firmware image.
  @retval EFI_ABORTED            The operation is aborted.  Additional details
                                 are provided in AbortReason.
  @retval EFI_INVALID_PARAMETER  The Image was NULL.
  @retval EFI_UNSUPPORTED        The operation is not supported.

**/
EFI_STATUS
EFIAPI
FmpDeviceSetImageWithStatus (
  IN  CONST VOID                                     *Image,
  IN  UINTN                                          ImageSize,
  IN  CONST VOID                                     *VendorCode,       OPTIONAL
  IN  EFI_FIRMWARE_MANAGEMENT_UPDATE_IMAGE_PROGRESS  Progress,          OPTIONAL
  IN  UINT32                                         CapsuleFwVersion,
  OUT CHAR16                                         **AbortReason,
  OUT UINT32                                         *LastAttemptStatus
  )
{
  EFI_STATUS Status;

  DEBUG((DEBUG_INFO, "FmpDeviceSetImage: IMX8 (ImageSize=%u)\n", ImageSize));

  EFI_HANDLE* Buffer;
  UINTN NoHandles = 0;

  Status = gBS->LocateHandleBuffer (
                  ByProtocol,
                  &gEfiSdMmcPartProtocolGuid,
                  NULL,
                  &NoHandles,
                  &Buffer);

  DEBUG((DEBUG_WARN, "gEfiSdMmcPartProtocolGuid Status=%r Count=%lu\n", Status, NoHandles));

  if (EFI_ERROR(Status) || NoHandles != 1) {
    DEBUG((DEBUG_WARN, "Unexcpected number of SdMmcPartProtocols Guid Status=%r Count=%lu\n", Status, NoHandles));
    return EFI_NOT_FOUND;
  }

  EFI_HANDLE* MmcHandle = Buffer[0];

  EFI_BLOCK_IO_PROTOCOL     *BlockIo = NULL;
  EFI_SDMMCPART_PROTOCOL    *SdMmcPart = NULL;

  Status = gBS->OpenProtocol(MmcHandle, &gEfiBlockIoProtocolGuid, (VOID**)&BlockIo, gImageHandle, NULL, EFI_OPEN_PROTOCOL_GET_PROTOCOL);
  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_WARN, "WriteCapsule failed to open gEfiBlockIoProtocolGuid: Status=%r\n", Status));
    return EFI_NOT_FOUND;
  }

  Status = gBS->OpenProtocol(MmcHandle, &gEfiSdMmcPartProtocolGuid, (VOID**)&SdMmcPart, gImageHandle, NULL, EFI_OPEN_PROTOCOL_GET_PROTOCOL);
  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_WARN, "WriteCapsule failed to open gEfiSdMmcPartProtocolGuid: Status=%r\n", Status));
    return EFI_NOT_FOUND;
  }

  // Get the writing offset
  UINT64 imageOffset = PcdGet64 (PcdFirmwareImageOffset);

  // Write the Capsule to disk
  Status = WriteCapsule(BlockIo, SdMmcPart, imageOffset, Image, ImageSize, Progress);

  gBS->CloseProtocol(MmcHandle, &gEfiBlockIoProtocolGuid, gImageHandle, NULL);
  gBS->CloseProtocol(MmcHandle, &gEfiSdMmcPartProtocolGuid, gImageHandle, NULL);

  return Status;
}

/**
  Lock the firmware device that contains a firmware image.  Once a firmware
  device is locked, any attempts to modify the firmware image contents in the
  firmware device must fail.

  @note It is recommended that all firmware devices support a lock method to
        prevent modifications to a stored firmware image.

  @note A firmware device lock mechanism is typically only cleared by a full
        system reset (not just sleep state/low power mode).

  @retval  EFI_SUCCESS      The firmware device was locked.
  @retval  EFI_UNSUPPORTED  The firmware device does not support locking

**/
EFI_STATUS
EFIAPI
FmpDeviceLock (
  VOID
  )
{
  return EFI_UNSUPPORTED;
}
