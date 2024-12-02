/*
 * Copyright 2022 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
*
*  SD_MMC_PARTITION protocol is an interface for switching MMC Hardware to Boot1
*  partition for writing the Firmware, and back to User partition after firmware
*  update is done.
*
**/

#include <Uefi.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include <Protocol/BlockIo.h>
#include <Protocol/DevicePath.h>
#include <Protocol/RpmbIo.h>
#include <Protocol/SdMmcPart.h>
#include <Protocol/Sdhc.h>

#include "SdMmcHw.h"
#include "SdMmc.h"
#include "Protocol.h"

BOOLEAN
EFIAPI
IsSdMmcPartitionInstalledOnTheSystem (
  VOID
  )
{
  EFI_SDMMCPART_PROTOCOL  SdMmcPartitionProtocol;
  EFI_STATUS Status;

  Status = gBS->LocateProtocol (
    &gEfiSdMmcPartProtocolGuid,
    NULL,
    (VOID **) &SdMmcPartitionProtocol);
  if (EFI_ERROR (Status)) {
    return FALSE;
  }

  return TRUE;
}


EFI_STATUS
InstallSdMmcPartitionProtocol (IN VOID  *_HostInst)
{
  SDHC_INSTANCE  *HostInst = (SDHC_INSTANCE*)_HostInst;

  EFI_STATUS Status;
  if (IsSdMmcPartitionInstalledOnTheSystem())
  {
    return EFI_SUCCESS;
  }

  Status = gBS->InstallMultipleProtocolInterfaces (
	  &HostInst->MmcHandle,
	  &gEfiSdMmcPartProtocolGuid,
	  &HostInst->SdMmcPart,
	  NULL);

  if (EFI_ERROR (Status)) {
    LOG_ERROR (
      "InstallSdMmcPartitionProtocol(): Failed installing EFI_SD_MMC_PARTITION_PROTOCOL interface. %r",
      Status);

    return Status;
  }
  LOG_INFO ("SdMmcPartition protocol installed for MMC");

  HostInst->SdMmcPartitionProtocolInstalled = TRUE;
  return EFI_SUCCESS;
}

EFI_STATUS
UninstallSdMmcPartitionProtocol (IN VOID  *_HostInst)
{
  SDHC_INSTANCE  *HostInst = (SDHC_INSTANCE*)_HostInst;
  EFI_STATUS Status;
  if (HostInst->SdMmcPartitionProtocolInstalled) {
    Status = gBS->UninstallMultipleProtocolInterfaces (
        HostInst->MmcHandle,
        &gEfiSdMmcPartProtocolGuid,
        &HostInst->SdMmcPart,
        NULL);

    if (EFI_ERROR (Status)) {
      LOG_ERROR (
        "UninstallSdMmcPartitionProtocol(): Failed to uninstall EFI_SDMMCPART_PROTOCOL_GUID interface. %r",
        Status);
      return Status;
    } else {
      HostInst->SdMmcPartitionProtocolInstalled = FALSE;
    }
  }
  return EFI_SUCCESS;
}

EFI_STATUS
DoPartitionSwitch (
  IN EFI_SDMMCPART_PROTOCOL   *This,
  IN UINT16   NewPartition,
  OUT UINT16* OriginalPartition
  )
{
  SDHC_INSTANCE                 *HostInst;
  MMC_EXT_CSD_PARTITION_ACCESS  Partition;

  ASSERT (This);

  HostInst = SDHC_INSTANCE_FROM_SDMMCPART_THIS (This);
  ASSERT (HostInst);
  ASSERT (HostInst->HostExt);

  if (OriginalPartition != NULL) {
    *OriginalPartition = (UINT16)HostInst->CurrentMmcPartition;
  }
  Partition = (MMC_EXT_CSD_PARTITION_ACCESS) NewPartition;

  EFI_STATUS SwitchStatus = SdhcSwitchPartitionMmc (HostInst, Partition);
  if (EFI_ERROR (SwitchStatus)) {
    DEBUG((DEBUG_INFO, "SdMmcPart SdhcSwitchPartitionMmc FAILED\n"));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}
