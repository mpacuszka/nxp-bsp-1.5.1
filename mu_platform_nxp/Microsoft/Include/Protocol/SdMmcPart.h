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
*  SD_MMC_PARTITION protocol enables switching to other then UserArea partition on MMC card.
*  This is needed e.g. for firmware writing to Boot1/Boot2 partitions.
*
**/

#ifndef __SDMMCPART_H__
#define __SDMMCPART_H__

// Global ID for the RPMB IO Protocol {210FE553-EB14-4fB1-A8C4-938E126e9ABD}
#define EFI_SDMMCPART_PROTOCOL_GUID \
  { 0x210FE553, 0xEB14, 0x4fB1, { 0xA8, 0xC4, 0x93, 0x8E, 0x12, 0x6e, 0x9A, 0xBD } };

#define EFI_SDMMCPART_PROTOCOL_REVISION  0x00010000

typedef struct _EFI_SDMMCPART_PROTOCOL EFI_SDMMCPART_PROTOCOL;

/** EFI_SDMMC_PARTITION_SWITCH

  @param[in] This Indicates a pointer to the calling context.
  @param[in] NewPartition       The partition to switch to. See enum in MMC_EXT_CSD_PARTITION_ACCESS.
  @param[out] OriginalPartition The partitoon to switch from. See enum in MMC_EXT_CSD_PARTITION_ACCESS.

  @retval EFI_SUCCESS if partition was successfully switched

  Switch to other MMC hardware partition, eg. MmcExtCsdPartitionAccessUserArea = 0,
  MmcExtCsdPartitionAccessBootPartition1 = 1,  MmcExtCsdPartitionAccessBootPartition2 = 2,  MmcExtCsdPartitionAccessRpmb = 3,...
**/
typedef
EFI_STATUS
(EFIAPI *EFI_SDMMC_PARTITION_SWITCH) (
    IN EFI_SDMMCPART_PROTOCOL   *This,
    IN UINT16   NewPartition,
    OUT UINT16* OriginalPartition
  );

struct _EFI_SDMMCPART_PROTOCOL {
  UINT64  Revision;

  // Protocol Callbacks
  EFI_SDMMC_PARTITION_SWITCH    PartitionSwitch;
};


/** DoPartitionSwitch

  @param[in] This Indicates a pointer to the calling context.
  @param[in] NewPartition       The partition to switch to. See enum in MMC_EXT_CSD_PARTITION_ACCESS.
  @param[out] OriginalPartition The partitoon to switch from. See enum in MMC_EXT_CSD_PARTITION_ACCESS.

  @retval EFI_SUCCESS if partition was successfully switched

  Switch to other MMC hardware partition, eg. MmcExtCsdPartitionAccessUserArea = 0,
  MmcExtCsdPartitionAccessBootPartition1 = 1,  MmcExtCsdPartitionAccessBootPartition2 = 2,  MmcExtCsdPartitionAccessRpmb = 3,...
**/
EFI_STATUS
DoPartitionSwitch (
  IN EFI_SDMMCPART_PROTOCOL   *This,
  IN UINT16   NewPartition,
  OUT UINT16* OriginalPartition
  );

EFI_STATUS
InstallSdMmcPartitionProtocol (IN VOID  *HostInst);

EFI_STATUS
UninstallSdMmcPartitionProtocol (IN VOID  *HostInst);


extern EFI_GUID gEfiSdMmcPartProtocolGuid;

#endif // __SDMMCPART_H__
