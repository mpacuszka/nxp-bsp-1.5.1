/** @file
*
*  Copyright (c) 2013-2015, ARM Limited. All rights reserved.
*  Copyright (c) 2018, Linaro Ltd. All rights reserved.
*  Copyright (c) Microsoft Corporation. All rights reserved.
*  Copyright 2022 - 2023 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#include "iMX8DxeInternal.h"

#include <IndustryStandard/Pci.h>
#include <Protocol/DevicePathFromText.h>
#include <Protocol/PciIo.h>
#include <Protocol/PciRootBridgeIo.h>
#include <Library/UefiBootManagerLib.h>
#include <Protocol/PlatformBootManager.h>

#include <Guid/EventGroup.h>
#include <Guid/GlobalVariable.h>

#include <Library/ArmShellCmdLib.h>
#include <Library/AcpiLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DevicePathLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/NonDiscoverableDeviceRegistrationLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiBootServicesTableLib.h>

// 7E374E25-8E01-4FEE-87F2-390C23C606CD is the global GUID for the AcpiTable.  
STATIC CONST EFI_GUID mImx8AcpiTableFile = { 0x7E374E25, 0x8E01, 0x4FEE, {0x87, 0xF2, 0x39, 0x0C, 0x23, 0xC6, 0x06, 0xCD} };

#define DP_NODE_LEN(Type) { (UINT8)sizeof (Type), (UINT8)(sizeof (Type) >> 8) }

#pragma pack (1)
typedef struct {
  USB_CLASS_DEVICE_PATH    Keyboard;
  EFI_DEVICE_PATH_PROTOCOL End;
} PLATFORM_USB_KEYBOARD;
#pragma pack ()

STATIC PLATFORM_USB_KEYBOARD mUsbKeyboard = {
  //
  // USB_CLASS_DEVICE_PATH Keyboard
  //
  {
    {
      MESSAGING_DEVICE_PATH, MSG_USB_CLASS_DP,
      DP_NODE_LEN (USB_CLASS_DEVICE_PATH)
    },
    0xFFFF, // VendorId: any
    0xFFFF, // ProductId: any
    3,      // DeviceClass: HID
    1,      // DeviceSubClass: boot
    1       // DeviceProtocol: keyboard
  },

  //
  // EFI_DEVICE_PATH_PROTOCOL End
  //
  {
    END_DEVICE_PATH_TYPE, END_ENTIRE_DEVICE_PATH_SUBTYPE,
    DP_NODE_LEN (EFI_DEVICE_PATH_PROTOCOL)
  }
};

STATIC
EFI_STATUS
CreatePlatformBootOptionFromPath (
  IN     CHAR16                          *PathStr,
  IN     CHAR16                          *Description,
  IN OUT EFI_BOOT_MANAGER_LOAD_OPTION    *BootOption
  )
{
  EFI_STATUS                   Status;
  EFI_DEVICE_PATH              *DevicePath;

  DevicePath = (EFI_DEVICE_PATH *)ConvertTextToDevicePath (PathStr);
  ASSERT (DevicePath != NULL);
  Status = EfiBootManagerInitializeLoadOption (
             BootOption,
             LoadOptionNumberUnassigned,
             LoadOptionTypeBoot,
             LOAD_OPTION_ACTIVE,
             Description,
             DevicePath,
             NULL,
             0
             );
  FreePool (DevicePath);
  return Status;
}

#define IMX_BOOT_OPTION_CNT 2
STATIC
EFI_STATUS
GetPlatBootOptionsAndKeys (
  OUT UINTN                              *BootCount,
  OUT EFI_BOOT_MANAGER_LOAD_OPTION       **BootOptions,
  OUT EFI_INPUT_KEY                      **BootKeys
  )
{
  CHAR16                                 *PathStr;
  CHAR16                                 *DevDescription;
  EFI_STATUS                             Status;
  UINTN                                  Size;
  UINTN                                  BootOptCount;

  BootOptCount = 0;
  /* Allocate pool for boot options */
  Size = sizeof (EFI_BOOT_MANAGER_LOAD_OPTION) * IMX_BOOT_OPTION_CNT;
  *BootOptions = (EFI_BOOT_MANAGER_LOAD_OPTION *)AllocateZeroPool (Size);
  if (*BootOptions == NULL) {
    DEBUG ((DEBUG_ERROR, "Failed to allocate memory for BootOptions\n"));
    return EFI_OUT_OF_RESOURCES;
  }
  /* Allocate pool for boot keys */
  Size = sizeof (EFI_INPUT_KEY) * IMX_BOOT_OPTION_CNT;
  *BootKeys = (EFI_INPUT_KEY *)AllocateZeroPool (Size);
  if (*BootKeys == NULL) {
    DEBUG ((DEBUG_ERROR, "Failed to allocate memory for BootKeys\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto Error;
  }
  /* Get info for 1. Boot device from PCD and create new Boot Option.
   * This must be defined.
   */
  PathStr = (CHAR16 *)PcdGetPtr (PcdBootDevice1Path);
  DevDescription = (CHAR16 *)PcdGetPtr (PcdBootDevice1Description);
  ASSERT (PathStr != NULL);
  Status = CreatePlatformBootOptionFromPath (
             PathStr,
             DevDescription,
             &(*BootOptions)[0]
             );
  ASSERT_EFI_ERROR (Status);
  BootOptCount++;
  /* Get info for 2. Boot device from PCD and create new Boot Option
   * This is optional.
   */
  PathStr = (CHAR16 *)PcdGetPtr (PcdBootDevice2Path);
  DevDescription = (CHAR16 *)PcdGetPtr (PcdBootDevice2Description);
  if (PathStr != NULL) {
    Status = CreatePlatformBootOptionFromPath (
             PathStr,
             DevDescription,
             &(*BootOptions)[1]
             );
    ASSERT_EFI_ERROR (Status);
    BootOptCount++;
  } else {
    DEBUG ((DEBUG_INFO, "Secondary imx boot device not specified. Skipping.\n"));
  }
  *BootCount =   BootOptCount;

  return EFI_SUCCESS;
Error:
  FreePool (*BootOptions);
  return Status;
}

PLATFORM_BOOT_MANAGER_PROTOCOL mPlatformBootManager = {
  GetPlatBootOptionsAndKeys
};


EFI_STATUS
EFIAPI
iMX8EntryPoint (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS            Status;
  EFI_PHYSICAL_ADDRESS  HypBase;

  // NXP BugFix: ConIn variable must be initialized before USB stack is started else Boot Manager application will not be able to detect "ESC" key from USB keyboard
  EfiBootManagerUpdateConsoleVariable (ConIn, (EFI_DEVICE_PATH_PROTOCOL *)&mUsbKeyboard, NULL);

  //
  // Register the EHCI controllers as non-coherent
  // non-discoverable devices.
  //
  #if FixedPcdGet64 (PcdUsb1EhciBaseAddress)
  Status = RegisterNonDiscoverableMmioDevice (
             NonDiscoverableDeviceTypeEhci,
             NonDiscoverableDeviceDmaTypeNonCoherent,
             NULL,
             NULL,
             1,
             FixedPcdGet64 (PcdUsb1EhciBaseAddress),
             SIZE_1MB
             );
  ASSERT_EFI_ERROR (Status);
  #endif
  #if FixedPcdGet64 (PcdUsb2EhciBaseAddress)
  Status = RegisterNonDiscoverableMmioDevice (
             NonDiscoverableDeviceTypeEhci,
             NonDiscoverableDeviceDmaTypeNonCoherent,
             NULL,
             NULL,
             1,
             FixedPcdGet64 (PcdUsb2EhciBaseAddress),
             SIZE_1MB
             );
  ASSERT_EFI_ERROR (Status);
  #endif
  //
  // Register the XHCI controllers as non-coherent
  // non-discoverable devices.
  //
  #if FixedPcdGet64 (PcdUsb1XhciBaseAddress)
  Status = RegisterNonDiscoverableMmioDevice (
             NonDiscoverableDeviceTypeXhci,
             NonDiscoverableDeviceDmaTypeNonCoherent,
             NULL,
             NULL,
             1,
             FixedPcdGet64 (PcdUsb1XhciBaseAddress),
             SIZE_1MB
             );
  ASSERT_EFI_ERROR (Status);
  #endif

  #if FixedPcdGet64 (PcdUsb2XhciBaseAddress)
  Status = RegisterNonDiscoverableMmioDevice (
             NonDiscoverableDeviceTypeXhci,
             NonDiscoverableDeviceDmaTypeNonCoherent,
             NULL,
             NULL,
             1,
             FixedPcdGet64 (PcdUsb2XhciBaseAddress),
             SIZE_1MB
             );
  ASSERT_EFI_ERROR (Status);
  #endif

  //
  // If a hypervisor has been declared then we need to make sure its region is protected at runtime
  //
  // Note: This code is only a workaround for our dummy hypervisor (ArmPkg/Extra/AArch64ToAArch32Shim/)
  //       that does not set up (yet) the stage 2 translation table to hide its own memory to EL1.
  //
  if (FixedPcdGet32 (PcdHypFvSize) != 0) {
    // Ensure the hypervisor region is strictly contained into a EFI_PAGE_SIZE-aligned region.
    // The memory must be a multiple of EFI_PAGE_SIZE to ensure we do not reserve more memory than the hypervisor itself.
    // A UEFI Runtime region size granularity cannot be smaller than EFI_PAGE_SIZE. If the hypervisor size is not rounded
    // to this size then there is a risk some non-runtime memory could be visible to the OS view.
    if (((FixedPcdGet32 (PcdHypFvSize) & EFI_PAGE_MASK) == 0) && ((FixedPcdGet32 (PcdHypFvBaseAddress) & EFI_PAGE_MASK) == 0)) {
      // The memory needs to be declared because the DXE core marked it as reserved and removed it from the memory space
      // as it contains the Firmware.
      Status = gDS->AddMemorySpace (
          EfiGcdMemoryTypeSystemMemory,
          FixedPcdGet32 (PcdHypFvBaseAddress), FixedPcdGet32 (PcdHypFvSize),
          EFI_MEMORY_WB | EFI_MEMORY_RUNTIME
          );
      if (!EFI_ERROR (Status)) {
        // We allocate the memory to ensure it is marked as runtime memory
        HypBase = FixedPcdGet32 (PcdHypFvBaseAddress);
        Status = gBS->AllocatePages (AllocateAddress, EfiRuntimeServicesCode,
                                     EFI_SIZE_TO_PAGES (FixedPcdGet32 (PcdHypFvSize)), &HypBase);
      }
    } else {
      // The hypervisor must be contained into a EFI_PAGE_SIZE-aligned region and its size must also be aligned
      // on a EFI_PAGE_SIZE boundary (ie: 4KB).
      Status = EFI_UNSUPPORTED;
      ASSERT_EFI_ERROR (Status);
    }

    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  if(FixedPcdGet32(PcdUefiRuntimeDebugEnable)) {
    // This memory space is used by UartDebugPrint. It needs to be added as EFI_MEMORY_RUNTIME in order
    // to be able to remap the UartBase to the virtual address, see DebugLibAddressChangeEvent
    UINT64 UartBase = (UINT64)FixedPcdGet32 (PcdSerialRegisterBase);

    Status = gDS->AddMemorySpace (
      EfiGcdMemoryTypeMemoryMappedIo,
      (UINT64)UartBase, 0x1000,
      EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
      );
    ASSERT_EFI_ERROR (Status);

    Status = gDS->SetMemorySpaceAttributes (
      (UINT64)UartBase, 0x1000,
      EFI_MEMORY_UC | EFI_MEMORY_RUNTIME);
    ASSERT_EFI_ERROR (Status);
  }

  if (!FixedPcdGet32(PcdEmuVariableNvModeEnable)) {
    UINT64 FlexSPIBase = (UINT64)FixedPcdGet32 (PcdFlexSpiBaseAddress);
    Status = gDS->AddMemorySpace (
      EfiGcdMemoryTypeMemoryMappedIo,
      (UINT64)FlexSPIBase, 0x10000,
      EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
      );
    ASSERT_EFI_ERROR (Status);

    Status = gDS->SetMemorySpaceAttributes (
      (UINT64)FlexSPIBase, 0x10000,
      EFI_MEMORY_UC | EFI_MEMORY_RUNTIME);
    ASSERT_EFI_ERROR (Status);
  }
  // Install dynamic Shell command to run baremetal binaries.
  Status = ShellDynCmdRunAxfInstall (ImageHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "iMX8Dxe: Failed to install ShellDynCmdRunAxf\n"));
  }

  // Install PlatformBootManagerProtocol
  Status = gBS->InstallProtocolInterface (
                  &ImageHandle,
                  &gPlatformBootManagerProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &mPlatformBootManager
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to allocate memory for BootOptions\n"));
  }

  return Status;
}

