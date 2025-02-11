/** @file
  This file implements the Graphics Output protocol for Arm platforms

  Copyright (c) 2011-2018, ARM Ltd. All rights reserved.<BR>
  Copyright 2020 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiDxe.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DevicePathLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/TimerLib.h>

#include <Guid/GlobalVariable.h>
#include "LcdGraphicsOutputDxe.h"

//
// Global variables
//
BOOLEAN mDisplayInitialized = FALSE;

LCD_INSTANCE mLcdTemplate = {
  LCD_INSTANCE_SIGNATURE,
  NULL, // Handle
  { // ModeInfo
    0, // Version
    0, // HorizontalResolution
    0, // VerticalResolution
    PixelBltOnly, // PixelFormat
    { 0 }, // PixelInformation
    0, // PixelsPerScanLine
  },
  {
    0, // MaxMode;
    0, // Mode;
    NULL, // Info;
    0, // SizeOfInfo;
    0, // FrameBufferBase;
    0 // FrameBufferSize;
  },
  { // Gop
    LcdGraphicsQueryMode,  // QueryMode
    LcdGraphicsSetMode,    // SetMode
    LcdGraphicsBlt,        // Blt
    NULL                     // *Mode
  },
  { // DevicePath
    {
      {
        HARDWARE_DEVICE_PATH, HW_VENDOR_DP,
        {
          (UINT8)(sizeof (VENDOR_DEVICE_PATH)),
          (UINT8)((sizeof (VENDOR_DEVICE_PATH)) >> 8)
        },
      },
      // Hardware Device Path for Lcd
      EFI_CALLER_ID_GUID // Use the driver's GUID
    },

    {
      END_DEVICE_PATH_TYPE,
      END_ENTIRE_DEVICE_PATH_SUBTYPE,
      {
        sizeof (EFI_DEVICE_PATH_PROTOCOL),
        0
      }
    }
  },
  (EFI_EVENT)NULL, // ExitBootServicesEvent
};

//
// LCD instance structure constructor
//
EFI_STATUS
LcdInstanceContructor (
  OUT LCD_INSTANCE** NewInstance
  )
{
  LCD_INSTANCE* Instance;

  Instance = AllocateCopyPool (sizeof (LCD_INSTANCE), &mLcdTemplate);
  if (Instance == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  Instance->Gop.Mode          = &Instance->Mode;
  Instance->Gop.Mode->MaxMode = LcdGetVideoModesCnt ();
  Instance->Mode.Info         = &Instance->ModeInfo;

  *NewInstance = Instance;
  return EFI_SUCCESS;
}

//
// Basic display initialization (Reading EDID & non-timing configuration)
//
EFI_STATUS
InitializeDisplay (
  IN LCD_INSTANCE* Instance
  )
{
  EFI_STATUS             Status = EFI_SUCCESS;

  // Setup all the relevant mode information
  Instance->Gop.Mode->SizeOfInfo      = sizeof (EFI_GRAPHICS_OUTPUT_MODE_INFORMATION);
  Instance->Gop.Mode->FrameBufferBase = (EFI_PHYSICAL_ADDRESS)FixedPcdGet64 (PcdArmLcdDdrFrameBufferBase);

  // Basic setup
  Status = LcdInitialize (Instance->Gop.Mode->FrameBufferBase);
  if (EFI_ERROR (Status)) {
    goto EXIT_ERROR_LCD_SHUTDOWN;
  }

  // Set the flag before changing the mode, to avoid infinite loops
  mDisplayInitialized = TRUE;

  // All is ok, so don't deal with any errors
  goto EXIT;

EXIT_ERROR_LCD_SHUTDOWN:
  DEBUG ((DEBUG_ERROR, "InitializeDisplay: ERROR - Can not initialise the display. Exit Status=%r\n", Status));

  LcdShutdown ();

EXIT:
  return Status;
}

//
// Driver's entry point
//
EFI_STATUS
EFIAPI
LcdGraphicsOutputDxeInitialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS  Status = EFI_SUCCESS;
  LCD_INSTANCE* Instance;

  // Detect connected LCD display
  Status = LcdDisplayDetect ();
  if (EFI_ERROR (Status)) {
    goto EXIT;
  }

  Status = LcdInstanceContructor (&Instance);
  if (EFI_ERROR (Status)) {
    goto EXIT;
  }

  // Install the Graphics Output Protocol and the Device Path
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Instance->Handle,
                  &gEfiGraphicsOutputProtocolGuid,
                  &Instance->Gop,
                  &gEfiDevicePathProtocolGuid,
                  &Instance->DevicePath,
                  NULL
                  );

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "LcdGraphicsOutputDxeInitialize: Can not install the protocol. Exit Status=%r\n", Status));
    goto EXIT;
  }

  // Register for an ExitBootServicesEvent
  // When ExitBootServices starts, this function will make sure that the
  // graphics driver shuts down properly, i.e. it will free up all
  // allocated memory and perform any necessary hardware re-configuration.
  Status = gBS->CreateEvent (
                  EVT_SIGNAL_EXIT_BOOT_SERVICES,
                  TPL_NOTIFY,
                  LcdGraphicsExitBootServicesEvent,
                  NULL,
                  &Instance->ExitBootServicesEvent
                  );

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "LcdGraphicsOutputDxeInitialize: Can not install the ExitBootServicesEvent handler. Exit Status=%r\n", Status));
    goto EXIT_ERROR_UNINSTALL_PROTOCOL;
  }

  // To get here, everything must be fine, so just exit
  goto EXIT;

EXIT_ERROR_UNINSTALL_PROTOCOL:
  // The following function could return an error message,
  // however, to get here something must have gone wrong already,
  // so preserve the original error, i.e. don't change
  // the Status variable, even it fails to uninstall the protocol.
  gBS->UninstallMultipleProtocolInterfaces (
         Instance->Handle,
         &gEfiGraphicsOutputProtocolGuid,
         &Instance->Gop, // Uninstall Graphics Output protocol
         &gEfiDevicePathProtocolGuid,
         &Instance->DevicePath,     // Uninstall device path
         NULL
         );

EXIT:
  return Status;
}

//
// This function should be called
// on Event: ExitBootServices
// to free up memory, stop the driver
// and uninstall the protocols
//
VOID
LcdGraphicsExitBootServicesEvent (
  IN EFI_EVENT  Event,
  IN VOID       *Context
  )
{
  // By default, this PCD is FALSE. But if a platform starts a predefined OS
  // that does not use a framebuffer then we might want to disable the display
  // controller to avoid to display corrupted information on the screen.
  if (FeaturePcdGet (PcdGopDisableOnExitBootServices)) {
    // Turn-off the Display controller
    LcdShutdown ();
  }
}

//
//  GraphicsOutput Protocol function, mapping to
//  EFI_GRAPHICS_OUTPUT_PROTOCOL.QueryMode
//
EFI_STATUS
EFIAPI
LcdGraphicsQueryMode (
  IN EFI_GRAPHICS_OUTPUT_PROTOCOL            *This,
  IN UINT32                                  ModeNumber,
  OUT UINTN                                  *SizeOfInfo,
  OUT EFI_GRAPHICS_OUTPUT_MODE_INFORMATION   **Info
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  LCD_INSTANCE *Instance;

  Instance = LCD_INSTANCE_FROM_GOP_THIS (This);

  // Setup the hardware if not already done
  if (!mDisplayInitialized) {
    Status = InitializeDisplay (Instance);
    if (EFI_ERROR (Status)) {
      goto EXIT;
    }
  }

  // Error checking
  if ((This == NULL) ||
      (Info == NULL) ||
      (SizeOfInfo == NULL) ||
      (ModeNumber >= This->Mode->MaxMode)) {
    DEBUG ((DEBUG_ERROR, "LcdGraphicsQueryMode: ERROR - For mode number %d : Invalid Parameter.\n", ModeNumber));
    Status = EFI_INVALID_PARAMETER;
    goto EXIT;
  }

  *Info = AllocatePool (sizeof (EFI_GRAPHICS_OUTPUT_MODE_INFORMATION));
  if (*Info == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto EXIT;
  }

  *SizeOfInfo = sizeof (EFI_GRAPHICS_OUTPUT_MODE_INFORMATION);

  Status = LcdQueryMode (ModeNumber, *Info);
  if (EFI_ERROR (Status)) {
    FreePool (*Info);
  }

EXIT:
  return Status;
}

//
//  GraphicsOutput Protocol function, mapping to
//  EFI_GRAPHICS_OUTPUT_PROTOCOL.SetMode
//
EFI_STATUS
EFIAPI
LcdGraphicsSetMode (
  IN EFI_GRAPHICS_OUTPUT_PROTOCOL   *This,
  IN UINT32                         ModeNumber
  )
{
  EFI_STATUS                      Status = EFI_SUCCESS;
  EFI_GRAPHICS_OUTPUT_BLT_PIXEL   FillColour;
  LCD_INSTANCE*                   Instance;
  LCD_BPP                         Bpp;

  Instance = LCD_INSTANCE_FROM_GOP_THIS (This);

  // Setup the hardware if not already done
  if (!mDisplayInitialized) {
    Status = InitializeDisplay (Instance);
    if (EFI_ERROR (Status)) {
      goto EXIT;
    }
  }

  // Check if this mode is supported
  if (ModeNumber >= This->Mode->MaxMode) {
    DEBUG ((DEBUG_ERROR, "LcdGraphicsSetMode: ERROR - Unsupported mode number %d .\n", ModeNumber));
    Status = EFI_UNSUPPORTED;
    goto EXIT;
  }

  // Update the UEFI mode information
  This->Mode->Mode = ModeNumber;
  LcdQueryMode (ModeNumber, &Instance->ModeInfo);
  Status = LcdGetBpp (ModeNumber, &Bpp);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "LcdGraphicsSetMode: ERROR - Couldn't get bytes per pixel, status: %r\n", Status));
    goto EXIT;
  }
  This->Mode->FrameBufferSize =  Instance->ModeInfo.VerticalResolution
                                 * Instance->ModeInfo.PixelsPerScanLine
                                 * GetBytesPerPixel (Bpp);

  // Set the hardware to the new mode
  Status = LcdSetMode (ModeNumber);
  if (EFI_ERROR (Status)) {
    Status = EFI_DEVICE_ERROR;
    goto EXIT;
  }

  // The UEFI spec requires that we now clear the visible portions of the
  // output display to black.

  // Set the fill colour to black
  SetMem (&FillColour, sizeof (EFI_GRAPHICS_OUTPUT_BLT_PIXEL), 0x0);

  // Fill the entire visible area with the same colour.
  Status = This->Blt (
      This,
      &FillColour,
      EfiBltVideoFill,
      0,
      0,
      0,
      0,
      This->Mode->Info->HorizontalResolution,
      This->Mode->Info->VerticalResolution,
      0
      );

EXIT:
  return Status;
}

//
// Function returns number of Bytes per pixel.
//
UINTN
GetBytesPerPixel (
  IN  LCD_BPP       Bpp
  )
{
  switch (Bpp) {
  case LCD_BITS_PER_PIXEL_24:
    return 4;

  case LCD_BITS_PER_PIXEL_16_565:
  case LCD_BITS_PER_PIXEL_16_555:
  case LCD_BITS_PER_PIXEL_12_444:
    return 2;

  case LCD_BITS_PER_PIXEL_8:
  case LCD_BITS_PER_PIXEL_4:
  case LCD_BITS_PER_PIXEL_2:
  case LCD_BITS_PER_PIXEL_1:
    return 1;

  default:
    return 0;
  }
}
