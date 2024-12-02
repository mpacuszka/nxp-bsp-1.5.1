#
# NXP iMX93 EVK board description
#
# The board is iMX93 with 2GB DRAM
#
#  Copyright (c) 2018, Microsoft Corporation. All rights reserved.
#  Copyright (c) 2013-2018, ARM Limited. All rights reserved.
#  Copyright 2019-2020, 2023 NXP
#
#  This program and the accompanying materials
#  are licensed and made available under the terms and conditions of the BSD License
#  which accompanies this distribution.  The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#


################################################################################
#
# Defines Section - statements that will be processed to create a Makefile.
#
################################################################################
[Defines]
  DEFINE BOARD_NAME       = MX93_11X11_EVK
  DEFINE IMX_FAMILY       = IMX93
  DEFINE FIRMWARE_VER     = 2020-02

  PLATFORM_NAME           = iMX8
  PLATFORM_GUID           = CB9441D6-BA9D-458E-BB02-341C12B7FB87
  PLATFORM_VERSION        = 0.1
  DSC_SPECIFICATION       = 0x00010019
  OUTPUT_DIRECTORY        = Build/$(BOARD_NAME)
  SUPPORTED_ARCHITECTURES = AARCH64
  BUILD_TARGETS           = DEBUG|RELEASE
  SKUID_IDENTIFIER        = DEFAULT
  BOARD_DIR               = NXP/$(BOARD_NAME)
  FLASH_DEFINITION        = $(BOARD_DIR)/$(BOARD_NAME).fdf

################################################################################
#
# includes Section - statements common to all iMX8 boards
#
################################################################################

# Include common peripherals
!include iMX8Pkg/iMX8CommonDsc.inc
!if $(CONFIG_FRONTPAGE) == TRUE
!include FrontpageDsc.inc
!endif

################################################################################
#
# Board specific Section - entries specific to this Platform
#
################################################################################
[LibraryClasses.common]
  ArmPlatformLib|$(BOARD_DIR)/Library/iMX8BoardLib/iMX8BoardLib.inf
  SerialPortLib|iMXPlatformPkg/Library/LPUartSerialPortLib/LPUartSerialPortLib.inf
  iMXLpi2cLib|iMXPlatformPkg/Library/iMXLpi2cLib/iMXLpi2cLib.inf

!if $(CONFIG_HEADLESS) != TRUE
  LcdHwLib|iMX8Pkg/Library/iMX8LcdHwLib/iMX93DisplayHwLib.inf
!endif

[Components.common]
  #
  # ACPI Support
  #
  MdeModulePkg/Universal/Acpi/AcpiTableDxe/AcpiTableDxe.inf
  MdeModulePkg/Universal/Acpi/AcpiPlatformDxe/AcpiPlatformDxe.inf
  $(BOARD_DIR)/AcpiTables/AcpiTables.inf

  #
  # Usb Support
  #
!if $(CONFIG_USB) == TRUE
  MdeModulePkg/Bus/Pci/EhciDxe/EhciDxe.inf
!endif
  #
  # SMBIOS/DMI
  #
  MdeModulePkg/Universal/SmbiosDxe/SmbiosDxe.inf
  $(BOARD_DIR)/Drivers/SmbiosPlatformDxe/SmbiosPlatformDxe.inf

  #
  # FPDT
  #
  #Enable ReportStatusCodeRouterRuntimeDxe & FirmwarePerformanceDxe for UEFI Performance
  MdeModulePkg/Universal/ReportStatusCodeRouter/RuntimeDxe/ReportStatusCodeRouterRuntimeDxe.inf
  MdeModulePkg/Universal/Acpi/FirmwarePerformanceDataTableDxe/FirmwarePerformanceDxe.inf {
    <LibraryClasses>
      LockBoxLib|MdeModulePkg/Library/LockBoxNullLib/LockBoxNullLib.inf
  }

  #Enable DpApp to visualize UEFI Performance
  ShellPkg/DynamicCommand/DpDynamicCommand/DpApp.inf
################################################################################
#
# Pcd Section - list of all EDK II PCD Entries defined by this Platform
#
################################################################################

[PcdsFeatureFlag.common]
  ## If TRUE, Graphics Output Protocol will be installed on virtual handle created by ConsplitterDxe.
  #  It could be set FALSE to save size.
  gEfiMdeModulePkgTokenSpaceGuid.PcdConOutGopSupport|TRUE


[PcdsFixedAtBuild.common]
  # +-------------------+===> (0x80200000) PcdTrustZonePrivateMemoryBase (OPTEE image base address)
  # | TZ Private Memory |  ^
  # | (OPTEE)           |  |  (0x01E00000) PcdTrustZonePrivateMemorySize 30MB
  # |                   |  v
  # +-------------------+===> (0x82000000) PcdTrustZoneSharedMemoryBase (includes mobj bookkeeping page)
  # | TZ Shared Memory  |  ^
  # |                   |  |  (0x00200000) PcdTrustZoneSharedMemorySize 2MB
  # |                   |  v
  # +-------------------+===> (0x82200000) Uboot
  # | Uboot             |
  # |                   |                  PcdSystemMemoryBase
  # +-------------------+===> (0x82300000) PcdGlobalDataBaseAddress
  # | Global Data       |  ^
  # |                   |  |  (0x00001000) PcdGlobalDataSize 4KB
  # |                   |  v
  # +-------------------+===> (0x82301000) PcdTpm2AcpiBufferBase
  # | TPM               |  ^
  # |                   |  |  (0x00003000) PcdTpm2AcpiBufferSize
  # |                   |  v
  # +-------------------+===> (0x82304000) RAM1 region base
  # | Operating System  |  ^
  # | Memory            |  |  (0x7CCFC000) RAM1 region size
  # |                   |  |
  # |                   |  v
  # +-------------------+===> (0xff000000) PcdArmLcdDdrFrameBufferBase (Frame buffer 1920x1080x32)
  # | HDMI framebuffer  |  ^
  # |                   |  |  (0x01000000) PcdArmLcdDdrFrameBufferSize ~16MB
  # |                   |  v
  # +-------------------+===> (0xFFFFFFFF) 

!if $(CONFIG_OPTEE) == TRUE
  gOpteeClientPkgTokenSpaceGuid.PcdTrustZonePrivateMemoryBase|0x80200000
  gOpteeClientPkgTokenSpaceGuid.PcdTrustZonePrivateMemorySize|0x01E00000

  #
  # TrustZone shared memory (2MB)
  # This memory is managed by the normal world but shared with the OpTEE OS.
  # It must match OpTEE optee_os/core/arch/arm/plat-imx/platform_config.h:
  #    CFG_SHMEM_START & CFG_SHMEM_SIZE
  # NOTE: The first page of the SHMEM is owned by OPTEE for mobj bookkeeping
  # and we should not touch it. We will skip the first 4K of SHMEM and take that
  # into account for SHMEM size in PcdTrustZoneSharedMemorySize.
  #
  gOpteeClientPkgTokenSpaceGuid.PcdTrustZoneSharedMemoryBase|0x82000000
  gOpteeClientPkgTokenSpaceGuid.PcdTrustZoneSharedMemorySize|0x00200000
!endif

  # FirmwareRevision 0.1
  gEfiMdeModulePkgTokenSpaceGuid.PcdFirmwareRevision|0x00000001

  # System Memory base
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x82300000
  giMX8TokenSpaceGuid.PcdBank1MemoryBase|0x00000000FF000000
!if $(CONFIG_HEADLESS) == TRUE
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x7DD00000
  giMX8TokenSpaceGuid.PcdBank1MemorySize|0x0000000001000000
!else
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x7CD00000
  giMX8TokenSpaceGuid.PcdBank1MemorySize|0x0000000000000000
!endif

  # GOP driver memory
  # Global data area
  giMXPlatformTokenSpaceGuid.PcdGlobalDataBaseAddress|0x82300000
  giMXPlatformTokenSpaceGuid.PcdGlobalDataSize|0x1000

  # Reserved for TPM2 ACPI
  gOpteeClientPkgTokenSpaceGuid.PcdTpm2AcpiBufferBase|0x82301000
  gOpteeClientPkgTokenSpaceGuid.PcdTpm2AcpiBufferSize|0x3000
!if $(CONFIG_HEADLESS) == FALSE
  # Frame buffer
  gArmPlatformTokenSpaceGuid.PcdArmLcdDdrFrameBufferBase|0xFF000000
  gArmPlatformTokenSpaceGuid.PcdArmLcdDdrFrameBufferSize|0x01000000
!endif

  #
  # NV Storage PCDs. Use base of 0x30370000 for SNVS?
  #
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageVariableBase|0x00000000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageVariableSize|0x00000000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwWorkingBase|0x00000000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwWorkingSize|0x00000000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwSpareBase|0x00000000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwSpareSize|0x00000000

  # i.MX93
  gArmPlatformTokenSpaceGuid.PcdCoreCount|2
  gArmPlatformTokenSpaceGuid.PcdClusterCount|1

  gArmTokenSpaceGuid.PcdVFPEnabled|1

  # i.MX93 Display configuration
  #  PcdDisplayEnable      - enable display, default enabled
  #  PcdDisplayInterface   - default display interface, 1=MIPI_DSI, 2=BOE EV121WXM-N12 LVDS panel
  #  PcdDisplayI2CBaseAddr - use I2C1 on iMX93
  #  PcdDisplayReadEDID    - TRUE = read EDID from display if available. If not available, set fixed resolution (default MIPI-DSI 1920x1080@60).
  #                                 Not available for LVDS panel, it is using fixed 1280x800@60 resolution.
  #                        - FALSE = skip EDID reading (even if available) and set fixed resolution (default MIPI-DSI 1920x1080@60, LVDS 1280x800@60)
  #  PcdDisplayForceConverterMaxResolution - mipi-hdmi and lvds-hdmi converters have max allowed resolution 1920x1080 (single LVDS limit is 1280x800)
  #                                        - TRUE - if EDID data read exceeds the limit, force set fixed resolution for converters only
  #                                        - FALSE - Use EDID data read even if exceeds the limit
  #                                        - if PcdDisplayReadEDID = FALSE, this option has no effect
  giMX8TokenSpaceGuid.PcdDisplayEnable|TRUE
  giMX8TokenSpaceGuid.PcdDisplayInterface|1
  giMX8TokenSpaceGuid.PcdDisplayI2CBaseAddr|0x44340000
  giMX8TokenSpaceGuid.PcdDisplayReadEDID|TRUE
  giMX8TokenSpaceGuid.PcdDisplayForceConverterMaxResolution|TRUE

  #
  # iMXPlatformPkg
  #

  ## iMXPlatformPackage - Serial Terminal
  giMXPlatformTokenSpaceGuid.PcdSerialRegisterBase|0x44380000

  ## iMXPlatformPackage - Debug UART instance LPUART1 0x44380000
  giMXPlatformTokenSpaceGuid.PcdKdUartInstance|1

  # uSDHCx | iMX93 MEK Connections
  #-------------------------------------
  # uSDHC1 | eMMC
  # uSDHC2 | SD Card slot
  # uSDHC3 | N/A
  # uSDHC4 | N/A
  #
  giMXPlatformTokenSpaceGuid.PcdSdhc1Base|0x42850000
  giMXPlatformTokenSpaceGuid.PcdSdhc2Base|0x42860000
  giMXPlatformTokenSpaceGuid.PcdSdhc3Base|0x428B0000
  giMXPlatformTokenSpaceGuid.PcdSdhc4Base|0x00000000

  giMXPlatformTokenSpaceGuid.PcdSdhc1Enable|TRUE
  giMXPlatformTokenSpaceGuid.PcdSdhc2Enable|TRUE
  giMXPlatformTokenSpaceGuid.PcdSdhc3Enable|FALSE
  giMXPlatformTokenSpaceGuid.PcdSdhc4Enable|FALSE

  giMXPlatformTokenSpaceGuid.PcdSdhc1CardDetectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc1WriteProtectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc1DeviceType|0x8
  giMXPlatformTokenSpaceGuid.PcdSdhc2CardDetectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc2WriteProtectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc2DeviceType|0x1
  giMXPlatformTokenSpaceGuid.PcdSdhc3CardDetectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc3WriteProtectSignal|0xFF01
  giMXPlatformTokenSpaceGuid.PcdSdhc4CardDetectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc4WriteProtectSignal|0xFF01

  ## SBSA Watchdog Count
!ifndef DISABLE_SBSA_WATCHDOG
  gArmPlatformTokenSpaceGuid.PcdWatchdogCount|2
!endif

  #
  # ARM Generic Interrupt Controller
  #
  gArmTokenSpaceGuid.PcdGicDistributorBase|0x48000000
  gArmTokenSpaceGuid.PcdGicRedistributorsBase|0x48040000

!if TRUE == FALSE
  #
  # Watchdog
  #
  gArmTokenSpaceGuid.PcdGenericWatchdogControlBase|0x00000000
#  gArmTokenSpaceGuid.PcdGenericWatchdogRefreshBase| *** unused ***
  gArmTokenSpaceGuid.PcdGenericWatchdogEl2IntrNum|110
!endif

  #
  # ARM Architectural Timer Frequency
  #
  gEmbeddedTokenSpaceGuid.PcdMetronomeTickPeriod|1000

  gEfiMdeModulePkgTokenSpaceGuid.PcdResetOnMemoryTypeInformationChange|FALSE

  #
  # SMBIOS entry point version
  #
  gEfiMdeModulePkgTokenSpaceGuid.PcdSmbiosVersion|0x0300
  gEfiMdeModulePkgTokenSpaceGuid.PcdSmbiosDocRev|0x0

  # Offset where firmware.bin starts in boot partition, see also imx-windows-iot\BSP\firmware\flash_bootloader.cmd
  gEfiMdeModulePkgTokenSpaceGuid.PcdFirmwareImageOffset|0x00000000

  # EXP_SEL signal setting for MX93 board
  #   TRUE  - EXP_SEL = 1: SPI3, SAI3, CAN2 to J1001 (RPi/EXP GPIO)
  #   FALSE - EXP_SEL = 0: SPI3 to M.2, SAI3 to CODEC, CAN2 to J1101
  giMX8TokenSpaceGuid.PcdMX93EXPSelSetting|FALSE

  #
  # Camera
  #
  # Configuration of camera type connected to CSI port
  giMX8TokenSpaceGuid.PcdCsi1CameraRpiCamMipi|0x0

  #
  # USB
  #
  giMX8TokenSpaceGuid.PcdUsb1EhciBaseAddress|0x4C100100
  giMX8TokenSpaceGuid.PcdUsb2EhciBaseAddress|0x4C200100

  # Boot options
  giMX8TokenSpaceGuid.PcdBootDevice1Path|L"VenHw(AAFB8DAA-7340-43AC-8D49-0CCE14812489,02000000)/SD(0x0)"
  giMX8TokenSpaceGuid.PcdBootDevice1Description|L"Boot from SD Card"
  giMX8TokenSpaceGuid.PcdBootDevice2Path|L"VenHw(AAFB8DAA-7340-43AC-8D49-0CCE14812489,01000000)/eMMC(0x0)"
  giMX8TokenSpaceGuid.PcdBootDevice2Description|L"Boot from eMMC"

[PcdsPatchableInModule]
  # Use system default resolution
  gEfiMdeModulePkgTokenSpaceGuid.PcdVideoHorizontalResolution|0
  gEfiMdeModulePkgTokenSpaceGuid.PcdVideoVerticalResolution|0

