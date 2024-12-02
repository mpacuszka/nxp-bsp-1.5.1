#
# NXP iMX8M Nano EVK board description
#
# The board is iMX8M_Nano with 2GB DRAM
#
#  Copyright (c) 2018, Microsoft Corporation. All rights reserved.
#  Copyright (c) 2013-2018, ARM Limited. All rights reserved.
#  Copyright 2019-2023 NXP
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
  DEFINE BOARD_NAME       = MX8M_NANO_EVK
  DEFINE IMX_FAMILY       = IMX8MN
  DEFINE FIRMWARE_VER     = 2020-02

  PLATFORM_NAME           = iMX8
  PLATFORM_GUID           = 8E3541B9-586D-4A60-90EB-AFF04EC3A62E
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
  iMX8ClkPwrLib|iMX8Pkg/Library/iMX8ClkPwrLib/iMX8ClkPwrLib.inf
  SerialPortLib|iMXPlatformPkg/Library/UartSerialPortLib/UartSerialPortLib.inf
  iMXI2cLib|iMXPlatformPkg/Library/iMXI2cLib/iMXI2cLib.inf

!if $(CONFIG_HEADLESS) != TRUE
  LcdHwLib|iMX8Pkg/Library/iMX8LcdHwLib/iMX8LcdHwLib.inf
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
  #                                        PcdSystemMemoryBase
  # +-------------------+===> (0x40000000) PcdArmLcdDdrFrameBufferBase (Frame buffer)
  # | HDMI framebuffer  |  ^
  # |                   |  |  (0x007E9000) PcdArmLcdDdrFrameBufferSize ~8MB
  # |                   |  v
  # +-------------------+===> (0x407E9000) PcdGlobalDataBaseAddress
  # | Global Data       |  ^
  # |                   |  |  (0x00001000) PcdGlobalDataSize 4KB
  # |                   |  v
  # +-------------------+===> (0x407EA000) 
  # | Operating System  |  ^
  # | Memory            |  |
  # |                   |  |
  # |                   |  |
  # |                   |  |
  # |                   |  v
  # +-------------------+===> (0xB5F00000) PcdFlashNvStorageVariableBase
  # | Nv UEFI Vars      |  |
  # +-------------------+===> (0xB6000000) PcdArmGPUReservedMemoryBase
  # | GPU Reserved      |  ^
  # | Memory            |  |  (0x08000000) PcdArmGPUReservedMemorySize
  # |                   |  v
  # +-------------------+===> (0xBE000000) PcdTrustZonePrivateMemoryBase (OPTEE image base address)
  # | TZ Private Memory |  ^
  # | (OPTEE)           |  |  (0x01E00000) PcdTrustZonePrivateMemorySize 30MB
  # |                   |  v
  # +-------------------+===> (0xBFE00000) PcdTrustZoneSharedMemoryBase (includes mobj bookkeeping page)
  # | TZ Shared Memory  |  ^
  # |                   |  |  (0x00200000) PcdTrustZoneSharedMemorySize 2MB
  # |                   |  v
  # +-------------------|===>

!if $(CONFIG_OPTEE) == TRUE
  gOpteeClientPkgTokenSpaceGuid.PcdTrustZonePrivateMemoryBase|0xBE000000
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
  gOpteeClientPkgTokenSpaceGuid.PcdTrustZoneSharedMemoryBase|0xBFE00000
  gOpteeClientPkgTokenSpaceGuid.PcdTrustZoneSharedMemorySize|0x00200000
!endif
  # System Memory base
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x40000000

  # System memory size (2GB)
!if $(CONFIG_OPTEE) == TRUE
  # OpTEE is loaded at top of memory by Arm-TF. Reduce memory size to avoid collision.
!if $(CONFIG_HEADLESS) == TRUE
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x7E000000
!else
  #The SystemMemorySize must exclude GPU reserved memory and 3*NvStorageVariableSize
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x75F00000
!endif
!else
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x80000000
!endif

# there is 0Gb in the 1Gb bank of memory after top of 32 bit address space
  giMX8TokenSpaceGuid.PcdBank1MemoryBase|0x0000000100000000
  giMX8TokenSpaceGuid.PcdBank1MemorySize|0x0000000000000000

  # GOP driver memory
!if $(CONFIG_HEADLESS) == TRUE
  # Global data area
  giMXPlatformTokenSpaceGuid.PcdGlobalDataBaseAddress|0x40000000
  giMXPlatformTokenSpaceGuid.PcdGlobalDataSize|0x1000

  # Reserved for TPM2 ACPI
  gOpteeClientPkgTokenSpaceGuid.PcdTpm2AcpiBufferBase|0x40001000
  gOpteeClientPkgTokenSpaceGuid.PcdTpm2AcpiBufferSize|0x3000
!else
  gArmPlatformTokenSpaceGuid.PcdArmLcdDdrFrameBufferBase|0x40000000
  gArmPlatformTokenSpaceGuid.PcdArmLcdDdrFrameBufferSize|0x007E9000	# ~8 MB (4 * 1920 * 1080)

  # Global data area
  giMXPlatformTokenSpaceGuid.PcdGlobalDataBaseAddress|0x407E9000
  giMXPlatformTokenSpaceGuid.PcdGlobalDataSize|0x1000

  # Reserved for TPM2 ACPI
  gOpteeClientPkgTokenSpaceGuid.PcdTpm2AcpiBufferBase|0x407EA000
  gOpteeClientPkgTokenSpaceGuid.PcdTpm2AcpiBufferSize|0x3000

  # GPU Reserved Memory, aligned to 1MB boundary
  giMX8TokenSpaceGuid.PcdArmGPUReservedMemoryBase|0xB6000000
  giMX8TokenSpaceGuid.PcdArmGPUReservedMemorySize|0x08000000	# 128 MB

!endif

  #
  # NV Storage PCDs. Use base of 0xB5F00000 ... area below GPU
  #
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageVariableBase|0xB5F00000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageVariableSize|0x00040000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwWorkingBase|0xB5F40000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwWorkingSize|0x00040000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwSpareBase|0xB5F80000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwSpareSize|0x00040000

  # i.MX8M Nano
  gArmPlatformTokenSpaceGuid.PcdCoreCount|4
  gArmPlatformTokenSpaceGuid.PcdClusterCount|1
  gArmPlatformTokenSpaceGuid.PcdPowerButtonEnabled|TRUE

  # Enable Overdrive on Nano when passive cooling or thermal management is in place (CPU 1.2GHz->1.4GHz, GPU 400 MHz->600 MHz)
  giMX8TokenSpaceGuid.PcdPmicOverDriveEnable|FALSE

  gArmTokenSpaceGuid.PcdVFPEnabled|1

  # i.MX8M Nano Display configuration
  #  PcdDisplayEnable      - enable display, default enabled
  #  PcdDisplayInterface   - default display interface, 0=HDMI, 1=MIPI_DSI
  #  PcdDisplayI2CBaseAddr - use I2C2 on iMX8MN
  #  PcdDisplayReadEDID    - TRUE = read EDID from display if available. If not available, set fixed resolution (default 1920x1080@60)
  #                        - FALSE = skip EDID reading (even if available) and set fixed resolution (default 1920x1080@60)
  #  PcdDisplayForceConverterMaxResolution - mipi-hdmi and lvds-hdmi converters have max allowed resolution 1920x1080
  #                                        - TRUE - if EDID data read exceeds the limit, force set fixed resolution for converters only (default 1920x1080@60)
  #                                        - FALSE - Use EDID data read even if exceeds converter limit
  #                                        - if PcdDisplayReadEDID = FALSE, this option has no effect
  giMX8TokenSpaceGuid.PcdDisplayEnable|TRUE
  giMX8TokenSpaceGuid.PcdDisplayInterface|1
  giMX8TokenSpaceGuid.PcdDisplayI2CBaseAddr|0x30A30000
  giMX8TokenSpaceGuid.PcdDisplayReadEDID|TRUE
  giMX8TokenSpaceGuid.PcdDisplayForceConverterMaxResolution|TRUE

  #
  # iMXPlatformPkg
  #

  ## iMXPlatformPackage - Serial Terminal
  giMXPlatformTokenSpaceGuid.PcdSerialRegisterBase|0x30890000

  ## iMXPlatformPackage - Debug UART instance UART2 0x30890000
  giMXPlatformTokenSpaceGuid.PcdKdUartInstance|2

  # uSDHCx | iMX8M Nano EVK Connections
  #-------------------------------------
  # uSDHC1 | N/A
  # uSDHC2 | SD Card slot
  # uSDHC3 | eMMC
  # uSDHC4 | N/A
  #

  giMXPlatformTokenSpaceGuid.PcdSdhc1Base|0x30B40000
  giMXPlatformTokenSpaceGuid.PcdSdhc2Base|0x30B50000
  giMXPlatformTokenSpaceGuid.PcdSdhc3Base|0x30B60000
  giMXPlatformTokenSpaceGuid.PcdSdhc4Base|0x00000000

  giMXPlatformTokenSpaceGuid.PcdSdhc1Enable|FALSE
  giMXPlatformTokenSpaceGuid.PcdSdhc2Enable|TRUE
  giMXPlatformTokenSpaceGuid.PcdSdhc3Enable|TRUE
  giMXPlatformTokenSpaceGuid.PcdSdhc4Enable|FALSE

  giMXPlatformTokenSpaceGuid.PcdSdhc1CardDetectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc1WriteProtectSignal|0xFF01
  giMXPlatformTokenSpaceGuid.PcdSdhc2CardDetectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc2WriteProtectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc2DeviceType|0x1
  giMXPlatformTokenSpaceGuid.PcdSdhc3CardDetectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc3WriteProtectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc3DeviceType|0x8
  giMXPlatformTokenSpaceGuid.PcdSdhc4CardDetectSignal|0xFF00
  giMXPlatformTokenSpaceGuid.PcdSdhc4WriteProtectSignal|0xFF01

  ## SBSA Watchdog Count
!ifndef DISABLE_SBSA_WATCHDOG
  gArmPlatformTokenSpaceGuid.PcdWatchdogCount|2
!endif

  #
  # ARM Generic Interrupt Controller
  #
  gArmTokenSpaceGuid.PcdGicDistributorBase|0x38800000
  gArmTokenSpaceGuid.PcdGicRedistributorsBase|0x38880000

!if TRUE == FALSE
  #
  # Watchdog
  #
  gArmTokenSpaceGuid.PcdGenericWatchdogControlBase|0x30280000
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
  
  #
  # Camera
  #
  # Configuration of camera type connected to CSI1 and CSI2 port
  giMX8TokenSpaceGuid.PcdCsi1CameraOv5640|0x0
  giMX8TokenSpaceGuid.PcdCsi1CameraOv10635|0x0

  #
  # USB
  #
  giMX8TokenSpaceGuid.PcdUsb1EhciBaseAddress|0x32E40100

  #
  # FlexSPI
  #
  giMX8TokenSpaceGuid.PcdFlexSpiBaseAddress|0x30bb0000

  # Boot options
  giMX8TokenSpaceGuid.PcdBootDevice1Path|L"VenHw(AAFB8DAA-7340-43AC-8D49-0CCE14812489,02000000)/SD(0x0)"
  giMX8TokenSpaceGuid.PcdBootDevice1Description|L"Boot from SD Card"
  giMX8TokenSpaceGuid.PcdBootDevice2Path|L"VenHw(AAFB8DAA-7340-43AC-8D49-0CCE14812489,03000000)/eMMC(0x0)"
  giMX8TokenSpaceGuid.PcdBootDevice2Description|L"Boot from eMMC"

[PcdsPatchableInModule]
  # Use system default resolution
  gEfiMdeModulePkgTokenSpaceGuid.PcdVideoHorizontalResolution|0
  gEfiMdeModulePkgTokenSpaceGuid.PcdVideoVerticalResolution|0

