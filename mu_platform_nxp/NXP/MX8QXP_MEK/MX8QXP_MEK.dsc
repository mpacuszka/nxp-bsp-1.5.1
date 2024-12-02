#
# NXP iMX8QXP CPU board with 3GB DRAM description
#
#  Copyright (c) 2018, Microsoft Corporation. All rights reserved.
#  Copyright (c) 2013-2018, ARM Limited. All rights reserved.
#  Copyright 2019-2020, 2022-2024 NXP
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
  DEFINE BOARD_NAME       = MX8QXP_MEK
  DEFINE IMX_FAMILY       = IMX8QXP
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
  iMX8xScfwLib|iMX8Pkg/Library/iMX8xScfwLib/iMX8xScfwLib.inf
  SerialPortLib|iMXPlatformPkg/Library/LPUartSerialPortLib/LPUartSerialPortLib.inf
  iMXLpi2cLib|iMXPlatformPkg/Library/iMXLpi2cLib/iMXLpi2cLib.inf

!if $(CONFIG_HEADLESS) != TRUE
  LcdHwLib|iMX8Pkg/Library/iMX8LcdHwLib/iMXDpuHwLib.inf
!endif

[Components.common]
  #
  # ACPI Support
  #
  MdeModulePkg/Universal/Acpi/AcpiTableDxe/AcpiTableDxe.inf
  MdeModulePkg/Universal/Acpi/AcpiPlatformDxe/AcpiPlatformDxe.inf
  $(BOARD_DIR)/AcpiTables/AcpiTables.inf

!if $(CONFIG_USB) == TRUE
  MdeModulePkg/Bus/Pci/XhciDxe/XhciDxe.inf
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

  gEfiMdeModulePkgTokenSpaceGuid.PcdTurnOffUsbLegacySupport|TRUE

[PcdsFixedAtBuild.common]
  #                                        PcdSystemMemoryBase
  # +-------------------+===> (0x80280000) PcdArmLcdDdrFrameBufferBase (Frame buffer)
  # | HDMI framebuffer  |  ^
  # |                   |  |  (0x00900000) PcdArmLcdDdrFrameBufferSize ~8MB
  # |                   |  v
  # +-------------------+===> (0x80B80000) PcdGlobalDataBaseAddress
  # | Global Data       |  ^
  # |                   |  |  (0x00001000) PcdGlobalDataSize 4KB
  # |                   |  v
  # +-------------------+===> (0x80B81000) PcdTpm2AcpiBufferBase
  # | TPM               |  ^
  # |                   |  |  (0x00003000) PcdTpm2AcpiBufferSize
  # |                   |  v
  # +-------------------+===> (0x80B84000) RAM1 region base
  # | Operating System  |  ^
  # | Memory            |  |
  # |                   |  |
  # |                   |  v
  # +-------------------+===> (0xE7B00000) PcdIsiDma0ReservedMemoryBase
  # | Isi Dma 0 FB      |  ^
  # | Memory            |  |  (0x00600000) PcdIsiDmaReservedMemorySize
  # |                   |  |
  # |                   |  v
  # +-------------------+===> (0xE8100000) PcdArmGPUReservedMemoryBase
  # | GPU Reserved      |  ^
  # | Memory            |  |  (0x08000000) PcdArmGPUReservedMemorySize
  # |                   |  v
  # +-------------------+===> (0xF0100000) PcdVPUFWBase
  # | VPU Firmwares     |  ^
  # |                   |  |  (0x02200000) PcdVPUFWSize
  # |                   |  |
  # |                   |  v
  # +-------------------+===> (0xF2300000) RAM2 region base
  # | Operating System  |  ^
  # | Memory            |  |  (0x0AD00000) RAM2 region size
  # |                   |  |
  # |                   |  v
  # +-------------------+===> (0xFD000000) PcdVPURPCBase
  # | VPU RPC           |  ^
  # | Memory            |  |  (0x01000000) PcdVPURPCSize
  # |                   |  |
  # |                   |  v
  # +-------------------+===> (0xFE000000) PcdTrustZonePrivateMemoryBase (OPTEE image base address)
  # | TZ Private Memory |  ^
  # | (OPTEE)           |  |  (0x01E00000) PcdTrustZonePrivateMemorySize 30MB
  # |                   |  v
  # +-------------------+===> (0xFFE00000) PcdTrustZoneSharedMemoryBase (includes mobj bookkeeping page)
  # | TZ Shared Memory  |  ^
  # |                   |  |  (0x00200000) PcdTrustZoneSharedMemorySize 2MB
  # |                   |  v
  # +-------------------|===>

!if $(CONFIG_OPTEE) == TRUE
  gOpteeClientPkgTokenSpaceGuid.PcdTrustZonePrivateMemoryBase|0xFE000000
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
  gOpteeClientPkgTokenSpaceGuid.PcdTrustZoneSharedMemoryBase|0xFFE00000
  gOpteeClientPkgTokenSpaceGuid.PcdTrustZoneSharedMemorySize|0x00200000
!endif

  # FirmwareRevision 0.1
  gEfiMdeModulePkgTokenSpaceGuid.PcdFirmwareRevision|0x00000001

  # System Memory base
!if $(CONFIG_HEADLESS) == TRUE
    !if $(CONFIG_OPTEE) == TRUE
      # OpTEE is loaded at top of memory by Arm-TF.
      gArmTokenSpaceGuid.PcdSystemMemoryBase|0x80280000
      gArmTokenSpaceGuid.PcdSystemMemorySize|0x7DD80000
    !else
      gArmTokenSpaceGuid.PcdSystemMemoryBase|0x80280000
      gArmTokenSpaceGuid.PcdSystemMemorySize|0x7F780000
    !endif
!else
    !if $(CONFIG_OPTEE) == TRUE
      # OpTEE is loaded at top of memory by Arm-TF.
      gArmTokenSpaceGuid.PcdSystemMemoryBase|0x80B80000
      gArmTokenSpaceGuid.PcdSystemMemorySize|0x66F80000
      gArmTokenSpaceGuid.PcdSystemMemory2Base|0xF2300000
      gArmTokenSpaceGuid.PcdSystemMemory2Size|0x0AD00000
    !else
      gArmTokenSpaceGuid.PcdSystemMemoryBase|0x80B80000
      gArmTokenSpaceGuid.PcdSystemMemorySize|0x66F80000
      gArmTokenSpaceGuid.PcdSystemMemory2Base|0xF2300000
      gArmTokenSpaceGuid.PcdSystemMemory2Size|0x0AD00000
      gArmTokenSpaceGuid.PcdSystemMemory3Base|0xFE000000
      gArmTokenSpaceGuid.PcdSystemMemory3Size|0x02000000
    !endif
!endif

# there is 0Gb in the 1Gb bank of memory after top of 32 bit address space
  giMX8TokenSpaceGuid.PcdBank1MemoryBase|0x0000000880000000
  giMX8TokenSpaceGuid.PcdBank1MemorySize|0x0000000040000000

  # GOP driver memory
!if $(CONFIG_HEADLESS) == TRUE
  # Global data area
  giMXPlatformTokenSpaceGuid.PcdGlobalDataBaseAddress|0x80280000
  giMXPlatformTokenSpaceGuid.PcdGlobalDataSize|0x1000

  # Reserved for TPM2 ACPI
  gOpteeClientPkgTokenSpaceGuid.PcdTpm2AcpiBufferBase|0x80281000
  gOpteeClientPkgTokenSpaceGuid.PcdTpm2AcpiBufferSize|0x3000
!else
  # Frame buffer
  gArmPlatformTokenSpaceGuid.PcdArmLcdDdrFrameBufferBase|0x80280000
  gArmPlatformTokenSpaceGuid.PcdArmLcdDdrFrameBufferSize|0x00900000

  # VPU FW region
  gArmPlatformTokenSpaceGuid.PcdVPUFWBase|0xF0100000
  gArmPlatformTokenSpaceGuid.PcdVPUFWSize|0x02200000

  # VPU RPC region
  gArmPlatformTokenSpaceGuid.PcdVPURPCBase|0xFD000000
  gArmPlatformTokenSpaceGuid.PcdVPURPCSize|0x01000000

  # Global data area
  giMXPlatformTokenSpaceGuid.PcdGlobalDataBaseAddress|0x80B80000
  giMXPlatformTokenSpaceGuid.PcdGlobalDataSize|0x1000

  # Reserved for TPM2 ACPI
  gOpteeClientPkgTokenSpaceGuid.PcdTpm2AcpiBufferBase|0x80B81000
  gOpteeClientPkgTokenSpaceGuid.PcdTpm2AcpiBufferSize|0x3000
  
  # ISI DMA 0 Buffer
  giMX8TokenSpaceGuid.PcdIsiDma0ReservedMemoryBase|0xE7B00000
  giMX8TokenSpaceGuid.PcdIsiDmaReservedMemorySize|0x00600000	# 3 buffers so 3x 2 MB; 1280*720*2
  
  # GPU Reserved Memory, aligned to 1MB boundary
  giMX8TokenSpaceGuid.PcdArmGPUReservedMemoryBase|0xE8100000
  giMX8TokenSpaceGuid.PcdArmGPUReservedMemorySize|0x08000000	# 128 MB
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

  # i.MX8QXP
  gArmPlatformTokenSpaceGuid.PcdCoreCount|4
  gArmPlatformTokenSpaceGuid.PcdClusterCount|1

  gArmTokenSpaceGuid.PcdVFPEnabled|1

  # i.MX8QXP Display configuration
  #  PcdDisplayEnable      - enable display, default enabled
  #  PcdDisplayInterface   - default display interface: 1=MIPI-DSI0, 2=LVDS0, 3=LVDS1, 6=MIPI-DSI1
  #  PcdDisplayI2CBaseAddr - I2C related to IMX-LVDS-HDMI or IMX-MIPI-HDMI converter:
  #                          DSI0_I2C0 - 0x56226000 (for LVDS0/DSI0), DSI1_I2C0 - 0x56246000 (for LVDS1/DSI1)
  #  PcdDisplayReadEDID    - applicable for lvds-hdmi or mipi_dsi-hdmi converter
  #                        - TRUE = read EDID from display if available. If not available, set fixed resolution (default 1920x1080@60)
  #                        - FALSE = skip EDID reading (even if available) and set fixed resolution (default 1920x1080@60)
  #  PcdDisplayForceConverterMaxResolution - mipi-hdmi and lvds-hdmi converters have max allowed resolution 1920x1080@60
  #                                        - TRUE - if EDID data read exceeds the limit, force set fixed resolution for converters only
  #                                        - FALSE - Use EDID data read even if exceeds the limit
  #                                        - if PcdDisplayReadEDID = FALSE, this option has no effect

  giMX8TokenSpaceGuid.PcdDisplayEnable|TRUE
  giMX8TokenSpaceGuid.PcdDisplayInterface|2
  giMX8TokenSpaceGuid.PcdDisplayI2CBaseAddr|0x56226000
  giMX8TokenSpaceGuid.PcdDisplayReadEDID|TRUE
  giMX8TokenSpaceGuid.PcdDisplayForceConverterMaxResolution|TRUE

  #
  # iMXPlatformPkg
  #

  ## iMXPlatformPackage - Serial Terminal
  giMXPlatformTokenSpaceGuid.PcdSerialRegisterBase|0x5a060000

  ## iMXPlatformPackage - Debug UART instance LPUART1 0x5a060000
  giMXPlatformTokenSpaceGuid.PcdKdUartInstance|0

  # PcdMuSCFWRegisterBase - Messaging Unit to SCFW base address
  giMXPlatformTokenSpaceGuid.PcdMuSCFWRegisterBase|0x5d1c0000

  # uSDHCx | iMX8QXP MEK Connections
  #-------------------------------------
  # uSDHC1 | eMMC
  # uSDHC2 | SD Card slot
  # uSDHC3 | N/A
  # uSDHC4 | N/A
  #
  giMXPlatformTokenSpaceGuid.PcdSdhc1Base|0x5B010000
  giMXPlatformTokenSpaceGuid.PcdSdhc2Base|0x5B020000
  giMXPlatformTokenSpaceGuid.PcdSdhc3Base|0x5B030000
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

  #
  # PCIE
  #
  # PCIe 1 Host config space is fixed at 0x5F010000. The memory range from
  # 0x70000000 - 0x7FFFFFFF is assigned for PCIe 1. The memory layout defined
  # by the boot loader is as below
  #
  # PCIe Device Config0 Space : 0x7FF80000 - 0x7FF8FFFF
  # PCIe Device Config1 Space : PCIe switch support is disabled
  # PCIe Device Memory Space : 0x70000000 - 0x7FF7FFFF
  giMX8TokenSpaceGuid.PcdPcie1Enable|TRUE
  giMX8TokenSpaceGuid.PcdPcie1ControlBaseAddress|0x5F010000
  giMX8TokenSpaceGuid.PcdPcie1DeviceMemoryBase|0x70000000
  giMX8TokenSpaceGuid.PcdPcie1DeviceMemorySize|0x0FF00000
  giMX8TokenSpaceGuid.PcdPcie1DeviceConfig0Base|0x7FF80000
  giMX8TokenSpaceGuid.PcdPcie1DeviceConfig0Size|0x00010000
  giMX8TokenSpaceGuid.PcdPcie1SwitchSupport|FALSE
  giMX8TokenSpaceGuid.PcdPcie1InternalRefClock|FALSE
  giMX8TokenSpaceGuid.PcdPcie1MaxLinkSpeed|3
  giMX8TokenSpaceGuid.PcdPcie1ResetGpio|TRUE
  giMX8TokenSpaceGuid.PcdPcie1ResetGpioBankNumber|4
  giMX8TokenSpaceGuid.PcdPcie1ResetGpioIoNumber|0

  ## SBSA Watchdog Count
!ifndef DISABLE_SBSA_WATCHDOG
  gArmPlatformTokenSpaceGuid.PcdWatchdogCount|2
!endif

  #
  # ARM Generic Interrupt Controller
  #
  gArmTokenSpaceGuid.PcdGicDistributorBase|0x51a00000
  gArmTokenSpaceGuid.PcdGicRedistributorsBase|0x51b00000

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
  
  #
  # Camera
  #
  # Configuration of camera type connected to CSI1 port
  giMX8TokenSpaceGuid.PcdCsi1CameraOv5640|0x0
  giMX8TokenSpaceGuid.PcdCsi1CameraOv10635|0x0
  
  #
  # Lpuart
  #
  # Uncomment to enable LPUART 1
  # giMX8TokenSpaceGuid.PcdLpuart1Enable|0x1

  #
  # USB
  #
  giMX8TokenSpaceGuid.PcdUsb1XhciBaseAddress|0x5B130000

  # Boot options
  giMX8TokenSpaceGuid.PcdBootDevice1Path|L"VenHw(AAFB8DAA-7340-43AC-8D49-0CCE14812489,02000000)/SD(0x0)"
  giMX8TokenSpaceGuid.PcdBootDevice1Description|L"Boot from SD Card"
  giMX8TokenSpaceGuid.PcdBootDevice2Path|L"VenHw(AAFB8DAA-7340-43AC-8D49-0CCE14812489,01000000)/eMMC(0x0)"
  giMX8TokenSpaceGuid.PcdBootDevice2Description|L"Boot from eMMC"

[PcdsPatchableInModule]
  # Use system default resolution
  gEfiMdeModulePkgTokenSpaceGuid.PcdVideoHorizontalResolution|0
  gEfiMdeModulePkgTokenSpaceGuid.PcdVideoVerticalResolution|0

